"""User-confirmed task console. Startup is nonactuating, with live robot feedback."""
import argparse
import copy
from datetime import datetime
import json
import math
import os
from pathlib import Path
from queue import SimpleQueue, Empty
import re
import signal
import shutil
import subprocess
import threading
import time
import traceback

import numpy as np
from PyQt5 import QtCore, QtWidgets
import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.parameter import Parameter
from rcl_interfaces.srv import SetParametersAtomically
from diagnostic_msgs.msg import DiagnosticArray
from sensor_msgs.msg import JointState, PointCloud2
from std_srvs.srv import Trigger
from std_msgs.msg import String
from control_msgs.action import FollowJointTrajectory
from g1_unitree_control.srv import ResumeTaskHold
from moveit_msgs.msg import DisplayTrajectory, DisplayRobotState, RobotTrajectory
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from rosidl_runtime_py.convert import message_to_ordereddict

from task_session import Session, ARMS, ARM_NAMES, FeedbackPending, RobotMoving
from task_future import wait_for_result
from task_console_handoff import (acquire_console_lock, load_handoff, process_identity,
                                 supported_gateway, supported_controller)
from task_motion_planner import TaskPlanner, robot_state, pose
from task_straight_policy import MAX_INDEX_TILT_DEG
from task_planning_config import DEFAULT_CONFIG,load_parameters,configure_planner,parameters_from_values,controller_bounded_parameters
from task_preview_markers import scoped_target_markers, with_live_tcp, live_tcp_markers, planning_tcp_offsets
from workbench_model import OfflineModel
from vision_rgbd import import_record, validate_capture_result, measured_scene_geometry
from vision_task_geometry import build_task_preview
from vision_task_preview import load_cloud
from estimate_index_tcp import generate
from task_preview_markers import target_markers
from task_hand_control import HandControl, model_joints, POSTURES, model_feedback, ORDER
from task_tcp_a_controls import ExperimentControls, TOPIC as TCP_A_CONTROL_TOPIC
from task_saved_test import load_archive, select_range, prepare_range, range_fingerprint, verify_test_plan, needs_contact_confirmation, live_test_plan
from task_plan_archive import write_json, digest
from task_dring_sequence import is_center_check,CENTER_SCHEMA
from check_saved_task_stage import compare_start
import task_calibration

ROOT = Path(__file__).resolve().parents[1]
LEFT_READY = ROOT/'logs/taught_poses/left_initial_candidate_20260908_134946_077146571/task_pose.json'


def demonstration_control(node, plan):
    workflow = plan['demo'].get('workflow', 'dring_demo')
    if workflow == 'dring_demo':return node.demo
    if workflow == 'landing_gear':return node.landing_gear
    raise RuntimeError('Unknown demonstration workflow')


class TaskNode(Node):
    def __init__(self, args):
        super().__init__('g1_real_task_console')
        self.args = args
        self.lock = threading.RLock()
        self.session = Session()
        self.calibration=task_calibration.load()
        from task_config_monitor import PlanningConfigMonitor
        self.planning_monitor=PlanningConfigMonitor(getattr(args,'planning_config',DEFAULT_CONFIG))
        self.planning_config_revision=self.planning_monitor.revision
        self.planning_config_pending=False
        self.worker = None
        self.closing = threading.Event()
        self.status = 'Waiting for live LowState. No hardware command has been sent.'
        self.validation_progress = None
        self.gateway = {}; self.gateway_received = float('-inf')
        self.stop_expected_epoch = 0
        self.recovery_pending = None
        self.shutdown_requested = False
        self.shutdown_process = None
        self.shutdown_result = None
        self.execution_checkpoint = None
        self.feedback_callbacks = MutuallyExclusiveCallbackGroup()
        self.diagnostic_callbacks = MutuallyExclusiveCallbackGroup()
        self.handoff = load_handoff(args.attach_handoff) if getattr(args,'attach_handoff',None) else None
        self.attachment_complete=False
        if self.handoff and (not args.hardware or args.disconnected):
            raise RuntimeError('Console attachment requires the existing live hardware session')
        self.run_dir = ROOT/'data/task_sessions'/datetime.now().strftime('%Y%m%d_%H%M%S_%f')
        self.run_dir.mkdir(parents=True)
        self.tcp = self.run_dir/'index_tcp.json'
        self.tcp.write_text(json.dumps(generate(), indent=2)+'\n')
        self.model = OfflineModel(args.robot_urdf)
        self.prepared = {'left':json.loads(LEFT_READY.read_text()), 'right':None}
        if args.ready_poses:
            supplied = json.loads(Path(args.ready_poses).read_text())
            for side in ARMS:
                if supplied.get(side): self.prepared[side] = supplied[side]
        if self.handoff:
            for side,target in self.handoff.get('ready_poses',{}).items():
                self.prepared[side] = target
        if self.prepared['right'] is None and self.prepared['left'] is not None:
            from task_kinematics import mirror_left_ready
            self.prepared['right'] = mirror_left_ready(self.model, self.prepared['left'])
        (self.run_dir/'ready_poses.json').write_text(json.dumps(self.prepared,indent=2)+'\n')
        self.hand_control=bool(getattr(args,'hand_control',False))
        self.hand=HandControl(args.network,enabled=args.hardware and self.hand_control,
                              peer_address=args.ssh.rsplit('@',1)[-1] if self.hand_control else None)
        self.hand_states={}
        self.hand_pub=self.create_publisher(JointState,'/joint_states',10)
        profile=getattr(args,'dring_profile','steep_straight')
        planner_class=TaskPlanner;options={}
        if profile=='steep_straight':
            from task_scoped_planner import ScopedTaskPlanner
            planner_class=ScopedTaskPlanner
            options=dict(robot_urdf=args.robot_urdf,check_plan=self.check,max_index_tilt=MAX_INDEX_TILT_DEG,
                         search_seconds=30.,search_attempts=220)
        self.planner = planner_class(self, self.model, self.call, self.progress,
                                  waist_assist=bool(getattr(args,'waist_assist',False)),
                                  articulated_hands=self.hand_control,**options)
        self.trigger = {name:self.create_client(Trigger, '/g1/arm_gateway/'+name)
                        for name in ('arm','stop_task','reset')}
        self.resume_hold=self.create_client(ResumeTaskHold,'/g1/arm_gateway/hold_after_stop')
        self.reset_task=self.create_client(ResumeTaskHold,'/g1/arm_gateway/reset_task')
        self.waist_admittance_stop=self.create_client(ResumeTaskHold,'/g1/arm_gateway/stop_task_with_waist')
        self.waist_parameters=self.create_client(
            SetParametersAtomically,'/g1_arm_sdk_hardware_gateway/set_parameters_atomically')
        self.actions = {side:ActionClient(self, FollowJointTrajectory,
                        '/'+side+'_arm_controller/follow_joint_trajectory') for side in ARMS}
        self.waist_action = ActionClient(self, FollowJointTrajectory,
                                        '/g1/arm_gateway/follow_waist_trajectory')
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.cloud_pub = self.create_publisher(PointCloud2,'/g1/vision_rgbd_surface',qos)
        self.marker_pub = self.create_publisher(MarkerArray,'/g1/vision_task_preview',qos)
        self.tcp_marker_offsets = task_calibration.marker_offsets(self.model,self.calibration)
        self.marker_pub.publish(live_tcp_markers(self.tcp_marker_offsets))
        self.display_pub = self.create_publisher(DisplayTrajectory,'/g1/task/validated_path',qos)
        self.demo_preview_pub = self.create_publisher(DisplayRobotState,'/g1/task/demo_preview_state',qos)
        self.demo_preview_pub.publish(DisplayRobotState(hide=True))
        from task_trace_picker import TracePicker
        self.trace_picker = TracePicker(self)
        self.status_pub = self.create_publisher(String,'/g1/task/console_status',qos)
        self.tcp_a_controls = ExperimentControls(self)
        self.create_subscription(String,TCP_A_CONTROL_TOPIC,self.tcp_a_controls.receive,10)
        self.create_subscription(DiagnosticArray,'/g1/arm_gateway/diagnostics',self.diagnostics,1,
                                 callback_group=self.diagnostic_callbacks)
        from task_screen_control import ScreenControl
        self.screen = ScreenControl(self)
        from task_rail_control import RailControl
        self.rail = RailControl(self)
        self.rail.restore_latest()
        from task_dring_demo_control import DringDemoControl
        self.demo = DringDemoControl(self)
        from task_landing_gear_control import LandingGearControl
        self.landing_gear = LandingGearControl(self)

    def joints(self, msg):
        names = set(msg.name)
        malformed = len(msg.name) != len(msg.position) or len(names) != len(msg.name)
        # cache_hand_state publishes finger-only visualization on this shared topic.
        # It is not a new arm sample and must neither refresh nor invalidate recording.
        hand_names = {f'{side}_{joint}_joint' for side in ARMS for joint in ORDER}
        if not malformed and names and names <= hand_names:
            return
        missing = [name for name in ARM_NAMES if name not in names]
        if malformed or missing:
            with self.lock:
                if hasattr(self,'rail'):
                    reason = ('关节消息长度不匹配或名称重复' if malformed else
                              '手臂反馈缺少关节：'+', '.join(missing))
                    self.rail.reject_feedback(reason)
            return
        received_at=time.monotonic()
        source_now_ns=self.get_clock().now().nanoseconds
        stamp_ns=msg.header.stamp.sec*1000000000+msg.header.stamp.nanosec
        with self.lock:
            acquired_at=time.monotonic()
            previous=getattr(self,'feedback_transport',{})
            self.feedback_transport=dict(received_at=received_at,
                callback_gap_ms=(received_at-previous.get('received_at',received_at))*1000,
                lock_wait_ms=(acquired_at-received_at)*1000,
                source_age_ms=(source_now_ns-stamp_ns)/1e6)
            try:
                q=dict(zip(msg.name,msg.position))
                if self.hand_control:
                    for side,record in self.hand_states.items():
                        # Cached model estimate, not new hand telemetry. Every
                        # plan/execute stage independently requests fresh DDS state.
                        q.update(model_joints(side,record['state']['feedback_q']))
                accepted=self.session.update(q,received_at=received_at,
                    source_stamp_ns=stamp_ns,source_now_ns=source_now_ns)
                if not accepted:
                    if hasattr(self,'rail'):self.rail.reject_feedback(self.session.feedback_rejection)
                    return
                if hasattr(self,'rail'):
                    self.rail.capture_feedback(self.session.current,self.session.received,self.session.source_stamp_ns)
                if self.session.phase=='HOLD_AWAITING_FEEDBACK':
                    try:self.session.measured(stationary=True)
                    except (FeedbackPending,RobotMoving):pass
                    else:
                        self.session.phase='HOLD_FOR_RECOVERY'
                        self.status='Recovery hold confirmed; fresh stationary feedback restored.'
                if self.session.phase=='WAIT_STATE':
                    self.session.phase='LIVE_READY'
                    self.status='Live robot feedback; session startup pose not captured.'
            except ValueError as error:
                if hasattr(self,'rail'):self.rail.reject_feedback(str(error))
                return

    def read_hands(self, ticket=None):
        if not self.hand_control or self.args.disconnected:
            raise RuntimeError('Hand DDS is not enabled in this session')
        result={}
        for side in ARMS:
            pending=f'Reading {side} hand position feedback (DDS)...'
            with self.lock:self.status=pending
            try:state=self.hand.read(side,lambda:self.check(ticket))
            except TimeoutError as error:
                raise TimeoutError(f'{side.capitalize()} hand state unavailable: DDS timeout (3104). '
                                   'Check onboard connection; no hand command was sent by this read.') from error
            result[side]=state
            self.cache_hand_state(side,state)
        if ticket is None:
            with self.lock:
                if self.status==pending:
                    self.status='Hand DDS: '+', '.join(f'{s} online, max {max(v["temperature"]):.0f} C' for s,v in result.items())
                    if self.session.stopped:self.status+='; controller remains '+self.gateway.get('phase','stopped')
        return result

    def cache_hand_state(self,side,state):
        msg=JointState();msg.header.stamp=self.get_clock().now().to_msg()
        values=model_joints(side,state['feedback_q'])
        msg.name=list(values);msg.position=list(values.values())
        self.hand_pub.publish(msg)
        with self.lock:
            self.hand_states[side]={'state':state,'received':time.monotonic()}
            # Preserve the arm feedback timestamp; hand RPC is not LowState.
            self.session.current={**self.session.current,**values}

    def finish_attachment(self):
        deadline=time.monotonic()+8
        while time.monotonic()<deadline:
            self.check()
            with self.lock:
                if time.monotonic()-self.gateway_received<1.5:
                    if self.handoff.get('attachment_mode')=='panel':
                        if process_identity(self.handoff['gateway_identity']['pid']) != self.handoff['gateway_identity']:
                            raise RuntimeError('Original controller changed during panel attachment')
                        stopped=self.gateway.get('task_stop_active')=='true'
                        epoch=int(self.gateway['task_stop_epoch'])
                        self.restore_attachment(stopped,self.gateway.get('phase','WAIT_STATE'),epoch,
                            'Panel connected. Controller state: '+self.gateway.get('phase','unknown')+
                            '. No ARM, reset, resume or motion was sent.')
                        return
                    try:
                        self.session.measured(stationary=True)
                    except RuntimeError:
                        pass
                    else:
                        if process_identity(self.handoff['gateway_identity']['pid']) != self.handoff['gateway_identity']:
                            raise RuntimeError('Original controller changed during attachment')
                        supported_gateway(self.gateway)
                        if int(self.gateway['task_stop_epoch']) != self.handoff['stop_epoch']:
                            raise RuntimeError('A newer stop interrupted the console handoff; no resume was sent')
                        stopped=self.handoff.get('attachment_mode')=='stopped'
                        if stopped:
                            if self.gateway['task_stop_active']!='true':
                                raise RuntimeError('Expected stopped controller; no command was sent')
                        elif self.gateway['phase']!='ARMED_HOLD' or self.gateway['task_stop_active']!='false':
                            raise RuntimeError('Controller is not holding; no automatic resume was sent')
                        self.restore_attachment(stopped,self.gateway['phase'] if stopped else 'HOLD_FOR_REPLAN',
                            self.handoff['stop_epoch'],
                            'Console reattached in stopped mode. Startup/ready poses preserved; no automatic resume.'
                            if stopped else 'Console reattached to the existing hold. Startup/ready poses preserved; replan before execution.')
                        return
            time.sleep(.05)
        raise RuntimeError('No fresh stationary feedback for console attachment; no command was sent')

    def restore_attachment(self,stopped,phase,epoch,status):
        record=self.handoff['session_home']
        self.session.home=dict(record['joints']) if record is not None else None
        self.session.invalidate()
        self.session.stopped=stopped
        self.stop_expected_epoch=epoch
        self.session.phase=phase
        if record is not None:
            (self.run_dir/'session_home.json').write_text(json.dumps(record,indent=2)+'\n')
        (self.run_dir/'ready_poses.json').write_text(json.dumps(self.prepared,indent=2)+'\n')
        (self.run_dir/'handoff_source.json').write_text(json.dumps(self.handoff,indent=2)+'\n')
        self.display_pub.publish(DisplayTrajectory())
        self.status=status
        if record is None:self.status+=' No startup saved; capture a return point before planning.'
        self.attachment_complete=True

    def diagnostics(self, msg):
        with self.lock:
            for item in msg.status:
                previous_mode = tuple(self.gateway.get(k) for k in ('waist_control_mode', 'waist_control_revision'))
                self.gateway = {v.key:v.value for v in item.values}
                self.gateway_received = time.monotonic()
                current_mode = tuple(self.gateway.get(k) for k in ('waist_control_mode', 'waist_control_revision'))
                if previous_mode != current_mode and previous_mode[0] is not None and hasattr(self, 'screen'):
                    self.screen.invalidate(alignment=False, clear_checks=True)
                if (self.gateway.get('phase') in ('LATCHED_SILENT', 'CONFIG_BLOCKED') and
                        hasattr(self, 'screen') and self.screen.aligned):
                    self.screen.invalidate()
                pending=self.recovery_pending
                if pending and pending['generation']==self.session.generation:
                    epoch=int(self.gateway.get('task_stop_epoch','-1'))
                    if epoch!=pending['epoch']:
                        self.recovery_pending=None
                    elif self.gateway.get('phase')=='ARMED_HOLD' and self.gateway.get('task_stop_active')=='false':
                        self.session.recovered_hold(acknowledged=True)
                        self.recovery_pending=None
                        self.status='Controller hold confirmed; waiting for fresh stationary feedback.'
                epoch=int(self.gateway.get('task_stop_epoch','0'))
                stopped=self.gateway.get('task_stop_active')=='true'
                if (epoch>self.stop_expected_epoch or
                        (stopped and not self.session.stopped and epoch>=self.stop_expected_epoch)):
                    if hasattr(self, 'screen'):
                        self.screen.invalidate()
                    if hasattr(self, 'rail'):
                        self.rail.started()
                    if hasattr(self, 'demo'):self.demo.cancel_trace('STOP 已变化，停止录制；旧轨迹保留')
                    if hasattr(self, 'landing_gear'):self.landing_gear.cancel_trace('STOP 已变化，停止录制；旧轨迹保留')
                    self.session.stop()
                    self.stop_expected_epoch=epoch
                    self.recovery_pending=None
                    self.display_pub.publish(DisplayTrajectory())
                    self.status='Controller STOP changed; previous work cancelled. A new preview may be planned.'
                if (self.session.phase=='STOP_REQUESTED' and stopped and epoch>=self.stop_expected_epoch):
                    self.session.phase=self.gateway.get('phase','STOP_ACKNOWLEDGED')

    def check(self, ticket=None):
        if self.closing.is_set(): raise RuntimeError('Application closing')
        if ticket:
            with self.lock:
                self.session.check(ticket)
                if (ticket.kind == 'screen' and self.session.phase == 'EXECUTING' and
                        hasattr(self, 'screen') and self.session.plan is not None):
                    self.screen.execution_guard(self.session.plan, running=True)
                if (ticket.kind == 'rail' and self.session.phase == 'EXECUTING' and
                        hasattr(self, 'rail') and self.session.plan is not None):
                    self.rail.execution_guard(self.session.plan, running=True)
                if ticket.kind=='dring_demo' and self.session.phase=='EXECUTING' and self.session.plan is not None:
                    demonstration_control(self,self.session.plan).execution_guard(self.session.plan,running=True)

    def wait(self, future, ticket=None, timeout=15):
        return wait_for_result(future, lambda:self.check(ticket), timeout)

    def progress(self, ticket, message):
        with self.lock:
            self.session.check(ticket)
            self.status = message
            match=re.search(r'validating trajectory\s+(\d+)/(\d+)',message)
            if match:
                completed,total=map(int,match.groups())
                self.validation_progress=dict(generation=ticket.generation,
                                              completed=completed,total=total)

    def finish_plan(self, ticket, plan):
        self.check(ticket)
        with self.lock:
            self.session.ready(ticket,plan)

    def call(self, client, request, ticket=None, timeout=15):
        self.check(ticket)
        deadline=time.monotonic()+timeout
        if not client.wait_for_service(timeout_sec=min(2.,timeout)):
            raise RuntimeError('Service unavailable: '+client.srv_name)
        remaining=deadline-time.monotonic()
        if remaining<=0:raise TimeoutError('ROS service discovery exhausted the request deadline')
        return self.wait(client.call_async(request),ticket,remaining)

    def wait_stationary(self, ticket=None, *, expected=None, timeout=5., guard=None,
                        context='segment boundary', settling=False, on_ready=None, tolerance_rad=.025):
        deadline=time.monotonic()+timeout
        while True:
            self.check(ticket)
            try:
                with self.lock:
                    if guard:guard()
                    q=self.session.measured()
                    watched=ARM_NAMES+(('waist_yaw_joint',) if expected and 'waist_yaw_joint' in expected else ())
                    if expected:
                        missing=[n for n in watched if n not in q]
                        if missing:raise FeedbackPending('Missing measured joints: '+', '.join(missing))
                        joint=max(watched,key=lambda n:abs(q[n]-expected[n]))
                        error=abs(q[joint]-expected[joint])
                        if error>tolerance_rad:
                            detail=(f'{joint}: error={np.degrees(error):.3f} deg, '
                                    f'limit={np.degrees(tolerance_rad):.3f} deg')
                            if not settling:raise RuntimeError('Segment start differs from actual state; replan; '+detail)
                            raise FeedbackPending('Waiting for measured segment endpoint; '+detail)
                    q=self.session.measured(stationary=True)
                    return on_ready(q) if on_ready else q
            except (FeedbackPending,RobotMoving) as error:
                with self.lock:self.status=context+': waiting for fresh stationary feedback ('+str(error)+')'
                if time.monotonic()>=deadline:
                    raise TimeoutError(context+': feedback/settling wait expired: '+str(error)) from error
                time.sleep(.02)

    def launch_work(self, function):
        with self.lock:
            if self.worker and self.worker.is_alive():
                raise RuntimeError('Previous worker is finishing cancellation; retry shortly')
            def run():
                try: function()
                except Exception as e:
                    with self.lock: self.status = str(e)
                    self.get_logger().error(str(e))
            self.worker = threading.Thread(target=run, daemon=True)
            self.worker.start()

    @staticmethod
    def empty_display():
        return DisplayTrajectory()

    def home(self):
        with self.lock:
            if self.session.home is not None:
                self.status='Return point already captured; existing point preserved.'
                return
            if (self.session.phase in ('PLANNING','EXECUTION_PREFLIGHT','EXECUTING') or
                    (self.worker is not None and self.worker.is_alive())):
                raise RuntimeError('Capture requires an idle session')
            if (time.monotonic()-self.gateway_received > 1.5 or
                    self.gateway.get('mode_machine') != '6' or self.gateway.get('mode_pr') != '0'):
                raise RuntimeError('Need fresh mode6/pr0 gateway diagnostics before startup capture')
            holding=self.gateway.get('armed') == 'true'
            if holding:
                if self.gateway.get('phase')!='ARMED_HOLD' or self.gateway.get('task_stop_active')!='false':
                    raise RuntimeError('Capture requires stationary ARMED_HOLD without an active stop')
                supported_gateway(self.gateway)
            elif (self.gateway.get('armed')!='false' or
                    self.gateway.get('phase') not in ('UNARMED','LATCHED_SILENT')):
                raise RuntimeError('Startup capture requires an unarmed controller')
            home = self.session.capture_home()
            (self.run_dir/'session_home.json').write_text(json.dumps({
                'captured_at':datetime.now().astimezone().isoformat(),
                'operator_confirmed_r2_a':not holding,
                'operator_confirmed_hold_reference':holding,
                'capture_mode':'ARMED_HOLD' if holding else 'R2_A', 'joints':home,
                'hand_state_saved':False},indent=2)+'\n')
            self.status = ('Current return point captured from stationary hold; not the original R2+A pose.'
                           if holding else 'Session R2+A pose captured. Both arms; no hand state saved.')

    def record_ready(self, side):
        with self.lock:
            if self.session.phase in ('PLANNING','EXECUTION_PREFLIGHT','EXECUTING','RAIL_RECORDING','DEMO_RECORDING'):
                raise RuntimeError('Cannot record a moving/planning target')
            q = self.session.measured(stationary=True)
            names=list(ARMS[side])+(['waist_yaw_joint'] if self.planner.waist_assist else [])
            prepared = dict(self.prepared)
            prepared[side] = {'joints':{n:q[n] for n in names},
                'captured_at':datetime.now().astimezone().isoformat(),
                'operator_confirmed_above_table':True}
            if side == 'left' and (prepared.get('right') or {}).get('mirrored_from') == 'left':
                from task_kinematics import mirror_left_ready
                prepared['right'] = mirror_left_ready(self.model, prepared['left'])
            self.prepared = prepared
            self.session.invalidate()
            (self.run_dir/'ready_poses.json').write_text(json.dumps(self.prepared,indent=2)+'\n')
            self.status = side+' ready pose recorded. Startup recovery pose unchanged.'

    def reset_latch(self):
        with self.lock:
            if not self.args.hardware or self.args.disconnected or self.shutdown_requested:
                raise RuntimeError('当前面板不允许解除硬件锁存')
            if self.session.phase in ('PLANNING', 'EXECUTION_PREFLIGHT', 'EXECUTING'):
                raise RuntimeError('请先结束当前操作')
            if time.monotonic()-self.gateway_received > 1.5:
                raise RuntimeError('网关状态过期，未发送复位请求')
            if (self.gateway.get('phase') not in ('LATCHED_SILENT', 'UNARMED') or
                    self.gateway.get('armed') != 'false' or
                    self.gateway.get('command_publisher_exists') != 'false'):
                raise RuntimeError('仅允许对无控制输出的锁存状态执行复位')
            if self.gateway.get('task_reset_supported') != 'true':
                raise RuntimeError('网关尚未加载 STOP 复位修复版本，需要更新控制器；未发送旧版复位')
            epoch = int(self.gateway['task_stop_epoch'])
            self.status = '正在检查状态并解除本次锁存 / STOP，不会 ARM 或运动'
            requested_at = time.monotonic()
        response = self.call(self.reset_task, ResumeTaskHold.Request(stop_epoch=epoch))
        if not response.success:
            raise RuntimeError('未解除锁存：'+response.message)
        deadline = time.monotonic()+3
        while time.monotonic() < deadline:
            with self.lock:
                g = self.gateway
                if self.gateway_received > requested_at:
                    if int(g['task_stop_epoch']) != epoch:
                        raise RuntimeError('复位期间收到新的 STOP，仍保持停止；请重新检查后确认')
                    if (g.get('phase') == 'UNARMED' and g.get('armed') == 'false' and
                            g.get('command_publisher_exists') == 'false' and
                            g.get('task_stop_active') == 'false' and g.get('task_stop_requested') == 'false'):
                        self.session.invalidate()
                        if hasattr(self, 'screen'): self.screen.invalidate(clear_checks=True)
                        self.session.stopped = False
                        self.session.phase = 'HOME_CAPTURED' if self.session.home is not None else 'LIVE_READY'
                        self.stop_expected_epoch = epoch
                        self.recovery_pending = None
                        self.status = '当前状态正常，锁存和此前 STOP 已解除；保持未 ARM，启动点保留，请手动 ARM。'
                        return
            time.sleep(.05)
        raise RuntimeError('复位已应答，但尚未确认网关解除 STOP；未 ARM，请检查当前状态')

    def arm(self):
        with self.lock:
            if self.shutdown_requested:raise RuntimeError('Application is closing')
            if self.session.home is None: raise RuntimeError('Capture the session startup first')
            if not self.args.hardware: raise RuntimeError('Read-only launch: ARM is disabled')
        response = self.call(self.trigger['arm'], Trigger.Request())
        with self.lock: self.status = response.message

    def set_waist_control_mode(self, mode):
        with self.lock:
            if mode not in ('guarded', 'official_pd'): raise ValueError('未知腰控制模式')
            if not self.args.hardware or self.args.disconnected or self.shutdown_requested:
                raise RuntimeError('当前面板不允许切换硬件模式')
            if time.monotonic()-self.gateway_received > 1.5:
                raise RuntimeError('需要新鲜网关状态')
            if (self.gateway.get('phase') not in ('UNARMED', 'ARMED_HOLD') or
                    self.gateway.get('waist_mode_switching') != 'false'):
                raise RuntimeError('需要未 ARM 或静止保持，且没有正在进行的模式过渡')
            if self.gateway.get('armed') == 'true': self.session.measured(stationary=True)
            self.screen.idle()
            self.screen.invalidate(alignment=False, clear_checks=True)
        request = SetParametersAtomically.Request()
        request.parameters = [Parameter('waist_control_mode', value=mode).to_parameter_msg()]
        response = self.call(self.waist_parameters, request, timeout=5)
        if not response.result.successful: raise RuntimeError(response.result.reason)
        with self.lock:
            self.status = '腰模式已接受：'+mode+'；保持时平滑过渡 1 秒，等待网关显示完成再规划'

    def set_waist_gains(self,kp,kd,ki,integral_limit):
        with self.lock:
            if not self.args.hardware:raise RuntimeError('Read-only launch: waist tuning is disabled')
            if self.gateway.get('waist_control_mode', 'guarded') != 'guarded' or self.gateway.get('waist_mode_switching') == 'true':
                raise RuntimeError('原增益编辑只适用于现有控制模式，且需等待切换完成')
            if time.monotonic()-self.gateway_received>1.5:raise RuntimeError('Fresh gateway diagnostics required')
            phase=self.gateway.get('phase','UNKNOWN')
            if phase not in ('UNARMED','ARMED_HOLD'):
                raise RuntimeError('Waist gains require UNARMED or ARMED_HOLD; current: '+phase)
            if self.gateway.get('armed')=='true':self.session.measured(stationary=True)
        request=SetParametersAtomically.Request()
        request.parameters=[Parameter('waist_kp',value=float(kp)).to_parameter_msg(),
                            Parameter('waist_kd',value=float(kd)).to_parameter_msg(),
                            Parameter('waist_ki',value=float(ki)).to_parameter_msg(),
                            Parameter('waist_integral_torque_limit_nm',
                                      value=float(integral_limit)).to_parameter_msg()]
        response=self.call(self.waist_parameters,request,timeout=5)
        if not response.result.successful:raise RuntimeError('Waist gain update rejected: '+response.result.reason)
        with self.lock:self.status=(f'Waist gains applied atomically: Kp={kp:.1f}, Kd={kd:.1f}, '
                                   f'Ki={ki:.1f}, |I|<={integral_limit:.2f} Nm')

    def apply_task_stage(self,side,profile,parameters,ticket,epoch,*,wrist_kp=0.,distal_wrist_kp=0.,arm_kp=None):
        with self.lock:
            if self.gateway.get('task_stage_supported')!='true':
                raise RuntimeError('阶段增益需要新版控制器；请完整重启后重新规划')
            if profile!='normal':
                from task_stage_control import validate_gateway_gains
                validate_gateway_gains(parameters,self.gateway,wrist_kp=wrist_kp,
                    distal_wrist_kp=distal_wrist_kp,arm_kp=arm_kp)
            previous=int(self.gateway.get('task_stage_revision','0'))
            unchanged=profile=='normal' and self.gateway.get('task_stage_profile')=='normal'
        kp=0. if profile=='normal' else float(getattr(parameters,profile+'_proximal_kp'))
        kd=0. if profile=='normal' else float(getattr(parameters,profile+'_proximal_kd'))
        waist_kp=0. if profile=='normal' else float(parameters.stage_waist_kp)
        waist_kd=0. if profile=='normal' else float(parameters.stage_waist_kd)
        request=SetParametersAtomically.Request()
        values=[
            ('task_stage_profile',profile),('task_stage_arm',side),('task_stage_kp',kp),
            ('task_stage_kd',kd),('task_stage_stop_epoch',int(epoch))]
        if waist_kp:
            values.extend((('task_stage_waist_kp',waist_kp),('task_stage_waist_kd',waist_kd)))
        if wrist_kp:
            values.append(('task_stage_wrist_kp',float(wrist_kp)))
        if distal_wrist_kp:
            values.append(('task_stage_distal_wrist_kp',float(distal_wrist_kp)))
        if arm_kp is not None:
            values.append(('task_stage_arm_kp',[float(v) for v in arm_kp]))
        request.parameters=[Parameter(name,value=value).to_parameter_msg() for name,value in values]
        if not unchanged:
            if not self.waist_parameters.wait_for_service(timeout_sec=2.):
                raise RuntimeError('Stage parameter service unavailable')
            settle_deadline=time.monotonic()+5.
            while True:
                self.check(ticket)
                with self.lock:
                    if (int(self.gateway.get('task_stop_epoch','-1'))!=epoch or
                            self.gateway.get('task_stop_active')=='true' or
                            self.gateway.get('task_stop_requested')=='true'):
                        raise RuntimeError('STOP changed during stage transition')
                    if time.monotonic()-self.gateway_received>=1.5:
                        raise RuntimeError('Gateway diagnostics stale during stage transition')
                remaining=settle_deadline-time.monotonic()
                if remaining<=0:raise RuntimeError('Stage stationary wait timed out after 5 seconds')
                response=self.wait(self.waist_parameters.call_async(request),ticket,remaining)
                if response.result.successful:break
                # Only this explicit nonactuating rejection is retryable. Every
                # request still undergoes all gateway guards and STOP checks.
                if not response.result.reason.startswith('STAGE_WAIT_STATIONARY:'):
                    raise RuntimeError('Stage gains rejected: '+response.result.reason)
                with self.lock:self.status='等待静止后切换阶段增益（速度门槛0.03 rad/s，最多5秒）'
                time.sleep(.05)
        deadline=time.monotonic()+5.
        while time.monotonic()<deadline:
            self.check(ticket)
            with self.lock:
                g=self.gateway
                if int(g.get('task_stop_epoch','-1'))!=epoch or g.get('task_stop_active')=='true':
                    raise RuntimeError('STOP changed during stage transition')
                if (time.monotonic()-self.gateway_received<1.5 and
                    int(g.get('task_stage_revision','0'))>=previous+(0 if unchanged else 1) and
                    g.get('task_stage_profile')==profile and
                    g.get('task_stage_switching')=='false' and g.get('phase')=='ARMED_HOLD'):
                    expected={'task_stage_kp':kp,'task_stage_kd':kd}
                    if g.get('task_stage_waist_supported')=='true':
                        expected.update(task_stage_waist_kp=waist_kp,task_stage_waist_kd=waist_kd)
                    if g.get('task_stage_wrist_supported')=='true':
                        expected['task_stage_wrist_kp']=wrist_kp
                    if g.get('task_stage_distal_wrist_supported')=='true':
                        expected['task_stage_distal_wrist_kp']=distal_wrist_kp
                    if g.get('task_stage_arm_kp_supported')=='true':
                        expected['task_stage_arm_kp_count']=0 if arm_kp is None else 7
                        if arm_kp is not None:
                            expected.update({f'task_stage_arm_kp_{i}':v for i,v in enumerate(arm_kp)})
                    if profile!='normal' and g.get('task_stage_arm')!=side:
                        raise RuntimeError('Stage arm acknowledgement mismatch')
                    for name,value in expected.items():
                        actual=float(g.get(name,'nan'))
                        if not np.isfinite(actual) or abs(actual-value)>1e-6:
                            raise RuntimeError('Stage gains acknowledgement mismatch: '+name)
                    return
            time.sleep(.02)
        raise RuntimeError('Stage gains acknowledgement timed out')

    def stop(self, include_waist=False):
        # GUI sends this directly, never through the vision/planning worker queue.
        with self.lock:
            if include_waist:
                if (not self.args.hardware or self.args.disconnected or self.shutdown_requested or
                    time.monotonic()-self.gateway_received>1.5 or
                    self.gateway.get('task_waist_admittance_supported')!='true' or
                    self.gateway.get('phase')!='ARMED_HOLD' or
                    not self.waist_admittance_stop.service_is_ready()):
                    self.status='腰导纳未启动：需要新版网关、已ARM且静止保持；不会自动ARM或从运动中切换。'
                    return
                try:self.session.measured(stationary=True)
                except Exception as error:
                    self.status='腰导纳未启动：'+str(error)
                    return
            if hasattr(self, 'screen'):
                self.screen.invalidate(alignment=not include_waist, clear_checks=True)
            if hasattr(self, 'rail'):
                self.rail.started()
            if hasattr(self, 'demo'):self.demo.cancel_trace('STOP 请求中止录制；旧轨迹保留')
            if hasattr(self, 'landing_gear'):self.landing_gear.cancel_trace('STOP 请求中止录制；旧轨迹保留')
            self.recovery_pending=None
            self.session.stop()
            self.stop_expected_epoch=int(self.gateway.get('task_stop_epoch','0'))+1
            self.display_pub.publish(DisplayTrajectory())
            self.status = 'STOP REQUESTED: awaiting gateway acknowledgement; do not assume admittance yet.'
        if self.hand_control:
            self.hand.request_stop()
            def stop_hands():
                try:self.hand.hold_active()
                except Exception as error:
                    with self.lock:self.status=str(error)
            threading.Thread(target=stop_hands,daemon=True).start()
        client=self.waist_admittance_stop if include_waist else self.trigger['stop_task']
        if not client.service_is_ready():
            with self.lock: self.status = 'STOP UNCONFIRMED: gateway unavailable. Use native robot stop if needed.'
            return
        request=ResumeTaskHold.Request(stop_epoch=self.stop_expected_epoch-1) if include_waist else Trigger.Request()
        future = client.call_async(request)
        def received(done):
            try:
                response = done.result()
                if not response.success:
                    with self.lock: self.status = 'STOP UNCONFIRMED: '+response.message
            except Exception as e:
                with self.lock: self.status = 'STOP UNCONFIRMED: '+str(e)
        future.add_done_callback(received)

    def snapshot(self, ticket):
        hands_before=self.read_hands(ticket) if self.hand_control else {}
        with self.lock:
            self.status = 'Requesting one fresh RGB-D capture...'
            before = self.session.measured(stationary=True)
        directory = self.run_dir/f'plan_{ticket.generation:04d}'
        directory.mkdir()
        capture = directory/'capture.json'
        command = [str(ROOT/'scripts/run_task_vision_capture.sh'), self.args.network,
                   str(capture), '--save-artifacts']
        with (directory/'capture.log').open('w') as log:
            environment=dict(os.environ,G1_VISION_DDS_PEER=self.args.vision_peer)
            process = subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,
                                       start_new_session=True,env=environment)
            try:
                deadline = time.monotonic()+75
                while process.poll() is None:
                    self.check(ticket)
                    if time.monotonic()>deadline: raise TimeoutError('Vision request timed out')
                    time.sleep(.05)
                if not capture.is_file():
                    raise RuntimeError('Vision client failed before saving a reply: '+str(directory/'capture.log'))
                capture_data=json.loads(capture.read_text())
                validate_capture_result(capture_data,ticket.kind)
                if process.returncode not in (0,2):
                    raise RuntimeError('Vision client exited abnormally: '+str(directory/'capture.log'))
            finally:
                if process.poll() is None:
                    os.killpg(process.pid,signal.SIGTERM)
                    try: process.wait(timeout=3)
                    except subprocess.TimeoutExpired:
                        os.killpg(process.pid,signal.SIGKILL); process.wait()
        self.check(ticket)
        # SSH uses an existing key or operator-established ControlMaster, never a saved password.
        manifest=import_record(capture,host=self.args.ssh,intrinsics=Path(self.args.intrinsics))
        self.check(ticket)
        geometry = (build_task_preview(capture_data,Path(self.args.intrinsics),self.tcp,
                                      entry_key='operation_diagonal_intersection',
                                      radial_offset_m=self.planner.planning_parameters.radial_offset_mm/1000.,
                                      height_offset_m=self.planner.planning_parameters.height_offset_mm/1000.,
                                      arc_center_toward_robot_m=self.planner.planning_parameters.arc_center_toward_robot_mm/1000.,
                                      calibration=self.calibration)
                    if ticket.kind=='task' else measured_scene_geometry(capture_data,manifest))
        cloud = load_cloud(directory, geometry)
        with np.load(directory/'cloud.npz',allow_pickle=False) as data:
            xyz = data['xyz'].copy(); uv=data['uv'].copy()
        xyz=task_calibration.cloud_points(xyz,geometry)
        if geometry.get('calibration'):
            for arm in ARMS:
                self.tcp_marker_offsets[arm]=np.asarray(geometry['fingertip_estimates']['sides'][arm]['states']['open']['wrist_yaw_from_index_tcp']['matrix_row_major'])
        hands_after=self.read_hands(ticket) if self.hand_control else {}
        for side in hands_before:
            if max(abs(a-b) for a,b in zip(hands_before[side]['feedback_q'],hands_after[side]['feedback_q']))>.03:
                raise RuntimeError('Hand moved during capture; replan')
        with self.lock:
            self.session.check(ticket)
            q = self.session.measured(stationary=True)
            watched = list(ARM_NAMES)+['waist_yaw_joint']
            if any(abs(q.get(n,0)-before.get(n,0))>.015 for n in watched):
                raise RuntimeError('Robot moved during vision capture; discard and recapture')
            torso = self.model.transform('torso_link',q)
            geometry['capture_frame']={'frame':'pelvis','pelvis_from_torso_row_major':torso.tolist(),
                'source':'capture-time stationary LowState; frozen for this plan'}
            (directory/'geometry.json').write_text(json.dumps(geometry,indent=2)+'\n')
            (directory/'capture_state.json').write_text(json.dumps({
                'joints':q,'pelvis_from_torso_row_major':torso.tolist(),
                'captured_at':datetime.now().astimezone().isoformat(),
                'source':'stationary measured feedback immediately after capture'},indent=2)+'\n')
            # Freeze the measured surface in pelvis coordinates, not on a moving torso TF.
            packed = np.frombuffer(bytes(cloud.data),dtype=[('x','<f4'),('y','<f4'),('z','<f4'),('rgb','<u4')]).copy()
            frozen = xyz @ torso[:3,:3].T + torso[:3,3]
            for i, axis in enumerate('xyz'): packed[axis] = frozen[:,i]
            cloud.data = packed.tobytes(); cloud.header.frame_id = 'pelvis'
            cloud.header.stamp = self.get_clock().now().to_msg()
            self.session.accept_snapshot(ticket,{'directory':str(directory),'source':geometry['source']})
            self.cloud_pub.publish(cloud)
            self.markers(geometry,torso,ticket.side,ticket.kind)
            self.status = 'RGB-D saved. Building task collision checks...'
        server=json.loads((directory/'server_result.json').read_text())
        self.planner.install_surface(xyz,torso,ticket,uv=uv,bbox=server.get('pedal_centered'),
                                     center=geometry['task_preview']['axis_center_m'] if ticket.kind=='task' else None)
        if self.planner.scene_report.get('profile')=='steep_straight':
            self.marker_pub.publish(with_live_tcp(scoped_target_markers(geometry,torso,ticket.side,self.planner.scene_report), self.tcp_marker_offsets))
        (directory/'contact_scene.json').write_text(json.dumps(self.planner.scene_report,indent=2)+'\n')
        return geometry, directory, q

    def capture_archive_scene(self, side, callback):
        with self.lock:
            if self.worker and self.worker.is_alive():
                raise RuntimeError('Previous operation is still finishing')
            ticket=self.session.begin('task',side,.15,.15)
            self.display_pub.publish(DisplayTrajectory())
        def work():
            try:
                parameters,_=load_parameters(getattr(self.args,'planning_config',DEFAULT_CONFIG))
                configure_planner(self.planner,parameters)
                _,directory,_=self.snapshot(ticket)
                with self.lock:
                    self.session.check(ticket)
                    self.session.invalidate()
                    self.session.phase='HOME_CAPTURED'
                    generation=self.session.generation
                    self.status='Fresh RGB-D captured for overlay only; no executable plan.'
                callback(directory,generation,None)
            except Exception as error:
                with self.lock:self.session.fail(ticket)
                callback(None,None,str(error))
        self.launch_work(work)

    def markers(self, geometry, torso, side, kind):
        self.marker_pub.publish(with_live_tcp(target_markers(geometry,torso,side,kind), self.tcp_marker_offsets))

    def poll_planning_config(self):
        monitor=getattr(self,'planning_monitor',None)
        if monitor is None:return
        monitor.poll()
        if monitor.revision!=getattr(self,'planning_config_revision',-1):
            self.planning_config_pending=True
            self.planning_config_revision=monitor.revision
        with self.lock:
            if not self.planning_config_pending:return
            if ((self.worker is not None and self.worker.is_alive()) or
                self.session.phase in ('PLANNING','EXECUTION_PREFLIGHT','EXECUTING')):return
            plan=self.session.plan
            if plan and plan.get('planning_config') and not monitor.matches(plan['planning_config'],self.gateway):
                self.session.invalidate();self.session.phase='REPLAN_REQUIRED'
                self.display_pub.publish(DisplayTrajectory())
                self.status='参数文件已改变，旧计划失效；请重新 Plan。未发送运动命令。'
            self.planning_config_pending=False

    def set_calibration(self,values):
        values=task_calibration.validate(values)
        with self.lock:
            if ((self.worker is not None and self.worker.is_alive()) or
                    self.session.phase in ('EXECUTING','EXECUTION_PREFLIGHT') or self.shutdown_requested):
                raise RuntimeError('规划、执行或关闭期间不能修改补偿')
            self.calibration=values
            if hasattr(self, 'screen'):
                self.screen.invalidate()
            self.session.invalidate();self.session.phase='REPLAN_REQUIRED'
            self.display_pub.publish(DisplayTrajectory())
            self.tcp_marker_offsets=task_calibration.marker_offsets(self.model,values)
            self.marker_pub.publish(with_live_tcp(MarkerArray(markers=[Marker(action=Marker.DELETEALL)]),self.tcp_marker_offsets))
            self.status='补偿已修改；旧计划失效，请重新 Plan。尚未发送运动命令。'

    def plan(self, kind, side, velocity, acceleration, hand_posture_confirmed=False, sweep_direction='left',center_check=False,
             speed_multiplier=1.):
        from task_dring_hand_settings import load as load_hand_settings
        hand_settings=load_hand_settings() if kind=='task' else None
        if center_check and (kind!='task' or side not in ARMS):
            raise ValueError('Center check requires one selected arm')
        if not hand_posture_confirmed:
            raise RuntimeError('Confirm actual fingers match the approximate planning model before planning')
        if kind == 'task' and side not in ('left','right'):
            raise RuntimeError('Select one hand per D-ring task')
        if kind=='task' and sweep_direction not in ('left','right'):
            raise ValueError('Select left or right D-ring sweep')
        if kind=='task' and not hasattr(self.planner,'configure_task'):
            raise RuntimeError('Legacy D-ring planner retired; use the steep_straight single-sweep profile')
        with self.lock:
            ticket=self.session.begin(kind,side,velocity,acceleration)
            planning_epoch=int(self.gateway.get('task_stop_epoch','0'))
            self.validation_progress=None
            self.display_pub.publish(DisplayTrajectory())
            self.status='Requesting a fresh observation for '+kind+'...'
        def work():
            try:
                parameters,config_record=load_parameters(getattr(self.args,'planning_config',DEFAULT_CONFIG))
                with self.lock:
                    parameters,config_record=controller_bounded_parameters(parameters,config_record,self.gateway)
                configure_planner(self.planner,parameters)
                self.planner.dring_hand_settings=hand_settings
                optimized=kind=='task' and not center_check
                from task_path_continuity import ContinuityPolicy
                self.planner.continuity_policy=ContinuityPolicy() if optimized else None
                self.planner.ik_backend='bounded' if optimized else 'moveit'
                self.planner.joint_route_backend='ompl' if optimized else 'line'
                self.planner.preinsert_policy='diverse' if optimized else 'legacy'
                solver=dict(ik=self.planner.ik_backend,return_route=self.planner.joint_route_backend,
                            preinsert=self.planner.preinsert_policy)
                if optimized:solver['continuity_policy']=self.planner.continuity_policy.record()
                if optimized:
                    from task_joint_route import backend_identity
                    solver['return_version']=backend_identity()
                self.planner.search_diagnostics={}
                geometry,directory,q=self.snapshot(ticket)
                with self.lock: self.status='Planning '+kind+' from measured state...'
                start=dict(q)
                if kind=='task':
                    if side=='both': raise RuntimeError('Select one hand per D-ring task')
                    with self.lock:
                        ready=self.prepared[side]
                        if ready is None and not center_check:raise RuntimeError(side+' above-table pose has not been recorded')
                        ready_joints=dict(q)
                        if ready is not None:ready_joints.update(ready['joints'])
                        self.planner.configure_task(side,self.session.home,ready_joints,sweep_direction)
                    if center_check:
                        from task_center_check import plan_center_check
                        segments=plan_center_check(self.planner,side,q,geometry,ticket)
                        write_json(directory/'geometry.json',geometry)
                        torso=np.asarray(geometry['capture_frame']['pelvis_from_torso_row_major'])
                        self.marker_pub.publish(with_live_tcp(scoped_target_markers(geometry,torso,side,self.planner.scene_report), self.tcp_marker_offsets))
                    else:segments=self.planner.ring(side,q,geometry,ticket)
                else:
                    segments=[]
                    for arm in (('left','right') if side=='both' else (side,)):
                        with self.lock:
                            target=self.session.home if kind=='recover' else (
                                self.prepared[arm]['joints'] if self.prepared[arm] else None)
                        if target is None: raise RuntimeError(arm+' above-table pose has not been recorded')
                        if self.hand_control and kind=='prepare':
                            segment=self.planner.hand_transition(arm,q,'open',ticket)
                            segments.append(segment);q=segment['end']
                        segment=self.planner.joint_path(arm,q,target,ticket,kind+' '+arm)
                        segments.append(segment); q=segment['end']
                        if self.hand_control and kind=='recover':
                            segment=self.planner.hand_transition(arm,q,'open',ticket)
                            segments.append(segment);q=segment['end']
                from task_trajectory_speed import scale_arm_segments
                segments,speed_report=scale_arm_segments(segments,1. if kind=='task' else speed_multiplier,self.model,parameters,
                    velocity=velocity,acceleration=acceleration,check=lambda:self.check(ticket),policy='uniform')
                plan={'start':start,'segments':segments,'geometry':geometry,'arm':side,
                      'dring_hand_settings':hand_settings,
                      'speed_scaling':speed_report,'solver':solver,
                      'directory':str(directory),'kind':kind,
                      'waist_assist':self.planner.waist_assist,
                      'hand_posture':'dds_feedback_approximate_model' if self.hand_control else 'operator_confirmed_index_only',
                      'planning_profile':getattr(self.args,'dring_profile','steep_straight'),
                      'planning_config':config_record,
                      'hardware_executable':False if kind=='task' else None}
                from task_dring_sequence import SCHEMA
                schema=CENTER_SCHEMA if center_check else SCHEMA
                if kind=='task':plan.update(task_schema=schema,sweep_direction=None if center_check else sweep_direction)
                (directory/'plan.json').write_text(json.dumps({
                    'kind':kind,'arm':side,'start':start,'velocity_scaling':velocity,
                    'task_schema':schema if kind=='task' else None,'sweep_direction':sweep_direction if kind=='task' and not center_check else None,
                    'center_check':geometry.get('center_check'),
                    'acceleration_scaling':acceleration,'hardware_executed':False,
                    'planning_profile':plan['planning_profile'],'hardware_executable':plan['hardware_executable'],
                    'radial_offset_m':parameters.radial_offset_mm/1000.,
                    'height_offset_m':parameters.height_offset_mm/1000.,
                    'arc_center_toward_robot_m':parameters.arc_center_toward_robot_mm/1000.,
                    'planning_config':config_record,
                    'dring_hand_settings':hand_settings,
                    'speed_scaling':speed_report,'solver':solver,
                    'hand_posture':plan['hand_posture'],'waist_assist':self.planner.waist_assist,
                    'hand_geometry_calibrated':False,'contact_scene':self.planner.scene_report,
                    'search_diagnostics':self.planner.search_diagnostics,
                    'planning_seconds':time.monotonic()-self.session.planning_at,
                    'segments':[{'side':s['side'],'label':s['label'],'kind':s.get('kind','arm'),
                                 'posture':s.get('posture'),'target_q':s.get('target_q'),
                                 'trajectory':message_to_ordereddict(s['trajectory'])} for s in segments]
                },indent=2)+'\n')
                if kind=='task' and not center_check:
                    from task_live_archive import publish_live_plan
                    self.progress(ticket,'Saving verified trajectory to history...')
                    plan['archive_directory']=publish_live_plan(directory,self.args.robot_urdf,
                        getattr(self.args,'trajectory_library',None),check=lambda:self.check(ticket))
                    with self.lock:
                        self.session.check(ticket)
                        self.latest_archive=plan['archive_directory']
                if kind=='task':
                    segments,speed_report=scale_arm_segments(segments,speed_multiplier,self.model,parameters,
                        velocity=velocity,acceleration=acceleration,check=lambda:self.check(ticket))
                    plan.update(segments=segments,speed_scaling=speed_report)
                    record=json.loads((directory/'plan.json').read_text())
                    for saved,segment in zip(record['segments'],segments):
                        saved['trajectory']=message_to_ordereddict(segment['trajectory'])
                    record.update(speed_scaling=speed_report,
                                  archive_directory=plan.get('archive_directory'))
                    write_json(directory/'plan.json',record)
                if kind=='task':
                    plan['live_review']=dict(generation=ticket.generation,stop_epoch=planning_epoch,
                        source_plan_sha256=digest(directory/'plan.json'),robot_model_sha256=digest(self.args.robot_urdf))
                self.finish_plan(ticket,plan)
                with self.lock:
                    self.session.check(ticket)
                    display=DisplayTrajectory(); display.trajectory_start=robot_state(start)
                    display.trajectory=[RobotTrajectory(joint_trajectory=s['trajectory']) for s in segments]
                    self.display_pub.publish(display)
                    elapsed=time.monotonic()-self.session.planning_at
                    self.status=(f'PLAN READY: center check +{parameters.center_check_height_mm:g} mm ({elapsed:.1f} s).' if center_check else
                        f'PLAN READY: {kind} ({elapsed:.1f} s). '+
                        f'Speed {speed_report["applied_multiplier"]:.2f}x / requested {speed_multiplier:g}x. '+
                        (('RViz only; requested speed is not hardware-authorized.' if plan.get('preview_only_timing') else
                          'Full D-ring test ready for contact/path confirmation.' if getattr(self.args,'dring_contact_test',False)
                          else 'Base/body-scoped preview; D-ring execution not commissioned.') if kind=='task' else
                         'Review the validated path, then confirm.')+
                        (' Saved to trajectory history.' if plan.get('archive_directory') else ''))
            except Exception as error:
                failure_dir=self.run_dir/f'plan_{ticket.generation:04d}'
                if failure_dir.is_dir():
                    (failure_dir/'planning_failure.json').write_text(json.dumps({
                        'kind':kind,'side':side,'message':str(error),
                        'diagnostic':getattr(error,'diagnostic',None),
                        'search_diagnostics':self.planner.search_diagnostics,
                        'hardware_executed':False},indent=2)+'\n')
                with self.lock:
                    self.session.fail(ticket)
                    self.display_pub.publish(DisplayTrajectory())
                raise
        try: self.launch_work(work)
        except Exception:
            with self.lock: self.session.fail(ticket)
            raise

    def recovery(self, velocity, acceleration):
        def work():
            with self.lock:
                generation = self.session.generation
                if time.monotonic()-self.gateway_received>1.5:
                    raise RuntimeError('Recovery requires fresh gateway stop acknowledgement')
                epoch=int(self.gateway['task_stop_epoch'])
            def unchanged():
                if self.session.generation!=generation:
                    raise RuntimeError('Recovery cancelled by a newer stop/request')
            TaskNode.wait_stationary(self,guard=unchanged,context='Recovery precheck')
            with self.lock:
                unchanged()
                self.recovery_pending=dict(generation=generation,epoch=epoch)
            response=self.call(self.resume_hold,ResumeTaskHold.Request(stop_epoch=epoch))
            if not response.success:
                with self.lock:self.recovery_pending=None
                raise RuntimeError(response.message)
            deadline=time.monotonic()+5
            while time.monotonic()<deadline:
                with self.lock:
                    if (not self.session.stopped and self.session.phase in
                            ('HOLD_AWAITING_FEEDBACK','HOLD_FOR_RECOVERY') and
                            int(self.gateway.get('task_stop_epoch','-1'))==epoch):
                        return
                    if self.session.generation!=generation or int(self.gateway.get('task_stop_epoch','-1'))!=epoch:
                        raise RuntimeError('Recovery cancelled by a newer stop/request')
                    if (time.monotonic()-self.gateway_received<1.5 and self.gateway.get('phase')=='ARMED_HOLD'
                            and self.gateway.get('task_stop_active')=='false'):
                        self.session.recovered_hold(acknowledged=True)
                        self.recovery_pending=None
                        self.status='Controller hold confirmed; waiting for fresh stationary feedback.'
                        return
                time.sleep(.05)
            raise RuntimeError('Hold acknowledgement pending; fresh controller diagnostics will reconcile the panel')
        self.launch_work(work)

    def saved_test_guard(self, plan=None, execution=True, *, stationary=True):
        if not self.args.hardware or not self.hand_control:
            raise RuntimeError('Saved tests require the hardware session and hand DDS')
        if time.monotonic()-self.gateway_received > 1.5:
            raise RuntimeError('Gateway diagnostics stale')
        if stationary:supported_gateway(self.gateway)
        else:supported_controller(self.gateway)
        if self.gateway['phase'] != 'ARMED_HOLD' or self.gateway['task_stop_active'] != 'false':
            raise RuntimeError('Saved test requires ARMED_HOLD with no STOP')
        if plan:
            proof=plan['test_range']
            if execution and is_center_check(plan) and not proof.get('hands_match_at_review'):
                raise RuntimeError('Actual hands differ from selected range start; preview only, reload after matching')
            if int(self.gateway['task_stop_epoch']) != proof['stop_epoch']:
                raise RuntimeError('Stop epoch changed; reload and check the selected range')
            if plan['waist_assist'] and self.gateway.get('task_waist_trajectory_supported') != 'true':
                raise RuntimeError('Waist execution is not enabled')

    def retime_plan(self, multiplier):
        with self.lock:
            if self.worker and self.worker.is_alive():raise RuntimeError('Previous operation is still finishing')
            source=self.session.plan
            if (self.session.phase not in ('PLAN_READY','RETIME_REQUIRED') or not source or
                    source.get('kind')!='task' or source.get('hardware_executable') is not False):
                raise RuntimeError('Select a completed live D-ring preview, or reload a saved range with the new speed')
            issue=self.session.preview_issue(source)
            if issue:raise RuntimeError(issue)
            ticket=self.session.ticket
            self.session.check(ticket)
            self.session.phase='PLANNING';self.session.planning_at=time.monotonic()
            self.display_pub.publish(DisplayTrajectory())
        def work():
            try:
                from task_trajectory_speed import scale_arm_segments, restore_baseline
                parameters,config=load_parameters(getattr(self.args,'planning_config',DEFAULT_CONFIG))
                with self.lock:
                    parameters,config=controller_bounded_parameters(parameters,config,self.gateway)
                if config!=source['planning_config']:
                    raise RuntimeError('Planning/controller configuration changed; replan')
                source_file=Path(source['directory'])/'plan.json'
                if digest(source_file)!=source['live_review']['source_plan_sha256']:
                    raise RuntimeError('Original plan changed; replan')
                previous_report=dict(source.get('speed_scaling',{}))
                if source.get('preview_only_timing'):previous_report['hand_timing_preserved']=False
                original=restore_baseline(source['segments'],previous_report)
                segments,report=scale_arm_segments(original,multiplier,self.model,parameters,
                    velocity=ticket.velocity,acceleration=ticket.acceleration,check=lambda:self.check(ticket))
                directory=self.run_dir/('retimed_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
                shutil.copytree(source['directory'],directory)
                record=json.loads(source_file.read_text())
                for saved,segment in zip(record['segments'],segments):
                    saved['trajectory']=message_to_ordereddict(segment['trajectory'])
                record.update(speed_scaling=report,retimed_from=str(source_file),preview_only_timing=False)
                write_json(directory/'plan.json',record)
                plan=dict(source,segments=segments,directory=str(directory),speed_scaling=report,preview_only_timing=False,
                    live_review=dict(source['live_review'],source_plan_sha256=digest(directory/'plan.json')))
                with self.lock:
                    self.session.check(ticket)
                    issue=self.session.preview_issue(plan)
                    if issue:raise RuntimeError(issue)
                    self.session.ready(ticket,plan)
                    display=DisplayTrajectory();display.trajectory_start=robot_state(plan['start'])
                    display.trajectory=[RobotTrajectory(joint_trajectory=s['trajectory']) for s in segments]
                    self.display_pub.publish(display)
                    self.status=(f'Retimed existing path: {report["applied_multiplier"]:.2f}x / requested {multiplier:g}x, '
                                 f'{report["duration_after_s"]:.1f} s. Motion limits applied; execution confirmations required.')
            except Exception as error:
                with self.lock:
                    self.session.fail(ticket)
                    self.status=str(error)
        self.launch_work(work)

    def load_saved_test(self, archive_path, first, last, velocity, acceleration,
                        scene_confirmed=False, contact_confirmed=False, hand_confirmed=False, selected_side=None,
                        speed_multiplier=None,use_collision_cache=False):
        from task_dring_hand_settings import load as load_hand_settings,archive_variant
        hand_settings=load_hand_settings()
        if not scene_confirmed or not hand_confirmed:
            raise RuntimeError('Confirm unchanged scene/base and actual hand model first')
        if type(first) is not int or type(last) is not int or not 1<=first<=last<=7:
            raise ValueError('Select an ordered range from stages 1-7')
        if needs_contact_confirmation(first,last) and not contact_confirmed:
            raise RuntimeError('Contact range requires explicit alignment/contact-test confirmation')
        with self.lock:
            if self.worker and self.worker.is_alive():
                raise RuntimeError('Previous operation is still finishing')
            self.saved_test_guard()
            ticket=self.session.begin('staged_test','both',velocity,acceleration)
            epoch=int(self.gateway['task_stop_epoch'])
            self.display_pub.publish(DisplayTrajectory())
            self.status='Checking saved test range; no movement commanded.'
        def work():
            directory=self.run_dir/f'staged_test_{ticket.generation:04d}'
            try:
                parameters,config_record=load_parameters(getattr(self.args,'planning_config',DEFAULT_CONFIG))
                with self.lock:
                    parameters,config_record=controller_bounded_parameters(parameters,config_record,self.gateway)
                archive=load_archive(archive_path,self.args.robot_urdf)
                if archive['plan'].get('preview_only_timing'):
                    raise RuntimeError('Import this preview-only archive into RViz, not the hardware test loader')
                archive=archive_variant(archive,hand_settings)
                if archive['hand_settings_changed'] and 3<=first<=5:
                    raise RuntimeError('New finger settings require including stage 2 (Hand dring_two_finger) in the selected range')
                archived_parameters=parameters_from_values(archive['plan'].get('planning_config',{}).get('values',{}))
                from task_planning_config import SECTIONS
                if any(getattr(archived_parameters,k)!=getattr(parameters,k) for k in SECTIONS['stage_control']):
                    raise RuntimeError('保存轨迹的阶段增益/速度配置与当前值不同；请重新规划或恢复归档配置')
                archived_geometry=json.loads((Path(archive['directory'])/'plan/geometry.json').read_text())
                archived_calibration=archived_geometry.get('calibration')
                if (archived_calibration != task_calibration.snapshot(self.calibration) and
                        (archived_calibration is not None or any(self.calibration[k] for k in ('a_enabled','b_enabled','c_enabled')))):
                    raise RuntimeError('保存轨迹的补偿配置与当前值不同；请重新规划，或恢复完全相同的配置')
                selected=select_range(archive,first,last)
                side=archive['plan']['arm']
                if selected_side is not None and selected_side != side:
                    raise RuntimeError('Saved archive is for '+side+'; select that arm above before loading')
                hands=self.read_hands(ticket)
                with self.lock:
                    self.session.check(ticket)
                    q=self.session.measured(stationary=True)
                    compared=compare_start(selected[0],q,hands)
                for hand_side,hand in hands.items():
                    q.update(model_joints(hand_side,hand['feedback_q']))
                from task_saved_transition import plan_transition,attach_transition,start_differences
                configure_planner(self.planner,parameters)
                cloud=load_cloud(Path(archive['directory'])/'plan',archived_geometry)
                torso=np.asarray(archived_geometry['capture_frame']['pelvis_from_torso_row_major'])
                packed=np.frombuffer(bytes(cloud.data),dtype=[('x','<f4'),('y','<f4'),('z','<f4'),('rgb','<u4')]).copy()
                xyz=np.column_stack([packed[axis] for axis in 'xyz'])
                self.progress(ticket,'Planning alignment from actual state to saved start...')
                prefix=plan_transition(self.planner,self.model,q,selected[0]['start'],side,ticket,parameters,
                    lambda:self.check(ticket))
                speed_report={};collision_report={}
                segments=prepare_range(archive,first,last,velocity,acceleration,self.model,
                    lambda:self.check(ticket),lambda message:self.progress(ticket,message),parameters,
                    speed_multiplier=speed_multiplier,speed_report=speed_report,
                    use_collision_cache=use_collision_cache,collision_report=collision_report)
                proof=dict(archive=archive['directory'],arm=side,first=first,last=last,
                    original_plan_sha256=archive['manifest']['original_plan_sha256'],
                    velocity_scaling=velocity,acceleration_scaling=acceleration,
                    speed_scaling=speed_report,
                    dring_hand_settings=hand_settings,hand_settings_changed=archive['hand_settings_changed'],
                    scene_confirmed=True,contact_confirmed=contact_confirmed,
                    stop_epoch=epoch,validated_fingerprint=range_fingerprint(segments))
                plan=dict(kind='staged_test',start=segments[0]['start'],segments=segments,
                          calibration=archived_calibration,
                          task_schema=archive['plan']['task_schema'],sweep_direction=archive['plan']['sweep_direction'],
                          directory=str(directory),waist_assist=archive['plan']['waist_assist'] or
                              any('waist_yaw_joint' in s['trajectory'].joint_names for s in prefix),
                          planning_config=config_record,
                          dring_hand_settings=hand_settings,
                          hardware_executable=True,test_range=proof,collision_validation=collision_report)
                plan=attach_transition(plan,prefix)
                proof=plan['test_range'];segments=plan['segments']
                verify_test_plan(plan)
                # Feedback is requested again after validation; cached hand joints
                # are not a substitute for current DDS posture.
                hands=self.read_hands(ticket)
                with self.lock:
                    self.session.check(ticket)
                    self.saved_test_guard(plan,execution=False)
                    reviewed_q=self.session.measured(stationary=True)
                    compared=compare_start(selected[0],reviewed_q,hands)
                    proof['hands_match_at_review']=compared['hand_start_matches']
                    proof['hand_posture_match_required']=False
                    proof['archive_start_comparison']=compared
                    if start_differences(reviewed_q,plan['start']):
                        raise RuntimeError('Robot moved after alignment planning; reload the saved range')
                directory.mkdir()
                write_json(directory/'test_plan.json',dict(test_range=proof,
                    dring_hand_settings=hand_settings,
                    collision_validation=collision_report,
                    task_schema=plan['task_schema'],sweep_direction=plan['sweep_direction'],
                    planning_config=config_record,
                    original_archive_executable=False,operator_confirmed_test=True,hardware_executed=False,
                    segments=[{**{k:v for k,v in s.items() if k!='trajectory'},
                               'trajectory':message_to_ordereddict(s['trajectory'])} for s in segments]))
                geometry=archived_geometry
                xyz=xyz @ torso[:3,:3].T + torso[:3,3]
                for i,axis in enumerate('xyz'):packed[axis]=xyz[:,i]
                cloud.data=packed.tobytes();cloud.header.frame_id='pelvis'
                cloud.header.stamp=self.get_clock().now().to_msg()
                with self.lock:
                    if use_collision_cache:
                        from task_collision_cache import remember
                        remember(collision_report['certificates'])
                    self.session.ready(ticket,plan)
                    self.cloud_pub.publish(cloud)
                    self.marker_pub.publish(with_live_tcp(scoped_target_markers(geometry,torso,side,archive['plan']['contact_scene']), self.tcp_marker_offsets))
                    display=DisplayTrajectory();display.trajectory_start=robot_state(plan['start'])
                    display.trajectory=[RobotTrajectory(joint_trajectory=s['trajectory']) for s in segments]
                    self.display_pub.publish(display)
                    duration=sum(s['duration'] for s in segments)
                    self.status=(f'TEST READY: {side} stages {first}-{last}, {duration:.1f} s. '+
                        (f'Speed {speed_report["applied_multiplier"]:.2f}x / requested {speed_multiplier:g}x. '
                         if speed_report else '')+
                        (f'Includes alignment: {", ".join(s["side"].upper() for s in prefix)}; collision NOT CHECKED. ' if prefix else '')+
                        'Confirm to execute the reviewed alignment and selected range.')
            except Exception as error:
                directory.mkdir(exist_ok=True)
                write_json(directory/'failure.json',dict(message=str(error),hardware_executed=False))
                with self.lock:
                    self.session.fail(ticket)
                    self.display_pub.publish(DisplayTrajectory())
                raise
        self.launch_work(work)

    def remember_saved_collision_check(self):
        with self.lock:
            plan=self.session.plan
            if not plan or plan.get('kind')!='staged_test':return
            verify_test_plan(plan)
            report=plan.get('collision_validation')
            if not report:raise RuntimeError('Validate this range once before remembering collision checks')
            from task_collision_cache import remember
            remember(report['certificates'])
            self.status='Collision checks saved for this exact range/timing; actual start and scene confirmation still required.'

    def execute(self, *, scene_confirmed=False, contact_confirmed=False):
        monitor=getattr(self,'planning_monitor',None)
        if monitor is not None:
            # Recheck synchronously before authorization, even between watcher polls.
            monitor.reload()
            issue=monitor.execution_issue(self.gateway)
            if issue:raise RuntimeError(issue)
        with self.lock:
            if not self.args.hardware: raise RuntimeError('Read-only launch: real execution disabled')
            if self.session.stopped:
                raise RuntimeError('Controller is stopped; execution requires hold')
            if time.monotonic()-self.gateway_received>1.5 or self.gateway.get('phase')!='ARMED_HOLD':
                raise RuntimeError('Gateway is not freshly confirmed ARMED_HOLD')
            original=self.session.plan;ticket=self.session.ticket
            if original and original.get('dring_hand_settings') is not None:
                from task_dring_hand_settings import load as load_hand_settings
                if original['dring_hand_settings']!=load_hand_settings():
                    raise RuntimeError('Finger settings changed for the next run; plan/reload the trajectory before execution')
            if (monitor is not None and original and original.get('planning_config') and
                not monitor.matches(original['planning_config'],self.gateway)):
                raise RuntimeError('参数文件与计划快照不同，请重新 Plan 后再执行')
            if self.session.phase!='PLAN_READY' or original is None:
                raise RuntimeError('No complete validated plan')
            if original.get('preview_only_timing'):
                raise RuntimeError('Unrestricted RViz speed is not authorized for hardware execution')
            self.session.check(ticket)
            self.session.measured(stationary=True)
            epoch=int(self.gateway.get('task_stop_epoch','0'))
            calibration=original.get('geometry',{}).get('calibration') or original.get('calibration')
            if calibration and calibration!=task_calibration.snapshot(self.calibration):
                raise RuntimeError('Calibration changed; replan before execution')
            if original['kind']=='task':
                if not is_center_check(original) and not getattr(self.args,'dring_contact_test',False):
                    raise RuntimeError('D-ring contact-test mode is not enabled')
                if not scene_confirmed or (not is_center_check(original) and not contact_confirmed):
                    raise RuntimeError('D-ring scene/contact confirmations required')
                issue=self.session.preview_issue(original)
                if issue:raise RuntimeError(issue)
                self.saved_test_guard()
            elif original['kind']=='staged_test':
                self.saved_test_guard(original)
                if original['test_range'].get('transition_count') and not scene_confirmed:
                    raise RuntimeError('Confirm the complete alignment path is clear; alignment collision was not checked')
            elif original['kind']=='screen':
                if not scene_confirmed:
                    raise RuntimeError('Confirm the screen path is clear before execution')
                self.screen.execution_guard(original, preflight=True)
            elif original['kind']=='rail':
                if not scene_confirmed:
                    raise RuntimeError('Confirm the rail path is clear before execution')
                self.rail.execution_guard(original, preflight=True)
            elif original['kind']=='dring_demo':
                if not scene_confirmed:raise RuntimeError('确认含起点衔接的完整回放路径净空')
                demonstration_control(self,original).execution_guard(original,preflight=True)
            elif original.get('hardware_executable') is False:
                raise RuntimeError('Preview only; hardware execution is not commissioned')
            if (original.get('waist_assist') and
                    self.gateway.get('task_waist_trajectory_supported')!='true'):
                raise RuntimeError('Waist-assisted plans are preview-only with this gateway; start a waist-enabled controller session')
            self.session.phase='EXECUTION_PREFLIGHT'
            self.status='Verifying execution authorization; no motion commanded.'
        def work():
            evidence=[];motion_started=False;plan=original;segment=None
            events_path=Path(original['directory'])/f'execution_events_{time.time_ns()}.jsonl'
            def checkpoint(name, **details):
                with self.lock:
                    age=self.session.clock()-self.session.received
                    entry=dict(checkpoint=name,monotonic_s=time.monotonic(),generation=ticket.generation,
                        stage=segment.get('number') if segment else None,
                        segment=segment.get('label') if segment else None,
                        state_age_s=age if np.isfinite(age) else None,
                        source_stamp_ns=self.session.source_stamp_ns,
                        feedback_rejection=self.session.feedback_rejection,
                        gateway_phase=self.gateway.get('phase'),**details)
                    self.execution_checkpoint=entry
                    if name in ('hand_transition_start','arm_send_pending'):
                        self.status='Executing: '+segment['label']
                with events_path.open('a') as log:log.write(json.dumps(entry,allow_nan=False)+'\n')
            def hold_guard(completed_action=False, *, during_hand=False):
                if int(self.gateway.get('task_stop_epoch','0'))!=epoch:
                    raise RuntimeError('Stop epoch changed during execution')
                if self.gateway.get('task_stop_active')=='true':
                    raise RuntimeError('Controller STOP active')
                if time.monotonic()-self.gateway_received>1.5:
                    raise FeedbackPending('Gateway diagnostics pending')
                if self.gateway.get('phase')!='ARMED_HOLD':
                    if completed_action and self.gateway.get('phase')=='RUNNING':
                        raise FeedbackPending('Completed action awaiting hold diagnostic')
                    raise RuntimeError('Controller hold lost')
                if plan['kind']=='staged_test':self.saved_test_guard(plan,stationary=not during_hand)
                if plan['kind']=='screen':self.screen.execution_guard(plan)
                if plan['kind']=='rail':self.rail.execution_guard(plan)
                if plan['kind']=='dring_demo':demonstration_control(self,plan).execution_guard(plan)
            def commit(_q):
                if self.session.plan is not original:
                    raise RuntimeError('Execution plan changed during authorization')
                result = self.session.execution(plan)
                if plan['kind']=='rail':self.rail.started()
                return result
            try:
                parameters=parameters_from_values(original.get('planning_config',{}).get('values',{}))
                dring_contact_task=not is_center_check(original) and original['kind'] in ('task','staged_test')
                from task_stage_control import recorded_replay_segment
                recorded_replay=any(recorded_replay_segment(original['kind'],s) for s in original['segments'])
                if recorded_replay:
                    from task_stage_control import replay_stage_parameters
                    parameters=replay_stage_parameters(parameters)
                stage_control=parameters.stage_control_enabled and (dring_contact_task or recorded_replay)
                if stage_control:
                    from task_stage_control import validate_gateway_gains,replay_arm_kp
                    validate_gateway_gains(parameters,self.gateway,
                        wrist_kp=parameters.demo_wrist_kp if recorded_replay else 0.,
                        distal_wrist_kp=parameters.demo_distal_wrist_kp if recorded_replay else 0.,
                        arm_kp=replay_arm_kp(parameters,original['arm']) if recorded_replay else None)
                if not stage_control and self.gateway.get('task_stage_profile','normal')!='normal':
                    raise RuntimeError('Previous stage gains still active; wait for normal profile before execution')
                endpoint_tolerance=float(np.radians(parameters.endpoint_tolerance_deg))
                if original.get('screen',{}).get('operation')=='waist_recovery':
                    endpoint_tolerance=min(endpoint_tolerance,float(np.radians(.5)))
                checkpoint('authorization_start',endpoint_tolerance_deg=float(np.degrees(endpoint_tolerance)),
                           endpoint_wait_seconds=parameters.endpoint_wait_seconds)
                # Parsing, fingerprints and file I/O must never hold the feedback lock.
                if original['kind']=='task':
                    plan=live_test_plan(original,ticket,epoch,self.args.robot_urdf,
                        scene_confirmed=scene_confirmed,contact_confirmed=contact_confirmed)
                elif plan['kind']=='staged_test':verify_test_plan(plan)
                checkpoint('authorization_checked')
                TaskNode.wait_stationary(self,ticket,expected=plan['start'],guard=hold_guard,
                    context='Execution preflight',on_ready=commit)
                if original['kind']=='task':
                    write_json(Path(plan['directory'])/'live_test_authorization.json',plan['test_range'])
                if self.hand_control and not (plan['kind']=='rail' and plan['rail'].get('arm_only')):
                    self.hand.begin_motion(lambda:self.check(ticket))
                for segment_index,segment in enumerate(plan['segments']):
                    self.check(ticket)
                    stage_revision=None
                    boundary_tolerance=endpoint_tolerance if segment_index else .025
                    checkpoint('segment_precheck')
                    TaskNode.wait_stationary(self,ticket,expected=segment['start'],guard=hold_guard,
                        context=segment['label']+' precheck',tolerance_rad=boundary_tolerance)
                    if stage_control:
                        from task_stage_control import segment_stage
                        profile=segment_stage(plan['kind'],segment)
                        checkpoint('stage_gains_requested',profile=profile)
                        motion_started=True
                        stage_options=({'wrist_kp':parameters.demo_wrist_kp,
                                        'distal_wrist_kp':parameters.demo_distal_wrist_kp}
                            if recorded_replay_segment(plan['kind'],segment) else {})
                        if stage_options:
                            gains=replay_arm_kp(parameters,segment['side'])
                            if gains is not None:stage_options['arm_kp']=gains
                        TaskNode.apply_task_stage(self,segment['side'],profile,parameters,ticket,epoch,**stage_options)
                        stage_revision=int(self.gateway['task_stage_revision'])
                        checkpoint('stage_gains_ready',profile=profile,
                                   revision=self.gateway.get('task_stage_revision'))
                    if self.hand_control and not (
                            plan['kind']=='dring_demo' and plan['demo'].get('arm_only') or
                            plan['kind']=='rail' and plan['rail'].get('arm_only')):
                        checkpoint('reading_hand_starts')
                        hands=(getattr(self,plan['kind']).read_stable_hands(ticket,lambda:self.check(ticket))
                               if plan['kind'] in ('screen','rail') else self.read_hands(ticket))
                        # D-ring contact may deflect fingers; retain health reads, not pose matching.
                        if not dring_contact_task and plan['kind']!='dring_demo':
                            for side in ARMS:
                                actual=model_joints(side,hands[side]['feedback_q'])
                                if any(abs(actual[n]-segment['start'][n])>.04 for n in actual):
                                    raise RuntimeError('Hand posture changed since planning; replan')
                        else:
                            checkpoint('hand_health_checked',posture_match_required=False)
                    TaskNode.wait_stationary(self,ticket,expected=segment['start'],guard=hold_guard,
                        context=segment['label']+' after hand read',tolerance_rad=boundary_tolerance)
                    if segment.get('kind')=='hand':
                        def hand_check():
                            self.check(ticket)
                            with self.lock:
                                # Stationary admission was checked before the ramp;
                                # during it, monitor controller health and held position.
                                hold_guard(during_hand=True)
                                # The arm gateway owns hold/LowState monitoring while
                                # fingers move. A delayed GUI sample is not hand loss.
                                measured=self.session.current
                                watched=ARM_NAMES+(('waist_yaw_joint',) if 'waist_yaw_joint' in segment['start'] else ())
                                if any(abs(measured[n]-segment['start'][n])>boundary_tolerance for n in watched):
                                    raise RuntimeError('Robot moved during hand transition; hold fingers and replan')
                        checkpoint('hand_transition_start',target_q=segment['target_q'])
                        motion_started=True
                        from task_hand_completion import transition as transition_hand
                        readings=transition_hand(self.hand,segment['side'],segment['target_q'],hand_check,
                            require_target=segment.get('hand_target_required',True),duration=segment['duration'],
                            on_feedback=lambda state:self.cache_hand_state(segment['side'],state),
                            on_event=lambda name,**details:checkpoint('hand_'+name,**details))
                        evidence.append({'segment':segment['label'],'kind':'hand','feedback':readings,
                                         'hand_target_required':segment.get('hand_target_required',True),
                                         'phalanx_geometry_measured':False})
                        checkpoint('segment_complete')
                        continue
                    client=(self.waist_action if plan.get('screen',{}).get('operation')=='waist_recovery'
                            else self.actions[segment['side']])
                    if not client.wait_for_server(timeout_sec=2):
                        raise RuntimeError('Waist hold-reference action unavailable; load the new gateway' if
                            plan.get('screen',{}).get('operation')=='waist_recovery' else 'Arm action unavailable')
                    # Serialize the send with stop invalidation. Gateway independently checks stop state.
                    checkpoint('arm_send_pending')
                    sent_at=[None]
                    delayed_motion=None
                    if plan['kind']=='rail' and segment.get('delayed_hand') is not None:
                        from task_rail_hand import MotionClock
                        delayed_motion=MotionClock()
                    def send(_q):
                        nonlocal motion_started
                        if stage_control and (int(self.gateway.get('task_stage_revision','-1'))!=stage_revision or
                            self.gateway.get('task_stage_profile')!=profile or
                            self.gateway.get('task_stage_switching')!='false'):
                            raise RuntimeError('Stage profile changed before sending trajectory; replan')
                        trajectory=segment['trajectory']
                        trajectory.header.stamp=self.get_clock().now().to_msg()
                        motion_started=True
                        sent_at[0]=time.monotonic()
                        options=({'feedback_callback':delayed_motion.feedback} if delayed_motion is not None else {})
                        return client.send_goal_async(FollowJointTrajectory.Goal(trajectory=trajectory),**options)
                    goal_future=TaskNode.wait_stationary(self,ticket,expected=segment['start'],guard=hold_guard,
                        context=segment['label']+' send',on_ready=send,tolerance_rad=boundary_tolerance)
                    goal=self.wait(goal_future,ticket)
                    if not goal.accepted: raise RuntimeError('Gateway rejected arm trajectory')
                    checkpoint('arm_accepted')
                    trajectory=segment['trajectory']
                    duration=trajectory.points[-1].time_from_start
                    if delayed_motion is not None:
                        from task_rail_hand import replay as replay_delayed_hand
                        result,finger_evidence=replay_delayed_hand(self.hand,segment['side'],segment['delayed_hand'],
                            goal.get_result_async(),delayed_motion,lambda:self.check(ticket),
                            lambda state:self.cache_hand_state(segment['side'],state),
                            lambda name,**details:checkpoint('hand_'+name,**details),
                            timeout=2*segment['duration']+30)
                        evidence.append(dict(segment=segment['label'],delayed_hand=finger_evidence))
                    elif plan['kind']=='dring_demo' and segment.get('recorded_hand') is not None:
                        if time.monotonic()-sent_at[0]>.15:
                            raise RuntimeError('手臂接收动作延迟超过 150 ms，无法同步手指回放')
                        from task_dring_demo_hand import replay as replay_fingers
                        result,finger_evidence=replay_fingers(self.hand,segment['side'],segment['recorded_hand'],
                            goal.get_result_async(),lambda:self.check(ticket),
                            lambda state:self.cache_hand_state(segment['side'],state))
                        evidence.append(dict(segment=segment['label'],recorded_fingers=finger_evidence))
                    else:
                        result=self.wait(goal.get_result_async(),ticket,2*(duration.sec+duration.nanosec*1e-9)+30)
                    evidence.append({'segment':segment['label'],'status':result.status,
                                     'error_code':result.result.error_code,'message':result.result.error_string})
                    if result.status!=4 or result.result.error_code!=0:
                        raise RuntimeError('Arm execution failed: '+result.result.error_string)
                    checkpoint('arm_completed_waiting_settle')
                    if is_center_check(plan) and segment['label']=='Center check':
                        from task_center_check import measurement
                        with self.lock:
                            center_q=dict(self.session.current)
                            center_age=self.session.clock()-self.session.received
                        evidence_record=measurement(self.model,segment['side'],center_q,
                            original['geometry']['center_check'])
                        write_json(Path(plan['directory'])/'center_check_result.json',
                            dict(evidence_record,settled=False,feedback_age_s=center_age,source='action_completed'))
                    TaskNode.wait_stationary(self,ticket,expected=segment['end'],guard=lambda:hold_guard(True),
                        context=segment['label']+' endpoint',settling=True,tolerance_rad=endpoint_tolerance,
                        timeout=parameters.endpoint_wait_seconds)
                    checkpoint('segment_complete')
                    if is_center_check(plan) and segment['label']=='Center check':
                        with self.lock:
                            center_q=dict(self.session.current)
                            center_age=self.session.clock()-self.session.received
                        evidence_record=measurement(self.model,segment['side'],center_q,
                            original['geometry']['center_check'])
                        write_json(Path(plan['directory'])/'center_check_result.json',
                            dict(evidence_record,settled=True,feedback_age_s=center_age,source='segment_complete'))
                if stage_control:
                    TaskNode.apply_task_stage(self,segment['side'],'normal',parameters,ticket,epoch)
                (Path(plan['directory'])/'execution.json').write_text(json.dumps(evidence,indent=2)+'\n')
                with self.lock:
                    self.session.check(ticket); self.session.invalidate(); self.session.phase='ARRIVED_HOLD'
                    proof=plan.get('test_range',{})
                    full_task=proof.get('first')==1 and proof.get('last')==7
                    self.status=('Center check complete; holding. FK offset XYZ (mm): '+
                                 ', '.join(f'{v:+.1f}' for v in evidence_record['fk_error_mm']) if is_center_check(plan) else
                                 f'D-ring {plan.get("sweep_direction", "")} task complete (1-7); holding.' if full_task else
                                 'Selected test range complete; holding.' if plan['kind']=='staged_test' else
                                 'Motion complete; holding. Old plan consumed.')
                    if plan['kind']=='screen':self.screen.completed(plan)
                    if plan['kind']=='rail':self.rail.completed(plan)
                    if plan['kind']=='dring_demo':demonstration_control(self,plan).completed(plan)
            except Exception as error:
                failure=dict(message=str(error),exception_type=type(error).__name__,traceback=traceback.format_exc(),
                    checkpoint=getattr(self,'execution_checkpoint',None),completed_evidence=evidence,
                    events_file=str(events_path),stop_requested=motion_started,
                    partial_hand_evidence=getattr(error,'hand_evidence',None),
                    hand_hold_confirmed=getattr(error,'hand_hold_confirmed',None),
                    hand_hold_error=getattr(error,'hand_hold_error',None))
                with self.lock:
                    failure['feedback_at_failure']=dict(age_s=self.session.clock()-self.session.received,
                        source_stamp_ns=self.session.source_stamp_ns,
                        rejection=self.session.feedback_rejection,status=self.status,
                        joints=dict(self.session.current))
                    failure['expected_endpoint']=segment.get('end') if segment else None
                    failure['feedback_transport']=getattr(self,'feedback_transport',{})
                    failure['planning_config']=original.get('planning_config')
                    failure['gateway_at_failure']=dict(self.gateway)
                if motion_started:self.stop()
                else:
                    with self.lock:
                        if self.session.ticket==ticket and not self.session.stopped:
                            self.session.plan=original;self.session.phase='PLAN_READY'
                write_json(Path(original['directory'])/'execution_failure.json',failure)
                raise
        try:self.launch_work(work)
        except Exception:
            with self.lock:
                if self.session.ticket==ticket and self.session.phase=='EXECUTION_PREFLIGHT':
                    self.session.phase='PLAN_READY'
            raise


class Window(QtWidgets.QWidget):
    def __init__(self,node):
        super().__init__(); self.node=node;self.waist_dirty=False
        self.setWindowTitle('G1 Task Console'); self.resize(720,860);self.setMinimumSize(620,720)
        self.setStyleSheet('''
            QWidget { font-size: 14px; color: #202428; }
            QFrame#statusFrame { background: #f1f3f4; border: 1px solid #c8cdd1; border-radius: 4px; }
            QLabel#modeLabel { font-weight: 700; padding: 6px 8px; background: #dfe4e7; border-radius: 3px; }
            QLabel.metricValue { font-weight: 600; color: #1f5f70; }
            QGroupBox { font-weight: 600; margin-top: 8px; padding-top: 8px; }
            QPushButton { min-height: 30px; padding: 3px 10px; }
            QTabWidget::pane { border: 1px solid #c8cdd1; }
            QProgressBar { min-height: 18px; text-align: center; border: 1px solid #aeb5ba; }
            QProgressBar::chunk { background: #2c7a7b; }
        ''')
        outer=QtWidgets.QVBoxLayout(self);outer.setContentsMargins(10,10,10,10);outer.setSpacing(7)
        frame=QtWidgets.QFrame();frame.setObjectName('statusFrame');header=QtWidgets.QVBoxLayout(frame)
        header.setContentsMargins(8,8,8,8);header.setSpacing(5)
        self.mode=QtWidgets.QLabel('DISCONNECTED - NO LIVE ROBOT DATA' if node.args.disconnected else
                                  ('HARDWARE - UNARMED' if node.args.hardware else 'READ ONLY - NO ARM OUTPUT'))
        self.mode.setObjectName('modeLabel');header.addWidget(self.mode)
        metrics=QtWidgets.QGridLayout();metrics.setHorizontalSpacing(12);header.addLayout(metrics)
        self.phase_value=self.add_metric(metrics,0,0,'Session')
        self.gateway_value=self.add_metric(metrics,0,2,'Gateway')
        self.state_value=self.add_metric(metrics,0,4,'State age')
        self.waist_value=self.add_metric(metrics,1,0,'Waist PID / I torque')
        self.tracking_value=self.add_metric(metrics,1,2,'Tracking')
        self.temperature_value=self.add_metric(metrics,1,4,'Temperature')
        self.status=QtWidgets.QLabel();self.status.setWordWrap(True);self.status.setMinimumHeight(38)
        self.status.setMaximumHeight(64);header.addWidget(self.status)
        self.applied_speed=QtWidgets.QLabel();self.applied_speed.setWordWrap(True)
        self.applied_speed.hide();header.addWidget(self.applied_speed)
        self.reset_latch_button=self.add_button(header,'检查状态并解除锁存 / STOP',
            lambda:node.launch_work(node.reset_latch),enabled=False)
        self.reset_latch_button.setIcon(self.style().standardIcon(QtWidgets.QStyle.SP_BrowserReload))
        self.reset_latch_button.setToolTip('由网关检查当前状态；正常才解除锁存，保持未 ARM，不恢复旧运动')
        self.progress=QtWidgets.QProgressBar();self.progress.setFixedHeight(22)
        self.progress.setTextVisible(True);self.progress.hide();header.addWidget(self.progress)
        outer.addWidget(frame)

        self.tabs=QtWidgets.QTabWidget();outer.addWidget(self.tabs,1)
        plan_page=QtWidgets.QScrollArea();plan_page.setWidgetResizable(True)
        plan_page.setFrameShape(QtWidgets.QFrame.NoFrame)
        plan_content=QtWidgets.QWidget();layout=QtWidgets.QVBoxLayout(plan_content);layout.setSpacing(6)
        layout.setSizeConstraint(QtWidgets.QLayout.SetMinimumSize);plan_page.setWidget(plan_content)
        self.tabs.addTab(plan_page,'Plan / session')
        motion=QtWidgets.QGroupBox('Motion setup');motion_layout=QtWidgets.QGridLayout(motion)
        self.side=QtWidgets.QComboBox();self.side.addItems(['left','right','both'])
        self.velocity=QtWidgets.QDoubleSpinBox(); self.acceleration=QtWidgets.QDoubleSpinBox()
        for widget in (self.velocity,self.acceleration):
            widget.setRange(.05,1.); widget.setSingleStep(.05); widget.setValue(.15)
        motion_layout.addWidget(QtWidgets.QLabel('执行手臂'),0,0);motion_layout.addWidget(self.side,0,1)
        self.side.setToolTip('left = 左臂；right = 右臂；both 仅用于双臂准备。拨环方向单独选择。')
        motion_layout.addWidget(QtWidgets.QLabel('Velocity limit'),0,2);motion_layout.addWidget(self.velocity,0,3)
        motion_layout.addWidget(QtWidgets.QLabel('Acceleration limit'),1,0);motion_layout.addWidget(self.acceleration,1,1)
        from task_speed_control import SpeedInput
        self.trajectory_speed=SpeedInput()
        motion_layout.addWidget(QtWidgets.QLabel('Requested speed (x)'),1,2)
        motion_layout.addWidget(self.trajectory_speed,1,3)
        self.retime_button=QtWidgets.QPushButton('Apply to planned path')
        self.retime_button.setIcon(self.style().standardIcon(QtWidgets.QStyle.SP_BrowserReload))
        self.retime_button.clicked.connect(lambda:self.invoke(lambda:node.retime_plan(self.trajectory_speed.value())))
        motion_layout.addWidget(self.retime_button,2,0,1,3)
        self.use_maximum_button=QtWidgets.QToolButton()
        self.use_maximum_button.setIcon(self.style().standardIcon(QtWidgets.QStyle.SP_MediaSeekForward))
        self.use_maximum_button.setToolTip('设为各段最快已验证倍率')
        self.use_maximum_button.setAccessibleName('Use maximum trajectory multiplier')
        self.use_maximum_button.clicked.connect(lambda:self.invoke(self.use_maximum_speed))
        motion_layout.addWidget(self.use_maximum_button,2,3)
        self.maximum_speed=QtWidgets.QLabel('');self.maximum_speed.setWordWrap(True)
        motion_layout.addWidget(self.maximum_speed,3,0,1,4)
        motion_layout.setColumnStretch(1,1);motion_layout.setColumnStretch(3,1)
        layout.addWidget(motion)

        from task_dring_hand_settings import load as load_hand_settings
        hand_settings=load_hand_settings()
        fingers=QtWidgets.QGroupBox('D-ring 手型 / 下次运行');finger_layout=QtWidgets.QGridLayout(fingers)
        self.index_openness=QtWidgets.QDoubleSpinBox();self.middle_openness=QtWidgets.QDoubleSpinBox()
        for column,(label,control,key) in enumerate((('食指开合',self.index_openness,'index_q'),
                                                    ('中指开合',self.middle_openness,'middle_q'))):
            control.setRange(0.,1.);control.setSingleStep(.05);control.setDecimals(2)
            control.setValue(hand_settings[key]);control.setToolTip('0 = 弯曲；1 = 伸直。归一化执行器值，不是角度。')
            finger_layout.addWidget(QtWidgets.QLabel(label),0,column*2)
            finger_layout.addWidget(control,0,column*2+1)
        self.save_fingers=QtWidgets.QPushButton('保存下次手型')
        self.save_fingers.setIcon(self.style().standardIcon(QtWidgets.QStyle.SP_DialogSaveButton))
        self.save_fingers.clicked.connect(lambda:self.invoke(self.save_hand_settings))
        finger_layout.addWidget(self.save_fingers,1,0,1,2)
        self.finger_settings_status=QtWidgets.QLabel('已保存');self.finger_settings_status.setWordWrap(True)
        finger_layout.addWidget(self.finger_settings_status,1,2,1,2)
        for control in (self.index_openness,self.middle_openness):
            control.valueChanged.connect(lambda _:self.finger_settings_status.setText('未保存'))
        layout.addWidget(fingers)

        session=QtWidgets.QGroupBox('Session control');session_layout=QtWidgets.QVBoxLayout(session)
        self.r2a=QtWidgets.QCheckBox('R2+A active; current pose is the session startup');session_layout.addWidget(self.r2a)
        row=QtWidgets.QHBoxLayout();session_layout.addLayout(row)
        self.capture_button=self.add_button(row,'Capture startup',lambda:self.require(self.r2a,node.home))
        self.arm_button=self.add_button(row,'ARM - take control',lambda:node.launch_work(node.arm),enabled=node.args.hardware)
        self.capture_confirmation_mode=None
        row=QtWidgets.QHBoxLayout();session_layout.addLayout(row)
        self.add_button(row,'Record LEFT ready',lambda:node.record_ready('left'))
        self.add_button(row,'Record RIGHT ready',lambda:node.record_ready('right'))
        layout.addWidget(session)

        self.plan_buttons=[]
        self.hand_posture=QtWidgets.QCheckBox('Both hands: index extended, other fingers folded (manual check)')
        if node.hand_control:
            self.hand_posture.setText('Actual fingers match the approximate hand model (DDS enabled)')
        hand_row=QtWidgets.QHBoxLayout();hand_row.addWidget(self.hand_posture,1)
        if node.hand_control:self.add_button(hand_row,'Read hand DDS state',lambda:node.launch_work(lambda:node.read_hands()))
        layout.addLayout(hand_row)

        planning=QtWidgets.QGroupBox('Planning');planning_layout=QtWidgets.QVBoxLayout(planning)
        self.config_status=QtWidgets.QLabel()
        self.config_status.setWordWrap(True)
        self.config_status.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
        test=QtWidgets.QGroupBox('Saved trajectories');test_layout=QtWidgets.QVBoxLayout(test)
        test_layout.setSizeConstraint(QtWidgets.QLayout.SetMinimumSize)
        saved_page=QtWidgets.QScrollArea();saved_page.setWidgetResizable(True)
        saved_page.setFrameShape(QtWidgets.QFrame.NoFrame)
        saved_content=QtWidgets.QWidget();saved_layout=QtWidgets.QVBoxLayout(saved_content)
        saved_layout.setSizeConstraint(QtWidgets.QLayout.SetMinimumSize)
        saved_layout.addWidget(test);saved_layout.addStretch(1);saved_page.setWidget(saved_content)
        self.tabs.addTab(saved_page,'History / RViz')
        self.saved_path=QtWidgets.QLineEdit(getattr(node.args,'saved_test_archive',None) or '')
        self.saved_path.setReadOnly(True)
        path_row=QtWidgets.QHBoxLayout();path_row.addWidget(self.saved_path,1);test_layout.addLayout(path_row)
        self.open_archive_button=self.add_button(path_row,'Open archive',self.choose_archive)
        self.open_archive_button.setText('')
        self.open_archive_button.setIcon(self.style().standardIcon(QtWidgets.QStyle.SP_DirOpenIcon))
        self.open_archive_button.setToolTip('Open saved task archive')
        self.open_archive_button.setAccessibleName('Open saved task archive')
        self.open_archive_button.setFixedSize(32,28)
        library_row=QtWidgets.QHBoxLayout();test_layout.addLayout(library_row)
        self.library_refresh=self.add_button(library_row,'',self.refresh_library)
        self.library_refresh.setIcon(self.style().standardIcon(QtWidgets.QStyle.SP_BrowserReload))
        self.library_refresh.setToolTip('Refresh offline trajectory library')
        self.library_refresh.setAccessibleName('Refresh offline trajectory library')
        self.library_refresh.setFixedSize(32,28)
        self.library_preview=self.add_button(library_row,'',self.preview_library_archive)
        self.library_preview.setIcon(self.style().standardIcon(QtWidgets.QStyle.SP_MediaPlay))
        self.library_preview.setToolTip('Open isolated RViz playback')
        self.library_preview.setAccessibleName('Open isolated RViz playback')
        self.library_preview.setFixedSize(32,28)
        self.library_summary=QtWidgets.QLabel();library_row.addWidget(self.library_summary,1)
        self.archive_scene=QtWidgets.QComboBox()
        self.archive_scene.addItems(['Current RGB-D','Archived RGB-D'])
        library_row.addWidget(self.archive_scene)
        speed_row=QtWidgets.QHBoxLayout();test_layout.addLayout(speed_row)
        self.archive_speed=SpeedInput()
        self.archive_speed.textChanged.connect(self.trajectory_speed.setText)
        self.trajectory_speed.textChanged.connect(self.archive_speed.setText)
        speed_row.addWidget(QtWidgets.QLabel('Requested speed (x)'));speed_row.addWidget(self.archive_speed)
        speed_row.addStretch(1)
        self.preview_results=SimpleQueue()
        self.library_table=QtWidgets.QTableWidget(0,6)
        self.library_table.setHorizontalHeaderLabels(['Capture','Arm / sweep','Start waist','Original duration','Solve','Position error'])
        self.library_table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.library_table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        self.library_table.setSelectionMode(QtWidgets.QAbstractItemView.SingleSelection)
        self.library_table.verticalHeader().hide()
        self.library_table.setMinimumHeight(160)
        self.library_table.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.ResizeToContents)
        self.library_table.horizontalHeader().setSectionResizeMode(0,QtWidgets.QHeaderView.Stretch)
        test_layout.addWidget(self.library_table)
        self.library_entries=[];self.library_preview_process=None
        self.library_preview_log=None;self.library_preview_exit_seen=False
        self.library_results=SimpleQueue();self.library_scan_thread=None
        self.last_auto_archive=None
        self.library_table.itemSelectionChanged.connect(self.select_library_archive)
        range_row=QtWidgets.QHBoxLayout();test_layout.addLayout(range_row)
        self.first_stage=QtWidgets.QComboBox();self.last_stage=QtWidgets.QComboBox()
        from task_saved_test import LABELS
        for number,label in enumerate(LABELS,1):
            if number==4:label='Sweep 120'
            self.first_stage.addItem(f'{number}. {label}',number)
            self.last_stage.addItem(f'{number}. {label}',number)
        if getattr(node.args,'dring_contact_test',False) and getattr(node.args,'saved_test_archive',None):
            self.last_stage.setCurrentIndex(len(LABELS)-1)
        range_row.addWidget(QtWidgets.QLabel('From'));range_row.addWidget(self.first_stage,1)
        range_row.addWidget(QtWidgets.QLabel('Through'));range_row.addWidget(self.last_stage,1)
        self.test_scene=QtWidgets.QCheckBox('Saved RGB-D scene unchanged; base locked')
        self.test_contact=QtWidgets.QCheckBox('Contact test: actual finger/target alignment checked')
        test_layout.addWidget(self.test_scene);test_layout.addWidget(self.test_contact)
        self.test_load=self.add_button(test_layout,'Validate hardware test range',self.load_test)
        self.collision_cache=QtWidgets.QCheckBox('Remember / reuse verified collision checks')
        self.collision_cache.setToolTip('Exact stages and timing only; changed trajectory/model/checker invalidates the cache')
        self.collision_cache.toggled.connect(lambda checked:self.invoke(self.node.remember_saved_collision_check) if checked else None)
        test_layout.addWidget(self.collision_cache)
        self.plan_buttons.append(self.test_load)
        self.test_summary=QtWidgets.QLabel('No saved range loaded')
        self.test_summary.setWordWrap(True);test_layout.addWidget(self.test_summary)
        self.plan_buttons.append(self.add_button(planning_layout,'Plan above-table preparation',lambda:node.plan(
            'prepare',self.side.currentText(),self.velocity.value(),self.acceleration.value(),
            hand_posture_confirmed=self.hand_posture.isChecked())))
        sweep_row=QtWidgets.QHBoxLayout();planning_layout.addLayout(sweep_row)
        self.task_arm=QtWidgets.QComboBox()
        self.task_arm.addItems(['左臂执行','右臂执行','双臂（仅准备）'])
        self.task_arm.setCurrentIndex(self.side.currentIndex())
        self.task_arm.currentIndexChanged.connect(self.side.setCurrentIndex)
        self.side.currentIndexChanged.connect(self.task_arm.setCurrentIndex)
        sweep_row.addWidget(self.task_arm)
        self.task_plan_buttons={}
        for direction in ('left','right'):
            button=self.add_button(sweep_row,'规划'+('左拨' if direction=='left' else '右拨')+' D环',
                lambda d=direction:node.plan('task',self.side.currentText(),
                    self.velocity.value(),self.acceleration.value(),
                    hand_posture_confirmed=self.hand_posture.isChecked(),sweep_direction=d,
                    speed_multiplier=self.trajectory_speed.value()))
            self.task_plan_buttons[direction]=button
            self.plan_buttons.append(button)
        self.plan_buttons.append(self.add_button(planning_layout,'Plan recovery',lambda:node.plan(
            'recover','both',self.velocity.value(),self.acceleration.value(),
            hand_posture_confirmed=self.hand_posture.isChecked())))
        self.center_check_button=self.add_button(planning_layout,'Plan center check',lambda:node.plan(
            'task',self.side.currentText(),self.velocity.value(),self.acceleration.value(),
            hand_posture_confirmed=self.hand_posture.isChecked(),center_check=True,
            speed_multiplier=self.trajectory_speed.value()))
        self.plan_buttons.append(self.center_check_button)
        self.calibration_tab=task_calibration.panel(self)
        self.tabs.addTab(self.calibration_tab,'A/B/C 调参')
        if hasattr(node, 'screen'):
            from task_screen_panel import ScreenPanel
            self.screen_panel = ScreenPanel(self)
            self.tabs.addTab(self.screen_panel, '屏幕示教')
        if hasattr(node, 'rail'):
            from task_rail_panel import RailPanel
            self.rail_panel = RailPanel(self)
            self.tabs.addTab(self.rail_panel, '滑轨推拉')
        if hasattr(node,'demo'):
            from task_dring_demo_panel import DringDemoPanel
            self.demo_panel=DringDemoPanel(self)
            self.tabs.addTab(self.demo_panel,'拉环示教')
            self.tabs.setCurrentWidget(self.demo_panel)
        if hasattr(node,'landing_gear'):
            from task_landing_gear_panel import LandingGearPanel
            self.landing_gear_panel=LandingGearPanel(self)
            self.tabs.addTab(self.landing_gear_panel,'起落架杆')
        layout.addWidget(planning)

        waist=QtWidgets.QGroupBox('Waist tuning');waist_layout=QtWidgets.QGridLayout(waist)
        self.waist_kp=QtWidgets.QDoubleSpinBox();self.waist_kp.setRange(20.,80.);self.waist_kp.setSingleStep(5.);self.waist_kp.setValue(60.)
        self.waist_kd=QtWidgets.QDoubleSpinBox();self.waist_kd.setRange(.5,5.);self.waist_kd.setSingleStep(.25);self.waist_kd.setValue(4.5)
        self.waist_ki=QtWidgets.QDoubleSpinBox();self.waist_ki.setRange(0.,30.);self.waist_ki.setSingleStep(2.5);self.waist_ki.setValue(20.)
        self.waist_i_limit=QtWidgets.QDoubleSpinBox();self.waist_i_limit.setRange(.1,2.);self.waist_i_limit.setSingleStep(.1);self.waist_i_limit.setValue(1.2);self.waist_i_limit.setSuffix(' Nm')
        waist_layout.addWidget(QtWidgets.QLabel('Kp'),0,0);waist_layout.addWidget(self.waist_kp,0,1)
        waist_layout.addWidget(QtWidgets.QLabel('Kd'),0,2);waist_layout.addWidget(self.waist_kd,0,3)
        waist_layout.addWidget(QtWidgets.QLabel('Ki'),1,0);waist_layout.addWidget(self.waist_ki,1,1)
        waist_layout.addWidget(QtWidgets.QLabel('I limit'),1,2);waist_layout.addWidget(self.waist_i_limit,1,3)
        self.waist_confirm=QtWidgets.QCheckBox('Waist unloaded and stationary');waist_layout.addWidget(self.waist_confirm,2,0,1,2)
        self.waist_apply=QtWidgets.QPushButton('Apply gains atomically');self.waist_apply.setEnabled(False)
        self.waist_apply.clicked.connect(lambda:self.invoke(self.apply_waist))
        waist_layout.addWidget(self.waist_apply,2,2,1,2)
        layout.addWidget(waist);waist.setVisible(node.planner.waist_assist)
        layout.addStretch(1)

        outer.addWidget(self.config_status)
        self.clear=QtWidgets.QCheckBox('Path clear; base locked; review matches actual scene')
        outer.addWidget(self.clear)
        self.dring_contact=QtWidgets.QCheckBox('D-ring contact test: alignment checked, STOP accessible')
        self.dring_contact.setToolTip('Commissioning test, not certified operation. Selected hand/base and arm/body checks only; '
            'table, other arm and D-ring are not fully collision-validated. Contact remains position-controlled.')
        self.dring_contact.setVisible(bool(getattr(node.args,'dring_contact_test',False)))
        outer.addWidget(self.dring_contact)
        self.execute_button=self.add_button(outer,'Confirm and execute',lambda:self.require(self.clear,
            lambda:node.execute(scene_confirmed=True,contact_confirmed=self.dring_contact.isChecked())))
        self.execute_reason=QtWidgets.QLabel(); self.execute_reason.setWordWrap(True)
        outer.addWidget(self.execute_reason)
        stop=self.add_button(outer,'STOP TASK / HOLD THEN ADMITTANCE',node.stop)
        stop.setMinimumHeight(64); stop.setStyleSheet('background:#a52222;color:white;font-weight:bold;font-size:16px;')
        self.waist_admittance_button=self.add_button(outer,'双臂＋腰 yaw 导纳（当前静止保持 → 手动拖动）',
            lambda: self.confirm_waist_admittance(), enabled=node.args.hardware)
        self.waist_admittance_status=QtWidgets.QLabel('腰 yaw 导纳：未确认'); outer.addWidget(self.waist_admittance_status)
        self.free=QtWidgets.QCheckBox('Finger disengaged; arms manually returned near startup'); outer.addWidget(self.free)
        self.add_button(outer,'Recovery: leave admittance and hold',lambda:self.require(
            self.free,lambda:node.recovery(self.velocity.value(),self.acceleration.value())),enabled=node.args.hardware)
        self.timer=QtCore.QTimer(self); self.timer.timeout.connect(self.refresh); self.timer.start(150)
        self.last_status_publish=0.
        self.confirmed_plan_generation=None
        for control in (self.side,self.velocity,self.acceleration):
            (control.currentTextChanged if control is self.side else control.valueChanged).connect(self.invalidate)
        self.trajectory_speed.valueChanged.connect(self.speed_changed)
        for control in (self.waist_kp,self.waist_kd,self.waist_ki,self.waist_i_limit):
            control.valueChanged.connect(self.waist_changed)
        self.hand_posture.toggled.connect(self.invalidate)
        for selector in (self.first_stage,self.last_stage):selector.currentIndexChanged.connect(self.test_selection_changed)
        for checkbox in (self.test_scene,self.test_contact):checkbox.toggled.connect(self.invalidate_test)
        self.refresh_library()
        from task_depth_reference_panel import DepthReferencePanel
        self.depth_reference=DepthReferencePanel(node)
        self.tabs.addTab(self.depth_reference,'历史深度参考')
        from task_editor_settings import ControlEditorSettings
        self.edit_preferences=ControlEditorSettings(self)

    def add_metric(self,layout,row,column,label):
        title=QtWidgets.QLabel(label);value=QtWidgets.QLabel('-');value.setProperty('class','metricValue')
        title.setWordWrap(True)
        title.setStyleSheet('font-size:12px;');value.setStyleSheet('font-size:12px;')
        value.setMinimumWidth(42)
        layout.addWidget(title,row,column);layout.addWidget(value,row,column+1)
        return value

    def waist_changed(self):
        self.waist_dirty=True

    def apply_waist(self):
        kp,kd=self.waist_kp.value(),self.waist_kd.value()
        ki,integral_limit=self.waist_ki.value(),self.waist_i_limit.value()
        self.require(self.waist_confirm,lambda:self.node.launch_work(
            lambda:self.node.set_waist_gains(kp,kd,ki,integral_limit)))

    def choose_archive(self):
        directory=QtWidgets.QFileDialog.getExistingDirectory(self,'Saved task archive',
            self.saved_path.text() or str(ROOT/'data/task_commissioning'))
        if directory:
            self.saved_path.setText(directory)
            self.test_scene.setChecked(False);self.test_contact.setChecked(False)
            self.invalidate_test()

    def refresh_library(self):
        from task_trajectory_library import catalog,DEFAULT_LIBRARY
        if self.library_scan_thread is not None and self.library_scan_thread.is_alive():return
        root=getattr(self.node.args,'trajectory_library',None) or DEFAULT_LIBRARY
        results=self.library_results
        self.library_summary.setText('Checking archives...')
        def scan():
            try:entries,errors=catalog(root)
            except Exception as error:entries,errors=[],[dict(directory=str(root),error=str(error))]
            results.put((entries,errors,root))
        self.library_scan_thread=threading.Thread(target=scan,name='archive-catalog',daemon=True)
        self.library_scan_thread.start()

    def poll_library(self):
        try:entries,errors,root=self.library_results.get_nowait()
        except Empty:return
        self.library_table.blockSignals(True);self.library_table.setRowCount(len(entries))
        self.library_entries=entries
        for row,entry in enumerate(entries):
            source=Path(entry['source_capture'])
            elapsed=entry['planning_seconds']
            error=entry.get('max_position_error_m')
            values=(source.parent.name+'/'+source.name,entry['arm']+' / '+entry['direction'],
                    f'{entry["waist_degrees"]:.1f} deg',f'{entry["trajectory_duration_s"]:.1f} s',
                    '-' if elapsed is None else f'{elapsed:.1f} s',
                    '-' if error is None else f'{error*1000:.2f} mm')
            for column,value in enumerate(values):
                item=QtWidgets.QTableWidgetItem(value)
                item.setToolTip(json.dumps({k:entry[k] for k in ('pair_id','source_capture','model_sha256','start')},indent=2))
                self.library_table.setItem(row,column,item)
        self.library_table.blockSignals(False)
        self.library_summary.setText(f'{len(entries)} verified archives; {len(errors)} rejected')
        self.library_summary.setToolTip(json.dumps(errors,indent=2) if errors else str(root))

    def select_library_archive(self):
        row=self.library_table.currentRow()
        if not 0<=row<len(self.library_entries):return
        entry=self.library_entries[row]
        self.saved_path.setText(entry['directory'])
        self.test_scene.setChecked(False);self.test_contact.setChecked(False)
        self.first_stage.setCurrentIndex(0);self.last_stage.setCurrentIndex(6)
        self.invalidate_test()

    def preview_library_archive(self):
        if not self.saved_path.text():raise RuntimeError('Select an archive first')
        if self.library_preview_process is not None and self.library_preview_process.poll() is None:
            raise RuntimeError('Close the existing offline playback window before opening another archive')
        with self.node.lock:
            if (self.node.session.phase in ('PLANNING','EXECUTING','EXECUTION_PREFLIGHT') or
                    self.node.worker is not None and self.node.worker.is_alive()):
                raise RuntimeError('Wait for the current operation to finish')
        options=(self.saved_path.text(),self.trajectory_speed.value(),self.velocity.value(),self.acceleration.value())
        if self.archive_scene.currentIndex()==0:
            if self.side.currentText() not in ('left','right'):raise ValueError('Select one arm')
            self.node.capture_archive_scene(self.side.currentText(),
                lambda path,generation,error:self.preview_results.put((options,path,generation,error)))
        else:self.launch_archive_preview(options)

    def poll_archive_preview(self):
        try:options,path,generation,error=self.preview_results.get_nowait()
        except Empty:return
        if error:
            self.node.status=error
            return
        with self.node.lock:
            if self.node.session.generation!=generation or self.node.session.phase!='HOME_CAPTURED':return
        try:current=(self.saved_path.text(),self.trajectory_speed.value(),self.velocity.value(),self.acceleration.value())
        except ValueError:return
        if options!=current:return
        self.invoke(lambda:self.launch_archive_preview(options,path))

    def launch_archive_preview(self,options,scene=None):
        archive,speed,velocity,acceleration=options
        output=ROOT/'data/offline_archive_previews'/datetime.now().strftime('%Y%m%d_%H%M%S_%f')
        output.parent.mkdir(parents=True,exist_ok=True)
        parameters,config=load_parameters(getattr(self.node.args,'planning_config',DEFAULT_CONFIG))
        with self.node.lock:
            parameters,config=controller_bounded_parameters(parameters,config,self.node.gateway)
        limits_file=output.with_suffix('.motion.json')
        write_json(limits_file,config['values'])
        from task_dring_hand_settings import load as load_hand_settings
        hand_file=output.with_suffix('.hand.json');write_json(hand_file,load_hand_settings())
        command=['bash',str(ROOT/'scripts/run_task_offline_gallery.sh'),'--archives',archive,
                 '--output',str(output),'--trajectory-speed',str(speed),
                 '--timing-policy','guarded','--motion-parameters',str(limits_file),'--hand-settings',str(hand_file),
                 '--velocity-limit-scaling',str(velocity),'--acceleration-limit-scaling',str(acceleration)]
        if scene is not None:command+=['--scene-capture',str(scene),'--scene-model',str(self.node.args.robot_urdf)]
        self.library_preview_log=output.with_suffix('.log');self.library_preview_exit_seen=False
        with output.with_suffix('.log').open('w') as log:
            self.library_preview_process=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,
                start_new_session=True)

    def test_selection_changed(self):
        self.test_contact.setChecked(False)
        self.invalidate_test()

    def invalidate_test(self):
        with self.node.lock:
            ticket=self.node.session.ticket
            if ticket and ticket.kind=='staged_test' and self.node.session.phase!='EXECUTING':
                self.invalidate()

    def load_test(self):
        if not self.saved_path.text():raise RuntimeError('Select a saved archive directory')
        self.node.load_saved_test(self.saved_path.text(),self.first_stage.currentData(),self.last_stage.currentData(),
            self.velocity.value(),self.acceleration.value(),scene_confirmed=self.test_scene.isChecked(),
            contact_confirmed=self.test_contact.isChecked(),hand_confirmed=self.hand_posture.isChecked(),
            selected_side=self.side.currentText(),speed_multiplier=self.trajectory_speed.value(),
            use_collision_cache=self.collision_cache.isChecked())

    def save_hand_settings(self):
        from task_dring_hand_settings import save
        save(dict(index_q=self.index_openness.value(),middle_q=self.middle_openness.value()))
        self.finger_settings_status.setText('已保存，下次规划/加载生效')

    def use_maximum_speed(self):
        with self.node.lock:
            session=self.node.session
            if (session.phase not in ('PLAN_READY','RETIME_REQUIRED') or not session.plan or
                    session.plan.get('kind')!='task' or session.plan.get('hardware_executable') is not False):
                raise RuntimeError('A completed live task plan is required')
            maximum=session.plan.get('speed_scaling',{}).get('maximum_multiplier')
        if maximum is None or not math.isfinite(maximum) or maximum<=0:
            raise RuntimeError('No finite speed maximum available')
        self.trajectory_speed.setValue(maximum)

    def add_button(self,layout,text,callback,enabled=True):
        button=QtWidgets.QPushButton(text); button.setEnabled(enabled)
        button.clicked.connect(lambda:self.invoke(callback)); layout.addWidget(button); return button

    def invoke(self,callback):
        try: callback()
        except Exception as e:
            with self.node.lock: self.node.status=str(e)

    def confirm_waist_admittance(self):
        answer=QtWidgets.QMessageBox.question(self,'确认双臂＋腰yaw导纳',
            '仅从静止保持进入。确认无实体接触、机器人支撑可靠、躯干与双臂净空？'
            '腰roll/pitch仍保持。等待TASK_ADMITTANCE及腰导纳确认后才轻推；'
            '首次小角度观察，异常停止。此功能不是急停，也不会返回旧姿态。',
            QtWidgets.QMessageBox.Yes|QtWidgets.QMessageBox.No,QtWidgets.QMessageBox.No)
        if answer==QtWidgets.QMessageBox.Yes:self.node.stop(include_waist=True)

    def require(self,checkbox,callback):
        if not checkbox.isChecked(): raise RuntimeError('Operator confirmation required: '+checkbox.text())
        callback(); checkbox.setChecked(False)

    def invalidate(self):
        with self.node.lock:
            if self.node.session.phase=='EXECUTING': return
            self.node.session.invalidate(); self.node.session.phase='REPLAN_REQUIRED'
            self.node.display_pub.publish(DisplayTrajectory())
            self.node.status='Selection/scaling changed; previous plan invalidated.'

    def speed_changed(self):
        self.clear.setChecked(False);self.dring_contact.setChecked(False)
        with self.node.lock:
            session=self.node.session
            if (session.phase in ('PLAN_READY','RETIME_REQUIRED') and session.plan and
                    session.plan.get('kind')=='task' and session.plan.get('hardware_executable') is False):
                session.phase='RETIME_REQUIRED'
                self.node.display_pub.publish(DisplayTrajectory())
                self.node.status='Apply speed to the planned path and review again.'
                return
        self.invalidate()

    def refresh(self):
        if not rclpy.ok():
            self.timer.stop()
            QtWidgets.QApplication.quit()
            return
        self.poll_library()
        self.poll_archive_preview()
        process=self.library_preview_process
        if process is not None and not self.library_preview_exit_seen and process.poll() is not None:
            self.library_preview_exit_seen=True
            if process.returncode:
                detail=(self.library_preview_log.read_text()[-1000:] if self.library_preview_log and
                        self.library_preview_log.is_file() else 'No preview log')
                with self.node.lock:self.node.status='RViz preview failed: '+detail
        latest=getattr(self.node,'latest_archive',None)
        if (latest and latest!=self.last_auto_archive and
                not (self.library_scan_thread and self.library_scan_thread.is_alive())):
            self.last_auto_archive=latest
            self.saved_path.setText(latest)
            self.refresh_library()
        TaskNode.poll_planning_config(self.node)
        monitor=getattr(self.node,'planning_monitor',None)
        if monitor is not None:
            self.config_status.setText(monitor.summary(self.node.gateway))
            self.config_status.setToolTip(str(monitor.path)+'\n'+(monitor.execution_issue(self.node.gateway) or ''))
        with self.node.lock:
            shutdown_process=getattr(self.node,'shutdown_process',None)
            if shutdown_process is not None and shutdown_process.poll() is not None:
                output=shutdown_process.communicate()[0] or ''
                self.node.shutdown_process=None
                if shutdown_process.returncode:
                    self.node.shutdown_requested=False
                    self.node.shutdown_result=None
                    self.node.session.phase='REPLAN_REQUIRED'
                    self.node.status='Application close did not finish: '+output[-600:]
            shutdown_result=getattr(self.node,'shutdown_result',None)
            if shutdown_result is not None and shutdown_result.exists():
                result=json.loads(shutdown_result.read_text())
                self.node.shutdown_result=None
                self.node.shutdown_requested=False
                if result.get('success') is True:
                    QtWidgets.QApplication.quit()
                else:
                    self.node.session.phase='REPLAN_REQUIRED'
                    self.node.status='Application close did not finish: '+str(result.get('message','Unknown error'))
            s=self.node.session
            expired=s.expire_plan()
            if expired:
                self.node.status=expired
            # Session updates replace their containers. Snapshot references without
            # copying dense trajectories, and keep all Qt rendering off this lock.
            s=copy.copy(s)
            gateway_data=dict(self.node.gateway)
            gateway_received=self.node.gateway_received
            status=self.node.status
            worker_busy=self.node.worker is not None and self.node.worker.is_alive()
            busy=worker_busy or getattr(self.node,'shutdown_requested',False)
            self.calibration_tab.setEnabled(not busy and self.node.session.phase not in ('EXECUTING','EXECUTION_PREFLIGHT'))
            last_validation=getattr(self.node,'validation_progress',None)
            execution_checkpoint=getattr(self.node,'execution_checkpoint',None)
            guard_issue=None
            if s.plan and s.plan['kind'] in ('task','staged_test'):
                try:self.node.saved_test_guard(s.plan if s.plan['kind']=='staged_test' else None)
                except RuntimeError as error:guard_issue=str(error)
        if expired:self.node.display_pub.publish(DisplayTrajectory())
        if hasattr(self, 'screen_panel'):
            self.screen_panel.refresh(busy or s.phase in ('EXECUTION_PREFLIGHT','EXECUTING'))
        if hasattr(self, 'rail_panel'):
            self.rail_panel.refresh(busy or s.phase in ('EXECUTION_PREFLIGHT','EXECUTING'))
        if hasattr(self,'demo_panel'):
            self.demo_panel.refresh(busy or s.phase in ('EXECUTION_PREFLIGHT','EXECUTING'))
        if hasattr(self,'landing_gear_panel'):
            self.landing_gear_panel.refresh(busy or s.phase in ('EXECUTION_PREFLIGHT','EXECUTING'))
        generation=s.generation if s.phase in ('PLAN_READY','EXECUTION_PREFLIGHT') else None
        if generation != self.confirmed_plan_generation:
            self.clear.setChecked(False)
            self.dring_contact.setChecked(False)
            self.confirmed_plan_generation=generation
        age=time.monotonic()-s.received
        gateway_age=time.monotonic()-gateway_received
        gateway=gateway_data.get('phase','NO GATEWAY') if gateway_age<1.5 else 'GATEWAY STALE'
        phase=s.phase
        planning_elapsed=time.monotonic()-s.planning_at if phase=='PLANNING' else None
        self.phase_value.setText(phase+(f' ({planning_elapsed:.1f}s)' if planning_elapsed is not None else ''))
        self.gateway_value.setText(gateway);self.state_value.setText(f'{age:.2f}s')
        tracking=gateway_data.get('tracking_error_rad')
        temperature=gateway_data.get('max_motor_temperature_c')
        self.tracking_value.setText(f'{float(tracking):.4f} rad' if tracking is not None else '-')
        self.temperature_value.setText(f'{float(temperature):.0f} C' if temperature is not None else '-')
        gateway_kp=gateway_data.get('waist_kp');gateway_kd=gateway_data.get('waist_kd')
        gateway_ki=gateway_data.get('waist_ki');gateway_i_limit=gateway_data.get('waist_integral_torque_limit_nm')
        if None not in (gateway_kp,gateway_kd,gateway_ki,gateway_i_limit):
            kp,kd,ki,integral_limit=map(float,(gateway_kp,gateway_kd,gateway_ki,gateway_i_limit))
            integral_torque=float(gateway_data.get('waist_integral_torque_nm','0'))
            integral_mode=gateway_data.get('waist_integral_mode','disabled')
            self.waist_value.setText(f'{kp:.0f}/{kd:.1f}/{ki:.1f} | {integral_torque:+.2f} Nm')
            self.waist_value.setToolTip(f'I limit {integral_limit:.2f} Nm; {integral_mode}')
            if gateway_data.get('waist_control_mode') == 'official_pd' or gateway_data.get('waist_mode_switching') == 'true':
                actual_kp=float(gateway_data.get('waist_command_kp', '0'))
                actual_kd=float(gateway_data.get('waist_command_kd', '0'))
                self.waist_value.setText(f'PD {actual_kp:.0f}/{actual_kd:.1f}')
                self.waist_value.setToolTip('模式：'+gateway_data.get('waist_control_mode', '')+
                    ('；平滑切换中' if gateway_data.get('waist_mode_switching') == 'true' else '；以屏幕页网关状态为准'))
            if (not self.waist_dirty or
                    (abs(self.waist_kp.value()-kp)<1e-6 and abs(self.waist_kd.value()-kd)<1e-6 and
                     abs(self.waist_ki.value()-ki)<1e-6 and
                     abs(self.waist_i_limit.value()-integral_limit)<1e-6)):
                for widget,value in ((self.waist_kp,kp),(self.waist_kd,kd),
                                     (self.waist_ki,ki),(self.waist_i_limit,integral_limit)):
                    blocked=widget.blockSignals(True);widget.setValue(value);widget.blockSignals(blocked)
                self.waist_dirty=False
        else:self.waist_value.setText('-')
        pending_feedback=None
        if s.phase=='PLAN_READY':
            try:s.measured(stationary=True)
            except FeedbackPending:
                pending_feedback='Plan retained; waiting for fresh stationary robot feedback.'
            except RuntimeError as error:pending_feedback=str(error)
        self.status.setText(pending_feedback or status)
        scaling=(s.plan.get('speed_scaling') or s.plan.get('test_range',{}).get('speed_scaling')) if s.plan else None
        self.applied_speed.setVisible(bool(scaling and 'requested_multiplier' in scaling))
        if scaling and 'requested_multiplier' in scaling:
            actual='手臂等效' if scaling.get('timing_policy')=='local_path_v1' else '实际'
            self.applied_speed.setText(f'请求 {scaling["requested_multiplier"]:g}x / {actual} {scaling["applied_multiplier"]:.3f}x'+
                ('（受限）' if scaling.get('limited') else ''))
        maximum=scaling.get('maximum_multiplier') if scaling else None
        self.maximum_speed.setVisible(maximum is not None)
        if maximum is not None:
            limiting=scaling.get('limiting_constraint') or {}
            if scaling.get('timing_policy')=='local_path_v1':
                self.maximum_speed.setText(f'各段最快已验证：请求 {maximum:.3f}x | '+
                    f'手臂等效 {scaling["maximum_effective_multiplier"]:.3f}x | '+
                    f'预计 {scaling["minimum_duration_s"]:.2f} s')
            else:
                self.maximum_speed.setText(f'当前路径最大倍率 {maximum:.3f}x | '+
                    f'{limiting.get("label", "")} / {limiting.get("joint", "")} / {limiting.get("constraint", "")}'+
                    (f' | 预计 {scaling["minimum_duration_s"]:.2f} s' if scaling.get('minimum_duration_s') is not None else ''))
            self.maximum_speed.setToolTip('\n'.join(
                f'{stage["label"]}: {stage["maximum_multiplier"]:.3f}x'
                for stage in scaling.get('stage_limits',[]) if stage.get('maximum_multiplier') is not None))
        progress=re.search(r'validating trajectory\s+(\d+)/(\d+)',status)
        if last_validation and last_validation['generation']!=s.generation:
            last_validation=None
        if phase=='PLANNING':
            self.progress.show()
            if progress:
                value,total=map(int,progress.groups())
                if value==0:
                    if self.progress.maximum()!=0:self.progress.setRange(0,0)
                    self.progress.setToolTip(f'Batch validation starting: {total} samples; waiting for checked counts')
                else:
                    self.progress.setRange(0,total);self.progress.setValue(value)
                    self.progress.setFormat(f'Validation {value}/{total} (%p%)')
                    self.progress.setToolTip('Validated sample count')
            else:
                if self.progress.maximum()!=0:self.progress.setRange(0,0)
                self.progress.setFormat('Planning and validation')
        elif (phase=='PLAN_READY' and last_validation and
              last_validation['completed']==last_validation['total']):
            total=last_validation['total']
            self.progress.show();self.progress.setRange(0,total);self.progress.setValue(total)
            self.progress.setFormat(f'Validation complete: {total}/{total} (100%)')
            self.progress.setToolTip('Validated sample count')
        else:self.progress.hide()
        self.mode.setText('DISCONNECTED - NO LIVE ROBOT DATA' if self.node.args.disconnected else
                         ('HARDWARE - '+gateway if self.node.args.hardware else 'READ ONLY - NO ARM OUTPUT'))
        mode_color=('#f8d7da' if self.node.args.disconnected or gateway=='GATEWAY STALE' else
                    '#d7f0e3' if gateway=='ARMED_HOLD' else '#fff0c2' if gateway=='TASK_ADMITTANCE' else '#dfe4e7')
        self.mode.setStyleSheet(f'background:{mode_color};')
        if hasattr(self,'waist_admittance_button'):
            fresh=time.monotonic()-gateway_received<1.5
            self.waist_admittance_button.setEnabled(bool(self.node.args.hardware and not busy and fresh and
                gateway=='ARMED_HOLD' and gateway_data.get('task_waist_admittance_supported')=='true'))
            self.waist_admittance_status.setText('腰 yaw 导纳：'+('状态过期，勿拖动' if not fresh else
                '已启用' if gateway_data.get('task_waist_admittance_active')=='true' and gateway=='TASK_ADMITTANCE' else
                '尚未就绪，勿拖动'))
        holding=gateway_data.get('armed')=='true'
        self.reset_latch_button.setEnabled(bool(self.node.args.hardware and not self.node.args.disconnected and
            not busy and s.phase not in ('PLANNING','EXECUTION_PREFLIGHT','EXECUTING') and gateway_age<1.5 and
            (gateway=='LATCHED_SILENT' or gateway=='UNARMED' and
             (gateway_data.get('task_stop_active')=='true' or gateway_data.get('task_stop_requested')=='true')) and
            gateway_data.get('armed')=='false' and
            gateway_data.get('command_publisher_exists')=='false'))
        capture_mode='hold' if holding else 'startup'
        if capture_mode!=self.capture_confirmation_mode:
            self.r2a.setChecked(False)
            self.capture_confirmation_mode=capture_mode
        self.r2a.setText('Current stationary pose is the return point' if holding else
                         'R2+A active; current pose is the session startup')
        self.capture_button.setText('Return point captured' if s.home is not None else
                                    'Capture return point' if holding else 'Capture startup')
        capture_state=(gateway=='ARMED_HOLD' and holding and gateway_data.get('task_stop_active')=='false'
                       or gateway in ('UNARMED','LATCHED_SILENT') and gateway_data.get('armed')=='false')
        capture_allowed=(s.home is None and not busy and age<.15 and gateway_age<1.5 and capture_state and
                         gateway_data.get('mode_machine')=='6' and gateway_data.get('mode_pr')=='0' and
                         s.phase not in ('PLANNING','EXECUTION_PREFLIGHT','EXECUTING'))
        self.capture_button.setEnabled(capture_allowed)
        self.r2a.setEnabled(capture_allowed)
        self.arm_button.setEnabled(bool(self.node.args.hardware and not holding and
                                       not busy and s.home is not None and gateway=='UNARMED'))
        planning_issue='The previous operation is still finishing' if busy else s.planning_issue()
        for button in self.plan_buttons:
            issue=planning_issue
            if (button in self.task_plan_buttons.values() or button is self.center_check_button) and issue is None and self.side.currentText()=='both':
                issue='Select one arm for the D-ring task'
            if button is self.test_load and issue is None:
                if s.stopped:issue='Saved-range authorization requires controller hold'
                else:
                    with self.node.lock:
                        try:self.node.saved_test_guard()
                        except RuntimeError as error:issue=str(error)
            button.setEnabled(issue is None)
            button.setToolTip(issue or 'Plan from measured state; no motion commanded')
        executing=s.phase in ('EXECUTION_PREFLIGHT','EXECUTING')
        self.retime_button.setEnabled(not busy and s.phase in ('PLAN_READY','RETIME_REQUIRED') and
            bool(s.plan and s.plan.get('kind')=='task' and s.plan.get('hardware_executable') is False))
        self.use_maximum_button.setEnabled(self.retime_button.isEnabled() and maximum is not None)
        for control in (self.side,self.task_arm,self.velocity,self.acceleration,self.trajectory_speed): control.setEnabled(not executing and not busy)
        guarded_waist = (gateway_data.get('waist_control_mode', 'guarded') == 'guarded' and
                         gateway_data.get('waist_mode_switching') != 'true')
        for control in (self.waist_kp,self.waist_kd,self.waist_ki,self.waist_i_limit,
                        self.waist_confirm):control.setEnabled(not executing and guarded_waist)
        waist_phase=gateway_data.get('phase') in ('UNARMED','ARMED_HOLD')
        self.waist_apply.setEnabled(bool(self.node.args.hardware and not busy and waist_phase and guarded_waist and
                                         gateway_age<1.5 and s.phase!='EXECUTING' and
                                         self.waist_confirm.isChecked()))
        self.waist_apply.setToolTip('Apply through the gateway guarded parameter callback' if self.waist_apply.isEnabled()
                                    else 'Requires fresh UNARMED/ARMED_HOLD gateway state and stationary confirmation')
        for control in (self.first_stage,self.last_stage,self.test_scene,self.test_contact,self.open_archive_button,
                        self.library_table,self.library_refresh,self.library_preview,self.archive_scene,self.archive_speed,self.collision_cache):
            control.setEnabled(not executing and not busy)
        self.hand_posture.setEnabled(not executing)
        self.dring_contact.setEnabled(not busy and s.phase=='PLAN_READY' and bool(s.plan and s.plan['kind']=='task' and not is_center_check(s.plan)))
        if not self.node.args.hardware: reason='Read-only session: real execution is disabled'
        elif s.phase=='PLANNING': reason='Planning/validation is still running: '+status
        elif s.phase=='RETIME_REQUIRED': reason='Apply the requested preview speed before reviewing the path'
        elif s.phase=='EXECUTION_PREFLIGHT': reason='Execution authorization is being checked'
        elif busy: reason='The previous operation is still finishing'
        elif s.stopped: reason='Controller is stopped; planning is available, execution requires hold'
        elif pending_feedback: reason=pending_feedback
        elif s.plan is not None and s.plan.get('preview_only_timing'):
            reason='RViz-only time scaling; use a separately validated hardware test'
        elif s.plan is not None and s.plan['kind']=='task':
            if is_center_check(s.plan):
                reason=s.preview_issue(s.plan) or guard_issue
            elif not getattr(self.node.args,'dring_contact_test',False):
                reason='D-ring execution is not commissioned; preview only'
            elif not self.dring_contact.isChecked():
                reason='Confirm D-ring contact test: alignment checked and STOP accessible'
            else:
                reason=s.preview_issue(s.plan)
                if reason is None:reason=guard_issue
        elif s.plan is not None and s.plan['kind']=='screen':
            reason=s.execution_issue()
            if reason is None:
                try:self.node.screen.execution_guard(s.plan)
                except RuntimeError as error:reason=str(error)
        elif s.plan is not None and s.plan['kind']=='rail':
            reason=s.execution_issue()
            if reason is None:
                try:self.node.rail.execution_guard(s.plan)
                except RuntimeError as error:reason=str(error)
        elif s.plan is not None and s.plan['kind']=='dring_demo':
            reason=s.execution_issue()
            if reason is None:
                try:demonstration_control(self.node,s.plan).execution_guard(s.plan)
                except RuntimeError as error:reason=str(error)
        elif (s.plan is not None and s.plan.get('waist_assist') and
                gateway_data.get('task_waist_trajectory_supported')!='true'):
            reason='Waist-assisted execution requires a waist-enabled gateway; preview only'
        elif gateway!='ARMED_HOLD': reason='Controller must be ARMED_HOLD; current: '+gateway
        else: reason=s.execution_issue()
        test_plan=s.plan if s.plan and s.plan.get('kind')=='staged_test' else None
        if test_plan:
            proof=test_plan['test_range']
            if reason is None:reason=guard_issue
            alignment=('Align '+', '.join(proof['transition_arms']).upper()+' + ' if proof.get('transition_count') else '')
            self.execute_button.setText(f'Execute {alignment}{proof.get("arm", "").upper()} stages {proof["first"]}-{proof["last"]} only')
            scaling=proof.get('speed_scaling',{})
            speed_note=(f'Speed {scaling["applied_multiplier"]:.2f}x / requested {scaling["requested_multiplier"]:g}x; '
                        if scaling else '')
            if proof.get('transition_count'):speed_note+='Alignment collision: NOT CHECKED; '
            cached=test_plan.get('collision_validation',{}).get('cached_stages',[])
            if cached:speed_note+='Collision cache: '+','.join(map(str,cached))+'; '
            self.test_summary.setText(speed_note+f'Stages {proof["first"]}-{proof["last"]}: '+
                ' | '.join(f'{s["number"]} {s["label"]} {s["duration"]:.1f}s' for s in test_plan['segments']))
        else:
            direction=(s.plan.get('sweep_direction') or '').upper() if s.plan else ''
            planned_arm=(s.plan.get('arm') or (s.ticket.side if s.ticket else '')) if s.plan else ''
            self.execute_button.setText('Execute center check' if is_center_check(s.plan) else
                f'Execute {planned_arm.upper()} ARM / {direction} sweep (1-7)' if s.plan and s.plan['kind']=='task' and
                getattr(self.node.args,'dring_contact_test',False) else 'Confirm and execute')
            self.test_summary.setText('No validated saved range selected')
        if s.plan and s.plan['kind']=='screen':
            proof=s.plan['screen']
            self.execute_button.setText(f'执行腰部恢复到 {np.degrees(proof["target_waist_rad"]):.2f}°' if proof.get('operation')=='waist_recovery' else
                '执行移动到公共点 T' if proof.get('operation')=='return_T' else
                '执行 '+proof['selected']+(' 无接触循环' if proof['dry'] else ' 触点循环'))
        if s.plan and s.plan['kind']=='rail':
            proof=s.plan['rail']
            self.execute_button.setText('执行移动到'+proof['label'] if proof['operation']=='move' else
                '执行滑轨'+('推入' if proof['operation']=='push' else '拉出')+
                ('（推入后保持握持）' if proof['mode']=='hold' else '（推入后返回准备位）'))
        if s.plan and s.plan['kind']=='dring_demo':
            from task_dring_teach_data import SLOTS
            if s.plan['demo'].get('workflow')=='landing_gear':
                from task_landing_gear_data import SLOTS
            operation=s.plan['demo']['operation']
            label=({key:value+'录制回放' for key,value in SLOTS.items()} |
                   {'ready':'回录入准备点','preparation':'回桌上方准备位'})[operation]
            if s.plan['demo'].get('full_cycle'):label=('左臂' if s.plan['arm']=='left' else '右臂')+' '+label+'完整循环'
            self.execute_button.setText('执行 '+label+('（含起点衔接）' if s.plan['demo']['transition_count'] else ''))
        if reason is None and not self.clear.isChecked(): reason='Review the validated path and check Path clear'
        if reason is None and monitor is not None:reason=monitor.execution_issue(gateway_data)
        self.execute_button.setEnabled(reason is None)
        self.execute_button.setToolTip(reason or 'Execute this validated plan once')
        self.execute_reason.setText('Execute unavailable: '+reason if reason else 'Ready for confirmed execution')
        if time.monotonic()-self.last_status_publish >= .5:
            self.node.status_pub.publish(String(data=json.dumps({
                'phase':s.phase,'gateway':gateway,'message':pending_feedback or status,
                'generation':s.generation,'execute_block_reason':reason,
                'plan_block_reason':planning_issue,'startup_captured':s.home is not None,
                'planning_config_sha256':monitor.record['sha256'] if monitor and monitor.record else None,
                'planning_config_error':monitor.error if monitor else None,
                'planning_config_restart_fields':monitor.restart_fields(gateway_data) if monitor else [],
                'plan_kind':s.plan['kind'] if s.plan else None,
                'plan_arm':s.plan.get('arm',s.ticket.side if s.ticket else None) if s.plan else None,
                'plan_sweep_direction':s.plan.get('sweep_direction') if s.plan else None,
                'console_pid':os.getpid(),
                'session_dir':str(self.node.run_dir),
                'attached_handoff':str(self.node.args.attach_handoff) if getattr(self.node,'attachment_complete',False) else None,
                'worker_busy':worker_busy,'shutdown_requested':getattr(self.node,'shutdown_requested',False),
                'state_age_s':age,
                'feedback_transport':getattr(self.node,'feedback_transport',{}),
                'feedback_rejection':s.feedback_rejection,
                'tcp_a_controls':self.node.tcp_a_controls.status() if hasattr(self.node,'tcp_a_controls') else {},
                'rail_teaching':dict(source=self.node.rail.path,
                    stages=sorted(self.node.rail.config.get('traces',{})),
                    hands=sorted(self.node.rail.config.get('hands',{})),
                    status=self.node.rail.trace_status) if hasattr(self.node,'rail') else None,
                'execution_checkpoint':execution_checkpoint})))
            self.last_status_publish=time.monotonic()

    def closeEvent(self,event):
        self.edit_preferences.save()
        if getattr(self.node.args,'managed_lifecycle',False):
            event.ignore()
            with self.node.lock:
                if getattr(self.node,'shutdown_requested',False):return
                if ((self.node.worker and self.node.worker.is_alive()) or
                        self.node.session.phase in ('EXECUTION_PREFLIGHT','EXECUTING')):
                    self.node.status='Finish or STOP the current task before closing the application.'
                    return
                self.node.session.invalidate()
                self.node.session.phase='SHUTTING_DOWN'
                self.node.shutdown_requested=True
                self.node.status='Closing application: releasing control, then stopping controller and RViz.'
                # The shutdown helper must outlive the application cgroup it stops.
                from task_panel_control import ENV_KEYS
                operation=str(time.time_ns())
                self.node.shutdown_result=self.node.run_dir/f'shutdown_{operation}.json'
                command=['systemd-run','--user','--collect',
                         '--unit=g1-task-shutdown-'+operation+'.service',
                         *['--setenv='+k+'='+os.environ[k] for k in ENV_KEYS if k in os.environ],
                         '--','/bin/bash',str(ROOT/'scripts/stop_task_panel.sh'),
                         '--result-file',str(self.node.shutdown_result)]
                try:
                    self.node.shutdown_process=subprocess.Popen(command,stdout=subprocess.PIPE,
                        stderr=subprocess.STDOUT,text=True,start_new_session=True)
                except OSError as error:
                    self.node.shutdown_requested=False
                    self.node.shutdown_result=None
                    self.node.session.phase='REPLAN_REQUIRED'
                    self.node.status='Cannot start application shutdown: '+str(error)
            return
        with self.node.lock:
            if self.node.gateway.get('armed')=='true':
                self.node.stop()
                self.node.status='Task stopped. Controller remains supporting arms; use a deliberate shutdown after unloading.'
                event.ignore(); return
        self.node.closing.set(); event.accept()


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--network',default='eth0')
    parser.add_argument('--ssh',default='unitree@192.168.0.249')
    parser.add_argument('--vision-peer',default=os.environ.get('G1_VISION_DDS_PEER','192.168.123.164'))
    parser.add_argument('--intrinsics',default='/home/syf/unitree_ws/head_camera_intrinsics.json')
    parser.add_argument('--robot-urdf',required=True)
    parser.add_argument('--ready-poses')
    parser.add_argument('--hardware',action='store_true')
    parser.add_argument('--managed-lifecycle',action='store_true',help='Window close stops the complete managed application')
    parser.add_argument('--hand-control',action='store_true',help='Use HandClient and the articulated hand model')
    parser.add_argument('--waist-assist',action='store_true',help='Eight-joint task planning; execution requires matching gateway capability')
    parser.add_argument('--dring-profile',choices=('steep_straight','legacy'),default='steep_straight')
    parser.add_argument('--planning-config',type=Path,default=DEFAULT_CONFIG)
    parser.add_argument('--dring-contact-test',action='store_true',help='Allow explicit full live D-ring commissioning tests')
    parser.add_argument('--disconnected',action='store_true')
    parser.add_argument('--offline-editor',action='store_true')
    parser.add_argument('--attach-handoff',help='Checked local manifest for an existing supported controller')
    parser.add_argument('--saved-test-archive',help='Preselect an immutable archive; never loads or executes it automatically')
    parser.add_argument('--trajectory-library',type=Path,help='Directory containing immutable offline archives')
    args,ros=parser.parse_known_args()
    if args.offline_editor and (args.hardware or not args.disconnected or args.managed_lifecycle or args.attach_handoff):
        parser.error('Offline editor requires --disconnected and cannot use hardware or attachment')
    console_lock=acquire_console_lock()
    rclpy.init(args=ros); app=QtWidgets.QApplication([]); node=TaskNode(args)
    executor=MultiThreadedExecutor(num_threads=3); executor.add_node(node)
    from task_joint_feedback import JointFeedback
    feedback=None
    if not args.offline_editor:
        feedback=JointFeedback(node);feedback.start()
    thread=threading.Thread(target=executor.spin,daemon=True); thread.start()
    if args.offline_editor:
        import signal
        from task_editor_offline_panel import OfflineEditorWindow
        window=OfflineEditorWindow(node)
        for sig in (signal.SIGTERM,signal.SIGINT):
            signal.signal(sig,lambda *_:setattr(window,'close_requested',True))
    else:
        window=Window(node)
    window.show()
    if node.handoff:
        node.launch_work(node.finish_attachment)
    try: app.exec_()
    finally:
        node.closing.set()
        if node.worker: node.worker.join(timeout=50)
        node.hand.close()
        if feedback is not None:feedback.close()
        executor.shutdown(); thread.join(timeout=3); node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
        console_lock.close()


if __name__=='__main__': main()
