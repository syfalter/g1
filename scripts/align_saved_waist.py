"""Bounded waist-only alignment through the existing arm gateway; never ARM/reset."""
import argparse
from datetime import datetime
import json
import math
from pathlib import Path
import subprocess
import threading
import time

import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, DurabilityPolicy
from control_msgs.action import FollowJointTrajectory
from diagnostic_msgs.msg import DiagnosticArray
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from rosidl_runtime_py.convert import message_to_ordereddict

from task_session import Session, ARMS, ARM_NAMES
from task_console_handoff import supported_gateway
from task_hand_control import HandControl, model_joints
from task_saved_test import load_archive
from task_hook_collision import HandBaseCollision
from task_trajectory import samples, validate_waist_envelope
from task_plan_archive import write_json, verify_files, digest
from workbench_model import OfflineModel


def load_collision_context(directory, robot_urdf):
    """Import only checked geometry; no stored motion is returned or authorized."""
    directory = Path(directory)
    manifest = verify_files(directory)
    if digest(robot_urdf) != manifest['sha256']['context/robot.urdf']:
        raise ValueError('Live robot model differs from archived model')
    scene = json.loads((directory/'plan/plan.json').read_text())['contact_scene']
    base = scene.get('base_top_pelvis_m')
    if (scene.get('profile') != 'steep_straight' or
            scene.get('base_radius_m') != .025 or scene.get('base_height_m') != .02 or
            scene.get('arm_body_collision_checked') is not True or
            not isinstance(base, list) or len(base) != 3 or
            not all(type(v) in (int, float) and math.isfinite(v) for v in base)):
        raise ValueError('Unsupported archived collision geometry')
    return scene


def trajectory_for(joints, held, target):
    measured=joints['waist_yaw_joint']
    start=held['waist_yaw_joint']
    if not all(math.isfinite(v) for v in (target,measured,start)) or abs(target-measured)>.10:
        raise ValueError('Waist alignment is limited to 0.10 rad from measured state')
    if abs(start-measured)>.015:
        raise ValueError('Waist must settle within 0.015 rad of its held reference')
    delta=target-start
    if abs(delta)>.10:
        raise ValueError('Waist alignment is limited to 0.10 rad from held reference')
    duration=max(2.,1.875*abs(delta)/.06,math.sqrt(5.773503*abs(delta)/.12))
    trajectory=JointTrajectory(joint_names=['waist_yaw_joint',*ARMS['left']])
    count=math.ceil(duration/.005)+1
    for fraction in np.linspace(0,1,count):
        blend=fraction**3*(10-15*fraction+6*fraction**2)
        velocity=30*fraction**2*(1-fraction)**2/duration
        acceleration=60*fraction*(1-fraction)*(1-2*fraction)/duration**2
        point=JointTrajectoryPoint(
            positions=[float(start+delta*blend),*[held[n] for n in ARMS['left']]],
            velocities=[float(delta*velocity),*([0.]*7)],
            accelerations=[float(delta*acceleration),*([0.]*7)])
        ns=round(float(duration*fraction)*1e9)
        point.time_from_start.sec,point.time_from_start.nanosec=divmod(ns,1000000000)
        trajectory.points.append(point)
    return trajectory,duration


def arm_excursion_for(joints, held, delta):
    """Small elbow out/dwell/back probe, retaining the original hold references."""
    if not math.isfinite(delta) or not 0 < abs(delta) <= .10:
        raise ValueError('Arm excursion must be nonzero and at most 0.10 rad')
    names=ARMS['left'];start=np.array([held[n] for n in names])
    if not np.isfinite(start).all() or any(abs(joints[n]-held[n])>.015 for n in names):
        raise ValueError('Arm must settle within 0.015 rad of its held reference')
    offset=np.zeros(7);offset[3]=delta
    leg=max(2.,1.875*abs(delta)/.06,math.sqrt(5.773503*abs(delta)/.12))
    trajectory=JointTrajectory(joint_names=list(names))
    for begin,origin,change in ((0.,start,offset),(leg+1.,start+offset,-offset)):
        for fraction in np.linspace(0,1,math.ceil(leg/.005)+1):
            blend=fraction**3*(10-15*fraction+6*fraction**2)
            point=JointTrajectoryPoint(
                positions=list(origin+change*blend),
                velocities=list(change*30*fraction**2*(1-fraction)**2/leg),
                accelerations=list(change*60*fraction*(1-fraction)*(1-2*fraction)/leg**2))
            ns=round(float(begin+leg*fraction)*1e9)
            point.time_from_start.sec,point.time_from_start.nanosec=divmod(ns,1000000000)
            trajectory.points.append(point)
    return trajectory,2*leg+1.


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--archive',required=True)
    parser.add_argument('--robot-urdf',required=True)
    parser.add_argument('--network',default='eth0')
    parser.add_argument('--output-root',required=True,type=Path)
    parser.add_argument('--target-waist',type=float)
    parser.add_argument('--arm-excursion-rad',type=float,
                        help='Bounded left elbow out-and-back; waist reference unchanged')
    parser.add_argument('--collision-context-only',action='store_true',
                        help='Import geometry only; requires an explicit new waist target')
    parser.add_argument('--conservative-hands',action='store_true',
                        help='Waist-only free-space test with all-angle finger enclosures; no hand RPC')
    parser.add_argument('--execute',action='store_true')
    args=parser.parse_args()
    if args.arm_excursion_rad is not None and (not args.collision_context_only or
            not args.conservative_hands or args.target_waist is not None):
        parser.error('Arm probe requires collision-context-only, conservative-hands, and no waist target')
    if args.conservative_hands and not args.collision_context_only:
        parser.error('--conservative-hands requires --collision-context-only')
    if args.collision_context_only:
        if args.target_waist is None and args.arm_excursion_rad is None:
            parser.error('--collision-context-only requires --target-waist')
        scene=load_collision_context(args.archive,args.robot_urdf)
        target=args.target_waist
    else:
        archive=load_archive(args.archive,args.robot_urdf)
        scene=archive['plan']['contact_scene']
        target=(archive['stages'][0]['end']['waist_yaw_joint'] if args.target_waist is None
                else args.target_waist)
    directory=args.output_root/datetime.now().strftime('waist_alignment_%Y%m%d_%H%M%S_%f')
    directory.mkdir(parents=True)
    report=dict(target_waist_rad=target,execute_requested=args.execute,goal_sent=False,
                reached=False,gains_changed=False,archive=args.archive,
                collision_context_only=args.collision_context_only,
                conservative_hands=args.conservative_hands)
    rclpy.init();node=rclpy.create_node('saved_waist_alignment')
    session=Session();lock=threading.RLock();live={};observations=[]
    def joint_callback(msg):
        with lock:
            try:
                session.update(dict(zip(msg.name,msg.position)))
                observations.append(dict(at=time.monotonic(),q=dict(session.current),
                    dq=dict(zip(msg.name,msg.velocity)),command=dict(live.get('command',{}))))
            except ValueError:pass
    def diag_callback(msg):
        with lock:
            for status in msg.status:
                data={v.key:v.value for v in status.values}
                if 'task_stop_epoch' in data:live.update(gateway=data,gateway_at=time.monotonic())
    probe_path=Path(__file__).resolve().parents[1]/'build/command_source_probe/command_source_probe'
    probe=subprocess.Popen([str(probe_path)],stdout=subprocess.PIPE,text=True)
    def read_commands():
        for line in probe.stdout:
            try:msg=json.loads(line)
            except ValueError:continue
            if 'gid' not in msg:continue
            with lock:
                counts=live.setdefault('command_sources',{})
                counts[msg['gid']]=counts.get(msg['gid'],0)+1
                values={n:float(msg['q'][15+i]) for i,n in enumerate(ARMS['left'])}
                values.update({n:float(msg['q'][22+i]) for i,n in enumerate(ARMS['right'])})
                values['waist_yaw_joint']=float(msg['q'][12])
                live.update(command=values,command_at=time.monotonic(),command_details=msg)
    command_thread=threading.Thread(target=read_commands,daemon=True);command_thread.start()
    def console_callback(msg):
        with lock:live.update(console=json.loads(msg.data),console_at=time.monotonic())
    node.create_subscription(JointState,'/joint_states',joint_callback,10)
    node.create_subscription(DiagnosticArray,'/g1/arm_gateway/diagnostics',diag_callback,10)
    node.create_subscription(String,'/g1/task/console_status',console_callback,
        QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    executor=SingleThreadedExecutor();executor.add_node(node)
    thread=threading.Thread(target=executor.spin,daemon=True);thread.start()
    action=ActionClient(node,FollowJointTrajectory,'/left_arm_controller/follow_joint_trajectory')
    stop=node.create_client(Trigger,'/g1/arm_gateway/stop_task')
    hand=HandControl(args.network,enabled=False)
    goal=None;result_received=False;baseline=None
    def checked(stationary=False):
        with lock:
            now=time.monotonic()
            if probe.poll() is not None or len(live.get('command_sources',{}))!=1:
                raise RuntimeError('Command monitor unavailable or multiple active command writers')
            if now-live.get('gateway_at',float('-inf'))>1.5 or now-live.get('command_at',float('-inf'))>.15:
                raise RuntimeError('Stale gateway/command feedback')
            q=session.measured(stationary=stationary)
            gateway=dict(live['gateway']);command=dict(live['command'])
            if float(gateway.get('max_motor_temperature_c','nan')) >= 65:
                raise RuntimeError('Live test temperature ceiling reached')
            if not all(math.isfinite(v) for v in command.values()):raise RuntimeError('Nonfinite held target')
            if baseline is not None and (gateway['task_stop_epoch']!=baseline['task_stop_epoch'] or
                                        gateway['task_stop_active']!='false'):
                raise RuntimeError('Stopped by operator/controller; no automatic resume')
            if gateway['phase'] not in ('ARMED_HOLD','RUNNING'):
                raise RuntimeError('Controller left hold/trajectory mode: '+gateway['phase'])
            return q,gateway,command
    def wait(future,timeout):
        deadline=time.monotonic()+timeout
        while not future.done():
            checked()
            if time.monotonic()>deadline:raise TimeoutError('Alignment action timed out')
            time.sleep(.02)
        return future.result()
    try:
        time.sleep(1.5)
        q,baseline,held=checked(stationary=True);supported_gateway(baseline)
        with lock:
            if time.monotonic()-live.get('console_at',float('-inf'))>1.5 or live['console']['worker_busy']:
                raise RuntimeError('Task console is unavailable or busy')
        publishers=node.get_publishers_info_by_topic('/arm_sdk')
        names=[p.node_name for p in publishers]
        if names.count('g1_arm_sdk_hardware_gateway')!=1 or any(
                n not in ('g1_arm_sdk_hardware_gateway','_CREATED_BY_BARE_DDS_APP_') for n in names):
            raise RuntimeError('Unexpected arm_sdk publisher discovery')
        with lock:
            if sum(live['command_sources'].values())<100:
                raise RuntimeError('Insufficient command-source observation')
            report.update(command_sources=dict(live['command_sources']),discovered_publishers=names,
                          initial_command_details=live['command_details'])
        if baseline.get('task_waist_trajectory_supported')!='true':raise RuntimeError('Waist trajectories disabled')
        if any(abs(q[n]-held[n])>.025 for n in ARM_NAMES):raise RuntimeError('Arms not settled at held targets')
        hands={} if args.conservative_hands else {s:hand.read(s) for s in ARMS}
        for side,state in hands.items():q.update(model_joints(side,state['feedback_q']))
        trajectory,duration=(trajectory_for(q,held,target) if args.arm_excursion_rad is None
                             else arm_excursion_for(q,held,args.arm_excursion_rad))
        model=OfflineModel(args.robot_urdf)
        checker=HandBaseCollision(model,args.robot_urdf,scene['base_top_pelvis_m'])
        if args.arm_excursion_rad is None:
            validate_waist_envelope(trajectory,-.6108652382+.02,.6108652382-.02,.06,.12)
        sample=dict(q)
        for positions in samples(trajectory,.06,.12):
            sample.update(zip(trajectory.joint_names,map(float,positions)))
            for name in trajectory.joint_names:
                lo,hi=model.limits[name]
                if not lo+.02 <= sample[name] <= hi-.02:
                    raise RuntimeError('Joint limit margin: '+name)
            if args.conservative_hands:
                collision=checker.check_conservative_hands(sample)
                if collision:raise RuntimeError('All-angle hand envelope collision: '+str(collision))
            for side in ARMS:
                collision=(None if args.conservative_hands else checker.check(sample,side)) or checker.check_arm_body(sample,side)
                if collision:raise RuntimeError('Alignment scoped collision: '+str(collision))
        actual,gateway,command=checked(stationary=True);supported_gateway(gateway)
        if any(abs(actual[n]-q[n])>.005 for n in ARM_NAMES+('waist_yaw_joint',)):
            raise RuntimeError('Robot moved during validation')
        if any(abs(command[n]-held[n])>1e-5 for n in held):raise RuntimeError('Held targets changed during validation')
        report.update(start_joints=q,held_targets=held,gateway_before=baseline,hands=hands,
                      duration_s=duration,trajectory=message_to_ordereddict(trajectory),
                      scope=('waist-only reference; arm references unchanged; bilateral base/body check'
                             if args.arm_excursion_rad is None else
                             'left elbow out-and-back; waist/right references unchanged; bilateral base/body check'),
                      arm_excursion_rad=args.arm_excursion_rad)
        write_json(directory/'preflight.json',report)
        if args.execute:
            if not action.wait_for_server(timeout_sec=2):raise RuntimeError('Existing arm action unavailable')
            checked(stationary=True)
            trajectory.header.stamp=node.get_clock().now().to_msg()
            report['sent_at_monotonic']=time.monotonic()
            future=action.send_goal_async(FollowJointTrajectory.Goal(trajectory=trajectory))
            report['goal_sent']=True
            goal=wait(future,4)
            if not goal.accepted:
                result_received=True
                raise RuntimeError('Gateway rejected alignment trajectory')
            result=wait(goal.get_result_async(),2*duration+12);result_received=True
            report['action_result']=dict(status=result.status,error_code=result.result.error_code,
                                         message=result.result.error_string)
            if result.status!=4 or result.result.error_code!=0:raise RuntimeError('Alignment action did not complete')
            deadline=time.monotonic()+6
            while time.monotonic()<deadline:
                actual,gateway,command=checked()
                endpoint=dict(zip(trajectory.joint_names,trajectory.points[-1].positions))
                if all(abs(actual[n]-v)<.01 for n,v in endpoint.items()):
                    try:checked(stationary=True)
                    except RuntimeError:pass
                    else:report['reached']=True;break
                time.sleep(.05)
            report.update(final_joints=actual,gateway_after=gateway,final_targets=command,
                          final_waist_error_rad=(actual['waist_yaw_joint']-target if target is not None else None),
                          final_endpoint_errors_rad={n:actual[n]-v for n,v in endpoint.items()})
            if not report['reached']:
                report['note']='Reference delivered, but actual alignment not reached; ordinary hold retained. Do not continue task.'
    except BaseException as error:
        report['error']=str(error)
        if report['goal_sent'] and not result_received:
            if stop.wait_for_service(timeout_sec=1):
                future=stop.call_async(Trigger.Request())
                deadline=time.monotonic()+2
                while not future.done() and time.monotonic()<deadline:time.sleep(.02)
                report['stop_requested']=True
        if isinstance(error,(KeyboardInterrupt,SystemExit)):raise
    finally:
        probe.terminate();probe.wait(timeout=3);command_thread.join(timeout=3)
        hand.close();executor.shutdown();thread.join(timeout=3);node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
        write_json(directory/'observations.json',observations)
        write_json(directory/'result.json',report)
    print(json.dumps(dict(directory=str(directory),**{k:v for k,v in report.items() if k in (
        'target_waist_rad','goal_sent','reached','error','final_waist_error_rad','note','duration_s','action_result')}),indent=2))
    return 0 if 'error' not in report and (not args.execute or report['reached']) else 1


if __name__=='__main__':raise SystemExit(main())
