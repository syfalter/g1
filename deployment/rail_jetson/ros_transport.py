"""Jetson-native transport for the recorded executor; no GUI or WSL dependency.

Construction starts only subscriptions and clients. initialize is the explicit
ARM/preparation command. The guarded C++ gateway remains the sole arm publisher.
"""
import hashlib
import copy
import math
import os
from pathlib import Path
import sys
import threading
import time

from bundle import ROOT
from remote_recovery import RecoveryChord
from automatic_recovery import TrajectoryRejected, near, endpoint
sys.path.insert(0, str(ROOT / 'scripts'))
from task_planning_config import parameters_from_values
from task_stage_control import replay_arm_kp, replay_stage_parameters, validate_gateway_gains

JOINTS = ('shoulder_pitch', 'shoulder_roll', 'shoulder_yaw', 'elbow', 'wrist_roll', 'wrist_pitch', 'wrist_yaw')
MOTORS = {side + '_' + joint + '_joint': first + index
          for side, first in (('left', 15), ('right', 22)) for index, joint in enumerate(JOINTS)}
MOTORS['waist_yaw_joint'] = 12


def silent_reset_candidate(gateway):
    return (gateway.get('phase') in ('UNARMED', 'LATCHED_SILENT')
            and gateway.get('armed') == 'false'
            and gateway.get('command_publisher_exists') == 'false')


class RosTransport:
    simulation = False

    def __init__(self, release, planner, urdf, sdk, network='eth0', managed_gateway=False, gateway_config=None):
        import rclpy
        from rclpy.action import ActionClient
        from rclpy.executors import MultiThreadedExecutor
        from rclpy.qos import qos_profile_sensor_data
        from control_msgs.action import FollowJointTrajectory
        from diagnostic_msgs.msg import DiagnosticArray
        from std_msgs.msg import Empty
        from unitree_hg.msg import LowState
        from std_srvs.srv import Trigger
        from rcl_interfaces.srv import SetParametersAtomically
        from g1_unitree_control.srv import ResumeTaskHold
        from task_hand_control import HandControl

        self.release, self.planner = release, planner
        self.model_sha = hashlib.sha256(Path(urdf).read_bytes()).hexdigest()
        self.lock = threading.RLock()
        self.stop_lock = threading.Lock()
        self.command_lock = threading.RLock()
        self.stopped = threading.Event()
        self.recovery_chord = RecoveryChord()
        self.recovery_stable_since = None
        self.remote_recovery_enabled = False
        from gateway_process import GatewayProcess
        self.managed_gateway = GatewayProcess(ROOT, gateway_config) if managed_gateway else None
        self.retire_requested = False
        self.gateway, self.q, self.dq = {}, {}, {}
        self.gateway_at = self.state_at = self.graph_at = 0.
        self.tick = None
        self.epoch = None
        self.graph_ok = False
        self.graph_error = 'Discovery warming up'
        self.authority_stop_sent = False
        self.armed_graph_ready = False
        self.authority = None
        if not (os.environ.get('ROS_DOMAIN_ID') == '174' and os.environ.get('ROS_LOCALHOST_ONLY') == '1'):
            from dds_authority import DdsAuthority
            self.authority = DdsAuthority(ROOT)
        self.stage_revision = None
        self.hand_enabled = False
        self.heartbeat_allowed = lambda: True
        self.node = rclpy.create_node('jetson_recorded_executor')
        self.services = {name: self.node.create_client(Trigger, '/g1/arm_gateway/' + name)
                         for name in ('arm', 'stop_task', 'disarm')}
        self.hold = self.node.create_client(ResumeTaskHold, '/g1/arm_gateway/hold_after_stop')
        self.reset = self.node.create_client(ResumeTaskHold, '/g1/arm_gateway/reset_task')
        self.parameters = self.node.create_client(SetParametersAtomically, '/g1_arm_sdk_hardware_gateway/set_parameters_atomically')
        self.actions = {side: ActionClient(self.node, FollowJointTrajectory,
                        '/' + side + '_arm_controller/follow_joint_trajectory') for side in ('left', 'right')}
        self.node.create_subscription(LowState, '/lowstate', self.lowstate, qos_profile_sensor_data)
        self.node.create_subscription(DiagnosticArray, '/g1/arm_gateway/diagnostics', self.diagnostics, 10)
        self.node.create_timer(.2, self.graph)
        self.heartbeat = self.node.create_publisher(Empty, '/g1/arm_gateway/task_heartbeat', 1)
        self.node.create_timer(.2, lambda: self.heartbeat.publish(Empty()) if (
            self.owns_gateway() and self.heartbeat_allowed() and self.graph_ok
            and time.monotonic() - self.graph_at < 1.) else None)
        self.spinner = MultiThreadedExecutor(num_threads=3)
        self.spinner.add_node(self.node)
        self.thread = threading.Thread(target=self.spinner.spin, daemon=True)
        self.thread.start()

        class JetsonHand(HandControl):
            def dds_environment(inner):
                environment = dict(os.environ)
                environment['LD_LIBRARY_PATH'] = str(Path(sdk) / 'thirdparty/lib/aarch64')
                return environment

        self.hand = JetsonHand(network, enabled=True)

    def lowstate(self, message):
        with self.lock:
            if message.tick == self.tick:
                return
            q = {name: float(message.motor_state[index].q) for name, index in MOTORS.items()}
            dq = {name: float(message.motor_state[index].dq) for name, index in MOTORS.items()}
            if not all(math.isfinite(v) for v in list(q.values()) + list(dq.values())):
                return
            self.tick, self.q, self.dq, self.state_at = message.tick, q, dq, time.monotonic()
            keys = int(message.wireless_remote[2]) | int(message.wireless_remote[3]) << 8
            if keys & 0x0220 == 0x0220 and self.managed_gateway:
                self.retire_requested = True
                self.stopped.set()
            self.recovery_chord.observe(keys, self.state_at)
            if not (self.remote_recovery_enabled or self.stopped.is_set()
                    or self.gateway.get('phase') == 'LATCHED_SILENT'):
                self.recovery_chord.cancel('waiting for new R2+A')

    def diagnostics(self, message):
        with self.lock:
            if not self.owns_gateway():
                return
            for status in message.status:
                self.gateway.update({item.key: item.value for item in status.values})
                self.gateway['status_message'] = status.message
            self.gateway_at = time.monotonic()

    def owns_gateway(self):
        return self.managed_gateway is None or self.managed_gateway.running

    def retire_if_silent(self):
        if not self.managed_gateway or not self.retire_requested:
            return
        with self.lock:
            gateway, fresh = dict(self.gateway), time.monotonic() - self.gateway_at < 1.5
        if self.managed_gateway.running and not (fresh and silent_reset_candidate(gateway)):
            return
        self.managed_gateway.retire(gateway, fresh)
        with self.lock:
            self.gateway, self.gateway_at, self.epoch = {}, 0., None
            self.retire_requested = False

    def ensure_gateway(self, check):
        if not self.managed_gateway:
            return
        if self.managed_gateway.running:
            return
        if self.retire_requested:
            raise RuntimeError('L2+B retirement pending; press R2+A after output exit')
        # Start a fresh observation window after WSL relinquishes ownership.
        # Historical WSL samples are not authority for this new session.
        if self.authority:
            from dds_authority import DdsAuthority
            self.authority.close()
            self.authority = DdsAuthority(ROOT)
        deadline = time.monotonic() + 5.
        while time.monotonic() < deadline:
            check()
            if self.stopped.is_set() or time.monotonic() - self.state_at > .5:
                raise RuntimeError('Startup stopped or LowState stale; no takeover')
            time.sleep(.05)
        own = self.authority.check() if self.authority else set()
        writers = self.node.get_publishers_info_by_topic('/arm_sdk')
        if (self.node.count_publishers('/g1/arm_gateway/diagnostics') or
                any(bytes(p.endpoint_gid).hex()[:32] not in own for p in writers)):
            raise RuntimeError('WSL/another controller still present; stop it before initialize')
        check()
        with self.lock:
            self.gateway, self.gateway_at = {}, 0.
            self.graph_ok = False
            self.epoch = None
            self.authority_stop_sent = False
        self.managed_gateway.start()
        deadline = time.monotonic() + 30.
        ready_since = None
        while time.monotonic() < deadline:
            check()
            if self.stopped.is_set():
                raise RuntimeError('Startup stopped; no ARM sent')
            if not self.managed_gateway.running:
                raise RuntimeError('Native gateway exited during initialize startup')
            with self.lock:
                if (time.monotonic() - self.gateway_at < 1.5 and self.graph_ok
                        and time.monotonic() - self.state_at < .5
                        and self.gateway.get('phase') == 'UNARMED'
                        and self.gateway.get('hardware_output_available') == 'true'
                        and self.gateway.get('service_state_ready') == 'true'
                        and self.gateway.get('upper_body_state_safe') == 'true'
                        and self.dq and max(abs(v) for v in self.dq.values()) <= .05):
                    # The cold gateway samples torque history at 50 Hz before
                    # takeover. Warm that history without issuing/retrying ARM.
                    ready_since = ready_since or time.monotonic()
                    if time.monotonic() - ready_since >= 2.:
                        return
                else:
                    ready_since = None
            time.sleep(.05)
        raise RuntimeError('Gateway startup did not become ready; no ARM sent')

    def graph(self):
        try:
            publishers = self.node.get_publishers_info_by_topic('/arm_sdk')
            diagnostic_count = self.node.count_publishers('/g1/arm_gateway/diagnostics')
            own = {bytes(p.endpoint_gid).hex()[:32] for p in publishers
                   if p.node_name == 'g1_arm_sdk_hardware_gateway'}
            native = self.authority.check(own) if self.authority else set()
            external = [p for p in publishers if bytes(p.endpoint_gid).hex()[:32] not in native]
            with self.lock:
                expected = 1 if self.gateway.get('command_publisher_exists') == 'true' else 0
                self.graph_ok = (diagnostic_count == 1 and len(external) == expected and
                    all(p.node_name == 'g1_arm_sdk_hardware_gateway' for p in external))
                self.graph_at = time.monotonic()
                self.graph_error = '' if self.graph_ok else 'Unexpected controller/diagnostic publisher count'
        except Exception as exc:
            with self.lock:
                self.graph_ok = False
                self.graph_error = str(exc)
                self.graph_at = time.monotonic()
        with self.lock:
            if self.gateway.get('armed') != 'true':
                self.armed_graph_ready = False
            elif self.graph_ok:
                self.armed_graph_ready = True
            must_stop = (self.owns_gateway() and not self.graph_ok and self.gateway.get('armed') == 'true'
                         and self.armed_graph_ready and self.epoch is not None and not self.authority_stop_sent)
            if must_stop:
                self.authority_stop_sent = True
                self.stopped.set()
        if must_stop:
            from std_srvs.srv import Trigger
            self.node.get_logger().error('Output ownership lost: ' + self.graph_error)
            self.services['stop_task'].call_async(Trigger.Request())

    def snapshot(self):
        with self.lock:
            if (self.managed_gateway and self.managed_gateway.process is not None
                    and not self.managed_gateway.running):
                self.stopped.set()
                self.retire_requested = True
            passive = not self.owns_gateway()
            return dict(connected=time.monotonic() - self.state_at < .5 and (passive or time.monotonic() - self.gateway_at < 1.5),
                        controllerRunning=not passive, controlOwner='none' if passive else 'jetson',
                        phase='DETACHED' if passive else self.gateway.get('phase'),
                        gateway={} if passive else dict(self.gateway), joints=dict(self.q),
                        sole_publisher=self.graph_ok, ownership_error=self.graph_error,
                        onboard_sdk=self.authority.snapshot() if self.authority else None,
                        remote_recovery=self.recovery_chord.status,
                        stop_latched=self.stopped.is_set())

    def cancel_remote_recovery(self):
        with self.lock:
            self.recovery_chord.cancel('New STOP; waiting for new R2+A')
            self.recovery_stable_since = None

    def claim_remote_recovery(self):
        with self.lock:
            now = time.monotonic()
            g = self.gateway
            if self.managed_gateway and not self.managed_gateway.running:
                if not self.retire_requested and now - self.state_at < .5 and self.recovery_chord.pending(now):
                    self.recovery_chord.cancel('R2+A accepted; controller stays detached until initialize')
                    return True
                return False
            ready = (self.recovery_chord.pending(now) and silent_reset_candidate(g)
                     and now - self.state_at < .5 and now - self.gateway_at < 1.5
                     and self.graph_ok and now - self.graph_at < 1.
                     and g.get('service_state_ready') == 'true'
                     and g.get('upper_body_state_safe') == 'true'
                     and g.get('temperature_alarm_active') == 'false'
                     and self.dq and max(abs(v) for v in self.dq.values()) <= .05)
            if not ready:
                self.recovery_stable_since = None
                return False
            if self.recovery_stable_since is None:
                self.recovery_stable_since = now
            if now - self.recovery_stable_since < .5:
                return False
            self.recovery_chord.cancel('R2+A silent reset in progress')
            self.recovery_stable_since = None
            return True

    def recover_from_remote(self):
        if self.managed_gateway and not self.managed_gateway.running:
            self.stopped.clear()
            return dict(recovered=True, requiresInitialization=True, controllerRunning=False)
        with self.lock:
            if not silent_reset_candidate(self.gateway):
                raise RuntimeError('R2+A reset requires silent unarmed output')
        return self.recover(silent_only=True)

    def assert_authority(self):
        with self.lock:
            if self.stopped.is_set():
                raise RuntimeError('STOP latched; explicit recovery required')
            g = self.gateway
            if g.get('phase') == 'LATCHED_SILENT':
                raise RuntimeError('Gateway safety latch retained: ' + g.get('status_message', '') +
                                   '; press R2+A to recover, then initialize')
            if time.monotonic() - self.gateway_at > 2. or time.monotonic() - self.state_at > .5:
                raise RuntimeError('Native controller/LowState unavailable')
            if not self.graph_ok or time.monotonic() - self.graph_at > 2.:
                raise RuntimeError('Jetson output ownership unavailable: ' + self.graph_error)
            if g.get('hardware_output_available') != 'true':
                raise RuntimeError('Native output not enabled')
            if g.get('task_stop_active') == 'true' or g.get('task_stop_requested') == 'true':
                raise RuntimeError('Native STOP active')
            if self.epoch is not None and int(g.get('task_stop_epoch', '-1')) != self.epoch:
                raise RuntimeError('STOP epoch changed')
            if self.epoch is not None and g.get('phase') not in ('UNARMED', 'ARMED_HOLD', 'RUNNING', 'TAU_ANCHORED_HOLD', 'GRAVITY_FEEDFORWARD_BLEND'):
                raise RuntimeError('Controller not in executable state: ' + str(g.get('phase')))

    def wait(self, future, check, timeout=30.):
        deadline = time.monotonic() + timeout
        while not future.done():
            check()
            if time.monotonic() >= deadline:
                raise TimeoutError('Native request outcome unknown; no automatic resend')
            time.sleep(.02)
        check()
        return future.result()

    def call(self, client, request, check, timeout=30.):
        if not client.wait_for_service(timeout_sec=3.):
            raise RuntimeError('Native service unavailable: ' + client.srv_name)
        with self.command_lock:
            check()
            future = client.call_async(request)
        return self.wait(future, check, timeout)

    def stationary(self, check, expected=None, tolerance=.025, timeout=30.):
        deadline = time.monotonic() + timeout
        stable_since = None
        while time.monotonic() < deadline:
            check()
            with self.lock:
                valid = self.gateway.get('phase') == 'ARMED_HOLD' and self.q and max(abs(v) for v in self.dq.values()) <= .03
                if expected:
                    valid = valid and all(abs(self.q[n] - value) <= tolerance for n, value in expected.items())
                if valid:
                    stable_since = stable_since or time.monotonic()
                    if time.monotonic() - stable_since >= .15:
                        return dict(self.q)
                else:
                    stable_since = None
            time.sleep(.02)
        raise TimeoutError('Stationary/endpoint wait expired; no trajectory resend')

    def verify_configuration(self, action):
        identity = action['identity']
        if identity.get('robot_sha256', identity.get('urdf_sha256')) != self.model_sha:
            raise RuntimeError('Runtime robot model differs from selected recording')
        base = next(iter(self.release['payload']['actions'].values()))
        a = parameters_from_values(action['plan']['planning_config']['values']).controller_parameters('left')
        b = parameters_from_values(base['plan']['planning_config']['values']).controller_parameters('left')
        if a != b:
            raise RuntimeError('Selected actions require different startup controller parameters')

    def initialize(self, release, check):
        from std_srvs.srv import Trigger
        check()
        with self.lock:
            phase = self.gateway.get('phase')
            if phase != 'UNARMED' and not (self.epoch is not None and phase == 'ARMED_HOLD'):
                raise RuntimeError('Initialization requires this process to own an unarmed or recovered controller')
            self.epoch = int(self.gateway['task_stop_epoch'])
        if phase == 'UNARMED':
            # ARM creates the publisher; allow the graph to catch up before the
            # normal authority guard resumes. Never resend an uncertain ARM.
            response = self.call(self.services['arm'], Trigger.Request(), lambda: None)
            if not response.success:
                self.epoch = None
                raise RuntimeError(response.message)
            deadline = time.monotonic() + 20.
            while time.monotonic() < deadline:
                if self.stopped.is_set():
                    raise RuntimeError('Initialization cancelled')
                with self.lock:
                    ready = self.graph_ok and self.gateway.get('phase') == 'ARMED_HOLD'
                if ready:
                    break
                time.sleep(.05)
            else:
                raise TimeoutError('ARM takeover did not complete')
        check()
        if self.gateway.get('task_dual_prepare_supported') != 'true':
            raise RuntimeError('Gateway does not support simultaneous preparation')
        base = next(iter(release['payload']['actions'].values()))
        self.verify_configuration(base)
        values = base['plan']['planning_config']['values']
        self.set_stage('left', 'normal', values, check)
        measured = self.stationary(check)
        trajectory = self.planner(measured, release['payload']['initialization']['target'], values, .7, .7)
        self.align_check(trajectory, check)
        result = self.arm(dict(side='left', kind='join', label='simultaneous preparation'), trajectory, check)
        if result['status'] != 4 or result['error_code'] != 0:
            raise RuntimeError('Preparation failed: ' + str(result))
        self.settle(release['payload']['initialization']['target'], values, check)
        return dict(status='SUCCEEDED', external_signal_verified=False)

    def return_home(self, release, check):
        from home_pose import reference
        check()
        with self.lock:
            if not self.owns_gateway() or self.epoch is None or self.gateway.get('phase') != 'ARMED_HOLD':
                raise RuntimeError('Return home requires this Jetson controller to be initialized and holding')
        if self.gateway.get('task_dual_prepare_supported') != 'true':
            raise RuntimeError('Gateway does not support simultaneous upper-body motion')
        base = next(iter(release['payload']['actions'].values()))
        self.verify_configuration(base)
        values = base['plan']['planning_config']['values']
        self.set_stage('left', 'normal', values, check)
        home = reference()
        trajectory = self.planner(self.stationary(check), home['target'], values, .7, .7)
        self.align_check(trajectory, check)
        result = self.arm(dict(side='left', kind='join', label='return R2+A upper-body home'), trajectory, check)
        if result['status'] != 4 or result['error_code'] != 0:
            raise RuntimeError('Return home failed: ' + str(result))
        self.settle(home['target'], values, check)
        return dict(status='SUCCEEDED', home_sha256=home['sha256'], output_released=False,
                    safe_to_power_off=False)

    def begin(self, action, check):
        check()
        self.hand_enabled = not action['plan'].get('demo', action['plan'].get('rail', {})).get('arm_only', False)
        if self.hand_enabled:
            self.hand.begin_motion(check)
            self.hand.read(action['plan']['arm'], check)

    def set_stage(self, side, profile, values, check):
        from rclpy.parameter import Parameter
        from rcl_interfaces.srv import SetParametersAtomically
        p = replay_stage_parameters(parameters_from_values(values))
        if not p.stage_control_enabled:
            profile = 'normal'
        arm_kp = replay_arm_kp(p, side) if profile != 'normal' else None
        wrist = p.demo_wrist_kp if profile != 'normal' else 0.
        distal = p.demo_distal_wrist_kp if profile != 'normal' else 0.
        self.stationary(check)
        if profile != 'normal':
            validate_gateway_gains(p, self.gateway, wrist_kp=wrist, distal_wrist_kp=distal, arm_kp=arm_kp)
        expected = dict(task_stage_kp=0. if profile == 'normal' else p.withdraw_proximal_kp,
                        task_stage_kd=0. if profile == 'normal' else p.withdraw_proximal_kd,
                        task_stage_waist_kp=0. if profile == 'normal' else p.stage_waist_kp,
                        task_stage_waist_kd=0. if profile == 'normal' else p.stage_waist_kd,
                        task_stage_wrist_kp=wrist, task_stage_distal_wrist_kp=distal)
        with self.lock:
            previous = int(self.gateway.get('task_stage_revision', '0'))
            unchanged = profile == 'normal' and self.gateway.get('task_stage_profile') == 'normal'
        values = dict(task_stage_profile=profile, task_stage_arm=side, task_stage_stop_epoch=self.epoch)
        values.update({k: float(v) for k, v in expected.items() if v != 0})
        values.update(task_stage_kp=float(expected['task_stage_kp']), task_stage_kd=float(expected['task_stage_kd']))
        if arm_kp is not None:
            values['task_stage_arm_kp'] = arm_kp
        request = SetParametersAtomically.Request(parameters=[Parameter(k, value=v).to_parameter_msg() for k, v in values.items()])
        deadline = time.monotonic() + 30.
        if not unchanged:
            while True:
                response = self.call(self.parameters, request, check)
                if response.result.successful:
                    break
                if not response.result.reason.startswith('STAGE_WAIT_STATIONARY:') or time.monotonic() >= deadline:
                    raise RuntimeError('Stage parameters rejected: ' + response.result.reason)
                time.sleep(.1)
        while time.monotonic() < deadline:
            check()
            with self.lock:
                g = self.gateway
                ready = (int(g.get('task_stage_revision', '-1')) >= previous + (0 if unchanged else 1) and
                         g.get('task_stage_switching') == 'false' and g.get('task_stage_profile') == profile and
                         g.get('phase') == 'ARMED_HOLD')
                if ready:
                    for name, target in expected.items():
                        if abs(float(g.get(name, 'nan')) - target) > 1e-6 or not math.isfinite(float(g.get(name, 'nan'))):
                            raise RuntimeError('Stage acknowledgement differs: ' + name)
                    if profile != 'normal' and g.get('task_stage_arm') != side:
                        raise RuntimeError('Stage arm acknowledgement differs')
                    if int(g.get('task_stage_arm_kp_count', '-1')) != (7 if arm_kp is not None else 0):
                        raise RuntimeError('Per-axis gain count differs')
                    for index, target in enumerate(arm_kp or []):
                        if abs(float(g['task_stage_arm_kp_' + str(index)]) - target) > 1e-6:
                            raise RuntimeError('Per-axis gain acknowledgement differs')
                    self.stage_revision = int(g['task_stage_revision'])
                    return
            time.sleep(.02)
        raise TimeoutError('Stage acknowledgement wait expired')

    def align_check(self, trajectory, check):
        expected = dict(zip(trajectory['joint_names'], trajectory['points'][0]['positions']))
        self.stationary(check, expected=expected)

    def arm(self, step, trajectory, check):
        from control_msgs.action import FollowJointTrajectory
        from trajectory_msgs.msg import JointTrajectory
        from rosidl_runtime_py.set_message import set_message_fields
        message = JointTrajectory()
        # Foxy's converter replaces nested list entries with ROS objects.
        # Never let it mutate the frozen JSON arrays or the endpoint evidence.
        set_message_fields(message, copy.deepcopy(trajectory))
        message.header.stamp = self.node.get_clock().now().to_msg()
        client = self.actions[step['side']]
        if not client.wait_for_server(timeout_sec=3.):
            raise RuntimeError('Arm action server unavailable')
        recorded = step.get('recorded_hand')
        delayed = step.get('delayed_hand')
        if (recorded or delayed) and not self.hand_enabled:
            raise RuntimeError('Finger timeline present in an arm-only action')
        motion = None
        options = {}
        if delayed:
            from task_rail_hand import MotionClock
            motion = MotionClock()
            options['feedback_callback'] = motion.feedback
        with self.command_lock:
            check()
            if self.stage_revision != int(self.gateway.get('task_stage_revision', '-1')):
                raise RuntimeError('Stage changed before send')
            sent = time.monotonic()
            future = client.send_goal_async(FollowJointTrajectory.Goal(trajectory=message), **options)
        goal = self.wait(future, check)
        if not goal.accepted:
            rejected_at = time.monotonic()
            deadline = time.monotonic() + 1.5
            while time.monotonic() < deadline:
                check()
                if self.gateway_at > rejected_at and self.gateway.get('last_rejection'):
                    break
                time.sleep(.02)
            reason = self.gateway.get('last_rejection', '')
            error = TrajectoryRejected if self.gateway_at > rejected_at and reason else RuntimeError
            raise error('Native gateway rejected trajectory: ' + (reason or 'diagnostic unavailable'))
        future = goal.get_result_async()
        duration = message.points[-1].time_from_start
        timeout = 2 * (duration.sec + duration.nanosec / 1e9) + 60.
        if recorded:
            if time.monotonic() - sent > .15:
                raise RuntimeError('Arm acceptance too late for recorded finger synchronization')
            from task_dring_demo_hand import replay
            result, _ = replay(self.hand, step['side'], recorded, future, check, lambda state: None)
        elif delayed:
            from task_rail_hand import replay
            result, _ = replay(self.hand, step['side'], delayed, future, motion, check,
                               lambda state: None, lambda *args, **kwargs: None, timeout=timeout)
        else:
            result = self.wait(future, check, timeout)
        return dict(status=result.status, error_code=result.result.error_code, message=result.result.error_string)

    def settle(self, endpoint, values, check):
        p = parameters_from_values(values)
        self.stationary(check, expected=endpoint, tolerance=math.radians(p.endpoint_tolerance_deg),
                        timeout=max(15., p.endpoint_wait_seconds))

    def hand_transition(self, step, check):
        if not self.hand_enabled:
            raise RuntimeError('Hand stage is not authorized by this action')
        from task_hand_completion import transition
        transition(self.hand, step['side'], step['target_q'], check, duration=step['duration'],
                   require_target=step.get('hand_target_required',True), timeout=max(15., step['duration'] + 8.),
                   finger_sequence=step.get('finger_sequence'))
        return dict(success=True)

    def stop_and_confirm(self):
        from std_srvs.srv import Trigger
        if not self.owns_gateway():
            self.stopped.set()
            return True
        with self.stop_lock:
            with self.command_lock:
                self.stopped.set()
                self.hand.request_stop()
            hand_ok = True
            client = self.services['stop_task']
            if not client.wait_for_service(timeout_sec=2.):
                return False
            previous = int(self.gateway.get('task_stop_epoch', '-1'))
            # Queue arm STOP before any potentially slow finger RPC.
            future = client.call_async(Trigger.Request())
            try:
                self.hand.hold_active()
            except Exception:
                hand_ok = False
            try:
                response = self.wait(future, lambda: None, timeout=5.)
                if not response.success:
                    return False
                deadline = time.monotonic() + 10.
                while time.monotonic() < deadline:
                    with self.lock:
                        acknowledged = int(self.gateway.get('task_stop_epoch', '-1')) > previous
                        held = (self.gateway.get('task_stop_active') == 'true' and
                                self.gateway.get('task_stop_requested') == 'false')
                        if (time.monotonic() - self.gateway_at < 1.5 and acknowledged and
                                (held or silent_reset_candidate(self.gateway))):
                            return hand_ok
                    time.sleep(.02)
            except Exception:
                return False
            return False

    def check_automatic_recovery(self, plan, release):
        heartbeat_ok = self.heartbeat_allowed()
        with self.lock:
            now, g = time.monotonic(), self.gateway
            if (not self.owns_gateway() or self.retire_requested or not heartbeat_ok or
                    now - self.state_at > .5 or now - self.gateway_at > 1.5 or
                    now - self.graph_at > 1. or not self.graph_ok):
                raise RuntimeError('Recovery lacks fresh feedback or exclusive Jetson ownership')
            required = dict(armed='true', service_state_ready='true', upper_body_state_safe='true',
                            hardware_output_available='true', task_stop_active='true',
                            task_stop_requested='false', temperature_alarm_active='false',
                            torque_overload_active='false', command_overload_active='false',
                            velocity_recovery_active='false')
            if (any(g.get(k) != v for k, v in required.items()) or
                    g.get('phase') not in ('TASK_STOP_HOLD', 'TASK_ADMITTANCE_BLEND', 'TASK_ADMITTANCE')):
                raise RuntimeError('Recovery blocked by stop, disable, fault or unconfirmed hold state')
            if not near(self.q, plan['checkpoint']):
                raise RuntimeError('Robot is not within 0.05 rad of the recorded recovery checkpoint')
            passive = {n: q for n, q in release['payload']['initialization']['target'].items()
                       if n not in plan['checkpoint']}
            if not near(self.q, passive):
                raise RuntimeError('Other arm is not at its preparation checkpoint')
            return (bool(self.dq) and max(abs(v) for v in self.dq.values()) <= .05 and
                    g.get('phase') == 'TASK_ADMITTANCE')

    def recover_supported(self, check):
        check()
        if not self.owns_gateway() or silent_reset_candidate(self.gateway):
            raise RuntimeError('Automatic recovery never arms or resets a disabled controller')
        return self.recover(check=check)

    def execute_recovery_route(self, plan, release, check):
        action = release['payload']['actions'][plan['action']]
        values = action['plan']['planning_config']['values']
        self.begin(action, check)
        for step in plan['steps']:
            check()
            self.set_stage(step['side'], 'normal', values, check)
            if step['kind'] == 'hand':
                self.hand_transition(step, check)
                continue
            trajectory = step['trajectory']
            target = endpoint(trajectory, 0)
            measured = self.stationary(check)
            if not near(measured, target):
                raise RuntimeError('Recovery checkpoint drifted; no approach retry')
            join = self.planner(measured, target, values, .35, .35)
            result = self.arm(dict(side=step['side'], kind='join'), join, check)
            if result.get('status') != 4 or result.get('error_code') != 0:
                raise RuntimeError('Recovery alignment failed')
            self.settle(target, values, check)
            self.set_stage(step['side'], step['profile'], values, check)
            self.align_check(trajectory, check)
            result = self.arm(step, trajectory, check)
            if result.get('status') != 4 or result.get('error_code') != 0:
                raise RuntimeError('Recorded escape failed')
            self.settle(endpoint(trajectory, -1), values, check)
        check()
        result = self.initialize(release, check)
        if result.get('status') != 'SUCCEEDED':
            raise RuntimeError('Automatic preparation failed')

    def recover(self, silent_only=False, check=lambda: None):
        from g1_unitree_control.srv import ResumeTaskHold
        if not self.owns_gateway():
            self.stopped.clear()
            return dict(recovered=True, requiresInitialization=True, controllerRunning=False)
        if silent_reset_candidate(self.gateway):
            self.graph()
            if not self.graph_ok:
                raise RuntimeError('Another output owner exists')
            epoch = int(self.gateway.get('task_stop_epoch', '-1'))
            response = self.call(self.reset, ResumeTaskHold.Request(stop_epoch=epoch), lambda: None)
            if not response.success:
                raise RuntimeError(response.message)
            deadline = time.monotonic() + 5.
            while time.monotonic() < deadline:
                with self.lock:
                    confirmed = (time.monotonic() - self.gateway_at < 1.5 and
                        self.gateway.get('phase') == 'UNARMED' and
                        self.gateway.get('task_stop_requested') == 'false' and
                        self.gateway.get('task_stop_active') == 'false' and
                        int(self.gateway.get('task_stop_epoch', '-1')) == epoch and
                        silent_reset_candidate(self.gateway))
                if confirmed:
                    break
                time.sleep(.02)
            else:
                raise RuntimeError('Silent reset not confirmed; no ARM or trajectory sent')
            self.epoch = None
            self.authority_stop_sent = False
            self.stopped.clear()
            return dict(recovered=True, requiresInitialization=True)
        if silent_only:
            raise RuntimeError('Output changed; no automatic supported-hold recovery')
        if self.gateway.get('task_stop_active') == 'true':
            self.stopped.set()
        if not self.stopped.is_set():
            raise RuntimeError('Recovery requires an acknowledged STOP')
        self.graph()
        if not self.graph_ok or time.monotonic() - self.gateway_at > 1.5:
            raise RuntimeError('Recovery cannot confirm sole live controller')
        epoch = int(self.gateway.get('task_stop_epoch', '-1'))
        if self.gateway.get('task_stop_active') != 'true':
            raise RuntimeError('Native STOP not acknowledged; no automatic latch reset')
        check()
        self.hand.hold_active()
        response = self.call(self.hold, ResumeTaskHold.Request(stop_epoch=epoch), check)
        if not response.success:
            raise RuntimeError(response.message)
        deadline = time.monotonic() + 15.
        while time.monotonic() < deadline:
            check()
            if (time.monotonic() - self.gateway_at < 1.5 and self.gateway.get('phase') == 'ARMED_HOLD' and
                self.gateway.get('task_stop_active') == 'false'):
                self.epoch = epoch
                self.authority_stop_sent = False
                self.stopped.clear()
                return dict(recovered=True, requiresInitialization=True)
            time.sleep(.05)
        raise TimeoutError('Recovery hold not confirmed')

    def release_output(self):
        from std_srvs.srv import Trigger
        if not self.owns_gateway():
            return dict(outputReleased=True, requiresInitialization=True, controllerRunning=False)
        self.hand.hold_active()
        response = self.call(self.services['disarm'], Trigger.Request(), lambda: None)
        if not response.success:
            raise RuntimeError(response.message)
        deadline = time.monotonic() + 15.
        while time.monotonic() < deadline:
            with self.lock:
                if (time.monotonic() - self.gateway_at < 1.5 and self.gateway.get('armed') == 'false' and
                    self.gateway.get('command_publisher_exists') == 'false' and self.gateway.get('phase') == 'UNARMED'):
                    self.epoch = None
                    self.stopped.set()
                    return dict(outputReleased=True, requiresInitialization=True)
            time.sleep(.05)
        raise TimeoutError('Output release not confirmed; do not start another controller')

    def close(self):
        if self.managed_gateway:
            try:
                self.managed_gateway.retire(dict(self.gateway), time.monotonic() - self.gateway_at < 1.5)
            except Exception as error:
                self.node.get_logger().error('Gateway retained; support state uncertain: ' + str(error))
        self.hand.close()
        self.spinner.shutdown()
        self.thread.join(timeout=5.)
        for client in self.actions.values():
            client.destroy()
        self.node.destroy_node()
        if self.authority:
            self.authority.close()
