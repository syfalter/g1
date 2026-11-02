"""Read-only live preflight for a frozen task segment; never sends motion/hand targets."""
import argparse
from datetime import datetime
import json
from pathlib import Path
import threading
import time

from task_console_handoff import supported_gateway
from task_hand_control import HandControl, model_joints
from task_plan_archive import verify, write_json
from task_session import Session, ARM_NAMES


def compare_start(stage, joints, hands):
    expected = stage['start']
    watched = ARM_NAMES + (('waist_yaw_joint',) if 'waist_yaw_joint' in expected else ())
    deltas = {name: joints[name] - expected[name] for name in watched}
    hand_deltas = {}
    for side, state in hands.items():
        hand_deltas[side] = {name: value - expected[name]
                            for name, value in model_joints(side, state['feedback_q']).items()}
    worst = max(deltas, key=lambda name: abs(deltas[name]))
    return dict(arm_waist_start_matches=all(abs(v) <= .025 for v in deltas.values()),
                hand_start_matches=all(abs(v) <= .04 for d in hand_deltas.values() for v in d.values()),
                worst_joint=worst, max_arm_waist_error_rad=abs(deltas[worst]),
                joint_errors_rad=deltas, hand_model_errors_rad=hand_deltas)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--archive', required=True, type=Path)
    parser.add_argument('--stage', type=int, default=1)
    parser.add_argument('--network', default='eth0')
    parser.add_argument('--output-directory', required=True, type=Path)
    args = parser.parse_args()
    manifest = verify(args.archive)
    if not 1 <= args.stage <= len(manifest['stages']):
        parser.error('Stage number is outside the saved plan')
    stage = json.loads((args.archive / f'stages/{args.stage:02d}.json').read_text())

    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import QoSProfile, DurabilityPolicy
    from sensor_msgs.msg import JointState
    from diagnostic_msgs.msg import DiagnosticArray
    from std_msgs.msg import String

    rclpy.init()
    node = rclpy.create_node('saved_task_stage_readonly_check')
    session, lock, live = Session(), threading.RLock(), {}

    def joints(message):
        if len(message.name) != len(message.position):
            return
        with lock:
            try:
                session.update(dict(zip(message.name, message.position)))
            except ValueError:
                pass

    def diagnostics(message):
        with lock:
            for status in message.status:
                data = {v.key: v.value for v in status.values}
                if 'task_stop_epoch' in data:
                    live['gateway'] = data
                    live['gateway_at'] = time.monotonic()

    def console(message):
        with lock:
            live['console'] = json.loads(message.data)
            live['console_at'] = time.monotonic()

    def checked_snapshot():
        with lock:
            now = time.monotonic()
            if any(now - live.get(name + '_at', float('-inf')) > 1.5 for name in ('gateway', 'console')):
                raise RuntimeError('Gateway or console status stale')
            gateway = dict(live['gateway'])
            supported_gateway(gateway)
            if gateway['phase'] != 'ARMED_HOLD' or gateway['task_stop_active'] != 'false':
                raise RuntimeError('Gateway is not in ordinary hold')
            if live['console']['worker_busy'] or live['console']['phase'] in ('PLANNING', 'EXECUTING'):
                raise RuntimeError('Main console is busy')
            if 'waist_yaw_joint' in stage['start'] and gateway.get('task_waist_trajectory_supported') != 'true':
                raise RuntimeError('Gateway lacks waist capability')
            return dict(joints=session.measured(stationary=True), gateway=gateway,
                        console=dict(live['console']), joint_state_age_s=now-session.received)

    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(JointState, '/joint_states', joints, 10)
    node.create_subscription(DiagnosticArray, '/g1/arm_gateway/diagnostics', diagnostics, 10)
    node.create_subscription(String, '/g1/task/console_status', console, qos)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    hand = HandControl(args.network, enabled=False)
    report = dict(checked_at=datetime.now().astimezone().isoformat(), archive=str(args.archive.resolve()),
                  original_plan_sha256=manifest['original_plan_sha256'],
                  stage=args.stage, label=stage['label'], duration_s=stage['duration_s'],
                  preflight_passed=False, hardware_executable=False, hardware_commands_sent=False,
                  limitations=['State comparison only, not an execution authorization or collision certification.',
                               'Environment/base motion and actual finger geometry are not measured by this check.',
                               'Full-task hand/contact/stop commissioning gate remains unchanged.'])
    try:
        time.sleep(2.)
        before = checked_snapshot()
        hands = {}
        for side in ('left', 'right'):
            first = hand.read(side)
            time.sleep(.15)
            second = hand.read(side)
            if second['timestamp_ms'] <= first['timestamp_ms']:
                raise RuntimeError('Hand feedback timestamp did not advance: ' + side)
            hands[side] = second
        after = checked_snapshot()
        for key in ('task_stop_epoch', 'received_trajectories', 'accepted_trajectories'):
            if before['gateway'][key] != after['gateway'][key]:
                raise RuntimeError('Controller changed during read-only check: ' + key)
        if before['console']['generation'] != after['console']['generation']:
            raise RuntimeError('Console selection changed during check')
        report.update(after)
        report['gateway_before'] = before['gateway']
        report['hands'] = hands
        comparison = compare_start(stage, after['joints'], hands)
        report.update(comparison)
        report['preflight_passed'] = comparison['arm_waist_start_matches'] and comparison['hand_start_matches']
    except Exception as error:
        report['error'] = str(error)
    finally:
        hand.close()
        executor.shutdown()
        thread.join(timeout=3)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    args.output_directory.mkdir(parents=True, exist_ok=True)
    output = args.output_directory / ('stage_%02d_' % args.stage + datetime.now().strftime('%Y%m%d_%H%M%S_%f') + '.json')
    write_json(output, report)
    print(json.dumps(dict(report_path=str(output), **{k: report[k] for k in (
        'stage', 'label', 'duration_s', 'preflight_passed', 'hardware_executable', 'hardware_commands_sent')},
        max_arm_waist_error_rad=report.get('max_arm_waist_error_rad'), error=report.get('error')), indent=2))
    return 0 if report['preflight_passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
