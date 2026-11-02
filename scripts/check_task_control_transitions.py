"""Only exercise command continuity and integral reset transitions on localhost DDS."""
import json
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from diagnostic_msgs.msg import DiagnosticArray
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from g1_unitree_control.srv import ResumeTaskHold
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from task_session import ARMS

ROOT = Path(__file__).resolve().parents[1]


def main():
    if (os.environ.get('ROS_DOMAIN_ID') != '87' or
            os.environ.get('ROS_LOCALHOST_ONLY') != '1' or
            os.environ.get('RMW_IMPLEMENTATION') != 'rmw_fastrtps_cpp'):
        raise RuntimeError('Requires localhost-only FastDDS domain 87')
    profile = str(ROOT / 'scripts/task_test_fastdds.xml')
    os.environ['FASTRTPS_DEFAULT_PROFILES_FILE'] = profile
    os.environ['FASTDDS_DEFAULT_PROFILES_FILE'] = profile
    output = ROOT / 'logs/control_transitions' / time.strftime('%Y%m%d_%H%M%S')
    output.mkdir(parents=True)
    rclpy.init()
    node = rclpy.create_node('task_control_transition_test')
    diagnostics, commands, processes = {}, [], []
    report = {'real_hardware_used': False, 'domain': 87, 'checks': {}}
    log = (output / 'nodes.log').open('w')

    def diagnostic(message):
        for status in message.status:
            diagnostics.update({item.key: item.value for item in status.values})

    node.create_subscription(DiagnosticArray, '/g1/arm_gateway/diagnostics', diagnostic, 10)
    node.create_subscription(JointState, '/task_test/command_tau', commands.append, 1000)

    def wait(predicate, seconds=10):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.01)
            if node.count_publishers('/arm_sdk') or node.count_publishers('/task_test/forbidden'):
                raise AssertionError('Non-test command publisher')
            if any(p.poll() is not None for p in processes):
                raise RuntimeError('Fixture exited: ' + str(output))
            if predicate():
                return
        raise TimeoutError(str(diagnostics))

    def trigger(name):
        client = node.create_client(Trigger, '/g1/arm_gateway/' + name)
        wait(client.service_is_ready)
        future = client.call_async(Trigger.Request())
        wait(future.done)
        assert future.result().success, future.result().message
        node.destroy_client(client)

    client = ActionClient(node, FollowJointTrajectory,
                          '/left_arm_controller/follow_joint_trajectory')

    def move(delta, finish=True, include_waist=True):
        wait(client.server_is_ready)
        base = commands[-1]
        q = list(base.position[:7])
        names = list(ARMS['left'])
        if include_waist:
            q.append(base.position[-1] - .01)
            names.append('waist_yaw_joint')
        count = len(q)
        trajectory = JointTrajectory(joint_names=names)
        trajectory.header.stamp = node.get_clock().now().to_msg()
        trajectory.points = [JointTrajectoryPoint(positions=q, velocities=[0.] * count,
                                                 accelerations=[0.] * count)]
        end = list(q)
        end[-1] += delta
        last = JointTrajectoryPoint(positions=end, velocities=[0.] * count, accelerations=[0.] * count)
        last.time_from_start.sec = 2
        trajectory.points.append(last)
        begin = len(commands) - 1
        future = client.send_goal_async(FollowJointTrajectory.Goal(trajectory=trajectory))
        wait(future.done)
        handle = future.result()
        assert handle.accepted, diagnostics
        result = handle.get_result_async()
        if finish:
            wait(result.done)
            assert result.result().status == 4, result.result()
            until = time.monotonic() + .15
            wait(lambda: time.monotonic() >= until)
        return begin, handle, result

    try:
        binary_dir = ROOT / 'install/g1_unitree_control/lib/g1_unitree_control'
        gateway = os.environ.get('G1_TRANSITION_TEST_GATEWAY', str(binary_dir / 'arm_sdk_hardware_gateway'))
        processes.append(subprocess.Popen([
            gateway, '--ros-args', '--params-file',
            str(ROOT / 'src/g1_unitree_control/config/arm_sdk_hardware_gateway.yaml'),
            '-p', 'task_application_enabled:=true', '-p', 'task_waist_enabled:=true',
            '-p', 'active_arm:=left', '-p', 'hardware_output_enabled:=true',
            '-p', 'hardware_authorization:=I_UNDERSTAND_ARM_SDK_OUTPUT',
            '-p', 'service_check_enabled:=false', '-p', 'state_topic:=/task_test/lowstate',
            '-p', 'command_topic:=/task_test/command', '-r', '/arm_sdk:=/task_test/forbidden',
            '-p', 'trajectory_log_directory:=' + str(output / 'trajectories')],
            stdout=log, stderr=subprocess.STDOUT))
        start = json.loads((ROOT / 'assets/workbench/task_start_pose.json').read_text())['joints']
        positions = [start[n] for n in ARMS['left'] + ARMS['right']]
        processes.append(subprocess.Popen([
            str(binary_dir / 'task_gateway_simulator'), '--ros-args',
            '-p', 'positions:=' + str(positions), '-p', 'include_waist:=true',
            '-p', 'waist_yaw:=0.12', '-p', 'waist_tracking_bias_rad:=0.01'],
            stdout=log, stderr=subprocess.STDOUT))
        wait(lambda: diagnostics.get('mode_machine') == '6', 30)
        until = time.monotonic() + 4.2
        wait(lambda: time.monotonic() >= until)
        trigger('arm')
        wait(lambda: diagnostics.get('phase') == 'ARMED_HOLD' and commands)
        wait(lambda: commands[-1].effort[-1] > .4)
        for label, delta, include_waist in [
                ('hold_to_curve', .02, True),
                ('curve_to_hold_to_reverse', -.02, True),
                ('arm_only_keeps_waist_support', .02, False)]:
            begin, _, _ = move(delta, include_waist=include_waist)
            trace = commands[begin:]
            pairs = list(zip(trace, trace[1:]))
            metrics = {
                'initial_integral_nm': trace[0].effort[-1],
                'max_q_step_rad': max(abs(b.position[-1] - a.position[-1]) for a, b in pairs),
                'max_dq_step_rad_s': max(abs(b.velocity[-1] - a.velocity[-1]) for a, b in pairs),
                'max_tau_step_nm': max(abs(b.effort[-1] - a.effort[-1]) for a, b in pairs),
                # Compare consecutive commands at the SAME hypothetical measured q/dq.
                'max_frozen_state_pid_step_nm': max(abs(
                    b.effort[-1] - a.effort[-1] + 60 * (b.position[-1] - a.position[-1]) +
                    4.5 * (b.velocity[-1] - a.velocity[-1])) for a, b in pairs),
                'final_command_dq_rad_s': trace[-1].velocity[-1],
                'samples': len(trace)}
            report['checks'][label] = metrics
            metrics['passed'] = (metrics['max_q_step_rad'] < .001 and
                                 metrics['max_dq_step_rad_s'] < .006 and
                                 metrics['max_tau_step_nm'] < .031 and
                                 metrics['max_frozen_state_pid_step_nm'] < .08 and
                                 abs(metrics['final_command_dq_rad_s']) < 1e-7)
        wait(lambda: commands[-1].effort[-1] > .15)
        _, handle, result = move(.02, finish=False)
        cancelled = handle.cancel_goal_async()
        wait(cancelled.done)
        wait(result.done)
        assert result.result().status == 5, result.result()
        wait(lambda: abs(commands[-1].effort[-1]) < 1e-7)
        report['checks']['cancel_resets_integral'] = True
        wait(lambda: commands[-1].effort[-1] > .15)
        _, _, result = move(.02, finish=False)
        trigger('cancel_trajectory')
        wait(result.done)
        assert result.result().status == 6, result.result()
        wait(lambda: abs(commands[-1].effort[-1]) < 1e-7)
        report['checks']['service_cancel_aborts_without_crash'] = True
        wait(lambda: commands[-1].effort[-1] > .15)
        trigger('stop_task')
        wait(lambda: diagnostics.get('phase') == 'TASK_ADMITTANCE' and
             abs(commands[-1].effort[-1]) < 1e-7)
        report['checks']['stop_resets_integral'] = True
        hold = node.create_client(ResumeTaskHold, '/g1/arm_gateway/hold_after_stop')
        wait(hold.service_is_ready)
        future = hold.call_async(ResumeTaskHold.Request(stop_epoch=int(diagnostics['task_stop_epoch'])))
        wait(future.done)
        assert future.result().success, future.result().message
        node.destroy_client(hold)
        wait(lambda: diagnostics.get('phase') == 'ARMED_HOLD')
        waist = node.create_client(ResumeTaskHold, '/g1/arm_gateway/stop_task_with_waist')
        wait(waist.service_is_ready)
        epoch=int(diagnostics['task_stop_epoch'])
        future=waist.call_async(ResumeTaskHold.Request(stop_epoch=epoch+10))
        wait(future.done)
        assert not future.result().success
        report['checks']['waist_rejects_wrong_epoch']=True
        future=waist.call_async(ResumeTaskHold.Request(stop_epoch=epoch))
        wait(future.done)
        assert future.result().success, future.result().message
        wait(lambda: diagnostics.get('phase')=='TASK_ADMITTANCE' and
             diagnostics.get('task_waist_admittance_active')=='true')
        assert abs(float(diagnostics['waist_effective_kp'])-20)<1e-6
        assert abs(commands[-1].effort[-1])<1e-7
        report['checks']['waist_admittance_blend_and_integral_reset']=True
        future=waist.call_async(ResumeTaskHold.Request(stop_epoch=int(diagnostics['task_stop_epoch'])))
        wait(future.done)
        assert not future.result().success
        report['checks']['waist_rejects_repeated_entry']=True
        hold=node.create_client(ResumeTaskHold,'/g1/arm_gateway/hold_after_stop')
        wait(hold.service_is_ready)
        begin=len(commands)-1
        future=hold.call_async(ResumeTaskHold.Request(stop_epoch=int(diagnostics['task_stop_epoch'])))
        wait(future.done)
        assert future.result().success
        wait(lambda: diagnostics.get('phase')=='ARMED_HOLD' and
             diagnostics.get('task_waist_admittance_active')=='false')
        assert abs(float(diagnostics['waist_effective_kp'])-60)<1e-6
        # The fixture has a 0.01rad tracking bias; hold recaptures actual, not old q.
        trace=commands[begin:]
        assert max(abs(b.position[-1]-a.position[-1]) for a,b in zip(trace,trace[1:]))<.01001
        report['checks']['waist_recovery_captures_current_pose']=True
        node.destroy_client(hold)
        node.destroy_client(waist)
        _, _, result = move(.02, finish=False)
        trigger('disarm')
        wait(result.done)
        assert result.result().status == 6, result.result()
        wait(lambda: diagnostics.get('armed') == 'false')
        report['checks']['active_disarm_aborts_without_crash'] = True
        report['passed'] = all(v['passed'] if isinstance(v, dict) else v
                               for v in report['checks'].values())
        assert report['passed'], report
    finally:
        (output / 'report.json').write_text(json.dumps(report, indent=2) + '\n')
        print(output / 'report.json')
        print(json.dumps(report, indent=2))
        for process in processes:
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()
        log.close()
        client.destroy()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
