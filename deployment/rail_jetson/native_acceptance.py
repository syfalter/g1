"""Full HTTP -> Foxy -> native gateway test against a loopback software plant.

Never run in domain 0. The only LowCmd subscriber is the test's software plant.
This verifies native control plumbing, not physical tracking or contact safety.
"""
import argparse
import copy
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import os
from pathlib import Path
import secrets
import signal
import subprocess
import threading
import time
from urllib.request import build_opener, ProxyHandler, Request
from urllib.error import HTTPError

from bundle import ROOT, digest, write
from releases import load_release, package, planned_action


def fixture(source):
    original = source['payload']['actions']['dring_left']
    ready = source['payload']['initialization']
    actions = {}
    for name in ('dring_left', 'dring_right', 'push', 'pull', 'gear_above_to_lower',
                 'gear_lower_to_upper', 'gear_upper_to_lower', 'gear_above_to_upper'):
        side = 'left' if name.startswith('dring_') else 'right'
        names = [n for n in ready['target'] if n.startswith(side + '_')] + ['waist_yaw_joint']
        q = {n: ready['target'][n] for n in names}
        end = dict(q)
        end[names[0]] += .02
        def segment(start, finish, recorded):
            trajectory = dict(joint_names=names, points=[dict(positions=[values[n] for n in names],
                velocities=[0.] * 8, accelerations=[0.] * 8, time_from_start=dict(sec=index, nanosec=0))
                for index, values in enumerate((start, finish))])
            result = dict(kind='arm', side=side, label='synthetic ' + ('recording' if recorded else 'join'),
                          trajectory=trajectory, start=start, end=finish, duration=1.)
            result.update(recording_fit={'synthetic': True}) if recorded else result.update(stage='prepare')
            return result
        plan = dict(kind='dring_demo', arm=side, hardware_executable=True,
                    planning_config=copy.deepcopy(original['plan']['planning_config']),
                    segments=[segment(q, q, False), segment(q, end, True), segment(end, q, False)],
                    demo=dict(operation=name.removeprefix('dring_') if hasattr(name, 'removeprefix') else name[6:],
                              full_cycle=True, arm_only=True))
        if name.startswith('gear_'):
            plan['demo'].update(operation=name[5:], workflow='landing_gear', full_cycle=False)
        if name in ('push', 'pull'):
            plan.update(kind='rail', rail=dict(operation=name, mode='cycle', path_source='recorded', arm_only=True))
        key, action = planned_action(plan, dict(identity=original['identity'], traces={}), 'synthetic/' + name)
        actions[key] = action
    result = package('isolated-native-test-only', actions, ready)
    result['payload']['test_only'] = True
    result['sha256'] = digest(result['payload'])
    return result


def main():
    if os.environ.get('ROS_DOMAIN_ID') != '174' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        raise RuntimeError('Requires isolated localhost-only DDS domain 174')
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--release', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--use-recorded-release', action='store_true')
    parser.add_argument('--latch-recovery-only', action='store_true')
    parser.add_argument('--handoff-only', action='store_true')
    args = parser.parse_args()
    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.qos import qos_profile_sensor_data
    from unitree_hg.msg import LowCmd, LowState
    from ros_transport import MOTORS

    root = ROOT
    args.output.mkdir(parents=True, exist_ok=False)
    selected = load_release(args.release) if args.use_recorded_release else fixture(load_release(args.release))
    if args.use_recorded_release:
        selected['payload']['test_only'] = True
        selected['sha256'] = digest(selected['payload'])
    path = args.output / 'synthetic.json'
    write(path, selected)
    token = secrets.token_urlsafe(32)
    token_file = args.output / 'token'
    token_file.write_text(token)
    token_file.chmod(0o600)
    config = json.loads((root / 'config/native.yaml').read_text())
    config['g1_arm_sdk_hardware_gateway']['ros__parameters'].update(
        hardware_output_enabled=True, hardware_authorization='I_UNDERSTAND_ARM_SDK_OUTPUT',
        service_check_enabled=False, gravity_feedforward_enabled=True, tau_anchor_sample_count=20,
        takeover_anchor_hold_s=.5, feedforward_blend_s=.5,
        trajectory_log_directory=str(args.output / 'motion_logs'))
    write(args.output / 'native.yaml', config)
    rclpy.init()
    plant = rclpy.create_node('isolated_software_robot')
    state = LowState()
    state.mode_machine = 6
    for motor in state.motor_state:
        motor.mode, motor.vol = 1, 48.
        motor.temperature = [25, 25]
    for name, index in MOTORS.items():
        state.motor_state[index].q = selected['payload']['initialization']['target'][name]
    commands = [0]
    lock = threading.Lock()
    def command(msg):
        with lock:
            commands[0] += 1
            for index in MOTORS.values():
                state.motor_state[index].q = msg.motor_cmd[index].q
                state.motor_state[index].dq = msg.motor_cmd[index].dq
    plant.create_subscription(LowCmd, '/arm_sdk', command, 10)
    publisher = plant.create_publisher(LowState, '/lowstate', qos_profile_sensor_data)
    def tick():
        with lock:
            state.tick += 1
            publisher.publish(state)
    plant.create_timer(.005, tick)
    spin = MultiThreadedExecutor(num_threads=2)
    spin.add_node(plant)
    thread = threading.Thread(target=spin.spin, daemon=True)
    thread.start()
    time.sleep(1.)
    if plant.count_publishers('/arm_sdk') or plant.count_publishers('/g1/arm_gateway/diagnostics'):
        spin.shutdown()
        plant.destroy_node()
        rclpy.shutdown()
        raise RuntimeError('Isolated test domain is occupied')
    callbacks = []
    class Callback(BaseHTTPRequestHandler):
        def do_POST(self):
            callbacks.append(json.loads(self.rfile.read(int(self.headers['Content-Length']))))
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.end_headers()
            self.wfile.write(b'{"code":0}')
        def log_message(self, *args):
            pass
    callback = ThreadingHTTPServer(('127.0.0.1', 0), Callback)
    callback_thread = threading.Thread(target=callback.serve_forever, daemon=True)
    callback_thread.start()
    callback_url = 'http://127.0.0.1:%s/result' % callback.server_port
    opener = build_opener(ProxyHandler({}))
    http_port = 18795
    def http(route, data=None):
        request = Request('http://127.0.0.1:%s' % http_port + route, data=None if data is None else json.dumps(data).encode(),
                          headers={'Content-Type': 'application/json', 'X-API-Key': token})
        with opener.open(request, timeout=35.) as response:
            return json.load(response)
    def until(function, timeout=60.):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if any(p.poll() is not None for p in children):
                raise RuntimeError('Test process exited; inspect logs')
            try:
                value = function()
                if value:
                    return value
            except OSError:
                pass
            time.sleep(.1)
        raise TimeoutError('Test deadline exceeded')
    def completed(task_id):
        def poll():
            result = http('/api/v1/tasks/' + task_id + '/query', {})
            return result if result['status'] in ('SUCCEEDED', 'FAILED', 'CANCELLED') else None
        duration = max(sum(s['duration'] for s in a['program']) for a in selected['payload']['actions'].values())
        result = until(poll, timeout=2*duration+90.)
        if result['status'] != 'SUCCEEDED':
            raise AssertionError(result)
        return result
    children, logs = [], []
    def spawn(name, cmd):
        log = (args.output / (name + '.log')).open('w')
        logs.append(log)
        p = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT)
        children.append(p)
        return p
    try:
        if not args.handoff_only:
            spawn('gateway', [str(root / 'native_ws/install/g1_unitree_control/lib/g1_unitree_control/arm_sdk_hardware_gateway'),
                              '--ros-args', '--params-file', str(args.output / 'native.yaml')])
        spawn('worker', ['/usr/bin/python3', str(root / 'deployment/rail_jetson/native_worker.py'), '--hardware',
                         '--release', str(path), '--urdf', str(root / 'assets/planner.urdf'), '--sdk',
                         '/home/unitree/unitree_sdk2-main', '--token-file', str(token_file),
                         '--journal', str(args.output / 'jobs.json'), '--port', '18794'] +
                         (['--managed-gateway', '--gateway-config', str(args.output / 'native.yaml')]
                          if args.handoff_only else []))
        def worker_ready():
            request = Request('http://127.0.0.1:18794/state', data=b'{}',
                              headers={'X-Native-Token': token, 'Content-Type': 'application/json'})
            with opener.open(request, timeout=3.) as response:
                return json.load(response)['native']['connected']
        until(worker_ready)
        gateway_root = root.parents[1] / 'communication/eln_http_demo'
        api_command = [str(gateway_root / '.venv/bin/python'), str(root / 'deployment/rail_jetson/production_api.py'),
                      '--gateway-root', str(gateway_root), '--release', str(path), '--database', str(args.output / 'http.sqlite3'),
                      '--native-token-file', str(token_file), '--api-key-file', str(token_file),
                      '--native-port', '18794', '--port', '18795']
        api_process = spawn('api', api_command)
        until(lambda: http('/health')['hardwareConnected'])
        assert commands[0] == 0, 'Startup must not ARM'
        if args.handoff_only:
            from diagnostic_msgs.msg import DiagnosticArray
            from std_msgs.msg import Empty
            spawn('relay', [str(gateway_root / '.venv/bin/python'), str(root / 'deployment/rail_jetson/http_relay.py'),
                            '--host', '127.0.0.1', '--port', '18790', '--upstream', 'http://127.0.0.1:18795'])
            http_port = 18790
            until(lambda: http('/health')['native']['phase'] == 'DETACHED')
            assert plant.count_publishers('/g1/arm_gateway/diagnostics') == 0
            try:
                http('/api/v1/robot/return-home', dict(taskId='detached-home', callbackUrl=callback_url, contactClear=True))
                raise AssertionError('Detached home must be rejected')
            except HTTPError as error:
                assert error.code == 409
            assert commands[0] == 0
            http('/api/v1/robot/initialize', dict(taskId='lazy-init', callbackUrl=callback_url))
            completed('lazy-init')
            assert commands[0] > 0
            with lock:
                state.wireless_remote[2], state.wireless_remote[3] = 0x20, 0x02
            until(lambda: http('/health')['native']['phase'] == 'DETACHED')
            until(lambda: plant.count_publishers('/g1/arm_gateway/diagnostics') == 0)
            with lock:
                state.wireless_remote[2], state.wireless_remote[3] = 0, 0
            time.sleep(.5)
            before = commands[0]
            heartbeats = [0]
            def heartbeat(message):
                heartbeats[0] += 1
            plant.create_subscription(Empty, '/g1/arm_gateway/task_heartbeat', heartbeat, 10)
            wsl = plant.create_publisher(DiagnosticArray, '/g1/arm_gateway/diagnostics', 10)
            with lock:
                state.wireless_remote[2], state.wireless_remote[3] = 0x10, 0x01
            until(lambda: not http('/health')['requiresRecovery'])
            with lock:
                state.wireless_remote[2], state.wireless_remote[3] = 0, 0
            assert http('/health')['native']['controllerRunning'] is False
            time.sleep(1.)
            assert commands[0] == before and heartbeats[0] == 0
            http('/api/v1/robot/initialize', dict(taskId='must-not-steal-wsl', callbackUrl=callback_url))
            failed = until(lambda: (lambda j: j if j['status'] == 'FAILED' else None)(
                http('/api/v1/tasks/must-not-steal-wsl/query', {})))
            assert 'another controller' in failed['message'], failed
            assert commands[0] == before and plant.count_publishers('/g1/arm_gateway/diagnostics') == 1
            plant.destroy_publisher(wsl)
            time.sleep(1.)
            with lock:
                state.wireless_remote[2], state.wireless_remote[3] = 0x10, 0x01
            until(lambda: not http('/health')['requiresRecovery'])
            with lock:
                state.wireless_remote[2], state.wireless_remote[3] = 0, 0
            http('/api/v1/robot/initialize', dict(taskId='reacquire', callbackUrl=callback_url))
            completed('reacquire')
            http('/api/v1/actions/dring_left/tasks', dict(taskId='relayed-action', callbackUrl=callback_url))
            completed('relayed-action')
            until(lambda: any(x.get('taskId') == 'relayed-action' and x.get('status') == 'SUCCEEDED' for x in callbacks))
            from home_pose import reference
            try:
                http('/api/v1/robot/return-home', dict(taskId='unclear-home', callbackUrl=callback_url, contactClear=False))
                raise AssertionError('Unconfirmed clearance must be rejected')
            except HTTPError as error:
                assert error.code == 409
            http('/api/v1/robot/return-home', dict(taskId='relayed-home', callbackUrl=callback_url, contactClear=True))
            completed('relayed-home')
            until(lambda: any(x.get('taskId') == 'relayed-home' and x.get('taskType') == 'ROBOT_RETURN_HOME'
                              and x.get('status') == 'SUCCEEDED' for x in callbacks))
            home_state = http('/health')['native']
            assert home_state['phase'] == 'ARMED_HOLD'
            assert max(abs(home_state['joints'][name] - value) for name, value in reference()['target'].items()) < .025
            with lock:
                state.wireless_remote[2], state.wireless_remote[3] = 0x20, 0x02
            until(lambda: http('/health')['native']['phase'] == 'DETACHED')
            result = dict(passed=True, physical_hardware_connected=False, domain=174,
                          lazy_initialize=True, l2b_exits_gateway=True, r2a_no_takeover=True,
                          wsl_owner_preserved=True, initialize_reacquires=True,
                          relayed_action_and_callback=True, return_home_and_callback=True,
                          home_endpoint_verified=True, home_retains_hold=True,
                          detached_home_rejected=True, uncleared_home_rejected=True,
                          simulated_motor_commands=commands[0])
            write(args.output / 'result.json', result)
            print(json.dumps(result, indent=2))
            return
        if args.latch_recovery_only:
            with lock:
                state.wireless_remote[2], state.wireless_remote[3] = 0x20, 0x02
            until(lambda: http('/health')['native']['phase'] == 'LATCHED_SILENT')
            with lock:
                state.wireless_remote[2], state.wireless_remote[3] = 0, 0
            until(lambda: http('/health')['native']['gateway']['keys'] == '0')
            assert http('/health')['requiresRecovery'] is True
            try:
                http('/api/v1/robot/initialize', dict(taskId='must-not-start', callbackUrl=callback_url))
                raise AssertionError('Initialize must not clear the historical latch')
            except HTTPError as error:
                assert error.code == 409
            assert http('/api/v1/robot/stop', {})['stopConfirmed'] is True
            with lock:
                state.wireless_remote[2], state.wireless_remote[3] = 0x10, 0x01
            until(lambda: http('/health')['native']['gateway']['keys'] == str(0x0110))
            with lock:
                state.wireless_remote[2], state.wireless_remote[3] = 0, 0
            until(lambda: not http('/health')['requiresRecovery'])
            result = http('/health')
            assert result['native']['phase'] == 'UNARMED' and not result['requiresRecovery']
            assert result['native']['gateway']['command_publisher_exists'] == 'false'
            assert commands[0] == 0
            result = dict(passed=True, physical_hardware_connected=False, domain=174,
                          historical_latch_reset=True, stop_on_silent_confirmed=True, r2a_auto_recovery=True,
                          initialize_before_recovery_rejected=True, motor_commands=commands[0])
            write(args.output / 'result.json', result)
            print(json.dumps(result, indent=2))
            return
        http('/api/v1/robot/initialize', dict(taskId='init', callbackUrl=callback_url))
        completed('init')
        for action in selected['payload']['actions']:
            http('/api/v1/actions/' + action + '/tasks', dict(taskId=action, callbackUrl=callback_url))
            completed(action)
        http('/api/v1/actions/dring_left/tasks', dict(taskId='cancel', callbackUrl=callback_url))
        until(lambda: http('/health')['native']['phase'] == 'RUNNING')
        http('/api/v1/tasks/cancel/cancel', dict(reason='isolated cancel test'))
        until(lambda: http('/api/v1/tasks/cancel/query', {})['status'] in ('CANCELLED', 'FAILED'))
        assert http('/health')['requiresRecovery']
        http('/api/v1/robot/recover', dict(contactClear=True, supportReady=True))
        http('/api/v1/robot/initialize', dict(taskId='init-after-stop', callbackUrl=callback_url))
        completed('init-after-stop')
        http('/api/v1/actions/dring_left/tasks', dict(taskId='after-stop', callbackUrl=callback_url))
        completed('after-stop')
        api_process.terminate()
        api_process.wait(timeout=15.)
        children.remove(api_process)
        def watchdog_stopped():
            request = Request('http://127.0.0.1:18794/state', data=b'{}',
                              headers={'X-Native-Token': token, 'Content-Type': 'application/json'})
            with opener.open(request, timeout=3.) as response:
                return json.load(response)['native']['gateway'].get('task_stop_active') == 'true'
        until(watchdog_stopped, timeout=25.)
        spawn('api-restarted', api_command)
        until(lambda: http('/health')['hardwareConnected'])
        assert not http('/health')['initialized']
        http('/api/v1/robot/recover', dict(contactClear=True, supportReady=True))
        http('/api/v1/robot/release', dict(contactClear=True, supportReady=True))
        assert http('/health')['native']['gateway']['command_publisher_exists'] == 'false'
        result = dict(passed=True, physical_hardware_connected=False, domain=174, localhost_only=True,
                      simulated_lowcmd_messages=commands[0], tested_actions=list(selected['payload']['actions']),
                      checks=['no_startup_arm', 'simultaneous_prepare', 'all_independent_actions', 'cancel',
                              'explicit_recovery', 'execute_after_recovery', 'api_exit_watchdog',
                              'restart_requires_initialization', 'output_release'], callbacks=len(callbacks),
                      original_recording=args.use_recorded_release)
        write(args.output / 'result.json', result)
        print(json.dumps(result, indent=2))
    finally:
        for child in reversed(children):
            if child.poll() is None:
                child.send_signal(signal.SIGINT)
                try:
                    child.wait(timeout=35.)
                except subprocess.TimeoutExpired:
                    child.kill()
                    child.wait()
        for log in logs:
            log.close()
        callback.shutdown()
        callback.server_close()
        spin.shutdown()
        thread.join(timeout=5.)
        plant.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
