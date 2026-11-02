"""Start the standalone native/API stack. Never automatically ARM or initialize.

On shutdown retain an armed gateway for physical support. Use the explicit
release procedure before terminating that remaining controller.
"""
import argparse
import fcntl
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import traceback
from urllib.request import build_opener, ProxyHandler, Request

from bundle import ROOT, write
from releases import load_release
from dds_authority import DdsAuthority


def publishers(node):
    result = {}
    for topic in ('/arm_sdk', '/g1/arm_gateway/diagnostics'):
        endpoints = node.get_publishers_info_by_topic(topic)
        result[topic] = [dict(node=info.node_name, namespace=info.node_namespace,
                              gid=bytes(info.endpoint_gid).hex()) for info in endpoints]
    return result


def output_may_be_armed(state):
    # Missing diagnostics are unknown, not evidence of released support.
    return state.get('native', {}).get('gateway', {}).get('armed') != 'false'


def runtime_parameters(root):
    # Foxy can give node-specific file values precedence over CLI -p overrides.
    # Authorize this explicit launch in one file, leaving selected defaults inert.
    config = json.loads((root / 'config/native.yaml').read_text())
    config['g1_arm_sdk_hardware_gateway']['ros__parameters'].update(
        hardware_output_enabled=True, hardware_authorization='I_UNDERSTAND_ARM_SDK_OUTPUT')
    path = root / 'data/runtime_gateway.yaml'
    write(path, config)
    return path


def wait_ready(read_state, children, label, timeout=60., stopping=lambda: False):
    deadline = time.monotonic() + timeout
    last_error = 'no response'
    while time.monotonic() < deadline:
        if stopping():
            raise RuntimeError('Startup interrupted')
        for name, child in children:
            code = child.poll()
            if code is not None:
                raise RuntimeError('%s exited (code=%s); inspect %s.log' % (name, code, name))
        try:
            return read_state()
        except OSError as error:
            last_error = str(error)
            time.sleep(.2)
    raise TimeoutError('%s unavailable after %.0fs: %s' % (label, timeout, last_error))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--hardware', action='store_true', required=True)
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=8795)
    parser.add_argument('--relay-port', type=int, default=0, help='Optional relay; 8090 is normally owned by eln_interface.service')
    parser.add_argument('--check-only', action='store_true', help='Read ROS discovery only; launch nothing')
    args = parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID') != '0' or os.environ.get('ROS_LOCALHOST_ONLY') != '0':
        raise RuntimeError('Use run_standalone.sh for the explicit robot network environment')
    root = ROOT
    selected = root / 'config/selected_bundle.json'
    release = load_release(selected)
    owner = (root / 'data/runtime.lock').open('a')
    fcntl.flock(owner, fcntl.LOCK_EX | fcntl.LOCK_NB)
    machine = Path('/tmp/g1-arm-sdk-owner-%s.lock' % os.getuid()).open('a')
    fcntl.flock(machine, fcntl.LOCK_EX | fcntl.LOCK_NB)
    # Refuse a pre-existing ROS gateway or writer before launching ours.
    import rclpy
    rclpy.init()
    probe = rclpy.create_node('jetson_standalone_start_check')
    authority = None
    try:
        authority = DdsAuthority(root)
        deadline = time.monotonic() + 4.
        while time.monotonic() < deadline:
            rclpy.spin_once(probe, timeout_sec=.1)
        occupied = publishers(probe)
        print('Controller discovery: ' + json.dumps(occupied), flush=True)
        native = authority.check() if args.check_only else set()
        print('Startup is passive; output ownership will be checked on initialize.' if not args.check_only
              else 'Identified idle onboard SDK endpoints: ' + json.dumps(sorted(native)), flush=True)
        external = [p for p in occupied['/arm_sdk'] if p['gid'][:32] not in native]
        if args.check_only and (external or occupied['/g1/arm_gateway/diagnostics']):
            raise RuntimeError('Another controller is present; no takeover attempted. '
                               'Disarming or closing a window is not process shutdown. '
                               'Publisher identities are printed above.')
    finally:
        if authority:
            authority.close()
        probe.destroy_node()
        rclpy.shutdown()
    if args.check_only:
        print('Preflight clear; no controller or API started, no motion sent.', flush=True)
        return
    gateway_root = root.parents[1] / 'communication/eln_http_demo'
    sdk = Path('/home/unitree/unitree_sdk2-main')
    token = (root / 'config/native.token').read_text().strip()
    opener = build_opener(ProxyHandler({}))
    logs = root / 'data/service_logs'
    logs.mkdir(parents=True, exist_ok=True)
    children, streams, named_children = [], [], []
    stopping = [False]
    def signal_stop(*unused):
        stopping[0] = True
    signal.signal(signal.SIGINT, signal_stop)
    signal.signal(signal.SIGTERM, signal_stop)
    def spawn(name, command):
        stream = (logs / (name + '.log')).open('a')
        streams.append(stream)
        process = subprocess.Popen(command, stdout=stream, stderr=subprocess.STDOUT, start_new_session=True)
        children.append(process)
        named_children.append((name, process))
        return process
    def state():
        request = Request('http://127.0.0.1:8794/state', data=b'{}', headers={'X-Native-Token': token, 'Content-Type': 'application/json'})
        with opener.open(request, timeout=3.) as response:
            return json.load(response)
    native = None
    armed = False
    try:
        runtime_parameters(root)
        spawn('worker', ['/usr/bin/python3', str(root / 'deployment/rail_jetson/native_worker.py'),
            '--hardware', '--managed-gateway', '--release', str(selected), '--urdf', str(root / 'assets/planner.urdf'),
            '--sdk', str(sdk), '--token-file', str(root / 'config/native.token'), '--journal', str(root / 'data/native_jobs.json')])
        wait_ready(state, named_children, 'Native worker', stopping=lambda: stopping[0])
        spawn('api', [str(gateway_root / '.venv/bin/python'), str(root / 'deployment/rail_jetson/production_api.py'),
            '--gateway-root', str(gateway_root), '--release', str(selected), '--database', str(root / 'data/production.sqlite3'),
            '--native-token-file', str(root / 'config/native.token'), '--api-key-file', str(root / 'config/api.key'),
            '--host', args.host, '--port', str(args.port)])
        def api_state():
            host = '127.0.0.1' if args.host in ('0.0.0.0', '127.0.0.1', 'localhost') else args.host
            with opener.open('http://%s:%s/health' % (host, args.port), timeout=3.) as response:
                result = json.load(response)
            if (result.get('backend') != 'recorded-native' or result.get('simulation') is not False
                    or result.get('releaseSha256') != release['sha256']):
                raise RuntimeError('HTTP backend/mode/release mismatch; no initialization attempted')
            return result
        wait_ready(api_state, named_children, 'Production HTTP API', stopping=lambda: stopping[0])
        if args.relay_port:
            spawn('relay', [str(gateway_root / '.venv/bin/python'), str(root / 'deployment/rail_jetson/http_relay.py'),
                           '--host', args.host, '--port', str(args.relay_port),
                           '--upstream', 'http://127.0.0.1:' + str(args.port)])
        def relay_ready():
            host = '127.0.0.1' if args.host == '0.0.0.0' else args.host
            with opener.open('http://%s:%s/health' % (host, args.relay_port), timeout=3.) as response:
                result = json.load(response)
            if result.get('backend') != 'recorded-native' or result.get('simulation') is not False:
                raise RuntimeError('Relay did not reach the real executor')
            return result
        if args.relay_port:
            wait_ready(relay_ready, named_children, 'Real HTTP relay', stopping=lambda: stopping[0])
        print('Native Jetson HTTP READY; controller DETACHED until initialize, no trajectory sent.', flush=True)
        print('HTTP http://%s:%s; actions=%s' % (args.host, args.port, ','.join(release['payload']['actions'])), flush=True)
        while not stopping[0]:
            if any(p.poll() is not None for p in children):
                raise RuntimeError('A stack process exited; stopping task admission')
            try:
                armed = output_may_be_armed(state())
            except OSError:
                armed = True
            time.sleep(.5)
    except BaseException:
        # Print before cleanup, which can intentionally retain an armed gateway.
        traceback.print_exc(file=sys.stderr)
        print('Failure logs: ' + str(logs), file=sys.stderr, flush=True)
        raise
    finally:
        try:
            armed = output_may_be_armed(state())
        except Exception:
            # Unknown output state must not cause loss of arm support.
            armed = native is not None and native.poll() is None
        for child in reversed(children):
            if child.poll() is None:
                child.send_signal(signal.SIGINT)
                try:
                    child.wait(timeout=40.)
                except subprocess.TimeoutExpired:
                    child.kill()
                    child.wait()
        if native and native.poll() is None:
            if armed:
                print('Controller support retained; native heartbeat will STOP motion. Do not kill the gateway while loaded.', flush=True)
                print('Gateway PID: %s. Unload/support the arms before the robot shutdown procedure.' % native.pid, flush=True)
                while native.poll() is None:
                    time.sleep(1.)
            else:
                native.send_signal(signal.SIGINT)
                native.wait(timeout=15.)
        for stream in streams:
            stream.close()
        owner.close()
        machine.close()


if __name__ == '__main__':
    main()
