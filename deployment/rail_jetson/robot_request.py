"""Submit and observe real tasks from Jetson or a LAN PC. May cause motion."""
import argparse
import getpass
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import os
from pathlib import Path
import sys
import threading
import time
from urllib.request import build_opener, ProxyHandler, Request
from urllib.error import HTTPError, URLError
from urllib.parse import urlsplit
import uuid

ROOT = (Path(__file__).resolve().parent / '../..').resolve()


def callback_binding(base, host):
    if urlsplit(base).hostname in ('127.0.0.1', 'localhost'):
        return '127.0.0.1', '127.0.0.1'
    if not host or host in ('127.0.0.1', 'localhost', '0.0.0.0'):
        raise ValueError('Remote submission requires --callback-host YOUR_PC_LAN_IP')
    if any(c in host for c in '/:@?#'):
        raise ValueError('Use a LAN IPv4 address or hostname for --callback-host')
    return '0.0.0.0', host


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=['status', 'stop', 'recover', 'release', 'initialize', 'return-home', 'dring_left', 'dring_right', 'push', 'pull',
        'gear_above_to_lower', 'gear_lower_to_upper', 'gear_upper_to_lower', 'gear_above_to_upper'])
    parser.add_argument('--base', default='http://127.0.0.1:8795')
    parser.add_argument('--key-file', type=Path, default=ROOT / 'config/api.key')
    parser.add_argument('--callback-host', help='This PC LAN IPv4 address, reachable from Jetson')
    parser.add_argument('--callback-port', type=int, default=18080)
    parser.add_argument('--contact-clear', action='store_true')
    parser.add_argument('--support-ready', action='store_true')
    parser.add_argument('--full-status', action='store_true')
    args = parser.parse_args()
    if args.action == 'return-home' and not args.contact_clear:
        parser.error('Return home requires --contact-clear: unload hands and clear the entire return path')
    opener = build_opener(ProxyHandler({}))
    key = os.environ.get('ROBOT_API_KEY', '').strip()
    if not key and args.key_file.is_file():
        key = args.key_file.read_text().strip()
    if not key and args.action != 'status':
        key = getpass.getpass('Jetson API key: ').strip()
    def http(route, body=None):
        request = Request(args.base + route, data=None if body is None else json.dumps(body).encode(),
                          headers={'Content-Type': 'application/json', 'X-API-Key': key})
        with opener.open(request, timeout=30.) as response:
            return json.load(response)
    if args.action == 'status':
        state = http('/health')
        if not args.full_status:
            native = state.get('native', {})
            gateway = native.get('gateway', {})
            state = dict(backend=state.get('backend'), simulation=state.get('simulation'),
                connected=state.get('hardwareConnected'), initialized=state.get('initialized'),
                activeTaskId=state.get('activeTaskId'), requiresRecovery=state.get('requiresRecovery'),
                availableActions=state.get('availableActions'), phase=native.get('phase'),
                controllerRunning=native.get('controllerRunning'),
                armed=gateway.get('armed'), hardware_output_available=gateway.get('hardware_output_available'),
                command_publisher_exists=gateway.get('command_publisher_exists'),
                hardware_command_count=gateway.get('hardware_command_count'),
                service_state_ready=gateway.get('service_state_ready'),
                ownership_ok=native.get('sole_publisher'), ownership_error=native.get('ownership_error'))
        print(json.dumps(state, indent=2))
        return
    if args.action in ('stop', 'recover', 'release'):
        body = {}
        if args.action != 'stop':
            if not args.contact_clear or not args.support_ready:
                raise ValueError('Inspect/unload/support the robot first, then pass --contact-clear --support-ready')
            body = dict(contactClear=True, supportReady=True)
        print(json.dumps(http('/api/v1/robot/' + args.action, body), indent=2))
        return
    bind, host = callback_binding(args.base, args.callback_host)
    task_id = 'manual-' + uuid.uuid4().hex
    callback_path = '/result/' + uuid.uuid4().hex
    class Callback(BaseHTTPRequestHandler):
        def do_POST(self):
            if self.path != callback_path:
                self.send_error(404)
                return
            try:
                size = int(self.headers.get('Content-Length', '0'))
                if not 0 < size <= 65536:
                    raise ValueError('Invalid body size')
                self.connection.settimeout(5.)
                event = json.loads(self.rfile.read(size))
                if event.get('taskId') != task_id:
                    raise ValueError('Unexpected task ID')
            except (ValueError, AttributeError, OSError):
                self.send_error(400)
                return
            print(json.dumps(event, ensure_ascii=False), flush=True)
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.end_headers()
            self.wfile.write(b'{"code":0}')
        def log_message(self, *args):
            pass
    server = ThreadingHTTPServer((bind, args.callback_port), Callback)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    print('taskId=' + task_id, flush=True)
    print('WARNING: this request can move the robot. Ctrl-C requests cancellation, not output release.', flush=True)
    try:
        route = '/api/v1/robot/' + args.action if args.action in ('initialize', 'return-home') else '/api/v1/actions/' + args.action + '/tasks'
        body = dict(taskId=task_id, callbackUrl='http://%s:%s%s' % (host, server.server_port, callback_path))
        if args.action == 'return-home':
            body['contactClear'] = True
        result = http(route, body)
        print(json.dumps(result), flush=True)
        while True:
            result = http('/api/v1/tasks/' + task_id + '/query', {})
            if result['status'] in ('SUCCEEDED', 'FAILED', 'CANCELLED'):
                deadline = time.monotonic() + 5.
                while result['callbackDelivery']['pending'] and time.monotonic() < deadline:
                    time.sleep(.2)
                    result = http('/api/v1/tasks/' + task_id + '/query', {})
                print(json.dumps(result, ensure_ascii=False, indent=2))
                if result['callbackDelivery']['pending']:
                    print('Callback delivery pending: check PC firewall/port. Do NOT repeat the motion task.', file=sys.stderr)
                if result['status'] != 'SUCCEEDED':
                    raise SystemExit(1)
                break
            time.sleep(.5)
    except KeyboardInterrupt:
        print(json.dumps(http('/api/v1/tasks/' + task_id + '/cancel', dict(reason='Local request helper interrupted'))))
        raise
    except (URLError, OSError):
        print('Request/result uncertain for ' + task_id + '. Do NOT resubmit with a new ID; query or STOP first.', file=sys.stderr)
        raise
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=2.)


if __name__ == '__main__':
    try:
        main()
    except HTTPError as error:
        print('HTTP %s: %s' % (error.code, error.read().decode(errors='replace')), file=sys.stderr)
        raise SystemExit(1)
    except (URLError, OSError, ValueError) as error:
        print('Request failed: %s. For startup failures inspect data/standalone.log and data/service_logs/.' % error, file=sys.stderr)
        raise SystemExit(1)
