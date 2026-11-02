"""Loopback HTTP acceptance against the existing ELN gateway, with no Web or ROS.

Starts and terminates only its own offline child process and callback listener.
Use a full release containing push/pull and both D-ring cycles.
"""
import argparse
from contextlib import contextmanager
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import socket
import sqlite3
import subprocess
import sys
import tempfile
import threading
import time
from urllib.error import HTTPError
from urllib.request import build_opener, ProxyHandler, Request


def wait_until(function, timeout=40):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        value = function()
        if value:
            return value
        time.sleep(.05)
    raise TimeoutError('Acceptance condition did not complete')


def check(gateway_root, release, output):
    callbacks = []
    failed_deliveries = []
    callback_lock = threading.Lock()

    class Callback(BaseHTTPRequestHandler):
        def do_POST(self):
            event = json.loads(self.rfile.read(int(self.headers['Content-Length'])))
            with callback_lock:
                fail = event['taskId'] == 'push' and len(failed_deliveries) < 2
                (failed_deliveries if fail else callbacks).append(event)
            body = json.dumps(dict(code=1 if fail else 0)).encode()
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def log_message(self, *args):
            pass

    callback = ThreadingHTTPServer(('127.0.0.1', 0), Callback)
    callback_thread = threading.Thread(target=callback.serve_forever, daemon=True)
    callback_thread.start()
    callback_url = 'http://127.0.0.1:%s/result' % callback.server_port
    opener = build_opener(ProxyHandler({}))
    port_socket = socket.socket()
    port_socket.bind(('127.0.0.1', 0))
    port = port_socket.getsockname()[1]
    port_socket.close()
    base = 'http://127.0.0.1:%s' % port
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)

    def http(path, body=None, expected=200):
        request = Request(base + path, data=None if body is None else json.dumps(body).encode(),
                          headers={'Content-Type': 'application/json'})
        try:
            response = opener.open(request, timeout=30)
        except HTTPError as error:
            response = error
        with response:
            data = json.load(response)
            if response.status != expected:
                raise AssertionError((path, response.status, data))
            return data

    @contextmanager
    def service(database, index):
        with (output / ('service-%s.log' % index)).open('w') as log:
            process = subprocess.Popen([sys.executable, str(Path(__file__).with_name('cloud_service.py')),
                '--gateway-root', str(gateway_root), '--release', str(release),
                '--database', str(database), '--port', str(port)], stdout=log, stderr=subprocess.STDOUT)
            try:
                def healthy():
                    if process.poll() is not None:
                        raise RuntimeError('Offline service exited; see ' + str(log.name))
                    try:
                        state = http('/health')
                        return state if state['backend'] == 'recorded-offline' else None
                    except OSError:
                        return None
                wait_until(healthy)
                yield process
            finally:
                process.terminate()
                try:
                    process.wait(timeout=15)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=5)

    def submit(task_id, kind, parameters):
        if kind == 'ROBOT_INITIALIZE':
            return http('/api/v1/robot/initialize', dict(taskId=task_id, callbackUrl=callback_url))
        return http('/api/v1/tasks', dict(taskId=task_id, taskType=kind,
                                         callbackUrl=callback_url, parameters=parameters))

    def completed(task_id):
        def poll():
            state = http('/api/v1/tasks/%s/query' % task_id, {})
            return state if state['status'] in ('SUCCEEDED', 'FAILED', 'CANCELLED') else None
        return wait_until(poll)

    summary = dict(simulation=True, hardware_commands_sent=0, browser_required=False, checks=[])
    try:
        with tempfile.TemporaryDirectory(prefix='http-', dir=str(output)) as temporary:
            database = Path(temporary) / 'gateway.sqlite3'
            with service(database, 1):
                health = http('/health')
                assert health['simulation'] and not health['hardwareConnected']
                assert not health['initialized']
                summary['releaseSha256'] = health['releaseSha256']
                assert submit('init', 'ROBOT_INITIALIZE', {})['status'] == 'ACCEPTED'
                assert completed('init')['status'] == 'SUCCEEDED'
                for task_id, kind, parameters in (
                    ('push', 'NLG_WOW_OPERATION', dict(wowSignal='AIR')),
                    ('pull', 'NLG_WOW_OPERATION', dict(wowSignal='GROUND')),
                    ('nws', 'NWS_CYCLE', dict(count=2))):
                    assert submit(task_id, kind, parameters)['status'] == 'ACCEPTED'
                    state = completed(task_id)
                    assert state['status'] == 'SUCCEEDED', state
                    assert state['progress'] == parameters.get('count', 1), state
                    assert 'NOT verified' in state['message']
                http('/api/v1/tasks', dict(taskId='push', taskType='NLG_WOW_OPERATION',
                    parameters=dict(wowSignal='AIR'), callbackUrl=callback_url), expected=409)
                http('/api/v1/tasks', dict(taskId='legacy-gear', taskType='GEAR_OPERATION',
                    parameters=dict(count=1, preAction='NONE', cycleMode='FULL_CYCLE'),
                    callbackUrl=callback_url), expected=400)
                assert http('/health')['initialized']
                summary['checks'].extend(['initialize', 'push', 'pull', 'nws_complete_pairs',
                                          'duplicate_task_rejected', 'legacy_gear_rejected_before_admission'])

                wait_until(lambda: http('/api/v1/tasks/push/query', {})['callbackDelivery']['pending'] == 0)
                with callback_lock:
                    push_events = [e for e in callbacks if e['taskId'] == 'push']
                    assert len(failed_deliveries) == 2
                    assert push_events[0]['status'] == 'ACCEPTED'
                    assert any(e['status'] == 'RUNNING' for e in push_events)
                    assert push_events[-1]['status'] == 'SUCCEEDED'
                summary['checks'].append('callback_retry_without_motion_replay')
                frozen = json.loads(Path(release).read_text())['payload']['actions']
                expected = [step['trajectory_sha256']
                            for name in ('push', 'pull', 'dring_left', 'dring_right', 'dring_left', 'dring_right')
                            for step in frozen[name]['program'] if step['kind'] == 'recorded']
                actual = [c['trajectory_sha256'] for c in http('/internal/v1/offline/evidence')['calls']
                          if c['kind'] == 'recorded']
                assert actual == expected, 'Recorded arrays changed, or an action was replayed'
                summary['checks'].append('exact_recorded_arrays_and_dispatch_count')

                assert submit('cancel', 'NWS_CYCLE', dict(count=100))['status'] == 'ACCEPTED'
                http('/api/v1/tasks/cancel/cancel', dict(reason='offline acceptance'))
                state = completed('cancel')
                assert state['status'] == 'CANCELLED', state
                assert http('/health')['requiresRecovery']
                summary['checks'].append('cancel_stops_and_latches_recovery')
                with sqlite3.connect(str(database.with_suffix('.execution.sqlite3'))) as db:
                    rows = db.execute('SELECT task_id,status,progress FROM recorded_executions ORDER BY task_id').fetchall()
                    assert len([r for r in rows if r[0] == 'push']) == 1
                    summary['execution_audit'] = rows

            # Simulate a persisted request whose previous process died before a result.
            with sqlite3.connect(str(database)) as db:
                original = db.execute("SELECT request,event FROM tasks WHERE id='push'").fetchone()
                req, event = (json.loads(value) for value in original)
                req['taskId'] = event['taskId'] = 'interrupted'
                event.update(status='RUNNING', progress=0)
                db.execute('INSERT INTO tasks(id,request,event,status) VALUES(?,?,?,?)',
                           ('interrupted', json.dumps(req), json.dumps(event), 'RUNNING'))
            with service(database, 2):
                state = http('/api/v1/tasks/interrupted/query', {})
                assert state['status'] == 'FAILED' and state['progress'] == 0, state
                assert not http('/health')['initialized']
                assert http('/health')['activeTaskId'] is None
                assert completed('push')['status'] == 'SUCCEEDED'
                with sqlite3.connect(str(database.with_suffix('.execution.sqlite3'))) as db:
                    assert db.execute("SELECT count(*) FROM recorded_executions WHERE task_id='interrupted'").fetchone()[0] == 0
                summary['checks'].append('restart_never_replays_persisted_requests')
            summary['passed'] = True
            (output / 'result.json').write_text(json.dumps(summary, indent=2) + '\n')
            return summary
    finally:
        callback.shutdown()
        callback.server_close()
        callback_thread.join(timeout=5)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--gateway-root', type=Path, required=True)
    parser.add_argument('--release', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(check(args.gateway_root, args.release, args.output), indent=2))
