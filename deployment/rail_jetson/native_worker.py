"""Foxy-side recorded executor. Loopback IPC only; no automatic initialization."""
import argparse
import copy
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import os
from pathlib import Path
import secrets
import sys
import threading
import time

from bundle import write
from executor import Cancelled, LocalJoinPlanner, SequenceExecutor
from releases import load_release
from automatic_recovery import AutomaticRecovery


class Worker:
    def __init__(self, release, transport, planner, journal, gear_library=None):
        self.release, self.transport, self.journal = release, transport, Path(journal)
        self.gear_library = gear_library
        self.sequence = SequenceExecutor(transport, planner)
        self.recovery_file = self.journal.with_suffix('.recovery.json')
        self.lock = threading.RLock()
        self.active = None
        self.thread = None
        self.jobs = {}
        self.last_heartbeat = time.monotonic()
        self.done = threading.Event()
        self.recovering = False
        self.recovery_generation = None
        self.recovery_revision = 0
        self.remote_recovery_result = None
        self.auto_recovery = AutomaticRecovery(self)
        if self.journal.exists():
            self.jobs = json.loads(self.journal.read_text())
            for job in self.jobs.values():
                if job['status'] == 'RUNNING':
                    job.update(status='DONE', result=dict(status='STOP_UNCONFIRMED',
                        message='Native worker restarted; no action replayed', requires_recovery=True))
                    self.mark_recovery('Native worker restarted with an interrupted job')
            write(self.journal, self.jobs)
        if self.recovery_file.exists() and json.loads(self.recovery_file.read_text()).get('required'):
            self.sequence.cancelled.set()
        self.transport.heartbeat_allowed = self.heartbeat_allowed
        self.watchdog = threading.Thread(target=self.watch, daemon=True)
        self.watchdog.start()

    def mark_recovery(self, reason):
        self.auto_recovery.invalidate(reason)
        self.recovery_revision += 1
        write(self.recovery_file, dict(required=True, reason=reason))
        self.sequence.cancelled.set()

    def stop(self):
        with self.lock:
            self.mark_recovery('STOP requested; operator recovery required')
            if hasattr(self.transport, 'cancel_remote_recovery'):
                self.transport.cancel_remote_recovery()
        return self.sequence.cancel()

    def heartbeat_allowed(self):
        with self.lock:
            return not self.done.is_set() and time.monotonic() - self.last_heartbeat < 10.

    def watch(self):
        while not self.done.wait(.25):
            with self.lock:
                expired = self.active is not None and not self.heartbeat_allowed()
            if expired and not self.sequence.cancelled.is_set():
                self.stop()
            try:
                self.auto_recovery.poll()
                self.poll_remote_recovery()
            except Exception as error:
                with self.lock:
                    self.remote_recovery_result = 'R2+A recovery failed: ' + str(error)
                print(self.remote_recovery_result, file=sys.stderr, flush=True)

    def poll_remote_recovery(self):
        with self.lock:
            self.state()
            if (not self.active and not self.recovering and
                    hasattr(self.transport, 'retire_if_silent')):
                self.transport.retire_if_silent()
            if (self.active or self.recovering or not self.sequence.cancelled.is_set()
                    or not self.heartbeat_allowed()
                    or not hasattr(self.transport, 'claim_remote_recovery')
                    or not self.transport.claim_remote_recovery()):
                return
            self.recovering = True
            revision = self.recovery_revision
        self.finish_recovery('R2+A', revision)

    def state(self):
        with self.lock:
            native = self.transport.snapshot() if hasattr(self.transport, 'snapshot') else dict(connected=True)
            if (native.get('phase') == 'LATCHED_SILENT' or native.get('stop_latched')) and not self.sequence.cancelled.is_set():
                self.mark_recovery('Gateway safety latch: ' + native.get('gateway', {}).get('status_message', ''))
            automatic = self.auto_recovery.snapshot()
            self.transport.remote_recovery_enabled = self.sequence.cancelled.is_set() and automatic['status'] not in ('pending', 'running')
            return dict(simulation=self.transport.simulation, releaseSha256=self.release['sha256'],
                        gearLibrarySha256=self.gear_library.sha256 if self.gear_library else None,
                        automaticRecovery=automatic,
                        recoveryGeneration=self.recovery_generation,
                        remoteRecoveryResult=self.remote_recovery_result,
                        active=self.active, recoveryRequired=self.sequence.cancelled.is_set(), native=native)

    def start(self, data):
        job_id, kind, action = data['id'], data['kind'], data.get('action')
        if not isinstance(job_id, str) or not 1 <= len(job_id) <= 128:
            raise ValueError('Invalid job ID')
        if data['sha256'] != self.release['sha256']:
            raise ValueError('Native worker pinned a different release')
        if kind not in ('initialize', 'return_home', 'action', 'gear') or (kind == 'action' and action not in self.release['payload']['actions']):
            raise ValueError('Unknown native operation')
        if kind == 'gear':
            from gear_protocol import recipe
            recipe(data.get('parameters', {}))
            if self.gear_library is None or data.get('gear_sha256') != self.gear_library.sha256:
                raise ValueError('Native gear library missing or differs from API')
        with self.lock:
            if job_id in self.jobs:
                old = self.jobs[job_id]
                if old['kind'] != kind or old.get('action') != action:
                    raise ValueError('Job ID already belongs to another request')
                if kind == 'gear' and (old.get('parameters') != data['parameters'] or
                                       old.get('gear_sha256') != data['gear_sha256']):
                    raise ValueError('Gear job ID already belongs to another recipe')
                return copy.deepcopy(old)
            if self.active or self.recovering:
                raise RuntimeError('Native worker busy')
            if self.sequence.cancelled.is_set():
                raise RuntimeError('Native recovery required')
            if not self.heartbeat_allowed():
                raise RuntimeError('Cloud coordinator heartbeat expired')
            job = dict(id=job_id, kind=kind, action=action, status='RUNNING', events=[], result=None)
            if kind == 'gear':
                job.update(parameters=copy.deepcopy(data['parameters']), gear_sha256=data['gear_sha256'])
            self.jobs[job_id] = job
            write(self.journal, self.jobs)
            self.active = job_id
            self.thread = threading.Thread(target=self.run, args=(job_id,), daemon=True)
            self.thread.start()
            return copy.deepcopy(job)

    def run(self, job_id):
        with self.lock:
            start_revision = self.recovery_revision
        def event(data):
            with self.lock:
                self.jobs[job_id]['events'].append(data)
        try:
            job = self.jobs[job_id]
            if job['kind'] == 'initialize':
                if hasattr(self.transport, 'ensure_gateway'):
                    self.transport.ensure_gateway(self.sequence.check_cancelled)
                result = self.transport.initialize(self.release, self.sequence.check)
            elif job['kind'] == 'return_home':
                result = self.transport.return_home(self.release, self.sequence.check)
            elif job['kind'] == 'gear':
                result = self.sequence.execute(self.release, 'gear_operation', event,
                    action_override=self.gear_library.action(job['parameters']))
                result['gear_sha256'] = self.gear_library.sha256
            else:
                result = self.sequence.execute(self.release, job['action'], event)
        except Exception as error:
            try:
                stopped = self.sequence.cancel()
            except Exception:
                stopped = False
            result = dict(status=('CANCELLED' if isinstance(error, Cancelled) else 'FAILED') if stopped else 'STOP_UNCONFIRMED', message=str(error),
                          stop_confirmed=stopped, requires_recovery=True)
        with self.lock:
            if result.get('requires_recovery'):
                interrupted = self.recovery_revision != start_revision
                self.mark_recovery(result.get('message', 'Task interrupted'))
                result['automatic_recovery'] = not interrupted and self.auto_recovery.schedule(job, result)
            self.jobs[job_id].update(status='DONE', result=result)
            write(self.journal, self.jobs)
            self.active = None

    def recover(self, data):
        if data.get('contactClear') is not True or data.get('supportReady') is not True:
            raise ValueError('Confirm contact clear and support before recovery')
        with self.lock:
            if self.active or self.recovering:
                raise RuntimeError('Wait for the interrupted task to finish stopping')
            self.recovering = True
            revision = self.recovery_revision
        return self.finish_recovery('HTTP', revision)

    def finish_recovery(self, source, revision):
        try:
            result = (self.transport.recover_from_remote() if source == 'R2+A'
                      else self.transport.recover())
            with self.lock:
                if revision != self.recovery_revision:
                    raise RuntimeError('A newer STOP arrived during recovery; recovery remains required')
                generation = secrets.token_hex(16)
                write(self.recovery_file, dict(required=False, reason=source + ' recovery completed',
                                               generation=generation))
                self.recovery_generation = generation
                self.sequence.cancelled.clear()
                self.remote_recovery_result = source + ' recovery completed; initialize required'
            return result
        finally:
            with self.lock:
                self.recovering = False

    def release_output(self, data):
        if data.get('supportReady') is not True or data.get('contactClear') is not True:
            raise ValueError('Unload/support both arms and clear contact before releasing output')
        with self.lock:
            if self.active or self.recovering:
                raise RuntimeError('Output release requires an idle worker')
            self.recovering = True
        try:
            result = self.transport.release_output()
            self.mark_recovery('Output released; explicit recovery and initialization required')
            return result
        finally:
            with self.lock:
                self.recovering = False

    def close(self):
        self.done.set()
        self.watchdog.join(timeout=2.)
        if self.auto_recovery.thread:
            self.auto_recovery.thread.join(timeout=30.)
        # An armed gateway must retain support after worker/API shutdown.
        state = self.state()['native']
        if self.active or state.get('gateway', {}).get('armed') == 'true':
            self.sequence.cancel()
        if self.thread:
            self.thread.join(timeout=30.)
        if hasattr(self.transport, 'close'):
            self.transport.close()


def create_server(worker, token, port):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def do_POST(self):
            try:
                if not secrets.compare_digest(self.headers.get('X-Native-Token', ''), token):
                    raise PermissionError('Native IPC token required')
                size = int(self.headers.get('Content-Length', '0'))
                if not 0 < size <= 16384:
                    raise ValueError('Invalid IPC body size')
                data = json.loads(self.rfile.read(size))
                if self.path == '/heartbeat':
                    with worker.lock:
                        worker.last_heartbeat = time.monotonic()
                    result = worker.state()
                elif self.path == '/state':
                    result = worker.state()
                elif self.path == '/jobs':
                    result = worker.start(data)
                elif self.path == '/job':
                    with worker.lock:
                        result = copy.deepcopy(worker.jobs[data['id']])
                elif self.path == '/stop':
                    result = dict(stop_confirmed=worker.stop())
                elif self.path == '/recover':
                    result = worker.recover(data)
                elif self.path == '/release':
                    result = worker.release_output(data)
                else:
                    raise ValueError('Unknown IPC route')
                code = 200
            except Exception as error:
                print('Native request %s rejected: %s' % (self.path, error), file=sys.stderr, flush=True)
                result, code = dict(error=str(error)), 403 if isinstance(error, PermissionError) else 409
            raw = json.dumps(result, allow_nan=False).encode()
            self.send_response(code)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(raw)))
            self.end_headers()
            self.wfile.write(raw)
    return ThreadingHTTPServer(('127.0.0.1', port), Handler)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--release', required=True, type=Path)
    parser.add_argument('--token-file', required=True, type=Path)
    parser.add_argument('--journal', required=True, type=Path)
    parser.add_argument('--urdf', type=Path)
    parser.add_argument('--sdk', type=Path)
    parser.add_argument('--port', type=int, default=8794)
    parser.add_argument('--managed-gateway', action='store_true')
    parser.add_argument('--gateway-config', type=Path)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument('--hardware', action='store_true')
    mode.add_argument('--offline', action='store_true')
    args = parser.parse_args()
    if args.gateway_config and not (
            os.environ.get('ROS_DOMAIN_ID') == '174' and os.environ.get('ROS_LOCALHOST_ONLY') == '1'):
        raise RuntimeError('Gateway config overrides are isolated-test only')
    release = load_release(args.release)
    if release['payload'].get('test_only') and not (
            os.environ.get('ROS_DOMAIN_ID') == '174' and os.environ.get('ROS_LOCALHOST_ONLY') == '1'):
        raise RuntimeError('Synthetic native fixtures require isolated localhost domain 174')
    token = args.token_file.read_text().strip()
    if len(token) < 32:
        raise ValueError('Native token too short')
    if args.hardware:
        import rclpy
        from ros_transport import RosTransport
        rclpy.init()
        planner = LocalJoinPlanner(args.urdf)
        transport = RosTransport(release, planner, args.urdf, args.sdk, managed_gateway=args.managed_gateway,
                                 gateway_config=args.gateway_config)
    else:
        from offline_transport import OfflineTransport, offline_join
        transport, planner = OfflineTransport(), offline_join
    from gear_protocol import load_optional
    worker = Worker(release, transport, planner, args.journal, load_optional(args.release, release))
    server = create_server(worker, token, args.port)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        worker.close()
        if args.hardware:
            rclpy.shutdown()


if __name__ == '__main__':
    main()
