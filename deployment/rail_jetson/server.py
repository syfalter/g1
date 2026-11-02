"""Loopback-only offline workflow preview; intentionally has no hardware mode."""
import argparse
import json
import os
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
import secrets
import sqlite3
import threading
from urllib.parse import urlsplit

from bundle import load
from coordinator import Coordinator, Rejected
from releases import ReleaseStore, catalog, publish

WEB = Path(__file__).with_name('web')


def create_server(bundle_path, database, port=8790, simulate_boot=False, *, releases=None, sessions=None,
                  jetson_target=None, jetson_root=None):
    bundle = load(bundle_path)
    store = ReleaseStore(releases) if releases else None
    fresh = not Path(database).exists()
    if not fresh:
        with sqlite3.connect(str(database)) as db:
            if db.execute("SELECT 1 FROM sqlite_master WHERE name='state'").fetchone():
                row = db.execute('SELECT body FROM state WHERE id=1').fetchone()
                saved = json.loads(row[0]) if row else {}
                if saved.get('release_version'):
                    if store is None:
                        raise ValueError('Restart requires the configured release store')
                    bundle = store.load(saved['release_version'])
                    if bundle['sha256'] != saved['release_sha256']:
                        raise ValueError('Active release integrity differs from persisted state')
    controller = Coordinator(bundle, database)
    secret = secrets.token_urlsafe(32)
    if simulate_boot:
        if not fresh:
            controller.close()
            raise ValueError('Boot simulation requires a new database; service restart never auto-moves')
        controller.submit(dict(taskId='BOOT-INITIALIZE', taskType='ROBOT_INITIALIZE', parameters={}))

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *_):
            pass

        def reply(self, status, body, content_type='application/json'):
            raw = json.dumps(body, ensure_ascii=False).encode() if content_type == 'application/json' else body
            self.send_response(status)
            self.send_header('Content-Type', content_type + '; charset=utf-8')
            self.send_header('Content-Length', str(len(raw)))
            self.send_header('Cache-Control', 'no-store')
            self.send_header('X-Content-Type-Options', 'nosniff')
            self.send_header('Content-Security-Policy', "default-src 'self'; script-src 'self'; style-src 'self'; img-src 'self'; frame-ancestors 'none'")
            self.end_headers()
            self.wfile.write(raw)

        def trusted_host(self):
            try:
                return urlsplit('http://' + self.headers.get('Host', '')).hostname in ('127.0.0.1', 'localhost', '::1')
            except ValueError:
                return False

        def do_GET(self):
            if not self.trusted_host():
                return self.reply(403, dict(message='Loopback Host required'))
            path = urlsplit(self.path).path
            if path == '/api/state':
                return self.reply(200, dict(controller.snapshot(), servicePid=os.getpid()))
            if path == '/api/session':
                return self.reply(200, dict(token=secret, simulation=True,
                    publicationEnabled=bool(store and sessions), releasesEnabled=bool(store),
                    uploadEnabled=bool(store and jetson_target and jetson_root)))
            if path == '/api/releases':
                return self.reply(200, store.list() if store else dict(releases=[], errors=[]))
            if path == '/api/catalog':
                return self.reply(200, catalog(sessions) if sessions else dict(entries=[], errors=[]))
            if path == '/api/curves':
                result = {}
                with controller.lock:
                    active = controller.bundle
                for operation, action in active['payload']['actions'].items():
                    result[operation] = [dict(label=s['label'], duration=s['duration'],
                        jointNames=s['trajectory']['joint_names'], points=s['trajectory']['points'])
                        for s in action['plan']['segments'] if s.get('trace_stage') or s.get('recording_fit')]
                return self.reply(200, result)
            files = {'/': ('index.html', 'text/html'), '/app.js': ('app.js', 'text/javascript'),
                     '/releases.js': ('releases.js', 'text/javascript'),
                     '/style.css': ('style.css', 'text/css')}
            if path not in files:
                return self.reply(404, dict(message='Not found'))
            filename, mime = files[path]
            self.reply(200, (WEB / filename).read_bytes(), mime)

        def do_POST(self):
            try:
                if not self.trusted_host() or not secrets.compare_digest(self.headers.get('X-Preview-Token', ''), secret):
                    raise Rejected(1002, 'Preview session token required')
                origin = self.headers.get('Origin')
                if origin and origin != 'http://' + self.headers['Host']:
                    raise Rejected(1002, 'Cross-origin request refused')
                length = int(self.headers.get('Content-Length', '0'))
                if length < 0 or length > 16384:
                    raise Rejected(1001, 'Invalid body size')
                data = json.loads(self.rfile.read(length) or b'{}')
                if not isinstance(data, dict):
                    raise Rejected(1001, 'JSON object required')
                path = urlsplit(self.path).path
                if path == '/api/task':
                    response = controller.submit(data)
                elif path == '/api/stop':
                    response = controller.stop()
                elif path == '/api/recovery':
                    response = controller.recovery(data['step'], data.get('contactClear'), data.get('supportReady'))
                elif path == '/api/authority':
                    response = controller.handoff(data['action'], data['epoch'], data.get('token'), data.get('supportReady'))
                elif path == '/api/mapping':
                    controller.set_mapping(data)
                    response = dict(code=0)
                elif path == '/api/simulate-fault':
                    response = controller.stop(fault='SIMULATED_EXECUTION_FAULT')
                elif path == '/api/releases/publish' and store and sessions:
                    envelope = publish(sessions, store, data['version'], data['actions'],
                                       controller.bundle['payload']['initialization'])
                    response = dict(code=0, version=envelope['payload']['version'], sha256=envelope['sha256'])
                elif path == '/api/releases/activate' and store:
                    candidate = store.load(data['version'])
                    with controller.lock:
                        if controller.state['active'] or data['expectedSha256'] != controller.bundle['sha256']:
                            raise Rejected(2001, 'Active task/release changed; refresh before switching')
                        envelope = store.compose(controller.bundle, candidate) if candidate['payload']['release_kind'] == 'patch' else candidate
                        response = controller.activate_release(envelope, data['expectedSha256'])
                elif path == '/api/releases/upload' and store and jetson_target and jetson_root:
                    from upload_release import upload
                    response = upload(store.path(data['version']), jetson_target, jetson_root)
                else:
                    return self.reply(404, dict(message='Not found'))
                self.reply(200, response)
            except Rejected as error:
                self.reply(409, dict(code=error.code, message=str(error)))
            except (ValueError, KeyError, TypeError, OSError) as error:
                self.reply(400, dict(code=1001, message=str(error)))
            except Exception as error:
                self.reply(500, dict(code=9000, message=str(error)))

    server = ThreadingHTTPServer(('127.0.0.1', port), Handler)
    server.controller = controller
    return server


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bundle', type=Path, required=True)
    parser.add_argument('--database', type=Path, required=True)
    parser.add_argument('--port', type=int, default=8790)
    parser.add_argument('--simulate-boot', action='store_true')
    parser.add_argument('--releases', type=Path)
    parser.add_argument('--sessions', type=Path)
    parser.add_argument('--jetson-target')
    parser.add_argument('--jetson-root')
    args = parser.parse_args()
    args.database.parent.mkdir(parents=True, exist_ok=True)
    server = create_server(args.bundle, args.database, args.port, args.simulate_boot,
        releases=args.releases, sessions=args.sessions,
        jetson_target=args.jetson_target, jetson_root=args.jetson_root)
    done = threading.Event()

    def tick():
        while not done.wait(.05):
            try:
                server.controller.tick()
            except Exception as error:
                server.controller.worker_error = str(error)
                server.controller.stop(fault='SIMULATION_WORKER_FAILED')
                return

    worker = threading.Thread(target=tick, daemon=True)
    worker.start()
    print(f'OFFLINE ONLY: http://127.0.0.1:{server.server_port}', flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        done.set()
        worker.join()
        server.server_close()
        server.controller.close()


if __name__ == '__main__':
    main()
