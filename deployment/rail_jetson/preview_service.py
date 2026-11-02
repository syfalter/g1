"""Manage the isolated loopback preview process, never a robot/controller service."""
import argparse
import fcntl
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
from urllib.request import urlopen

from bundle import ROOT, read, write


def identity(pid):
    try:
        return Path('/proc/' + str(pid) + '/stat').read_text().rsplit(')', 1)[1].split()[19]
    except (OSError, IndexError):
        return None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=('start', 'status', 'stop'))
    parser.add_argument('--port', type=int, default=8790)
    parser.add_argument('--bundle', type=Path)
    args = parser.parse_args()
    runtime = ROOT / 'data/web_preview'
    runtime.mkdir(parents=True, exist_ok=True)
    with (runtime / 'service.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        record_path = runtime / 'process.json'
        record = read(record_path) if record_path.exists() else {}
        pid = record.get('pid')
        alive = pid is not None and record.get('start_ticks') is not None and identity(pid) == record['start_ticks']
        if args.action == 'status':
            print(json.dumps(dict(record, running=alive, hardwareConnected=False)))
            return
        if args.action == 'stop':
            if alive:
                os.kill(pid, signal.SIGINT)
            print(json.dumps(dict(stopped_requested=alive, hardware_commands_sent=0)))
            return
        if alive:
            print(json.dumps(dict(record, running=True, already_running=True)))
            return
        if args.bundle is None:
            raise ValueError('--bundle is required for the initial preview')
        if not 1024 <= args.port <= 65535:
            raise ValueError('Invalid preview port')
        with (runtime / 'server.log').open('ab') as log:
            process = subprocess.Popen([sys.executable, str(Path(__file__).with_name('server.py')),
                '--bundle', str(args.bundle.resolve()), '--database', str(runtime / 'preview.sqlite3'),
                '--releases', str(ROOT / 'data/action_releases'), '--port', str(args.port)],
                cwd=ROOT, stdin=subprocess.DEVNULL, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        record = dict(pid=process.pid, start_ticks=identity(process.pid), port=args.port,
                      simulation=True, hardwareConnected=False)
        write(record_path, record)
        deadline = time.monotonic() + 40
        while time.monotonic() < deadline:
            if process.poll() is not None:
                raise RuntimeError('Preview exited: inspect ' + str(runtime / 'server.log'))
            try:
                with urlopen('http://127.0.0.1:' + str(args.port) + '/api/state', timeout=1) as response:
                    state = json.load(response)
                if state.get('simulation') is True and state.get('servicePid') == process.pid:
                    print(json.dumps(dict(record, ready=True)))
                    return
            except OSError:
                pass
            time.sleep(.2)
        process.send_signal(signal.SIGINT)
        process.wait(timeout=10)
        raise TimeoutError('Preview HTTP startup timed out; process stopped')


if __name__ == '__main__':
    main()
