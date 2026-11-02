#!/usr/bin/env python3
"""Start the isolated Jetson preview and establish its loopback-only SSH tunnel."""
import argparse
import json
from pathlib import Path
import re
import shlex
import subprocess
import time
from urllib.request import build_opener, ProxyHandler

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--local-port', type=int, default=8791)
    args = parser.parse_args()
    if not 1024 <= args.local_port <= 65535:
        raise ValueError('Invalid local port')
    config = json.loads((ROOT / 'config/jetson_push.json').read_text())
    target, remote = config['target'], config['root']
    if (not re.fullmatch(r'[a-zA-Z0-9_][a-zA-Z0-9_.-]*@[a-zA-Z0-9][a-zA-Z0-9_.-]*', target)
            or not remote.startswith('/home/unitree/rh56dfx/workspace/execution/') or '..' in Path(remote).parts):
        raise ValueError('Invalid Jetson deployment target')
    command = shlex.join(['/usr/bin/python3', remote + '/deployment/rail_jetson/preview_service.py',
        'start', '--bundle', remote + '/data/action_releases/baseline-20260921-v2/bundle.json', '--port', '8790'])
    process = subprocess.run(['ssh', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=8', target, command],
                             check=True, text=True, capture_output=True, timeout=60)
    service = json.loads(process.stdout)
    url = 'http://127.0.0.1:' + str(args.local_port)
    opener = build_opener(ProxyHandler({}))
    def healthy():
        try:
            with opener.open(url + '/api/state', timeout=2) as response:
                state = json.load(response)
            return state.get('simulation') is True and state.get('servicePid') == service['pid']
        except (OSError, ValueError):
            return False
    if not healthy():
        forward = '127.0.0.1:' + str(args.local_port) + ':127.0.0.1:8790'
        # Reuse the project's SSH master where available. Verify the endpoint,
        # not only ssh's exit code: an existing forward can outlive its session.
        result = subprocess.run(['ssh', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=8',
            '-O', 'forward', '-L', forward, target], capture_output=True, text=True, timeout=15)
        if result.returncode:
            subprocess.run(['ssh', '-fN', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=8',
                '-o', 'ExitOnForwardFailure=yes', '-L', forward, target],
                capture_output=True, text=True, timeout=15)
        for _ in range(20):
            if healthy():
                break
            time.sleep(.2)
        else:
            raise RuntimeError('Web tunnel unavailable; check SSH or choose another --local-port')
    print('Jetson version preview (simulation only): ' + url)
    print('WSL Execute and robot controller are unchanged. No motion sent.')


if __name__ == '__main__':
    main()
