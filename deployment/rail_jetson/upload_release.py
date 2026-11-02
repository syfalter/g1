"""Upload a verified bundle into an isolated Jetson release store; never activate/move."""
import argparse
import json
from pathlib import Path, PurePosixPath
import re
import shlex
import subprocess

from releases import load_release


def upload(bundle, target, root, run=subprocess.run):
    if not re.fullmatch(r'[a-zA-Z0-9_][a-zA-Z0-9_.-]*@[a-zA-Z0-9][a-zA-Z0-9_.-]*', target):
        raise ValueError('Expected SSH user@host, without options')
    path = PurePosixPath(root)
    if (not root.startswith('/home/unitree/rh56dfx/workspace/execution/') or '..' in path.parts
            or not re.fullmatch(r'[A-Za-z0-9_./-]+', root)):
        raise ValueError('Use an isolated execution workspace on Jetson')
    envelope = load_release(bundle)
    sha = envelope['sha256']
    remote = root.rstrip('/')
    ssh = ['ssh', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=8', target]
    def call(args):
        try:
            result = run(args, check=True, timeout=180, capture_output=True, text=True)
        except subprocess.CalledProcessError as error:
            raise RuntimeError('SSH transfer/import failed: ' + (error.stderr or str(error))[-1600:]) from error
        return result.stdout
    call(ssh + ['mkdir -p ' + shlex.quote(remote + '/incoming')])
    incoming = remote + '/incoming/' + sha + '.json'
    call(['scp', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=8', str(Path(bundle).resolve()),
          target + ':' + incoming])
    output = call(ssh + [shlex.join(['/usr/bin/python3', remote + '/deployment/rail_jetson/releases.py',
        '--store', remote + '/data/action_releases', 'import', incoming])])
    receipt = json.loads(output)
    if receipt['sha256'] != sha:
        raise RuntimeError('Jetson import receipt differs from uploaded release')
    return dict(code=0, version=receipt['version'], sha256=sha, uploaded=True,
                activated=False, hardware_commands_sent=0)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bundle', type=Path)
    parser.add_argument('--target', required=True)
    parser.add_argument('--root', required=True)
    args = parser.parse_args()
    print(json.dumps(upload(args.bundle, args.target, args.root), indent=2))
