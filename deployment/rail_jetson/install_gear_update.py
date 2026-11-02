"""Install an offline-verified gear mapping only on an idle detached Jetson."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time
from urllib.request import build_opener, ProxyHandler


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def health():
    with build_opener(ProxyHandler({})).open('http://127.0.0.1:8795/health', timeout=5.) as response:
        return json.load(response)


def detached(state):
    native = state.get('native', {})
    if (state.get('initialized') is not False or state.get('controllerRunning') is not False or
            state.get('activeTaskId') is not None or native.get('controlOwner') != 'none' or
            native.get('phase') != 'DETACHED' or
            state.get('automaticRecovery', {}).get('status') not in ('idle', 'ready', 'blocked')):
        raise RuntimeError('Update requires idle uninitialized DETACHED output with no recovery pending')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, required=True)
    parser.add_argument('--staging', type=Path, required=True)
    parser.add_argument('--check-only', action='store_true')
    args = parser.parse_args()
    root, stage = args.root.resolve(), args.staging.resolve()
    manifest = json.loads((stage / 'manifest.json').read_text())
    detached(health())
    for name, expected in manifest['before'].items():
        if sha(root / name) != expected:
            raise RuntimeError('Live code changed since inspection: ' + name)
    for name in manifest['absent']:
        if (root / name).exists():
            raise RuntimeError('New file already exists; inspect before updating: ' + name)
    for name, expected in manifest['staged'].items():
        if sha(stage / name) != expected:
            raise RuntimeError('Package checksum mismatch: ' + name)
    selected = root / 'config/selected_bundle.json'
    if sha(selected) != manifest['selected_bundle_file_sha256']:
        raise RuntimeError('Selected release changed since inspection; revalidate gear bindings')
    target_selected_sha = manifest.get('target_selected_bundle_file_sha256',
                                       manifest['selected_bundle_file_sha256'])
    if 'bundle.json' in manifest['staged']:
        sys.path[:0] = [str(stage / 'runtime'), str(root / 'deployment/rail_jetson')]
        from bundle import read
        from releases import validate
        from gear_protocol import GearLibrary
        if 'gear_protocol.json' not in manifest['staged']:
            raise RuntimeError('Trajectory update requires a matching gear library')
        release = validate(read(stage / 'bundle.json'))
        library = GearLibrary(read(stage / 'gear_protocol.json'), release)
        if 'dring_protocol.json' in manifest['staged']:
            from dring_protocol import DringLibrary
            dring = DringLibrary(read(stage / 'dring_protocol.json'), release)
            if dring.sha256 != manifest['dring_library_sha256']:
                raise RuntimeError('New D-ring library does not match manifest')
        if (sha(stage / 'bundle.json') != target_selected_sha or
                release['sha256'] != manifest['release_sha256'] or
                library.sha256 != manifest['library_sha256']):
            raise RuntimeError('New trajectory/library pair does not match manifest')
    if args.check_only:
        print('Preflight passed; no service restart or motion requested')
        return
    files = {'deployment/rail_jetson/' + Path(n).name: stage / n
             for n in manifest['staged'] if n.startswith('runtime/')}
    files.update({n: stage / n for n in manifest['staged'] if n.startswith('scripts/')})
    if 'gear_protocol.json' in manifest['staged']:
        files['config/gear_protocol.json'] = stage / 'gear_protocol.json'
    if 'bundle.json' in manifest['staged']:
        files['config/selected_bundle.json'] = stage / 'bundle.json'
    if 'dring_protocol.json' in manifest['staged']:
        files['config/dring_protocol.json'] = stage / 'dring_protocol.json'
    backup = root / 'data/deployment_backups' / ('gear-protocol-' + str(time.time_ns()))
    backup.mkdir(parents=True)
    originals = {}
    for name in files:
        source = root / name
        originals[name] = source.exists()
        if source.exists():
            destination = backup / name
            destination.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(source, destination)
    (backup / 'originals.json').write_text(json.dumps(originals, indent=2) + '\n')
    def service(command):
        subprocess.run(['sudo', '-n', 'systemctl', command, 'eln_executor.service'], check=True)
    detached(health())
    service('stop')
    processes = subprocess.check_output(['ps', '-eo', 'args'], universal_newlines=True).splitlines()
    binary = str(root / 'native_ws/install/g1_unitree_control/lib/g1_unitree_control/arm_sdk_hardware_gateway')
    if any(line.startswith(binary) for line in processes):
        raise RuntimeError('Retained gateway after service stop; no files changed; inspect manually')
    try:
        for name, source in files.items():
            target = root / name
            target.parent.mkdir(parents=True, exist_ok=True)
            tmp = target.with_name(target.name + '.gear-new')
            shutil.copy2(source, tmp)
            os.replace(tmp, target)
        service('start')
        deadline = time.monotonic() + 120.
        while True:
            try:
                state = health()
                break
            except Exception:
                if time.monotonic() >= deadline:
                    raise
                time.sleep(.5)
        detached(state)
        if (state.get('gearOperationConfigured') is not True or
                state.get('gearLibrarySha256') != manifest['library_sha256']):
            raise RuntimeError('Activated API did not load expected gear mapping')
        if sha(selected) != target_selected_sha:
            raise RuntimeError('Selected release changed during update')
        if ('release_sha256' in manifest and
                state.get('releaseSha256') != manifest['release_sha256']):
            raise RuntimeError('Activated API did not load expected release')
        if ('dring_library_sha256' in manifest and
                (state.get('dringCyclesConfigured') is not True or
                 state.get('dringLibrarySha256') != manifest['dring_library_sha256'])):
            raise RuntimeError('Activated API did not load expected D-ring library')
        receipt = dict(backup=str(backup), installed={n: sha(root / n) for n in files},
                       health=state, hardware_commands_sent=0, initialization_requested=False)
        (stage / 'activation_receipt.json').write_text(json.dumps(receipt, indent=2) + '\n')
        print('Gear mapping activated, controller remains DETACHED; no motion requested')
    except Exception:
        # Do not overwrite files if another operator has acquired output meanwhile.
        try:
            detached(health())
        except Exception:
            print('Activation unverified; inspect executor and backup before rollback: ' + str(backup))
            raise
        service('stop')
        for name, existed in originals.items():
            if existed:
                shutil.copy2(backup / name, root / name)
            elif (root / name).exists():
                (root / name).unlink()
        service('start')
        raise


if __name__ == '__main__':
    main()
