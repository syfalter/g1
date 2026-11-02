"""Activate an explicitly authorized data-only trial; never initialize hardware."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time
from urllib.request import build_opener, ProxyHandler, Request


def read(path):
    return json.loads(path.read_text())


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def detached(state):
    native = state.get('native', {})
    if (state.get('initialized') is not False or state.get('controllerRunning') is not False
            or state.get('activeTaskId') is not None or native.get('controlOwner') != 'none'
            or native.get('phase') != 'DETACHED'
            or state.get('automaticRecovery', {}).get('status') != 'idle'):
        raise RuntimeError('Not idle, disabled and DETACHED; activation refused')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, required=True)
    parser.add_argument('--staging', type=Path, required=True)
    parser.add_argument('--activate', action='store_true')
    parser.add_argument('--sudo-password-stdin', action='store_true')
    args = parser.parse_args()
    sudo_password = sys.stdin.readline().rstrip('\n') if args.sudo_password_stdin else None
    root, stage = args.root.resolve(), args.staging.resolve()
    manifest = read(stage / 'export_report.json')
    before = read(stage / 'jetson_readonly/receipt.json')['hashes']
    if not manifest.get('user_authorized_trial') or not manifest.get('user_confirmed_disabled'):
        raise RuntimeError('Explicit trial authorization required')
    opener = build_opener(ProxyHandler({}))

    def health():
        with opener.open('http://127.0.0.1:8795/health', timeout=5) as response:
            return json.load(response)

    def verify_before():
        detached(health())
        for name, expected in before.items():
            if sha(root / name) != expected:
                raise RuntimeError('Parent changed: ' + name)

    verify_before()
    files = manifest['files']
    configs = {'config/selected_bundle.json', 'config/gear_protocol.json', 'config/dring_protocol.json'}
    if not configs.issubset(files):
        raise RuntimeError('Incomplete configuration set')
    for name, expected in files.items():
        path = Path(name)
        if path.is_absolute() or '..' in path.parts or path.suffix != '.json':
            raise RuntimeError('Invalid data path')
        if name not in configs and not name.startswith('data/rail_deployment/'):
            raise RuntimeError('Non-data write forbidden')
        if sha(stage / 'data_only' / name) != expected:
            raise RuntimeError('Package checksum mismatch')
        if name not in configs and (root / name).exists() and sha(root / name) != expected:
            raise RuntimeError('Immutable snapshot conflict')
    sys.path.insert(0, str(root / 'deployment/rail_jetson'))
    from releases import validate
    from gear_protocol import GearLibrary
    from dring_protocol import DringLibrary
    release = validate(read(stage / 'data_only/config/selected_bundle.json'))
    gear = GearLibrary(read(stage / 'data_only/config/gear_protocol.json'), release)
    dring = DringLibrary(read(stage / 'data_only/config/dring_protocol.json'), release)
    if (release['sha256'], gear.sha256, dring.sha256) != (
            manifest['release_sha256'], manifest['gear_library_sha256'], manifest['dring_library_sha256']):
        raise RuntimeError('Release identity mismatch')
    if not args.activate:
        print('Data-only preflight passed; nothing changed', flush=True)
        return
    backup = root / 'data/deployment_backups' / ('session-trial-' + str(time.time_ns()))
    backup.mkdir(parents=True)
    for name in configs:
        destination = backup / name
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(root / name, destination)
    (backup / 'manifest.json').write_text(json.dumps(dict(before=before, selected=manifest), indent=2))

    def service(command):
        options = ['-S', '-p', ''] if sudo_password is not None else ['-n']
        subprocess.run(['sudo', *options, 'systemctl', command, 'eln_executor.service'],
                       input=sudo_password + '\n' if sudo_password is not None else None,
                       text=True, check=True, timeout=210)

    def no_processes():
        processes = subprocess.check_output(['ps', '-eo', 'args'], text=True).splitlines()
        markers = ('/native_worker.py', '/production_api.py', '/arm_sdk_hardware_gateway')
        if any(str(root) in line and any(marker in line for marker in markers) for line in processes):
            raise RuntimeError('Retained runtime process; no forced termination or overwrite')

    def wait_health():
        deadline = time.monotonic() + 120
        while True:
            try:
                return health()
            except Exception:
                if time.monotonic() > deadline:
                    raise
                time.sleep(1)

    verify_before()
    service('stop')
    no_processes()
    try:
        for name in files:
            target = root / name
            target.parent.mkdir(parents=True, exist_ok=True)
            temporary = target.with_name(target.name + '.trial-new')
            shutil.copy2(stage / 'data_only' / name, temporary)
            os.replace(temporary, target)
        service('start')
        state = wait_health()
        detached(state)
        for key, expected in [('releaseSha256', release['sha256']), ('gearLibrarySha256', gear.sha256),
                              ('dringLibrarySha256', dring.sha256)]:
            if state.get(key) != expected:
                raise RuntimeError('API loaded wrong data: ' + key)
        token = (root / 'config/native.token').read_text().strip()
        request = Request('http://127.0.0.1:8794/state', data=b'{}',
                          headers={'X-Native-Token': token, 'Content-Type': 'application/json'})
        with opener.open(request, timeout=5) as response:
            native = json.load(response)
        for key, expected in [('releaseSha256', release['sha256']), ('gearLibrarySha256', gear.sha256),
                              ('dringLibrarySha256', dring.sha256)]:
            if native.get(key) != expected:
                raise RuntimeError('Native data mismatch: ' + key)
        for name, expected in files.items():
            if sha(root / name) != expected:
                raise RuntimeError('Installed data changed: ' + name)
        receipt = dict(backup=str(backup), health=state, native=native, installed=files,
                       hardware_commands_sent=0, initialization_requested=False,
                       physical_verified=False, user_authorized_trial=True)
        (stage / 'activation_receipt.json').write_text(json.dumps(receipt, indent=2) + '\n')
        print('Activated ' + manifest['version'] + '; remains DETACHED; backup=' + str(backup), flush=True)
    except Exception:
        try:
            detached(health())
            service('stop')
            no_processes()
            for name in configs:
                shutil.copy2(backup / name, root / name)
            service('start')
            detached(wait_health())
            print('Restored previous selected configuration', flush=True)
        except Exception as error:
            print('Rollback requires inspection: ' + str(error) + '; backup=' + str(backup), flush=True)
        raise


if __name__ == '__main__':
    main()
