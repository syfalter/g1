"""Install a tested runtime only while detached. Never initialize or recover hardware."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import time
from urllib.request import build_opener, ProxyHandler


RUNTIME = ('automatic_recovery.py', 'executor.py', 'ros_transport.py', 'native_worker.py',
           'native_runtime.py', 'cloud_executor.py', 'production_api.py', 'select_production.py')
CPP = 'native_ws/src/g1_unitree_control/src/arm_sdk_hardware_gateway.cpp'
HEADER = 'native_ws/src/g1_unitree_control/include/g1_unitree_control/task_waist_rate.hpp'
BINARY = 'native_ws/install/g1_unitree_control/lib/g1_unitree_control/arm_sdk_hardware_gateway'


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def health():
    with build_opener(ProxyHandler({})).open('http://127.0.0.1:8795/health', timeout=5.) as response:
        return json.load(response)


def detached(state):
    n = state.get('native', {})
    if (state.get('initialized') is not False or state.get('controllerRunning') is not False or
            state.get('activeTaskId') is not None or n.get('controlOwner') != 'none' or
            n.get('phase') != 'DETACHED'):
        raise RuntimeError('Deployment requires idle, uninitialized, DETACHED output')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', required=True, type=Path)
    parser.add_argument('--staging', required=True, type=Path)
    args = parser.parse_args()
    root, staging = args.root.resolve(), args.staging.resolve()
    manifest = json.loads((staging / 'manifest.json').read_text())
    detached(health())
    for name, expected in manifest['before'].items():
        if sha(root / name) != expected:
            raise RuntimeError('Live file changed since inspection: ' + name)
    for name, expected in manifest['staged'].items():
        if sha(staging / name) != expected:
            raise RuntimeError('Staging checksum mismatch: ' + name)
    files = {('deployment/rail_jetson/' + n): staging / 'runtime' / n for n in RUNTIME}
    files.update({CPP: staging / 'native/arm_sdk_hardware_gateway.cpp',
                  HEADER: staging / 'native/task_waist_rate.hpp',
                  BINARY: staging / 'native/install/lib/g1_unitree_control/arm_sdk_hardware_gateway'})
    backup = root / 'data/deployment_backups' / ('recovery-waist-' + str(time.time_ns()))
    backup.mkdir(parents=True)
    changed = list(files) + ['config/native.yaml', 'native_ws/source_hashes.json']
    original = {}
    for name in changed:
        path = root / name
        original[name] = path.exists()
        if path.exists():
            target = backup / name
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(path, target)
    selected_sha = sha(root / 'config/selected_bundle.json')
    def service(command):
        subprocess.run(['sudo', '-n', 'systemctl', command, 'eln_executor.service'], check=True)
    detached(health())
    service('stop')
    processes = subprocess.check_output(['ps', '-eo', 'args'], text=True).splitlines()
    if any(line.startswith(str(root / BINARY)) for line in processes):
        raise RuntimeError('Gateway still exists after stop; no files changed, do not force-kill')
    try:
        for name, source in files.items():
            target = root / name
            tmp = target.with_name(target.name + '.recovery-new')
            shutil.copy2(source, tmp)
            os.replace(tmp, target)
        config = json.loads((root / 'config/native.yaml').read_text())
        p = config['g1_arm_sdk_hardware_gateway']['ros__parameters']
        if p['task_waist_velocity_limit_rad_s'] != .35 or p['max_command_step_rad'] != .006:
            raise RuntimeError('Inspected motion configuration changed')
        p['waist_command_rate_hz'] = 100.
        (root / 'config/native.yaml').write_text(json.dumps(config, indent=2) + '\n')
        source_hashes = json.loads((root / 'native_ws/source_hashes.json').read_text())
        for name in (CPP, HEADER):
            source_hashes[name.removeprefix('native_ws/src/') if hasattr(name, 'removeprefix') else
                          name[len('native_ws/src/'):]] = sha(root / name)
        (root / 'native_ws/source_hashes.json').write_text(json.dumps(source_hashes, indent=2) + '\n')
        service('start')
        state = health()
        detached(state)
        if sha(root / 'config/selected_bundle.json') != selected_sha:
            raise RuntimeError('Selected trajectories unexpectedly changed')
        if state.get('automaticRecovery', {}).get('status') != 'idle':
            raise RuntimeError('Startup unexpectedly scheduled recovery')
        receipt = dict(backup=str(backup), installed={name: sha(root / name) for name in changed},
                       selected_bundle_file_sha256=selected_sha, health=state,
                       hardware_commands_sent=0, initialization_requested=False)
        (staging / 'activation_receipt.json').write_text(json.dumps(receipt, indent=2) + '\n')
        print(json.dumps(dict(backup=str(backup), release=state['releaseVersion'],
                             controllerRunning=state['controllerRunning'],
                             automaticRecovery=state['automaticRecovery'], installed=True), indent=2))
    except Exception:
        # Never replace a controller that may have been initialized by somebody else.
        try:
            detached(health())
        except Exception:
            print('Activation not verified; inspect service and retained backup before rollback')
            raise
        service('stop')
        for name in changed:
            if original[name]:
                shutil.copy2(backup / name, root / name)
            elif (root / name).exists():
                (root / name).unlink()
        service('start')
        raise


if __name__ == '__main__':
    main()
