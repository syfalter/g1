"""Select exact published actions for a stopped standalone Jetson installation.

This writes data/configuration only. It cannot start, ARM or move the robot.
"""
import argparse
import fcntl
import hashlib
import json
import os
from pathlib import Path
import secrets
import sys

from bundle import ROOT, write
from releases import load_release, package
sys.path.insert(0, str(ROOT / 'scripts'))
from task_planning_config import parameters_from_values


def configure(root, bundles):
    import yaml
    root = Path(root).resolve()
    (root / 'data').mkdir(parents=True, exist_ok=True)
    with (root / 'data/runtime.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        releases = [load_release(path) for path in bundles]
        if any(r['payload'].get('test_only') for r in releases):
            raise ValueError('Synthetic acceptance fixtures cannot be selected for production')
        actions = {}
        initialization = releases[0]['payload']['initialization']
        for release in releases:
            if release['payload']['initialization'] != initialization:
                raise ValueError('Selected modules use different preparation targets; select compatible recordings')
            for name, action in release['payload']['actions'].items():
                if name in actions:
                    raise ValueError('Select exactly one source for ' + name)
                actions[name] = action
        key = hashlib.sha256(json.dumps([r['sha256'] for r in releases]).encode()).hexdigest()[:20]
        envelope = package('production-' + key, actions, initialization)
        model = root / 'assets/planner.urdf'
        model_sha = hashlib.sha256(model.read_bytes()).hexdigest()
        controller = None
        for action in actions.values():
            identity = action['identity']
            if identity.get('robot_sha256', identity.get('urdf_sha256')) != model_sha:
                raise ValueError('Selected recording does not match the deployed planner URDF')
            selected = parameters_from_values(action['plan']['planning_config']['values']).controller_parameters('left')
            if controller is not None and selected != controller:
                raise ValueError('Selected modules require different startup gains/limits')
            controller = selected
        base = yaml.safe_load((root / 'assets/gateway_defaults.yaml').read_text())['g1_arm_sdk_hardware_gateway']['ros__parameters']
        base.update(controller)
        base.update(active_arm='left', task_application_enabled=True, task_waist_enabled=True,
                    waist_command_rate_hz=100.,
                    task_dual_prepare_enabled=True, task_client_watchdog_s=5.,
                    hardware_output_enabled=False, hardware_authorization='',
                    gravity_urdf_path=str(root / 'assets/gravity.urdf'),
                    left_payload_fit_path=str(root / 'assets/left_payload.yaml'),
                    right_payload_fit_path=str(root / 'assets/right_payload.yaml'),
                    trajectory_log_directory=str(root / 'data/native_motion_logs'))
        (root / 'config').mkdir(exist_ok=True)
        write(root / 'config/selected_bundle.json', envelope)
        # YAML accepts JSON documents; preserve structured numeric and bool types.
        write(root / 'config/native.yaml', {'g1_arm_sdk_hardware_gateway': {'ros__parameters': base}})
        for filename in ('native.token', 'api.key'):
            path = root / 'config' / filename
            if not path.exists():
                fd = os.open(str(path), os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
                with os.fdopen(fd, 'w') as stream:
                    stream.write(secrets.token_urlsafe(48) + '\n')
        return dict(version=envelope['payload']['version'], sha256=envelope['sha256'],
                    selectedActions=list(actions), sourceVerification={k:a['verification'] for k,a in actions.items()},
                    hardware_commands_sent=0, service_started=False)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, default=ROOT)
    parser.add_argument('--bundle', type=Path, required=True, action='append')
    args = parser.parse_args()
    print(json.dumps(configure(args.root, args.bundle), indent=2))
