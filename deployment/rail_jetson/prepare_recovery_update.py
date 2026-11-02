"""Checksummed deployment manifest from inspected originals and tested staging."""
import argparse
import json
from pathlib import Path

from install_recovery_update import sha, RUNTIME, CPP, HEADER, BINARY


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, required=True)
    parser.add_argument('--staging', type=Path, required=True)
    args = parser.parse_args()
    root, staging = args.root, args.staging
    expected = {}
    for name in RUNTIME:
        original = staging / 'before' / name
        if original.exists():
            expected['deployment/rail_jetson/' + name] = sha(original)
        elif (root / 'deployment/rail_jetson' / name).exists():
            raise RuntimeError('New module already exists: ' + name)
    expected[CPP] = sha(staging / 'before/arm_sdk_hardware_gateway.cpp')
    for name in (BINARY, 'config/native.yaml', 'native_ws/source_hashes.json'):
        expected[name] = sha(root / name)
    for name, value in expected.items():
        if sha(root / name) != value:
            raise RuntimeError('Inspected original changed: ' + name)
    paths = ['runtime/' + n for n in RUNTIME] + ['native/arm_sdk_hardware_gateway.cpp',
        'native/task_waist_rate.hpp', 'native/install/lib/g1_unitree_control/arm_sdk_hardware_gateway']
    manifest = dict(before=expected, staged={n: sha(staging / n) for n in paths})
    (staging / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
    print('Manifest prepared; no active runtime files changed')


if __name__ == '__main__':
    main()
