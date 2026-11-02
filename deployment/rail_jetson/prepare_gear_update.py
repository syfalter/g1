"""Build a gear-only update from the inspected deployed baseline, entirely offline."""
import argparse
import json
from pathlib import Path
import shutil

from bundle import read
from gear_protocol import GearLibrary
from install_recovery_update import sha

FILES = ('automatic_recovery.py', 'executor.py', 'cloud_executor.py',
         'native_runtime.py', 'native_worker.py', 'production_api.py')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--staging', type=Path, required=True)
    parser.add_argument('--receipt', type=Path, required=True)
    parser.add_argument('--release', type=Path, required=True)
    args = parser.parse_args()
    stage = args.staging
    receipt = read(args.receipt)
    before = {}
    for name in FILES:
        target = 'deployment/rail_jetson/' + name
        before[target] = sha(stage / 'before' / name)
        if receipt['installed'][target] != before[target]:
            raise RuntimeError('Baseline differs from previous activation receipt: ' + name)
    here = Path(__file__).resolve().parent
    shutil.copy2(here / 'gear_protocol.py', stage / 'runtime/gear_protocol.py')
    shutil.copy2(here / 'install_gear_update.py', stage / 'install_gear_update.py')
    shutil.copy2(here / 'GEAR_PROTOCOL_UPDATE.md', stage / 'GEAR_PROTOCOL_UPDATE.md')
    shutil.copy2(here / 'test_gear_protocol.py', stage / 'test_gear_protocol.py')
    library = GearLibrary(read(stage / 'gear_protocol.json'), read(args.release))
    paths = ['runtime/' + name for name in FILES + ('gear_protocol.py',)]
    paths += ['gear_protocol.json', 'install_gear_update.py', 'GEAR_PROTOCOL_UPDATE.md', 'test_gear_protocol.py']
    manifest = dict(schema=1, before=before,
                    absent=['deployment/rail_jetson/gear_protocol.py', 'config/gear_protocol.json'],
                    staged={name: sha(stage / name) for name in paths},
                    bindings=library.payload['bindings'], library_sha256=library.sha256,
                    selected_bundle_file_sha256=receipt['selected_bundle_file_sha256'],
                    hardware_commands_sent=0, deployed=False)
    (stage / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
    print('Offline gear package prepared; no network connection or deployment performed')


if __name__ == '__main__':
    main()
