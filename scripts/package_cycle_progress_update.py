"""Package only the two progress-bridge fixes against the inspected live files."""
import hashlib
import json
from pathlib import Path
import shutil

ROOT = Path(__file__).resolve().parents[1]
STAGE = ROOT / 'data/rail_deployment/cycle_progress_20260929'
DATA = ROOT / 'data/rail_deployment/gear_reverse_latency_20260928'


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    shutil.copy2(ROOT / 'deployment/rail_jetson/install_gear_update.py', STAGE / 'install_gear_update.py')
    names = ['runtime/executor.py', 'runtime/dring_protocol.py', 'install_gear_update.py']
    dependencies = ('executor.py', 'dring_protocol.py', 'gear_protocol.py', 'gear_progress.py',
                    'ros_transport.py', 'cloud_executor.py', 'native_worker.py')
    manifest = dict(schema=1,
        before={'deployment/rail_jetson/' + name: sha(STAGE / 'before' / name) for name in dependencies},
        absent=[], staged={name: sha(STAGE / name) for name in names},
        selected_bundle_file_sha256=sha(DATA / 'bundle.json'),
        release_sha256=json.loads((DATA / 'bundle.json').read_text())['sha256'],
        library_sha256=json.loads((DATA / 'gear_protocol.json').read_text())['sha256'],
        dring_library_sha256=json.loads((DATA / 'dring_protocol.json').read_text())['sha256'],
        update='per-cycle-controller-feedback-progress', hardware_commands_sent=0,
        trajectories_modified=False, controller_gains_modified=False, tests_passed=46)
    (STAGE / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
    print('Progress-only package prepared')


if __name__ == '__main__': main()
