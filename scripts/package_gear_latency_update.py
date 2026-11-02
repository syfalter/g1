"""Assemble a local-only update from the inspected Jetson runtime baseline."""
import hashlib
import json
from pathlib import Path
import shutil

ROOT = Path(__file__).resolve().parents[1]
STAGE = ROOT / 'data/rail_deployment/gear_reverse_latency_20260928'


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    # Existing staged runtime carries reviewed deltas against the Jetson baseline.
    # Do not replace it with older latency-only files when repackaging.
    shutil.copy2(ROOT / 'deployment/rail_jetson/release_validation_cache.py',
                 STAGE / 'runtime/release_validation_cache.py')
    shutil.copy2(ROOT / 'deployment/rail_jetson/dring_protocol.py', STAGE / 'runtime/dring_protocol.py')
    for name in ('install_gear_update.py', 'benchmark_release_validation.py'):
        shutil.copy2(ROOT / 'deployment/rail_jetson' / name, STAGE / name)
    report = json.loads((STAGE / 'build_report.json').read_text())
    names = ['runtime/' + p.name for p in sorted((STAGE / 'runtime').glob('*.py'))]
    names += ['bundle.json', 'gear_protocol.json', 'dring_protocol.json', 'install_gear_update.py', 'benchmark_release_validation.py']
    before = {'deployment/rail_jetson/' + n: sha(STAGE / 'before' / n)
              for n in ('executor.py', 'cloud_executor.py', 'production_api.py', 'releases.py',
                        'native_worker.py', 'native_runtime.py', 'automatic_recovery.py')}
    before['config/gear_protocol.json'] = sha(STAGE / 'before/gear_protocol.json')
    manifest = dict(schema=1, before=before,
        absent=['deployment/rail_jetson/release_validation_cache.py',
                'deployment/rail_jetson/dring_protocol.py', 'config/dring_protocol.json'],
        staged={n: sha(STAGE / n) for n in names},
        selected_bundle_file_sha256=sha(STAGE / 'before/selected_bundle.json'),
        target_selected_bundle_file_sha256=sha(STAGE / 'bundle.json'),
        release_sha256=report['release_sha256'], library_sha256=report['library_sha256'],
        dring_library_sha256=json.loads((STAGE / 'dring_protocol.json').read_text())['sha256'],
        update='dring-continuous-bounded-lowering-and-startup-latency', deployed=False,
        hardware_commands_sent=0, controller_gains_modified=False,
        requires_operator_safe_disable_confirmation=True)
    (STAGE / 'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    print('Local package prepared; no upload, restart or hardware request')


if __name__ == '__main__':
    main()
