"""Build a latency-only patch against an inspected Jetson runtime."""
import json
from pathlib import Path
import shutil

from install_gear_update import sha


def main():
    root = Path(__file__).resolve().parents[2]
    here = Path(__file__).resolve().parent
    stage = root / 'data/rail_deployment/latency_motor_20260928'
    old = json.loads((root / 'data/rail_deployment/gear_protocol_20260928/manifest.json').read_text())
    before = {'deployment/rail_jetson/'+n: sha(stage / 'before' / n)
              for n in ('executor.py', 'cloud_executor.py', 'production_api.py')}
    for name in ('release_validation_cache.py',):
        shutil.copy2(here / name, stage / 'runtime' / name)
    for name in ('install_gear_update.py', 'benchmark_release_validation.py'):
        shutil.copy2(here / name, stage / name)
    paths = ['runtime/'+n for n in ('executor.py', 'cloud_executor.py', 'production_api.py',
                                   'release_validation_cache.py')]
    paths += ['install_gear_update.py', 'benchmark_release_validation.py']
    manifest = dict(schema=1, before=before,
        absent=['deployment/rail_jetson/release_validation_cache.py'],
        staged={n: sha(stage/n) for n in paths},
        library_sha256=old['library_sha256'],
        selected_bundle_file_sha256=old['selected_bundle_file_sha256'],
        update='startup-validation-and-redundant-alignment', deployed=False,
        hardware_commands_sent=0, trajectories_modified=False, controller_gains_modified=False)
    (stage / 'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    print('Latency package prepared, not activated')


if __name__ == '__main__':
    main()
