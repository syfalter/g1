"""Package the gear hand-policy fix without modifying any trajectory or live state."""
import json
from pathlib import Path
import shutil

from install_gear_update import sha


def main():
    root = Path(__file__).resolve().parents[2]
    stage = root / 'data/rail_deployment/gear_hand_policy_20260928'
    deployed = json.loads((root / 'data/rail_deployment/gear_protocol_20260928/activation_receipt.json').read_text())
    original = json.loads((root / 'data/rail_deployment/gear_protocol_20260928/manifest.json').read_text())
    before = {}
    for name in ('executor.py', 'gear_protocol.py'):
        target = 'deployment/rail_jetson/' + name
        before[target] = sha(stage / 'before' / name)
        if before[target] != deployed['installed'][target]:
            raise RuntimeError('Baseline changed: ' + name)
    before['deployment/rail_jetson/ros_transport.py'] = sha(stage / 'before/ros_transport.py')
    before['scripts/task_hand_completion.py'] = sha(stage / 'before/task_hand_completion.py')
    here = Path(__file__).resolve().parent
    shutil.copy2(here / 'gear_protocol.py', stage / 'runtime/gear_protocol.py')
    shutil.copy2(here / 'install_gear_update.py', stage / 'install_gear_update.py')
    (stage / 'scripts').mkdir(exist_ok=True)
    shutil.copy2(root / 'scripts/task_hand_completion.py', stage / 'scripts/task_hand_completion.py')
    paths = ['runtime/executor.py', 'runtime/gear_protocol.py', 'runtime/ros_transport.py',
             'scripts/task_hand_completion.py', 'install_gear_update.py']
    manifest = dict(schema=1, before=before, absent=[],
        staged={name: sha(stage / name) for name in paths},
        library_sha256=original['library_sha256'],
        selected_bundle_file_sha256=original['selected_bundle_file_sha256'],
        update='gear-hand-policy-only', trajectories_modified=False,
        hardware_commands_sent=0, deployed=False)
    (stage / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
    print('Hand-policy package prepared; no robot commands sent')


if __name__ == '__main__':
    main()
