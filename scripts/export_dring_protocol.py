"""Export existing optimized loop units without modifying or retiming their paths."""
import argparse
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'deployment/rail_jetson'))
from bundle import digest, read, write
from dring_protocol import DringLibrary
from task_optimizer_snapshot import load
from task_offline_optimizer import decode, validate
from task_planning_config import parameters_from_values
from workbench_model import OfflineModel


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--release', type=Path, required=True)
    parser.add_argument('--urdf', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    release = read(args.release)
    action = release['payload']['actions']['dring_pair']
    model = OfflineModel(args.urdf)
    saved = load(ROOT / action['source'], model)
    proof = action['plan']['export_validation']
    if read(ROOT / action['source'])['sha256'] != proof['snapshot_sha256']:
        raise ValueError('D-ring source snapshot differs from pinned release')
    units = saved['variants']['flow']['payload']['units']
    params = parameters_from_values(action['plan']['planning_config']['values'])
    payload = dict(schema=1, contract='eln-dring-pinned-units-v1', hardware_enabled=False,
        binding=action['source_plan_sha256'], snapshot_sha256=proof['snapshot_sha256'],
        model_sha256=proof['model_sha256'],
        units={k: dict(trajectory=v['trajectory'], sha256=digest(v['trajectory'])) for k, v in units.items()},
        validation={k: validate(decode(v['trajectory']), model, params, 5.) for k, v in units.items()})
    result = dict(payload=payload, sha256=digest(payload))
    library = DringLibrary(result, release)
    # Check repeated joins with the same continuous derivative validator as export.
    for count in (1, 2, 3, 8, 9):
        for step in library.program(dict(count=count)):
            if step['kind'] == 'recorded':
                validate(decode(step['trajectory']), model, params, 5.)
    write(args.output, result)
    print(library.describe(dict(count=2)))


if __name__ == '__main__':
    main()
