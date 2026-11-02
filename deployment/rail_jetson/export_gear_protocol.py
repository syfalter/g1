"""Offline export of the exact approved gear flow units; no robot connection."""
import argparse
import hashlib
from pathlib import Path
import sys

from bundle import ROOT, digest, read, write
from gear_protocol import BINDINGS, UNITS, GearLibrary
from releases import load_release


def export(release_path, urdf, output):
    sys.path.insert(0, str(ROOT / 'scripts'))
    from workbench_model import OfflineModel
    from task_optimizer_snapshot import load
    from task_offline_optimizer import decode, validate
    from task_planning_config import parameters_from_values
    release = load_release(release_path)
    base = release['payload']['actions'][BINDINGS[0]]
    source = ROOT / base['source']
    model = OfflineModel(urdf)
    saved = read(source)
    result = load(source, model)['variants']['flow']
    parameters = parameters_from_values(base['plan']['planning_config']['values'])
    units, reports = {}, {}
    for name in UNITS:
        trajectory = result['payload']['units'][name]['trajectory']
        reports[name] = validate(decode(trajectory), model, parameters, 5.)
        units[name] = dict(trajectory=trajectory, sha256=digest(trajectory))
    payload = dict(schema=1, contract='eln-gear-pinned-units-v1', hardware_enabled=False,
        snapshot_sha256=saved['sha256'], model_sha256=hashlib.sha256(Path(urdf).read_bytes()).hexdigest(),
        bindings={n: release['payload']['actions'][n]['source_plan_sha256'] for n in BINDINGS},
        units=units, validation=dict(method='pinned-v5-polynomial-validation', peaks=reports,
                                    physical_verified=False))
    library = dict(payload=payload, sha256=digest(payload))
    GearLibrary(library, release)
    write(output, library)
    print('Gear library exported: ' + library['sha256'])
    return library


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--release', type=Path, required=True)
    p.add_argument('--urdf', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args()
    export(a.release, a.urdf, a.output)
