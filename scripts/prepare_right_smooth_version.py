"""Prepare a bounded standalone right-stroke edit; never write a live bundle."""
import argparse
import copy
from datetime import datetime
import json
from pathlib import Path
import time

from task_screen_teach import fingerprint, write_json, now
from task_trace_versions import bank, delete, select

ROOT = Path(__file__).resolve().parents[1]
STORE = ROOT/'data/task_panel/dring_demonstration.json'
OUTPUT = ROOT/'data/composition_candidates/right_standalone_smooth_version.json'


def oldest_edit(entry):
    choices = []
    for index, trace in enumerate(entry['edits'], 1):
        if trace is not None:
            stamp = trace.get('edit_origin', {}).get('created_at')
            if not stamp:
                raise ValueError('Cannot order an edit with no timestamp')
            choices.append((datetime.fromisoformat(stamp).timestamp(), index))
    if not choices:
        raise ValueError('No editable versions to replace')
    return min(choices)[1]


def install_candidate(config, artifact):
    if fingerprint(config) != artifact['bundle_sha256']:
        raise ValueError('Bundle changed after candidate generation')
    slot = artifact['slot']; entry = bank(config, slot)
    if oldest_edit(entry) != artifact['replace_version']:
        raise ValueError('Oldest version changed')
    if artifact['replace_version'] in (entry['selected'], artifact['source_version']):
        raise ValueError('Cannot replace the active or source version')
    edited = copy.deepcopy(artifact['trace'])
    # Replace exactly the authorized oldest edit, even if another slot is empty.
    result = delete(config, slot, artifact['replace_version'])
    result['trace_versions'][slot]['edits'][artifact['replace_version']-1] = edited
    result = select(result, slot, entry['selected'])
    return result


def prepare(urdf):
    from task_trace_selection import restore
    from task_composition_preview import parent_of_trim, validate_timing
    from task_composition_smoothing import smooth_candidate
    from task_dring_teach_data import fit_trace, arrays, moving
    from task_fast_timing import fastest
    from task_trace_timing import _preserving, encode, stored_trajectory
    from task_planning_config import load_parameters
    from workbench_model import OfflineModel
    config = restore(json.loads(STORE.read_text()), STORE)
    slot = 'right_4'; entry = bank(config, slot); trace = config['traces'][slot]
    source_version = entry['selected']
    if trace.get('optimization', {}).get('kind') == 'trim_tail':
        trace, _, provenance = parent_of_trim(config, slot)
        source_version = provenance['parentVersion']
    replacement = oldest_edit(entry)
    if replacement in (source_version, entry['selected']):
        raise ValueError('Oldest edit is active or is the standalone source; refusing to replace it')
    model = OfflineModel(urdf); parameters, _ = load_parameters()
    started = time.monotonic()
    _, report = _preserving(trace, model, config['side'], parameters, workflow='dring_demo',
        config=config, slot=slot, velocity=1., acceleration=1., check=lambda:None,
        progress=lambda text:print(text, flush=True))
    original, _ = fit_trace(trace, model, config['side'])
    print('Continuous bounded spatial smoothing', flush=True)
    smoothed, bounds = smooth_candidate(original, model, config['side'],
        joint_tolerance=.003, tcp_tolerance=.001, window=.02)
    trajectory, search = fastest(smoothed, model, parameters, velocity=1., acceleration=1.)
    validate_timing(trajectory, model, parameters, lambda:None)
    from task_trajectory import time_seconds
    seconds = time_seconds(trajectory.points[-1])
    report.update(path_modified=True, smoothing=bounds, smoothed_search=search,
        method='bounded_spatial_smoothing_'+search['method'], duration_after_s=seconds,
        speedup_over_original_1x=report['original_1x_seconds']/seconds,
        speedup_over_original_fastest=report['original_fastest_seconds']/seconds,
        computation_seconds=time.monotonic()-started)
    edited = encode(trace, trajectory, report)
    edited['optimization']['label'] = '完整右划：偏差约束平滑＋尽快调速'
    edited['edit_origin'] = dict(slot=slot, version=source_version, operation='bounded_smooth_fastest',
                                created_at=now(), source_sha256=fingerprint(trace))
    stored_trajectory(edited, model, moving(config['side']))
    arrays(edited, model, config['side'])
    artifact = dict(schema=1, slot=slot, source_version=source_version, replace_version=replacement,
        bundle_sha256=fingerprint(config), trace=edited, hardware_commands_sent=0)
    install_candidate(config, artifact)
    write_json(OUTPUT, artifact)
    print(json.dumps(dict(output=str(OUTPUT), source=source_version, replace=replacement,
        source_seconds=trace['raw']['time_s'][-1], seconds=seconds, bounds=bounds,
        original_fastest_seconds=report['original_fastest_seconds']), ensure_ascii=False, indent=2))


def apply():
    import shutil
    import subprocess
    from task_trace_selection import restore, journal
    if subprocess.run(['systemctl', '--user', 'is-active', '--quiet',
                       'g1-trajectory-editor-offline.service']).returncode == 0:
        raise RuntimeError('Stop the offline editor before applying the version')
    config = restore(json.loads(STORE.read_text()), STORE)
    artifact = json.loads(OUTPUT.read_text())
    updated = install_candidate(config, artifact)
    side_store = STORE.with_name(STORE.stem+'_'+config['side']+STORE.suffix)
    backup = ROOT/'data/trajectory_edit_audit'/('right_smooth_'+datetime.now().strftime('%Y%m%d_%H%M%S'))
    backup.mkdir(parents=True, exist_ok=False)
    for path in (STORE, journal(STORE), side_store, journal(side_store)):
        if path.exists():shutil.copy2(path, backup/path.name)
    write_json(backup/'replacement.json', dict(slot=artifact['slot'], removed_version=artifact['replace_version'],
        candidate_sha256=fingerprint(artifact['trace']), previous_bundle_sha256=artifact['bundle_sha256']))
    write_json(STORE, updated)
    write_json(side_store, updated)
    print(json.dumps(dict(backup=str(backup), replaced=artifact['replace_version'],
        selected=bank(updated, artifact['slot'])['selected']), indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--urdf', type=Path)
    parser.add_argument('--apply', action='store_true')
    args = parser.parse_args()
    if args.apply:apply()
    elif args.urdf:prepare(args.urdf)
    else:parser.error('--urdf is required for generation')
