"""Offline inventory and reversible pruning of the selected D-ring recordings."""
import argparse
import copy
import hashlib
import json
from pathlib import Path
import shutil
import time

import numpy as np

from task_screen_teach import write_json
from task_landing_gear_data import ENDPOINTS
from task_trace_selection import restore

ROOT = Path(__file__).resolve().parents[1]
KEEP = ('left_5', 'right_4')


def prune(paths, destination):
    sources = [(p, p.read_bytes()) for p in paths]
    replacements = []
    for path, blob in sources:
        config = json.loads(blob)
        if not all(k in config['traces'] for k in KEEP):
            raise ValueError('Missing requested baseline in '+str(path))
        updated = copy.deepcopy(config)
        updated['traces'] = {k:config['traces'][k] for k in KEEP}
        updated['retained_slots'] = list(KEEP)
        if 'trace_versions' in updated:
            updated['trace_versions'] = {k:v for k,v in updated['trace_versions'].items() if k in KEEP}
        replacements.append((path, blob, updated))
    destination.mkdir(parents=True, exist_ok=False)
    manifest = []
    for path, blob, updated in replacements:
        if path.read_bytes() != blob:raise RuntimeError('Source changed during migration')
        backup = destination/path.name
        shutil.copy2(path, backup)
        if backup.read_bytes() != blob:raise RuntimeError('Backup verification failed')
        manifest.append(dict(path=str(path), backup=str(backup), sha256=hashlib.sha256(blob).hexdigest(),
            removed_slots=[k for k in json.loads(blob)['traces'] if k not in KEEP]))
    write_json(destination/'manifest.json', dict(kept_slots=list(KEEP), files=manifest))
    for path, blob, updated in replacements:
        if path.read_bytes() != blob:raise RuntimeError('Source changed; migration stopped')
        write_json(path, updated)
    return manifest


def audit(root):
    path=root/'data/task_panel/landing_gear_demonstration.json'
    gear = restore(json.loads(path.read_text()),path)
    result = dict(hardware_commands_sent=0, landing_gear={}, endpoints={})
    for slot, trace in gear['traces'].items():
        q = np.asarray(trace['raw']['positions']); t = np.asarray(trace['raw']['time_s'])
        speed = np.max(np.abs(np.diff(q, axis=0))/np.diff(t)[:, None], axis=1)
        result['landing_gear'][slot] = dict(seconds=float(t[-1]), samples=len(t),
            peak_recorded_joint_rate_rad_s=float(np.max(speed)), median_recorded_joint_rate_rad_s=float(np.median(speed)))
    for endpoint in ('preparation', 'lower', 'upper'):
        rows = [(slot+':'+str(i), np.asarray(trace['raw']['positions'][i]))
                for slot, trace in gear['traces'].items()
                for i, name in zip((0, -1), ENDPOINTS[slot]) if name == endpoint]
        result['endpoints'][endpoint] = [dict(first=ka, second=kb,
            max_joint_delta_rad=float(np.max(np.abs(qa-qb))),
            joint_deltas_rad=(qb-qa).tolist())
            for a, (ka, qa) in enumerate(rows) for kb, qb in rows[a+1:]]
    candidates = list((root/'data/task_sessions').glob('*/rail_autosave/config.json'))
    if candidates:
        path = max(candidates, key=lambda p:p.stat().st_mtime_ns)
        rail = restore(json.loads(path.read_text()),path)
        result['rail'] = dict(source=str(path), slots={k:dict(seconds=v['raw']['time_s'][-1],
            samples=len(v['raw']['time_s'])) for k,v in rail.get('traces', {}).items()})
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--prune-dring', action='store_true')
    parser.add_argument('--output', type=Path, default=ROOT/'data/trajectory_edit_audit')
    args = parser.parse_args()
    result = audit(ROOT)
    if args.prune_dring:
        paths = [ROOT/'data/task_panel'/name for name in ('dring_demonstration.json', 'dring_demonstration_left.json')]
        result['pruning'] = prune(paths, args.output/('backup_'+str(time.time_ns())))
    write_json(args.output/'report.json', result)
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == '__main__':main()
