#!/usr/bin/env python3
"""Freeze an inclusive history inventory. No ROS, inference, or hardware access."""
import argparse
from collections import Counter, defaultdict
import hashlib
import json
from pathlib import Path


def digest(path):
    h = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


def encoded(value):
    return (json.dumps(value, sort_keys=True, indent=2, allow_nan=False) + '\n').encode()


def read(path):
    return json.loads(path.read_text())


def records(path):
    return [json.loads(line) for line in path.read_text().splitlines() if line.strip()]


def inside(root, relative):
    path = (root / relative).resolve()
    if not path.is_relative_to(root.resolve()):
        raise ValueError('Path outside dataset: ' + relative)
    return path


def build(dataset):
    pairs = records(dataset / 'pairs.jsonl')
    contexts = records(dataset / 'cases.jsonl')
    by_pair = defaultdict(list)
    for context in contexts:
        by_pair[context['pair_id']].append(context)
    ids = {p['id'] for p in pairs}
    if len(ids) != len(pairs) or set(by_pair) - ids:
        raise ValueError('Duplicate pair ID or orphan context')
    rows = []
    failures = Counter()
    for pair in sorted(pairs, key=lambda p: p['id']):
        directory = inside(dataset, pair['directory'])
        for name, expected in pair['sha256'].items():
            if digest(inside(directory, name)) != expected:
                raise ValueError('Pair checksum mismatch: ' + pair['id'] + '/' + name)
        entries = []
        for context in sorted(by_pair[pair['id']], key=lambda c: c['directory']):
            path = inside(dataset, context['directory'])
            hashes = {str(p.relative_to(dataset)): digest(p)
                      for p in sorted(path.iterdir()) if p.is_file()}
            for name in ('ready_poses.json', 'session_home.json', 'index_tcp.json'):
                p = path.parent / name
                if p.is_file():
                    hashes[str(p.relative_to(dataset))] = digest(p)
            blockers = list(context['context_integrity_issues'])
            if not context['planning_replay_candidate']:
                blockers.append('not_replay_candidate')
            if context['recorded_kind'] != 'task':
                blockers.append('task_applicability_unresolved')
            if context['recorded_direction'] not in ('left', 'right'):
                blockers.append('historical_direction_missing')
            arm = context['recorded_arm']
            if arm not in ('left', 'right'):
                blockers.append('historical_arm_missing')
            for name in ('capture_state.json', 'geometry.json', 'manifest.json'):
                if not (path / name).is_file():
                    blockers.append('missing:' + name)
            ready = read(path.parent / 'ready_poses.json') if (path.parent / 'ready_poses.json').is_file() else {}
            if not (ready.get(arm) or {}).get('joints'):
                blockers.append('historical_ready_missing')
            if not (path.parent / 'session_home.json').is_file():
                blockers.append('historical_home_missing')
            failure = context.get('failure_diagnostic') or {}
            for item in failure.get('failures', [failure] if failure else []):
                cause = item.get('last_candidate', item)
                failures[(item.get('stage', 'unknown'), cause.get('code', 'unknown'),
                          cause.get('joint', ''), '|'.join(cause.get('pair', [])))] += 1
            entries.append(dict(directory=context['directory'], arm=arm,
                                direction=context['recorded_direction'],
                                recorded_kind=context['recorded_kind'], blockers=sorted(set(blockers)),
                                input_sha256=hashes))
        task_evidence = [e['directory'] for e in entries if e['recorded_kind'] == 'task']
        rows.append(dict(pair_id=pair['id'], directory=pair['directory'],
                         pair_sha256=pair['sha256'],
                         camera_calibration_sha256=hashlib.sha256(encoded(pair['calibration'])).hexdigest(),
                         recorded_dates=sorted({s['recorded_at'][:10] for s in pair['sources'] if s.get('recorded_at')}),
                         scene_group=None, scene_group_status='requires_visual_and_calibration_review',
                         applicability='recorded_task' if task_evidence else 'unresolved',
                         applicability_evidence=task_evidence,
                         detector_valid_3d=pair['valid_3d'], contexts=entries,
                         denominator_retained=True,
                         status='pending' if any(not e['blockers'] for e in entries) else 'blocked'))
    return dict(schema='g1.history.inventory.v1', dataset=str(dataset.resolve()),
                source_sha256={n: digest(dataset / n) for n in ('pairs.jsonl', 'cases.jsonl', 'summary.json')},
                policy=dict(unit='unique_rgbd_pair', case_budget_seconds=180,
                            direction='all_recorded_task_directions_per_pair; unknown blocks success',
                            exclusions='none; unresolved applicability remains in coverage denominator',
                            missing_state='never substitute current or synthetic state',
                            success='all stages validated and reloadable archive saved within budget',
                            split='unassigned until scene review; no validation-score claim',
                            final_task_denominator_frozen=False),
                pairs=rows,
                summary=dict(total_pairs=len(rows), applicability=dict(Counter(r['applicability'] for r in rows)),
                             readiness=dict(Counter(r['status'] for r in rows)),
                             replayable_contexts=sum(not e['blockers'] for r in rows for e in r['contexts'])),
                historical_candidate_failures=[dict(stage=k[0], code=k[1], joint=k[2], collision_pair=k[3], count=v)
                                               for k, v in sorted(failures.items())])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dataset', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.resolve().is_relative_to(args.dataset.resolve()):
        parser.error('Output must be outside immutable dataset')
    manifest = build(args.dataset.resolve())
    payload = encoded(manifest)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    # Exclusive creation preserves the pre-optimization snapshot.
    with args.output.open('xb') as stream:
        stream.write(payload)
    sha = hashlib.sha256(payload).hexdigest()
    with args.output.with_suffix(args.output.suffix + '.sha256').open('x') as stream:
        stream.write(sha + '  ' + args.output.name + '\n')
    print(json.dumps(dict(path=str(args.output), sha256=sha, **manifest['summary']), indent=2))


if __name__ == '__main__':
    main()
