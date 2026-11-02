"""Freeze successful rail runs without importing ROS or opening hardware."""
import argparse
import hashlib
import json
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
RUNS = {
    'push': '20260921_160930_476802/rail_plan_0066',
    'pull': '20260921_170539_419462/rail_plan_0007',
}


def digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':'),
                                     ensure_ascii=True, allow_nan=False).encode()).hexdigest()


def read(path):
    return json.loads(Path(path).read_text())


def write(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix('.tmp')
    temporary.write_text(json.dumps(value, indent=2, ensure_ascii=True, allow_nan=False) + '\n')
    temporary.replace(path)


def seconds(point):
    stamp = point['time_from_start']
    return stamp['sec'] + stamp['nanosec'] * 1e-9


def validate_trajectory(trajectory):
    names, points = trajectory['joint_names'], trajectory['points']
    if not names or len(set(names)) != len(names) or len(points) < 2:
        raise ValueError('Invalid trajectory shape')
    previous = -1.
    for point in points:
        t = seconds(point)
        if not math.isfinite(t) or t <= previous:
            raise ValueError('Nonmonotonic trajectory time')
        previous = t
        for field in ('positions', 'velocities', 'accelerations'):
            row = point[field]
            if len(row) != len(names) or not all(math.isfinite(v) for v in row):
                raise ValueError('Invalid trajectory field: ' + field)


def export(destination):
    destination = Path(destination)
    if destination.exists():
        raise ValueError('Use a new bundle directory; existing releases are immutable')
    actions = {}
    for operation, relative in RUNS.items():
        folder = ROOT / 'data/task_sessions' / relative
        plan, evidence, result = [read(folder / name) for name in
                                  ('plan.json', 'execution.json', 'rail_result.json')]
        if plan['rail']['operation'] != operation or plan['rail']['mode'] != 'cycle':
            raise ValueError('Wrong operation baseline')
        arms = [e for e in evidence if 'status' in e]
        if (not arms or any(e['status'] != 4 or e['error_code'] != 0 for e in arms)
                or len(arms) != sum(s['kind'] == 'arm' for s in plan['segments'])):
            raise ValueError('Baseline lacks successful arm results')
        if result['state'] != ('outer' if operation == 'pull' else 'inner_released'):
            raise ValueError('Missing complete operation result')
        for segment in plan['segments']:
            validate_trajectory(segment['trajectory'])
            if segment.get('trace_stage'):
                cache = read(ROOT / 'data/task_panel/rail_replay_cache' /
                             (segment['trace_report']['replay_cache']['key'] + '.json'))
                # Header stamps are transport metadata; all joint/time arrays must match.
                saved = cache['payload']['trajectory']
                for field in ('joint_names', 'points'):
                    if saved[field] != segment['trajectory'][field]:
                        raise ValueError('Plan differs from persisted curve: ' + segment['label'])
        actions[operation] = dict(source=relative, plan=plan, result=result,
                                  execution=evidence, teaching=read(folder / 'config.json'))
    ready = read(ROOT / 'data/task_panel/ready_poses.json')
    waist = actions['pull']['plan']['rail']['ready']['waist_yaw_joint']
    target = {}
    for side in ('left', 'right'):
        target.update({k: v for k, v in ready[side]['joints'].items()
                       if k.startswith(side + '_') and any(p in k for p in ('shoulder', 'elbow', 'wrist'))})
    target['waist_yaw_joint'] = waist
    payload = dict(schema=1, hardware_enabled=False, actions=actions,
                   initialization=dict(mode='simultaneous', velocity_scale=.7,
                                       acceleration_scale=.7, target=target),
                   mapping={'AIR': 'push', 'GROUND': 'pull'},
                   success_semantics='robot_sequence_completed_without_external_WOW_confirmation')
    destination.mkdir(parents=True)
    write(destination / 'bundle.json', dict(sha256=digest(payload), payload=payload))
    source = (ROOT / 'src/g1_unitree_control/src/arm_sdk_hardware_gateway.cpp').read_text()
    start = source.index('  TrajectorySample interpolate_locked(double elapsed) const\n')
    end = source.index('  void publish_preview_locked(', start)
    method = source[start:end]
    (destination / 'interpolation.inc').write_text(method)
    write(destination / 'provenance.json', dict(gateway_source_sha256=hashlib.sha256(source.encode()).hexdigest(),
        interpolation_sha256=hashlib.sha256(method.encode()).hexdigest(), baseline_runs=RUNS))
    return load(destination / 'bundle.json')


def load(path):
    envelope = read(path)
    payload = envelope['payload']
    if payload.get('schema') == 2:
        from releases import validate
        return validate(envelope)
    if envelope['sha256'] != digest(payload) or payload['schema'] != 1:
        raise ValueError('Deployment bundle integrity mismatch')
    if payload['mapping'] != {'AIR': 'push', 'GROUND': 'pull'}:
        raise ValueError('Frozen baseline mapping modified; use runtime configuration')
    return envelope


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('destination', type=Path)
    args = parser.parse_args()
    print(export(args.destination)['sha256'])
