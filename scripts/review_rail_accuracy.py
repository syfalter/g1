"""Read-only rail geometry/tracking analysis; never imports hardware transports."""
import argparse
import csv
import hashlib
import json
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree

from task_hand_control import model_joints
from workbench_model import OfflineModel

ROOT = Path(__file__).resolve().parents[1]


def time_of(point):
    t = point['time_from_start']
    return t['sec'] + t['nanosec'] * 1e-9


def xyz(model, trajectory, context, link):
    return model.batch_transforms([link], context, trajectory['joint_names'],
        np.array([p['positions'] for p in trajectory['points']]))[link][:, :3, 3]


def metrics(times, positions):
    delta = np.diff(positions, axis=0)
    distance = np.linalg.norm(delta, axis=1)
    dt = np.diff(times)
    speed = distance / dt
    return dict(seconds=float(times[-1] - times[0]), path_mm=float(sum(distance) * 1000),
        chord_mm=float(np.linalg.norm(positions[-1] - positions[0]) * 1000),
        below_2_mm_s=float(sum(dt[speed < .002])),
        below_5_mm_s=float(sum(dt[speed < .005])),
        x_backward_mm=float(-sum(delta[:, 0][delta[:, 0] < 0]) * 1000),
        x_forward_mm=float(sum(delta[:, 0][delta[:, 0] > 0]) * 1000))


def review(release_path, urdf, log_root):
    release = json.loads(release_path.read_text())
    model = OfflineModel(urdf)
    report = dict(release_sha256=release['sha256'], model_sha256=hashlib.sha256(urdf.read_bytes()).hexdigest(),
        frame='URDF root/pelvis; +X forward; no measured base/world pose',
        warning='Finger FK uses commanded hand posture approximation, not measured fingertip/object position.', actions={})
    for name in ('push', 'pull'):
        action = release['payload']['actions'][name]
        saved = json.loads((ROOT / action['source']).read_text())['data']
        assert saved['model_sha256'] == report['model_sha256']
        context = dict(saved['variants']['flow']['payload']['context'])
        posture = next(s['target_q'] for s in action['program'] if s['kind'] == 'hand')
        context.update(model_joints('right', posture))
        result = dict(source=action['source'], variants={}, logs=[])
        for variant, value in saved['variants'].items():
            units = {}
            for key, unit in value['payload']['units'].items():
                if not key.startswith('motion_'):
                    continue
                trajectory = unit['trajectory']
                t = np.array([time_of(p) for p in trajectory['points']])
                p = xyz(model, trajectory, context, 'right_index_tcp')
                index = int(np.argmax(p[:, 0]))
                tail = t >= max(0, t[-1] - 10)
                near = np.linalg.norm(p - p[-1], axis=1) < .01
                first = int(np.flatnonzero(near)[0])
                units[key] = dict(metrics(t, p), peak_x_mm=float(p[index, 0] * 1000),
                    peak_time_s=float(t[index]), peak_xyz_mm=(p[index] * 1000).tolist(),
                    endpoint_mm=(p[-1] * 1000).tolist(), last_10_seconds=metrics(t[tail], p[tail]),
                    first_within_10_mm_s=float(t[first]), after_first_within_10_mm=metrics(t[first:], p[first:]))
            result['variants'][variant] = units
        # Match the archived controller trace to this immutable goal by initial target.
        trajectory = saved['variants']['flow']['payload']['units']['motion_0']['trajectory']
        names = trajectory['joint_names']
        planned = xyz(model, trajectory, context, 'right_index_tcp')
        tree = cKDTree(planned)
        for path in sorted(log_root.glob('*.csv')):
            with path.open() as stream:
                rows = list(csv.DictReader(stream))
            if not rows or not all(n + '_target' in rows[0] for n in names):
                continue
            target = np.array([[float(r[n + '_target']) for n in names] for r in rows])
            if max(abs(target[0] - np.array(trajectory['points'][0]['positions']))) > 1e-7:
                continue
            times = np.array([float(r['trajectory_progress_s']) for r in rows])
            if abs(max(times) - time_of(trajectory['points'][-1])) > .1:
                continue
            actual = np.array([[float(r[n + '_q']) for n in names] for r in rows])
            transforms = lambda q: model.batch_transforms(['right_index_tcp'], context, names, q)['right_index_tcp'][:, :3, 3]
            desired, measured = transforms(target), transforms(actual)
            error = (measured - desired) * 1000
            k = int(np.argmax(desired[:, 0]))
            window = abs(times - times[k]) < .25
            tail = times >= max(times) - 1.
            matches = tree.query(desired)[0] * 1000
            result['logs'].append(dict(file=path.name, samples=len(rows), seconds=float(max(times)),
                desired_path_nearest_vertex_max_mm=float(max(matches)),
                desired_peak_x_mm=float(max(desired[:, 0]) * 1000),
                measured_peak_x_mm=float(max(measured[:, 0]) * 1000),
                peak_time_s=float(times[k]), error_at_desired_peak_mm=error[k].tolist(),
                error_near_peak_median_mm=np.median(error[window], axis=0).tolist(),
                error_final_second_median_mm=np.median(error[tail], axis=0).tolist(),
                peak_joint_errors_rad=dict(zip(names, (actual[k] - target[k]).tolist())),
                minimum_time_scale=float(min(float(r['time_scale']) for r in rows if r['event'] == 'sample'))))
        report['actions'][name] = result
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--release', type=Path, default=ROOT / 'data/rail_deployment/approved_mirror_20260928/bundle.json')
    parser.add_argument('--urdf', type=Path, required=True)
    parser.add_argument('--logs', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    report = review(args.release, args.urdf, args.logs)
    args.output.write_text(json.dumps(report, indent=2) + '\n')
    for name, result in report['actions'].items():
        print(name)
        for variant, units in result['variants'].items():
            print(variant, json.dumps(units['motion_0']))
        print('LOGS', json.dumps(result['logs']))


if __name__ == '__main__':
    main()
