"""Offline-only bounded pull-approach tail experiment. Never selects or deploys it."""
import argparse
import json
from pathlib import Path

import numpy as np
from rosidl_runtime_py.convert import message_to_ordereddict

from task_composition_preview import at, section, concatenate
from task_offline_optimizer import decode, validate
from task_planning_config import parameters_from_values
from task_stop_blending import spatial_bound, materialize
from task_trajectory import time_seconds, peak, coefficients
from task_trajectory_speed import motion_limits
from workbench_model import OfflineModel

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--urdf', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    release = json.loads((ROOT / 'data/rail_deployment/approved_mirror_20260928/bundle.json').read_text())
    action = release['payload']['actions']['pull']
    snapshot = json.loads((ROOT / action['source']).read_text())['data']
    variants = snapshot['variants']
    source = decode(variants['flow']['payload']['units']['motion_0']['trajectory'])
    original = decode(variants['baseline']['payload']['units']['motion_0']['trajectory'])
    duration, old_duration = time_seconds(source.points[-1]), time_seconds(original.points[-1])
    parameters = parameters_from_values(action['plan']['planning_config']['values'])
    model = OfflineModel(args.urdf)
    vmax, amax = motion_limits(source, model, 1., 1., parameters)
    limits = [[d[n] for n in source.joint_names] for d in (vmax, amax, dict.fromkeys(source.joint_names, 5.))]
    trials, best = [], None
    for begin in (12., 13., 14., 14.4, 15., 16., 17., 18.):
        original_begin = old_duration - (duration - begin)
        reference = section(original, original_begin, old_duration)
        # Only touch an unchanged tail, so the original path budget is not spent twice.
        current_tail = section(source, begin, duration)
        from task_composition_smoothing import evaluate
        times = np.linspace(0., duration - begin, 501)
        if max(abs(evaluate(reference, times) - evaluate(current_tail, times)).ravel()) > 1e-7:
            trials.append(dict(begin=begin, rejected='tail differs from frozen original'))
            continue
        for span in (.75, 1., 1.25, 1.5, 2., 2.5, 3., 4., 5.):
            # Match position/velocity/acceleration, not the noisy recorded jerk.
            # Jerk remains bounded on both sides; C3 continuity is not claimed.
            poly = np.pad(coefficients(at(source, begin), at(source, duration), span), ((0, 2), (0, 0)))
            if any(peak(np.polynomial.polynomial.polyder(poly, k)[:, axis]) / span**k > limit * .999
                   for k, values in enumerate(limits, 1) for axis, limit in enumerate(values)):
                trials.append(dict(begin=begin, span=span, rejected='derivative limit'))
                continue
            bounds = spatial_bound(reference, poly, model, 'right')
            if any(bounds[k] > limit for k, limit in
                   (('joint_rad', .015), ('tcp_m', .005), ('orientation_rad', np.deg2rad(2.5)))):
                trials.append(dict(begin=begin, span=span, rejected='original path tolerance', bounds=bounds))
                continue
            bridge, error = materialize(poly, span, source.joint_names, at(source, begin),
                                        at(source, duration), lambda: None)
            try:
                peaks = validate(bridge, model, parameters, 5.)
            except ValueError as exc:
                trials.append(dict(begin=begin, span=span, rejected=str(exc)))
                continue
            candidate = concatenate([section(source, 0., begin), bridge])
            item = dict(method='bounded_c2_terminal_refit', begin=begin, span=span, seconds=time_seconds(candidate.points[-1]), bounds=bounds,
                        interpolation_error_rad=error, peaks=peaks)
            trials.append(item)
            if best is None or item['seconds'] < best[0]['seconds']:
                best = item, candidate
    report = dict(source=action['source'], source_release_sha256=release['sha256'],
        original_seconds=duration, trials=trials, hardware_executable=False, physical_verified=False,
        collision_checked=False, selected=False, deployed=False)
    if best:
        item, trajectory = best
        report['candidate'] = dict(item, trajectory=message_to_ordereddict(trajectory),
                                   full_validation=validate(trajectory, model, parameters, 5.))
        assert trajectory.points[-1].positions == source.points[-1].positions
        assert trajectory.points[-1].velocities == source.points[-1].velocities
        assert trajectory.points[-1].accelerations == source.points[-1].accelerations
        print(json.dumps({k: v for k, v in item.items() if k != 'peaks'}), flush=True)
    else:
        print('No candidate met the original-path budget', flush=True)
    args.output.write_text(json.dumps(report, indent=2) + '\n')


if __name__ == '__main__':
    main()
