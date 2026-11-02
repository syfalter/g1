"""Trim the gripping pull by 5 mm in root X, with a stopped release and C2 exit."""
import copy
import math
import shutil

import numpy as np
from scipy.optimize import brentq
from scipy.spatial.transform import Rotation
from trajectory_msgs.msg import JointTrajectory

from trajectory_only_candidates import (ROOT, BASE, read, load, decode, encode, evaluate,
    context_for, metrics, checked, refresh, write_json, OfflineModel, parameters_from_values,
    time_seconds, stamp)
from task_composition_preview import at, section, concatenate
from task_cruise_timing import compose_batch, materialize as materialize_clock
from task_optimizer_snapshot import save, DIRECTORY
from task_trajectory import coefficients

OUTPUT = ROOT / 'data/rail_deployment/pull_shorter_5mm_20260929'


def endpoint_pose(source, model, context, t):
    return model.batch_transforms(['right_index_tcp'], context, source.joint_names,
                                 evaluate(source, np.array([t])))['right_index_tcp'][0]


def find_cut(source, model, context, millimeters=5.):
    end = time_seconds(source.points[-1])
    target = endpoint_pose(source, model, context, end)[0, 3] + millimeters / 1000.
    times = np.linspace(0., end, math.ceil(end / .002) + 1)
    x = model.batch_transforms(['right_index_tcp'], context, source.joint_names,
                              evaluate(source, times))['right_index_tcp'][:, 0, 3]
    crossings = np.flatnonzero((x[:-1] >= target) & (x[1:] < target))
    if not len(crossings) or x[-1] >= x[0]:
        raise ValueError('Expected a negative-X pull crossing the shorter endpoint')
    i = int(crossings[-1])
    cut = brentq(lambda t:endpoint_pose(source, model, context, t)[0, 3] - target,
                 times[i], times[i + 1], xtol=1e-13)
    return round(cut * 1e9) / 1e9


def stop_on_prefix(source, cut, width):
    """Existing polynomial clock materializer preserves the retained spatial path."""
    lo = round((cut - width) * 1e9) / 1e9
    if lo <= 0.:
        raise ValueError('Braking window consumes the complete pull')
    distance = cut - lo
    span = 2. * distance
    # Source-time rate drops monotonically 1 -> 0, with zero rate derivative
    # at both ends. Split at every source knot before composing polynomials.
    warp = np.array([lo, span, 0., -span, .5 * span])
    source_knots = np.array([time_seconds(p) for p in source.points])
    knots = np.r_[lo, source_knots[(source_knots > lo) & (source_knots < cut)], cut]
    inverse = [0.] + [brentq(lambda u:np.polynomial.polynomial.polyval(u, warp) - t,
                            0., 1., xtol=1e-14) for t in knots[1:-1]] + [1.]
    pieces = []
    for a, b, u, v in zip(knots[:-1], knots[1:], inverse[:-1], inverse[1:]):
        owner = int(np.clip(np.searchsorted(source_knots, (a + b) / 2.) - 1, 0, len(source_knots) - 2))
        dt = source_knots[owner + 1] - source_knots[owner]
        poly = coefficients(source.points[owner], source.points[owner + 1], dt)
        local = np.array([np.polynomial.polynomial.polyval(u,
            np.polynomial.polynomial.polyder(warp, k)) * (v - u)**k / math.factorial(k) for k in range(5)])
        local[0] -= source_knots[owner]
        exact = compose_batch(poly[None], (local / dt)[None])[0]
        pieces.append((u * span, (v - u) * span, exact))
    brake, error = materialize_clock(pieces, source.joint_names, lambda:None)
    brake.points[0] = stamp(at(source, lo), 0.)
    end = at(source, cut)
    end.velocities = [0.] * len(source.joint_names)
    end.accelerations = [0.] * len(source.joint_names)
    brake.points[-1] = stamp(end, span)
    return concatenate([section(source, 0., lo), brake]), dict(
        source_cut_s=cut, braking_source_begin_s=lo, braking_seconds=span,
        removed_source_tail_s=time_seconds(source.points[-1]) - cut,
        clock_interpolation_bound_rad=error, method='retained_path_monotone_c2_braking_clock')


def connect_exit(source, release, hi):
    first = copy.deepcopy(release)
    last = at(source, hi)
    bridge = JointTrajectory(joint_names=list(source.joint_names),
        points=[stamp(first, 0.), stamp(last, hi)])
    return concatenate([bridge, section(source, hi, time_seconds(source.points[-1]))])


def main():
    model = OfflineModel(ROOT / 'data/trajectory_edit_audit/fast_timing_all/panel_robot.urdf')
    parent = read(BASE / 'bundle.json')
    prior = read(ROOT / 'data/rail_deployment/trajectory_only_20260929/pull_report.json')
    if prior['source_release_sha256'] != parent['sha256']:
        raise ValueError('Pull parent mismatch')
    saved = load(ROOT / prior['preview'], model)
    payload = saved['variants']['flow']['payload']
    parameters = parameters_from_values(parent['payload']['actions']['pull']['plan']['planning_config']['values'])
    original = decode(payload['units']['motion_1']['trajectory'])
    exit_original = decode(payload['units']['motion_2']['trajectory'])
    context = context_for(payload, 'motion_1')
    cut = find_cut(original, model, context)
    trials = []
    for width in (.15, .2, .25, .3, .4, .5, .6, .8, 1.):
        tr, evidence = stop_on_prefix(original, cut, width)
        try:
            peaks = checked(tr, model, parameters)
        except ValueError as exc:
            trials.append(dict(width_s=width, rejected=str(exc)))
            continue
        evidence.update(derivative_validation=peaks, trials=trials)
        break
    else:
        raise ValueError('No derivative-valid stopped prefix')
    before = endpoint_pose(original, model, context, time_seconds(original.points[-1]))
    after = endpoint_pose(tr, model, context, time_seconds(tr.points[-1]))
    shift = (after[:3, 3] - before[:3, 3]) * 1000.
    if abs(shift[0] - 5.) > .001:
        raise ValueError('Wrong shorter endpoint X')
    exit_trials = []
    exit_context = context_for(payload, 'motion_2')
    before_exit_metrics = metrics(exit_original, model, exit_context)[0]
    for hi in (.4, .5, .6, .75, 1., 1.25, 1.5, 1.75, 2.):
        departure = connect_exit(exit_original, tr.points[-1], hi)
        try:
            exit_peaks = checked(departure, model, parameters)
            diagnostic = metrics(departure, model, exit_context)[0]
            if diagnostic['stops']['count'] > before_exit_metrics['stops']['count']:
                raise ValueError('Exit adds an internal stop')
            if diagnostic['axis_excess_travel_mm'][0] > .05:
                raise ValueError('Exit adds an X reversal')
        except ValueError as exc:
            exit_trials.append(dict(rejoin_s=hi, rejected=str(exc)))
            continue
        break
    else:
        raise ValueError('No valid departure from the earlier release')
    before_metrics = metrics(original, model, context)[0]
    after_metrics = metrics(tr, model, context)[0]
    if after_metrics['stops']['count'] > before_metrics['stops']['count']:
        raise ValueError('Contact trim adds an internal stop')
    if after_metrics['below_5_mm_s'] > before_metrics['below_5_mm_s'] + .02:
        raise ValueError('Contact trim adds low-speed hesitation')
    evidence.update(before=before_metrics, selected=after_metrics, motion_limits_passed=True,
        publishable=False, intentional_geometry_change=True, physical_verified=False,
        hardware_executable=False, endpoint_shift_root_mm=shift.tolist(),
        endpoint_orientation_change_deg=float(np.rad2deg(Rotation.from_matrix(before[:3, :3].T @ after[:3, :3]).magnitude())),
        contact_path_policy='truncate existing path, not a pure Cartesian translation')
    exit_evidence = dict(rejoin_s=hi, before=before_exit_metrics, selected=diagnostic,
        derivative_validation=exit_peaks, trials=exit_trials, motion_limits_passed=True,
        publishable=False, physical_verified=False, hardware_executable=False,
        unchanged_after_rejoin=True, method='local_c2_exit_bridge_from_earlier_release')
    probes = np.linspace(0., hi, math.ceil(hi / .002) + 1)
    old_frames = model.batch_transforms(['right_index_tcp'], exit_context, exit_original.joint_names,
                                       evaluate(exit_original, probes))['right_index_tcp']
    new_frames = model.batch_transforms(['right_index_tcp'], exit_context, departure.joint_names,
                                       evaluate(departure, probes))['right_index_tcp']
    exit_evidence.update(sampled_tcp_difference_mm=float(np.max(np.linalg.norm(
        new_frames[:, :3, 3] - old_frames[:, :3, 3], axis=1)) * 1000.),
        sampled_orientation_difference_deg=float(np.rad2deg(np.max(Rotation.from_matrix(
            old_frames[:, :3, :3].transpose(0, 2, 1) @ new_frames[:, :3, :3]).magnitude()))),
        geometry_sampling_period_s=float(probes[1] - probes[0]),
        geometry_certification='sampled diagnostics, not collision or contact certification')
    hold = decode(payload['units']['hand_2']['trajectory'])
    for point in hold.points:
        point.positions = list(tr.points[-1].positions)
        point.velocities = [0.] * len(hold.joint_names)
        point.accelerations = [0.] * len(hold.joint_names)
    hold_evidence = dict(method='stationary_at_earlier_release', hand_command_unchanged=True,
        duration_unchanged=True, derivative_validation=checked(hold, model, parameters),
        motion_limits_passed=True, publishable=False)
    path = refresh(saved, {'motion_1':(tr, evidence), 'motion_2':(departure, exit_evidence),
                          'hand_2':(hold, hold_evidence)},
                   model, OUTPUT / 'individual', 'pull-shorter-x5mm-v1')
    changed = load(path, model)
    deployed = load(ROOT / parent['payload']['actions']['pull']['source'], model)
    variants = dict(original=copy.deepcopy(deployed['variants']['flow']),
                    baseline=copy.deepcopy(saved['variants']['flow']),
                    candidate=copy.deepcopy(changed['variants']['flow']))
    for key, result, pose, label in (
            ('original', variants['original'], before, '已部署 / 原松手点'),
            ('baseline', variants['baseline'], before, '当前有界优化 / 原松手点'),
            ('candidate', variants['candidate'], after, '少拉出 5 mm / 提前松手')):
        result['payload']['comparisonLabel'] = label
        result['payload']['optimizer']['variant'] = key
        result['payload']['spatialComparison'] = dict(focus='release',
            release_xyz_mm=(pose[:3, 3] * 1000.).tolist(), focus_xyz_mm=(pose[:3, 3] * 1000.).tolist(),
            reference_frame='URDF root', sampling_mm=.05, physical_verified=False)
    comparison = save(variants, copy.deepcopy(saved['snapshot_controls']), model, directory=OUTPUT / 'comparison')
    load(comparison, model)
    target = DIRECTORY / comparison.name
    shutil.copyfile(comparison, target)
    manifest_path = DIRECTORY / 'experiments.json'
    manifest = read(manifest_path) if manifest_path.exists() else {}
    entries = manifest.setdefault('rail:pull_cycle', [])
    entries[:] = [v for v in entries if v['id'] != 'pull-shorter-x5mm-v1']
    entries.append(dict(id='pull-shorter-x5mm-v1', label='拉出少 5 mm / 松手位置对比',
                        file=target.name, sha256=read(target)['sha256']))
    write_json(manifest_path, manifest)
    write_json(OUTPUT / 'report.json', dict(source_release_sha256=parent['sha256'],
        source_snapshot=prior['preview'], candidate_snapshot=str(path.relative_to(ROOT)),
        comparison_snapshot=str(target.relative_to(ROOT)), action='pull',
        source_speed_options=payload['options'], contact=evidence, exit=exit_evidence,
        release_before_root_mm=(before[:3, 3] * 1000.).tolist(),
        release_after_root_mm=(after[:3, 3] * 1000.).tolist(),
        approach_unchanged=True, hand_order_and_durations_unchanged=True,
        old_versions_preserved=True, physical_verified=False, deployed=False, hardware_commands_sent=0))
    print('REPORT', OUTPUT / 'report.json', 'SHIFT', shift, 'CONTACT_SECONDS', time_seconds(tr.points[-1]),
          'EXIT_REJOIN', hi, flush=True)


if __name__ == '__main__':
    main()
