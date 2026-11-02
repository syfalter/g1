"""Local 0/3/4 mm IK previews, with explicit unmeasured-axis and exit gates."""
import argparse
import copy
from pathlib import Path

import numpy as np
from scipy.interpolate import make_interp_spline
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from trajectory_only_candidates import (ROOT, BASE, read, load, decode, encode, evaluate,
    context_for, metrics, checked, refresh, write_json, OfflineModel, parameters_from_values,
    time_seconds, stamp, ledger)


def smooth7(u):
    u = np.clip(u, 0., 1.)
    return 35*u**4 - 84*u**5 + 70*u**6 - 20*u**7


def weights(times, lo, top, end_top, hi):
    if not lo < top <= end_top < hi:
        raise ValueError('Invalid local compensation support')
    return smooth7((times-lo)/(top-lo)) * (1.-smooth7((times-end_top)/(hi-end_top)))


def compensation(source, model, context, axis, millimeters, support):
    if millimeters not in (0., 3., 4., 5., 6., 7., 8., 9.):
        raise ValueError('Only explicitly requested compensation amplitudes are supported')
    axis = np.asarray(axis, dtype=float)
    if axis.shape != (3,) or not np.isfinite(axis).all() or abs(np.linalg.norm(axis)-1.) > 1e-8:
        raise ValueError('Compensation axis must be a finite unit vector')
    if millimeters == 0:
        return copy.deepcopy(source), dict(amplitude_mm=0., exact_identity=True)
    lo, top, end_top, hi = support
    names = source.joint_names
    active = [i for i, n in enumerate(names) if n.startswith('right_')]
    if len(active) != 7:
        raise ValueError('Expected seven arm joints; waist compensation is forbidden')
    times = np.unique(np.round(np.r_[np.linspace(lo, hi, int(np.ceil((hi-lo)/.025))+1),
        np.linspace(end_top, hi, max(12,int(np.ceil((hi-end_top)/.025))+1)), top, end_top]*1e9)/1e9)
    original = evaluate(source, times)
    frames = model.batch_transforms(['right_index_tcp'], context, names, original)['right_index_tcp']
    weight = weights(times, *support)
    shifts = np.zeros_like(original)
    residuals = []
    previous = np.zeros(7)
    for i, (q, frame, w) in enumerate(zip(original, frames, weight)):
        if i in (0, len(times)-1):
            continue
        target = frame[:3, 3] + axis * millimeters * .001 * w
        base = q[active]
        lower = np.maximum(-.15, [model.limits[names[j]][0]+.02-base[k] for k,j in enumerate(active)])
        upper = np.minimum(.15, [model.limits[names[j]][1]-.02-base[k] for k,j in enumerate(active)])
        def residual(delta):
            state = dict(context, **dict(zip(names, q)))
            state.update({names[j]: float(base[k]+delta[k]) for k,j in enumerate(active)})
            pose = model.transform('right_index_tcp', state)
            angular = Rotation.from_matrix(frame[:3, :3] @ pose[:3, :3].T).as_rotvec()
            return np.r_[(pose[:3, 3]-target)*1000., angular*150., delta*.01]
        fit = least_squares(residual, np.clip(previous, lower+1e-10, upper-1e-10),
                            bounds=(lower, upper), ftol=1e-11, xtol=1e-11, gtol=1e-11, max_nfev=80)
        if not fit.success or np.linalg.norm(fit.fun[:3]) > .005 or np.linalg.norm(fit.fun[3:6])/150 > 1e-4:
            raise ValueError('IK target not reached continuously')
        shifts[i, active] = fit.x
        previous = fit.x
        residuals.append(float(np.linalg.norm(fit.fun[:3])))
    zeros = np.zeros(len(names))
    spline = make_interp_spline(times, shifts, k=5,
                               bc_type=([(1, zeros), (2, zeros)], [(1, zeros), (2, zeros)]))
    # Add a spline to the original polynomial, never refit the original path.
    # Union knots represent their sum exactly as quintic command intervals.
    knots = np.unique(np.r_[[time_seconds(p) for p in source.points], spline.t])
    values = [evaluate(source, knots, k) for k in range(3)]
    inside = (knots > lo) & (knots < hi)
    for k in range(3):
        values[k][inside] += spline(knots[inside], nu=k)
    result = JointTrajectory(joint_names=list(names))
    for i, t in enumerate(knots):
        result.points.append(stamp(JointTrajectoryPoint(positions=values[0][i].tolist(),
            velocities=values[1][i].tolist(), accelerations=values[2][i].tolist()), t))
    for k in range(3):
        probes = np.r_[np.linspace(0., lo, 100), np.linspace(hi, knots[-1], 100)]
        if np.max(abs(evaluate(result, probes, k)-evaluate(source, probes, k))) > 1e-7:
            raise ValueError('Compensation changed protected path or boundary derivatives')
    probes = np.linspace(lo, hi, max(100, int(np.ceil((hi-lo)/.002))+1))
    old = model.batch_transforms(['right_index_tcp'], context, names, evaluate(source, probes))['right_index_tcp']
    new = model.batch_transforms(['right_index_tcp'], context, names, evaluate(result, probes))['right_index_tcp']
    intended = weights(probes, *support)[:, None] * axis * millimeters * .001
    residual = new[:, :3, 3] - old[:, :3, 3] - intended
    rotation = Rotation.from_matrix(old[:, :3, :3].transpose(0, 2, 1) @ new[:, :3, :3]).magnitude()
    evidence = dict(amplitude_mm=millimeters, axis_root=axis.tolist(), support_s=list(support),
                    peak_target_residual_mm=float(np.max(np.linalg.norm(residual, axis=1))*1000),
                    peak_orientation_change_deg=float(np.rad2deg(max(rotation))),
                    knot_ik_residual_mm=max(residuals), waist_unchanged=True,
                    outside_support_qva_unchanged=True, geometry_sampling_period_s=float(probes[1]-probes[0]),
                    geometry_certification='sampled FK, not a continuous collision/contact proof',
                    added_smoothing_budget_mm=0.)
    if evidence['peak_target_residual_mm'] > .02 or evidence['peak_orientation_change_deg'] > .05:
        raise ValueError('Interpolated compensation does not preserve intended pose')
    return result, evidence


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output = args.output.resolve()
    model = OfflineModel(ROOT / 'data/trajectory_edit_audit/fast_timing_all/panel_robot.urdf')
    bundle = read(BASE / 'bundle.json')
    action = bundle['payload']['actions']['push']
    saved = load(ROOT / action['source'], model)
    payload = saved['variants']['flow']['payload']
    source = decode(payload['units']['motion_0']['trajectory'])
    context = context_for(payload, 'motion_0')
    params = parameters_from_values(action['plan']['planning_config']['values'])
    before, _ = metrics(source, model, context)
    single = load(ROOT / 'data/offline_optimizer_previews' / (payload['sources']['push']['snapshot']+'.json'), model)
    original = single['variants']['baseline']['trajectory']
    endpoints = model.batch_transforms(['right_index_tcp'], context, original.joint_names,
        np.array([original.points[0].positions, original.points[-1].positions]))['right_index_tcp'][:, :3, 3]
    axis = endpoints[1]-endpoints[0]
    axis /= np.linalg.norm(axis)
    # Protect the entire old blend crossing the original retreat boundary.
    # A nearest-position match in the smoothed flow is NOT a role boundary.
    retreat = load(ROOT / 'data/offline_optimizer_previews' / (payload['sources']['retreat_B']['snapshot']+'.json'), model)
    target = retreat['variants']['baseline']['trajectory'].points[0].positions
    baseline = saved['variants']['baseline']['payload']['units']['motion_0']['trajectory']
    baseline = decode(baseline)
    error = [np.max(abs(np.array(p.positions)-target)) for p in baseline.points]
    if min(error) > 1e-8:
        raise ValueError('Original retreat boundary is not retained in frozen baseline')
    original_cutoff = time_seconds(baseline.points[int(np.argmin(error))])
    prior = payload['flowEvidence']['motion_0']
    intervals, _ = ledger(prior)
    cutoff = None
    for edit, (lo, hi, _) in zip(sorted(prior['edits'],key=lambda e:e['source_begin_s']),intervals):
        if edit['source_begin_s'] <= original_cutoff <= edit['source_end_s']:
            cutoff = lo
            break
    if cutoff is None:
        raise ValueError('No certified original-to-flow retreat-boundary mapping')
    peak_time = before['peak_time_s']
    report = dict(source_release_sha256=bundle['sha256'], source_snapshot=action['source'], before=before,
        axis_root=axis.tolist(), axis_provenance='unmeasured hypothesis: frozen push endpoint chord in URDF root',
        angle_to_root_forward_deg=float(np.rad2deg(np.arccos(axis[0]))),
        physical_axis_confirmed=False, retreat_cutoff_s=cutoff, original_retreat_start_s=original_cutoff,
        retreat_boundary_match_rad=min(error), retreat_cutoff_policy='protect entire historical blend crossing retreat start',
        warning='FK and commanded finger posture do not measure object travel, deformation or mechanical end stop.',
        physical_verified=False, collision_checked=False, deployed=False, hardware_commands_sent=0, candidates={})
    for mode, end in (('strict_exit', cutoff), ('transition_review', peak_time+1.5)):
        for mm in (0., 3., 4.):
            if mode == 'transition_review' and mm == 0:
                continue
            label = mode + '_' + str(int(mm)) + 'mm'
            print(label, flush=True)
            support = (round((peak_time-2.5)*1e9)/1e9, round((peak_time-.2)*1e9)/1e9,
                       round(peak_time*1e9)/1e9, round(end*1e9)/1e9)
            candidate, evidence = compensation(source, model, context, axis, mm, support)
            evidence.update(metrics=metrics(candidate, model, context)[0],
                publishable=False, physical_axis_confirmed=False,
                original_retreat_preserved=(mode == 'strict_exit'),
                requires_exit_transition_approval=(mode != 'strict_exit'),
                rejection_gates=['physical rail axis unmeasured', 'mechanical contact/end-stop clearance unverified'])
            try:
                evidence['derivative_validation'] = checked(candidate, model, params)
                evidence['motion_limits_passed'] = True
            except ValueError as exc:
                evidence.update(motion_limits_passed=False, validation_error=str(exc))
                evidence['rejection_gates'].append(str(exc))
            if mode != 'strict_exit':
                evidence['rejection_gates'].append('changes first part of exit: explicit approval required')
            path = refresh(saved, {'motion_0': (candidate, evidence)}, model,
                           args.output / 'depth_previews' / label, label+'-UNAPPROVED')
            evidence['preview'] = str(path.relative_to(ROOT))
            report['candidates'][label] = evidence
            write_json(args.output / 'push_depth_report.json', report)
    print('REPORT', args.output / 'push_depth_report.json', flush=True)


if __name__ == '__main__':
    main()
