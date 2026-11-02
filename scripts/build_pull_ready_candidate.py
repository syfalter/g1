"""Remove bounded terminal hesitation before pull_grip; keep the gripping pose."""
import copy
import shutil

import numpy as np
from trajectory_msgs.msg import JointTrajectory

from trajectory_only_candidates import (ROOT, BASE, read, load, decode, checked, refresh,
    write_json, OfflineModel, parameters_from_values, time_seconds, context_for, metrics,
    ledger, remaining_budget, KEYS)
from task_composition_preview import at, section, concatenate, stamp
from task_stop_blending import spatial_bound
from task_offline_optimizer import Policy
from task_trajectory import coefficients, peak
from task_trajectory_speed import motion_limits
from task_optimizer_snapshot import save, DIRECTORY

OUTPUT = ROOT / 'data/rail_deployment/pull_ready_shorter5_20260929'


def main():
    model = OfflineModel(ROOT / 'data/trajectory_edit_audit/fast_timing_all/panel_robot.urdf')
    bundle = read(BASE / 'bundle.json')
    prior = read(ROOT / 'data/rail_deployment/pull_shorter_5mm_20260929/report.json')
    saved = load(ROOT / prior['candidate_snapshot'], model)
    deployed = load(ROOT / bundle['payload']['actions']['pull']['source'], model)
    payload = saved['variants']['flow']['payload']
    source = decode(payload['units']['motion_0']['trajectory'])
    old_evidence = deployed['variants']['flow']['payload']['flowEvidence']['motion_0']
    intervals, reserve = ledger(old_evidence)
    parameters = parameters_from_values(bundle['payload']['actions']['pull']['plan']['planning_config']['values'])
    context = context_for(payload, 'motion_0')
    before = metrics(source, model, context)[0]
    duration = time_seconds(source.points[-1])
    last = at(source, duration)
    vmax, amax = motion_limits(source, model, 1., 1., parameters)
    limits = [np.array([d[n] for n in source.joint_names]) for d in
              (vmax, amax, dict.fromkeys(source.joint_names, 5.))]
    best = None
    trials = []
    for lo in (14., 14.5, 15., 15.5, 16., 16.5, 16.7, 17., 17.5, 18., 18.5, 19., 19.5, 20., 20.5, 21.):
        print('TERMINAL BEGIN', lo, flush=True)
        available = remaining_budget(intervals, reserve, Policy().limits('transit'), lo, duration)
        first = at(source, lo)
        reference = section(source, lo, duration)
        for span in (.35, .5, .65, .8, 1., 1.25, 1.5, 2., 2.5, 3.):
            if lo + span >= duration or (best and lo + span >= best[0]):
                continue
            poly = coefficients(first, last, span)
            if any(peak(np.polynomial.polynomial.polyder(poly, order)[:, i]) / span**order > cap * .999
                   for order, caps in enumerate(limits, 1) for i, cap in enumerate(caps)):
                trials.append(dict(begin_s=lo, span_s=span, rejected='motion_limits'))
                continue
            bound = spatial_bound(reference, np.pad(poly, ((0,2),(0,0))), model, 'right')
            if np.any(np.array([bound[k] for k in KEYS]) > available * .98):
                trials.append(dict(begin_s=lo, span_s=span, rejected='remaining_path_budget', bounds=bound))
                continue
            bridge = JointTrajectory(joint_names=list(source.joint_names), points=[
                stamp(copy.deepcopy(first), 0.), stamp(copy.deepcopy(last), span)])
            candidate = concatenate([section(source, 0., lo), bridge])
            peaks = checked(candidate, model, parameters)
            after = metrics(candidate, model, context)[0]
            if (after['stops']['count'] > before['stops']['count'] or
                after['below_5_mm_s'] >= before['below_5_mm_s'] or
                sum(after['joint_travel_rad'].values()) > sum(before['joint_travel_rad'].values()) + .001):
                trials.append(dict(begin_s=lo, span_s=span, rejected='hesitation_or_travel_regression'))
                continue
            evidence = dict(method='terminal_c2_bounded_replacement', begin_s=lo,
                source_end_s=duration, replacement_seconds=span, bounds=bound,
                remaining_before_edit=dict(zip(KEYS, available.tolist())),
                total_original_path_bound=dict(zip(KEYS, (Policy().limits('transit') - available +
                                                          np.array([bound[k] for k in KEYS])).tolist())),
                before=before, selected=after, derivative_validation=peaks, motion_limits_passed=True,
                publishable=False, physical_verified=False, hardware_executable=False,
                final_grasp_pose_unchanged=True, hand_wait_unchanged=True,
                prefix_before_begin_unchanged=True, budget_reference='installed original-path ledger; unused terminal budget')
            best = (lo + span, candidate, evidence)
            trials.append(dict(begin_s=lo, span_s=span, accepted=True, seconds=lo+span, bounds=bound))
    if best is None:
        write_json(OUTPUT / 'rejected.json', dict(trials=trials, before=before))
        raise ValueError('No bounded terminal improvement')
    _, candidate, evidence = best
    edits = [{k:copy.deepcopy(evidence[k]) for k in ('begin_s', 'source_end_s', 'replacement_seconds',
              'bounds', 'remaining_before_edit', 'total_original_path_bound')}]
    upper = evidence['begin_s']
    for _ in range(3):
        current = candidate
        current_metrics = metrics(current, model, context)[0]
        choice = None
        print('EARLIER SLOW WINDOW ENDS', upper, flush=True)
        for lo in np.arange(14., upper - .25, .5):
            lo = float(lo)
            available = remaining_budget(intervals, reserve, Policy().limits('transit'), lo, upper)
            first, last = at(source, lo), at(source, upper)
            reference = section(source, lo, upper)
            for span in (.35, .5, .65, .8, 1., 1.25, 1.5, 2., 2.5, 3.):
                seconds = time_seconds(current.points[-1]) - (upper - lo) + span
                if upper - lo <= span + .05 or (choice and seconds >= choice[0]):
                    continue
                poly = coefficients(first, last, span)
                if any(peak(np.polynomial.polynomial.polyder(poly, order)[:, i]) / span**order > cap * .999
                       for order, caps in enumerate(limits, 1) for i, cap in enumerate(caps)):
                    continue
                bound = spatial_bound(reference, np.pad(poly, ((0,2),(0,0))), model, 'right')
                if np.any(np.array([bound[k] for k in KEYS]) > available * .98):
                    continue
                bridge = JointTrajectory(joint_names=list(source.joint_names), points=[
                    stamp(copy.deepcopy(first), 0.), stamp(copy.deepcopy(last), span)])
                proposal = concatenate([section(source, 0., lo), bridge,
                                       section(current, upper, time_seconds(current.points[-1]))])
                peaks = checked(proposal, model, parameters)
                after = metrics(proposal, model, context)[0]
                if (after['stops']['count'] > current_metrics['stops']['count'] or
                    after['below_5_mm_s'] >= current_metrics['below_5_mm_s'] or
                    sum(after['joint_travel_rad'].values()) > sum(current_metrics['joint_travel_rad'].values()) + .001):
                    continue
                edit = dict(begin_s=lo, source_end_s=upper, replacement_seconds=span, bounds=bound,
                    remaining_before_edit=dict(zip(KEYS, available.tolist())),
                    total_original_path_bound=dict(zip(KEYS, (Policy().limits('transit') - available +
                                                              np.array([bound[k] for k in KEYS])).tolist())))
                choice = (seconds, proposal, after, peaks, edit)
        if choice is None:
            break
        _, candidate, after, peaks, edit = choice
        edits.append(edit)
        upper = edit['begin_s']
        evidence.update(selected=after, derivative_validation=peaks)
    evidence['edits'] = edits
    evidence['begin_s'] = min(edit['begin_s'] for edit in edits)
    evidence['total_original_path_bound'] = {k:max(edit['total_original_path_bound'][k] for edit in edits) for k in KEYS}
    evidence['bounds'] = {k:max(edit['bounds'][k] for edit in edits) for k in KEYS}
    ordered = sorted(edits, key=lambda edit:edit['begin_s'])
    if any(a['source_end_s'] > b['begin_s'] for a, b in zip(ordered, ordered[1:])):
        raise ValueError('Approach error budgets overlap')
    path = refresh(saved, {'motion_0':(candidate, evidence)}, model, OUTPUT / 'individual',
                   'pull-ready-c2-shorter-x5mm-v2')
    updated = load(path, model)
    variants = dict(original=copy.deepcopy(deployed['variants']['flow']),
                    baseline=copy.deepcopy(saved['variants']['flow']),
                    candidate=copy.deepcopy(updated['variants']['flow']))
    xyz = model.batch_transforms(['right_index_tcp'], context, candidate.joint_names,
        np.asarray([candidate.points[-1].positions]))['right_index_tcp'][0,:3,3] * 1000.
    for key, label in (('original','已部署 / 原接近'), ('baseline','少拉出 5 mm / 原接近'),
                       ('candidate','少拉出 5 mm / 接近去迟滞')):
        result = variants[key]
        result['payload']['comparisonLabel'] = label
        result['payload']['optimizer']['variant'] = key
        result['payload']['spatialComparison'] = dict(focus='grip_ready', focus_xyz_mm=xyz.tolist(),
            point_xyz_mm=xyz.tolist(), point_label='抓握前 X', reference_frame='URDF root', physical_verified=False)
    comparison = save(variants, copy.deepcopy(saved['snapshot_controls']), model, directory=OUTPUT / 'comparison')
    load(comparison, model)
    target = DIRECTORY / comparison.name
    shutil.copyfile(comparison, target)
    manifest_path = DIRECTORY / 'experiments.json'
    manifest = read(manifest_path)
    entries = manifest.setdefault('rail:pull_cycle', [])
    entries[:] = [v for v in entries if v['id'] != 'pull-ready-c2-shorter-x5mm-v2']
    entries.append(dict(id='pull-ready-c2-shorter-x5mm-v2', label='拉出接近去迟滞 + 少拉出 5 mm',
                        file=target.name, sha256=read(target)['sha256']))
    write_json(manifest_path, manifest)
    write_json(OUTPUT / 'report.json', dict(source_release_sha256=bundle['sha256'],
        source_snapshot=prior['candidate_snapshot'], candidate_snapshot=str(path.relative_to(ROOT)),
        comparison_snapshot=str(target.relative_to(ROOT)), action='pull', approach=evidence,
        source_pull_shorter_report='data/rail_deployment/pull_shorter_5mm_20260929/report.json',
        full_preview_seconds=time_seconds(updated['variants']['flow']['trajectory'].points[-1]),
        trials=trials, final_grasp_pose_unchanged=True, hand_order_and_waits_unchanged=True,
        physical_verified=False, deployed=False, hardware_commands_sent=0))
    print('SELECTED', edits, 'SECONDS', time_seconds(candidate.points[-1]),
          evidence['total_original_path_bound'], flush=True)


if __name__ == '__main__':
    main()
