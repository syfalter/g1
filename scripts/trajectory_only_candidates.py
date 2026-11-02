"""Pinned trajectory-only experiments. No hardware transport or deployment API."""
import argparse
import copy
import json
import sys
from pathlib import Path

import numpy as np
from rosidl_runtime_py.convert import message_to_ordereddict as encode
from task_composition_preview import stamp, materialize
from task_composition_smoothing import evaluate
from task_hand_control import model_joints
from task_offline_optimizer import Policy, decode, validate
from task_optimizer_snapshot import load, save, model_hash
from task_planning_config import parameters_from_values
from task_screen_teach import write_json
from task_stop_blending import blend_internal_stops, stop_audit
from task_trajectory import time_seconds, coefficients, peak
from task_trajectory_speed import motion_limits
from workbench_model import OfflineModel

ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT / 'data/rail_deployment/gear_reverse_latency_20260928'
sys.path.insert(0, str(ROOT / 'deployment/rail_jetson'))
from bundle import digest
KEYS = ('joint_rad', 'tcp_m', 'orientation_rad')


def read(path):
    return json.loads(Path(path).read_text())


def ledger(report):
    """Map certified, disjoint source edits onto the installed trajectory clock."""
    if 'report' in report:
        report = report['report']
    if report.get('polishing', {}).get('accepted') or report.get('fallback') or report.get('accepted') is False:
        raise ValueError('Unsupported prior polishing/fallback: no extra geometry budget granted')
    if 'edits' not in report:
        raise ValueError('Missing original-path error ledger')
    offset = 0.
    intervals = []
    previous = 0.
    for edit in sorted(report['edits'], key=lambda e: e['source_begin_s']):
        lo, hi = edit['source_begin_s'], edit['source_end_s']
        if lo < previous - 1e-8:
            raise ValueError('Overlapping historical geometry edits')
        end = lo + offset + edit['replacement_seconds']
        bound = np.array([edit['bounds'][k] for k in KEYS])
        intervals.append((lo + offset, end, bound))
        offset += edit['replacement_seconds'] - (hi - lo)
        previous = hi
    reserve = np.array([report.get('reference_path_error_reserve', {}).get(k, 0.) for k in KEYS])
    # Includes historical polynomial materialization and timestamp roundoff.
    reserve += np.array([1e-7, 1e-6, 1e-6])
    return intervals, reserve


def remaining_budget(intervals, reserve, limits, lo, hi):
    used = np.max([b for a, z, b in intervals if a < hi and z > lo], axis=0) if any(
        a < hi and z > lo for a, z, _ in intervals) else np.zeros(3)
    return np.maximum(0., limits - used - reserve)


def context_for(payload, unit):
    context = dict(payload['context'])
    posture = 'grip'
    for name in payload.get('sequence', []):
        if name == unit:
            break
        if name in payload.get('handWaits', {}):
            posture = payload['handWaits'][name]['posture']
    context.update(model_joints('right', payload['handPresets'][posture]['feedback_q']))
    return context


def metrics(tr, model, context):
    duration = time_seconds(tr.points[-1])
    times = np.linspace(0., duration, max(32, int(np.ceil(duration / .005)) + 1))
    q = evaluate(tr, times)
    frames = model.batch_transforms(['right_index_tcp'], context, tr.joint_names, q)['right_index_tcp']
    xyz = frames[:, :3, 3]
    delta = np.diff(xyz, axis=0)
    dt = np.diff(times)
    speed = np.linalg.norm(delta, axis=1) / dt
    travel = np.sum(abs(np.diff(q, axis=0)), axis=0)
    # Per-axis excess travel is diagnostic: necessary gross reversals are retained.
    excess = np.sum(abs(delta), axis=0) - abs(xyz[-1] - xyz[0])
    k = int(np.argmax(xyz[:, 0]))
    return dict(seconds=duration, tcp_path_mm=float(np.sum(np.linalg.norm(delta, axis=1)) * 1000),
                below_2_mm_s=float(sum(dt[speed < .002])), below_5_mm_s=float(sum(dt[speed < .005])),
                axis_excess_travel_mm=(excess * 1000).tolist(), peak_x_mm=float(xyz[k, 0] * 1000),
                peak_time_s=float(times[k]), joint_travel_rad=dict(zip(tr.joint_names, travel.tolist())),
                stops=stop_audit(tr), sampling_period_s=float(dt[0])), frames


def continuous_positions(tr, model):
    for a, b in zip(tr.points, tr.points[1:]):
        poly = coefficients(a, b, time_seconds(b) - time_seconds(a))
        for i, name in enumerate(tr.joint_names):
            lo, hi = model.limits[name]
            center = (lo + hi) / 2
            shifted = poly[:, i].copy()
            shifted[0] -= center
            if peak(shifted) > (hi - lo) / 2 + 1e-9:
                raise ValueError('Continuous joint position limit: ' + name)


def checked(tr, model, params):
    continuous_positions(tr, model)
    result = validate(tr, model, params, 5.)
    vmax, amax = motion_limits(tr, model, 1., 1., params)
    result['limits'] = dict(velocity=vmax, acceleration=amax, jerk_policy=5.)
    result['gateway_100hz_step_upper_rad'] = {n: v * .01 for n, v in result['by_joint'][0].items()}
    if max(result['gateway_100hz_step_upper_rad'].values()) > .006:
        raise ValueError('Gateway step bound exceeded')
    return result


def optimize_unit(tr, prior, model, params, context, role, progress, baseline=None):
    before, _ = metrics(tr, model, context)
    reference = tr
    try:
        intervals, reserve = ledger(prior)
    except ValueError:
        if baseline is None:
            raise
        reference = baseline
        intervals, reserve = [], np.array([1e-7, 1e-6, 1e-6])
    limits = Policy().limits(role)
    budget = lambda lo, hi: remaining_budget(intervals, reserve, limits, lo, hi)
    proposal, evidence = blend_internal_stops(reference, model, 'right', params, 5., limits,
        progress=progress, limits_at=budget, compress_idle=True, fit_order=5)
    after, _ = metrics(proposal, model, context)
    # Time savings alone cannot justify more low-speed hesitation or reversals.
    accepted = (after['seconds'] < before['seconds'] - .001 and
                after['below_5_mm_s'] <= before['below_5_mm_s'] + .02 and
                after['stops']['count'] <= before['stops']['count'] and
                sum(after['joint_travel_rad'].values()) <= sum(before['joint_travel_rad'].values()) + .001)
    for edit in evidence['edits']:
        lo, hi = edit['source_begin_s'], edit['source_end_s']
        available = budget(lo, hi)
        new = np.array([edit['bounds'][k] for k in KEYS])
        if np.any(new > available):
            raise ValueError('Cumulative geometry budget exceeded')
        edit['remaining_before_edit'] = dict(zip(KEYS, available.tolist()))
        edit['total_original_path_bound'] = dict(zip(KEYS, (limits - available + new).tolist()))
    result = proposal if accepted else tr
    return result, dict(accepted=accepted, before=before, trial=after,
        selected=after if accepted else before, evidence=evidence,
        derivative_validation=checked(result, model, params),
        reference=('installed trajectory plus non-overlapping historical error ledger' if reference is tr
                   else 'frozen original-path baseline; no previous geometry edits reused'),
        physical_verified=False, collision_checked=False, hardware_executable=False)


def refresh(saved, replacements, model, directory, label):
    from task_composition_panel import preview_samples
    from task_smooth_timing import smoothness
    results = copy.deepcopy(saved['variants'])
    flow = results['flow']
    payload = flow['payload']
    changed = 0
    for unit in payload['units'].values():
        count = stop_audit(decode(unit['trajectory']))['count']
        unit['report']['stopAudit'] = dict(before=count, after=count)
    for name, (tr, report) in replacements.items():
        changed += int(payload['units'][name]['trajectory'] != encode(tr))
        details = copy.deepcopy(report)
        details['stopAudit'] = dict(before=payload['units'][name]['report']['stopAudit']['before'],
                                   after=stop_audit(tr)['count'])
        payload['units'][name] = dict(trajectory=encode(tr), seconds=time_seconds(tr.points[-1]), report=details)
        payload['flowEvidence'][name] = details
    if payload['workflow'] == 'landing_gear':
        from task_composition_preview import concatenate
        for name, order in (('cycle_lower', ('lu', 'ul')), ('cycle_upper', ('ul', 'lu'))):
            tr = concatenate(decode(payload['units'][k]['trajectory']) for k in order)
            audits = [payload['units'][k]['report']['stopAudit'] for k in order]
            payload['units'][name] = dict(trajectory=encode(tr), seconds=time_seconds(tr.points[-1]),
                report=dict(parts=order, stopAudit={key:sum(a[key] for a in audits) for key in ('before','after')}))
    payload['options']['trajectory_only_revision'] = label
    payload['optimizer'].update(changed=changed,total=len(replacements))
    status = ('rejected-motion-limits' if any(r.get('motion_limits_passed') is False for _,r in replacements.values())
              else 'requires-contact-review' if any(r.get('publishable') is False for _,r in replacements.values())
              else 'offline-validated')
    payload['trajectoryReview'] = dict(revision=label,status=status,deployed=False,physical_verified=False)
    tr, events = materialize(payload, flow['sequence'])
    times, tcp, joint = preview_samples(tr, payload, events, model, 'right')
    matrices = [evaluate(tr, times, k) for k in (1, 2, 3)]
    flow.update(trajectory=tr, events=events, times=times, tcp=tcp, joint=joint,
                joint_names=list(tr.joint_names), joint_matrix=matrices[0], acceleration_matrix=matrices[1],
                jerk_matrix=matrices[2], acceleration=np.max(abs(matrices[1]), axis=1),
                jerk=np.max(abs(matrices[2]), axis=1), smoothness=smoothness(tr))
    path = save(results, copy.deepcopy(saved['snapshot_controls']), model, directory=directory)
    load(path, model)
    return path


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--urdf', type=Path, default=ROOT / 'data/trajectory_edit_audit/fast_timing_all/panel_robot.urdf')
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--actions', nargs='+', default=['push', 'pull', 'gear_roundtrip_lower_upper'])
    args = p.parse_args()
    args.output = args.output.resolve()
    model = OfflineModel(args.urdf)
    base = read(BASE / 'bundle.json')
    if digest(base['payload']) != base['sha256']:
        raise ValueError('Invalid release hash')
    args.output.mkdir(parents=True, exist_ok=True)
    for name in args.actions:
        action = base['payload']['actions'][name]
        saved = load(ROOT / action['source'], model)
        payload = saved['variants']['flow']['payload']
        params = parameters_from_values(action['plan']['planning_config']['values'])
        units = ['lu', 'ul'] if name.startswith('gear_') else [k for k in payload['units'] if k.startswith('motion_')]
        replacements = {}
        report = dict(source_release_sha256=base['sha256'], source_snapshot=action['source'],
                      model_sha256=model_hash(model), units={}, hardware_commands_sent=0, deployed=False)
        for unit in units:
            print(name, unit, flush=True)
            tr = decode(payload['units'][unit]['trajectory'])
            prior = payload['flowEvidence'][unit]
            if name.startswith('gear_') and unit == 'ul':
                prior = read(BASE / 'bounded_lowering.json')['evidence']
            context = context_for(payload, unit)
            role = 'transit' if name == 'pull' and unit in ('motion_0', 'motion_2') else 'operation'
            try:
                candidate, evidence = optimize_unit(tr, prior, model, params, context, role,
                    lambda text: print(name, unit, text, flush=True),
                    decode(saved['variants']['baseline']['payload']['units'][unit]['trajectory']))
            except ValueError as exc:
                evidence = dict(accepted=False, reason=str(exc), before=metrics(tr, model, context)[0])
                candidate = tr
            replacements[unit] = candidate, evidence
            report['units'][unit] = evidence
            write_json(args.output / (name + '_report.json'), report)
        path = refresh(saved, replacements, model, args.output / 'previews' / name, 'trajectory-only-c2-ledger-v1')
        report['preview'] = str(path.relative_to(ROOT))
        write_json(args.output / (name + '_report.json'), report)
        print('PREVIEW', path, flush=True)


if __name__ == '__main__':
    main()
