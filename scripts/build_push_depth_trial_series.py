"""Prepare incremental depth trials without changing the running robot."""
import argparse
import copy
import hashlib
import shutil

from trajectory_only_candidates import (ROOT, BASE, read, load, decode, checked, context_for,
    metrics, refresh, write_json, digest, OfflineModel, parameters_from_values, time_seconds,
    materialize)
from trajectory_push_depth import compensation
from export_trajectory_only_candidates import capture
from releases import planned_action, validate
from gear_protocol import GearLibrary
from dring_protocol import DringLibrary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    output = ROOT / args.output
    audit = output / 'jetson_readonly'
    capture(audit)
    parent = read(audit / 'config/selected_bundle.json')
    expected = read(ROOT / 'data/rail_deployment/session_trial_activation_20260929/data_only/config/selected_bundle.json')
    if parent != expected:
        raise ValueError('Selected parent differs from verified 3mm release')
    model = OfflineModel(ROOT / 'data/trajectory_edit_audit/fast_timing_all/panel_robot.urdf')
    baseline = read(BASE / 'bundle.json')['payload']['actions']['push']
    saved = load(ROOT / baseline['source'], model)
    payload = saved['variants']['flow']['payload']
    source = decode(payload['units']['motion_0']['trajectory'])
    context = context_for(payload, 'motion_0')
    params = parameters_from_values(baseline['plan']['planning_config']['values'])
    prior = read(ROOT / 'data/rail_deployment/trajectory_only_20260929/push_depth_report.json')
    peak = prior['before']['peak_time_s']
    report = dict(parent_release_sha256=parent['sha256'], amplitudes_relative_to_original=True,
                  physical_verified=False, deployed=False, candidates={})
    for mm in range(4, 10):
        trials = []
        if mm == 4:
            evidence = read(ROOT / 'data/rail_deployment/push_depth_exit_v2_20260929/report.json')['candidates']['4']
            snapshot = ROOT / evidence['snapshot']
        else:
            snapshot = None
            for rise, tail in ((2.5, 2.), (2.5, 2.5), (3., 3.), (3.5, 3.5), (4., 4.)):
                support = tuple(round(t, 9) for t in (peak-rise, peak-.2, peak, peak+tail))
                if support[0] <= 0 or support[-1] >= time_seconds(source.points[-1]):
                    continue
                print('DEPTH', mm, 'RISE', rise, 'EXIT', tail, flush=True)
                try:
                    trajectory, evidence = compensation(source, model, context, prior['axis_root'], mm, support)
                    validation = checked(trajectory, model, params)
                except ValueError as error:
                    trials.append(dict(rise_s=rise, exit_s=tail, rejected=str(error)))
                    continue
                evidence.update(derivative_validation=validation, metrics=metrics(trajectory, model, context)[0],
                    trials=trials, motion_limits_passed=True, publishable=False, physical_verified=False,
                    physical_axis_confirmed=False, collision_checked=False, hardware_executable=False,
                    user_requested_trial=True, pending_checks=['physical axis', 'end-stop clearance', 'contact safety'])
                snapshot = refresh(saved, {'motion_0': (trajectory, evidence)}, model,
                    output / 'snapshots' / str(mm), 'push-depth-trial-' + str(mm) + 'mm')
                break
            if snapshot is None:
                report['candidates'][str(mm)] = dict(rejected=True, trials=trials)
                write_json(output / 'report.json', report)
                continue
        flow = load(snapshot, model)['variants']['flow']
        candidate = flow['payload']
        old = parent['payload']['actions']['push']
        plan = copy.deepcopy(old['plan'])
        arms = [s for s in plan['segments'] if s['kind'] == 'arm' and s['stage'] != 'prepare']
        trajectories = [candidate['units'][k]['trajectory'] for k in candidate['sequence'] if k.startswith('motion_')]
        if len(arms) != len(trajectories):
            raise ValueError('Changed push layout')
        peaks = []
        for segment, trajectory in zip(arms, trajectories):
            peaks.append(checked(decode(trajectory), model, params))
            segment.update(trajectory=trajectory, duration=time_seconds(decode(trajectory).points[-1]),
                start=dict(zip(trajectory['joint_names'], trajectory['points'][0]['positions'])),
                end=dict(zip(trajectory['joint_names'], trajectory['points'][-1]['positions'])),
                recording_fit=dict(source_snapshot=read(snapshot)['sha256'][:12]))
        full, events = materialize(candidate, candidate['sequence'])
        plan['export_validation'].update(segments_sha256=digest(plan['segments']), peaks=peaks,
            snapshot_sha256=read(snapshot)['sha256'], sequence=candidate['sequence'],
            preview_seconds=time_seconds(full.points[-1]), handEvents=events, physical_verified=False,
            trajectory_only_parent=parent['sha256'])
        _, action = planned_action(plan, dict(identity=old['identity']), str(snapshot.relative_to(ROOT)))
        if len(old['program']) != len(action['program']):
            raise ValueError('Program layout changed')
        for previous, new in zip(old['program'], action['program']):
            if previous['kind'] != 'recorded' and previous != new:
                raise ValueError('Protected runtime step changed')
        release = copy.deepcopy(parent)
        release['payload']['actions']['push'] = action
        release['payload'].update(version='session-push-'+str(mm)+'mm-'+digest(action)[:12], parents=[parent['sha256']])
        release['sha256'] = digest(release['payload'])
        validate(release)
        gear = read(audit / 'config/gear_protocol.json')
        dring = read(audit / 'config/dring_protocol.json')
        GearLibrary(gear, release)
        DringLibrary(dring, release)
        stage = output / ('trial_' + str(mm) + 'mm')
        data = stage / 'data_only'
        write_json(data / 'config/selected_bundle.json', release)
        for name in ('gear_protocol.json', 'dring_protocol.json'):
            shutil.copyfile(audit / 'config' / name, data / 'config' / name)
        target = data / snapshot.relative_to(ROOT)
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(snapshot, target)
        shutil.copytree(audit, stage / 'jetson_readonly', dirs_exist_ok=True)
        manifest = dict(version=release['payload']['version'], release_sha256=release['sha256'],
            parent_release_sha256=parent['sha256'], gear_library_sha256=gear['sha256'],
            dring_library_sha256=dring['sha256'], user_authorized_trial=True, user_confirmed_disabled=False,
            physical_verified=False, deployed=False, hardware_commands_sent=0,
            requires_fresh_disabled_state=True, amplitude_mm=mm, unchanged_actions=[n for n in parent['payload']['actions'] if n != 'push'],
            files={str(p.relative_to(data)):hashlib.sha256(p.read_bytes()).hexdigest() for p in data.rglob('*.json')})
        write_json(stage / 'export_report.json', manifest)
        report['candidates'][str(mm)] = dict(snapshot=str(snapshot.relative_to(ROOT)), evidence=evidence,
            release_sha256=release['sha256'], stage=str(stage.relative_to(ROOT)))
        write_json(output / 'report.json', report)
        print('READY', mm, release['sha256'], flush=True)


if __name__ == '__main__':
    main()
