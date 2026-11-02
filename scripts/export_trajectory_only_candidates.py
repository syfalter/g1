"""Build a data-only, unactivated candidate against a read-only Jetson audit."""
import argparse
import copy
import hashlib
import importlib.util
import shutil
import subprocess
from datetime import datetime, timezone
from pathlib import Path
from types import SimpleNamespace

from trajectory_only_candidates import (ROOT, BASE, read, write_json, digest, load, decode,
    checked, materialize, refresh, OfflineModel, parameters_from_values, time_seconds)
from bundle import write
from releases import planned_action, validate as validate_release
from task_offline_optimizer_cases import sequence_for


def capture(directory):
    remote = '/home/unitree/rh56dfx/workspace/execution/action_runtime_v3/'
    names = ['config/selected_bundle.json', 'config/gear_protocol.json', 'config/dring_protocol.json',
             'deployment/rail_jetson/gear_protocol.py', 'deployment/rail_jetson/gear_progress.py',
             'deployment/rail_jetson/executor.py', 'deployment/rail_jetson/dring_protocol.py']
    hashes = {}
    for name in names:
        raw = subprocess.check_output(['ssh', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=5',
                                       'unitree@192.168.123.164', 'cat ' + remote + name], timeout=20)
        path = directory / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(raw)
        hashes[name] = hashlib.sha256(raw).hexdigest()
    health = subprocess.check_output(['ssh', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=5',
        'unitree@192.168.123.164', 'curl -fsS http://127.0.0.1:8795/health'], timeout=20)
    import json
    write(directory / 'receipt.json', dict(captured_at=datetime.now(timezone.utc).isoformat(),
        hashes=hashes, health=json.loads(health), read_only=True, hardware_commands_sent=0))


def imported(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def checkpoints_test(library, progress_type, baseline):
    results = []
    for count in (1, 2, 3, 8, 9, 17):
        for pre in ('NONE', 'EXTEND_DOWNLOCK'):
            for mode in ('FULL_CYCLE', 'FINAL_RETRACT_ONLY'):
                request = dict(count=count, preAction=pre, cycleMode=mode)
                program = list(library.program(request))
                counts = []
                interiors = 0
                for step in program:
                    checkpoints = step.get('gear_progress', [])
                    if checkpoints:
                        seen = []
                        probe = progress_type(checkpoints, seen.append)
                        def feed(ns):
                            stamp = SimpleNamespace(sec=ns//1000000000, nanosec=ns%1000000000)
                            probe.feedback(SimpleNamespace(feedback=SimpleNamespace(
                                desired=SimpleNamespace(time_from_start=stamp))))
                            probe.drain()
                        end = round(time_seconds(decode(step['trajectory']).points[-1])*1e9)
                        previous = -1
                        for cp in checkpoints:
                            if not previous < cp['time_ns'] < end:
                                raise ValueError('Invalid progress clock')
                            feed(cp['time_ns']-1)
                            if cp['progress'] in seen:
                                raise ValueError('Progress reported early')
                            feed(cp['time_ns'])
                            feed(cp['time_ns'])
                            if seen.count(cp['progress']) != 1:
                                raise ValueError('Progress duplicate or missing')
                            previous = cp['time_ns']
                        counts.extend(seen)
                        interiors += len(seen)
                    if 'business_progress' in step:
                        counts.append(step['business_progress'])
                old_counts = []
                for step in baseline.program(request):
                    old_counts.extend(cp['progress'] for cp in step.get('gear_progress', []))
                    if 'business_progress' in step:
                        old_counts.append(step['business_progress'])
                distinct = [v for i,v in enumerate(counts) if i == 0 or v != counts[i-1]]
                if counts != old_counts or distinct != list(range(1, count+1)):
                    raise ValueError('Changed count semantics: ' + str((request, counts)))
                if program[-1].get('gear_phase') != 'exit' or program[-2]['kind'] != 'hand':
                    raise ValueError('Missing release and final exit')
                results.append(dict(request=request, controller_clock_checkpoints=interiors, counts=counts,
                    same_value_block_milestones_preserved=True, original_progress_sequence_unchanged=True))
    return results


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--staging', type=Path, required=True)
    parser.add_argument('--capture-readonly', action='store_true')
    parser.add_argument('--session-trial-fast-reverse', action='store_true')
    args = parser.parse_args()
    stage = args.staging.resolve()
    audit = stage / 'jetson_readonly'
    if args.capture_readonly:
        capture(audit)
    before = read(audit / 'config/selected_bundle.json')
    if before != read(BASE / 'bundle.json'):
        raise ValueError('Live baseline changed; regenerate against the selected release')
    for name in ('gear_protocol.json', 'dring_protocol.json'):
        if read(audit / 'config' / name) != read(BASE / name):
            raise ValueError('Live library changed: ' + name)
    protocol = imported('audited_gear_protocol', audit / 'deployment/rail_jetson/gear_protocol.py')
    progress = imported('audited_gear_progress', audit / 'deployment/rail_jetson/gear_progress.py')
    model = OfflineModel(ROOT / 'data/trajectory_edit_audit/fast_timing_all/panel_robot.urdf')
    actions = copy.deepcopy(before['payload']['actions'])
    snapshots = {}
    if args.session_trial_fast_reverse:
        snapshots['push'] = ROOT / 'data/rail_deployment/push_depth_exit_v2_20260929/individual/3/fe73928e4c18ebc23dbc07d7a858e0aa0979691d4687339eeda0204e061c7794.json'
        snapshots['pull'] = ROOT / 'data/rail_deployment/pull_ready_shorter5_20260929/individual/ae2d838b8a3b866b234d9ea5c9872f571327cbdba454cdbcd60c307d38a3e21f.json'
        source = ROOT / 'data/rail_deployment/trajectory_only_20260929/previews/gear_roundtrip_lower_upper/6fb7f2e53bb5b9c04dc260ac6ef152bf4cf4047f2d8902b0d3abcd32124750df.json'
        saved = load(source, model)
        from automatic_recovery import reversed_path
        reverse = decode(reversed_path(saved['variants']['flow']['payload']['units']['lu']['trajectory']))
        params = parameters_from_values(actions[protocol.BINDINGS[0]]['plan']['planning_config']['values'])
        evidence = dict(derivative_validation=checked(reverse, model, params),
            motion_limits_passed=True, publishable=False, physical_verified=False,
            source_snapshot=read(source)['sha256'], source_unit='lu', method='exact-time-reversal',
            user_authorized_trial=True)
        snapshots[protocol.BINDINGS[0]] = refresh(saved, {'ul': (reverse, evidence)}, model,
            stage / 'previews/gear_fast_reverse', 'session-trial-lu8.963-ul-fast-reverse')
    for action in (() if args.session_trial_fast_reverse else ('push', 'pull', protocol.BINDINGS[0])):
        report_path = stage / (action + '_report.json')
        report = read(report_path)
        if report['source_release_sha256'] != before['sha256']:
            raise ValueError('Wrong candidate parent')
        if 'preview' not in report:
            directory = stage / 'previews' / action
            report['preview'] = str((directory / read(directory / 'latest.json')['file']).relative_to(ROOT))
            write_json(report_path, report)
        snapshots[action] = ROOT / report['preview']
    changes = {}
    for name in ('push', 'pull', *protocol.BINDINGS):
        old = actions[name]
        path = snapshots[protocol.BINDINGS[0] if name in protocol.BINDINGS else name]
        saved = load(path, model)
        payload = saved['variants']['flow']['payload']
        snapshot_sha = read(path)['sha256']
        params = parameters_from_values(old['plan']['planning_config']['values'])
        if name in protocol.BINDINGS:
            lower = name == protocol.BINDINGS[0]
            start, finish = ('LOWER', 'UPPER') if lower else ('UPPER', 'LOWER')
            order = ('entry_lower', 'cycle_lower', 'lu', 'exit_upper') if lower else (
                'entry_upper', 'cycle_upper', 'ul', 'exit_lower')
            trajectories = [payload['units'][order[0]]['trajectory'],
                protocol.concatenate(payload['units'][k]['trajectory'] for k in order[1:3]),
                payload['units'][order[3]]['trajectory']]
            sequence = sequence_for(payload, count=1, start=start, finish=finish, leave=True)
        else:
            trajectories = [payload['units'][k]['trajectory'] for k in payload['sequence'] if k.startswith('motion_')]
            sequence = list(payload['sequence'])
        plan = copy.deepcopy(old['plan'])
        segments = [s for s in plan['segments'] if s['kind'] == 'arm' and s['stage'] != 'prepare']
        if len(segments) != len(trajectories):
            raise ValueError('Action layout changed')
        peaks = []
        for segment, tr in zip(segments, trajectories):
            peaks.append(checked(decode(tr), model, params))
            if segment['trajectory'] != tr:
                segment.update(trajectory=copy.deepcopy(tr), duration=time_seconds(decode(tr).points[-1]),
                    start=dict(zip(tr['joint_names'], tr['points'][0]['positions'])),
                    end=dict(zip(tr['joint_names'], tr['points'][-1]['positions'])),
                    recording_fit=dict(source_snapshot=snapshot_sha[:12]))
        for a, b in zip(old['plan']['segments'], plan['segments']):
            if a['kind'] == 'hand' or a.get('stage') == 'prepare':
                if a != b:
                    raise ValueError('Protected hand/prepare segment changed')
        if args.session_trial_fast_reverse and name == 'pull':
            hand_units = [payload['units'][k]['trajectory'] for k in sequence if k.startswith('hand_')]
            hand_segments = [s for s in plan['segments'] if s['kind'] == 'hand']
            if len(hand_units) != len(hand_segments):
                raise ValueError('Hand layout changed')
            for segment, trajectory in zip(hand_segments, hand_units):
                segment['trajectory'] = copy.deepcopy(trajectory)
        full, events = materialize(payload, sequence)
        plan['export_validation'].update(segments_sha256=digest(plan['segments']), peaks=peaks,
            snapshot_sha256=snapshot_sha, sequence=sequence, preview_seconds=time_seconds(full.points[-1]),
            handEvents=events, physical_verified=False,
            trajectory_only_parent=before['sha256'])
        _, actions[name] = planned_action(plan, dict(identity=old['identity']), str(path.relative_to(ROOT)))
        for a, b in zip(old['program'], actions[name]['program']):
            if a['kind'] != 'recorded' and a != b:
                raise ValueError('Protected runtime step changed')
        changes[name] = dict(before_sha256=digest(old), after_sha256=digest(actions[name]),
            before_recorded_seconds=sum(s.get('duration', 0.) for s in old['program'] if s['kind']=='recorded'),
            after_recorded_seconds=sum(s.get('duration', 0.) for s in actions[name]['program'] if s['kind']=='recorded'))
    result = copy.deepcopy(before)
    prefix = 'session-trial-fast-reverse-' if args.session_trial_fast_reverse else 'trajectory-only-v1-'
    result['payload'].update(actions=actions, version=prefix+digest(actions)[:16], parents=[before['sha256']])
    result['sha256'] = digest(result['payload'])
    validate_release(result)
    if actions['dring_pair'] != before['payload']['actions']['dring_pair']:
        raise ValueError('Unrelated D-ring action changed')
    library = copy.deepcopy(read(audit / 'config/gear_protocol.json')['payload'])
    snapshot = read(snapshots[protocol.BINDINGS[0]])
    units = snapshot['data']['variants']['flow']['payload']['units']
    library.update(snapshot_sha256=snapshot['sha256'], bindings={n:actions[n]['source_plan_sha256'] for n in protocol.BINDINGS})
    library['units'] = {k:dict(trajectory=v['trajectory'], sha256=digest(v['trajectory'])) for k,v in units.items()}
    params = parameters_from_values(actions[protocol.BINDINGS[0]]['plan']['planning_config']['values'])
    library['validation'] = dict(method='trajectory-only-ledger-c2', physical_verified=False,
        peaks={k:checked(decode(v['trajectory']), model, params) for k,v in library['units'].items()})
    envelope = dict(payload=library, sha256=digest(library))
    verified = protocol.GearLibrary(envelope, result)
    tests = checkpoints_test(verified, progress.GearProgress,
        protocol.GearLibrary(read(audit / 'config/gear_protocol.json'), before))
    output = stage / 'data_only'
    if output.exists():
        raise ValueError('Refusing to overwrite an existing export')
    write(output / 'config/selected_bundle.json', result)
    write(output / 'config/gear_protocol.json', envelope)
    shutil.copyfile(audit / 'config/dring_protocol.json', output / 'config/dring_protocol.json')
    for path in set(snapshots.values()):
        destination = output / path.relative_to(ROOT)
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(path, destination)
    rollback = stage / 'rollback_data'
    if rollback.exists():
        for path in (audit/'config').iterdir():
            if (rollback/'config'/path.name).read_bytes()!=path.read_bytes():
                raise ValueError('Existing rollback does not match audited baseline')
    else:
        shutil.copytree(audit / 'config', rollback / 'config', dirs_exist_ok=False)
    manifest = dict(version=result['payload']['version'], release_sha256=result['sha256'],
        gear_library_sha256=envelope['sha256'], dring_library_sha256=read(output/'config/dring_protocol.json')['sha256'],
        parent_release_sha256=before['sha256'], changes=changes, unchanged_actions=['dring_pair'],
        initialization_and_mapping_unchanged=True, hand_prepare_steps_unchanged=True,
        gear_reconstruction_exact=True, progress_tests=tests, files={},
        deployed=False, physical_verified=False, hardware_commands_sent=0,
        requires_user_approval=True, requires_fresh_disabled_state=True,
        excluded_candidates=[] if args.session_trial_fast_reverse else ['reverse lowering', 'all push depth modifications'], runtime_files_in_package=0,
        user_authorized_trial=args.session_trial_fast_reverse,
        user_confirmed_disabled=args.session_trial_fast_reverse)
    for path in output.rglob('*'):
        if path.is_file():
            if path.suffix != '.json':
                raise ValueError('Data-only export contains runtime code')
            manifest['files'][str(path.relative_to(output))] = hashlib.sha256(path.read_bytes()).hexdigest()
    write(stage / 'export_report.json', manifest)
    print(result['payload']['version'], result['sha256'], 'progress cases', len(tests), flush=True)


if __name__ == '__main__':
    main()
