"""Upload the reviewed 3 mm offline candidate, never install or activate it."""
import argparse
import hashlib
import json
import shutil
import subprocess
from datetime import datetime, timezone

from trajectory_only_candidates import ROOT, BASE, read, load, decode, checked, OfflineModel, parameters_from_values
from export_trajectory_only_candidates import capture, imported
from bundle import write, digest


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--include-pull-shorter', action='store_true')
    parser.add_argument('--include-pull-ready', action='store_true',
                        help='Use shortened pull plus bounded pre-grasp hesitation removal')
    parser.add_argument('--include-gear-bounded', action='store_true',
                        help='Also stage both bounded gear actions and their validated library')
    args = parser.parse_args()
    if args.include_pull_ready:
        args.include_pull_shorter = True
    stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
    prefix = 'push3_pullshort5_upload_' if args.include_pull_shorter else 'push_depth_3mm_upload_'
    if args.include_pull_ready:
        prefix = 'push3_pullshort5_ready_upload_'
    if args.include_gear_bounded:
        prefix = 'session_bounded_upload_'
    stage = ROOT / 'data/rail_deployment' / (prefix + stamp)
    audit = stage / 'jetson_readonly'
    capture(audit)
    before = read(audit / 'config/selected_bundle.json')
    report = read(ROOT / 'data/rail_deployment/push_depth_exit_v2_20260929/report.json')
    if before != read(BASE / 'bundle.json') or before['sha256'] != report['source_release_sha256']:
        raise ValueError('Online parent changed; candidate must be reviewed again')
    evidence = report['candidates']['3']
    source = ROOT / evidence['snapshot']
    model = OfflineModel(ROOT / 'data/trajectory_edit_audit/fast_timing_all/panel_robot.urdf')
    saved = load(source, model)
    payload = saved['variants']['flow']['payload']
    if payload['trajectoryReview']['revision'] != 'push-depth-exit-v2-3mm':
        raise ValueError('Wrong candidate variant')
    if payload['trajectoryReview']['status'] != 'requires-contact-review':
        raise ValueError('Candidate safety status changed')
    action = before['payload']['actions']['push']
    peaks = checked(decode(payload['units']['motion_0']['trajectory']), model,
                    parameters_from_values(action['plan']['planning_config']['values']))
    output = stage / 'candidate'
    output.mkdir(parents=True, exist_ok=False)
    shutil.copyfile(source, output / source.name)
    write(output / 'evidence.json', dict(source_release_sha256=before['sha256'], candidate=evidence,
        validation_rechecked=peaks, physical_verified=False, hardware_executable=False))
    remote_name = 'push3-pullshort5-' if args.include_pull_shorter else 'push-depth-3mm-'
    if args.include_pull_ready:
        remote_name = 'push3-pullshort5-ready-'
    if args.include_gear_bounded:
        remote_name = 'session-bounded-'
    remote = '/home/unitree/rh56dfx/workspace/execution/trajectory_candidates/' + remote_name + stamp
    manifest = dict(kind='offline-candidate-not-an-installable-release', action='push', amplitude_mm=3,
        snapshot=source.name, snapshot_sha256=read(source)['sha256'], selected_variant='flow',
        trajectory_sha256=digest(payload['units']['motion_0']['trajectory']),
        parent_release_sha256=before['sha256'], uploaded_at_utc=stamp,
        remote_directory=remote, activated=False, physical_verified=False, hardware_executable=False,
        activation_blockers=['Confirm physical rail axis and mechanical clearance',
                             'Confirm safe disabled state and resolve health warnings',
                             'Build and validate a push-only release against fresh online parent'],
        unchanged_online_actions=list(before['payload']['actions']), runtime_files_in_package=0)
    if args.include_pull_shorter:
        pull_report = read(ROOT / 'data/rail_deployment/pull_shorter_5mm_20260929/report.json')
        trim_report = pull_report
        revision = 'pull-shorter-x5mm-v1'
        if args.include_pull_ready:
            pull_report = read(ROOT / 'data/rail_deployment/pull_ready_shorter5_20260929/report.json')
            revision = 'pull-ready-c2-shorter-x5mm-v2'
        if pull_report['source_release_sha256'] != before['sha256']:
            raise ValueError('Pull candidate parent mismatch')
        pull_source = ROOT / pull_report['candidate_snapshot']
        pull = load(pull_source, model)['variants']['flow']
        if pull['payload']['trajectoryReview']['revision'] != revision:
            raise ValueError('Wrong pull candidate')
        params = parameters_from_values(before['payload']['actions']['pull']['plan']['planning_config']['values'])
        checked(pull['trajectory'], model, params)
        shutil.copyfile(pull_source, output / pull_source.name)
        write(output / 'pull_evidence.json', pull_report)
        if args.include_pull_ready:
            write(output / 'pull_trim_evidence.json', trim_report)
        # The old online configuration and both prior snapshots remain distinct
        # files; candidate staging never overwrites an installed release.
        shutil.copytree(audit / 'config', output / 'previous_online/config')
        old_sources = {before['payload']['actions'][name]['source'] for name in ('push', 'pull')}
        old_sources.add(pull_report['source_snapshot'])
        old_sources.add(trim_report['source_snapshot'])
        for relative in old_sources:
            path = ROOT / relative
            load(path, model)
            destination = output / 'previous_snapshots' / path.name
            destination.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(path, destination)
        manifest['selected_candidates'] = dict(
            push=dict(snapshot=source.name, variant='flow', revision='push-depth-exit-v2-3mm'),
            pull=dict(snapshot=pull_source.name, variant='flow', revision=revision))
        manifest['actions'] = ['push', 'pull']
        manifest['previous_versions_preserved'] = True
        manifest['activation_blockers'][-1] = 'Build and validate a push/pull-only release against fresh online parent'
    if args.include_gear_bounded:
        gear_stage = ROOT / 'data/rail_deployment/trajectory_only_20260929'
        gear_report = read(gear_stage / 'gear_roundtrip_lower_upper_report.json')
        if gear_report['source_release_sha256'] != before['sha256']:
            raise ValueError('Gear candidate parent mismatch')
        gear_release = read(gear_stage / 'data_only/config/selected_bundle.json')
        gear_envelope = read(gear_stage / 'data_only/config/gear_protocol.json')
        protocol = imported('staged_audited_gear_protocol', audit / 'deployment/rail_jetson/gear_protocol.py')
        library = protocol.GearLibrary(gear_envelope, gear_release)
        gear_source = ROOT / gear_report['preview']
        gear_saved = load(gear_source, model)['variants']['flow']
        gear_params = parameters_from_values(before['payload']['actions'][protocol.BINDINGS[0]]['plan']['planning_config']['values'])
        for key, tr in library.units.items():
            if tr != gear_saved['payload']['units'][key]['trajectory']:
                raise ValueError('Gear snapshot/library mismatch: ' + key)
            checked(decode(tr), model, gear_params)
        shutil.copyfile(gear_source, output / gear_source.name)
        write(output / 'gear_evidence.json', gear_report)
        write(output / 'gear_protocol_candidate.json', gear_envelope)
        actions = {key:gear_release['payload']['actions'][key] for key in protocol.BINDINGS}
        write(output / 'gear_action_candidates.json', dict(actions=actions, sha256=digest(actions),
            source_release_sha256=gear_release['sha256'], staged_only=True, activated=False))
        manifest.setdefault('selected_candidates', dict(push=dict(snapshot=source.name, variant='flow',
            revision='push-depth-exit-v2-3mm')))['gear'] = dict(snapshot=gear_source.name, variant='flow',
            revision='trajectory-only-c2-ledger-v1', library='gear_protocol_candidate.json',
            actions='gear_action_candidates.json', library_sha256=gear_envelope['sha256'],
            ul_seconds=gear_saved['payload']['units']['ul']['seconds'],
            lu_seconds=gear_saved['payload']['units']['lu']['seconds'], reverse_selected=False)
        manifest['actions'] = list(dict.fromkeys(manifest.get('actions',['push']) + list(protocol.BINDINGS)))
        manifest['activation_blockers'][-1] = 'Merge selected candidates and validate full release/library bindings against fresh online parent'
        previous = output / 'previous_online/config'
        if not previous.exists():
            shutil.copytree(audit / 'config', previous)
        for name in protocol.BINDINGS:
            old_source = ROOT / before['payload']['actions'][name]['source']
            destination = output / 'previous_snapshots' / old_source.name
            destination.parent.mkdir(parents=True,exist_ok=True)
            shutil.copyfile(old_source,destination)
        manifest['previous_versions_preserved'] = True
    manifest['files'] = {str(p.relative_to(output)):hashlib.sha256(p.read_bytes()).hexdigest()
                         for p in output.rglob('*') if p.is_file()}
    write(output / 'manifest.json', manifest)
    target = 'unitree@192.168.123.164'
    ssh = ['ssh', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=8', target]
    subprocess.run(ssh + ['mkdir -p /home/unitree/rh56dfx/workspace/execution/trajectory_candidates && mkdir ' + remote],
                   check=True, timeout=20)
    files = sorted(p for p in output.rglob('*') if p.is_file())
    if any(p.suffix != '.json' for p in files):
        raise ValueError('Candidate upload must contain only JSON data')
    subprocess.run(['scp', '-r', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=8',
                    *map(str, sorted(output.iterdir())), target + ':' + remote + '/'], check=True, timeout=90)
    # Verify the transferred bytes and confirm that selected online data stayed untouched.
    hashes = subprocess.check_output(ssh + ['sha256sum ' + ' '.join(remote + '/' + str(p.relative_to(output)) for p in files)],
                                     timeout=20).decode()
    remote_hashes = {line.split()[1].removeprefix(remote + '/'):line.split()[0] for line in hashes.splitlines()}
    for path in files:
        if remote_hashes[str(path.relative_to(output))] != hashlib.sha256(path.read_bytes()).hexdigest():
            raise ValueError('Transferred file hash mismatch: ' + path.name)
    root = '/home/unitree/rh56dfx/workspace/execution/action_runtime_v3/'
    receipt = read(audit / 'receipt.json')
    for name in ('selected_bundle.json', 'gear_protocol.json', 'dring_protocol.json'):
        raw = subprocess.check_output(ssh + ['cat ' + root + 'config/' + name], timeout=20)
        if hashlib.sha256(raw).hexdigest() != receipt['hashes']['config/' + name]:
            raise ValueError('Online config changed during upload: ' + name)
    health = json.loads(subprocess.check_output(ssh + [
        'curl -fsS --max-time 8 http://127.0.0.1:8795/health'], timeout=20))
    if health['releaseSha256'] != before['sha256']:
        raise ValueError('Loaded online release changed during upload')
    write(stage / 'upload_receipt.json', dict(uploaded=True, activated=False, remote_directory=remote,
        files=remote_hashes, online_configs_unchanged=True, health=health,
        hardware_commands_sent=0, services_restarted=[], uploaded_at=datetime.now(timezone.utc).isoformat()))
    print(json.dumps(dict(uploaded=True, activated=False, remote_directory=remote,
                         receipt=str(stage / 'upload_receipt.json')), indent=2))


if __name__ == '__main__':
    main()
