"""Immutable, ROS-free publication of explicitly selected successful recordings."""
import argparse
import copy
from datetime import datetime, timezone
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import re
import tempfile
import sys

from bundle import ROOT, digest, read, seconds, validate_trajectory, write

CONTRACT = 'g1-recorded-actions-v2'
VERSION = re.compile(r'[A-Za-z0-9][A-Za-z0-9_.-]{0,79}')
GEAR = ('above_to_lower', 'lower_to_upper', 'upper_to_lower', 'above_to_upper',
        'lower_to_above', 'upper_to_above')
OPTIMIZED_ACTIONS = ('dring_pair', 'gear_roundtrip_lower_upper', 'gear_roundtrip_upper_lower')
ACTION_NAMES = ('push', 'pull', 'dring_left', 'dring_right', 'dring_left_segment', 'dring_right_segment') + tuple('gear_' + x for x in GEAR) + OPTIMIZED_ACTIONS


def action_name(plan):
    if plan.get('kind') == 'optimized_task':
        name = plan.get('action')
        proof = plan.get('export_validation', {})
        if (name not in OPTIMIZED_ACTIONS + ('push', 'pull') or
                proof.get('method') != 'pinned-v5-polynomial-validation' or
                proof.get('physical_verified') is not False or
                proof.get('segments_sha256') != digest(plan['segments'])):
            raise ValueError('Invalid certified optimizer export')
        return name
    if plan.get('kind') == 'rail':
        rail = plan['rail']
        if rail.get('mode') != 'cycle' or rail.get('path_source') != 'recorded':
            raise ValueError('Only complete recorded rail cycles may be released')
        name = rail['operation']
        if name not in ('push', 'pull'):
            raise ValueError('Unknown rail operation')
        return name
    if plan.get('kind') == 'dring_demo':
        demo = plan['demo']
        if demo.get('workflow', 'dring_demo') == 'landing_gear':
            if demo['operation'] not in GEAR:
                raise ValueError('Select a recorded landing-gear segment')
            return 'gear_' + demo['operation']
        direction = demo['operation'].split('_')[0]
        if direction in ('left', 'right'):
            return 'dring_' + direction + ('' if demo.get('full_cycle') else '_segment')
    raise ValueError('Unsupported recorded action')


def finite(values):
    return all(type(v) in (int, float) and math.isfinite(v) for v in values)


def checked_hand(timeline, duration):
    times, rows = timeline['time_s'], timeline['q']
    if (len(times) < 2 or len(times) != len(rows) or not finite(times)
            or times[0] != 0 or abs(times[-1] - duration) > .15
            or any(b <= a for a, b in zip(times, times[1:]))
            or any(len(q) != 6 or not finite(q) or min(q) < 0 or max(q) > 1 for q in rows)):
        raise ValueError('Invalid recorded hand timeline')


def program(plan):
    """Keep recorded polynomials exactly; describe other arm stages as local joins."""
    result = []
    for segment in plan['segments']:
        trajectory = segment['trajectory']
        validate_trajectory(trajectory)
        side = segment['side']
        if side not in ('left', 'right'):
            raise ValueError('Invalid arm')
        duration = segment['duration']
        if type(duration) not in (float, int) or not math.isfinite(duration) or duration <= 0:
            raise ValueError('Invalid segment duration')
        item = dict(label=segment['label'], side=side, duration=duration)
        if segment.get('kind', 'arm') == 'hand':
            q = segment['target_q']
            if len(q) != 6 or not finite(q) or min(q) < 0 or max(q) > 1 or not 1 <= duration <= 10:
                raise ValueError('Invalid hand target/duration')
            item.update(kind='hand', target_q=q)
            if 'hand_target_required' in segment:
                if type(segment['hand_target_required']) is not bool:
                    raise ValueError('Invalid hand target requirement')
                item['hand_target_required'] = segment['hand_target_required']
            if 'finger_sequence' in segment:
                if (segment['finger_sequence'] not in ('four_then_thumb', 'thumb_then_four') or
                        item.get('hand_target_required') is not False or segment['duration'] != 1.):
                    raise ValueError('Invalid staged hand policy')
                item['finger_sequence'] = segment['finger_sequence']
        else:
            names = trajectory['joint_names']
            expected = {side + '_' + n + '_joint' for n in (
                'shoulder_pitch', 'shoulder_roll', 'shoulder_yaw', 'elbow',
                'wrist_roll', 'wrist_pitch', 'wrist_yaw')} | {'waist_yaw_joint'}
            if set(names) != expected or len(names) != 8:
                raise ValueError('Only single-arm plus waist recorded stages are supported')
            if abs(seconds(trajectory['points'][-1]) - duration) > 1e-6:
                raise ValueError('Segment duration differs from trajectory')
            for boundary, point in [('start', trajectory['points'][0]), ('end', trajectory['points'][-1])]:
                if any(abs(segment[boundary][n] - v) > 1e-7 for n, v in zip(names, point['positions'])):
                    raise ValueError('Segment boundary differs from trajectory')
            recorded = segment.get('trace_stage') is not None or segment.get('recording_fit') is not None
            if recorded:
                item.update(kind='recorded', trajectory=copy.deepcopy(trajectory),
                            trajectory_sha256=digest(trajectory), profile='withdraw')
                if segment.get('recorded_hand') is not None:
                    checked_hand(segment['recorded_hand'], duration)
                    item['recorded_hand'] = copy.deepcopy(segment['recorded_hand'])
                if segment.get('delayed_hand') is not None:
                    spec = segment['delayed_hand']
                    q = spec['target_q']
                    if (not finite([spec['delay_s'], spec['duration']]) or not 0 <= spec['delay_s'] < duration
                            or not 1 <= spec['duration'] <= 10 or len(q) != 6 or not finite(q)
                            or min(q) < 0 or max(q) > 1):
                        raise ValueError('Invalid delayed hand transition')
                    item['delayed_hand'] = copy.deepcopy(spec)
            elif segment.get('stage') == 'prepare':
                item.update(kind='join', target=dict(zip(names, trajectory['points'][-1]['positions'])),
                            reference_trajectory_sha256=digest(trajectory), profile='normal')
            else:
                raise ValueError('Unclassified arm segment: ' + segment['label'])
        result.append(item)
    if not any(s['kind'] == 'recorded' for s in result):
        raise ValueError('No recorded trajectory selected')
    return result


def successful(plan, evidence):
    if not isinstance(evidence, list):
        raise ValueError('Missing execution evidence')
    arms = [s for s in plan['segments'] if s.get('kind', 'arm') == 'arm']
    completed = [s for s in evidence if 'status' in s]
    if (len(arms) != len(completed) or not arms or
            any(e.get('status') != 4 or e.get('error_code') != 0 or e.get('segment') != s['label']
                for s, e in zip(arms, completed))):
        raise ValueError('Not all selected arm stages completed successfully')
    for s in plan['segments']:
        if s.get('kind') == 'hand' and not any(
                e.get('kind') == 'hand' and e.get('segment') == s['label'] and e.get('feedback') for e in evidence):
            raise ValueError('Missing successful hand stage feedback')
        for field, proof in (('recorded_hand', 'recorded_fingers'), ('delayed_hand', 'delayed_hand')):
            if s.get(field) is not None and not any(e.get('segment') == s['label'] and e.get(proof) for e in evidence):
                raise ValueError('Missing synchronized finger execution evidence')


def source_folder(root, source):
    root = Path(root).resolve()
    if not isinstance(source, str) or len(Path(source).parts) != 2 or Path(source).is_absolute():
        raise ValueError('Select a session/action relative identifier')
    folder = (root / source).resolve()
    unresolved = root / source
    if (root not in folder.parents or unresolved.is_symlink() or unresolved.parent.is_symlink()
            or any((folder / p).is_symlink() for p in ('plan.json', 'execution.json', 'config.json'))):
        raise ValueError('Source escapes session root')
    return folder


def load_source(root, source):
    folder = source_folder(root, source)
    plan, evidence, config = (read(folder / name) for name in ('plan.json', 'execution.json', 'config.json'))
    name = action_name(plan)
    if plan.get('hardware_executable') is not True or plan.get('preview_only_timing'):
        raise ValueError('Preview plans cannot become hardware-validated releases')
    if (folder / 'execution_failure.json').exists() or (folder / 'failure.json').exists():
        raise ValueError('This run has a failure record; select an unambiguous successful run')
    successful(plan, evidence)
    steps = program(plan)
    result = read(folder / 'rail_result.json') if name in ('push', 'pull') else None
    if result is not None and result.get('state') != ('inner_released' if name == 'push' else 'outer'):
        raise ValueError('Rail cycle did not reach its final state')
    return name, dict(source=source, plan=plan, execution=evidence, teaching=config,
                      result=result, program=steps, source_plan_sha256=digest(plan),
                      evidence_sha256=digest(evidence), identity=config['identity'],
                      verification='executed')


def planned_action(plan, config, source, evidence=None, result=None):
    """A PLAN_READY snapshot is publishable, but never becomes execution evidence."""
    name = action_name(plan)
    if plan.get('hardware_executable') is not True or plan.get('preview_only_timing'):
        raise ValueError('Select a hardware-timed PLAN_READY recording, not an unrestricted preview')
    status = 'planned'
    if evidence is not None:
        successful(plan, evidence)
        if name in ('push', 'pull') and (not result or result.get('state') !=
                                        ('outer' if name == 'pull' else 'inner_released')):
            raise ValueError('Missing complete rail execution result')
        status = 'executed'
    return name, dict(source=source, plan=copy.deepcopy(plan), teaching=copy.deepcopy(config),
        execution=copy.deepcopy(evidence), result=copy.deepcopy(result), verification=status,
        program=program(plan), source_plan_sha256=digest(plan), evidence_sha256=digest(evidence),
        identity=copy.deepcopy(config['identity']))


def catalog(root):
    entries, errors = [], []
    for execution in sorted(Path(root).glob('*/*/execution.json'), reverse=True):
        folder = execution.parent
        # Other UI workflows have execution.json as well, but are not recordings.
        if not (folder.name.startswith('rail_plan_') or folder.name.startswith('dring_demo_')
                or folder.name.startswith('landing_gear_')):
            continue
        source = str(folder.relative_to(root))
        try:
            name, action = load_source(root, source)
            plan = action['plan']
            slot = plan.get('demo', {}).get('operation', name)
            entries.append(dict(source=source, action=name, slot=slot,
                plan_sha256=action['source_plan_sha256'],
                duration=sum(s['duration'] for s in plan['segments']),
                completed_at=datetime.fromtimestamp(execution.stat().st_mtime, timezone.utc).isoformat(),
                source_status='controller_sequence_completed',
                arm_only=plan.get('demo', plan.get('rail', {})).get('arm_only', False)))
        except (ValueError, KeyError, OSError, TypeError) as error:
            errors.append(dict(source=source, error=str(error)))
    return dict(entries=entries, errors=errors)


def validate(envelope):
    p = envelope['payload']
    if (envelope['sha256'] != digest(p) or p.get('schema') != 2 or
            p.get('contract') != CONTRACT or not VERSION.fullmatch(p.get('version', ''))):
        raise ValueError('Invalid release identity or integrity')
    if p.get('hardware_enabled') is not False or not p['actions']:
        raise ValueError('A data release cannot grant hardware authorization')
    if p.get('mapping') not in ({'AIR': 'push', 'GROUND': 'pull'}, {'AIR': 'pull', 'GROUND': 'push'}):
        raise ValueError('AIR/GROUND must map to the two rail actions')
    identities = set()
    compatibility = p['compatibility']
    for name, a in p['actions'].items():
        if a['plan'].get('hardware_executable') is not True or a['plan'].get('preview_only_timing'):
            raise ValueError('Release includes unrestricted preview timing')
        if name != action_name(a['plan']) or a['program'] != program(a['plan']):
            raise ValueError('Action program differs from selected successful plan')
        if a.get('verification') == 'executed':
            successful(a['plan'], a['execution'])
            if name in ('push', 'pull') and (not a['result'] or a['result'].get('state') !=
                                            ('outer' if name == 'pull' else 'inner_released')):
                raise ValueError('Missing rail completion evidence')
        elif a.get('verification') != 'planned' or a['execution'] is not None or a['result'] is not None:
            raise ValueError('Invalid planning/execution evidence classification')
        if a['source_plan_sha256'] != digest(a['plan']) or a['evidence_sha256'] != digest(a['execution']):
            raise ValueError('Changed plan/evidence')
        if a['identity'] != a['teaching']['identity']:
            raise ValueError('Changed model identity')
        identity = a['identity']
        identities.add(identity.get('urdf_sha256', identity.get('robot_sha256')))
        # Rail archives hash the adapter source; demonstration archives save its
        # numeric mapping. They are different identities, not interchangeable hashes.
        if 'hand_mapping_sha256' in identity:
            if identity['hand_mapping_sha256'] != compatibility['hand_control_sha256']:
                raise ValueError('Rail hand adapter source differs from release runtime')
        elif identity.get('hand_mapping') != compatibility['hand_mapping']:
            raise ValueError('Demonstration hand mapping differs from release runtime')
    if len(identities) != 1 or None in identities:
        raise ValueError('Selected actions use different models/hand mappings')
    if p.get('release_kind') not in ('full', 'patch'):
        raise ValueError('Unknown release scope')
    init = p['initialization']
    expected = {side + '_' + n + '_joint' for side in ('left', 'right') for n in (
        'shoulder_pitch', 'shoulder_roll', 'shoulder_yaw', 'elbow', 'wrist_roll', 'wrist_pitch', 'wrist_yaw')}
    if (set(init['target']) != expected | {'waist_yaw_joint'} or not finite(init['target'].values())
            or init['velocity_scale'] != .7 or init['acceleration_scale'] != .7 or init['mode'] != 'simultaneous'):
        raise ValueError('Invalid simultaneous preparation contract')
    return envelope


def load_release(path):
    return validate(read(path))


def describe(envelope):
    p = envelope['payload']
    return dict(version=p['version'], sha256=envelope['sha256'], created_at=p['created_at'],
        release_kind=p['release_kind'],
        actions={name:dict(source=a['source'], slot=a['plan'].get('demo', {}).get('operation', name),
                          verification=a['verification'],
                          scope='segment' if name.endswith('_segment') or name.startswith('gear_') else 'cycle',
                          trajectory_hashes=[s['trajectory_sha256'] for s in a['program'] if s['kind']=='recorded'])
                 for name, a in p['actions'].items()}, hardware_authorized=False)


class ReleaseStore:
    def __init__(self, root):
        self.root = Path(root).resolve()
        self.root.mkdir(parents=True, exist_ok=True)
        self._summaries = {}

    def path(self, version):
        if not isinstance(version, str) or not VERSION.fullmatch(version):
            raise ValueError('Invalid release version')
        folder = self.root / version
        if folder.is_symlink():
            raise ValueError('Release directory cannot be a symlink')
        return folder / 'bundle.json'

    def load(self, version):
        path = self.path(version)
        if path.is_symlink():
            raise ValueError('Release file cannot be a symlink')
        value = load_release(path)
        if value['payload']['version'] != version:
            raise ValueError('Release directory/manifest mismatch')
        return value

    def install(self, envelope):
        validate(envelope)
        destination = self.path(envelope['payload']['version']).parent
        # Serialize publication and reject overwrites even for concurrent HTTP requests.
        with (self.root / '.publish.lock').open('a') as lock:
            fcntl.flock(lock, fcntl.LOCK_EX)
            if destination.exists():
                existing = self.load(envelope['payload']['version'])
                if existing['sha256'] != envelope['sha256']:
                    raise ValueError('Version already exists with different content')
                return describe(existing)
            with tempfile.TemporaryDirectory(prefix='.incoming-', dir=self.root) as tmp:
                path = Path(tmp) / 'release'
                write(path / 'bundle.json', envelope)
                load_release(path / 'bundle.json')
                with (path / 'bundle.json').open('rb') as handle:
                    os.fsync(handle.fileno())
                path.rename(destination)
                fd = os.open(str(self.root), os.O_DIRECTORY)
                try: os.fsync(fd)
                finally: os.close(fd)
        return describe(envelope)

    def list(self):
        rows, errors = [], []
        for directory in sorted(self.root.iterdir(), reverse=True):
            if directory.name.startswith('.') or not directory.is_dir():
                continue
            try:
                path = self.path(directory.name)
                stat = path.stat()
                signature = (stat.st_ino, stat.st_size, stat.st_mtime_ns, stat.st_ctime_ns)
                cached = self._summaries.get(directory.name)
                if not cached or cached[0] != signature:
                    cached = (signature, describe(self.load(directory.name)))
                    self._summaries[directory.name] = cached
                rows.append(copy.deepcopy(cached[1]))
            except (ValueError, OSError, KeyError, TypeError) as error:
                errors.append(dict(version=directory.name, error=str(error)))
        return dict(releases=rows, errors=errors)

    def compose(self, base, patch):
        validate(base)
        validate(patch)
        if patch['payload']['release_kind'] != 'patch':
            return patch
        if base['payload']['initialization'] != patch['payload']['initialization']:
            raise ValueError('Preparation changed; publish a reviewed full release instead of a module patch')
        if base['payload']['compatibility'] != patch['payload']['compatibility']:
            raise ValueError('Runtime compatibility changed; module patch refused')
        payload = copy.deepcopy(base['payload'])
        payload['actions'].update(copy.deepcopy(patch['payload']['actions']))
        combined = digest(dict(base=base['sha256'], patch=patch['sha256']))
        payload.update(version='set-' + combined[:24], release_kind='full',
                       created_at=patch['payload']['created_at'],
                       parents=[base['sha256'], patch['sha256']])
        envelope = dict(payload=payload, sha256=digest(payload))
        self.install(envelope)
        return envelope


def package(version, actions, initialization, *, kind='full'):
    sys.path.insert(0, str(ROOT / 'scripts'))
    from task_hand_control import ORDER, OPEN, CLOSED
    compatibility = dict(hand_control_sha256=hashlib.sha256((ROOT / 'scripts/task_hand_control.py').read_bytes()).hexdigest(),
                         hand_mapping=dict(order=list(ORDER), open=list(OPEN), closed=list(CLOSED)))
    p = dict(schema=2, contract=CONTRACT, version=version,
             created_at=datetime.now(timezone.utc).isoformat(), hardware_enabled=False,
             release_kind=kind, actions=actions, initialization=copy.deepcopy(initialization),
             compatibility=compatibility,
             mapping={'AIR': 'push', 'GROUND': 'pull'},
             success_semantics='robot_sequence_completed_without_external_signal_confirmation')
    return validate(dict(payload=p, sha256=digest(p)))


def publish(root, store, version, selections, initialization):
    if not VERSION.fullmatch(version) or not selections or set(selections) - set(ACTION_NAMES):
        raise ValueError('Invalid version/action selection')
    actions = {}
    for expected, selection in selections.items():
        # A catalog hash prevents accidentally publishing a slot overwritten after selection.
        name, action = load_source(root, selection['source'])
        if name != expected or action['source_plan_sha256'] != selection['plan_sha256']:
            raise ValueError('Selected source changed or belongs to a different action')
        actions[name] = action
    envelope = package(version, actions, initialization)
    store.install(envelope)
    return envelope


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--store', type=Path, default=ROOT / 'data/action_releases')
    commands = parser.add_subparsers(dest='command', required=True)
    cat = commands.add_parser('catalog')
    cat.add_argument('--sessions', type=Path, default=ROOT / 'data/task_sessions')
    commands.add_parser('list')
    imp = commands.add_parser('import')
    imp.add_argument('bundle', type=Path)
    check = commands.add_parser('verify')
    check.add_argument('version')
    pub = commands.add_parser('publish')
    pub.add_argument('--sessions', type=Path, default=ROOT / 'data/task_sessions')
    pub.add_argument('--selection', type=Path, required=True)
    args = parser.parse_args()
    store = ReleaseStore(args.store)
    if args.command == 'catalog': result = catalog(args.sessions)
    elif args.command == 'list': result = store.list()
    elif args.command == 'verify': result = describe(store.load(args.version))
    elif args.command == 'import': result = store.install(load_release(args.bundle))
    else:
        selection = read(args.selection)
        result = describe(publish(args.sessions, store, selection['version'], selection['actions'], selection['initialization']))
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
