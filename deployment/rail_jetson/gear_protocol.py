"""ELN gear cycles over pinned motion units; no planning, ROS or hardware I/O."""
import copy
from pathlib import Path

from automatic_recovery import endpoint, near
from bundle import digest, read, seconds, validate_trajectory

BINDINGS = ('gear_roundtrip_lower_upper', 'gear_roundtrip_upper_lower')
UNITS = ('entry_lower', 'entry_upper', 'lu', 'ul', 'cycle_lower', 'cycle_upper', 'exit_lower', 'exit_upper')
MAX_BLOCK_CYCLES = 8


def recipe(parameters):
    if set(parameters) != {'count', 'preAction', 'cycleMode'}:
        raise ValueError('GEAR_OPERATION requires count, preAction and cycleMode')
    count = parameters['count']
    if type(count) is not int or not 1 <= count <= 2147483647:
        raise ValueError('Gear count must be an integer in 1..2147483647')
    pre, mode = parameters['preAction'], parameters['cycleMode']
    if pre not in ('NONE', 'EXTEND_DOWNLOCK') or mode not in ('FULL_CYCLE', 'FINAL_RETRACT_ONLY'):
        raise ValueError('Unsupported gear preAction/cycleMode')
    upper = pre == 'EXTEND_DOWNLOCK'
    final = mode == 'FINAL_RETRACT_ONLY'
    return dict(parameters=dict(parameters), start='UPPER' if upper else 'LOWER',
                finish='UPPER' if final else 'LOWER',
                repeat='cycle_upper' if upper else 'cycle_lower',
                repetitions=count if upper or not final else count - 1,
                tail='ul' if upper and not final else 'lu' if not upper and final else None,
                total=count, preActionCounted=False, completionSemantics='motion-sequence-only')


def concatenate(parts):
    """Same nanosecond rounding and C2 rule as the pinned preview concatenator."""
    result, offset = None, 0.
    for trajectory in parts:
        if result is None:
            result = dict(header=copy.deepcopy(trajectory.get('header', {})),
                          joint_names=list(trajectory['joint_names']), points=[])
        if trajectory['joint_names'] != result['joint_names']:
            raise ValueError('Gear units have different joint ordering')
        if result['points']:
            a, b = result['points'][-1], trajectory['points'][0]
            for field in ('positions', 'velocities', 'accelerations'):
                if any(abs(x-y) > 1e-7 for x, y in zip(a[field], b[field])):
                    raise ValueError('Gear unit boundary is not C2: ' + field)
        for point in trajectory['points'][1:] if result['points'] else trajectory['points']:
            point = copy.deepcopy(point)
            ns = round((offset + seconds(point)) * 1e9)
            point['time_from_start'] = dict(sec=ns // 1000000000, nanosec=ns % 1000000000)
            result['points'].append(point)
        offset += seconds(trajectory['points'][-1])
    if result is None:
        raise ValueError('Empty gear contact block')
    return result


class GearLibrary:
    def __init__(self, envelope, release):
        p = envelope['payload']
        if envelope['sha256'] != digest(p) or p.get('schema') != 1:
            raise ValueError('Gear library checksum/schema mismatch')
        if p.get('contract') != 'eln-gear-pinned-units-v1' or p.get('hardware_enabled') is not False:
            raise ValueError('Gear library cannot grant hardware authorization')
        actions = release['payload']['actions']
        if p['bindings'] != {n: actions[n]['source_plan_sha256'] for n in BINDINGS}:
            raise ValueError('Gear library differs from selected gear recordings; export it again')
        if set(p['units']) != set(UNITS):
            raise ValueError('Incomplete gear motion units')
        for name in BINDINGS:
            proof = actions[name]['plan']['export_validation']
            if proof['snapshot_sha256'] != p['snapshot_sha256'] or proof['model_sha256'] != p['model_sha256']:
                raise ValueError('Gear snapshot/model differs from selected recording')
        self.release, self.payload, self.sha256 = release, p, envelope['sha256']
        self.units = {k: v['trajectory'] for k, v in p['units'].items()}
        names = actions[BINDINGS[0]]['program'][2]['trajectory']['joint_names']
        for name, record in p['units'].items():
            trajectory = record['trajectory']
            validate_trajectory(trajectory)
            if (record['sha256'] != digest(trajectory) or trajectory['joint_names'] != names or
                    seconds(trajectory['points'][0]) != 0.):
                raise ValueError('Invalid gear unit identity: ' + name)
            for point in (trajectory['points'][0], trajectory['points'][-1]):
                if any(abs(v) > 1e-7 for f in ('velocities', 'accelerations') for v in point[f]):
                    raise ValueError('Gear unit boundary is not stationary: ' + name)
        # Publicly selectable originals must reproduce exactly from these units.
        for name, start, repeat, tail, finish in (
            (BINDINGS[0], 'lower', 'cycle_lower', 'lu', 'upper'),
            (BINDINGS[1], 'upper', 'cycle_upper', 'ul', 'lower')):
            original = actions[name]['program']
            expected = [self.units['entry_' + start], concatenate([self.units[repeat], self.units[tail]]),
                        self.units['exit_' + finish]]
            if [s['trajectory'] for s in original if s['kind'] == 'recorded'] != expected:
                raise ValueError('Gear unit reconstruction differs from pinned full action: ' + name)
        for start in ('lower', 'upper'):
            point = endpoint(self.units['entry_' + start], -1)
            if not near(point, endpoint(self.units['exit_' + start], 0), 1e-7):
                raise ValueError('Gear entry/exit endpoints disagree')
        for keys in (('lu', 'ul'), ('ul', 'lu'), ('cycle_lower', 'cycle_lower'),
                     ('cycle_upper', 'cycle_upper'), ('cycle_lower', 'lu'), ('cycle_upper', 'ul')):
            concatenate(self.units[k] for k in keys)
        self._blocks = {}

    def describe(self, parameters):
        r = recipe(parameters)
        durations = {k: seconds(t['points'][-1]) for k, t in self.units.items()}
        contact = durations[r['repeat']] * r['repetitions'] + (durations[r['tail']] if r['tail'] else 0.)
        total = durations['entry_' + r['start'].lower()] + contact + durations['exit_' + r['finish'].lower()] + 3.
        return dict(recipe=r, librarySha256=self.sha256, nominalSeconds=total,
                    durationExcludes='live start alignment and controller settling', physicalVerified=False)

    def hand(self, posture):
        index = 1 if posture == 'open' else 3
        step = copy.deepcopy(self.release['payload']['actions'][BINDINGS[0]]['program'][index])
        step['label'] = 'gear protocol hand ' + posture
        # Same policy as LandingGearControl; ACK + healthy fresh feedback, not grasp verification.
        step['hand_target_required'] = False
        step['hand_completion_policy'] = 'landing-gear-command-and-fresh-feedback-v1'
        step.update(finger_sequence='four_then_thumb' if posture == 'grip' else 'thumb_then_four', duration=1.)
        return step

    def motion(self, keys, start, finish, phase):
        key = tuple(keys)
        if key not in self._blocks:
            tr = self.units[key[0]] if len(key) == 1 else concatenate(self.units[k] for k in key)
            self._blocks[key] = dict(trajectory=tr, trajectory_sha256=digest(tr))
        return dict(kind='recorded', side='right', profile='withdraw',
                    label='gear protocol ' + phase + ': ' + ','.join(key),
                    gear_start=start, gear_end=finish, gear_phase=phase,
                    **self._blocks[key])

    def program(self, parameters):
        r = recipe(parameters)
        base = BINDINGS[0] if r['start'] == 'LOWER' else BINDINGS[1]
        join = copy.deepcopy(self.release['payload']['actions'][base]['program'][0])
        join.update(gear_start='ABOVE', gear_end='ABOVE', gear_phase='entry')
        yield join
        yield self.hand('open')
        yield self.motion(['entry_' + r['start'].lower()], 'ABOVE', r['start'], 'entry')
        yield self.hand('grip')
        remaining, completed = r['repetitions'], 0
        while remaining:
            size = min(remaining, MAX_BLOCK_CYCLES)
            remaining -= size
            completed += size
            keys, end = [r['repeat']] * size, r['start']
            if not remaining and r['tail']:
                keys.append(r['tail'])
                end = r['finish']
            yield self.motion(keys, r['start'], end, 'contact')
            progress = (r['total'] if not remaining else
                        max(0, completed - (1 if r['start'] == 'UPPER' else 0)))
            yield dict(kind='milestone', label='Gear motion cycles completed', business_progress=progress)
        if r['repetitions'] == 0:
            yield self.motion([r['tail']], r['start'], r['finish'], 'contact')
            yield dict(kind='milestone', label='Final retraction completed', business_progress=r['total'])
        yield self.hand('open')
        yield self.motion(['exit_' + r['finish'].lower()], r['finish'], 'ABOVE', 'exit')

    def action(self, parameters):
        recipe(parameters)
        base = self.release['payload']['actions'][BINDINGS[0]]
        return dict(base, program=self.program(parameters))

    def recovery_route(self, failure):
        if failure.get('kind') not in ('rejected', 'endpoint'):
            raise ValueError('Gear failure has no confirmed checkpoint')
        position = failure.get('gear_position')
        if position == 'ABOVE':
            expected = endpoint(self.units['entry_lower'], 0)
            steps = [self.hand('open')]
        elif position in ('LOWER', 'UPPER'):
            expected = endpoint(self.units['exit_' + position.lower()], 0)
            steps = [self.hand('open'), self.motion(['exit_' + position.lower()], position, 'ABOVE', 'exit')]
        else:
            raise ValueError('Gear checkpoint position is unknown')
        if not near(expected, failure.get('checkpoint', {}), 1e-7):
            raise ValueError('Gear recovery checkpoint does not match a recorded exit')
        return dict(action=BINDINGS[0], checkpoint=failure['checkpoint'], steps=steps)


def load_optional(release_path, release):
    path = Path(release_path).parent / 'gear_protocol.json'
    return GearLibrary(read(path), release) if path.exists() else None
