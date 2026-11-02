"""Pinned left/right loops: one entry, intermediate trimmed cycles, one exit."""
import copy
from collections import OrderedDict
from pathlib import Path

from automatic_recovery import endpoint, near
from bundle import digest, read, seconds, validate_trajectory
from gear_protocol import concatenate

MAX_BLOCK_CYCLES = 8


def count_of(parameters):
    if set(parameters) != {'count'} or type(parameters['count']) is not int:
        raise ValueError('NWS_CYCLE requires an integer count')
    if not 1 <= parameters['count'] <= 2147483647:
        raise ValueError('NWS count must be in 1..2147483647')
    return parameters['count']


class DringLibrary:
    def __init__(self, envelope, release):
        p = envelope['payload']
        action = release['payload']['actions']['dring_pair']
        proof = action['plan']['export_validation']
        if (digest(p) != envelope['sha256'] or p.get('schema') != 1 or
                p.get('contract') != 'eln-dring-pinned-units-v1' or
                p.get('hardware_enabled') is not False):
            raise ValueError('Invalid D-ring library identity')
        if (p['binding'] != action['source_plan_sha256'] or
                p['snapshot_sha256'] != proof['snapshot_sha256'] or
                p['model_sha256'] != proof['model_sha256']):
            raise ValueError('D-ring library differs from selected recording')
        if set(p['units']) != {'entry', 'cycle', 'final'}:
            raise ValueError('Incomplete D-ring loop units')
        self.units = {}
        for name, item in p['units'].items():
            tr = item['trajectory']
            validate_trajectory(tr)
            if item['sha256'] != digest(tr) or seconds(tr['points'][0]) != 0.:
                raise ValueError('Invalid D-ring unit: ' + name)
            if any(abs(v) > 1e-7 for pt in (tr['points'][0], tr['points'][-1])
                   for field in ('velocities', 'accelerations') for v in pt[field]):
                raise ValueError('D-ring block boundaries must be stationary')
            self.units[name] = tr
        if [s['kind'] for s in action['program']] != ['join', 'recorded']:
            raise ValueError('D-ring action layout changed')
        motion = action['program'][1]
        if motion.get('recorded_hand') or motion.get('delayed_hand'):
            raise ValueError('D-ring finger timeline needs a separately reviewed loop export')
        if concatenate([self.units['entry'], self.units['final']]) != motion['trajectory']:
            raise ValueError('D-ring single-cycle reconstruction differs from selected action')
        concatenate(self.units[k] for k in ('entry', 'cycle', 'cycle', 'final'))
        self.payload, self.sha256, self.release = p, envelope['sha256'], release
        self._blocks = OrderedDict()
        # Common demonstration counts are ready before the API accepts requests.
        for count in (1, 2, 3):
            self.motion(['entry'] + ['cycle'] * (count - 1) + ['final'])

    def describe(self, parameters):
        count = count_of(parameters)
        times = {k: seconds(t['points'][-1]) for k, t in self.units.items()}
        return dict(librarySha256=self.sha256, count=count,
                    nominalSeconds=times['entry'] + (count - 1) * times['cycle'] + times['final'],
                    maxCyclesPerBlock=MAX_BLOCK_CYCLES, entryCount=1, exitCount=1,
                    completionSemantics='motion-sequence-only', physicalVerified=False)

    def motion(self, keys):
        key = tuple(keys)
        if key not in self._blocks:
            tr = concatenate(self.units[k] for k in keys)
            self._blocks[key] = dict(trajectory=tr, trajectory_sha256=digest(tr))
            if len(self._blocks) > 12:
                self._blocks.popitem(last=False)
        self._blocks.move_to_end(key)
        return dict(kind='recorded', side='left', profile='withdraw',
                    label='D-ring continuous: ' + ','.join(keys),
                    duration=seconds(self._blocks[key]['trajectory']['points'][-1]),
                    **self._blocks[key])

    def program(self, parameters):
        count = count_of(parameters)
        yield copy.deepcopy(self.release['payload']['actions']['dring_pair']['program'][0])
        remaining, completed, first = count, 0, True
        while remaining:
            size = min(remaining, MAX_BLOCK_CYCLES)
            remaining -= size
            keys = (['entry'] if first else []) + ['cycle'] * (size - (not remaining))
            if not remaining:
                keys.append('final')
            step = self.motion(keys)
            elapsed_ns, value, checkpoints = 0, completed, []
            end_ns = round(seconds(step['trajectory']['points'][-1]) * 1e9)
            for key in keys:
                elapsed_ns += round(seconds(self.units[key]['points'][-1]) * 1e9)
                if key in ('cycle', 'final'):
                    value += 1
                    if elapsed_ns < end_ns:
                        checkpoints.append(dict(time_ns=elapsed_ns, progress=value))
            # Reuse the deployed controller-clock bridge; do not split arm goals.
            step['gear_progress'] = checkpoints
            step['progress_label'] = 'D-ring left/right pair completed'
            yield step
            first = False
            completed += size
            yield dict(kind='milestone', label='D-ring left/right pairs completed',
                       business_progress=completed)

    def action(self, parameters):
        count_of(parameters)
        return dict(self.release['payload']['actions']['dring_pair'], program=self.program(parameters))

    def recovery_route(self, parameters, failure):
        if failure.get('kind') not in ('rejected', 'endpoint'):
            raise ValueError('No confirmed D-ring checkpoint')
        index = failure.get('index')
        count = count_of(parameters)
        last_motion_index = 1 + 2 * ((count - 1) // MAX_BLOCK_CYCLES)
        if index in (0, 1) and failure['kind'] == 'rejected':
            expected = endpoint(self.units['entry'], 0)
        elif index == 0 and failure['kind'] == 'endpoint':
            expected = endpoint(self.units['entry'], 0)
        elif index == last_motion_index and failure['kind'] == 'endpoint':
            expected = endpoint(self.units['final'], -1)
        else:
            raise ValueError('D-ring intermediate contact checkpoint has no reviewed escape')
        if not near(expected, failure.get('checkpoint', {}), 1e-7):
            raise ValueError('D-ring recovery checkpoint differs from pinned path')
        return dict(action='dring_pair', checkpoint=failure['checkpoint'], steps=[])


def load_optional(release_path, release):
    path = Path(release_path).parent / 'dring_protocol.json'
    return DringLibrary(read(path), release) if path.exists() else None
