"""One bounded recovery attempt at a proven recorded checkpoint, never task replay."""
import copy
import math
import threading
import time

from bundle import digest, write


class TrajectoryRejected(RuntimeError):
    pass


def endpoint(trajectory, index):
    return dict(zip(trajectory['joint_names'], trajectory['points'][index]['positions']))


def near(actual, target, tolerance=.05):
    return bool(target) and all(n in actual and math.isfinite(actual[n]) and
                               abs(actual[n] - q) <= tolerance for n, q in target.items())


def reversed_path(trajectory):
    result = copy.deepcopy(trajectory)
    def ns(p):
        t = p['time_from_start']
        return t['sec'] * 1000000000 + t['nanosec']
    end = ns(trajectory['points'][-1])
    result['points'].reverse()
    for point in result['points']:
        t = end - ns(point)
        point['time_from_start'] = dict(sec=t // 1000000000, nanosec=t % 1000000000)
        point['velocities'] = [-v for v in point['velocities']]
    return result


def recovery_route(release, name, failure):
    """Only the reviewed five complete actions have known escape paths.

    Partial/uncertain contact motion is deliberately excluded. A rejected goal
    is at its start; a completed arm goal with a settling failure is at its end.
    """
    if failure.get('kind') not in ('rejected', 'endpoint'):
        raise ValueError('No confirmed trajectory checkpoint')
    action = release['payload']['actions'][name]
    if action['plan'].get('kind') != 'optimized_task':
        raise ValueError('No approved recovery route for this action format')
    program = action['program']
    index = failure['index']
    step = program[index]
    ready = release['payload']['initialization']['target']
    checkpoint = failure['checkpoint']
    result = []
    if near(ready, checkpoint, .04):
        # Includes rejection before a combined operation, or completion at ABOVE.
        if program[-1]['kind'] == 'hand' and program[-1]['label'].endswith(' hand open'):
            result = [copy.deepcopy(program[-1])]
    elif name in ('pull', 'gear_roundtrip_lower_upper', 'gear_roundtrip_upper_lower'):
        if ([s['kind'] for s in program] !=
                ['join', 'hand', 'recorded', 'hand', 'recorded', 'hand', 'recorded']):
            raise ValueError('Recovery layout differs from reviewed recording')
        if not all(program[i]['label'].endswith(' hand open') for i in (1, 5)):
            raise ValueError('Recorded release-hand checkpoints missing')
        at_entry = ((index == 2 and failure['kind'] == 'endpoint') or
                    (index == 4 and failure['kind'] == 'rejected'))
        at_exit = ((index == 4 and failure['kind'] == 'endpoint') or
                   (index == 6 and failure['kind'] == 'rejected'))
        if at_entry:
            motion = copy.deepcopy(program[2])
            motion['trajectory'] = reversed_path(motion['trajectory'])
            result = [copy.deepcopy(program[1]), motion]
        elif at_exit:
            result = [copy.deepcopy(program[5]), copy.deepcopy(program[6])]
        else:
            raise ValueError('No recorded contact-clear escape from this checkpoint')
    else:
        raise ValueError('Interrupted combined contact motion cannot be replayed as recovery')
    if result and result[-1]['kind'] == 'recorded':
        motion = result[-1]
        trajectory = motion['trajectory']
        if motion.get('recorded_hand') or motion.get('delayed_hand'):
            raise ValueError('Recovery may not replay a contact finger timeline')
        if not near(endpoint(trajectory, 0), checkpoint, 1e-6):
            raise ValueError('Escape path does not start at the confirmed checkpoint')
        if not near(ready, endpoint(trajectory, -1), .04):
            raise ValueError('Escape path does not finish at preparation')
        motion['trajectory_sha256'] = digest(trajectory)
    # Never infer a checkpoint from a message alone.
    expected = (step.get('target') if step['kind'] == 'join' else
                endpoint(step['trajectory'], 0 if failure['kind'] == 'rejected' else -1))
    if not near(expected, checkpoint, 1e-6):
        raise ValueError('Failure checkpoint differs from the pinned trajectory')
    return dict(action=name, checkpoint=checkpoint, steps=result)


class AutomaticRecovery:
    def __init__(self, worker):
        self.worker = worker
        self.status = 'idle'
        self.message = ''
        self.plan = None
        self.revision = None
        self.generation = None
        self.thread = None

    def invalidate(self, reason):
        if self.status in ('pending', 'running', 'ready'):
            self.status, self.message = 'blocked', reason

    def snapshot(self):
        return dict(status=self.status, message=self.message, generation=self.generation)

    def schedule(self, job, outcome):
        w = self.worker
        if (job['kind'] not in ('action', 'gear') or outcome.get('status') != 'FAILED' or
                outcome.get('stop_confirmed') is not True or not w.heartbeat_allowed() or
                not hasattr(w.transport, 'check_automatic_recovery')):
            return False
        try:
            failure = outcome.get('recovery_checkpoint', {})
            plan = (w.gear_library.recovery_route(failure) if job['kind'] == 'gear' else
                    recovery_route(w.release, job['action'], failure))
            w.transport.check_automatic_recovery(plan, w.release)
        except Exception as error:
            self.status, self.message = 'blocked', str(error)
            return False
        self.plan, self.revision = plan, w.recovery_revision
        self.status, self.message = 'pending', 'Checking stopped robot before recorded escape and preparation'
        self.generation = None
        return True

    def poll(self):
        w = self.worker
        with w.lock:
            if self.status != 'pending' or w.active or w.recovering:
                return
            w.recovering = True
            self.status = 'running'
            self.thread = threading.Thread(target=self.run, daemon=True)
            self.thread.start()

    def check(self):
        w = self.worker
        with w.lock:
            if (w.done.is_set() or self.revision != w.recovery_revision or
                    self.status != 'running' or not w.heartbeat_allowed()):
                raise RuntimeError('Automatic recovery interrupted; no retry')
        if getattr(w.transport, 'retire_requested', False):
            raise RuntimeError('Operator retired the controller')

    def run(self):
        w = self.worker
        try:
            # STOP acknowledgement precedes the gateway's settling/blend phase.
            deadline, stable = time.monotonic() + 15., None
            while time.monotonic() < deadline:
                self.check()
                if w.transport.check_automatic_recovery(self.plan, w.release):
                    stable = stable or time.monotonic()
                    if time.monotonic() - stable >= .5:
                        break
                else:
                    stable = None
                if w.done.wait(.1):
                    self.check()
            else:
                raise RuntimeError('Robot did not settle within the recovery window')
            self.check()
            w.transport.recover_supported(self.check)
            with w.lock:
                self.check()
                w.sequence.cancelled.clear()
            def check():
                self.check()
                w.sequence.check()
            w.transport.execute_recovery_route(self.plan, w.release, check)
            with w.lock:
                self.check()
                self.status, self.message = 'ready', 'Preparation restored; ready for a new task'
                self.generation = str(time.time_ns())
                w.recovery_generation = self.generation
                w.remote_recovery_result = self.message
                write(w.recovery_file, dict(required=False, reason=self.message,
                                            generation=self.generation))
        except Exception as error:
            try:
                w.sequence.cancel()
            except Exception:
                pass
            with w.lock:
                w.mark_recovery('Automatic recovery failed: ' + str(error))
                self.status, self.message = 'blocked', str(error)
        finally:
            with w.lock:
                w.recovering = False
