"""Headless recorded-action sequence; transports own real feedback and authority.

The CLI only materializes offline trajectories. It never creates a ROS node or
hardware client. A production transport must independently enforce ownership.
"""
import argparse
import copy
import json
import math
from pathlib import Path
import sys
import threading

from bundle import ROOT, digest, read, write
from releases import load_release
from automatic_recovery import TrajectoryRejected, endpoint


class Cancelled(RuntimeError):
    pass


class LocalJoinPlanner:
    def __init__(self, urdf):
        sys.path.insert(0, str(ROOT / 'scripts'))
        from workbench_model import OfflineModel
        self.model = OfflineModel(urdf)

    def __call__(self, measured, target, parameters, velocity=.7, acceleration=.7):
        from rosidl_runtime_py.convert import message_to_ordereddict
        from task_path_smoothing import joint_line_trajectory
        from task_planning_config import parameters_from_values
        from task_trajectory_speed import scale_arm_segments, motion_limits
        from task_trajectory import samples
        names = list(target)
        trajectory = joint_line_trajectory(names, [measured[n] for n in names], [target[n] for n in names])
        params = parameters_from_values(parameters)
        segments, _ = scale_arm_segments([dict(label='local join', trajectory=trajectory)], 1., self.model,
            params, policy='uniform', velocity=velocity, acceleration=acceleration)
        trajectory = segments[0]['trajectory']
        vmax, amax = motion_limits(trajectory, self.model, velocity, acceleration, params)
        for positions in samples(trajectory, vmax, amax):
            for name, value in zip(names, positions):
                lo, hi = self.model.limits[name]
                if not lo + .02 - 1e-7 <= value <= hi - .02 + 1e-7:
                    raise ValueError('Local join exceeds joint margin: ' + name)
        return message_to_ordereddict(trajectory)


def scales(action):
    record = action['plan'].get('speed_scaling', {})
    return record.get('velocity_limit_scaling', .7), record.get('acceleration_limit_scaling', .7)


def hand_step_policy(action, step):
    """Restore the WSL gear policy omitted by the original optimized exporter."""
    if (action['plan'].get('kind') == 'optimized_task' and
            action['plan'].get('action') in ('gear_roundtrip_lower_upper', 'gear_roundtrip_upper_lower')):
        step = dict(step)
        step.setdefault('hand_target_required', False)
        step['hand_completion_policy'] = 'landing-gear-command-and-fresh-feedback-v1'
        if step['label'].endswith(' hand grip') and step['hand_target_required'] is False:
            step.update(finger_sequence='four_then_thumb', duration=1.)
        elif step['label'].endswith(' hand open') and step['hand_target_required'] is False:
            step.update(finger_sequence='thumb_then_four', duration=1.)
    return step


def alignment_already_satisfied(step, measured):
    if step.get('kind') != 'join' or step.get('label') != 'align to pinned task start':
        return False
    target = step.get('target', {})
    return bool(target) and all(n in measured and math.isfinite(measured[n]) and math.isfinite(q)
                               and abs(measured[n]-q) <= .005 for n,q in target.items())


def materialize(envelope, name, measured, planner):
    """Offline reference, not a runnable startup plan from a stale measured state."""
    action = envelope['payload']['actions'][name]
    parameters = action['plan']['planning_config']['values']
    q = dict(measured)
    result = []
    for original in action['program']:
        step = copy.deepcopy(original)
        if step['kind'] == 'join':
            step['trajectory'] = planner(q, step['target'], parameters, *scales(action))
        if step['kind'] in ('join', 'recorded'):
            trajectory = step['trajectory']
            names = trajectory['joint_names']
            if step['kind'] == 'recorded':
                if any(abs(q[n] - v) > .025 for n, v in zip(names, trajectory['points'][0]['positions'])):
                    raise ValueError('Program lacks an alignment to recorded start')
                if digest(trajectory) != original['trajectory_sha256']:
                    raise ValueError('Recorded curve changed during materialization')
            q.update(zip(names, trajectory['points'][-1]['positions']))
        result.append(step)
    return dict(release_sha256=envelope['sha256'], action=name, steps=result,
                recorded_arrays_unchanged=True, hardware_commands_sent=0, hardware_executable=False,
                collision_checked=False, final_joints=q)


class SequenceExecutor:
    """No automatic ARM, output acquisition, uncertain-goal retry, or fault reset.

    The transport must check its authority grant before every command, and stop
    both arms and active fingers on failure. Cancellation uses a separate path.
    """
    def __init__(self, transport, planner):
        self.transport = transport
        self.planner = planner
        self.cancelled = threading.Event()
        self.lock = threading.Lock()

    def check_cancelled(self):
        if self.cancelled.is_set():
            raise Cancelled('Task cancelled; interrupted contact segments are never resumed')

    def check(self):
        self.check_cancelled()
        self.transport.assert_authority()

    def cancel(self):
        self.cancelled.set()
        return self.transport.stop_and_confirm()

    def execute(self, envelope, name, on_event=lambda event: None, action_override=None):
        if not self.lock.acquire(blocking=False):
            raise RuntimeError('Executor occupied')
        evidence = []
        recovery_checkpoint = None
        try:
            # A stopped executor is not reset implicitly by submitting another task.
            self.check()
            action = action_override if action_override is not None else envelope['payload']['actions'][name]
            parameters = action['plan']['planning_config']['values']
            self.transport.verify_configuration(action)
            self.transport.begin(action, self.check)
            for index, step in enumerate(action['program']):
                self.check()
                if step['kind'] == 'milestone':
                    on_event(dict(status='RUNNING', stage=index + 1, label=step['label'],
                                  business_progress=step['business_progress'], release_sha256=envelope['sha256']))
                    continue
                on_event(dict(status='RUNNING', stage=index + 1, label=step['label'],
                              release_sha256=envelope['sha256']))
                measured = self.transport.stationary(self.check)
                if index == 0 and alignment_already_satisfied(step, measured):
                    self.check()
                    evidence.append(dict(label=step['label'], result=dict(status='ALREADY_ALIGNED',
                        max_error_rad=max(abs(measured[n]-q) for n,q in step['target'].items()),
                        tolerance_rad=.005, trajectory_sent=False)))
                    continue
                if step['kind'] == 'hand':
                    step = hand_step_policy(action, step)
                    self.transport.set_stage(step['side'], 'normal', parameters, self.check)
                    result = self.transport.hand_transition(step, self.check)
                    result = dict(result, hand_target_required=step.get('hand_target_required', True),
                                  grasp_verified=False)
                else:
                    trajectory = (self.planner(measured, step['target'], parameters, *scales(action))
                                  if step['kind'] == 'join' else copy.deepcopy(step['trajectory']))
                    if step['kind'] == 'recorded' and digest(trajectory) != step['trajectory_sha256']:
                        raise RuntimeError('Immutable recording changed')
                    self.transport.set_stage(step['side'], step['profile'], parameters, self.check)
                    # Fresh measured admission remains necessary after a local solve or gain switch.
                    self.transport.align_check(trajectory, self.check)
                    self.check()
                    try:
                        if step.get('gear_progress'):
                            def report_progress(value):
                                self.check()
                                on_event(dict(status='RUNNING', stage=index + 1,
                                    label=step.get('progress_label', 'Gear motion cycle completed; external lock signals unverified'),
                                    business_progress=value, release_sha256=envelope['sha256']))
                            result = self.transport.arm(step, trajectory, self.check,
                                                        report_progress=report_progress)
                        else:
                            result = self.transport.arm(step, trajectory, self.check)
                    except TrajectoryRejected:
                        recovery_checkpoint = dict(kind='rejected', index=index,
                                                   gear_position=step.get('gear_start'),
                                                   checkpoint=endpoint(trajectory, 0))
                        raise
                    if result.get('status') != 4 or result.get('error_code') != 0:
                        if result.get('error_code') == -5:
                            recovery_checkpoint = dict(kind='endpoint', index=index,
                                                       gear_position=step.get('gear_end'),
                                                       checkpoint=endpoint(trajectory, -1))
                        raise RuntimeError('Arm action did not complete successfully: ' + str(result))
                    target = endpoint(trajectory, -1)
                    try:
                        self.transport.settle(target, parameters, self.check)
                    except TimeoutError:
                        recovery_checkpoint = dict(kind='endpoint', index=index, checkpoint=target,
                                                   gear_position=step.get('gear_end'))
                        raise
                evidence.append(dict(label=step['label'], result=result))
            self.transport.set_stage(action['plan']['arm'], 'normal', parameters, self.check)
            self.check()
            return dict(status='SUCCEEDED', evidence=evidence, release_sha256=envelope['sha256'],
                        external_signal_verified=False)
        except Exception as error:
            self.cancelled.set()
            try:
                stopped = self.transport.stop_and_confirm()
            except Exception:
                stopped = False
            return dict(status='CANCELLED' if isinstance(error, Cancelled) and stopped else
                        'FAILED' if stopped else 'STOP_UNCONFIRMED',
                        message=str(error), stop_confirmed=stopped, evidence=evidence,
                        requires_recovery=True, recovery_checkpoint=recovery_checkpoint or {},
                        release_sha256=envelope['sha256'])
        finally:
            self.lock.release()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bundle', type=Path, required=True)
    parser.add_argument('--action', required=True)
    parser.add_argument('--measured', type=Path, required=True)
    parser.add_argument('--urdf', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    measured = read(args.measured)
    result = materialize(load_release(args.bundle), args.action, measured.get('joints', measured), LocalJoinPlanner(args.urdf))
    write(args.output, result)
    print(json.dumps({k: v for k, v in result.items() if k not in ('steps', 'final_joints')}))
