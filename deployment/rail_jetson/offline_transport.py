"""Explicitly simulated sequence transport. No ROS, HandClient or hardware I/O."""
import threading

from bundle import digest


class OfflineTransport:
    simulation = True

    def __init__(self, step_seconds=.01):
        self.step_seconds = step_seconds
        self.joints = {}
        self.stopped = threading.Event()
        self.calls = []

    def assert_authority(self):
        if self.stopped.is_set():
            raise RuntimeError('Offline stop latched')

    def initialize(self, release, check):
        check()
        self.joints = dict(release['payload']['initialization']['target'])
        self.calls.append(dict(kind='initialize', simulation=True))
        return dict(status='SUCCEEDED', simulation=True, hardware_commands_sent=0)

    def verify_configuration(self, action):
        pass

    def return_home(self, release, check):
        from home_pose import reference
        check()
        self.stopped.wait(self.step_seconds)
        check()
        self.joints = reference()['target']
        self.calls.append(dict(kind='return_home', simulation=True))
        return dict(status='SUCCEEDED', simulation=True, hardware_commands_sent=0)

    def begin(self, action, check):
        check()

    def stationary(self, check):
        check()
        return dict(self.joints)

    def set_stage(self, side, profile, parameters, check):
        check()

    def align_check(self, trajectory, check):
        check()
        if any(abs(self.joints[n] - q) > .025 for n, q in
               zip(trajectory['joint_names'], trajectory['points'][0]['positions'])):
            raise ValueError('Offline start mismatch')

    def arm(self, step, trajectory, check, report_progress=None):
        check()
        self.calls.append(dict(kind=step['kind'], trajectory_sha256=digest(trajectory),
                               recorded_hand=step.get('recorded_hand'), delayed_hand=step.get('delayed_hand')))
        self.stopped.wait(self.step_seconds)
        check()
        if report_progress:
            for checkpoint in step.get('gear_progress', []):
                check()
                report_progress(checkpoint['progress'])
        self.joints.update(zip(trajectory['joint_names'], trajectory['points'][-1]['positions']))
        return dict(status=4, error_code=0, simulation=True, hardware_commands_sent=0)

    def hand_transition(self, step, check):
        check()
        self.calls.append(dict(kind='hand', target_q=step['target_q']))
        self.stopped.wait(self.step_seconds)
        check()
        return dict(success=True, simulation=True, hardware_commands_sent=0)

    def settle(self, endpoint, parameters, check):
        check()

    def stop_and_confirm(self):
        self.stopped.set()
        return True


def offline_join(measured, target, parameters, velocity, acceleration):
    """Synthetic endpoints for control-flow tests, NOT a motion planner."""
    return dict(joint_names=list(target), points=[
        dict(positions=[measured[n] for n in target], time_from_start=dict(sec=0, nanosec=0)),
        dict(positions=list(target.values()), time_from_start=dict(sec=1, nanosec=0))])
