"""Offline simultaneous preparation using the existing trajectory/timing helpers."""
import argparse
import sys
from pathlib import Path

from bundle import ROOT, load, read, write


def build(envelope, measured):
    sys.path.insert(0, str(ROOT / 'scripts'))
    from rosidl_runtime_py.convert import message_to_ordereddict
    from task_path_smoothing import joint_line_trajectory
    from task_planning_config import parameters_from_values
    from task_session import ARM_NAMES
    from task_trajectory import (stretch_to_limits, task_motion_limits, task_jerk_limits,
                                 validate_jerk_limits, samples, time_seconds)

    payload = envelope['payload']
    init = payload['initialization']
    names = list(ARM_NAMES) + ['waist_yaw_joint']
    target = init['target']
    params = parameters_from_values(payload['actions']['pull']['plan']['planning_config']['values'])
    trajectory = joint_line_trajectory(names, [measured[n] for n in names], [target[n] for n in names])
    velocity, acceleration = task_motion_limits(names, init['velocity_scale'], init['acceleration_scale'], params)
    jerk = task_jerk_limits(names)
    stretch_to_limits(trajectory, velocity, acceleration, jerk)
    validate_jerk_limits(trajectory, jerk)
    count = sum(1 for _ in samples(trajectory, velocity, acceleration))
    return dict(schema=1, hardware_executable=False, collision_checked=False,
                gateway_support_required='normal-profile 15-joint dual-arm preparation',
                bundle=envelope['sha256'], duration=time_seconds(trajectory.points[-1]),
                validated_samples=count, velocity_scale=.7, acceleration_scale=.7,
                trajectory=message_to_ordereddict(trajectory))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bundle', type=Path, required=True)
    parser.add_argument('--measured', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    measured = read(args.measured)
    write(args.output, build(load(args.bundle), measured.get('joints', measured)))
