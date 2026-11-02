"""Fixed-workcell rail paths, using wrist FK rather than the index-finger TCP."""
import math

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation, Slerp

from task_hand_control import checked_q, model_joints
from task_path_smoothing import joint_line_trajectory, smooth_candidates
from task_screen_motion import scale_time
from task_screen_teach import MOVING, JOINT_MARGIN, guarded_joints
from task_session import ARMS
from task_trajectory import (samples, stretch_to_limits, task_motion_limits, task_jerk_limits,
                             validate_jerk_limits, validate_waist_envelope, time_seconds)

LINK = 'right_wrist_yaw_link'
MODES = ('hold', 'release', 'cycle')
STATES = ('outer', 'inner_held', 'inner_released')
VELOCITY_SCALE = .7
ACCELERATION_SCALE = .7


def required_state(operation, mode):
    if operation not in ('push', 'pull') or mode not in MODES:
        raise ValueError('Invalid rail operation/mode')
    return 'outer' if operation == 'push' else ('inner_held' if mode == 'hold' else 'inner_released')


def cartesian(model, start, target, parameters, check=lambda: None, *, fixed_waist=False):
    """Stop-to-stop Cartesian line; validate the actual quintic joint interpolant."""
    a, b = (model.transform(LINK, q) for q in (start, target))
    rotations = Rotation.from_matrix(np.array([a[:3, :3], b[:3, :3]]))
    slerp = Slerp([0., 1.], rotations)
    distance = float(np.linalg.norm(b[:3, 3]-a[:3, 3]))
    angle = (rotations[0].inv()*rotations[1]).magnitude()
    count = max(3, math.ceil(distance/.002)+1, math.ceil(angle/math.radians(1.))+1,
                math.ceil(max(abs(start[n]-target[n]) for n in MOVING)/.04)+1)
    if count > 2000:
        raise ValueError('Rail waypoint interval is too large')
    progress = np.linspace(0., 1., count)
    names = ARMS['right']
    current = dict(start)
    values = [[start[n] for n in MOVING]]
    guarded_joints(model, [target[n] for n in MOVING], context='Rail target')
    for t in progress[1:]:
        check()
        xyz = a[:3, 3]+t*(b[:3, 3]-a[:3, 3])
        rotation = slerp(t)
        current['waist_yaw_joint'] = (start['waist_yaw_joint'] if fixed_waist else
            start['waist_yaw_joint']+t*(target['waist_yaw_joint']-start['waist_yaw_joint']))
        seed = np.array([current[n] for n in names])
        preferred = np.array([start[n]+t*(target[n]-start[n]) for n in names])
        lo = np.array([max(model.limits[n][0]+JOINT_MARGIN, current[n]-.12) for n in names])
        hi = np.array([min(model.limits[n][1]-JOINT_MARGIN, current[n]+.12) for n in names])
        def residual(q):
            check()
            pose = model.transform(LINK, dict(current, **dict(zip(names, q))))
            return np.r_[1000*(pose[:3, 3]-xyz),
                         10*(rotation.inv()*Rotation.from_matrix(pose[:3, :3])).as_rotvec(),
                         .02*(q-preferred)]
        if np.any(lo >= hi):
            raise ValueError('No local rail IK interval')
        result = least_squares(residual, np.clip(seed, lo, hi), bounds=(lo, hi), max_nfev=80)
        current.update(zip(names, map(float, result.x)))
        pose = model.transform(LINK, current)
        if (np.linalg.norm(pose[:3, 3]-xyz) > .0003 or
                (rotation.inv()*Rotation.from_matrix(pose[:3, :3])).magnitude() > math.radians(.3)):
            raise ValueError('Rail Cartesian IK failed; teach closer waypoints')
        values.append([current[n] for n in MOVING])
    # Preserve the taught endpoint, including its redundant-arm configuration.
    if max(abs(current[n]-target[n]) for n in MOVING) > (.025 if fixed_waist else .08):
        raise ValueError('Rail endpoint IK branch differs; add a waypoint')
    if not fixed_waist:
        values[-1] = [target[n] for n in MOVING]
    vmax, amax = task_motion_limits(MOVING, VELOCITY_SCALE, ACCELERATION_SCALE, parameters)
    jerk = task_jerk_limits(MOVING)
    error = None
    for trajectory, _ in smooth_candidates(MOVING, values, progress, .3):
        try:
            check()
            stretch_to_limits(trajectory, vmax, amax, jerk)
            rows = np.asarray(list(samples(trajectory, vmax, amax)))
            guarded_joints(model, rows, context='Rail interpolant')
            if fixed_waist and np.max(np.abs(rows[:, -1]-start['waist_yaw_joint'])) > 1e-9:
                raise ValueError('Rail contact waist must remain fixed')
            poses = []
            for i in range(0, len(rows), 1000):
                check()
                poses.append(model.batch_transforms([LINK], start, MOVING, rows[i:i+1000])[LINK])
            poses = np.concatenate(poses)
            times = np.concatenate([np.linspace(time_seconds(x), time_seconds(y),
                max(2, math.ceil((time_seconds(y)-time_seconds(x))/.005)+1))
                for x, y in zip(trajectory.points, trajectory.points[1:])])
            t = times/time_seconds(trajectory.points[-1])
            s = t**3*(10+t*(-15+6*t))
            expected = a[:3, 3]+s[:, None]*(b[:3, 3]-a[:3, 3])
            angles = (slerp(np.clip(s, 0., 1.)).inv()*Rotation.from_matrix(poses[:, :3, :3])).magnitude()
            if np.max(np.linalg.norm(poses[:, :3, 3]-expected, axis=1)) > .0005 or np.max(angles) > math.radians(.5):
                raise ValueError('Rail interpolant exceeds 0.5 mm / 0.5 degree model bounds')
            dt = np.diff(times); valid = dt > 1e-9
            speed = np.max(np.linalg.norm(np.diff(poses[:, :3, 3], axis=0)[valid], axis=1)/dt[valid])
            scale_time(trajectory, max(1., speed/.005*1.02))
            validate_jerk_limits(trajectory, jerk)
            lo, hi = model.limits['waist_yaw_joint']
            validate_waist_envelope(trajectory, lo+JOINT_MARGIN, hi-JOINT_MARGIN,
                                    vmax['waist_yaw_joint'], amax['waist_yaw_joint'])
            return trajectory
        except ValueError as exc:
            error = exc
    raise ValueError('No bounded Cartesian rail path: '+str(error))


def build(model, config, ready, start, operation, mode, parameters, check=lambda: None):
    required_state(operation, mode)
    if mode == 'cycle':raise ValueError('完整三段循环需要记录轨迹重放')
    points = config['points']
    if any(k not in points for k in ('A', 'B')) or any(not config['routes'][k] for k in ('A', 'B')):
        raise ValueError('Teach A, B and at least one approach waypoint for each end')
    poses = [model.transform(LINK, points[k]['joints']) for k in ('A', 'B')]
    if abs(points['A']['joints']['waist_yaw_joint']-points['B']['joints']['waist_yaw_joint']) > math.radians(.5):
        raise ValueError('A/B waist angles differ; teach with the waist held fixed')
    if (Rotation.from_matrix(poses[0][:3, :3]).inv()*Rotation.from_matrix(poses[1][:3, :3])).magnitude() > math.radians(.5):
        raise ValueError('A/B wrist orientations differ by more than 0.5 degree')
    if np.linalg.norm(poses[1][:3, 3]-poses[0][:3, 3]) < .005:
        raise ValueError('A/B rail stroke must exceed 5 mm')
    for name in ('open', 'grip'):
        checked_q(config['hands'][name]['feedback_q'])
    for slot in ('A', 'B'):
        if max(abs(a-b) for a, b in zip(checked_q(points[slot]['hand']['feedback_q']),
                                        config['hands']['grip']['feedback_q'])) > .02:
            raise ValueError('A/B must be taught with the recorded grip posture')
    segments = []
    current = dict(start)

    def append(trajectory, target, label, **extra):
        nonlocal current
        segments.append(dict(number=len(segments)+1, side='right', label=label, stage='rail',
            kind='arm', trajectory=trajectory, start=dict(current), end=dict(target),
            duration=time_seconds(trajectory.points[-1]), **extra))
        current = dict(target)

    def arm(target, label, contact=False):
        end = dict(current, **{k:target[k] for k in MOVING})
        trajectory = cartesian(model, current, end, parameters, check, fixed_waist=contact)
        end.update(zip(MOVING, trajectory.points[-1].positions))
        append(trajectory, end, label)

    def hand(posture):
        q = list(config['hands'][posture]['feedback_q'])
        end = dict(current, **model_joints('right', q))
        names = list(model_joints('right', q))
        trajectory = joint_line_trajectory(names, [current[k] for k in names], [end[k] for k in names])
        scale_time(trajectory, 1.)
        append(trajectory, end, 'Rail hand '+posture)
        segments[-1].update(kind='hand', target_q=q, posture=posture)

    def approach(slot):
        for i, record in enumerate(config['routes'][slot]):
            arm(record['joints'], f'Rail {slot} approach {i+1}')
        arm(points[slot]['joints'], 'Rail '+slot+' grasp position')

    def retreat(slot):
        for i, record in enumerate(reversed(config['routes'][slot])):
            arm(record['joints'], f'Rail {slot} retreat {i+1}')
        arm(ready, 'Rail return ready')

    if operation == 'push':
        hand('open'); approach('A'); hand('grip')
        arm(points['B']['joints'], 'Rail push A to B', True)
        if mode == 'release':
            hand('open'); retreat('B')
        final = 'inner_held' if mode == 'hold' else 'inner_released'
    else:
        if mode == 'release':
            hand('open'); approach('B')
        if 'pull_grip' in config['hands']:
            hand('pull_grip')
        elif mode == 'release':
            hand('grip')
        arm(points['A']['joints'], 'Rail pull B to A', True)
        hand('open'); retreat('A')
        final = 'outer'
    return dict(kind='rail', arm='right', start=dict(start), segments=segments, waist_assist=True,
                hardware_executable=True, collision_checked=False, contact_force_measured=False,
                rail=dict(operation=operation, mode=mode, final_state=final),
                description='Operator-confirmed fixed rail, Cartesian wrist paths, no visual guidance')
