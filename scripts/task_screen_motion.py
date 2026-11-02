"""Endpoint-based screen paths. Joint/IK/timing checks, no collision queries."""
import math

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from task_path_smoothing import joint_line_trajectory, smooth_candidates
from task_screen_teach import MOVING, JOINT_MARGIN, guarded_joints, matrix, parameters
from task_session import ARMS
from task_trajectory import (samples, stretch_to_limits, task_motion_limits, task_jerk_limits,
                             validate_jerk_limits, validate_waist_envelope, time_seconds)


def tip(model, q, tcp):
    return model.transform('right_wrist_yaw_link', q) @ tcp


def scale_time(trajectory, factor):
    for p in trajectory.points:
        ns = round(time_seconds(p)*factor*1e9)
        p.time_from_start.sec, p.time_from_start.nanosec = divmod(ns, 1000000000)
        p.velocities = [v/factor for v in p.velocities]
        p.accelerations = [a/factor**2 for a in p.accelerations]


def return_to_transition(model, config, start, planning_parameters, check=lambda: None, *, waist_target=None,
                         hold_reference=None):
    """Explicit free-space return from measured joints; no contact or IK step."""
    check()
    p = parameters(config['parameters'])
    reference = dict(start)
    if waist_target is not None:
        if hold_reference is None or set(hold_reference) != set(MOVING):
            raise ValueError('Waist recovery requires the current gateway hold reference')
        if any(not math.isfinite(hold_reference[n]) or abs(hold_reference[n]-start[n]) > .025 for n in MOVING):
            raise ValueError('Waist hold reference differs from measured start by more than 0.025 rad')
        reference.update(hold_reference)
    target = (config['points']['T']['joints'] if waist_target is None else
              dict(reference, waist_yaw_joint=float(waist_target)))
    label = 'Move to T (right + waist)' if waist_target is None else f'Waist recovery {math.degrees(waist_target):.2f} deg'
    guarded_joints(model, [start[n] for n in MOVING], context=label+' start')
    guarded_joints(model, [reference[n] for n in MOVING], context=label+' command reference')
    guarded_joints(model, [target[n] for n in MOVING], context=label+' target')
    trajectory = joint_line_trajectory(MOVING, [reference[n] for n in MOVING], [target[n] for n in MOVING])
    vmax, amax = task_motion_limits(MOVING, p['velocity'], p['acceleration'], planning_parameters)
    jerk = task_jerk_limits(MOVING)
    stretch_to_limits(trajectory, vmax, amax, jerk)
    lower = np.array([model.limits[n][0]+JOINT_MARGIN for n in MOVING])
    upper = np.array([model.limits[n][1]-JOINT_MARGIN for n in MOVING])
    for index, sample in enumerate(samples(trajectory, vmax, amax)):
        if index % 100 == 0: check()
        guarded_joints(model, sample, context=label)
    validate_jerk_limits(trajectory, jerk)
    validate_waist_envelope(trajectory, lower[-1], upper[-1], vmax[MOVING[-1]], amax[MOVING[-1]])
    end = dict(start, **{n:float(target[n]) for n in MOVING})
    segment = dict(side='right', label=label, stage='screen_transit',
                   trajectory=trajectory, start=dict(start), end=end,
                   duration=time_seconds(trajectory.points[-1]), number=1)
    check()
    return dict(kind='screen', arm='right', start=dict(start), segments=[segment], waist_assist=True,
                hardware_executable=True, collision_checked=False, contact_force_measured=False,
                screen=dict(selected='T' if waist_target is None else 'WAIST',
                            operation='return_T' if waist_target is None else 'waist_recovery',
                            target_waist_rad=float(target['waist_yaw_joint']), dry=True, extra_feed_mm=0.,
                            **(dict(hold_reference_policy='continuous_hold_v1',
                                    hold_reference={n:reference[n] for n in MOVING}) if waist_target is not None else {})),
                description=('Operator-confirmed free-space return to taught joints; no collision verification' if waist_target is None else
                             'Operator-confirmed waist-only joint motion; arms held, no collision verification'))


def normal_states(model, record, normal, approach_m, clearance_m, check):
    tcp = matrix(record['wrist_from_tcp'])
    origin = matrix(record['pelvis_from_tcp'])
    current = dict(record['joints'])
    guarded_joints(model, [current[n] for n in MOVING], context='示教触点')
    names = ARMS['right']
    limits = model.limits
    rotation = Rotation.from_matrix(origin[:3, :3])
    distances = sorted(set(np.linspace(0., approach_m, max(3, math.ceil(approach_m/.002)+1)).tolist()+[clearance_m]))
    states = [dict(current)]
    for distance in distances[1:]:
        check()
        target = origin[:3, 3]+distance*normal
        seed = np.array([current[n] for n in names])
        lo = np.array([max(limits[n][0]+JOINT_MARGIN, current[n]-.10) for n in names])
        hi = np.array([min(limits[n][1]-JOINT_MARGIN, current[n]+.10) for n in names])
        def residual(values):
            check()
            q = dict(current); q.update(zip(names, values))
            pose = tip(model, q, tcp)
            return np.r_[1000*(pose[:3, 3]-target),
                         10*(rotation.inv()*Rotation.from_matrix(pose[:3, :3])).as_rotvec(),
                         .01*(values-seed)]
        if np.any(lo >= hi):
            raise ValueError('No guarded local IK interval')
        result = least_squares(residual, np.clip(seed, lo, hi), bounds=(lo, hi), max_nfev=60)
        current = dict(current); current.update(zip(names, map(float, result.x)))
        pose = tip(model, current, tcp)
        if (np.linalg.norm(pose[:3, 3]-target) > .0003 or
                (rotation.inv()*Rotation.from_matrix(pose[:3, :3])).magnitude() > math.radians(.3)):
            raise ValueError('Normal approach IK failed; adjust the taught posture or approach distance')
        states.append(current)
    return distances, states


def plan(model, config, selected, start, dry, planning_parameters, check=lambda: None):
    p = parameters(config['parameters'])
    point = config['points'][selected]
    for label, q in (('当前起点', start), ('T', config['points']['T']['joints']), (selected, point['joints'])):
        guarded_joints(model, [q[n] for n in MOVING], context=label)
    tcp = matrix(point['wrist_from_tcp'])
    normal = np.asarray(config['plane']['outward_normal'])
    origin = matrix(point['pelvis_from_tcp'])
    clearance = p['dry_clearance_mm']/1000. if dry else 0.
    distances, outward = normal_states(model, point, normal, p['approach_mm']/1000., clearance, check)
    outward = outward[distances.index(clearance):]
    # Only the selected right arm and waist may change. Other joints stay measured.
    def owned(q):
        return dict(start, **{n:q[n] for n in MOVING})
    outward = [owned(q) for q in outward]
    vmax, amax = task_motion_limits(MOVING, p['velocity'], p['acceleration'], planning_parameters)
    jerk = task_jerk_limits(MOVING)
    lower = np.array([model.limits[n][0]+JOINT_MARGIN for n in MOVING])
    upper = np.array([model.limits[n][1]-JOINT_MARGIN for n in MOVING])
    segments = []

    def finish(trajectory, initial, label, normal_path=False):
        check()
        stretch_to_limits(trajectory, vmax, amax, jerk)
        if normal_path:
            # Dense FK validation checks the actual gateway Hermite interpolation.
            sampled = np.asarray(list(samples(trajectory, vmax, amax)))
            transforms = []
            for offset in range(0, len(sampled), 1000):
                check()
                transforms.append(model.batch_transforms(['right_wrist_yaw_link'], initial, MOVING,
                    sampled[offset:offset+1000])['right_wrist_yaw_link'] @ tcp)
            transforms = np.concatenate(transforms)
            positions = transforms[:, :3, 3]
            delta = positions-origin[:3, 3]
            distance = delta@normal
            angles = (Rotation.from_matrix(origin[:3, :3]).inv()*Rotation.from_matrix(transforms[:, :3, :3])).magnitude()
            if (np.max(np.linalg.norm(delta-distance[:, None]*normal, axis=1)) > .0005 or
                    np.min(distance) < clearance-.0003 or np.max(distance) > p['approach_mm']/1000.+.0003 or
                    np.max(angles) > math.radians(.5) or
                    np.max(np.abs(sampled[:, -1]-point['joints']['waist_yaw_joint'])) > 1e-9):
                raise ValueError('Interpolated normal path exceeds TCP/orientation/waist bounds')
            # Repeated boundary samples are included; associate the same times.
            times = np.concatenate([np.linspace(time_seconds(a), time_seconds(b),
                    max(2, math.ceil((time_seconds(b)-time_seconds(a))/.005)+1))
                    for a, b in zip(trajectory.points, trajectory.points[1:])])
            dt = np.diff(times); valid = dt > 1e-9
            speed = np.max(np.linalg.norm(np.diff(positions, axis=0)[valid], axis=1)/dt[valid])
            scale_time(trajectory, max(1., speed/(p['speed_mm_s']/1000.)*1.02))
        sampled = np.asarray(list(samples(trajectory, vmax, amax)))
        guarded_joints(model, sampled, context=label)
        validate_jerk_limits(trajectory, jerk)
        i = MOVING.index('waist_yaw_joint')
        validate_waist_envelope(trajectory, lower[i], upper[i], vmax[MOVING[i]], amax[MOVING[i]])
        end = dict(initial); end.update(zip(MOVING, trajectory.points[-1].positions))
        segment = dict(side='right', label=label, stage='screen_normal' if normal_path else 'screen_transit',
                       trajectory=trajectory, start=dict(initial), end=end,
                       duration=time_seconds(trajectory.points[-1]), number=len(segments)+1)
        segments.append(segment)
        return end

    def line(a, b, label):
        return finish(joint_line_trajectory(MOVING, [a[n] for n in MOVING], [b[n] for n in MOVING]), a, label)

    def normal_segment(states, label):
        values = np.array([[q[n] for n in MOVING] for q in states])
        if len(states) == 2:
            return finish(joint_line_trajectory(MOVING, values[0], values[1]), states[0], label, True)
        # Distance, rather than sample index, also handles the inserted dry endpoint.
        xyz = np.array([tip(model, q, tcp)[:3, 3] for q in states])
        progress = np.r_[0., np.cumsum(np.linalg.norm(np.diff(xyz, axis=0), axis=1))]
        last_error = None
        for trajectory, _ in smooth_candidates(MOVING, values, progress, .3):
            try:
                return finish(trajectory, states[0], label, True)
            except ValueError as error:
                last_error = error
        raise ValueError('No bounded smooth normal path: '+str(last_error))

    current = line(start, outward[-1], selected+' outside approach (right + waist)')
    inward = list(reversed(outward)); inward[0] = current
    current = normal_segment(inward, selected+(' dry approach' if dry else ' taught touch'))
    if p['dwell_s'] > 0:
        trajectory = JointTrajectory(joint_names=list(MOVING))
        for t in (0., p['dwell_s']):
            pt = JointTrajectoryPoint(positions=[current[n] for n in MOVING], velocities=[0.]*8, accelerations=[0.]*8)
            pt.time_from_start.sec, pt.time_from_start.nanosec = divmod(round(t*1e9), 1000000000)
            trajectory.points.append(pt)
        finish(trajectory, current, selected+' dwell', True)
    outward[0] = current
    current = normal_segment(outward, selected+' normal withdrawal')
    line(current, owned(config['points']['T']['joints']), 'Return T (right + waist)')
    return dict(kind='screen', arm='right', start=dict(start), segments=segments, waist_assist=True,
                hardware_executable=True, collision_checked=False, contact_force_measured=False,
                screen=dict(selected=selected, dry=bool(dry), extra_feed_mm=0.),
                description='Joint limits, continuity and timing checked; path clearance is operator-confirmed')
