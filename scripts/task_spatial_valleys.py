"""Offline local valley replacement, certified under a spatial correspondence."""
import copy

import numpy as np
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from task_composition_smoothing import evaluate, bounds_from_joint_delta
from task_trajectory import time_seconds, coefficients, bernstein_controls, task_jerk_limits


def correspondence_bound(reference, candidate, model, side, check):
    from task_local_timing import _compose
    duration = time_seconds(reference.points[-1])
    times = np.unique(np.r_[[time_seconds(p) for p in reference.points],
                           np.linspace(0., duration, max(16, int(np.ceil(duration/.015))+1))])
    old = [evaluate(reference, times, k) for k in range(3)]
    arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(old[0], axis=0), axis=1))]
    span = time_seconds(candidate.points[-1])
    poly = coefficients(candidate.points[0], candidate.points[-1], span)
    u = np.linspace(0., 1., 257)
    points = np.polynomial.polynomial.polyval(u, poly).T
    new_arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(points, axis=0), axis=1))]
    if arc[-1] < 1e-12 or new_arc[-1] < 1e-12:
        raise ValueError('Stationary valley')
    keep = np.r_[True, np.diff(new_arc)>1e-14]
    mapped = np.interp(arc/arc[-1], new_arc[keep]/new_arc[-1], u[keep])
    mapped[0], mapped[-1] = 0., 1.
    delta = np.zeros(len(reference.joint_names))
    # Arc samples choose a correspondence, not a sampled safety test. Each
    # intervening affine map is certified against both exact polynomials.
    for i, dt in enumerate(np.diff(times)):
        check()
        a, b = [JointTrajectoryPoint(positions=old[0][j].tolist(),
            velocities=old[1][j].tolist(), accelerations=old[2][j].tolist()) for j in (i,i+1)]
        error = _compose(poly, np.array([mapped[i], mapped[i+1]-mapped[i]]))
        error[:6] -= coefficients(a, b, dt)
        delta = np.maximum(delta, np.max(np.abs(bernstein_controls(error)), axis=0))
    return bounds_from_joint_delta(delta, reference.joint_names, model, side+'_index_tcp')


def valley_centers(source, seams):
    duration = time_seconds(source.points[-1])
    times = np.linspace(0., duration, max(32, int(np.ceil(duration/.01))+1))
    speed = np.linalg.norm(evaluate(source, times, 1), axis=1)
    centers = []
    for i in range(1, len(times)-1):
        t = times[i]
        if t < .15 or duration-t < .15 or not any(abs(t-s)<.5 for s in seams):
            continue
        if speed[i] > speed[i-1] or speed[i] >= speed[i+1]:
            continue
        left = speed[max(0,i-30):i]; right = speed[i+1:i+31]
        shoulder = min(max(left), max(right))
        if shoulder > .002 and speed[i] < .3*shoulder:
            centers.append((t, speed[i]))
    # Prefer the deepest valley if several tiny minima occupy one neighborhood.
    selected = []
    for t, speed in sorted(centers, key=lambda item:item[1]):
        if all(abs(t-other)>.15 for other in selected):
            selected.append(float(t))
    return sorted(selected)


def refit_valleys(source, model, side, seams, limits, check, parameters=None, *, limits_at=None):
    from task_composition_preview import at, section, stamp, concatenate, check_positions
    from task_cruise_timing import execution_limit_factors
    from task_trajectory_speed import motion_limits
    vmax, amax = motion_limits(source, model, 1., 1., parameters)
    motion = [np.array([d[n] for n in source.joint_names])
              for d in (vmax, amax, task_jerk_limits(source.joint_names))]
    result = copy.deepcopy(source)
    bounds = dict(joint_rad=0., tcp_m=0., orientation_rad=0.)
    edits = []
    duration = time_seconds(source.points[-1]); upper = duration
    for center in reversed(valley_centers(source, seams)):
        check(); best = None
        for width in (.65, .45, .3, .2, .12):
            begin, end = max(.001, center-width), min(upper-.001, center+width)
            begin,end=round(begin*1e9)/1e9,round(end*1e9)/1e9
            if not begin < center < end or end-begin < .1:
                continue
            a, b = at(source, begin), at(source, end)
            chord = np.asarray(b.positions)-a.positions
            if np.dot(a.velocities, b.velocities) <= 0 or np.linalg.norm(chord) < 1e-5:
                continue
            reference = section(source, begin, end)
            local_limits=limits if limits_at is None else limits_at(begin,end)
            old_cost = sum(np.diff([time_seconds(p) for p in reference.points])*
                           execution_limit_factors(reference, motion, check))
            for ratio in (.35, .5, .7, 1.):
                span = (end-begin)*ratio
                bridge = JointTrajectory(joint_names=list(source.joint_names), points=[
                    stamp(copy.deepcopy(a), 0.), stamp(copy.deepcopy(b), span)])
                factor = max(execution_limit_factors(bridge, motion, check))
                cost = span*factor
                if cost >= old_cost*.98:
                    continue
                bound = correspondence_bound(reference, bridge, model, side, check)
                if any(bound[key] > limit*.98 for key, limit in zip(bounds, local_limits)):
                    continue
                check_positions(bridge, model)
                gain = old_cost-cost
                if best is None or gain > best[0]:
                    best = gain, begin, end, bridge, bound, old_cost, cost
        if best is None:
            continue
        gain, begin, end, bridge, bound, old_cost, cost = best
        result = concatenate([section(result, 0., begin), bridge,
                              section(result, end, time_seconds(result.points[-1]))])
        upper = begin
        for key in bounds:
            bounds[key] = max(bounds[key], bound[key])
        edits.append(dict(source_time_s=center, source_begin_s=begin, source_end_s=end,
            replacement_seconds=time_seconds(bridge.points[-1]),
            estimated_before_s=float(old_cost), estimated_after_s=float(cost),
            bounds=bound,
            candidate_mid_speed_rad_s=float(np.linalg.norm(at(bridge,time_seconds(bridge.points[-1])/2).velocities))))
    return result, bounds, edits
