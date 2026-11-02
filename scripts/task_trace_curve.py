"""Validate and retain the analytical quintic chosen in the keypoint editor."""
import math

import numpy as np
from scipy.interpolate import BSpline


def validate_curve(trace, model, names):
    info = trace['optimization']
    spec = info['spline']
    knots, controls = np.asarray(spec['knots']), np.asarray(spec['controls'])
    if (spec['degree'] != 5 or controls.ndim != 2 or controls.shape[1] != len(names) or
            knots.ndim != 1 or len(knots) != len(controls)+6 or
            not np.isfinite(knots).all() or not np.isfinite(controls).all() or
            np.any(np.diff(knots) < 0) or not np.all(knots[:6] == 0) or not np.all(knots[-6:] == 1)):
        raise ValueError('Invalid stored quintic curve')
    if len(knots[6:-6]) and (np.any(knots[6:-6] <= 0) or np.any(knots[6:-6] >= 1) or
                            np.any(np.diff(knots[6:-6]) <= 0)):
        raise ValueError('Curve interior knots must be simple (C4 continuity)')
    spline = BSpline(knots, controls, 5)
    start, end = info['interval_s']
    if not all(math.isfinite(v) for v in (start,end)) or end <= start:
        raise ValueError('Invalid curve interval')
    t = np.asarray(trace['raw']['time_s'])
    q = np.asarray(trace['raw']['positions'])
    if start < 0 or end > t[-1]+1e-9:raise ValueError('Curve interval exceeds the trajectory')
    mask = (t >= start) & (t <= end)
    if np.max(np.abs(spline((t[mask]-start)/(end-start))-q[mask])) > 1e-8:
        raise ValueError('Stored curve differs from the selected version samples')
    breaks = np.unique(knots)
    for a, b in zip(breaks, breaks[1:]):
        coeff = np.array([spline(a,nu=i)*(b-a)**i/math.factorial(i) for i in range(6)])
        for axis, name in enumerate(names):
            poly = coeff[:,axis]
            roots = np.polynomial.polynomial.polyroots(np.polynomial.polynomial.polyder(poly))
            probes = [0.,1.]+[r.real for r in roots if abs(r.imag)<1e-8 and 0<r.real<1]
            values = np.polynomial.polynomial.polyval(probes,poly)
            lo,hi = model.limits[name]
            if min(values)<lo+.02 or max(values)>hi-.02:
                raise ValueError('关键点曲线超出关节限位：'+name)
    return spline


def trajectory(trace, model, names):
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
    spline = validate_curve(trace,model,names)
    info = trace['optimization']
    start, end = info['interval_s']
    if not info['whole_trace'] or start != 0 or abs(end-trace['raw']['time_s'][-1])>1e-9:
        raise ValueError('Only a complete curve has an exact standalone trajectory')
    breaks = np.unique(spline.t)
    times = np.r_[np.concatenate([np.linspace(a,b,max(1,math.ceil((b-a)*end/.1))+1)[:-1]
                                  for a,b in zip(breaks,breaks[1:])]), 1.]
    result = JointTrajectory(joint_names=list(names))
    for value in times:
        ns = round(float(value*end)*1e9)
        u = ns*1e-9/end
        point = JointTrajectoryPoint(positions=spline(u).tolist(),velocities=(spline(u,1)/end).tolist(),
                                     accelerations=(spline(u,2)/end**2).tolist())
        point.time_from_start.sec,point.time_from_start.nanosec = divmod(ns,1_000_000_000)
        result.points.append(point)
    for point in (result.points[0],result.points[-1]):
        point.velocities=[0.]*len(names);point.accelerations=[0.]*len(names)
    return result, dict(recorded_seconds=end,raw_count=len(trace['raw']['time_s']),point_count=len(result.points),
        fit_error_rad=0.,fit_tolerance_rad=0.,control_points=len(spline.c),sample_policy='offline_edited',
        source='analytical_keypoint_curve',interior_continuity='C4',keypoint_count=len(info['key_times_s']))
