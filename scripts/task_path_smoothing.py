"""Bounded quintic joint-path fitting; candidates still require robot validation."""
import math

import numpy as np
from scipy.interpolate import BSpline
from scipy.special import betaincinv
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


def joint_line_trajectory(names, start, end):
    q0=np.asarray(start,dtype=float);delta=np.asarray(end,dtype=float)-q0
    if q0.shape!=(len(names),) or delta.shape!=q0.shape or not np.isfinite([q0,delta]).all():
        raise ValueError('Invalid joint-line endpoints')
    trajectory=JointTrajectory(joint_names=list(names))
    # Subdivision preserves the exact curve and tightens the gateway's Bernstein
    # enclosure; a single long polynomial gives an unnecessarily loose waist cap.
    for t in np.linspace(0.,1.,21):
        s=t**3*(10.+t*(-15.+6.*t))
        v=30.*t*t*(1.-t)**2
        a=60.*t*(1.-t)*(1.-2.*t)
        point=JointTrajectoryPoint(positions=(q0+s*delta).tolist(),
            velocities=(v*delta).tolist(),accelerations=(a*delta).tolist())
        ns=round(float(t)*1e9)
        point.time_from_start.sec=ns//1000000000;point.time_from_start.nanosec=ns%1000000000
        trajectory.points.append(point)
    return trajectory


def joint_waypoint_trajectory(names, positions):
    q=np.asarray(positions,dtype=float)
    if q.ndim!=2 or q.shape[1]!=len(names) or len(q)<2 or not np.isfinite(q).all():
        raise ValueError('Invalid joint-waypoint path')
    trajectory=JointTrajectory(joint_names=list(names))
    for index,(start,end) in enumerate(zip(q,q[1:])):
        leg=joint_line_trajectory(names,start,end)
        for point in leg.points[int(index>0):]:
            point.time_from_start.sec+=index
            trajectory.points.append(point)
    return trajectory


def smooth_candidates(names, positions, progress, max_deviation_deg, *, counts=(6,10,14,22,34), adaptive_knots=False):
    q=np.asarray(positions,dtype=float);s=np.asarray(progress,dtype=float)
    if (q.ndim!=2 or q.shape!=(len(s),len(names)) or len(s)<3 or
            not np.isfinite(q).all() or not np.isfinite(s).all() or
            np.any(np.diff(s)<0) or s[-1]<=s[0] or
            not math.isfinite(max_deviation_deg) or max_deviation_deg<=0):
        raise ValueError('Invalid joint-path fit input')
    s=(s-s[0])/(s[-1]-s[0])
    # Inverse quintic progress lets both endpoint velocity and acceleration be
    # exactly zero without forcing every Cartesian sample to be a stopping point.
    x=betaincinv(3.,3.,s)
    for count in counts:
        interior=np.linspace(0.,1.,count-4)[1:-1]
        if adaptive_knots:interior=np.quantile(x,interior)
        knots=np.r_[np.zeros(6),interior,np.ones(6)]
        basis=BSpline(knots,np.eye(count),5)(x)
        controls=np.zeros((count,len(names)))
        controls[:3]=q[0];controls[-3:]=q[-1]
        if count>6:
            free=basis[:,3:-3]
            solution,_,rank,_=np.linalg.lstsq(free,q-basis@controls,rcond=1e-8)
            if rank<free.shape[1]:continue
            controls[3:-3]=solution
        spline=BSpline(knots,controls,5)
        deviation=float(np.max(np.abs(spline(x)-q)))
        if deviation>math.radians(max_deviation_deg):continue
        trajectory=JointTrajectory(joint_names=list(names))
        # Every knot is retained: Hermite interpolation with analytic derivatives
        # reproduces each quintic piece, rather than inventing derivatives via TOTG.
        for t in np.unique(knots):
            point=JointTrajectoryPoint(positions=spline(t).tolist(),
                velocities=spline(t,1).tolist(),accelerations=spline(t,2).tolist())
            ns=round(float(t)*1e9)
            point.time_from_start.sec=ns//1000000000
            point.time_from_start.nanosec=ns%1000000000
            trajectory.points.append(point)
        yield trajectory,dict(control_points=count,max_fit_deviation_deg=math.degrees(deviation))
