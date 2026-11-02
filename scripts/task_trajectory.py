"""Validate/sampling of the same Hermite interpolation used by the arm gateway."""
import math
from functools import lru_cache
import numpy as np


def time_seconds(point):
    return point.time_from_start.sec+point.time_from_start.nanosec*1e-9


def sample_count(trajectory):
    return sum(max(2, math.ceil((time_seconds(b)-time_seconds(a))/.005)+1)
               for a,b in zip(trajectory.points,trajectory.points[1:]))


def coefficients(a,b,dt):
    q0,q1=np.asarray(a.positions),np.asarray(b.positions)
    c=np.zeros((6,len(q0))); c[0]=q0
    if len(a.velocities)==len(q0) and len(b.velocities)==len(q0):
        v0,v1=np.asarray(a.velocities)*dt,np.asarray(b.velocities)*dt
        c[1]=v0
        if len(a.accelerations)==len(q0) and len(b.accelerations)==len(q0):
            c[2]=np.asarray(a.accelerations)*dt*dt/2
            delta=q1-c[0]-c[1]-c[2]
            v=v1-c[1]-2*c[2]
            acc=np.asarray(b.accelerations)*dt*dt-2*c[2]
            c[3]=10*delta-4*v+acc/2
            c[4]=-15*delta+7*v-acc
            c[5]=6*delta-3*v+acc/2
        else:
            c[2]=3*(q1-q0)-2*v0-v1
            c[3]=-2*(q1-q0)+v0+v1
    else: c[1]=q1-q0
    return c


def condition_endpoints(trajectory):
    """Make arm trajectory boundaries continuous with the gateway's zero-speed hold."""
    if len(trajectory.points)<2: raise ValueError('Empty timed path')
    count=len(trajectory.joint_names)
    for point in trajectory.points:
        if (len(point.positions)!=count or len(point.velocities)!=count or
                len(point.accelerations)!=count):
            raise ValueError('Timed arm path requires complete position/velocity/acceleration arrays')
        if any(not np.isfinite(np.asarray(values,dtype=float)).all()
               for values in (point.positions,point.velocities,point.accelerations)):
            raise ValueError('Nonfinite timed arm path')
    zeros=[0.]*count
    for point in (trajectory.points[0],trajectory.points[-1]):
        point.velocities=list(zeros)
        point.accelerations=list(zeros)
    return trajectory


def peak(polynomial):
    roots=np.polynomial.polynomial.polyroots(np.polynomial.polynomial.polyder(polynomial))
    locations=[0.,1.]+[float(x.real) for x in roots if abs(x.imag)<1e-8 and 0<x.real<1]
    return max(abs(np.polynomial.polynomial.polyval(t,polynomial)) for t in locations)


@lru_cache(maxsize=8)
def _bernstein_matrix(degree):
    result=np.zeros((degree+1,degree+1))
    for k in range(degree+1):
        for i in range(k+1):result[k,i]=math.comb(k,i)/math.comb(degree,i)
    result.flags.writeable=False
    return result


def bernstein_controls(power):
    """Match the gateway's conservative whole-segment waist envelope."""
    return _bernstein_matrix(len(power)-1) @ power


def validate_waist_envelope(trajectory, lower, upper, velocity_limit, acceleration_limit):
    axis=trajectory.joint_names.index('waist_yaw_joint')
    for a,b in zip(trajectory.points,trajectory.points[1:]):
        dt=time_seconds(b)-time_seconds(a)
        c=coefficients(a,b,dt)[:,axis]
        d=np.array([i*c[i]/dt for i in range(1,6)])
        dd=np.array([i*d[i]/dt for i in range(1,5)])
        for power,lo,hi in ((c,lower,upper),(d,-velocity_limit,velocity_limit),
                            (dd,-acceleration_limit,acceleration_limit)):
            controls=bernstein_controls(power)
            if not np.isfinite(controls).all() or np.min(controls)<lo-1e-6 or np.max(controls)>hi+1e-6:
                raise ValueError('Waist interpolation envelope exceeds gateway limits')


def task_jerk_limits(names):
    """Absolute command jerk caps, including the task's additional waist axis."""
    return dict.fromkeys(names,5.0)


def task_motion_limits(names, velocity, acceleration, parameters=None):
    from task_planning_config import PlanningParameters
    parameters=parameters or PlanningParameters()
    return ({n:(parameters.waist_velocity_rad_s if n=='waist_yaw_joint' else parameters.arm_velocity_rad_s)*velocity for n in names},
            dict.fromkeys(names,1.5*acceleration))


def axis_limit(limits, name):
    value=limits[name] if isinstance(limits,dict) else limits
    if not math.isfinite(value) or value<=0:raise ValueError('Invalid motion limit: '+name)
    return value


def stretch_to_limits(trajectory, velocity_limit, acceleration_limit, jerk_limits=None):
    """Stretch final Hermite timing without changing its joint-space curve."""
    if jerk_limits is not None:
        for name in trajectory.joint_names:
            limit=jerk_limits[name]
            if not math.isfinite(limit) or limit<=0:
                raise ValueError('Invalid trajectory jerk limit')
    factor=1.
    for a,b in zip(trajectory.points,trajectory.points[1:]):
        dt=time_seconds(b)-time_seconds(a)
        if not math.isfinite(dt) or dt<=0: raise ValueError('Invalid trajectory duration')
        c=coefficients(a,b,dt)
        if not np.isfinite(c).all(): raise ValueError('Nonfinite trajectory coefficients')
        d=np.array([i*c[i]/dt for i in range(1,6)])
        dd=np.array([i*d[i]/dt for i in range(1,5)])
        ddd=np.array([i*dd[i]/dt for i in range(1,4)]) if jerk_limits is not None else None
        vb=np.max(np.abs(bernstein_controls(d)),axis=0)
        ab=np.max(np.abs(bernstein_controls(dd)),axis=0)
        for axis,name in enumerate(trajectory.joint_names):
            vmax=axis_limit(velocity_limit,name);amax=axis_limit(acceleration_limit,name)
            if name=='waist_yaw_joint':
                factor=max(factor,vb[axis]/vmax,math.sqrt(ab[axis]/amax))
            else:
                # A Bernstein enclosure proves small peaks without polynomial roots.
                if vb[axis]>factor*vmax:
                    factor=max(factor,peak(d[:,axis])/vmax)
                if ab[axis]>factor*factor*amax:
                    factor=max(factor,math.sqrt(peak(dd[:,axis])/amax))
            if jerk_limits is not None:
                factor=max(factor,(peak(ddd[:,axis])/jerk_limits[name])**(1./3.))
    if factor>1.:
        factor*=1.005
        duration=time_seconds(trajectory.points[-1])*factor
        if not math.isfinite(duration) or duration>=2147483647:
            raise ValueError('Retimed duration exceeds ROS duration range')
        for point in trajectory.points:
            ns=round(time_seconds(point)*factor*1e9)
            point.time_from_start.sec=ns//1000000000
            point.time_from_start.nanosec=ns%1000000000
            point.velocities=[float(v/factor) for v in point.velocities]
            point.accelerations=[float(a/(factor*factor)) for a in point.accelerations]
    return factor


def validate_jerk_limits(trajectory, jerk_limits):
    """Check every polynomial extremum after endpoint edits and ns rounding."""
    for a,b in zip(trajectory.points,trajectory.points[1:]):
        dt=time_seconds(b)-time_seconds(a)
        if not math.isfinite(dt) or dt<=0:raise ValueError('Invalid trajectory duration')
        c=coefficients(a,b,dt)
        jerk=np.polynomial.polynomial.polyder(c,3)/dt**3
        if not np.isfinite(jerk).all():raise ValueError('Nonfinite trajectory jerk')
        for axis,name in enumerate(trajectory.joint_names):
            limit=jerk_limits[name]
            if not math.isfinite(limit) or limit<=0:raise ValueError('Invalid trajectory jerk limit')
            if peak(jerk[:,axis])>limit+1e-4:
                raise ValueError('Gateway interpolation exceeds jerk bound: '+name)


def samples(trajectory, velocity_limit, acceleration_limit):
    if len(trajectory.points)<2: raise ValueError('Empty timed path')
    count=len(trajectory.joint_names)
    for a,b in zip(trajectory.points,trajectory.points[1:]):
        dt=time_seconds(b)-time_seconds(a)
        if not math.isfinite(dt) or dt<=0 or len(a.positions)!=count or len(b.positions)!=count:
            raise ValueError('Invalid trajectory time/position dimensions')
        c=coefficients(a,b,dt)
        if not np.isfinite(c).all(): raise ValueError('Nonfinite Hermite coefficients')
        d=np.array([i*c[i]/dt for i in range(1,6)])
        dd=np.array([i*d[i]/dt for i in range(1,5)])
        # Check actual polynomial extrema, not merely declared endpoint derivatives.
        vb=np.max(np.abs(bernstein_controls(d)),axis=0)
        ab=np.max(np.abs(bernstein_controls(dd)),axis=0)
        for axis in range(count):
            name=trajectory.joint_names[axis]
            for polynomial,bound,enclosure in ((d[:,axis],axis_limit(velocity_limit,name),vb[axis]),
                                                (dd[:,axis],axis_limit(acceleration_limit,name),ab[axis])):
                if enclosure>bound+1e-4 and peak(polynomial)>bound+1e-4:
                    raise ValueError('Gateway interpolation exceeds velocity/acceleration bound')
        fractions=np.linspace(0,1,max(2,math.ceil(dt/.005)+1))
        yield from np.polynomial.polynomial.polyval(fractions,c).T
