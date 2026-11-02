"""Continuous, path-preserving local timing with certified Hermite resampling."""
import copy
import hashlib
import math
from collections import OrderedDict
from threading import Lock

import numpy as np
from scipy.optimize import minimize
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from task_trajectory import (coefficients, time_seconds, bernstein_controls,
                             stretch_to_limits)

PATH_ERROR_RAD = 1e-8
MAX_POINTS = 20000
_CACHE = OrderedDict()
_CACHE_LOCK = Lock()


def _evaluate(coeffs, locations):
    result = np.zeros((*locations.shape, coeffs.shape[-1]))
    for k in range(coeffs.shape[1]-1, -1, -1):
        result = result*locations[..., None]+coeffs[:, k, None, :]
    return result


def optimize_rates(trajectory, vmax, amax, jerk, uniform, check, rate_ceiling=None, *, initial_rates=None):
    points = trajectory.points
    dt = np.diff([time_seconds(p) for p in points])
    c = np.array([coefficients(a, b, t) for a, b, t in zip(points, points[1:], dt)])
    derivatives = [c]
    for order in range(1, 4):
        previous = derivatives[-1]
        derivatives.append(previous[:, 1:]*np.arange(1, previous.shape[1])[None, :, None])
    limits = [np.array([bounds[n] for n in trajectory.joint_names]) for bounds in (vmax, amax, jerk)]
    u = np.linspace(0., 1., 17)[None, :]

    ratio_cache=OrderedDict()
    def ratios(left, right, lo=0, hi=None):
        check()
        key=(lo,hi,left.tobytes(),right.tobytes())
        if key in ratio_cache:
            ratio_cache.move_to_end(key)
            return ratio_cache[key]
        value=measure_ratios(left,right,lo,hi)
        ratio_cache[key]=value
        if len(ratio_cache)>3:ratio_cache.popitem(last=False)
        return value

    def measure_ratios(left, right,lo,hi):
        spans=dt[lo:hi]
        duration = 2*spans/(left+right)
        k = 2*left/(left+right)
        z = k[:, None]*u+2*(1-k[:, None])*(u**3-.5*u**4)
        rate = left[:, None]+(right-left)[:, None]*(3*u*u-2*u**3)
        rate1 = (right-left)[:, None]*6*u*(1-u)/duration[:, None]
        rate2 = (right-left)[:, None]*6*(1-2*u)/duration[:, None]**2
        q1, q2, q3 = [_evaluate(derivatives[i][lo:hi], z)/spans[:, None, None]**i for i in (1, 2, 3)]
        values = (q1*rate[..., None],
                  q2*rate[..., None]**2+q1*rate1[..., None],
                  q3*rate[..., None]**3+3*q2*rate[..., None]*rate1[..., None]+q1*rate2[..., None])
        return np.stack([np.max(np.abs(value)/limit, axis=(1, 2))
                         for value, limit in zip(values, limits)], axis=1)

    # Only neighboring rates affect an interval. A sparse two-column numerical
    # Jacobian avoids one complete path evaluation per optimization variable.
    target = np.array([1/1.005**order for order in (1, 2, 3)])
    count = len(dt)
    best = np.ones(count+1)
    best_cost = 1.
    def objective(x):
        check()
        return float(np.sum(2*dt/(x[:-1]+x[1:]))/np.sum(dt))
    def gradient(x):
        terms = -2*dt/(x[:-1]+x[1:])**2/np.sum(dt)
        return np.r_[terms, 0.]+np.r_[0., terms]
    def constraint(x):
        return (target-ratios(x[:-1]*uniform, x[1:]*uniform)).ravel()
    def jacobian(x):
        left, right = x[:-1]*uniform, x[1:]*uniform
        base = ratios(left, right)
        h = 1e-5*uniform
        dl = -(ratios(left+h, right)-base)/1e-5
        dr = -(ratios(left, right+h)-base)/1e-5
        rows = np.arange(count*3)
        jac = np.zeros((count*3, count+1))
        jac[rows, rows//3] = dl.ravel()
        jac[rows, rows//3+1] = dr.ravel()
        return jac
    def retain(x, interval=None, outside=1.):
        nonlocal best, best_cost
        check()
        if not np.isfinite(x).all() or np.min(x)<=0:
            return
        lo,hi=(0,count) if interval is None else interval
        measured=ratios(x[lo:hi]*uniform,x[lo+1:hi+1]*uniform,lo,hi)/target
        correction=max(1.,outside,float(np.max(measured**(1./np.arange(1,4)))))
        candidate=x/correction
        value = objective(candidate)
        if value < best_cost:
            best, best_cost = candidate.copy(), value
    upper=16. if rate_ceiling is None else rate_ceiling/uniform
    if initial_rates is not None:
        seed=np.asarray(initial_rates,dtype=float)/uniform
        if seed.shape!=best.shape or not np.isfinite(seed).all() or np.min(seed)<=0:
            raise ValueError('Invalid initial local rates')
        seed=np.clip(seed,.05,upper)
        retain(seed)
    if count>96:
        # Include both boundary intervals and overlap blocks, so constraints
        # across a block boundary are never omitted or replaced by a stop.
        starts=list(range(0,count+1,32))
        iterations=0;converged=True
        for start in starts+starts[::-1]:
            stop=min(count+1,start+64);lo=max(0,start-1);hi=min(count,stop)
            base=best.copy()
            # All rates outside this block are fixed for the entire subproblem.
            # Their constraint maximum need not be reevaluated at every callback.
            envelope=(ratios(base[:-1]*uniform,base[1:]*uniform)/target)**(1./np.arange(1,4))
            outside=max(1.,float(np.max(envelope[:lo],initial=0.)),float(np.max(envelope[hi:],initial=0.)))
            def expand(x):
                full=base.copy();full[start:stop]=x
                return full
            def local_constraint(x):
                full=expand(x)*uniform
                return (target-ratios(full[lo:hi],full[lo+1:hi+1],lo,hi)).ravel()
            def local_jacobian(x):
                full=expand(x)*uniform
                left,right=full[lo:hi],full[lo+1:hi+1]
                measured=ratios(left,right,lo,hi);h=1e-5*uniform
                dl=-(ratios(left+h,right,lo,hi)-measured)/1e-5
                dr=-(ratios(left,right+h,lo,hi)-measured)/1e-5
                rows=np.arange((hi-lo)*3);columns=rows//3+lo-start
                jac=np.zeros((len(rows),stop-start))
                for col,values in ((columns,dl),(columns+1,dr)):
                    inside=(col>=0)&(col<stop-start)
                    jac[rows[inside],col[inside]]=values.ravel()[inside]
                return jac
            result=minimize(lambda x:objective(expand(x)),base[start:stop],
                jac=lambda x:gradient(expand(x))[start:stop],method='SLSQP',
                bounds=[(.05,upper)]*(stop-start),
                constraints=[dict(type='ineq',fun=local_constraint,jac=local_jacobian)],
                callback=lambda x:retain(expand(x),(lo,hi),outside),options=dict(maxiter=30,ftol=1e-8))
            retain(expand(result.x),(lo,hi),outside);iterations+=result.nit;converged &= bool(result.success)
        solver='scipy_overlapping_slsqp_time_warp_v1'
    else:
        result = minimize(objective, best, jac=gradient, method='SLSQP',
            bounds=[(.05, upper)]*(count+1),
            constraints=[dict(type='ineq', fun=constraint, jac=jacobian)], callback=retain,
            options=dict(maxiter=60, ftol=1e-7))
        solver='scipy_slsqp_continuous_time_warp_v1'
        retain(result.x)
        iterations=result.nit;converged=bool(result.success)
    return best*uniform, dict(solver=solver,
        iterations=int(iterations), converged=converged,
        search_bound_active=bool(np.max(best)>15.99), sampled_duration_ratio=best_cost)


def _compose(c, warp):
    result = np.zeros((1, c.shape[1]))
    for row in c[::-1]:
        result = np.array([np.convolve(result[:, axis], warp) for axis in range(c.shape[1])]).T
        result[0] += row
    return result


def materialize(trajectory, rates, check):
    """Bound curve error over complete intervals, not just sampled positions."""
    rates=np.asarray(rates,dtype=float)
    if rates.shape!=(len(trajectory.points),) or not np.isfinite(rates).all() or np.min(rates)<=0:
        raise ValueError('Invalid local time-warp rates')
    output = JointTrajectory(joint_names=list(trajectory.joint_names))
    elapsed = 0.
    error_bound = 0.
    source_points = trajectory.points
    for index, (a, b) in enumerate(zip(source_points, source_points[1:])):
        check()
        old_dt = time_seconds(b)-time_seconds(a)
        left, right = rates[index:index+2]
        duration = 2*old_dt/(left+right)
        k = 2*left/(left+right)
        warp = np.array([0., k, 0., 2*(1-k), -(1-k)])
        c = coefficients(a, b, old_dt)

        def point(u):
            z = np.polynomial.polynomial.polyval(u, warp)
            rate = left+(right-left)*(3*u*u-2*u**3)
            rate1 = (right-left)*6*u*(1-u)/duration
            q = np.polynomial.polynomial.polyval(z, c)
            q1 = np.polynomial.polynomial.polyval(z, np.polynomial.polynomial.polyder(c))/old_dt
            q2 = np.polynomial.polynomial.polyval(z, np.polynomial.polynomial.polyder(c, 2))/old_dt**2
            p = JointTrajectoryPoint(positions=q.tolist(), velocities=(q1*rate).tolist(),
                                    accelerations=(q2*rate*rate+q1*rate1).tolist())
            if u in (0., 1.):
                original = a if u == 0. else b
                p.positions = list(original.positions)
                p.velocities = [float(v*rate) for v in original.velocities]
                p.accelerations = [float(v*rate*rate) for v in original.accelerations]
            ns = round((elapsed+u*duration)*1e9)
            p.time_from_start.sec, p.time_from_start.nanosec = divmod(ns, 1000000000)
            return p

        def append(lo, hi, start, end, depth=0):
            nonlocal error_bound
            check()
            dt = time_seconds(end)-time_seconds(start)
            if dt <= 0:
                raise ValueError('Local timing collapses trajectory timestamps')
            local_warp = np.array([np.polynomial.polynomial.polyval(lo,
                np.polynomial.polynomial.polyder(warp, order))*(hi-lo)**order/math.factorial(order)
                for order in range(5)])
            exact = _compose(c, local_warp)
            difference = exact.copy()
            difference[:6] -= coefficients(start, end, dt)
            error = float(np.max(np.abs(bernstein_controls(difference))))
            if not math.isfinite(error):
                raise ValueError('Nonfinite local path error')
            if error > PATH_ERROR_RAD:
                if depth >= 12:
                    raise ValueError('Cannot certify local path preservation')
                middle = (lo+hi)*.5
                mid = point(middle)
                append(lo, middle, start, mid, depth+1)
                append(middle, hi, mid, end, depth+1)
                return
            error_bound = max(error_bound, error)
            output.points.append(end)
            if len(output.points) > MAX_POINTS:
                raise ValueError('Local timing exceeds gateway point budget')

        start, end = point(0.), point(1.)
        if index == 0:
            output.points.append(start)
        append(0., 1., output.points[-1], end)
        elapsed += duration
    return output, error_bound


def fastest_trajectory(trajectory, vmax, amax, jerk, uniform, check=lambda: None, *, rate_ceiling=None):
    check()
    if rate_ceiling is not None:
        if not math.isfinite(rate_ceiling) or rate_ceiling<=0:raise ValueError('Invalid local rate ceiling')
        uniform=min(uniform,rate_ceiling)
    identity=hashlib.sha256(repr((trajectory.joint_names,uniform,rate_ceiling,
        [[bounds[n] for n in trajectory.joint_names] for bounds in (vmax,amax,jerk)])).encode())
    identity.update(np.asarray([[p.time_from_start.sec,p.time_from_start.nanosec,
        *p.positions,*p.velocities,*p.accelerations] for p in trajectory.points],dtype=np.float64).tobytes())
    key=identity.digest()
    with _CACHE_LOCK:
        if key in _CACHE:
            _CACHE.move_to_end(key)
            return copy.deepcopy(_CACHE[key])
    rates, report = optimize_rates(trajectory, vmax, amax, jerk, uniform, check,rate_ceiling)
    result, error = materialize(trajectory, rates, check)
    # Dense optimization is only a proposal. Exact polynomial derivative extrema
    # and the gateway's waist enclosure determine the final executable timing.
    before=copy.deepcopy(result)
    correction = float(stretch_to_limits(result, vmax, amax, jerk))
    rounding_error=0.
    for a,b,c,d in zip(before.points,before.points[1:],result.points,result.points[1:]):
        difference=coefficients(a,b,time_seconds(b)-time_seconds(a))-coefficients(c,d,time_seconds(d)-time_seconds(c))
        rounding_error=max(rounding_error,float(np.max(np.abs(bernstein_controls(difference)))))
    error+=rounding_error
    if error>1e-7:raise ValueError('Final timestamp rounding exceeds path error bound')
    report.update(max_joint_path_difference_rad=error, final_limit_stretch=correction,
                  rate_min=float(np.min(rates)/correction), rate_max=float(np.max(rates)/correction),
                  point_count=len(result.points))
    with _CACHE_LOCK:
        _CACHE[key]=copy.deepcopy((result,report))
        if len(_CACHE)>16:_CACHE.popitem(last=False)
    return result, report
