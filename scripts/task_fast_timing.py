"""Path-preserving local rate search, with continuous execution-limit checks."""
import copy
import numpy as np

from task_trajectory import coefficients,time_seconds,task_jerk_limits,validate_jerk_limits,validate_waist_envelope,peak
from task_trajectory_speed import motion_limits,rescale,speed_capacity,scale_arm_segments


def local_candidate(source,model,parameters,velocity,acceleration,check=lambda:None,*,jerk_limits=None):
    from task_cruise_timing import compose_batch,clock_limit_factors,execution_limit_factors
    from task_local_timing import materialize,optimize_rates
    times=np.array([time_seconds(p) for p in source.points]);dt=np.diff(times)
    polys=np.array([coefficients(a,b,d) for a,b,d in zip(source.points,source.points[1:],dt)])
    vmax,amax=motion_limits(source,model,velocity,acceleration,parameters)
    jerk=task_jerk_limits(source.joint_names) if jerk_limits is None else jerk_limits
    if set(jerk)!=set(source.joint_names) or not all(np.isfinite(v) and v>0 for v in jerk.values()):
        raise ValueError('Invalid experimental jerk limits')
    limits=[np.array([b[n] for n in source.joint_names]) for b in (vmax,amax,jerk)]
    factors=execution_limit_factors(source,limits,check)
    caps=1./np.maximum(factors,1e-8)/1.01
    stationary=np.max(np.abs(polys[:,1:]),axis=(1,2))<1e-12
    caps[stationary]=1.
    rates=np.minimum(np.r_[caps[0],caps],np.r_[caps,caps[-1]])
    polished=False
    best=None
    for iteration in range(160):
        check()
        spans=2*dt/(rates[:-1]+rates[1:]);k=2*rates[:-1]/(rates[:-1]+rates[1:])
        warps=np.column_stack((np.zeros(len(dt)),k,np.zeros(len(dt)),2*(1-k),-(1-k)))
        composed=compose_batch(polys,warps)
        pieces=list(zip(np.r_[0.,np.cumsum(spans)[:-1]],spans,composed))
        factors=clock_limit_factors(pieces,limits)
        if not np.isfinite(factors).all():raise ValueError('局部速率计算非有限数')
        if np.max(factors)<=1./1.005:
            result,error=materialize(source,rates,check)
            exact=execution_limit_factors(result,limits,check)
            if np.max(exact)<=1.:
                if best is None or time_seconds(result.points[-1])<time_seconds(best[0].points[-1]):
                    best=(result,error,rates.copy())
                if polished:break
                polished=True
                uniform=min(caps)
                proposed,_=optimize_rates(source,vmax,amax,jerk,uniform,check,
                    rate_ceiling=float(max(caps)),initial_rates=rates)
                rates=proposed
                continue
            stamps=np.array([time_seconds(p) for p in result.points])
            owner=np.clip(np.searchsorted(np.r_[0.,np.cumsum(spans)],(stamps[:-1]+stamps[1:])/2)-1,0,len(dt)-1)
            np.maximum.at(factors,owner,exact)
        correction=np.maximum(1.,factors*1.015)
        knots=np.maximum(np.r_[correction[0],correction],np.r_[correction,correction[-1]])
        rates/=knots**.8
    if best is None:raise ValueError('局部速率搜索未收敛')
    result,error,rates=best
    validate_jerk_limits(result,jerk)
    if 'waist_yaw_joint' in result.joint_names:
        lo,hi=model.limits['waist_yaw_joint']
        validate_waist_envelope(result,lo+.02,hi-.02,vmax['waist_yaw_joint'],amax['waist_yaw_joint'])
    return result,dict(method='local_rate_limits_v1',iterations=iteration+1,
        max_joint_path_difference_rad=error,rate_min=float(min(rates)),rate_max=float(max(rates)))


def fastest(source,model,parameters=None,*,velocity=.7,acceleration=.7,check=lambda:None):
    """Compare the original policy's verified maximum with a local-rate candidate."""
    source=rescale(source,1.)
    for p in (source.points[0],source.points[-1]):
        if max(abs(v) for v in (*p.velocities,*p.accelerations))>1e-6:
            raise ValueError('整段重定时要求起终点停稳')
    original,old=scale_arm_segments([dict(trajectory=source)],1e6,model,parameters,
        velocity=velocity,acceleration=acceleration,policy='local',check=check)
    result=original[0]['trajectory'];old_seconds=time_seconds(result.points[-1])
    details=dict(method='original_verified_fastest');error=None;candidate_seconds=None
    try:
        candidate,detail=local_candidate(source,model,parameters,velocity,acceleration,check)
        candidate_seconds=time_seconds(candidate.points[-1])
        if candidate_seconds<old_seconds-1e-6:result,details=candidate,detail
    except ValueError as exc:
        check();error=str(exc)
    seconds=time_seconds(result.points[-1])
    return result,dict(method=details['method'],source_seconds=time_seconds(source.points[-1]),
        duration_after_s=seconds,original_fastest_seconds=old_seconds,
        speedup_over_original_fastest=old_seconds/seconds,candidate_seconds=candidate_seconds,
        local_error=error,details=details,velocity_scaling=velocity,acceleration_scaling=acceleration,
        globally_optimal=False,hardware_commands_sent=0)
