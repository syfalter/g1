"""Fingertip arc-clock retiming with certified preservation of the input path."""
import math

import numpy as np
from scipy.interpolate import CubicSpline, PchipInterpolator
from scipy.optimize import brentq
from scipy.spatial.transform import Rotation
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from task_local_timing import _compose, MAX_POINTS, PATH_ERROR_RAD
from task_trajectory import coefficients, time_seconds, bernstein_controls, peak


def progress(t, ramp=.15):
    if t < ramp:
        u = t/ramp
        return ramp*(u**3-.5*u**4)/(1-ramp)
    if t > 1-ramp:
        return 1-progress(1-t, ramp)
    return (t-ramp/2)/(1-ramp)


def minimum(poly):
    roots = np.polynomial.polynomial.polyroots(np.polynomial.polynomial.polyder(poly))
    probes = [0., 1.]+[r.real for r in roots if abs(r.imag) < 1e-8 and 0 < r.real < 1]
    return float(np.min(np.polynomial.polynomial.polyval(probes, poly)))


def arc_clock(trajectory, model, context, side, check, limits=None, *, multiplier=1.):
    times = np.array([time_seconds(p) for p in trajectory.points])
    polys = [coefficients(a, b, dt) for a, b, dt in
             zip(trajectory.points, trajectory.points[1:], np.diff(times))]
    samples, rows, owners = [], [], []
    for i, (poly, dt) in enumerate(zip(polys, np.diff(times))):
        check()
        count = max(4, math.ceil(dt/.1))
        for u in np.linspace(0, 1, count+1)[:-1]:
            samples.append(times[i]+u*dt)
            rows.append(np.polynomial.polynomial.polyval(u, poly))
            owners.append(i)
    samples.append(times[-1]); rows.append(trajectory.points[-1].positions)
    samples, rows = np.asarray(samples), np.asarray(rows)
    if len(samples) > MAX_POINTS:
        raise ValueError('近匀速计算超过轨迹点预算，请缩短或平滑该段')
    link = side+'_index_tcp'
    pose = model.batch_transforms([link], context, trajectory.joint_names, rows)[link]
    check()
    distance = np.linalg.norm(np.diff(pose[:, :3, 3], axis=0), axis=1)
    angles = (Rotation.from_matrix(pose[:-1, :3, :3]).inv()*
              Rotation.from_matrix(pose[1:, :3, :3])).magnitude()
    joint = np.linalg.norm(np.diff(rows, axis=0), axis=1)
    # Translational arc dominates; rotation and joint motion regularize a stationary TCP.
    metric = np.sqrt(distance**2+(.01*angles)**2+(.005*joint)**2)
    total = float(np.sum(metric))
    if total < 1e-10:
        return None, dict(metric='stationary',path_length_m=0.,preserved_stationary=True)
    # Regularize almost stationary intervals; exact constant pieces keep their duration below.
    metric = np.hypot(metric,total*.3*np.diff(samples)/times[-1])
    arc = np.r_[0., np.cumsum(metric)]; arc /= arc[-1]
    target = np.array([0.]+[brentq(lambda t:progress(t)-s,0.,1.,xtol=1e-14)
                            for s in arc[1:-1]]+[1.])*times[-1]/multiplier
    dwell=joint<1e-13
    target=np.r_[0.,np.cumsum(np.where(dwell,np.maximum(np.diff(target),np.diff(samples)),np.diff(target)))]
    nominal=np.diff(target).copy()
    iterations=0
    best=None
    refinements=0
    for iterations in range(80):
        check()
        pieces=clock_pieces(target,samples,times,polys,owners,check)
        if limits is None:break
        factors=clock_limit_factors(pieces,limits)
        if not np.isfinite(factors).all():raise ValueError('局部时序限值非有限数')
        if np.max(factors)<=1.:
            artifact,_=materialize(pieces,trajectory.joint_names,check)
            exact=execution_limit_factors(artifact,limits,check)
            if np.max(exact)>1.:
                stamps=np.array([time_seconds(p) for p in artifact.points])
                owner=np.clip(np.searchsorted(target,(stamps[:-1]+stamps[1:])*.5,side='right')-1,
                              0,len(factors)-1)
                np.maximum.at(factors,owner,exact)
        if np.max(factors)<=1.:
            if best is None or target[-1]<best[0][-1]:best=(target.copy(),pieces)
            # Reclaim conservative excess locally; never shrink a required stop.
            spans=np.diff(target)
            shorter=np.maximum(nominal,spans*np.clip(factors*1.04,.9,1.))
            refinements+=1
            if refinements>=4 or np.sum(spans-shorter)<.002*target[-1]:break
            target=np.r_[0.,np.cumsum(smooth_delays(shorter,nominal,samples))]
            continue
        # Only offending intervals grow. Shared C2 knot derivatives automatically
        # create acceleration/deceleration shoulders in adjacent intervals.
        correction=np.where(factors>1.,np.minimum(8.,(factors*1.02)**.8),1.)
        target=np.r_[0.,np.cumsum(smooth_delays(np.diff(target)*correction,nominal,samples))]
    else:
        if best is None:raise ValueError('局部限速未收敛；未生成轨迹，不回退到整段限速')
    if best is not None:target,pieces=best
    return pieces, dict(metric='index_translation_rotation_joint_arc_v1',
        path_length_m=float(np.sum(distance)),rotation_weight_m=.01,joint_weight_m=.005,
        stationary_clock_regularization=.3,
        ramp_fraction=.15,clock_samples=len(samples),original_seconds=float(times[-1]),
        stationary_intervals=int(np.count_nonzero(dwell)),local_limit_iterations=iterations+1,
        requested_duration_s=float(np.sum(nominal)),
        locally_limited_intervals=int(np.count_nonzero(np.diff(target)>nominal*1.001)),
        uniform_limit_stretch=1.)


def smooth_delays(spans,nominal,samples):
    """Finite local shoulders, projected from total dilation, not compounded per pass."""
    log=np.log(np.maximum(1.,spans/nominal))
    mid=(samples[1:]+samples[:-1])*.5
    decay=1.*np.diff(mid)
    for i in range(1,len(log)):log[i]=max(log[i],log[i-1]-decay[i-1])
    for i in range(len(log)-2,-1,-1):log[i]=max(log[i],log[i+1]-decay[i])
    return nominal*np.exp(log)


def execution_limit_factors(trajectory,limits,check):
    factors=[]
    for a,b in zip(trajectory.points,trajectory.points[1:]):
        check()
        dt=time_seconds(b)-time_seconds(a)
        polynomial=coefficients(a,b,dt)
        factor=0.
        for order,bounds in enumerate(limits,1):
            derivative=np.polynomial.polynomial.polyder(polynomial,order)/dt**order
            envelope=np.max(np.abs(bernstein_controls(derivative)),axis=0)
            for axis,name in enumerate(trajectory.joint_names):
                ratio=envelope[axis]/bounds[axis]
                if not (name=='waist_yaw_joint' and order<3) and ratio>1.:
                    ratio=peak(derivative[:,axis])/bounds[axis]
                factor=max(factor,float(ratio)**(1/order))
        factors.append(factor)
    return np.asarray(factors)


def clock_limit_factors(pieces,limits):
    """Whole-interval enclosures, evaluated per clock piece, never a global factor."""
    spans=np.asarray([p[1] for p in pieces])
    polynomial=np.asarray([p[2] for p in pieces]).transpose(1,0,2)
    factors=np.zeros(len(pieces))
    for order,bounds in enumerate(limits,1):
        derivative=np.polynomial.polynomial.polyder(polynomial,order,axis=0)
        shape=derivative.shape
        controls=bernstein_controls(derivative.reshape(shape[0],-1)).reshape(shape)
        # Subdivision tightens high-degree composition bounds without sampling gaps.
        controls=controls[:,None,:,:]
        for _ in range(2):
            work=controls.copy();left=[work[0]];right=[work[-1]]
            for k in range(1,len(work)):
                work=(work[:-1]+work[1:])*.5
                left.append(work[0]);right.append(work[-1])
            controls=np.concatenate((np.asarray(left),np.asarray(right[::-1])),axis=1)
        envelope=np.max(np.abs(controls),axis=(0,1))/spans[:,None]**order
        factors=np.maximum(factors,np.max(envelope/bounds,axis=1)**(1/order))
    return factors


def clock_pieces(target,samples,times,polys,owners,check):
    clock = CubicSpline(target, samples, bc_type='natural')
    spans=np.diff(target);secants=np.diff(samples)/spans
    speeds=clock(target,1)
    fallback=PchipInterpolator(target,samples)(target,1)
    ceilings=1.2*np.minimum(np.r_[secants[0],secants],np.r_[secants,secants[-1]])
    safe_speeds=np.minimum(np.maximum(np.where(speeds>0,speeds,fallback),1e-12),ceilings)
    accels=clock(target,2)
    # Nonnegative Bernstein controls certify monotonicity of each quintic clock.
    # Shared endpoint v/a keep the clock C2 even next to captured pauses.
    slack=5*secants-2*safe_speeds[:-1]-2*safe_speeds[1:]
    caps=np.full(len(samples),np.inf)
    caps[:-1]=np.minimum(caps[:-1],np.minimum(2*safe_speeds[:-1],2*slack)/spans)
    caps[1:]=np.minimum(caps[1:],np.minimum(2*safe_speeds[1:],2*slack)/spans)
    safe_accels=np.clip(accels,-caps,caps)
    owners=np.asarray(owners)
    old_dt=np.diff(times)[owners]
    lo=(samples[:-1]-times[owners])/old_dt
    hi=(samples[1:]-times[owners])/old_dt
    clamped=np.zeros(len(samples),dtype=bool)
    while True:
        check()
        v=np.where(clamped,safe_speeds,speeds)
        a=np.where(clamped,safe_accels,accels)
        warp=np.zeros((6,len(owners)))
        warp[0]=lo;warp[1]=v[:-1]*spans/old_dt
        warp[2]=a[:-1]*spans**2/(2*old_dt)
        delta=hi-lo-warp[1]-warp[2]
        dv=v[1:]*spans/old_dt-warp[1]-2*warp[2]
        da=a[1:]*spans**2/old_dt-2*warp[2]
        warp[3]=10*delta-4*dv+.5*da
        warp[4]=-15*delta+7*dv-da
        warp[5]=6*delta-3*dv+.5*da
        derivative=np.polynomial.polynomial.polyder(warp,axis=0)
        suspect=np.flatnonzero(np.min(bernstein_controls(derivative),axis=0)<-1e-10)
        bad=[k for k in suspect if minimum(derivative[:,k])<-1e-10]
        if not bad:break
        nodes=np.unique(np.r_[bad,np.asarray(bad)+1])
        if np.all(clamped[nodes]):raise ValueError('不能构造单调局部时钟，原轨迹保持不变')
        clamped[nodes]=True
    composed=compose_batch(np.asarray(polys)[owners],warp.T)
    pieces = []
    for k in range(len(owners)):pieces.append((target[k],spans[k],composed[k]))
    return pieces


def compose_batch(polynomials,warps):
    result=np.zeros((len(polynomials),1,polynomials.shape[-1]))
    for row in polynomials.transpose(1,0,2)[::-1]:
        product=np.zeros((len(result),result.shape[1]+warps.shape[1]-1,result.shape[2]))
        for i in range(warps.shape[1]):
            product[:,i:i+result.shape[1]]+=result*warps[:,i,None,None]
        product[:,0]+=row
        result=product
    return result


def materialize(pieces, names, check):
    result = JointTrajectory(joint_names=list(names))
    bound = 0.
    for offset, span, exact in pieces:
        check()
        def point(u):
            p = JointTrajectoryPoint(positions=np.polynomial.polynomial.polyval(u,exact).tolist(),
                velocities=(np.polynomial.polynomial.polyval(u,
                    np.polynomial.polynomial.polyder(exact))/span).tolist(),
                accelerations=(np.polynomial.polynomial.polyval(u,
                    np.polynomial.polynomial.polyder(exact,2))/span**2).tolist())
            p.time_from_start.sec,p.time_from_start.nanosec=divmod(round((offset+span*u)*1e9),10**9)
            return p
        def append(lo,hi,a,b,depth=0):
            nonlocal bound
            check()
            dt=time_seconds(b)-time_seconds(a)
            if dt<=0:raise ValueError('近匀速时间戳精度不足')
            local=_compose(exact,np.array([lo,hi-lo]))
            local[:6]-=coefficients(a,b,dt)
            error=float(np.max(np.abs(bernstein_controls(local))))
            if not math.isfinite(error):raise ValueError('近匀速路径误差非有限数')
            if error>PATH_ERROR_RAD:
                if depth>=12:raise ValueError('不能保证近匀速路径误差')
                middle=(lo+hi)/2;mid=point(middle)
                append(lo,middle,a,mid,depth+1);append(middle,hi,mid,b,depth+1)
                return
            bound=max(bound,error);result.points.append(b)
            if len(result.points)>MAX_POINTS:raise ValueError('近匀速输出超过执行器轨迹点预算')
        if not result.points:result.points.append(point(0.))
        append(0.,1.,result.points[-1],point(1.))
    return result,bound


def retime(trajectory, model, context, side, multiplier, parameters=None, *,
           velocity=1., acceleration=1., check=lambda:None, original_policy=None):
    if original_policy is not None:
        from task_trajectory_speed import scale_arm_segments
        baseline,baseline_report=scale_arm_segments([dict(trajectory=trajectory)],multiplier,
            model,parameters,velocity=velocity,acceleration=acceleration,check=check,
            policy=original_policy)
        baseline=baseline[0]['trajectory']
        original_seconds=time_seconds(baseline.points[-1])
        error=None
        try:
            candidate,report=_retime(trajectory,model,context,side,multiplier,parameters,
                velocity=velocity,acceleration=acceleration,check=check)
            candidate_seconds=time_seconds(candidate.points[-1])
        except ValueError as exc:
            check()
            error=str(exc);candidate_seconds=None
        fallback=error is not None or candidate_seconds>original_seconds+1e-9
        if fallback:
            candidate,report=baseline,baseline_report
        report.update(generation_mode='cruise',selected_generation_mode='original' if fallback else 'cruise',
            timing_comparison=dict(original_policy=original_policy,original_seconds=original_seconds,
                cruise_seconds=candidate_seconds,fallback=fallback,cruise_error=error))
        return candidate,report
    return _retime(trajectory,model,context,side,multiplier,parameters,
        velocity=velocity,acceleration=acceleration,check=check)


def _retime(trajectory, model, context, side, multiplier, parameters=None, *,
            velocity=1., acceleration=1., check=lambda:None):
    from task_trajectory_speed import rescale, speed_capacity, motion_limits
    from task_trajectory import task_jerk_limits,validate_jerk_limits,validate_waist_envelope
    from rosidl_runtime_py.convert import message_to_ordereddict
    if not math.isfinite(multiplier) or multiplier<=0:raise ValueError('倍率必须为有限正数')
    if any(not math.isfinite(v) or not .05<=v<=1. for v in (velocity,acceleration)):
        raise ValueError('运动限值比例须在 0.05 到 1 之间')
    source=rescale(trajectory,1.)
    check()
    for p in (source.points[0],source.points[-1]):
        if max(abs(v) for v in (*p.velocities,*p.accelerations))>1e-7:
            raise ValueError('近匀速段要求起终点停稳，请先生成完整录制段')
    vmax,amax=motion_limits(source,model,velocity,acceleration,parameters)
    limits=[np.array([d[n] for n in source.joint_names]) for d in
            (vmax,amax,task_jerk_limits(source.joint_names))]
    pieces,report=arc_clock(source,model,context,side,check,limits,multiplier=multiplier)
    result,error=materialize(pieces,source.joint_names,check) if pieces else (source,0.)
    for a,b in zip(result.points,result.points[1:]):
        check();poly=coefficients(a,b,time_seconds(b)-time_seconds(a))
        for axis,name in enumerate(result.joint_names):
            lo,hi=model.limits[name];center=(lo+hi)/2
            c=poly[:,axis].copy();c[0]-=center
            if peak(c)>(hi-lo)/2-.02+1e-9:raise ValueError('近匀速曲线超出关节限位：'+name)
    # This is a read-only final gate, not a global correction or multiplier clip.
    capacity=speed_capacity(result,model,parameters,velocity=velocity,acceleration=acceleration,check=check)
    maximum=capacity['maximum_multiplier']
    if maximum is not None and maximum*1.005<1.-1e-6:
        raise ValueError('最终插值区间仍超限；未生成轨迹，不回退到整段限速')
    validate_jerk_limits(result,task_jerk_limits(result.joint_names))
    if 'waist_yaw_joint' in result.joint_names:
        lo,hi=model.limits['waist_yaw_joint']
        validate_waist_envelope(result,lo+.02,hi-.02,vmax['waist_yaw_joint'],amax['waist_yaw_joint'])
    duration=time_seconds(result.points[-1]);original=time_seconds(source.points[-1])
    speed=dict(mode='guarded',timing_policy='cruise_local_v2',limits_enforced=True,
        requested_multiplier=multiplier,applied_multiplier=original/duration,
        multiplier_semantics='requested_cruise_rate_with_local_limit_slowdowns',
        limited=duration>original/multiplier+1e-6,maximum_multiplier=None,
        limiting_constraint=capacity['limiting_constraint'],hand_timing_preserved=True,
        duration_before_s=original,duration_after_s=duration,source_seconds=original,
        velocity_limit_scaling=velocity,acceleration_limit_scaling=acceleration,
        baseline_trajectories=[message_to_ordereddict(source)],hardware_commands_sent=0)
    stats=cruise_statistics(result,model,context,side,check)
    speed.update(generation_mode='cruise',cruise=dict(report,
        baseline_seconds=duration,baseline_limit_stretch=1.,
        max_joint_path_difference_rad=error,point_count=len(result.points),**stats),
        source_seconds=time_seconds(source.points[-1]))
    return result,speed


def cruise_statistics(trajectory,model,context,side,check):
    times=np.array([time_seconds(p) for p in trajectory.points])
    probes=np.linspace(.2,.8,401)*times[-1]
    rows=[]
    for t in probes:
        check()
        i=min(len(times)-2,np.searchsorted(times,t,side='right')-1)
        dt=times[i+1]-times[i]
        poly=coefficients(trajectory.points[i],trajectory.points[i+1],dt)
        rows.append(np.polynomial.polynomial.polyval((t-times[i])/dt,poly))
    link=side+'_index_tcp'
    pose=model.batch_transforms([link],context,trajectory.joint_names,np.asarray(rows))[link]
    speed=np.linalg.norm(np.diff(pose[:,:3,3],axis=0),axis=1)/np.diff(probes)
    return dict(cruise_speed_p10_m_s=float(np.percentile(speed,10)),
        cruise_speed_p90_m_s=float(np.percentile(speed,90)),
        cruise_speed_cv=float(np.std(speed)/np.mean(speed)) if np.mean(speed)>1e-10 else None,
        cruise_statistics_window='middle_60_percent_of_final_time',
        cruise_statistics_sampled=True)


def summary(report):
    comparison=report.get('timing_comparison',{})
    if comparison.get('fallback'):
        reason='未生成可行结果' if comparison.get('cruise_error') else '未优于原策略'
        return (f'近匀速{reason}，采用原策略时序；请求 {report["requested_multiplier"]:g}x；'
                f'{report["duration_after_s"]:.2f} s')
    limit=report.get('limiting_constraint') or {}
    text=(f'近匀速局部限速：请求 {report["requested_multiplier"]:g}x / 平均实际 '
          f'{report["applied_multiplier"]:.3g}x；{report["duration_after_s"]:.2f} s')
    if limit:text+=f'；限制：{limit["joint"]} / {limit["constraint"]}'
    info=report.get('cruise',{})
    if 'cruise_speed_p10_m_s' in info:
        text+=f'；中段速度 P10/P90：{info["cruise_speed_p10_m_s"]*1000:.1f}/{info["cruise_speed_p90_m_s"]*1000:.1f} mm/s'
    if (info.get('cruise_speed_cv') or 0)>.25:
        text+='；局部变化仍明显，可先去抖或平滑折返'
    return text
