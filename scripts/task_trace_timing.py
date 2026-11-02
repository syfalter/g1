"""Persistent exact timing candidates shared by all offline editors."""
import math
import copy
import time
import numpy as np

from task_trajectory import coefficients,time_seconds,peak


def evaluate(trajectory, stamps):
    times=np.array([time_seconds(p) for p in trajectory.points])
    stamps=np.asarray(stamps)
    indices=np.clip(np.searchsorted(times,stamps,side='right')-1,0,len(times)-2)
    result=np.empty((len(stamps),len(trajectory.joint_names)))
    for i in np.unique(indices):
        mask=indices==i;dt=times[i+1]-times[i]
        poly=coefficients(trajectory.points[i],trajectory.points[i+1],dt)
        result[mask]=np.polynomial.polynomial.polyval((stamps[mask]-times[i])/dt,poly).T
    return result


def stored_trajectory(trace,model,names):
    from trajectory_msgs.msg import JointTrajectory
    from rosidl_runtime_py.set_message import set_message_fields
    from task_trajectory_speed import rescale
    from task_screen_teach import fingerprint
    info=trace['optimization'];spec=info['timed_trajectory']
    if fingerprint(spec)!=info['timed_sha256']:raise ValueError('保存的时序校验不一致')
    tr=JointTrajectory();set_message_fields(tr,copy.deepcopy(spec));tr=rescale(tr,1.)
    if list(tr.joint_names)!=list(names):raise ValueError('保存的时序关节不一致')
    t=np.asarray(trace['raw']['time_s']);q=np.asarray(trace['raw']['positions'])
    if abs(t[-1]-time_seconds(tr.points[-1]))>1e-9 or np.max(np.abs(evaluate(tr,t)-q))>1e-8:
        raise ValueError('保存的时序与编辑轨迹不一致')
    for p in (tr.points[0],tr.points[-1]):
        if max(abs(v) for v in (*p.velocities,*p.accelerations))>1e-6:
            raise ValueError('保存的整段时序起终点未停稳')
    for a,b in zip(tr.points,tr.points[1:]):
        poly=coefficients(a,b,time_seconds(b)-time_seconds(a))
        for axis,name in enumerate(names):
            lo,hi=model.limits[name];c=poly[:,axis].copy();c[0]-=(lo+hi)/2
            if peak(c)>(hi-lo)/2-.02+1e-9:raise ValueError('保存时序超过关节限位：'+name)
    return tr,dict(source='stored_exact_timing',recorded_seconds=float(t[-1]),
        raw_count=len(t),point_count=len(tr.points),fit_error_rad=0.,sample_policy='offline_edited')


def encode(trace,trajectory,report):
    from task_trace_edit import synthetic
    from task_screen_teach import fingerprint
    from rosidl_runtime_py.convert import message_to_ordereddict
    from task_trajectory import bernstein_controls
    duration=time_seconds(trajectory.points[-1])
    if not .4<=duration<=300:raise ValueError('重定时结果超出轨迹存储时长范围 0.4 至 300 秒')
    stamps=[]
    for a,b in zip(trajectory.points,trajectory.points[1:]):
        lo=time_seconds(a);dt=time_seconds(b)-lo
        poly=coefficients(a,b,dt)
        bound=float(np.max(np.abs(bernstein_controls(np.polynomial.polynomial.polyder(poly,axis=0)))))
        count=max(1,math.ceil(dt/.005),math.ceil(bound/.005))
        if len(stamps)+count>600000:raise ValueError('重定时超过样本数量预算')
        stamps.extend(np.linspace(lo,lo+dt,count+1)[:-1])
    stamps=np.unique(np.rint(np.r_[stamps,duration]*1e9).astype(np.int64))*1e-9
    rows=evaluate(trajectory,stamps)
    spec=message_to_ordereddict(trajectory)
    return synthetic(trace,rows,stamps,'限值内尽快重定时',kind='fast_timing',timing_policy='uniform',
        timed_trajectory=spec,timed_sha256=fingerprint(spec),timing_report=report,
        endpoints_preserved=True,source_sha256=fingerprint(trace))


def _preserving(trace,model,side,parameters,*,workflow,config,slot,velocity,acceleration,check,progress):
    from dataclasses import replace
    from task_dring_teach_data import fit_trace,moving
    from task_fast_timing import fastest
    from task_trajectory_speed import scale_arm_segments
    if isinstance(trace.get('hand'),dict):raise ValueError('同步手指录制暂不支持整段重定时')
    progress('原路径拟合与原策略时序')
    if workflow=='rail':
        from task_rail_trace import replay
        from task_trace_markers import hand_context
        from task_rail_motion import VELOCITY_SCALE,ACCELERATION_SCALE
        if velocity!=VELOCITY_SCALE or acceleration!=ACCELERATION_SCALE:
            raise ValueError('滑轨请使用与执行面板相同的限值比例')
        context=hand_context(trace,side,dict(trace.get('context_joints',{})),config,slot)
        context.update(zip(moving(side),trace['raw']['positions'][0]))
        source,details=replay(model,trace,context,replace(parameters,rail_replay_cruise=False,
            rail_replay_fastest=False,rail_replay_multiplier=1.),check)
        original=source
        original_fast,_=replay(model,trace,context,replace(parameters,rail_replay_cruise=False,
            rail_replay_fastest=True,rail_replay_multiplier=1.),check)
        original_seconds=time_seconds(source.points[-1])
    else:
        source,_=fit_trace(trace,model,side,check)
        original,_=scale_arm_segments([dict(trajectory=source)],1.,model,parameters,
            velocity=velocity,acceleration=acceleration,check=check,
            policy=trace.get('optimization',{}).get('timing_policy','recorded'))
        original_seconds=time_seconds(original[0]['trajectory'].points[-1])
        original=original[0]['trajectory']
    progress('原路径局部重定时搜索')
    result,report=fastest(source,model,parameters,velocity=velocity,acceleration=acceleration,check=check)
    if workflow=='rail':
        report['original_fastest_seconds']=time_seconds(original_fast.points[-1])
        if time_seconds(original_fast.points[-1])<time_seconds(result.points[-1]):
            result=original_fast;report['method']='original_verified_fastest'
    if original_seconds<time_seconds(result.points[-1]):
        result=original;report['method']='original_1x'
    report.update(path_modified=False,preserving_seconds=time_seconds(result.points[-1]),original_1x_seconds=original_seconds)
    return result,report


def generate(trace,model,side,parameters,*,workflow,config,slot,velocity=.7,acceleration=.7,
             denoise=False,tolerance=.003,window=.15,check=lambda:None,progress=lambda text:None,cache_dir=None):
    from task_trace_timing_cache import identity,resolve
    from task_dring_teach_data import moving
    started=time.monotonic()
    check()
    key=identity(trace,model,side,parameters,workflow,config,slot,velocity,acceleration) if cache_dir is not None else None
    options=dict(denoise=bool(denoise),tolerance=tolerance,window=window) if denoise else dict(denoise=False)

    def compute():
        progress('读取或计算原路径时序')
        result,report,hit=resolve(cache_dir,key,'preserving',{},lambda:_preserving(trace,model,side,parameters,
            workflow=workflow,config=config,slot=slot,velocity=velocity,acceleration=acceleration,
            check=check,progress=progress),check)
        report.update(denoise_requested=bool(denoise),preserving_cache_hit=hit)
        if denoise:
            def smoothed_candidate():
                from dataclasses import replace
                from task_trace_edit import smooth
                from task_dring_teach_data import fit_trace
                from task_fast_timing import fastest
                progress('关节去抖与路径拟合')
                smoothed=smooth(trace,tolerance=tolerance,window=window)
                check()
                if workflow=='rail':
                    from task_rail_trace import replay
                    from task_trace_markers import hand_context
                    context=hand_context(trace,side,dict(trace.get('context_joints',{})),config,slot)
                    context.update(zip(moving(side),trace['raw']['positions'][0]))
                    proposed,_=replay(model,smoothed,context,replace(parameters,rail_replay_cruise=False,
                        rail_replay_fastest=False,rail_replay_multiplier=1.),check)
                else:proposed,_=fit_trace(smoothed,model,side,check)
                progress('去抖路径局部重定时搜索')
                proposed,details=fastest(proposed,model,parameters,velocity=velocity,acceleration=acceleration,check=check)
                return proposed,dict(search=details,smoothing=smoothed['optimization'])
            progress('读取或计算去抖时序')
            try:
                proposed,details,hit=resolve(cache_dir,key,'smoothed',options,smoothed_candidate,check)
                report.update(smoothed_seconds=time_seconds(proposed.points[-1]),smoothed_cache_hit=hit)
                if report['smoothed_seconds']<time_seconds(result.points[-1])-1e-6:
                    result=proposed
                    report.update(method='denoised_'+details['search']['method'],path_modified=True,
                        smoothing=details['smoothing'],smoothed_search=details['search'])
            except ValueError as exc:
                check();report['denoise_error']=str(exc)
        seconds=time_seconds(result.points[-1])
        report.update(duration_after_s=seconds,speedup_over_original_fastest=report['original_fastest_seconds']/seconds,
            speedup_over_original_1x=report['original_1x_seconds']/seconds)
        return result,report

    progress('检查重定时结果缓存')
    result,report,hit=resolve(cache_dir,key,'complete',options,compute,check)
    progress('最终连续限值与保存数据检查')
    from task_trajectory_speed import motion_limits
    from task_trajectory import task_jerk_limits
    from task_cruise_timing import execution_limit_factors
    vmax,amax=motion_limits(result,model,velocity,acceleration,parameters)
    limits=[np.array([bounds[n] for n in result.joint_names]) for bounds in
        (vmax,amax,task_jerk_limits(result.joint_names))]
    factors=execution_limit_factors(result,limits,check)
    if not np.isfinite(factors).all() or np.max(factors)>1.+1e-6:
        raise ValueError('重定时结果超过当前连续运动限值')
    report.update(result_cache_hit=hit)
    check()
    edited=encode(trace,result,report)
    if report['path_modified']:edited['optimization']['label']='去抖＋限值内尽快重定时'
    stored_trajectory(edited,model,moving(side))
    check()
    report['computation_seconds']=time.monotonic()-started
    return edited
