"""Fixed-workcell D-ring demonstration bundles and measured-time joint playback."""
import copy
import math
from types import SimpleNamespace

import numpy as np
from scipy.interpolate import BSpline
from scipy.sparse.linalg import lsqr
from trajectory_msgs.msg import JointTrajectory,JointTrajectoryPoint

from task_hand_control import checked_q,model_joints
from task_path_smoothing import joint_line_trajectory
from task_rail_trace import MAX_GAP,MAX_SECONDS,MAX_SAMPLES
from task_session import ARMS
from task_trajectory import coefficients,time_seconds,bernstein_controls,peak
from task_trajectory_speed import scale_arm_segments
from task_saved_transition import plan_transition

SCHEMA='g1_dring_demonstration_v1'
SLOTS={key:label+f' {index}' for direction,label in (('left','左拨'),('right','右拨'))
       for index in range(1,6) for key in (direction if index==1 else f'{direction}_{index}',)}
FIT_RAD=.005
FULL_RATE_FIT_RAD=.001
FULL_RATE_MAX_SAMPLES=600000
HAND_RATE=1.


def moving(side):
    if side not in ARMS:raise ValueError('Unknown demonstration arm')
    return ARMS[side]+('waist_yaw_joint',)


def joint_array(model,names,values):
    rows=np.asarray(values,dtype=float)
    if rows.ndim!=2 or rows.shape[1]!=len(names) or not len(rows) or not np.isfinite(rows).all():
        raise ValueError('录制关节数组无效')
    low=np.array([model.limits[n][0]+.02 for n in names])
    high=np.array([model.limits[n][1]-.02 for n in names])
    if np.any(rows<low) or np.any(rows>high):raise ValueError('录制关节超过软件限位')
    return rows


def arrays(trace,model,side):
    raw=trace['raw'];names=moving(side)
    q=joint_array(model,names,raw['positions']);t=np.asarray(raw['time_s'],dtype=float)
    stamps=raw['stamp_ns']
    limit=FULL_RATE_MAX_SAMPLES if raw.get('sample_policy') in ('all_received','offline_edited') else MAX_SAMPLES
    if (not 3<=len(q)<=limit or t.shape!=(len(q),) or len(stamps)!=len(q) or
            not np.isfinite(t).all() or abs(t[0])>1e-9 or not .4<=t[-1]<=MAX_SECONDS or
            any(type(s) is not int or s<=0 for s in stamps)):
        raise ValueError('录制时间或样本数量无效')
    dt=np.diff(t);source_dt=np.diff(np.array(stamps,dtype=np.int64))*1e-9
    if min(np.min(dt),np.min(source_dt))<=0 or max(np.max(dt),np.max(source_dt))>MAX_GAP+1e-8:
        raise ValueError('录制反馈不连续，原始文件保留，未覆盖旧轨迹')
    if np.max(np.abs(np.diff(q,axis=0)))>.04:raise ValueError('录制关节反馈跳变超过 0.04 rad')
    hand=trace.get('hand')
    if hand is not None:
        ht=np.asarray(hand['time_s'],dtype=float);hq=np.array([checked_q(row) for row in hand['q']])
        if (len(ht)<2 or hq.shape!=(len(ht),6) or not np.isfinite(ht).all() or
                np.any(np.diff(ht)<=0) or ht[0]>0 or ht[-1]<t[-1] or np.max(np.diff(ht))>.75):
            raise ValueError('手指记录没有完整覆盖手臂时间线')
    return q,t


def fit_trace(trace,model,side,check=lambda:None):
    """Fit measured time, preserving dwells; do not impose the rail's 5 mm/s cap."""
    q,t=arrays(trace,model,side);names=moving(side);duration=float(t[-1]);x=t/duration
    if trace.get('optimization',{}).get('kind')=='fast_timing':
        from task_trace_timing import stored_trajectory
        check()
        return stored_trajectory(trace,model,names)
    if trace.get('optimization',{}).get('whole_trace') and trace['optimization'].get('spline'):
        from task_trace_curve import trajectory
        check()
        return trajectory(trace,model,names)
    full_rate=trace['raw'].get('sample_policy') in ('all_received','offline_candidate','offline_edited')
    tolerance=FULL_RATE_FIT_RAD if full_rate else FIT_RAD
    if np.max(np.ptp(q,axis=0))<.002 and trace.get('hand') is None:
        raise ValueError('没有足够的手臂或手指动作，旧轨迹保留')
    selected=None
    counts=(6,10,16,24,36,54,80,120,180,260,380)+( (540,760,1080) if full_rate else () )
    for count in counts:
        check()
        if count>len(q)+4:break
        knots=np.r_[np.zeros(6),np.linspace(0.,1.,count-4)[1:-1],np.ones(6)]
        basis=BSpline.design_matrix(x,knots,5) if full_rate else BSpline(knots,np.eye(count),5)(x)
        controls=np.zeros((count,len(names)));controls[:3]=q[0];controls[-3:]=q[-1]
        if count>6:
            residual=q-basis@controls
            if full_rate:
                solutions=[]
                for axis in range(len(names)):
                    check()
                    result=lsqr(basis[:,3:-3],residual[:,axis],atol=1e-10,btol=1e-10,iter_lim=4*count)
                    if result[1] not in (0,1,2):break
                    solutions.append(result[0])
                if len(solutions)!=len(names):continue
                solution=np.column_stack(solutions)
            else:
                solution,_,rank,_=np.linalg.lstsq(basis[:,3:-3],residual,rcond=1e-8)
                if rank<count-6:continue
            controls[3:-3]=solution
        spline=BSpline(knots,controls,5)
        if np.max(np.abs(spline(x)-q))>tolerance:continue
        # Certify the fitted curve against the measured piecewise-linear trace,
        # including between samples, where least-squares residuals are insufficient.
        breaks=np.unique(np.r_[x,knots]);error=0.
        for first in range(0,len(breaks)-1,4096):
            check()
            block=breaks[first:min(first+4097,len(breaks))];lo=block[:-1];width=np.diff(block)
            idx=np.clip(np.searchsorted(x,lo,side='right')-1,0,len(x)-2)
            slope=(q[idx+1]-q[idx])/(x[idx+1]-x[idx])[:,None]
            diff=np.array([spline(lo,nu=k)*width[:,None]**k/math.factorial(k) for k in range(6)])
            diff[0]-=q[idx]+(lo-x[idx])[:,None]*slope;diff[1]-=width[:,None]*slope
            error=max(error,float(np.max(np.abs(bernstein_controls(diff.reshape(6,-1))))))
            if error>tolerance:break
        if error>tolerance:continue
        selected=(spline,error,count);break
    if selected is None:raise ValueError(f'录制抖动过大，不能在 {tolerance:g} rad 偏差内平滑；原始数据已保留')
    spline,error,count=selected
    # Short exact quintic pieces tighten the same waist envelope used by the gateway.
    # Subdivide each spline span, rather than unioning two independent grids:
    # near-coincident grid/knots amplify timestamp rounding into artificial jerk.
    breaks=np.unique(spline.t)
    times=np.r_[np.concatenate([np.linspace(a,b,max(1,math.ceil((b-a)*duration/.1))+1)[:-1]
                               for a,b in zip(breaks,breaks[1:])]),breaks[-1]]
    tr=JointTrajectory(joint_names=list(names))
    for u in times:
        ns=round(float(u*duration)*1e9)
        u=(ns*1e-9)/duration
        p=JointTrajectoryPoint(positions=spline(u).tolist(),velocities=(spline(u,1)/duration).tolist(),
                               accelerations=(spline(u,2)/duration**2).tolist())
        p.time_from_start.sec,p.time_from_start.nanosec=divmod(ns,1000000000)
        if tr.points and time_seconds(p)<=time_seconds(tr.points[-1]):continue
        tr.points.append(p)
    tr.points[0].positions=q[0].tolist();tr.points[-1].positions=q[-1].tolist()
    for p in (tr.points[0],tr.points[-1]):p.velocities=[0.]*len(names);p.accelerations=[0.]*len(names)
    for a,b in zip(tr.points,tr.points[1:]):
        check();c=coefficients(a,b,time_seconds(b)-time_seconds(a))
        for axis,name in enumerate(names):
            values=c[:,axis];roots=np.polynomial.polynomial.polyroots(np.polynomial.polynomial.polyder(values))
            probes=[0.,1.]+[r.real for r in roots if abs(r.imag)<1e-8 and 0<r.real<1]
            positions=np.polynomial.polynomial.polyval(probes,values)
            lower,upper=model.limits[name]
            if min(positions)<lower+.02 or max(positions)>upper-.02:
                raise ValueError('平滑插值超过关节限位')
    return tr,dict(recorded_seconds=duration,raw_count=len(q),point_count=len(tr.points),
                   fit_error_rad=error,fit_tolerance_rad=tolerance,control_points=count,
                   sample_policy=trace['raw'].get('sample_policy','measured_100hz'),
                   stored_hz=(len(q)-1)/duration,
                   source='offline_candidate' if trace.get('optimization') else 'measured_joint_feedback')


def hand_timeline(trace,factor):
    hand=trace.get('hand')
    if hand is None:return None
    duration=trace['raw']['time_s'][-1]
    ht=np.asarray(hand['time_s']);hq=np.asarray(hand['q'])
    t=np.unique(np.r_[0.,ht[(ht>0)&(ht<duration)],duration])
    q=np.column_stack([np.interp(t,ht,hq[:,i]) for i in range(6)])
    return dict(time_s=(t/factor).tolist(),q=q.tolist())


def build(model,config,start,operation,parameters,velocity,acceleration,multiplier,check=lambda:None,
          *,preparation=None,full_cycle=False,cache_dir=None,progress=lambda text:None,slots=None,
          timing_mode='original'):
    if timing_mode not in ('original','cruise'):raise ValueError('Unknown generation timing mode')
    slots=SLOTS if slots is None else slots
    side=config['side'];names=moving(side);q=dict(start);segments=[]
    ready=config['ready']
    if ready is None and operation!='preparation':raise ValueError('请先记录准备位姿')
    if full_cycle or operation=='preparation':
        if preparation is None:raise ValueError('当前手臂缺少 above-table preparation')
        joint_array(model,names,[[preparation[n] for n in names]])
    def align(target,label=None,role='saved_start_transition'):
        nonlocal q
        prefix=plan_transition(SimpleNamespace(waist_assist=True),model,q,target,side,
            SimpleNamespace(velocity=velocity,acceleration=acceleration),parameters,check,tolerance=1e-6)
        segments.extend(prefix)
        for s in prefix:
            s['role']=role
            if label:s['label']=label
        if prefix:q=dict(prefix[-1]['end'])
    def add_hand(target):
        nonlocal q
        if target is None:return
        target=checked_q(target);end=dict(q,**model_joints(side,target))
        if any(n not in q for n in model_joints(side,target)):raise ValueError('当前手指反馈缺失')
        if max(abs(end[n]-q[n]) for n in model_joints(side,target))<.02:return
        hand_names=list(model_joints(side,target))
        tr=joint_line_trajectory(hand_names,[q[n] for n in hand_names],[end[n] for n in hand_names])
        segments.append(dict(kind='hand',side=side,label='录制起点手指',trajectory=tr,duration=1.,
                             start=dict(q),end=end,target_q=list(target)))
        q=end
    if operation in ('ready','preparation'):
        target=ready['joints'] if operation=='ready' else preparation
        align(dict(q,**{n:target[n] for n in names}),
              '移动到录入准备点' if operation=='ready' else '移动到桌上方准备位')
        report=dict(requested_multiplier=1.,applied_multiplier=1.,limited=False,mode='guarded')
        if operation=='ready':add_hand(ready.get('hand_q'))
        if not segments:raise ValueError('已经处于记录的准备位姿')
    else:
        trace=config['traces'][operation]
        def compute():
            progress('拟合录制关节轨迹')
            original,fit=fit_trace(trace,model,side,check)
            if timing_mode=='cruise':
                from task_cruise_timing import retime
                from task_trace_markers import hand_context
                progress('计算食指弧长与整段近匀速时钟')
                context=hand_context(trace,side,dict(q,**trace.get('context_joints',{})),config,operation)
                timed,report=retime(original,model,context,side,multiplier,parameters,
                    velocity=velocity,acceleration=acceleration,check=check,
                    original_policy=trace.get('optimization',{}).get('timing_policy','recorded'))
                return timed,fit,report
            progress(f'局部调速：{len(original.points)} 个轨迹点')
            scaled,report=scale_arm_segments([dict(label=slots[operation],trajectory=original)],multiplier,
                model,parameters,velocity=velocity,acceleration=acceleration,
                policy=trace.get('optimization',{}).get('timing_policy','recorded'),check=check)
            return scaled[0]['trajectory'],fit,report
        if trace.get('hand') is None:
            if cache_dir is not None:
                from task_dring_cache import cache_key,resolve
                progress('读取已验证录制段缓存')
                key=cache_key(trace,model,side,multiplier,velocity,acceleration,parameters,timing_mode=timing_mode,
                              hand_context=config.get('hands'))
                original,fit,cached_report=resolve(cache_dir,key,compute,check)
            else:original,fit,cached_report=compute()
        else:
            if timing_mode=='cruise':raise ValueError('同步手指录制暂不支持近匀速；请使用原时序')
            original,fit=fit_trace(trace,model,side,check)
        progress('生成起点衔接和回位')
        if full_cycle:
            align(dict(q,**{n:preparation[n] for n in names}),'前往桌上方准备位','demo_preparation')
            align(dict(q,**{n:ready['joints'][n] for n in names}),'前往录入准备点','demo_ready')
        align(dict(q,**dict(zip(names,original.points[0].positions))))
        hand=hand_timeline(trace,1.)
        factor=multiplier;hand_cap=None
        if hand is not None:
            rates=np.abs(np.diff(np.asarray(hand['q']),axis=0))/np.diff(hand['time_s'])[:,None]
            if np.max(rates)>1e-9:
                hand_cap=HAND_RATE/float(np.max(rates));factor=min(factor,hand_cap)
        if hand is None:
            scaled=[dict(trajectory=original)];report=cached_report
        else:
            scaled,report=scale_arm_segments([dict(label=slots[operation],trajectory=original)],factor,model,parameters,
                velocity=velocity,acceleration=acceleration,policy='uniform',check=check)
        report['requested_multiplier']=multiplier
        report['limited']=report['applied_multiplier']<multiplier-1e-6
        if hand_cap is not None:
            report['maximum_multiplier']=min(report.get('maximum_multiplier') or hand_cap,hand_cap)
            report['hand_rate_limit_normalized_s']=HAND_RATE
        hand=hand_timeline(trace,report['applied_multiplier'])
        if hand is not None:add_hand(hand['q'][0])
        tr=scaled[0]['trajectory'];end=dict(q,**dict(zip(names,tr.points[-1].positions)))
        if hand is not None:end.update(model_joints(side,hand['q'][-1]))
        segments.append(dict(kind='arm',side=side,label=slots[operation]+'录制回放',trajectory=tr,
            duration=time_seconds(tr.points[-1]),start=dict(q),end=end,recorded_hand=hand,recording_fit=fit,
            optimization=trace.get('optimization')))
        q=end
        if full_cycle:
            align(dict(q,**{n:preparation[n] for n in names}),'返回桌上方准备位','demo_return')
    for i,s in enumerate(segments,1):s['number']=i
    if timing_mode=='cruise' and report.get('baseline_trajectories'):
        from rosidl_runtime_py.convert import message_to_ordereddict
        baseline=report['baseline_trajectories'][0]
        report['baseline_trajectories']=[baseline if 'recording_fit' in s else
            message_to_ordereddict(s['trajectory']) for s in segments]
    report['duration_after_s']=sum(s['duration'] for s in segments)
    return dict(kind='dring_demo',arm=side,start=dict(start),segments=segments,waist_assist=True,
                hardware_executable=True,speed_scaling=report,
                demo=dict(operation=operation,full_cycle=full_cycle,collision_checked=False,source='recorded_demonstration',
                    transition_count=sum(s.get('role') in ('saved_start_transition','demo_preparation','demo_ready') for s in segments)))


def preview(plan):
    """Full-state RViz playback on the same recorded arm/finger clock."""
    from task_rail_trace import preview_trajectory
    tr=preview_trajectory(plan['segments'])
    offset=0.;names=tr.joint_names
    for segment in plan['segments']:
        hand=segment.get('recorded_hand')
        if hand:
            for p in tr.points:
                t=time_seconds(p)-offset
                if -.000001<=t<=segment['duration']+.000001:
                    normalized=[float(np.interp(t,hand['time_s'],np.asarray(hand['q'])[:,i])) for i in range(6)]
                    for name,value in model_joints(segment['side'],normalized).items():p.positions[names.index(name)]=value
        offset+=segment['duration']
    return tr
