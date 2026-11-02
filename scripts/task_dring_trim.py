"""Non-destructive recorded-time edits, independent of hardware and planning."""
import copy
import math

import numpy as np
from scipy.interpolate import BSpline, make_interp_spline


def sample(trace,seconds):
    raw=trace['raw'];t=np.asarray(raw['time_s']);q=np.asarray(raw['positions'])
    return np.array([np.interp(seconds,t,q[:,i]) for i in range(q.shape[1])]).T


def boundary_state(trace, seconds):
    """Use analytical derivatives, or a local regression instead of raw differences."""
    t=np.asarray(trace['raw']['time_s'],dtype=float)
    q=np.asarray(trace['raw']['positions'],dtype=float)
    position=sample(trace,seconds)
    if seconds==0 or seconds==t[-1]:
        return position,np.zeros(q.shape[1]),np.zeros(q.shape[1])
    info=trace.get('optimization',{})
    if info.get('spline'):
        lo,hi=info['interval_s']
        if lo<=seconds<=hi:
            spec=info['spline'];spline=BSpline(spec['knots'],spec['controls'],spec['degree'])
            u=(seconds-lo)/(hi-lo)
            return spline(u),spline(u,1)/(hi-lo),spline(u,2)/(hi-lo)**2
    indices=np.flatnonzero(np.abs(t-seconds)<=.1)
    if len(indices)<4:indices=np.sort(np.argsort(np.abs(t-seconds))[:min(6,len(t))])
    x=t[indices]-seconds
    scale=max(float(np.max(np.abs(x))),1e-9)
    degree=min(3,len(indices)-1)
    basis=np.column_stack([(x/scale)**i for i in range(1,degree+1)])
    c=np.linalg.lstsq(basis,q[indices]-position,rcond=None)[0]
    return position,c[0]/scale,2*c[1]/scale**2 if degree>=2 else np.zeros(q.shape[1])


def cut(trace,start,end,bridge_seconds=.6):
    """Replace an interval by a boundary-matched quintic, without forced stops.

    Raw-data boundary derivatives are estimates. The downstream continuous
    fitting and dynamics checks still apply before a new candidate can execute.
    """
    t=np.asarray(trace['raw']['time_s'],dtype=float);duration=float(t[-1])
    if not all(math.isfinite(v) for v in (start,end,bridge_seconds)):
        raise ValueError('裁切时间必须是有限数值')
    if not 0<=start<end<=duration:raise ValueError('时间范围须满足 0 ≤ 起点 < 终点 ≤ 当前时长')
    if bridge_seconds<=0:raise ValueError('衔接时长须为正数')
    if not .4 <= duration-(end-start)+bridge_seconds <= 300:
        raise ValueError('修改后的整段轨迹须满足 0.4 至 300 秒的存储格式要求')
    times=[];rows=[]

    def append(ts,qs):
        for stamp,row in zip(ts,qs):
            if times and stamp<=times[-1]+1e-9:continue
            times.append(float(stamp));rows.append(row.tolist())

    q=np.asarray(trace['raw']['positions'])
    keep=t<start;append(t[keep],q[keep])
    a,va,aa=boundary_state(trace,start);b,vb,ab=boundary_state(trace,end)
    spline=make_interp_spline([0.,bridge_seconds],np.array([a,b]),k=5,
        bc_type=([(1,va),(2,aa)],[(1,vb),(2,ab)]))
    # Bound per-sample displacement, not the requested replay speed.
    rate=float(np.max(np.abs(spline.derivative().c)))
    count=max(2,math.ceil(bridge_seconds/.005)+1,math.ceil(rate*bridge_seconds/.005)+1)
    if count+len(t)+162 > 600000:raise ValueError('修改后的轨迹超过样本数量预算')
    u=np.linspace(0,bridge_seconds,count)
    append(start+u,spline(u))
    keep=t>end
    append(t[keep]-end+start+bridge_seconds,q[keep])
    result=copy.deepcopy(trace)
    # Synthetic times must not masquerade as captured low-state timestamps.
    result['raw']=dict(positions=rows,time_s=times,
        stamp_ns=[1_000_000_000+round(v*1e9) for v in times],sample_policy='offline_edited')
    result['hand']=None
    result.pop('optimization',None)
    result['trim_edit']=dict(start_s=start,end_s=end,bridge_s=bridge_seconds,
        previous_seconds=duration,label=f'裁切 {start:.3f}–{end:.3f}s',synthetic=True,
        bridge_policy='matched_boundary_derivatives_v2',
        boundary_velocities=[va.tolist(),vb.tolist()],boundary_accelerations=[aa.tolist(),ab.tolist()])
    return result
