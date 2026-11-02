"""Offline removal of artificial internal stops with certified local deviations."""
import copy
import math
import hashlib
import json
from dataclasses import asdict
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from scipy.signal import find_peaks
from trajectory_msgs.msg import JointTrajectory,JointTrajectoryPoint

from task_composition_preview import at,section,stamp,concatenate
from task_composition_smoothing import evaluate,bounds_from_joint_delta
from task_smooth_timing import polynomials,restrict
from task_trajectory import time_seconds,coefficients,bernstein_controls,peak


def stop_audit(source):
    duration=time_seconds(source.points[-1])
    times=np.linspace(0.,duration,max(32,int(math.ceil(duration/.005))+1))
    speed=np.linalg.norm(evaluate(source,times,1),axis=1)
    indices=find_peaks(-speed,prominence=.01,distance=max(1,int(.15/(times[1]-times[0]))))[0]
    stops=[]
    for index in indices:
        t=times[index]
        if t<.15 or duration-t<.15 or speed[index]>.015:continue
        width=max(2,int(.6/(times[1]-times[0])))
        shoulder=min(max(speed[max(0,index-width):index]),max(speed[index+1:min(len(speed),index+width+1)]))
        if speed[index]<shoulder*.2:
            stops.append(dict(time_s=float(t),joint_speed_rad_s=float(speed[index]),shoulder_speed_rad_s=float(shoulder)))
    interior=(times>.15)&(times<duration-.15)
    return dict(stops=stops,count=len(stops),low_speed_seconds=float(np.sum(interior&(speed<.015))*(times[1]-times[0])),
                threshold_joint_norm_rad_s=.015,sample_period_s=float(times[1]-times[0]))


def hermite7(source,lo,hi,span):
    ends=[evaluate(source,np.array([lo,hi]),k) for k in range(4)]
    poly=np.zeros((8,len(source.joint_names)))
    for k in range(4):poly[k]=ends[k][0]*span**k/math.factorial(k)
    matrix=np.array([[math.factorial(k)/math.factorial(k-j) for k in range(4,8)] for j in range(4)])
    rhs=np.array([ends[j][1]*span**j-sum(poly[k]*math.factorial(k)/math.factorial(k-j)
                    for k in range(j,4)) for j in range(4)])
    poly[4:]=np.linalg.solve(matrix,rhs)
    return poly


def spatial_bound(source,poly,model,side):
    knots,old=polynomials(source);duration=knots[-1]
    times=np.unique(np.r_[knots,np.linspace(0.,duration,max(16,int(math.ceil(duration/.01))+1))])
    positions=evaluate(source,times)
    arc=np.r_[0.,np.cumsum(np.linalg.norm(np.diff(positions,axis=0),axis=1))]
    u=np.linspace(0.,1.,1025);q=np.polynomial.polynomial.polyval(u,poly).T
    new_arc=np.r_[0.,np.cumsum(np.linalg.norm(np.diff(q,axis=0),axis=1))]
    if min(arc[-1],new_arc[-1])<1e-10:raise ValueError('Stationary path')
    keep=np.r_[True,np.diff(new_arc)>1e-14]
    mapped=np.interp(arc/arc[-1],new_arc[keep]/new_arc[-1],u[keep]);mapped[0]=0.;mapped[-1]=1.
    lo=times[:-1];dt=np.diff(times);owners=np.clip(np.searchsorted(knots,lo,side='right')-1,0,len(knots)-2)
    old_width=np.diff(knots)[owners]
    before=restrict(old[owners],(lo-knots[owners])/old_width,dt/old_width)
    after=restrict(np.broadcast_to(poly,(len(lo),*poly.shape)),mapped[:-1],np.diff(mapped))
    after[:,:6]-=before
    controls=bernstein_controls(after.transpose(1,0,2).reshape(8,-1))
    delta=np.max(abs(controls.reshape(8,len(lo),-1)),axis=(0,1))
    return bounds_from_joint_delta(delta,source.joint_names,model,side+'_index_tcp')


def materialize(poly,span,names,first,last,check):
    """Also bound jerk interpolation error; position error alone is insufficient."""
    result=JointTrajectory(joint_names=list(names));max_error=0.
    tolerances=(1e-9,1e-6,1e-5,1e-4)
    def point(t):
        values=[np.polynomial.polynomial.polyval(t/span,np.polynomial.polynomial.polyder(poly,k))/span**k for k in range(3)]
        return stamp(JointTrajectoryPoint(positions=values[0].tolist(),velocities=values[1].tolist(),accelerations=values[2].tolist()),t)
    def append(a,b,depth=0):
        nonlocal max_error
        check();lo=time_seconds(a);hi=time_seconds(b);dt=hi-lo
        exact=restrict(poly[None],np.array([lo/span]),np.array([dt/span]))[0]
        error=exact.copy();error[:6]-=coefficients(a,b,dt)
        bounds=[float(np.max(abs(bernstein_controls(np.polynomial.polynomial.polyder(error,k)))))/dt**k for k in range(4)]
        if any(e>tol for e,tol in zip(bounds,tolerances)):
            if depth>=14:raise ValueError('Cannot certify stop blend interpolation')
            mid=point(round((lo+hi)*.5*1e9)/1e9)
            if not lo<time_seconds(mid)<hi:raise ValueError('Stop blend timestamp collapse')
            append(a,mid,depth+1);append(mid,b,depth+1)
        else:max_error=max(max_error,bounds[0]);result.points.append(b)
    a=stamp(copy.deepcopy(first),0.);b=stamp(copy.deepcopy(last),span)
    result.points.append(a);append(a,b)
    return result,max_error


def min_speed(poly,span):
    velocity=np.polynomial.polynomial.polyder(poly)/span
    squared=np.zeros(1)
    for axis in velocity.T:squared=np.polynomial.polynomial.polyadd(squared,np.polynomial.polynomial.polymul(axis,axis))
    roots=np.polynomial.polynomial.polyroots(np.polynomial.polynomial.polyder(squared))
    probes=[0.,1.]+[r.real for r in roots if abs(r.imag)<1e-7 and 0.<r.real<1.]
    return math.sqrt(max(0.,float(min(np.polynomial.polynomial.polyval(probes,squared)))))


def retimed_role_intervals(original,retimed,ranges):
    """Locate retained role-boundary positions in a path-preserving retimer.

    Ambiguous stationary boundaries shrink the transit interval. Missing retained
    endpoints never earn the wider transit budget.
    """
    merged=[]
    for lo,hi,role in ranges:
        if merged and merged[-1][2]==role:merged[-1]=(merged[-1][0],hi,role)
        else:merged.append((lo,hi,role))
    times=np.array([time_seconds(p) for p in retimed.points]);q=np.array([p.positions for p in retimed.points])
    result=[]
    for lo,hi,role in merged:
        if role!='transit':continue
        ends=[]
        for t in (lo,hi):
            target=np.array(at(original,t).positions)
            indices=np.flatnonzero(np.max(abs(q-target),axis=1)<=1e-7)
            if not len(indices):break
            ends.append((float(times[indices[0]]),float(times[indices[-1]])))
        if len(ends)==2 and ends[0][1]<ends[1][0]:result.append((ends[0][1],ends[1][0],'transit'))
    return result


def blend_internal_stops(source,model,side,parameters,jerk,limits,check=lambda:None,progress=lambda text:None,*,limits_at=None,compress_idle=False,fit_order=7):
    from task_trajectory_speed import motion_limits
    from task_offline_optimizer import validate
    if fit_order not in (5,7):raise ValueError('Stop blend order must be 5 (C2) or 7 (C3)')
    limits=np.asarray(limits,dtype=float)
    if limits.shape!=(3,) or not np.isfinite(limits).all() or np.any(limits<=0):raise ValueError('Invalid remaining stop-blend budget')
    vmax,amax=motion_limits(source,model,1.,1.,parameters)
    motion=[np.array([d[n] for n in source.joint_names]) for d in (vmax,amax,dict.fromkeys(source.joint_names,jerk))]
    audit=stop_audit(source);result=copy.deepcopy(source);duration=time_seconds(source.points[-1]);upper=duration
    edits=[];rejected=[]
    candidates=list(audit['stops'])
    if compress_idle:
        ts=np.linspace(0.,duration,max(32,int(math.ceil(duration/.01))+1))
        quiet=np.linalg.norm(evaluate(source,ts,1),axis=1)<.03
        edges=np.diff(np.r_[False,quiet,False].astype(int))
        for begin,end in zip(np.flatnonzero(edges==1),np.flatnonzero(edges==-1)):
            lo,hi=float(ts[begin]),float(ts[min(end,len(ts)-1)])
            if hi-lo>=.3:
                candidates.append(dict(time_s=(lo+hi)*.5,joint_speed_rad_s=0.,shoulder_speed_rad_s=0.,
                                       quiet_interval=[lo,hi]))
    for stop in sorted(candidates,key=lambda s:s['time_s'],reverse=True):
        center=stop['time_s'];best=None;reasons={}
        progress(f'内部停顿 {center:.3f}s');check()
        windows=[(w,0.) for w in (1.2,1.,.8,.7,.6,.5,.45,.4,.35,.3,.25,.2,.15,.12)]
        windows.extend((w,s*w) for w in (.8,.6,.4,.3) for s in (-.5,-.25,.25,.5))
        quiet=stop.get('quiet_interval')
        if quiet:
            half=(quiet[1]-quiet[0])*.5
            windows=[(half+extra,0.) for extra in (.05,.15,.3,.5,.8)]
        for width,shift in windows:
            lo=round(max(0. if quiet else .001,center-width+shift)*1e9)/1e9
            hi=round(min(upper-.002 if upper<duration else duration,center+width+shift,duration if quiet else duration-.001)*1e9)/1e9
            if not lo<center<hi or min(center-lo,hi-center)<.05:continue
            first,last=at(source,lo),at(source,hi)
            va,vb=np.array(first.velocities),np.array(last.velocities)
            if not quiet and (va@vb<=0. or min(np.linalg.norm(va),np.linalg.norm(vb))<.005):
                reasons['opposing_or_slow_boundaries']=reasons.get('opposing_or_slow_boundaries',0)+1;continue
            reference=section(source,lo,hi)
            local_limits=limits if limits_at is None else np.asarray(limits_at(lo,hi))
            floor=max(stop['joint_speed_rad_s']*3.,min(.025,stop['shoulder_speed_rad_s']*.2))
            for ratio in ((.05,.1,.15,.2,.3,.4,.5,.65,.8) if quiet else (.3,.4,.5,.6,.7,.8,.9,1.)):
                span=round((hi-lo)*ratio*1e9)/1e9
                if span<.03:continue
                poly=(hermite7(source,lo,hi,span) if fit_order==7 else
                      np.pad(coefficients(first,last,span),((0,2),(0,0))))
                minimum=min_speed(poly,span)
                if not quiet and minimum<floor:
                    reasons['speed_floor']=reasons.get('speed_floor',0)+1;continue
                if any(peak(np.polynomial.polynomial.polyder(poly,k)[:,axis])/span**k>limit*.999
                    for k,ls in enumerate(motion,1) for axis,limit in enumerate(ls)):
                    reasons['motion_limits']=reasons.get('motion_limits',0)+1;continue
                try:bound=spatial_bound(reference,poly,model,side)
                except ValueError:
                    reasons['stationary_path']=reasons.get('stationary_path',0)+1;continue
                if any(bound[k]>limit*.98 for k,limit in zip(('joint_rad','tcp_m','orientation_rad'),local_limits)):
                    reasons['path_tolerance']=reasons.get('path_tolerance',0)+1;continue
                # Prefer time saved, then a higher verified minimum speed.
                score=(hi-lo-span)+minimum*.05
                if best is None or score>best[0]:best=(score,lo,hi,span,poly,bound,minimum,first,last)
        if best is None:rejected.append(dict(**stop,reasons=reasons));continue
        _,lo,hi,span,poly,bound,minimum,first,last=best
        try:
            bridge,error=materialize(poly,span,source.joint_names,first,last,check)
            validate(bridge,model,parameters,jerk,check)
        except ValueError as exc:
            check();rejected.append(dict(**stop,reasons=dict(final_validation=str(exc))));continue
        pieces=[]
        if lo>1e-9:pieces.append(section(result,0.,lo))
        pieces.append(bridge)
        if time_seconds(result.points[-1])-hi>1e-9:
            pieces.append(section(result,hi,time_seconds(result.points[-1])))
        result=concatenate(pieces);upper=lo
        edits.append(dict(source_time_s=center,source_begin_s=lo,source_end_s=hi,replacement_seconds=span,
            minimum_joint_speed_rad_s=minimum,bounds=bound,interpolation_error_rad=error,quiet_compression=bool(quiet)))
    validate(result,model,parameters,jerk,check)
    return result,dict(method='local_c3_stop_blending_v5' if fit_order==7 else 'local_c2_stop_blending_v6',before=audit,after=stop_audit(result),edits=edits,rejected=rejected,
        bounds={k:max((e['bounds'][k] for e in edits),default=0.) for k in ('joint_rad','tcp_m','orientation_rad')},
        hardware_executable=False,physical_verified=False)


def cached_flow(result,original,ranges,model,side,parameters,policy,check=lambda:None,
                progress=lambda text:None,max_extra=.15,directory=None,compress_idle=False):
    from task_offline_optimizer import decode
    from task_composition_preview import ROOT
    from task_screen_teach import fingerprint,write_json
    from rosidl_runtime_py.convert import message_to_ordereddict
    from task_smooth_timing import smoothness,smooth_candidate
    code=hashlib.sha256()
    for name in ('task_stop_blending','task_smooth_timing','task_offline_optimizer','task_composition_smoothing','task_trajectory','task_composition_preview'):
        code.update(Path(__file__).with_name(name+'.py').read_bytes())
    incumbent=result['variants'].get('smooth',result['variants']['candidate'])
    identity=dict(baseline=result['variants']['baseline']['trajectory'],incumbent=incumbent['trajectory'],
        original=message_to_ordereddict(original),ranges=ranges,policy=asdict(policy),parameters=asdict(parameters),
        side=side,fastest=result['variants']['candidate']['seconds'],max_extra=max_extra,
        model=hashlib.sha256(ET.tostring(model.robot)).hexdigest(),code=code.hexdigest(),compress_idle=compress_idle,
        idle_revision=3)
    key=fingerprint(identity);path=Path(directory or ROOT/'data/stop_blending_cache')/(key+'.json')
    try:
        saved=json.loads(path.read_text())
        if saved['key']==key and fingerprint(saved['payload'])==saved['sha256']:
            check();return saved['payload'],True
    except (OSError,ValueError,KeyError):pass
    reference=decode(identity['baseline']);previous=decode(identity['incumbent'])
    intervals=retimed_role_intervals(original,reference,ranges)
    keys=('joint_rad','tcp_m','orientation_rad')
    reserve=bounds_from_joint_delta(np.full(len(reference.joint_names),1e-7),reference.joint_names,model,side+'_index_tcp')
    reserve_array=np.array([reserve[k] for k in keys])
    pure_transit=all(role=='transit' for _,_,role in ranges)
    def role_at(lo,hi):return 'transit' if pure_transit or any(a<=lo and hi<=b for a,b,_ in intervals) else 'operation'
    def budget(lo,hi):return policy.limits(role_at(lo,hi))-reserve_array
    tr,report=blend_internal_stops(reference,model,side,parameters,policy.jerk,
        policy.limits('operation')-reserve_array,check,progress,limits_at=budget,compress_idle=compress_idle)
    for edit in report['edits']:edit['role']=role_at(edit['source_begin_s'],edit['source_end_s'])
    report['role_intervals']=intervals;report['reference_path_error_reserve']=reserve
    report['geometry_reference']='same_policy_original_path_retimed'
    before=stop_audit(previous);after=report['after'];report['previous_candidate_audit']=before
    report['trial_audit']=copy.deepcopy(after)
    report['trial_seconds']=time_seconds(tr.points[-1])
    applied=bool(report['edits']) and after['count']<=before['count'] and (
        after['count']<before['count'] or after['low_speed_seconds']<before['low_speed_seconds']*.85)
    if compress_idle and report['edits']:
        # A shorter stationary interval matters more than the number of very
        # shallow velocity minima. Never accept this alternative if it is slower.
        applied=applied or (after['low_speed_seconds']<before['low_speed_seconds']*.85 and
                           after['count']<=before['count']+1 and
                           time_seconds(tr.points[-1])<=time_seconds(previous.points[-1])*.95)
    applied=applied and time_seconds(tr.points[-1])<=identity['fastest']*(1+max_extra)
    if applied:
        # Smooth the new clock only after the old stop constraints are removed.
        # Additive budgets prevent spending the same geometric tolerance twice.
        roles={role for _,_,role in ranges}
        remaining=np.min([policy.limits(role)-reserve_array-np.array([
            max((e['bounds'][k] for e in report['edits'] if e['role']==role or e['role']=='operation'),default=0.)
            for k in keys]) for role in roles],axis=0)
        if np.all(remaining>0):
            polished,polish=smooth_candidate(tr,model,side,parameters,policy.jerk,remaining,
                min(identity['fastest']*(1+max_extra),time_seconds(tr.points[-1])*1.12,time_seconds(previous.points[-1])),check,progress,0.)
            report['polishing']=polish
            if polished is not None:
                polished_audit=stop_audit(polished)
                if polished_audit['count']<=after['count'] and polished_audit['low_speed_seconds']<=after['low_speed_seconds']*1.05+1e-6:
                    tr=polished;report['after']=polished_audit;report['polishing_selected']=True
    else:
        tr=previous;report['fallback']='no_stop_improvement_within_path_and_time_limits';report['after']=before
    report['accepted']=applied;report['final_metrics']=smoothness(tr)
    payload=dict(trajectory=message_to_ordereddict(tr),seconds=time_seconds(tr.points[-1]),report=report)
    check();write_json(path,dict(key=key,payload=payload,sha256=fingerprint(payload)))
    return payload,False
