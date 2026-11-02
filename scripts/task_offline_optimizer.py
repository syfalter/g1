"""Segment-aware path experiments. No execution or release interface."""
import copy
from dataclasses import asdict, dataclass
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from rosidl_runtime_py.convert import message_to_ordereddict
from rosidl_runtime_py.set_message import set_message_fields
from trajectory_msgs.msg import JointTrajectory

from task_composition_preview import ROOT, section, concatenate, check_positions
from task_composition_smoothing import smooth_candidate
from task_screen_teach import fingerprint, write_json
from task_trajectory import time_seconds, coefficients, peak, validate_waist_envelope

CACHE = ROOT/'data/offline_optimizer_cache'


@dataclass(frozen=True)
class Policy:
    jerk: float = 5.
    contact_mm: float = 1.
    transit_mm: float = 5.
    window: float = .02

    def validate(self):
        if not all(math.isfinite(v) and v>0 for v in asdict(self).values()):
            raise ValueError('Invalid offline optimization policy')
        if self.transit_mm<self.contact_mm:
            raise ValueError('Transit tolerance must not be smaller than contact tolerance')

    def limits(self, role):
        scale=(self.transit_mm if role=='transit' else self.contact_mm)
        return np.array([.003*scale,.001*scale,math.radians(.5)*scale])


def validate(trajectory, model, parameters, jerk, check=lambda:None):
    from task_trajectory_speed import motion_limits
    if not math.isfinite(jerk) or jerk<=0 or len(trajectory.points)<2:
        raise ValueError('Invalid experimental trajectory or jerk policy')
    for p in trajectory.points:
        if any(len(getattr(p,k))!=len(trajectory.joint_names) or not np.isfinite(getattr(p,k)).all()
               for k in ('positions','velocities','accelerations')):
            raise ValueError('Incomplete or non-finite experimental trajectory')
    check_positions(trajectory,model)
    vmax,amax=motion_limits(trajectory,model,1.,1.,parameters)
    limits=[vmax,amax,dict.fromkeys(trajectory.joint_names,jerk)]
    peaks=[dict.fromkeys(trajectory.joint_names,0.) for _ in limits]
    for a,b in zip(trajectory.points,trajectory.points[1:]):
        check();dt=time_seconds(b)-time_seconds(a)
        if dt<=0:raise ValueError('Non-increasing experimental trajectory time')
        poly=coefficients(a,b,dt)
        for order in (1,2,3):
            derivative=np.polynomial.polynomial.polyder(poly,order)/dt**order
            for i,name in enumerate(trajectory.joint_names):
                value=peak(derivative[:,i]);peaks[order-1][name]=max(peaks[order-1][name],value)
                if value>limits[order-1][name]*(1+1e-6):
                    raise ValueError(f'Offline candidate exceeds derivative {order}: {name}')
    if 'waist_yaw_joint' in trajectory.joint_names:
        lo,hi=model.limits['waist_yaw_joint']
        validate_waist_envelope(trajectory,lo+.02,hi-.02,vmax['waist_yaw_joint'],amax['waist_yaw_joint'])
    return dict(velocity=max(peaks[0].values()),acceleration=max(peaks[1].values()),
                jerk=max(peaks[2].values()),jerk_limit=jerk,by_joint=peaks)


def retime(source, model, parameters, jerk, check, baseline=None):
    from task_fast_timing import fastest, local_candidate
    from task_trajectory_speed import motion_limits,rescale
    from task_cruise_timing import execution_limit_factors
    if jerk==5.:
        try:result,report=fastest(source,model,parameters,velocity=1.,acceleration=1.,check=check)
        except ValueError as error:
            check()
            vmax,amax=motion_limits(source,model,1.,1.,parameters)
            limits=[np.array([d[n] for n in source.joint_names])
                    for d in (vmax,amax,dict.fromkeys(source.joint_names,jerk))]
            factor=max(execution_limit_factors(source,limits,check))*1.01
            choices=[(rescale(source,1./max(factor,1e-8)),dict(method='verified_uniform_fallback',previous_error=str(error)))]
            try:choices.append(local_candidate(source,model,parameters,1.,1.,check))
            except ValueError:check()
            result,report=min(choices,key=lambda item:time_seconds(item[0].points[-1]))
    else:
        if baseline is None:
            baseline,_=retime(source,model,parameters,5.,check)
        vmax,amax=motion_limits(source,model,1.,1.,parameters)
        jerk_limits=dict.fromkeys(source.joint_names,jerk)
        limits=[np.array([d[n] for n in source.joint_names]) for d in (vmax,amax,jerk_limits)]
        choices=[]
        if jerk>=5.:choices.append((baseline,dict(method='production_policy_incumbent')))
        for tr in (source,baseline):
            factor=max(execution_limit_factors(tr,limits,check))*1.01
            choices.append((rescale(tr,1./max(factor,1e-8)),dict(method='verified_uniform_experiment')))
        try:
            choices.append(local_candidate(source,model,parameters,1.,1.,check,jerk_limits=jerk_limits))
        except ValueError:
            check()
        result,report=min(choices,key=lambda item:time_seconds(item[0].points[-1]))
    evidence=validate(result,model,parameters,jerk,check)
    return result,dict(report,peaks=evidence,jerk_policy_rad_s3=jerk,
        limit_provenance='project_policy_not_manufacturer_motor_limit',hardware_executable=False)


def validate_ranges(source,ranges):
    duration=time_seconds(source.points[-1]);cursor=0.
    for begin,end,role in ranges:
        if abs(begin-cursor)>1e-8 or not begin<end<=duration+1e-8 or role not in ('transit','operation'):
            raise ValueError('Path roles must cover the complete unit without gaps')
        cursor=end
    if abs(cursor-duration)>1e-8:raise ValueError('Path roles do not cover the endpoint')


def local_path(source, model, side, ranges, policy, parameters, check):
    """Independent C2 fits avoid one difficult corner throttling all smoothing."""
    policy.validate();validate_ranges(source,ranges)
    duration=time_seconds(source.points[-1])
    parts=[];reports=[]
    for begin,end,role in ranges:
        # Larger windows span teaching ripples but local endpoint derivatives
        # remain matched, so boundaries do not introduce additional stops.
        knots=np.array([time_seconds(p) for p in source.points if begin<time_seconds(p)<end])
        divisions=[begin]
        for target in np.arange(begin+1.5,end-.25,1.5):
            if len(knots):
                t=float(knots[np.argmin(abs(knots-target))])
                if t-divisions[-1]>.3:divisions.append(t)
        divisions.append(end)
        limits=policy.limits(role)*.65
        for lo,hi in zip(divisions,divisions[1:]):
            check();piece=section(source,lo,hi)
            if len(piece.points)<6:
                parts.append(piece);continue
            fitted,report=smooth_candidate(piece,model,side,joint_tolerance=limits[0],
                tcp_tolerance=limits[1],orientation_tolerance=limits[2],window=policy.window,check=check)
            parts.append(fitted);reports.append(dict(begin=lo,end=hi,role=role,**report))
    fitted=concatenate(parts)
    from task_spatial_valleys import refit_valleys
    def remaining(lo,hi):
        return np.min([policy.limits(role)*.35 for a,b,role in ranges if a<hi and b>lo],axis=0)
    candidate,bounds,edits=refit_valleys(fitted,model,side,np.arange(0.,duration,.4),
        policy.limits('operation')*.35,check,parameters,limits_at=remaining)
    for edit in edits:
        edit['roles']=sorted({r for a,b,r in ranges if a<edit['source_end_s'] and b>edit['source_begin_s']})
    by_role={}
    for role in ('transit','operation'):
        smoothing=[r for r in reports if r['role']==role]
        blending=[e['bounds'] for e in edits if role in e['roles']]
        by_role[role]={k:max([r[k] for r in smoothing],default=0.)+
                           max([r[k] for r in blending],default=0.)
                      for k in ('joint_rad','tcp_m','orientation_rad')}
        if any(by_role[role][k]>v+1e-9 for k,v in zip(by_role[role],policy.limits(role))):
            raise ValueError('Combined spatial edits exceed role tolerance')
    return candidate,dict(method='local_c2_fits_and_spatial_valleys_v3',by_role=by_role,
        local_fits=reports,valley_replacements=edits,source_ranges=ranges,
        endpoints_preserved=True,hardware_executable=False)


def optimize(source, model, side, ranges, policy, parameters, check=lambda:None,
             progress=lambda text:None, directory=CACHE, *, seams=()):
    policy.validate();validate_ranges(source,ranges)
    code=hashlib.sha256()
    for name in ('task_offline_optimizer','task_composition_preview','task_composition_smoothing','task_composition_spatial','task_spatial_valleys',
                 'task_fast_timing','task_local_timing','task_cruise_timing','task_trajectory_speed','task_trajectory'):
        code.update(Path(__file__).with_name(name+'.py').read_bytes())
    identity=dict(source=message_to_ordereddict(source),ranges=ranges,seams=list(seams),side=side,policy=asdict(policy),
        parameters=asdict(parameters),model=hashlib.sha256(ET.tostring(model.robot)).hexdigest(),code=code.hexdigest())
    key=fingerprint(identity);path=Path(directory)/(key+'.json')
    try:
        data=json.loads(path.read_text())
        if data['key']==key and fingerprint(data['payload'])==data['sha256']:
            check();return data['payload'],True
    except (OSError,ValueError,KeyError):pass
    progress('生产策略原路径')
    original,original_report=retime(source,model,parameters,5.,check)
    progress('当前 jerk 策略原路径')
    baseline,baseline_report=(original,original_report) if policy.jerk==5. else retime(
        source,model,parameters,policy.jerk,check,baseline=original)
    progress('分区间连续拟合及低速谷修整')
    try:
        revised,geometry=local_path(source,model,side,ranges,policy,parameters,check)
        progress('拟合路径重新调速与连续限值检查')
        candidate,report=retime(revised,model,parameters,policy.jerk,check)
        trials=[dict(method=geometry['method'],seconds=time_seconds(candidate.points[-1]))]
    except ValueError as error:
        check();candidate,report=baseline,baseline_report
        geometry=dict(method='unchanged_verified_path',source_ranges=ranges,by_role={},hardware_executable=False)
        trials=[dict(method='local_c2_fits_and_spatial_valleys_v3',rejected=str(error))]
    # Keep previous bounded fitting strategies in the search. A new proposal
    # must not replace a faster verified incumbent simply because it is newer.
    limits=np.min([policy.limits(role) for _,_,role in ranges],axis=0)
    from task_composition_spatial import rebuild
    for method in (['global_smoothing','previous_seam_blend','previous_valley_refit'] if seams else ['global_smoothing']):
        progress('比较既有路径策略：'+method);check()
        kwargs=dict(joint_tolerance=limits[0],tcp_tolerance=limits[1],orientation_tolerance=limits[2],window=policy.window,check=check)
        try:
            if method=='global_smoothing':other,bound=smooth_candidate(source,model,side,**kwargs)
            else:other,bound=rebuild(source,model,side,seams=seams,
                valley_refit=method=='previous_valley_refit',parameters=parameters,**kwargs)
            other,details=retime(other,model,parameters,policy.jerk,check)
            seconds=time_seconds(other.points[-1]);trials.append(dict(method=method,seconds=seconds))
            if seconds<time_seconds(candidate.points[-1]):
                candidate,report=other,details
                geometry=dict(method=method,by_role={role:{k:bound[k] for k in ('joint_rad','tcp_m','orientation_rad')}
                    for _,_,role in ranges},bound=bound,source_ranges=ranges,endpoints_preserved=True,hardware_executable=False)
        except ValueError as error:
            check();trials.append(dict(method=method,rejected=str(error)))
    tested=time_seconds(candidate.points[-1]);before=time_seconds(baseline.points[-1])
    selected=tested<before-1e-6
    if not selected:candidate,report=baseline,baseline_report
    payload=dict(policy=asdict(policy),source_sha256=fingerprint(identity['source']),
        geometry=geometry,geometry_selected=selected,trial_seconds=tested,trials=trials,
        hardware_executable=False,physical_verified=False,globally_optimal=False,variants={})
    for name,tr,details in (('original',original,original_report),('baseline',baseline,baseline_report),
                            ('candidate',candidate,report)):
        payload['variants'][name]=dict(trajectory=message_to_ordereddict(tr),seconds=time_seconds(tr.points[-1]),report=details)
    check();write_json(path,dict(key=key,payload=payload,sha256=fingerprint(payload)))
    return payload,False


def decode(spec):
    tr=JointTrajectory();set_message_fields(tr,copy.deepcopy(spec));return tr
