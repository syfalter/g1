"""Offline smooth-first candidates, certified on the final quintic commands."""
import math
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET
from dataclasses import asdict

import numpy as np
from scipy.interpolate import BSpline
from scipy.sparse import csr_matrix
from scipy.sparse.linalg import spsolve
from trajectory_msgs.msg import JointTrajectory,JointTrajectoryPoint

from task_composition_preview import stamp
from task_composition_smoothing import evaluate,bounds_from_joint_delta
from task_trajectory import coefficients,time_seconds,bernstein_controls


def polynomials(trajectory):
    times=np.array([time_seconds(p) for p in trajectory.points])
    if len(times)<2 or times[0]!=0. or np.any(np.diff(times)<=0):raise ValueError('Invalid smooth timing clock')
    return times,np.array([coefficients(a,b,dt) for a,b,dt in zip(trajectory.points,trajectory.points[1:],np.diff(times))])


def restrict(polys,offset,width):
    result=np.zeros_like(polys)
    for k in range(polys.shape[1]):
        for j in range(k+1):
            result[:,j]+=polys[:,k]*(math.comb(k,j)*offset**(k-j)*width**j)[:,None]
    return result


def path_bound(reference,candidate,model,side):
    """Continuous Bernstein difference on the union of both sets of breaks."""
    a,p=polynomials(reference);b,q=polynomials(candidate)
    if reference.joint_names!=candidate.joint_names or abs(a[-1]-b[-1])>1e-8:
        raise ValueError('Path certification requires matching clocks and joints')
    times=np.unique(np.r_[a,b]);lo=times[:-1];width=np.diff(times)
    pieces=[]
    for knots,poly in ((a,p),(b,q)):
        owners=np.clip(np.searchsorted(knots,lo,side='right')-1,0,len(knots)-2)
        dt=np.diff(knots)[owners]
        pieces.append(restrict(poly[owners],(lo-knots[owners])/dt,width/dt))
    delta=bernstein_controls((pieces[1]-pieces[0]).transpose(1,0,2).reshape(6,-1))
    delta=np.max(abs(delta.reshape(6,len(lo),-1)),axis=(0,1))
    return bounds_from_joint_delta(delta,reference.joint_names,model,side+'_index_tcp')


def _absolute_integral(poly):
    roots=np.polynomial.polynomial.polyroots(poly)
    cuts=sorted([0.,1.]+[float(r.real) for r in roots if abs(r.imag)<1e-8 and 0.<r.real<1.])
    primitive=np.polynomial.polynomial.polyint(poly)
    return sum(abs(np.polynomial.polynomial.polyval(b,primitive)-np.polynomial.polynomial.polyval(a,primitive))
               for a,b in zip(cuts,cuts[1:]))


def smoothness(trajectory):
    times,poly=polynomials(trajectory);dt=np.diff(times);jerks=[];energy=0.;variation=0.;snap_variation=0.
    for p,d in zip(poly,dt):
        jerk=np.polynomial.polynomial.polyder(p,3)/d**3;jerks.append(jerk)
        for axis in jerk.T:
            square=np.polynomial.polynomial.polymul(axis,axis)
            energy+=d*sum(v/(i+1) for i,v in enumerate(square))
            variation+=d*_absolute_integral(axis)
            snap_variation+=_absolute_integral(np.polynomial.polynomial.polyder(axis))
    left=np.array([j[0] for j in jerks]);right=np.array([np.sum(j,axis=0) for j in jerks])
    jumps=abs(left[1:]-right[:-1]);total_jump=float(np.sum(jumps)+np.sum(abs(left[0]))+np.sum(abs(right[-1])))
    duration=float(times[-1])
    return dict(seconds=duration,integral_squared_jerk=float(energy),acceleration_total_variation=float(variation),
        jerk_total_variation=float(snap_variation+total_jump),max_jerk_jump=float(np.max(jumps,initial=0.)),
        boundary_jumps_over_1=int(np.sum(np.max(jumps,axis=1)>1.)) if len(jumps) else 0,
        endpoint_jerk_max=float(max(np.max(abs(left[0])),np.max(abs(right[-1])))),
        normalized_jerk_energy=float(energy*duration**5),normalized_acceleration_variation=float(variation*duration**2),
        normalized_jerk_variation=float((snap_variation+total_jump)*duration**3))


def fit_regularized(source,spacing,strength,check=lambda:None,*,robust=False):
    duration=time_seconds(source.points[-1]);names=list(source.joint_names)
    for p in (source.points[0],source.points[-1]):
        if max(abs(v) for v in (*p.velocities,*p.accelerations))>1e-6:
            raise ValueError('Smooth business units must start and finish at rest')
    spans=max(4,int(math.ceil(duration/spacing)))
    if spans>1800:raise ValueError('Smooth fitting knot budget exceeded')
    breaks=np.round(np.linspace(0.,duration,spans+1)*1e9)/1e9
    knots=np.r_[[0.]*6,breaks[1:-1],[duration]*6];count=len(knots)-6
    controls=np.zeros((count,len(names)))
    # Four equal endpoint controls impose q, v=0, a=0 and jerk=0 at each end.
    controls[:4]=source.points[0].positions;controls[-4:]=source.points[-1].positions
    samples=np.linspace(0.,duration,max(32,int(math.ceil(duration/.02))+1))
    basis=BSpline.design_matrix(samples,knots,5).tocsr()
    target=evaluate(source,samples);weight=duration/(len(samples)-1)
    free=basis[:,4:-4];residual=target-basis@controls
    gauss,weights=np.polynomial.legendre.leggauss(3)
    dt=np.diff(breaks);mid=(breaks[:-1]+breaks[1:])/2
    probes=(mid[:,None]+dt[:,None]*gauss/2).ravel();quadrature=(dt[:,None]*weights/2).ravel()
    basis_spline=BSpline(knots,np.eye(count),5)
    penalty=csr_matrix((count-8,count-8));penalty_rhs=np.zeros((count-8,len(names)))
    for order,coefficient in ((3,.1*strength**6),(4,strength**8)):
        check();derivative=basis_spline.derivative(order)
        design=BSpline.design_matrix(probes,derivative.t,derivative.k)@csr_matrix(derivative.c[:len(derivative.t)-derivative.k-1])
        d=design[:,4:-4];weighted=d.multiply((quadrature*coefficient)[:,None])
        penalty+=d.T@weighted
        penalty_rhs-=weighted.T@(design@controls)
    weights_fit=np.ones(len(samples))
    for iteration in range(3 if robust else 1):
        check();weighted=free.multiply((weight*weights_fit)[:,None])
        normal=free.T@weighted+penalty;rhs=weighted.T@residual+penalty_rhs
        controls[4:-4]=np.asarray(spsolve(normal.tocsc(),rhs)).reshape(count-8,len(names))
        if robust:
            error=np.linalg.norm(basis@controls-target,axis=1)/math.sqrt(len(names))
            scale=max(5e-5,1.4826*float(np.median(abs(error-np.median(error)))))
            weights_fit=1./np.sqrt(1.+(error/(2.*scale))**2)
    if not np.isfinite(controls).all():raise ValueError('Non-finite smooth fit')
    spline=BSpline(knots,controls,5)
    times=np.unique(np.r_[np.concatenate([np.linspace(a,b,max(1,int(math.ceil((b-a)/.08)))+1)[:-1]
        for a,b in zip(breaks,breaks[1:])]),duration])
    times=np.unique(np.round(times*1e9)/1e9)
    result=JointTrajectory(joint_names=names)
    values=[spline(times,nu=k) for k in range(3)]
    for i,t in enumerate(times):
        result.points.append(stamp(JointTrajectoryPoint(positions=values[0][i].tolist(),
            velocities=values[1][i].tolist(),accelerations=values[2][i].tolist()),t))
    return result


def smooth_candidate(reference,model,side,parameters,jerk,limits,fastest_seconds,check=lambda:None,
                     progress=lambda text:None,max_extra=.15):
    """Search smooth curves, then only uniformly scale; never reapply local warps."""
    from task_trajectory_speed import motion_limits,rescale
    from task_cruise_timing import execution_limit_factors
    from task_offline_optimizer import validate
    limits=np.asarray(limits,dtype=float)
    if limits.shape!=(3,) or not np.isfinite(limits).all() or np.any(limits<=0) or not 0<=max_extra<=1:
        raise ValueError('Invalid smooth-first budget')
    initial=smoothness(reference);choices=[];trials=[]
    vmax,amax=motion_limits(reference,model,1.,1.,parameters)
    motion=[np.array([d[n] for n in reference.joint_names]) for d in (vmax,amax,dict.fromkeys(reference.joint_names,jerk))]
    for spacing in (.06,.09,.12,.18):
        for strength in (.015,.03,.04,.05):
            check();progress(f'平顺拟合：结点 {spacing:g}s / 正则尺度 {strength:g}s')
            trial=dict(spacing_s=spacing,strength_s=strength)
            try:
                candidate=fit_regularized(reference,spacing,strength,check,robust=True)
                bound=path_bound(reference,candidate,model,side);trial['bounds']=bound
                if any(bound[k]>limit*.98 for k,limit in zip(('joint_rad','tcp_m','orientation_rad'),limits)):
                    trial['rejected']='path_tolerance';trials.append(trial);continue
                factor=max(execution_limit_factors(candidate,motion,check))*1.01
                candidate=rescale(candidate,1./max(factor,1e-8))
                peaks=validate(candidate,model,parameters,jerk,check);metrics=smoothness(candidate)
                trial.update(seconds=metrics['seconds'],metrics=metrics,peaks=peaks)
                if metrics['max_jerk_jump']>max(1e-4,jerk*1e-4) or metrics['endpoint_jerk_max']>max(1e-4,jerk*1e-4):
                    trial['rejected']='final_jerk_continuity'
                elif metrics['seconds']>fastest_seconds*(1+max_extra):trial['rejected']='time_budget'
                else:
                    lower_energy=metrics['normalized_jerk_energy']<initial['normalized_jerk_energy']*.98
                    # A short minimum-jerk bridge may already minimize energy,
                    # yet have a jerk step into the stationary hold. Closing
                    # that boundary can modestly increase energy; label it.
                    boundary_only=(max(initial['max_jerk_jump'],initial['endpoint_jerk_max'])>1e-3 and
                        metrics['normalized_jerk_energy']<=initial['normalized_jerk_energy']*1.2 and
                        metrics['normalized_acceleration_variation']<=initial['normalized_acceleration_variation']*1.1)
                    if not lower_energy and not boundary_only:trial['rejected']='no_shape_improvement'
                    else:
                        score=metrics['normalized_jerk_energy']/max(initial['normalized_jerk_energy'],1e-20)+.2*metrics['normalized_jerk_variation']/max(initial['normalized_jerk_variation'],1e-20)+.3*(metrics['seconds']/fastest_seconds)**2
                        trial.update(score=score,selection_reason='roughness_reduction' if lower_energy else 'boundary_continuity')
                        choices.append((score,candidate,trial))
            except ValueError as error:
                check();trial['rejected']=str(error)
            trials.append(trial)
    if not choices:return None,dict(accepted=False,trials=trials,reference_metrics=initial,hardware_executable=False)
    _,result,chosen=min(choices,key=lambda item:item[0])
    return result,dict(accepted=True,chosen=chosen,trials=trials,reference_metrics=initial,
        method='regularized_c4_quintic_uniform_timing_v4',hardware_executable=False,physical_verified=False,
        geometry_reference='same_policy_original_path_retimed',max_extra_fraction=max_extra)


def cached_candidate(result,model,side,parameters,policy,ranges,check=lambda:None,
                     progress=lambda text:None,max_extra=.15,directory=None):
    from task_offline_optimizer import decode
    from task_composition_preview import ROOT
    from task_screen_teach import fingerprint,write_json
    from rosidl_runtime_py.convert import message_to_ordereddict
    limits=np.min([policy.limits(role) for _,_,role in ranges],axis=0)
    code=hashlib.sha256()
    for name in ('task_smooth_timing','task_offline_optimizer','task_composition_smoothing','task_trajectory','task_trajectory_speed','task_cruise_timing'):
        code.update(Path(__file__).with_name(name+'.py').read_bytes())
    identity=dict(baseline=result['variants']['baseline']['trajectory'],fastest=result['variants']['candidate']['seconds'],
        side=side,parameters=asdict(parameters),jerk=policy.jerk,limits=limits.tolist(),max_extra=max_extra,
        model=hashlib.sha256(ET.tostring(model.robot)).hexdigest(),code=code.hexdigest())
    key=fingerprint(identity);path=Path(directory or ROOT/'data/smooth_timing_cache')/(key+'.json')
    try:
        saved=json.loads(path.read_text())
        if saved['key']==key and fingerprint(saved['payload'])==saved['sha256']:
            check();return saved['payload'],True
    except (OSError,ValueError,KeyError):pass
    reference=decode(identity['baseline'])
    # Reserve space for the preceding path-preserving retimer's numerical error.
    reserve=bounds_from_joint_delta(np.full(len(reference.joint_names),1e-7),reference.joint_names,model,side+'_index_tcp')
    adjusted=limits-np.array([reserve[k] for k in ('joint_rad','tcp_m','orientation_rad')])
    tr,report=smooth_candidate(reference,model,side,parameters,policy.jerk,adjusted,identity['fastest'],
        check,progress,max_extra)
    report['reference_path_error_reserve']=reserve
    report['mixed_roles_use_tightest_tolerance']=len({r for _,_,r in ranges})>1
    if tr is None:
        tr=decode(result['variants']['candidate']['trajectory'])
        report['fallback']='previous_fastest_candidate_no_accepted_smooth_fit'
    report['final_metrics']=smoothness(tr)
    payload=dict(trajectory=message_to_ordereddict(tr),seconds=time_seconds(tr.points[-1]),report=report)
    check();write_json(path,dict(key=key,payload=payload,sha256=fingerprint(payload)))
    return payload,False
