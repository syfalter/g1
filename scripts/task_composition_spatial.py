"""Bounded continuous-unit candidates for offline gear composition."""
import math
import copy

import numpy as np
from trajectory_msgs.msg import JointTrajectory

from task_composition_smoothing import smooth_candidate,bounds_from_joint_delta
from task_trajectory import time_seconds,coefficients,bernstein_controls


def continuous_gear_phases(phases, recipes):
    from task_composition_preview import concatenate
    merged = {}
    replacements = {}
    for unit in ('entry_lower', 'entry_upper', 'lu', 'ul', 'exit_lower', 'exit_upper'):
        sequence = recipes[unit]
        name = unit+'_continuous'
        classes = {phases[p]['motionClass'] for p in sequence}
        trajectory = concatenate(phases[p]['trajectory'] for p in sequence)
        elapsed=0.;seams=[]
        for i,p in enumerate(sequence):
            seams.extend(elapsed+t for t in phases[p].get('stopHints',[]))
            elapsed+=time_seconds(phases[p]['trajectory'].points[-1])
            if i<len(sequence)-1:seams.append(elapsed)
        merged[name] = dict(trajectory=trajectory,
            motionClass=next(iter(classes)) if len(classes)==1 else 'mixed',
            smooth=True, spatial=True, mergedPhases=list(sequence),stopHints=sorted(set(seams)))
        replacements[unit] = [name]
    replacements['cycle_lower'] = replacements['lu']+replacements['ul']
    replacements['cycle_upper'] = replacements['ul']+replacements['lu']
    return merged, replacements


def blend_stops(source,model,side,seams,limits,check):
    from task_composition_preview import at,section,stamp,concatenate,check_positions
    from task_local_timing import _compose
    result=copy.deepcopy(source);bounds=dict(joint_rad=0.,tcp_m=0.,orientation_rad=0.);edits=[]
    anchors=np.unique([0.,*seams,time_seconds(source.points[-1])])
    # Work backwards: replacing a later disjoint interval does not shift the
    # original clock coordinates of an earlier interval.
    for i in range(len(anchors)-2,0,-1):
        check();t=anchors[i];width=min(.8,.45*(t-anchors[i-1]),.45*(anchors[i+1]-t))
        if width<.03:continue
        center=at(source,t)
        if np.linalg.norm(center.velocities)>.02:continue
        for scale in (1.,.7,.5,.3,.2,.1):
            w=width*scale
            if w<.02:continue
            a,b=at(source,t-w),at(source,t+w)
            if np.dot(np.asarray(center.positions)-a.positions,np.asarray(b.positions)-center.positions)<-1e-12:
                continue
            reference=section(source,t-w,t+w)
            span=2*w*.5
            bridge=JointTrajectory(joint_names=list(source.joint_names),
                points=[stamp(copy.deepcopy(a),0.),stamp(copy.deepcopy(b),span)])
            span=time_seconds(bridge.points[-1]);reference_span=time_seconds(reference.points[-1])
            poly=coefficients(bridge.points[0],bridge.points[1],span)
            delta=np.zeros(len(source.joint_names))
            for left,right in zip(reference.points,reference.points[1:]):
                dt=time_seconds(right)-time_seconds(left)
                mapped=_compose(poly,np.array([time_seconds(left)/reference_span,dt/reference_span]))
                error=mapped.copy();error[:6]-=coefficients(left,right,dt)
                delta=np.maximum(delta,np.max(np.abs(bernstein_controls(error)),axis=0))
            bound=bounds_from_joint_delta(delta,source.joint_names,model,side+'_index_tcp')
            if any(bound[key]>limit*.98 for key,limit in zip(bounds,limits)):continue
            check_positions(bridge,model)
            result=concatenate([section(result,0.,t-w),bridge,
                                section(result,t+w,time_seconds(result.points[-1]))])
            for key in bounds:bounds[key]=max(bounds[key],bound[key])
            edits.append(dict(source_time_s=float(t),source_window_s=float(2*w),
                replacement_seconds=float(span),candidate_mid_speed_rad_s=float(np.linalg.norm(at(bridge,span/2).velocities))))
            break
    return result,bounds,edits


def rebuild(source, model, side, *, joint_tolerance=.003, tcp_tolerance=.001,
            orientation_tolerance=math.radians(.5), window=.02, seams=(),check=lambda:None,
            valley_refit=False, parameters=None):
    """Only business endpoints are constrained to rest, not constituent seams.

    Reuse certified spatial smoothing across the complete unit, then let the
    caller retime it. The source clock avoids magnifying small path ripples
    through a nearly singular inverse arc clock during candidate fitting.
    """
    for point in (source.points[0],source.points[-1]):
        if max(abs(v) for v in (*point.velocities,*point.accelerations))>1e-6:
            raise ValueError('连续路径单元必须在业务边界停稳')
    limits=(joint_tolerance,tcp_tolerance,orientation_tolerance)
    if not all(math.isfinite(v) and v>0 for v in limits):raise ValueError('Invalid continuity bounds')
    if valley_refit:
        from task_spatial_valleys import refit_valleys
        blended,bounds,edits=refit_valleys(source,model,side,seams,[v*.7 for v in limits],check,parameters)
    else:
        blended,bounds,edits=blend_stops(source,model,side,seams,[v*.7 for v in limits],check)
    remaining=[limit-bounds[key] for key,limit in zip(bounds,limits)]
    candidate,report=smooth_candidate(blended,model,side,joint_tolerance=remaining[0],
        tcp_tolerance=remaining[1],orientation_tolerance=remaining[2],
        window=window,check=check)
    for key in bounds:report[key]+=bounds[key]
    report.update(joint_tolerance_rad=joint_tolerance,tcp_tolerance_m=tcp_tolerance,
        orientation_tolerance_rad=orientation_tolerance,local_blends=edits,
        bound_policy='disjoint_local_time_correspondences_plus_continuous_smoothing')
    report.update(method='bounded_spatial_valleys_v2' if valley_refit else 'bounded_shared_unit_path_v1',
        internal_stop_constraints_preserved=False,source_clock_used_only_for_candidate_fit=True)
    if valley_refit:
        report['bound_policy']='continuous_spatial_correspondences_plus_continuous_smoothing'
        report['valley_refit']=True
    return candidate,report
