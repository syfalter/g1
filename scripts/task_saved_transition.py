"""Plan a reviewed prefix into an immutable archive; never sends robot commands."""
import copy
import math

from task_session import ARMS, ARM_NAMES
from task_path_smoothing import joint_line_trajectory
from task_trajectory import time_seconds,samples
from task_trajectory_speed import scale_arm_segments,motion_limits

COLLISION_SCOPE='operator_clearance_no_collision_check'


def start_differences(actual, target, tolerance=.025):
    watched=ARM_NAMES+(('waist_yaw_joint',) if 'waist_yaw_joint' in target else ())
    if any(name not in actual or name not in target or
           not math.isfinite(actual[name]) or not math.isfinite(target[name]) for name in watched):
        raise ValueError('Missing or nonfinite arm/waist feedback for saved-start transition')
    return {name:actual[name]-target[name] for name in watched
            if abs(actual[name]-target[name])>tolerance}


def plan_transition(planner, model, actual, target, side, ticket, parameters, check, *, tolerance=.025):
    differences=start_differences(actual,target,tolerance)
    if not differences:return []
    if 'waist_yaw_joint' in differences and not planner.waist_assist:
        raise RuntimeError('Saved start requires waist alignment; enable waist planning first')
    check()
    current=copy.deepcopy(actual)
    segments=[]
    # Match the passive arm first, then the task arm (and shared waist).
    for arm in (('right' if side=='left' else 'left'),side):
        changed=any(name in differences for name in ARMS[arm])
        if not changed and not (arm==side and 'waist_yaw_joint' in differences):continue
        check()
        names=list(ARMS[arm])+(['waist_yaw_joint'] if planner.waist_assist else [])
        end=dict(current);end.update({name:target[name] for name in names})
        trajectory=joint_line_trajectory(names,[current[n] for n in names],[end[n] for n in names])
        segment=dict(side=arm,label='Align '+arm.upper()+' to saved start',stage='prepare',
                     trajectory=trajectory,start=dict(current),end=end,number=0,kind='arm',
                     role='saved_start_transition',collision_checked=False,
                     duration=time_seconds(trajectory.points[-1]))
        segments.append(segment)
        current=dict(segment['end'])
    if start_differences(current,target):
        raise RuntimeError('Planned alignment does not reach the selected saved start')
    # Prefix timing is independent of the requested archive playback multiplier.
    segments,_=scale_arm_segments(segments,1.,model,parameters,policy='uniform',velocity=ticket.velocity,
                                  acceleration=ticket.acceleration,check=check)
    for segment in segments:
        trajectory=segment['trajectory']
        vmax,amax=motion_limits(trajectory,model,ticket.velocity,ticket.acceleration,parameters)
        for index,positions in enumerate(samples(trajectory,vmax,amax)):
            if index%200==0:check()
            for name,value in zip(trajectory.joint_names,positions):
                lo,hi=model.limits[name]
                if not lo+.02-1e-7<=value<=hi-.02+1e-7:
                    raise ValueError('Alignment joint limit exceeded: '+name)
    check()
    return segments


def attach_transition(plan, prefix):
    if not prefix:return plan
    from task_saved_test import range_fingerprint
    result=dict(plan,segments=prefix+plan['segments'],start=dict(prefix[0]['start']))
    result['test_range']=dict(plan['test_range'],transition_count=len(prefix),
        transition_fingerprint=range_fingerprint(prefix),
        combined_fingerprint=range_fingerprint(result['segments']),
        transition_collision_scope=COLLISION_SCOPE,
        transition_arms=[segment['side'] for segment in prefix])
    return result


def verify_transition(plan, prefix, archived):
    from task_saved_test import range_fingerprint
    proof=plan['test_range']
    if (not archived or proof.get('transition_collision_scope')!=COLLISION_SCOPE or
            range_fingerprint(prefix)!=proof.get('transition_fingerprint') or
            range_fingerprint(plan['segments'])!=proof.get('combined_fingerprint') or
            plan['start']!=prefix[0]['start']):
        raise RuntimeError('Saved-start transition changed after validation')
    previous=plan['start']
    sides=[]
    for segment in prefix:
        side=segment['side'];sides.append(side)
        if (side not in ARMS or segment.get('number')!=0 or segment.get('kind')!='arm' or
                segment.get('role')!='saved_start_transition' or segment.get('collision_checked') is not False or
                segment['start']!=previous or
                segment['label']!='Align '+side.upper()+' to saved start'):
            raise RuntimeError('Invalid saved-start transition ownership or continuity')
        trajectory=segment['trajectory'];names=trajectory.joint_names
        allowed=set(ARMS[side])|({'waist_yaw_joint'} if plan['waist_assist'] else set())
        if set(names)!=allowed or len(set(names))!=len(names) or len(trajectory.points)<2:
            raise RuntimeError('Invalid saved-start transition joints')
        for boundary,point in (('start',trajectory.points[0]),('end',trajectory.points[-1])):
            if len(point.positions)!=len(names) or any(
                    not math.isfinite(value) or abs(value-segment[boundary][name])>1e-7
                    for name,value in zip(names,point.positions)):
                raise RuntimeError('Saved-start transition endpoint differs from trajectory')
        if any(name not in segment['end'] or segment['end'][name]!=value
               for name,value in previous.items() if name not in allowed):
            raise RuntimeError('Saved-start transition changed an unowned joint')
        previous=segment['end']
    if len(set(sides))!=len(sides) or sides!=proof.get('transition_arms') or start_differences(previous,archived[0]['start']):
        raise RuntimeError('Saved-start transition does not join the selected archive')
