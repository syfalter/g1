"""Explicit commissioning ranges from immutable archives, not production task plans."""
import copy
import hashlib
import json
import math
from pathlib import Path
import tempfile
from contextlib import ExitStack

import numpy as np
from rosidl_runtime_py.convert import message_to_ordereddict
from rosidl_runtime_py.set_message import set_message_fields
from trajectory_msgs.msg import JointTrajectory

from task_plan_archive import digest, verify, stages
from task_model_resources import archived_model_tree,verify_live_meshes
from task_trajectory import (samples, sample_count, stretch_to_limits, time_seconds, validate_waist_envelope,
                             task_motion_limits, task_jerk_limits, validate_jerk_limits)
from task_dring_sequence import labels, plan_labels
from task_dring_sequence import is_center_check

LABELS = labels('left')


def load_archive(directory, robot_urdf):
    directory = Path(directory).resolve()
    manifest = verify(directory)
    plan = json.loads((directory / 'plan/plan.json').read_text())
    if is_center_check(plan):raise ValueError('Center checks require a fresh observation; do not load as a D-ring range')
    if (plan.get('planning_profile') != 'steep_straight' or
            tuple(s['label'] for s in plan['segments']) != plan_labels(plan) or
            plan.get('hardware_executable') is not False):
        raise ValueError('Expected the complete archived straight-index preview task')
    if digest(robot_urdf) != manifest['sha256']['context/robot.urdf']:
        raise ValueError('Live robot model differs from archived model')
    verify_live_meshes(directory,robot_urdf)
    scene = plan['contact_scene']
    if (scene.get('profile') != 'steep_straight' or
            scene.get('base_radius_m') != .025 or scene.get('base_height_m') != .02 or
            scene.get('arm_body_collision_checked') is not True):
        raise ValueError('Unsupported archived collision profile')
    return dict(directory=str(directory), manifest=manifest, plan=plan, stages=stages(plan))


def select_range(archive, first, last):
    if type(first) is not int or type(last) is not int or not 1 <= first <= last <= len(archive['stages']):
        raise ValueError('Select an ordered, nonempty stage range')
    return copy.deepcopy(archive['stages'][first-1:last])


def needs_contact_confirmation(first, last):
    return first <= 5 and last >= 3


def speed_copy(original, old_velocity, velocity, acceleration, waist, parameters=None):
    """Uniform time scaling preserves the stored Hermite curve; no IK or path fitting."""
    if not all(math.isfinite(v) and .05 <= v <= 1 for v in (old_velocity, velocity, acceleration)):
        raise ValueError('Scaling must be in [0.05,1.0]')
    result = copy.deepcopy(original)
    ratio = old_velocity / velocity
    for point in result.points:
        ns = round(time_seconds(point) * ratio * 1e9)
        point.time_from_start.sec, point.time_from_start.nanosec = divmod(ns, 1000000000)
        point.velocities = [v/ratio for v in point.velocities]
        point.accelerations = [a/ratio**2 for a in point.accelerations]
    vmax, amax = task_motion_limits(result.joint_names,velocity,acceleration,parameters)
    jerk=task_jerk_limits(result.joint_names)
    stretch_to_limits(result,vmax,amax,jerk)
    validate_jerk_limits(result,jerk)
    return result


def range_fingerprint(segments):
    data = []
    for segment in segments:
        trajectory = message_to_ordereddict(segment['trajectory'])
        # Gateway freshness stamping is deliberately separate from trajectory content.
        trajectory['header']['stamp'] = dict(sec=0, nanosec=0)
        data.append({k: segment.get(k) for k in ('number', 'side', 'label', 'kind', 'posture',
                                               'target_q', 'duration', 'start', 'end')})
        data[-1]['trajectory'] = trajectory
        if 'delayed_hand' in segment:data[-1]['delayed_hand']=segment['delayed_hand']
        if 'hand_target_required' in segment:data[-1]['hand_target_required']=segment['hand_target_required']
    return hashlib.sha256(json.dumps(data, sort_keys=True, allow_nan=False).encode()).hexdigest()


def verify_test_plan(plan):
    if plan.get('preview_only_timing'):
        raise RuntimeError('Unrestricted preview timing cannot authorize hardware execution')
    expected_labels=plan_labels(plan)
    proof = plan.get('test_range', {})
    first, last = proof.get('first'), proof.get('last')
    if (plan.get('kind') != 'staged_test' or type(first) is not int or type(last) is not int or
            not 1 <= first <= last <= len(expected_labels) or proof.get('scene_confirmed') is not True or
            (not is_center_check(plan) and needs_contact_confirmation(first, last) and proof.get('contact_confirmed') is not True)):
        raise RuntimeError('Missing explicit staged-test confirmations/range')
    if is_center_check(plan) and (proof.get('source')!='live_plan' or first!=1 or last!=3):
        raise RuntimeError('Center check requires its complete fresh three-stage plan')
    count=proof.get('transition_count',0)
    if type(count) is not int or not 0<=count<=2 or (count and proof.get('source')=='live_plan'):
        raise RuntimeError('Invalid saved-start transition count')
    segments = plan['segments'][count:]
    if count:
        from task_saved_transition import verify_transition
        verify_transition(plan,plan['segments'][:count],segments)
    if (len(segments) != last-first+1 or tuple(s['number'] for s in segments) != tuple(range(first,last+1)) or
            tuple(s['label'] for s in segments) != expected_labels[first-1:last] or
            range_fingerprint(segments) != proof.get('validated_fingerprint')):
        raise RuntimeError('Staged-test trajectory changed after validation')
    source=(Path(proof['source_plan_file']) if proof.get('source')=='live_plan' else
            Path(proof['archive']) / 'plan/plan.json')
    if digest(source) != proof['original_plan_sha256']:
        raise RuntimeError('Archived plan changed; reload the saved test')
    if plan_labels(json.loads(source.read_text()))!=expected_labels:
        raise RuntimeError('D-ring task direction changed after validation')


def live_test_plan(plan, ticket, stop_epoch, robot_urdf, *, scene_confirmed, contact_confirmed):
    """Authorize exactly the current validated live curve as a full commissioning test."""
    review=plan.get('live_review',{})
    if plan.get('preview_only_timing'):
        raise RuntimeError('Unrestricted preview timing cannot authorize hardware execution')
    if (not scene_confirmed or (not is_center_check(plan) and not contact_confirmed) or plan.get('kind')!='task' or
            plan.get('hardware_executable') is not False or
            review.get('generation')!=ticket.generation or review.get('stop_epoch')!=stop_epoch or
            review.get('robot_model_sha256')!=digest(robot_urdf)):
        raise RuntimeError('Current D-ring plan/contact confirmations or controller epoch do not match')
    source=Path(plan['directory'])/'plan.json'
    if digest(source)!=review.get('source_plan_sha256'):
        raise RuntimeError('Live plan changed after validation; replan')
    saved=json.loads(source.read_text())
    scene=saved.get('contact_scene',{})
    if (saved.get('hardware_executable') is not False or
            saved.get('planning_profile')!='steep_straight' or saved.get('arm')!=ticket.side or
            scene.get('profile')!='steep_straight' or scene.get('arm_body_collision_checked') is not True or
            scene.get('base_radius_m')!=.025 or scene.get('base_height_m')!=.02):
        raise RuntimeError('Unsupported D-ring test collision profile')
    records=stages(saved)
    expected_labels=plan_labels(saved)
    if (plan_labels(plan)!=expected_labels or tuple(r['label'] for r in records)!=expected_labels or
            len(plan['segments'])!=len(expected_labels)):
        raise RuntimeError('A complete validated three-stage center-check plan is required' if is_center_check(saved) else
                           'A complete validated seven-stage single-side D-ring plan is required')
    segments=[];expected=[]
    for record,segment in zip(records,plan['segments']):
        for boundary in ('start','end'):
            actual=segment[boundary];saved_state=record[boundary]
            if (actual.keys()!=saved_state.keys() or any(not math.isfinite(actual[n]) or
                    abs(actual[n]-saved_state[n])>1e-7 for n in saved_state)):
                raise RuntimeError('Live segment boundary differs from validated source')
        item=dict(segment,number=record['number'],kind=segment.get('kind','arm'),
                  duration=time_seconds(segment['trajectory'].points[-1]))
        segments.append(item)
        trajectory=JointTrajectory()
        set_message_fields(trajectory,record['original_segment']['trajectory'])
        expected.append(dict(number=record['number'],side=record['side'],label=record['label'],
            kind=record['kind'],posture=record['original_segment'].get('posture'),
            target_q=record['original_segment'].get('target_q'),trajectory=trajectory,
            duration=record['duration_s'],start=segment['start'],end=segment['end']))
    if plan['start']!=saved['start'] or range_fingerprint(segments)!=range_fingerprint(expected):
        raise RuntimeError('Live trajectory differs from validated source')
    proof=dict(source='live_plan',source_plan_file=str(source),first=1,last=len(expected_labels),arm=ticket.side,
        original_plan_sha256=review['source_plan_sha256'],validated_fingerprint=range_fingerprint(segments),
        scene_confirmed=True,contact_confirmed=bool(contact_confirmed),hands_match_at_review=True,stop_epoch=stop_epoch,
        collision_scope=scene.get('collision_scope'),production_commissioned=False)
    result=dict(plan,kind='staged_test',hardware_executable=True,test_range=proof,segments=segments)
    verify_test_plan(result)
    return result


def prepare_range(archive, first, last, velocity, acceleration, model, check, progress, parameters=None,
                  *, speed_multiplier=None, speed_report=None,use_collision_cache=False,collision_report=None):
    from task_hook_collision import HandBaseCollision
    selected = select_range(archive, first, last)
    source = archive['plan']
    if source.get('preview_only_timing'):
        raise RuntimeError('Unrestricted preview timing cannot authorize hardware execution')
    scaled = None
    if speed_multiplier is not None:
        from task_trajectory_speed import scale_arm_segments
        originals = []
        # Scale the whole archive first: a selected range must match full RViz timing.
        for record in archive['stages']:
            trajectory = JointTrajectory()
            set_message_fields(trajectory, copy.deepcopy(record['original_segment']['trajectory']))
            originals.append(dict(kind=record['kind'],label=record['label'],trajectory=trajectory))
        scaled, scaling = scale_arm_segments(originals, speed_multiplier, model, parameters,
            velocity=velocity, acceleration=acceleration, check=check)
        if speed_report is not None:
            speed_report.update(scaling)
        scaled=scaled[first-1:last]
    # FCL consumes mesh bytes during construction, so the relocated URDF is temporary.
    with ExitStack() as resources:
        temporary=resources.enter_context(tempfile.TemporaryDirectory(prefix='g1-saved-collision-'))
        tree,_=archived_model_tree(archive['directory'],require_snapshots=False)
        relocated=Path(temporary)/'robot.urdf'
        tree.write(relocated,encoding='unicode')
        checker=lambda:HandBaseCollision(model,str(relocated),source['contact_scene']['base_top_pelvis_m'])
        from task_parallel_collision import create_pool,validate
        from task_collision_cache import context
        cache_context=context(archive)
        pool=resources.enter_context(create_pool(relocated,source['contact_scene']['base_top_pelvis_m']))
        return _validate_range(selected,scaled,source,velocity,acceleration,model,parameters,
                               checker,pool,validate,check,progress,cache_context,use_collision_cache,collision_report)


def _validate_range(selected,scaled,source,velocity,acceleration,model,parameters,
                    checker_factory,pool,validate,check,progress,cache_context,use_collision_cache,collision_report):
    from task_collision_cache import certificate,matches
    result = []
    certificates=[];hits=[];checker=None
    for selected_index, record in enumerate(selected):
        check()
        original = record['original_segment']
        trajectory = JointTrajectory()
        set_message_fields(trajectory, copy.deepcopy(original['trajectory']))
        arm = record['kind'] == 'arm'
        if scaled is not None:
            trajectory = scaled[selected_index]['trajectory']
        elif arm:
            trajectory = speed_copy(trajectory, source['velocity_scaling'], velocity, acceleration,
                                    source['waist_assist'],parameters)
        vmax, amax = task_motion_limits(trajectory.joint_names,velocity,acceleration,parameters)
        limits = {}
        for name in trajectory.joint_names:
            limit = model.robot.find(f"joint[@name='{name}']/limit")
            if limit is None:
                raise ValueError('Missing joint limits: '+name)
            limits[name] = (float(limit.get('lower')), float(limit.get('upper')))
        if arm and 'waist_yaw_joint' in limits:
            lo,hi = limits['waist_yaw_joint']
            validate_waist_envelope(trajectory,lo+.02,hi-.02,vmax['waist_yaw_joint'],amax['waist_yaw_joint'])
        path = samples(trajectory,vmax,amax) if arm else (
            p.positions for p in trajectory.points)
        total = sample_count(trajectory) if arm else len(trajectory.points)
        segment=dict(number=record['number'], side=record['side'], label=record['label'],
                     kind=record['kind'], posture=original.get('posture'),
                     target_q=original.get('target_q'), trajectory=trajectory,
                     duration=time_seconds(trajectory.points[-1]),start=record['start'],end=record['end'])
        cert=certificate(cache_context,segment)
        cached=use_collision_cache and matches(cert)
        if cached:
            hits.append(record['number'])
            progress(f'Stage {record["number"]}: collision cache hit; checking motion limits')
        elif (not (arm and total>400) or pool is None) and checker is None:
            checker=checker_factory()
        validate(pool if arm and total>400 and not cached else None,None if cached else checker,path,record['start'],record['side'],
                 trajectory.joint_names,limits,.02 if arm else 0.,check,
                 lambda index:progress(f'Stage {record["number"]}: '+
                     ('limits only (cached collision)' if cached else 'checking (parallel)' if arm and total>400 else 'checking')+
                     f' {index}/{total}'))
        certificates.append(cert)
        result.append(segment)
    if collision_report is not None:
        collision_report.update(certificates=certificates,cached_stages=hits)
    return result
