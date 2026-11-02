"""Verified archive catalog and visualization adapters. No control or IK clients."""
import json
import copy
import math
from pathlib import Path

from task_plan_archive import write_json
from task_saved_test import load_archive
from task_model_resources import archived_model_tree

ROOT=Path(__file__).resolve().parents[1]
DEFAULT_LIBRARY=ROOT/'data/offline_trajectory_library'


def describe(directory):
    directory=Path(directory).resolve()
    archive=load_archive(directory,directory/'context/robot.urdf')
    plan=archive['plan']
    provenance=directory/'plan/offline_provenance.json'
    provenance=json.loads(provenance.read_text()) if provenance.is_file() else {}
    validation=directory/'plan/offline_validation.json'
    validation=json.loads(validation.read_text()) if validation.is_file() else {}
    geometry=[s.get('geometry_validation') for s in plan['segments'] if s.get('stage') in ('insert','rotate','withdraw')]
    return dict(directory=str(directory),arm=plan['arm'],direction=plan['sweep_direction'],
                pair_id=provenance.get('pair_id'),
                source_capture=provenance.get('source_capture',archive['manifest']['source_plan_directory']),
                saved_at=archive['manifest'].get('saved_at',''),
                model_sha256=archive['manifest']['sha256']['context/robot.urdf'],
                start=dict(plan['start']),waist_degrees=math.degrees(plan['start'].get('waist_yaw_joint',0.)),
                trajectory_duration_s=sum(s['duration_s'] for s in archive['stages']),
                planning_seconds=validation.get('planning_seconds',validation.get('planning_seconds_at_export',plan.get('planning_seconds'))),
                max_position_error_m=(max(s['max_position_error_m'] for s in geometry)
                                      if len(geometry)==3 and all(geometry) else None),
                stage_count=len(archive['stages']),hardware_executable=False)


def catalog(root=DEFAULT_LIBRARY):
    root=Path(root)
    entries=[];errors=[]
    if not root.exists():return entries,errors
    for manifest in sorted(root.rglob('archive_manifest.json')):
        if any(part.startswith('.') for part in manifest.relative_to(root).parts):continue
        try:entries.append(describe(manifest.parent))
        except (ValueError,KeyError,OSError,TypeError) as error:
            errors.append(dict(directory=str(manifest.parent),error=str(error)))
    entries.sort(key=lambda entry:entry['saved_at'],reverse=True)
    return entries,errors


def playback_copy(directory,output,*,speed_multiplier=1.,velocity=1.,acceleration=1.,
                  scene_capture=None,scene_model=None,timing_policy='guarded',motion_parameters=None,hand_settings=None):
    """Relocate mesh URIs only in a visualization copy; preserve source URDF/hash."""
    if (timing_policy not in ('guarded','preview') or
            not math.isfinite(speed_multiplier) or speed_multiplier <= 0 or
            any(not math.isfinite(v) or not .05 <= v <= 1. for v in (velocity,acceleration))):
        raise ValueError('Invalid trajectory multiplier or motion-limit scaling')
    directory=Path(directory).resolve();output=Path(output)
    archive=load_archive(directory,scene_model or directory/'context/robot.urdf')
    if hand_settings is not None:
        from task_dring_hand_settings import archive_variant
        archive=archive_variant(archive,hand_settings)
    plan=archive['plan'];output.mkdir(parents=True,exist_ok=False)
    tree,mesh_hashes=archived_model_tree(directory)
    tree.write(output/'robot.urdf',encoding='unicode')
    report=dict(mode='ARCHIVE_SCOPED_OFFLINE',offline_only=True,success=True,
                hardware_executable=False,hardware_commands_sent=0,
                capture=str(directory/'plan'),arm=plan['arm'],sweep_direction=plan['sweep_direction'],
                state_note='Archived baseline; simulated visualization only, not live robot feedback',
                base_collision_checked=True,arm_body_collision_checked=True,
                full_self_collision_checked=False,environment_collision_checked=False,
                scene=plan['contact_scene'],segments=plan['segments'],archive=str(directory),
                source_model_sha256=archive['manifest']['sha256']['context/robot.urdf'],
                source_mesh_sha256=mesh_hashes)
    report['dring_hand_settings']=plan.get('dring_hand_settings')
    if timing_policy in ('preview','guarded'):
        from rosidl_runtime_py.set_message import set_message_fields
        from rosidl_runtime_py.convert import message_to_ordereddict
        from trajectory_msgs.msg import JointTrajectory
        from workbench_model import OfflineModel
        from task_planning_config import parameters_from_values
        from task_trajectory_speed import scale_arm_segments,preview_segments
        segments=[]
        for segment in plan['segments']:
            trajectory=JointTrajectory()
            set_message_fields(trajectory,copy.deepcopy(segment['trajectory']))
            segments.append(dict(segment,trajectory=trajectory))
        values=motion_parameters if motion_parameters is not None else plan.get('planning_config',{}).get('values',{})
        parameters=parameters_from_values(values)
        report['timing_policy']=timing_policy
        report['motion_parameters']=values
        report['velocity_limit_scaling']=velocity
        report['acceleration_limit_scaling']=acceleration
        if timing_policy == 'preview':
            scaled,record=preview_segments(segments,speed_multiplier)
        else:
            scaled,record=scale_arm_segments(segments,speed_multiplier,OfflineModel(output/'robot.urdf'),
                parameters,velocity=velocity,acceleration=acceleration)
        report['segments']=[dict(segment,trajectory=message_to_ordereddict(segment['trajectory'])) for segment in scaled]
        report['speed_scaling']=record
    if scene_capture is not None:
        from task_archive_scene import snapshot_overlay
        from workbench_model import OfflineModel
        archived_geometry=json.loads((directory/'plan/geometry.json').read_text())
        report['scene_overlay']=snapshot_overlay(scene_capture,output/'scene_snapshot',plan,
                                                 archived_geometry,OfflineModel(output/'robot.urdf'))
        report['state_note']='Archived joint path in captured RGB-D; overlay only, NOT current-scene motion validation'
    write_json(output/'reconstructed_state.json',dict(joints=plan['start']))
    write_json(output/'report.json',report)
    return output/'report.json',report


def require_compatible_models(reports):
    first=reports[0][1]
    for _,report in reports[1:]:
        if any(report.get(key)!=first.get(key) for key in ('source_model_sha256','source_mesh_sha256')):
            raise ValueError('Gallery model or mesh mismatch')
