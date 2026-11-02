"""Noncontact shaft-center pointing task and nominal transform-chain evidence."""
from dataclasses import replace
from itertools import islice
import numpy as np
from scipy.spatial.transform import Rotation

from task_hand_control import POSTURES, model_joints
from task_motion_planner import PlanningFailure
from task_kinematics import capture_transform
from task_dring_sequence import CENTER_SCHEMA as SCHEMA,CENTER_LABELS as LABELS,is_center_check


def target_geometry(geometry, model, start, height_mm):
    frozen=capture_transform(geometry,model,start)
    center=frozen[:3,:3]@np.asarray(geometry['task_preview']['axis_center_m'])+frozen[:3,3]
    target=center+np.array([0.,0.,height_mm/1000.])
    return dict(frame='pelvis',center_pelvis_m=center.tolist(),target_pelvis_m=target.tolist(),
        height_mm=height_mm,operation_offsets_applied=False,
        pelvis_from_capture_torso=frozen.tolist(),
        torso_from_color_optical=geometry['transforms']['torso_from_color_optical_row_major'])


def plan_center_check(planner, side, start, geometry, ticket):
    original=planner.planning_parameters
    # This measurement target is not the loose D-ring contact path.
    planner.planning_parameters=replace(original,path_tolerance_mm=2.,target_tolerance_mm=2.,
        position_search_fraction=0.,smooth_insert_path=True,smooth_joint_path=True)
    planner.search_diagnostics={}
    try:
        evidence=target_geometry(geometry,planner.model,start,original.center_check_height_mm)
        if geometry.get('calibration'):evidence['calibration']=geometry['calibration']
        evidence.update(path_tolerance_mm=2.,target_tolerance_mm=2.,camera_mount_calibrated=False,
                        hand_geometry_calibrated=False,approach='vertical_above_center')
        geometry['center_check']=evidence
        target=np.asarray(evidence['target_pelvis_m'])
        posture,wrist=planner.contact_hand_setup(side,start,geometry)
        evidence['wrist_from_index_tcp']=wrist.tolist()
        q=dict(start);q.update(model_joints(side,POSTURES[posture]))
        pre=np.eye(4);pre[:3,3]=target+[0.,0.,.10]
        pre[:3,:3]=Rotation.from_euler('y',180,degrees=True).as_matrix()
        batches=planner.preinsert_batches(side,q,pre,target,wrist,ticket,vertical=True)
        candidates=(candidate for batch in batches for candidate in batch)
        failures=[]
        try:
            for candidate in islice(candidates,original.complete_candidates):
                try:
                    hand=planner.hand_transition(side,start,posture,ticket)
                    approach=planner.joint_path(side,hand['end'],candidate['goal'],ticket,
                        'Center approach',stage='approach')
                    initial=planner.tip(side,approach['end'],wrist)[:3,3]
                    delta=target-initial
                    def curve(p):return float(curve.batch(np.asarray(p)[None,:])[0])
                    curve.batch=lambda p:np.linalg.norm(p-(initial+np.clip((p-initial)@delta/(delta@delta),0,1)[:,None]*delta),axis=1)
                    end=planner.cartesian(side,approach['end'],np.linspace(initial,target,original.line_points)[1:],
                        candidate['nominal'],wrist,ticket,'Center check',curve,'insert')
                except PlanningFailure as error:
                    if error.diagnostic['code'] not in ('COLLISION','JOINT_MARGIN','CONTINUOUS_IK_FAILED',
                                                       'MOTION_PLAN_FAILED','RETIMED_TCP_BOUNDS'):raise
                    failures.append(error.diagnostic);continue
                actual=planner.tip(side,end['end'],wrist)
                evidence.update(planned_tcp_pelvis=actual.tolist(),
                    planned_residual_mm=float(np.linalg.norm(actual[:3,3]-target)*1000.),
                    candidate_failures=failures,model_calibrated=False)
                planner.search_diagnostics['center_check']=evidence
                return [hand,approach,end]
            raise PlanningFailure('APPROACH_PATH_FAILED','center_check',
                                  'No complete center-check approach',failures=failures)
        finally:
            candidates.close();batches.close()
    finally:planner.planning_parameters=original


def measurement(model, side, joints, evidence):
    wrist=model.transform(side+'_wrist_yaw_link',joints)
    tcp=model.transform(side+'_index_tcp',joints)
    torso=model.transform('torso_link',joints)
    nominal=wrist@np.asarray(evidence['wrist_from_index_tcp'])
    target=np.asarray(evidence['target_pelvis_m'])
    return dict(target_pelvis_m=target.tolist(),measured_joints=dict(joints),
        pelvis_from_torso=torso.tolist(),pelvis_from_wrist=wrist.tolist(),
        wrist_from_tcp_model=(np.linalg.inv(wrist)@tcp).tolist(),
        measured_fk_tcp_pelvis=nominal.tolist(),original_model_fk_tcp_pelvis=tcp.tolist(),
        nominal_posture_tcp_pelvis=nominal.tolist(),
        fk_error_mm=((nominal[:3,3]-target)*1000.).tolist(),
        original_model_fk_error_mm=((tcp[:3,3]-target)*1000.).tolist(),
        calibration=evidence.get('calibration'),
        nominal_posture_error_mm=((nominal[:3,3]-target)*1000.).tolist(),
        physical_tip_observed=False,calibration_identified=False,
        note='Joint-feedback FK is not an independent optical/contact measurement of the physical fingertip')
