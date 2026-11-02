"""Shared D-ring straight-index geometry policy for online and archived planning."""
import math
import numpy as np
from scipy.spatial.transform import Rotation
from task_hand_control import POSTURES,model_joints
from task_planning_config import PlanningParameters

RADIAL_OFFSET_M=.010
HEIGHT_OFFSET_M=.010
MAX_INDEX_TILT_DEG=45.
ARC_YAW_GAIN=.15


def straight_rotations(max_tilt):
    if not math.isfinite(max_tilt) or not 10<=max_tilt<=50:raise ValueError('Tilt must be in [10,50] degrees from downward')
    result=[]
    for tilt in (max_tilt-10,max_tilt,max_tilt-20):
        for yaw in np.arange(0.,360.,45.):
            base=Rotation.from_euler('z',yaw,degrees=True)*Rotation.from_euler('y',180-max(0.,tilt),degrees=True)
            for twist in (0.,90.,180.,270.):
                result.append((base*Rotation.from_euler('z',twist,degrees=True)).as_quat())
    return result


def arc_reference(nominal,center,entry,gain=ARC_YAW_GAIN):
    direction=np.asarray(entry)[:2]-np.asarray(center)[:2]
    def reference(point):
        offset=np.asarray(point)[:2]-np.asarray(center)[:2]
        angle=math.atan2(direction[0]*offset[1]-direction[1]*offset[0],float(direction@offset))
        return (Rotation.from_euler('z',gain*angle)*Rotation.from_quat(nominal)).as_quat()
    def batch(points):
        offsets=np.asarray(points)[:,:2]-np.asarray(center)[:2]
        angles=np.arctan2(direction[0]*offsets[:,1]-direction[1]*offsets[:,0],offsets@direction)
        return (Rotation.from_euler('z',gain*angles)*Rotation.from_quat(nominal)).as_quat()
    reference.batch=batch
    return reference


class StraightIndexPolicy:
    def tangent_mode(self):
        return (getattr(self,'planning_parameters',PlanningParameters()).hand_back_tangent_xy
                and not getattr(self,'center_check',False))

    def configure_contact_geometry(self,center,entry):
        self.tangent_center=np.asarray(center).copy()
        self.tangent_entry=np.asarray(entry).copy()
        self.tangent_branch=None
        if self.tangent_mode():
            if np.linalg.norm(self.tangent_entry[:2]-self.tangent_center[:2])<1e-6:
                raise ValueError('Undefined TCP-to-center XY direction')
            self.search_diagnostics['straight_index'].update(
                orientation_policy='physical_center_xy_tangent',tangent_tolerance_deg=5.,
                tangent_center_m=self.tangent_center.tolist(),arc_yaw_gain=1.)

    def tangent_errors(self,tips,*,any_branch=False):
        tips=np.asarray(tips)
        normals=tips[:,:3,:3]@self.dorsal_in_tip
        radial=tips[:,:2,3]-self.tangent_center[:2]
        tangent=np.column_stack((-radial[:,1],radial[:,0]))
        scale=np.linalg.norm(normals[:,:2],axis=1)*np.linalg.norm(radial,axis=1)
        valid=(np.linalg.norm(normals[:,:2],axis=1)>=1e-6)&(np.linalg.norm(radial,axis=1)>=1e-6)
        cosine=np.sum(normals[:,:2]*tangent,axis=1)/np.where(valid,scale,1.)
        branch=None if any_branch else getattr(self,'tangent_branch',None)
        cosine=np.abs(cosine) if branch is None else branch*cosine
        return np.where(valid,np.degrees(np.arccos(np.clip(cosine,-1.,1.))),np.inf)

    def contact_pose_ok(self,tip,stage):
        if not self.tangent_mode() or stage=='approach':return True
        margin=getattr(self,'planning_parameters',PlanningParameters()).planning_margin_deg
        return bool(self.tangent_errors([tip],any_branch=stage=='preinsert')[0]<=5.-margin)

    def tangent_reference(self,nominal,reference_point=None):
        normal=Rotation.from_quat(nominal).apply(self.dorsal_in_tip)
        radial=self.tangent_entry[:2]-self.tangent_center[:2]
        tangent=np.array([-radial[1],radial[0]])
        initial=radial if reference_point is None else np.asarray(reference_point)[:2]-self.tangent_center[:2]
        if np.linalg.norm(initial)<1e-6:raise ValueError('Undefined reference TCP-to-center XY direction')
        initial_tangent=np.array([-initial[1],initial[0]])
        if np.linalg.norm(normal[:2])<1e-6:raise ValueError('Undefined hand-back XY projection')
        self.tangent_branch=1. if normal[:2]@initial_tangent>=0 else -1.
        target=self.tangent_branch*tangent
        angle=math.atan2(target[1],target[0])-math.atan2(normal[1],normal[0])
        aligned=(Rotation.from_euler('z',angle)*Rotation.from_quat(nominal)).as_quat()
        return arc_reference(aligned,self.tangent_center,self.tangent_entry,gain=1.)

    def preinsert_trial_rotation(self,quaternion,position):
        if not self.tangent_mode():return super().preinsert_trial_rotation(quaternion,position)
        entry=self.tangent_entry[:2]-self.tangent_center[:2]
        radial=np.asarray(position)[:2]-self.tangent_center[:2]
        if np.linalg.norm(radial)<1e-6:return None
        tip=np.eye(4);tip[:3,3]=position;tip[:3,:3]=Rotation.from_quat(quaternion).as_matrix()
        if self.contact_pose_ok(tip,'preinsert'):return quaternion
        # The saved inclined approach ray need not have the entry point's XY bearing.
        angle=math.atan2(entry[0]*radial[1]-entry[1]*radial[0],float(entry@radial))
        return (Rotation.from_euler('z',angle)*Rotation.from_quat(quaternion)).as_quat()

    def solve(self,side,q,target,wrist_from_tip,ticket,stage='approach',timeout=.1,rpc_timeout=15.,*,
              continuity_reference=None):
        params=getattr(self,'planning_parameters',PlanningParameters())
        if (stage=='rotate' and getattr(self,'sweep_direction','left')=='right' and
                getattr(self,'waist_assist',False) and params.right_sweep_waist_bias):
            from task_ik_search import waist_biased_continuation
            from task_motion_planner import PlanningFailure
            report=self.search_diagnostics.setdefault('right_sweep_waist_bias',
                dict(policy='weighted_local_continuation_v1',attempted=0,accepted=0,fallback=0))
            report['attempted']+=1
            answer=waist_biased_continuation(self.names(side),self.joint_limits(side),q,target,
                                            lambda state:self.tip(side,state,wrist_from_tip))
            if answer is not None:
                try:self.validate(answer,ticket,side,stage)
                except PlanningFailure as error:
                    if error.diagnostic['code'] not in ('COLLISION','JOINT_MARGIN'):raise
                    answer=None
            if answer is not None:
                report['accepted']+=1
                self.last_ik=dict(code='WAIST_BIASED_IK',stage=stage)
                return answer
            report['fallback']+=1
        options={'continuity_reference':continuity_reference} if continuity_reference is not None else {}
        return super().solve(side,q,target,wrist_from_tip,ticket,stage,timeout,rpc_timeout,**options)

    def solve_constrained_contact(self,side,seed,previous,point,reference,wrist_from_tip,
                                  ticket,stage,policy,tolerance):
        if (ticket.kind!='task' or getattr(self,'reference_frame',None)!='pelvis' or
                not self.tangent_mode() or self.tangent_branch not in (-1.,1.)):
            raise ValueError('Constrained continuation requires a fixed physical tangent task')
        from task_contact_ik import constrained_contact_ik
        from task_motion_planner import PlanningFailure
        params=getattr(self,'planning_parameters',PlanningParameters())
        def check():
            self.check_plan(ticket)
            self.check_contact_search_budget()
        answer,diagnostic=constrained_contact_ik(self.model,side+'_wrist_yaw_link',self.names(side),
            self.joint_limits(side),seed,previous,point,reference,wrist_from_tip,
            self.tangent_center,self.dorsal_in_tip,self.tangent_branch,policy,check,.1,
            position_tolerance=min(.002,tolerance),tangent_limit_deg=5.-params.planning_margin_deg)
        self.last_ik=dict(diagnostic,stage=stage)
        if answer is None:return None
        try:self.validate(answer,ticket,side,stage)
        except PlanningFailure as error:
            if error.diagnostic['code'] not in ('COLLISION','JOINT_MARGIN'):raise
            self.last_ik.update(error.diagnostic)
            return None
        self.last_ik['code']='VALID'
        return answer

    def configure_task(self,side,startup,ready,direction):
        if direction not in ('left','right'):raise ValueError('Unknown D-ring sweep direction')
        if startup is None or ready is None:raise ValueError('Startup and above-table pose required')
        forward=self.model.transform('torso_link',startup)[:3,0]
        if np.linalg.norm(forward[:2])<1e-6:raise ValueError('Undefined startup forward heading')
        self.fixed_forward_yaw=math.atan2(forward[1],forward[0])
        self.sweep_direction=direction
        self.ready_target=dict(ready)

    def contact_hand_setup(self,side,start,geometry):
        self.center_check=bool(geometry and geometry.get('center_check'))
        posture_name='index_only' if self.center_check else 'dring_two_finger'
        from task_hand_control import posture_targets
        posture=posture_targets(posture_name,getattr(self,'dring_hand_settings',None))
        state=dict(start);state.update(model_joints(side,posture))
        # Keep the commissioned straight-index TCP despite the bent-finger posture.
        straight=dict(start);straight.update(model_joints(side,POSTURES['index_only']))
        wrist=np.linalg.inv(self.model.transform(side+'_wrist_yaw_link',straight))@self.model.transform(side+'_index_tcp',straight)
        if geometry and geometry.get('calibration'):
            from task_calibration import validate
            values=validate(geometry['calibration']['values'])
            if values['a_enabled']:
                wrist[:3,3]=[values[f'{side}_{axis}_mm']/1000 for axis in 'xyz']
        # RH56DFX index flexes towards +X in left hand_base, -X in right.
        # The dorsal normal is opposite that flexion direction, not the TCP Z axis.
        if self.model.robot.find(f"link[@name='{side}_hand_base']") is not None:
            base=self.model.transform(side+'_hand_base',state)
        else:
            import json
            from task_hand_model import ROOT
            metadata=json.loads((ROOT/'assets/workbench/rh56dfx/task/provenance.json').read_text())
            base=self.model.transform(side+'_rubber_hand',state)@np.asarray(metadata['hands'][side]['mount_transform'])
        tcp=self.model.transform(side+'_wrist_yaw_link',state)@wrist
        self.dorsal_in_tip=tcp[:3,:3].T@base[:3,:3]@np.array([-1. if side=='left' else 1.,0.,0.])
        if not hasattr(self,'fixed_forward_yaw'):
            forward=self.model.transform('torso_link',start)[:3,0]
            self.fixed_forward_yaw=math.atan2(forward[1],forward[0])
        params=getattr(self,'planning_parameters',PlanningParameters())
        self.search_diagnostics['straight_index']={'actuator_q':list(posture),'wrist_from_tip':wrist.tolist(),
            'nominal_max_tilt_from_down_deg':self.max_index_tilt,'local_tilt_tolerance_deg':params.tilt_deg,
            'local_twist_tolerance_deg':params.twist_deg,'model_calibrated':False,'arc_yaw_gain':0.,
            'dorsal_normal_in_tcp':self.dorsal_in_tip.tolist(),
            'fixed_forward_yaw_deg':math.degrees(self.fixed_forward_yaw),
            'tcp_posture_reference':'straight_index_no_bend_compensation',
            'preferred_hand_back_yaw_deg':self.preferred_hand_back_yaw(),
            'directed_hand_back_bounds_deg':self.directed_heading_bounds(),
            'hand_back_yaw_limit_deg':params.hand_back_yaw_deg}
        return posture_name,wrist

    def preferred_hand_back_yaw(self):
        if getattr(self,'center_check',False) or self.tangent_mode():return 0.
        params=getattr(self,'planning_parameters',PlanningParameters())
        direction=getattr(self,'sweep_direction',None)
        return getattr(params,direction+'_sweep_hand_back_yaw_deg') if direction in ('left','right') else 0.

    def preferred_heading_error(self,quaternion):
        normal=Rotation.from_quat(quaternion).apply(self.dorsal_in_tip)
        target=self.fixed_forward_yaw+math.radians(self.preferred_hand_back_yaw())
        return abs(math.degrees((math.atan2(normal[1],normal[0])-target+math.pi)%(2*math.pi)-math.pi))

    def heading_error(self,quaternions):
        normals=Rotation.from_quat(quaternions).apply(self.dorsal_in_tip)
        yaw=np.arctan2(normals[...,1],normals[...,0])-self.fixed_forward_yaw
        error=np.abs(np.degrees((yaw+math.pi)%(2*math.pi)-math.pi))
        return np.where(np.linalg.norm(normals[...,:2],axis=-1)<1e-6,np.inf,error)

    def directed_heading_bounds(self):
        if self.tangent_mode() or getattr(self,'center_check',False) or getattr(self,'sweep_direction',None) not in ('left','right'):
            return None
        params=getattr(self,'planning_parameters',PlanningParameters())
        if params.sweep_hand_back_max_deg==0:return None
        return (params.sweep_hand_back_min_deg,params.sweep_hand_back_max_deg)

    def directed_heading(self,quaternions):
        normals=Rotation.from_quat(quaternions).apply(self.dorsal_in_tip)
        yaw=np.arctan2(normals[...,1],normals[...,0])-self.fixed_forward_yaw
        signed=np.degrees((yaw+math.pi)%(2*math.pi)-math.pi)
        directed=(-1 if self.sweep_direction=='left' else 1)*signed
        return np.where(np.linalg.norm(normals[...,:2],axis=-1)<1e-6,np.nan,directed)

    def heading_allowed(self,quaternions):
        if self.tangent_mode():return np.ones(np.asarray(quaternions).shape[:-1],dtype=bool)
        params=getattr(self,'planning_parameters',PlanningParameters())
        allowed=self.heading_error(quaternions)<=params.hand_back_yaw_deg-params.planning_margin_deg
        bounds=self.directed_heading_bounds()
        if bounds:
            directed=self.directed_heading(quaternions)
            allowed=allowed & (directed>bounds[0]+params.planning_margin_deg) & (directed<bounds[1]-params.planning_margin_deg)
        return allowed

    def preinsert_rotations(self,nominal,policy):
        rotations=straight_rotations(self.max_index_tilt)
        if self.tangent_mode():
            radial=self.tangent_entry[:2]-self.tangent_center[:2]
            yaw=math.atan2(radial[1],radial[0])+math.pi/2
            candidates=[]
            for q in rotations:
                normal=Rotation.from_quat(q).apply(self.dorsal_in_tip)
                if np.linalg.norm(normal[:2])<1e-6:continue
                for branch in (0.,math.pi):
                    shifted=(Rotation.from_euler('z',yaw+branch-math.atan2(normal[1],normal[0]))*Rotation.from_quat(q)).as_quat()
                    if not any(abs(float(shifted@other))>1-1e-10 for other in candidates):candidates.append(shifted)
            return candidates
        bias=self.preferred_hand_back_yaw()
        if bias:
            preferred=[]
            biases=[bias]
            bounds=self.directed_heading_bounds()
            if bounds:
                sign=-1 if self.sweep_direction=='left' else 1
                biases.extend(sign*(bounds[0]+fraction*(bounds[1]-bounds[0])) for fraction in (.25,.75))
            for candidate_bias in biases:
                for q in rotations:
                    normal=Rotation.from_quat(q).apply(self.dorsal_in_tip)
                    if np.linalg.norm(normal[:2])<1e-6:continue
                    yaw=self.fixed_forward_yaw+math.radians(candidate_bias)-math.atan2(normal[1],normal[0])
                    shifted=(Rotation.from_euler('z',yaw)*Rotation.from_quat(q)).as_quat()
                    if self.heading_allowed(shifted) and not any(abs(float(shifted@other))>1-1e-10 for other in preferred):
                        preferred.append(shifted)
            fallback=sorted((q for q in rotations if self.heading_allowed(q)),key=self.preferred_heading_error)
            return preferred+fallback
        return sorted((q for q in rotations if self.heading_allowed(q)),key=lambda q:float(self.heading_error(q)))

    def preinsert_score(self,side,start,answer,previous,actual):
        score=super().preinsert_score(side,start,answer,previous,actual)
        if score is not None and self.preferred_hand_back_yaw():
            penalty=.2*math.radians(self.preferred_heading_error(actual))**2
            score=dict(score,total=score['total']+penalty,hand_back_heading_penalty=penalty)
        return score

    def orientation_ok(self,nominal,actual,policy):
        return super().orientation_ok(nominal,actual,policy) and bool(self.heading_allowed(actual))

    def cartesian_orientations(self,nominal,previous,policy):
        candidates=super().cartesian_orientations(nominal,previous,policy)
        return [q for q in candidates if self.heading_allowed(q)]

    def heading_failure(self,tips,stage,label,report):
        from task_motion_planner import PlanningFailure
        if stage=='approach':return None
        if self.tangent_mode():
            errors=self.tangent_errors(tips)
            report['max_hand_back_tangent_error_deg']=max(report.get('max_hand_back_tangent_error_deg',0.),float(np.max(errors)))
            failed=np.flatnonzero(errors>5.+1e-6)
            if len(failed):
                index=int(failed[0])
                return PlanningFailure('RETIMED_TCP_BOUNDS',stage,label+': hand-back XY tangent exceeded',
                    tangent_error_deg=float(errors[index]),tangent_limit_deg=5.,
                    tcp_position_m=np.asarray(tips)[index,:3,3].tolist(),batch_sample_index=index)
            return None
        params=getattr(self,'planning_parameters',PlanningParameters())
        quaternions=Rotation.from_matrix(np.asarray(tips)[:,:3,:3]).as_quat()
        bounds=self.directed_heading_bounds()
        if bounds:
            directed=self.directed_heading(quaternions)
            finite=directed[np.isfinite(directed)]
            if len(finite):
                report['min_directed_hand_back_yaw_deg']=min(report.get('min_directed_hand_back_yaw_deg',math.inf),float(np.min(finite)))
                report['max_directed_hand_back_yaw_deg']=max(report.get('max_directed_hand_back_yaw_deg',-math.inf),float(np.max(finite)))
            failed=np.flatnonzero(~np.isfinite(directed)|(directed<=bounds[0]+1e-6)|(directed>=bounds[1]-1e-6))
            if len(failed):
                index=int(failed[0])
                return PlanningFailure('RETIMED_TCP_BOUNDS',stage,label+': opposite-sweep hand-back yaw outside strict range',
                    directed_hand_back_yaw_deg=float(directed[index]) if np.isfinite(directed[index]) else None,
                    directed_hand_back_bounds_deg=bounds,tcp_position_m=np.asarray(tips)[index,:3,3].tolist(),
                    batch_sample_index=index)
        errors=self.heading_error(quaternions)
        report['max_hand_back_yaw_deg']=max(report.get('max_hand_back_yaw_deg',0.),float(np.max(errors)))
        failed=np.flatnonzero(errors>params.hand_back_yaw_deg+1e-4)
        if len(failed):
            index=int(failed[0])
            return PlanningFailure('RETIMED_TCP_BOUNDS',stage,label+': fixed hand-back heading exceeded',
                hand_back_yaw_deg=float(errors[index]),hand_back_yaw_limit_deg=params.hand_back_yaw_deg,
                tcp_position_m=np.asarray(tips)[index,:3,3].tolist(),batch_sample_index=index)

    def check_tcp_geometry(self,side,state,wrist_from_tip,nominal,curve,policy,stage,label,report):
        error=self.heading_failure([self.tip(side,state,wrist_from_tip)],stage,label,report)
        if error:raise error
        return super().check_tcp_geometry(side,state,wrist_from_tip,nominal,curve,policy,stage,label,report)

    def check_tcp_geometry_batch(self,tips,nominal,curve,policy,stage,label,report):
        error=self.heading_failure(tips,stage,label,report)
        return error or super().check_tcp_geometry_batch(tips,nominal,curve,policy,stage,label,report)

    def contact_segments(self,side,q,pre,entry,center,nominal,wrist_from_tip,ticket,*,terminal=None):
        reference=self.tangent_reference(nominal,pre[:3,3]) if self.tangent_mode() else nominal
        return super().contact_segments(side,q,pre,entry,center,nominal,wrist_from_tip,ticket,
                                        arc_nominal=reference,terminal=terminal)
