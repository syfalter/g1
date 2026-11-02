"""Curled-index experiment restricted by the parent to offline domain 93."""
import math
import numpy as np
from scipy.spatial.transform import Rotation
from task_kinematic_preview import KinematicPreviewPlanner
from task_motion_planner import PlanningFailure
from task_hand_control import POSTURES, model_joints
from task_hook_collision import HandBaseCollision

HOOK=(1.,1.,1.,.3,1.,1.)


def hook_rotations():
    result=[]
    for tilt in (0.,10.,20.):
        for yaw in np.arange(0.,360.,30.):
            rotation=Rotation.from_euler('z',yaw,degrees=True)*Rotation.from_euler('y',180.-tilt,degrees=True)
            result.append(rotation.as_quat())
    return result


def arc_reference(nominal,center,entry,gain=.35):
    direction=np.asarray(entry)[:2]-np.asarray(center)[:2]
    def reference(point):
        offset=np.asarray(point)[:2]-np.asarray(center)[:2]
        angle=math.atan2(direction[0]*offset[1]-direction[1]*offset[0],float(direction@offset))
        return (Rotation.from_euler('z',gain*angle)*Rotation.from_quat(nominal)).as_quat()
    return reference


class HookPreviewPlanner(KinematicPreviewPlanner):
    def __init__(self,*args,preview_urdf,**kwargs):
        super().__init__(*args,**kwargs)
        if not self.articulated_hands:raise ValueError('Hook preview requires articulated hands')
        self.preview_urdf=preview_urdf;self.base_check=None
        # Registered only in this isolated process, not in hardware posture defaults.
        POSTURES['index_hook_open_thumb']=HOOK

    def install_surface(self,xyz,torso_transform,ticket,*,center,**kwargs):
        top=torso_transform[:3,:3]@np.asarray(center)+torso_transform[:3,3]
        self.base_check=HandBaseCollision(self.model,self.preview_urdf,top)
        self.scene_report={'source':'User supplied axis base cylinder','base_top_pelvis_m':top.tolist(),
            'base_radius_m':.025,'base_height_m':.02,'collision_scope':'selected hand, palm and connector vs base only',
            'collision_geometry':'conservative per-link convex hulls and exact cylinder',
            'checked_links':self.base_check.links,'self_collision_checked':False,
            'environment_collision_checked':False,'d_ring_modeled':False,'hardware_validated':False}

    def validate(self,q,ticket,side=None,stage=None):
        super().validate(q,ticket,side,stage)
        if self.base_check:
            link=self.base_check.check(q,side or ticket.side)
            if link:raise PlanningFailure('COLLISION',stage or ticket.kind,'Hand intersects axis base',link=link)

    def contact_hand_setup(self,side,start,geometry):
        state=dict(start);state.update(model_joints(side,HOOK))
        wrist=np.linalg.inv(self.model.transform(side+'_wrist_yaw_link',state))@self.model.transform(side+'_index_tcp',state)
        self.search_diagnostics['hook']={'actuator_q':list(HOOK),'wrist_from_tip':wrist.tolist(),
            'thumb_root':'fully open, non-opposed','model_calibrated':False,'arc_yaw_gain':.35}
        return 'index_hook_open_thumb',wrist

    def preinsert_rotations(self,nominal,policy):return hook_rotations()

    def cartesian_orientations(self,nominal,previous,policy):
        candidates=super().cartesian_orientations(nominal,previous,policy)
        # Follow the moving reference when feasible instead of freezing last orientation.
        return sorted(candidates,key=lambda q:(Rotation.from_quat(nominal).inv()*Rotation.from_quat(q)).magnitude())

    def contact_segments(self,side,q,pre,entry,center,nominal,wrist_from_tip,ticket,*,terminal=None):
        reference=arc_reference(nominal,center,entry)
        return super().contact_segments(side,q,pre,entry,center,reference,wrist_from_tip,ticket,terminal=terminal)
