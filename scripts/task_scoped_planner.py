"""Online D-ring preview integration. Prepare/recover retain the original validator.

Local base/body checks are not full collision validation or hardware commissioning.
This module never publishes trajectories or sends a hardware command.
"""
import math
import numpy as np
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from task_motion_planner import TaskPlanner,PlanningFailure
from task_straight_policy import StraightIndexPolicy,straight_rotations,MAX_INDEX_TILT_DEG
from task_hook_collision import HandBaseCollision


class ScopedTaskPlanner(StraightIndexPolicy,TaskPlanner):
    task_hardware_executable=False

    def __init__(self,*args,robot_urdf,check_plan,max_index_tilt=MAX_INDEX_TILT_DEG,**kwargs):
        super().__init__(*args,**kwargs)
        if not self.articulated_hands:raise ValueError('Steep straight-index profile requires articulated hands')
        self.max_index_tilt=max_index_tilt;straight_rotations(max_index_tilt)
        self.robot_urdf=robot_urdf;self.check_plan=check_plan;self.base_check=None
        self.scoped_generation=None
        self.scoped_limits={side:self.joint_limits(side) for side in ('left','right')}

    def install_surface(self,xyz,torso_transform,ticket,*,center=None,**kwargs):
        self.base_check=None;self.scoped_generation=None
        if ticket.kind!='task':return super().install_surface(xyz,torso_transform,ticket,center=center,**kwargs)
        self.check_plan(ticket)
        top=torso_transform[:3,:3]@np.asarray(center)+torso_transform[:3,3]
        self.base_check=HandBaseCollision(self.model,self.robot_urdf,top)
        self.scoped_generation=ticket.generation
        self.scene_report={'source':'User supplied axis base cylinder','base_top_pelvis_m':top.tolist(),
            'base_radius_m':.025,'base_height_m':.02,
            'collision_scope':'selected hand/palm/connector vs base; selected upper arm/elbow/forearm vs torso/pelvis/head',
            'collision_geometry':'conservative per-link convex hulls and exact cylinder',
            'checked_links':self.base_check.links,'arm_body_collision_checked':True,
            'arm_body_links':[s[0] for s in self.base_check.arm_shapes+self.base_check.body_shapes],
            'self_collision_checked':False,'full_self_collision_checked':False,
            'environment_collision_checked':False,'d_ring_modeled':False,'hardware_validated':False,
            'profile':'steep_straight','task_hardware_executable':False}

    def ik_avoid_collisions(self,ticket,stage):
        return False if ticket.kind=='task' else super().ik_avoid_collisions(ticket,stage)

    def validate(self,q,ticket,side=None,stage=None):
        if ticket.kind!='task':return super().validate(q,ticket,side,stage)
        self.check_plan(ticket)
        if self.base_check is None or self.scoped_generation!=ticket.generation:
            raise PlanningFailure('SCENE_NOT_READY',stage or ticket.kind,'Fresh scoped target required')
        if not all(math.isfinite(v) for v in q.values()):raise ValueError('Nonfinite planned state')
        side=side or ticket.side
        for name,(lo,hi) in self.scoped_limits[side].items():
            if not lo+.02<=q[name]<=hi-.02:
                raise PlanningFailure('JOINT_MARGIN',stage or ticket.kind,'Joint limit: '+name,
                    joint=name,position_rad=float(q[name]),model_lower_rad=float(lo),model_upper_rad=float(hi),
                    guarded_lower_rad=float(lo+.02),guarded_upper_rad=float(hi-.02))
        link,pair=self.base_check.check_state(q,side)
        if link:raise PlanningFailure('COLLISION',stage or ticket.kind,'Hand intersects axis base',link=link)
        if pair:raise PlanningFailure('COLLISION',stage or ticket.kind,'Arm intersects robot body',pair=pair)

    def solve(self,*args,**kwargs):
        try:return super().solve(*args,**kwargs)
        except PlanningFailure as error:
            if error.diagnostic['code']!='JOINT_MARGIN':raise
            self.last_ik=error.diagnostic;return None

    def validate_trajectory_samples(self,q,ticket,side,stage,joint_names,positions,geometry_check,progress):
        self.check_plan(ticket)
        if self.base_check is None or self.scoped_generation!=ticket.generation:
            raise PlanningFailure('SCENE_NOT_READY',stage or ticket.kind,'Fresh scoped target required')
        collision,wrists=self.base_check.check_batch(q,side,joint_names,positions,progress,
                                                     lambda:self.check_plan(ticket))
        geometry_error=None
        if geometry_check is not None:
            tips=np.matmul(wrists,geometry_check.wrist_from_tip)
            geometry_error=geometry_check.batch(tips)
        collision_index=collision['sample_index'] if collision is not None else math.inf
        geometry_index=(geometry_error.diagnostic['batch_sample_index']
                        if geometry_error is not None else math.inf)
        # Scalar validation checks collision before TCP geometry at each sample.
        if geometry_error is not None and geometry_index<collision_index:raise geometry_error
        if collision is not None:
            if collision['kind']=='hand':
                raise PlanningFailure('COLLISION',stage or ticket.kind,'Hand intersects axis base',
                                      link=collision['link'],batch_sample_index=collision_index)
            raise PlanningFailure('COLLISION',stage or ticket.kind,'Arm intersects robot body',
                                  pair=collision['pair'],batch_sample_index=collision_index)
        self.check_plan(ticket)

    def joint_path(self,side,q,target,ticket,label,stage=None):
        if ticket.kind!='task':return super().joint_path(side,q,target,ticket,label,stage)
        goal=dict(q);goal.update({name:target[name] for name in self.names(side)})
        self.validate(q,ticket,side,stage);self.validate(goal,ticket,side,stage)
        try:return self.scoped_joint_line(side,q,goal,ticket,label,stage)
        except PlanningFailure as error:
            if (stage!='return_ready' or getattr(self,'joint_route_backend','line')!='ompl' or
                    error.diagnostic['code']!='COLLISION'):raise
            from task_joint_route import return_route
            return return_route(self,side,q,goal,ticket,label,error.diagnostic)

    def scoped_joint_line(self,side,q,goal,ticket,label,stage):
        path=RobotTrajectory();path.joint_trajectory.joint_names=list(self.names(side))
        from task_planning_config import PlanningParameters
        if getattr(self,'planning_parameters',PlanningParameters()).smooth_joint_path:
            from task_path_smoothing import joint_line_trajectory
            trajectory=joint_line_trajectory(self.names(side),[q[n] for n in self.names(side)],
                [goal[n] for n in self.names(side)])
            return self.retime(side,q,path,ticket,label,stage=stage,timed_trajectory=trajectory)
        count=max(2,math.ceil(max(abs(goal[n]-q[n]) for n in self.names(side))/.04)+1)
        for fraction in np.linspace(0,1,count):
            path.joint_trajectory.points.append(JointTrajectoryPoint(
                positions=[float(q[n]+fraction*(goal[n]-q[n])) for n in self.names(side)]))
        return self.retime(side,q,path,ticket,label,stage=stage)
