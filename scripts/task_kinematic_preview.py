"""Collision-unchecked kinematic experiments, restricted to offline ROS domain 93."""
import os
import math
import numpy as np
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from task_motion_planner import TaskPlanner, PlanningFailure


class KinematicPreviewPlanner(TaskPlanner):
    def __init__(self,node,model,call,*args,check_preview,**kwargs):
        if (os.environ.get('ROS_DOMAIN_ID')!='93' or os.environ.get('ROS_LOCALHOST_ONLY')!='1'
                or os.environ.get('RMW_IMPLEMENTATION')!='rmw_fastrtps_cpp'):
            raise RuntimeError('Collision-unchecked planning is offline domain 93 only')
        self.check_preview=check_preview
        def preview_call(client,request,*args,**kwargs):
            if hasattr(request,'ik_request'):request.ik_request.avoid_collisions=False
            return call(client,request,*args,**kwargs)
        super().__init__(node,model,preview_call,*args,**kwargs)
        self.preview_limits={side:self.joint_limits(side) for side in ('left','right')}
        self.unchecked_states=0

    def install_surface(self,*args,**kwargs):
        self.scene_report={'source':'archived RGB-D, visualization only',
                           'collision_checked':False,'hardware_validated':False,
                           'self_collision_checked':False,'environment_collision_checked':False}

    def solve(self,*args,**kwargs):
        try:return super().solve(*args,**kwargs)
        except PlanningFailure as error:
            if error.diagnostic['code']!='JOINT_MARGIN':raise
            self.last_ik=error.diagnostic
            return None

    def validate(self,q,ticket,side=None,stage=None):
        self.check_preview()
        self.unchecked_states+=1
        if not all(math.isfinite(v) for v in q.values()):raise ValueError('Nonfinite preview state')
        for name,(lo,hi) in self.preview_limits[side or ticket.side].items():
            if not lo+.02<=q[name]<=hi-.02:
                raise PlanningFailure('JOINT_MARGIN',stage or ticket.kind,'Preview joint limit: '+name)

    def joint_path(self,side,q,target,ticket,label,stage=None):
        goal=dict(q)
        goal.update({name:target[name] for name in self.names(side)})
        self.validate(q,ticket,side,stage);self.validate(goal,ticket,side,stage)
        path=RobotTrajectory();path.joint_trajectory.joint_names=list(self.names(side))
        count=max(2,math.ceil(max(abs(goal[n]-q[n]) for n in self.names(side))/.04)+1)
        for fraction in np.linspace(0,1,count):
            path.joint_trajectory.points.append(JointTrajectoryPoint(
                positions=[float(q[n]+fraction*(goal[n]-q[n])) for n in self.names(side)]))
        return self.retime(side,q,path,ticket,label,stage=stage)
