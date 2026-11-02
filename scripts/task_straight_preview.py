"""Isolated archive adapter for the same steep-straight planner used online."""
import os
from task_scoped_planner import ScopedTaskPlanner
from task_straight_policy import straight_rotations


class StraightPreviewPlanner(ScopedTaskPlanner):
    def __init__(self,*args,preview_urdf,check_preview,**kwargs):
        if (os.environ.get('ROS_DOMAIN_ID')!='93' or os.environ.get('ROS_LOCALHOST_ONLY')!='1'
                or os.environ.get('RMW_IMPLEMENTATION')!='rmw_fastrtps_cpp'):
            raise RuntimeError('Straight-index archive planning is offline domain 93 only')
        super().__init__(*args,robot_urdf=preview_urdf,check_plan=lambda ticket:check_preview(),**kwargs)
