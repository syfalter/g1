"""Refit archived D-ring paths with actual FK/FCL, without initializing ROS/DDS."""
import argparse
import copy
import json
import math
from pathlib import Path
from types import SimpleNamespace

import numpy as np
from moveit_msgs.msg import RobotTrajectory
from rosidl_runtime_py.convert import message_to_ordereddict
from rosidl_runtime_py.set_message import set_message_fields
from scipy.spatial.transform import Rotation
from trajectory_msgs.msg import JointTrajectory

from task_hook_collision import HandBaseCollision
from task_kinematics import arm_names
from task_planning_config import load_parameters,configure_planner
from task_scoped_planner import ScopedTaskPlanner
from task_session import Ticket
from task_trajectory import coefficients,peak,time_seconds,samples,task_motion_limits
from workbench_model import OfflineModel


def metrics(trajectory):
    # Historical artifacts used .6/.4; this is analysis, not execution admission.
    vmax={n:.4 if n=='waist_yaw_joint' else .6 for n in trajectory.joint_names}
    amax=dict.fromkeys(trajectory.joint_names,1.5)
    sampled=np.asarray(list(samples(trajectory,vmax,amax)))
    reversals={}
    for i,name in enumerate(trajectory.joint_names):
        delta=np.diff(sampled[:,i]);sign=np.sign(delta[np.abs(delta)>1e-6])
        reversals[name]=int(np.count_nonzero(np.diff(sign)))
    jerk=0.
    for a,b in zip(trajectory.points,trajectory.points[1:]):
        dt=time_seconds(b)-time_seconds(a)
        polynomial=np.polynomial.polynomial.polyder(coefficients(a,b,dt),3)/dt**3
        jerk=max(jerk,max(peak(polynomial[:,i]) for i in range(len(trajectory.joint_names))))
    return dict(duration_s=time_seconds(trajectory.points[-1]),peak_jerk_rad_s3=jerk,
                reversals=reversals,total_joint_travel_deg=np.degrees(np.abs(np.diff(sampled,axis=0)).sum(axis=0)).tolist())


def replay(path,urdf):
    saved=json.loads(path.read_text());side=saved['arm'];direction=saved['sweep_direction']
    planner=ScopedTaskPlanner.__new__(ScopedTaskPlanner)
    planner.model=OfflineModel(urdf);planner.waist_assist=saved['waist_assist']
    planner.reference_frame='pelvis';planner.search_diagnostics={}
    planner._names={s:arm_names(s,planner.waist_assist) for s in ('left','right')}
    planner._joint_limits={s:{n:planner.model.limits[n] for n in planner.names(s)} for s in ('left','right')}
    planner.scoped_limits=planner._joint_limits
    params,_=load_parameters();configure_planner(planner,params)
    info=saved['search_diagnostics']['straight_index']
    planner.dorsal_in_tip=np.asarray(info['dorsal_normal_in_tcp'])
    planner.fixed_forward_yaw=math.radians(info['fixed_forward_yaw_deg'])
    planner.base_check=HandBaseCollision(planner.model,urdf,saved['contact_scene']['base_top_pelvis_m'])
    planner.scoped_generation=1;planner.check_plan=lambda ticket:None
    planner.progress=lambda *args:None;planner.timing=None
    wrist=np.asarray(info['wrist_from_tip']);q=dict(saved['start'])
    center=np.asarray(saved['contact_scene']['base_top_pelvis_m'])
    nominal=saved['search_diagnostics']['preinsert']['selected']['orientation_reference_xyzw']
    segments=[]
    for segment in saved['segments']:
        trajectory=JointTrajectory();set_message_fields(trajectory,segment['trajectory'])
        start=dict(q);q.update(zip(trajectory.joint_names,trajectory.points[-1].positions))
        if segment['label'] in ('Approach','Return ready'):
            result=planner.joint_path(side,start,q,Ticket(1,'task',side,saved['velocity_scaling'],
                saved['acceleration_scaling']),segment['label'],stage='approach' if segment['label']=='Approach' else 'return_ready')
            new=result['trajectory']
            new_order=[new.joint_names.index(n) for n in trajectory.joint_names]
            for index in (0,-1):
                np.testing.assert_allclose(np.asarray(new.points[index].positions)[new_order],
                    trajectory.points[index].positions,atol=1e-12,rtol=0)
            segments.append(dict(label=segment['label'],before=metrics(trajectory),after=metrics(new),
                geometry={'kind':'same_joint_line','scoped_collision_checked':True},trajectory=message_to_ordereddict(new)))
            continue
        if segment['label'] not in ('Insert','Left 120','Right 120','Withdraw'):continue
        stage=('withdraw' if segment['label']=='Withdraw' else 'insert' if segment['label']=='Insert' else 'rotate')
        initial=planner.tip(side,start,wrist)[:3,3]
        target=planner.tip(side,q,wrist)[:3,3]
        if stage=='rotate':
            radius=np.linalg.norm(initial[:2]-center[:2]);angle=math.atan2(*(initial-center)[[1,0]])
            sign=1 if direction=='left' else -1
            def batch(points):
                points=np.asarray(points);a=np.arctan2(points[:,1]-center[1],points[:,0]-center[0])-angle
                a=(a+np.pi)%(2*np.pi)-np.pi;a=angle+sign*np.clip(sign*a,0,math.radians(120))
                nearest=np.column_stack((center[0]+radius*np.cos(a),center[1]+radius*np.sin(a),np.full(len(a),initial[2])))
                return np.linalg.norm(points-nearest,axis=1)
            reference=nominal
        else:
            delta=target-initial
            def batch(points):
                points=np.asarray(points)
                return np.linalg.norm(points-(initial+np.clip((points-initial)@delta/(delta@delta),0,1)[:,None]*delta),axis=1)
            reference=nominal if stage=='insert' else Rotation.from_matrix(planner.tip(side,start,wrist)[:3,:3]).as_quat()
        def curve(p):return float(batch([p])[0])
        curve.batch=batch;curve.target=target
        report=dict(checked_samples=0,max_position_error_m=0.,max_tilt_deg=0.,max_twist_deg=0.)
        def check(state):
            planner.check_tcp_geometry(side,state,wrist,reference,curve,params.orientation_policy(),stage,segment['label'],report)
        check.wrist_from_tip=wrist
        check.batch=lambda tips:planner.check_tcp_geometry_batch(tips,reference,curve,params.orientation_policy(),stage,segment['label'],report)
        planner.call=lambda *args:SimpleNamespace(success=True,trajectory=RobotTrajectory(joint_trajectory=copy.deepcopy(trajectory)))
        result=planner.retime_cartesian(side,start,RobotTrajectory(joint_trajectory=trajectory),
            Ticket(1,'task',side,saved['velocity_scaling'],saved['acceleration_scaling']),segment['label'],check,stage,report)
        new=result['trajectory']
        np.testing.assert_allclose(new.points[0].positions,trajectory.points[0].positions,atol=1e-12,rtol=0)
        np.testing.assert_allclose(new.points[-1].positions,trajectory.points[-1].positions,atol=1e-12,rtol=0)
        segments.append(dict(label=segment['label'],before=metrics(trajectory),after=metrics(new),
            geometry=report,trajectory=message_to_ordereddict(new)))
    return dict(source=str(path.resolve()),direction=direction,segments=segments,diagnostics=planner.search_diagnostics)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--plans',nargs='+',type=Path,required=True)
    parser.add_argument('--urdf',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    report=dict(offline_only=True,hardware_commands_sent=0,ros_initialized=False,
        note='Refit archived timed waypoints, not unavailable original raw IK; identical endpoints and actual FK/FCL checks',
        plans=[replay(path,args.urdf) for path in args.plans])
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(report,indent=2)+'\n')
    for plan in report['plans']:
        for s in plan['segments']:
            print(plan['direction'],s['label'],s['before']['duration_s'],'->',s['after']['duration_s'],s['geometry'])


if __name__=='__main__':main()
