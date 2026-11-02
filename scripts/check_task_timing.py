"""Call the nonactuating time-parameterization service in the disconnected UI domain."""
import json
import argparse
import copy
import os
import re
import time
from pathlib import Path
import rclpy
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from rosidl_runtime_py.set_message import set_message_fields
from g1_upper_body_moveit_config.srv import TimeTaskPath
from task_motion_planner import robot_state
from task_session import ARMS
from task_trajectory import (samples, stretch_to_limits, time_seconds, coefficients, peak,
                             condition_endpoints, task_jerk_limits, validate_jerk_limits)
import numpy as np
from types import SimpleNamespace


def archived_geometry_check(planner, plan, geometry, label):
    """Recreate the original contact oracle without running IK or moving a robot."""
    from task_straight_policy import arc_reference
    from workbench_orientation import orientation_policy
    if label in ('Approach','Return ready'):return None
    stage='insert' if label=='Insert' else 'withdraw' if label=='Withdraw' else 'rotate'
    selected=plan['search_diagnostics']['preinsert']['selected']
    pre=np.asarray(selected['preinsert_tcp_matrix'])[:3,3]
    transform=np.asarray(geometry['capture_frame']['pelvis_from_torso_row_major'])
    entry=transform[:3,:3]@geometry['task_preview']['insertion_reference_m']+transform[:3,3]
    center=transform[:3,:3]@geometry['task_preview']['axis_center_m']+transform[:3,3]
    nominal=arc_reference(selected['orientation_reference_xyzw'],center,entry)
    wrist=np.asarray(plan['search_diagnostics']['straight_index']['wrist_from_tip'])
    if stage=='rotate':
        radius=np.linalg.norm(entry[:2]-center[:2])
        curve=lambda p:np.hypot(np.linalg.norm(p[:2]-center[:2])-radius,p[2]-center[2])
        curve.batch=lambda p:np.hypot(np.linalg.norm(p[:,:2]-center[:2],axis=1)-radius,p[:,2]-center[2])
    else:
        direction=entry-pre
        curve=lambda p:np.linalg.norm(p-(pre+np.clip((p-pre)@direction/(direction@direction),0,1)*direction))
        curve.batch=lambda p:np.linalg.norm(p-(pre+np.clip((p-pre)@direction/(direction@direction),0,1)[:,None]*direction),axis=1)
    report=dict(checked_samples=0,max_position_error_m=0.,max_tilt_deg=0.,max_twist_deg=0.)
    def check(q):
        planner.check_tcp_geometry(plan['arm'],q,wrist,nominal,curve,orientation_policy({}),stage,label,report)
    check.wrist_from_tip=wrist
    check.batch=lambda tips:planner.check_tcp_geometry_batch(
        tips,nominal,curve,orientation_policy({}),stage,label,report)
    check.report=report
    return check


def metrics(trajectory):
    maxima=np.zeros((3,len(trajectory.joint_names)))
    for a,b in zip(trajectory.points,trajectory.points[1:]):
        dt=time_seconds(b)-time_seconds(a)
        c=coefficients(a,b,dt)
        for order in range(1,4):
            d=np.polynomial.polynomial.polyder(c,order)/dt**order
            maxima[order-1]=np.maximum(maxima[order-1],
                                     [peak(d[:,i]) for i in range(d.shape[1])])
    return {'duration_s':time_seconds(trajectory.points[-1]),
            'points':len(trajectory.points),'joint_names':list(trajectory.joint_names),
            'peak_velocity_rad_s':maxima[0].tolist(),
            'peak_acceleration_rad_s2':maxima[1].tolist(),
            'peak_jerk_rad_s3':maxima[2].tolist()}


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--live',action='store_true')
    parser.add_argument('--plan',type=Path,action='append',default=[])
    parser.add_argument('--output',type=Path)
    parser.add_argument('--validate-geometry',action='store_true')
    parser.add_argument('--robot-urdf',type=Path)
    args=parser.parse_args()
    live=args.live
    if live and args.plan:raise RuntimeError('Archived timing replay is offline only')
    if args.validate_geometry and (not args.plan or not args.robot_urdf or live):
        raise RuntimeError('Geometry replay requires offline plans and an explicit URDF')
    if live and os.environ.get('ROS_DOMAIN_ID')!='0': raise RuntimeError('Live service test requires domain 0')
    if not live and (os.environ.get('ROS_DOMAIN_ID')!='88' or os.environ.get('ROS_LOCALHOST_ONLY')!='1'):
        raise RuntimeError('Disconnected domain 88 required')
    os.environ.setdefault('CYCLONEDDS_URI','<CycloneDDS><Domain><Discovery><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>80</MaxAutoParticipantIndex></Discovery></Domain></CycloneDDS>')
    root=Path(__file__).resolve().parents[1]
    initial=json.loads((root/'assets/workbench/task_start_pose.json').read_text())['joints']
    initial={n:v for n,v in initial.items() if n not in ('waist_roll_joint','waist_pitch_joint')}
    rclpy.init(); node=rclpy.create_node('task_timing_readonly_test')
    client=node.create_client(TimeTaskPath,'/g1/task/time_path')
    try:
        assert client.wait_for_service(timeout_sec=5)
        results=[]
        for plan_path in args.plan:
            plan=json.loads(plan_path.read_text())
            q=dict(plan['start'])
            planner=None
            if args.validate_geometry:
                from task_motion_planner import TaskPlanner
                from task_scoped_planner import ScopedTaskPlanner
                from workbench_model import OfflineModel
                model=OfflineModel(args.robot_urdf)
                geometry=json.loads((plan_path.parent/'geometry.json').read_text())
                ticket=SimpleNamespace(kind=plan['kind'],side=plan['arm'],generation=1,
                    velocity=plan['velocity_scaling'],acceleration=plan['acceleration_scaling'])
                def call(service,request,ticket=None,timeout=30):
                    if node.count_publishers('/arm_sdk'):raise RuntimeError('Unexpected hardware publisher')
                    if not service.wait_for_service(timeout_sec=5):raise RuntimeError(service.srv_name)
                    future=service.call_async(request)
                    rclpy.spin_until_future_complete(node,future,timeout_sec=timeout)
                    if not future.done():raise TimeoutError(service.srv_name)
                    return future.result()
                kwargs=dict(waist_assist=plan['waist_assist'],articulated_hands=True)
                if plan['kind']=='task':
                    kwargs.update(robot_urdf=args.robot_urdf,check_plan=lambda ticket:None)
                    planner=ScopedTaskPlanner(node,model,call,**kwargs)
                else:planner=TaskPlanner(node,model,call,**kwargs)
                progress_updates=[]
                def on_progress(ticket,message):
                    match=re.search(r'validating trajectory (\d+)/(\d+)',message)
                    if match:
                        progress_updates.append(dict(at_monotonic=time.monotonic(),
                            checked=int(match[1]),total=int(match[2])))
                planner.progress=on_progress
                cloud=np.load(plan_path.parent/'cloud.npz')
                transform=np.asarray(geometry['capture_frame']['pelvis_from_torso_row_major'])
                planner.install_surface(cloud['xyz'],transform,ticket,
                    center=geometry.get('task_preview',{}).get('axis_center_m'))
            for segment in plan['segments']:
                saved=segment.get('trajectory',{})
                if not any('shoulder' in n for n in saved.get('joint_names',[])):
                    if saved.get('points'):q.update(zip(saved['joint_names'],saved['points'][-1]['positions']))
                    continue
                path=RobotTrajectory();set_message_fields(path.joint_trajectory,saved)
                old=copy.deepcopy(path.joint_trajectory)
                waist='waist_yaw_joint' in old.joint_names
                req=TimeTaskPath.Request(arm=segment['side']+('_waist' if waist else ''),
                    start_state=robot_state(q),trajectory=path,
                    velocity_scaling=plan['velocity_scaling'],acceleration_scaling=plan['acceleration_scaling'])
                future=client.call_async(req)
                rclpy.spin_until_future_complete(node,future,timeout_sec=40)
                if not future.done() or not future.result().success:
                    raise RuntimeError(future.result().message if future.done() else 'timing timeout')
                result=future.result();trajectory=result.trajectory.joint_trajectory
                before=metrics(trajectory)
                condition_endpoints(trajectory)
                vmax,amax=(.4,.5) if waist else (.6,1.5)
                limits=task_jerk_limits(trajectory.joint_names)
                factor=stretch_to_limits(trajectory,vmax*req.velocity_scaling,
                                         amax*req.acceleration_scaling,limits)
                validate_jerk_limits(trajectory,limits)
                checked=np.asarray(list(samples(trajectory,vmax*req.velocity_scaling,
                                                amax*req.acceleration_scaling)))
                endpoint=max(np.max(np.abs(np.asarray(old.points[i].positions)-trajectory.points[i].positions))
                             for i in (0,-1))
                assert endpoint<1e-6,endpoint
                geometry_report=None
                if planner is not None:
                    progress_updates.clear()
                    check=(archived_geometry_check(planner,plan,geometry,segment['label'])
                           if plan['kind']=='task' else None)
                    remote_call=planner.call
                    # Validate this exact already-timed output, not a second smoothing pass.
                    planner.call=lambda client,*a,**kw:result if client==planner.timing else remote_call(client,*a,**kw)
                    try:
                        planner.retime(segment['side'],q,path,ticket,segment['label'],
                            geometry_check=check,stage=plan['kind'])
                    finally:planner.call=remote_call
                    geometry_report=check.report if check else None
                results.append({'plan':str(plan_path),'label':segment['label'],
                    'timing_message':result.message,'baseline':metrics(old),
                    'ruckig':before,'final':metrics(trajectory),'hermite_stretch':factor,
                    'endpoint_error_rad':float(endpoint),'checked_states':len(checked),
                    'collision_revalidated':planner is not None,
                    'collision_scope':plan.get('contact_scene',{}).get('collision_scope','full_scene') if planner else None,
                    'tcp_geometry':geometry_report})
                if planner is not None:
                    results[-1]['validation_progress']=list(progress_updates)
                q.update(zip(trajectory.joint_names,trajectory.points[-1].positions))
                print(f"{segment['label']}: {time_seconds(old.points[-1]):.3f}s -> "
                      f"{time_seconds(trajectory.points[-1]):.3f}s; Hermite stretch {factor:.3f}",flush=True)
        for scale in (() if args.plan else (.25,.5,1.)):
            path=RobotTrajectory(); path.joint_trajectory.joint_names=list(ARMS['left'])
            q=[initial[n] for n in ARMS['left']]
            path.joint_trajectory.points=[JointTrajectoryPoint(positions=q)]
            q=list(q);q[0]+=.04
            path.joint_trajectory.points.append(JointTrajectoryPoint(positions=q))
            request=TimeTaskPath.Request(arm='left',start_state=robot_state(initial),trajectory=path,
                                         velocity_scaling=scale,acceleration_scaling=scale)
            future=client.call_async(request)
            rclpy.spin_until_future_complete(node,future,timeout_sec=15)
            assert future.done() and future.result().success
            trajectory=future.result().trajectory.joint_trajectory
            factor=stretch_to_limits(trajectory,.60*scale,1.50*scale)
            checked=list(samples(trajectory,.60*scale,1.50*scale))
            assert abs(checked[-1][0]-q[0])<.001
            results.append({'scaling':scale,'duration_s':time_seconds(trajectory.points[-1]),
                            'hermite_time_stretch':factor,'checked_states':len(checked)})
        if not args.plan:assert results[0]['duration_s']>results[1]['duration_s']>results[2]['duration_s']
        if not live: assert node.count_publishers('/arm_sdk')==0
        output={'real_motion':False,'hardware_executable':False,
                'live_scene_updated':False,'offline_scene_updated':args.validate_geometry,
                'timing_checks':results}
        output_path=args.output or root/'logs/task_application_gateway_test/timing.json'
        output_path.parent.mkdir(parents=True,exist_ok=True)
        output_path.write_text(json.dumps(output,indent=2)+'\n')
        print(output_path)
    finally:node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
