"""Benchmark scalar versus C++ batch full-scene validation on an archived prepare plan."""
import argparse
from datetime import datetime
import json
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import threading
import time

import numpy as np
import rclpy
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from rosidl_runtime_py.set_message import set_message_fields

from task_future import wait_for_result
from task_motion_planner import PlanningFailure, TaskPlanner
from task_session import Ticket
from task_trajectory import condition_endpoints, samples, sample_count, stretch_to_limits
from workbench_model import OfflineModel


ROOT=Path(__file__).resolve().parents[1]
DEFAULT_PLAN=ROOT/'data/task_sessions/20260910_111856_168749/plan_0010'


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--plan',type=Path,default=DEFAULT_PLAN)
    parser.add_argument('--scalar-runs',type=int,default=3)
    parser.add_argument('--batch-runs',type=int,default=5)
    args=parser.parse_args()
    if (os.environ.get('ROS_DOMAIN_ID')!='94' or os.environ.get('ROS_LOCALHOST_ONLY')!='1' or
            os.environ.get('RMW_IMPLEMENTATION')!='rmw_fastrtps_cpp'):
        raise RuntimeError('Use isolated localhost-only Fast DDS domain 94')
    if not 1<=args.scalar_runs<=10 or not 1<=args.batch_runs<=20:
        raise ValueError('Invalid benchmark repetition count')

    directory=args.plan.resolve()
    plan=json.loads((directory/'plan.json').read_text())
    if plan['kind']!='prepare' or plan['arm']!='left' or not plan.get('waist_assist'):
        raise ValueError('Benchmark requires the archived left waist-assisted prepare plan')
    state=dict(plan['start']);arm_segment=None
    for segment in plan['segments']:
        if segment['kind']=='arm':
            arm_segment=segment
            break
        trajectory=segment['trajectory']
        state.update(zip(trajectory['joint_names'],trajectory['points'][-1]['positions']))
    if arm_segment is None:raise ValueError('Archived plan has no arm segment')
    trajectory=JointTrajectory();set_message_fields(trajectory,arm_segment['trajectory'])
    condition_endpoints(trajectory)
    stretch=stretch_to_limits(trajectory,.4*plan['velocity_scaling'],.5*plan['acceleration_scaling'])
    sampled=np.asarray(list(samples(
        trajectory,.4*plan['velocity_scaling'],.5*plan['acceleration_scaling'])))
    if len(sampled)!=sample_count(trajectory):raise RuntimeError('Sample accounting mismatch')
    max_step=float(np.max(np.abs(np.diff(sampled,axis=0))))
    if max_step>.005+1e-12:raise RuntimeError('Archived trajectory requires scalar intersample fallback')

    capture=json.loads((directory/'capture_state.json').read_text())
    torso=np.asarray(capture['pelvis_from_torso_row_major'])
    with np.load(directory/'cloud.npz',allow_pickle=False) as data:
        xyz=data['xyz'].copy();uv=data['uv'].copy()
    server=json.loads((directory/'server_result.json').read_text())
    geometry=json.loads((directory/'geometry.json').read_text())
    output=ROOT/'logs/task_validation_benchmark'/datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    output.mkdir(parents=True)
    report={'date':'2026-09-10','source_plan':str(directory),'offline_only':True,
            'hardware_commands_sent':0,'samples':len(sampled),'trajectory_points':len(trajectory.points),
            'max_neighbor_step_rad':max_step,'endpoint_conditioning_stretch':stretch,'success':False}

    rclpy.init();node=rclpy.create_node('task_full_scene_validation_benchmark')
    executor=MultiThreadedExecutor(num_threads=3);executor.add_node(node)
    thread=threading.Thread(target=executor.spin,daemon=True);thread.start()
    launch=None
    with tempfile.TemporaryDirectory(prefix='g1-validation-benchmark-') as temporary:
        urdf=Path(temporary)/'robot.urdf'
        command=['ros2','launch','g1_upper_body_moveit_config','real_shadow_moveit.launch.py',
            'task_application_enabled:=true','active_arm:=left','start_state_bridge:=false',
            'task_articulated_hands:=true','task_waist_enabled:=true','start_hardware_gateway:=false',
            'hardware_output_enabled:=false','start_shadow_executor:=false',
            'enable_rviz_interactive_execution:=false','show_planned_overlay:=false','use_rviz:=false',
            'export_robot_urdf:='+str(urdf)]
        log=(output/'ros_launch.log').open('w')
        rpc={}
        def check():
            if launch is not None and launch.poll() is not None:
                raise RuntimeError('Offline launch exited; inspect '+str(output/'ros_launch.log'))
        def call(client,request,ticket=None,timeout=30):
            if not client.wait_for_service(timeout_sec=10):raise RuntimeError('Unavailable '+client.srv_name)
            started=time.monotonic()
            try:return wait_for_result(client.call_async(request),check,timeout)
            finally:rpc.setdefault(client.srv_name,[]).append(time.monotonic()-started)
        try:
            time.sleep(.3)
            if any(name!='task_full_scene_validation_benchmark'
                   for name,_ in node.get_node_names_and_namespaces()):
                raise RuntimeError('Domain 94 already contains ROS nodes')
            launch=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            deadline=time.monotonic()+40
            while not urdf.is_file():
                check()
                if time.monotonic()>deadline:raise TimeoutError('Robot model export timed out')
                time.sleep(.05)
            state_pub=node.create_publisher(JointState,'/joint_states',10)
            def publish_state():
                message=JointState();message.header.stamp=node.get_clock().now().to_msg()
                message.name=list(state);message.position=[float(state[name]) for name in state]
                state_pub.publish(message)
            state_timer=node.create_timer(.02,publish_state)
            model=OfflineModel(urdf)
            planner=TaskPlanner(node,model,call,waist_assist=True,articulated_hands=True)
            for client in (planner.valid,planner.batch_valid,planner.scene,planner.apply):
                if not client.wait_for_service(timeout_sec=20):raise RuntimeError('Unavailable '+client.srv_name)
            ticket=Ticket(1,'prepare','left',plan['velocity_scaling'],plan['acceleration_scaling'])
            planner.install_surface(xyz,torso,ticket,uv=uv,bbox=server.get('pedal_centered'),
                                    center=geometry['task_preview']['axis_center_m'])

            def scalar(values):
                for positions in values:
                    current=dict(state);current.update(zip(trajectory.joint_names,map(float,positions)))
                    planner.validate(current,ticket,'left','prepare')
            def batch(values):
                planner.validate_trajectory_batch(
                    state,ticket,'left','prepare',tuple(trajectory.joint_names),values)
            scalar(sampled[:1]);batch(sampled[:1])
            scalar_seconds=[];batch_seconds=[]
            for _ in range(args.scalar_runs):
                started=time.monotonic();scalar(sampled);scalar_seconds.append(time.monotonic()-started)
            for _ in range(args.batch_runs):
                started=time.monotonic();batch(sampled);batch_seconds.append(time.monotonic()-started)

            invalid=sampled[:6].copy();invalid[3,0]=10.
            scalar_failure=None
            try:scalar(invalid)
            except PlanningFailure as error:
                scalar_failure={'code':error.diagnostic['code'],'sample_index':3}
            batch_failure=None
            try:batch(invalid)
            except PlanningFailure as error:
                batch_failure={'code':error.diagnostic['code'],
                               'sample_index':error.diagnostic['batch_sample_index']}
            if scalar_failure!=batch_failure or scalar_failure!={'code':'JOINT_BOUNDS','sample_index':3}:
                raise AssertionError((scalar_failure,batch_failure))
            scalar_mean=float(np.mean(scalar_seconds));batch_mean=float(np.mean(batch_seconds))
            report.update(scalar_seconds=scalar_seconds,batch_seconds=batch_seconds,
                scalar_mean_seconds=scalar_mean,batch_mean_seconds=batch_mean,
                throughput_speedup=scalar_mean/batch_mean,
                wall_time_reduction=1.-batch_mean/scalar_mean,
                first_failure_equivalence={'scalar':scalar_failure,'batch':batch_failure},
                rpc_counts={name:len(values) for name,values in rpc.items()},success=True)
            if report['throughput_speedup']<1.5:
                raise AssertionError('Batch validation did not reach 1.5x throughput')
        except Exception as error:
            report['error']=str(error)
            raise
        finally:
            (output/'report.json').write_text(json.dumps(report,indent=2)+'\n')
            print(json.dumps(report,indent=2),flush=True)
            if launch is not None and launch.poll() is None:
                os.killpg(launch.pid,signal.SIGINT)
                try:launch.wait(timeout=12)
                except subprocess.TimeoutExpired:
                    os.killpg(launch.pid,signal.SIGKILL);launch.wait()
            log.close();executor.shutdown();thread.join(timeout=3);node.destroy_node();rclpy.shutdown()
    return 0


if __name__=='__main__':raise SystemExit(main())
