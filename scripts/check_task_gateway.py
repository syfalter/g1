"""Isolated simulated-state test of the actual gateway control loop, never robot DDS."""
import json
import csv
import argparse
import os
from pathlib import Path
import signal
import subprocess
import time
import math

import rclpy
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from control_msgs.action import FollowJointTrajectory
from diagnostic_msgs.msg import DiagnosticArray
from rcl_interfaces.srv import SetParametersAtomically
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from g1_unitree_control.srv import ResumeTaskHold
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from unitree_hg.msg import LowState, LowCmd
from task_session import ARMS

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--waist',action='store_true')
    parser.add_argument('--output',type=Path)
    parser.add_argument('--planning-config',type=Path)
    parser.add_argument('--replay-report',type=Path)
    parser.add_argument('--velocity-bias',type=float,default=0.)
    args=parser.parse_args()
    if not math.isfinite(args.velocity_bias) or not 0<=args.velocity_bias<=.1:
        raise ValueError('Velocity readout bias must be within [0,0.1]')
    replay=json.loads(args.replay_report.read_text()) if args.replay_report else None
    if replay is not None and (not args.waist or not replay.get('success') or
            replay.get('mode')!='STRAIGHT_PREVIEW_SCOPED' or replay.get('hardware_executable') is not False):
        raise ValueError('Replay requires a successful scoped offline report and --waist')
    start=json.loads((args.replay_report.parent/'reconstructed_state.json' if replay is not None else
                     ROOT/'assets/workbench/task_start_pose.json').read_text())['joints']
    if os.environ.get('ROS_DOMAIN_ID') != '87' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        raise RuntimeError('Test requires localhost-only ROS domain 87')
    if os.environ.get('RMW_IMPLEMENTATION')=='rmw_fastrtps_cpp':
        os.environ['FASTRTPS_DEFAULT_PROFILES_FILE']=str(ROOT/'scripts/task_test_fastdds.xml')
        os.environ['FASTDDS_DEFAULT_PROFILES_FILE']=str(ROOT/'scripts/task_test_fastdds.xml')
    rclpy.init(); node=rclpy.create_node('task_gateway_simulation')
    time.sleep(.5)
    if any(name!='task_gateway_simulation' for name in node.get_node_names()):
        node.destroy_node();rclpy.shutdown();raise RuntimeError('Offline domain 87 is occupied')
    directory=args.output or ROOT/('logs/task_application_gateway_test_waist' if args.waist else 'logs/task_application_gateway_test')
    directory.mkdir(exist_ok=True,parents=True)
    log=(directory/'gateway.log').open('w')
    controller_settings={}
    if args.planning_config:
        from task_planning_config import load_parameters
        parameters,_=load_parameters(args.planning_config)
        controller_settings=parameters.controller_parameters('left')
    overrides=[]
    for name,value in controller_settings.items():overrides+=['-p',name+':='+json.dumps(value)]
    process=subprocess.Popen([
        str(ROOT/'install/g1_unitree_control/lib/g1_unitree_control/arm_sdk_hardware_gateway'),
        '--ros-args','--params-file',str(ROOT/'src/g1_unitree_control/config/arm_sdk_hardware_gateway.yaml'),
        '-p','task_application_enabled:=true','-p','active_arm:=left',
        '-p','task_waist_enabled:='+str(args.waist).lower(),
        '-p','hardware_output_enabled:=true','-p','hardware_authorization:=I_UNDERSTAND_ARM_SDK_OUTPUT',
        '-p','service_check_enabled:=false','-p','state_topic:=/task_test/lowstate',
        '-p','command_topic:=/task_test/command','-r','/arm_sdk:=/task_test/forbidden',
        '-p',f'trajectory_log_directory:={directory}/trajectories',*overrides],
        stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    positions=[start[n] for n in ARMS['left']+ARMS['right']]
    simulator=subprocess.Popen([
        str(ROOT/'install/g1_unitree_control/lib/g1_unitree_control/task_gateway_simulator'),
        '--ros-args','-p','positions:='+str(positions),'-p','include_waist:='+str(args.waist).lower(),
        '-p','waist_yaw:='+str(start['waist_yaw_joint'] if replay is not None else .12),'-p',
        'waist_tracking_bias_rad:='+('0.01' if args.waist else '0.0'),'-p',
        'moving_velocity_bias_rad_s:='+str(args.velocity_bias)],
        stdout=log,stderr=subprocess.STDOUT)
    commands=[]; diagnostics={};low_commands=[]
    node.create_subscription(LowCmd,'/task_test/command',lambda msg:low_commands.append(msg),10)
    def command(msg):
        commands.append((time.monotonic(),msg))
        positions[:]=msg.position
    def diagnostic(msg):
        for item in msg.status: diagnostics.update({v.key:v.value for v in item.values})
    node.create_subscription(JointState,'/task_test/command_state',command,10)
    node.create_subscription(DiagnosticArray,'/g1/arm_gateway/diagnostics',diagnostic,10)
    service_clients={name:node.create_client(Trigger,'/g1/arm_gateway/'+name)
                     for name in ('arm','stop_task','cancel_trajectory')}
    service_clients['hold_after_stop']=node.create_client(ResumeTaskHold,'/g1/arm_gateway/hold_after_stop')
    action_clients={side:ActionClient(node,FollowJointTrajectory,'/'+side+'_arm_controller/follow_joint_trajectory')
                    for side in ('left','right')}
    def wait(predicate,timeout=10):
        deadline=time.monotonic()+timeout
        while time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.01)
            if node.count_publishers('/arm_sdk') or node.count_publishers('/task_test/forbidden'):
                raise AssertionError('Unexpected non-test command publisher')
            if process.poll() is not None: raise RuntimeError('Gateway exited; inspect '+str(directory))
            if predicate(): return
        raise TimeoutError(str({'diagnostics':diagnostics,'nodes':node.get_node_names(),
            'state_publishers':[str(p) for p in node.get_publishers_info_by_topic('/task_test/lowstate')],
            'diagnostic_publishers':[str(p) for p in node.get_publishers_info_by_topic('/g1/arm_gateway/diagnostics')]}))
    def service(name, epoch=None):
        kind=Trigger if epoch is None else ResumeTaskHold
        request=Trigger.Request() if epoch is None else ResumeTaskHold.Request(stop_epoch=epoch)
        client=service_clients[name]
        wait(lambda:client.service_is_ready())
        future=client.call_async(request); wait(future.done)
        return future.result()
    parameter_setter=node.create_client(
        SetParametersAtomically,'/g1_arm_sdk_hardware_gateway/set_parameters_atomically')
    def set_parameters(**values):
        wait(parameter_setter.service_is_ready)
        request=SetParametersAtomically.Request()
        request.parameters=[
            Parameter(name,value=float(value)).to_parameter_msg()
            for name,value in values.items()]
        future=parameter_setter.call_async(request);wait(future.done)
        return future.result().result
    def goal(side, stale=False, malformed=None, reorder=False, fast_waist=False):
        client=action_clients[side]
        wait(client.server_is_ready)
        trajectory=JointTrajectory(joint_names=list(ARMS[side]))
        if not stale: trajectory.header.stamp=node.get_clock().now().to_msg()
        base=0 if side=='left' else 7
        q=[float(positions[base+i]) for i in range(7)]
        if args.waist:
            trajectory.joint_names.append('waist_yaw_joint');q.append(float(positions[14]))
        count=len(q)
        trajectory.points=[JointTrajectoryPoint(positions=q,velocities=[0.]*count,accelerations=[0.]*count)]
        q=list(q); q[0]+=.04
        if args.waist:q[-1]+=.02
        end=JointTrajectoryPoint(positions=q,velocities=[0.]*count,accelerations=[0.]*count)
        end.time_from_start.sec=3; trajectory.points.append(end)
        if fast_waist:
            assert args.waist
            end.time_from_start.sec=1
            end.positions[-1]=trajectory.points[0].positions[-1]+.06
        if malformed=='missing_arm':
            trajectory.joint_names.pop(0)
            for p in trajectory.points:
                p.positions.pop(0);p.velocities.pop(0);p.accelerations.pop(0)
        elif malformed=='duplicate':trajectory.joint_names[0]='waist_yaw_joint'
        elif malformed=='mixed':trajectory.joint_names[0]='right_shoulder_pitch_joint'
        elif malformed=='pitch':trajectory.joint_names[-1]='waist_pitch_joint'
        elif malformed=='velocity':trajectory.points[-1].velocities[-1]=.41
        elif malformed=='margin':trajectory.points[-1].positions[-1]=.61
        elif malformed=='overshoot':
            end.time_from_start.sec=8;end.positions[-1]=trajectory.points[0].positions[-1]
            trajectory.points[0].velocities[-1]=.4;end.velocities[-1]=-.4
        elif malformed=='missing_derivatives':trajectory.points[0].accelerations=[]
        elif malformed=='endpoint_acceleration':trajectory.points[-1].accelerations[0]=.05
        elif malformed=='arm_motion_budget':
            mid=JointTrajectoryPoint(positions=[(a+b)/2 for a,b in zip(trajectory.points[0].positions,end.positions)],
                velocities=[.45]+[0.]*(count-1),accelerations=[0.]*count)
            mid.time_from_start.sec=1;mid.time_from_start.nanosec=500000000
            trajectory.points.insert(1,mid)
        if reorder:
            trajectory.joint_names=list(reversed(trajectory.joint_names))
            for p in trajectory.points:
                p.positions=list(reversed(p.positions));p.velocities=list(reversed(p.velocities));p.accelerations=list(reversed(p.accelerations))
        future=client.send_goal_async(FollowJointTrajectory.Goal(trajectory=trajectory))
        wait(future.done); return future.result()
    try:
        wait(lambda:diagnostics.get('mode_machine')=='6',30)
        # Collect the median tau_est anchor history, with simulated zero residual.
        until=time.monotonic()+4.2; wait(lambda:time.monotonic()>until,6)
        assert service('arm').success,diagnostics
        wait(lambda:diagnostics.get('phase')=='ARMED_HOLD',8)
        replay_results=[]
        if replay is not None:
            from rosidl_runtime_py.set_message import set_message_fields
            for segment in replay['segments']:
                if segment.get('kind')=='hand':continue
                trajectory=JointTrajectory();set_message_fields(trajectory,segment['trajectory'])
                client=action_clients[replay['arm']]
                wait(client.server_is_ready)
                trajectory.header.stamp=node.get_clock().now().to_msg()
                future=client.send_goal_async(FollowJointTrajectory.Goal(trajectory=trajectory));wait(future.done)
                accepted=future.result();assert accepted.accepted,(segment['label'],diagnostics)
                result=accepted.get_result_async();wait(result.done,60)
                assert result.result().status==4,(segment['label'],result.result())
                wait(lambda:diagnostics.get('phase')=='ARMED_HOLD')
                replay_results.append(dict(label=segment['label'],completed=True,
                    peak_measured_velocity_rad_s=float(diagnostics['peak_measured_velocity_rad_s'])))
        wait(lambda:diagnostics.get('phase')=='ARMED_HOLD',8)
        if controller_settings:
            wait(lambda:bool(low_commands))
            for side,first in (('left',15),('right',22)):
                for gain in ('kp','kd'):
                    for i,value in enumerate(controller_settings[side+'_arm_'+gain]):
                        assert abs(getattr(low_commands[-1].motor_cmd[first+i],gain)-value)<1e-5
        expected_waist={name:controller_settings.get(name,default) for name,default in
            (('waist_kp',60.),('waist_kd',4.5),('waist_ki',20.),('waist_integral_torque_limit_nm',1.2))}
        for name,value in expected_waist.items():
            assert abs(float(diagnostics[name])-value)<1e-6,(name,diagnostics[name],value)
        def set_integral_limit_gradually(value):
            current=float(diagnostics['waist_integral_torque_limit_nm'])
            while abs(current-value)>1e-6:
                next_value=current+max(-.5,min(.5,value-current))
                result=set_parameters(waist_integral_torque_limit_nm=next_value)
                assert result.successful,result.reason
                wait(lambda:abs(float(diagnostics['waist_integral_torque_limit_nm'])-next_value)<1e-6)
                current=next_value
        integral_limit_guard=None
        if args.waist:
            wait(lambda:float(diagnostics.get('waist_integral_torque_nm','0'))>.15,3)
            set_integral_limit_gradually(.7)
            wait(lambda:diagnostics.get('waist_integral_torque_limit_nm')=='0.700000')
            wait(lambda:float(diagnostics.get('waist_integral_torque_nm','0'))>.65,5)
            active_integral=float(diagnostics['waist_integral_torque_nm'])
            rejected=set_parameters(waist_integral_torque_limit_nm=.6)
            assert not rejected.successful,rejected.reason
            assert 'below the active integral output' in rejected.reason,rejected.reason
            assert diagnostics['waist_integral_torque_limit_nm']=='0.700000',diagnostics
            set_integral_limit_gradually(expected_waist['waist_integral_torque_limit_nm'])
            integral_limit_guard={
                'simulated_tracking_bias_rad':.01,
                'active_integral_before_rejection_nm':active_integral,
                'rejected_limit_nm':.6,
                'rejection':rejected.reason,
                'restored_limit_nm':expected_waist['waist_integral_torque_limit_nm']}
        rejected_cases=[]
        if controller_settings:
            assert not goal('left',malformed='arm_motion_budget').accepted
            rejected_cases.append('arm_motion_budget')
        for case in ('missing_derivatives','endpoint_acceleration'):
            assert not goal('left',malformed=case).accepted,case
            rejected_cases.append(case)
        if args.waist:
            assert diagnostics['task_waist_trajectory_supported']=='true'
            for case in ('missing_arm','duplicate','mixed','pitch','velocity','margin','overshoot'):
                assert not goal('left',malformed=case).accepted,case
                rejected_cases.append(case)
            # Bernstein acceleration envelope is 1.2 rad/s^2: old .5 rejects it,
            # new shared 1.5 accepts it. All output stays on test-only topics.
            fast=goal('left',fast_waist=True);assert fast.accepted
            fast_result=fast.get_result_async();wait(fast_result.done,8)
            assert fast_result.result().status==4,fast_result.result()
            wait(lambda:diagnostics.get('phase')=='ARMED_HOLD')
        left=goal('left',reorder=args.waist); assert left.accepted
        wait(lambda:diagnostics.get('phase')=='RUNNING')
        t0=time.monotonic(); actual=list(positions)
        assert service('stop_task').success
        result=left.get_result_async(); wait(result.done)
        wait(lambda:diagnostics.get('phase')=='TASK_ADMITTANCE',5)
        if controller_settings:
            wait(lambda:bool(low_commands) and abs(low_commands[-1].motor_cmd[18].kp-2.)<1e-5)
        assert result.result().status==6,result.result()
        assert diagnostics['task_stop_active']=='true'
        assert abs(float(diagnostics['waist_integral_torque_nm']))<1e-9
        assert not service('cancel_trajectory').success
        rejected=goal('right'); assert not rejected.accepted
        observed=[(t,c) for t,c in commands if t>=t0]
        assert observed
        hold_error=max(abs(observed[0][1].position[i]-actual[i]) for i in range(len(actual)))
        assert hold_error<.006,hold_error
        assert not service('hold_after_stop',epoch=0).success
        assert service('hold_after_stop',epoch=int(diagnostics['task_stop_epoch'])).success
        wait(lambda:diagnostics.get('phase')=='ARMED_HOLD' and diagnostics.get('task_stop_active')=='false')
        if controller_settings:
            wait(lambda:abs(low_commands[-1].motor_cmd[18].kp-controller_settings['left_arm_kp'][3])<1e-5)
        assert not goal('right',stale=True).accepted
        before_right=list(positions)
        right_started=time.monotonic()
        right=goal('right'); assert right.accepted
        right_result=right.get_result_async(); wait(right_result.done,8)
        assert right_result.result().status==4,right_result.result()
        completed_at=time.monotonic()
        wait(lambda:any(t>=completed_at and c.velocity and max(map(abs,c.velocity))<1e-7
                        for t,c in commands),2)
        right_commands=[c for t,c in commands if t>=right_started and c.velocity]
        hold_commands=[c for t,c in commands if t>=completed_at and c.velocity]
        peak_command_dq=max(abs(v) for c in right_commands for v in c.velocity)
        final_command_dq=max(map(abs,hold_commands[-1].velocity))
        assert peak_command_dq>1e-4,peak_command_dq
        assert final_command_dq<1e-7,final_command_dq
        assert diagnostics.get('safety_latched')!='true',diagnostics
        if args.waist:
            wait(lambda:abs(positions[14]-before_right[14]-.02)<1e-5,2)
            assert max(abs(positions[i]-before_right[i]) for i in range(7))<1e-6
        gaps=[b[0]-a[0] for a,b in zip(commands,commands[1:])]
        stamp=lambda c:c.header.stamp.sec+c.header.stamp.nanosec*1e-9
        source_gaps=[stamp(b[1])-stamp(a[1]) for a,b in zip(commands,commands[1:])]
        motion_log=Path(diagnostics['last_motion_log_path'])
        def settling_recorded():
            with motion_log.open() as stream:
                return any(row.get('event')=='settling_hold' for row in csv.DictReader(stream))
        wait(settling_recorded,2)
        header=motion_log.open().readline().strip().split(',')
        integral_columns={'waist_integral_torque_nm','waist_integral_gain_scale',
                          'waist_integral_mode','waist_integral_saturated'}
        integral_columns.update(n+'_arm_integral_nm' for n in ARMS['left']+ARMS['right'])
        assert len(header)==len(set(header)),header
        assert integral_columns.issubset(header),header
        report={'domain':87,'rmw':os.environ.get('RMW_IMPLEMENTATION'),
                'real_hardware_used':False,'left_stopped_and_aborted':True,
                'waist_enabled':args.waist,'malformed_rejected':rejected_cases,
                'waist_shared_acceleration_curve_completed':args.waist,
                'waist_final_delta_rad':positions[14]-before_right[14] if args.waist else None,
                'waist_pid':{'kp':float(diagnostics['waist_kp']),
                             'kd':float(diagnostics['waist_kd']),
                             'ki':float(diagnostics['waist_ki']),
                             'integral_limit_nm':float(diagnostics['waist_integral_torque_limit_nm'])},
                'integral_columns_logged':True,
                'settling_hold_logged_without_delaying_action':True,
                'controller_settings':controller_settings,
                'replayed_arm_segments':replay_results,'simulated_moving_velocity_bias_rad_s':args.velocity_bias,
                'simulation_note':'Ideal position following with injected velocity readout bias; not robot/contact dynamics',
                'pd_output_and_admittance_verified':bool(controller_settings),
                'armed_integral_limit_guard':integral_limit_guard,
                'peak_waist_integral_torque_nm':float(diagnostics['peak_waist_integral_torque_nm']),
                'admittance_entered':True,'stale_goal_rejected':True,'cancel_cannot_clear_stop':True,
                'right_after_explicit_hold_completed':True,'first_post_stop_command_error_rad':hold_error,
                'peak_commanded_velocity_rad_s':peak_command_dq,
                'final_hold_commanded_velocity_rad_s':final_command_dq,
                'maximum_simulated_command_receive_gap_s':max(gaps),'command_count':len(commands)}
        report['maximum_simulator_command_timestamp_gap_s']=max(source_gaps)
        (directory/'report.json').write_text(json.dumps(report,indent=2)+'\n')
        print(json.dumps(report,indent=2))
    finally:
        process.send_signal(signal.SIGINT)
        try: process.wait(timeout=5)
        except subprocess.TimeoutExpired: process.kill(); process.wait()
        simulator.send_signal(signal.SIGINT)
        try: simulator.wait(timeout=5)
        except subprocess.TimeoutExpired: simulator.kill(); simulator.wait()
        log.close(); node.destroy_node(); rclpy.shutdown()


if __name__=='__main__': main()
