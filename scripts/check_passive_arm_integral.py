"""Actual gateway + biased fake LowState, exclusively localhost test domain 87."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from std_srvs.srv import Trigger
from task_planning_config import load_parameters
from task_session import ARMS

ROOT=Path(__file__).resolve().parents[1]


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--disable-passive',action='store_true')
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID')!='87' or os.environ.get('ROS_LOCALHOST_ONLY')!='1':
        raise RuntimeError('Requires isolated localhost domain 87')
    if os.environ.get('RMW_IMPLEMENTATION')=='rmw_fastrtps_cpp':
        os.environ['FASTRTPS_DEFAULT_PROFILES_FILE']=str(ROOT/'scripts/task_test_fastdds.xml')
        os.environ['FASTDDS_DEFAULT_PROFILES_FILE']=str(ROOT/'scripts/task_test_fastdds.xml')
    params,config=load_parameters();settings=params.controller_parameters('left')
    settings['passive_arm_integral_enabled']=not args.disable_passive
    q=json.loads((ROOT/'assets/workbench/task_start_pose.json').read_text())['joints']
    positions=[q[n] for n in ARMS['left']+ARMS['right']]
    bias=[0.]*14;bias[7]=-.052
    rclpy.init();node=rclpy.create_node('passive_integral_test');diagnostics={};processes=[]
    time.sleep(.5)
    if any(n!='passive_integral_test' for n in node.get_node_names()):
        node.destroy_node();rclpy.shutdown();raise RuntimeError('Domain 87 occupied')
    def diagnostic(message):
        for status in message.status:diagnostics.update({v.key:v.value for v in status.values})
    node.create_subscription(DiagnosticArray,'/g1/arm_gateway/diagnostics',diagnostic,10)
    clients={n:node.create_client(Trigger,'/g1/arm_gateway/'+n) for n in ('arm','stop_task')}
    args.output.mkdir(parents=True,exist_ok=True)
    with (args.output/'gateway.log').open('w') as log:
        try:
            command=[str(ROOT/'install/g1_unitree_control/lib/g1_unitree_control/arm_sdk_hardware_gateway'),
                '--ros-args','--params-file',str(ROOT/'src/g1_unitree_control/config/arm_sdk_hardware_gateway.yaml'),
                '-p','task_application_enabled:=true','-p','active_arm:=left','-p','task_waist_enabled:=true',
                '-p','hardware_output_enabled:=true','-p','hardware_authorization:=I_UNDERSTAND_ARM_SDK_OUTPUT',
                '-p','service_check_enabled:=false','-p','state_topic:=/task_test/lowstate',
                '-p','command_topic:=/task_test/command','-r','/arm_sdk:=/task_test/forbidden']
            for name,value in settings.items():command+=['-p',name+':='+json.dumps(value)]
            processes.append(subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT))
            processes.append(subprocess.Popen([str(ROOT/'install/g1_unitree_control/lib/g1_unitree_control/task_gateway_simulator'),
                '--ros-args','-p','positions:='+str(positions),'-p','include_waist:=true',
                '-p','waist_yaw:=0.12','-p','arm_tracking_bias_rad:='+str(bias)],stdout=log,stderr=subprocess.STDOUT))
            def wait(predicate,timeout=15):
                deadline=time.monotonic()+timeout
                while time.monotonic()<deadline:
                    rclpy.spin_once(node,timeout_sec=.01)
                    if any(p.poll() is not None for p in processes):raise RuntimeError('Test process exited')
                    if node.count_publishers('/arm_sdk') or node.count_publishers('/task_test/forbidden'):
                        raise AssertionError('Non-test command publisher')
                    if predicate():return
                raise TimeoutError(str(diagnostics))
            def service(name):
                wait(clients[name].service_is_ready)
                f=clients[name].call_async(Trigger.Request());wait(f.done)
                assert f.result().success,f.result().message
            wait(lambda:diagnostics.get('mode_machine')=='6',30)
            deadline=time.monotonic()+4.2;wait(lambda:time.monotonic()>deadline)
            service('arm');wait(lambda:diagnostics.get('phase')=='ARMED_HOLD')
            key='right_shoulder_pitch_joint_integral_nm'
            if args.disable_passive:
                deadline=time.monotonic()+3.;wait(lambda:time.monotonic()>deadline)
                assert float(diagnostics[key])==0.,diagnostics
            else:
                wait(lambda:float(diagnostics.get(key,'0'))<-.5,8)
                assert diagnostics['right_shoulder_pitch_joint_integral_mode']=='bounded_slow_recovery'
                assert abs(float(diagnostics['left_shoulder_pitch_joint_integral_nm']))<1e-6
            before=float(diagnostics[key]);assert abs(before)<=params.shoulder_pitch_integral_limit_nm
            service('stop_task');wait(lambda:diagnostics.get('phase')=='TASK_ADMITTANCE')
            for n in ARMS['left']+ARMS['right']:
                assert abs(float(diagnostics[n+'_integral_nm']))<1e-9,n
            assert float(diagnostics['waist_integral_torque_nm'])==0.
            report=dict(success=True,domain=87,real_hardware_used=False,
                passive_enabled=not args.disable_passive,simulated_right_pitch_error_rad=-.052,
                passive_integral_before_stop_nm=before,both_arms_zero_after_stop=True,
                configuration=config,simulation_note='Fixed injected bias, not robot dynamics')
            (args.output/'report.json').write_text(json.dumps(report,indent=2)+'\n')
            print({k:v for k,v in report.items() if k!='configuration'})
        finally:
            for p in processes:
                if p.poll() is None:p.send_signal(signal.SIGINT)
            for p in processes:
                try:p.wait(timeout=5)
                except subprocess.TimeoutExpired:p.kill();p.wait()
            node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
