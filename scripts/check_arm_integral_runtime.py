"""Startup/readonly parameter checks on localhost domain 88, hardware output disabled."""
import json
import argparse
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from rcl_interfaces.srv import GetParameters,SetParametersAtomically
from rclpy.parameter import Parameter

from task_planning_config import load_parameters

ROOT=Path(__file__).resolve().parents[1]


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--output',type=Path,default=ROOT/'logs/arm_integral_20260911')
    parser.add_argument('--arm',choices=('left','right'),default='left')
    args=parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID')!='88' or os.environ.get('ROS_LOCALHOST_ONLY')!='1':
        raise RuntimeError('Requires isolated localhost domain 88')
    params,config=load_parameters();settings=params.controller_parameters(args.arm)
    output=args.output;output.mkdir(exist_ok=True,parents=True)
    executable=ROOT/'install/g1_unitree_control/lib/g1_unitree_control/arm_sdk_hardware_gateway'
    command=[str(executable),'--ros-args','--params-file',
        str(ROOT/'src/g1_unitree_control/config/arm_sdk_hardware_gateway.yaml'),
        '-p','hardware_output_enabled:=false','-p','service_check_enabled:=false',
        '-p','active_arm:='+args.arm,'-p','state_topic:=/arm_integral_test/lowstate']
    for key,value in settings.items():command+=['-p',key+':='+json.dumps(value)]
    rclpy.init();node=rclpy.create_node('arm_integral_runtime_test');diagnostics={}
    time.sleep(.5)
    if any(name!='arm_integral_runtime_test' for name in node.get_node_names()):
        node.destroy_node();rclpy.shutdown();raise RuntimeError('Offline domain 88 is occupied')
    def diagnostic(message):
        for status in message.status:diagnostics.update({v.key:v.value for v in status.values})
    node.create_subscription(DiagnosticArray,'/g1/arm_gateway/diagnostics',diagnostic,10)
    base='/g1_arm_sdk_hardware_gateway'
    getter=node.create_client(GetParameters,base+'/get_parameters')
    setter=node.create_client(SetParametersAtomically,base+'/set_parameters_atomically')
    log=(output/'runtime.log').open('w')
    process=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    def wait(predicate):
        deadline=time.monotonic()+10
        while time.monotonic()<deadline:
            if process.poll() is not None:raise RuntimeError('Gateway exited; inspect runtime.log')
            rclpy.spin_once(node,timeout_sec=.02)
            if node.count_publishers('/arm_sdk'):raise AssertionError('Unexpected hardware publisher')
            if predicate():return
        raise TimeoutError('No isolated gateway response')
    try:
        wait(lambda:getter.service_is_ready() and setter.service_is_ready() and
             diagnostics.get('phase')=='OUTPUT_DISABLED')
        request=GetParameters.Request(names=list(settings))
        future=getter.call_async(request);wait(future.done)
        values=future.result().values
        for (key,expected),value in zip(settings.items(),values):
            actual=(value.bool_value if isinstance(expected,bool) else
                    list(value.double_array_value) if isinstance(expected,list) else value.double_value)
            assert actual==expected,(key,actual,expected)
        assert diagnostics['armed']=='false' and diagnostics['command_publisher_exists']=='false'
        assert diagnostics['hardware_command_count']=='0'
        assert diagnostics['arm_integral_enabled']=='true'
        assert float(diagnostics['left_shoulder_roll_joint_ki'])==settings['arm_integral_ki'][1]
        assert diagnostics['left_shoulder_roll_joint_integral_nm']=='0.000000'
        assert diagnostics['passive_arm_integral_enabled']==str(settings['passive_arm_integral_enabled']).lower()
        assert abs(float(diagnostics['arm_integral_recovery_error_deg'])-settings['arm_integral_recovery_error_deg'])<1e-6
        assert diagnostics['arm_integral_policy']=='bounded_slow_recovery_v4_hold_rate'
        assert float(diagnostics['arm_integral_hold_rate_nm_s'])==settings['arm_integral_hold_rate_nm_s']
        assert diagnostics['right_shoulder_pitch_joint_integral_nm']=='0.000000'
        assert diagnostics['waist_integral_policy']=='bounded_slow_recovery_v3'
        for key in ('waist_kp','waist_kd','waist_ki','waist_integral_torque_limit_nm',
                    'waist_integral_recovery_error_deg','waist_integral_build_rate_nm_s',
                    'waist_integral_curve_scale'):
            assert abs(float(diagnostics[key])-settings[key])<1e-6,(key,diagnostics[key],settings[key])
        assert diagnostics['waist_integral_smooth_stiction']==str(settings['waist_integral_smooth_stiction']).lower()
        from task_planning_config import ARM_INTEGRAL_JOINTS
        for side in ('left','right'):
            for gain in ('kp','kd'):
                for joint,value in zip(ARM_INTEGRAL_JOINTS,settings[side+'_arm_'+gain]):
                    assert float(diagnostics[side+'_'+joint+'_joint_'+gain])==value
        rejected=[]
        for name,value in settings.items():
            if name in ('waist_kp','waist_kd','waist_ki','waist_integral_torque_limit_nm'):
                continue  # Existing guarded gain updates are unchanged; new policy settings are readonly.
            replacement=(False if isinstance(value,bool) else [0.]*7 if isinstance(value,list) else value*.9)
            request=SetParametersAtomically.Request(parameters=[
                Parameter(name,value=replacement).to_parameter_msg()])
            future=setter.call_async(request);wait(future.done)
            result=future.result().result
            assert not result.successful,result.reason
            rejected.append(dict(parameter=name,reason=result.reason))
        report=dict(domain=88,arm=args.arm,hardware_output_enabled=False,hardware_commands_sent=0,
            phase=diagnostics['phase'],settings=settings,configuration=config,
            runtime_changes_rejected=rejected,success=True)
        (output/'runtime.json').write_text(json.dumps(report,indent=2)+'\n')
        print(json.dumps({k:v for k,v in report.items() if k!='configuration'},indent=2))
    finally:
        node.destroy_node();rclpy.shutdown()
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:process.wait(timeout=5)
            except subprocess.TimeoutExpired:process.kill();process.wait()
        log.close()


if __name__=='__main__':main()
