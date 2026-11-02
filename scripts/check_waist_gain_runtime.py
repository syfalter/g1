"""Isolated runtime waist-gain parameter test; never enables or arms hardware output."""
import json
import os
from pathlib import Path
import signal
import subprocess
import time

from diagnostic_msgs.msg import DiagnosticArray
from rcl_interfaces.srv import GetParameters,SetParametersAtomically
import rclpy
from rclpy.parameter import Parameter


ROOT=Path(__file__).resolve().parents[1]
NODE='/g1_arm_sdk_hardware_gateway'


def main():
    if os.environ.get('ROS_DOMAIN_ID')!='88' or os.environ.get('ROS_LOCALHOST_ONLY')!='1':
        raise RuntimeError('Test requires isolated localhost ROS domain 88')
    if os.environ.get('RMW_IMPLEMENTATION')=='rmw_fastrtps_cpp':
        profile=str(ROOT/'scripts/task_test_fastdds.xml')
        os.environ['FASTRTPS_DEFAULT_PROFILES_FILE']=profile
        os.environ['FASTDDS_DEFAULT_PROFILES_FILE']=profile
    output=ROOT/'logs/waist_gain_runtime_test';output.mkdir(exist_ok=True)
    executable=str(ROOT/'install/g1_unitree_control/lib/g1_unitree_control/arm_sdk_hardware_gateway')
    config=str(ROOT/'src/g1_unitree_control/config/arm_sdk_hardware_gateway.yaml')
    with (output/'invalid_startup.log').open('w') as invalid_log:
        invalid=subprocess.Popen([
            executable,'--ros-args','--params-file',config,
            '-p','hardware_output_enabled:=false','-p','service_check_enabled:=false',
            '-p','waist_kp:=81.0'],stdout=invalid_log,stderr=subprocess.STDOUT,start_new_session=True)
        try:invalid.wait(timeout=5)
        except subprocess.TimeoutExpired:
            invalid.send_signal(signal.SIGINT);invalid.wait(timeout=5)
            raise AssertionError('Invalid startup waist gain was not rejected')
        assert invalid.returncode!=0,'Invalid startup waist gain unexpectedly launched'
    log=(output/'gateway.log').open('w')
    process=subprocess.Popen([
        executable,'--ros-args','--params-file',config,
        '-p','hardware_output_enabled:=false','-p','service_check_enabled:=false',
        '-p','state_topic:=/waist_gain_test/lowstate',
        '-p',f'trajectory_log_directory:={output}/trajectories'],
        stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    rclpy.init();node=rclpy.create_node('waist_gain_runtime_test')
    diagnostics={}
    def diagnostic(message):
        for status in message.status:
            values={item.key:item.value for item in status.values}
            if 'waist_kp' in values:diagnostics.update(values)
    node.create_subscription(DiagnosticArray,'/g1/arm_gateway/diagnostics',diagnostic,10)
    setter=node.create_client(SetParametersAtomically,NODE+'/set_parameters_atomically')
    getter=node.create_client(GetParameters,NODE+'/get_parameters')
    def wait(predicate,seconds=10):
        deadline=time.monotonic()+seconds
        while time.monotonic()<deadline:
            if process.poll() is not None:raise RuntimeError('Gateway exited; inspect '+str(log.name))
            rclpy.spin_once(node,timeout_sec=.02)
            if node.count_publishers('/arm_sdk'):
                raise AssertionError('Dry-run unexpectedly created an /arm_sdk publisher')
            if predicate():return
        raise TimeoutError(str(diagnostics))
    def set_values(**values):
        request=SetParametersAtomically.Request()
        request.parameters=[Parameter(name,value=value).to_parameter_msg() for name,value in values.items()]
        future=setter.call_async(request);wait(future.done)
        return future.result().result
    def get_values():
        request=GetParameters.Request(names=[
            'waist_kp','waist_kd','waist_ki','waist_integral_torque_limit_nm'])
        future=getter.call_async(request);wait(future.done)
        values=future.result().values
        return tuple(value.double_value for value in values)
    checks=['invalid_startup_rejected'];rejections={}
    try:
        wait(lambda:setter.service_is_ready() and getter.service_is_ready() and
                    diagnostics.get('phase')=='OUTPUT_DISABLED')
        assert get_values()==(60.,4.5,20.,1.2)
        assert diagnostics['waist_kp']=='60.000000' and diagnostics['waist_kd']=='4.500000'
        assert diagnostics['waist_ki']=='20.000000'
        assert diagnostics['waist_integral_torque_limit_nm']=='1.200000'
        assert diagnostics['waist_integral_mode']=='disabled'
        assert diagnostics['waist_integral_torque_nm']=='0.000000'
        assert diagnostics['waist_gain_revision']=='0'
        checks.append('startup_60_4_5_20_1_2')

        accepted=set_values(waist_kp=55.,waist_kd=4.,waist_ki=15.,
                            waist_integral_torque_limit_nm=1.)
        assert accepted.successful,accepted.reason
        wait(lambda:diagnostics.get('waist_gain_revision')=='1')
        assert get_values()==(55.,4.,15.,1.)
        assert diagnostics['waist_kp']=='55.000000' and diagnostics['waist_kd']=='4.000000'
        assert diagnostics['waist_ki']=='15.000000'
        assert diagnostics['waist_integral_torque_limit_nm']=='1.000000'
        assert 'Kp 60 -> 55' in diagnostics['last_waist_gain_change']
        assert 'Ki 20 -> 15' in diagnostics['last_waist_gain_change']
        checks.append('atomic_update')

        rejected=set_values(waist_kp=81.,waist_kd=4.)
        assert not rejected.successful and 'within [20, 80]' in rejected.reason
        assert get_values()==(55.,4.,15.,1.)
        rejections['out_of_range']=rejected.reason
        checks.append('out_of_range_rejected')

        rejected=set_values(waist_kp=50)
        assert not rejected.successful and ('type' in rejected.reason.lower() or
                                             'floating-point' in rejected.reason.lower()),rejected.reason
        assert get_values()==(55.,4.,15.,1.)
        rejections['integer_type']=rejected.reason
        checks.append('integer_type_rejected')

        rejected=set_values(waist_kp=30.,waist_kd=4.)
        assert not rejected.successful and 'at most 20' in rejected.reason
        assert get_values()==(55.,4.,15.,1.)
        rejections['large_step']=rejected.reason
        checks.append('large_step_rejected')

        rejected=set_values(waist_integral_torque_limit_nm=2.1)
        assert not rejected.successful and 'within [0.1, 2.0]' in rejected.reason
        assert get_values()==(55.,4.,15.,1.)
        rejections['integral_limit']=rejected.reason
        checks.append('integral_limit_rejected')

        accepted=set_values(waist_kp=60.,waist_kd=4.5,waist_ki=20.,
                            waist_integral_torque_limit_nm=1.2)
        assert accepted.successful,accepted.reason
        wait(lambda:diagnostics.get('waist_gain_revision')=='2')
        assert get_values()==(60.,4.5,20.,1.2)
        assert diagnostics['waist_kp']=='60.000000' and diagnostics['waist_kd']=='4.500000'
        assert diagnostics['waist_ki']=='20.000000'
        assert diagnostics['waist_integral_torque_limit_nm']=='1.200000'
        assert diagnostics['last_waist_gain_rejection']==''
        checks.append('diagnostics_parameter_sync')
        report={'domain':88,'real_hardware_used':False,'hardware_publisher_created':False,
                'final_phase':diagnostics['phase'],'final_waist_kp':60.,'final_waist_kd':4.5,
                'final_waist_ki':20.,'final_integral_torque_limit_nm':1.2,
                'final_revision':2,'checks':checks,'rejections':rejections}
        (output/'report.json').write_text(json.dumps(report,indent=2)+'\n')
        print(json.dumps(report,indent=2))
    finally:
        node.destroy_node();rclpy.shutdown()
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:process.wait(timeout=5)
            except subprocess.TimeoutExpired:process.kill();process.wait()
        log.close()


if __name__=='__main__':main()
