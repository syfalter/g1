"""Exercise real gateway release/restart on localhost simulation only."""
import json
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy

from task_panel_lifecycle import Probe, silent
from task_session import ARMS

ROOT=Path(__file__).resolve().parents[1]


def main():
    if os.environ.get('ROS_DOMAIN_ID')!='87' or os.environ.get('ROS_LOCALHOST_ONLY')!='1':
        raise RuntimeError('Lifecycle test requires localhost-only ROS domain 87')
    if os.environ.get('RMW_IMPLEMENTATION')=='rmw_fastrtps_cpp':
        profile=str(ROOT/'scripts/task_test_fastdds.xml')
        os.environ['FASTRTPS_DEFAULT_PROFILES_FILE']=profile
        os.environ['FASTDDS_DEFAULT_PROFILES_FILE']=profile
    directory=ROOT/'logs/task_lifecycle_test'
    directory.mkdir(exist_ok=True)
    rclpy.init();probe=Probe();processes=[];gateway=None
    log=(directory/'gateway.log').open('w')
    q=json.loads((ROOT/'assets/workbench/task_start_pose.json').read_text())['joints']
    positions=[q[n] for n in ARMS['left']+ARMS['right']]
    gateway_command=[str(ROOT/'install/g1_unitree_control/lib/g1_unitree_control/arm_sdk_hardware_gateway'),
        '--ros-args','--params-file',str(ROOT/'src/g1_unitree_control/config/arm_sdk_hardware_gateway.yaml'),
        '-p','task_application_enabled:=true','-p','task_waist_enabled:=true',
        '-p','hardware_output_enabled:=true','-p','hardware_authorization:=I_UNDERSTAND_ARM_SDK_OUTPUT',
        '-p','service_check_enabled:=false','-p','state_topic:=/task_test/lowstate',
        '-p','command_topic:=/task_test/command','-r','/arm_sdk:=/task_test/forbidden',
        '-p',f'trajectory_log_directory:={directory}/trajectories']
    simulator_command=[str(ROOT/'install/g1_unitree_control/lib/g1_unitree_control/task_gateway_simulator'),
        '--ros-args','-p','positions:='+str(positions),'-p','include_waist:=true','-p','waist_yaw:=0.12']
    def launch(command):
        process=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        processes.append(process)
        return process
    def stop(process):
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
            process.wait(timeout=8)
    def wait(predicate,timeout=15):
        def checked():
            if probe.node.count_publishers('/arm_sdk') or probe.node.count_publishers('/task_test/forbidden'):
                raise AssertionError('Unexpected non-test command publisher')
            if gateway.poll() is not None:
                raise RuntimeError('Test gateway exited; inspect '+str(directory))
            return predicate()
        try:probe.wait(checked,timeout)
        except RuntimeError:
            print(json.dumps(dict(gateway=probe.gateway,nodes=probe.node.get_node_names(),
                state_publishers=probe.node.count_publishers('/task_test/lowstate'),
                diagnostic_publishers=probe.node.count_publishers('/g1/arm_gateway/diagnostics'))),flush=True)
            raise
    def new_gateway():
        nonlocal gateway
        probe.gateway={};probe.gateway_at=float('-inf')
        gateway=launch(gateway_command)
        wait(lambda:probe.gateway.get('mode_machine')=='6' and probe.gateway.get('phase')=='UNARMED'
             and probe.gateway.get('task_stop_epoch')=='0' and silent(probe.gateway),30)
        assert probe.gateway['hardware_command_count']=='0',probe.gateway
        assert probe.gateway['task_stop_active']=='false',probe.gateway
        return gateway.pid
    def arm_simulation():
        until=time.monotonic()+4.3
        wait(lambda:time.monotonic()>until,6)
        probe.trigger('arm')
        wait(lambda:probe.gateway.get('phase')=='ARMED_HOLD',10)
    try:
        simulator=launch(simulator_command)
        first=new_gateway()
        arm_simulation();probe.trigger('stop_task')
        wait(lambda:probe.gateway.get('phase')=='TASK_ADMITTANCE')
        stop_epoch=probe.gateway['task_stop_epoch']
        probe.release()
        assert silent(probe.gateway),probe.gateway
        assert float(probe.gateway['command_weight'])<=1e-6,probe.gateway
        stop(gateway)
        second=new_gateway()
        assert first!=second
        arm_simulation();stop(simulator)
        wait(lambda:probe.gateway.get('phase')=='LATCHED_SILENT' and silent(probe.gateway))
        probe.release()
        stop(gateway)
        simulator=launch(simulator_command)
        third=new_gateway()
        assert third not in (first,second)
        report=dict(result='PASS',domain=87,real_robot_used=False,
            admittance_release_completed=True,release_weight_reached_zero=True,
            stopped_epoch_before_restart=int(stop_epoch),new_stop_epoch=int(probe.gateway['task_stop_epoch']),
            old_latch_not_inherited=True,new_processes=[first,second,third],
            final_phase=probe.gateway['phase'],final_armed=probe.gateway['armed'],
            final_publisher_exists=probe.gateway['command_publisher_exists'],
            final_hardware_command_count=int(probe.gateway['hardware_command_count']))
        (directory/'report.json').write_text(json.dumps(report,indent=2)+'\n')
        print(json.dumps(report,indent=2))
    finally:
        for process in reversed(processes):
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
                try:process.wait(timeout=8)
                except subprocess.TimeoutExpired:process.kill();process.wait()
        log.close();probe.node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
