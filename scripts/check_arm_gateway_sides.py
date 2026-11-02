"""Isolated dry-run acceptance test. Never arm or enable hardware output."""
import json
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from diagnostic_msgs.msg import DiagnosticArray
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from unitree_hg.msg import LowState

ROOT=Path(__file__).resolve().parents[1]
PARTS=('shoulder_pitch','shoulder_roll','shoulder_yaw','elbow','wrist_roll','wrist_pitch','wrist_yaw')


def main():
    if os.environ.get('ROS_DOMAIN_ID')!='86' or os.environ.get('ROS_LOCALHOST_ONLY')!='1':
        raise RuntimeError('Run only on isolated localhost ROS domain 86')
    rclpy.init()
    reports=[]
    try:
        for side in ('left','right'):
            node=rclpy.create_node('gateway_side_verification_'+side)
            process=None
            log_path=ROOT/f'logs/left_arm_adaptation_20260908_001/dryrun_{side}.log'
            with log_path.open('w') as log:
                try:
                    process=subprocess.Popen([
                        str(ROOT/'install/g1_unitree_control/lib/g1_unitree_control/arm_sdk_hardware_gateway'),
                        '--ros-args','--params-file',str(ROOT/'src/g1_unitree_control/config/arm_sdk_hardware_gateway.yaml'),
                        '-p',f'active_arm:={side}','-p','hardware_output_enabled:=false',
                        '-p','service_check_enabled:=false','-p','state_topic:=/side_test/lowstate',
                        '-p','trajectory_topic:=/side_test/trajectory',
                        '-p',f'trajectory_log_directory:={log_path.parent}/dryrun_trajectories'],
                        stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
                    state=LowState();state.mode_machine=6;state.mode_pr=0
                    for i in [12]+list(range(15,29)):
                        state.motor_state[i].mode=1;state.motor_state[i].vol=52.
                    state.motor_state[18].q=.8;state.motor_state[25].q=.8
                    pub=node.create_publisher(LowState,'/side_test/lowstate',qos_profile_sensor_data)
                    traj_pub=node.create_publisher(JointTrajectory,'/side_test/trajectory',10)
                    observed={};previews=[]
                    def diagnostics(msg):
                        for status in msg.status:
                            values={v.key:v.value for v in status.values}
                            if values.get('active_arm')==side:observed.update(values)
                    node.create_subscription(DiagnosticArray,'/g1/arm_gateway/diagnostics',diagnostics,10)
                    node.create_subscription(JointState,'/g1/arm_gateway/command_preview',lambda msg:previews.append(msg),10)
                    def wait(predicate,seconds=10):
                        end=time.monotonic()+seconds
                        while time.monotonic()<end:
                            if process.poll() is not None:raise RuntimeError('Gateway exited: '+str(log_path))
                            pub.publish(state)
                            rclpy.spin_once(node,timeout_sec=.02)
                            if node.count_publishers('/arm_sdk'):
                                raise AssertionError('Dry-run unexpectedly created an arm command publisher')
                            if predicate():return
                        raise TimeoutError(str(dict(diagnostics=observed,
                            state_subscribers=[str(x) for x in node.get_subscriptions_info_by_topic('/side_test/lowstate')],
                            state_publishers=[str(x) for x in node.get_publishers_info_by_topic('/side_test/lowstate')],
                            topics=node.get_topic_names_and_types())))
                    wait(lambda:observed.get('phase')=='OUTPUT_DISABLED' and
                         observed.get('mode_machine')=='6' and traj_pub.get_subscription_count()>0)
                    def trajectory(arm):
                        msg=JointTrajectory();msg.joint_names=[f'{arm}_{part}_joint' for part in PARTS]
                        start=JointTrajectoryPoint();start.positions=[0.,0.,0.,.8,0.,0.,0.]
                        end=JointTrajectoryPoint();end.positions=list(start.positions);end.positions[0]=.005
                        end.time_from_start.sec=1
                        msg.points=[start,end]
                        return msg
                    other='right' if side=='left' else 'left'
                    traj_pub.publish(trajectory(other))
                    wait(lambda:int(observed.get('rejected_trajectories','0'))==1)
                    assert 'configured active arm' in observed['last_rejection'],observed
                    assert int(observed['accepted_trajectories'])==0,observed
                    traj_pub.publish(trajectory(side))
                    wait(lambda:observed.get('phase')=='DRY_RUN_COMPLETE')
                    assert int(observed['accepted_trajectories'])==1,observed
                    assert previews and all(all(n.startswith(side+'_') for n in p.name) for p in previews)
                    assert observed['command_publisher_exists']=='false',observed
                    reports.append(dict(arm=side,wrong_side_rejected=True,own_side_completed=True,
                                        hardware_publisher_created=False,preview_count=len(previews)))
                finally:
                    node.destroy_node()
                    if process is not None and process.poll() is None:
                        process.send_signal(signal.SIGINT)
                        try:process.wait(timeout=5)
                        except subprocess.TimeoutExpired:
                            process.terminate();process.wait(timeout=5)
        output=ROOT/'logs/left_arm_adaptation_20260908_001/gateway_dryrun.json'
        output.write_text(json.dumps(reports,indent=2)+'\n')
        print(json.dumps(reports,indent=2))
    finally:
        rclpy.shutdown()


if __name__=='__main__':main()
