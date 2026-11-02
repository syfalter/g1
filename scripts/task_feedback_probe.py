"""Read-only latency/preview diagnostics; no service calls or motion publishers."""
import json
import time
import argparse
from types import SimpleNamespace

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import JointState
from unitree_hg.msg import LowState
from std_msgs.msg import String
from moveit_msgs.msg import DisplayTrajectory


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--stress',action='store_true');args=parser.parse_args()
    rclpy.init()
    node=Node('task_feedback_readonly_probe',enable_rosout=False,start_parameter_services=False)
    stats={};subscriptions=[];latest={}
    def receive(topic,msg):
        now=time.monotonic();s=stats.setdefault(topic,dict(count=0,max_gap_ms=0.,max_age_ms=0.,last=now))
        s['max_gap_ms']=max(s['max_gap_ms'],(now-s['last'])*1000);s['last']=now;s['count']+=1
        if isinstance(msg,JointState):
            if 'left_elbow_joint' not in msg.name:return
            age=(node.get_clock().now().nanoseconds-msg.header.stamp.sec*10**9-msg.header.stamp.nanosec)/1e6
            s['max_age_ms']=max(s['max_age_ms'],age)
        elif isinstance(msg,String):latest['console']=json.loads(msg.data)
        elif isinstance(msg,DisplayTrajectory):latest['preview_points']=[len(t.joint_trajectory.points) for t in msg.trajectory]
    for topic,typ,latched in (('/lowstate',LowState,False),('/joint_states',JointState,False),
            ('/g1/task/console_status',String,True),('/g1/task/validated_path',DisplayTrajectory,True)):
        qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE if latched else ReliabilityPolicy.BEST_EFFORT,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL if latched else DurabilityPolicy.VOLATILE)
        subscriptions.append(node.create_subscription(typ,topic,lambda msg,t=topic:receive(t,msg),qos))
    pump=None
    if args.stress:
        from task_joint_feedback import JointFeedback
        pump=JointFeedback(SimpleNamespace(context=node.context,joints=lambda msg:receive('dedicated_joint_states',msg)))
        pump.start()
        node.create_timer(.5,lambda:time.sleep(.2))
    end=time.monotonic()+6.
    while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.02)
    latest['preview_subscribers']=[dict(node=x.node_name,reliability=str(x.qos_profile.reliability),
        durability=str(x.qos_profile.durability)) for x in node.get_subscriptions_info_by_topic('/g1/task/validated_path')]
    print(json.dumps(dict(stats=stats,latest=latest),ensure_ascii=False,indent=2))
    if pump:pump.close()
    node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
