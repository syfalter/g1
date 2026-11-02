"""Bounded live subscriber check. Does not request samples, hand reads or motion."""
import argparse
import json
import os
from pathlib import Path
import time
from types import SimpleNamespace

import rclpy
from std_msgs.msg import String
from rclpy.qos import QoSProfile, DurabilityPolicy
from tcp_a_node import TcpANode, ROOT


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID','0')!='0':
        parser.error('Live check expects the existing real domain 0; never publishes fake feedback')
    rclpy.init()
    probe=rclpy.create_node('tcp_a_live_readonly_probe');status={};owner={}
    qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    probe.create_subscription(String,'/g1/tcp_a/status',lambda m:status.update(json.loads(m.data)),qos)
    probe.create_subscription(String,'/g1/task/console_status',lambda m:owner.update(json.loads(m.data)),qos)
    backend=None
    try:
        deadline=time.monotonic()+2
        while time.monotonic()<deadline:rclpy.spin_once(probe,timeout_sec=.1)
        if not probe.count_publishers('/g1/tcp_a/status'):
            backend=TcpANode(SimpleNamespace(config=ROOT/'src/g1_tcp_diagnostics/config/tcp_a.yaml',
                offline=False,session=None,robot_urdf=None,lowstate_topic='/lowstate',network='eth0',hand_peer='192.168.123.164'))
        deadline=time.monotonic()+10
        while time.monotonic()<deadline:
            if backend:rclpy.spin_once(backend,timeout_sec=.02)
            rclpy.spin_once(probe,timeout_sec=.02)
        report=dict(mode='live read-only; NO contact sample, NO hand read, NO control request',
            observed_at=time.time(),ready=status.get('ready'),reason=status.get('reason'),source=status.get('source'),
            model=status.get('model'),quality=status.get('quality'),gateway_phase=status.get('gateway_phase'),
            actual_joints=status.get('actual_joints'),tracking=status.get('tracking'),tracking_reason=status.get('tracking_reason'),
            owner_pid=owner.get('console_pid'),owner_controls=owner.get('tcp_a_controls'),
            publishers={topic:[dict(node=p.node_name,type=p.topic_type) for p in probe.get_publishers_info_by_topic(topic)]
                for topic in ('/lowstate','/arm_sdk','/robot_description','/g1/task/console_status')},
            hardware_contact_validated=False)
        if backend:
            topics=probe.get_publisher_names_and_types_by_node(backend.get_name(),backend.get_namespace())
            application=sorted(t for t,_ in topics if t not in ('/rosout','/parameter_events'))
            assert application==['/g1/tcp_a/markers','/g1/tcp_a/status'],application
            report['diagnostic_publisher_allowlist']=application
            with backend.data_lock:
                report['lowstate_window_frames']=len(backend.history)
                report['last_advancing_tick']=backend.last_tick
                report['latest_feedback_age_s']=time.monotonic()-backend.history[-1]['received'] if backend.history else None
            assert backend.hand is None and not backend.session['samples']
        args.output.parent.mkdir(parents=True,exist_ok=True)
        args.output.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
        print(json.dumps(report,ensure_ascii=False,indent=2))
    finally:
        if backend:
            backend.stop_event.set();backend.worker.join(timeout=3);backend.destroy_node()
        probe.destroy_node();rclpy.try_shutdown()


if __name__=='__main__':main()
