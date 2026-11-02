#!/usr/bin/python3
"""Bounded domain-96 integration check; never publishes robot state or commands."""
import argparse
import json
import os
from pathlib import Path
import time
import uuid

import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String
from visualization_msgs.msg import MarkerArray


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path,required=True);args=parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID')!='96' or os.environ.get('ROS_LOCALHOST_ONLY')!='1':
        parser.error('This check requires isolated localhost domain 96')
    rclpy.init();node=rclpy.create_node('tcp_a_offline_verifier');seen={};counts={}
    qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(String,'/g1/tcp_a/status',lambda m:seen.update(json.loads(m.data)),qos)
    node.create_subscription(MarkerArray,'/g1/tcp_a/markers',lambda m:counts.update(markers=len(m.markers),frames=list({v.header.frame_id for v in m.markers if v.action!=3})),qos)
    pub=node.create_publisher(String,'/g1/tcp_a/request',10)
    def wait(test,timeout=15):
        deadline=time.monotonic()+timeout
        while time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.1)
            if test():return
        raise RuntimeError('Timed out: '+json.dumps(seen,ensure_ascii=False)[:500])
    def send(op,**fields):
        pub.publish(String(data=json.dumps(dict(op=op,request_id=uuid.uuid4().hex,sent_at=time.time(),session_id=seen['session_id'],**fields))))
    try:
        wait(lambda:seen.get('mode')=='offline' and pub.get_subscription_count()>0)
        send('demo');wait(lambda:len(seen.get('samples',[]))==18 and not seen.get('busy'))
        send('solve');wait(lambda:isinstance(seen.get('result'),dict) and not seen.get('busy'))
        first=seen['result']['s_candidate_m'];assert seen['result']['rank']==6
        args.output.mkdir(parents=True,exist_ok=True)
        (args.output/'panel_status.json').write_text(json.dumps(seen,ensure_ascii=False,indent=2))
        sid=seen['samples'][0]['id'];send('select',sample_id=sid);wait(lambda:seen.get('selected')==sid)
        send('exclude',sample_id=sid);wait(lambda:seen['samples'][0]['excluded'] and seen.get('result') is None)
        send('exclude',sample_id=sid);wait(lambda:not seen['samples'][0]['excluded'])
        send('solve');wait(lambda:isinstance(seen.get('result'),dict))
        assert seen['result']['s_candidate_m']==first
        send('capture');wait(lambda:'离线模式禁止' in seen.get('message',''))
        assert len(seen['samples'])==18
        send('export');wait(lambda:'已导出' in seen.get('message','') and not seen.get('busy'))
        wait(lambda:counts.get('markers',0)>30)
        assert counts['frames']==['pelvis']
        endpoints={name:[(p.node_name,p.topic_type) for p in node.get_publishers_info_by_topic(name)]
                   for name in ('/arm_sdk','/lowcmd','/joint_states','/tf','/tf_static')}
        assert all(not v for v in endpoints.values()),endpoints
        nodes=node.get_node_names_and_namespaces()
        publishers=node.get_publisher_names_and_types_by_node('g1_tcp_a_diagnostics','/')
        application=[name for name,_ in publishers if name not in ('/rosout','/parameter_events')]
        assert sorted(application)==['/g1/tcp_a/markers','/g1/tcp_a/status'],application
        report=dict(result='PASS',mode='offline synthetic only',nodes=nodes,markers=counts,
                    forbidden_publishers=endpoints,candidate=first,export_message=seen['message'])
        (args.output/'integration.json').write_text(json.dumps(report,ensure_ascii=False,indent=2))
        print(json.dumps(report,ensure_ascii=False,indent=2))
    finally:
        node.destroy_node();rclpy.try_shutdown()


if __name__=='__main__':main()
