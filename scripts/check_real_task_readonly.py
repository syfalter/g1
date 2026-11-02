"""Inspect the disconnected application domain; never publish robot state or commands."""
import json
import os
from pathlib import Path
import time
import sys
import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from moveit_msgs.srv import GetPlanningScene
from moveit_msgs.msg import PlanningSceneComponents


def main():
    live='--live' in sys.argv
    if live and os.environ.get('ROS_DOMAIN_ID')!='0':
        raise RuntimeError('Live check requires domain 0')
    if not live and (os.environ.get('ROS_DOMAIN_ID')!='88' or os.environ.get('ROS_LOCALHOST_ONLY')!='1'):
        raise RuntimeError('Disconnected localhost domain 88 required')
    os.environ.setdefault('CYCLONEDDS_URI','<CycloneDDS><Domain><Discovery><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>80</MaxAutoParticipantIndex></Discovery></Domain></CycloneDDS>')
    rclpy.init(); node=rclpy.create_node('real_task_readonly_check')
    result={}; clouds=[]; states=[]
    from sensor_msgs.msg import PointCloud2, JointState
    def diagnostics(message):
        for status in message.status: result.update({v.key:v.value for v in status.values})
    node.create_subscription(DiagnosticArray,'/g1/arm_gateway/diagnostics',diagnostics,10)
    node.create_subscription(PointCloud2,'/g1/vision_rgbd_surface',lambda msg:clouds.append(msg),10)
    node.create_subscription(JointState,'/joint_states',lambda msg:states.append(msg),10)
    try:
        deadline=time.monotonic()+3
        while time.monotonic()<deadline: rclpy.spin_once(node,timeout_sec=.05)
        assert result.get('hardware_output_available')=='false',result
        assert result.get('command_publisher_exists')=='false',result
        if not live:
            assert node.count_publishers('/arm_sdk')==0
            assert not clouds and not states,'Unexpected historical or simulated data on the live/snapshot topics'
        publishers=node.get_publishers_info_by_topic('/joint_states')
        assert [p.node_name for p in publishers]==['arm_state_bridge'],publishers
        scene=node.create_client(GetPlanningScene,'/get_planning_scene')
        assert scene.wait_for_service(timeout_sec=2)
        request=GetPlanningScene.Request()
        request.components.components=PlanningSceneComponents.WORLD_OBJECT_NAMES
        future=scene.call_async(request)
        rclpy.spin_until_future_complete(node,future,timeout_sec=4)
        assert future.done()
        objects=[o.id for o in future.result().scene.world.collision_objects]
        assert all(name=='task_measured_rgbd_snapshot' for name in objects),'Old CAD in task scene'
        output={'domain':0 if live else 88,'hardware_output_available':False,
                'gateway_command_publisher_exists':False,'global_arm_sdk_publishers':node.count_publishers('/arm_sdk'),
                'joint_state_publisher':'arm_state_bridge','received_actual_joint_states':len(states),
                'mode_machine':result.get('mode_machine'),'mode_pr':result.get('mode_pr'),
                'received_snapshot_messages':len(clouds),'old_cad_objects':0,
                'task_gateway_enabled':result['task_application_enabled']}
        directory=Path(__file__).resolve().parents[1]/'logs/task_application_gateway_test'
        (directory/('readonly_live_app.json' if live else 'readonly_app.json')).write_text(json.dumps(output,indent=2)+'\n')
        print(json.dumps(output,indent=2))
    finally: node.destroy_node(); rclpy.shutdown()


if __name__=='__main__': main()
