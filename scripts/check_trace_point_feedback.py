"""Exercise visualization-only click feedback on the isolated offline domain."""
import os
import time

import rclpy
from moveit_msgs.msg import DisplayRobotState
from visualization_msgs.msg import InteractiveMarkerFeedback
from visualization_msgs.srv import GetInteractiveMarkers


def main():
    if os.environ.get('ROS_DOMAIN_ID') != '89' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        raise RuntimeError('This check requires the isolated offline editor environment')
    rclpy.init()
    node = rclpy.create_node('offline_point_feedback_check')
    try:
        states=[]
        node.create_subscription(DisplayRobotState,'/g1/task/demo_preview_state',states.append,10)
        client=node.create_client(GetInteractiveMarkers,'/g1/task/trace_points/get_interactive_markers')
        assert client.wait_for_service(timeout_sec=10), 'No picker service'
        markers=[];deadline=time.monotonic()+10
        while not markers and time.monotonic()<deadline:
            future=client.call_async(GetInteractiveMarkers.Request())
            rclpy.spin_until_future_complete(node,future,timeout_sec=2)
            if future.done() and future.result():
                markers=[m for m in future.result().markers if any(c.name=='select' for c in m.controls)]
            if not markers:rclpy.spin_once(node,timeout_sec=.1)
        assert len(markers) >= 1, len(markers)
        subscribers=node.get_subscriptions_info_by_topic('/g1/task/trace_points/update')
        assert any('rviz' in info.node_name for info in subscribers), 'RViz has not connected to the point picker namespace'
        assert not any('gateway' in name or 'arm_state_bridge' in name for name in node.get_node_names())
        publisher=node.create_publisher(InteractiveMarkerFeedback,'/g1/task/trace_points/feedback',10)
        deadline=time.monotonic()+5
        while publisher.get_subscription_count()==0 and time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.1)
        states.clear()
        feedback=InteractiveMarkerFeedback()
        feedback.marker_name=markers[len(markers)//2].name
        feedback.control_name='select';feedback.client_id='offline_smoke_check'
        feedback.event_type=InteractiveMarkerFeedback.BUTTON_CLICK
        feedback.header=markers[len(markers)//2].header
        feedback.pose=markers[len(markers)//2].pose
        publisher.publish(feedback)
        deadline=time.monotonic()+8
        while not states and time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.1)
        assert states and not states[-1].hide, 'Click did not produce a robot preview'
        names=states[-1].state.joint_state.name
        assert any(side+'_index_1_joint' in names and side+'_shoulder_pitch_joint' in names
                   for side in ('left','right')), names
        print(f'PASS: {len(markers)} sparse clickable points; click produced arm + finger preview; no gateway')
    finally:
        node.destroy_node();rclpy.shutdown()


if __name__ == '__main__':main()
