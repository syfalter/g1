"""Dedicated read-only executor for measured joints, separate from action RPCs."""
import threading

from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile,ReliabilityPolicy
from sensor_msgs.msg import JointState


class JointFeedback:
    def __init__(self,owner):
        self.node=Node('g1_task_measured_feedback',context=owner.context,
                       enable_rosout=False,start_parameter_services=False)
        self.subscription=self.node.create_subscription(JointState,'/joint_states',owner.joints,
            QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT))
        self.executor=SingleThreadedExecutor(context=owner.context)
        self.executor.add_node(self.node)
        self.thread=threading.Thread(target=self.executor.spin,name='measured-joints',daemon=True)

    def start(self):self.thread.start()

    def close(self):
        self.executor.shutdown();self.thread.join(timeout=3.);self.node.destroy_node()
