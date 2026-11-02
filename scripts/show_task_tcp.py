"""Display-only bridge for an already-running panel; exits with that panel."""
import argparse
from pathlib import Path
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, DurabilityPolicy
from visualization_msgs.msg import MarkerArray
from tf2_ros import Buffer, TransformListener

from task_preview_markers import live_tcp_markers, planning_tcp_offsets
from workbench_model import OfflineModel


def identity(pid):
    try:
        return Path(f'/proc/{pid}/stat').read_text().rsplit(')', 1)[1].split()[19]
    except (OSError, IndexError):
        return None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--panel-pid', type=int, required=True)
    args = parser.parse_args()
    original = identity(args.panel_pid)
    if original is None:
        raise SystemExit('Panel already exited')
    command = Path(f'/proc/{args.panel_pid}/cmdline').read_bytes().decode().split('\0')
    model = OfflineModel(command[command.index('--robot-urdf') + 1])
    offsets = planning_tcp_offsets(model)
    rclpy.init()
    node = rclpy.create_node('g1_live_tcp_visualization')
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    publisher = node.create_publisher(MarkerArray, '/g1/vision_task_preview', qos)
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    last_publish, reported = 0., False
    try:
        while rclpy.ok() and identity(args.panel_pid) == original:
            rclpy.spin_once(node, timeout_sec=.1)
            now = time.monotonic()
            if now - last_publish >= 1.:
                publisher.publish(live_tcp_markers(offsets))
                last_publish = now
                ready = all(buffer.can_transform('pelvis', side + '_wrist_yaw_link', rclpy.time.Time())
                            for side in ('left', 'right'))
                subscribers = node.get_subscriptions_info_by_topic('/g1/vision_task_preview')
                rviz = any('rviz' in entry.node_name for entry in subscribers)
                if ready and rviz and not reported:
                    node.get_logger().info('Planning index_only TCP markers published; both live wrist TFs available; RViz subscriber discovered.')
                    reported = True
    except ExternalShutdownException:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
