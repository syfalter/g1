"""Read-only nominal D435 axes. No robot commands or calibrated TF publication."""
from pathlib import Path
import argparse
import json
import os
import xml.etree.ElementTree as ET

import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import JointState

ROOT = Path(__file__).resolve().parents[1]


def markers():
    source = ET.parse(ROOT / 'assets/camera_reference/official_d435.urdf.xacro')
    properties = {e.get('name'): e.get('value') for e in source.iter()
                  if e.tag.endswith('}property')}
    offset = float(properties['d435_cam_depth_to_color_offset'])
    result = MarkerArray()

    def item(name, kind, color):
        m = Marker()
        m.header.frame_id = 'd435_link'
        m.ns = name
        m.id = len(result.markers)
        m.type = kind
        m.action = Marker.ADD
        m.pose.orientation.w = 1.
        m.frame_locked = True
        m.color.r, m.color.g, m.color.b, m.color.a = color
        result.markers.append(m)
        return m

    def arrow(name, start, end, color):
        m = item(name, Marker.ARROW, color)
        m.points = [Point(x=float(p[0]), y=float(p[1]), z=float(p[2])) for p in (start, end)]
        m.scale.x, m.scale.y, m.scale.z = .0025, .006, .015

    # RealSense optical +Z is body +X (optical rpy = -pi/2, 0, -pi/2).
    arrow('Nominal depth optical Z', (0, 0, 0), (.32, 0, 0), (.15, .55, 1., 1.))
    arrow('Nominal RGB optical Z', (0, offset, 0), (.32, offset, 0), (1., .5, .08, 1.))
    arrow('Original Unitree d435 Z', (0, 0, 0), (0, 0, .10), (.85, .25, .9, 1.))
    for name, y, color in [('Depth origin', 0., (.15, .55, 1., 1.)),
                            ('RGB origin', offset, (1., .5, .08, 1.))]:
        m = item(name, Marker.SPHERE, color)
        m.pose.position.y = y
        m.scale.x = m.scale.y = m.scale.z = .005
    body = item('Nominal housing - assumed frame origin', Marker.CUBE, (.7, .75, .8, .18))
    body.pose.position.y = -float(properties['d435_cam_depth_py'])
    body.scale.x = float(properties['d435_cam_depth'])
    body.scale.y = float(properties['d435_cam_width'])
    body.scale.z = float(properties['d435_cam_height'])
    for text, xyz in [
        ('NOMINAL ONLY: Unitree d435 = RealSense camera_link assumed', (0., 0., .15)),
        ('BLUE depth Z / ORANGE RGB Z (15 mm apart)', (.34, 0., -.02)),
        ('PURPLE: URDF Z, not optical Z', (0., 0., .115)),
    ]:
        m = item(text, Marker.TEXT_VIEW_FACING, (1., 1., 1., 1.))
        m.text = text
        m.pose.position = Point(x=xyz[0], y=xyz[1], z=xyz[2])
        m.scale.z = .013
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--offline-pose', type=Path)
    args, ros_args = parser.parse_known_args()
    if args.offline_pose and (os.environ.get('ROS_DOMAIN_ID') != '91' or
                              os.environ.get('ROS_LOCALHOST_ONLY') != '1'):
        raise RuntimeError('Offline joint snapshots require isolated localhost domain 91')
    rclpy.init(args=ros_args)
    node = rclpy.create_node('g1_camera_axis_preview_readonly')
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    pub = node.create_publisher(MarkerArray, '/g1/camera_axis_preview', qos)
    message = markers()
    if args.offline_pose:
        saved = json.loads(args.offline_pose.read_text())
        state = JointState()
        state.name = list(saved['joints'])
        state.position = list(saved['joints'].values())
        state_pub = node.create_publisher(JointState, '/joint_states', 10)
        def publish_state():
            state.header.stamp = node.get_clock().now().to_msg()
            state_pub.publish(state)
        node.create_timer(.1, publish_state)
        message.markers[-3].text = 'OFFLINE SNAPSHOT / NOMINAL CAMERA FRAME - NOT CALIBRATED'
    node.create_timer(.5, lambda: pub.publish(message))
    node.get_logger().info('Read-only nominal axes; no arm commands, no calibrated camera TF')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
