"""Visualization-only playback. Never publish measured or command joint states."""
import argparse
import bisect
import json
import math
import os
from pathlib import Path
import time
import xml.etree.ElementTree as ET

import numpy as np

from task_trajectory import coefficients, time_seconds


class PreviewTimeline:
    def __init__(self, names):
        self.values = dict.fromkeys(names, 0.)
        self.segments = []
        self.started = 0.

    def state(self, message):
        # Hide is an overlay command, not a new pose. The sole offline model
        # stays at the last pose until another preview arrives.
        if message.hide:
            return
        joint = message.state.joint_state
        if len(joint.name) != len(joint.position) or not all(map(math.isfinite, joint.position)):
            raise ValueError('Invalid preview state')
        self.segments = []
        self.values.update((n, v) for n, v in zip(joint.name, joint.position) if n in self.values)

    def trajectory(self, message, now):
        segments = []
        offset = 0.
        for robot in message.trajectory:
            tr = robot.joint_trajectory
            if not tr.points:
                continue
            knots = [time_seconds(p) for p in tr.points]
            if (knots[0] < 0 or not all(map(math.isfinite, knots)) or
                    any(b <= a for a, b in zip(knots, knots[1:])) or
                    any(len(p.positions) != len(tr.joint_names) or
                        not all(map(math.isfinite, p.positions)) for p in tr.points)):
                raise ValueError('Invalid preview trajectory')
            segments.append((offset, tr, knots))
            offset += knots[-1]
        self.segments = segments
        self.started = now
        joint = message.trajectory_start.joint_state
        if len(joint.name) == len(joint.position):
            self.values.update((n, v) for n, v in zip(joint.name, joint.position)
                               if n in self.values and math.isfinite(v))

    def sample(self, now):
        elapsed = max(0., now-self.started)
        for offset, tr, knots in self.segments:
            t = elapsed-offset
            if t >= knots[-1]:
                positions = tr.points[-1].positions
            elif t <= knots[0]:
                positions = tr.points[0].positions
            else:
                i = bisect.bisect_right(knots, t)-1
                dt = knots[i+1]-knots[i]
                poly = coefficients(tr.points[i], tr.points[i+1], dt)
                positions = np.polynomial.polynomial.polyval((t-knots[i])/dt, poly)
            self.values.update((n, float(v)) for n, v in zip(tr.joint_names, positions)
                               if n in self.values)
            if t < knots[-1]:
                break
        return self.values.copy()


def main():
    if os.environ.get('ROS_DOMAIN_ID') != '89' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        raise RuntimeError('Offline preview requires localhost-only ROS domain 89')
    parser = argparse.ArgumentParser()
    parser.add_argument('--robot-urdf', type=Path, required=True)
    args = parser.parse_args()
    robot = ET.parse(args.robot_urdf).getroot()
    names = [j.get('name') for j in robot.findall('joint')
             if j.get('type') in ('revolute', 'continuous', 'prismatic') and j.find('mimic') is None]
    children = {j.find('child').get('link') for j in robot.findall('joint')}
    root = next(link.get('name') for link in robot.findall('link') if link.get('name') not in children)

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, DurabilityPolicy
    from moveit_msgs.msg import DisplayRobotState, DisplayTrajectory
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String
    from geometry_msgs.msg import TransformStamped
    from tf2_ros import StaticTransformBroadcaster

    rclpy.init()
    node = Node('g1_offline_preview')
    timeline = PreviewTimeline(names)
    publisher = node.create_publisher(JointState, '/g1/offline_preview/joint_states', 1)
    health = node.create_publisher(String, '/g1/offline_preview/status', 1)
    period = {'last':time.monotonic(), 'since':time.monotonic(), 'count':0, 'max_gap':0.}
    fixed = StaticTransformBroadcaster(node)
    transform = TransformStamped()
    transform.header.frame_id = root
    transform.child_frame_id = 'offline_preview/'+root
    transform.transform.rotation.w = 1.
    transform.header.stamp = node.get_clock().now().to_msg()
    fixed.sendTransform(transform)

    def receive(message):
        try:
            if isinstance(message, DisplayTrajectory):
                timeline.trajectory(message, time.monotonic())
            else:
                timeline.state(message)
        except (ValueError, TypeError, IndexError) as error:
            timeline.segments = []
            node.get_logger().error(str(error))

    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    subscriptions = [node.create_subscription(kind, topic, receive, qos) for kind, topic in (
        (DisplayRobotState, '/g1/task/demo_preview_state'),
        (DisplayTrajectory, '/g1/task/validated_path'))]

    def tick():
        now = time.monotonic()
        period['max_gap'] = max(period['max_gap'], now-period['last'])
        period['last'] = now
        period['count'] += 1
        values = timeline.sample(now)
        message = JointState(name=list(values), position=list(values.values()))
        message.header.stamp = node.get_clock().now().to_msg()
        publisher.publish(message)
        if now-period['since'] >= 1.:
            health.publish(String(data=json.dumps(dict(hz=period['count']/(now-period['since']),
                max_gap_ms=period['max_gap']*1000, trajectory_active=bool(timeline.segments)))))
            period.update(since=now, count=0, max_gap=0.)

    # Playback and its scheduler share the monotonic clock. A WSL wall-clock
    # correction must not suspend a ROS-time timer and then skip ahead in poses.
    deadline = time.monotonic()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=max(0., min(.01, deadline-time.monotonic())))
            now = time.monotonic()
            if now >= deadline:
                tick()
                deadline += 1./30.
                if deadline <= now:deadline = now+1./30.
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
