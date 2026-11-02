"""Publish a read-only RViz preview of transformed vision task geometry."""
from pathlib import Path
import argparse
import hashlib
import json
import os

import numpy as np
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.qos import DurabilityPolicy, QoSProfile
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState, PointCloud2, PointField
from visualization_msgs.msg import Marker, MarkerArray


def load_cloud(directory, geometry):
    directory = Path(directory)
    manifest = json.loads((directory / 'manifest.json').read_text())
    if (manifest['source_sequence'] != geometry['source']['sequence']
            or manifest['source_request_id'] != geometry['source']['request_id']
            or manifest['frame_id'] != 'torso_link'
            or not np.allclose(manifest['torso_from_color_optical_row_major'],
                               geometry['transforms'].get('uncalibrated_torso_from_color_optical_row_major',
                                   geometry['transforms']['torso_from_color_optical_row_major']))):
        raise ValueError('Cloud and target preview refer to different captures/transforms')
    path = directory / 'cloud.npz'
    if hashlib.sha256(path.read_bytes()).hexdigest() != manifest['sha256']['cloud.npz']:
        raise ValueError('Archived cloud checksum mismatch')
    with np.load(path, allow_pickle=False) as saved:
        xyz, rgb = saved['xyz'], saved['rgb']
    if (xyz.ndim != 2 or xyz.shape[1] != 3 or rgb.shape != xyz.shape
            or rgb.dtype != np.uint8 or not np.isfinite(xyz).all()
            or not 0 < len(xyz) <= 1280*720):
        raise ValueError('Invalid point cloud arrays')
    packed = np.empty(len(xyz), dtype=[('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('rgb', '<u4')])
    from task_calibration import cloud_points
    xyz=cloud_points(xyz,geometry)
    for i, axis in enumerate('xyz'):
        packed[axis] = xyz[:, i]
    color = rgb.astype(np.uint32)
    packed['rgb'] = color[:, 0] << 16 | color[:, 1] << 8 | color[:, 2]
    message = PointCloud2()
    message.header.frame_id = 'torso_link'
    message.height, message.width = 1, len(xyz)
    message.fields = [PointField(name=name, offset=i*4, datatype=PointField.FLOAT32, count=1)
                      for i, name in enumerate(('x', 'y', 'z', 'rgb'))]
    message.is_bigendian = False
    message.point_step, message.row_step = 16, 16*len(xyz)
    message.data, message.is_dense = packed.tobytes(), True
    return message


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("artifact", type=Path)
    parser.add_argument("--offline-pose", type=Path, required=True)
    parser.add_argument("--arm", choices=("left", "right", "both"), default="left")
    parser.add_argument('--rgbd-dir', type=Path)
    args, ros_args = parser.parse_known_args()
    if os.environ.get("ROS_DOMAIN_ID") != "92" or os.environ.get("ROS_LOCALHOST_ONLY") != "1":
        raise RuntimeError("Vision task preview requires isolated localhost ROS domain 92")
    data = json.loads(args.artifact.read_text())
    pose = json.loads(args.offline_pose.read_text())
    cloud = load_cloud(args.rgbd_dir, data) if args.rgbd_dir else None

    rclpy.init(args=ros_args)
    node = rclpy.create_node("g1_vision_task_preview_readonly")
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    marker_pub = node.create_publisher(MarkerArray, "/g1/vision_task_preview", qos)
    state_pub = node.create_publisher(JointState, "/joint_states", 10)
    cloud_pub = node.create_publisher(PointCloud2, '/g1/vision_rgbd_surface', qos)
    result = MarkerArray()

    def marker(name, marker_type, color, frame="torso_link"):
        item = Marker()
        item.header.frame_id = frame
        item.ns = name
        item.id = len(result.markers)
        item.type = marker_type
        item.action = Marker.ADD
        item.pose.orientation.w = 1.0
        item.frame_locked = True
        item.color.r, item.color.g, item.color.b, item.color.a = color
        result.markers.append(item)
        return item

    def sphere(name, xyz, color, scale=0.014, frame="torso_link"):
        item = marker(name, Marker.SPHERE, color, frame)
        item.pose.position = Point(x=float(xyz[0]), y=float(xyz[1]), z=float(xyz[2]))
        item.scale.x = item.scale.y = item.scale.z = scale

    def arrow(name, start, end, color, frame="torso_link"):
        item = marker(name, Marker.ARROW, color, frame)
        item.points = [Point(x=float(p[0]), y=float(p[1]), z=float(p[2])) for p in (start, end)]
        item.scale.x, item.scale.y, item.scale.z = 0.004, 0.009, 0.016

    task = data["task_preview"]
    axis = np.asarray(task["axis_center_m"])
    raw = np.asarray(task["operation_point_raw_m"])
    projected = np.asarray(task["operation_point_projected_m"])
    entry = np.asarray(task.get('insertion_reference_m', projected))
    preinsert = np.asarray(task["preinsert_point_m"])
    sphere("Raw shaft center candidate", axis, (0.95, 0.2, 0.2, 1.0), 0.018)
    sphere("Raw operation point", raw, (1.0, 0.72, 0.1, 1.0), 0.014)
    sphere("Selected insertion reference", entry, (0.1, 0.9, 0.78, 1.0), 0.014)
    if 'ray_plane_intersections_m' in task:
        source = np.asarray(task['selected_source_point_raw_m'])
        intersection = np.asarray(task['ray_plane_intersections_m'][task['entry_source_key']])
        sphere('Old Z-only reference - comparison', projected, (.65, .65, .65, .7), .009)
        sphere('Raw pre-offset diagonal point', data['points_torso_raw_m']['operation_diagonal_intersection'],
               (.95, .9, .2, .8), .008)
        old_ray = task['ray_plane_intersections_m']['operation_point']
        if old_ray is not None:
            sphere('Upstream outward point on plane - comparison', old_ray, (1., .45, .1, .85), .009)
        arrow('Selected ray to horizontal plane', source, intersection, (.95, .9, .2, .8))
        ray = marker('Selected camera ray', Marker.LINE_STRIP, (.2, .65, 1., .55))
        ray.scale.x = .0015
        camera_origin = np.asarray(data['transforms']['torso_from_color_optical_row_major'])[:3, 3]
        ray.points = [Point(**dict(zip('xyz', np.asarray(p).tolist()))) for p in (camera_origin, source)]
    else:
        arrow("Raw depth plane correction", raw, projected, (1.0, 0.72, 0.1, 0.9))
    arrow("Assumed shaft top normal", axis, axis + np.array([0.0, 0.0, 0.12]), (0.9, 0.25, 0.85, 1.0))
    arrow("Preinsert approach", preinsert, entry, (0.2, 0.75, 1.0, 1.0))
    arc = marker("Provisional +/-120 degree sweep", Marker.LINE_STRIP, (0.1, 0.9, 0.78, 1.0))
    arc.scale.x = 0.006
    arc.points = [Point(x=float(p[0]), y=float(p[1]), z=float(p[2])) for p in task["arc_minus120_to_plus120_m"]]

    transform = np.asarray(data["transforms"]["torso_from_color_optical_row_major"])
    origin = transform[:3, 3]
    for index, (name, color) in enumerate(
        (("RGB optical X", (1.0, 0.2, 0.2, 1.0)),
         ("RGB optical Y", (0.2, 1.0, 0.2, 1.0)),
         ("RGB optical Z", (0.2, 0.55, 1.0, 1.0)))
    ):
        arrow(name, origin, origin + transform[:3, index] * 0.14, color)

    for side in ("left", "right"):
        frame = f"{side}_wrist_yaw_link"
        states = data["fingertip_estimates"]["sides"][side]["states"]
        open_tip = states["open"]["wrist_yaw_from_index_tcp"]["xyz_m"]
        folded_tip = states["folded"]["wrist_yaw_from_index_tcp"]["xyz_m"]
        sphere(f"{side} open index TCP estimate", open_tip, (0.2, 0.8, 1.0, 1.0), 0.012, frame)
        sphere(f"{side} folded index TCP estimate", folded_tip, (0.95, 0.42, 0.16, 1.0), 0.010, frame)
        arrow(f"{side} wrist to open index TCP", (0.0, 0.0, 0.0), open_tip,
              (0.2, 0.8, 1.0, 0.8), frame)

    for side, candidate in data.get("grasp_candidates", {}).items():
        if args.arm not in (side, "both"):
            continue
        poses = candidate["poses"]
        for name in ("wrist_yaw", "index_tcp"):
            pose_item = poses[name]
            center = np.asarray(pose_item["position_m"])
            rotation = Rotation.from_quat(pose_item["quaternion_xyzw"]).as_matrix()
            for i, color in enumerate(((1., .2, .2, 1.), (.2, 1., .2, 1.), (.2, .5, 1., 1.))):
                arrow(f"{side} candidate {name} {'XYZ'[i]}", center, center + rotation[:, i] * .06, color)
        arrow(f"{side} tilted insertion candidate", poses["preinsert_index_tcp"]["position_m"],
              poses["index_tcp"]["position_m"], (.9, .9, .9, 1.))

    label = marker("Preview status", Marker.TEXT_VIEW_FACING, (1.0, 1.0, 1.0, 1.0))
    label.pose.position = Point(x=float(axis[0]), y=float(axis[1]), z=float(axis[2] + 0.16))
    label.scale.z = 0.014
    quality = data["quality"]
    label.text = (
        f"{data['source']['capture_kind'].upper()} / REVIEW ONLY\n"
        f"confidence={quality['stage2_confidence']:.3f}  "
        f"depth/plane={quality.get('selected_plane_delta_m', quality['operation_plane_delta_m']) * 1000:.1f} mm  "
        f"radius={quality['entry_radius_m'] * 1000:.1f} mm\n"
        f"Entry: {task.get('entry_source_key', 'legacy')} / radial={task.get('radial_offset_m', 0.)*1000:+.0f} mm / height={task.get('height_offset_m', 0.)*1000:+.0f} mm\n"
        f"Robot snapshot: {pose.get('captured_at', 'unknown')[:19]} / NOT LIVE\n"
        f"RGB-D: {cloud.width if cloud else 0} points / NOMINAL CAMERA MOUNT"
    )

    state = JointState()
    state.name = list(pose["joints"])
    state.position = list(pose["joints"].values())

    def publish():
        now = node.get_clock().now().to_msg()
        state.header.stamp = now
        state_pub.publish(state)
        marker_pub.publish(result)

    def publish_cloud():
        if cloud is not None:
            cloud.header.stamp = node.get_clock().now().to_msg()
            cloud_pub.publish(cloud)

    node.create_timer(0.2, publish)
    node.create_timer(1.0, publish_cloud)
    node.get_logger().info(
        "Read-only nominal vision task preview; no arm, hand, or camera commands"
    )
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
