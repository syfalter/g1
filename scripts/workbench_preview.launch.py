"""Isolated, static RViz preview. No gateway, DDS robot bridge, or execution."""
import json
import math
import sys
from pathlib import Path
import xml.etree.ElementTree as ET

import xacro
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
from preview_payload import apply_preview_payload


def build_scene(context):
    height = float(LaunchConfiguration("table_height").perform(context))
    gap = float(LaunchConfiguration("table_gap").perform(context))
    retreat = float(LaunchConfiguration("robot_retreat").perform(context))
    platform_height = float(LaunchConfiguration("platform_height").perform(context))
    if not math.isfinite(platform_height) or not 0 <= platform_height <= .3:
        raise ValueError("Platform height must be within 0 to 0.3 m")
    if not math.isfinite(retreat) or not 0 <= retreat <= 0.8:
        raise ValueError("Preview robot retreat must be within 0 to 0.8 m")
    if not 0.1 <= height <= 2.0 or not 0 <= gap <= 2.0:
        raise ValueError("Invalid preview height or gap")
    meta = json.loads((ROOT / "assets/workbench/metadata.json").read_text())
    front = float(LaunchConfiguration("pelvis_front_offset").perform(context))
    pitch = math.radians(float(LaunchConfiguration("table_pitch_deg").perform(context)))
    yaw = math.radians(float(LaunchConfiguration("table_yaw_deg").perform(context)))
    align_bottom = LaunchConfiguration("align_bottom_to_feet").perform(context).lower() == "true"
    lo, hi = meta["bounds_m"]
    cx, cz = (lo[0] + hi[0]) / 2, (lo[2] + hi[2]) / 2
    c, s = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rotated_min_x = min(
        cy * (c * x + s * z) - sy * y
        for x in (lo[0], hi[0])
        for y in (lo[1], hi[1])
        for z in (lo[2], hi[2])
    )
    ty = -(sy * (c * cx + s * cz) + cy * (lo[1] + hi[1]) / 2)
    # Preserve the original vertical centre while correcting the CAD orientation.
    # The old top face is not a valid work-surface height reference after rotation.
    tx = front + gap - rotated_min_x
    tz = height - meta["tabletop_local_z_m"] + cz - (-s * cx + c * cz)
    sole_z = 0.793 + meta["zero_pose_sole_z_relative_pelvis_m"]
    rotated_min_z = min(-s * x + c * z for x in (lo[0], hi[0]) for z in (lo[2], hi[2]))
    if align_bottom:
        tz = sole_z - rotated_min_z
    xml = xacro.process_file(str(ROOT / "src/g1_mode6_description/urdf/g1_upper_body.urdf.xacro")).toxml()
    robot = ET.fromstring(xml)
    apply_preview_payload(robot, json.loads((ROOT / "assets/workbench/payload_preview.json").read_text()))
    profile = json.loads(Path(LaunchConfiguration("initial_pose_file").perform(context)).read_text())
    initial_joints = profile["joints"]
    for name, value in initial_joints.items():
        joint = robot.find(f"joint[@name='{name}']")
        if joint is None or not math.isfinite(value):
            raise ValueError(f"Invalid initial joint: {name}")
        limit = joint.find("limit")
        if limit is not None and not float(limit.get("lower", "-inf")) <= value <= float(limit.get("upper", "inf")):
            raise ValueError(f"Initial pose outside joint limits: {name}")
    mesh_dir = Path("/home/syf/unitree_ws/unitree_ros/robots/g1_description/meshes")
    for mesh in robot.iter("mesh"):
        name = mesh.get("filename", "")
        if name.startswith("meshes/"):
            mesh.set("filename", (mesh_dir / name[7:]).as_uri())
    ET.SubElement(robot, "link", name="world")
    joint = ET.SubElement(robot, "joint", name="preview_world_pelvis", type="fixed")
    ET.SubElement(joint, "parent", link="world")
    ET.SubElement(joint, "child", link="pelvis")
    ET.SubElement(joint, "origin", xyz=f"{-retreat} 0 {0.793 + platform_height}", rpy="0 0 0")
    table = ET.SubElement(robot, "link", name="workbench")
    for kind in ("visual", "collision"):
        element = ET.SubElement(table, kind)
        geometry = ET.SubElement(element, "geometry")
        ET.SubElement(geometry, "mesh", filename=(ROOT / "assets/workbench/workbench.stl").as_uri())
        if kind == "visual":
            material = ET.SubElement(element, "material", name="workbench_gray")
            ET.SubElement(material, "color", rgba="0.58 0.65 0.68 1")
    joint = ET.SubElement(robot, "joint", name="preview_world_workbench", type="fixed")
    ET.SubElement(joint, "parent", link="world")
    ET.SubElement(joint, "child", link="workbench")
    ET.SubElement(joint, "origin", xyz=f"{tx} {ty} {tz}", rpy=f"0 {pitch} {yaw}")
    description = {"robot_description": ET.tostring(robot, encoding="unicode")}
    print(f"STATIC PREVIEW: pitch={math.degrees(pitch)}deg, yaw={math.degrees(yaw)}deg, gap={gap}m, front offset={front}m; height reference pending orientation confirmation; NOT live robot state")
    print(f"Bottom alignment={align_bottom}: cabinet bottom={tz + rotated_min_z:.6f}m, "
          f"zero-pose sole reference={sole_z:.6f}m; measured base/world registration not applied")
    print(f"Preview robot retreat={retreat:.3f}m; actual pelvis-front/table-edge gap={gap + retreat:.3f}m")
    print(f"Platform height={platform_height:.3f}m; robot raised only, cabinet remains at ground reference; "
          "platform footprint/collision geometry not measured")
    source = profile.get("source", "Unspecified reference")
    print(f"OFFLINE initial reference: right={profile.get('right_arm_source', source)}; "
          f"left={profile.get('left_arm_source', source)}; "
          f"R2+A verified={profile.get('r2_a_pose_verified', False)}; NOT live robot state")
    return description, initial_joints


def setup(context):
    description, initial_joints = build_scene(context)
    return [
        Node(package="robot_state_publisher", executable="robot_state_publisher", parameters=[description]),
        Node(package="joint_state_publisher", executable="joint_state_publisher", parameters=[description, {"zeros": initial_joints}]),
        Node(package="rviz2", executable="rviz2", arguments=["-d", str(ROOT / "assets/workbench/preview.rviz")]),
    ]


def generate_launch_description():
    meta = json.loads((ROOT / "assets/workbench/metadata.json").read_text())
    return LaunchDescription([
        DeclareLaunchArgument("table_height", default_value="0.90"),
        DeclareLaunchArgument("table_gap", default_value="0.05"),
        DeclareLaunchArgument("robot_retreat", default_value="0.06"),
        DeclareLaunchArgument("platform_height", default_value="0.07"),
        DeclareLaunchArgument("table_pitch_deg", default_value="-90"),
        DeclareLaunchArgument("table_yaw_deg", default_value="180"),
        DeclareLaunchArgument("align_bottom_to_feet", default_value="true"),
        DeclareLaunchArgument("initial_pose_file", default_value=str(ROOT / "assets/workbench/task_start_pose.json")),
        DeclareLaunchArgument("pelvis_front_offset", default_value=str(meta["pelvis_front_offset_m"])),
        OpaqueFunction(function=setup),
    ])
