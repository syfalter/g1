from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


MODEL_PATH = Path(
    "/home/syf/unitree_ws/unitree_ros/robots/g1_description/"
    "g1_29dof_lock_waist_rev_1_0.urdf"
)
OUTPUT_DIR = "/home/syf/g1_reachability_data/g1_mode6/5mm_front_v0"


def _description():
    text = MODEL_PATH.read_text(encoding="utf-8")
    mesh_uri = (MODEL_PATH.parent / "meshes").resolve().as_uri()
    text = text.replace('filename="meshes/', f'filename="{mesh_uri}/')
    tcp_xml = """
  <link name="left_tool0"/>
  <joint name="left_tool0_joint" type="fixed">
    <parent link="left_rubber_hand"/><child link="left_tool0"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
  </joint>
  <link name="right_tool0"/>
  <joint name="right_tool0_joint" type="fixed">
    <parent link="right_rubber_hand"/><child link="right_tool0"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
  </joint>
"""
    return text.replace("</robot>", tcp_xml + "</robot>")


def _setup(context):
    package_share = Path(get_package_share_directory("g1_kdl_ik"))
    srdf = (package_share / "config" / "g1_mode6.srdf").read_text(encoding="utf-8")
    return [Node(
        package="g1_kdl_ik",
        executable="reachability_sampler",
        name="g1_front_reachability_sampler",
        output="screen",
        parameters=[
            {"robot_description": _description()},
            {"robot_description_semantic": srdf},
            {"model_path": str(MODEL_PATH)},
            {"output_dir": LaunchConfiguration("output_dir")},
            {"samples_per_arm": LaunchConfiguration("samples_per_arm")},
            {"resolution": 0.005},
            {"sampling_mode": "front"},
            {"roi_min": [0.15, -0.40, -0.15]},
            {"roi_max": [0.43, 0.40, 0.35]},
            {"local_range_fraction": 0.30},
        ],
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("output_dir", default_value=OUTPUT_DIR),
        DeclareLaunchArgument("samples_per_arm", default_value="5000000"),
        OpaqueFunction(function=_setup),
    ])
