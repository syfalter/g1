from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


DEFAULT_MODEL_PATH = (
    "/home/syf/unitree_ws/unitree_ros/robots/g1_description/"
    "g1_29dof_lock_waist_rev_1_0.urdf"
)
DEFAULT_OUTPUT_DIR = "/home/syf/g1_reachability_data/g1_mode6/5mm_v0"


def _description(model_path):
    text = model_path.read_text(encoding="utf-8")
    mesh_uri = (model_path.parent / "meshes").resolve().as_uri()
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
    model_path = Path(LaunchConfiguration("model_path").perform(context)).resolve()
    if not model_path.is_file():
        raise RuntimeError(f"G1 URDF does not exist: {model_path}")
    package_share = Path(get_package_share_directory("g1_kdl_ik"))
    srdf = (package_share / "config" / "g1_mode6.srdf").read_text(encoding="utf-8")
    return [Node(
        package="g1_kdl_ik",
        executable="reachability_sampler",
        name="g1_reachability_sampler",
        output="screen",
        parameters=[
            {"robot_description": _description(model_path)},
            {"robot_description_semantic": srdf},
            {"model_path": str(model_path)},
            {"output_dir": LaunchConfiguration("output_dir")},
            {"samples_per_arm": LaunchConfiguration("samples_per_arm")},
            {"resolution": LaunchConfiguration("resolution")},
        ],
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("model_path", default_value=DEFAULT_MODEL_PATH),
        DeclareLaunchArgument("output_dir", default_value=DEFAULT_OUTPUT_DIR),
        DeclareLaunchArgument("samples_per_arm", default_value="1000000"),
        DeclareLaunchArgument("resolution", default_value="0.005"),
        OpaqueFunction(function=_setup),
    ])
