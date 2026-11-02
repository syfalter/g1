from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


DEFAULT_MODEL_PATH = (
    "/home/syf/unitree_ws/unitree_ros/robots/g1_description/"
    "g1_29dof_lock_waist_rev_1_0.urdf"
)


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
    description = _description(model_path)
    srdf = (package_share / "config" / "g1_mode6.srdf").read_text(encoding="utf-8")
    with (package_share / "config" / "kinematics.yaml").open(encoding="utf-8") as stream:
        kinematics = yaml.safe_load(stream)
    rviz_config = str(
        Path(get_package_share_directory("g1_mode6_description"))
        / "rviz" / "g1_mode6.rviz"
    )
    robot_description = {"robot_description": description}
    return [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[robot_description],
            output="screen",
        ),
        Node(
            package="g1_kdl_ik",
            executable="kdl_ik_server",
            name="g1_kdl_ik_server",
            parameters=[
                robot_description,
                {"robot_description_semantic": srdf},
                {"robot_description_kinematics": kinematics},
                {"default_timeout": 0.05},
                {"default_motion_duration": LaunchConfiguration("duration")},
                {"max_visual_joint_speed": LaunchConfiguration("max_joint_speed")},
                {"publish_rate": 50.0},
                {"run_self_test": False},
            ],
            output="screen",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", rviz_config],
            output="screen",
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("model_path", default_value=DEFAULT_MODEL_PATH),
        DeclareLaunchArgument("duration", default_value="2.0"),
        DeclareLaunchArgument("max_joint_speed", default_value="0.5"),
        OpaqueFunction(function=_setup),
    ])
