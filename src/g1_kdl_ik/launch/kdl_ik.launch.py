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


def _load_robot_description(model_path: Path) -> str:
    text = model_path.read_text(encoding="utf-8")
    mesh_uri = (model_path.parent / "meshes").resolve().as_uri()
    text = text.replace('filename="meshes/', f'filename="{mesh_uri}/')

    # These explicit TCP links are coincident with the official rubber-hand
    # links for now. Their transforms can be calibrated later without changing
    # the arm groups or the IK service API.
    tcp_xml = """
  <link name="left_tool0"/>
  <joint name="left_tool0_joint" type="fixed">
    <parent link="left_rubber_hand"/>
    <child link="left_tool0"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
  </joint>
  <link name="right_tool0"/>
  <joint name="right_tool0_joint" type="fixed">
    <parent link="right_rubber_hand"/>
    <child link="right_tool0"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
  </joint>
"""
    return text.replace("</robot>", tcp_xml + "</robot>")


def _launch_setup(context):
    model_path = Path(LaunchConfiguration("model_path").perform(context)).resolve()
    if not model_path.is_file():
        raise RuntimeError(f"G1 URDF does not exist: {model_path}")

    package_share = Path(get_package_share_directory("g1_kdl_ik"))
    srdf = (package_share / "config" / "g1_mode6.srdf").read_text(encoding="utf-8")
    with (package_share / "config" / "kinematics.yaml").open(encoding="utf-8") as stream:
        kinematics = yaml.safe_load(stream)

    return [
        Node(
            package="g1_kdl_ik",
            executable="kdl_ik_server",
            name="g1_kdl_ik_server",
            output="screen",
            parameters=[
                {"robot_description": _load_robot_description(model_path)},
                {"robot_description_semantic": srdf},
                {"robot_description_kinematics": kinematics},
                {"default_timeout": 0.05},
                {"default_motion_duration": 2.0},
                {"max_visual_joint_speed": 0.5},
                {"publish_rate": 50.0},
                {"run_self_test": LaunchConfiguration("run_self_test")},
            ],
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("model_path", default_value=DEFAULT_MODEL_PATH),
            DeclareLaunchArgument("run_self_test", default_value="false"),
            OpaqueFunction(function=_launch_setup),
        ]
    )
