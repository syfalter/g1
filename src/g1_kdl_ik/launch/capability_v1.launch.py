from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


DEFAULT_MODEL_PATH = (
    "/home/syf/unitree_ws/unitree_ros/robots/g1_description/"
    "g1_29dof_lock_waist_rev_1_0.urdf"
)
DEFAULT_DATA_DIR = "/home/syf/g1_reachability_data/g1_mode6/10mm_capability_v1"


def _description(model_path, left_tcp, right_tcp):
    text = model_path.read_text(encoding="utf-8")
    mesh_uri = (model_path.parent / "meshes").resolve().as_uri()
    text = text.replace('filename="meshes/', f'filename="{mesh_uri}/')
    tool_xml = """
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
  <link name="left_tcp"/>
  <joint name="left_tcp_joint" type="fixed">
    <parent link="left_tool0"/><child link="left_tcp"/>
    <origin xyz="{left_xyz}" rpy="{left_rpy}"/>
  </joint>
  <link name="right_tcp"/>
  <joint name="right_tcp_joint" type="fixed">
    <parent link="right_tool0"/><child link="right_tcp"/>
    <origin xyz="{right_xyz}" rpy="{right_rpy}"/>
  </joint>
"""
    tool_xml = tool_xml.format(
        left_xyz=" ".join(str(value) for value in left_tcp[:3]),
        left_rpy=" ".join(str(value) for value in left_tcp[3:]),
        right_xyz=" ".join(str(value) for value in right_tcp[:3]),
        right_rpy=" ".join(str(value) for value in right_tcp[3:]),
    )
    return text.replace("</robot>", tool_xml + "</robot>")


def _tcp_values(context, side):
    return tuple(
        float(LaunchConfiguration(f"{side}_tcp_{name}").perform(context))
        for name in ("x", "y", "z", "roll", "pitch", "yaw")
    )


def _setup(context):
    model_path = Path(LaunchConfiguration("model_path").perform(context)).resolve()
    data_dir = Path(LaunchConfiguration("data_dir").perform(context)).resolve()
    if not model_path.is_file():
        raise RuntimeError(f"G1 URDF does not exist: {model_path}")
    if not data_dir.is_dir():
        raise RuntimeError(f"Capability directory does not exist: {data_dir}")
    package_share = Path(get_package_share_directory("g1_kdl_ik"))
    left_tcp = _tcp_values(context, "left")
    right_tcp = _tcp_values(context, "right")
    use_joint_gui = LaunchConfiguration("use_joint_gui").perform(context).lower() in (
        "true", "1", "yes"
    )
    publish_debug_clouds = LaunchConfiguration(
        "publish_debug_clouds"
    ).perform(context).lower() in ("true", "1", "yes")
    description = _description(model_path, left_tcp, right_tcp)
    srdf = (package_share / "config" / "g1_mode6.srdf").read_text(encoding="utf-8")
    with (package_share / "config" / "kinematics.yaml").open(encoding="utf-8") as stream:
        kinematics = yaml.safe_load(stream)
    model_parameters = [
        {"robot_description": description},
        {"robot_description_semantic": srdf},
        {"robot_description_kinematics": kinematics},
    ]
    return [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": description}],
            output="screen",
        ),
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            parameters=[{"robot_description": description}],
            condition=IfCondition(LaunchConfiguration("use_joint_gui")),
            output="screen",
        ),
        Node(
            package="g1_kdl_ik",
            executable="capability_map_server",
            parameters=model_parameters + [
                {"data_dir": str(data_dir)},
                {"resolution": LaunchConfiguration("resolution")},
                {"visualization_stride": LaunchConfiguration("visualization_stride")},
                {"publish_debug_clouds": publish_debug_clouds},
                {"left_tcp_x": left_tcp[0]},
                {"left_tcp_y": left_tcp[1]},
                {"left_tcp_z": left_tcp[2]},
                {"left_tcp_roll": left_tcp[3]},
                {"left_tcp_pitch": left_tcp[4]},
                {"left_tcp_yaw": left_tcp[5]},
                {"right_tcp_x": right_tcp[0]},
                {"right_tcp_y": right_tcp[1]},
                {"right_tcp_z": right_tcp[2]},
                {"right_tcp_roll": right_tcp[3]},
                {"right_tcp_pitch": right_tcp[4]},
                {"right_tcp_yaw": right_tcp[5]},
                {"publish_solution_joint_states": not use_joint_gui},
            ],
            output="screen",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", str(package_share / "rviz" / "capability_v1.rviz")],
            condition=IfCondition(LaunchConfiguration("use_rviz")),
            output="screen",
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("model_path", default_value=DEFAULT_MODEL_PATH),
        DeclareLaunchArgument("data_dir", default_value=DEFAULT_DATA_DIR),
        DeclareLaunchArgument("resolution", default_value="0.01"),
        DeclareLaunchArgument("visualization_stride", default_value="2"),
        DeclareLaunchArgument("publish_debug_clouds", default_value="false"),
        DeclareLaunchArgument("left_tcp_x", default_value="0.0"),
        DeclareLaunchArgument("left_tcp_y", default_value="0.0"),
        DeclareLaunchArgument("left_tcp_z", default_value="0.0"),
        DeclareLaunchArgument("left_tcp_roll", default_value="0.0"),
        DeclareLaunchArgument("left_tcp_pitch", default_value="0.0"),
        DeclareLaunchArgument("left_tcp_yaw", default_value="0.0"),
        DeclareLaunchArgument("right_tcp_x", default_value="0.0"),
        DeclareLaunchArgument("right_tcp_y", default_value="0.0"),
        DeclareLaunchArgument("right_tcp_z", default_value="0.0"),
        DeclareLaunchArgument("right_tcp_roll", default_value="0.0"),
        DeclareLaunchArgument("right_tcp_pitch", default_value="0.0"),
        DeclareLaunchArgument("right_tcp_yaw", default_value="0.0"),
        DeclareLaunchArgument("use_joint_gui", default_value="false"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        OpaqueFunction(function=_setup),
    ])
