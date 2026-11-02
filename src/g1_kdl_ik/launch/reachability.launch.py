from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


DEFAULT_MODEL_PATH = (
    "/home/syf/unitree_ws/unitree_ros/robots/g1_description/"
    "g1_29dof_lock_waist_rev_1_0.urdf"
)
DEFAULT_DATA_DIR = "/home/syf/g1_reachability_data/g1_mode6/5mm_v0"
DEFAULT_SUPPLEMENTAL_DATA_DIR = (
    "/home/syf/g1_reachability_data/g1_mode6/5mm_front_v0"
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
    data_dir = Path(LaunchConfiguration("data_dir").perform(context)).resolve()
    supplemental_data_dir = Path(
        LaunchConfiguration("supplemental_data_dir").perform(context)
    ).resolve()
    if not model_path.is_file():
        raise RuntimeError(f"G1 URDF does not exist: {model_path}")
    if not data_dir.is_dir():
        raise RuntimeError(f"Reachability directory does not exist: {data_dir}")
    if not supplemental_data_dir.is_dir():
        raise RuntimeError(
            f"Supplemental reachability directory does not exist: {supplemental_data_dir}"
        )
    package_share = Path(get_package_share_directory("g1_kdl_ik"))
    description = _description(model_path)
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
            package="g1_kdl_ik",
            executable="kdl_ik_server",
            parameters=model_parameters + [
                {"default_timeout": 0.05},
                {"default_motion_duration": 2.0},
                {"max_visual_joint_speed": 0.5},
                {"publish_rate": 50.0},
                {
                    "publish_joint_states": ParameterValue(
                        PythonExpression([
                            "'", LaunchConfiguration("use_joint_gui"),
                            "' == 'false'",
                        ]),
                        value_type=bool,
                    )
                },
                {"run_self_test": False},
            ],
            output="screen",
        ),
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            name="g1_joint_state_publisher_gui",
            parameters=[{"robot_description": description}],
            condition=IfCondition(LaunchConfiguration("use_joint_gui")),
            output="screen",
        ),
        Node(
            package="g1_kdl_ik",
            executable="reachability_map_server",
            parameters=model_parameters + [
                {"data_dir": str(data_dir)},
                {"supplemental_data_dir": str(supplemental_data_dir)},
                {"resolution": 0.005},
                {"validation_stride": LaunchConfiguration("validation_stride")},
                {"visualization_stride": LaunchConfiguration("visualization_stride")},
            ],
            output="screen",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", str(package_share / "rviz" / "reachability.rviz")],
            condition=IfCondition(LaunchConfiguration("use_rviz")),
            output="screen",
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("model_path", default_value=DEFAULT_MODEL_PATH),
        DeclareLaunchArgument("data_dir", default_value=DEFAULT_DATA_DIR),
        DeclareLaunchArgument(
            "supplemental_data_dir", default_value=DEFAULT_SUPPLEMENTAL_DATA_DIR
        ),
        DeclareLaunchArgument("validation_stride", default_value="101"),
        DeclareLaunchArgument("visualization_stride", default_value="8"),
        DeclareLaunchArgument("use_joint_gui", default_value="false"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        OpaqueFunction(function=_setup),
    ])
