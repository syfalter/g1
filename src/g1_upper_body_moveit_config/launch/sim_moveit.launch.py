from pathlib import Path

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def _setup(context):
    description_share = Path(get_package_share_directory("g1_mode6_description"))
    config_share = Path(get_package_share_directory("g1_upper_body_moveit_config"))
    xacro_path = description_share / "urdf" / "g1_upper_body.urdf.xacro"

    # Expand once and pass exactly the same description to every consumer.
    description_xml = xacro.process_file(str(xacro_path)).toxml()
    vendor_model = Path(
        "/home/syf/unitree_ws/unitree_ros/robots/g1_description/"
        "g1_29dof_lock_waist_rev_1_0.urdf"
    )
    mesh_uri = (vendor_model.parent / "meshes").resolve().as_uri()
    description_xml = description_xml.replace(
        'filename="meshes/', f'filename="{mesh_uri}/'
    )

    moveit_config = (
        MoveItConfigsBuilder(
            "g1_upper_body", package_name="g1_upper_body_moveit_config"
        )
        .robot_description(file_path=str(xacro_path))
        .robot_description_semantic(file_path="config/g1_upper_body.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .planning_pipelines(default_planning_pipeline="ompl", pipelines=["ompl"])
        .trajectory_execution(file_path=None, moveit_manage_controllers=False)
        .to_moveit_configs()
    )
    moveit_config.robot_description = {"robot_description": description_xml}
    moveit_parameters = moveit_config.to_dict()
    moveit_parameters.update(
        {
            "allow_trajectory_execution": False,
            "publish_robot_description_semantic": True,
            "publish_planning_scene": True,
            "publish_geometry_updates": True,
            "publish_state_updates": True,
            "publish_transforms_updates": True,
        }
    )

    rviz_config = str(config_share / "rviz" / "g1_upper_body_moveit.rviz")
    tool_profiles_config = str(config_share / "config" / "tool_profiles.yaml")
    description = {"robot_description": description_xml}

    return [
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="world_to_pelvis",
            arguments=[
                "--x", "0", "--y", "0", "--z", "0.793",
                "--roll", "0", "--pitch", "0", "--yaw", "0",
                "--frame-id", "world", "--child-frame-id", "pelvis",
            ],
            output="screen",
        ),
        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            name="g1_sim_joint_states",
            parameters=[description, {"rate": 50}],
            output="screen",
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="g1_robot_state_publisher",
            parameters=[description],
            output="screen",
        ),
        Node(
            package="moveit_ros_move_group",
            executable="move_group",
            name="move_group",
            parameters=[moveit_parameters],
            output="screen",
        ),
        Node(
            package="g1_upper_body_moveit_config",
            executable="single_arm_planner_server",
            name="g1_single_arm_planner",
            parameters=[
                moveit_parameters,
                tool_profiles_config,
                {
                    "add_demo_table": True,
                    "add_demo_obstacle": LaunchConfiguration("demo_obstacle"),
                },
            ],
            output="screen",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="g1_moveit_rviz",
            arguments=["-d", rviz_config],
            parameters=[
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.robot_description_kinematics,
                moveit_config.planning_pipelines,
                moveit_config.joint_limits,
            ],
            output="screen",
            condition=IfCondition(LaunchConfiguration("use_rviz")),
        ),
        Node(
            package="g1_upper_body_moveit_config",
            executable="single_arm_plan_demo",
            name="g1_single_arm_plan_demo",
            output="screen",
            condition=IfCondition(LaunchConfiguration("auto_plan")),
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("auto_plan", default_value="true"),
            DeclareLaunchArgument("demo_obstacle", default_value="false"),
            OpaqueFunction(function=_setup),
        ]
    )
