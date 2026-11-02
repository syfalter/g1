from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory("g1_unitree_control"))
    config = str(share / "config" / "arm_state_bridge.yaml")
    return LaunchDescription(
        [
            DeclareLaunchArgument("input_topic", default_value="/lowstate"),
            DeclareLaunchArgument(
                "output_topic", default_value="/g1/arm_joint_states"
            ),
            DeclareLaunchArgument("include_lower_body", default_value="false"),
            Node(
                package="g1_unitree_control",
                executable="arm_state_bridge",
                name="arm_state_bridge",
                parameters=[
                    config,
                    {
                        "input_topic": LaunchConfiguration("input_topic"),
                        "output_topic": LaunchConfiguration("output_topic"),
                        "include_lower_body": ParameterValue(
                            LaunchConfiguration("include_lower_body"),
                            value_type=bool,
                        ),
                    },
                ],
                output="screen",
            ),
        ]
    )
