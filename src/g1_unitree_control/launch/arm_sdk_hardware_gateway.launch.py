from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory("g1_unitree_control"))
    config = str(share / "config" / "arm_sdk_hardware_gateway.yaml")
    return LaunchDescription(
        [
            DeclareLaunchArgument("active_arm", default_value="right"),
            DeclareLaunchArgument("hardware_output_enabled", default_value="false"),
            DeclareLaunchArgument("hardware_authorization", default_value=""),
            Node(
                package="g1_unitree_control",
                executable="arm_sdk_hardware_gateway",
                name="g1_arm_sdk_hardware_gateway",
                parameters=[
                    config,
                    {
                        "active_arm": LaunchConfiguration("active_arm"),
                        "hardware_output_enabled": ParameterValue(
                            LaunchConfiguration("hardware_output_enabled"),
                            value_type=bool,
                        ),
                        "hardware_authorization": LaunchConfiguration(
                            "hardware_authorization"
                        ),
                    },
                ],
                output="screen",
            ),
        ]
    )
