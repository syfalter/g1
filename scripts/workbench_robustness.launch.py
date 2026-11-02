"""Isolated test-only MoveIt process; no GUI, robot bridge or motor controller."""
from pathlib import Path
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    if os.environ.get('ROS_DOMAIN_ID') not in tuple(str(v) for v in range(78,86)) or os.environ.get('ROS_LOCALHOST_ONLY')!='1':
        raise RuntimeError('Robustness tests require localhost-only domains 78 through 85')
    root=Path(LaunchConfiguration('run_directory').perform(context))
    return [Node(package='moveit_ros_move_group',executable='move_group',
                 parameters=[str(root/'scene/move_group.yaml')],output='screen')]


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('run_directory'),OpaqueFunction(function=setup)])
