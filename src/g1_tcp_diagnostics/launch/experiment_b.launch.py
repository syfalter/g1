"""Isolated B UI with optional read-only RPC to the existing camera owner."""
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    import os
    if os.environ.get('ROS_DOMAIN_ID') != '96' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        raise RuntimeError('Use start_camera_b.sh; isolated localhost/domain 96 required')
    root = Path('/home/syf/g1_upper_body_ws')
    mode = LaunchConfiguration('mode').perform(context)
    if mode not in ('offline', 'live'):
        raise RuntimeError('mode must be offline or live')
    cmd = ['/usr/bin/python3', str(root/'scripts/camera_b_node.py'), '--'+mode]
    for key, flag in [('network', '--network'), ('ssh_host', '--ssh'), ('peer', '--peer')]:
        cmd += [flag, LaunchConfiguration(key).perform(context)]
    for key, flag in [('session_file', '--session'), ('import_path', '--import-path')]:
        value = LaunchConfiguration(key).perform(context)
        if value:
            cmd += [flag, value]
    backend = ExecuteProcess(cmd=cmd, output='screen')
    actions = [backend, RegisterEventHandler(OnProcessExit(target_action=backend,
               on_exit=[EmitEvent(event=Shutdown(reason='B backend exited'))]))]
    if LaunchConfiguration('use_window').perform(context).lower() == 'true':
        if LaunchConfiguration('use_rviz').perform(context).lower() == 'true':
            window = Node(package='rviz2', executable='rviz2', output='screen',
                          arguments=['-d', str(root/'src/g1_tcp_diagnostics/rviz/experiment_b.rviz')])
        else:
            window = Node(package='g1_tcp_diagnostics', executable='camera_b_window', output='screen')
        actions += [window, RegisterEventHandler(OnProcessExit(target_action=window,
                    on_exit=[EmitEvent(event=Shutdown(reason='B window closed; robot unaffected'))]))]
    return actions


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('mode', default_value='offline'),
        DeclareLaunchArgument('network', default_value='eth0'),
        DeclareLaunchArgument('ssh_host', default_value='unitree@192.168.0.249'),
        DeclareLaunchArgument('peer', default_value='192.168.123.164'),
        DeclareLaunchArgument('session_file', default_value=''),
        DeclareLaunchArgument('import_path', default_value=''),
        DeclareLaunchArgument('use_rviz', default_value='false'),
        DeclareLaunchArgument('use_window', default_value='true'), OpaqueFunction(function=setup)])
