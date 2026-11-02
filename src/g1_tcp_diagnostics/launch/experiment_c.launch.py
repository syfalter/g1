"""Launch only C processes; owner/controller lifecycle remains external."""
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    root = Path('/home/syf/g1_upper_body_ws')
    value = lambda key: LaunchConfiguration(key).perform(context)
    mode = value('mode')
    if mode not in ('offline', 'live'):
        raise ValueError('Unknown C mode')
    cmd = ['/usr/bin/python3', str(root/'scripts/camera_c_node.py'), '--'+mode]
    for key, flag in [('session_file', '--session'), ('network', '--network'), ('ssh_host', '--ssh'), ('peer', '--peer')]:
        if value(key):
            cmd += [flag, value(key)]
    backend = ExecuteProcess(cmd=cmd, output='screen', sigterm_timeout='120', sigkill_timeout='10')
    actions = [backend, RegisterEventHandler(OnProcessExit(target_action=backend,
        on_exit=[EmitEvent(event=Shutdown(reason='C backend exited; owner unaffected'))]))]
    if value('use_window') == 'true':
        if value('use_rviz') == 'true':
            window = Node(package='rviz2', executable='rviz2', output='screen',
                          arguments=['-d', str(root/'src/g1_tcp_diagnostics/rviz/experiment_c.rviz')])
        else:
            window = Node(package='g1_tcp_diagnostics', executable='camera_c_window', output='screen')
        actions += [window, RegisterEventHandler(OnProcessExit(target_action=window,
            on_exit=[EmitEvent(event=Shutdown(reason='C window closed; owner unaffected'))]))]
    return actions


def generate_launch_description():
    defaults = dict(mode='offline', session_file='', network='eth0', ssh_host='unitree@192.168.0.249',
                    peer='192.168.123.164', use_window='true', use_rviz='false')
    return LaunchDescription([DeclareLaunchArgument(k, default_value=v) for k, v in defaults.items()]+[OpaqueFunction(function=setup)])
