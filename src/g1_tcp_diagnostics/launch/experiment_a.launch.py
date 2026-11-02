"""Diagnostics only. No gateway, hand owner, camera owner, RSP or TF publisher."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def setup(context):
    root = Path('/home/syf/g1_upper_body_ws')
    share = Path(get_package_share_directory('g1_tcp_diagnostics'))
    offline = LaunchConfiguration('offline').perform(context).lower() == 'true'
    command = ['/usr/bin/python3', str(root/'scripts/tcp_a_node.py'), '--config', str(share/'config/tcp_a.yaml')]
    if offline:
        command += ['--offline']
    for key, flag in (('session_file', '--session'), ('robot_urdf', '--robot-urdf')):
        value = LaunchConfiguration(key).perform(context)
        if value:
            command += [flag, value]
    backend = ExecuteProcess(cmd=command, output='screen')
    actions = [backend, RegisterEventHandler(OnProcessExit(target_action=backend,
               on_exit=[EmitEvent(event=Shutdown(reason='诊断后端退出；原控制系统不受影响'))]))]
    if LaunchConfiguration('use_rviz').perform(context).lower() == 'true':
        rviz = Node(package='rviz2', executable='rviz2', name='g1_tcp_a_rviz', output='screen',
                    arguments=['-d', str(share/'rviz'/('experiment_a_offline.rviz' if offline else 'experiment_a.rviz'))])
        actions += [rviz, RegisterEventHandler(OnProcessExit(target_action=rviz,
                    on_exit=[EmitEvent(event=Shutdown(reason='诊断窗口关闭；不调用任何控制服务'))]))]
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('offline', default_value='false'),
        DeclareLaunchArgument('session_file', default_value=''),
        DeclareLaunchArgument('robot_urdf', default_value=''),
        DeclareLaunchArgument('use_rviz', default_value='true'), OpaqueFunction(function=setup)])
