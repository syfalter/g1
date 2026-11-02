from pathlib import Path

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import EmitEvent, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node


def _setup(_context):
    description_share = Path(get_package_share_directory("g1_mode6_description"))
    control_share = Path(get_package_share_directory("g1_unitree_control"))
    config_share = Path(get_package_share_directory("g1_upper_body_moveit_config"))
    description_xml = xacro.process_file(
        str(description_share / "urdf" / "g1_upper_body.urdf.xacro")
    ).toxml()
    vendor_model = Path(
        "/home/syf/unitree_ws/unitree_ros/robots/g1_description/"
        "g1_29dof_lock_waist_rev_1_0.urdf"
    )
    description_xml = description_xml.replace(
        'filename="meshes/', f'filename="{(vendor_model.parent / "meshes").resolve().as_uri()}/'
    )
    semantic_xml = (config_share / "config" / "g1_upper_body.srdf").read_text()

    bridge = Node(
        package="g1_unitree_control",
        executable="arm_state_bridge",
        name="elbow_drag_read_only_state_bridge",
        parameters=[
            str(control_share / "config" / "arm_state_bridge.yaml"),
            {"output_topic": "/joint_states", "include_lower_body": True,
             "publish_rate_hz": 100.0},
        ],
        output="screen",
    )
    checker = Node(
        package="g1_upper_body_moveit_config",
        executable="right_arm_calibration_pose_screener",
        name="right_elbow_drag_range_checker",
        parameters=[{
            "robot_description": description_xml,
            "robot_description_semantic": semantic_xml,
            "elbow_drag_range_rad": 0.1745329252,
            "target_check_output":
                "/home/syf/g1_upper_body_ws/calibration/right_elbow_drag_range.yaml",
        }],
        output="screen",
    )
    stop = RegisterEventHandler(OnProcessExit(
        target_action=checker,
        on_exit=[EmitEvent(event=Shutdown(reason="elbow drag range check complete"))],
    ))
    return [bridge, checker, stop]


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=_setup)])
