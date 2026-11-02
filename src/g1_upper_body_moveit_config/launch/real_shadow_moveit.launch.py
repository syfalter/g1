import copy
import importlib.util
import json
import sys
import tempfile
from pathlib import Path
import xml.etree.ElementTree as ET

import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnShutdown
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


def _setup(context):
    active_arm = LaunchConfiguration("active_arm").perform(context)
    if active_arm not in ("left", "right"):
        raise ValueError("active_arm must be left or right")
    interactive_execution_enabled = (
        LaunchConfiguration("enable_rviz_interactive_execution")
        .perform(context)
        .strip()
        .lower()
        in {"1", "true", "yes", "on"}
    )
    description_share = Path(get_package_share_directory("g1_mode6_description"))
    config_share = Path(get_package_share_directory("g1_upper_body_moveit_config"))
    control_share = Path(get_package_share_directory("g1_unitree_control"))
    xacro_path = description_share / "urdf" / "g1_upper_body.urdf.xacro"

    description_xml = xacro.process_file(str(xacro_path)).toxml()
    vendor_model = Path(
        "/home/syf/unitree_ws/unitree_ros/robots/g1_description/"
        "g1_29dof_lock_waist_rev_1_0.urdf"
    )
    mesh_uri = (vendor_model.parent / "meshes").resolve().as_uri()
    description_xml = description_xml.replace(
        'filename="meshes/', f'filename="{mesh_uri}/'
    )
    # Use the same measured connector dimensions and hand assets as workbench.
    workspace = Path('/home/syf/g1_upper_body_ws')
    sys.path.insert(0,str(workspace/'scripts'))
    from task_hand_model import (apply_task_hand_model, fixed_hand_pairs,
                                 apply_articulated_task_hands, articulated_hand_pairs)
    from task_kinematics import configure_waist, arm_group
    task_mode=LaunchConfiguration('task_application_enabled',default='false').perform(context).lower() in ('true','1','yes')
    task_hands=LaunchConfiguration('task_articulated_hands',default='false').perform(context).lower() in ('true','1','yes')
    task_waist=LaunchConfiguration('task_waist_enabled',default='false').perform(context).lower() in ('true','1','yes')
    controller_parameters={}
    integral_config=LaunchConfiguration('task_planning_config',default='').perform(context)
    if task_mode and integral_config:
        from task_planning_config import load_parameters
        parameters,_=load_parameters(integral_config)
        controller_parameters=parameters.controller_parameters(active_arm)
    if (task_hands or task_waist) and not task_mode:
        raise ValueError('Task hand/waist model requires task_application_enabled')
    spec = importlib.util.spec_from_file_location(
        'g1_shared_payload', workspace / 'scripts/preview_payload.py')
    payload_module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(payload_module)
    robot = ET.fromstring(description_xml)
    payload_module.apply_preview_payload(
        robot, json.loads((workspace / 'assets/workbench/payload_preview.json').read_text()))
    if task_mode:
        (apply_articulated_task_hands if task_hands else apply_task_hand_model)(robot)
    if task_waist:configure_waist(robot)
    description_xml = ET.tostring(robot, encoding='unicode')
    export_path = LaunchConfiguration('export_robot_urdf', default='').perform(context)
    if export_path:
        Path(export_path).write_text(description_xml)

    moveit_config = (
        MoveItConfigsBuilder(
            "g1_upper_body", package_name="g1_upper_body_moveit_config"
        )
        .robot_description(file_path=str(xacro_path))
        .robot_description_semantic(file_path="config/g1_upper_body.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .planning_pipelines(default_planning_pipeline="ompl", pipelines=["ompl"])
        .trajectory_execution(
            file_path="config/moveit_controllers.yaml",
            moveit_manage_controllers=False,
        )
        .to_moveit_configs()
    )
    moveit_config.robot_description = {"robot_description": description_xml}
    semantic = ET.fromstring(moveit_config.robot_description_semantic['robot_description_semantic'])
    pairs = {frozenset((item.get('link1'), item.get('link2')))
             for item in semantic.findall('disable_collisions')}
    if task_mode:
        for first,second in (articulated_hand_pairs if task_hands else fixed_hand_pairs)(robot):
            if frozenset((first,second)) not in pairs:
                ET.SubElement(semantic,'disable_collisions',link1=first,link2=second,reason='RigidTaskHandAssembly')
                pairs.add(frozenset((first,second)))
    if task_waist:
        configure_waist(robot,semantic)
        for side in ('left','right'):
            moveit_config.robot_description_kinematics['robot_description_kinematics'][arm_group(side,True)] = copy.deepcopy(
                moveit_config.robot_description_kinematics['robot_description_kinematics'][side+'_arm'])
            moveit_config.planning_pipelines['ompl'][arm_group(side,True)] = copy.deepcopy(
                moveit_config.planning_pipelines['ompl'][side+'_arm'])
        moveit_config.joint_limits['robot_description_planning']['joint_limits']['waist_yaw_joint'] = dict(
            has_velocity_limits=True,max_velocity=.4,has_acceleration_limits=True,max_acceleration=1.5)
    for side in ('left', 'right'):
        mount_links=(f'{side}_wrist_yaw_link',f'{side}_rubber_hand')
        if task_hands:mount_links+=(f'{side}_hand_base',)
        for mount_link in mount_links:
            pair = frozenset((f'{side}_payload_collision', mount_link))
            if pair not in pairs:
                ET.SubElement(semantic, 'disable_collisions',
                              link1=f'{side}_payload_collision', link2=mount_link,
                              reason='RigidPayloadMount')
                pairs.add(pair)
    moveit_config.robot_description_semantic = {
        'robot_description_semantic': ET.tostring(semantic, encoding='unicode')}
    moveit_parameters = moveit_config.to_dict()
    moveit_parameters.update(
        {
            "allow_trajectory_execution": interactive_execution_enabled,
            "publish_robot_description_semantic": True,
            "publish_planning_scene": True,
            "publish_geometry_updates": True,
            "publish_state_updates": True,
            "publish_transforms_updates": True,
        }
    )
    # RViz can request up to 100% of MoveIt's configured limits.  Give the
    # interactive MoveGroup instance the same absolute limits enforced by the
    # guarded hardware gateway so every RViz-generated trajectory is eligible
    # for validation regardless of the panel's scaling-slider position.  Keep
    # the endpoint API on the original model limits; it applies its own guarded
    # execution scaling in single_arm_planner_server.
    interactive_moveit_parameters = copy.deepcopy(moveit_parameters)
    interactive_joint_limits = interactive_moveit_parameters.get(
        "robot_description_planning", {}
    ).get("joint_limits", {})
    for limits in interactive_joint_limits.values():
        if limits.get("has_velocity_limits", False):
            limits["max_velocity"] = min(float(limits["max_velocity"]), 0.60)
        if limits.get("has_acceleration_limits", False):
            limits["max_acceleration"] = min(
                float(limits["max_acceleration"]), 1.50
            )

    description = {"robot_description": description_xml}
    rviz_config = str(config_share / "rviz" / "g1_upper_body_moveit.rviz")
    custom_rviz = LaunchConfiguration('rviz_config', default='').perform(context)
    if custom_rviz:
        rviz_config = custom_rviz
    cleanup_actions = []
    if active_arm == "left" and not custom_rviz:
        rviz_data = yaml.safe_load(Path(rviz_config).read_text())
        for display in rviz_data['Visualization Manager']['Displays']:
            if display.get('Class') == 'moveit_rviz_plugin/MotionPlanning':
                display['Planning Request']['Planning Group'] = 'left_arm'
        with tempfile.NamedTemporaryFile(mode='w', suffix='.rviz', prefix='g1-left-', delete=False) as stream:
            yaml.safe_dump(rviz_data, stream, sort_keys=False)
            rviz_config = stream.name
        def cleanup_rviz(_):
            Path(rviz_config).unlink(missing_ok=True)
            return []
        cleanup_actions.append(RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=cleanup_rviz)])))
    shadow_config = str(control_share / "config" / "shadow_executor.yaml")
    gateway_config = str(control_share / "config" / "arm_sdk_hardware_gateway.yaml")
    tool_profiles_config = str(config_share / "config" / "tool_profiles.yaml")

    return cleanup_actions + [
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
            package="g1_unitree_control",
            executable="arm_state_bridge",
            name="arm_state_bridge",
            parameters=[
                str(control_share / "config" / "arm_state_bridge.yaml"),
                {
                    "output_topic": "/joint_states",
                    "include_lower_body": True,
                    "publish_rate_hz": 200.0,
                },
            ],
            output="screen",
            condition=IfCondition(LaunchConfiguration('start_state_bridge')),
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="g1_robot_state_publisher",
            parameters=[description],
            output="screen",
        ),
        Node(
            package="g1_unitree_control",
            executable="planned_joint_state_overlay",
            name="g1_planned_joint_state_overlay",
            parameters=[{
                "actual_topic": "/joint_states",
                "reference_topic": "/g1/arm_gateway/command_preview",
                "output_topic": "/g1/planned_joint_states",
            }],
            output="screen",
            condition=IfCondition(LaunchConfiguration("show_planned_overlay")),
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            namespace="planned",
            name="robot_state_publisher",
            parameters=[description, {"frame_prefix": "planned/"}],
            remappings=[("joint_states", "/g1/planned_joint_states")],
            output="screen",
            condition=IfCondition(LaunchConfiguration("show_planned_overlay")),
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="world_to_planned_pelvis",
            arguments=[
                "--x", "0", "--y", "0", "--z", "0.793",
                "--roll", "0", "--pitch", "0", "--yaw", "0",
                "--frame-id", "world", "--child-frame-id", "planned/pelvis",
            ],
            output="screen",
            condition=IfCondition(LaunchConfiguration("show_planned_overlay")),
        ),
        Node(
            package="g1_unitree_control",
            executable="arm_sdk_hardware_gateway",
            name="g1_arm_sdk_hardware_gateway",
            parameters=[
                gateway_config,
                {
                    "active_arm": active_arm,
                    **controller_parameters,
                    "task_application_enabled": ParameterValue(
                        LaunchConfiguration('task_application_enabled'), value_type=bool),
                    "task_waist_enabled": task_waist,
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
            condition=IfCondition(LaunchConfiguration("start_hardware_gateway")),
        ),
        Node(
            package="moveit_ros_move_group",
            executable="move_group",
            name="move_group",
            parameters=[interactive_moveit_parameters],
            output="screen",
        ),
        Node(
            package='g1_upper_body_moveit_config', executable='task_state_validator',
            name='g1_task_state_validator', parameters=[moveit_parameters, {'publish_planning_scene': False}],
            output='screen', condition=IfCondition(LaunchConfiguration('task_application_enabled')),
        ),
        Node(
            package="g1_unitree_control",
            executable="shadow_trajectory_executor",
            name="g1_shadow_trajectory_executor",
            parameters=[
                shadow_config,
                {
                    "enforce_tracking_error": ParameterValue(
                        LaunchConfiguration("enforce_tracking_error"),
                        value_type=bool,
                    )
                },
            ],
            output="screen",
            condition=IfCondition(LaunchConfiguration("start_shadow_executor")),
        ),
        Node(
            package="g1_upper_body_moveit_config",
            executable="single_arm_planner_server",
            name="g1_single_arm_planner",
            parameters=[
                moveit_parameters,
                tool_profiles_config,
                {
                    "hardware_arm": active_arm,
                    "add_demo_table": False,
                    "add_demo_obstacle": False,
                    "publish_shadow_trajectory": True,
                    "shadow_trajectory_topic": LaunchConfiguration(
                        "trajectory_output_topic"
                    ),
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
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("active_arm", default_value="right"),
            DeclareLaunchArgument('task_application_enabled', default_value='false'),
            DeclareLaunchArgument('task_articulated_hands', default_value='false'),
            DeclareLaunchArgument('task_waist_enabled', default_value='false'),
            DeclareLaunchArgument('task_planning_config', default_value=''),
            DeclareLaunchArgument('start_state_bridge', default_value='true'),
            DeclareLaunchArgument('export_robot_urdf', default_value=''),
            DeclareLaunchArgument('rviz_config', default_value=''),
            DeclareLaunchArgument(
                "enable_rviz_interactive_execution",
                default_value="false",
                description="Allow RViz Execute to send a trajectory to the guarded right-arm action server.",
            ),
            DeclareLaunchArgument("show_planned_overlay", default_value="true"),
            DeclareLaunchArgument(
                "start_hardware_gateway",
                default_value="false",
                description="Start the guarded gateway; it remains dry-run unless separately authorized and armed.",
            ),
            DeclareLaunchArgument("hardware_output_enabled", default_value="false"),
            DeclareLaunchArgument("hardware_authorization", default_value=""),
            DeclareLaunchArgument(
                "enforce_tracking_error",
                default_value="false",
                description="Keep false for shadow playback against a stationary real arm.",
            ),
            DeclareLaunchArgument("start_shadow_executor", default_value="true"),
            DeclareLaunchArgument(
                "trajectory_output_topic",
                default_value="/g1/shadow_executor/trajectory",
                description="Validated planner trajectory sink; use the guarded gateway topic only during an approved hardware test.",
            ),
            OpaqueFunction(function=_setup),
        ]
    )
