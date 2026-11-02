from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


DEFAULT_MODEL_PATH = (
    "/home/syf/unitree_ws/unitree_ros/robots/g1_description/"
    "g1_29dof_lock_waist_rev_1_0.urdf"
)


def _launch_setup(context):
    model_path = Path(LaunchConfiguration("model_path").perform(context)).resolve()
    if not model_path.is_file():
        raise RuntimeError(f"G1 URDF does not exist: {model_path}")

    # The official URDF uses paths relative to its own directory. Convert them
    # to file URIs so RViz can resolve the meshes regardless of the launch cwd.
    robot_description = model_path.read_text(encoding="utf-8")
    mesh_uri = (model_path.parent / "meshes").resolve().as_uri()
    robot_description = robot_description.replace(
        'filename="meshes/', f'filename="{mesh_uri}/'
    )

    description_parameters = {"robot_description": robot_description}
    rviz_config = str(
        Path(get_package_share_directory("g1_mode6_description"))
        / "rviz"
        / "g1_mode6.rviz"
    )

    return [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="g1_robot_state_publisher",
            output="screen",
            parameters=[description_parameters],
        ),
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            name="g1_joint_state_publisher_gui",
            output="screen",
            parameters=[description_parameters],
            condition=IfCondition(LaunchConfiguration("use_gui")),
        ),
        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            name="g1_joint_state_publisher",
            output="screen",
            parameters=[description_parameters],
            condition=UnlessCondition(LaunchConfiguration("use_gui")),
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="left_tool0_publisher",
            arguments=[
                "--x", "0", "--y", "0", "--z", "0",
                "--roll", "0", "--pitch", "0", "--yaw", "0",
                "--frame-id", "left_rubber_hand",
                "--child-frame-id", "left_tool0",
            ],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="right_tool0_publisher",
            arguments=[
                "--x", "0", "--y", "0", "--z", "0",
                "--roll", "0", "--pitch", "0", "--yaw", "0",
                "--frame-id", "right_rubber_hand",
                "--child-frame-id", "right_tool0",
            ],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="g1_rviz",
            output="screen",
            arguments=["-d", rviz_config],
            condition=IfCondition(LaunchConfiguration("use_rviz")),
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "model_path",
                default_value=DEFAULT_MODEL_PATH,
                description="Absolute path to the official mode_machine 6 URDF.",
            ),
            DeclareLaunchArgument(
                "use_gui",
                default_value="true",
                description="Start joint_state_publisher_gui.",
            ),
            DeclareLaunchArgument(
                "use_rviz",
                default_value="true",
                description="Start RViz 2 with the G1 display configuration.",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
