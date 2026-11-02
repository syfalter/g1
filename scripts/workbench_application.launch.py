"""Offline workbench application: MoveIt planning, simulated state, RViz, UI."""
import importlib.util
import json
import sys
from pathlib import Path
import xml.etree.ElementTree as ET

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, ExecuteProcess
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
from workbench_stance import add_stance_joints
spec = importlib.util.spec_from_file_location("preview", ROOT / "scripts/workbench_preview.launch.py")
preview = importlib.util.module_from_spec(spec)
spec.loader.exec_module(preview)


def setup(context):
    description, initial = preview.build_scene(context)
    robot = ET.fromstring(description["robot_description"])
    add_stance_joints(robot)
    assets = ROOT / "assets/workbench"
    initial_pose_file=LaunchConfiguration('initial_pose_file').perform(context)
    profile=json.loads(Path(initial_pose_file).read_text())
    posture=profile.get('hand_startup_posture','open')
    if posture not in ('open','folded'):
        raise ValueError('Unknown startup hand posture')
    hand_assets=assets/'rh56dfx'/('task_open' if posture=='open' else 'task')
    meta = json.loads((assets / "metadata.json").read_text())
    target = json.loads((assets / "task/target.json").read_text())
    # Offline task group: bounded yaw assistance, never change hardware URDF.
    waist_limit = robot.find("joint[@name='waist_yaw_joint']/limit")
    waist_limit.set("lower", "-0.6108652382")
    waist_limit.set("upper", "0.6108652382")
    table = robot.find("link[@name='workbench']")
    for mesh in table.iter("mesh"):
        mesh.set("filename", (assets / "task/cabinet.stl").as_uri())
    origin = robot.find("joint[@name='preview_world_workbench']/origin")
    origin.set("xyz", f"{meta['pelvis_front_offset_m'] + .05} 0 {0.793 + meta['zero_pose_sole_z_relative_pelvis_m']}")
    origin.set("rpy", "0 0 0")
    handle = ET.SubElement(robot, "link", name="task_handle")
    for kind in ("visual", "collision"):
        element = ET.SubElement(handle, kind)
        geometry = ET.SubElement(element, "geometry")
        ET.SubElement(geometry, "mesh", filename=(assets / "task/handle.stl").as_uri())
        if kind == "visual":
            material = ET.SubElement(element, "material", name="handle_teal")
            ET.SubElement(material, "color", rgba="0.1 0.7 0.6 1")
    joint = ET.SubElement(robot, "joint", name="task_handle_angle", type="revolute")
    ET.SubElement(joint, "parent", link="world")
    ET.SubElement(joint, "child", link="task_handle")
    ET.SubElement(joint, "origin", xyz=" ".join(map(str,target["axis_center"])), rpy="0 0 0")
    ET.SubElement(joint, "axis", xyz="0 0 1")
    ET.SubElement(joint, "limit", lower="-2.2", upper="2.2", effort="1", velocity="1")
    for side in ("left", "right"):
        hand = robot.find(f"link[@name='{side}_rubber_hand']")
        for kind in ("visual", "collision"):
            hand.find(f"{kind}/geometry/mesh").set("filename", (hand_assets / f"{side}_body_{kind}.stl").as_uri())
        assembly=ET.parse(hand_assets/f"{side}_index.urdf").getroot()
        for element in assembly:robot.append(element)
    srdf = ET.parse(ROOT / "src/g1_upper_body_moveit_config/config/g1_upper_body.srdf").getroot()
    for element in list(srdf):
        if element.tag in ("virtual_joint", "end_effector"):
            srdf.remove(element)
    for side in ("left", "right"):
        srdf.find(f"group[@name='{side}_arm']/chain").set("tip_link", f"{side}_rubber_hand")
        srdf.find(f"group[@name='{side}_arm']/chain").set("base_link", "pelvis")
        ET.SubElement(srdf.find(f"group_state[@name='home'][@group='{side}_arm']"),"joint",name="waist_yaw_joint",value="0")
        index_names=[link.get('name') for link in robot.findall('link') if link.get('name').startswith(side+'_index_')]
        for i,a in enumerate(index_names):
            for b in [f'{side}_rubber_hand']+index_names[i+1:]:
                ET.SubElement(srdf,'disable_collisions',link1=a,link2=b,reason='FingerAssemblyOverlap')
        for a,b in ((f"{side}_payload_collision",f"{side}_wrist_yaw_link"),
                    (f"{side}_payload_collision",f"{side}_rubber_hand")):
            ET.SubElement(srdf,"disable_collisions",link1=a,link2=b,reason="FixedMount")
    # Existing vendor overlap in the locked lower body, unrelated to arm planning.
    for a,b in (("waist_collision","waist_roll_link"), ("left_ankle_roll_link","left_knee_link"),
                ("right_ankle_roll_link","right_knee_link"), ("task_handle","workbench")):
        ET.SubElement(srdf,"disable_collisions",link1=a,link2=b,reason="FixedSceneBoundary")
    description={"robot_description":ET.tostring(robot,encoding="unicode")}
    config=ROOT / "src/g1_upper_body_moveit_config/config"
    params=dict(description)
    params["robot_description_semantic"]=ET.tostring(srdf,encoding="unicode")
    params["robot_description_kinematics"]=yaml.safe_load((config/"kinematics.yaml").read_text())
    params["robot_description_planning"]=yaml.safe_load((config/"joint_limits.yaml").read_text())
    params["robot_description_planning"]["joint_limits"]["waist_yaw_joint"] = dict(
        has_velocity_limits=True,max_velocity=.4,has_acceleration_limits=True,max_acceleration=.5)
    params["planning_pipelines"]=["ompl"]
    params["default_planning_pipeline"]="ompl"
    params["ompl"]=yaml.safe_load((config/"ompl_planning.yaml").read_text())
    for side in ("left_arm", "right_arm"):
        params["ompl"][side]["longest_valid_segment_fraction"] = .002
    params["ompl"].update(path_tolerance=.005, resample_dt=.04)
    params.update(allow_trajectory_execution=False,publish_robot_description_semantic=True,
                  publish_planning_scene=True,publish_geometry_updates=True,publish_state_updates=True,publish_transforms_updates=True)
    initial["task_handle_angle"]=0.
    # Snapshot the exact offline planning model for tests and reproducibility.
    (assets/"task/robot.urdf").write_text(description["robot_description"])
    (assets/"task/robot.srdf").write_text(params["robot_description_semantic"])
    return [
        Node(package="robot_state_publisher",executable="robot_state_publisher",parameters=[description]),
        Node(package="moveit_ros_move_group",executable="move_group",parameters=[params]),
        ExecuteProcess(cmd=["/usr/bin/python3",str(ROOT/"scripts/workbench_app.py"),
                            "--ros-args","-p",f"initial_pose_file:={initial_pose_file}"],output="screen"),
        Node(package="rviz2",executable="rviz2",arguments=["-d",str(assets/"task/application.rviz")]),
    ]


def generate_launch_description():
    declarations = [a for a in preview.generate_launch_description().entities if isinstance(a, DeclareLaunchArgument)]
    return LaunchDescription(declarations+[OpaqueFunction(function=setup)])
