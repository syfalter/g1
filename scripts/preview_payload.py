"""Shared RH56DFX geometry for workbench and real endpoint planning."""
import copy
import math
from pathlib import Path
import xml.etree.ElementTree as ET


def apply_preview_payload(robot, config):
    length = float(config["connector_length_m"])
    radius = float(config["connector_radius_m"])
    if not all(math.isfinite(v) and 0 < v <= 0.3 for v in (length, radius)):
        raise ValueError("Invalid preview connector dimensions")
    for side in ("left", "right"):
        hand_joint = robot.find(f"joint[@name='{side}_hand_palm_joint']")
        if hand_joint is None:
            raise ValueError(f"Missing {side} hand mount")
        payload = robot.find(f"link[@name='{side}_payload_collision']")
        if payload is None:
            payload = ET.SubElement(robot, "link", name=f"{side}_payload_collision")
            joint = ET.SubElement(robot, "joint", name=f"{side}_payload_collision_joint", type="fixed")
            ET.SubElement(joint, "parent", link=f"{side}_wrist_yaw_link")
            ET.SubElement(joint, "child", link=f"{side}_payload_collision")
        mount = hand_joint.find("origin")
        if any(abs(float(v)) > 1e-9 for v in mount.get("rpy", "0 0 0").split()):
            raise ValueError("Preview connector expects the original aligned wrist mount")
        start = [float(v) for v in mount.get("xyz").split()]
        centre = [start[0] + length / 2, start[1], start[2]]
        for child in list(payload):
            if child.tag in ("visual", "collision"):
                payload.remove(child)
        collision = ET.SubElement(payload, "collision")
        ET.SubElement(collision, "origin", xyz=" ".join(map(str, centre)),
                      rpy=f"0 {math.pi / 2} 0")
        geometry = ET.SubElement(collision, "geometry")
        ET.SubElement(geometry, "cylinder", radius=str(radius), length=str(length))
        visual = copy.deepcopy(collision)
        visual.tag = "visual"
        material = ET.SubElement(visual, "material", name=f"{side}_preview_connector_metal")
        ET.SubElement(material, "color", rgba="0.25 0.65 0.75 1")
        payload.append(visual)
        # Moving the fixed hand joint also moves its collision geometry and tool0.
        start[0] += length
        mount.set("xyz", " ".join(map(str, start)))
    if config.get("hand_model") == "RH56DFX":
        mesh_dir = Path(__file__).resolve().parents[1] / "assets/workbench/rh56dfx"
        for side in ("left", "right"):
            hand = robot.find(f"link[@name='{side}_rubber_hand']")
            if hand is None:
                raise ValueError(f"Missing {side} hand link")
            for element in list(hand):
                if element.tag in ("visual", "collision", "inertial"):
                    hand.remove(element)
            for kind in ("visual", "collision"):
                path = mesh_dir / f"{side}_{kind}.stl"
                if not path.is_file():
                    raise FileNotFoundError(path)
                element = ET.SubElement(hand, kind)
                geometry = ET.SubElement(element, "geometry")
                ET.SubElement(geometry, "mesh", filename=path.as_uri())
                if kind == "visual":
                    material = ET.SubElement(element, "material", name=f"{side}_rh56dfx_white")
                    ET.SubElement(material, "color", rgba="0.85 0.85 0.88 1")
            old_box = robot.find(f"link[@name='{side}_hand_collision']")
            if old_box is not None:
                for collision in list(old_box.findall("collision")):
                    old_box.remove(collision)
