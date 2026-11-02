"""Offline geometry sweep, not a robot motion command or execution validator."""
import json
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET
from urllib.parse import unquote, urlparse

import numpy as np
import trimesh
from preview_payload import apply_preview_payload

ROOT = Path(__file__).resolve().parents[1]
ASSETS = ROOT / "assets/workbench"
meta = json.loads((ASSETS / "metadata.json").read_text())
profile = json.loads((ASSETS / "task_start_pose.json").read_text())
xml = subprocess.check_output([
    "/usr/bin/python3", "-c",
    "import xacro; print(xacro.process_file(" + repr(str(ROOT / "src/g1_mode6_description/urdf/g1_upper_body.urdf.xacro")) + ").toxml())",
], text=True)
robot = ET.fromstring(xml)
envelope = json.loads((ASSETS / "payload_preview.json").read_text())
apply_preview_payload(robot, envelope)
parents = {j.find("child").get("link"): j for j in robot.findall("joint")}
vendor = Path("/home/syf/unitree_ws/unitree_ros/robots/g1_description")


def origin(element):
    if element is None:
        return np.eye(4)
    t = trimesh.transformations.euler_matrix(*map(float, element.get("rpy", "0 0 0").split()))
    t[:3, 3] = list(map(float, element.get("xyz", "0 0 0").split()))
    return t


def pose(link):
    if link == "pelvis":
        t = np.eye(4)
        t[2, 3] = 0.793
        return t
    j = parents[link]
    t = pose(j.find("parent").get("link")) @ origin(j.find("origin"))
    if j.get("type") in ("revolute", "continuous"):
        axis = list(map(float, j.find("axis").get("xyz").split()))
        t = t @ trimesh.transformations.rotation_matrix(profile["joints"].get(j.get("name"), 0), axis)
    return t


parts = {}
for link in robot.findall("link"):
    name = link.get("name")
    for kind in ("visual", "collision"):
        for i, element in enumerate(link.findall(kind)):
            g = element.find("geometry")
            if g.find("mesh") is not None:
                spec = g.find("mesh")
                uri = spec.get("filename")
                path = Path(unquote(urlparse(uri).path)) if uri.startswith("file://") else vendor / uri
                mesh = trimesh.load(path, force="mesh")
                mesh.apply_scale(list(map(float, spec.get("scale", "1 1 1").split())))
            elif g.find("box") is not None:
                mesh = trimesh.creation.box(list(map(float, g.find("box").get("size").split())))
            elif g.find("cylinder") is not None:
                spec = g.find("cylinder")
                mesh = trimesh.creation.cylinder(float(spec.get("radius")), float(spec.get("length")), sections=64)
            elif g.find("sphere") is not None:
                mesh = trimesh.creation.icosphere(radius=float(g.find("sphere").get("radius")))
            else:
                raise ValueError(f"Unsupported geometry: {name}")
            mesh.apply_transform(pose(name) @ origin(element.find("origin")))
            parts[f"{name}/{kind}/{i}"] = mesh

table = trimesh.load(ASSETS / "workbench.stl", force="mesh")
t = trimesh.transformations.euler_matrix(0, -np.pi / 2, np.pi)
table.apply_transform(t)
table.apply_translation([
    meta["pelvis_front_offset_m"] + 0.05 - table.bounds[0, 0],
    -table.bounds[:, 1].mean(),
    0.793 + meta["zero_pose_sole_z_relative_pelvis_m"] - table.bounds[0, 2],
])
manager = trimesh.collision.CollisionManager()
for name, mesh in parts.items():
    manager.add_object(name, mesh)

def check(retreat):
    shift = np.eye(4)
    shift[0, 3] = -retreat
    for name in parts:
        manager.set_transform(name, shift)
    collision, names = manager.in_collision_single(table, return_names=True)
    distance, closest = manager.min_distance_single(table, return_name=True)
    return {"retreat_m": retreat, "colliding": sorted(names),
              "clearance_m": float(distance), "closest": closest}


records = []
for millimeters in range(0, 801, 5):
    record = check(millimeters / 1000)
    records.append(record)
    if millimeters % 25 == 0 or not record["colliding"]:
        print(json.dumps(record), flush=True)
    if not record["colliding"] and record["clearance_m"] >= 0.020:
        break
else:
    raise RuntimeError("No position with 20 mm model clearance within 0.8 m")
report = {"scope": "All available robot visual/collision meshes versus STEP, static poses only",
          "pose_file": "assets/workbench/task_start_pose.json",
          "pose": profile,
          "payload_envelope": envelope,
          "requested_preview": check(0.06),
          "configuration": "pitch=-90 yaw=180; cabinet bottom aligned to feet; initial gap=0.05m",
          "step_m": 0.005, "required_clearance_m": 0.02,
          "selected": records[-1], "scan": records}
(ASSETS / "retreat_check.json").write_text(json.dumps(report, indent=2) + "\n")
print("Requested preview:", json.dumps(report["requested_preview"]))
