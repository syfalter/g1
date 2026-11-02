"""Convert the supplied STEP to a metre-scale, front-edge-origin preview mesh."""
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import cadquery as cq
import numpy as np
import trimesh

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "assets" / "workbench"
OUT.mkdir(parents=True, exist_ok=True)
shape = cq.importers.importStep("/home/syf/unitree_ws/台面.STEP").val()
vertices, triangles = shape.tessellate(0.5, 0.15)
v = np.array([p.toTuple() for p in vertices]) * 0.001
# CAD +Y becomes robot +X, CAD +X becomes robot -Y (proper rotation).
v = np.column_stack((v[:, 1], -v[:, 0], v[:, 2]))
v[:, 0] -= v[:, 0].min()
v[:, 1] -= (v[:, 1].min() + v[:, 1].max()) / 2
mesh = trimesh.Trimesh(vertices=v, faces=triangles, process=True)
mesh.export(OUT / "workbench.stl")
pelvis_dir = Path("/home/syf/unitree_ws/unitree_ros/robots/g1_description/meshes")
front = max(trimesh.load(pelvis_dir / name).bounds[1, 0]
            for name in ("pelvis.STL", "pelvis_contour_link.STL"))


def origin_matrix(origin):
    if origin is None:
        return np.eye(4)
    rpy = [float(x) for x in origin.get("rpy", "0 0 0").split()]
    matrix = trimesh.transformations.euler_matrix(*rpy)
    matrix[:3, 3] = [float(x) for x in origin.get("xyz", "0 0 0").split()]
    return matrix


# The isolated preview publishes zero joint positions, not measured robot state.
robot = ET.parse(pelvis_dir.parent / "g1_29dof_lock_waist_rev_1_0.urdf").getroot()
parents = {j.find("child").get("link"): j for j in robot.findall("joint")}


def zero_pose(link):
    if link == "pelvis":
        return np.eye(4)
    joint = parents[link]
    return zero_pose(joint.find("parent").get("link")) @ origin_matrix(joint.find("origin"))


sole_heights = []
for side in ("left", "right"):
    name = f"{side}_ankle_roll_link"
    link = robot.find(f"link[@name='{name}']")
    for visual in link.findall("visual"):
        mesh_element = visual.find("geometry/mesh")
        foot = trimesh.load(pelvis_dir.parent / mesh_element.get("filename"))
        scale = np.array([float(x) for x in mesh_element.get("scale", "1 1 1").split()])
        points = trimesh.transform_points(foot.vertices * scale, zero_pose(name) @ origin_matrix(visual.find("origin")))
        sole_heights.append(float(points[:, 2].min()))
metadata = {
    "source": "/home/syf/unitree_ws/台面.STEP",
    "bounds_m": mesh.bounds.tolist(),
    "dimensions_depth_width_height_m": mesh.extents.tolist(),
    "tabletop_local_z_m": 0.83,
    "pelvis_front_offset_m": float(front),
    "zero_pose_sole_z_relative_pelvis_m": min(sole_heights),
    "triangles": len(mesh.faces),
    "orientation_assumption": "CAD +Y -> robot +X, CAD +X -> robot -Y",
}
(OUT / "metadata.json").write_text(json.dumps(metadata, indent=2) + "\n")
print(json.dumps(metadata, indent=2))
