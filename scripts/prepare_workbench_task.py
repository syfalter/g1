"""Extract a movable CAD handle for the offline application scene."""
import json
from pathlib import Path
import cadquery as cq
import numpy as np
import trimesh

root = Path(__file__).resolve().parents[1]
out = root / "assets/workbench/task"
out.mkdir(parents=True, exist_ok=True)
shape = cq.importers.importStep("/home/syf/unitree_ws/台面.STEP").val()
crop = cq.Solid.makeBox(150, 100, 140, cq.Vector(150, 731.5, 210))
pieces = {"cabinet": shape.cut(crop), "handle": shape.intersect(crop)}
for name, solid in pieces.items():
    verts, faces = solid.tessellate(0.5, 0.15)
    vertices = np.array([p.toTuple() for p in verts]) / 1000
    # Corrected CAD axes: depth=CAD Z, width=CAD X, height=CAD Y.
    vertices = vertices[:, [2, 0, 1]]
    if name == "handle":
        vertices -= [0.252, 0.225, 0.7415]
    mesh = trimesh.Trimesh(vertices=vertices, faces=faces, process=True)
    mesh.export(out / f"{name}.stl")
    print(name, len(mesh.faces), mesh.bounds.tolist())
meta = json.loads((root / "assets/workbench/metadata.json").read_text())
z0 = 0.793 + meta["zero_pose_sole_z_relative_pelvis_m"]
front = meta["pelvis_front_offset_m"] + 0.05
target = {"frame_id": "world", "source": "CAD mock, uncalibrated",
          "axis_center": [front + 0.252, 0.225, z0 + 0.7415],
          "axis_quaternion_xyzw": [0., 0., 0., 1.],
          "entry_center": [front + 0.292, 0.225, z0 + 0.7415],
          "angle_deg": 120., "preinsert_height_m": 0.10,
          "robot_retreat_m": 0.06,
          "geometry_note": "CAD top cropped at source Y=731.5 mm; validate moving-part boundary and entry point physically"}
(out / "target.json").write_text(json.dumps(target, indent=2) + "\n")
