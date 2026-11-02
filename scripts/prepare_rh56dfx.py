"""Bake community RH56DFX meshes in a documented static open-hand pose."""
import json
import argparse
import copy
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

import numpy as np
import trimesh

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "assets/rh56dfx_description"
OUT = ROOT / "assets/workbench/rh56dfx"
parser = argparse.ArgumentParser()
parser.add_argument("--task-pose", action="store_true")
parser.add_argument("--other-fingers-open", action="store_true")
args = parser.parse_args()
if args.other_fingers_open and not args.task_pose:
    parser.error('--other-fingers-open requires --task-pose')
if args.task_pose:
    OUT = OUT / ("task_open" if args.other_fingers_open else "task")
OUT.mkdir(parents=True, exist_ok=True)
report = {"source_repository": "https://github.com/ookkshirsagar/rh56dfx_description",
          "source_commit": "77491738aaed410d1b668749160e7b943f6fdd46",
          "source_status": "Community model, not manufacturer-verified",
          "pose": "Static open fingers; proximal flexion=0.15/1.1169, distal=0, thumb=0",
          "collision": "Union of individual source collision-link convex hulls; conservative per-link approximation",
          "mount": "Approximate: palm-base rear bounding plane, centred transverse to finger direction",
          "excluded": "Upstream hand_root, wrist_base_link, wrist_yaw (G1 supplies wrist)",
          "hands": {}}


def origin(e):
    if e is None:
        return np.eye(4)
    t = trimesh.transformations.euler_matrix(*map(float, e.get("rpy", "0 0 0").split()))
    t[:3, 3] = list(map(float, e.get("xyz", "0 0 0").split()))
    return t


for side in ("left", "right"):
    wrapper = f'''<robot name="rh56dfx" xmlns:xacro="http://ros.org/wiki/xacro">
      <xacro:include filename="{SOURCE}/urdf/rh56dfx_macro.xacro"/>
      <xacro:rh56dfx_hand side="{side}" prefix="{side}_"/>
    </robot>'''
    xml = subprocess.check_output(["/usr/bin/python3", "-c",
        "import sys,xacro; d=xacro.parse(sys.argv[1]); xacro.process_doc(d); print(d.toxml())", wrapper], text=True)
    robot = ET.fromstring(xml)
    joints = {j.get("name"): j for j in robot.findall("joint")}
    parents = {j.find("child").get("link"): j for j in joints.values()}
    base = side + "_hand_base"

    def angle(j):
        mimic = j.find("mimic")
        if mimic is not None:
            return angle(joints[mimic.get("joint")]) * float(mimic.get("multiplier", "1")) + float(mimic.get("offset", "0"))
        name = j.get("name")
        if args.task_pose and not args.other_fingers_open:
            if name in [side + "_" + f + "_1_joint" for f in ("middle", "ring", "little")]:
                return 1.2
            if name == side + "_thumb_1_joint":
                return 0.8
            if name == side + "_thumb_2_joint":
                return 0.4
        return 0.15 / 1.1169 if name in [side + "_" + f + "_1_joint" for f in ("index", "middle", "ring", "little")] else 0.0

    def pose(name):
        if name == base:
            return np.eye(4)
        if name not in parents:
            return None
        j = parents[name]
        parent = pose(j.find("parent").get("link"))
        if parent is None:
            return None
        t = parent @ origin(j.find("origin"))
        if j.get("type") == "revolute":
            q = angle(j)
            limit = j.find("limit")
            assert float(limit.get("lower")) - 1e-8 <= q <= float(limit.get("upper")) + 1e-8
            t = t @ trimesh.transformations.rotation_matrix(q, list(map(float, j.find("axis").get("xyz").split())))
        return t

    # Finger direction +Y -> G1 hand +X; palm normals face inward for both sides.
    mount = trimesh.transformations.euler_matrix(0, 0, -np.pi / 2)
    palm = trimesh.load(SOURCE / f"meshes/{side}/visual/{side}_hand_base.STL", force="mesh")
    palm.apply_transform(mount)
    mount[:3, 3] = [-palm.bounds[0, 0], -palm.bounds[:, 1].mean(), -palm.bounds[:, 2].mean()]
    details = {"mount_transform": mount.tolist()}
    tip = (mount @ pose(side + "_index_tip"))[:3, 3]
    distal = (mount @ pose(side + "_index_2"))[:3, 3]
    z_axis = (tip - distal) / np.linalg.norm(tip - distal)
    x_axis = np.array([0., 1., 0.])
    x_axis -= np.dot(x_axis, z_axis) * z_axis
    x_axis /= np.linalg.norm(x_axis)
    rotation = np.column_stack((x_axis, np.cross(z_axis, x_axis), z_axis))
    transform = np.eye(4); transform[:3, :3] = rotation
    details["index_tcp_xyz"] = tip.tolist()
    details["index_tcp_rpy"] = list(trimesh.transformations.euler_from_matrix(transform))
    if args.task_pose:
        assembly = ET.Element('robot', name=side+'_task_index')
        index_links = [link for link in robot.findall('link') if '_index_' in link.get('name')]
        index_names = {link.get('name') for link in index_links}
        for link in index_links:
            link = copy.deepcopy(link)
            for kind in ('visual', 'collision'):
                for element in link.findall(kind):
                    spec = element.find('geometry/mesh')
                    relative = spec.get('filename').removeprefix('package://rh56dfx_description/')
                    path = SOURCE / relative
                    if kind == 'collision':
                        hull = trimesh.load(path, force='mesh').convex_hull
                        path = OUT / (link.get('name')+'_hull.stl')
                        hull.export(path)
                    spec.set('filename', path.as_uri())
            assembly.append(link)
        for joint in robot.findall('joint'):
            if joint.find('child').get('link') not in index_names:
                continue
            joint = copy.deepcopy(joint)
            if joint.find('parent').get('link') == base:
                joint.find('parent').set('link', side+'_rubber_hand')
                t = mount @ origin(joint.find('origin'))
                joint.find('origin').set('xyz', ' '.join(map(str,t[:3,3])))
                joint.find('origin').set('rpy', ' '.join(map(str,trimesh.transformations.euler_from_matrix(t))))
            assembly.append(joint)
        tcp = np.eye(4);tcp[:3,:3]=rotation;tcp[:3,3]=tip
        local = np.linalg.inv(mount @ pose(side+'_index_tip')) @ tcp
        ET.SubElement(assembly,'link',name=side+'_index_tcp')
        fixed = ET.SubElement(assembly,'joint',name=side+'_index_tcp_fixed',type='fixed')
        ET.SubElement(fixed,'parent',link=side+'_index_tip')
        ET.SubElement(fixed,'child',link=side+'_index_tcp')
        ET.SubElement(fixed,'origin',xyz=' '.join(map(str,local[:3,3])),rpy=' '.join(map(str,trimesh.transformations.euler_from_matrix(local))))
        ET.ElementTree(assembly).write(OUT / (side+'_index.urdf'),encoding='unicode')
    for kind in ("visual", "collision"):
        pieces = []
        index_pieces = []
        body_pieces = []
        for link in robot.findall("link"):
            transform = pose(link.get("name"))
            if transform is None:
                continue
            for element in link.findall(kind):
                spec = element.find("geometry/mesh")
                if spec is None:
                    raise ValueError("Expected mesh geometry")
                relative = spec.get("filename").removeprefix("package://rh56dfx_description/")
                m = trimesh.load(SOURCE / relative, force="mesh")
                if kind == "collision":
                    m = m.convex_hull
                m.apply_transform(mount @ transform @ origin(element.find("origin")))
                pieces.append(m)
                (index_pieces if "_index_" in link.get("name") else body_pieces).append(m)
        merged = trimesh.util.concatenate(pieces)
        merged.export(OUT / f"{side}_{kind}.stl")
        if args.task_pose:
            trimesh.util.concatenate(index_pieces).export(OUT / f"{side}_index_{kind}.stl")
            trimesh.util.concatenate(body_pieces).export(OUT / f"{side}_body_{kind}.stl")
        details[kind] = {"bounds_m": merged.bounds.tolist(), "triangles": len(merged.faces)}
    report["hands"][side] = details
if args.task_pose:
    report["pose"] = ("All fingers open; non-index fingers are static open meshes" if args.other_fingers_open else
        "Index extended, other three fingers flexed 1.2 rad, thumb rotation 0.8/flexion 0.4 rad; offline kinematic pose only")
    report["articulated_index"] = "Per-side index URDF preserves source joints/mimic. Application selects initial posture; not calibrated actuator values."
(OUT / "provenance.json").write_text(json.dumps(report, indent=2) + "\n")
print(json.dumps(report, indent=2))
