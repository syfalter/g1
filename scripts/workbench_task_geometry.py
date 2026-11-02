"""Geometry contract shared by the offline UI and task planner."""
import numpy as np
from scipy.spatial.transform import Rotation
from workbench_orientation import orientation_policy


def parse_target(data):
    orientation_policy(data)
    c = np.asarray(data["axis_center"], dtype=float)
    p = np.asarray(data["entry_center"], dtype=float)
    q = np.asarray(data["axis_quaternion_xyzw"], dtype=float)
    if c.shape != (3,) or p.shape != (3,) or q.shape != (4,) or not np.isfinite(np.r_[c, p, q]).all():
        raise ValueError("Target pose has invalid dimensions or nonfinite values")
    if np.linalg.norm(q) < 1e-8:
        raise ValueError("Invalid axis quaternion")
    angle = float(data.get('angle_deg', 120))
    height = float(data.get('preinsert_height_m', .1))
    if not np.isfinite([angle, height]).all() or not 0 < angle <= 120 or not .02 <= height <= .30:
        raise ValueError("Angle must be in (0,120] deg and approach height in [0.02,0.30] m")
    if data.get('frame_id', 'world') != 'world':
        raise ValueError("Transform target to world before planning")
    n = Rotation.from_quat(q).apply([0., 0., 1.])
    d = p-c
    if abs(np.dot(d, n)) > 0.003:
        raise ValueError("Entry point must lie in the shaft-top plane (3 mm tolerance)")
    r = d - n*np.dot(d, n)
    if not .005 <= np.linalg.norm(r) <= .25:
        raise ValueError("Entry radius outside 5 to 250 mm")
    return c, p, n, r


def arc_point(data, angle):
    c, p, n, r = parse_target(data)
    return c + n*np.dot(p-c, n) + Rotation.from_rotvec(n*angle).apply(r)


def insertion_orientation(data, yaw, tilt):
    _, _, n, _ = parse_target(data)
    basis = Rotation.from_quat(data["axis_quaternion_xyzw"]).as_matrix()
    horizontal = basis[:, 0]*np.cos(yaw) + basis[:, 1]*np.sin(yaw)
    z = -n*np.cos(tilt) + horizontal*np.sin(tilt)
    x = np.cross(n, horizontal)
    y = np.cross(z, x)
    return Rotation.from_matrix(np.column_stack((x, y, z))).as_quat()
