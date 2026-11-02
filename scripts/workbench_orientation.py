"""Bounded TCP swing/twist freedom for the offline index-finger task."""
import math

import numpy as np
from scipy.spatial.transform import Rotation


def orientation_policy(target):
    supplied = target.get('orientation_tolerance', {})
    defaults = dict(tilt_deg=5., roll_deg=10., step_deg=2.)
    if not isinstance(supplied, dict) or set(supplied) - set(defaults):
        raise ValueError('Unknown orientation tolerance fields')
    result = {key: float(supplied.get(key, value)) for key, value in defaults.items()}
    if (not np.isfinite(list(result.values())).all() or not 0 <= result['tilt_deg'] <= 10
            or not 0 <= result['roll_deg'] <= 20 or not 0 < result['step_deg'] <= 5):
        raise ValueError('Orientation tolerance: tilt [0,10], roll [0,20], step (0,5] degrees')
    return result


def orientation_error(nominal, actual):
    relative = Rotation.from_quat(nominal).inv() * Rotation.from_quat(actual)
    tilt = math.degrees(math.acos(float(np.clip(relative.as_matrix()[2, 2], -1., 1.))))
    q = relative.as_quat()
    roll = math.degrees(abs((2*math.atan2(q[2], q[3]) + math.pi) % (2*math.pi) - math.pi))
    return tilt, roll


def orientation_allowed(nominal, actual, policy, epsilon=1e-7):
    tilt, roll = orientation_error(nominal, actual)
    return tilt <= policy['tilt_deg'] + epsilon and roll <= policy['roll_deg'] + epsilon


def orientation_candidates(nominal, previous, policy):
    """Keep the previous pose first; bound total drift and each local change."""
    reference, last = Rotation.from_quat(nominal), Rotation.from_quat(previous)
    candidates = [last]
    step = math.radians(policy['step_deg'])
    back = (last.inv() * reference).as_rotvec()
    if np.linalg.norm(back) > 1e-12:
        candidates.append(last * Rotation.from_rotvec(back * min(1., step/np.linalg.norm(back))))
    for amount in (step/2, step):
        for axis in np.eye(3):
            for sign in (-1, 1):
                candidates.append(last * Rotation.from_rotvec(axis*sign*amount))
    result = []
    for candidate in candidates:
        quat = candidate.as_quat()
        if (orientation_allowed(nominal, quat, policy)
                and (last.inv()*candidate).magnitude() <= step + 1e-9
                and not any(abs(float(np.dot(quat, old))) > 1-1e-12 for old in result)):
            result.append(quat)
    return result
