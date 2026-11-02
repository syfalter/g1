"""Synthetic height experiments relative to an archived observation, not calibration."""
from copy import deepcopy
import math
from task_kinematics import capture_transform


def raised_preview_geometry(geometry,model,start,raise_m):
    if not math.isfinite(raise_m) or not 0<=raise_m<=.30:
        raise ValueError('Additional preview height must be between 0 and 0.30 m')
    result=deepcopy(geometry)
    transform=capture_transform(geometry,model,start)
    # The world target stays fixed while the new pelvis origin moves upward.
    transform[2,3]-=raise_m
    result['capture_frame']={'frame':'pelvis','pelvis_from_torso_row_major':transform.tolist(),
        'synthetic_robot_raise_m':raise_m,
        'note':'Additional robot height relative to archive, NOT a measured camera extrinsic'}
    return result
