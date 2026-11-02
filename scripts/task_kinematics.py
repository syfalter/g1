"""Task-only waist groups and immutable capture-frame geometry."""
import math
import xml.etree.ElementTree as ET
import numpy as np
from task_session import ARMS

WAIST = 'waist_yaw_joint'
WAIST_LIMIT = math.radians(45.)


def mirror_left_ready(model, record):
    """Mirror the saved arm posture about the pelvis sagittal plane, not its path."""
    source = record['joints']
    target = {}
    for left, right, sign in zip(ARMS['left'], ARMS['right'], (1, -1, -1, 1, -1, 1, -1)):
        value = float(source[left])
        if not math.isfinite(value): raise ValueError('Nonfinite left preparation joint: '+left)
        target[right] = sign*value
    waist_joint = model.robot.find(f"joint[@name='{WAIST}']")
    if WAIST in source and waist_joint is not None and waist_joint.get('type') != 'fixed':
        target[WAIST] = -float(source[WAIST])
    for name, value in target.items():
        lower, upper = model.limits[name]
        if not math.isfinite(value) or not lower+.02 <= value <= upper-.02:
            raise ValueError('Mirrored preparation exceeds guarded joint range: '+name)
    reflected = np.diag([1., -1., 1., 1.])
    left_frame = model.transform('left_wrist_yaw_link', source)
    right_frame = model.transform('right_wrist_yaw_link', dict(source, **target))
    expected = reflected @ left_frame @ reflected
    position_error = float(np.linalg.norm(right_frame[:3, 3]-expected[:3, 3]))
    if position_error > 2e-5 or not np.allclose(right_frame[:3, :3], expected[:3, :3], atol=1e-6, rtol=0):
        raise ValueError('Robot model does not match the expected left/right preparation symmetry')
    return dict(label='right_mirrored_left_preparation', joints=target, mirrored_from='left',
                source_joints={n:source[n] for n in (*ARMS['left'], WAIST) if n in source},
                source_captured_at=record.get('captured_at'),
                wrist_mirror_error_m=position_error, hardware_execution_validated=False)


def arm_names(side, waist=False):
    return ARMS[side] + ((WAIST,) if waist else ())


def arm_group(side, waist=False):
    return side + ('_arm_with_waist' if waist else '_arm')


def configure_waist(robot, semantic=None):
    joint=robot.find(f"joint[@name='{WAIST}']")
    if joint is None or joint.get('type') != 'revolute':
        raise ValueError('Task waist assistance requires a revolute waist yaw joint')
    joint.find('limit').set('lower',str(-WAIST_LIMIT))
    joint.find('limit').set('upper',str(WAIST_LIMIT))
    for name in ('waist_roll_joint','waist_pitch_joint'):
        other=robot.find(f"joint[@name='{name}']")
        if other is not None and other.get('type')!='fixed':
            raise ValueError('Waist roll/pitch must remain fixed for task assistance')
    if semantic is not None:
        for side in ARMS:
            group=ET.SubElement(semantic,'group',name=arm_group(side,True))
            ET.SubElement(group,'chain',base_link='pelvis',tip_link=side+'_tool0')


def capture_transform(geometry, model, start):
    saved=geometry.get('capture_frame',{})
    value=(saved['pelvis_from_torso_row_major'] if 'pelvis_from_torso_row_major' in saved else
           model.transform('torso_link',start))
    transform=np.asarray(value,dtype=float)
    if (transform.shape!=(4,4) or not np.isfinite(transform).all() or
            not np.allclose(transform[3],[0,0,0,1]) or
            not np.allclose(transform[:3,:3].T@transform[:3,:3],np.eye(3),atol=1e-6) or
            not np.isclose(np.linalg.det(transform[:3,:3]),1.)):
        raise ValueError('Invalid capture transform')
    return transform.copy()
