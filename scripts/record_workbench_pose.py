"""Convert a stationary LowState capture to named, bilateral wrist references."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import re

import numpy as np
from scipy.spatial.transform import Rotation

from capture_workbench_start_pose import JOINTS, validate_window
from workbench_model import OfflineModel

VENDOR_MODEL = Path('/home/syf/unitree_ws/unitree_ros/robots/g1_description/g1_29dof_lock_waist_rev_1_0.urdf')


def build_record(profile, model_path, label, platform_height):
    if not re.fullmatch(r'[A-Za-z0-9_-]{1,64}', label):
        raise ValueError('Pose label must contain 1-64 ASCII letters, digits, underscores or hyphens')
    if not math.isfinite(platform_height) or not 0 <= platform_height <= .3:
        raise ValueError('Invalid platform height')
    q = {name: float(profile['joints'][name]) for name in JOINTS}
    if not all(math.isfinite(value) for value in q.values()):
        raise ValueError('Nonfinite captured joints')
    model = OfflineModel(model_path)
    links = {link.get('name') for link in model.robot.findall('link')}
    poses = {}
    for frame in ('torso_link', 'pelvis'):
        if frame not in links:
            raise ValueError('Missing reference frame: ' + frame)
        poses[frame] = {}
        reference = np.linalg.inv(model.transform(frame, q))
        for side in ('left', 'right'):
            link = side + '_wrist_yaw_link'
            if link not in links:
                raise ValueError('Missing wrist link: ' + link)
            transform = reference @ model.transform(link, q)
            poses[frame][side] = dict(link_name=link, position_m=transform[:3, 3].tolist(),
                quaternion_xyzw=Rotation.from_matrix(transform[:3, :3]).as_quat().tolist())
    return dict(label=label, captured_at=profile['captured_at'], joints=q,
        measured_joints=profile['measured_joints'],
        unmeasured_locked_joint_assumptions=profile['unmeasured_locked_joint_assumptions'],
        quality=profile['quality'], wrist_poses=poses,
        model_path=str(model_path), model_sha256=hashlib.sha256(model_path.read_bytes()).hexdigest(),
        platform_height_m=platform_height, platform_height_source='Operator-reported; not measured by capture',
        world_registration_verified=False, hardware_execution_validated=False,
        hand_note='No finger angles saved. Default open is an assumption, not a measurement.',
        reference_note='Measured joint reference; wrist poses are nominal URDF FK, not calibrated fingertip poses.')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--capture', required=True, type=Path)
    parser.add_argument('--label', required=True)
    parser.add_argument('--platform-height', type=float, default=.07)
    parser.add_argument('--model', type=Path, default=VENDOR_MODEL)
    args = parser.parse_args()
    profile = json.loads((args.capture / 'startup_pose.json').read_text())
    samples = json.loads((args.capture / 'sampling_window.json').read_text())
    # Revalidate stored evidence; its age is the capture-time age, not live telemetry.
    validate_window(samples, profile['quality']['last_receive_age_s'])
    if any(profile['joints'][name] != value for name, value in zip(JOINTS, samples[-1]['q'])):
        raise ValueError('Saved reference does not match the final captured frame')
    record = build_record(profile, args.model, args.label, args.platform_height)
    output = args.capture / 'task_pose.json'
    with output.open('x') as stream:
        json.dump(record, stream, indent=2, allow_nan=False)
        stream.write('\n')
    print(json.dumps(dict(output=str(output), label=args.label, wrist_poses=record['wrist_poses']), indent=2))


if __name__ == '__main__':
    main()
