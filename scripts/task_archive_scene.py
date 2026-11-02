"""Freeze a separate RGB-D overlay in capture-time pelvis coordinates; never authorize motion."""
import json
from pathlib import Path
import shutil

import numpy as np

from task_kinematics import capture_transform
from task_plan_archive import digest, write_json


def snapshot_overlay(capture, output, archived_plan, archived_geometry, model):
    capture, output = Path(capture).resolve(), Path(output)
    required = ('rgb.png', 'depth_aligned.png', 'cloud.npz', 'intrinsics.json',
                'geometry.json', 'capture_state.json', 'manifest.json')
    hashes = {name:digest(capture/name) for name in required}
    geometry = json.loads((capture/'geometry.json').read_text())
    state = json.loads((capture/'capture_state.json').read_text())
    if 'pelvis_from_torso_row_major' not in geometry.get('capture_frame', {}):
        raise ValueError('Current RGB-D requires its capture-time pelvis transform')
    frame = capture_transform(geometry, model, state['joints'])
    if not np.allclose(frame, model.transform('torso_link', state['joints']), atol=1e-6, rtol=0.):
        raise ValueError('Current scene capture frame does not match its measured state')
    if geometry.get('calibration') != archived_geometry.get('calibration'):
        raise ValueError('Current and archived camera/TCP calibration differ; recapture or replan')
    old_frame = capture_transform(archived_geometry, model, archived_plan['start'])
    errors = {}
    for name in ('axis_center_m', 'insertion_reference_m', 'arc_center_m'):
        old = archived_geometry.get('task_preview', {}).get(name)
        new = geometry.get('task_preview', {}).get(name)
        if old is not None and new is not None:
            a = old_frame[:3, :3]@np.asarray(old)+old_frame[:3, 3]
            b = frame[:3, :3]@np.asarray(new)+frame[:3, 3]
            distance = float(np.linalg.norm(a-b))
            if not np.isfinite(distance):
                raise ValueError('Nonfinite scene target geometry')
            errors[name] = distance
    with np.load(capture/'cloud.npz', allow_pickle=False) as cloud:
        xyz = cloud['xyz']
        if xyz.ndim != 2 or xyz.shape[1] != 3 or not len(xyz) or not np.isfinite(xyz).all():
            raise ValueError('Invalid current scene point cloud')
    from vision_task_preview import load_cloud
    load_cloud(capture,geometry)
    output.mkdir(parents=True, exist_ok=False)
    for name in required:
        shutil.copyfile(capture/name, output/name)
        if digest(output/name) != hashes[name] or digest(capture/name) != hashes[name]:
            raise ValueError('Current scene changed while taking its visualization snapshot')
    record = dict(source_capture=str(capture), capture=str(output.resolve()), sha256=hashes,
        captured_at=state.get('captured_at'), pelvis_from_torso_row_major=frame.tolist(),
        target_displacement_m=errors,
        target_within_3mm=(all(name in errors for name in ('axis_center_m', 'insertion_reference_m'))
                             and all(value <= .003 for value in errors.values())),
        path_transformed=False, current_scene_collision_validated=False,
        hardware_executable=False, hardware_commands_sent=0)
    write_json(output/'overlay.json', record)
    return record
