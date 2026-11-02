"""Atomic, reload-verified publication of a completed live RGB-D plan."""
from datetime import datetime
import hashlib
import os
from pathlib import Path
import tempfile

from task_offline_export import snapshot_meshes
from task_plan_archive import digest, freeze, write_json
from task_saved_test import load_archive
from task_trajectory_library import DEFAULT_LIBRARY


def publish_live_plan(source, robot_urdf, library=None, *, check=lambda: None):
    source, robot_urdf = Path(source).resolve(), Path(robot_urdf).resolve()
    root = Path(library or DEFAULT_LIBRARY)/'live'
    root.mkdir(parents=True, exist_ok=True)
    check()
    resources = snapshot_meshes(robot_urdf, source/'model_resources')
    image_hashes = [digest(source/name) for name in ('rgb.png', 'depth_aligned.png')]
    write_json(source/'offline_provenance.json', dict(origin='live_rgbd_plan',
        source_capture=str(source), source_model_sha256=digest(robot_urdf),
        pair_id=hashlib.sha256(''.join(image_hashes).encode()).hexdigest(),
        mesh_resources=resources, hardware_commands_sent=0,
        scene_identity='live capture; not an execution authorization'))
    destination = root/(datetime.now().strftime('%Y%m%d_%H%M%S_%f')+'_'+source.name)
    with tempfile.TemporaryDirectory(prefix='.publish-', dir=root) as temporary:
        staged = Path(temporary)/'archive'
        check()
        freeze(source, staged, robot_urdf, operator_scene_unchanged_commitment=False)
        load_archive(staged, robot_urdf)
        check()
        os.rename(staged, destination)
    return str(destination)
