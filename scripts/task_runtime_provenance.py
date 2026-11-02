"""Freeze local native nodes used by offline replay, alongside the script snapshot."""
import hashlib
from pathlib import Path
import shutil

ROOT = Path(__file__).resolve().parents[1]
PACKAGE = 'g1_upper_body_moveit_config'
RUNTIME_FILES = (
    f'src/{PACKAGE}/src/single_arm_planner_server.cpp',
    f'src/{PACKAGE}/src/task_state_validator.cpp',
    f'src/{PACKAGE}/src/task_scene_revision.hpp',
    f'src/{PACKAGE}/srv/TimeTaskPath.srv',
    f'src/{PACKAGE}/srv/ValidateTaskState.srv',
    f'src/{PACKAGE}/srv/ValidateTaskTrajectory.srv',
    f'install/{PACKAGE}/lib/{PACKAGE}/single_arm_planner_server',
    f'install/{PACKAGE}/lib/{PACKAGE}/task_state_validator',
    f'install/{PACKAGE}/share/{PACKAGE}/launch/real_shadow_moveit.launch.py',
    'scripts/task_test_fastdds.xml',
)
SCOPE = ('Local native planner/validator sources and binaries, replay launch and DDS profile; '
         'external ROS shared libraries are not snapshotted')


def checksum(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def runtime_hashes(root=ROOT):
    return {name: checksum(root/name) for name in RUNTIME_FILES}


def runtime_matches(expected, root=ROOT):
    if set(expected) != set(RUNTIME_FILES):
        return False
    try:
        return runtime_hashes(root) == expected
    except OSError:
        return False


def snapshot_runtime(destination, root=ROOT):
    destination.mkdir()
    for name in RUNTIME_FILES:
        target = destination/name
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(root/name, target)
    frozen = runtime_hashes(destination)
    if not runtime_matches(frozen, root):
        raise ValueError('Native runtime changed while snapshotting')
    return frozen
