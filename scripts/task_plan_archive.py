"""Freeze a task plan and derive read-only segment records. No ROS or actuators."""
import argparse
from datetime import datetime
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import tempfile

from task_session import ARMS, ARM_NAMES
from task_hand_control import model_joints


def digest(path):
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest() if hasattr(hashlib, 'file_digest') else _digest(stream)


def _digest(stream):
    checksum = hashlib.sha256()
    for block in iter(lambda: stream.read(1024 * 1024), b''):
        checksum.update(block)
    return checksum.hexdigest()


def seconds(point):
    stamp = point['time_from_start']
    if (type(stamp['sec']) is not int or type(stamp['nanosec']) is not int or
            stamp['sec'] < 0 or not 0 <= stamp['nanosec'] < 1000000000):
        raise ValueError('Invalid trajectory time')
    return stamp['sec'] + stamp['nanosec'] * 1e-9


def finite(values):
    return all(type(v) in (int, float) and math.isfinite(v) for v in values)


def stages(plan):
    """Reconstruct full starts/ends without changing the original trajectories."""
    side = plan['arm']
    if plan['kind'] != 'task' or side not in ARMS or not plan['segments']:
        raise ValueError('Need a single-arm complete task plan')
    state = dict(plan['start'])
    if not set(ARM_NAMES).issubset(state) or not finite(state.values()):
        raise ValueError('Missing or invalid baseline joints')
    records = []
    for index, segment in enumerate(plan['segments'], 1):
        if segment['side'] != side or segment['kind'] not in ('arm', 'hand'):
            raise ValueError('Invalid segment side/kind')
        trajectory = segment['trajectory']
        names, points = trajectory['joint_names'], trajectory['points']
        allowed = set(ARMS[side])
        if segment['kind'] == 'hand':
            allowed = set(model_joints(side, segment['target_q']))
        elif plan.get('waist_assist'):
            allowed.add('waist_yaw_joint')
        if len(names) != len(set(names)) or set(names) != allowed or len(points) < 2:
            raise ValueError('Incomplete trajectory joints/points')
        previous = -1.
        for point in points:
            stamp = seconds(point)
            if stamp <= previous:
                raise ValueError('Non-increasing trajectory time')
            previous = stamp
            for field in ('positions', 'velocities', 'accelerations', 'effort'):
                values = point.get(field, [])
                required = field == 'positions' or (
                    segment['kind'] == 'arm' and field in ('velocities', 'accelerations'))
                if ((required or values) and len(values) != len(names)) or not finite(values):
                    raise ValueError('Invalid trajectory array: ' + field)
        if seconds(points[0]) != 0 or seconds(points[-1]) <= 0:
            raise ValueError('Trajectory must start at zero with positive duration')
        first = dict(zip(names, points[0]['positions']))
        if any(n not in state or abs(state[n] - v) > 1e-7 for n, v in first.items()):
            raise ValueError('Discontinuous segment start: ' + segment['label'])
        if segment['kind'] == 'arm' and any(
                abs(v) > 1e-6 for p in (points[0], points[-1])
                for field in ('velocities','accelerations') for v in p[field]):
            raise ValueError('Segment boundary is not at rest: velocity/acceleration must be zero')
        end = dict(state)
        end.update(zip(names, points[-1]['positions']))
        if segment['kind'] == 'hand' and any(
                abs(end[n] - v) > 1e-7 for n, v in model_joints(side, segment['target_q']).items()):
            raise ValueError('Hand endpoint disagrees with DDS target')
        records.append(dict(number=index, label=segment['label'], side=side,
                            kind=segment['kind'], start=state, end=end,
                            duration_s=seconds(points[-1]), point_count=len(points),
                            hardware_executable=False, original_segment=segment))
        state = end
    return records


def write_json(path, data):
    Path(path).write_text(json.dumps(data, indent=2, allow_nan=False) + '\n')


def verify_files(directory):
    """Verify immutable file provenance, not trajectory execution validity."""
    directory = Path(directory)
    manifest = json.loads((directory / 'archive_manifest.json').read_text())
    actual = {str(p.relative_to(directory)) for p in directory.rglob('*') if p.is_file()
              and p != directory / 'archive_manifest.json'}
    if actual != set(manifest['sha256']):
        raise ValueError('Archive file set changed')
    for name, checksum in manifest['sha256'].items():
        path = directory / name
        if path.is_symlink() or not path.resolve().is_relative_to(directory.resolve()) or digest(path) != checksum:
            raise ValueError('Archive checksum mismatch: ' + name)
    return manifest


def verify(directory):
    directory = Path(directory)
    manifest = verify_files(directory)
    derived = stages(json.loads((directory / 'plan/plan.json').read_text()))
    for stage in derived:
        if stage != json.loads((directory / f'stages/{stage["number"]:02d}.json').read_text()):
            raise ValueError('Derived stage differs from original plan')
    return manifest


def freeze(source, destination, robot_urdf, *, operator_scene_unchanged_commitment=True):
    source, destination = Path(source).resolve(), Path(destination).resolve()
    if destination.exists():
        raise FileExistsError('Archive already exists; will not overwrite')
    required = ('plan.json', 'capture.json', 'capture_state.json', 'geometry.json',
                'rgb.png', 'depth_aligned.png', 'intrinsics.json')
    for name in required:
        if not (source / name).is_file():
            raise FileNotFoundError(source / name)
    files = [p for p in source.rglob('*') if p.is_file()]
    if any(p.is_symlink() for p in source.rglob('*')):
        raise ValueError('Source archive must not contain symlinks')
    checksums = {str(p.relative_to(source)): digest(p) for p in files}
    records = stages(json.loads((source / 'plan.json').read_text()))
    destination.parent.mkdir(parents=True, exist_ok=True)
    temporary = Path(tempfile.mkdtemp(prefix='.freezing-', dir=destination.parent))
    try:
        shutil.copytree(source, temporary / 'plan')
        for name, checksum in checksums.items():
            if digest(source / name) != checksum or digest(temporary / 'plan' / name) != checksum:
                raise ValueError('Source changed during archival')
        context = temporary / 'context'
        context.mkdir()
        for name in ('session_home.json', 'ready_poses.json', 'index_tcp.json', 'handoff_source.json'):
            if (source.parent / name).is_file():
                shutil.copy2(source.parent / name, context / name)
        shutil.copy2(robot_urdf, context / 'robot.urdf')
        code = context / 'code'
        code.mkdir()
        for name in ('real_task_app.py', 'task_session.py', 'task_motion_planner.py',
                     'task_scoped_planner.py', 'task_straight_policy.py', 'task_trajectory.py', 'task_dring_sequence.py',
                     'task_hand_control.py', 'task_dring_hand_settings.py', 'task_hand_model.py', 'workbench_model.py',
                     'task_plan_archive.py','task_model_resources.py','task_ik_search.py',
                     'task_trajectory_speed.py','task_live_archive.py','task_archive_scene.py',
                     'task_speed_control.py','task_trajectory_library.py','task_path_continuity.py',
                     'task_path_smoothing.py','task_local_timing.py','task_cruise_timing.py'):
            shutil.copy2(Path(__file__).parent / name, code / name)
        (temporary / 'stages').mkdir()
        for record in records:
            write_json(temporary / f'stages/{record["number"]:02d}.json', record)
        manifest = dict(schema_version=1, saved_at=datetime.now().astimezone().isoformat(),
                        source_plan_directory=str(source), source_robot_urdf=str(robot_urdf),
                        original_plan_sha256=checksums['plan.json'],
                        hardware_executable=False, hardware_commands_sent=False,
                        operator_scene_unchanged_commitment=operator_scene_unchanged_commitment,
                        notes=['Archive is not an execution authorization.',
                               'Check fresh arm, waist and hand starts before each stage.',
                               'STOP or unexpected motion requires reassessment, never automatic continuation.',
                               'URDF mesh references remain external; code is a disk snapshot, not runtime attestation.'],
                        stages=[{k: r[k] for k in ('number', 'label', 'kind', 'duration_s', 'point_count')}
                                for r in records])
        manifest['sha256'] = {str(p.relative_to(temporary)): digest(p)
                              for p in sorted(temporary.rglob('*')) if p.is_file()}
        write_json(temporary / 'archive_manifest.json', manifest)
        verify(temporary)
        for path in temporary.rglob('*'):
            if path.is_file():
                path.chmod(0o444)
        os.rename(temporary, destination)
        return manifest
    except Exception:
        shutil.rmtree(temporary)
        raise


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--verify', type=Path)
    parser.add_argument('--source', type=Path)
    parser.add_argument('--output', type=Path)
    parser.add_argument('--robot-urdf', type=Path)
    args = parser.parse_args()
    if args.verify:
        manifest = verify(args.verify)
    elif all((args.source, args.output, args.robot_urdf)):
        manifest = freeze(args.source, args.output, args.robot_urdf)
    else:
        parser.error('Use --verify ARCHIVE or --source PLAN --output ARCHIVE --robot-urdf URDF')
    print(json.dumps({k: v for k, v in manifest.items() if k != 'sha256'}, indent=2))


if __name__ == '__main__':
    main()
