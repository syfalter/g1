"""Archive paired service images and deproject aligned depth for read-only RViz."""
import argparse
from datetime import datetime
import hashlib
import json
import os
from pathlib import Path
import re
import shlex
import shutil
import subprocess
import tempfile

import cv2
import numpy as np

from vision_task_geometry import (
    EXPECTED_CAMERA_FRAME, _profile_for_observation, torso_color_transform,
)

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_INTRINSICS = Path('/home/syf/unitree_ws/head_camera_intrinsics.json')


def validate_capture_result(capture, kind):
    if kind not in ('prepare','recover','task'):
        raise ValueError('Unsupported visual capture purpose')
    status_code=capture.get('status_result')
    if status_code == 3104:
        raise TimeoutError('Vision DDS status reply timed out (3104); detection was not started')
    if status_code != 0:
        raise RuntimeError('Vision service status failed: '+str(status_code))
    status=capture.get('status',{})
    if status.get('camera_ready') is not True or status.get('model_ready') is not True:
        raise RuntimeError('Vision camera/model not ready: '+str(status.get('last_error','')))
    observation=capture.get('observation')
    code=capture.get('observation_result')
    if code == 3104:
        raise TimeoutError('Vision recognition reply timed out (3104); no confirmed image result')
    if code not in ((0,102) if kind in ('prepare','recover') else (0,)):
        message=observation.get('message','') if isinstance(observation,dict) else ''
        raise RuntimeError(f'Vision detection failed ({code}): {message}')
    if (not isinstance(observation,dict) or observation.get('code') != code or
            type(capture.get('request_id')) is not int or capture['request_id'] <= 0 or
            observation.get('request_id') != capture['request_id'] or
            not observation.get('capture_result')):
        raise ValueError('Missing or mismatched saved visual capture')
    return observation


def measured_scene_geometry(capture, manifest):
    observation=capture['observation']
    if (manifest['source_sequence'] != observation['sequence'] or
            manifest['source_request_id'] != capture['request_id']):
        raise ValueError('Measured scene does not match capture')
    return dict(frame_id='torso_link',preview_only=True,task_target_available=False,
                source={key:observation.get(key) for key in
                        ('sequence','request_id','captured_at_ms','camera_serial','camera_frame')},
                transforms={'torso_from_color_optical_row_major':
                            manifest['torso_from_color_optical_row_major']})


def aligned_cloud(depth, bgr, intrinsics, depth_scale, transform, stride=3,
                  min_depth=.1, max_depth=2.0):
    """Return measured XYZ/RGB/UV, with no projection onto the CAD surface."""
    if depth.dtype != np.uint16 or depth.ndim != 2:
        raise ValueError('Expected original uint16 aligned depth PNG')
    if bgr.dtype != np.uint8 or bgr.shape != (*depth.shape, 3):
        raise ValueError('RGB and depth dimensions/encoding differ')
    if (intrinsics['height'], intrinsics['width']) != depth.shape:
        raise ValueError('Image dimensions do not match color intrinsics')
    if not isinstance(stride, int) or not 1 <= stride <= 16:
        raise ValueError('Stride must be an integer in [1,16]')
    values = np.asarray([intrinsics[k] for k in ('fx', 'fy', 'cx', 'cy')], float)
    coeffs = np.asarray(intrinsics.get('distortion', []), float)
    if (not np.isfinite(values).all() or np.any(values[:2] <= 0)
            or coeffs.shape != (5,) or not np.isfinite(coeffs).all()
            or not np.allclose(coeffs, 0., atol=1e-10)):
        raise ValueError('Validated zero-distortion color intrinsics required')
    if not np.isfinite([depth_scale, min_depth, max_depth]).all() or not (
            0 < depth_scale < .01 and 0 < min_depth < max_depth <= 10):
        raise ValueError('Invalid depth units/range')
    transform = np.asarray(transform, float)
    if (transform.shape != (4, 4) or not np.isfinite(transform).all()
            or not np.allclose(transform[3], [0, 0, 0, 1])
            or not np.allclose(transform[:3, :3].T @ transform[:3, :3], np.eye(3), atol=1e-5)
            or not np.isclose(np.linalg.det(transform[:3, :3]), 1., atol=1e-5)):
        raise ValueError('Invalid rigid optical-to-torso transform')
    v, u = np.mgrid[0:depth.shape[0]:stride, 0:depth.shape[1]:stride]
    z = depth[v, u].astype(float) * depth_scale
    valid = (z >= min_depth) & (z <= max_depth)
    u, v, z = u[valid], v[valid], z[valid]
    fx, fy, cx, cy = values
    optical = np.column_stack(((u-cx)*z/fx, (v-cy)*z/fy, z))
    xyz = optical @ transform[:3, :3].T + transform[:3, 3]
    return (xyz.astype(np.float32), bgr[v, u, ::-1].copy(),
            np.column_stack((u, v)).astype(np.int32))


def _same_serialized_calibration(left, right):
    if isinstance(left, dict) and isinstance(right, dict):
        return left.keys() == right.keys() and all(
            _same_serialized_calibration(left[key], right[key]) for key in left)
    if isinstance(left, list) and isinstance(right, list):
        return len(left) == len(right) and all(
            _same_serialized_calibration(a, b) for a, b in zip(left, right))
    if isinstance(left, (float, int)) and isinstance(right, (float, int)):
        return np.isfinite([left, right]).all() and abs(left-right) <= 1e-10
    return left == right


def validate_pair(observation, record):
    # A server record lacks the RPC envelope/code, but must identify the same frames.
    keys = ('sequence', 'camera_serial', 'camera_frame', 'color_frame_number', 'depth_frame_number')
    for key in keys:
        if key not in observation or key not in record or observation[key] != record[key]:
            raise ValueError(f'Server images and DDS observation mismatch: {key}')
    # C++ RPC and Python record serializers may round the last double digit.
    # Never use a relative tolerance on epoch-sized millisecond timestamps.
    for key in ('color_timestamp_ms', 'depth_timestamp_ms'):
        a, b = float(observation.get(key, float('nan'))), float(record.get(key, float('nan')))
        if not np.isfinite([a, b]).all() or abs(a-b) > .001:
            raise ValueError(f'Server images and DDS observation mismatch: {key}')
    for key in ('color_intrinsics', 'depth_scale_m_per_unit', 'depth_to_color_extrinsics'):
        if key not in observation or key not in record or not _same_serialized_calibration(observation[key], record[key]):
            raise ValueError(f'Server images and DDS observation mismatch: {key}')
    if record['camera_frame'] != EXPECTED_CAMERA_FRAME:
        raise ValueError('Only color-aligned RGB-D is supported')
    if not 0 <= float(record.get('timestamp_delta_ms', float('nan'))) <= 50:
        raise ValueError('RGB/depth timestamp difference exceeds 50 ms or is invalid')


def remote_copy(host, remote, destination):
    if not re.fullmatch(r'[a-zA-Z0-9_.-]+@[a-zA-Z0-9_.-]+', host):
        raise ValueError('Use an explicit SSH user@host')
    if not Path(remote).is_absolute() or '\n' in remote or '\x00' in remote:
        raise ValueError('Invalid server artifact path')
    # Batch mode never blocks on a password; use existing authorized SSH access.
    control_path = os.environ.get('G1_VISION_SSH_CONTROL_PATH')
    connection = ['-o', 'ControlPath='+control_path] if control_path else []
    with destination.open('xb') as stream:
        try:
            subprocess.run(['ssh', *connection, '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=5',
                            '-o', 'StrictHostKeyChecking=yes', host,
                            'cat -- ' + shlex.quote(remote)], stdout=stream,
                           check=True, timeout=45)
        except (subprocess.SubprocessError, OSError):
            destination.unlink(missing_ok=True)
            raise


def import_record(capture_path, *, host=None, server_record=None,
                  intrinsics=DEFAULT_INTRINSICS, stride=3):
    capture_path = Path(capture_path).resolve()
    with tempfile.TemporaryDirectory(prefix='.rgbd-import-', dir=capture_path.parent) as temporary:
        directory = Path(temporary)
        manifest = _import_record(capture_path, directory, host=host,
                                  server_record=server_record, intrinsics=intrinsics, stride=stride)
        created = []
        try:
            # Publish the completion manifest last; a failed transfer remains retryable.
            for source in sorted(directory.iterdir(), key=lambda path: path.name == 'manifest.json'):
                destination = capture_path.parent / source.name
                os.link(source, destination)
                created.append(destination)
        except OSError:
            for path in created:
                path.unlink()
            raise
        return manifest


def import_camera_record(capture_path, *, host=None, server_record=None,
                         intrinsics=DEFAULT_INTRINSICS):
    """Archive paired camera files without task targets, torso FK or a cloud.

    Used by B. Default task import_record behavior remains unchanged. A failed
    transfer never publishes a manifest, and a subsequent attempt can retry.
    """
    capture_path = Path(capture_path).resolve()
    capture = json.loads(capture_path.read_text())
    observation = validate_capture_result(capture, 'prepare')
    _profile_for_observation(intrinsics, observation)
    names = ('server_result.json', 'rgb.png', 'depth_aligned.png', 'intrinsics.json', 'manifest.json')
    if any((capture_path.parent/name).exists() for name in names):
        raise FileExistsError('Camera archive exists; use a fresh capture directory')
    with tempfile.TemporaryDirectory(prefix='.camera-import-', dir=capture_path.parent) as temporary:
        directory = Path(temporary)
        if server_record:
            shutil.copyfile(server_record, directory/'server_result.json')
        elif host:
            remote_copy(host, observation['capture_result'], directory/'server_result.json')
        else:
            raise ValueError('Specify an existing local server record or authorized SSH host')
        record = json.loads((directory/'server_result.json').read_text())
        validate_pair(observation, record)
        parent = Path(observation['capture_result']).parent
        saved = record['capture']
        for key, name in (('color_image', 'rgb.png'), ('depth_image', 'depth_aligned.png')):
            remote = Path(saved[key])
            if not remote.is_absolute() or remote.parent != parent or not remote.name.startswith(saved['id']+'_'):
                raise ValueError('Unexpected camera artifact path')
            if server_record:
                shutil.copyfile(Path(server_record).parent/remote.name, directory/name)
            else:
                remote_copy(host, str(remote), directory/name)
        color = cv2.imread(str(directory/'rgb.png'), cv2.IMREAD_COLOR)
        depth = cv2.imread(str(directory/'depth_aligned.png'), cv2.IMREAD_UNCHANGED)
        intr = observation['color_intrinsics']
        if (color is None or depth is None or depth.dtype != np.uint16 or depth.ndim != 2 or
                color.shape != (*depth.shape, 3) or depth.shape != (intr['height'], intr['width'])):
            raise ValueError('Paired RGB-D images have invalid encoding or dimensions')
        shutil.copyfile(intrinsics, directory/'intrinsics.json')
        files = {p.name: hashlib.sha256(p.read_bytes()).hexdigest()
                 for p in [capture_path, *(directory/name for name in names[:-1])]}
        manifest = dict(schema_version=1, frame_id=EXPECTED_CAMERA_FRAME, preview_only=True,
                        depth_encoding='uint16 original depth Z raster-aligned to RGB',
                        depth_scale_m_per_unit=observation['depth_scale_m_per_unit'],
                        source_sequence=observation['sequence'], source_request_id=capture['request_id'],
                        captured_at_ms=observation['captured_at_ms'],
                        timestamp_note='Server post-inference wall time is not exposure time',
                        sha256=files)
        (directory/'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
        created = []
        try:
            for name in names:
                destination = capture_path.parent/name
                os.link(directory/name, destination); created.append(destination)
        except OSError:
            for path in created:
                path.unlink()
            raise
    return manifest


def _import_record(capture_path, directory, *, host, server_record, intrinsics, stride):
    capture_path = Path(capture_path).resolve()
    capture = json.loads(capture_path.read_text())
    observation = capture.get('observation')
    if not isinstance(observation, dict) or not observation.get('capture_result'):
        raise ValueError('No saved server images in this DDS reply; capture with --save-artifacts')
    record_path = directory / 'server_result.json'
    destinations = [directory / name for name in (
        'server_result.json', 'rgb.png', 'depth_aligned.png', 'annotated.jpg',
        'intrinsics.json', 'cloud.npz', 'manifest.json')]
    if any((capture_path.parent / path.name).exists() for path in destinations):
        raise FileExistsError('Archive output exists; use a new directory or retain the partial archive for inspection')
    if server_record:
        shutil.copyfile(server_record, record_path)
    elif host:
        remote_copy(host, observation['capture_result'], record_path)
    else:
        raise ValueError('Specify --ssh user@host or --server-record LOCAL_RESULT_JSON')
    record = json.loads(record_path.read_text())
    validate_pair(observation, record)
    _profile_for_observation(intrinsics, observation)
    saved = record['capture']
    parent = Path(observation['capture_result']).parent
    for key, filename in (('color_image', 'rgb.png'), ('depth_image', 'depth_aligned.png'),
                          ('annotated_image', 'annotated.jpg')):
        remote = Path(saved[key])
        if remote.parent != parent or not remote.name.startswith(saved['id'] + '_'):
            raise ValueError('Unexpected server image location')
        if server_record:
            shutil.copyfile(Path(server_record).parent / remote.name, directory / filename)
        else:
            remote_copy(host, str(remote), directory / filename)
    depth = cv2.imread(str(directory / 'depth_aligned.png'), cv2.IMREAD_UNCHANGED)
    color = cv2.imread(str(directory / 'rgb.png'), cv2.IMREAD_COLOR)
    if depth is None or color is None:
        raise ValueError('Saved image could not be decoded')
    transform = torso_color_transform(observation['depth_to_color_extrinsics'])
    xyz, rgb, uv = aligned_cloud(depth, color, observation['color_intrinsics'],
                               observation['depth_scale_m_per_unit'], transform, stride)
    if not len(xyz):
        raise ValueError('No valid depth points in display range')
    shutil.copyfile(intrinsics, directory / 'intrinsics.json')
    np.savez_compressed(directory / 'cloud.npz', xyz=xyz, rgb=rgb, uv=uv)
    files = {path.name: hashlib.sha256(path.read_bytes()).hexdigest()
             for path in [capture_path, *destinations[:-1]]}
    manifest = dict(schema_version=1, preview_only=True, frame_id='torso_link',
                    camera_mount_calibrated=False, registration_to_cad_applied=False,
                    depth_encoding='uint16 Z16 aligned to RGB; not raw depth intrinsics',
                    depth_scale_m_per_unit=observation['depth_scale_m_per_unit'],
                    source_sequence=observation['sequence'],
                    source_request_id=capture.get('request_id'),
                    captured_at_ms=observation['captured_at_ms'],
                    timestamp_note='Server result wall time; not synchronized robot pose',
                    torso_from_color_optical_row_major=transform.tolist(),
                    point_count=len(xyz), stride=stride, display_depth_range_m=[.1, 2.],
                    xyz_bounds_m=[xyz.min(axis=0).tolist(), xyz.max(axis=0).tolist()],
                    sha256=files)
    (directory / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
    return manifest


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--capture', type=Path, help='Import an existing DDS capture instead of acquiring')
    parser.add_argument('--network', default='eth0')
    parser.add_argument('--output', type=Path)
    source = parser.add_mutually_exclusive_group()
    source.add_argument('--ssh', help='Authorized service host, e.g. unitree@192.168.123.164')
    source.add_argument('--server-record', type=Path, help='Transferred result JSON alongside original images')
    parser.add_argument('--intrinsics', type=Path, default=DEFAULT_INTRINSICS)
    parser.add_argument('--stride', type=int, default=3)
    args = parser.parse_args()
    capture = args.capture
    if capture is None:
        directory = args.output or ROOT / 'data/vision_captures' / datetime.now().strftime('%Y%m%d_%H%M%S_%f')
        directory.mkdir(parents=True, exist_ok=False)
        capture = directory / 'capture.json'
        result = subprocess.run([str(ROOT / 'scripts/run_task_vision_capture.sh'),
                                 args.network, str(capture), '--save-artifacts'], timeout=100)
        # Even a failed detector can have saved usable paired images.
        if not capture.exists():
            raise RuntimeError(f'Capture failed before producing metadata: {result.returncode}')
    try:
        manifest = import_record(capture, host=args.ssh, server_record=args.server_record,
                                 intrinsics=args.intrinsics, stride=args.stride)
    except (OSError, ValueError, KeyError, subprocess.SubprocessError) as error:
        parser.exit(2, f'RGB-D archive incomplete: {error}\nDDS response retained: {capture}\n')
    print(f'RGB-D archived: {capture.parent}; {manifest["point_count"]} measured points, preview only')


if __name__ == '__main__':
    main()
