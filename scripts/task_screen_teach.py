"""Screen endpoint records and workcell bundles. No ROS or hardware I/O."""
import copy
from datetime import datetime
import hashlib
import io
import json
import math
import os
from pathlib import Path
import tempfile
import zipfile

import numpy as np

from task_session import ARM_NAMES, ARMS

SCHEMA = 'g1_screen_teach_v1'
SLOTS = ('T', 'P1', 'P2', 'P3', 'P4', 'P5')
MOVING = ARMS['right'] + ('waist_yaw_joint',)
JOINT_MARGIN = .02
ALIGNMENT_YAW = 0.
ALIGNMENT_YAW_TOLERANCE = math.radians(.5)
DEFAULTS = dict(approach_mm=30., dry_clearance_mm=10., speed_mm_s=5., dwell_s=.15,
                velocity=.15, acceleration=.15, extra_feed_mm=0.)
BOUNDS = dict(approach_mm=(10., 80.), dry_clearance_mm=(5., 40.), speed_mm_s=(1., 10.),
              dwell_s=(0., .5), velocity=(.05, .3), acceleration=(.05, .3), extra_feed_mm=(0., 0.))


def now():
    return datetime.now().astimezone().isoformat()


def fingerprint(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, allow_nan=False).encode()).hexdigest()


def atomic(path, content):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(dir=path.parent, delete=False) as stream:
            temporary = Path(stream.name)
            stream.write(content)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)


def write_json(path, value):
    atomic(path, (json.dumps(value, indent=2, allow_nan=False, ensure_ascii=False)+'\n').encode())


def matrix(value):
    t = np.asarray(value, dtype=float)
    if (t.shape != (4, 4) or not np.isfinite(t).all() or
            not np.allclose(t[3], [0, 0, 0, 1]) or
            not np.allclose(t[:3, :3].T @ t[:3, :3], np.eye(3), atol=1e-6) or
            not np.isclose(np.linalg.det(t[:3, :3]), 1.)):
        raise ValueError('Invalid screen rigid transform')
    return t


def parameters(values):
    if set(values) != set(DEFAULTS):
        raise ValueError('Screen parameter schema mismatch')
    for key, (lo, hi) in BOUNDS.items():
        v = values[key]
        if type(v) not in (int, float) or not np.isfinite(v) or not lo <= v <= hi:
            raise ValueError('Screen parameter outside limits: '+key)
    if values['dry_clearance_mm'] >= values['approach_mm']:
        raise ValueError('Dry clearance must be smaller than approach distance')
    return dict(values)


def new_config(identity):
    return dict(schema=SCHEMA, frame='pelvis', arm='right', identity=identity,
                points={}, plane=None, parameters=dict(DEFAULTS), reference=None,
                contact_policy='position_only_zero_extra_feed', created_at=now())


def guarded_joints(model, values, names=MOVING, context='屏幕路径'):
    rows = np.atleast_2d(np.asarray(values, dtype=float))
    lower = np.array([model.limits[n][0]+JOINT_MARGIN for n in names])
    upper = np.array([model.limits[n][1]-JOINT_MARGIN for n in names])
    if rows.shape[1] != len(names) or not np.isfinite(rows).all():
        raise ValueError(context+'：关节数据缺失或非有限值')
    bad = np.argwhere((rows < lower-1e-9) | (rows > upper+1e-9))
    if len(bad):
        row, axis = bad[0]
        raise ValueError(f'{context}：{names[axis]}={math.degrees(rows[row, axis]):.3f}° 超出可用范围 '
                         f'[{math.degrees(lower[axis]):.3f}°, {math.degrees(upper[axis]):.3f}°]'
                         f'（已预留 {math.degrees(JOINT_MARGIN):.3f}° 边界余量）')


def effective_tcp(model, joints, calibration):
    wrist = model.transform('right_wrist_yaw_link', joints)
    tcp = np.linalg.inv(wrist) @ model.transform('right_index_tcp', joints)
    if calibration['a_enabled']:
        tcp[:3, 3] = [calibration[f'right_{a}_mm']/1000. for a in 'xyz']
    return matrix(tcp)


def endpoint(model, joints, hands, tcp, stamp, epoch):
    q = {k: float(v) for k, v in joints.items()}
    if not all(k in q and np.isfinite(q[k]) for k in ARM_NAMES+('waist_yaw_joint',)):
        raise ValueError('Measured arms and waist required')
    return dict(joints=q, hands=copy.deepcopy(hands), wrist_from_tcp=matrix(tcp).tolist(),
                pelvis_from_tcp=(model.transform('right_wrist_yaw_link', q) @ tcp).tolist(),
                captured_at=now(), source_stamp_ns=stamp, stop_epoch=epoch,
                source='stationary_joint_feedback_and_fresh_hand_DDS', physical_contact='operator_confirmed')


def fit_plane(points):
    if any(k not in points for k in SLOTS[1:]):
        raise ValueError('Record all five screen points first')
    xyz = np.array([matrix(points[k]['pelvis_from_tcp'])[:3, 3] for k in SLOTS[1:]])
    center = xyz.mean(axis=0)
    _, singular, axes = np.linalg.svd(xyz-center)
    if singular[1] < .005 or singular[1] < .02*singular[0]:
        raise ValueError('Screen points are coincident/near-collinear; spread the points over the screen')
    normal = axes[-1]
    # Initial sign is only a suggestion; the operator must inspect the arrow.
    if 'T' in points and np.dot(matrix(points['T']['pelvis_from_tcp'])[:3, 3]-center, normal) < 0:
        normal = -normal
    errors = (xyz-center) @ normal
    return dict(center_m=center.tolist(), outward_normal=normal.tolist(),
                residual_mm=(errors*1000).tolist(), rms_mm=float(np.sqrt(np.mean(errors**2))*1000),
                max_mm=float(np.max(np.abs(errors))*1000), confirmed=False,
                points_sha256=fingerprint(points), source='taught_contact_FK_not_physical_plane_measurement')


def cloud_array(xyz):
    xyz = np.asarray(xyz, dtype=np.float32)
    if (xyz.ndim != 2 or xyz.shape[1] != 3 or not 20 <= len(xyz) <= 1280*720 or
            not np.isfinite(xyz).all() or np.max(np.abs(xyz)) > 10):
        raise ValueError('Invalid/empty workcell cloud')
    return xyz


def validate(config):
    if (config.get('schema') != SCHEMA or config.get('frame') != 'pelvis' or
            config.get('arm') != 'right' or config.get('contact_policy') != 'position_only_zero_extra_feed'):
        raise ValueError('Not a supported screen endpoint bundle')
    parameters(config['parameters'])
    if not isinstance(config['identity'], dict) or not isinstance(config['points'], dict):
        raise ValueError('Missing screen configuration identity')
    for slot, record in config['points'].items():
        if slot not in SLOTS:
            raise ValueError('Unknown endpoint: '+slot)
        matrix(record['wrist_from_tcp']); matrix(record['pelvis_from_tcp'])
        q = record['joints']
        if not all(k in q and np.isfinite(q[k]) for k in ARM_NAMES+('waist_yaw_joint',)):
            raise ValueError('Incomplete endpoint joints')
        if not all(type(v) in (int, float) and np.isfinite(v) for v in q.values()):
            raise ValueError('Nonfinite endpoint joint')
        from task_hand_control import require_state
        for side in ARMS:
            require_state(record['hands'][side], side)
    plane = config.get('plane')
    if plane is not None:
        fitted = fit_plane(config['points'])
        normal = np.asarray(plane['outward_normal'], float)
        if (normal.shape != (3,) or not np.isfinite(normal).all() or
                not np.isclose(np.linalg.norm(normal), 1.) or
                abs(np.dot(normal, fitted['outward_normal'])) < .999999 or
                plane['points_sha256'] != fitted['points_sha256'] or
                not np.allclose(plane['center_m'], fitted['center_m'], atol=1e-9) or
                not np.isclose(plane['max_mm'], fitted['max_mm'])):
            raise ValueError('Plane no longer matches recorded points')
        if type(plane.get('confirmed')) is not bool:
            raise ValueError('Invalid plane confirmation')
    return config


def save_bundle(path, config, reference):
    saved = copy.deepcopy(validate(config))
    if reference is not None:
        data = io.BytesIO()
        np.savez_compressed(data, xyz=cloud_array(reference))
        raw = data.getvalue()
        sha = hashlib.sha256(raw).hexdigest()
        name = 'reference_'+sha[:16]+'.npz'
        atomic(Path(path).parent/name, raw)
        saved['reference'] = dict(saved['reference'], file=name, sha256=sha)
    elif saved['reference'] is not None:
        raise ValueError('Reference metadata has no point cloud')
    write_json(path, saved)


def load_bundle(path):
    path = Path(path)
    if path.stat().st_size > 2_000_000:
        raise ValueError('Screen configuration is too large')
    config = validate(json.loads(path.read_text()))
    reference = None
    if config['reference'] is not None:
        info = config['reference']; name = info['file']
        if Path(name).name != name or not name.startswith('reference_') or not name.endswith('.npz'):
            raise ValueError('Invalid reference cloud path')
        cloud_path = path.parent/name
        if cloud_path.stat().st_size > 20_000_000:
            raise ValueError('Reference cloud is too large')
        raw = cloud_path.read_bytes()
        if hashlib.sha256(raw).hexdigest() != info['sha256']:
            raise ValueError('Reference cloud checksum mismatch')
        with zipfile.ZipFile(io.BytesIO(raw)) as archive:
            entries = archive.infolist()
            if len(entries) != 1 or entries[0].filename != 'xyz.npy' or entries[0].file_size > 20_000_000:
                raise ValueError('Invalid reference cloud archive dimensions')
        with np.load(io.BytesIO(raw), allow_pickle=False) as data:
            reference = cloud_array(data['xyz']).copy()
    return config, reference
