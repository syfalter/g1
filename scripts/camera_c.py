"""Experiment C archives and torso-camera / wrist-board consistency geometry.

No ROS or hardware I/O. New-stream owner evidence plus a stationary receipt
window permits diagnostic association, never a hardware-sync claim.
"""
from __future__ import annotations

import argparse
import copy
from datetime import datetime
from itertools import combinations
from pathlib import Path
import shutil
import uuid

import cv2
import numpy as np
from scipy.spatial.transform import Rotation

from camera_b_node import ROOT, digest, local_file, read_json, write_json, validate_bundle
from camera_board import BOARD, LIMITS, FRAME, detect_board, rgb_pose, depth_metrics
from vision_task_geometry import torso_color_transform, DEFAULT_VENDOR_URDF
from workbench_model import OfflineModel

SCHEMA = 'g1_camera_c_session_v1'
PROTOCOL = 'camera_c_v1'
PAIR_LIMITS = dict(margin_s=.15, max_gap_s=.05, max_clock_step_s=.02,
                   max_translation_mm=1., max_rotation_deg=.3,
                   max_duration_s=75., max_owner_age_s=1.5)
SOLVE_LIMITS = dict(min_fit=6, min_second_axis_deg=5., min_translation_span_mm=20.,
                    max_translation_condition=100.)
BOUNDARY_GAP = '缺少原相机服务的新流取帧证据；旧快照不自动追认为停稳采样。'


def fresh_stream_evidence(capture, record):
    evidence = record.get('frame_timing', {})
    if (evidence.get('schema') != 'vision_frame_timing_v1' or
            evidence.get('capture_contract') != 'owner_stream_restart_static_v1'):
        raise ValueError('相机服务未提供新流取帧协议；请先更新并正常重启 vision_service')
    if (type(capture.get('request_id')) is not int or capture['request_id'] <= 0 or
            evidence.get('request_id') != capture['request_id']):
        raise ValueError('New-stream evidence does not match this RPC request')
    for key in ('worker_session', 'stream_session'):
        value = evidence.get(key)
        if not isinstance(value, str) or len(value) != 32 or any(c not in '0123456789abcdef' for c in value):
            raise ValueError('Missing owner/stream session identity')
    keys = ('request_entered_monotonic_ns', 'stream_stop_started_monotonic_ns',
            'stream_stopped_monotonic_ns', 'stream_started_monotonic_ns',
            'wait_started_monotonic_ns', 'frames_received_monotonic_ns', 'observation_ready_monotonic_ns')
    values = [evidence.get(k) for k in keys]
    if (any(type(v) is not int or v <= 0 for v in values) or
            any(b <= a for a, b in zip(values, values[1:])) or
            (values[-1]-values[0])/1e9 > PAIR_LIMITS['max_duration_s']):
        raise ValueError('Invalid owner stream restart event order/duration')
    if type(evidence.get('warmup_frames')) is not int or evidence['warmup_frames'] < 15:
        raise ValueError('New stream warmup incomplete')
    for kind in ('color', 'depth'):
        frame = evidence.get(kind, {})
        if (frame.get('frame_number') != record.get(kind+'_frame_number') or
                frame.get('timestamp_ms') != record.get(kind+'_timestamp_ms')):
            raise ValueError('New-stream evidence refers to another image')
    return evidence


def transform(value):
    t = np.asarray(value, dtype=float)
    if (t.shape != (4, 4) or not np.isfinite(t).all() or
            not np.allclose(t[3], [0, 0, 0, 1], atol=1e-9) or
            not np.allclose(t[:3, :3].T @ t[:3, :3], np.eye(3), atol=1e-6) or
            abs(np.linalg.det(t[:3, :3])-1) > 1e-6):
        raise ValueError('Invalid SE(3) row-major matrix')
    return t


def mean_transform(values):
    values = np.array([transform(v) for v in values])
    if not len(values):
        raise ValueError('No transforms to average')
    t = np.eye(4)
    t[:3, :3] = Rotation.from_matrix(values[:, :3, :3]).mean().as_matrix()
    t[:3, 3] = values[:, :3, 3].mean(axis=0)
    return t


def chain_joints(model, link):
    if link not in model.joints:
        raise ValueError('Model missing link: '+link)
    names = set()
    while link in model.joints:
        parent, kind, _, _, _, name, _, _ = model.joints[link]
        if kind != 'fixed':
            names.add(name)
        link = parent
    if link != 'pelvis':
        raise ValueError('Unexpected model root: '+link)
    return names


def actual_fk(model, side, rows):
    if side not in ('left', 'right') or not rows:
        raise ValueError('Missing arm or actual window')
    wrist = side+'_wrist_yaw_link'
    names = sorted(chain_joints(model, wrist) | chain_joints(model, 'torso_link'))
    if any(not set(names) <= r['q'].keys() for r in rows):
        raise ValueError('Missing measured joint; zero fill is forbidden')
    positions = [[r['q'][n] for n in names] for r in rows]
    frames = model.batch_transforms(['torso_link', wrist], {}, names, positions)
    return np.linalg.inv(frames['torso_link']) @ frames[wrist]


def static_window(rows, bounds, model, side, limits=PAIR_LIMITS):
    """Check complete local receipt interval, not just its two endpoint poses.

    Bounds contain RPC monotonic/realtime pairs. This is a necessary static
    check only: receipt timestamps do not bound exposure or DDS transport age.
    """
    start, end = [float(bounds[k]) for k in ('rpc_start_monotonic', 'rpc_end_monotonic')]
    wall_start, wall_end = [float(bounds[k]) for k in ('rpc_start_realtime', 'rpc_end_realtime')]
    if not np.isfinite([start, end, wall_start, wall_end]).all() or not 0 < end-start <= limits['max_duration_s']:
        raise ValueError('Invalid RPC monotonic interval')
    monotonic_only = bounds.get('clock_basis') == 'local_monotonic_receipt_v2'
    clock_delta = (wall_end-wall_start)-(end-start)
    if not monotonic_only and abs(clock_delta) > limits['max_clock_step_s']:
        raise ValueError('Realtime clock changed during RPC')
    times = np.array([r['received'] for r in rows], float)
    if len(times) < 3 or not np.isfinite(times).all() or np.any(np.diff(times) <= 0):
        raise ValueError('Missing or nonmonotonic actual stream')
    lo, hi = start-limits['margin_s'], end+limits['margin_s']
    if times[0] > lo or times[-1] < hi:
        raise ValueError('Actual does not cover entire RPC interval and margins')
    first = max(0, int(np.searchsorted(times, lo, side='right'))-1)
    last = min(len(rows), int(np.searchsorted(times, hi, side='left'))+1)
    rows = rows[first:last]
    times = times[first:last]
    if any(r.get('actual_valid') is False for r in rows):
        raise ValueError('Invalid motor feedback inside actual interval')
    for key in ('stream_epoch', 'context_epoch'):
        if any(key in r for r in rows):
            if any(r.get(key) != rows[0].get(key) for r in rows):
                raise ValueError('Actual stream/context epoch changed during interval: '+key)
    gap = float(np.max(np.diff(times)))
    if gap > limits['max_gap_s']:
        raise ValueError('Actual stream gap exceeds limit')
    ticks = [r['tick'] for r in rows]
    if any(type(t) is not int or not 0 <= t < 2**32 for t in ticks) or any(b <= a for a, b in zip(ticks, ticks[1:])):
        raise ValueError('Actual tick duplicate, rollback or wrap: start a new capture')
    required = ('publisher_gid', 'model_sha256', 'owner_token', 'console_pid',
                'generation', 'stop_epoch', 'gateway_phase', 'mode_machine', 'mode_pr')
    for key in required:
        if any(key not in r or r[key] is None or r[key] == '' for r in rows):
            raise ValueError('Missing actual provenance: '+key)
        if any(r[key] != rows[0][key] for r in rows):
            raise ValueError('Actual provenance changed during interval: '+key)
    if rows[0]['gateway_phase'] not in ('ARMED_HOLD', 'TASK_ADMITTANCE'):
        raise ValueError('Original owner must remain in hold or stationary admittance')
    if rows[0]['gateway_phase'] == 'TASK_ADMITTANCE' and bounds.get('stationary_confirmed') is not True:
        raise ValueError('Admittance sampling requires explicit per-capture stationary confirmation')
    if monotonic_only:
        readers = {r.get('reader_pid') for r in rows}
        if len(readers) != 1 or None in readers:
            raise ValueError('C receiver changed during capture')
    for r in rows:
        if (r.get('source') != 'lowstate_actual' or r.get('publisher_count') != 1 or
                r.get('owner_publisher_count') != 1 or r.get('gateway_fresh') is not True):
            raise ValueError('Actual / original owner source missing, ambiguous or stale')
        age = float(r.get('owner_age_s', float('nan')))
        if not np.isfinite(age) or not 0 <= age <= limits['max_owner_age_s']:
            raise ValueError('Original owner status is stale')
        if 'discovered_actual_gids' in r:
            discovery_age = r.get('discovery_age_s')
            if (type(discovery_age) not in (int, float) or not np.isfinite(discovery_age) or
                    not 0 <= discovery_age <= .5 or
                    r.get('discovered_actual_gids') != [r['publisher_gid']] or
                    r.get('discovered_owner_gids') != [r.get('owner_publisher_gid')]):
                raise ValueError('Receipt source does not match fresh discovery observations')
            owner = r.get('owner_status', {})
            controls = owner.get('tcp_a_controls', {})
            stamp = controls.get('stamp')
            owner_wall = r.get('owner_received_realtime') if monotonic_only else r.get('realtime')
            if (controls.get('hardware') is not True or owner.get('shutdown_requested') is not False or
                    type(stamp) not in (int, float) or not np.isfinite(stamp) or
                    type(owner_wall) not in (int, float) or not np.isfinite(owner_wall) or
                    abs(owner_wall-stamp) > limits['max_owner_age_s']):
                raise ValueError('Original owner hardware/shutdown state or wall timestamp invalid')
        if monotonic_only:
            delay = r.get('pipe_delay_s')
            if (r.get('clock_basis') != 'local_monotonic_receipt_v2' or
                    type(delay) not in (int, float) or not np.isfinite(delay) or
                    not 0 <= delay <= limits['max_gap_s']):
                raise ValueError('C local receipt pipe delayed or clock contract missing')
        else:
            offset = float(r.get('realtime', float('nan')))-r['received']
            if not np.isfinite(offset) or abs(offset-(wall_start-start)) > limits['max_clock_step_s']:
                raise ValueError('Actual receipt realtime/monotonic mapping changed')
    gs = actual_fk(model, side, rows)
    translation = float(2*np.max(np.linalg.norm(gs[:, :3, 3]-gs[0, :3, 3], axis=-1))*1000)
    # Reference-radius bound avoids quadratic SO(3) allocation for long RPCs.
    angles = Rotation.from_matrix(gs[0, :3, :3].T @ gs[:, :3, :3]).magnitude()
    rotation_bound = float(2*np.degrees(np.max(angles)))
    if translation > limits['max_translation_mm'] or rotation_bound > limits['max_rotation_deg']:
        raise ValueError('Wrist relative to torso moved during full actual interval')
    return dict(static_valid=True, G_T_W_row_major=mean_transform(gs).tolist(),
                actual_count=len(rows), covered_monotonic=[float(times[0]), float(times[-1])],
                max_gap_s=gap, translation_diameter_upper_mm=translation,
                rotation_diameter_upper_deg=rotation_bound,
                clock_basis=bounds.get('clock_basis', 'legacy_wall_clock_checked'),
                realtime_offset_change_ms=clock_delta*1000,
                realtime_changed=abs(clock_delta)>limits['max_clock_step_s'],
                stationary_confirmed=bounds.get('stationary_confirmed') is True,
                gateway_phase=rows[0]['gateway_phase'],
                unobserved_motion_bound=None,
                limitation='Sampled encoder stability only; no bound on intersample motion, mechanical compliance or DDS transport latency')


def pairing(rows, bounds, model, side, limits=PAIR_LIMITS, *, capture=None, record=None, rigid=False):
    result = dict(pairing_valid=False, static_valid=False, reasons=[],
                  limits=copy.deepcopy(limits), hardware_sync=False)
    try:
        result.update(static_window(rows, bounds, model, side, limits))
        if bounds.get('stationary_confirmed') is not True or rigid is not True:
            raise ValueError('需要本次采样的停稳与刚性安装确认；preview 不能直接用于求解')
        if bounds.get('clock_basis') != 'local_monotonic_receipt_v2':
            raise ValueError('New-stream sampling requires native monotonic actual receipts')
        evidence = fresh_stream_evidence(capture or {}, record or {})
        result.update(pairing_valid=True, method='owner_stream_restart_static_v1',
                      stream_session=evidence['stream_session'],
                      limitation='Stationary software association, not hardware synchronization; DDS transport age and mechanical compliance are not certified')
    except (ValueError, KeyError, TypeError, IndexError) as error:
        result['reasons'].append(str(error))
    return result


def excitation(gs, limits=SOLVE_LIMITS):
    vectors, blocks = [], []
    for a, b in combinations(gs, 2):
        r = a[:3, :3].T @ b[:3, :3]
        vectors.append(Rotation.from_matrix(r).as_rotvec())
        blocks.append(r-np.eye(3))
    spectrum = np.linalg.svd(np.asarray(vectors), compute_uv=False)/np.sqrt(len(vectors))
    singular = np.linalg.svd(np.concatenate(blocks), compute_uv=False)
    condition = float(singular[0]/singular[-1]) if singular[-1] > 1e-12 else None
    span = float(max(np.linalg.norm(a[:3, 3]-b[:3, 3]) for a, b in combinations(gs, 2))*1000)
    report = dict(rotation_axis_rms_deg=np.degrees(spectrum).tolist(),
                  translation_span_mm=span, translation_condition=condition,
                  limits=copy.deepcopy(limits))
    if (np.degrees(spectrum[1]) < limits['min_second_axis_deg'] or
            span < limits['min_translation_span_mm'] or condition is None or
            condition > limits['max_translation_condition']):
        raise ValueError('Degenerate / insufficient multi-axis arm excitation: '+str(report))
    return report


def solve_transforms(gs, ps, limits=SOLVE_LIMITS):
    """Pure geometry, not an admission gate. G^-1 X P = Y.

    OpenCV calibrateHandEye receives gripper2base=G^-1, target2cam=P;
    its cam2gripper result is X, not Y. PARK is fixed, never picked on holdout.
    """
    gs, ps = [transform(g) for g in gs], [transform(p) for p in ps]
    if len(gs) != len(ps) or len(gs) < limits['min_fit']:
        raise ValueError('At least six paired fit poses required')
    coverage = excitation(gs, limits)
    inverse = [np.linalg.inv(g) for g in gs]
    r, t = cv2.calibrateHandEye([g[:3, :3] for g in inverse], [g[:3, 3] for g in inverse],
                               [p[:3, :3] for p in ps], [p[:3, 3] for p in ps],
                               method=cv2.CALIB_HAND_EYE_PARK)
    x = np.eye(4); x[:3, :3] = r; x[:3, 3] = t.reshape(3)
    x = transform(x)
    y = mean_transform([np.linalg.inv(g) @ x @ p for g, p in zip(gs, ps)])
    return x, y, coverage


def residuals(gs, ps, x, y):
    values = []
    for g, p in zip(gs, ps):
        a, b = x @ transform(p), transform(g) @ y
        values.append(dict(translation_mm=float(np.linalg.norm(a[:3, 3]-b[:3, 3])*1000),
                           rotation_deg=float(np.degrees(Rotation.from_matrix(a[:3, :3].T @ b[:3, :3]).magnitude()))))
    return dict(count=len(values), per_sample=values,
                **{key+'_rms': float(np.sqrt(np.mean([v[key]**2 for v in values]))) if values else None
                   for key in ('translation_mm', 'rotation_deg')})


class CameraCSession:
    def __init__(self, root=None, side='left', mount_id='unconfirmed'):
        if side not in ('left', 'right') or not mount_id.strip():
            raise ValueError('Arm and nonempty installation ID required')
        self.root = Path(root or ROOT/'data/camera_c/sessions'/uuid.uuid4().hex)
        self.root.mkdir(parents=True, exist_ok=False)
        self.state = dict(schema=SCHEMA, protocol=PROTOCOL, session_id=self.root.name,
                          created_at=datetime.now().isoformat(), side=side, mount_id=mount_id,
                          board=copy.deepcopy(BOARD), rgb_limits=copy.deepcopy(LIMITS),
                          pairing_limits=copy.deepcopy(PAIR_LIMITS), solve_limits=copy.deepcopy(SOLVE_LIMITS),
                          layout=None, camera_identity=None, model=None, nominal_model=None,
                          samples=[], audit=[], candidate=None, applied=False, hardware_validated=False,
                          frames=dict(T='torso_link', C=FRAME, W=side+'_wrist_yaw_link', Q='board'))
        self.last_snapshot = ''

    def event(self, action, **fields):
        self.state['audit'].append(dict(at=datetime.now().isoformat(), action=action, **fields))

    def verify(self, sample):
        for name, sha in sample['sha256'].items():
            if digest(local_file(self.root, name)) != sha:
                raise ValueError('Source SHA mismatch: '+name)
        if sample.get('receipt_window'):
            for name in ('actual.json', 'bounds.json', 'context.json', 'manifest.json'):
                if sample['receipt_window']+'/'+name not in sample['sha256']:
                    raise ValueError('Receipt reference is not covered by sample SHA')
            if sample.get('actual') != sample['receipt_window']+'/actual.json':
                raise ValueError('Actual reference differs from receipt archive')

    def import_bundle(self, source, group='preview', note='', source_audit=None):
        if group not in ('fit', 'validation', 'preview'):
            raise ValueError('Unknown sample group')
        source = Path(source).resolve()
        manifest = read_json(source/'manifest.json')
        names = ['rgb.png', 'depth_aligned.png', 'capture.json', 'server_result.json']
        for name in names:
            if digest(source/name) != manifest.get('sha256', {}).get(name):
                raise ValueError('Source manifest mismatch: '+name)
        validate_bundle(source)
        identity = digest(source/'rgb.png')
        if any(s['source_identity'] == identity for s in self.state['samples']):
            raise ValueError('Duplicate RGB frame')
        sid = f"c_{len(self.state['samples'])+1:04d}_{uuid.uuid4().hex[:8]}"
        folder = self.root/'samples'/sid; folder.mkdir(parents=True)
        for name in names+['manifest.json']:
            shutil.copyfile(source/name, folder/name)
        sample = dict(id=sid, group=group, note=note, excluded=False, exclusion_reason='',
                      directory=folder.relative_to(self.root).as_posix(),
                      source=str(source), source_identity=identity,
                      source_audit=copy.deepcopy(source_audit), source_kind='unpaired_archive',
                      sha256={p.relative_to(self.root).as_posix(): digest(p) for p in folder.iterdir()},
                      actual=None, bounds=None, rigid_mount_confirmed=False)
        self.state['samples'].append(sample)
        self.analyze(sample)
        self.state['candidate'] = None
        self.event('import', sample_id=sid, group=group)
        self.save()
        return sample

    def analyze(self, sample):
        self.verify(sample)
        folder = self.root/sample['directory']
        image = cv2.imread(str(local_file(self.root, sample['directory']+'/rgb.png')))
        result = dict(rgb_valid=False, rgb_reason='', depth_diagnostic=None,
                      pairing_valid=False, pairing=dict(reasons=[BOUNDARY_GAP, 'No paired actual archive']),
                      solve_usable=False, pose=None)
        overlay = image.copy() if image is not None else np.zeros((360, 640, 3), np.uint8)
        try:
            obs = validate_bundle(folder)
            identity = {k: obs[k] for k in ('camera_serial', 'camera_frame', 'color_intrinsics', 'depth_to_color_extrinsics')}
            if obs['camera_frame'] != FRAME:
                raise ValueError('Unexpected RGB optical frame')
            if self.state['camera_identity'] not in (None, identity):
                raise ValueError('Camera/profile changed; new session required')
            self.state['camera_identity'] = identity
            detection, overlay = detect_board(image, self.state['layout'], self.state['board'], self.state['rgb_limits'])
            result['detection'] = detection
            if not detection['valid']:
                raise ValueError(detection['reason'])
            self.state['layout'] = detection['layout']
            pose = rgb_pose(detection, obs['color_intrinsics'], image.shape, self.state['rgb_limits'])
            result.update(rgb_valid=True, pose=pose)
            try:
                metrics, _ = depth_metrics(cv2.imread(str(folder/'depth_aligned.png'), cv2.IMREAD_UNCHANGED),
                    obs['color_intrinsics'], obs['depth_scale_m_per_unit'], obs['depth_to_color_extrinsics'],
                    detection, pose, self.state['rgb_limits'], self.state['board'])
                result['depth_diagnostic'] = dict(computable=True, metrics=metrics, used_for_rgb_or_fit=False)
            except (ValueError, cv2.error, TypeError, AttributeError) as error:
                result['depth_diagnostic'] = dict(computable=False, reason=str(error), used_for_rgb_or_fit=False)
        except (ValueError, KeyError, cv2.error, TypeError) as error:
            result['rgb_reason'] = str(error)
        if sample.get('actual'):
            try:
                if sample.get('actual_association_error'):
                    raise ValueError(sample['actual_association_error'])
                model = self.state['model']
                if not model:
                    raise ValueError('No received model available for archived actual')
                path = local_file(self.root, model['file'])
                if digest(path) != model['sha256']:
                    raise ValueError('Model SHA mismatch')
                rows = read_json(local_file(self.root, sample['actual']))
                if any(r.get('model_sha256') != model['sha256'] for r in rows):
                    raise ValueError('Actual model SHA differs from archived model')
                bounds = (read_json(local_file(self.root, sample['receipt_window']+'/bounds.json'))
                          if sample.get('receipt_window') else sample['bounds'])
                result['pairing'] = pairing(rows, bounds, OfflineModel(path), self.state['side'], self.state['pairing_limits'],
                    capture=read_json(folder/'capture.json'), record=read_json(folder/'server_result.json'),
                    rigid=sample.get('rigid_mount_confirmed') is True)
            except (ValueError, KeyError, TypeError) as error:
                result['pairing'] = dict(pairing_valid=False, reasons=[BOUNDARY_GAP, str(error)])
        result['pairing_valid'] = result['pairing'].get('pairing_valid') is True
        result['solve_usable'] = result['rgb_valid'] and result['pairing_valid'] and sample['group'] in ('fit', 'validation')
        output = folder/('analysis_'+uuid.uuid4().hex[:10]); output.mkdir()
        if not cv2.imwrite(str(output/'overlay.png'), overlay):
            raise OSError('Cannot save RGB overlay')
        write_json(output/'result.json', result)
        sample.update(result=result, views=dict(rgb=sample['directory']+'/rgb.png',
                      overlay=(output/'overlay.png').relative_to(self.root).as_posix()))
        return result

    def attach_receipt_window(self, sample_id, source):
        """Import raw RPC receipt evidence even when incomplete or inconsistent."""
        sample = next(s for s in self.state['samples'] if s['id'] == sample_id)
        if sample.get('actual'):
            raise ValueError('Actual evidence already attached')
        source = Path(source).resolve()
        manifest = read_json(source/'manifest.json')
        if manifest.get('schema') != 'camera_c_receipt_window_v1':
            raise ValueError('Unknown receipt archive schema')
        files = manifest.get('sha256', {})
        if not isinstance(files, dict) or not {'actual.json', 'bounds.json', 'context.json'} <= files.keys():
            raise ValueError('Receipt archive missing required evidence')
        for name, sha in files.items():
            if Path(name).is_absolute() or '..' in Path(name).parts:
                raise ValueError('Receipt manifest requires local relative paths')
            if digest(local_file(source, name)) != sha:
                raise ValueError('Receipt evidence SHA mismatch: '+name)
        folder = self.root/sample['directory']/'receipt_window'
        folder.mkdir()
        for name in [*files, 'manifest.json']:
            target = folder/name; target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(local_file(source, name), target)
            sample['sha256'][target.relative_to(self.root).as_posix()] = digest(target)
        sample['receipt_window'] = folder.relative_to(self.root).as_posix()
        sample['actual'] = sample['receipt_window']+'/actual.json'
        sample['bounds'] = read_json(folder/'bounds.json')
        rows = read_json(folder/'actual.json')
        if not isinstance(rows, list) or any(not isinstance(r, dict) for r in rows):
            sample['actual_association_error'] = 'Invalid raw actual JSON array; evidence retained'
            self.analyze(sample); self.event('attach_receipt_window_invalid', sample_id=sample_id)
            return self.save()
        hashes = {r.get('model_sha256') for r in rows}
        error = ''
        if len(hashes) != 1 or None in hashes:
            error = 'Missing or changing model within raw actual evidence'
        else:
            sha = next(iter(hashes)); name = 'model_'+sha+'.urdf'
            if files.get(name) != sha:
                error = 'Received model not archived with matching SHA'
            elif self.state['model'] and self.state['model']['sha256'] != sha:
                error = 'Model differs from session; raw evidence retained, new session required'
            else:
                self.state['model'] = dict(file=sample['receipt_window']+'/'+name, sha256=sha,
                                          source='/robot_description; local received text')
        sample['actual_association_error'] = error
        self.state['candidate'] = None
        self.analyze(sample)
        self.event('attach_receipt_window', sample_id=sample_id, actual_count=len(rows),
                   pairing_valid=sample['result']['pairing_valid'], association_error=error)
        return self.save()

    def attach_actual(self, sample_id, actual_path, model_path, bounds, rigid_mount_confirmed=False):
        """Import existing evidence, never synthesize missing timing or joints."""
        sample = next(s for s in self.state['samples'] if s['id'] == sample_id)
        if sample.get('actual'):
            raise ValueError('Actual evidence already attached; cannot overwrite provenance')
        rows = read_json(actual_path)
        if not isinstance(rows, list) or not rows or len(rows) > 20000:
            raise ValueError('Expected nonempty actual JSON array, at most 20000 records')
        sha = digest(model_path)
        model = OfflineModel(model_path)
        actual_fk(model, self.state['side'], rows)
        if any(r.get('model_sha256') != sha for r in rows):
            raise ValueError('Actual model SHA differs from provided model')
        if self.state['model'] and self.state['model']['sha256'] != sha:
            raise ValueError('Model changed; new session required')
        target = self.root/('model_'+sha+'.urdf')
        if not target.exists():
            shutil.copyfile(model_path, target)
        if digest(target) != sha:
            raise ValueError('Archived model SHA mismatch')
        self.state['model'] = dict(file=target.name, sha256=sha, source=str(Path(model_path).resolve()))
        actual = self.root/sample['directory']/'actual.json'
        shutil.copyfile(actual_path, actual)
        sample['actual'] = actual.relative_to(self.root).as_posix()
        sample['sha256'][sample['actual']] = digest(actual)
        sample['bounds'] = copy.deepcopy(bounds)
        sample['rigid_mount_confirmed'] = rigid_mount_confirmed is True
        self.analyze(sample)
        self.state['candidate'] = None
        self.event('attach_actual', sample_id=sample_id, pairing_valid=sample['result']['pairing_valid'])
        return self.save()

    def save(self):
        path = self.root/('session_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f')+'.json')
        write_json(path, self.state); self.last_snapshot = str(path)
        return path

    @classmethod
    def load(cls, path):
        path = Path(path).resolve(); state = read_json(path)
        if (state.get('schema') != SCHEMA or state.get('protocol') != PROTOCOL or
                state.get('applied') is not False or state.get('board') != BOARD or
                state.get('rgb_limits') != LIMITS or state.get('pairing_limits') != PAIR_LIMITS or
                state.get('solve_limits') != SOLVE_LIMITS):
            raise ValueError('Not a compatible unapplied C archive')
        if state.get('side') not in ('left', 'right') or not state.get('mount_id'):
            raise ValueError('Missing installation/arm')
        obj = object.__new__(cls); obj.root = path.parent; obj.state = state; obj.last_snapshot = str(path)
        ids = set()
        for sample in state['samples']:
            if sample['id'] in ids or sample['group'] not in ('fit', 'validation', 'preview'):
                raise ValueError('Duplicate ID / invalid group')
            ids.add(sample['id']); obj.verify(sample)
            local_file(obj.root, sample['directory']+'/rgb.png')
            if sample.get('actual') and sample['actual'] not in sample['sha256']:
                raise ValueError('Actual archive missing SHA')
        for key in ('model', 'nominal_model'):
            if state.get(key) and digest(local_file(obj.root, state[key]['file'])) != state[key]['sha256']:
                raise ValueError('Archived model SHA mismatch')
        # Saved result flags and candidates are not evidence; recompute admission.
        obj.state['candidate'] = None
        return obj

    def recompute(self):
        self.state['candidate'] = None
        for sample in self.state['samples']:
            self.analyze(sample)
        if self.state['camera_identity']:
            try:
                self.reference_camera()
                self.state['nominal_preview_error'] = ''
            except (ValueError, OSError) as error:
                self.state['nominal_preview_error'] = str(error)
        self.event('recompute')
        return self.save()

    def reference_camera(self):
        if not self.state['camera_identity']:
            raise ValueError('No archived RGB camera identity')
        if not self.state['nominal_model']:
            sha = digest(DEFAULT_VENDOR_URDF)
            target = self.root/('nominal_camera_model_'+sha+'.urdf')
            if not target.exists():
                shutil.copyfile(DEFAULT_VENDOR_URDF, target)
            if digest(target) != sha:
                raise ValueError('Nominal camera model SHA mismatch')
            self.state['nominal_model'] = dict(file=target.name, sha256=sha, source=str(DEFAULT_VENDOR_URDF))
            self.event('archive_nominal_camera_model', sha256=sha)
        archived = self.state['nominal_model']
        path = local_file(self.root, archived['file'])
        if digest(path) != archived['sha256']:
            raise ValueError('Nominal camera model changed')
        return torso_color_transform(self.state['camera_identity']['depth_to_color_extrinsics'], path)

    def solve(self):
        self.recompute()
        usable = [s for s in self.state['samples'] if s['result']['solve_usable'] and not s['excluded']]
        fit = [s for s in usable if s['group'] == 'fit']
        gs = [s['result']['pairing']['G_T_W_row_major'] for s in fit]
        ps = [s['result']['pose']['T_C_Q_row_major'] for s in fit]
        x, y, coverage = solve_transforms(gs, ps, self.state['solve_limits'])
        x_old = self.reference_camera()
        y_old = mean_transform([np.linalg.inv(g) @ x_old @ p for g, p in zip(gs, ps)])
        reports = {}
        for group in ('fit', 'validation'):
            samples = [s for s in usable if s['group'] == group]
            g = [s['result']['pairing']['G_T_W_row_major'] for s in samples]
            p = [s['result']['pose']['T_C_Q_row_major'] for s in samples]
            reports[group] = dict(ids=[s['id'] for s in samples], old=residuals(g, p, x_old, y_old),
                                  candidate=residuals(g, p, x, y))
        self.state['candidate'] = dict(X_T_torso_RGB_row_major=x.tolist(), Y_T_wrist_board_row_major=y.tolist(),
            X_old_row_major=x_old.tolist(), Y_old_fit_only_row_major=y_old.tolist(), applied=False,
            mount_id=self.state['mount_id'], side=self.state['side'], coverage=coverage, reports=reports,
            validation_status='diagnostic_only_not_field_accepted', method='OpenCV PARK; SO(3) mean Y; fit only',
            interpretation='Consistency with current wrist kinematics, not absolute ground truth')
        self.event('solve', fit_ids=[s['id'] for s in fit]); self.save()
        return self.state['candidate']

    def export(self):
        if self.state['candidate'] is not None:
            self.solve()
        else:
            self.recompute()
        path = self.root/'exports'/datetime.now().strftime('%Y%m%d_%H%M%S_%f')
        path.mkdir(parents=True)
        for sample in self.state['samples']:
            self.verify(sample)
            for name in set(sample['sha256']) | set(sample['views'].values()):
                target = path/name; target.parent.mkdir(parents=True, exist_ok=True)
                shutil.copyfile(local_file(self.root, name), target)
        for key in ('model', 'nominal_model'):
            if self.state.get(key):
                name = self.state[key]['file']; target = path/name; target.parent.mkdir(parents=True, exist_ok=True)
                shutil.copyfile(local_file(self.root, name), target)
        self.event('export', path=str(path))
        write_json(path/'session.json', self.state)
        write_json(path/'report.json', dict(applied=False, candidate=self.state['candidate'],
            validation_status='not_field_accepted', pairing_method='owner_stream_restart_static_v1',
            hardware_sync=False, transport_age_certified=False,
            counts=dict(samples=len(self.state['samples']),
                        rgb_computable=sum(s['result']['rgb_valid'] for s in self.state['samples']),
                        paired=sum(s['result']['pairing_valid'] for s in self.state['samples'])),
            interpretation='RGB replay and current wrist kinematic consistency, not absolute truth',
            exclusion_audit=[dict(id=s['id'], group=s['group'], excluded=s['excluded'], reason=s['exclusion_reason'])
                             for s in self.state['samples']]))
        write_json(path/'manifest.json', dict(schema=SCHEMA, applied=False,
                   sha256={p.relative_to(path).as_posix(): digest(p) for p in path.rglob('*') if p.is_file()}))
        self.save()
        return path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--session', type=Path)
    parser.add_argument('--import-b-session', type=Path)
    parser.add_argument('--root', type=Path)
    parser.add_argument('--side', choices=['left', 'right'], default='left')
    parser.add_argument('--mount-id', default='offline-unconfirmed')
    parser.add_argument('--export', action='store_true')
    parser.add_argument('--solve', action='store_true')
    parser.add_argument('--actual', type=Path)
    parser.add_argument('--model', type=Path)
    parser.add_argument('--bounds', type=Path)
    parser.add_argument('--sample-id')
    args = parser.parse_args()
    cv2.setNumThreads(1)
    session = CameraCSession.load(args.session) if args.session else CameraCSession(args.root, args.side, args.mount_id)
    if args.import_b_session:
        from camera_b_node import CameraBSession
        source = CameraBSession.load(args.import_b_session)
        for sample in source.state['samples']:
            new = session.import_bundle(source.root/sample['directory'], sample['group'], sample['note'],
                                        dict(b_session=str(args.import_b_session), b_sample_id=sample['id'],
                                             excluded=sample['excluded'], acquisition=sample.get('acquisition')))
            new['excluded'] = sample['excluded']
            new['exclusion_reason'] = 'Inherited B exclusion' if sample['excluded'] else ''
    session.recompute()
    if args.actual:
        if not all((args.model, args.bounds, args.sample_id)):
            parser.error('--actual requires --model, --bounds and --sample-id')
        session.attach_actual(args.sample_id, args.actual, args.model, read_json(args.bounds))
    if args.solve:
        session.solve()
    if args.export:
        print('Export:', session.export())
    print('Session:', session.save())
    print('Samples:', len(session.state['samples']), 'RGB valid:', sum(s['result']['rgb_valid'] for s in session.state['samples']),
          'paired:', sum(s['result']['pairing_valid'] for s in session.state['samples']))


if __name__ == '__main__':
    main()
