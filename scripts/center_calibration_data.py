"""Offline center-check observations. No robot, ROS, or planner dependencies."""
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import tempfile
from datetime import datetime, timezone
import uuid

import numpy as np


SCHEMA = 'center_observations_v2'


def vector(value, name):
    result = np.asarray(value, dtype=float)
    if result.shape != (3,) or not np.isfinite(result).all():
        raise ValueError(f'{name}: expected three finite numbers')
    return result


def read_source(path):
    path = Path(path).resolve()
    raw = path.read_bytes()
    result = json.loads(raw)
    plan = json.loads((path.parent / 'plan.json').read_bytes())
    if plan.get('task_schema') != 'index_center_check_v1':
        raise ValueError('不是 Center check 记录')
    if result.get('settled') is not True:
        raise ValueError('该记录尚未完成到位核验')
    arm = plan.get('arm')
    if arm not in ('left', 'right'):
        raise ValueError('缺少有效手臂信息')
    evidence = plan['center_check']
    if evidence.get('frame') != 'pelvis' or evidence.get('operation_offsets_applied') is not False:
        raise ValueError('需要未应用任务补偿的 pelvis 定位记录')
    target = vector(evidence['target_pelvis_m'], 'target')
    center = vector(evidence['center_pelvis_m'], 'center')
    height = float(evidence['height_mm'])
    if not math.isfinite(height) or not 0 < height < 1000:
        raise ValueError('目标高度无效')
    if not np.allclose(target - center, [0, 0, height / 1000], atol=1e-8, rtol=0):
        raise ValueError('目标高度与圆心不一致')
    if not np.allclose(vector(result['target_pelvis_m'], 'result target'), target, atol=1e-8, rtol=0):
        raise ValueError('结果与规划目标不一致')
    fk = vector(result['fk_error_mm'], 'FK error')
    return dict(source_id=f'{path.parent.parent.name}/{path.parent.name}',
                source_path=str(path), source_sha256=hashlib.sha256(raw).hexdigest(),
                arm=arm, height_mm=height, center_pelvis_m=center.tolist(),
                target_pelvis_m=target.tolist(), fk_error_mm=fk.tolist(),
                result_snapshot=result, geometry_snapshot=evidence)


def scan_sources(root):
    sources, errors = [], []
    for path in sorted(Path(root).glob('*/plan_*/center_check_result.json'), reverse=True):
        try:
            sources.append(read_source(path))
        except (ValueError, KeyError, OSError, TypeError) as error:
            errors.append(f'{path.parent.parent.name}/{path.parent.name}: {error}')
    return sources, errors


def observation(source, forward_mm, left_mm, height_mm, datum, role='fit', note=''):
    if not datum.strip():
        raise ValueError('请填写固定的实体测量点名称')
    if role not in ('fit', 'validation', 'excluded'):
        raise ValueError('样本用途无效')
    values = vector([forward_mm, left_mm, height_mm], '实测数据')
    if np.max(np.abs(values)) > 1000:
        raise ValueError('输入单位为毫米，请检查数值')
    physical = values - [0, 0, source['height_mm']]
    geometric = physical - vector(source['fk_error_mm'], 'FK error')
    return dict(sample_id=uuid.uuid4().hex, source=source, datum=datum.strip(), role=role, note=note.strip(),
                measured_forward_left_height_mm=values.tolist(),
                physical_error_mm=physical.tolist(), geometric_error_mm=geometric.tolist(),
                recorded_at=datetime.now(timezone.utc).isoformat())


def atomic_json(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    text = json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + '\n'
    fd, temporary = tempfile.mkstemp(prefix=f'.{path.name}.', dir=path.parent)
    try:
        with os.fdopen(fd, 'w', encoding='utf-8') as stream:
            stream.write(text)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def load_samples(path):
    path = Path(path)
    if not path.exists():
        return []
    data = json.loads(path.read_text(encoding='utf-8'))
    if data.get('schema') not in (SCHEMA, 'center_observations_v1'):
        raise ValueError('不支持的标定文件格式')
    samples = []
    keys = set()
    for index, row in enumerate(data['samples']):
        # Recompute derived fields rather than trusting stale edited calculations.
        rebuilt = observation(row['source'], *row['measured_forward_left_height_mm'],
                              row['datum'], row['role'], row.get('note', ''))
        rebuilt['recorded_at'] = row['recorded_at']
        key = row.get('sample_id')
        if key is None and data['schema'] == 'center_observations_v1':
            key = uuid.uuid5(uuid.NAMESPACE_URL, f"{row['source']['source_id']}:{row['recorded_at']}:{index}").hex
        if not isinstance(key, str) or not key:
            raise ValueError('缺少有效样本编号')
        if key in keys:
            raise ValueError('存在重复样本编号')
        rebuilt['sample_id'] = key
        if 'updated_at' in row:
            rebuilt['updated_at'] = row['updated_at']
        keys.add(key)
        samples.append(rebuilt)
    return samples


def save_samples(path, samples):
    path = Path(path)
    data = dict(schema=SCHEMA, samples=samples)
    # Validate serialization before touching the current file or its history.
    json.dumps(data, allow_nan=False)
    keys = [s['sample_id'] for s in samples]
    if len(set(keys)) != len(keys):
        raise ValueError('存在重复样本编号')
    if path.exists():
        load_samples(path)
        old = json.loads(path.read_text(encoding='utf-8'))
        stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ')
        backup = path.parent / (path.stem + '_history') / f'{stamp}_{uuid.uuid4().hex[:8]}.json'
        atomic_json(backup, old)
    atomic_json(path, data)


def metrics(errors):
    errors = np.asarray(errors, dtype=float)
    return dict(count=len(errors), mean_xyz_mm=errors.mean(axis=0).tolist(),
                rmse_3d_mm=float(np.sqrt(np.mean(np.sum(errors ** 2, axis=1)))),
                max_3d_mm=float(np.linalg.norm(errors, axis=1).max()))


def fit_samples(samples):
    active = [s for s in samples if s['role'] != 'excluded']
    train = [s for s in active if s['role'] == 'fit']
    if not train:
        raise ValueError('至少需要一个参与拟合的样本')
    if len({(s['source']['arm'], s['datum']) for s in active}) != 1:
        raise ValueError('不能混合不同手臂或实体测量点；请排除另一组样本')
    raw = np.array([s['physical_error_mm'] for s in train])
    geometric = np.array([s['geometric_error_mm'] for s in train])
    bias = geometric.mean(axis=0)
    centers = np.array([s['source']['center_pelvis_m'] for s in train]) * 1000
    warnings = ['固定偏移拟合不能单独判定相机外参或指尖模型误差。',
                'XYZ 按固定初始机器人方向测量；实体点、机器人基座和安装关系需保持一致。']
    if len(train) < 3:
        warnings.append('不足 3 个拟合样本，仅为初步偏移估计。')
    if np.max(np.ptp(centers, axis=0)) < 20:
        warnings.append('目标位置跨度不足 20 mm，尚未验证跨位置一致性。')
    validation = [s for s in active if s['role'] == 'validation']
    if not validation:
        warnings.append('没有独立验证样本；拟合残差不是补偿后的真机精度。')
    train_plans = {s['source']['source_id'] for s in train}
    if len(train_plans) < len(train):
        warnings.append('包含同一规划的重复测量；样本数不等于独立定位次数。')
    if train_plans & {s['source']['source_id'] for s in validation}:
        warnings.append('验证与拟合包含相同规划，不属于独立新位置验证。')
    residuals = []
    for s in active:
        residual = np.asarray(s['geometric_error_mm']) - bias
        residuals.append(dict(sample_id=s['sample_id'], source_id=s['source']['source_id'], role=s['role'],
                              residual_xyz_mm=residual.tolist(),
                              residual_3d_mm=float(np.linalg.norm(residual))))
    return dict(schema='center_constant_fit_v1', model='constant_geometric_offset',
                applied=False, extrinsics_identified=False, arm=train[0]['source']['arm'],
                datum=train[0]['datum'], fit_count=len(train), validation_count=len(validation),
                fit_plan_count=len(train_plans),
                mean_physical_error_mm=raw.mean(axis=0).tolist(),
                mean_fk_error_mm=np.mean([s['source']['fk_error_mm'] for s in train], axis=0).tolist(),
                geometric_bias_mm=bias.tolist(), suggested_target_offset_mm=(-bias).tolist(),
                center_span_xyz_mm=np.ptp(centers, axis=0).tolist(),
                fit_before=metrics(geometric), fit_after=metrics(geometric - bias),
                validation_before=metrics([s['geometric_error_mm'] for s in validation]) if validation else None,
                validation_after=metrics([np.asarray(s['geometric_error_mm']) - bias for s in validation]) if validation else None,
                residuals=residuals, warnings=warnings,
                sources=[dict(sample_id=s['sample_id'], source_id=s['source']['source_id'], role=s['role'],
                              sha256=s['source']['source_sha256'], recorded_at=s['recorded_at']) for s in active])


def export_csv(path, samples):
    with Path(path).open('w', encoding='utf-8-sig', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow(['plan', 'arm', 'datum', 'role', 'forward_mm', 'left_mm', 'height_mm',
                         'target_height_mm', 'error_x_mm', 'error_y_mm', 'error_z_mm',
                         'fk_x_mm', 'fk_y_mm', 'fk_z_mm', 'geometry_x_mm', 'geometry_y_mm',
                         'geometry_z_mm', 'note', 'sample_id', 'recorded_at', 'updated_at'])
        for s in samples:
            writer.writerow([s['source']['source_id'], s['source']['arm'], s['datum'], s['role'],
                             *s['measured_forward_left_height_mm'], s['source']['height_mm'],
                             *s['physical_error_mm'], *s['source']['fk_error_mm'],
                             *s['geometric_error_mm'], s['note'], s['sample_id'], s['recorded_at'], s.get('updated_at', '')])
