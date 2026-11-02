"""Experiment A: immutable measured wrist poses, pivot fit and held-out checks.

T_A_B maps B coordinates to A. Lengths: m; angles: rad; matrices: row-major;
quaternions, when present: xyzw. No ROS, hand transport or control dependencies.
"""
from __future__ import annotations

import copy
import csv
from datetime import datetime, timezone
import io
import json
from pathlib import Path
import uuid

import numpy as np
from scipy.spatial.transform import Rotation


DEFAULT_LIMITS = dict(window_s=1.0, max_age_s=.15, max_gap_s=.10,
                      max_speed_rad_s=.02, max_joint_span_rad=.002, max_tip_drift_m=.00075,
                      max_wrist_rotation_rad=.002, max_condition=100.,
                      outlier_m=.003, hand_change=.02, reference_skew_s=.03)
CONFIRMATIONS = ('source_actual', 'pelvis_fixed', 'hand_rigid', 'contact_fixed', 'light_contact')
LIMITATIONS = [
    '无相机参与的同点散布不能仅由相机外参解释；也可能来自接触、手型、腰臂模型或基座移动。',
    '固定TCP修正只估计该手型的三维接触点，不是六维工具姿态或整只手的标定。',
    '独立验证改善支持有效末端偏置解释部分误差，不能单独定位加长件长度、安装角或手指尺寸。',
    '执行器反馈不是独立指骨角度测量；固定手型仍需人工确认，受力弯曲不由编码器排除。',
    '跟踪TCP差由同一模型传播得到，不是实体指尖的独立测量；不参与TCP拟合，也不从拟合残差中扣除。',
    '新实体点的重新居中只检验同点一致性，不验证相对胯部的绝对位置精度。',
    'pelvis相对桌面不动是实验条件，不由RViz固定坐标系或固定world TF证明。',
]


def now_iso():
    return datetime.now(timezone.utc).isoformat()


def checked_limits(values):
    # Older sessions did not have a measured-position span gate.
    if set(values) == set(DEFAULT_LIMITS)-{'max_joint_span_rad'}:
        values = {**values, 'max_joint_span_rad': DEFAULT_LIMITS['max_joint_span_rad']}
    if set(values) != set(DEFAULT_LIMITS):
        raise ValueError('诊断门限字段不完整或含未知字段')
    for key, value in values.items():
        if isinstance(value, bool) or not isinstance(value, (float, int)) or not np.isfinite(value) or value <= 0:
            raise ValueError('无效诊断门限：'+key)
    if not .5 <= values['window_s'] <= 5 or values['max_gap_s'] > .2 or values['max_age_s'] > .3:
        raise ValueError('采样窗口/时效门限超出支持范围')
    if values['max_speed_rad_s'] > .05 or values['max_joint_span_rad'] > .01 or values['max_tip_drift_m'] > .003 or values['max_wrist_rotation_rad'] > .01:
        raise ValueError('稳定门限过宽，拒绝用于接触标定')
    return dict(values)


def vector(value, size=3):
    result = np.asarray(value, float)
    if result.shape != (size,) or not np.isfinite(result).all():
        raise ValueError('数值维度错误或含非有限值')
    return result


def transform(value):
    t = np.asarray(value, float)
    if (t.shape != (4, 4) or not np.isfinite(t).all() or
            not np.allclose(t[3], [0, 0, 0, 1], atol=1e-8) or
            not np.allclose(t[:3, :3].T @ t[:3, :3], np.eye(3), atol=1e-6) or
            not np.isclose(np.linalg.det(t[:3, :3]), 1., atol=1e-6)):
        raise ValueError('无效刚体变换 T_pelvis_wrist')
    return t


def point(g, s):
    g = transform(g)
    return g[:3, :3] @ vector(s) + g[:3, 3]


def transform_window(values):
    gs = np.asarray(values, float)
    if (gs.ndim != 3 or gs.shape[1:] != (4, 4) or not len(gs) or not np.isfinite(gs).all() or
            not np.allclose(gs[:, 3, :], [0, 0, 0, 1], atol=1e-8) or
            not np.allclose(gs[:, :3, :3].transpose(0, 2, 1) @ gs[:, :3, :3], np.eye(3), atol=1e-6) or
            not np.allclose(np.linalg.det(gs[:, :3, :3]), 1., atol=1e-6)):
        raise ValueError('窗口含无效刚体变换 T_pelvis_wrist')
    return gs


def metrics(points, center=None):
    p = np.asarray(points, float)
    if p.ndim != 2 or p.shape[1] != 3 or not len(p) or not np.isfinite(p).all():
        raise ValueError('缺少有效点')
    center = p.mean(axis=0) if center is None else vector(center)
    residuals = p - center
    distances = np.linalg.norm(residuals, axis=1)
    return dict(n=len(p), center_m=center.tolist(), axis_std_m=p.std(axis=0).tolist(),
                rms_m=float(np.sqrt(np.mean(distances**2))), max_m=float(distances.max()),
                p95_m=float(np.quantile(distances, .95)), residuals_m=residuals.tolist())


def stability(records, at, limits, s, *, entire_window=False):
    """Check a complete contiguous window, not just its two endpoints."""
    if not records or at-records[-1]['received'] > limits['max_age_s']:
        raise ValueError('LowState断流/过期：没有新鲜的实际关节反馈')
    end = records[-1]['received']
    window = list(records) if entire_window else [r for r in records if r['received'] >= end-limits['window_s']-.02]
    times = np.array([r['received'] for r in window])
    if len(times) < 10 or times[-1]-times[0] < limits['window_s']-.025:
        raise ValueError(f"稳定观察窗不足：需连续观察{limits['window_s']:.1f}秒后再采样")
    if np.max(np.diff(times)) > limits['max_gap_s'] or np.any(np.diff(times) <= 0):
        raise ValueError('稳定窗内反馈断流或时间倒跳')
    if any(r['source'] != 'lowstate_actual' for r in window):
        raise ValueError('拒绝命令、默认值或演示状态作为实际反馈')
    gs = transform_window([r['G'] for r in window])
    tips = np.einsum('nij,j->ni', gs[:, :3, :3], vector(s)) + gs[:, :3, 3]
    velocities = np.asarray([r['dq'] for r in window], float)
    if velocities.ndim != 2 or not velocities.shape[1] or not np.isfinite(velocities).all():
        raise ValueError('窗口关节速度维度错误或含非有限值')
    speed = float(np.max(np.abs(velocities)))
    if any('q_stability' not in r for r in window):
        raise ValueError('缺少实际关节角窗口，不能仅凭FK或速度读数判断稳定')
    positions = np.asarray([r['q_stability'] for r in window], float)
    if positions.shape != velocities.shape or not np.isfinite(positions).all():
        raise ValueError('窗口实际关节角维度错误或含非有限值')
    span = float(np.max(np.ptp(positions, axis=0)))
    drift = float(np.max(np.linalg.norm(tips-tips[-1], axis=1)))
    angle = float(np.max(Rotation.from_matrix(gs[-1, :3, :3].T @ gs[:, :3, :3]).magnitude()))
    if span > limits.get('max_joint_span_rad', DEFAULT_LIMITS['max_joint_span_rad']):
        raise ValueError(f'实际关节角不稳定：窗口跨度 {np.degrees(span):.3f} deg；不是原始dq峰值判定')
    if drift > limits['max_tip_drift_m'] or angle > limits['max_wrist_rotation_rad']:
        raise ValueError(f'实际姿态不稳定：TCP漂移 {drift*1000:.3f} mm，腕转动 {np.degrees(angle):.3f} deg')
    return dict(window_s=float(times[-1]-times[0]), frames=len(window),
                age_s=float(at-end), max_speed_rad_s=speed, tip_drift_m=drift,
                wrist_rotation_rad=angle, max_joint_span_rad=span,
                stability_method='measured_position_window_v2; raw dq warning only',
                warning=f'原始dq峰值 {speed:.4f} rad/s 偏高，但实际关节角/位姿窗口满足门限' if speed > limits['max_speed_rad_s'] else '')


def average_observation(records, wrist_fk, s):
    """One contact/one wrist pose: arithmetic joint means, then consistent FK.

    Also retain the exact arithmetic mean and scatter of per-frame model tips.
    No filtered/artificial state is published back to the robot or TF.
    """
    if len(records) < 2:
        raise ValueError('缺少平均采样帧')
    result = copy.deepcopy(records[-1])
    for key in ('q', 'dq_map', 'tau_est'):
        names = sorted(records[0][key])
        if any(set(r[key]) != set(names) for r in records):
            raise ValueError('平均采样期间关节字段改变')
        result[key] = dict(zip(names, vector(np.mean([[r[key][n] for n in names] for r in records], axis=0), len(names)).tolist()))
    result['G'] = transform(wrist_fk(result['q'])).tolist()
    result['received'] = float(np.mean([r['received'] for r in records]))
    result['ros_receipt_ns'] = round(sum(r['ros_receipt_ns'] for r in records)/len(records))
    gs = transform_window([r['G'] for r in records])
    tips = np.einsum('nij,j->ni', gs[:, :3, :3], vector(s)) + gs[:, :3, 3]
    scatter = metrics(tips)
    return result, dict(method='arithmetic mean measured q then FK; per-frame tip arithmetic mean also retained',
        duration_s=records[-1]['received']-records[0]['received'], frames=len(records),
        first_tick=records[0]['tick'], last_tick=records[-1]['tick'],
        first_ros_receipt_ns=records[0]['ros_receipt_ns'], last_ros_receipt_ns=records[-1]['ros_receipt_ns'],
        mean_model_point_m=scatter['center_m'], model_point_std_m=scatter['axis_std_m'],
        model_point_rms_m=scatter['rms_m'], model_point_max_m=scatter['max_m'],
        fk_of_mean_q_minus_mean_model_point_m=(point(result['G'], s)-tips.mean(axis=0)).tolist())


def tracking(g_actual, g_reference, s):
    a, b = point(g_actual, s), point(g_reference, s)
    return dict(actual_m=a.tolist(), reference_m=b.tolist(),
                actual_minus_reference_m=(a-b).tolist(), norm_m=float(np.linalg.norm(a-b)))


def new_session(side='left', mode='live', model=None, s_model=None, limits=None):
    if side not in ('left', 'right') or mode not in ('live', 'offline'):
        raise ValueError('无效实验模式/手臂')
    return dict(schema_version=1, experiment='A', session_id=uuid.uuid4().hex,
                created_at=now_iso(), side=side, mode=mode, base_frame='pelvis',
                wrist_frame=side+'_wrist_yaw_link', tcp_frame=side+'_index_tcp',
                convention='T_A_B maps B to A; m; rad; matrix row-major; quaternion xyzw',
                contact_definition='食指最末端正前方的小接触点；不是整片指腹；每次同一实体位置',
                hand_assumption='人工确认刚性固定index_only；模型/mimic不是实测指骨角',
                model=copy.deepcopy(model or {}), s_model=vector(s_model if s_model is not None else [0, 0, 0]).tolist(),
                limits=checked_limits({**DEFAULT_LIMITS, **(limits or {})}), samples=[], audit=[], result=None)


def check_sample(session, sample):
    transform(sample['G'])
    if not isinstance(sample['target_id'], str) or not sample['target_id'].strip():
        raise ValueError('实体触点ID不能为空')
    if not isinstance(sample.get('excluded', False), bool):
        raise ValueError('排除标记必须是布尔值')
    if sample.get('group') not in ('fit', 'validation') or sample.get('phase') not in ('unified', 'fixed_waist', 'waist_validation'):
        raise ValueError('无效样本用途/腰部阶段')
    if not all(sample.get('confirmations', {}).get(k) is True for k in CONFIRMATIONS):
        raise ValueError('缺少真实来源、胯部固定、固定手型、真实同点或轻触确认')
    if session['mode'] == 'live' and sample.get('source') != 'lowstate_actual':
        raise ValueError('真机会话禁止加入演示/目标/回放状态')
    if sample.get('model_sha256') != session['model'].get('sha256'):
        raise ValueError('样本模型与会话基线不一致')
    if sample.get('hand_feedback') is not None:
        q = vector(sample['hand_feedback']['feedback_q'], 6)
        if np.any(q < 0) or np.any(q > 1):
            raise ValueError('无效手部执行器反馈')
        previous = next((r for r in session['samples'] if r.get('hand_feedback') is not None), None)
        if previous and max(abs(q-vector(previous['hand_feedback']['feedback_q'], 6))) > session['limits']['hand_change']:
            raise ValueError('手型执行器反馈改变：请恢复原手型或新建会话')


def append_sample(session, sample):
    sample = copy.deepcopy(sample)
    sample.setdefault('id', uuid.uuid4().hex[:12])
    sample.setdefault('excluded', False)
    sample.setdefault('created_at', now_iso())
    check_sample(session, sample)
    if any(s['id'] == sample['id'] for s in session['samples']):
        raise ValueError('样本ID重复')
    session['samples'].append(sample)
    invalidate(session, 'append', sample['id'])
    return sample


def invalidate(session, operation, sample_id=''):
    session['result'] = None
    session['audit'].append(dict(at=now_iso(), operation=operation, sample_id=sample_id))


def solve(session):
    checked_limits(session['limits'])
    samples = [r for r in session['samples'] if not r['excluded']]
    fits = [r for r in samples if r['group'] == 'fit']
    if len(fits) < 6:
        raise ValueError('至少需要6个拟合样本；建议8~12个多轴腕姿，数量本身不是通过条件')
    target_ids = {r['target_id'] for r in fits}
    if len(target_ids) != 1:
        raise ValueError('第一版只拟合一个实体点；其他target_id请设为独立验证')
    gs = [transform(r['G']) for r in fits]
    design = np.vstack([np.c_[g[:3, :3], -np.eye(3)] for g in gs])
    rhs = np.concatenate([-g[:3, 3] for g in gs])
    values, _, rank, singular = np.linalg.lstsq(design, rhs, rcond=None)
    condition = float(singular[0]/singular[-1]) if singular[-1] > 1e-12 else None
    if rank < 6 or condition is None or condition > session['limits']['max_condition']:
        raise ValueError(f'姿态覆盖不足/不可可靠求解：rank={rank}/6，condition={condition}；需改变不同旋转轴')
    s, c = values[:3], values[3:]
    original = [point(g, session['s_model']) for g in gs]
    old = metrics(original)
    corrected = metrics([point(g, s) for g in gs], c)
    result = dict(computed_at=now_iso(), fit_target_id=next(iter(target_ids)),
                  s_model_m=session['s_model'], s_candidate_m=s.tolist(),
                  delta_tcp_m=(s-vector(session['s_model'])).tolist(),
                  fit_center_m=c.tolist(), rank=int(rank), singular_values=singular.tolist(),
                  condition=condition, fit_old=old, fit_new=corrected, validation={}, per_sample={}, warnings=[],
                  limitations=LIMITATIONS, algorithm='unweighted SVD least squares [R,-I][s,c]=-t; no automatic exclusions')
    validations = [r for r in samples if r['group'] == 'validation']
    for target_id in sorted({r['target_id'] for r in validations}):
        group = [r for r in validations if r['target_id'] == target_id]
        a = [point(r['G'], session['s_model']) for r in group]
        b = [point(r['G'], s) for r in group]
        same = target_id == result['fit_target_id']
        result['validation'][target_id] = dict(
            n=len(group), same_target=same, old=metrics(a, old['center_m'] if same else None),
            new=metrics(b, c if same else None),
            interpretation='使用拟合组固定中心，验证组不再拟合' if same else '新点分别居中，仅同点一致性；至少3个新腕姿才有参考意义')
    for r in session['samples']:
        a, b = point(r['G'], session['s_model']), point(r['G'], s)
        reference = c if r['target_id'] == result['fit_target_id'] else None
        if reference is None and r['target_id'] in result['validation']:
            reference = result['validation'][r['target_id']]['new']['center_m']
        result['per_sample'][r['id']] = dict(old_m=a.tolist(), new_m=b.tolist(),
            center_m=None if reference is None else list(reference),
            residual_m=None if reference is None else (b-reference).tolist())
        if r in fits and np.linalg.norm(b-c) > session['limits']['outlier_m']:
            result['warnings'].append('残差超过参考阈值 '+r['id']+'；可能为模型、手型或异常接触，不能据此自动排除')
    for target_id, validation in result['validation'].items():
        if validation['new']['rms_m'] >= validation['old']['rms_m']:
            result['warnings'].append('独立验证未改善：'+target_id+'；不建议应用候选TCP，不能只依据拟合残差判断有效')
    if not validations:
        result['warnings'].append('尚无独立验证，低拟合残差不能证明标定有效')
    for row in validations:
        rotation=np.asarray(row['G'])[:3,:3]
        nearest=min(Rotation.from_matrix(rotation @ np.asarray(f['G'])[:3,:3].T).magnitude() for f in fits)
        if nearest < np.radians(3.):
            result['warnings'].append('验证样本 '+row['id']+' 与拟合腕姿相差不足3 deg；不构成新的姿态覆盖')
    if len(fits) < 8:
        result['warnings'].append('拟合样本偏少，建议补充新腕姿')
    if corrected['rms_m'] > session['limits']['outlier_m']:
        result['warnings'].append('修正后仍有明显散布，检查接触、手型及腰臂运动学')
    result['warnings'].append('结果尚未经实体接触精度验收，不自动应用')
    session['result'] = result
    return result


def load_session(path):
    data = json.loads(Path(path).read_text())
    if data.get('schema_version') != 1 or data.get('experiment') != 'A' or data.get('base_frame') != 'pelvis':
        raise ValueError('不是兼容的实验A pelvis会话')
    expected = new_session(data['side'], data['mode'], data['model'], data['s_model'], data['limits'])
    data['limits'] = expected['limits']
    for sample in data['samples']:
        append_sample(expected, sample)
    if data.get('wrist_frame') != data['side']+'_wrist_yaw_link' or data.get('tcp_frame') != data['side']+'_index_tcp':
        raise ValueError('会话坐标系与所选手臂不一致')
    # Derived results are never trusted on load; recompute explicitly.
    data['result'] = None
    return data


def write_new(path, text):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('x', encoding='utf-8') as stream:
        stream.write(text)


def save_session(session, directory):
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    path = directory / ('session_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f')+'.json')
    write_new(path, json.dumps(session, ensure_ascii=False, indent=2, allow_nan=False)+'\n')
    return path


def export_session(session, directory):
    import yaml
    result = solve(session)
    directory = Path(directory) / ('export_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
    directory.mkdir(parents=True, exist_ok=False)
    save_session(session, directory)
    rows = io.StringIO()
    writer = csv.writer(rows)
    writer.writerow(['sample_id', 'target_id', 'group', 'excluded', 'phase', 'source',
                     'x_old_m', 'y_old_m', 'z_old_m', 'x_new_m', 'y_new_m', 'z_new_m',
                     'tracking_norm_m', 'created_at'])
    for r in session['samples']:
        p = result['per_sample'][r['id']]
        writer.writerow([r['id'], r['target_id'], r['group'], r['excluded'], r['phase'], r['source'],
                         *p['old_m'], *p['new_m'], (r.get('tracking') or {}).get('norm_m', ''), r['created_at']])
    write_new(directory/'samples.csv', rows.getvalue())
    candidate = dict(experiment='A', applied=False, hardware_validated=False,
        source_session=session['session_id'], reference_frame='pelvis',
        parent_frame=session['wrist_frame'], contact_position_in_parent_m=result['s_candidate_m'],
        orientation_estimated=False, units='m, rad', quaternion_order='xyzw',
        contact_definition=session['contact_definition'], hand_assumption=session['hand_assumption'],
        model=session['model'], limitations=LIMITATIONS)
    write_new(directory/'candidate_tcp.yaml', yaml.safe_dump(candidate, allow_unicode=True, sort_keys=False))
    lines = ['# 实验A：胯部坐标系固定手型TCP诊断', '',
             f"模式：{session['mode']}；会话：{session['session_id']}；手臂：{session['side']}",
             '离线/合成数据不是本次真机测量。' if session['mode']=='offline' else '来源：实际LowState；实体接触由操作者确认。',
             f"坐标：{session['convention']}", f"接触点：{session['contact_definition']}",
             f"手型条件：{session['hand_assumption']}", '', '## 拟合与独立验证',
             f"拟合数量 {result['fit_new']['n']}；rank {result['rank']}/6；condition {result['condition']:.3f}",
             f"原TCP (mm)：{(np.array(session['s_model'])*1000).tolist()}",
             f"候选TCP (mm)：{(np.array(result['s_candidate_m'])*1000).tolist()}",
             f"拟合RMS：{result['fit_old']['rms_m']*1000:.3f} -> {result['fit_new']['rms_m']*1000:.3f} mm",
             f"修正后最大残差：{result['fit_new']['max_m']*1000:.3f} mm"]
    for target_id, v in result['validation'].items():
        lines.append(f"验证 {target_id}，n={v['n']}：RMS {v['old']['rms_m']*1000:.3f} -> {v['new']['rms_m']*1000:.3f} mm；{v['interpretation']}")
    tracked = [r['tracking']['norm_m'] for r in session['samples'] if not r['excluded'] and r.get('tracking')]
    lines += ['', '## 运控跟踪',
              f'可配对样本 {len(tracked)}；与几何拟合分开，缺失参考不伪造为零。']
    if tracked:
        lines.append(f'模型TCP跟踪差中位数 {np.median(tracked)*1000:.3f} mm；最大 {max(tracked)*1000:.3f} mm。')
    lines += ['', '## 判断边界'] + ['- '+s for s in result['warnings']+LIMITATIONS]
    lines += ['', '## 适用范围', '仅适用于本会话手臂、安装、固定手型、轻触条件和所覆盖腕姿；原始姿态、dq、时间、确认项及门限见JSON。',
              '', '## 后续手工接入与恢复',
              '候选只是 wrist_yaw_link 到接触点的位置。需另行核对 scripts/task_hand_model.py、scripts/estimate_index_tcp.py 和实际运行URDF中的 index_tcp 链；不得把它当相机平移。',
              '本工具未修改上述文件、URDF或在线控制参数，无需恢复。后续如人工接入，先备份对应模型/配置，独立验收后启用；恢复时换回备份并按原流程安全重启。']
    write_new(directory/'REPORT.md', '\n'.join(lines)+'\n')
    return directory


def synthetic_session():
    """A labelled numerical fixture, intentionally not a fabricated robot FK log."""
    rng = np.random.default_rng(19)
    session = new_session(mode='offline', model=dict(sha256='synthetic', source='synthetic_fixture'),
                          s_model=[.2694, -.0249, .0306])
    s = vector(session['s_model']) + [.008, -.004, .005]
    c = np.array([.43, .16, .23])
    for i in range(18):
        g = np.eye(4)
        g[:3, :3] = Rotation.from_rotvec(rng.normal(size=3)*.6).as_matrix()
        g[:3, 3] = c-g[:3, :3]@s+rng.normal(size=3)*.00012
        append_sample(session, dict(G=g.tolist(), source='synthetic_not_hardware', model_sha256='synthetic',
            target_id='fixture_point', group='fit' if i<12 else 'validation',
            phase='fixed_waist' if i<15 else 'waist_validation',
            confirmations={k:True for k in CONFIRMATIONS}, joints={}, tracking=None,
            note='合成数学夹具：这些腕姿不是由真实机器人关节生成'))
    return session
