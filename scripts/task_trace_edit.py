"""Offline joint edits. These create candidates, never execution authorization."""
import copy
import math

import numpy as np
from scipy.ndimage import gaussian_filter1d
from scipy.interpolate import make_interp_spline
from scipy.special import betaincinv


def synthetic(trace, q, t, operation, **details):
    result = copy.deepcopy(trace)
    result['raw'] = dict(positions=np.asarray(q).tolist(), time_s=np.asarray(t).tolist(),
        stamp_ns=[1_000_000_000+round(float(v)*1e9) for v in t], sample_policy='offline_edited')
    # Rail's `hand` is a posture name, not a recorded timeline.
    if isinstance(result.get('hand'), dict):
        result['hand'] = None
    result.pop('trim_edit', None)
    result['optimization'] = dict(label=operation, **details, synthetic=True,
                                  hardware_execution_validated=False)
    return result


def smooth(trace, tolerance=.003, window=.15, step=.002):
    """Bounded time-domain denoising followed by joint arc-length resampling.

    The recording clock and dwells are deliberately removed. The downstream
    quintic fitter and velocity/acceleration/jerk retimer remain mandatory for
    this newly created candidate's first plan.
    """
    if not all(math.isfinite(v) and v > 0 for v in (tolerance, window, step)):
        raise ValueError('Smoothing parameters must be finite and positive')
    q = np.asarray(trace['raw']['positions'], dtype=float)
    t = np.asarray(trace['raw']['time_s'], dtype=float)
    if q.ndim != 2 or len(q) != len(t) or not np.isfinite(q).all() or np.any(np.diff(t) <= 0):
        raise ValueError('Invalid source trajectory')
    uniform = np.linspace(0., t[-1], max(3, math.ceil(t[-1]/.005)+1))
    original = np.column_stack([np.interp(uniform, t, axis) for axis in q.T])
    sigma = window/(uniform[1]-uniform[0])
    if not math.isfinite(sigma) or (8*sigma+1)*original.size > 200000000:
        raise ValueError('平滑计算超过资源预算，请减小窗口或缩短轨迹')
    filtered = gaussian_filter1d(original, sigma, axis=0, mode='nearest')
    u = uniform/uniform[-1]
    filtered += (1-u[:, None])*(q[0]-filtered[0])+u[:, None]*(q[-1]-filtered[-1])
    filtered = np.column_stack([np.interp(t, uniform, axis) for axis in filtered.T])
    original = q
    error = float(np.max(np.abs(filtered-original)))
    filtered = original+(filtered-original)*min(1., tolerance/max(error, 1e-15))
    filtered[0], filtered[-1] = q[0], q[-1]
    arc = np.r_[0., np.cumsum(np.max(np.abs(np.diff(filtered, axis=0)), axis=1))]
    keep = np.r_[True, np.diff(arc) > 1e-12]
    if arc[-1] < .002:
        raise ValueError('Not enough motion to reparameterize')
    # Quintic progress provides stationary ends without stopping at each sample.
    count = max(3, math.ceil(t[-1]/.005)+1, math.ceil(2*arc[-1]/step)+1)
    if count > 600000:
        raise ValueError('Edited trajectory exceeds the sample limit')
    clock = np.linspace(0., t[-1], count)
    u = clock/clock[-1]
    progress = arc[-1]*u**3*(10-15*u+6*u*u)
    rows = np.column_stack([np.interp(progress, arc[keep], axis[keep]) for axis in filtered.T])
    rows[0], rows[-1] = q[0], q[-1]
    return synthetic(trace, rows, clock, '关节去抖＋空间重参数化',
        max_smoothing_rad=float(np.max(np.abs(filtered-original))), tolerance_rad=tolerance,
        window_s=window, joint_step_rad=step, joint_length_rad=float(arc[-1]),
        timing_policy='local', dwells_removed=True, endpoints_preserved=True)


def reverse_into(source, target):
    t = np.asarray(source['raw']['time_s'], dtype=float)
    return synthetic(target, source['raw']['positions'][::-1], t[-1]-t[::-1],
                     '反向路径候选', endpoints_preserved=True, timing_policy='local')


def anchor(trace, targets, max_delta=.02):
    """Blend small endpoint corrections into 20% shoulders; never average poses."""
    if not math.isfinite(max_delta) or max_delta <= 0:
        raise ValueError('Endpoint correction bound must be finite and positive')
    q = np.asarray(trace['raw']['positions'], dtype=float)
    t = np.asarray(trace['raw']['time_s'], dtype=float)
    result = q.copy(); corrections = {}
    for index, target in targets.items():
        if index not in (0, -1):raise ValueError('Expected a start or end anchor')
        delta = np.asarray(target)-q[index]
        if delta.shape != q.shape[1:] or not np.isfinite(delta).all():raise ValueError('Invalid endpoint anchor')
        magnitude = float(np.max(np.abs(delta)))
        if magnitude > max_delta:
            raise ValueError(f'端点偏差 {magnitude:.4f} rad 超过 {max_delta:.4f} rad；请使用反向路径或重录，未自动拉齐')
        u = np.clip((t if index == 0 else t[-1]-t)/(.2*t[-1]), 0., 1.)
        weight = 1-u**3*(10-15*u+6*u*u)
        result += weight[:, None]*delta
        result[index] = target
        corrections[str(index)] = magnitude
    if not corrections or max(corrections.values()) < 1e-12:
        raise ValueError('端点已一致，无需生成新版本')
    return synthetic(trace, result, t, '端点约束拟合候选', endpoint_corrections_rad=corrections,
                     max_endpoint_correction_rad=max_delta)


def cut(trace, start, end, bridge):
    from task_dring_trim import cut as trim
    result = trim(trace, start, end, bridge)
    if isinstance(trace.get('hand'), str):
        result['hand'] = trace['hand']
    return result


def trim_tail(trace, seconds):
    """Discard a suffix, retaining only a linearly sampled cutoff endpoint."""
    t = np.asarray(trace['raw']['time_s'], dtype=float)
    q = np.asarray(trace['raw']['positions'], dtype=float)
    if (t.ndim != 1 or len(t) < 3 or q.ndim != 2 or len(q) != len(t) or
            not np.isfinite(t).all() or not np.isfinite(q).all() or
            t[0] != 0 or np.any(np.diff(t) <= 0)):
        raise ValueError('源轨迹时间或关节数据无效')
    if not math.isfinite(seconds) or seconds <= 0:
        raise ValueError('末尾删除时长须为有限正数')
    end = round(float(t[-1]-seconds), 9)
    if not .4 <= end < t[-1]:
        raise ValueError('截尾后须保留至少 0.4 秒，且删除时长须大于时间戳精度')
    keep = t < end
    times = np.r_[t[keep], end]
    rows = np.vstack((q[keep], [np.interp(end, t, axis) for axis in q.T]))
    if len(times) < 3 or np.any(np.diff(np.rint(times*1e9)) <= 0):
        raise ValueError('截尾后有效样本不足或时间戳过于接近')
    result = synthetic(trace, rows, times, '删除末尾', kind='trim_tail',
        source_seconds=float(t[-1]), deleted_seconds=float(t[-1]-end),
        edited_seconds=end, endpoints_preserved=False, bridge=False)
    if isinstance(trace.get('hand'), dict):
        hand = copy.deepcopy(trace['hand'])
        ht = np.asarray(hand['time_s'], dtype=float)
        hq = np.asarray(hand['q'], dtype=float)
        if (ht.ndim != 1 or len(ht) < 2 or hq.shape != (len(ht), 6) or
                not np.isfinite(ht).all() or not np.isfinite(hq).all() or
                np.any(np.diff(ht) <= 0) or ht[0] > 0 or ht[-1] < end):
            raise ValueError('手指记录未有效覆盖保留区间')
        keep = ht < end
        hand['time_s'] = np.r_[ht[keep], end].tolist()
        hand['q'] = np.vstack((hq[keep], [np.interp(end, ht, axis) for axis in hq.T])).tolist()
        result['hand'] = hand
    return result


def local_speed(trace, start, end, factor=1., blend=.2):
    """Monotone C3 time warp; preserve the joint path and synchronized fingers."""
    t = np.asarray(trace['raw']['time_s'], dtype=float)
    q = np.asarray(trace['raw']['positions'], dtype=float)
    if (not all(math.isfinite(v) for v in (start, end, factor, blend)) or
            not 0 <= start < end <= t[-1] or
            factor <= 0 or blend <= 0):
        raise ValueError('请选择轨迹内的有效区间，倍率和过渡时长须为有限正数')
    if abs(factor-1.) < 1e-9:raise ValueError('1 倍不改变轨迹，不占用历史位置')
    ramp = min(blend, (end-start)/2)
    def integral(u):
        u = np.clip(u, 0., 1.)
        return 2.5*u**4-3*u**5+u**6
    def clock(times):
        x = np.asarray(times, dtype=float)-start
        first = ramp*integral(x/ramp)
        middle = np.clip(x-ramp, 0., end-start-2*ramp)
        last_x = np.clip(x-(end-start-ramp), 0., ramp)
        last = last_x-ramp*integral(last_x/ramp)
        return np.asarray(times)+(1/factor-1)*(first+middle+last)
    mapped = clock(t)
    if not .4 <= mapped[-1] <= 300 or np.any(np.diff(mapped) <= 1e-9):
        raise ValueError('调速后的时间线超过 0.4 至 300 秒或时间戳精度限制')
    # Add samples only along the original piecewise-linear joint path.
    times = np.unique(np.rint(np.r_[mapped, np.linspace(0., mapped[-1], math.ceil(mapped[-1]/.02)+1)]*1e9).astype(np.int64))*1e-9
    if len(times) > 600000:raise ValueError('局部调速超过样本数量上限')
    rows = np.column_stack([np.interp(times, mapped, values) for values in q.T])
    rows[0], rows[-1] = q[0], q[-1]
    result = synthetic(trace, rows, times, '选区局部调速', kind='local_speed',
        timing_policy='uniform', requested_factor=factor, blend_source_s=ramp,
        source_interval_s=[start,end], interval_s=clock([start,end]).tolist(),
        source_seconds=float(t[-1]), edited_seconds=float(times[-1]),
        effective_multiplier=float(t[-1]/times[-1]), endpoints_preserved=True,
        source_optimization=copy.deepcopy(trace.get('optimization')))
    if isinstance(trace.get('hand'), dict):
        hand = trace['hand']; ht = clock(hand['time_s'])
        dense = np.unique(np.r_[ht, np.linspace(ht[0],ht[-1],math.ceil((ht[-1]-ht[0])/.1)+1)])
        result['hand'] = dict(time_s=dense.tolist(), q=np.column_stack([
            np.interp(dense, ht, values) for values in np.asarray(hand['q']).T]).tolist())
    return result


def curve(trace, key_times, seconds=0.):
    """Replace the keypoint interval by a C4 quintic joint-space interpolant.

    Interior points are pass-through constraints, not stop points. Partial
    edits match estimated boundary derivatives, like the trim tool.
    """
    from task_dring_trim import sample,boundary_state
    raw_t = np.asarray(trace['raw']['time_s'], dtype=float)
    keys = np.asarray(sorted(key_times), dtype=float)
    if (keys.ndim != 1 or not 3 <= len(keys) <= 64 or not np.isfinite(keys).all() or
            np.any(np.diff(keys) < .001) or keys[0] < 0 or keys[-1] > raw_t[-1]):
        raise ValueError('请选择 3 至 64 个不同时间点，间隔至少 1 ms')
    duration = keys[-1]-keys[0] if seconds == 0 else seconds
    if not math.isfinite(duration) or duration <= 0:
        raise ValueError('曲线时长须为有限正数')
    total = raw_t[-1]-(keys[-1]-keys[0])+duration
    if not .4 <= total <= 300:
        raise ValueError('修改后的整段轨迹须满足 0.4 至 300 秒的存储格式要求')
    points = sample(trace, keys)
    chord = np.linalg.norm(np.diff(points, axis=0), axis=1)
    if np.any(chord < 1e-5):
        raise ValueError('相邻关键点位姿几乎相同，请删除重复点')
    progress = np.r_[0., np.cumsum(chord)]/np.sum(chord)
    whole=bool(keys[0]==0 and keys[-1]==raw_t[-1])
    knots_at = betaincinv(3., 3., progress) if whole else progress
    _,va,aa=boundary_state(trace,keys[0]);_,vb,ab=boundary_state(trace,keys[-1])
    spline = make_interp_spline(knots_at, points, k=5,
        bc_type=([(1,va*duration),(2,aa*duration**2)],[(1,vb*duration),(2,ab*duration**2)]))
    # The convex hull of derivative control points bounds displacement between
    # samples, including overshoot between selected points.
    rate = float(np.max(np.abs(spline.derivative().c)))
    count = max(3, math.ceil(duration/.005)+1, math.ceil(rate/.0005)+1)
    if count > 600000:raise ValueError('关键点过于密集或曲率过大，请减少重复关键点')
    u = np.unique(np.r_[np.linspace(0.,1.,count), knots_at, np.unique(spline.t)])
    q = spline(u); q[0], q[-1] = points[0], points[-1]
    a, b = float(keys[0]), float(keys[-1])
    times, rows = [], []
    def add(ts, values):
        for stamp, value in zip(ts, values):
            if times and stamp <= times[-1]+1e-9:continue
            times.append(float(stamp)); rows.append(value.tolist())
    original = np.asarray(trace['raw']['positions'])
    mask = raw_t < a; add(raw_t[mask], original[mask])
    begin = a
    add(begin+duration*u, q)
    tail = begin+duration
    mask = raw_t > b; add(raw_t[mask]-b+tail, original[mask])
    if times[-1] > 300 or len(times) > 600000:
        raise ValueError('修改后轨迹超过 300 秒或样本上限')
    result = synthetic(trace, rows, times, '多关键点五次曲线', timing_policy='local',
        source_key_times_s=keys.tolist(), key_times_s=(begin+duration*knots_at).tolist(),
        key_positions=points.tolist(), interval_s=[begin,tail],
        spline=dict(knots=spline.t.tolist(),controls=spline.c.tolist(),degree=5),
        whole_trace=whole, endpoints_preserved=True,bridge_policy='matched_boundary_derivatives_v2',
        interior_continuity='C4', cartesian_circle_exact=False)
    return result
