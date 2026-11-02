"""Position-only fingertip edits. No actuator or motion-service dependencies."""
import copy
import math

import numpy as np
from scipy.interpolate import make_interp_spline
from scipy.spatial.transform import Rotation

from task_dring_teach_data import moving
from task_dring_trim import sample
from task_trace_edit import synthetic
from task_trace_markers import hand_context


def point_state(model, trace, side, context, config, slot, stamp):
    if not np.isfinite(stamp) or not 0 <= stamp <= trace['raw']['time_s'][-1]:
        raise ValueError('时间点不在当前轨迹内')
    state = dict(context)
    state.update(trace.get('context_joints', {}))
    state = hand_context(trace, side, state, config, slot)
    if isinstance(trace.get('hand'),dict):
        from task_hand_control import model_joints
        hand=trace['hand']
        state.update(model_joints(side,[np.interp(stamp,hand['time_s'],axis)
            for axis in np.asarray(hand['q']).T]))
    state.update(zip(moving(side), sample(trace, stamp)))
    return state, model.transform(side+'_index_tcp', state)


def solve_point(model, trace, side, context, config, slot, stamp, position):
    from task_ik_search import bounded_pose_ik
    state, target = point_state(model, trace, side, context, config, slot, stamp)
    position = np.asarray(position, dtype=float)
    if position.shape != (3,) or not np.isfinite(position).all():
        raise ValueError('不成功：目标坐标须为有限数值')
    target = target.copy(); target[:3, 3] = position
    names = moving(side)
    answer, diagnostic = bounded_pose_ik(model, side+'_index_tcp', names[:-1], model.limits,
        state, target, np.eye(4), lambda: None, 2.)
    if answer is None:raise ValueError('不成功：保持原姿态时无法解算该位置')
    return dict(time=float(stamp), position=position.tolist(),
                joints=[float(answer[name]) for name in names],
                position_error_m=diagnostic['position_error_m'])


def manual_trace(trace, points, start, end, model, side, context, config, slot):
    """Add a C4 correction within a fixed interval, retaining the source clock."""
    if not points:raise ValueError('尚无成功解算的手动点')
    t = np.asarray(trace['raw']['time_s'], dtype=float)
    if not np.isfinite([start,end]).all() or not 0 <= start < end <= t[-1]:
        raise ValueError('手动编辑区间无效')
    names = moving(side)
    corrections = {float(start):np.zeros(len(names)), float(end):np.zeros(len(names))}
    seen = set()
    for point in points:
        stamp = point['time']
        if not start <= stamp <= end or stamp in seen:raise ValueError('手动点重复或超出编辑区间')
        if stamp in (start,end) and stamp not in (0.,t[-1]):
            raise ValueError('内部选区边界须保持原位')
        seen.add(stamp)
        state, original = point_state(model,trace,side,context,config,slot,stamp)
        q = np.asarray(point['joints'],dtype=float)
        position = np.asarray(point['position'],dtype=float)
        if q.shape != (len(names),) or position.shape != (3,) or not np.isfinite(np.r_[q,position]).all():
            raise ValueError('手动点数据无效')
        if abs(q[-1]-state[names[-1]])>1e-12:raise ValueError('手动点编辑不能改变腰部位置')
        state.update(zip(names,q)); actual = model.transform(side+'_index_tcp',state)
        if (np.linalg.norm(actual[:3,3]-position) > .0002 or
                Rotation.from_matrix(original[:3,:3].T@actual[:3,:3]).magnitude() > .002):
            raise ValueError('不成功：手动点未满足位置或原姿态约束')
        corrections[float(stamp)] = q-sample(trace,stamp)
    keys = np.array(sorted(corrections))
    if np.any(np.diff(keys)<1e-9):raise ValueError('手动点间隔小于时间戳精度')
    values = np.array([corrections[key] for key in keys])
    zero = np.zeros(len(names))
    spline = make_interp_spline(keys,values,k=5,
        bc_type=([(1,zero),(2,zero)],[(1,zero),(2,zero)]))
    # Bound added sample displacement; original samples remain in the output.
    rate = float(np.max(np.abs(spline.derivative().c)))
    requested = max((end-start)/.005, (end-start)*rate/.002)
    if not np.isfinite(requested) or requested+len(t)+len(keys)>600000:
        raise ValueError('手动编辑超过采样资源预算')
    fixed = np.unique(np.r_[t,keys])
    if np.any(np.diff(fixed)<=1e-9):
        raise ValueError('手动点与已有采样过近，请调整时间位置')
    grid = np.linspace(start,end,max(2,int(np.ceil(requested))+1))
    index = np.searchsorted(fixed,grid)
    distance = np.minimum(abs(grid-fixed[np.maximum(0,index-1)]),
                          abs(grid-fixed[np.minimum(index,len(fixed)-1)]))
    times = np.sort(np.r_[fixed,grid[distance>1e-9]])
    rows = sample(trace,times)
    mask = (times>=start)&(times<=end)
    rows[mask] += spline(times[mask])
    # Validate extrema on each polynomial piece of the source + correction.
    from scipy.interpolate import PPoly
    limits = np.array([model.limits[name] for name in names])
    breaks = np.unique(np.r_[keys,t[(t>start)&(t<end)]])
    starts,steps = breaks[:-1],np.diff(breaks)
    source = sample(trace,breaks)
    power_to_bernstein = np.array([[math.comb(k,j)/math.comb(5,j) if j<=k else 0.
                                   for j in range(6)] for k in range(6)])
    for axis in range(len(names)):
        piece = PPoly.from_spline((spline.t,spline.c[:,axis],5))
        coefficients = np.array([piece(starts,nu=i)*steps**i/math.factorial(i) for i in range(6)]).T
        coefficients[:,0] += source[:-1,axis]
        coefficients[:,1] += np.diff(source[:,axis])
        hull = coefficients@power_to_bernstein.T
        uncertain = np.flatnonzero((hull.min(axis=1)<limits[axis,0]+.02)|
                                  (hull.max(axis=1)>limits[axis,1]-.02))
        for index in uncertain:
            coeff=coefficients[index]
            roots = np.polynomial.polynomial.polyroots(np.polynomial.polynomial.polyder(coeff))
            probes = [0.,1.]+[r.real for r in roots if abs(r.imag)<1e-8 and 0<r.real<1.]
            extrema = np.polynomial.polynomial.polyval(probes,coeff)
            if min(extrema)<limits[axis,0]+.02 or max(extrema)>limits[axis,1]-.02:
                raise ValueError('不成功：连接曲线超出关节限位 '+names[axis])
    result = synthetic(trace,rows,times,'手动点平移与插点',kind='manual_points',
        timing_policy='local',manual_points=copy.deepcopy(points),interval_s=[start,end],
        fixed_keypoint_orientation=True,collision_checked=False)
    result['hand'] = copy.deepcopy(trace.get('hand'))
    return result
