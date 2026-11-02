"""Measured rail paths and bounded joint replay; never estimates an unknown TCP."""
import copy
import math
from types import SimpleNamespace

import numpy as np
from scipy.interpolate import make_interp_spline
from scipy.special import betaincinv
from scipy.spatial.transform import Rotation
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from task_hand_control import checked_q, model_joints
from task_path_smoothing import joint_line_trajectory, joint_waypoint_trajectory, smooth_candidates
from task_rail_motion import LINK, required_state, VELOCITY_SCALE, ACCELERATION_SCALE
from task_screen_motion import scale_time
from task_screen_teach import MOVING, JOINT_MARGIN, guarded_joints, fingerprint
from task_saved_transition import plan_transition
from task_trajectory import (samples, time_seconds, stretch_to_limits, task_motion_limits,
                             task_jerk_limits, validate_jerk_limits, validate_waist_envelope,
                             coefficients)

STAGES = {
    'approach_A': ('接近推进 A', 'open'),
    'push': ('推进：推进 A → 推进 B', 'grip'),
    'retreat_B': ('推进 B 退出', 'open'),
    'approach_B': ('接近拉出 B', 'open'),
    'pull': ('拉出：拉出 B → 拉出 A', 'pull_grip'),
    'retreat_A': ('拉出 A 退出', 'open'),
}
ENDPOINTS = {
    'push_A': ('推进 A', 'approach_A'),
    'push_B': ('推进 B', 'push'),
    'pull_B': ('拉出 B', 'approach_B'),
    'pull_A': ('拉出 A', 'pull'),
}
PRECEDING_ENDPOINT = {'push':'push_A', 'retreat_B':'push_B',
                      'pull':'pull_B', 'retreat_A':'pull_A'}
TRACE_SCHEMA = 'g1_rail_measured_trace_v1'
MAX_SAMPLES = 30000
FULL_RATE_MAX_SAMPLES = 600000
MAX_SECONDS = 300.
MAX_GAP = .10
SPACING = .002
ANGLE_STEP = math.radians(.5)
JOINT_STEP = .01
TUBE_M = .0005
TUBE_RAD = math.radians(.5)
BOUNDARY_M = .020
BOUNDARY_RAD = math.radians(5.)
BOUNDARY_JOINT = .15


def hand_identity(hands, posture=None):
    keys = ('open', 'grip') if posture is None else (posture,)
    return fingerprint({key:list(checked_q(hands[key]['feedback_q'])) for key in keys})


def trace_hand_matches(trace, stage, hands):
    if trace.get('hand_policy') == 'not_recorded':
        return True
    expected = STAGES[stage][1]
    if trace.get('hand_identity_scope') == 'posture':
        return (trace.get('hand') == expected and
                trace.get('hand_sha256') == hand_identity(hands, expected))
    # Old bundles used one grip and a combined open/grip fingerprint.
    if stage == 'pull' and trace.get('hand') == 'grip':
        if 'pull_grip' in hands and checked_q(hands['pull_grip']['feedback_q']) != checked_q(hands['grip']['feedback_q']):
            return False
        expected = 'grip'
    return trace.get('hand') == expected and trace.get('hand_sha256') == hand_identity(hands)


def recorded_position(config, stage, index):
    trace = config.get('traces', {}).get(stage)
    if trace is None:
        raise ValueError('尚未录制：'+STAGES[stage][0])
    row = trace['raw']['positions'][index]
    if len(row) != len(MOVING) or not all(math.isfinite(v) for v in row):
        raise ValueError('录制端点关节无效')
    return dict(zip(MOVING, row))


def endpoint(config, key):
    return recorded_position(config, ENDPOINTS[key][1], -1)


def align(model, current, target, parameters, check, label):
    """Reviewed right-arm/waist transition; passive arm and fingers stay put."""
    end = dict(current, **{n:target[n] for n in MOVING})
    result = plan_transition(SimpleNamespace(waist_assist=True), model, current, end, 'right',
        SimpleNamespace(velocity=VELOCITY_SCALE, acceleration=ACCELERATION_SCALE), parameters, check, tolerance=1e-9)
    for segment in result:
        segment['label'] = label
        segment['role'] = 'rail_alignment'
    return result


class TraceBuffer:
    """Append-only callback buffer. Caller owns the node lock; no FK or I/O here."""
    def __init__(self, model):
        self.rows, self.times, self.stamps = [], [], []
        self.first_received = None
        self.error = None
        self.lower = np.array([model.limits[n][0]+JOINT_MARGIN for n in MOVING])
        self.upper = np.array([model.limits[n][1]-JOINT_MARGIN for n in MOVING])

    def feed(self, q, received, stamp):
        if self.error:
            return
        try:
            row = np.array([q[n] for n in MOVING], dtype=float)
            if not np.isfinite(row).all() or np.any(row < self.lower) or np.any(row > self.upper):
                raise ValueError('录制关节反馈缺失、非有限或超限')
            if type(stamp) is not int or stamp <= 0 or not math.isfinite(received):
                raise ValueError('录制需要有效源时间戳')
            if self.first_received is None:
                self.first_received = received
            t = received-self.first_received
            if self.rows:
                dt, source_dt = t-self.times[-1], (stamp-self.stamps[-1])*1e-9
                if dt <= 0 or source_dt <= 0 or max(dt, source_dt) > MAX_GAP+1e-8:
                    raise ValueError('录制反馈中断/时间戳异常；该段未保存')
                if np.max(np.abs(row-self.rows[-1])) > .04:
                    raise ValueError('相邻关节反馈跳变过大；请慢速重新示教')
            if len(self.rows) >= MAX_SAMPLES or t > MAX_SECONDS:
                raise ValueError('单段录制超过 5 分钟或 30000 个原始样本')
            self.rows.append(row.tolist()); self.times.append(t); self.stamps.append(stamp)
        except (KeyError, ValueError) as error:
            self.error = str(error)

    def data(self):
        if self.error:
            raise ValueError(self.error)
        return dict(positions=copy.deepcopy(self.rows), time_s=list(self.times), stamp_ns=list(self.stamps))


def arrays(raw, model):
    q = np.asarray(raw['positions'], dtype=float)
    t = np.asarray(raw['time_s'], dtype=float)
    stamps = raw['stamp_ns']
    limit = FULL_RATE_MAX_SAMPLES if raw.get('sample_policy') in ('all_received','offline_edited') else MAX_SAMPLES
    if (q.ndim != 2 or q.shape[1] != len(MOVING) or not 3 <= len(q) <= limit or
            t.shape != (len(q),) or len(stamps) != len(q) or not np.isfinite(t).all() or
            abs(t[0]) > 1e-9 or t[-1] > MAX_SECONDS or t[-1] < .1 or
            any(type(s) is not int or s <= 0 for s in stamps)):
        raise ValueError('Invalid measured rail sample arrays/duration')
    source_dt = np.array([(b-a)*1e-9 for a,b in zip(stamps, stamps[1:])])
    if np.any(np.diff(t) <= 0) or np.any(source_dt <= 0) or max(np.max(np.diff(t)), np.max(source_dt)) > MAX_GAP+1e-8:
        raise ValueError('Measured rail feedback gap exceeds 100 ms')
    guarded_joints(model, q, context='Recorded rail path')
    if np.max(np.abs(np.diff(q, axis=0))) > .04:
        raise ValueError('Recorded rail joint jump exceeds bounds')
    return q, t


def transforms(model, base, q, check):
    result = []
    for i in range(0, len(q), 1000):
        check()
        result.append(model.batch_transforms([LINK], base, MOVING, q[i:i+1000])[LINK])
    return np.concatenate(result)


def analyze(model, trace, check=lambda:None):
    q, t = arrays(trace['raw'], model)
    context = trace.get('context_joints',{})
    if not all(type(v) in (int,float) and math.isfinite(v) for v in context.values()):
        raise ValueError('Invalid recorded joint context')
    poses = transforms(model, context, q, check)
    distance = np.linalg.norm(np.diff(poses[:, :3, 3], axis=0), axis=1)
    rotations = Rotation.from_matrix(poses[:, :3, :3])
    angle = (rotations[:-1].inv()*rotations[1:]).magnitude()
    joint = np.max(np.abs(np.diff(q, axis=0)), axis=1)
    edited=(trace['raw'].get('sample_policy')=='offline_edited' and
            (trace.get('optimization',{}).get('synthetic') is True or
             trace.get('trim_edit',{}).get('synthetic') is True))
    # Acquisition jumps indicate lost feedback. Generated curve samples are not
    # measurements: their continuous interpolant is checked by replay() below.
    if not edited and (np.max(distance) > SPACING+1e-8 or np.max(angle) > math.radians(2.)):
        raise ValueError('原始相邻反馈超过 2 mm 或 2 度；请慢速重录，不补造实测点')
    metric = np.r_[0., np.cumsum(np.maximum.reduce([distance/SPACING, angle/ANGLE_STEP, joint/JOINT_STEP]))]
    if metric[-1] < .25:
        raise ValueError('该段没有足够运动；保留旧记录，请重新录制')
    selected = [0]; traveled = turned = 0.
    for i in range(1, len(q)):
        traveled += distance[i-1]; turned += angle[i-1]
        if (traveled >= SPACING or turned >= ANGLE_STEP or
                np.max(np.abs(q[i]-q[selected[-1]])) >= JOINT_STEP):
            selected.append(i); traveled = turned = 0.
    if selected[-1] != len(q)-1:
        selected.append(len(q)-1)
    # Include corners that a simple distance trigger would otherwise cut across.
    stack = list(zip(selected, selected[1:])); selected = set(selected)
    while stack:
        check()
        a,b = stack.pop()
        if b-a < 2 or metric[b]-metric[a] < 1e-10:
            continue
        u = (metric[a:b+1]-metric[a])/(metric[b]-metric[a])
        line = q[a]+u[:, None]*(q[b]-q[a])
        actual = transforms(model, context, line, check)
        pos_error = np.linalg.norm(actual[:, :3, 3]-poses[a:b+1, :3, 3], axis=1)
        rot_error = (rotations[a:b+1].inv()*Rotation.from_matrix(actual[:, :3, :3])).magnitude()
        ratio = np.maximum.reduce([pos_error/.0003, rot_error/math.radians(.3),
                                   np.max(np.abs(line-q[a:b+1]), axis=1)/.005])
        index = a+int(np.argmax(ratio))
        if np.max(ratio) > 1. and a < index < b:
            selected.add(index); stack.extend([(a,index),(index,b)])
    selected = sorted(selected)
    # Exact duplicates have no geometric interval; preserve the final measured endpoint.
    keep = [selected[0]]
    for i in selected[1:]:
        if metric[i]-metric[keep[-1]] > 1e-8:
            keep.append(i)
        elif len(keep) > 1:
            keep[-1] = i
    if len(keep) < 2 or len(keep) > 4000:
        raise ValueError('Too few/many replay points; teach a shorter, steadier segment')
    return q, metric, keep, dict(raw_count=len(q), replay_count=len(keep),
        length_mm=float(np.sum(distance)*1000), recorded_seconds=float(t[-1]),
        max_gap_ms=float(max(np.max(np.diff(t)), max((b-a)*1e-9 for a,b in
            zip(trace['raw']['stamp_ns'],trace['raw']['stamp_ns'][1:])))*1000),
        max_raw_step_mm=float(np.max(distance)*1000), selected_indices=keep,
        max_sample_step_deg=float(np.degrees(np.max(angle))),
        input_geometry='offline_edited' if edited else 'measured_feedback',
        measured_step_guard_applied=not edited,
        sampling_mm=2., tcp_estimated=False,
        sample_policy=trace['raw'].get('sample_policy','measured_100hz'),
        stored_hz=float((len(q)-1)/t[-1]))


def validate_trace(trace, stage, model, hands):
    if (stage not in STAGES or trace.get('schema') != TRACE_SCHEMA or trace.get('stage') != stage or
            not trace_hand_matches(trace, stage, hands)):
        raise ValueError('轨迹所属阶段/手型不匹配；手型改变后需重录轨迹')
    arrays(trace['raw'], model)
    if trace.get('hand_policy') == 'not_recorded':
        return
    evidence = trace['hand_evidence']
    if not isinstance(evidence, list) or len(evidence) < 2:
        raise ValueError('Trace lacks beginning/end hand evidence')
    expected = checked_q(hands[trace['hand']]['feedback_q'])
    previous = None
    for state in evidence:
        from task_hand_control import require_state
        require_state(state, 'right')
        if max(abs(a-b) for a,b in zip(state['feedback_q'], expected)) > .02:
            raise ValueError('Recorded hand changed during the arm segment')
        if previous is not None and state['timestamp_ms'] <= previous:
            raise ValueError('Recorded hand timestamps did not advance')
        previous = state['timestamp_ms']


def close_boundary(model, a, b):
    ta, tb = (model.transform(LINK, q) for q in (a, b))
    joint = max(abs(a[n]-b[n]) for n in MOVING)
    distance = np.linalg.norm(ta[:3, 3]-tb[:3, 3])
    angle = (Rotation.from_matrix(ta[:3, :3]).inv()*Rotation.from_matrix(tb[:3, :3])).magnitude()
    if joint > BOUNDARY_JOINT or distance > BOUNDARY_M or angle > BOUNDARY_RAD:
        raise ValueError(f'轨迹端点偏差：{distance*1000:.1f} mm / {math.degrees(angle):.1f} 度 / '
                         f'{joint:.3f} rad；上限 20 mm / 5 度 / 0.15 rad，请稍作对齐')


def preview_trajectory(segments, check=lambda:None):
    """One full-state animation, sampled from the same Hermite curves as replay."""
    names = sorted(set().union(*(s['start'].keys() for s in segments)))
    result = JointTrajectory(joint_names=names)
    offset = 0.
    for segment in segments:
        tr = segment['trajectory']
        for a,b in zip(tr.points,tr.points[1:]):
            check()
            dt = time_seconds(b)-time_seconds(a)
            curve = coefficients(a,b,dt)
            for fraction in np.linspace(0.,1.,max(2,math.ceil(dt/.05)+1)):
                stamp = round((offset+time_seconds(a)+fraction*dt)*1e9)
                if result.points and stamp <= round(time_seconds(result.points[-1])*1e9):
                    continue
                values = np.polynomial.polynomial.polyval(fraction,curve)
                state = dict(segment['start'],**dict(zip(tr.joint_names,values)))
                point = JointTrajectoryPoint(positions=[float(state[n]) for n in names])
                point.time_from_start.sec,point.time_from_start.nanosec = divmod(stamp,1000000000)
                result.points.append(point)
        offset += time_seconds(tr.points[-1])
    return result


def sampled_times(trajectory):
    return np.concatenate([np.linspace(time_seconds(a),time_seconds(b),
        max(2,math.ceil((time_seconds(b)-time_seconds(a))/.005)+1))
        for a,b in zip(trajectory.points,trajectory.points[1:])])


def bounded_join(model, start, end, parameters, check):
    close_boundary(model,start,end)
    tr = joint_line_trajectory(MOVING,[start[n] for n in MOVING],[end[n] for n in MOVING])
    vmax,amax = task_motion_limits(MOVING,VELOCITY_SCALE,ACCELERATION_SCALE,parameters)
    jerk = task_jerk_limits(MOVING)
    stretch_to_limits(tr,vmax,amax,jerk)
    rows = np.asarray(list(samples(tr,vmax,amax)))
    guarded_joints(model,rows,context='Rail bounded join')
    check()
    validate_jerk_limits(tr,jerk)
    lo,hi = model.limits[MOVING[-1]]
    validate_waist_envelope(tr,lo+JOINT_MARGIN,hi-JOINT_MARGIN,vmax[MOVING[-1]],amax[MOVING[-1]])
    return tr


def trim_idle_selection(model, trace, q, selected, check):
    """Drop near-endpoint knots/time padding, not measured endpoint positions."""
    poses = transforms(model,trace.get('context_joints',{}),q,check)
    rotations = Rotation.from_matrix(poses[:,:3,:3])
    def near(index):
        return ((np.linalg.norm(poses[:,:3,3]-poses[index,:3,3],axis=1) <= .0003) &
                ((rotations[index].inv()*rotations).magnitude() <= math.radians(.3)) &
                (np.max(np.abs(q-q[index]),axis=1) <= .003))
    first = np.flatnonzero(~near(0)); last = np.flatnonzero(~near(-1))
    if not len(first) or not len(last):return selected, None
    lo,hi = max(0,int(first[0])-1),min(len(q)-1,int(last[-1])+1)
    if hi <= lo:return selected, None
    times = trace['raw']['time_s']
    indices = [0]+[i for i in selected[1:-1] if lo < i < hi]+[len(q)-1]
    return indices,dict(leading_seconds=float(times[lo]-times[0]),
        trailing_seconds=float(times[-1]-times[hi]),active_seconds=float(times[hi]-times[lo]),
        removed_knots=len(selected)-len(indices),endpoints_preserved=True,
        position_tolerance_mm=.3,rotation_tolerance_deg=.3,joint_tolerance_rad=.003)


def replay(model, trace, start, parameters, check=lambda:None):
    q, metric, selected, report = analyze(model, trace, check)
    manual_timing = trace.get('optimization',{}).get('kind') == 'local_speed'
    stored_timing = trace.get('optimization',{}).get('kind') == 'fast_timing'
    if stored_timing or manual_timing or (trace.get('optimization',{}).get('whole_trace') and trace['optimization'].get('spline')):
        from task_trace_curve import trajectory as curve_trajectory
        from task_trajectory_speed import scale_arm_segments
        target_start=dict(start,**dict(zip(MOVING,q[0])))
        close_boundary(model,start,target_start)
        if stored_timing:
            from task_trace_timing import stored_trajectory
            curve,fit=stored_trajectory(trace,model,MOVING)
        elif manual_timing:
            from task_dring_teach_data import fit_trace
            curve,fit=fit_trace(dict(trace,hand=None),model,'right',check)
        else:curve,fit=curve_trajectory(trace,model,MOVING)
        multiplier=1e6 if parameters.rail_replay_fastest else parameters.rail_replay_multiplier
        if not getattr(parameters,'rail_replay_cruise',False):
            timed,timing=scale_arm_segments([dict(label=trace['stage'],trajectory=curve)],multiplier,model,parameters,
                velocity=VELOCITY_SCALE,acceleration=ACCELERATION_SCALE,
                policy='uniform' if manual_timing or stored_timing else 'local',check=check)
            curve=timed[0]['trajectory']
        fit_mm=fit_deg=0.
        if manual_timing:
            vmax,amax=task_motion_limits(MOVING,VELOCITY_SCALE,ACCELERATION_SCALE,parameters)
            fitted=np.asarray(list(samples(curve,vmax,amax)))
            old_times=sampled_times(curve)/time_seconds(curve.points[-1])*trace['raw']['time_s'][-1]
            expected=np.column_stack([np.interp(old_times,trace['raw']['time_s'],q[:,axis])
                                      for axis in range(len(MOVING))])
            actual_pose=transforms(model,start,fitted,check)
            expected_pose=transforms(model,start,expected,check)
            distance=np.linalg.norm(actual_pose[:,:3,3]-expected_pose[:,:3,3],axis=1)
            angle=(Rotation.from_matrix(expected_pose[:,:3,:3]).inv()*Rotation.from_matrix(actual_pose[:,:3,:3])).magnitude()
            if np.max(distance)>TUBE_M or np.max(angle)>TUBE_RAD or np.max(np.abs(fitted-expected))>.01:
                raise ValueError('局部调速拟合超过滑轨 0.5 mm / 0.5 度 / 0.01 rad 路径偏差限制')
            fit_mm=float(np.max(distance)*1000);fit_deg=float(np.degrees(np.max(angle)))
        if getattr(parameters,'rail_replay_cruise',False):
            from task_cruise_timing import retime
            # Reuse the already certified geometry, not its locally optimized clock.
            from task_trajectory_speed import rescale
            curve=rescale(curve,time_seconds(curve.points[-1])/trace['raw']['time_s'][-1])
            curve,timing=retime(curve,model,start,'right',multiplier,parameters,
                velocity=VELOCITY_SCALE,acceleration=ACCELERATION_SCALE,check=check,
                original_policy='uniform' if manual_timing or stored_timing else 'local')
        joined=max(abs(start[n]-target_start[n]) for n in MOVING)>1e-9
        if joined:
            prefix=bounded_join(model,start,target_start,parameters,check)
            shift=time_seconds(prefix.points[-1])
            for point in curve.points:
                ns=round((time_seconds(point)+shift)*1e9)
                point.time_from_start.sec,point.time_from_start.nanosec=divmod(ns,1_000_000_000)
            curve.points=prefix.points[:-1]+curve.points
        vmax,amax=task_motion_limits(MOVING,VELOCITY_SCALE,ACCELERATION_SCALE,parameters)
        validate_jerk_limits(curve,task_jerk_limits(MOVING))
        lo,hi=model.limits['waist_yaw_joint']
        validate_waist_envelope(curve,lo+JOINT_MARGIN,hi-JOINT_MARGIN,vmax[MOVING[-1]],amax[MOVING[-1]])
        if time_seconds(curve.points[-1])>600:raise ValueError('关键点曲线回放超过 10 分钟')
        report.update(interpolation='stored_exact_timing' if stored_timing else 'manual_local_timing' if manual_timing else 'analytical_keypoint_curve',curve_fit=fit,timing=timing,
            start_join=joined,max_fit_mm=fit_mm,max_fit_deg=fit_deg,
            replay_seconds=time_seconds(curve.points[-1]))
        return curve,report
    original_selected = selected
    trim = None
    if parameters.rail_trim_idle_ends:
        selected,trim = trim_idle_selection(model,trace,q,selected,check)
        report['idle_trim'] = trim
    target_start = dict(start, **dict(zip(MOVING, q[0])))
    close_boundary(model, start, target_start)
    vmax, amax = task_motion_limits(MOVING, VELOCITY_SCALE, ACCELERATION_SCALE, parameters)
    jerk = task_jerk_limits(MOVING)
    unique = np.r_[True, np.diff(metric) > 1e-10]
    s_raw, q_raw = metric[unique]/metric[-1], q[unique]
    progress = metric[selected]/metric[-1]
    x = betaincinv(3., 3., np.clip(progress, 0., 1.))
    zeros = np.zeros(len(MOVING))
    spline = make_interp_spline(x, q[selected], k=5,
        bc_type=([(1, zeros), (2, zeros)], [(1, zeros), (2, zeros)]))
    trajectory = JointTrajectory(joint_names=list(MOVING))
    for t in np.unique(spline.t):
        p = JointTrajectoryPoint(positions=spline(t).tolist(), velocities=spline(t,1).tolist(),
                                 accelerations=spline(t,2).tolist())
        p.time_from_start.sec, p.time_from_start.nanosec = divmod(round(float(t)*1e9), 1000000000)
        trajectory.points.append(p)
    stretch_to_limits(trajectory, vmax, amax, jerk)
    if time_seconds(trajectory.points[-1]) > 600.:
        raise ValueError('重放轨迹超过 10 分钟，记录抖动/转折过多；请重新示教')
    def measure(candidate, waypoint=False):
        batch = []
        for i,row in enumerate(samples(candidate,vmax,amax)):
            if i % 1000 == 0:check()
            batch.append(row)
        rows = np.asarray(batch)
        ts = sampled_times(candidate)/time_seconds(candidate.points[-1])
        if waypoint:
            # Each stop-to-stop leg has its own quintic progress, not the global spline clock.
            local = np.clip(ts,0.,1.)*(len(selected)-1)
            leg = np.minimum(local.astype(int),len(selected)-2)
            u = local-leg
            u = u**3*(10+u*(-15+6*u))
            desired_s = progress[leg]+u*(progress[leg+1]-progress[leg])
        else:
            desired_s = ts**3*(10+ts*(-15+6*ts))
        reference = np.column_stack([np.interp(desired_s,s_raw,q_raw[:,i]) for i in range(len(MOVING))])
        actual = transforms(model,start,rows,check)
        expected = transforms(model,start,reference,check)
        pos = np.linalg.norm(actual[:,:3,3]-expected[:,:3,3],axis=1)
        rot = (Rotation.from_matrix(expected[:,:3,:3]).inv()*Rotation.from_matrix(actual[:,:3,:3])).magnitude()
        within = np.max(pos)<=TUBE_M and np.max(rot)<=TUBE_RAD and np.max(np.abs(rows-reference))<=.01
        return rows,ts,actual,pos,rot,within

    interpolation = 'quintic_measured_joint_spline'
    rows,ts,actual_pose,pos_error,rot_error,within = measure(trajectory)
    if not within and selected != original_selected:
        selected = original_selected
        progress = metric[selected]/metric[-1]
        x = betaincinv(3.,3.,np.clip(progress,0.,1.))
        spline = make_interp_spline(x,q[selected],k=5,
            bc_type=([(1,zeros),(2,zeros)],[(1,zeros),(2,zeros)]))
        trajectory = JointTrajectory(joint_names=list(MOVING))
        for t in np.unique(spline.t):
            point = JointTrajectoryPoint(positions=spline(t).tolist(),velocities=spline(t,1).tolist(),
                accelerations=spline(t,2).tolist())
            point.time_from_start.sec,point.time_from_start.nanosec = divmod(round(float(t)*1e9),1000000000)
            trajectory.points.append(point)
        stretch_to_limits(trajectory,vmax,amax,jerk)
        rows,ts,actual_pose,pos_error,rot_error,within = measure(trajectory)
        report['idle_trim']['removed_knots'] = 0
    if parameters.rail_replay_smooth:
        best_duration = time_seconds(trajectory.points[-1]) if within else math.inf
        for candidate,fit in smooth_candidates(MOVING,q[selected],progress,math.degrees(.01),
                counts=(6,10,14,22,34,50,74,110,160,220,300),adaptive_knots=True):
            check()
            stretch_to_limits(candidate,vmax,amax,jerk)
            duration = time_seconds(candidate.points[-1])
            if duration > min(600.,best_duration):continue
            measured = measure(candidate)
            if measured[-1]:
                trajectory = candidate
                rows,ts,actual_pose,pos_error,rot_error,within = measured
                interpolation = 'bounded_continuous_joint_spline'
                report['continuous_fit'] = fit
                best_duration = duration
    if not within:
        report['spline_rejected'] = dict(max_fit_mm=float(np.max(pos_error)*1000),
            max_fit_deg=float(np.degrees(np.max(rot_error))))
        check()
        trajectory = joint_waypoint_trajectory(MOVING,q[selected])
        # Each leg is one exact quintic. Its 20 subdivisions are redundant for
        # timing optimization and turn a small path into a huge dense SLSQP solve.
        trajectory.points = trajectory.points[::20]
        scale_time(trajectory,.1)
        stretch_to_limits(trajectory,vmax,amax,jerk)
        if time_seconds(trajectory.points[-1]) > 600.:
            raise ValueError('贴合原始取点的回退轨迹超过 10 分钟；未执行，原始数据保留')
        rows,ts,actual_pose,pos_error,rot_error,within = measure(trajectory,waypoint=True)
        if not within and selected != original_selected:
            # Trimming is optional: never reject a previously valid path for it.
            selected = original_selected
            progress = metric[selected]/metric[-1]
            trajectory = joint_waypoint_trajectory(MOVING,q[selected])
            trajectory.points = trajectory.points[::20]
            scale_time(trajectory,.1)
            stretch_to_limits(trajectory,vmax,amax,jerk)
            rows,ts,actual_pose,pos_error,rot_error,within = measure(trajectory,waypoint=True)
            report['idle_trim']['removed_knots'] = 0
        if not within:
            raise ValueError('原始取点回退仍超过 0.5 mm / 0.5 度 / 0.01 rad；未执行，原始数据保留')
        interpolation = 'quintic_measured_waypoint_fallback'
    guarded_joints(model, rows, context='Rail replay spline')
    # Reuse D-ring's recorded-speed local retimer; do not impose the old 5 mm/s rail cap.
    from task_trajectory_speed import scale_arm_segments
    baseline_seconds = trim['active_seconds'] if trim else report['recorded_seconds']
    scale_time(trajectory,baseline_seconds/time_seconds(trajectory.points[-1]))
    multiplier = 1e6 if parameters.rail_replay_fastest else parameters.rail_replay_multiplier
    policy = 'local' if parameters.rail_replay_fastest or multiplier > 1. else 'recorded'
    if getattr(parameters,'rail_replay_cruise',False):
        from task_cruise_timing import retime
        curve,timing=retime(trajectory,model,start,'right',multiplier,parameters,
            velocity=VELOCITY_SCALE,acceleration=ACCELERATION_SCALE,check=check,original_policy=policy)
        timed=[dict(trajectory=curve)]
    else:
        timed,timing = scale_arm_segments([dict(label=trace['stage'],trajectory=trajectory)],multiplier,model,parameters,
            velocity=VELOCITY_SCALE,acceleration=ACCELERATION_SCALE,policy=policy,check=check)
    report['speed_request'] = dict(multiplier=parameters.rail_replay_multiplier,
        fastest=parameters.rail_replay_fastest,baseline_seconds=baseline_seconds)
    trajectory = timed[0]['trajectory']
    report['timing'] = {k:v for k,v in timing.items() if k != 'baseline_trajectories'}
    if time_seconds(trajectory.points[-1]) > 600.:
        raise ValueError('重放轨迹超过 10 分钟，记录抖动/转折过多；请分段重录')
    # A bounded, stop-to-stop join keeps the first command continuous with measured hold.
    if max(abs(start[n]-target_start[n]) for n in MOVING) > 1e-9:
        join = bounded_join(model,start,target_start,parameters,check)
        shift = time_seconds(join.points[-1])
        for p in trajectory.points:
            ns = round((time_seconds(p)+shift)*1e9)
            p.time_from_start.sec,p.time_from_start.nanosec = divmod(ns,1000000000)
        trajectory.points = join.points[:-1]+trajectory.points
        report['start_join'] = True
    else:
        report['start_join'] = False
    # Recheck the gateway Hermite curve, including the bounded start join.
    batch = []
    for sample in samples(trajectory, vmax, amax):
        batch.append(sample)
        if len(batch) == 1000:
            check(); guarded_joints(model, batch, context='Rail replay'); batch = []
    if batch:
        guarded_joints(model, batch, context='Rail replay')
    validate_jerk_limits(trajectory, jerk)
    lo,hi = model.limits['waist_yaw_joint']
    validate_waist_envelope(trajectory,lo+JOINT_MARGIN,hi-JOINT_MARGIN,vmax[MOVING[-1]],amax[MOVING[-1]])
    report.update(interpolation=interpolation,
                  max_fit_mm=float(np.max(pos_error)*1000), max_fit_deg=float(np.degrees(np.max(rot_error))),
                  replay_seconds=time_seconds(trajectory.points[-1]))
    return trajectory, report


def required_stages(operation, mode):
    required_state(operation, mode)
    if mode == 'cycle':
        return ['approach_A','push','retreat_B'] if operation == 'push' else ['approach_B','pull','retreat_A']
    if operation == 'push':
        return ['approach_A', 'push']+(['retreat_B'] if mode == 'release' else [])
    return (['approach_B'] if mode == 'release' else [])+['pull', 'retreat_A']


def build_replay(model, config, ready, start, operation, mode, parameters, check=lambda:None,
                 *, cache_dir=None, require_cache=False, progress=lambda message:None):
    stages = required_stages(operation, mode)
    traces = config.get('traces', {})
    if any(k not in traces for k in stages):
        raise ValueError('缺少本次操作所需轨迹段：'+', '.join(STAGES[k][0] for k in stages if k not in traces))
    for k in stages:
        validate_trace(traces[k], k, model, config['hands'])
    if ('pull' in stages and traces['pull'].get('hand_policy') == 'not_recorded' and
            'pull_grip' not in config['hands']):
        raise ValueError('拉出回放前请保存独立拉出手型；轨迹无需重录')
    current, segments = dict(start), []
    def connect(target, label):
        nonlocal current
        progress(label)
        prefix = align(model,current,target,parameters,check,label)
        segments.extend(prefix)
        if prefix:current = dict(prefix[-1]['end'])
    def add(trajectory, end, label, **extra):
        nonlocal current
        segments.append(dict(number=len(segments)+1,kind='arm',side='right',stage='rail',label=label,
            start=dict(current),end=dict(end),trajectory=trajectory,
            duration=time_seconds(trajectory.points[-1]),**extra))
        current = end
    def hand(posture):
        target = checked_q(config['hands'][posture]['feedback_q'])
        end = dict(current, **model_joints('right',target))
        names = list(model_joints('right',target))
        if mode == 'cycle' and max(abs(current[n]-end[n]) for n in names) <= .02:return
        tr = joint_line_trajectory(names,[current[n] for n in names],[end[n] for n in names])
        scale_time(tr,1.)
        add(tr,end,'Rail hand '+posture)
        segments[-1].update(kind='hand',target_q=list(target),posture=posture)
    if mode == 'cycle':
        connect(ready,'移动到右手 above-table')
        hand('grip' if operation == 'push' else 'open')
    elif operation == 'push' or mode == 'release':
        hand('open')
    for stage in stages:
        from task_trace_versions import bank
        selected=bank(config,stage)['selected']
        version_label='蓝本' if selected==0 else f'修改 {selected}'
        progress('处理录制段 '+STAGES[stage][0]+' / '+version_label)
        if mode == 'cycle':
            if stage == 'retreat_A':hand('open')
            connect(recorded_position(config,stage,0),'衔接 '+STAGES[stage][0]+' 起点')
        elif stage in ('push', 'pull'):
            posture = 'pull_grip' if stage == 'pull' and 'pull_grip' in config['hands'] else 'grip'
            if not (stage == 'pull' and mode == 'hold' and posture == 'grip'):
                hand(posture)
        elif stage in ('retreat_A', 'retreat_B'):
            hand('open')
        if mode != 'cycle':
            connect(recorded_position(config,stage,0),'衔接 '+STAGES[stage][0]+' 起点')
        if cache_dir is None:
            progress('拟合及速度规划：'+STAGES[stage][0])
            tr, report = replay(model,traces[stage],current,parameters,check)
        else:
            from task_rail_cache import resolve
            progress('读取缓存 / 拟合及速度规划：'+STAGES[stage][0])
            tr, report = resolve(cache_dir,traces[stage],model,parameters,
                lambda:replay(model,traces[stage],current,parameters,check),check,require_hit=require_cache,context=current)
        end = dict(current, **dict(zip(MOVING,tr.points[-1].positions)))
        add(tr,end,'Rail recorded '+stage,trace_stage=stage,trace_report=report,
            trace_version=dict(index=selected,label=version_label,
                created_at=traces[stage].get('edit_origin',{}).get('created_at'),
                sha256=fingerprint(traces[stage])))
        if mode == 'cycle' and stage == 'approach_B':
            hand('pull_grip' if 'pull_grip' in config['hands'] else 'grip')
        progress('完成录制段：'+STAGES[stage][0])
    final = 'outer' if operation == 'pull' else ('inner_held' if mode == 'hold' else 'inner_released')
    if final != 'inner_held':
        connect(ready,'返回右手 above-table')
    if mode == 'cycle':hand('open')
    for i,segment in enumerate(segments,1):segment['number']=i
    return dict(kind='rail',arm='right',start=dict(start),segments=segments,waist_assist=True,
        hardware_executable=True,collision_checked=False,contact_force_measured=False,
        rail=dict(operation=operation,mode=mode,final_state=final,path_source='recorded'),
        description='Measured joint-path replay; no TCP estimation, vision or contact-force claim')
