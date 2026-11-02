"""Shared guarded timing for live plans, saved execution and RViz; no robot output."""
import copy
import math

import numpy as np

from task_trajectory import (coefficients, time_seconds, task_motion_limits, task_jerk_limits,
                             stretch_to_limits, validate_jerk_limits, validate_waist_envelope, peak,
                             bernstein_controls)


def rescale(trajectory, multiplier, *, require_derivatives=True):
    if not math.isfinite(multiplier) or multiplier <= 0:
        raise ValueError('Invalid time multiplier')
    if not trajectory.joint_names or len(set(trajectory.joint_names)) != len(trajectory.joint_names):
        raise ValueError('Expected unique trajectory joints')
    result = copy.deepcopy(trajectory)
    previous = -1
    for point in result.points:
        if point.time_from_start.sec < 0 or not 0 <= point.time_from_start.nanosec < 1000000000:
            raise ValueError('Invalid ROS timestamp')
        count = len(result.joint_names)
        if (len(point.positions) != count or not np.isfinite(point.positions).all() or
                any((len(values) != count and (require_derivatives or len(values))) or
                    not np.isfinite(values).all() for values in (point.velocities, point.accelerations))):
            raise ValueError('Complete finite trajectory derivatives required')
        seconds = time_seconds(point)/multiplier
        if not math.isfinite(seconds) or seconds >= 2**31:
            raise ValueError('Requested speed exceeds ROS timestamp range')
        ns = round(seconds*1e9)
        if ns <= previous or ns < 0 or ns >= (2**31)*1000000000:
            raise ValueError('Time scaling collapses trajectory timestamps')
        previous = ns
        point.time_from_start.sec, point.time_from_start.nanosec = divmod(ns, 1000000000)
        point.velocities = [value*multiplier for value in point.velocities]
        point.accelerations = [value*multiplier*multiplier for value in point.accelerations]
        if not all(math.isfinite(v) for v in (*point.velocities, *point.accelerations)):
            raise ValueError('Scaled derivatives are not finite')
    if len(result.points) < 2 or time_seconds(result.points[0]) != 0:
        raise ValueError('A zero-based nonempty trajectory is required')
    return result


def preview_segments(segments, multiplier, *, check=lambda: None):
    """Exact requested time scaling for visualization, explicitly NOT executable."""
    if not math.isfinite(multiplier) or multiplier <= 0 or not segments:
        raise ValueError('A positive finite multiplier and nonempty trajectory are required')
    result = copy.deepcopy(segments)
    error = 0.
    path_error = 0.
    peaks = {}
    for old, new in zip(segments, result):
        check()
        kind = old.get('kind', 'arm')
        if kind not in ('arm', 'hand'):
            raise ValueError('Unknown trajectory kind')
        trajectory = rescale(old['trajectory'], multiplier, require_derivatives=kind == 'arm')
        for a, b, c, d in zip(old['trajectory'].points, old['trajectory'].points[1:],
                              trajectory.points, trajectory.points[1:]):
            check()
            before = coefficients(a, b, time_seconds(b)-time_seconds(a))
            dt = time_seconds(d)-time_seconds(c)
            after = coefficients(c, d, dt)
            error = max(error, float(np.max(np.abs(before-after))))
            path_error = max(path_error, max(float(peak(values)) for values in (before-after).T))
            if path_error > 1e-7:
                raise ValueError('Requested speed exceeds timestamp precision for this path')
            if kind == 'arm':
                for order, key in ((1, 'velocity_rad_s'), (2, 'acceleration_rad_s2'), (3, 'jerk_rad_s3')):
                    derivative = np.polynomial.polynomial.polyder(after, order)/dt**order
                    for index, name in enumerate(trajectory.joint_names):
                        record = peaks.setdefault(name, {})
                        record[key] = max(record.get(key, 0.), float(peak(derivative[:, index])))
        new['trajectory'] = trajectory
        new['duration'] = time_seconds(trajectory.points[-1])
    return result, dict(mode='unrestricted_rviz_only', requested_multiplier=multiplier,
        applied_multiplier=multiplier, limited=False, max_polynomial_difference=error,
        max_joint_path_difference_rad=path_error,
        duration_before_s=sum(time_seconds(s['trajectory'].points[-1]) for s in segments),
        duration_after_s=sum(time_seconds(s['trajectory'].points[-1]) for s in result),
        hand_timing_preserved=False, joint_peaks=peaks, limits_enforced=False,
        hardware_executable=False, hardware_commands_sent=0)


def motion_limits(trajectory, model, velocity, acceleration, parameters):
    vmax, amax = task_motion_limits(trajectory.joint_names, velocity, acceleration, parameters)
    for name in trajectory.joint_names:
        limit = model.robot.find(f"joint[@name='{name}']/limit")
        value = float(limit.get('velocity', 'nan')) if limit is not None else float('nan')
        if not math.isfinite(value) or value <= 0:
            raise ValueError('Missing model velocity limit: '+name)
        vmax[name] = min(vmax[name], value)
    return vmax, amax


def speed_capacity(trajectory, model, parameters=None, *, velocity=1., acceleration=1.,
                   check=lambda: None):
    """Uniform speed headroom, using the same interpolation/enclosures as execution."""
    trajectory = rescale(trajectory, 1.)
    vmax, amax = motion_limits(trajectory, model, velocity, acceleration, parameters)
    jerk = task_jerk_limits(trajectory.joint_names)
    maximum = math.inf
    limiting = None
    for interval, (a, b) in enumerate(zip(trajectory.points, trajectory.points[1:])):
        check()
        dt = time_seconds(b)-time_seconds(a)
        c = coefficients(a, b, dt)
        for order, key, limits in ((1, 'velocity', vmax), (2, 'acceleration', amax), (3, 'jerk', jerk)):
            derivative = np.polynomial.polynomial.polyder(c, order)/dt**order
            envelope = np.max(np.abs(bernstein_controls(derivative)), axis=0)
            for axis, name in enumerate(trajectory.joint_names):
                bound = float(limits[name])
                if not math.isfinite(bound) or bound <= 0:
                    raise ValueError('Invalid motion limit: '+name)
                value = float(envelope[axis] if name == 'waist_yaw_joint' and order < 3
                              else peak(derivative[:, axis]))
                if not math.isfinite(value):
                    raise ValueError('Nonfinite trajectory derivative')
                candidate = (bound/value)**(1./order) if value > 0 else math.inf
                if candidate < maximum:
                    maximum = candidate
                    limiting = dict(joint=name, constraint=key, interval=interval,
                                    peak_at_1x=value, limit=bound)
    # Match stretch_to_limits' 0.5% timing margin, including when requesting the maximum.
    return dict(maximum_multiplier=maximum/1.005 if math.isfinite(maximum) else None,
                limiting_constraint=limiting)


def _uniform_arm_segments(segments, multiplier, model, parameters=None, *, velocity=1.,
                       acceleration=1., check=lambda: None):
    if (not math.isfinite(multiplier) or multiplier <= 0 or
            any(not math.isfinite(v) or not .05 <= v <= 1. for v in (velocity, acceleration))):
        raise ValueError('Invalid trajectory multiplier or motion-limit scaling')
    arms = [s for s in segments if s.get('kind', 'arm') == 'arm']
    if not segments or any(s.get('kind', 'arm') not in ('arm', 'hand') for s in segments):
        raise ValueError('Expected nonempty arm/hand segments')
    stage_limits = []
    maximum = math.inf
    limiting = None
    for segment in arms:
        check()
        capacity = speed_capacity(segment['trajectory'], model, parameters,
                                  velocity=velocity, acceleration=acceleration, check=check)
        cap = capacity['maximum_multiplier']
        label = segment.get('label', 'arm')
        stage_limits.append(dict(label=label, **capacity,
                                 requested_time_stretch=max(1., multiplier/cap) if cap is not None else 1.))
        if cap is not None and cap < maximum:
            maximum = cap
            limiting = dict(capacity['limiting_constraint'], label=label)
    applied = float(min(multiplier, maximum)) if arms else 1.
    result = copy.deepcopy(segments)
    error = 0.
    for old, new in zip(segments, result):
        check()
        if old.get('kind', 'arm') == 'hand':
            continue
        trajectory = rescale(old['trajectory'], applied)
        vmax, amax = motion_limits(trajectory, model, velocity, acceleration, parameters)
        jerk = task_jerk_limits(trajectory.joint_names)
        # Recheck rounded ROS timestamps; never silently apply a different per-stage factor.
        if stretch_to_limits(copy.deepcopy(trajectory), vmax, amax, jerk) > 1.+1e-6:
            raise ValueError('Rounded time scaling exceeds motion limits')
        validate_jerk_limits(trajectory, jerk)
        if 'waist_yaw_joint' in trajectory.joint_names:
            lo, hi = model.limits['waist_yaw_joint']
            validate_waist_envelope(trajectory, lo+.02, hi-.02,
                                   vmax['waist_yaw_joint'], amax['waist_yaw_joint'])
        for a, b, c, d in zip(old['trajectory'].points, old['trajectory'].points[1:],
                              trajectory.points, trajectory.points[1:]):
            before = coefficients(a, b, time_seconds(b)-time_seconds(a))
            after = coefficients(c, d, time_seconds(d)-time_seconds(c))
            error = max(error, float(np.max(np.abs(before-after))))
        if error > 1e-7:
            raise ValueError('Time scaling changed the normalized interpolation curve')
        new['trajectory'] = trajectory
        new['duration'] = time_seconds(trajectory.points[-1])
    return result, dict(mode='guarded', timing_policy='uniform', limits_enforced=True,
        requested_multiplier=multiplier, applied_multiplier=applied,
        maximum_multiplier=maximum if math.isfinite(maximum) else None,
        limiting_constraint=limiting, timing_margin=1.005,
        minimum_duration_s=(sum(time_seconds(s['trajectory'].points[-1])/
            (maximum if s.get('kind','arm')=='arm' else 1.) for s in segments)
            if math.isfinite(maximum) else None),
        limited=applied < multiplier-1e-6, max_polynomial_difference=error,
        duration_before_s=sum(time_seconds(s['trajectory'].points[-1]) for s in segments),
        duration_after_s=sum(time_seconds(s['trajectory'].points[-1]) for s in result),
        hand_timing_preserved=True, velocity_limit_scaling=velocity,
        acceleration_limit_scaling=acceleration, stage_limits=stage_limits, hardware_commands_sent=0)


def restore_baseline(segments, report):
    """Local timing is not invertible by a single multiplier. Retain the exact source."""
    from rosidl_runtime_py.set_message import set_message_fields
    result = copy.deepcopy(segments)
    baseline = report.get('baseline_trajectories')
    if report.get('timing_policy') in ('local_path_v1','recorded_local_v1','cruise_local_v2'):
        if not isinstance(baseline, list) or len(baseline) != len(result):
            raise ValueError('Missing original local-timing baseline; reload the archive')
        for segment, original in zip(result, baseline):
            set_message_fields(segment['trajectory'], copy.deepcopy(original))
            segment['duration'] = time_seconds(segment['trajectory'].points[-1])
    else:
        factor = report.get('applied_multiplier', 1.)
        for segment in result:
            arm = segment.get('kind', 'arm') == 'arm'
            if arm or not report.get('hand_timing_preserved', True):
                segment['trajectory'] = rescale(segment['trajectory'], 1./factor, require_derivatives=arm)
                segment['duration'] = time_seconds(segment['trajectory'].points[-1])
    return result


def scale_arm_segments(segments, multiplier, model, parameters=None, *, velocity=1.,
                       acceleration=1., check=lambda: None, policy='local'):
    if policy=='recorded':
        return _recorded_arm_segments(segments,multiplier,model,parameters,
            velocity=velocity,acceleration=acceleration,check=check)
    if policy == 'uniform':
        return _uniform_arm_segments(segments, multiplier, model, parameters,
                                     velocity=velocity, acceleration=acceleration, check=check)
    if policy != 'local':
        raise ValueError('Unknown timing policy')
    if (not math.isfinite(multiplier) or multiplier <= 0 or
            any(not math.isfinite(v) or not .05 <= v <= 1. for v in (velocity, acceleration)) or
            not segments or any(s.get('kind', 'arm') not in ('arm', 'hand') for s in segments)):
        raise ValueError('Invalid trajectory multiplier, motion limits or stages')
    from rosidl_runtime_py.convert import message_to_ordereddict
    from task_local_timing import fastest_trajectory
    result = copy.deepcopy(segments)
    stage_limits = []
    total_before = total_after = fastest_total = hand_duration = 0.
    maximum = 0.
    limiting = None
    path_error = 0.
    limited = False
    for old, new in zip(segments, result):
        check()
        original = old['trajectory']
        duration = time_seconds(original.points[-1])
        if old.get('kind', 'arm') == 'hand':
            hand_duration += duration
            continue
        capacity = speed_capacity(original, model, parameters, velocity=velocity,
                                  acceleration=acceleration, check=check)
        if any(abs(value)>1e-6 for p in (original.points[0],original.points[-1])
               for values in (p.velocities,p.accelerations) for value in values):
            raise ValueError('Independent arm stages must have stationary boundaries')
        uniform = capacity['maximum_multiplier']
        vmax, amax = motion_limits(original, model, velocity, acceleration, parameters)
        jerk = task_jerk_limits(original.joint_names)
        detail = dict(method='uniform_fallback')
        fastest = rescale(original, uniform if uniform is not None else multiplier)
        if uniform is not None:
            try:
                candidate, detail = fastest_trajectory(original, vmax, amax, jerk, uniform, check)
                if time_seconds(candidate.points[-1]) < time_seconds(fastest.points[-1])-1e-6:
                    fastest = candidate
                    detail['method'] = 'continuous_local'
                else:
                    detail = dict(method='uniform_fallback', reason='No verified local improvement')
            except (ValueError, np.linalg.LinAlgError) as error:
                detail = dict(method='uniform_fallback', reason=str(error))
        fastest_duration = time_seconds(fastest.points[-1])
        if detail.get('method')=='uniform_fallback':
            uniform_error=0.
            for a,b,c,d in zip(original.points,original.points[1:],fastest.points,fastest.points[1:]):
                difference=coefficients(a,b,time_seconds(b)-time_seconds(a))-coefficients(c,d,time_seconds(d)-time_seconds(c))
                uniform_error=max(uniform_error,float(np.max(np.abs(bernstein_controls(difference)))))
            detail['max_joint_path_difference_rad']=uniform_error
        cap = duration/fastest_duration
        applied = min(multiplier, cap)
        trajectory = rescale(fastest, applied/cap)
        if stretch_to_limits(copy.deepcopy(trajectory), vmax, amax, jerk) > 1.+1e-6:
            raise ValueError('Rounded local timing exceeds motion limits')
        validate_jerk_limits(trajectory, jerk)
        if any(abs(value)>1e-6 for p in (trajectory.points[0],trajectory.points[-1])
               for values in (p.velocities,p.accelerations) for value in values):
            raise ValueError('Retimed stage boundary is not stationary')
        if 'waist_yaw_joint' in trajectory.joint_names:
            lo, hi = model.limits['waist_yaw_joint']
            validate_waist_envelope(trajectory, lo+.02, hi-.02, vmax['waist_yaw_joint'], amax['waist_yaw_joint'])
        rounding_error = 0.
        for a, b, c, d in zip(fastest.points, fastest.points[1:], trajectory.points, trajectory.points[1:]):
            check()
            difference = coefficients(a, b, time_seconds(b)-time_seconds(a))-coefficients(c, d, time_seconds(d)-time_seconds(c))
            rounding_error = max(rounding_error, float(np.max(np.abs(bernstein_controls(difference)))))
        error = detail.get('max_joint_path_difference_rad', 0.)+rounding_error
        if error > 1e-7:
            raise ValueError('Local timing exceeds certified joint-path difference')
        path_error = max(path_error, error)
        new['trajectory'] = trajectory
        new['duration'] = time_seconds(trajectory.points[-1])
        label = old.get('label', 'arm')
        stage_limits.append(dict(label=label, maximum_multiplier=cap if uniform is not None else None,
            uniform_maximum_multiplier=uniform, applied_multiplier=applied,
            duration_before_s=duration, duration_after_s=new['duration'], minimum_duration_s=fastest_duration,
            limiting_constraint=capacity['limiting_constraint'], local_timing=detail))
        if uniform is not None and cap > maximum:
            maximum = cap
            limiting = dict(capacity['limiting_constraint'], label=label)
        limited |= applied < multiplier-1e-6
        total_before += duration
        total_after += new['duration']
        fastest_total += fastest_duration
    return result, dict(mode='guarded', timing_policy='local_path_v1', limits_enforced=True,
        requested_multiplier=multiplier, applied_multiplier=total_before/total_after if total_after else 1.,
        maximum_multiplier=maximum or None,
        maximum_effective_multiplier=total_before/fastest_total if fastest_total else 1.,
        maximum_is_verified_not_global_optimum=True, limiting_constraint=limiting,
        timing_margin=1.005, limited=bool(limited),
        minimum_duration_s=fastest_total+hand_duration if maximum else None,
        duration_before_s=total_before+hand_duration, duration_after_s=total_after+hand_duration,
        max_joint_path_difference_rad=path_error, hand_timing_preserved=True,
        velocity_limit_scaling=velocity, acceleration_limit_scaling=acceleration,
        stage_limits=stage_limits, hardware_commands_sent=0,
        baseline_trajectories=[message_to_ordereddict(s['trajectory']) for s in segments])


def _recorded_arm_segments(segments,multiplier,model,parameters,*,velocity,acceleration,check):
    """Cap local rates at the requested recording speed; never speed up easy spans."""
    from task_local_timing import fastest_trajectory
    from rosidl_runtime_py.convert import message_to_ordereddict
    # The uniform result is also a fully validated fallback if local fitting fails.
    fallback,base=_uniform_arm_segments(segments,multiplier,model,parameters,
        velocity=velocity,acceleration=acceleration,check=check)
    if not base['limited']:return fallback,base
    result=copy.deepcopy(segments);details=[];path_error=0.
    for old,new,safe in zip(segments,result,fallback):
        check()
        if old.get('kind','arm')!='arm':continue
        original=old['trajectory']
        vmax,amax=motion_limits(original,model,velocity,acceleration,parameters)
        jerk=task_jerk_limits(original.joint_names)
        cap=speed_capacity(original,model,parameters,velocity=velocity,acceleration=acceleration,check=check)['maximum_multiplier']
        try:
            tr,detail=fastest_trajectory(original,vmax,amax,jerk,min(multiplier,cap or multiplier),
                                         check,rate_ceiling=multiplier)
            if stretch_to_limits(copy.deepcopy(tr),vmax,amax,jerk)>1.+1e-6:
                raise ValueError('Rounded recorded local timing exceeds motion limits')
            validate_jerk_limits(tr,jerk)
            if 'waist_yaw_joint' in tr.joint_names:
                lo,hi=model.limits['waist_yaw_joint']
                validate_waist_envelope(tr,lo+.02,hi-.02,vmax['waist_yaw_joint'],amax['waist_yaw_joint'])
            if time_seconds(tr.points[-1])>safe['duration']+1e-6:
                raise ValueError('Local timing did not improve the verified uniform fallback')
        except (ValueError,np.linalg.LinAlgError) as error:
            tr=safe['trajectory']
            detail=dict(method='uniform_fallback',reason=str(error),rate_min=base['applied_multiplier'],
                        rate_max=base['applied_multiplier'],max_joint_path_difference_rad=base['max_polynomial_difference'])
        new['trajectory']=tr;new['duration']=time_seconds(tr.points[-1])
        details.append(dict(label=old.get('label','arm'),**detail))
        path_error=max(path_error,detail['max_joint_path_difference_rad'])
    duration=sum(s['duration'] for s in result)
    return result,dict(base,timing_policy='recorded_local_v1',applied_multiplier=base['duration_before_s']/duration,
        duration_after_s=duration,maximum_multiplier=None,minimum_duration_s=None,
        rate_min=min(d['rate_min'] for d in details),rate_max=max(d['rate_max'] for d in details),
        stage_limits=details,max_joint_path_difference_rad=path_error,
        baseline_trajectories=[message_to_ordereddict(s['trajectory']) for s in segments])
