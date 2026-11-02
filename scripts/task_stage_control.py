"""Frozen stage settings and exact time stretching; no robot I/O."""
import math
from dataclasses import replace

# Numeric representation of LowCmd gains, not a controller tuning ceiling.
STAGE_GAIN_FLOAT_MAX=float.fromhex('0x1.fffffep+127')


def gateway_gain_limit(raw):
    if raw=='unbounded':return math.inf
    try:limit=float(raw)
    except (TypeError,ValueError):raise RuntimeError('Invalid gateway stage gain limits') from None
    if not math.isfinite(limit) or limit<0:
        raise RuntimeError('Invalid gateway stage gain limits')
    return limit


def replay_arm_kp(parameters,side):
    """Resolve per-axis recorded replay overrides without changing old snapshots."""
    if side not in ('left','right'):raise ValueError('Unknown replay arm')
    joints=('shoulder_pitch','shoulder_roll','shoulder_yaw','elbow','wrist_roll','wrist_pitch','wrist_yaw')
    values=[getattr(parameters,'replay_'+joint+'_kp') for joint in joints]
    if all(v==-1 for v in values):return None
    fallback=[parameters.withdraw_proximal_kp]*4
    for joint in joints[4:]:
        gain=parameters.demo_distal_wrist_kp if joint!='wrist_roll' else 0.
        fallback.append(gain or parameters.demo_wrist_kp or getattr(parameters,side+'_'+joint+'_kp'))
    return [float(base if value==-1 else value) for value,base in zip(values,fallback)]


def replay_stage_parameters(parameters):
    """Resolve recorded replay's stage override, leaving its frozen snapshot intact."""
    kp=parameters.replay_waist_kp
    if kp==-1:return parameters
    kd=(parameters.stage_waist_kd or parameters.waist_kd) if kp else 0.
    return replace(parameters,stage_waist_kp=kp,stage_waist_kd=kd)


def validate_gateway_gains(parameters,gateway,*,wrist_kp=0.,distal_wrist_kp=0.,arm_kp=None):
    if gateway.get('task_stage_supported')!='true':
        raise RuntimeError('Stage gains require an updated controller; restart and replan before motion')
    if arm_kp is not None:
        if len(arm_kp)!=7 or any(not math.isfinite(v) or not 0<=v<=STAGE_GAIN_FLOAT_MAX for v in arm_kp):
            raise RuntimeError('Replay arm Kp requires seven nonnegative finite LowCmd gains')
        if gateway.get('task_stage_arm_kp_supported')!='true':
            raise RuntimeError('Per-joint replay gains require the updated gateway; restart controller and panel')
    if not math.isfinite(wrist_kp) or not 0<=wrist_kp<=110:
        raise RuntimeError('Replay wrist Kp must be in [0,110]')
    if wrist_kp and gateway.get('task_stage_wrist_supported')!='true':
        raise RuntimeError('Replay wrist gains require the updated gateway; restart controller and panel')
    if not math.isfinite(distal_wrist_kp) or not 0<=distal_wrist_kp<=220:
        raise RuntimeError('Replay distal wrist Kp must be in [0,220]')
    if distal_wrist_kp and gateway.get('task_stage_distal_wrist_supported')!='true':
        raise RuntimeError('Replay distal wrist gains require the updated gateway; restart controller and panel')
    for gain,legacy_limit in (('kp',80.),('kd',5.)):
        limit=gateway_gain_limit(gateway.get('task_stage_max_'+gain,legacy_limit))
        for profile in ('rotate','withdraw'):
            requested=float(getattr(parameters,profile+'_proximal_'+gain))
            if not math.isfinite(requested) or not 0<=requested<=STAGE_GAIN_FLOAT_MAX:
                raise RuntimeError('Stage gains must be nonnegative finite values representable in LowCmd')
            if requested>limit:
                raise RuntimeError(f'{profile} {gain}={requested:g} exceeds running gateway limit {limit:g}; '
                                   'restart the updated controller and panel, then replan')
    if parameters.stage_waist_kp:
        if (gateway.get('task_stage_waist_supported')!='true' or
                gateway.get('task_waist_trajectory_supported')!='true' or
                gateway.get('waist_control_mode')!='guarded'):
            raise RuntimeError('Stage waist gains require an updated gateway with task waist enabled in guarded mode')
        for gain in ('kp','kd'):
            limit=gateway_gain_limit(gateway.get('task_stage_waist_max_'+gain,'nan'))
            value=getattr(parameters,'stage_waist_'+gain)
            if not math.isfinite(value) or not 0<value<=min(limit,STAGE_GAIN_FLOAT_MAX):
                raise RuntimeError('Stage waist gains exceed running gateway limits; restart and replan')


def stage_name(label):
    if label in ('Left 120','Right 120'):return 'rotate'
    if label=='Withdraw':return 'withdraw'
    return 'normal'


def recorded_replay_segment(kind,segment):
    return segment.get('kind','arm')=='arm' and (
        kind=='dring_demo' and segment.get('recording_fit') is not None or
        kind=='rail' and segment.get('trace_stage') is not None)


def segment_stage(kind,segment):
    if kind in ('dring_demo','rail'):
        return 'withdraw' if recorded_replay_segment(kind,segment) else 'normal'
    return stage_name(segment['label'])


def slow_segment(segment,factor):
    if not math.isfinite(factor) or not .1<=factor<=1:
        raise ValueError('Stage speed factor must be in [0.1,1]')
    if factor==1:return
    trajectory=segment['trajectory']
    for point in trajectory.points:
        ns=round((point.time_from_start.sec*1000000000+point.time_from_start.nanosec)/factor)
        point.time_from_start.sec,point.time_from_start.nanosec=divmod(ns,1000000000)
        point.velocities=[v*factor for v in point.velocities]
        point.accelerations=[v*factor*factor for v in point.accelerations]
    last=trajectory.points[-1].time_from_start
    segment['duration']=last.sec+last.nanosec*1e-9
    segment['stage_velocity_factor']=factor
