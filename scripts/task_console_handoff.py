"""Local console attachment data; never creates or re-arms a controller."""
import fcntl
import json
import math
import os
from pathlib import Path
import time

from task_session import ARMS, ARM_NAMES, RobotMoving


def process_identity(pid):
    root = Path('/proc')/str(int(pid))
    if root.stat().st_uid != os.getuid():
        raise RuntimeError('Controller process belongs to another user')
    fields = (root/'stat').read_text().rsplit(')', 1)[1].split()
    return {'pid':int(pid), 'start_ticks':int(fields[19])}


def acquire_console_lock():
    stream = open(f'/tmp/g1-task-console-{os.getuid()}.lock', 'a')
    try:
        fcntl.flock(stream, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except OSError:
        stream.close()
        raise RuntimeError('Another task console is active; do not open a second command UI')
    return stream


def validate_handoff(data, now=None):
    now = time.time() if now is None else now
    created = float(data['created_at_epoch_s'])
    # Panel-only restoration grants no hold/motion authority; wall-clock changes
    # must not prevent opening the UI. Guarded hold handoffs retain their TTL.
    if not math.isfinite(created) or (data.get('attachment_mode')!='panel' and not 0 <= now-created < 120):
        raise RuntimeError('Console handoff expired')
    if data['gateway_identity'] != process_identity(data['gateway_identity']['pid']):
        raise RuntimeError('The original gateway has exited or restarted')
    record = data['session_home']
    if record is None and data.get('startup_capture_pending') is True:
        home = None
    elif record is None:
        raise RuntimeError('Handoff lacks a confirmed home or explicit pending capture')
    elif not (record.get('operator_confirmed_r2_a') is True or
              (record.get('operator_confirmed_hold_reference') is True and
               record.get('capture_mode') == 'ARMED_HOLD')):
        raise RuntimeError('Handoff lacks the original confirmed startup pose')
    else:
        home = record['joints']
        if not isinstance(home,dict):
            raise RuntimeError('Invalid startup joints in console handoff')
    if home is not None and (set(home) not in (set(ARM_NAMES),set(ARM_NAMES)|{'waist_yaw_joint'}) or
                            not all(math.isfinite(float(v)) for v in home.values())):
        raise RuntimeError('Invalid startup joints in console handoff')
    for side, target in data.get('ready_poses', {}).items():
        if side not in ARMS:
            raise RuntimeError('Invalid preparation arm in console handoff')
        if target is not None and not all(n in target['joints'] and math.isfinite(float(target['joints'][n]))
                                          for n in ARMS[side]):
            raise RuntimeError('Incomplete preparation pose in console handoff')
    if type(data['stop_epoch']) is not int or data['stop_epoch'] < 0:
        raise RuntimeError('Invalid stop epoch')
    if data.get('attachment_mode','hold') not in ('hold','stopped','panel'):
        raise RuntimeError('Invalid console attachment mode')
    return data


def load_handoff(path):
    return validate_handoff(json.loads(Path(path).read_text()))


def supported_controller(gateway, context='Task execution'):
    required = {'armed':'true', 'hardware_output_available':'true',
                'task_application_enabled':'true', 'upper_body_state_safe':'true',
                'service_state_ready':'true', 'mode_machine':'6', 'mode_pr':'0'}
    if any(gateway.get(k) != v for k,v in required.items()):
        raise RuntimeError(context+' requires the same healthy supported controller')
    for key in ('torque_overload_active','command_overload_active','velocity_recovery_active',
                'temperature_alarm_active','feedforward_saturated','service_conflict'):
        if gateway.get(key) != 'false':
            raise RuntimeError(context+' refused: '+key)
    velocity = float(gateway['max_measured_velocity_rad_s'])
    if not math.isfinite(velocity):
        raise RuntimeError('Controller measured velocity is nonfinite')
    return velocity


def supported_gateway(gateway):
    velocity=supported_controller(gateway,context='Handoff')
    if velocity > .05:
        raise RobotMoving('Handoff requires stationary measured joints')
