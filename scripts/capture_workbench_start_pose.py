"""Read real LowState only. Never publish or take motor ownership."""
import argparse
from datetime import datetime
import json
import os
from pathlib import Path
import shutil
import time

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
PARTS = ('shoulder_pitch', 'shoulder_roll', 'shoulder_yaw', 'elbow',
         'wrist_roll', 'wrist_pitch', 'wrist_yaw')
LEGS = ('hip_pitch', 'hip_roll', 'hip_yaw', 'knee', 'ankle_pitch', 'ankle_roll')
JOINTS = ([f'left_{p}_joint' for p in LEGS] + [f'right_{p}_joint' for p in LEGS]
          + ['waist_yaw_joint', 'waist_roll_joint', 'waist_pitch_joint']
          + [f'left_{p}_joint' for p in PARTS] + [f'right_{p}_joint' for p in PARTS])


def write_json(path, data):
    temporary = path.with_suffix(path.suffix + '.tmp')
    temporary.write_text(json.dumps(data, indent=2, allow_nan=False) + '\n')
    temporary.replace(path)


def validate_window(samples, age):
    if len(samples) < 50 or age > .2:
        raise ValueError('Insufficient fresh real LowState samples')
    if any(s['mode_machine'] != 6 or s['mode_pr'] != 0 for s in samples):
        raise ValueError('Unexpected machine/joint interpretation mode')
    ticks = [s['tick'] for s in samples]
    if len(set(ticks)) < 50:
        raise ValueError('LowState tick is not advancing')
    q = np.asarray([s['q'] for s in samples])
    dq = np.asarray([s['dq'] for s in samples])
    if not np.isfinite(q).all() or not np.isfinite(dq).all():
        raise ValueError('Nonfinite joint data')
    active = list(range(13)) + list(range(15, 29))
    if any(any(s['vol'][i] <= 1 or s['mode'][i] != 1 for i in active) for s in samples):
        raise ValueError('Required physical joint lacks active telemetry')
    upper = [12] + list(range(15, 29))
    spread = float(np.ptp(q[:, upper], axis=0).max())
    peak_dq = float(np.abs(dq[:, upper]).max())
    if spread > .025 or peak_dq > .15:
        raise ValueError(f'Upper body not stationary: spread={spread}, dq={peak_dq}')
    return dict(samples=len(samples),tick_first=ticks[0],tick_last=ticks[-1],
                last_receive_age_s=age,upper_body_q_span_rad=spread,
                upper_body_max_abs_dq_rad_s=peak_dq)


def main():
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from rosidl_runtime_py.convert import message_to_ordereddict
    from unitree_hg.msg import LowState

    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--set-start', action='store_true')
    parser.add_argument('--label', default='startup_reference')
    args = parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID') != '0' or os.environ.get('ROS_LOCALHOST_ONLY') == '1':
        raise RuntimeError('Requires real-robot domain 0, not offline domain 77')
    args.output.mkdir(parents=True, exist_ok=True)
    if (args.output / 'startup_pose.json').exists():
        raise ValueError('Refusing to overwrite an existing capture')
    samples = []; latest = {}
    rclpy.init(); node = rclpy.create_node('g1_readonly_startup_capture')

    def body_callback(msg):
        now = time.monotonic()
        samples.append(dict(t=now, tick=int(msg.tick), mode_machine=int(msg.mode_machine),
            mode_pr=int(msg.mode_pr),q=[float(m.q) for m in msg.motor_state[:29]],
            dq=[float(m.dq) for m in msg.motor_state[:29]],
            vol=[float(m.vol) for m in msg.motor_state[:29]],
            mode=[int(m.mode) for m in msg.motor_state[:29]]))
        latest.update(received=now, msg=msg)

    node.create_subscription(LowState, '/lowstate', body_callback, qos_profile_sensor_data)
    started = time.monotonic()
    try:
        while time.monotonic() - started < 15:
            rclpy.spin_once(node, timeout_sec=.05)
            if samples and time.monotonic() - samples[0]['t'] >= 2.:
                break
        now = time.monotonic()
        quality = validate_window(samples, now - latest.get('received', started))
        raw = message_to_ordereddict(latest['msg'])
        profile = dict(purpose='Measured, user-confirmed motion-ready startup reference',
            pose_label=args.label,
            source='Real /lowstate, domain 0, eth0; final coherent frame of stationary window',
            captured_at=datetime.now().astimezone().isoformat(),r2_a_pose_verified=False,
            operator_confirmed_motion_ready=True,quality=quality,
            joints=dict(zip(JOINTS,samples[-1]['q'])),
            measured_joints=[JOINTS[i] for i,m in enumerate(raw['motor_state'][:29]) if m['vol']>1],
            unmeasured_locked_joint_assumptions={JOINTS[i]:0. for i in (13,14) if raw['motor_state'][i]['vol']<=1},
            validation_scope='Stationary joint capture only; world/table registration and collision checks pending',
            hand_startup_posture='open',
            hand_geometry_note='Both hands default open by operator instruction; no hand actuator positions saved or commanded.',
            base_pose_note='LowState IMU is retained as evidence, not treated as calibrated task-world pose')
        write_json(args.output/'lowstate.json',raw)
        write_json(args.output/'sampling_window.json',samples)
        write_json(args.output/'startup_pose.json',profile)
        if args.set_start:
            target=ROOT/'assets/workbench/task_start_pose.json'
            if (ROOT/'assets/workbench/task/initial_pose.json').exists():
                raise ValueError('Another initial_pose override exists; capture saved, default unchanged')
            shutil.copy2(target,args.output/'previous_task_start_pose.json')
            write_json(target,profile)
        print(json.dumps(dict(output=str(args.output),startup_reference_updated=args.set_start,
            quality=quality,
            upper_body_max_temperature_c=max(max(m['temperature']) for m in raw['motor_state'][15:29])),indent=2))
    finally:
        node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':main()
