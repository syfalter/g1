"""Summarize archived native CSV telemetry without opening a controller."""
import argparse
import csv
import json
from pathlib import Path

import numpy as np


def longest_interval(mask, times):
    boundaries = np.diff(np.r_[False, mask, False].astype(int))
    spans = [(times[min(end, len(times)-1)] - times[start])
             for start, end in zip(np.flatnonzero(boundaries == 1), np.flatnonzero(boundaries == -1))]
    return float(max(spans, default=0.))


def summarize(path):
    with path.open() as stream:
        rows = list(csv.DictReader(stream))
    rows = [r for r in rows if r['event'] in ('sample', 'settling_hold')]
    times = np.array([float(r['ros_time_s']) for r in rows])
    names = [key[:-7] for key in rows[0] if key.endswith('_target')]
    joints = {}
    array = lambda key: np.array([float(r[key]) for r in rows])
    for name in names:
        error = array(name+'_q') - array(name+'_target')
        speed = array(name+'_dq')
        torque = array(name+'_tau_est')
        index = int(np.argmax(abs(speed)))
        joints[name] = dict(peak_velocity_rad_s=float(max(abs(speed))),
            p99_velocity_rad_s=float(np.quantile(abs(speed), .99)),
            peak_velocity_progress_s=float(rows[index]['trajectory_progress_s']),
            longest_above_05_rad_s=longest_interval(abs(speed) > .5, times),
            peak_command_velocity_rad_s=float(max(abs(array(name+'_command_dq')))),
            peak_torque_nm=float(max(abs(torque))),
            peak_tracking_error_rad=float(max(abs(error))),
            p95_tracking_error_rad=float(np.quantile(abs(error), .95)),
            final_tracking_error_rad=float(error[-1]))
    return dict(file=str(path), samples=len(rows), wall_seconds=float(times[-1]-times[0]),
        progress_seconds=float(max(array('trajectory_progress_s'))),
        min_time_scale=float(min(float(r['time_scale']) for r in rows if r['event']=='sample')),
        max_control_gap_ms=float(max(array('control_gap_ms'))),
        max_feedback_age_ms=float(max(array('lowstate_steady_age_ms'))), joints=joints,
        note='Temperature, audio, measured tool forces and absolute base pose are not in this CSV.')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('log', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    report = summarize(args.log)
    args.output.write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
