"""Exercise edited real recordings offline, without creating ROS nodes."""
import argparse
import json
from pathlib import Path
import time

import numpy as np

from task_dring_teach_data import arrays, fit_trace
from task_trace_edit import smooth
from task_trace_selection import restore
from task_trajectory_speed import speed_capacity
from task_planning_config import load_parameters
from task_screen_teach import write_json
from workbench_model import OfflineModel

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('robot', type=Path)
    parser.add_argument('--output', type=Path, default=ROOT/'data/trajectory_edit_audit/smoothing.json')
    args = parser.parse_args()
    model = OfflineModel(args.robot)
    params, _ = load_parameters()
    report = dict(hardware_commands_sent=0, candidates=[])
    for filename in ('dring_demonstration.json', 'landing_gear_demonstration.json'):
        path=ROOT/'data/task_panel'/filename
        config = restore(json.loads(path.read_text()),path)
        for slot, trace in config['traces'].items():
            began = time.monotonic()
            row = dict(file=filename, slot=slot, recorded_seconds=trace['raw']['time_s'][-1])
            try:
                edited = smooth(trace)
                q, _ = arrays(edited, model, config['side'])
                np.testing.assert_array_equal(q[[0,-1]], np.asarray(trace['raw']['positions'])[[0,-1]])
                tr, fit = fit_trace(edited, model, config['side'])
                capacity = speed_capacity(tr, model, params, velocity=.7, acceleration=.7)
                row.update(ok=True, fit=fit, capacity=capacity, smoothing=edited['optimization'])
            except Exception as error:
                row.update(ok=False, error=str(error))
            row['computation_seconds'] = time.monotonic()-began
            report['candidates'].append(row)
            write_json(args.output, report)
            print(json.dumps(row, ensure_ascii=False), flush=True)
    if not all(row['ok'] for row in report['candidates']):raise SystemExit(1)


if __name__ == '__main__':main()
