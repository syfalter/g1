"""Read-only robot-free timing regression against an immutable saved plan."""
import argparse
import copy
import json
from pathlib import Path
import time

from rosidl_runtime_py.convert import message_to_ordereddict
from rosidl_runtime_py.set_message import set_message_fields
from trajectory_msgs.msg import JointTrajectory

from task_plan_archive import write_json
from task_planning_config import parameters_from_values
from task_trajectory import time_seconds, stretch_to_limits, task_jerk_limits
from task_trajectory_speed import scale_arm_segments, motion_limits, restore_baseline
from workbench_model import OfflineModel


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--plan',type=Path,required=True)
    parser.add_argument('--model',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    if args.output.exists():
        raise ValueError('Use a new output directory')
    source=json.loads(args.plan.read_text())
    parameters=parameters_from_values(source['planning_config']['values'])
    model=OfflineModel(args.model)
    segments=[]
    for record in source['segments']:
        trajectory=JointTrajectory();set_message_fields(trajectory,record['trajectory'])
        segments.append(dict(record,trajectory=trajectory))
    began=time.monotonic()
    scaled,report=scale_arm_segments(segments,24.,model,parameters,
        velocity=source['velocity_scaling'],acceleration=source['acceleration_scaling'])
    elapsed=time.monotonic()-began
    restored=restore_baseline(scaled,report)
    for old,new in zip(segments,restored):
        assert old['trajectory']==new['trajectory'],'Local timing baseline changed'
    for segment in scaled:
        if segment.get('kind','arm')=='hand':continue
        trajectory=segment['trajectory']
        vmax,amax=motion_limits(trajectory,model,source['velocity_scaling'],source['acceleration_scaling'],parameters)
        assert stretch_to_limits(copy.deepcopy(trajectory),vmax,amax,task_jerk_limits(trajectory.joint_names))<=1.+1e-6
        for point in (trajectory.points[0],trajectory.points[-1]):
            assert max(map(abs,point.velocities))<1e-6
            assert max(map(abs,point.accelerations))<1e-6
    args.output.mkdir(parents=True)
    write_json(args.output/'timing_report.json',dict(report,source_plan=str(args.plan),
        compute_seconds=elapsed,hardware_executable=False,hardware_commands_sent=0))
    write_json(args.output/'retimed_trajectories.json',dict(hardware_executable=False,
        segments=[dict(s,trajectory=message_to_ordereddict(s['trajectory'])) for s in scaled]))
    for stage in report['stage_limits']:
        print(f'{stage["label"]}: {stage["duration_before_s"]:.3f} -> {stage["duration_after_s"]:.3f} s; '
              f'{stage["local_timing"]["method"]}',flush=True)
    print(f'Total: {report["duration_before_s"]:.3f} -> {report["duration_after_s"]:.3f} s; compute {elapsed:.3f} s')
    print('Certified joint path difference:',report['max_joint_path_difference_rad'],'rad')


if __name__=='__main__':main()
