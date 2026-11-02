"""Precompute persistent playback caches and exports using archived robot data."""
import argparse
import copy
from dataclasses import replace
import json
from pathlib import Path
import time

from rosidl_runtime_py.convert import message_to_ordereddict
from rosidl_runtime_py.set_message import set_message_fields
from trajectory_msgs.msg import JointTrajectory

from prepare_dring_candidates import ARCHIVE,ROOT
from task_dring_cache import cache_key,store
from task_dring_teach_data import build,preview
from task_planning_config import load_parameters,parameters_from_values
from task_screen_teach import write_json,fingerprint
from workbench_model import OfflineModel


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('direction',choices=('left','right'))
    args=parser.parse_args();direction=args.direction
    bundle=json.loads((ARCHIVE/'candidates/demonstration.json').read_text())
    model=OfflineModel(ARCHIVE/'robot.urdf')
    frozen=json.loads((ARCHIVE/('dring_demo_0093' if direction=='left' else 'dring_demo_0020')/'plan.json').read_text())
    stage=next(s for s in frozen['segments'] if s.get('recording_fit'))
    archived=parameters_from_values(frozen['planning_config']['values'])
    current,_=load_parameters();parameters=replace(current,waist_velocity_rad_s=min(current.waist_velocity_rad_s,archived.waist_velocity_rad_s))
    cache=ROOT/'data/task_panel/dring_replay_cache'
    saved_trajectory=copy.deepcopy(stage['trajectory'])
    trajectory=JointTrajectory();set_message_fields(trajectory,copy.deepcopy(saved_trajectory))
    report=copy.deepcopy(frozen['speed_scaling']);report['duration_after_s']=stage['duration']
    report['preserved_successful_plan']=str(ARCHIVE/('dring_demo_0093' if direction=='left' else 'dring_demo_0020'))
    for multiplier in (1.,1.5):
        if multiplier<report.get('rate_max',1.5):continue
        record=dict(report,requested_multiplier=multiplier)
        key=cache_key(bundle['traces'][direction],model,bundle['side'],multiplier,.7,.7,parameters)
        store(cache,key,(trajectory,stage['recording_fit'],record))
    start=dict(frozen['start'],**bundle['ready']['joints'])
    results=[]
    for index in range(1,6):
        slot=direction if index==1 else f'{direction}_{index}'
        for multiplier in (1.5,1.):
            began=time.monotonic()
            plan=build(model,bundle,start,slot,parameters,.7,.7,multiplier,cache_dir=cache)
            motion=next(s for s in plan['segments'] if s.get('recording_fit'))
            # The exact same verified trajectory also satisfies the looser YAML waist limit.
            key=cache_key(bundle['traces'][slot],model,bundle['side'],multiplier,.7,.7,current)
            store(cache,key,(motion['trajectory'],motion['recording_fit'],plan['speed_scaling']))
            output=ARCHIVE/'candidates'/f'{slot}_{multiplier:g}x'
            write_json(output/'plan.json',dict(plan,segments=[dict(s,trajectory=message_to_ordereddict(s['trajectory'])) for s in plan['segments']]))
            write_json(output/'rviz_trajectory.json',message_to_ordereddict(preview(plan)))
            # Force a disk-cache load with fitting/timing disabled to prove no recomputation.
            from unittest.mock import patch
            warm=time.monotonic()
            with patch('task_dring_teach_data.fit_trace',side_effect=AssertionError('Unexpected refit')):
                again=build(model,bundle,start,slot,current,.7,.7,multiplier,cache_dir=cache)
            replay=next(s for s in again['segments'] if s.get('recording_fit'))
            assert replay['trajectory']==motion['trajectory']
            item=dict(slot=slot,multiplier=multiplier,duration_s=motion['duration'],
                fit=motion['recording_fit'],cache_hit=plan['speed_scaling']['replay_cache']['hit'],
                disk_cache_reload_s=time.monotonic()-warm,total_compute_s=time.monotonic()-began,
                path_error_rad=plan['speed_scaling'].get('max_joint_path_difference_rad',0.),
                baseline_geometry_unchanged=index==1,
                exact_saved_timing=index==1 and message_to_ordereddict(motion['trajectory'])==saved_trajectory,
                physical_tested=index==1 and message_to_ordereddict(motion['trajectory'])==saved_trajectory)
            results.append(item);print(json.dumps(item),flush=True)
            write_json(ARCHIVE/'candidates'/f'{direction}_validation.json',results)


if __name__=='__main__':main()
