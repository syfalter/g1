"""Content-addressed editor computations; cached plans are not execution permits."""
import hashlib
from dataclasses import asdict
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import scipy

from task_screen_teach import fingerprint

DEFAULT_DIRECTORY=Path(__file__).resolve().parents[1]/'data/task_panel/trace_timing_cache'
MODULES=('task_trace_timing','task_trace_timing_cache','task_fast_timing','task_local_timing',
    'task_trajectory','task_trajectory_speed','task_cruise_timing','task_rail_trace',
    'task_rail_motion','task_dring_teach_data','task_trace_edit','task_trace_curve',
    'task_trace_markers','task_planning_config','workbench_model','task_hand_control',
    'task_landing_gear_data','task_path_smoothing','task_session','task_dring_cache','task_screen_teach')


def identity(trace,model,side,parameters,workflow,config,slot,velocity,acceleration):
    code=hashlib.sha256()
    for name in MODULES:code.update(Path(__file__).with_name(name+'.py').read_bytes())
    return fingerprint(dict(code=code.hexdigest(),libraries=(np.__version__,scipy.__version__),trace=trace,side=side,
        model=hashlib.sha256(ET.tostring(model.robot)).hexdigest(),limits=model.limits,
        parameters=asdict(parameters),workflow=workflow,slot=slot,
        hands=config.get('hands',{}),velocity=velocity,acceleration=acceleration))


def resolve(directory,identity,stage,options,compute,check):
    if directory is None:
        trajectory,report=compute()
        return trajectory,report,False
    from task_dring_cache import resolve as replay_resolve
    key=fingerprint(dict(identity=identity,stage=stage,options=options))
    def run():
        trajectory,report=compute()
        return trajectory,{},report
    trajectory,_,report=replay_resolve(directory,key,run,check)
    hit=report.pop('replay_cache')['hit']
    return trajectory,report,hit
