"""Content-addressed, persistent replay timing cache; no hardware authorization."""
import copy
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

from rosidl_runtime_py.convert import message_to_ordereddict
from rosidl_runtime_py.set_message import set_message_fields
from trajectory_msgs.msg import JointTrajectory

from task_screen_teach import fingerprint,write_json


def cache_key(trace,model,side,multiplier,velocity,acceleration,parameters,*,timing_mode='original',hand_context=None):
    from task_dring_teach_data import moving
    from task_trajectory_speed import motion_limits
    from task_trajectory import task_jerk_limits
    names=list(moving(side))
    vmax,amax=motion_limits(JointTrajectory(joint_names=names),model,velocity,acceleration,parameters)
    code=hashlib.sha256()
    for name in ('task_dring_teach_data.py','task_trajectory.py','task_trajectory_speed.py','task_local_timing.py','task_trace_curve.py','task_cruise_timing.py','task_trace_markers.py','task_trace_timing.py'):
        code.update(Path(__file__).with_name(name).read_bytes())
    return fingerprint(dict(version=1,code=code.hexdigest(),trace=trace,side=side,multiplier=multiplier,
        timing_mode=timing_mode,hand_context=hand_context if timing_mode=='cruise' else None,
        velocity=velocity,acceleration=acceleration,vmax=vmax,amax=amax,jerk=task_jerk_limits(names),
        model=hashlib.sha256(ET.tostring(model.robot)).hexdigest()))


def store(directory,key,motion):
    tr,fit,report=motion
    payload=dict(trajectory=message_to_ordereddict(tr),fit=fit,report=report)
    write_json(Path(directory)/(key+'.json'),dict(schema=1,key=key,payload=payload,sha256=fingerprint(payload)))


def resolve(directory,key,compute,check=lambda:None):
    check();path=Path(directory)/(key+'.json')
    try:
        item=json.loads(path.read_text())
        if item['schema']!=1 or item['key']!=key or fingerprint(item['payload'])!=item['sha256']:
            raise ValueError('Replay cache integrity mismatch')
        payload=item['payload'];tr=JointTrajectory();set_message_fields(tr,payload['trajectory'])
        from task_trajectory_speed import rescale
        tr=rescale(tr,1.)
        result=(tr,payload['fit'],payload['report']);hit=True
    except (OSError,ValueError,KeyError,TypeError,AssertionError):
        result=compute();check();store(directory,key,result);hit=False
    check()
    tr,fit,report=result
    report=copy.deepcopy(report);report['replay_cache']=dict(hit=hit,key=key,path=str(path))
    return tr,fit,report
