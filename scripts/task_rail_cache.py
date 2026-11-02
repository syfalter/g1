"""Persistent validated rail curves, never saved execution authorization."""
import copy
import hashlib
import json
from dataclasses import asdict
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np

from rosidl_runtime_py.convert import message_to_ordereddict
from rosidl_runtime_py.set_message import set_message_fields
from trajectory_msgs.msg import JointTrajectory

from task_screen_teach import fingerprint, write_json, MOVING
from task_trajectory_speed import rescale


def key(trace, model, parameters, *, context=None):
    code = hashlib.sha256()
    for name in ('task_rail_cache.py','task_rail_trace.py','task_rail_motion.py','task_rail_hand.py',
                 'task_trajectory.py','task_trajectory_speed.py','task_local_timing.py',
                 'task_screen_teach.py','task_path_smoothing.py','task_trace_curve.py','task_cruise_timing.py','task_trace_timing.py'):
        code.update(Path(__file__).with_name(name).read_bytes())
    return fingerprint(dict(version=1,trace=trace,parameters=asdict(parameters),
        cruise_context=context if getattr(parameters,'rail_replay_cruise',False) else None,
        model=hashlib.sha256(ET.tostring(model.robot)).hexdigest(),code=code.hexdigest()))


def resolve(directory, trace, model, parameters, compute, check, *, require_hit=False,context=None):
    check()
    identity = key(trace,model,parameters,context=context)
    path = Path(directory)/(identity+'.json')
    try:
        item = json.loads(path.read_text())
        payload = item['payload']
        if item['schema'] != 1 or item['key'] != identity or fingerprint(payload) != item['sha256']:
            raise ValueError('Rail replay cache integrity mismatch')
        tr = JointTrajectory(); set_message_fields(tr,payload['trajectory'])
        tr = rescale(tr,1.)
        if (list(tr.joint_names) != list(MOVING) or
                not np.allclose(tr.points[0].positions,trace['raw']['positions'][0],atol=1e-9,rtol=0.) or
                not np.allclose(tr.points[-1].positions,trace['raw']['positions'][-1],atol=1e-9,rtol=0.)):
            raise ValueError('Rail cached endpoints mismatch')
        report = copy.deepcopy(payload['report']); hit = True
    except (OSError,ValueError,KeyError,TypeError,IndexError,AssertionError) as error:
        if require_hit:
            raise RuntimeError('已验证回放缓存缺失、失效或损坏，请先规划一次') from error
        tr,report = compute(); check()
        payload = dict(trajectory=message_to_ordereddict(tr),report=report)
        write_json(path,dict(schema=1,key=identity,payload=payload,sha256=fingerprint(payload)))
        hit = False
    check()
    report = dict(report,replay_cache=dict(hit=hit,key=identity,path=str(path)))
    return tr,report


def cycle_key(config, model, parameters, ready, operation, mode):
    from task_rail_trace import required_stages
    return fingerprint(dict(operation=operation,mode=mode,ready=ready,
        hands={k:v['feedback_q'] for k,v in config['hands'].items()},
        stages={s:key(config['traces'][s],model,parameters) for s in required_stages(operation,mode)}))


def require_success(directory, identity):
    try:
        record=json.loads((Path(directory)/(identity+'.success.json')).read_text())
        if record != dict(schema=1,key=identity,planned=True):raise ValueError('Invalid success record')
    except (OSError,ValueError,TypeError) as error:
        raise RuntimeError('这组轨迹/手型/准备位/参数没有成功规划记录，请先规划一次') from error


def record_success(directory, identity):
    write_json(Path(directory)/(identity+'.success.json'),dict(schema=1,key=identity,planned=True))
