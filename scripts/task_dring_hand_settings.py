"""Next-plan hand settings and immutable archive variants; no hardware I/O."""
import copy
import json
import math
import os
from pathlib import Path
import tempfile

from task_hand_control import model_joints,posture_targets

DEFAULT_PATH=Path(__file__).resolve().parents[1]/'data/task_panel/dring_hand_settings.json'
DEFAULTS=dict(index_q=.8,middle_q=.8)


def validate(values):
    if not isinstance(values,dict) or set(values)!=set(DEFAULTS):
        raise ValueError('Expected index_q and middle_q')
    if any(type(v) not in (int,float) or not math.isfinite(v) or not 0<=v<=1 for v in values.values()):
        raise ValueError('Finger openness must be finite and in [0,1]')
    return {key:float(values[key]) for key in DEFAULTS}


def load(path=DEFAULT_PATH):
    path=Path(path)
    return validate(json.loads(path.read_text())) if path.is_file() else dict(DEFAULTS)


def save(values,path=DEFAULT_PATH):
    values=validate(values);path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
    temporary=None
    try:
        with tempfile.NamedTemporaryFile(mode='w',dir=path.parent,suffix='.tmp',delete=False) as stream:
            temporary=Path(stream.name);json.dump(values,stream,allow_nan=False)
        os.replace(temporary,path)
    finally:
        if temporary is not None:temporary.unlink(missing_ok=True)


def archive_variant(archive,settings):
    from task_plan_archive import stages,seconds
    settings=validate(settings)
    result=copy.deepcopy(archive);plan=result['plan'];side=plan['arm']
    target=posture_targets('dring_two_finger',settings)
    state=dict(plan['start']);changed=False
    names={side+'_index_1_joint',side+'_middle_1_joint'}
    for segment in plan['segments']:
        trajectory=segment['trajectory'];joints=trajectory['joint_names'];points=trajectory['points']
        if segment['kind']=='hand':
            if segment.get('posture')=='dring_two_finger' or segment['label']=='Hand dring_two_finger':
                segment['target_q']=list(target)
            goal=model_joints(side,segment['target_q'])
            duration=seconds(points[-1])
            for index,name in enumerate(joints):
                if name not in names:continue
                if abs(points[0]['positions'][index]-state[name])<=1e-12 and abs(points[-1]['positions'][index]-goal[name])<=1e-12:
                    continue
                changed=True
                delta=goal[name]-state[name]
                for point in points:
                    t=seconds(point)/duration
                    point['positions'][index]=state[name]+t**3*(10+t*(-15+6*t))*delta
                    if point.get('velocities'):
                        point['velocities'][index]=30*t*t*(1-t)**2*delta/duration
                    if point.get('accelerations'):
                        point['accelerations'][index]=60*t*(1-t)*(1-2*t)*delta/duration**2
        state.update(zip(joints,points[-1]['positions']))
    plan['dring_hand_settings']=dict(settings)
    result['stages']=stages(plan)
    result['hand_settings_changed']=changed
    return result
