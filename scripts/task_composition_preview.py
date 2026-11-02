"""Offline-only reusable motion units. No hardware clients or execution authorization."""
import copy
import hashlib
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from rosidl_runtime_py.convert import message_to_ordereddict
from rosidl_runtime_py.set_message import set_message_fields

from task_trajectory import coefficients, time_seconds, peak
from task_screen_teach import fingerprint, write_json

ROOT = Path(__file__).resolve().parents[1]
CACHE = ROOT / 'data/composition_preview_cache'


def stamp(point, seconds):
    ns = round(float(seconds)*1e9)
    point.time_from_start.sec, point.time_from_start.nanosec = divmod(ns, 1000000000)
    return point


def at(trajectory, seconds):
    times = np.array([time_seconds(p) for p in trajectory.points])
    if seconds < -1e-8 or seconds > times[-1]+1e-8:
        raise ValueError('Sample outside trajectory')
    i = min(max(int(np.searchsorted(times, seconds, side='right'))-1, 0), len(times)-2)
    dt = times[i+1]-times[i]
    poly = coefficients(trajectory.points[i], trajectory.points[i+1], dt)
    u = (seconds-times[i])/dt
    values = [np.polynomial.polynomial.polyval(u, np.polynomial.polynomial.polyder(poly, k))/dt**k for k in range(3)]
    return stamp(JointTrajectoryPoint(positions=values[0].tolist(), velocities=values[1].tolist(),
                                      accelerations=values[2].tolist()), seconds)


def section(trajectory, begin, end):
    if not 0 <= begin < end <= time_seconds(trajectory.points[-1])+1e-9:
        raise ValueError('Invalid section')
    # Use the ROS nanosecond clock before splitting. Floating-point sums at a
    # unit boundary can otherwise produce two knots with the same timestamp.
    first,last=round(begin*1e9),round(end*1e9)
    if first>=last:raise ValueError('Section shorter than timestamp resolution')
    begin,end=first/1e9,last/1e9
    inner=sorted({round(time_seconds(p)*1e9) for p in trajectory.points
                  if first<round(time_seconds(p)*1e9)<last})
    times = [begin] + [t/1e9 for t in inner] + [end]
    return JointTrajectory(joint_names=list(trajectory.joint_names),
                           points=[stamp(at(trajectory, t), t-begin) for t in times])


def concatenate(parts, limit=200000):
    parts = list(parts)
    if not parts:
        raise ValueError('Empty composition')
    if sum(len(p.points) for p in parts) > limit:
        raise ValueError('预览点数超过预算，请减少预览次数；缓存单元仍保留')
    result = JointTrajectory(joint_names=list(parts[0].joint_names))
    offset = 0.
    for tr in parts:
        if list(tr.joint_names) != list(result.joint_names):
            raise ValueError('Composition joint order differs')
        if result.points:
            for field in ('positions', 'velocities', 'accelerations'):
                if not np.allclose(getattr(result.points[-1], field), getattr(tr.points[0], field), atol=1e-7, rtol=0):
                    raise ValueError('Composition boundary is not C2: ' + field)
        for p in tr.points[1:] if result.points else tr.points:
            result.points.append(stamp(copy.deepcopy(p), offset+time_seconds(p)))
        offset += time_seconds(tr.points[-1])
    return result


def stationary(names, q):
    return JointTrajectoryPoint(positions=[q[n] for n in names], velocities=[0.]*len(names), accelerations=[0.]*len(names))


def bridge(names, a, b):
    # Quintic interpolation carries the source derivatives, including cut endpoints.
    duration = max(.6, float(np.max(np.abs(np.asarray(a.positions)-b.positions)))/.25)
    source = JointTrajectory(joint_names=list(names), points=[stamp(copy.deepcopy(a), 0.), stamp(copy.deepcopy(b), duration)])
    return JointTrajectory(joint_names=list(names), points=[at(source, t) for t in np.linspace(0., duration, 21)])


def check_positions(trajectory, model):
    for a,b in zip(trajectory.points, trajectory.points[1:]):
        c = coefficients(a,b,time_seconds(b)-time_seconds(a))
        for axis,name in enumerate(trajectory.joint_names):
            lo,hi = model.limits[name]
            poly = c[:,axis].copy(); poly[0] -= (lo+hi)/2
            if peak(poly) > (hi-lo)/2-.02+1e-8:
                raise ValueError('组合连接曲线超出关节限位：'+name)


def parent_of_trim(config, slot):
    from task_trace_versions import bank
    entry = bank(config,slot); trace = config['traces'][slot]
    origin = trace.get('edit_origin', {})
    if trace.get('optimization',{}).get('kind') != 'trim_tail' or origin.get('operation') != 'trim_tail':
        raise ValueError('组合需要选中删除末尾后的版本：'+slot)
    index = origin.get('version')
    if type(index) is not int or not 0 <= index <= len(entry['edits']):
        raise ValueError('截尾父版本编号无效')
    parent = entry['baseline'] if index == 0 else entry['edits'][index-1]
    if parent is None:
        raise ValueError('截尾父版本已删除，不能恢复原退出段')
    q,t = np.asarray(parent['raw']['positions']), np.asarray(parent['raw']['time_s'])
    kept = np.asarray(trace['raw']['time_s'])
    expected = np.column_stack([np.interp(kept,t,axis) for axis in q.T])
    if kept[-1] >= t[-1] or not np.allclose(expected,trace['raw']['positions'],atol=1e-8,rtol=0):
        raise ValueError('当前轨迹不再是父版本的截尾前缀；不能自动选取退出段')
    return parent, float(kept[-1]), dict(slot=slot, selectedVersion=entry['selected'], parentVersion=index,
                                        selectedSha256=fingerprint(trace), parentSha256=fingerprint(parent))


def fit(trace, model, side, check):
    from task_dring_teach_data import fit_trace
    if isinstance(trace.get('hand'),dict):
        raise ValueError('同步手指轨迹尚不能用于此组合预览')
    return fit_trace(dict(trace,hand=None),model,side,check)[0]


def raw_phases(workflow, config, ready, model, options, check):
    from task_dring_teach_data import moving
    side = config['side']; names = moving(side)
    above = stationary(names,ready[side]['joints'])
    phases, recipes, sources = {}, {}, {}
    def add(name, trajectory, motion='operation',smooth=False,smoothing_end=None):
        phases[name]=dict(trajectory=trajectory,motionClass=motion,smooth=smooth,smoothing_end=smoothing_end)
    if workflow == 'dring_demo':
        prefixes = {}
        for direction in ('left','right'):
            slot = options[direction+'Slot']
            if not slot.startswith(direction+'_'):
                raise ValueError('左/右方向槽位不匹配')
            parent, cutoff, sources[direction] = parent_of_trim(config,slot)
            full = fit(parent,model,side,check)
            prefixes[direction] = section(full,0.,cutoff)
            if direction == 'right':
                tail = section(full,cutoff,time_seconds(full.points[-1]))
        left,right = prefixes['left'],prefixes['right']
        lr = bridge(names,left.points[-1],right.points[0])
        rl = bridge(names,right.points[-1],left.points[0])
        taught = stationary(names,config['ready']['joints'])
        add('above_to_ready',bridge(names,above,taught),'transit')
        add('ready_to_left',bridge(names,taught,left.points[0]))
        # Keep moving cut endpoints inside a phase, never independently rescale them.
        add('left_to_right',concatenate([left,lr]),smooth=True,smoothing_end=time_seconds(left.points[-1]))
        add('right_to_left',concatenate([right,rl]),smooth=True,smoothing_end=time_seconds(right.points[-1]))
        add('right_with_exit_tail',concatenate([right,tail]),smooth=True,smoothing_end=time_seconds(right.points[-1]))
        add('return_above',bridge(names,tail.points[-1],above),'transit')
        recipes['entry']=['above_to_ready','ready_to_left']
        recipes['cycle']=['left_to_right','right_to_left']
        recipes['final']=['left_to_right','right_with_exit_tail','return_above']
        sources['exitPolicy'] = 'right-trim-parent-suffix-then-original-style-preparation-join'
    else:
        from task_landing_gear_data import SLOTS
        units={}
        for slot in SLOTS:
            trace = config['traces'][slot]
            units[slot] = fit(trace,model,side,check)
            sources[slot] = dict(sha256=fingerprint(trace))
        lower,upper = units['lower_to_upper'].points[0],units['upper_to_lower'].points[0]
        for position,point in (('lower',lower),('upper',upper)):
            entry = units['above_to_'+position]
            add('approach_'+position,bridge(names,above,entry.points[0]),'transit')
            add('above_to_'+position,entry,'transit',smooth=True)
            add('engage_'+position,bridge(names,entry.points[-1],point))
            recipes['entry_'+position]=['approach_'+position,'above_to_'+position,'engage_'+position]
        lu,ul = units['lower_to_upper'],units['upper_to_lower']
        add('lower_to_upper',lu,smooth=True)
        add('upper_to_lower',ul,smooth=True)
        add('align_upper',bridge(names,lu.points[-1],upper))
        add('align_lower',bridge(names,ul.points[-1],lower))
        recipes['lu']=['lower_to_upper','align_upper']
        recipes['ul']=['upper_to_lower','align_lower']
        recipes['cycle_lower']=recipes['lu']+recipes['ul']
        recipes['cycle_upper']=recipes['ul']+recipes['lu']
        for position,point in (('lower',lower),('upper',upper)):
            exit_tr = units[position+'_to_above']
            add('disengage_'+position,bridge(names,point,exit_tr.points[0]))
            add(position+'_to_above',exit_tr,'transit',smooth=True)
            add('return_'+position,bridge(names,exit_tr.points[-1],above),'transit')
            recipes['exit_'+position]=['disengage_'+position,position+'_to_above','return_'+position]
    for phase in phases.values():
        check();check_positions(phase['trajectory'],model)
    if workflow=='landing_gear':
        for name,phase in phases.items():
            edit=config['traces'].get(name,{}).get('trim_edit',{})
            if 'start_s' in edit and 'bridge_s' in edit:
                duration=time_seconds(phase['trajectory'].points[-1])
                phase['stopHints']=[t for t in (edit['start_s'],edit['start_s']+edit['bridge_s']) if 0.<t<duration]
    return phases,recipes,sources


def raw_units(workflow, config, ready, model, options, check):
    phases,recipes,sources=raw_phases(workflow,config,ready,model,options,check)
    return {name:concatenate(phases[p]['trajectory'] for p in sequence)
            for name,sequence in recipes.items()},sources


def validate_timing(trajectory, model, parameters, check):
    from task_trajectory_speed import speed_capacity
    check_positions(trajectory,model)
    capacity=speed_capacity(trajectory,model,parameters,velocity=1.,acceleration=1.,check=check)
    # speed_capacity includes a planning margin; the final gate checks actual limits.
    if capacity['maximum_multiplier'] is not None and capacity['maximum_multiplier']*1.005<1.-1e-6:
        raise ValueError('分阶段时序超过配置限值')
    return capacity


def assemble_phases(phases, recipes, model, parameters, operation_rate, transit_rate, check):
    from task_trajectory_speed import rescale
    timed={};reports={}
    for name,phase in phases.items():
        check()
        rate=(min(transit_rate,operation_rate) if phase['motionClass']=='mixed' else
              transit_rate if phase['motionClass']=='transit' else operation_rate)
        trajectory=rescale(phase['trajectory'],rate)
        capacity=validate_timing(trajectory,model,parameters,check)
        timed[name]=trajectory
        reports[name]=dict(phase['report'],motionClass=phase['motionClass'],rate=rate,
                          seconds=time_seconds(trajectory.points[-1]),validation=capacity)
    saved={}
    for name,sequence in recipes.items():
        trajectory=concatenate(timed[p] for p in sequence)
        validate_timing(trajectory,model,parameters,check)
        saved[name]=dict(trajectory=message_to_ordereddict(trajectory),seconds=time_seconds(trajectory.points[-1]),
                        report=dict(timing_policy='phase_fastest_v1',limits_enforced=True,
                                    phases=[dict(name=p,**reports[p]) for p in sequence]))
    return saved


def fastest_units(workflow,config,ready,model,parameters,options,check,progress,directory,identity):
    import json
    from task_fast_timing import fastest
    # Slower playback rates and repetition counts never trigger another rate search.
    identity=dict(identity,mode='phase_fastest_base',multiplier=1.,
                  options={k:v for k,v in options.items() if k!='transitRate'})
    key=fingerprint(identity);path=Path(directory)/(key+'.json')
    cached=None
    try:
        data=json.loads(path.read_text())
        if data['key']==key and fingerprint(data['payload'])==data['sha256']:cached=data['payload']
    except (OSError,ValueError,KeyError):pass
    hit=cached is not None
    if cached is None:
        phases,recipes,sources=raw_phases(workflow,config,ready,model,options,check)
        if workflow=='landing_gear' and options.get('smooth'):
            from task_composition_spatial import continuous_gear_phases
            phases,recipes=continuous_gear_phases(phases,recipes)
            sources['continuityPolicy']='spatial-units-stop-only-at-grip-release-and-reversal'
        stored={}
        for name,phase in phases.items():
            check();progress('局部尽快规划：'+name)
            source=phase['trajectory'];smoothing=None
            if phase.get('spatial'):
                from task_composition_spatial import rebuild
                source,smoothing=rebuild(source,model,config['side'],
                    joint_tolerance=options.get('jointTolerance',.003),
                    tcp_tolerance=options.get('tcpTolerance',.001),
                    window=options.get('smoothWindow',.02),seams=phase.get('stopHints',()),check=check,
                    valley_refit=bool(options.get('entryValleyRefit') and name=='entry_lower_continuous'),
                    parameters=parameters)
            elif options.get('smooth') and phase.get('smooth'):
                from task_composition_smoothing import smooth_candidate
                end=phase.get('smoothing_end') or time_seconds(source.points[-1])
                prefix=section(source,0.,end)
                prefix,smoothing=smooth_candidate(prefix,model,config['side'],
                    joint_tolerance=options.get('jointTolerance',.003),tcp_tolerance=options.get('tcpTolerance',.001),
                    window=options.get('smoothWindow',.02),check=check)
                source=concatenate([prefix,section(source,end,time_seconds(source.points[-1]))]) if end<time_seconds(source.points[-1]) else prefix
                smoothing['protected_suffix_from_s']=end
            trajectory,report=fastest(source,model,parameters,
                                     velocity=1.,acceleration=1.,check=check)
            if smoothing is not None:report['pathSmoothing']=smoothing
            if phase.get('mergedPhases'):report['mergedPhases']=phase['mergedPhases']
            validate_timing(trajectory,model,parameters,check)
            stored[name]=dict(trajectory=message_to_ordereddict(trajectory),report=report,
                              motionClass=phase['motionClass'])
        cached=dict(phases=stored,recipes=recipes,sources=sources)
        check();write_json(path,dict(key=key,payload=cached,sha256=fingerprint(cached)))
    phases={}
    for name,phase in cached['phases'].items():
        trajectory=JointTrajectory();set_message_fields(trajectory,copy.deepcopy(phase['trajectory']))
        phases[name]=dict(phase,trajectory=trajectory)
    return phases,cached['recipes'],cached['sources'],hit


def prepare(workflow, config, ready, model, parameters, mode, multiplier, options, check=lambda:None,
            progress=lambda text:None, directory=CACHE):
    from task_trajectory_speed import scale_arm_segments
    from task_cruise_timing import retime
    from dataclasses import asdict
    if mode not in ('original','cruise','fastest') or not np.isfinite(multiplier) or not .1 <= multiplier <= 5:
        raise ValueError('Invalid composition speed')
    transit_rate=options.get('transitRate',1.)
    if mode=='fastest' and (multiplier>1. or not np.isfinite(transit_rate) or not .1<=transit_rate<=1.):
        raise ValueError('尽快策略的操作和接近退出速率须为 0.1 至 1.0')
    # Count/finish/exit are intentionally absent: changing repetitions never retimes units.
    traces = config['traces']
    parents = {d:parent_of_trim(config,options[d+'Slot'])[0] for d in ('left','right')} if workflow=='dring_demo' else {}
    code = hashlib.sha256()
    for filename in ('task_composition_preview.py','task_dring_teach_data.py','task_cruise_timing.py',
                     'task_trajectory_speed.py','task_local_timing.py','task_trace_timing.py','task_trace_curve.py',
                     'task_trajectory.py','workbench_model.py','task_hand_control.py','task_fast_timing.py',
                     'task_landing_gear_data.py','task_composition_smoothing.py','task_composition_spatial.py',
                     'task_spatial_valleys.py'):
        code.update(Path(__file__).with_name(filename).read_bytes())
    identity = dict(schema=1,workflow=workflow,side=config['side'],traces=traces,parents=parents,ready=ready,
                    taught=config.get('ready'),hands=config.get('hands'),options=options,
                    parameters=asdict(parameters),mode=mode,multiplier=multiplier,
                    code=code.hexdigest(),model=hashlib.sha256(ET.tostring(model.robot)).hexdigest())
    key=fingerprint(identity); path=Path(directory)/(key+'.json')
    import json
    try:
        data=json.loads(path.read_text())
        if data['key']!=key or fingerprint(data['payload'])!=data['sha256']:raise ValueError('Cache integrity')
        check();return data['payload'],True,path
    except (OSError,ValueError,KeyError):
        pass
    progress('生成进入 / 循环 / 末轮单元')
    phase_cache_hit=False
    if mode=='fastest':
        phases,recipes,sources,phase_cache_hit=fastest_units(workflow,config,ready,model,parameters,options,
                                                         check,progress,directory,identity)
        units={}
    else:units,sources=raw_units(workflow,config,ready,model,options,check)
    context=dict(ready['left']['joints'],**ready['right']['joints'])
    from task_hand_control import model_joints,POSTURES
    for side in ('left','right'):context.update(model_joints(side,POSTURES['index_only']))
    if workflow=='landing_gear':
        context.update(model_joints('right',config['hands']['open']['feedback_q']))
    saved=assemble_phases(phases,recipes,model,parameters,multiplier,transit_rate,check) if mode=='fastest' else {}
    for name,tr in units.items():
        progress('速度规划：'+name);check()
        unit_context=dict(context)
        if workflow=='landing_gear' and not name.startswith(('entry_','exit_')):
            unit_context.update(model_joints('right',config['hands']['grip']['feedback_q']))
        if mode=='cruise':
            timed,report=retime(tr,model,unit_context,config['side'],multiplier,parameters,
                              velocity=.7,acceleration=.7,check=check,original_policy='uniform')
        else:
            segments,report=scale_arm_segments([dict(trajectory=tr)],multiplier,model,parameters,
                                               velocity=.7,acceleration=.7,policy='uniform',check=check)
            timed=segments[0]['trajectory']
        check_positions(timed,model)
        saved[name]=dict(trajectory=message_to_ordereddict(timed),report=report,seconds=time_seconds(timed.points[-1]))
    payload=dict(schema=1,workflow=workflow,units=saved,sources=sources,context=context,options=options,
                 mode=mode,multiplier=multiplier,transitRate=transit_rate,phaseCacheHit=phase_cache_hit,
                 cycleBoundary='stationary-no-added-wait',
                 handPresets=config.get('hands',{}),hardware_executable=False,physical_verified=False)
    check();write_json(path,dict(key=key,payload=payload,sha256=fingerprint(payload)))
    return payload,False,path


def select(payload, count, start='LOWER', finish='UPPER', leave=False):
    if type(count) is not int or not 1 <= count <= 100:
        raise ValueError('预览次数须为 1 至 100')
    if payload['workflow']=='dring_demo':
        return ['entry']+['cycle']*(count-1)+['final']
    if start not in ('LOWER','UPPER') or finish not in ('LOWER','UPPER') or type(leave) is not bool:
        raise ValueError('Invalid gear composition')
    result=['entry_'+start.lower(),'grip_wait']+['cycle_'+start.lower()]*count
    if start!=finish:result.append('lu' if start=='LOWER' else 'ul')
    if leave:result+=['open_wait','exit_'+finish.lower()]
    return result


def materialize(payload, sequence):
    units={}
    for name,item in payload['units'].items():
        tr=JointTrajectory();set_message_fields(tr,copy.deepcopy(item['trajectory']));units[name]=tr
    parts=[];events=[];elapsed=0.
    for name in sequence:
        spec=payload.get('handWaits',{}).get(name)
        if spec is not None:
            tr=units[name]
            events.append(dict(time=elapsed,hand=spec['side'],posture=spec['posture']))
        elif name.endswith('_wait'):
            last=parts[-1].points[-1]
            tr=JointTrajectory(joint_names=list(parts[-1].joint_names),
                               points=[stamp(copy.deepcopy(last),0.),stamp(copy.deepcopy(last),1.)])
            events.append(dict(time=elapsed,hand='right',posture=name.split('_')[0]))
        else:tr=units[name]
        parts.append(tr);elapsed+=time_seconds(tr.points[-1])
    if payload['workflow']=='landing_gear':events.insert(0,dict(time=0.,hand='right',posture='open'))
    return concatenate(parts),events
