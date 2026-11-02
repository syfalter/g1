"""Build a bounded lowering update and an explicitly unselected reverse preview."""
import argparse
import copy
import json
from pathlib import Path
import sys

import numpy as np
from rosidl_runtime_py.convert import message_to_ordereddict

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'deployment/rail_jetson'))
from automatic_recovery import reversed_path
from bundle import digest, read, write
from gear_protocol import BINDINGS, GearLibrary, concatenate as join_json
from releases import package, planned_action, validate as validate_release
from task_composition_preview import materialize
from task_composition_panel import preview_samples
from task_composition_smoothing import evaluate
from task_offline_optimizer import decode, validate
from task_offline_optimizer_cases import sequence_for
from task_optimizer_snapshot import load, save
from task_planning_config import parameters_from_values
from task_smooth_timing import smoothness
from task_trajectory import time_seconds
from workbench_model import OfflineModel


def preview(saved, lowering, model, directory, method):
    results=copy.deepcopy(saved['variants'])
    flow=results['flow']
    payload=flow['payload']
    units=payload['units']
    units['ul']=dict(trajectory=lowering,seconds=time_seconds(decode(lowering).points[-1]),
                     report=dict(method=method,physical_verified=False))
    for name,order in (('cycle_lower',('lu','ul')),('cycle_upper',('ul','lu'))):
        tr=join_json(units[k]['trajectory'] for k in order)
        units[name]=dict(trajectory=tr,seconds=time_seconds(decode(tr).points[-1]),
                         report=dict(parts=list(order),method=method,physical_verified=False))
    payload['options']['lowering_revision']=method
    payload['flowEvidence']['ul']=units['ul']['report']
    for name in ('cycle_lower','cycle_upper'):
        payload['flowEvidence'][name]=units[name]['report']
    tr,events=materialize(payload,flow['sequence'])
    times,tcp,joint=preview_samples(tr,payload,events,model,'right')
    matrices=[evaluate(tr,times,k) for k in (1,2,3)]
    flow.update(trajectory=tr,events=events,times=times,tcp=tcp,joint=joint,
        joint_names=list(tr.joint_names),joint_matrix=matrices[0],acceleration_matrix=matrices[1],
        jerk_matrix=matrices[2],acceleration=np.max(abs(matrices[1]),axis=1),
        jerk=np.max(abs(matrices[2]),axis=1),smoothness=smoothness(tr))
    path=save(results,copy.deepcopy(saved['snapshot_controls']),model,directory=directory)
    return path,payload


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--urdf',type=Path,required=True)
    parser.add_argument('--staging',type=Path,required=True)
    parser.add_argument('--candidate',type=Path,required=True)
    args=parser.parse_args()
    stage=args.staging
    base=read(stage/'before/selected_bundle.json')
    old_library=read(stage/'before/gear_protocol.json')
    old_action=base['payload']['actions'][BINDINGS[0]]
    model=OfflineModel(args.urdf)
    params=parameters_from_values(old_action['plan']['planning_config']['values'])
    saved=load(ROOT/old_action['source'],model)
    candidate=read(args.candidate)
    if candidate['source_snapshot_sha256']!=old_library['payload']['snapshot_sha256']:
        raise ValueError('Candidate belongs to another source snapshot')
    lower=candidate['trajectory']
    if candidate['selected_seconds']>=candidate['original_seconds']:
        raise ValueError('No faster bounded candidate was selected')
    bounds=candidate['evidence']['bounds']
    if bounds['tcp_m']>.001 or bounds['joint_rad']>.003 or bounds['orientation_rad']>np.deg2rad(.5):
        raise ValueError('Candidate exceeds original contact-path budget')
    for index in (0,-1):
        for field in ('positions','velocities','accelerations'):
            if not np.allclose(lower['points'][index][field],old_library['payload']['units']['ul']['trajectory']['points'][index][field],atol=1e-7,rtol=0):
                raise ValueError('Candidate changed a gear boundary')
    new_path,payload=preview(saved,lower,model,stage/'previews/bounded','bounded-lowering-v6')
    reverse_path,_=preview(saved,reversed_path(old_library['payload']['units']['lu']['trajectory']),
        model,stage/'previews/reverse','reverse-lift-candidate-needs-contact-review')
    snapshot_sha=read(new_path)['sha256']
    actions=copy.deepcopy(base['payload']['actions'])
    for name,start,finish,parts in (
        (BINDINGS[0],'LOWER','UPPER',('entry_lower','cycle_lower','lu','exit_upper')),
        (BINDINGS[1],'UPPER','LOWER',('entry_upper','cycle_upper','ul','exit_lower'))):
        old=actions[name]
        plan=copy.deepcopy(old['plan'])
        trajectories=[payload['units'][parts[0]]['trajectory'],
            join_json(payload['units'][k]['trajectory'] for k in parts[1:3]),
            payload['units'][parts[3]]['trajectory']]
        arms=[s for s in plan['segments'] if s['kind']=='arm' and s['stage']!='prepare']
        if len(arms)!=3:raise ValueError('Unexpected selected gear layout')
        peaks=[]
        for segment,tr in zip(arms,trajectories):
            segment.update(trajectory=copy.deepcopy(tr),duration=time_seconds(decode(tr).points[-1]),
                start=dict(zip(tr['joint_names'],tr['points'][0]['positions'])),
                end=dict(zip(tr['joint_names'],tr['points'][-1]['positions'])),
                recording_fit=dict(source_snapshot=snapshot_sha[:12]))
            peaks.append(validate(decode(tr),model,params,5.))
        for segment in plan['segments']:
            if segment['kind']=='hand':
                grip=segment['label'].endswith(' hand grip')
                segment.update(hand_target_required=False,duration=1.,
                    finger_sequence='four_then_thumb' if grip else 'thumb_then_four')
        seq=sequence_for(payload,count=1,start=start,finish=finish,leave=True)
        tr,events=materialize(payload,seq)
        plan['export_validation'].update(segments_sha256=digest(plan['segments']),peaks=peaks,
            snapshot_sha256=snapshot_sha,sequence=seq,preview_seconds=time_seconds(tr.points[-1]),
            handEvents=events,physical_verified=False,lowering_reference=old_library['payload']['snapshot_sha256'],
            lowering_bounds=bounds)
        _,actions[name]=planned_action(plan,dict(identity=old['identity']),str(new_path.relative_to(ROOT)))
    bundle=package('gear-bounded-v6-'+digest(lower)[:16],actions,base['payload']['initialization'])
    bundle['payload'].update(mapping=copy.deepcopy(base['payload']['mapping']),parents=[base['sha256']])
    bundle['sha256']=digest(bundle['payload'])
    validate_release(bundle)
    for name in ('push','pull','dring_pair'):
        if bundle['payload']['actions'][name]!=base['payload']['actions'][name]:
            raise ValueError('Unrelated action changed: '+name)
    lib=copy.deepcopy(old_library['payload'])
    lib.update(snapshot_sha256=snapshot_sha,
        bindings={n:actions[n]['source_plan_sha256'] for n in BINDINGS})
    for name in lib['units']:
        tr=payload['units'][name]['trajectory']
        lib['units'][name]=dict(trajectory=tr,sha256=digest(tr))
    lib['validation']=dict(method='bounded-lowering-v6-polynomial-validation',
        peaks={n:validate(decode(v['trajectory']),model,params,5.) for n,v in lib['units'].items()},
        contact_bounds=bounds,physical_verified=False)
    envelope=dict(payload=lib,sha256=digest(lib))
    verified=GearLibrary(envelope,bundle)
    write(stage/'bundle.json',bundle)
    write(stage/'gear_protocol.json',envelope)
    report=dict(version=bundle['payload']['version'],release_sha256=bundle['sha256'],
        library_sha256=verified.sha256,original_release_sha256=base['sha256'],
        bounded_preview=str(new_path.relative_to(ROOT)),reverse_preview=str(reverse_path.relative_to(ROOT)),
        reverse_selected=False,reverse_requires_contact_path_confirmation=True,
        old_lowering_seconds=candidate['original_seconds'],new_lowering_seconds=candidate['selected_seconds'],
        contact_bounds=bounds,other_actions_unchanged=True,hardware_commands_sent=0,deployed=False)
    write(stage/'build_report.json',report)
    print(json.dumps(report,indent=2),flush=True)


if __name__=='__main__':main()
