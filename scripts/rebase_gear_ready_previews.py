"""Rebuild only gear entry/exit units for the mirrored shared preparation."""
import argparse
import copy
import json
from pathlib import Path

import numpy as np
from rosidl_runtime_py.convert import message_to_ordereddict

from task_composition_preview import ROOT, materialize
from task_composition_panel import preview_samples
from task_composition_smoothing import evaluate
from task_offline_optimizer_cases import source_units
from task_offline_optimizer import Policy, optimize, decode
from task_optimizer_snapshot import for_case, load, save
from task_planning_config import load_parameters
from task_smooth_timing import cached_candidate, smoothness
from task_stop_blending import cached_flow
from task_trace_selection import restore
from workbench_model import OfflineModel


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--urdf',type=Path,required=True)
    args=parser.parse_args();model=OfflineModel(args.urdf)
    path=for_case('landing_gear',None);saved=load(path,model)
    source=ROOT/'data/task_panel/landing_gear_demonstration.json'
    config=restore(json.loads(source.read_text()),source)
    ready=json.loads((ROOT/'data/task_panel/ready_poses.json').read_text())
    parameters,_=load_parameters();policy=Policy()
    variants={k:copy.deepcopy(v['payload']) for k,v in saved['variants'].items()}
    units,_,sources=source_units('landing_gear',config,ready,model,{},lambda:None)
    for name in ('entry_lower','entry_upper','exit_lower','exit_upper'):
        tr,ranges,seams=units[name]
        progress=lambda text:print(name+': '+text,flush=True)
        result,_=optimize(tr,model,'right',ranges,policy,parameters,progress=progress,seams=seams)
        smooth,_=cached_candidate(result,model,'right',parameters,policy,ranges,progress=progress,max_extra=.15)
        result['variants']['smooth']=smooth
        flow,_=cached_flow(result,tr,ranges,model,'right',parameters,policy,progress=progress,
                           max_extra=.15,compress_idle=True)
        result['variants']['flow']=flow
        for key,payload in variants.items():
            item=result['variants'][key]
            report=dict(parts=[name],offlinePolicy=5.)
            if key=='flow':
                report['stopAudit']=dict(before=flow['report']['previous_candidate_audit']['count'],after=flow['report']['after']['count'])
                payload['flowEvidence'][name]={k:v for k,v in flow.items() if k!='trajectory'}
            payload['units'][name]=dict(trajectory=item['trajectory'],seconds=item['seconds'],report=report)
            payload['context'].update(ready['right']['joints'])
            payload['options']['readyRevision']='mirrored-left-20260928'
            payload['sources']=sources
    results={}
    for key,payload in variants.items():
        sequence=saved['variants'][key]['sequence'];tr,events=materialize(payload,sequence)
        ts,tcp,joint=preview_samples(tr,payload,events,model,'right')
        matrices=[evaluate(tr,ts,k) for k in (1,2,3)]
        results[key]=dict(payload=payload,sequence=sequence,trajectory=tr,events=events,times=ts,tcp=tcp,joint=joint,
            joint_names=list(tr.joint_names),joint_matrix=matrices[0],acceleration_matrix=matrices[1],jerk_matrix=matrices[2],
            acceleration=np.max(abs(matrices[1]),axis=1),jerk=np.max(abs(matrices[2]),axis=1),smoothness=smoothness(tr))
        for name in ('lu','ul','cycle_lower','cycle_upper'):
            assert payload['units'][name]==saved['variants'][key]['payload']['units'][name]
    path=save(results,saved['snapshot_controls'],model)
    print(json.dumps(dict(snapshot=str(path),seconds=results['flow']['smoothness']['seconds'],contact_units_unchanged=True)),flush=True)


if __name__=='__main__':main()
