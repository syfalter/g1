"""Batch offline evidence for selected traces and reusable business units."""
import argparse
import json
from pathlib import Path
import time

from task_composition_preview import ROOT,materialize
from task_offline_optimizer import Policy,validate
from task_offline_optimizer_cases import comparison,sequence_for
from task_planning_config import load_parameters
from task_screen_teach import write_json
from task_trace_selection import restore
from task_trace_versions import bank,select_shared
from task_trajectory import time_seconds
from workbench_model import OfflineModel


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--urdf',type=Path,required=True)
    parser.add_argument('--workflow',choices=('dring_demo','landing_gear','rail'),required=True)
    parser.add_argument('--source',type=Path)
    parser.add_argument('--slot',help='One selected trace, or all for every standalone trace')
    parser.add_argument('--version',type=int)
    parser.add_argument('--jerk',type=float,default=5.)
    parser.add_argument('--contact-mm',type=float,default=1.)
    parser.add_argument('--transit-mm',type=float,default=5.)
    parser.add_argument('--smooth-first',action='store_true')
    parser.add_argument('--continuous-flow',action='store_true')
    parser.add_argument('--smooth-extra-fraction',type=float,default=.15)
    args=parser.parse_args()
    if args.continuous_flow:args.smooth_first=True
    if args.workflow=='rail' and (args.source is None or args.slot is None):parser.error('Rail requires --source and --slot')
    if args.version is not None and args.slot in (None,'all'):parser.error('--version requires one --slot')
    source=args.source or ROOT/'data/task_panel'/('dring_demonstration.json' if args.workflow=='dring_demo' else 'landing_gear_demonstration.json')
    config=restore(json.loads(source.read_text()),source)
    if args.version is not None:config=select_shared(config,args.slot,args.version)
    ready=json.loads((ROOT/'data/task_panel/ready_poses.json').read_text())
    model=OfflineModel(args.urdf);parameters,_=load_parameters()
    policy=Policy(jerk=args.jerk,contact_mm=args.contact_mm,transit_mm=args.transit_mm)
    for slot in list(config['traces']) if args.slot=='all' else [args.slot]:
        options=dict(smoothFirst=True,smoothExtraFraction=args.smooth_extra_fraction) if args.smooth_first else {}
        if args.continuous_flow:options['continuousFlow']=True
        if slot:options['slot']=slot
        elif args.workflow=='dring_demo':
            for direction in ('left','right'):
                candidates=[]
                for name in config['traces']:
                    if name.startswith(direction+'_'):
                        entry=bank(config,name)
                        candidates.extend((name,i) for i,t in enumerate([entry['baseline']]+entry['edits'])
                            if t and t.get('optimization',{}).get('kind')=='trim_tail')
                if not candidates:raise ValueError('No trimmed prefix for '+direction)
                name,index=candidates[-1];config=select_shared(config,name,index);options[direction+'Slot']=name
        began=time.monotonic()
        payloads,hit=comparison(args.workflow,config,ready,model,parameters,policy,options,
            progress=lambda text:print(text,flush=True))
        evidence=dict(workflow=args.workflow,slot=slot,source=str(source),jerk=args.jerk,cache_hit=hit,
            computation_seconds=time.monotonic()-began,hardware_commands_sent=0,physical_verified=False,variants={})
        version=bank(config,slot)['selected'] if slot else None;evidence['selected_version']=version
        identity=f'{slot}-version{version}' if slot else 'combination'
        tag=f'{args.workflow}-{identity}-jerk{args.jerk:g}-contact{args.contact_mm:g}-transit{args.transit_mm:g}-v3'
        if args.smooth_first:tag=tag[:-2]+f'v4-extra{args.smooth_extra_fraction:g}'
        if args.continuous_flow:tag=tag.replace('-v4-extra','-v5-extra')
        for variant,payload in payloads.items():
            counts={}
            for count in (1,) if slot else (1,3,10):
                began=time.monotonic();sequence=sequence_for(payload,count,leave=True);tr,events=materialize(payload,sequence)
                counts[str(count)]=dict(seconds=time_seconds(tr.points[-1]),assembly_seconds=time.monotonic()-began)
                if count==1:
                    peaks=validate(tr,model,parameters,5. if variant=='original' else args.jerk)
                    from task_smooth_timing import smoothness
                    metrics=smoothness(tr)
                    write_json(ROOT/'data/composition_candidates'/(tag+'-'+variant+'.json'),dict(schema=1,
                        kind='offline_composition_candidate',units=payload,sequence=sequence,handEvents=events,
                        hardware_executable=False,physical_verified=False,cache='offline_optimizer_cache'))
            evidence['variants'][variant]=dict(counts=counts,peaks=peaks,smoothness=metrics)
        write_json(ROOT/'data/composition_candidates'/(tag+'-evidence.json'),evidence)
        print(json.dumps(evidence,ensure_ascii=False),flush=True)


if __name__=='__main__':main()
