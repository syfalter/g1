"""Generate offline composition evidence from saved selections, without ROS nodes."""
import argparse
from pathlib import Path
import time

from task_composition_preview import prepare,select,materialize,ROOT
from task_screen_teach import write_json
from task_trace_selection import restore
from task_planning_config import load_parameters
from workbench_model import OfflineModel


def main():
    import json
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--urdf',type=Path,required=True)
    parser.add_argument('--mode',choices=['original','cruise','fastest'],default='original')
    parser.add_argument('--operation-rate',type=float,default=1.)
    parser.add_argument('--transit-rate',type=float,default=1.)
    parser.add_argument('--smooth',action='store_true')
    parser.add_argument('--entry-valley-refit',action='store_true')
    parser.add_argument('--workflow',choices=['dring_demo','landing_gear'],default='dring_demo')
    args=parser.parse_args()
    if args.entry_valley_refit and (not args.smooth or args.workflow!='landing_gear'):
        parser.error('--entry-valley-refit requires --smooth --workflow landing_gear')
    source=ROOT/'data/task_panel'/('dring_demonstration.json' if args.workflow=='dring_demo' else 'landing_gear_demonstration.json')
    config=restore(json.loads(source.read_text()),source)
    ready=json.loads((ROOT/'data/task_panel/ready_poses.json').read_text())
    params,_=load_parameters();model=OfflineModel(args.urdf)
    options=dict(leftSlot='left_5',rightSlot='right_4') if args.workflow=='dring_demo' else {}
    if args.mode=='fastest':options['transitRate']=args.transit_rate
    if args.smooth:
        if args.mode!='fastest':parser.error('--smooth requires --mode fastest')
        options.update(smooth=True,jointTolerance=.003,tcpTolerance=.001,smoothWindow=.02)
        if args.entry_valley_refit:options['entryValleyRefit']=True
    began=time.monotonic()
    payload,hit,path=prepare(args.workflow,config,ready,model,params,args.mode,args.operation_rate,options,progress=lambda text:print(text,flush=True))
    evidence=dict(cache=str(path),cache_hit=hit,build_seconds=time.monotonic()-began,
                  workflow=args.workflow,mode=args.mode,operation_rate=args.operation_rate,transit_rate=args.transit_rate,
                  phase_cache_hit=payload.get('phaseCacheHit',False),hardware_commands_sent=0,physical_verified=False,counts={})
    tag=args.workflow+'-'+args.mode
    if args.smooth:tag+='-smooth'
    if args.entry_valley_refit:tag+='-valleys-v2'
    if args.operation_rate!=1. or args.transit_rate!=1.:tag+=f'-op{args.operation_rate:g}-transit{args.transit_rate:g}'
    for count in (1,3,10):
        began=time.monotonic();sequence=select(payload,count,leave=True);trajectory,events=materialize(payload,sequence)
        from task_trajectory import time_seconds
        evidence['counts'][str(count)]=dict(units=sequence,seconds=time_seconds(trajectory.points[-1]),
                                          points=len(trajectory.points),assembly_seconds=time.monotonic()-began)
        if count==3:
            write_json(ROOT/'data/composition_candidates'/(tag+'-preview.json'),
                       dict(schema=1,kind='offline_composition_candidate',units=payload,sequence=sequence,
                            handEvents=events,hardware_executable=False,physical_verified=False,cache=str(path)))
    write_json(ROOT/'data/composition_candidates'/(tag+'-evidence.json'),evidence)
    print(json.dumps(evidence,indent=2),flush=True)


if __name__=='__main__':main()
