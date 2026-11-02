"""Read-only timing benchmark; no ROS nodes or recording-bank writes."""
import argparse
import cProfile
import json
import time
from pathlib import Path


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--bundle',type=Path,required=True)
    parser.add_argument('--slot',required=True)
    parser.add_argument('--version',type=int,default=0)
    parser.add_argument('--model',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--repeat',type=int,default=1)
    parser.add_argument('--profile',action='store_true')
    parser.add_argument('--baseline-modules',type=Path)
    parser.add_argument('--cache-dir',type=Path)
    args=parser.parse_args()
    if args.baseline_modules:
        import sys
        sys.path.insert(0,str(args.baseline_modules))
    from task_trace_versions import bank
    from task_trace_timing import generate
    from task_planning_config import load_parameters
    from workbench_model import OfflineModel
    config=json.loads(args.bundle.read_text())
    entry=bank(config,args.slot)
    trace=([entry['baseline']]+entry['edits'])[args.version]
    workflow='rail' if 'rail' in config.get('schema','') else 'landing_gear' if 'landing' in config.get('schema','') else 'dring_demo'
    model=OfflineModel(args.model);parameters,_=load_parameters()
    args.output.mkdir(parents=True,exist_ok=True)
    results=[]
    for index in range(args.repeat):
        profiler=cProfile.Profile() if args.profile else None
        started=time.monotonic()
        if profiler:profiler.enable()
        options={}
        if not args.baseline_modules:
            options=dict(cache_dir=args.cache_dir,progress=lambda text:print(text,flush=True))
        edited=generate(trace,model,config.get('side','right'),parameters,
            workflow=workflow,config=config,slot=args.slot,denoise=True,**options)
        if profiler:
            profiler.disable();profiler.dump_stats(str(args.output/f'profile_{index}.pstats'))
        elapsed=time.monotonic()-started
        result=dict(elapsed_s=elapsed,**edited['optimization']['timing_report'])
        results.append(result)
        (args.output/'results.json').write_text(json.dumps(results,ensure_ascii=False,indent=2)+'\n')
        (args.output/f'candidate_{index}.json').write_text(json.dumps(edited,ensure_ascii=False)+'\n')
        print(json.dumps(result,ensure_ascii=False),flush=True)


if __name__=='__main__':main()
