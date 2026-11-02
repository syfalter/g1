"""Read-only recording-bank timing audit; never constructs a ROS node."""
import argparse
import hashlib
import json
from pathlib import Path
import time
import subprocess
import xml.etree.ElementTree as ET

ROOT=Path(__file__).resolve().parents[1]


def model_file(path):
    import xacro
    from preview_payload import apply_preview_payload
    from task_hand_model import apply_articulated_task_hands
    from task_kinematics import configure_waist
    robot=ET.fromstring(xacro.process_file(str(ROOT/'src/g1_mode6_description/urdf/g1_upper_body.urdf.xacro')).toxml())
    apply_preview_payload(robot,json.loads((ROOT/'assets/workbench/payload_preview.json').read_text()))
    apply_articulated_task_hands(robot);configure_waist(robot)
    ET.ElementTree(robot).write(path,encoding='unicode')
    return path


def bundles(archives=False):
    panel=ROOT/'data/task_panel'
    paths=[p for p in panel.glob('*demonstration*.json') if not p.name.endswith('.selection.json')]
    rails=list((ROOT/'data/task_sessions').glob('*/rail_autosave/config.json'))
    if rails:paths.append(max(rails,key=lambda p:(p.stat().st_mtime_ns,str(p))))
    if archives:
        found=subprocess.run(['rg','-l','--glob','*.json','^  "traces": \\{',str(ROOT/'data')],
            text=True,capture_output=True,check=False)
        if found.returncode not in (0,1):raise RuntimeError(found.stderr)
        paths += [Path(p) for p in found.stdout.splitlines()]
    return sorted(set(paths))


def inventory(paths):
    from task_trace_versions import bank
    from task_trace_selection import restore
    from task_screen_teach import fingerprint
    items={};errors=[]
    for path in paths:
        try:
            config=restore(json.loads(path.read_text()),path)
            if not config.get('traces'):continue
            workflow='rail' if 'rail' in config.get('schema','') else 'landing_gear' if 'landing' in config.get('schema','') else 'dring_demo'
            side=config.get('side','right')
            for slot in config['traces']:
                entry=bank(config,slot)
                for index,trace in enumerate([entry['baseline']]+entry['edits']):
                    if trace is None:continue
                    identity=fingerprint(dict(workflow=workflow,side=side,slot=slot,trace=trace,
                        hands=config.get('hands',{})))
                    alias=dict(file=str(path.relative_to(ROOT)),slot=slot,version=index,selected=index==entry['selected'])
                    if identity in items:items[identity]['aliases'].append(alias)
                    else:items[identity]=dict(id=identity,workflow=workflow,side=side,slot=slot,
                        trace=trace,config={k:config[k] for k in ('hands','ready','side') if k in config},aliases=[alias])
        except Exception as exc:errors.append(dict(file=str(path),error=str(exc)))
    return list(items.values()),errors


def audit_one(item,robot,parameters,denoise):
    from workbench_model import OfflineModel
    from task_trace_timing import generate
    started=time.monotonic();row={k:v for k,v in item.items() if k not in ('trace','config')}
    try:
        edited=generate(item['trace'],OfflineModel(robot),item['side'],parameters,workflow=item['workflow'],
            config=item['config'],slot=item['slot'],denoise=denoise)
        row.update(ok=True,**edited['optimization']['timing_report'])
    except Exception as exc:row.update(ok=False,error=str(exc))
    row['computation_seconds']=time.monotonic()-started
    return row


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--output',type=Path,default=ROOT/'data/trajectory_edit_audit/fast_timing')
    parser.add_argument('--include-archives',action='store_true')
    parser.add_argument('--inventory-only',action='store_true')
    parser.add_argument('--limit',type=int)
    parser.add_argument('--workers',type=int,default=4)
    parser.add_argument('--denoise',action='store_true')
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=True)
    from task_screen_teach import write_json
    items,errors=inventory(bundles(args.include_archives))
    write_json(args.output/'inventory.json',dict(entries=[{k:v for k,v in i.items() if k not in ('trace','config')} for i in items],errors=errors))
    print('unique versions',len(items),'bundle errors',len(errors),flush=True)
    if args.inventory_only:return
    from task_planning_config import load_parameters
    from concurrent.futures import ProcessPoolExecutor,as_completed
    robot=model_file(args.output/'robot.urdf');params,snapshot=load_parameters()
    signature=hashlib.sha256(robot.read_bytes()+repr((params,args.denoise)).encode())
    for name in ('task_fast_timing','task_trace_timing','task_local_timing','task_trajectory_speed',
                 'task_trajectory','task_cruise_timing','task_dring_teach_data','task_rail_trace',
                 'task_trace_curve','task_trace_edit','task_trace_markers','task_planning_config'):
        signature.update((ROOT/'scripts'/f'{name}.py').read_bytes())
    signature=signature.hexdigest();cache=args.output/'results';cache.mkdir(exist_ok=True)
    report=dict(scope='all_recorded_archives' if args.include_archives else 'current_banks_all_versions',
        planning_config=snapshot,hardware_commands_sent=0,entries=[],inventory_errors=errors,
        denoise_enabled=args.denoise,velocity_scaling=.7,acceleration_scaling=.7)
    report['signature']=signature
    with ProcessPoolExecutor(max_workers=args.workers) as pool:
        jobs={}
        for item in items[:args.limit]:
            path=cache/(item['id']+'.json')
            if path.exists():
                cached=json.loads(path.read_text())
                if cached.get('signature')==signature:
                    report['entries'].append(dict(cached['row'],aliases=item['aliases']));continue
            jobs[pool.submit(audit_one,item,robot,params,args.denoise)]=path
        for job in as_completed(jobs):
            row=job.result();write_json(jobs[job],dict(signature=signature,row=row))
            report['entries'].append(row);write_json(args.output/'report.json',report)
            print(json.dumps({k:v for k,v in row.items() if k not in ('aliases','details')},ensure_ascii=False),flush=True)
    write_json(args.output/'report.json',report)
    import csv
    with (args.output/'times.csv').open('w',newline='') as stream:
        writer=csv.writer(stream)
        writer.writerow(['workflow','slot','id','status','original_1x_s','original_fastest_s','preserving_s',
            'smoothed_s','selected_s','speedup_vs_1x','speedup_vs_old_fastest','path_modified','error'])
        for row in report['entries']:
            writer.writerow([row.get(k,'') for k in ('workflow','slot','id','ok','original_1x_seconds',
                'original_fastest_seconds','preserving_seconds','smoothed_seconds','duration_after_s',
                'speedup_over_original_1x','speedup_over_original_fastest','path_modified','error')])


if __name__=='__main__':main()
