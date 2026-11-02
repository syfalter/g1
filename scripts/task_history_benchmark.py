#!/usr/bin/env python3
"""Isolated parallel history replay with per-case process-group deadlines.

Full task success remains false until the complete evaluation contract is frozen.
Planner/optional archive-pass metrics are diagnostic, not the requested 95%.
"""
import argparse
from collections import Counter,defaultdict
from concurrent.futures import ThreadPoolExecutor, as_completed
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time
from queue import Queue

from task_history_manifest import digest, encoded, inside, read
from task_history_evaluation import materialize
from task_runtime_provenance import snapshot_runtime, runtime_matches, SCOPE as RUNTIME_SCOPE

ROOT = Path(__file__).resolve().parents[1]
NUMERICAL_THREADS={name:'1' for name in ('OMP_NUM_THREADS','OPENBLAS_NUM_THREADS',
    'MKL_NUM_THREADS','NUMEXPR_NUM_THREADS','VECLIB_MAXIMUM_THREADS','BLIS_NUM_THREADS')}


class GroupResources:
    """Sample only the worker's process group; RSS includes shared mappings per process."""
    def __init__(self,pgid):
        self.pgid=pgid;self.cpu={};self.peak_rss=0;self.samples=0;self.peak_processes=0
        self.clock_ticks=os.sysconf('SC_CLK_TCK');self.page_size=os.sysconf('SC_PAGE_SIZE')

    def sample(self):
        rss=0;count=0
        for path in Path('/proc').glob('[0-9]*/stat'):
            try:
                fields=path.read_text().rsplit(') ',1)[1].split()
                if int(fields[2])!=self.pgid or fields[0]=='Z':continue
                key=(path.parent.name,fields[19])
                self.cpu[key]=max(self.cpu.get(key,0),int(fields[11])+int(fields[12]))
                rss+=int(fields[21])*self.page_size;count+=1
            except (OSError,ValueError,IndexError):continue
        self.samples+=1;self.peak_rss=max(self.peak_rss,rss)
        self.peak_processes=max(self.peak_processes,count)

    def report(self):
        return dict(sampled_cpu_seconds=sum(self.cpu.values())/self.clock_ticks,
                    peak_sampled_group_rss_bytes=self.peak_rss,sample_count=self.samples,
                    peak_sampled_process_count=self.peak_processes,
                    method='Linux /proc process-group samples, nominal 0.25s interval; CPU and peak are lower bounds; RSS is summed, not PSS')


def stop_group(pid, sig):
    try:
        os.killpg(pid, sig)
    except ProcessLookupError:
        pass


def supervise(command, log, budget, cancel=None):
    """Include imports/startup conservatively; reap our group even after normal exit."""
    started = time.monotonic()
    with log.open('xb') as stream:
        # The small exec shim knows the session ID before replay starts any children.
        shim = ('import os,sys; os.environ["G1_OFFLINE_SUPERVISED_PGID"]=str(os.getpgrp()); '
                'os.execvp(sys.argv[1],sys.argv[1:])')
        process = subprocess.Popen([sys.executable, '-c', shim, *command], stdout=stream,
                                   stderr=subprocess.STDOUT, start_new_session=True,
                                   env=dict(os.environ,**NUMERICAL_THREADS))
        resources=GroupResources(process.pid);last_sample=0.
        timed_out = False
        cancelled = False
        try:
            while process.poll() is None:
                if time.monotonic()-last_sample>=.25:
                    resources.sample();last_sample=time.monotonic()
                if cancel is not None and cancel.is_set():
                    cancelled=True
                    break
                remaining=budget-(time.monotonic()-started)
                if remaining<=0:
                    timed_out=True
                    break
                try:process.wait(timeout=min(.1,remaining))
                except subprocess.TimeoutExpired:pass
        finally:
            elapsed = time.monotonic() - started
            resources.sample()
            # No grace period extends the accepted budget; cleanup is reported separately.
            stop_group(process.pid, signal.SIGKILL)
            process.wait()
    return dict(returncode=process.returncode, timed_out=timed_out,cancelled=cancelled,
                worker_wall_seconds=elapsed, cleanup_seconds=time.monotonic()-started-elapsed,
                resources=resources.report(),numerical_threads=dict(NUMERICAL_THREADS))


def run_case(dataset, row, context, output, domain, budget, config, export=False,cancel=None,ik_backend='moveit',
             joint_route_backend='line',preinsert_policy='legacy'):
    began=time.monotonic()
    output.mkdir()
    result = dict(pair_id=row['pair_id'], context=context['directory'],
                  case_id=context.get('case_id'),
                  scene=row['scene_group'], recorded_dates=row['recorded_dates'],
                  direction=context['direction'], arm=context['arm'],
                  ros_domain_id=domain,
                  full_task_success=False, planner_pass=False,
                  archive_gate='pending' if export else 'not_requested', hardware_commands_sent=0)
    # Integrity audit precedes case execution; no planning or detection is cached here.
    for name, expected in context['input_sha256'].items():
        if digest(inside(dataset, name)) != expected:
            raise ValueError('Frozen input changed: ' + name)
    capture = materialize(dataset,context,output/'recovered_inputs')
    result['input_preparation_seconds']=time.monotonic()-began
    remaining=budget-result['input_preparation_seconds']
    if remaining<=0:raise TimeoutError('Input preparation exhausted case budget')
    command = ['bash', str(ROOT/'scripts/run_task_offline_replay.sh'), '--production-planner',
               '--arm', context['arm'], '--sweep-direction', context['direction'],
               '--articulated-hands', '--waist-assist', '--ros-domain-id', str(domain),
               '--budget', str(remaining), '--capture', str(capture),
               '--start-state', str(capture/'capture_state.json'),
               '--output-dir', str(output/'replay'), '--planning-config', str(config), '--ik-backend',ik_backend,
               '--joint-route-backend',joint_route_backend,'--preinsert-policy',preinsert_policy]
    if export:command.append('--export-archive')
    result['command'] = command
    result.update(supervise(command, output/'worker.log', remaining,cancel))
    result['case_wall_seconds']=result['input_preparation_seconds']+result['worker_wall_seconds']
    report_path = output/'replay/report.json'
    try:report = read(report_path) if report_path.is_file() else {}
    except (ValueError,OSError):report={}
    result.update(report=str(report_path), log=str(output/'worker.log'),
                  planning_seconds=report.get('planning_seconds'),
                  rpc_timing=report.get('rpc_timing'), error=report.get('error'),
                  diagnostic=report.get('diagnostic'),
                  model_sha256=digest(output/'replay/robot.urdf') if (output/'replay/robot.urdf').is_file() else None,
                  config_sha256=digest(config))
    stages=[s.get('stage') for s in report.get('segments', [])]
    required=('approach','insert','rotate','withdraw','return_ready')
    result['planner_pass'] = bool(not result['timed_out'] and result['returncode']==0
                                  and report.get('success') and all(s in stages for s in required)
                                  and result['case_wall_seconds']<=budget)
    result['status'] = ('cancelled' if result['cancelled'] else 'timeout' if result['timed_out'] else
                        'planner_pass_archive_pending' if result['planner_pass'] else 'failed')
    result['archive']=report.get('archive')
    if export:
        result['archive_gate']=('reload_verified' if result['planner_pass'] and
                                (result['archive'] or {}).get('reload_verified') else 'failed')
        if result['archive_gate']=='reload_verified':result['status']='archive_reload_pass'
    geometry=[s.get('geometry_validation') for s in report.get('segments',[])
              if s.get('stage') in ('insert','rotate','withdraw')]
    validated=[s.get('validation') for s in report.get('segments',[]) if s.get('kind')=='arm']
    result['accepted_geometry_summaries']=geometry
    result['accepted_joint_summaries']=validated
    result['complete_archive_pass']=result['archive_gate']=='reload_verified'
    if not result['planner_pass']:
        if not result['error']:
            lines=(output/'worker.log').read_text(errors='replace').splitlines()
            result['error']=lines[-1] if lines else 'Worker exited without a report'
        diagnostic=result.get('diagnostic') or {}
        result['failure_category']=('deadline' if result['timed_out'] else
            'input' if not report else 'startup' if str(result['error']).startswith('Unavailable ') else
            'rpc_timeout' if 'ROS response timed out' in str(result['error']) else
            diagnostic.get('code','planning_or_export'))
    (output/'result.json').write_bytes(encoded(result))
    return result


def coverage_metrics(manifest,results):
    """One pair succeeds only if every frozen direction succeeds; missing work is failure."""
    retained=[p for p in manifest['pairs'] if p['denominator_retained']]
    by_case={r['case_id']:r for r in results if r.get('case_id')}
    passed=[];groups={'scene':defaultdict(list),'date':defaultdict(list)}
    for pair in retained:
        tasks=pair.get('required_tasks',[])
        success=bool(tasks and not pair.get('pair_blockers') and all(
            not t['blockers'] and by_case.get(t['case_id'],{}).get('complete_archive_pass') for t in tasks))
        if success:passed.append(pair['pair_id'])
        groups['scene'][pair.get('scene_group') or 'unresolved'].append(success)
        for date in pair['recorded_dates'] or ['unknown']:groups['date'][date].append(success)
    def rate(values):
        return dict(total=len(values),passed=sum(values),rate=sum(values)/len(values) if values else None)
    directions=defaultdict(list)
    for pair in retained:
        for task in pair.get('required_tasks',[]):
            directions[task['direction']].append(bool(not task['blockers'] and
                by_case.get(task['case_id'],{}).get('complete_archive_pass')))
        if pair.get('pair_blockers'):directions['unresolved'].append(False)
    return dict(denominator=len(retained),complete_archive_pairs=len(passed),
                conservative_coverage_rate=len(passed)/len(retained) if retained else None,
                final_95_claim_eligible=manifest['policy'].get('final_task_denominator_frozen',False),
                metric='Diagnostic conservative coverage; unresolved and unevaluated pairs count as failures',
                by_direction_task={key:rate(values) for key,values in sorted(directions.items())},
                **{'by_'+name:{key:rate(values) for key,values in sorted(group.items())} for name,group in groups.items()})


def parallel_cases(selected,domains,run,cancel):
    """Lease each domain to one worker until the entire process group is reaped."""
    if not domains or len(set(domains))!=len(domains) or any(d not in range(93,100) for d in domains):
        raise ValueError('Need unique offline domains in 93..99')
    leases=Queue()
    for domain in domains:leases.put(domain)
    queued=time.monotonic()
    def invoke(index,item):
        if cancel.is_set():return None
        domain=leases.get()
        began=time.monotonic()
        try:
            result=run(index,item,domain)
            result.update(queue_wait_seconds=began-queued,worker_count=len(domains))
            return index,result
        finally:leases.put(domain)
    pool=ThreadPoolExecutor(max_workers=len(domains),thread_name_prefix='offline-domain')
    futures=[pool.submit(invoke,index,item) for index,item in enumerate(selected)]
    try:
        for future in as_completed(futures):
            item=future.result()
            if item is not None:yield item
    finally:
        cancel.set()
        for future in futures:future.cancel()
        pool.shutdown(wait=True,cancel_futures=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--context', action='append', help='Explicit subset by frozen context path')
    parser.add_argument('--domain', type=int, choices=range(93,100), default=94)
    parser.add_argument('--workers',type=int,default=1,help='Independent contiguous domains starting at --domain')
    parser.add_argument('--budget', type=float, default=180.)
    parser.add_argument('--export-archive', action='store_true')
    parser.add_argument('--planning-config', type=Path, default=ROOT/'config/task_planning.yaml')
    parser.add_argument('--ik-backend',choices=('moveit','bounded','bounded-task','constrained'),default='moveit')
    parser.add_argument('--joint-route-backend',choices=('line','ompl'),default='line')
    parser.add_argument('--preinsert-policy',choices=('legacy','diverse'),default='legacy')
    args = parser.parse_args()
    if not 1<=args.workers<=7 or args.domain+args.workers-1>99:
        parser.error('Workers must fit distinct domains in 93..99')
    if not math.isfinite(args.budget) or not 0<args.budget<=180:
        parser.error('Budget must be positive and at most 180 seconds')
    expected=args.manifest.with_suffix(args.manifest.suffix+'.sha256').read_text().split()[0]
    if digest(args.manifest)!=expected:
        parser.error('Frozen manifest checksum mismatch')
    manifest=read(args.manifest)
    dataset=Path(manifest['dataset'])
    for name, sha in manifest['source_sha256'].items():
        if digest(inside(dataset,name))!=sha:
            parser.error('Dataset index changed: '+name)
    selected=[]
    requested=set(args.context or [])
    for row in manifest['pairs']:
        for context in row['contexts']:
            if requested and context['directory'] not in requested:
                continue
            if not requested and not context.get('evaluation_selected',True):continue
            if context['blockers']:
                if context['directory'] in requested:
                    parser.error('Selected context blocked: '+str(context['blockers']))
                continue
            selected.append((row,context))
    if requested-set(c['directory'] for _,c in selected):
        parser.error('Unknown selected context')
    args.output=args.output.resolve()
    if args.output.is_relative_to(dataset.resolve()):
        parser.error('Output must be outside dataset')
    args.output.mkdir(parents=True,exist_ok=False)
    config=args.planning_config.resolve()
    (args.output/'planning.yaml').write_bytes(config.read_bytes())
    config=args.output/'planning.yaml'
    code=args.output/'code_snapshot';code.mkdir()
    for path in (ROOT/'scripts').iterdir():
        if path.suffix in ('.py','.sh','.cpp','.so') and path.is_file():
            (code/path.name).write_bytes(path.read_bytes())
    runtime=snapshot_runtime(args.output/'runtime_snapshot')
    summary=dict(manifest_sha256=expected, total_inventory_pairs=len(manifest['pairs']),
                 scope='explicit_subset' if requested else 'all_unblocked_contexts', worker_count=args.workers,
                 domains=list(range(args.domain,args.domain+args.workers)),
                 host=dict(machine=os.uname().machine,cpu_count=os.cpu_count(),
                           available_cpus=len(os.sched_getaffinity(0)),
                           physical_memory_bytes=os.sysconf('SC_PAGE_SIZE')*os.sysconf('SC_PHYS_PAGES')),
                 code_sha256={p.name:digest(p) for p in code.iterdir()},ik_backend=args.ik_backend,
                 runtime_sha256=runtime,runtime_snapshot_scope=RUNTIME_SCOPE,
                 joint_route_backend=args.joint_route_backend,
                 preinsert_policy=args.preinsert_policy,
                 selected_pairs=len({r['pair_id'] for r,_ in selected}), selected_contexts=len(selected),
                 full_task_success_pairs=0, full_task_rate=None, results=[])
    summary['coverage']=coverage_metrics(manifest,[])
    (args.output/'summary.json').write_bytes(encoded(summary))
    cancel=threading.Event()
    def interrupted(signum,frame):
        cancel.set()
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM,interrupted)
    def run(index,item,domain):
        row,context=item
        print(f'[{index+1}/{len(selected)} domain={domain}] {context["directory"]}',flush=True)
        directory=args.output/f'{index:04d}'
        try:
            for name,sha in summary['code_sha256'].items():
                if digest(ROOT/'scripts'/name)!=sha:raise ValueError('Code changed during frozen batch: '+name)
            if not runtime_matches(runtime):raise ValueError('Native runtime changed during frozen batch')
            result=run_case(dataset,row,context,directory,domain,args.budget,config,args.export_archive,cancel,
                            args.ik_backend,args.joint_route_backend,args.preinsert_policy)
            result['runtime_integrity_verified']=runtime_matches(runtime)
            result['code_integrity_verified']=(result['runtime_integrity_verified'] and
                all(digest(ROOT/'scripts'/n)==sha for n,sha in summary['code_sha256'].items()))
            if not result['code_integrity_verified']:
                result.update(status='code_changed',planner_pass=False,complete_archive_pass=False,
                              archive_gate='invalidated_by_code_change')
            return result
        except Exception as error:
            directory.mkdir(exist_ok=True)
            return dict(pair_id=row['pair_id'],context=context['directory'],case_id=context.get('case_id'),
                        direction=context['direction'],arm=context['arm'],ros_domain_id=domain,
                        status='worker_exception',error=str(error),full_task_success=False,
                        planner_pass=False,complete_archive_pass=False,hardware_commands_sent=0)
    started=time.monotonic()
    try:
        for index,result in parallel_cases(selected,summary['domains'],run,cancel):
            (args.output/f'{index:04d}'/'result.json').write_bytes(encoded(result))
            summary['results'].append(result)
            summary['results'].sort(key=lambda r:(r['pair_id'],r['context']))
            summary['batch_wall_seconds']=time.monotonic()-started
            summary['coverage']=coverage_metrics(manifest,summary['results'])
            summary['status_counts']=dict(Counter(r['status'] for r in summary['results']))
            temporary=args.output/'summary.tmp'
            temporary.write_bytes(encoded(summary));temporary.replace(args.output/'summary.json')
            print(f'[{index+1}/{len(selected)}] {result["status"]}',flush=True)
    except KeyboardInterrupt:
        print('Cancelled; offline worker groups have been terminated.',flush=True)
        raise SystemExit(130)


if __name__=='__main__':
    main()
