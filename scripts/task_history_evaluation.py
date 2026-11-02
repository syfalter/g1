#!/usr/bin/env python3
"""Freeze auditable task semantics and byte-exact recovery without modifying history."""
import argparse
from collections import Counter,defaultdict
import copy
import hashlib
from pathlib import Path
import shutil

from task_history_manifest import digest,encoded,inside,read


def semantics(context,directory):
    plan=read(directory/'plan.json') if (directory/'plan.json').is_file() else {}
    labels=tuple(s.get('label') for s in plan.get('segments',[]))
    if plan.get('task_schema')=='index_center_check_v1' or labels==('Hand index_only','Center approach','Center check'):
        return 'center_check',[]
    directions=[d for d in ('left','right') if d.title()+' 120' in labels]
    recorded=context['direction']
    if recorded in ('left','right'):
        if directions and directions!=[recorded]:return 'conflicting_direction',[]
        return 'single_sweep',[recorded]
    if context['recorded_kind']=='task':
        if len(directions)==2:return 'legacy_dual_sweep',directions
        if len(directions)==1:return 'single_sweep',directions
        return 'unknown_task_intent',[]
    return 'not_recorded_as_dring_task',[]


def recovery_index(dataset,inventory):
    names={b.split(':',1)[1] for p in inventory['pairs'] for c in p['contexts']
           for b in c['blockers'] if b.startswith('missing:')}
    index={}
    for name in sorted(names):
        if Path(name).name!=name:raise ValueError('Unsafe missing artifact name')
        for path in sorted(dataset.rglob(name)):
            if path.is_file() and not path.is_symlink():
                index.setdefault((name,digest(path)),str(path.relative_to(dataset)))
    return index


def recoveries(context,directory,index):
    manifest=read(directory/'manifest.json') if (directory/'manifest.json').is_file() else {}
    result={}
    for name,expected in manifest.get('sha256',{}).items():
        if not (directory/name).exists() and (name,expected) in index:
            result[name]=dict(source=index[name,expected],sha256=expected,
                              evidence='byte-exact SHA256 match to original capture manifest')
    return result


def materialize(dataset,context,output):
    """Copy only when a manifest-proven file needs recovery. No state synthesis."""
    source=inside(dataset,context['directory'])
    recovery=context.get('recovery',{})
    if not recovery:return source
    destination=Path(output)/source.parent.name/source.name
    if any(p.is_symlink() for p in source.rglob('*')):raise ValueError('Capture symlinks are not allowed')
    shutil.copytree(source,destination)
    for name,record in recovery.items():
        target=inside(destination,name)
        original=inside(dataset,record['source'])
        if target.exists() or digest(original)!=record['sha256']:
            raise ValueError('Recovery input changed: '+name)
        shutil.copy2(original,target)
    for name in ('ready_poses.json','session_home.json','index_tcp.json'):
        if (source.parent/name).is_file():shutil.copy2(source.parent/name,destination.parent/name)
    (destination/'history_recovery.json').write_bytes(encoded(dict(
        original_dataset=str(dataset.resolve()),original_context=context['directory'],
        original_input_sha256=context['input_sha256'],recovered_files=recovery,
        synthetic_robot_state=False)))
    return destination


def build(inventory,review,refinements=None):
    dataset=Path(inventory['dataset']);result=copy.deepcopy(inventory)
    by_id={row['pair_id']:row for row in review['pairs']}
    if len(by_id)!=len(inventory['pairs']) or set(by_id)!={r['pair_id'] for r in inventory['pairs']}:
        raise ValueError('Incomplete or duplicate visual review')
    refinements=refinements or {};refined={r['number']:r for r in refinements.get('pairs',[])}
    index=recovery_index(dataset,inventory);semantic_counts=Counter();recovery_counts=Counter()
    for pair in result['pairs']:
        visual=copy.deepcopy(by_id[pair['pair_id']])
        if visual['raw_rgb_sha256']!=pair['pair_sha256']['rgb.png']:
            raise ValueError('Review RGB hash mismatch')
        if visual['number'] in refined:
            item=refined[visual['number']]
            if item['pair_id']!=pair['pair_id']:raise ValueError('Refinement pair mismatch')
            visual.update(visual_category=item['visual_category'],refinement_note=item['note'])
        pair['visual_review']=visual
        pair['scene_group']='unregistered_camera_'+pair['camera_calibration_sha256'][:12]
        pair['scene_group_status']='Conservative calibration group, not verified physical scene registration'
        pair['split']='all_history_descriptive; no independent holdout claim'
        entries=[];unknown=[]
        for original in pair['contexts']:
            directory=inside(dataset,original['directory'])
            kind,directions=semantics(original,directory);semantic_counts[kind]+=1
            recovery=recoveries(original,directory,index)
            recovery_counts.update(recovery.keys())
            blockers=set(original['blockers'])
            for name in recovery:blockers.discard('missing:'+name)
            geometry=read(directory/'geometry.json') if (directory/'geometry.json').is_file() else {}
            required=('capture_state.json','cloud.npz','manifest.json','capture.json')
            if (geometry.get('geometry_valid_for_planning_seed') and
                    all((directory/n).is_file() or n in recovery for n in required) and
                    not any(b.startswith(('missing:','checksum:','unsafe_manifest_path:')) for b in blockers)):
                blockers.discard('not_replay_candidate')
            if directions:
                blockers.discard('historical_direction_missing');blockers.discard('task_applicability_unresolved')
            elif kind in ('unknown_task_intent','conflicting_direction'):
                unknown.append(original['directory'])
            else:blockers.add('not_recorded_as_sweep_task')
            for direction in directions or [None]:
                entry=copy.deepcopy(original)
                entry.update(direction=direction,semantics=kind,blockers=sorted(blockers),
                             recovery=recovery,evaluation_selected=False)
                entry['case_id']=hashlib.sha256(encoded([pair['pair_id'],entry['directory'],entry['arm'],direction])).hexdigest()[:20]
                entries.append(entry)
        pair['contexts']=entries
        excluded=(visual['visual_category'] in ('no_target_calibration_board','no_target_floor_or_people')
                  and not any(c['recorded_kind']=='task' for c in entries))
        pair['denominator_retained']=not excluded
        pair['exclusion_evidence']='Raw visual no-target finding and no recorded task context' if excluded else None
        grouped=defaultdict(list)
        for context in entries:
            if context['direction'] in ('left','right'):grouped[context['arm'],context['direction']].append(context)
        tasks=[]
        for (arm,direction),candidates in sorted(grouped.items(),key=lambda item:str(item[0])):
            # Selection is independent of planner outcome, including historic success/failure.
            selected=min(candidates,key=lambda c:(bool(c['blockers']),len(c['blockers']),c['directory']))
            selected['evaluation_selected']=not excluded
            tasks.append(dict(case_id=selected['case_id'],arm=arm,direction=direction,
                              context=selected['directory'],blockers=selected['blockers']))
        pair['required_tasks']=tasks
        pair['pair_blockers']=((['historical_sweep_direction_unrecorded'] if not tasks else [])+
                               (['unresolved_task_intent'] if unknown else [])) if not excluded else []
        pair['unknown_task_contexts']=unknown
        pair['status']=('excluded_no_target' if excluded else
                        'blocked' if pair['pair_blockers'] or any(t['blockers'] for t in tasks) else 'pending')
    retained=[r for r in result['pairs'] if r['denominator_retained']]
    result['schema']='g1.history.evaluation.v1'
    result['policy'].update(
        final_task_denominator_frozen=False,
        coverage_denominator_frozen=True,
        direction='All explicit sweep directions per pair/arm; legacy dual sweep requires both; unknown intent blocks pair success',
        context_selection='Unblocked first, then fewest input blockers, then lexicographic directory; never planner outcome',
        exclusions='Visual no-target and no recorded task evidence; uncertain/occluded retained',
        recovery='Only byte-exact original-manifest SHA256 matches; never current or synthetic robot state',
        split='All-history descriptive benchmark, not independent validation; physical scene grouping remains unresolved',
        final_claim_blockers=['Unresolved task applicability/direction','Physical scene groups not verified',
                              'Current evaluation frozen after initial four-case development'])
    result['summary']=dict(total_pairs=len(result['pairs']),coverage_denominator=len(retained),
        excluded_no_target=len(result['pairs'])-len(retained),
        readiness=dict(Counter(r['status'] for r in result['pairs'])),
        visual_categories=dict(Counter(r['visual_review']['visual_category'] for r in result['pairs'])),
        original_context_semantics=dict(semantic_counts),byte_exact_recoverable_files=dict(recovery_counts),
        selected_tasks=sum(len(r['required_tasks']) for r in retained),
        replayable_contexts=sum(c['evaluation_selected'] and not c['blockers'] for r in retained for c in r['contexts']))
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--inventory',type=Path,required=True)
    parser.add_argument('--review',type=Path,required=True)
    parser.add_argument('--refinements',type=Path)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    expected=args.inventory.with_suffix(args.inventory.suffix+'.sha256').read_text().split()[0]
    if digest(args.inventory)!=expected:parser.error('Frozen inventory changed')
    inventory=read(args.inventory);dataset=Path(inventory['dataset'])
    if args.output.resolve().is_relative_to(dataset.resolve()):parser.error('Cannot overwrite immutable dataset')
    for name,sha in inventory['source_sha256'].items():
        if digest(inside(dataset,name))!=sha:parser.error('Dataset index changed: '+name)
    for row in inventory['pairs']:
        for context in row['contexts']:
            for name,sha in context['input_sha256'].items():
                if digest(inside(dataset,name))!=sha:parser.error('Dataset input changed: '+name)
    result=build(inventory,read(args.review),read(args.refinements) if args.refinements else None)
    result['evidence_sha256']={str(p.resolve()):digest(p) for p in (args.inventory,args.review,args.refinements) if p}
    args.output.parent.mkdir(parents=True,exist_ok=True)
    with args.output.open('xb') as stream:stream.write(encoded(result))
    with args.output.with_suffix(args.output.suffix+'.sha256').open('x') as stream:
        stream.write(digest(args.output)+'  '+args.output.name+'\n')
    print(encoded(result['summary']).decode())


if __name__=='__main__':main()
