"""Read-only adapters from editor selections to reusable offline experiments."""
import copy

from rosidl_runtime_py.convert import message_to_ordereddict

from task_composition_preview import raw_phases, concatenate, fit, select
from task_offline_optimizer import optimize, decode
from task_trajectory import time_seconds


def source_units(workflow, config, ready, model, options, check):
    if 'slot' in options:
        slot=options['slot'];trace=config['traces'][slot]
        source=fit(trace,model,config['side'],check)
        duration=time_seconds(source.points[-1])
        role='transit' if (workflow=='landing_gear' and 'above' in slot or
                           workflow=='rail' and slot.startswith(('approach_','retreat_'))) else 'operation'
        ranges=[(0.,duration,role)]
        if options.get('transitInterval') is not None:
            begin,end=options['transitInterval']
            if not 0<=begin<end<=duration:raise ValueError('接近区间必须位于原轨迹时间范围内')
            ranges=([(0.,begin,'operation')] if begin else [])+[(begin,end,'transit')]+(
                [(end,duration,'operation')] if end<duration else [])
        edit=trace.get('trim_edit',{});seams=[]
        if 'start_s' in edit and 'bridge_s' in edit:
            seams=[t for t in (edit['start_s'],edit['start_s']+edit['bridge_s']) if 0.<t<duration]
        from task_trace_versions import bank
        from task_screen_teach import fingerprint
        return {slot:(source,ranges,seams)},{'single':[slot]},dict(slot=slot,selected_only=True,selectedVersion=bank(config,slot)['selected'],
            selectedSha256=fingerprint(trace),
            production_rail_tcp_policy_applied=False if workflow=='rail' else None,
            origin_workflow=workflow)
    phases,recipes,sources=raw_phases(workflow,config,ready,model,options,check)
    if workflow=='dring_demo':
        units={name:(p['trajectory'],[(0.,time_seconds(p['trajectory'].points[-1]),p['motionClass'])],[])
               for name,p in phases.items()}
    else:
        units={};replacement={}
        for name in ('entry_lower','entry_upper','lu','ul','exit_lower','exit_upper'):
            cursor=0.;ranges=[];parts=[];seams=[]
            for phase in recipes[name]:
                p=phases[phase];end=cursor+time_seconds(p['trajectory'].points[-1])
                seams.extend(cursor+t for t in p.get('stopHints',[]))
                if cursor:seams.append(cursor)
                ranges.append((cursor,end,p['motionClass']));cursor=end;parts.append(p['trajectory'])
            units[name]=(concatenate(parts),ranges,sorted(set(seams)));replacement[name]=[name]
        replacement['cycle_lower']=['lu','ul'];replacement['cycle_upper']=['ul','lu']
        recipes=replacement
    return units,recipes,sources


def comparison(workflow, config, ready, model, parameters, policy, options,
               check=lambda:None, progress=lambda text:None, directory=None):
    if workflow=='rail' and options.get('railCycle'):
        from task_rail_composition import comparison as rail_comparison
        return rail_comparison(config,ready,model,parameters,policy,options,check,progress)
    if workflow=='rail':config=dict(config,side='right')
    units,recipes,sources=source_units(workflow,config,ready,model,options,check)
    context=dict(ready['left']['joints'],**ready['right']['joints'])
    from task_hand_control import model_joints,POSTURES
    for side in ('left','right'):context.update(model_joints(side,POSTURES['index_only']))
    if 'slot' in options:
        from task_trace_markers import hand_context
        trace=config['traces'][options['slot']]
        context.update(trace.get('context_joints',{}))
        context=hand_context(trace,config['side'],context,config,options['slot'])
    elif workflow=='landing_gear':context.update(model_joints('right',config['hands']['open']['feedback_q']))
    results={};hits=[]
    for name,(source,ranges,seams) in units.items():
        progress(name);check()
        kwargs={} if directory is None else dict(directory=directory)
        result,hit=optimize(source,model,config['side'],ranges,policy,parameters,check,
            progress=lambda text:progress(name+' : '+text),seams=seams,**kwargs)
        if options.get('smoothFirst'):
            from task_smooth_timing import cached_candidate
            smooth,smooth_hit=cached_candidate(result,model,config['side'],parameters,policy,ranges,check,
                progress=lambda text:progress(name+' : '+text),max_extra=options.get('smoothExtraFraction',.15))
            result['variants']['smooth']=smooth;hit=hit and smooth_hit
        results[name]=result;hits.append(hit)
    evidence={name:dict(result,variants={key:{k:v for k,v in variant.items() if k!='trajectory'}
              for key,variant in result['variants'].items()}) for name,result in results.items()}
    flows={};flow_recipes=recipes
    if options.get('continuousFlow'):
        from task_stop_blending import cached_flow
        groups=recipes if workflow=='dring_demo' and 'slot' not in options else {name:[name] for name in units}
        if workflow=='dring_demo' and 'slot' not in options:flow_recipes={name:[name] for name in recipes}
        for name,parts in groups.items():
            combined={};offset=0.;flow_ranges=[];source_parts=[]
            for part in parts:
                original,ranges,_=units[part];source_parts.append(original)
                flow_ranges.extend((lo+offset,hi+offset,role) for lo,hi,role in ranges)
                offset+=time_seconds(original.points[-1])
            for variant in ('baseline','candidate')+(('smooth',) if options.get('smoothFirst') else ()):
                tr=concatenate(decode(results[p]['variants'][variant]['trajectory']) for p in parts)
                combined[variant]=dict(trajectory=message_to_ordereddict(tr),seconds=time_seconds(tr.points[-1]))
            flow,hit=cached_flow(dict(variants=combined),concatenate(source_parts),flow_ranges,model,config['side'],
                parameters,policy,check,lambda text:progress(name+' : '+text),options.get('smoothExtraFraction',.15))
            flows[name]=flow;hits.append(hit)
    variants={}
    for variant in ('original','baseline','candidate')+(('smooth',) if options.get('smoothFirst') else ())+(('flow',) if flows else ()):
        saved={}
        for name,sequence in (flow_recipes if variant=='flow' else recipes).items():
            tr=concatenate(decode(flows[p]['trajectory'] if variant=='flow' else results[p]['variants'][variant]['trajectory']) for p in sequence)
            saved[name]=dict(trajectory=message_to_ordereddict(tr),seconds=time_seconds(tr.points[-1]),
                report=dict(offlinePolicy=policy.jerk,parts=sequence))
            if variant=='flow':
                saved[name]['report']['stopAudit']=dict(
                    before=sum(flows[p]['report']['previous_candidate_audit']['count'] for p in sequence),
                    after=sum(flows[p]['report']['after']['count'] for p in sequence))
        variants[variant]=dict(schema=1,workflow=workflow if 'slot' not in options else 'single',
            mode='offline_optimizer',units=saved,context=context,handPresets=config.get('hands',{}),
            sources=sources,options=copy.deepcopy(options),hardware_executable=False,physical_verified=False,
            optimizer=dict(jerk=5. if variant=='original' else policy.jerk,
                           changed=(sum(r['report']['accepted'] for r in flows.values()) if variant=='flow' else
                                    sum(r['variants']['smooth']['report']['accepted'] for r in results.values()) if variant=='smooth'
                                    else sum(r['geometry_selected'] for r in results.values()) if variant=='candidate' else 0),
                           variant=variant,
                           total=len(flows) if variant=='flow' else len(results)),unitEvidence=evidence,
            flowEvidence={name:{k:v for k,v in flow.items() if k!='trajectory'} for name,flow in flows.items()} if variant=='flow' else {})
    return variants,all(hits)


def sequence_for(payload,count=1,start='LOWER',finish='UPPER',leave=False):
    if payload['workflow']=='rail_cycle':return list(payload['sequence'])
    return ['single'] if payload['workflow']=='single' else select(payload,count,start,finish,leave)
