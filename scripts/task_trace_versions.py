"""Portable immutable baselines and ten explicitly managed edit slots."""
import copy

EDIT_SLOTS = 10


def validate(config, validate_trace):
    entries = config.get('trace_versions', {})
    if not isinstance(entries, dict) or not set(entries) <= set(config['traces']):
        raise ValueError('Version banks must belong to existing trajectory slots')
    for slot in entries:
        entry = bank(config, slot)
        if not isinstance(entry['edits'], list):raise ValueError('Invalid version slots')
        for trace in [entry['baseline']]+[v for v in entry['edits'] if v is not None]:
            validate_trace(trace)
        selected = entry['baseline'] if entry['selected'] == 0 else entry['edits'][entry['selected']-1]
        if selected is None or selected != config['traces'][slot]:
            raise ValueError('Selected trajectory does not match its stored version')


def bank(config, slot):
    existing = config.get('trace_versions', {}).get(slot)
    if existing is not None:
        if (not isinstance(existing.get('edits'),list) or len(existing['edits']) not in (5,EDIT_SLOTS) or
                type(existing.get('selected')) is not int or not 0 <= existing['selected'] <= len(existing['edits'])):
            raise ValueError('Invalid trajectory version bank')
        if len(existing['edits']) < EDIT_SLOTS:
            return dict(existing,edits=existing['edits']+[None]*(EDIT_SLOTS-len(existing['edits'])))
        return existing
    return dict(baseline=config['traces'][slot], edits=[None]*EDIT_SLOTS, selected=0)


def select(config, slot, index):
    result = copy.deepcopy(config)
    entry = copy.deepcopy(bank(config, slot))
    if type(index) is not int or not 0 <= index <= EDIT_SLOTS:
        raise ValueError('Invalid version index')
    trace = entry['baseline'] if index == 0 else entry['edits'][index-1]
    if trace is None:
        raise ValueError('Selected version is empty')
    entry['selected'] = index
    result.setdefault('trace_versions', {})[slot] = entry
    result['traces'][slot] = copy.deepcopy(trace)
    return result


def select_shared(config,slot,index):
    """Internal read-only trace projection; copy mappings, never sample arrays.

    Callers must replace/deepcopy a trace before editing its contents. Recording
    and editing already do this; public select() retains its independent-copy API.
    """
    entry=dict(bank(config,slot))
    if type(index) is not int or not 0<=index<=EDIT_SLOTS:raise ValueError('Invalid version index')
    trace=entry['baseline'] if index==0 else entry['edits'][index-1]
    if trace is None:raise ValueError('Selected version is empty')
    entry['edits']=list(entry['edits']);entry['selected']=index
    return dict(config,traces=dict(config['traces'],**{slot:trace}),
                trace_versions=dict(config.get('trace_versions',{}),**{slot:entry}))


def append(config, slot, trace):
    entry = copy.deepcopy(bank(config, slot))
    try:
        index = entry['edits'].index(None)
    except ValueError:
        raise ValueError(f'{EDIT_SLOTS} 个修改位置已满，请先删除一个版本') from None
    entry['edits'][index] = copy.deepcopy(trace)
    result = copy.deepcopy(config)
    result.setdefault('trace_versions', {})[slot] = entry
    return select(result, slot, index+1)


def delete(config, slot, index):
    if type(index) is not int or not 1 <= index <= EDIT_SLOTS:
        raise ValueError('The baseline cannot be deleted')
    result = copy.deepcopy(config)
    entry = copy.deepcopy(bank(config, slot))
    if entry['selected'] == index:
        entry['selected'] = 0
        result['traces'][slot] = copy.deepcopy(entry['baseline'])
    entry['edits'][index-1] = None
    result.setdefault('trace_versions', {})[slot] = entry
    return result
