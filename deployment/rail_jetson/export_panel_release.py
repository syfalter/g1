"""Freeze persisted panel selections and plan a data-only release; no ROS nodes."""
import argparse
import copy
from datetime import datetime
import hashlib
from pathlib import Path
import sys

from bundle import ROOT, read, write
from releases import package, planned_action

sys.path.insert(0, str(ROOT / 'scripts'))


def export(urdf, output, resume=False):
    from rosidl_runtime_py.convert import message_to_ordereddict
    from workbench_model import OfflineModel
    from task_planning_config import load_parameters, parameters_from_values
    from task_hand_control import model_joints
    from task_trace_selection import restore, revision
    from task_dring_teach_data import build as dring_build, moving
    from task_landing_gear_data import build as gear_build, SLOTS
    from task_rail_trace import build_replay

    output = Path(output).resolve()
    output.mkdir(parents=True, exist_ok=resume)
    panel = ROOT / 'data/task_panel'
    rails = list((ROOT / 'data/task_sessions').glob('*/rail_autosave/config.json'))
    if not rails:
        raise ValueError('No persisted rail selection')
    sources = dict(dring=panel / 'dring_demonstration.json',
                   gear=panel / 'landing_gear_demonstration.json',
                   rail=max(rails, key=lambda p: (p.stat().st_mtime_ns, str(p))))
    configs, manifest = {}, {}
    for name, path in sources.items():
        if resume:
            configs[name] = read(output / (name + '_snapshot.json'))
            manifest[name] = dict(frozen_snapshot=str(output / (name + '_snapshot.json')))
            continue
        before = revision(path)
        config = restore(read(path), path)
        if revision(path) != before:
            raise RuntimeError('Panel changed while freezing: ' + str(path))
        manifest[name] = dict(source=str(path), revision=before,
                              selected={k:v['selected'] for k,v in config.get('trace_versions', {}).items()})
        config.pop('trace_versions', None)
        configs[name] = config
        write(output / (name + '_snapshot.json'), config)
    ready = read(output / 'ready_poses.json' if resume else panel / 'ready_poses.json')
    model = OfflineModel(urdf)
    model_sha = hashlib.sha256(Path(urdf).read_bytes()).hexdigest()
    for config in configs.values():
        identity = config['identity']
        if identity.get('robot_sha256', identity.get('urdf_sha256')) != model_sha:
            raise ValueError('Panel model differs from selected URDF')
    if resume:
        record = read(output / 'planning_config.json')
        parameters = parameters_from_values(record['values'])
    else:
        parameters, record = load_parameters()
    write(output / 'planning_config.json', record)
    write(output / 'ready_poses.json', ready)
    target = {n:ready[side]['joints'][n] for side in ('left', 'right') for n in moving(side) if n != 'waist_yaw_joint'}
    target['waist_yaw_joint'] = read(ROOT / 'config/jetson_push.json')['waist_target_rad']
    initialization = dict(mode='simultaneous', velocity_scale=.7, acceleration_scale=.7, target=target)
    start = dict(ready['left']['joints'], **ready['right']['joints'])
    for side in ('left', 'right'):
        start.update(model_joints(side, configs['rail']['hands']['open']['feedback_q']))
    actions, errors = {}, {}

    def save(plan, config, workflow):
        plan['planning_config'] = copy.deepcopy(record)
        plan['directory'] = str(output)
        proof = plan['rail'] if plan['kind'] == 'rail' else plan['demo']
        if plan['kind'] == 'rail':
            proof['path_source'] = 'recorded'
        else:
            proof.update(workflow=workflow, arm_only=workflow == 'dring_demo')
        frozen = {**plan, 'segments': [{**s, 'trajectory':message_to_ordereddict(s['trajectory'])} for s in plan['segments']]}
        name, action = planned_action(frozen, config, str(output.relative_to(ROOT)) + '/' + proof['operation'])
        if name in actions:
            raise ValueError('Ambiguous action selection: ' + name)
        actions[name] = action
        write(output / (name + '.plan.json'), frozen)
        print(name, 'planned', round(sum(s['duration'] for s in plan['segments']), 3), 'seconds', flush=True)

    config = configs['dring']
    for direction in ('left', 'right'):
        slots = [slot for slot in config['traces'] if slot.startswith(direction + '_')]
        if len(slots) != 1:
            raise ValueError('Select exactly one persisted ' + direction + ' D-ring slot; found ' + str(slots))
        selected = copy.deepcopy(config)
        selected['ready']['hand_q'] = None
        for trace in selected['traces'].values():
            trace['hand'] = None
        existing = output / ('dring_' + direction + '.plan.json')
        if resume and existing.exists():
            frozen = read(existing)
            name, action = planned_action(frozen, selected, str(existing.relative_to(ROOT)))
            actions[name] = action
            continue
        plan = dring_build(model, selected, start, slots[0], parameters, .7, .7, 1.,
                           preparation=ready[config['side']]['joints'], full_cycle=True,
                           cache_dir=output / 'dring_cache', progress=lambda text:print(text, flush=True))
        save(plan, selected, 'dring_demo')
    for operation in ('push', 'pull'):
        try:
            plan = build_replay(model, configs['rail'], ready['right']['joints'], start, operation, 'cycle', parameters,
                                cache_dir=output / 'rail_cache', progress=lambda text:print(text, flush=True))
            save(plan, configs['rail'], 'rail')
        except ValueError as error:
            errors[operation] = str(error)
            print(operation, 'BLOCKED:', error, flush=True)
    for operation in SLOTS:
        try:
            if operation not in configs['gear']['traces']:
                raise ValueError('Missing current gear recording: ' + operation)
            plan = gear_build(model, configs['gear'], start, operation, parameters, .7, .7, 1.,
                              cache_dir=output / 'gear_cache', progress=lambda text:print(text, flush=True))
            save(plan, configs['gear'], 'landing_gear')
        except ValueError as error:
            errors['gear_' + operation] = str(error)
            print(operation, 'BLOCKED:', error, flush=True)
    for name, path in sources.items():
        if 'revision' in manifest[name]:
            manifest[name]['changed_after_snapshot'] = revision(path) != manifest[name]['revision']
    version = 'panel-' + datetime.now().strftime('%Y%m%d-%H%M%S')
    envelope = package(version, actions, initialization)
    write(output / 'bundle.json', envelope)
    write(output / 'sources.json', dict(sources=manifest, version=version, sha256=envelope['sha256'],
                                      actions=list(actions), blocked_actions=errors,
                                      complete=not errors, hardware_commands_sent=0))
    print('BUNDLE', output / 'bundle.json', flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--urdf', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--resume', action='store_true', help='Use existing frozen snapshots, never reread live panel data')
    args = parser.parse_args()
    export(args.urdf, args.output, args.resume)
