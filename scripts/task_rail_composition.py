"""Full rail cycles assembled from pinned optimizer paths, without robot I/O."""
import copy
import json

from rosidl_runtime_py.convert import message_to_ordereddict

from task_composition_preview import concatenate
from task_offline_optimizer import Policy, decode, validate
from task_optimizer_snapshot import for_case, load
from task_trajectory import time_seconds

CYCLES = {'push_cycle': ('approach_A', 'push', 'retreat_B'),
          'pull_cycle': ('approach_B', 'pull', 'retreat_A')}


def comparison(config, ready, model, parameters, policy, options,
               check=lambda: None, progress=lambda text: None):
    from task_hand_control import model_joints
    from task_path_smoothing import joint_line_trajectory
    from task_offline_optimizer import retime
    from task_stop_blending import cached_flow
    from task_screen_teach import fingerprint
    from task_trace_versions import bank
    from task_rail_trace import close_boundary

    operation = options['railCycle']
    stages = CYCLES[operation]
    pinned = {}
    sources = {}
    for stage in stages:
        path = for_case('rail', stage)
        if path is None:
            raise ValueError('Missing pinned rail path: ' + stage)
        saved = load(path, model)['variants']
        source = saved['flow']['payload']['sources']
        if (source['selectedVersion'] != bank(config, stage)['selected'] or
                source['selectedSha256'] != fingerprint(config['traces'][stage])):
            raise ValueError('Rail snapshot differs from selected trace: ' + stage)
        if saved['flow']['payload']['optimizer']['jerk'] != policy.jerk:
            raise ValueError('Regenerate the single rail paths for the selected jerk first')
        pinned[stage] = saved
        sources[stage] = dict(source, snapshot=json.loads(path.read_text())['sha256'])

    context = dict(ready['left']['joints'], **ready['right']['joints'])
    context.update(model_joints('right', config['hands']['open']['feedback_q']))
    names = pinned[stages[0]]['flow']['trajectory'].joint_names
    home = [context[n] for n in names]
    output = {}
    # Rebuild each comparison from its matching unit version; no regeneration of
    # the reviewed paths, and no additional per-unit geometry tolerance is spent.
    for variant in ('original', 'baseline', 'candidate', 'smooth', 'flow'):
        units, sequence, waits, ranges = {}, [], {}, {}
        current = list(home)
        parts, roles = [], []
        group = 0
        last_posture = None

        def flush():
            nonlocal parts, roles, group
            if not parts:
                return
            name = 'motion_' + str(group)
            tr = concatenate(parts)
            units[name] = dict(trajectory=message_to_ordereddict(tr), seconds=time_seconds(tr.points[-1]), report={})
            ranges[name] = roles
            sequence.append(name)
            group += 1
            parts, roles = [], []

        def add(tr, role):
            nonlocal current
            offset = sum(time_seconds(p.points[-1]) for p in parts)
            roles.append((offset, offset + time_seconds(tr.points[-1]), role))
            parts.append(tr)
            current = list(tr.points[-1].positions)

        def connect(target, boundary=False):
            if max(abs(a-b) for a,b in zip(current, target)) < 1e-9:
                return
            if boundary:
                close_boundary(model, dict(context, **dict(zip(names, current))),
                               dict(context, **dict(zip(names, target))))
            line = joint_line_trajectory(names, current, target)
            # The join is explicitly part of the full-cycle preview and export.
            line, _ = retime(line, model, parameters, policy.jerk, check)
            add(line, 'transit' if not boundary else 'operation')

        def hand(posture):
            nonlocal last_posture
            if posture == last_posture:
                return
            last_posture = posture
            flush()
            name = 'hand_' + str(len(waits))
            tr = joint_line_trajectory(names, current, current)
            from task_screen_motion import scale_time
            scale_time(tr, 1. / time_seconds(tr.points[-1]))
            units[name] = dict(trajectory=message_to_ordereddict(tr), seconds=1., report={})
            waits[name] = dict(side='right', posture=posture, duration=1.)
            sequence.append(name)

        hand('grip' if operation == 'push_cycle' else 'open')
        for stage in stages:
            check()
            if stage == 'pull':
                hand('pull_grip' if 'pull_grip' in config['hands'] else 'grip')
            if stage == 'retreat_A':
                hand('open')
            tr = pinned[stage][variant]['trajectory']
            connect(tr.points[0].positions, boundary=stage != stages[0])
            add(tr, 'transit' if stage.startswith(('approach_', 'retreat_')) else 'operation')
        connect(home)
        flush()
        hand('open')
        payload = dict(schema=1, workflow='rail_cycle', mode='offline_optimizer', units=units,
                       context=context, handPresets=copy.deepcopy(config['hands']), handWaits=waits,
                       sequence=sequence, sources=sources, options=copy.deepcopy(options),
                       hardware_executable=False, physical_verified=False,
                       optimizer=dict(variant=variant, jerk=policy.jerk, changed=0, total=len(ranges)), unitEvidence={}, flowEvidence={})
        if variant == 'baseline':
            baseline_ranges = ranges
        if variant == 'flow':
            # Optimize across joins starting from the original-path baseline,
            # not from already displaced V5 paths (which would spend tolerance twice).
            for name in ranges:
                progress(operation + ' / ' + name)
                variants = {key: output[key]['units'][name] for key in ('baseline', 'candidate', 'smooth')}
                # Compare against the assembled V5 single paths as incumbent.
                variants['smooth'] = units[name]
                flowed, _ = cached_flow(dict(variants=variants), decode(variants['baseline']['trajectory']),
                    baseline_ranges[name], model, 'right', parameters, policy, check, progress,
                    options.get('smoothExtraFraction', .15),compress_idle=True)
                units[name] = {k: flowed[k] for k in ('trajectory', 'seconds', 'report')}
                units[name]['report']['stopAudit'] = dict(before=flowed['report']['previous_candidate_audit']['count'],
                    after=flowed['report']['after']['count'])
                payload['optimizer']['changed'] += int(flowed['report']['accepted'])
                payload['flowEvidence'][name] = flowed['report']
                validate(decode(flowed['trajectory']), model, parameters, policy.jerk, check)
        output[variant] = payload
    return output, False
