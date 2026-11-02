"""Export the exact pinned V5 motion blocks; no motion, no implicit activation."""
import argparse
import copy
import hashlib
import json
from pathlib import Path
import shutil
import sys

from bundle import ROOT, digest, write
from releases import package, planned_action

sys.path.insert(0, str(ROOT / 'scripts'))


def stage_install(output):
    output=Path(output)
    envelope=json.loads((output/'bundle.json').read_text())
    runtime=output/'runtime';runtime.mkdir(exist_ok=True)
    shutil.copy2(Path(__file__).with_name('releases.py'),runtime/'releases.py')
    shutil.copy2(Path(__file__).with_name('install_approved_release.py'),output/'install_approved_release.py')
    if (runtime/'cloud_executor.py').exists() and (output/'remote_before/releases.py').exists():
        files=['bundle.json','manifest.json','runtime/releases.py','runtime/cloud_executor.py','install_approved_release.py']
        write(output/'install_manifest.json',dict(version=envelope['payload']['version'],release_sha256=envelope['sha256'],
            files={name:hashlib.sha256((output/name).read_bytes()).hexdigest() for name in files},
            previous_code={'deployment/rail_jetson/'+name:hashlib.sha256((output/'remote_before'/name).read_bytes()).hexdigest()
                           for name in ('releases.py','cloud_executor.py')}))


def export(urdf, base_path, output):
    from rosidl_runtime_py.convert import message_to_ordereddict
    from task_composition_preview import materialize, concatenate
    from task_offline_optimizer import validate, decode
    from task_optimizer_snapshot import for_case, load
    from task_offline_optimizer_cases import sequence_for
    from task_planning_config import load_parameters
    from task_trajectory import time_seconds
    from task_hand_control import ORDER, OPEN, CLOSED
    from workbench_model import OfflineModel

    model = OfflineModel(urdf)
    parameters, planning = load_parameters()
    # Offline cruise generation is already baked into the arrays. Older Jetson
    # parsers do not know this planning-only switch; runtime gains/limits remain.
    planning = copy.deepcopy(planning)
    planning['values'].pop('rail_replay_cruise', None)
    planning['sha256'] = digest(planning['values'])
    model_sha = hashlib.sha256(Path(urdf).read_bytes()).hexdigest()
    base = json.loads(Path(base_path).read_text())
    initialization = copy.deepcopy(base['payload']['initialization'])
    identity = dict(robot_sha256=model_sha, hand_mapping=dict(order=list(ORDER), open=list(OPEN), closed=list(CLOSED)))
    actions, manifest = {}, {}
    cases = [('dring_pair', 'dring_demo', None, 'LOWER', 'UPPER'),
             ('push', 'rail', 'push_cycle', 'LOWER', 'UPPER'),
             ('pull', 'rail', 'pull_cycle', 'LOWER', 'UPPER'),
             ('gear_roundtrip_lower_upper', 'landing_gear', None, 'LOWER', 'UPPER'),
             ('gear_roundtrip_upper_lower', 'landing_gear', None, 'UPPER', 'LOWER')]
    for name, workflow, slot, start, finish in cases:
        path = for_case(workflow, slot)
        result = load(path, model)['variants']['flow']
        payload = result['payload']
        sequence = sequence_for(payload, count=1, start=start, finish=finish, leave=True)
        trajectory, events = materialize(payload, sequence)
        side = 'left' if workflow == 'dring_demo' else 'right'
        segments, motion = [], []
        peaks = []

        def arm(tr, label, prepare=False):
            row = dict(number=len(segments)+1, kind='arm', side=side, stage='prepare' if prepare else 'optimized',
                label=label, trajectory=message_to_ordereddict(tr), duration=time_seconds(tr.points[-1]),
                start=dict(zip(tr.joint_names, tr.points[0].positions)),
                end=dict(zip(tr.joint_names, tr.points[-1].positions)))
            if not prepare:
                row['recording_fit'] = dict(source_snapshot=result['snapshot_id'])
                peaks.append(validate(tr, model, parameters, 5.))
            segments.append(row)

        def flush():
            if motion:
                arm(concatenate(motion), name + ' continuous motion ' + str(len(segments)))
                motion.clear()

        # Only the pre-task alignment is resolved from live state on Jetson.
        # Everything after it, including the return, is the pinned preview path.
        from task_path_smoothing import joint_line_trajectory
        first = trajectory.points[0]
        if side=='right':
            if any(abs(initialization['target'][n]-q)>1e-8 for n,q in zip(trajectory.joint_names,first.positions)):
                raise ValueError('Right task start differs from shared mirrored initialization: '+name)
        arm(joint_line_trajectory(trajectory.joint_names, first.positions, first.positions),
            'align to pinned task start', prepare=True)

        def hand(posture, hold=None):
            flush()
            target = payload['handPresets'][posture]['feedback_q']
            if hold is None:
                hold = joint_line_trajectory(trajectory.joint_names, first.positions, first.positions)
            segment = dict(number=len(segments)+1, kind='hand', side=side, label=name+' hand '+posture,
                duration=1., target_q=target, trajectory=message_to_ordereddict(hold))
            if workflow == 'landing_gear':
                segment['hand_target_required'] = False
                segment['finger_sequence'] = 'four_then_thumb' if posture == 'grip' else 'thumb_then_four'
            segments.append(segment)

        open_first=workflow=='landing_gear'
        if open_first:
            hand('open')
        for unit_name in sequence:
            wait = payload.get('handWaits', {}).get(unit_name)
            if wait:
                hand(wait['posture'], decode(payload['units'][unit_name]['trajectory']))
            elif unit_name.endswith('_wait'):
                hand(unit_name.split('_')[0])
            else:
                motion.append(decode(payload['units'][unit_name]['trajectory']))
        flush()
        plan = dict(kind='optimized_task', action=name, arm=side, segments=segments,
            hardware_executable=True, physical_verified=False, collision_checked=False,
            demo=dict(arm_only=workflow=='dring_demo'), planning_config=planning,
            speed_scaling=dict(velocity_limit_scaling=.7, acceleration_limit_scaling=.7),
            export_validation=dict(method='pinned-v5-polynomial-validation', physical_verified=False,
                segments_sha256=digest(segments), peaks=peaks, model_sha256=model_sha,
                snapshot_sha256=json.loads(path.read_text())['sha256'],
                sequence=sequence, preview_seconds=time_seconds(trajectory.points[-1]),
                handEvents=events, initialization_hand_seconds=1. if open_first else 0.))
        actual, action = planned_action(plan, dict(identity=identity), str(path.relative_to(ROOT)))
        assert actual == name
        actions[name] = action
        manifest[name] = dict(seconds=time_seconds(trajectory.points[-1]), sequence=sequence,
            snapshot=str(path.relative_to(ROOT)), snapshot_id=result['snapshot_id'],
            plan_sha256=action['source_plan_sha256'], hand_events=events, physical_verified=False)
    envelope = package('approved-v5-' + digest(manifest)[:16], actions, initialization)
    output = Path(output)
    write(output/'bundle.json', envelope)
    write(output/'manifest.json', dict(version=envelope['payload']['version'], sha256=envelope['sha256'],
        actions=manifest, hardware_commands_sent=0, activated=False))
    stage_install(output)
    print(json.dumps(dict(version=envelope['payload']['version'], sha256=envelope['sha256'], actions=manifest), indent=2))
    return envelope


if __name__ == '__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--urdf',type=Path,required=True)
    parser.add_argument('--base',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    export(args.urdf,args.base,args.output)
