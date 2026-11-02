"""Right-arm landing-gear recordings on the shared demonstration engine."""
import copy

import task_dring_teach_data as demo
from task_dring_teach_data import moving, joint_array, arrays, preview
from task_hand_control import checked_q, model_joints
from task_path_smoothing import joint_line_trajectory
from task_trajectory import time_seconds

SCHEMA = 'g1_landing_gear_demonstration_v1'
SLOTS = {
    'above_to_lower': '桌上方 → 下杆位',
    'lower_to_upper': '下杆位 → 上杆位',
    'upper_to_lower': '上杆位 → 下杆位',
    'above_to_upper': '桌上方 → 上杆位',
    'lower_to_above': '下杆位 → 桌上方',
    'upper_to_above': '上杆位 → 桌上方',
}
ENDPOINTS = {
    'above_to_lower': ('preparation', 'lower'),
    'lower_to_upper': ('lower', 'upper'),
    'upper_to_lower': ('upper', 'lower'),
    'above_to_upper': ('preparation', 'upper'),
    'lower_to_above': ('lower', 'preparation'),
    'upper_to_above': ('upper', 'preparation'),
}
HAND_POLICY = {
    'above_to_lower': ('open', 'grip'),
    'lower_to_upper': ('grip',),
    'upper_to_lower': ('grip',),
    'above_to_upper': ('open', 'grip'),
    'lower_to_above': ('open',),
    'upper_to_above': ('open',),
}
HAND_POLICY_VERSION = 'landing_gear_hand_sequence_v2_command_feedback'


def hand_segment(start, posture, hands):
    target = checked_q(hands[posture]['feedback_q'])
    end = dict(start, **model_joints('right', target))
    names = list(model_joints('right', target))
    if any(n not in start for n in names):
        raise ValueError('当前右手实测反馈缺失')
    trajectory = joint_line_trajectory(names, [start[n] for n in names], [end[n] for n in names])
    return dict(kind='hand', side='right', label='右手'+('张开' if posture == 'open' else '抓握'),
                posture=posture, target_q=list(target), hand_target_required=False, start=dict(start), end=end,
                trajectory=trajectory, duration=time_seconds(trajectory.points[-1]))


def build(model, config, start, operation, *args, **kwargs):
    if config['side'] != 'right':
        raise ValueError('Landing gear requires the right arm')
    if kwargs.get('full_cycle'):
        raise ValueError('Landing gear replays one explicitly selected segment')
    if operation not in SLOTS:
        raise ValueError('Unknown landing gear operation')
    bundle = copy.deepcopy(config)
    rows, _ = arrays(bundle['traces'][operation], model, 'right')
    bundle['ready'] = dict(joints=dict(zip(moving('right'), rows[0])), hand_q=None)
    policy = HAND_POLICY[operation]
    for posture in policy:
        if posture not in bundle['hands']:
            raise ValueError('请先保存'+('张开' if posture == 'open' else '抓握')+'手型')
    # Set the required hand before any start alignment; contact strokes never open.
    first = hand_segment(start, policy[0], bundle['hands'])
    plan = demo.build(model, bundle, first['end'], operation, *args, slots=SLOTS, **kwargs)
    plan['start'] = dict(start)
    plan['segments'].insert(0, first)
    if len(policy) == 2:
        plan['segments'].append(hand_segment(plan['segments'][-1]['end'], policy[1], bundle['hands']))
    for number, segment in enumerate(plan['segments'], 1):
        segment['number'] = number
    plan['demo'].update(hand_policy=HAND_POLICY_VERSION, arm_only=False)
    plan['speed_scaling']['duration_after_s'] = sum(s['duration'] for s in plan['segments'])
    return plan
