"""Independent landing-gear data, sharing capture, replay and hand controls."""
import copy
from pathlib import Path

from task_dring_demo_control import DringDemoControl
from task_hand_control import require_state, model_joints
from task_rail_control import RailControl
from task_screen_teach import now
import task_landing_gear_data as data

STORE = Path(__file__).resolve().parents[1]/'data/task_panel/landing_gear_demonstration.json'


class LandingGearControl(DringDemoControl):
    data = data
    workflow = 'landing_gear'
    default_side = 'right'
    arm_only = False
    hand_target_required = False
    capture_namespace = 'landing_gear_recordings'
    replay_cache_name = 'landing_gear_replay_cache'
    trim_history_name = 'landing_gear_trim_history'
    hand_motion_labels = {'open':'右手张开', 'grip':'右手抓握'}
    hand_completion_note = '起落架杆状态需重新核对'

    def gateway_guard(self, hold=True):
        super().gateway_guard(hold, stationary_velocity=False)

    def __init__(self, node, store=STORE):
        self.latest_hand = None
        super().__init__(node, store)
        if not self.config['traces']:
            self.status = '尚未录入起落架杆轨迹'

    def empty(self, side='right'):
        if side != 'right':
            raise ValueError('起落架杆仅使用右臂')
        return dict(schema=data.SCHEMA, identity=self.identity(), side='right', ready=None,
                    hands={}, traces={}, saved_at=now())

    def validate(self, config):
        if (config.get('schema') != data.SCHEMA or config.get('identity') != self.identity()
                or config.get('side') != 'right' or config.get('ready') is not None):
            raise ValueError('起落架杆示教模型、业务或执行手臂不匹配')
        for key, allowed in (('hands', ('open', 'grip')), ('traces', data.SLOTS)):
            if not isinstance(config.get(key), dict) or not set(config[key]) <= set(allowed):
                raise ValueError('起落架杆示教槽位无效：'+key)
        for state in config['hands'].values():
            require_state(state, 'right')
        for trace in config['traces'].values():
            data.arrays(trace, self.node.model, 'right')
            if trace.get('hand') is not None:
                raise ValueError('起落架杆使用张开/抓握预设，不接受录制手指时间线')
        from task_trace_versions import validate
        validate(config,lambda trace:data.arrays(trace,self.node.model,'right'))
        return config

    def new(self, side='right'):
        with self.node.lock:
            self.idle()
            config = self.empty(side)
            config['hands'] = copy.deepcopy(self.config['hands'])
            self.persist(config)
            self.path = None
            self.status = '已新建起落架杆示教；手型保留'

    def changed(self):
        self.persist(copy.deepcopy(self.config))

    def persist_hand(self, name, state):
        # RailControl.record commits the independent bundle via changed().
        if name not in ('open', 'grip'):
            raise ValueError('起落架杆仅支持张开和抓握手型')
        require_state(state, 'right')

    def record(self, slot=None, *, waypoint=False, hand=None):
        if slot is not None or waypoint or hand not in (None, 'open', 'grip'):
            raise ValueError('请选择张开或抓握手型')
        if not self.node.hand_control:
            raise RuntimeError('当前面板未启用手指控制')
        return RailControl.record(self, hand=hand)

    def apply_hand(self, name, *, confirmed=False):
        if name not in ('open', 'grip'):
            raise ValueError('请选择张开或抓握手型')
        if not self.node.hand_control:
            raise RuntimeError('当前面板未启用手指控制')
        return RailControl.apply_hand(self, name, confirmed=confirmed)

    def recording_start(self, slot):
        return None

    def preparation_required(self, operation, full_cycle):
        return False

    def prepare(self, operation, *args, **kwargs):
        if operation not in data.SLOTS or kwargs.get('full_cycle'):
            raise ValueError('请选择六段已录制轨迹之一；返回准备位也使用录制段')
        if not self.node.hand_control:
            raise RuntimeError('自动手型回放需要启用手指控制')
        return super().prepare(operation, *args, **kwargs)

    def planning_start(self, ticket, guard):
        n = self.node
        before = super().planning_start(ticket, guard)
        hands = n.read_hands(ticket)
        with n.lock:
            n.session.check(ticket); guard(); self.fresh_hands(hands)
            q = dict(n.session.measured(stationary=True))
            if any(abs(q[k]-before[k]) > .01 for k in data.moving('right')):
                raise RuntimeError('读取手型期间手臂移动，请停稳后重新生成')
            for side, state in hands.items():
                q.update(model_joints(side, require_state(state, side)['feedback_q']))
            return q

    def execution_guard(self, plan, running=False, preflight=False):
        if plan.get('recorded_history'):
            return super().execution_guard(plan,running=running,preflight=preflight)
        proof = plan['demo']
        if proof.get('hand_policy') != data.HAND_POLICY_VERSION or proof.get('arm_only') is not False:
            raise RuntimeError('旧起落架杆计划缺少自动手型流程，请重新生成')
        return super().execution_guard(plan, running=running, preflight=preflight)

    def start_trace(self, slot, with_hand=False):
        if with_hand:
            raise ValueError('手型单独保存，不随手臂轨迹录制')
        return super().start_trace(slot, False)

    def delete_trace(self, slot):
        with self.node.lock:
            self.idle()
            config = copy.deepcopy(self.config)
            config['traces'].pop(slot, None)
            config.get('trace_versions',{}).pop(slot,None)
            self.persist(config)
            self.status = '已删除选中段；原始采集文件保留'

    def completed(self, plan):
        operation = plan['demo']['operation']
        if plan.get('recorded_history'):
            self.status = '历史轨迹回放完成，保持历史终点：'+data.SLOTS.get(operation,operation)
            self.node.status = self.status
            return
        posture = data.HAND_POLICY[operation][-1]
        self.status = data.SLOTS[operation]+'回放完成；'+('保持张开' if posture == 'open' else '保持抓握')
        self.node.status = self.status
