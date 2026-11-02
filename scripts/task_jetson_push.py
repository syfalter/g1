"""Publish the current ready plan, without creating robot clients or changing control."""
import copy
from datetime import datetime
import json
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
DEPLOYMENT = ROOT / 'deployment/rail_jetson'
if str(DEPLOYMENT) not in sys.path:
    sys.path.append(str(DEPLOYMENT))

from bundle import digest, read, write
from releases import ReleaseStore, action_name, package, planned_action
from upload_release import upload

DEFAULT_CONFIG = ROOT / 'config/jetson_push.json'


def issue(node, workflow):
    """Cheap UI check; the worker rechecks generation and copies the actual plan."""
    plan = node.session.plan
    if node.session.phase != 'PLAN_READY' or not plan:
        return '请先生成当前模块的可执行规划'
    if workflow == 'rail':
        if plan.get('kind') != 'rail':
            return '当前计划不属于滑轨模块'
    elif plan.get('kind') != 'dring_demo' or plan.get('demo', {}).get('workflow', 'dring_demo') != workflow:
        return '当前计划属于其他模块'
    try:
        action_name(plan)
    except (KeyError, ValueError) as error:
        return str(error)
    if plan.get('hardware_executable') is not True or plan.get('preview_only_timing'):
        return '当前计划仅供预览，不能作为生产动作上传'
    control = getattr(node, 'demo' if workflow == 'dring_demo' else workflow)
    if getattr(control, 'recording', None) is not None:
        return '录制中不能上传'
    proof = plan['rail'] if workflow == 'rail' else plan['demo']
    if proof.get('revision') != control.revision:
        return '录制内容已变化，请重新生成规划'
    return None


def serialize(plan):
    from rosidl_runtime_py.convert import message_to_ordereddict
    result = {k: copy.deepcopy(v) for k, v in plan.items() if k != 'segments'}
    result['segments'] = []
    for segment in plan['segments']:
        result['segments'].append({**{k: copy.deepcopy(v) for k, v in segment.items() if k != 'trajectory'},
            'trajectory': message_to_ordereddict(segment['trajectory'])})
    return result


def snapshot(node, workflow, generation, *, convert=serialize, waist_target=-.015293545089662075):
    with node.lock:
        problem = issue(node, workflow)
        if problem or node.session.generation != generation:
            raise RuntimeError(problem or '规划已变化，请重新确认 Push')
        plan = node.session.plan
        control = getattr(node, 'demo' if workflow == 'dring_demo' else workflow)
        source = control.config
        revision = control.revision
        prepared = copy.deepcopy(node.prepared)
    # Large arrays are serialized outside the LowState feedback lock.
    frozen = convert(plan)
    directory = Path(frozen['directory']).resolve()
    sessions = (ROOT / 'data/task_sessions').resolve()
    if sessions not in directory.parents:
        raise ValueError('计划来源不在当前工作空间会话目录中')
    saved = read(directory / 'plan.json')
    if digest(saved) != digest(frozen):
        raise RuntimeError('当前计划与落盘快照不一致，请重新生成规划')
    config = read(directory / 'config.json')
    from task_screen_teach import fingerprint
    proof = plan['rail'] if workflow == 'rail' else plan['demo']
    if fingerprint(source) != proof['config_sha256']:
        raise RuntimeError('示教参数已变化，请重新生成规划')
    monitor = getattr(node, 'planning_monitor', None)
    if monitor is not None:
        monitor.reload()
        if not monitor.matches(plan['planning_config'], node.gateway):
            raise RuntimeError('规划参数已变化，请重新生成规划')
    with node.lock:
        if (node.session.plan is not plan or node.session.generation != generation
                or node.session.phase != 'PLAN_READY' or control.config is not source
                or control.revision != revision):
            raise RuntimeError('冻结期间规划发生变化；未上传')
    target = {}
    for side in ('left', 'right'):
        for suffix in ('shoulder_pitch', 'shoulder_roll', 'shoulder_yaw', 'elbow',
                       'wrist_roll', 'wrist_pitch', 'wrist_yaw'):
            name = side + '_' + suffix + '_joint'
            target[name] = prepared[side]['joints'][name]
    target['waist_yaw_joint'] = waist_target
    init = dict(mode='simultaneous', velocity_scale=.7, acceleration_scale=.7, target=target)
    name, action = planned_action(frozen, config, str(directory.relative_to(sessions)))
    return name, action, init


def push(node, workflow, generation, *, config_path=DEFAULT_CONFIG, notify=lambda text: None):
    settings = read(config_path)
    notify('冻结当前规划…')
    name, action, init = snapshot(node, workflow, generation, waist_target=settings['waist_target_rad'])
    version = datetime.now().strftime('%Y%m%d-%H%M%S') + '-' + name + '-' + str(time.time_ns())[-6:]
    envelope = package(version, {name: action}, init, kind='patch')
    store = ReleaseStore(ROOT / 'data/action_releases')
    store.install(envelope)
    receipt_path = ROOT / 'data/jetson_push_receipts' / (version + '.json')
    write(receipt_path, dict(version=version, sha256=envelope['sha256'], uploaded=False,
                            action=name, source=action['source'], verification=action['verification']))
    notify('上传 ' + name + '；不会启用或运动…')
    try:
        result = upload(store.path(version), settings['target'], settings['root'])
    except Exception as error:
        write(receipt_path, dict(version=version, sha256=envelope['sha256'], uploaded=False,
            action=name, source=action['source'], message=str(error), local_bundle=str(store.path(version))))
        raise RuntimeError('Push 未确认完成，原计划未修改。可重传同一版本：' + version + '\n' + str(error)) from error
    result.update(action=name, source=action['source'], verification=action['verification'],
                  local_bundle=str(store.path(version)))
    write(receipt_path, result)
    return result
