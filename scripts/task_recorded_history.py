"""Load complete recorded plans verbatim. No fitter, retimer or collision solver."""
import copy
import hashlib
import json
from pathlib import Path

from moveit_msgs.msg import DisplayTrajectory, RobotTrajectory
from rosidl_runtime_py.set_message import set_message_fields
from trajectory_msgs.msg import JointTrajectory

from task_motion_planner import robot_state
from task_saved_test import range_fingerprint
from task_screen_teach import fingerprint, write_json
from task_trajectory_speed import rescale


def read_plan(path, workflow, identity):
    path = Path(path)
    if path.is_dir():path = path/'plan.json'
    blob = path.read_bytes()
    plan = json.loads(blob)
    proof_key = 'rail' if workflow == 'rail' else 'demo'
    if plan.get('kind') != ('rail' if workflow == 'rail' else 'dring_demo'):
        raise ValueError('历史计划业务不匹配')
    if proof_key == 'demo' and plan['demo'].get('workflow', 'dring_demo') != workflow:
        raise ValueError('历史计划示教业务不匹配')
    if plan.get('hardware_executable') is not True or plan.get('preview_only_timing'):
        raise ValueError('该历史结果仅用于预览，没有真机执行资格')
    config = json.loads((path.parent/'config.json').read_text())
    proof = plan[proof_key]
    if config.get('identity') != identity or fingerprint(config) != proof.get('config_sha256'):
        raise ValueError('历史模型/手指映射或示教快照不匹配')
    if workflow == 'landing_gear':
        from task_landing_gear_data import HAND_POLICY_VERSION
        legacy_return = proof.get('arm_only') is True and proof.get('operation') in ('ready','preparation')
        if proof.get('hand_policy') not in ('landing_gear_hand_sequence_v1',HAND_POLICY_VERSION) and not legacy_return:
            raise ValueError('历史手型执行策略不兼容')
    segments = plan.get('segments', [])
    if not segments:raise ValueError('历史轨迹为空')
    for segment in segments:
        tr = JointTrajectory(); set_message_fields(tr, segment['trajectory'])
        # Structural/integrity checks only; do not recalculate motion bounds.
        segment['trajectory'] = rescale(tr, 1., require_derivatives=segment.get('kind', 'arm') == 'arm')
    if range_fingerprint(segments) != proof.get('trajectory_sha256'):
        raise ValueError('历史轨迹内容与已保存指纹不匹配')
    return plan, config, dict(path=str(path.resolve()), sha256=hashlib.sha256(blob).hexdigest(),
                             model_identity=identity, workflow=workflow)


def display(plan):
    if plan['kind'] == 'dring_demo':
        from task_dring_teach_data import preview
        tr = preview(plan)
    else:
        from task_rail_trace import preview_trajectory
        tr = preview_trajectory(plan['segments'])
    return DisplayTrajectory(trajectory_start=robot_state(plan['start']),
                             trajectory=[RobotTrajectory(joint_trajectory=tr)])


def load(control, path, *, preview_only=False):
    n = control.node
    with n.lock:
        control.idle()
        revision = control.revision
        n.session.invalidate()
        if n.session.phase=='PLAN_READY':n.session.phase='REPLAN_REQUIRED'
        generation = n.session.generation
        control.editor_hide()
        n.display_pub.publish(DisplayTrajectory())
    def unchanged():
        if (control.revision != revision or n.session.generation != generation or
                n.shutdown_requested or control.recording is not None):
            raise RuntimeError('载入期间会话/STOP/示教数据改变')
    def work():
        ticket = None
        try:
            plan, config, record = read_plan(path, control.workflow, control.identity())
            if preview_only:
                message = display(plan)
                with n.lock:
                    unchanged()
                    n.display_pub.publish(message)
                    n.status = '历史原速 RViz 回放；未重新计算，未发送真机指令'
                return
            with n.lock:
                unchanged(); control.gateway_guard()
                ticket = n.session.begin(plan['kind'], plan['arm'], .7, .7)
                key = 'rail' if plan['kind'] == 'rail' else 'demo'
                proof = plan[key]
                epoch = n.gateway['task_stop_epoch']
                proof.update(revision=revision, stop_epoch=int(epoch) if key == 'rail' else epoch,
                    **{k:n.gateway.get(k) for k in ('waist_control_mode', 'waist_control_revision')})
                record.update(revision=revision, trajectory_sha256=range_fingerprint(plan['segments']))
                plan['recorded_history'] = record
            # The historical start is not silently replaced by current feedback.
            # A mismatch requires explicit positioning or a newly planned join.
            from real_task_app import TaskNode
            TaskNode.wait_stationary(n, ticket, expected=plan['start'],
                guard=lambda:control.execution_guard(plan), context='历史轨迹起点')
            directory = n.run_dir/f'{control.workflow}_history_{ticket.generation:04d}'
            plan['directory'] = str(directory)
            from rosidl_runtime_py.convert import message_to_ordereddict
            write_json(directory/'config.json', config)
            write_json(directory/'plan.json', dict(plan, segments=[dict(s,
                trajectory=message_to_ordereddict(s['trajectory'])) for s in plan['segments']]))
            with n.lock:
                n.session.check(ticket); control.execution_guard(plan, preflight=True)
                n.session.ready(ticket, plan)
                n.display_pub.publish(display(plan))
                n.status = '历史轨迹已直接载入；保持原路径/原时间，核对净空后执行'
                control.status = n.status
        except Exception:
            if ticket is not None:
                with n.lock:n.session.fail(ticket)
            raise
    n.launch_work(work)


def guard(control, plan, *, preflight=False):
    record = plan['recorded_history']
    if record['revision'] != control.revision:
        raise RuntimeError('历史载入后示教配置改变，请重新载入')
    if preflight:
        if not control.node.hand_control and any(s.get('kind')=='hand' or s.get('recorded_hand') is not None
                                                for s in plan['segments']):
            raise RuntimeError('历史手型回放需要启用手部控制')
        if record['model_identity'] != control.identity():
            raise RuntimeError('历史机器人模型不匹配')
        if range_fingerprint(plan['segments']) != record['trajectory_sha256']:
            raise RuntimeError('历史轨迹已改变，请重新载入')
        if hashlib.sha256(Path(record['path']).read_bytes()).hexdigest() != record['sha256']:
            raise RuntimeError('历史来源文件已改变，请重新载入')
