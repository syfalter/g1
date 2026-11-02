"""Rail teaching and authorization in the existing hardware owner."""
import copy
import hashlib
import json
import math
import threading
import time
from pathlib import Path

import numpy as np
from moveit_msgs.msg import DisplayTrajectory, RobotTrajectory
from rosidl_runtime_py.convert import message_to_ordereddict

from task_hand_control import checked_q, model_joints, require_state, ORDER, OPEN, CLOSED
from task_motion_planner import robot_state
from task_planning_config import DEFAULT_CONFIG, load_parameters, controller_bounded_parameters, rail_panel_parameters, parameters_from_values
from task_rail_motion import LINK, STATES, required_state, build, VELOCITY_SCALE, ACCELERATION_SCALE
from task_saved_test import range_fingerprint
from task_screen_control import ScreenControl
from task_dring_demo_control import DringDemoControl
from task_screen_teach import MOVING, guarded_joints, fingerprint, now, write_json
from task_session import ARMS
from task_rail_capture import DirectCapture
import task_rail_trace as traces
from task_trace_editor_control import TraceEditorControl

SCHEMA = 'g1_rail_teach_v1'


class RailControl(TraceEditorControl):
    workflow = 'rail'

    def persist_versions(self, config):
        self.validate(config)
        write_json(self.node.run_dir/'rail_autosave/config.json',config)
        self.config=config
        self.remember_selection_source(self.node.run_dir/'rail_autosave/config.json')
        self.invalidate()
        self.trace_review=None

    hand_motion_labels = {'open':'右手张开', 'grip':'右手推进', 'pull_grip':'右手拉出'}
    hand_completion_note = '滑杆状态需重新确认'
    def gateway_guard(self, hold=True):
        ScreenControl.gateway_guard(self,hold,stationary_velocity=False)

    def arm_gateway_guard(self, hold=True):
        DringDemoControl.gateway_guard(self,hold,stationary_velocity=False)
    fresh_hands = ScreenControl.fresh_hands
    read_stable_hands = ScreenControl.read_stable_hands

    def __init__(self, node):
        self.node = node
        self.config = self.empty()
        self.revision = 0
        self.path = None
        self.state = 'unknown'
        self.state_epoch = None
        self.latest_hand = None
        self.recording = None
        self.trace_status = '尚未录制轨迹'
        self.trace_review = None
        self.cache_dir = Path(__file__).resolve().parents[1]/'data/task_panel/rail_replay_cache'

    def restore_latest(self):
        """Startup-only data restore; never restore authorization or send commands."""
        path = None
        try:
            candidates = list(self.node.run_dir.parent.glob('*/rail_autosave/config.json'))
            if not candidates:
                return
            path = max(candidates, key=lambda p: (p.stat().st_mtime_ns, str(p)))
            if path.stat().st_size > 800_000_000:
                raise ValueError('Rail file too large')
            from task_trace_selection import restore
            config = self.validate(restore(json.loads(path.read_text()),path))
        except (OSError, ValueError, KeyError, TypeError, IndexError) as error:
            self.trace_status = f'自动加载滑轨示教失败：{path or self.node.run_dir.parent}；{error}；未加载更旧记录'
            self.node.status = self.trace_status
            return
        self.config = config
        self.remember_selection_source(path)
        self.path = str(path)
        self.state = 'unknown'
        self.state_epoch = None
        self.trace_status = f'已自动加载上次滑轨示教（{len(config.get("traces", {}))} 段）；可直接规划，已有有效缓存可复用执行'
        self.node.status = self.trace_status

    def idle(self):
        if self.recording is not None:
            raise RuntimeError('请先结束保存或取消当前轨迹录制')
        ScreenControl.idle(self)

    def identity(self):
        return dict(urdf_sha256=hashlib.sha256(Path(self.node.args.robot_urdf).read_bytes()).hexdigest(),
            hand_mapping_sha256=hashlib.sha256((Path(__file__).parent/'task_hand_control.py').read_bytes()).hexdigest(),
            reference_link=LINK)

    def empty(self):
        identity = self.identity()
        hands = {}
        preset_path = Path(__file__).resolve().parents[1]/'config/rail_hand_presets.json'
        if preset_path.is_file():
            preset = json.loads(preset_path.read_text())
            saved_identity = preset.get('identity', {})
            mapping = dict(order=list(ORDER), open_model_q=list(OPEN), closed_model_q=list(CLOSED))
            # Fixed finger targets do not depend on unrelated named task postures.
            # Full recorded trajectory bundles retain their stricter identity check.
            compatible = (saved_identity == identity if 'mapping' not in preset else
                preset['mapping'] == mapping and all(saved_identity.get(k) == identity[k]
                    for k in ('urdf_sha256', 'reference_link')))
            if preset.get('schema') == 'g1_rail_hand_presets_v1' and compatible:
                hands = {key:require_state(preset['hands'][key], 'right')
                         for key in ('open', 'grip', 'pull_grip') if key in preset.get('hands', {})}
        return dict(schema=SCHEMA, identity=identity, points={}, routes={'A':[], 'B':[]},
                    hands=hands, traces={}, created_at=now())

    def persist_hand(self, name, state):
        saved = self.empty()['hands']
        saved[name] = copy.deepcopy(require_state(state, 'right'))
        write_json(Path(__file__).resolve().parents[1]/'config/rail_hand_presets.json',
            dict(schema='g1_rail_hand_presets_v1', identity=self.identity(),
                 mapping=dict(order=list(ORDER), open_model_q=list(OPEN), closed_model_q=list(CLOSED)),
                 hands=saved, saved_at=now(), source=str(self.node.run_dir)))

    def invalidate(self):
        self.revision += 1
        self.state = 'unknown'
        self.state_epoch = None
        self.node.session.invalidate()
        if self.node.session.phase == 'PLAN_READY':
            self.node.session.phase = 'REPLAN_REQUIRED'
        self.node.display_pub.publish(DisplayTrajectory())

    def changed(self):
        self.invalidate()
        self.trace_review = None
        write_json(self.node.run_dir/'rail_autosave/config.json', self.config)
        self.remember_selection_source(self.node.run_dir/'rail_autosave/config.json')

    def ready(self, *, teaching_waist=None):
        record = self.node.prepared.get('right')
        if not record:
            raise RuntimeError('No existing right ready pose')
        q = dict(record['joints'])
        # Older ready poses store only the seven arm axes. Keep the taught waist.
        if 'waist_yaw_joint' not in q:
            if self.config.get('trace_ready'):
                q['waist_yaw_joint'] = self.config['trace_ready']['waist_yaw_joint']
            elif 'A' in self.config['points']:
                q['waist_yaw_joint'] = self.config['points']['A']['joints']['waist_yaw_joint']
            elif teaching_waist is not None:
                q['waist_yaw_joint'] = teaching_waist
            else:
                raise RuntimeError('Teach A first to fix the waist for the existing right ready pose')
        guarded_joints(self.node.model, [q[n] for n in MOVING], context='Rail ready')
        return {n:q[n] for n in MOVING}

    def validate(self, config):
        if config.get('schema') != SCHEMA or config.get('identity') != self.identity():
            raise ValueError('Rail model/hand mapping identity mismatch')
        if set(config['routes']) != {'A', 'B'} or not set(config['points']) <= {'A', 'B'}:
            raise ValueError('Invalid rail point/route keys')
        if not set(config['hands']) <= {'open', 'grip', 'pull_grip'}:
            raise ValueError('Invalid rail hand slots')
        for state in config['hands'].values():
            require_state(state, 'right')
        records = list(config['points'].values())
        for route in config['routes'].values():
            if not isinstance(route, list) or len(route) > 40:
                raise ValueError('Rail routes support at most 40 waypoints')
            records += route
        for record in records:
            q = record['joints']
            if not all(type(v) in (int, float) and math.isfinite(v) for v in q.values()):
                raise ValueError('Nonfinite rail joint')
            guarded_joints(self.node.model, [q[n] for n in MOVING], context='Rail record')
            require_state(record['hand'], 'right')
            pose = np.asarray(record['pelvis_from_wrist'], dtype=float)
            if pose.shape != (4, 4) or not np.allclose(self.node.model.transform(LINK, q), pose, atol=1e-8):
                raise ValueError('Rail record FK does not match its joints')
        if not isinstance(config.get('traces', {}), dict) or not set(config.get('traces', {})) <= set(traces.STAGES):
            raise ValueError('Invalid recorded rail stages')
        for stage, trace in config.get('traces', {}).items():
            # A changed hand marks a trace unusable, but must not prevent exporting it.
            if trace.get('schema') != traces.TRACE_SCHEMA or trace.get('stage') != stage:
                raise ValueError('Invalid recorded rail trace schema')
            traces.arrays(trace['raw'], self.node.model)
        if config.get('trace_ready'):
            guarded_joints(self.node.model,[config['trace_ready'][n] for n in MOVING],context='Trace ready')
        from task_trace_versions import validate
        validate(config,lambda trace:traces.arrays(trace['raw'],self.node.model))
        return config

    def new(self):
        with self.node.lock:
            self.idle()
            hands = copy.deepcopy(self.config['hands'])
            self.config = self.empty()
            self.config['hands'].update(hands)
            self.path = None; self.latest_hand = None
            self.changed()
            self.trace_status = '尚未录制轨迹'

    def save(self, path):
        with self.node.lock:
            self.idle(); self.validate(self.config)
            write_json(path, self.config)
            self.path = str(path)
            if hasattr(self,'selection_source') and Path(path).resolve()==self.selection_source[0].resolve():
                self.remember_selection_source(path)

    def load(self, path):
        path = Path(path)
        if path.stat().st_size > 800_000_000:
            raise ValueError('Rail file too large')
        from task_trace_selection import restore
        config = self.validate(restore(json.loads(path.read_text()),path))
        with self.node.lock:
            self.idle(); self.config = config; self.path = str(path); self.latest_hand = None
            self.changed()
            self.trace_status = '示教文件已加载；轨迹执行前需重新检查和规划'
            self.node.status = '滑轨示教已加载；滑杆位置待人工确认，未恢复执行授权'

    def capture_feedback(self, q, received, stamp):
        rec = self.recording
        if rec is None or rec.get('source') == 'direct_lowstate' or rec['phase'] != 'recording':
            return
        if any(abs(q[n]-rec['initial'][n]) > .025 for n in ARMS['left']):
            rec['buffer'].error = '录制期间左臂移动；只允许示教右臂'
        rec['buffer'].feed(q, received, stamp)

    def reject_feedback(self, reason):
        if (self.recording is not None and self.recording.get('source') != 'direct_lowstate'
                and self.recording['phase'] == 'recording'):
            self.recording['buffer'].error = '录制反馈被拒绝：'+str(reason)

    def cancel_trace(self, reason='已取消录制，原轨迹和手型保留'):
        with self.node.lock:
            if self.recording is not None:
                self.recording['error'] = reason
                self.recording['cancel'].set()

    def finish_trace(self):
        with self.node.lock:
            if self.recording is None or self.recording['phase'] != 'recording':
                raise RuntimeError('当前没有正在录制的轨迹段')
            if self.recording.get('source') != 'direct_lowstate':
                self.node.session.measured(stationary=True)
            else:
                self.recording['receiver'].request_finish(time.monotonic())
            self.recording['finish'].set()
            self.recording['phase'] = 'saving'
            self.trace_status = '录制已截止，正在保存'
            self.node.status = self.trace_status

    def start_trace(self, stage, *, confirmed=False):
        if stage not in traces.STAGES or confirmed is not True:
            raise RuntimeError('选择轨迹段并确认外部控制已停止、机器人静止')
        n = self.node
        with n.lock:
            self.idle(); self.arm_gateway_guard(False)
            if n.gateway.get('phase') != 'TASK_ADMITTANCE':
                raise RuntimeError('轨迹录制需要 TASK_ADMITTANCE；不会自动进入导纳')
            initial = n.session.measured(stationary=True)
            if self.config['identity'] != self.identity():
                raise RuntimeError('滑轨模型已变化，请重新加载匹配的示教')
            self.invalidate()
            rec = dict(stage=stage,phase='starting',generation=n.session.generation,
                source='direct_lowstate',receiver=None,
                epoch=n.gateway['task_stop_epoch'],initial=initial,
                waist_mode=tuple(n.gateway[k] for k in ('waist_control_mode','waist_control_revision')),
                waist_admittance=n.gateway.get('task_waist_admittance_active'),
                finish=threading.Event(),cancel=threading.Event(),error=None,
                buffer=traces.TraceBuffer(n.model))
            self.recording = rec
            n.session.phase = 'RAIL_RECORDING'
            self.trace_status = '等待独立 LowState 直采就绪；保持静止'
            n.status = self.trace_status

        def check():
            n.check()
            with n.lock:
                if rec['cancel'].is_set():
                    raise RuntimeError(rec['error'])
                if (n.session.generation != rec['generation'] or n.gateway['task_stop_epoch'] != rec['epoch'] or
                        tuple(n.gateway[k] for k in ('waist_control_mode','waist_control_revision')) != rec['waist_mode'] or
                        n.gateway.get('task_waist_admittance_active') != rec['waist_admittance'] or
                        n.gateway.get('phase') != 'TASK_ADMITTANCE'):
                    raise RuntimeError('STOP/导纳模式/会话已改变；录制未保存')
                self.arm_gateway_guard(False)
                if rec['receiver'] is not None:
                    rec['receiver'].check()
                else:
                    n.session.measured()
                if rec['buffer'].error:
                    raise RuntimeError(rec['buffer'].error)

        def work():
            try:
                check()
                rec['receiver'] = DirectCapture(n,stage,initial,rec['generation'],rec['epoch'],full_rate=True)
                while not rec['receiver'].check().get('ready'):
                    check()
                    time.sleep(.02)
                with n.lock:
                    check()
                    rec['phase'] = 'recording'
                    self.trace_status = 'LowState 直采录制中：'+traces.STAGES[stage][0]
                    n.status = self.trace_status
                while not rec['finish'].is_set():
                    check()
                    time.sleep(.01)
                check()
                raw = rec['receiver'].finish(check)
                with n.lock:
                    check()
                    rec['phase'] = 'finalizing'
                    self.trace_status = '检查原始反馈和 2 mm 取点；尚未覆盖旧轨迹'
                    n.status = self.trace_status
                trace = dict(schema=traces.TRACE_SCHEMA,stage=stage,hand_policy='not_recorded',raw=raw,
                    capture_source='direct_lowstate',capture_directory=str(rec['receiver'].directory),
                    capture_diagnostics=copy.deepcopy(rec['receiver'].latest),
                    captured_at=now(),stop_epoch=int(rec['epoch']),
                    context_joints={k:float(v) for k,v in initial.items()})
                traces.validate_trace(trace,stage,n.model,self.config['hands'])
                _,_,_,trace['summary'] = traces.analyze(n.model,trace,check)
                candidate = copy.deepcopy(self.config)
                candidate.setdefault('traces',{})[stage] = trace
                candidate.get('trace_versions',{}).pop(stage,None)
                check()
                pending = n.run_dir/'rail_autosave/pending.json'
                write_json(pending,candidate)
                with n.lock:
                    check()
                    pending.replace(n.run_dir/'rail_autosave/config.json')
                    self.config = candidate
                    self.remember_selection_source(n.run_dir/'rail_autosave/config.json')
                    self.invalidate()
                    n.session.phase = 'TASK_ADMITTANCE'
                    self.trace_review = None
                    self.trace_status = f'{traces.STAGES[stage][0]} 已保存：原始 {len(raw["positions"])} 点，取点 {trace["summary"]["replay_count"]} 点'
                    n.status = self.trace_status
            except Exception as error:
                if rec['receiver'] is not None:
                    try:
                        write_json(rec['receiver'].directory/'failure.json',dict(stage=stage,
                            message=str(error),captured_at=now(),
                            gateway=dict(n.gateway),receiver=rec['receiver'].latest,hardware_executed=False))
                    except Exception as archive_error:
                        n.get_logger().error('保存录制失败说明失败：'+str(archive_error))
                with n.lock:
                    self.trace_status = str(error)+('；原始数据保留于 '+str(rec['receiver'].directory)
                        if rec['receiver'] is not None else '')
                    if n.session.generation == rec['generation'] and n.session.phase == 'RAIL_RECORDING':
                        n.session.phase = n.gateway.get('phase','REPLAN_REQUIRED')
                raise
            finally:
                if rec['receiver'] is not None:
                    rec['receiver'].close()
                with n.lock:
                    if self.recording is rec:
                        self.recording = None
        try:
            n.launch_work(work)
        except Exception:
            with n.lock:
                if self.recording is rec:
                    self.recording = None
                if n.session.phase == 'RAIL_RECORDING':
                    n.session.phase = n.gateway.get('phase','REPLAN_REQUIRED')
            raise

    def delete_trace(self, stage):
        with self.node.lock:
            self.idle()
            self.config.get('traces',{}).pop(stage,None)
            self.config.get('trace_versions',{}).pop(stage,None)
            self.trace_review = None
            self.changed()

    def trace_alignment(self, stage):
        n = self.node
        try:
            q = n.session.measured()
            key = 'connect:'+stage
            target = self.movement_target(key,waist=q['waist_yaw_joint'])
            label = self.movement_label(key)
            a = n.model.transform(LINK,q)
            b = n.model.transform(LINK,dict(q,**target))
            delta = (a[:3,3]-b[:3,3])*1000
            joint_deg = math.degrees(max(abs(q[k]-target[k]) for k in MOVING))
            return f'相对{label}：XYZ '+', '.join(f'{v:+.2f}' for v in delta)+f' mm；最大关节差 {joint_deg:.2f}°'
        except (RuntimeError,KeyError,ValueError):
            return '对齐参考：反馈过期或参考未就绪'

    def movement_label(self, key):
        if key == 'preparation':return '右手 above-table'
        if key in traces.ENDPOINTS:return traces.ENDPOINTS[key][0]
        kind, _, stage = key.partition(':')
        if stage not in traces.STAGES or kind not in ('start','connect'):
            raise ValueError('未知滑轨移动目标')
        if kind == 'start':return traces.STAGES[stage][0]+' 已录起点'
        return self.movement_label(traces.PRECEDING_ENDPOINT.get(stage,'preparation'))

    def movement_target(self, key, *, waist):
        self.movement_label(key)
        if key == 'preparation':
            saved = self.node.prepared.get('right')
            if not saved:raise RuntimeError('尚未记录右手 above-table 准备位')
            target = {n:saved['joints'][n] for n in ARMS['right']}
            target['waist_yaw_joint'] = saved['joints'].get('waist_yaw_joint',waist)
        elif key in traces.ENDPOINTS:
            target = traces.endpoint(self.config,key)
        else:
            kind, stage = key.split(':')
            if kind == 'start':target = traces.recorded_position(self.config,stage,0)
            else:return self.movement_target(traces.PRECEDING_ENDPOINT.get(stage,'preparation'),waist=waist)
        guarded_joints(self.node.model,[target[n] for n in MOVING],context='Rail move target')
        return target

    def prepare_move(self, key, *, confirmed=False):
        if confirmed is not True:raise RuntimeError('请确认从当前位置到目标的完整路径净空')
        n = self.node
        with n.lock:
            self.idle(); self.arm_gateway_guard()
            initial = n.session.measured(stationary=True)
            target = self.movement_target(key,waist=initial['waist_yaw_joint'])
            label = self.movement_label(key)
            self.invalidate()
            ticket = n.session.begin('rail','right',VELOCITY_SCALE,ACCELERATION_SCALE)
            revision, epoch = self.revision, int(n.gateway['task_stop_epoch'])
            waist = {k:n.gateway[k] for k in ('waist_control_mode','waist_control_revision')}
            n.status = '规划移动到'+label+'；未发送运动'
        directory = n.run_dir/f'rail_move_{ticket.generation:04d}'
        def work():
            from real_task_app import TaskNode
            try:
                params, record = load_parameters(getattr(n.args,'planning_config',DEFAULT_CONFIG))
                q = TaskNode.wait_stationary(n,ticket,expected=initial,guard=self.arm_gateway_guard,
                                             context='滑轨单独移动')
                with n.lock:
                    n.session.check(ticket)
                    params, record = controller_bounded_parameters(params,record,n.gateway)
                segments = traces.align(n.model,q,target,params,lambda:n.check(ticket),'移动到'+label)
                if not segments:raise RuntimeError('当前已在'+label+'，未发送运动')
                for i,segment in enumerate(segments,1):segment['number']=i
                plan = dict(kind='rail',arm='right',start=q,segments=segments,waist_assist=True,
                    hardware_executable=True,collision_checked=False,directory=str(directory),planning_config=record,
                    rail=dict(operation='move',mode='move',path_source='move',target=key,target_joints=target,
                        label=label,arm_only=True,final_state='unknown',revision=revision,stop_epoch=epoch,
                        trajectory_sha256=range_fingerprint(segments),**waist))
                write_json(directory/'plan.json',dict(plan,segments=[dict(s,
                    trajectory=message_to_ordereddict(s['trajectory'])) for s in segments]))
                display = traces.preview_trajectory(segments,lambda:n.check(ticket))
                self.execution_guard(plan,preflight=True)
                TaskNode.wait_stationary(n,ticket,expected=q,guard=lambda:self.execution_guard(plan),
                                         context='滑轨单独移动就绪')
                with n.lock:
                    n.session.check(ticket); self.execution_guard(plan)
                    n.session.ready(ticket,plan)
                    n.display_pub.publish(DisplayTrajectory(trajectory_start=robot_state(q),trajectory=[
                        RobotTrajectory(joint_trajectory=display)]))
                    n.status = '移动计划就绪：'+label
            except Exception as error:
                with n.lock:n.session.fail(ticket)
                write_json(directory/'failure.json',dict(message=str(error),hardware_executed=False))
                raise
        n.launch_work(work)

    def inspect_trace(self, stage):
        n = self.node
        with n.lock:
            self.idle(); self.validate(self.config)
            trace = copy.deepcopy(self.config.get('traces',{}).get(stage))
            if trace is None:
                raise RuntimeError('该轨迹段尚未录制')
            traces.validate_trace(trace,stage,n.model,self.config['hands'])
            self.invalidate(); generation = n.session.generation
            hands = copy.deepcopy(self.config['hands'])
        def check():
            n.check()
            with n.lock:
                if n.session.generation != generation:
                    raise RuntimeError('轨迹检查被取消')
        def work():
            params,_ = load_parameters(getattr(n.args,'planning_config',DEFAULT_CONFIG))
            q = dict(n.session.current,**dict(zip(MOVING,trace['raw']['positions'][0])))
            posture = hands.get(traces.STAGES[stage][1])
            if posture:q.update(model_joints('right',posture['feedback_q']))
            tr,report = traces.replay(n.model,trace,q,params,check)
            with n.lock:
                check(); self.trace_review = dict(stage=stage,**report)
                n.display_pub.publish(DisplayTrajectory(trajectory_start=robot_state(q),
                    trajectory=[RobotTrajectory(joint_trajectory=tr)]))
                self.trace_status = f'该段检查通过：{report["replay_seconds"]:.1f} s，模型拟合偏差 {report["max_fit_mm"]:.3f} mm；仅预览，不能执行'
                n.status = self.trace_status
        n.launch_work(work)

    def preview_cycle(self, mode):
        if mode not in ('hold', 'release', 'cycle'):
            raise ValueError('请选择推入后的操作模式')
        n = self.node
        with n.lock:
            self.idle(); self.validate(self.config)
            config = copy.deepcopy(self.config)
            if not config.get('traces',{}).get('approach_A'):
                raise RuntimeError('请先录制接近外端轨迹')
            ready = self.ready(teaching_waist=n.session.current['waist_yaw_joint'])
            q = dict(config['traces']['approach_A']['context_joints'],**ready)
            q.update(model_joints('right',config['hands']['open']['feedback_q']))
            self.invalidate(); generation = n.session.generation
            n.status = '正在生成完整推拉动画；仅预览，不读取手部或发送动作'
        def check():
            n.check()
            with n.lock:
                if n.session.generation != generation:
                    raise RuntimeError('完整流程预览已取消')
        def work():
            params,_ = load_parameters(getattr(n.args,'planning_config',DEFAULT_CONFIG))
            push = traces.build_replay(n.model,config,ready,q,'push',mode,params,check)
            pull = traces.build_replay(n.model,config,ready,push['segments'][-1]['end'],
                                       'pull',mode,params,check)
            segments = push['segments']+pull['segments']
            animation = traces.preview_trajectory(segments,check)
            with n.lock:
                check()
                self.trace_review = dict(mode=mode,preview_only=True,
                    segments=[dict(label=s['label'],duration=s['duration']) for s in segments])
                n.display_pub.publish(DisplayTrajectory(trajectory_start=robot_state(q),
                    trajectory=[RobotTrajectory(joint_trajectory=animation)]))
                self.trace_status = f'完整推入＋拉出预览已发布到 RViz，共 {len(segments)} 段；含腰和手型。仅动画，不可执行'
                n.status = self.trace_status
        n.launch_work(work)

    def remove(self, slot, index=None):
        with self.node.lock:
            self.idle()
            if slot not in ('A', 'B'):
                raise ValueError('Invalid rail slot')
            if index is None:
                self.config['points'].pop(slot, None)
            else:
                del self.config['routes'][slot][index]
            self.changed()

    def apply_hand(self, name, *, confirmed=False):
        """Explicit right-hand-only motion, including stationary admittance teaching."""
        if name not in ('open', 'grip', 'pull_grip') or not confirmed:
            raise RuntimeError('请确认手部动作和外部 Web 已停止控制')
        n = self.node
        with n.lock:
            self.idle(); self.gateway_guard(False)
            if self.config['identity'] != self.identity():
                raise RuntimeError('滑轨模型或手部映射已变化，请重新加载匹配的示教')
            phase = n.gateway.get('phase')
            if phase not in ('TASK_ADMITTANCE', 'ARMED_HOLD'):
                raise RuntimeError('手型切换需要静止导纳或静止保持状态')
            if phase == 'ARMED_HOLD':
                self.gateway_guard()
            if name not in self.config['hands']:
                raise RuntimeError('尚未记录此手型，请先加载手型或读取记录')
            target = checked_q(self.config['hands'][name]['feedback_q'])
            before = n.session.measured(stationary=True)
            self.invalidate()
            generation, epoch = n.session.generation, n.gateway['task_stop_epoch']
        def check():
            n.check()
            with n.lock:
                self.gateway_guard(False)
                if (generation != n.session.generation or epoch != n.gateway['task_stop_epoch'] or
                        n.gateway.get('phase') != phase):
                    raise RuntimeError('STOP、模式或会话变化，手型切换已取消')
                q = n.session.measured()
                if any(abs(q[k]-before[k]) > .025 for k in MOVING+ARMS['left']):
                    raise RuntimeError('切换手型期间手臂移动，请停稳后重试')
        def feedback(state):
            n.cache_hand_state('right', state)
            with n.lock:
                self.latest_hand = copy.deepcopy(state)
        def work():
            n.hand.begin_motion(check)
            require_target = getattr(self, 'hand_target_required', True)
            if require_target:
                readings = n.hand.transition('right', target, check, duration=1., on_feedback=feedback)
            else:
                from task_hand_completion import transition
                readings = transition(n.hand, 'right', target, check, duration=1., on_feedback=feedback,
                                      require_target=False)
            check()
            if require_target and max(abs(a-b) for a,b in zip(readings[-1]['feedback_q'], target)) > .02:
                raise RuntimeError('手型偏差超过录制门槛；未授权录制，请检查实际接触状态')
            with n.lock:
                result = '手型已到位' if require_target else '指令已确认，未校验手型到位'
                n.status = self.hand_motion_labels[name]+result+'；未移动手臂，'+self.hand_completion_note
        n.status = '正在切换右手手型，请保持手臂静止'
        n.launch_work(work)

    def record(self, slot=None, *, waypoint=False, hand=None):
        """A None slot/hand means read-only hand feedback, never an actuator command."""
        n = self.node
        if slot not in (None, 'A', 'B') or hand not in (None, 'open', 'grip', 'pull_grip'):
            raise ValueError('Invalid rail teaching slot')
        with n.lock:
            self.idle(); self.gateway_guard(False)
            if self.config['identity'] != self.identity():
                raise RuntimeError('Rail model changed; create a new workcell')
            if (slot or hand) and n.gateway.get('phase') != 'TASK_ADMITTANCE':
                raise RuntimeError('Record rail points/hand postures only in TASK_ADMITTANCE')
            n.session.measured(stationary=True)
            generation, epoch = n.session.generation, n.gateway['task_stop_epoch']
            before = n.session.measured(stationary=True)
        def work():
            def unchanged():
                n.check()
                with n.lock:
                    if generation != n.session.generation or epoch != n.gateway['task_stop_epoch']:
                        raise RuntimeError('STOP/session changed while reading rail feedback')
            hands = self.read_stable_hands(check=unchanged)
            with n.lock:
                unchanged(); self.gateway_guard(False); self.fresh_hands(hands)
                if (slot or hand) and n.gateway.get('phase') != 'TASK_ADMITTANCE':
                    raise RuntimeError('Rail teaching mode changed')
                q = n.session.measured(stationary=True)
                if any(abs(q[k]-before[k]) > .01 for k in MOVING):
                    raise RuntimeError('Robot moved during rail recording')
                self.latest_hand = copy.deepcopy(hands['right'])
                if slot:
                    for side in ARMS:
                        q.update(model_joints(side, hands[side]['feedback_q']))
                    guarded_joints(n.model, [q[k] for k in MOVING], context='Rail teach')
                    record = dict(joints=q, hand=copy.deepcopy(hands['right']),
                        pelvis_from_wrist=n.model.transform(LINK, q).tolist(), captured_at=now(),
                        source_stamp_ns=n.session.source_stamp_ns, stop_epoch=int(epoch))
                    if waypoint:
                        if len(self.config['routes'][slot]) >= 40:
                            raise ValueError('Maximum 40 waypoints per route')
                        self.config['routes'][slot].append(record)
                    else:
                        self.config['points'][slot] = record
                if hand:
                    self.persist_hand(hand, hands['right'])
                    self.config['hands'][hand] = copy.deepcopy(hands['right'])
                if slot or hand:
                    self.changed()
                n.status = ('手型已覆盖保存为面板默认值，下次启动自动载入；未发送动作' if hand else
                            '右手反馈已读取'+('并保存示教' if slot else '')+'；未发送手型或运动命令')
        n.launch_work(work)

    def confirm_state(self, state):
        if state not in STATES:
            raise ValueError('Invalid rail state')
        with self.node.lock:
            self.idle(); self.gateway_guard()
            self.node.session.measured(stationary=True)
            self.invalidate()
            self.state = state
            self.state_epoch = int(self.node.gateway['task_stop_epoch'])

    def start_guard(self, operation, mode, path_source='endpoints'):
        self.gateway_guard()
        if path_source != 'recorded' and (self.state != required_state(operation, mode) or
                self.state_epoch != int(self.node.gateway['task_stop_epoch'])):
            raise RuntimeError('Confirm the actual rail position and grasp state first')
        q = self.node.session.measured(stationary=True)
        if path_source == 'recorded':
            stage = traces.required_stages(operation,mode)[0]
            trace = self.config.get('traces',{}).get(stage)
            if trace is None:
                raise RuntimeError('请先录制：'+traces.STAGES[stage][0])
        else:
            target = self.config['points']['B']['joints'] if operation == 'pull' and mode == 'hold' else self.ready()
        if path_source != 'recorded' and any(abs(q[n]-target[n]) > .025 for n in MOVING):
            raise RuntimeError('Robot is not at the required ready/B start; no automatic repositioning')
        return q

    def plan(self, operation, mode, *, confirmed=False, path_source='endpoints', reuse=False,
             replay_multiplier=None, replay_fastest=None, trim_idle_ends=None):
        if path_source not in ('endpoints','recorded'):
            raise ValueError('Invalid rail path source')
        if (mode == 'cycle' or reuse) and path_source != 'recorded':
            raise ValueError('完整三段循环及缓存复用仅支持录制轨迹')
        if path_source != 'recorded' and confirmed is not True:
            raise RuntimeError('Confirm web command output is stopped and the workcell is fixed')
        overrides = {key:value for key,value in (('rail_replay_multiplier',replay_multiplier),
            ('rail_replay_fastest',replay_fastest),('rail_trim_idle_ends',trim_idle_ends)) if value is not None}
        if path_source=='recorded':overrides['rail_replay_cruise']=getattr(self,'generation_mode','original')=='cruise'
        if overrides:
            if path_source != 'recorded':raise ValueError('回放处理选项仅用于录制轨迹')
            parameters_from_values(overrides)
        n = self.node
        with n.lock:
            self.idle(); self.start_guard(operation, mode, path_source)
            if any(k not in self.config['hands'] for k in ('open', 'grip')):
                raise RuntimeError('Record open and grip hand feedback first')
            ticket = n.session.begin('rail', 'right', VELOCITY_SCALE, ACCELERATION_SCALE)
            source = self.config
            ready = self.ready(teaching_waist=n.session.current['waist_yaw_joint'])
            revision, epoch = self.revision, int(n.gateway['task_stop_epoch'])
            waist = {k:n.gateway[k] for k in ('waist_control_mode', 'waist_control_revision')}
            n.display_pub.publish(DisplayTrajectory())
            n.status = '正在规划滑轨路径；尚未发送运动'
        directory = n.run_dir/f'rail_plan_{ticket.generation:04d}'
        def work():
            try:
                from real_task_app import TaskNode
                config = copy.deepcopy(source)
                self.validate(config)
                hands = (n.read_hands(ticket) if path_source == 'recorded' else
                         self.read_stable_hands(ticket, lambda:n.check(ticket)))
                with n.lock:
                    n.session.check(ticket)
                    n.status = '手指位置已读取；正在生成滑轨轨迹及速度规划，尚未发送运动'
                params, config_record = load_parameters(getattr(n.args, 'planning_config', DEFAULT_CONFIG))
                TaskNode.wait_stationary(n,ticket,guard=lambda:self.gateway_guard(),context='滑轨规划')
                with n.lock:
                    n.session.check(ticket); self.fresh_hands(hands)
                    q = self.start_guard(operation, mode, path_source)
                    for side in ARMS:
                        q.update(model_joints(side, hands[side]['feedback_q']))
                    postures = (['grip']+(['pull_grip'] if 'pull_grip' in config['hands'] else [])
                                if operation == 'pull' and mode == 'hold' else ['open'])
                    if path_source != 'recorded' and not any(max(abs(a-b) for a,b in zip(hands['right']['feedback_q'],
                            config['hands'][posture]['feedback_q'])) <= .02 for posture in postures):
                        raise RuntimeError('Set the recorded open/grip hand posture before planning')
                    params, config_record = controller_bounded_parameters(params, config_record, n.gateway)
                    if overrides:params,config_record = rail_panel_parameters(params,config_record,overrides)
                builder = traces.build_replay if path_source == 'recorded' else build
                cache_identity = None
                if path_source == 'recorded':
                    from task_rail_cache import cycle_key,require_success,record_success
                    cache_identity = cycle_key(config,n.model,params,ready,operation,mode)
                    if reuse:require_success(self.cache_dir,cache_identity)
                options = dict(cache_dir=self.cache_dir,require_cache=reuse) if path_source == 'recorded' else {}
                if path_source == 'recorded':
                    from task_rail_planning import build_isolated
                    def progress(message):
                        with n.lock:
                            n.session.check(ticket)
                            n.status = message+'；未发送运动'
                    plan = build_isolated(n.args.robot_urdf,config,ready,q,operation,mode,params,directory,
                        lambda:n.check(ticket),progress,**options)
                else:
                    plan = builder(n.model, config, ready, q, operation, mode, params, lambda:n.check(ticket),**options)
                plan['rail']['path_source'] = path_source
                plan['rail'].update(reuse=reuse,revision=revision, stop_epoch=epoch, config_sha256=fingerprint(config),
                    ready=ready, trajectory_sha256=range_fingerprint(plan['segments']), **waist)
                plan.update(directory=str(directory), planning_config=config_record)
                write_json(directory/'config.json', config)
                write_json(directory/'plan.json', {**plan, 'segments':[
                    {**s, 'trajectory':message_to_ordereddict(s['trajectory'])} for s in plan['segments']]})
                self.execution_guard(plan, preflight=True)
                TaskNode.wait_stationary(n,ticket,expected=q,guard=lambda:self.execution_guard(plan),
                                         context='滑轨规划就绪')
                with n.lock:
                    n.session.check(ticket); self.execution_guard(plan)
                    n.session.ready(ticket, plan)
                    if cache_identity is not None:record_success(self.cache_dir,cache_identity)
                    n.display_pub.publish(DisplayTrajectory(trajectory_start=robot_state(q), trajectory=[
                        RobotTrajectory(joint_trajectory=s['trajectory']) for s in plan['segments']]))
                    hits = sum(s.get('trace_report',{}).get('replay_cache',{}).get('hit',False) for s in plan['segments'])
                    fallbacks = sum(s.get('trace_report',{}).get('interpolation') == 'quintic_measured_waypoint_fallback'
                                    for s in plan['segments'])
                    duration = sum(s['duration'] for s in plan['segments'])
                    speed = '已验证限速内尽快' if params.rail_replay_fastest else f'请求 {params.rail_replay_multiplier:g} 倍速'
                    if path_source != 'recorded':speed = '端点规划'
                    n.status = (f'滑轨计划完成，预计 {duration:.1f} 秒（不含到位等待），{speed}；'
                        f'共 {len(plan["segments"])} 段，复用 {hits} 段缓存，贴合回退 {fallbacks} 段；核对预览后执行')
                    if params.rail_replay_cruise:
                        from task_cruise_timing import summary
                        for segment in plan['segments']:
                            timing=segment.get('trace_report',{}).get('timing',{})
                            if timing.get('generation_mode')=='cruise':
                                n.status+='\n'+segment['label']+'：'+summary(timing)
            except Exception as error:
                with n.lock:
                    n.session.fail(ticket)
                write_json(directory/'failure.json', dict(message=str(error), hardware_executed=False))
                raise
        n.launch_work(work)

    def execution_guard(self, plan, running=False, preflight=False):
        n, proof = self.node, plan['rail']
        if not running:
            (self.arm_gateway_guard if proof.get('arm_only') else self.gateway_guard)()
        if (proof['revision'] != self.revision or proof['stop_epoch'] != int(n.gateway['task_stop_epoch']) or
                n.gateway.get('task_stop_active') != 'false' or
                any(proof[k] != n.gateway.get(k) for k in ('waist_control_mode', 'waist_control_revision'))):
            raise RuntimeError('Rail configuration/STOP/waist mode changed; replan')
        if plan.get('recorded_history'):
            from task_recorded_history import guard
            guard(self,plan,preflight=preflight)
            return
        if preflight:
            if proof.get('operation') == 'move':
                if (self.config['identity'] != self.identity() or proof['target_joints'] !=
                        self.movement_target(proof['target'],waist=plan['start']['waist_yaw_joint']) or
                        proof['trajectory_sha256'] != range_fingerprint(plan['segments'])):
                    raise RuntimeError('移动目标或轨迹已改变，请重新规划')
                return
            self.start_guard(proof['operation'], proof['mode'], proof.get('path_source','endpoints'))
            self.validate(self.config)
            if (proof['config_sha256'] != fingerprint(self.config) or proof['ready'] != self.ready(
                    teaching_waist=plan['start']['waist_yaw_joint']) or
                    proof['trajectory_sha256'] != range_fingerprint(plan['segments'])):
                raise RuntimeError('Rail teaching/ready pose/trajectory changed; replan')
        # The accepted arm action uses the gateway LowState watchdog. Measured
        # freshness and hold checks remain at admission and segment boundaries.

    def started(self):
        self.cancel_trace('STOP/执行请求中止轨迹录制；原记录保留')
        self.state = 'unknown'
        self.state_epoch = None

    def completed(self, plan):
        if plan['rail'].get('operation') == 'move':
            self.state = 'unknown'; self.state_epoch = None
            self.node.status = '已移动到'+plan['rail']['label']+'；手型保持，滑杆状态待确认'
            write_json(Path(plan['directory'])/'rail_result.json',
                dict(state='unknown',target=plan['rail']['target'],completed_at=now(),contact_verified=False))
            return
        self.state = plan['rail']['final_state']
        self.state_epoch = plan['rail']['stop_epoch']
        self.node.status = {'outer':'已拉回外端、松手并回右手准备位',
            'inner_held':'已推入内端，保持握持；等待单独下发拉出指令',
            'inner_released':'已推入内端、松手并回右手准备位；等待单独下发拉出指令'}[self.state]
        write_json(Path(plan['directory'])/'rail_result.json',
                   dict(state=self.state, completed_at=now(), contact_verified=False))
