"""D-ring teaching/replay in the existing hardware owner; no visual planning."""
import copy
import hashlib
import json
import math
from pathlib import Path
import threading
import time

from moveit_msgs.msg import DisplayTrajectory,DisplayRobotState,RobotTrajectory
from rosidl_runtime_py.convert import message_to_ordereddict

from task_console_handoff import supported_controller,supported_gateway
from task_hand_control import require_state,model_joints,ORDER,OPEN,CLOSED,POSTURES
from task_motion_planner import robot_state
from task_planning_config import load_parameters,controller_bounded_parameters,DEFAULT_CONFIG
from task_rail_capture import DirectCapture
from task_saved_test import range_fingerprint
from task_screen_control import ScreenControl
from task_screen_teach import fingerprint,write_json,now
from task_session import ARMS
import task_dring_teach_data as data
from task_trace_editor_control import TraceEditorControl

STORE=Path(__file__).resolve().parents[1]/'data/task_panel/dring_demonstration.json'


class DringDemoControl(TraceEditorControl):
    data = data
    workflow = 'dring_demo'
    default_side = 'left'
    arm_only = True
    capture_namespace = 'dring_recordings'
    replay_cache_name = 'dring_replay_cache'
    trim_history_name = 'dring_trim_history'
    read_stable_hands=ScreenControl.read_stable_hands
    fresh_hands=ScreenControl.fresh_hands

    def __init__(self,node,store=STORE):
        self.node=node;self.store=Path(store);self.revision=0;self.recording=None;self.path=None
        self.preview_visible=False
        self.config=self.empty(self.default_side);self.status='尚未录入准备位和轨迹'
        if self.store.is_file():
            try:
                from task_trace_selection import restore
                self.config=self.validate(restore(json.loads(self.store.read_text()),self.store));self.path=str(self.store)
                self.remember_selection_source(self.store)
                self.status='已载入上次示教；未恢复执行授权'
            except Exception as error:self.status='上次示教未加载：'+str(error)

    def identity(self):
        return dict(robot_sha256=hashlib.sha256(Path(self.node.args.robot_urdf).read_bytes()).hexdigest(),
                    hand_mapping=dict(order=list(ORDER),open=list(OPEN),closed=list(CLOSED)))

    def empty(self,side):
        self.data.moving(side)
        return dict(schema=self.data.SCHEMA,identity=self.identity(),side=side,ready=None,traces={},saved_at=now())

    def idle(self):
        if self.recording is not None:raise RuntimeError('请先结束或取消本次录制')
        ScreenControl.idle(self)

    def gateway_guard(self,hold=True,*,stationary_velocity=None):
        n=self.node
        if not n.args.hardware or n.args.disconnected:raise RuntimeError('录入与执行需要硬件面板')
        if time.monotonic()-n.gateway_received>1.5:raise RuntimeError('网关反馈已过期')
        if stationary_velocity is None:stationary_velocity = self.workflow != 'dring_demo'
        # Recorded workflows admit motion using their measured position window;
        # an instantaneous velocity spike during a finger ramp is not a handoff.
        (supported_gateway if hold and stationary_velocity else supported_controller)(n.gateway)
        if n.gateway.get('waist_mode_switching')!='false':raise RuntimeError('腰控制模式正在切换')
        if n.gateway.get('task_waist_trajectory_supported')!='true':raise RuntimeError('当前网关没有启用腰轨迹')
        lo,hi=n.model.limits['waist_yaw_joint']
        if any(not math.isclose(float(n.gateway.get(key,'nan')),value,abs_tol=1e-5,rel_tol=0.)
               for key,value in (('waist_guard_lower_rad',lo+.02),('waist_guard_upper_rad',hi-.02))):
            raise RuntimeError('模型与控制器的腰限位不一致')
        if hold and (n.gateway.get('phase')!='ARMED_HOLD' or n.gateway.get('task_stop_active')!='false'):
            raise RuntimeError('请先退出导纳并原地保持')
        if not hold and n.gateway.get('phase') not in ('TASK_ADMITTANCE','ARMED_HOLD','RUNNING'):
            raise RuntimeError('当前控制器状态不支持录制/回放')

    def validate(self,config):
        if config.get('schema')!=self.data.SCHEMA or config.get('identity')!=self.identity():
            raise ValueError('示教文件的机器人模型或手指映射不匹配')
        names=self.data.moving(config['side'])
        if not isinstance(config.get('traces'),dict) or not set(config['traces'])<=set(self.data.SLOTS):
            raise ValueError('示教文件轨迹槽无效')
        retained=config.get('retained_slots')
        if retained is not None and (not isinstance(retained,list) or not retained or
                len(set(retained))!=len(retained) or not set(retained)<=set(self.data.SLOTS)):
            raise ValueError('保留轨迹槽列表无效')
        ready=config.get('ready')
        if ready is not None:
            self.data.joint_array(self.node.model,names,[[ready['joints'][n] for n in names]])
            if ready.get('hand_q') is not None:self.data.checked_q(ready['hand_q'])
        for trace in config['traces'].values():
            if ready is None:raise ValueError('录制缺少准备位')
            q,_=self.data.arrays(trace,self.node.model,config['side'])
            if max(abs(q[0,i]-ready['joints'][n]) for i,n in enumerate(names))>.025:
                raise ValueError('录制起点与准备位不一致')
        from task_trace_versions import validate
        validate(config,lambda trace:self.data.arrays(trace,self.node.model,config['side']))
        return config

    def invalidate(self):
        self.revision+=1;self.node.session.invalidate()
        if self.node.session.phase=='PLAN_READY':self.node.session.phase='REPLAN_REQUIRED'
        self.node.display_pub.publish(DisplayTrajectory())

    def persist(self,config):
        self.validate(config)
        write_json(self.node.run_dir/(self.workflow+'_autosave')/f'{time.time_ns()}.json',config)
        write_json(self.store,config)
        write_json(self.side_store(config['side']),config)
        with self.node.lock:
            self.config=config;self.remember_selection_source(self.store);self.invalidate()

    def edit_trace(self,slot,start=None,end=None,bridge=.6,undo=False):
        if not undo:
            return self.edit_version(slot, 'cut', start=start, end=end, bridge=bridge)
        from task_trace_versions import bank
        entry=bank(self.config,slot)
        if entry['selected']:
            return self.edit_version(slot,'select',index=0)
        from task_dring_trim import cut
        n=self.node
        with n.lock:
            self.idle();source=self.config
            if slot not in source['traces']:raise ValueError('该槽位尚未录制')
            previous=source['traces'][slot]
        def work():
            if undo:
                path=previous.get('trim_edit',{}).get('undo_path')
                if not path:raise ValueError('该槽位没有可撤销的裁切')
                replacement=json.loads(Path(path).read_text())
            else:
                replacement=cut(previous,start,end,bridge)
                backup=self.store.parent/self.trim_history_name/f'{time.time_ns()}_{slot}.json'
                write_json(backup,previous)
                replacement['trim_edit']['undo_path']=str(backup)
            config=copy.deepcopy(source);config['traces'][slot]=replacement
            # Disk I/O and full-rate validation stay outside the feedback lock.
            self.persist(config)
            self.status=self.data.SLOTS[slot]+('裁切已撤销' if undo else '裁切已保存')+'；未发送真机指令'
            n.status=self.status
        n.launch_work(work)

    def preview_pose(self,positions):
        """Display-only frames: no measured-state wait, IK, timing solve or action."""
        n=self.node;names=self.data.moving(self.config['side'])
        with n.lock:
            if self.recording is not None or n.session.phase in ('PLANNING','EXECUTION_PREFLIGHT','EXECUTING'):
                return
            q=dict(getattr(n.session,'current',{}) or {})
            q.update(zip(names,map(float,positions)))
            if not self.preview_visible:
                n.display_pub.publish(DisplayTrajectory())
            n.demo_preview_pub.publish(DisplayRobotState(state=robot_state(q)))
            self.preview_visible=True

    def hide_preview(self):
        with self.node.lock:
            if self.preview_visible:
                self.node.demo_preview_pub.publish(DisplayRobotState(hide=True))
                self.preview_visible=False

    def side_store(self,side):
        self.data.moving(side)
        return self.store.with_name(self.store.stem+'_'+side+self.store.suffix)

    def select_side(self,side):
        with self.node.lock:
            self.idle()
            if side==self.config['side']:return
            path=self.side_store(side)
            config=self.validate(json.loads(path.read_text())) if path.is_file() else self.empty(side)
            if config['side']!=side:raise ValueError('保存的执行手臂不匹配')
            write_json(self.side_store(self.config['side']),self.config)
            self.persist(config);self.path=None;self.status=('左臂' if side=='left' else '右臂')+'录制已切换；未发送运动'

    def preparation(self):
        side=self.config['side'];pose=self.node.prepared.get(side)
        if not pose:raise RuntimeError('当前手臂没有系统准备位，请先记录 above-table preparation')
        names=self.data.moving(side)
        target={n:pose['joints'][n] for n in names}
        self.data.joint_array(self.node.model,names,[[target[n] for n in names]])
        return target

    def new(self,side):
        with self.node.lock:
            self.idle();self.persist(self.empty(side));self.path=None;self.status='新建示教；旧录制原始文件保留'

    def save(self,path):
        with self.node.lock:
            self.idle();self.validate(self.config);write_json(path,self.config);self.path=str(path)
            if Path(path).resolve()==self.store.resolve():self.remember_selection_source(self.store)

    def load(self,path):
        path=Path(path)
        if path.stat().st_size>800_000_000:raise ValueError('示教文件过大')
        from task_trace_selection import restore
        config=self.validate(restore(json.loads(path.read_text()),path))
        with self.node.lock:
            self.idle();self.persist(config);self.path=str(path);self.status='示教已载入，未发送运动'

    def index_middle(self):
        n=self.node
        with n.lock:
            self.idle();self.gateway_guard()
            if not n.hand_control:raise RuntimeError('当前面板未启用手指控制')
            initial=n.session.measured(stationary=True);side=self.config['side']
            self.invalidate();generation=n.session.generation;epoch=n.gateway['task_stop_epoch']
            self.status='伸出食指和中指；手臂保持';n.status=self.status
        def check():
            n.check()
            with n.lock:
                if generation!=n.session.generation or epoch!=n.gateway['task_stop_epoch']:
                    raise RuntimeError('手指操作被 STOP 或会话变化取消')
                self.gateway_guard()
                q=n.session.measured()
                if any(abs(q[k]-initial[k])>.025 for k in (*ARMS['left'],*ARMS['right'],'waist_yaw_joint')):
                    raise RuntimeError('手指动作期间手臂或腰发生移动')
        def work():
            try:
                n.hand.begin_motion(check)
                target=list(POSTURES['index_only']);target[ORDER.index('middle_1')]=1.
                n.hand.transition(side,tuple(target),check,duration=1.,
                                  on_feedback=lambda state:n.cache_hand_state(side,state))
                check()
                with n.lock:self.status='食指和中指伸出，其他手指收拢；手臂未移动';n.status=self.status
            except Exception:
                n.stop();raise
        n.launch_work(work)

    def record_ready(self,with_hand=False):
        n=self.node
        with n.lock:
            self.idle();self.gateway_guard(False)
            initial=n.session.measured(stationary=True);generation=n.session.generation
            epoch=n.gateway['task_stop_epoch'];side=self.config['side']
        def check():
            n.check()
            with n.lock:
                if generation!=n.session.generation or epoch!=n.gateway['task_stop_epoch']:
                    raise RuntimeError('记录准备位期间状态改变，未保存')
                self.gateway_guard(False)
        def work():
            hand=None
            if with_hand:
                if not n.hand_control:raise RuntimeError('未启用手反馈；可取消同步手指后重新录入')
                hand=require_state(n.hand.read(side,check),side);n.cache_hand_state(side,hand)
            with n.lock:
                check();q=n.session.measured(stationary=True)
                names=self.data.moving(side)
                if max(abs(q[k]-initial[k]) for k in names)>.012:raise RuntimeError('记录期间仍在移动')
                config=self.empty(side)
                config['ready']=dict(joints={k:q[k] for k in names},hand_q=hand['feedback_q'] if hand else None,
                                     recorded_at=now())
                self.persist(config);self.status='准备位已保存；左右各五个槽位等待录制';n.status=self.status
        n.launch_work(work)

    def cancel_trace(self,reason='已取消；旧轨迹保留'):
        with self.node.lock:
            if self.recording is not None:
                self.recording['error']=reason;self.recording['cancel'].set()

    def finish_trace(self):
        with self.node.lock:
            if self.recording is None or self.recording['phase']!='recording':raise RuntimeError('没有正在录制的轨迹')
            rec=self.recording
            rec['receiver'].request_finish(time.monotonic())
            rec['finish'].set();rec['phase']='saving'
            self.status='录制已截止，正在保存';self.node.status=self.status

    def start_trace(self,slot,with_hand=False):
        if slot not in self.data.SLOTS:raise ValueError('请选择有效录制段')
        n=self.node
        with n.lock:
            self.idle();self.gateway_guard(False)
            if n.gateway['phase']!='TASK_ADMITTANCE':raise RuntimeError('请先进入导纳，再手动拖动录制')
            if with_hand and not n.hand_control:raise RuntimeError('手部反馈未启用，可取消同步手指')
            initial=n.session.measured(stationary=True);side=self.config['side'];names=self.data.moving(side)
            target=self.recording_start(slot)
            if target is not None and max(abs(initial[k]-target[k]) for k in names)>.025:
                raise RuntimeError('请先拖回本段起点，再开始录制')
            self.invalidate()
            rec=dict(slot=slot,phase='starting',generation=n.session.generation,epoch=n.gateway['task_stop_epoch'],
                waist_mode=(n.gateway.get('waist_control_mode'),n.gateway.get('waist_control_revision')),
                receiver=None,cancel=threading.Event(),finish=threading.Event(),error=None)
            self.recording=rec;n.session.phase='DEMO_RECORDING';self.status='直采启动中，保持静止'
        def check():
            n.check()
            with n.lock:
                if rec['cancel'].is_set():raise RuntimeError(rec['error'])
                if (n.session.generation!=rec['generation'] or n.gateway['task_stop_epoch']!=rec['epoch'] or
                        n.gateway.get('phase')!='TASK_ADMITTANCE' or rec['waist_mode']!=
                        (n.gateway.get('waist_control_mode'),n.gateway.get('waist_control_revision'))):
                    raise RuntimeError('STOP、导纳或会话变化，录制未覆盖旧文件')
                self.gateway_guard(False)
                if rec['receiver'] is not None:rec['receiver'].check()
        def work():
            evidence=[]
            def read_hand():
                check();state=require_state(n.hand.read(side,check),side);received=time.monotonic()
                if evidence and (state['timestamp_ms']<=evidence[-1]['timestamp_ms'] or received-evidence[-1]['received']>.75):
                    raise RuntimeError('手指反馈未推进或录制间隔过大；旧轨迹保留')
                n.cache_hand_state(side,state)
                evidence.append(dict(received=received,timestamp_ms=state['timestamp_ms'],q=list(state['feedback_q'])))
            try:
                if with_hand:read_hand()
                rec['receiver']=DirectCapture(n,slot,initial,rec['generation'],rec['epoch'],side=side,
                    namespace=self.capture_namespace,full_rate=True)
                next_hand=time.monotonic()+.1
                while not rec['receiver'].check().get('ready'):
                    check()
                    if with_hand and time.monotonic()>=next_hand:read_hand();next_hand=time.monotonic()+.1
                    time.sleep(.01)
                with n.lock:rec['phase']='recording';self.status=self.data.SLOTS[slot]+'录制中';n.status=self.status
                while not rec['finish'].is_set():
                    check()
                    if with_hand and time.monotonic()>=next_hand:read_hand();next_hand=time.monotonic()+.1
                    time.sleep(.01)
                raw=rec['receiver'].finish(check)
                if with_hand:read_hand()
                with n.lock:rec['phase']='saving';self.status='保存实测轨迹'
                hand=(dict(time_s=[e['received']-raw['received_monotonic_start'] for e in evidence],
                           q=[e['q'] for e in evidence],feedback=evidence) if with_hand else None)
                trace=dict(raw=raw,hand=hand,recorded_at=now(),capture_directory=str(rec['receiver'].directory))
                self.data.arrays(trace,n.model,side)
                with n.lock:
                    check();config=copy.deepcopy(self.config);config['traces'][slot]=trace
                    config.get('trace_versions',{}).pop(slot,None)
                    self.persist(config)
                    n.session.phase='TASK_ADMITTANCE';self.status=f'{self.data.SLOTS[slot]}已保存，{raw["time_s"][-1]:.2f} 秒';n.status=self.status
            except Exception as error:
                with n.lock:
                    self.status=str(error);n.status=self.status
                    if n.session.generation==rec['generation']:n.session.phase=n.gateway.get('phase','REPLAN_REQUIRED')
                raise
            finally:
                if rec['receiver'] is not None:rec['receiver'].close()
                with n.lock:
                    if self.recording is rec:self.recording=None
        try:
            n.launch_work(work)
        except Exception:
            with n.lock:
                if self.recording is rec:self.recording=None
                if n.session.phase=='DEMO_RECORDING':n.session.phase=n.gateway.get('phase','REPLAN_REQUIRED')
            raise

    def recording_start(self,slot):
        if self.config['ready'] is None:raise RuntimeError('请先录入准备位')
        return self.config['ready']['joints']

    def preparation_required(self,operation,full_cycle):
        return full_cycle or operation=='preparation'

    def planning_start(self,ticket,guard):
        return self.node.wait_stationary(ticket,guard=guard,context='录制回放生成')

    def prepare(self,operation,multiplier,velocity,acceleration,confirmed=False,*,full_cycle=False):
        self.hide_preview()
        if operation not in (*self.data.SLOTS,'ready','preparation') or not confirmed:raise RuntimeError('请选择动作并确认固定工位和路径净空')
        if not math.isfinite(multiplier) or multiplier<=0:raise ValueError('倍率必须是正数')
        n=self.node
        with n.lock:
            self.idle();self.gateway_guard()
            if operation in self.data.SLOTS and operation not in self.config['traces']:raise RuntimeError('该方向尚未录制')
            preparation=self.preparation() if self.preparation_required(operation,full_cycle) else None
            ticket=n.session.begin('dring_demo',self.config['side'],velocity,acceleration)
            source=self.config;revision=self.revision;epoch=n.gateway['task_stop_epoch']
            timing_mode=getattr(self,'generation_mode','original')
            waist={k:n.gateway.get(k) for k in ('waist_control_mode','waist_control_revision')}
            n.display_pub.publish(DisplayTrajectory());n.status='生成录制回放；不采集图像、不求解 IK'
        directory=n.run_dir/f'{self.workflow}_{ticket.generation:04d}'
        def work():
            try:
                # Full-rate recordings must not block the measured-feedback lock.
                config=copy.deepcopy(source)
                self.validate(config)
                source_fingerprint=fingerprint(config)
                # Raw finger timelines stay archived; workflows may add explicit hand stages.
                if config['ready']:config['ready']['hand_q']=None
                for trace in config['traces'].values():trace['hand']=None
                params,record=load_parameters(getattr(n.args,'planning_config',DEFAULT_CONFIG))
                def guard():
                    self.gateway_guard()
                    if self.config is not source or self.revision!=revision or n.gateway['task_stop_epoch']!=epoch:
                        raise RuntimeError('示教或 STOP 状态变化，请重新生成回放')
                q=self.planning_start(ticket,guard)
                with n.lock:
                    n.session.check(ticket);guard()
                    params,record=controller_bounded_parameters(params,record,n.gateway)
                def progress(text):
                    with n.lock:
                        n.session.check(ticket);n.status=text
                plan=self.data.build(n.model,config,q,operation,params,velocity,acceleration,multiplier,lambda:n.check(ticket),
                                preparation=preparation,full_cycle=full_cycle,
                                cache_dir=self.store.parent/self.replay_cache_name,progress=progress,timing_mode=timing_mode)
                plan.update(directory=str(directory),planning_config=record)
                plan['demo'].update(workflow=self.workflow,revision=revision,stop_epoch=epoch,config_sha256=source_fingerprint,arm_only=self.arm_only,
                    preparation=preparation,
                    trajectory_sha256=range_fingerprint(plan['segments']),hand_sha256=fingerprint(
                        [s.get('recorded_hand') for s in plan['segments']]),**waist)
                write_json(directory/'config.json',config)
                write_json(directory/'plan.json',dict(plan,segments=[dict(s,trajectory=message_to_ordereddict(s['trajectory'])) for s in plan['segments']]))
                display=self.data.preview(plan)
                with n.lock:
                    n.session.check(ticket);self.execution_guard(plan,preflight=True)
                n.wait_stationary(ticket,expected=q,guard=guard,context='录制回放就绪')
                with n.lock:
                    n.session.check(ticket);guard();n.session.ready(ticket,plan)
                    n.display_pub.publish(DisplayTrajectory(trajectory_start=robot_state(q),trajectory=[RobotTrajectory(joint_trajectory=display)]))
                    self.status=f'回放就绪：{sum(s["duration"] for s in plan["segments"]):.2f} 秒；未发送运动'
                    if plan['speed_scaling'].get('generation_mode')=='cruise':
                        from task_cruise_timing import summary
                        self.status+='；'+summary(plan['speed_scaling'])
                    n.status=self.status
            except Exception as error:
                with n.lock:n.session.fail(ticket);self.status=str(error)
                raise
        n.launch_work(work)

    def replay_preview(self):
        self.hide_preview()
        n=self.node
        with n.lock:
            self.idle();plan=n.session.plan
            if not plan or plan.get('kind')!='dring_demo' or plan['demo'].get('workflow','dring_demo')!=self.workflow:
                raise RuntimeError('请先生成本页回放')
            generation=n.session.generation
        def work():
            tr=self.data.preview(plan)
            with n.lock:
                if generation!=n.session.generation or n.session.plan is not plan:raise RuntimeError('回放已改变')
                n.display_pub.publish(DisplayTrajectory(trajectory_start=robot_state(plan['start']),
                    trajectory=[RobotTrajectory(joint_trajectory=tr)]))
                self.status='已重新发送 RViz 回放；未发送真机指令';n.status=self.status
        n.launch_work(work)

    def execution_guard(self,plan,running=False,preflight=False):
        n=self.node;proof=plan['demo']
        if proof.get('workflow','dring_demo')!=self.workflow:raise RuntimeError('示教业务不匹配，请重新生成回放')
        if not running:self.gateway_guard()
        if (proof['revision']!=self.revision or proof['stop_epoch']!=n.gateway.get('task_stop_epoch') or
                n.gateway.get('task_stop_active')!='false' or
                any(proof[k]!=n.gateway.get(k) for k in ('waist_control_mode','waist_control_revision'))):
            raise RuntimeError('示教、STOP 或腰模式变化，请重新生成回放')
        if plan.get('recorded_history'):
            from task_recorded_history import guard
            guard(self,plan,preflight=preflight)
            return
        if preflight:
            if proof.get('preparation') is not None and proof['preparation']!=self.preparation():
                raise RuntimeError('系统准备位已改变，请重新生成完整回放')
            if not n.hand_control and any(s.get('kind')=='hand' or s.get('recorded_hand') is not None for s in plan['segments']):
                raise RuntimeError('录制手指回放需要手部控制')
            if (proof['config_sha256']!=fingerprint(self.config) or
                    proof['trajectory_sha256']!=range_fingerprint(plan['segments']) or
                    proof['hand_sha256']!=fingerprint([s.get('recorded_hand') for s in plan['segments']])):
                raise RuntimeError('录制回放内容改变，请重新生成回放')
        # Accepted actions use the controller's own watchdog and action result.
        # Do not turn delayed UI diagnostics/joint messages into a second watchdog.
        # STOP/configuration invalidation above and measured boundary checks remain.

    def completed(self,plan):
        self.status=('完整循环完成，已回桌上方准备位' if plan['demo'].get('full_cycle') else
                     '回位完成' if plan['demo']['operation'] in ('ready','preparation') else '录制回放完成，保持记录终点')
        self.node.status=self.status
