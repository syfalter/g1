"""Shared offline version editing for the three recorded workflows."""
import copy

from task_trace_versions import EDIT_SLOTS, bank, append, select_shared, delete


class TraceEditorControl:
    def editor_playback(self, trace, slot, position):
        n=self.node
        if not (getattr(n.args,'offline_editor',False) and n.args.disconnected):
            return False
        from task_trace_playback import trajectory,message
        from moveit_msgs.msg import DisplayRobotState
        with n.lock:
            self.idle()
            if not hasattr(n,'_offline_preview_health_subscription'):
                import json
                import time
                from std_msgs.msg import String
                def feedback(message):
                    try:n.offline_preview_health=(time.monotonic(),json.loads(message.data))
                    except (ValueError,TypeError):pass
                n._offline_preview_health_subscription=n.create_subscription(
                    String,'/g1/offline_preview/status',feedback,1)
            cached=getattr(self,'_editor_playback_cache',None)
            if cached is None or cached[0] is not trace:
                cached=(trace,trajectory(trace,n.model,self.editor_side()))
                self._editor_playback_cache=cached
            preview=message(cached[1],trace,position,n.model,self.editor_side(),
                            dict(getattr(n.session,'current',{}) or {}),self.config,slot)
            n.demo_preview_pub.publish(DisplayRobotState(hide=True))
            n.display_pub.publish(preview)
            self._editor_playback_active=True
        return True

    def editor_stop_playback(self):
        if getattr(self,'_editor_playback_active',False):
            from moveit_msgs.msg import DisplayTrajectory
            self.node.display_pub.publish(DisplayTrajectory())
            self._editor_playback_active=False

    def set_generation_mode(self,mode):
        if mode not in ('original','cruise'):raise ValueError('Unknown timing mode')
        with self.node.lock:
            self.idle()
            if mode==getattr(self,'generation_mode','original'):return
            self.generation_mode=mode
            self.invalidate()
            self.editor_feedback='生成方式已切换，请重新生成预览或执行计划；已有历史不变'

    def preview_timing(self,slot,mode,multiplier):
        from task_dring_teach_data import fit_trace,moving
        from task_trace_markers import hand_context
        from task_planning_config import load_parameters,DEFAULT_CONFIG
        from task_trajectory_speed import scale_arm_segments
        from task_cruise_timing import retime,summary
        from task_motion_planner import robot_state
        from moveit_msgs.msg import DisplayTrajectory,RobotTrajectory
        n=self.node
        with n.lock:
            self.idle()
            if mode not in ('original','cruise'):raise ValueError('Unknown timing mode')
            source=self.config;revision=self.revision
            if slot not in source['traces']:raise ValueError('该段尚未录制')
            trace=source['traces'][slot];side=self.editor_side()
            context=dict(getattr(n.session,'current',{}) or {})
            context.update(trace.get('context_joints',{}))
            context=hand_context(trace,side,context,source,slot)
            if self.workflow=='rail' and slot=='pull' and 'pull_grip' in source.get('hands',{}):
                from task_hand_control import model_joints
                context.update(model_joints(side,source['hands']['pull_grip']['feedback_q']))
            context.update(zip(moving(side),trace['raw']['positions'][0]))
            generation=getattr(n.session,'generation',None)
            self.editor_feedback='正在生成时序预览；未发送真机指令'
        def check():
            if (n.shutdown_requested or self.config is not source or self.revision!=revision or
                    getattr(n.session,'generation',None)!=generation):
                raise RuntimeError('预览计算期间状态已改变')
        def work():
            try:
                params,_=load_parameters(getattr(n.args,'planning_config',DEFAULT_CONFIG))
                # This is a single-stroke visual preview, never an execution authorization.
                if self.workflow=='rail':
                    from dataclasses import replace
                    from task_rail_trace import replay
                    params=replace(params,rail_replay_cruise=mode=='cruise',
                                   rail_replay_multiplier=multiplier,rail_replay_fastest=False)
                    trajectory,details=replay(n.model,trace,context,params,check)
                    report=details['timing']
                else:
                    trajectory,_=fit_trace(dict(trace,hand=None),n.model,side,check)
                    if mode=='cruise':
                        trajectory,report=retime(trajectory,n.model,context,side,multiplier,params,
                            velocity=.7,acceleration=.7,check=check,
                            original_policy=trace.get('optimization',{}).get('timing_policy','recorded'))
                    else:
                        scaled,report=scale_arm_segments([dict(trajectory=trajectory)],multiplier,n.model,params,
                            velocity=.7,acceleration=.7,check=check,
                            policy=trace.get('optimization',{}).get('timing_policy','recorded'))
                        trajectory=scaled[0]['trajectory']
                with n.lock:
                    check()
                    from moveit_msgs.msg import DisplayRobotState
                    n.demo_preview_pub.publish(DisplayRobotState(hide=True))
                    n.display_pub.publish(DisplayTrajectory(trajectory_start=robot_state(context),
                        trajectory=[RobotTrajectory(joint_trajectory=trajectory)]))
                    text=(summary(report) if mode=='cruise' else
                          f'原方式预览：{report["duration_after_s"]:.2f} s')
                    self.editor_feedback=text+'；仅预览，真机执行须在完整面板生成计划'
                    n.status=self.editor_feedback
            except Exception as error:
                self.editor_feedback='不成功：'+str(error)
                raise
        n.launch_work(work)

    def edit_version(self, slot, action, **options):
        n = self.node
        with n.lock:
            self.idle()
            source, revision = self.config, self.revision
            generation = getattr(n.session,'generation',None)
            if slot not in source['traces']:
                raise ValueError('该段尚未录制')
            if action not in ('select', 'delete') and None not in bank(source, slot)['edits']:
                raise ValueError(f'{EDIT_SLOTS} 个修改位置已满，请先删除一个版本')
            if action=='select' and type(options['index']) is int and options['index']==bank(source,slot)['selected']:return
            self.editor_feedback = '正在处理轨迹版本，请稍候'
            if action=='fastest':
                self.editor_feedback='正在后台比较原策略与局部最快时序，并检查连续运动限值；未发送真机指令'

        def work():
            from task_trace_edit import cut, trim_tail, smooth, reverse_into, anchor, curve, local_speed
            trace = source['traces'][slot]
            if action == 'select':
                updated = select_shared(source, slot, options['index'])
            elif action == 'delete':
                updated = delete(source, slot, options['index'])
            else:
                if action == 'cut':
                    edited = cut(trace, options['start'], options['end'], options['bridge'])
                elif action == 'trim_tail':
                    edited = trim_tail(trace, options['seconds'])
                elif action == 'fastest':
                    import time
                    from task_trace_timing import generate
                    from task_trace_timing_cache import DEFAULT_DIRECTORY
                    from task_planning_config import load_parameters,DEFAULT_CONFIG
                    started=time.monotonic()
                    phase=['准备计算',0.]
                    def check():
                        if (n.shutdown_requested or self.config is not source or self.revision!=revision or
                                getattr(n.session,'generation',None)!=generation):
                            raise RuntimeError('重定时期间状态已改变，未保存')
                        now=time.monotonic()
                        if now-phase[1]>=.5:
                            with n.lock:
                                self.editor_feedback=f'重定时：{phase[0]}；已用 {now-started:.1f} s；未发送真机指令'
                            phase[1]=now
                    def progress(text):
                        phase[:]=[text,0.]
                        check()
                    params,_=load_parameters(getattr(n.args,'planning_config',DEFAULT_CONFIG))
                    edited=generate(trace,n.model,self.editor_side(),params,workflow=self.workflow,
                        config=source,slot=slot,check=check,progress=progress,cache_dir=DEFAULT_DIRECTORY,**options)
                elif action == 'smooth':
                    edited = smooth(trace, **options)
                elif action == 'local_speed':
                    edited = local_speed(trace, **options)
                elif action == 'curve':
                    edited = curve(trace, options['key_times'],options.get('seconds',0.))
                    from task_trace_curve import validate_curve
                    from task_dring_teach_data import moving
                    validate_curve(edited,n.model,moving(self.editor_side()))
                elif action == 'manual':
                    from task_trace_manual import manual_trace
                    context = dict(getattr(n.session,'current',{}) or {})
                    edited = manual_trace(trace,model=n.model,side=self.editor_side(),
                        context=context,config=source,slot=slot,**options)
                elif action == 'reverse':
                    from task_landing_gear_data import ENDPOINTS
                    if self.workflow != 'landing_gear' or ENDPOINTS[slot] != ENDPOINTS[options['source']][::-1]:
                        raise ValueError('反向来源必须是同一对端点的起落架杆轨迹')
                    edited = reverse_into(source['traces'][options['source']], trace)
                    edited['optimization']['source_slot'] = options['source']
                elif action == 'anchor':
                    from task_landing_gear_data import ENDPOINTS
                    if self.workflow != 'landing_gear':raise ValueError('端点约束仅用于起落架杆')
                    reference = options['source']
                    if reference not in ('lower_to_upper','upper_to_lower'):
                        raise ValueError('请选择上下杆主行程')
                    raw = source['traces'][reference]['raw']['positions']
                    poses = dict(zip(ENDPOINTS[reference], (raw[0], raw[-1])))
                    targets = {i:poses[name] for i,name in zip((0,-1),ENDPOINTS[slot]) if name in poses}
                    edited = anchor(trace, targets, options.get('max_delta',.02))
                    edited['optimization']['source_slot'] = reference
                else:
                    raise ValueError('Unknown edit operation')
                from task_screen_teach import now
                edited['edit_origin']=dict(slot=slot,version=bank(source,slot)['selected'],
                                           operation=action,created_at=now())
                updated = append(source, slot, edited)
            if action=='select':
                # Keep the workflow's active-trace contracts, without revalidating every history.
                self.validate(dict(updated,traces={slot:updated['traces'][slot]},trace_versions={}))
            else:self.validate(updated)
            with n.lock:
                if (self.config is not source or self.revision != revision or self.recording is not None or
                        n.shutdown_requested or getattr(n.session,'generation',None) != generation or
                        n.session.phase in ('PLANNING','EXECUTION_PREFLIGHT','EXECUTING')):
                    raise RuntimeError('编辑期间示教数据已改变，未保存')
                if action=='select':self.persist_selection(updated)
                else:self.persist_versions(updated)
                self.status = '轨迹版本已保存；未发送真机指令'
                n.status = self.status
                self.editor_feedback = self.status
                if action=='fastest':
                    info=edited['optimization']['timing_report']
                    self.editor_feedback=(f'重定时版本已保存：{info["duration_after_s"]:.3f} s；'
                        f'原策略 1x：{info["original_1x_seconds"]:.3f} s；'
                        f'相对原 1x：{info["speedup_over_original_1x"]:.2f}x；'
                        f'计算 {info.get("computation_seconds",0.):.1f} s'
                        f'{"（复用结果）" if info.get("result_cache_hit") else ""}；未发送真机指令')
        def run():
            try:work()
            except Exception as error:
                self.editor_feedback = '不成功：'+str(error)
                raise
        n.launch_work(run)

    def persist_versions(self, config):
        self.persist(config)

    def remember_selection_source(self,path):
        from pathlib import Path
        from task_trace_selection import revision
        self.selection_source=(Path(path),revision(path))

    def persist_selection(self,config):
        from task_trace_selection import save
        if not hasattr(self,'selection_source'):
            raise ValueError('当前轨迹尚无已保存的版本库，请先保存')
        save(config,*self.selection_source)
        self.config=config;self.invalidate()
        if hasattr(self,'trace_review'):self.trace_review=None

    def editor_side(self):
        return self.config.get('side', 'right')

    def editor_pose(self, positions, trace=None, slot=None, hand_time=None):
        from moveit_msgs.msg import DisplayRobotState
        from task_motion_planner import robot_state
        from task_dring_teach_data import moving
        n = self.node
        self.editor_stop_playback()
        with n.lock:
            if self.recording is not None or n.session.phase in ('PLANNING','EXECUTION_PREFLIGHT','EXECUTING'):
                return
            q = dict(getattr(n.session, 'current', {}) or {})
            if trace is not None:
                from task_trace_markers import hand_context
                q.update(trace.get('context_joints', {}))
                q = hand_context(trace, self.editor_side(), q, self.config, slot)
                if hand_time is not None and isinstance(trace.get('hand'),dict):
                    import numpy as np
                    from task_hand_control import model_joints
                    hand=trace['hand']
                    q.update(model_joints(self.editor_side(),[np.interp(hand_time,hand['time_s'],axis)
                        for axis in np.asarray(hand['q']).T]))
            q.update(zip(moving(self.editor_side()), map(float, positions)))
            n.demo_preview_pub.publish(DisplayRobotState(state=robot_state(q)))

    def editor_hide(self):
        self.editor_stop_playback()
        from moveit_msgs.msg import DisplayRobotState
        if hasattr(self.node,'demo_preview_pub'):
            self.node.demo_preview_pub.publish(DisplayRobotState(hide=True))
        from task_trace_markers import clear_markers
        if hasattr(self.node, 'marker_pub'):
            self.node.marker_pub.publish(clear_markers())

    def editor_markers(self, slot, start, end):
        from task_trace_markers import trace_markers
        n = self.node
        if not hasattr(n, 'marker_pub') or not hasattr(n.model, 'batch_transforms'):
            return
        trace = self.config['traces'].get(slot)
        if trace is None:
            self.editor_hide()
            return
        context = dict(getattr(n.session, 'current', {}) or {})
        context.update(trace.get('context_joints', {}))
        n.marker_pub.publish(trace_markers(n.model, trace, self.editor_side(), context, start, end,
                                           self.config, slot))
