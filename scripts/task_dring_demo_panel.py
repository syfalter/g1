"""Compact teach-and-replay view; all commands use the existing hardware owner."""
from datetime import datetime
import time
import numpy as np
from PyQt5 import QtCore,QtWidgets

from task_dring_teach_data import SLOTS
from task_speed_control import SpeedInput
from task_numeric_input import FreeDoubleSpinBox


class DringDemoPanel(QtWidgets.QWidget):
    def __init__(self,window):
        super().__init__();self.window=window;self.control=window.node.demo;self.controls=[];self.revision=None
        self.pending_move=None
        self.preview_trace=None;self.preview_time=0.;self.preview_origin=0.
        self.preview_timer=QtCore.QTimer(self);self.preview_timer.setInterval(33)
        self.preview_timer.timeout.connect(self.preview_tick)
        outer=QtWidgets.QVBoxLayout(self);scroll=QtWidgets.QScrollArea();scroll.setWidgetResizable(True)
        body=QtWidgets.QWidget();layout=QtWidgets.QVBoxLayout(body);scroll.setWidget(body);outer.addWidget(scroll)
        row=QtWidgets.QHBoxLayout();layout.addLayout(row)
        self.side=QtWidgets.QComboBox();self.side.addItem('左臂','left');self.side.addItem('右臂','right')
        row.addWidget(QtWidgets.QLabel('执行手臂'));row.addWidget(self.side);self.controls.append(self.side)
        self.side.currentIndexChanged.connect(self.switch_side)
        self.button(row,'新建',self.new,QtWidgets.QStyle.SP_FileIcon)
        self.button(row,'导入',self.load,QtWidgets.QStyle.SP_DialogOpenButton)
        self.button(row,'导出',self.save,QtWidgets.QStyle.SP_DialogSaveButton)
        self.file=QtWidgets.QLabel();self.file.setWordWrap(True);layout.addWidget(self.file)
        row=QtWidgets.QHBoxLayout();layout.addLayout(row)
        self.button(row,'进入手臂导纳',lambda:self.teach(False))
        self.button(row,'进入手臂＋腰导纳',lambda:self.teach(True))
        self.button(layout,'退出导纳，原地保持',self.hold)
        row=QtWidgets.QHBoxLayout();layout.addLayout(row)
        self.button(row,'伸出食指和中指',self.index_middle)
        self.button(row,'记录准备位姿',self.ready,QtWidgets.QStyle.SP_DialogSaveButton)
        self.ready_status=QtWidgets.QLabel();layout.addWidget(self.ready_status)
        self.table=QtWidgets.QTableWidget(len(SLOTS),4)
        self.table.setHorizontalHeaderLabels(['录制槽','时长','样本','版本 / 时间'])
        self.table.verticalHeader().hide();self.table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        self.table.setSelectionMode(QtWidgets.QAbstractItemView.SingleSelection)
        self.table.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.Stretch)
        self.table.setFixedHeight(355)
        for i,label in enumerate(SLOTS.values()):
            for j,value in enumerate((label,'未录制','-','-')):self.table.setItem(i,j,QtWidgets.QTableWidgetItem(value))
        self.table.selectRow(0);layout.addWidget(self.table);self.controls.append(self.table)
        self.table.itemSelectionChanged.connect(self.selected)
        self.table.cellClicked.connect(lambda *_:self.start_preview())
        from task_trace_editor_panel import TraceEditorPanel
        self.editor=TraceEditorPanel(window,self.control,self.slot,timeline=False)
        layout.addWidget(self.editor)
        row=QtWidgets.QHBoxLayout();layout.addLayout(row)
        self.play_preview=QtWidgets.QPushButton();self.play_preview.setToolTip('播放 / 暂停录制时间轴')
        self.play_preview.setIcon(self.style().standardIcon(QtWidgets.QStyle.SP_MediaPlay))
        self.play_preview.clicked.connect(self.toggle_preview);row.addWidget(self.play_preview);self.controls.append(self.play_preview)
        self.timeline=QtWidgets.QSlider(QtCore.Qt.Horizontal);self.timeline.setRange(0,0)
        self.timeline.valueChanged.connect(self.seek_preview);row.addWidget(self.timeline);self.controls.append(self.timeline)
        self.timestamp=QtWidgets.QLabel('0.000 / 0.000 s');self.timestamp.setMinimumWidth(160);row.addWidget(self.timestamp)
        form=QtWidgets.QFormLayout();layout.addLayout(form)
        self.cut_start=QtWidgets.QDoubleSpinBox();self.cut_end=QtWidgets.QDoubleSpinBox()
        self.bridge=FreeDoubleSpinBox(.6,positive=True)
        for label,control in (('选区起点 (s)',self.cut_start),('选区终点 (s)',self.cut_end),('衔接时长 (s)',self.bridge)):
            if control is not self.bridge:control.setDecimals(3)
            control.setSingleStep(.1);form.addRow(label,control);self.controls.append(control)
        row=QtWidgets.QHBoxLayout();layout.addLayout(row)
        self.button(row,'设为起点',lambda:self.cut_start.setValue(self.preview_time),hide_preview=False)
        self.button(row,'设为终点',lambda:self.cut_end.setValue(self.preview_time),hide_preview=False)
        self.button(row,'删除区间并衔接',self.trim,QtWidgets.QStyle.SP_DialogDiscardButton)
        self.undo_trim=self.button(row,'返回蓝本',lambda:self.control.edit_version(self.slot(),'select',index=0),QtWidgets.QStyle.SP_ArrowBack)
        self.editor.external_range=lambda:(self.cut_start.value(),self.cut_end.value())
        self.editor.external_position=lambda:self.preview_time
        self.editor.external_pause=self.stop_preview
        self.editor.external_seek=lambda value:self.timeline.setValue(value)
        self.cut_start.valueChanged.connect(self.editor.schedule_markers)
        self.cut_end.valueChanged.connect(self.editor.schedule_markers)
        row=QtWidgets.QHBoxLayout();layout.addLayout(row)
        self.start=self.button(row,'开始录制',self.record,QtWidgets.QStyle.SP_MediaPlay)
        self.finish=self.button(row,'结束保存',self.control.finish_trace,QtWidgets.QStyle.SP_DialogSaveButton)
        self.cancel=self.button(row,'取消录制',self.control.cancel_trace,QtWidgets.QStyle.SP_DialogCancelButton)
        self.speed=SpeedInput();self.speed.setToolTip('相对于录入时间的倍率；衔接和返回段单独限速，手指保持')
        self.mode=QtWidgets.QComboBox();self.mode.addItem('完整循环：准备位 → 录入点 → 轨迹 → 准备位',True)
        self.mode.addItem('仅录制段（含起点衔接）',False)
        layout.addWidget(self.mode);self.controls.append(self.mode);self.mode.currentIndexChanged.connect(self.changed)
        self.velocity=QtWidgets.QDoubleSpinBox();self.acceleration=QtWidgets.QDoubleSpinBox()
        for control in (self.velocity,self.acceleration):control.setRange(.05,1.);control.setValue(.7);control.setSingleStep(.05)
        for a,b in ((self.velocity,self.editor.fast_velocity),(self.acceleration,self.editor.fast_acceleration)):
            a.valueChanged.connect(b.setValue);b.valueChanged.connect(a.setValue)
        form=QtWidgets.QFormLayout();layout.addLayout(form)
        for label,control in (('录制回放倍率',self.speed),('Velocity limit',self.velocity),('Acceleration limit',self.acceleration)):
            form.addRow(label,control);self.controls.append(control);control.valueChanged.connect(self.changed)
        self.confirm=QtWidgets.QCheckBox('工位与站位固定；外部控制已停止；衔接及回放路径净空')
        self.confirm.setToolTip('起点衔接不做碰撞检测；执行前核对完整 RViz 预览')
        layout.addWidget(self.confirm);self.controls.append(self.confirm)
        row=QtWidgets.QHBoxLayout();layout.addLayout(row)
        self.button(row,'生成回放 / RViz',lambda:self.prepare(self.slot()),QtWidgets.QStyle.SP_MediaPlay)
        self.button(row,'重新播放 RViz',self.control.replay_preview,QtWidgets.QStyle.SP_MediaPlay)
        from task_recorded_history_panel import RecordedHistoryPanel
        self.history=RecordedHistoryPanel(window,self.control);layout.addWidget(self.history)
        from task_jetson_push_panel import JetsonPushPanel
        self.jetson_push=JetsonPushPanel(window,'dring_demo');layout.addWidget(self.jetson_push)
        row=QtWidgets.QHBoxLayout();layout.addLayout(row)
        self.button(row,'移动到录入准备点',lambda:self.move('ready'))
        self.button(row,'返回桌上方准备位',lambda:self.move('preparation'))
        self.status=QtWidgets.QLabel();self.status.setWordWrap(True);layout.addWidget(self.status)
        layout.addStretch(1)

    def button(self,layout,text,action,icon=None,*,hide_preview=True):
        b=QtWidgets.QPushButton(text)
        if icon is not None:b.setIcon(self.style().standardIcon(icon))
        def invoke():
            self.stop_preview(hide=hide_preview);self.window.invoke(action)
        b.clicked.connect(invoke);layout.addWidget(b);self.controls.append(b);return b

    def selected(self,*_):
        self.changed();self.start_preview()

    def start_preview(self):
        self.stop_preview()
        n=self.window.node
        if (self.control.recording is not None or (n.worker is not None and n.worker.is_alive()) or
                n.session.phase in ('PLANNING','EXECUTION_PREFLIGHT','EXECUTING')):return
        trace=self.control.config['traces'].get(self.slot())
        self.preview_trace=trace
        if trace is None:
            self.control.hide_preview()
            self.timeline.setRange(0,0);self.timestamp.setText('未录制');return
        self.preview_times=np.asarray(trace['raw']['time_s'])
        self.preview_positions=np.asarray(trace['raw']['positions'])
        duration=float(self.preview_times[-1]);self.preview_time=0.
        blocked=self.timeline.blockSignals(True);self.timeline.setRange(0,round(duration*1000));self.timeline.setValue(0)
        self.timeline.blockSignals(blocked)
        for spin in (self.cut_start,self.cut_end):spin.setRange(0.,duration)
        self.cut_start.setValue(0.);self.cut_end.setValue(0.)
        if hasattr(self,'editor'):self.editor.refresh()
        self.draw_preview()
        self.preview_origin=time.monotonic();self.preview_timer.start()
        self.play_preview.setIcon(self.style().standardIcon(QtWidgets.QStyle.SP_MediaPause))

    def stop_preview(self,hide=False):
        self.preview_timer.stop()
        if hide:self.control.hide_preview()
        if hasattr(self,'play_preview'):self.play_preview.setIcon(self.style().standardIcon(QtWidgets.QStyle.SP_MediaPlay))

    def toggle_preview(self):
        if self.preview_timer.isActive():self.stop_preview();return
        if self.preview_trace is None:self.start_preview();return
        if self.preview_time>=self.preview_times[-1]:self.preview_time=0.
        self.preview_origin=time.monotonic()-self.preview_time;self.preview_timer.start()
        self.play_preview.setIcon(self.style().standardIcon(QtWidgets.QStyle.SP_MediaPause))

    def draw_preview(self):
        if self.preview_trace is None:return
        q=[np.interp(self.preview_time,self.preview_times,self.preview_positions[:,i])
           for i in range(self.preview_positions.shape[1])]
        self.control.preview_pose(q)
        self.timestamp.setText(f'{self.preview_time:.3f} / {self.preview_times[-1]:.3f} s')
        blocked=self.timeline.blockSignals(True);self.timeline.setValue(round(self.preview_time*1000));self.timeline.blockSignals(blocked)

    def preview_tick(self):
        if not self.isVisible():self.stop_preview(hide=True);return
        self.preview_time=min(time.monotonic()-self.preview_origin,float(self.preview_times[-1]))
        self.draw_preview()
        if self.preview_time>=self.preview_times[-1]:self.stop_preview()

    def seek_preview(self,value):
        if self.preview_trace is None:return
        self.stop_preview();self.preview_time=min(value*.001,float(self.preview_times[-1]));self.draw_preview()

    def trim(self):
        duration=self.control.config['traces'][self.slot()]['raw']['time_s'][-1]
        # Millisecond display rounding must still select the exact final sample.
        def timestamp(spin):return duration if spin.value()==spin.maximum() else spin.value()
        self.control.edit_trace(self.slot(),timestamp(self.cut_start),timestamp(self.cut_end),self.bridge.value())

    def hideEvent(self,event):
        self.stop_preview(hide=True);super().hideEvent(event)

    def showEvent(self,event):
        super().showEvent(event)
        QtCore.QTimer.singleShot(0,self.start_preview)

    def ask(self,title,text):
        return QtWidgets.QMessageBox.question(self,title,text,QtWidgets.QMessageBox.Yes|QtWidgets.QMessageBox.No,
            QtWidgets.QMessageBox.No)==QtWidgets.QMessageBox.Yes

    def slot(self):return tuple(SLOTS)[max(0,self.table.currentRow())]

    def switch_side(self,*_):
        try:self.window.invoke(lambda:self.control.select_side(self.side.currentData()))
        finally:
            blocked=self.side.blockSignals(True)
            self.side.setCurrentIndex(self.side.findData(self.control.config['side']))
            self.side.blockSignals(blocked)

    def new(self):
        if self.ask('新建示教','清空当前准备位与十个录制槽？已保存的原始文件不删除。'):
            self.control.new(self.side.currentData())

    def load(self):
        path,_=QtWidgets.QFileDialog.getOpenFileName(self,'导入拉环示教',str(self.control.store.parent),'JSON (*.json)')
        if path:self.control.load(path)

    def save(self):
        path,_=QtWidgets.QFileDialog.getSaveFileName(self,'导出拉环示教',self.control.path or
            str(self.window.node.run_dir/'dring_demonstration.json'),'JSON (*.json)')
        if path:self.control.save(path)

    def teach(self,waist):
        self.control.idle();self.control.gateway_guard();self.window.node.session.measured(stationary=True)
        if self.ask('进入导纳','确认机器人有可靠支撑，当前静止且可手动拖动？'):
            self.window.node.stop(include_waist=waist)

    def hold(self):
        self.control.idle();self.control.gateway_guard(False)
        if self.ask('恢复保持','确认已停稳？退出导纳后仅保持当前位置，不自动回准备位。'):
            self.window.node.recovery(.15,.15)

    def ready(self):
        if self.side.currentData()!=self.control.config['side']:raise RuntimeError('更换执行手臂请先点新建')
        if self.ask('记录准备位','保存当前静止位姿？覆盖准备位会清空当前十个录制槽，旧原始文件保留。'):
            self.control.record_ready(False)

    def index_middle(self):
        if self.side.currentData()!=self.control.config['side']:raise RuntimeError('更换执行手臂请先点新建')
        if self.ask('伸出食指和中指','选中手的食指和中指同时伸直，其余手指收拢。确认手指周围净空？手臂不会移动。'):
            self.control.index_middle()

    def record(self):
        if self.side.currentData()!=self.control.config['side']:raise RuntimeError('更换执行手臂请先点新建')
        if self.ask('录制'+SLOTS[self.slot()], '从准备位开始；等待显示录制中后拖动，结束前停稳。保存成功才覆盖该段。'):
            self.control.start_trace(self.slot(),False)

    def prepare(self,operation):
        if self.side.currentData()!=self.control.config['side']:raise RuntimeError('更换执行手臂请先点新建')
        self.control.prepare(operation,self.speed.value(),self.velocity.value(),self.acceleration.value(),self.confirm.isChecked(),
                             full_cycle=operation in SLOTS and self.mode.currentData())
        self.confirm.setChecked(False);self.window.clear.setChecked(False)

    def move(self,operation):
        label='录入准备点' if operation=='ready' else '桌上方准备位'
        if not self.ask('移动到'+label,'确认从当前位置直接移动到'+label+'？不做碰撞检测，请确认完整路径净空。'):
            return
        self.control.prepare(operation,1.,self.velocity.value(),self.acceleration.value(),True)
        self.pending_move=(self.window.node.session.generation,operation)
        self.window.clear.setChecked(False)

    def changed(self,*_):
        n=self.window.node
        with n.lock:
            if (n.session.plan and n.session.plan.get('kind')=='dring_demo' and
                    n.session.plan['demo'].get('workflow','dring_demo')==self.control.workflow):
                if n.session.phase in ('EXECUTION_PREFLIGHT','EXECUTING'):return
                n.session.invalidate();n.session.phase='REPLAN_REQUIRED';self.window.clear.setChecked(False)

    def refresh(self,busy):
        if busy or self.control.recording is not None:self.stop_preview(hide=True)
        if self.pending_move is not None:
            n=self.window.node;generation,operation=self.pending_move
            if n.session.generation!=generation:self.pending_move=None
            elif not busy:
                self.pending_move=None
                plan=n.session.plan
                if (n.session.phase=='PLAN_READY' and plan and plan.get('kind')=='dring_demo' and
                        plan['demo'].get('workflow','dring_demo')==self.control.workflow and
                        plan['demo']['operation']==operation):
                    self.window.invoke(lambda:n.execute(scene_confirmed=True))
                    busy=True
        c=self.control;recording=c.recording
        for control in self.controls:
            if control not in (self.finish,self.cancel):
                control.setEnabled(not busy and recording is None)
        self.finish.setEnabled(recording is not None and recording['phase']=='recording' and not recording['finish'].is_set())
        self.cancel.setEnabled(recording is not None and not recording['cancel'].is_set())
        if self.revision!=c.revision:
            self.revision=c.revision
            blocked=self.side.blockSignals(True);self.side.setCurrentIndex(self.side.findData(c.config['side']))
            self.side.blockSignals(blocked)
            self.ready_status.setText('准备位：已录入' if c.config['ready'] else '准备位：未录入')
            for i,slot in enumerate(SLOTS):
                self.table.setRowHidden(i,slot not in c.config.get('retained_slots',SLOTS))
                trace=c.config['traces'].get(slot)
                recorded=trace.get('recorded_at','') if trace else ''
                try:stamp=datetime.fromisoformat(recorded).strftime('%m-%d %H:%M')
                except ValueError:stamp='-'
                if trace and trace.get('optimization'):stamp=trace['optimization']['label']
                if trace and trace.get('trim_edit'):stamp=trace['trim_edit']['label']
                values=(f'{trace["raw"]["time_s"][-1]:.2f} s',str(len(trace['raw']['positions'])),
                        stamp) if trace else ('未录制','-','-')
                for j,value in enumerate(values,1):self.table.item(i,j).setText(value)
                self.table.item(i,3).setToolTip(recorded)
            if self.table.isRowHidden(self.table.currentRow()):
                self.table.selectRow(next(i for i in range(len(SLOTS)) if not self.table.isRowHidden(i)))
            visible=sum(not self.table.isRowHidden(i) for i in range(len(SLOTS)))
            self.table.setFixedHeight(self.table.horizontalHeader().height()+visible*self.table.verticalHeader().defaultSectionSize()+4)
            if self.isVisible() and not busy and recording is None:self.start_preview()
        trace=c.config['traces'].get(self.slot(),{})
        if self.isVisible() and not busy and recording is None and trace and trace is not self.preview_trace:
            self.start_preview()
        from task_trace_versions import bank
        self.undo_trim.setEnabled(not busy and recording is None and bool(trace) and bank(c.config,self.slot())['selected']!=0)
        self.editor.refresh(busy)
        self.history.refresh(busy)
        self.file.setText(c.path or str(c.store))
        text=c.status
        if recording and recording.get('receiver'):
            latest=recording['receiver'].latest
            text+=f' | {latest.get("seconds",0.):.1f} s / {latest.get("count",0)} 点'
        plan=self.window.node.session.plan
        if plan and plan.get('kind')=='dring_demo' and plan['demo'].get('workflow','dring_demo')==c.workflow:
            text+='\n'+' | '.join(f'{s["label"]} {s["duration"]:.2f}s' for s in plan['segments'])
            values=plan.get('planning_config',{}).get('values',{})
            if values.get('stage_control_enabled') and plan['demo']['operation'] in SLOTS:
                text+=f'\n复现肩肘 Kp/Kd {values["withdraw_proximal_kp"]:g}/{values["withdraw_proximal_kd"]:g}'
                text+=f'；腰 Kp/Kd {values["stage_waist_kp"]:g}/{values["stage_waist_kd"]:g}'
            speed=plan.get('speed_scaling',{})
            if speed.get('timing_comparison'):
                from task_cruise_timing import summary
                timing_text=summary(speed)
                if timing_text not in text:text+='\n'+timing_text
            if speed.get('replay_cache',{}).get('hit'):text+='\n录制段：已读取验证缓存'
            for segment in plan['segments']:
                if segment.get('optimization'):text+='\n'+segment['optimization']['label']+'；未实机验证'
            text+=f'\n请求 {speed.get("requested_multiplier",1.):.2f}x / 平均实际 {speed.get("applied_multiplier",1.):.2f}x'
            if speed.get('timing_policy')=='recorded_local_v1':
                text+=f' / 局部倍率 {speed["rate_min"]:.2f}–{speed["rate_max"]:.2f}x'
                if any(s.get('method')=='uniform_fallback' for s in speed.get('stage_limits',[])):
                    text+=' / 局部调速未通过，已回退整体限速'
            maximum=speed.get('maximum_multiplier')
            if maximum is not None:text+=f' / 允许上限 {maximum:.2f}x'
            text+='\n衔接碰撞检测：未执行'
        self.status.setText(text)
        self.jetson_push.refresh(busy)
