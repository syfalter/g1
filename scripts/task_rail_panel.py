"""Rail teaching tab in the existing Qt task console."""
import time

from PyQt5 import QtWidgets

from task_hand_control import ORDER
from task_rail_trace import STAGES, trace_hand_matches
from task_rail_trace import ENDPOINTS, endpoint
from task_rail_motion import LINK, VELOCITY_SCALE, ACCELERATION_SCALE
from task_planning_config import DEFAULT_CONFIG, PlanningParameters, load_parameters

HAND_LABELS = {'open':'张开', 'grip':'推进握持', 'pull_grip':'拉出握持'}


class RailPanel(QtWidgets.QWidget):
    def __init__(self, window):
        super().__init__()
        self.window, self.control = window, window.node.rail
        self.controls = []
        self.revision = None
        self.pending_move = None
        self.pending_replay = None
        self.move_buttons = {}
        outer = QtWidgets.QVBoxLayout(self)
        scroll = QtWidgets.QScrollArea(); scroll.setWidgetResizable(True)
        content = QtWidgets.QWidget(); layout = QtWidgets.QVBoxLayout(content)
        scroll.setWidget(content); outer.addWidget(scroll)
        row = QtWidgets.QHBoxLayout(); layout.addLayout(row)
        self.button(row, '新建滑轨示教', self.new)
        self.button(row, '加载', self.load)
        self.button(row, '导出', self.save)
        self.file_label = QtWidgets.QLabel(); self.file_label.setWordWrap(True); layout.addWidget(self.file_label)
        row = QtWidgets.QHBoxLayout(); layout.addLayout(row)
        self.button(row, '进入手臂导纳', self.teach)
        self.button(row, '退出导纳，原地保持', self.hold)
        self.button(layout, '进入双臂＋腰 yaw 导纳', self.teach_with_waist)
        self.waist_status = QtWidgets.QLabel(); layout.addWidget(self.waist_status)
        self.move_buttons['preparation'] = self.button(layout,'移动到右手 above-table',lambda:self.move('preparation'))

        self.hand_table = QtWidgets.QTableWidget(6, 5)
        self.hand_table.setHorizontalHeaderLabels(['通道', '实测', '张开', '推进', '拉出'])
        self.hand_table.verticalHeader().hide()
        self.hand_table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.hand_table.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.Stretch)
        self.hand_table.setMinimumHeight(215)
        for i, name in enumerate(('小指', '无名指', '中指', '食指', '拇指 2', '拇指 1')):
            for j, text in enumerate((name, '-', '-', '-', '-')):
                self.hand_table.setItem(i, j, QtWidgets.QTableWidgetItem(text))
            self.hand_table.item(i, 0).setToolTip(ORDER[i])
        layout.addWidget(self.hand_table)
        row = QtWidgets.QHBoxLayout(); layout.addLayout(row)
        self.button(row, '读取右手反馈', lambda:self.control.record())
        self.button(row, '覆盖保存张开手型', lambda:self.record_hand('open'))
        self.button(row, '覆盖保存推进手型', lambda:self.record_hand('grip'))
        self.button(layout, '覆盖保存拉出手型', lambda:self.record_hand('pull_grip'))
        self.hand_status = QtWidgets.QLabel(); self.hand_status.setWordWrap(True); layout.addWidget(self.hand_status)
        row = QtWidgets.QHBoxLayout(); layout.addLayout(row)
        self.open_hand_button = self.button(row, '右手张开', lambda:self.apply_hand('open'))
        self.grip_hand_button = self.button(row, '推进手型', lambda:self.apply_hand('grip'))
        self.pull_hand_button = self.button(row, '拉出手型', lambda:self.apply_hand('pull_grip'))
        self.open_hand_button.setToolTip('切换到本页已保存的张开手型，仅控制右手手指')
        self.grip_hand_button.setToolTip('切换到本页已保存的握持手型，仅控制右手手指')
        self.pull_hand_button.setToolTip('切换到本页已保存的拉出手型，仅控制右手手指')

        form = QtWidgets.QFormLayout(); layout.addLayout(form)
        self.path_source = QtWidgets.QComboBox()
        self.path_source.addItem('记录轨迹重放（关节插补）', 'recorded')
        self.path_source.addItem('旧端点规划（腕部直线）', 'endpoints')
        form.addRow('路径来源', self.path_source); self.controls.append(self.path_source)
        self.path_source.currentIndexChanged.connect(self.selection_changed)
        page_layout = layout
        self.trace_section = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(self.trace_section); layout.setContentsMargins(0,0,0,0)
        page_layout.addWidget(self.trace_section)
        form = QtWidgets.QFormLayout(); layout.addLayout(form)
        self.trace_stage = QtWidgets.QComboBox()
        for key,(label,_) in STAGES.items():
            self.trace_stage.addItem(label,key)
        form.addRow('录制段',self.trace_stage); self.controls.append(self.trace_stage)
        self.trace_table = QtWidgets.QTableWidget(len(STAGES),4)
        self.trace_table.setHorizontalHeaderLabels(['轨迹段','原始 / 取点','长度 mm','状态'])
        self.trace_table.verticalHeader().hide()
        self.trace_table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.trace_table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        self.trace_table.setSelectionMode(QtWidgets.QAbstractItemView.SingleSelection)
        self.trace_table.horizontalHeader().setSectionResizeMode(0,QtWidgets.QHeaderView.Stretch)
        for col in (1,2,3):self.trace_table.setColumnWidth(col,95)
        self.trace_table.setMinimumHeight(215)
        for i,(_, (label,_)) in enumerate(STAGES.items()):
            for j,text in enumerate((label,'-','-','未录制')):
                self.trace_table.setItem(i,j,QtWidgets.QTableWidgetItem(text))
        self.trace_table.cellClicked.connect(lambda row,col:self.trace_stage.setCurrentIndex(row))
        self.trace_stage.currentIndexChanged.connect(lambda row:self.trace_table.selectRow(row))
        self.trace_table.selectRow(0); layout.addWidget(self.trace_table)
        from task_trace_editor_panel import TraceEditorPanel
        self.editor=TraceEditorPanel(window,self.control,self.trace_stage.currentData)
        layout.addWidget(self.editor)
        from task_recorded_history_panel import RecordedHistoryPanel
        self.history=RecordedHistoryPanel(window,self.control);layout.addWidget(self.history)
        self.trace_stage.currentIndexChanged.connect(lambda:self.editor.refresh())
        row = QtWidgets.QHBoxLayout(); layout.addLayout(row)
        self.start_trace_button = self.button(row,'开始录制选中段',self.start_trace)
        self.finish_trace_button = self.button(row,'结束并保存本段',self.control.finish_trace)
        self.cancel_trace_button = self.button(row,'取消本次录制',self.control.cancel_trace)
        row = QtWidgets.QHBoxLayout(); layout.addLayout(row)
        self.button(row,'检查 / 预览选中段',lambda:self.control.inspect_trace(self.trace_stage.currentData()))
        self.button(row,'删除选中轨迹段',self.delete_trace)
        self.trace_status = QtWidgets.QLabel(); self.trace_status.setWordWrap(True); layout.addWidget(self.trace_status)
        self.trace_alignment = QtWidgets.QLabel(); self.trace_alignment.setWordWrap(True); layout.addWidget(self.trace_alignment)
        self.recorded_points = QtWidgets.QTableWidget(len(ENDPOINTS),3)
        self.recorded_points.setHorizontalHeaderLabels(['端点','末帧来源','腕部 XYZ (m)'])
        self.recorded_points.verticalHeader().hide()
        self.recorded_points.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.recorded_points.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.Stretch)
        self.recorded_points.setFixedHeight(155)
        for i,(_, (label,stage)) in enumerate(ENDPOINTS.items()):
            for j,text in enumerate((label,STAGES[stage][0].split('：')[0],'未录制')):
                self.recorded_points.setItem(i,j,QtWidgets.QTableWidgetItem(text))
        layout.addWidget(self.recorded_points)
        for keys in (('push_A','push_B'),('pull_B','pull_A')):
            row = QtWidgets.QHBoxLayout(); layout.addLayout(row)
            for key in keys:
                self.move_buttons[key] = self.button(row,'移动到'+ENDPOINTS[key][0],lambda key=key:self.move(key))
        row = QtWidgets.QHBoxLayout(); layout.addLayout(row)
        self.move_buttons['connect'] = self.button(row,'移动到本段衔接起点',lambda:self.move('connect:'+self.trace_stage.currentData()))
        self.move_buttons['start'] = self.button(row,'移动到本段已录起点',lambda:self.move('start:'+self.trace_stage.currentData()))

        self.endpoint_section = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(self.endpoint_section); layout.setContentsMargins(0,0,0,0)
        page_layout.addWidget(self.endpoint_section)
        form = QtWidgets.QFormLayout(); layout.addLayout(form)
        self.slot = QtWidgets.QComboBox()
        self.slot.addItem('外端 A', 'A'); self.slot.addItem('内端 B', 'B')
        form.addRow('示教端', self.slot); self.controls.append(self.slot)
        row = QtWidgets.QHBoxLayout(); layout.addLayout(row)
        self.button(row, '追加接近关键点', lambda:self.record_point(True))
        self.button(row, '记录抓握端点', lambda:self.record_point(False))
        self.points = QtWidgets.QTableWidget(0, 3)
        self.points.setHorizontalHeaderLabels(['端 / 路径点', '记录时间', '腕部 XYZ (m)'])
        self.points.horizontalHeader().setSectionResizeMode(2, QtWidgets.QHeaderView.Stretch)
        self.points.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.points.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        self.points.setSelectionMode(QtWidgets.QAbstractItemView.SingleSelection)
        self.points.setMinimumHeight(180); layout.addWidget(self.points)
        self.point_keys = []
        self.button(layout, '删除选中示教点', self.remove)

        layout = page_layout
        form = QtWidgets.QFormLayout(); layout.addLayout(form)
        self.mode = QtWidgets.QComboBox()
        self.mode.addItem('完整三段循环，返回准备位后张手', 'cycle')
        self.mode.addItem('推入后保持握持', 'hold')
        self.mode.addItem('推入后松手回准备位', 'release')
        self.mode.currentIndexChanged.connect(self.selection_changed)
        form.addRow('操作模式', self.mode); self.controls.append(self.mode)
        self.speed_section = QtWidgets.QWidget()
        speed_form = QtWidgets.QFormLayout(self.speed_section)
        speed_form.setContentsMargins(0,0,0,0)
        try:defaults,_ = load_parameters(getattr(window.node.args,'planning_config',DEFAULT_CONFIG))
        except (OSError,ValueError,TypeError):defaults = PlanningParameters()
        self.replay_speed = QtWidgets.QDoubleSpinBox()
        self.replay_speed.setRange(.1,10.);self.replay_speed.setDecimals(1)
        self.replay_speed.setSingleStep(.1);self.replay_speed.setSuffix(' x')
        self.replay_speed.setValue(defaults.rail_replay_multiplier)
        self.replay_speed.setToolTip('录制轨迹的请求倍速；实际受速度、加速度及 jerk 限制。修改后需重新规划。')
        self.replay_fastest = QtWidgets.QCheckBox('已验证限速内最快')
        self.replay_fastest.setChecked(defaults.rail_replay_fastest)
        self.replay_fastest.setToolTip('忽略请求倍速，使用局部优化的已验证最快结果，不代表机械极限。')
        self.trim_idle_ends = QtWidgets.QCheckBox('裁减首尾静止 / 微动冗余')
        self.trim_idle_ends.setChecked(defaults.rail_trim_idle_ends)
        self.trim_idle_ends.setToolTip('首尾同时满足 0.3 mm、0.3 度、0.003 rad 的微动范围才处理。'
            '仅修改规划副本，原始录制和首末端点不变；删点影响贴合时保留取点。修改后需重新规划。')
        self.trim_result = QtWidgets.QLabel('首尾裁减：尚未规划')
        self.trim_result.setWordWrap(True)
        speed_form.addRow('回放倍速',self.replay_speed)
        speed_form.addRow(self.replay_fastest)
        speed_form.addRow(self.trim_idle_ends)
        speed_form.addRow(self.trim_result)
        layout.addWidget(self.speed_section)
        self.controls.extend((self.replay_speed,self.replay_fastest,self.trim_idle_ends))
        self.replay_speed.valueChanged.connect(self.selection_changed)
        self.replay_fastest.toggled.connect(self.selection_changed)
        self.trim_idle_ends.toggled.connect(self.selection_changed)
        self.preview_cycle_button = self.button(layout, 'RViz 预览完整推入＋拉出（不执行）',
            lambda:self.control.preview_cycle(self.mode.currentData()))
        self.state = QtWidgets.QComboBox()
        for label, value in (('外端', 'outer'), ('内端，仍握持', 'inner_held'), ('内端，已松手', 'inner_released')):
            self.state.addItem(label, value)
        self.state_prompt = QtWidgets.QLabel('人工核对滑杆状态')
        form.addRow(self.state_prompt, self.state); self.controls.append(self.state)
        self.confirm_state_button = self.button(layout, '确认滑杆当前状态', self.confirm_state)
        self.state_label = QtWidgets.QLabel(); self.state_label.setWordWrap(True); layout.addWidget(self.state_label)
        self.confirm = QtWidgets.QCheckBox('Web 停控；工位固定；完整路径净空；松手后滑杆稳定')
        self.confirm.setToolTip('完整路径包含段间补接和手型切换；衔接不做碰撞检测。每次规划前重新确认；本面板不能锁定外部 Web 控制源')
        layout.addWidget(self.confirm); self.controls.append(self.confirm)
        row = QtWidgets.QHBoxLayout(); layout.addLayout(row)
        self.button(row, '规划推入', lambda:self.plan('push'))
        self.button(row, '规划拉出', lambda:self.plan('pull'))
        from task_jetson_push_panel import JetsonPushPanel
        self.jetson_push = JetsonPushPanel(window, 'rail'); layout.addWidget(self.jetson_push)
        row = QtWidgets.QHBoxLayout(); layout.addLayout(row)
        self.reuse_push = self.button(row, '复用缓存并执行推入', lambda:self.replay_cached('push'))
        self.reuse_pull = self.button(row, '复用缓存并执行拉出', lambda:self.replay_cached('pull'))
        self.status = QtWidgets.QLabel(); self.status.setWordWrap(True); layout.addWidget(self.status)
        layout.addStretch(1)
        self.update_sections()

    def button(self, layout, text, action):
        button = QtWidgets.QPushButton(text)
        button.setToolTip(text)
        button.clicked.connect(lambda:self.window.invoke(action))
        layout.addWidget(button); self.controls.append(button)
        return button

    def ask(self, title, text):
        return QtWidgets.QMessageBox.question(self, title, text,
            QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No, QtWidgets.QMessageBox.No) == QtWidgets.QMessageBox.Yes

    def new(self):
        if self.ask('新建', '清空轨迹和端点？当前张开/握持手型保留，已有导出文件不删除。'):
            self.control.new()

    def load(self):
        path, _ = QtWidgets.QFileDialog.getOpenFileName(self, '加载滑轨示教',
            str(self.window.node.run_dir.parent), 'JSON (*.json)')
        if path and self.ask('加载', '替换当前滑轨示教？滑杆状态需重新确认。'):
            self.control.load(path)

    def save(self):
        path, _ = QtWidgets.QFileDialog.getSaveFileName(self, '导出滑轨示教',
            self.control.path or str(self.window.node.run_dir/'rail_workcell.json'), 'JSON (*.json)')
        if path:
            self.control.save(path)

    def teach(self):
        self.control.idle()
        self.control.gateway_guard()
        self.window.node.session.measured(stationary=True)
        if self.ask('手臂导纳', '确认机器人静止且有固定支撑？腰保持不动，仅手臂进入导纳。'):
            self.window.node.stop()

    def hold(self):
        self.control.idle()
        self.control.gateway_guard(False)
        if self.ask('恢复保持', '确认机器人静止且准备恢复当前姿态保持？不会自动回准备位。'):
            self.window.node.recovery(VELOCITY_SCALE, ACCELERATION_SCALE)

    def teach_with_waist(self):
        self.control.idle()
        self.control.gateway_guard()
        self.window.node.session.measured(stationary=True)
        if self.ask('双臂＋腰 yaw 导纳',
                    '确认机器人静止且支撑稳固？将允许双臂及腰 yaw 手动拖动，腰 roll/pitch 仍保持。\n'
                    '录制只记录右臂和腰 yaw，左臂关节保持不动。等待腰导纳显示已启用后再拖动。'):
            self.window.node.stop(include_waist=True)

    def record_hand(self, name):
        if self.ask('覆盖保存手型', '将当前实测手型覆盖保存为本页及以后启动的默认手型？\n'
                    '已有轨迹需重新核对。确认手指静止；只读取保存，不发送动作。'):
            self.control.record(hand=name)

    def apply_hand(self, name):
        label = HAND_LABELS[name]
        if self.ask('右手'+label,
                    '确认 Web 已锁定或关闭、手臂停稳，右手动作范围内无人手？\n'
                    '将实际切换到本页保存的'+label+'手型，不移动手臂。\n'
                    '旧计划失效，滑杆状态需重新确认。'):
            self.control.apply_hand(name, confirmed=True)

    def record_point(self, waypoint):
        slot = self.slot.currentData()
        text = ('按准备位到抓握端的顺序追加接近关键点，退出时逆序回放。确认当前姿态静止？' if waypoint else
                '确认当前为实际握住杆部的端点，未继续顶压机械限位？将覆盖该端点。')
        if self.ask('记录 '+slot, text):
            self.control.record(slot, waypoint=waypoint)

    def remove(self):
        row = self.points.currentRow()
        if row >= 0 and self.ask('删除', '删除选中的示教点？旧计划失效。'):
            self.control.remove(*self.point_keys[row])

    def selection_changed(self):
        self.pending_replay = None
        n = self.window.node
        with n.lock:
            if n.session.plan and n.session.plan.get('kind') == 'rail':
                n.session.invalidate(); n.session.phase = 'REPLAN_REQUIRED'
        if hasattr(self,'confirm'):self.confirm.setChecked(False)
        self.update_sections()

    def update_sections(self):
        if hasattr(self,'endpoint_section'):
            recorded = self.path_source.currentData() == 'recorded'
            self.trace_section.setVisible(recorded)
            self.endpoint_section.setVisible(not recorded)
            if hasattr(self,'speed_section'):
                self.speed_section.setVisible(recorded)
                self.replay_speed.setEnabled(not self.replay_fastest.isChecked())
            if hasattr(self,'confirm'):
                for widget in (self.state_prompt,self.state,self.confirm_state_button,self.state_label,self.confirm):
                    widget.setVisible(not recorded)
            if hasattr(self,'mode'):
                self.mode.model().item(0).setEnabled(recorded)
                for index in (1,2):self.mode.model().item(index).setEnabled(not recorded)
                if recorded and self.mode.currentData() != 'cycle':self.mode.setCurrentIndex(0)
                if not recorded and self.mode.currentData() == 'cycle':self.mode.setCurrentIndex(2)

    def start_trace(self):
        stage = self.trace_stage.currentData()
        overwrite = '\n保存成功后覆盖该段旧轨迹；取消或失败保留旧轨迹。' if stage in self.control.config.get('traces',{}) else ''
        if self.ask('开始录制','确认外部控制已停止、机器人静止？从当前位置开始，只记录右臂与腰。\n'
                    '等待显示“录制中”后再慢速拖动，结束前停稳。'+overwrite):
            self.control.start_trace(stage,confirmed=True)

    def move(self, key):
        label = self.control.movement_label(key)
        if not self.ask('移动到'+label,'确认从当前位置移动到'+label+'？\n'
                '右手手型保持；不做碰撞检测，请确认含腰运动的完整路径净空。'):
            return
        self.control.prepare_move(key,confirmed=True)
        self.pending_move = (self.window.node.session.generation,key)

    def delete_trace(self):
        stage = self.trace_stage.currentData()
        if self.ask('删除轨迹段','删除 '+STAGES[stage][0]+'？保留手型、端点及其他轨迹段。'):
            self.control.delete_trace(stage)

    def confirm_state(self):
        if self.ask('核对滑杆', '确认滑杆实际位置和抓握状态与所选项一致？这不是传感器接触检测。'):
            self.control.confirm_state(self.state.currentData())

    def plan(self, operation):
        self.control.plan(operation, self.mode.currentData(), confirmed=self.confirm.isChecked(),
                          path_source=self.path_source.currentData(),**self.speed_options())
        self.confirm.setChecked(False)

    def speed_options(self):
        if self.path_source.currentData() != 'recorded':return {}
        return dict(replay_multiplier=self.replay_speed.value(),replay_fastest=self.replay_fastest.isChecked(),
            trim_idle_ends=self.trim_idle_ends.isChecked())

    def replay_cached(self, operation):
        if self.path_source.currentData() != 'recorded':raise RuntimeError('请选择记录轨迹重放')
        if not self.ask('复用回放', '确认当前位置经 above-table 到录制路径及返回路径均净空？将复用缓存并执行；缺失或失效时不执行。'):
            return
        n = self.window.node
        self.control.plan(operation,self.mode.currentData(),confirmed=True,path_source='recorded',reuse=True,**self.speed_options())
        self.pending_replay = (n.session.generation,operation)
        self.confirm.setChecked(False)

    def refresh(self, busy):
        self.editor.refresh(busy)
        self.history.refresh(busy)
        c, n = self.control, self.window.node
        if self.pending_replay is not None:
            generation,operation = self.pending_replay
            if n.session.generation != generation:self.pending_replay = None
            elif not busy:
                self.pending_replay = None
                plan = n.session.plan
                if (n.session.phase == 'PLAN_READY' and plan and plan.get('kind') == 'rail' and
                        plan['rail'].get('reuse') and plan['rail'].get('operation') == operation):
                    self.window.invoke(lambda:n.execute(scene_confirmed=True))
                    busy = True
        if self.pending_move is not None:
            generation,key = self.pending_move
            if n.session.generation != generation:self.pending_move = None
            elif not busy:
                self.pending_move = None
                plan = n.session.plan
                if (n.session.phase == 'PLAN_READY' and plan and plan.get('kind') == 'rail' and
                        plan['rail'].get('operation') == 'move' and plan['rail'].get('target') == key):
                    self.window.invoke(lambda:n.execute(scene_confirmed=True))
                    busy = True
        recording = c.recording
        self.jetson_push.refresh(busy)
        active = recording is not None
        plan = n.session.plan
        if plan and plan.get('kind') == 'rail' and plan.get('rail',{}).get('path_source') == 'recorded':
            reports = [(s['trace_stage'],s.get('trace_report',{}).get('idle_trim'))
                for s in plan['segments'] if 'trace_stage' in s]
            trimmed = [(stage,report) for stage,report in reports if report]
            seconds = sum(r['leading_seconds']+r['trailing_seconds'] for _,r in trimmed)
            knots = sum(r['removed_knots'] for _,r in trimmed)
            self.trim_result.setText(f'原录制首尾时长裁减：{seconds:.2f} 秒；拟合取点减少：{knots} 个')
            self.trim_result.setToolTip('不等同于最终回放节省的时间；原始录制和端点不变。\n'+'\n'.join(
                f'{STAGES[stage][0]}：首 {r["leading_seconds"]:.2f} 秒，尾 {r["trailing_seconds"]:.2f} 秒，减少 {r["removed_knots"]} 点'
                for stage,r in trimmed))
        else:
            self.trim_result.setText('首尾裁减：等待新计划' if busy else '首尾裁减：尚未规划')
            self.trim_result.setToolTip('')
        # Disabling a pressed Qt button cancels its click, even if re-enabled immediately.
        for widget in self.controls:
            if widget not in (self.finish_trace_button, self.cancel_trace_button):
                widget.setEnabled(not busy and not active)
        self.replay_speed.setEnabled(not busy and not active and not self.replay_fastest.isChecked())
        for button, name in ((self.open_hand_button, 'open'), (self.grip_hand_button, 'grip'),
                             (self.pull_hand_button, 'pull_grip')):
            button.setEnabled(not busy and not active and name in c.config['hands'] and
                              n.gateway.get('phase') in ('TASK_ADMITTANCE', 'ARMED_HOLD') and
                              n.session.phase not in ('PLANNING', 'EXECUTION_PREFLIGHT', 'EXECUTING'))
        self.finish_trace_button.setEnabled(active and recording['phase'] == 'recording' and not recording['finish'].is_set())
        self.cancel_trace_button.setEnabled(active and not recording['cancel'].is_set())
        self.trace_table.setEnabled(not busy and not active)
        self.points.setEnabled(not busy and not active)
        self.preview_cycle_button.setEnabled(not busy and not active and self.path_source.currentData() == 'recorded')
        for button in (self.reuse_push,self.reuse_pull):
            button.setEnabled(not busy and not active and self.path_source.currentData() == 'recorded')
        fresh_gateway = time.monotonic()-n.gateway_received <= 1.5
        for key,button in self.move_buttons.items():
            target = key+':'+self.trace_stage.currentData() if key in ('start','connect') else key
            try:c.movement_target(target,waist=n.session.current.get('waist_yaw_joint',0.))
            except (ValueError,RuntimeError,KeyError):available = False
            else:available = True
            button.setEnabled(available and not busy and not active and fresh_gateway and
                              n.gateway.get('phase') == 'ARMED_HOLD' and n.gateway.get('task_stop_active') == 'false')
        waist_active = fresh_gateway and n.gateway.get('phase') == 'TASK_ADMITTANCE' and n.gateway.get('task_waist_admittance_active') == 'true'
        self.waist_status.setText('腰 yaw 导纳：'+('状态过期，勿拖动' if not fresh_gateway else
            '已启用' if waist_active else '未启用，腰保持'))
        for i,stage in enumerate(STAGES):
            trace = c.config.get('traces',{}).get(stage)
            summary = trace.get('summary',{}) if trace else {}
            self.trace_table.item(i,1).setText(f'{summary.get("raw_count", "?")} / {summary.get("replay_count", "?")}' if trace else '-')
            self.trace_table.item(i,2).setText(f'{summary.get("length_mm",0):.1f}' if trace else '-')
            valid = bool(trace)
            if trace:
                try:valid = trace_hand_matches(trace,stage,c.config['hands'])
                except (KeyError,ValueError):valid = False
            self.trace_table.item(i,3).setText('已录制' if valid else '手型已变' if trace else '未录制')
        detail = ''
        if active:
            receiver = recording.get('receiver')
            if receiver is not None:
                status = receiver.latest
                detail = (f' | 直采 {status.get("count",0)} 点 / {status.get("seconds",0):.1f} s'
                          f' / {status.get("stored_hz",0):.0f} Hz')
            else:
                buffer = recording['buffer']
                detail = f' | 原始 {len(buffer.rows)} 点 / {buffer.times[-1] if buffer.times else 0:.1f} s'
        self.trace_status.setText(c.trace_status+detail)
        self.trace_alignment.setText(c.trace_alignment(self.trace_stage.currentData()))
        if c.revision != self.revision:
            for i,key in enumerate(ENDPOINTS):
                try:
                    q = dict(n.session.current,**endpoint(c.config,key))
                    xyz = n.model.transform(LINK,q)[:3,3]
                    text = ', '.join(f'{v:.3f}' for v in xyz)
                except (KeyError,ValueError):text = '未录制'
                self.recorded_points.item(i,2).setText(text)
                self.recorded_points.item(i,2).setToolTip(text)
            self.point_keys = []
            records = []
            for slot in ('A', 'B'):
                for i, record in enumerate(c.config['routes'][slot]):
                    self.point_keys.append((slot, i)); records.append((f'{slot} 接近 {i+1}', record))
                if slot in c.config['points']:
                    self.point_keys.append((slot, None)); records.append((slot+' 抓握端', c.config['points'][slot]))
            self.points.setRowCount(len(records))
            for i, (label, record) in enumerate(records):
                xyz = ', '.join(f'{record["pelvis_from_wrist"][k][3]:.3f}' for k in range(3))
                for j, text in enumerate((label, record['captured_at'][11:19], xyz)):
                    self.points.setItem(i, j, QtWidgets.QTableWidgetItem(text))
            self.revision = c.revision
        for col, state in enumerate((c.latest_hand, c.config['hands'].get('open'), c.config['hands'].get('grip'),
                                     c.config['hands'].get('pull_grip')), 1):
            for i in range(6):
                self.hand_table.item(i, col).setText(f'{state["feedback_q"][i]:.3f}' if state else '-')
                self.hand_table.item(i, col).setToolTip(f'记录时目标 {state["target_q"][i]:.3f}' if state else '')
        cached = n.hand_states.get('right')
        fresh = bool(c.latest_hand and cached and time.monotonic()-cached['received'] <= .5 and
                     cached['state']['timestamp_ms'] == c.latest_hand['timestamp_ms'])
        self.hand_status.setText('读取反馈：'+('新鲜' if fresh else '历史快照 / 未读取'))
        state = c.state if c.state_epoch == int(n.gateway.get('task_stop_epoch', '-1')) else 'unknown'
        self.state_label.setText('滑杆状态：'+{'unknown':'待确认', 'outer':'外端',
            'inner_held':'内端，保持握持', 'inner_released':'内端，已松手'}[state])
        self.file_label.setText(c.path or '未导出（示教修改自动备份）')
        text=str(n.status)
        plan=n.session.plan
        if plan and plan.get('kind')=='rail':
            versions=[STAGES[s['trace_stage']][0]+'：'+s['trace_version']['label']
                      for s in plan.get('segments',[]) if 'trace_stage' in s and 'trace_version' in s]
            if versions:text+='\n计划采用版本：'+'；'.join(versions)
            from task_cruise_timing import summary
            for segment in plan.get('segments',[]):
                timing=segment.get('trace_report',{}).get('timing',{})
                if timing.get('timing_comparison'):
                    timing_text=summary(timing)
                    if timing_text not in text:text+='\n'+STAGES[segment['trace_stage']][0]+'：'+timing_text
        self.status.setText(text)
