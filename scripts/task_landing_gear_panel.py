"""Landing-gear teaching tab in the existing Qt console."""
from PyQt5 import QtWidgets

from task_landing_gear_data import SLOTS
from task_rail_panel import RailPanel
from task_speed_control import SpeedInput


class LandingGearPanel(QtWidgets.QWidget):
    button = RailPanel.button
    ask = RailPanel.ask
    teach = RailPanel.teach
    teach_with_waist = RailPanel.teach_with_waist
    hold = RailPanel.hold

    def __init__(self, window):
        super().__init__()
        self.window, self.control = window, window.node.landing_gear
        self.controls = []
        outer = QtWidgets.QVBoxLayout(self)
        scroll = QtWidgets.QScrollArea(); scroll.setWidgetResizable(True)
        body = QtWidgets.QWidget(); layout = QtWidgets.QVBoxLayout(body)
        scroll.setWidget(body); outer.addWidget(scroll)
        row = QtWidgets.QHBoxLayout(); layout.addLayout(row)
        self.button(row, '新建', self.new)
        self.button(row, '导入', self.load)
        self.button(row, '导出', self.save)
        self.file = QtWidgets.QLabel(); self.file.setWordWrap(True); layout.addWidget(self.file)
        row = QtWidgets.QHBoxLayout(); layout.addLayout(row)
        self.button(row, '进入手臂导纳', self.teach)
        self.button(row, '退出导纳，原地保持', self.hold)
        self.button(layout, '进入双臂＋腰 yaw 导纳', self.teach_with_waist)
        self.hand_table = QtWidgets.QTableWidget(6, 4)
        self.hand_table.setHorizontalHeaderLabels(['通道', '实测', '张开', '抓握'])
        self.hand_table.verticalHeader().hide()
        self.hand_table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.hand_table.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.Stretch)
        self.hand_table.setFixedHeight(215)
        for i, label in enumerate(('小指', '无名指', '中指', '食指', '拇指 2', '拇指 1')):
            for j, text in enumerate((label, '-', '-', '-')):
                self.hand_table.setItem(i, j, QtWidgets.QTableWidgetItem(text))
        layout.addWidget(self.hand_table)
        self.button(layout, '读取右手反馈', lambda:self.control.record())
        row = QtWidgets.QHBoxLayout(); layout.addLayout(row)
        for name, label in (('open', '张开'), ('grip', '抓握')):
            self.button(row, '保存'+label+'手型', lambda name=name:self.record_hand(name))
        row = QtWidgets.QHBoxLayout(); layout.addLayout(row)
        self.hand_buttons = {}
        for name, label in (('open', '右手张开'), ('grip', '右手抓握')):
            self.hand_buttons[name] = self.button(row, label, lambda name=name:self.apply_hand(name))
        self.table = QtWidgets.QTableWidget(len(SLOTS), 3)
        self.table.setHorizontalHeaderLabels(['录制段', '时长', '样本数'])
        self.table.verticalHeader().hide()
        self.table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        self.table.setSelectionMode(QtWidgets.QAbstractItemView.SingleSelection)
        self.table.horizontalHeader().setSectionResizeMode(0, QtWidgets.QHeaderView.Stretch)
        self.table.setColumnWidth(1, 85); self.table.setColumnWidth(2, 85)
        self.table.setFixedHeight(225)
        for i, label in enumerate(SLOTS.values()):
            for j, text in enumerate((label, '未录制', '-')):
                self.table.setItem(i, j, QtWidgets.QTableWidgetItem(text))
        self.table.selectRow(0); layout.addWidget(self.table); self.controls.append(self.table)
        self.table.itemSelectionChanged.connect(self.changed)
        from task_trace_editor_panel import TraceEditorPanel
        self.editor=TraceEditorPanel(window,self.control,self.slot)
        layout.addWidget(self.editor)
        self.table.itemSelectionChanged.connect(lambda:self.editor.refresh())
        row = QtWidgets.QHBoxLayout(); layout.addLayout(row)
        self.button(row, '开始录制', self.record)
        self.finish = self.button(row, '结束保存', self.control.finish_trace)
        self.cancel = self.button(row, '取消录制', self.control.cancel_trace)
        self.button(layout, '删除选中段', self.delete)
        form = QtWidgets.QFormLayout(); layout.addLayout(form)
        self.speed = SpeedInput()
        self.speed.setToolTip('录制回放倍率；起点衔接和手型切换单独限速')
        self.velocity = QtWidgets.QDoubleSpinBox(); self.acceleration = QtWidgets.QDoubleSpinBox()
        for spin in (self.velocity, self.acceleration):
            spin.setRange(.05, 1.); spin.setValue(.7); spin.setSingleStep(.05)
        for a,b in ((self.velocity,self.editor.fast_velocity),(self.acceleration,self.editor.fast_acceleration)):
            a.valueChanged.connect(b.setValue);b.valueChanged.connect(a.setValue)
        for label, widget in (('回放倍率', self.speed), ('Velocity limit', self.velocity), ('Acceleration limit', self.acceleration)):
            form.addRow(label, widget); self.controls.append(widget); widget.valueChanged.connect(self.changed)
        self.confirm = QtWidgets.QCheckBox('外部控制已停止；杆位与手型已核对；含衔接的完整路径净空')
        self.confirm.setToolTip('衔接不做碰撞检测；抓握后不得通过未知路径自动回位')
        layout.addWidget(self.confirm); self.controls.append(self.confirm)
        row = QtWidgets.QHBoxLayout(); layout.addLayout(row)
        self.button(row, '生成选中段 / RViz', self.prepare)
        self.button(row, '重新播放 RViz', self.control.replay_preview)
        from task_recorded_history_panel import RecordedHistoryPanel
        self.history=RecordedHistoryPanel(window,self.control);layout.addWidget(self.history)
        from task_jetson_push_panel import JetsonPushPanel
        self.jetson_push = JetsonPushPanel(window, 'landing_gear'); layout.addWidget(self.jetson_push)
        self.status = QtWidgets.QLabel(); self.status.setWordWrap(True); layout.addWidget(self.status)
        layout.addStretch(1)

    def slot(self):
        return tuple(SLOTS)[max(0, self.table.currentRow())]

    def changed(self, *_):
        n = self.window.node
        with n.lock:
            plan = n.session.plan
            if plan and plan.get('demo', {}).get('workflow') == 'landing_gear' and n.session.phase not in ('EXECUTION_PREFLIGHT', 'EXECUTING'):
                n.session.invalidate(); n.session.phase = 'REPLAN_REQUIRED'
                self.window.clear.setChecked(False)
        if hasattr(self, 'confirm'):self.confirm.setChecked(False)

    def new(self):
        if self.ask('新建起落架杆示教', '清空六段轨迹？保留手型和历史原始文件。'):
            self.control.new()

    def load(self):
        path, _ = QtWidgets.QFileDialog.getOpenFileName(self, '导入起落架杆示教', str(self.control.store.parent), 'JSON (*.json)')
        if path and self.ask('导入', '替换当前起落架杆示教？旧执行授权将失效。'):self.control.load(path)

    def save(self):
        path, _ = QtWidgets.QFileDialog.getSaveFileName(self, '导出起落架杆示教',
            self.control.path or str(self.window.node.run_dir/'landing_gear.json'), 'JSON (*.json)')
        if path:self.control.save(path)

    def record_hand(self, name):
        if self.ask('保存手型', '保存当前右手实测手型？仅读取，不发送动作；只影响起落架杆模块。'):
            self.control.record(hand=name)

    def apply_hand(self, name):
        if self.ask('右手手型', '确认外部控制已停止、手臂静止、右手周围净空，且松手后杆件稳定？'):
            self.control.apply_hand(name, confirmed=True)

    def record(self):
        if self.ask('录制 '+SLOTS[self.slot()], '从当前静止位置开始录制，不校验预存起点。等待录制中后拖动，到本段终点停稳再结束；录制不自动切换手型。'):
            self.control.start_trace(self.slot())

    def delete(self):
        if self.ask('删除轨迹', '删除 '+SLOTS[self.slot()]+'？原始文件保留。'):self.control.delete_trace(self.slot())

    def prepare(self):
        self.control.prepare(self.slot(), self.speed.value(), self.velocity.value(), self.acceleration.value(), self.confirm.isChecked())
        self.confirm.setChecked(False); self.window.clear.setChecked(False)

    def refresh(self, busy):
        c = self.control; rec = c.recording
        self.editor.refresh(busy)
        self.history.refresh(busy)
        for widget in self.controls:
            if widget not in (self.finish, self.cancel):widget.setEnabled(not busy and rec is None)
        self.finish.setEnabled(rec is not None and rec['phase'] == 'recording' and not rec['finish'].is_set())
        self.cancel.setEnabled(rec is not None and not rec['cancel'].is_set())
        for name, button in self.hand_buttons.items():
            button.setEnabled(not busy and rec is None and name in c.config['hands'])
        for i, slot in enumerate(SLOTS):
            raw = c.config['traces'].get(slot, {}).get('raw')
            self.table.item(i, 1).setText(f'{raw["time_s"][-1]:.2f} s' if raw else '未录制')
            self.table.item(i, 2).setText(str(len(raw['positions'])) if raw else '-')
        for j, state in enumerate((c.latest_hand, c.config['hands'].get('open'), c.config['hands'].get('grip')), 1):
            for i in range(6):self.hand_table.item(i, j).setText(f'{state["feedback_q"][i]:.3f}' if state else '-')
        self.file.setText(c.path or str(c.store))
        text = c.status
        if rec and rec.get('receiver'):
            latest = rec['receiver'].latest
            text += f' | {latest.get("seconds", 0.):.1f} s / {latest.get("count", 0)} 点'
        plan = self.window.node.session.plan
        if plan and plan.get('demo', {}).get('workflow') == c.workflow:
            text += '\n'+' | '.join(f'{s["label"]} {s["duration"]:.2f}s' for s in plan['segments'])
            text += '\n衔接碰撞检测：未执行'
        self.status.setText(text)
        self.jetson_push.refresh(busy)
