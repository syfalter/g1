"""Screen teaching tab for the existing Qt console."""
import math
import time
from pathlib import Path

from PyQt5 import QtCore, QtGui, QtWidgets

import task_screen_teach as data


class CropView(QtWidgets.QGraphicsView):
    def __init__(self, path):
        super().__init__()
        scene = QtWidgets.QGraphicsScene(self)
        self.setScene(scene)
        pixmap = QtGui.QPixmap(str(path))
        if pixmap.isNull(): raise ValueError('无法读取本次 RGB 图像')
        self.image = scene.addPixmap(pixmap)
        self.bounds = self.image.boundingRect()
        self.rectangle = self.bounds
        self.box = scene.addRect(self.rectangle, QtGui.QPen(QtGui.QColor('#ffbe20'), 2))
        self.setMinimumSize(540, 330)
        self.start = None

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self.fitInView(self.bounds, QtCore.Qt.KeepAspectRatio)

    def mousePressEvent(self, event):
        if event.button() == QtCore.Qt.LeftButton:
            self.start = self.mapToScene(event.pos())

    def mouseMoveEvent(self, event):
        if self.start is not None:
            self.rectangle = QtCore.QRectF(self.start, self.mapToScene(event.pos())).normalized().intersected(self.bounds)
            self.box.setRect(self.rectangle)

    def mouseReleaseEvent(self, event):
        self.mouseMoveEvent(event)
        self.start = None


class ScreenPanel(QtWidgets.QWidget):
    def __init__(self, window):
        super().__init__()
        self.window = window
        self.control = window.node.screen
        self.revision = None
        outer = QtWidgets.QVBoxLayout(self)
        scroll = QtWidgets.QScrollArea(); scroll.setWidgetResizable(True)
        content = QtWidgets.QWidget(); layout = QtWidgets.QVBoxLayout(content)
        scroll.setWidget(content); outer.addWidget(scroll)
        self.edit_controls = []

        row = QtWidgets.QHBoxLayout(); layout.addLayout(row)
        self.button(row, '新建', self.new)
        self.button(row, '加载', self.load, QtWidgets.QStyle.SP_DialogOpenButton)
        self.button(row, '导出', self.save, QtWidgets.QStyle.SP_DialogSaveButton)
        self.file_label = QtWidgets.QLabel('未导出'); self.file_label.setWordWrap(True)
        layout.addWidget(self.file_label)

        layout.addWidget(QtWidgets.QLabel('工位对齐 · 固定腰角 0°'))
        form = QtWidgets.QFormLayout(); layout.addLayout(form)
        self.waist_profile = QtWidgets.QComboBox()
        self.waist_profile.addItem('现有控制（PD＋积分/前馈）', 'guarded')
        self.waist_profile.addItem('官方 PD（Kp 240 / Kd 6）', 'official_pd')
        form.addRow('腰控制模式', self.waist_profile)
        self.edit_controls.append(self.waist_profile)
        self.apply_waist_profile = self.button(layout, '应用腰控制模式', self.change_waist_profile)
        self.waist_profile_status = QtWidgets.QLabel()
        self.waist_profile_status.setWordWrap(True)
        layout.addWidget(self.waist_profile_status)
        form = QtWidgets.QFormLayout(); layout.addLayout(form)
        self.waist_target = QtWidgets.QDoubleSpinBox()
        limits = self.window.node.screen.node.model.limits['waist_yaw_joint']
        self.waist_target.setRange(math.degrees(limits[0]+data.JOINT_MARGIN), math.degrees(limits[1]-data.JOINT_MARGIN))
        self.waist_target.setDecimals(2); self.waist_target.setSingleStep(1.)
        self.waist_target.setValue(0.); self.waist_target.setSuffix('°')
        form.addRow('腰部恢复目标', self.waist_target); self.edit_controls.append(self.waist_target)
        self.waist_target.valueChanged.connect(self.waist_target_changed)
        row = QtWidgets.QHBoxLayout(); layout.addLayout(row)
        self.button(row, '规划腰部恢复', self.recover_waist)
        self.button(row, '目标设为 0°', lambda:self.waist_target.setValue(0.))
        self.waist_label = QtWidgets.QLabel(); self.waist_label.setWordWrap(True); layout.addWidget(self.waist_label)
        row = QtWidgets.QHBoxLayout(); layout.addLayout(row)
        self.button(row, '刷新当前点云', self.control.capture, QtWidgets.QStyle.SP_BrowserReload)
        self.button(row, '框选并保存参考物', self.reference)
        self.button(row, '确认本次工位对齐', self.align)
        self.alignment_label = QtWidgets.QLabel('工位：未确认'); self.alignment_label.setWordWrap(True)
        layout.addWidget(self.alignment_label)
        self.button(layout, '站位 / 支撑已改变', self.reset_alignment)
        layout.addWidget(QtWidgets.QLabel('端点示教'))
        self.button(layout, '进入双臂＋腰导纳示教', self.teach)

        self.table = QtWidgets.QTableWidget(6, 3)
        self.table.setHorizontalHeaderLabels(['端点', '记录状态', '腰 yaw (deg)'])
        self.table.verticalHeader().hide()
        self.table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        self.table.setSelectionMode(QtWidgets.QAbstractItemView.SingleSelection)
        self.table.horizontalHeader().setSectionResizeMode(1, QtWidgets.QHeaderView.Stretch)
        self.table.setMinimumHeight(225); self.table.setMaximumHeight(250)
        for i, slot in enumerate(data.SLOTS):
            for j, value in enumerate((slot, '未记录', '-')):
                self.table.setItem(i, j, QtWidgets.QTableWidgetItem(value))
        self.table.selectRow(0)
        layout.addWidget(self.table)
        row = QtWidgets.QHBoxLayout(); layout.addLayout(row)
        self.button(row, '记录选中端点', self.record)
        self.button(row, '退出导纳，原地保持', self.hold)
        self.button(layout, '规划移动到 T（当前姿态 → 公共点）', self.return_to_T)
        self.operation_label = QtWidgets.QLabel(); self.operation_label.setWordWrap(True)
        layout.addWidget(self.operation_label)

        row = QtWidgets.QHBoxLayout(); layout.addLayout(row)
        self.button(row, '拟合屏幕平面', self.control.fit)
        self.button(row, '反转法向', self.control.flip_normal)
        self.button(row, '确认朝外法向', self.confirm_plane)
        self.plane_label = QtWidgets.QLabel('平面：未拟合'); self.plane_label.setWordWrap(True)
        layout.addWidget(self.plane_label)

        form = QtWidgets.QFormLayout(); layout.addLayout(form)
        self.fields = {}
        labels = dict(approach_mm='外侧接近距离 (mm)', dry_clearance_mm='模型预演偏移 (mm)',
                      speed_mm_s='法向速度上限 (mm/s)', dwell_s='到位后额外驻留 (s)',
                      velocity='转场速度比例', acceleration='加速度比例')
        for key, label in labels.items():
            spin = QtWidgets.QDoubleSpinBox(); spin.setRange(*data.BOUNDS[key]); spin.setDecimals(2)
            spin.setSingleStep(.05 if key in ('velocity', 'acceleration', 'dwell_s') else 1.)
            spin.setValue(data.DEFAULTS[key]); self.fields[key] = spin
            spin.valueChanged.connect(lambda _, k=key:self.change_parameters())
            form.addRow(label, spin); self.edit_controls.append(spin)
        form.addRow('额外进给', QtWidgets.QLabel('0 mm（首版不施压）'))
        self.selected = QtWidgets.QComboBox(); self.selected.addItems(data.SLOTS[1:])
        self.mode = QtWidgets.QComboBox(); self.mode.addItems(['模型偏移预演', '回放示教触点'])
        form.addRow('操作点', self.selected); form.addRow('回放模式', self.mode)
        self.edit_controls.extend([self.selected, self.mode])
        self.selected.currentIndexChanged.connect(self.selection_changed)
        self.mode.currentIndexChanged.connect(self.selection_changed)
        row = QtWidgets.QHBoxLayout(); layout.addLayout(row)
        self.button(row, '规划单点循环', lambda:self.control.plan(self.selected.currentText(), self.mode.currentIndex() == 0))
        self.button(row, '对位 / 点击成功', lambda:self.result(True))
        self.button(row, '对位 / 点击失败', lambda:self.result(False))
        self.result_label = QtWidgets.QLabel('尚无本次回放结果'); self.result_label.setWordWrap(True)
        layout.addWidget(self.result_label)
        layout.addStretch(1)

    def button(self, layout, label, callback, icon=None):
        button = QtWidgets.QPushButton(label)
        if icon is not None:
            button.setIcon(self.style().standardIcon(icon))
        button.clicked.connect(lambda:self.window.invoke(callback))
        button.setToolTip(label); layout.addWidget(button)
        self.edit_controls.append(button)
        return button

    def ask(self, title, text):
        return QtWidgets.QMessageBox.question(self, title, text,
            QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No, QtWidgets.QMessageBox.No) == QtWidgets.QMessageBox.Yes

    def new(self):
        if self.ask('新建工位', '清空面板内六点和参考点云？已有导出文件不会删除。'):
            self.control.new()

    def load(self):
        path, _ = QtWidgets.QFileDialog.getOpenFileName(self, '加载屏幕工位', str(self.window.node.run_dir.parent), 'JSON (*.json)')
        if path and self.ask('加载工位', '替换面板内当前数据？执行授权不会恢复。\n若模型唯一变化为腰限幅 35° → 45°，允许校验后迁移旧端点；旧点云需重新建立腰 0° 参考。'):
            self.control.load(path, allow_limit_upgrade=True)

    def save(self):
        path, _ = QtWidgets.QFileDialog.getSaveFileName(self, '导出屏幕工位',
            self.control.path or str(self.window.node.run_dir/'screen_workcell/config.json'), 'JSON (*.json)')
        if path: self.control.save(path)

    def record(self):
        row = self.table.currentRow()
        if row < 0: raise RuntimeError('请选择端点')
        slot = data.SLOTS[row]
        text = ('确认公共点位于屏幕外，指尖未接触物体，机器人已静止？' if slot == 'T' else
                '确认指尖刚触及所选屏幕位置，没有主动继续压入，机器人已静止？')
        if slot in self.control.config['points']: text += '\n将覆盖 '+slot+'，旧平面和计划失效。'
        if self.ask('记录 '+slot, text): self.control.record(slot)

    def hold(self):
        if self.ask('退出导纳', '确认指尖已脱离屏幕且机器人静止？只恢复当前姿态保持，无需手动复刻公共点。'):
            n = self.window.node
            if not n.args.hardware or n.args.disconnected:
                raise RuntimeError('离线/只读面板不允许恢复控制器保持')
            self.control.idle()
            n.recovery(.15, .15)

    def return_to_T(self):
        if self.ask('规划移动到公共点 T',
                    '确认指尖已脱离屏幕、站位和固定支撑未变，右臂及腰转动的回程路径净空？\n'
                    '从当前姿态规划至 T，不新增碰撞检测；查看预览后还需点击执行。'):
            self.control.plan('T', True, return_confirmed=True)

    def change_waist_profile(self):
        mode = self.waist_profile.currentData()
        if self.ask('切换腰控制模式',
                    '确认双手脱离物体、腰部转动范围净空且机器人静止？\n'
                    '静止保持时用 1 秒平滑切换，无需重启或重新 ARM。旧计划和预演确认失效。\n'
                    '官方 PD 稳态 Kp=240、Kd=6、dq=0、tau=0；导纳和异常处理仍沿用原流程。'):
            self.window.node.set_waist_control_mode(mode)

    def recover_waist(self):
        angle = self.waist_target.value()
        if self.ask('规划腰部恢复', f'确认双手已脱离物体、支撑未变、躯干及双臂转动范围内无障碍？\n'
                    f'目标腰角 {angle:.2f}°，两臂关节指令保持不变，但会随躯干转动。查看预览后仍需执行。'):
            self.control.plan('WAIST', True, return_confirmed=True, waist_target_deg=angle)

    def waist_target_changed(self):
        n = self.window.node
        if n.session.plan and n.session.plan.get('screen', {}).get('operation') == 'waist_recovery':
            self.window.invoke(lambda:self.selection_changed())

    def teach(self):
        if not self.control.aligned: raise RuntimeError('先在腰 0° 完成本次工位对齐')
        if self.ask('进入示教', '确认机器人静止、支撑稳固，准备手动示教双臂和腰 yaw？'):
            self.window.node.stop(include_waist=True)

    def confirm_plane(self):
        if self.ask('确认屏幕法向', '已在 RViz 核对绿色法向箭头朝屏幕外侧，且接受当前拟合残差？'):
            self.control.confirm_plane()

    def reference(self):
        capture = self.control.current_capture
        if capture is None: raise RuntimeError('请先刷新当前点云')
        dialog = QtWidgets.QDialog(self); dialog.setWindowTitle('框选固定特征物（包含边缘 / 角点）')
        layout = QtWidgets.QVBoxLayout(dialog)
        view = CropView(Path(capture['directory'])/'rgb.png'); layout.addWidget(view)
        buttons = QtWidgets.QDialogButtonBox(QtWidgets.QDialogButtonBox.Save | QtWidgets.QDialogButtonBox.Cancel)
        buttons.accepted.connect(dialog.accept); buttons.rejected.connect(dialog.reject); layout.addWidget(buttons)
        if dialog.exec_() != QtWidgets.QDialog.Accepted: return
        if self.control.reference is not None and not self.ask('替换参考', '替换旧参考点云？工位和无接触核对将失效。'): return
        r = view.rectangle
        self.control.set_reference([int(r.left()), int(r.top()), int(r.right()), int(r.bottom())])

    def align(self):
        if self.ask('本次工位确认', '确认物理标记、固定支撑均复现；RViz 金色参考与青色新点云的表面、边缘均对齐，屏幕未移动？'):
            self.control.confirm_alignment()

    def reset_alignment(self):
        with self.window.node.lock:
            self.control.idle(); self.control.invalidate()
            self.control.current_capture = self.control.current_cloud = None
            self.window.node.status = '工位确认已清除；请重新采集点云核对'

    def change_parameters(self):
        values = dict(data.DEFAULTS, **{k:w.value() for k, w in self.fields.items()})
        self.window.invoke(lambda:self.control.set_parameters(values))
        self.revision = None

    def selection_changed(self):
        with self.window.node.lock:
            if self.window.node.session.plan and self.window.node.session.plan.get('kind') == 'screen':
                self.control.idle(); self.control.invalidate(alignment=False)

    def result(self, success):
        result = self.control.last_result
        if result is None: raise RuntimeError('没有待确认的执行结果')
        text = ('确认试运行时指尖与目标对位正确且全程保持离屏？' if result['dry'] else '确认屏幕实际响应了本次点击？')
        if not success or self.ask('记录结果', text): self.control.confirm_result(success)

    def refresh(self, busy):
        c = self.control
        for widget in self.edit_controls: widget.setEnabled(not busy)
        if c.revision != self.revision:
            for i, slot in enumerate(data.SLOTS):
                record = c.config['points'].get(slot)
                self.table.item(i, 1).setText(record['captured_at'][11:19] if record else '未记录')
                self.table.item(i, 2).setText(f'{math.degrees(record["joints"]["waist_yaw_joint"]):.2f}' if record else '-')
            for key, spin in self.fields.items():
                spin.blockSignals(True); spin.setValue(c.config['parameters'][key]); spin.blockSignals(False)
            self.revision = c.revision
        self.file_label.setText(c.path or '未导出（记录后有会话自动备份）')
        n = self.window.node
        self.operation_label.setText(str(c.node.status))
        gateway = n.gateway
        available = (not busy and time.monotonic()-n.gateway_received <= 1.5 and
                     gateway.get('phase') in ('UNARMED', 'ARMED_HOLD') and
                     gateway.get('waist_mode_switching') == 'false')
        self.apply_waist_profile.setEnabled(available)
        self.waist_profile_status.setText(
            '网关模式：'+gateway.get('waist_control_mode', '未知 / 需新版网关')+
            ('（过渡中）' if gateway.get('waist_mode_switching') == 'true' else '')+
            ' | 腰目标 '+gateway.get('waist_command_rate_hz', '?')+' Hz'+
            ' | 统一发包 '+gateway.get('control_rate_hz', '?')+' Hz'+
            ' | 下发 Kp/Kd '+gateway.get('waist_command_kp', '?')+'/'+gateway.get('waist_command_kd', '?'))
        q = n.session.current.get('waist_yaw_joint')
        fresh = n.session.clock()-n.session.received <= .15
        self.waist_label.setText('当前腰角：反馈不可用' if q is None or not fresh else
            f'当前腰角 {math.degrees(q):.2f}° | 目标差 {math.degrees(q)-self.waist_target.value():+.2f}°')
        plane = c.config['plane']
        self.plane_label.setText('平面：未拟合' if plane is None else
            f'FK 平面 RMS {plane["rms_mm"]:.2f} mm / 最大 {plane["max_mm"]:.2f} mm；'+('法向已确认' if plane['confirmed'] else '法向待确认'))
        age = f'{time.monotonic()-c.current_capture["received"]:.0f}s' if c.current_capture else '-'
        self.alignment_label.setText(('工位已确认' if c.aligned else '工位未确认')+
            f' | 当前点云年龄 {age} | 无接触已核对：'+(', '.join(sorted(c.dry_checks)) or '无'))
        self.result_label.setText('尚无本次回放结果' if c.last_result is None else
            c.last_result['selected']+' | '+c.last_result['outcome'])
        if self.window.tabs.currentWidget() is self and time.monotonic()-c.last_markers > 2:
            c.publish()
