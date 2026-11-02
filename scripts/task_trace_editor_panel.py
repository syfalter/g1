"""Immutable baseline, fixed edit slots and an offline recorded-time editor."""
from datetime import datetime
import threading
import time

import numpy as np
from PyQt5 import QtCore, QtWidgets

from task_trace_versions import EDIT_SLOTS, bank
from task_numeric_input import FreeDoubleSpinBox, FreeIntegerSpinBox


class TraceEditorPanel(QtWidgets.QWidget):
    markers_ready = QtCore.pyqtSignal(object, int)
    markers_failed = QtCore.pyqtSignal(str, int)
    point_picked = QtCore.pyqtSignal(object, int)

    def __init__(self, window, control, slot, *, timeline=True):
        super().__init__()
        self.window, self.control, self.slot = window, control, slot
        self.key = None
        self.trace = None
        self.controls = []
        self.marker_serial = 0
        self.marker_thread = None
        from task_trace_display_cache import TraceDisplayCache
        self.display_cache=TraceDisplayCache()
        self.last_marker_blob=None
        self.position = 0.
        self.key_times = []
        self.compared = []
        self.updating = False
        self.was_busy = False
        self.native_playback = False
        self.timer = QtCore.QTimer(self)
        self.timer.setInterval(33)
        self.timer.timeout.connect(self.tick)
        self.marker_timer = QtCore.QTimer(self)
        self.marker_timer.setSingleShot(True)
        self.marker_timer.setInterval(60)
        self.marker_timer.timeout.connect(self.draw_markers)
        self.switch_timer=QtCore.QTimer(self);self.switch_timer.setInterval(16)
        self.switch_timer.timeout.connect(self.finish_switch)
        self.markers_ready.connect(self.publish_markers)
        self.markers_failed.connect(self.marker_error)
        self.point_picked.connect(self.pick_point)
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        self.capacity = QtWidgets.QLabel();layout.addWidget(self.capacity)
        self.versions = QtWidgets.QTableWidget(EDIT_SLOTS+1, 5)
        self.versions.setHorizontalHeaderLabels(['版本', '时长', '修改', '对比', '编辑时间'])
        self.versions.verticalHeader().hide()
        self.versions.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.versions.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        self.versions.setSelectionMode(QtWidgets.QAbstractItemView.SingleSelection)
        self.versions.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.Stretch)
        self.versions.horizontalHeader().setSectionResizeMode(3,QtWidgets.QHeaderView.Fixed)
        self.versions.setColumnWidth(3,48)
        self.versions.horizontalHeader().setSectionResizeMode(4,QtWidgets.QHeaderView.Fixed)
        self.versions.setColumnWidth(4,145)
        self.versions.verticalHeader().setDefaultSectionSize(44)
        self.versions.setFixedHeight(300)
        for i in range(EDIT_SLOTS+1):
            for j, text in enumerate(('蓝本' if i == 0 else f'修改 {i}', '-', '-')):
                self.versions.setItem(i, j, QtWidgets.QTableWidgetItem(text))
            item=QtWidgets.QTableWidgetItem()
            item.setFlags(QtCore.Qt.ItemIsEnabled|QtCore.Qt.ItemIsUserCheckable)
            item.setCheckState(QtCore.Qt.Unchecked);self.versions.setItem(i,3,item)
            self.versions.setItem(i,4,QtWidgets.QTableWidgetItem('-'))
        self.versions.cellClicked.connect(self.choose)
        self.versions.itemChanged.connect(self.comparison_changed)
        layout.addWidget(self.versions)
        self.controls.append(self.versions)
        form = QtWidgets.QFormLayout(); layout.addLayout(form)
        self.generation_mode=QtWidgets.QComboBox()
        self.generation_mode.addItem('原轨迹生成 / 原调速方式','original')
        self.generation_mode.addItem('整段近匀速重定时','cruise')
        self.generation_mode.currentIndexChanged.connect(self.change_generation_mode)
        self.generation_speed=FreeDoubleSpinBox(1.,positive=True)
        self.generation_speed.setSuffix(' x')
        self.generation_speed.setSingleStep(.1)
        form.addRow('执行轨迹生成方式',self.generation_mode)
        form.addRow('离线时序预览倍率',self.generation_speed)
        self.controls.extend((self.generation_mode,self.generation_speed))
        self.button(layout,'生成时序预览 / RViz',self.generate_timing_preview)
        form=QtWidgets.QFormLayout();layout.addLayout(form)
        self.fast_velocity=FreeDoubleSpinBox(.7,positive=True)
        self.fast_acceleration=FreeDoubleSpinBox(.7,positive=True)
        for label,widget in (('重定时速度限值比例',self.fast_velocity),('重定时加速度限值比例',self.fast_acceleration)):
            widget.setSingleStep(.05);form.addRow(label,widget);self.controls.append(widget)
        self.fast_denoise=QtWidgets.QCheckBox('重定时前去抖（改变路径）')
        layout.addWidget(self.fast_denoise);self.controls.append(self.fast_denoise)
        self.fast_button=self.button(layout,'限值内尽快重定时并保存版本',self.generate_fast_version,
            QtWidgets.QStyle.SP_MediaSeekForward)
        self.timing_status=QtWidgets.QLabel();self.timing_status.setWordWrap(True)
        layout.addWidget(self.timing_status)
        form=QtWidgets.QFormLayout();layout.addLayout(form)
        self.point_spacing=FreeDoubleSpinBox(20.,positive=True)
        self.point_limit=FreeIntegerSpinBox(40)
        self.arrow_length=FreeDoubleSpinBox(40.,positive=True)
        self.arrow_diameter=FreeDoubleSpinBox(3.6,positive=True);self.arrow_diameter.setSingleStep(.5)
        self.point_diameter=FreeDoubleSpinBox(18.,positive=True)
        self.point_diameter.setSingleStep(2.)
        for label,widget in (('轨迹点间距 (mm)',self.point_spacing),('每条轨迹最多显示点',self.point_limit),('方向箭头长度 (mm)',self.arrow_length),('箭杆直径 (mm)',self.arrow_diameter),('选点球直径 (mm)',self.point_diameter)):
            form.addRow(label,widget);self.controls.append(widget)
            widget.valueChanged.connect(self.schedule_markers)
        self.compare_mode=QtWidgets.QComboBox()
        self.compare_mode.addItem('路径进度','path');self.compare_mode.addItem('归一化时间','time')
        self.compare_reverse=QtWidgets.QCheckBox('反向 B')
        self.compare_only=QtWidgets.QCheckBox('只突出差异');self.compare_only.setChecked(True)
        self.compare_reverse.setToolTip('仅将 B 反向用于几何对比，不修改或反向执行轨迹')
        self.compare_mm=FreeDoubleSpinBox(2.,positive=True)
        self.compare_deg=FreeDoubleSpinBox(2.,positive=True)
        for label,widget in (('对比对齐',self.compare_mode),('',self.compare_reverse),('',self.compare_only),
                             ('位置差异阈值 (mm)',self.compare_mm),('朝向差异阈值 (°)',self.compare_deg)):
            form.addRow(label,widget);self.controls.append(widget)
        self.compare_mode.currentIndexChanged.connect(self.schedule_markers)
        self.compare_reverse.toggled.connect(self.schedule_markers)
        self.compare_only.toggled.connect(self.schedule_markers)
        self.compare_mm.valueChanged.connect(self.schedule_markers)
        self.compare_deg.valueChanged.connect(self.schedule_markers)
        self.comparison_status=QtWidgets.QLabel();self.comparison_status.setWordWrap(True)
        layout.addWidget(self.comparison_status)
        row = QtWidgets.QHBoxLayout(); layout.addLayout(row)
        self.remove = self.button(row, '删除版本', self.delete, QtWidgets.QStyle.SP_TrashIcon)
        self.smooth_button = self.button(row, '去抖并重参数化', self.smooth)
        self.tolerance = FreeDoubleSpinBox(.003,positive=True)
        self.tolerance.setSingleStep(.001)
        self.window_s = FreeDoubleSpinBox(.15,positive=True); self.window_s.setSingleStep(.05)
        form = QtWidgets.QFormLayout(); layout.addLayout(form)
        form.addRow('去抖偏差上限 (rad)', self.tolerance)
        form.addRow('平滑窗口 (s)', self.window_s)
        self.controls.extend((self.tolerance, self.window_s))
        if control.workflow == 'landing_gear':
            self.button(layout, '由对应反向段生成候选', self.reverse)
            self.anchor_source = QtWidgets.QComboBox()
            self.anchor_source.addItem('下杆位 → 上杆位','lower_to_upper')
            self.anchor_source.addItem('上杆位 → 下杆位','upper_to_lower')
            self.anchor_limit = FreeDoubleSpinBox(.02,positive=True); self.anchor_limit.setSingleStep(.005)
            form.addRow('上下杆位来源',self.anchor_source)
            form.addRow('端点修正上限 (rad)',self.anchor_limit)
            self.controls.extend((self.anchor_source,self.anchor_limit))
            self.button(layout, '按主行程对齐本段端点', lambda:self.control.edit_version(
                self.slot(),'anchor',source=self.anchor_source.currentData(),max_delta=self.anchor_limit.value()))
        self.has_timeline = timeline
        if timeline:
            row = QtWidgets.QHBoxLayout(); layout.addLayout(row)
            self.play = self.button(row, '', self.toggle, QtWidgets.QStyle.SP_MediaPlay)
            self.play.setToolTip('播放 / 暂停 RViz')
            self.slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
            self.slider.valueChanged.connect(self.seek)
            row.addWidget(self.slider); self.controls.append(self.slider)
            self.stamp = QtWidgets.QLabel('0.000 s'); row.addWidget(self.stamp)
            form = QtWidgets.QFormLayout(); layout.addLayout(form)
            self.start = QtWidgets.QDoubleSpinBox(); self.end = QtWidgets.QDoubleSpinBox()
            self.bridge = FreeDoubleSpinBox(.6,positive=True)
            for label, spin in (('选区起点 (s)', self.start), ('选区终点 (s)', self.end), ('衔接时长 (s)', self.bridge)):
                if spin is not self.bridge:spin.setDecimals(3)
                spin.setSingleStep(.1); form.addRow(label, spin); self.controls.append(spin)
            self.start.valueChanged.connect(self.schedule_markers)
            self.end.valueChanged.connect(self.schedule_markers)
            row = QtWidgets.QHBoxLayout(); layout.addLayout(row)
            self.button(row, '设为起点', lambda:self.start.setValue(self.position))
            self.button(row, '设为终点', lambda:self.end.setValue(self.position))
            self.button(layout, '删除区间并平滑衔接', self.cut, QtWidgets.QStyle.SP_DialogDiscardButton)
        form = QtWidgets.QFormLayout(); layout.addLayout(form)
        self.tail_seconds=FreeDoubleSpinBox(1.,positive=True)
        self.tail_seconds.setSingleStep(.1)
        form.addRow('末尾删除时长 (s)',self.tail_seconds)
        self.controls.append(self.tail_seconds)
        self.button(layout,'删除末尾并保存版本',self.trim_tail,QtWidgets.QStyle.SP_TrashIcon)
        form = QtWidgets.QFormLayout(); layout.addLayout(form)
        self.local_factor=FreeDoubleSpinBox(1.5,positive=True)
        self.local_factor.setSingleStep(.1);self.local_factor.setSuffix(' x')
        self.local_blend=FreeDoubleSpinBox(.2,positive=True)
        self.local_blend.setSingleStep(.1)
        form.addRow('选区调速倍率',self.local_factor);form.addRow('调速过渡 (s)',self.local_blend)
        self.controls.extend((self.local_factor,self.local_blend))
        self.button(layout,'选区调速并保存版本',self.retime_selection)
        layout.addWidget(QtWidgets.QLabel('曲线关键点'))
        self.keys=QtWidgets.QListWidget();self.keys.setFixedHeight(110)
        self.keys.itemClicked.connect(self.seek_key);layout.addWidget(self.keys);self.controls.append(self.keys)
        row=QtWidgets.QHBoxLayout();layout.addLayout(row)
        self.button(row,'添加当前点',self.add_key,QtWidgets.QStyle.SP_DialogApplyButton)
        self.button(row,'移除选点',self.remove_key,QtWidgets.QStyle.SP_TrashIcon)
        self.button(row,'清空选点',self.clear_keys,QtWidgets.QStyle.SP_DialogResetButton)
        self.curve_seconds=FreeDoubleSpinBox(0.)
        self.curve_seconds.setMinimum(0.)
        self.curve_seconds.setSpecialValueText('沿用选中区间时长');self.controls.append(self.curve_seconds)
        form=QtWidgets.QFormLayout();layout.addLayout(form);form.addRow('曲线时长 (s)',self.curve_seconds)
        self.curve_button=self.button(layout,'通过选点生成平滑曲线',self.interpolate_keys)
        self.operation_status=QtWidgets.QLabel();self.operation_status.setWordWrap(True)
        self.operation_status.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
        layout.addWidget(self.operation_status)
        from task_trace_manual_panel import ManualPointPanel
        self.manual = ManualPointPanel(self)
        layout.addWidget(self.manual);self.controls.append(self.manual)
        self.error = QtWidgets.QLabel(); self.error.setWordWrap(True); layout.addWidget(self.error)
        self.setToolTip('食指末端模型轨迹，箭头沿机械臂末端 tool0 指向当前手型食指尖；未录制手指时使用固定手型，不代表实测接触点')

    def change_generation_mode(self):
        def change():
            try:self.control.set_generation_mode(self.generation_mode.currentData())
            except Exception:
                blocker=QtCore.QSignalBlocker(self.generation_mode)
                self.generation_mode.setCurrentIndex(self.generation_mode.findData(
                    getattr(self.control,'generation_mode','original')))
                del blocker
                raise
        self.window.invoke(change)

    def generate_timing_preview(self):
        self.stop_playback()
        if hasattr(self,'external_pause'):self.external_pause()
        self.control.preview_timing(self.slot(),self.generation_mode.currentData(),self.generation_speed.value())

    def generate_fast_version(self):
        self.stop_playback()
        if hasattr(self,'external_pause'):self.external_pause()
        self.control.edit_version(self.slot(),'fastest',velocity=self.fast_velocity.value(),
            acceleration=self.fast_acceleration.value(),denoise=self.fast_denoise.isChecked(),
            tolerance=self.tolerance.value(),window=self.window_s.value())

    def button(self, layout, label, action, icon=None):
        button = QtWidgets.QPushButton(label)
        if icon is not None:button.setIcon(self.style().standardIcon(icon))
        button.clicked.connect(lambda:self.window.invoke(lambda:self.perform(action)))
        layout.addWidget(button); self.controls.append(button)
        return button

    def perform(self,action):
        try:action()
        except Exception as error:
            self.control.editor_feedback='不成功：'+str(error)
            self.operation_status.setText(self.control.editor_feedback)
            raise
        self.operation_status.setText(getattr(self.control,'editor_feedback',''))

    def choose(self, index, _column=0):
        if _column == 3:return
        self.stop_playback()
        if self.trace is not None:
            entry = bank(self.control.config, self.slot())
            if index == entry['selected']:return
            if index and entry['edits'][index-1] is None:return
            self.window.invoke(lambda:self.control.edit_version(self.slot(), 'select', index=index))
            for widget in self.controls:widget.setEnabled(False)
            self.was_busy=True;self.switch_timer.start()

    def finish_switch(self):
        worker=self.control.node.worker
        if worker is not None and worker.is_alive():return
        self.switch_timer.stop();self.refresh()

    def comparison_changed(self,item):
        if self.updating or item.column()!=3:return
        index=item.row()
        if item.checkState()==QtCore.Qt.Checked:
            if len(self.compared)>=2:
                self.updating=True;item.setCheckState(QtCore.Qt.Unchecked);self.updating=False
                self.error.setText('最多同时对比两个版本');return
            self.compared.append(index)
        elif index in self.compared:self.compared.remove(index)
        if hasattr(self.window,'clear'):self.window.clear.setChecked(False)
        self.schedule_markers()

    def selected_versions(self):
        if self.trace is None:return []
        entry=bank(self.control.config,self.slot())
        rows=[entry['baseline']]+entry['edits']
        return [rows[i] for i in self.compared if rows[i] is not None]

    def key_list(self):
        self.keys.clear()
        for i,stamp in enumerate(self.key_times,1):self.keys.addItem(f'{i:02d}   {stamp:.3f} s')
        self.curve_button.setEnabled(len(self.key_times)>=3)
        self.schedule_markers()

    def add_key(self):
        if self.trace is None:return
        self.stop_playback()
        if not self.has_timeline:self.external_pause()
        stamp=self.position if self.has_timeline else self.external_position()
        duration=self.trace['raw']['time_s'][-1]
        if abs(stamp-duration)<.00051:stamp=duration
        if len(self.key_times)>=64:raise ValueError('最多选择 64 个关键点')
        if any(abs(stamp-old)<.001 for old in self.key_times):raise ValueError('当前时间点已选中')
        self.key_times=sorted([*self.key_times,float(stamp)]);self.key_list()

    def remove_key(self):
        index=self.keys.currentRow()
        if index>=0:self.key_times.pop(index);self.key_list()

    def clear_keys(self):
        self.key_times=[];self.key_list()

    def seek_key(self,item):
        index=self.keys.row(item)
        if self.has_timeline:self.seek(round(self.key_times[index]*1000))
        else:self.external_seek(round(self.key_times[index]*1000))

    def interpolate_keys(self):
        self.stop_playback()
        if len(self.key_times)<3:raise ValueError('曲线至少需要三个关键点')
        self.control.edit_version(self.slot(),'curve',key_times=list(self.key_times),seconds=self.curve_seconds.value())

    def delete(self):
        index = self.versions.currentRow()
        if index <= 0:raise ValueError('蓝本不能删除')
        self.control.edit_version(self.slot(), 'delete', index=index)

    def smooth(self):
        self.stop_playback()
        self.control.edit_version(self.slot(), 'smooth', tolerance=self.tolerance.value(), window=self.window_s.value())

    def retime_selection(self):
        self.stop_playback()
        if hasattr(self,'external_pause'):self.external_pause()
        start,end=self.cut_range()
        self.control.edit_version(self.slot(),'local_speed',start=start,end=end,
                                  factor=self.local_factor.value(),blend=self.local_blend.value())

    def reverse(self):
        from task_landing_gear_data import ENDPOINTS
        target = self.slot()
        source = next(k for k, ends in ENDPOINTS.items() if ends == ENDPOINTS[target][::-1])
        if source not in self.control.config['traces']:raise ValueError('对应反向段尚未录制')
        self.control.edit_version(target, 'reverse', source=source)

    def cut(self):
        self.stop_playback()
        a, b = self.cut_range()
        self.control.edit_version(self.slot(), 'cut', start=a, end=b, bridge=self.bridge.value())

    def trim_tail(self):
        self.stop_playback()
        if hasattr(self,'external_pause'):self.external_pause()
        self.control.edit_version(self.slot(),'trim_tail',seconds=self.tail_seconds.value())

    def cut_range(self):
        if not self.has_timeline:return self.external_range()
        if self.trace is None:return 0., 0.
        duration = self.trace['raw']['time_s'][-1]
        return tuple(duration if spin.value() == spin.maximum() else spin.value() for spin in (self.start, self.end))

    def refresh(self, busy=False):
        self.operation_status.setText(getattr(self.control,'editor_feedback',''))
        key = (self.control.revision, self.slot())
        trace = self.control.config['traces'].get(self.slot())
        blocked = busy or self.control.recording is not None
        for widget in self.controls:widget.setEnabled(not blocked and trace is not None)
        self.remove.setEnabled(not blocked and self.versions.currentRow() > 0)
        self.curve_button.setEnabled(not blocked and len(self.key_times)>=3)
        if blocked:
            if hasattr(self.control.node,'trace_picker'):self.control.node.trace_picker.clear(self)
            if not self.was_busy:self.control.editor_hide()
            self.last_marker_blob=None
            self.was_busy=True
            self.stop_playback(); self.marker_timer.stop(); self.marker_serial += 1
            return
        if self.was_busy:self.schedule_markers()
        self.was_busy=False
        if key == self.key:return
        previous_slot=self.key[1] if self.key else None
        previous_trace=self.trace
        self.stop_playback(); self.key = key; self.trace = trace
        timing=trace.get('optimization',{}).get('timing_report') if trace else None
        self.timing_status.setText((f'当前 {timing["duration_after_s"]:.3f} s | '
            f'原策略 1x {timing["original_1x_seconds"]:.3f} s | '
            f'原策略已验证最快 {timing["original_fastest_seconds"]:.3f} s | '
            f'相对原 1x {timing["speedup_over_original_1x"]:.2f}x | '+
            ('已去抖，路径已改变' if timing.get('path_modified') else '保留拟合路径')+
            f' | 保存限值比例 V/A {timing["velocity_scaling"]:g}/{timing["acceleration_scaling"]:g}') if timing else '')
        if trace is not previous_trace:
            self.manual.reset()
            self.key_times=list(trace.get('optimization',{}).get('key_times_s',[])) if trace else []
            self.key_list()
        if previous_slot!=self.slot():self.compared=[]
        entry = bank(self.control.config, self.slot()) if trace is not None else None
        rows = [entry['baseline']]+entry['edits'] if entry else [None]*(EDIT_SLOTS+1)
        used=sum(item is not None for item in rows[1:])
        self.capacity.setText(f'修改版本 {used} / {EDIT_SLOTS}'+('（已满，请删除一个后再编辑）' if used==EDIT_SLOTS else ''))
        self.compared=[i for i in self.compared if rows[i] is not None]
        self.updating=True
        for i, item in enumerate(rows):
            name='蓝本' if i==0 else f'修改 {i}'
            self.versions.item(i,0).setText(name+(' *' if entry and i==entry['selected'] else ''))
            self.versions.item(i, 1).setText(f'{item["raw"]["time_s"][-1]:.3f} s' if item else '空')
            label = (item.get('optimization') or item.get('trim_edit') or {}).get('label', '原始录制') if item else '-'
            self.versions.item(i, 2).setText(label)
            origin=item.get('edit_origin') if item else None
            parent=('蓝本' if origin['version']==0 else f'修改 {origin["version"]}') if origin else ''
            self.versions.item(i, 2).setToolTip(label+(f'\n来源：{parent}\n{origin.get("created_at", "未记录")}' if origin else ''))
            recorded=origin.get('created_at') if origin else None
            text='蓝本' if i==0 and item else '未记录' if item else '-'
            if recorded:
                try:text=datetime.fromisoformat(recorded).astimezone().strftime('%Y-%m-%d\n%H:%M:%S')
                except (TypeError,ValueError):text=str(recorded)
            self.versions.item(i,4).setText(text)
            self.versions.item(i,4).setToolTip(str(recorded or text))
            flags=QtCore.Qt.ItemIsUserCheckable|(QtCore.Qt.ItemIsEnabled if item else QtCore.Qt.NoItemFlags)
            self.versions.item(i,3).setFlags(flags)
            self.versions.item(i,3).setCheckState(QtCore.Qt.Checked if i in self.compared else QtCore.Qt.Unchecked)
        self.updating=False
        if trace is None:self.control.editor_hide()
        if entry:self.versions.selectRow(entry['selected'])
        if trace and self.has_timeline:
            self.times = np.asarray(trace['raw']['time_s']); self.rows = np.asarray(trace['raw']['positions'])
            self.position = 0.
            self.slider.blockSignals(True); self.slider.setRange(0, round(self.times[-1]*1000)); self.slider.setValue(0); self.slider.blockSignals(False)
            for spin in (self.start, self.end):spin.blockSignals(True); spin.setRange(0., self.times[-1])
            self.start.setValue(0.); self.end.setValue(0.)
            for spin in (self.start, self.end):spin.blockSignals(False)
            if self.isVisible():self.pose()
        self.schedule_markers()

        if self.isVisible():self.marker_timer.start(0)

    def pose(self):
        right = min(int(np.searchsorted(self.times, self.position, side='right')), len(self.times)-1)
        left = max(0, right-1)
        delta = self.times[right]-self.times[left]
        weight = np.clip((self.position-self.times[left])/delta, 0., 1.) if delta > 0 else 0.
        q = self.rows[left]+weight*(self.rows[right]-self.rows[left])
        self.control.editor_pose(q, self.trace, self.slot())
        self.position_widgets()

    def position_widgets(self):
        text=f'{self.position:.3f} / {self.times[-1]:.3f} s'
        if self.native_playback:
            from task_composition_preview import at
            source=self.control._editor_playback_cache[1]
            velocity=at(source,self.position).velocities
            text+=f' | 轨迹关节峰值 {max(map(abs,velocity),default=0.):.3f} rad/s'
            received,health=getattr(self.control.node,'offline_preview_health',(0.,{}))
            if time.monotonic()-received<2.:
                text+=f' | 姿态输出 {health.get("hz",0.):.0f} Hz / 最大间隔 {health.get("max_gap_ms",0.):.0f} ms'
            else:text+=' | 姿态输出反馈未收到'
        self.stamp.setWordWrap(True)
        self.stamp.setText(text)
        self.slider.blockSignals(True); self.slider.setValue(round(self.position*1000)); self.slider.blockSignals(False)

    def seek(self, value):
        if self.trace is None:return
        self.stop_playback(); self.position = min(value*.001, self.times[-1]); self.pose()

    def toggle(self):
        if self.timer.isActive():self.stop_playback();self.pose();return
        if self.trace is None:return
        if self.position >= self.times[-1]:self.position = 0.
        self.native_playback=self.control.editor_playback(self.trace,self.slot(),self.position)
        self.origin = time.monotonic()-self.position; self.timer.start()

    def stop_playback(self):
        self.timer.stop()
        self.control.editor_stop_playback()
        self.native_playback=False

    def tick(self):
        if not self.isVisible():self.stop_playback();return
        self.position = min(time.monotonic()-self.origin, self.times[-1])
        if self.native_playback:self.position_widgets()
        else:self.pose()
        if self.position >= self.times[-1]:self.stop_playback();self.pose()

    def schedule_markers(self, *_):
        self.marker_serial += 1
        if self.isVisible():self.marker_timer.start()

    def draw_markers(self):
        n = self.control.node
        if not self.isVisible() or self.trace is None or not hasattr(n.model, 'batch_transforms'):return
        if self.marker_thread is not None and self.marker_thread.is_alive():
            self.marker_timer.start(); return
        if self.control.recording is not None or n.session.phase in ('PLANNING','EXECUTION_PREFLIGHT','EXECUTING'):return
        serial = self.marker_serial
        trace, config, slot = self.trace, self.control.config, self.slot()
        context = dict(getattr(n.session, 'current', {}) or {})
        context.update(trace.get('context_joints', {}))
        start, end = self.cut_range()
        side = self.control.editor_side()
        key_times=list(self.key_times)
        compared=self.selected_versions()
        active_version=bank(config,slot)['selected']
        compared_indices=list(self.compared)
        options=dict(mode=self.compare_mode.currentData(),reverse=self.compare_reverse.isChecked(),
                     position_mm=self.compare_mm.value(),orientation_deg=self.compare_deg.value())
        labels=['蓝本' if i==0 else f'修改 {i}' for i in self.compared]
        only_differences=self.compare_only.isChecked()
        display_options=dict(spacing=self.point_spacing.value()*.001,maximum=self.point_limit.value(),
                             arrow_length=self.arrow_length.value()*.001,arrow_diameter=self.arrow_diameter.value()*.001)
        frame_key=(id(trace),tuple(map(id,compared)),side,slot,id(config.get('hands')),
                   tuple(sorted(context.items())),start,end,tuple(key_times),tuple(compared_indices),active_version,
                   tuple(options.items()),only_differences,tuple(display_options.items()))
        def work():
            from task_trace_markers import trace_markers,comparison_markers,hand_context,pick_points
            try:
                cached=self.display_cache.frame(frame_key)
                if cached is not None:
                    self.markers_ready.emit(cached,serial);return
                report=None
                if len(compared)==2:
                    from task_trace_compare import compare
                    result=compare(n.model,*compared,side,hand_context(trace,side,context,config,slot),**options)
                    report=result['report'];report['labels']=labels
                    markers=comparison_markers(result,only_differences=only_differences,**display_options)
                else:
                    geometry=self.display_cache.geometry(n.model,trace,side,context,config,slot)
                    markers=trace_markers(n.model,trace,side,context,start,end,config,slot,key_times,
                                          geometry=geometry,**display_options)
                points=[]
                sources=list(zip(compared,labels)) if len(compared)==2 else [(trace,'当前版本')]
                for i,(source,label) in enumerate(sources):
                    source_context=dict(context);source_context.update(source.get('context_joints',{}))
                    picks=pick_points(n.model,source,side,source_context,config,slot,
                                      display_options['spacing'],display_options['maximum'],
                                      geometry=self.display_cache.geometry(n.model,source,side,source_context,config,slot))
                    if len(compared)==2 and only_differences:
                        from task_trace_compare import differing_pick
                        mask=result['differences'][side+'_index_tcp']
                        keep=differing_pick(source,[p['time'] for p in picks],mask,
                            options['mode'],i==1 and options['reverse'])
                        picks=[p for p,selected in zip(picks,keep) if selected]
                    for point in picks:
                        point['label']=label;point['color']=(1.,.85,0.,1.) if i==0 else (.05,.8,1.,1.)
                        point['active']=len(compared)!=2 or compared_indices[i]==active_version
                    points.extend(picks)
                from rclpy.serialization import serialize_message
                result=(serialize_message(markers),report,points)
                self.display_cache.remember_frame(frame_key,result)
                self.markers_ready.emit(result, serial)
            except Exception as error:
                self.markers_failed.emit(str(error), serial)
        self.marker_thread = threading.Thread(target=work, daemon=True)
        self.marker_thread.start()

    def publish_markers(self, result, serial):
        n=self.control.node
        if (serial == self.marker_serial and self.isVisible() and self.control.recording is None and
                n.session.phase not in ('PLANNING','EXECUTION_PREFLIGHT','EXECUTING')):
            blob,report,points=result
            if blob!=self.last_marker_blob:
                self.control.node.marker_pub.publish(blob)
                self.last_marker_blob=blob
            if hasattr(n,'trace_picker'):
                n.trace_picker.show(self,points,lambda point:self.point_picked.emit(point,serial),
                                    diameter=self.point_diameter.value()*.001)
                self.manual.show_handle()
            if report:
                lines=[f'A {report["labels"][0]}（黄） / B {report["labels"][1]}（青）',
                       f'时长 {report["first_seconds"]:.3f} / {report["second_seconds"]:.3f} s；最大关节差 {report["max_joint_delta_rad"]:.4f} rad']
                for hand,label in (('left','左手'),('right','右手')):
                    d=report['hands'][hand]
                    lines.append((f'{label}：最大 {d["max_position_mm"]:.2f} mm / {d["max_orientation_deg"]:.2f}°；差异 {d["different_percent"]:.1f}%')
                                 if d is not None else label+'：缺少手臂关节上下文')
                if self.compare_only.isChecked() and all(d is None or d['different_percent']==0 for d in report['hands'].values()):
                    lines.append('几何差异均在阈值内；重合路径已隐藏')
                self.comparison_status.setText('\n'.join(lines))
            else:self.comparison_status.clear()
            self.error.clear()

    def marker_error(self, text, serial):
        if serial == self.marker_serial:self.error.setText('RViz: '+text)

    def pick_point(self, point, serial):
        n=self.control.node
        if (serial != self.marker_serial or not self.isVisible() or
                self.was_busy or self.control.recording is not None or (n.worker and n.worker.is_alive()) or
                n.session.phase in ('PLANNING','EXECUTION_PREFLIGHT','EXECUTING')):return
        self.stop_playback()
        if hasattr(self,'external_pause'):self.external_pause()
        if point.get('active',point['trace'] is self.trace):
            if self.has_timeline:self.seek(round(point['time']*1000))
            elif hasattr(self,'external_seek'):self.external_seek(round(point['time']*1000))
        self.control.editor_pose(point['positions'],point['trace'],point['slot'])
        self.error.setText(f'{point["label"]} / {point["time"]:.3f} s')

    def hideEvent(self, event):
        self.stop_playback(); self.marker_timer.stop();self.switch_timer.stop(); self.marker_serial += 1
        if hasattr(self.control.node,'trace_picker'):self.control.node.trace_picker.clear(self)
        self.control.editor_hide()
        self.last_marker_blob=None
        super().hideEvent(event)

    def showEvent(self, event):
        super().showEvent(event)
        self.schedule_markers()
