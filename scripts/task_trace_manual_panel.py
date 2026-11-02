"""Draft manual points with position-only RViz handles and asynchronous IK."""
import copy
import threading

import numpy as np
from PyQt5 import QtCore, QtWidgets

from task_numeric_input import FreeDoubleSpinBox
from task_trace_manual import point_state, solve_point
from task_trace_versions import EDIT_SLOTS, bank


class ManualPointPanel(QtWidgets.QWidget):
    dragged = QtCore.pyqtSignal(object, int)
    solved = QtCore.pyqtSignal(object, str, int)

    def __init__(self, editor):
        super().__init__()
        self.editor = editor
        self.points = {}; self.active = None; self.region = None
        self.token = 0; self.thread = None; self.last_position = None
        self.dragged.connect(self.solve); self.solved.connect(self.finish)
        layout = QtWidgets.QVBoxLayout(self);layout.setContentsMargins(0,0,0,0)
        layout.addWidget(QtWidgets.QLabel('手动点编辑（锁定原姿态）'))
        row = QtWidgets.QHBoxLayout();layout.addLayout(row)
        self.button(row,'编辑当前点',self.begin_current)
        self.button(row,'在选区中点插入',self.insert_midpoint)
        form = QtWidgets.QFormLayout();layout.addLayout(form)
        self.xyz = [FreeDoubleSpinBox(0.) for _ in range(3)]
        for name,spin in zip(('X (mm)','Y (mm)','Z (mm)'),self.xyz):form.addRow(name,spin)
        self.button(layout,'解算并预览位置',self.solve_input)
        self.list = QtWidgets.QListWidget();self.list.setFixedHeight(85)
        self.list.itemClicked.connect(self.select_item);layout.addWidget(self.list)
        row = QtWidgets.QHBoxLayout();layout.addLayout(row)
        self.button(row,'删除手动点',self.remove)
        self.button(row,'取消草稿',self.reset)
        self.button(layout,'保存手动点为新版本',self.save)
        self.status = QtWidgets.QLabel();self.status.setWordWrap(True);layout.addWidget(self.status)

    def button(self,layout,label,action):
        button=QtWidgets.QPushButton(label)
        button.clicked.connect(lambda:self.editor.window.invoke(lambda:self.invoke(action)));layout.addWidget(button)

    def invoke(self,action):
        try:action()
        except Exception as error:
            self.status.setText('不成功：'+str(error))
            raise

    def context(self):
        return dict(getattr(self.editor.control.node.session,'current',{}) or {})

    def guard(self):
        e=self.editor
        if e.trace is None:raise ValueError('请先选择轨迹')
        e.control.idle()
        if e.was_busy:raise ValueError('当前不能编辑')
        if None not in bank(e.control.config,e.slot())['edits']:
            raise ValueError(f'{EDIT_SLOTS} 个修改位置已满，请先删除一个版本')

    def begin_current(self):
        e=self.editor
        self.begin(e.position if e.has_timeline else e.external_position())

    def insert_midpoint(self):
        a,b=self.editor.cut_range()
        if a>=b:raise ValueError('请先设置有效选区')
        self.begin((a+b)/2)

    def begin(self,stamp):
        self.guard()
        if self.thread is not None and self.thread.is_alive():raise ValueError('正在解算，请稍候')
        e=self.editor;e.timer.stop()
        if hasattr(e,'external_pause'):e.external_pause()
        duration=e.trace['raw']['time_s'][-1]
        stamp=float(np.clip(stamp,0.,duration))
        if self.region is None:
            a,b=e.cut_range()
            self.region=(a,b) if a < stamp < b else (max(0.,stamp-.5),min(duration,stamp+.5))
        a,b=self.region
        if not a<=stamp<=b:raise ValueError('新点须位于当前草稿区间内；保存或取消后可换区间')
        if stamp in (a,b) and stamp not in (0.,duration):
            raise ValueError('内部选区边界保持原位，请选择区间内部的时间点')
        _,pose=point_state(e.control.node.model,e.trace,e.control.editor_side(),self.context(),
                           e.control.config,e.slot(),stamp)
        self.token+=1;self.active=stamp
        point=self.points.get(stamp)
        self.last_position=np.array(point['position'] if point else pose[:3,3])
        self.set_xyz(self.last_position)
        if e.has_timeline:e.seek(round(stamp*1000))
        else:e.external_seek(round(stamp*1000))
        if point:e.control.editor_pose(point['joints'],e.trace,e.slot(),hand_time=stamp)
        self.status.setText(f'编辑 {stamp:.3f} s；区间 {a:.3f} 至 {b:.3f} s；尚未保存')
        self.show_handle()

    def set_xyz(self,position):
        for spin,value in zip(self.xyz,position):spin.setValue(float(value)*1000)

    def show_handle(self):
        e=self.editor
        if not hasattr(e.control.node,'trace_picker'):return
        token=self.token
        position=self.last_position if self.active is not None and e.isVisible() and not e.was_busy else None
        e.control.node.trace_picker.move_handle(e,position,
            lambda xyz:self.dragged.emit(xyz,token),e.point_diameter.value()*.001)

    def solve_input(self):self.solve([spin.value()*.001 for spin in self.xyz],self.token)

    def solve(self,position,token):
        if token!=self.token:return
        try:
            self.guard()
            if self.active is None:raise ValueError('请先点击“编辑当前点”或插入中点')
            if self.thread is not None and self.thread.is_alive():
                raise ValueError('正在解算，请稍候再移动')
        except Exception as error:
            self.status.setText(str(error));self.show_handle();return
        e=self.editor
        trace,config,slot,stamp=e.trace,e.control.config,e.slot(),self.active
        revision=e.control.revision
        model,side,context=e.control.node.model,e.control.editor_side(),self.context()
        self.status.setText('正在解算；未发送真机指令')
        def work():
            try:
                point=solve_point(model,trace,side,context,config,slot,stamp,position)
                self.solved.emit((point,trace,revision),'',token)
            except Exception as error:self.solved.emit(None,'不成功：'+str(error).removeprefix('不成功：'),token)
        self.thread=threading.Thread(target=work,daemon=True);self.thread.start()

    def finish(self,result,error,token):
        e=self.editor
        if token!=self.token or e.was_busy or not e.isVisible():return
        if error:
            self.status.setText(error+'；保留上次有效位置')
            self.set_xyz(self.last_position);self.show_handle();return
        point,trace,revision=result
        if (e.trace is not trace or e.control.revision!=revision or e.control.recording is not None or
                e.control.node.session.phase in ('PLANNING','EXECUTION_PREFLIGHT','EXECUTING')):return
        self.points[point['time']]=point
        self.last_position=np.array(point['position']);self.set_xyz(self.last_position)
        self.refresh_list();e.control.editor_pose(point['joints'],trace,e.slot(),hand_time=point['time'])
        self.status.setText(f'解算成功：{point["time"]:.3f} s；草稿 {len(self.points)} 个点，尚未保存')
        self.show_handle()

    def refresh_list(self):
        self.list.clear()
        for stamp,point in sorted(self.points.items()):
            xyz=np.array(point['position'])*1000
            item=QtWidgets.QListWidgetItem(f'{stamp:.3f} s | {xyz[0]:.2f}, {xyz[1]:.2f}, {xyz[2]:.2f} mm')
            item.setData(QtCore.Qt.UserRole,stamp);self.list.addItem(item)

    def select_item(self,item):self.editor.window.invoke(lambda:self.begin(item.data(QtCore.Qt.UserRole)))

    def remove(self):
        item=self.list.currentItem()
        if item is None:return
        stamp=item.data(QtCore.Qt.UserRole);self.points.pop(stamp,None)
        self.token+=1;self.active=None;self.refresh_list();self.show_handle()
        self.status.setText('已移除草稿点；尚未保存')

    def reset(self):
        self.token+=1;self.points={};self.region=None;self.active=None;self.last_position=None
        self.refresh_list();self.status.clear();self.show_handle()

    def save(self):
        self.guard()
        if self.thread is not None and self.thread.is_alive():raise ValueError('正在解算，请稍候')
        if not self.points:raise ValueError('尚无成功解算的手动点')
        self.editor.control.edit_version(self.editor.slot(),'manual',
            points=copy.deepcopy(list(self.points.values())),start=self.region[0],end=self.region[1])

    def draft(self):
        return dict(points=copy.deepcopy(list(self.points.values())),region=self.region)

    def restore(self,data):
        if not isinstance(data,dict) or self.editor.trace is None:return
        region=data.get('region');points=data.get('points')
        duration=self.editor.trace['raw']['time_s'][-1]
        try:
            if region is None or len(region)!=2 or not 0<=region[0]<region[1]<=duration:return
            if not isinstance(points,list):return
            from task_dring_teach_data import moving
            count=len(moving(self.editor.control.editor_side()))
            restored={}
            for point in points:
                stamp=point['time'];q=np.asarray(point['joints']);p=np.asarray(point['position'])
                if (q.shape!=(count,) or p.shape!=(3,) or not np.isfinite(np.r_[q,p,stamp]).all() or
                        not region[0]<=stamp<=region[1]):return
                restored[float(stamp)]=copy.deepcopy(point)
            self.points=restored;self.region=tuple(region);self.refresh_list()
            if restored:self.status.setText(f'已恢复 {len(restored)} 个手动草稿点；尚未保存为轨迹版本')
        except (TypeError,ValueError,KeyError):return
