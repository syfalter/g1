"""Offline composition controls and time-accurate RViz playback."""
import copy
from pathlib import Path
import time

import numpy as np
from PyQt5 import QtCore, QtGui, QtWidgets

from task_composition_preview import prepare, select, materialize, at
from task_trajectory import time_seconds

PHASE_LABELS={
    'entry_lower':'准备位至下杆位','entry_upper':'准备位至上杆位',
    'cycle_lower':'下杆起始往返','cycle_upper':'上杆起始往返','lu':'下杆至上杆','ul':'上杆至下杆',
    'exit_lower':'下杆位退出','exit_upper':'上杆位退出',
    'entry':'拉环接近','cycle':'左拨右拨循环','final':'末次左拨右拨及退出',
    'above_to_ready':'桌上方至准备位','ready_to_left':'准备位至左拨起点',
    'left_to_right':'左拨及右拨衔接','right_to_left':'右拨及左拨衔接',
    'right_with_exit_tail':'末次右拨含退出尾段','return_above':'返回桌上方',
    'approach_lower':'接近下杆位','above_to_lower':'下降至下杆位','engage_lower':'下杆位对接',
    'approach_upper':'接近上杆位','above_to_upper':'进入上杆位','engage_upper':'上杆位对接',
    'lower_to_upper':'下杆至上杆','upper_to_lower':'上杆至下杆',
    'align_lower':'下杆位衔接','align_upper':'上杆位衔接',
    'disengage_lower':'下杆位脱离','disengage_upper':'上杆位脱离',
    'lower_to_above':'下杆退出','upper_to_above':'上杆退出',
    'return_lower':'下杆退出后归位','return_upper':'上杆退出后归位',
    'grip_wait':'握紧等待','open_wait':'松开等待',
    'entry_lower_continuous':'连续接近下杆位','entry_upper_continuous':'连续接近上杆位',
    'lu_continuous':'连续上杆及对齐','ul_continuous':'连续下杆及对齐',
    'exit_lower_continuous':'下杆位连续退出','exit_upper_continuous':'上杆位连续退出',
}


def sample_rows(trajectory, times, *, derivatives=False):
    from task_trajectory import coefficients
    knots=np.array([time_seconds(p) for p in trajectory.points])
    owners=np.clip(np.searchsorted(knots,times,side='right')-1,0,len(knots)-2)
    rows=np.empty((len(times),len(trajectory.joint_names)));speeds=np.empty_like(rows)
    for i in np.unique(owners):
        mask=owners==i;dt=knots[i+1]-knots[i]
        poly=coefficients(trajectory.points[i],trajectory.points[i+1],dt)
        u=(times[mask]-knots[i])/dt
        rows[mask]=np.polynomial.polynomial.polyval(u,poly).T
        speeds[mask]=(np.polynomial.polynomial.polyval(u,np.polynomial.polynomial.polyder(poly))/dt).T
    return rows,speeds if derivatives else np.max(np.abs(speeds),axis=1)


def visible_joint_values(model, values):
    names={joint.get('name') for joint in model.robot.findall('joint')
           if joint.get('type') in ('revolute','continuous','prismatic')}
    return {name:float(value) for name,value in values.items() if name in names}


def native_playback_message(result, position, model):
    from moveit_msgs.msg import DisplayTrajectory,RobotTrajectory
    from trajectory_msgs.msg import JointTrajectory
    from task_motion_planner import robot_state
    from task_composition_preview import stamp
    from task_hand_control import model_joints
    source=result['trajectory'];first=at(source,position)
    points=[first]+[copy.deepcopy(p) for p in source.points if time_seconds(p)>position+1e-9]
    times=np.array([time_seconds(p) for p in points])
    names=list(source.joint_names)
    hands=hand_values(result['payload'],result['events'],times)
    if hands is not None:
        from task_hand_control import OPEN,CLOSED,ORDER
        hand_speed=hand_values(result['payload'],result['events'],times,order=1)
        hand_accel=hand_values(result['payload'],result['events'],times,order=2)
        factors={f'right_{name}_joint':(i,o-c) for i,(name,o,c) in enumerate(zip(ORDER,OPEN,CLOSED))}
        mapped=[visible_joint_values(model,model_joints('right',q)) for q in hands]
        extra=[name for name in mapped[0] if name not in names];names+=extra
        for i,(point,q) in enumerate(zip(points,mapped)):
            point.positions.extend(q[name] for name in extra)
            point.velocities.extend(float(hand_speed[i,factors[name][0]]*factors[name][1]) for name in extra)
            point.accelerations.extend(float(hand_accel[i,factors[name][0]]*factors[name][1]) for name in extra)
    context=dict(result['payload']['context']);context.update(zip(names,points[0].positions))
    trajectory=JointTrajectory(joint_names=names,points=[stamp(p,t-position) for p,t in zip(points,times)])
    return DisplayTrajectory(trajectory_start=robot_state(visible_joint_values(model,context)),
                             trajectory=[RobotTrajectory(joint_trajectory=trajectory)])


def tcp_velocity(model,link,context,names,positions,velocities):
    """Geometric point Jacobian times qdot, including mimic joint contributions."""
    chain=[];child=link
    while child in model.joints:
        joint=model.joints[child];chain.append(joint);child=joint[0]
    frames=model.batch_transforms([link]+[j[0] for j in chain],context,names,positions)
    target=frames[link][:,:3,3];result=np.zeros_like(target)
    columns={name:i for i,name in enumerate(names)}
    for parent,kind,origin,axis,_,name,multiplier,_ in chain:
        if name not in columns or kind not in ('revolute','continuous','prismatic'):continue
        frame=frames[parent]@origin
        direction=frame[:,:3,:3]@axis
        rate=velocities[:,columns[name]]*multiplier
        contribution=direction if kind=='prismatic' else np.cross(direction,target-frame[:,:3,3])
        result+=contribution*rate[:,None]
    return result


def hand_values(payload, events, times, *, order=0):
    if not events:return None
    if order not in (0,1,2):raise ValueError('Unsupported hand preview derivative')
    presets=payload['handPresets'];previous=np.asarray(presets['open']['feedback_q'])
    result=np.tile(previous,(len(times),1)) if order==0 else np.zeros((len(times),len(previous)))
    for event in events:
        target=np.asarray(presets[event['posture']]['feedback_q'])
        if event['time']==0:
            if order==0:result[:]=target
        else:
            mask=times>=event['time'];u=np.clip(times[mask]-event['time'],0,1)
            blend=(10*u**3-15*u**4+6*u**5 if order==0 else
                   30*u**2*(1-u)**2 if order==1 else 60*u-180*u**2+120*u**3)
            if order==0:blend=np.clip(blend,0.,1.)
            result[mask]=(previous if order==0 else 0.)+blend[:,None]*(target-previous)
        previous=target
    return result


def preview_samples(trajectory, payload, events, model, side):
    duration=time_seconds(trajectory.points[-1])
    times=np.linspace(0.,duration,min(4000,max(200,int(duration*15))))
    rows,rates=sample_rows(trajectory,times,derivatives=True)
    joint=np.max(np.abs(rates),axis=1)
    hands=hand_values(payload,events,times)
    names=list(trajectory.joint_names)
    if hands is not None:
        from task_hand_control import model_joints
        mapped=[model_joints('right',q) for q in hands]
        hand_names=list(mapped[0]);names+=hand_names
        rows=np.column_stack((rows,[[q[key] for key in hand_names] for q in mapped]))
        eps=1e-5
        before=hand_values(payload,events,times-eps);after=hand_values(payload,events,times+eps)
        hand_rates=[]
        for a,b in zip(before,after):
            qa,qb=model_joints('right',a),model_joints('right',b)
            hand_rates.append([(qb[key]-qa[key])/(2*eps) for key in hand_names])
        rates=np.column_stack((rates,hand_rates))
    link=side+'_index_tcp'
    tcp=np.linalg.norm(tcp_velocity(model,link,payload['context'],names,rows,rates),axis=1)*1000
    return times,tcp,joint


class SpeedPlot(QtWidgets.QWidget):
    seek = QtCore.pyqtSignal(float)

    def __init__(self):
        super().__init__();self.setMinimumHeight(180);self.setMinimumWidth(260)
        self.samples=None;self.cursor=0.;self.label='食指速度 mm/s'

    def paintEvent(self,event):
        p=QtGui.QPainter(self);p.fillRect(self.rect(),QtGui.QColor('#fafafa'))
        box=self.rect().adjusted(58,28,-14,-30)
        p.setPen(QtGui.QColor('#777777'));p.drawRect(box);p.drawText(10,18,self.label)
        if self.samples is None:return
        t,v=self.samples;maximum=max(float(np.max(v))*1.05,0.);minimum=min(float(np.min(v))*1.05,0.)
        span=max(maximum-minimum,1e-6);duration=max(float(t[-1]),1e-6)
        p.drawText(3,40,f'{maximum:.2f}');p.drawText(5,box.bottom(),f'{minimum:.2f}')
        p.drawText(box.left(),self.height()-8,'0 s');p.drawText(box.right()-80,self.height()-8,f'{duration:.1f} s')
        if minimum<0<maximum:
            p.setPen(QtGui.QPen(QtGui.QColor('#bbbbbb'),1.,QtCore.Qt.DashLine))
            zero=box.bottom()+minimum/span*box.height()
            p.drawLine(QtCore.QPointF(box.left(),zero),QtCore.QPointF(box.right(),zero))
        path=QtGui.QPainterPath()
        for i,(x,y) in enumerate(zip(t,v)):
            point=QtCore.QPointF(box.left()+x/duration*box.width(),box.bottom()-(y-minimum)/span*box.height())
            path.moveTo(point) if i==0 else path.lineTo(point)
        p.setPen(QtGui.QPen(QtGui.QColor('#137c70'),1.5));p.drawPath(path)
        p.setPen(QtGui.QPen(QtGui.QColor('#c44340'),1.))
        x=box.left()+self.cursor/duration*box.width();p.drawLine(QtCore.QPointF(x,box.top()),QtCore.QPointF(x,box.bottom()))

    def mousePressEvent(self,event):
        if self.samples is not None:
            fraction=np.clip((event.x()-58)/max(1,self.width()-72),0,1)
            self.seek.emit(float(fraction*self.samples[0][-1]))


class CompositionPanel(QtWidgets.QWidget):
    completed=QtCore.pyqtSignal(object)

    def __init__(self,node, *, viewer_only=False):
        super().__init__();self.node=node;self.result=None;self.playing=False;self.position=0.
        self.viewer_only=viewer_only
        self.cancelled=False;self.last_tick=0.;self.worker_active=False
        self.variants=None
        layout=QtWidgets.QVBoxLayout(self);form_body=QtWidgets.QWidget();form=QtWidgets.QFormLayout(form_body)
        if viewer_only:layout.setSizeConstraint(QtWidgets.QLayout.SetMinimumSize)
        form.setContentsMargins(0,0,0,0);layout.addWidget(form_body)
        self.workflow=QtWidgets.QComboBox();self.workflow.addItem('拉环：左拨 → 右拨','dring_demo');self.workflow.addItem('起落架往返','landing_gear')
        self.count=QtWidgets.QSpinBox();self.count.setRange(1,100);self.count.setValue(1)
        self.mode=QtWidgets.QComboBox();self.mode.addItem('原时序 · 限值约束','original');self.mode.addItem('整单元近匀速 · 与原策略择优','cruise')
        self.mode.addItem('分阶段限值内尽快','fastest');self.mode.setCurrentIndex(2)
        self.rate=QtWidgets.QDoubleSpinBox();self.rate.setRange(.1,5.);self.rate.setSingleStep(.1);self.rate.setValue(1.)
        self.transit_rate=QtWidgets.QDoubleSpinBox();self.transit_rate.setRange(.1,1.);self.transit_rate.setSingleStep(.1);self.transit_rate.setValue(1.)
        self.rate.setToolTip('尽快策略：1.0 为当前配置限值内搜索出的最快时序；0.5 为其一半速度')
        self.transit_rate.setToolTip('仅调整接近和退出；插入、拨动及接触状态不明确的衔接使用操作速率')
        self.left=QtWidgets.QComboBox();self.right=QtWidgets.QComboBox()
        self.source_choices_token=None;self.refresh_sources()
        self.start=QtWidgets.QComboBox();self.finish=QtWidgets.QComboBox()
        for widget in (self.start,self.finish):
            widget.addItem('下杆位','LOWER');widget.addItem('上杆位','UPPER')
        self.finish.setCurrentIndex(1)
        self.leave=QtWidgets.QCheckBox('最终退出到桌上方')
        self.smoothing=QtWidgets.QCheckBox('生成偏差约束平滑候选');self.smoothing.setChecked(True)
        self.joint_bound=QtWidgets.QDoubleSpinBox();self.joint_bound.setRange(.1,10.);self.joint_bound.setValue(3.);self.joint_bound.setSuffix(' mrad')
        self.tcp_bound=QtWidgets.QDoubleSpinBox();self.tcp_bound.setRange(.1,10.);self.tcp_bound.setValue(1.);self.tcp_bound.setSuffix(' mm')
        self.smooth_window=QtWidgets.QDoubleSpinBox();self.smooth_window.setDecimals(3);self.smooth_window.setRange(.005,.1);self.smooth_window.setSingleStep(.005);self.smooth_window.setValue(.02);self.smooth_window.setSuffix(' rad')
        self.smooth_window.setToolTip('沿关节空间弧长去抖；姿态偏差同时限制为 0.5 度')
        for label,widget in (('业务',self.workflow),('次数 / 完整往返',self.count),('速度策略',self.mode),
                             ('操作速率 / 旧策略倍率',self.rate),('接近退出速率',self.transit_rate),('左拨选中版本',self.left),('右拨选中版本',self.right),
                             ('往返起始杆位',self.start),('最终杆位',self.finish),('退出',self.leave),
                             ('路径候选',self.smoothing),('最大关节偏差',self.joint_bound),
                             ('最大食指位置偏差',self.tcp_bound),('空间平滑窗口',self.smooth_window)):
            form.addRow(label,widget)
        row=QtWidgets.QHBoxLayout();layout.addLayout(row)
        self.generate=QtWidgets.QPushButton('生成组合预览');self.generate.setIcon(self.style().standardIcon(QtWidgets.QStyle.SP_BrowserReload))
        self.play=QtWidgets.QToolButton();self.play.setIcon(self.style().standardIcon(QtWidgets.QStyle.SP_MediaPlay));self.play.setToolTip('按规划时间播放 / 暂停')
        self.stop=QtWidgets.QToolButton();self.stop.setIcon(self.style().standardIcon(QtWidgets.QStyle.SP_MediaStop));self.stop.setToolTip('取消计算 / 停止预览')
        self.export=QtWidgets.QToolButton();self.export.setIcon(self.style().standardIcon(QtWidgets.QStyle.SP_DialogSaveButton));self.export.setToolTip('保存离线组合候选')
        for button in (self.generate,self.play,self.stop,self.export):row.addWidget(button)
        row.addStretch()
        self.summary=QtWidgets.QLabel('未生成');self.summary.setWordWrap(True);layout.addWidget(self.summary)
        self.variant=QtWidgets.QComboBox();self.variant.addItem('原路径 · 分阶段调速','original');self.variant.addItem('平滑候选 · 分阶段调速','candidate');layout.addWidget(self.variant)
        self.comparison=QtWidgets.QLabel();self.comparison.setWordWrap(True);layout.addWidget(self.comparison)
        self.comparison.setToolTip('偏差相对原拟合路径，在重新调速前计算；不是实物接触误差或碰撞检查')
        self.path_panel=None
        if viewer_only:
            from task_spatial_comparison import SpatialComparison
            self.path_panel=SpatialComparison(node)
            self.view_tabs=QtWidgets.QTabWidget();layout.addWidget(self.view_tabs)
            self.view_tabs.addTab(self.path_panel,'末端轨迹')
            self.path_panel.minimum_height_changed.connect(
                lambda height:self.view_tabs.setMinimumHeight(height+self.view_tabs.tabBar().sizeHint().height()+4))
            speed_body=QtWidgets.QWidget();speed_layout=QtWidgets.QVBoxLayout(speed_body)
            self.view_tabs.addTab(speed_body,'速度 / 导数')
        else:speed_layout=layout
        self.metric=QtWidgets.QComboBox();self.metric.addItem('食指速度 mm/s','tcp');self.metric.addItem('最大关节速度 rad/s','joint');speed_layout.addWidget(self.metric)
        self.joint_axis=QtWidgets.QComboBox();self.joint_axis.addItem('各关节绝对值最大','all')
        self.joint_axis.setToolTip('选择单个关节显示带正负号的曲线，区分加减速与关节切换')
        if viewer_only:speed_layout.addWidget(self.joint_axis)
        else:self.joint_axis.hide()
        self.plot=SpeedPlot();speed_layout.addWidget(self.plot)
        self.slider=QtWidgets.QSlider(QtCore.Qt.Horizontal);self.slider.setRange(0,10000);layout.addWidget(self.slider)
        self.clock=QtWidgets.QLabel('0.00 / 0.00 s');layout.addWidget(self.clock)
        self.table=QtWidgets.QTableWidget(0,3);self.table.setHorizontalHeaderLabels(['单元','复用次数','时长 s'])
        self.table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers);self.table.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.Stretch)
        self.table.setMinimumHeight(180);layout.addWidget(self.table);layout.addStretch()
        self.generate.clicked.connect(self.build);self.play.clicked.connect(self.toggle);self.stop.clicked.connect(self.halt)
        self.export.clicked.connect(self.save);self.completed.connect(self.accept)
        self.metric.currentIndexChanged.connect(self.update_plot);self.plot.seek.connect(self.seek)
        self.joint_axis.currentIndexChanged.connect(self.update_plot)
        self.slider.valueChanged.connect(lambda v:self.seek(v/10000*self.duration()) if self.result else None)
        self.workflow.currentIndexChanged.connect(self.controls)
        self.mode.currentIndexChanged.connect(self.controls)
        self.variant.currentIndexChanged.connect(self.switch_variant)
        self.smoothing.toggled.connect(self.controls);self.smoothing.toggled.connect(self.invalidate)
        for widget in (self.joint_bound,self.tcp_bound,self.smooth_window):widget.valueChanged.connect(self.invalidate)
        for widget in (self.count,self.start,self.finish):widget.valueChanged.connect(self.invalidate) if widget is self.count else widget.currentIndexChanged.connect(self.invalidate)
        for widget in (self.workflow,self.mode,self.left,self.right):widget.currentIndexChanged.connect(self.invalidate)
        self.rate.valueChanged.connect(self.invalidate);self.transit_rate.valueChanged.connect(self.invalidate);self.leave.toggled.connect(self.invalidate)
        self.timer=QtCore.QTimer(self);self.timer.setInterval(40);self.timer.timeout.connect(self.tick);self.timer.start()
        self.controls();self.refresh(False)
        if viewer_only:
            form_body.hide()
            self.generate.hide()
            self.variant.clear()
            for label,key in (('原路径 · jerk 5','original'),('原路径 · 所选 jerk','baseline'),('路径拟合择优 · 所选 jerk','candidate')):
                self.variant.addItem(label,key)
            self.metric.addItem('最大关节加速度 rad/s²','acceleration')
            self.metric.addItem('最大关节 jerk rad/s³','jerk')

    def controls(self):
        if self.viewer_only:return
        gear=self.workflow.currentData()=='landing_gear'
        self.variant.blockSignals(True)
        if gear and self.variant.findData('baseline')<0:
            self.variant.addItem('上一版连续路径候选','baseline')
        elif not gear and self.variant.findData('baseline')>=0:
            self.variant.removeItem(self.variant.findData('baseline'))
        self.variant.setItemText(1,'V2 · 进入下杆低速谷优化' if gear else '平滑候选 · 分阶段调速')
        self.variant.blockSignals(False)
        self.smoothing.setText('生成偏差约束连续路径候选' if gear else '生成偏差约束平滑候选')
        fast=self.mode.currentData()=='fastest'
        self.rate.setMaximum(1. if fast else 5.)
        self.transit_rate.setEnabled(fast and not self.worker_active)
        self.smoothing.setEnabled(fast and not self.worker_active)
        for w in (self.joint_bound,self.tcp_bound,self.smooth_window):w.setEnabled(fast and self.smoothing.isChecked() and not self.worker_active)
        self.transit_rate.setToolTip('连续接近/退出含对接段时，采用操作与接近退出速率的较小值' if gear
                                    else '仅调整接近和退出；接触状态不明确的衔接使用操作速率')
        for w in (self.left,self.right):w.setEnabled(not gear and not self.worker_active)
        for w in (self.start,self.finish,self.leave):w.setEnabled(gear and not self.worker_active)

    def refresh_sources(self):
        from task_trace_versions import bank
        control=self.node.demo
        token=(id(control.config),getattr(control,'revision',0))
        if token==self.source_choices_token:return
        self.source_choices_token=token
        for direction,widget in (('left',self.left),('right',self.right)):
            previous=widget.currentData();widget.blockSignals(True);widget.clear()
            preferred=None
            for slot,trace in control.config['traces'].items():
                if not slot.startswith(direction+'_'):continue
                widget.addItem(slot+' · v'+str(bank(control.config,slot)['selected']),slot)
                if preferred is None and trace.get('optimization',{}).get('kind')=='trim_tail':preferred=slot
            index=widget.findData(previous if previous is not None else preferred)
            if index>=0:widget.setCurrentIndex(index)
            widget.blockSignals(False)

    def invalidate(self,*args):
        if self.playing:self.clear_native()
        if self.path_panel:self.path_panel.clear()
        self.variants=None;self.comparison.clear()
        self.playing=False;self.result=None;self.plot.samples=None;self.plot.update();self.summary.setText('参数已变化，未生成')

    def hideEvent(self,event):
        if self.path_panel:self.path_panel.clear_markers()
        super().hideEvent(event)

    def showEvent(self,event):
        super().showEvent(event)
        if self.path_panel:self.path_panel.publish()

    def switch_variant(self,*args):
        if self.variants:
            self.clear_native();self.playing=False
            self.accept(self.variants[self.variant.currentData()])

    def build(self):
        n=self.node
        try:
            if not n.args.disconnected:raise ValueError('组合预览仅允许离线环境')
            self.clear_native();self.variants=None;self.comparison.clear()
            self.playing=False;self.result=None;self.cancelled=False
            workflow=self.workflow.currentData();control=n.demo if workflow=='dring_demo' else n.landing_gear
            with n.lock:
                source=control.config;revision=control.revision
                ready=copy.deepcopy(n.prepared)
            options=dict(leftSlot=self.left.currentData(),rightSlot=self.right.currentData()) if workflow=='dring_demo' else {}
            mode,multiplier=self.mode.currentData(),self.rate.value()
            if mode=='fastest':options['transitRate']=self.transit_rate.value()
            smoothing=(dict(smooth=True,jointTolerance=self.joint_bound.value()*.001,
                            tcpTolerance=self.tcp_bound.value()*.001,smoothWindow=self.smooth_window.value())
                       if mode=='fastest' and self.smoothing.isChecked() else None)
            count,start,finish,leave=self.count.value(),self.start.currentData(),self.finish.currentData(),self.leave.isChecked()
            self.worker_active=True
            def check():
                if self.cancelled or n.shutdown_requested or control.config is not source or control.revision!=revision:
                    raise RuntimeError('组合计算已取消或源版本变化')
            def work():
                try:
                    from task_planning_config import load_parameters,DEFAULT_CONFIG
                    began=time.monotonic();params,_=load_parameters(getattr(n.args,'planning_config',DEFAULT_CONFIG))
                    check();config=copy.deepcopy(source)
                    variants={}
                    requests=[('original',{})]
                    if smoothing:
                        if workflow=='landing_gear':
                            requests += [('baseline',smoothing),('candidate',dict(smoothing,entryValleyRefit=True))]
                        else:requests.append(('candidate',smoothing))
                    for label,extra in requests:
                        payload,hit,path=prepare(workflow,config,ready,n.model,params,mode,multiplier,dict(options,**extra),check,
                                                 progress=lambda text:setattr(n,'status',label+'：'+text))
                        sequence=select(payload,count,start,finish,leave)
                        trajectory,events=materialize(payload,sequence);check()
                        times,tcp,joint=preview_samples(trajectory,payload,events,n.model,config['side'])
                        check();variants[label]=dict(payload=payload,sequence=sequence,trajectory=trajectory,events=events,
                            cache=str(path),hit=hit,elapsed=time.monotonic()-began,times=times,tcp=tcp,joint=joint,
                            sourceToken=(id(source),revision))
                    self.completed.emit(dict(variants=variants) if smoothing else variants['original'])
                except Exception as error:self.completed.emit(dict(error=str(error)))
            n.launch_work(work)
        except Exception as error:
            self.worker_active=False;n.status=str(error);self.summary.setText(str(error))

    def accept(self,result):
        self.worker_active=False
        if self.cancelled:
            self.summary.setText('计算已取消');return
        if 'error' in result:self.summary.setText(result['error']);self.node.status=result['error'];return
        if 'variants' in result:
            self.variants=result['variants'];chosen='candidate'
            self.variant.blockSignals(True)
            if self.viewer_only:
                index=self.variant.findData('smooth')
                if 'smooth' in self.variants:
                    if index<0:self.variant.addItem('平顺优先 · V4 稳健样条','smooth')
                    chosen='smooth'
                elif index>=0:self.variant.removeItem(index)
                index=self.variant.findData('flow')
                if 'flow' in self.variants:
                    if index<0:self.variant.addItem('连续通行 · V5 内部去停顿','flow')
                    revision=self.variants['flow']['payload'].get('trajectoryReview',{}).get('revision')
                    self.variant.setItemText(self.variant.findData('flow'),revision or '连续通行 · V5 内部去停顿')
                    chosen='flow'
                elif index>=0:self.variant.removeItem(index)
            result=self.variants[chosen]
            for key,item in self.variants.items():
                label=item['payload'].get('comparisonLabel')
                if label and self.variant.findData(key)>=0:self.variant.setItemText(self.variant.findData(key),label)
            self.variant.setCurrentIndex(self.variant.findData(chosen));self.variant.blockSignals(False)
            before=time_seconds(self.variants['original']['trajectory'].points[-1])
            after=time_seconds(result['trajectory'].points[-1])
            bounds=[p['pathSmoothing'] for u in result['payload']['units'].values()
                    for p in u['report'].get('phases',[]) if 'pathSmoothing' in p]
            label='V2 低速谷候选' if result['payload']['workflow']=='landing_gear' else '平滑候选'
            previous=(f' / 上版候选 {time_seconds(self.variants["baseline"]["trajectory"].points[-1]):.2f} s'
                      if 'baseline' in self.variants else '')
            self.comparison.setText(f'原路径 {before:.2f} s{previous} / {label} {after:.2f} s\n连续偏差上界：'
                f'关节 {max((b["joint_rad"] for b in bounds),default=0)*1000:.3f} mrad · '
                f'食指 {max((b["tcp_m"] for b in bounds),default=0)*1000:.3f} mm · '
                f'姿态 {np.degrees(max((b["orientation_rad"] for b in bounds),default=0)):.3f}°')
        elif self.variants is None:
            self.variant.blockSignals(True);self.variant.setCurrentIndex(0);self.variant.blockSignals(False)
        self.result=result;self.position=0.
        if self.path_panel:
            try:self.path_panel.set_results(self.variants or {'candidate':result})
            except (ValueError,KeyError) as error:
                self.path_panel.clear();self.path_panel.readout.setText('空间轨迹未加载：'+str(error))
        if self.viewer_only:
            selected=self.joint_axis.currentData();self.joint_axis.blockSignals(True);self.joint_axis.clear()
            self.joint_axis.addItem('各关节绝对值最大','all')
            for name in result.get('joint_names',[]):self.joint_axis.addItem(name,name)
            self.joint_axis.setCurrentIndex(max(0,self.joint_axis.findData(selected)));self.joint_axis.blockSignals(False)
        from collections import Counter
        counts=Counter(result['sequence']);rows=[]
        fast=result['payload']['mode']=='fastest'
        if fast:
            phase_counts=Counter();details={}
            for name,count in counts.items():
                unit=result['payload']['units'].get(name)
                if unit is None:
                    rows.append((PHASE_LABELS.get(name,name),'手指等待',str(count),'1.00',''))
                    continue
                for phase in unit['report']['phases']:
                    phase_counts[phase['name']]+=count;details[phase['name']]=phase
            for name,count in phase_counts.items():
                phase=details[name];limit=phase['validation'].get('limiting_constraint') or {}
                constraint=limit.get('constraint','')
                constraint={'jerk':'加加速度','velocity':'速度','acceleration':'加速度'}.get(constraint,constraint)
                category={'transit':'接近退出','operation':'操作','mixed':'联动（取较低速率）'}[phase['motionClass']]
                rows.append((PHASE_LABELS.get(name,name),category,str(count),
                             f'{phase["seconds"]:.2f}',constraint))
            headers=['阶段','类别','次数','单次 s','约束']
        else:
            headers=['单元','复用次数','时长 s']
            for name,count in counts.items():
                seconds=result['payload']['units'].get(name,{}).get('seconds',1.)
                rows.append((PHASE_LABELS.get(name,name),str(count),f'{seconds:.2f}'))
        self.table.setColumnCount(len(headers));self.table.setHorizontalHeaderLabels(headers);self.table.setRowCount(len(rows))
        for row,values in enumerate(rows):
            for column,value in enumerate(values):
                item=QtWidgets.QTableWidgetItem(value);item.setToolTip(value);self.table.setItem(row,column,item)
        reports=[x['report'] for x in result['payload']['units'].values()]
        fallbacks=sum(bool(x.get('timing_comparison',{}).get('fallback')) for x in reports)
        policy='分阶段尽快 · 配置限值 100%' if fast else f'原策略择优 {fallbacks} 段'
        boundary='业务边界停稳' if result['payload'].get('sources',{}).get('continuityPolicy') else '单元边界零速'
        self.summary.setText(f'离线候选 · {"缓存命中" if result["hit"] else "新单元"} · 计算 {result["elapsed"]:.1f} s\n'
                             f'总时长 {self.duration():.2f} s · {boundary} · {policy}\n'
                             f'采样峰值：关节 {max(result["joint"]):.3f} rad/s · 食指 {max(result["tcp"]):.1f} mm/s')
        if self.viewer_only:
            meta=result['payload']['optimizer']
            self.summary.setText(f'离线候选 · {"缓存命中" if result["hit"] else "新计算"} · {result["elapsed"]:.1f} s\n'
                f'总时长 {self.duration():.2f} s · jerk 策略 {meta["jerk"]:g} rad/s³ · 优化路径选用 {meta["changed"]}/{meta["total"]} 单元')
            from task_optimizer_snapshot import describe
            self.summary.setText(self.summary.text()+'\n'+describe(result))
            if 'snapshot_id' in result:
                self.summary.setText(self.summary.text()+f'\n固定结果 {result["snapshot_id"]} · {result["snapshot_created"]}')
            review=result['payload'].get('trajectoryReview')
            if review:
                status={'offline-validated':'离线校验通过 · 未部署',
                        'requires-contact-review':'接触安全待核验 · 未部署 · 禁止执行',
                        'rejected-motion-limits':'导数超限 · 已拒绝执行'}.get(review['status'],review['status'])
                self.summary.setText(self.summary.text()+f'\n{review["revision"]} · {status}')
            if self.variants:
                self.comparison.setText(' / '.join(f'{self.variants[key]["payload"].get("comparisonLabel",label)} {time_seconds(self.variants[key]["trajectory"].points[-1]):.2f} s'
                    for key,label in (('original','原路径 jerk5'),('baseline','原路径同策略'),('candidate','最快择优'),('smooth','平顺 V4'),
                                      ('flow','本轮候选' if self.variants.get('flow',{}).get('payload',{}).get('trajectoryReview') else '连续 V5')) if key in self.variants))
            if meta.get('variant')=='smooth' and meta['changed']<meta['total']:
                self.summary.setText(self.summary.text()+f'\n平顺拟合通过 {meta["changed"]}/{meta["total"]}；其余保留旧候选')
            if meta.get('variant')=='flow':
                before=after=0
                for name,count in counts.items():
                    audit=result['payload']['units'].get(name,{}).get('report',{}).get('stopAudit',{})
                    before+=count*audit.get('before',0);after+=count*audit.get('after',0)
                self.summary.setText(self.summary.text()+f'\n单元内低速谷 {before} → {after}（不含业务等待与单元端点）')
                if meta['changed']<meta['total']:
                    self.summary.setText(self.summary.text()+f'\n去停顿通过 {meta["changed"]}/{meta["total"]}；其余保留旧候选')
            if 'smoothness' in result:
                metrics=result['smoothness']
                self.summary.setText(self.summary.text()+f'\n最大 jerk 跳变 {metrics["max_jerk_jump"]:.5f} rad/s³ · 加速度总变化 {metrics["acceleration_total_variation"]:.2f} rad/s²')
            if result['payload']['sources'].get('origin_workflow')=='rail':
                self.summary.setText(self.summary.text()+'\n滑轨：关节路径实验，未应用生产接触 TCP 速度策略')
        self.node.status='组合候选已生成；未授权真机执行'
        self.update_plot();self.show_pose();self.refresh(False)

    def duration(self):return time_seconds(self.result['trajectory'].points[-1]) if self.result else 0.

    def update_plot(self,*args):
        label=self.metric.currentText()
        if self.result:
            key=self.metric.currentData();values=self.result[key]
            name=self.joint_axis.currentData()
            if self.viewer_only and key!='tcp' and name!='all' and key+'_matrix' in self.result:
                values=self.result[key+'_matrix'][:,self.result['joint_names'].index(name)]
                label=name+' · '+{'joint':'rad/s','acceleration':'rad/s²','jerk':'rad/s³'}[key]
            self.plot.samples=(self.result['times'],values)
        self.plot.label=label;self.plot.update()

    def show_pose(self):
        if not self.result:return
        from moveit_msgs.msg import DisplayRobotState
        from task_motion_planner import robot_state
        from task_hand_control import model_joints
        q=dict(self.result['payload']['context']);point=at(self.result['trajectory'],self.position)
        q.update(zip(self.result['trajectory'].joint_names,point.positions))
        hands=hand_values(self.result['payload'],self.result['events'],np.array([self.position]))
        if hands is not None:q.update(model_joints('right',hands[0]))
        q=visible_joint_values(self.node.model,q)
        self.node.demo_preview_pub.publish(DisplayRobotState(state=robot_state(q)))
        self.update_position_widgets()

    def update_position_widgets(self):
        self.clock.setText(f'{self.position:.2f} / {self.duration():.2f} s')
        self.slider.blockSignals(True);self.slider.setValue(round(10000*self.position/max(self.duration(),1e-9)));self.slider.blockSignals(False)
        self.plot.cursor=self.position;self.plot.update()

    def clear_native(self):
        from moveit_msgs.msg import DisplayTrajectory
        self.node.display_pub.publish(DisplayTrajectory())

    def seek(self,seconds):
        self.playing=False;self.clear_native();self.position=seconds;self.show_pose()

    def toggle(self):
        if not self.result:return
        if self.playing:
            self.playing=False;self.clear_native();self.show_pose();return
        if self.position>=self.duration():self.position=0.
        from moveit_msgs.msg import DisplayRobotState
        message=native_playback_message(self.result,self.position,self.node.model)
        self.node.demo_preview_pub.publish(DisplayRobotState(hide=True))
        self.node.display_pub.publish(message)
        self.playing=True;self.last_tick=time.monotonic()

    def tick(self):
        if not self.isVisible():
            if self.playing:self.clear_native()
            self.playing=False;return
        if not self.playing:return
        now=time.monotonic();self.position=min(self.duration(),self.position+now-self.last_tick);self.last_tick=now
        self.update_position_widgets()
        if self.position>=self.duration():
            self.playing=False;self.clear_native();self.show_pose()

    def halt(self):
        self.cancelled=True;self.playing=False
        if self.path_panel:self.path_panel.clear()
        self.clear_native()
        from moveit_msgs.msg import DisplayRobotState
        self.node.demo_preview_pub.publish(DisplayRobotState(hide=True))

    def refresh(self,busy):
        if not self.viewer_only:self.refresh_sources()
        if self.result and 'sourceToken' in self.result:
            control=self.node.demo if self.result['payload']['workflow']=='dring_demo' else self.node.landing_gear
            if self.result['sourceToken']!=(id(control.config),control.revision):self.invalidate()
        for w in (self.workflow,self.count,self.mode,self.rate,self.generate):w.setEnabled(not busy and not self.worker_active)
        self.controls()
        if busy:
            for w in (self.left,self.right,self.start,self.finish,self.leave,self.transit_rate,self.smoothing,self.joint_bound,self.tcp_bound,self.smooth_window):w.setEnabled(False)
        self.variant.setEnabled(bool(self.variants) and not busy and not self.worker_active)
        self.joint_axis.setEnabled(self.result is not None and not busy and self.metric.currentData()!='tcp')
        self.play.setIcon(self.style().standardIcon(QtWidgets.QStyle.SP_MediaPause if self.playing else QtWidgets.QStyle.SP_MediaPlay))
        for w in (self.play,self.export,self.slider):w.setEnabled(self.result is not None and not busy and not self.worker_active)

    def save(self):
        if not self.result:return
        from task_screen_teach import write_json,fingerprint
        payload=self.result['payload']
        artifact=dict(schema=1,kind='offline_composition_candidate',units=payload,
                      sequence=self.result['sequence'],handEvents=self.result['events'],
                      cache=self.result['cache'],hardware_executable=False,physical_verified=False)
        path=Path(__file__).resolve().parents[1]/'data/composition_candidates'/(fingerprint(artifact)+'.json')
        write_json(path,artifact);self.node.status='组合候选已保存：'+str(path)
