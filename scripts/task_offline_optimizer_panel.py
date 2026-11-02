"""Offline-only optimization comparison; never installs execution candidates."""
import copy
import json
from pathlib import Path
import time

from PyQt5 import QtCore,QtWidgets

from task_composition_panel import CompositionPanel,preview_samples
from task_composition_preview import materialize
from task_offline_optimizer import Policy
from task_offline_optimizer_cases import comparison,sequence_for


def data_index(widget,value):
    # Qt QVariant matching does not compare Python tuple userData by value.
    if isinstance(value,list):value=tuple(value)
    return next((i for i in range(widget.count()) if widget.itemData(i)==value),-1)


class OptimizerPanel(QtWidgets.QWidget):
    completed=QtCore.pyqtSignal(object)

    def __init__(self,node,*,snapshot_directory=None):
        super().__init__();self.node=node;self.active=False;self.cancelled=False;self.source_token=None
        from task_optimizer_snapshot import DIRECTORY
        self.snapshot_directory=snapshot_directory or DIRECTORY
        layout=QtWidgets.QVBoxLayout(self)
        header=QtWidgets.QFormLayout();layout.addLayout(header)
        form_body=QtWidgets.QWidget();form=QtWidgets.QFormLayout(form_body)
        self.case=QtWidgets.QComboBox()
        self.case.addItem('拉环：左拨后右拨组合',('dring_demo',None))
        self.case.addItem('起落架：连续往返组合',('landing_gear',None))
        self.case.addItem('滑轨：完整推进（接近、推进、退出、返回）',('rail','push_cycle'))
        self.case.addItem('滑轨：完整拉出（接近、拉出、退出、返回）',('rail','pull_cycle'))
        for workflow,label,control in (('dring_demo','拉环',node.demo),('landing_gear','起落架',node.landing_gear),('rail','滑轨',node.rail)):
            for slot in control.config.get('traces',{}):self.case.addItem(label+' · '+slot,(workflow,slot))
        self.comparison_set=QtWidgets.QComboBox();self.comparison_set.addItem('有界优化',None);self.experiment_case=None
        header.addRow('任务 / 当前选中单段',self.case);header.addRow('对比组',self.comparison_set)
        self.parameters_toggle=QtWidgets.QToolButton();self.parameters_toggle.setText('优化参数')
        self.parameters_toggle.setToolButtonStyle(QtCore.Qt.ToolButtonTextBesideIcon)
        self.parameters_toggle.setArrowType(QtCore.Qt.RightArrow);self.parameters_toggle.setCheckable(True)
        layout.addWidget(self.parameters_toggle);layout.addWidget(form_body);form_body.hide()
        self.parameters_toggle.toggled.connect(form_body.setVisible)
        self.parameters_toggle.toggled.connect(lambda open_:self.parameters_toggle.setArrowType(QtCore.Qt.DownArrow if open_ else QtCore.Qt.RightArrow))
        self.jerk=QtWidgets.QComboBox()
        for value in (5.,10.,20.):self.jerk.addItem(f'{value:g} rad/s³ · 离线策略',value)
        self.jerk.setToolTip('5 为项目默认值，不是已证实的电机上限；10/20 仅用于离线比较')
        self.contact=QtWidgets.QDoubleSpinBox();self.contact.setRange(.1,5.);self.contact.setValue(1.);self.contact.setSuffix(' mm')
        self.transit=QtWidgets.QDoubleSpinBox();self.transit.setRange(.1,20.);self.transit.setValue(5.);self.transit.setSuffix(' mm')
        self.rate=QtWidgets.QDoubleSpinBox();self.rate.setRange(.1,1.);self.rate.setSingleStep(.1);self.rate.setValue(1.)
        self.smooth_first=QtWidgets.QCheckBox('生成平顺优先候选');self.smooth_first.setChecked(True)
        self.continuous_flow=QtWidgets.QCheckBox('消除非必要内部停顿');self.continuous_flow.setChecked(True)
        self.extra=QtWidgets.QDoubleSpinBox();self.extra.setRange(0,50);self.extra.setValue(15);self.extra.setSuffix(' %')
        self.extra.setToolTip('平顺候选相对现有最快单元允许增加的时长；不达标时保留旧候选并标明')
        self.count=QtWidgets.QSpinBox();self.count.setRange(1,100)
        self.start=QtWidgets.QComboBox();self.finish=QtWidgets.QComboBox()
        for w in (self.start,self.finish):w.addItem('下杆位','LOWER');w.addItem('上杆位','UPPER')
        self.finish.setCurrentIndex(1);self.leave=QtWidgets.QCheckBox('结束后退出');self.leave.setChecked(True)
        self.manual=QtWidgets.QCheckBox('指定单段的无接触区间（原轨迹秒）')
        self.begin=QtWidgets.QDoubleSpinBox();self.end=QtWidgets.QDoubleSpinBox()
        for w in (self.begin,self.end):w.setRange(0,10000);w.setDecimals(3);w.setSuffix(' s')
        self.end.setValue(1.)
        self.left=QtWidgets.QComboBox();self.right=QtWidgets.QComboBox()
        self.trim_token=None;self.refresh_trims()
        for label,w in (('左拨截尾版本',self.left),('右拨截尾版本',self.right),
                        ('jerk',self.jerk),('接触位置偏差',self.contact),('接近退出位置偏差',self.transit),
                        ('速度比例',self.rate),('平顺优化',self.smooth_first),('内部连续性',self.continuous_flow),('最大额外时长',self.extra),
                        ('次数 / 完整往返',self.count),('起始',self.start),('结束',self.finish),
                        ('退出',self.leave),('无接触标记',self.manual),('区间开始',self.begin),('区间结束',self.end)):
            form.addRow(label,w)
        self.generate=QtWidgets.QPushButton('生成优化对比');self.generate.setIcon(self.style().standardIcon(QtWidgets.QStyle.SP_BrowserReload));layout.addWidget(self.generate)
        self.viewer=CompositionPanel(node,viewer_only=True);layout.addWidget(self.viewer)
        self.generate.clicked.connect(self.build);self.completed.connect(self.accept)
        self.viewer.stop.clicked.connect(self.cancel)
        self.case.currentIndexChanged.connect(self.case_changed)
        self.comparison_set.currentIndexChanged.connect(self.experiment_changed)
        for w in (self.jerk,self.start,self.finish,self.left,self.right):w.currentIndexChanged.connect(self.invalidate)
        for w in (self.contact,self.transit,self.rate,self.count,self.begin,self.end,self.extra):w.valueChanged.connect(self.invalidate)
        for w in (self.manual,self.leave,self.smooth_first,self.continuous_flow):w.toggled.connect(self.invalidate)
        self.refresh(False)

    def refresh_trims(self):
        from task_trace_versions import bank
        control=self.node.demo;token=(id(control.config),control.revision)
        if token==self.trim_token:return
        self.trim_token=token
        for direction,widget in (('left',self.left),('right',self.right)):
            previous=widget.currentData();widget.blockSignals(True);widget.clear()
            for slot in control.config.get('traces',{}):
                if not slot.startswith(direction+'_'):continue
                entry=bank(control.config,slot)
                for i,trace in enumerate([entry['baseline']]+entry['edits']):
                    if trace and trace.get('optimization',{}).get('kind')=='trim_tail':
                        widget.addItem(f'{slot} · v{i}',(slot,i))
            index=data_index(widget,previous)
            if index>=0:widget.setCurrentIndex(index)
            elif widget.count():widget.setCurrentIndex(widget.count()-1)
            widget.blockSignals(False)

    def control(self,workflow):
        return {'dring_demo':self.node.demo,'landing_gear':self.node.landing_gear,'rail':self.node.rail}[workflow]

    def invalidate(self,*args):
        self.viewer.invalidate();self.source_token=None

    def case_changed(self,*args):
        self.invalidate()
        self.experiment_options()
        self.restore_case()

    def experiment_options(self):
        from task_optimizer_snapshot import case_key
        token=(str(self.snapshot_directory),self.case.currentData())
        if token==self.experiment_case:return
        self.experiment_case=token
        blocker=QtCore.QSignalBlocker(self.comparison_set)
        self.comparison_set.clear();self.comparison_set.addItem('有界优化',None)
        try:
            entries=json.loads((Path(self.snapshot_directory)/'experiments.json').read_text()).get(case_key(*self.case.currentData()),[])
            for item in entries:
                name=item['file']
                if Path(name).name!=name or not name.endswith('.json'):raise ValueError('Invalid experiment snapshot path')
                self.comparison_set.addItem(item['label'],name)
        except FileNotFoundError:pass
        except (ValueError,KeyError,TypeError) as error:self.node.status='对比组未加载：'+str(error)
        del blocker

    def experiment_changed(self,*args):
        self.invalidate();self.restore_case()

    def restore_startup(self):
        from task_optimizer_snapshot import latest,load
        try:
            path=latest(self.snapshot_directory)
            if path is None:return False
            saved=load(path,self.node.model)
            index=data_index(self.case,saved['snapshot_controls']['case'])
            if index<0:return False
            blocker=QtCore.QSignalBlocker(self.case);self.case.setCurrentIndex(index);del blocker
            self.experiment_options()
            index=self.comparison_set.findData(path.name)
            if index>=0:
                blocker=QtCore.QSignalBlocker(self.comparison_set);self.comparison_set.setCurrentIndex(index);del blocker
            return self.restore_case()
        except (OSError,ValueError,KeyError,TypeError) as error:
            self.viewer.summary.setText('优化结果未加载：'+str(error));return False

    def snapshot_controls(self):
        return dict(case=list(self.case.currentData()),jerk=self.jerk.currentData(),
            contact=self.contact.value(),transit=self.transit.value(),rate=self.rate.value(),
            smooth_first=self.smooth_first.isChecked(),extra=self.extra.value(),count=self.count.value(),
            continuous_flow=self.continuous_flow.isChecked(),
            start=self.start.currentData(),finish=self.finish.currentData(),leave=self.leave.isChecked(),
            manual=self.manual.isChecked(),begin=self.begin.value(),end=self.end.value(),
            left=self.left.currentData(),right=self.right.currentData())

    def restore_case(self):
        from task_optimizer_snapshot import for_case,load
        if self.active:return False
        try:
            self.experiment_options()
            workflow,slot=self.case.currentData();name=self.comparison_set.currentData()
            path=Path(self.snapshot_directory)/name if name else for_case(workflow,slot,self.snapshot_directory)
            if path is None:
                self.viewer.summary.setText('此任务尚无已保存的优化结果');return False
            result=load(path,self.node.model)
            if result['snapshot_controls']['case']!=[workflow,slot]:raise ValueError('对比快照与当前任务不匹配')
            controls=dict(result['snapshot_controls']);controls.setdefault('continuous_flow',False);control=self.control(workflow)
            from task_trace_versions import bank
            from task_screen_teach import fingerprint
            selected=next(iter(result['variants'].values()))['payload']['sources']
            rail_cycle=workflow=='rail' and slot in ('push_cycle','pull_cycle')
            if rail_cycle:
                for name,source in selected.items():
                    if source['selectedSha256']!=fingerprint(control.config['traces'].get(name)):
                        raise ValueError('滑轨轨迹已变化，没有匹配的完整组合结果')
            if slot and not rail_cycle and selected.get('selectedVersion')!=bank(control.config,slot)['selected']:
                raise ValueError('当前选中版本已变化，没有匹配的优化结果；未加载旧版本')
            if slot and not rail_cycle and selected.get('selectedSha256') and selected['selectedSha256']!=fingerprint(control.config['traces'][slot]):
                raise ValueError('当前轨迹内容已变化，没有匹配的优化结果')
            if workflow=='landing_gear' and slot is None:
                for name,source in selected.items():
                    if isinstance(source,dict) and 'sha256' in source and source['sha256']!=fingerprint(control.config['traces'].get(name)):
                        raise ValueError('起落架轨迹版本已变化，没有匹配的组合结果')
            if workflow=='dring_demo' and slot is None:
                for direction in ('left','right'):
                    source=selected[direction];entry=bank(control.config,source['slot'])
                    for version_key,hash_key in (('selectedVersion','selectedSha256'),('parentVersion','parentSha256')):
                        version=source[version_key]
                        trace=entry['baseline'] if version==0 else entry['edits'][version-1]
                        if fingerprint(trace)!=source[hash_key]:
                            raise ValueError('拉环截尾或退出父版本已变化，没有匹配的组合结果')
            widgets=[getattr(self,name) for name in controls if name!='case']
            blockers=[QtCore.QSignalBlocker(widget) for widget in widgets]
            for name,value in controls.items():
                if name=='case':continue
                widget=getattr(self,name)
                if isinstance(widget,QtWidgets.QComboBox):
                    index=data_index(widget,value)
                    if index<0 and value is not None:raise ValueError('已保存结果的版本选项已不存在')
                    widget.setCurrentIndex(index)
                elif isinstance(widget,QtWidgets.QCheckBox):widget.setChecked(value)
                else:widget.setValue(value)
            del blockers
            self.cancelled=False;self.viewer.cancelled=False;self.viewer.clear_native()
            self.viewer.accept(result)
            self.source_token=(workflow,id(control.config),control.revision)
            self.refresh(False)
            self.node.status='已加载固定优化结果，可直接播放；未重新规划'
            return True
        except (OSError,ValueError,KeyError,TypeError) as error:
            self.viewer.invalidate();self.viewer.summary.setText('优化结果未加载：'+str(error));return False

    def cancel(self):
        self.cancelled=True

    def build(self):
        n=self.node
        try:
            if not n.args.disconnected or not n.args.offline_editor:raise ValueError('只允许离线编辑器生成实验候选')
            if self.comparison_set.currentData():raise ValueError('固定轨迹对比只读，不能覆盖生成')
            if self.active:return
            self.viewer.clear_native();self.viewer.invalidate();self.cancelled=False;self.viewer.cancelled=False
            workflow,slot=self.case.currentData();control=self.control(workflow)
            with n.lock:
                source=control.config;revision=control.revision;ready=copy.deepcopy(n.prepared)
            config=copy.deepcopy(source);options={}
            if self.smooth_first.isChecked():options.update(smoothFirst=True,smoothExtraFraction=self.extra.value()/100.)
            if self.continuous_flow.isChecked():options.update(continuousFlow=True,smoothExtraFraction=self.extra.value()/100.)
            if workflow=='rail' and slot in ('push_cycle','pull_cycle'):
                options['railCycle']=slot
            elif slot:
                options['slot']=slot
                if self.manual.isChecked():options['transitInterval']=(self.begin.value(),self.end.value())
            elif workflow=='dring_demo':
                from task_trace_versions import select_shared
                for direction,widget in (('left',self.left),('right',self.right)):
                    if widget.currentData() is None:raise ValueError('缺少'+direction+'截尾版本')
                    chosen,version=widget.currentData();config=select_shared(config,chosen,version);options[direction+'Slot']=chosen
            policy=Policy(jerk=self.jerk.currentData(),contact_mm=self.contact.value(),transit_mm=self.transit.value());policy.validate()
            count,start,finish,leave,rate=self.count.value(),self.start.currentData(),self.finish.currentData(),self.leave.isChecked(),self.rate.value()
            saved_controls=self.snapshot_controls()
            self.active=True;self.source_token=(workflow,id(source),revision);self.refresh(True)
            def check():
                if self.cancelled or n.shutdown_requested or control.config is not source or control.revision!=revision:
                    raise RuntimeError('计算已取消或轨迹版本已变化')
            def work():
                try:
                    from task_planning_config import load_parameters,DEFAULT_CONFIG
                    from task_trajectory_speed import rescale
                    began=time.monotonic();parameters,_=load_parameters(getattr(n.args,'planning_config',DEFAULT_CONFIG))
                    payloads,hit=comparison(workflow,config,ready,n.model,parameters,policy,options,check,
                        progress=lambda text:setattr(n,'status',text))
                    results={}
                    for name,payload in payloads.items():
                        # Slowing cached units changes neither fitting nor their reusable boundaries.
                        from rosidl_runtime_py.convert import message_to_ordereddict
                        from task_offline_optimizer import decode
                        from task_trajectory import time_seconds
                        for unit in payload['units'].values():
                            tr=rescale(decode(unit['trajectory']),rate)
                            unit['trajectory']=message_to_ordereddict(tr);unit['seconds']=time_seconds(tr.points[-1])
                        payload['options']['rate']=rate
                        payload['options']['assembly']=dict(count=count,start=start,finish=finish,leave=leave)
                        sequence=sequence_for(payload,count,start,finish,leave);tr,events=materialize(payload,sequence);check()
                        times,tcp,joint=preview_samples(tr,payload,events,n.model,config.get('side','right'));check()
                        import numpy as np
                        from task_composition_smoothing import evaluate
                        from task_smooth_timing import smoothness
                        results[name]=dict(payload=payload,sequence=sequence,trajectory=tr,events=events,
                            times=times,tcp=tcp,joint=joint,hit=hit,elapsed=time.monotonic()-began,cache='offline_optimizer_cache')
                        results[name].update(acceleration=np.max(np.abs(evaluate(tr,times,2)),axis=1),
                                             jerk=np.max(np.abs(evaluate(tr,times,3)),axis=1),
                                             smoothness=smoothness(tr),joint_names=list(tr.joint_names),
                                             joint_matrix=evaluate(tr,times,1),acceleration_matrix=evaluate(tr,times,2),
                                             jerk_matrix=evaluate(tr,times,3))
                    from task_optimizer_snapshot import save,load
                    check();path=save(results,saved_controls,n.model,self.snapshot_directory)
                    self.completed.emit(load(path,n.model))
                except Exception as error:self.completed.emit(dict(error=str(error)))
            n.launch_work(work)
        except Exception as error:
            self.active=False;self.viewer.summary.setText(str(error));n.status=str(error)

    def accept(self,result):
        self.active=False
        if self.cancelled:self.viewer.summary.setText('计算已取消');return
        self.viewer.accept(result)

    def refresh(self,busy):
        self.refresh_trims()
        from task_trace_versions import bank
        for index in range(self.case.count()):
            workflow,slot=self.case.itemData(index)
            if workflow=='rail' and slot in ('push_cycle','pull_cycle'):continue
            if slot:
                config=self.control(workflow).config
                label={'dring_demo':'拉环','landing_gear':'起落架','rail':'滑轨'}[workflow]
                version=bank(config,slot)['selected'] if slot in config.get('traces',{}) else '已删除'
                self.case.setItemText(index,f'{label} · {slot} · v{version}')
        if self.source_token:
            workflow,identity,revision=self.source_token;control=self.control(workflow)
            if (id(control.config),control.revision)!=(identity,revision):self.cancel();self.invalidate()
        workflow,slot=self.case.currentData();enabled=not busy and not self.active
        for w in (self.case,self.comparison_set):w.setEnabled(enabled)
        enabled=enabled and self.comparison_set.currentData() is None
        for w in (self.jerk,self.contact,self.transit,self.rate,self.generate,self.smooth_first,self.continuous_flow):w.setEnabled(enabled)
        self.extra.setEnabled(enabled and (self.smooth_first.isChecked() or self.continuous_flow.isChecked()))
        for w in (self.left,self.right):w.setEnabled(enabled and workflow=='dring_demo' and slot is None)
        self.count.setEnabled(enabled and slot is None)
        for w in (self.start,self.finish,self.leave):w.setEnabled(enabled and workflow=='landing_gear' and slot is None)
        single=slot is not None and not (workflow=='rail' and slot in ('push_cycle','pull_cycle'))
        self.manual.setEnabled(enabled and single)
        for w in (self.begin,self.end):w.setEnabled(enabled and single and self.manual.isChecked())
        self.viewer.refresh(busy or self.active)

    def halt(self):
        self.cancel();self.viewer.halt()
