"""Millimeter-scale FK overlays and line-only RViz markers for offline review."""
import math

import numpy as np
from PyQt5 import QtCore, QtGui, QtWidgets

COLORS=('#465b70','#db6b17','#13865d','#b44772','#287fc1')
NAMESPACE='offline_optimizer_paths'


def tcp_positions(result,model,times):
    from task_composition_smoothing import evaluate
    from task_composition_panel import hand_values
    from task_hand_control import model_joints
    tr=result['trajectory'];names=list(tr.joint_names)
    q=evaluate(tr,times)
    hands=hand_values(result['payload'],result['events'],times)
    if hands is not None:
        rows=[model_joints('right',v) for v in hands]
        extra=[n for n in rows[0] if n not in names]
        names+=extra
        q=np.column_stack((q,[[v[n] for n in extra] for v in rows]))
    return model.batch_transforms(['right_index_tcp'],result['payload']['context'],names,q)['right_index_tcp'][:,:3,3]


def sample_path(result,model,spacing_mm=.05):
    from task_trajectory import time_seconds
    tr=result['trajectory'];duration=time_seconds(tr.points[-1])
    times=np.unique(np.r_[[time_seconds(p) for p in tr.points],np.linspace(0.,duration,max(2,math.ceil(duration/.02)+1))])
    xyz=tcp_positions(result,model,times)
    for _ in range(14):
        mid=(times[:-1]+times[1:])/2
        middle=tcp_positions(result,model,mid)
        chord=np.linalg.norm(np.diff(xyz,axis=0),axis=1)*1000
        error=np.linalg.norm(middle-(xyz[:-1]+xyz[1:])/2,axis=1)*1000
        split=(chord>spacing_mm)|(error>.005)
        if not np.any(split):
            return dict(times=times,xyz_mm=xyz*1000,max_chord_mm=float(max(chord)),
                        midpoint_error_mm=float(max(error)))
        if len(times)+sum(split)>160000:raise ValueError('Spatial preview sample budget exceeded')
        merged=np.r_[times,mid[split]];order=np.argsort(merged)
        xyz=np.concatenate((xyz,middle[split]))[order];times=merged[order]
    raise ValueError('Spatial preview cannot meet display resolution')


def path_markers(curves,visible):
    from geometry_msgs.msg import Point
    from visualization_msgs.msg import Marker,MarkerArray
    result=MarkerArray(markers=[Marker(ns=NAMESPACE,id=i,action=Marker.DELETE) for i in range(8)])
    from task_trace_markers import clear_markers
    result.markers.extend(clear_markers().markers)
    for i,(key,curve) in enumerate(curves.items()):
        if key not in visible:continue
        color=QtGui.QColor(curve['color'])
        marker=Marker(ns=NAMESPACE,id=i,type=Marker.LINE_STRIP,action=Marker.ADD)
        marker.header.frame_id='pelvis';marker.pose.orientation.w=1.
        marker.scale.x=.00015
        marker.color.r=color.redF();marker.color.g=color.greenF();marker.color.b=color.blueF();marker.color.a=1.
        marker.points=[Point(x=float(x/1000),y=float(y/1000),z=float(z/1000)) for x,y,z in curve['xyz_mm']]
        result.markers.append(marker)
    return result


class SpatialPlot(QtWidgets.QWidget):
    def __init__(self):
        super().__init__();self.setMinimumSize(300,310)
        self.curves={};self.visible=set();self.axes=(0,2);self.center=np.zeros(2)
        self.span=24.;self.drag=None
        self.setToolTip('滚轮缩放；拖动平移；坐标为 URDF root 下的命令手型 FK，单位 mm')

    def box(self):return QtCore.QRectF(self.rect().adjusted(58,22,-18,-40))

    def pixels_per_mm(self):return self.box().width()/self.span

    def project(self,xy):
        box=self.box();value=(np.asarray(xy)-self.center)*self.pixels_per_mm()
        return np.column_stack((box.center().x()+value[:,0],box.center().y()-value[:,1]))

    def fit(self,local=False):
        curves=[v['xyz_mm'] for k,v in self.curves.items() if k in self.visible]
        if not curves:return
        if local:
            peaks=np.array([v.get('focus_xyz_mm',v['xyz_mm'][np.argmax(v['xyz_mm'][:,0])])
                            for k,v in self.curves.items() if k in self.visible])
            self.center=np.mean(peaks[:,self.axes],axis=0);self.span=24.
        else:
            q=np.concatenate(curves)[:,self.axes];lo=q.min(axis=0);hi=q.max(axis=0)
            self.center=(lo+hi)/2
            self.span=max(hi[0]-lo[0],(hi[1]-lo[1])*self.box().width()/max(1,self.box().height()),1.)*1.12
        self.update()

    def paintEvent(self,event):
        painter=QtGui.QPainter(self);painter.setRenderHint(QtGui.QPainter.Antialiasing)
        painter.fillRect(self.rect(),QtGui.QColor('#ffffff'));box=self.box()
        scale=self.pixels_per_mm();height=box.height()/scale
        wanted=60/scale;power=10**math.floor(math.log10(max(wanted,1e-9)))
        step=next(v*power for v in (1,2,5,10) if v*power>=wanted)
        lower=self.center-np.array([self.span,height])/2;upper=self.center+np.array([self.span,height])/2
        font=painter.font();font.setPointSize(9);painter.setFont(font)
        for axis in (0,1):
            for value in np.arange(math.ceil(lower[axis]/step)*step,upper[axis]+step*.001,step):
                q=self.center.copy();q[axis]=value;x,y=self.project(q[None])[0]
                painter.setPen(QtGui.QPen(QtGui.QColor('#e5e9ec'),1))
                if axis==0:painter.drawLine(QtCore.QPointF(x,box.top()),QtCore.QPointF(x,box.bottom()))
                else:painter.drawLine(QtCore.QPointF(box.left(),y),QtCore.QPointF(box.right(),y))
                painter.setPen(QtGui.QColor('#4e5962'));text=f'{value:.2f}' if step<1 else f'{value:.1f}'
                if axis==0:painter.drawText(QtCore.QRectF(x-30,box.bottom()+4,60,20),QtCore.Qt.AlignCenter,text)
                else:painter.drawText(QtCore.QRectF(0,y-10,53,20),QtCore.Qt.AlignRight|QtCore.Qt.AlignVCenter,text)
        painter.setPen(QtGui.QColor('#66737c'));painter.drawRect(box)
        painter.drawText(8,15,'XYZ'[self.axes[1]]+' / mm')
        painter.drawText(int(box.right()-75),self.height()-3,'XYZ'[self.axes[0]]+' / mm')
        painter.save();painter.setClipRect(box)
        for key,curve in self.curves.items():
            if key not in self.visible:continue
            xy=self.project(curve['xyz_mm'][:,self.axes]);path=QtGui.QPainterPath()
            for i,(x,y) in enumerate(xy):
                if i==0:path.moveTo(float(x),float(y))
                else:path.lineTo(float(x),float(y))
            painter.setPen(QtGui.QPen(QtGui.QColor(curve['color']),1.5));painter.drawPath(path)
        painter.restore()

    def wheelEvent(self,event):
        self.span=float(np.clip(self.span*(.8 if event.angleDelta().y()>0 else 1.25),.5,3000.))
        self.update();event.accept()

    def mousePressEvent(self,event):
        if event.button()==QtCore.Qt.LeftButton:self.drag=event.pos()

    def mouseMoveEvent(self,event):
        if self.drag is not None:
            delta=event.pos()-self.drag;self.drag=event.pos()
            self.center+=np.array([-delta.x(),delta.y()])/self.pixels_per_mm();self.update()

    def mouseReleaseEvent(self,event):self.drag=None


class SpatialComparison(QtWidgets.QWidget):
    minimum_height_changed=QtCore.pyqtSignal(int)
    def __init__(self,node):
        super().__init__();self.node=node;self.curves={};self.token=None
        layout=QtWidgets.QVBoxLayout(self);layout.setContentsMargins(0,0,0,0)
        layout.setSizeConstraint(QtWidgets.QLayout.SetMinimumSize)
        row=QtWidgets.QHBoxLayout();layout.addLayout(row)
        self.plane=QtWidgets.QComboBox()
        for label,axes in (('XZ / 侧视',(0,2)),('XY / 俯视',(0,1)),('YZ / 正视',(1,2))):self.plane.addItem(label,axes)
        row.addWidget(self.plane)
        self.local=QtWidgets.QToolButton();self.local.setIcon(self.style().standardIcon(QtWidgets.QStyle.SP_FileDialogContentsView))
        self.local.setToolTip('比较位置：24 mm 局部视野')
        self.full=QtWidgets.QToolButton();self.full.setIcon(self.style().standardIcon(QtWidgets.QStyle.SP_TitleBarMaxButton));self.full.setToolTip('完整轨迹')
        row.addWidget(self.local);row.addWidget(self.full);row.addStretch()
        self.legend=QtWidgets.QHBoxLayout();layout.addLayout(self.legend)
        self.plot=SpatialPlot();layout.addWidget(self.plot)
        self.readout=QtWidgets.QLabel();self.readout.setWordWrap(True)
        self.readout.setSizePolicy(QtWidgets.QSizePolicy.Preferred,QtWidgets.QSizePolicy.Minimum)
        layout.addWidget(self.readout)
        self.local.clicked.connect(lambda:self.plot.fit(True));self.full.clicked.connect(lambda:self.plot.fit(False))
        self.plane.currentIndexChanged.connect(self.change_plane)

    def change_plane(self):
        self.plot.axes=self.plane.currentData();self.plot.fit(self.is_depth)

    @property
    def is_depth(self):return any(v.get('depth') for v in self.curves.values())

    def set_results(self,variants):
        if not hasattr(self.node.model,'batch_transforms'):
            self.clear();return
        token=tuple((k,id(v['trajectory'])) for k,v in variants.items())
        if token==self.token:return
        self.token=token;self.curves={}
        while self.legend.count():
            item=self.legend.takeAt(0)
            if item.widget():item.widget().hide();item.widget().deleteLater()
        labels=dict(original='原路径',baseline='原路径同策略',candidate='拟合候选',smooth='平顺候选',flow='本轮候选')
        for i,(key,result) in enumerate(variants.items()):
            curve=sample_path(result,self.node.model)
            curve.update(color=COLORS[i%len(COLORS)],label=result['payload'].get('comparisonLabel',labels.get(key,key)),
                         depth='spatialComparison' in result['payload'])
            focus=result['payload'].get('spatialComparison',{})
            if focus.get('focus')=='release':
                curve.update(focus_xyz_mm=focus['focus_xyz_mm'],release_xyz_mm=focus['release_xyz_mm'])
            elif 'point_xyz_mm' in focus:
                curve.update(focus_xyz_mm=focus['focus_xyz_mm'],point_xyz_mm=focus['point_xyz_mm'],
                             point_label=focus['point_label'])
            self.curves[key]=curve
            checkbox=QtWidgets.QCheckBox(curve['label']);checkbox.setStyleSheet('color: '+curve['color'])
            checkbox.setChecked(True);checkbox.toggled.connect(lambda checked,k=key:self.toggle_path(k,checked))
            self.legend.addWidget(checkbox)
        self.plot.curves=self.curves;self.plot.visible=set(self.curves)
        self.plot.fit(self.is_depth)
        release=all('release_xyz_mm' in v for v in self.curves.values())
        focused=all('point_xyz_mm' in v for v in self.curves.values())
        peaks=[(v['label'],float(v['release_xyz_mm'][0] if release else
                               v['point_xyz_mm'][0] if focused else v['xyz_mm'][:,0].max())) for v in self.curves.values()]
        metric='松手前 X' if release else next(iter(self.curves.values()))['point_label'] if focused else 'Xmax'
        text='\n'.join(f'{name}: {metric} {x:.3f} mm'+(f' / ΔX {x-peaks[0][1]:+.3f} mm' if i else '') for i,(name,x) in enumerate(peaks))
        self.readout.setText(text+'\nURDF root · 命令手型 FK · 轨迹采样间距 ≤0.05 mm · 非实物测量')
        self.update_readout_height()
        self.publish()

    def update_readout_height(self):
        self.readout.setMinimumHeight(max(0,self.readout.heightForWidth(max(1,self.readout.width()))))
        self.minimum_height_changed.emit(self.layout().minimumSize().height())

    def resizeEvent(self,event):
        super().resizeEvent(event)
        self.update_readout_height()

    def toggle_path(self,key,checked):
        if checked:self.plot.visible.add(key)
        else:self.plot.visible.discard(key)
        self.plot.update();self.publish()

    def publish(self):
        publisher=getattr(self.node,'marker_pub',None)
        if publisher is not None:publisher.publish(path_markers(self.curves,self.plot.visible))

    def clear(self):
        self.token=None;self.curves={};self.plot.curves={};self.plot.visible=set();self.plot.update();self.readout.clear()
        while self.legend.count():
            item=self.legend.takeAt(0)
            if item.widget():item.widget().hide();item.widget().deleteLater()
        self.publish()

    def clear_markers(self):
        publisher=getattr(self.node,'marker_pub',None)
        if publisher is not None:
            from visualization_msgs.msg import Marker,MarkerArray
            publisher.publish(MarkerArray(markers=[Marker(ns=NAMESPACE,id=i,action=Marker.DELETE) for i in range(8)]))

    def showEvent(self,event):
        super().showEvent(event);self.plot.fit(self.is_depth);self.publish()
