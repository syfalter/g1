"""Historical RGB-D overlay, on a visualization-only topic."""
import copy
import json
from pathlib import Path
import threading

import numpy as np
from PyQt5 import QtCore, QtWidgets
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import PointCloud2, PointField
from task_numeric_input import FreeDoubleSpinBox

PACKED = np.dtype([('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('rgb', '<u4')])


def preview_cloud(packed, voxel=.005, maximum=30000):
    xyz = np.column_stack([packed[a] for a in 'xyz'])
    _, indices = np.unique(np.floor(xyz/voxel).astype(np.int64), axis=0, return_index=True)
    indices.sort()
    if len(indices) > maximum:
        indices = indices[np.linspace(0, len(indices)-1, maximum).round().astype(int)]
    return packed[indices]


def load_reference(directory, model):
    from vision_task_preview import load_cloud
    directory = Path(directory)
    geometry = json.loads((directory/'geometry.json').read_text())
    cloud = load_cloud(directory, geometry)
    frame = geometry.get('capture_frame', {}).get('pelvis_from_torso_row_major')
    if frame is None and (directory/'capture_state.json').is_file():
        state = json.loads((directory/'capture_state.json').read_text())
        frame = state.get('pelvis_from_torso_row_major')
        if frame is None:frame = model.transform('torso_link', state['joints'])
    if frame is None:frame = model.transform('torso_link', {})
    transform = np.asarray(frame, dtype=float)
    if transform.shape != (4, 4) or not np.isfinite(transform).all():
        raise ValueError('Invalid historical camera transform')
    packed = np.frombuffer(bytes(cloud.data), dtype=PACKED).copy()
    xyz = np.column_stack([packed[a] for a in 'xyz'])@transform[:3, :3].T+transform[:3, 3]
    for i, axis in enumerate('xyz'):packed[axis] = xyz[:, i]
    packed = preview_cloud(packed)
    cloud.width = len(packed); cloud.row_step = cloud.point_step*cloud.width
    cloud.data = packed.tobytes(); cloud.header.frame_id = 'pelvis'
    return cloud


def reference_message(cloud, offset=(0., 0., 0.), visible=True):
    if cloud is None or not visible:
        empty = PointCloud2(); empty.header.frame_id = 'pelvis'
        empty.height = 1; empty.point_step = 16
        empty.fields = [PointField(name=name, offset=i*4, datatype=PointField.FLOAT32, count=1)
                        for i,name in enumerate(('x','y','z','rgb'))]
        return empty
    result = copy.deepcopy(cloud)
    packed = np.frombuffer(bytes(result.data), dtype=PACKED).copy()
    for axis, delta in zip('xyz', offset):packed[axis] += delta
    result.data = packed.tobytes()
    return result


class DepthReferencePanel(QtWidgets.QWidget):
    loaded = QtCore.pyqtSignal(object, str, int)

    def __init__(self, node):
        super().__init__()
        self.node = node; self.cloud = None; self.serial = 0; self.pending_source = None
        self.publisher = node.create_publisher(PointCloud2, '/g1/task/historical_depth_reference',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        layout = QtWidgets.QVBoxLayout(self); layout.setContentsMargins(0, 0, 0, 0)
        row = QtWidgets.QHBoxLayout(); layout.addLayout(row)
        self.visible = QtWidgets.QCheckBox('历史深度参考'); self.visible.setChecked(True)
        row.addWidget(self.visible)
        self.sources = QtWidgets.QComboBox(); self.sources.setMinimumContentsLength(15)
        self.sources.setSizeAdjustPolicy(QtWidgets.QComboBox.AdjustToMinimumContentsLengthWithIcon)
        row.addWidget(self.sources, 1)
        self.refresh_button = QtWidgets.QPushButton()
        self.refresh_button.setIcon(self.style().standardIcon(QtWidgets.QStyle.SP_BrowserReload))
        self.refresh_button.setToolTip('刷新历史深度帧'); row.addWidget(self.refresh_button)
        row = QtWidgets.QHBoxLayout(); layout.addLayout(row)
        self.offsets = []
        self.timer = QtCore.QTimer(self); self.timer.setSingleShot(True); self.timer.setInterval(120)
        self.timer.timeout.connect(self.publish)
        for axis in 'XYZ':
            row.addWidget(QtWidgets.QLabel(axis))
            spin = FreeDoubleSpinBox(0.)
            spin.setSingleStep(.01); spin.setSuffix(' m'); row.addWidget(spin)
            spin.valueChanged.connect(lambda *_:self.timer.start()); self.offsets.append(spin)
        reset = QtWidgets.QPushButton(); reset.setIcon(self.style().standardIcon(QtWidgets.QStyle.SP_DialogResetButton))
        reset.setToolTip('复位参考点云平移'); reset.clicked.connect(self.reset); row.addWidget(reset)
        self.status = QtWidgets.QLabel('历史参考 / 非当前测量'); self.status.setWordWrap(True)
        layout.addWidget(self.status)
        self.loaded.connect(self.accept)
        self.sources.currentIndexChanged.connect(self.load)
        self.visible.toggled.connect(self.publish)
        self.refresh_button.clicked.connect(self.scan)
        QtCore.QTimer.singleShot(0, self.scan)

    def scan(self):
        root = self.node.run_dir.parent
        paths = sorted((p.parent for p in root.glob('*/plan_*/cloud.npz')
                        if (p.parent/'geometry.json').is_file() and (p.parent/'manifest.json').is_file()),
                       reverse=True)
        previous = self.pending_source or self.sources.currentData()
        self.pending_source = None
        self.sources.blockSignals(True); self.sources.clear()
        for path in paths:self.sources.addItem(path.parent.name+' / '+path.name, str(path))
        index = self.sources.findData(previous)
        if index >= 0:self.sources.setCurrentIndex(index)
        self.sources.blockSignals(False); self.load()

    def load(self, *_):
        self.serial += 1; serial = self.serial
        self.cloud = None; self.publish()
        path = self.sources.currentData()
        if not path:
            self.status.setText('没有历史深度帧'); return
        self.status.setText('正在载入历史深度参考')
        def work():
            try:self.loaded.emit(load_reference(path, self.node.model), '', serial)
            except Exception as error:self.loaded.emit(None, str(error), serial)
        threading.Thread(target=work, daemon=True).start()

    def accept(self, cloud, error, serial):
        if serial != self.serial:return
        self.cloud = cloud
        self.status.setText(('历史参考载入失败：'+error) if error else
                            f'历史参考 / 非当前测量 / {cloud.width} 点')
        self.publish()

    def reset(self):
        for spin in self.offsets:spin.setValue(0.)

    def settings(self):
        return dict(visible=self.visible.isChecked(), source=self.pending_source or self.sources.currentData(),
                    offsets=[spin.value() for spin in self.offsets])

    def restore_settings(self, data):
        if type(data.get('visible')) is bool:self.visible.setChecked(data['visible'])
        if isinstance(data.get('source'), str):self.pending_source = data['source']
        offsets = data.get('offsets', [])
        if isinstance(offsets, list):
            for spin, value in zip(self.offsets, offsets):
                if type(value) in (int, float) and np.isfinite(value):spin.setValue(value)

    def publish(self, *_):
        message = reference_message(self.cloud, [s.value() for s in self.offsets], self.visible.isChecked())
        message.header.stamp = self.node.get_clock().now().to_msg()
        self.publisher.publish(message)

    def shutdown(self):
        self.serial += 1; self.timer.stop()
        self.publisher.publish(reference_message(None))
