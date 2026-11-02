"""Experimental task calibration, frozen per plan; no robot I/O."""
import copy
import hashlib
import json
import os
from pathlib import Path
import tempfile
import time

import numpy as np
from scipy.spatial.transform import Rotation

CONFIG = Path(__file__).resolve().parents[1]/'config/task_calibration.json'
C_ROTATION = np.array([[.006050632841559165,-.7513865076806444,.6598346049713533],
    [-.9999378697487158,-.010723825873369647,-.0030424005388949094],
    [.00936197012482195,-.6597752008329212,-.751404709781127]])
DEFAULTS = dict(a_enabled=True,b_enabled=True,c_enabled=True,
    left_x_mm=290.179716,left_y_mm=-21.103831,left_z_mm=22.065103,
    right_x_mm=269.275904,right_y_mm=25.160820,right_z_mm=28.126456,
    depth_bias_mm=0.,depth_scale=1.,
    camera_x_mm=46.2995039749191,camera_y_mm=37.06267321254929,camera_z_mm=441.4262584877524,
    camera_rx_deg=0.,camera_ry_deg=0.,camera_rz_deg=0.)
BOUNDS={k:(-400.,400.) for k in DEFAULTS if k.startswith(('left_','right_'))}
BOUNDS.update(depth_bias_mm=(-30.,30.),depth_scale=(.95,1.05))
BOUNDS.update({f'camera_{a}_mm':(-600.,600.) for a in 'xyz'})
BOUNDS.update({f'camera_r{a}_deg':(-10.,10.) for a in 'xyz'})


def validate(values):
    if not isinstance(values,dict) or set(values)!=set(DEFAULTS):
        raise ValueError('Calibration fields/version mismatch')
    result={}
    for key,default in DEFAULTS.items():
        value=values[key]
        if isinstance(default,bool):
            if type(value) is not bool:raise ValueError('Expected checkbox: '+key)
        else:
            if type(value) not in (int,float) or not np.isfinite(value) or not BOUNDS[key][0]<=value<=BOUNDS[key][1]:
                raise ValueError('Calibration value outside bounds: '+key)
            value=float(value)
        result[key]=value
    return result


def load(path=CONFIG):
    return validate(json.loads(Path(path).read_text())) if Path(path).exists() else dict(DEFAULTS)


def save(values,path=CONFIG):
    values=validate(values);path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
    with tempfile.NamedTemporaryFile('w',dir=path.parent,delete=False) as f:
        json.dump(values,f,indent=2);f.write('\n');temp=f.name
    os.replace(temp,path)


def snapshot(values):
    values=validate(values)
    return dict(schema='task_abc_v1',values=values,
        sha256=hashlib.sha256(json.dumps(values,sort_keys=True).encode()).hexdigest(),
        experimental=True,accuracy_certified=False,
        sources=dict(A='20260912 left candidate; right nominal (right candidate failed validation)',
                     B='Identity default; observed ~9mm discrepancy is NOT a correction',
                     C='6f5aa0b2464c4d87b619be397490ed89; 12 fit / 6 validation'),
        rotation_convention='R = Rz(dz) Ry(dy) Rx(dx) R_C_candidate; increments in torso axes')


def camera_transform(nominal,values):
    v=validate(values)
    if not v['c_enabled']:return np.array(nominal,copy=True)
    t=np.eye(4)
    t[:3,:3]=Rotation.from_euler('xyz',[v[f'camera_r{a}_deg'] for a in 'xyz'],degrees=True).as_matrix()@C_ROTATION
    t[:3,3]=[v[f'camera_{a}_mm']/1000 for a in 'xyz']
    return t


def depth_points(points,values):
    v=validate(values);p=np.array(points,dtype=float,copy=True)
    if not v['b_enabled']:return p
    z=p[...,2];new=z*v['depth_scale']+v['depth_bias_mm']/1000
    if np.any(z<=0) or np.any(new<=0) or not np.isfinite(p).all():
        raise ValueError('Depth compensation produced invalid geometry')
    return p*(new/z)[...,None]


def fingertips(original,values):
    v=validate(values);result=copy.deepcopy(original)
    if not v['a_enabled']:return result
    for side in ('left','right'):
        state=result['sides'][side]['states']['open']
        w=np.array(state['wrist_yaw_from_index_tcp']['matrix_row_major'])
        palm=np.array(state['palm_from_index_tcp']['matrix_row_major'])
        wrist_from_palm=w@np.linalg.inv(palm)
        w[:3,3]=[v[f'{side}_{a}_mm']/1000 for a in 'xyz']
        for key,t in [('wrist_yaw_from_index_tcp',w),('palm_from_index_tcp',np.linalg.inv(wrist_from_palm)@w)]:
            state[key]['matrix_row_major']=t.tolist();state[key]['xyz_m']=t[:3,3].tolist()
    return result


def cloud_points(xyz,geometry):
    record=geometry.get('calibration')
    if not record:return xyz
    old=np.asarray(geometry['transforms']['uncalibrated_torso_from_color_optical_row_major'])
    new=np.asarray(geometry['transforms']['torso_from_color_optical_row_major'])
    camera=(np.asarray(xyz)-old[:3,3])@old[:3,:3]
    return depth_points(camera,record['values'])@new[:3,:3].T+new[:3,3]


def marker_offsets(model,values):
    from task_preview_markers import planning_tcp_offsets
    offsets=planning_tcp_offsets(model);values=validate(values)
    if values['a_enabled']:
        for side,t in offsets.items():t[:3,3]=[values[f'{side}_{a}_mm']/1000 for a in 'xyz']
    return offsets


def panel(window):
    from PyQt5 import QtWidgets
    widget=QtWidgets.QWidget();layout=QtWidgets.QVBoxLayout(widget)
    header=QtWidgets.QLabel('实验参数 · A 左侧候选 / 右侧原值 · B 默认无修正 · C 本轮候选')
    header.setWordWrap(True);layout.addWidget(header)
    controls={}
    def changed():
        values={k:(c.isChecked() if isinstance(c,QtWidgets.QCheckBox) else c.value()) for k,c in controls.items()}
        try:window.node.set_calibration(values)
        except Exception as error:
            QtWidgets.QMessageBox.warning(widget,'参数未应用',str(error))
            assign(window.node.calibration)
        window.clear.setChecked(False)
    sections=[('A：腕坐标系 TCP（mm）','a_enabled',[(f'{s}_{a}_mm',f'{s} {a.upper()}') for s in ('left','right') for a in 'xyz']),
        ('B：RGB 光学系深度（z新 = scale × z原 + bias）','b_enabled',[('depth_bias_mm','bias (mm)'),('depth_scale','scale')]),
        ('C：RGB 相机在 torso_link 中的外参','c_enabled',[(f'camera_{a}_mm',f'{a.upper()} (mm)') for a in 'xyz']+[(f'camera_r{a}_deg',f'绕 torso {a.upper()} 增量 (deg)') for a in 'xyz'])]
    for title,toggle,fields in sections:
        check=QtWidgets.QCheckBox(title);check.setChecked(window.node.calibration[toggle]);controls[toggle]=check;layout.addWidget(check)
        form=QtWidgets.QFormLayout();layout.addLayout(form)
        for key,label in fields:
            spin=QtWidgets.QDoubleSpinBox();spin.setRange(*BOUNDS[key]);spin.setDecimals(6)
            spin.setSingleStep(.001 if key=='depth_scale' else (.1 if key.endswith('deg') else 1.))
            spin.setValue(window.node.calibration[key]);controls[key]=spin;form.addRow(label,spin)
    def assign(values):
        for key,c in controls.items():
            c.blockSignals(True)
            if isinstance(c,QtWidgets.QCheckBox):c.setChecked(values[key])
            else:c.setValue(values[key])
            c.blockSignals(False)
    for c in controls.values():
        (c.toggled if isinstance(c,QtWidgets.QCheckBox) else c.valueChanged).connect(changed)
    buttons=QtWidgets.QHBoxLayout();layout.addLayout(buttons)
    def reset(values):assign(values);changed()
    for label,callback in [('保存为启动配置',lambda:save(window.node.calibration)),
                           ('恢复本轮初值',lambda:reset(DEFAULTS)),
                           ('关闭全部补偿',lambda:reset(dict(window.node.calibration,a_enabled=False,b_enabled=False,c_enabled=False)))]:
        button=QtWidgets.QPushButton(label);button.clicked.connect(lambda _,fn=callback:window.invoke(fn));buttons.addWidget(button)
    manual=QtWidgets.QPushButton('记录尺测误差');layout.addWidget(manual)
    def ruler():
        path,_=QtWidgets.QFileDialog.getOpenFileName(widget,'选择本次 center_check_result.json',str(window.node.run_dir),'JSON (*.json)')
        if not path:return
        source=Path(path);raw=source.read_bytes();record=json.loads(raw)
        if record.get('settled') is not True or 'target_pelvis_m' not in record:
            raise ValueError('请选择已到位的 center check 结果，不是计划文件')
        dialog=QtWidgets.QDialog(widget);dialog.setWindowTitle('尺测：真实指尖减目标（pelvis 坐标系）')
        form=QtWidgets.QFormLayout(dialog);inputs=[]
        for axis in 'XYZ':
            spin=QtWidgets.QDoubleSpinBox();spin.setRange(-200,200);spin.setSuffix(' mm');spin.setDecimals(1)
            form.addRow(axis,spin);inputs.append(spin)
        note=QtWidgets.QLineEdit();form.addRow('备注 / 测量方法',note)
        ok=QtWidgets.QDialogButtonBox(QtWidgets.QDialogButtonBox.Save|QtWidgets.QDialogButtonBox.Cancel)
        ok.accepted.connect(dialog.accept);ok.rejected.connect(dialog.reject);form.addRow(ok)
        if dialog.exec_()!=QtWidgets.QDialog.Accepted:return
        output=source.parent/f'ruler_{time.time_ns()}.json'
        with output.open('x') as f:json.dump(dict(source=str(source),source_sha256=hashlib.sha256(raw).hexdigest(),
            physical_tip_error_pelvis_mm=[s.value() for s in inputs],note=note.text(),
            calibration=record.get('calibration'),target_pelvis_m=record['target_pelvis_m'],
            method='operator_ruler',automatic_correction=False),f,indent=2,ensure_ascii=False)
        QtWidgets.QMessageBox.information(widget,'已记录',str(output))
    manual.clicked.connect(lambda:window.invoke(ruler))
    layout.addStretch()
    scroll=QtWidgets.QScrollArea();scroll.setWidgetResizable(True);scroll.setWidget(widget)
    return scroll
