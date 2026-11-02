"""Local UI preferences, kept separate from trajectory and execution state."""
import hashlib
import json
import math
from pathlib import Path

import numpy as np
from PyQt5 import QtCore, QtWidgets
from task_trace_versions import EDIT_SLOTS

DEFAULT_PATH = Path(__file__).resolve().parents[1]/'data/task_panel/offline_editor_settings.json'
PARAMETERS = ('point_spacing', 'point_limit', 'arrow_length', 'arrow_diameter', 'point_diameter',
              'compare_mode', 'compare_reverse', 'compare_only', 'compare_mm', 'compare_deg',
              'tolerance', 'window_s', 'bridge', 'curve_seconds', 'anchor_source', 'anchor_limit',
              'local_factor', 'local_blend', 'generation_mode', 'generation_speed',
              'fast_velocity', 'fast_acceleration', 'fast_denoise', 'tail_seconds')


def read(path):
    if not path.exists():return {}
    data = json.loads(path.read_text())
    if not isinstance(data, dict) or data.get('schema') != 1:
        raise ValueError('Unsupported editor settings format')
    return data


def values(editor):
    result = {}
    for name in PARAMETERS:
        widget = getattr(editor, name, None)
        if isinstance(widget, QtWidgets.QComboBox):result[name] = widget.currentData()
        elif isinstance(widget, QtWidgets.QCheckBox):result[name] = widget.isChecked()
        elif widget is not None:result[name] = widget.value()
    return result


def restore_values(editor, data):
    if not isinstance(data, dict):return
    for name in PARAMETERS:
        widget = getattr(editor, name, None)
        if widget is None or name not in data:continue
        value = data[name]
        blocker = QtCore.QSignalBlocker(widget)
        if isinstance(widget, QtWidgets.QComboBox):
            index = widget.findData(value)
            if index >= 0:widget.setCurrentIndex(index)
        elif isinstance(widget, QtWidgets.QCheckBox):
            if type(value) is bool:widget.setChecked(value)
        elif type(value) in (int, float) and math.isfinite(value):
            if hasattr(widget,'accepts') and not widget.accepts(value):continue
            if widget.maximum() is not None:value = min(widget.maximum(), value)
            value = max(widget.minimum(), value)
            widget.setValue(int(value) if isinstance(widget, QtWidgets.QSpinBox) else value)
        del blocker
    editor.schedule_markers()
    if hasattr(editor,'generation_mode'):
        editor.control.generation_mode=editor.generation_mode.currentData()


def trace_id(trace):
    digest = hashlib.sha256()
    for key in ('time_s', 'positions'):
        digest.update(np.asarray(trace['raw'][key], dtype='<f8').tobytes())
    return digest.hexdigest()


def draft(editor):
    if editor.trace is None:return {}
    return dict(trace_id=trace_id(editor.trace), start=editor.start.value(), end=editor.end.value(),
                position=editor.position, key_times=list(editor.key_times), compared=list(editor.compared),
                manual=editor.manual.draft())


def restore_draft(editor, data):
    if (not isinstance(data, dict) or editor.trace is None or
            data.get('trace_id') != trace_id(editor.trace)):return
    duration = editor.trace['raw']['time_s'][-1]
    editor.manual.restore(data.get('manual'))
    def valid(value):return type(value) in (int, float) and math.isfinite(value) and 0 <= value <= duration
    for name in ('start', 'end'):
        if valid(data.get(name)):getattr(editor, name).setValue(data[name])
    keys = data.get('key_times')
    if isinstance(keys, list) and len(keys) <= 64 and all(valid(v) for v in keys):
        editor.key_times = sorted(set(keys)); editor.key_list()
    compared = data.get('compared', [])
    if isinstance(compared, list):
        for index in compared[:2]:
            if type(index) is int and 0 <= index <= EDIT_SLOTS:
                item = editor.versions.item(index, 3)
                if item.flags() & QtCore.Qt.ItemIsEnabled:item.setCheckState(QtCore.Qt.Checked)
    if valid(data.get('position')):
        editor.position = data['position']
        editor.slider.blockSignals(True); editor.slider.setValue(round(editor.position*1000))
        editor.slider.blockSignals(False)
        editor.stamp.setText(f'{editor.position:.3f} / {duration:.3f} s')
        if editor.isVisible():editor.pose()
    editor.timer.stop(); editor.schedule_markers()


def mapping(value):
    return value if isinstance(value, dict) else {}


class EditorSettings:
    def __init__(self, window, path=DEFAULT_PATH):
        self.window, self.path = window, Path(path)
        self.data = {}; self.drafts = {}; self.error = ''
        try:self.data = read(self.path)
        except (OSError, ValueError) as error:self.error = '设置读取失败，使用默认值：'+str(error)

    def remember_draft(self, editor):
        if editor.key and editor.trace is not None:
            workflow = editor.control.workflow
            self.drafts.setdefault(workflow, {})[editor.key[1]] = draft(editor)

    def change_slot(self, editor):
        self.remember_draft(editor)
        editor.refresh()
        restore_draft(editor, self.drafts.get(editor.control.workflow, {}).get(editor.slot()))

    def restore(self):
        w = self.window
        geometry = self.data.get('geometry')
        if isinstance(geometry, str):w.restoreGeometry(QtCore.QByteArray.fromBase64(geometry.encode()))
        depth = getattr(w.depth_reference, 'restore_settings', None)
        if depth:depth(mapping(self.data.get('depth')))
        for slots, editor, history in w.pages:
            page = mapping(mapping(self.data.get('pages')).get(editor.control.workflow))
            self.drafts[editor.control.workflow] = mapping(page.get('drafts')).copy()
            index = slots.findData(page.get('slot'))
            if index >= 0:
                slots.blockSignals(True); slots.setCurrentIndex(index); slots.blockSignals(False)
            editor.refresh()
            restore_values(editor, page.get('parameters'))
            restore_draft(editor, self.drafts[editor.control.workflow].get(editor.slot()))
            index = history.path.findData(page.get('history'))
            if index >= 0:history.path.setCurrentIndex(index)
        tab = self.data.get('tab')
        if type(tab) is int and 0 <= tab < w.tabs.count():w.tabs.setCurrentIndex(tab)

    def save(self):
        from task_screen_teach import write_json
        w = self.window
        data = dict(schema=1, geometry=bytes(w.saveGeometry().toBase64()).decode(),
                    tab=w.tabs.currentIndex(), pages={})
        depth = getattr(w.depth_reference, 'settings', None)
        if depth:data['depth'] = depth()
        for slots, editor, history in w.pages:
            self.remember_draft(editor)
            data['pages'][editor.control.workflow] = dict(slot=slots.currentData(),
                parameters=values(editor), drafts=self.drafts.get(editor.control.workflow, {}),
                history=history.path.currentData())
        write_json(self.path, data)


class ControlEditorSettings(QtCore.QObject):
    """Share editor preferences with the live UI, never its motion permissions."""
    def __init__(self, window):
        super().__init__(window)
        self.window = window
        self.timer = QtCore.QTimer(self); self.timer.setSingleShot(True); self.timer.setInterval(500)
        self.timer.timeout.connect(self.save)
        try:data = read(DEFAULT_PATH)
        except (OSError, ValueError):data = {}
        self.editors = [getattr(window,name).editor for name in ('demo_panel','rail_panel','landing_gear_panel') if hasattr(window,name)]
        for editor in self.editors:
            restore_values(editor, mapping(mapping(data.get('pages')).get(editor.control.workflow)).get('parameters'))
            for name in PARAMETERS:
                widget = getattr(editor,name,None)
                if widget is None:continue
                signal = (widget.currentIndexChanged if isinstance(widget,QtWidgets.QComboBox) else
                          widget.toggled if isinstance(widget,QtWidgets.QCheckBox) else widget.valueChanged)
                signal.connect(lambda *_:self.timer.start())
        depth = window.depth_reference
        depth.restore_settings(mapping(data.get('depth')))
        depth.visible.toggled.connect(lambda *_:self.timer.start())
        depth.sources.currentIndexChanged.connect(lambda *_:self.timer.start())
        for spin in depth.offsets:spin.valueChanged.connect(lambda *_:self.timer.start())

    def save(self):
        from task_screen_teach import write_json
        try:
            try:data = read(DEFAULT_PATH)
            except (OSError,ValueError):data = {}
            data['schema'] = 1
            pages = mapping(data.get('pages')); data['pages'] = pages
            for editor in self.editors:
                page = mapping(pages.get(editor.control.workflow)); pages[editor.control.workflow] = page
                page['parameters'] = values(editor)
            data['depth'] = self.window.depth_reference.settings()
            write_json(DEFAULT_PATH,data)
        except (OSError,ValueError) as error:self.window.node.status = '编辑设置保存失败：'+str(error)
