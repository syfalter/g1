"""Editing-only window; no recording, hand, gateway or execution controls."""
from PyQt5 import QtCore, QtWidgets

from task_trace_editor_panel import TraceEditorPanel
from task_recorded_history_panel import RecordedHistoryPanel


class OfflineEditorWindow(QtWidgets.QWidget):
    def __init__(self, node, *, settings_path=None, snapshot_directory=None):
        super().__init__()
        self.node = node
        self.close_requested = False
        self.setWindowTitle('G1 - 离线轨迹编辑')
        self.resize(900, 850)
        outer = QtWidgets.QVBoxLayout(self)
        from task_depth_reference_panel import DepthReferencePanel
        self.depth_reference = DepthReferencePanel(node)
        outer.addWidget(self.depth_reference)
        self.tabs = QtWidgets.QTabWidget()
        outer.addWidget(self.tabs)
        self.status = QtWidgets.QLabel('离线编辑'); self.status.setWordWrap(True)
        outer.addWidget(self.status)
        from task_dring_teach_data import SLOTS as DRING
        from task_landing_gear_data import SLOTS as GEAR
        from task_rail_trace import STAGES
        self.pages = []
        from task_editor_settings import EditorSettings, DEFAULT_PATH
        self.settings = EditorSettings(self, settings_path if settings_path is not None else DEFAULT_PATH)
        for label, control, labels in (
                ('拉环', node.demo, DRING), ('滑轨', node.rail, {k:v[0] for k,v in STAGES.items()}),
                ('起落架杆', node.landing_gear, GEAR)):
            body = QtWidgets.QWidget(); layout = QtWidgets.QVBoxLayout(body)
            slots = QtWidgets.QComboBox()
            traces = control.config.get('traces', {})
            for key, text in labels.items():
                if key in traces:slots.addItem(text, key)
            layout.addWidget(slots)
            editor = TraceEditorPanel(self, control, slots.currentData)
            layout.addWidget(editor)
            history = RecordedHistoryPanel(self, control)
            history.buttons[-1].hide()
            layout.addWidget(history); layout.addStretch()
            slots.currentIndexChanged.connect(lambda _=0, e=editor:self.settings.change_slot(e))
            scroll = QtWidgets.QScrollArea(); scroll.setWidgetResizable(True); scroll.setWidget(body)
            self.tabs.addTab(scroll, label)
            self.pages.append((slots, editor, history))
        from task_composition_panel import CompositionPanel
        self.composition=CompositionPanel(node)
        scroll=QtWidgets.QScrollArea();scroll.setWidgetResizable(True);scroll.setWidget(self.composition)
        self.tabs.addTab(scroll,'组合预览')
        from task_offline_optimizer_panel import OptimizerPanel
        self.optimizer=OptimizerPanel(node,snapshot_directory=snapshot_directory)
        scroll=QtWidgets.QScrollArea();scroll.setWidgetResizable(True);scroll.setWidget(self.optimizer)
        self.tabs.addTab(scroll,'优化对比')
        self.tabs.currentChanged.connect(self.tab_changed)
        self.settings.restore()
        self.restore_optimized_preview()
        if self.settings.error or self.optimizer.viewer.result is None:
            node.status = self.settings.error or '离线编辑'
        self.timer = QtCore.QTimer(self); self.timer.setInterval(200)
        self.timer.timeout.connect(self.refresh); self.timer.start()

    def restore_optimized_preview(self):
        if self.close_requested:return
        if self.optimizer.restore_startup():self.tabs.setCurrentIndex(self.tabs.count()-1)

    def invoke(self, action):
        try:action()
        except Exception as error:self.node.status = str(error)

    def tab_changed(self, index):
        for _, editor, _ in self.pages:
            editor.stop_playback()
        if self.composition.playing:self.composition.clear_native()
        self.composition.playing=False
        if self.optimizer.viewer.playing:self.optimizer.viewer.clear_native()
        self.optimizer.viewer.playing=False
        if index<len(self.pages):self.pages[index][1].schedule_markers()

    def refresh(self):
        busy = bool(self.node.worker and self.node.worker.is_alive())
        for slots, editor, history in self.pages:
            slots.setEnabled(not busy and not self.close_requested)
            editor.refresh(busy or self.close_requested)
            history.refresh(busy or self.close_requested)
        self.status.setText(str(self.node.status))
        self.composition.refresh(busy or self.close_requested)
        self.optimizer.refresh(busy or self.close_requested)
        if self.close_requested and not busy:self.close()

    def closeEvent(self, event):
        self.close_requested = True
        self.optimizer.cancel()
        if self.node.worker and self.node.worker.is_alive():
            self.node.status = '等待当前编辑保存完成后关闭'
            event.ignore(); return
        try:self.settings.save()
        except (OSError, ValueError) as error:
            self.close_requested = False
            self.node.status = '设置保存失败，未退出：'+str(error)
            event.ignore(); return
        for _, editor, _ in self.pages:
            editor.stop_playback(); editor.marker_timer.stop()
        self.composition.halt();self.composition.timer.stop()
        self.optimizer.halt();self.optimizer.viewer.timer.stop()
        self.node.closing.set()
        self.depth_reference.shutdown()
        event.accept()
