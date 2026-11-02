"""History browser for complete D-ring, rail and landing-gear plans."""
from pathlib import Path

from PyQt5 import QtWidgets


class RecordedHistoryPanel(QtWidgets.QWidget):
    def __init__(self, window, control):
        super().__init__()
        self.window, self.control = window, control
        layout = QtWidgets.QVBoxLayout(self); layout.setContentsMargins(0, 0, 0, 0)
        self.path = QtWidgets.QComboBox(); self.path.setMinimumContentsLength(20)
        self.path.setSizeAdjustPolicy(QtWidgets.QComboBox.AdjustToMinimumContentsLengthWithIcon)
        layout.addWidget(self.path)
        row = QtWidgets.QHBoxLayout(); layout.addLayout(row)
        self.buttons = []
        for label, icon, action in (
                ('', QtWidgets.QStyle.SP_BrowserReload, self.scan),
                ('', QtWidgets.QStyle.SP_DialogOpenButton, self.browse),
                ('历史 RViz', QtWidgets.QStyle.SP_MediaPlay, lambda:self.load(True)),
                ('载入历史执行', QtWidgets.QStyle.SP_DialogApplyButton, lambda:self.load(False))):
            b = QtWidgets.QPushButton(label); b.setIcon(self.style().standardIcon(icon))
            b.setToolTip(label or ('刷新历史列表' if icon == QtWidgets.QStyle.SP_BrowserReload else '选择历史 plan.json'))
            b.clicked.connect(lambda _=False, action=action:window.invoke(action))
            row.addWidget(b); self.buttons.append(b)
        self.scan()

    def scan(self):
        root = self.control.node.run_dir.parent
        pattern = {'dring_demo':'dring_demo_*', 'landing_gear':'landing_gear_*', 'rail':'rail_*'}[self.control.workflow]
        selected = self.path.currentData()
        paths = sorted(root.glob('*/'+pattern+'/plan.json'), key=lambda p:p.stat().st_mtime_ns, reverse=True)
        self.path.clear()
        for p in paths:
            if not (p.parent/'config.json').is_file():continue
            self.path.addItem(p.parent.parent.name+' / '+p.parent.name, str(p))
            self.path.setItemData(self.path.count()-1, str(p), 3)
        if selected is not None:
            index = self.path.findData(selected)
            if index >= 0:self.path.setCurrentIndex(index)

    def browse(self):
        path, _ = QtWidgets.QFileDialog.getOpenFileName(self, '历史计划', str(self.control.node.run_dir.parent), 'JSON (*.json)')
        if path:
            self.path.addItem(str(Path(path).parent), path); self.path.setCurrentIndex(self.path.count()-1)

    def load(self, preview):
        if not self.path.currentData():raise ValueError('请选择已计算完成的历史计划')
        from task_recorded_history import load
        load(self.control, self.path.currentData(), preview_only=preview)

    def refresh(self, busy):
        enabled = not busy and self.control.recording is None
        self.path.setEnabled(enabled)
        for button in self.buttons:button.setEnabled(enabled)
