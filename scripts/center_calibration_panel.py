"""Standalone Qt measurement notebook; deliberately has no control connection."""
import argparse
from datetime import datetime, timezone
import hashlib
from pathlib import Path
import sys

from PyQt5 import QtCore, QtWidgets

from center_calibration_data import (atomic_json, export_csv, fit_samples, load_samples,
                                     observation, save_samples, scan_sources)


ROOT = Path(__file__).resolve().parents[1]
ROLES = [('参与拟合', 'fit'), ('独立验证', 'validation'), ('排除', 'excluded')]


def xyz(values):
    return ' / '.join(f'{v:+.2f}' for v in values)


class Window(QtWidgets.QMainWindow):
    def __init__(self, sessions, database):
        super().__init__()
        self.sessions, self.database = Path(sessions), Path(database)
        self.samples = load_samples(self.database)
        self.database_version = self.file_version()
        self.sources = []
        self.selected_sample_id = None
        self.dirty = False
        self.setWindowTitle('圆心定位标定 · 离线记录')
        self.resize(1040, 790)
        self.setMinimumSize(680, 550)
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        outer = QtWidgets.QVBoxLayout(central)
        outer.setContentsMargins(0, 0, 0, 0)
        scroll = self.scroll = QtWidgets.QScrollArea()
        scroll.setWidgetResizable(True)
        outer.addWidget(scroll, 1)
        body = QtWidgets.QWidget()
        scroll.setWidget(body)
        layout = QtWidgets.QVBoxLayout(body)
        toolbar = QtWidgets.QHBoxLayout()
        heading = QtWidgets.QLabel('圆心定位标定')
        heading.setStyleSheet('font-size: 20px; font-weight: 600;')
        toolbar.addWidget(heading)
        toolbar.addStretch()
        self.button(toolbar, '刷新记录', self.refresh_sources, QtWidgets.QStyle.SP_BrowserReload)
        self.button(toolbar, '导出 CSV', self.export, QtWidgets.QStyle.SP_DialogSaveButton)
        layout.addLayout(toolbar)
        self.source_combo = QtWidgets.QComboBox()
        self.source_combo.setSizeAdjustPolicy(QtWidgets.QComboBox.AdjustToMinimumContentsLengthWithIcon)
        self.source_combo.setMinimumContentsLength(20)
        layout.addWidget(self.source_combo)
        self.source_info = QtWidgets.QLabel()
        self.source_info.setWordWrap(True)
        self.source_info.setTextFormat(QtCore.Qt.PlainText)
        layout.addWidget(self.source_info)
        form = QtWidgets.QFormLayout()
        self.datum = QtWidgets.QLineEdit('食指末端中心')
        self.forward = QtWidgets.QLineEdit()
        self.left = QtWidgets.QLineEdit()
        self.height = QtWidgets.QLineEdit()
        self.note = QtWidgets.QLineEdit()
        self.role = QtWidgets.QComboBox()
        for label, value in ROLES:
            self.role.addItem(label, value)
        for label, widget in [('实体测量点', self.datum), ('前后偏差 mm（前 + / 后 −）', self.forward),
                              ('左右偏差 mm（左 + / 右 −）', self.left),
                              ('距圆心表面高度 mm（上 + / 下 −）', self.height),
                              ('样本用途', self.role), ('备注 / 安装状态', self.note)]:
            form.addRow(label, widget)
        layout.addLayout(form)
        sample_actions = QtWidgets.QHBoxLayout()
        sample_actions.addWidget(QtWidgets.QLabel('已记录样本'))
        sample_actions.addStretch()
        self.button(sample_actions, '删除选中样本', self.delete, QtWidgets.QStyle.SP_TrashIcon)
        layout.addLayout(sample_actions)
        self.table = QtWidgets.QTableWidget(0, 6)
        self.table.setHorizontalHeaderLabels(['规划记录', '用途', '实测偏差 XYZ / mm', '扣除 FK 后 / mm', '残差 3D / mm', '测量点'])
        self.table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        self.table.setSelectionMode(QtWidgets.QAbstractItemView.SingleSelection)
        self.table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.table.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.ResizeToContents)
        self.table.horizontalHeader().setStretchLastSection(True)
        self.table.setMinimumHeight(150)
        layout.addWidget(self.table)
        self.output = QtWidgets.QPlainTextEdit()
        self.output.setReadOnly(True)
        self.output.setMinimumHeight(180)
        layout.addWidget(self.output)
        footer = QtWidgets.QWidget()
        actions = QtWidgets.QHBoxLayout(footer)
        actions.setContentsMargins(12, 8, 12, 8)
        self.save_button = self.button(actions, '保存新样本', self.save, QtWidgets.QStyle.SP_DialogSaveButton)
        self.save_button.setMinimumSize(150, 42)
        self.save_button.setStyleSheet(
            'QPushButton {background: #137763; color: white; border: 0; border-radius: 4px; font-weight: 600;}'
            'QPushButton:hover {background: #10634f;}'
            'QPushButton:pressed {background: #0b5040;}'
            'QPushButton:disabled {background: #777777; color: #eeeeee;}')
        self.save_state = QtWidgets.QLabel()
        self.save_state.setWordWrap(True)
        actions.addWidget(self.save_state, 1)
        self.update_button = self.button(actions, '修改选中样本', self.update_selected, QtWidgets.QStyle.SP_FileDialogDetailedView)
        self.update_button.setEnabled(False)
        self.button(actions, '计算补偿', self.calculate, QtWidgets.QStyle.SP_ArrowRight)
        outer.addWidget(footer)
        for field in (self.forward, self.left, self.height, self.datum, self.note):
            field.textEdited.connect(self.mark_unsaved)
        self.role.activated.connect(self.mark_unsaved)
        self.source_combo.currentIndexChanged.connect(self.source_changed)
        self.table.itemSelectionChanged.connect(self.select_sample)
        self.refresh_sources()
        self.refresh_table()
        self.show_fit()

    def button(self, layout, text, callback, icon):
        button = QtWidgets.QPushButton(self.style().standardIcon(icon), text)
        button.clicked.connect(lambda: self.guard(callback))
        layout.addWidget(button)
        return button

    def guard(self, callback):
        try:
            callback()
        except Exception as error:
            QtWidgets.QMessageBox.warning(self, '未完成', str(error))

    def mark_unsaved(self, *_):
        self.dirty = True
        self.save_state.setText('有未保存的修改')

    def file_version(self):
        return hashlib.sha256(self.database.read_bytes()).hexdigest() if self.database.exists() else None

    def commit_samples(self, samples):
        if self.file_version() != self.database_version:
            raise ValueError('测量文件已被其他窗口修改，未覆盖磁盘数据。请记录当前输入后关闭并重新打开标定窗口。')
        save_samples(self.database, samples)
        self.database_version = self.file_version()
        self.samples = samples

    def refresh_sources(self):
        previous = self.source_combo.currentData()
        has_input = self.dirty or any(field.text().strip() for field in (self.forward, self.left, self.height))
        sources, errors = scan_sources(self.sessions)
        # Archived samples remain editable even when their source session is moved.
        known = {s['source_id'] for s in sources}
        for sample in self.samples:
            if sample['source']['source_id'] not in known:
                sources.append(sample['source'])
                known.add(sample['source']['source_id'])
        self.sources = sources
        self.source_combo.blockSignals(True)
        self.source_combo.clear()
        for s in self.sources:
            self.source_combo.addItem(f"{s['source_id']} · {s['arm']} · 目标高度 {s['height_mm']:g} mm", s['source_id'])
        index = self.source_combo.findData(previous) if has_input or self.selected_sample_id is not None else 0
        self.source_combo.setCurrentIndex(max(0, index) if self.sources else -1)
        self.source_combo.blockSignals(False)
        # Keep unsaved measurements when refresh did not change the selected plan.
        if previous != self.source_combo.currentData() or previous is None:
            self.source_changed()
        self.statusBar().showMessage(f'{len(sources)} 条到位记录；{len(errors)} 条无效/未到位记录；数据：{self.database}')
        self.source_combo.setToolTip('\n'.join(errors))

    def source_changed(self):
        index = self.source_combo.currentIndex()
        self.save_button.setEnabled(index >= 0)
        if self.selected_sample_id is None:
            for field in (self.forward, self.left, self.height, self.note):
                field.clear()
            self.role.setCurrentIndex(0)
            self.dirty = False
        if index < 0:
            self.source_info.setText('暂无已到位的 Center check 记录')
            self.save_state.setText('暂无可关联的到位记录')
            return
        s = self.sources[index]
        self.source_info.setText(f"圆心 pelvis XYZ / mm：{xyz([v * 1000 for v in s['center_pelvis_m']])}\n"
                                 f"模型跟踪偏差 XYZ / mm：{xyz(s['fk_error_mm'])}")
        self.save_state.setText('已选中已有样本' if self.selected_sample_id else '新样本 · 尚未保存')

    def refresh_table(self):
        self.table.blockSignals(True)
        self.table.setRowCount(len(self.samples))
        for row, s in enumerate(self.samples):
            values = [f"#{row + 1} · {s['source']['source_id']}", dict((v, k) for k, v in ROLES)[s['role']],
                      xyz(s['physical_error_mm']), xyz(s['geometric_error_mm']), '', s['datum']]
            for column, value in enumerate(values):
                self.table.setItem(row, column, QtWidgets.QTableWidgetItem(value))
        self.table.blockSignals(False)

    def select_sample(self):
        row = self.table.currentRow()
        if row >= 0 and self.table.selectedItems():
            sample = self.samples[row]
            self.selected_sample_id = sample['sample_id']
            index = self.source_combo.findData(sample['source']['source_id'])
            self.source_combo.setCurrentIndex(index)
            self.source_changed()
            self.datum.setText(sample['datum'])
            for field, value in zip((self.forward, self.left, self.height), sample['measured_forward_left_height_mm']):
                field.setText(f'{value:g}')
            self.role.setCurrentIndex(self.role.findData(sample['role']))
            self.note.setText(sample['note'])
            self.update_button.setEnabled(True)
            self.dirty = False

    def entered_sample(self):
        index = self.source_combo.currentIndex()
        if index < 0:
            raise ValueError('没有可关联的到位记录')
        try:
            numbers = [float(f.text().strip()) for f in (self.forward, self.left, self.height)]
        except ValueError:
            raise ValueError('请填写三个实测数值，单位为毫米；零偏差请填 0')
        source = self.sources[index]
        return observation(source, *numbers, self.datum.text(), self.role.currentData(), self.note.text())

    def finish_save(self, message):
        self.selected_sample_id = None
        self.update_button.setEnabled(False)
        self.dirty = False
        self.table.blockSignals(True)
        self.table.clearSelection()
        self.table.blockSignals(False)
        self.refresh_table()
        self.source_changed()
        self.show_fit()
        self.save_state.setText(f'{message} · 共 {len(self.samples)} 条')
        self.statusBar().showMessage(f'{message}；历史备份：{self.database.stem}_history')

    def save(self):
        sample = self.entered_sample()
        self.commit_samples(self.samples + [sample])
        self.finish_save('已新增样本')

    def update_selected(self):
        old = next((s for s in self.samples if s['sample_id'] == self.selected_sample_id), None)
        if old is None:
            raise ValueError('请先在表格选中要修改的样本；新增请点“保存新样本”')
        sample = self.entered_sample()
        if sample['source']['source_id'] == old['source']['source_id']:
            sample = observation(old['source'], *sample['measured_forward_left_height_mm'],
                                 sample['datum'], sample['role'], sample['note'])
        sample['sample_id'] = old['sample_id']
        sample['recorded_at'] = old['recorded_at']
        sample['updated_at'] = datetime.now(timezone.utc).isoformat()
        self.commit_samples([sample if s['sample_id'] == old['sample_id'] else s for s in self.samples])
        self.finish_save('已修改选中样本')

    def delete(self):
        row = next((i for i, s in enumerate(self.samples) if s['sample_id'] == self.selected_sample_id), None)
        if row is None:
            raise ValueError('请先选择一个样本')
        if QtWidgets.QMessageBox.question(self, '删除样本', self.samples[row]['source']['source_id']) != QtWidgets.QMessageBox.Yes:
            return
        changed = self.samples[:row] + self.samples[row + 1:]
        self.commit_samples(changed)
        self.finish_save('已删除选中样本')

    def show_fit(self):
        for row in range(self.table.rowCount()):
            self.table.item(row, 4).setText('--')
        try:
            result = fit_samples(self.samples)
        except ValueError as error:
            self.output.setPlainText(str(error))
            return
        residuals = {r['sample_id']: r['residual_3d_mm'] for r in result['residuals']}
        for row, sample in enumerate(self.samples):
            value = residuals.get(sample['sample_id'])
            if value is not None:
                self.table.item(row, 4).setText(f'{value:.2f}')
        lines = [f"固定偏移估计 · 拟合 {result['fit_count']} 点 / 独立验证 {result['validation_count']} 点",
                 f"实测总偏差均值 XYZ / mm：{xyz(result['mean_physical_error_mm'])}",
                 f"扣除 FK 后偏差 XYZ / mm：{xyz(result['geometric_bias_mm'])}",
                 f"建议目标补偿 XYZ / mm：{xyz(result['suggested_target_offset_mm'])}（未应用）",
                 f"拟合残差 3D RMS：{result['fit_after']['rmse_3d_mm']:.2f} mm；最大：{result['fit_after']['max_3d_mm']:.2f} mm"]
        if result['validation_after']:
            lines.append(f"独立验证 3D RMS：{result['validation_before']['rmse_3d_mm']:.2f} → {result['validation_after']['rmse_3d_mm']:.2f} mm（预测）")
        lines.extend(result['warnings'])
        self.output.setPlainText('\n'.join(lines))

    def calculate(self):
        result = fit_samples(self.samples)
        # Export the exact observation set alongside fit provenance for reproducibility.
        result['samples'] = self.samples
        path = self.database.with_name(self.database.stem + '_fit.json')
        atomic_json(path, result)
        self.show_fit()
        self.statusBar().showMessage(f'计算结果已保存：{path}；未应用到机器人或规划配置')

    def export(self):
        path, _ = QtWidgets.QFileDialog.getSaveFileName(self, '导出测量', str(self.database.with_suffix('.csv')), 'CSV (*.csv)')
        if path:
            export_csv(path, self.samples)
            self.statusBar().showMessage(f'已导出：{path}')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--sessions', type=Path, default=ROOT / 'data/task_sessions')
    parser.add_argument('--data', type=Path, default=ROOT / 'data/center_calibration/measurements.json')
    args = parser.parse_args()
    app = QtWidgets.QApplication(sys.argv[:1])
    args.data = args.data.resolve()
    args.data.parent.mkdir(parents=True, exist_ok=True)
    lock = QtCore.QLockFile(str(args.data) + '.lock')
    if not lock.tryLock(0):
        QtWidgets.QMessageBox.warning(None, '标定面板', '这份测量数据已在另一个面板打开')
        return 1
    try:
        window = Window(args.sessions, args.data)
    except Exception as error:
        QtWidgets.QMessageBox.critical(None, '读取失败', str(error))
        return 1
    window.show()
    return app.exec_()


if __name__ == '__main__':
    sys.exit(main())
