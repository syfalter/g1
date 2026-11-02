"""Shared Push control embedded in the existing Qt teaching panels."""
from PyQt5 import QtCore, QtWidgets

from task_jetson_push import action_name, issue, push


class JetsonPushPanel(QtWidgets.QWidget):
    progress = QtCore.pyqtSignal(str)
    finished = QtCore.pyqtSignal(object)

    def __init__(self, window, workflow):
        super().__init__()
        self.window, self.workflow = window, workflow
        self.running = False
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        self.button = QtWidgets.QPushButton('Push 到 Jetson')
        self.button.setIcon(self.style().standardIcon(QtWidgets.QStyle.SP_ArrowUp))
        self.button.setToolTip('上传当前规划为待启用版本；不执行、不切换控制权')
        self.button.clicked.connect(lambda: window.invoke(self.start))
        layout.addWidget(self.button)
        self.status = QtWidgets.QLabel('尚未上传')
        self.status.setWordWrap(True)
        layout.addWidget(self.status)
        self.progress.connect(self.status.setText)
        self.finished.connect(self.complete)

    def refresh(self, busy):
        with self.window.node.lock:
            problem = issue(self.window.node, self.workflow)
        self.button.setEnabled(not busy and not self.running and problem is None)
        self.button.setToolTip(problem or '上传当前规划为待启用版本；不执行、不切换控制权')

    def start(self):
        node = self.window.node
        if self.running:
            raise RuntimeError('Push 正在进行')
        with node.lock:
            problem = issue(node, self.workflow)
            if problem:
                raise RuntimeError(problem)
            generation = node.session.generation
            plan = node.session.plan
            name = action_name(plan)
            source = plan['directory'].rsplit('/', 2)[-2:]
            slot = plan.get('demo', {}).get('operation', name)
        answer = QtWidgets.QMessageBox.question(self, 'Push 当前规划',
            f'目标动作：{name}\n录制槽：{slot}\n来源：{"/".join(source)}\n\n'
            '本次上传标记为仅规划、未实机验证。Jetson 仅接收待启用版本，不运动，不替换其他模块。',
            QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No, QtWidgets.QMessageBox.No)
        if answer != QtWidgets.QMessageBox.Yes:
            return
        self.running = True
        self.button.setEnabled(False)
        def work():
            try:
                result = push(node, self.workflow, generation, notify=self.progress.emit)
            except Exception as error:
                self.finished.emit(dict(error=str(error)))
            else:
                self.finished.emit(result)
        try:
            node.launch_work(work)
        except Exception:
            self.running = False
            raise

    def complete(self, result):
        self.running = False
        if 'error' in result:
            self.status.setText(result['error'])
        else:
            self.status.setText('Jetson 已接收，待启用；未发送运动\n' + result['version'] +
                                '\nSHA256 ' + result['sha256'][:16])
