"""Qt Push interaction tests; no ROS node, SSH connection or robot command."""
import os
os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

import threading
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from PyQt5 import QtWidgets
from task_jetson_push_panel import JetsonPushPanel


class PushPanelTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    def setUp(self):
        self.node = SimpleNamespace(lock=threading.RLock(), execute=Mock(),
            session=SimpleNamespace(generation=7, plan=dict(kind='dring_demo',
                demo=dict(workflow='landing_gear', operation='lower_to_upper'), directory='/sessions/s1/p1')),
            launch_work=Mock(side_effect=lambda work: work()))
        self.window = SimpleNamespace(node=self.node, invoke=lambda fn: fn())
        self.panel = JetsonPushPanel(self.window, 'landing_gear')
        self.issue = patch('task_jetson_push_panel.issue', return_value=None).start()
        self.upload = patch('task_jetson_push_panel.push', return_value=dict(version='test-v1', sha256='a'*64)).start()
        self.question = patch('task_jetson_push_panel.QtWidgets.QMessageBox.question',
                              return_value=QtWidgets.QMessageBox.Yes).start()
        self.addCleanup(patch.stopall)
        self.addCleanup(self.panel.deleteLater)

    def test_button_uploads_captured_generation_without_execute(self):
        self.panel.refresh(False)
        self.panel.button.click()
        self.upload.assert_called_once()
        self.assertEqual(self.upload.call_args.args, (self.node, 'landing_gear', 7))
        self.node.execute.assert_not_called()
        self.assertEqual(self.node.session.generation, 7)
        self.assertIn('test-v1', self.panel.status.text())
        self.assertFalse(self.panel.running)

    def test_confirmation_identifies_independent_segment(self):
        self.panel.start()
        message = self.question.call_args.args[2]
        self.assertIn('gear_lower_to_upper', message)
        self.assertIn('s1/p1', message)

    def test_decline_does_not_upload_or_execute(self):
        self.question.return_value = QtWidgets.QMessageBox.No
        self.panel.start()
        self.upload.assert_not_called()
        self.node.launch_work.assert_not_called()
        self.node.execute.assert_not_called()

    def test_busy_or_invalid_plan_disables_push(self):
        self.panel.refresh(True)
        self.assertFalse(self.panel.button.isEnabled())
        self.issue.return_value = 'stale plan'
        self.panel.refresh(False)
        self.assertFalse(self.panel.button.isEnabled())
        with self.assertRaisesRegex(RuntimeError, 'stale plan'):
            self.panel.start()
        self.upload.assert_not_called()

    def test_failed_upload_preserves_plan_and_allows_retry(self):
        original = self.node.session.plan
        self.upload.side_effect = RuntimeError('upload timeout')
        self.panel.start()
        self.assertFalse(self.panel.running)
        self.assertIn('upload timeout', self.panel.status.text())
        self.assertIs(self.node.session.plan, original)
        self.node.execute.assert_not_called()
        self.panel.refresh(False)
        self.assertTrue(self.panel.button.isEnabled())

    def test_change_during_confirmation_keeps_old_generation_for_backend_guard(self):
        def answer(*args):
            self.node.session.generation = 8
            return QtWidgets.QMessageBox.Yes
        self.question.side_effect = answer
        self.upload.side_effect = RuntimeError('stale generation')
        self.panel.start()
        self.assertEqual(self.upload.call_args.args[2], 7)
        self.assertIn('stale generation', self.panel.status.text())
        self.node.execute.assert_not_called()


if __name__ == '__main__':
    unittest.main()
