import json
from pathlib import Path
from tempfile import TemporaryDirectory
import threading
import time
from types import SimpleNamespace as S
import unittest
from unittest.mock import Mock,patch

from real_task_app import TaskNode
from task_panel_control import attachment,refresh_session_context
from task_console_handoff import validate_handoff
from task_session import ARM_NAMES,Session


class PanelAttachmentTests(unittest.TestCase):
    def context(self):
        return dict(gateway_identity=dict(pid=123,start_ticks=456),source_session='/saved/session',
                    session_home=dict(operator_confirmed_r2_a=True,joints=dict.fromkeys(ARM_NAMES,.1)),
                    ready_poses={'left':None,'right':None})

    def gateway(self,phase='FAULT_LATCHED',stopped=True):
        return dict(phase=phase,task_stop_active='true' if stopped else 'false',
                    task_stop_epoch='7',armed='false',upper_body_state_safe='false')

    def test_unhealthy_gateway_can_open_panel_but_cannot_use_hold_handoff(self):
        context=self.context()
        with patch('task_panel_control.check_identity'),\
                patch('task_console_handoff.process_identity',return_value=context['gateway_identity']):
            manifest=attachment(context,self.gateway(),panel_only=True)
            self.assertEqual(manifest['attachment_mode'],'panel')
            self.assertEqual(manifest['stop_epoch'],7)
            self.assertIsNone(manifest['session_home'])
            self.assertTrue(manifest['startup_capture_pending'])
            self.assertEqual(manifest['ready_poses'],context['ready_poses'])
            with self.assertRaisesRegex(RuntimeError,'healthy'):
                attachment(context,self.gateway())

    def test_only_explicit_resume_imports_confirmed_home(self):
        context=self.context()
        with patch('task_panel_control.check_identity'),\
                patch('task_console_handoff.process_identity',return_value=context['gateway_identity']):
            manifest=attachment(context,self.gateway(),panel_only=True,resume_session=True)
        self.assertEqual(manifest['session_home'],context['session_home'])

    def test_context_refresh_uses_latest_capture_and_never_resurrects_old_home(self):
        with TemporaryDirectory() as directory:
            context=self.context();context.update(source_session=directory,console_command=[])
            root=Path(directory)
            (root/'ready_poses.json').write_text(json.dumps(context['ready_poses']))
            refresh_session_context(context)
            self.assertIsNone(context['session_home'])
            record=dict(operator_confirmed_r2_a=True,joints=dict.fromkeys(ARM_NAMES,.3))
            (root/'session_home.json').write_text(json.dumps(record))
            refresh_session_context(context)
            self.assertEqual(context['session_home'],record)

    def test_new_panel_restore_requires_capture_and_preserves_stop_without_commands(self):
        with TemporaryDirectory() as directory:
            context=self.context();session=Session()
            node=S(handoff=dict(context,session_home=None,startup_capture_pending=True),
                   session=session,run_dir=Path(directory),prepared=context['ready_poses'],
                   display_pub=Mock(),call=Mock(),arm=Mock())
            TaskNode.restore_attachment(node,True,'LATCHED_SILENT',7,'Connected')
            self.assertIsNone(session.home)
            self.assertTrue(session.stopped)
            self.assertIn('capture',session.planning_issue())
            self.assertFalse((node.run_dir/'session_home.json').exists())
            node.call.assert_not_called();node.arm.assert_not_called()

    def test_panel_open_is_not_a_wall_clock_limited_motion_handoff(self):
        context=self.context()
        manifest=dict(context,attachment_mode='panel',stop_epoch=7,created_at_epoch_s=100.)
        with patch('task_console_handoff.process_identity',return_value=context['gateway_identity']):
            for now in (98.,100.,300.):
                self.assertEqual(validate_handoff(manifest,now)['attachment_mode'],'panel')
                if now!=100.:
                    with self.assertRaisesRegex(RuntimeError,'expired'):
                        validate_handoff(dict(manifest,attachment_mode='hold'),now)

    def test_panel_restore_needs_no_joint_freshness_or_stationary_pose_and_sends_no_control(self):
        for phase,stopped in (('FAULT_LATCHED',True),('UNARMED',False),('TASK_ADMITTANCE',True)):
            with self.subTest(phase=phase),TemporaryDirectory() as directory:
                context=self.context();session=Session()
                session.measured=Mock(side_effect=RuntimeError('No robot state'))
                manifest=dict(context,attachment_mode='panel',stop_epoch=6)
                node=S(handoff=manifest,session=session,lock=threading.RLock(),check=Mock(),
                       gateway=self.gateway(phase,stopped),gateway_received=time.monotonic(),
                       run_dir=Path(directory),prepared=context['ready_poses'],display_pub=Mock(),
                       arm=Mock(),stop=Mock(),recovery=Mock(),call=Mock(),attachment_complete=False)
                node.restore_attachment=lambda *args:TaskNode.restore_attachment(node,*args)
                with patch('real_task_app.process_identity',return_value=context['gateway_identity']),\
                        patch('real_task_app.supported_gateway') as supported:
                    TaskNode.finish_attachment(node)
                supported.assert_not_called();session.measured.assert_not_called()
                for method in (node.arm,node.stop,node.recovery,node.call):method.assert_not_called()
                self.assertTrue(node.attachment_complete)
                self.assertEqual(session.phase,phase)
                self.assertEqual(session.stopped,stopped)
                self.assertEqual(node.stop_expected_epoch,7)
                self.assertIsNone(session.plan)
                self.assertEqual(json.loads((Path(directory)/'session_home.json').read_text()),context['session_home'])

    def test_panel_restore_still_rejects_a_replaced_gateway(self):
        context=self.context()
        node=S(handoff=dict(context,attachment_mode='panel'),lock=threading.RLock(),check=Mock(),
               gateway=self.gateway(),gateway_received=time.monotonic(),restore_attachment=Mock())
        with patch('real_task_app.process_identity',return_value=dict(pid=123,start_ticks=999)):
            with self.assertRaisesRegex(RuntimeError,'changed'):TaskNode.finish_attachment(node)
        node.restore_attachment.assert_not_called()


if __name__=='__main__':unittest.main()
