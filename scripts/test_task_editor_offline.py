"""Offline UI tests use real recordings and mock publishers, never robot DDS."""
import os
from pathlib import Path
import threading
import tempfile
from types import SimpleNamespace as S
import unittest
from unittest.mock import Mock, patch

from PyQt5 import QtWidgets

from task_editor_offline import launch_command


class LaunchTests(unittest.TestCase):
    def test_arrow_direction_is_endpoint_to_fingertip(self):
        import numpy as np
        from task_trace_markers import arrow_vertices
        tip=np.array([[1.,2.,3.],[0.,0.,2.],[1.,1.,1.]])
        end=np.array([[1.,1.,3.],[0.,0.,0.],[1.,1.,1.]])
        arrows,valid=arrow_vertices(tip,end,.04)
        np.testing.assert_allclose(arrows[:2,1]-arrows[:2,0],[[0.,.04,0.],[0.,0.,.04]])
        np.testing.assert_array_equal(valid,[True,True,False])
        self.assertTrue(np.all(np.isfinite(arrows)))

    def test_sparse_points_keep_endpoints_and_limit(self):
        import numpy as np
        from task_trace_markers import sparse_indices
        pose=np.repeat(np.eye(4)[None],10001,axis=0)
        pose[:,0,3]=np.linspace(0.,1.,len(pose))
        indices=sparse_indices(pose,.02,30)
        self.assertEqual((indices[0],indices[-1]),(0,10000))
        self.assertLessEqual(len(indices),30)
        self.assertTrue(np.all(np.diff(indices)>0))

    def test_picker_ignores_drag_and_stale_feedback(self):
        from task_trace_picker import TracePicker
        from visualization_msgs.msg import InteractiveMarkerFeedback as Feedback
        with patch('task_trace_picker.InteractiveMarkerServer') as server:
            picker=TracePicker(Mock()); owner=object(); callback=Mock()
            picker.show(owner,[dict(position=(0.,0.,0.),time=1.)],callback)
            marker=server.return_value.insert.call_args.args[0]
            self.assertEqual(marker.controls[0].markers[0].scale.x,.018)
            feedback=server.return_value.insert.call_args.kwargs['feedback_callback']
            feedback(Feedback(event_type=Feedback.POSE_UPDATE));callback.assert_not_called()
            feedback(Feedback(event_type=Feedback.BUTTON_CLICK));callback.assert_called_once()
            picker.clear(owner)
            feedback(Feedback(event_type=Feedback.BUTTON_CLICK));callback.assert_called_once()
            picker.show(owner,[dict(position=(0.,0.,0.),time=1.)],callback,diameter=.03)
            marker=server.return_value.insert.call_args.args[0]
            self.assertEqual(marker.controls[0].markers[0].scale.x,.03)
            with self.assertRaises(ValueError):picker.show(owner,[],callback,diameter=0.)

    def test_no_hardware_processes(self):
        command = launch_command('/tmp/test.urdf')
        for option in ('start_state_bridge:=false', 'start_hardware_gateway:=false',
                       'hardware_output_enabled:=false', 'start_shadow_executor:=false',
                       'enable_rviz_interactive_execution:=false'):
            self.assertIn(option, command)
        self.assertIn('task_articulated_hands:=true', command)
        self.assertIn('task_waist_enabled:=true', command)
        import yaml
        root=Path(__file__).resolve().parents[1]
        config=yaml.safe_load((root/'assets/camera_reference/real_task.rviz').read_text())
        display=next(d for d in config['Visualization Manager']['Displays']
                     if d['Class']=='rviz_default_plugins/InteractiveMarkers')
        self.assertEqual(display['Interactive Markers Namespace'],'/g1/task/trace_points')
        self.assertNotIn('Update Topic',display)

    def test_display_decimation_keeps_cut_boundaries(self):
        import numpy as np
        from task_trace_markers import display_indices
        selected=np.zeros(23000,dtype=bool);selected[1234:5678]=True
        indices=display_indices(len(selected),selected)
        self.assertLessEqual(len(indices),2004)
        self.assertTrue({0,1233,1234,5677,5678,22999} <= set(indices))


@unittest.skipUnless(os.environ.get('G1_OFFLINE_TEST_URDF'), 'Set G1_OFFLINE_TEST_URDF to generated model')
class WindowTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    def test_saved_recordings_without_live_feedback(self):
        from workbench_model import OfflineModel
        from task_dring_demo_control import DringDemoControl
        from task_landing_gear_control import LandingGearControl
        from task_rail_control import RailControl
        from task_editor_offline_panel import OfflineEditorWindow
        root = Path(__file__).resolve().parents[1]
        urdf = os.environ['G1_OFFLINE_TEST_URDF']
        n = S(args=S(robot_urdf=urdf, hardware=False, disconnected=True),
              model=OfflineModel(urdf), lock=threading.RLock(),
              session=S(phase='IDLE', plan=None, current=None, home=None, generation=0,
                        invalidate=Mock(), stopped=False),
              display_pub=Mock(), demo_preview_pub=Mock(), marker_pub=Mock(),
              run_dir=root/'data/task_sessions/offline_test_not_written', worker=None,
              shutdown_requested=False, closing=threading.Event(),
              hand_control=False, hand_states={}, prepared={}, gateway={}, status='')
        n.demo = DringDemoControl(n); n.landing_gear = LandingGearControl(n)
        n.rail = RailControl(n); n.rail.restore_latest()
        temporary = tempfile.TemporaryDirectory(); self.addCleanup(temporary.cleanup)
        settings_path = Path(temporary.name)/'settings.json'
        with patch('task_depth_reference_panel.DepthReferencePanel', return_value=QtWidgets.QWidget()) as depth:
            depth.return_value.shutdown=Mock()
            window = OfflineEditorWindow(n, settings_path=settings_path,snapshot_directory=Path(temporary.name)/'previews')
        window.show()
        try:
            self.assertEqual([p[0].count() for p in window.pages], [2, 6, 6])
            for index, (_, editor, history) in enumerate(window.pages):
                window.tabs.setCurrentIndex(index); window.refresh(); self.app.processEvents()
                self.assertIsNotNone(editor.trace)
                self.assertIsNone(editor.point_limit.maximum())
                self.assertTrue(editor.smooth_button.isEnabled())
                self.assertTrue(history.buttons[-1].isHidden())
                self.assertGreater(history.path.count(), 0)
                editor.seek(0)
                n.demo_preview_pub.publish.assert_called()
                point=dict(trace=editor.trace,slot=editor.slot(),time=1.,
                           positions=editor.trace['raw']['positions'][0],label='test')
                n.demo_preview_pub.publish.reset_mock()
                editor.pick_point(point,editor.marker_serial-1)
                n.demo_preview_pub.publish.assert_not_called()
                editor.pick_point(point,editor.marker_serial)
                state=n.demo_preview_pub.publish.call_args.args[0].state.joint_state
                self.assertIn(editor.control.editor_side()+'_index_1_joint',state.name)
                self.assertAlmostEqual(editor.position,1.)
                editor.point_limit.setValue(500)
                editor.point_diameter.setValue(26.)
                editor.arrow_diameter.setValue(6.)
                editor.window_s.setValue(.35)
                editor.compare_reverse.setChecked(True)
                editor.start.setValue(.25); editor.end.setValue(.75)
                editor.key_times=[0.,.5,1.]; editor.key_list()
                editor.marker_timer.stop()
                if index == 0:
                    output = root/'data/trajectory_edit_audit/screenshots/offline_editor_offscreen.png'
                    output.parent.mkdir(parents=True, exist_ok=True)
                    self.assertTrue(window.grab().save(str(output)))
            self.assertIsNone(n.session.current)
            n.worker = S(is_alive=lambda:True)
            window.close(); self.assertTrue(window.isVisible())
            n.worker = None
            window.refresh(); self.assertFalse(window.isVisible())
            self.assertTrue(n.closing.is_set())
            self.assertTrue(settings_path.is_file())
            with patch('task_depth_reference_panel.DepthReferencePanel', return_value=QtWidgets.QWidget()) as depth:
                depth.return_value.shutdown=Mock()
                restored=OfflineEditorWindow(n, settings_path=settings_path,snapshot_directory=Path(temporary.name)/'previews')
            try:
                self.assertEqual(restored.tabs.currentIndex(),2)
                for slots,editor,_ in restored.pages:
                    self.assertEqual(editor.point_limit.value(),500)
                    self.assertEqual(editor.point_diameter.value(),26.)
                    self.assertEqual(editor.arrow_diameter.value(),6.)
                    self.assertEqual(editor.window_s.value(),.35)
                    self.assertTrue(editor.compare_reverse.isChecked())
                    self.assertEqual(editor.start.value(),.25)
                    self.assertEqual(editor.end.value(),.75)
                    self.assertEqual(editor.key_times,[0.,.5,1.])
                    self.assertEqual(editor.position,1.)
                    self.assertFalse(editor.timer.isActive())
                    original=slots.currentIndex()
                    slots.setCurrentIndex((original+1)%slots.count())
                    slots.setCurrentIndex(original)
                    self.assertEqual(editor.key_times,[0.,.5,1.])
                    from task_editor_settings import draft, restore_draft, restore_values
                    stale=draft(editor);stale['trace_id']='changed'
                    editor.key_times=[];restore_draft(editor,stale)
                    self.assertEqual(editor.key_times,[])
                    restore_values(editor,dict(point_limit=99999,window_s=float('nan')))
                    self.assertEqual(editor.point_limit.value(),99999)
                    self.assertEqual(editor.window_s.value(),.35)
            finally:restored.close()
        finally:
            n.worker = None; window.close(); self.app.processEvents()
            for _, editor, _ in window.pages:
                if editor.marker_thread:editor.marker_thread.join(timeout=20)


if __name__ == '__main__':unittest.main()
