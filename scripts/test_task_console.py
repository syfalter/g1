"""Qt state checks with a fake node: no ROS initialization or hardware clients."""
import json
import os
from pathlib import Path
import threading
import tempfile
import time
from types import SimpleNamespace as S
import unittest
from unittest.mock import Mock,patch

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
try:
    from PyQt5 import QtWidgets
    from real_task_app import Window, TaskNode
except ImportError:
    Window = None

from task_session import Session, ARM_NAMES


@unittest.skipIf(Window is None, 'Source the ROS workspace to test the Qt console')
class ConsoleTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    def setUp(self):
        preferences=tempfile.TemporaryDirectory();self.addCleanup(preferences.cleanup)
        settings_patch=patch('task_editor_settings.DEFAULT_PATH',Path(preferences.name)/'settings.json')
        settings_patch.start();self.addCleanup(settings_patch.stop)
        context=patch('real_task_app.rclpy.ok',return_value=True)
        context.start();self.addCleanup(context.stop)
        self.t = time.monotonic()-.6
        session = Session(lambda: self.t)
        self.q = dict.fromkeys(ARM_NAMES, .2)
        for _ in range(6):
            session.update(self.q); self.t += .1
        session.capture_home()
        self.node = S(session=session, lock=threading.RLock(), status='Test status',run_dir=Path('/tmp/task-console-test'),
                      hand_control=False,planner=S(waist_assist=False),
                      args=S(hardware=True, disconnected=False), worker=None,
                      gateway=dict(phase='ARMED_HOLD',armed='true',task_stop_active='false',
                                   mode_machine='6',mode_pr='0'),
                      gateway_received=time.monotonic(), display_pub=Mock(), status_pub=Mock(),
                      home=Mock(), arm=Mock(), launch_work=Mock(), record_ready=Mock(),
                      plan=Mock(), execute=Mock(), stop=Mock(), recovery=Mock(),set_waist_gains=Mock())
        self.node.load_saved_test=Mock();self.node.saved_test_guard=Mock()
        from builtin_interfaces.msg import Time
        self.node.create_publisher=Mock();self.node.get_clock=Mock()
        self.node.get_clock.return_value.now.return_value.to_msg.return_value=Time()
        self.node.remember_saved_collision_check=Mock()
        from task_calibration import DEFAULTS
        self.node.calibration=dict(DEFAULTS)
        self.node.set_calibration=Mock()
        with patch('task_trajectory_library.catalog',return_value=([],[])):
            self.window = Window(self.node)
            self.window.library_scan_thread.join(2.)
            self.window.poll_library()
        self.window.timer.stop()

    def tearDown(self):
        self.window.deleteLater()
        self.app.processEvents()

    def ready(self, kind='prepare'):
        s = self.node.session
        ticket = s.begin(kind, 'left', .15, .15)
        s.ready(ticket, {'kind':kind,'start':self.q,'segments':['test']})
        self.window.refresh()

    def test_planning_is_disabled_and_explained(self):
        self.node.session.begin('prepare','left',.15,.15)
        self.node.status = 'prepare left: validating trajectory 100/6000'
        self.window.refresh()
        self.assertFalse(self.window.execute_button.isEnabled())
        self.assertIn('100/6000', self.window.execute_reason.text())

    def test_all_demo_slots_refresh_execute_label_and_publish_ready_status(self):
        from task_dring_teach_data import SLOTS
        self.node.demo=S(execution_guard=Mock())
        labels={key:value+'录制回放' for key,value in SLOTS.items()}
        labels.update(ready='回录入准备点',preparation='回桌上方准备位')
        for operation,label in labels.items():
            for full_cycle in (False,True):
                with self.subTest(operation=operation,full_cycle=full_cycle):
                    s=self.node.session;ticket=s.begin('dring_demo','left',.7,.7)
                    s.ready(ticket,dict(kind='dring_demo',arm='left',start=self.q,segments=[{}],
                        demo=dict(operation=operation,transition_count=1,full_cycle=full_cycle)))
                    self.node.status='回放就绪'
                    self.window.last_status_publish=0.
                    self.window.refresh()
                    self.assertIn(label,self.window.execute_button.text())
                    self.assertEqual('完整循环' in self.window.execute_button.text(),full_cycle)
                    status=json.loads(self.node.status_pub.publish.call_args.args[0].data)
                    self.assertEqual(status['phase'],'PLAN_READY')
                    self.assertEqual(status['message'],'回放就绪')

    def test_landing_gear_slots_use_own_guard_and_execute_labels(self):
        from task_landing_gear_data import SLOTS
        self.node.demo=S(execution_guard=Mock())
        self.node.landing_gear=S(execution_guard=Mock())
        for operation,label in dict(SLOTS,preparation='回桌上方准备位').items():
            with self.subTest(operation=operation):
                s=self.node.session;ticket=s.begin('dring_demo','right',.7,.7)
                s.ready(ticket,dict(kind='dring_demo',arm='right',start=self.q,segments=[{}],
                    demo=dict(workflow='landing_gear',operation=operation,transition_count=1,full_cycle=False)))
                self.window.refresh()
                self.assertIn(label,self.window.execute_button.text())
                self.node.landing_gear.execution_guard.assert_called_with(s.plan)
        self.node.demo.execution_guard.assert_not_called()
        self.node.execute.assert_not_called()

    def test_library_selection_never_loads_or_executes_and_resets_confirmations(self):
        entry=dict(directory='/verified/archive',arm='left',direction='right',pair_id='pair',
                   source_capture='/session/plan_0013',model_sha256='model',start=self.q,
                   waist_degrees=0.,trajectory_duration_s=40.,planning_seconds=12.)
        with patch('task_trajectory_library.catalog',return_value=([entry],[])):
            self.window.refresh_library()
            self.window.library_scan_thread.join(2.)
            self.window.poll_library()
        self.window.test_scene.setChecked(True);self.window.test_contact.setChecked(True)
        self.window.library_table.selectRow(0)
        self.assertEqual(self.window.saved_path.text(),'/verified/archive')
        self.assertFalse(self.window.test_scene.isChecked())
        self.assertFalse(self.window.test_contact.isChecked())
        self.assertEqual(self.window.last_stage.currentData(),7)
        self.node.load_saved_test.assert_not_called()
        self.node.execute.assert_not_called();self.node.arm.assert_not_called()

    def test_library_hashing_never_blocks_feedback_or_qt_refresh(self):
        entered=threading.Event();release=threading.Event()
        def slow_catalog(root):
            entered.set();release.wait(2.);return [],[]
        with patch('task_trajectory_library.catalog',side_effect=slow_catalog):
            try:
                started=time.monotonic();self.window.refresh_library()
                self.assertTrue(entered.wait(1.))
                self.window.refresh()
                self.assertLess(time.monotonic()-started,.5)
                self.assertEqual(self.window.library_summary.text(),'Checking archives...')
            finally:
                release.set();self.window.library_scan_thread.join(2.)
        self.window.poll_library()
        self.assertIn('0 verified',self.window.library_summary.text())

    def test_library_controls_locked_during_execution(self):
        self.node.session.phase='EXECUTING';self.window.refresh()
        for control in (self.window.library_table,self.window.library_refresh,self.window.library_preview):
            self.assertFalse(control.isEnabled())

    def test_preview_captures_current_scene_without_loading_execution(self):
        self.node.capture_archive_scene=Mock()
        self.window.saved_path.setText('/archive')
        self.window.trajectory_speed.setValue(4.)
        self.window.preview_library_archive()
        self.node.capture_archive_scene.assert_called_once()
        self.node.load_saved_test.assert_not_called()
        self.node.arm.assert_not_called();self.node.execute.assert_not_called()

    def test_stale_scene_callback_cannot_open_preview(self):
        self.window.saved_path.setText('/archive')
        self.window.preview_results.put((('/archive',1.,.15,.15),'/capture',-1,None))
        with patch.object(self.window,'launch_archive_preview') as launch:
            self.window.poll_archive_preview()
            launch.assert_not_called()

    def test_live_speed_change_preserves_path_but_blocks_execution_until_retimed(self):
        self.ready('task')
        self.node.session.plan['hardware_executable']=False
        original=self.node.session.plan
        self.window.clear.setChecked(True)
        self.window.trajectory_speed.setValue(2.)
        self.assertIs(self.node.session.plan,original)
        self.assertEqual(self.node.session.phase,'RETIME_REQUIRED')
        self.assertFalse(self.window.clear.isChecked())
        self.window.refresh()
        self.assertFalse(self.window.execute_button.isEnabled())
        self.assertTrue(self.window.retime_button.isEnabled())
        self.node.args.dring_contact_test=True
        self.window.dring_contact.setChecked(True);self.window.clear.setChecked(True)
        self.window.refresh()
        self.assertFalse(self.window.execute_button.isEnabled())

    def test_speed_and_scene_controls_locked_during_worker(self):
        self.node.worker=S(is_alive=lambda:True)
        self.window.refresh()
        for control in (self.window.trajectory_speed,self.window.archive_scene,self.window.library_preview):
            self.assertFalse(control.isEnabled())

    def test_retime_live_path_keeps_capture_age_and_uses_new_source_file(self):
        from task_plan_archive import write_json,digest
        from task_planning_config import load_parameters,controller_bounded_parameters,DEFAULT_CONFIG
        from trajectory_msgs.msg import JointTrajectory,JointTrajectoryPoint
        from rosidl_runtime_py.convert import message_to_ordereddict
        trajectory=JointTrajectory(joint_names=['left_wrist_yaw_joint'])
        for seconds,value in ((0,0.),(10,.2)):
            point=JointTrajectoryPoint(positions=[value],velocities=[0.],accelerations=[0.])
            point.time_from_start.sec=seconds;trajectory.points.append(point)
        self.ready('task');session=self.node.session
        session.plan.update(hardware_executable=False,segments=[dict(kind='arm',trajectory=trajectory)])
        parameters,config=load_parameters(DEFAULT_CONFIG)
        _,config=controller_bounded_parameters(parameters,config,self.node.gateway)
        session.plan['planning_config']=config
        self.node.model=Mock();self.node.check=lambda ticket:session.check(ticket)
        self.node.launch_work=lambda function:function()
        with tempfile.TemporaryDirectory() as temporary:
            self.node.run_dir=Path(temporary);source=Path(temporary)/'original';source.mkdir()
            write_json(source/'plan.json',dict(segments=[dict(trajectory=message_to_ordereddict(trajectory))]))
            checksum=digest(source/'plan.json')
            session.plan.update(directory=str(source),live_review=dict(source_plan_sha256=checksum))
            session.snapshot_at=self.t-10.
            age=session.snapshot_at
            report=dict(applied_multiplier=2.,duration_after_s=5.)
            with patch('task_trajectory_speed.scale_arm_segments',return_value=(session.plan['segments'],report)):
                TaskNode.retime_plan(self.node,2.)
            self.assertEqual(session.phase,'PLAN_READY',self.node.status)
            self.assertEqual(session.snapshot_at,age)
            self.assertNotEqual(session.plan['directory'],str(source))
            self.assertEqual(digest(source/'plan.json'),checksum)
            self.assertEqual(session.plan['live_review']['source_plan_sha256'],
                             digest(Path(session.plan['directory'])/'plan.json'))
        self.node.execute.assert_not_called();self.node.arm.assert_not_called()

    def test_archived_scene_launch_carries_real_timing_options(self):
        self.window.saved_path.setText('/archive')
        self.window.archive_scene.setCurrentIndex(1)
        self.window.trajectory_speed.setValue(4.)
        with patch('real_task_app.subprocess.Popen') as launch:
            self.window.preview_library_archive()
            command=launch.call_args.args[0]
            self.assertEqual(command[command.index('--trajectory-speed')+1],'4.0')
            self.assertNotIn('--scene-capture',command)
            self.assertEqual(command[command.index('--timing-policy')+1],'guarded')
            self.assertIn('--motion-parameters',command)

    def test_speed_inputs_accept_custom_values_and_stay_synchronized(self):
        self.window.archive_speed.setValue(73.125)
        self.assertEqual(self.window.trajectory_speed.value(),73.125)
        self.window.trajectory_speed.setValue(.025)
        self.assertEqual(self.window.archive_speed.value(),.025)

    def test_next_run_fingers_save_during_execution_without_touching_active_plan(self):
        self.ready('task')
        plan=self.node.session.plan;generation=self.node.session.generation
        self.node.session.phase='EXECUTING'
        self.window.index_openness.setValue(.65);self.window.middle_openness.setValue(.9)
        with patch('task_dring_hand_settings.save') as save:
            self.window.save_hand_settings()
            save.assert_called_once_with(dict(index_q=.65,middle_q=.9))
        self.assertEqual(self.node.session.phase,'EXECUTING')
        self.assertEqual(self.node.session.generation,generation)
        self.assertIs(self.node.session.plan,plan)
        self.node.execute.assert_not_called();self.node.stop.assert_not_called()

    def test_requested_and_applied_speed_visible_during_execution(self):
        self.ready('task')
        self.node.session.plan['speed_scaling']=dict(requested_multiplier=24.,applied_multiplier=1.0769,limited=True)
        self.node.session.phase='EXECUTING';self.window.refresh()
        self.assertIn('24x',self.window.applied_speed.text())
        self.assertIn('1.077x',self.window.applied_speed.text())

    def test_maximum_speed_is_visible_and_selection_does_not_execute(self):
        self.ready('task')
        self.node.session.plan.update(hardware_executable=False,speed_scaling=dict(
            requested_multiplier=1.,applied_multiplier=1.,maximum_multiplier=3.125,
            minimum_duration_s=20.,limiting_constraint=dict(label='Left 120',joint='left_elbow_joint',constraint='jerk')))
        self.window.refresh()
        self.assertIn('3.125x',self.window.maximum_speed.text())
        self.assertIn('jerk',self.window.maximum_speed.text())
        self.assertIn('20.00 s',self.window.maximum_speed.text())
        self.window.use_maximum_speed()
        self.assertEqual(self.window.trajectory_speed.value(),3.125)
        self.assertEqual(self.node.session.phase,'RETIME_REQUIRED')
        self.node.execute.assert_not_called();self.node.arm.assert_not_called()
        self.node.session.phase='EXECUTING';self.window.refresh()
        self.assertFalse(self.window.use_maximum_button.isEnabled())
        with self.assertRaises(RuntimeError):self.window.use_maximum_speed()

    def test_local_timing_shows_verified_request_and_effective_speed_separately(self):
        self.ready('task')
        self.node.session.plan.update(hardware_executable=False,speed_scaling=dict(
            timing_policy='local_path_v1',requested_multiplier=24.,applied_multiplier=3.4,
            maximum_multiplier=5.31,maximum_effective_multiplier=3.4,
            minimum_duration_s=58.54,limiting_constraint=None))
        self.window.refresh()
        self.assertIn('手臂等效 3.400x',self.window.applied_speed.text())
        self.assertIn('各段最快已验证',self.window.maximum_speed.text())
        self.assertIn('58.54 s',self.window.maximum_speed.text())
        self.window.use_maximum_speed()
        self.assertEqual(self.window.trajectory_speed.value(),5.31)
        self.node.execute.assert_not_called()

    def test_compact_history_scrolls_instead_of_overlapping_controls(self):
        self.window.resize(620,720);self.window.tabs.setCurrentIndex(1)
        self.window.show();self.app.processEvents()
        self.assertGreater(self.window.library_table.y(),self.window.archive_speed.geometry().bottom())
        self.assertGreater(self.window.first_stage.y(),self.window.library_table.geometry().bottom())
        self.assertGreater(self.window.test_load.y(),self.window.test_contact.geometry().bottom())
        self.assertGreater(self.window.tabs.widget(1).verticalScrollBar().maximum(),0)
        self.window.hide()

    def test_new_saved_plan_refreshes_history_without_authorizing_motion(self):
        self.node.latest_archive='/new/live/archive'
        with patch.object(self.window,'refresh_library') as refresh:
            self.window.refresh()
            refresh.assert_called_once()
        self.assertEqual(self.window.saved_path.text(),'/new/live/archive')
        self.node.execute.assert_not_called();self.node.arm.assert_not_called()

    def test_unrestricted_live_preview_cannot_enable_execute_button(self):
        self.ready('task')
        self.node.session.plan['preview_only_timing']=True
        self.node.args.dring_contact_test=True
        self.window.dring_contact.setChecked(True)
        self.window.refresh()
        self.assertFalse(self.window.execute_button.isEnabled())
        self.assertIn('RViz-only',self.window.execute_reason.text())

    def test_fresh_plan_saves_baseline_before_applying_guarded_speed(self):
        from trajectory_msgs.msg import JointTrajectory,JointTrajectoryPoint
        from task_planning_config import load_parameters,controller_bounded_parameters,DEFAULT_CONFIG
        import numpy as np
        parameters,config=load_parameters(DEFAULT_CONFIG)
        _,config=controller_bounded_parameters(parameters,config,self.node.gateway)
        trajectory=JointTrajectory(joint_names=['left_wrist_yaw_joint'])
        for seconds in (0,24):
            point=JointTrajectoryPoint(positions=[.2],velocities=[0.],accelerations=[0.])
            point.time_from_start.sec=seconds;trajectory.points.append(point)
        segments=[dict(kind='arm',side='left',label='Approach',trajectory=trajectory,
                       start=dict(self.q),end=dict(self.q))]
        self.node.model=Mock()
        self.node.prepared={'left':dict(joints=dict(self.q))}
        self.node.planner=Mock(waist_assist=False)
        self.node.planner.ring.return_value=segments
        self.node.planner.scene_report={}
        self.node.launch_work=lambda function:function()
        self.node.check=lambda ticket:self.node.session.check(ticket)
        self.node.progress=Mock()
        self.node.finish_plan=lambda ticket,plan:self.node.session.ready(ticket,plan)
        with tempfile.TemporaryDirectory() as temporary:
            self.node.run_dir=Path(temporary)
            model=Path(temporary)/'robot.urdf';model.write_text('<robot/>')
            self.node.args.robot_urdf=str(model)
            def snapshot(ticket):
                directory=self.node.run_dir/f'plan_{ticket.generation:04d}';directory.mkdir()
                self.node.session.accept_snapshot(ticket,dict(directory=str(directory)))
                return dict(capture_frame=dict(pelvis_from_torso_row_major=np.eye(4).tolist())),directory,dict(self.q)
            self.node.snapshot=snapshot
            saved=[]
            def publish(directory,*args,**kwargs):
                record=json.loads((directory/'plan.json').read_text())
                saved.append(record['segments'][0]['trajectory']['points'][-1]['time_from_start']['sec'])
                return '/history/new_plan'
            with patch('task_live_archive.publish_live_plan',side_effect=publish), \
                    patch('task_joint_route.backend_identity',return_value={'method':'test'}), \
                    patch('task_trajectory_speed.scale_arm_segments',return_value=(segments,dict(applied_multiplier=1.))) as scale:
                TaskNode.plan(self.node,'task','left',.15,.15,hand_posture_confirmed=True,speed_multiplier=24.)
            self.assertEqual(saved,[24])
            self.assertEqual(self.node.latest_archive,'/history/new_plan')
            self.assertEqual(self.node.session.phase,'PLAN_READY')
            self.assertFalse(self.node.session.plan.get('preview_only_timing',False))
            self.assertEqual([call.args[1] for call in scale.call_args_list],[1.,24.])
            self.assertEqual(self.node.session.plan['segments'][0]['trajectory'].points[-1].time_from_start.sec,24)
            self.assertEqual(self.node.planner.ik_backend,'bounded')
            self.assertEqual(self.node.planner.joint_route_backend,'ompl')
        self.node.execute.assert_not_called();self.node.arm.assert_not_called()

    def test_calibration_tab_enabled_idle_and_locked_while_busy(self):
        from PyQt5.QtWidgets import QCheckBox
        toggles=self.window.calibration_tab.findChildren(QCheckBox)
        self.assertEqual(len(toggles),3)
        self.assertTrue(all(c.isChecked() for c in toggles))
        toggles[0].setChecked(False)
        self.assertFalse(self.node.set_calibration.call_args[0][0]['a_enabled'])
        self.node.worker=S(is_alive=lambda:True)
        self.window.refresh()
        self.assertFalse(self.window.calibration_tab.isEnabled())

    def test_calibration_edit_invalidates_plan_and_rejects_execution(self):
        from task_calibration import DEFAULTS
        self.ready();self.node.shutdown_requested=False
        self.node.model=Mock();self.node.marker_pub=Mock()
        with patch('task_calibration.marker_offsets',return_value=None):
            TaskNode.set_calibration(self.node,dict(DEFAULTS,depth_bias_mm=1.))
        self.assertIsNone(self.node.session.plan)
        self.assertEqual(self.node.session.phase,'REPLAN_REQUIRED')
        self.node.session.phase='EXECUTING'
        with self.assertRaises(RuntimeError):TaskNode.set_calibration(self.node,DEFAULTS)
        self.assertEqual(self.node.calibration['depth_bias_mm'],1.)

    def test_two_dring_plan_buttons_select_sweep_independently_of_arm(self):
        self.window.hand_posture.setChecked(True)
        for arm in ('left','right'):
            self.window.side.setCurrentText(arm);self.window.refresh()
            for direction,button in self.window.task_plan_buttons.items():
                with self.subTest(arm=arm,direction=direction):
                    self.node.plan.reset_mock();button.click()
                    self.node.plan.assert_called_once_with('task',arm,.15,.15,
                        hand_posture_confirmed=True,sweep_direction=direction,speed_multiplier=1.)
        self.assertFalse(any(b.text()=='Plan D-ring task' for b in self.window.plan_buttons))

    def test_dring_buttons_reject_both_arms_and_are_disabled_during_execution(self):
        self.window.side.setCurrentText('both');self.window.refresh()
        for button in self.window.task_plan_buttons.values():
            self.assertFalse(button.isEnabled());self.assertIn('one arm',button.toolTip())
        self.window.side.setCurrentText('left')
        self.node.session.phase='EXECUTING';self.window.refresh()
        for button in self.window.task_plan_buttons.values():self.assertFalse(button.isEnabled())

    def test_center_check_button_plans_only_and_rejects_both(self):
        self.window.hand_posture.setChecked(True);self.window.refresh()
        self.window.center_check_button.click()
        self.node.plan.assert_called_once_with('task','left',.15,.15,
            hand_posture_confirmed=True,center_check=True,speed_multiplier=1.)
        self.node.execute.assert_not_called()
        self.window.side.setCurrentText('both');self.window.refresh()
        self.assertFalse(self.window.center_check_button.isEnabled())

    def test_center_check_has_its_own_execution_label_without_contact_confirmation(self):
        from task_dring_sequence import CENTER_SCHEMA
        self.ready('task');s=self.node.session
        s.plan.update(task_schema=CENTER_SCHEMA,sweep_direction=None)
        self.window.refresh()
        self.assertEqual(self.window.execute_button.text(),'Execute center check')
        self.assertFalse(self.window.dring_contact.isEnabled())

    def test_execute_button_identifies_both_planned_arm_and_sweep(self):
        self.node.args.dring_contact_test=True;self.ready('task')
        for direction in ('left','right'):
            self.node.session.plan['sweep_direction']=direction;self.window.refresh()
            self.assertEqual(self.window.execute_button.text(),f'Execute LEFT ARM / {direction.upper()} sweep (1-7)')

    def test_task_arm_selector_shares_state_and_invalidates_old_plan(self):
        self.ready('task')
        self.window.task_arm.setCurrentIndex(1)
        self.assertEqual(self.window.side.currentText(),'right')
        self.assertIsNone(self.node.session.plan)
        self.window.side.setCurrentText('left')
        self.assertEqual(self.window.task_arm.currentIndex(),0)
        self.node.session.phase='EXECUTING';self.window.refresh()
        self.assertFalse(self.window.task_arm.isEnabled())

    def test_minimum_window_scrolls_planning_instead_of_overlapping_buttons(self):
        self.window.resize(620,720);self.window.show();self.window.refresh();self.app.processEvents()
        page=self.window.tabs.widget(0)
        self.assertIsInstance(page,QtWidgets.QScrollArea)
        self.assertGreater(page.verticalScrollBar().maximum(),0)
        buttons=[b for b in self.window.plan_buttons if b is not self.window.test_load]
        for i,button in enumerate(buttons):
            bounds=button.geometry()
            bounds.moveTopLeft(button.mapTo(page.widget(),button.rect().topLeft()))
            self.assertGreaterEqual(button.height(),button.minimumSizeHint().height())
            for other in buttons[i+1:]:
                other_bounds=other.geometry()
                other_bounds.moveTopLeft(other.mapTo(page.widget(),other.rect().topLeft()))
                self.assertFalse(bounds.intersects(other_bounds))

    def test_record_ready_keeps_waist_for_the_return_path(self):
        self.node.planner.waist_assist=True;self.node.prepared={'left':None,'right':None}
        q=dict(self.q,waist_yaw_joint=.12)
        with tempfile.TemporaryDirectory() as directory,patch.object(self.node.session,'measured',return_value=q):
            self.node.run_dir=Path(directory);TaskNode.record_ready(self.node,'left')
            ready=json.loads((self.node.run_dir/'ready_poses.json').read_text())
            self.assertEqual(ready['left']['joints']['waist_yaw_joint'],.12)

    def test_stopped_controller_allows_plan_buttons_not_execute_or_saved_authorization(self):
        self.node.session.stop()
        self.node.gateway.update(phase='LATCHED_SILENT',armed='false',task_stop_active='true')
        self.window.refresh()
        for button in self.window.plan_buttons:
            self.assertEqual(button.isEnabled(),button is not self.window.test_load)
        self.ready('task');self.node.args.dring_contact_test=True
        self.window.clear.setChecked(True);self.window.dring_contact.setChecked(True)
        self.window.refresh()
        self.assertFalse(self.window.execute_button.isEnabled())
        self.assertIn('stopped',self.window.execute_reason.text())
        with self.assertRaisesRegex(RuntimeError,'stopped'):
            TaskNode.execute(self.node,scene_confirmed=True,contact_confirmed=True)
        self.node.launch_work.assert_not_called()

    def test_missing_startup_disables_plan_with_reason(self):
        self.node.session.home=None;self.window.refresh()
        for button in self.window.plan_buttons:
            self.assertFalse(button.isEnabled())
            self.assertIn('capture',button.toolTip())

    def test_startup_checkbox_is_required_after_new_start_in_unarmed_latch(self):
        self.node.session.home=None;self.node.session.stop()
        self.node.gateway.update(phase='LATCHED_SILENT',armed='false',task_stop_active='true')
        self.window.refresh()
        self.assertTrue(self.window.r2a.isEnabled())
        self.assertFalse(self.window.r2a.isChecked())
        with patch.object(QtWidgets.QMessageBox,'warning'):
            self.window.capture_button.click()
        self.node.home.assert_not_called()
        self.window.r2a.setChecked(True);self.window.capture_button.click()
        self.node.home.assert_called_once()
        self.node.arm.assert_not_called();self.node.recovery.assert_not_called()

    def test_unarmed_latched_capture_only_records_current_joints(self):
        self.node.session.home=None;self.node.session.stop()
        self.node.gateway.update(phase='LATCHED_SILENT',armed='false',task_stop_active='true')
        with tempfile.TemporaryDirectory() as directory:
            self.node.run_dir=Path(directory);TaskNode.home(self.node)
            record=json.loads((self.node.run_dir/'session_home.json').read_text())
            self.assertTrue(record['operator_confirmed_r2_a'])
            self.assertEqual(record['joints'],self.q)
        self.assertTrue(self.node.session.stopped)
        self.assertIsNone(self.node.session.planning_issue())
        self.node.arm.assert_not_called();self.node.recovery.assert_not_called()
        self.node.execute.assert_not_called();self.node.launch_work.assert_not_called()

    def test_qt_rendering_does_not_hold_joint_feedback_lock(self):
        acquired=threading.Event()
        original=self.window.phase_value.setText
        def render(text):
            def feedback():
                with self.node.lock:
                    self.node.session.update(self.q)
                    acquired.set()
            worker=threading.Thread(target=feedback)
            worker.start()
            try:self.assertTrue(acquired.wait(.5),'Qt rendering blocked joint callback')
            finally:worker.join(.5)
            original(text)
        with patch.object(self.window.phase_value,'setText',side_effect=render):
            self.window.refresh()

    def test_batch_validation_zero_uses_visible_busy_indicator(self):
        self.node.session.begin('prepare','left',.15,.15)
        self.node.status='prepare left: validating trajectory 0/4910'
        self.window.refresh()
        self.assertFalse(self.window.progress.isHidden())
        self.assertEqual(self.window.progress.maximum(),0)
        self.assertIn('4910',self.window.progress.toolTip())

    def test_completed_validation_survives_ready_status_and_stale_generation_does_not(self):
        s=self.node.session;t=s.begin('prepare','left',.15,.15)
        TaskNode.progress(self.node,t,'prepare left: validating trajectory 4910/4910 (100%)')
        s.ready(t,dict(kind='prepare',start=self.q,segments=['test']))
        self.node.status='PLAN READY: prepare'
        self.window.refresh()
        self.assertFalse(self.window.progress.isHidden())
        self.assertEqual(self.window.progress.value(),4910)
        self.assertIn('complete',self.window.progress.format())
        s.invalidate();self.window.refresh()
        self.assertTrue(self.window.progress.isHidden())

    def test_real_intermediate_counts_advance_the_visible_bar(self):
        ticket=self.node.session.begin('prepare','left',.15,.15)
        for completed in (100,500,1000):
            TaskNode.progress(self.node,ticket,f'prepare left: validating trajectory {completed}/4000')
            self.window.refresh()
            self.assertFalse(self.window.progress.isHidden())
            self.assertEqual(self.window.progress.maximum(),4000)
            self.assertEqual(self.window.progress.value(),completed)

    def test_pending_feedback_keeps_preview_and_disables_execute(self):
        s=self.node.session;ticket=s.begin('task','left',.15,.15)
        s.accept_snapshot(ticket,{'sequence':1})
        plan=dict(kind='task',hardware_executable=False,start=dict(self.q),segments=[1])
        s.ready(ticket,plan);self.t+=.2;self.window.refresh()
        self.assertIs(s.plan,plan);self.assertEqual(s.phase,'PLAN_READY')
        self.assertIn('Plan retained',self.window.status.text())
        self.assertFalse(self.window.execute_button.isEnabled())

    def test_live_dring_requires_contact_confirmation_even_with_path_clear(self):
        self.node.args.dring_contact_test=True
        self.ready('task');self.window.clear.setChecked(True);self.window.refresh()
        self.assertFalse(self.window.execute_button.isEnabled())
        self.assertIn('Confirm D-ring contact',self.window.execute_reason.text())
        self.window.dring_contact.setChecked(True);self.window.refresh()
        self.assertTrue(self.window.execute_button.isEnabled())
        self.assertIn('1-7',self.window.execute_button.text())

    def test_plan_completion_retains_result_through_long_loss_and_recovery(self):
        s=self.node.session;ticket=s.begin('task','left',.15,.15)
        s.accept_snapshot(ticket,{'sequence':1})
        plan=dict(kind='task',hardware_executable=False,start=dict(self.q),segments=[1])
        self.node.check=s.check;self.node.args.dring_contact_test=True
        self.t+=5.
        with patch('real_task_app.time.sleep') as sleep:
            TaskNode.finish_plan(self.node,ticket,plan)
        sleep.assert_not_called()
        self.window.refresh()
        self.assertIs(s.plan,plan);self.assertEqual(s.phase,'PLAN_READY')
        self.assertIn('Plan retained',self.window.status.text())
        self.assertFalse(self.window.execute_button.isEnabled())
        self.window.clear.setChecked(True);self.window.dring_contact.setChecked(True)
        self.window.refresh();self.assertFalse(self.window.execute_button.isEnabled())
        s.update(self.q);self.window.refresh()
        self.assertFalse(self.window.execute_button.isEnabled())
        for _ in range(6):self.t+=.1;s.update(self.q)
        self.window.refresh()
        self.assertIs(s.plan,plan);self.assertTrue(self.window.execute_button.isEnabled())
        self.node.execute.assert_not_called();self.node.display_pub.publish.assert_not_called()

    def test_plan_completion_cancellation_still_rejects_result(self):
        s=self.node.session;ticket=s.begin('task','left',.15,.15)
        s.accept_snapshot(ticket,{'sequence':1});self.node.check=s.check
        self.t+=5.;s.stop()
        with self.assertRaisesRegex(RuntimeError,'cancelled'):
            TaskNode.finish_plan(self.node,ticket,dict(segments=[1]))
        self.assertIsNone(s.plan)

    def test_armed_missing_home_can_capture_without_arm_or_motion(self):
        self.node.session.home=None
        self.node.gateway.update(task_stop_active='false',mode_machine='6',mode_pr='0')
        with tempfile.TemporaryDirectory() as directory:
            self.node.run_dir=Path(directory)
            with patch('real_task_app.supported_gateway') as guard:
                TaskNode.home(self.node)
                guard.assert_called_once()
            record=json.loads((self.node.run_dir/'session_home.json').read_text())
            self.assertFalse(record['operator_confirmed_r2_a'])
            self.assertTrue(record['operator_confirmed_hold_reference'])
            self.assertEqual(record['joints'],self.q)
            TaskNode.home(self.node)
            self.assertEqual(record,json.loads((self.node.run_dir/'session_home.json').read_text()))
        self.node.arm.assert_not_called();self.node.execute.assert_not_called()

    def test_capture_rejects_stopped_or_running_controller(self):
        self.node.session.home=None
        self.node.gateway.update(mode_machine='6',mode_pr='0',task_stop_active='false')
        for phase in ('RUNNING','TASK_ADMITTANCE'):
            self.node.gateway['phase']=phase
            with self.assertRaisesRegex(RuntimeError,'ARMED_HOLD'):TaskNode.home(self.node)
        self.assertIsNone(self.node.session.home)

    def test_capture_button_recovers_armed_missing_home_without_rearming(self):
        self.node.session.home=None
        self.window.refresh()
        self.assertTrue(self.window.capture_button.isEnabled())
        self.assertEqual(self.window.capture_button.text(),'Capture return point')
        self.assertFalse(self.window.arm_button.isEnabled())

    def test_ros_shutdown_quits_qt_without_publishing(self):
        with patch('real_task_app.rclpy.ok',return_value=False),patch('real_task_app.QtWidgets.QApplication.quit') as quit:
            self.window.refresh();quit.assert_called_once()
        self.node.status_pub.publish.assert_not_called()

    def test_window_close_runs_the_full_shutdown_outside_the_application_service(self):
        self.node.args.managed_lifecycle=True
        process=Mock();process.poll.return_value=None
        with patch('real_task_app.subprocess.Popen',return_value=process) as launch:
            event=Mock();self.window.closeEvent(event)
            event.ignore.assert_called_once()
            self.assertTrue(self.node.shutdown_requested)
            self.assertEqual(self.node.session.phase,'SHUTTING_DOWN')
            command=launch.call_args.args[0]
            self.assertEqual(command[:2],['systemd-run','--user'])
            self.assertNotIn('--wait',command)
            self.assertIn(str(Path(__file__).resolve().parents[1]/'scripts/stop_task_panel.sh'),command)
            self.assertEqual(command[-2],'--result-file')
            self.assertFalse(any('BindsTo=' in item or 'PartOf=' in item for item in command))
            self.window.closeEvent(Mock());self.assertEqual(launch.call_count,1)
            self.window.refresh()
        self.assertFalse(self.window.arm_button.isEnabled())
        self.assertTrue(all(not button.isEnabled() for button in self.window.plan_buttons))
        self.node.arm.assert_not_called();self.node.recovery.assert_not_called()

    def test_failed_window_shutdown_reenables_panel_without_restoring_plan(self):
        self.node.shutdown_requested=True
        self.node.shutdown_process=Mock()
        self.node.shutdown_process.poll.return_value=1
        self.node.shutdown_process.communicate.return_value=('release failed',None)
        self.window.refresh()
        self.assertFalse(self.node.shutdown_requested)
        self.assertIsNone(self.node.shutdown_process)
        self.assertEqual(self.node.session.phase,'REPLAN_REQUIRED')
        self.assertIn('release failed',self.node.status)

    def test_shutdown_service_start_is_not_mistaken_for_shutdown_completion(self):
        self.node.shutdown_requested=True
        self.node.shutdown_process=Mock(returncode=0)
        self.node.shutdown_process.poll.return_value=0
        self.node.shutdown_process.communicate.return_value=('Running as unit',None)
        self.window.refresh()
        self.assertTrue(self.node.shutdown_requested)
        self.assertIsNone(self.node.shutdown_process)
        self.assertFalse(self.window.arm_button.isEnabled())

    def test_shutdown_failure_result_restores_ui_with_reason(self):
        with tempfile.TemporaryDirectory() as directory:
            self.node.shutdown_requested=True
            self.node.shutdown_result=Path(directory)/'result.json'
            self.node.shutdown_result.write_text(json.dumps(dict(success=False,message='release timed out')))
            self.window.refresh()
        self.assertFalse(self.node.shutdown_requested)
        self.assertIsNone(self.node.shutdown_result)
        self.assertIn('release timed out',self.node.status)

    def test_ready_requires_fresh_confirmation(self):
        self.window.clear.setChecked(True)
        self.ready()
        self.assertFalse(self.window.clear.isChecked())
        self.assertFalse(self.window.execute_button.isEnabled())
        self.window.clear.setChecked(True); self.window.refresh()
        self.assertTrue(self.window.execute_button.isEnabled())
        self.node.execute.assert_not_called()

    def test_preflight_keeps_confirmations_and_freezes_motion_settings(self):
        self.node.args.dring_contact_test=True
        self.ready('task')
        self.window.clear.setChecked(True);self.window.dring_contact.setChecked(True)
        self.node.session.phase='EXECUTION_PREFLIGHT';self.window.refresh()
        self.assertTrue(all(not button.isEnabled() for button in self.window.plan_buttons))
        self.assertTrue(self.window.clear.isChecked());self.assertTrue(self.window.dring_contact.isChecked())
        self.assertFalse(self.window.velocity.isEnabled());self.assertFalse(self.window.execute_button.isEnabled())
        self.node.session.phase='PLAN_READY';self.window.refresh()
        self.assertTrue(self.window.execute_button.isEnabled())

    def test_expired_plan_clears_preview_and_explains(self):
        self.ready()
        self.t += 61.
        self.window.refresh()
        self.assertEqual(self.node.session.phase, 'REPLAN_REQUIRED')
        self.assertIsNone(self.node.session.plan)
        self.assertIn('expired', self.node.status)
        self.assertFalse(self.window.execute_button.isEnabled())
        self.node.display_pub.publish.assert_called_once()

    def test_task_execution_remains_blocked(self):
        self.ready('task')
        self.window.clear.setChecked(True); self.window.refresh()
        self.assertFalse(self.window.execute_button.isEnabled())
        self.assertIn('not commissioned', self.window.execute_reason.text())

    def test_gateway_stale_is_explained(self):
        self.ready()
        self.node.gateway_received = time.monotonic()-3
        self.window.refresh()
        self.assertFalse(self.window.execute_button.isEnabled())
        self.assertIn('GATEWAY STALE', self.window.execute_reason.text())

    def test_waist_preview_cannot_reach_hardware_execution(self):
        self.ready()
        self.node.session.plan['waist_assist']=True
        self.window.clear.setChecked(True);self.window.refresh()
        self.assertFalse(self.window.execute_button.isEnabled())
        self.assertIn('Waist-assisted',self.window.execute_reason.text())
        with self.assertRaisesRegex(RuntimeError,'preview-only'):
            TaskNode.execute(self.node)
        self.node.launch_work.assert_not_called()

    def test_capable_gateway_allows_waist_prepare_but_not_contact_task(self):
        self.ready()
        self.node.gateway['task_waist_trajectory_supported']='true'
        self.node.session.plan['waist_assist']=True
        self.window.clear.setChecked(True);self.window.refresh()
        self.assertTrue(self.window.execute_button.isEnabled())
        self.node.session.plan['kind']='task'
        self.window.refresh()
        self.assertFalse(self.window.execute_button.isEnabled())
        self.assertIn('not commissioned',self.window.execute_reason.text())

    def test_robot_state_stale_is_explained(self):
        self.ready()
        self.t += .2
        self.window.refresh()
        self.assertFalse(self.window.execute_button.isEnabled())
        self.assertIn('waiting for fresh', self.window.execute_reason.text())

    def test_status_contains_button_reason(self):
        self.ready()
        report=json.loads(self.node.status_pub.publish.call_args.args[0].data)
        self.assertEqual(report['phase'], 'PLAN_READY')
        self.assertIn('Path clear', report['execute_block_reason'])

    def test_readonly_is_explained(self):
        self.ready()
        self.node.args.hardware=False
        self.window.refresh()
        self.assertFalse(self.window.execute_button.isEnabled())
        self.assertIn('Read-only', self.window.execute_reason.text())

    def test_saved_controls_pass_exact_selected_range_and_scaling(self):
        self.window.saved_path.setText('/saved')
        self.window.first_stage.setCurrentIndex(2);self.window.last_stage.setCurrentIndex(4)
        self.window.velocity.setValue(.3);self.window.acceleration.setValue(.5)
        self.window.trajectory_speed.setValue(24.)
        self.window.hand_posture.setChecked(True);self.window.test_scene.setChecked(True)
        self.window.test_contact.setChecked(True)
        self.window.load_test()
        self.node.load_saved_test.assert_called_once_with('/saved',3,5,.3,.5,
            scene_confirmed=True,contact_confirmed=True,hand_confirmed=True,selected_side='left',speed_multiplier=24.,
            use_collision_cache=False)
        self.node.execute.assert_not_called()

    def test_collision_cache_is_explicit_opt_in_and_does_not_execute(self):
        self.assertFalse(self.window.collision_cache.isChecked())
        self.window.collision_cache.setChecked(True)
        self.node.remember_saved_collision_check.assert_called_once()
        self.window.saved_path.setText('/saved')
        self.window.load_test()
        self.assertTrue(self.node.load_saved_test.call_args.kwargs['use_collision_cache'])
        self.node.execute.assert_not_called()

    def test_saved_execute_label_and_confirmation_invalidated_by_range_change(self):
        s=self.node.session;t=s.begin('staged_test','both',.15,.15)
        s.ready(t,dict(kind='staged_test',hardware_executable=True,start=self.q,
            test_range=dict(first=1,last=2),segments=[dict(number=1,label='Approach',duration=35.6)]))
        self.window.refresh();self.window.clear.setChecked(True);self.window.refresh()
        self.assertTrue(self.window.execute_button.isEnabled())
        self.assertIn('1-2 only',self.window.execute_button.text())
        self.window.last_stage.setCurrentIndex(2);self.window.refresh()
        self.assertFalse(self.window.execute_button.isEnabled())
        self.assertFalse(self.window.clear.isChecked())
        self.assertIsNone(s.plan)

    def test_waist_tuning_requires_confirmation_and_safe_gateway_phase(self):
        self.node.planner.waist_assist=True
        self.node.gateway.update(waist_kp='60.0',waist_kd='4.5',waist_ki='20.0',
                                 waist_integral_torque_limit_nm='1.2',
                                 waist_integral_torque_nm='0.0',waist_integral_mode='disabled')
        self.window.refresh();self.assertFalse(self.window.waist_apply.isEnabled())
        self.window.waist_confirm.setChecked(True);self.window.refresh()
        self.assertTrue(self.window.waist_apply.isEnabled())
        self.window.waist_kp.setValue(55.);self.window.apply_waist()
        self.node.launch_work.assert_called_once()
        self.node.gateway['phase']='RUNNING';self.window.waist_confirm.setChecked(True);self.window.refresh()
        self.assertFalse(self.window.waist_apply.isEnabled())

    def test_gateway_parameter_request_is_atomic_and_double_typed(self):
        result=S(result=S(successful=True,reason=''))
        node=S(lock=threading.RLock(),args=S(hardware=True),gateway_received=time.monotonic(),
               gateway={'phase':'UNARMED','armed':'false'},waist_parameters=S(),call=Mock(return_value=result),status='')
        TaskNode.set_waist_gains(node,60.,4.5,20.,1.2)
        request=node.call.call_args.args[1]
        self.assertEqual([p.name for p in request.parameters],
                         ['waist_kp','waist_kd','waist_ki','waist_integral_torque_limit_nm'])
        self.assertEqual([p.value.double_value for p in request.parameters],[60.,4.5,20.,1.2])
        self.assertIn('atomically',node.status)

    def test_manual_latch_reset_button_and_guarded_request(self):
        n=self.node; n.reset_latch=Mock()
        n.gateway.update(phase='LATCHED_SILENT',armed='false',command_publisher_exists='false')
        self.window.refresh()
        self.assertTrue(self.window.reset_latch_button.isEnabled())
        self.window.reset_latch_button.click()
        n.launch_work.assert_called_once_with(n.reset_latch)
        n.shutdown_requested=False
        n.reset_task=object()
        n.gateway.update(task_reset_supported='true',task_stop_epoch='4',task_stop_active='true',task_stop_requested='true')
        n.session.stopped=True
        def acknowledge(*args):
            n.gateway.update(phase='UNARMED',task_stop_active='false',task_stop_requested='false')
            n.gateway_received=time.monotonic()
            return S(success=True,message='reset; remains unarmed')
        n.call=Mock(side_effect=acknowledge)
        home=dict(n.session.home)
        TaskNode.reset_latch(n)
        n.call.assert_called_once_with(n.reset_task,unittest.mock.ANY)
        self.assertEqual(n.call.call_args.args[1].stop_epoch,4)
        self.assertFalse(n.session.stopped)
        n.arm.assert_not_called()
        self.assertEqual(n.session.home,home)
        self.assertIsNone(n.session.plan)
        self.assertIn('未 ARM',n.status)
        n.call.reset_mock()
        n.call.side_effect=None
        n.call.return_value=S(success=False,message='state not safe')
        with self.assertRaisesRegex(RuntimeError,'未解除锁存.*state not safe'):TaskNode.reset_latch(n)
        n.call.reset_mock()
        n.gateway.update(phase='ARMED_HOLD',armed='true',command_publisher_exists='true')
        self.window.refresh()
        self.assertFalse(self.window.reset_latch_button.isEnabled())
        with self.assertRaisesRegex(RuntimeError,'无控制输出'):TaskNode.reset_latch(n)
        n.call.assert_not_called()

    def test_reset_does_not_clear_newer_stop_after_ack(self):
        n=self.node; n.shutdown_requested=False; n.reset_task=object()
        n.gateway.update(phase='UNARMED',armed='false',command_publisher_exists='false',
            task_stop_active='true',task_stop_requested='false',task_stop_epoch='4',task_reset_supported='true')
        self.window.refresh()
        self.assertTrue(self.window.reset_latch_button.isEnabled())
        n.session.stopped=True
        def newer_stop(*args):
            n.gateway['task_stop_epoch']='5';n.gateway_received=time.monotonic()
            return S(success=True,message='reset')
        n.call=Mock(side_effect=newer_stop)
        with self.assertRaisesRegex(RuntimeError,'新的 STOP'):TaskNode.reset_latch(n)
        self.assertTrue(n.session.stopped)
        n.arm.assert_not_called()
        n.call.reset_mock()
        n.gateway.update(phase='LATCHED_SILENT',armed='false',command_publisher_exists='false')
        n.gateway_received=time.monotonic()-2
        self.window.refresh()
        self.assertFalse(self.window.reset_latch_button.isEnabled())
        with self.assertRaisesRegex(RuntimeError,'状态过期'):TaskNode.reset_latch(n)
        n.call.assert_not_called()


class PreviewTopicTest(unittest.TestCase):
    def test_rviz_uses_tf_transformer(self):
        import yaml
        root=Path(__file__).resolve().parents[1]
        config=yaml.safe_load((root/'assets/camera_reference/real_task.rviz').read_text())
        self.assertEqual(config['Visualization Manager']['Transformation']['Current']['Class'],
                         'rviz_default_plugins/TF')

    def test_rviz_uses_console_only_topic(self):
        import yaml
        root=Path(__file__).resolve().parents[1]
        config=yaml.safe_load((root/'assets/camera_reference/real_task.rviz').read_text())
        topics=[item['Trajectory Topic'] for item in config['Visualization Manager']['Displays']
                if 'Trajectory Topic' in item]
        self.assertEqual(topics, ['/g1/task/validated_path'])

    @unittest.skipIf(Window is None, 'Source ROS workspace')
    def test_manual_hand_posture_required_before_any_capture(self):
        with self.assertRaisesRegex(RuntimeError,'Confirm actual fingers'):
            TaskNode.plan(S(),'prepare','left',.15,.15)

    @unittest.skipIf(Window is None, 'Source ROS workspace')
    def test_both_rejected_before_capture_for_ring(self):
        with self.assertRaisesRegex(RuntimeError,'Select one hand'):
            TaskNode.plan(S(),'task','both',.15,.15,hand_posture_confirmed=True)


@unittest.skipIf(Window is None, 'Source ROS workspace')
class HandReadStatusTests(unittest.TestCase):
    def node(self,stopped=True):
        return S(hand_control=True,args=S(disconnected=False),lock=threading.RLock(),status='',
                 session=S(stopped=stopped),gateway={'phase':'LATCHED_SILENT' if stopped else 'ARMED_HOLD'},
                 hand=S(read=Mock(return_value={'temperature':[32.]*6})),check=Mock(),cache_hand_state=Mock())

    def test_success_replaces_pending_status_even_when_stopped(self):
        for stopped in (True,False):
            node=self.node(stopped)
            result=TaskNode.read_hands(node)
            self.assertEqual(set(result),{'left','right'})
            self.assertIn('left online',node.status)
            self.assertIn('right online',node.status)
            self.assertNotIn('Reading',node.status)
            self.assertEqual(node.session.stopped,stopped)
            if stopped:self.assertIn('LATCHED_SILENT',node.status)

    def test_does_not_overwrite_a_new_stop_message(self):
        node=self.node()
        def cache(side,state):
            if side=='right':node.status='STOP requested during hand read'
        node.cache_hand_state.side_effect=cache
        TaskNode.read_hands(node)
        self.assertEqual(node.status,'STOP requested during hand read')

    def test_timeout_is_not_reported_as_success(self):
        node=self.node();node.hand.read.side_effect=[{'temperature':[32.]*6},TimeoutError('DDS')]
        with self.assertRaisesRegex(TimeoutError,'Right hand state unavailable'):
            TaskNode.read_hands(node)
        self.assertNotIn('right online',node.status)


if __name__ == '__main__': unittest.main()
