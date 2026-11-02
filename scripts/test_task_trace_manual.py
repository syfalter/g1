"""Manual point edits, with fake Cartesian joints and an optional real model."""
import copy
import os
import unittest
from unittest.mock import Mock,patch

import numpy as np
from scipy.spatial.transform import Rotation

from task_dring_teach_data import arrays,moving
from task_dring_trim import sample
from task_trace_manual import manual_trace,point_state,solve_point
from test_task_dring_demo import fixture


class ManualTests(unittest.TestCase):
    def setUp(self):
        self.model,self.context,self.config=fixture()
        self.trace=self.config['traces']['left']
        def transform(link,state):
            pose=np.eye(4);pose[:3,3]=[state[n] for n in moving('left')[:3]]
            pose[:3,:3]=Rotation.from_rotvec([state[moving('left')[3]],0.,0.]).as_matrix()
            return pose
        self.model.transform=transform

    def point(self,stamp,shift=.01):
        q=sample(self.trace,stamp);q[1]+=shift
        return dict(time=stamp,joints=q.tolist(),position=q[:3].tolist())

    def edit(self,points,start=1.,end=3.):
        return manual_trace(self.trace,points,start,end,self.model,'left',self.context,self.config,'left')

    def test_single_move_and_insert_exactly_preserve_outside(self):
        before=copy.deepcopy(self.trace)
        for points in ([self.point(2.)],[self.point(1.2345),self.point(2.4567,-.005)]):
            edited=self.edit(points);arrays(edited,self.model,'left')
            for point in points:np.testing.assert_allclose(sample(edited,point['time']),point['joints'],atol=1e-12)
            for stamp in (0.,.5,1.,3.,3.5,4.):
                np.testing.assert_array_equal(sample(edited,stamp),sample(self.trace,stamp))
            self.assertEqual(edited['optimization']['kind'],'manual_points')
        self.assertEqual(self.trace,before)

    def test_invalid_pose_orientation_limit_and_boundary(self):
        point=self.point(2.);point['position'][2]=10.
        with self.assertRaisesRegex(ValueError,'姿态约束'):self.edit([point])
        point=self.point(2.);point['joints'][3]=.1
        with self.assertRaisesRegex(ValueError,'姿态约束'):self.edit([point])
        with self.assertRaisesRegex(ValueError,'关节限位'):self.edit([self.point(2.,3.)])
        with self.assertRaisesRegex(ValueError,'边界'):self.edit([self.point(1.)])

    def test_endpoint_and_recorded_hand_preserved(self):
        _,_,config=fixture(True);self.trace=config['traces']['left']
        edited=self.edit([self.point(0.)],0.,1.)
        np.testing.assert_allclose(sample(edited,0.),self.point(0.)['joints'])
        self.assertEqual(edited['hand'],self.trace['hand'])

    def test_ik_failure_does_not_modify_source(self):
        before=copy.deepcopy(self.trace)
        with patch('task_ik_search.bounded_pose_ik',return_value=(None,{})):
            with self.assertRaisesRegex(ValueError,'不成功'):
                solve_point(self.model,self.trace,'left',self.context,self.config,'left',2.,[9.,9.,9.])
        self.assertEqual(self.trace,before)

    def test_drag_mouse_up_and_stale_feedback(self):
        from task_trace_picker import TracePicker
        from visualization_msgs.msg import InteractiveMarkerFeedback as Feedback,InteractiveMarkerControl as Control
        with patch('task_trace_picker.InteractiveMarkerServer') as server:
            picker=TracePicker(Mock());owner=object();callback=Mock()
            picker.show(owner,[],Mock());picker.move_handle(owner,[1.,2.,3.],callback)
            marker=server.return_value.insert.call_args.args[0]
            axes=marker.controls[1:]
            self.assertEqual(len(axes),3)
            self.assertTrue(all(axis.interaction_mode==Control.MOVE_AXIS for axis in axes))
            for axis,direction in zip(axes,np.eye(3)):
                q=axis.orientation
                np.testing.assert_allclose(Rotation.from_quat([q.x,q.y,q.z,q.w]).apply([1,0,0]),direction,atol=1e-12)
            feedback=server.return_value.insert.call_args.kwargs['feedback_callback']
            feedback(Feedback(event_type=Feedback.POSE_UPDATE));callback.assert_not_called()
            feedback(Feedback(event_type=Feedback.MOUSE_UP));callback.assert_called_once()
            picker.clear(owner);feedback(Feedback(event_type=Feedback.MOUSE_UP));callback.assert_called_once()

    @unittest.skipUnless(os.environ.get('G1_OFFLINE_TEST_URDF'),'Generated URDF required')
    def test_real_model_small_translation_locked_orientation_and_waist(self):
        from workbench_model import OfflineModel
        from task_dring_demo_control import DringDemoControl
        from types import SimpleNamespace as S
        model=OfflineModel(os.environ['G1_OFFLINE_TEST_URDF'])
        control=DringDemoControl(S(model=model,args=S(robot_urdf=os.environ['G1_OFFLINE_TEST_URDF'])))
        slot=next(iter(control.config['traces']));trace=control.config['traces'][slot]
        side=control.editor_side();stamp=trace['raw']['time_s'][-1]/2
        context=dict(trace.get('context_joints',{}))
        state,pose=point_state(model,trace,side,context,control.config,slot,stamp)
        target=pose[:3,3]+[.001,0.,0.]
        point=solve_point(model,trace,side,context,control.config,slot,stamp,target)
        state.update(zip(moving(side),point['joints']))
        actual=model.transform(side+'_index_tcp',state)
        self.assertLess(np.linalg.norm(actual[:3,3]-target),.0002)
        self.assertLess(Rotation.from_matrix(pose[:3,:3].T@actual[:3,:3]).magnitude(),.002)
        self.assertEqual(point['joints'][-1],sample(trace,stamp)[-1])
        before=copy.deepcopy(trace)
        edited=manual_trace(trace,[point],stamp-.5,stamp+.5,model,side,context,control.config,slot)
        arrays(edited,model,side)
        self.assertEqual(trace,before)


class ManualPanelTests(unittest.TestCase):
    def setUp(self):
        from test_task_dring_demo import DemoPersistenceTests
        from task_trace_editor_panel import TraceEditorPanel
        from types import SimpleNamespace as S
        from PyQt5 import QtWidgets
        DemoPersistenceTests.setUp(self)
        self.node.launch_work=lambda work:work()
        self.control.persist(self.config)
        self.app=QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
        self.panel=TraceEditorPanel(S(invoke=lambda action:action()),self.control,lambda:'left')
        self.panel.refresh();self.panel.show();self.app.processEvents()
        self.addCleanup(self.panel.close)
        def transform(link,state):
            pose=np.eye(4);pose[:3,3]=[state[n] for n in moving('left')[:3]];return pose
        self.node.model.transform=transform

    def test_insert_solve_fail_restore_and_save_one_version(self):
        from task_editor_settings import draft,restore_draft
        from task_trace_versions import bank
        editor=self.panel;manual=editor.manual
        editor.start.setValue(1.);editor.end.setValue(3.)
        manual.insert_midpoint()
        self.assertEqual(manual.active,2.)
        q=sample(editor.trace,2.);q[1]+=.01
        point=dict(time=2.,joints=q.tolist(),position=q[:3].tolist())
        with patch('task_trace_manual_panel.solve_point',return_value=point):
            manual.solve_input();manual.thread.join(timeout=5);self.app.processEvents()
        self.assertEqual(len(manual.points),1)
        self.assertIn('成功',manual.status.text())
        with patch('task_trace_manual_panel.solve_point',side_effect=ValueError('不成功：无法到达')):
            manual.solve_input();manual.thread.join(timeout=5);self.app.processEvents()
        self.assertIn('不成功',manual.status.text())
        self.assertEqual(manual.points[2.],point)
        saved=draft(editor);manual.reset();restore_draft(editor,saved)
        self.assertEqual(manual.points[2.],point)
        manual.save();editor.refresh()
        entry=bank(self.control.config,'left')
        self.assertEqual(entry['selected'],1)
        self.assertEqual(sum(v is not None for v in entry['edits']),1)
        from datetime import datetime
        timestamp=entry['edits'][0]['edit_origin']['created_at']
        self.assertIsNotNone(datetime.fromisoformat(timestamp).tzinfo)
        self.assertEqual(editor.versions.rowCount(),11)
        self.assertEqual(editor.versions.item(1,4).toolTip(),timestamp)
        self.assertFalse(manual.points)
        np.testing.assert_allclose(sample(editor.trace,2.),q,atol=1e-12)

    def test_stale_results_and_full_bank_do_not_modify_drafts(self):
        from task_trace_versions import EDIT_SLOTS,append
        manual=self.panel.manual;self.panel.seek(2000);manual.begin_current()
        old_token=manual.token;manual.reset()
        manual.finish(None,'old failure',old_token)
        self.assertEqual(manual.status.text(),'')
        for _ in range(EDIT_SLOTS):self.control.config=append(self.control.config,'left',self.panel.trace)
        with self.assertRaisesRegex(ValueError,'已满'):manual.begin_current()
        with self.assertRaisesRegex(ValueError,'已满'):
            self.panel.perform(lambda:self.control.edit_version('left','curve',key_times=[0.,1.,2.,4.]))
        self.assertIn('已满',self.panel.operation_status.text())

    def test_failed_worker_keeps_error_visible_and_restores_markers(self):
        self.panel.refresh(busy=True)
        self.control.editor_feedback='不成功：曲线超出关节限位'
        self.panel.refresh(busy=False)
        self.assertTrue(self.panel.marker_timer.isActive())
        self.assertIn('关节限位',self.panel.operation_status.text())

    def test_quick_version_switch_disables_old_edits_until_refresh(self):
        self.control.edit_version('left','cut',start=1.,end=2.,bridge=.6)
        self.panel.refresh()
        self.panel.choose(0)
        self.assertTrue(self.panel.was_busy)
        self.assertFalse(self.panel.manual.isEnabled())
        self.panel.finish_switch()
        self.assertFalse(self.panel.was_busy)
        self.assertTrue(self.panel.manual.isEnabled())
        self.assertEqual(self.panel.key,(self.control.revision,'left'))


if __name__=='__main__':unittest.main()
