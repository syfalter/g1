"""Offline trim/undo/preview tests; never create a ROS node or command client."""
import copy
import json
import os
from pathlib import Path
import unittest
from unittest.mock import Mock,patch

import numpy as np

import task_dring_teach_data as data
from task_dring_trim import cut,sample
import test_task_dring_demo as fixtures

fixture=fixtures.fixture


class TrimTests(unittest.TestCase):
    def test_rounded_ui_endpoint_maps_to_exact_last_timestamp(self):
        from types import SimpleNamespace as S
        from task_dring_demo_panel import DringDemoPanel
        control=S(config=dict(traces=dict(left=dict(raw=dict(time_s=[0.,26.426507115])))),edit_trace=Mock())
        panel=S(control=control,slot=lambda:'left',
                cut_start=S(value=lambda:1.,maximum=lambda:26.427),
                cut_end=S(value=lambda:26.427,maximum=lambda:26.427),bridge=S(value=lambda:.6))
        DringDemoPanel.trim(panel)
        control.edit_trace.assert_called_once_with('left',1.,26.426507115,.6)

    def test_bridge_and_retained_samples(self):
        model,_,config=fixture();trace=config['traces']['left'];old=copy.deepcopy(trace)
        edited=cut(trace,1.,3.,.6);q,t=data.arrays(edited,model,'left')
        self.assertEqual(trace,old)
        self.assertAlmostEqual(t[-1],2.6)
        np.testing.assert_array_equal(q[0],trace['raw']['positions'][0])
        np.testing.assert_array_equal(q[-1],trace['raw']['positions'][-1])
        np.testing.assert_allclose(sample(edited,[.2,.6]),sample(trace,[.2,.6]),atol=1e-14)
        np.testing.assert_allclose(sample(edited,[2.4]),sample(trace,[3.8]),atol=1e-14)
        from task_dring_trim import boundary_state
        from scipy.interpolate import make_interp_spline
        a,va,aa=boundary_state(trace,1.);b,vb,ab=boundary_state(trace,3.)
        spline=make_interp_spline([0.,.6],[a,b],k=5,
            bc_type=([(1,va),(2,aa)],[(1,vb),(2,ab)]))
        self.assertGreater(np.max(np.abs(va)),1e-4)
        mask=(t>=1.)&(t<=1.6)
        np.testing.assert_allclose(q[mask],spline(t[mask]-1.),atol=1e-14)
        np.testing.assert_allclose(spline(0.,1),va,atol=1e-12)
        np.testing.assert_allclose(spline(.6,2),ab,atol=1e-12)
        tr,report=data.fit_trace(edited,model,'left')
        self.assertLessEqual(report['fit_error_rad'],.001)
        self.assertEqual(report['sample_policy'],'offline_edited')

    def test_boundaries_repeated_edits_and_invalid_ranges(self):
        model,_,config=fixture();trace=config['traces']['left']
        for start,end in ((0.,1.),(3.,4.),(0.,4.),(.001,3.999)):
            edited=cut(trace,start,end);data.arrays(edited,model,'left')
            np.testing.assert_array_equal(edited['raw']['positions'][0],trace['raw']['positions'][0])
            np.testing.assert_array_equal(edited['raw']['positions'][-1],trace['raw']['positions'][-1])
        edited=cut(cut(trace,1.,2.),.5,1.5);data.arrays(edited,model,'left')
        for a,b,d in ((-1.,2.,.6),(2.,1.,.6),(0.,5.,.6),(0.,1.,0.),(0.,float('nan'),.6)):
            with self.assertRaises(ValueError):cut(trace,a,b,d)


class TrimPersistenceTests(unittest.TestCase):
    def setUp(self):
        fixtures.DemoPersistenceTests.setUp(self)
        self.node.demo_preview_pub=Mock()

    def test_all_ten_independent_versions_survive_restart(self):
        from task_dring_demo_control import DringDemoControl
        c=self.control;n=self.node;n.launch_work=lambda work:work()
        config=copy.deepcopy(self.config)
        for slot in data.SLOTS:config['traces'][slot]=copy.deepcopy(config['traces']['left'])
        c.persist(config);original=copy.deepcopy(c.config)
        with patch.object(c,'gateway_guard',side_effect=AssertionError('No hardware checks')):
            for slot in data.SLOTS:
                c.edit_trace(slot,1.,2.,.6)
                self.assertIn('trim_edit',c.config['traces'][slot])
            once=copy.deepcopy(c.config)
            c.edit_trace('left',.5,1.,.6)
            c=DringDemoControl(n,c.store)
            saved_bundle=c.store.read_bytes()
            c.edit_version('left','select',index=1)
            self.assertEqual(c.config['traces'],once['traces'])
            for slot in data.SLOTS:c.edit_version(slot,'select',index=0)
        self.assertEqual(c.config['traces'],original['traces'])
        self.assertEqual(c.store.read_bytes(),saved_bundle)
        self.assertEqual(DringDemoControl(n,c.store).config,c.config)
        self.assertIsNotNone(c.config['trace_versions']['left']['edits'][1])

    def test_invalid_cut_leaves_store_untouched(self):
        c=self.control;self.node.launch_work=lambda work:work();c.persist(self.config)
        before=c.store.read_bytes()
        with self.assertRaises(ValueError):c.edit_trace('left',3.,1.)
        self.assertEqual(c.store.read_bytes(),before)

    def test_preview_needs_no_live_state_and_does_not_authorize_execution(self):
        c=self.control;n=self.node;c.persist(self.config)
        n.session.current={};n.session.measured=Mock(side_effect=AssertionError('No live state'))
        n.session.invalidate.reset_mock();n.display_pub.reset_mock()
        c.preview_pose(self.config['traces']['left']['raw']['positions'][100])
        message=n.demo_preview_pub.publish.call_args.args[0]
        self.assertFalse(message.hide)
        self.assertEqual(len(message.state.joint_state.position),8)
        n.display_pub.reset_mock()
        for _ in range(60):c.preview_pose([0.]*8)
        n.display_pub.publish.assert_not_called()
        n.session.invalidate.assert_not_called();n.session.measured.assert_not_called()
        c.hide_preview();self.assertTrue(n.demo_preview_pub.publish.call_args.args[0].hide)
        n.session.phase='EXECUTING';n.demo_preview_pub.reset_mock()
        c.preview_pose([0.]*8);n.demo_preview_pub.publish.assert_not_called()

    def test_rviz_has_dedicated_persistent_state_display(self):
        import yaml
        config=yaml.safe_load((Path(__file__).resolve().parents[1]/'assets/camera_reference/real_task.rviz').read_text())
        displays=config['Visualization Manager']['Displays']
        state=next(d for d in displays if d['Class']=='moveit_rviz_plugin/RobotState')
        self.assertEqual(state['Robot State Topic'],'/g1/task/demo_preview_state')
        self.assertTrue(state['Enabled']);self.assertTrue(state['Visual Enabled'])
        self.assertFalse(state['Collision Enabled'])

    def test_panel_selection_scrub_cut_and_undo(self):
        from types import SimpleNamespace as S
        from PyQt5 import QtWidgets
        from task_dring_demo_panel import DringDemoPanel
        app=QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
        c=self.control;n=self.node;n.launch_work=lambda work:work();n.demo=c;n.session.plan=None
        config=copy.deepcopy(self.config)
        for slot in data.SLOTS:config['traces'][slot]=copy.deepcopy(config['traces']['left'])
        c.persist(config)
        window=S(node=n,invoke=lambda action:action(),clear=QtWidgets.QCheckBox())
        panel=DringDemoPanel(window);panel.resize(620,1000);panel.show();app.processEvents();panel.refresh(False)
        try:
            self.assertTrue(panel.preview_timer.isActive())
            for index,slot in enumerate(data.SLOTS):
                panel.table.selectRow(index);app.processEvents()
                self.assertIs(panel.preview_trace,c.config['traces'][slot])
                panel.timeline.setValue(1250)
                self.assertFalse(panel.preview_timer.isActive())
                self.assertAlmostEqual(panel.preview_time,1.25)
                self.assertIn('1.250',panel.timestamp.text())
                message=n.demo_preview_pub.publish.call_args.args[0]
                np.testing.assert_allclose(message.state.joint_state.position,
                                           sample(c.config['traces'][slot],1.25))
            panel.cut_start.setValue(1.);panel.cut_end.setValue(2.);panel.trim();panel.refresh(False)
            self.assertTrue(panel.preview_timer.isActive());self.assertTrue(panel.undo_trim.isEnabled())
            panel.undo_trim.click();panel.refresh(False)
            self.assertEqual(c.config['traces'],config['traces']);self.assertFalse(panel.undo_trim.isEnabled())
            self.assertIsNotNone(c.config['trace_versions'][panel.slot()]['edits'][0])
            panel.stop_preview();self.assertTrue(c.preview_visible)
            panel.hide();app.processEvents();self.assertFalse(c.preview_visible)
            self.assertTrue(n.demo_preview_pub.publish.call_args.args[0].hide)
            panel.show();app.processEvents();self.assertTrue(c.preview_visible)
            screenshots=os.environ.get('DRING_TRIM_SCREENSHOT_DIR')
            if screenshots:
                directory=Path(screenshots);directory.mkdir(parents=True,exist_ok=True)
                for width in (620,1100):
                    panel.resize(width,1000);app.processEvents()
                    panel.grab().save(str(directory/f'trim_panel_{width}.png'))
            n.session.phase='EXECUTING';panel.refresh(True)
            self.assertFalse(panel.preview_timer.isActive());self.assertFalse(panel.play_preview.isEnabled())
        finally:panel.close();app.processEvents()


if __name__=='__main__':unittest.main()
