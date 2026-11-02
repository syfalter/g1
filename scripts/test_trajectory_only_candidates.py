"""Offline regression gates; no ROS node or hardware transport is constructed."""
import copy
import unittest
from pathlib import Path
from types import SimpleNamespace as S
from unittest.mock import Mock, patch

import numpy as np
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from trajectory_only_candidates import (ROOT, BASE, KEYS, ledger, remaining_budget,
    read, load, decode, encode, checked, context_for, metrics, OfflineModel,
    parameters_from_values, stamp, digest)
from trajectory_push_depth import compensation, weights

STAGE = ROOT / 'data/rail_deployment/trajectory_only_20260929'


class BudgetTests(unittest.TestCase):
    def test_ledger_uses_retimed_clock_and_no_double_budget(self):
        bound = dict(zip(KEYS, [.001, .0006, .002]))
        report = dict(edits=[dict(source_begin_s=2., source_end_s=4., replacement_seconds=.5, bounds=bound),
                             dict(source_begin_s=6., source_end_s=7., replacement_seconds=.5, bounds=bound)])
        intervals, reserve = ledger(report)
        self.assertEqual([(a,b) for a,b,_ in intervals], [(2.,2.5),(4.5,5.)])
        limits = np.array([.003,.001,.008])
        np.testing.assert_allclose(remaining_budget(intervals,reserve,limits,2.1,2.4),
                                   limits-np.array([.001,.0006,.002])-reserve)
        np.testing.assert_allclose(remaining_budget(intervals,reserve,limits,3.,4.),limits-reserve)

    def test_untraceable_polishing_and_overlap_fail_closed(self):
        for report in ({}, dict(edits=[],polishing=dict(accepted=True)), dict(edits=[],fallback='unknown')):
            with self.assertRaises(ValueError):
                ledger(report)
        edit = dict(source_begin_s=1.,source_end_s=3.,replacement_seconds=1.,bounds=dict.fromkeys(KEYS,0.))
        with self.assertRaises(ValueError):
            ledger(dict(edits=[edit,copy.deepcopy(edit)]))

    def test_zero_compensation_is_exact_identity(self):
        tr = JointTrajectory(joint_names=['q'], points=[stamp(JointTrajectoryPoint(
            positions=[q], velocities=[0.], accelerations=[0.]),t) for t,q in ((0.,0.),(1.,.1))])
        result, report = compensation(tr,None,{},[1.,0.,0.],0.,(0.,.2,.8,1.))
        self.assertEqual(encode(tr),encode(result))
        self.assertIsNot(tr,result)
        self.assertTrue(report['exact_identity'])
        with self.assertRaises(ValueError):
            compensation(tr,None,{},[0.,0.,0.],0.,(0.,.2,.8,1.))

    def test_local_weight_does_not_translate_whole_path(self):
        t = np.array([-1.,0.,.25,.5,.75,1.,2.])
        np.testing.assert_allclose(weights(t,0.,.25,.75,1.),[0,0,1,1,1,0,0],atol=1e-12)


@unittest.skipUnless((STAGE/'export_report.json').exists(),'Generate pinned candidates before artifact tests')
class ArtifactTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.model = OfflineModel(ROOT/'data/trajectory_edit_audit/fast_timing_all/panel_robot.urdf')
        cls.before = read(BASE/'bundle.json')
        cls.after = read(STAGE/'data_only/config/selected_bundle.json')

    def test_protected_business_and_hand_steps_unchanged(self):
        a,b = self.before['payload'],self.after['payload']
        for key in ('initialization','mapping'):
            self.assertEqual(a[key],b[key])
        self.assertEqual(a['actions']['dring_pair'],b['actions']['dring_pair'])
        for name in a['actions']:
            old,new = a['actions'][name],b['actions'][name]
            self.assertEqual(old['plan']['planning_config'],new['plan']['planning_config'])
            for x,y in zip(old['program'],new['program']):
                if x['kind'] != 'recorded':
                    self.assertEqual(x,y)
        self.assertEqual((STAGE/'data_only/config/dring_protocol.json').read_bytes(),
                         (STAGE/'jetson_readonly/config/dring_protocol.json').read_bytes())

    def test_gear_complete_reconstruction_and_all_limits(self):
        from export_trajectory_only_candidates import imported,checkpoints_test
        protocol = imported('test_audited_gear_protocol',STAGE/'jetson_readonly/deployment/rail_jetson/gear_protocol.py')
        progress = imported('test_audited_progress',STAGE/'jetson_readonly/deployment/rail_jetson/gear_progress.py')
        library = protocol.GearLibrary(read(STAGE/'data_only/config/gear_protocol.json'),self.after)
        old = protocol.GearLibrary(read(BASE/'gear_protocol.json'),self.before)
        self.assertEqual(len(checkpoints_test(library,progress.GearProgress,old)),24)
        for name,action in self.after['payload']['actions'].items():
            if name == 'dring_pair':
                continue
            params = parameters_from_values(action['plan']['planning_config']['values'])
            for step in action['program']:
                if step['kind'] == 'recorded':
                    checked(decode(step['trajectory']),self.model,params)

    def test_unapproved_depth_never_exported(self):
        report = read(STAGE/'push_depth_report.json')
        for name,item in report['candidates'].items():
            self.assertFalse(item['publishable'])
            if name.startswith('strict_exit_') and not name.endswith('0mm'):
                self.assertFalse(item['motion_limits_passed'])
        self.assertTrue(report['candidates']['transition_review_3mm']['motion_limits_passed'])
        self.assertTrue(report['candidates']['transition_review_3mm']['requires_exit_transition_approval'])
        action = self.after['payload']['actions']['push']
        saved = load(ROOT/action['source'],self.model)
        context = context_for(saved['variants']['flow']['payload'],'motion_0')
        peak = metrics(decode(saved['variants']['flow']['payload']['units']['motion_0']['trajectory']),self.model,context)[0]['peak_x_mm']
        self.assertLess(abs(peak-report['before']['peak_x_mm']),.02)

    def test_data_only_hashes_and_no_runtime(self):
        import hashlib
        report = read(STAGE/'export_report.json')
        self.assertFalse(report['deployed'])
        self.assertEqual(report['runtime_files_in_package'],0)
        for name,expected in report['files'].items():
            path = STAGE/'data_only'/name
            self.assertEqual(path.suffix,'.json')
            self.assertEqual(hashlib.sha256(path.read_bytes()).hexdigest(),expected)
        self.assertEqual(digest(self.after['payload']),self.after['sha256'])

    def test_panel_loads_exact_pinned_durations_without_generation(self):
        from PyQt5 import QtWidgets
        from task_offline_optimizer_panel import OptimizerPanel, data_index
        from task_trace_selection import restore
        from task_optimizer_snapshot import for_case
        app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
        rail_path = ROOT/'data/task_sessions/20260924_153036_985614/rail_autosave/config.json'
        gear_path = ROOT/'data/task_panel/landing_gear_demonstration.json'
        rail = S(config=restore(read(rail_path),rail_path),revision=0)
        gear = S(config=restore(read(gear_path),gear_path),revision=0)
        empty = S(config={'traces':{}},revision=0)
        node = S(demo=empty,landing_gear=gear,rail=rail,model=self.model,
                 display_pub=Mock(),demo_preview_pub=Mock(),status='',launch_work=Mock())
        panel = OptimizerPanel(node,snapshot_directory=STAGE/'panel')
        try:
            with patch('task_offline_optimizer_panel.comparison',side_effect=AssertionError('Unexpected regeneration')):
                for workflow,slot in (('rail','push_cycle'),('rail','pull_cycle'),('landing_gear',None)):
                    panel.case.setCurrentIndex(data_index(panel.case,(workflow,slot)))
                    app.processEvents()
                    self.assertIsNotNone(panel.viewer.result,panel.viewer.summary.text())
                    expected = load(for_case(workflow,slot,STAGE/'panel'),self.model)['variants']['flow']
                    self.assertEqual(panel.viewer.result['trajectory'],expected['trajectory'])
                    self.assertAlmostEqual(panel.viewer.duration(),expected['times'][-1],places=8)
                    self.assertIn('trajectory-only-c2-ledger-v1',panel.viewer.summary.text())
                    self.assertIn('未部署',panel.viewer.summary.text())
                    panel.viewer.toggle()
                    self.assertTrue(panel.viewer.playing)
                    panel.viewer.toggle()
                    panel.viewer.resize(1050,920)
                    panel.viewer.show()
                    app.processEvents()
                    screenshot = STAGE/(slot or 'gear')
                    self.assertTrue(panel.viewer.grab().save(str(screenshot.with_suffix('.png'))))
                panel.snapshot_directory=STAGE/'depth_panels/strict_exit_3mm'
                panel.case.setCurrentIndex(data_index(panel.case,('rail','push_cycle')))
                app.processEvents()
                self.assertIn('导数超限',panel.viewer.summary.text())
                self.assertIn('已拒绝执行',panel.viewer.summary.text())
                self.assertTrue(panel.viewer.grab().save(str(STAGE/'depth_rejected.png')))
                node.launch_work.assert_not_called()
        finally:
            panel.halt()
            panel.viewer.timer.stop()
            panel.close()


if __name__ == '__main__':
    unittest.main()
