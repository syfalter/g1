"""Check approach compression without changing grasp, release or hand waits."""
import unittest
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
from PyQt5 import QtWidgets

from trajectory_only_candidates import (ROOT, BASE, read, load, decode, checked, evaluate,
    OfflineModel, parameters_from_values, time_seconds, ledger, remaining_budget, KEYS)
from task_composition_preview import section, materialize
from task_stop_blending import spatial_bound
from task_trajectory import coefficients
from task_offline_optimizer import Policy


class PullReadyTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
        cls.model = OfflineModel(ROOT / 'data/trajectory_edit_audit/fast_timing_all/panel_robot.urdf')
        cls.directory = ROOT / 'data/rail_deployment/pull_ready_shorter5_20260929'
        cls.report = read(cls.directory / 'report.json')
        cls.old = load(ROOT / cls.report['source_snapshot'], cls.model)['variants']['flow']
        cls.new = load(ROOT / cls.report['candidate_snapshot'], cls.model)['variants']['flow']
        cls.action = read(BASE / 'bundle.json')['payload']['actions']['pull']
        cls.parameters = parameters_from_values(cls.action['plan']['planning_config']['values'])

    def test_continuous_limits_and_fixed_grasp(self):
        checked(self.new['trajectory'], self.model, self.parameters)
        a = decode(self.old['payload']['units']['motion_0']['trajectory'])
        b = decode(self.new['payload']['units']['motion_0']['trajectory'])
        for index in (0, -1):
            for field in ('positions', 'velocities', 'accelerations'):
                np.testing.assert_allclose(getattr(a.points[index], field), getattr(b.points[index], field), atol=1e-10)
        ts = np.linspace(0., self.report['approach']['begin_s'], 1001)
        for order in range(3):
            np.testing.assert_allclose(evaluate(a,ts,order), evaluate(b,ts,order), atol=1e-8, rtol=0.)
        self.assertLess(time_seconds(b.points[-1]), time_seconds(a.points[-1]) - 1.)

    def test_disjoint_continuous_geometry_budgets(self):
        a = decode(self.old['payload']['units']['motion_0']['trajectory'])
        b = decode(self.new['payload']['units']['motion_0']['trajectory'])
        deployed = load(ROOT / self.action['source'], self.model)
        intervals, reserve = ledger(deployed['variants']['flow']['payload']['flowEvidence']['motion_0'])
        edits = sorted(self.report['approach']['edits'], key=lambda e:e['begin_s'])
        offset = 0.
        previous = 0.
        for edit in edits:
            lo, hi, span = edit['begin_s'], edit['source_end_s'], edit['replacement_seconds']
            self.assertGreaterEqual(lo, previous)
            bridge = section(b, lo + offset, lo + offset + span)
            poly = coefficients(bridge.points[0], bridge.points[-1], span)
            bound = spatial_bound(section(a, lo, hi), np.pad(poly, ((0,2),(0,0))), self.model, 'right')
            available = remaining_budget(intervals, reserve, Policy().limits('transit'), lo, hi)
            self.assertTrue(np.all(np.array([bound[k] for k in KEYS]) <= available + 1e-8))
            offset += span - (hi - lo)
            previous = hi
        for key in ('stops',):
            self.assertLessEqual(self.report['approach']['selected'][key]['count'], self.report['approach']['before'][key]['count'])
        self.assertLess(self.report['approach']['selected']['below_5_mm_s'], self.report['approach']['before']['below_5_mm_s'])

    def test_trimmed_pull_exit_and_hand_commands_unchanged(self):
        a, b = self.old['payload'], self.new['payload']
        for key in a['units']:
            if key != 'motion_0':
                self.assertEqual(a['units'][key]['trajectory'], b['units'][key]['trajectory'])
        for key in ('handWaits', 'handPresets', 'sequence'):
            self.assertEqual(a[key], b[key])
        _, old_events = materialize(a, a['sequence'])
        _, new_events = materialize(b, b['sequence'])
        savings = self.report['approach']['before']['seconds'] - self.report['approach']['selected']['seconds']
        self.assertEqual(old_events[0], new_events[0])
        for old, new in zip(old_events[1:], new_events[1:]):
            self.assertEqual(old['posture'], new['posture'])
            self.assertAlmostEqual(old['time'] - new['time'], savings, places=7)

    def test_standard_panel_registered_group(self):
        from task_offline_optimizer_panel import OptimizerPanel, data_index
        from task_trace_selection import restore
        rail = ROOT / 'data/task_sessions/20260924_153036_985614/rail_autosave/config.json'
        gear = ROOT / 'data/task_panel/landing_gear_demonstration.json'
        node = SimpleNamespace(model=self.model,rail=SimpleNamespace(config=restore(read(rail),rail),revision=0),
            landing_gear=SimpleNamespace(config=restore(read(gear),gear),revision=0),
            demo=SimpleNamespace(config={'traces':{}},revision=0),display_pub=Mock(),demo_preview_pub=Mock(),
            marker_pub=Mock(),status='',launch_work=Mock())
        panel = OptimizerPanel(node)
        scroll = QtWidgets.QScrollArea();scroll.setWidgetResizable(True);scroll.setWidget(panel)
        scroll.resize(1000,1100);scroll.show()
        try:
            panel.case.setCurrentIndex(data_index(panel.case, ('rail','pull_cycle')))
            index = panel.comparison_set.findText('拉出接近去迟滞 + 少拉出 5 mm')
            self.assertGreaterEqual(index, 0)
            panel.comparison_set.setCurrentIndex(index)
            for _ in range(5):self.app.processEvents()
            self.assertIsNotNone(panel.viewer.result, panel.viewer.summary.text())
            self.assertFalse(panel.generate.isEnabled())
            self.assertIn('抓握前 X', panel.viewer.path_panel.readout.text())
            self.assertAlmostEqual(panel.viewer.duration(), self.report['full_preview_seconds'], places=7)
            self.assertTrue(scroll.grab().save(str(self.directory / 'standard_panel_ready.png')))
            panel.viewer.view_tabs.setCurrentIndex(1)
            for _ in range(3):self.app.processEvents()
            self.assertTrue(scroll.grab().save(str(self.directory / 'standard_panel_speed.png')))
            node.launch_work.assert_not_called()
        finally:
            panel.halt();panel.viewer.timer.stop();scroll.close()


if __name__ == '__main__':
    unittest.main()
