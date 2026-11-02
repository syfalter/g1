"""Regression gates for the retained-path 5 mm pull truncation."""
import unittest
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
from PyQt5 import QtWidgets

from trajectory_only_candidates import (ROOT, BASE, read, load, decode, checked, evaluate,
    OfflineModel, parameters_from_values, time_seconds)
from task_composition_preview import materialize
from task_spatial_comparison import SpatialComparison


class PullShorterTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
        cls.model = OfflineModel(ROOT / 'data/trajectory_edit_audit/fast_timing_all/panel_robot.urdf')
        cls.directory = ROOT / 'data/rail_deployment/pull_shorter_5mm_20260929'
        cls.report = read(cls.directory / 'report.json')
        cls.old = load(ROOT / cls.report['source_snapshot'], cls.model)['variants']['flow']
        cls.new = load(ROOT / cls.report['candidate_snapshot'], cls.model)['variants']['flow']
        cls.parameters = parameters_from_values(read(BASE / 'bundle.json')['payload']['actions']['pull']['plan']['planning_config']['values'])

    def test_endpoint_and_retained_prefix(self):
        original = decode(self.old['payload']['units']['motion_1']['trajectory'])
        trimmed = decode(self.new['payload']['units']['motion_1']['trajectory'])
        details = self.report['contact']
        lo = details['braking_source_begin_s']
        span = details['braking_seconds']
        t = np.linspace(0., time_seconds(trimmed.points[-1]), 5001)
        u = np.clip((t - lo) / span, 0., 1.)
        mapped = np.where(t <= lo, t, lo + span * (u - u**3 + .5 * u**4))
        np.testing.assert_allclose(evaluate(trimmed, t), evaluate(original, mapped), atol=2e-8, rtol=0.)
        np.testing.assert_allclose(trimmed.points[-1].positions,
            evaluate(original, np.array([details['source_cut_s']]))[0], atol=1e-12, rtol=0.)
        np.testing.assert_allclose(trimmed.points[-1].velocities, 0., atol=1e-12)
        np.testing.assert_allclose(trimmed.points[-1].accelerations, 0., atol=1e-12)
        self.assertAlmostEqual(self.report['release_after_root_mm'][0] - self.report['release_before_root_mm'][0], 5., places=5)
        self.assertLess(time_seconds(trimmed.points[-1]), time_seconds(original.points[-1]))

    def test_limits_full_sequence_and_release_hold(self):
        payload = self.new['payload']
        full, events = materialize(payload, payload['sequence'])
        checked(full, self.model, self.parameters)
        contact = decode(payload['units']['motion_1']['trajectory'])
        hold = decode(payload['units']['hand_2']['trajectory'])
        departure = decode(payload['units']['motion_2']['trajectory'])
        for point in hold.points:
            np.testing.assert_array_equal(point.positions, contact.points[-1].positions)
            np.testing.assert_array_equal(point.velocities, np.zeros(len(hold.joint_names)))
        np.testing.assert_array_equal(departure.points[0].positions, contact.points[-1].positions)
        self.assertEqual([e['posture'] for e in events], ['open', 'pull_grip', 'open'])
        self.assertEqual(payload['handWaits'], self.old['payload']['handWaits'])
        self.assertEqual(payload['handPresets'], self.old['payload']['handPresets'])

    def test_unrelated_geometry_and_policy_unchanged(self):
        old, new = self.old['payload'], self.new['payload']
        for key in ('motion_0', 'hand_0', 'hand_1'):
            self.assertEqual(old['units'][key]['trajectory'], new['units'][key]['trajectory'])
        old_options, new_options = dict(old['options']), dict(new['options'])
        old_options.pop('trajectory_only_revision');new_options.pop('trajectory_only_revision')
        self.assertEqual(old_options, new_options)
        a = decode(old['units']['motion_2']['trajectory'])
        b = decode(new['units']['motion_2']['trajectory'])
        t = np.linspace(self.report['exit']['rejoin_s'], time_seconds(a.points[-1]), 1001)
        for order in range(3):
            np.testing.assert_allclose(evaluate(a, t, order), evaluate(b, t, order), atol=1e-8, rtol=0.)
        for key in ('contact', 'exit'):
            evidence = self.report[key]
            self.assertLessEqual(evidence['selected']['stops']['count'], evidence['before']['stops']['count'])
            self.assertLessEqual(evidence['selected']['below_5_mm_s'], evidence['before']['below_5_mm_s'] + .02)

    def test_release_focused_overlay(self):
        saved = load(ROOT / self.report['comparison_snapshot'], self.model)
        node = SimpleNamespace(model=self.model, marker_pub=Mock())
        spatial = SpatialComparison(node)
        spatial.resize(980,600);spatial.show()
        try:
            spatial.set_results(saved['variants'])
            for _ in range(5):self.app.processEvents()
            self.assertIn('+5.000', spatial.readout.text())
            self.assertIn('松手前 X', spatial.readout.text())
            self.assertLess(spatial.plot.span, 24.001)
            for key in ('original', 'baseline', 'candidate'):
                curve = spatial.curves[key]
                xy = spatial.plot.project(np.asarray(curve['release_xyz_mm'])[None, [0,2]])[0]
                self.assertTrue(spatial.plot.box().contains(float(xy[0]), float(xy[1])))
            self.assertTrue(spatial.grab().save(str(self.directory / 'release_comparison.png')))
        finally:
            spatial.close()


if __name__ == '__main__':
    unittest.main()
