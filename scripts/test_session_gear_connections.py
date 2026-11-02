"""Regression checks for the audited gear routes; never access robot hardware."""
import copy
import unittest

from trajectory_only_candidates import ROOT, read, load, decode, context_for, OfflineModel
from audit_session_gear_connections import boundary


class GearConnectionsTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.model = OfflineModel(ROOT/'data/trajectory_edit_audit/fast_timing_all/panel_robot.urdf')
        cls.report = read(ROOT/'data/rail_deployment/session_inventory_20260929/gear_connections.json')
        cls.payload = load(ROOT/cls.report['gear_bounded_snapshot'],cls.model)['variants']['flow']['payload']
        cls.context = context_for(cls.payload,'lu')

    def test_stationary_connections_and_business_cases(self):
        for scenario in self.report['scenarios'].values():
            self.assertEqual(len(scenario['boundaries']),10)
            self.assertEqual(len(scenario['progress_cases']),24)
            self.assertEqual(len(scenario['sequences']),4)
            self.assertLess(scenario['boundary_max_rad'],1e-7)
            for evidence in scenario['boundaries'].values():
                self.assertTrue(evidence['stationary_c2_passed'])
                self.assertLess(evidence['max_boundary_velocity_or_acceleration'],1e-7)
            for limits in scenario['continuous_limits'].values():
                self.assertLessEqual(limits['jerk'],5.000005)
            self.assertFalse(scenario['physical_verified'])
            self.assertFalse(scenario['production_bindings_rebuilt'])

    def test_reverse_ends_at_lower_not_upper_exit(self):
        reverse = load(ROOT/self.report['existing_reverse_snapshot'],self.model)['variants']['flow']['payload']
        tr = decode(reverse['units']['ul']['trajectory'])
        lower = decode(self.payload['units']['exit_lower']['trajectory'])
        upper = decode(self.payload['units']['exit_upper']['trajectory'])
        self.assertTrue(boundary(tr,-1,lower,0,self.model,self.context)['stationary_c2_passed'])
        with self.assertRaises(ValueError):
            boundary(tr,-1,upper,0,self.model,self.context)

    def test_position_match_alone_cannot_pass(self):
        tr = decode(self.payload['units']['lu']['trajectory'])
        bad = copy.deepcopy(tr)
        bad.points[-1].velocities[0] = .01
        with self.assertRaises(ValueError):
            boundary(tr,-1,bad,-1,self.model,self.context)


if __name__=='__main__':
    unittest.main()
