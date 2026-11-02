"""Geometry and generated task-model regression tests, no ROS or hardware."""
import copy
import json
import math
from pathlib import Path
import unittest

import numpy as np
from scipy.spatial.transform import Rotation
from workbench_task_geometry import parse_target, arc_point, insertion_orientation
from workbench_model import OfflineModel

ASSETS = Path(__file__).resolve().parents[1] / 'assets/workbench'


class TaskGeometryTests(unittest.TestCase):
    def setUp(self):
        self.target = dict(axis_center=[0, 0, .74], entry_center=[.04, 0, .74],
                           axis_quaternion_xyzw=[0, 0, 0, 1])

    def test_circle(self):
        np.testing.assert_allclose(arc_point(self.target, 0), self.target['entry_center'])
        for sign in (-1, 1):
            p = arc_point(self.target, sign * 2 * math.pi / 3)
            np.testing.assert_allclose(p, [-.02, sign * math.sqrt(3) * .02, .74], atol=1e-12)

    def test_tilted_axis_and_quaternion_sign(self):
        rotation = Rotation.from_euler('xyz', [.3, -.5, .2])
        self.target['axis_quaternion_xyzw'] = (-rotation.as_quat()).tolist()
        self.target['entry_center'] = (rotation.apply([.04, 0, 0]) + self.target['axis_center']).tolist()
        c, p, n, r = parse_target(self.target)
        for theta in np.linspace(-2, 2, 20):
            delta = arc_point(self.target, theta) - c
            self.assertAlmostEqual(np.linalg.norm(delta), .04)
            self.assertAlmostEqual(np.dot(delta, n), 0)

    def test_invalid_targets(self):
        for field, value in [('axis_center', [0, 0, float('nan')]), ('entry_center', [0, 0, .74]),
                             ('entry_center', [.04, 0, .76]), ('axis_quaternion_xyzw', [0, 0, 0, 0]),
                             ('angle_deg', 121), ('angle_deg', float('nan')), ('preinsert_height_m', -.1),
                             ('frame_id', 'camera')]:
            target = copy.deepcopy(self.target)
            target[field] = value
            with self.subTest(field=field, value=value), self.assertRaises(ValueError):
                parse_target(target)

    def test_insertion_axis(self):
        for yaw in (0, .8, 3):
            q = insertion_orientation(self.target, yaw, .4)
            self.assertAlmostEqual(Rotation.from_quat(q).apply([0, 0, 1])[2], -math.cos(.4))

    def test_generated_hand_mounts(self):
        model = OfflineModel(ASSETS / 'task/robot.urdf')
        for side in ('left', 'right'):
            hand = model.robot.find(f"link[@name='{side}_index_tip']")
            self.assertIsNotNone(hand.find('collision/geometry/mesh'))
            q={side+'_index_1_joint':.15/1.1169}
            relative=np.linalg.inv(model.transform(side+'_rubber_hand',q)) @ model.transform(side+'_index_tcp',q)
            meta=json.loads((ASSETS/'rh56dfx/task/provenance.json').read_text())['hands'][side]
            np.testing.assert_allclose(relative[:3,3],meta['index_tcp_xyz'],atol=1e-8)
            q[side+'_index_1_joint']=1.1
            folded=np.linalg.inv(model.transform(side+'_rubber_hand',q)) @ model.transform(side+'_index_tcp',q)
            self.assertLess(folded[0,3],relative[0,3]-.04)
        limit = model.robot.find("joint[@name='waist_yaw_joint']/limit")
        self.assertLessEqual(float(limit.get('upper')), math.radians(35) + 1e-8)

    def test_recorded_complete_plans(self):
        for side in ('left','right'):
            with self.subTest(arm=side):
                report=json.loads((ASSETS/f'task/last_{side}_full_plan.json').read_text())
                self.assertTrue(report['offline_only'])
                frames=report['frames']
                phases={f['phase'] for f in frames}
                self.assertTrue({'EXTEND_INDEX','FOLD_INDEX','ROTATE_LEFT','ROTATE_RIGHT','RETURN_HOME'} <= phases)
                self.assertEqual(frames[0]['q'],frames[-1]['q'])
                initial_index=frames[0]['q'][side+'_index_1_joint']
                if math.isclose(initial_index,.15/1.1169,abs_tol=1e-8):
                    self.assertTrue({'FOLD_START_INDEX','RESTORE_START_INDEX'} <= phases)
                else:
                    self.assertAlmostEqual(initial_index,1.1)
                for f in frames:
                    if f['phase']=='INSERT':
                        self.assertAlmostEqual(f['q'][side+'_index_1_joint'],.15/1.1169)
                angles=[f['q']['task_handle_angle'] for f in frames]
                self.assertAlmostEqual(max(angles),math.radians(120))
                self.assertAlmostEqual(min(angles),-math.radians(120))
                self.assertLess(report['validation']['max_arc_error_m'],.002)


if __name__ == '__main__':
    unittest.main()
