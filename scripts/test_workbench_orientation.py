"""Bounded local orientation search, including planner rejection paths."""
import math
from types import SimpleNamespace
import unittest
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation

from workbench_orientation import orientation_policy, orientation_error, orientation_allowed, orientation_candidates
from workbench_app import WorkbenchApp


class OrientationTests(unittest.TestCase):
    def test_default_policy_and_invalid_bounds(self):
        self.assertEqual(orientation_policy({}), dict(tilt_deg=5., roll_deg=10., step_deg=2.))
        for settings in (dict(tilt_deg=11), dict(roll_deg=-1), dict(step_deg=0),
                         dict(tilt_deg=float('nan')), dict(unknown=2), []):
            with self.assertRaises(ValueError):
                orientation_policy(dict(orientation_tolerance=settings))

    def test_swing_twist_and_quaternion_sign(self):
        nominal = Rotation.from_euler('xyz', [.4, 1.2, -.8])
        actual = nominal * Rotation.from_euler('x', 4, degrees=True) * Rotation.from_euler('z', -9, degrees=True)
        np.testing.assert_allclose(orientation_error(nominal.as_quat(), actual.as_quat()), [4, 9], atol=1e-9)
        np.testing.assert_allclose(orientation_error(-nominal.as_quat(), actual.as_quat()), [4, 9], atol=1e-9)

    def test_random_candidates_obey_cone_roll_step_and_no_drift(self):
        rng = np.random.default_rng(20260909)
        policy = orientation_policy({})
        for _ in range(20):
            nominal = Rotation.random(random_state=rng).as_quat()
            previous = nominal
            for _ in range(100):
                candidates = orientation_candidates(nominal, previous, policy)
                self.assertTrue(candidates)
                for candidate in candidates:
                    self.assertTrue(orientation_allowed(nominal, candidate, policy))
                    change=(Rotation.from_quat(previous).inv()*Rotation.from_quat(candidate)).magnitude()
                    self.assertLessEqual(change, math.radians(2)+1e-9)
                previous = candidates[int(rng.integers(len(candidates)))]

    def test_zero_tolerance_keeps_only_reference(self):
        policy = orientation_policy(dict(orientation_tolerance=dict(tilt_deg=0, roll_deg=0)))
        self.assertEqual(len(orientation_candidates([0,0,0,1], [0,0,0,1], policy)), 1)

    def test_continuous_solver_fallback_and_rejections_both_arms(self):
        for side in ('left', 'right'):
            name = side+'_wrist_roll_joint'
            nominal = Rotation.from_euler('x', 0).as_quat()
            def transform(link, q):
                result = np.eye(4)
                result[:3,:3] = Rotation.from_quat(q.get('orientation', nominal)).as_matrix()
                result[:3,3] = q.get('point', [0,0,0])
                return result
            app = SimpleNamespace(target={}, names=[name], arm_joint=WorkbenchApp.arm_joint)
            app.model=SimpleNamespace(transform=transform, robot=ET.fromstring(
                f'<robot><joint name="{name}"><limit lower="-1" upper="1"/></joint></robot>'))
            start={name:0., 'orientation':nominal}
            point=np.array([.1,0,0])
            calls=[]
            def solve(side, p, quat, q, angle, continuous):
                calls.append(quat)
                if abs(np.dot(quat,nominal))>1-1e-12:return None
                return {name:.02, 'point':p, 'orientation':quat}
            app.solve=solve
            result=WorkbenchApp.solve_relaxed(app,side,point,nominal,start)
            self.assertIsNotNone(result)
            self.assertGreater(len(calls),1)
            np.testing.assert_allclose(result['point'], point)
            for bad in ({name:.3, 'point':point, 'orientation':nominal},
                        {name:.02, 'point':point+.01, 'orientation':nominal},
                        {name:.02, 'point':point, 'orientation':Rotation.from_euler('z',15,degrees=True).as_quat()}):
                app.solve=lambda *a, **kw: bad
                self.assertIsNone(WorkbenchApp.solve_relaxed(app,side,point,nominal,start))
            app.solve=lambda *a, **kw: None
            self.assertIsNone(WorkbenchApp.solve_relaxed(app,side,point,nominal,start))

    def test_interpolated_orientation_and_position_are_checked(self):
        for failure in ('orientation', 'position'):
            def transform(link, q):
                t=np.eye(4)
                bend=math.sin(math.pi*q['left_wrist_roll_joint'])
                if failure=='orientation':
                    t[:3,:3]=Rotation.from_euler('x',6*bend,degrees=True).as_matrix()
                else:t[1,3]=.01*bend
                return t
            app=SimpleNamespace(target={}, names=['left_wrist_roll_joint','waist_yaw_joint','task_handle_angle'],
                active_orientation_reference=[0,0,0,1], model=SimpleNamespace(transform=transform),
                message=lambda *a:None, contact_policy=lambda *a:None, validate=lambda *a:None)
            start=dict.fromkeys(app.names,0.)
            end={**start, 'left_wrist_roll_joint':1.}
            frames=[dict(q=start,phase='INSERT'),dict(q=end,phase='INSERT')]
            with self.subTest(failure=failure), self.assertRaisesRegex(RuntimeError, 'Interpolated'):
                WorkbenchApp.validate_frames(app,'left',frames)


if __name__ == '__main__':
    unittest.main()
