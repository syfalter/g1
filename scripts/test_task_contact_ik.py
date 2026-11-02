import math
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np
from scipy.spatial.transform import Rotation

from task_contact_ik import ContactProblem, constrained_contact_ik


class CartesianModel:
    names = ['x', 'y', 'z', 'roll', 'pitch', 'yaw']

    def transform(self, link, state):
        pose = np.eye(4)
        pose[:3, 3] = [.4+state['x'], .05+state['y'], .1+state['z']]
        pose[:3, :3] = Rotation.from_euler('xyz', [state[n] for n in self.names[3:]]).as_matrix()
        return pose

    def pose_jacobian(self, link, state, names, offset):
        base = self.transform(link, state)
        pose = base@offset
        yaw = Rotation.from_euler('z', state['yaw'])
        pitch = Rotation.from_euler('y', state['pitch'])
        angular = np.column_stack(((yaw*pitch).apply([1., 0., 0.]), yaw.apply([0., 1., 0.]), [0., 0., 1.]))
        full = np.zeros((6, 6))
        full[:3, :3] = np.eye(3)
        full[3:, 3:] = angular
        full[:3, 3:] = np.cross(angular.T, pose[:3, 3]-base[:3, 3]).T
        return pose, full[:, [self.names.index(n) for n in names]]


class ContactIKTests(unittest.TestCase):
    def setUp(self):
        self.model = CartesianModel()
        self.names = self.model.names
        self.previous = dict(zip(self.names, [0., 0., 0., .2, -.1, .3]))
        self.previous['passive'] = .7
        self.seed = dict(self.previous)
        self.offset = np.eye(4)
        self.offset[:3, 3] = [.013, -.021, .033]
        pose = self.model.transform('tip', self.previous)@self.offset
        self.point = pose[:3, 3]+[.0005, .0003, -.0002]
        self.center = pose[:3, 3]-[.05, 0., 0.]
        self.dorsal = pose[:3, :3].T@[0., 1., 0.]
        self.reference = Rotation.from_matrix(pose[:3, :3]).as_quat()
        self.policy = dict(tilt_deg=4.9, roll_deg=9.9, step_deg=2.)
        self.limits = dict.fromkeys(self.names, (-1., 1.))

    def solve(self, **options):
        kwargs = dict(model=self.model, link='tip', names=self.names, limits=self.limits,
                      seed=self.seed, previous=self.previous, point=self.point, reference=self.reference,
                      offset=self.offset, center=self.center, dorsal=self.dorsal, branch=1.,
                      policy=self.policy, check=lambda: None, timeout=1.)
        kwargs.update(options)
        return constrained_contact_ik(**kwargs)

    def problem(self):
        return ContactProblem(self.model, 'tip', self.names, self.limits, self.seed,
            self.previous, self.offset, self.point, self.reference, self.center, self.dorsal,
            1., self.policy, lambda: None, time.monotonic()+5.)

    def test_objective_and_all_constraints_have_correct_analytic_derivatives(self):
        problem = self.problem()
        for offset in (np.zeros(6), np.array([.001, -.0015, .0004, .012, -.019, .017])):
            q = problem.initial+offset
            value = problem.evaluate(q)
            columns = []
            for delta in np.eye(len(q))*1e-6:
                plus, minus = problem.evaluate(q+delta), problem.evaluate(q-delta)
                columns.append(np.r_[plus[0]-minus[0], plus[2]-minus[2]]/2e-6)
            np.testing.assert_allclose(np.asarray(columns).T, np.vstack((value[1], value[3])),
                                       atol=3e-6, rtol=1e-5)

    def test_candidate_is_repeatable_preserves_previous_passive_state_and_inputs(self):
        self.seed.update(x=.08, passive=-.4)
        saved_seed, saved_previous = dict(self.seed), dict(self.previous)
        answers = []
        for _ in range(2):
            answer, record = self.solve()
            self.assertIsNotNone(answer)
            self.assertEqual(record['code'], 'LOCAL_TASK_VALID')
            self.assertEqual(answer['passive'], .7)
            self.assertLessEqual(record['position_error_m'], .002)
            self.assertLessEqual(record['max_continuation_step_rad'], .119)
            self.assertLessEqual(record['tangent_error_deg'], 4.9)
            self.assertLessEqual(record['adjacent_rotation_deg'], 2.)
            answers.append(answer)
        self.assertEqual(answers[0], answers[1])
        self.assertEqual(self.seed, saved_seed)
        self.assertEqual(self.previous, saved_previous)

    def test_wrong_branch_cannot_flip_or_bypass_orientation_step(self):
        answer, record = self.solve(branch=-1.)
        self.assertIsNone(answer)
        self.assertEqual(record['code'], 'LOCAL_TASK_FAILED')

    def test_unreachable_position_and_invalid_inputs_fail_closed(self):
        answer, record = self.solve(point=self.point+[1., 0., 0.])
        self.assertIsNone(answer)
        self.assertEqual(record['code'], 'LOCAL_TASK_FAILED')
        for options in (dict(position_tolerance=.003), dict(timeout=0.), dict(branch=None),
                        dict(center=self.point), dict(dorsal=[0., 0., 0.]),
                        dict(policy=dict(self.policy, step_deg=3.)),
                        dict(seed=dict(self.seed, x=float('nan')))):
            with self.subTest(options=options), self.assertRaises(ValueError):
                self.solve(**options)

    def test_local_budget_does_not_swallow_task_deadline_or_stop(self):
        for error in (TimeoutError('task deadline'), RuntimeError('STOP')):
            def check():
                raise error
            with self.subTest(error=error), self.assertRaises(type(error)) as caught:
                self.solve(check=check)
            self.assertIs(caught.exception, error)
        with patch('task_contact_ik.time.monotonic', side_effect=[0., 1., 1.]):
            answer, record = self.solve(timeout=.1)
        self.assertIsNone(answer)
        self.assertEqual(record['code'], 'LOCAL_TASK_BUDGET')

    def test_optimizer_success_is_not_a_geometry_acceptance_gate(self):
        invalid = np.array([self.previous[n] for n in self.names])
        invalid[0] += .2
        result = SimpleNamespace(x=invalid, status=0, nit=1, success=True)
        with patch('task_contact_ik.minimize', return_value=result):
            answer, record = self.solve()
        self.assertIsNone(answer)
        self.assertTrue(record['optimizer_success'])

    def test_swing_twist_and_actual_position_reference_match_independent_definitions(self):
        from workbench_orientation import orientation_error
        problem = self.problem()
        values = problem.last+np.array([.001, -.001, .0003, .019, -.028, .024])
        pose = self.model.transform('tip', problem.state(values))@self.offset
        _, report = problem.validate(values)
        a = problem.point[:2]-problem.center[:2]
        b = pose[:2, 3]-problem.center[:2]
        moving = (Rotation.from_euler('z', math.atan2(a[0]*b[1]-a[1]*b[0], a@b))
                  * Rotation.from_quat(self.reference))
        tilt, twist = orientation_error(moving.as_quat(), Rotation.from_matrix(pose[:3, :3]).as_quat())
        self.assertAlmostEqual(report['moving_tilt_deg'], tilt)
        self.assertAlmostEqual(report['moving_twist_deg'], twist)
        self.assertNotAlmostEqual(report['twist_deg'], report['moving_twist_deg'], places=3)


if __name__ == '__main__':
    unittest.main()
