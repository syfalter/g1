"""Opt-in physical-tangent continuation candidates; final validators remain mandatory."""
import math
import time

import numpy as np
from scipy.optimize import minimize
from scipy.spatial.transform import Rotation

from workbench_orientation import orientation_error


class _LocalBudget(Exception):
    pass


class ContactProblem:
    labels = ('position', 'tangent', 'step', 'nominal_tilt', 'nominal_twist',
              'moving_tilt', 'moving_twist')

    def __init__(self, model, link, names, limits, seed, previous, offset, point,
                 reference, center, dorsal, branch, policy, check, deadline, *,
                 position_tolerance=.002, tangent_limit_deg=4.9):
        self.model, self.link, self.names = model, link, list(names)
        if not self.names or len(set(self.names)) != len(self.names):
            raise ValueError('Distinct selected joints required')
        self.previous = dict(previous)
        self.offset = np.asarray(offset, dtype=float).copy()
        self.point = np.asarray(point, dtype=float).copy()
        self.center = np.asarray(center, dtype=float).copy()
        self.dorsal = np.asarray(dorsal, dtype=float).copy()
        if (self.offset.shape != (4, 4) or any(v.shape != (3,) for v in
                (self.point, self.center, self.dorsal)) or not np.isfinite(np.r_[
                self.offset.ravel(), self.point, self.center, self.dorsal]).all()):
            raise ValueError('Invalid contact geometry')
        if (branch not in (-1., 1.) or np.linalg.norm(self.dorsal) < 1e-6 or
                np.linalg.norm(self.point[:2]-self.center[:2]) < 1e-6):
            raise ValueError('Undefined fixed tangent branch')
        self.branch = float(branch)
        self.reference = Rotation.from_quat(reference).as_matrix()
        self.policy = {key:float(policy[key]) for key in ('tilt_deg', 'roll_deg', 'step_deg')}
        if any(not math.isfinite(self.policy[k]) or not .01 < self.policy[k] <= limit
               for k, limit in (('tilt_deg', 5.), ('roll_deg', 10.), ('step_deg', 2.))):
            raise ValueError('Unsupported constrained orientation policy')
        if (not math.isfinite(position_tolerance) or not 0 < position_tolerance <= .002 or
                not math.isfinite(tangent_limit_deg) or not .01 < tangent_limit_deg <= 5.):
            raise ValueError('Invalid constrained candidate tolerance')
        self.position_tolerance, self.tangent_limit_deg = position_tolerance, tangent_limit_deg
        self.initial = np.array([seed[n] for n in self.names], dtype=float)
        self.last = np.array([previous[n] for n in self.names], dtype=float)
        self.lower = np.maximum([limits[n][0]+.021 for n in self.names], self.last-.119)
        self.upper = np.minimum([limits[n][1]-.021 for n in self.names], self.last+.119)
        if (not np.isfinite(np.r_[self.initial, self.last, self.lower, self.upper,
                                 list(previous.values())]).all() or np.any(self.lower >= self.upper)):
            raise ValueError('Invalid guarded contact joint interval')
        self.check, self.deadline = check, deadline
        self.check_budget()
        previous_pose = model.transform(link, previous)@self.offset
        self.previous_rotation = previous_pose[:3, :3]
        self.cached = self.value = None
        self.evaluations = 0

    def check_budget(self):
        self.check()
        if time.monotonic() >= self.deadline:
            raise _LocalBudget

    def state(self, values):
        state = dict(self.previous)
        state.update(zip(self.names, map(float, values)))
        return state

    def moving_reference(self, position):
        radial = position[:2]-self.center[:2]
        initial = self.point[:2]-self.center[:2]
        angle = math.atan2(initial[0]*radial[1]-initial[1]*radial[0], float(initial@radial))
        return Rotation.from_euler('z', angle).as_matrix()@self.reference

    def evaluate(self, values):
        self.check_budget()
        if self.cached is not None and np.array_equal(values, self.cached):
            return self.value
        self.evaluations += 1
        pose, jacobian = self.model.pose_jacobian(self.link, self.state(values), self.names, self.offset)
        rotation, position = pose[:3, :3], pose[:3, 3]
        dp, angular = jacobian[:3], jacobian[3:]
        dr = np.cross(angular.T[:, None, :], rotation.T[None, :, :]).transpose(0, 2, 1)
        error = position-self.point
        delta = (values-self.last)/.119
        rotation_error = rotation-self.reference
        objective = float(error@error/self.position_tolerance**2 + .02*(delta@delta)
                          + .02*np.sum(rotation_error**2)/.01)
        gradient = (2*error@dp/self.position_tolerance**2 + .04*delta/.119
                    + .04*np.einsum('ij,nij->n', rotation_error, dr)/.01)
        constraints = [1-float(error@error)/self.position_tolerance**2]
        derivatives = [-2*error@dp/self.position_tolerance**2]

        radial = position[:2]-self.center[:2]
        normal = rotation@self.dorsal
        dn = np.einsum('nij,j->in', dr, self.dorsal)[:2]
        radial_sq, normal_sq = float(radial@radial), float(normal[:2]@normal[:2])
        if radial_sq < 1e-12 or normal_sq < 1e-12:
            raise ValueError('Degenerate physical tangent projection')
        tangent = self.branch*np.array([-radial[1], radial[0]])
        dt = self.branch*np.vstack((-dp[1], dp[0]))
        scale = math.sqrt(radial_sq*normal_sq)
        cosine = float(normal[:2]@tangent)/scale
        derivative = ((tangent@dn+normal[:2]@dt)/scale
                      - cosine*(normal[:2]@dn/normal_sq+radial@dp[:2]/radial_sq))

        def cosine_constraint(value, jac, degrees):
            threshold = math.cos(math.radians(degrees-.01))
            constraints.append((value-threshold)/(1-threshold))
            derivatives.append(jac/(1-threshold))

        cosine_constraint(cosine, derivative, self.tangent_limit_deg)
        cosine_constraint((np.sum(self.previous_rotation*rotation)-1)/2,
                          np.einsum('ij,nij->n', self.previous_rotation, dr)/2,
                          self.policy['step_deg'])

        # Dense validation follows the reference at actual TCP, not just the waypoint.
        moving = self.moving_reference(position)
        angle_gradient = np.array([-radial[1], radial[0]])@dp[:2]/radial_sq
        moving_derivative = np.cross([0., 0., 1.], moving.T).T
        for ref, dref in ((self.reference, np.zeros_like(dr)),
                          (moving, angle_gradient[:, None, None]*moving_derivative)):
            relative = ref.T@rotation
            drelative = (np.einsum('ij,njk->nik', ref.T, dr)
                         + np.einsum('nji,jk->nik', dref, rotation))
            cosine_constraint(relative[2, 2], drelative[:, 2, 2], self.policy['tilt_deg'])
            # This is the swing/twist Z angle, not a rotation-vector component.
            a, b = relative[0, 0]+relative[1, 1], relative[1, 0]-relative[0, 1]
            da = drelative[:, 0, 0]+drelative[:, 1, 1]
            db = drelative[:, 1, 0]-drelative[:, 0, 1]
            length = math.hypot(a, b)
            if length < 1e-8:
                raise ValueError('Degenerate swing/twist decomposition')
            cosine_constraint(a/length, da/length-a*(a*da+b*db)/length**3,
                              self.policy['roll_deg'])
        self.cached = values.copy()
        self.value = (objective, gradient, np.asarray(constraints), np.asarray(derivatives))
        return self.value

    def validate(self, values):
        if not np.isfinite(values).all():
            return False, {}
        pose = self.model.transform(self.link, self.state(values))@self.offset
        rotation = Rotation.from_matrix(pose[:3, :3])
        actual = rotation.as_quat()
        reference = Rotation.from_matrix(self.reference).as_quat()
        moving = Rotation.from_matrix(self.moving_reference(pose[:3, 3])).as_quat()
        tilt, twist = orientation_error(reference, actual)
        moving_tilt, moving_twist = orientation_error(moving, actual)
        step = math.degrees((Rotation.from_matrix(self.previous_rotation).inv()*rotation).magnitude())
        radial = pose[:2, 3]-self.center[:2]
        normal = (pose[:3, :3]@self.dorsal)[:2]
        scale = np.linalg.norm(radial)*np.linalg.norm(normal)
        tangent = math.inf
        if min(np.linalg.norm(radial), np.linalg.norm(normal)) >= 1e-6:
            tangent = math.degrees(math.acos(float(np.clip(
                self.branch*(normal@np.array([-radial[1], radial[0]]))/scale, -1., 1.))))
        position = float(np.linalg.norm(pose[:3, 3]-self.point))
        record = dict(position_error_m=position, tangent_error_deg=tangent,
                      tilt_deg=tilt, twist_deg=twist, moving_tilt_deg=moving_tilt,
                      moving_twist_deg=moving_twist, adjacent_rotation_deg=step,
                      max_continuation_step_rad=float(np.max(np.abs(values-self.last))))
        valid = (np.isfinite(list(record.values())).all() and position <= self.position_tolerance
                 and tangent <= self.tangent_limit_deg
                 and max(tilt, moving_tilt) <= self.policy['tilt_deg']
                 and max(twist, moving_twist) <= self.policy['roll_deg']
                 and step <= self.policy['step_deg'] and np.all(values >= self.lower)
                 and np.all(values <= self.upper))
        return bool(valid), record


def constrained_contact_ik(model, link, names, limits, seed, previous, point, reference,
                           offset, center, dorsal, branch, policy, check, timeout, *,
                           position_tolerance=.002, tangent_limit_deg=4.9):
    if not math.isfinite(timeout) or timeout <= 0:
        raise ValueError('Invalid local contact budget')
    began = time.monotonic()
    diagnostic = dict(backend='scipy_task_slsqp_v1', code='LOCAL_TASK_FAILED', rpc_seconds=0.,
                      guarded_margin_rad=.021, continuity_interval_rad=.119,
                      candidate_position_tolerance_m=position_tolerance, max_iterations=60,
                      candidate_tangent_limit_deg=tangent_limit_deg, randomized=False)
    try:
        problem = ContactProblem(model, link, names, limits, seed, previous, offset, point,
            reference, center, dorsal, branch, policy, check, began+timeout,
            position_tolerance=position_tolerance, tangent_limit_deg=tangent_limit_deg)
        result = minimize(lambda q: problem.evaluate(q)[0],
                          np.clip(problem.initial, problem.lower, problem.upper),
                          jac=lambda q: problem.evaluate(q)[1], method='SLSQP',
                          bounds=list(zip(problem.lower, problem.upper)),
                          constraints=[dict(type='ineq', fun=lambda q: problem.evaluate(q)[2],
                                            jac=lambda q: problem.evaluate(q)[3])],
                          options=dict(maxiter=60, ftol=1e-8))
        problem.check_budget()
        valid, metrics = problem.validate(result.x)
        problem.check_budget()
        diagnostic.update(metrics, optimizer_status=int(result.status), iterations=int(result.nit),
                          evaluations=problem.evaluations, optimizer_success=bool(result.success),
                          candidate_joints=dict(zip(problem.names, map(float, result.x))))
        if not valid:
            return None, diagnostic
        diagnostic['code'] = 'LOCAL_TASK_VALID'
        return problem.state(result.x), diagnostic
    except _LocalBudget:
        diagnostic['code'] = 'LOCAL_TASK_BUDGET'
        return None, diagnostic
    finally:
        diagnostic['local_seconds'] = time.monotonic()-began
