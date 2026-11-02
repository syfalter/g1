"""OMPL candidate generation and scoped return integration, with no ROS nodes."""
import time
from types import SimpleNamespace as S
import unittest
from unittest.mock import Mock,patch

import numpy as np
from task_joint_route import return_route
from task_motion_planner import PlanningFailure

try:
    import _task_ompl_route as native
except ImportError:
    native=None


@unittest.skipIf(native is None,'Build the optional offline OMPL adapter first')
class NativeRouteTests(unittest.TestCase):
    def test_seeded_detour_is_exact_repeatable_and_collision_checked(self):
        def valid(q):return not (abs(q[0])<.12 and abs(q[1])<.45)
        paths=[]
        for _ in range(2):
            until=time.monotonic()+2.
            path=np.asarray(native.solve([-.8,0.],[.8,0.],[-1.,-1.],[1.,1.],
                valid,lambda:time.monotonic()>=until,19))
            self.assertGreater(len(path),2)
            np.testing.assert_allclose(path[[0,-1]],[[-.8,0.],[.8,0.]],atol=1e-12,rtol=0)
            self.assertTrue(np.all(abs(path)<=1.))
            for a,b in zip(path,path[1:]):
                self.assertTrue(all(valid(q) for q in np.linspace(a,b,101)))
            paths.append(path)
        np.testing.assert_array_equal(*paths)

    def test_cancel_propagates_from_validity_and_termination_callbacks(self):
        def cancel(*_):raise RuntimeError('STOP cancelled')
        for valid,stop in ((cancel,lambda:False),(lambda _:True,cancel)):
            with self.subTest(valid=valid),self.assertRaisesRegex(RuntimeError,'STOP cancelled'):
                native.solve([-.8,0.],[.8,0.],[-1.,-1.],[1.,1.],valid,stop,19)

    def test_approximate_solution_is_not_accepted(self):
        until=time.monotonic()+.03
        result=native.solve([-.8,0.],[.8,0.],[-1.,-1.],[1.,1.],
            lambda q:abs(q[0])>.2,lambda:time.monotonic()>=until,19)
        self.assertEqual(result,[])

    def test_invalid_or_nonfinite_bounds_and_dimensions_rejected(self):
        for start,goal,lower,upper in (([],[],[],[]),([0.],[0.,1.],[-1.],[1.]),
                ([2.],[0.],[-1.],[1.]),([0.],[0.],[1.],[-1.]),([float('nan')],[0.],[-1.],[1.])):
            with self.subTest(start=start),self.assertRaises(ValueError):
                native.solve(start,goal,lower,upper,lambda _:True,lambda:False,19)


@unittest.skipIf(native is None,'Build the optional offline OMPL adapter first')
class ReturnIntegrationTests(unittest.TestCase):
    def fixture(self):
        p=S(names=lambda _:['a','b'],joint_limits=lambda _:{'a':(-1.,1.),'b':(-1.,1.)},
            search_diagnostics={},check_plan=Mock(),validate=Mock(),retime=Mock(return_value={}),
            validate_trajectory_samples=Mock(side_effect=PlanningFailure('COLLISION','return_ready','blocked')))
        q=dict(a=-.8,b=0.,passive=.7);goal=dict(a=.8,b=0.,passive=.7)
        return p,q,goal,S(kind='task',side='left',generation=1)

    def run_route(self,p,q,goal,ticket):
        return return_route(p,'left',q,goal,ticket,'Return ready',dict(code='COLLISION',stage='return_ready'))

    def test_native_candidate_keeps_passive_state_and_requires_final_retime_validation(self):
        p,q,goal,ticket=self.fixture();original=dict(q)
        raw=[[-.8,0.],[-.3,.6],[.3,.6],[.8,0.]]
        def solve(start,end,lower,upper,valid,stop,seed):
            self.assertEqual(seed,19)
            self.assertTrue(valid(raw[1]));self.assertFalse(stop())
            return raw
        with patch.object(native,'solve',side_effect=solve):
            result=self.run_route(p,q,goal,ticket)
        self.assertEqual(q,original)
        self.assertEqual(p.validate.call_args.args[0]['passive'],.7)
        self.assertTrue(p.search_diagnostics['return_routing'][0]['accepted'])
        self.assertEqual(result['joint_route']['waypoints'],4)
        self.assertEqual(p.retime.call_args.kwargs['stage'],'return_ready')
        trajectory=p.retime.call_args.kwargs['timed_trajectory']
        np.testing.assert_allclose(trajectory.points[0].positions,[-.8,0.])
        np.testing.assert_allclose(trajectory.points[-1].positions,[.8,0.])

    def test_final_collision_and_stop_are_not_hidden(self):
        for error in (PlanningFailure('COLLISION','return_ready','dense collision'),RuntimeError('STOP cancelled')):
            p,q,goal,ticket=self.fixture();p.retime.side_effect=error
            with patch.object(native,'solve',return_value=[[-.8,0.],[-.3,.6],[.8,0.]]):
                with self.assertRaises(type(error)):self.run_route(p,q,goal,ticket)
            self.assertFalse(p.search_diagnostics['return_routing'][0]['accepted'])

    def test_empty_partial_and_unbounded_paths_cannot_reach_retime(self):
        for raw,error in (([],PlanningFailure),([[-.8,0.],[.7,0.]],RuntimeError),
                          ([[-.8,0.],[1.1,.5],[.8,0.]],RuntimeError)):
            p,q,goal,ticket=self.fixture()
            with patch.object(native,'solve',return_value=raw),self.assertRaises(error):
                self.run_route(p,q,goal,ticket)
            p.retime.assert_not_called()

    def test_invalid_scope_error_is_not_treated_as_sample_collision(self):
        p,q,goal,ticket=self.fixture()
        p.validate.side_effect=PlanningFailure('SCENE_NOT_READY','return_ready','stale')
        with self.assertRaisesRegex(PlanningFailure,'SCENE_NOT_READY'):
            self.run_route(p,q,goal,ticket)
        p.retime.assert_not_called()


if __name__=='__main__':unittest.main()
