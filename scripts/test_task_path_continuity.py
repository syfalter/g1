"""Offline continuation tests, with no ROS node or actuator client."""
from types import SimpleNamespace as S
import unittest
from unittest.mock import Mock,patch

import numpy as np

from task_motion_planner import TaskPlanner,PlanningFailure
from task_path_continuity import ContinuityPolicy,continuation_seed,slope_error,subdivision_point
from task_session import ARMS,ARM_NAMES,Ticket


class ContinuityTests(unittest.TestCase):
    def test_subdivision_is_bounded(self):
        policy=ContinuityPolicy()
        np.testing.assert_allclose(subdivision_point([0,0,0],[1,0,0],0,0,policy),[.5,0,0])
        self.assertIsNone(subdivision_point([0,0,0],[1,0,0],6,0,policy))
        self.assertIsNone(subdivision_point([0,0,0],[1,0,0],0,240,policy))
        self.assertIsNone(subdivision_point([0,0,0],[0,0,0],0,0,policy))

    def test_slope_score_accounts_for_unequal_spacing_and_reversal(self):
        np.testing.assert_allclose(slope_error([[0],[.02]],[.03],2.,1.),0.,atol=1e-12)
        self.assertAlmostEqual(slope_error([[0],[.02]],[.01],2.,1.)[0],-.02)

    def test_predictor_is_step_and_margin_bounded(self):
        seed=continuation_seed(['a','b'],{'a':(-1,1),'b':(-1,1)},dict(a=.97,b=.1),
            [[.95,.08],[.97,.1]],1.,10.,ContinuityPolicy())
        self.assertAlmostEqual(seed['a'],.98)
        self.assertAlmostEqual(seed['b'],.14)

    def planner(self):
        p=TaskPlanner.__new__(TaskPlanner)
        q=dict.fromkeys(ARM_NAMES,0.)
        p.continuity_policy=ContinuityPolicy();p.search_diagnostics={}
        p.progress=Mock();p.last_ik={'code':'VALID'}
        p.joint_limits=Mock(return_value={n:(-2.,2.) for n in ARMS['left']})
        def tip(side,state,offset):
            result=np.eye(4);result[0,3]=state[ARMS['left'][0]]
            return result
        p.tip=tip
        def solve(side,state,target,*args,**kwargs):
            result=dict(state);result[ARMS['left'][0]]=float(target[0,3]);return result
        p.solve=Mock(side_effect=solve)
        p.retime=Mock(side_effect=lambda side,start,path,*args:
            dict(end=dict(start,**dict(zip(path.joint_trajectory.joint_names,
                                         path.joint_trajectory.points[-1].positions)))))
        return p,q

    def test_large_jump_is_subdivided_without_changing_target_or_adding_stops(self):
        p,q=self.planner();ticket=Ticket(1,'task','left',.7,.7)
        with patch('task_motion_planner.orientation_candidates',return_value=[[0,0,0,1]]):
            result=p.cartesian('left',q,[[.08,0,0]],[0,0,0,1],np.eye(4),ticket,
                               'Insert',lambda xyz:0.,'insert')
        path=p.retime.call_args.args[2].joint_trajectory
        positions=np.array([point.positions for point in path.points])
        self.assertLessEqual(np.max(np.abs(np.diff(positions,axis=0))),.04+1e-12)
        self.assertAlmostEqual(result['end'][ARMS['left'][0]],.08)
        self.assertEqual(len(p.search_diagnostics['cartesian'][0]['subdivisions']),1)
        self.assertTrue(all(not point.velocities for point in path.points))
        self.assertEqual(p.retime.call_count,1)

    def test_noncontinuity_failure_does_not_trigger_subdivision(self):
        p,q=self.planner();p.solve=Mock(return_value=None);p.last_ik={'code':'IK_FAILED'}
        with patch('task_motion_planner.orientation_candidates',return_value=[[0,0,0,1]]):
            with self.assertRaises(PlanningFailure):
                p.cartesian('left',q,[[.08,0,0]],[0,0,0,1],np.eye(4),Ticket(1,'task','left',.7,.7),
                            'Insert',lambda xyz:0.,'insert')
        self.assertEqual(p.search_diagnostics['cartesian'][0]['subdivisions'],[])
        p.retime.assert_not_called()

    def test_unresolvable_branch_jump_stops_at_refinement_budget(self):
        p,q=self.planner()
        def solve(side,state,target,*args,**kwargs):
            result=dict(state);result[ARMS['left'][1]]=.2;return result
        p.solve=Mock(side_effect=solve)
        with patch('task_motion_planner.orientation_candidates',return_value=[[0,0,0,1]]):
            with self.assertRaises(PlanningFailure):
                p.cartesian('left',q,[[.08,0,0]],[0,0,0,1],np.eye(4),Ticket(1,'task','left',.7,.7),
                            'Insert',lambda xyz:0.,'insert')
        self.assertEqual(len(p.search_diagnostics['cartesian'][0]['subdivisions']),6)
        self.assertEqual(p.solve.call_count,7)
        p.retime.assert_not_called()

    def test_candidate_ranking_prefers_continued_direction_over_reversal(self):
        p,q=self.planner();calls=[]
        def solve(side,state,target,*args,**kwargs):
            result=dict(state);result[ARMS['left'][0]]=float(target[0,3])
            result[ARMS['left'][1]]=(.02,.015,.04)[len(calls)];calls.append(state)
            return result
        p.solve=Mock(side_effect=solve)
        with patch('task_motion_planner.orientation_candidates',return_value=[[0,0,0,1]]):
            result=p.cartesian('left',q,[[.01,0,0],[.02,0,0]],[0,0,0,1],np.eye(4),
                              Ticket(1,'task','left',.7,.7),'Insert',lambda xyz:0.,'insert')
        self.assertAlmostEqual(result['end'][ARMS['left'][1]],.04)
        self.assertEqual(p.solve.call_count,3)


if __name__=='__main__':unittest.main()
