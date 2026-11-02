"""Local timing interpolation and shared execution-limit regressions; no robot I/O."""
import copy
import json
from pathlib import Path
import unittest
from unittest.mock import Mock,patch

import numpy as np

from task_local_timing import materialize,fastest_trajectory
from task_path_smoothing import joint_line_trajectory,joint_waypoint_trajectory
from task_trajectory import coefficients,time_seconds,stretch_to_limits,validate_jerk_limits


class LocalTimingTests(unittest.TestCase):
    def test_large_path_uses_bounded_overlapping_subproblems_and_keeps_limits(self):
        from scipy.optimize import minimize
        source=joint_waypoint_trajectory(['a'],[[0.],[.3],[.32],[.5],[.52],[.83]])
        vmax={'a':.2};amax={'a':1.05};jerk={'a':5.}
        uniform=copy.deepcopy(source)
        factor=stretch_to_limits(uniform,vmax,amax,jerk)
        with patch('task_local_timing.minimize',wraps=minimize) as solve:
            result,report=fastest_trajectory(source,vmax,amax,jerk,1./factor)
        self.assertEqual(report['solver'],'scipy_overlapping_slsqp_time_warp_v1')
        self.assertGreater(solve.call_count,1)
        self.assertTrue(all(len(call.kwargs['bounds'])<=64 for call in solve.call_args_list))
        self.assertLess(time_seconds(result.points[-1]),time_seconds(uniform.points[-1]))
        self.assertEqual(stretch_to_limits(copy.deepcopy(result),vmax,amax,jerk),1.)
        self.assertLessEqual(report['max_joint_path_difference_rad'],1e-7)
        self.assertEqual(result.points[0].positions,source.points[0].positions)
        self.assertEqual(result.points[-1].positions,source.points[-1].positions)

    def test_recorded_rate_ceiling_slows_only_needed_regions(self):
        source=joint_waypoint_trajectory(['a'],[[0.],[.4],[.41]])
        vmax={'a':.1};amax={'a':1.05};jerk={'a':5.}
        uniform=copy.deepcopy(source)
        stretch=stretch_to_limits(uniform,vmax,amax,jerk)
        result,report=fastest_trajectory(source,vmax,amax,jerk,1./stretch,rate_ceiling=1.)
        self.assertLessEqual(report['rate_max'],1.+1e-9)
        self.assertGreater(report['rate_max'],.9)
        self.assertLess(report['rate_min'],.5)
        self.assertLess(time_seconds(result.points[-1]),time_seconds(uniform.points[-1]))
        self.assertEqual(stretch_to_limits(copy.deepcopy(result),vmax,amax,jerk),1.)
        self.assertLessEqual(report['max_joint_path_difference_rad'],1e-7)
        self.assertEqual(result.points[-1].positions,source.points[-1].positions)

    def test_recorded_online_plan_no_longer_inherits_global_bottleneck(self):
        from rosidl_runtime_py.set_message import set_message_fields
        from trajectory_msgs.msg import JointTrajectory
        from task_planning_config import parameters_from_values
        from task_trajectory_speed import scale_arm_segments,restore_baseline
        from workbench_model import OfflineModel
        root=Path(__file__).resolve().parents[1]
        plan=root/'data/task_sessions/20260918_165453_064529/plan_0013/plan.json'
        robot=root/'data/offline_trajectory_library/all_replayable_diverse_preinsert_parallel6_v1/2ca166b9c0876d6596e7/context/robot.urdf'
        if not plan.is_file() or not robot.is_file():self.skipTest('Recorded online fixture unavailable')
        source=json.loads(plan.read_text());segments=[]
        for record in source['segments']:
            trajectory=JointTrajectory();set_message_fields(trajectory,record['trajectory'])
            segments.append(dict(record,trajectory=trajectory))
        result,report=scale_arm_segments(segments,24.,OfflineModel(robot),
            parameters_from_values(source['planning_config']['values']),velocity=.7,acceleration=.7)
        by_label={s['label']:s for s in report['stage_limits']}
        self.assertLess(by_label['Left 120']['duration_after_s'],25.)
        self.assertLess(by_label['Return ready']['duration_after_s'],12.)
        self.assertLess(report['duration_after_s'],65.)
        self.assertLessEqual(report['max_joint_path_difference_rad'],1e-7)
        self.assertEqual(report['hardware_commands_sent'],0)
        for before,after in zip(segments,restore_baseline(result,report)):
            self.assertEqual(before['trajectory'],after['trajectory'])

    def test_continuous_warp_preserves_curve_between_all_output_points(self):
        source=joint_line_trajectory(['a'],[0.],[.4])
        rates=1.2+.7*np.sin(np.linspace(0.,np.pi,len(source.points)))
        actual,error=materialize(source,rates,lambda:None)
        self.assertLessEqual(error,1e-8)
        original_times=np.array([time_seconds(p) for p in source.points])
        dt=np.diff(original_times)
        durations=2*dt/(rates[:-1]+rates[1:])
        boundaries=np.r_[0.,np.cumsum(durations)]
        maximum=0.
        for a,b in zip(actual.points,actual.points[1:]):
            start=time_seconds(a);span=time_seconds(b)-start
            c=coefficients(a,b,span)
            for u in (.1,.3,.5,.7,.9):
                t=start+u*span
                i=min(len(dt)-1,np.searchsorted(boundaries,t,side='right')-1)
                x=(t-boundaries[i])/durations[i]
                k=2*rates[i]/(rates[i]+rates[i+1])
                z=k*x+2*(1-k)*(x**3-.5*x**4)
                original=coefficients(source.points[i],source.points[i+1],dt[i])
                expected=np.polynomial.polynomial.polyval(z,original)
                maximum=max(maximum,float(np.max(np.abs(np.polynomial.polynomial.polyval(u,c)-expected))))
        self.assertLess(maximum,1.1e-8)
        self.assertEqual(actual.points[0].positions,source.points[0].positions)
        self.assertEqual(actual.points[-1].positions,source.points[-1].positions)
        self.assertTrue(all(p.velocities[0]>0 for p in actual.points[1:-1]))
        for p in (actual.points[0],actual.points[-1]):
            self.assertEqual(list(p.velocities),[0.]);self.assertEqual(list(p.accelerations),[0.])

    def test_local_timing_shortens_unequal_return_legs_and_preserves_existing_stop(self):
        source=joint_waypoint_trajectory(['a'],[[0.],[.4],[.5]])
        original=copy.deepcopy(source)
        vmax={'a':.1};amax={'a':1.05};jerk={'a':5.}
        uniform=copy.deepcopy(source)
        stretch=stretch_to_limits(uniform,vmax,amax,jerk)
        result,report=fastest_trajectory(source,vmax,amax,jerk,1./stretch)
        self.assertLess(time_seconds(result.points[-1]),.8*time_seconds(uniform.points[-1]))
        self.assertEqual(source,original)
        middle=[p for p in result.points if abs(p.positions[0]-.4)<1e-12 and abs(p.velocities[0])<1e-12]
        self.assertEqual(len(middle),1)
        self.assertLess(time_seconds(result.points[-1])-time_seconds(middle[0]),time_seconds(middle[0]))
        self.assertEqual(stretch_to_limits(copy.deepcopy(result),vmax,amax,jerk),1.)
        validate_jerk_limits(result,jerk)
        self.assertLessEqual(report['max_joint_path_difference_rad'],1e-8)
        again,_=fastest_trajectory(source,vmax,amax,jerk,1./stretch)
        self.assertEqual(again,result)
        again.points[0].positions=[9.]
        cached,_=fastest_trajectory(source,vmax,amax,jerk,1./stretch)
        self.assertEqual(cached,result)

    def test_cancellation_is_not_swallowed_even_on_cached_paths(self):
        source=joint_line_trajectory(['a'],[0.],[.4])
        check=Mock(side_effect=RuntimeError('cancelled'))
        with self.assertRaisesRegex(RuntimeError,'cancelled'):
            fastest_trajectory(source,{'a':.1},{'a':1.05},{'a':5.},.1,check)

    def test_invalid_rates_rejected(self):
        source=joint_line_trajectory(['a'],[0.],[.4])
        for rates in ([1.],np.zeros(21),np.full(21,float('nan')),np.full(21,-1.)):
            with self.assertRaises(ValueError):materialize(source,rates,lambda:None)


if __name__=='__main__':unittest.main()
