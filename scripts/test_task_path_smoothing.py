"""Spline and planning integration regressions; no ROS initialization or hardware."""
from dataclasses import replace
import math
from types import SimpleNamespace as S
import unittest
from unittest.mock import Mock

import numpy as np
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectory,JointTrajectoryPoint

from task_motion_planner import TaskPlanner,PlanningFailure
from task_path_smoothing import smooth_candidates,joint_line_trajectory,joint_waypoint_trajectory
from task_planning_config import PlanningParameters,parameters_from_values,controller_bounded_parameters,load_parameters
from task_trajectory import (coefficients,condition_endpoints,samples,stretch_to_limits,
    task_motion_limits,task_jerk_limits,validate_jerk_limits,validate_waist_envelope,time_seconds)


class SplineTests(unittest.TestCase):
    def test_rank_deficient_fit_does_not_create_exploding_controls(self):
        s=np.r_[0.,np.full(40,.5),1.]
        q=np.column_stack((s*.1,s*.2))
        for trajectory,_ in smooth_candidates(['a','b'],q,s,2.):
            self.assertLess(np.max(np.abs([p.positions for p in trajectory.points])),1.)
            self.assertLess(np.max(np.abs([p.velocities for p in trajectory.points])),10.)

    def test_joint_line_subdivision_preserves_exact_curve_and_stationary_boundaries(self):
        start=np.array([.01,-.3]);end=np.array([-.5,1.1])
        tr=joint_line_trajectory(['waist_yaw_joint','left_elbow_joint'],start,end)
        self.assertEqual(len(tr.points),21)
        for a,b in zip(tr.points,tr.points[1:]):
            dt=time_seconds(b)-time_seconds(a)
            for u in (0.,.3,.7,1.):
                t=time_seconds(a)+u*dt;s=t**3*(10+t*(-15+6*t))
                np.testing.assert_allclose(np.polynomial.polynomial.polyval(u,coefficients(a,b,dt)),
                    start+s*(end-start),atol=1e-12)
        for p in (tr.points[0],tr.points[-1]):
            np.testing.assert_allclose(p.velocities,0.,atol=1e-12)
            np.testing.assert_allclose(p.accelerations,0.,atol=1e-12)

    def test_joint_line_supports_zero_motion_without_nan(self):
        tr=joint_line_trajectory(['waist_yaw_joint'],[.1],[.1])
        self.assertTrue(np.isfinite([p.positions for p in tr.points]).all())
        self.assertTrue(all(list(p.positions)==[.1] for p in tr.points))
        with self.assertRaises(ValueError):joint_line_trajectory(['a'],[math.nan],[.2])

    def test_joint_waypoints_preserve_each_line_and_stop_at_corners(self):
        positions=np.array([[.1,.2],[.3,-.1],[.4,.5]])
        tr=joint_waypoint_trajectory(['waist_yaw_joint','left_elbow_joint'],positions)
        self.assertEqual(len(tr.points),41)
        for index in (0,20,40):
            np.testing.assert_allclose(tr.points[index].positions,positions[index//20])
            np.testing.assert_allclose(tr.points[index].velocities,0.)
            np.testing.assert_allclose(tr.points[index].accelerations,0.)
        for index,(a,b) in enumerate(zip(tr.points,tr.points[1:])):
            dt=time_seconds(b)-time_seconds(a);self.assertGreater(dt,0.)
            leg=index//20
            for u in (.2,.7):
                t=time_seconds(a)+u*dt-leg;s=t**3*(10+t*(-15+6*t))
                expected=positions[leg]+s*(positions[leg+1]-positions[leg])
                np.testing.assert_allclose(np.polynomial.polynomial.polyval(u,coefficients(a,b,dt)),expected,atol=1e-12)
        with self.assertRaises(ValueError):joint_waypoint_trajectory(['a'],[[0.],[math.nan]])

    def path(self):
        s=np.linspace(0.,1.,121)
        return ['waist_yaw_joint','left_elbow_joint'],s,np.column_stack((.1+.25*s,.1*np.sin(np.pi*s)))

    def test_stationary_boundaries_exact_endpoints_and_finite_outputs(self):
        names,s,q=self.path()
        candidates=list(smooth_candidates(names,q,s,2.))
        self.assertTrue(candidates)
        for t,fit in candidates:
            for p,expected in ((t.points[0],q[0]),(t.points[-1],q[-1])):
                np.testing.assert_allclose(p.positions,expected,atol=1e-12)
                np.testing.assert_allclose(p.velocities,0.,atol=1e-12)
                np.testing.assert_allclose(p.accelerations,0.,atol=1e-12)
            self.assertLessEqual(fit['max_fit_deviation_deg'],2.)
            self.assertTrue(all(time_seconds(b)>time_seconds(a) for a,b in zip(t.points,t.points[1:])))

    def test_exact_hermite_jerk_continuity_at_internal_knots(self):
        names,s,q=self.path()
        t,_=next(smooth_candidates(names,q,s,2.))
        self.assertGreater(len(t.points),2)
        for a,b,c in zip(t.points,t.points[1:],t.points[2:]):
            ab=time_seconds(b)-time_seconds(a);bc=time_seconds(c)-time_seconds(b)
            first=np.polynomial.polynomial.polyder(coefficients(a,b,ab),3)/ab**3
            second=np.polynomial.polynomial.polyder(coefficients(b,c,bc),3)/bc**3
            np.testing.assert_allclose(first.sum(axis=0),second[0],atol=5e-4)

    def test_limits_and_waist_envelope_at_both_speed_settings(self):
        names,s,q=self.path()
        durations=[]
        for scaling in (1.,.3):
            t,_=next(smooth_candidates(names,q,s,2.));condition_endpoints(t)
            vmax,amax=task_motion_limits(names,scaling,scaling)
            stretch_to_limits(t,vmax,amax,task_jerk_limits(names))
            validate_jerk_limits(t,task_jerk_limits(names))
            validate_waist_envelope(t,-1.,1.,vmax[names[0]],amax[names[0]])
            self.assertTrue(np.isfinite(list(samples(t,vmax,amax))).all())
            durations.append(time_seconds(t.points[-1]))
        self.assertGreater(durations[1],durations[0])

    def test_noise_is_not_forced_into_a_waypoint_stop(self):
        names,s,q=self.path();q[:,0]+=.001*np.sin(s*40*np.pi)
        t,_=next(smooth_candidates(names,q,s,2.))
        self.assertTrue(all(abs(p.velocities[0])>1e-4 for p in t.points[1:-1]))

    def test_invalid_inputs_rejected(self):
        names,s,q=self.path()
        for positions,progress,bound in ((q*float('nan'),s,2.),(q,s[::-1],2.),
                (q,np.zeros(len(s)),2.),(q,s,0.),(q,s,float('nan')),(q[:-1],s,2.)):
            with self.subTest(bound=bound),self.assertRaises(ValueError):
                list(smooth_candidates(names,positions,progress,bound))

    def test_old_snapshots_keep_original_algorithm(self):
        self.assertFalse(parameters_from_values({}).smooth_contact_path)
        for values in ({'smoothing_joint_deviation_deg':0},{'smooth_contact_path':1},
                       {'smoothing_joint_deviation_deg':float('nan')}):
            with self.assertRaises(ValueError):parameters_from_values(values)


class IntegrationTests(unittest.TestCase):
    def setUp(self):
        self.p=TaskPlanner.__new__(TaskPlanner)
        self.p.planning_parameters=replace(PlanningParameters(),smooth_contact_path=True)
        self.p.search_diagnostics={}
        self.q={'joint':0.}
        self.path=RobotTrajectory(joint_trajectory=JointTrajectory(joint_names=['joint'],
            points=[JointTrajectoryPoint(positions=[float(v)]) for v in np.linspace(0.,.1,60)]))
        def tip(side,q,wrist):
            result=np.eye(4);result[0,3]=q['joint'];return result
        self.p.tip=tip
        self.check=Mock();self.check.wrist_from_tip=np.eye(4)
        self.report={'max_position_error_m':0.}
        self.ticket=S(kind='task',velocity=1.,acceleration=1.)

    def run_fit(self,stage='rotate'):
        return self.p.retime_cartesian('left',self.q,self.path,self.ticket,'Test',self.check,stage,self.report)

    def test_adopts_only_validated_candidate_and_preserves_raw_path(self):
        self.p.retime=Mock(return_value={'end':{'joint':.1},'trajectory':'accepted'})
        result=self.run_fit()
        self.assertEqual(result['trajectory'],'accepted')
        self.assertIn('timed_trajectory',self.p.retime.call_args.kwargs)
        self.assertEqual(len(self.path.joint_trajectory.points),60)
        self.assertEqual(self.p.search_diagnostics['path_smoothing'][0]['method'],'quintic_fit')

    def test_all_geometric_failures_fall_back_without_skipping_validation(self):
        def retime(*args,**kwargs):
            if 'timed_trajectory' in kwargs:
                self.report['max_position_error_m']=999.
                raise PlanningFailure('RETIMED_TCP_BOUNDS','rotate','fixture')
            self.assertEqual(self.report['max_position_error_m'],0.)
            return {'original':True}
        self.p.retime=Mock(side_effect=retime)
        self.assertEqual(self.run_fit(),{'original':True})
        self.assertEqual(self.p.search_diagnostics['path_smoothing'][0]['method'],'totg_ruckig_fallback')

    def test_collision_and_joint_margin_try_next_candidate(self):
        for code in ('COLLISION','JOINT_MARGIN','MOTION_PLAN_FAILED'):
            self.p.retime=Mock(side_effect=[PlanningFailure(code,'rotate','fixture'),{'end':self.q}])
            self.assertEqual(self.run_fit(),{'end':self.q})
            self.assertEqual(self.p.retime.call_count,2)

    def test_stop_timeout_and_model_errors_are_never_swallowed(self):
        for error in (TimeoutError('cancelled'),RuntimeError('STOP'),
                      PlanningFailure('SCENE_NOT_READY','rotate','fixture')):
            self.p.retime=Mock(side_effect=error)
            with self.assertRaises(type(error)):self.run_fit()
            self.assertEqual(self.p.retime.call_count,1)

    def test_other_stages_and_disabled_config_use_original_timing(self):
        for stage in ('insert','approach','return_ready'):
            self.p.retime=Mock(return_value={});self.run_fit(stage)
            self.assertNotIn('timed_trajectory',self.p.retime.call_args.kwargs)
        self.p.planning_parameters=PlanningParameters()
        self.p.retime=Mock(return_value={});self.run_fit()
        self.assertNotIn('timed_trajectory',self.p.retime.call_args.kwargs)

    def test_insert_smoothing_is_independently_enabled_and_validated(self):
        self.p.planning_parameters=replace(PlanningParameters(),smooth_insert_path=True)
        self.p.retime=Mock(return_value={});self.run_fit('insert')
        self.assertIn('timed_trajectory',self.p.retime.call_args.kwargs)
        self.p.retime=Mock(return_value={});self.run_fit('rotate')
        self.assertNotIn('timed_trajectory',self.p.retime.call_args.kwargs)

    def test_scoped_joint_path_keeps_endpoint_and_uses_final_validator(self):
        from task_scoped_planner import ScopedTaskPlanner
        self.p=ScopedTaskPlanner.__new__(ScopedTaskPlanner)
        self.p.names=lambda side:['joint'];self.p.validate=Mock()
        self.p.planning_parameters=replace(PlanningParameters(),smooth_joint_path=True)
        self.p.retime=Mock(return_value={'validated':True})
        result=ScopedTaskPlanner.joint_path(self.p,'left',{'joint':.1,'hand':.8},
            {'joint':.5},self.ticket,'Approach',stage='approach')
        self.assertEqual(result,{'validated':True})
        tr=self.p.retime.call_args.kwargs['timed_trajectory']
        self.assertEqual(list(tr.points[0].positions),[.1]);self.assertEqual(list(tr.points[-1].positions),[.5])
        self.assertEqual(self.p.validate.call_args_list[-1].args[0]['hand'],.8)


class MotionBudgetTests(unittest.TestCase):
    def test_config_planner_and_gateway_share_limits_and_tracking_margin(self):
        p,_=load_parameters();settings=p.controller_parameters('left')
        limits,_=task_motion_limits(['left_elbow_joint','waist_yaw_joint'],1.,1.,p)
        self.assertEqual(limits['left_elbow_joint'],settings['max_velocity_rad_s'])
        self.assertEqual(limits['waist_yaw_joint'],settings['task_waist_velocity_limit_rad_s'])
        for v in limits.values():self.assertLessEqual(v+.1,settings['max_measured_velocity_rad_s']+1e-12)

    def test_unsafe_config_limits_are_rejected(self):
        for key in ('arm_velocity_rad_s','waist_velocity_rad_s'):
            for value in (.41,.6,0.,float('nan'),True):
                with self.subTest(key=key,value=value),self.assertRaises(ValueError):
                    parameters_from_values({key:value})

    def test_reload_cannot_exceed_running_gateway_but_can_lower_limits(self):
        p,record=load_parameters()
        bounded,snapshot=controller_bounded_parameters(p,record,{
            'command_arm_velocity_limit_rad_s':'.25','command_waist_velocity_limit_rad_s':'.3'})
        self.assertEqual(bounded.arm_velocity_rad_s,.25);self.assertEqual(bounded.waist_velocity_rad_s,.3)
        self.assertEqual(snapshot['requested_motion_limits']['arm_velocity_rad_s'],.4)
        self.assertEqual(snapshot['values']['arm_velocity_rad_s'],.25)
        lower=replace(p,arm_velocity_rad_s=.2)
        self.assertEqual(controller_bounded_parameters(lower,record,{
            'command_arm_velocity_limit_rad_s':'.4'})[0].arm_velocity_rad_s,.2)
        self.assertEqual(p.arm_velocity_rad_s,.4)
        with self.assertRaises(ValueError):controller_bounded_parameters(p,record,{
            'command_arm_velocity_limit_rad_s':'nan'})

    def test_saved_curve_rescaling_obeys_operator_motion_budget(self):
        from task_saved_test import speed_copy
        p=replace(PlanningParameters(),arm_velocity_rad_s=.2,waist_velocity_rad_s=.15)
        tr=joint_line_trajectory(['waist_yaw_joint','left_elbow_joint'],[0.,0.],[.1,.5])
        result=speed_copy(tr,1.,1.,1.,True,p)
        limits,acc=task_motion_limits(result.joint_names,1.,1.,p)
        list(samples(result,limits,acc))
        validate_waist_envelope(result,-.6,.6,.15,1.5)
        np.testing.assert_allclose(result.points[-1].positions,[.1,.5])


if __name__=='__main__':unittest.main()
