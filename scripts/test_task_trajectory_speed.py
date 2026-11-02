import copy
import json
from types import SimpleNamespace
import unittest
import xml.etree.ElementTree as ET

import numpy as np
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from task_trajectory import coefficients, time_seconds, stretch_to_limits, task_jerk_limits
from task_trajectory_speed import rescale, scale_arm_segments, motion_limits, preview_segments, speed_capacity


class SpeedTests(unittest.TestCase):
    def test_recorded_local_policy_restores_source_and_respects_requested_ceiling(self):
        from task_path_smoothing import joint_waypoint_trajectory
        from task_trajectory_speed import restore_baseline
        tr=joint_waypoint_trajectory(['left_wrist_yaw_joint'],[[0.],[.4],[.41]])
        source=[dict(trajectory=tr)]
        result,report=scale_arm_segments(source,1.,self.model,policy='recorded')
        self.assertEqual(report['timing_policy'],'recorded_local_v1')
        self.assertLessEqual(report['rate_max'],1.+1e-9)
        self.assertLessEqual(report['max_joint_path_difference_rad'],1e-7)
        restored=restore_baseline(result,report)
        self.assertEqual(restored[0]['trajectory'],tr)
        self.assertEqual(result[0]['trajectory'].points[-1].positions,tr.points[-1].positions)

    def setUp(self):
        self.model=SimpleNamespace(robot=ET.fromstring(
            '<robot><joint name="left_wrist_yaw_joint"><limit velocity="0.1"/></joint></robot>'))
        self.trajectory=JointTrajectory(joint_names=['left_wrist_yaw_joint'])
        for t,q in ((0,0.),(10,.2)):
            p=JointTrajectoryPoint(positions=[q],velocities=[0.],accelerations=[0.])
            p.time_from_start.sec=t
            self.trajectory.points.append(p)

    def test_common_multiplier_caps_urdf_and_preserves_curve_and_hand(self):
        source=[dict(kind='arm',trajectory=self.trajectory),
                dict(kind='arm',trajectory=rescale(self.trajectory,.5)),
                dict(kind='hand',trajectory=copy.deepcopy(self.trajectory))]
        before=copy.deepcopy(source)
        result,report=scale_arm_segments(source,8.,self.model,policy='uniform')
        self.assertEqual(source,before)
        self.assertTrue(report['limited'])
        json.dumps(report,allow_nan=False)
        self.assertEqual(result[-1],source[-1])
        for old,new in zip(source[:2],result[:2]):
            a,b=old['trajectory'],new['trajectory']
            np.testing.assert_allclose(coefficients(*a.points,time_seconds(a.points[-1])),
                                       coefficients(*b.points,time_seconds(b.points[-1])),atol=1e-7)
            self.assertAlmostEqual(time_seconds(a.points[-1])/time_seconds(b.points[-1]),report['applied_multiplier'])
            vmax,amax=motion_limits(b,self.model,1.,1.,None)
            self.assertEqual(stretch_to_limits(copy.deepcopy(b),vmax,amax,task_jerk_limits(b.joint_names)),1.)

    def test_hand_only_keeps_timing(self):
        source=[dict(kind='hand',trajectory=self.trajectory)]
        result,report=scale_arm_segments(source,8.,self.model)
        self.assertEqual(result,source)
        self.assertEqual(report['applied_multiplier'],1.)
        self.assertIsNone(report['maximum_multiplier'])

    def test_maximum_is_independent_of_request_and_is_executable(self):
        source=[dict(label='Approach',trajectory=self.trajectory),
                dict(label='Rotate',trajectory=rescale(self.trajectory,.1))]
        _,slow=scale_arm_segments(source,1.,self.model,policy='uniform')
        result,fast=scale_arm_segments(source,100.,self.model,policy='uniform')
        self.assertAlmostEqual(slow['maximum_multiplier'],fast['maximum_multiplier'])
        self.assertAlmostEqual(fast['applied_multiplier'],fast['maximum_multiplier'])
        self.assertEqual(fast['limiting_constraint']['label'],'Approach')
        self.assertEqual(fast['limiting_constraint']['constraint'],'velocity')
        self.assertAlmostEqual(fast['maximum_multiplier'],.1/.0375/1.005)
        _,at_max=scale_arm_segments(source,fast['maximum_multiplier'],self.model,policy='uniform')
        self.assertFalse(at_max['limited'])
        self.assertGreater(time_seconds(result[0]['trajectory'].points[-1]),0.)

    def test_capacity_detects_acceleration_and_jerk_between_points(self):
        model=SimpleNamespace(robot=ET.fromstring(
            '<robot><joint name="left_wrist_yaw_joint"><limit velocity="100"/></joint></robot>'))
        params=SimpleNamespace(arm_velocity_rad_s=100.,waist_velocity_rad_s=100.)
        for acceleration,constraint in ((1.,'jerk'),(.05,'acceleration')):
            capacity=speed_capacity(self.trajectory,model,params,acceleration=acceleration)
            self.assertEqual(capacity['limiting_constraint']['constraint'],constraint)
            result,_=scale_arm_segments([dict(trajectory=self.trajectory)],capacity['maximum_multiplier'],
                model,params,acceleration=acceleration)
            vmax,amax=motion_limits(result[0]['trajectory'],model,1.,acceleration,params)
            self.assertEqual(stretch_to_limits(copy.deepcopy(result[0]['trajectory']),vmax,amax,
                task_jerk_limits(self.trajectory.joint_names)),1.)

    def test_zero_motion_has_no_finite_derivative_cap(self):
        trajectory=copy.deepcopy(self.trajectory)
        trajectory.points[-1].positions=[0.]
        _,report=scale_arm_segments([dict(trajectory=trajectory)],24.,self.model)
        self.assertIsNone(report['maximum_multiplier'])
        self.assertIsNone(report['limiting_constraint'])
        json.dumps(report,allow_nan=False)

    def test_guarded_custom_speed_has_no_arbitrary_32x_cap(self):
        for factor in (.025,73.125):
            result,report=scale_arm_segments([dict(trajectory=self.trajectory)],factor,self.model)
            self.assertEqual(report['requested_multiplier'],factor)
            self.assertLessEqual(report['applied_multiplier'],factor)
            self.assertTrue(report['limits_enforced'])

    def test_limit_scaling_is_a_cap_not_a_second_speed_multiplier(self):
        source=[dict(trajectory=rescale(self.trajectory,.01))]
        _,full=scale_arm_segments(source,24.,self.model,velocity=1.,acceleration=1.)
        _,reduced=scale_arm_segments(source,24.,self.model,velocity=.5,acceleration=.5)
        self.assertAlmostEqual(full['applied_multiplier'],24.)
        self.assertAlmostEqual(reduced['applied_multiplier'],24.)

    def test_independent_stage_limits_do_not_throttle_slower_rotation(self):
        source=[dict(label='Approach',trajectory=self.trajectory),
                dict(label='Rotate',trajectory=rescale(self.trajectory,.1))]
        result,report=scale_arm_segments(source,24.,self.model)
        self.assertEqual(report['timing_policy'],'local_path_v1')
        approach,rotate=report['stage_limits']
        self.assertLess(approach['applied_multiplier'],3.)
        self.assertAlmostEqual(rotate['applied_multiplier'],24.)
        self.assertGreater(report['maximum_multiplier'],24.)
        self.assertGreater(report['applied_multiplier'],approach['applied_multiplier'])

    def test_local_retiming_restores_exact_baseline_without_compounding(self):
        from task_trajectory_speed import restore_baseline
        source=[dict(label='Approach',trajectory=self.trajectory),
                dict(label='Rotate',trajectory=rescale(self.trajectory,.1))]
        first,report=scale_arm_segments(source,24.,self.model)
        restored=restore_baseline(first,report)
        for a,b in zip(source,restored):self.assertEqual(a['trajectory'],b['trajectory'])
        again,second=scale_arm_segments(restored,2.,self.model)
        direct,expected=scale_arm_segments(source,2.,self.model)
        self.assertEqual(again,direct)
        self.assertEqual(second,expected)
        del report['baseline_trajectories']
        with self.assertRaises(ValueError):restore_baseline(first,report)

    def test_preview_uses_exact_custom_speed_without_software_cap(self):
        source=[dict(kind='arm',trajectory=self.trajectory),dict(kind='hand',trajectory=self.trajectory)]
        for factor in (.01,24.,73.125,100.):
            result,report=preview_segments(source,factor)
            self.assertEqual(report['applied_multiplier'],factor)
            self.assertFalse(report['limits_enforced'])
            self.assertFalse(report['hardware_executable'])
            self.assertFalse(report['hand_timing_preserved'])
            self.assertAlmostEqual(report['duration_after_s'],20./factor,places=7)
            self.assertAlmostEqual(time_seconds(result[1]['trajectory'].points[-1]),10./factor,places=7)
            json.dumps(report,allow_nan=False)

    def test_preview_supports_hand_without_derivatives(self):
        hand=copy.deepcopy(self.trajectory)
        for point in hand.points:point.velocities=[];point.accelerations=[]
        result,report=preview_segments([dict(kind='hand',trajectory=hand)],40.)
        self.assertAlmostEqual(report['duration_after_s'],.25)
        self.assertEqual(list(result[0]['trajectory'].points[0].velocities),[])

    def test_preview_invalid_number_and_unrepresentable_time_rejected(self):
        for factor in (0.,-1.,float('nan'),float('inf'),1e-100,1e100):
            with self.assertRaises(ValueError):preview_segments([dict(trajectory=self.trajectory)],factor)

    def test_unrestricted_preview_cannot_authorize_hardware(self):
        from task_saved_test import live_test_plan,verify_test_plan
        with self.assertRaisesRegex(RuntimeError,'preview timing'):
            live_test_plan(dict(preview_only_timing=True),None,0,None,scene_confirmed=True,contact_confirmed=True)
        with self.assertRaisesRegex(RuntimeError,'preview timing'):
            verify_test_plan(dict(preview_only_timing=True))

    def test_malformed_and_invalid_scaling_rejected(self):
        for factor in (0.,-1.,float('nan'),float('inf')):
            with self.assertRaises(ValueError):rescale(self.trajectory,factor)
        self.trajectory.points[1].velocities=[]
        with self.assertRaises(ValueError):rescale(self.trajectory,2.)

    def test_collapsed_timestamps_rejected(self):
        self.trajectory.points[1].time_from_start.sec=0
        self.trajectory.points[1].time_from_start.nanosec=1
        with self.assertRaises(ValueError):rescale(self.trajectory,32.)


if __name__=='__main__':unittest.main()
