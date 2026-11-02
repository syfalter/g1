import copy
import unittest
from types import SimpleNamespace as S
from unittest.mock import patch

import numpy as np
from trajectory_msgs.msg import JointTrajectory,JointTrajectoryPoint

from task_composition_preview import stamp,at
from task_smooth_timing import fit_regularized,path_bound,smoothness,smooth_candidate
from task_trajectory_speed import rescale
from task_trajectory import time_seconds,task_jerk_limits


class SmoothTimingTests(unittest.TestCase):
    def setUp(self):
        tip=np.eye(4);tip[0,3]=1.
        self.model=S(limits={'joint':(-2.,2.)},joints={
            'right_index_tcp':('arm','fixed',tip,np.zeros(3),None,'tip',1.,0.),
            'arm':('root','revolute',np.eye(4),np.array([0.,0.,1.]),None,'joint',1.,0.)})
        self.source=JointTrajectory(joint_names=['joint'],points=[stamp(JointTrajectoryPoint(
            positions=[q],velocities=[0.],accelerations=[0.]),t) for t,q in ((0.,0.),(2.,.15),(4.,.3))])

    def test_regularized_curve_preserves_endpoints_and_jerk_continuity(self):
        original=copy.deepcopy(self.source)
        tr=fit_regularized(self.source,.12,.03,robust=True)
        self.assertEqual(original,self.source)
        for index in (0,-1):
            for key in ('positions','velocities','accelerations'):
                np.testing.assert_allclose(getattr(tr.points[index],key),getattr(original.points[index],key),atol=1e-10)
        metrics=smoothness(tr)
        self.assertLess(metrics['max_jerk_jump'],1e-4)
        self.assertLess(metrics['endpoint_jerk_max'],1e-4)

    def test_continuous_bound_not_only_sample_distance(self):
        moved=copy.deepcopy(self.source)
        for p in moved.points:p.positions[0]+=.001
        bounds=path_bound(self.source,moved,self.model,'right')
        self.assertGreaterEqual(bounds['joint_rad'],.001-1e-12)
        self.assertAlmostEqual(bounds['tcp_m'],.001,places=10)
        # Same position knots, different acceleration still changes the path.
        moved=copy.deepcopy(self.source);moved.points[1].accelerations=[.01]
        self.assertGreater(path_bound(self.source,moved,self.model,'right')['joint_rad'],0.)

    def test_shape_metrics_do_not_reward_uniform_slowdown(self):
        a=smoothness(self.source);b=smoothness(rescale(self.source,.5))
        for key in ('normalized_jerk_energy','normalized_acceleration_variation','normalized_jerk_variation'):
            self.assertAlmostEqual(a[key],b[key],delta=abs(a[key])*1e-10)
        self.assertAlmostEqual(b['acceleration_total_variation'],a['acceleration_total_variation']/4.)

    def test_robust_fit_reduces_isolated_outlier_influence(self):
        clean=fit_regularized(self.source,.12,.03)
        noisy=copy.deepcopy(clean);center=len(noisy.points)//3
        noisy.points[center].positions[0]+=.03
        ordinary=fit_regularized(noisy,.12,.03,robust=False)
        robust=fit_regularized(noisy,.12,.03,robust=True)
        t=time_seconds(noisy.points[center])
        truth=at(clean,t).positions[0]
        self.assertLess(abs(at(robust,t).positions[0]-truth),abs(at(ordinary,t).positions[0]-truth))

    def test_moving_business_boundary_rejected(self):
        self.source.points[0].velocities=[.1]
        with self.assertRaises(ValueError):fit_regularized(self.source,.12,.03)

    def test_unaccepted_fit_does_not_relax_tolerance_or_jerk(self):
        with patch('task_trajectory_speed.motion_limits',return_value=({'joint':.4},{'joint':1.5})):
            tr,report=smooth_candidate(self.source,self.model,'right',None,5.,[1e-12]*3,4.)
        self.assertIsNone(tr);self.assertFalse(report['accepted'])
        self.assertTrue(all(t['rejected']=='path_tolerance' for t in report['trials']))
        self.assertEqual(task_jerk_limits(['joint']),{'joint':5.})

    def test_search_meets_final_limits_and_time_budget(self):
        with patch('task_trajectory_speed.motion_limits',return_value=({'joint':.4},{'joint':1.5})):
            tr,report=smooth_candidate(self.source,self.model,'right',None,5.,[.003,.003,.003],4.)
        self.assertIsNotNone(tr)
        self.assertTrue(report['accepted'])
        self.assertLessEqual(time_seconds(tr.points[-1]),4.6)
        self.assertLessEqual(report['chosen']['peaks']['jerk'],5.)
        self.assertLess(report['chosen']['metrics']['max_jerk_jump'],.0005)


if __name__=='__main__':unittest.main()
