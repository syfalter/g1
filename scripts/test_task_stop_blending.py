import copy
import unittest
from types import SimpleNamespace as S
import xml.etree.ElementTree as ET

import numpy as np
from trajectory_msgs.msg import JointTrajectory,JointTrajectoryPoint

from task_composition_preview import stamp
from task_composition_smoothing import evaluate
from task_stop_blending import (stop_audit,blend_internal_stops,hermite7,materialize,
                                min_speed,retimed_role_intervals,spatial_bound)
from task_offline_optimizer import validate
from task_trajectory import time_seconds


class StopBlendingTests(unittest.TestCase):
    def setUp(self):
        tip=np.eye(4);tip[0,3]=1.
        self.model=S(robot=ET.fromstring('<robot><joint name="joint"><limit velocity="0.4"/></joint></robot>'),limits={'joint':(-2.,2.)},joints={
            'right_index_tcp':('arm','fixed',tip,np.zeros(3),None,'tip',1.,0.),
            'arm':('root','revolute',np.eye(4),np.array([0.,0.,1.]),None,'joint',1.,0.)})
        self.source=self.path((0.,.1,.2))

    def path(self,positions):
        return JointTrajectory(joint_names=['joint'],points=[stamp(JointTrajectoryPoint(
            positions=[q],velocities=[0.],accelerations=[0.]),i*1.4) for i,q in enumerate(positions)])

    def test_same_direction_artificial_stop_removed_without_slowing(self):
        before=copy.deepcopy(self.source)
        tr,report=blend_internal_stops(self.source,self.model,'right',None,5.,(.003,.001,.01))
        self.assertTrue(report['edits'])
        self.assertEqual(report['before']['count'],1);self.assertEqual(report['after']['count'],0)
        self.assertLess(time_seconds(tr.points[-1]),time_seconds(self.source.points[-1]))
        self.assertEqual(self.source,before)
        for i in (0,-1):
            for field in ('positions','velocities','accelerations'):
                np.testing.assert_allclose(getattr(tr.points[i],field),getattr(before.points[i],field),atol=1e-12)
        validate(tr,self.model,None,5.)

    def test_real_reversal_not_forced_to_nonzero_speed(self):
        source=self.path((0.,.1,0.))
        tr,report=blend_internal_stops(source,self.model,'right',None,5.,(.003,.001,.01))
        self.assertFalse(report['edits']);self.assertEqual(tr,source)

    def test_c2_candidate_preserves_boundaries_and_limits(self):
        tr,report=blend_internal_stops(self.source,self.model,'right',None,5.,(.003,.001,.01),fit_order=5)
        self.assertTrue(report['edits'])
        for i in (0,-1):
            for field in ('positions','velocities','accelerations'):
                np.testing.assert_allclose(getattr(tr.points[i],field),getattr(self.source.points[i],field),atol=1e-10)
        validate(tr,self.model,None,5.)

    def test_unsupported_fit_order_rejected(self):
        with self.assertRaises(ValueError):
            blend_internal_stops(self.source,self.model,'right',None,5.,(.003,.001,.01),fit_order=3)

    def test_septic_matches_boundary_through_jerk_and_export_checks(self):
        lo,hi=.6,2.2;span=1.2;poly=hermite7(self.source,lo,hi,span)
        for order in range(4):
            actual=np.polynomial.polynomial.polyval(np.array([0.,1.]),np.polynomial.polynomial.polyder(poly,order)).T/span**order
            np.testing.assert_allclose(actual,evaluate(self.source,np.array([lo,hi]),order),atol=1e-10)
        from task_composition_preview import at
        tr,error=materialize(poly,span,['joint'],at(self.source,lo),at(self.source,hi),lambda:None)
        times=np.linspace(0.,span,1001)
        for order,tolerance in enumerate((1e-9,1e-6,1e-5,1e-4)):
            exact=np.polynomial.polynomial.polyval(times/span,np.polynomial.polynomial.polyder(poly,order)).T/span**order
            self.assertLessEqual(np.max(abs(evaluate(tr,times,order)-exact)),tolerance*1.01)
        self.assertLessEqual(error,1e-9)
        sampled=np.min(abs(np.polynomial.polynomial.polyval(times/span,np.polynomial.polynomial.polyder(poly)))/span)
        self.assertLessEqual(min_speed(poly,span),sampled+1e-7)

    def test_tight_tolerance_rejects_instead_of_relaxing(self):
        tr,report=blend_internal_stops(self.source,self.model,'right',None,5.,(1e-12,1e-12,1e-12))
        self.assertFalse(report['edits']);self.assertEqual(tr,self.source)

    def test_role_boundaries_use_retained_points_and_missing_is_conservative(self):
        ranges=[(0.,1.4,'transit'),(1.4,2.8,'operation')]
        self.assertEqual(retimed_role_intervals(self.source,self.source,ranges),[(0.,1.4,'transit')])
        changed=copy.deepcopy(self.source);changed.points[1].positions=[.105]
        self.assertEqual(retimed_role_intervals(self.source,changed,ranges),[])

    def test_spatial_bound_uses_ordered_geometry_not_old_stop_clock(self):
        poly=np.zeros((8,1));poly[1]=.2
        bound=spatial_bound(self.source,poly,self.model,'right')
        self.assertLess(bound['joint_rad'],1e-4)
        self.assertGreater(np.max(abs(evaluate(self.source,np.linspace(0.,2.8,101))[:,0]-np.linspace(0.,.2,101))),.01)

    def test_quiet_interval_can_include_both_endpoints_without_empty_sections(self):
        source=self.path((0.,.0001,.0002))
        tr,report=blend_internal_stops(source,self.model,'right',None,5.,(.003,.001,.01),compress_idle=True)
        self.assertTrue(any(e['quiet_compression'] for e in report['edits']))
        self.assertLess(time_seconds(tr.points[-1]),time_seconds(source.points[-1])*.5)
        for i in (0,-1):
            for field in ('positions','velocities','accelerations'):
                np.testing.assert_allclose(getattr(tr.points[i],field),getattr(source.points[i],field),atol=1e-10)
        validate(tr,self.model,None,5.)


if __name__=='__main__':unittest.main()
