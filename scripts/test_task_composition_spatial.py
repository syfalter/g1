import copy
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from task_composition_preview import stamp, at, concatenate, assemble_phases
from task_composition_spatial import rebuild, continuous_gear_phases
from task_spatial_valleys import correspondence_bound, valley_centers


class SpatialTests(unittest.TestCase):
    def setUp(self):
        tip=np.eye(4);tip[0,3]=1.
        self.model=SimpleNamespace(limits={'joint':(-2.,2.)},joints={
            'right_index_tcp':('arm','fixed',tip,np.zeros(3),None,'tip',1.,0.),
            'arm':('root','revolute',np.eye(4),np.array([0.,0.,1.]),None,'joint',1.,0.)})
        self.source=JointTrajectory(joint_names=['joint'],points=[stamp(JointTrajectoryPoint(
            positions=[q],velocities=[0.],accelerations=[0.]),t)
            for t,q in ((0.,0.),(2.,.15),(4.,.3))])

    def test_internal_stop_replaced_without_changing_source_or_ends(self):
        before=copy.deepcopy(self.source)
        tr,report=rebuild(self.source,self.model,'right')
        self.assertEqual(self.source,before)
        middle=min(tr.points,key=lambda p:abs(p.positions[0]-.15))
        self.assertGreater(middle.velocities[0],1e-5)
        self.assertLessEqual(report['tcp_m'],.001)
        self.assertLessEqual(report['joint_rad'],.003)
        self.assertFalse(report['internal_stop_constraints_preserved'])
        for a,b in zip((before.points[0],before.points[-1]),(tr.points[0],tr.points[-1])):
            for key in ('positions','velocities','accelerations'):
                np.testing.assert_allclose(getattr(a,key),getattr(b,key),atol=1e-10)
        self.assertTrue(all(at(tr,t).velocities[0]>0 for t in np.linspace(.2,3.8,100)))

    def test_mixed_unit_uses_slower_rate(self):
        phases={'a':dict(trajectory=self.source,motionClass='mixed',report={})}
        with patch('task_composition_preview.validate_timing',return_value={}):
            for op,transit in ((.5,1.),(1.,.5)):
                units=assemble_phases(phases,{'unit':['a']},None,None,op,transit,lambda:None)
                self.assertAlmostEqual(units['unit']['seconds'],8.)

    def test_bounded_local_blend_shortens_artificial_stop(self):
        from task_trajectory import time_seconds
        tr,report=rebuild(self.source,self.model,'right',seams=[2.])
        self.assertTrue(report['local_blends'])
        self.assertLess(time_seconds(tr.points[-1]),4.)
        self.assertGreater(report['local_blends'][0]['candidate_mid_speed_rad_s'],.001)
        self.assertLessEqual(report['tcp_m'],.001)
        for i in (0,-1):
            np.testing.assert_allclose(tr.points[i].velocities,[0.],atol=1e-12)
            np.testing.assert_allclose(tr.points[i].accelerations,[0.],atol=1e-12)

    def test_grouping_preserves_reversal_boundaries_and_reuses_units(self):
        phases={'a':dict(trajectory=self.source,motionClass='transit')}
        recipes={name:['a'] for name in ('entry_lower','entry_upper','lu','ul','exit_lower','exit_upper')}
        grouped,result=continuous_gear_phases(phases,recipes)
        self.assertEqual(len(grouped),6)
        self.assertEqual(result['cycle_lower'],result['lu']+result['ul'])
        self.assertEqual(result['cycle_upper'],result['ul']+result['lu'])
        for phase in grouped.values():
            np.testing.assert_allclose(phase['trajectory'].points[-1].velocities,[0.],atol=1e-12)

    def test_invalid_bounds_rejected(self):
        with self.assertRaises(ValueError):rebuild(self.source,self.model,'right',tcp_tolerance=0.)

    def test_spatial_correspondence_does_not_inherit_pause_clock(self):
        candidate=JointTrajectory(joint_names=['joint'],points=[stamp(JointTrajectoryPoint(
            positions=[q],velocities=[0.],accelerations=[0.]),t)
            for t,q in ((0.,0.),(1.,.3))])
        bound=correspondence_bound(self.source,candidate,self.model,'right',lambda:None)
        self.assertLess(bound['tcp_m'],.001)
        # A displaced path must not pass simply because its endpoints progress.
        candidate.points[-1].positions[0]+=.01
        bound=correspondence_bound(self.source,candidate,self.model,'right',lambda:None)
        self.assertGreaterEqual(bound['joint_rad'],.009999)

    def test_valley_detection_searches_around_not_only_at_edit_marker(self):
        centers=valley_centers(self.source,[2.2])
        self.assertTrue(any(abs(t-2.)<.02 for t in centers))
        self.assertFalse(valley_centers(self.source,[]))


if __name__=='__main__':unittest.main()
