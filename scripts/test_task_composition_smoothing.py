import copy
from types import SimpleNamespace
import unittest

import numpy as np
from scipy.interpolate import make_interp_spline
from trajectory_msgs.msg import JointTrajectory,JointTrajectoryPoint

from task_composition_preview import stamp,section,concatenate,at
from task_composition_smoothing import smooth_candidate,evaluate,deviation_bounds


class SmoothingTests(unittest.TestCase):
    def setUp(self):
        identity=np.eye(4);tip=np.eye(4);tip[0,3]=1.
        self.model=SimpleNamespace(limits={'joint':(-2.,2.)},joints={
            'left_index_tcp':('arm','fixed',tip,np.zeros(3),None,'tip',1.,0.),
            'arm':('root','revolute',identity,np.array([0.,0.,1.]),None,'joint',1.,0.)})
        times=np.linspace(0.,4.,81);u=times/4
        q=(.3*(10*u**3-15*u**4+6*u**5)+.002*np.sin(u*30*np.pi)*np.sin(np.pi*u)**2)[:,None]
        spline=make_interp_spline(times,q,k=5,bc_type=([(1,[0.]),(2,[0.])],[(1,[0.]),(2,[0.])]))
        self.source=JointTrajectory(joint_names=['joint'],points=[stamp(JointTrajectoryPoint(
            positions=spline(t).tolist(),velocities=spline(t,1).tolist(),accelerations=spline(t,2).tolist()),t) for t in times])

    def test_continuous_deviation_and_end_derivatives(self):
        before=copy.deepcopy(self.source)
        candidate,report=smooth_candidate(self.source,self.model,'left',tcp_tolerance=.001)
        self.assertLessEqual(report['joint_rad'],.003)
        self.assertLessEqual(report['tcp_m'],.001)
        self.assertGreater(report['joint_rad'],0.)
        self.assertEqual(before,self.source)
        for i in (0,-1):
            for key in ('positions','velocities','accelerations'):
                np.testing.assert_allclose(getattr(candidate.points[i],key),getattr(self.source.points[i],key),atol=1e-12)
        times=np.linspace(0,4,10001)
        error=np.max(np.abs(evaluate(candidate,times)-evaluate(self.source,times)))
        self.assertLessEqual(error,report['joint_rad']+1e-10)

    def test_moving_cut_boundary_and_suffix_are_preserved(self):
        prefix=section(self.source,0,2.)
        candidate,_=smooth_candidate(prefix,self.model,'left')
        joined=concatenate([candidate,section(self.source,2.,4.)])
        for t in np.linspace(2.,4.,101):
            for key in ('positions','velocities','accelerations'):
                np.testing.assert_allclose(getattr(at(joined,t),key),getattr(at(self.source,t),key),atol=1e-9)

    def test_invalid_constraint_rejected(self):
        for bound in (0.,-1.,float('nan')):
            with self.assertRaises(ValueError):smooth_candidate(self.source,self.model,'left',tcp_tolerance=bound)


if __name__=='__main__':unittest.main()
