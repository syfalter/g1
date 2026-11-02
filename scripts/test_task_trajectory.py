from types import SimpleNamespace as S
import unittest
import numpy as np
from task_trajectory import (coefficients, condition_endpoints, samples, sample_count,
                             stretch_to_limits, validate_waist_envelope, peak, time_seconds,
                             task_jerk_limits, validate_jerk_limits)


def point(q,t,v=0.,a=0.):
    return S(positions=[q],velocities=[v],accelerations=[a],time_from_start=S(sec=t,nanosec=0))


class TrajectoryTests(unittest.TestCase):
    def test_unrepresentable_duration_rejected_before_mutation(self):
        path=S(joint_names=['test'],points=[point(0,0),point(1e20,1)])
        with self.assertRaisesRegex(ValueError,'ROS duration range'):
            stretch_to_limits(path,.3,.75)
        self.assertEqual(time_seconds(path.points[-1]),1.)

    def test_quintic_endpoints_and_derivatives(self):
        a,b=point(.2,0,.1,.2),point(.4,2,.05,-.1)
        c=coefficients(a,b,2)[:,0]
        for t,p in ((0,a),(1,b)):
            self.assertAlmostEqual(np.polynomial.polynomial.polyval(t,c),p.positions[0])
            self.assertAlmostEqual(np.polynomial.polynomial.polyval(t,np.polynomial.polynomial.polyder(c))/2,p.velocities[0])
            self.assertAlmostEqual(np.polynomial.polynomial.polyval(t,np.polynomial.polynomial.polyder(c,2))/4,p.accelerations[0])

    def test_interior_velocity_peak_rejected(self):
        path=S(joint_names=['test'],points=[point(0,0),point(1,1)])
        with self.assertRaises(ValueError): list(samples(path,1.,10.))

    def test_slow_path_passes_and_samples_end(self):
        path=S(joint_names=['test'],points=[point(0,0),point(.01,2)])
        actual=list(samples(path,.3,.75))
        self.assertGreaterEqual(len(actual),401)
        self.assertAlmostEqual(actual[-1][0],.01)
        self.assertEqual(sample_count(path), len(actual))

    def test_invalid_time(self):
        path=S(joint_names=['test'],points=[point(0,1),point(.01,1)])
        with self.assertRaises(ValueError): list(samples(path,.3,.75))

    def test_final_hermite_jerk_is_bounded_without_changing_geometry(self):
        path=S(joint_names=['waist_yaw_joint'],points=[point(0,0),point(.1,1)])
        limits=task_jerk_limits(path.joint_names)
        original=coefficients(*path.points,1.)
        with self.assertRaisesRegex(ValueError,'jerk bound'):
            validate_jerk_limits(path,limits)
        factor=stretch_to_limits(path,.4,.5,limits)
        self.assertGreater(factor,1.)
        validate_jerk_limits(path,limits)
        np.testing.assert_allclose(coefficients(*path.points,time_seconds(path.points[-1])),
                                   original,atol=1e-8)
        validate_waist_envelope(path,-.59,.59,.4,.5)

    def test_jerk_limits_reject_invalid_limits(self):
        for value in (0.,-1.,float('nan'),float('inf')):
            path=S(joint_names=['joint'],points=[point(0,0),point(.1,1)])
            with self.assertRaises(ValueError):stretch_to_limits(path,1.,1.,{'joint':value})

    def test_stretch_preserves_path_and_bounds(self):
        path=S(joint_names=['test'],points=[point(0,0),point(1,1)])
        factor=stretch_to_limits(path,.3,.75)
        self.assertGreater(factor,1.)
        actual=list(samples(path,.3,.75))
        self.assertAlmostEqual(actual[-1][0],1.)

    def test_endpoint_conditioning_zeroes_velocity_and_acceleration(self):
        path=S(joint_names=['test'],points=[point(0,0,.1,.2),point(.5,1,.2,.3),point(1,2,.1,-.2)])
        self.assertIs(condition_endpoints(path),path)
        for endpoint in (path.points[0],path.points[-1]):
            self.assertEqual(endpoint.velocities,[0.])
            self.assertEqual(endpoint.accelerations,[0.])
        self.assertEqual(path.points[1].velocities,[.2])
        self.assertEqual(path.points[1].accelerations,[.3])

    def test_endpoint_conditioning_requires_complete_derivatives(self):
        path=S(joint_names=['test'],points=[point(0,0),point(1,1)])
        path.points[1].accelerations=[]
        with self.assertRaisesRegex(ValueError,'complete'):
            condition_endpoints(path)

    def test_waist_stretch_satisfies_gateway_envelope(self):
        path=S(joint_names=['waist_yaw_joint'],points=[point(0,0),point(.2,1)])
        with self.assertRaises(ValueError):validate_waist_envelope(path,-.59,.59,.4,.5)
        self.assertGreater(stretch_to_limits(path,.4,.5),1.)
        validate_waist_envelope(path,-.59,.59,.4,.5)

    def test_waist_position_overshoot_cannot_be_fixed_by_slowing(self):
        path=S(joint_names=['waist_yaw_joint'],points=[point(.12,0,.4),point(.12,8,-.4)])
        stretch_to_limits(path,.4,.5)
        with self.assertRaises(ValueError):validate_waist_envelope(path,-.59,.59,.4,.5)

    def test_batched_samples_match_scalar_points_and_exact_extrema_checks(self):
        rng=np.random.default_rng(81)
        for _ in range(30):
            a=point(float(rng.uniform(-.1,.1)),0,float(rng.uniform(-.05,.05)))
            b=point(float(rng.uniform(-.1,.1)),2,float(rng.uniform(-.05,.05)))
            trajectory=S(joint_names=['arm'],points=[a,b])
            c=coefficients(a,b,2.)
            d=np.array([i*c[i]/2 for i in range(1,6)])
            dd=np.array([i*d[i]/2 for i in range(1,5)])
            bound=.1
            if peak(d[:,0])>bound+1e-4 or peak(dd[:,0])>bound+1e-4:
                with self.assertRaises(ValueError):list(samples(trajectory,bound,bound))
            else:
                expected=np.array([np.polynomial.polynomial.polyval(t,c) for t in np.linspace(0,1,401)])
                np.testing.assert_allclose(list(samples(trajectory,bound,bound)),expected,atol=1e-14)


if __name__=='__main__': unittest.main()
