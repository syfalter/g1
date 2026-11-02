import unittest
import numpy as np
from scipy.spatial.transform import Rotation
from task_center_approach import approach_geometry
from task_planning_config import parameters_from_values,load_parameters


class CenterApproachTests(unittest.TestCase):
    def test_both_sides_use_real_center_and_continuous_orientation(self):
        for side in (-1,1):
            initial=np.eye(4);initial[:3,3]=[.2,.2*side,.3]
            pre=np.eye(4);pre[:3,3]=[.4,.15*side,.18]
            pre[:3,:3]=Rotation.from_euler('xyz',[40,30,80],degrees=True).as_matrix()
            center=np.array([.4,.2*side,.05])
            points,nominal,curve,record=approach_geometry(initial,pre,center,.12,60,2.)
            via=np.array(record['center_above_m'])
            self.assertAlmostEqual(via[2],.18)
            self.assertLess(np.min(np.linalg.norm(np.array(points)-via,axis=1)),1e-10)
            self.assertLess(np.max(curve.batch(points)),1e-10)
            np.testing.assert_allclose(points[-1],pre[:3,3])
            quats=nominal.batch([initial[:3,3],*points])
            rot=Rotation.from_quat(quats)
            self.assertLess(np.max((rot[:-1].inv()*rot[1:]).magnitude()),np.deg2rad(2))
            self.assertLess((rot[0].inv()*Rotation.from_matrix(initial[:3,:3])).magnitude(),1e-10)
            self.assertLess((rot[-1].inv()*Rotation.from_matrix(pre[:3,:3])).magnitude(),1e-10)
            for point in points[::20]:
                np.testing.assert_allclose(nominal(point),nominal.batch([point])[0])
                self.assertAlmostEqual(curve(point),curve.batch([point])[0])

    def test_live_rollback_and_legacy_use_joint_approach(self):
        self.assertFalse(parameters_from_values({}).approach_via_center)
        self.assertFalse(load_parameters()[0].approach_via_center)
        self.assertTrue(parameters_from_values({'approach_via_center':True}).approach_via_center)
        for values in ({'approach_center_height_mm':50},{'approach_via_center':1}):
            with self.assertRaises(ValueError):parameters_from_values(values)

    def test_degenerate_route_is_explicitly_rejected(self):
        initial=np.eye(4);initial[:3,3]=[.4,0,.12]
        with self.assertRaises(ValueError):
            approach_geometry(initial,initial,[.4,0,0],.12,60,2.)


if __name__=='__main__':unittest.main()
