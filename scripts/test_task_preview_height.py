from copy import deepcopy
from types import SimpleNamespace as S
import unittest
import numpy as np
from task_preview_height import raised_preview_geometry
from task_kinematics import capture_transform


class PreviewHeightTests(unittest.TestCase):
    def test_scene_stays_fixed_in_world_when_robot_rises(self):
        original=np.eye(4);original[:3,3]=[.01,.02,.04]
        geometry={'capture_frame':{'pelvis_from_torso_row_major':original.tolist()},
                  'task_preview':{'axis_center_m':[.36,.1,-.03]}}
        saved=deepcopy(geometry);model=S(transform=lambda *_:np.eye(4))
        for height in (0.,.07,.1,.12):
            result=raised_preview_geometry(geometry,model,{},height)
            local=capture_transform(result,model,{})
            point=np.array([.36,.1,-.03,1.])
            world=np.eye(4);world[2,3]=height
            np.testing.assert_allclose(world@local@point,original@point,atol=1e-12)
            self.assertAlmostEqual((local@point)[2],(original@point)[2]-height)
        self.assertEqual(geometry,saved)

    def test_invalid_height_rejected(self):
        for value in (-.01,.31,float('nan'),float('inf')):
            with self.assertRaises(ValueError):raised_preview_geometry({},None,{},value)

    def test_archive_without_transform_uses_capture_pose_once(self):
        matrix=np.eye(4);matrix[2,3]=.08
        model=S(transform=lambda *_:matrix)
        result=raised_preview_geometry({},model,{},.07)
        self.assertAlmostEqual(result['capture_frame']['pelvis_from_torso_row_major'][2][3],.01)
        self.assertAlmostEqual(matrix[2,3],.08)


if __name__=='__main__':unittest.main()
