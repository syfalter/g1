import json
from types import SimpleNamespace as S
import unittest
from unittest.mock import Mock,patch
import numpy as np
from scipy.spatial.transform import Rotation
from task_straight_preview import StraightPreviewPlanner,straight_rotations
from task_hook_collision import HandBaseCollision
from task_hand_control import POSTURES,model_joints


class StraightTests(unittest.TestCase):
    def test_steeper_search_has_twist_and_tilt_coverage(self):
        rotations=straight_rotations(45.)
        self.assertEqual(len(rotations),96)
        tilts=[np.degrees(np.arccos(np.clip(-Rotation.from_quat(q).as_matrix()[2,2],-1,1))) for q in rotations]
        self.assertEqual(set(round(x) for x in tilts),{25,35,45})
        steeper=straight_rotations(30.)
        tilts=[np.degrees(np.arccos(np.clip(-Rotation.from_quat(q).as_matrix()[2,2],-1,1))) for q in steeper]
        self.assertEqual(set(round(x) for x in tilts),{10,20,30})
        self.assertAlmostEqual(tilts[0],20.)
        for bad in (float('nan'),0,65):
            with self.assertRaises(ValueError):straight_rotations(bad)

    def test_same_straight_hand_posture_and_recomputed_tcp(self):
        p=StraightPreviewPlanner.__new__(StraightPreviewPlanner)
        p.model=Mock();p.model.transform.return_value=np.eye(4)
        p.search_diagnostics={};p.max_index_tilt=45
        posture,wrist=p.contact_hand_setup('left',{},None)
        self.assertEqual(posture,'dring_two_finger')
        self.assertEqual(p.search_diagnostics['straight_index']['actuator_q'],[0.,0.,.8,.8,1.,1.])
        np.testing.assert_allclose(wrist,np.eye(4))
        from task_calibration import DEFAULTS,snapshot
        _,calibrated=p.contact_hand_setup('left',{},dict(calibration=snapshot(DEFAULTS)))
        np.testing.assert_allclose(calibrated[:3,3],[DEFAULTS[f'left_{a}_mm']/1000 for a in 'xyz'])
        posture,_=p.contact_hand_setup('left',{},dict(center_check={'height_mm':10}))
        self.assertEqual(posture,'index_only')

    def test_elbow_body_collision_is_not_ignored(self):
        shapes=[{'link':name,'kind':'box','parameters':{'size':'.1 .1 .1'},'xyz':[0,0,0],'rpy':[0,0,0]}
                for name in ('left_elbow_link','torso_link','left_palm')]
        model=Mock();model.transform.return_value=np.eye(4)
        with patch('task_hook_collision.subprocess.run',return_value=S(stdout=json.dumps(shapes))):
            checker=HandBaseCollision(model,'unused',[2,2,2])
        self.assertEqual(checker.check_arm_body({},'left'),('left_elbow_link','torso_link'))
        self.assertIsNone(checker.check_arm_body({},'right'))
        self.assertEqual(checker.links,['left_palm'])
        def transform(name,q):
            t=np.eye(4)
            if name=='left_elbow_link':t[0,3]=.2
            return t
        model.transform.side_effect=transform
        self.assertIsNone(checker.check_arm_body({},'left'))
        self.assertAlmostEqual(checker.arm_body_clearance({},'left')[0],.1)


if __name__=='__main__':unittest.main()
