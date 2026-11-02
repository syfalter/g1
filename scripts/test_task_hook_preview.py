import json
import os
from pathlib import Path
from types import SimpleNamespace as S
import unittest
from unittest.mock import Mock,patch
import numpy as np
from scipy.spatial.transform import Rotation
from task_hook_preview import HOOK,HookPreviewPlanner,arc_reference,hook_rotations
from task_hook_collision import HandBaseCollision
from task_hand_control import model_joints,OPEN,ORDER
from workbench_model import OfflineModel

ROOT=Path(__file__).resolve().parents[1]


class HookTests(unittest.TestCase):
    def test_bilateral_open_thumb_and_other_fingers(self):
        for side in ('left','right'):
            q=model_joints(side,HOOK)
            for i,name in enumerate(ORDER):
                if name!='index_1':self.assertAlmostEqual(q[side+'_'+name+'_joint'],OPEN[i])
            self.assertGreater(q[side+'_index_1_joint'],.8)

    def test_no_hardware_domain(self):
        with patch.dict(os.environ,{'ROS_DOMAIN_ID':'0'}):
            node=Mock()
            with self.assertRaisesRegex(RuntimeError,'offline domain 93'):
                HookPreviewPlanner(node,None,None,preview_urdf='unused',check_preview=Mock())
            node.create_client.assert_not_called()

    def test_downward_search_and_changing_arc_reference(self):
        for quat in hook_rotations():
            z=Rotation.from_quat(quat).as_matrix()[:,2]
            self.assertLessEqual(np.degrees(np.arccos(np.clip(-z[2],-1.,1.))),20.00001)
        nominal=hook_rotations()[0];ref=arc_reference(nominal,np.zeros(3),[.08,0,0])
        endpoint=Rotation.from_euler('z',120,degrees=True).apply([.08,0,0])
        change=(Rotation.from_quat(ref(endpoint))*Rotation.from_quat(nominal).inv()).magnitude()
        self.assertAlmostEqual(np.degrees(change),42.)
        np.testing.assert_allclose(ref([.08,0,0]),nominal)

    def fixture(self,link='left_middle_1'):
        model=Mock();model.transform.return_value=np.eye(4)
        shape={'link':link,'kind':'box','parameters':{'size':'.01 .01 .01'},'xyz':[0,0,0],'rpy':[0,0,0]}
        with patch('task_hook_collision.subprocess.run',return_value=S(stdout=json.dumps([shape]))):
            return HandBaseCollision(model,'unused',[0,0,.02])

    def test_base_is_below_top_and_contains_finger_volume(self):
        q=model_joints('left',HOOK)
        for link in ('left_middle_1','left_thumb_1','left_index_2','left_hand_base_link'):
            checker=self.fixture(link)
            t=np.eye(4);t[2,3]=.01;checker.model.transform.return_value=t
            self.assertEqual(checker.check(q,'left'),link)
            t=np.eye(4);t[2,3]=.031;checker.model.transform.return_value=t
            self.assertIsNone(checker.check(q,'left'))
            self.assertGreater(checker.clearance(q,'left')[0],.005)

    def test_curved_tcp_is_recomputed_bilaterally(self):
        path=ROOT/'data/offline_task_tests/20260909_185831_614140/robot.urdf'
        if not path.is_file():self.skipTest('Archived articulated URDF unavailable')
        model=OfflineModel(path)
        for side in ('left','right'):
            straight=model_joints(side,(1.,)*6);curled=model_joints(side,HOOK)
            origin=np.linalg.inv(model.transform(side+'_wrist_yaw_link',straight))
            a=origin@model.transform(side+'_index_tcp',straight)
            b=origin@model.transform(side+'_index_tcp',curled)
            self.assertGreater(np.linalg.norm(a[:3,3]-b[:3,3]),.08)
            self.assertGreater(np.degrees((Rotation.from_matrix(a[:3,:3]).inv()*Rotation.from_matrix(b[:3,:3])).magnitude()),85.)


if __name__=='__main__':unittest.main()
