import json
from pathlib import Path
import unittest

import numpy as np

from task_hand_control import POSTURES,model_joints
from task_hook_collision import HandBaseCollision,fcl
from workbench_model import OfflineModel

ROOT=Path(__file__).resolve().parents[1]
ARCHIVE=ROOT/'data/offline_task_tests/20260909_231527_214317'


@unittest.skipUnless((ARCHIVE/'robot.urdf').is_file(),'Offline broad-phase fixture unavailable')
class CollisionBroadPhaseTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.model=OfflineModel(ARCHIVE/'robot.urdf')
        cls.q=json.loads((ARCHIVE/'reconstructed_state.json').read_text())['joints']
        top=np.array([0.3829095037005722,0.11581349755573292,-0.05809135541755193])
        cls.collision=HandBaseCollision(cls.model,ARCHIVE/'robot.urdf',top)

    def brute_hand(self,q,side):
        for name,obj in self.collision.objects(q,side):
            if fcl.collide(obj,self.collision.base,fcl.CollisionRequest(),fcl.CollisionResult()):return name
        return None

    def brute_arm_body(self,q,side):
        body=list(self.collision.body_objects(q,self.collision.body_shapes))
        for name,obj in self.collision.body_objects(q,self.collision.arm_by_side[side]):
            for other,obstacle in body:
                if fcl.collide(obj,obstacle,fcl.CollisionRequest(),fcl.CollisionResult()):return name,other
        return None

    def test_conservative_broad_phase_matches_original_fcl_checks(self):
        rng=np.random.default_rng(902)
        for side in ('left','right'):
            base=dict(self.q);base.update(model_joints(side,POSTURES['index_only']))
            names=[name for name in self.model.limits if name.startswith(side+'_') and name in base]
            for _ in range(12):
                q=dict(base);q['waist_yaw_joint']=float(rng.uniform(-.35,.35))
                for name in names:
                    lo,hi=self.model.limits[name]
                    q[name]=float(np.clip(q[name]+rng.uniform(-.12,.12),lo+.03,hi-.03))
                self.assertEqual(self.collision.check(q,side),self.brute_hand(q,side))
                self.assertEqual(self.collision.check_arm_body(q,side),self.brute_arm_body(q,side))

    def test_broad_phase_reduces_narrow_phase_calls(self):
        q=dict(self.q);q.update(model_joints('left',POSTURES['index_only']))
        before=self.collision.narrow_check_count
        for _ in range(20):self.collision.check(q,'left');self.collision.check_arm_body(q,'left')
        used=self.collision.narrow_check_count-before
        brute=20*(len(self.collision.hand_by_side['left'])+
                  len(self.collision.arm_by_side['left'])*len(self.collision.body_shapes))
        self.assertLess(used,brute//2)

    def test_batch_returns_same_first_collision_as_scalar_sequence(self):
        q=dict(self.q);q.update(model_joints('left',POSTURES['index_only']))
        wrist=self.model.transform('left_wrist_yaw_link',q)
        hand=self.collision.hand_local(q,'left',wrist)
        local_center=next(item[3] for item in hand if item[0]=='left_hand_base')
        collision_center=wrist[:3,:3]@local_center+wrist[:3,3]
        old=(self.collision.base,self.collision.base_center,
             self.collision.base_radius,self.collision.base_half_height)
        try:
            self.collision.base=fcl.CollisionObject(fcl.Cylinder(.025,.02),fcl.Transform(collision_center))
            self.collision.base_center=collision_center
            self.collision.base_radius=.025;self.collision.base_half_height=.01
            values=np.asarray([[.5],[-.5],[q['waist_yaw_joint']]])
            scalar=[]
            for value in values[:,0]:
                state=dict(q);state['waist_yaw_joint']=float(value)
                scalar.append(self.collision.check_state(state,'left'))
            collision,_=self.collision.check_batch(q,'left',['waist_yaw_joint'],values)
            self.assertEqual(scalar[:2],[(None,None),(None,None)])
            self.assertEqual(scalar[2],('left_hand_base',None))
            self.assertEqual(collision,{'sample_index':2,'kind':'hand','link':'left_hand_base'})
        finally:
            (self.collision.base,self.collision.base_center,
             self.collision.base_radius,self.collision.base_half_height)=old


if __name__=='__main__':unittest.main()
