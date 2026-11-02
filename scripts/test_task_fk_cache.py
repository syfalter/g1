import concurrent.futures
from pathlib import Path
import unittest
from unittest.mock import patch
import numpy as np
from scipy.spatial.transform import Rotation
from workbench_model import OfflineModel

ROOT=Path(__file__).resolve().parents[1]


def reference(model,link,q):
    if link not in model.parents:return np.eye(4)
    joint,origin=model.parents[link]
    result=reference(model,joint.find('parent').get('link'),q)@origin
    if joint.get('type') in ('revolute','continuous'):
        axis=np.array([float(v) for v in joint.find('axis').get('xyz').split()])
        mimic=joint.find('mimic')
        angle=(q.get(joint.get('name'),0) if mimic is None else
               q.get(mimic.get('joint'),0)*float(mimic.get('multiplier','1'))+float(mimic.get('offset','0')))
        motion=np.eye(4);motion[:3,:3]=Rotation.from_rotvec(axis*angle).as_matrix()
        result=result@motion
    elif joint.get('type')=='prismatic':
        motion=np.eye(4)
        motion[:3,3]=np.array([float(v) for v in joint.find('axis').get('xyz').split()])*q.get(joint.get('name'),0)
        result=result@motion
    return result


class FKCacheTests(unittest.TestCase):
    def setUp(self):
        path=ROOT/'data/offline_task_tests/20260909_210403_699097/robot.urdf'
        if not path.is_file():self.skipTest('Articulated archive model unavailable')
        self.model=OfflineModel(path)
        self.names=[j.get('name') for j in self.model.robot.findall('joint') if j.get('type')!='fixed']

    def test_cached_fk_matches_previous_algorithm_including_mimic(self):
        rng=np.random.default_rng(19)
        for _ in range(12):
            q=dict(zip(self.names,rng.uniform(-.3,.3,len(self.names))))
            for link in self.model.parents:
                np.testing.assert_allclose(self.model.transform(link,q),reference(self.model,link,q),atol=1e-13)

    def test_mutation_and_concurrent_states_do_not_reuse_wrong_cache(self):
        q=dict.fromkeys(self.names,0.)
        result=self.model.transform('left_index_tcp',q);result[:]=0
        np.testing.assert_allclose(self.model.transform('left_index_tcp',q),reference(self.model,'left_index_tcp',q))
        q['left_elbow_joint']=.3
        np.testing.assert_allclose(self.model.transform('left_index_tcp',q),reference(self.model,'left_index_tcp',q))
        def run(value):
            state=dict.fromkeys(self.names,value)
            for _ in range(20):
                np.testing.assert_allclose(self.model.transform('right_index_tcp',state),
                                           reference(self.model,'right_index_tcp',state),atol=1e-13)
        with concurrent.futures.ThreadPoolExecutor(2) as pool:list(pool.map(run,[.1,.2]))

    def test_batched_transforms_match_scalar_and_are_independent(self):
        q=dict.fromkeys(self.names,.07)
        links=['torso_link','left_elbow_link','left_index_tcp','right_wrist_yaw_link']
        batch=self.model.transforms(links,q)
        for link in links:np.testing.assert_allclose(batch[link],reference(self.model,link,q),atol=1e-13)
        batch[links[0]][:]=0
        np.testing.assert_allclose(self.model.transforms([links[0]],q)[links[0]],reference(self.model,links[0],q))

    def test_trajectory_batch_fk_matches_each_scalar_state(self):
        rng=np.random.default_rng(77);links=['torso_link','left_elbow_link','left_index_tcp']
        moving=[name for name in self.names if name in self.model.limits][:12]
        positions=rng.uniform(-.2,.2,(9,len(moving)));base=dict.fromkeys(self.names,.03)
        batch=self.model.batch_transforms(links,base,moving,positions)
        for index,row in enumerate(positions):
            q=dict(base);q.update(zip(moving,row))
            for link in links:np.testing.assert_allclose(batch[link][index],reference(self.model,link,q),atol=1e-13)

    def test_pose_jacobian_matches_finite_differences_with_offset_and_mimic(self):
        rng=np.random.default_rng(91)
        names=['waist_yaw_joint','left_shoulder_pitch_joint','left_shoulder_roll_joint',
               'left_shoulder_yaw_joint','left_elbow_joint','left_wrist_roll_joint',
               'left_wrist_pitch_joint','left_wrist_yaw_joint','left_index_1_joint','right_elbow_joint']
        offset=np.eye(4);offset[:3,3]=[.021,-.012,.044]
        offset[:3,:3]=Rotation.from_euler('xyz',[.2,-.3,.1]).as_matrix()
        for link in ('left_wrist_yaw_link','left_index_tcp'):
            q=dict(zip(self.names,rng.uniform(-.2,.2,len(self.names))))
            pose,jacobian=self.model.pose_jacobian(link,q,names,offset)
            np.testing.assert_allclose(pose,self.model.transform(link,q)@offset,atol=1e-13)
            for index,name in enumerate(names):
                plus=dict(q);minus=dict(q);plus[name]+=1e-6;minus[name]-=1e-6
                a=self.model.transform(link,plus)@offset;b=self.model.transform(link,minus)@offset
                numeric=np.r_[(a[:3,3]-b[:3,3])/2e-6,
                              Rotation.from_matrix(a[:3,:3]@b[:3,:3].T).as_rotvec()/2e-6]
                np.testing.assert_allclose(jacobian[:,index],numeric,rtol=1e-6,atol=1e-8)

    def test_bounded_ik_is_repeatable_and_preserves_passive_state(self):
        from task_ik_search import bounded_pose_ik
        names=['waist_yaw_joint']+[f'left_{suffix}_joint' for suffix in (
            'shoulder_pitch','shoulder_roll','shoulder_yaw','elbow','wrist_roll','wrist_pitch','wrist_yaw')]
        limits={name:self.model.limits[name] for name in names}
        q=dict.fromkeys(self.names,.01);q.update({name:sum(limits[name])/2 for name in names})
        original=dict(q);desired=dict(q);desired.update({name:q[name]+.03 for name in names})
        offset=np.eye(4);offset[:3,3]=[.02,0,.15]
        target=self.model.transform('left_wrist_yaw_link',desired)@offset
        answers=[]
        for _ in range(2):
            answer,record=bounded_pose_ik(self.model,'left_wrist_yaw_link',names,limits,q,target,offset,lambda:None,1.)
            self.assertIsNotNone(answer);self.assertFalse(record['randomized'])
            self.assertLessEqual(record['position_error_m'],.0002)
            self.assertLessEqual(record['orientation_error_rad'],.002)
            for name,(lo,hi) in limits.items():self.assertTrue(lo+.021<=answer[name]<=hi-.021)
            self.assertTrue(all(answer[name]==q[name] for name in q if name not in names))
            answers.append(answer)
        self.assertEqual(q,original)
        np.testing.assert_allclose([answers[0][name] for name in names],[answers[1][name] for name in names],atol=1e-12,rtol=0)
        with self.assertRaisesRegex(RuntimeError,'cancelled'):
            bounded_pose_ik(self.model,'left_wrist_yaw_link',names,limits,q,target,offset,
                            lambda:(_ for _ in ()).throw(RuntimeError('cancelled')),1.)
        with patch('task_ik_search.time.monotonic',side_effect=[0.,1.,1.]):
            answer,record=bounded_pose_ik(self.model,'left_wrist_yaw_link',names,limits,q,target,offset,lambda:None,.1)
        self.assertIsNone(answer);self.assertEqual(record['code'],'LOCAL_IK_BUDGET')
        unreachable=target.copy();unreachable[0,3]+=10.
        answer,record=bounded_pose_ik(self.model,'left_wrist_yaw_link',names,limits,q,unreachable,offset,lambda:None,1.)
        self.assertIsNone(answer);self.assertGreater(record['position_error_m'],.0002)

    def test_task_ik_uses_previous_state_bounds_on_articulated_model(self):
        from task_ik_search import bounded_pose_ik
        names=['waist_yaw_joint']+[f'left_{suffix}_joint' for suffix in (
            'shoulder_pitch','shoulder_roll','shoulder_yaw','elbow','wrist_roll','wrist_pitch','wrist_yaw')]
        limits={name:self.model.limits[name] for name in names}
        previous=dict.fromkeys(self.names,.01)
        previous.update({name:sum(limits[name])/2 for name in names})
        desired=dict(previous);desired.update({name:previous[name]+.03 for name in names})
        seed=dict(previous);seed.update({name:previous[name]+.08 for name in names})
        original=dict(seed);offset=np.eye(4);offset[:3,3]=[.02,0,.15]
        target=self.model.transform('left_wrist_yaw_link',desired)@offset
        answer,record=bounded_pose_ik(self.model,'left_wrist_yaw_link',names,limits,seed,
            target,offset,lambda:None,1.,continuity_reference=previous,
            position_tolerance=.002,orientation_tolerance=np.deg2rad(2.))
        self.assertIsNotNone(answer)
        self.assertLessEqual(record['position_error_m'],.002)
        self.assertLessEqual(record['orientation_error_rad'],np.deg2rad(2.))
        for name,(lower,upper) in limits.items():
            self.assertLessEqual(abs(answer[name]-previous[name]),.119+1e-12)
            self.assertTrue(lower+.021<=answer[name]<=upper-.021)
        self.assertTrue(all(answer[name]==previous[name] for name in previous if name not in names))
        self.assertEqual(seed,original)


if __name__=='__main__':unittest.main()
