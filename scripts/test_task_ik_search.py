import math
import unittest
import numpy as np
from scipy.spatial.transform import Rotation
from task_ik_search import (solution_score,preinsert_orientations,broad_preinsert_orientations,
                            preinsert_seeds,preinsert_jobs,diverse_preinsert_batch)
from task_session import ARMS,ARM_NAMES
from workbench_orientation import orientation_policy,orientation_allowed,orientation_error


class TaskToleranceIKTests(unittest.TestCase):
    class LinearModel:
        def transform(self,link,state):
            pose=np.eye(4);pose[0,3]=state['axis'];return pose

        def pose_jacobian(self,link,state,names,offset):
            jacobian=np.zeros((6,len(names)));jacobian[0,0]=1.
            return self.transform(link,state)@offset,jacobian

    def solve(self,position,*,seed=0.,reference=None,**kwargs):
        from task_ik_search import bounded_pose_ik
        target=np.eye(4);target[0,3]=position
        return bounded_pose_ik(self.LinearModel(),'tip',['axis'],{'axis':(-1.,1.)},
            {'axis':seed,'passive':.7},target,np.eye(4),lambda:None,1.,
            continuity_reference=reference,**kwargs)

    def test_task_candidate_uses_actual_previous_state_not_perturbed_seed(self):
        exact,_=self.solve(.1205)
        self.assertGreater(exact['axis'],.12)
        reference={'axis':0.}
        candidate,record=self.solve(.1205,seed=.08,reference=reference,
                                    position_tolerance=.002,orientation_tolerance=math.radians(2.))
        self.assertIsNotNone(candidate)
        self.assertLessEqual(candidate['axis'],.119)
        self.assertGreater(record['position_error_m'],.0002)
        self.assertLessEqual(record['position_error_m'],.002)
        self.assertEqual(candidate['passive'],.7)
        self.assertEqual(reference,{'axis':0.})

    def test_strict_candidate_and_outside_task_tube_still_fail(self):
        candidate,record=self.solve(.1205,reference={'axis':0.})
        self.assertIsNone(candidate)
        self.assertLessEqual(record['candidate_joints']['axis'],.119)
        self.assertEqual(record['code'],'LOCAL_IK_FAILED')
        candidate,record=self.solve(.123,reference={'axis':0.},position_tolerance=.002)
        self.assertIsNone(candidate)
        self.assertGreater(record['position_error_m'],.003)

    def test_invalid_candidate_threshold_or_reference_fails_closed(self):
        for kwargs in ({'position_tolerance':.003},{'orientation_tolerance':.1},
                       {'position_tolerance':float('nan')},{'reference':{'axis':float('nan')}},
                       {'reference':{'axis':2.}}):
            with self.subTest(kwargs=kwargs),self.assertRaises(ValueError):self.solve(.1,**kwargs)


class SearchCoverageTests(unittest.TestCase):
    def test_broad_search_covers_cad_nominal_tilts_and_twists(self):
        nominal=Rotation.from_euler('x',160,degrees=True).as_quat()
        candidates=broad_preinsert_orientations(nominal)
        self.assertEqual(len(candidates),160)
        tilts=set()
        for quat in candidates:
            matrix=Rotation.from_quat(quat).as_matrix()
            np.testing.assert_allclose(matrix.T@matrix,np.eye(3),atol=1e-12)
            tilts.add(round(np.degrees(np.arccos(np.clip(-matrix[2,2],-1,1)))))
            direction=matrix[:,2]
            entry=np.array([.4,.1,.2])
            pre=entry-direction*(.1/-direction[2])
            self.assertAlmostEqual(pre[2]-entry[2],.1)
            np.testing.assert_allclose(np.cross(entry-pre,direction),0,atol=1e-12)
        self.assertEqual(tilts,{0,20,35,50,65})
        for index in range(0,160,4):
            np.testing.assert_allclose(Rotation.from_quat(candidates[index:index+4]).as_matrix()[:,:,2],
                                      np.tile(Rotation.from_quat(candidates[index]).as_matrix()[:,2],(4,1)),atol=1e-12)

    def test_broad_search_accepts_vertical_nominal(self):
        self.assertEqual(len(broad_preinsert_orientations([1.,0.,0.,0.])),160)

    def test_combined_diagonal_tilt_and_twist_stay_in_existing_bounds(self):
        policy=orientation_policy({});nominal=Rotation.from_euler('xyz',[.2,-.1,.7]).as_quat()
        candidates=preinsert_orientations(nominal,policy)
        self.assertEqual(len(candidates),91)
        for quat in candidates:self.assertTrue(orientation_allowed(nominal,quat,policy,epsilon=1e-7))
        self.assertTrue(any(tilt>4.9 and roll>9.9 for tilt,roll in
                            (orientation_error(nominal,q) for q in candidates)))
        relative=[(Rotation.from_quat(nominal).inv()*Rotation.from_quat(q)).as_rotvec() for q in candidates]
        self.assertTrue(any(abs(r[0])>.02 and abs(r[1])>.02 and abs(r[2])>.05 for r in relative))

    def test_previous_seven_orientations_are_preserved(self):
        policy=orientation_policy({});nominal=np.array([0.,0.,0.,1.])
        result=preinsert_orientations(nominal,policy)
        self.assertAlmostEqual(abs(result[0]@nominal),1.)
        for axis,limit in ((0,5.),(1,5.),(2,10.)):
            for sign in (-1,1):
                v=np.zeros(3);v[axis]=math.radians(sign*limit*.9)
                expected=Rotation.from_rotvec(v).as_quat()
                self.assertTrue(any(abs(q@expected)>1-1e-12 for q in result[:7]))

    def test_seed_ownership_limits_and_determinism_for_both_hands(self):
        q=dict.fromkeys(ARM_NAMES,.1)
        for side,other in (('left','right'),('right','left')):
            limits={n:(-1.,1.) for n in ARMS[side]}
            seeds=preinsert_seeds(side,q,limits)
            self.assertEqual(len(seeds),5)
            self.assertEqual(seeds,preinsert_seeds(side,q,limits))
            for seed in seeds:
                for name in ARMS[other]:self.assertEqual(seed[name],q[name])
                for name in ARMS[side]:self.assertTrue(-.97<=seed[name]<=.97)

    def test_schedule_covers_angles_early_without_duplicate_requests(self):
        jobs=list(preinsert_jobs(91,5))
        self.assertEqual(len(jobs),91*3*5)
        self.assertEqual(len(set(jobs)),len(jobs))
        self.assertEqual(len({o for o,h,s in jobs[:91]}),91)
        self.assertEqual({h for o,h,s in jobs[:91]},{0,1,2})
        self.assertEqual({s for o,h,s in jobs[:91]},set(range(5)))
        self.assertTrue({(o,h,s) for o in range(7) for h in range(3) for s in range(3)}.issubset(set(jobs[:154])))

    def test_diverse_schedule_preserves_first_batch_and_entire_grid(self):
        for angles,seeds,batch in ((1,1,32),(24,7,32),(91,5,16),(24,6,32),(8,3,8)):
            with self.subTest(angles=angles,seeds=seeds,batch=batch):
                original=list(preinsert_jobs(angles,seeds))
                changed=list(preinsert_jobs(angles,seeds,policy='diverse',initial_trials=batch))
                self.assertEqual(changed[:batch],original[:batch])
                self.assertEqual(len(changed),angles*seeds*3)
                self.assertEqual(set(changed),set(original))
                self.assertEqual(changed,list(preinsert_jobs(angles,seeds,policy='diverse',initial_trials=batch)))

    def test_diverse_schedule_interleaves_all_seeds_before_seed_major_tail(self):
        jobs=list(preinsert_jobs(24,7,policy='diverse'))
        prefix=jobs[:32+24*6]
        for orientation in range(24):
            self.assertEqual({s for o,h,s in prefix if o==orientation},set(range(7)))
        self.assertLess(jobs.index((18,2,2)),list(preinsert_jobs(24,7)).index((18,2,2)))

    def test_unknown_search_policy_fails_closed(self):
        with self.assertRaises(ValueError):list(preinsert_jobs(24,7,policy='other'))


class DiverseCandidateTests(unittest.TestCase):
    def candidate(self,index,orientation,position,score):
        return dict(trial_index=index,orientation_index=orientation,goal={'q':position},score={'total':score})

    def test_first_batch_order_is_unchanged(self):
        rows=[self.candidate(i,i%2,i*.1,4-i) for i in range(4)]
        self.assertEqual(diverse_preinsert_batch(rows,[],['q'],2),rows[:2])

    def test_untried_orientation_precedes_far_state_in_repeated_orientation(self):
        old=[self.candidate(0,7,0.,0.)]
        rows=[self.candidate(1,7,1.,1.),self.candidate(2,18,.1,100.)]
        selected=diverse_preinsert_batch(rows,old,['q'],1)
        self.assertEqual(selected,[rows[1]])
        self.assertEqual(len(rows),2);self.assertEqual(len(old),1)

    def test_same_orientation_uses_max_min_joint_distance_then_score(self):
        old=[self.candidate(0,7,0.,0.),self.candidate(1,7,1.,0.)]
        rows=[self.candidate(2,7,.01,1.),self.candidate(3,7,.5,10.),self.candidate(4,7,.5,9.)]
        self.assertEqual(diverse_preinsert_batch(rows,old,['q'],2),[rows[2],rows[0]])

    def test_batch_updates_orientation_counts_and_keeps_all_candidates_available(self):
        old=[self.candidate(0,7,0.,0.),self.candidate(1,15,0.,0.)]
        rows=[self.candidate(2,7,.9,1.),self.candidate(3,7,.5,2.),self.candidate(4,15,.1,3.)]
        self.assertEqual(diverse_preinsert_batch(rows,old,['q'],4),[rows[0],rows[2],rows[1]])


class OriginalScoreTests(unittest.TestCase):
    def test_original_weighted_objective(self):
        names=ARMS['left'];q=dict.fromkeys(names,0.);answer=dict.fromkeys(names,.1)
        limits={n:(-1.,1.) for n in names};limits[names[0]]=(-1.,.2)
        actual=Rotation.from_rotvec([0,0,.05]).as_quat()
        score=solution_score(names,limits,q,answer,[0,0,0,1],actual)
        expected=7*.1**2+.5*(3*.1**2)+2*(.15-.1)**2+.2*.05**2
        self.assertAlmostEqual(score['total'],expected)

    def test_guarded_margin_and_nonfinite_candidates_are_rejected(self):
        names=ARMS['left'];q=dict.fromkeys(names,0.);limits={n:(-1.,1.) for n in names}
        for value in (.99,float('nan'),float('inf')):
            answer=dict(q);answer[names[0]]=value
            self.assertIsNone(solution_score(names,limits,q,answer,[0,0,0,1],[0,0,0,1]))


if __name__=='__main__':unittest.main()
