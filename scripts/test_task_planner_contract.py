"""Unit checks for accepting IK only after scoped collision validation."""
import json
from pathlib import Path
from types import SimpleNamespace as S
import unittest
from unittest.mock import Mock
from unittest.mock import patch
import xml.etree.ElementTree as ET
import numpy as np
from task_motion_planner import TaskPlanner, PlanningFailure
from task_session import ARMS, ARM_NAMES, Ticket
from scipy.spatial.transform import Rotation
from workbench_orientation import orientation_policy
from task_straight_policy import StraightIndexPolicy,arc_reference


class ContactReferenceTests(unittest.TestCase):
    def test_recorded_insert_failures_use_the_wrong_arc_reference(self):
        directory=Path(__file__).resolve().parents[1]/'data/task_sessions/20260911_113650_819948/plan_0006'
        if not (directory/'planning_failure.json').exists():self.skipTest('Archive unavailable')
        failure=json.loads((directory/'planning_failure.json').read_text())
        geometry=json.loads((directory/'geometry.json').read_text())
        diagnostics=failure['search_diagnostics']
        frozen=np.asarray(diagnostics['capture_frame']['pelvis_from_torso_row_major'])
        def transform(key):return frozen[:3,:3]@geometry['task_preview'][key]+frozen[:3,3]
        center=transform('axis_center_m');entry=transform('insertion_reference_m')
        planner=TaskPlanner.__new__(TaskPlanner)
        errors=[f for f in failure['diagnostic']['failures'] if f['code']=='RETIMED_TCP_BOUNDS']
        self.assertEqual(len(errors),3)
        for error in errors:
            nominal=diagnostics['preinsert']['trials'][error['trial_index']]['quaternion_xyzw']
            old=arc_reference(nominal,center,entry)
            np.testing.assert_allclose(old(error['tcp_position_m']),error['reference_quaternion_xyzw'],atol=1e-12)
            tip=np.eye(4);tip[:3,3]=error['tcp_position_m']
            tip[:3,:3]=Rotation.from_quat(error['actual_quaternion_xyzw']).as_matrix()
            # Isolate the recorded orientation rejection, not an IK/path replay.
            curve=lambda point:error['position_error_m']
            report={'checked_samples':0,'max_position_error_m':0.,'max_tilt_deg':0.,'max_twist_deg':0.}
            old_error=planner.check_tcp_geometry_batch(np.array([tip]),old,curve,orientation_policy({}),
                                                       'insert','Insert',dict(report))
            self.assertIsNotNone(old_error)
            self.assertAlmostEqual(old_error.diagnostic['tilt_deg'],error['tilt_deg'])
            new_error=planner.check_tcp_geometry_batch(np.array([tip]),nominal,curve,orientation_policy({}),
                                                       'insert','Insert',report)
            self.assertIsNone(new_error)
            self.assertLess(report['max_tilt_deg'],.001)

    def test_raised_arc_and_stage_specific_orientation(self):
        class Planner(StraightIndexPolicy,TaskPlanner):pass
        planner=Planner.__new__(Planner)
        center=np.array([.3,.1,-.05]);entry=center+[.05,0,.010]
        nominal=Rotation.from_euler('y',145,degrees=True).as_quat()
        pre=np.eye(4);pre[:3,:3]=Rotation.from_quat(nominal).as_matrix()
        pre[:3,3]=entry+[-.03,.08,.08]
        planner.cartesian=Mock(return_value={'end':{}})
        planner.tip=Mock(return_value=pre)
        planner.contact_segments('left',{},pre,entry,center,nominal,np.eye(4),object())
        calls=planner.cartesian.call_args_list
        self.assertEqual(len(calls),3)
        for call in (calls[0],calls[-1]):
            points=np.asarray(call.args[2]);reference=call.args[3];curve=call.args[7]
            self.assertEqual(len(points),59)
            self.assertFalse(callable(reference))
            np.testing.assert_allclose(reference,nominal)
            np.testing.assert_allclose(curve.batch(points),0.,atol=1e-12)
        for call,end_angle in zip(calls[1:2],(120,)):
            points=np.asarray(call.args[2]);reference=call.args[3];curve=call.args[7]
            self.assertEqual(len(points),120)
            np.testing.assert_allclose(points[:,2],entry[2],atol=1e-12)
            np.testing.assert_allclose(np.linalg.norm(points[:,:2]-center[:2],axis=1),.05,atol=1e-12)
            np.testing.assert_allclose([curve(p) for p in points],0.,atol=1e-12)
            np.testing.assert_allclose(curve.batch(points),0.,atol=1e-12)
            lowered=points-[0,0,.010]
            np.testing.assert_allclose(curve.batch(lowered),.010,atol=1e-12)
            self.assertAlmostEqual(curve(lowered[0]),.010)
            np.testing.assert_allclose(reference,nominal)
        # The old arc reference at a slanted preinsert pose rejected the start itself.
        old=arc_reference(nominal,center,entry)
        report={'checked_samples':0,'max_position_error_m':0.,'max_tilt_deg':0.,'max_twist_deg':0.}
        planner.tip=Mock(return_value=pre)
        planner.dorsal_in_tip=Rotation.from_quat(nominal).inv().apply([1.,0.,0.])
        planner.fixed_forward_yaw=0.
        with self.assertRaisesRegex(PlanningFailure,'RETIMED_TCP_BOUNDS'):
            planner.check_tcp_geometry('left',{},np.eye(4),old,calls[0].args[7],
                                       orientation_policy({}),'insert','Insert',dict(report))
        planner.check_tcp_geometry('left',{},np.eye(4),calls[0].args[3],calls[0].args[7],
                                   orientation_policy({}),'insert','Insert',dict(report))
        error=planner.check_tcp_geometry_batch(np.array([pre]),calls[0].args[3],calls[0].args[7],
                                              orientation_policy({}),'insert','Insert',dict(report))
        self.assertIsNone(error)


class PlannerContractTests(unittest.TestCase):
    def setUp(self):
        self.p=TaskPlanner.__new__(TaskPlanner)
        self.p.ik=object();self.p.valid=object()
        self.p.snapshot_id='task_measured_rgbd_snapshot_test'
        self.p.target_id='task_ring_contact_sweep_test'
        self.q=dict.fromkeys(ARM_NAMES,.1)
        self.ticket=Ticket(1,'task','left',.15,.15)
        self.p.search_seconds=40.;self.p.search_attempts=400;self.p.search_diagnostics={}
        self.p.progress=Mock()

    def result(self,code=1):
        return S(error_code=S(val=code),solution=S(joint_state=S(name=list(ARMS['left']),position=[.2]*7)))

    def test_collision_candidate_never_accepted(self):
        self.p.call=Mock(return_value=self.result())
        self.p.validate=Mock(side_effect=PlanningFailure('COLLISION','insert','table contact',contacts=[]))
        result=self.p.solve('left',self.q,np.eye(4),np.eye(4),self.ticket,'insert')
        self.assertIsNone(result)
        self.assertEqual(self.p.last_ik['code'],'COLLISION')
        self.p.validate.assert_called_once()

    def test_accepted_ik_preserves_other_hand_and_uses_stage(self):
        self.p.call=Mock(return_value=self.result());self.p.validate=Mock()
        result=self.p.solve('left',self.q,np.eye(4),np.eye(4),self.ticket,'rotate')
        self.assertEqual([result[n] for n in ARMS['right']],[.1]*7)
        self.p.validate.assert_called_once_with(result,self.ticket,'left','rotate')

    def test_kinematic_failure_is_not_reported_as_collision(self):
        self.p.call=Mock(return_value=self.result(-31));self.p.validate=Mock()
        self.assertIsNone(self.p.solve('left',self.q,np.eye(4),np.eye(4),self.ticket))
        self.assertEqual(self.p.last_ik['code'],'IK_FAILED');self.p.validate.assert_not_called()

    def test_broad_free_space_ik_searches_collision_free_branches(self):
        self.p.orientation_search='broad'
        self.p.call=Mock(return_value=self.result());self.p.validate=Mock()
        for stage,expected in (('approach',True),('insert',False),('rotate',False),('withdraw',False)):
            self.p.solve('left',self.q,np.eye(4),np.eye(4),self.ticket,stage)
            self.assertEqual(self.p.call.call_args.args[1].ik_request.avoid_collisions,expected)
        self.assertEqual(self.p.validate.call_count,4)

    def test_model_mismatch_is_not_retried_as_ik(self):
        self.p.call=Mock(return_value=self.result())
        self.p.validate=Mock(side_effect=PlanningFailure('MODEL_MISMATCH','insert','old hand'))
        with self.assertRaisesRegex(PlanningFailure,'MODEL_MISMATCH'):
            self.p.solve('left',self.q,np.eye(4),np.eye(4),self.ticket,'insert')

    def test_known_retimer_failures_are_candidate_failures(self):
        from moveit_msgs.msg import RobotTrajectory
        self.p.timing=object()
        for message in ('MoveIt TOTG failed','MoveIt task Ruckig smoothing failed'):
            self.p.call=Mock(return_value=S(success=False,message=message))
            with self.assertRaises(PlanningFailure) as caught:
                self.p.retime('left',self.q,RobotTrajectory(),self.ticket,'Insert',stage='insert')
            self.assertEqual(caught.exception.diagnostic['code'],'MOTION_PLAN_FAILED')
            self.assertEqual(caught.exception.diagnostic['stage'],'insert')
            self.assertEqual(caught.exception.diagnostic['retiming_error'],message)

    def test_retimer_contract_errors_and_cancellation_are_not_reclassified(self):
        from moveit_msgs.msg import RobotTrajectory
        self.p.timing=object()
        self.p.call=Mock(return_value=S(success=False,message='invalid side/scaling'))
        with self.assertRaisesRegex(RuntimeError,'invalid side/scaling') as caught:
            self.p.retime('left',self.q,RobotTrajectory(),self.ticket,'Insert',stage='insert')
        self.assertNotIsInstance(caught.exception,PlanningFailure)
        for error in (TimeoutError('deadline'),RuntimeError('STOP')):
            self.p.call=Mock(side_effect=error)
            with self.assertRaises(type(error)) as caught:
                self.p.retime('left',self.q,RobotTrajectory(),self.ticket,'Insert',stage='insert')
            self.assertIs(caught.exception,error)

    def test_validator_request_carries_exact_snapshot_stage_and_side(self):
        self.p.call=Mock(return_value=S(valid=True,code='VALID'))
        self.p.validate(self.q,self.ticket,'right','withdraw')
        request=self.p.call.call_args.args[1]
        self.assertEqual((request.arm,request.stage),('right','withdraw'))
        self.assertEqual(request.snapshot_object_id,self.p.snapshot_id)
        self.assertEqual(request.target_object_id,self.p.target_id)

    def test_preinsert_search_is_bounded_and_keeps_entry_unchanged(self):
        entry=np.array([.4,.1,0.]);pre=np.eye(4);pre[:3,3]=entry+[-.03,0.,.1]
        geometry={'geometry_valid_for_planning_seed':True,
            'task_preview':{'insertion_reference_m':entry.tolist(),'axis_center_m':[.3,.1,0.]},
            'grasp_candidates':{'left':{'poses':{'preinsert_index_tcp':{'matrix_row_major':pre.tolist()}}}},
            'fingertip_estimates':{'sides':{'left':{'states':{'open':{'wrist_yaw_from_index_tcp':
                {'matrix_row_major':np.eye(4).tolist()}}}}}}}
        self.p.model=S(robot=Mock())
        self.p.model.robot.find.return_value=ET.fromstring('<limit lower="-2" upper="2"/>')
        self.p.progress=Mock();self.p.solve=Mock(return_value=None);self.p.joint_path=Mock()
        self.p.tip=Mock(return_value=np.eye(4))
        self.p.last_ik={'code':'IK_FAILED','moveit_error_code':-31}
        with patch('task_motion_planner.require_task_hand_model'):
            with self.assertRaisesRegex(PlanningFailure,'No valid candidate in 400 trials'):
                self.p.ring('left',self.q,geometry,self.ticket)
        self.assertEqual(self.p.solve.call_count,400)
        heights={round(call.args[2][2,3],3) for call in self.p.solve.call_args_list}
        self.assertEqual(heights,{.10,.08,.06})
        np.testing.assert_equal(geometry['task_preview']['insertion_reference_m'],entry)
        self.p.joint_path.assert_not_called()
        self.assertGreater(self.p.search_diagnostics['preinsert']['covered_orientations'],80)
        self.assertEqual(self.p.search_diagnostics['preinsert']['stop_reason'],'attempt_budget')

    def configure_ranking(self):
        self.p.joint_limits=Mock(return_value={n:(-2.,2.) for n in ARMS['left']})
        self.p.progress=Mock();self.p.retime=Mock(return_value={'end':dict(self.q)})
        self.p.last_ik={'code':'VALID'}
        self.p.tip=Mock(return_value=np.eye(4))

    def test_cartesian_chooses_lower_score_not_first_valid(self):
        self.configure_ranking()
        first=dict(self.q);second=dict(self.q)
        first.update(dict.fromkeys(ARMS['left'],.16));second.update(dict.fromkeys(ARMS['left'],.12))
        self.p.solve=Mock(side_effect=[first,second])
        with patch('task_motion_planner.orientation_candidates',return_value=[[0,0,0,1]]*2):
            self.p.cartesian('left',self.q,[[0,0,0]],[0,0,0,1],np.eye(4),self.ticket,'Insert',lambda p:0.,'insert')
        trajectory=self.p.retime.call_args.args[2].joint_trajectory
        np.testing.assert_allclose(trajectory.points[-1].positions,[.12]*7)
        self.assertEqual(self.p.solve.call_count,2)

    def test_original_small_motion_fast_path_is_retained(self):
        self.configure_ranking()
        answer=dict(self.q);answer.update(dict.fromkeys(ARMS['left'],.11))
        self.p.solve=Mock(return_value=answer)
        self.p.cartesian('left',self.q,[[0,0,0]],[0,0,0,1],np.eye(4),self.ticket,'Insert',lambda p:0.,'insert')
        self.assertEqual(self.p.solve.call_count,1)
        self.assertTrue(self.p.search_diagnostics['cartesian'][0]['points'][0]['fast_path'])

    def test_rejected_candidate_records_actual_previous_state_without_aliasing(self):
        self.configure_ranking()
        answer=dict(self.q);answer[ARMS['left'][0]]=.25
        self.p.solve=Mock(return_value=answer)
        with patch('task_motion_planner.orientation_candidates',return_value=[[0,0,0,1]]):
            with self.assertRaisesRegex(PlanningFailure,'CONTINUOUS_IK_FAILED'):
                self.p.cartesian('left',self.q,[[0,0,0]],[0,0,0,1],np.eye(4),self.ticket,
                                 'Insert',lambda p:0.,'insert')
        example=self.p.search_diagnostics['cartesian'][0]['rejection_examples']['JOINT_CONTINUITY']
        inputs=example['candidate_input'];name=ARMS['left'][0]
        self.assertEqual(inputs['previous_state'][name],.1)
        self.assertEqual(inputs['solved_state'][name],.25)
        self.assertEqual(inputs['point_index'],0)
        np.testing.assert_array_equal(inputs['target_tcp_matrix'],np.eye(4))
        answer[name]=1.;self.q[name]=1.
        self.assertEqual(inputs['previous_state'][name],.1)
        self.assertEqual(inputs['solved_state'][name],.25)
        self.p.retime.assert_not_called()

    def test_constrained_cartesian_uses_one_task_candidate_and_keeps_outer_gates(self):
        self.configure_ranking();self.p.ik_backend='constrained'
        answer=dict(self.q);answer.update(dict.fromkeys(ARMS['left'],.11))
        self.p.solve_constrained_contact=Mock(return_value=answer)
        self.p.solve=Mock()
        self.p.cartesian('left',self.q,[[0,0,0]],[0,0,0,1],np.eye(4),self.ticket,
                         'Insert',lambda p:0.,'insert')
        self.p.solve_constrained_contact.assert_called_once()
        self.p.solve.assert_not_called()
        self.assertEqual(self.p.solve_constrained_contact.call_args.args[2],self.q)
        answer[ARMS['left'][0]]=.3
        with self.assertRaisesRegex(PlanningFailure,'CONTINUOUS_IK_FAILED'):
            self.p.cartesian('left',self.q,[[0,0,0]],[0,0,0,1],np.eye(4),self.ticket,
                             'Insert',lambda p:0.,'insert')
        self.assertEqual(self.p.last_ik['code'],'JOINT_CONTINUITY')

    def test_local_ik_repair_reuses_prefix_and_rebuilds_suffix(self):
        self.configure_ranking()
        states=[dict(self.q,**dict.fromkeys(ARMS['left'],value)) for value in (.16,.17,.18)]
        self.p.solve=Mock(side_effect=[states[0],states[1],None,None,states[2]])
        with patch('task_motion_planner.orientation_candidates',return_value=[[0,0,0,1]]*2):
            self.p.cartesian('left',self.q,[[0,0,0]]*2,[0,0,0,1],np.eye(4),self.ticket,
                             'Insert',lambda p:0.,'insert')
        path=self.p.retime.call_args.args[2].joint_trajectory
        np.testing.assert_allclose([p.positions[0] for p in path.points],[.1,.17,.18])
        self.assertEqual(self.p.search_diagnostics['cartesian'][0]['local_repairs'][0]['restart_point'],1)

    def test_completed_segment_resumes_saved_branch_after_downstream_failure(self):
        self.configure_ranking()
        states=[dict(self.q,**dict.fromkeys(ARMS['left'],value)) for value in (.16,.17)]
        self.p.solve=Mock(side_effect=states)
        self.p.contact_seeds=Mock(return_value=[])
        with patch('task_motion_planner.orientation_candidates',return_value=[[0,0,0,1]]*2):
            paths=self.p.cartesian_paths('left',self.q,[[0,0,0]],[0,0,0,1],np.eye(4),self.ticket,
                                         'Insert',lambda p:0.,'insert',branching=True)
            try:
                next(paths)
                paths.send(PlanningFailure('CONTINUOUS_IK_FAILED','withdraw','blocked'))
            finally:paths.close()
        self.assertEqual(self.p.solve.call_count,2)
        self.assertEqual(self.p.retime.call_count,2)
        repair=self.p.search_diagnostics['cartesian'][0]['local_repairs'][0]
        self.assertTrue(repair['downstream'])
        self.assertEqual(repair['reason']['stage'],'withdraw')
        self.assertAlmostEqual(self.p.retime.call_args.args[2].joint_trajectory.points[-1].positions[0],.17)

    def test_local_retimed_repair_revalidates_complete_segment(self):
        self.configure_ranking()
        states=[dict(self.q,**dict.fromkeys(ARMS['left'],value)) for value in (.16,.17)]
        self.p.solve=Mock(side_effect=states)
        self.p.retime.side_effect=[PlanningFailure('RETIMED_TCP_BOUNDS','insert','tilt',tcp_position_m=[0,0,0]),{'end':dict(self.q)}]
        with patch('task_motion_planner.orientation_candidates',return_value=[[0,0,0,1]]*2):
            result=self.p.cartesian('left',self.q,[[0,0,0]],[0,0,0,1],np.eye(4),self.ticket,
                                    'Insert',lambda p:0.,'insert')
        self.assertEqual(result['end'],dict(self.q));self.assertEqual(self.p.retime.call_count,2)
        self.assertEqual(result['geometry_validation']['stage'],'insert')
        self.assertLessEqual(result['geometry_validation']['max_position_error_m'],.003)
        self.assertEqual(self.p.solve.call_count,2)
        self.assertEqual(self.p.search_diagnostics['cartesian'][0]['local_repairs'][0]['reselected_point'],0)

    def test_local_repair_does_not_swallow_cancellation(self):
        self.configure_ranking()
        self.p.solve=Mock(side_effect=RuntimeError('cancelled'))
        with self.assertRaisesRegex(RuntimeError,'cancelled'):
            self.p.cartesian('left',self.q,[[0,0,0]],[0,0,0,1],np.eye(4),self.ticket,
                             'Insert',lambda p:0.,'insert')
        self.p.retime.assert_not_called()

    def test_near_limit_first_solution_does_not_take_fast_path(self):
        self.configure_ranking()
        name=ARMS['left'][0]
        q=dict(self.q);q[name]=.88
        first=dict(q);first[name]=.90
        second=dict(q);second[name]=.85
        self.p.joint_limits.return_value={n:(-2.,1.) for n in ARMS['left']}
        self.p.solve=Mock(side_effect=[first,second])
        with patch('task_motion_planner.orientation_candidates',return_value=[[0,0,0,1]]*2):
            self.p.cartesian('left',q,[[0,0,0]],[0,0,0,1],np.eye(4),self.ticket,'Insert',lambda p:0.,'insert')
        self.assertEqual(self.p.solve.call_count,2)
        trajectory=self.p.retime.call_args.args[2].joint_trajectory
        self.assertAlmostEqual(trajectory.points[-1].positions[0],.85)

    def test_initial_ranking_keeps_searching_after_first_valid(self):
        self.configure_ranking()
        pre=np.eye(4);pre[2,3]=.1
        first=dict(self.q);second=dict(self.q)
        first.update(dict.fromkeys(ARMS['left'],.5));second.update(dict.fromkeys(ARMS['left'],.2))
        self.p.solve=Mock(side_effect=[first,second])
        self.p.tip=Mock(return_value=pre)
        with patch('task_motion_planner.preinsert_jobs',return_value=[(0,0,0),(1,0,0)]):
            ranked=self.p.rank_preinsert('left',self.q,pre,np.zeros(3),np.eye(4),self.ticket)
        self.assertEqual(len(ranked),2)
        self.assertEqual(ranked[0]['goal'],second)
        self.assertLess(ranked[0]['score']['total'],ranked[1]['score']['total'])

    def test_initial_time_budget_stops_without_claiming_exhaustive_failure(self):
        self.configure_ranking();self.p.solve=Mock()
        pre=np.eye(4);pre[2,3]=.1
        with patch('task_motion_planner.time.monotonic',side_effect=[0.,100.,100.]):
            with self.assertRaises(PlanningFailure):
                self.p.rank_preinsert('left',self.q,pre,np.zeros(3),np.eye(4),self.ticket)
        self.p.solve.assert_not_called()
        self.assertEqual(self.p.search_diagnostics['preinsert']['stop_reason'],'time_budget')
        self.assertTrue(self.p.search_diagnostics['preinsert']['truncated'])

    def test_cancellation_is_not_swallowed_by_candidate_search(self):
        self.configure_ranking();self.p.solve=Mock(side_effect=RuntimeError('cancelled'))
        pre=np.eye(4);pre[2,3]=.1
        with self.assertRaisesRegex(RuntimeError,'cancelled'):
            self.p.rank_preinsert('left',self.q,pre,np.zeros(3),np.eye(4),self.ticket)
        self.assertEqual(self.p.search_diagnostics['preinsert']['stop_reason'],'interrupted')

    def test_approach_can_try_next_ranked_goal_without_changing_entry(self):
        self.p.model=S()
        entry=np.array([.4,.1,0.]);pre=np.eye(4);pre[:3,3]=entry+[-.03,0.,.1]
        geometry={'geometry_valid_for_planning_seed':True,
            'task_preview':{'insertion_reference_m':entry.tolist(),'axis_center_m':[.3,.1,0.]},
            'grasp_candidates':{'left':{'poses':{'preinsert_index_tcp':{'matrix_row_major':pre.tolist()}}}},
            'fingertip_estimates':{'sides':{'left':{'states':{'open':{'wrist_yaw_from_index_tcp':
                {'matrix_row_major':np.eye(4).tolist()}}}}}}}
        first=dict(self.q);first.update(dict.fromkeys(ARMS['left'],.2))
        second=dict(self.q);second.update(dict.fromkeys(ARMS['left'],.3))
        candidates=[{'goal':goal,'pre':pre.copy(),'score':{'total':float(index)},'trial_index':index}
                    for index,goal in enumerate((first,second))]
        def ranking(*args):
            self.p.search_diagnostics['preinsert']={}
            return candidates
        self.p.rank_preinsert=Mock(side_effect=ranking)
        self.p.joint_path=Mock(side_effect=[PlanningFailure('MOTION_PLAN_FAILED','approach','unconnected'),
                                           {'end':second},{'end':self.q}])
        self.p.cartesian=Mock(return_value={'end':second})
        self.p.tip=Mock(return_value=pre)
        with patch('task_motion_planner.require_task_hand_model'):
            segments=self.p.ring('left',self.q,geometry,self.ticket)
        self.assertEqual(len(segments),5)
        self.assertEqual(self.p.joint_path.call_args_list[1].args[2],second)
        self.assertEqual(self.p.search_diagnostics['preinsert']['selected']['trial_index'],1)
        insertion_points=self.p.cartesian.call_args_list[0].args[2]
        np.testing.assert_allclose(insertion_points[-1],entry)

    def test_complete_path_failure_retries_with_selected_nominal(self):
        self.p.model=S();self.p.orientation_search='broad'
        entry=np.array([.4,.1,0.]);pre=np.eye(4);pre[:3,3]=entry+[0,0,.1]
        geometry={'geometry_valid_for_planning_seed':True,
            'task_preview':{'insertion_reference_m':entry.tolist(),'axis_center_m':[.3,.1,0.]},
            'grasp_candidates':{'left':{'poses':{'preinsert_index_tcp':{'matrix_row_major':pre.tolist()}}}},
            'fingertip_estimates':{'sides':{'left':{'states':{'open':{'wrist_yaw_from_index_tcp':
                {'matrix_row_major':np.eye(4).tolist()}}}}}}}
        candidates=[{'goal':dict(self.q),'pre':pre,'score':{'total':i},'trial_index':i,
                     'nominal':np.array(n)} for i,n in enumerate(([1,0,0,0],[0,1,0,0]))]
        def ranking(*args):
            self.p.search_diagnostics['preinsert']={}
            return candidates
        self.p.rank_preinsert=Mock(side_effect=ranking)
        self.p.joint_path=Mock(return_value={'end':self.q})
        self.p.ring_contact_path=Mock(side_effect=[
            PlanningFailure('CONTINUOUS_IK_FAILED','rotate','branch ended'),['complete']])
        with patch('task_motion_planner.require_task_hand_model'):
            result=self.p.ring('left',self.q,geometry,self.ticket)
        self.assertEqual(result,['complete'])
        for call,candidate in zip(self.p.ring_contact_path.call_args_list,candidates):
            np.testing.assert_equal(call.args[5],candidate['nominal'])
        self.assertEqual(self.p.search_diagnostics['preinsert']['selected']['trial_index'],1)


class RetimedGeometryTests(unittest.TestCase):
    def setUp(self):
        self.p=TaskPlanner.__new__(TaskPlanner)
        self.p.tip=Mock(return_value=np.eye(4))
        self.report=dict(checked_samples=0,max_position_error_m=0.,max_tilt_deg=0.,max_twist_deg=0.)

    def check(self,distance=0.,angles=(0.,0.,0.)):
        tip=np.eye(4);tip[:3,:3]=Rotation.from_euler('xyz',angles,degrees=True).as_matrix()
        self.p.tip.return_value=tip
        self.p.check_tcp_geometry('left',{},np.eye(4),[0,0,0,1],lambda p:distance,
                                  orientation_policy({}),'insert','Insert',self.report)

    def test_position_tilt_and_twist_are_identified_separately(self):
        for field,distance,angles in [('position',.0021,(0,0,0)),('tilt',0.,(5.1,0,0)),
                                      ('twist',0.,(0,0,10.1))]:
            with self.subTest(field=field),self.assertRaises(PlanningFailure) as caught:
                self.check(distance,angles)
            d=caught.exception.diagnostic
            self.assertEqual(d['code'],'RETIMED_TCP_BOUNDS')
            self.assertEqual(d['exceeded'],[field])
            self.assertEqual(d['position_limit_m'],.002)
            self.assertEqual((d['tilt_limit_deg'],d['twist_limit_deg']),(5.,10.))
            self.assertIs(self.report['failure'],d)

    def test_existing_limits_and_epsilon_unchanged(self):
        self.check(.002,(5.,0,0));self.check(.001,(0,0,10.00005))
        self.assertEqual(self.report['checked_samples'],2)
        self.assertEqual(self.report['max_position_error_m'],.002)
        self.assertNotIn('failure',self.report)

    def test_nonfinite_distance_is_not_accepted_or_retried(self):
        with self.assertRaisesRegex(PlanningFailure,'INVALID_TCP_GEOMETRY'):
            self.check(float('nan'))

    def test_callable_arc_reference_is_used(self):
        tip=np.eye(4);tip[:3,:3]=Rotation.from_euler('z',30,degrees=True).as_matrix()
        self.p.tip.return_value=tip
        reference=Mock(return_value=Rotation.from_euler('z',30,degrees=True).as_quat())
        self.p.check_tcp_geometry('left',{},np.eye(4),reference,lambda p:0.,
                                 orientation_policy({}),'rotate','Left 120',self.report)
        self.assertAlmostEqual(self.report['max_twist_deg'],0.)
        reference.assert_called_once()

    def test_retime_failure_records_sample_location(self):
        self.p.search_diagnostics={}
        from moveit_msgs.msg import RobotTrajectory
        from trajectory_msgs.msg import JointTrajectoryPoint
        trajectory=RobotTrajectory()
        trajectory.joint_trajectory.joint_names=list(ARMS['left'])
        for second in (0,1):
            point=JointTrajectoryPoint(positions=[.1]*7,velocities=[0.]*7,accelerations=[0.]*7)
            point.time_from_start.sec=second
            trajectory.joint_trajectory.points.append(point)
        self.p.timing=object();self.p.call=Mock(return_value=S(success=True,trajectory=trajectory))
        self.p.model=S(robot=Mock());self.p.progress=Mock();self.p.validate=Mock()
        self.p.model.robot.find.return_value=ET.fromstring('<limit lower="-2" upper="2"/>')
        failure=PlanningFailure('RETIMED_TCP_BOUNDS','insert','position')
        check=Mock(side_effect=failure)
        with self.assertRaises(PlanningFailure) as caught:
            self.p.retime('left',dict.fromkeys(ARM_NAMES,.1),trajectory,
                          Ticket(1,'task','left',.15,.15),'Insert',check,'insert')
        self.assertIs(caught.exception,failure)
        self.assertEqual(failure.diagnostic['timed_sample_index'],1)
        self.assertGreater(failure.diagnostic['timed_sample_count'],1)
        self.assertIsNone(failure.diagnostic['intersample_fraction'])
        records=self.p.search_diagnostics['performance']['trajectory_validation']
        self.assertEqual(records[-1]['status'],'failed')
        self.assertEqual(records[-1]['failure']['code'],'RETIMED_TCP_BOUNDS')
        self.assertGreaterEqual(records[-1]['seconds'],0.)
        check.side_effect=TimeoutError('task deadline')
        with self.assertRaisesRegex(TimeoutError,'task deadline'):
            self.p.retime('left',dict.fromkeys(ARM_NAMES,.1),trajectory,
                          Ticket(1,'task','left',.15,.15),'Insert',check,'insert')
        self.assertEqual(records[-1]['status'],'failed')
        self.assertEqual(records[-1]['error_type'],'TimeoutError')
        self.assertGreaterEqual(records[-1]['seconds'],0.)
        self.p.validate_trajectory_samples=Mock(side_effect=PlanningFailure(
            'COLLISION','insert','base',batch_sample_index=3))
        with self.assertRaisesRegex(PlanningFailure,'COLLISION'):
            self.p.retime('left',dict.fromkeys(ARM_NAMES,.1),trajectory,
                          Ticket(1,'task','left',.15,.15),'Insert',check,'insert')
        self.assertEqual(records[-1]['validator'],'scoped_numpy_fcl')
        self.assertEqual(records[-1]['failure']['timed_sample_index'],4)
        self.p.validate_trajectory_samples.side_effect=None
        self.p.retime('left',dict.fromkeys(ARM_NAMES,.1),trajectory,
                      Ticket(1,'task','left',.15,.15),'Insert',check,'insert')
        self.assertEqual(records[-1]['status'],'passed')
        self.assertEqual(records[-1]['last_progress_sample'],records[-1]['samples'])
        self.assertEqual(len(records),4)


class CompleteCandidateRetryTests(unittest.TestCase):
    def setUp(self):
        self.p=TaskPlanner.__new__(TaskPlanner)
        self.p.model=S();self.p.orientation_search='broad';self.p.progress=Mock()
        self.q=dict.fromkeys(ARM_NAMES,.1);pre=np.eye(4);pre[2,3]=.1
        self.geometry={'geometry_valid_for_planning_seed':True,
            'task_preview':{'insertion_reference_m':[0,0,0],'axis_center_m':[-.1,0,0]},
            'grasp_candidates':{'left':{'poses':{'preinsert_index_tcp':{'matrix_row_major':pre.tolist()}}}}}
        self.p.contact_hand_setup=Mock(return_value=('index_only',np.eye(4)))
        self.candidates=[dict(goal=dict(self.q),pre=pre.copy(),score={'total':i},trial_index=i)
                         for i in range(10)]
        def rank(*args):
            self.p.search_diagnostics['preinsert']={}
            return self.candidates
        self.p.rank_preinsert=Mock(side_effect=rank)
        self.p.joint_path=Mock(return_value={'end':dict(self.q)})
        self.p.ring_contact_path=Mock()

    def run_plan(self):
        with patch('task_motion_planner.require_task_hand_model'):
            return self.p.ring('left',self.q,self.geometry,Ticket(1,'task','left',.15,.15))

    def test_geometry_failure_retries_next_candidate_with_diagnostics(self):
        failure=PlanningFailure('RETIMED_TCP_BOUNDS','insert','tilt',tilt_deg=5.01)
        self.p.ring_contact_path.side_effect=[failure,['complete']]
        self.assertEqual(self.run_plan(),['complete'])
        report=self.p.search_diagnostics['preinsert']
        self.assertEqual(report['selected']['trial_index'],1)
        self.assertEqual(report['approach_failures'][0]['tilt_deg'],5.01)
        self.assertEqual(report['approach_failures'][0]['trial_index'],0)
        self.assertEqual(self.q,dict.fromkeys(ARM_NAMES,.1))

    def test_all_failed_candidates_are_bounded_and_recorded(self):
        self.p.ring_contact_path.side_effect=lambda *args: (_ for _ in ()).throw(
            PlanningFailure('RETIMED_TCP_BOUNDS','insert','deviation'))
        with self.assertRaises(PlanningFailure) as caught:self.run_plan()
        self.assertEqual(caught.exception.diagnostic['code'],'APPROACH_PATH_FAILED')
        self.assertEqual(self.p.ring_contact_path.call_count,8)
        self.assertEqual([d['trial_index'] for d in caught.exception.diagnostic['failures']],list(range(8)))
        self.assertNotIn('selected',self.p.search_diagnostics['preinsert'])

    def test_stop_timeout_and_invalid_scene_are_not_retried(self):
        for failure in (RuntimeError('cancelled'),TimeoutError('budget'),
                        PlanningFailure('SCENE_NOT_READY','insert','stale'),
                        PlanningFailure('INVALID_TCP_GEOMETRY','insert','nonfinite')):
            with self.subTest(error=str(failure)):
                self.p.ring_contact_path.reset_mock()
                self.p.ring_contact_path.side_effect=failure
                with self.assertRaises(type(failure)) as caught:self.run_plan()
                self.assertIs(caught.exception,failure)
                self.assertEqual(self.p.ring_contact_path.call_count,1)


if __name__=='__main__':unittest.main()
