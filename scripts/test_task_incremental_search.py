"""Production lazy search and candidate selection with deterministic, offline IK."""
from types import SimpleNamespace as S
import unittest
from unittest.mock import Mock, patch

import numpy as np
from scipy.spatial.transform import Rotation
from task_motion_planner import TaskPlanner, PlanningFailure
from task_session import ARM_NAMES, ARMS, Ticket


class IncrementalSearchTests(unittest.TestCase):
    def setUp(self):
        self.now=0.;self.calls=0;self.timeouts=[];self.solutions={}
        self.p=TaskPlanner.__new__(TaskPlanner)
        self.p.model=S();self.p.orientation_search='broad';self.p.incremental_search=True
        self.p.search_seconds=30.;self.p.search_attempts=220;self.p.search_diagnostics={}
        self.p.progress=Mock();self.q=dict.fromkeys(ARM_NAMES,0.)
        self.p.joint_limits=Mock(return_value={n:(-2.,2.) for n in ARMS['left']})
        self.pre=np.eye(4);self.pre[:3,:3]=Rotation.from_euler('x',180,degrees=True).as_matrix()
        self.pre[2,3]=.1
        self.geometry={'geometry_valid_for_planning_seed':True,
            'task_preview':{'insertion_reference_m':[0,0,0],'axis_center_m':[-.1,0,0]},
            'grasp_candidates':{'left':{'poses':{'preinsert_index_tcp':{'matrix_row_major':self.pre.tolist()}}}}}
        self.ticket=Ticket(1,'task','left',.15,.15)
        self.p.contact_hand_setup=Mock(return_value=('index_only',np.eye(4)))
        self.p.tip=lambda side,q,wrist:self.solutions.get(q[ARMS['left'][0]],self.pre)
        self.p.solve=Mock(side_effect=self.solve)
        self.p.joint_path=Mock(side_effect=lambda side,q,goal,*a,**kw:dict(end=goal))
        self.p.ring_contact_path=Mock(return_value=['complete'])

    def solve(self,side,q,target,*args,**kwargs):
        self.calls+=1;self.timeouts.append(kwargs['rpc_timeout']);self.now+=.05
        answer=dict(q);answer[ARMS['left'][0]]=.01+self.calls*.001
        self.solutions[answer[ARMS['left'][0]]]=target.copy()
        self.p.last_ik={'code':'VALID'}
        return answer

    def run_ring(self):
        with patch('task_motion_planner.require_task_hand_model'),\
                patch('task_motion_planner.time.monotonic',lambda:self.now):
            return self.p.ring('left',self.q,self.geometry,self.ticket)

    def test_first_complete_success_stops_at_32_not_220_trials(self):
        self.assertEqual(self.run_ring(),['complete'])
        report=self.p.search_diagnostics['preinsert']
        self.assertEqual((self.calls,report['attempted']),(32,32))
        self.assertEqual(report['stop_reason'],'complete_candidate_accepted')
        self.assertEqual(report['batches'][0]['attempted'],32)
        self.assertAlmostEqual(report['elapsed_seconds'],1.6)
        self.p.ring_contact_path.assert_called_once()

    def test_diverse_mode_preserves_first_success_and_reports_explicit_policy(self):
        self.p.preinsert_policy='diverse'
        self.assertEqual(self.run_ring(),['complete'])
        report=self.p.search_diagnostics['preinsert']
        self.assertEqual(report['candidate_policy'],'diverse')
        self.assertEqual(report['attempted'],32)
        self.p.ring_contact_path.assert_called_once()

    def test_diverse_selection_receives_previously_delivered_batches(self):
        from task_ik_search import diverse_preinsert_batch
        self.p.preinsert_policy='diverse'
        self.p.ring_contact_path.side_effect=[PlanningFailure('COLLISION','rotate','blocked'),
            PlanningFailure('CONTINUOUS_IK_FAILED','rotate','dead branch'),['complete']]
        seen=[]
        def record(candidates,delivered,names,count):
            seen.append([c['trial_index'] for c in delivered])
            return diverse_preinsert_batch(candidates,delivered,names,count)
        with patch('task_motion_planner.diverse_preinsert_batch',side_effect=record):
            self.assertEqual(self.run_ring(),['complete'])
        batches=self.p.search_diagnostics['preinsert']['batches']
        self.assertEqual(seen[0],[])
        self.assertEqual(seen[1],[c['trial_index'] for c in batches[0]['candidates']])
        self.assertEqual(self.calls,64)
        goals=[tuple(c.args[2][n] for n in ARMS['left']) for c in self.p.joint_path.call_args_list]
        self.assertEqual(len(goals),len(set(goals)))

    def test_invalid_preinsert_policy_does_not_attempt_ik(self):
        self.p.preinsert_policy='other'
        with self.assertRaisesRegex(ValueError,'preinsert search policy'):self.run_ring()
        self.p.solve.assert_not_called()

    def test_failed_full_candidates_extend_search_without_duplicate_goals(self):
        self.p.ring_contact_path.side_effect=[PlanningFailure('COLLISION','rotate','offline obstacle'),
            PlanningFailure('CONTINUOUS_IK_FAILED','insert','offline branch ended'),['complete']]
        self.assertEqual(self.run_ring(),['complete']);self.assertEqual(self.calls,64)
        report=self.p.search_diagnostics['preinsert']
        self.assertEqual([b['attempted'] for b in report['batches']],[32,64])
        self.assertEqual(len(report['approach_failures']),2)
        goals=[tuple(c.args[2][n] for n in ARMS['left']) for c in self.p.joint_path.call_args_list]
        self.assertEqual(len(goals),len(set(goals)))

    def test_complete_path_validation_time_does_not_consume_initial_ik_budget(self):
        def validate(*args):
            self.now+=40.
            if self.p.ring_contact_path.call_count<3:
                raise PlanningFailure('COLLISION','rotate','offline obstacle')
            return ['complete']
        self.p.ring_contact_path.side_effect=validate
        self.assertEqual(self.run_ring(),['complete']);self.assertEqual(self.calls,64)
        self.assertAlmostEqual(self.p.search_diagnostics['preinsert']['elapsed_seconds'],3.2)
        self.assertGreater(self.now,120.)

    def test_no_solution_obeys_attempt_cap(self):
        def fail(*args,**kw):
            self.calls+=1;self.now+=.01;self.p.last_ik={'code':'IK_FAILED'}
        self.p.solve.side_effect=fail
        with self.assertRaisesRegex(PlanningFailure,'220 trials'):self.run_ring()
        self.assertEqual(self.calls,220)
        self.assertEqual(self.p.search_diagnostics['preinsert']['stop_reason'],'attempt_budget')
        self.p.joint_path.assert_not_called()

    def test_remaining_search_budget_is_propagated_to_each_rpc(self):
        self.p.search_seconds=.2
        def fail(*args,**kw):
            self.timeouts.append(kw['rpc_timeout']);self.now+=.08
            self.p.last_ik={'code':'IK_FAILED'}
        self.p.solve.side_effect=fail
        with self.assertRaises(PlanningFailure):self.run_ring()
        np.testing.assert_allclose(self.timeouts,[.2,.12,.04])
        self.assertEqual(self.p.search_diagnostics['preinsert']['stop_reason'],'time_budget')

    def test_nonretryable_validation_error_closes_search_without_more_ik(self):
        self.p.ring_contact_path.side_effect=PlanningFailure('MODEL_MISMATCH','insert','model changed')
        with self.assertRaisesRegex(PlanningFailure,'MODEL_MISMATCH'):self.run_ring()
        report=self.p.search_diagnostics['preinsert']
        self.assertEqual((self.calls,report['attempted']),(32,32))
        self.assertEqual(report['stop_reason'],'complete_path_failed_or_cancelled')

    def test_cancellation_during_complete_validation_does_not_resume_search(self):
        self.p.ring_contact_path.side_effect=RuntimeError('cancelled')
        with self.assertRaisesRegex(RuntimeError,'cancelled'):self.run_ring()
        self.assertEqual(self.calls,32)
        self.assertEqual(self.p.search_diagnostics['preinsert']['attempted'],32)

    def test_preinsert_candidate_reserves_margin_inside_final_angular_limit(self):
        original=self.p.solve.side_effect
        def boundary(*args,**kwargs):
            q=original(*args,**kwargs)
            pose=self.solutions[q[ARMS['left'][0]]]
            pose[:3,:3]=pose[:3,:3]@Rotation.from_euler('x',5.00005,degrees=True).as_matrix()
            return q
        self.p.solve.side_effect=boundary;self.p.search_attempts=1
        with self.assertRaisesRegex(PlanningFailure,'No valid candidate'):self.run_ring()
        report=self.p.search_diagnostics['preinsert']
        self.assertEqual(report['trials'][0]['code'],'TCP_RESIDUAL_OR_ORIENTATION')
        self.p.joint_path.assert_not_called()

    def test_trial_rotation_is_used_for_ik_reference_and_saved_candidate(self):
        shift=Rotation.from_euler('z',25,degrees=True)
        self.p.preinsert_trial_rotation=lambda quat,position:(shift*Rotation.from_quat(quat)).as_quat()
        self.assertEqual(self.run_ring(),['complete'])
        report=self.p.search_diagnostics['preinsert']
        selected=report['selected'];trial=report['trials'][selected['trial_index']]
        np.testing.assert_allclose(selected['orientation_reference_xyzw'],trial['quaternion_xyzw'])
        np.testing.assert_allclose(np.asarray(selected['preinsert_tcp_matrix'])[:3,:3],
                                   Rotation.from_quat(trial['quaternion_xyzw']).as_matrix())


class ContactSequenceSearchTests(unittest.TestCase):
    def setUp(self):
        self.p=TaskPlanner.__new__(TaskPlanner)
        self.p.planning_parameters=S(contact_search_seconds=10.,contact_branch_attempts=6,contact_candidate_seconds=5.)
        self.p.search_diagnostics={};self.p.progress=Mock()
        self.now=0.;self.opened=[];self.closed=[];self.resumed=[];self.generators=[]
        self.start={'branch':'start'};self.wrist=np.eye(4);self.ticket=object()
        self.specifications=tuple(Mock(return_value=([],None,stage,None,stage))
                                  for stage in ('insert','rotate','withdraw'))
        self.branches={};self.results={}
        self.p.cartesian_paths=Mock(side_effect=self.paths)
        clock=patch('task_motion_planner.time.monotonic',lambda:self.now)
        clock.start();self.addCleanup(clock.stop)

    def segment(self,name):
        return self.results.setdefault(name,{'label':name,'end':{'branch':name},
                                             'trajectory':object()})

    def paths(self,side,state,points,reference,wrist,ticket,label,curve,stage,*,branching):
        self.assertEqual(side,'left');self.assertTrue(branching)
        self.assertIs(wrist,self.wrist);self.assertIs(ticket,self.ticket)
        key=(stage,state['branch'])
        def generate():
            self.opened.append(key)
            try:
                for candidate in self.branches[key]:
                    if isinstance(candidate,Exception):raise candidate
                    error=yield self.segment(candidate)
                    self.assertIsInstance(error,PlanningFailure)
                    self.resumed.append((key,error))
            finally:self.closed.append(key)
        generator=generate();self.generators.append(generator)
        return generator

    def run_search(self):
        return self.p.search_contact_sequence('left',self.start,self.specifications,
                                              self.wrist,self.ticket)

    def assert_closed(self):
        self.assertCountEqual(self.closed,self.opened)
        self.assertTrue(all(generator.gi_frame is None for generator in self.generators))

    def assert_complete(self,result,names):
        self.assertEqual(len(result),3)
        for segment,name in zip(result,names):self.assertIs(segment,self.results[name])
        self.assertTrue(self.p.search_diagnostics['contact_branch_search'][-1]['accepted'])
        self.assert_closed()

    def test_withdraw_failure_sends_error_to_rotate_without_resuming_insert(self):
        error=PlanningFailure('COLLISION','withdraw','blocked exit')
        self.branches={('insert','start'):['i1','unused'],
                       ('rotate','i1'):['r1','r2'],
                       ('withdraw','r1'):[error],('withdraw','r2'):['w2']}
        self.assert_complete(self.run_search(),['i1','r2','w2'])
        self.assertEqual(self.resumed,[(('rotate','i1'),error)])
        self.assertEqual(self.opened,[('insert','start'),('rotate','i1'),
                                      ('withdraw','r1'),('withdraw','r2')])
        self.specifications[0].assert_called_once_with(self.start)
        self.specifications[1].assert_called_once_with(self.results['i1']['end'])
        self.assertEqual([call.args[0] for call in self.specifications[2].call_args_list],
                         [self.results['r1']['end'],self.results['r2']['end']])
        self.assertEqual(self.closed,[('withdraw','r1'),('withdraw','r2'),
                                      ('rotate','i1'),('insert','start')])

    def test_exhausted_rotate_resumes_insert_and_rebuilds_only_suffix(self):
        error=PlanningFailure('CONTINUOUS_IK_FAILED','withdraw','no exit branch')
        self.branches={('insert','start'):['i1','i2'],('rotate','i1'):['r1'],
                       ('withdraw','r1'):[error],('rotate','i2'):['r2'],
                       ('withdraw','r2'):['w2']}
        self.assert_complete(self.run_search(),['i2','r2','w2'])
        self.assertEqual(self.resumed,[(('rotate','i1'),error),(('insert','start'),error)])
        self.assertEqual(self.opened,[('insert','start'),('rotate','i1'),
                                      ('withdraw','r1'),('rotate','i2'),('withdraw','r2')])
        self.specifications[0].assert_called_once_with(self.start)
        self.assertEqual([call.args[0] for call in self.specifications[1].call_args_list],
                         [self.results['i1']['end'],self.results['i2']['end']])

    def test_terminal_errors_close_generators_without_returning_partial_trajectory(self):
        errors=(PlanningFailure('CONTACT_SEARCH_BUDGET','contact','deadline'),
                RuntimeError('cancelled'),PlanningFailure('MODEL_MISMATCH','withdraw','changed model'))
        for error in errors:
            with self.subTest(error=str(error)):
                self.opened.clear();self.closed.clear();self.resumed.clear()
                self.branches={('insert','start'):['i1','unused'],
                               ('rotate','i1'):['r1','unused'],('withdraw','r1'):[error]}
                with self.assertRaises(type(error)) as caught:self.run_search()
                self.assertIs(caught.exception,error);self.assertEqual(self.resumed,[])
                self.assertEqual(self.closed,[('withdraw','r1'),('rotate','i1'),('insert','start')])
                self.assertFalse(self.p.search_diagnostics['contact_branch_search'][-1]['accepted'])
                self.assert_closed()

    def test_return_collision_resumes_withdraw_and_keeps_contact_prefix(self):
        error=PlanningFailure('COLLISION','return_ready','blocked return')
        self.branches={('insert','start'):['i1'],('rotate','i1'):['r1'],
                       ('withdraw','r1'):['w1','w2']}
        terminal=Mock(side_effect=[error,[self.segment('open2'),self.segment('ready2')]])
        result=self.p.search_contact_sequence('left',self.start,self.specifications,
                                              self.wrist,self.ticket,terminal=terminal)
        self.assertEqual([s['label'] for s in result],['i1','r1','w2','open2','ready2'])
        self.assertEqual(self.resumed,[(('withdraw','r1'),error)])
        self.assertEqual([c.args[0]['branch'] for c in terminal.call_args_list],['w1','w2'])
        self.assert_closed()

    def test_return_failure_exhaustion_never_returns_contact_only_success(self):
        error=PlanningFailure('MOTION_PLAN_FAILED','return_ready','no route')
        self.branches={('insert','start'):['i1'],('rotate','i1'):['r1'],('withdraw','r1'):['w1']}
        terminal=Mock(side_effect=error)
        with self.assertRaises(PlanningFailure) as caught:
            self.p.search_contact_sequence('left',self.start,self.specifications,
                                          self.wrist,self.ticket,terminal=terminal)
        self.assertIs(caught.exception,error)
        self.assertFalse(self.p.search_diagnostics['contact_branch_search'][-1]['accepted'])
        self.assert_closed()

    def test_return_timeout_is_not_retried(self):
        self.branches={('insert','start'):['i1'],('rotate','i1'):['r1'],('withdraw','r1'):['w1','w2']}
        terminal=Mock(side_effect=TimeoutError('full task deadline'))
        with self.assertRaises(TimeoutError):
            self.p.search_contact_sequence('left',self.start,self.specifications,
                                          self.wrist,self.ticket,terminal=terminal)
        self.assertEqual(self.resumed,[])
        terminal.assert_called_once()
        self.assert_closed()

if __name__=='__main__':unittest.main()
