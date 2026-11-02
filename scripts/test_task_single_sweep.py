"""Single-sweep regression fixtures; no live ROS nodes or command publishers."""
import copy
from dataclasses import replace
import json
import math
from pathlib import Path
from types import SimpleNamespace as S
import unittest
from unittest.mock import Mock,patch

import numpy as np
from scipy.spatial.transform import Rotation

from task_dring_sequence import SCHEMA,labels,plan_labels
from task_motion_planner import TaskPlanner,PlanningFailure
from task_planning_config import load_parameters,configure_planner
from task_straight_policy import StraightIndexPolicy
from task_trajectory import task_motion_limits,task_jerk_limits,stretch_to_limits,samples,validate_waist_envelope


class Planner(StraightIndexPolicy,TaskPlanner):
    pass


class SingleSweepTests(unittest.TestCase):
    def test_local_waist_bias_preserves_target_and_rejects_unreachable(self):
        from task_ik_search import waist_biased_continuation
        names=['left_shoulder_roll_joint','waist_yaw_joint']
        current=dict.fromkeys(names,0.);limits=dict.fromkeys(names,(-1.,1.))
        def tip(state):
            result=np.eye(4);result[0,3]=sum(state.values());return result
        target=np.eye(4);target[0,3]=.05
        result=waist_biased_continuation(names,limits,current,target,tip)
        self.assertIsNotNone(result)
        self.assertLess(result[names[0]],result[names[1]])
        np.testing.assert_allclose(tip(result),target,atol=1e-5)
        self.assertLessEqual(max(abs(v) for v in result.values()),.1)
        target[0,3]=1.
        self.assertIsNone(waist_biased_continuation(names,limits,current,target,tip))

    def test_waist_bias_scope_and_original_collision_validation(self):
        self.p.planning_parameters=replace(self.parameters,right_sweep_waist_bias=True)
        self.p.waist_assist=True;self.p.search_diagnostics={}
        self.p.names=Mock(return_value=['waist_yaw_joint'])
        self.p.joint_limits=Mock(return_value={'waist_yaw_joint':(-1.,1.)})
        self.p.validate=Mock()
        candidate={'waist_yaw_joint':.02};ticket=object()
        with patch('task_ik_search.waist_biased_continuation',return_value=candidate) as local,\
                patch.object(TaskPlanner,'solve',return_value={'legacy':True}) as legacy:
            for direction,stage,waist in (('left','rotate',True),('right','insert',True),
                                          ('right','withdraw',True),('right','rotate',False)):
                self.p.sweep_direction=direction;self.p.waist_assist=waist
                self.assertEqual(self.p.solve('left',{},np.eye(4),np.eye(4),ticket,stage),{'legacy':True})
            local.assert_not_called()
            self.p.sweep_direction='right';self.p.waist_assist=True
            self.assertEqual(self.p.solve('left',{},np.eye(4),np.eye(4),ticket,'rotate'),candidate)
            self.p.validate.assert_called_once_with(candidate,ticket,'left','rotate')
            self.p.validate.side_effect=PlanningFailure('COLLISION','rotate','blocked')
            self.assertEqual(self.p.solve('left',{},np.eye(4),np.eye(4),ticket,'rotate'),{'legacy':True})
            self.assertEqual(self.p.search_diagnostics['right_sweep_waist_bias']['fallback'],1)
            self.p.validate.side_effect=RuntimeError('STOP')
            with self.assertRaisesRegex(RuntimeError,'STOP'):
                self.p.solve('left',{},np.eye(4),np.eye(4),ticket,'rotate')

    def setUp(self):
        self.p=Planner.__new__(Planner)
        self.parameters,_=load_parameters()
        self.parameters=replace(self.parameters,path_tolerance_mm=8.,target_tolerance_mm=10.,
                                position_search_fraction=.75,sweep_hand_back_min_deg=0.,sweep_hand_back_max_deg=0.,
                                hand_back_tangent_xy=False,contact_branch_search=False)
        configure_planner(self.p,self.parameters)
        self.p.dorsal_in_tip=np.array([1.,0.,0.])
        self.p.fixed_forward_yaw=0.

    def tangent_setup(self):
        self.p.planning_parameters=replace(self.parameters,hand_back_tangent_xy=True)
        self.p.search_diagnostics={'straight_index':{}}
        self.p.configure_contact_geometry(np.zeros(3),np.array([.1,0.,0.]))

    def test_tangent_full_sweep_both_branches_and_directions(self):
        self.tangent_setup()
        for dorsal in ([1.,0.,0.],[-1.,0.,0.]):
            self.p.dorsal_in_tip=np.array(dorsal)
            for yaw in (90.,-90.):
                reference=self.p.tangent_reference(Rotation.from_euler('z',yaw,degrees=True).as_quat())
                for direction in (-1,1):
                    angles=np.linspace(0,direction*120,121)
                    points=Rotation.from_euler('z',angles,degrees=True).apply([.1,0,0])
                    quats=reference.batch(points)
                    np.testing.assert_allclose(quats,np.array([reference(p) for p in points]),atol=1e-12)
                    tips=np.tile(np.eye(4),(len(points),1,1))
                    tips[:,:3,3]=points;tips[:,:3,:3]=Rotation.from_quat(quats).as_matrix()
                    self.assertLess(np.max(self.p.tangent_errors(tips)),1e-5)
                    self.assertIsNone(self.p.heading_failure(tips,'rotate','test',{}))
                    tips[60,:3,:3]=Rotation.from_euler('z',180,degrees=True).as_matrix()@tips[60,:3,:3]
                    failure=self.p.heading_failure(tips,'rotate','test',{})
                    self.assertEqual(failure.diagnostic['batch_sample_index'],60)

    def test_tangent_actual_position_degeneracy_and_margin(self):
        self.tangent_setup()
        tip=np.eye(4);tip[:3,3]=[.01,0,0]
        tip[:3,:3]=Rotation.from_euler('z',90,degrees=True).as_matrix()
        self.assertTrue(self.p.contact_pose_ok(tip,'insert'))
        tip[1,3]=.003
        self.assertFalse(self.p.contact_pose_ok(tip,'insert'))
        tip[:3,3]=[0,0,.1]
        self.assertFalse(self.p.contact_pose_ok(tip,'withdraw'))
        tip[:3,3]=[.1,0,0]
        tip[:3,:3]=Rotation.from_euler('y',90,degrees=True).as_matrix()
        self.assertFalse(self.p.contact_pose_ok(tip,'insert'))
        for error,allowed in ((4.8,True),(5.,False),(5.1,False)):
            tip[:3,:3]=Rotation.from_euler('z',90+error,degrees=True).as_matrix()
            self.assertEqual(self.p.contact_pose_ok(tip,'insert'),allowed)
        self.assertTrue(self.p.contact_pose_ok(tip,'approach'))
        self.p.center_check=True
        self.assertFalse(self.p.tangent_mode())

    def test_tangent_preinsert_candidates_and_retry_branch(self):
        self.tangent_setup()
        candidates=self.p.preinsert_rotations([0,0,0,1],{})
        self.assertTrue(candidates)
        for q in candidates:
            tip=np.eye(4);tip[:3,3]=[.1,0,.1];tip[:3,:3]=Rotation.from_quat(q).as_matrix()
            self.p.tangent_branch=-1.
            self.assertTrue(self.p.contact_pose_ok(tip,'preinsert'))
        self.assertTrue(self.p.heading_allowed(Rotation.from_euler('z',170,degrees=True).as_quat()))

    def test_inclined_preinsert_ray_uses_trial_bearing_without_flipping_branch(self):
        self.tangent_setup()
        for yaw,branch in ((90.,1.),(-90.,-1.)):
            original=Rotation.from_euler('z',yaw,degrees=True).as_quat()
            for position in ([.02,.02,.1],[-.02,.02,.1]):
                shifted=self.p.preinsert_trial_rotation(original,position)
                tip=np.eye(4);tip[:3,3]=position;tip[:3,:3]=Rotation.from_quat(shifted).as_matrix()
                self.assertTrue(self.p.contact_pose_ok(tip,'preinsert'))
                reference=self.p.tangent_reference(shifted,position)
                self.assertEqual(self.p.tangent_branch,branch)
                np.testing.assert_allclose(Rotation.from_quat(reference(position)).as_matrix(),tip[:3,:3],atol=1e-12)
                entry_tip=np.eye(4);entry_tip[:3,3]=self.p.tangent_entry
                entry_tip[:3,:3]=Rotation.from_quat(reference(self.p.tangent_entry)).as_matrix()
                self.assertLess(self.p.tangent_errors([tip,entry_tip]).max(),1e-5)
        self.assertIsNone(self.p.preinsert_trial_rotation(original,[0,0,.1]))
        self.p.center_check=True
        np.testing.assert_allclose(self.p.preinsert_trial_rotation(original,[.02,.02,.1]),original)

    def test_existing_preinsert_candidates_inside_tangent_margin_are_preserved_exactly(self):
        self.tangent_setup()
        original=Rotation.from_euler('z',90,degrees=True).as_quat()
        for angle in (0.,4.2,-4.8):
            position=Rotation.from_euler('z',angle,degrees=True).apply([.1,0,.1])
            self.assertIs(self.p.preinsert_trial_rotation(original,position),original)
        position=Rotation.from_euler('z',5.2,degrees=True).apply([.1,0,.1])
        self.assertIsNot(self.p.preinsert_trial_rotation(original,position),original)

    def test_tangent_reference_used_in_all_contact_segments(self):
        self.tangent_setup()
        self.p.sweep_direction='right'
        tip=np.eye(4);tip[:3,3]=[-.05,-.0866,0.]
        self.p.tip=Mock(return_value=tip)
        self.p.cartesian=Mock(side_effect=lambda *a,**kw:dict(end={},trajectory=None))
        pre=np.eye(4);pre[:3,3]=[.1,0,.1]
        with patch('task_stage_control.slow_segment'):
            self.p.contact_segments('left',{},pre,np.array([.1,0,0]),np.zeros(3),
                Rotation.from_euler('z',90,degrees=True).as_quat(),np.eye(4),None)
        refs=[c.args[3] for c in self.p.cartesian.call_args_list]
        self.assertTrue(callable(refs[0]))
        self.assertTrue(all(ref is refs[0] for ref in refs))

    def test_heading_is_horizontal_yaw_not_a_three_dimensional_cone(self):
        tilted=Rotation.from_euler('y',70,degrees=True).as_quat()
        self.assertAlmostEqual(float(self.p.heading_error(tilted)),0.)
        self.assertTrue(self.p.heading_allowed(tilted))
        for yaw in (-46,46,180):
            self.assertFalse(self.p.heading_allowed(Rotation.from_euler('z',yaw,degrees=True).as_quat()))

    def test_yaw_wrap_and_vertical_degeneracy(self):
        self.p.fixed_forward_yaw=math.radians(179)
        self.assertAlmostEqual(float(self.p.heading_error(Rotation.from_euler('z',-179,degrees=True).as_quat())),2.)
        self.assertFalse(self.p.heading_allowed(Rotation.from_euler('y',90,degrees=True).as_quat()))

    def test_reference_is_frozen_at_startup_and_ready_is_copied(self):
        def transform(link,q):
            t=np.eye(4);t[:3,:3]=Rotation.from_euler('z',q['waist']).as_matrix();return t
        self.p.model=S(transform=transform)
        ready={'arm':.2};startup={'waist':.3}
        self.p.configure_task('left',startup,ready,'right')
        startup['waist']=1.;ready['arm']=2.
        self.assertAlmostEqual(self.p.fixed_forward_yaw,.3)
        self.assertEqual(self.p.ready_target,{'arm':.2})
        self.assertEqual(self.p.sweep_direction,'right')

    def test_candidates_always_respect_frozen_hand_heading(self):
        candidates=self.p.preinsert_rotations([0,0,0,1],self.parameters.orientation_policy())
        self.assertTrue(candidates)
        self.assertTrue(all(self.p.heading_allowed(q) for q in candidates))
        self.assertLessEqual(max(float(self.p.heading_error(q)) for q in candidates),45.)

    def test_directional_hand_heading_prefers_opposite_small_yaw(self):
        self.p.planning_parameters=replace(self.parameters,left_sweep_hand_back_yaw_deg=-10.,
                                           right_sweep_hand_back_yaw_deg=10.)
        self.p.fixed_forward_yaw=math.radians(17.)
        for direction,expected in (('left',-10.),('right',10.)):
            self.p.sweep_direction=direction
            candidates=self.p.preinsert_rotations([0,0,0,1],self.parameters.orientation_policy())
            normal=Rotation.from_quat(candidates[0]).apply(self.p.dorsal_in_tip)
            actual=math.degrees(math.atan2(normal[1],normal[0])-self.p.fixed_forward_yaw)
            self.assertAlmostEqual(actual,expected)
            self.assertTrue(all(self.p.heading_allowed(q) for q in candidates))
            self.assertAlmostEqual(self.p.preferred_heading_error(candidates[0]),0.)
        self.p.center_check=True
        self.assertEqual(self.p.preferred_hand_back_yaw(),0.)

    def test_hard_heading_range_filters_candidates_and_retimed_interior(self):
        self.p.planning_parameters=replace(self.parameters,left_sweep_hand_back_yaw_deg=-20.,
            right_sweep_hand_back_yaw_deg=20.,sweep_hand_back_min_deg=10.,sweep_hand_back_max_deg=30.)
        self.p.fixed_forward_yaw=math.radians(179.)
        for direction,sign in (('left',-1),('right',1)):
            self.p.sweep_direction=direction
            def rotation(degrees):
                return Rotation.from_euler('z',179+sign*degrees,degrees=True)
            for degrees in (-20.,0.,10.,30.,35.):
                self.assertFalse(self.p.heading_allowed(rotation(degrees).as_quat()))
            self.assertTrue(self.p.heading_allowed(rotation(20.).as_quat()))
            candidates=self.p.preinsert_rotations([0,0,0,1],self.parameters.orientation_policy())
            self.assertTrue(candidates)
            directed=self.p.directed_heading(candidates)
            self.assertTrue(np.all((directed>10)&(directed<30)))
            self.assertAlmostEqual(float(directed[0]),20.)
            tips=np.repeat(np.eye(4)[None,:,:],3,axis=0)
            tips[:,:3,:3]=rotation(np.array([20.,0.,20.])).as_matrix()
            for stage in ('insert','rotate','withdraw'):
                failure=self.p.heading_failure(tips,stage,stage,{})
                self.assertEqual(failure.diagnostic['batch_sample_index'],1)
            self.assertIsNone(self.p.heading_failure(tips,'approach','Approach',{}))
            for boundary in (10.,30.):
                tips[:,:3,:3]=rotation(boundary).as_matrix()
                self.assertIsNotNone(self.p.heading_failure(tips,'rotate','Rotate',{}))
        self.p.center_check=True
        self.assertIsNone(self.p.directed_heading_bounds())

    def test_preferred_heading_scores_without_changing_hard_envelope(self):
        self.p.planning_parameters=replace(self.parameters,left_sweep_hand_back_yaw_deg=-10.)
        self.p.sweep_direction='left'
        preferred=Rotation.from_euler('z',-10,degrees=True).as_quat()
        other=Rotation.from_euler('z',10,degrees=True).as_quat()
        with patch.object(TaskPlanner,'preinsert_score',return_value={'total':1.}):
            a=self.p.preinsert_score('left',{}, {},preferred,preferred)
            b=self.p.preinsert_score('left',{}, {},preferred,other)
        self.assertLess(a['total'],b['total'])
        self.assertTrue(self.p.heading_allowed(other))
        self.assertFalse(self.p.heading_allowed(Rotation.from_euler('z',-46,degrees=True).as_quat()))

    def test_both_directions_withdraw_from_actual_endpoint_not_center(self):
        # This geometry-only fixture does not create timed trajectories.
        self.p.planning_parameters=replace(self.parameters,rotate_velocity_factor=1.,withdraw_velocity_factor=1.)
        center=np.array([.3,.1,.01]);entry=center+[.08,0,.02]
        pre=np.eye(4);pre[:3,3]=entry+[0,0,.1]
        for direction,sign in (('left',1),('right',-1)):
            with self.subTest(direction=direction):
                calls=[];self.p.sweep_direction=direction
                def cartesian(side,q,points,nominal,wrist,ticket,label,curve,stage):
                    points=np.asarray(points);actual=np.eye(4);actual[:3,3]=points[-1]
                    if stage=='rotate':actual[:3,3]+=[.003,-.002,.001]
                    calls.append(dict(points=points,nominal=nominal,label=label,curve=curve,actual=actual))
                    return dict(end={'tip':actual},label=label)
                self.p.cartesian=cartesian;self.p.tip=lambda side,q,wrist:q['tip']
                stages=self.p.contact_segments('left',{'tip':pre},pre,entry,center,[0,0,0,1],np.eye(4),None)
                self.assertEqual([s['label'] for s in stages],['Insert',direction.title()+' 120','Withdraw'])
                expected=center+Rotation.from_euler('z',sign*120,degrees=True).apply(entry-center)
                np.testing.assert_allclose(calls[1]['points'][-1],expected)
                np.testing.assert_allclose(calls[1]['curve'].batch(calls[1]['points']),0.,atol=1e-12)
                opposite=center+Rotation.from_euler('z',-sign*60,degrees=True).apply(entry-center)
                self.assertGreater(calls[1]['curve'](opposite),.05)
                actual=calls[1]['actual'][:3,3]
                np.testing.assert_allclose(calls[2]['points'][:,:2],np.tile(actual[:2],(len(calls[2]['points']),1)))
                self.assertAlmostEqual(calls[2]['points'][-1,2]-actual[2],self.parameters.withdraw_height_mm/1000.)
                np.testing.assert_allclose(calls[1]['nominal'],[0,0,0,1])

    def test_return_uses_saved_above_table_not_current_start(self):
        self.p.ready_target={'arm':.8}
        self.p.contact_segments=Mock(side_effect=lambda *args,terminal:
                                    [{'end':{'arm':.4}}]+terminal({'arm':.4}))
        self.p.hand_transition=Mock(return_value={'end':{'arm':.4}})
        self.p.joint_path=Mock(return_value={'end':{'arm':.8}})
        self.p.ring_contact_path('left',{'arm':.1},None,None,None,None,None,None,[{'end':{'arm':.2}}],True)
        self.assertEqual(self.p.joint_path.call_args.args[2],{'arm':.8})
        self.assertEqual(self.p.hand_transition.call_args.args[2],'open')

    def test_path_and_target_position_tolerances_are_distinct(self):
        curve=lambda p:abs(p[1]);curve.target=np.array([1.,0.,0.])
        self.assertAlmostEqual(float(self.p.position_limit([.5,.007,0],curve)),.008)
        self.assertAlmostEqual(float(self.p.position_limit([1.,.009,0],curve)),.010)
        report=dict(checked_samples=0,max_position_error_m=0.,max_tilt_deg=0.,max_twist_deg=0.)
        tip=np.eye(4);tip[:3,3]=[1.,.009,0]
        args=([tip],[0,0,0,1],curve,self.parameters.orientation_policy(),'rotate','Left 120',dict(report))
        self.assertIsNone(self.p.check_tcp_geometry_batch(*args))
        tip[:3,3]=[.5,.009,0]
        self.assertIsNotNone(self.p.check_tcp_geometry_batch(*args))
        tip[:3,3]=[1.,.011,0]
        self.assertIsNotNone(self.p.check_tcp_geometry_batch(*args))

    def test_retimed_heading_is_checked_in_scalar_and_batch_paths(self):
        tip=np.eye(4);tip[:3,:3]=Rotation.from_euler('z',46,degrees=True).as_matrix()
        self.p.tip=Mock(return_value=tip)
        report=dict(checked_samples=0,max_position_error_m=0.,max_tilt_deg=0.,max_twist_deg=0.)
        args=([0,0,0,1],lambda p:0.,self.parameters.orientation_policy(),'rotate','Left 120',report)
        with self.assertRaisesRegex(PlanningFailure,'heading'):
            self.p.check_tcp_geometry('left',{},np.eye(4),*args)
        error=self.p.check_tcp_geometry_batch([tip],*args)
        self.assertEqual(error.diagnostic['batch_sample_index'],0)

    def test_position_search_can_rescue_nominal_ik_failure(self):
        import test_task_planner_contract as fixtures
        f=fixtures.PlannerContractTests();f.setUp();f.configure_ranking();p=f.p
        configure_planner(p,self.parameters)
        solved=dict(f.q);tip=np.eye(4)
        def solve(side,current,target,*args):
            if np.linalg.norm(target[:3,3])<.001:return None
            tip[:3,3]=target[:3,3];return solved
        p.solve=solve;p.tip=Mock(side_effect=lambda *args:tip.copy())
        with patch('task_motion_planner.orientation_candidates',return_value=[[0,0,0,1]]):
            p.cartesian('left',f.q,[[0,0,0]],[0,0,0,1],np.eye(4),f.ticket,'Insert',lambda p:0.,'insert')
        self.assertGreater(np.linalg.norm(tip[:3,3]),.002)
        self.assertLessEqual(np.linalg.norm(tip[:3,3]),self.parameters.target_tolerance_mm/1000.)


class TaskIdentityTests(unittest.TestCase):
    def fixture(self):
        import test_task_live_test as fixtures
        f=fixtures.LiveTestTests();f.setUp();self.addCleanup(f.doCleanups);return f

    def test_historical_ten_stages_cannot_be_loaded_for_execution(self):
        from task_saved_test import load_archive
        f=self.fixture();source=json.loads(f.source.read_text());source.pop('task_schema')
        source['segments']+=copy.deepcopy(source['segments'][:3])
        directory=f.directory/'plan';directory.mkdir();(directory/'plan.json').write_text(json.dumps(source))
        with patch('task_saved_test.verify',return_value={}):
            with self.assertRaisesRegex(ValueError,'retired'):load_archive(f.directory,f.urdf)

    def test_direction_change_invalidates_live_authorization(self):
        f=self.fixture();f.plan['sweep_direction']='right'
        with self.assertRaisesRegex(RuntimeError,'seven-stage'):f.authorize()

    def test_right_sweep_executes_five_arm_and_two_hand_actions(self):
        from task_plan_archive import digest
        f=self.fixture();source=json.loads(f.source.read_text())
        source['sweep_direction']='right';f.plan['sweep_direction']='right'
        source['segments'][3]['label']='Right 120';f.plan['segments'][3]['label']='Right 120'
        f.source.write_text(json.dumps(source));f.plan['live_review']['source_plan_sha256']=digest(f.source)
        node,arms,hands=f.run_full()
        self.assertEqual(len(arms),5);self.assertEqual(len(hands),2);node.stop.assert_not_called()

    def test_new_schema_is_required_and_direction_is_separate_from_arm(self):
        self.assertEqual(plan_labels({'task_schema':SCHEMA,'sweep_direction':'right','arm':'left'}),labels('right'))
        for plan in ({},{'task_schema':SCHEMA,'sweep_direction':'both'}):
            with self.assertRaises(ValueError):plan_labels(plan)


class MotionLimitTests(unittest.TestCase):
    def test_waist_acceleration_jerk_match_arm_without_limiting_arm_velocity(self):
        names=['waist_yaw_joint','left_elbow_joint']
        v,a=task_motion_limits(names,.65,.65);jerk=task_jerk_limits(names)
        self.assertAlmostEqual(v[names[0]],.2275);self.assertAlmostEqual(v[names[1]],.26)
        self.assertEqual(a,dict.fromkeys(names,.9750000000000001))
        self.assertEqual(jerk,dict.fromkeys(names,5.))

    def test_waist_curve_above_old_acceleration_is_accepted_under_new_bound(self):
        from test_task_trajectory import point
        path=S(joint_names=['waist_yaw_joint'],points=[point(0,0,0,0),point(.1,1.,0,0)])
        v,a=task_motion_limits(path.joint_names,1.,1.)
        stretch_to_limits(path,v,a,task_jerk_limits(path.joint_names))
        self.assertGreater(len(list(samples(path,v,a))),1)
        validate_waist_envelope(path,-.59,.59,.4,1.5)
        with self.assertRaises(ValueError):validate_waist_envelope(path,-.59,.59,.4,.5)


if __name__=='__main__':unittest.main()
