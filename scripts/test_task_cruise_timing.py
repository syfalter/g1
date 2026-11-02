"""Offline arc-clock, limit, and mode integration tests; no hardware output."""
import copy
import unittest
import xml.etree.ElementTree as ET
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np

from task_cruise_timing import retime, progress
from task_path_smoothing import joint_line_trajectory
from task_trajectory import coefficients,time_seconds,stretch_to_limits


class Model:
    limits={'a':(-3.,3.)}
    robot=ET.fromstring('<robot><joint name="a"><limit velocity="2"/></joint></robot>')
    rotation_only=False

    def batch_transforms(self,links,context,names,rows):
        q=np.asarray(rows)[:,0]
        pose=np.tile(np.eye(4),(len(q),1,1))
        if self.rotation_only:
            pose[:,0,0]=pose[:,1,1]=np.cos(q)
            pose[:,1,0]=np.sin(q);pose[:,0,1]=-np.sin(q)
        else:
            pose[:,0,3]=.2*np.sin(q);pose[:,1,3]=.2*np.cos(q)
        return {links[0]:pose}


def evaluate(tr,seconds,order=0):
    t=np.array([time_seconds(p) for p in tr.points])
    rows=[]
    for s in seconds:
        i=min(len(t)-2,max(0,np.searchsorted(t,s,side='right')-1));dt=t[i+1]-t[i]
        c=coefficients(tr.points[i],tr.points[i+1],dt)
        rows.append(np.polynomial.polynomial.polyval((s-t[i])/dt,
                    np.polynomial.polynomial.polyder(c,order))/dt**order)
    return np.asarray(rows)


class CruiseTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.model=Model()
        from task_trajectory_speed import rescale
        cls.source=rescale(joint_line_trajectory(['a'],[0.],[.8]),.125)
        cls.result,cls.report=retime(cls.source,cls.model,{},'left',1.)

    def test_cruise_is_uniform_on_circle(self):
        duration=time_seconds(self.result.points[-1])
        speed=evaluate(self.result,np.linspace(.2,.8,401)*duration,1)[:,0]
        self.assertLess(np.std(speed)/np.mean(speed),.01)
        self.assertGreater(np.min(speed),0.)
        for p in (self.result.points[0],self.result.points[-1]):
            self.assertLess(np.max(np.abs(p.velocities)),1e-8)
            self.assertLess(np.max(np.abs(p.accelerations)),1e-7)

    def test_certified_path_endpoints_and_limits(self):
        self.assertLessEqual(self.report['cruise']['max_joint_path_difference_rad'],1e-7)
        np.testing.assert_allclose(self.result.points[0].positions,self.source.points[0].positions,atol=1e-10)
        np.testing.assert_allclose(self.result.points[-1].positions,self.source.points[-1].positions,atol=1e-10)
        clone=copy.deepcopy(self.result)
        self.assertEqual(stretch_to_limits(clone,{'a':1.},{'a':1.5},{'a':5.}),1.)
        from task_trajectory_speed import rescale
        self.assertEqual(self.source,rescale(joint_line_trajectory(['a'],[0.],[.8]),.125))

    def test_unconstrained_multiplier_preserves_clock(self):
        slower,report=retime(self.source,self.model,{},'left',.5)
        self.assertAlmostEqual(report['applied_multiplier'],.5)
        self.assertAlmostEqual(time_seconds(slower.points[-1]),2*time_seconds(self.result.points[-1]),places=7)
        for a,b in zip(slower.points,self.result.points):
            np.testing.assert_allclose(a.positions,b.positions,atol=1e-9)
            np.testing.assert_allclose(a.velocities,np.asarray(b.velocities)*.5,atol=1e-8)

    def test_rotation_only_and_stationary(self):
        model=Model();model.rotation_only=True
        tr,report=retime(self.source,model,{},'left',1.)
        self.assertEqual(report['cruise']['path_length_m'],0.)
        self.assertGreater(time_seconds(tr.points[-1]),0.)
        still=joint_line_trajectory(['a'],[.1],[.1])
        result,report=retime(still,model,{},'left',1.)
        self.assertTrue(report['cruise']['preserved_stationary'])
        np.testing.assert_allclose([p.positions for p in result.points],.1,atol=1e-10)

    def test_original_policy_duration_guard_and_visible_fallback(self):
        from unittest.mock import patch
        from task_trajectory_speed import scale_arm_segments,rescale
        from task_cruise_timing import summary
        for policy in ('recorded','local','uniform'):
            baseline,baseline_report=scale_arm_segments([dict(trajectory=self.source)],1.,self.model,policy=policy)
            expected=baseline[0]['trajectory']
            slow=rescale(expected,.5)
            with patch('task_cruise_timing._retime',return_value=(slow,{})):
                result,report=retime(self.source,self.model,{},'left',1.,original_policy=policy)
            self.assertEqual(result,expected)
            self.assertTrue(report['timing_comparison']['fallback'])
            self.assertEqual(report['timing_policy'],baseline_report['timing_policy'])
            self.assertIn('采用原策略',summary(report))
        with patch('task_cruise_timing._retime',side_effect=ValueError('not converged')):
            result,report=retime(self.source,self.model,{},'left',1.,original_policy='recorded')
            self.assertEqual(report['timing_comparison']['cruise_error'],'not converged')
        with patch('task_cruise_timing._retime',side_effect=RuntimeError('cancel')):
            with self.assertRaisesRegex(RuntimeError,'cancel'):
                retime(self.source,self.model,{},'left',1.,original_policy='recorded')

    def test_local_jerk_bottleneck_does_not_slow_distant_cruise(self):
        from trajectory_msgs.msg import JointTrajectory,JointTrajectoryPoint
        from unittest.mock import patch
        model=Model();model.limits={'a':(-3.,3.),'b':(-3.,3.)}
        model.robot=ET.fromstring('<robot><joint name="a"><limit velocity="2"/></joint>'
                                 '<joint name="b"><limit velocity="2"/></joint></robot>')
        def path(bump):
            curve=JointTrajectory(joint_names=['a','b'])
            shape=np.polynomial.polynomial.polypow([1.,0.,-1.],4)
            for t in np.linspace(0.,20.,201):
                u=t/20.;x=(t-10.)/.3
                qa=.8*u**3*(10-15*u+6*u*u)
                va=.8*30*u*u*(1-u)**2/20
                aa=.8*60*u*(1-u)*(1-2*u)/400
                qb,vb,ab=[(.03*np.polynomial.polynomial.polyval(x,
                    np.polynomial.polynomial.polyder(shape,k))/.3**k if bump and abs(x)<1 else 0.)
                    for k in range(3)]
                point=JointTrajectoryPoint(positions=[qa,qb],velocities=[va,vb],accelerations=[aa,ab])
                point.time_from_start.sec,point.time_from_start.nanosec=divmod(round(t*1e9),10**9)
                curve.points.append(point)
            return curve
        with (patch('task_trajectory.stretch_to_limits',side_effect=AssertionError('global stretch forbidden')),
                patch('task_trajectory_speed.scale_arm_segments',side_effect=AssertionError('global clip forbidden'))):
            clean,_=retime(path(False),model,{},'left',1.)
            limited,report=retime(path(True),model,{},'left',1.)
        def duration(curve,a,b):
            positions=np.array([p.positions[0] for p in curve.points])
            times=np.array([time_seconds(p) for p in curve.points])
            return float(np.interp(b,positions,times)-np.interp(a,positions,times))
        for a,b in ((.02,.08),(.72,.78)):
            self.assertLess(duration(limited,a,b)/duration(clean,a,b),1.05)
        self.assertGreater(duration(limited,.38,.42),duration(clean,.38,.42))
        self.assertEqual(report['cruise']['uniform_limit_stretch'],1.)
        self.assertEqual(report['cruise']['baseline_limit_stretch'],1.)
        self.assertEqual(report['timing_policy'],'cruise_local_v2')
        from task_trajectory_speed import restore_baseline
        self.assertEqual(restore_baseline([dict(trajectory=limited)],report)[0]['trajectory'],path(True))
        # At aggressive rates the heuristic can regress; production must select
        # the original-policy result instead of exporting that slower candidate.
        faster,fast_report=retime(path(True),model,{},'left',2.,original_policy='recorded')
        comparison=fast_report['timing_comparison']
        self.assertLessEqual(time_seconds(faster.points[-1]),comparison['original_seconds']+1e-9)

    def test_batch_composition_matches_existing_certified_utility(self):
        from task_cruise_timing import compose_batch
        from task_local_timing import _compose
        random=np.random.default_rng(123)
        polynomials=random.normal(size=(4,6,3));warps=random.normal(size=(4,6))
        actual=compose_batch(polynomials,warps)
        for i in range(4):np.testing.assert_allclose(actual[i],_compose(polynomials[i],warps[i]),atol=1e-10)

    def test_cancel_and_invalid_input(self):
        with self.assertRaisesRegex(RuntimeError,'cancel'):
            retime(self.source,self.model,{},'left',1.,check=Mock(side_effect=RuntimeError('cancel')))
        source=copy.deepcopy(self.source);source.points[0].velocities=[1.]
        with self.assertRaises(ValueError):retime(source,self.model,{},'left',1.)
        with self.assertRaises(ValueError):retime(self.source,self.model,{},'left',float('nan'))

    def test_mode_invalidates_plan_without_mutating_versions(self):
        from task_trace_editor_control import TraceEditorControl
        import threading
        c=TraceEditorControl();c.node=SimpleNamespace(lock=threading.RLock())
        c.idle=Mock();c.invalidate=Mock()
        c.set_generation_mode('original');c.invalidate.assert_not_called()
        c.set_generation_mode('cruise');c.invalidate.assert_called_once()
        self.assertEqual(c.generation_mode,'cruise')
        with self.assertRaises(ValueError):c.set_generation_mode('bad')

    def test_rail_parameter_round_trip(self):
        from dataclasses import asdict
        from task_planning_config import PlanningParameters,rail_panel_parameters,parameters_from_values
        params=PlanningParameters();record=dict(values=asdict(params))
        changed,_=rail_panel_parameters(params,record,dict(rail_replay_cruise=True))
        self.assertTrue(parameters_from_values(asdict(changed)).rail_replay_cruise)
        self.assertFalse(params.rail_replay_cruise)

    def test_new_mode_build_cache_and_preview_share_final_timeline(self):
        from test_task_dring_demo import fixture
        from task_dring_cache import cache_key
        from task_dring_teach_data import build,preview
        from task_planning_config import PlanningParameters
        from unittest.mock import patch
        import tempfile
        model,start,config=fixture()
        model.batch_transforms=Model().batch_transforms
        params=PlanningParameters();trace=config['traces']['left']
        original=copy.deepcopy(config)
        old_key=cache_key(trace,model,'left',1.,.7,.7,params)
        new_key=cache_key(trace,model,'left',1.,.7,.7,params,timing_mode='cruise')
        self.assertNotEqual(old_key,new_key)
        with tempfile.TemporaryDirectory() as directory:
            plan=build(model,config,start,'left',params,.7,.7,1.,timing_mode='cruise',cache_dir=directory)
            self.assertEqual(plan['speed_scaling']['generation_mode'],'cruise')
            self.assertAlmostEqual(time_seconds(preview(plan).points[-1]),
                                   sum(s['duration'] for s in plan['segments']))
            with patch('task_cruise_timing.retime',side_effect=AssertionError('unexpected recomputation')):
                again=build(model,config,start,'left',params,.7,.7,1.,timing_mode='cruise',cache_dir=directory)
            self.assertTrue(again['speed_scaling']['replay_cache']['hit'])
            self.assertEqual(again['segments'][-1]['trajectory'],plan['segments'][-1]['trajectory'])
        self.assertEqual(config,original)

    def test_rail_cache_separates_hand_context_and_modes(self):
        from task_rail_cache import key
        from task_planning_config import PlanningParameters
        from test_task_dring_demo import fixture
        from dataclasses import replace
        model,_,config=fixture();trace=config['traces']['left'];params=PlanningParameters()
        old=key(trace,model,params)
        new=replace(params,rail_replay_cruise=True)
        self.assertNotEqual(old,key(trace,model,new))
        self.assertNotEqual(key(trace,model,new,context={'finger':0.}),
                            key(trace,model,new,context={'finger':1.}))
        self.assertEqual(old,key(trace,model,params,context={'finger':1.}))

    def test_full_cycle_fallback_preserves_original_segments_and_baseline(self):
        from unittest.mock import patch
        from test_task_dring_demo import fixture
        from task_dring_teach_data import build,fit_trace
        from task_planning_config import PlanningParameters
        from task_trajectory_speed import restore_baseline
        model,start,config=fixture();params=PlanningParameters()
        baseline=build(model,config,start,'left',params,.7,.7,1.,full_cycle=True,
            preparation=config['ready']['joints'])
        with patch('task_cruise_timing._retime',side_effect=ValueError('not converged')):
            plan=build(model,config,start,'left',params,.7,.7,1.,full_cycle=True,timing_mode='cruise',
                preparation=config['ready']['joints'])
        self.assertEqual(plan['segments'],baseline['segments'])
        restored=restore_baseline(plan['segments'],plan['speed_scaling'])
        source,_=fit_trace(config['traces']['left'],model,'left')
        for old,new in zip(plan['segments'],restored):
            expected=source if 'recording_fit' in old else old['trajectory']
            self.assertEqual(new['trajectory'],expected)

    def test_original_hand_timeline_never_silently_retimed(self):
        from test_task_dring_demo import fixture
        from task_dring_teach_data import build
        from task_planning_config import PlanningParameters
        model,start,config=fixture(True)
        with self.assertRaisesRegex(ValueError,'同步手指'):
            build(model,config,start,'left',PlanningParameters(),.7,.7,1.,timing_mode='cruise')

    def test_offline_preview_does_not_authorize_or_persist(self):
        import threading
        from test_task_dring_demo import fixture
        from task_trace_editor_control import TraceEditorControl
        model,_,config=fixture();model.batch_transforms=Model().batch_transforms
        control=TraceEditorControl();control.config=config;control.revision=0
        control.workflow='dring_demo';control.idle=Mock()
        control.node=SimpleNamespace(lock=threading.RLock(),session=SimpleNamespace(current=None,generation=0),
            model=model,args=SimpleNamespace(),shutdown_requested=False,display_pub=Mock(),demo_preview_pub=Mock(),
            launch_work=lambda work:work())
        original=copy.deepcopy(config)
        control.preview_timing('left','cruise',1.)
        control.node.display_pub.publish.assert_called_once()
        self.assertIn('仅预览',control.editor_feedback)
        self.assertEqual(config,original)
        self.assertFalse(hasattr(control.node.session,'plan'))

    def test_generation_mode_restores_to_control_as_well_as_widget(self):
        from PyQt5 import QtWidgets
        from task_editor_settings import values,restore_values
        app=QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
        combo=QtWidgets.QComboBox();combo.addItem('old','original');combo.addItem('new','cruise')
        editor=SimpleNamespace(generation_mode=combo,control=SimpleNamespace(),schedule_markers=Mock())
        restore_values(editor,dict(generation_mode='cruise'))
        self.assertEqual(editor.control.generation_mode,'cruise')
        self.assertEqual(values(editor)['generation_mode'],'cruise')


if __name__=='__main__':unittest.main()
