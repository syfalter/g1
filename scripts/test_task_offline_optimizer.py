import copy
import json
import os
from pathlib import Path
import tempfile
import threading
from types import SimpleNamespace as S
import unittest
from unittest.mock import Mock,patch
import xml.etree.ElementTree as ET

import numpy as np
from trajectory_msgs.msg import JointTrajectory,JointTrajectoryPoint

from task_composition_preview import stamp,section,ROOT,materialize
from task_offline_optimizer import Policy,local_path,optimize,retime,validate
from task_offline_optimizer_cases import source_units,sequence_for
from task_planning_config import PlanningParameters
from task_trajectory import task_jerk_limits,time_seconds


class OptimizerTests(unittest.TestCase):
    def setUp(self):
        tip=np.eye(4);tip[0,3]=1.
        self.model=S(limits={'joint':(-2.,2.)},robot=ET.fromstring('<robot/>'),joints={
            'right_index_tcp':('arm','fixed',tip,np.zeros(3),None,'tip',1.,0.),
            'arm':('root','revolute',np.eye(4),np.array([0.,0.,1.]),None,'joint',1.,0.)})
        self.source=JointTrajectory(joint_names=['joint'],points=[stamp(JointTrajectoryPoint(
            positions=[q],velocities=[0.],accelerations=[0.]),t) for t,q in ((0.,0.),(2.,.15),(4.,.3))])

    def test_policy_and_role_validation(self):
        for p in (Policy(jerk=0),Policy(jerk=float('nan')),Policy(contact_mm=2,transit_mm=1)):
            with self.assertRaises(ValueError):p.validate()
        for ranges in ([],[(0.,3.,'operation')],[(0.,2.,'unknown'),(2.,4.,'operation')],[(0.,2.,'transit'),(2.1,4.,'operation')]):
            with self.assertRaises(ValueError):local_path(self.source,self.model,'right',ranges,Policy(),None,lambda:None)

    def test_near_boundary_section_has_strict_nanosecond_knots(self):
        tr=section(self.source,0.,2.+1e-15)
        self.assertEqual(len(tr.points),2)
        self.assertTrue(np.all(np.diff([time_seconds(p) for p in tr.points])>0))
        with self.assertRaises(ValueError):section(self.source,2.,2.+1e-15)

    def test_local_replacements_preserve_source_and_business_ends(self):
        before=copy.deepcopy(self.source)
        with patch('task_trajectory_speed.motion_limits',return_value=({'joint':.4},{'joint':1.5})):
            candidate,report=local_path(self.source,self.model,'right',[(0.,2.,'transit'),(2.,4.,'operation')],Policy(),None,lambda:None)
        self.assertEqual(before,self.source)
        self.assertTrue(report['valley_replacements'])
        for role,mm in (('transit',5.),('operation',1.)):
            self.assertLessEqual(report['by_role'][role]['tcp_m'],mm*.001)
        for i in (0,-1):
            for field in ('positions','velocities','accelerations'):
                np.testing.assert_allclose(getattr(candidate.points[i],field),getattr(before.points[i],field),atol=1e-10)

    def test_nonfinite_derivatives_rejected(self):
        self.source.points[1].velocities=[float('nan')]
        with self.assertRaises(ValueError):validate(self.source,self.model,None,5.)

    def test_timing_roundoff_failure_has_verified_fallback(self):
        with patch('task_trajectory_speed.motion_limits',return_value=({'joint':.4},{'joint':1.5})),patch(
                'task_fast_timing.fastest',side_effect=ValueError('Rounded local timing exceeds motion limits')):
            result,report=retime(self.source,self.model,None,5.,lambda:None)
            self.assertLessEqual(report['peaks']['jerk'],5.)
            self.assertTrue(all(np.isfinite(p.positions).all() for p in result.points))

    def test_experimental_jerk_does_not_change_production_policy(self):
        before=copy.deepcopy(self.source)
        with patch('task_trajectory_speed.motion_limits',return_value=({'joint':.4},{'joint':1.5})):
            result,report=retime(self.source,self.model,None,10.,lambda:None,baseline=self.source)
            self.assertLessEqual(report['peaks']['jerk'],10.)
            self.assertLessEqual(time_seconds(result.points[-1]),4.)
        self.assertEqual(before,self.source)
        self.assertEqual(task_jerk_limits(['joint']),{'joint':5.})
        self.assertFalse(report['hardware_executable'])

    def test_slower_geometry_rejected_and_cache_is_policy_specific(self):
        from task_trajectory_speed import rescale
        params=PlanningParameters();ranges=[(0.,4.,'operation')]
        with tempfile.TemporaryDirectory() as temp:
            with patch('task_offline_optimizer.retime',side_effect=lambda source,*args,**kw:(source,{})),patch(
                    'task_offline_optimizer.local_path',return_value=(rescale(self.source,.5),{'method':'test'})) as fitter,patch(
                    'task_offline_optimizer.smooth_candidate',side_effect=ValueError('test rejected')):
                a,hit=optimize(self.source,self.model,'right',ranges,Policy(),params,directory=temp)
                self.assertFalse(hit);self.assertFalse(a['geometry_selected'])
                self.assertEqual(a['variants']['candidate']['seconds'],a['variants']['baseline']['seconds'])
                b,hit=optimize(self.source,self.model,'right',ranges,Policy(),params,directory=temp)
                self.assertTrue(hit);self.assertEqual(a,b);self.assertEqual(fitter.call_count,1)
                optimize(self.source,self.model,'right',ranges,Policy(transit_mm=6.),params,directory=temp)
                self.assertEqual(fitter.call_count,2)

    def test_single_approach_annotations_do_not_spread_to_contacts(self):
        config=dict(side='right',traces={'push':{}})
        with patch('task_offline_optimizer_cases.fit',return_value=self.source):
            units,_,_=source_units('rail',config,{},None,dict(slot='push',transitInterval=(1.,2.)),lambda:None)
            self.assertEqual(units['push'][1],[(0.,1.,'operation'),(1.,2.,'transit'),(2.,4.,'operation')])
            units,_,_=source_units('rail',config,{},None,dict(slot='push'),lambda:None)
            self.assertEqual(units['push'][1],[(0.,4.,'operation')])


class ViewerTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from PyQt5 import QtWidgets
        cls.app=QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    def test_offline_viewer_keeps_all_three_variants(self):
        from task_composition_panel import CompositionPanel
        node=S(demo=S(config={'traces':{}}),display_pub=Mock(),demo_preview_pub=Mock(),status='')
        panel=CompositionPanel(node,viewer_only=True)
        try:
            panel.refresh(False)
            self.assertEqual([panel.variant.itemData(i) for i in range(panel.variant.count())],['original','baseline','candidate'])
            self.assertTrue(panel.generate.isHidden())
        finally:panel.timer.stop();panel.close()

    def test_optimizer_rejects_nonoffline_request(self):
        from task_offline_optimizer_panel import OptimizerPanel
        control=S(config={'traces':{}},revision=0)
        node=S(demo=control,landing_gear=control,rail=control,args=S(disconnected=False,offline_editor=False),
            display_pub=Mock(),demo_preview_pub=Mock(),status='')
        panel=OptimizerPanel(node)
        try:
            panel.build();self.assertIn('只允许离线',node.status);self.assertFalse(panel.active)
        finally:panel.viewer.timer.stop();panel.close()

    @unittest.skipUnless(os.environ.get('G1_OFFLINE_TEST_URDF'),'Set G1_OFFLINE_TEST_URDF')
    def test_actual_optimizer_controls_generate_cached_comparison(self):
        from PyQt5 import QtWidgets
        from task_offline_optimizer_panel import OptimizerPanel
        from task_trace_selection import restore
        from workbench_model import OfflineModel
        controls={}
        for name,filename in (('demo','dring_demonstration.json'),('landing_gear','landing_gear_demonstration.json')):
            path=ROOT/'data/task_panel'/filename
            controls[name]=S(config=restore(json.loads(path.read_text()),path),revision=0)
        node=S(**controls,rail=S(config={'traces':{}},revision=0),model=OfflineModel(Path(os.environ['G1_OFFLINE_TEST_URDF'])),
            args=S(disconnected=True,offline_editor=True),lock=threading.RLock(),shutdown_requested=False,
            prepared=json.loads((ROOT/'data/task_panel/ready_poses.json').read_text()),
            display_pub=Mock(),demo_preview_pub=Mock(),status='',launch_work=lambda fn:fn())
        import tempfile
        temporary=tempfile.TemporaryDirectory();self.addCleanup(temporary.cleanup)
        panel=OptimizerPanel(node,snapshot_directory=Path(temporary.name));scroll=QtWidgets.QScrollArea();scroll.setWidgetResizable(True);scroll.setWidget(panel)
        scroll.resize(900,900);scroll.show()
        try:
            panel.case.setCurrentIndex(1);panel.build();self.app.processEvents();panel.refresh(False)
            self.assertIsNotNone(panel.viewer.result,panel.viewer.summary.text())
            self.assertFalse(panel.active)
            panel.build();self.app.processEvents()
            self.assertTrue(panel.viewer.result['hit'])
            self.assertEqual(panel.viewer.variant.currentData(),'flow')
            self.assertIn('jerk',panel.viewer.result)
            self.assertEqual(panel.viewer.result['payload']['workflow'],'landing_gear')
            self.assertEqual(len(panel.viewer.variants),5)
            output=ROOT/'data/trajectory_edit_audit/screenshots/offline_optimizer_controls_v3.png'
            self.assertTrue(scroll.grab().save(str(output)))
            panel.rate.setValue(.5);self.assertIsNone(panel.viewer.result)
            self.assertEqual(node.landing_gear.revision,0)
        finally:panel.halt();panel.viewer.timer.stop();scroll.close()

    @unittest.skipUnless(os.environ.get('G1_OFFLINE_TEST_URDF'),'Set G1_OFFLINE_TEST_URDF')
    def test_actual_candidate_switch_plot_and_one_shot_playback(self):
        from task_composition_panel import CompositionPanel,preview_samples
        from task_composition_smoothing import evaluate
        from workbench_model import OfflineModel
        model=OfflineModel(Path(os.environ['G1_OFFLINE_TEST_URDF']))
        node=S(demo=S(config={'traces':{}}),model=model,display_pub=Mock(),demo_preview_pub=Mock(),status='')
        panel=CompositionPanel(node,viewer_only=True);panel.resize(850,950);panel.show()
        variants={}
        try:
            for variant in ('original','baseline','candidate','smooth'):
                path=ROOT/'data/composition_candidates'/f'landing_gear-combination-jerk5-contact1-transit5-v4-extra0.15-{variant}.json'
                artifact=json.loads(path.read_text());payload=artifact['units'];sequence=artifact['sequence']
                tr,events=materialize(payload,sequence)
                times,tcp,joint=preview_samples(tr,payload,events,model,'right')
                variants[variant]=dict(payload=payload,sequence=sequence,trajectory=tr,events=events,
                    times=times,tcp=tcp,joint=joint,hit=True,elapsed=.1,cache='offline_optimizer_cache',
                    joint_names=list(tr.joint_names),joint_matrix=evaluate(tr,times,1),
                    acceleration_matrix=evaluate(tr,times,2),jerk_matrix=evaluate(tr,times,3),
                    acceleration=np.max(abs(evaluate(tr,times,2)),axis=1),jerk=np.max(abs(evaluate(tr,times,3)),axis=1))
            panel.accept(dict(variants=variants));panel.refresh(False)
            self.assertEqual(panel.variant.currentData(),'smooth')
            for index in range(4):
                panel.variant.setCurrentIndex(index)
                self.assertIs(panel.result,variants[panel.variant.currentData()])
                for metric in range(4):
                    panel.metric.setCurrentIndex(metric)
                    self.assertTrue(np.isfinite(panel.plot.samples[1]).all())
            panel.variant.setCurrentIndex(panel.variant.findData('smooth'))
            panel.metric.setCurrentIndex(2);panel.joint_axis.setCurrentIndex(1)
            np.testing.assert_allclose(panel.plot.samples[1],panel.result['acceleration_matrix'][:,0])
            panel.toggle();node.display_pub.publish.reset_mock();panel.last_tick-=.1;panel.tick()
            self.assertGreater(panel.position,0.);node.display_pub.publish.assert_not_called()
            panel.toggle();self.assertFalse(panel.playing)
            self.app.processEvents()
            output=ROOT/'data/trajectory_edit_audit/screenshots/offline_optimizer_v4.png'
            output.parent.mkdir(parents=True,exist_ok=True);self.assertTrue(panel.grab().save(str(output)))
            for start in ('LOWER','UPPER'):
                for finish in ('LOWER','UPPER'):
                    for leave in (False,True):
                        payload=variants['candidate']['payload']
                        tr,events=materialize(payload,sequence_for(payload,3,start,finish,leave))
                        self.assertTrue(np.all(np.diff([time_seconds(p) for p in tr.points])>0))
                        self.assertEqual([e['posture'] for e in events],['open','grip']+(['open'] if leave else []))
        finally:panel.halt();panel.timer.stop();panel.close()


if __name__=='__main__':unittest.main()
