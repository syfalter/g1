import json
import os
import xml.etree.ElementTree as ET
from pathlib import Path
from types import SimpleNamespace as S
import unittest
from unittest.mock import Mock

import numpy as np
from PyQt5 import QtWidgets, QtCore

from task_composition_panel import (CompositionPanel,sample_rows,hand_values,preview_samples,
                                    visible_joint_values,tcp_velocity,native_playback_message)
from task_composition_preview import materialize,ROOT
from task_trajectory import time_seconds


class PanelTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app=QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    def test_hand_transition_matches_hold_schedule(self):
        payload={'handPresets':{'open':{'feedback_q':[0.]*6},'grip':{'feedback_q':[1.]*6}}}
        events=[dict(time=0.,posture='open'),dict(time=2.,posture='grip'),dict(time=5.,posture='open')]
        times=np.array([0.,1.,2.,2.5,3.,4.,5.,5.5,6.])
        before=json.dumps(payload)
        rows=hand_values(payload,events,times)
        np.testing.assert_allclose(rows[:,0],[0.,0.,0.,.5,1.,1.,1.,.5,0.])
        self.assertEqual(json.dumps(payload),before)

    def test_fixed_and_unknown_joints_are_not_sent_to_rviz(self):
        model=S(robot=ET.fromstring('<robot><joint name="waist_roll_joint" type="fixed"/>'
            '<joint name="arm" type="revolute"/><joint name="finger" type="continuous"/></robot>'))
        result=visible_joint_values(model,{'waist_roll_joint':0.,'arm':.5,'finger':.2,'unknown':1.})
        self.assertEqual(result,{'arm':.5,'finger':.2})

    def test_fastest_rates_are_bounded_and_invalidate_preview(self):
        node=S(demo=S(config={'traces':{}}),display_pub=Mock(),demo_preview_pub=Mock(),status='')
        panel=CompositionPanel(node)
        try:
            self.assertEqual(panel.mode.currentData(),'fastest')
            self.assertEqual(panel.rate.maximum(),1.)
            self.assertTrue(panel.transit_rate.isEnabled())
            panel.result={'test':True};panel.transit_rate.setValue(.5)
            self.assertIsNone(panel.result)
            panel.mode.setCurrentIndex(0)
            self.assertFalse(panel.transit_rate.isEnabled())
            panel.rate.setValue(4.);panel.mode.setCurrentIndex(2)
            self.assertEqual(panel.rate.value(),1.)
        finally:panel.timer.stop();panel.close()

    def test_gear_exposes_previous_candidate_without_changing_dring(self):
        node=S(demo=S(config={'traces':{}}),display_pub=Mock(),demo_preview_pub=Mock(),status='')
        panel=CompositionPanel(node)
        try:
            self.assertEqual(panel.variant.findData('baseline'),-1)
            panel.workflow.setCurrentIndex(1)
            self.assertGreaterEqual(panel.variant.findData('baseline'),0)
            self.assertIn('V2',panel.variant.itemText(1))
            panel.workflow.setCurrentIndex(0)
            self.assertEqual(panel.variant.findData('baseline'),-1)
        finally:panel.timer.stop();panel.close()

    def test_actual_fastest_candidate_render(self):
        urdf=os.environ.get('G1_OFFLINE_TEST_URDF')
        if not urdf:self.skipTest('Set G1_OFFLINE_TEST_URDF for model-based preview')
        from workbench_model import OfflineModel
        from task_trace_selection import restore
        model=OfflineModel(Path(urdf))
        for workflow in ('dring_demo','landing_gear'):
            source=ROOT/'data/composition_candidates'/(workflow+'-fastest-preview.json')
            if not source.exists():self.fail('Generate actual fastest candidates first')
            candidate=json.loads(source.read_text());payload=candidate['units']
            trajectory,events=materialize(payload,candidate['sequence'])
            configs={}
            for key,filename in (('demo','dring_demonstration.json'),('landing_gear','landing_gear_demonstration.json')):
                path=ROOT/'data/task_panel'/filename
                configs[key]=S(config=restore(json.loads(path.read_text()),path),revision=0)
            node=S(**configs,model=model,display_pub=Mock(),demo_preview_pub=Mock(),status='')
            panel=CompositionPanel(node);panel.resize(880,1120);panel.show()
            try:
                if workflow=='landing_gear':panel.workflow.setCurrentIndex(1);panel.leave.setChecked(True)
                panel.count.setValue(3)
                side=configs['demo' if workflow=='dring_demo' else 'landing_gear'].config['side']
                times,tcp,joint=preview_samples(trajectory,payload,events,model,side)
                self.assertTrue(np.isfinite(tcp).all());self.assertGreater(max(tcp),0.)
                panel.accept(dict(payload=payload,trajectory=trajectory,sequence=candidate['sequence'],events=events,
                    cache=candidate['cache'],hit=True,elapsed=.5,times=times,tcp=tcp,joint=joint))
                smooth_path=ROOT/'data/composition_candidates'/(workflow+'-fastest-smooth-preview.json')
                if smooth_path.exists():
                    saved=json.loads(smooth_path.read_text());smooth=saved['units']
                    tr,ev=materialize(smooth,saved['sequence']);ts,vs,js=preview_samples(tr,smooth,ev,model,side)
                    panel.accept(dict(variants={'original':panel.result,'candidate':dict(payload=smooth,
                        trajectory=tr,sequence=saved['sequence'],events=ev,cache=saved['cache'],hit=True,
                        elapsed=.5,times=ts,tcp=vs,joint=js)}))
                    self.assertIn('连续偏差上界',panel.comparison.text())
                    panel.variant.setCurrentIndex(0)
                    self.assertAlmostEqual(panel.duration(),time_seconds(trajectory.points[-1]))
                    panel.variant.setCurrentIndex(1)
                    if workflow=='landing_gear':
                        baseline=panel.result;original=panel.variants['original']
                        saved=json.loads((ROOT/'data/composition_candidates/landing_gear-fastest-smooth-valleys-v2-preview.json').read_text())
                        revised=saved['units'];tr,ev=materialize(revised,saved['sequence'])
                        ts,vs,js=preview_samples(tr,revised,ev,model,side)
                        panel.accept(dict(variants={'original':original,'baseline':baseline,'candidate':dict(
                            payload=revised,trajectory=tr,sequence=saved['sequence'],events=ev,cache=saved['cache'],
                            hit=True,elapsed=.5,times=ts,tcp=vs,joint=js)}))
                        panel.variant.setCurrentIndex(panel.variant.findData('baseline'))
                        self.assertAlmostEqual(panel.duration(),time_seconds(baseline['trajectory'].points[-1]))
                        panel.variant.setCurrentIndex(1)
                        self.assertAlmostEqual(panel.duration(),time_seconds(tr.points[-1]))
                self.app.processEvents();panel.seek(panel.duration()*.4)
                message=node.demo_preview_pub.publish.call_args.args[0]
                self.assertNotIn('waist_roll_joint',message.state.joint_state.name)
                self.assertNotIn('waist_pitch_joint',message.state.joint_state.name)
                node.demo_preview_pub.publish.reset_mock()
                panel.toggle();native=node.display_pub.publish.call_args.args[0]
                self.assertNotIn('waist_roll_joint',native.trajectory_start.joint_state.name)
                for _ in range(5):panel.last_tick-=.1;panel.tick()
                self.assertEqual(node.demo_preview_pub.publish.call_count,1)  # Hide only, no per-frame pose flood.
                panel.toggle()
                self.assertEqual(panel.table.columnCount(),5)
                self.assertTrue(panel.grab().save(str(ROOT/'data/composition_candidates'/(workflow+'-fastest-panel.png'))))
            finally:panel.halt();panel.timer.stop();panel.close()

    def test_actual_cached_candidate_controls_and_render(self):
        source=ROOT/'data/composition_candidates/dring_demo-cruise-preview.json'
        if not source.exists():self.skipTest('Run the real-data offline composition check first')
        candidate=json.loads(source.read_text());payload=candidate['units']
        trajectory,events=materialize(payload,candidate['sequence'])
        config={'traces':{'left_5':{},'right_4':{}}}
        robot=ET.Element('robot')
        for name in payload['context']:ET.SubElement(robot,'joint',name=name,type='revolute')
        node=S(demo=S(config=config),model=S(robot=robot),display_pub=Mock(),demo_preview_pub=Mock(),status='')
        panel=CompositionPanel(node);panel.resize(800,1080);panel.show()
        try:
            times=np.linspace(0.,time_seconds(trajectory.points[-1]),400)
            _,speeds=sample_rows(trajectory,times)
            panel.accept(dict(payload=payload,trajectory=trajectory,sequence=candidate['sequence'],events=events,
                              cache=candidate['cache'],hit=True,elapsed=.5,times=times,tcp=speeds*100,joint=speeds))
            self.app.processEvents()
            panel.seek(panel.duration()*.6)
            self.assertGreater(panel.slider.value(),5900)
            self.assertTrue(node.demo_preview_pub.publish.called)
            panel.toggle();published=node.display_pub.publish.call_count
            native=node.display_pub.publish.call_args.args[0].trajectory[0].joint_trajectory
            self.assertEqual(time_seconds(native.points[0]),0.)
            self.assertAlmostEqual(time_seconds(native.points[-1]),panel.duration()-panel.position,places=7)
            panel.last_tick-=.1;before=panel.position;panel.tick()
            self.assertGreater(panel.position,before)
            self.assertEqual(node.display_pub.publish.call_count,published)
            panel.toggle();panel.metric.setCurrentIndex(1);self.app.processEvents()
            self.assertEqual(panel.plot.label,'最大关节速度 rad/s')
            screenshot=ROOT/'data/composition_candidates/composition-panel.png'
            self.assertTrue(panel.grab().save(str(screenshot)))
            panel.count.setValue(4);self.assertIsNone(panel.result)
            panel.refresh(False);self.assertFalse(panel.play.isEnabled())
        finally:
            panel.halt();panel.timer.stop();panel.close()

    def test_native_hand_playback_does_not_mutate_candidate(self):
        import copy
        from task_hand_control import ORDER
        from task_composition_preview import at
        path=ROOT/'data/composition_candidates/landing_gear-fastest-preview.json'
        candidate=json.loads(path.read_text());payload=candidate['units']
        tr,events=materialize(payload,candidate['sequence'])
        robot=ET.Element('robot')
        for name in payload['context']:ET.SubElement(robot,'joint',name=name,type='revolute')
        for name in ORDER:ET.SubElement(robot,'joint',name='right_'+name+'_joint',type='revolute')
        result=dict(payload=payload,trajectory=tr,events=events)
        before=copy.deepcopy(tr)
        message=native_playback_message(result,10.,S(robot=robot))
        native=message.trajectory[0].joint_trajectory
        self.assertEqual(tr,before)
        self.assertEqual(time_seconds(native.points[0]),0.)
        self.assertTrue(all(len(p.positions)==len(native.joint_names) for p in native.points))
        self.assertGreater(len(native.joint_names),len(tr.joint_names))
        expected=[at(tr,10.)]+[p for p in tr.points if time_seconds(p)>10.+1e-9]
        for actual,original in zip(native.points,expected):
            for field in ('positions','velocities','accelerations'):
                self.assertEqual(len(getattr(actual,field)),len(native.joint_names))
                np.testing.assert_allclose(getattr(actual,field)[:len(tr.joint_names)],getattr(original,field),atol=1e-12)

    def test_tcp_derivative_matches_fk_including_mimic(self):
        urdf=os.environ.get('G1_OFFLINE_TEST_URDF')
        if not urdf:self.skipTest('Set G1_OFFLINE_TEST_URDF')
        from workbench_model import OfflineModel
        model=OfflineModel(Path(urdf))
        for side in ('left','right'):
            names=[side+'_shoulder_pitch_joint',side+'_index_1_joint']
            positions=np.array([[.1,.4],[.2,.5]])
            rates=np.array([[.2,.3],[-.1,.4]])
            link=side+'_index_tcp';eps=1e-6
            expected=(model.batch_transforms([link],{},names,positions+rates*eps)[link][:,:3,3]-
                      model.batch_transforms([link],{},names,positions-rates*eps)[link][:,:3,3])/(2*eps)
            np.testing.assert_allclose(tcp_velocity(model,link,{},names,positions,rates),expected,atol=1e-8)


if __name__=='__main__':unittest.main()
