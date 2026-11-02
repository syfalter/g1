import copy
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace as S
import unittest
from unittest.mock import Mock,patch
import xml.etree.ElementTree as ET

import numpy as np
from rosidl_runtime_py.convert import message_to_ordereddict
from trajectory_msgs.msg import JointTrajectory,JointTrajectoryPoint

from task_composition_preview import stamp
from task_optimizer_snapshot import save,load,latest,for_case


class SnapshotTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.directory=Path(self.tmp.name)
        self.model=S(robot=ET.fromstring('<robot name="test"/>'))
        tr=JointTrajectory(joint_names=['a'],points=[stamp(JointTrajectoryPoint(
            positions=[q],velocities=[0.],accelerations=[0.]),t) for t,q in ((0.,0.),(2.,.1))])
        payload=dict(workflow='single',mode='offline_optimizer',context={},sources=dict(slot='above_to_lower',selectedVersion=0),
                     options={},optimizer=dict(jerk=5.,changed=1,total=1),
                     units={'single':dict(trajectory=message_to_ordereddict(tr),seconds=2.,report={})})
        item=dict(payload=payload,sequence=['single'],trajectory=tr,times=np.array([0.,2.]),
                  tcp=np.array([0.,0.]),joint=np.array([0.,0.]),joint_names=['a'])
        self.results={k:copy.deepcopy(item) for k in ('original','baseline','candidate','smooth')}
        self.controls=dict(case=['landing_gear','above_to_lower'])

    def tearDown(self):self.tmp.cleanup()

    def test_roundtrip_exact_trajectory_without_planning(self):
        original=copy.deepcopy(self.results)
        path=save(self.results,self.controls,self.model,self.directory)
        restored=load(path,self.model)
        self.assertEqual(restored['variants']['smooth']['trajectory'],original['smooth']['trajectory'])
        self.assertEqual(self.results['smooth']['trajectory'],original['smooth']['trajectory'])
        np.testing.assert_array_equal(restored['variants']['smooth']['times'],original['smooth']['times'])
        self.assertEqual(latest(self.directory),path)
        self.assertEqual(for_case(*self.controls['case'],self.directory),path)
        self.assertIsNone(for_case('rail','push',self.directory))

    def test_task_catalog_keeps_independent_results(self):
        first=save(self.results,self.controls,self.model,self.directory)
        second=save(self.results,dict(case=['rail','push']),self.model,self.directory)
        self.assertEqual(for_case(*self.controls['case'],self.directory),first)
        self.assertEqual(for_case('rail','push',self.directory),second)

    def test_tampered_or_wrong_model_not_loaded(self):
        path=save(self.results,self.controls,self.model,self.directory)
        with self.assertRaisesRegex(ValueError,'模型'):load(path,S(robot=ET.fromstring('<robot name="other"/>')))
        data=json.loads(path.read_text());data['data']['variants']['smooth']['seconds']=3.
        path.write_text(json.dumps(data))
        with self.assertRaisesRegex(ValueError,'校验'):load(path,self.model)

    def test_task_selector_loads_without_generating_and_rejects_changed_version(self):
        from PyQt5 import QtWidgets
        from task_offline_optimizer_panel import OptimizerPanel,data_index
        app=QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
        save(self.results,self.controls,self.model,self.directory)
        control=S(config={'traces':{'above_to_lower':{}}},revision=0)
        empty=S(config={'traces':{}},revision=0)
        node=S(demo=empty,landing_gear=control,rail=empty,model=self.model,
               display_pub=Mock(),demo_preview_pub=Mock(),status='',launch_work=Mock())
        panel=OptimizerPanel(node,snapshot_directory=self.directory)
        try:
            with patch('task_offline_optimizer_panel.comparison',side_effect=AssertionError('Unexpected optimization')):
                self.assertTrue(panel.restore_startup())
                self.assertEqual(panel.case.currentData(),('landing_gear','above_to_lower'))
                panel.case.setCurrentIndex(0)
                panel.case.setCurrentIndex(data_index(panel.case,('landing_gear','above_to_lower')))
                app.processEvents()
                self.assertIsNotNone(panel.viewer.result,panel.viewer.summary.text())
                self.assertEqual(panel.viewer.duration(),2.)
                self.assertTrue(panel.viewer.play.isEnabled())
                self.assertIn('固定结果',panel.viewer.summary.text())
                panel.viewer.toggle()
                self.assertTrue(node.display_pub.publish.called)
                panel.viewer.toggle()
                node.launch_work.assert_not_called()
                control.config['trace_versions']={'above_to_lower':dict(baseline={},edits=[{}]+[None]*9,selected=1)}
                panel.case.setCurrentIndex(0)
                panel.case.setCurrentIndex(data_index(panel.case,('landing_gear','above_to_lower')))
                self.assertIsNone(panel.viewer.result)
                self.assertIn('版本已变化',panel.viewer.summary.text())
        finally:panel.halt();panel.viewer.timer.stop();panel.close()


if __name__=='__main__':unittest.main()
