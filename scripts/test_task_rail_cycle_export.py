"""Pinned-cycle parity and selector tests; never creates a ROS node."""
import copy
import json
import os
from pathlib import Path
import sys
from types import SimpleNamespace as S
import unittest
from unittest.mock import Mock, patch

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'deployment/rail_jetson'))


class RailCycleTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from workbench_model import OfflineModel
        cls.model=OfflineModel(Path(os.environ['G1_OFFLINE_TEST_URDF']))
        cls.directory=Path(os.environ.get('G1_DEPLOY_TEST_DIR',ROOT/'data/rail_deployment/approved_v5_20260928'))

    def test_full_cycle_snapshots_and_stationary_waits(self):
        from task_optimizer_snapshot import for_case,load
        from task_offline_optimizer import decode,validate
        from task_planning_config import load_parameters
        parameters,_=load_parameters()
        for operation,stages in [('push_cycle',{'approach_A','push','retreat_B'}),
                                 ('pull_cycle',{'approach_B','pull','retreat_A'})]:
            result=load(for_case('rail',operation),self.model)['variants']['flow']
            payload=result['payload'];tr=result['trajectory']
            self.assertEqual(set(payload['sources']),stages)
            np.testing.assert_allclose(tr.points[0].positions,tr.points[-1].positions,atol=1e-10)
            validate(tr,self.model,parameters,5.)
            for name in payload['handWaits']:
                wait=decode(payload['units'][name]['trajectory'])
                for point in wait.points:
                    np.testing.assert_allclose(point.positions,wait.points[0].positions,atol=1e-12)
                    np.testing.assert_allclose(point.velocities,0.,atol=1e-12)
            self.assertEqual(len(result['events']),2 if operation=='push_cycle' else 3)
            from task_kinematics import mirror_left_ready
            ready=json.loads((ROOT/'data/task_panel/ready_poses.json').read_text())
            expected=mirror_left_ready(self.model,ready['left'])['joints']
            np.testing.assert_allclose(tr.points[0].positions,[expected[n] for n in tr.joint_names],atol=1e-10)

    def test_exported_arms_are_exact_preview_blocks(self):
        from releases import load_release
        from task_optimizer_snapshot import load
        from task_composition_preview import concatenate
        from task_offline_optimizer import decode
        from rosidl_runtime_py.convert import message_to_ordereddict
        bundle=load_release(self.directory/'bundle.json')
        manifest=json.loads((self.directory/'manifest.json').read_text())
        for name,entry in manifest['actions'].items():
            source=load(ROOT/entry['snapshot'],self.model)['variants']['flow']['payload']
            expected=[];pending=[]
            for unit in entry['sequence']:
                if unit.endswith('_wait') or unit in source.get('handWaits',{}):
                    if pending:expected.append(message_to_ordereddict(concatenate(pending)));pending=[]
                else:pending.append(decode(source['units'][unit]['trajectory']))
            if pending:expected.append(message_to_ordereddict(concatenate(pending)))
            action=bundle['payload']['actions'][name]
            actual=[s['trajectory'] for s in action['program'] if s['kind']=='recorded']
            self.assertEqual(actual,expected,name)
            self.assertEqual(action['verification'],'planned')
            self.assertFalse(action['plan']['physical_verified'])
            if action['plan']['arm']=='right':
                initial=bundle['payload']['initialization']['target']
                for tr,index in ((actual[0],0),(actual[-1],-1)):
                    np.testing.assert_allclose(tr['points'][index]['positions'],
                        [initial[n] for n in tr['joint_names']],atol=1e-8)

    def test_gear_contact_units_unchanged_after_preparation_fix(self):
        from task_optimizer_snapshot import for_case,load
        before=load(ROOT/'data/offline_optimizer_previews/9095e5dd08bd5c105606fa3c214979779db666d13631c516ed4a654904e1aa21.json',self.model)
        after=load(for_case('landing_gear',None),self.model)
        for name in ('lu','ul','cycle_lower','cycle_upper'):
            self.assertEqual(before['variants']['flow']['payload']['units'][name],
                             after['variants']['flow']['payload']['units'][name])

    def test_selector_plays_full_cycles_without_replanning(self):
        from PyQt5 import QtWidgets
        from task_offline_optimizer_panel import OptimizerPanel,data_index
        from task_trace_selection import restore
        app=QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
        source=ROOT/'data/task_sessions/20260924_153036_985614/rail_autosave/config.json'
        rail=S(config=restore(json.loads(source.read_text()),source),revision=0)
        empty=S(config={'traces':{}},revision=0)
        node=S(demo=empty,landing_gear=empty,rail=rail,model=self.model,
               display_pub=Mock(),demo_preview_pub=Mock(),status='',launch_work=Mock())
        panel=OptimizerPanel(node)
        try:
            with patch('task_offline_optimizer_panel.comparison',side_effect=AssertionError('Unexpected optimizer')):
                for operation in ('push_cycle','pull_cycle'):
                    panel.case.setCurrentIndex(data_index(panel.case,('rail',operation)))
                    app.processEvents()
                    self.assertIsNotNone(panel.viewer.result,panel.viewer.summary.text())
                    self.assertEqual(panel.viewer.result['payload']['optimizer']['variant'],'flow')
                    panel.viewer.toggle();self.assertTrue(panel.viewer.playing)
                    panel.viewer.toggle()
                    panel.viewer.resize(980,850);panel.viewer.show();app.processEvents()
                    self.assertTrue(panel.viewer.grab().save(str(self.directory/(operation+'-preview.png'))))
                node.launch_work.assert_not_called()
                rail.config['traces']['push']['invalidated_test']=True
                panel.case.setCurrentIndex(data_index(panel.case,('rail','push_cycle')))
                self.assertIsNone(panel.viewer.result)
        finally:panel.halt();panel.viewer.timer.stop();panel.close()

    def test_pair_routing_and_export_tamper_rejection(self):
        from cloud_executor import RecordedTaskExecutor
        from releases import load_release,validate
        from bundle import digest
        release=load_release(self.directory/'bundle.json')
        request=S(taskType='NWS_CYCLE',parameters={'count':1},total=1)
        self.assertEqual(list(RecordedTaskExecutor.groups(request,release)),[(['dring_pair'],True)])
        for name in ('push','pull','gear_roundtrip_lower_upper','gear_roundtrip_upper_lower'):
            request=S(taskType='RECORDED_ACTION',parameters={'action':name},total=1)
            self.assertEqual(list(RecordedTaskExecutor.groups(request,release)),[([name],True)])
        bad=copy.deepcopy(release)
        bad['payload']['actions']['push']['plan']['export_validation']['segments_sha256']='bad'
        bad['sha256']=digest(bad['payload'])
        with self.assertRaises(ValueError):validate(bad)


if __name__=='__main__':unittest.main()
