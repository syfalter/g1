"""Read-only integration checks against generated offline composition candidates."""
import copy
import json
import os
from pathlib import Path
import unittest

import numpy as np
from rosidl_runtime_py.set_message import set_message_fields
from trajectory_msgs.msg import JointTrajectory

from task_composition_preview import ROOT,select,materialize,validate_timing
from task_planning_config import load_parameters
from task_trajectory import time_seconds
from workbench_model import OfflineModel


class ActualCompositionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        urdf=os.environ.get('G1_OFFLINE_TEST_URDF')
        if not urdf:raise unittest.SkipTest('Set G1_OFFLINE_TEST_URDF')
        cls.model=OfflineModel(Path(urdf));cls.parameters,_=load_parameters()

    def payload(self,workflow,suffix=''):
        path=ROOT/'data/composition_candidates'/(workflow+'-fastest'+suffix+'-preview.json')
        self.assertTrue(path.exists(),str(path))
        return json.loads(path.read_text())['units']

    def test_all_fastest_units_pass_actual_continuous_limits(self):
        for workflow in ('dring_demo','landing_gear'):
            for suffix in ('','-smooth'):
                payload=self.payload(workflow,suffix)
                self.assertFalse(payload['hardware_executable'])
                for name,unit in payload['units'].items():
                    with self.subTest(workflow=workflow,variant=suffix,unit=name):
                        trajectory=JointTrajectory();set_message_fields(trajectory,copy.deepcopy(unit['trajectory']))
                        validate_timing(trajectory,self.model,self.parameters,lambda:None)

    def test_all_gear_start_end_exit_combinations_join(self):
        for suffix in ('','-smooth'):
            payload=self.payload('landing_gear',suffix)
            for start in ('LOWER','UPPER'):
                for finish in ('LOWER','UPPER'):
                    for leave in (False,True):
                        with self.subTest(variant=suffix,start=start,finish=finish,leave=leave):
                            sequence=select(payload,3,start,finish,leave)
                            tr,events=materialize(payload,sequence)
                            self.assertTrue(np.all(np.diff([time_seconds(p) for p in tr.points])>0))
                            self.assertEqual([e['posture'] for e in events],['open','grip']+(['open'] if leave else []))

    def test_gear_candidate_has_one_timed_phase_per_business_unit(self):
        payload=self.payload('landing_gear','-smooth')
        for name in ('entry_lower','entry_upper','lu','ul','exit_lower','exit_upper'):
            phases=payload['units'][name]['report']['phases']
            self.assertEqual(len(phases),1)
            self.assertGreater(len(phases[0]['mergedPhases']),1)
            bounds=phases[0]['pathSmoothing']
            self.assertFalse(bounds['internal_stop_constraints_preserved'])
            self.assertLessEqual(bounds['tcp_m'],.001+1e-9)

    def test_valley_candidate_is_scoped_bounded_and_faster(self):
        old=self.payload('landing_gear','-smooth')
        new=self.payload('landing_gear','-smooth-valleys-v2')
        for name,unit in new['units'].items():
            trajectory=JointTrajectory();set_message_fields(trajectory,copy.deepcopy(unit['trajectory']))
            validate_timing(trajectory,self.model,self.parameters,lambda:None)
            if name!='entry_lower':
                self.assertEqual(unit,old['units'][name])
        before=old['units']['entry_lower'];after=new['units']['entry_lower']
        self.assertLess(after['seconds'],before['seconds'])
        report=after['report']['phases'][0]['pathSmoothing']
        self.assertEqual(report['method'],'bounded_spatial_valleys_v2')
        self.assertGreaterEqual(len(report['local_blends']),2)
        self.assertLessEqual(report['tcp_m'],.001)
        self.assertLessEqual(report['joint_rad'],.003)
        self.assertLessEqual(report['orientation_rad'],np.radians(.5))
        for index in (0,-1):
            for field in ('positions','velocities','accelerations'):
                np.testing.assert_allclose(after['trajectory']['points'][index][field],
                    before['trajectory']['points'][index][field],atol=1e-9)

    def test_rate_adjustment_preserves_transit_durations(self):
        for workflow in ('dring_demo','landing_gear'):
            original=self.payload(workflow);slower=self.payload(workflow,'-op0.5-transit1')
            self.assertTrue(slower['phaseCacheHit'])
            for name,unit in original['units'].items():
                for a,b in zip(unit['report']['phases'],slower['units'][name]['report']['phases']):
                    ratio=1. if a['motionClass']=='transit' else 2.
                    self.assertAlmostEqual(b['seconds'],ratio*a['seconds'],places=7)
            tr,_=materialize(slower,select(slower,3,leave=True))
            validate_timing(tr,self.model,self.parameters,lambda:None)


if __name__=='__main__':unittest.main()
