import copy
import unittest
import tempfile
from types import SimpleNamespace
from unittest.mock import patch
import xml.etree.ElementTree as ET

import numpy as np
from trajectory_msgs.msg import JointTrajectory,JointTrajectoryPoint

from task_composition_preview import (stamp,at,section,concatenate,parent_of_trim,select,materialize,
                                      bridge,assemble_phases,fastest_units,validate_timing)
from task_trace_edit import trim_tail


def line():
    return JointTrajectory(joint_names=['joint'],points=[
        stamp(JointTrajectoryPoint(positions=[q],velocities=[v],accelerations=[a]),t)
        for q,v,a,t in ((0.,0.,0.,0.),(1.,0.,0.,2.))])


class CompositionTests(unittest.TestCase):
    def test_exact_section_reconstructs_original_polynomial(self):
        source=line();left=section(source,0.,.8);right=section(source,.8,2.)
        rebuilt=concatenate([left,right])
        for t in np.linspace(0.,2.,101):
            for field in ('positions','velocities','accelerations'):
                np.testing.assert_allclose(getattr(at(source,t),field),getattr(at(rebuilt,t),field),atol=1e-10)
        self.assertGreater(abs(left.points[-1].velocities[0]),0.)

    def test_bridge_carries_cut_derivatives(self):
        source=line();a=at(source,.7);b=at(source,1.3)
        tr=bridge(['joint'],a,b)
        for field in ('positions','velocities','accelerations'):
            np.testing.assert_allclose(getattr(tr.points[0],field),getattr(a,field),atol=1e-10)
            np.testing.assert_allclose(getattr(tr.points[-1],field),getattr(b,field),atol=1e-10)

    def test_mismatched_velocity_cannot_be_spliced(self):
        source=line();a=section(source,0.,.8);b=section(source,.8,2.)
        b.points[0].velocities=[0.]
        with self.assertRaises(ValueError):concatenate([a,b])

    def test_tail_parent_requires_exact_provenance(self):
        trace={'raw':{'positions':[[0.],[.5],[1.]],'time_s':[0.,1.,2.]},'hand':None}
        shortened=trim_tail(trace,.5);shortened['edit_origin']={'operation':'trim_tail','version':0}
        config={'traces':{'right_1':shortened},'trace_versions':{'right_1':{
            'baseline':trace,'edits':[shortened]+[None]*9,'selected':1}}}
        parent,cut,info=parent_of_trim(config,'right_1')
        self.assertEqual(cut,1.5);self.assertEqual(parent,trace);self.assertEqual(info['parentVersion'],0)
        config['traces']['right_1']=copy.deepcopy(shortened)
        config['traces']['right_1']['raw']['positions'][-1][0]+=.1
        with self.assertRaises(ValueError):parent_of_trim(config,'right_1')

    def test_one_pair_uses_final_only_multiple_reuse_cycle(self):
        payload={'workflow':'dring_demo'}
        self.assertEqual(select(payload,1),['entry','final'])
        self.assertEqual(select(payload,3),['entry','cycle','cycle','final'])
        for count in (0,True,101):
            with self.assertRaises(ValueError):select(payload,count)

    def test_gear_keeps_grip_and_exit_events(self):
        self.assertEqual(select({'workflow':'landing_gear'},2,'UPPER','LOWER',True),
                         ['entry_upper','grip_wait','cycle_upper','cycle_upper','ul','open_wait','exit_lower'])

    def test_size_budget_is_enforced(self):
        with self.assertRaises(ValueError):concatenate([line()],limit=1)

    def test_materializing_repeatedly_never_mutates_cached_json(self):
        from rosidl_runtime_py.convert import message_to_ordereddict
        payload=dict(workflow='dring_demo',units={'entry':{'trajectory':message_to_ordereddict(line())}})
        before=copy.deepcopy(payload)
        for _ in range(3):materialize(payload,['entry'])
        self.assertEqual(payload,before)

    def test_operation_rate_does_not_slow_transit(self):
        a=line();b=line()
        for point in b.points:point.positions[0]+=1.
        phases={'a':dict(trajectory=a,motionClass='operation',report={}),
                'b':dict(trajectory=b,motionClass='transit',report={})}
        with patch('task_composition_preview.validate_timing',return_value={}):
            result=assemble_phases(phases,{'unit':['a','b']},None,None,.5,1.,lambda:None)
        self.assertAlmostEqual(result['unit']['seconds'],6.)
        self.assertEqual([p['seconds'] for p in result['unit']['report']['phases']],[4.,2.])
        self.assertEqual(a.points[-1].time_from_start.sec,2)

    def test_phase_cache_reused_when_rates_change(self):
        phases={'a':dict(trajectory=line(),motionClass='operation')}
        with tempfile.TemporaryDirectory() as folder, \
             patch('task_composition_preview.raw_phases',return_value=(phases,{'unit':['a']},{})), \
             patch('task_fast_timing.fastest',return_value=(line(),{'method':'test'})) as planner, \
             patch('task_composition_preview.validate_timing',return_value={}):
            for rate in (1.,.4):
                _,_,_,hit=fastest_units('dring_demo',{}, {},None,None,{'transitRate':rate},
                    lambda:None,lambda text:None,folder,{'multiplier':rate,'code':'same'})
                self.assertEqual(hit,rate!=1.)
            self.assertEqual(planner.call_count,1)

    def test_final_limit_gate_rejects_overspeed(self):
        from task_trajectory_speed import rescale
        model=SimpleNamespace(limits={'joint':(-10.,10.)},
            robot=ET.fromstring('<robot><joint name="joint"><limit velocity="10"/></joint></robot>'))
        with self.assertRaises(ValueError):validate_timing(line(),model,None,lambda:None)
        validate_timing(rescale(line(),.1),model,None,lambda:None)


if __name__=='__main__':unittest.main()
