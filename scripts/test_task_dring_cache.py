"""Persistent replay cache and offline candidate geometry regressions."""
import copy
from dataclasses import replace
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import numpy as np

from prepare_dring_candidates import circle_fit,blend
from task_dring_cache import cache_key
import task_dring_teach_data as data
from task_planning_config import PlanningParameters
from test_task_dring_demo import fixture


class ReplayCacheTests(unittest.TestCase):
    def test_disk_hit_skips_fit_and_retime_and_survives_gain_change(self):
        model,q,config=fixture();params=PlanningParameters()
        with tempfile.TemporaryDirectory() as directory:
            first=data.build(model,config,q,'left',params,.7,.7,1.,cache_dir=directory)
            self.assertFalse(first['speed_scaling']['replay_cache']['hit'])
            with patch('task_dring_teach_data.fit_trace',side_effect=AssertionError('refit')),\
                    patch('task_dring_teach_data.scale_arm_segments',side_effect=AssertionError('retime')):
                second=data.build(model,config,q,'left',replace(params,replay_waist_kp=240.),.7,.7,1.,cache_dir=directory)
            self.assertTrue(second['speed_scaling']['replay_cache']['hit'])
            self.assertEqual(first['segments'][-1]['trajectory'],second['segments'][-1]['trajectory'])
            path=next(Path(directory).glob('*.json'))
            item=json.loads(path.read_text());item['payload']['trajectory']['points'][0]['positions'][0]=1.
            path.write_text(json.dumps(item))
            repaired=data.build(model,config,q,'left',params,.7,.7,1.,cache_dir=directory)
            self.assertFalse(repaired['speed_scaling']['replay_cache']['hit'])
            self.assertEqual(first['segments'][-1]['trajectory'],repaired['segments'][-1]['trajectory'])

    def test_key_tracks_trace_limits_model_and_speed_not_current_position(self):
        model,_,config=fixture();trace=config['traces']['left'];params=PlanningParameters()
        base=cache_key(trace,model,'left',1.,.7,.7,params)
        for speed,velocity,acceleration in ((2.,.7,.7),(1.,.6,.7),(1.,.7,.6)):
            self.assertNotEqual(base,cache_key(trace,model,'left',speed,velocity,acceleration,params))
        altered=copy.deepcopy(trace);altered['raw']['positions'][10][0]+=.001
        self.assertNotEqual(base,cache_key(altered,model,'left',1.,.7,.7,params))
        model.robot.find('joint/limit').set('velocity','.2')
        self.assertNotEqual(base,cache_key(trace,model,'left',1.,.7,.7,params))

    def test_cached_replay_still_builds_transition_from_new_actual_position(self):
        model,q,config=fixture();params=PlanningParameters()
        with tempfile.TemporaryDirectory() as directory:
            first=data.build(model,config,q,'left',params,.7,.7,1.,cache_dir=directory)
            shifted=dict(q);shifted[data.moving('left')[0]]=.2
            with patch('task_dring_teach_data.fit_trace',side_effect=AssertionError('refit')):
                second=data.build(model,config,shifted,'left',params,.7,.7,1.,cache_dir=directory)
            self.assertEqual(second['segments'][0]['start'],shifted)
            self.assertEqual(first['segments'][-1]['trajectory'],second['segments'][-1]['trajectory'])
            self.assertEqual(second['demo']['transition_count'],1)


class CandidateGeometryTests(unittest.TestCase):
    def test_circle_fit_recovers_synthetic_arc_without_claiming_calibration(self):
        angle=np.linspace(.3,1.5,300);center=np.array([.38,.17]);radius=.06
        points=np.c_[center[0]+radius*np.cos(angle),center[1]+radius*np.sin(angle),np.full(300,-.1)]
        fit=circle_fit(points,np.linspace(0,10,300))
        np.testing.assert_allclose(fit['center_xy_pelvis_m'],center,atol=1e-8)
        self.assertAlmostEqual(fit['radius_m'],radius)
        self.assertFalse(fit['physically_calibrated'])
        np.testing.assert_allclose(blend(np.array([-1,0,1,2])),[0,0,1,1])

    def test_small_angle_arc_is_rejected(self):
        angle=np.linspace(.3,.4,100)
        points=np.c_[.38+.06*np.cos(angle),.17+.06*np.sin(angle),np.full(100,-.1)]
        with self.assertRaises(ValueError):circle_fit(points,np.linspace(0,2,100))


if __name__=='__main__':unittest.main()
