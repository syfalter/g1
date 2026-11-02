"""Offline timing candidates, exact reload, and transactional editor persistence."""
import copy
import unittest
from unittest.mock import Mock,patch
import numpy as np

from task_fast_timing import fastest
from task_trace_timing import encode,stored_trajectory,generate
from task_dring_teach_data import fit_trace,moving
from task_planning_config import PlanningParameters
from task_trajectory import time_seconds,task_jerk_limits,validate_jerk_limits
from task_trajectory_speed import speed_capacity
import test_task_dring_demo as fixtures
fixture=fixtures.fixture


class FastTimingTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.model,cls.start,cls.config=fixture()
        cls.source,_=fit_trace(cls.config['traces']['left'],cls.model,'left')
        cls.result,cls.report=fastest(cls.source,cls.model,PlanningParameters())
        cls.report.update(original_1x_seconds=4.,speedup_over_original_1x=4/time_seconds(cls.result.points[-1]))

    def test_faster_than_original_1x_and_not_slower_than_original_maximum(self):
        self.assertLess(time_seconds(self.result.points[-1]),4.)
        self.assertLessEqual(time_seconds(self.result.points[-1]),self.report['original_fastest_seconds']+1e-9)
        self.assertFalse(self.report['globally_optimal'])
        cap=speed_capacity(self.result,self.model,PlanningParameters(),velocity=.7,acceleration=.7)
        self.assertGreaterEqual(cap['maximum_multiplier'],1.-1e-6)
        validate_jerk_limits(self.result,task_jerk_limits(self.result.joint_names))
        np.testing.assert_allclose(self.result.points[0].positions,self.source.points[0].positions,atol=1e-10)
        np.testing.assert_allclose(self.result.points[-1].positions,self.source.points[-1].positions,atol=1e-10)

    def test_exact_stored_timing_is_not_refitted(self):
        trace=self.config['traces']['left'];before=copy.deepcopy(trace)
        edited=encode(trace,self.result,self.report)
        reread,fit=fit_trace(edited,self.model,'left')
        self.assertEqual(reread,self.result)
        import json
        reloaded=json.loads(json.dumps(edited))
        self.assertEqual(stored_trajectory(reloaded,self.model,moving('left'))[0],self.result)
        self.assertEqual(fit['source'],'stored_exact_timing')
        self.assertEqual(trace,before)
        self.assertLessEqual(max(np.diff(edited['raw']['time_s'])),.005+1e-9)

    def test_tampered_timing_or_samples_rejected(self):
        edited=encode(self.config['traces']['left'],self.result,self.report)
        edited['raw']['positions'][10][0]+=.001
        with self.assertRaisesRegex(ValueError,'编辑轨迹'):stored_trajectory(edited,self.model,moving('left'))
        edited=encode(self.config['traces']['left'],self.result,self.report)
        edited['optimization']['timed_trajectory']['points'][1]['velocities'][0]+=1.
        with self.assertRaisesRegex(ValueError,'校验'):stored_trajectory(edited,self.model,moving('left'))

    def test_failed_local_search_keeps_verified_original_maximum(self):
        with patch('task_fast_timing.local_candidate',side_effect=ValueError('not feasible')):
            tr,report=fastest(self.source,self.model,PlanningParameters())
        self.assertEqual(time_seconds(tr.points[-1]),report['original_fastest_seconds'])
        self.assertEqual(report['local_error'],'not feasible')
        with self.assertRaisesRegex(RuntimeError,'cancel'):
            fastest(self.source,self.model,check=Mock(side_effect=RuntimeError('cancel')))

    def test_hand_timeline_not_silently_dropped(self):
        model,_,config=fixture(True)
        with self.assertRaisesRegex(ValueError,'同步手指'):
            generate(config['traces']['left'],model,'left',PlanningParameters(),workflow='dring_demo',config=config,slot='left')

    def test_optional_denoise_is_explicit_and_never_slower_than_reference(self):
        trace=self.config['traces']['left'];original=copy.deepcopy(trace)
        edited=generate(trace,self.model,'left',PlanningParameters(),workflow='dring_demo',
            config=self.config,slot='left',denoise=True)
        report=edited['optimization']['timing_report']
        self.assertTrue(report['denoise_requested'])
        self.assertLessEqual(report['duration_after_s'],report['preserving_seconds']+1e-9)
        self.assertLessEqual(report['duration_after_s'],report['original_1x_seconds']+1e-9)
        if report['path_modified']:
            self.assertLessEqual(report['smoothing']['max_smoothing_rad'],.003+1e-12)
        self.assertEqual(trace,original)

    def test_rail_exact_candidate_reload_without_refitting(self):
        import test_task_screen as fixtures
        from task_rail_trace import replay
        fixtures.RailTests.setUpClass()
        fixture=fixtures.RailTests();fixture.setUp();self.addCleanup(fixture.doCleanups)
        config=fixture.traced();trace=config['traces']['push']
        edited=generate(trace,fixture.n.model,'right',PlanningParameters(),workflow='rail',config=config,slot='push')
        self.assertEqual(edited['hand'],trace['hand'])
        exact,_=stored_trajectory(edited,fixture.n.model,moving('right'))
        start=dict(fixture.q,**dict(zip(moving('right'),edited['raw']['positions'][0])))
        with patch('task_rail_trace.make_interp_spline',side_effect=AssertionError('must not refit')):
            tr,report=replay(fixture.n.model,edited,start,PlanningParameters())
        self.assertEqual(report['interpolation'],'stored_exact_timing')
        from task_trajectory_speed import rescale
        self.assertEqual(tr,rescale(exact,report['timing']['applied_multiplier']))
        self.assertGreater(report['timing']['applied_multiplier'],.995)


class FastEditorTests(unittest.TestCase):
    def setUp(self):
        fixtures.DemoPersistenceTests.setUp(self)
        cache=patch('task_trace_timing_cache.DEFAULT_DIRECTORY',self.node.run_dir/'timing_cache')
        cache.start();self.addCleanup(cache.stop)
        self.node.launch_work=lambda work:work()
        self.control.persist(self.config)

    def test_save_one_slot_with_time_and_replay_exactly(self):
        from task_trace_versions import bank
        original=copy.deepcopy(self.control.config['traces']['left'])
        self.control.edit_version('left','fastest',velocity=.7,acceleration=.7)
        entry=bank(self.control.config,'left')
        self.assertEqual(entry['selected'],1)
        self.assertEqual(entry['baseline'],original)
        self.assertEqual(sum(v is not None for v in entry['edits']),1)
        edited=entry['edits'][0]
        self.assertIn('created_at',edited['edit_origin'])
        fitted,_=fit_trace(edited,self.node.model,'left')
        self.assertAlmostEqual(time_seconds(fitted.points[-1]),edited['raw']['time_s'][-1])
        self.assertIn('原策略 1x',self.control.editor_feedback)

    def test_full_bank_refuses_before_calculation(self):
        from task_trace_versions import append,EDIT_SLOTS
        c=self.control
        config=c.config
        for _ in range(EDIT_SLOTS):config=append(config,'left',config['traces']['left'])
        c.persist(config);before=c.store.read_bytes()
        with patch('task_trace_timing.generate',side_effect=AssertionError('must not compute')):
            with self.assertRaises(ValueError):c.edit_version('left','fastest')
        self.assertEqual(c.store.read_bytes(),before)

    def test_stale_calculation_cannot_commit(self):
        jobs=[];self.node.launch_work=jobs.append
        self.control.edit_version('left','fastest')
        self.control.revision+=1;before=self.control.store.read_bytes()
        with self.assertRaises(RuntimeError):jobs[0]()
        self.assertEqual(self.control.store.read_bytes(),before)


if __name__=='__main__':unittest.main()
