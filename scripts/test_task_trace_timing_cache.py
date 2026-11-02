"""Editor cache invalidation, cancellation and final continuous-limit checks."""
import copy
import tempfile
import unittest
from dataclasses import replace
from pathlib import Path
from unittest.mock import Mock,patch

from test_task_dring_demo import fixture
from task_planning_config import PlanningParameters
from task_trace_timing import generate,stored_trajectory
from task_trace_timing_cache import identity
from task_dring_teach_data import moving


class TimingCacheTests(unittest.TestCase):
    def setUp(self):
        self.model,_,self.config=fixture()
        self.trace=self.config['traces']['left']
        self.parameters=PlanningParameters()
        temporary=tempfile.TemporaryDirectory();self.addCleanup(temporary.cleanup)
        self.directory=Path(temporary.name)

    def generate(self,**options):
        return generate(self.trace,self.model,'left',self.parameters,workflow='dring_demo',
            config=self.config,slot='left',cache_dir=self.directory,**options)

    def test_repeated_request_reuses_search_but_checks_final_limits(self):
        first=self.generate(denoise=True)
        from task_cruise_timing import execution_limit_factors
        with (patch('task_trace_timing._preserving',side_effect=AssertionError('repeated search')),
              patch('task_cruise_timing.execution_limit_factors',wraps=execution_limit_factors) as limits):
            second=self.generate(denoise=True)
        self.assertTrue(second['optimization']['timing_report']['result_cache_hit'])
        self.assertEqual(limits.call_count,1)
        self.assertEqual(stored_trajectory(first,self.model,moving('left'))[0],
            stored_trajectory(second,self.model,moving('left'))[0])
        self.assertIn('computation_seconds',second['optimization']['timing_report'])

    def test_denoise_toggle_and_window_reuse_preserving_result(self):
        self.generate()
        with patch('task_trace_timing._preserving',side_effect=AssertionError('repeated preserving search')):
            a=self.generate(denoise=True)
            b=self.generate(denoise=True,window=.2)
        for item in (a,b):
            report=item['optimization']['timing_report']
            self.assertTrue(report['preserving_cache_hit'])
            self.assertFalse(report['result_cache_hit'])
        self.assertFalse(b['optimization']['timing_report']['smoothed_cache_hit'])

    def test_identity_includes_limits_model_context_and_source(self):
        def key():return identity(self.trace,self.model,'left',self.parameters,'dring_demo',self.config,'left',.7,.7)
        original=key()
        self.parameters=replace(self.parameters,arm_velocity_rad_s=.2)
        self.assertNotEqual(key(),original)
        self.parameters=PlanningParameters()
        self.assertEqual(key(),original)
        self.trace=copy.deepcopy(self.trace);self.trace['raw']['positions'][1][0]+=.001
        self.assertNotEqual(key(),original)
        self.trace=self.config['traces']['left']
        joint=next(iter(self.model.limits));lo,hi=self.model.limits[joint]
        self.model.limits[joint]=(lo+.001,hi)
        self.assertNotEqual(key(),original)

    def test_bad_cache_is_recomputed_and_cancel_still_applies(self):
        self.generate()
        for path in self.directory.glob('*.json'):path.write_text('{}')
        item=self.generate()
        self.assertFalse(item['optimization']['timing_report']['result_cache_hit'])
        with self.assertRaisesRegex(RuntimeError,'cancel'):
            self.generate(check=Mock(side_effect=RuntimeError('cancel')))

    def test_cached_trajectory_still_rejects_excess_speed(self):
        from task_trajectory_speed import rescale
        item=self.generate()
        tr,_=stored_trajectory(item,self.model,moving('left'))
        bad=rescale(tr,100.)
        with patch('task_trace_timing_cache.resolve',return_value=(bad,item['optimization']['timing_report'],True)):
            with self.assertRaisesRegex(ValueError,'连续运动限值'):self.generate()


if __name__=='__main__':unittest.main()
