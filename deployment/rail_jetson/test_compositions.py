import asyncio
import copy
import json
from itertools import product
from pathlib import Path
import tempfile
import unittest

from compositions import gear_recipe, actions, required, alternatives
from cloud_executor import ExecutionAudit, RecordedTaskExecutor
from bundle import digest
from test_cloud_executor import release, request


def parameters(start='LOWER', rounds=2, finish='UPPER', leave=False):
    return dict(startPosition=start, roundTrips=rounds, finishPosition=finish, exitToAbove=leave)


class RecipeTests(unittest.TestCase):
    def test_lower_round_trip_then_finish_upper(self):
        self.assertEqual(list(actions(gear_recipe(parameters(rounds=1)))), [
            'gear_above_to_lower', 'gear_lower_to_upper', 'gear_upper_to_lower', 'gear_lower_to_upper'])

    def test_upper_round_trip_finish_lower_then_exit(self):
        self.assertEqual(list(actions(gear_recipe(parameters('UPPER', 1, 'LOWER', True)))), [
            'gear_above_to_upper', 'gear_upper_to_lower', 'gear_lower_to_upper',
            'gear_upper_to_lower', 'gear_lower_to_above'])

    def test_all_combinations_are_contiguous_at_business_level(self):
        for start, finish, leave, count in product(('LOWER', 'UPPER'), ('LOWER', 'UPPER'), (False, True), (0, 1, 3)):
            recipe = gear_recipe(parameters(start, count, finish, leave))
            position = 'above'
            sequence = list(actions(recipe))
            for action in sequence:
                source, destination = action.removeprefix('gear_').split('_to_')
                self.assertEqual(source, position)
                position = destination
            self.assertEqual(position, 'above' if leave else finish.lower())
            self.assertEqual(len(sequence), 1 + 2 * count + int(start != finish) + int(leave))
            self.assertEqual(required(recipe), set(sequence))

    def test_zero_rounds_does_not_require_unused_strokes(self):
        self.assertEqual(required(gear_recipe(parameters(rounds=0, finish='LOWER'))), {'gear_above_to_lower'})

    def test_invalid_values_and_extra_parameters_are_rejected(self):
        for key, values in dict(roundTrips=[True, -1, 1001, 1.5, '1'], exitToAbove=[0, 1, 'false'],
                                startPosition=['ABOVE', None], finishPosition=['DOWN', None]).items():
            for value in values:
                with self.assertRaises(ValueError):
                    gear_recipe(dict(parameters(), **{key:value}))
        with self.assertRaises(ValueError):
            gear_recipe(dict(parameters(), skipChecks=True))

    def test_alternatives_never_modify_recorded_curves(self):
        bundle = release()
        original = copy.deepcopy(bundle)
        variants = alternatives(bundle)
        self.assertEqual(len(variants), 26)
        self.assertTrue(all(v['configured'] for v in variants.values()))
        self.assertEqual(bundle, original)


class FakeRuntime:
    def __init__(self):
        self.calls = []
        self.fail_on = None
        self.cancel_on = None
        self.stops = 0

    async def initialize(self, release):
        return {'status':'SUCCEEDED'}

    async def run(self, release, action, report):
        self.calls.append(action)
        await report(dict(label=action, stage=1))
        if self.cancel_on is not None:
            self.cancel_on.set()
        return dict(status='FAILED' if len(self.calls) == self.fail_on else 'SUCCEEDED', message='test')

    async def cancel(self):
        self.stops += 1
        return True

    async def close(self):
        pass


class CompositionExecutionTests(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.bundle = release()
        self.runtime = FakeRuntime()
        self.audit = ExecutionAudit(Path(self.temp.name) / 'audit.sqlite3')
        self.executor = RecordedTaskExecutor(lambda:self.bundle, self.runtime, self.audit)
        self.executor.initialized_sha = self.bundle['sha256']
        self.reports = []

    async def asyncTearDown(self):
        await self.executor.close()
        self.temp.cleanup()

    async def report(self, progress, message):
        self.reports.append((progress, message))

    async def execute(self, p, cancel=None):
        return await self.executor.execute(request('gear', 'GEAR_SEQUENCE', **p), cancel or asyncio.Event(), self.report)

    async def test_whole_task_success_waits_for_exit_and_audits_recipe(self):
        p = parameters('UPPER', 2, 'LOWER', True)
        result = await self.execute(p)
        self.assertEqual((result.status, result.progress), ('SUCCEEDED', 1))
        self.assertEqual(self.runtime.calls, list(actions(gear_recipe(p))))
        self.assertTrue(all(progress == 0 for progress, _ in self.reports[:-1]))
        details = json.loads(self.audit.db.execute('SELECT details FROM recorded_executions').fetchone()[0])
        self.assertEqual(details['composition']['recipe'], gear_recipe(p))
        self.assertEqual(details['composition']['releaseSha256'], self.bundle['sha256'])

    async def test_missing_exit_rejects_before_any_motion(self):
        del self.bundle['payload']['actions']['gear_upper_to_above']
        self.bundle['sha256'] = digest(self.bundle['payload'])
        self.executor.initialized_sha = self.bundle['sha256']
        result = await self.execute(parameters(leave=True))
        self.assertEqual(result.status, 'FAILED')
        self.assertIn('gear_upper_to_above', result.message)
        self.assertEqual(self.runtime.calls, [])

    async def test_failed_middle_step_never_dispatches_remaining_steps(self):
        self.runtime.fail_on = 3
        result = await self.execute(parameters(rounds=3, leave=True))
        self.assertEqual((result.status, result.progress), ('FAILED', 0))
        self.assertEqual(len(self.runtime.calls), 3)
        self.assertTrue(self.executor.recovery_required)

    async def test_failed_exit_is_not_success(self):
        p = parameters(rounds=0, leave=True)
        self.runtime.fail_on = len(list(actions(gear_recipe(p))))
        result = await self.execute(p)
        self.assertEqual((result.status, result.progress), ('FAILED', 0))

    async def test_cancel_prevents_next_segment(self):
        cancel = asyncio.Event()
        self.runtime.cancel_on = cancel
        result = await self.execute(parameters(), cancel)
        self.assertEqual(result.status, 'CANCELLED')
        self.assertEqual(len(self.runtime.calls), 1)

    async def test_no_implicit_initialize(self):
        self.executor.initialized_sha = None
        result = await self.execute(parameters())
        self.assertEqual(result.status, 'FAILED')
        self.assertEqual(self.runtime.calls, [])
