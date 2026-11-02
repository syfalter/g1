import asyncio
import copy
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest

from bundle import digest
from cloud_executor import ExecutionAudit, RecordedTaskExecutor, SequenceRuntime
from executor import SequenceExecutor
from offline_transport import OfflineTransport, offline_join
from releases import package, planned_action
from test_releases import fixture


def request(task_id, kind='NWS_CYCLE', **parameters):
    return SimpleNamespace(taskId=task_id, taskType=kind, parameters=parameters,
                           total=parameters.get('count', 1))


def release():
    actions = {}
    for index, slot in enumerate(('left_1', 'right_1', 'above_to_lower', 'above_to_upper', 'lower_to_upper', 'upper_to_lower',
                                 'lower_to_above', 'upper_to_above')):
        plan, config, init = fixture(slot)
        segment = plan['segments'][0]
        endpoint = .1 + index * .01
        segment['trajectory']['points'][-1]['positions'] = [endpoint] * 8
        segment['end'] = {name: endpoint for name in segment['end']}
        if '_to_' in slot:
            plan['demo'].update(workflow='landing_gear', full_cycle=False)
            plan['arm'] = segment['side'] = 'right'
            segment['trajectory']['joint_names'] = [name.replace('left_', 'right_')
                                                    for name in segment['trajectory']['joint_names']]
            for boundary in ('start', 'end'):
                segment[boundary] = {name.replace('left_', 'right_'): value for name, value in segment[boundary].items()}
        name, action = planned_action(plan, config, 'test/' + slot)
        # Align to the recording for repeated fake actions, as production plans do.
        segment = copy.deepcopy(plan['segments'][0])
        segment.pop('recording_fit')
        segment['stage'] = 'prepare'
        segment['label'] = 'alignment'
        segment['trajectory']['points'][-1]['positions'] = [0.] * 8
        segment['end'] = dict(segment['start'])
        plan['segments'].insert(0, segment)
        name, action = planned_action(plan, config, 'test/' + slot)
        actions[name] = action
    return package('cloud-test-v1', actions, init)


class CloudExecutorTests(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.bundle = release()
        self.transport = OfflineTransport(.002)
        self.runtime = SequenceRuntime(SequenceExecutor(self.transport, offline_join))
        self.audit_path = Path(self.temp.name) / 'audit.sqlite3'
        self.audit = ExecutionAudit(self.audit_path)
        self.executor = RecordedTaskExecutor(lambda: self.bundle, self.runtime, self.audit)
        self.events = []

    async def asyncTearDown(self):
        await self.executor.close()
        self.temp.cleanup()

    async def report(self, progress, message):
        self.events.append((progress, message))

    async def run_task(self, task, cancel=None):
        return await self.executor.execute(task, cancel or asyncio.Event(), self.report)

    async def initialize(self):
        result = await self.run_task(request('init', 'ROBOT_INITIALIZE'))
        self.assertEqual(result.status, 'SUCCEEDED', result.message)

    async def test_no_web_needed_and_count_is_a_complete_pair(self):
        await self.initialize()
        result = await self.run_task(request('pair', count=2))
        self.assertEqual((result.status, result.progress), ('SUCCEEDED', 2))
        self.assertEqual(len([c for c in self.transport.calls if c['kind'] == 'recorded']), 4)
        self.assertIn('NOT verified', result.message)
        row = self.audit.db.execute('SELECT release_sha256,status,progress FROM recorded_executions WHERE task_id=?',
                                    ('pair',)).fetchone()
        self.assertEqual(row, (self.bundle['sha256'], 'SUCCEEDED', 2))

    async def test_not_initialized_dispatches_nothing(self):
        result = await self.run_task(request('before-init', count=1))
        self.assertEqual(result.status, 'FAILED')
        self.assertEqual(self.transport.calls, [])
        self.assertFalse(self.executor.recovery_required)

    async def test_confirmed_failure_does_not_stop_scheduled_recovery_again(self):
        await self.initialize()
        stops = []
        async def failed(*args):
            return dict(status='FAILED', message='rejected before contact', stop_confirmed=True,
                        requires_recovery=True, automatic_recovery=True)
        async def cancel():
            stops.append(True)
            return True
        self.runtime.run, self.runtime.cancel = failed, cancel
        result = await self.run_task(request('failed-with-recovery', count=1))
        self.assertEqual(result.status, 'FAILED')
        self.assertIn('automatic preparation recovery scheduled', result.message)
        self.assertEqual(stops, [])

    async def test_return_home_requires_initialization(self):
        result = await self.run_task(request('home-before-init', 'ROBOT_RETURN_HOME'))
        self.assertEqual(result.status, 'FAILED')
        self.assertEqual(self.transport.calls, [])

    async def test_return_home_uses_historical_target_and_pins_it(self):
        import json
        from home_pose import reference
        await self.initialize()
        result = await self.run_task(request('home', 'ROBOT_RETURN_HOME'))
        self.assertEqual((result.status, result.progress), ('SUCCEEDED', 1), result.message)
        self.assertEqual(self.transport.joints, reference()['target'])
        self.assertIn('power-off safety NOT verified', result.message)
        self.assertEqual([c['kind'] for c in self.transport.calls], ['initialize', 'return_home'])
        details = self.audit.db.execute('SELECT details FROM recorded_executions WHERE task_id=?', ('home',)).fetchone()[0]
        self.assertEqual(json.loads(details)['home'], reference())

    async def test_return_home_does_not_clear_fault(self):
        await self.initialize()
        self.executor.recovery_required = True
        result = await self.run_task(request('fault-home', 'ROBOT_RETURN_HOME'))
        self.assertEqual(result.status, 'FAILED')
        self.assertEqual(len(self.transport.calls), 1)

    async def test_cancel_return_home(self):
        await self.initialize()
        self.transport.step_seconds = .5
        cancel = asyncio.Event()
        task = asyncio.create_task(self.run_task(request('cancel-home', 'ROBOT_RETURN_HOME'), cancel))
        await asyncio.sleep(.03)
        cancel.set()
        result = await asyncio.wait_for(task, 1.)
        self.assertEqual(result.status, 'CANCELLED', result.message)
        self.assertTrue(self.executor.recovery_required)

    async def test_missing_second_binding_fails_before_first_arm(self):
        del self.bundle['payload']['actions']['dring_right']
        self.bundle['sha256'] = digest(self.bundle['payload'])
        await self.initialize()
        result = await self.run_task(request('missing', count=1))
        self.assertEqual(result.status, 'FAILED')
        self.assertEqual(len(self.transport.calls), 1)

    async def test_release_switch_requires_initialization(self):
        await self.initialize()
        self.bundle['payload']['version'] = 'cloud-test-v2'
        self.bundle['sha256'] = digest(self.bundle['payload'])
        result = await self.run_task(request('changed', count=1))
        self.assertEqual(result.status, 'FAILED')
        self.assertIn('Initialize', result.message)

    async def test_failure_in_right_does_not_count_left_as_a_pair(self):
        await self.initialize()
        original = self.runtime.run

        async def fail_right(bundle, action, report):
            if action == 'dring_right':
                return dict(status='STOP_UNCONFIRMED', message='injected failure')
            return await original(bundle, action, report)

        self.runtime.run = fail_right
        result = await self.run_task(request('fail-right', count=1))
        self.assertEqual((result.status, result.progress), ('FAILED', 0))
        self.assertTrue(self.executor.recovery_required)
        again = await self.run_task(request('new-init', 'ROBOT_INITIALIZE'))
        self.assertEqual(again.status, 'FAILED')
        self.assertIn('Explicit recovery', again.message)

    async def test_cancel_reaches_blocking_sequence_and_latches_recovery(self):
        await self.initialize()
        self.transport.step_seconds = .5
        cancel = asyncio.Event()
        task = asyncio.create_task(self.run_task(request('cancel', count=10), cancel))
        await asyncio.sleep(.03)
        cancel.set()
        result = await asyncio.wait_for(task, 1)
        self.assertEqual((result.status, result.progress), ('CANCELLED', 0), result.message)
        self.assertTrue(self.transport.stopped.is_set())
        self.assertTrue(self.executor.recovery_required)

    async def test_cancel_without_stop_confirmation_is_failure(self):
        await self.initialize()

        async def bad_stop():
            return False

        self.runtime.cancel = bad_stop
        cancel = asyncio.Event()
        cancel.set()
        result = await self.run_task(request('unknown-stop', count=1), cancel)
        self.assertEqual(result.status, 'FAILED')
        self.assertIn('stop_confirmed=False', result.message)

    async def test_duplicate_audit_cannot_dispatch_twice(self):
        await self.initialize()
        await self.run_task(request('same', count=1))
        before = len(self.transport.calls)
        result = await self.run_task(request('same', count=1))
        self.assertEqual(result.status, 'FAILED')
        self.assertEqual(len(self.transport.calls), before)

    async def test_each_gear_segment_dispatches_exactly_one_recording(self):
        await self.initialize()
        for index, segment in enumerate(('above_to_lower', 'lower_to_upper', 'upper_to_lower', 'above_to_upper')):
            result = await self.run_task(request('gear-' + segment, 'GEAR_SEGMENT', segment=segment))
            self.assertEqual((result.status, result.progress), ('SUCCEEDED', 1), result.message)
            recordings = [c for c in self.transport.calls if c['kind'] == 'recorded']
            self.assertEqual(len(recordings), index + 1)
            expected = self.bundle['payload']['actions']['gear_' + segment]['program'][-1]['trajectory_sha256']
            self.assertEqual(recordings[-1]['trajectory_sha256'], expected)

    async def test_legacy_gear_cycle_is_not_automatically_composed(self):
        await self.initialize()
        result = await self.run_task(request('legacy', 'GEAR_OPERATION', count=2,
                                             preAction='EXTEND_DOWNLOCK', cycleMode='FULL_CYCLE'))
        self.assertEqual(result.status, 'FAILED')
        self.assertIn('not configured', result.message)
        self.assertEqual(len(self.transport.calls), 1)

    async def test_gear_segment_cannot_request_count_or_unknown_segment(self):
        await self.initialize()
        for index, parameters in enumerate((dict(segment='lower_to_upper', count=2), dict(segment='unknown'))):
            result = await self.run_task(request('invalid-' + str(index), 'GEAR_SEGMENT', **parameters))
            self.assertEqual(result.status, 'FAILED')
        self.assertEqual(len(self.transport.calls), 1)

    async def test_audit_restart_marks_unknown_without_replay(self):
        self.audit.pin(request('interrupted', count=1), self.bundle)
        before = len(self.transport.calls)
        self.audit.close()
        other = ExecutionAudit(self.audit_path)
        try:
            status = other.db.execute('SELECT status FROM recorded_executions').fetchone()[0]
            self.assertEqual(status, 'OUTCOME_UNKNOWN')
            self.assertEqual(len(self.transport.calls), before)
        finally:
            other.close()
            self.audit = ExecutionAudit(self.audit_path)
            self.executor.audit = self.audit

    async def test_second_process_cannot_recover_active_audit(self):
        self.audit.pin(request('active', count=1), self.bundle)
        with self.assertRaises(BlockingIOError):
            ExecutionAudit(self.audit_path)
        self.assertEqual(self.audit.db.execute('SELECT status FROM recorded_executions').fetchone()[0], 'RUNNING')

    async def test_missing_gear_segment_does_not_dispatch_any_other_segment(self):
        del self.bundle['payload']['actions']['gear_upper_to_lower']
        self.bundle['sha256'] = digest(self.bundle['payload'])
        await self.initialize()
        result = await self.run_task(request('incomplete-gear', 'GEAR_SEGMENT', segment='upper_to_lower'))
        self.assertEqual(result.status, 'FAILED')
        self.assertEqual(len(self.transport.calls), 1)


if __name__ == '__main__':
    unittest.main()
