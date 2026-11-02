import asyncio
import copy
import os
from itertools import islice
from pathlib import Path
import tempfile
import time
from types import SimpleNamespace
import unittest

from automatic_recovery import endpoint
from bundle import digest, read
from cloud_executor import ExecutionAudit, RecordedTaskExecutor, SequenceRuntime
from executor import SequenceExecutor, hand_step_policy
from gear_protocol import BINDINGS, GearLibrary, concatenate, recipe
from native_worker import Worker
from offline_transport import OfflineTransport, offline_join

ROOT = Path(__file__).resolve().parents[2]
STAGE = ROOT / 'data/rail_deployment/gear_protocol_20260928'
RELEASE_PATH = Path(os.environ.get('GEAR_TEST_BUNDLE', str(ROOT / 'data/rail_deployment/approved_mirror_20260928/bundle.json')))
LIBRARY_PATH = Path(os.environ.get('GEAR_TEST_LIBRARY', str(STAGE / 'gear_protocol.json')))


def fixtures():
    release = read(RELEASE_PATH)
    library = GearLibrary(read(LIBRARY_PATH), release)
    return release, library


def parameters(count=1, upper=False, final=False):
    return dict(count=count, preAction='EXTEND_DOWNLOCK' if upper else 'NONE',
                cycleMode='FINAL_RETRACT_ONLY' if final else 'FULL_CYCLE')


class GearTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.release, cls.library = fixtures()

    def test_all_protocol_leg_orders(self):
        for count in (1, 2, 3, 9, 17):
            for upper in (False, True):
                for final in (False, True):
                    p = parameters(count, upper, final)
                    r = recipe(p)
                    keys = [r['repeat']] * r['repetitions'] + ([r['tail']] if r['tail'] else [])
                    legs = []
                    for key in keys:
                        legs.extend({'cycle_lower': ['lu', 'ul'], 'cycle_upper': ['ul', 'lu']}.get(key, [key]))
                    expected = (['ul'] if upper else []) + ['lu', 'ul'] * count
                    if final:
                        expected.pop()
                    self.assertEqual(legs, expected, p)

    def test_approved_composites_identical_including_speed_arrays(self):
        for name, p in zip(BINDINGS, (parameters(2, False, True), parameters(1, True, False))):
            steps = list(self.library.program(p))
            original = self.release['payload']['actions'][name]['program']
            self.assertEqual([s['trajectory'] for s in steps if s['kind'] == 'recorded'],
                             [s['trajectory'] for s in original if s['kind'] == 'recorded'])
            self.assertEqual([s['target_q'] for s in steps if s['kind'] == 'hand'],
                             [s['target_q'] for s in original if s['kind'] == 'hand'])

    def test_large_count_lazy_and_blocks_bounded(self):
        for upper in (False, True):
            for final in (False, True):
                steps = list(islice(self.library.program(parameters(2147483647, upper, final)), 6))
                self.assertEqual(len(steps), 6)
                self.assertEqual(steps[4]['trajectory'], concatenate(
                    [self.library.units['cycle_upper' if upper else 'cycle_lower']] * 8))
                self.assertEqual(steps[5]['business_progress'], 7 if upper else 8)

    def test_progress_and_one_entry_exit(self):
        for upper in (False, True):
            for final in (False, True):
                for count in (1, 2, 9, 17):
                    steps = list(self.library.program(parameters(count, upper, final)))
                    progress = [s['business_progress'] for s in steps if s['kind'] == 'milestone']
                    self.assertEqual(progress, sorted(set(progress)))
                    self.assertEqual(progress[-1], count)
                    self.assertEqual(sum(s['kind'] == 'hand' for s in steps), 3)
                    self.assertEqual(sum(s.get('gear_phase') == 'exit' for s in steps), 1)

    def test_invalid_parameters(self):
        for change in ({'count': True}, {'count': 0}, {'count': 2147483648}, {'count': 1.0},
                       {'preAction': 'UPPER'}, {'cycleMode': 'OTHER'}, {'extra': 1}):
            with self.assertRaises(ValueError):
                recipe(dict(parameters(), **change))

    def test_gear_hand_policy_matches_wsl_without_changing_other_tasks(self):
        for name in BINDINGS:
            action = self.release['payload']['actions'][name]
            for step in action['program']:
                if step['kind'] == 'hand':
                    restored = hand_step_policy(action, step)
                    self.assertIs(restored['hand_target_required'], False)
                    self.assertEqual(restored['target_q'], step['target_q'])
                    self.assertIs(hand_step_policy(action, dict(step, hand_target_required=True))['hand_target_required'], True)
        for name in ('push', 'pull', 'dring_pair'):
            action = self.release['payload']['actions'][name]
            for step in action['program']:
                if step['kind'] == 'hand':
                    self.assertEqual(hand_step_policy(action, step), step)
        for posture in ('open', 'grip'):
            self.assertIs(self.library.hand(posture)['hand_target_required'], False)
        self.assertEqual(self.library.hand('grip')['finger_sequence'], 'four_then_thumb')
        self.assertEqual(self.library.hand('open')['finger_sequence'], 'thumb_then_four')

    def test_real_hand_policy_is_passed_by_sequence_executor(self):
        class CheckingTransport(OfflineTransport):
            def hand_transition(inner, step, check):
                self.assertIs(step.get('hand_target_required'), False)
                if step['label'].endswith(' hand grip'):
                    self.assertEqual(step.get('finger_sequence'), 'four_then_thumb')
                elif step['label'].endswith(' hand open'):
                    self.assertEqual(step.get('finger_sequence'), 'thumb_then_four')
                return super().hand_transition(step, check)
        for name in (*BINDINGS, 'gear_operation'):
            transport = CheckingTransport(0.)
            transport.initialize(self.release, lambda: None)
            sequence = SequenceExecutor(transport, offline_join)
            override = self.library.action(parameters()) if name == 'gear_operation' else None
            result = sequence.execute(self.release, name, action_override=override)
            self.assertEqual(result['status'], 'SUCCEEDED', result.get('message'))
            hand = [e['result'] for e in result['evidence'] if 'hand_target_required' in e['result']]
            self.assertEqual(len(hand), 3)
            self.assertTrue(all(e['hand_target_required'] is False and e['grasp_verified'] is False for e in hand))

    def test_tampering_and_binding_changes(self):
        original = read(LIBRARY_PATH)
        for field in ('snapshot_sha256', 'model_sha256', 'bindings'):
            data = copy.deepcopy(original)
            data['payload'][field] = {} if field == 'bindings' else 'wrong'
            data['sha256'] = digest(data['payload'])
            with self.assertRaises(ValueError):
                GearLibrary(data, self.release)
        data = copy.deepcopy(original)
        data['sha256'] = 'wrong'
        with self.assertRaises(ValueError):
            GearLibrary(data, self.release)

    def test_recovery_only_known_endpoint(self):
        for position in ('LOWER', 'UPPER', 'ABOVE'):
            tr = self.library.units['entry_lower' if position == 'ABOVE' else 'exit_' + position.lower()]
            failure = dict(kind='endpoint', gear_position=position, checkpoint=endpoint(tr, 0))
            route = self.library.recovery_route(failure)
            self.assertEqual(route['steps'][0]['kind'], 'hand')
            with self.assertRaises(ValueError):
                self.library.recovery_route(dict(failure, kind='mid-contact'))
            with self.assertRaises(ValueError):
                self.library.recovery_route(dict(failure, checkpoint={}))


class GearCloudTests(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.release, self.library = fixtures()
        self.temp = tempfile.TemporaryDirectory()
        self.transport = OfflineTransport(.001)
        self.runtime = SequenceRuntime(SequenceExecutor(self.transport, offline_join))
        from release_validation_cache import PinnedRelease
        self.pinned_release = PinnedRelease(self.release)
        self.executor = RecordedTaskExecutor(lambda: self.pinned_release, self.runtime,
            ExecutionAudit(Path(self.temp.name) / 'audit.sqlite3'), self.library)
        self.events = []

    async def asyncTearDown(self):
        await self.executor.close()
        self.temp.cleanup()

    async def report(self, progress, message):
        self.events.append(progress)

    async def run_task(self, ident, kind, p, cancel=None):
        req = SimpleNamespace(taskId=ident, taskType=kind, parameters=p, total=p.get('count', 1))
        return await self.executor.execute(req, cancel or asyncio.Event(), self.report)

    async def test_all_modes_through_executor_and_duplicate(self):
        init = await self.run_task('init', 'ROBOT_INITIALIZE', {})
        self.assertEqual(init.status, 'SUCCEEDED', init.message)
        for upper in (False, True):
            for final in (False, True):
                p = parameters(2, upper, final)
                ident = str(p)
                result = await self.run_task(ident, 'GEAR_OPERATION', p)
                self.assertEqual((result.status, result.progress), ('SUCCEEDED', 2), result.message)
                before = len(self.transport.calls)
                again = await self.run_task(ident, 'GEAR_OPERATION', p)
                self.assertEqual(again.status, 'FAILED')
                self.assertEqual(len(self.transport.calls), before)

    async def test_no_initialize_no_motion(self):
        result = await self.run_task('gear', 'GEAR_OPERATION', parameters())
        self.assertEqual(result.status, 'FAILED')
        self.assertEqual(self.transport.calls, [])

    async def test_cancel_stops_without_next_block(self):
        await self.run_task('init', 'ROBOT_INITIALIZE', {})
        self.transport.step_seconds = .1
        cancel = asyncio.Event()
        task = asyncio.create_task(self.run_task('gear', 'GEAR_OPERATION', parameters(17), cancel))
        await asyncio.sleep(.02)
        cancel.set()
        result = await asyncio.wait_for(task, 2.)
        self.assertEqual(result.status, 'CANCELLED', result.message)
        self.assertTrue(self.transport.stopped.is_set())


class GearWorkerTests(unittest.TestCase):
    def test_gear_rejection_recovers_without_replaying_failed_task(self):
        from test_automatic_recovery import RecoveringTransport
        release, library = fixtures()
        with tempfile.TemporaryDirectory() as tmp:
            transport = RecoveringTransport()
            transport.joints = dict(release['payload']['initialization']['target'])
            worker = Worker(release, transport, offline_join, Path(tmp) / 'jobs.json', library)
            try:
                worker.start(dict(id='failed-gear', kind='gear', sha256=release['sha256'],
                                  gear_sha256=library.sha256, parameters=parameters()))
                self.assertTrue(transport.recovery_started.wait(5.))
                transport.finish_recovery.set()
                worker.auto_recovery.thread.join(5.)
                self.assertEqual(worker.state()['automaticRecovery']['status'], 'ready')
                self.assertFalse(worker.state()['recoveryRequired'])
                self.assertEqual(worker.jobs['failed-gear']['result']['status'], 'FAILED')
                self.assertFalse(any(c['kind'] == 'recorded' for c in transport.calls))
            finally:
                transport.finish_recovery.set()
                worker.close()

    def test_pinned_ipc_duplicate_and_restart(self):
        release, library = fixtures()
        with tempfile.TemporaryDirectory() as tmp:
            transport = OfflineTransport(.001)
            worker = Worker(release, transport, offline_join, Path(tmp) / 'jobs.json', library)
            def finish(ident):
                deadline = time.monotonic() + 5
                while time.monotonic() < deadline:
                    with worker.lock:
                        job = copy.deepcopy(worker.jobs[ident])
                    if job['status'] == 'DONE':
                        return job
                    time.sleep(.01)
                self.fail('Worker timed out')
            try:
                worker.start(dict(id='init', kind='initialize', sha256=release['sha256']))
                self.assertEqual(finish('init')['result']['status'], 'SUCCEEDED')
                data = dict(id='gear', kind='gear', sha256=release['sha256'],
                            gear_sha256=library.sha256, parameters=parameters())
                with self.assertRaises(ValueError):
                    worker.start(dict(data, gear_sha256='wrong'))
                worker.start(data)
                self.assertEqual(finish('gear')['result']['status'], 'SUCCEEDED')
                calls = len(transport.calls)
                worker.start(data)
                self.assertEqual(len(transport.calls), calls)
                with self.assertRaises(ValueError):
                    worker.start(dict(data, parameters=parameters(2)))
            finally:
                worker.close()
            fresh = OfflineTransport(.001)
            worker = Worker(release, fresh, offline_join, Path(tmp) / 'jobs.json', library)
            try:
                worker.start(data)
                self.assertEqual(fresh.calls, [])
            finally:
                worker.close()


if __name__ == '__main__':
    unittest.main()
