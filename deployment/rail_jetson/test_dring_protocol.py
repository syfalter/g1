import asyncio
import copy
from itertools import islice
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest

from automatic_recovery import endpoint
from bundle import digest, read
from cloud_executor import ExecutionAudit, RecordedTaskExecutor, SequenceRuntime
from dring_protocol import DringLibrary, count_of
from executor import SequenceExecutor
from native_worker import Worker
from offline_transport import OfflineTransport, offline_join
from release_validation_cache import PinnedRelease

STAGE = Path(__file__).resolve().parents[2] / 'data/rail_deployment/gear_reverse_latency_20260928'


def fixtures():
    release = read(STAGE / 'bundle.json')
    return release, DringLibrary(read(STAGE / 'dring_protocol.json'), release)


class DringTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.release, cls.library = fixtures()

    def test_one_cycle_identical(self):
        program = list(self.library.program(dict(count=1)))
        self.assertEqual(program[1]['trajectory'], self.release['payload']['actions']['dring_pair']['program'][1]['trajectory'])

    def test_multiple_only_one_entry_and_exit(self):
        for count in (2, 3, 8, 9, 17):
            program = list(self.library.program(dict(count=count)))
            labels = ','.join(s['label'] for s in program if s['kind'] == 'recorded')
            self.assertEqual(labels.count('entry'), 1)
            self.assertEqual(labels.count('final'), 1)
            self.assertEqual(labels.count('cycle'), count - 1)
            self.assertEqual(program[-1]['business_progress'], count)
            self.assertEqual(sum(s['kind'] == 'join' for s in program), 1)

    def test_large_count_is_lazy_and_bounded(self):
        program = list(islice(self.library.program(dict(count=2147483647)), 3))
        self.assertEqual(program[-1]['business_progress'], 8)
        self.assertLess(len(program[1]['trajectory']['points']), 22000)

    def test_invalid_and_tampered(self):
        for p in ({'count': 0}, {'count': True}, {'count': 1.2}, {'count': 2147483648}, {'count': 1, 'other': 0}):
            with self.assertRaises(ValueError): count_of(p)
        data = read(STAGE / 'dring_protocol.json')
        data['payload']['binding'] = 'wrong'
        data['sha256'] = digest(data['payload'])
        with self.assertRaises(ValueError): DringLibrary(data, self.release)

    def test_recovery_only_outside_contact(self):
        p = dict(count=9)
        for index, kind, unit, end in ((1, 'rejected', 'entry', 0), (3, 'endpoint', 'final', -1)):
            route = self.library.recovery_route(p, dict(index=index, kind=kind, checkpoint=endpoint(self.library.units[unit], end)))
            self.assertEqual(route['steps'], [])
        with self.assertRaises(ValueError):
            self.library.recovery_route(p, dict(index=1, kind='endpoint', checkpoint=endpoint(self.library.units['cycle'], -1)))

    def test_worker_uses_same_library_and_rejects_changed_duplicate(self):
        with tempfile.TemporaryDirectory() as tmp:
            transport = OfflineTransport(.001)
            transport.initialize(self.release, lambda: None)
            worker = Worker(self.release, transport, offline_join, Path(tmp) / 'jobs.json', dring_library=self.library)
            data = dict(id='loop', kind='dring', parameters=dict(count=2), sha256=self.release['sha256'], dring_sha256=self.library.sha256)
            try:
                worker.start(data)
                worker.thread.join(10.)
                self.assertFalse(worker.thread.is_alive())
                self.assertEqual(worker.jobs['loop']['result']['status'], 'SUCCEEDED')
                self.assertEqual(sum(c['kind'] == 'recorded' for c in transport.calls), 1)
                with self.assertRaises(ValueError): worker.start(dict(data, parameters=dict(count=3)))
            finally:
                worker.close()


class DringCloudTests(unittest.IsolatedAsyncioTestCase):
    async def test_cloud_count_progress_and_no_initialize(self):
        release, library = fixtures()
        transport = OfflineTransport(.001)
        runtime = SequenceRuntime(SequenceExecutor(transport, offline_join))
        with tempfile.TemporaryDirectory() as tmp:
            pinned = PinnedRelease(release)
            executor = RecordedTaskExecutor(lambda: pinned, runtime, ExecutionAudit(Path(tmp) / 'audit.db'), dring_library=library)
            events = []
            async def report(progress, message): events.append(progress)
            req = SimpleNamespace(taskId='loop', taskType='NWS_CYCLE', parameters=dict(count=2), total=2)
            try:
                failed = await executor.execute(req, asyncio.Event(), report)
                self.assertEqual(failed.status, 'FAILED')
                self.assertEqual(transport.calls, [])
                init = SimpleNamespace(taskId='init', taskType='ROBOT_INITIALIZE', parameters={}, total=1)
                self.assertEqual((await executor.execute(init, asyncio.Event(), report)).status, 'SUCCEEDED')
                events.clear()
                result = await executor.execute(req, asyncio.Event(), report)
                self.assertEqual((result.status, result.progress), ('SUCCEEDED', 2), result.message)
                self.assertEqual(events, sorted(events))
                self.assertEqual(sum(c['kind'] == 'recorded' for c in transport.calls), 1)
            finally:
                await executor.close()


if __name__ == '__main__': unittest.main()
