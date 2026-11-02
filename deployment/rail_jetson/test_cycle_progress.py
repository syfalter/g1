import asyncio
from pathlib import Path
import tempfile
from types import SimpleNamespace as S
import unittest

from bundle import read
from cloud_executor import RecordedTaskExecutor, ExecutionAudit, SequenceRuntime
from dring_protocol import DringLibrary
from executor import SequenceExecutor
from gear_progress import GearProgress
from gear_protocol import GearLibrary
from offline_transport import OfflineTransport, offline_join
from release_validation_cache import PinnedRelease

ROOT = Path(__file__).resolve().parents[2]
DATA = ROOT / 'data/rail_deployment/gear_reverse_latency_20260928'


def message(ns):
    return S(feedback=S(desired=S(time_from_start=S(sec=ns // 1000000000, nanosec=ns % 1000000000))))


class ProgressTests(unittest.TestCase):
    def test_controller_clock_pause_duplicates_and_skipped_feedback(self):
        values = []
        clock = GearProgress([dict(time_ns=n * 1000000000, progress=n) for n in (1, 2, 3)], values.append)
        for ns in (0, 500000000, 500000000):
            clock.feedback(message(ns)); clock.drain()
        self.assertEqual(values, [])
        clock.feedback(message(1000000000)); clock.drain()
        self.assertEqual(values, [1])
        for ns in (1000000000, 0, 3200000000, 3200000000):
            clock.feedback(message(ns)); clock.drain()
        self.assertEqual(values, [1, 2, 3])

    def test_each_count_across_blocks_and_gear_modes(self):
        release = read(DATA / 'bundle.json')
        dring = DringLibrary(read(DATA / 'dring_protocol.json'), release)
        gear = GearLibrary(read(DATA / 'gear_protocol.json'), release)
        for library, cases in ((dring, [dict(count=n) for n in (1, 2, 3, 8, 9, 17)]),
                (gear, [dict(count=n, preAction=pre, cycleMode=mode) for n in (1, 2, 9)
                        for pre in ('NONE', 'EXTEND_DOWNLOCK') for mode in ('FULL_CYCLE', 'FINAL_RETRACT_ONLY')])):
            for parameters in cases:
                progress = []
                for step in library.program(parameters):
                    for checkpoint in step.get('gear_progress', []):
                        progress.append(checkpoint['progress'])
                    if step['kind'] == 'milestone':
                        progress.append(step['business_progress'])
                unique = [n for i, n in enumerate(progress) if n > 0 and (i == 0 or n != progress[i-1])]
                self.assertEqual(unique, list(range(1, parameters['count'] + 1)), parameters)

    def test_metadata_does_not_change_arm_arrays(self):
        release = read(DATA / 'bundle.json')
        library = DringLibrary(read(DATA / 'dring_protocol.json'), release)
        for count in (1, 2, 3):
            step = list(library.program(dict(count=count)))[1]
            old = library.motion(['entry'] + ['cycle'] * (count - 1) + ['final'])
            self.assertEqual(step['trajectory_sha256'], old['trajectory_sha256'])
            self.assertEqual(len(step['gear_progress']), count - 1)


class ProgressiveTransport(OfflineTransport):
    def __init__(self, fail_after_first=False):
        super().__init__(.001)
        self.fail_after_first = fail_after_first
        self.inside_arm = False

    def arm(self, step, trajectory, check, report_progress=None):
        self.inside_arm = True
        try:
            if report_progress:
                clock = GearProgress(step['gear_progress'], report_progress)
                for checkpoint in step['gear_progress']:
                    clock.feedback(message(checkpoint['time_ns']))
                    clock.drain()
                    if self.fail_after_first:
                        raise RuntimeError('Injected mid-block failure')
            return super().arm(step, trajectory, check)
        finally:
            self.inside_arm = False


class CloudProgressTests(unittest.IsolatedAsyncioTestCase):
    async def exercise(self, fail):
        release = read(DATA / 'bundle.json')
        library = DringLibrary(read(DATA / 'dring_protocol.json'), release)
        transport = ProgressiveTransport(fail)
        transport.initialize(release, lambda: None)
        with tempfile.TemporaryDirectory() as temp:
            pinned = PinnedRelease(release)
            executor = RecordedTaskExecutor(lambda: pinned, SequenceRuntime(SequenceExecutor(transport, offline_join)),
                ExecutionAudit(Path(temp) / 'audit.db'), dring_library=library)
            executor.initialized_sha = release['sha256']
            events = []
            async def report(value, text): events.append((value, transport.inside_arm))
            request = S(taskId='progress-test', taskType='NWS_CYCLE', parameters=dict(count=3), total=3)
            try:
                result = await executor.execute(request, asyncio.Event(), report)
                return result, events
            finally:
                await executor.close()

    async def test_progress_arrives_before_arm_result(self):
        result, events = await self.exercise(False)
        self.assertEqual(result.status, 'SUCCEEDED', result.message)
        self.assertIn((1, True), events)
        self.assertIn((2, True), events)
        self.assertEqual(result.progress, 3)

    async def test_failure_preserves_first_completed_pair(self):
        result, events = await self.exercise(True)
        self.assertEqual((result.status, result.progress), ('FAILED', 1), result.message)
        self.assertNotIn((2, True), events)


if __name__ == '__main__': unittest.main()
