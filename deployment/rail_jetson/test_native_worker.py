import copy
from pathlib import Path
import tempfile
import time
import unittest

from native_worker import Worker
from native_acceptance import fixture as native_fixture
from offline_transport import OfflineTransport, offline_join
from test_cloud_executor import release


class WorkerTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.bundle = native_fixture(release())
        self.transport = OfflineTransport(.002)
        self.worker = Worker(self.bundle, self.transport, offline_join, Path(self.temp.name) / 'jobs.json')

    def tearDown(self):
        self.worker.close()
        self.temp.cleanup()

    def submit(self, job_id, action=None):
        return self.worker.start(dict(id=job_id, kind='action' if action else 'initialize',
                                     action=action, sha256=self.bundle['sha256']))

    def finish(self, job_id):
        deadline = time.monotonic() + 3.
        while time.monotonic() < deadline:
            with self.worker.lock:
                result = copy.deepcopy(self.worker.jobs[job_id])
            if result['status'] == 'DONE':
                return result
            time.sleep(.01)
        self.fail('Worker did not finish')

    def test_startup_does_not_initialize(self):
        self.assertEqual(self.transport.calls, [])

    def test_home_dispatch_and_duplicate_do_not_initialize(self):
        self.submit('init')
        self.finish('init')
        data = dict(id='home', kind='return_home', sha256=self.bundle['sha256'])
        self.worker.start(data)
        self.assertEqual(self.finish('home')['result']['status'], 'SUCCEEDED')
        count = len(self.transport.calls)
        self.worker.start(data)
        self.assertEqual(len(self.transport.calls), count)
        self.assertEqual([c['kind'] for c in self.transport.calls], ['initialize', 'return_home'])

    def test_cancel_home_retains_recovery_requirement(self):
        self.submit('init')
        self.finish('init')
        self.transport.step_seconds = .5
        self.worker.start(dict(id='home', kind='return_home', sha256=self.bundle['sha256']))
        time.sleep(.02)
        self.worker.sequence.cancel()
        self.assertEqual(self.finish('home')['result']['status'], 'CANCELLED')
        self.assertTrue(self.worker.state()['recoveryRequired'])

    def test_all_eight_bindings_without_execution_history(self):
        self.submit('init')
        self.assertEqual(self.finish('init')['result']['status'], 'SUCCEEDED')
        for index, action in enumerate(self.bundle['payload']['actions']):
            self.submit(action, action)
            self.assertEqual(self.finish(action)['result']['status'], 'SUCCEEDED')
            self.assertEqual(len([c for c in self.transport.calls if c['kind'] == 'recorded']), index + 1)

    def test_duplicate_job_returns_evidence_without_replay(self):
        self.submit('init')
        self.finish('init')
        self.submit('left', 'dring_left')
        self.finish('left')
        count = len(self.transport.calls)
        result = self.submit('left', 'dring_left')
        self.assertEqual(result['status'], 'DONE')
        self.assertEqual(len(self.transport.calls), count)
        with self.assertRaises(ValueError):
            self.submit('left', 'dring_right')

    def test_release_hash_mismatch_never_dispatches(self):
        with self.assertRaises(ValueError):
            self.worker.start(dict(id='wrong', kind='initialize', sha256='a' * 64))
        self.assertEqual(self.transport.calls, [])

    def test_lost_coordinator_stops_without_replay(self):
        self.submit('init')
        self.finish('init')
        self.transport.step_seconds = 2.
        self.submit('left', 'dring_left')
        with self.worker.lock:
            self.worker.last_heartbeat = time.monotonic() - 11.
        result = self.finish('left')
        self.assertEqual(result['result']['status'], 'CANCELLED')
        self.assertTrue(self.worker.sequence.cancelled.is_set())
        with self.assertRaises(RuntimeError):
            self.submit('next', 'dring_right')

    def test_recovery_requires_physical_confirmations(self):
        with self.assertRaises(ValueError):
            self.worker.recover(dict(contactClear=False, supportReady=True))
        with self.assertRaises(ValueError):
            self.worker.release_output(dict(contactClear=True, supportReady=False))

    def test_recovery_requirement_survives_process_restart(self):
        self.worker.mark_recovery('Historical native L2+B latch')
        self.worker.close()
        self.worker = Worker(self.bundle, OfflineTransport(.002), offline_join, self.worker.journal)
        self.assertTrue(self.worker.state()['recoveryRequired'])
        with self.assertRaisesRegex(RuntimeError, 'recovery required'):
            self.submit('init-after-restart')
        self.worker.transport.recover = lambda: dict(recovered=True, requiresInitialization=True)
        self.worker.recover(dict(contactClear=True, supportReady=True))
        self.assertFalse(self.worker.state()['recoveryRequired'])

    def test_physical_recovery_without_http_confirmation_or_replay(self):
        self.worker.mark_recovery('L2+B')
        self.transport.claim_remote_recovery = lambda: True
        self.transport.recover_from_remote = lambda: dict(recovered=True)
        self.worker.poll_remote_recovery()
        state = self.worker.state()
        self.assertFalse(state['recoveryRequired'])
        self.assertTrue(state['recoveryGeneration'])
        self.assertEqual(self.transport.calls, [])

    def test_new_stop_during_recovery_remains_required(self):
        self.worker.mark_recovery('L2+B')
        self.transport.claim_remote_recovery = lambda: True
        def newer_stop():
            self.worker.mark_recovery('New STOP')
            return dict(recovered=True)
        self.transport.recover_from_remote = newer_stop
        with self.assertRaisesRegex(RuntimeError, 'newer STOP'):
            self.worker.poll_remote_recovery()
        self.assertTrue(self.worker.state()['recoveryRequired'])
        self.assertIsNone(self.worker.state()['recoveryGeneration'])


if __name__ == '__main__':
    unittest.main()
