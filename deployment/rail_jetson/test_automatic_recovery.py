"""Pure offline tests: the production bundle is data, never a motion command."""
import asyncio
import copy
import json
import os
from pathlib import Path
import tempfile
import threading
import time
import unittest

from automatic_recovery import endpoint, near, recovery_route, reversed_path, TrajectoryRejected
from native_runtime import NativeRuntime
from native_worker import Worker
from offline_transport import OfflineTransport, offline_join
from ros_transport import RosTransport

ROOT = Path(__file__).resolve().parents[2]
BUNDLE = Path(os.environ.get('RECOVERY_TEST_BUNDLE', str(
    ROOT / 'data/rail_deployment/approved_mirror_20260928/bundle.json')))


class RecoveryRoutes(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.release = json.loads(BUNDLE.read_text())

    def route(self, name, index, kind):
        t = self.release['payload']['actions'][name]['program'][index]['trajectory']
        return recovery_route(self.release, name, dict(kind=kind, index=index,
            checkpoint=endpoint(t, 0 if kind == 'rejected' else -1)))

    def test_all_five_recordings_rejected_at_ready_have_safe_route(self):
        for name in self.release['payload']['actions']:
            program = self.release['payload']['actions'][name]['program']
            i = next(i for i, s in enumerate(program) if s['kind'] == 'recorded')
            route = self.route(name, i, 'rejected')
            self.assertTrue(all(s['kind'] == 'hand' for s in route['steps']))

    def test_approach_endpoint_reverses_only_approach_after_open(self):
        for name in ('pull', 'gear_roundtrip_lower_upper', 'gear_roundtrip_upper_lower'):
            route = self.route(name, 2, 'endpoint')
            self.assertTrue(route['steps'][0]['label'].endswith(' hand open'))
            original = self.release['payload']['actions'][name]['program'][2]['trajectory']
            reverse = route['steps'][1]['trajectory']
            self.assertEqual(endpoint(reverse, 0), endpoint(original, -1))
            self.assertEqual(endpoint(reverse, -1), endpoint(original, 0))
            self.assertEqual(reversed_path(reverse), original)

    def test_contact_endpoint_uses_open_then_recorded_exit_not_contact_replay(self):
        for name in ('pull', 'gear_roundtrip_lower_upper', 'gear_roundtrip_upper_lower'):
            route = self.route(name, 4, 'endpoint')
            program = self.release['payload']['actions'][name]['program']
            self.assertEqual(route['steps'], program[5:])

    def test_contact_goal_rejected_uses_reverse_approach(self):
        self.assertEqual(self.route('pull', 4, 'rejected')['steps'],
                         self.route('pull', 2, 'endpoint')['steps'])

    def test_uncertain_contact_and_tampered_checkpoints_blocked(self):
        with self.assertRaisesRegex(ValueError, 'confirmed'):
            recovery_route(self.release, 'push', {})
        with self.assertRaises(ValueError):
            recovery_route(self.release, 'pull', dict(kind='endpoint', index=4,
                checkpoint=self.release['payload']['initialization']['target']))


class RecoveringTransport(OfflineTransport):
    def __init__(self):
        super().__init__(0.)
        self.fail_next = True
        self.allow_recovery = True
        self.recovery_started = threading.Event()
        self.finish_recovery = threading.Event()

    def snapshot(self):
        return dict(connected=True, controllerRunning=True, controlOwner='jetson',
                    phase='TASK_ADMITTANCE' if self.stopped.is_set() else 'ARMED_HOLD',
                    gateway={})

    def arm(self, step, trajectory, check):
        if step['kind'] == 'recorded' and self.fail_next:
            self.fail_next = False
            raise TrajectoryRejected('Native gateway rejected trajectory: declared velocity exceeds limit')
        return super().arm(step, trajectory, check)

    def check_automatic_recovery(self, plan, release):
        if not self.allow_recovery or not near(self.joints, plan['checkpoint']):
            raise RuntimeError('unsafe recovery checkpoint')
        return True

    def recover_supported(self, check):
        check()
        self.stopped.clear()

    def execute_recovery_route(self, plan, release, check):
        self.recovery_started.set()
        while not self.finish_recovery.wait(.01):
            check()
        check()
        self.initialize(release, check)


class RecoveryWorker(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.release = json.loads(BUNDLE.read_text())

    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.transport = RecoveringTransport()
        self.transport.joints = dict(self.release['payload']['initialization']['target'])
        self.worker = Worker(self.release, self.transport, offline_join, Path(self.temp.name) / 'jobs.json')

    def tearDown(self):
        self.transport.finish_recovery.set()
        self.worker.close()
        self.temp.cleanup()

    def submit(self, name):
        self.worker.start(dict(id=name, kind='action', action='push', sha256=self.release['sha256']))

    def wait_job(self, name):
        deadline = time.monotonic() + 5.
        while time.monotonic() < deadline:
            with self.worker.lock:
                job = copy.deepcopy(self.worker.jobs[name])
            if job['status'] == 'DONE':
                return job['result']
            time.sleep(.01)
        self.fail('job timeout')

    def test_failed_task_stays_failed_recovery_does_not_replay_next_task_runs(self):
        self.submit('old')
        result = self.wait_job('old')
        self.assertEqual(result['status'], 'FAILED')
        self.assertTrue(result['automatic_recovery'])
        self.assertTrue(self.transport.recovery_started.wait(3.))
        self.assertEqual(self.worker.state()['automaticRecovery']['status'], 'running')
        self.transport.finish_recovery.set()
        self.worker.auto_recovery.thread.join(3.)
        self.assertEqual(self.worker.state()['automaticRecovery']['status'], 'ready')
        self.assertFalse(self.worker.state()['recoveryRequired'])
        self.assertFalse(any(c['kind'] == 'recorded' for c in self.transport.calls))
        self.assertEqual(self.worker.jobs['old']['result']['status'], 'FAILED')
        self.submit('new')
        self.assertEqual(self.wait_job('new')['status'], 'SUCCEEDED')
        self.assertEqual(sum(c['kind'] == 'recorded' for c in self.transport.calls), 1)

    def test_stop_during_recovery_never_reinitializes(self):
        self.submit('old')
        self.wait_job('old')
        self.assertTrue(self.transport.recovery_started.wait(3.))
        self.worker.stop()
        self.transport.finish_recovery.set()
        self.worker.auto_recovery.thread.join(3.)
        self.assertTrue(self.worker.state()['recoveryRequired'])
        self.assertEqual(self.worker.state()['automaticRecovery']['status'], 'blocked')
        self.assertFalse(any(c['kind'] == 'initialize' for c in self.transport.calls))

    def test_safety_gate_blocks_automatic_recovery(self):
        self.transport.allow_recovery = False
        self.submit('old')
        self.assertFalse(self.wait_job('old')['automatic_recovery'])
        self.assertTrue(self.worker.state()['recoveryRequired'])

    def test_restart_does_not_resume_pending_recovery(self):
        self.transport.allow_recovery = False
        self.submit('old')
        self.wait_job('old')
        self.worker.close()
        self.worker = Worker(self.release, self.transport, offline_join, self.worker.journal)
        self.assertEqual(self.worker.state()['automaticRecovery']['status'], 'idle')
        self.assertTrue(self.worker.state()['recoveryRequired'])
        self.assertFalse(self.transport.recovery_started.is_set())


class RecoveryWait(unittest.IsolatedAsyncioTestCase):
    async def test_new_task_waits_until_ready(self):
        runtime = NativeRuntime.__new__(NativeRuntime)
        states = iter([dict(recoveryRequired=True, automaticRecovery=dict(status='running')),
                       dict(recoveryRequired=False, automaticRecovery=dict(status='ready'))])
        async def rpc(*args):
            return next(states)
        runtime.rpc = rpc
        events = []
        async def report(event):
            events.append(event)
        await runtime.wait_for_recovery(report)
        self.assertEqual(len(events), 1)
        self.assertEqual(events[0]['stage'], 0)

    async def test_recovery_failure_prevents_new_dispatch(self):
        runtime = NativeRuntime.__new__(NativeRuntime)
        async def rpc(*args):
            return dict(recoveryRequired=True, automaticRecovery=dict(status='blocked', message='STOP'))
        runtime.rpc = rpc
        with self.assertRaisesRegex(RuntimeError, 'STOP'):
            await runtime.wait_for_recovery()


class RecoverySafetyGate(unittest.TestCase):
    def setUp(self):
        self.transport = RosTransport.__new__(RosTransport)
        t = self.transport
        t.lock = threading.RLock()
        t.owns_gateway = lambda: True
        t.retire_requested = False
        t.heartbeat_allowed = lambda: True
        t.state_at = t.gateway_at = t.graph_at = time.monotonic()
        t.graph_ok = True
        t.q, t.dq = dict(right=0., left=0.), dict(right=0., left=0.)
        t.gateway = dict(armed='true', service_state_ready='true', upper_body_state_safe='true',
            hardware_output_available='true', task_stop_active='true', task_stop_requested='false',
            temperature_alarm_active='false', torque_overload_active='false', command_overload_active='false',
            velocity_recovery_active='false', phase='TASK_ADMITTANCE')
        self.plan = dict(checkpoint=dict(right=0.))
        self.release = dict(payload=dict(initialization=dict(target=dict(right=0., left=0.))))

    def check(self):
        return self.transport.check_automatic_recovery(self.plan, self.release)

    def test_valid_gate_and_stop_blend_wait(self):
        self.assertTrue(self.check())
        self.transport.gateway['phase'] = 'TASK_STOP_HOLD'
        self.assertFalse(self.check())
        self.transport.gateway['phase'] = 'TASK_ADMITTANCE_BLEND'
        self.assertFalse(self.check())
        self.transport.gateway['phase'] = 'TASK_ADMITTANCE'
        self.transport.dq['right'] = .08
        self.assertFalse(self.check())

    def test_each_fault_and_missing_signal_blocks(self):
        for key, value in list(self.transport.gateway.items()):
            with self.subTest(key=key):
                self.transport.gateway.pop(key)
                with self.assertRaises(RuntimeError):
                    self.check()
                self.transport.gateway[key] = value

    def test_stale_feedback_other_owner_retirement_and_checkpoint_drift_block(self):
        t = self.transport
        for key, value in [('state_at', 0.), ('gateway_at', 0.), ('graph_at', 0.),
                           ('graph_ok', False), ('retire_requested', True)]:
            old = getattr(t, key)
            setattr(t, key, value)
            with self.assertRaises(RuntimeError):
                self.check()
            setattr(t, key, old)
        for name in t.q:
            t.q[name] = .051
            with self.assertRaises(RuntimeError):
                self.check()
            t.q[name] = 0.


if __name__ == '__main__':
    unittest.main()
