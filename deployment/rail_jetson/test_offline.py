import copy
import json
from pathlib import Path
import tempfile
import threading
import unittest
from urllib.error import HTTPError
from urllib.request import Request, urlopen

from bundle import ROOT, digest, load, validate_trajectory
from coordinator import Coordinator, Rejected, SimulationDriver
from parity import compare
from server import create_server

BUNDLE = ROOT / 'data/rail_deployment/20260921_offline_v1/bundle.json'


class WorkflowTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.database = Path(self.temp.name) / 'state.sqlite3'
        self.now = 0.
        self.driver = SimulationDriver()
        self.c = Coordinator(load(BUNDLE), self.database, self.driver, lambda: self.now)

    def tearDown(self):
        self.c.close()
        self.temp.cleanup()

    def submit(self, task_id='test', signal='AIR'):
        return self.c.submit(dict(taskId=task_id, taskType='NLG_WOW_OPERATION', parameters={'wowSignal': signal}))

    def ready(self):
        self.c.submit(dict(taskId='init', taskType='ROBOT_INITIALIZE', parameters={}))
        self.c.tick()
        self.now += 1000
        self.c.tick()
        self.assertEqual(self.c.state['phase'], 'READY')

    def handoff(self, action, token=None, support=True):
        return self.c.handoff(action, self.c.state['epoch'], token, support)

    def test_requires_initialization(self):
        with self.assertRaises(Rejected): self.submit()

    def test_success_only_after_all_steps(self):
        self.ready(); self.submit()
        self.assertEqual(self.c.task('test')['status'], 'ACCEPTED')
        for _ in self.c.steps:
            self.now += 1000; self.c.tick()
            self.assertEqual(self.c.task('test')['status'], 'RUNNING')
            self.assertEqual(self.c.task('test')['progress'], 0)
        self.now += 1000; self.c.tick()
        self.assertEqual(self.c.task('test')['status'], 'SUCCEEDED')
        self.assertEqual(self.c.task('test')['progress'], 1)

    def test_concurrent_tasks_are_rejected(self):
        self.ready(); self.submit()
        with self.assertRaises(Rejected): self.submit('second')

    def test_duplicate_never_replayed(self):
        self.ready(); self.submit(); self.c.stop()
        with self.assertRaises(Rejected) as error: self.submit()
        self.assertEqual(error.exception.code, 1003)

    def test_unconfirmed_stop_retains_active_slot(self):
        self.ready(); self.submit(); self.driver.stop_ack = False
        self.c.stop()
        self.assertEqual(self.c.state['phase'], 'STOP_UNCONFIRMED')
        self.assertEqual(self.c.state['active'], 'test')
        with self.assertRaises(Rejected): self.submit('second')
        with self.assertRaises(Rejected): self.c.recovery('confirm_stop')
        self.driver.stop_ack = True
        self.c.recovery('confirm_stop')
        self.assertIsNone(self.c.state['active'])

    def test_recovery_requires_all_steps_and_new_initialization(self):
        self.ready(); self.submit(); self.c.stop(fault='fault')
        with self.assertRaises(Rejected): self.c.recovery('reset')
        with self.assertRaises(Rejected): self.c.recovery('inspect', True, True)
        self.c.recovery('confirm_stop')
        with self.assertRaises(Rejected): self.c.recovery('inspect', True, False)
        self.c.recovery('inspect', True, True)
        self.c.recovery('reset')
        self.assertEqual(self.c.state['phase'], 'STARTUP_REQUIRED')
        with self.assertRaises(Rejected): self.submit('next')

    def test_handoff_requires_release_ack_and_support(self):
        with self.assertRaises(Rejected): self.handoff('release_jetson', support=False)
        self.driver.release_ack = False
        with self.assertRaises(Rejected): self.handoff('release_jetson')
        self.assertIsNone(self.c.state['lease'])
        self.assertNotEqual(self.c.state['owner'], 'wsl')

    def test_handoff_never_preempts_running_task(self):
        self.ready(); self.submit()
        with self.assertRaises(Rejected): self.handoff('release_jetson')

    def test_full_handoff_no_timeout_reclaim(self):
        grant = self.handoff('release_jetson')
        with self.assertRaises(Rejected): self.handoff('claim_wsl', 'wrong')
        self.handoff('claim_wsl', grant['token'])
        self.now += 1000000; self.c.tick()
        self.assertEqual(self.c.state['owner'], 'wsl')
        with self.assertRaises(Rejected): self.handoff('acquire_jetson')
        with self.assertRaises(Rejected): self.submit()
        self.handoff('release_wsl', grant['token'])
        self.handoff('acquire_jetson')
        self.assertEqual(self.c.state['phase'], 'STARTUP_REQUIRED')

    def test_stale_ownership_generation_rejected(self):
        self.handoff('release_jetson')
        with self.assertRaises(Rejected): self.c.handoff('claim_wsl', 0, self.c.state['lease'])

    def test_restart_does_not_replay_or_reclaim(self):
        self.ready(); self.submit(); self.c.close()
        self.c = Coordinator(load(BUNDLE), self.database, self.driver)
        self.assertEqual(self.c.task('test')['status'], 'FAILED')
        self.assertEqual(self.c.state['owner'], 'unknown')
        self.assertEqual(self.c.state['phase'], 'STOP_UNCONFIRMED')
        with self.assertRaises(Rejected): self.submit('new')
        self.c.recovery('confirm_stop'); self.c.recovery('inspect', True, True)
        with self.assertRaises(Rejected): self.c.recovery('reset')
        self.c.recovery('reconcile_owner', support_ready=True)
        self.handoff('acquire_jetson')
        self.assertEqual(self.c.state['phase'], 'STARTUP_REQUIRED')

    def test_mapping_changes_only_idle_and_persists(self):
        mapping = {'AIR': 'pull', 'GROUND': 'push'}
        self.c.set_mapping(mapping); self.ready(); self.submit()
        self.assertEqual(self.c.task('test')['operation'], 'pull')
        with self.assertRaises(Rejected): self.c.set_mapping({'AIR':'push','GROUND':'pull'})
        self.c.close(); self.c = Coordinator(load(BUNDLE), self.database)
        self.assertEqual(self.c.state['mapping'], mapping)

    def test_other_business_never_fakes_success(self):
        with self.assertRaises(Rejected):
            self.c.submit(dict(taskId='other', taskType='NWS_CYCLE', parameters={'count':1}))

    def test_bundle_has_six_recordings_and_original_joint_arrays(self):
        payload = self.c.bundle['payload']
        self.assertEqual(len(payload['initialization']['target']), 15)
        for action in payload['actions'].values():
            self.assertEqual(len([s for s in action['plan']['segments'] if s.get('trace_stage')]), 3)
            self.assertEqual(action['plan']['planning_config']['values']['waist_velocity_rad_s'], .285)
            for s in action['plan']['segments']: validate_trajectory(s['trajectory'])

    def test_bundle_corruption_is_rejected(self):
        envelope = copy.deepcopy(self.c.bundle)
        envelope['payload']['initialization']['velocity_scale'] = 1.
        path = Path(self.temp.name) / 'corrupt.json'; path.write_text(json.dumps(envelope))
        with self.assertRaises(ValueError): load(path)

    def test_hardware_driver_explicitly_refused(self):
        self.driver.hardware = True
        with self.assertRaises(ValueError): Coordinator(load(BUNDLE), self.database, self.driver)

    def test_dual_preparation_is_synchronized_and_not_hardware_authorized(self):
        prepared = json.loads((BUNDLE.parent / 'dual_prepare.json').read_text())
        trajectory = prepared['trajectory']
        self.assertFalse(prepared['hardware_executable'])
        self.assertEqual(len(trajectory['joint_names']), 15)
        validate_trajectory(trajectory)
        target = self.c.bundle['payload']['initialization']['target']
        for name, position in zip(trajectory['joint_names'], trajectory['points'][-1]['positions']):
            self.assertAlmostEqual(position, target[name], places=12)
        for point in (trajectory['points'][0], trajectory['points'][-1]):
            self.assertTrue(all(abs(v) < 1e-10 for v in point['velocities'] + point['accelerations']))

    def test_cross_architecture_report(self):
        report = compare(BUNDLE.parent / 'x86.output', BUNDLE.parent / 'arm64.output',
                         BUNDLE.parent / 'interpolation.input.json')
        self.assertTrue(report['passed'])
        self.assertEqual(report['segments'], 16)


class HttpTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.server = create_server(BUNDLE, Path(self.temp.name) / 'state.db', port=0)
        self.thread = threading.Thread(target=self.server.serve_forever)
        self.thread.start()
        self.base = 'http://127.0.0.1:' + str(self.server.server_port)

    def tearDown(self):
        self.server.shutdown(); self.thread.join(); self.server.server_close()
        self.server.controller.close(); self.temp.cleanup()

    def request(self, path, body=None, token=None, origin=None):
        headers = {}
        if token: headers['X-Preview-Token'] = token
        if origin: headers['Origin'] = origin
        request = Request(self.base + path, data=None if body is None else json.dumps(body).encode(), headers=headers)
        with urlopen(request, timeout=3) as response: return json.load(response)

    def test_simulation_state_and_auth(self):
        self.assertFalse(self.request('/api/state')['hardwareConnected'])
        with self.assertRaises(HTTPError): self.request('/api/stop', {})
        token = self.request('/api/session')['token']
        with self.assertRaises(HTTPError): self.request('/api/stop', {}, token, 'http://evil.invalid')
        response = self.request('/api/task', dict(taskId='init',taskType='ROBOT_INITIALIZE',parameters={}), token)
        self.assertEqual(response['status'], 'ACCEPTED')


if __name__ == '__main__':
    unittest.main()
