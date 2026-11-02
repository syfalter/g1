"""Production HTTP admission tests with an injected runtime; no ROS or motion."""
import asyncio
import copy
import os
from pathlib import Path
import tempfile
import time
import unittest
from unittest.mock import patch

import httpx
from fastapi.testclient import TestClient
from native_runtime import NativeRuntime
import production_api


class FakeRuntime(NativeRuntime):
    def __init__(self, *args):
        self.state = dict(simulation=True, releaseSha256=os.environ['RECOVERY_RELEASE_SHA'],
            recoveryRequired=True, recoveryGeneration=None,
            automaticRecovery=dict(status='running', message='returning to preparation'),
            native=dict(connected=True, controllerRunning=True, phase='ARMED_HOLD'))
        self.executed = []
        self.stops = 0

    async def start(self):
        self.last_state = copy.deepcopy(self.state)

    async def rpc(self, path, data=None):
        assert path == '/state'
        return copy.deepcopy(self.state)

    async def run(self, release, action, report):
        await self.wait_for_recovery(report)
        self.executed.append(action)
        return dict(status='SUCCEEDED')

    async def cancel(self):
        self.stops += 1
        self.state['automaticRecovery'] = dict(status='blocked', message='operator stop')
        self.state['recoveryRequired'] = True
        return True

    async def close(self):
        pass


class RecoveryHTTP(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.stub = FakeRuntime()
        with patch.object(production_api, 'NativeRuntime', return_value=self.stub):
            self.app = production_api.create_app(os.environ['RECOVERY_GATEWAY_ROOT'],
                os.environ['RECOVERY_TEST_BUNDLE'], Path(self.temp.name) / 'test.sqlite3',
                'offline-token', 'offline-key', allow_offline=True)
        self.client = TestClient(self.app)
        self.client.__enter__()
        async def callbacks():
            await self.app.state.manager.client.aclose()
            self.app.state.manager.client = httpx.AsyncClient(transport=httpx.MockTransport(
                lambda request: httpx.Response(200, json={'code': 0})))
        self.client.portal.call(callbacks)

    def tearDown(self):
        self.stub.state['automaticRecovery'] = dict(status='blocked', message='test teardown')
        self.client.__exit__(None, None, None)
        self.temp.cleanup()

    def submit(self, task_id):
        return self.client.post('/api/v1/actions/push/tasks', headers={'X-API-Key': 'offline-key'},
            json=dict(taskId=task_id, callbackUrl='http://callback.invalid/status'))

    def query(self, task_id):
        return self.client.post('/api/v1/tasks/' + task_id + '/query',
            headers={'X-API-Key': 'offline-key'}, json={}).json()

    def terminal(self, task_id):
        deadline = time.monotonic() + 30.
        while time.monotonic() < deadline:
            event = self.query(task_id)
            if event.get('status') in ('SUCCEEDED', 'FAILED'):
                return event
            time.sleep(.05)
        self.fail('HTTP task did not finish: ' + str(event))

    def test_accept_during_recovery_and_execute_only_after_ready(self):
        response = self.submit('new-after-failure')
        self.assertEqual(response.json()['status'], 'ACCEPTED', response.text)
        time.sleep(.2)
        self.assertEqual(self.stub.executed, [])
        self.assertFalse(self.client.get('/health').json()['initialized'])
        self.stub.state.update(recoveryRequired=False, recoveryGeneration='recovered')
        self.stub.state['automaticRecovery'] = dict(status='ready', generation='recovered')
        self.assertEqual(self.terminal('new-after-failure')['status'], 'SUCCEEDED')
        self.assertEqual(self.stub.executed, ['push'])
        self.assertEqual(self.submit('new-after-failure').json()['code'], 1003)
        self.assertEqual(self.stub.executed, ['push'])

    def test_recovery_failure_fails_waiting_task_without_motion(self):
        self.assertEqual(self.submit('waiting').json()['status'], 'ACCEPTED')
        self.stub.state['automaticRecovery'] = dict(status='blocked', message='unsafe checkpoint')
        self.assertEqual(self.terminal('waiting')['status'], 'FAILED')
        self.assertEqual(self.stub.executed, [])

    def test_explicit_stop_blocks_waiting_task(self):
        self.assertEqual(self.submit('waiting-stop').json()['status'], 'ACCEPTED')
        result = self.client.post('/api/v1/robot/stop', headers={'X-API-Key': 'offline-key'}, json={})
        self.assertTrue(result.json()['stopConfirmed'])
        self.assertEqual(self.terminal('waiting-stop')['status'], 'FAILED')
        self.assertEqual(self.stub.executed, [])


if __name__ == '__main__':
    unittest.main()
