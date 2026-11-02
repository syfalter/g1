"""Offline internal-segment API acceptance; no upstream gear contract is assumed."""
import argparse
import json
from pathlib import Path
import tempfile
import time

from cloud_service import create_offline_app


def check(gateway_root, release):
    from fastapi.testclient import TestClient

    with tempfile.TemporaryDirectory() as directory:
        app = create_offline_app(gateway_root, release, Path(directory) / 'test.sqlite3', api_key='test-only')
        with TestClient(app) as client:
            headers = {'X-API-Key': 'test-only'}
            body = dict(taskId='init', callbackUrl='http://127.0.0.1:9/offline-unavailable-callback')

            def complete(task_id):
                deadline = time.monotonic() + 10
                while time.monotonic() < deadline:
                    response = client.post('/api/v1/tasks/%s/query' % task_id, json={}, headers=headers)
                    assert response.status_code == 200, response.text
                    result = response.json()
                    if result['status'] in ('SUCCEEDED', 'FAILED', 'CANCELLED'):
                        assert result['status'] == 'SUCCEEDED', result
                        assert result['progress'] == 1
                        return result
                    time.sleep(.01)
                raise TimeoutError(task_id)

            assert client.post('/api/v1/robot/initialize', json=body, headers=headers).status_code == 200
            complete('init')
            segments = ('above_to_lower', 'lower_to_upper', 'upper_to_lower', 'above_to_upper')
            for index, segment in enumerate(segments):
                path = '/internal/v1/gear-segments/%s/tasks' % segment
                body['taskId'] = segment
                assert client.post(path, json=body).status_code == 401
                response = client.post(path, json=body, headers=headers)
                assert response.status_code == 200, response.text
                complete(segment)
                calls = client.get('/internal/v1/offline/evidence', headers=headers).json()['calls']
                assert len([c for c in calls if c['kind'] == 'recorded']) == index + 1
                # Waiting is not permission to execute a next segment or return home.
                time.sleep(.05)
                assert client.get('/internal/v1/offline/evidence', headers=headers).json()['calls'] == calls
            assert client.post('/internal/v1/gear-segments/unknown/tasks', json=body, headers=headers).status_code == 400
            assert client.post('/internal/v1/gear-segments/above_to_lower/tasks',
                               json=dict(body, taskId='extra', count=2), headers=headers).status_code == 400
            assert client.post('/api/v1/tasks', headers=headers, json=dict(taskId='cloud-gear',
                callbackUrl=body['callbackUrl'], taskType='GEAR_SEGMENT', parameters=dict(segment='above_to_lower'))).status_code == 400
    return dict(passed=True, simulation=True, hardware_commands_sent=0, independent_segments=list(segments),
                cloud_segment_contract='pending', internal_endpoint_only=True,
                checks=['single_segment_only', 'no_automatic_next_or_return', 'authentication',
                        'unknown_segment_rejected', 'count_rejected', 'no_unagreed_public_cloud_type'])


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--gateway-root', type=Path, required=True)
    parser.add_argument('--release', type=Path, required=True, help='Synthetic single-recording-per-action fixture')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    result = check(args.gateway_root, args.release)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result, indent=2))
