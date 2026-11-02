"""ELN headless offline acceptance service; never starts a robot controller.

Reuses the Jetson project's HTTP API, durable task store and callback outbox.
Only the explicit offline transport is wired here. A real backend is not an
environment-variable option and must be commissioned separately.
"""
import argparse
from pathlib import Path
import secrets
import sys

from cloud_executor import ExecutionAudit, RecordedTaskExecutor, SequenceRuntime
from executor import SequenceExecutor
from offline_transport import OfflineTransport, offline_join
from releases import load_release


def create_offline_app(gateway_root, release_path, database, step_seconds=.01, api_key=None):
    import math
    if not math.isfinite(step_seconds) or not 0 < step_seconds <= 10:
        raise ValueError('Offline stage delay must be between zero and ten seconds')
    gateway_root = Path(gateway_root).resolve()
    if not (gateway_root / 'robot_gateway' / 'app.py').is_file():
        raise ValueError('Select the existing ELN gateway source root')
    sys.path.insert(0, str(gateway_root))
    from robot_gateway.app import create_app
    from robot_gateway.config import Settings
    from robot_gateway.manager import BusinessError
    from robot_gateway.models import ControlRequest, InitializeRequest
    from fastapi import Depends, Header
    from typing import Literal, Optional
    from releases import GEAR

    class SegmentRequest(InitializeRequest):
        taskType: Literal['GEAR_SEGMENT'] = 'GEAR_SEGMENT'
        parameters: dict

        @property
        def total(self):
            return 1

    # Loading once pins this process to an explicit immutable version. Web preview
    # state and future uploads cannot activate a different production version.
    release = load_release(release_path)
    database = Path(database).resolve()
    database.parent.mkdir(parents=True, exist_ok=True)
    transport = OfflineTransport(step_seconds)
    executor = RecordedTaskExecutor(lambda: release, SequenceRuntime(SequenceExecutor(transport, offline_join)),
                                    ExecutionAudit(database.with_suffix('.execution.sqlite3')))
    settings = Settings(database=str(database), backend='recorded-offline', simulation=True, api_key=api_key)
    app = create_app(settings, executor=executor)
    app.title = 'Recorded ELN Headless Acceptance (OFFLINE ONLY)'
    app.router.routes = [route for route in app.router.routes
                         if getattr(route, 'path', None) not in ('/health', '/api/v1/simulation/config', '/api/v1/tasks')]

    async def authorize(x_api_key: Optional[str] = Header(default=None)):
        if api_key and not secrets.compare_digest((x_api_key or '').encode(), api_key.encode()):
            raise BusinessError(1002, 'unauthorized', 401)

    @app.post('/api/v1/tasks', dependencies=[Depends(authorize)])
    async def submit(request: ControlRequest):
        if request.taskType == 'GEAR_OPERATION':
            raise BusinessError(1001, 'Combined gear operations are not configured; cloud segment contract pending', 400)
        if executor.recovery_required:
            raise BusinessError(2003, 'Explicit recovery required before another task')
        return await app.state.manager.submit(request)

    @app.post('/internal/v1/gear-segments/{segment}/tasks', dependencies=[Depends(authorize)])
    async def gear_segment(segment: str, request: InitializeRequest):
        if segment not in GEAR:
            raise BusinessError(1001, 'Unknown gear segment', 400)
        if executor.recovery_required:
            raise BusinessError(2003, 'Explicit recovery required before another task')
        return await app.state.manager.submit(SegmentRequest(taskId=request.taskId,
            callbackUrl=request.callbackUrl, parameters=dict(segment=segment)))

    @app.get('/health')
    async def health():
        manager = app.state.manager
        return dict(status='ok', backend='recorded-offline', simulation=True, hardwareConnected=False,
                    hardware_commands_sent=0, initialized=manager.initialized and not executor.recovery_required,
                    activeTaskId=manager.active, releaseVersion=release['payload']['version'],
                    releaseSha256=release['sha256'], requiresRecovery=executor.recovery_required,
                    protocolVersion='1.5', completionSemantics='motion-sequence-only')

    @app.get('/internal/v1/offline/evidence', dependencies=[Depends(authorize)])
    async def evidence():
        return dict(simulation=True, hardware_commands_sent=0,
                    calls=[dict(kind=c['kind'], trajectory_sha256=c.get('trajectory_sha256'))
                           for c in transport.calls])

    app.state.offline_transport = transport
    app.state.recorded_executor = executor
    return app


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--gateway-root', type=Path, required=True)
    parser.add_argument('--release', type=Path, required=True)
    parser.add_argument('--database', type=Path, required=True)
    parser.add_argument('--port', type=int, default=8793)
    args = parser.parse_args()
    import uvicorn
    uvicorn.run(create_offline_app(args.gateway_root, args.release, args.database),
                host='127.0.0.1', port=args.port, workers=1)


if __name__ == '__main__':
    main()
