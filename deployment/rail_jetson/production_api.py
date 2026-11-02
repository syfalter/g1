"""Jetson ELN API backed by the native ROS worker, independent of Web and WSL."""
import argparse
import os
from contextlib import asynccontextmanager
from pathlib import Path
import secrets
import sys

from cloud_executor import ExecutionAudit, RecordedTaskExecutor
from native_runtime import NativeRuntime
from releases import ACTION_NAMES, load_release
from gear_protocol import load_optional as load_gear_library


def create_app(gateway_root, release_path, database, native_token, api_key, native_port=8794, allow_offline=False):
    sys.path.insert(0, str(Path(gateway_root).resolve()))
    from fastapi import Depends, Header
    from pydantic import StrictBool, StrictInt, Field, ValidationError
    from typing import Literal, Optional
    from robot_gateway.app import create_app as gateway_app
    from robot_gateway.config import Settings
    from robot_gateway.manager import BusinessError
    from robot_gateway.models import ControlRequest, InitializeRequest, StrictModel, TaskRequest

    class ActionRequest(InitializeRequest):
        taskType: Literal['RECORDED_ACTION'] = 'RECORDED_ACTION'
        parameters: dict

        @property
        def total(self):
            return 1

    class RecoveryRequest(StrictModel):
        contactClear: StrictBool
        supportReady: StrictBool

    class ReturnHomeTask(InitializeRequest):
        taskType: Literal['ROBOT_RETURN_HOME'] = 'ROBOT_RETURN_HOME'
        parameters: dict = {}

        @property
        def total(self):
            return 1

    class ReturnHomeRequest(InitializeRequest):
        contactClear: StrictBool

    class GearParameters(StrictModel):
        startPosition: Literal['LOWER', 'UPPER']
        roundTrips: StrictInt = Field(ge=0, le=1000)
        finishPosition: Literal['LOWER', 'UPPER']
        exitToAbove: StrictBool

    class GearSequenceRequest(InitializeRequest):
        parameters: GearParameters

    class GearSequenceTask(InitializeRequest):
        taskType: Literal['GEAR_SEQUENCE'] = 'GEAR_SEQUENCE'
        parameters: dict

        @property
        def total(self):
            return 1

    release = load_release(release_path)
    if release['payload'].get('test_only') and not (
            os.environ.get('ROS_DOMAIN_ID') == '174' and os.environ.get('ROS_LOCALHOST_ONLY') == '1'):
        raise RuntimeError('Synthetic fixtures may not be used on the robot DDS domain')
    if release['payload']['release_kind'] != 'full':
        raise ValueError('Select a composed full release before starting production')
    database = Path(database)
    database.parent.mkdir(parents=True, exist_ok=True)
    runtime = NativeRuntime(native_token, native_port)
    gear_library = load_gear_library(release_path, release)
    from release_validation_cache import PinnedRelease
    pinned_release = PinnedRelease(release)
    executor = RecordedTaskExecutor(lambda: pinned_release, runtime,
        ExecutionAudit(database.with_suffix('.execution.sqlite3')), gear_library)
    app = gateway_app(Settings(database=str(database), backend='recorded-native', api_key=api_key), executor=executor)
    original_lifespan = app.router.lifespan_context

    @asynccontextmanager
    async def lifespan(application):
        entered = False
        try:
            await runtime.start()
            state = runtime.last_state
            if state['releaseSha256'] != release['sha256'] or bool(state['simulation']) != bool(allow_offline):
                raise RuntimeError('Native mode/release differs from API configuration')
            if gear_library and state.get('gearLibrarySha256') != gear_library.sha256:
                raise RuntimeError('Native/API gear libraries differ')
            async with original_lifespan(application):
                entered = True
                yield
        finally:
            if not entered:
                await executor.close()

    app.router.lifespan_context = lifespan
    app.title = 'Jetson Recorded Robot Tasks' if not allow_offline else 'Jetson Native IPC Acceptance (OFFLINE)'
    app.router.routes = [route for route in app.router.routes if getattr(route, 'path', '')
                         not in ('/health', '/api/v1/tasks', '/api/v1/simulation/config', '/api/v1/robot/initialize')]

    async def authorize(x_api_key: Optional[str] = Header(default=None)):
        if not secrets.compare_digest((x_api_key or '').encode(), api_key.encode()):
            raise BusinessError(1002, 'unauthorized', 401)

    auth = [Depends(authorize)]
    observed_recovery = None

    async def sync_recovery(state):
        nonlocal observed_recovery
        manager = app.state.manager
        async with manager.lock:
            generation = state.get('recoveryGeneration')
            automatic = state.get('automaticRecovery', {})
            if (automatic.get('status') in ('pending', 'running', 'ready') and
                    not manager.active and not executor.busy and state['native'].get('controllerRunning')):
                executor.recovery_required = False
                executor.initialized_sha = release['sha256']
                manager.initialized = True
                observed_recovery = generation
                return
            if (generation and generation != observed_recovery and not state['recoveryRequired']
                    and not manager.active and not executor.busy):
                executor.recovery_required = False
                executor.initialized_sha = None
                manager.initialized = False
                observed_recovery = generation

    @app.get('/health')
    async def health():
        state = await runtime.rpc('/state')
        await sync_recovery(state)
        manager = app.state.manager
        return dict(backend='recorded-native', simulation=state['simulation'],
                    hardwareConnected=not state['simulation'] and state['native'].get('connected', False),
                    initialized=manager.initialized and not executor.recovery_required and not state['recoveryRequired']
                        and state.get('automaticRecovery', {}).get('status') not in ('pending', 'running')
                        and state['native'].get('controllerRunning', True),
                    controllerRunning=state['native'].get('controllerRunning', True),
                    activeTaskId=manager.active, requiresRecovery=executor.recovery_required or state['recoveryRequired'],
                    releaseVersion=release['payload']['version'], releaseSha256=release['sha256'],
                    availableActions=list(release['payload']['actions']),
                    gearOperationConfigured=gear_library is not None,
                    gearLibrarySha256=gear_library.sha256 if gear_library else None,
                    recoveryGeneration=state.get('recoveryGeneration'),
                    remoteRecoveryResult=state.get('remoteRecoveryResult'),
                    automaticRecovery=state.get('automaticRecovery', {}),
                    native=state['native'], completionSemantics='motion-sequence-only')

    async def admission():
        state = await runtime.rpc('/state')
        await sync_recovery(state)
        if state.get('automaticRecovery', {}).get('status') in ('pending', 'running'):
            return
        if executor.recovery_required or state['recoveryRequired']:
            gateway = state.get('native', {}).get('gateway', {})
            reason = gateway.get('status_message', '') if gateway.get('phase') == 'LATCHED_SILENT' else ''
            raise BusinessError(2003, 'Press R2+A to recover, then initialize; HTTP recovery is optional. ' + reason)

    @app.post('/api/v1/robot/initialize', dependencies=auth)
    async def initialize(request: InitializeRequest):
        await admission()
        try:
            task = TaskRequest(taskId=request.taskId, taskType='ROBOT_INITIALIZE',
                               callbackUrl=request.callbackUrl, parameters={})
        except ValidationError:
            raise BusinessError(1001, 'invalid callbackUrl', 400)
        return await app.state.manager.submit(task)

    @app.post('/api/v1/robot/return-home', dependencies=auth)
    async def return_home(request: ReturnHomeRequest):
        """Return both arms and waist to R2+A reference. Keep output; do NOT power off.

        Requires initialized Jetson ownership, no active task, no safety latch,
        and confirmation that the arms are unloaded and the return path is clear.
        """
        await admission()
        if not request.contactClear:
            raise BusinessError(2002, 'Unload hands and clear the entire return path before return home')
        try:
            TaskRequest(taskId=request.taskId, taskType='ROBOT_INITIALIZE',
                        callbackUrl=request.callbackUrl, parameters={})
        except ValidationError:
            raise BusinessError(1001, 'invalid callbackUrl', 400)
        state = await runtime.rpc('/state')
        if not state['native'].get('controllerRunning') or state['native'].get('phase') != 'ARMED_HOLD':
            raise BusinessError(2003, 'Return home requires initialized Jetson hold; it never acquires WSL control')
        return await app.state.manager.submit(ReturnHomeTask(taskId=request.taskId, callbackUrl=request.callbackUrl))

    @app.post('/api/v1/tasks', dependencies=auth)
    async def submit(request: ControlRequest):
        await admission()
        if request.taskType == 'GEAR_OPERATION':
            if gear_library is None:
                raise BusinessError(1001, 'Gear protocol motion library is not installed', 400)
        missing = executor.required_actions(request, release) - release['payload']['actions'].keys()
        if missing:
            raise BusinessError(1001, 'No selected recording for: ' + ', '.join(sorted(missing)), 400)
        return await app.state.manager.submit(request)

    @app.post('/api/v1/gear/preview', dependencies=auth)
    async def gear_preview(parameters: GearParameters):
        """Read-only recipe inspection. Does not initialize, submit or move."""
        from compositions import gear_recipe, describe
        return dict(code=0, **describe(gear_recipe(parameters.model_dump()), release))

    @app.post('/api/v1/gear/tasks', dependencies=auth)
    async def gear_task(request: GearSequenceRequest):
        """One task: enter from ABOVE, round trips, final transfer, optional recorded exit."""
        await admission()
        try:
            TaskRequest(taskId=request.taskId, taskType='ROBOT_INITIALIZE',
                        callbackUrl=request.callbackUrl, parameters={})
        except ValidationError:
            raise BusinessError(1001, 'invalid callbackUrl', 400)
        task = GearSequenceTask(taskId=request.taskId, callbackUrl=request.callbackUrl,
                                parameters=request.parameters.model_dump())
        missing = executor.required_actions(task, release) - release['payload']['actions'].keys()
        if missing:
            raise BusinessError(1001, 'No selected recording for: ' + ', '.join(sorted(missing)), 400)
        return await app.state.manager.submit(task)

    @app.post('/api/v1/actions/{action}/tasks', dependencies=auth)
    async def action_task(action: str, request: InitializeRequest):
        await admission()
        if action not in ACTION_NAMES or action.endswith('_segment'):
            raise BusinessError(1001, 'Unknown complete action or gear segment', 400)
        if action not in release['payload']['actions']:
            raise BusinessError(1001, 'No selected recording for ' + action + '; Push and select it first', 400)
        return await app.state.manager.submit(ActionRequest(taskId=request.taskId, callbackUrl=request.callbackUrl,
                                                           parameters=dict(action=action)))

    @app.post('/api/v1/robot/stop', dependencies=auth)
    async def stop():
        manager = app.state.manager
        async with manager.lock:
            if manager.cancel:
                manager.cancel.set()
            executor.recovery_required = True
            executor.initialized_sha = None
            manager.initialized = False
        confirmed = await runtime.cancel()
        return dict(code=0 if confirmed else 2004, stopConfirmed=confirmed, requiresRecovery=True)

    @app.post('/api/v1/robot/recover', dependencies=auth)
    async def recover(request: RecoveryRequest):
        manager = app.state.manager
        async with manager.lock:
            if manager.active or executor.busy:
                raise BusinessError(2001, 'Wait for task termination before recovery')
            if not request.contactClear or not request.supportReady:
                raise BusinessError(2002, 'Contact clear and physical support must be confirmed')
            try:
                result = await runtime.recover(request.model_dump())
            except RuntimeError as error:
                raise BusinessError(2004, str(error)) from error
            executor.recovery_required = False
            executor.initialized_sha = None
            manager.initialized = False
            return dict(code=0, **result)

    @app.post('/api/v1/robot/release', dependencies=auth)
    async def release_output(request: RecoveryRequest):
        manager = app.state.manager
        async with manager.lock:
            if manager.active or executor.busy:
                raise BusinessError(2001, 'Output release requires no active task')
            if not request.contactClear or not request.supportReady:
                raise BusinessError(2002, 'Unload/support both arms and clear contact before release')
            try:
                result = await runtime.rpc('/release', request.model_dump())
            except RuntimeError as error:
                raise BusinessError(2004, str(error)) from error
            executor.recovery_required = True
            executor.initialized_sha = None
            manager.initialized = False
            return dict(code=0, **result)

    return app


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--gateway-root', required=True, type=Path)
    parser.add_argument('--release', required=True, type=Path)
    parser.add_argument('--database', required=True, type=Path)
    parser.add_argument('--native-token-file', required=True, type=Path)
    parser.add_argument('--api-key-file', required=True, type=Path)
    parser.add_argument('--native-port', type=int, default=8794)
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=8795)
    parser.add_argument('--offline-acceptance', action='store_true')
    args = parser.parse_args()
    key = args.api_key_file.read_text().strip()
    if len(key) < 32:
        raise ValueError('A dedicated API key of at least 32 characters is required')
    import uvicorn
    uvicorn.run(create_app(args.gateway_root, args.release, args.database, args.native_token_file.read_text().strip(),
                           key, args.native_port, args.offline_acceptance), host=args.host, port=args.port, workers=1)
