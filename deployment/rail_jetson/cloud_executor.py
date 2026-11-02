"""Recorded task adapter for the existing ELN TaskManager, without a Web UI.

The runtime is injected. This module never imports ROS, acquires output authority,
or resets an interrupted sequence. Completion describes motion, not WOW sensors.
"""
import asyncio
import copy
from dataclasses import dataclass
import functools
import fcntl
import json
import sqlite3

from releases import ACTION_NAMES, GEAR, validate
from gear_protocol import recipe as eln_gear_recipe, BINDINGS as GEAR_BINDINGS
from release_validation_cache import ReleaseValidationCache
from compositions import gear_recipe, actions as recipe_actions, required as recipe_required, describe


@dataclass
class Result:
    status: str
    progress: int
    message: str
    error_code: str = None


class ExecutionAudit:
    """Durable version pin written before dispatch; never a replay queue."""
    def __init__(self, path):
        self.lock = open(str(path) + '.lock', 'a')
        try:
            fcntl.flock(self.lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            # App factories may run before the ASGI event-loop thread starts.
            # All pin/finish operations remain serialized by that event loop.
            self.db = sqlite3.connect(str(path), check_same_thread=False)
        except Exception:
            self.lock.close()
            raise
        self.db.execute('''CREATE TABLE IF NOT EXISTS recorded_executions (
            task_id TEXT PRIMARY KEY, release_sha256 TEXT NOT NULL,
            release_version TEXT NOT NULL, details TEXT NOT NULL,
            status TEXT NOT NULL, progress INTEGER NOT NULL DEFAULT 0)''')
        with self.db:
            self.db.execute("UPDATE recorded_executions SET status='OUTCOME_UNKNOWN' WHERE status='RUNNING'")

    def pin(self, request, release, gear_library=None):
        details = dict(mapping=release['payload']['mapping'],
                       external_signal_verified=False,
                       actions={name: dict(source=a['source'], plan_sha256=a['source_plan_sha256'])
                                for name, a in release['payload']['actions'].items()})
        if request.taskType == 'GEAR_OPERATION':
            if gear_library is None:
                raise ValueError('Gear protocol motion library is not installed')
            details['gear'] = gear_library.describe(request.parameters)
        if request.taskType == 'ROBOT_RETURN_HOME':
            from home_pose import reference
            details['home'] = reference()
        if request.taskType == 'GEAR_SEQUENCE':
            details['composition'] = describe(gear_recipe(request.parameters), release)
        details['task_type'] = request.taskType
        details['parameters'] = request.parameters
        with self.db:
            self.db.execute('INSERT INTO recorded_executions VALUES(?,?,?,?,?,?)',
                            (request.taskId, release['sha256'], release['payload']['version'],
                             json.dumps(details), 'RUNNING', 0))

    def finish(self, task_id, result):
        with self.db:
            self.db.execute('UPDATE recorded_executions SET status=?,progress=? WHERE task_id=?',
                            (result.status, result.progress, task_id))

    def close(self):
        self.db.close()
        self.lock.close()


class SequenceRuntime:
    """Bridge the synchronous sequence to ELN's asyncio loop.

    A transport must provide initialize(release, check) in addition to the
    SequenceExecutor transport contract. It owns preparation and recovery policy.
    """
    def __init__(self, sequence):
        self.sequence = sequence

    async def _call(self, function, *args):
        return await asyncio.get_running_loop().run_in_executor(None, functools.partial(function, *args))

    async def initialize(self, release):
        self.sequence.check()
        return await self._call(self.sequence.transport.initialize, release, self.sequence.check)

    async def return_home(self, release):
        from executor import Cancelled
        try:
            self.sequence.check()
            return await self._call(self.sequence.transport.return_home, release, self.sequence.check)
        except Cancelled as error:
            stopped = await self.cancel()
            return dict(status='CANCELLED' if stopped else 'STOP_UNCONFIRMED',
                        stop_confirmed=stopped, message=str(error))

    async def run(self, release, action, report):
        loop = asyncio.get_running_loop()

        def on_event(event):
            asyncio.run_coroutine_threadsafe(report(event), loop).result()

        return await self._call(self.sequence.execute, release, action, on_event)

    async def cancel(self):
        return await self._call(self.sequence.cancel)

    async def run_gear(self, release, library, parameters, report):
        loop = asyncio.get_running_loop()
        def on_event(event):
            asyncio.run_coroutine_threadsafe(report(event), loop).result()
        return await self._call(self.sequence.execute, release, 'gear_operation', on_event, library.action(parameters))

    async def close(self):
        pass


class RecordedTaskExecutor:
    def __init__(self, release_loader, runtime, audit, gear_library=None):
        self.release_loader, self.runtime, self.audit = release_loader, runtime, audit
        self.busy = False
        self.initialized_sha = None
        self.recovery_required = False
        self.gear_library = gear_library
        self.release_validation = ReleaseValidationCache()

    @staticmethod
    def groups(request, release):
        """Yield (actions, counts_as_one); avoid allocating count-sized lists."""
        kind, p = request.taskType, request.parameters
        if kind == 'ROBOT_INITIALIZE':
            yield ['initialize'], True
        elif kind == 'ROBOT_RETURN_HOME':
            if p or request.total != 1:
                raise ValueError('Return home takes no motion parameters')
            yield ['return_home'], True
        elif kind == 'NLG_WOW_OPERATION':
            yield [release['payload']['mapping'][p['wowSignal']]], True
        elif kind == 'NWS_CYCLE':
            for _ in range(p['count']):
                yield (['dring_pair'] if 'dring_pair' in release['payload']['actions']
                       else ['dring_left', 'dring_right']), True
        elif kind == 'GEAR_SEQUENCE':
            if request.total != 1:
                raise ValueError('A gear sequence is one business task')
            for action in recipe_actions(gear_recipe(p)):
                yield [action], False
            yield [], True
        elif kind == 'RECORDED_ACTION':
            if (set(p) != {'action'} or p['action'] not in ACTION_NAMES or request.total != 1
                    or p['action'].endswith('_segment')):
                raise ValueError('Select one complete action or one independent gear segment')
            yield [p['action']], True
        elif kind == 'GEAR_SEGMENT':
            # Internal command only: the upstream cloud field is not agreed yet.
            if set(p) != {'segment'} or p['segment'] not in GEAR or request.total != 1:
                raise ValueError('Select exactly one of the four recorded gear segments')
            yield ['gear_' + p['segment']], True
        elif kind == 'GEAR_OPERATION':
            eln_gear_recipe(p)
            yield ['gear_operation'], False
        else:
            raise ValueError('Unsupported ELN task type')

    def required_actions(self, request, release):
        if request.taskType == 'GEAR_OPERATION':
            eln_gear_recipe(request.parameters)
            if self.gear_library is None:
                raise ValueError('Gear combinations are not configured')
            if request.total != request.parameters['count']:
                raise ValueError('Gear total differs from count')
            return set(GEAR_BINDINGS)
        if request.taskType == 'GEAR_SEQUENCE':
            return recipe_required(gear_recipe(request.parameters))
        # NWS repetitions use the same two bindings regardless of count.
        sample = copy.copy(request)
        sample.parameters = dict(request.parameters)
        if 'count' in sample.parameters:
            sample.parameters['count'] = min(sample.parameters['count'], 1)
        return {name for actions, _ in self.groups(sample, release)
                for name in actions if name not in ('initialize', 'return_home')}

    async def execute(self, request, cancel, report):
        if self.busy:
            return Result('FAILED', 0, 'Recorded executor busy', '2001')
        self.busy = True
        progress, pinned, started = 0, False, False
        watcher = None
        stopped = None
        failed_outcome = None
        try:
            if self.recovery_required:
                raise ValueError('Explicit recovery required; initialize does not clear an interrupted task')
            # Large recorded arrays must not block query/cancel/callback handling.
            selected = self.release_loader()
            release = await asyncio.get_running_loop().run_in_executor(
                None, lambda: self.release_validation.snapshot(selected))
            if release['payload']['release_kind'] != 'full':
                raise ValueError('Activate a composed full release, not an individual module patch')
            if request.taskType != 'ROBOT_INITIALIZE' and self.initialized_sha != release['sha256']:
                raise ValueError('Initialize the selected release before executing tasks')
            missing = self.required_actions(request, release) - release['payload']['actions'].keys()
            if missing:
                raise ValueError('Release missing actions: ' + ', '.join(sorted(missing)))
            if request.taskType == 'GEAR_OPERATION':
                if self.gear_library is None:
                    raise ValueError('Gear protocol motion library is not installed')
                if self.gear_library.payload['bindings'] != {
                        n: release['payload']['actions'][n]['source_plan_sha256'] for n in GEAR_BINDINGS}:
                    raise ValueError('Selected gear recording changed; update the pinned gear library')
                self.audit.pin(request, release, self.gear_library)
            else:
                self.audit.pin(request, release)
            pinned = True

            async def watch_cancel():
                nonlocal stopped
                await cancel.wait()
                stopped = await self.runtime.cancel()

            watcher = asyncio.create_task(watch_cancel())

            async def stage(event):
                nonlocal progress
                if request.taskType == 'GEAR_OPERATION' and 'business_progress' in event:
                    value = event['business_progress']
                    if type(value) is not int or not progress <= value <= request.total:
                        raise ValueError('Invalid gear cycle progress')
                    progress = value
                await report(progress, '%s; action=%s; release=%s; stage=%s' %
                             (event['label'], action, release['payload']['version'], event['stage']))

            for actions, counts in self.groups(request, release):
                for action in actions:
                    if cancel.is_set():
                        await watcher
                        raise InterruptedError('Cancelled before next action')
                    started = True
                    outcome = (await self.runtime.initialize(release) if action == 'initialize' else
                               await self.runtime.run_gear(release, self.gear_library, request.parameters, stage)
                               if action == 'gear_operation' else
                               await self.runtime.return_home(release) if action == 'return_home' else
                               await self.runtime.run(release, action, stage))
                    if outcome.get('status') != 'SUCCEEDED':
                        failed_outcome = outcome
                        if outcome.get('status') == 'CANCELLED' and outcome.get('stop_confirmed') is True:
                            raise InterruptedError(outcome.get('message', 'Cancelled'))
                        raise RuntimeError('%s: %s' % (outcome.get('status'), outcome.get('message', action)))
                    if action == 'gear_operation':
                        progress = request.total
                        await report(progress, 'Gear motion cycles and recorded exit completed; external lock signals unverified')
                if counts:
                    progress += 1
                    await report(progress, 'Motion sequence completed %s/%s; release=%s; external signals unverified' %
                                 (progress, request.total, release['payload']['version']))
            if cancel.is_set():
                await watcher
                raise InterruptedError('Cancelled at task completion')
            if request.taskType == 'ROBOT_INITIALIZE':
                self.initialized_sha = release['sha256']
            result = Result('SUCCEEDED', progress, 'Motion sequence completed; release=%s; external WOW/lock signals NOT verified' %
                            release['payload']['version'])
            if request.taskType == 'ROBOT_RETURN_HOME':
                from home_pose import reference
                result.message = ('R2+A upper-body home reached; home_sha256=%s; supporting output retained; '
                                  'power-off safety NOT verified; no leg or hand motion commanded' % reference()['sha256'])
        except Exception as error:
            if started or cancel.is_set():
                self.recovery_required = True
                self.initialized_sha = None
                if watcher and cancel.is_set():
                    try:
                        await watcher
                    except Exception:
                        stopped = False
                elif failed_outcome and failed_outcome.get('stop_confirmed') is True:
                    # The worker already stopped. A duplicate STOP would cancel
                    # the separately supervised preparation recovery.
                    stopped = True
                elif started:
                    try:
                        stopped = await self.runtime.cancel()
                    except Exception:
                        stopped = False
            status = 'CANCELLED' if isinstance(error, InterruptedError) and stopped is True else 'FAILED'
            message = str(error)
            if started or cancel.is_set():
                automatic = bool(failed_outcome and failed_outcome.get('automatic_recovery') and not cancel.is_set())
                message += '; stop_confirmed=%s; %s' % (stopped is True,
                    'automatic preparation recovery scheduled; failed task will not replay' if automatic else
                    'explicit recovery required')
            result = Result(status, progress, message, '2004' if status == 'FAILED' else None)
        finally:
            if watcher and not watcher.done():
                watcher.cancel()
                await asyncio.gather(watcher, return_exceptions=True)
            self.busy = False
        if pinned:
            self.audit.finish(request.taskId, result)
        return result

    async def close(self):
        await self.runtime.close()
        self.audit.close()
