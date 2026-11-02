"""Async cloud-to-Foxy loopback bridge. Never retries an uncertain motion POST."""
import asyncio
import uuid


class NativeRuntime:
    def __init__(self, token, port=8794):
        import httpx
        self.client = httpx.AsyncClient(base_url='http://127.0.0.1:' + str(port), trust_env=False,
                                        headers={'X-Native-Token': token}, timeout=30.)
        self.heartbeat_task = None
        self.last_state = None
        self.heartbeat_error = None

    async def rpc(self, path, data=None):
        response = await self.client.post(path, json=data or {})
        if response.status_code == 409:
            raise RuntimeError(response.json().get('error', 'Native request rejected'))
        response.raise_for_status()
        result = response.json()
        if 'error' in result:
            raise RuntimeError(result['error'])
        return result

    async def start(self):
        self.last_state = await self.rpc('/heartbeat')
        self.heartbeat_task = asyncio.create_task(self.heartbeat())

    async def heartbeat(self):
        while True:
            await asyncio.sleep(1.)
            try:
                self.last_state = await self.rpc('/heartbeat')
                self.heartbeat_error = None
            except Exception as error:
                self.heartbeat_error = str(error)

    async def job(self, release, kind, action=None, report=None, parameters=None, gear_sha256=None):
        if self.heartbeat_task is None:
            raise RuntimeError('Native runtime has not started')
        job_id = uuid.uuid4().hex
        offset = 0
        try:
            await self.wait_for_recovery(report)
            body = dict(id=job_id, kind=kind, action=action, sha256=release['sha256'])
            if kind == 'gear':
                body.update(parameters=parameters, gear_sha256=gear_sha256)
            await self.rpc('/jobs', body)
            while True:
                job = await self.rpc('/job', dict(id=job_id))
                for event in job['events'][offset:]:
                    if report:
                        await report(event)
                offset = len(job['events'])
                if job['status'] == 'DONE':
                    return job['result']
                await asyncio.sleep(.1)
        except BaseException:
            # Even a failed acknowledgement may mean native motion was accepted.
            # STOP is allowed to repeat; motion submission is not.
            try:
                await asyncio.shield(self.cancel())
            except Exception:
                pass
            raise

    async def wait_for_recovery(self, report=None):
        deadline = asyncio.get_running_loop().time() + 180.
        reported = False
        while True:
            state = await self.rpc('/state')
            recovery = state.get('automaticRecovery', {})
            if recovery.get('status') not in ('pending', 'running'):
                if state.get('recoveryRequired'):
                    raise RuntimeError('Recovery is blocked: ' + recovery.get('message', 'explicit recovery required'))
                return
            if report and not reported:
                await report(dict(stage=0, label='Restoring preparation before the new task'))
                reported = True
            if asyncio.get_running_loop().time() > deadline:
                raise TimeoutError('Automatic recovery wait expired; no new motion submitted')
            await asyncio.sleep(.1)

    async def initialize(self, release):
        return await self.job(release, 'initialize')

    async def return_home(self, release):
        return await self.job(release, 'return_home')

    async def run(self, release, action, report):
        return await self.job(release, 'action', action, report)

    async def run_gear(self, release, library, parameters, report):
        return await self.job(release, 'gear', report=report, parameters=parameters, gear_sha256=library.sha256)

    async def cancel(self):
        return (await self.rpc('/stop')).get('stop_confirmed') is True

    async def recover(self, data):
        return await self.rpc('/recover', data)

    async def close(self):
        if self.heartbeat_task:
            self.heartbeat_task.cancel()
            await asyncio.gather(self.heartbeat_task, return_exceptions=True)
        await self.client.aclose()
