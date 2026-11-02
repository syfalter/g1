"""Offline control/recovery contract. No hardware backend is enabled here."""
import copy
import json
import re
import secrets
import sqlite3
import threading
import time


class Rejected(Exception):
    def __init__(self, code, message):
        self.code = code
        super().__init__(message)


class SimulationDriver:
    hardware = False

    def __init__(self):
        self.stop_ack = True
        self.release_ack = True
        self.acquire_ack = True

    def stop(self):
        return self.stop_ack

    def release(self):
        return self.release_ack

    def acquire(self):
        return self.acquire_ack


class Coordinator:
    """An unknown stop/output acknowledgement never frees the task or ownership."""
    def __init__(self, bundle, database, driver=None, clock=time.monotonic):
        self.bundle = bundle
        self.driver = driver or SimulationDriver()
        if self.driver.hardware:
            raise ValueError('Hardware backend has not been commissioned')
        self.clock = clock
        self.lock = threading.RLock()
        self.db = sqlite3.connect(str(database), check_same_thread=False)
        self.db.execute('CREATE TABLE IF NOT EXISTS state (id INTEGER PRIMARY KEY, body TEXT NOT NULL)')
        self.db.execute('CREATE TABLE IF NOT EXISTS tasks (id TEXT PRIMARY KEY, body TEXT NOT NULL)')
        row = self.db.execute('SELECT body FROM state WHERE id=1').fetchone()
        if row and json.loads(row[0]).get('release_sha256', bundle['sha256']) != bundle['sha256']:
            self.db.close()
            raise ValueError('Persisted active release differs; load its exact verified bundle')
        self.state = json.loads(row[0]) if row else dict(
            phase='STARTUP_REQUIRED', owner='jetson', epoch=0, active=None,
            stage=None, fault=None, stop_confirmed=True, output_released=False,
            recovery_step=0, mapping=dict(bundle['payload']['mapping']), events=[], lease=None)
        if row:
            # Process restart is not a fresh robot boot, and is never a takeover grant.
            active = self.state['active']
            if active:
                task = self.task(active)
                task.update(status='FAILED', errorCode='2004', message='Service restarted; physical stop unknown')
                self._save_task(task)
            self.state.update(phase='STOP_UNCONFIRMED', owner='unknown',
                              fault='SERVICE_RESTART', stop_confirmed=False,
                              output_released=False, recovery_step=0, lease=None)
            self.state['epoch'] += 1
        self.steps = []
        self.step_index = 0
        self.next_tick = 0.
        self.worker_error = None
        self.state.update(release_sha256=bundle['sha256'],
                          release_version=bundle['payload'].get('version'))
        self._persist()

    def _persist(self):
        self.db.execute('INSERT OR REPLACE INTO state VALUES (1, ?)', (json.dumps(self.state),))
        self.db.commit()

    def _event(self, message):
        self.state['events'] = (self.state['events'] + [dict(time=time.time(), message=message)])[-60:]

    def _save_task(self, task):
        self.db.execute('INSERT OR REPLACE INTO tasks VALUES (?, ?)', (task['taskId'], json.dumps(task)))

    def task(self, task_id):
        with self.lock:
            row = self.db.execute('SELECT body FROM tasks WHERE id=?', (task_id,)).fetchone()
            if not row:
                raise Rejected(2005, 'Task not found')
            return json.loads(row[0])

    def _terminal(self, status, message):
        if self.state['active']:
            task = self.task(self.state['active'])
            task.update(status=status, progress=task.get('total', 1) if status == 'SUCCEEDED' else task['progress'],
                        message=message, errorCode='2004' if status == 'FAILED' else None)
            self._save_task(task)
        self.state['active'] = None
        self._event(message)

    def submit(self, request):
        with self.lock:
            task_id = request.get('taskId', '')
            if not isinstance(task_id, str) or not re.fullmatch(r'[A-Za-z0-9_.-]{1,128}', task_id):
                raise Rejected(1001, 'Invalid taskId')
            if self.db.execute('SELECT 1 FROM tasks WHERE id=?', (task_id,)).fetchone():
                raise Rejected(1003, 'Duplicate taskId; motion is not replayed')
            kind = request.get('taskType')
            params = request.get('parameters', {})
            if not isinstance(params, dict):
                raise Rejected(1001, 'Parameters must be an object')
            if kind == 'ROBOT_INITIALIZE':
                if params:
                    raise Rejected(1001, 'Initialization takes no parameters')
                operation = 'initialize'
            elif kind == 'NLG_WOW_OPERATION' and set(params) == {'wowSignal'} and params['wowSignal'] in ('AIR', 'GROUND'):
                operation = self.state['mapping'][params['wowSignal']]
            elif kind == 'RECORDED_ACTION' and set(params) == {'action'} and isinstance(params['action'], str):
                operation = params['action']
            elif kind == 'NWS_CYCLE' and set(params) == {'count'} and type(params['count']) is int and 1 <= params['count'] <= 100:
                operation = 'nws_cycle'
            else:
                raise Rejected(1001, 'Unsupported task or parameters; no motion accepted')
            if self.state['owner'] != 'jetson' or self.state['active']:
                raise Rejected(2001, 'Execution authority unavailable or task still occupied')
            required = ('READY', 'STARTUP_REQUIRED') if operation == 'initialize' else ('READY',)
            if self.state['phase'] not in required:
                raise Rejected(2003, 'Complete recovery/initialization first')
            actions = self.bundle['payload']['actions']
            names = (['dring_left', 'dring_right'] * params['count'] if operation == 'nws_cycle'
                     else [] if operation == 'initialize' else [operation])
            if any(name not in actions for name in names):
                raise Rejected(1001, 'Action not installed in active release')
            steps = [dict(label='双臂同时准备', duration=10.)] if operation == 'initialize' else []
            for index, name in enumerate(names):
                part = copy.deepcopy(actions[name]['plan']['segments'])
                part[-1]['completed_count'] = (index + 1) // 2 if operation == 'nws_cycle' else 1
                steps.extend(part)
            task = dict(taskId=task_id, taskType=kind, parameters=params, operation=operation,
                        status='ACCEPTED', progress=0, total=params.get('count', 1), message='accepted', errorCode=None,
                        releaseVersion=self.state['release_version'], releaseSha256=self.bundle['sha256'])
            self._save_task(task)
            self.state.update(active=task_id, phase='RUNNING', stage='ACCEPTED', stop_confirmed=False)
            self.steps = steps
            self.step_index = -1
            self.next_tick = self.clock()
            self._event('Task accepted: ' + task_id)
            self._persist()
            return dict(code=0, taskId=task_id, status='ACCEPTED', simulation=True)

    def tick(self):
        with self.lock:
            if self.state['phase'] != 'RUNNING' or self.clock() < self.next_tick:
                return
            if self.step_index >= 0 and 'completed_count' in self.steps[self.step_index]:
                task = self.task(self.state['active'])
                # Count complete left+right groups only, never a half group.
                if task['operation'] == 'nws_cycle':
                    task['progress'] = self.steps[self.step_index]['completed_count']
                    self._save_task(task)
            self.step_index += 1
            if self.step_index >= len(self.steps):
                self._terminal('SUCCEEDED', 'Simulated robot sequence complete; external WOW not measured')
                self.state.update(phase='READY', stage=None, stop_confirmed=True)
            else:
                step = self.steps[self.step_index]
                self.state['stage'] = step['label']
                task = self.task(self.state['active'])
                task.update(status='RUNNING', message=step['label'])
                self._save_task(task)
                self.next_tick = self.clock() + max(.25, step['duration'] / 20.)
                self._event(step['label'])
            self._persist()

    def stop(self, task_id=None, fault=None):
        with self.lock:
            if self.state['owner'] not in ('jetson', 'unknown'):
                raise Rejected(2001, 'WSL owns control; Jetson cannot issue stop commands')
            if task_id is not None and task_id != self.state['active']:
                task = self.task(task_id)
                if task['status'] in ('SUCCEEDED', 'FAILED', 'CANCELLED'):
                    return dict(code=0, status=task['status'])
                raise Rejected(2001, 'Task is not active')
            self.state.update(phase='STOP_UNCONFIRMED', stop_confirmed=False,
                              fault=fault or 'CANCELLED', recovery_step=0)
            self._persist()
            if self.driver.stop():
                self.state.update(phase='RECOVERY_REQUIRED', stop_confirmed=True)
                self._terminal('FAILED' if fault else 'CANCELLED', fault or 'Controlled stop confirmed')
            self._persist()
            return dict(code=0, phase=self.state['phase'], stopConfirmed=self.state['stop_confirmed'])

    def recovery(self, step, contact_clear=False, support_ready=False):
        with self.lock:
            if step == 'confirm_stop':
                if self.state['phase'] not in ('STOP_UNCONFIRMED', 'RECOVERY_REQUIRED'):
                    raise Rejected(2002, 'Recovery is not pending')
                if not self.driver.stop():
                    raise Rejected(2004, 'Stop remains unconfirmed; control stays occupied')
                self._terminal('FAILED', self.state['fault'] or 'Recovery after unknown stop')
                self.state.update(phase='RECOVERY_REQUIRED', stop_confirmed=True, recovery_step=1)
            elif step == 'inspect':
                if self.state['phase'] != 'RECOVERY_REQUIRED' or self.state['recovery_step'] != 1:
                    raise Rejected(2002, 'Confirm stop first')
                if contact_clear is not True or support_ready is not True:
                    raise Rejected(2002, 'Contact and physical support confirmations required')
                self.state['recovery_step'] = 2
            elif step == 'reset':
                if self.state['phase'] != 'RECOVERY_REQUIRED' or self.state['recovery_step'] != 2:
                    raise Rejected(2002, 'Complete inspection first')
                if self.state['owner'] == 'unknown':
                    raise Rejected(2002, 'Ownership unknown after restart; reconcile both controllers first')
                self.state.update(phase='STARTUP_REQUIRED', fault=None, stage=None, recovery_step=0)
            elif step == 'reconcile_owner':
                if (self.state['owner'] != 'unknown' or self.state['recovery_step'] != 2
                        or self.state['phase'] != 'RECOVERY_REQUIRED'):
                    raise Rejected(2002, 'Inspect before reconciling unknown ownership')
                if support_ready is not True or not self.driver.release():
                    raise Rejected(2004, 'Both output owners must be confirmed released')
                self.state.update(owner='none', phase='AWAITING_JETSON', output_released=True, lease=None)
                self.state['epoch'] += 1
            else:
                raise Rejected(1001, 'Unknown recovery step')
            self._event('Recovery: ' + step)
            self._persist()
            return dict(code=0, phase=self.state['phase'])

    def handoff(self, action, epoch, token=None, support_ready=False):
        with self.lock:
            if type(epoch) is not int or epoch != self.state['epoch']:
                raise Rejected(2002, 'Stale ownership generation')
            if self.state['active']:
                raise Rejected(2001, 'Stop/finish the active task first')
            if action == 'release_jetson':
                if self.state['owner'] != 'jetson' or self.state['phase'] not in ('READY', 'STARTUP_REQUIRED'):
                    raise Rejected(2002, 'Recovery or handoff is pending')
                if support_ready is not True:
                    raise Rejected(2002, 'Confirm physical support before releasing output')
                self.state.update(phase='HANDOFF_PENDING', stop_confirmed=False)
                self._persist()
                if not self.driver.stop() or not self.driver.release():
                    self.state.update(phase='STOP_UNCONFIRMED', fault='OUTPUT_RELEASE_UNCONFIRMED')
                    self._persist()
                    raise Rejected(2004, 'Output release unconfirmed; no WSL grant issued')
                self.state.update(owner='none', phase='AWAITING_WSL', output_released=True,
                                  stop_confirmed=True, lease=secrets.token_urlsafe(24))
                self.state['epoch'] += 1
            elif action == 'claim_wsl':
                if (self.state['phase'] != 'AWAITING_WSL' or not self.state['output_released']
                        or not token or not secrets.compare_digest(token, self.state['lease'] or '')):
                    raise Rejected(2002, 'No matching WSL handoff grant')
                self.state.update(owner='wsl', phase='WSL_OWNED')
            elif action == 'release_wsl':
                if self.state['phase'] != 'WSL_OWNED' or not token or not secrets.compare_digest(token, self.state['lease'] or ''):
                    raise Rejected(2002, 'Only the WSL holder can acknowledge release')
                if support_ready is not True or not self.driver.release():
                    raise Rejected(2004, 'WSL output release not confirmed')
                self.state.update(owner='none', phase='AWAITING_JETSON', lease=None)
                self.state['epoch'] += 1
            elif action == 'acquire_jetson':
                if self.state['phase'] != 'AWAITING_JETSON' or self.state['owner'] != 'none':
                    raise Rejected(2002, 'WSL release acknowledgement required; no forced takeover')
                if not self.driver.acquire():
                    raise Rejected(2004, 'Jetson takeover unconfirmed')
                self.state.update(owner='jetson', phase='STARTUP_REQUIRED', output_released=False,
                                  fault=None, stage=None, recovery_step=0)
                self.state['epoch'] += 1
            else:
                raise Rejected(1001, 'Unknown handoff action')
            self._event('Authority: ' + action)
            self._persist()
            return dict(code=0, owner=self.state['owner'], epoch=self.state['epoch'], token=self.state['lease'])

    def set_mapping(self, mapping):
        with self.lock:
            if self.state['active'] or self.state['phase'] not in ('READY', 'STARTUP_REQUIRED'):
                raise Rejected(2001, 'Change mapping only while idle')
            if mapping not in ({'AIR': 'push', 'GROUND': 'pull'}, {'AIR': 'pull', 'GROUND': 'push'}):
                raise Rejected(1001, 'Mapping must pair AIR/GROUND with push/pull')
            self.state['mapping'] = dict(mapping)
            self._event('Mapping changed')
            self._persist()

    def activate_release(self, envelope, expected_sha256):
        from releases import validate
        validate(envelope)
        with self.lock:
            if (self.state['active'] or self.state['owner'] != 'jetson'
                    or self.state['phase'] not in ('READY', 'STARTUP_REQUIRED')):
                raise Rejected(2001, 'Release switch requires idle Jetson ownership and no pending recovery')
            if expected_sha256 != self.bundle['sha256']:
                raise Rejected(2002, 'Active release changed; refresh before switching')
            if envelope['sha256'] == self.bundle['sha256']:
                return dict(code=0, unchanged=True)
            previous = dict(version=self.state['release_version'], sha256=self.bundle['sha256'])
            state = copy.deepcopy(self.state)
            state.update(release_version=envelope['payload']['version'], release_sha256=envelope['sha256'],
                         previous_release=previous, phase='STARTUP_REQUIRED', stage=None)
            state['events'] = (state['events'] + [dict(time=time.time(), message='Release activated: ' +
                                                        envelope['payload']['version'])])[-60:]
            self.db.execute('INSERT OR REPLACE INTO state VALUES (1, ?)', (json.dumps(state),))
            self.db.commit()
            self.state, self.bundle = state, envelope
            return dict(code=0, version=state['release_version'], requiresInitialization=True)

    def snapshot(self):
        with self.lock:
            state = copy.deepcopy(self.state)
            state.pop('lease', None)
            state.update(simulation=True, hardwareConnected=False, bundle=self.bundle['sha256'],
                         availableActions=list(self.bundle['payload']['actions']),
                         workerError=self.worker_error,
                         tasks=[json.loads(r[0]) for r in self.db.execute('SELECT body FROM tasks ORDER BY rowid DESC LIMIT 10')])
            return state

    def close(self):
        with self.lock:
            self.db.close()
