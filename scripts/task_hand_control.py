"""RH56 task postures and a serialized adapter to the existing HandClient service.

The joint mapping is a preview approximation, not a finger calibration. Creating
this object never opens DDS or moves a hand. Only explicit commands can actuate.
"""
import json
import math
import os
from pathlib import Path
import select
import subprocess
import threading
import time
from task_sdk_environment import sdk_environment

ROOT = Path(__file__).resolve().parents[1]
ORDER = ('little_1', 'ring_1', 'middle_1', 'index_1', 'thumb_2', 'thumb_1')
OPEN = (.15 / 1.1169,) * 4 + (0., 0.)
CLOSED = (1.2,) * 4 + (.4, .8)
POSTURES = {'open': (1.,) * 6, 'index_only': (0., 0., 0., 1., 0., 0.),
            'dring_two_finger': (0., 0., .8, .8, 1., 1.)}


def checked_q(q):
    if len(q) != 6 or not all(math.isfinite(x) and 0 <= x <= 1 for x in q):
        raise ValueError('RH56 requires six finite normalized targets in [0,1]')
    return tuple(float(x) for x in q)


def posture_targets(posture, settings=None):
    target=list(POSTURES[posture])
    if posture=='dring_two_finger' and settings is not None:
        target[2]=settings['middle_q'];target[3]=settings['index_q']
    return checked_q(target)


def model_joints(side, q):
    if side not in ('left', 'right'):
        raise ValueError('Select one hand')
    return {f'{side}_{name}_joint': closed + value * (opened - closed)
            for name, value, opened, closed in zip(ORDER, checked_q(q), OPEN, CLOSED)}


def model_feedback(side, state):
    return checked_q([(state[f'{side}_{n}_joint'] - c) / (o - c)
                      for n, o, c in zip(ORDER, OPEN, CLOSED)])


def require_state(reply, side):
    if reply.get('code') != 0 or reply.get('hand') != side or reply.get('online') is not True:
        raise RuntimeError('Hand unavailable: ' + str(reply))
    checked_q(reply.get('feedback_q', []))
    checked_q(reply.get('target_q', []))
    for key in ('error', 'temperature'):
        if len(reply.get(key, [])) != 6 or not all(math.isfinite(v) for v in reply[key]):
            raise RuntimeError('Incomplete hand feedback: ' + key)
    if any(reply['error']) or any(v >= 60 for v in reply['temperature']):
        raise RuntimeError('Hand fault/temperature limit; no automatic reset')
    if not isinstance(reply.get('timestamp_ms'), (int, float)) or reply['timestamp_ms'] <= 0:
        raise RuntimeError('Hand feedback has no valid timestamp')
    return reply


class HandControl:
    def __init__(self, network, *, enabled=False, transport=None, clock=time.monotonic, sleep=time.sleep,
                 peer_address=None):
        self.network, self.enabled = network, enabled
        self.transport, self.clock, self.sleep = transport, clock, sleep
        self.lock = threading.RLock()
        self.process = None
        self.buffer = b''
        self.sequence = time.time_ns() // 1000
        self.active = set()
        self.peer_address = peer_address
        self.stop_event = threading.Event()
        self.stop_pending = threading.Event()
        self.stop_generation = 0
        self.stop_lock = threading.Lock()
        self.stop_error = None

    def request_stop(self):
        with self.stop_lock:
            self.stop_generation += 1
            self.stop_event.set()
            self.stop_pending.set()

    def begin_motion(self, check):
        generation=self.stop_generation
        with self.lock:
            check()
            with self.stop_lock:
                if generation!=self.stop_generation or self.stop_pending.is_set() or self.stop_error:
                    raise RuntimeError('Hand STOP is not confirmed; retry hold before a new motion')
                self.stop_event.clear()

    def dds_environment(self):
        return sdk_environment(self.peer_address)

    def _exchange(self, command, check=lambda:None):
        check()
        if self.transport:
            reply=self.transport(command)
            check()
            return reply
        if self.process is None:
            binary = ROOT / 'build/task_hand_client'
            if not binary.is_file():
                raise RuntimeError('Build the hand adapter using scripts/build_task_hand_client.sh')
            # Match the vision client: SDK C++ DDS must not load ROS's DDS C library.
            environment=self.dds_environment()
            self.process = subprocess.Popen([str(binary), self.network], stdin=subprocess.PIPE,
                                            stdout=subprocess.PIPE, bufsize=0, env=environment)
        if self.process.poll() is not None:
            self.close()
            raise RuntimeError('Hand adapter exited; do not assume hand hold was acknowledged')
        check()
        with self.stop_lock:
            if command.startswith('set ') and self.stop_event.is_set():
                raise RuntimeError('Hand operation cancelled by STOP; no target replay')
            self.process.stdin.write((command + '\n').encode())
        deadline = self.clock() + 4.
        while self.clock() < deadline:
            check()
            while b'\n' in self.buffer:
                line, self.buffer = self.buffer.split(b'\n', 1)
                if line.startswith(b'TASK_HAND '):
                    return json.loads(line[len(b'TASK_HAND '):])
            readable, _, _ = select.select([self.process.stdout], [], [], .02)
            if readable:
                chunk = os.read(self.process.stdout.fileno(), 65536)
                if not chunk:
                    raise RuntimeError('Hand adapter disconnected')
                self.buffer += chunk
        # Do not reuse a stream whose late reply could be mistaken for a new RPC.
        self.process.kill(); self.process.wait()
        self.close()
        raise TimeoutError('Hand RPC timed out; hold is unconfirmed')

    def rpc(self, operation, side, q=None, check=lambda: None):
        if side not in ('left', 'right') or operation not in ('get', 'set', 'hold'):
            raise ValueError('Invalid hand operation')
        if operation != 'get' and not self.enabled:
            raise RuntimeError('Hand output disabled')
        if q is not None:
            q = checked_q(q)
        generation=self.stop_generation
        def poll_check():
            if operation!='hold' and (generation!=self.stop_generation or
                                      (operation=='set' and self.stop_event.is_set())):
                raise RuntimeError('Hand operation cancelled by STOP; no target replay')
            check()
        while not self.lock.acquire(timeout=.02):poll_check()
        try:
            # STOP invalidates the ticket before waiting for an in-flight RPC.
            poll_check()
            self.sequence += 1
            command = f'{operation} {side} {self.sequence}'
            if operation == 'set':
                if q is None:
                    raise ValueError('Missing hand target')
                self.active.add(side)
                command += ' ' + ' '.join(map(str, q))
            try:reply = self._exchange(command,poll_check)
            except Exception:
                # An interrupted stream may still have a late response. Discard
                # the adapter, never mistake that response for the subsequent hold.
                self.close(abort=True)
                raise
            if reply.get('code') == 3104:
                self.close(abort=True)
                raise TimeoutError(f'Hand {operation} {side}: DDS reply timed out (3104); '
                                   'no service acknowledgement received')
            if reply.get('code') != 0:
                raise RuntimeError('Hand RPC rejected: ' + str(reply))
            return require_state(reply, side) if operation == 'get' else reply
        finally:self.lock.release()

    def read(self, side, check=lambda: None):
        return self.rpc('get', side, check=check)

    def transition(self, side, target, check, *, duration=2., timeout=8., on_feedback=lambda state: None,
                   on_event=lambda name,**details:None):
        """Bounded command ramp, then fresh execution feedback, never ACK-only."""
        target = checked_q(target)
        if not math.isfinite(duration) or not 1 <= duration <= 10:
            raise ValueError('Hand transition duration out of range')
        began=self.clock();stable=0;evidence=[]
        try:
            first=self.read(side,check)
            check()
            on_event('initial_feedback',state=first)
            start=checked_q(first['feedback_q'])
            ramp_started=self.clock();ramp_duration=duration
            last_stamp=first['timestamp_ms'];advanced_at=ramp_started
            while self.clock() - began < timeout:
                t=min(1.,(self.clock()-ramp_started)/ramp_duration)
                check()
                blend = t*t*t*(10 + t*(-15 + 6*t))
                q = [a + blend*(b-a) for a, b in zip(start, target)]
                on_event('target_requested',target_q=q)
                self.rpc('set', side, q, check)
                on_event('target_acknowledged',target_q=q)
                state = self.read(side, check)
                on_feedback(state)
                on_event('feedback',state=state)
                if state['timestamp_ms'] > last_stamp:
                    last_stamp = state['timestamp_ms']; advanced_at = self.clock()
                    reached = t == 1 and max(abs(a-b) for a,b in zip(state['feedback_q'],target)) <= .03
                    stable = stable+1 if reached else 0
                elif self.clock()-advanced_at > .5:
                    raise RuntimeError('Hand feedback is not advancing')
                evidence.append(state)
                if stable >= 3:
                    self.rpc('hold', side, check=check)
                    on_event('hold_acknowledged')
                    return evidence
                self.sleep(.05)
            raise TimeoutError('Hand posture did not settle; arm must not proceed')
        except Exception as error:
            # Also used without a GUI; never automatically reopen a caught finger.
            error.hand_evidence=evidence
            try:
                self.rpc('hold', side)
                error.hand_hold_confirmed=True
                on_event('hold_acknowledged_after_failure')
            except Exception as hold_error:
                error.hand_hold_confirmed=False
                error.hand_hold_error=str(hold_error)
            raise

    def hold_active(self):
        self.request_stop()
        generation=self.stop_generation
        with self.lock:
            sides = sorted(self.active)
        failures = []
        for side in sides:
            try: self.rpc('hold', side)
            except Exception as error: failures.append(f'{side}: {error}')
        error='; '.join(failures) if failures else None
        with self.stop_lock:
            if generation==self.stop_generation:
                self.stop_error=error
                self.stop_pending.clear()
        if failures:raise RuntimeError('Hand STOP unconfirmed: ' + error)

    def close(self, *, abort=False):
        with self.lock:
            process=self.process
            self.process=None
            self.buffer=b''
            if process is None:return
            if abort and process.poll() is None:process.terminate()
            process.stdin.close()
            if process.poll() is None:
                try:process.wait(timeout=.2 if abort else 2)
                except subprocess.TimeoutExpired:
                    process.kill();process.wait()
            process.stdout.close()
