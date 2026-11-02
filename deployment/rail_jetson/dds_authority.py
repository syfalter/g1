"""Identify the robot's idle SDK endpoint, never blanket-allow bare DDS apps."""
import json
from pathlib import Path
import subprocess
import threading
import time


LOWCMD = 'unitree_hg::msg::dds_::LowCmd_'


def native_endpoints(state):
    peers = state['participants']
    result = set()
    for writer in state['writers']:
        peer = peers.get(writer['participant'], {})
        receives = any(r['participant'] == writer['participant'] and r['type'] == LOWCMD
                       for r in state['readers'])
        if (peer.get('process') == 'g1_arm_example' and peer.get('host') == 'Unitree'
                and writer['type'] == LOWCMD and receives):
            result.add(writer['guid'])
    if len(result) != 1:
        raise RuntimeError('Expected exactly one identified onboard g1_arm_example endpoint')
    return result


def validate(state, pinned, own=(), past_own=()):
    if native_endpoints(state) != pinned:
        raise RuntimeError('Onboard SDK identity changed; restart only after confirming ownership')
    if state.get('unknown_messages') or any(state['messages'].get(g, 0) for g in pinned):
        raise RuntimeError('Onboard/unknown publisher sent arm commands; competing output is blocked')
    allowed = pinned | set(own)
    foreign = {w['guid'] for w in state['writers']} - allowed
    if foreign or set(state['messages']) - allowed - set(past_own):
        raise RuntimeError('Another arm command source is present: ' + ','.join(sorted(foreign)))


class DdsAuthority:
    def __init__(self, root):
        self.lock = threading.Lock()
        self.state, self.at, self.error, self.pinned = {}, 0., '', None
        self.past_own = set()
        self.closed = False
        self.process = subprocess.Popen([str(Path(root) / 'build/task_dds_observer')],
                                        stdout=subprocess.PIPE, text=True, bufsize=1)
        self.thread = threading.Thread(target=self.read, daemon=True)
        self.thread.start()

    def read(self):
        try:
            for line in self.process.stdout:
                state = json.loads(line)
                with self.lock:
                    self.state, self.at = state, time.monotonic()
        except Exception as error:
            with self.lock:
                self.error = str(error)

    def check(self, own=()):
        with self.lock:
            if self.error or self.process.poll() is not None or time.monotonic() - self.at > 1.:
                raise RuntimeError('DDS source observer unavailable: ' + self.error)
            if self.state['observed_seconds'] < 3.:
                raise RuntimeError('DDS source observation warming up')
            if self.pinned is None:
                pinned = native_endpoints(self.state)
                validate(self.state, pinned, own)
                self.pinned = pinned
            validate(self.state, self.pinned, own, self.past_own)
            self.past_own.update(own)
            return set(self.pinned)

    def snapshot(self):
        with self.lock:
            return dict(pinned_native=sorted(self.pinned or ()), observer_alive=self.process.poll() is None,
                        age_s=time.monotonic() - self.at if self.at else None, discovery=self.state)

    def close(self):
        if self.closed:
            return
        self.closed = True
        if self.process.poll() is None:
            self.process.terminate()
            try:
                self.process.wait(timeout=3.)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait()
        self.thread.join(timeout=2.)
        self.process.stdout.close()
