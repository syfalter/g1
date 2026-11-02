"""Local process lifecycle and archive reader for read-only rail capture."""
import json
from pathlib import Path
import subprocess
import sys
import time
import threading

from task_screen_teach import MOVING, JOINT_MARGIN, write_json
from task_session import ARMS


class DirectCapture:
    def __init__(self, node, stage, initial, generation, epoch, *, side='right', namespace='rail_recordings', full_rate=False):
        if side not in ARMS:raise ValueError('Unknown capture arm')
        self.moving=ARMS[side]+('waist_yaw_joint',)
        self.directory = node.run_dir/namespace/f'{time.time_ns()}_{stage}'
        motors = {name:index for side,offset in (('left',15),('right',22))
                  for index,name in enumerate(ARMS[side],offset)}
        motors['waist_yaw_joint'] = 12
        write_json(self.directory/'request.json',dict(stage=stage,initial=initial,
            sample_policy='all_received' if full_rate else 'measured_100hz',
            generation=generation,stop_epoch=epoch,source='/lowstate',moving=list(self.moving),
            left=list(ARMS['left']),passive=list(ARMS['right' if side=='left' else 'left']),motors=motors,
            limits={k:[node.model.limits[k][0]+JOINT_MARGIN,node.model.limits[k][1]-JOINT_MARGIN]
                    for k in motors}))
        self.latest = {}
        self.finish_at=None;self.finish_lock=threading.Lock()
        self.started = time.monotonic()
        self.log = (self.directory/'receiver.log').open('w')
        try:
            self.process = subprocess.Popen([sys.executable,
                str(Path(__file__).with_name('task_rail_receiver.py')),str(self.directory)],
                stdin=subprocess.PIPE,stdout=self.log,stderr=subprocess.STDOUT,text=True)
        except Exception:
            self.log.close()
            raise

    def check(self):
        path = self.directory/'status.json'
        if path.is_file():
            self.latest = json.loads(path.read_text())
        if self.latest.get('error'):
            raise RuntimeError(self.latest['error']+'；原始数据：'+str(self.directory))
        if self.process.poll() is not None and not self.latest.get('done'):
            raise RuntimeError('直采进程已退出；详见 '+str(self.directory/'receiver.log'))
        if not self.latest.get('done') and time.monotonic()-self.latest.get('heartbeat',self.started) > (2. if self.latest else 10.):
            raise RuntimeError('独立录制进程无响应；原始数据：'+str(self.directory))
        return self.latest

    def request_finish(self,at=None):
        with self.finish_lock:
            if self.finish_at is not None:return
            self.finish_at=time.monotonic() if at is None else at
            self.process.stdin.write(f'finish {self.finish_at:.9f}\n');self.process.stdin.flush()

    def finish(self, check):
        self.request_finish()
        deadline = time.monotonic()+5.
        while time.monotonic() < deadline:
            check()
            if self.check().get('done'):
                self.process.wait(timeout=2.)
                raw=dict(positions=[],time_s=[],stamp_ns=[],tick=[],
                    requested_finish_monotonic=self.finish_at)
                first=None
                # Stream the telemetry archive; keep only playback angles in the bundle.
                with (self.directory/'samples.jsonl').open() as stream:
                    for line in stream:
                        row=json.loads(line)
                        if row['received_monotonic']>self.finish_at:continue
                        if first is None:first=row['received_monotonic']
                        raw['positions'].append([row['joints'][k] for k in self.moving])
                        raw['time_s'].append(row['received_monotonic']-first)
                        raw['stamp_ns'].append(row['source_timestamp_ns'])
                        raw['tick'].append(row.get('tick'))
                if first is None:raise RuntimeError('截止时刻前没有录制样本')
                raw['received_monotonic_start']=first
                spec=json.loads((self.directory/'request.json').read_text())
                raw['sample_policy']=spec.get('sample_policy','measured_100hz')
                raw['telemetry_path']=str(self.directory/'samples.jsonl')
                return raw
            time.sleep(.02)
        raise RuntimeError('直采结束确认超时；原始数据保留于 '+str(self.directory))

    def close(self):
        try:
            if self.process.poll() is None:
                self.process.terminate()
                try:self.process.wait(timeout=2.)
                except subprocess.TimeoutExpired:
                    self.process.kill(); self.process.wait(timeout=2.)
        finally:
            self.process.stdin.close()
            self.log.close()
