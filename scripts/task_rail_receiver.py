"""Read-only, separate-process LowState recorder. No robot command publishers."""
import argparse
from collections import deque
import ctypes
import json
import math
import os
from pathlib import Path
import select
import signal
import sys
import time


def save(path, value):
    temporary = path.with_suffix('.tmp')
    temporary.write_text(json.dumps(value, allow_nan=False))
    os.replace(temporary, path)


class Recorder:
    def __init__(self, directory):
        self.directory = directory
        self.spec = json.loads((directory/'request.json').read_text())
        self.full_rate = self.spec.get('sample_policy') == 'all_received'
        self.stream = (directory/'samples.jsonl').open('x', buffering=65536)
        self.count = 0
        self.first = self.last = self.selected = None
        self.source = self.tick = self.publisher = None
        self.error = None
        self.done = False
        self.finish_at = None
        self.history = deque()
        self.max_gap = self.max_source_gap = 0.
        self.latest = None
        self.started = time.monotonic()
        self.last_status = 0.

    def sample(self, msg, info):
        if self.error or self.done:
            return
        try:
            at = time.monotonic()
            source = int(info['source_timestamp'])
            publisher = bytes(info['publisher_gid']).hex() if 'publisher_gid' in info else None
            if source <= 0:
                raise ValueError('LowState 缺少 DDS 源时间戳')
            if self.publisher is not None and publisher != self.publisher:
                raise ValueError('LowState 发布源变化或存在多个发布源')
            if int(msg.mode_machine) != 6 or int(msg.mode_pr) != 0:
                raise ValueError('LowState 不再处于 mode6/pr0')
            if self.tick is not None:
                advance = (int(msg.tick)-self.tick) % (2**32)
                if advance == 0:
                    return
                if advance > 2**31:
                    raise ValueError('LowState tick 倒退')
                gap = at-self.last
                source_gap = (source-self.source)*1e-9
                self.max_gap = max(self.max_gap, gap)
                self.max_source_gap = max(self.max_source_gap, source_gap)
                if source_gap <= 0 or max(gap, source_gap) > .1:
                    raise ValueError(f'直采反馈间隔异常：接收 {gap*1000:.1f} ms，DDS 源 {source_gap*1000:.1f} ms')
            q = {}; velocities = {}; torques = {}; states = {}
            for name, index in self.spec['motors'].items():
                motor = msg.motor_state[index]
                value = float(motor.q)
                lo,hi = self.spec['limits'][name]
                if not all(math.isfinite(v) for v in (value,motor.dq,motor.vol)) or motor.vol <= 1.:
                    raise ValueError('LowState 无效电机反馈：'+name)
                if not lo <= value <= hi:
                    raise ValueError('LowState 关节角度超限：'+name)
                q[name] = value
                if self.full_rate:
                    torque=float(motor.tau_est)
                    if not math.isfinite(torque):raise ValueError('LowState 非有限力矩：'+name)
                    velocities[name]=float(motor.dq);torques[name]=torque
                    states[name]=dict(mode=int(motor.mode),temperature=[int(v) for v in motor.temperature],
                        vol=float(motor.vol),sensor=[int(v) for v in motor.sensor],motorstate=int(motor.motorstate))
            if any(abs(q[k]-self.spec['initial'][k]) > .025 for k in self.spec.get('passive',self.spec['left'])):
                raise ValueError('直采检测到非录制手臂移动，停止本段记录')
            if self.latest and any(abs(q[k]-self.latest['joints'][k]) > .04 for k in self.spec['moving']):
                raise ValueError('直采检测到相邻关节反馈跳变超过 0.04 rad')
            if self.first is None:
                if any(abs(q[k]-self.spec['initial'][k]) > math.radians(2.) for k in self.spec['moving']):
                    raise ValueError('直采起点与确认姿态相差超过 2 度，请停稳后重录')
                self.first = at
            if at-self.first > 300.:
                raise ValueError('单段录制超过 5 分钟')
            self.last,self.source,self.tick,self.publisher = at,source,int(msg.tick),publisher
            self.latest = dict(received_monotonic=at,source_timestamp_ns=source,
                rmw_received_timestamp_ns=int(info['received_timestamp']),tick=self.tick,joints=q)
            if self.full_rate:
                self.latest.update(velocities=velocities,torques_nm=torques,motor_status=states,
                    mode_machine=int(msg.mode_machine),mode_pr=int(msg.mode_pr))
            self.history.append((at,q))
            while len(self.history) > 1 and self.history[1][0] < at-.8:
                self.history.popleft()
            # Full-rate archives retain every distinct tick; old callers may downsample.
            if self.full_rate or self.selected is None or at-self.selected >= .01:
                self.append()
        except Exception as error:
            self.error = str(error)

    def append(self):
        limit=600000 if self.full_rate else 30000
        if self.count >= limit:
            raise ValueError(f'单段原始样本超过 {limit} 点')
        self.stream.write(json.dumps(self.latest,allow_nan=False)+'\n')
        self.count += 1
        self.selected = self.latest['received_monotonic']

    def finish(self,cutoff=None):
        cutoff=time.monotonic() if cutoff is None else cutoff
        if not math.isfinite(cutoff) or cutoff>time.monotonic()+.001:raise ValueError('无效录制截止时刻')
        history=[(at,q) for at,q in self.history if at<=cutoff]
        if not history or cutoff-history[-1][0] > .1:
            raise ValueError('结束时直采反馈不新鲜')
        end=history[-1][0]
        if end-history[0][0] < .4-1e-9:
            raise ValueError('结束时静止窗口不足 0.4 秒')
        first=max(i for i,(at,_) in enumerate(history) if at<=end-.4+1e-9)
        history=history[first:]
        if any(max(q[k] for _,q in history)-min(q[k] for _,q in history) > .012
               for k in self.spec['motors']):
            raise ValueError('结束时右臂、左臂或腰仍在运动')
        if self.latest['received_monotonic'] > self.selected:
            self.append()
        self.done = True
        self.finish_at = cutoff

    def status(self, force=False):
        at = time.monotonic()
        if not force and at-self.last_status < .1:
            return
        self.stream.flush()
        save(self.directory/'status.json', dict(schema='g1_rail_direct_status_v1',
            heartbeat=at, ready=self.first is not None, done=self.done,error=self.error,
            count=self.count,seconds=(min(self.last,self.finish_at if self.finish_at is not None else self.last)-self.first
                                     if self.first is not None else 0.),
            requested_finish_monotonic=self.finish_at,
            max_receive_gap_ms=self.max_gap*1000,max_source_gap_ms=self.max_source_gap*1000,
            sample_policy='all_received' if self.full_rate else 'measured_100hz',
            stored_hz=(self.count-1)/(self.selected-self.first)
                if self.count>1 and self.selected>self.first else 0.,
            publisher_gid=self.publisher,last_sample=self.latest))
        self.last_status = at


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('directory',type=Path)
    args = parser.parse_args()
    parent = os.getppid()
    if ctypes.CDLL(None).prctl(1,signal.SIGTERM) != 0 or os.getppid() != parent:
        raise RuntimeError('Cannot bind recorder lifetime to panel')
    recorder = Recorder(args.directory)
    def interrupted(*_):
        recorder.error = '录制进程被中断；原始数据保留'
    signal.signal(signal.SIGTERM,interrupted)
    signal.signal(signal.SIGINT,interrupted)
    node = None
    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        from unitree_hg.msg import LowState
        rclpy.init()
        signal.signal(signal.SIGTERM,interrupted)
        signal.signal(signal.SIGINT,interrupted)
        node = Node('g1_rail_readonly_recorder',enable_rosout=False,
                    start_parameter_services=False)
        # A dedicated process and bounded DDS queue isolate capture from Qt/MoveIt.
        subscription = node.create_subscription(LowState,'/lowstate',lambda msg:None,
            QoSProfile(depth=256,reliability=ReliabilityPolicy.BEST_EFFORT))
        while rclpy.ok() and not recorder.error and not recorder.done:
            if select.select([sys.stdin],[],[],0)[0]:
                command=sys.stdin.readline().strip().split()
                if command and command[0]=='finish' and len(command)<=2:
                    recorder.finish(float(command[1]) if len(command)==2 else None)
                else:recorder.error='取消录制或面板连接关闭；原始数据保留'
                recorder.status(force=True)
                break
            # Humble's executor discards MessageInfo. Use its same take API,
            # retaining DDS source/receive stamps before any GUI processing.
            received = False
            for _ in range(32):
                with subscription.handle:
                    item = subscription.handle.take_message(LowState,False)
                if item is None:
                    break
                received = True
                recorder.sample(*item)
                if recorder.error:
                    break
            at = time.monotonic()
            if recorder.last is not None and at-recorder.last > .1:
                recorder.error = f'直采 LowState 接收超时 {(at-recorder.last)*1000:.1f} ms'
            elif recorder.last is None and at-recorder.started > 8.:
                recorder.error = '直采进程 8 秒内未收到有效 LowState'
            recorder.status()
            if not received:
                time.sleep(.001)
    except Exception as error:
        recorder.error = str(error)
    finally:
        recorder.status(force=True)
        recorder.stream.close()
        if node is not None:
            node.destroy_node()
        if 'rclpy' in locals() and rclpy.ok():
            rclpy.shutdown()
    return 1 if recorder.error else 0


if __name__ == '__main__':
    sys.exit(main())
