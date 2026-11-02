#!/usr/bin/python3
"""C Qt/RViz bridge. Diagnostic publishers only; no control or TF output."""
import argparse
from collections import deque
import copy
import fcntl
import hashlib
import json
import os
from pathlib import Path
import queue
import signal
import subprocess
import threading
import time
from types import SimpleNamespace
import uuid

import cv2
import numpy as np

from camera_c import CameraCSession, PROTOCOL, BOUNDARY_GAP, ROOT, PAIR_LIMITS, transform
from camera_b_node import VisionReadOnlyClient, write_json, digest, validate_bundle
from vision_rgbd import validate_capture_result, import_camera_record
from camera_c import fresh_stream_evidence


class TimedVisionClient(VisionReadOnlyClient):
    """Bracket the inherited RPC call, independently of download and PnP."""
    on_rpc_end = None

    def environment(self):
        env = super().environment()
        env['CAMERA_C_FRESH_STREAM'] = '1'
        return env

    def capture(self, directory, progress=None):
        if progress:
            progress('保持停稳：原相机服务重开同一图像流、预热并取图；不操作机械臂')
        capture = self.rpc(directory)
        validate_capture_result(capture, 'prepare')
        import_camera_record(directory/'capture.json', host=self.host)
        validate_bundle(directory)
        record = json.loads((directory/'server_result.json').read_text())
        evidence = fresh_stream_evidence(capture, record)
        return dict(frame_timing=evidence, exposure_sync_verified=False,
                    association_method='owner_stream_restart_static_v1')

    def rpc(self, directory, status_only=False):
        if status_only:
            return super().rpc(directory, status_only=True)
        bounds = dict(rpc_start_monotonic=time.monotonic(), rpc_start_realtime=time.time(),
                      clock_basis='local_monotonic_receipt_v2',
                      semantics='local Python RPC call envelope, NOT exposure bounds')
        try:
            return super().rpc(directory, status_only=False)
        finally:
            bounds.update(rpc_end_monotonic=time.monotonic(), rpc_end_realtime=time.time())
            if self.on_rpc_end:
                self.on_rpc_end(bounds)


def save_receipt_window(directory, rows, models, bounds, context):
    """Keep missing/invalid evidence too; this does not certify a pairing."""
    directory.mkdir(parents=True, exist_ok=False)
    write_json(directory/'actual.json', rows)
    write_json(directory/'bounds.json', bounds)
    write_json(directory/'context.json', context)
    for sha, xml in models.items():
        if hashlib.sha256(xml.encode()).hexdigest() != sha:
            raise ValueError('Received model text SHA mismatch')
        with (directory/('model_'+sha+'.urdf')).open('x', encoding='utf-8') as stream:
            stream.write(xml)
    write_json(directory/'manifest.json', dict(schema='camera_c_receipt_window_v1',
        exposure_bounds_verified=False, pairing_valid=False,
        sha256={p.name: digest(p) for p in directory.iterdir() if p.is_file()}))
    return directory


def reject_nonfinite_json(value):
    raise ValueError('Nonfinite original owner JSON value: '+value)


def message_publisher_gid(info):
    """Older Humble executors pass only the message, without MessageInfo."""
    value = info.get('publisher_gid') if isinstance(info, dict) else getattr(info, 'publisher_gid', None)
    if value is None:
        return None
    try:
        raw = bytes(value)
    except (TypeError, ValueError, OverflowError):
        return None
    return raw.hex() if raw and any(raw) else None


def build_markers(session, selected):
    """Coordinates only, in torso_link; never broadcast a second camera TF."""
    from visualization_msgs.msg import Marker, MarkerArray
    from geometry_msgs.msg import Point
    from std_msgs.msg import ColorRGBA
    out = MarkerArray(markers=[Marker(action=Marker.DELETEALL)])

    def marker(kind, color, width=.003):
        m = Marker(); m.header.frame_id = 'torso_link'; m.ns = 'camera_c'
        m.id = len(out.markers); m.type = kind; m.action = Marker.ADD
        m.pose.orientation.w = 1.; m.scale.x = width
        m.color = ColorRGBA(r=float(color[0]), g=float(color[1]), b=float(color[2]), a=1.)
        out.markers.append(m)
        return m

    def point(xyz):
        return Point(x=float(xyz[0]), y=float(xyz[1]), z=float(xyz[2]))

    def axes(t, length=.055):
        m = marker(Marker.LINE_LIST, (1., 1., 1.))
        for i, color in enumerate(((1., .2, .2), (.2, 1., .2), (.3, .5, 1.))):
            m.points += [point(t[:3, 3]), point(t[:3, 3]+length*t[:3, i])]
            m.colors += [ColorRGBA(r=color[0], g=color[1], b=color[2], a=1.)]*2

    def label(xyz, text):
        m = marker(Marker.TEXT_VIEW_FACING, (.95, .95, .95)); m.scale.z = .018
        m.pose.position = point(xyz); m.text = text

    try:
        sample = next((s for s in session.state['samples'] if s['id'] == selected), None)
        if not sample or not sample['result']['rgb_valid']:
            return out
        session.verify(sample)
        p = transform(sample['result']['pose']['T_C_Q_row_major'])
        x_old = session.reference_camera()
        layout = sample['result']['detection']['layout']
        w = layout['cols']*session.state['board']['square_m']
        h = layout['rows']*session.state['board']['square_m']
        outline = np.array([[0., 0., 0., 1.], [w, 0., 0., 1.], [w, h, 0., 1.],
                            [0., h, 0., 1.], [0., 0., 0., 1.]])

        def board(t, color, text):
            m = marker(Marker.LINE_STRIP, color)
            m.points = [point(v[:3]) for v in outline @ t.T]
            axes(t, .04)
            label(t[:3, 3]+np.array([0., 0., .035]), text)

        axes(np.eye(4)); label(np.array([0., 0., -.035]), 'torso_link')
        axes(x_old); label(x_old[:3, 3]+np.array([0., 0., .035]), 'nominal RGB camera')
        board(x_old @ p, (1., .7, .15), 'NOMINAL RGB PROJECTION - NOT A PAIRED POSE')
        candidate = session.state.get('candidate')
        if candidate and sample['result'].get('pairing_valid') and candidate.get('applied') is False:
            x = transform(candidate['X_T_torso_RGB_row_major'])
            y = transform(candidate['Y_T_wrist_board_row_major'])
            g = transform(sample['result']['pairing']['G_T_W_row_major'])
            board(x @ p, (.15, .8, 1.), 'candidate RGB board - UNAPPLIED')
            board(g @ y, (.3, 1., .45), 'paired actual wrist + fixed board')
            m = marker(Marker.LINE_LIST, (1., .3, .7))
            m.points = [point((x @ p)[:3, 3]), point((g @ y)[:3, 3])]
    except (ValueError, KeyError, TypeError, OSError) as error:
        out = MarkerArray(markers=[Marker(action=Marker.DELETEALL)])
        label(np.array([0., 0., 0.]), 'C preview unavailable: '+str(error))
    return out


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument('--offline', action='store_true')
    mode.add_argument('--live', action='store_true')
    parser.add_argument('--session', type=Path)
    parser.add_argument('--network', default='eth0')
    parser.add_argument('--ssh', default='unitree@192.168.0.249')
    parser.add_argument('--peer', default='192.168.123.164')
    args = parser.parse_args()
    if args.offline and (os.environ.get('ROS_DOMAIN_ID') != '96' or os.environ.get('ROS_LOCALHOST_ONLY') != '1'):
        parser.error('Offline requires localhost/domain96')
    if args.live and (os.environ.get('ROS_DOMAIN_ID', '0') != '0' or os.environ.get('ROS_LOCALHOST_ONLY') == '1'):
        parser.error('Live requires original robot domain0 environment')
    lock = open(f'/tmp/g1-camera-c-{os.getuid()}.lock', 'a')
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        parser.error('C backend already running; close its own window first')
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
    from std_msgs.msg import String
    from visualization_msgs.msg import MarkerArray
    cv2.setNumThreads(1)
    rclpy.init()

    class Bridge(Node):
        def __init__(self):
            super().__init__('g1_camera_c_diagnostics')
            self.session = CameraCSession.load(args.session) if args.session else CameraCSession()
            self.session.recompute()
            self.selected = self.session.state['samples'][0]['id'] if self.session.state['samples'] else ''
            self.camera = TimedVisionClient(args.network, args.ssh, args.peer) if args.live else None
            self.camera_status = dict(ready=False, message='Not checked' if args.live else 'Offline')
            self.guard = threading.Lock(); self.busy = False; self.seen = deque(maxlen=1024)
            self.jobs = queue.Queue(maxsize=1); self.stop = threading.Event()
            self.message = '离线回放；按归档新帧与静止证据复算' if args.offline else '在线模式；启动不采图、不控制机器人'
            self.ack = {}; self.owner = {}; self.owner_at = 0.; self.owner_wall_at = None; self.model_xml = ''; self.model_at = 0.
            self.actual = deque(maxlen=20000); self.last_record = 0.; self.last_tick = None
            self.actual_error = ''; self.source_count = 0
            self.data_guard = threading.Lock()
            self.models = {}; self.model_sha = None; self.owner_gid = None
            self.source_gids = []; self.owner_gids = []; self.discovery_at = 0.
            self.discovery_history = deque(maxlen=100)
            self.stream_epoch = 0; self.last_gid = None
            self.context_epoch = 0
            self.cache = self.snapshot()
            qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.pub = self.create_publisher(String, '/g1/camera_c/status', qos)
            self.marker_pub = self.create_publisher(MarkerArray, '/g1/camera_c/markers', qos)
            self.create_subscription(String, '/g1/camera_c/request', self.request, 10)
            self.reader_process = None; self.reader_thread = None; self.reader_log = None
            if args.live:
                binary = ROOT/'install/g1_tcp_diagnostics/lib/g1_tcp_diagnostics/camera_c_actual_reader'
                if not binary.is_file():
                    raise RuntimeError('C actual reader missing; restart C with --build')
                logdir = ROOT/'logs/camera_c_readers'; logdir.mkdir(exist_ok=True)
                self.reader_log = (logdir/(uuid.uuid4().hex+'.log')).open('w')
                self.reader_process = subprocess.Popen([str(binary)], stdout=subprocess.PIPE,
                    stderr=self.reader_log, text=True, bufsize=1, start_new_session=True)
                self.reader_thread = threading.Thread(target=self.read_actual_pipe, daemon=False)
                self.reader_thread.start()
            self.create_timer(.3, self.publish)
            self.worker = threading.Thread(target=self.work, daemon=False); self.worker.start()

        def read_actual_pipe(self):
            try:
                for line in self.reader_process.stdout:
                    if self.stop.is_set():
                        break
                    record = json.loads(line, parse_constant=reject_nonfinite_json)
                    if record.get('clock_basis') != 'local_monotonic_receipt_v2':
                        raise ValueError('Unexpected C receiver clock contract')
                    if record['kind'] == 'actual':
                        motors = [SimpleNamespace(**{k:float('nan') if v is None else v for k,v in m.items()})
                                  for m in record['motor_state']]
                        self.lowstate(SimpleNamespace(tick=record['tick'], motor_state=motors,
                            mode_machine=record['mode_machine'], mode_pr=record['mode_pr']), record)
                    elif record['kind'] == 'owner':
                        self.owner_status(SimpleNamespace(data=record['data']), record)
                    elif record['kind'] == 'model':
                        self.description(SimpleNamespace(data=record['data']))
            except Exception as error:
                with self.data_guard:
                    self.actual_error = 'C receiver failed: '+str(error); self.stream_epoch += 1
            finally:
                if not self.stop.is_set():
                    with self.data_guard:
                        self.actual_error = self.actual_error or 'C receiver stopped; no automatic restart'
                        self.stream_epoch += 1

        def close_reader(self):
            if self.reader_process:
                if self.reader_process.poll() is None:
                    self.reader_process.send_signal(signal.SIGINT)
                    try:
                        self.reader_process.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        self.reader_process.terminate(); self.reader_process.wait(timeout=5)
                if self.reader_thread:
                    self.reader_thread.join(timeout=5)
                self.reader_process.stdout.close()
                self.reader_log.close()

        def snapshot(self):
            self.marker_cache = build_markers(self.session, self.selected)
            return dict(copy.deepcopy(self.session.state), root=str(self.session.root), selected=self.selected,
                        last_snapshot=self.session.last_snapshot)

        def owner_status(self, msg, info=None):
            gid = message_publisher_gid(info)
            received = info.get('received_monotonic', time.monotonic()) if isinstance(info, dict) else time.monotonic()
            try:
                data = json.loads(msg.data, parse_constant=reject_nonfinite_json)
                if not isinstance(data, dict):
                    raise ValueError('Original owner status is not an object')
                with self.data_guard:
                    keys = ('owner_token', 'console_pid', 'generation', 'stop_epoch', 'hardware', 'gateway_fresh')
                    old = self.owner.get('tcp_a_controls', {})
                    new = data.get('tcp_a_controls', {})
                    if not isinstance(new, dict):
                        raise ValueError('Malformed original owner controls')
                    if (any(old.get(k) != new.get(k) for k in keys) or
                            self.owner.get('gateway') != data.get('gateway') or
                            self.owner.get('shutdown_requested') != data.get('shutdown_requested') or
                            self.owner_gid != gid):
                        self.context_epoch += 1
                    self.owner = data; self.owner_at = received
                    self.owner_wall_at = info.get('received_realtime') if isinstance(info, dict) else time.time()
                    self.owner_gid = gid
            except (ValueError, TypeError) as error:
                with self.data_guard:
                    self.owner = dict(parse_error=str(error), raw_message=msg.data)
                    self.owner_at = 0.; self.owner_gid = None; self.context_epoch += 1

        def description(self, msg):
            sha = hashlib.sha256(msg.data.encode()).hexdigest()
            with self.data_guard:
                if sha != self.model_sha:
                    self.context_epoch += 1
                self.model_xml = msg.data; self.model_at = time.monotonic(); self.model_sha = sha
                self.models[sha] = msg.data

        def lowstate(self, msg, info=None):
            info = info if isinstance(info, dict) else {}
            at = info.get('received_monotonic', time.monotonic())
            received_wall = info.get('received_realtime', time.time())
            tick = int(msg.tick)
            gid = message_publisher_gid(info)
            from tcp_a_node import MOTORS
            q = {n: float(msg.motor_state[i].q) for n, i in MOTORS.items()}
            dq = {n: float(msg.motor_state[i].dq) for n, i in MOTORS.items()}
            valid = all(np.isfinite(v) for d in (q, dq) for v in d.values())
            voltages = [float(msg.motor_state[i].vol) for i in MOTORS.values()]
            valid = valid and all(np.isfinite(v) and v > 1. for v in voltages)
            with self.data_guard:
                changed = self.last_tick is not None and gid != self.last_gid
                rollback = self.last_tick is not None and tick < self.last_tick
                if changed or rollback or not valid:
                    self.stream_epoch += 1
                    self.actual_error = 'source changed / tick rollback / invalid motor feedback'
                if valid and not changed and tick == self.last_tick:
                    return
                self.last_tick = tick; self.last_gid = gid
                if valid and not changed and not rollback and at-self.last_record < .009:
                    return
                controls = self.owner.get('tcp_a_controls')
                controls = controls if isinstance(controls, dict) else {}
                discovery = next((v for v in reversed(self.discovery_history) if v[0] <= at), (None, [], []))
                row = dict(received=at, realtime=received_wall, tick=tick,
                    q={k:v if np.isfinite(v) else None for k,v in q.items()},
                    dq_map={k:v if np.isfinite(v) else None for k,v in dq.items()},
                    actual_valid=valid, publisher_gid=gid, source='lowstate_actual',
                    message_info_available=gid is not None,
                    raw_rmw_publisher_gid=info.get('raw_rmw_publisher_gid'),
                    publisher_identity_contract=info.get('publisher_identity_contract'),
                    clock_basis=info.get('clock_basis'), reader_pid=info.get('reader_pid'),
                    source_timestamp=info.get('source_timestamp'),
                    pipe_delay_s=time.monotonic()-at,
                    stream_epoch=self.stream_epoch, context_epoch=self.context_epoch,
                    mode_machine=int(msg.mode_machine), mode_pr=int(msg.mode_pr),
                    model_sha256=self.model_sha, owner_received=self.owner_at or None,
                    owner_received_realtime=self.owner_wall_at,
                    owner_age_s=at-self.owner_at if self.owner_at else None, owner_publisher_gid=self.owner_gid,
                    gateway_phase=self.owner.get('gateway'), owner_status=copy.deepcopy(self.owner),
                    publisher_count=len(discovery[1]), owner_publisher_count=len(discovery[2]),
                    discovered_actual_gids=list(discovery[1]), discovered_owner_gids=list(discovery[2]),
                    discovery_age_s=at-discovery[0] if discovery[0] is not None else None)
                row.update({k:controls.get(k) for k in ('owner_token', 'console_pid', 'generation', 'stop_epoch', 'gateway_fresh')})
                self.actual.append(row); self.last_record = at

        def freeze_receipts(self, directory, bounds):
            self.progress('RPC 已返回；保留 actual 后余量，随后下载并检查新流取帧证据。')
            hi = bounds['rpc_end_monotonic']+PAIR_LIMITS['margin_s']
            self.stop.wait(max(0., hi+PAIR_LIMITS['max_gap_s']-time.monotonic()))
            with self.data_guard:
                raw = list(self.actual)
                lo = bounds['rpc_start_monotonic']-PAIR_LIMITS['margin_s']
                before = [r for r in raw if r['received'] <= lo]
                inside = [r for r in raw if lo < r['received'] < hi]
                after = [r for r in raw if r['received'] >= hi]
                rows = copy.deepcopy(before[-1:]+inside+after[:1])
                hashes = {r.get('model_sha256') for r in rows}
                models = {sha:self.models[sha] for sha in hashes if sha in self.models}
                context = dict(frozen_monotonic=time.monotonic(), stream_error=self.actual_error,
                    interrupted=self.stop.is_set(), current_model_sha=self.model_sha,
                    actual_receipt_semantics='local monotonic DDS receipt, no certified transport age',
                    source_semantics='GIDs and periodically observed discovery; not a continuous uniqueness proof',
                    missing_pre_margin=not before, missing_post_margin=not after)
            return save_receipt_window(directory/'receipt_window', rows, models, bounds, context)

        def publish(self):
            now = time.monotonic()
            if args.live:
                sources = self.get_publishers_info_by_topic('/lowstate')
                owners = self.get_publishers_info_by_topic('/g1/task/console_status')
                with self.data_guard:
                    source_gids = sorted(bytes(v.endpoint_gid).hex() for v in sources)
                    owner_gids = sorted(bytes(v.endpoint_gid).hex() for v in owners)
                    if source_gids != self.source_gids or owner_gids != self.owner_gids:
                        self.context_epoch += 1
                    self.source_gids = source_gids; self.owner_gids = owner_gids
                    self.discovery_at = now; self.source_count = len(sources)
                    self.discovery_history.append((now, source_gids, owner_gids))
            with self.guard:
                status = dict(self.cache, stamp=time.time(), mode='live' if args.live else 'offline', busy=self.busy,
                              message=self.message, ack=self.ack, camera=self.camera_status,
                              actual=dict(count=len(self.actual), age_s=now-self.last_record if self.actual else None,
                                          publishers=self.source_count, error=self.actual_error, formal_archive_ready=False,
                                          message_info_available=self.last_gid is not None,
                                          provenance_note='' if self.last_gid is not None else
                                          '逐消息 GID 不可用；发现列表不作为消息来源证明，正式配对仍拒绝'),
                              owner=dict(available=bool(self.owner) and now-self.owner_at < 1.5,
                                         gateway=self.owner.get('gateway'), controls=self.owner.get('tcp_a_controls')),
                              model=dict(received=bool(self.model_xml)), formal_capture_enabled=bool(self.camera),
                              diagnostic_capture_enabled=bool(self.camera),
                              pairing_gap=BOUNDARY_GAP)
            self.pub.publish(String(data=json.dumps(status, ensure_ascii=False, allow_nan=False)))
            self.marker_pub.publish(self.marker_cache)

        def request(self, msg):
            req = {}
            try:
                if len(msg.data) > 16384:
                    raise ValueError('Request too large')
                req = json.loads(msg.data)
                if not isinstance(req, dict) or req.get('protocol') != PROTOCOL:
                    raise ValueError('C protocol mismatch')
                if req.get('op') not in {'new', 'load', 'import', 'select', 'save', 'recompute', 'solve', 'export',
                                          'exclude', 'group', 'check_camera', 'snapshot', 'capture'}:
                    raise ValueError('Unknown C operation; no robot controls exposed')
                rid = req.get('request_id')
                if not isinstance(rid, str) or not 8 <= len(rid) <= 80:
                    raise ValueError('Invalid request ID')
                stamp = float(req.get('stamp', 0))
                if not np.isfinite(stamp) or abs(time.time()-stamp) > 5:
                    raise ValueError('Stale request')
                if req['op'] in ('snapshot', 'capture', 'check_camera') and not self.camera:
                    raise ValueError('Offline mode forbids hardware acquisition')
                if req['op'] == 'capture':
                    if (req.get('stationary_confirmed') is not True or req.get('rigid_mount_confirmed') is not True or
                            req.get('group') not in ('fit', 'validation')):
                        raise ValueError('Sampling requires stationary and rigid-mount confirmation plus fit/validation group')
                with self.guard:
                    if self.busy or rid in self.seen:
                        raise ValueError('Busy or duplicate request; no automatic retry')
                    if req.get('session_id') != self.cache['session_id']:
                        raise ValueError('Session changed')
                    self.seen.append(rid); self.busy = True
                    self.ack = dict(request_id=rid, state='accepted')
                    self.jobs.put_nowait(req)
            except (ValueError, TypeError, queue.Full) as error:
                with self.guard:
                    self.message = str(error)
                    self.ack = dict(request_id=req.get('request_id', '') if isinstance(req, dict) else '',
                                    state='rejected', message=str(error))

        def progress(self, text):
            with self.guard:
                self.message = text

        def work(self):
            while not self.stop.is_set():
                try:
                    req = self.jobs.get(timeout=.2)
                except queue.Empty:
                    continue
                try:
                    op = req['op']; message = '完成'
                    if op == 'new':
                        self.session.save()
                        self.session = CameraCSession(side=req['side'], mount_id=req['mount_id'])
                        self.selected = ''
                    elif op == 'load':
                        loaded = CameraCSession.load(req['path']); loaded.recompute(); self.session.save()
                        self.session = loaded; self.selected = loaded.state['samples'][0]['id'] if loaded.state['samples'] else ''
                    elif op == 'import':
                        self.selected = self.session.import_bundle(req['path'], req.get('group', 'preview'), req.get('note', ''))['id']
                    elif op == 'select':
                        if not any(s['id'] == req['sample_id'] for s in self.session.state['samples']):
                            raise ValueError('Unknown sample')
                        self.selected = req['sample_id']
                    elif op in ('exclude', 'group'):
                        sample = next(s for s in self.session.state['samples'] if s['id'] == req['sample_id'])
                        if op == 'exclude':
                            reason = str(req.get('reason', '')).strip()
                            if not reason:
                                raise ValueError('Explicit audit reason required')
                            sample['excluded'] = not sample['excluded']; sample['exclusion_reason'] = reason
                        else:
                            if req.get('group') not in ('fit', 'validation', 'preview'):
                                raise ValueError('Unknown group')
                            if self.session.state['candidate']:
                                raise ValueError('Cannot regroup after candidate evaluation; use new independent validation')
                            sample['group'] = req['group']
                        self.session.state['candidate'] = None
                        self.session.event(op, sample_id=sample['id'], request=req)
                    elif op == 'recompute':
                        self.session.recompute()
                    elif op == 'solve':
                        self.session.solve()
                    elif op == 'export':
                        message = '导出：'+str(self.session.export())
                    elif op == 'check_camera':
                        self.camera_status = self.camera.check()
                    elif op in ('snapshot', 'capture'):
                        directory = ROOT/'data/camera_c/captures'/uuid.uuid4().hex
                        def finished(bounds):
                            bounds['stationary_confirmed'] = req.get('stationary_confirmed') is True
                            self.freeze_receipts(directory, bounds)
                        self.camera.on_rpc_end = finished
                        try:
                            acquisition = self.camera.capture(directory, self.progress)
                        finally:
                            self.camera.on_rpc_end = None
                        acquisition['receipt_window'] = str(directory/'receipt_window')
                        sample = self.session.import_bundle(directory, req['group'] if op == 'capture' else 'preview', req.get('note', ''), acquisition)
                        sample['rigid_mount_confirmed'] = req.get('rigid_mount_confirmed') is True
                        self.session.attach_receipt_window(sample['id'], directory/'receipt_window')
                        self.selected = sample['id']
                        if sample['result']['solve_usable']:
                            message = '采样通过：新流 RGB 与全窗口静止 actual 已配对，可用于求解'
                        else:
                            message = '样本已归档；未通过：'+ '; '.join(filter(None, [sample['result']['rgb_reason'], *sample['result']['pairing']['reasons']]))
                    self.session.save()
                    ack = dict(request_id=req['request_id'], state='completed', message=message)
                except Exception as error:
                    message = str(error); ack = dict(request_id=req['request_id'], state='failed', message=message)
                    evidence = str(directory) if req['op'] in ('snapshot', 'capture') else None
                    self.session.event('request_failed', request=req, reason=message, capture_directory=evidence)
                    self.session.save()
                with self.guard:
                    self.cache = self.snapshot(); self.message = message; self.ack = ack; self.busy = False

    node = Bridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop.set(); node.close_reader(); node.worker.join(); node.session.save(); node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
