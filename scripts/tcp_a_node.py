#!/usr/bin/python3
"""Read-only RViz experiment A backend. Never publishes TF or motor commands."""
import argparse
from collections import deque
import copy
import fcntl
import hashlib
import json
import os
from pathlib import Path
import queue
import threading
import time

import numpy as np
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor, ExternalShutdownException
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import String
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from unitree_hg.msg import LowState, LowCmd

from workbench_model import OfflineModel
from task_hand_control import HandControl, model_joints, POSTURES
from tcp_a_core import (CONFIRMATIONS, DEFAULT_LIMITS, checked_limits, append_sample, new_session, point, stability,
                        tracking, solve, invalidate, load_session, save_session,
                        export_session, synthetic_session, now_iso, write_new, average_observation)

ROOT = Path(__file__).resolve().parents[1]
PARTS = ('shoulder_pitch', 'shoulder_roll', 'shoulder_yaw', 'elbow', 'wrist_roll', 'wrist_pitch', 'wrist_yaw')
MOTORS = {f'{side}_{part}_joint': start+i for side, start in (('left', 15), ('right', 22))
          for i, part in enumerate(PARTS)}
MOTORS['waist_yaw_joint'] = 12
PREFIX = '/g1/tcp_a'


def required_joints(model, link):
    required = set()
    while link != 'pelvis':
        if link not in model.joints:
            raise ValueError('模型链没有连接到pelvis：'+link)
        parent, kind, _, _, _, name, _, _ = model.joints[link]
        if kind != 'fixed':
            required.add(name)
        link = parent
    return required


def measured_wrist(model, side, q):
    wrist = side+'_wrist_yaw_link'
    missing = required_joints(model, wrist)-q.keys()
    if missing:
        raise ValueError('缺少实际FK关节，禁止默认补零：'+','.join(sorted(missing)))
    if not all(np.isfinite(q[n]) for n in required_joints(model, wrist)):
        raise ValueError('实际关节值非有限')
    return np.linalg.inv(model.transform('pelvis', q)) @ model.transform(wrist, q)


def build_markers(session, selected='', candidate=True, live_g=None, reference_g=None):
    output = MarkerArray()
    clear = Marker(); clear.action = Marker.DELETEALL
    output.markers.append(clear)
    counter = 0

    def marker(kind, xyz, color, size=.006, text='', points=None):
        nonlocal counter
        item = Marker(); item.header.frame_id = 'pelvis'; item.ns = 'tcp_a'; item.id = counter
        counter += 1
        item.type = kind; item.action = Marker.ADD; item.pose.orientation.w = 1.
        item.pose.position.x, item.pose.position.y, item.pose.position.z = map(float, xyz)
        item.scale.x = item.scale.y = item.scale.z = size
        item.color.r, item.color.g, item.color.b, item.color.a = color
        item.text = text
        if points is not None:
            for p in points:
                value = Point(); value.x, value.y, value.z = map(float, p); item.points.append(value)
        output.markers.append(item)
        return item

    def line(a, b, color, width=.001):
        marker(Marker.LINE_LIST, [0, 0, 0], color, width, points=[a, b])

    def axes(g, length=.045, width=.0014):
        for j, color in enumerate(((1., .25, .2, 1.), (.25, 1., .3, 1.), (.25, .5, 1., 1.))):
            line(g[:3, 3], g[:3, 3]+g[:3, j]*length, color, width)

    axes(np.eye(4), .14)
    marker(Marker.TEXT_VIEW_FACING, [0, 0, .16], (.9, .9, .9, 1.), .017,
           'pelvis | OFFLINE - NOT HARDWARE' if session['mode']=='offline' else 'pelvis | READ ONLY')
    result = session.get('result')
    for row in session['samples']:
        g = np.asarray(row['G']); original = point(g, session['s_model'])
        old_color = (.43, .43, .43, .6) if row['excluded'] else (.24, .65, 1., 1.)
        kind = Marker.CUBE if row['group']=='validation' else Marker.SPHERE
        marker(kind, original, old_color)
        if result and candidate:
            p = result['per_sample'][row['id']]
            corrected = p['new_m']
            marker(kind, corrected, (.4, .4, .4, .5) if row['excluded'] else (.1, .9, .55, 1.), .004)
            if p['center_m'] is not None:
                line(corrected, p['center_m'], (.8, .4, .9, .5) if row['excluded'] else (1., .35, .3, .9))
        if row['id'] == selected:
            axes(g)
            line(g[:3, 3], original, (.95, .85, .15, .8))
            marker(Marker.SPHERE, original, (1., .9, .1, .35), .014)
            marker(Marker.TEXT_VIEW_FACING, original+[0, 0, .022], (1., 1., .7, 1.), .012,
                   row['id']+' | '+row['group'])
            if row.get('tracking'):
                t = row['tracking']
                marker(Marker.CUBE, t['reference_m'], (1., .65, .12, .9), .007)
                line(t['reference_m'], t['actual_m'], (1., .65, .12, 1.), .002)
    if result:
        marker(Marker.SPHERE, result['fit_old']['center_m'], (.65, .8, 1., .8), .004)
        if candidate:
            marker(Marker.SPHERE, result['fit_center_m'], (.85, .4, 1., 1.), .006)
            marker(Marker.TEXT_VIEW_FACING, np.array(result['fit_center_m'])+[0, 0, .04], (.9, .8, 1., 1.),
                   .012, 'Estimated contact (not ground truth)')
    if live_g is not None:
        axes(live_g, .05)
        tip = point(live_g, session['s_model'])
        marker(Marker.SPHERE, tip, (.3, .75, 1., .7), .008)
        line(live_g[:3, 3], tip, (.3, .75, 1., .7))
        if reference_g is not None:
            target = point(reference_g, session['s_model'])
            marker(Marker.CUBE, target, (1., .65, .12, 1.), .008)
            line(target, tip, (1., .65, .12, 1.), .002)
        if result and candidate:
            marker(Marker.SPHERE, point(live_g, result['s_candidate_m']), (.1, .9, .55, 1.), .006)
    return output


class TcpANode(Node):
    def __init__(self, args):
        super().__init__('g1_tcp_a_diagnostics')
        self.args = args
        import yaml
        self.limits = checked_limits(yaml.safe_load(args.config.read_text())) if args.config else dict(DEFAULT_LIMITS)
        self.data_lock = threading.Lock(); self.session_lock = threading.RLock()
        self.history = deque(maxlen=1600); self.commands = deque(maxlen=300)
        self.last_tick = None; self.last_record = 0.; self.diag = {}; self.diag_at = 0.
        self.last_command_record = 0.
        self.model = None; self.model_xml = ''; self.model_hash = ''; self.model_error = ''; self.pending_xml = ''
        self.session = new_session(mode='offline' if args.offline else 'live', limits=self.limits)
        self.selected = ''; self.candidate = True; self.message = '等待系统检查'; self.busy = False
        self.requests = queue.Queue(maxsize=8); self.seen_requests = deque(maxlen=256)
        self.stop_event = threading.Event(); self.publisher_names = []
        self.hand = None
        self.capture_progress = None
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.state_pub = self.create_publisher(String, PREFIX+'/status', qos)
        self.markers_pub = self.create_publisher(MarkerArray, PREFIX+'/markers', qos)
        group = ReentrantCallbackGroup()
        self.input_group = MutuallyExclusiveCallbackGroup()
        self.update_group = MutuallyExclusiveCallbackGroup()
        self.create_subscription(String, PREFIX+'/request', self.request, 10, callback_group=group)
        if not args.offline:
            sensor_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
            self.create_subscription(LowState, args.lowstate_topic, self.lowstate, sensor_qos, callback_group=self.input_group)
            self.create_subscription(LowCmd, '/arm_sdk', self.command, sensor_qos, callback_group=self.input_group)
            self.create_subscription(DiagnosticArray, '/g1/arm_gateway/diagnostics', self.diagnostics, 1, callback_group=self.input_group)
            self.create_subscription(String, '/robot_description', self.description, qos, callback_group=group)
        if args.offline:
            self.session = load_session(args.session) if args.session else new_session(mode='offline')
            self.message = '离线回放：不订阅真机、不读取手DDS、不采集真机状态'
        elif args.robot_urdf:
            self.accept_description(Path(args.robot_urdf).read_text())
        self.worker = threading.Thread(target=self.work, daemon=True); self.worker.start()
        self.create_timer(.35, self.update, callback_group=self.update_group)
        self.get_logger().info('Experiment A ready: '+('OFFLINE domain 96' if args.offline else 'LIVE READ-ONLY')+
                               '; publishers: diagnostic status and markers only')

    def directory(self):
        return ROOT/'data/tcp_a'/self.session['session_id']

    def accept_description(self, xml):
        sha = hashlib.sha256(xml.encode()).hexdigest()
        with self.session_lock:
            if sha == self.model_hash:
                return
            if self.session['samples'] and sha != self.session['model'].get('sha256'):
                self.pending_xml = xml
                self.model_error = '运行模型改变，旧会话禁止继续采样；请新建会话'
                return
            path = ROOT/'data/tcp_a/models'/('robot_'+sha+'.urdf')
            if not path.exists():
                write_new(path, xml)
            if hashlib.sha256(path.read_bytes()).hexdigest() != sha:
                raise ValueError('模型存档校验失败')
            model = OfflineModel(path)
            for side in ('left', 'right'):
                names = required_joints(model, side+'_wrist_yaw_link')
                if not names <= MOTORS.keys():
                    raise ValueError('腕部链含没有实际反馈映射的关节：'+str(names-MOTORS.keys()))
                if side+'_index_tcp' not in model.joints:
                    raise ValueError('运行模型没有 index_tcp；不能猜测指尖')
            self.model = model; self.model_xml = xml; self.model_hash = sha; self.model_error = ''; self.pending_xml = ''
            self.reset_session(self.session['side'])

    def description(self, msg):
        try:
            self.accept_description(msg.data)
        except Exception as error:
            self.model_error = str(error)

    def reset_session(self, side):
        if self.args.offline:
            self.session = new_session(side=side, mode='offline', limits=self.limits)
        else:
            if self.model is None:
                raise ValueError('尚未收到当前运行robot_description')
            q = {n:0. for n in MOTORS}
            q.update(model_joints(side, POSTURES['index_only']))
            needed = required_joints(self.model, side+'_index_tcp')
            if not needed <= q.keys():
                raise ValueError('指尖链缺少已说明的模型手型：'+str(needed-q.keys()))
            wrist = measured_wrist(self.model, side, q)
            s = (np.linalg.inv(wrist) @ self.model.transform(side+'_index_tcp', q))[:3, 3]
            self.session = new_session(side=side, model=dict(sha256=self.model_hash,
                source='/robot_description', file=str(ROOT/'data/tcp_a/models'/('robot_'+self.model_hash+'.urdf')),
                wrist_fk='actual LowState, no default joints',
                hand_fk='nominal index_only mapping + URDF mimic; NOT measured phalanx angles',
                waist_roll_pitch='fixed in installed URDF, NOT measured'), s_model=s, limits=self.limits)
        self.selected = ''; self.model_error = ''

    def lowstate(self, msg):
        at = time.monotonic()
        with self.data_lock:
            tick = int(msg.tick)
            if self.last_tick is not None:
                advance = (tick-self.last_tick) % (2**32)
                if advance == 0:
                    return
                if advance > 2**31:
                    self.history.clear()
            self.last_tick = tick
            if at-self.last_record < .009:
                return
            q = {n:float(msg.motor_state[i].q) for n, i in MOTORS.items()}
            dq = {n:float(msg.motor_state[i].dq) for n, i in MOTORS.items()}
            tau = {n:float(msg.motor_state[i].tau_est) for n, i in MOTORS.items()}
            if not all(np.isfinite(v) for data in (q, dq, tau) for v in data.values()):
                self.history.clear(); return
            if any(not np.isfinite(msg.motor_state[i].vol) or msg.motor_state[i].vol <= 1. for i in MOTORS.values()):
                self.history.clear(); return
            self.history.append(dict(received=at, ros_receipt_ns=self.get_clock().now().nanoseconds,
                tick=tick, q=q, dq_map=dq, tau_est=tau, mode_machine=int(msg.mode_machine),
                mode_pr=int(msg.mode_pr), source='lowstate_actual'))
            self.last_record = at

    def command(self, msg):
        at = time.monotonic()
        with self.data_lock:
            if at-self.last_command_record < .009:
                return
            self.last_command_record = at
            self.commands.append(dict(received=at, q={n:float(msg.motor_cmd[i].q) for n, i in MOTORS.items()},
                weight=float(msg.motor_cmd[29].q)))

    def diagnostics(self, msg):
        values = {v.key:v.value for status in msg.status for v in status.values}
        with self.data_lock:
            self.diag, self.diag_at = values, time.monotonic()

    def observations(self, session, raw_records=None):
        if self.args.offline:
            raise ValueError('离线模式不采集真机状态，请加载会话或合成示例')
        if self.model is None or self.model_error:
            raise ValueError(self.model_error or '等待当前运行robot_description')
        with self.data_lock:
            rows = list(self.history) if raw_records is None else list(raw_records)
            snapshot_at = time.monotonic()
        if not rows:
            return []
        if raw_records is None:
            rows = [r for r in rows if rows[-1]['received']-r['received'] <= session['limits']['window_s']+.04]
        names = sorted(required_joints(self.model, session['wrist_frame']))
        for r in rows:
            if not set(names) <= r['q'].keys():
                raise ValueError('缺少实际FK关节，禁止默认补零')
        positions = np.asarray([[r['q'][n] for n in names] for r in rows])
        frames = self.model.batch_transforms(['pelvis', session['wrist_frame']], {}, names, positions)
        base, wrist = frames['pelvis'], frames[session['wrist_frame']]
        inverse_rotation = base[:, :3, :3].transpose(0, 2, 1)
        gs = np.broadcast_to(np.eye(4), (len(rows), 4, 4)).copy()
        gs[:, :3, :3] = inverse_rotation @ wrist[:, :3, :3]
        gs[:, :3, 3] = np.einsum('nij,nj->ni', inverse_rotation, wrist[:, :3, 3]-base[:, :3, 3])
        return [dict(r, G=g.tolist(), q_stability=p.tolist(), snapshot_at=snapshot_at,
                     dq=[r['dq_map'][n] for n in names]) for r,g,p in zip(rows,gs,positions)]

    def check_window(self, rows, session, *, entire_window=False):
        # A frozen acquisition window does not become stale merely because FK
        # takes time. Independently verify that the live receiver is still fresh.
        snapshot_at = rows[-1]['snapshot_at'] if rows else time.monotonic()
        quality = stability(rows, snapshot_at, session['limits'], session['s_model'], entire_window=entire_window)
        with self.data_lock:
            latest = self.history[-1]['received'] if self.history else float('-inf')
        now = time.monotonic()
        if now-latest > session['limits']['max_age_s']:
            raise ValueError(f'LowState接收端断流/过期：距最新反馈 {(now-latest)*1000:.1f} ms；不是FK快照年龄')
        quality.update(processing_s=now-snapshot_at, stream_age_s=now-latest)
        return quality

    def reference(self, row, session):
        with self.data_lock:
            commands, diag, diag_at = list(self.commands), dict(self.diag), self.diag_at
        if not commands or time.monotonic()-diag_at > 1.5 or diag.get('armed') != 'true':
            return None, '无新鲜已接管控制器参考；跟踪差不记为零'
        if diag.get('phase') not in ('ARMED_HOLD', 'RUNNING'):
            return None, '当前不是普通轨迹/保持阶段：'+diag.get('phase', 'unknown')
        if diag.get('task_waist_trajectory_supported') != 'true':
            return None, '缺少腰部有效控制参考，不能计算完整pelvis跟踪差'
        ref = min(commands, key=lambda c:abs(c['received']-row['received']))
        skew = abs(ref['received']-row['received'])
        if skew > session['limits']['reference_skew_s'] or ref['weight'] < .99:
            return None, '命令与反馈不同步或处于接管混合阶段'
        g = measured_wrist(self.model, session['side'], ref['q'])
        result = tracking(row['G'], g, session['s_model'])
        names = required_joints(self.model, session['wrist_frame'])
        result.update(reference_G=g.tolist(), reference_q=ref['q'],
            joint_actual_minus_reference_rad={n:row['q'][n]-ref['q'][n] for n in sorted(names)},
            reference_source='/arm_sdk observed LowCmd; NOT a command from this tool', skew_s=skew,
            time_basis='local monotonic receipt pairing; not hardware clock synchronization',
            other_arm_max_error_rad=max(abs(row['q'][n]-ref['q'][n]) for n in MOTORS if
                                       not n.startswith(session['side']) and n!='waist_yaw_joint'))
        return result, ''

    def request(self, msg):
        try:
            data = json.loads(msg.data)
            if not isinstance(data, dict) or not isinstance(data.get('request_id'), str):
                raise ValueError('缺少请求ID')
            with self.data_lock:
                if data['request_id'] in self.seen_requests:
                    return
                self.seen_requests.append(data['request_id'])
            if abs(time.time()-float(data['sent_at'])) > 3.:
                raise ValueError('请求过期；不执行延迟采样')
            self.requests.put_nowait(data)
        except Exception as error:
            self.message = '请求拒绝：'+str(error)

    def capture(self, data):
        if self.args.offline:
            raise ValueError('离线模式禁止真实采样')
        if not all(data.get('confirmations', {}).get(k) is True for k in CONFIRMATIONS):
            raise ValueError('请逐项确认真实来源、胯部固定、手型固定、同点接触和轻触')
        if len(self.get_publishers_info_by_topic(self.args.lowstate_topic)) != 1:
            raise ValueError('LowState来源缺失或有多个发布者；先核对来源，禁止静默混用')
        with self.session_lock:
            session = copy.deepcopy(self.session)
        before = self.observations(session)
        self.check_window(before, session)
        hand_feedback = None
        if data.get('read_hand'):
            if self.hand is None:
                self.hand = HandControl(self.args.network, enabled=False, peer_address=self.args.hand_peer)
            first = self.hand.read(session['side'])
        started = time.monotonic()
        with self.data_lock:
            gateway_epoch = self.diag.get('task_stop_epoch')
            gateway_phase = self.diag.get('phase')
        while True:
            if self.stop_event.wait(.025):
                raise ValueError('诊断窗口关闭，取消本次平均采样')
            with self.data_lock:
                raw = [r for r in self.history if r['received'] >= started]
                if self.diag.get('task_stop_epoch') != gateway_epoch or self.diag.get('phase') != gateway_phase:
                    raise ValueError('采样期间控制状态改变，请重新确认接触')
            now = time.monotonic()
            if now-started > session['limits']['max_age_s'] and (not raw or now-raw[-1]['received']>session['limits']['max_age_s']):
                raise ValueError('一秒采样期间LowState断流/过期')
            elapsed = raw[-1]['received']-raw[0]['received'] if raw else 0.
            self.capture_progress = dict(elapsed_s=elapsed, duration_s=session['limits']['window_s'], frames=len(raw))
            self.message = f'正在采样 {min(elapsed, session["limits"]["window_s"]):.2f} / {session["limits"]["window_s"]:.2f} 秒；保持同一轻触，勿改变腕姿'
            if elapsed >= session['limits']['window_s']:
                break
            if now-started > session['limits']['window_s']+.5:
                raise ValueError('没有获得完整连续的一秒反馈；本次不保存')
        rows = self.observations(session, raw)
        quality = self.check_window(rows, session, entire_window=True)
        if data.get('read_hand'):
            second = self.hand.read(session['side'])
            if second['timestamp_ms'] <= first['timestamp_ms']:
                raise ValueError('手部反馈时间戳未推进')
            if max(abs(a-b) for a,b in zip(first['feedback_q'], second['feedback_q'])) > session['limits']['hand_change']:
                raise ValueError('手部执行器在采样期间发生变化')
            hand_feedback = second
        if hand_feedback is not None:
            with self.data_lock:
                interval = [r for r in self.history if r['received'] >= before[-1]['received']]
            if not interval or interval[0]['received']-before[-1]['received'] > .03:
                raise ValueError('手部读取期间状态记录不完整')
            self.check_window(self.observations(session, interval), session, entire_window=True)
        current, averaging = average_observation(rows, lambda q:measured_wrist(self.model, session['side'], q), session['s_model'])
        phase = data.get('phase', 'unified')
        # Pair each actual frame first; never combine a one-second actual mean
        # with an arbitrary latest command. Missing coverage stays unavailable.
        paired = [self.reference(r, session) for r in rows]
        valid_refs = [value for value, _ in paired if value is not None]
        ref = None
        ref_reason = next((reason for value, reason in paired if value is None), '')
        if len(valid_refs) == len(rows):
            reference_q = {n:float(np.mean([v['reference_q'][n] for v in valid_refs])) for n in MOTORS}
            reference_g = measured_wrist(self.model, session['side'], reference_q)
            ref = dict(valid_refs[-1])
            ref.update(tracking(current['G'], reference_g, session['s_model']))
            ref.update(reference_q=reference_q, reference_G=reference_g.tolist(),
                joint_actual_minus_reference_rad={n:current['q'][n]-reference_q[n] for n in required_joints(self.model, session['wrist_frame'])},
                other_arm_max_error_rad=max(abs(current['q'][n]-reference_q[n]) for n in MOTORS if not n.startswith(session['side']) and n!='waist_yaw_joint'),
                skew_s=max(v['skew_s'] for v in valid_refs), paired_frames=len(valid_refs),
                time_basis='per-frame local receipt pairing, then average actual/reference q separately; not hardware clock synchronization')
        elif valid_refs:
            ref_reason=f'一秒窗口参考覆盖不完整：{len(valid_refs)}/{len(rows)}；'+ref_reason
        if len(self.get_publishers_info_by_topic('/arm_sdk')) != 1:
            ref, ref_reason = None, '命令来源并非唯一，跟踪差不可用'
        row = dict(G=current['G'], joints=current['q'], dq=current['dq_map'], tau_est=current['tau_est'],
            source='lowstate_actual', source_topic=self.args.lowstate_topic,
            source_publishers=self.publisher_names, tick=current['tick'], ros_receipt_ns=current['ros_receipt_ns'],
            received_monotonic=current['received'], timestamp_note='One-second mean measured joints -> FK; receipt timestamp is window mean, tick is final frame; raw window retained',
            model_sha256=session['model']['sha256'], hand_feedback=hand_feedback,
            hand_mode='DDS actuator + manual rigidity assumption' if hand_feedback else 'manual fixed posture; no measured finger angles',
            target_id=data.get('target_id', '').strip(), group=data.get('group', 'fit'), phase=phase,
            confirmations=data['confirmations'], quality=quality, averaging=averaging, tracking=ref, tracking_unavailable_reason=ref_reason,
            note=data.get('note', ''), gateway=dict(self.diag),
            raw_window=[{k:r[k] for k in ('tick','received','ros_receipt_ns','q','dq_map','tau_est')} for r in rows])
        with self.session_lock:
            if session['session_id'] != self.session['session_id'] or session['model'] != self.session['model']:
                raise ValueError('采集期间会话/模型变化，拒绝混合样本')
            row = append_sample(self.session, row)
            self.selected = row['id']
            path = save_session(self.session, self.directory())
        self.message = f'已保存1条平均样本 {row["id"]}：{len(rows)}帧 / {averaging["duration_s"]:.2f}秒，点散布RMS {averaging["model_point_rms_m"]*1000:.3f} mm。可离开触点换腕姿。\n'+str(path)

    def work(self):
        while not self.stop_event.is_set():
            try:
                data = self.requests.get(timeout=.2)
            except queue.Empty:
                continue
            self.busy = True
            try:
                op = data.get('op')
                if op == 'capture' and abs(time.time()-float(data['sent_at'])) > 3.:
                    raise ValueError('采样请求在队列中过期，请重新确认本次接触')
                if data.get('session_id') != self.session['session_id']:
                    raise ValueError('会话已改变，请等待界面刷新后重试')
                if op == 'capture':
                    self.capture(data)
                    continue
                with self.session_lock:
                    if op == 'new':
                        if self.session['samples']:
                            save_session(self.session, self.directory())
                        if self.pending_xml:
                            old_session = self.session
                            self.session = new_session(side=data.get('side', 'left'), mode='live', limits=self.limits)
                            try:
                                self.accept_description(self.pending_xml)
                            except Exception:
                                self.session = old_session
                                raise
                        else:
                            if self.model_error:
                                raise ValueError('运行模型仍无效，先恢复正确robot_description：'+self.model_error)
                            self.reset_session(data.get('side', 'left'))
                        self.message = '新会话已建立；核对手臂、接触点和系统来源，再开始采样'
                    elif op == 'demo':
                        if not self.args.offline:
                            raise ValueError('合成示例只允许离线模式')
                        self.session = synthetic_session(); self.selected=self.session['samples'][0]['id']
                        self.message = '合成示例，非真机数据。可选择样本、排除、验证和导出。'
                    elif op == 'load':
                        loaded = load_session(data['path'])
                        if not self.args.offline and (loaded['mode']!='live' or loaded['model']['sha256']!=self.model_hash):
                            raise ValueError('真机只允许恢复同模型的真实会话；其他文件请用离线入口')
                        if not self.args.offline:
                            for row in loaded['samples']:
                                if not np.allclose(measured_wrist(self.model, loaded['side'], row['joints']), row['G'], atol=1e-8, rtol=0):
                                    raise ValueError('存档的实际关节与腕部FK不一致，禁止继续真机采集')
                        if self.args.offline:
                            loaded['mode']='offline'
                        self.session = loaded; self.selected = ''
                        self.message = '已加载原始样本；旧派生结果已清除，请重新求解'
                    elif op == 'select':
                        self.selected = data.get('sample_id', '')
                    elif op == 'preview':
                        self.candidate = bool(data.get('candidate'))
                    elif op in ('exclude', 'group', 'undo'):
                        if op == 'undo':
                            row = next((r for r in reversed(self.session['samples']) if not r['excluded']), None)
                        else:
                            row = next((r for r in self.session['samples'] if r['id']==data.get('sample_id')), None)
                        if row is None:
                            raise ValueError('请先选择样本')
                        if op == 'group':
                            row['group']='validation' if row['group']=='fit' else 'fit'
                        else:
                            row['excluded'] = True if op=='undo' else not row['excluded']
                        invalidate(self.session, op, row['id'])
                        save_session(self.session, self.directory())
                        self.message = '样本用途已更新，原始数据保留；请重新求解'
                    elif op == 'solve':
                        result = solve(self.session)
                        save_session(self.session, self.directory())
                        self.message = '求解完成。'+('；'.join(result['warnings']))
                    elif op == 'save':
                        self.message = '已另存：'+str(save_session(self.session, self.directory()))
                    elif op == 'export':
                        self.message = '已导出，未应用：'+str(export_session(self.session, self.directory()))
                    else:
                        raise ValueError('不支持的只读诊断操作')
            except Exception as error:
                self.message = '操作拒绝：'+str(error)
                self.get_logger().warning(self.message)
            finally:
                self.busy = False
                self.capture_progress = None
                self.requests.task_done()

    def update(self):
        try:
            with self.session_lock:
                session = copy.deepcopy(self.session)
                selected, candidate = self.selected, self.candidate
            ready = False; quality = {}; ref = None; live_g = None; source = '离线，不连接真机'
            reason = ''; ref_reason = ''; current = None
            if not self.args.offline:
                self.publisher_names = [p.node_namespace+'/'+p.node_name for p in
                                        self.get_publishers_info_by_topic(self.args.lowstate_topic)]
                source = self.args.lowstate_topic+' -> '+', '.join(self.publisher_names)
                try:
                    rows = self.observations(session)
                    if rows and time.monotonic()-rows[-1]['received'] <= session['limits']['max_age_s']:
                        current = rows[-1]; live_g = np.array(current['G'])
                        ref, ref_reason = self.reference(current, session)
                        if len(self.get_publishers_info_by_topic('/arm_sdk')) != 1:
                            ref, ref_reason = None, '命令来源并非唯一，跟踪差不可用'
                    quality = self.check_window(rows, session)
                    if len(self.publisher_names) != 1:
                        raise ValueError('实际LowState发布者必须唯一，请核对系统来源')
                    ready = True
                    reason = quality.get('warning', '')
                except Exception as error:
                    reason = str(error)
            status = dict(mode='offline' if self.args.offline else 'live', busy=self.busy,
                ready=ready, reason=reason, message=self.message, side=session['side'],
                session_id=session['session_id'], source=source, model=session['model'],
                s_model=session['s_model'], limits=session['limits'], quality=quality,
                tracking=ref, tracking_reason=ref_reason, samples=session['samples'], result=session['result'],
                selected=selected, output_directory=str(self.directory()),
                actual_joints={} if current is None else current['q'],
                actual_dq={} if current is None else current['dq_map'], gateway_phase=self.diag.get('phase', 'unknown'),
                capture_progress=self.capture_progress, stamp=time.time())
            # Keep the UI transport bounded; full raw windows remain in JSON snapshots.
            status['samples'] = [{k:v for k,v in r.items() if k not in ('raw_window','gateway')} for r in status['samples']]
            self.state_pub.publish(String(data=json.dumps(status, ensure_ascii=False, allow_nan=False)))
            ref_g = np.array(ref['reference_G']) if ref else None
            self.markers_pub.publish(build_markers(session, selected, candidate, live_g, ref_g))
        except Exception as error:
            self.get_logger().error('诊断刷新失败：'+str(error))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--offline', action='store_true')
    parser.add_argument('--session', type=Path)
    parser.add_argument('--robot-urdf', type=Path)
    parser.add_argument('--config', type=Path)
    parser.add_argument('--lowstate-topic', default='/lowstate')
    parser.add_argument('--network', default='eth0')
    parser.add_argument('--hand-peer', default='192.168.123.164')
    args, ros_args = parser.parse_known_args()
    if args.offline and os.environ.get('ROS_DOMAIN_ID') != '96':
        parser.error('离线模式只允许隔离 ROS_DOMAIN_ID=96')
    lock = open(f'/tmp/g1-tcp-a-{os.getuid()}-{os.environ.get("ROS_DOMAIN_ID", "0")}.lock', 'a')
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        parser.error('本域已有实验A后端；不启动第二个会话写入者')
    rclpy.init(args=ros_args)
    node = TcpANode(args)
    executor = MultiThreadedExecutor(num_threads=3); executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.stop_event.set(); node.worker.join(timeout=10)
        if node.hand:
            node.hand.close(abort=True)
        executor.shutdown(); node.destroy_node(); rclpy.try_shutdown(); lock.close()


if __name__ == '__main__':
    main()
