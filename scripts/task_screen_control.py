"""Screen workflow inside the existing console; no additional hardware owner."""
import copy
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
import xml.etree.ElementTree as ET

import numpy as np
from geometry_msgs.msg import Point
from moveit_msgs.msg import DisplayTrajectory, RobotTrajectory
from visualization_msgs.msg import Marker, MarkerArray

import task_calibration
import task_screen_teach as data
from task_console_handoff import supported_controller, supported_gateway
from task_hand_control import POSTURES, model_joints
from task_motion_planner import robot_state
from task_planning_config import DEFAULT_CONFIG, load_parameters, controller_bounded_parameters
from task_saved_test import range_fingerprint
from task_session import ARM_NAMES, ARMS
from vision_rgbd import import_record, validate_capture_result


class ScreenControl:
    def __init__(self, node):
        self.node = node
        self.config = data.new_config(self.identity())
        self.reference = None
        self.current_cloud = None
        self.current_capture = None
        self.aligned = False
        self.dry_checks = set()
        self.last_result = None
        self.revision = 0
        self.path = None
        self.last_markers = 0.

    def identity(self):
        n = self.node
        return dict(urdf_sha256=hashlib.sha256(Path(n.args.robot_urdf).read_bytes()).hexdigest(),
                    calibration=task_calibration.snapshot(n.calibration),
                    intrinsics_sha256=(hashlib.sha256(Path(n.args.intrinsics).read_bytes()).hexdigest()
                                       if Path(n.args.intrinsics).is_file() else None),
                    hand_mapping_sha256=hashlib.sha256((Path(__file__).parent/'task_hand_control.py').read_bytes()).hexdigest())

    def idle(self):
        n = self.node
        if (n.shutdown_requested or (n.worker is not None and n.worker.is_alive()) or
                n.session.phase in ('PLANNING', 'EXECUTION_PREFLIGHT', 'EXECUTING')):
            raise RuntimeError('屏幕操作需要空闲面板；先等待当前操作结束')

    def invalidate(self, alignment=True, clear_checks=False):
        self.revision += 1
        self.node.session.invalidate()
        if self.node.session.phase == 'PLAN_READY':
            self.node.session.phase = 'REPLAN_REQUIRED'
        if alignment:
            self.aligned = False
        if alignment or clear_checks:
            self.dry_checks.clear()
        self.last_result = None
        self.node.display_pub.publish(DisplayTrajectory())

    def same_identity(self):
        model = self.node.model
        if (any(name not in model.limits for name in data.MOVING) or
                model.robot.find("link[@name='right_index_tcp']") is None):
            raise RuntimeError('当前模型缺少右食指 TCP 或可动腰 yaw；原任务功能不受影响')
        if self.config['identity'] != self.identity():
            raise RuntimeError('模型、TCP、A/B/C 或相机配置已变化；恢复原配置或新建工位重新示教')

    def gateway_guard(self, hold=True, *, stationary_velocity=True):
        n = self.node
        if n.args.disconnected or not n.args.hardware or not n.hand_control:
            raise RuntimeError('屏幕示教/回放需要原硬件面板和手 DDS；离线面板不发送命令')
        if time.monotonic()-n.gateway_received > 1.5:
            raise RuntimeError('网关状态过期')
        if n.gateway.get('waist_mode_switching') != 'false':
            raise RuntimeError('腰控制模式仍在过渡或网关版本过旧；等待过渡完成')
        (supported_gateway if hold and stationary_velocity else supported_controller)(n.gateway)
        if hold and (n.gateway.get('phase') != 'ARMED_HOLD' or n.gateway.get('task_stop_active') != 'false'):
            raise RuntimeError('请先退出导纳并确认静止保持；不会自动 ARM 或回公共点')
        if n.gateway.get('task_waist_trajectory_supported') != 'true':
            raise RuntimeError('当前网关未启用腰轨迹；不会自动重启控制器')
        limit = n.model.limits['waist_yaw_joint']
        advertised = [float(n.gateway.get(k, 'nan')) for k in ('waist_guard_lower_rad', 'waist_guard_upper_rad')]
        if not np.allclose(advertised, [limit[0]+data.JOINT_MARGIN, limit[1]-data.JOINT_MARGIN], atol=1e-5, rtol=0):
            raise RuntimeError('模型与网关腰限幅不一致或网关版本过旧；需要下次完整启动加载同一版本，未发送运动')

    def waist_hold_reference(self):
        if self.node.gateway.get('waist_hold_reference_policy') != 'continuous_hold_v1':
            raise RuntimeError('腰部连续起点需要新版网关；安全结束后整套重启，未发送运动')
        try:
            reference = {n:float(self.node.gateway['hold_reference_'+n]) for n in data.MOVING}
        except (KeyError, TypeError, ValueError) as error:
            raise RuntimeError('网关保持参考缺失或无效，不能以实测值代替') from error
        if not all(math.isfinite(v) for v in reference.values()):
            raise RuntimeError('网关保持参考非有限值')
        return reference

    def new(self):
        with self.node.lock:
            self.idle()
            self.config = data.new_config(self.identity())
            self.reference = self.current_cloud = self.current_capture = None
            self.path = None
            self.invalidate()
            self.node.status = '新建屏幕工位；旧文件未删除，尚未示教'
        self.publish()

    def save(self, path):
        with self.node.lock:
            self.idle(); self.same_identity()
            data.save_bundle(path, self.config, self.reference)
            self.path = str(path)
            self.node.status = '屏幕工位已导出：'+str(path)

    def load(self, path, *, allow_limit_upgrade=False):
        config, reference = data.load_bundle(path)
        with self.node.lock:
            self.idle()
            if config['identity'] != self.identity():
                # Only an exact former generated URDF may be upgraded; no geometry
                # or calibration mismatch is waived along with the waist limits.
                robot = ET.fromstring(Path(self.node.args.robot_urdf).read_text())
                limit = robot.find("joint[@name='waist_yaw_joint']/limit")
                legacy = self.identity()
                if (limit is None or not np.allclose([float(limit.get('lower')), float(limit.get('upper'))],
                                                     [-math.pi/4, math.pi/4], atol=1e-12)):
                    raise RuntimeError('配置文件与当前模型/TCP/相机设置不匹配；未加载')
                limit.set('lower', str(-math.radians(35.))); limit.set('upper', str(math.radians(35.)))
                legacy['urdf_sha256'] = hashlib.sha256(ET.tostring(robot, encoding='unicode').encode()).hexdigest()
                if not allow_limit_upgrade or config['identity'] != legacy:
                    raise RuntimeError('配置文件与当前模型/TCP/相机设置不匹配；未加载')
                config['identity'] = self.identity()
                if config.get('reference'):
                    config['reference']['calibration'] = config['identity']
                if config.get('plane'): config['plane']['confirmed'] = False
            # Recompute FK, never trust an edited pose matrix as measured geometry.
            for record in config['points'].values():
                actual = self.node.model.transform('right_wrist_yaw_link', record['joints']) @ data.matrix(record['wrist_from_tcp'])
                if not np.allclose(actual, record['pelvis_from_tcp'], atol=1e-8):
                    raise ValueError('端点 FK 与所存关节不一致')
            for slot, record in config['points'].items():
                data.guarded_joints(self.node.model, [record['joints'][k] for k in data.MOVING], context=slot)
            self.config, self.reference = config, reference
            self.current_cloud = self.current_capture = None
            self.path = str(path)
            self.invalidate()
            self.node.status = '屏幕工位已加载；本次站位和两个分离位置仍需核对，不恢复执行授权'
        self.publish()

    def set_parameters(self, values):
        with self.node.lock:
            self.idle()
            updated = data.parameters(values)
            if (updated['approach_mm'] != self.config['parameters']['approach_mm'] and
                    self.config['plane'] is not None):
                self.config['plane']['confirmed'] = False
            self.config['parameters'] = updated
            self.invalidate(alignment=False, clear_checks=True)

    def record(self, slot):
        if slot not in data.SLOTS:
            raise ValueError('Unknown screen point')
        n = self.node
        with n.lock:
            self.idle(); self.same_identity(); self.gateway_guard(False)
            if n.gateway.get('phase') != 'TASK_ADMITTANCE':
                raise RuntimeError('请先使用原导纳按钮，等 TASK_ADMITTANCE 后拖动示教')
            if not self.aligned:
                raise RuntimeError('示教前请先在腰 0° 采集参考并确认工位对齐')
            q = n.session.measured(stationary=True)
            data.guarded_joints(n.model, [q[k] for k in data.MOVING], context=slot+' 记录未保存')
            generation = n.session.generation
            epoch = n.gateway['task_stop_epoch']
        def work():
            def unchanged():
                n.check()
                with n.lock:
                    if generation != n.session.generation or epoch != n.gateway['task_stop_epoch']:
                        raise RuntimeError('记录期间 STOP/会话变化；端点未保存')
            hands = self.read_stable_hands(check=unchanged)
            with n.lock:
                if generation != n.session.generation or epoch != n.gateway['task_stop_epoch']:
                    raise RuntimeError('记录期间 STOP/会话变化；端点未保存')
                self.same_identity(); self.gateway_guard(False)
                if n.gateway.get('phase') != 'TASK_ADMITTANCE':
                    raise RuntimeError('示教模式已变化')
                self.fresh_hands(hands)
                q = n.session.measured(stationary=True)
                data.guarded_joints(n.model, [q[k] for k in data.MOVING], context=slot+' 记录未保存')
                for side in ARMS: q.update(model_joints(side, hands[side]['feedback_q']))
                if max(abs(a-b) for a, b in zip(hands['right']['feedback_q'], POSTURES['index_only'])) > .03:
                    raise RuntimeError('右手需要食指伸直、其余手指收拢；不会自动改变手型')
                tcp = data.effective_tcp(n.model, q, n.calibration)
                record = data.endpoint(n.model, q, hands, tcp, n.session.source_stamp_ns, int(epoch))
                if slot == 'T': record['physical_contact'] = 'none_transition_point'
                self.config['points'][slot] = record
                self.config['plane'] = None
                self.invalidate(alignment=False, clear_checks=True)
                data.save_bundle(n.run_dir/'screen_autosave/config.json', self.config, self.reference)
                n.status = slot+' 实测端点已记录并自动保存；未发送姿态命令'
            self.publish()
        n.launch_work(work)

    def fresh_hands(self, hands):
        for side in ARMS:
            state = self.node.hand_states[side]
            if time.monotonic()-state['received'] > .5 or state['state']['timestamp_ms'] != hands[side]['timestamp_ms']:
                raise RuntimeError('手反馈不同步或过期，请重试读取')

    def read_stable_hands(self, ticket=None, check=lambda:None):
        first = self.node.read_hands(ticket)
        check()
        time.sleep(.05)
        check()
        hands = self.node.read_hands(ticket)
        check()
        self.fresh_hands(hands)
        for side in ARMS:
            if (hands[side]['timestamp_ms'] <= first[side]['timestamp_ms'] or
                    max(abs(a-b) for a,b in zip(first[side]['feedback_q'], hands[side]['feedback_q'])) > .01):
                raise RuntimeError('手反馈时间戳未推进或手指仍在运动；端点/回放未授权')
        return hands

    def fit(self):
        with self.node.lock:
            self.idle(); self.same_identity()
            self.config['plane'] = data.fit_plane(self.config['points'])
            self.invalidate(alignment=False, clear_checks=True)
        self.publish()

    def flip_normal(self):
        with self.node.lock:
            self.idle()
            plane = self.config['plane']
            if plane is None: raise RuntimeError('先拟合屏幕平面')
            plane['outward_normal'] = (-np.asarray(plane['outward_normal'])).tolist()
            plane['confirmed'] = False
            self.invalidate(alignment=False, clear_checks=True)
        self.publish()

    def confirm_plane(self):
        with self.node.lock:
            self.idle(); self.same_identity()
            plane = self.config['plane']
            if plane is None: raise RuntimeError('先拟合屏幕平面')
            if plane['max_mm'] > 5.:
                raise RuntimeError('平面最大残差超过 5 mm；请复核触点，不能自动投影修正')
            transition = data.matrix(self.config['points']['T']['pelvis_from_tcp'])[:3, 3]
            distance = float((transition-np.asarray(plane['center_m'])) @ plane['outward_normal'])
            if distance < self.config['parameters']['approach_mm']/1000.:
                raise RuntimeError('公共点不在所选法向外侧，或离屏距离不足；检查法向和公共点')
            self.invalidate(alignment=False, clear_checks=True)
            plane['confirmed'] = True

    def alignment_pose(self):
        q = self.node.session.measured(stationary=True)
        yaw = q.get('waist_yaw_joint', float('nan'))
        if not math.isfinite(yaw) or abs(yaw-data.ALIGNMENT_YAW) > data.ALIGNMENT_YAW_TOLERANCE:
            raise RuntimeError(f'对齐要求腰 0° ±0.5°，当前 {math.degrees(yaw):.2f}°；先规划并执行腰部恢复到 0°')
        return q

    def alignment_capture(self):
        self.alignment_pose()
        capture = self.current_capture
        if capture is None:
            raise RuntimeError('先在腰 0° 刷新当前点云')
        yaw = capture.get('waist_yaw_rad', float('nan'))
        if not math.isfinite(yaw) or abs(yaw-data.ALIGNMENT_YAW) > data.ALIGNMENT_YAW_TOLERANCE:
            raise RuntimeError('快照没有合格的腰 0° 记录，请重新采集')
        return capture

    def capture(self):
        n = self.node
        with n.lock:
            self.idle(); self.same_identity()
            if n.args.disconnected: raise RuntimeError('离线面板不能请求相机')
            n.session.require_planning_ready()
            self.alignment_pose()
            self.invalidate()
            ticket = n.session.begin('screen_capture', 'right', .15, .15)
            before = n.session.measured(stationary=True)
            n.status = '采集工位 RGB-D 快照；未发送运动或碰撞请求'
        def work():
            directory = n.run_dir/f'screen_capture_{ticket.generation:04d}'
            directory.mkdir()
            capture = directory/'capture.json'
            process = None
            try:
                with (directory/'capture.log').open('w') as log:
                    process = subprocess.Popen([str(Path(__file__).parent/'run_task_vision_capture.sh'),
                        n.args.network, str(capture), '--save-artifacts'], stdout=log, stderr=subprocess.STDOUT,
                        start_new_session=True, env=dict(os.environ, G1_VISION_DDS_PEER=n.args.vision_peer))
                    deadline = time.monotonic()+75
                    while process.poll() is None:
                        n.check(ticket)
                        with n.lock:
                            q = n.session.measured(stationary=True)
                            self.alignment_pose()
                            if any(abs(q[k]-before[k]) > .01 for k in ARM_NAMES+('waist_yaw_joint',)):
                                raise RuntimeError('采集期间机器人移动；快照作废')
                        if time.monotonic() > deadline: raise TimeoutError('工位采集超时')
                        time.sleep(.04)
                raw = json.loads(capture.read_text())
                validate_capture_result(raw, 'prepare')
                if process.returncode not in (0, 2): raise RuntimeError('工位采集失败')
                manifest = import_record(capture, host=n.args.ssh, intrinsics=Path(n.args.intrinsics))
                n.check(ticket)
                with n.lock:
                    q = n.session.measured(stationary=True)
                    self.alignment_pose()
                    if n.session.snapshot_motion or any(abs(q[k]-before[k]) > .01 for k in ARM_NAMES+('waist_yaw_joint',)):
                        raise RuntimeError('采集期间机器人移动；快照作废')
                old = np.asarray(manifest['torso_from_color_optical_row_major'])
                camera = task_calibration.camera_transform(old, n.calibration)
                with np.load(directory/'cloud.npz', allow_pickle=False) as archive:
                    xyz = archive['xyz'].copy(); uv = archive['uv'].copy()
                optical = (xyz-old[:3, 3]) @ old[:3, :3]
                optical = task_calibration.depth_points(optical, n.calibration)
                torso = n.model.transform('torso_link', before)
                cloud = optical @ camera[:3, :3].T+camera[:3, 3]
                cloud = data.cloud_array(cloud @ torso[:3, :3].T+torso[:3, 3])
                with n.lock:
                    n.session.check(ticket)
                    self.current_capture = dict(directory=str(directory), xyz=cloud, uv=uv,
                        captured_at=data.now(), received=time.monotonic(), source=raw['observation']['sequence'],
                        camera_serial=raw['observation']['camera_serial'], calibration=self.config['identity'],
                        generation=ticket.generation, waist_yaw_rad=before['waist_yaw_joint'])
                    self.current_cloud = cloud
                    n.session.invalidate(); n.session.phase = 'SCREEN_CAPTURED'
                    n.status = '新点云已采集（青色）；可框选固定特征建立参考，或核对金色参考'
                self.publish()
            except Exception as error:
                with n.lock: n.session.fail(ticket)
                data.write_json(directory/'failure.json', dict(message=str(error), hardware_executed=False))
                raise
            finally:
                if process is not None and process.poll() is None:
                    os.killpg(process.pid, signal.SIGTERM)
                    try: process.wait(timeout=3)
                    except subprocess.TimeoutExpired:
                        os.killpg(process.pid, signal.SIGKILL); process.wait()
        n.launch_work(work)

    def set_reference(self, rectangle):
        with self.node.lock:
            self.idle(); self.same_identity()
            capture = self.alignment_capture()
            if time.monotonic()-capture['received'] > 120:
                raise RuntimeError('当前快照超过两分钟，请刷新后建立参考')
            u0, v0, u1, v1 = rectangle
            uv = capture['uv']
            selected = (uv[:, 0] >= u0) & (uv[:, 0] <= u1) & (uv[:, 1] >= v0) & (uv[:, 1] <= v1)
            self.reference = data.cloud_array(capture['xyz'][selected]).copy()
            self.config['reference'] = dict(captured_at=capture['captured_at'],
                source=capture['source'], camera_serial=capture['camera_serial'],
                roi_pixels=list(rectangle), frame='pelvis', calibration=self.config['identity'],
                waist_yaw_rad=capture['waist_yaw_rad'], alignment_waist_yaw_rad=data.ALIGNMENT_YAW)
            self.invalidate()
            data.save_bundle(self.node.run_dir/'screen_autosave/config.json', self.config, self.reference)
        self.publish()

    def confirm_alignment(self):
        with self.node.lock:
            self.idle(); self.same_identity()
            if self.reference is None or self.current_capture is None:
                raise RuntimeError('需要参考点云和本次新点云')
            self.alignment_capture()
            info = self.config['reference']
            yaw = info.get('waist_yaw_rad', float('nan'))
            if (info.get('alignment_waist_yaw_rad') != data.ALIGNMENT_YAW or not math.isfinite(yaw) or
                    abs(yaw-data.ALIGNMENT_YAW) > data.ALIGNMENT_YAW_TOLERANCE):
                raise RuntimeError('旧参考没有腰 0° 对齐记录，请在腰 0° 重新框选保存参考物')
            if time.monotonic()-self.current_capture['received'] > 120:
                raise RuntimeError('当前快照超过两分钟，请刷新后核对')
            if self.current_capture['camera_serial'] != self.config['reference']['camera_serial']:
                raise RuntimeError('相机序列号已变化')
            self.invalidate()
            self.aligned = True
            self.node.status = '本次工位人工对齐已确认；接触回放前仍需两个分离点的无接触核对'

    def ready_guard(self, selected, dry):
        n = self.node
        self.same_identity(); self.gateway_guard()
        data.validate(self.config)
        if selected == 'WAIST':
            return n.session.measured(stationary=True)
        if selected == 'T':
            if 'T' not in self.config['points']:
                raise RuntimeError('请先记录或加载公共点 T')
            return n.session.measured(stationary=True)
        if not self.aligned: raise RuntimeError('先刷新点云并确认本次工位对齐')
        if selected not in data.SLOTS[1:] or any(k not in self.config['points'] for k in data.SLOTS):
            raise RuntimeError('先记录公共点和五个屏幕点，并选择一个屏幕点')
        if not self.config['plane'] or not self.config['plane']['confirmed']:
            raise RuntimeError('先拟合并确认朝外法向')
        plane = self.config['plane']
        transition = data.matrix(self.config['points']['T']['pelvis_from_tcp'])[:3, 3]
        if (plane['max_mm'] > 5. or float((transition-np.asarray(plane['center_m'])) @ plane['outward_normal']) <
                self.config['parameters']['approach_mm']/1000.):
            raise RuntimeError('平面残差或公共点外侧距离不满足当前参数，请复核')
        if not dry and len(self.dry_checks) < 2:
            raise RuntimeError('接触回放前，需要两个分离位置的无接触试运行及人工对位确认')
        q = n.session.measured(stationary=True)
        target = self.config['points']['T']['joints']
        # The passive left arm stays at its measured start, not its old teaching pose.
        joint = max(data.MOVING, key=lambda k:abs(q[k]-target[k]))
        if abs(q[joint]-target[joint]) > .025:
            raise RuntimeError(f'尚未到公共点 T（{joint} 差 {math.degrees(abs(q[joint]-target[joint])):.2f} 度）；'
                               '请先“规划移动到 T”并确认执行，无需手动复刻 T')
        return q

    def plan(self, selected, dry, *, return_confirmed=False, waist_target_deg=None):
        n = self.node
        waist_only = selected == 'WAIST'
        returning = selected in ('T', 'WAIST')
        if returning and return_confirmed is not True:
            raise RuntimeError('恢复前须明确确认指尖已脱离、站位/支撑未变及转动路径净空')
        with n.lock:
            self.idle(); self.ready_guard(selected, dry)
            if waist_only:
                if type(waist_target_deg) not in (int, float) or not math.isfinite(waist_target_deg):
                    raise ValueError('腰部目标角度必须为有限数值')
                data.guarded_joints(n.model, [math.radians(waist_target_deg)], ('waist_yaw_joint',), '腰部恢复目标')
            self.last_result = None
            p = self.config['parameters']
            ticket = n.session.begin('screen', 'right', p['velocity'], p['acceleration'])
            revision = self.revision
            epoch = int(n.gateway['task_stop_epoch'])
            config = copy.deepcopy(self.config)
            n.display_pub.publish(DisplayTrajectory())
            n.status = (f'核对实测状态与保持参考，规划腰部恢复到 {waist_target_deg:.2f}°；尚未发送运动' if waist_only else
                        '从当前实测姿态规划移动到 T；尚未发送运动' if returning else
                        '生成 '+selected+' 单点屏幕路径；不新增碰撞检测')
        directory = n.run_dir/f'screen_plan_{ticket.generation:04d}'
        def work():
            try:
                hands = self.read_stable_hands(ticket, lambda:n.check(ticket))
                params, config_record = load_parameters(getattr(n.args, 'planning_config', DEFAULT_CONFIG))
                with n.lock:
                    n.session.check(ticket); self.fresh_hands(hands)
                    q = self.ready_guard(selected, dry)
                    for side in ARMS: q.update(model_joints(side, hands[side]['feedback_q']))
                    if not waist_only:
                        taught = config['points'][selected]['hands']['right']['feedback_q']
                        if max(abs(a-b) for a, b in zip(hands['right']['feedback_q'], taught)) > .02:
                            raise RuntimeError('实际右手手型与示教不匹配；不会自动改变手型')
                    params, config_record = controller_bounded_parameters(params, config_record, n.gateway)
                    hold_reference = self.waist_hold_reference() if waist_only else None
                from task_screen_motion import plan as build, return_to_transition
                plan = (return_to_transition(n.model, config, q, params, lambda:n.check(ticket),
                                            waist_target=math.radians(waist_target_deg) if waist_only else None,
                                            hold_reference=hold_reference) if returning else
                        build(n.model, config, selected, q, dry, params, lambda:n.check(ticket)))
                proof = plan['screen']
                if returning: proof['return_confirmed'] = True
                proof.update(revision=revision, stop_epoch=epoch, config_sha256=data.fingerprint(config),
                    waist_control_mode=n.gateway['waist_control_mode'],
                    waist_control_revision=n.gateway['waist_control_revision'],
                    trajectory_sha256=range_fingerprint(plan['segments']))
                plan.update(directory=str(directory), planning_config=config_record)
                data.save_bundle(directory/'config.json', config, self.reference)
                from rosidl_runtime_py.convert import message_to_ordereddict
                data.write_json(directory/'plan.json', {**plan, 'segments':[
                    {**s, 'trajectory':message_to_ordereddict(s['trajectory'])} for s in plan['segments']]})
                with n.lock:
                    n.session.check(ticket)
                    self.execution_guard(plan)
                    n.session.ready(ticket, plan)
                    display = DisplayTrajectory(trajectory_start=robot_state(q), trajectory=[
                        RobotTrajectory(joint_trajectory=s['trajectory']) for s in plan['segments']])
                    n.display_pub.publish(display)
                    n.status = (f'腰部连续恢复到 {waist_target_deg:.2f}° ' if waist_only else
                                '移动到 T ' if returning else selected+(' 无接触' if dry else ' 触点'))+f'计划完成，{sum(s["duration"] for s in plan["segments"]):.1f} s；检查路径后单次执行'
            except Exception as error:
                with n.lock: n.session.fail(ticket)
                data.write_json(directory/'failure.json', dict(message=str(error), hardware_executed=False))
                raise
        n.launch_work(work)

    def execution_guard(self, plan, running=False, preflight=False):
        n = self.node; proof = plan['screen']
        waist_only = proof.get('operation') == 'waist_recovery'
        returning = proof.get('operation') in ('return_T', 'waist_recovery')
        self.gateway_guard(not running)
        if any(proof.get(key) != n.gateway.get(key) for key in
               ('waist_control_mode', 'waist_control_revision')):
            raise RuntimeError('腰控制模式已改变；重新规划并执行预演核对')
        if n.gateway.get('task_stop_active') != 'false':
            raise RuntimeError('屏幕执行期间 STOP 已生效')
        if returning:
            if proof.get('return_confirmed') is not True or proof['selected'] != ('WAIST' if waist_only else 'T') or len(plan['segments']) != 1:
                raise RuntimeError('缺少本次恢复动作的独立确认或单段计划')
            if waist_only:
                if proof.get('hold_reference_policy') != 'continuous_hold_v1':
                    raise RuntimeError('旧腰部计划不支持连续保持起点，请重新规划')
                reference = proof.get('hold_reference', {})
                if (set(reference) != set(data.MOVING) or
                        any(not math.isfinite(reference[k]) or abs(reference[k]-plan['start'][k]) > .025
                            for k in data.MOVING)):
                    raise RuntimeError('腰部保持参考与实测起点不一致')
                # Post-action settling checks the endpoint, not the old start reference.
                if not running and (preflight or n.session.phase != 'EXECUTING'):
                    current = self.waist_hold_reference()
                    if any(abs(current[k]-reference[k]) > 1e-12 for k in data.MOVING):
                        raise RuntimeError('网关保持参考已改变，请重新规划腰部运动')
            target = ({**plan['start'], **reference, 'waist_yaw_joint':proof['target_waist_rad']} if waist_only else
                      self.config['points']['T']['joints'])
            if any(abs(plan['segments'][0]['end'][k]-target[k]) > 1e-9 for k in data.MOVING):
                raise RuntimeError('恢复动作的目标已变化')
            if waist_only:
                data.guarded_joints(n.model, [proof['target_waist_rad']], ('waist_yaw_joint',), '腰部恢复目标')
                trajectory = plan['segments'][0]['trajectory']
                if (list(trajectory.joint_names) != list(data.MOVING) or
                        any(abs(trajectory.points[0].positions[i]-reference[k]) > 1e-12 for i,k in enumerate(data.MOVING)) or
                        any(any(abs(p.positions[i]-reference[k]) > 1e-12 or
                                    abs(p.velocities[i]) > 1e-12 or abs(p.accelerations[i]) > 1e-12
                                for i,k in enumerate(ARMS['right']))
                            for p in trajectory.points)):
                    raise RuntimeError('腰部恢复不得改变手臂关节指令')
        if (not returning and not self.aligned) or proof['revision'] != self.revision or proof['stop_epoch'] != int(n.gateway['task_stop_epoch']):
            raise RuntimeError('屏幕工位/STOP/配置授权已变化，请重新核对和规划')
        if task_calibration.snapshot(n.calibration) != self.config['identity']['calibration']:
            raise RuntimeError('屏幕 TCP/相机设置已变化')
        if preflight:
            self.same_identity()
            if (proof['config_sha256'] != data.fingerprint(self.config) or
                    proof['trajectory_sha256'] != range_fingerprint(plan['segments'])):
                raise RuntimeError('屏幕计划内容已变化，请重新规划')
        if running:
            n.session.measured()

    def completed(self, plan):
        if plan['screen'].get('operation') == 'waist_recovery':
            self.last_result = None
            target = plan['screen']['target_waist_rad']
            actual = self.node.session.measured()['waist_yaw_joint']
            data.write_json(Path(plan['directory'])/'waist_result.json', dict(
                waist_control_mode=plan['screen']['waist_control_mode'],
                waist_control_revision=plan['screen']['waist_control_revision'],
                hold_reference_policy=plan['screen']['hold_reference_policy'],
                command_start_deg=math.degrees(plan['screen']['hold_reference']['waist_yaw_joint']),
                measured_start_deg=math.degrees(plan['start']['waist_yaw_joint']),
                waist_command_log_path=self.node.gateway.get('last_waist_command_log_path'),
                target_deg=math.degrees(target), measured_deg=math.degrees(actual),
                error_deg=math.degrees(actual-target), completed_at=data.now()))
            self.node.status = f'腰部恢复完成；目标 {math.degrees(target):.2f}°，实测 {math.degrees(actual):.2f}°；对齐仍须满足 0° ±0.5°'
            return
        if plan['screen'].get('operation') == 'return_T':
            self.last_result = None
            self.node.status = '已到公共点 T 并保持；未执行触屏。可继续工位核对或单点规划。'
            return
        self.last_result = dict(plan['screen'], directory=plan['directory'], completed_at=data.now(), outcome='pending')
        self.node.status = plan['screen']['selected']+' 已回公共点；'+('无接触对位待人工确认' if plan['screen']['dry'] else '点击结果待人工确认')

    def confirm_result(self, success):
        with self.node.lock:
            self.idle()
            result = self.last_result
            if result is None or result['outcome'] != 'pending': raise RuntimeError('没有待确认的屏幕结果')
            if not self.aligned or result['revision'] != self.revision:
                raise RuntimeError('结果所属工位已失效')
            result['outcome'] = 'operator_confirmed_success' if success else 'operator_confirmed_failure'
            if success and result['dry']:
                slot = result['selected']
                xyz = data.matrix(self.config['points'][slot]['pelvis_from_tcp'])[:3, 3]
                if not self.dry_checks or any(np.linalg.norm(xyz-data.matrix(self.config['points'][k]['pelvis_from_tcp'])[:3, 3]) >= .02 for k in self.dry_checks):
                    self.dry_checks.add(slot)
            if not success:
                self.dry_checks.clear()
            data.write_json(Path(result['directory'])/'operator_result.json', result)
            self.node.status = '结果已记录；未自动重试'

    def publish(self):
        markers = MarkerArray()
        for index in range(18):
            markers.markers.append(Marker(ns='screen_teach', id=index, action=Marker.DELETE))
        def add(index, kind, color):
            m = Marker(ns='screen_teach', id=index, type=kind, action=Marker.ADD)
            m.header.frame_id = 'pelvis'; m.pose.orientation.w = 1.
            m.color.r, m.color.g, m.color.b, m.color.a = color
            markers.markers.append(m)
            return m
        for i, slot in enumerate(data.SLOTS):
            if slot not in self.config['points']: continue
            xyz = data.matrix(self.config['points'][slot]['pelvis_from_tcp'])[:3, 3]
            m = add(i, Marker.SPHERE, (.9, .3, .2, 1.))
            m.scale.x = m.scale.y = m.scale.z = .009
            m.pose.position = Point(**dict(zip('xyz', xyz.tolist())))
            text = add(i+6, Marker.TEXT_VIEW_FACING, (1., 1., 1., 1.))
            text.scale.z = .015; text.text = slot
            text.pose.position = Point(x=float(xyz[0]), y=float(xyz[1]), z=float(xyz[2]+.018))
        plane = self.config['plane']
        if plane:
            m = add(12, Marker.ARROW, (.3, 1., .3, 1.))
            a = np.asarray(plane['center_m']); b = a+.10*np.asarray(plane['outward_normal'])
            m.points = [Point(**dict(zip('xyz', p.tolist()))) for p in (a, b)]
            m.scale.x, m.scale.y, m.scale.z = .003, .01, .02
            # Outline the taught region, not an inferred full screen perimeter.
            from scipy.spatial import ConvexHull
            xyz = np.array([data.matrix(self.config['points'][slot]['pelvis_from_tcp'])[:3, 3] for slot in data.SLOTS[1:]])
            _, _, axes = np.linalg.svd(xyz-a)
            hull = ConvexHull((xyz-a) @ axes[:2].T).vertices
            outline = add(15, Marker.LINE_STRIP, (.3, 1., .3, .8))
            outline.scale.x = .002
            outline.points = [Point(**dict(zip('xyz', xyz[i].tolist()))) for i in list(hull)+[hull[0]]]
        for index, xyz, color in ((13, self.reference, (1., .75, .15, .85)),
                                  (14, self.current_cloud, (.1, .85, 1., .6))):
            if xyz is None: continue
            m = add(index, Marker.POINTS, color)
            m.scale.x = m.scale.y = .003
            m.points = [Point(x=float(p[0]), y=float(p[1]), z=float(p[2])) for p in xyz[::max(1, math.ceil(len(xyz)/12000))]]
        self.node.marker_pub.publish(markers)
        self.last_markers = time.monotonic()
