"""Offline-first RGB-D consistency sessions and a read-only Qt/ROS bridge."""
from __future__ import annotations

import argparse
import copy
import csv
from datetime import datetime
import hashlib
import ipaddress
import json
import os
from pathlib import Path
import queue
import re
import signal
import shutil
import subprocess
import tempfile
import threading
import time
import uuid
import xml.etree.ElementTree as ET

import cv2
import numpy as np

from camera_board import BOARD, LIMITS, FRAME, detect_board, rgb_pose, depth_metrics
from vision_rgbd import validate_pair, validate_capture_result, import_camera_record
from task_sdk_environment import sdk_environment

ROOT = Path(__file__).resolve().parents[1]
PROTOCOL = 'camera_b_v1'
SCHEMA = 'g1_camera_b_session_v1'


def existing_backends():
    found = []
    for proc in Path('/proc').glob('[0-9]*'):
        try:
            if proc.stat().st_uid != os.getuid() or int(proc.name) == os.getpid():
                continue
            args = (proc/'cmdline').read_bytes().split(b'\0')
            if len(args) > 1 and args[1] == str(Path(__file__).resolve()).encode():
                found.append(int(proc.name))
        except (OSError, ValueError):
            continue
    return found


def stop_existing_backends():
    """Explicit --replace affects only this exact B Python entrypoint."""
    pids = existing_backends()
    for pid in pids:
        try:
            os.kill(pid, signal.SIGINT)
        except ProcessLookupError:
            pass
    deadline = time.monotonic()+30
    while any(pid in existing_backends() for pid in pids):
        if time.monotonic() > deadline:
            raise RuntimeError('原B后端尚未正常退出；不强杀。请关闭B窗口后重试。')
        time.sleep(.2)
    if pids:
        print('原B后端已正常退出并保存；未操作任务/A或机器人控制器。', flush=True)


class VisionReadOnlyClient:
    def __init__(self, network='eth0', host='unitree@192.168.0.249', peer='192.168.123.164'):
        if not re.fullmatch(r'[a-zA-Z0-9_.:-]+', network):
            raise ValueError('无效网卡名称')
        if not re.fullmatch(r'[a-zA-Z0-9_.-]+@[a-zA-Z0-9_.-]+', host):
            raise ValueError('SSH必须为明确的 user@host')
        self.network, self.host, self.peer = network, host, str(ipaddress.ip_address(peer))
        self.binary = ROOT/'build/probe_camera_b_vision'
        self.last_frame = None

    def environment(self):
        env = sdk_environment()
        # Only the child RPC process joins robot domain 0. Qt/backend remain on 96.
        root = ET.Element('CycloneDDS'); domain = ET.SubElement(root, 'Domain')
        general = ET.SubElement(domain, 'General'); interfaces = ET.SubElement(general, 'Interfaces')
        ET.SubElement(interfaces, 'NetworkInterface', name=self.network, priority='default', multicast='default')
        discovery = ET.SubElement(domain, 'Discovery')
        ET.SubElement(discovery, 'ParticipantIndex').text = 'none'
        peers = ET.SubElement(discovery, 'Peers'); ET.SubElement(peers, 'Peer', Address=self.peer)
        env['CYCLONEDDS_URI'] = ET.tostring(root, encoding='unicode')
        env['ROS_DOMAIN_ID'] = '0'; env.pop('ROS_LOCALHOST_ONLY', None)
        return env

    def build(self):
        import fcntl
        sdk = Path('/home/syf/unitree_sdk2')
        include = ROOT/'third_party/g1-rh56dfx-control/src'
        inputs = [ROOT/'scripts/probe_task_vision.cpp', include/'vision/vision_api.hpp',
                  include/'vision/vision_client.hpp', sdk/'lib/x86_64/libunitree_sdk2.a']
        self.binary.parent.mkdir(exist_ok=True)
        with (self.binary.parent/'camera_b_build.lock').open('a') as lock:
            fcntl.flock(lock, fcntl.LOCK_EX)
            if self.binary.exists() and self.binary.stat().st_mtime_ns >= max(p.stat().st_mtime_ns for p in inputs):
                return
            with tempfile.TemporaryDirectory(prefix='.camera-b-build-', dir=self.binary.parent) as tmp:
                target = Path(tmp)/'probe'
                cmd = ['g++', '-std=c++17', '-O2', '-pthread', str(inputs[0]), '-I'+str(include),
                       '-I'+str(sdk/'include'), '-I'+str(sdk/'thirdparty/include'),
                       '-I'+str(sdk/'thirdparty/include/ddscxx'), str(inputs[-1]),
                       '-L'+str(sdk/'thirdparty/lib/x86_64'), '-Wl,-rpath,'+str(sdk/'thirdparty/lib/x86_64'),
                       '-lddscxx', '-lddsc', '-o', str(target)]
                subprocess.run(cmd, check=True, timeout=120)
                os.replace(target, self.binary)

    def rpc(self, directory, status_only=False):
        self.build(); directory.mkdir(parents=True, exist_ok=False)
        path = directory/'capture.json'
        flags = ['--status-only'] if status_only else ['--save-artifacts', '--camera-only']
        with (directory/'rpc.log').open('x') as log:
            result = subprocess.run([str(self.binary), self.network, str(path), *flags], env=self.environment(),
                                    stdout=log, stderr=subprocess.STDOUT, timeout=75)
        if not path.exists():
            raise RuntimeError(f'视觉RPC未返回元数据，退出码{result.returncode}；详见 {directory}/rpc.log')
        capture = read_json(path)
        if capture.get('status_result') != 0:
            raise RuntimeError(f"视觉服务GetStatus失败/超时：{capture.get('status_result')}；不自动重启机载服务")
        status = capture.get('status', {})
        if status.get('camera_ready') is not True or status.get('model_ready') is not True:
            raise RuntimeError('现有视觉服务相机/模型未就绪：'+str(status.get('last_error', '')))
        return capture

    def check(self):
        links = json.loads(subprocess.check_output(['ip', '-j', 'address', 'show', 'dev', self.network], timeout=5))
        if not links or 'UP' not in links[0].get('flags', []) or not any(v.get('family') == 'inet' for v in links[0].get('addr_info', [])):
            raise RuntimeError('网卡未UP或没有IPv4地址：'+self.network)
        routes = json.loads(subprocess.check_output(['ip', '-j', 'route', 'get', self.peer], timeout=5))
        if not routes or routes[0].get('dev') != self.network:
            raise RuntimeError('到机载地址的实际路由不经过所选网卡')
        subprocess.run(['ssh', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=5', '-o', 'StrictHostKeyChecking=yes',
                        self.host, 'true'], check=True, timeout=10, capture_output=True)
        directory = ROOT/'data/camera_b/connection_checks'/uuid.uuid4().hex
        capture = self.rpc(directory, status_only=True)
        return dict(ready=True, checked_at=time.time(), network=self.network, host=self.host,
                    message='SSH与vision_service就绪（最近一次检查；未采图）',
                    record=str(directory/'capture.json'), service=capture['status'])

    def capture(self, directory, progress=None):
        if progress:
            progress('正在向原vision_service请求图像；板与相机保持静止')
        capture = self.rpc(directory)
        observation = validate_capture_result(capture, 'prepare')
        start, end = capture.get('request_started_unix_ms', 0), capture.get('client_end_unix_ms', 0)
        if not np.isfinite([start, end]).all() or not 0 < start <= end or end-start > 75000:
            raise ValueError('本次RPC请求时间区间无效')
        self.check_new_frame(observation)
        if progress:
            progress('图像已返回，正在下载与检测；当前显示仍可能是上一张快照')
        import_camera_record(directory/'capture.json', host=self.host)
        validate_bundle(directory)
        self.last_frame = {key: observation[key] for key in ('camera_serial', 'color_frame_number',
                           'depth_frame_number', 'color_timestamp_ms', 'depth_timestamp_ms')}
        return dict(request_started_unix_ms=start, client_end_unix_ms=end,
                    requested_at=datetime.fromtimestamp(start/1000).astimezone().isoformat(),
                    frame=self.last_frame, exposure_sync_verified=False,
                    freshness_note='Per-request identity and increasing device frames; server wall time is post-inference, not exposure')

    def check_new_frame(self, observation):
        keys = ('color_frame_number', 'depth_frame_number', 'color_timestamp_ms', 'depth_timestamp_ms')
        if any(not np.isfinite(observation.get(k, np.nan)) or observation[k] <= 0 for k in keys):
            raise ValueError('图像帧号或设备时间无效')
        if self.last_frame:
            if observation['camera_serial'] != self.last_frame['camera_serial']:
                raise ValueError('相机序列号改变，请新建会话')
            if any(observation[k] <= self.last_frame[k] for k in keys):
                raise ValueError('图像帧号/时间未前进；若相机重启，请新建会话，不接受旧帧')


def read_json(path):
    return json.loads(Path(path).read_text(encoding='utf-8'))


def write_json(path, payload):
    with Path(path).open('x', encoding='utf-8') as stream:
        json.dump(payload, stream, indent=2, ensure_ascii=False, allow_nan=False)
        stream.write('\n')


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def local_file(root, relative):
    path = (root/relative).resolve()
    if not path.is_relative_to(root.resolve()) or not path.is_file():
        raise ValueError('归档文件缺失或路径越界：'+str(relative))
    return path


def validate_bundle(directory):
    capture = read_json(directory/'capture.json')
    observation = capture.get('observation')
    code = capture.get('observation_result')
    if capture.get('status_result') != 0 or code not in (0, 102):
        raise ValueError('归档没有成功的相机响应（允许未检测到踏板 102）')
    if not isinstance(observation, dict) or observation.get('code') != code:
        raise ValueError('相机响应 code 不匹配')
    if type(capture.get('request_id')) is not int or capture['request_id'] <= 0 or observation.get('request_id') != capture['request_id']:
        raise ValueError('相机 request_id 不匹配')
    record = read_json(directory/'server_result.json')
    validate_pair(observation, record)
    if abs(abs(float(observation['color_timestamp_ms'])-float(observation['depth_timestamp_ms']))-
           float(record['timestamp_delta_ms'])) > .01:
        raise ValueError('彩深时间差与记录不一致')
    if observation.get('units') not in ('m', 'meter', 'meters'):
        raise ValueError('相机几何单位不是米')
    return observation


class CameraBSession:
    def __init__(self, root=None, mode='offline'):
        if mode not in ('offline', 'live'):
            raise ValueError('未知会话模式')
        self.root = Path(root or ROOT/'data/camera_b/sessions'/uuid.uuid4().hex)
        self.root.mkdir(parents=True, exist_ok=False)
        self.state = dict(schema=SCHEMA, protocol=PROTOCOL, session_id=self.root.name,
                          created_at=datetime.now().isoformat(), mode=mode,
                          board=copy.deepcopy(BOARD), limits=copy.deepcopy(LIMITS),
                          layout=None, camera_identity=None, samples=[], audit=[],
                          applied=False, hardware_validated=False, frame_id=FRAME)
        self.selected = ''
        self.last_snapshot = ''

    def event(self, action, **fields):
        self.state['audit'].append(dict(at=datetime.now().isoformat(), action=action, **fields))

    def selected_sample(self):
        return next((s for s in self.state['samples'] if s['id'] == self.selected), None)

    def verify_files(self, sample):
        for relative, sha in sample['sha256'].items():
            if digest(local_file(self.root, relative)) != sha:
                raise ValueError('原始文件 SHA256 已改变：'+relative)

    def import_path(self, source, group='fit', note='', acquisition=None):
        if group not in ('fit', 'validation') and not (group == 'preview' and acquisition):
            raise ValueError('未知样本用途')
        source = Path(source).expanduser().resolve()
        if source.is_file() and source.suffix.lower() == '.json':
            source = source.parent
        is_bundle = source.is_dir()
        if is_bundle:
            names = ['rgb.png', 'depth_aligned.png', 'capture.json', 'server_result.json', 'manifest.json']
            if not all((source/n).is_file() for n in names):
                raise ValueError('RGB-D 目录缺少原图、capture/server_result 或 manifest；可单独导入 RGB 预览')
            manifest = read_json(source/'manifest.json')
            for name in names[:-1]:
                if manifest.get('sha256', {}).get(name) != digest(source/name):
                    raise ValueError('源 RGB-D manifest 未验证或文件被修改：'+name)
            identity = digest(source/'rgb.png')+digest(source/'depth_aligned.png')
        elif source.is_file() and source.suffix.lower() in ('.png', '.jpg', '.jpeg'):
            names = [source.name]
            identity = digest(source)
        else:
            raise ValueError('请选择 RGB 图片或已有 RGB-D 归档目录')
        if any(s['source_identity'] == identity for s in self.state['samples']):
            raise ValueError('该图像/图像对已导入；不把同一帧重复计为新样本')
        sample_id = f"b_{len(self.state['samples'])+1:04d}_{uuid.uuid4().hex[:8]}"
        directory = self.root/'samples'/sample_id; directory.mkdir(parents=True)
        for name in names:
            shutil.copyfile(source/name if is_bundle else source, directory/(name if is_bundle else 'rgb_source'+source.suffix.lower()))
        rgb_name = 'rgb.png' if is_bundle else 'rgb_source'+source.suffix.lower()
        files = {path.relative_to(self.root).as_posix(): digest(path) for path in directory.iterdir()}
        kind = ('synthetic_test_pair' if manifest.get('synthetic_test') else 'paired_archive') if is_bundle else 'rgb_only'
        if acquisition:
            if not is_bundle or manifest.get('synthetic_test'):
                raise ValueError('现场采集不能来自RGB-only或合成归档')
            kind = 'live_preview' if group == 'preview' else 'live_rgbd'
        sample = dict(id=sample_id, imported_at=datetime.now().isoformat(), source=str(source),
                      source_kind=kind, source_identity=identity,
                      group=group, note=str(note)[:500], excluded=False, sha256=files,
                      acquisition=copy.deepcopy(acquisition),
                      rgb=(directory/rgb_name).relative_to(self.root).as_posix(),
                      directory=directory.relative_to(self.root).as_posix())
        self.state['samples'].append(sample); self.selected = sample_id
        self.analyze(sample)
        self.event('import', sample_id=sample_id, group=group)
        self.save()
        return sample

    def analyze(self, sample):
        self.verify_files(sample)
        directory = self.root/sample['directory']
        output = directory/('analysis_'+uuid.uuid4().hex[:10]); output.mkdir()
        image = cv2.imread(str(local_file(self.root, sample['rgb'])), cv2.IMREAD_COLOR)
        result = dict(valid=False, reason='', metrics=None, detection=None, pose=None)
        overlay = image.copy() if image is not None else np.full((360, 640, 3), 45, np.uint8)
        views = dict(rgb=sample['rgb'])
        try:
            detection, overlay = detect_board(image, self.state['layout'], self.state['board'], self.state['limits'])
            result['detection'] = detection
            if not detection['valid']:
                raise ValueError(detection['reason'])
            if self.state['layout'] is None:
                self.state['layout'] = detection['layout']
                self.event('layout_matched', layout=self.state['layout'], sample_id=sample['id'])
            if sample['source_kind'] == 'rgb_only':
                raise ValueError('RGB 板布局已核对；缺少配对深度/本帧内参，仅图像预览，不计入 B 指标')
            observation = validate_bundle(directory)
            camera_identity = {k: observation[k] for k in ('camera_serial', 'camera_frame', 'color_intrinsics',
                               'depth_scale_m_per_unit', 'depth_to_color_extrinsics')}
            if self.state['camera_identity'] is not None and self.state['camera_identity'] != camera_identity:
                raise ValueError('相机/profile/工厂标定改变，请另建会话')
            pose = rgb_pose(detection, observation['color_intrinsics'], image.shape, self.state['limits'])
            result['pose'] = pose
            transform = np.array(pose['T_C_Q_row_major'])
            from camera_board import camera_matrix
            k, distortion = camera_matrix(observation['color_intrinsics'], image.shape)
            cv2.drawFrameAxes(overlay, k, distortion, cv2.Rodrigues(transform[:3, :3])[0], transform[:3, 3], .03, 2)
            depth = cv2.imread(str(directory/'depth_aligned.png'), cv2.IMREAD_UNCHANGED)
            if depth is None or depth.shape != image.shape[:2]:
                raise ValueError('原始彩深图像尺寸/编码不匹配')
            metric, heat = depth_metrics(depth, observation['color_intrinsics'], observation['depth_scale_m_per_unit'],
                                        observation['depth_to_color_extrinsics'], detection, pose, self.state['limits'], self.state['board'])
            result.update(valid=True, metrics=metric, observation={key: observation.get(key) for key in
                          ('camera_serial', 'camera_frame', 'sequence', 'request_id', 'color_frame_number',
                           'depth_frame_number', 'color_timestamp_ms', 'depth_timestamp_ms', 'captured_at_ms')})
            self.state['camera_identity'] = camera_identity
            if not cv2.imwrite(str(output/'heatmap.png'), heat):
                raise OSError('无法保存误差图')
            views['heatmap'] = (output/'heatmap.png').relative_to(self.root).as_posix()
            scaled = np.clip(depth.astype(float)*observation['depth_scale_m_per_unit']/self.state['limits']['max_depth_m']*255, 0, 255).astype(np.uint8)
            visual = cv2.applyColorMap(scaled, cv2.COLORMAP_VIRIDIS); visual[depth == 0] = 0
            if not cv2.imwrite(str(output/'depth.png'), visual):
                raise OSError('无法保存深度预览')
            views['depth'] = (output/'depth.png').relative_to(self.root).as_posix()
        except (ValueError, KeyError, TypeError, cv2.error) as error:
            result.update(valid=False, reason=str(error))
        if not cv2.imwrite(str(output/'overlay.png'), overlay):
            raise OSError('无法保存角点预览')
        views['overlay'] = (output/'overlay.png').relative_to(self.root).as_posix()
        write_json(output/'result.json', result)
        sample.update(result=result, views=views)

    def save(self):
        self.state['selected'] = self.selected
        path = self.root/('session_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f')+'.json')
        write_json(path, self.state); self.last_snapshot = str(path)
        return path

    @classmethod
    def load(cls, path):
        path = Path(path).expanduser().resolve(); state = read_json(path)
        if state.get('schema') != SCHEMA or state.get('mode') not in ('offline', 'live') or state.get('applied') is not False:
            raise ValueError('不是实验 B 会话；不接受 A/C 或已应用配置')
        if state.get('board') != BOARD or state.get('limits') != LIMITS:
            raise ValueError('该归档使用不同板参数/计算版本；不能静默替换后复算')
        if not isinstance(state.get('samples'), list) or len(state['samples']) > 2000:
            raise ValueError('会话样本列表无效')
        instance = object.__new__(cls)
        instance.root = path.parent; instance.state = state
        instance.selected = state.get('selected', ''); instance.last_snapshot = str(path)
        ids = set()
        for sample in state['samples']:
            if sample['id'] in ids or sample.get('group') not in ('fit', 'validation', 'preview'):
                raise ValueError('会话样本 ID/用途无效')
            ids.add(sample['id'])
            instance.verify_files(sample)
            for relative in [sample['rgb'], *sample.get('views', {}).values()]:
                local_file(instance.root, relative)
            if not (instance.root/sample['directory']).resolve().is_relative_to(instance.root.resolve()):
                raise ValueError('样本目录越界')
        return instance

    def recompute(self):
        for sample in self.state['samples']:
            self.verify_files(sample)
        for sample in self.state['samples']:
            self.analyze(sample)
        self.event('recompute_all'); return self.save()

    def export(self):
        for sample in self.state['samples']:
            self.verify_files(sample)
        target = self.root/('export_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f')); target.mkdir()
        if (self.root/'samples').exists():
            shutil.copytree(self.root/'samples', target/'samples')
        if self.state.get('trend'):
            shutil.copyfile(local_file(self.root, self.state['trend']), target/self.state['trend'])
        write_json(target/'session.json', {**self.state, 'selected': self.selected})
        keys = ['id', 'group', 'excluded', 'valid', 'source_kind', 'board_center_z_m', 'tilt_deg',
                'reprojection_rms_px', 'signed_mean_mm', 'median_absolute_mm', 'rms_mm',
                'p95_absolute_mm', 'normal_difference_deg', 'valid_fraction', 'reason', 'note']
        with (target/'samples.csv').open('x', newline='', encoding='utf-8-sig') as stream:
            writer = csv.DictWriter(stream, fieldnames=keys); writer.writeheader()
            for sample in self.state['samples']:
                row = {**sample, **sample['result'], **(sample['result']['metrics'] or {})}
                writer.writerow({k: row.get(k, '') for k in keys})
        valid = [s for s in self.state['samples'] if s['result']['valid'] and not s['excluded'] and s['group'] != 'preview']
        lines = ['# 实验 B：RGB/深度一致性', '',
                 f"有效参与统计 {len(valid)} 条；总记录 {len(self.state['samples'])} 条。排除/无效记录保留。",
                 f"其中合成测试 {sum(s['source_kind'] == 'synthetic_test_pair' for s in valid)} 条，不代表实机采样。",
                 f"现场图像 {sum(s['source_kind'] == 'live_rgbd' for s in valid)} 条；预览 {sum(s['group'] == 'preview' for s in self.state['samples'])} 条不参加正式统计。",
                 '板尺寸来源：用户确认的板面标签，9x12、15mm、11.25mm、5x5；未独立测量印刷尺度。',
                 'charuco-200 保留为标签，不推断为字典容量。布局由 RGB marker ID 与空间关系核对。',
                 '全部几何在 RGB optical 系计算；不使用手臂 FK、TCP 或 torso/pelvis 外参。',
                 '正误差表示沿远离相机的 RGB 平面法向偏远。P95 对绝对误差计算。',
                 '深度按 SDK 对齐保留原 depth Z 的定义，经工厂彩深外参和 RGB 射线换算；存在栅格铺展/遮挡近似。',
                 'ROI 内缩、剔除零值及固定范围外深度；不按残差删点。彩深时间差仅做归档核验，不是硬件同步证明。',
                 '现场静止条件是操作者点击确认，不是独立运动传感器证明；历史图像曝光时是否静止未核实。',
                 '现有相机owner只在收到显式快照请求时取帧；本地验请求ID和连续帧进展，不称硬件同步。',
                 '诊断组和验证组显式记录；本版没有拟合/自动应用修正参数。',
                 'RGB PnP 也有误差；低平面残差不证明全部像素配准或绝对精度通过。', '',
                 '| 样本 | 用途 | 距离m | 倾角deg | 均值mm | RMSmm | P95mm |',
                 '|---|---|---:|---:|---:|---:|---:|']
        for s in valid:
            m = s['result']['metrics']
            lines.append(f"| {s['id']} | {s['group']} | {m['board_center_z_m']:.3f} | {m['tilt_deg']:.2f} | {m['signed_mean_mm']:.3f} | {m['rms_mm']:.3f} | {m['p95_absolute_mm']:.3f} |")
        (target/'REPORT.md').write_text('\n'.join(lines)+'\n', encoding='utf-8')
        self.trend(target/'trend.png')
        self.event('export', path=str(target)); self.save()
        return target

    def trend(self, path):
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig, axes = plt.subplots(1, 2, figsize=(10, 3.4), constrained_layout=True)
        for group, color, marker in [('fit', '#167d8d', 'o'), ('validation', '#b33e58', 's')]:
            samples = [s for s in self.state['samples'] if s['group'] == group and s['result']['valid'] and not s['excluded']]
            for ax, key in zip(axes, ['board_center_z_m', 'tilt_deg']):
                ax.scatter([s['result']['metrics'][key] for s in samples],
                           [s['result']['metrics']['rms_mm'] for s in samples], c=color, marker=marker, label=group)
        for ax, label in zip(axes, ['Board center Z (m)', 'Tilt (deg)']):
            ax.set_xlabel(label); ax.set_ylabel('RGB-plane residual RMS (mm)'); ax.grid(alpha=.2); ax.legend()
        fig.savefig(path, dpi=130); plt.close(fig)

    def status(self):
        samples = []
        for sample in self.state['samples']:
            samples.append({key: sample.get(key) for key in ('id', 'group', 'excluded', 'source_kind', 'note', 'result', 'acquisition')})
        selected = self.selected_sample()
        views = {} if selected is None else {k: str(self.root/v) for k, v in selected['views'].items()}
        if self.state.get('trend'):
            views['trend'] = str(local_file(self.root, self.state['trend']))
        return dict(protocol=PROTOCOL, mode=self.state['mode'], session_id=self.state['session_id'],
                    board=self.state['board'], layout=self.state['layout'], selected=self.selected,
                    samples=samples, snapshot=self.last_snapshot,
                    views=views,
                    camera_identity=self.state['camera_identity'])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument('--offline', action='store_true')
    mode.add_argument('--live', action='store_true')
    parser.add_argument('--session', type=Path)
    parser.add_argument('--import-path', type=Path)
    parser.add_argument('--network', default='eth0')
    parser.add_argument('--ssh', default='unitree@192.168.0.249')
    parser.add_argument('--peer', default='192.168.123.164')
    parser.add_argument('--check-only', action='store_true')
    args = parser.parse_args()
    if args.check_only:
        if not args.live:
            parser.error('--check-only 需要 --live；离线模式不访问硬件')
        print(json.dumps(VisionReadOnlyClient(args.network, args.ssh, args.peer).check(), ensure_ascii=False, indent=2))
        return
    if os.environ.get('ROS_DOMAIN_ID') != '96' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        parser.error('B界面/后端必须使用 localhost / ROS_DOMAIN_ID=96；仅RPC子进程访问相机')
    import fcntl
    lock = open(f'/tmp/g1-camera-b-{os.getuid()}.lock', 'a')
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        parser.error('已有B后端运行，请关闭旧B窗口或用 --replace 启动脚本')
    if existing_backends():
        parser.error('已有旧版B后端，请关闭旧B窗口或用 --replace 启动脚本')
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, DurabilityPolicy
    from std_msgs.msg import String
    cv2.setNumThreads(1)
    rclpy.init()

    class Bridge(Node):
        def __init__(self):
            super().__init__('g1_camera_b_live' if args.live else 'g1_camera_b_offline')
            self.runtime_mode = 'live' if args.live else 'offline'
            self.session = CameraBSession.load(args.session) if args.session else CameraBSession(mode=self.runtime_mode)
            self.camera = VisionReadOnlyClient(args.network, args.ssh, args.peer) if args.live else None
            if self.camera:
                self.camera.last_frame = self.session.state.get('last_live_frame')
            self.camera_status = dict(ready=False, message='等待连接检查' if args.live else '离线模式不连接相机')
            self.guard = threading.Lock(); self.busy = False; self.seen = set()
            self.queue = queue.Queue(maxsize=1); self.stop = threading.Event()
            self.message = '正在检查相机连接；不会自动采图' if args.live else '离线就绪：导入历史 RGB 或完整 RGB-D 归档'
            self.cache = self.session.status()
            qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.pub = self.create_publisher(String, '/g1/camera_b/status', qos)
            self.create_subscription(String, '/g1/camera_b/request', self.request, 10)
            self.create_timer(.3, self.publish)
            self.worker = threading.Thread(target=self.work, daemon=False); self.worker.start()
            if args.import_path:
                self.busy = True
                self.queue.put(dict(op='import', path=str(args.import_path), group='fit', note='启动时导入历史数据'))
            elif args.live:
                self.busy = True; self.queue.put(dict(op='check_camera'))

        def publish(self):
            with self.guard:
                status = {**self.cache, 'mode': self.runtime_mode, 'camera': self.camera_status,
                          'stamp': time.time(), 'busy': self.busy, 'message': self.message}
            self.pub.publish(String(data=json.dumps(status, ensure_ascii=False, allow_nan=False)))

        def request(self, msg):
            try:
                request = json.loads(msg.data)
                if request.get('protocol') != PROTOCOL or request.get('op') not in {
                    'new', 'import', 'load', 'save', 'export', 'select', 'exclude', 'group', 'recompute',
                    'check_camera', 'snapshot', 'capture'}:
                    raise ValueError('未知 B 请求；此后端没有手臂/腰/手控制操作')
                if request['op'] in ('check_camera', 'snapshot', 'capture') and not self.camera:
                    raise ValueError('离线模式禁止相机请求')
                if request['op'] == 'capture' and (request.get('stationary_confirmed') is not True or request.get('group') not in ('fit', 'validation')):
                    raise ValueError('正式采样需要明确停稳确认与诊断/验证用途')
                rid = request.get('request_id')
                if not isinstance(rid, str) or not 1 <= len(rid) <= 80:
                    raise ValueError('请求 ID 无效')
                with self.guard:
                    if rid in self.seen or self.busy:
                        return
                    if request.get('session_id') != self.cache['session_id']:
                        raise ValueError('会话已改变，请等待刷新')
                    stamp = float(request.get('stamp', 0))
                    if not np.isfinite(stamp) or abs(time.time()-stamp) > 5:
                        raise ValueError('请求已过期，不自动重试')
                    self.seen.add(rid)
                    if len(self.seen) > 10000:
                        raise ValueError('本次进程请求过多，请保存后重开')
                    self.busy = True; self.message = '正在处理：'+request['op']
                    self.queue.put_nowait(request)
            except (ValueError, TypeError, queue.Full) as error:
                with self.guard:
                    self.message = str(error)

        def work(self):
            while not self.stop.is_set():
                try:
                    req = self.queue.get(timeout=.2)
                except queue.Empty:
                    continue
                try:
                    op = req['op']; message = '完成'
                    if op == 'new':
                        self.session.save(); self.session = CameraBSession(mode=self.runtime_mode)
                        if self.camera:
                            self.camera.last_frame = None
                    elif op == 'load':
                        loaded = CameraBSession.load(req['path']); self.session.save(); self.session = loaded
                        if self.camera:
                            self.camera.last_frame = loaded.state.get('last_live_frame')
                    elif op == 'check_camera':
                        checked = self.camera.check()
                        with self.guard:
                            self.camera_status = checked
                        message = checked['message']
                    elif op in ('snapshot', 'capture'):
                        if not self.camera:
                            raise ValueError('离线模式禁止采图')
                        def progress(text):
                            with self.guard:
                                self.message = text
                        directory = ROOT/'data/camera_b/captures'/uuid.uuid4().hex
                        acquired = self.camera.capture(directory, progress)
                        acquired['stationary_confirmed'] = req.get('stationary_confirmed', False)
                        group = 'preview' if op == 'snapshot' else req['group']
                        sample = self.session.import_path(directory, group=group, note=req.get('note', ''), acquisition=acquired)
                        self.session.state['last_live_frame'] = self.camera.last_frame
                        self.session.state['mode'] = 'live'; self.session.save()
                        with self.guard:
                            self.camera_status = dict(ready=True, checked_at=time.time(), network=args.network, host=args.ssh,
                                                      message='最近一次图像请求成功')
                        message = ('快照已更新（不参加正式统计）' if op == 'snapshot' else '本次样本已保存')
                        if not sample['result']['valid']:
                            message += '；'+sample['result']['reason']
                    elif op == 'import':
                        sample = self.session.import_path(req['path'], req.get('group', 'fit'), req.get('note', ''))
                        message = '导入完成：'+('RGB-D 指标可用' if sample['result']['valid'] else sample['result']['reason'])
                    elif op == 'save':
                        message = '已保存：'+str(self.session.save())
                    elif op == 'export':
                        message = '已导出：'+str(self.session.export())
                    elif op == 'recompute':
                        message = '原始数据复算完成：'+str(self.session.recompute())
                    elif op in ('select', 'exclude', 'group'):
                        sample = next((s for s in self.session.state['samples'] if s['id'] == req.get('sample_id')), None)
                        if sample is None:
                            raise ValueError('未选择有效样本 ID')
                        self.session.selected = sample['id']
                        if op == 'exclude':
                            sample['excluded'] = not sample['excluded']
                        elif op == 'group':
                            if sample['group'] == 'preview':
                                raise ValueError('预览不能转成正式样本；停稳后重新点击采样')
                            sample['group'] = 'validation' if sample['group'] == 'fit' else 'fit'
                        if op != 'select':
                            self.session.event(op, sample_id=sample['id'], excluded=sample['excluded'], group=sample['group'])
                            self.session.save()
                    if op in ('import', 'capture', 'exclude', 'group', 'recompute'):
                        trend = 'trend_'+uuid.uuid4().hex[:10]+'.png'
                        self.session.trend(self.session.root/trend)
                        self.session.state['trend'] = trend
                        self.session.save()
                    cache = self.session.status()
                except Exception as error:
                    message = '未完成：'+str(error); cache = self.session.status()
                    if req['op'] in ('check_camera', 'snapshot', 'capture'):
                        with self.guard:
                            self.camera_status = dict(ready=False, checked_at=time.time(), message=message)
                        self.session.event('camera_request_failed', operation=req['op'], reason=str(error))
                        self.session.save()
                        message += '；未把旧图当作本次结果，当前显示可能仍为上一条记录'
                        cache = self.session.status()
                    self.get_logger().warning(message)
                finally:
                    with self.guard:
                        self.cache = {**cache, 'last_request_id': req.get('request_id', '')}
                        self.message = message; self.busy = False
                    self.queue.task_done()

    node = Bridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.stop.set(); node.worker.join()
        node.session.save(); node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
