"""RGB-D units, alignment, archive identity and ROS packing regression tests."""
import copy
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import cv2
import numpy as np
from scipy.spatial.transform import Rotation

from vision_rgbd import (aligned_cloud, validate_pair, import_record, import_camera_record, ROOT,
                        validate_capture_result, measured_scene_geometry)
from vision_task_geometry import build_task_preview
from vision_task_preview import load_cloud
from camera_board import (BOARD, LIMITS, board_for_layout, detect_board, rgb_pose,
                          aligned_depth_points, depth_metrics)
from camera_b_node import CameraBSession, VisionReadOnlyClient, write_json, digest, validate_bundle


class RGBDTests(unittest.TestCase):
    def test_no_target_does_not_block_scene_only_but_never_authorizes_task(self):
        capture=dict(status_result=0,status=dict(camera_ready=True,model_ready=True),
                     request_id=4,observation_result=102,
                     observation=dict(code=102,request_id=4,capture_result='/capture/result.json',
                                      message='target not detected',sequence=7))
        for kind in ('prepare','recover'):
            self.assertEqual(validate_capture_result(capture,kind),capture['observation'])
        with self.assertRaisesRegex(RuntimeError,'102'):validate_capture_result(capture,'task')
        manifest=dict(source_sequence=7,source_request_id=4,
                      torso_from_color_optical_row_major=np.eye(4).tolist())
        geometry=measured_scene_geometry(capture,manifest)
        self.assertFalse(geometry['task_target_available'])
        self.assertNotIn('task_preview',geometry)
        capture['observation']['request_id']=5
        with self.assertRaisesRegex(ValueError,'mismatched'):validate_capture_result(capture,'prepare')

    def test_scene_only_still_rejects_communication_and_camera_failure(self):
        with self.assertRaisesRegex(TimeoutError,'not started'):
            validate_capture_result(dict(status_result=3104),'prepare')
        with self.assertRaisesRegex(RuntimeError,'not ready'):
            validate_capture_result(dict(status_result=0,status=dict(camera_ready=False)),'prepare')
        capture=dict(status_result=0,status=dict(camera_ready=True,model_ready=True),
                     observation_result=200,observation=dict(message='camera read failed'))
        with self.assertRaisesRegex(RuntimeError,'200'):validate_capture_result(capture,'prepare')

    def setUp(self):
        self.intrinsics = dict(width=3, height=2, fx=2., fy=2., cx=1., cy=0., distortion=[0.]*5)
        self.depth = np.array([[1000, 1000, 0], [500, 3000, 1000]], dtype=np.uint16)
        self.color = np.zeros((2, 3, 3), np.uint8)
        self.color[0, 0] = [10, 20, 30]

    def test_projection_units_invalid_depth_and_rgb_order(self):
        xyz, rgb, uv = aligned_cloud(self.depth, self.color, self.intrinsics, .001, np.eye(4), stride=1)
        np.testing.assert_allclose(xyz, [[-.5, 0, 1], [0, 0, 1], [-.25, .25, .5], [.5, .5, 1]])
        np.testing.assert_array_equal(uv, [[0, 0], [1, 0], [0, 1], [2, 1]])
        np.testing.assert_array_equal(rgb[0], [30, 20, 10])

    def test_full_rigid_transform_not_plane_projection(self):
        transform = np.eye(4)
        transform[:3, :3] = Rotation.from_euler('xyz', [.3, -.5, .2]).as_matrix()
        transform[:3, 3] = [.1, .2, .3]
        base, _, _ = aligned_cloud(self.depth, self.color, self.intrinsics, .001, np.eye(4), stride=1)
        actual, _, _ = aligned_cloud(self.depth, self.color, self.intrinsics, .001, transform, stride=1)
        np.testing.assert_allclose(actual, base @ transform[:3, :3].T + transform[:3, 3], atol=1e-7)
        self.assertGreater(float(np.ptp(actual[:, 2])), .1)

    def test_invalid_encoding_calibration_and_limits(self):
        cases = [dict(depth=self.depth.astype(np.uint8)), dict(bgr=self.color[:1]),
                 dict(depth_scale=0.), dict(stride=0), dict(stride=1.5),
                 dict(intrinsics={**self.intrinsics, 'distortion': [1., 0, 0, 0, 0]}),
                 dict(intrinsics={**self.intrinsics, 'fx': 0}), dict(transform=np.zeros((4, 4)))]
        for override in cases:
            args = dict(depth=self.depth, bgr=self.color, intrinsics=self.intrinsics,
                        depth_scale=.001, transform=np.eye(4))
            args.update(override)
            with self.subTest(override=override), self.assertRaises(ValueError):
                aligned_cloud(**args)

    def test_pair_rejects_different_frame(self):
        capture = json.loads((ROOT/'logs/vision_live_20260909_104045/capture.json').read_text())
        observation = capture['observation']
        validate_pair(observation, observation)
        for key in ('sequence', 'depth_frame_number', 'camera_serial', 'color_timestamp_ms'):
            record = copy.deepcopy(observation)
            record[key] = 'different'
            with self.subTest(key=key), self.assertRaises(ValueError):
                validate_pair(observation, record)

    def test_pair_allows_only_serialization_roundoff(self):
        observation = json.loads((ROOT/'logs/vision_live_20260909_104045/capture.json').read_text())['observation']
        record = copy.deepcopy(observation)
        record['depth_timestamp_ms'] = float(np.nextafter(record['depth_timestamp_ms'], np.inf))
        record['depth_to_color_extrinsics']['rotation'][0] += 1e-16
        validate_pair(observation, record)
        record['depth_timestamp_ms'] += .01
        with self.assertRaisesRegex(ValueError, 'depth_timestamp_ms'):
            validate_pair(observation, record)
        record = copy.deepcopy(observation)
        record['depth_to_color_extrinsics']['translation_m'][0] += 1e-6
        with self.assertRaisesRegex(ValueError, 'depth_to_color_extrinsics'):
            validate_pair(observation, record)
        record = copy.deepcopy(observation)
        record['color_intrinsics']['fx'] += .01
        with self.assertRaisesRegex(ValueError, 'color_intrinsics'):
            validate_pair(observation, record)

    def test_bundle_import_cloud_packing_identity_and_no_overwrite(self):
        # Synthetic images are confined to a temporary directory, never shown as a live capture.
        capture = json.loads((ROOT/'logs/vision_live_20260909_104045/capture.json').read_text())
        capture['observation']['capture_result'] = '/service/captures/test_result.json'
        observation = capture['observation']
        with tempfile.TemporaryDirectory() as tmp:
            directory = Path(tmp)
            source = directory/'source';source.mkdir()
            destination = directory/'archive';destination.mkdir()
            capture_path = destination/'capture.json'
            capture_path.write_text(json.dumps(capture))
            record = copy.deepcopy(observation)
            record['capture'] = dict(id='test', color_image='/service/captures/test_color.png',
                                     depth_image='/service/captures/test_depth.png',
                                     annotated_image='/service/captures/test_annotated.jpg')
            record_path = source/'test_result.json';record_path.write_text(json.dumps(record))
            depth = np.full((720, 1280), 600, np.uint16)
            color = np.full((720, 1280, 3), [10, 20, 30], np.uint8)
            cv2.imwrite(str(source/'test_color.png'), color)
            cv2.imwrite(str(source/'test_depth.png'), depth)
            cv2.imwrite(str(source/'test_annotated.jpg'), color)
            manifest = import_record(capture_path, server_record=record_path)
            self.assertGreater(manifest['point_count'], 100000)
            self.assertFalse(manifest['registration_to_cad_applied'])
            geometry = build_task_preview(capture, '/home/syf/unitree_ws/head_camera_intrinsics.json',
                                         ROOT/'assets/workbench/rh56dfx/index_tcp_estimates.json')
            cloud = load_cloud(destination, geometry)
            self.assertEqual(len(cloud.data), cloud.width*16)
            self.assertEqual(cloud.header.frame_id, 'torso_link')
            first = np.frombuffer(bytes(cloud.data[:16]), dtype=[('xyz', '<f4', 3), ('rgb', '<u4')])
            self.assertEqual(first['rgb'][0], 30 << 16 | 20 << 8 | 10)
            with self.assertRaises(FileExistsError):
                import_record(capture_path, server_record=record_path)
            geometry['source']['sequence'] += 1
            with self.assertRaises(ValueError):
                load_cloud(destination, geometry)


class CameraBTests(unittest.TestCase):
    def test_live_child_environment_and_frame_freshness(self):
        client = VisionReadOnlyClient()
        with patch.dict('os.environ', {'ROS_DOMAIN_ID': '96', 'ROS_LOCALHOST_ONLY': '1'}):
            env = client.environment()
            self.assertEqual(env['ROS_DOMAIN_ID'], '0')
            self.assertNotIn('ROS_LOCALHOST_ONLY', env)
            self.assertIn('eth0', env['CYCLONEDDS_URI'])
            import os
            self.assertEqual(os.environ['ROS_DOMAIN_ID'], '96')
        frame = dict(camera_serial='test', color_frame_number=1, depth_frame_number=2,
                     color_timestamp_ms=10., depth_timestamp_ms=11.)
        client.check_new_frame(frame)
        client.last_frame = frame
        with self.assertRaisesRegex(ValueError, '未前进'):
            client.check_new_frame(frame)
        with self.assertRaisesRegex(ValueError, '序列号'):
            client.check_new_frame({**frame, 'camera_serial': 'other'})
        client.check_new_frame({k: v+1 if k != 'camera_serial' else v for k, v in frame.items()})

    def test_camera_only_import_no_pedal_and_no_torso(self):
        capture = json.loads((ROOT/'logs/vision_live_20260909_104045/capture.json').read_text())
        capture['observation_result'] = 102
        capture['observation']['code'] = 102
        capture['observation']['capture_result'] = '/service/test_result.json'
        record = copy.deepcopy(capture['observation'])
        record['capture'] = dict(id='test', color_image='/service/test_color.png', depth_image='/service/test_depth.png')
        with tempfile.TemporaryDirectory() as tmp:
            base = Path(tmp); src = base/'source'; src.mkdir(); dest = base/'dest'; dest.mkdir()
            write_json(src/'test_result.json', record); write_json(dest/'capture.json', capture)
            cv2.imwrite(str(src/'test_color.png'), np.zeros((720,1280,3), np.uint8))
            cv2.imwrite(str(src/'test_depth.png'), np.full((720,1280),600,np.uint16))
            with patch('vision_rgbd.torso_color_transform', side_effect=AssertionError('No FK in B')):
                manifest = import_camera_record(dest/'capture.json', server_record=src/'test_result.json')
            self.assertEqual(manifest['frame_id'], 'head_color_camera_optical_frame')
            self.assertFalse((dest/'cloud.npz').exists())
            validate_bundle(dest)
            (dest/'rgb.png').write_bytes(b'corrupt')
            with self.assertRaises(ValueError):
                CameraBSession(base/'session').import_path(dest)

    def test_live_preview_portable_replay(self):
        with tempfile.TemporaryDirectory() as tmp:
            base = Path(tmp); bundle = ROOT/'data/vision_captures/20260909_rgbd_001'
            session = CameraBSession(base/'session', mode='live')
            sample = session.import_path(bundle, group='preview', acquisition={'stationary_confirmed':False, 'test_only':True})
            self.assertEqual(sample['group'], 'preview')
            loaded = CameraBSession.load(session.save())
            self.assertEqual(loaded.state['samples'][0]['acquisition']['stationary_confirmed'], False)
            exported = session.export()
            report = (exported/'REPORT.md').read_text()
            self.assertIn('预览 1 条不参加正式统计', report)

    @staticmethod
    def synthetic_bundle(directory, depth_factor=1., parity=1):
        directory = Path(directory); directory.mkdir(parents=True)
        layout = dict(cols=12, rows=9, parity=parity)
        board, _, _ = board_for_layout(layout)
        template_rows = board.getChessboardSize()[1]
        texture = board.draw((1200, template_rows*100))[:900]
        intr = dict(width=960, height=720, fx=950., fy=950., cx=480., cy=360., distortion=[0.]*5)
        k = np.array([[950., 0, 480], [0, 950., 360], [0, 0, 1]])
        rotation = Rotation.from_euler('xyz', [.25, -.3, .05]).as_matrix()
        translation = np.array([-.09, -.0675, .55])
        corners = np.array([[0, 0, 0], [.18, 0, 0], [.18, .135, 0], [0, .135, 0]])
        projected = cv2.projectPoints(corners, cv2.Rodrigues(rotation)[0], translation, k, np.zeros(5))[0].reshape(-1, 2)
        homography = cv2.getPerspectiveTransform(np.array([[0, 0], [1200, 0], [1200, 900], [0, 900]], np.float32), projected.astype(np.float32))
        gray = cv2.warpPerspective(texture, homography, (960, 720), borderValue=185)
        image = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        v, u = np.mgrid[:720, :960]
        rays = np.stack(((u-480)/950., (v-360)/950., np.ones_like(u)), axis=-1)
        normal = rotation[:, 2]
        color_xyz = rays*((normal @ translation)/(rays @ normal))[..., None]
        ext_r = Rotation.from_euler('xyz', [.015, -.008, .004]).as_matrix()
        ext_t = np.array([.014, -.001, .003])
        depth_xyz = (color_xyz-ext_t) @ ext_r
        scale = .0001
        depth = np.rint(depth_xyz[..., 2]/scale*depth_factor).astype(np.uint16)
        cv2.imwrite(str(directory/'rgb.png'), image); cv2.imwrite(str(directory/'depth_aligned.png'), depth)
        observation = dict(code=102, request_id=44, sequence=101, camera_serial='SYNTHETIC_TEST',
                           camera_frame='head_color_camera_optical_frame', units='m',
                           image_width=960, image_height=720, color_frame_number=101, depth_frame_number=102,
                           color_timestamp_ms=1000., depth_timestamp_ms=1002., timestamp_delta_ms=2.,
                           captured_at_ms=1500., color_intrinsics=intr, depth_scale_m_per_unit=scale,
                           depth_to_color_extrinsics=dict(rotation=ext_r.reshape(-1, order='F').tolist(), translation_m=ext_t.tolist()))
        write_json(directory/'server_result.json', observation)
        write_json(directory/'capture.json', dict(status_result=0, observation_result=102, request_id=44, observation=observation))
        write_json(directory/'manifest.json', dict(synthetic_test=True,
            sha256={name:digest(directory/name) for name in ['rgb.png', 'depth_aligned.png', 'capture.json', 'server_result.json']}))
        return directory

    def test_b_board_parities_and_dimensions_with_opencv_interpolator(self):
        for cols, rows in ((9, 12), (12, 9)):
            for parity in (0, 1):
                layout = dict(cols=cols, rows=rows, parity=parity)
                board, _, _ = board_for_layout(layout)
                image = board.draw((cols*60, board.getChessboardSize()[1]*60))[:rows*60]
                image = cv2.copyMakeBorder(image, 30, 30, 30, 30, cv2.BORDER_CONSTANT, value=255)
                result, _ = detect_board(cv2.cvtColor(image, cv2.COLOR_GRAY2BGR))
                with self.subTest(layout=layout):
                    self.assertTrue(result['valid'], result['reason'])
                    self.assertEqual(result['layout'], layout)
                    self.assertGreater(result['corner_count'], 50)

    def test_b_real_historical_board_not_just_pedal(self):
        image = cv2.imread(str(ROOT/'data/vision_captures/20260909_rgbd_001/rgb.png'))
        result, _ = detect_board(image)
        self.assertTrue(result['valid'], result['reason'])
        self.assertEqual(result['layout'], dict(cols=12, rows=9, parity=1))
        self.assertGreaterEqual(result['corner_count'], LIMITS['min_corners'])

    def test_b_depth_z_conversion_with_rotation_and_translation(self):
        intr = dict(fx=800., fy=800., cx=320., cy=240.)
        uv = np.array([[200, 180], [500, 300], [320, 240]])
        rotation = Rotation.from_euler('xyz', [.08, -.06, .02]).as_matrix()
        translation = np.array([.02, -.004, .01])
        ext = dict(rotation=rotation.reshape(-1, order='F').tolist(), translation_m=translation.tolist())
        expected = np.column_stack(((uv[:, 0]-320)/800, (uv[:, 1]-240)/800, np.ones(3)))*np.array([.5, .8, 1.])[:, None]
        z_depth = ((expected-translation) @ rotation)[:, 2]
        actual = aligned_depth_points(uv, z_depth, intr, ext)
        np.testing.assert_allclose(actual, expected, atol=1e-12)
        self.assertGreater(np.max(abs(z_depth-expected[:, 2])), .005)

    def test_b_paired_no_pedal_scale_sensitivity_and_portable_recompute(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            source = self.synthetic_bundle(root/'source')
            validate_bundle(source)
            session = CameraBSession(root/'session')
            first = session.import_path(source, group='validation', note='synthetic geometry only')
            self.assertTrue(first['result']['valid'], first['result']['reason'])
            self.assertEqual(first['source_kind'], 'synthetic_test_pair')
            self.assertLess(first['result']['metrics']['rms_mm'], 1.5)
            transform = np.asarray(first['result']['pose']['T_C_Q_row_major'])
            np.testing.assert_allclose(transform[:3, 3], [-.09, -.0675, .55], atol=.002)
            expected_rotation = Rotation.from_euler('xyz', [.25, -.3, .05]).as_matrix()
            self.assertLess(Rotation.from_matrix(transform[:3, :3] @ expected_rotation.T).magnitude(), .02)
            changed = self.synthetic_bundle(root/'scaled', 1.02)
            second = session.import_path(changed)
            self.assertTrue(second['result']['valid'], second['result']['reason'])
            self.assertGreater(second['result']['metrics']['signed_mean_mm']-first['result']['metrics']['signed_mean_mm'], 8.)
            saved = session.save(); before = copy.deepcopy([s['result']['metrics'] for s in session.state['samples']])
            loaded = CameraBSession.load(saved); loaded.recompute()
            for sample, original in zip(loaded.state['samples'], before):
                self.assertAlmostEqual(sample['result']['metrics']['rms_mm'], original['rms_mm'], places=6)
            self.assertEqual(loaded.state['samples'][0]['group'], 'validation')
            output = loaded.export(); portable = CameraBSession.load(output/'session.json'); portable.recompute()
            self.assertEqual(len(portable.state['samples']), 2)
            self.assertFalse(portable.state['applied'])
            with self.assertRaisesRegex(ValueError, '已导入'):
                session.import_path(source)
            (root/'session'/first['rgb']).write_bytes(b'changed')
            with self.assertRaisesRegex(ValueError, 'SHA256'):
                CameraBSession.load(saved)

    def test_b_rgb_only_and_incomplete_archive_do_not_fabricate_depth(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp); source = self.synthetic_bundle(root/'source')
            session = CameraBSession(root/'session')
            sample = session.import_path(source/'rgb.png')
            self.assertTrue(sample['result']['detection']['valid'])
            self.assertFalse(sample['result']['valid'])
            self.assertIsNone(sample['result']['metrics'])
            (source/'depth_aligned.png').write_bytes(b'tampered')
            with self.assertRaisesRegex(ValueError, 'manifest'):
                session.import_path(source)

    def test_b_pair_and_depth_invalidity(self):
        with tempfile.TemporaryDirectory() as tmp:
            source = self.synthetic_bundle(Path(tmp)/'source')
            observation = validate_bundle(source)
            image = cv2.imread(str(source/'rgb.png')); detection, _ = detect_board(image)
            pose = rgb_pose(detection, observation['color_intrinsics'], image.shape)
            with self.assertRaisesRegex(ValueError, '有效深度不足'):
                depth_metrics(np.zeros(image.shape[:2], np.uint16), observation['color_intrinsics'],
                              observation['depth_scale_m_per_unit'], observation['depth_to_color_extrinsics'], detection, pose)
            capture = json.loads((source/'capture.json').read_text()); capture['request_id'] += 1
            (source/'capture.json').write_text(json.dumps(capture))
            with self.assertRaisesRegex(ValueError, 'request_id'):
                validate_bundle(source)
            with self.assertRaisesRegex(ValueError, 'RGB 位姿|RGB 板|PnP'):
                rgb_pose({**detection, 'valid':False}, observation['color_intrinsics'], image.shape)


if __name__ == '__main__':
    unittest.main()
