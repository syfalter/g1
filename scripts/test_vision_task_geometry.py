import json
from pathlib import Path
import sys
import unittest

import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

from estimate_index_tcp import CONNECTOR_LENGTH, generate
from vision_task_geometry import (
    EXPECTED_CAMERA_FRAME,
    build_task_preview,
    depth_to_color_transform,
    torso_color_transform,
    ray_plane_intersection,
    with_radial_offset,
)
from task_straight_policy import RADIAL_OFFSET_M, HEIGHT_OFFSET_M


INTRINSICS = Path("/home/syf/unitree_ws/head_camera_intrinsics.json")
FINGERTIPS = ROOT / "assets/workbench/rh56dfx/index_tcp_estimates.json"


class VisionTaskGeometryTest(unittest.TestCase):
    def test_abc_target_cloud_consistency_and_identity_switches(self):
        from task_calibration import DEFAULTS,cloud_points,camera_transform,depth_points
        off=dict(DEFAULTS,a_enabled=False,b_enabled=False,c_enabled=False)
        original=build_task_preview(self.capture(),INTRINSICS,FINGERTIPS)
        disabled=build_task_preview(self.capture(),INTRINSICS,FINGERTIPS,calibration=off)
        self.assertEqual(original['task_preview'],disabled['task_preview'])
        self.assertEqual(original['fingertip_estimates'],disabled['fingertip_estimates'])
        values=dict(DEFAULTS,depth_bias_mm=2.,depth_scale=1.001)
        changed=build_task_preview(self.capture(),INTRINSICS,FINGERTIPS,calibration=values)
        old=np.array(original['points_torso_raw_m']['black_center_point'])
        np.testing.assert_allclose(cloud_points(old[None,:],changed)[0],changed['task_preview']['axis_center_m'],atol=1e-12)
        np.testing.assert_allclose(changed['fingertip_estimates']['sides']['left']['states']['open']['wrist_yaw_from_index_tcp']['xyz_m'],
                                   [DEFAULTS[f'left_{a}_mm']/1000 for a in 'xyz'])
        self.assertTrue(changed['calibration']['experimental'])

    def test_calibration_rejects_nonfinite_and_roundtrips(self):
        from task_calibration import DEFAULTS,validate,save,load
        from tempfile import TemporaryDirectory
        for value in (float('nan'),float('inf'),100):
            with self.assertRaises(ValueError):validate(dict(DEFAULTS,depth_scale=value))
        with TemporaryDirectory() as temp:
            path=Path(temp)/'calibration.json';save(DEFAULTS,path)
            self.assertEqual(load(path),DEFAULTS)

    def test_shift_arc_pivot_preserves_entry_preinsert_and_measured_base(self):
        before=build_task_preview(self.capture(),INTRINSICS,FINGERTIPS,height_offset_m=.025)
        after=build_task_preview(self.capture(),INTRINSICS,FINGERTIPS,height_offset_m=.025,
                                 arc_center_toward_robot_m=.02)
        old,new=before['task_preview'],after['task_preview']
        for key in ('axis_center_m','insertion_reference_m','preinsert_point_m','ray_plane_intersections_m'):
            self.assertEqual(old[key],new[key])
        self.assertEqual(before['grasp_candidates'],after['grasp_candidates'])
        np.testing.assert_allclose(np.array(new['arc_center_m'])-old['arc_center_m'],[-.02,0,0],atol=1e-12)
        self.assertAlmostEqual(new['arc_radius_m'],old['arc_radius_m']+.02)
        np.testing.assert_allclose(new['arc_minus120_to_plus120_m'][60],new['insertion_reference_m'],atol=1e-12)
        for i,angle in ((0,-120),(120,120)):
            center=np.array(new['arc_center_m']);entry=np.array(new['insertion_reference_m'])
            expected=center+Rotation.from_euler('z',angle,degrees=True).apply(entry-center)
            np.testing.assert_allclose(new['arc_minus120_to_plus120_m'][i],expected,atol=1e-12)
        rebuilt=with_radial_offset(after,0.)
        self.assertEqual(rebuilt,after)
        restored=with_radial_offset(after,0.,arc_center_toward_robot_m=0.)
        self.assertEqual(restored,before)

    def test_reject_invalid_arc_shift(self):
        for value in (-.001,.031,float('nan')):
            with self.assertRaises(ValueError):
                build_task_preview(self.capture(),INTRINSICS,FINGERTIPS,arc_center_toward_robot_m=value)

    def capture(self, plane_delta=0.0, diagonal_delta=0.0, entry_radius=.035):
        profile = json.loads(INTRINSICS.read_text())["profiles"]["color_1280x720_30fps"]
        rotation = Rotation.from_euler("xyz", [0.04, -0.03, 0.02]).as_matrix()
        extrinsics = {
            "rotation": rotation.reshape(9, order="F").tolist(),
            "translation_m": [0.0148, -0.0002, 0.0004],
        }
        torso_from_color = torso_color_transform(extrinsics)
        color_from_torso = np.linalg.inv(torso_from_color)
        torso_points = {
            "black_center_point": np.array([0.42, 0.04, -0.08]),
            "operation_diagonal_intersection": np.array([0.42+entry_radius, 0.04, -0.08 + diagonal_delta]),
            "operation_point": np.array([0.47, 0.04, -0.08 + plane_delta]),
        }
        camera_points = {
            key: (color_from_torso @ np.r_[point, 1.0])[:3]
            for key, point in torso_points.items()
        }
        pixels = {
            key: [
                round(profile["fx"] * point[0] / point[2] + profile["cx"], 6),
                round(profile["fy"] * point[1] / point[2] + profile["cy"], 6),
            ]
            for key, point in camera_points.items()
        }
        observation = {
            "code": 0,
            "valid_3d": True,
            "camera_frame": EXPECTED_CAMERA_FRAME,
            "units": "m",
            "camera_name": "Intel RealSense D435I",
            "camera_serial": "243322073672",
            "image_width": 1280,
            "image_height": 720,
            "color_intrinsics": {
                "fx": profile["fx"], "fy": profile["fy"],
                "cx": profile["cx"], "cy": profile["cy"],
                "width": 1280, "height": 720,
                "distortion_model": "inverse_brown_conrady",
                "distortion": [0.0] * 5,
            },
            "depth_to_color_extrinsics": extrinsics,
            "points_camera": {key: value.tolist() for key, value in camera_points.items()},
            "depth_scale_m_per_unit": 0.001,
            "stage1_confidence": 0.9,
            "stage2_confidence": 0.9,
            "timestamp_delta_ms": 0.2,
            "invalid_depth_keys": [],
            **pixels,
        }
        return {"status_result": 0, "observation_result": 0, "observation": observation}

    def test_column_major_extrinsic_is_inverted_for_color_points(self):
        capture = self.capture()
        artifact = build_task_preview(capture, INTRINSICS, FINGERTIPS)
        np.testing.assert_allclose(
            artifact["task_preview"]["axis_center_m"], [0.42, 0.04, -0.08], atol=1e-9
        )
        np.testing.assert_allclose(
            artifact["task_preview"]["operation_point_projected_m"],
            [0.47, 0.04, -0.08], atol=1e-9,
        )
        self.assertTrue(artifact["geometry_valid_for_planning_seed"])
        decoded = depth_to_color_transform(capture["observation"]["depth_to_color_extrinsics"])
        self.assertFalse(np.allclose(decoded[:3, :3], decoded[:3, :3].T))

    def test_large_plane_mismatch_is_visible_and_invalid(self):
        artifact = build_task_preview(self.capture(0.05, 0.05), INTRINSICS, FINGERTIPS)
        self.assertFalse(artifact["geometry_valid_for_planning_seed"])
        self.assertAlmostEqual(artifact["quality"]["operation_plane_delta_m"], 0.05)
        self.assertNotEqual(
            artifact["task_preview"]["operation_point_raw_m"],
            artifact["task_preview"]["operation_point_projected_m"],
        )

    def test_rejects_wrong_frame_and_missing_point(self):
        wrong = self.capture()
        wrong["observation"]["camera_frame"] = "head_depth_camera_optical_frame"
        with self.assertRaisesRegex(ValueError, "Unexpected camera frame"):
            build_task_preview(wrong, INTRINSICS, FINGERTIPS)
        missing = self.capture()
        missing["observation"]["points_camera"]["operation_point"] = []
        with self.assertRaisesRegex(ValueError, "operation_point"):
            build_task_preview(missing, INTRINSICS, FINGERTIPS)

    def test_rejects_timeout_wrapper(self):
        with self.assertRaisesRegex(ValueError, "status failed: 3104"):
            build_task_preview(
                {"status_result": 3104, "observation_result": -1, "observation": None},
                INTRINSICS,
                FINGERTIPS,
            )

    def test_rejects_invalid_depth_scale(self):
        capture = self.capture()
        capture["observation"]["depth_scale_m_per_unit"] = 0.0
        with self.assertRaisesRegex(ValueError, "Invalid depth scale"):
            build_task_preview(capture, INTRINSICS, FINGERTIPS)

    def test_candidate_wrist_and_palm_place_model_tip_at_entry(self):
        data = build_task_preview(self.capture(), INTRINSICS, FINGERTIPS)
        for side, candidate in data["grasp_candidates"].items():
            self.assertTrue(candidate["preview_only"])
            self.assertFalse(candidate["ik_checked"])
            self.assertFalse(candidate["collision_checked"])
            poses = candidate["poses"]
            tip = np.asarray(poses["index_tcp"]["matrix_row_major"])
            estimate = data["fingertip_estimates"]["sides"][side]["states"]["open"]
            for name, offset in (("wrist_yaw", "wrist_yaw_from_index_tcp"),
                                 ("palm", "palm_from_index_tcp")):
                np.testing.assert_allclose(
                    np.asarray(poses[name]["matrix_row_major"]) @
                    np.asarray(estimate[offset]["matrix_row_major"]), tip, atol=1e-12)
            np.testing.assert_allclose(tip[:3, 3], data["task_preview"]["insertion_reference_m"])
            pre = np.asarray(poses["preinsert_index_tcp"]["matrix_row_major"])
            self.assertAlmostEqual(pre[2, 3] - tip[2, 3], .10)
            self.assertLess(pre[0, 3], tip[0, 3])

    def test_low_confidence_retains_points_but_requires_review(self):
        capture = self.capture()
        capture["observation"]["stage2_confidence"] = .01
        data = build_task_preview(capture, INTRINSICS, FINGERTIPS)
        self.assertIn("stage2_confidence=0.0100", " ".join(data["review_reasons"]))
        self.assertTrue(data["preview_only"])
        self.assertFalse(data["hardware_execution_validated"])
        self.assertIn("operation_point", data["points_torso_raw_m"])

    def test_invalid_plane_does_not_create_grasp_candidates(self):
        data = build_task_preview(self.capture(.05, .05), INTRINSICS, FINGERTIPS)
        self.assertEqual(data["grasp_candidates"], {})

    def test_default_uses_pre_offset_point_and_retains_legacy_reference(self):
        data = build_task_preview(self.capture(), INTRINSICS, FINGERTIPS)
        task = data['task_preview']
        self.assertEqual(task['entry_source_key'], 'operation_diagonal_intersection')
        self.assertTrue(data['assumptions']['upstream_radial_offset_bypassed'])
        np.testing.assert_allclose(task['insertion_reference_m'], [.455,.04,-.08], atol=1e-10)
        np.testing.assert_allclose(task['operation_point_projected_m'], [.47,.04,-.08], atol=1e-10)
        np.testing.assert_allclose(task['preinsert_point_m'], [.455,.04,.02], atol=1e-10)
        self.assertAlmostEqual(data['quality']['entry_radius_m'], .035)
        for point in task['arc_minus120_to_plus120_m']:
            delta = np.array(point)-task['axis_center_m']
            self.assertAlmostEqual(np.linalg.norm(delta), .035)
            self.assertAlmostEqual(delta[2], 0.)

    def test_ray_intersection_preserves_ray_and_moves_xy(self):
        origin = np.array([0.,0.,1.])
        point = np.array([.8,.4,-1.])
        hit = ray_plane_intersection(origin,point,[0,0,0],[0,0,2])
        np.testing.assert_allclose(hit,[.4,.2,0.],atol=1e-12)
        np.testing.assert_allclose(np.cross(hit-origin,point-origin),0.,atol=1e-12)
        shifted_depth = origin+3*(point-origin)
        np.testing.assert_allclose(ray_plane_intersection(origin,shifted_depth,[0,0,0],[0,0,1]),hit)

    def test_ray_intersection_rejects_degenerate_parallel_and_backward(self):
        for point,normal in (([1,0,1],[0,0,1]),([1,0,1.001],[0,0,1]),
                             ([0,0,2],[0,0,1]),([0,0,1],[0,0,1]),
                             ([0,0,0],[0,0,0]),([float('nan'),0,0],[0,0,1])):
            with self.subTest(point=point), self.assertRaises(ValueError):
                ray_plane_intersection([0,0,1],point,[0,0,0],normal)

    def test_ray_plane_uses_selected_depth_not_outward_point_for_quality_gate(self):
        data = build_task_preview(self.capture(.05,0), INTRINSICS,FINGERTIPS)
        self.assertTrue(data['geometry_valid_for_planning_seed'])
        self.assertAlmostEqual(data['quality']['selected_plane_delta_m'],0.)
        self.assertAlmostEqual(data['quality']['operation_plane_delta_m'],.05)
        self.assertTrue(data['review_reasons'])

    def test_optional_upstream_source_and_radial_offset(self):
        data = build_task_preview(self.capture(), INTRINSICS,FINGERTIPS, entry_key='operation_point',radial_offset_m=-.02)
        self.assertFalse(data['assumptions']['upstream_radial_offset_bypassed'])
        np.testing.assert_allclose(data['task_preview']['insertion_reference_m'], [.45,.04,-.08], atol=1e-10)
        self.assertAlmostEqual(data['quality']['entry_radius_m'],.03)
        for options in ({'entry_key':'black_center_point'}, {'radial_offset_m':.101},
                        {'radial_offset_m':float('nan')}, {'plane_adjust_limit_m':0}):
            with self.assertRaises(ValueError):
                build_task_preview(self.capture(), INTRINSICS,FINGERTIPS, **options)

    def test_current_target_moves_inward_five_mm_and_up_ten_mm(self):
        capture=self.capture(diagonal_delta=.004)
        saved=json.dumps(capture,sort_keys=True)
        before=build_task_preview(capture,INTRINSICS,FINGERTIPS,radial_offset_m=.015)
        after=build_task_preview(capture,INTRINSICS,FINGERTIPS,
                                 radial_offset_m=RADIAL_OFFSET_M,height_offset_m=HEIGHT_OFFSET_M)
        old=before['task_preview'];new=after['task_preview'];axis=np.asarray(old['axis_center_m'])
        direction=np.asarray(old['insertion_reference_m'])-axis
        direction/=np.linalg.norm(direction)
        shift=-.005*direction+[0,0,.010]
        for key in ('insertion_reference_m','preinsert_point_m'):
            np.testing.assert_allclose(np.asarray(new[key])-old[key],shift,atol=1e-12)
        for key in ('axis_center_m','ray_plane_intersections_m','selected_source_point_raw_m'):
            self.assertEqual(new[key],old[key])
        for key in ('source','points_camera_raw_m','points_torso_raw_m','transforms','review_reasons'):
            self.assertEqual(after[key],before[key])
        self.assertEqual(after['geometry_valid_for_planning_seed'],before['geometry_valid_for_planning_seed'])
        self.assertEqual(after['quality']['selected_plane_delta_m'],before['quality']['selected_plane_delta_m'])
        self.assertAlmostEqual(after['quality']['entry_radius_m'],before['quality']['entry_radius_m']-.005)
        arcs=np.asarray(new['arc_minus120_to_plus120_m'])
        np.testing.assert_allclose(arcs[:,2],axis[2]+.010,atol=1e-12)
        np.testing.assert_allclose(np.linalg.norm(arcs[:,:2]-axis[:2],axis=1),after['quality']['entry_radius_m'])
        for side in ('left','right'):
            for key,pose in after['grasp_candidates'][side]['poses'].items():
                prior=before['grasp_candidates'][side]['poses'][key]
                np.testing.assert_allclose(np.asarray(pose['position_m'])-prior['position_m'],shift,atol=1e-12)
                np.testing.assert_allclose(np.asarray(pose['matrix_row_major'])[:3,:3],
                                           np.asarray(prior['matrix_row_major'])[:3,:3],atol=1e-12)
        rebuilt=with_radial_offset(before,RADIAL_OFFSET_M,height_offset_m=HEIGHT_OFFSET_M)
        self.assertEqual(rebuilt['task_preview'],new)
        self.assertEqual(rebuilt['grasp_candidates'],after['grasp_candidates'])
        self.assertEqual(with_radial_offset(rebuilt,RADIAL_OFFSET_M),rebuilt)
        self.assertEqual(with_radial_offset(rebuilt,RADIAL_OFFSET_M,height_offset_m=HEIGHT_OFFSET_M),rebuilt)
        self.assertEqual(json.dumps(capture,sort_keys=True),saved)

    def test_radial_offsets_100mm_bounds_and_center_crossing(self):
        capture=self.capture(entry_radius=.14)
        base=build_task_preview(capture,INTRINSICS,FINGERTIPS)
        for offset in (-.10,.10):
            direct=build_task_preview(capture,INTRINSICS,FINGERTIPS,radial_offset_m=offset)
            rebuilt=with_radial_offset(base,offset)
            for result in (direct,rebuilt):
                task=result['task_preview']
                self.assertEqual(task['radial_offset_m'],offset)
                np.testing.assert_allclose(task['axis_center_m'],base['task_preview']['axis_center_m'])
                self.assertAlmostEqual(np.linalg.norm(np.array(task['insertion_reference_m'])-
                    task['axis_center_m']),.14+offset)
        for offset in (-.101,.101,float('nan'),float('inf')):
            with self.assertRaises(ValueError):build_task_preview(capture,INTRINSICS,FINGERTIPS,radial_offset_m=offset)
            with self.assertRaises(ValueError):with_radial_offset(base,offset)
        small=build_task_preview(self.capture(),INTRINSICS,FINGERTIPS)
        with self.assertRaises(ValueError):with_radial_offset(small,-.10)
        with self.assertRaises(ValueError):build_task_preview(self.capture(),INTRINSICS,FINGERTIPS,radial_offset_m=-.10)

    def test_height_offset_does_not_hide_invalid_source_plane(self):
        data=build_task_preview(self.capture(.05,.05),INTRINSICS,FINGERTIPS,height_offset_m=.010)
        self.assertFalse(data['geometry_valid_for_planning_seed'])
        self.assertEqual(data['grasp_candidates'],{})
        self.assertAlmostEqual(data['quality']['selected_plane_delta_m'],.05)

    def test_invalid_height_offsets_are_rejected(self):
        geometry=build_task_preview(self.capture(),INTRINSICS,FINGERTIPS)
        for height in (float('nan'),float('inf'),-.031,.101):
            with self.subTest(height=height):
                with self.assertRaises(ValueError):
                    build_task_preview(self.capture(),INTRINSICS,FINGERTIPS,height_offset_m=height)
                with self.assertRaises(ValueError):
                    with_radial_offset(geometry,.010,height_offset_m=height)

    def test_height_offset_up_to_100mm_is_preserved_without_clipping(self):
        for height in (-.03,.031,.10):
            with self.subTest(height=height):
                base=build_task_preview(self.capture(),INTRINSICS,FINGERTIPS)
                direct=build_task_preview(self.capture(),INTRINSICS,FINGERTIPS,height_offset_m=height)
                rebuilt=with_radial_offset(base,0.,height_offset_m=height)
                for result in (direct,rebuilt):
                    task=result['task_preview']
                    self.assertEqual(task['height_offset_m'],height)
                    np.testing.assert_allclose(task['axis_center_m'],base['task_preview']['axis_center_m'])
                    np.testing.assert_allclose(np.array(task['insertion_reference_m'])-
                        base['task_preview']['insertion_reference_m'],
                        np.array(task['assumed_normal'])*height,atol=1e-10)

    def test_actual_saved_frame_new_reference_and_same_rgbd_transform(self):
        capture=json.loads((ROOT/'data/vision_captures/20260909_115247_points3d/capture.json').read_text())
        data=build_task_preview(capture,INTRINSICS,FINGERTIPS)
        np.testing.assert_allclose(data['task_preview']['insertion_reference_m'],
                                   [.4269333,.0815144,-.0248901],atol=1e-7)
        self.assertAlmostEqual(data['quality']['entry_radius_m'],.055995,places=6)
        for key,point in capture['observation']['points_camera'].items():
            self.assertEqual(data['points_camera_raw_m'][key],point)
        self.assertTrue(data['geometry_valid_for_planning_seed'])


class IndexTcpEstimateTest(unittest.TestCase):
    def test_bilateral_offsets_include_connector_and_change_with_posture(self):
        estimates = generate()
        self.assertEqual(estimates["connector_length_m"], CONNECTOR_LENGTH)
        self.assertFalse(estimates["hardware_calibrated"])
        for side, y in (("left", 0.003), ("right", -0.003)):
            palm = estimates["sides"][side]["wrist_yaw_from_palm"]["xyz_m"]
            np.testing.assert_allclose(palm, [0.0815, y, 0.0])
            states = estimates["sides"][side]["states"]
            self.assertGreater(states["open"]["wrist_yaw_from_index_tcp"]["xyz_m"][0], 0.26)
            self.assertLess(states["folded"]["wrist_yaw_from_index_tcp"]["xyz_m"][0], 0.18)
            self.assertNotEqual(
                states["open"]["wrist_yaw_from_index_tcp"]["xyz_m"],
                states["folded"]["wrist_yaw_from_index_tcp"]["xyz_m"],
            )


if __name__ == "__main__":
    unittest.main()
