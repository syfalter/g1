import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from pedal_3d_projector import (
    CameraIntrinsics,
    Point3D,
    RigidTransform,
    deproject_pixel,
    project_pedal_result,
    transform_points,
)


class FakePedalResult:
    black_center_point = [320, 240]
    operation_diagonal_intersection = [420, 340]
    operation_point = [220, 140]


class Pedal3DProjectorTest(unittest.TestCase):
    def setUp(self):
        self.intrinsics = CameraIntrinsics(500.0, 500.0, 320.0, 240.0, 640, 480)

    def test_deprojects_pixel_in_camera_metres(self):
        depth = np.full((480, 640), 1500, dtype=np.uint16)
        point = deproject_pixel([420, 340], depth, self.intrinsics, depth_scale=0.001)
        self.assertIsNotNone(point)
        self.assertAlmostEqual(point.x, 0.3)
        self.assertAlmostEqual(point.y, 0.3)
        self.assertAlmostEqual(point.z, 1.5)

    def test_uses_median_of_valid_depth_patch(self):
        depth = np.zeros((480, 640), dtype=np.uint16)
        depth[238:243, 318:323] = 1200
        depth[240, 320] = 8000
        point = deproject_pixel([320, 240], depth, self.intrinsics, depth_scale=0.001)
        self.assertIsNotNone(point)
        self.assertAlmostEqual(point.z, 1.2)

    def test_projects_recognizer_keypoints(self):
        depth = np.full((480, 640), 1000, dtype=np.uint16)
        result = project_pedal_result(FakePedalResult(), depth, self.intrinsics, depth_scale=0.001)
        self.assertTrue(result.ready)
        self.assertEqual(result.invalid_depth_keys, ())
        self.assertEqual(result.points_camera["black_center_point"].to_list(), [0.0, 0.0, 1.0])

    def test_rejects_unaligned_depth_shape(self):
        with self.assertRaisesRegex(ValueError, "aligned to the color image"):
            deproject_pixel([1, 1], np.zeros((240, 320), dtype=np.uint16), self.intrinsics, depth_scale=0.001)

    def test_transforms_camera_points_to_configured_target(self):
        transform = RigidTransform(
            "head_color_camera_optical_frame",
            "base_link",
            ((0.0, 0.0, 1.0), (-1.0, 0.0, 0.0), (0.0, -1.0, 0.0)),
            (0.2, 0.0, 0.5),
        )
        points = transform_points(
            {"operation_point": Point3D(0.1, 0.2, 1.0), "missing": None}, transform
        )
        self.assertEqual(points["operation_point"].to_list(), [1.2, -0.1, 0.3])
        self.assertIsNone(points["missing"])

    def test_rejects_non_rotation_matrix(self):
        with self.assertRaisesRegex(ValueError, "orthonormal"):
            RigidTransform(
                "camera",
                "base_link",
                ((1.0, 0.0, 0.0), (0.0, 2.0, 0.0), (0.0, 0.0, 1.0)),
                (0.0, 0.0, 0.0),
            )


if __name__ == "__main__":
    unittest.main()
