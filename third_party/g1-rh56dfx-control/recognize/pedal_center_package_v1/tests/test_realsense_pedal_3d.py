import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from vision_runtime import _points_payload, project_aligned_result


class FakeResult:
    black_center_point = [2, 2]
    operation_diagonal_intersection = None
    operation_point = [4, 3]


class FakeRealSense:
    @staticmethod
    def rs2_deproject_pixel_to_point(_intrinsics, pixel, depth_m):
        return [pixel[0] * depth_m, pixel[1] * depth_m, depth_m]


class RealsensePedal3DTest(unittest.TestCase):
    def test_projects_from_aligned_depth_pixels(self):
        depth = np.full((6, 8), 1000, dtype=np.uint16)
        result = project_aligned_result(
            FakeResult(), depth, 0.001, object(), 0, FakeRealSense
        )
        self.assertEqual(result.points_camera["operation_point"].to_list(), [4.0, 3.0, 1.0])
        self.assertIsNone(result.points_camera["operation_diagonal_intersection"])
        self.assertEqual(result.invalid_depth_keys, ())

    def test_serializes_missing_points_without_a_fake_transform(self):
        depth = np.full((6, 8), 1000, dtype=np.uint16)
        result = project_aligned_result(
            FakeResult(), depth, 0.001, object(), 0, FakeRealSense
        )
        payload = _points_payload(result.points_camera)
        self.assertEqual(payload["operation_diagonal_intersection"], [])
        self.assertEqual(payload["operation_point"], [4.0, 3.0, 1.0])


if __name__ == "__main__":
    unittest.main()
