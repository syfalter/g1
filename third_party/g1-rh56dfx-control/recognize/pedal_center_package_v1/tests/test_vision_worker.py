import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from vision_worker import (
    INVALID_ARGUMENT,
    INVALID_DEPTH,
    OK,
    TARGET_FRAME_UNAVAILABLE,
    WorkerApplication,
)


class FakeRuntime:
    def __init__(self, observation):
        self.observation = observation

    def capture_once(self, save_artifacts=False):
        return dict(self.observation)

    def status(self):
        return {
            "state": "ready",
            "camera_ready": True,
            "model_ready": True,
            "calibration_configured": False,
            "camera_frame": "head_color_camera_optical_frame",
            "target_frame": "",
            "last_error": "",
        }

def observation(**overrides):
    import time

    value = {
        "sequence": 7,
        "captured_at_ms": time.time() * 1000.0,
        "valid_2d": True,
        "valid_3d": True,
        "target_frame_ready": False,
    }
    value.update(overrides)
    return value


class WorkerApplicationTest(unittest.TestCase):
    def test_reports_status_without_running_inference(self):
        reply = WorkerApplication(FakeRuntime(observation())).handle(
            {"action": "get_status", "request": {"request_id": 8}}
        )
        self.assertEqual(reply["code"], OK)
        self.assertEqual(reply["state"], "ready")

    def test_returns_one_camera_frame_without_calibration(self):
        reply = WorkerApplication(FakeRuntime(observation())).handle(
            {"action": "recognize_once", "request": {"request_id": 9}}
        )
        self.assertEqual(reply["code"], OK)
        self.assertEqual(reply["request_id"], 9)

    def test_rejects_non_boolean_options(self):
        reply = WorkerApplication(FakeRuntime(observation())).handle(
            {"action": "recognize_once", "request": {"require_3d": "yes"}}
        )
        self.assertEqual(reply["code"], INVALID_ARGUMENT)

    def test_can_require_target_frame(self):
        reply = WorkerApplication(FakeRuntime(observation())).handle(
            {
                "action": "recognize_once",
                "request": {"request_id": 9, "require_target_frame": True},
            }
        )
        self.assertEqual(reply["code"], TARGET_FRAME_UNAVAILABLE)

    def test_rejects_invalid_depth_when_required(self):
        reply = WorkerApplication(FakeRuntime(observation(valid_3d=False))).handle(
            {"action": "recognize_once", "request": {"require_3d": True}}
        )
        self.assertEqual(reply["code"], INVALID_DEPTH)


if __name__ == "__main__":
    unittest.main()
