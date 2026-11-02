"""Persistent synchronized RGB-D capture and pedal perception runtime."""

from __future__ import annotations

import json
import os
import threading
import time
import uuid
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any

import cv2
import numpy as np

try:
    from .pedal_3d_projector import (
        PedalCenter3DResult,
        Point3D,
        RigidTransform,
        load_rigid_transform,
        transform_points,
    )
except ImportError:
    from pedal_3d_projector import (
        PedalCenter3DResult,
        Point3D,
        RigidTransform,
        load_rigid_transform,
        transform_points,
    )


PROJECT_ROOT = Path(__file__).resolve().parents[2]
PACKAGE_ROOT = Path(__file__).resolve().parent
DEFAULT_CAPTURES_DIR = PROJECT_ROOT / "captures"
DEFAULT_HEAD_SERIAL = "243322073672"
CAMERA_FRAME = "head_color_camera_optical_frame"
DEFAULT_STAGE1 = PACKAGE_ROOT / "models/stage1_best.pt"
DEFAULT_STAGE2 = PACKAGE_ROOT / "models/stage2_best.pt"


class CameraBusyError(RuntimeError):
    """The D435I stream is owned by another process."""


@dataclass(frozen=True)
class VisionConfig:
    serial: str = DEFAULT_HEAD_SERIAL
    width: int = 1280
    height: int = 720
    fps: int = 30
    warmup_frames: int = 15
    device: str = "0"
    depth_patch_radius: int = 2
    stage1_path: Path = DEFAULT_STAGE1
    stage2_path: Path = DEFAULT_STAGE2
    target_extrinsics: Path | None = None
    captures_dir: Path = DEFAULT_CAPTURES_DIR

    def validate(self) -> None:
        if self.width <= 0 or self.height <= 0 or self.fps <= 0:
            raise ValueError("width, height and fps must be positive")
        if self.warmup_frames < 0 or self.depth_patch_radius < 0:
            raise ValueError("warmup_frames and depth_patch_radius cannot be negative")


def _median_depth(
    depth_image: np.ndarray,
    pixel: list[float],
    depth_scale: float,
    patch_radius: int,
) -> float | None:
    u, v = round(pixel[0]), round(pixel[1])
    height, width = depth_image.shape
    if not (0 <= u < width and 0 <= v < height):
        return None
    x1, x2 = max(0, u - patch_radius), min(width, u + patch_radius + 1)
    y1, y2 = max(0, v - patch_radius), min(height, v + patch_radius + 1)
    patch_m = depth_image[y1:y2, x1:x2].astype(np.float64, copy=False) * depth_scale
    valid = patch_m[np.isfinite(patch_m) & (patch_m >= 0.1) & (patch_m <= 10.0)]
    return float(np.median(valid)) if valid.size else None


def project_aligned_result(
    pedal_result: Any,
    aligned_depth: np.ndarray,
    depth_scale: float,
    color_intrinsics: Any,
    patch_radius: int,
    rs: Any,
) -> PedalCenter3DResult:
    keypoints = {
        "black_center_point": pedal_result.black_center_point,
        "operation_diagonal_intersection": pedal_result.operation_diagonal_intersection,
        "operation_point": pedal_result.operation_point,
    }
    points: dict[str, Point3D | None] = {}
    invalid: list[str] = []
    for name, color_pixel in keypoints.items():
        point = None
        if color_pixel is not None:
            pixel = [float(color_pixel[0]), float(color_pixel[1])]
            depth_m = _median_depth(aligned_depth, pixel, depth_scale, patch_radius)
            if depth_m is not None:
                xyz = rs.rs2_deproject_pixel_to_point(color_intrinsics, pixel, depth_m)
                point = Point3D(*(float(value) for value in xyz))
            else:
                invalid.append(name)
        points[name] = point
    return PedalCenter3DResult(points, tuple(invalid))


def _points_payload(points: dict[str, Point3D | None]) -> dict[str, list[float]]:
    return {
        name: [] if point is None else point.to_list()
        for name, point in points.items()
    }


def _intrinsics_payload(intrinsics: Any) -> dict[str, Any]:
    return {
        "fx": intrinsics.fx,
        "fy": intrinsics.fy,
        "cx": intrinsics.ppx,
        "cy": intrinsics.ppy,
        "width": intrinsics.width,
        "height": intrinsics.height,
        "distortion_model": str(intrinsics.model),
        "distortion": list(intrinsics.coeffs),
    }


def _extrinsics_payload(extrinsics: Any) -> dict[str, Any]:
    return {
        "rotation": list(extrinsics.rotation),
        "translation_m": list(extrinsics.translation),
    }


def _frame_payload(frame: Any, encoding: str) -> dict[str, Any]:
    video = frame.profile.as_video_stream_profile()
    return {
        "frame_number": frame.get_frame_number(),
        "timestamp_ms": frame.get_timestamp(),
        "timestamp_domain": str(frame.get_frame_timestamp_domain()),
        "width": video.width(),
        "height": video.height(),
        "fps": frame.profile.fps(),
        "format": str(frame.profile.format()),
        "encoding": encoding,
    }


def _timing_payload(frame: Any, rs: Any) -> dict[str, Any]:
    metadata = {}
    errors = {}
    for name in ("frame_counter", "frame_timestamp", "sensor_timestamp",
                 "actual_exposure", "backend_timestamp", "time_of_arrival"):
        key = getattr(rs.frame_metadata_value, name, None)
        try:
            metadata[name] = (
                int(frame.get_frame_metadata(key))
                if key is not None and frame.supports_frame_metadata(key) else None
            )
        except RuntimeError as error:
            metadata[name] = None
            errors[name] = str(error)
    return {
        "frame_number": int(frame.get_frame_number()),
        "timestamp_ms": float(frame.get_timestamp()),
        "timestamp_domain": str(frame.get_frame_timestamp_domain()),
        "metadata_raw": metadata,
        "metadata_errors": errors,
    }


def _device_info(device: Any, key: Any) -> str:
    return device.get_info(key) if device.supports(key) else ""


class PedalVisionRuntime:
    """Keep camera and models ready, but run inference only when requested."""

    def __init__(self, config: VisionConfig, rs: Any):
        config.validate()
        try:
            from .pedal_center_recognizer import PedalCenterRecognizer
        except ImportError:
            from pedal_center_recognizer import PedalCenterRecognizer

        self.config = config
        self.rs = rs
        self.transform = self._load_transform(config.target_extrinsics)
        self.recognizer = PedalCenterRecognizer(
            config.stage1_path, config.stage2_path, device=config.device
        )
        self._capture_lock = threading.Lock()
        self._state = "idle"
        self._last_error = ""
        self._timing_session = uuid.uuid4().hex
        self._stream_ready = False
        self._pipeline, self._profile, self._align_to_color, self._depth_scale = (
            self._open_camera()
        )

    @staticmethod
    def _load_transform(path: Path | None) -> RigidTransform | None:
        if path is None:
            return None
        transform = load_rigid_transform(path)
        if transform.source_frame != CAMERA_FRAME:
            raise ValueError(
                f"extrinsics source_frame must be {CAMERA_FRAME!r}, got "
                f"{transform.source_frame!r}"
            )
        return transform

    def _open_camera(self) -> tuple[Any, Any, Any, float]:
        pipeline, rs_config = self.rs.pipeline(), self.rs.config()
        rs_config.enable_device(self.config.serial)
        rs_config.enable_stream(
            self.rs.stream.color, self.config.width, self.config.height,
            self.rs.format.bgr8, self.config.fps,
        )
        rs_config.enable_stream(
            self.rs.stream.depth, self.config.width, self.config.height,
            self.rs.format.z16, self.config.fps,
        )
        try:
            profile = pipeline.start(rs_config)
        except RuntimeError as error:
            if "busy" in str(error).lower() or "errno=16" in str(error).lower():
                raise CameraBusyError(
                    "head D435I RGB stream is busy; vision_service must be the "
                    "camera owner instead of videohub_pc4"
                ) from error
            raise
        for _ in range(self.config.warmup_frames):
            pipeline.wait_for_frames(5000)
        self._rs_config = rs_config
        self._stream_ready = True
        return (
            pipeline,
            profile,
            self.rs.align(self.rs.stream.color),
            profile.get_device().first_depth_sensor().get_depth_scale(),
        )

    def capture_once(self, save_artifacts: bool = False, *, restart_stream: bool = False,
                     request_id: int = 0) -> dict[str, Any]:
        request_entered = time.monotonic_ns()
        with self._capture_lock:
            self._state = "capturing"
            try:
                timing = {
                    "schema": "vision_frame_timing_v1",
                    "worker_session": self._timing_session,
                    "worker_pid": os.getpid(),
                    "request_entered_monotonic_ns": request_entered,
                    "wait_started_monotonic_ns": time.monotonic_ns(),
                    "wait_started_realtime_ns": time.time_ns(),
                    "exposure_bounds_verified": False,
                    "semantics": "Owner receipt evidence only; device and host clocks are not equated",
                }
                if restart_stream:
                    timing["request_id"] = request_id
                    timing["stream_stop_started_monotonic_ns"] = time.monotonic_ns()
                    self._stream_ready = False
                    self._pipeline.stop()
                    timing["stream_stopped_monotonic_ns"] = time.monotonic_ns()
                    self._profile = self._pipeline.start(self._rs_config)
                    self._stream_ready = True
                    self._align_to_color = self.rs.align(self.rs.stream.color)
                    self._depth_scale = self._profile.get_device().first_depth_sensor().get_depth_scale()
                    timing["stream_started_monotonic_ns"] = time.monotonic_ns()
                    timing["stream_session"] = uuid.uuid4().hex
                    # stop() tears down the pipeline's queues before starting
                    # the same owner-held pipeline again. No parallel owner.
                    warmup = max(15, self.config.warmup_frames)
                    for _ in range(warmup):
                        self._pipeline.wait_for_frames(5000)
                    timing["warmup_frames"] = warmup
                    timing["capture_contract"] = "owner_stream_restart_static_v1"
                    timing["wait_started_monotonic_ns"] = time.monotonic_ns()
                    timing["wait_started_realtime_ns"] = time.time_ns()
                frames = self._pipeline.wait_for_frames(5000)
                timing["frames_received_monotonic_ns"] = time.monotonic_ns()
                timing["frames_received_realtime_ns"] = time.time_ns()
                raw_color = frames.get_color_frame()
                raw_depth = frames.get_depth_frame()
                if not raw_color or not raw_depth:
                    raise RuntimeError("D435I did not return a complete raw RGB-D frameset")
                timing["color"] = _timing_payload(raw_color, self.rs)
                timing["depth"] = _timing_payload(raw_depth, self.rs)
                aligned = self._align_to_color.process(frames)
                color_frame = aligned.get_color_frame()
                depth_frame = aligned.get_depth_frame()
                if not color_frame or not depth_frame:
                    raise RuntimeError("D435I did not return a complete aligned RGB-D frameset")

                color = np.asanyarray(color_frame.get_data()).copy()
                depth_image = np.asanyarray(depth_frame.get_data()).copy()
                color_intrinsics = color_frame.profile.as_video_stream_profile().get_intrinsics()
                started = time.monotonic()
                result_2d = self.recognizer.predict(color)
                result_3d = project_aligned_result(
                    result_2d, depth_image, self._depth_scale, color_intrinsics,
                    self.config.depth_patch_radius, self.rs,
                )
                inference_ms = (time.monotonic() - started) * 1000.0
                annotated = self.recognizer.annotate(color, result_2d)
                observation = self._build_observation(
                    result_2d, result_3d, color_frame, depth_frame,
                    color_intrinsics, inference_ms, self._profile, self._depth_scale,
                )
                timing["observation_ready_monotonic_ns"] = time.monotonic_ns()
                observation["frame_timing"] = timing
                observation["capture_result"] = (
                    self._save_record(observation, color, depth_image, annotated)
                    if save_artifacts else ""
                )
                self._state = "idle"
                self._last_error = ""
                return observation
            except Exception as error:
                self._state = "degraded"
                self._last_error = str(error)
                raise

    def _build_observation(
        self, result_2d: Any, result_3d: PedalCenter3DResult,
        color_frame: Any, depth_frame: Any, color_intrinsics: Any,
        inference_ms: float, profile: Any, depth_scale: float,
    ) -> dict[str, Any]:
        color_meta = _frame_payload(color_frame, "BGR8")
        depth_meta = _frame_payload(depth_frame, "Z16 aligned to color")
        points_target = (
            {} if self.transform is None
            else transform_points(result_3d.points_camera, self.transform)
        )
        device = profile.get_device()
        active_color = profile.get_stream(self.rs.stream.color).as_video_stream_profile()
        active_depth = profile.get_stream(self.rs.stream.depth).as_video_stream_profile()
        return {
            "sequence": int(color_meta["frame_number"]),
            "captured_at_ms": time.time() * 1000.0,
            "inference_ms": inference_ms,
            "valid_2d": result_2d.ready,
            "valid_3d": result_3d.ready,
            "target_frame_ready": self.transform is not None and result_3d.ready,
            "image_width": result_2d.image_width,
            "image_height": result_2d.image_height,
            "pedal_centered": result_2d.pedal_centered or [],
            "black_center": result_2d.black_center or [],
            "black_center_point": result_2d.black_center_point or [],
            "operation_zone": result_2d.operation_zone or [],
            "operation_diagonal_intersection": result_2d.operation_diagonal_intersection or [],
            "operation_point": result_2d.operation_point or [],
            "stage1_confidence": -1.0 if result_2d.stage1_confidence is None else result_2d.stage1_confidence,
            "stage2_confidence": -1.0 if result_2d.stage2_confidence is None else result_2d.stage2_confidence,
            "points_camera": _points_payload(result_3d.points_camera),
            "points_target": _points_payload(points_target),
            "camera_frame": CAMERA_FRAME,
            "target_frame": "" if self.transform is None else self.transform.target_frame,
            "transform_status": "not_configured" if self.transform is None else "applied",
            "units": "m",
            "invalid_depth_keys": list(result_3d.invalid_depth_keys),
            "camera_name": _device_info(device, self.rs.camera_info.name),
            "camera_serial": _device_info(device, self.rs.camera_info.serial_number),
            "depth_scale_m_per_unit": depth_scale,
            "color_frame_number": int(color_meta["frame_number"]),
            "depth_frame_number": int(depth_meta["frame_number"]),
            "color_timestamp_ms": float(color_meta["timestamp_ms"]),
            "depth_timestamp_ms": float(depth_meta["timestamp_ms"]),
            "timestamp_delta_ms": abs(float(color_meta["timestamp_ms"]) - float(depth_meta["timestamp_ms"])),
            "color_intrinsics": _intrinsics_payload(color_intrinsics),
            "depth_to_color_extrinsics": _extrinsics_payload(active_depth.get_extrinsics_to(active_color)),
        }

    def status(self) -> dict[str, Any]:
        return {
            "frame_timing_schema": "vision_frame_timing_v1",
            "capture_contracts": ["owner_stream_restart_static_v1"],
            "worker_session": self._timing_session,
            "state": self._state,
            "camera_ready": self._pipeline is not None and self._stream_ready,
            "model_ready": self.recognizer is not None,
            "calibration_configured": self.transform is not None,
            "camera_frame": CAMERA_FRAME,
            "target_frame": "" if self.transform is None else self.transform.target_frame,
            "last_error": self._last_error,
        }

    def _save_record(
        self,
        observation: dict[str, Any],
        color: np.ndarray,
        depth_image: np.ndarray,
        annotated: np.ndarray,
    ) -> str:
        payload = dict(observation)
        captured_at = datetime.now().astimezone()
        captures_dir = self.config.captures_dir.expanduser().resolve()
        captures_dir.mkdir(parents=True, exist_ok=True)
        capture_id = "head_rgbd_pedal_" + captured_at.strftime("%Y%m%d_%H%M%S_%f")[:-3]
        paths = {
            "color_image": captures_dir / f"{capture_id}_color.png",
            "depth_image": captures_dir / f"{capture_id}_depth.png",
            "annotated_image": captures_dir / f"{capture_id}_annotated.jpg",
            "result": captures_dir / f"{capture_id}_result.json",
        }
        if any(path.exists() for path in paths.values()):
            raise RuntimeError(f"capture record already exists: {capture_id}")
        for key, image in (("color_image", color), ("depth_image", depth_image), ("annotated_image", annotated)):
            if not cv2.imwrite(str(paths[key]), image):
                raise RuntimeError(f"failed to save capture image: {paths[key]}")
        payload["capture"] = {
            "id": capture_id,
            "recorded_at": captured_at.isoformat(timespec="milliseconds"),
            **{name: str(path) for name, path in paths.items()},
        }
        paths["result"].write_text(
            json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
        )
        return str(paths["result"])

    def close(self) -> None:
        if self._pipeline is not None:
            self._pipeline.stop()
            self._pipeline = None

    def __enter__(self) -> "PedalVisionRuntime":
        return self

    def __exit__(self, *_: Any) -> None:
        self.close()
