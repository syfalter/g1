#!/usr/bin/env python3
"""Reusable end-to-end recognizer for the returned-to-center pedal.

Import :class:`PedalCenterRecognizer` from other Python modules.  Its public
``predict`` method accepts a single OpenCV BGR image and returns only plain
pixel geometry—no YOLO objects, UI windows, or filesystem side effects.
"""

from __future__ import annotations

import os
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, List

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parent
# Keep Ultralytics' runtime settings outside the delivered package so that the
# package directory remains read-only after deployment.
_USER_CONFIG_ROOT = Path(os.environ.get("LOCALAPPDATA", str(Path.home())))
os.environ.setdefault("YOLO_CONFIG_DIR", str(_USER_CONFIG_ROOT / "pedal_center_recognizer" / "ultralytics"))
from ultralytics import YOLO  # noqa: E402

DEFAULT_RUN = ROOT / "runs" / "pedal_center_yolov8_twostage_v1"
# A distributable package places its weights beside this module in ``models``.
# Keep the training-run locations as a fallback when using this repository.
_BUNDLED_MODELS = ROOT / "models"
DEFAULT_STAGE1 = _BUNDLED_MODELS / "stage1_best.pt"
if not DEFAULT_STAGE1.is_file():
    DEFAULT_STAGE1 = DEFAULT_RUN / "stage1" / "weights" / "best.pt"
DEFAULT_STAGE2 = _BUNDLED_MODELS / "stage2_best.pt"
if not DEFAULT_STAGE2.is_file():
    DEFAULT_STAGE2 = DEFAULT_RUN / "stage2" / "weights" / "best.pt"

Box = List[int]
Point = List[int]
Polygon = List[Point]


@dataclass(frozen=True)
class PedalCenterResult:
    """JSON-serializable output from a complete two-stage recognition pass."""

    image_width: int
    image_height: int
    pedal_centered: Box | None = None
    black_center: Box | None = None
    black_center_point: Point | None = None
    operation_zone: Polygon | None = None
    operation_diagonal_intersection: Point | None = None
    operation_point: Point | None = None
    stage1_confidence: float | None = None
    stage2_confidence: float | None = None

    @property
    def ready(self) -> bool:
        return self.operation_point is not None

    def to_dict(self) -> dict[str, Any]:
        payload = asdict(self)
        payload["ready"] = self.ready
        return payload


def _clamp_box(box: list[float], width: int, height: int) -> Box:
    x1, y1, x2, y2 = (round(value) for value in box)
    return [max(0, min(x1, width - 1)), max(0, min(y1, height - 1)), max(1, min(x2, width)), max(1, min(y2, height))]


def _highest_box(result: Any) -> tuple[float, list[float]] | None:
    if result.boxes is None or len(result.boxes) == 0:
        return None
    index = int(result.boxes.conf.argmax().item())
    return float(result.boxes.conf[index].item()), result.boxes.xyxy[index].detach().cpu().tolist()


def _padded_crop(box: Box, width: int, height: int, padding: float) -> Box:
    x1, y1, x2, y2 = box
    padding_x, padding_y = (x2 - x1) * padding, (y2 - y1) * padding
    return [max(0, round(x1 - padding_x)), max(0, round(y1 - padding_y)), min(width, round(x2 + padding_x)), min(height, round(y2 + padding_y))]


def operation_zone(outer: Box, center: Box) -> Polygon:
    """Approximate the hollow finger-entry area as a downward narrowing trapezoid."""
    outer_x1, outer_y1, outer_x2, outer_y2 = outer
    center_x1, center_y1, center_x2, _ = center
    outer_height = outer_y2 - outer_y1
    center_x, center_width = (center_x1 + center_x2) / 2, center_x2 - center_x1
    top_y, bottom_y = round(outer_y1 + outer_height * 0.12), round(center_y1 - outer_height * 0.02)
    top_half_width, bottom_half_width = center_width * 0.90, center_width * 0.35
    return [
        [round(max(outer_x1, center_x - top_half_width)), top_y],
        [round(min(outer_x2, center_x + top_half_width)), top_y],
        [round(min(outer_x2, center_x + bottom_half_width)), bottom_y],
        [round(max(outer_x1, center_x - bottom_half_width)), bottom_y],
    ]


def diagonal_intersection(polygon: Polygon) -> Point:
    """Return the P0-P2 and P1-P3 intersection for a four-point polygon."""
    (x1, y1), (x2, y2), (x3, y3), (x4, y4) = polygon[0], polygon[2], polygon[1], polygon[3]
    denominator = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
    if abs(denominator) < 1e-9:
        return [round(sum(point[0] for point in polygon) / 4), round(sum(point[1] for point in polygon) / 4)]
    determinant_a, determinant_b = x1 * y2 - y1 * x2, x3 * y4 - y3 * x4
    return [
        round((determinant_a * (x3 - x4) - (x1 - x2) * determinant_b) / denominator),
        round((determinant_a * (y3 - y4) - (y1 - y2) * determinant_b) / denominator),
    ]


def box_center(box: Box) -> Point:
    """Intersection of the two diagonals of the approximately square black-center box."""
    x1, y1, x2, y2 = box
    return [round((x1 + x2) / 2), round((y1 + y2) / 2)]


def radial_offset(center: Point, point: Point, scale: float) -> Point:
    """Move ``point`` away from ``center`` along the same radius."""
    return [round(center[0] + (point[0] - center[0]) * scale), round(center[1] + (point[1] - center[1]) * scale)]


class PedalCenterRecognizer:
    """Load the models once and return robot-ready geometry for each BGR frame.

    The low default stage-2 threshold accommodates the wider real head-camera
    view.  The chosen maximum box remains constrained inside the stage-1 crop.
    """

    def __init__(
        self,
        stage1_path: str | Path = DEFAULT_STAGE1,
        stage2_path: str | Path = DEFAULT_STAGE2,
        *,
        device: str = "0",
        stage1_conf: float = 0.10,
        stage2_conf: float = 0.001,
        crop_padding: float = 0.15,
        radial_offset_scale: float = 1.35,
    ) -> None:
        self.stage1_path, self.stage2_path = Path(stage1_path), Path(stage2_path)
        if not self.stage1_path.is_file() or not self.stage2_path.is_file():
            raise FileNotFoundError(f"找不到两阶段权重：{self.stage1_path}；{self.stage2_path}")
        self.device = device
        self.stage1_conf, self.stage2_conf = stage1_conf, stage2_conf
        self.crop_padding, self.radial_offset_scale = crop_padding, radial_offset_scale
        self._stage1, self._stage2 = YOLO(str(self.stage1_path)), YOLO(str(self.stage2_path))

    def predict(self, bgr_image: np.ndarray) -> PedalCenterResult:
        """Recognize one HxWx3 OpenCV BGR image; all returned coordinates are pixels."""
        if bgr_image is None or bgr_image.ndim != 3 or bgr_image.shape[2] != 3:
            raise ValueError("predict() 需要 HxWx3 的 OpenCV BGR 图像")
        height, width = bgr_image.shape[:2]
        first = self._stage1.predict(bgr_image, imgsz=960, conf=self.stage1_conf, device=self.device, verbose=False)[0]
        outer = _highest_box(first)
        if outer is None:
            return PedalCenterResult(image_width=width, image_height=height)
        outer_confidence, outer_raw = outer
        outer_box = _clamp_box(outer_raw, width, height)
        x1, y1, x2, y2 = _padded_crop(outer_box, width, height, self.crop_padding)
        second = self._stage2.predict(bgr_image[y1:y2, x1:x2], imgsz=640, conf=self.stage2_conf, device=self.device, verbose=False)[0]
        center = _highest_box(second)
        if center is None:
            return PedalCenterResult(image_width=width, image_height=height, pedal_centered=outer_box, stage1_confidence=outer_confidence)
        center_confidence, center_raw = center
        local_center = _clamp_box(center_raw, x2 - x1, y2 - y1)
        center_box = [local_center[0] + x1, local_center[1] + y1, local_center[2] + x1, local_center[3] + y1]
        center_point = box_center(center_box)
        zone = operation_zone(outer_box, center_box)
        diagonal_point = diagonal_intersection(zone)
        final_point = radial_offset(center_point, diagonal_point, self.radial_offset_scale)
        return PedalCenterResult(
            image_width=width, image_height=height, pedal_centered=outer_box, black_center=center_box,
            black_center_point=center_point, operation_zone=zone, operation_diagonal_intersection=diagonal_point,
            operation_point=final_point, stage1_confidence=outer_confidence, stage2_confidence=center_confidence,
        )

    def annotate(self, bgr_image: np.ndarray, result: PedalCenterResult) -> np.ndarray:
        """Return a review image only; never writes files or opens a window."""
        preview = bgr_image.copy()
        if result.operation_zone is not None:
            overlay = preview.copy()
            cv2.fillPoly(overlay, [np.asarray(result.operation_zone, dtype=np.int32)], (255, 255, 0))
            preview = cv2.addWeighted(overlay, 0.25, preview, 0.75, 0)
            cv2.polylines(preview, [np.asarray(result.operation_zone, dtype=np.int32)], True, (255, 255, 0), 2)
        if result.pedal_centered is not None:
            cv2.rectangle(preview, tuple(result.pedal_centered[:2]), tuple(result.pedal_centered[2:]), (0, 220, 0), 2)
        if result.black_center is not None:
            cv2.rectangle(preview, tuple(result.black_center[:2]), tuple(result.black_center[2:]), (255, 0, 255), 2)
        if result.black_center_point is not None:
            cv2.drawMarker(preview, tuple(result.black_center_point), (255, 0, 255), cv2.MARKER_CROSS, 16, 2)
        if result.operation_point is not None:
            cv2.circle(preview, tuple(result.operation_point), 7, (0, 0, 255), -1)
        return preview
