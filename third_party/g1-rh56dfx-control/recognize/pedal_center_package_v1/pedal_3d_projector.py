"""Lift pedal-recognition pixels into the aligned color-camera frame.

This module deliberately does not estimate depth from an RGB image.  It needs
an aligned depth frame captured at the same time as the color frame, because a
pixel and camera intrinsics alone describe a ray, not a unique 3D point.
"""

from __future__ import annotations

import json
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Mapping, Sequence

import cv2
import numpy as np

Point2D = Sequence[float]


@dataclass(frozen=True)
class CameraIntrinsics:
    """Pinhole intrinsics for the color image to which depth is aligned."""

    fx: float
    fy: float
    cx: float
    cy: float
    width: int
    height: int
    distortion: tuple[float, ...] = ()

    def __post_init__(self) -> None:
        if self.fx <= 0 or self.fy <= 0:
            raise ValueError("fx and fy must be positive")
        if self.width <= 0 or self.height <= 0:
            raise ValueError("image width and height must be positive")
        if len(self.distortion) not in (0, 4, 5, 8, 12, 14):
            raise ValueError("distortion must have an OpenCV-supported length")

    @property
    def matrix(self) -> np.ndarray:
        return np.array(
            [[self.fx, 0.0, self.cx], [0.0, self.fy, self.cy], [0.0, 0.0, 1.0]],
            dtype=np.float64,
        )

    def normalized_ray(self, pixel: Point2D) -> tuple[float, float]:
        """Return the undistorted pinhole ray ``(x/z, y/z)`` for one pixel."""
        if len(pixel) != 2:
            raise ValueError("pixel must contain [u, v]")
        u, v = float(pixel[0]), float(pixel[1])
        if not np.isfinite([u, v]).all():
            raise ValueError("pixel coordinates must be finite")
        if self.distortion:
            points = np.array([[[u, v]]], dtype=np.float64)
            normalized = cv2.undistortPoints(
                points, self.matrix, np.asarray(self.distortion, dtype=np.float64)
            )[0, 0]
            return float(normalized[0]), float(normalized[1])
        return (u - self.cx) / self.fx, (v - self.cy) / self.fy


@dataclass(frozen=True)
class Point3D:
    """A point in metres, in the aligned color-camera optical frame."""

    x: float
    y: float
    z: float

    def to_list(self) -> list[float]:
        return [self.x, self.y, self.z]


@dataclass(frozen=True)
class RigidTransform:
    """A calibrated rigid transform mapping source-frame points to target."""

    source_frame: str
    target_frame: str
    rotation: tuple[tuple[float, float, float], ...]
    translation_m: tuple[float, float, float]

    def __post_init__(self) -> None:
        if not self.source_frame or not self.target_frame:
            raise ValueError("source_frame and target_frame must be non-empty")
        rotation = np.asarray(self.rotation, dtype=np.float64)
        translation = np.asarray(self.translation_m, dtype=np.float64)
        if rotation.shape != (3, 3):
            raise ValueError("rotation must be a 3x3 matrix")
        if translation.shape != (3,):
            raise ValueError("translation_m must contain three values")
        if not np.isfinite(rotation).all() or not np.isfinite(translation).all():
            raise ValueError("transform values must be finite")
        if not np.allclose(rotation @ rotation.T, np.eye(3), atol=1e-5):
            raise ValueError("rotation must be orthonormal")
        if not np.isclose(np.linalg.det(rotation), 1.0, atol=1e-5):
            raise ValueError("rotation determinant must be +1")

    def apply(self, point: Point3D) -> Point3D:
        transformed = (
            np.asarray(self.rotation, dtype=np.float64)
            @ np.asarray(point.to_list(), dtype=np.float64)
            + np.asarray(self.translation_m, dtype=np.float64)
        )
        return Point3D(*(float(value) for value in transformed))

    def to_dict(self) -> dict[str, Any]:
        return {
            "source_frame": self.source_frame,
            "target_frame": self.target_frame,
            "rotation": [list(row) for row in self.rotation],
            "translation_m": list(self.translation_m),
        }


def load_rigid_transform(path: Path) -> RigidTransform:
    """Load and validate a source-to-target transform from a JSON file."""
    payload = json.loads(path.expanduser().read_text(encoding="utf-8"))
    try:
        return RigidTransform(
            source_frame=str(payload["source_frame"]),
            target_frame=str(payload["target_frame"]),
            rotation=tuple(
                tuple(float(value) for value in row) for row in payload["rotation"]
            ),
            translation_m=tuple(float(value) for value in payload["translation_m"]),
        )
    except (KeyError, TypeError) as error:
        raise ValueError(f"invalid rigid-transform JSON: {error}") from error


def transform_points(
    points: Mapping[str, Point3D | None], transform: RigidTransform
) -> dict[str, Point3D | None]:
    return {
        name: transform.apply(point) if point is not None else None
        for name, point in points.items()
    }


@dataclass(frozen=True)
class PedalCenter3DResult:
    """3D counterparts of the three actionable 2D pedal keypoints."""

    points_camera: dict[str, Point3D | None]
    invalid_depth_keys: tuple[str, ...]

    @property
    def ready(self) -> bool:
        return self.points_camera.get("operation_point") is not None

    def to_dict(self) -> dict[str, Any]:
        payload = asdict(self)
        payload["points_camera"] = {
            name: point.to_list() if point is not None else None
            for name, point in self.points_camera.items()
        }
        payload["ready"] = self.ready
        payload["frame"] = "aligned_color_camera_optical"
        payload["units"] = "m"
        return payload


def _depth_at_pixel(
    depth_image: np.ndarray,
    pixel: Point2D,
    *,
    depth_scale: float,
    patch_radius: int,
    minimum_depth_m: float,
    maximum_depth_m: float,
) -> float | None:
    if len(pixel) != 2:
        raise ValueError("pixel must contain [u, v]")
    u, v = round(float(pixel[0])), round(float(pixel[1]))
    height, width = depth_image.shape
    if not (0 <= u < width and 0 <= v < height):
        return None
    x1, x2 = max(0, u - patch_radius), min(width, u + patch_radius + 1)
    y1, y2 = max(0, v - patch_radius), min(height, v + patch_radius + 1)
    depth_m = depth_image[y1:y2, x1:x2].astype(np.float64, copy=False) * depth_scale
    valid = depth_m[
        np.isfinite(depth_m)
        & (depth_m >= minimum_depth_m)
        & (depth_m <= maximum_depth_m)
    ]
    return float(np.median(valid)) if valid.size else None


def deproject_pixel(
    pixel: Point2D,
    depth_image: np.ndarray,
    intrinsics: CameraIntrinsics,
    *,
    depth_scale: float,
    patch_radius: int = 2,
    minimum_depth_m: float = 0.05,
    maximum_depth_m: float = 10.0,
) -> Point3D | None:
    """Deproject one color pixel using an aligned raw depth image.

    ``depth_scale`` converts depth-image units to metres.  For a RealSense
    ``z16`` frame, read it from ``device.first_depth_sensor().get_depth_scale``.
    A small patch median makes isolated invalid depth pixels harmless.
    """
    if depth_image.ndim != 2:
        raise ValueError("depth_image must be a single-channel HxW image")
    if depth_image.shape != (intrinsics.height, intrinsics.width):
        raise ValueError(
            "depth image must be aligned to the color image and match its "
            f"intrinsics size {intrinsics.width}x{intrinsics.height}; got "
            f"{depth_image.shape[1]}x{depth_image.shape[0]}"
        )
    if depth_scale <= 0 or not np.isfinite(depth_scale):
        raise ValueError("depth_scale must be a positive finite metres-per-unit value")
    if patch_radius < 0:
        raise ValueError("patch_radius must be non-negative")
    if not 0 < minimum_depth_m <= maximum_depth_m:
        raise ValueError("invalid depth range")

    depth_m = _depth_at_pixel(
        depth_image,
        pixel,
        depth_scale=depth_scale,
        patch_radius=patch_radius,
        minimum_depth_m=minimum_depth_m,
        maximum_depth_m=maximum_depth_m,
    )
    if depth_m is None:
        return None
    ray_x, ray_y = intrinsics.normalized_ray(pixel)
    return Point3D(ray_x * depth_m, ray_y * depth_m, depth_m)


def project_pedal_result(
    pedal_result: Any,
    depth_image: np.ndarray,
    intrinsics: CameraIntrinsics,
    *,
    depth_scale: float,
    patch_radius: int = 2,
) -> PedalCenter3DResult:
    """Project the keypoints returned by ``PedalCenterRecognizer.predict``.

    The returned coordinates are in the color-camera optical frame: ``+x`` to
    the image right, ``+y`` downward, and ``+z`` forward.  Transform them with
    a separately calibrated camera-to-robot extrinsic before commanding a
    robot.
    """
    keypoints: Mapping[str, Point2D | None] = {
        "black_center_point": getattr(pedal_result, "black_center_point", None),
        "operation_diagonal_intersection": getattr(
            pedal_result, "operation_diagonal_intersection", None
        ),
        "operation_point": getattr(pedal_result, "operation_point", None),
    }
    points: dict[str, Point3D | None] = {}
    invalid: list[str] = []
    for name, pixel in keypoints.items():
        point = (
            None
            if pixel is None
            else deproject_pixel(
                pixel,
                depth_image,
                intrinsics,
                depth_scale=depth_scale,
                patch_radius=patch_radius,
            )
        )
        points[name] = point
        if pixel is not None and point is None:
            invalid.append(name)
    return PedalCenter3DResult(points, tuple(invalid))
