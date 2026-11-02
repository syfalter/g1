"""Validate a vision_service capture and build a torso-frame task preview."""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_VENDOR_URDF = Path(
    "/home/syf/unitree_ws/unitree_ros/robots/g1_description/"
    "g1_29dof_lock_waist_rev_1_0.urdf"
)
EXPECTED_CAMERA_FRAME = "head_color_camera_optical_frame"
POINT_KEYS = (
    "black_center_point",
    "operation_diagonal_intersection",
    "operation_point",
)
ENTRY_KEYS = ('operation_diagonal_intersection', 'operation_point')


def ray_plane_intersection(origin, point_on_ray, plane_point, normal):
    values = [np.asarray(value, dtype=float) for value in
              (origin, point_on_ray, plane_point, normal)]
    if any(value.shape != (3,) for value in values) or not np.isfinite(values).all():
        raise ValueError('Invalid ray/plane coordinates')
    origin, point_on_ray, plane_point, normal = values
    direction = point_on_ray-origin
    if np.linalg.norm(direction) < 1e-9 or np.linalg.norm(normal) < 1e-9:
        raise ValueError('Degenerate ray or plane normal')
    direction = direction/np.linalg.norm(direction)
    normal = normal/np.linalg.norm(normal)
    denominator = float(np.dot(normal, direction))
    if abs(denominator) < .05:
        raise ValueError('Camera ray is nearly parallel to the operation plane')
    distance = float(np.dot(normal, plane_point-origin)/denominator)
    if not .1 <= distance <= 10.:
        raise ValueError('Ray-plane intersection is behind camera or outside [0.1,10] m')
    return origin+distance*direction


def _transform(xyz, rpy=(0.0, 0.0, 0.0)):
    result = np.eye(4)
    result[:3, :3] = Rotation.from_euler("xyz", rpy).as_matrix()
    result[:3, 3] = xyz
    return result


def nominal_torso_depth_transform(vendor_urdf=DEFAULT_VENDOR_URDF):
    robot = ET.parse(vendor_urdf).getroot()
    joint = robot.find("joint[@name='d435_joint']")
    if joint is None:
        raise ValueError("Vendor model has no d435_joint")
    origin = joint.find("origin")
    if joint.find("parent").get("link") != "torso_link" or joint.find("child").get("link") != "d435_link":
        raise ValueError("Unexpected d435_joint topology")
    xyz = [float(value) for value in origin.get("xyz").split()]
    rpy = [float(value) for value in origin.get("rpy").split()]
    # RealSense optical +Z is camera-body +X.
    body_to_depth_optical = _transform(
        (0.0, 0.0, 0.0), (-math.pi / 2.0, 0.0, -math.pi / 2.0)
    )
    return _transform(xyz, rpy) @ body_to_depth_optical


def depth_to_color_transform(extrinsics):
    values = np.asarray(extrinsics.get("rotation"), dtype=float)
    translation = np.asarray(extrinsics.get("translation_m"), dtype=float)
    if values.shape != (9,) or translation.shape != (3,) or not np.isfinite(np.r_[values, translation]).all():
        raise ValueError("Invalid depth_to_color_extrinsics dimensions")
    rotation = values.reshape((3, 3), order="F")
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=2e-3) or not np.isclose(
        np.linalg.det(rotation), 1.0, atol=2e-3
    ):
        raise ValueError("depth_to_color rotation is not a proper rotation")
    result = np.eye(4)
    result[:3, :3] = rotation
    result[:3, 3] = translation
    return result


def torso_color_transform(extrinsics, vendor_urdf=DEFAULT_VENDOR_URDF):
    # RealSense provides p_color = T_color_depth * p_depth. The service's 3-D
    # points are already in color optical coordinates, hence the inverse here.
    return nominal_torso_depth_transform(vendor_urdf) @ np.linalg.inv(
        depth_to_color_transform(extrinsics)
    )


def _point(values, name):
    point = np.asarray(values, dtype=float)
    if point.shape != (3,) or not np.isfinite(point).all():
        raise ValueError(f"Missing or invalid 3-D point: {name}")
    if not 0.1 <= point[2] <= 10.0:
        raise ValueError(f"Camera depth outside [0.1, 10] m: {name}")
    return point


def _apply(transform, point):
    return (transform @ np.r_[point, 1.0])[:3]


def _profile_for_observation(intrinsics_file, observation):
    config = json.loads(Path(intrinsics_file).read_text())
    camera = config.get("camera", {})
    if camera.get("serial") != observation.get("camera_serial"):
        raise ValueError(
            f"Camera serial mismatch: file={camera.get('serial')} response={observation.get('camera_serial')}"
        )
    response = observation.get("color_intrinsics", {})
    width = int(observation.get("image_width", 0))
    height = int(observation.get("image_height", 0))
    if (response.get("width"), response.get("height")) != (width, height):
        raise ValueError("Response image size and color intrinsics size differ")
    candidates = [
        profile
        for profile in config.get("profiles", {}).values()
        if profile.get("stream") == "color"
        and (profile.get("width"), profile.get("height")) == (width, height)
    ]
    if len(candidates) != 1:
        raise ValueError(f"No unique color calibration profile for {width}x{height}")
    profile = candidates[0]
    for key in ("fx", "fy", "cx", "cy"):
        value = float(response.get(key, float("nan")))
        if not math.isfinite(value) or abs(value - float(profile[key])) > 2.0:
            raise ValueError(f"Response {key} does not match saved device calibration")
    coefficients = np.asarray(response.get("distortion", []), dtype=float)
    if coefficients.shape != (5,) or not np.isfinite(coefficients).all():
        raise ValueError("Invalid color distortion coefficients")
    if not np.allclose(coefficients, 0.0, atol=1e-10):
        raise ValueError("RGB/3-D pinhole consistency check requires zero distortion")
    return config, profile


def _observation(capture):
    if "observation" in capture:
        if capture.get("status_result") != 0:
            raise ValueError(f"vision_service status failed: {capture.get('status_result')}")
        if capture.get("observation_result") != 0:
            raise ValueError(f"vision_service recognition failed: {capture.get('observation_result')}")
        observation = capture.get("observation")
    else:
        observation = capture
    if not isinstance(observation, dict):
        raise ValueError("Capture has no observation")
    if observation.get("code") != 0 or not observation.get("valid_3d"):
        raise ValueError(
            f"Recognition has no valid 3-D result: code={observation.get('code')}"
        )
    if observation.get("camera_frame") != EXPECTED_CAMERA_FRAME:
        raise ValueError(f"Unexpected camera frame: {observation.get('camera_frame')}")
    if observation.get("units") not in ("m", "meter", "meters"):
        raise ValueError(f"Unexpected point units: {observation.get('units')}")
    return observation


def _pixel_consistency(observation, points):
    intrinsics = observation["color_intrinsics"]
    fx, fy = float(intrinsics["fx"]), float(intrinsics["fy"])
    cx, cy = float(intrinsics["cx"]), float(intrinsics["cy"])
    errors = {}
    for key, point in points.items():
        pixel = np.asarray(observation.get(key), dtype=float)
        if pixel.shape != (2,) or not np.isfinite(pixel).all():
            raise ValueError(f"Missing or invalid RGB keypoint: {key}")
        projected = np.array([fx * point[0] / point[2] + cx, fy * point[1] / point[2] + cy])
        errors[key] = float(np.linalg.norm(projected - pixel))
        if errors[key] > 2.0:
            raise ValueError(f"RGB/3-D inconsistency exceeds 2 px: {key}")
    return errors


def grasp_candidates(entry, fingertip):
    """Geometric candidates only; never an IK solution or a motion command."""
    tilt = math.radians(20.0)
    direction = np.array([math.sin(tilt), 0.0, -math.cos(tilt)])
    x_axis = np.array([0.0, 1.0, 0.0])
    tip = np.eye(4)
    tip[:3, :3] = np.column_stack((x_axis, np.cross(direction, x_axis), direction))
    tip[:3, 3] = entry
    pre_tip = tip.copy()
    pre_tip[:3, 3] -= direction * (0.10 / math.cos(tilt))
    candidates = {}
    for side in ("left", "right"):
        estimate = fingertip["sides"][side]["states"]["open"]
        wrist_from_tip = np.asarray(estimate["wrist_yaw_from_index_tcp"]["matrix_row_major"])
        palm_from_tip = np.asarray(estimate["palm_from_index_tcp"]["matrix_row_major"])
        transforms = {
            "index_tcp": tip,
            "wrist_yaw": tip @ np.linalg.inv(wrist_from_tip),
            "palm": tip @ np.linalg.inv(palm_from_tip),
            "preinsert_index_tcp": pre_tip,
            "preinsert_wrist_yaw": pre_tip @ np.linalg.inv(wrist_from_tip),
        }
        candidates[side] = {
            "preview_only": True,
            "ik_checked": False,
            "collision_checked": False,
            "finger_posture": "open_model_assumption",
            "insertion_tilt_deg": 20.0,
            "orientation_source": "geometric candidate, not measured vision orientation",
            "poses": {key: {
                "frame_id": "torso_link",
                "position_m": value[:3, 3].tolist(),
                "quaternion_xyzw": Rotation.from_matrix(value[:3, :3]).as_quat().tolist(),
                "matrix_row_major": value.tolist(),
            } for key, value in transforms.items()},
        }
    return candidates


def build_task_preview(
    capture,
    intrinsics_file,
    fingertip_file,
    vendor_urdf=DEFAULT_VENDOR_URDF,
    plane_adjust_limit_m=0.03,
    entry_key='operation_diagonal_intersection',
    radial_offset_m=0.0,
    height_offset_m=0.0,
    arc_center_toward_robot_m=0.0,
    calibration=None,
):
    if entry_key not in ENTRY_KEYS:
        raise ValueError('Unsupported insertion reference key')
    if not math.isfinite(radial_offset_m) or abs(radial_offset_m) > .10:
        raise ValueError('Radial offset must be within [-0.10,0.10] m')
    if not math.isfinite(height_offset_m) or not -.03 <= height_offset_m <= .10:
        raise ValueError('Height offset must be within [-0.03,0.10] m')
    if not math.isfinite(plane_adjust_limit_m) or plane_adjust_limit_m <= 0:
        raise ValueError('Invalid plane adjustment limit')
    observation = _observation(capture)
    if capture.get("request_id") is not None and observation.get("request_id") is not None:
        if int(capture["request_id"]) != int(observation["request_id"]):
            raise ValueError("Capture and observation request IDs differ")
    depth_scale = float(observation.get("depth_scale_m_per_unit", float("nan")))
    timestamp_delta = float(observation.get("timestamp_delta_ms", float("nan")))
    if not math.isfinite(depth_scale) or not 0.0 < depth_scale < 0.01:
        raise ValueError("Invalid depth scale")
    if not math.isfinite(timestamp_delta) or timestamp_delta < 0.0:
        raise ValueError("Invalid color/depth timestamp delta")
    intrinsics_config, profile = _profile_for_observation(intrinsics_file, observation)
    camera_points = {
        key: _point(observation.get("points_camera", {}).get(key), key)
        for key in POINT_KEYS
    }
    pixel_errors = _pixel_consistency(observation, camera_points)
    transform = torso_color_transform(observation["depth_to_color_extrinsics"], vendor_urdf)
    nominal_transform=transform.copy()
    if calibration is not None:
        from task_calibration import camera_transform,depth_points
        camera_points={key:depth_points(value,calibration) for key,value in camera_points.items()}
        transform=camera_transform(transform,calibration)
    torso_points = {key: _apply(transform, value) for key, value in camera_points.items()}

    axis = torso_points["black_center_point"]
    operation_raw = torso_points["operation_point"]
    normal = np.array([0.0, 0.0, 1.0])
    plane_delta = float(np.dot(operation_raw - axis, normal))
    # Keep the previous Z-only projection for comparison, never as the new target.
    operation_projected = operation_raw - normal * plane_delta
    selected_raw = torso_points[entry_key]
    selected_plane_delta = float(np.dot(selected_raw-axis, normal))
    origin = transform[:3, 3]
    intersections = {}
    intersection_errors = []
    for key in ENTRY_KEYS:
        try:
            intersections[key] = ray_plane_intersection(origin, torso_points[key], axis, normal)
        except ValueError as error:
            if key == entry_key:
                raise
            intersections[key] = None
            intersection_errors.append(f'{key}: {error}')
    selected_intersection = intersections[entry_key]
    radial = selected_intersection-axis
    original_radius = float(np.linalg.norm(radial))
    if original_radius < 1e-9:
        raise ValueError('Insertion reference coincides with shaft center')
    adjusted_radius = original_radius+radial_offset_m
    if adjusted_radius <= 0:
        raise ValueError('Radial offset reaches or crosses the shaft center')
    radial = radial*(adjusted_radius/original_radius)
    # Offset the operation plane only; measured shaft/base geometry stays fixed.
    operation_center = axis+normal*height_offset_m
    insertion = operation_center+radial
    radius = float(np.linalg.norm(radial))
    geometry_valid = (
        abs(selected_plane_delta) <= plane_adjust_limit_m and 0.005 <= radius <= 0.25
    )
    angle = math.radians(120.0)
    arc = [
        (operation_center + Rotation.from_rotvec(normal * value).apply(radial)).tolist()
        for value in np.linspace(-angle, angle, 121)
    ]
    fingertip = json.loads(Path(fingertip_file).read_text())
    if calibration is not None:
        from task_calibration import fingertips
        fingertip=fingertips(fingertip,calibration)
    review_reasons = list(intersection_errors)
    for key in ("stage1_confidence", "stage2_confidence"):
        confidence = float(observation.get(key, -1.0))
        if not math.isfinite(confidence) or not 0.25 <= confidence <= 1.0:
            review_reasons.append(f"{key}={confidence:.4f}; preview review threshold=0.25")
    if abs(plane_delta) > .003:
        review_reasons.append(f"Raw entry/axis height mismatch {plane_delta * 1000:.1f} mm")
    if abs(selected_plane_delta) > .003:
        review_reasons.append(f'Selected {entry_key} depth/plane mismatch {selected_plane_delta*1000:.1f} mm')
    if timestamp_delta > 50.0:
        review_reasons.append(f"RGB/depth timestamp difference {timestamp_delta:.1f} ms")
    if not geometry_valid:
        review_reasons.append("Plane/radius geometry check failed")
    candidates = grasp_candidates(insertion, fingertip) if geometry_valid else {}
    result = {
        "schema_version": 2,
        "frame_id": "torso_link",
        "hardware_execution_validated": False,
        "preview_only": True,
        "review_reasons": review_reasons,
        "geometry_valid_for_planning_seed": geometry_valid,
        "assumptions": {
            "camera_mount": "Unitree nominal torso->d435 and RealSense body->depth optical",
            "camera_mount_calibrated": False,
            "axis_semantics": "black_center_point provisionally treated as shaft axis center",
            "entry_semantics": f'{entry_key} camera ray intersected with shaft-top horizontal plane',
            "upstream_radial_offset_bypassed": entry_key == 'operation_diagonal_intersection',
            "operation_zone_is_measured_contour": False,
            "shaft_top_normal": "assumed torso +Z; service does not return target orientation",
            "finger_tcp": "community RH56DFX model FK; not measured on hardware",
        },
        "source": {
            "camera_name": observation.get("camera_name"),
            "camera_serial": observation.get("camera_serial"),
            "capture_kind": capture.get("capture_kind", "vision_service"),
            "camera_frame": observation.get("camera_frame"),
            "sequence": observation.get("sequence"),
            "request_id": observation.get("request_id"),
            "captured_at_ms": observation.get("captured_at_ms"),
            "capture_timestamp_note": "server result wall time; device timestamps may use a different clock",
            "intrinsics_queried_at": intrinsics_config["camera"].get("queried_at"),
            "intrinsics_profile": f"color_{profile['width']}x{profile['height']}_{profile['fps']}fps",
            "depth_scale_m_per_unit": depth_scale,
        },
        "quality": {
            "stage1_confidence": observation.get("stage1_confidence"),
            "stage2_confidence": observation.get("stage2_confidence"),
            "rgb_reprojection_error_px": pixel_errors,
            "color_depth_timestamp_delta_ms": timestamp_delta,
            "operation_plane_delta_m": plane_delta,
            "selected_plane_delta_m": selected_plane_delta,
            "entry_radius_before_local_offset_m": original_radius,
            "plane_adjust_limit_m": plane_adjust_limit_m,
            "entry_radius_m": radius,
            "invalid_depth_keys": observation.get("invalid_depth_keys", []),
            "confidence_review_threshold": 0.25,
        },
        "transforms": {
            "torso_from_color_optical_row_major": transform.tolist(),
            "depth_to_color_column_major": observation["depth_to_color_extrinsics"],
        },
        "points_camera_raw_m": {key: value.tolist() for key, value in camera_points.items()},
        "points_torso_raw_m": {key: value.tolist() for key, value in torso_points.items()},
        "task_preview": {
            "axis_center_m": axis.tolist(),
            "operation_point_raw_m": operation_raw.tolist(),
            "operation_point_projected_m": operation_projected.tolist(),
            "legacy_projection_note": 'Old orthogonal Z-only projection; comparison only',
            "entry_source_key": entry_key,
            "projection_method": 'camera_ray_horizontal_plane_intersection',
            "selected_source_point_raw_m": selected_raw.tolist(),
            "ray_plane_intersections_m": {key: point.tolist() if point is not None else None
                                          for key, point in intersections.items()},
            "radial_offset_m": radial_offset_m,
            "radial_offset_sign": 'positive away from shaft center, negative toward center',
            "height_offset_m": height_offset_m,
            "height_offset_sign": 'positive above shaft-top plane along assumed normal',
            "insertion_reference_m": insertion.tolist(),
            "assumed_normal": normal.tolist(),
            "preinsert_point_m": (insertion + normal * 0.10).tolist(),
            "arc_minus120_to_plus120_m": arc,
        },
        "fingertip_estimates": fingertip,
        "grasp_candidates": candidates,
    }
    _set_arc_center(result['task_preview'], arc_center_toward_robot_m)
    if calibration is not None:
        from task_calibration import snapshot
        result['calibration']=snapshot(calibration)
        result['transforms']['uncalibrated_torso_from_color_optical_row_major']=nominal_transform.tolist()
    return result


def _set_arc_center(task, toward_robot_m):
    """Shift the virtual pivot, never the observed base or the insertion targets."""
    if not math.isfinite(toward_robot_m) or not 0 <= toward_robot_m <= .03:
        raise ValueError('Arc center offset must be within [0,0.03] m')
    center = np.asarray(task['axis_center_m'], dtype=float).copy()
    entry = np.asarray(task['insertion_reference_m'], dtype=float)
    center[0] -= toward_robot_m
    center[2] = entry[2]
    radial = entry-center
    radius = float(np.linalg.norm(radial[:2]))
    if toward_robot_m and not .005 <= radius <= .25:
        raise ValueError('Invalid shifted arc radius')
    task.update(arc_center_m=center.tolist(), arc_center_toward_robot_m=toward_robot_m,
                arc_center_offset_frame='capture-time torso -X; frozen with the plan',
                arc_radius_m=radius,
                arc_minus120_to_plus120_m=[
                    (center+Rotation.from_euler('z',a).apply(radial)).tolist()
                    for a in np.linspace(-math.radians(120),math.radians(120),121)])


def with_radial_offset(geometry,offset_m,*,height_offset_m=None,arc_center_toward_robot_m=None):
    """Rebuild from the original intersection, preserving height unless specified."""
    import copy
    if not math.isfinite(offset_m) or abs(offset_m)>.10:raise ValueError('Invalid radial offset')
    result=copy.deepcopy(geometry);task=result['task_preview']
    if height_offset_m is None:height_offset_m=task.get('height_offset_m',0.)
    if not math.isfinite(height_offset_m) or not -.03<=height_offset_m<=.10:raise ValueError('Invalid height offset')
    axis=np.asarray(task['axis_center_m']);source=np.asarray(task['ray_plane_intersections_m'][task['entry_source_key']])
    radial=source-axis;length=float(np.linalg.norm(radial));radius=length+offset_m
    if length<1e-9 or not .005<=radius<=.25:raise ValueError('Invalid adjusted radius')
    radial*=radius/length;normal=np.asarray(task['assumed_normal'])
    operation_center=axis+normal*height_offset_m;entry=operation_center+radial
    task.update(radial_offset_m=offset_m,height_offset_m=height_offset_m,
        height_offset_sign='positive above shaft-top plane along assumed normal',insertion_reference_m=entry.tolist(),
        preinsert_point_m=(entry+normal*.10).tolist(),
        arc_minus120_to_plus120_m=[(operation_center+Rotation.from_rotvec(normal*a).apply(radial)).tolist()
                                 for a in np.linspace(-math.radians(120),math.radians(120),121)])
    result['quality']['entry_radius_m']=radius
    result['grasp_candidates']=grasp_candidates(entry,result['fingertip_estimates']) if result['geometry_valid_for_planning_seed'] else {}
    if arc_center_toward_robot_m is None:
        arc_center_toward_robot_m=task.get('arc_center_toward_robot_m',0.)
    _set_arc_center(task,arc_center_toward_robot_m)
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("capture", type=Path)
    parser.add_argument(
        "--intrinsics",
        type=Path,
        default=Path("/home/syf/unitree_ws/head_camera_intrinsics.json"),
    )
    parser.add_argument(
        "--fingertips",
        type=Path,
        default=ROOT / "assets/workbench/rh56dfx/index_tcp_estimates.json",
    )
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument('--entry-key', choices=ENTRY_KEYS, default=ENTRY_KEYS[0])
    parser.add_argument('--radial-offset-m', type=float, default=0.)
    parser.add_argument('--height-offset-m', type=float, default=0.)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError(f"Refusing to overwrite {args.output}")
    try:
        result = build_task_preview(
            json.loads(args.capture.read_text()), args.intrinsics, args.fingertips,
            entry_key=args.entry_key, radial_offset_m=args.radial_offset_m,height_offset_m=args.height_offset_m,
        )
    except (OSError, ValueError, KeyError, TypeError, json.JSONDecodeError) as error:
        parser.exit(2, f"vision task input rejected: {error}\n")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2) + "\n")
    print(
        f"wrote {args.output}; seed_valid={result['geometry_valid_for_planning_seed']} "
        f"radius={result['quality']['entry_radius_m']:.4f}m "
        f"selected_plane_delta={result['quality']['selected_plane_delta_m']:.4f}m "
        f"entry_key={args.entry_key} radial_offset={args.radial_offset_m:.4f}m height_offset={args.height_offset_m:.4f}m"
    )


if __name__ == "__main__":
    main()
