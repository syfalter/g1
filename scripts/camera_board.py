"""Experiment B board geometry. No ROS, robot model, camera owner or control I/O."""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np

from vision_task_geometry import depth_to_color_transform

BOARD = dict(label='charuco-200', dimensions=[9, 12], square_m=.015,
             marker_m=.01125, dictionary='DICT_5X5_1000', first_id=0,
             size_source='user_confirmed_board_label', metric_size_measured=False)
LIMITS = dict(min_markers=8, min_corners=12, layout_rms_px=2., layout_gap_px=.5,
              reprojection_rms_px=1.5, ambiguity_gap_px=.1, ambiguity_angle_deg=3.,
              min_depth_m=.1, max_depth_m=3., min_depth_pixels=200,
              min_valid_fraction=.5, roi_inset_squares=.5, roi_erode_px=3,
              heatmap_limit_mm=20.)
FRAME = 'head_color_camera_optical_frame'


def board_for_layout(layout, config=BOARD):
    """Use OpenCV's old interpolator for both checker parities, without a new SDK.

    An extra bottom row changes the top parity of the 4.5 board. Its unprinted
    markers and corners are excluded. Q has origin top-left, X right, Y down.
    """
    cols, rows, parity = (int(layout[k]) for k in ('cols', 'rows', 'parity'))
    if sorted((cols, rows)) != sorted(config['dimensions']) or parity not in (0, 1):
        raise ValueError('板布局与标签尺寸不一致')
    height = rows if rows % 2 == parity else rows + 1
    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, config['dictionary']))
    board = cv2.aruco.CharucoBoard_create(cols, height, config['square_m'], config['marker_m'], dictionary)
    board.setIds(np.arange(config['first_id'], config['first_id']+len(board.ids), dtype=np.int32))
    def canonical(points):
        points = np.array(points, dtype=np.float64, copy=True)
        points[..., 1] = height * config['square_m'] - points[..., 1]
        points[..., 2] *= -1
        return points
    markers = {int(i): canonical(p) for i, p in zip(board.ids, board.objPoints)
               if canonical(p)[:, 1].max() < rows*config['square_m']}
    return board, markers, canonical(board.chessboardCorners)


def detect_board(bgr, locked=None, config=BOARD, limits=LIMITS):
    if bgr is None or bgr.dtype != np.uint8 or bgr.ndim != 3 or bgr.shape[2] != 3:
        raise ValueError('无法读取 RGB 图像')
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, config['dictionary']))
    params = cv2.aruco.DetectorParameters_create()
    # Small ChArUco markers border chess squares. Generic marker subpixel windows
    # can snap to those edges; refine chess intersections with the board API.
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_NONE
    corners, ids, _ = cv2.aruco.detectMarkers(gray, dictionary, parameters=params)
    overlay = bgr.copy()
    result = dict(valid=False, reason='', marker_count=0 if ids is None else len(ids),
                  marker_ids=[] if ids is None else ids.flatten().tolist(), candidates=[])
    if ids is not None:
        cv2.aruco.drawDetectedMarkers(overlay, corners)
    if ids is None or len(ids) < limits['min_markers']:
        result['reason'] = f"5x5 标记不足：{result['marker_count']} / {limits['min_markers']}"
        return result, overlay
    if len(set(ids.flatten().tolist())) != len(ids):
        result['reason'] = '图中存在重复 marker ID；请只保留一块板'
        return result, overlay
    layouts = [locked] if locked else [dict(cols=c, rows=r, parity=p)
        for c, r in (config['dimensions'], config['dimensions'][::-1]) for p in (0, 1)]
    possible = []
    for layout in layouts:
        board, markers, canonical = board_for_layout(layout, config)
        if any(int(i) not in markers for i in ids.flatten()):
            continue
        obj = np.concatenate([markers[int(i)][:, :2] for i in ids.flatten()])
        image = np.concatenate(corners).reshape(-1, 2)
        homography, _ = cv2.findHomography(obj, image, 0)
        if homography is None:
            continue
        predicted = cv2.perspectiveTransform(obj.reshape(-1, 1, 2), homography).reshape(-1, 2)
        rms = float(np.sqrt(np.mean(np.sum((predicted-image)**2, axis=1))))
        possible.append((rms, layout, board, canonical))
    possible.sort(key=lambda x: x[0])
    result['candidates'] = [dict(**v[1], rms_px=v[0]) for v in possible]
    if not possible or possible[0][0] > limits['layout_rms_px']:
        result['reason'] = 'marker ID/空间布局不匹配标签的 9x12 标准板；保留原图，不猜测布局'
        return result, overlay
    if len(possible) > 1 and possible[1][0]-possible[0][0] < limits['layout_gap_px']:
        result['reason'] = '板布局有多个接近的候选；请让更多板面入镜'
        return result, overlay
    rms, layout, board, canonical = possible[0]
    count, charuco, corner_ids = cv2.aruco.interpolateCornersCharuco(corners, ids, gray, board, minMarkers=2)
    if charuco is None:
        result['reason'] = '已检测 marker，但未得到棋盘角点'
        return result, overlay
    obj = canonical[corner_ids.flatten()]
    keep = (obj[:, 1] > 1e-6) & (obj[:, 1] < layout['rows']*config['square_m']-1e-6)
    obj, charuco, corner_ids = obj[keep], charuco[keep], corner_ids[keep]
    cv2.aruco.drawDetectedCornersCharuco(overlay, charuco)
    span = np.ptp(obj[:, :2], axis=0) if len(obj) else np.zeros(2)
    valid = len(obj) >= limits['min_corners'] and np.all(span >= 3*config['square_m']-1e-6)
    result.update(valid=bool(valid), reason='' if valid else '棋盘角点数量或二维覆盖不足',
                  layout=layout, layout_rms_px=rms, corner_count=len(obj),
                  object_points_m=obj.tolist(), image_points_px=charuco.reshape(-1, 2).tolist(),
                  corner_ids=corner_ids.flatten().tolist(),
                  dictionary_note='5x5 家族共享前缀；使用 1000 字典解码，不推断厂家字典容量',
                  board_frame='Q: printed top-left origin; X right, Y down, Z into board')
    return result, overlay


def camera_matrix(intrinsics, shape):
    if (intrinsics.get('height'), intrinsics.get('width')) != tuple(shape[:2]):
        raise ValueError('图像尺寸与本帧内参不匹配')
    values = np.array([intrinsics.get(k, np.nan) for k in ('fx', 'fy', 'cx', 'cy')], float)
    distortion = np.asarray(intrinsics.get('distortion', []), float)
    if not np.isfinite(values).all() or np.any(values[:2] <= 0) or distortion.shape != (5,) or not np.isfinite(distortion).all():
        raise ValueError('本帧内参/畸变字段无效')
    # Existing service serializes five coefficients; nonzero RS distortion needs
    # its exact distortion model, which the old RPC does not fully identify.
    if not np.allclose(distortion, 0, atol=1e-10):
        raise ValueError('当前离线链路仅接受已核实零畸变的 RGB profile')
    fx, fy, cx, cy = values
    return np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1.]]), distortion


def rgb_pose(detection, intrinsics, shape, limits=LIMITS):
    k, distortion = camera_matrix(intrinsics, shape)
    obj = np.asarray(detection['object_points_m'], float)
    image = np.asarray(detection['image_points_px'], float)
    if not detection['valid'] or np.linalg.matrix_rank(obj-obj.mean(axis=0)) < 2:
        raise ValueError('RGB 板检测无效，不能求位姿')
    _, rotations, translations, _ = cv2.solvePnPGeneric(obj, image, k, distortion, flags=cv2.SOLVEPNP_IPPE)
    solutions = []
    for r, t in zip(rotations, translations):
        rotation = cv2.Rodrigues(r)[0]
        if np.min((obj @ rotation.T+t.reshape(3))[:, 2]) <= 0:
            continue
        projected = cv2.projectPoints(obj, r, t, k, distortion)[0].reshape(-1, 2)
        rms = float(np.sqrt(np.mean(np.sum((projected-image)**2, axis=1))))
        if np.isfinite(rms):
            solutions.append((rms, r, t, rotation))
    solutions.sort(key=lambda v: v[0])
    if not solutions or solutions[0][0] > limits['reprojection_rms_px']:
        raise ValueError('RGB PnP 重投影误差过大或没有正深度解')
    if len(solutions) > 1:
        angle = np.degrees(np.arccos(np.clip(np.dot(solutions[0][3][:, 2], solutions[1][3][:, 2]), -1, 1)))
        if solutions[1][0]-solutions[0][0] < limits['ambiguity_gap_px'] and angle > limits['ambiguity_angle_deg']:
            raise ValueError('RGB 平面位姿有两个接近的解；请略微倾斜或靠近板，不用深度选解')
    _, r, t, _ = solutions[0]
    r, t = cv2.solvePnPRefineLM(obj, image, k, distortion, r, t)
    rotation = cv2.Rodrigues(r)[0]
    projected = cv2.projectPoints(obj, r, t, k, distortion)[0].reshape(-1, 2)
    rms = float(np.sqrt(np.mean(np.sum((projected-image)**2, axis=1))))
    if rms > limits['reprojection_rms_px'] or np.min((obj @ rotation.T+t.reshape(3))[:, 2]) <= 0:
        raise ValueError('RGB 位姿细化后无效')
    transform = np.eye(4); transform[:3, :3] = rotation; transform[:3, 3] = t.reshape(3)
    return dict(T_C_Q_row_major=transform.tolist(), reprojection_rms_px=rms,
                tilt_deg=float(np.degrees(np.arccos(np.clip(abs(rotation[2, 2]), 0, 1)))),
                alternatives_rms_px=[v[0] for v in solutions])


def aligned_depth_points(uv, z_depth, intrinsics, extrinsics):
    """Reconstruct RGB rays using the original depth-camera Z retained by align.

    SDK align.cpp copies input Z16, not transformed color Z. For p_C=lambda*r,
    z_D=e3.T*(R_DC*p_C+t_DC), so lambda=(z_D-t_DC.z)/(e3.T*R_DC*r).
    Raster splatting/occlusion remains an approximation; raw depth is not saved.
    """
    uv, z_depth = np.asarray(uv, float), np.asarray(z_depth, float)
    rays = np.column_stack(((uv[:, 0]-intrinsics['cx'])/intrinsics['fx'],
                            (uv[:, 1]-intrinsics['cy'])/intrinsics['fy'], np.ones(len(uv))))
    inverse = np.linalg.inv(depth_to_color_transform(extrinsics))
    denominator = rays @ inverse[2, :3]
    if np.any(denominator <= 1e-6):
        raise ValueError('彩深外参导致退化/反向射线')
    z_color = (z_depth-inverse[2, 3])/denominator
    if not np.isfinite(z_color).all() or np.any(z_color <= 0):
        raise ValueError('彩深换算后存在非正深度')
    return rays*z_color[:, None]


def depth_metrics(depth, intrinsics, scale, extrinsics, detection, pose, limits=LIMITS, config=BOARD):
    if depth.dtype != np.uint16 or depth.ndim != 2:
        raise ValueError('需要原始 uint16 对齐深度 PNG')
    k, distortion = camera_matrix(intrinsics, depth.shape)
    if not np.isfinite(scale) or not 0 < scale < .01:
        raise ValueError('实际深度尺度缺失或无效')
    transform = np.asarray(pose['T_C_Q_row_major'])
    rotation, origin = transform[:3, :3], transform[:3, 3]
    layout = detection['layout']
    w, h = np.array([layout['cols'], layout['rows']])*config['square_m']
    inset = limits['roi_inset_squares']*config['square_m']
    outer = np.array([[0, 0, 0], [w, 0, 0], [w, h, 0], [0, h, 0]], float)
    rv = cv2.Rodrigues(rotation)[0]
    boundary = cv2.projectPoints(outer, rv, origin, k, distortion)[0].reshape(-1, 2)
    if np.any(boundary < 1) or np.any(boundary[:, 0] > depth.shape[1]-2) or np.any(boundary[:, 1] > depth.shape[0]-2):
        raise ValueError('板面不完整入镜；请移回画面内部')
    roi = np.array([[inset, inset, 0], [w-inset, inset, 0], [w-inset, h-inset, 0], [inset, h-inset, 0]])
    polygon = cv2.projectPoints(roi, rv, origin, k, distortion)[0].reshape(-1, 2)
    mask = np.zeros(depth.shape, np.uint8)
    cv2.fillConvexPoly(mask, np.rint(polygon).astype(np.int32), 255)
    covered = np.zeros_like(mask)
    cv2.fillConvexPoly(covered, cv2.convexHull(np.rint(detection['image_points_px']).astype(np.int32)), 255)
    mask &= covered
    erosion = limits['roi_erode_px']
    mask = cv2.erode(mask, np.ones((erosion*2+1, erosion*2+1), np.uint8))
    v, u = np.nonzero(mask)
    z = depth[v, u].astype(float)*scale
    valid = (z >= limits['min_depth_m']) & (z <= limits['max_depth_m'])
    fraction = float(np.mean(valid)) if len(valid) else 0.
    if np.sum(valid) < limits['min_depth_pixels'] or fraction < limits['min_valid_fraction']:
        raise ValueError(f'板内有效深度不足：{np.sum(valid)}/{len(valid)} ({fraction:.1%})')
    uv = np.column_stack((u[valid], v[valid]))
    points = aligned_depth_points(uv, z[valid], intrinsics, extrinsics)
    # Signed normal always faces away from camera, independent of board parity.
    normal = rotation[:, 2].copy()
    if np.dot(normal, origin) < 0:
        normal *= -1
    residual = (points-origin) @ normal
    centered = points-points.mean(axis=0)
    _, _, vt = np.linalg.svd(centered, full_matrices=False)
    angle = float(np.degrees(np.arccos(np.clip(abs(np.dot(vt[-1], normal)), 0, 1))))
    center = rotation @ np.array([w/2, h/2, 0])+origin
    metrics = dict(signed_mean_mm=float(np.mean(residual)*1000),
                   median_absolute_mm=float(np.median(np.abs(residual))*1000),
                   rms_mm=float(np.sqrt(np.mean(residual**2))*1000),
                   p95_absolute_mm=float(np.percentile(np.abs(residual), 95)*1000),
                   normal_difference_deg=angle, valid_fraction=fraction,
                   roi_pixels=len(valid), valid_pixels=int(np.sum(valid)), rejected_pixels=int(np.sum(~valid)),
                   board_center_z_m=float(center[2]), tilt_deg=pose['tilt_deg'],
                   reprojection_rms_px=pose['reprojection_rms_px'],
                   sign='positive = farther along camera-facing-away RGB plane normal',
                   geometry='RGB ray + original depth Z + inverse factory depth-to-color; raster approximation')
    heat = np.full((*depth.shape, 3), 35, np.uint8)
    normalized = np.clip(residual*1000/limits['heatmap_limit_mm'], -1, 1)
    colors = np.full((len(residual), 3), 245., float)
    colors[:, 0] -= np.maximum(normalized, 0)*220
    colors[:, 1] -= np.abs(normalized)*180
    colors[:, 2] -= np.maximum(-normalized, 0)*220
    heat[uv[:, 1], uv[:, 0]] = colors.astype(np.uint8)
    cv2.polylines(heat, [np.rint(polygon).astype(np.int32)], True, (80, 200, 80), 1)
    return metrics, heat


def scan_images(roots):
    seen = set(); matches = []; count = 0
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_1000)
    for root in roots:
        for path in sorted(Path(root).rglob('*.png')):
            if 'rgb' not in path.name.lower() and 'color' not in path.name.lower():
                continue
            sha = hashlib.sha256(path.read_bytes()).hexdigest()
            if sha in seen:
                continue
            seen.add(sha); count += 1
            gray = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
            if gray is None:
                continue
            _, ids, _ = cv2.aruco.detectMarkers(gray, dictionary)
            if ids is not None and len(ids) >= 2:
                matches.append(dict(path=str(path), markers=len(ids), ids=ids.flatten().tolist()))
    return dict(unique_rgb_images=count, matches=matches)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scan', nargs='+', required=True)
    args = parser.parse_args()
    cv2.setNumThreads(1)
    print(json.dumps(scan_images(args.scan), indent=2, ensure_ascii=False))
