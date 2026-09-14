# -*- coding: utf-8 -*-
"""外参标定核心：地面摆位、H 求解(180° 歧义消除)、H-QC、连拍均值、接缝诊断、
solvePnP 6DoF。算法与阈值逐段端口自参考项目 avm/calibrate_extrinsics.py。

坐标系约定：
- 车体系 (gx, gy)：x 右、y 前，原点=车体中心地面投影。
- base_link (ROS)：x 前、y 左、z 上，z=0 为地面；换算 (ros_x, ros_y)=(gy,-gx)。
- BEV 画布：scale px/m，车心=画布中心，画布 y 向上(行号减小)。
- 存盘 H：去畸变图坐标 -> BEV 画布像素。
"""
from __future__ import annotations

import json
import os
import shutil
from datetime import datetime
from pathlib import Path

import numpy as np
import cv2

from j501_avm_calib.config import CAM_AXIS


# ---------------- 摆位 -> 地面坐标 ----------------

def board_dims(cols, rows, square) -> tuple[float, float]:
    """板物理尺寸：长边=(cols+1)*s，短边=(rows+1)*s。"""
    return (cols + 1) * float(square), (rows + 1) * float(square)


def ground_corners(direction, near, lateral, orient, cols, rows, square):
    """4 个最外层内角点（车体系米，CCW）。

    near   : 板近边物理边缘到车体中心距离(沿视线, >0)
    lateral: 板中心横向偏移(+右)
    orient : 'long-lateral'(长边横向,默认) | 'long-along'
    内角点从板物理边缘各缩进 1 格。
    """
    long_m, short_m = board_dims(cols, rows, square)
    S = float(square)
    if orient == "long-lateral":
        w_lat, w_dep = long_m, short_m
    else:
        w_lat, w_dep = short_m, long_m
    half_lat = w_lat / 2.0
    u0 = float(lateral) - half_lat + S
    u1 = float(lateral) + half_lat - S
    v0 = float(near) + S
    v1 = float(near) + w_dep - S
    local = [(u0, v0), (u1, v0), (u1, v1), (u0, v1)]
    ax, ay = CAM_AXIS[direction]
    hx, hy = -ay, ax  # 横向单位向量 = 视线逆时针转 90°
    corners = [(u * hx + v * ax, u * hy + v * ay) for (u, v) in local]
    return corners, (w_lat, w_dep)


def grid_outer_idx(cols, rows):
    """栅格 4 外角点 row-major 下标：TL,TR,BR,BL（检测序成环）。"""
    return [0, cols - 1, rows * cols - 1, (rows - 1) * cols]


def bilinear_ground(g_ll, g_lr, g_ul, g_br, cols, rows):
    """4 外角点(地面)双线性插值全部内角点地面坐标。"""
    g_ll = np.asarray(g_ll, dtype=np.float64)
    g_lr = np.asarray(g_lr, dtype=np.float64)
    g_ul = np.asarray(g_ul, dtype=np.float64)
    g_br = np.asarray(g_br, dtype=np.float64)
    pts = np.zeros((rows, cols, 2), np.float64)
    for r in range(rows):
        for c in range(cols):
            u = c / (cols - 1) if cols > 1 else 0.0
            v = r / (rows - 1) if rows > 1 else 0.0
            pts[r, c] = (g_ll * (1 - u) * (1 - v) + g_lr * u * (1 - v)
                         + g_ul * (1 - u) * v + g_br * u * v)
    return pts.reshape(-1, 2)


def ground_to_canvas(gx, gy, scale, cx, cy):
    """车体系地面坐标(米) -> BEV 画布像素。y 前 = 画布向上。"""
    return np.float32([gx * scale + cx, -gy * scale + cy])


# ---------------- 角点对齐 / 连拍均值 ----------------

def _corner_mean_dist(a: np.ndarray, b: np.ndarray) -> float:
    return float(np.mean(np.linalg.norm(np.asarray(a, dtype=np.float64)
                                        - np.asarray(b, dtype=np.float64),
                                        axis=1)))


def align_corners_to_ref(corners: np.ndarray, ref: np.ndarray,
                         cols: int, rows: int):
    """把一帧角点序对齐到参考帧（处理 180° 翻转/反向枚举）。"""
    c = np.asarray(corners, dtype=np.float64).reshape(-1, 2)
    r = np.asarray(ref, dtype=np.float64).reshape(-1, 2)
    if c.shape != r.shape:
        return None, float("inf")
    candidates = [c, c[::-1]]
    grid = c.reshape(rows, cols, 2)
    rot180 = np.flip(np.flip(grid, 0), 1).reshape(-1, 2)
    candidates.append(rot180)
    candidates.append(rot180[::-1])
    best, best_d = None, float("inf")
    for cand in candidates:
        d = _corner_mean_dist(cand, r)
        if d < best_d:
            best, best_d = cand, d
    return best, best_d


def _seam_order_candidates(corners: np.ndarray, cols: int, rows: int):
    """接缝诊断角点序候选：原序/反向/180°/180°反向。"""
    c = np.asarray(corners, dtype=np.float64).reshape(-1, 2)
    grid = c.reshape(rows, cols, 2)
    rot180 = np.flip(np.flip(grid, 0), 1).reshape(-1, 2)
    return [c, c[::-1], rot180, rot180[::-1]]


def average_corners(corners_list, cols: int, rows: int, *,
                    outlier_rms_px: float = 2.5,
                    align_max_px: float = 40.0):
    """多帧角点对齐后取均值，剔除离群帧。

    返回 (mean_corners (N,2), n_used, stats)。
    """
    if not corners_list:
        raise ValueError("corners_list 为空")
    ref = np.asarray(corners_list[0], dtype=np.float64).reshape(-1, 2)
    aligned = [ref]
    rejected_align = 0
    for c in corners_list[1:]:
        a, dist = align_corners_to_ref(c, ref, cols, rows)
        if a is None or dist > align_max_px:
            rejected_align += 1
            continue
        aligned.append(a)
    stack = np.stack(aligned, axis=0)
    median = np.median(stack, axis=0)
    kept = []
    frame_rms = []
    for a in aligned:
        rms = float(np.sqrt(np.mean(np.sum((a - median) ** 2, axis=1))))
        frame_rms.append(rms)
        if rms <= outlier_rms_px * 3.0:
            kept.append(a)
    if len(kept) < max(2, (len(aligned) + 1) // 2):
        kept = aligned
    mean = np.mean(np.stack(kept, axis=0), axis=0)
    stats = {
        "n_input": len(corners_list),
        "n_aligned": len(aligned),
        "n_used": len(kept),
        "rejected_align": rejected_align,
        "frame_rms_px": frame_rms,
        "mean_frame_rms_px": float(np.mean(frame_rms)) if frame_rms else None,
        "corner_jitter_px": float(np.mean(np.std(np.stack(kept, axis=0),
                                                 axis=0))),
    }
    return mean.astype(np.float64), len(kept), stats


# ---------------- H 求解 ----------------

def _project_corners_h(corners: np.ndarray, H) -> np.ndarray:
    pts = np.asarray(corners, dtype=np.float32).reshape(-1, 1, 2)
    return cv2.perspectiveTransform(pts, np.asarray(H, dtype=np.float64)
                                    ).reshape(-1, 2)


def _reproj_rms(corners: np.ndarray, targets: np.ndarray, H) -> float:
    pred = _project_corners_h(corners, H)
    tgt = np.asarray(targets, dtype=np.float64).reshape(-1, 2)
    return float(np.sqrt(np.mean(np.sum((pred - tgt) ** 2, axis=1))))


def best_homography(corners, ground4, cols, rows, scale, canvas):
    """全部角点 RANSAC 求 H（端口参考实现，含 180° 歧义消除）。

    corners: 检测角点 row-major (N,2)
    ground4: 4 外角点车体系坐标 CCW（placement 生成）
    返回 (H, rms) 或 (None, None)。
    """
    cw, ch = canvas
    cx, cy = cw / 2.0, ch / 2.0
    g4 = np.asarray(ground4, np.float64)
    outer_idx = grid_outer_idx(cols, rows)
    det_outer = np.asarray(corners, np.float64)[outer_idx]
    grid_pos = [(0, 0), (0, cols - 1), (rows - 1, cols - 1), (rows - 1, 0)]

    # 规则棋盘 4 个旋转假设重投影 RMS 都极小，不能靠 RMS 判方向：
    # 先用长短边比排 90° 错解，再用近边透视规律定 180°。
    det_edge_len = np.array(
        [np.linalg.norm(det_outer[(i + 1) % 4] - det_outer[i])
         for i in range(4)], dtype=np.float64)
    ground_edge_len = np.array(
        [np.linalg.norm(g4[(i + 1) % 4] - g4[i]) for i in range(4)],
        dtype=np.float64)
    grid_ratio = (cols - 1) / max(rows - 1, 1)
    dim_error = []
    for k in range(4):
        assigned_ratio = ground_edge_len[k] / max(
            ground_edge_len[(k + 1) % 4], 1e-12)
        dim_error.append(abs(np.log(assigned_ratio / grid_ratio)))
    min_dim_error = min(dim_error)
    dimension_valid = [k for k, err in enumerate(dim_error)
                       if err <= min_dim_error + 1e-6]
    ground_edge_mid_norm = [
        np.linalg.norm((g4[i] + g4[(i + 1) % 4]) * 0.5) for i in range(4)]
    near_ground_edge = int(np.argmin(ground_edge_mid_norm))

    def near_edge_score(k):
        i_near = (near_ground_edge - k) % 4
        i_far = (i_near + 2) % 4
        return float(det_edge_len[i_near] - det_edge_len[i_far])

    candidate_ks = [max(dimension_valid, key=near_edge_score)]

    grid_all = np.zeros((rows * cols, 2), dtype=np.float64)
    for r in range(rows):
        for c in range(cols):
            grid_all[r * cols + c] = [c, r]

    corners64 = np.asarray(corners, dtype=np.float64).reshape(-1, 2)
    best = None
    best_inlier = -1.0
    for k in candidate_ks:
        order = [(k + i) % 4 for i in range(4)]
        g_assign = [g4[order[i]] for i in range(4)]
        gmap = {grid_pos[i]: g_assign[i] for i in range(4)}
        full_ground = bilinear_ground(
            gmap[(0, 0)], gmap[(0, cols - 1)],
            gmap[(rows - 1, 0)], gmap[(rows - 1, cols - 1)], cols, rows)
        canvas_pts = np.float32(
            [ground_to_canvas(g[0], g[1], scale, cx, cy) for g in full_ground])
        H, mask = cv2.findHomography(
            corners64, canvas_pts, cv2.RANSAC, ransacReprojThreshold=3.0,
            maxIters=2000, confidence=0.995)
        if H is None:
            continue
        inlier_mask = (mask.ravel().astype(bool) if mask is not None
                       else np.ones(len(corners64), dtype=bool))
        inlier_c = corners64[inlier_mask]
        inlier_t = canvas_pts[inlier_mask]
        if len(inlier_c) < 4:
            continue
        pred = cv2.perspectiveTransform(
            inlier_c.reshape(-1, 1, 2), H).reshape(-1, 2)
        rms = float(np.sqrt(np.mean(np.sum((pred - inlier_t) ** 2, axis=1))))
        inlier_ratio = len(inlier_c) / len(corners64)
        if inlier_ratio < 0.5:
            continue
        if (best is None or inlier_ratio > best_inlier + 1e-9
                or (abs(inlier_ratio - best_inlier) < 1e-9 and rms < best[1])):
            best = (H, rms)
            best_inlier = inlier_ratio

    # 回退：RANSAC 全失败 -> 4 点 getPerspectiveTransform
    if best is None:
        for k in candidate_ks:
            order = [(k + i) % 4 for i in range(4)]
            g_assign = [g4[order[i]] for i in range(4)]
            img_pts = np.float32([det_outer[i] for i in range(4)])
            canvas_pts_4 = np.float32(
                [ground_to_canvas(g[0], g[1], scale, cx, cy)
                 for g in g_assign])
            H = cv2.getPerspectiveTransform(img_pts, canvas_pts_4)
            gmap = {grid_pos[i]: g_assign[i] for i in range(4)}
            full_ground = bilinear_ground(
                gmap[(0, 0)], gmap[(0, cols - 1)],
                gmap[(rows - 1, 0)], gmap[(rows - 1, cols - 1)], cols, rows)
            exp = np.float32(
                [ground_to_canvas(g[0], g[1], scale, cx, cy)
                 for g in full_ground])
            pred = cv2.perspectiveTransform(
                corners64.reshape(-1, 1, 2), H).reshape(-1, 2)
            rms = float(np.sqrt(np.mean(np.sum((pred - exp) ** 2, axis=1))))
            if best is None or rms < best[1]:
                best = (H, rms)
    if best is None:
        return None, None
    return np.asarray(best[0], dtype=np.float64), float(best[1])


def quality_label(rms):
    if rms < 1.0:
        return "✅ <1px 合格"
    if rms < 5.0:
        return "⚠️ 1~5px 可用，建议检查 placement 距离/板是否平贴地面"
    return "❌ ≥5px 不准：检查 near_m / 板轴对齐 / 内参是否合格"


# ---------------- H 质量控制 ----------------

def board_quad_metrics(corners, cols, rows):
    """棋盘外框几何：对边比、边长，用于发现翘曲/严重倾斜。"""
    outer = np.asarray(corners, dtype=np.float64)[grid_outer_idx(cols, rows)]
    top = float(np.linalg.norm(outer[1] - outer[0]))
    bottom = float(np.linalg.norm(outer[2] - outer[3]))
    left = float(np.linalg.norm(outer[3] - outer[0]))
    right = float(np.linalg.norm(outer[2] - outer[1]))
    edge_ratio = (max(top, bottom, left, right)
                  / max(min(top, bottom, left, right), 1e-6))
    return {"edge_ratio": edge_ratio, "top_px": top, "bottom_px": bottom,
            "left_px": left, "right_px": right}


def analyze_homography(H, direction, img_size, canvas,
                       board_metrics=None, svd_min_thresh=0.03,
                       edge_span_min=25.0, center_tol=350.0,
                       center_flip_tol=50.0, board_edge_ratio_max=1.35):
    """单路 H 质量分析：SVD 条件数、BEV 位移模长、中心落点、翻转判定。

    注意：跨度必须取位移模长（侧向相机图像水平轴映射到 BEV 垂直方向，
    只看 x 分量的旧判据会把正常 H 恒误判成压扁——参考项目 4.5 教训）。
    """
    iw, ih = img_size
    cw, ch = canvas
    cx_canvas, cy_canvas = cw / 2.0, ch / 2.0
    A = np.asarray(H[:2, :2], dtype=np.float64)
    _, s, _ = np.linalg.svd(A)
    s_max, s_min = float(s[0]), float(s[1])
    cond = s_max / max(s_min, 1e-9)
    sample = np.float32([
        [iw / 2.0, ih / 2.0],
        [0.0, ih / 2.0], [iw - 1.0, ih / 2.0],
        [iw / 2.0, 0.0], [iw / 2.0, ih - 1.0],
    ])
    warped = cv2.perspectiveTransform(sample.reshape(-1, 1, 2), H
                                      ).reshape(-1, 2)
    center_bev = warped[0]
    h_span = float(np.linalg.norm(warped[2] - warped[1]))
    v_span = float(np.linalg.norm(warped[4] - warped[3]))
    center_err = float(np.linalg.norm(center_bev - [cx_canvas, cy_canvas]))
    ax, ay = CAM_AXIS[direction]
    along = float((center_bev[0] - cx_canvas) * ax
                  + (cy_canvas - center_bev[1]) * ay)
    warnings = []
    failures = []
    if s_min < svd_min_thresh:
        warnings.append(
            f"σ₂={s_min:.4f}<{svd_min_thresh}: 透视较强，建议结合 BEV 预览复核")
    if h_span < edge_span_min:
        failures.append(f"水平跨度 {h_span:.0f}px<{edge_span_min:.0f}: 投影退化")
    if v_span < edge_span_min:
        failures.append(f"垂直跨度 {v_span:.0f}px<{edge_span_min:.0f}: 投影退化")
    if along < -center_flip_tol:
        # 单块棋盘只约束局部平面。把远离棋盘的鱼眼光心外推到 BEV 后，
        # 侧向相机很容易落到“反方向”；这不是可作为硬拒绝的方向证据。
        # 保留为需人工查看 BEV 的警告，避免丢掉角点/RMS 都合格的候选。
        warnings.append(
            f"图像中心外推到 {direction} 反方向 ({along:.0f}px)："
            "侧向鱼眼的远距离外推不可靠，请查看 BEV 后确认保存")
    elif along <= 0:
        warnings.append(
            f"图像中心几乎贴着车心 ({along:.0f}px): 方向存疑，请核对 placement")
    elif center_err > center_tol:
        warnings.append(
            f"光心落点距画布中心 {center_err:.0f}px>{center_tol:.0f}")
    if board_metrics and board_metrics["edge_ratio"] > board_edge_ratio_max:
        warnings.append(
            f"棋盘外框对边比 {board_metrics['edge_ratio']:.2f}"
            f">{board_edge_ratio_max}: 板可能翘曲/放桌上/朝向不对")
    if failures:
        status = "bad"
    elif warnings:
        status = "warn"
    else:
        status = "ok"
    return {
        "status": status,
        "svd_max": s_max,
        "svd_min": s_min,
        "svd_cond": cond,
        "h00": float(H[0, 0]),
        "h10": float(H[1, 0]),
        "bev_h_span_px": h_span,
        "bev_v_span_px": v_span,
        "center_offset_px": center_err,
        "center_along_axis_px": along,
        "board_edge_ratio": (None if not board_metrics
                             else board_metrics["edge_ratio"]),
        "failures": failures,
        "warnings": warnings,
    }


def homography_qc_label(qc):
    if qc["status"] == "ok":
        return "✅ H 几何正常"
    if qc["status"] == "warn":
        return "⚠️ H 有疑点"
    return "❌ H 病态"


def homography_matches_saved_qc(H, direction, img_size, canvas, saved_qc,
                                relative_tol=0.03):
    """确认 H 仍对应保存时的 QC 快照，发现静默改写或文件损坏。"""
    if not saved_qc:
        return {"ok": False, "reason": "缺少 H 保存时的 QC 快照"}
    actual = analyze_homography(H, direction, img_size, canvas)
    checks = ("svd_max", "svd_min", "bev_h_span_px", "bev_v_span_px",
              "center_offset_px", "center_along_axis_px")
    mismatches = []
    for key in checks:
        if saved_qc.get(key) is None:
            continue
        expected = float(saved_qc[key])
        observed = float(actual[key])
        tolerance = max(1e-4, abs(expected) * float(relative_tol))
        if abs(observed - expected) > tolerance:
            mismatches.append(
                f"{key}={observed:.4g}，保存值={expected:.4g}")
    for key in ("h00", "h10"):
        if saved_qc.get(key) is not None and abs(
                float(actual[key]) - float(saved_qc[key])) > 1e-6:
            mismatches.append(
                f"{key}={actual[key]:.6g}，保存值={float(saved_qc[key]):.6g}")
    return {"ok": not mismatches, "actual": actual,
            "reason": ("H 与保存 QC 一致" if not mismatches else
                       "H 已在 QC 之后被改写: " + "；".join(mismatches[:3]))}


def _principal_sigma(H):
    return float(np.linalg.svd(np.asarray(H, dtype=np.float64)[:2, :2],
                               compute_uv=False)[0])


def check_lr_symmetry(homographies, lr_sigma_ratio_max=4.0):
    """左右相机整体缩放应同量级（比主奇异值，与坐标轴朝向无关）。"""
    if "left" not in homographies or "right" not in homographies:
        return []
    s_l = _principal_sigma(homographies["left"])
    s_r = _principal_sigma(homographies["right"])
    lo, hi = min(s_l, s_r), max(s_l, s_r)
    ratio = hi / max(lo, 1e-9)
    if ratio <= lr_sigma_ratio_max:
        return []
    return [f"左/右 H 主奇异值比={ratio:.1f} (left={s_l:.4f}, "
            f"right={s_r:.4f})>{lr_sigma_ratio_max}: 摆放或视角明显不对称"]


# ---------------- 位姿恢复 / 接缝诊断 ----------------

def homography_from_pose(pose, new_K, scale, vehicle_center):
    """由已保存的相机位姿重建 undistorted pixel -> BEV canvas H。

    pose 约定 X_optical = R @ X_base_link + t；地面为 base_link z=0。
    base_link(x前,y左) 到 BEV(canvas x右,y上) 的符号转换在 A 中完成。
    """
    R = np.asarray(pose["R"], dtype=np.float64).reshape(3, 3)
    t = np.asarray(pose["t"], dtype=np.float64).reshape(3)
    K = np.asarray(new_K, dtype=np.float64).reshape(3, 3)
    cx, cy = (float(value) for value in vehicle_center)
    s = float(scale)
    ground_to_canvas_m = np.array(
        [[0.0, -s, cx], [-s, 0.0, cy], [0.0, 0.0, 1.0]],
        dtype=np.float64)
    ground_to_image = K @ np.column_stack((R[:, 0], R[:, 1], t))
    if abs(np.linalg.det(ground_to_image)) < 1e-12:
        raise ValueError("相机位姿无法生成有效地面单应矩阵")
    H = ground_to_canvas_m @ np.linalg.inv(ground_to_image)
    if not np.isfinite(H).all() or abs(H[2, 2]) < 1e-12:
        raise ValueError("位姿重建 H 非有限或退化")
    return H / H[2, 2]


def homography_pose_consistency(H, H_pose, img_size, canvas,
                                median_limit_px=25.0, max_limit_px=100.0):
    """在 pose-H 可见的 BEV 网格上比较候选 H，防止局部拟合破坏全局。"""
    iw, ih = (int(img_size[0]), int(img_size[1]))
    cw, ch = (int(canvas[0]), int(canvas[1]))
    xs = np.linspace(cw * 0.1, cw * 0.9, 9)
    ys = np.linspace(ch * 0.1, ch * 0.9, 9)
    expected = np.float32([(x, y) for y in ys for x in xs])
    src = _project_corners_h(expected, np.linalg.inv(H_pose))
    visible = (np.isfinite(src).all(axis=1)
               & (src[:, 0] >= 0) & (src[:, 0] < iw)
               & (src[:, 1] >= 0) & (src[:, 1] < ih))
    if int(visible.sum()) < 6:
        return {"ok": False, "sample_count": int(visible.sum()),
                "median_error_px": None, "max_error_px": None,
                "reason": "位姿参考的有效全局采样点不足"}
    predicted = _project_corners_h(src[visible], H)
    errors = np.linalg.norm(predicted - expected[visible], axis=1)
    finite = np.isfinite(errors)
    if int(finite.sum()) < 6:
        return {"ok": False, "sample_count": int(finite.sum()),
                "median_error_px": None, "max_error_px": None,
                "reason": "候选 H 的全局投影退化"}
    median = float(np.median(errors[finite]))
    maximum = float(np.max(errors[finite]))
    ok = median <= float(median_limit_px) and maximum <= float(max_limit_px)
    return {"ok": ok, "sample_count": int(finite.sum()),
            "median_error_px": median, "max_error_px": maximum,
            "reason": ("全局几何一致" if ok else
                       f"偏离位姿参考: median={median:.1f}px, max={maximum:.1f}px")}


def evaluate_seam_alignment(corners_ref, corners_slave, H_ref, H_slave,
                            cols: int, rows: int):
    """只读评估两路棋盘在 BEV 中的对齐误差，绝不生成替换 H。"""
    ref = np.asarray(corners_ref, dtype=np.float64).reshape(-1, 2)
    if ref.shape[0] != cols * rows:
        return {"error": f"参考路角点数 {ref.shape[0]} != {cols*rows}"}
    P_ref = _project_corners_h(ref, H_ref)
    best = None
    for i, cand in enumerate(_seam_order_candidates(corners_slave, cols, rows)):
        if cand.shape[0] != cols * rows:
            continue
        projected = _project_corners_h(cand, H_slave)
        errors = np.linalg.norm(projected - P_ref, axis=1)
        rms = float(np.sqrt(np.mean(errors ** 2)))
        if best is None or rms < best[0]:
            best = (rms, i, errors, projected)
    if best is None:
        return {"error": "两路角点无法对齐"}
    rms, order_i, errors, P_slave = best
    ref_center, slave_center = P_ref.mean(axis=0), P_slave.mean(axis=0)
    center_delta = float(np.linalg.norm(ref_center - slave_center))
    ref_span = float(np.linalg.norm(P_ref.max(axis=0) - P_ref.min(axis=0)))
    slave_span = float(np.linalg.norm(P_slave.max(axis=0) - P_slave.min(axis=0)))
    span = ref_span
    scale_ratio = max(ref_span, slave_span) / max(min(ref_span, slave_span), 1e-6)
    same_target_limit = max(30.0, 1.5 * max(ref_span, slave_span))
    same_target = center_delta <= same_target_limit
    # Center/scale normalisation separates a true shape mismatch from a uniform
    # translation.  It is diagnostic only; this function never changes H.
    ref_shape = P_ref - ref_center
    slave_shape = P_slave - slave_center
    ref_norm = max(float(np.sqrt(np.mean(np.sum(ref_shape ** 2, axis=1)))), 1e-6)
    slave_norm = max(float(np.sqrt(np.mean(np.sum(slave_shape ** 2, axis=1)))), 1e-6)
    shape_rms = float(np.sqrt(np.mean(np.sum(
        (ref_shape / ref_norm - slave_shape / slave_norm) ** 2, axis=1))) * ref_norm)
    if not same_target:
        status, reason_code = "invalid", "different_target"
    elif rms <= 2.0:
        status, reason_code = "ok", "aligned"
    elif rms <= 5.0:
        status, reason_code = "warn", "alignment_warning"
    else:
        status, reason_code = "bad", "alignment_blocked"
    reasons = []
    if span < 30.0:
        reasons.append(f"棋盘在 BEV 中跨度仅 {span:.1f}px，不能约束全局外参")
    if not same_target:
        reasons.append("两路棋盘投影中心相距 %.1fpx（阈值 %.1fpx），可能检测到不同棋盘；"
                       "请移走多余棋盘后重试当前相机对" % (center_delta, same_target_limit))
    elif rms > 5.0 or scale_ratio > 1.35:
        reasons.append("同一棋盘对齐不合格（RMS %.1fpx，尺寸比 %.2f）；"
                       "请转入多位置外参，不能用局部棋盘重写完整单应矩阵" %
                       (rms, scale_ratio))
    elif rms > 2.0:
        reasons.append(f"同一棋盘对齐 RMS={rms:.1f}px，允许内容接缝优化，但建议后续多位置核验")
    return {"status": status, "rms_px": rms,
            "mean_error_px": float(np.mean(errors)),
            "max_error_px": float(np.max(errors)),
            "order_index": int(order_i), "board_span_bev_px": span,
            "ref_board_span_bev_px": ref_span,
            "slave_board_span_bev_px": slave_span,
            "ref_center_bev_px": [float(x) for x in ref_center],
            "slave_center_bev_px": [float(x) for x in slave_center],
            "center_delta_px": center_delta,
            "same_target_limit_px": same_target_limit,
            "same_target": same_target,
            "scale_ratio": scale_ratio,
            "shape_rms_px": shape_rms,
            "reason_code": reason_code,
            "reason": "；".join(reasons) if reasons else "两路局部对齐正常"}

def refine_seam_homography(corners_ref, corners_slave, H_ref, H_slave_old,
                           cols: int, rows: int):
    """兼容旧调用，但安全拒绝从单块局部棋盘重写完整 H。"""
    stats = evaluate_seam_alignment(
        corners_ref, corners_slave, H_ref, H_slave_old, cols, rows)
    stats["error"] = "已禁用不安全的局部全 H 重拟合；请使用接缝诊断"
    return None, stats


# ---------------- 6DoF（solvePnP） ----------------

def analyze_camera_pose(pose, direction, pose_qc=None):
    """Check that a planar-PnP pose matches an outward-facing AVM camera.

    Planar targets admit a mirrored solution with almost identical reprojection
    error.  Height alone cannot reject it: the mirrored camera can be above the
    ground while sitting beyond the board and looking back toward the vehicle.
    """
    cfg = dict(pose_qc or {})
    height_min = float(cfg.get("height_min_m", 0.05))
    height_max = float(cfg.get("height_max_m", 0.60))
    side_min = float(cfg.get("side_min_m", 0.05))
    side_max = float(cfg.get("side_max_m", 0.65))
    cross_max = float(cfg.get("cross_axis_max_m", 0.45))
    optical_dot_min = float(cfg.get("optical_axis_dot_min", 0.25))

    R = np.asarray(pose["R"], dtype=np.float64).reshape(3, 3)
    t = np.asarray(pose["t"], dtype=np.float64).reshape(3)
    center = -R.T @ t
    optical = R.T @ np.array([0.0, 0.0, 1.0], dtype=np.float64)
    ax, ay = CAM_AXIS[direction]
    # Vehicle (right, forward) -> ROS base_link (forward, left).
    axis = np.array([ay, -ax, 0.0], dtype=np.float64)
    along = float(center @ axis)
    cross = float(np.linalg.norm(center[:2] - along * axis[:2]))
    optical_xy = optical[:2]
    optical_norm = float(np.linalg.norm(optical_xy))
    optical_dot = (float(optical_xy @ axis[:2]) / optical_norm
                   if optical_norm > 1e-9 else -1.0)
    height = float(center[2])
    failures = []
    if not height_min <= height <= height_max:
        failures.append(
            f"相机高度 {height:.3f}m 不在 [{height_min:.2f},{height_max:.2f}]m")
    if not side_min <= along <= side_max:
        failures.append(
            f"相机沿 {direction} 安装位置 {along:.3f}m 不在 "
            f"[{side_min:.2f},{side_max:.2f}]m")
    if cross > cross_max:
        failures.append(f"相机偏离 {direction} 安装轴 {cross:.3f}m>{cross_max:.2f}m")
    if optical_dot < optical_dot_min:
        failures.append(
            f"相机光轴没有朝 {direction} 外侧 (dot={optical_dot:.3f}"
            f"<{optical_dot_min:.2f})")
    return {
        "status": "ok" if not failures else "bad",
        "camera_center_base_m": center.tolist(),
        "optical_axis_base": optical.tolist(),
        "center_along_axis_m": along,
        "center_cross_axis_m": cross,
        "optical_axis_dot": optical_dot,
        "failures": failures,
    }


def solve_camera_pose(undist_corners, ground_pts_base_ros, new_K,
                      direction=None, pose_qc=None):
    """去畸变角点(Nx2) + base_link 3D 板角点(Nx3,z=0) -> 相机光心系 6DoF。

    返回 (R_opt (3x3), t_opt (3x1))：X_optical = R @ X_base + t。
    """
    object_points = np.asarray(
        ground_pts_base_ros, dtype=np.float64).reshape(-1, 1, 3)
    image_points = np.asarray(
        undist_corners, dtype=np.float64).reshape(-1, 1, 2)
    camera_matrix = np.asarray(new_K, dtype=np.float64)
    candidates = []
    try:
        result = cv2.solvePnPGeneric(
            object_points, image_points, camera_matrix, None,
            flags=cv2.SOLVEPNP_IPPE)
        if result and result[0]:
            errors = result[3] if len(result) > 3 else []
            for index, (rvec, tvec) in enumerate(zip(result[1], result[2])):
                err = float(np.asarray(errors[index]).ravel()[0]) if len(errors) > index else 0.0
                candidates.append((err, rvec, tvec))
    except cv2.error:
        pass
    if not candidates:
        ok, rvec, tvec = cv2.solvePnP(
            object_points, image_points, camera_matrix, None,
            flags=cv2.SOLVEPNP_ITERATIVE)
        if not ok:
            return None
        candidates.append((0.0, rvec, tvec))

    candidates.sort(key=lambda item: item[0])
    for _error, rvec, tvec in candidates:
        R, _ = cv2.Rodrigues(rvec)
        pose = {"R": R.tolist(), "t": np.asarray(tvec).reshape(3).tolist()}
        if direction is None or analyze_camera_pose(
                pose, direction, pose_qc).get("status") == "ok":
            return R, np.asarray(tvec, dtype=np.float64).reshape(3, 1)
    return None


def placement_ground_pts_ros(direction, cols, rows, square, placement):
    """全部内角点在 base_link 下的 3D 坐标（z=0），供 PnP/可视化。

    base_link: x 前 y 左；由车体系 (x 右 y 前) 换算 (ros_x, ros_y)=(gy,-gx)。
    """
    g4, _ = ground_corners(direction, placement["near_m"],
                           placement.get("lateral_m", 0.0),
                           placement.get("orient", "long-lateral"),
                           cols, rows, square)
    gmap = {(0, 0): g4[0], (0, cols - 1): g4[1],
            (rows - 1, cols - 1): g4[2], (rows - 1, 0): g4[3]}
    full = bilinear_ground(gmap[(0, 0)], gmap[(0, cols - 1)],
                           gmap[(rows - 1, 0)], gmap[(rows - 1, cols - 1)],
                           cols, rows)
    ros_pts = np.zeros((len(full), 3), dtype=np.float64)
    ros_pts[:, 0] = full[:, 1]   # ros_x = gy（前）
    ros_pts[:, 1] = -full[:, 0]  # ros_y = -gx（左）
    return ros_pts


# ---------------- 外参 JSON 读写（部分重标合并保存） ----------------

def load_extrinsics_file(path: Path):
    """读 extrinsics.json，homographies 转 float64 ndarray 返回。"""
    with Path(path).open("r", encoding="utf-8") as f:
        data = json.load(f)
    H = {d: np.asarray(M, dtype=np.float64)
         for d, M in (data.get("homographies") or {}).items()}
    return data, H


def _bev_compatible(data: dict, bev_cfg: dict) -> bool:
    meta = data.get("_meta") or {}
    return (meta.get("scale_px_per_meter") == float(bev_cfg["scale_px_per_meter"])
            and meta.get("canvas_size") == list(bev_cfg["canvas_size"])
            and meta.get("extrinsic_balance") == float(bev_cfg["extrinsic_balance"]))


def save_extrinsics(path: Path, direction: str, H_new, *, rms,
                    qc, burst_stats, pose, placements_used, bev_cfg,
                    pattern, seam_meta=None):
    """保存某一路 H（写过的一路覆盖，未重标的一路延续上次——参考 4.4 教训）。

    scale/canvas/balance 任一变则抛弃旧 H，只留本次方向。
    """
    p = Path(path)
    if p.is_file():
        backup_dir = p.parent / "backups"
        backup_dir.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d-%H%M%S-%f")
        shutil.copy2(p, backup_dir / f"extrinsics.{stamp}.json")
        backups = sorted(backup_dir.glob("extrinsics.*.json"), reverse=True)
        for stale in backups[20:]:
            stale.unlink()
    old = {"homographies": {}, "rms_errors": {}, "homography_qc": {},
           "seam_refined": [], "poses": {}}
    if p.is_file():
        with p.open("r", encoding="utf-8") as f:
            old = json.load(f)
        if not _bev_compatible(old, bev_cfg):
            print("⚠️  scale/canvas/balance 与上次不一致，旧 H 全部作废，仅保存本次方向")
            old = {"homographies": {}, "rms_errors": {},
                   "homography_qc": {}, "seam_refined": [], "poses": {}}
    data = dict(old)
    data["pattern_size"] = [int(pattern[0]), int(pattern[1])]
    data["square_size_m"] = float(bev_cfg.get("square_size_m", 0.025))
    data["scale_px_per_meter"] = float(bev_cfg["scale_px_per_meter"])
    data["canvas_size"] = [int(x) for x in bev_cfg["canvas_size"]]
    data["vehicle_center"] = [float(x) for x in bev_cfg["vehicle_center"]]
    data["balance"] = float(bev_cfg["balance"])
    data["extrinsic_balance"] = float(bev_cfg["extrinsic_balance"])
    data["_meta"] = {
        "scale_px_per_meter": data["scale_px_per_meter"],
        "canvas_size": data["canvas_size"],
        "extrinsic_balance": data["extrinsic_balance"],
    }
    data.setdefault("homographies", {})[direction] = (
        np.asarray(H_new, dtype=np.float64).tolist())
    data.setdefault("rms_errors", {})[direction] = float(rms)
    data.setdefault("homography_qc", {})[direction] = qc
    data.setdefault("burst", {})[direction] = burst_stats
    data.setdefault("poses", {})[direction] = pose
    data.setdefault("pose_qc", {})[direction] = (
        analyze_camera_pose(pose, direction) if pose else {
            "status": "unavailable", "failures": ["没有通过物理合理性检查的位姿"]})
    data.setdefault("homography_source", {})[direction] = "single_position_direct_h"
    data.setdefault("placements", {})[direction] = placements_used
    if seam_meta is not None:
        data.setdefault("seam_refined", []).append(dict(seam_meta))
    tmp = p.with_suffix(p.suffix + ".tmp")
    with tmp.open("w", encoding="utf-8") as f:
        json.dump(data, f, indent=2, ensure_ascii=False)
        f.flush()
        os.fsync(f.fileno())
    os.replace(tmp, p)
    return data
