# -*- coding: utf-8 -*-
"""内参标定效果评估（阈值与判据端口自参考项目 calibrate_intrinsics.py）。

输出 status: pass | warn | fail，附带整体/逐帧 RMS、K/D 合理性、FOV 覆盖。
"""
from __future__ import annotations

import numpy as np
import cv2

from j501_avm_calib.config import DEFAULTS as _D
from j501_avm_calib.fisheye_math import project_points_fisheye

_QC = _D["intrinsic_qc"]


def compute_per_view_errors(obj_points, img_points, K, D, rvecs, tvecs,
                            model: str = "equidistant"):
    """逐帧重投影 RMS(px)，返回 list[float]。"""
    per_view = []
    for op, ip, rv, tv in zip(obj_points, img_points, rvecs, tvecs):
        if model == "equidistant":
            projected, _ = project_points_fisheye(op, rv, tv, K, D)
        else:
            projected, _ = cv2.projectPoints(
                np.asarray(op, dtype=np.float64).reshape(-1, 1, 3),
                np.asarray(rv, dtype=np.float64),
                np.asarray(tv, dtype=np.float64),
                np.asarray(K, dtype=np.float64),
                np.asarray(D, dtype=np.float64).ravel())
        err = np.sqrt(np.mean(np.sum(
            (projected.reshape(-1, 2) - np.asarray(ip).reshape(-1, 2)) ** 2,
            axis=1)))
        per_view.append(float(err))
    return per_view


def check_k_sanity(K, image_size, qc=None):
    """K 物理合理性。(ok, warnings)。"""
    qc = qc or _QC
    w, h = image_size
    fx, fy = float(K[0, 0]), float(K[1, 1])
    cx, cy = float(K[0, 2]), float(K[1, 2])
    warnings = []
    aspect = max(fx, fy) / max(min(fx, fy), 1e-9)
    if aspect > qc["k_aspect_ratio_max"]:
        warnings.append(
            f"fx/fy 比值={aspect:.2f}>{qc['k_aspect_ratio_max']}: "
            "像素纵横比异常，可能标定不足")
    cx_dev = abs(cx - w / 2.0) / w
    cy_dev = abs(cy - h / 2.0) / h
    if cx_dev > qc["k_center_tol"]:
        warnings.append(f"cx={cx:.1f} 偏离图像中心 {cx_dev*100:.1f}%"
                        f">{qc['k_center_tol']*100:.0f}%")
    if cy_dev > qc["k_center_tol"]:
        warnings.append(f"cy={cy:.1f} 偏离图像中心 {cy_dev*100:.1f}%"
                        f">{qc['k_center_tol']*100:.0f}%")
    if fx < w * 0.15 or fx > w * 0.8:
        warnings.append(f"fx={fx:.1f} 超出预期范围 [{w*0.15:.0f}, {w*0.8:.0f}]")
    if fy < h * 0.15 or fy > h * 0.8:
        warnings.append(f"fy={fy:.1f} 超出预期范围 [{h*0.15:.0f}, {h*0.8:.0f}]")
    return len(warnings) == 0, warnings


def check_distortion_sanity(D, qc=None):
    """畸变系数合理性。"""
    qc = qc or _QC
    D = np.asarray(D, dtype=np.float64).reshape(-1)
    warnings = []
    for i, d in enumerate(D):
        if abs(d) > qc["d_max_abs"]:
            warnings.append(f"D[{i}]={d:.4f} 绝对值>{qc['d_max_abs']}: "
                            "畸变系数过大，可能标定失败或棋盘姿态不足")
    if D.size and D[0] < -0.02:
        warnings.append(f"D[0]={D[0]:.4f}<0: 不典型的枕形畸变，"
                        "检查是否用错了棋盘格方向或标定板")
    return len(warnings) == 0, warnings


def check_fov_coverage(img_points, image_size):
    """角点 FOV 覆盖：3x3 网格≥5 区且四角覆盖。"""
    w, h = image_size
    pts = np.vstack([np.asarray(p).reshape(-1, 2) for p in img_points])
    grid_hits = np.zeros((3, 3), dtype=bool)
    for pt in pts:
        gx = min(int(pt[0] / w * 3), 2)
        gy = min(int(pt[1] / h * 3), 2)
        grid_hits[gy, gx] = True
    covered = int(grid_hits.sum())
    corners_covered = all([
        np.any((pts[:, 0] < w * 0.2) & (pts[:, 1] < h * 0.2)),
        np.any((pts[:, 0] > w * 0.8) & (pts[:, 1] < h * 0.2)),
        np.any((pts[:, 0] < w * 0.2) & (pts[:, 1] > h * 0.8)),
        np.any((pts[:, 0] > w * 0.8) & (pts[:, 1] > h * 0.8)),
    ])
    info = {"grid_coverage": f"{covered}/9", "grid_hits": grid_hits.tolist(),
            "total_points": int(len(pts)), "corners_covered": bool(corners_covered)}
    ok = covered >= 5 and corners_covered
    warnings = []
    if not ok:
        if covered < 5:
            warnings.append(f"仅覆盖 {covered}/9 个网格区域")
        if not corners_covered:
            warnings.append("四角覆盖不足")
        info["warnings"] = warnings
    return ok, info


def evaluate_intrinsics(K, D, rms, image_size, obj_points=None,
                        img_points=None, rvecs=None, tvecs=None,
                        per_view_errors=None, model="equidistant",
                        qc=None) -> dict:
    """综合内参评估。返回 report dict（status/passed/warnings/明细）。"""
    qc = qc or _QC
    K = np.asarray(K, dtype=np.float64)
    D = np.asarray(D, dtype=np.float64).reshape(-1)
    warnings: list[str] = []
    report = {
        "status": "pass",
        "overall_rms": float(rms),
        "per_view_rms": [],
        "outlier_views": [],
        "k_sanity": {},
        "distortion_sanity": {},
        "fov_coverage": {},
        "warnings": warnings,
        "passed": True,
    }
    if rms > qc["rms_fail_px"]:
        report["status"] = "fail"
        report["passed"] = False
        warnings.append(f"整体 RMS={rms:.3f}px > {qc['rms_fail_px']}px: 标定不合格")
    elif rms > qc["rms_pass_px"]:
        report["status"] = "warn"
        warnings.append(f"整体 RMS={rms:.3f}px > {qc['rms_pass_px']}px: 精度偏低")

    if per_view_errors is None and obj_points is not None and img_points is not None and rvecs is not None:
        per_view_errors = compute_per_view_errors(
            obj_points, img_points, K, D, rvecs, tvecs, model=model)
    if per_view_errors:
        report["per_view_rms"] = [float(e) for e in per_view_errors]
        outlier = [i for i, e in enumerate(per_view_errors)
                   if e > qc["per_view_outlier_px"]]
        report["outlier_views"] = outlier
        if len(outlier) > len(per_view_errors) * 0.2:
            if report["status"] == "pass":
                report["status"] = "warn"
            warnings.append(f"{len(outlier)}/{len(per_view_errors)} 帧异常"
                            f"(RMS>{qc['per_view_outlier_px']}px): 建议删除这些帧后重新标定")
        warn_views = [i for i, e in enumerate(per_view_errors)
                      if qc["per_view_rms_warn_px"] < e <= qc["per_view_outlier_px"]]
        if warn_views:
            warnings.append(f"{len(warn_views)} 帧 RMS 偏高"
                            f"({qc['per_view_rms_warn_px']}~{qc['per_view_outlier_px']}px)")

    k_ok, k_w = check_k_sanity(K, image_size, qc)
    report["k_sanity"] = {"ok": bool(k_ok), "warnings": k_w}
    if not k_ok:
        if report["status"] == "pass":
            report["status"] = "warn"
        warnings.extend(k_w)

    d_ok, d_w = check_distortion_sanity(D, qc)
    report["distortion_sanity"] = {"ok": bool(d_ok), "warnings": d_w}
    if not d_ok:
        report["status"] = "fail"
        report["passed"] = False
        warnings.extend(d_w)

    if img_points is not None and len(img_points) > 0:
        fov_ok, fov_info = check_fov_coverage(img_points, image_size)
        report["fov_coverage"] = fov_info
        if not fov_ok:
            if report["status"] == "pass":
                report["status"] = "warn"
            warnings.extend(fov_info.get("warnings", []))

    if img_points is not None and len(img_points) < qc["min_valid_views"]:
        if report["status"] == "pass":
            report["status"] = "warn"
        warnings.append(f"仅 {len(img_points)} 帧 < {qc['min_valid_views']} 帧: "
                        "推荐至少 15 帧覆盖不同姿态")
    report["passed"] = report["status"] != "fail"
    return report


def print_intrinsic_report(report, direction=""):
    """终端打印内参评估报告（中文）。"""
    label = f" [{direction}]" if direction else ""
    icon = {"pass": "✅", "warn": "⚠️", "fail": "❌"}.get(report["status"], "?")
    print(f"\n{'='*60}")
    print(f"  内参评估报告{label}  {icon} {report['status'].upper()}")
    print(f"{'='*60}")
    print(f"  整体 RMS:         {report['overall_rms']:.4f} px")
    if report["per_view_rms"]:
        pv = report["per_view_rms"]
        print(f"  逐帧 RMS:         min={min(pv):.3f}  max={max(pv):.3f}  "
              f"median={np.median(pv):.3f} px")
        if report["outlier_views"]:
            print(f"  异常帧:           {report['outlier_views']}")
    for w in report["k_sanity"].get("warnings", []):
        print(f"  K 矩阵:           ⚠️  {w}")
    for w in report["distortion_sanity"].get("warnings", []):
        print(f"  畸变系数:         ⚠️  {w}")
    fov = report.get("fov_coverage") or {}
    if fov:
        print(f"  FOV 覆盖率:       {fov.get('grid_coverage','?')}/9 网格"
              f"  四角={'✅' if fov.get('corners_covered') else '❌'}")
    print(f"  结论:             "
          f"{'合格，可进入外参标定' if report['passed'] else '不合格，建议重新采集标定图像'}")
    print(f"{'='*60}\n")