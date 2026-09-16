# -*- coding: utf-8 -*-
"""内参解析：ROS2 CameraInfo YAML -> calib_results/<dir>.json（兼容参考项目格式）。

另有 RMS 回填：cameracalibrator 的 SAVE 会写 /tmp/calibrationdata.tar.gz
（含采样棋盘图的 PNG），CameraInfo 本身不带 RMS——这里解包重检角点、
用已标定 K/D 逐视角重投影，得到 overall rms / per_view_rms / 异常帧名单。

用法:
    ros2 run j501_avm_calib intrinsics_parse --direction front [--tar /tmp/calibrationdata.tar.gz]
    ros2 run j501_avm_calib intrinsics_parse --all [--tar ...]   # 四路(每路各自 tar)
"""
from __future__ import annotations

import argparse
import json
import tarfile
import tempfile
from pathlib import Path

import numpy as np
import cv2

from j501_avm_calib.config import load_config, camera_info_dir, results_dir, DIRECTIONS
from j501_avm_calib import cam_info_io
from j501_avm_calib.fisheye_math import fit_inverse_polynomial
from j501_avm_calib.detect_board import find_board_corners
from j501_avm_calib.intrinsic_quality import compute_per_view_errors


def _extract_images(tar_path: str, tmp: Path) -> list[Path]:
    """从 calibrationdata.tar.gz 解出 left-*.png，按编号排序返回。"""
    out = []
    with tarfile.open(tar_path, "r:gz") as tf:
        names = sorted([n for n in tf.getnames()
                        if n.startswith("left-") and n.endswith(".png")])
        for n in names:
            dst = tmp / Path(n).name
            with tf.extractfile(n) as src, dst.open("wb") as f:
                f.write(src.read())
            out.append(dst)
    return out


def _build_obj_points(cols, rows, square):
    objp = np.zeros((cols * rows, 3), np.float64)
    objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2).astype(np.float64)
    objp *= float(square)
    return objp.reshape(-1, 1, 3)


def _rms_from_images(tar_path: str, K, D, pattern, square,
                     detect_max_width=1280):
    """解 tar -> 检角点 -> fisheye 逐视角重投影（PNP 独立解位姿）。

    返回 dict(rms 总体, per_view_rms, n_views, dropped_views)。
    """
    cols, rows = pattern
    objp = _build_obj_points(cols, rows, square)
    with tempfile.TemporaryDirectory() as td:
        images = _extract_images(tar_path, Path(td))
    if not images:
        return {"rms": None, "per_view_rms": [], "n_views": 0,
                "dropped_views": 0, "error": "tar 中没有采样图"}
    obj_pts, img_pts, rvecs, tvecs = [], [], [], []
    dropped = 0
    for p in images:
        bgr = cv2.imread(str(p))
        if bgr is None:
            dropped += 1
            continue
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        corners = find_board_corners(gray, pattern, use_sb=True,
                                     photo_retry=True, allow_classic=True)
        if corners is None:
            dropped += 1
            continue
        c = corners.reshape(-1, 2)
        try:
            retval, rvec, tvec, _inliers = cv2.solvePnP(
                objp, c, K, D, flags=cv2.SOLVEPNP_ITERATIVE)
            if not retval:
                dropped += 1
                continue
        except cv2.error:
            dropped += 1
            continue
        obj_pts.append(objp)
        img_pts.append(c)
        rvecs.append(np.asarray(rvec, dtype=np.float64).reshape(3, 1))
        tvecs.append(np.asarray(tvec, dtype=np.float64).reshape(3, 1))
    if not obj_pts:
        return {"rms": None, "per_view_rms": [], "n_views": 0,
                "dropped_views": dropped, "error": "没有可解位姿的视角"}
    per_view = compute_per_view_errors(obj_pts, img_pts, K, D, rvecs, tvecs,
                                       model="equidistant")
    rms = float(np.sqrt(np.mean([e ** 2 for e in per_view])))
    return {"rms": rms, "per_view_rms": per_view, "n_views": len(obj_pts),
            "dropped_views": dropped}


def parse_one(direction: str, tar_path: str | None = None,
              out_dir: Path | None = None) -> dict:
    """解析一路 CameraInfo YAML -> 结果 JSON 文件。返回 file data。"""
    cfg = load_config()
    yaml_path = camera_info_dir() / f"{direction}.yaml"
    data = cam_info_io.read_calibration_yaml(yaml_path)
    K = data["K"]
    D = data["D"]
    if not cam_info_io.is_calibrated(
            cam_info_io.camera_info_to_yaml_dict(
                direction, data["width"], data["height"], K, D, data["model"])):
        raise RuntimeError(f"{yaml_path} 尚未标定（K 未初始化），请先完成 "
                           f"{direction} 的 GUI/自动标定")
    if data["model"] != "equidistant":
        print(f"⚠️  [{direction}] distortion_model='{data['model']}' != "
              f"'equidistant'：pinhole 拟合鱼眼会损失视野，建议用 fisheye 模式重标")

    cols, rows = cfg["pattern_size"]
    square = cfg["chessboard"]["square_size_m"]
    rms = None
    per_view = []
    if tar_path and Path(tar_path).is_file():
        r = _rms_from_images(tar_path, K, D, (cols, rows), square)
        rms = r["rms"]
        per_view = r["per_view_rms"]
        print(f"[{direction}] tar 重投影: rms={rms and f'{rms:.4f}px' or 'N/A'} "
              f"视角 {r['n_views']} 有效 / {r['dropped_views']} 丢弃")
    elif tar_path:
        print(f"⚠️  [{direction}] tar 不存在({tar_path})，rms 留空")
    D_inv, max_err = fit_inverse_polynomial(D)

    out = out_dir or results_dir()
    out.mkdir(parents=True, exist_ok=True)
    result = {
        "K": K.tolist(),
        "D": D.tolist(),
        "D_inv": D_inv.tolist(),
        "rms": float(rms) if rms is not None else None,
        "per_view_rms": per_view,
        "image_size": [int(data["width"]), int(data["height"])],
        "model": data["model"],
        "d_inv_max_err": max_err,
    }
    with (out / f"{direction}.json").open("w", encoding="utf-8") as f:
        json.dump(result, f, indent=2, ensure_ascii=False)
    print(f"[{direction}] 已写入 {out / (direction + '.json')} "
          f"(K {K.tolist()[0][0]:.2f}, D {[round(x,4) for x in D.tolist()]}, "
          f"rms={rms and f'{rms:.4f}' or 'N/A'})")
    return result


def main(args=None):
    parser = argparse.ArgumentParser(description="解析 ROS2 CameraInfo -> calib_results JSON")
    parser.add_argument("--direction", choices=list(DIRECTIONS),
                        help="单路方向；缺省与 --all 配合处理四路")
    parser.add_argument("--all", action="store_true", help="处理全部四路")
    parser.add_argument("--tar", type=str, default=None,
                        help="cameracalibrator SAVE 的 /tmp/calibrationdata.tar.gz")
    parser.add_argument("--prefix", type=str, default="",
                        help="--all 时 tar 前缀，如 /tmp/calib_front_，"
                             "取 /tmp/calib_{dir}_calibrationdata.tar.gz")
    a, _ = parser.parse_known_args(args)
    if not a.all and not a.direction:
        parser.error("需要 --direction 或 --all")
    ok = True
    if a.direction:
        targets = [a.direction]
    else:
        targets = list(DIRECTIONS)
    for d in targets:
        tar = a.tar
        if a.all and not tar and a.prefix:
            tar = f"{a.prefix}{d}_calibrationdata.tar.gz"
        try:
            parse_one(d, tar)
        except Exception as exc:  # noqa: BLE001
            print(f"❌ [{d}] 解析失败: {exc}")
            ok = False
    raise SystemExit(0 if ok else 1)


if __name__ == "__main__":
    main()