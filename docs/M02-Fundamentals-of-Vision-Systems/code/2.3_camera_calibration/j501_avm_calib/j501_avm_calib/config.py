# -*- coding: utf-8 -*-
"""全局配置：默认值、阈值、目录解析（真源 config/calib_config.yaml）。"""
from __future__ import annotations

import json
import os
from pathlib import Path
from typing import Any

import yaml

DIRECTIONS = ("front", "back", "left", "right")

# 相邻相机接缝精修顺序（参考项目 SEAM_PAIRS）
SEAM_PAIRS = (
    ("front", "left"),
    ("front", "right"),
    ("back", "left"),
    ("back", "right"),
)

# 车体系视线方向单位向量（x 右 y 前），用于 H 翻转判定 / 摆位换算
CAM_AXIS = {"front": (0.0, 1.0), "back": (0.0, -1.0),
            "left": (-1.0, 0.0), "right": (1.0, 0.0)}

# ---------------- 默认标定参数（calib_config.yaml 可覆盖） ----------------
DEFAULTS: dict[str, Any] = {
    "chessboard": {
        "pattern_size": [8, 6],       # 内角点 cols x rows，8x6
        "square_size_m": 0.025,       # 格宽(米)，必须与实物一致
    },
    "placements": {
        "front": {"near_m": 0.56, "lateral_m": 0.0, "orient": "long-lateral"},
        "back": {"near_m": 0.56, "lateral_m": 0.0, "orient": "long-lateral"},
        "left": {"near_m": 0.35, "lateral_m": 0.0, "orient": "long-lateral"},
        "right": {"near_m": 0.35, "lateral_m": 0.0, "orient": "long-lateral"},
    },
    "bev": {
        "scale_px_per_meter": 100.0,
        "canvas_size": [1000, 1000],
        "vehicle_center": [500.0, 500.0],
        "balance": 0.8,               # 去畸变 0~1 平衡（0.8=保留更多视野）
        "extrinsic_balance": 0.8,
    },
    "capture": {
        "width": 1920,
        "height": 1536,
        "fourcc": "YUYV",
        "backend": "v4l2",            # v4l2 | gstreamer
        "gst_pipeline_template": "",
        "frame_rate_hz": 10.0,
        "preview_scale": 0.4,         # BEV/DEMO 预览尺度（CPU 拼接用）
        "cameras": {
            "front": {"device": 0},
            "back": {"device": 2},
            "left": {"device": 3},
            "right": {"device": 1},
        },
    },
    "extrinsic": {
        "stable_frames": 10,          # 预览连续检出帧数(READY)
        "burst_frames": 8,            # 连拍帧数
        "corner_outlier_rms_px": 2.5,
        "corner_align_max_px": 40.0,
        "seam_fresh_max_age": 1.5,    # 接缝联合计数最大帧龄(秒)
        "inview_margin_px": 2,        # 整板角点须落入去畸变图(留边)
        "detect_max_width": 1920,
    },
    # 外参 H 质量控制阈值（与参考项目一致）
    "qc": {
        "h_svd_min": 0.03,
        "h_edge_span_min_px": 25.0,
        "h_center_tol_px": 350.0,
        "h_center_flip_tol_px": 50.0,
        "board_edge_ratio_max": 1.35,
        "lr_sigma_ratio_max": 4.0,
        "rms_good_px": 1.0,           # <1px 合格
        "rms_ok_px": 3.0,             # 单点外参硬门槛；<1px 为目标
    },
    # 内参评估阈值（与参考项目一致）
    "intrinsic_qc": {
        "rms_pass_px": 0.8,
        "rms_fail_px": 1.5,
        "per_view_rms_warn_px": 2.0,
        "per_view_outlier_px": 3.5,
        "d_max_abs": 0.35,
        "k_aspect_ratio_max": 1.15,
        "k_center_tol": 0.12,
        "min_valid_views": 12,
    },
    "pose_qc": {                       # 平面 PnP 必须通过的物理合理性门槛
        "height_min_m": 0.05,
        "height_max_m": 0.6,
        "side_min_m": 0.05,
        "side_max_m": 0.65,
        "cross_axis_max_m": 0.45,
        "optical_axis_dot_min": 0.25,
    },
}


def _env_path(name: str) -> Path | None:
    v = os.environ.get(name)
    return Path(v).resolve() if v else None


def config_dir() -> Path:
    """可写配置目录：首先 J501_AVM_CALIB_CONFIG_DIR，其次源码树 config/，
    最后包 share 目录（只读时返回但调用方负责处理写失败）。"""
    d = _env_path("J501_AVM_CALIB_CONFIG_DIR")
    if d:
        return d
    local = Path(__file__).resolve().parents[2] / "config"
    if local.is_dir():
        return local
    from ament_index_python.packages import get_package_share_directory
    return Path(get_package_share_directory("j501_avm_calib")) / "config"


def results_dir() -> Path:
    """标定结果输出目录。首先 J501_AVM_CALIB_RESULTS_DIR，其次 cwd/calib_results。"""
    d = _env_path("J501_AVM_CALIB_RESULTS_DIR")
    if d:
        return d
    return Path.cwd() / "calib_results"


def camera_info_dir() -> Path:
    """ROS2 CameraInfo YAML 目录（cameracalibrator COMMIT 的落盘位置）。

    优先 config_dir()/camera_info；share 安装目录只读时回退到
    ~/.j501_avm_calib/camera_info（保证 set_camera_info 可写）。
    """
    d = config_dir() / "camera_info"
    if d.is_dir() and os.access(d, os.W_OK):
        return d
    if not d.exists():
        try:
            d.mkdir(parents=True, exist_ok=True)
            if os.access(d, os.W_OK):
                return d
        except OSError:
            pass
    fallback = Path.home() / ".j501_avm_calib" / "camera_info"
    fallback.mkdir(parents=True, exist_ok=True)
    return fallback


# ---------------- 配置加载 ----------------

def _deep_merge(base: dict, override: dict) -> dict:
    out = json.loads(json.dumps(base))
    for k, v in (override or {}).items():
        if isinstance(v, dict) and isinstance(out.get(k), dict):
            out[k] = _deep_merge(out[k], v)
        else:
            out[k] = v
    return out


def load_config(path: Path | None = None) -> dict:
    """加载 calib_config.yaml（缺失则用 DEFAULTS 并写一份模板）。"""
    cfg = json.loads(json.dumps(DEFAULTS))
    p = Path(path) if path else (config_dir() / "calib_config.yaml")
    if p.is_file():
        with p.open("r", encoding="utf-8") as f:
            cfg = _deep_merge(cfg, yaml.safe_load(f) or {})
    else:
        p.parent.mkdir(parents=True, exist_ok=True)
        with p.open("w", encoding="utf-8") as f:
            yaml.safe_dump(DEFAULTS, f, allow_unicode=True, sort_keys=False)
    cfg["pattern_size"] = (int(cfg["chessboard"]["pattern_size"][0]),
                           int(cfg["chessboard"]["pattern_size"][1]))
    return cfg
