# -*- coding: utf-8 -*-
"""ROS2 CameraInfo YAML 读写（自有实现）。

本机只装了 camera_calibration_parsers 的 C++ 库与 convert 工具，
没有 python 绑定（/opt/ros/humble/lib/python3.10/site-packages 缺失），
因此这里用 PyYAML 直接读写标准 camera_info YAML，并实现
sensor_msgs/CameraInfo 消息 <-> dict 的换算，接口与行为对齐
camera_calibration_parsers.read/writeCalibration。
"""
from __future__ import annotations

from pathlib import Path
from typing import Any

import numpy as np
import yaml


def _mat_rows(mat: np.ndarray) -> dict:
    m = np.asarray(mat, dtype=np.float64)
    if m.size == 0:
        m = np.zeros((3, 3))
    return {"rows": int(m.shape[0]), "cols": int(m.shape[1]),
            "data": [float(x) for x in m.ravel()]}


def _vec_rows(vec) -> dict:
    v = np.asarray(vec, dtype=np.float64).ravel()
    return {"rows": 1, "cols": int(v.size), "data": [float(x) for x in v]}


def _to_array(d: Any, shape) -> np.ndarray:
    d = d or {}
    data = d.get("data") or []
    rows = int(d.get("rows", shape[0]))
    cols = int(d.get("cols", shape[1]))
    a = np.asarray(data, dtype=np.float64).reshape(rows, cols)
    return a


def camera_info_to_yaml_dict(name: str, width: int, height: int,
                             K: np.ndarray, D: np.ndarray,
                             model: str = "equidistant") -> dict:
    """构造 ROS2 CameraInfo YAML 数据。P = [K | 0]（鱼眼无标准 P）。"""
    k = np.asarray(K, dtype=np.float64).reshape(3, 3)
    P = np.zeros((3, 4), dtype=np.float64)
    P[:3, :3] = k
    R = np.eye(3, dtype=np.float64)
    return {
        "image_width": int(width),
        "image_height": int(height),
        "camera_name": str(name),
        "camera_matrix": _mat_rows(k),
        "distortion_model": str(model),
        "distortion_coefficients": _vec_rows(D),
        "rectification_matrix": _mat_rows(R),
        "projection_matrix": _mat_rows(P[:3, :4]),
    }


def write_calibration_yaml(path: Path, name: str, width: int, height: int,
                           K: np.ndarray, D: np.ndarray,
                           model: str = "equidistant") -> None:
    """写标准 camera_info YAML（cam_info_io 自有实现）。"""
    p = Path(path)
    p.parent.mkdir(parents=True, exist_ok=True)
    data = camera_info_to_yaml_dict(name, width, height, K, D, model)
    with p.open("w", encoding="utf-8") as f:
        yaml.safe_dump(data, f, default_flow_style=False, sort_keys=False)


def read_calibration_yaml(path: Path) -> dict:
    """读标准 camera_info YAML，返回 {'name','width','height','K','D',
    'model','R','P'}。"""
    p = Path(path)
    with p.open("r", encoding="utf-8") as f:
        d = yaml.safe_load(f) or {}
    K = _to_array(d.get("camera_matrix"), (3, 3))
    if K.shape != (3, 3):
        raise ValueError(f"{p}: camera_matrix 形状异常 {K.shape}")
    D = _to_array(d.get("distortion_coefficients"), (1, 4)).ravel()
    return {
        "name": str(d.get("camera_name") or p.stem),
        "width": int(d.get("image_width") or 0),
        "height": int(d.get("image_height") or 0),
        "K": K,
        "D": D,
        "model": str(d.get("distortion_model") or "unknown"),
        "R": _to_array(d.get("rectification_matrix"), (3, 3)),
        "P": _to_array(d.get("projection_matrix"), (3, 4)),
    }


def empty_calibration_yaml(name: str, width: int, height: int) -> dict:
    """未标定模板（K=0 等），供 driver 初启发布 / cameracalibrator 等待输入。"""
    return {
        "image_width": int(width),
        "image_height": int(height),
        "camera_name": str(name),
        "camera_matrix": {"rows": 3, "cols": 3,
                          "data": [0.0] * 9},
        "distortion_model": "",
        "distortion_coefficients": {"rows": 1, "cols": 0, "data": []},
        "rectification_matrix": {"rows": 3, "cols": 3,
                                 "data": [1.0, 0, 0, 0, 1.0, 0, 0, 0, 1.0]},
        "projection_matrix": {"rows": 3, "cols": 4, "data": [0.0] * 12},
    }


# ---------------- sensor_msgs 消息换算 ----------------

def to_camera_info_msg(header_stamp, frame_id: str, data: dict):
    """dict(YAML 数据) -> sensor_msgs.msg.CameraInfo。"""
    from sensor_msgs.msg import CameraInfo
    msg = CameraInfo()
    msg.header.stamp = header_stamp
    msg.header.frame_id = frame_id
    msg.height = int(data.get("image_height") or 0)
    msg.width = int(data.get("image_width") or 0)
    msg.distortion_model = str(data.get("distortion_model") or "")
    d = data.get("distortion_coefficients") or {}
    msg.d = [float(x) for x in (d.get("data") or [])]
    kd = data.get("camera_matrix") or {}
    msg.k = [float(x) for x in (kd.get("data") or [])]
    rd = data.get("rectification_matrix") or {}
    msg.r = [float(x) for x in (rd.get("data") or [])]
    pd = data.get("projection_matrix") or {}
    msg.p = [float(x) for x in (pd.get("data") or [])]
    msg.binning_x = 0
    msg.binning_y = 0
    msg.roi.x_offset = msg.roi.y_offset = 0
    msg.roi.height = msg.roi.width = 0
    msg.roi.do_rectify = False
    return msg


def from_camera_info_msg(msg) -> dict:
    """sensor_msgs.msg.CameraInfo -> dict(YAML 数据)。"""
    k = list(msg.k) if len(msg.k) else [0.0] * 9
    r = list(msg.r) if len(msg.r) else [1.0, 0, 0, 0, 1.0, 0, 0, 0, 1.0]
    p = list(msg.p) if len(msg.p) else [0.0] * 12
    return {
        "image_width": int(msg.width),
        "image_height": int(msg.height),
        "camera_name": str(msg.header.frame_id),
        "camera_matrix": {"rows": 3, "cols": 3, "data": k},
        "distortion_model": str(msg.distortion_model or ""),
        "distortion_coefficients": {"rows": 1, "cols": len(msg.d),
                                    "data": [float(x) for x in msg.d]},
        "rectification_matrix": {"rows": 3, "cols": 3, "data": r},
        "projection_matrix": {"rows": 3, "cols": 4, "data": p},
    }


def is_calibrated(data: dict) -> bool:
    """K 第一元 >0 视为已标定（ROS camera_info 惯例）。"""
    d = (data.get("camera_matrix") or {}).get("data") or [0.0]
    return float(d[0]) > 0.0