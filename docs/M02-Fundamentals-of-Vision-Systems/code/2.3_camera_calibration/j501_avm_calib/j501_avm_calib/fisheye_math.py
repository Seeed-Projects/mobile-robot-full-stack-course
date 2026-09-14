# -*- coding: utf-8 -*-
"""鱼眼(Kannala-Brandt/equidistant)数学工具：去畸变、D_inv、点位映射。

端口自参考项目 avm/cuda_cv.py 的 undistort_new_K / init_undistort_maps /
undistort_points_fisheye，保证「角点映射」与「整图去畸变」用同一个 new_K，
否则求 H 的坐标与去畸变图不一致会引入系统误差。
"""
from __future__ import annotations

import numpy as np
import cv2


def undistort_new_K(K, w: int, h: int, balance: float) -> np.ndarray:
    """与 init_undistort_maps 完全一致的 new_K（balance 缩放 fx/fy，主点居中）。"""
    new_K = np.asarray(K, dtype=np.float64).copy()
    new_K[0, 0] *= float(balance)
    new_K[1, 1] *= float(balance)
    new_K[0, 2] = float(w) / 2.0
    new_K[1, 2] = float(h) / 2.0
    return new_K


def init_undistort_maps(K, D, w: int, h: int, balance: float,
                        for_cuda: bool = False):
    """构建鱼眼去畸变 remap 表（CV_32FC1 x,y / CV_16SC2 合并）。"""
    new_K = undistort_new_K(K, w, h, balance)
    mtype = cv2.CV_32FC1 if for_cuda else cv2.CV_16SC2
    return cv2.fisheye.initUndistortRectifyMap(
        np.asarray(K, dtype=np.float64),
        np.asarray(D, dtype=np.float64),
        np.eye(3, dtype=np.float64),
        new_K,
        (int(w), int(h)),
        mtype,
    )


def undistort_points_fisheye(pts: np.ndarray, K, D, w: int, h: int,
                             balance: float) -> np.ndarray:
    """鱼眼原图点 -> 去畸变图坐标（与存放的 H 坐标系一致）。

    参考项目实测：直接映射点比重采样整图再检测更准（少一次插值）。
    输入 (N,1,2)/(N,2)，输出同形状 float32。
    """
    shape = pts.shape
    p = np.asarray(pts, dtype=np.float64).reshape(-1, 1, 2)
    new_K = undistort_new_K(K, w, h, balance)
    out = cv2.fisheye.undistortPoints(
        p,
        np.asarray(K, dtype=np.float64),
        np.asarray(D, dtype=np.float64),
        R=np.eye(3),
        P=new_K,
    )
    return out.astype(np.float32).reshape(shape)


def fit_inverse_polynomial(D, max_theta: float = 1.5,
                           num_samples: int = 1000) -> tuple[np.ndarray, float]:
    """拟合 D_inv：由 theta_d -> theta 的 4 系数多项式（端口参考实现）。

    D   : 正向畸变 [k1,k2,k3,k4]（fisheye theta_d = theta(1+k1θ²+k2θ⁴+...)）
    返回 (D_inv, max_err)。
    """
    D = np.asarray(D, dtype=np.float64).reshape(-1)
    theta = np.linspace(0.0, max_theta, num_samples)
    t3, t5, t7, t9 = (theta ** 3, theta ** 5, theta ** 7, theta ** 9)
    theta_d = theta + D[0] * t3 + D[1] * t5 + D[2] * t7 + D[3] * t9
    td3, td5 = theta_d ** 3, theta_d ** 5
    td7, td9 = theta_d ** 7, theta_d ** 9
    A = np.column_stack([td3, td5, td7, td9])
    b = theta - theta_d
    D_inv, _, _, _ = np.linalg.lstsq(A, b, rcond=None)
    D_inv = D_inv.flatten()
    fitted = (theta_d + D_inv[0] * td3 + D_inv[1] * td5
              + D_inv[2] * td7 + D_inv[3] * td9)
    max_err = float(np.max(np.abs(fitted - theta)))
    return D_inv, max_err


def scale_intrinsics(K, src_w, src_h, dst_w, dst_h) -> np.ndarray:
    """分辨率缩放：fx,fy,cx,cy 线性缩放；D 不变（用于不同采集分辨率）。"""
    K = np.asarray(K, dtype=np.float64).copy()
    sx = float(dst_w) / float(src_w)
    sy = float(dst_h) / float(src_h)
    K[0, 0] *= sx
    K[1, 1] *= sy
    K[0, 2] *= sx
    K[1, 2] *= sy
    return K


def project_points_fisheye(obj_pts, rvec, tvec, K, D):
    """cv2.fisheye.projectPoints 封装（合成测试/重投影评估共用）。"""
    return cv2.fisheye.projectPoints(
        np.asarray(obj_pts, dtype=np.float64).reshape(-1, 1, 3),
        np.asarray(rvec, dtype=np.float64).reshape(3, 1),
        np.asarray(tvec, dtype=np.float64).reshape(3, 1),
        np.asarray(K, dtype=np.float64),
        np.asarray(D, dtype=np.float64).reshape(-1, 1),
    )