# -*- coding: utf-8 -*-
"""fisheye_math：D_inv 拟合 / K 缩放 / undistortPoints / 合成重投影。"""
import numpy as np
import cv2

from j501_avm_calib import fisheye_math as FM
from synth_utils import make_synthetic_intrinsics


def test_fit_inverse_polynomial():
    D = np.array([0.09, -0.005, 0.002, 0.0005])
    D_inv, max_err = FM.fit_inverse_polynomial(D, max_theta=1.4)
    assert D_inv.shape == (4,)
    # 正反变换在 [0, max_theta] 上误差应很小
    assert max_err < 2e-3


def test_scale_intrinsics():
    K, _ = make_synthetic_intrinsics(1920, 1536)
    Ks = FM.scale_intrinsics(K, 1920, 1536, 960, 768)
    assert np.isclose(Ks[0, 0], K[0, 0] * 0.5)
    assert np.isclose(Ks[0, 2], K[0, 2] * 0.5)
    assert np.isclose(Ks[1, 1], K[1, 1] * 0.5)
    assert np.isclose(Ks[1, 2], K[1, 2] * 0.5)


def test_undistort_points_roundtrip():
    """投影点经 undistortPoints(P=newK) 应近似还原针孔透视点。"""
    K, D = make_synthetic_intrinsics()
    obj = np.random.rand(30, 1, 3).astype(np.float64)
    obj[:, 0, 0] = (obj[:, 0, 0] - 0.5) * 0.5
    obj[:, 0, 1] = (obj[:, 0, 1] - 0.5) * 0.5
    obj[:, 0, 2] = 1.0
    rvec = np.zeros((3, 1))
    tvec = np.array([[0.0], [0.0], [0.0]])
    proj, _ = cv2.fisheye.projectPoints(obj, rvec, tvec, K, D)
    out = FM.undistort_points_fisheye(proj, K, D, 1280, 960, balance=1.0)
    # P=K（balance=1 且主点重置到中心）: 期望 = K @ obj / z
    expect = proj.copy()
    K_und = FM.undistort_new_K(K, 1280, 960, 1.0)
    for i in range(len(obj)):
        x, y, z = obj[i, 0]
        expect[i, 0, 0] = K_und[0, 0] * x / z + K_und[0, 2]
        expect[i, 0, 1] = K_und[1, 1] * y / z + K_und[1, 2]
    err = np.linalg.norm(out - expect, axis=2).mean()
    assert err < 1e-3


def test_synthetic_calibrate_recovers():
    """合成棋盘视角 -> cv2.fisheye.calibrate 恢复 K/D（RMS<0.5px）。"""
    K_true, D_true = make_synthetic_intrinsics(1600, 1200)
    w, h = 1600, 1200
    cols, rows, sq = 8, 6, 0.025
    objp = np.zeros((cols * rows, 1, 3), np.float64)
    objp[:, 0, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * sq
    obj_list, img_list = [], []
    rng = np.random.default_rng(7)
    for _ in range(20):
        # 相机固定，板在图像前方不同位姿（俯仰/偏航/距离都在视野内）
        while True:
            rvec = np.array([rng.uniform(-0.8, 0.8),
                             rng.uniform(-0.8, 0.8),
                             rng.uniform(-0.1, 0.1)])
            tvec = np.array([rng.uniform(-0.3, 0.3),
                             rng.uniform(-0.2, 0.3),
                             rng.uniform(0.9, 1.4)])
            pts, _ = cv2.fisheye.projectPoints(objp, rvec, tvec, K_true,
                                               D_true)
            xs, ys = pts[:, 0, 0], pts[:, 0, 1]
            if (xs.min() > 80 and xs.max() < w - 80
                    and ys.min() > 80 and ys.max() < h - 80):
                break
        obj_list.append(objp.astype(np.float64))
        img_list.append(pts.astype(np.float64))
    K = np.eye(3)
    D = np.zeros((4, 1))
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-8)
    rms, K, D, _, _ = cv2.fisheye.calibrate(
        obj_list, img_list, (w, h), K, D,
        flags=(cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC
               + cv2.fisheye.CALIB_FIX_SKEW),
        criteria=criteria)
    assert rms < 0.5, f"合成标定 RMS 过高: {rms}"
    assert abs(K[0, 0] - K_true[0, 0]) / K_true[0, 0] < 0.05
    assert abs(K[1, 1] - K_true[1, 1]) / K_true[1, 1] < 0.05
    assert abs(K[0, 2] - K_true[0, 2]) < 15
    assert abs(K[1, 2] - K_true[1, 2]) < 15
    for i in range(4):
        assert abs(D[i, 0] - D_true[i]) < 0.02