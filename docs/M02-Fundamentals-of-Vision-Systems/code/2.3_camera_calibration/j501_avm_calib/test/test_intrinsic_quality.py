# -*- coding: utf-8 -*-
"""intrinsic_quality：内参评估阈值与判据测试。"""
import numpy as np

from j501_avm_calib.intrinsic_quality import (evaluate_intrinsics,
                                              check_fov_coverage)


def _K(w=1920, h=1536):
    return np.array([[500.0, 0, w / 2], [0, 498.0, h / 2], [0, 0, 1.0]])


def test_pass():
    r = evaluate_intrinsics(_K(), np.array([0.1, -0.02, 0.003, 0.001]),
                            0.5, (1920, 1536))
    assert r["status"] == "pass" and r["passed"]


def test_rms_fail():
    r = evaluate_intrinsics(_K(), np.array([0.1, -0.02, 0.003, 0.001]),
                            2.0, (1920, 1536))
    assert r["status"] == "fail" and not r["passed"]


def test_rms_warn_band():
    r = evaluate_intrinsics(_K(), np.array([0.1, -0.02, 0.003, 0.001]),
                            1.0, (1920, 1536))
    assert r["status"] == "warn"


def test_distortion_fail():
    r = evaluate_intrinsics(_K(), np.array([0.5, 0.0, 0.0, 0.0]),
                            0.4, (1920, 1536))
    assert r["status"] == "fail"
    assert any("D[0]" in w for w in r["warnings"])


def test_pinhole_style_d_warns():
    r = evaluate_intrinsics(_K(), np.array([-0.05, 0.0, 0.0, 0.0]),
                            0.4, (1920, 1536))
    assert any("枕形" in w for w in r["warnings"])


def test_k_aspect_warns():
    K = _K().copy()
    K[0, 0] = 500 * 1.3   # fx/fy = 1.302 > 1.15
    r = evaluate_intrinsics(K, np.array([0.1, 0, 0, 0]), 0.4, (1920, 1536))
    assert r["status"] == "warn"
    assert any("纵横比" in w for w in r["k_sanity"]["warnings"])


def test_fov_coverage():
    w, h = 1920, 1536
    # 只覆盖图像中央 -> 网格 1/9，四角不覆盖 -> fail
    pts = [np.array([[w / 2 + i, h / 2 + j] for i in range(4)
                     for j in range(4)])]
    ok, info = check_fov_coverage(pts, (w, h))
    assert not ok
    assert info["grid_coverage"] == "1/9"
    # 覆盖中央 + 四角和足够多的网格
    pts2 = [np.array([[w / 2 + i * 150, h / 2 + j * 150]
                      for i in range(-4, 5) for j in range(-4, 5)])]
    ok2, _ = check_fov_coverage(pts2, (w, h))
    assert ok2


def test_min_views_warn():
    img_pts = [np.array([[400.0, 300.0]])] * 5  # 5 帧 < 12
    r = evaluate_intrinsics(_K(), np.array([0.1, 0, 0, 0]), 0.5,
                            (1920, 1536), img_points=img_pts)
    assert r["status"] == "warn"
    assert any("帧" in w for w in r["warnings"])