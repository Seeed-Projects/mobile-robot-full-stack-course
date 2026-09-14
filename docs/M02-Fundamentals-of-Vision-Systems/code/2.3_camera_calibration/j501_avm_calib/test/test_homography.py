# -*- coding: utf-8 -*-
"""homography：摆位/H 求解/QC/连拍均值/接缝诊断 测试。"""
import numpy as np
import cv2

import j501_avm_calib.homography as H


def test_ground_corners_front():
    g4, (bw, bh) = H.ground_corners("front", 0.35, 0.0, "long-lateral",
                                    8, 6, 0.025)
    assert abs(bw - 0.225) < 1e-9 and abs(bh - 0.175) < 1e-9
    g4 = np.asarray(g4)
    # 车体系 y 前：近边内角 y = near + s = 0.375
    assert np.allclose(g4[:, 1].min(), 0.375)
    # 长边横向：x 范围 0.0875
    assert np.allclose(g4[:, 0].min(), -0.0875)
    assert np.allclose(g4[:, 0].max(), 0.0875)


def test_ground_corners_left_axis():
    # left 视线 = (-1,0)：板在 -x 侧
    g4, _ = H.ground_corners("left", 0.35, 0.0, "long-lateral", 8, 6, 0.025)
    g4 = np.asarray(g4)
    assert np.allclose(g4[:, 0].max(), -0.375)  # 近边内角 x=-near-s


def test_best_homography_recovers_true():
    """已知 H_true(img->BEV)，用其反向生成检测角点 -> 应(近似)恢复。"""
    cols, rows, sq = 8, 6, 0.025
    scale, canvas = 100.0, (1000, 1000)
    cx, cy = 500.0, 500.0
    g4, _ = H.ground_corners("front", 0.35, 0.0, "long-lateral",
                             cols, rows, sq)
    gmap = {(0, 0): g4[0], (0, cols - 1): g4[1],
            (rows - 1, cols - 1): g4[2], (rows - 1, 0): g4[3]}
    full = H.bilinear_ground(gmap[(0, 0)], gmap[(0, cols - 1)],
                             gmap[(rows - 1, 0)], gmap[(rows - 1, cols - 1)],
                             cols, rows)
    canvas_pts = np.float32([H.ground_to_canvas(g[0], g[1], scale, cx, cy)
                             for g in full])
    # 模拟一个合理的去畸变图->BEV 真值：BEV=Ht@img
    Ht = np.array([[0.980, -0.05, 120.0],
                   [0.02, -0.99, 880.0],
                   [0.0, 0.0, 1.0]], dtype=np.float64)
    det = cv2.perspectiveTransform(
        canvas_pts.reshape(-1, 1, 2), np.linalg.inv(Ht)).reshape(-1, 2)
    H_est, rms = H.best_homography(det, g4, cols, rows, scale, canvas)
    assert H_est is not None
    # 检测点经 H_est 映射到 BEV 的误差应接近 0（已知无噪声）
    assert rms < 0.05
    assert np.allclose(H_est, Ht, atol=1e-2)


def test_best_homography_rotated_board_orientation():
    """板摆反(180°)也能正确定向。"""
    cols, rows, sq = 8, 6, 0.025
    scale, canvas = 100.0, (1000, 1000)
    cx, cy = 500.0, 500.0
    g4, _ = H.ground_corners("back", 0.35, 0.0, "long-lateral",
                             cols, rows, sq)
    gmap = {(0, 0): g4[0], (0, cols - 1): g4[1],
            (rows - 1, cols - 1): g4[2], (rows - 1, 0): g4[3]}
    full = H.bilinear_ground(gmap[(0, 0)], gmap[(0, cols - 1)],
                             gmap[(rows - 1, 0)], gmap[(rows - 1, cols - 1)],
                             cols, rows)
    canvas_pts = np.float32([H.ground_to_canvas(g[0], g[1], scale, cx, cy)
                             for g in full])
    Ht = np.array([[0.99, 0.03, 300.0], [0.01, -1.0, 900.0],
                   [0.0, 0.0, 1.0]], dtype=np.float64)
    det = cv2.perspectiveTransform(
        canvas_pts.reshape(-1, 1, 2), np.linalg.inv(Ht)).reshape(-1, 2)
    # 180° 翻转角点序
    grid = det.reshape(rows, cols, 2)
    rot = np.flip(np.flip(grid, 0), 1).reshape(-1, 2)
    H_est, rms = H.best_homography(rot, g4, cols, rows, scale, canvas)
    assert H_est is not None
    err = np.linalg.norm(
        cv2.perspectiveTransform(rot.reshape(-1, 1, 2).astype(np.float32),
                                 H_est).reshape(-1, 2) - canvas_pts, axis=1)
    # 定向正确：输入(180°翻转序)经 H_est 全量重投影应贴齐画布目标
    assert rms < 0.5 and err.mean() < 1.0


def test_analyze_homography_qc():
    # 物理合理：图像中心 -> 画布中心正前方(上方) 340 行，中心误差 160px
    H_ok = np.array([[0.6, 0.0, 140.0], [0.0, -0.6, 610.0], [0, 0, 1.0]])
    qc = H.analyze_homography(H_ok, "front", (1200, 900), (1000, 1000), None)
    assert qc["status"] == "ok", qc["warnings"]

    # 病态：2x2 块几乎奇异
    H_bad = np.array([[1e-5, 0.0, 500.0], [0.0, 1e-5, 500.0], [0, 0, 1.0]])
    qc = H.analyze_homography(H_bad, "front", (1200, 900), (1000, 1000),
                              None)
    assert qc["status"] == "bad"
    assert any("退化" in item for item in qc["failures"])

    # 翻转：中心落到视线反方向（front 轴线正前方 -> 画布下方 v>550）
    H_flip = np.array([[0.5, 0.0, 400.0], [0.0, -0.5, 790.0], [0, 0, 1.0]])
    qc = H.analyze_homography(H_flip, "front", (1200, 900), (1000, 1000),
                              None)
    assert any("反方向" in w or "贴" in w for w in qc["warnings"])


def test_average_corners_outlier():
    # 构造 5 帧几乎相同角点 + 1 帧整体偏移 30px
    base = np.random.default_rng(0).normal(0, 1, (8 * 6, 2)) + np.array(
        [[400, 300]])
    frames = [base.copy() for _ in range(6)]
    frames[5] = base + np.array([[30.0, -15.0]])
    mean, n_used, stats = H.average_corners(frames, 8, 6)
    assert stats["n_input"] == 6
    assert n_used == 5
    assert np.linalg.norm(mean - base, axis=1).mean() < 0.5


def test_seam_is_read_only_diagnostic():
    """局部棋盘只报告误差，旧兼容入口也不得生成替换 H。"""
    cols, rows = 8, 6
    g4 = [(0.0, 0.0), (0.225, 0.0), (0.225, 0.175), (0.0, 0.175)]
    from j501_avm_calib.config import SEAM_PAIRS
    assert len(SEAM_PAIRS) == 4
    # 参考 H：undist->BEV
    H_ref = np.array([[100.0, 0.0, 500.0], [0.0, -100.0, 500.0], [0, 0, 1.0]])
    H_slave_true = np.array([[100.0, 0.0, 420.0], [0.0, -100.0, 480.0],
                             [0, 0, 1.0]])
    H_slave_old = H_slave_true + np.array([[2.0, 0.1, 12.0],
                                           [0.05, -1.5, -8.0], [0, 0, 0]])
    gmap = {(0, 0): g4[0], (0, cols - 1): g4[1],
            (rows - 1, cols - 1): g4[2], (rows - 1, 0): g4[3]}
    full = H.bilinear_ground(gmap[(0, 0)], gmap[(0, cols - 1)],
                             gmap[(rows - 1, 0)], gmap[(rows - 1, cols - 1)],
                             cols, rows)
    bev_pts = np.float32([[g[0] * 100 + 500, -g[1] * 100 + 500] for g in full])
    ref_c = cv2.perspectiveTransform(
        bev_pts.reshape(-1, 1, 2), np.linalg.inv(H_ref)).reshape(-1, 2)
    # 从路角点来自真值 H（含扰动旧 H 才是被精修的对象）
    slave_c = cv2.perspectiveTransform(
        bev_pts.reshape(-1, 1, 2), np.linalg.inv(H_slave_true)
    ).reshape(-1, 2)
    before = H_slave_old.copy()
    stats = H.evaluate_seam_alignment(ref_c, slave_c, H_ref,
                                      H_slave_old, cols, rows)
    assert stats["rms_px"] > 1.0
    assert stats["max_error_px"] >= stats["mean_error_px"]
    assert np.array_equal(H_slave_old, before)
    H_new, rejected = H.refine_seam_homography(
        ref_c, slave_c, H_ref, H_slave_old, cols, rows)
    assert H_new is None
    assert "已禁用" in rejected["error"]
    assert np.array_equal(H_slave_old, before)


def test_homography_from_pose_and_global_consistency():
    K = np.array([[700.0, 0.0, 960.0],
                  [0.0, 700.0, 768.0],
                  [0.0, 0.0, 1.0]])
    pose = {
        "R": [[1.0, 0.0, 0.0],
              [0.0, 1.0, 0.0],
              [0.0, 0.0, 1.0]],
        "t": [0.0, 0.0, 1.0],
    }
    H_pose = H.homography_from_pose(pose, K, 100.0, (500.0, 500.0))
    metrics = H.homography_pose_consistency(
        H_pose, H_pose, (1920, 1536), (1000, 1000))
    assert metrics["ok"]
    assert metrics["max_error_px"] < 1e-3
    collapsed = H_pose.copy()
    collapsed[:2, :2] *= 0.1
    bad = H.homography_pose_consistency(
        collapsed, H_pose, (1920, 1536), (1000, 1000))
    assert not bad["ok"]


def test_saved_qc_detects_silent_homography_rewrite():
    original = np.array([[0.6, 0.0, 140.0],
                         [0.0, -0.6, 610.0],
                         [0.0, 0.0, 1.0]])
    saved = H.analyze_homography(
        original, "front", (1200, 900), (1000, 1000))
    assert H.homography_matches_saved_qc(
        original, "front", (1200, 900), (1000, 1000), saved)["ok"]
    rewritten = original.copy()
    rewritten[0, 0] *= 0.2
    result = H.homography_matches_saved_qc(
        rewritten, "front", (1200, 900), (1000, 1000), saved)
    assert not result["ok"]
    assert "被改写" in result["reason"]
