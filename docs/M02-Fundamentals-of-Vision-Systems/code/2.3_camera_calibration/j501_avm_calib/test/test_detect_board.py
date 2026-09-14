# -*- coding: utf-8 -*-
"""detect_board：合成鱼眼棋盘图像上的检测测试（SB 优先）。

注意合成板的渲染规则：8x6『内角点』的棋盘需要 9x7 个方块，
方格网格线是 10x8；只画内角之间的四边形会少外圈方块，检测器
无法完成任务。另外检测器自适应阈值块大小随图像尺寸变化，
640x480 下格宽取 ~40px 检出最稳（1400x1050 需 ~120px+）。
"""
import numpy as np
import cv2

from j501_avm_calib.detect_board import find_board_corners
from synth_utils import (make_synthetic_intrinsics, synth_cell_grid_points,
                         render_checkerboard_image)

_CV_IMG_W, _CV_IMG_H = 640, 480


def render_synth_fisheye_board(sq=0.10, z=0.45):
    """投影 8x6 内角(9x7 方格)棋盘到鱼眼图并渲染。返回 (img, cols, rows)。"""
    K, D = make_synthetic_intrinsics(_CV_IMG_W, _CV_IMG_H)
    cols, rows = 8, 6
    objp_cells, ccol, crow = synth_cell_grid_points(cols, rows, sq)
    rvec = np.array([0.10, 0.04, 0.02], dtype=np.float64)
    tvec = np.array([0.0, 0.02, z], dtype=np.float64)
    pts, _ = cv2.fisheye.projectPoints(objp_cells, rvec, tvec, K, D)
    img = render_checkerboard_image(pts, ccol, crow, _CV_IMG_W, _CV_IMG_H)
    return img, cols, rows


def test_detect_on_synthetic_fisheye():
    img, cols, rows = render_synth_fisheye_board()
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    corners = find_board_corners(gray, (cols, rows), use_sb=True,
                                 photo_retry=True, allow_classic=True)
    assert corners is not None, "合成棋盘应被检出"
    assert len(corners) == cols * rows


def test_detect_corners_close_to_truth():
    """检出的内角点应贴近投影真值（SB 自带亚像素，容差内）。"""
    K, D = make_synthetic_intrinsics(_CV_IMG_W, _CV_IMG_H)
    cols, rows = 8, 6
    sq, z = 0.10, 0.45
    rvec = np.array([0.10, 0.04, 0.02], dtype=np.float64)
    tvec = np.array([0.0, 0.02, z], dtype=np.float64)
    objp_cells, ccol, crow = synth_cell_grid_points(cols, rows, sq)
    pts_cells, _ = cv2.fisheye.projectPoints(objp_cells, rvec, tvec, K, D)
    img = render_checkerboard_image(pts_cells, ccol, crow, _CV_IMG_W,
                                    _CV_IMG_H)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    corners = find_board_corners(gray, (cols, rows), use_sb=True,
                                 photo_retry=True, allow_classic=True)
    assert corners is not None
    # 真值内角点 = 方格网格线的 1..cols, 1..rows 内部顶点
    truth = pts_cells.reshape(crow, ccol, 2)[1:rows + 1, 1:cols + 1
                                             ].reshape(-1, 2)
    err = np.linalg.norm(corners.reshape(-1, 2) - truth, axis=1)
    assert err.mean() < 3.0 and err.max() < 6.0


def test_detect_miss_on_blank():
    blank = np.full((600, 800), 128, dtype=np.uint8)
    rng = np.random.default_rng(1)
    blank = (blank + rng.integers(0, 30, blank.shape)).astype(np.uint8)
    c = find_board_corners(blank, (8, 6), use_sb=True, photo_retry=True,
                           allow_classic=True)
    assert c is None