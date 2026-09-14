# -*- coding: utf-8 -*-
"""合成数据工具：无相机环境下验证标定算法。"""
import numpy as np
import cv2


def make_synthetic_intrinsics(w=1280, h=960, balance=0.8):
    """构造一组典型鱼眼内参。"""
    fx = 0.31 * w
    fy = 0.31 * h * 1.001
    K = np.array([[fx, 0.0, w / 2 + 3], [0.0, fy, h / 2 - 5], [0, 0, 1]],
                 dtype=np.float64)
    D = np.array([0.09, -0.005, 0.002, 0.0005], dtype=np.float64)
    return K, D


def synth_board_points(cols=8, rows=6, square_m=0.025):
    """内角点物体坐标 (cols*rows,1,3)，板中心置于 x=0、近边 y=0。"""
    objp = np.zeros((cols * rows, 1, 3), np.float64)
    objp[:, 0, :2] = (np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
                      - np.array([(cols - 1) / 2, 0.0])) * float(square_m)
    return objp


def synth_cell_grid_points(cols=8, rows=6, square_m=0.025):
    """完整棋盘『方格网格线』顶点 (cols+2)*(rows+2) 个 = 方格 (cols+1)*(rows+1) 个。

    8x6 内角点的棋盘需要 9x7 个方格，方格网格线是 10x8。
    内角点 = 方格网格线的内部顶点（线 1..cols, 1..rows）。
    """
    cell_cols, cell_rows = cols + 2, rows + 2
    objp = np.zeros((cell_cols * cell_rows, 1, 3), np.float64)
    objp[:, 0, :2] = (np.mgrid[0:cell_cols, 0:cell_rows].T.reshape(-1, 2)
                      - np.array([(cols + 1) / 2, 0.0])) * float(square_m)
    return objp, cell_cols, cell_rows


def render_checkerboard_image(pts_2d, cell_cols, cell_rows,
                              img_w=640, img_h=480,
                              dark=30, light=235, bg=200):
    """由『方格网格线顶点』2D 投影渲染完整棋盘。

    方格数 = (cell_cols-1) x (cell_rows-1)；8x6 内角棋盘 => 9x7 方格。
    """
    img = np.full((img_h, img_w, 3), bg, dtype=np.uint8)
    pts = pts_2d.reshape(cell_rows, cell_cols, 2)
    for gy in range(cell_rows - 1):
        for gx in range(cell_cols - 1):
            quad = np.array([pts[gy, gx], pts[gy, gx + 1],
                             pts[gy + 1, gx + 1], pts[gy + 1, gx]],
                            dtype=np.int32)
            color = dark if (gx + gy) % 2 == 0 else light
            img = cv2.fillPoly(img, [quad], (color,) * 3)
    return img


def synthetic_board_view(K, D, cols=8, rows=6, square_m=0.025,
                         rvec=None, tvec=None, img_w=1280, img_h=960):
    """把地面棋盘 3D 点经鱼眼模型投影为 2D 角点（内角点）。"""
    if rvec is None:
        rvec = np.array([0.25, 0.15, -0.02], dtype=np.float64)
    if tvec is None:
        tvec = np.array([0.0, 0.9, 0.12], dtype=np.float64)
    objp = synth_board_points(cols, rows, square_m)
    pts, _ = cv2.fisheye.projectPoints(objp, rvec, tvec, K, D)
    inside = ((pts[:, 0, 0] > 0) & (pts[:, 0, 0] < img_w - 1)
              & (pts[:, 0, 1] > 0) & (pts[:, 0, 1] < img_h - 1))
    return objp, pts.astype(np.float64), inside