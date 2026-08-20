#!/usr/bin/env python3
"""深度图滤波与空洞填充

使用 OpenCV 和 scipy 对深度图进行滤波处理。
"""

import cv2
import numpy as np
from scipy.ndimage import binary_dilation, binary_fill_holes

class DepthFilter:
    """深度图滤波器"""

    def __init__(self, width=1280, height=720):
        """初始化深度滤波器

        Args:
            width: 深度图宽度
            height: 深度图高度
        """
        self.width = width
        self.height = height
        self.prev_depth = None  # 用于时间滤波

    def median_filter(self, depth, ksize=5):
        """中值滤波 - 去除孤立噪点

        Args:
            depth: 深度图（uint16，单位 mm）
            ksize: 滤波核大小（奇数）

        Returns:
            滤波后的深度图
        """
        # 将 0 值（无效深度）替换为中值，避免影响滤波
        mask = depth == 0
        if np.any(mask):
            depth_filled = cv2.medianBlur(depth, ksize)
            depth_filled[mask] = 0
        else:
            depth_filled = cv2.medianBlur(depth, ksize)
        return depth_filled

    def bilateral_filter(self, depth, d=9, sigma_color=75, sigma_space=75):
        """双边滤波 - 边缘保持平滑

        Args:
            depth: 深度图（uint16）
            d: 邻域直径
            sigma_color: 颜色空间标准差
            sigma_space: 坐标空间标准差

        Returns:
            滤波后的深度图
        """
        # 转换为 float32 进行双边滤波
        depth_f = depth.astype(np.float32)
        depth_f[depth_f == 0] = np.nan  # 无效深度设为 NaN

        # 用中值填充 NaN 后再双边滤波
        mask = np.isnan(depth_f)
        if np.any(mask):
            depth_f = self._fill_holes(depth_f)
        depth_filtered = cv2.bilateralFilter(
            depth_f.astype(np.float32), d, sigma_color, sigma_space
        )
        return depth_filtered.astype(np.uint16)

    def temporal_filter(self, depth, alpha=0.8):
        """时间滤波 - 多帧加权平均

        Args:
            depth: 当前深度图
            alpha: 历史帧权重（0-1，越大越平滑）

        Returns:
            滤波后的深度图
        """
        if self.prev_depth is None:
            self.prev_depth = depth.astype(np.float32)
            return depth

        depth_f = depth.astype(np.float32)
        # 只对有效像素进行时间滤波
        mask = (depth > 0) & (self.prev_depth > 0)
        result = self.prev_depth.copy()
        result[mask] = alpha * self.prev_depth[mask] + (1 - alpha) * depth_f[mask]
        result[~mask] = depth_f[~mask]

        self.prev_depth = result.copy()
        return result.astype(np.uint16)

    def fill_holes(self, depth, max_hole_size=1000):
        """空洞填充 - 使用形态学操作

        Args:
            depth: 深度图
            max_hole_size: 最大填充空洞面积

        Returns:
            填充后的深度图
        """
        depth_f = depth.astype(np.float32)
        depth_f[depth_f == 0] = np.nan

        # 使用最近邻插值填充空洞
        mask = np.isnan(depth_f)
        if np.any(mask):
            depth_filled = self._fill_holes(depth_f)
            # 只填充小空洞
            hole_mask = binary_dilation(mask, iterations=2)
            hole_mask = binary_fill_holes(hole_mask)
            result = depth.copy()
            result[hole_mask & mask] = depth_filled[hole_mask & mask].astype(np.uint16)
            return result
        return depth

    def _fill_holes(self, depth_f):
        """内部空洞填充方法"""
        from scipy.interpolate import griddata
        h, w = depth_f.shape
        x, y = np.meshgrid(np.arange(w), np.arange(h))
        valid = ~np.isnan(depth_f)
        if np.sum(valid) < 10:
            return depth_f
        points = np.column_stack((x[valid], y[valid]))
        values = depth_f[valid]
        # 使用最近邻插值（速度快）
        filled = griddata(points, values, (x, y), method='nearest')
        return filled

    def full_pipeline(self, depth):
        """完整滤波管线

        顺序：中值滤波 → 双边滤波 → 时间滤波 → 空洞填充

        Args:
            depth: 原始深度图

        Returns:
            处理后的深度图
        """
        depth = self.median_filter(depth, ksize=5)
        depth = self.bilateral_filter(depth)
        depth = self.temporal_filter(depth, alpha=0.8)
        depth = self.fill_holes(depth)
        return depth
