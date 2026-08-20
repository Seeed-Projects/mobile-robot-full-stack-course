#!/usr/bin/env python3
"""BEV 拼接质量评估"""

import cv2
import numpy as np

class BEVQualityEvaluator:
    """BEV 质量评估器"""

    def __init__(self, bev_width=1000, bev_height=1000):
        """初始化评估器"""
        self.bev_width = bev_width
        self.bev_height = bev_height

    def evaluate_seam_brightness(self, bev_image, seam_positions):
        """评估接缝亮度差

        Args:
            bev_image: BEV 拼接图
            seam_positions: 接缝位置列表 [(x1, y1, x2, y2), ...]

        Returns:
            float: 平均亮度差（百分比）
        """
        gray = cv2.cvtColor(bev_image, cv2.COLOR_BGR2GRAY)
        diffs = []

        for x1, y1, x2, y2 in seam_positions:
            # 在接缝两侧取条带
            left_region = gray[max(0,y1-10):y1+10, max(0,x1-20):x1]
            right_region = gray[max(0,y1-10):y1+10, x1:min(x1+20, gray.shape[1])]

            if left_region.size > 0 and right_region.size > 0:
                left_mean = np.mean(left_region)
                right_mean = np.mean(right_region)
                diff = abs(left_mean - right_mean) / max(left_mean, 1)
                diffs.append(diff)

        return np.mean(diffs) * 100 if diffs else 0

    def evaluate_coverage(self, bev_image):
        """评估拼接覆盖率

        Args:
            bev_image: BEV 拼接图

        Returns:
            float: 覆盖率（百分比）
        """
        gray = cv2.cvtColor(bev_image, cv2.COLOR_BGR2GRAY)
        valid_pixels = np.sum(gray > 10)  # 非黑像素
        total_pixels = gray.size
        return (valid_pixels / total_pixels) * 100

    def evaluate_position_error(self, bev_image, known_positions,
                                detected_positions):
        """评估位置误差

        Args:
            bev_image: BEV 拼接图
            known_positions: 已知标志物位置 [(x, y), ...]
            detected_positions: 检测到的位置 [(x, y), ...]

        Returns:
            float: 平均位置误差（像素）
        """
        if len(known_positions) != len(detected_positions):
            return float('inf')

        errors = []
        for (kx, ky), (dx, dy) in zip(known_positions, detected_positions):
            error = np.sqrt((kx - dx)**2 + (ky - dy)**2)
            errors.append(error)

        return np.mean(errors)

    def report(self, bev_image, seam_positions=None,
               known_positions=None, detected_positions=None):
        """生成质量评估报告"""
        print("\n" + "=" * 50)
        print("BEV 拼接质量评估报告")
        print("=" * 50)

        # 覆盖率
        coverage = self.evaluate_coverage(bev_image)
        print(f"拼接覆盖率: {coverage:.1f}%")

        # 接缝亮度差
        if seam_positions:
            seam_diff = self.evaluate_seam_brightness(
                bev_image, seam_positions
            )
            print(f"接缝亮度差: {seam_diff:.2f}%")

        # 位置误差
        if known_positions and detected_positions:
            pos_error = self.evaluate_position_error(
                bev_image, known_positions, detected_positions
            )
            print(f"位置误差: {pos_error:.2f} px")

        print("=" * 50)


if __name__ == '__main__':
    evaluator = BEVQualityEvaluator()

    # 加载 BEV 图像
    bev = cv2.imread('/tmp/bev_stitched.jpg')
    if bev is not None:
        evaluator.report(bev)
