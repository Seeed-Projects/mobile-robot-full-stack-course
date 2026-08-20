#!/usr/bin/env python3
"""深度精度评估脚本

在已知距离下采集深度数据，计算 RMS 误差。
"""

import pyrealsense2 as rs
import numpy as np
import time

class DepthAccuracyEvaluator:
    """深度精度评估器"""

    def __init__(self):
        """初始化 RealSense 管线"""
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
        self.pipeline.start(config)

    def measure_distance(self, known_distance_m, num_samples=100):
        """在已知距离下测量深度精度

        Args:
            known_distance_m: 已知距离（米）
            num_samples: 采样帧数

        Returns:
            dict: 包含 RMS 误差、平均误差、标准差
        """
        measurements = []

        print(f"采集 {num_samples} 帧，已知距离: {known_distance_m}m")

        for i in range(num_samples):
            frames = self.pipeline.wait_for_frames(5000)
            depth_frame = frames.get_depth_frame()

            if not depth_frame:
                continue

            depth_image = np.asanyarray(depth_frame.get_data())

            # 取中心区域 100x100 像素的平均深度
            h, w = depth_image.shape
            center_region = depth_image[
                h//2-50:h//2+50, w//2-50:w//2+50
            ]

            # 过滤无效值
            valid = center_region > 0
            if np.any(valid):
                avg_depth = np.mean(center_region[valid]) * 0.001  # mm → m
                measurements.append(avg_depth)

            if (i + 1) % 20 == 0:
                print(f"  已采集 {i+1}/{num_samples} 帧")

        if len(measurements) < 10:
            print("有效测量不足")
            return None

        measurements = np.array(measurements)

        # 计算误差
        errors = measurements - known_distance_m
        rms_error = np.sqrt(np.mean(errors**2))
        mean_error = np.mean(np.abs(errors))
        std_error = np.std(measurements)
        relative_error = (rms_error / known_distance_m) * 100

        result = {
            'known_distance_m': known_distance_m,
            'num_samples': len(measurements),
            'mean_measured_m': float(np.mean(measurements)),
            'rms_error_mm': float(rms_error * 1000),
            'mean_error_mm': float(mean_error * 1000),
            'std_mm': float(std_error * 1000),
            'relative_error_pct': float(relative_error),
        }

        return result

    def report(self, result):
        """打印精度报告"""
        if result is None:
            return
        print("\n" + "=" * 50)
        print("深度精度评估报告")
        print("=" * 50)
        print(f"已知距离:     {result['known_distance_m']:.2f} m")
        print(f"采样帧数:     {result['num_samples']}")
        print(f"平均测量值:   {result['mean_measured_m']:.4f} m")
        print(f"RMS 误差:     {result['rms_error_mm']:.2f} mm")
        print(f"平均绝对误差: {result['mean_error_mm']:.2f} mm")
        print(f"标准差:       {result['std_mm']:.2f} mm")
        print(f"相对误差:     {result['relative_error_pct']:.2f} %")
        print("=" * 50)

    def stop(self):
        """停止管线"""
        self.pipeline.stop()


if __name__ == '__main__':
    evaluator = DepthAccuracyEvaluator()

    # 在多个距离下评估
    distances = [0.5, 1.0, 2.0, 3.0, 5.0]

    for dist in distances:
        input(f"\n将目标放置在 {dist}m 处，按 Enter 继续...")
        result = evaluator.measure_distance(dist, num_samples=100)
        evaluator.report(result)

    evaluator.stop()
