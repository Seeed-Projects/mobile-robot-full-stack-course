#!/usr/bin/env python3
"""多相机同步精度评估脚本

通过比较多路相机同一时刻采集的图像，
评估帧级同步精度。
"""

import cv2
import numpy as np
import time
from collections import deque

class SyncEvaluator:
    """多相机同步精度评估器"""

    def __init__(self, num_cameras=4, buffer_size=100):
        """初始化同步评估器

        Args:
            num_cameras: 相机数量
            buffer_size: 缓冲帧数
        """
        self.num_cameras = num_cameras
        self.timestamps = [deque(maxlen=buffer_size)
                          for _ in range(num_cameras)]
        self.frames = [deque(maxlen=buffer_size)
                      for _ in range(num_cameras)]

    def add_frame(self, cam_id, frame, timestamp=None):
        """添加一帧图像和时间戳

        Args:
            cam_id: 相机编号 (0-3)
            frame: 图像数据
            timestamp: 时间戳（秒），None 则使用当前时间
        """
        if timestamp is None:
            timestamp = time.time()

        self.timestamps[cam_id].append(timestamp)
        self.frames[cam_id].append(frame)

    def compute_sync_error(self):
        """计算多相机同步误差

        Returns:
            dict: 包含最大误差、平均误差、标准差
        """
        if not all(len(ts) > 0 for ts in self.timestamps):
            return None

        # 找到所有相机共有的时间窗口
        min_len = min(len(ts) for ts in self.timestamps)

        errors = []
        for i in range(min_len):
            # 取各相机第 i 帧的时间戳
            times = [self.timestamps[c][i] for c in range(self.num_cameras)]
            # 计算最大时间差
            max_diff = max(times) - min(times)
            errors.append(max_diff * 1000)  # 转换为毫秒

        errors = np.array(errors)

        return {
            'max_error_ms': float(np.max(errors)),
            'avg_error_ms': float(np.mean(errors)),
            'std_error_ms': float(np.std(errors)),
            'p99_error_ms': float(np.percentile(errors, 99)),
        }

    def report(self):
        """生成同步精度报告"""
        result = self.compute_sync_error()
        if result is None:
            print("数据不足，无法计算同步误差")
            return

        print("=" * 50)
        print("多相机同步精度报告")
        print("=" * 50)
        print(f"相机数量: {self.num_cameras}")
        print(f"评估帧数: {len(self.timestamps[0])}")
        print(f"最大同步误差: {result['max_error_ms']:.3f} ms")
        print(f"平均同步误差: {result['avg_error_ms']:.3f} ms")
        print(f"标准差:       {result['std_error_ms']:.3f} ms")
        print(f"P99 误差:     {result['p99_error_ms']:.3f} ms")
        print("=" * 50)

        # 评估等级
        if result['avg_error_ms'] < 0.001:
            grade = "优秀（亚微秒级，PTP 同步）"
        elif result['avg_error_ms'] < 0.1:
            grade = "良好（亚毫秒级，FSYNC 同步）"
        elif result['avg_error_ms'] < 1.0:
            grade = "一般（毫秒级，软件触发）"
        else:
            grade = "差（需检查同步配置）"

        print(f"同步等级: {grade}")
