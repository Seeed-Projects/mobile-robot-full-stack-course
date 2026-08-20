#!/usr/bin/env python3
"""从相机安装参数计算 IPM 单应性矩阵"""

import numpy as np
import cv2

class IPMCalculator:
    """逆透视变换计算器"""

    def __init__(self, K, bev_width=500, bev_height=500,
                 bev_resolution=0.01):
        """初始化 IPM 计算器

        Args:
            K: 相机内参矩阵（3×3）
            bev_width: BEV 图像宽度（像素）
            bev_height: BEV 图像高度（像素）
            bev_resolution: BEV 分辨率（米/像素）
        """
        self.K = K
        self.bev_width = bev_width
        self.bev_height = bev_height
        self.bev_resolution = bev_resolution

    def compute_homography(self, height, pitch, yaw=0, roll=0,
                           dx=0, dy=0):
        """计算 IPM 单应性矩阵

        Args:
            height: 相机安装高度（米）
            pitch: 俯仰角（度，向下为负）
            yaw: 偏航角（度）
            roll: 横滚角（度）
            dx: 水平偏移 X（米）
            dy: 水平偏移 Y（米）

        Returns:
            H: 3×3 单应性矩阵（图像 → BEV）
        """
        # 角度转弧度
        pitch_rad = np.radians(pitch)
        yaw_rad = np.radians(yaw)
        roll_rad = np.radians(roll)

        # 构造旋转矩阵
        R_yaw = np.array([
            [np.cos(yaw_rad), -np.sin(yaw_rad), 0],
            [np.sin(yaw_rad),  np.cos(yaw_rad), 0],
            [0, 0, 1]
        ])
        R_pitch = np.array([
            [np.cos(pitch_rad), 0, np.sin(pitch_rad)],
            [0, 1, 0],
            [-np.sin(pitch_rad), 0, np.cos(pitch_rad)]
        ])
        R_roll = np.array([
            [1, 0, 0],
            [0, np.cos(roll_rad), -np.sin(roll_rad)],
            [0, np.sin(roll_rad),  np.cos(roll_rad)]
        ])
        R = R_yaw @ R_pitch @ R_roll

        # 平移向量
        t = np.array([dx, dy, height])

        # 计算单应性矩阵 H = K × [r1 r2 t]
        r1 = R[:, 0]
        r2 = R[:, 1]
        H = self.K @ np.column_stack([r1, r2, t])

        # 计算 BEV 坐标系偏移
        # BEV 图像中心对应地面坐标 (0, 0)
        # BEV 像素坐标 → 地面坐标：
        #   X_ground = (u_bev - bev_width/2) * resolution
        #   Y_ground = (v_bev - bev_height/2) * resolution
        # 需要额外的平移变换

        # BEV 到地面的变换矩阵
        T_bev2ground = np.array([
            [self.bev_resolution, 0, -self.bev_width/2 * self.bev_resolution],
            [0, self.bev_resolution, -self.bev_height/2 * self.bev_resolution],
            [0, 0, 1]
        ])

        # 图像到 BEV 的变换 = (BEV→地面)^(-1) × (图像→地面)
        H_img2bev = np.linalg.inv(T_bev2ground) @ np.linalg.inv(H)

        return H_img2bev

    def warp_image(self, image, H):
        """将透视图像变换为 BEV 图像

        Args:
            image: 输入透视图像
            H: 单应性矩阵（图像 → BEV）

        Returns:
            BEV 图像
        """
        bev_image = cv2.warpPerspective(
            image, H,
            (self.bev_width, self.bev_height),
            flags=cv2.INTER_LINEAR
        )
        return bev_image

    def warp_bev_to_image(self, bev_image, H):
        """将 BEV 图像逆变换为透视图像（用于叠加）

        Args:
            bev_image: BEV 图像
            H: 单应性矩阵（图像 → BEV）

        Returns:
            透视图像
        """
        H_inv = np.linalg.inv(H)
        image = cv2.warpPerspective(
            bev_image, H_inv,
            (bev_image.shape[1], bev_image.shape[0]),
            flags=cv2.INTER_LINEAR
        )
        return image


if __name__ == '__main__':
    # 示例：计算前相机的 IPM 矩阵
    K = np.array([
        [640.0, 0, 320.0],
        [0, 640.0, 240.0],
        [0, 0, 1.0]
    ])

    ipm = IPMCalculator(K, bev_width=500, bev_height=500,
                        bev_resolution=0.01)

    # 前相机：高度 1.2m，俯仰 -45°
    H_front = ipm.compute_homography(
        height=1.2, pitch=-45, yaw=0
    )
    print(f"前相机 IPM 矩阵:\n{H_front}\n")

    # 左相机：高度 1.0m，俯仰 -45°，偏航 90°
    H_left = ipm.compute_homography(
        height=1.0, pitch=-45, yaw=90
    )
    print(f"左相机 IPM 矩阵:\n{H_left}\n")

    # 右相机：高度 1.0m，俯仰 -45°，偏航 -90°
    H_right = ipm.compute_homography(
        height=1.0, pitch=-45, yaw=-90
    )
    print(f"右相机 IPM 矩阵:\n{H_right}\n")

    # 后相机：高度 1.2m，俯仰 -45°，偏航 180°
    H_rear = ipm.compute_homography(
        height=1.2, pitch=-45, yaw=180
    )
    print(f"后相机 IPM 矩阵:\n{H_rear}\n")
