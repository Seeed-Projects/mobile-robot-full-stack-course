#!/usr/bin/env python3
"""手眼标定

使用 OpenCV 的 calibrateHandEye 方法求解相机到机械臂末端的变换。
"""

import cv2
import numpy as np

class HandEyeCalibrator:
    """手眼标定器"""

    def __init__(self, method=cv2.CALIB_HAND_EYE_TSAI):
        """初始化手眼标定器

        Args:
            method: 标定方法
                cv2.CALIB_HAND_EYE_TSAI     - Tsai-Lenz 方法
                cv2.CALIB_HAND_EYE_PARK     - Park 方法
                cv2.CALIB_HAND_EYE_HORAUD   - Horaud 方法
                cv2.CALIB_HAND_EYE_ANDREFF  - Andreff 方法
                cv2.CALIB_HAND_EYE_DANIILIDIS - Daniilidis 对偶四元数方法
        """
        self.method = method
        self.gripper_poses = []  # 机械臂末端位姿列表
        self.target_poses = []    # 标定板在相机中的位姿列表

    def add_sample(self, gripper_pose, target_pose):
        """添加一组标定样本

        Args:
            gripper_pose: 机械臂末端位姿（4×4 齐次变换矩阵）
            target_pose: 标定板在相机坐标系中的位姿（4×4）
        """
        self.gripper_poses.append(gripper_pose)
        self.target_poses.append(target_pose)

    def calibrate(self):
        """执行手眼标定

        Returns:
            tuple: (R_cam_ee, t_cam_ee) 相机到末端的旋转和平移
        """
        if len(self.gripper_poses) < 3:
            print("样本不足（需 ≥3 组）")
            return None

        # 提取旋转矩阵和平移向量
        R_gripper2base = []
        t_gripper2base = []
        R_target2cam = []
        t_target2cam = []

        for i in range(len(self.gripper_poses)):
            R_gripper2base.append(self.gripper_poses[i][:3, :3])
            t_gripper2base.append(self.gripper_poses[i][:3, 3])
            R_target2cam.append(self.target_poses[i][:3, :3])
            t_target2cam.append(self.target_poses[i][:3, 3])

        # 执行手眼标定
        R_cam2gripper, t_cam2gripper = cv2.calibrateHandEye(
            R_gripper2base, t_gripper2base,
            R_target2cam, t_target2cam,
            method=self.method
        )

        # 组合为 4×4 变换矩阵
        T_cam2gripper = np.eye(4)
        T_cam2gripper[:3, :3] = R_cam2gripper
        T_cam2gripper[:3, 3] = t_cam2gripper

        return T_cam2gripper

    def evaluate(self, T_cam2gripper):
        """评估标定精度

        对每组样本，验证 AX=XB 是否成立

        Args:
            T_cam2gripper: 标定结果

        Returns:
            float: 平均误差
        """
        errors = []
        for i in range(len(self.gripper_poses) - 1):
            # A = T_gripper[i]^-1 × T_gripper[i+1]
            A = np.linalg.inv(self.gripper_poses[i]) @ self.gripper_poses[i+1]
            # B = T_target[i] × T_target[i+1]^-1
            B = self.target_poses[i] @ np.linalg.inv(self.target_poses[i+1])
            # AX = XB → A @ X vs X @ B
            AX = A @ T_cam2gripper
            XB = T_cam2gripper @ B
            error = np.linalg.norm(AX - XB)
            errors.append(error)

        return np.mean(errors)

    def report(self, T_cam2gripper):
        """打印标定报告"""
        if T_cam2gripper is None:
            return

        print("\n" + "=" * 60)
        print("手眼标定报告")
        print("=" * 60)
        print(f"标定方法: {self.method}")
        print(f"样本数: {len(self.gripper_poses)}")
        print(f"\n相机到末端变换矩阵 T_cam2gripper:")
        print(f"  旋转矩阵 R:")
        for row in T_cam2gripper[:3, :3]:
            print(f"    [{row[0]:.6f}, {row[1]:.6f}, {row[2]:.6f}]")
        print(f"  平移向量 t:")
        print(f"    [{T_cam2gripper[0,3]:.6f}, "
              f"{T_cam2gripper[1,3]:.6f}, "
              f"{T_cam2gripper[2,3]:.6f}] m")

        avg_error = self.evaluate(T_cam2gripper)
        print(f"\nAX=XB 平均误差: {avg_error:.6f}")
        print("=" * 60)


if __name__ == '__main__':
    calibrator = HandEyeCalibrator(
        method=cv2.CALIB_HAND_EYE_DANIILIDIS
    )

    # 模拟数据（实际使用时替换为真实采集数据）
    np.random.seed(42)
    true_T = np.eye(4)
    true_T[:3, 3] = [0.05, -0.03, 0.10]

    for i in range(10):
        # 模拟机械臂位姿
        gripper_pose = np.eye(4)
        gripper_pose[:3, :3] = cv2.Rodrigues(
            np.random.randn(3) * 0.3
        )[0]
        gripper_pose[:3, 3] = np.random.randn(3) * 0.2

        # 模拟标定板位姿
        target_pose = np.linalg.inv(gripper_pose) @ true_T @ np.eye(4)
        target_pose[:3, :3] = cv2.Rodrigues(
            np.random.randn(3) * 0.2
        )[0]

        calibrator.add_sample(gripper_pose, target_pose)

    T_result = calibrator.calibrate()
    calibrator.report(T_result)
