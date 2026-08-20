#!/usr/bin/env python3
"""相机标定重投影误差计算

使用 OpenCV 计算标定后的重投影误差。
"""

import cv2
import numpy as np
import glob

class CalibrationEvaluator:
    """标定质量评估器"""

    def __init__(self, pattern_size=(9, 6), square_size=0.025):
        """初始化标定评估器

        Args:
            pattern_size: 棋盘格内角点数（列, 行）
            square_size: 方格边长（米）
        """
        self.pattern_size = pattern_size
        self.square_size = square_size

        # 生成 3D 角点世界坐标
        self.objp = np.zeros(
            (pattern_size[0] * pattern_size[1], 3), dtype=np.float64
        )
        self.objp[:, :2] = np.mgrid[
            0:pattern_size[0], 0:pattern_size[1]
        ].T.reshape(-1, 2)
        self.objp *= square_size

    def calibrate(self, image_dir, image_extension="jpg"):
        """执行标定

        Args:
            image_dir: 标定图像目录
            image_extension: 图像扩展名

        Returns:
            tuple: (ret, K, dist, rvecs, tvecs, errors)
        """
        objpoints = []  # 3D 角点
        imgpoints = []  # 2D 角点
        image_size = None

        images = sorted(glob.glob(f"{image_dir}/*.{image_extension}"))
        print(f"找到 {len(images)} 张标定图像")

        for idx, fname in enumerate(images):
            img = cv2.imread(fname)
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

            if image_size is None:
                image_size = gray.shape[::-1]

            # 检测棋盘格角点
            ret, corners = cv2.findChessboardCorners(
                gray, self.pattern_size, None
            )

            if ret:
                # 亚像素精化
                corners2 = cv2.cornerSubPix(
                    gray, corners, (11, 11), (-1, -1),
                    (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
                )
                objpoints.append(self.objp)
                imgpoints.append(corners2)
                print(f"  [{idx+1}/{len(images)}] 角点检测成功")
            else:
                print(f"  [{idx+1}/{len(images)}] 角点检测失败: {fname}")

        if len(objpoints) < 3:
            print("有效图像不足（需 ≥3 张）")
            return None

        # 执行标定
        ret, K, dist, rvecs, tvecs = cv2.calibrateCamera(
            objpoints, imgpoints, image_size, None, None
        )

        errors = []
        for i in range(len(objpoints)):
            imgpoints_proj, _ = cv2.projectPoints(
                objpoints[i], rvecs[i], tvecs[i], K, dist
            )
            error = cv2.norm(
                imgpoints[i], imgpoints_proj, cv2.NORM_L2
            ) / len(imgpoints_proj)
            errors.append(error)

        return ret, K, dist, rvecs, tvecs, errors

    def report(self, ret, K, dist, rvecs, tvecs, errors):
        """打印单目标定报告

        Args:
            ret: 标定 RMS 误差
            K: 内参矩阵
            dist: 畸变系数
            rvecs: 旋转向量列表
            tvecs: 平移向量列表
            errors: 每张图像的重投影误差列表
        """
        print("\n" + "=" * 60)
        print("单目相机标定报告")
        print("=" * 60)
        print(f"标定 RMS 误差: {ret:.4f}")
        print(f"内参矩阵 K:")
        print(f"  fx = {K[0,0]:.4f}, fy = {K[1,1]:.4f}")
        print(f"  cx = {K[0,2]:.4f}, cy = {K[1,2]:.4f}")
        print(f"畸变系数: {dist.flatten()}")
        print(f"\n每张图像重投影误差:")
        for i, err in enumerate(errors):
            print(f"  [{i+1}] {err:.4f} px")
        print(f"平均误差: {np.mean(errors):.4f} px")
        print("=" * 60)

    def save_yaml(self, K, dist, output_path):
        """保存标定结果到 YAML

        Args:
            K: 内参矩阵
            dist: 畸变系数
            output_path: 输出文件路径
        """
        fs = cv2.FileStorage(output_path, cv2.FILE_STORAGE_WRITE)
        fs.write("K", K)
        fs.write("dist", dist)
        fs.release()
        print(f"标定结果已保存到: {output_path}")


if __name__ == '__main__':
    evaluator = CalibrationEvaluator(
        pattern_size=(9, 6), square_size=0.025
    )
    result = evaluator.calibrate("/tmp/calib_images", "jpg")
    if result is not None:
        ret, K, dist, rvecs, tvecs, errors = result
        evaluator.report(ret, K, dist, rvecs, tvecs, errors)
        evaluator.save_yaml(K, dist, "/tmp/mono_calibration.yaml")
