#!/usr/bin/env python3
"""双目立体标定

使用 OpenCV 的 stereoCalibrate 和 stereoRectify 完成双目标定。
"""

import cv2
import numpy as np
import glob

class StereoCalibrator:
    """双目立体标定器"""

    def __init__(self, pattern_size=(9, 6), square_size=0.025):
        """初始化双目标定器

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

    def _find_corners(self, image_dir, image_extension="jpg"):
        """检测棋盘格角点

        Args:
            image_dir: 图像目录
            image_extension: 图像扩展名

        Returns:
            tuple: (objpoints, imgpoints, image_size)
        """
        objpoints = []
        imgpoints = []
        image_size = None

        images = sorted(glob.glob(f"{image_dir}/*.{image_extension}"))
        print(f"找到 {len(images)} 张图像")

        for idx, fname in enumerate(images):
            img = cv2.imread(fname)
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

            if image_size is None:
                image_size = gray.shape[::-1]

            ret, corners = cv2.findChessboardCorners(
                gray, self.pattern_size, None
            )

            if ret:
                corners2 = cv2.cornerSubPix(
                    gray, corners, (11, 11), (-1, -1),
                    (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
                )
                objpoints.append(self.objp)
                imgpoints.append(corners2)
                print(f"  [{idx+1}/{len(images)}] 角点检测成功")
            else:
                print(f"  [{idx+1}/{len(images)}] 角点检测失败: {fname}")

        return objpoints, imgpoints, image_size

    def calibrate_stereo(self, left_dir, right_dir, image_size,
                         image_extension="jpg"):
        """执行双目标定

        Args:
            left_dir: 左相机图像目录
            right_dir: 右相机图像目录
            image_size: 图像尺寸 (width, height)
            image_extension: 图像扩展名

        Returns:
            dict: 包含所有标定结果
        """
        # 检测左右角点
        objpoints_l, imgpoints_l, _ = self._find_corners(left_dir, image_extension)
        objpoints_r, imgpoints_r, _ = self._find_corners(right_dir, image_extension)

        # 取共同检测成功的图像
        n = min(len(imgpoints_l), len(imgpoints_r))
        if n < 3:
            print("有效图像对不足（需 ≥3 对）")
            return None

        objpoints_l = objpoints_l[:n]
        imgpoints_l = imgpoints_l[:n]
        objpoints_r = objpoints_r[:n]
        imgpoints_r = imgpoints_r[:n]

        # 分别标定左右相机内参
        _, K_l, dist_l, _, _ = cv2.calibrateCamera(
            objpoints_l, imgpoints_l, image_size, None, None
        )
        _, K_r, dist_r, _, _ = cv2.calibrateCamera(
            objpoints_r, imgpoints_r, image_size, None, None
        )

        # 双目标定
        ret, K_l, dist_l, K_r, dist_r, R, T, E, F = cv2.stereoCalibrate(
            objpoints_l, imgpoints_l, imgpoints_r,
            K_l, dist_l, K_r, dist_r,
            image_size,
            criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-6),
            flags=cv2.CALIB_FIX_INTRINSIC
        )

        # 极线校正
        R_left, R_right, P_left, P_right, Q, _, _ = cv2.stereoRectify(
            K_l, dist_l, K_r, dist_r, image_size, R, T
        )

        # 计算极线误差
        epipolar_error = self._compute_epipolar_error(
            imgpoints_l, imgpoints_r, F
        )

        result = {
            'K_left': K_l, 'dist_left': dist_l,
            'K_right': K_r, 'dist_right': dist_r,
            'R': R, 'T': T,
            'R_left': R_left, 'R_right': R_right,
            'P_left': P_left, 'P_right': P_right,
            'Q': Q,
            'epipolar_error': epipolar_error,
            'rms': ret,
        }
        return result

    def _compute_epipolar_error(self, imgpoints_l, imgpoints_r, F):
        """计算平均极线误差

        Args:
            imgpoints_l: 左图像角点列表
            imgpoints_r: 右图像角点列表
            F: 基础矩阵

        Returns:
            float: 平均极线误差（像素）
        """
        errors = []
        for pts_l, pts_r in zip(imgpoints_l, imgpoints_r):
            for pt_l, pt_r in zip(pts_l, pts_r):
                pt_l = pt_l.reshape(-1, 1)
                pt_r = pt_r.reshape(-1, 1)
                # 极线约束: x_l^T F x_r ≈ 0
                error = abs(pt_l.T @ F @ pt_r)
                errors.append(error)
        return float(np.mean(errors))

    def report(self, result):
        """打印双目标定报告"""
        if result is None:
            return

        print("\n" + "=" * 60)
        print("双目相机标定报告")
        print("=" * 60)
        print(f"左相机内参:")
        print(f"  fx = {result['K_left'][0,0]:.4f}, fy = {result['K_left'][1,1]:.4f}")
        print(f"  cx = {result['K_left'][0,2]:.4f}, cy = {result['K_left'][1,2]:.4f}")
        print(f"右相机内参:")
        print(f"  fx = {result['K_right'][0,0]:.4f}, fy = {result['K_right'][1,1]:.4f}")
        print(f"  cx = {result['K_right'][0,2]:.4f}, cy = {result['K_right'][1,2]:.4f}")
        print(f"\n相对外参:")
        print(f"  旋转矩阵 R = {result['R'].flatten()}")
        print(f"  平移向量 T = {result['T'].flatten()}")
        print(f"  基线距离 = {np.linalg.norm(result['T']):.4f} m")
        print(f"\n极线误差: {result['epipolar_error']:.4f} px")
        print("=" * 60)

    def save_yaml(self, result, output_path):
        """保存双目标定结果"""
        fs = cv2.FileStorage(output_path, cv2.FILE_STORAGE_WRITE)
        fs.write("K_left", result['K_left'])
        fs.write("dist_left", result['dist_left'])
        fs.write("K_right", result['K_right'])
        fs.write("dist_right", result['dist_right'])
        fs.write("R", result['R'])
        fs.write("T", result['T'])
        fs.write("R_left", result['R_left'])
        fs.write("R_right", result['R_right'])
        fs.write("P_left", result['P_left'])
        fs.write("P_right", result['P_right'])
        fs.write("Q", result['Q'])
        fs.write("epipolar_error", result['epipolar_error'])
        fs.release()
        print(f"双目标定结果已保存到: {output_path}")


if __name__ == '__main__':
    calibrator = StereoCalibrator(
        pattern_size=(9, 6), square_size=0.025
    )
    result = calibrator.calibrate_stereo(
        "/tmp/stereo_left", "/tmp/stereo_right", (1280, 720)
    )
    calibrator.report(result)
    calibrator.save_yaml(result, "/tmp/stereo_calibration.yaml")
