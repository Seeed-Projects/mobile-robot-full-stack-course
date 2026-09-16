# -*- coding: utf-8 -*-
"""无头内参自动标定节点（GUI 备选）：订阅一路图像流，实时 SB 检测，
按 ROS calibrator 的「视角多样性」goodenough 判据自动收样本，
集满后自动 cv2.fisheye.calibrate -> 写 CameraInfo YAML + 调 set_camera_info
服务落盘（与 GUI 输出同构），再做内参质量评估并打印。

用法:
    ros2 run j501_avm_calib intrinsics_auto --direction front
    ros2 run j501_avm_calib intrinsics_auto --direction front --max-views 25
"""
from __future__ import annotations

import argparse
import json

import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from sensor_msgs.srv import SetCameraInfo

from j501_avm_calib.config import load_config, camera_info_dir, results_dir
from j501_avm_calib import cam_info_io
from j501_avm_calib.detect_board import find_board_corners
from j501_avm_calib.fisheye_math import fit_inverse_polynomial
from j501_avm_calib.intrinsic_quality import (evaluate_intrinsics,
                                              print_intrinsic_report)

PARAM_NAMES = ["X", "Y", "Size", "Skew"]
# cameracalibrator 的 param_ranges（同 ROS 标定器）
PARAM_RANGES = [0.7, 0.7, 0.4, 0.5]


class IntrinsicsAutoNode(Node):
    def __init__(self, direction: str, max_views: int = 25):
        super().__init__(f"intrinsics_auto_{direction}")
        self.direction = direction
        self.max_views = max_views
        self.cfg = load_config()
        cols, rows = self.cfg["pattern_size"]
        self.cols, self.rows = cols, rows
        self.square = float(self.cfg["chessboard"]["square_size_m"])

        self.objp = np.zeros((cols * rows, 1, 3), np.float64)
        self.objp[:, 0, :2] = \
            np.mgrid[0:cols, 0:rows].T.reshape(-1, 2).astype(np.float64)
        self.objp *= self.square

        self.db: list[tuple[list[float], np.ndarray]] = []  # params, corners
        self.done = False
        self.last_frame_corners = None

        self.qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=5,
                              reliability=ReliabilityPolicy.RELIABLE)
        self.sub = self.create_subscription(
            Image, f"/cameras/{direction}/image_raw", self._on_image, self.qos)
        self.set_cam_cli = self.create_client(
            SetCameraInfo, f"/cameras/{direction}/set_camera_info")
        self.get_logger().info(
            f"自动标定 [{direction}]：请手持 {cols}x{rows} (格宽 "
            f"{self.square*1000:.0f}mm) 棋盘在画面各区域缓慢移动/翻转角度")

    def _on_image(self, msg: Image):
        if self.done:
            return
        if msg.encoding != "bgr8":
            self.get_logger().warn(f"暂只支持 bgr8，收到 {msg.encoding}")
            return
        frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(
            msg.height, msg.width, 3)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners = find_board_corners(gray, (self.cols, self.rows),
                                     use_sb=True, photo_retry=True,
                                     allow_classic=True)
        if corners is None:
            return
        c = corners.reshape(-1, 2).astype(np.float64)
        params = self._params(c, msg.width, msg.height)
        if not self._is_good_sample(params, c):
            return
        self.db.append((params, c))
        self.last_frame_corners = c
        self._print_progress()
        if self._goodenough() or len(self.db) >= self.max_views:
            self.done = True
            self._calibrate(msg.width, msg.height)

    def _params(self, corners, width, height):
        """X/Y/Size/Skew 归一化参数（移植 camera_calibration 逻辑）。"""
        c = corners.reshape(-1, 2)
        xs, ys = c[:, 0], c[:, 1]
        border = float(np.linalg.norm(c.max(axis=0) - c.min(axis=0)))
        area = cv2.contourArea(c.astype(np.float32))
        area = area if area > 0 else 1.0
        border = max(border, np.sqrt(area))
        p_x = min(1.0, max(0.0, (float(np.mean(xs)) - border / 2)
                            / float(width - border)))
        p_y = min(1.0, max(0.0, (float(np.mean(ys)) - border / 2)
                            / float(height - border)))
        p_size = float(np.sqrt(area / (width * height)))
        skew = float(np.sqrt(
            (np.sum((c[:, 0] - np.mean(xs)) * (c[:, 1] - np.mean(ys)))
             / max(len(c), 1)) / max(area, 1e-6)))
        return [p_x, p_y, p_size, skew]

    def _is_good_sample(self, params, corners):
        if not self.db:
            return True

        def pd(p1, p2):
            return sum(abs(a - b) for a, b in zip(p1, p2))

        d = min(pd(params, s[0]) for s in self.db)
        return d > 0.2

    def _goodenough(self):
        if not self.db:
            return False
        if len(self.db) >= 40:
            return True
        all_p = [s[0] for s in self.db]
        mins = list(all_p[0])
        maxs = list(all_p[0])
        for p in all_p[1:]:
            mins = [min(a, b) for a, b in zip(mins, p)]
            maxs = [max(a, b) for a, b in zip(maxs, p)]
        mins[2] = mins[3] = 0.0  # 小 Size/Skew 不奖励
        progress = [min((hi - lo) / r, 1.0) for lo, hi, r in
                    zip(mins, maxs, PARAM_RANGES)]
        return all(p >= 1.0 for p in progress)

    def _print_progress(self):
        if not self.db:
            return
        all_p = [s[0] for s in self.db]
        mins = list(all_p[0])
        maxs = list(all_p[0])
        for p in all_p[1:]:
            mins = [min(a, b) for a, b in zip(mins, p)]
            maxs = [max(a, b) for a, b in zip(maxs, p)]
        mins[2] = mins[3] = 0.0
        progress = [min((hi - lo) / r, 1.0) for lo, hi, r in
                    zip(mins, maxs, PARAM_RANGES)]
        bars = " ".join(
            f"{n}:{p*100:3.0f}%" for n, p in zip(PARAM_NAMES, progress))
        self.get_logger().info(f"[{self.direction}] 样本 {len(self.db)} | {bars}")

    def _calibrate(self, width: int, height: int):
        self.get_logger().info("样本集满，开始 fisheye 标定...")
        obj_list = [self.objp] * len(self.db)
        img_list = [c.reshape(-1, 1, 2).astype(np.float64) for _, c in self.db]
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-6)
        K = np.eye(3, dtype=np.float64)
        D = np.zeros((4, 1), dtype=np.float64)
        flags = (cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC
                 + cv2.fisheye.CALIB_FIX_SKEW)
        try:
            rms, K, D, rvecs, tvecs = cv2.fisheye.calibrate(
                obj_list, img_list, (width, height), K, D,
                flags=flags + cv2.fisheye.CALIB_CHECK_COND, criteria=criteria)
        except cv2.error as exc:
            self.get_logger().warn(f"带 CHECK_COND 失败({exc})，去标志重试...")
            rms, K, D, rvecs, tvecs = cv2.fisheye.calibrate(
                obj_list, img_list, (width, height), K, D,
                flags=flags, criteria=criteria)
        D = np.asarray(D, dtype=np.float64).reshape(-1)

        report = evaluate_intrinsics(
            K, D, float(rms), (width, height),
            obj_points=obj_list, img_points=img_list,
            rvecs=rvecs, tvecs=tvecs, model="equidistant")
        print_intrinsic_report(report, self.direction)

        # 落盘：YAML（camera_info 目录）+ JSON（calib_results）
        yaml_path = camera_info_dir() / f"{self.direction}.yaml"
        cam_info_io.write_calibration_yaml(yaml_path, self.direction,
                                           width, height, K, D, "equidistant")
        D_inv, max_err = fit_inverse_polynomial(D)
        out = results_dir()
        out.mkdir(parents=True, exist_ok=True)
        with (out / f"{self.direction}.json").open("w", encoding="utf-8") as f:
            json.dump({
                "K": K.tolist(), "D": D.tolist(), "D_inv": D_inv.tolist(),
                "rms": float(rms),
                "per_view_rms": report["per_view_rms"],
                "image_size": [width, height], "model": "equidistant",
                "d_inv_max_err": max_err,
                "evaluation": {"status": report["status"]},
            }, f, indent=2, ensure_ascii=False)

        # 同步 driver（set_camera_info 服务）
        if not self.set_cam_cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn("set_camera_info 服务不可用，仅本地落盘")
            return
        req = SetCameraInfo.Request()
        from builtin_interfaces.msg import Time
        req.camera_info.header.stamp = Time()
        req.camera_info.header.frame_id = f"camera_{self.direction}_optical"
        req.camera_info.width = width
        req.camera_info.height = height
        req.camera_info.distortion_model = "equidistant"
        req.camera_info.d = D.tolist()
        req.camera_info.k = K.ravel().tolist()
        req.camera_info.r = [1.0, 0, 0, 0, 1.0, 0, 0, 0, 1.0]
        P = np.zeros((3, 4), dtype=np.float64)
        P[:3, :3] = K
        req.camera_info.p = P.ravel().tolist()
        fut = self.set_cam_cli.call_async(req)
        fut.add_done_callback(self._commit_done)

    def _commit_done(self, fut):
        try:
            resp = fut.result()
            if resp.success:
                self.get_logger().info(f"✅ 已 COMMIT 到 driver: {resp.status_message}")
            else:
                self.get_logger().error(f"❌ COMMIT 失败: {resp.status_message}")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"COMMIT 调用异常: {exc}")


def main(args=None):
    parser = argparse.ArgumentParser(description="无头 fisheye 内参自动标定")
    parser.add_argument("--direction", required=True, choices=["front", "back",
                                                               "left", "right"])
    parser.add_argument("--max-views", type=int, default=25)
    a, _ = parser.parse_known_args(args)
    rclpy.init(args=args)
    node = IntrinsicsAutoNode(a.direction, a.max_views)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except rclpy.executors.ExternalShutdownException:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()