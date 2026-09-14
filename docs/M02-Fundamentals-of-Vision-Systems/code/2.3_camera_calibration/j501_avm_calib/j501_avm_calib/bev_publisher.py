# -*- coding: utf-8 -*-
"""效果可视化节点：四路去畸变预览 + BEV 拼接 + 评估覆盖层 + TF/网格标记。

几何链路（每路，预览尺度 s = preview_scale）:
  1. 原图 -> cv2.resize -> 缩放鱼眼图（用 scale_intrinsics 得到匹配的 Ks）
  2. cv2.fisheye.initUndistortRectifyMap(Ks,D,I,newK(Ks,balance)) ->
     去畸变小图（newK 与标定时 undistortPoints 用的构造一致）
  3. H 的定义域是「全分辨率去畸变图」：H · p_full = p_bev，
     缩放图坐标 p_small = s · p_full，故 H_s = H·diag(1/s,1/s,1) 后
     p_small = H_s · p_full 成立，backward warp 用
     warpPerspective(WARP_INVERSE_MAP) 从缩放去畸变图直接映射 BEV。
  4. 羽化权重：同一 warp 作用在 alpha 上 + distanceTransform，
     四路权重归一化融合。

输出话题:
  /calib/undist/<dir>      去畸变预览（叠加像素网格辅助直线度目检）
  /calib/bev/image         BEV 拼接图
  /calib/bev/eval_image    BEV + 期望/实测板角 + 误差线 + 0.25m 世界网格
  /calib/markers/bev_ground  地面网格 Marker
  /tf                      base_link -> camera_<dir>
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
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped, Point

from j501_avm_calib.config import load_config, results_dir, DIRECTIONS
from j501_avm_calib.fisheye_math import undistort_new_K, scale_intrinsics
from j501_avm_calib import homography as HOM


def _camera_pose_to_tf(R, t):
    """PnP gives X_cam=R X_base+t; return base->camera translation/quaternion."""
    R = np.asarray(R, dtype=np.float64).reshape(3, 3)
    t = np.asarray(t, dtype=np.float64).reshape(3)
    R_tf = R.T
    trans = -R_tf @ t
    qmat = R_tf
    tr = float(np.trace(qmat))
    if tr > 0:
        s = np.sqrt(tr + 1.0) * 2
        q = np.array([(qmat[2, 1] - qmat[1, 2]) / s,
                      (qmat[0, 2] - qmat[2, 0]) / s,
                      (qmat[1, 0] - qmat[0, 1]) / s, 0.25 * s])
    else:
        i = int(np.argmax(np.diag(qmat)))
        j, k = (i + 1) % 3, (i + 2) % 3
        s = np.sqrt(1.0 + qmat[i, i] - qmat[j, j] - qmat[k, k]) * 2
        qv = np.zeros(4)
        qv[i] = 0.25 * s
        qv[3] = (qmat[k, j] - qmat[j, k]) / s
        qv[j] = (qmat[j, i] + qmat[i, j]) / s
        qv[k] = (qmat[k, i] + qmat[i, k]) / s
        q = qv
    q /= max(np.linalg.norm(q), 1e-12)
    return trans, q

QOS = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=5,
                 reliability=ReliabilityPolicy.RELIABLE)


class BevPublisher(Node):
    def __init__(self, rate_hz: float = 5.0):
        super().__init__("bev_publisher")
        self.cfg = load_config()
        bev = self.cfg["bev"]
        cols, rows = self.cfg["pattern_size"]
        self.cols, self.rows = cols, rows
        self.square = float(self.cfg["chessboard"]["square_size_m"])
        self.scale = float(bev["scale_px_per_meter"])
        self.canvas = (int(bev["canvas_size"][0]), int(bev["canvas_size"][1]))
        self.cx, self.cy = self.canvas[0] / 2.0, self.canvas[1] / 2.0
        self.balance = float(bev["balance"])
        s = float(self.cfg["capture"]["preview_scale"])
        self.s = s

        rd = results_dir()
        self.intr = {}
        for d in DIRECTIONS:
            with (rd / f"{d}.json").open() as f:
                self.intr[d] = json.load(f)
        with (rd / "extrinsics.json").open() as f:
            self.extr = json.load(f)
        self.H = {d: np.asarray(m, dtype=np.float64)
                  for d, m in self.extr.get("homographies", {}).items()}

        # 每路预构: undist map（缩放图）+ 缩放后的 H（full->small 域）
        self.undist_map: dict[str, tuple] = {}
        self.H_scaled: dict[str, np.ndarray] = {}
        self.small_size: dict[str, tuple] = {}
        for d in DIRECTIONS:
            w, h = self._intr_size(d)
            sw, sh = max(1, int(w * s)), max(1, int(h * s))
            self.small_size[d] = (sw, sh)
            K = np.asarray(self.intr[d]["K"], dtype=np.float64)
            D = np.asarray(self.intr[d]["D"], dtype=np.float64)
            Ks = scale_intrinsics(K, w, h, sw, sh)
            m1, m2 = cv2.fisheye.initUndistortRectifyMap(
                Ks, D, np.eye(3), undistort_new_K(Ks, sw, sh, self.balance),
                (sw, sh), cv2.CV_16SC2)
            self.undist_map[d] = (m1, m2)
            if d in self.H:
                sx = float(sw) / float(w)
                sy = float(sh) / float(h)
                Hs = self.H[d] @ np.diag([1.0 / sx, 1.0 / sy, 1.0])
                self.H_scaled[d] = Hs
        # 羽化权重静态预计算（H 不变）
        self.weights = {d: self._feather_weight(d) for d in self.H}

        self.frames: dict[str, np.ndarray] = {d: None for d in DIRECTIONS}
        self.subs = {d: self.create_subscription(
            Image, f"/cameras/{d}/image_raw",
            (lambda m, dd=d: self._on_frame(m, dd)), QOS) for d in DIRECTIONS}
        self.pub_undist = {d: self.create_publisher(
            Image, f"/calib/undist/{d}", QOS) for d in DIRECTIONS}
        self.pub_bev = self.create_publisher(Image, "/calib/bev/image", QOS)
        self.pub_eval = self.create_publisher(
            Image, "/calib/bev/eval_image", QOS)
        self.marker_pub = self.create_publisher(
            MarkerArray, "/calib/markers/bev_ground", QOS)
        self.tfb = TransformBroadcaster(self)
        self.timer = self.create_timer(1.0 / max(rate_hz, 0.5), self._tick)
        self.get_logger().info(
            f"BEV 拼合发布器启动（preview_scale={s:.2f}，{rate_hz}Hz，"
            f"canvas={self.canvas}，已标定 {len(self.H)}/4 路）")

    def _intr_size(self, d):
        s = self.intr[d].get("image_size") or [1920, 1536]
        return int(s[0]), int(s[1])

    def _on_frame(self, msg: Image, d: str):
        if msg.encoding != "bgr8":
            return
        try:
            self.frames[d] = np.frombuffer(
                msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        except ValueError:
            pass

    # ---------------- 主循环 ----------------

    def _image_to_msg(self, img, frame_id="base_link"):
        stamp = self.get_clock().now().to_msg()
        msg = Image()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        msg.height, msg.width = img.shape[:2]
        msg.encoding = "bgr8"
        msg.is_bigendian = False
        msg.step = int(img.shape[1] * 3)
        msg.data = img.astype(np.uint8).tobytes()
        return msg

    def _resize_and_undist(self, d, frame):
        sw, sh = self.small_size[d]
        small = cv2.resize(frame, (sw, sh), interpolation=cv2.INTER_AREA)
        m1, m2 = self.undist_map[d]
        return cv2.remap(small, m1, m2, cv2.INTER_LINEAR)

    def _warp_to_bev(self, d, undist_small):
        return cv2.warpPerspective(
            undist_small, self.H_scaled[d], self.canvas,
            flags=cv2.INTER_LINEAR,
            borderMode=cv2.BORDER_CONSTANT, borderValue=0)

    def _feather_weight(self, d):
        """经同一 warp 得到 alpha 区域，distanceTransform 距离归一为权重。"""
        sw, sh = self.small_size[d]
        alpha = np.full((sh, sw), 255, dtype=np.uint8)
        warped_alpha = self._warp_to_bev(d, alpha)
        bin_a = (warped_alpha > 0).astype(np.uint8) * 255
        if not bin_a.any():
            return np.zeros(self.canvas[::-1], dtype=np.float32)
        dist = cv2.distanceTransform(bin_a, cv2.DIST_L2, 5)
        m = float(dist.max()) or 1.0
        weight = np.clip((dist / m).astype(np.float32), 0.1, 1.0)
        weight[bin_a == 0] = 0.0
        return weight

    def _tick(self):
        for d in DIRECTIONS:
            frame = self.frames[d]
            if frame is None:
                continue
            undist = self._resize_and_undist(d, frame)
            preview = undist.copy()
            # 像素网格：辅助目检残余畸变（直线应保持直）
            h, w = preview.shape[:2]
            for gx in range(0, w, max(16, w // 8)):
                cv2.line(preview, (gx, 0), (gx, h), (120, 120, 120), 1)
            for gy in range(0, h, max(16, h // 8)):
                cv2.line(preview, (0, gy), (w, gy), (120, 120, 120), 1)
            self.pub_undist[d].publish(self._image_to_msg(
                preview, frame_id=f"camera_{d}_optical"))

        if self.H and any(self.frames[d] is not None for d in self.H):
            bev = self._compose()
            if bev is not None:
                self.pub_bev.publish(self._image_to_msg(bev))
                self.pub_eval.publish(
                    self._image_to_msg(self._overlay_eval(bev)))
                self._publish_ground_markers()

    def _compose(self):
        """合成 BEV；无可用帧返回 None。使用每路羽化权重归一融合。"""
        acc = np.zeros(self.canvas[::-1] + (3,), dtype=np.float64)
        wsum = np.zeros(self.canvas[::-1], dtype=np.float64)
        any_frame = False
        for d in self.H:
            frame = self.frames[d]
            if frame is None:
                continue
            any_frame = True
            undist = self._resize_and_undist(d, frame)
            w = self.weights[d][..., None]
            acc += self._warp_to_bev(d, undist).astype(np.float64) * w
            wsum += self.weights[d]
        if not any_frame:
            return None
        nz = wsum > 1e-6
        acc[nz] /= wsum[nz][..., None]
        return np.clip(acc, 0, 255).astype(np.uint8)

    # ---------------- 评估覆盖层 ----------------

    def _overlay_eval(self, bev_img: np.ndarray) -> np.ndarray:
        out = bev_img.copy()

        def bev_pt(gx, gy):
            return (int(gx * self.scale + self.cx),
                    int(-gy * self.scale + self.cy))

        # 0.25m 世界网格（虚线）
        for gx in range(-10, 11):
            u = int(gx * 0.25 * self.scale + self.cx)
            for v in range(0, self.canvas[1], 4):
                cv2.line(out, (u, v), (u, min(v + 2, self.canvas[1] - 1)),
                         (90, 90, 90), 1)
        for gy in range(-10, 11):
            v = int(gy * 0.25 * self.scale + self.cy)
            for u in range(0, self.canvas[0], 4):
                cv2.line(out, (u, v), (min(u + 2, self.canvas[0] - 1), v),
                         (90, 90, 90), 1)

        # 期望板角（绿）+ 实测板角（红）+ 误差线（黄）
        for d in self.H:
            place = self.extr.get("placements", {}).get(d) or {}
            if not place:
                continue
            g4, _ = HOM.ground_corners(
                d, float(place["near_m"]), float(place.get("lateral_m", 0.0)),
                place.get("orient", "long-lateral"),
                self.cols, self.rows, self.square)
            exp = [bev_pt(g[0], g[1]) for g in g4]
            meas = self.extr.get("board_measured_bev", {}).get(d)
            for i in range(4):
                cv2.circle(out, exp[i], 4, (0, 200, 0), -1)
                if meas:
                    mp = (int(meas[i][0]), int(meas[i][1]))
                    cv2.circle(out, mp, 4, (0, 0, 220), -1)
                    cv2.line(out, exp[i], mp, (0, 220, 220), 2)
            cv2.putText(out, d, (exp[0][0] + 8, exp[0][1] - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        y = 24
        for d, r in (self.extr.get("rms_errors") or {}).items():
            color = ((0, 255, 0) if r < 1.0
                     else (0, 200, 255) if r < 5.0 else (0, 0, 255))
            cv2.putText(out, f"{d}: rms {r:.3f}px", (8, y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
            y += 22
        return out

    # ---------------- 3D 地面标记 + TF ----------------

    def _publish_ground_markers(self):
        now = self.get_clock().now().to_msg()
        ma = MarkerArray()
        m = Marker()
        m.header.stamp = now
        m.header.frame_id = "base_link"
        m.ns = "bev_ground"
        m.id = 0
        m.type = Marker.LINE_LIST
        m.action = Marker.ADD
        m.scale.x = 0.01
        m.color.r = m.color.g = m.color.b = 0.6
        m.color.a = 0.7
        for i in range(-10, 11):
            v = i * 0.25
            for a, b in [((v, -2.5), (v, 2.5)), ((-2.5, v), (2.5, v))]:
                for x, y in (a, b):
                    m.points.append(Point(x=float(x), y=float(y), z=0.005))
        ma.markers.append(m)
        self.marker_pub.publish(ma)
        for d, pose in (self.extr.get("poses") or {}).items():
            R = np.asarray(pose["R"], dtype=np.float64)
            t = np.asarray(pose["t"], dtype=np.float64).ravel()
            trans, quat = _camera_pose_to_tf(R, t)
            ts = TransformStamped()
            ts.header.stamp = now
            ts.header.frame_id = "base_link"
            ts.child_frame_id = f"camera_{d}"
            ts.transform.translation.x = float(trans[0])
            ts.transform.translation.y = float(trans[1])
            ts.transform.translation.z = float(trans[2])
            ts.transform.rotation.x = float(quat[0])
            ts.transform.rotation.y = float(quat[1])
            ts.transform.rotation.z = float(quat[2])
            ts.transform.rotation.w = float(quat[3])
            self.tfb.sendTransform(ts)


def main(args=None):
    parser = argparse.ArgumentParser(description="BEV 拼接 + RViz 覆盖层")
    parser.add_argument("--rate", type=float, default=5.0)
    a, _ = parser.parse_known_args(args)
    rclpy.init(args=args)
    try:
        node = BevPublisher(a.rate)
    except FileNotFoundError as exc:
        print(f"缺少标定文件: {exc}。请先完成内参解析与外参标定。")
        if rclpy.ok():
            rclpy.shutdown()
        raise SystemExit(1)
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
