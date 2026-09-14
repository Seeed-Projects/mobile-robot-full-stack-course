# -*- coding: utf-8 -*-
"""外参标定向导节点：逐路地面棋盘 -> 去畸变角点 -> BEV 单应 H + QC，
再加 接缝诊断 / 多点验证位 / PnP 6DoF(TF+RViz 标记)。

流程(与参考项目一致):
  1. 逐路顺序：front -> back -> left -> right（可 skip/relock）
  2. 每路：稳定连续检出 stable_frames 帧 -> 连拍 burst_frames 帧 ->
     角点对齐均值 -> best_homography(RANSAC, 180° 歧义消除) -> H-QC
  3. 四路齐后可选「seam」接缝诊断（4 对，联合计数）
  4. 「verify <dir> <near> <lateral>」把板放到手工测量的新位置，
     实测 BEV 角点 vs 期望角的误差（强回归评估）
  5. save 合并保存 extrinsics.json（部分重标不丢旧 H）

控制台命令: skip | relock | seam | verify <dir> <near> <lateral> |
            status | save | quit
前置: calib_results/<dir>.json 四路内参已解析；相机驱动已在运行。

用法:
    ros2 run j501_avm_calib extrinsic_calibrator [--preview]
"""
from __future__ import annotations

import argparse
import json
import threading
import time
from collections import deque

import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

from j501_avm_calib.config import (load_config, results_dir, DIRECTIONS,
                                   SEAM_PAIRS)
from j501_avm_calib.fisheye_math import undistort_points_fisheye
from j501_avm_calib.detect_board import find_board_corners
from j501_avm_calib import homography as HOM

MSG_QOS = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=5,
                     reliability=ReliabilityPolicy.RELIABLE)


class ExtrinsicCalibrator(Node):
    def __init__(self, preview: bool = False):
        super().__init__("extrinsic_calibrator")
        self.cfg = load_config()
        self.preview = preview
        cols, rows = self.cfg["pattern_size"]
        self.cols, self.rows = cols, rows
        self.square = float(self.cfg["chessboard"]["square_size_m"])
        ext = self.cfg["extrinsic"]
        bev = self.cfg["bev"]
        self.qc_cfg = self.cfg["qc"]
        self.stable_need = int(ext["stable_frames"])
        self.burst_need = int(ext["burst_frames"])
        self.inview_margin = int(ext["inview_margin_px"])
        self.avm_bev = bev

        # 内参加载
        self.cams: dict[str, dict] = {}
        res_dir = results_dir()
        for d in DIRECTIONS:
            p = res_dir / f"{d}.json"
            if not p.is_file():
                self.get_logger().error(
                    f"缺少 {p}，请先完成内在标定与 intrinsics_parse")
                raise RuntimeError(f"missing intrinsics: {p}")
            with p.open("r") as f:
                data = json.load(f)
            if data.get("rms") is None:
                self.get_logger().warn(f"[{d}] rms 未知（GUI 未供给 tar）")
            self.cams[d] = data

        # 订阅 + 帧环缓存（连拍数据源；驱动独占设备，这里不直接开相机）
        self.frames: dict[str, deque] = {d: deque(maxlen=self.burst_need * 3)
                                         for d in DIRECTIONS}
        import sys
        self.auto_exit = not sys.stdin.isatty()  # launch/无终端 -> 自动模式
        self.subs = {}
        for d in DIRECTIONS:
            self.subs[d] = self.create_subscription(
                Image, f"/cameras/{d}/image_raw",
                (lambda msg, dd=d: self._on_image(msg, dd)), MSG_QOS)

        # 标定状态
        self.H: dict[str, np.ndarray] = {}          # 已锁 H
        self.qc: dict[str, dict] = {}
        self.rms_errors: dict[str, float] = {}
        self.burst_stats: dict[str, dict] = {}
        self.poses: dict[str, dict] = {}
        self.measured_bev: dict[str, list] = {}     # 外层 4 角点实测 BEV px
        self.seam_history: list[dict] = []
        self.verifications: list[dict] = []

        self.target: str | None = "front"           # 当前标定目标
        self.streak = 0
        self.last_detect_ts = 0.0
        self.bursting = False
        # 接缝模式
        self.seam_mode = False
        self.seam_pair_i = 0
        self.seam_streak = 0
        self.seam_last_seen = {}
        self.seam_last_refine = 0.0
        self.fresh_corners: dict[str, np.ndarray] = {}
        self.fresh_ts: dict[str, float] = {}

        # 可视化
        self.tfb = TransformBroadcaster(self)
        self.marker_pub = self.create_publisher(
            MarkerArray, "/calib/markers/boards", MSG_QOS)
        self.tick = self.create_timer(1.0 / 5.0, self._tick)
        self.viz_timer = self.create_timer(0.5, self._publish_viz)

        self.get_logger().info(
            f"外参向导启动。当前目标: {self.target}。摆好地面棋盘后自动锁定。")
        self.get_logger().info(
            "命令: skip | relock | seam | verify <dir> <near_m> <lateral_m> "
            "| status | save | quit")

    # ---------------- 回调/基础设施 ----------------

    def _on_image(self, msg: Image, d: str):
        if msg.encoding != "bgr8":
            return
        buf = self.frames[d]
        try:
            frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                msg.height, msg.width, 3)
        except ValueError:
            return
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        buf.append((frame, stamp))

    def _camera_size(self, d: str):
        s = self.cams[d].get("image_size") or [1920, 1536]
        return int(s[0]), int(s[1])

    def _undist_corners(self, d: str, corners):
        K = np.asarray(self.cams[d]["K"], dtype=np.float64)
        D = np.asarray(self.cams[d]["D"], dtype=np.float64)
        w, h = self._camera_size(d)
        b = float(self.avm_bev["balance"])
        return undistort_points_fisheye(corners, K, D, w, h, b)

    def _inview_ok(self, d: str, corners_undist) -> tuple[bool, int]:
        w, h = self._camera_size(d)
        pts = np.asarray(corners_undist).reshape(-1, 2)
        m = self.inview_margin
        inside = ((pts[:, 0] >= m) & (pts[:, 0] <= w - 1 - m)
                  & (pts[:, 1] >= m) & (pts[:, 1] <= h - 1 - m))
        return bool(inside.all()), int(inside.sum())

    def _detect(self, d: str):
        """目标路最新帧上做 SB+光度重试检测，返回去畸变角点(N,2) 或 None。"""
        buf = self.frames[d]
        if not buf:
            return None
        frame, _ = buf[-1]
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners = find_board_corners(gray, (self.cols, self.rows),
                                     use_sb=True, photo_retry=True,
                                     allow_classic=True)
        if corners is None:
            return None
        und = self._undist_corners(d, corners)
        ok, n_in = self._inview_ok(d, und)
        if not ok:
            self.get_logger().info(
                f"[{d}] NOT FULLY IN VIEW {n_in}/{self.cols * self.rows}",
                throttle_duration_sec=3.0)
            return None
        return np.asarray(und, dtype=np.float64).reshape(-1, 2)

    # ---------------- 主状态机（5Hz） ----------------

    def _tick(self):
        if self.seam_mode:
            self._tick_seam()
            return
        if self.target is None or self.bursting:
            return
        if self.target in self.H:
            self._advance_target()
            return
        now = time.time()
        corners = self._detect(self.target)
        if corners is None:
            if now - self.last_detect_ts > 3.0:
                self.streak = 0
            return
        self.last_detect_ts = now
        self.streak += 1
        if self.streak >= self.stable_need:
            self.get_logger().info(
                f"[{self.target}] READY({self.streak}) -> 连拍 {self.burst_need} 帧")
            self.bursting = True
            threading.Thread(target=self._do_burst, args=(self.target,),
                             daemon=True).start()

    def _advance_target(self):
        order = [d for d in DIRECTIONS if d not in self.H]
        self.target = order[0] if order else None
        self.streak = 0
        if self.target:
            self.get_logger().info(f"→ 当前目标: {self.target}")
        else:
            self.get_logger().info(
                "✅ 四路已全部锁定。可输入 seam（接缝诊断）/ verify / save / quit")
            if self.auto_exit:
                self.get_logger().info("自动模式：四路锁定完成，退出。")
                if rclpy.ok():
                    rclpy.shutdown()

    def _do_burst(self, d: str):
        """连拍 burst_need 帧 -> 角点均值 -> H -> QC -> PnP -> 保存。"""
        try:
            views = []
            buf = self.frames[d]
            frames = list(buf)[-self.burst_need:]
            for frame, _stamp in frames:
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                corners = find_board_corners(gray, (self.cols, self.rows),
                                             use_sb=True, photo_retry=True,
                                             allow_classic=True)
                if corners is None:
                    continue
                und = self._undist_corners(d, corners)
                ok, _n = self._inview_ok(d, und)
                if ok:
                    views.append(np.asarray(und, dtype=np.float64
                                            ).reshape(-1, 2))
            if len(views) < max(2, self.burst_need // 4):
                self.get_logger().error(
                    f"[{d}] 连拍检出不足 {len(views)}/{self.burst_need}，"
                    "retry...")
                self.bursting = False
                self.streak = max(0, self.streak - 3)
                return
            mean_corners, n_used, bstats = HOM.average_corners(
                views, self.cols, self.rows,
                outlier_rms_px=self.cfg["extrinsic"]["corner_outlier_rms_px"],
                align_max_px=self.cfg["extrinsic"]["corner_align_max_px"])
            self.get_logger().info(
                f"[{d}] 连拍完成: 输入 {bstats['n_input']} / 对齐 "
                f"{bstats['n_aligned']} / 使用 {n_used}，"
                f"帧 RMS 均值 {bstats['mean_frame_rms_px']:.4f}px，"
                f"角点抖动 {bstats['corner_jitter_px']:.4f}px")

            place = self.cfg["placements"][d]
            g4, _ = HOM.ground_corners(
                d, place["near_m"], place.get("lateral_m", 0.0),
                place.get("orient", "long-lateral"),
                self.cols, self.rows, self.square)
            bev = self.avm_bev
            H, rms = HOM.best_homography(
                mean_corners, g4, self.cols, self.rows,
                float(bev["scale_px_per_meter"]),
                (int(bev["canvas_size"][0]), int(bev["canvas_size"][1])))
            if H is None:
                self.get_logger().error(f"[{d}] H 求解失败，relock...")
                self.bursting = False
                self.streak = 0
                return
            img_size = self._camera_size(d)
            canvas = (int(bev["canvas_size"][0]), int(bev["canvas_size"][1]))
            bm = HOM.board_quad_metrics(mean_corners, self.cols, self.rows)
            qc = HOM.analyze_homography(
                H, d, img_size, canvas, bm,
                svd_min_thresh=self.qc_cfg["h_svd_min"],
                edge_span_min=self.qc_cfg["h_edge_span_min_px"],
                center_tol=self.qc_cfg["h_center_tol_px"],
                center_flip_tol=self.qc_cfg["h_center_flip_tol_px"],
                board_edge_ratio_max=self.qc_cfg["board_edge_ratio_max"])
            # PnP 6DoF
            K = np.asarray(self.cams[d]["K"], dtype=np.float64)
            w, h = img_size
            new_K = np.asarray(K, dtype=np.float64).copy()
            new_K[0, 0] *= float(bev["balance"])
            new_K[1, 1] *= float(bev["balance"])
            new_K[0, 2] = w / 2.0
            new_K[1, 2] = h / 2.0
            ground_ros = HOM.placement_ground_pts_ros(
                d, self.cols, self.rows, self.square, place)
            res = HOM.solve_camera_pose(
                mean_corners, ground_ros, new_K, d, self.cfg["pose_qc"])
            pose = None
            if res is not None:
                R, t = res
                pose = {"R": R.tolist(), "t": [float(x) for x in t.ravel()]}
            else:
                self.get_logger().warn(
                    f"[{d}] solvePnP 候选均未通过朝外安装检查，无 TF；直接 H 不受影响")

            # 实测外层 4 角 -> BEV px（供 RViz 评估覆盖层）
            outer_idx = HOM.grid_outer_idx(self.cols, self.rows)
            meas_pts = cv2.perspectiveTransform(
                mean_corners[outer_idx].reshape(-1, 1, 2).astype(np.float32),
                H.astype(np.float32)).reshape(-1, 2)

            self.H[d] = H
            self.qc[d] = qc
            self.rms_errors[d] = float(rms)
            self.burst_stats[d] = bstats
            if pose:
                self.poses[d] = pose
            self.measured_bev[d] = meas_pts.tolist()
            self._save_merged(d, pose)
            label = HOM.homography_qc_label(qc)
            self.get_logger().info(
                f"[{d}] 已锁定 H: rms={rms:.4f}px "
                f"{HOM.quality_label(rms)} | {label}")
            for wmsg in qc["warnings"]:
                self.get_logger().warn(f"[{d}] QC: {wmsg}")
            if qc["status"] == "bad":
                self.get_logger().error(
                    f"[{d}] H 病态！建议 relock 或检查摆位/内参")
            self._publish_viz()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"[{d}] 连拍异常: {exc}")
        finally:
            self.bursting = False
            self.streak = 0
            self._advance_target()

    # ---------------- 接缝诊断（联合计数） ----------------

    def _tick_seam(self):
        if self.seam_pair_i >= len(SEAM_PAIRS):
            if self.seam_mode:
                self.seam_mode = False
                self.get_logger().info("✅ 接缝诊断全部完成（4/4 对）")
            return
        pair = SEAM_PAIRS[self.seam_pair_i]
        if time.time() - self.seam_last_refine < 5.0:  # 换对冷却
            return
        ref_d, slave_d = pair
        now = time.time()
        for d in (ref_d, slave_d):
            c = self._detect(d)
            if c is not None:
                self.fresh_corners[d] = c
                self.fresh_ts[d] = now
        if (ref_d in self.fresh_corners and slave_d in self.fresh_corners
                and (now - self.fresh_ts[ref_d]
                     <= self.cfg["extrinsic"]["seam_fresh_max_age"])
                and (now - self.fresh_ts[slave_d]
                     <= self.cfg["extrinsic"]["seam_fresh_max_age"])):
            self.seam_streak += 1
            self.get_logger().info(
                f"[seam {ref_d}+{slave_d}] SYNC {self.seam_streak}/3",
                throttle_duration_sec=1.0)
            if self.seam_streak >= 3:
                self._do_seam_refine(ref_d, slave_d)
                self.seam_streak = 0
                self.fresh_corners.clear()
                self.seam_last_refine = time.time()
                self.seam_pair_i += 1
                if self.seam_pair_i < len(SEAM_PAIRS):
                    nxt = SEAM_PAIRS[self.seam_pair_i]
                    self.get_logger().info(
                        f"→ 下一对: {nxt[0]}+{nxt[1]}，把板放到重叠区")
                else:
                    self.seam_mode = False
                    self.get_logger().info("✅ 接缝诊断全部完成（4/4 对）")
        elif now - max(self.fresh_ts.get(ref_d, 0),
                       self.fresh_ts.get(slave_d, 0)) > 2.0:
            if self.seam_streak:
                self.get_logger().info("[seam] 联合计数清零")
            self.seam_streak = 0

    def _do_seam_refine(self, ref_d: str, slave_d: str):
        if ref_d not in self.H or slave_d not in self.H:
            self.get_logger().error("参考/从路 H 缺失，先完成四路锁定")
            return
        ref = self.fresh_corners[ref_d]
        slave = self.fresh_corners[slave_d]
        stats = HOM.evaluate_seam_alignment(
            ref, slave, self.H[ref_d], self.H[slave_d],
            self.cols, self.rows)
        if stats.get("error"):
            self.get_logger().error(f"[seam {ref_d}+{slave_d}] 诊断失败: "
                                    f"{stats['error']}")
            return
        self.get_logger().info(
            f"[seam {ref_d}+{slave_d}] 只读诊断 RMS={stats['rms_px']:.2f}px, "
            f"max={stats['max_error_px']:.2f}px；{stats['reason']}；正式 H 未修改")
        self._publish_viz()

    # ---------------- 多点验证位 ----------------

    def _verify_position(self, d: str, near: float, lateral: float):
        if d not in self.H:
            self.get_logger().error(f"[{d}] 尚未标定 H")
            return
        views = []
        for frame, _stamp in list(self.frames[d])[-self.burst_need:]:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners = find_board_corners(gray, (self.cols, self.rows),
                                         use_sb=True, photo_retry=True,
                                         allow_classic=True)
            if corners is None:
                continue
            und = self._undist_corners(d, corners)
            ok, _ = self._inview_ok(d, und)
            if ok:
                views.append(np.asarray(und, dtype=np.float64).reshape(-1, 2))
        if not views:
            self.get_logger().error(f"[verify {d}] 未检出棋盘")
            return
        mean, _, _ = HOM.average_corners(views, self.cols, self.rows)
        g4, _ = HOM.ground_corners(d, near, lateral,
                                   self.cfg["placements"][d].get(
                                       "orient", "long-lateral"),
                                   self.cols, self.rows, self.square)
        bev = self.avm_bev
        scale = float(bev["scale_px_per_meter"])
        cx = float(bev["canvas_size"][0]) / 2.0
        cy = float(bev["canvas_size"][1]) / 2.0
        expected = np.float32([HOM.ground_to_canvas(g[0], g[1], scale, cx, cy)
                               for g in g4])
        meas = cv2.perspectiveTransform(
            mean[HOM.grid_outer_idx(self.cols, self.rows)
                 ].reshape(-1, 1, 2).astype(np.float32),
            np.asarray(self.H[d], dtype=np.float32)).reshape(-1, 2)
        err_px = np.linalg.norm(meas - expected, axis=1)
        rec = {
            "direction": d, "near_m": float(near), "lateral_m": float(lateral),
            "error_mean_px": float(np.mean(err_px)),
            "error_max_px": float(np.max(err_px)),
            "error_mean_m": float(np.mean(err_px) / scale),
            "n_views": len(views), "ts": time.time(),
        }
        self.verifications.append(rec)
        self._save_merged(d, self.poses.get(d))
        status = ("✅" if rec["error_mean_px"] < 5.0 else "❌")
        self.get_logger().info(
            f"[verify {d}] near={near}m lateral={lateral}m -> 均值误差 "
            f"{rec['error_mean_px']:.2f}px ({rec['error_mean_m']*100:.1f}cm) "
            f"{status}")

    # ---------------- 保存（合并） ----------------

    def _save_merged(self, d: str, pose, seam_meta=None):
        bev = dict(self.avm_bev)
        bev["square_size_m"] = self.square
        HOM.save_extrinsics(
            results_dir() / "extrinsics.json", d, self.H[d],
            rms=self.rms_errors[d], qc=self.qc[d],
            burst_stats=self.burst_stats[d], pose=pose or {}, 
            placements_used={
                "near_m": float(self.cfg["placements"][d]["near_m"]),
                "lateral_m": float(self.cfg["placements"][d].get("lateral_m", 0)),
                "orient": self.cfg["placements"][d].get("orient",
                                                       "long-lateral"),
                "board_w_m": (self.cols + 1) * self.square,
                "board_h_m": (self.rows + 1) * self.square,
            },
            bev_cfg=bev, pattern=(self.cols, self.rows),
            seam_meta=seam_meta)
        # 附加上次接缝历史/验证记录/实测角点（save_extrinsics 不会覆盖）
        p = results_dir() / "extrinsics.json"
        with p.open("r") as f:
            data = json.load(f)
        data["seam_refined"] = self.seam_history
        data["verifications"] = self.verifications
        data.setdefault("board_measured_bev", {})[d] = self.measured_bev.get(d)
        with p.open("w") as f:
            json.dump(data, f, indent=2, ensure_ascii=False)

    # ---------------- RViz：TF + Marker ----------------

    def _publish_viz(self):
        now = self.get_clock().now().to_msg()
        for d, pose in self.poses.items():
            R = np.asarray(pose["R"], dtype=np.float64)
            t = np.asarray(pose["t"], dtype=np.float64).ravel()
            rvec, _ = cv2.Rodrigues(R)
            ts = TransformStamped()
            ts.header.stamp = now
            ts.header.frame_id = "base_link"
            ts.child_frame_id = f"camera_{d}"
            ts.transform.translation.x = float(t[0])
            ts.transform.translation.y = float(t[1])
            ts.transform.translation.z = float(t[2])
            ts.transform.rotation.x = float(rvec[0])
            ts.transform.rotation.y = float(rvec[1])
            ts.transform.rotation.z = float(rvec[2])
            ts.transform.rotation.w = 1.0
            self.tfb.sendTransform(ts)
        self.marker_pub.publish(self._make_markers(now))

    def _make_markers(self, now) -> MarkerArray:
        ma = MarkerArray()
        ident = 0

        def marker(ns, mtype, scale, color, pts=None, frame="base_link",
                   lifetime=0.0):
            nonlocal ident
            m = Marker()
            m.header.stamp = now
            m.header.frame_id = frame
            m.ns = ns
            m.id = ident
            ident += 1
            m.type = mtype
            m.action = Marker.ADD
            m.scale.x = scale[0]
            m.scale.y = scale[1] if len(scale) > 1 else scale[0]
            m.scale.z = scale[2] if len(scale) > 2 else scale[0]
            m.color.r, m.color.g, m.color.b, m.color.a = color
            m.lifetime.sec = int(lifetime)
            if pts is not None:
                from geometry_msgs.msg import Point
                m.points = [Point(x=float(p[0]), y=float(p[1]),
                                  z=float(p[2])) for p in pts]
            return m

        # 地面网格 ±2.5m @0.25m（base_link, z=0.005）
        grid_pts = []
        for i in range(-10, 11):
            v = i * 0.25
            grid_pts += [(v, -2.5, 0.005), (v, 2.5, 0.005),
                         (-2.5, v, 0.005), (2.5, v, 0.005)]
        m = marker("ground_grid", Marker.LINE_LIST, (0.01, 0.0, 0.0),
                   (0.4, 0.4, 0.4, 0.5), grid_pts)
        ma.markers.append(m)

        bev = self.avm_bev
        scale = float(bev["scale_px_per_meter"])
        cx = float(bev["canvas_size"][0]) / 2.0
        cy = float(bev["canvas_size"][1]) / 2.0

        def bev_to_ros(u, v):
            gx = (float(u) - cx) / scale
            gy = -(float(v) - cy) / scale
            return (gy, -gx, 0.0)  # base_link: x 前 y 左

        for d in DIRECTIONS:
            if d not in self.H:
                continue
            g4, _ = HOM.ground_corners(
                d, self.cfg["placements"][d]["near_m"],
                self.cfg["placements"][d].get("lateral_m", 0.0),
                self.cfg["placements"][d].get("orient", "long-lateral"),
                self.cols, self.rows, self.square)
            exp = [(float(g[1]), float(-g[0]), 0.01) for g in g4]
            exp_loop = exp + [exp[0]]
            m = marker(f"board_expected_{d}", Marker.LINE_STRIP,
                       (0.02, 0.0, 0.0), (0.1, 0.9, 0.2, 0.9), exp_loop)
            ma.markers.append(m)
            if d in self.measured_bev:
                meas = [bev_to_ros(u, v) for u, v in self.measured_bev[d]]
                meas_loop = meas + [meas[0]]
                m = marker(f"board_measured_{d}", Marker.LINE_STRIP,
                           (0.02, 0.0, 0.0), (0.9, 0.1, 0.1, 0.9), meas_loop)
                ma.markers.append(m)
                err_lines = []
                for ept, mpt in zip(exp, meas):
                    err_lines += [ept, mpt]
                m = marker(f"board_error_{d}", Marker.LINE_LIST,
                           (0.01, 0.0, 0.0), (1.0, 0.8, 0.0, 0.9), err_lines)
                ma.markers.append(m)
        # 相机坐标系（线长 0.15m）
        for d, pose in self.poses.items():
            t = np.asarray(pose["t"], dtype=np.float64).ravel()
            R = np.asarray(pose["R"], dtype=np.float64)
            p0 = tuple(float(x) for x in t)
            colors = [(1, 0, 0, 0.9), (0, 1, 0, 0.9), (0, 0, 1, 0.9)]
            for i in range(3):
                v = R[:, i] * 0.15 + t
                m = marker(f"cam_axis_{d}_{i}", Marker.LINE_LIST,
                           (0.01, 0.0, 0.0), colors[i],
                           [p0, tuple(float(x) for x in v)])
                ma.markers.append(m)
        return ma

    # ---------------- 控制台 ----------------

    def handle_cmd(self, line: str) -> bool:
        """返回 False 表示退出。"""
        parts = line.strip().split()
        if not parts:
            return True
        cmd = parts[0].lower()
        if cmd == "quit":
            return False
        if cmd == "save":
            self.get_logger().info(
                f"已保存 extrinsics.json（{len(self.H)} 路 H）")
            return True
        if cmd == "status":
            self._print_status()
            return True
        if cmd == "skip":
            self.streak = 0
            if self.seam_mode:
                if self.seam_pair_i + 1 < len(SEAM_PAIRS):
                    self.seam_pair_i += 1
                    self.seam_streak = 0
                    self.seam_last_refine = 0.0
                    pair = SEAM_PAIRS[self.seam_pair_i]
                    self.get_logger().info(
                        f"→ 跳到下一对: {pair[0]}+{pair[1]}")
                else:
                    self.seam_mode = False
                    self.get_logger().info("退出接缝模式")
                return True
            self._advance_target()
            if not self.target and not self.seam_mode:
                self.get_logger().info("已无未锁定目标")
            return True
        if cmd == "relock":
            if self.target in self.H:
                self.get_logger().warn("当前目标已锁定；请用 skip 后回退重锁")
                return True
            self.streak = 0
            self.get_logger().info(f"[{self.target}] 重新计数")
            return True
        if cmd == "seam":
            if len(self.H) < 4:
                self.get_logger().error("接缝诊断需要四路 H 齐备")
                return True
            self.seam_mode = True
            self.seam_pair_i = 0
            self.seam_streak = 0
            self.seam_last_refine = 0.0
            self.fresh_corners.clear()
            pair = SEAM_PAIRS[0]
            self.get_logger().info(
                f"进入接缝诊断：把棋盘放到 {pair[0]}+{pair[1]} 重叠区，"
                "两路同步达标后自动生成只读诊断，正式 H 不会修改")
            return True
        if cmd == "verify":
            if len(parts) != 4:
                self.get_logger().error("用法: verify <front|back|left|right> "
                                        "<near_m> <lateral_m>")
                return True
            try:
                d = parts[1]
                if d not in DIRECTIONS:
                    raise ValueError(d)
                near = float(parts[2])
                lateral = float(parts[3])
                threading.Thread(target=self._verify_position,
                                 args=(d, near, lateral), daemon=True).start()
            except ValueError:
                self.get_logger().error("参数格式错误")
            return True
        self.get_logger().warn(f"未知命令: {line}")
        return True

    def _print_status(self):
        print("-" * 60)
        for d in DIRECTIONS:
            if d in self.H:
                qc = self.qc[d]
                print(f"  {d:6s} ✅ rms={self.rms_errors[d]:.4f}px "
                      f"{HOM.homography_qc_label(qc)}")
            elif d == self.target:
                print(f"  {d:6s} ⏳ 检测计数 {self.streak}/{self.stable_need}")
            else:
                print(f"  {d:6s} ⏸  待标定")
        if self.seam_history:
            print(f"  接缝诊断: {len(self.seam_history)} 次")
        if self.verifications:
            v = self.verifications[-1]
            print(f"  最近验证: [{v['direction']}] 误差均值 "
                  f"{v['error_mean_px']:.2f}px ({v['error_mean_m']*100:.1f}cm)")
        print("-" * 60)


def main(args=None):
    parser = argparse.ArgumentParser(description="外参标定向导")
    parser.add_argument("--preview", action="store_true",
                        help="cv2.imshow 预览当前目标路(需 DISPLAY)")
    a, _ = parser.parse_known_args(args)
    rclpy.init(args=args)
    try:
        node = ExtrinsicCalibrator(preview=a.preview)
    except RuntimeError:
        if rclpy.ok():
            rclpy.shutdown()
        raise SystemExit(1)
    import sys as _sys
    if not _sys.stdin.isatty():
        # 非交互（launch/无终端）：自动逐路锁定，四路齐后保存并退出
        node.get_logger().info(
            "无交互终端，自动模式：逐路锁定 -> 四路齐后退出。"
            "如需 seam/verify 命令，请用 ros2 run 在终端运行。")
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        except rclpy.executors.ExternalShutdownException:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        return
    spin_t = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_t.start()
    print("输入命令 (help 见启动打印): ", flush=True)
    try:
        while rclpy.ok():
            line = input()
            if not node.handle_cmd(line):
                break
    except (KeyboardInterrupt, EOFError):
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
