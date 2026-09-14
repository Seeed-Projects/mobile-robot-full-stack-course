#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Web 标定 Pipeline：内参 + 外参 + BEV 俯视预览，纯 Python，无需显示器。

在 Jetson 上 `python3 tools/calib_web.py`（或 `scripts/run_calib_web.sh`），
外部 PC 浏览器打开 http://<jetson-ip>:8090 即可完成全部标定操作：
  - 内参：实时画面 + 棋盘角点 overlay + 多样性进度 + fisheye.calibrate + 去畸变预览 + 落盘
  - 外参：逐路 front→back→left→right 摆板 → 连拍锁 H → 独立接缝页双路预览 → 多点验证
  - BEV：实时俯视拼接（羽化加权融合）+ 评估 overlay

设计：
  - 采集复用 camera_probe_gui.CamGrabber（按路径 open、BUFFERSIZE=1、grab+retrieve
    配对、target_fps 节流），**不走 ROS2 camera_driver**，绕开其 context 崩溃。
  - 标定数学直接 import j501_avm_calib（detect_board/fisheye_math/homography/
    intrinsic_quality/cam_info_io/config），从 src 路径 import，不触发 ament_index。
  - 结果写 camera_info/<dir>.yaml（src + install 两份）+ calib_results/<dir>.json
    + extrinsics.json，与现有 ROS2 pipeline 同格式，driver/ROS2 修好后可直接消费。
  - 单后台 state 线程（~5Hz）跑当前模式的检测/状态机；ThreadingHTTPServer 处理
    MJPEG 流 + API + 页面。耗时计算（calibrate/burst/seam/verify）丢后台线程。

安全：默认 0.0.0.0:8090 内网无鉴权；更安全可 --host 127.0.0.1 + ssh 隧道。
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
import re
import shutil
import socket
import sys
import threading
import time
from collections import deque
from datetime import datetime
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from types import SimpleNamespace
from urllib.parse import parse_qs, urlparse

# Jetson 上保留系统 OpenCV 给其它程序使用；标定网页优先加载独立安装的 CUDA 版。
_CUDA_CV_PY = Path("/home/seeed/.local/opencv-4.14.0-cuda/lib/python3.10/dist-packages/cv2/python-3.10")
if os.environ.get("J501_AVM_USE_CUDA_OPENCV", "1") != "0" and _CUDA_CV_PY.is_dir():
    sys.path.insert(0, str(_CUDA_CV_PY))
import cv2
import numpy as np

# 限制 OpenCV 内部并行（cvtColor/findChessboardCornersSB/cornerSubPix 默认用满
# 所有核 → 4 路检测+采集轻松 300%+ CPU）。标定用 2 线程足够，省出 CPU 给 MJPEG。
cv2.setNumThreads(2)

# ---------- 路径 Setup：先加 j501_avm_calib src，再加 tools 同胞 ----------
_J501_SRC = "/home/seeed/ros2_ws/src/j501_avm_calib"
if _J501_SRC not in sys.path:
    sys.path.insert(0, _J501_SRC)
_TOOLS_DIR = str(Path(__file__).resolve().parent)
if _TOOLS_DIR not in sys.path:
    sys.path.insert(0, _TOOLS_DIR)

from camera_probe_gui import (  # noqa: E402  复用已验证的抓帧/探测/落盘
    CamGrabber,
    DEFAULT_WIDTH,
    DEFAULT_HEIGHT,
    enumerate_devices,
    probe_devices,
    find_j501_yaml,
    save_json,
    load_json,
)
from j501_avm_calib.config import (  # noqa: E402
    load_config,
    camera_info_dir,
    results_dir,
    DIRECTIONS,
    SEAM_PAIRS,
    CAM_AXIS,
)
from j501_avm_calib import (  # noqa: E402
    cam_info_io,
    detect_board,
    fisheye_math,
    homography as HOM,
    intrinsic_quality,
)
import bev_advanced as ADV  # noqa: E402

# 固定结果/配置目录（不跟 cwd 走），与 ROS2 pipeline 共用。
# config.py 的 config_dir() 用 parents[2]/config，src import 下解析到 src/
# 而非 src/j501_avm_calib/，会误走 ament_index fallback（需 ROS2）。显式指定绕开。
os.environ.setdefault("J501_AVM_CALIB_RESULTS_DIR",
                      "/home/seeed/workspace/ros2_bev/calib_results")
os.environ.setdefault("J501_AVM_CALIB_CONFIG_DIR",
                      "/home/seeed/ros2_ws/src/j501_avm_calib/config")

PARAM_NAMES = ["X", "Y", "Size", "Skew"]
PARAM_RANGES = [0.7, 0.7, 0.4, 0.5]  # 同 ROS camera_calibration
MIN_INTR_SAMPLES = 15
MAX_INTR_QUARANTINE_RATIO = 0.20
MIN_INTR_EDGE_PX = 1.0
MIN_INTR_EDGE_RATIO = 0.05
MIN_INTR_CELL_AREA_PX2 = 1.0
MIN_INTR_CELL_AREA_RATIO = 0.01
BOUNDARY = "frame"
BEV_PREVIEW_SETTINGS = "bev_preview.json"
DEFAULT_BODY_SIZE_M = (0.46, 0.46)  # 宽、长；用户实测的车体占地尺寸。
SEAM_RAW_MARGIN_PX = 8
SEAM_BEV_MARGIN_PX = 2
SEAM_MIN_BEV_SPAN_PX = 10.0


def cuda_backend_available() -> bool:
    """CUDA OpenCV 必须同时有设备和本次拼接使用的算子。"""
    try:
        return bool(cv2.cuda.getCudaEnabledDeviceCount() > 0
                    and hasattr(cv2.cuda, "remap")
                    and hasattr(cv2.cuda, "warpPerspective"))
    except (AttributeError, cv2.error):
        return False

# install 侧 camera_info 目录（driver 实际读的位置，与 src 镜像写）
_INSTALL_CFG = Path(
    "/home/seeed/ros2_ws/install/j501_avm_calib/share/j501_avm_calib/config")


# =========================================================================
#  标定状态：全局单例，所有线程共享
# =========================================================================

class CalibState:
    """全局标定状态 + 三阶段逻辑。HTTP handler 与 state 线程都读写它。"""

    def __init__(self, grabbers: dict[str, CamGrabber], cfg: dict):
        self.grabbers = grabbers          # dir -> CamGrabber (已 start)
        self.cfg = cfg
        cap = cfg["capture"]
        self.width = int(cap["width"])
        self.height = int(cap["height"])
        self.cols, self.rows = cfg["pattern_size"]
        self.square = float(cfg["chessboard"]["square_size_m"])
        self.avm_bev = cfg["bev"]
        self.ext_cfg = cfg["extrinsic"]
        self.qc_cfg = cfg["qc"]
        self.pose_qc = cfg["pose_qc"]
        self.placements = cfg["placements"]

        self.mode = "idle"            # idle | intrinsics | extrinsics | bev
        self.active_dir = None       # intrinsics 当前方向
        self.last_log = ""

        # ---- 内参 ----
        self.intr_samples: dict[str, list] = {d: [] for d in DIRECTIONS}
        self.intr_rejected: dict[str, list[dict]] = {d: [] for d in DIRECTIONS}
        self.intr_results: dict[str, dict] = {}   # dir -> {K,D,rms,...} 落盘后/启动时加载
        self.intr_candidates: dict[str, dict] = {}
        self.intr_collecting: dict[str, bool] = {d: False for d in DIRECTIONS}
        self.intr_last_corners: dict[str, object] = {d: None for d in DIRECTIONS}
        self.intr_last_frame_ts: dict[str, float] = {d: 0.0 for d in DIRECTIONS}
        # direction -> (solution signature, map1, map2)。候选/正式参数变化时失效。
        self.intr_preview_maps: dict[str, tuple] = {}
        self.intr_preview_revision: dict[str, int] = {d: 0 for d in DIRECTIONS}
        # 只用于内参页面的去畸变画面观察；绝不写配置、绝不参与外参/BEV。
        self.intr_preview_balance: dict[str, float] = {
            d: float(self.avm_bev["balance"]) for d in DIRECTIONS}
        self.intr_task = None                        # 正在跑的 calibrate 线程
        self.intr_task_msg = ""
        self.intr_task_ui = None

        # ---- 外参（移植 extrinsic_calibrator 状态机）----
        self.target = None
        self.ext_running = False
        self.ext_candidate: dict[str, dict] = {}
        # 单点重标先暂存四路；四路齐全后才原子替换正式外参。
        self.ext_pending: dict[str, dict] = {}
        self.streak = 0
        self.bursting = False
        self.last_detect_ts = 0.0
        # 外参检测证据。保留原始角点用于 overlay，snapshot 只导出可 JSON 化摘要。
        self.ext_live: dict[str, dict] = {}
        self.ext_stable_ref = None
        self.ext_last_frame_id: dict[str, int] = {d: -1 for d in DIRECTIONS}
        self.ext_burst_evidence: dict[str, list[dict]] = {d: [] for d in DIRECTIONS}
        self.ext_burst_state: dict[str, dict] = {d: {"phase": "idle", "accepted": 0,
                                                     "rejected": 0, "expected": 0}
                                                  for d in DIRECTIONS}
        # 未识别时低频做一次全分辨率恢复与局部棋盘探测；不让额外的 SB 搜索
        # 占满 5Hz 状态线程。局部结果只说明少了一行/列，绝不推断用户拿错了板。
        self.ext_pattern_hint: dict[str, tuple | None] = {d: None for d in DIRECTIONS}
        self.ext_pattern_probe_ts: dict[str, float] = {d: 0.0 for d in DIRECTIONS}
        self.H: dict[str, np.ndarray] = {}
        self.qc: dict[str, dict] = {}
        self.rms_errors: dict[str, float] = {}
        # 启动时从外参文件读取“曾保存过”的目录。即使某一路内参后来
        # 更新，旧 H 不能再参与 BEV，也仍要在页面上明确告知用户它存在。
        # 不能只看 self.H：过期结果不会载入 self.H，以免被误用。
        self.ext_saved_catalog: dict[str, dict] = {}
        self.ext_results_stale = False
        self.burst_stats: dict[str, dict] = {}
        self.poses: dict[str, dict] = {}
        self.measured_bev: dict[str, list] = {}
        self.seam_history: list = []
        self.seam_diagnostics: list = []
        self.verifications: list = []
        # seam 子状态
        self.seam_mode = False
        self.seam_complete = False
        self.seam_pair_i = 0
        self.seam_streak = 0
        self.seam_last_refine = 0.0
        self.fresh_corners: dict[str, object] = {}   # dir -> undist corners
        self.fresh_ts: dict[str, float] = {}
        self.ext_task = None
        self.ext_task_msg = ""

        # ---- 多位置外参会话（候选与正式结果严格隔离）----
        self.multi_session_dir = results_dir() / "extrinsic_sessions" / "active"
        self.multi_session = ADV.load_session(self.multi_session_dir, DIRECTIONS)
        self.multi_task = None
        self.multi_task_msg = ""

        # ---- BEV 渲染缓存 ----
        self.bev_cache: dict = {}     # {undist_map, H_scaled, small_size, weights}
        self.bev_loaded_ok = False
        self.bev_error = ""
        # BEV 输出视野是会话级预览设置，不会重写外参文件中的 H 坐标系。
        self.bev_view_m = 4.0
        self.bev_grid_m = 0.0
        self.bev_display_mode = "blend"
        self.bev_backend = "cuda" if cuda_backend_available() else "cpu"
        # Web preview keeps the validated 1000px default.  Non-web consumers
        # may lower this before bev_load() to trade output detail for latency.
        self.bev_canvas_px = 1000
        self.bev_transition_m = 0.04
        self.bev_show_seams = False
        # 这只是预览叠加设置，永不写入正式内/外参文件。
        self.bev_body_size_m: tuple[float, float] | None = DEFAULT_BODY_SIZE_M
        self.bev_seam = {"state": "pending", "message": "尚未优化；当前使用稳定方向分区",
                         "reason_code": "seam_missing", "updated_at": None,
                         "gains": {d: 1.0 for d in DIRECTIONS}, "sample_count": 0,
                         "changed_px": 0}
        self.bev_seam_task = None
        # Every rendered cache is immutable after installation.  Workers keep
        # this generation to discard results produced for an obsolete view.
        self.bev_cache_version = 0
        self.bev_compare: dict[str, bytes] = {}
        self.bev_fingerprint = ""
        self.bev_preview_cache: tuple[float, bytes] | None = None
        self.bev_preview_lock = threading.Lock()
        self.bev_stats = {"fps": 0.0, "render_ms": 0.0, "frame_age_ms": None,
                          "active_dirs": [], "backend": self.bev_backend}

        self._lock = threading.RLock()

    # ---------- 通用 ----------
    def _public_seam_state(self, seam):
        """Keep old persisted states readable while exposing one API contract."""
        item = dict(seam or {})
        legacy = str(item.get("state", "pending"))
        state = {"locked": "ready", "degraded": "pending", "running": "sampling"}.get(legacy, legacy)
        if state not in ("pending", "sampling", "ready", "failed"):
            state = "failed"
        item["state"] = state
        item.setdefault("reason_code", "legacy_" + legacy if legacy != state else state)
        item.setdefault("sample_count", 0)
        item.setdefault("changed_px", 0)
        item.setdefault("pair_metrics", {})
        item.setdefault("fingerprint", self.bev_fingerprint[:12])
        return item

    def latest_frame(self, d: str):
        g = self.grabbers.get(d)
        return g.latest() if g else (None, 0.0)

    def latest_frame_info(self, d: str):
        g = self.grabbers.get(d)
        return g.latest_with_id() if g else (None, 0.0, -1)

    def log(self, msg: str):
        self.last_log = f"[{datetime.now():%H:%M:%S}] {msg}"
        print(self.last_log, flush=True)

    # ---------- 内参 ----------
    def _set_intr_message(self, code, params=None, severity="info", detail=None):
        self.intr_task_ui = {
            "code": str(code), "params": dict(params or {}),
            "severity": str(severity), "detail": detail,
        }

    def intr_sample_quality(self, corners):
        """Reject locally collapsed/folded grids before OpenCV pose bootstrap."""
        metrics = {}
        try:
            flat = np.asarray(corners, dtype=np.float64).reshape(-1, 2)
        except (TypeError, ValueError):
            return False, "invalid_shape", metrics
        expected = self.cols * self.rows
        if flat.shape != (expected, 2):
            return False, "invalid_shape", metrics
        if not np.isfinite(flat).all():
            return False, "nonfinite", metrics
        grid = flat.reshape(self.rows, self.cols, 2)
        horizontal = np.linalg.norm(np.diff(grid, axis=1), axis=2)
        vertical = np.linalg.norm(np.diff(grid, axis=0), axis=2)
        edges = np.concatenate((horizontal.ravel(), vertical.ravel()))
        edge_median = float(np.median(edges))
        edge_min = float(np.min(edges))
        metrics.update(edge_min_px=edge_min, edge_median_px=edge_median)
        if (edge_median <= 0 or edge_min < MIN_INTR_EDGE_PX or
                edge_min < edge_median * MIN_INTR_EDGE_RATIO):
            return False, "collapsed_edge", metrics

        a = grid[:-1, :-1]
        b = grid[:-1, 1:]
        c = grid[1:, 1:]
        d = grid[1:, :-1]
        cross_1 = ((b[..., 0] - a[..., 0]) * (c[..., 1] - a[..., 1]) -
                   (b[..., 1] - a[..., 1]) * (c[..., 0] - a[..., 0]))
        cross_2 = ((c[..., 0] - a[..., 0]) * (d[..., 1] - a[..., 1]) -
                   (c[..., 1] - a[..., 1]) * (d[..., 0] - a[..., 0]))
        triangles = np.concatenate((cross_1.ravel(), cross_2.ravel()))
        absolute = np.abs(triangles)
        area_median = float(np.median(absolute))
        area_min = float(np.min(absolute))
        metrics.update(cell_area_min_px2=area_min,
                       cell_area_median_px2=area_median)
        if (area_median <= 0 or area_min < MIN_INTR_CELL_AREA_PX2 or
                area_min < area_median * MIN_INTR_CELL_AREA_RATIO):
            return False, "collapsed_cell", metrics
        signs = np.sign(triangles)
        if np.any(signs != signs[0]):
            return False, "folded_grid", metrics
        return True, "ok", metrics

    def _audit_intr_samples(self, samples):
        valid, rejected = [], []
        for index, sample in enumerate(samples):
            ok, reason, metrics = self.intr_sample_quality(sample[1])
            if ok:
                valid.append((index, sample))
            else:
                rejected.append((index, sample, reason, metrics))
        return valid, rejected

    @staticmethod
    def _intr_rejected_record(index, sample, reason, metrics):
        params, corners = sample
        return {
            "source_index": int(index + 1),
            "reason": str(reason),
            "metrics": {k: float(v) for k, v in metrics.items()},
            "params": list(map(float, params)),
            "corners": np.asarray(corners, dtype=np.float64).reshape(-1, 2).tolist(),
            "rejected_at": datetime.now().isoformat(timespec="seconds"),
        }

    def intr_params(self, corners, width, height):
        """返回棋盘位置/尺寸/倾斜度；退化或非有限角点不参与采样。"""
        c = np.asarray(corners, dtype=np.float64).reshape(-1, 2)
        expected = self.cols * self.rows
        if c.shape[0] != expected or not np.isfinite(c).all():
            return None
        # scan_board 的角点按行排列：左上、右上、右下、左下。
        outer = c[[0, self.cols - 1, expected - 1,
                   (self.rows - 1) * self.cols]]
        area = float(cv2.contourArea(
            outer.astype(np.float32).reshape(-1, 1, 2)))
        if not np.isfinite(area) or area <= 1.0:
            return None
        cx, cy = np.mean(outer, axis=0)
        p_x = float(np.clip(cx / max(float(width), 1.0), 0.0, 1.0))
        p_y = float(np.clip(cy / max(float(height), 1.0), 0.0, 1.0))
        p_size = float(np.clip(
            np.sqrt(area / max(float(width * height), 1.0)), 0.0, 1.0))

        angle_errors = []
        for i, cur in enumerate(outer):
            v1 = outer[(i - 1) % 4] - cur
            v2 = outer[(i + 1) % 4] - cur
            denom = float(np.linalg.norm(v1) * np.linalg.norm(v2))
            if denom <= 1e-9:
                return None
            cos_angle = float(np.clip(np.dot(v1, v2) / denom, -1.0, 1.0))
            angle_errors.append(abs(float(np.degrees(np.arccos(cos_angle))) - 90.0))
        p_skew = float(np.clip(np.mean(angle_errors) / 45.0, 0.0, 1.0))
        params = [p_x, p_y, p_size, p_skew]
        return params if np.isfinite(params).all() else None

    def intr_is_good_sample(self, params, samples):
        if params is None or not np.isfinite(params).all():
            return False
        if not samples:
            return True
        def pd(a, b):
            return sum(abs(x - y) for x, y in zip(a, b))
        return min(pd(params, s[0]) for s in samples) > 0.2

    def intr_goodenough(self, samples):
        if not samples:
            return False
        if len(samples) >= 40:
            return True
        all_p = [s[0] for s in samples]
        mins = list(all_p[0])
        maxs = list(all_p[0])
        for p in all_p[1:]:
            mins = [min(a, b) for a, b in zip(mins, p)]
            maxs = [max(a, b) for a, b in zip(maxs, p)]
        mins[2] = mins[3] = 0.0
        progress = [min((hi - lo) / r, 1.0) for lo, hi, r in
                    zip(mins, maxs, PARAM_RANGES)]
        return all(p >= 1.0 for p in progress)

    def intr_progress(self, samples):
        if not samples:
            return [0.0] * 4
        all_p = [s[0] for s in samples]
        mins = list(all_p[0]); maxs = list(all_p[0])
        for p in all_p[1:]:
            mins = [min(a, b) for a, b in zip(mins, p)]
            maxs = [max(a, b) for a, b in zip(maxs, p)]
        mins[2] = mins[3] = 0.0
        return [min((hi - lo) / r, 1.0) for lo, hi, r in
                zip(mins, maxs, PARAM_RANGES)]

    def intr_undist_corners(self, d, corners):
        res = self.intr_results.get(d)
        if not res:
            return None
        K = np.asarray(res["K"], dtype=np.float64)
        D = np.asarray(res["D"], dtype=np.float64)
        b = float(self.avm_bev["balance"])
        return fisheye_math.undistort_points_fisheye(corners, K, D,
                                                     self.width, self.height, b)

    @staticmethod
    def _intr_solution_signature(solution):
        if not solution:
            return None
        k = tuple(np.asarray(solution["K"], dtype=np.float64).reshape(-1).tolist())
        dist = tuple(np.asarray(solution["D"], dtype=np.float64).reshape(-1).tolist())
        return k + dist

    def _intr_preview_solution(self, d):
        """候选参数优先；候选不存在时使用已保存参数。"""
        with self._lock:
            source = "candidate" if d in self.intr_candidates else "saved"
            solution = self.intr_candidates.get(d) or self.intr_results.get(d)
            return source, dict(solution) if solution else None

    def _invalidate_intr_preview(self, d):
        with self._lock:
            self.intr_preview_maps.pop(d, None)
            self.intr_preview_revision[d] += 1

    def intr_set_preview_balance(self, d, balance):
        """调整内参页的临时去畸变视野，避免误改正式 BEV 几何。"""
        if d not in DIRECTIONS:
            return False, "方向无效"
        if not np.isfinite(balance) or not (0.5 <= balance <= 1.5):
            return False, "预览缩放需在 0.50–1.50 之间"
        with self._lock:
            self.intr_preview_balance[d] = float(balance)
            self._set_intr_message(
                "intr.preview_balance",
                {"direction": d, "balance": f"{balance:.2f}"})
        self._invalidate_intr_preview(d)
        return True, (f"{d} 去畸变预览缩放已设为 {balance:.2f}；"
                      "仅影响当前预览，不影响外参或 BEV")

    def intr_undistort_preview(self, d, frame):
        _, res = self._intr_preview_solution(d)
        if not res:
            return frame
        K = np.asarray(res["K"], dtype=np.float64)
        D = np.asarray(res["D"], dtype=np.float64)
        b = float(self.intr_preview_balance.get(d, self.avm_bev["balance"]))
        signature = self._intr_solution_signature(res)
        with self._lock:
            cached = self.intr_preview_maps.get(d)
        if cached is not None and cached[0] == signature and cached[1] == b:
            m1, m2 = cached[2], cached[3]
        else:
            m1, m2 = fisheye_math.init_undistort_maps(
                K, D, self.width, self.height, b)
            with self._lock:
                self.intr_preview_maps[d] = (signature, b, m1, m2)
        return cv2.remap(frame, m1, m2, cv2.INTER_LINEAR)

    def intr_tick(self):
        """5Hz：当前 active_dir 检测棋盘，收样本（多样性判据）。"""
        d = self.active_dir
        if d is None or self.intr_task or not self.intr_collecting.get(d, False):
            return
        fr, frame_ts = self.latest_frame(d)
        if fr is None:
            self.intr_last_corners[d] = None
            return
        if frame_ts <= self.intr_last_frame_ts[d]:
            return
        self.intr_last_frame_ts[d] = frame_ts
        # scan_board 降到 960 宽检测（~4x 快于全分辨率，避免 5Hz 全分辨率
        # find_board_corners 在无棋盘时 ~300ms/帧 = 单核 100%）；角点映射回全分辨率。
        found, corners, _, _ = detect_board.scan_board(
            fr, (self.cols, self.rows), scan_width=960,
            use_sb=True, photo_retry=True)
        corners = corners if found else None
        self.intr_last_corners[d] = corners
        if corners is None:
            return
        params = self.intr_params(corners, self.width, self.height)
        if params is None:
            return
        with self._lock:
            samples = self.intr_samples[d]
            valid, _ = self._audit_intr_samples(samples)
            usable = [sample for _, sample in valid]
            if self.intr_is_good_sample(params, usable):
                sample = (params, np.array(corners, dtype=np.float64, copy=True))
                ok, reason, metrics = self.intr_sample_quality(sample[1])
                if not ok:
                    record = self._intr_rejected_record(
                        len(samples) + len(self.intr_rejected[d]), sample,
                        reason, metrics)
                    self.intr_rejected[d].append(record)
                    self.intr_rejected[d] = self.intr_rejected[d][-50:]
                    self._set_intr_message(
                        "intr.sample_quarantined",
                        {"direction": d, "reason": reason}, "warning")
                    self._save_intr_samples(d)
                    return
                samples.append(sample)
                n = len(samples)
                self.intr_task_msg = f"正在采集 {d}；已保存 {n} 张有效样本"
                self._set_intr_message(
                    "intr.capturing", {"direction": d, "count": n})
                prog = self.intr_progress(usable + [sample])
                self.log(f"[intr {d}] 样本 {n} | "
                         + " ".join(f"{nm}:{p*100:3.0f}%"
                                    for nm, p in zip(PARAM_NAMES, prog)))
                if self.intr_goodenough(usable + [sample]) or n >= 25:
                    self.log(f"[intr {d}] 样本 {n}，建议覆盖度已达到；请手动开始 Calibrate")
                self._save_intr_samples(d)

    def intr_start_calibrate(self, d):
        # 原子 check-then-act：state 线程 + HTTP POST 可能同时触发
        with self._lock:
            if self.intr_task:
                return False, "已有标定任务正在运行"
            for direction in DIRECTIONS:
                self.intr_collecting[direction] = False
            valid, rejected = self._audit_intr_samples(self.intr_samples[d])
            count = len(valid)
            if count < MIN_INTR_SAMPLES:
                self._set_intr_message(
                    "intr.not_enough_usable",
                    {"usable": count, "minimum": MIN_INTR_SAMPLES,
                     "quarantined": len(rejected)}, "error")
                return False, f"有效样本不足 {count}/{MIN_INTR_SAMPLES}"
            self.intr_task_msg = "标定中..."
            self._set_intr_message(
                "intr.calibrating",
                {"direction": d, "usable": count,
                 "quarantined": len(rejected)})
            self.intr_task = threading.Thread(target=self._intr_calibrate,
                                               args=(d,), daemon=True)
            self.intr_task.start()
        return True, f"{d} 标定中..."

    def _intr_calibrate(self, d):
        try:
            with self._lock:
                samples = list(self.intr_samples[d])
                existing_rejected = list(self.intr_rejected[d])
            audited, topology_rejected = self._audit_intr_samples(samples)
            usable = [(index, sample) for index, sample in audited]
            rejected_records = [
                self._intr_rejected_record(index, sample, reason, metrics)
                for index, sample, reason, metrics in topology_rejected
            ]
            if len(usable) < MIN_INTR_SAMPLES:
                self.intr_task_msg = f"有效样本不足 {len(usable)}/{MIN_INTR_SAMPLES}"
                self._set_intr_message(
                    "intr.not_enough_usable",
                    {"usable": len(usable), "minimum": MIN_INTR_SAMPLES,
                     "quarantined": len(rejected_records)}, "error")
                return
            objp = np.zeros((self.cols * self.rows, 1, 3), np.float64)
            objp[:, 0, :2] = np.mgrid[0:self.cols, 0:self.rows].T.reshape(
                -1, 2).astype(np.float64)
            # Uniform board scale only changes translation, not K/D or RMS.
            # Unit coordinates avoid ill-conditioned fisheye initialization.
            flags = (cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC
                     + cv2.fisheye.CALIB_FIX_SKEW
                     + cv2.fisheye.CALIB_CHECK_COND
                     + cv2.fisheye.CALIB_USE_INTRINSIC_GUESS)
            criteria = (cv2.TERM_CRITERIA_EPS
                        + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-6)

            initial_f = max(self.width, self.height) / np.pi
            initial_K = np.array([
                [initial_f, 0.0, self.width / 2.0],
                [0.0, initial_f, self.height / 2.0],
                [0.0, 0.0, 1.0],
            ], dtype=np.float64)

            max_auto_reject = max(1, int(len(samples) * MAX_INTR_QUARANTINE_RATIO))
            opencv_rejected = 0
            while True:
                obj_list = [objp] * len(usable)
                img_list = [sample[1].reshape(-1, 1, 2).astype(np.float64)
                            for _, sample in usable]
                try:
                    rms, K, D, rvecs, tvecs = cv2.fisheye.calibrate(
                        obj_list, img_list, (self.width, self.height),
                        initial_K.copy(), np.zeros((4, 1), np.float64),
                        flags=flags, criteria=criteria)
                    break
                except cv2.error as exc:
                    self.log(f"[intr {d}] OpenCV fisheye detail: {exc}")
                    match = re.search(r"input array (\d+)", str(exc))
                    if (not match or opencv_rejected >= max_auto_reject or
                            len(usable) <= MIN_INTR_SAMPLES):
                        raise RuntimeError(
                            "OpenCV could not initialize a stable fisheye solution") from exc
                    bad_position = int(match.group(1))
                    if not 0 <= bad_position < len(usable):
                        raise RuntimeError(
                            "OpenCV reported an invalid calibration view index") from exc
                    source_index, bad_sample = usable.pop(bad_position)
                    rejected_records.append(self._intr_rejected_record(
                        source_index, bad_sample, "opencv_ill_conditioned", {}))
                    opencv_rejected += 1
                    self.log(f"[intr {d}] 隔离病态样本 #{source_index + 1} 后重试")

            if len(rejected_records) > max_auto_reject:
                raise RuntimeError("Too many calibration samples were quarantined")
            D = np.asarray(D, dtype=np.float64).reshape(-1)
            report = intrinsic_quality.evaluate_intrinsics(
                K, D, float(rms), (self.width, self.height),
                obj_points=obj_list, img_points=img_list,
                rvecs=rvecs, tvecs=tvecs, model="equidistant")
            D_inv, max_err = fisheye_math.fit_inverse_polynomial(D)
            all_rejected = existing_rejected + rejected_records
            candidate = {
                "K": K.tolist(), "D": D.tolist(),
                "D_inv": D_inv.tolist(), "rms": float(rms),
                "per_view_rms": report.get("per_view_rms", []),
                "image_size": [self.width, self.height],
                "model": "equidistant", "d_inv_max_err": max_err,
                "evaluation": report, "sample_count": len(usable),
                "captured_sample_count": len(samples) + len(existing_rejected),
                "used_sample_count": len(usable),
                "quarantined_sample_count": len(all_rejected),
                "used_sample_indices": [index + 1 for index, _ in usable],
                "rejected_samples": all_rejected,
            }
            with self._lock:
                if rejected_records:
                    rejected_indices = {item["source_index"] - 1
                                        for item in rejected_records}
                    self.intr_samples[d] = [
                        sample for index, sample in enumerate(samples)
                        if index not in rejected_indices
                    ]
                    self.intr_rejected[d].extend(rejected_records)
                    self._backup_intr_samples(d)
                    self._save_intr_samples(d)
                self.intr_candidates[d] = candidate
                self.intr_collecting[d] = False
            self._invalidate_intr_preview(d)
            self.log(f"[intr {d}] ✅ 计算完成：使用 {len(usable)} 张，隔离 {len(all_rejected)} 张，RMS={rms:.3f}px {report['status']}；点击保存结果")
            self.intr_task_msg = (f"✅ {d} 计算完成：使用 {len(usable)} 张，"
                                  f"隔离 {len(all_rejected)} 张，RMS={rms:.3f}px "
                                  f"({report['status']})；已切换到矫正预览，请审核后保存")
            self._set_intr_message(
                "intr.calibrated",
                {"direction": d, "used": len(usable),
                 "quarantined": len(all_rejected),
                 "rms": round(float(rms), 4), "quality": report["status"]},
                "success" if report["status"] == "pass" else "warning")
        except Exception as exc:  # noqa: BLE001
            self.log(f"[intr {d}] 标定异常: {exc}")
            self.intr_task_msg = f"❌ {exc}"
            self._set_intr_message(
                "intr.calibration_failed", {"direction": d}, "error", str(exc))
        finally:
            with self._lock:
                self.intr_task = None

    def intr_clear(self, d, confirm=False):
        with self._lock:
            if self.intr_task:
                return False, "标定计算中，不能清空样本"
            if not confirm:
                return False, "请确认清空当前样本和未保存候选结果"
            self.intr_samples[d] = []
            self.intr_rejected[d] = []
            self.intr_candidates.pop(d, None)
            self.intr_collecting[d] = False
        self.intr_last_corners[d] = None
        self.intr_last_frame_ts[d] = 0.0
        self._invalidate_intr_preview(d)
        sp = results_dir() / f"{d}.intr_samples.json"
        try:
            if sp.exists():
                sp.unlink()
        except OSError as exc:
            self.log(f"[intr {d}] 样本缓存删除失败: {exc}")
        self.intr_task_msg = f"{d} 本轮样本和未保存候选结果已清空；正式结果保留"
        self._set_intr_message(
            "intr.samples_cleared", {"direction": d}, "success")
        self.log(f"[intr {d}] 本轮样本已清空，正式结果保留")
        return True, self.intr_task_msg

    def _backup_intr_samples(self, d):
        path = results_dir() / f"{d}.intr_samples.json"
        if not path.is_file():
            return None
        stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
        backup = path.with_name(f"{path.stem}.pre-quarantine-{stamp}.json")
        if not backup.exists():
            shutil.copy2(path, backup)
        return backup

    def _save_intr_samples(self, d):
        with self._lock:
            payload = [{"params": list(map(float, p)), "corners": np.asarray(c).reshape(-1, 2).tolist()}
                       for p, c in self.intr_samples[d]]
            rejected = list(self.intr_rejected[d])
        out = results_dir(); out.mkdir(parents=True, exist_ok=True)
        path = out / f"{d}.intr_samples.json"
        tmp = path.with_suffix(path.suffix + ".tmp")
        with tmp.open("w", encoding="utf-8") as f:
            json.dump({"image_size": [self.width, self.height],
                       "pattern_size": [self.cols, self.rows],
                       "samples": payload, "rejected_samples": rejected},
                      f, ensure_ascii=False)
            f.flush(); os.fsync(f.fileno())
        os.replace(tmp, path)

    def intr_collect(self, d, enabled):
        with self._lock:
            if self.intr_task:
                return False, "标定计算中，不能切换采集"
            for direction in DIRECTIONS:
                self.intr_collecting[direction] = bool(enabled and direction == d)
            self.mode = "intrinsics"
            self.active_dir = d
            if enabled:
                self.intr_task_msg = f"正在采集 {d}；已有样本会保留"
                self._set_intr_message(
                    "intr.capturing",
                    {"direction": d, "count": len(self.intr_samples[d])})
            else:
                self.intr_task_msg = f"{d} 采集已暂停"
                self._set_intr_message(
                    "intr.capture_paused", {"direction": d})
        return True, ("开始采集" if enabled else "已暂停采集")

    def intr_select(self, d):
        """Atomically select one camera and pause every capture flag."""
        if d not in DIRECTIONS:
            return False, "方向无效"
        with self._lock:
            if self.intr_task:
                return False, "标定计算中，不能切换相机"
            for direction in DIRECTIONS:
                self.intr_collecting[direction] = False
            self.mode = "intrinsics"
            self.active_dir = d
            self.intr_task_msg = f"已切换到 {d}；采集保持暂停"
            self._set_intr_message(
                "intr.camera_selected", {"direction": d})
            grabber = self.grabbers.get(d)
            frame, _ = self.latest_frame(d)
            available = grabber is not None and frame is not None
            device = (grabber.device if grabber is not None else
                      f"/dev/video{int(self.cfg['capture']['cameras'][d]['device'])}")
        return True, {"msg": self.intr_task_msg, "available": available,
                      "device": device}

    def intr_save_candidate(self, d, confirm=False):
        with self._lock:
            candidate = self.intr_candidates.get(d)
            if not candidate:
                return False, "没有候选结果，请先计算"
            status = (candidate.get("evaluation") or {}).get("status", "fail")
            if status == "fail":
                return False, "质量评估不合格，不能保存"
            if status == "warn" and not confirm:
                return False, "质量评估为 warn；确认后再次保存"
            result = dict(candidate)
        out = results_dir(); out.mkdir(parents=True, exist_ok=True)
        yaml_dir = camera_info_dir(); yaml_dir.mkdir(parents=True, exist_ok=True)
        yp = yaml_dir / f"{d}.yaml"
        cam_info_io.write_calibration_yaml(yp, d, self.width, self.height,
                                           np.asarray(result["K"]), np.asarray(result["D"]), "equidistant")
        if _INSTALL_CFG.exists():
            inst_dir = _INSTALL_CFG / "camera_info"; inst_dir.mkdir(parents=True, exist_ok=True)
            shutil.copy2(yp, inst_dir / f"{d}.yaml")
        jp = out / f"{d}.json"
        tmp = jp.with_suffix(jp.suffix + ".tmp")
        with tmp.open("w", encoding="utf-8") as f:
            json.dump(result, f, indent=2, ensure_ascii=False)
            f.flush(); os.fsync(f.fileno())
        os.replace(tmp, jp)
        with self._lock:
            self.intr_results[d] = {k: result[k] for k in (
                "K", "D", "image_size", "rms", "model", "evaluation",
                "sample_count") if k in result}
            self.intr_candidates.pop(d, None)
            # 新内参与旧外参不再属于同一标定版本，禁止继续拼接旧 H。
            self.H.clear(); self.ext_candidate.clear(); self.ext_pending.clear()
            self.qc.clear()
            self.rms_errors.clear(); self.burst_stats.clear(); self.poses.clear()
            self.measured_bev.clear(); self.seam_history.clear()
            self.seam_diagnostics.clear(); self.verifications.clear()
            self.bev_loaded_ok = False; self.bev_cache = {}
            self._mark_multi_session_stale(
                f"{d} 内参已更新；旧多点观测不能与新内参混用")
        pending_path = out / "extrinsics.single_pending.json"
        if pending_path.exists():
            pending_path.unlink()
        stale = out / "extrinsics.stale.json"
        with stale.open("w", encoding="utf-8") as f:
            # 外参按相机方向独立依赖对应内参。新内参落盘后，旧 H 都先
            # 以保守方式标为过期；后续逐路重标会记录 fresh_directions，
            # 让网页能区分“历史保存”和“已按当前内参重新保存”。
            json.dump({"reason": "intrinsic updated", "direction": d,
                       "ts": time.time(), "fresh_directions": []}, f,
                      ensure_ascii=False)
        self.log(f"[intr {d}] 已保存正式结果 {jp.name}；样本保留，可继续补采")
        self.intr_task_msg = f"✅ {d} 已保存正式结果；样本保留，可继续补采"
        self._set_intr_message(
            "intr.saved", {"direction": d}, "success")
        return True, self.intr_task_msg

    # ---------- 外参 ----------
    def _ext_camera_KD(self, d):
        res = self.intr_results.get(d)
        if not res:
            return None, None
        return np.asarray(res["K"], dtype=np.float64), np.asarray(res["D"], dtype=np.float64)

    def ext_undist_corners(self, d, corners):
        K, D = self._ext_camera_KD(d)
        if K is None:
            return None
        b = float(self.avm_bev["balance"])
        return fisheye_math.undistort_points_fisheye(corners, K, D,
                                                      self.width, self.height, b)

    def ext_inview_ok(self, d, undist):
        pts = np.asarray(undist).reshape(-1, 2)
        m = int(self.ext_cfg["inview_margin_px"])
        inside = ((pts[:, 0] >= m) & (pts[:, 0] <= self.width - 1 - m)
                  & (pts[:, 1] >= m) & (pts[:, 1] <= self.height - 1 - m))
        return bool(inside.all()), int(inside.sum())

    @staticmethod
    def _ext_reason_text(reason):
        return {
            "no_frame": "尚未收到相机画面",
            "board_not_found": "未检测到完整棋盘：请让棋盘完整入镜、避免反光",
            "partial_board": "已识别棋盘局部，但完整 8×6 角点未识别：请靠近并移向画面中央",
            "intrinsics_missing": "该方向还没有正式内参，无法去畸变计算外参",
            "out_of_view": "角点靠近或超出画面边缘：请把棋盘往画面中央移动",
            "seam_edge_valid": "边缘棋盘已完整检出，可用于只读接缝诊断",
            "seam_raw_edge": "棋盘太靠近原图边缘：请稍向两路共同可见区域移动",
            "seam_nonfinite": "边缘去畸变坐标无效：请把棋盘稍向画面内移动",
            "seam_outside_bev": "棋盘不在当前 BEV 有效范围：请移到车身邻角地面",
            "seam_too_small": "棋盘在 BEV 中跨度过小：请靠近相机后重试",
            "moving": "棋盘仍在移动：请保持不动",
            "stable": "棋盘稳定，正在累计确认帧",
            "ready": "棋盘稳定，开始自动连拍",
            "bursting": "正在自动连拍并保存检测证据",
            "candidate": "候选结果已生成，请检查证据后保存",
            "duplicate_frame": "等待相机下一张新画面",
            "burst_timeout": "等待新画面超时",
        }.get(reason, reason or "等待检测")

    def _set_ext_live(self, d, obs):
        """保存最新检测；数组仅留在内存，避免 /api/status 变得很大。"""
        with self._lock:
            self.ext_live[d] = obs

    def ext_detect(self, d, *, update_live=True, require_inview=True):
        """检测最新画面，始终返回含原因、原图角点及去畸变角点的结构化记录。"""
        fr, ts, frame_id = self.latest_frame_info(d)
        rec = {"direction": d, "frame": fr, "ts": ts, "frame_id": frame_id,
               "expected": self.cols * self.rows, "found": False, "ok": False,
               "reason": "no_frame", "raw": None, "undist": None,
               "inview_count": 0, "phase": "detecting", "motion_rms_px": None}
        if fr is None:
            if update_live:
                self._set_ext_live(d, rec)
            return rec
        found, corners, _, _ = detect_board.scan_board(
            fr, (self.cols, self.rows), scan_width=960,
            use_sb=True, photo_retry=True)
        if not found or corners is None:
            # 实时路径为省 CPU 固定在 960 宽，棋盘较远时外圈角点会丢失。
            # 只要快速扫描失败，就用完整检测恢复；否则下一帧的快速 miss 会把
            # 正确的 48 角点状态覆盖掉，稳定计数永远无法增长。
            now = time.monotonic()
            full_found, full_corners, _ = detect_board.detect_board(
                fr, (self.cols, self.rows), max_width=0,
                try_scales=(1.0,), use_sb=True, photo_retry=True)
            if full_found and full_corners is not None:
                found, corners = True, full_corners
            if not found or corners is None:
                hint = None
                if now - self.ext_pattern_probe_ts[d] >= 1.5:
                    self.ext_pattern_probe_ts[d] = now
                    for probe_size in ((self.cols, self.rows - 1),
                                       (self.cols - 1, self.rows)):
                        if min(probe_size) < 2:
                            continue
                        probe_found, probe_corners, _, _ = detect_board.scan_board(
                            fr, probe_size, scan_width=960, use_sb=True,
                            photo_retry=False)
                        if probe_found and probe_corners is not None:
                            hint = probe_size
                            break
                    self.ext_pattern_hint[d] = hint
                hint = self.ext_pattern_hint.get(d)
                if hint:
                    rec.update({"reason": "partial_board",
                                "detected_pattern": list(hint)})
                else:
                    rec["reason"] = "board_not_found"
                if update_live:
                    self._set_ext_live(d, rec)
                return rec
        raw = np.asarray(corners, dtype=np.float64).reshape(-1, 2)
        rec.update({"found": True, "raw": raw, "corners_detected": int(len(raw))})
        und = self.ext_undist_corners(d, raw)
        if und is None:
            rec["reason"] = "intrinsics_missing"
            if update_live:
                self._set_ext_live(d, rec)
            return rec
        und = np.asarray(und, dtype=np.float64).reshape(-1, 2)
        ok, n_in = self.ext_inview_ok(d, und)
        rec.update({"undist": und, "inview_count": n_in})
        rec["inview_ok"] = bool(ok)
        if require_inview and not ok:
            rec["reason"] = "out_of_view"
            if update_live:
                self._set_ext_live(d, rec)
            return rec
        reason = "stable" if ok else "seam_edge_valid"
        rec.update({"ok": True, "reason": reason, "phase": "detected"})
        if update_live:
            self._set_ext_live(d, rec)
        return rec

    def _ext_validate_seam_observation(self, d, obs):
        """Accept edge observations only when raw and projected BEV geometry is safe."""
        if not obs.get("ok") or obs.get("raw") is None or obs.get("undist") is None:
            return obs
        raw = np.asarray(obs["raw"], dtype=np.float64).reshape(-1, 2)
        und = np.asarray(obs["undist"], dtype=np.float64).reshape(-1, 2)
        frame = obs.get("frame")
        if frame is None or raw.shape[0] != self.cols * self.rows:
            obs.update({"ok": False, "reason": "board_not_found", "phase": "failed"})
            return obs
        h, w = frame.shape[:2]
        m = SEAM_RAW_MARGIN_PX
        raw_inside = ((raw[:, 0] >= m) & (raw[:, 0] <= w - 1 - m)
                      & (raw[:, 1] >= m) & (raw[:, 1] <= h - 1 - m))
        if not bool(raw_inside.all()):
            obs.update({"ok": False, "reason": "seam_raw_edge", "phase": "failed"})
            return obs
        if not bool(np.isfinite(und).all()) or d not in self.H:
            obs.update({"ok": False, "reason": "seam_nonfinite", "phase": "failed"})
            return obs
        projected = cv2.perspectiveTransform(
            und.astype(np.float32).reshape(-1, 1, 2),
            np.asarray(self.H[d], dtype=np.float64)).reshape(-1, 2)
        canvas_w, canvas_h = map(int, self.avm_bev["canvas_size"])
        bm = SEAM_BEV_MARGIN_PX
        bev_inside = (np.isfinite(projected).all(axis=1)
                      & (projected[:, 0] >= bm) & (projected[:, 0] <= canvas_w - 1 - bm)
                      & (projected[:, 1] >= bm) & (projected[:, 1] <= canvas_h - 1 - bm))
        obs["seam_bev_inview_count"] = int(bev_inside.sum())
        if not bool(bev_inside.all()):
            obs.update({"ok": False, "reason": "seam_outside_bev", "phase": "failed"})
            return obs
        span = float(np.linalg.norm(projected.max(axis=0) - projected.min(axis=0)))
        obs["seam_bev_span_px"] = round(span, 2)
        if span < SEAM_MIN_BEV_SPAN_PX:
            obs.update({"ok": False, "reason": "seam_too_small", "phase": "failed"})
            return obs
        obs.update({"ok": True, "reason": "seam_edge_valid", "phase": "detected"})
        return obs

    def _ext_evidence_jpeg(self, obs, slot, accepted):
        """把连拍当时的原图与角点一同保存为审核缩略图。"""
        fr = obs.get("frame")
        if fr is None:
            return None
        scale = min(1.0, 360.0 / max(1, fr.shape[1]))
        img = cv2.resize(fr, (max(1, int(fr.shape[1] * scale)),
                              max(1, int(fr.shape[0] * scale))))
        color = (30, 210, 30) if accepted else (40, 60, 235)
        raw = obs.get("raw")
        if raw is not None:
            for x, y in np.asarray(raw).reshape(-1, 2):
                cv2.circle(img, (int(x * scale), int(y * scale)), 3, color, -1)
            outer = np.asarray(raw)[HOM.grid_outer_idx(self.cols, self.rows)]
            cv2.polylines(img, [np.int32(outer * scale).reshape(-1, 1, 2)],
                          True, color, 2)
        label = f"#{slot + 1} {'OK' if accepted else 'REJECT'} {self._ext_reason_text(obs.get('reason'))}"
        cv2.rectangle(img, (0, 0), (img.shape[1], 28), (0, 0, 0), -1)
        cv2.putText(img, label[:62], (8, 20), cv2.FONT_HERSHEY_SIMPLEX,
                    0.45, color, 1, cv2.LINE_AA)
        return _encode_jpeg(img, quality=72)

    def _ext_burst_frames(self, d, n):
        """只取新帧连拍；每张保留带角点的可审核缩略图和拒绝原因。"""
        views = []
        last_frame_id = -1
        with self._lock:
            self.ext_burst_evidence[d] = []
            self.ext_burst_state[d] = {"phase": "bursting", "accepted": 0,
                                       "rejected": 0, "expected": n}
            live = self.ext_live.get(d, {})
            live.update({"phase": "bursting", "reason": "bursting"})
            self.ext_live[d] = live
        for slot in range(n):
            deadline = time.monotonic() + 0.8
            obs = None
            while time.monotonic() < deadline:
                probe = self.ext_detect(d, update_live=False)
                if probe["frame_id"] > last_frame_id:
                    obs = probe
                    last_frame_id = probe["frame_id"]
                    break
                time.sleep(0.02)
            if obs is None:
                obs = {"direction": d, "frame": None, "frame_id": last_frame_id,
                       "expected": self.cols * self.rows, "found": False, "ok": False,
                       "reason": "burst_timeout", "raw": None, "undist": None,
                       "inview_count": 0, "motion_rms_px": None}
            accepted = bool(obs.get("ok"))
            if accepted:
                views.append(obs["undist"])
            item = {
                "slot": slot, "frame_id": int(obs.get("frame_id", -1)),
                "accepted": accepted, "reason": obs.get("reason"),
                "reason_text": self._ext_reason_text(obs.get("reason")),
                "corners_detected": int(obs.get("corners_detected", 0)),
                "inview_count": int(obs.get("inview_count", 0)),
                "jpeg": self._ext_evidence_jpeg(obs, slot, accepted),
            }
            with self._lock:
                self.ext_burst_evidence[d].append(item)
                state = self.ext_burst_state[d]
                state["accepted"] += int(accepted)
                state["rejected"] += int(not accepted)
            time.sleep(0.05)
        with self._lock:
            self.ext_burst_state[d]["phase"] = "complete"
        return views
        return views

    def ext_tick(self):
        """5Hz 外参状态机（移植 extrinsic_calibrator._tick）。"""
        if self.ext_task or not self.ext_running:
            return
        if self.seam_mode:
            self._ext_tick_seam()
            return
        if self.target is None or self.bursting:
            return
        obs = self.ext_detect(self.target)
        if obs["frame_id"] == self.ext_last_frame_id[self.target]:
            obs.update({"phase": "waiting", "reason": "duplicate_frame"})
            self._set_ext_live(self.target, obs)
            return
        self.ext_last_frame_id[self.target] = obs["frame_id"]
        if not obs["ok"]:
            self.streak = 0
            self.ext_stable_ref = None
            obs["phase"] = "failed"
            self._set_ext_live(self.target, obs)
            return
        corners = obs["undist"]
        if self.ext_stable_ref is None:
            self.ext_stable_ref = corners
            motion = 0.0
        else:
            aligned, motion = HOM.align_corners_to_ref(corners, self.ext_stable_ref,
                                                        self.cols, self.rows)
            if aligned is None:
                motion = float("inf")
            else:
                corners = aligned
        obs["motion_rms_px"] = round(float(motion), 3)
        stable_limit = float(self.ext_cfg["corner_outlier_rms_px"])
        if not np.isfinite(motion) or motion > stable_limit:
            self.streak = 1
            self.ext_stable_ref = corners
            obs.update({"phase": "moving", "reason": "moving"})
            self._set_ext_live(self.target, obs)
            return
        self.ext_stable_ref = corners
        self.streak += 1
        obs.update({"phase": "stable", "reason": "stable"})
        self._set_ext_live(self.target, obs)
        if self.streak >= int(self.ext_cfg["stable_frames"]):
            self.log(f"[ext {self.target}] READY({self.streak}) -> 连拍 "
                     f"{self.ext_cfg['burst_frames']} 帧")
            self.bursting = True
            self.ext_running = False
            obs.update({"phase": "ready", "reason": "ready"})
            self._set_ext_live(self.target, obs)
            self.ext_task = threading.Thread(target=self._ext_do_burst,
                                            args=(self.target,), daemon=True)
            self.ext_task.start()

    def _ext_advance_target(self):
        order = [d for d in DIRECTIONS if d not in self.ext_pending]
        self.target = order[0] if order else None
        self.streak = 0
        self.ext_stable_ref = None
        if self.target:
            self.ext_last_frame_id[self.target] = -1
            self.ext_burst_evidence[self.target] = []
            self.ext_burst_state[self.target] = {"phase": "idle", "accepted": 0,
                                                 "rejected": 0, "expected": 0}
            self.log(f"→ 外参目标: {self.target}（摆板 near="
                     f"{self.placements[self.target]['near_m']}m）")
        else:
            self.log("✅ 四路外参已锁定。可 seam（只读接缝诊断）/ verify")

    def _ext_do_burst(self, d):
        try:
            views = self._ext_burst_frames(d, int(self.ext_cfg["burst_frames"]))
            need = max(2, int(self.ext_cfg["burst_frames"]) // 4)
            if len(views) < need:
                self.log(f"[ext {d}] 连拍检出不足 {len(views)}/{need}，retry")
                self.ext_task_msg = f"❌ 连拍只有 {len(views)}/{need} 张有效；请保持棋盘完整可见后重新检测"
                self.bursting = False
                self.streak = max(0, self.streak - 3)
                return
            mean_corners, n_used, bstats = HOM.average_corners(
                views, self.cols, self.rows,
                outlier_rms_px=float(self.ext_cfg["corner_outlier_rms_px"]),
                align_max_px=float(self.ext_cfg["corner_align_max_px"]))
            if n_used < need:
                self.log(f"[ext {d}] 离群过滤后有效帧不足 {n_used}/{need}，请保持棋盘稳定后重采")
                self.ext_task_msg = f"❌ 连拍抖动过大，仅 {n_used}/{need} 张可用；请重新检测"
                return
            place = self.placements[d]
            g4, _ = HOM.ground_corners(
                d, float(place["near_m"]), float(place.get("lateral_m", 0.0)),
                str(place.get("orient", "long-lateral")),
                self.cols, self.rows, self.square)
            bev = self.avm_bev
            H, rms = HOM.best_homography(
                mean_corners, g4, self.cols, self.rows,
                float(bev["scale_px_per_meter"]),
                (int(bev["canvas_size"][0]), int(bev["canvas_size"][1])))
            if H is None:
                self.log(f"[ext {d}] H 求解失败，relock")
                self.bursting = False
                self.streak = 0
                return
            img_size = (self.width, self.height)
            canvas = (int(bev["canvas_size"][0]), int(bev["canvas_size"][1]))
            bm = HOM.board_quad_metrics(mean_corners, self.cols, self.rows)
            qc = HOM.analyze_homography(
                H, d, img_size, canvas, bm,
                svd_min_thresh=float(self.qc_cfg["h_svd_min"]),
                edge_span_min=float(self.qc_cfg["h_edge_span_min_px"]),
                center_tol=float(self.qc_cfg["h_center_tol_px"]),
                center_flip_tol=float(self.qc_cfg["h_center_flip_tol_px"]),
                board_edge_ratio_max=float(self.qc_cfg["board_edge_ratio_max"]))
            # PnP 6DoF
            K, _ = self._ext_camera_KD(d)
            new_K = np.asarray(K, dtype=np.float64).copy()
            new_K[0, 0] *= float(bev["balance"])
            new_K[1, 1] *= float(bev["balance"])
            new_K[0, 2] = self.width / 2.0
            new_K[1, 2] = self.height / 2.0
            ground_ros = HOM.placement_ground_pts_ros(
                d, self.cols, self.rows, self.square, place)
            res = HOM.solve_camera_pose(
                mean_corners, ground_ros, new_K, d, self.pose_qc)
            pose = {}
            pose_quality = {"status": "unavailable",
                            "failures": ["没有通过物理合理性检查的位姿"]}
            if res is not None:
                R, t = res
                pose = {"R": np.asarray(R).tolist(),
                        "t": [float(x) for x in np.asarray(t).ravel()]}
                pose_quality = HOM.analyze_camera_pose(pose, d, self.pose_qc)
            else:
                self.log(f"[ext {d}] ⚠ 平面 PnP 候选均不符合朝外安装，"
                         "本次仅保存直接棋盘 H，不保存 TF pose")
            outer_idx = HOM.grid_outer_idx(self.cols, self.rows)
            meas = cv2.perspectiveTransform(
                mean_corners[outer_idx].reshape(-1, 1, 2).astype(np.float32),
                H.astype(np.float32)).reshape(-1, 2)
            rms_limit = float(self.qc_cfg.get("rms_ok_px", 5.0))
            failures = list(qc.get("failures", []))
            if qc.get("status") == "bad" or rms > rms_limit or not np.isfinite(H).all() or not np.isfinite(rms):
                why = "；".join(failures) or (f"重投影 RMS={rms:.3f}px>{rms_limit:.1f}px"
                                                    if np.isfinite(rms) else "H 或 RMS 非有限")
                self.ext_task_msg = f"❌ 候选不可用：{why}。请重新检测当前方向"
                self.log(f"[ext {d}] {self.ext_task_msg}")
                return
            with self._lock:
                self.ext_candidate[d] = {"H": H, "qc": qc, "rms": float(rms),
                                         "burst": bstats, "pose": pose,
                                         "pose_qc": pose_quality,
                                         "placement": dict(place),
                                         "meas": meas.tolist()}
                self.ext_running = False
                live = self.ext_live.get(d, {})
                live.update({"phase": "candidate", "reason": "candidate"})
                self.ext_live[d] = live
            label = HOM.homography_qc_label(qc)
            self.ext_task_msg = (f"✅ 候选已生成：RMS={rms:.4f}px，{n_used}/{len(views)} 张有效；"
                                 f"请检查连拍角点后保存")
            self.log(f"[ext {d}] ✅ 候选 H: rms={rms:.4f}px "
                     f"{HOM.quality_label(rms)} | {label}")
            for wmsg in qc["warnings"]:
                self.log(f"[ext {d}] QC: {wmsg}")
        except Exception as exc:  # noqa: BLE001
            self.log(f"[ext {d}] 连拍异常: {exc}")
        finally:
            self.bursting = False
            self.streak = 0
            self.ext_stable_ref = None
            self.ext_task = None

    def ext_save_candidate(self, d, confirm=False):
        with self._lock:
            c = self.ext_candidate.get(d)
            if not c:
                return False, "没有候选外参"
            if c.get("qc", {}).get("status") == "warn" and not confirm:
                return False, "外参质量为 warn；确认后再次保存"
            self.ext_pending[d] = c
            self.ext_candidate.pop(d, None)
            ADV.save_single_pending(
                results_dir() / "extrinsics.single_pending.json",
                self.ext_pending, DIRECTIONS)
        if len(self.ext_pending) == len(DIRECTIONS):
            try:
                backup = ADV.commit_single_position(
                    results_dir(), self.ext_pending, DIRECTIONS,
                    self.avm_bev, (self.cols, self.rows), self.square)
                committed = dict(self.ext_pending)
                self.ext_pending.clear()
                pending_path = results_dir() / "extrinsics.single_pending.json"
                if pending_path.exists():
                    pending_path.unlink()
                with self._lock:
                    self.H = {x: np.asarray(committed[x]["H"], np.float64)
                              for x in DIRECTIONS}
                    self.qc = {x: committed[x]["qc"] for x in DIRECTIONS}
                    self.rms_errors = {x: float(committed[x]["rms"])
                                       for x in DIRECTIONS}
                    self.burst_stats = {x: committed[x]["burst"] for x in DIRECTIONS}
                    self.poses = {x: committed[x]["pose"] for x in DIRECTIONS
                                  if committed[x].get("pose")}
                    self.measured_bev = {x: committed[x]["meas"] for x in DIRECTIONS}
                    self.ext_saved_catalog = {
                        x: {"state": "active", "rms": float(committed[x]["rms"]),
                            "quality": committed[x]["qc"].get("status")}
                        for x in DIRECTIONS}
                stale = results_dir() / "extrinsics.stale.json"
                if stale.exists():
                    stale.unlink()
                self.ext_results_stale = False
                self.bev_cache = {}
                self.bev_loaded_ok = False
                self.bev_seam = {"state": "pending",
                                 "message": "新外参已提交；旧接缝已失效，请重新优化",
                                 "updated_at": None,
                                 "gains": {x: 1.0 for x in DIRECTIONS}}
            except Exception as exc:  # noqa: BLE001
                self.ext_task_msg = f"❌ 四路原子提交失败：{exc}；正式外参未改变"
                self.log(self.ext_task_msg)
                return False, self.ext_task_msg
            self.target = None
            self.ext_running = False
            self.ext_task_msg = (f"✅ 四路单点外参已原子提交（备份 {backup.name}）。"
                                 "旧接缝已失效；下一步进入 BEV 重新优化接缝")
            self.log(self.ext_task_msg)
            return True, self.ext_task_msg
        remaining = [x for x in DIRECTIONS if x not in self.ext_pending]
        self.target = remaining[0]
        self.ext_running = False
        self.streak = 0
        self.ext_task_msg = (f"✅ {d} 已暂存（{len(self.ext_pending)}/4），"
                             f"正式外参未改变；下一方向 {self.target}")
        return True, self.ext_task_msg

    def _ext_tick_seam(self):
        if self.seam_pair_i >= len(SEAM_PAIRS):
            self.seam_mode = False
            self.seam_complete = True
            self.log("接缝诊断全部完成，正式 H 未修改")
            return
        ref_d, slave_d = SEAM_PAIRS[self.seam_pair_i]
        now = time.time()
        if now - self.seam_last_refine < 5.0 and self.seam_streak == 0:
            return
        # 每路独立 detect；命中才更新 fresh（失败保留旧值，容错单帧漏检，
        # 与 extrinsic_calibrator._tick_seam 一致——否则 seam_fresh_max_age 形同虚设）
        _, ref_ts = self.latest_frame(ref_d)
        _, slave_ts = self.latest_frame(slave_d)
        if ref_ts and slave_ts and abs(ref_ts - slave_ts) > 0.2:
            self.seam_streak = 0
            return
        rc_obs = self._ext_validate_seam_observation(
            ref_d, self.ext_detect(ref_d, update_live=False, require_inview=False))
        sc_obs = self._ext_validate_seam_observation(
            slave_d, self.ext_detect(slave_d, update_live=False, require_inview=False))
        self._set_ext_live(ref_d, rc_obs)
        self._set_ext_live(slave_d, sc_obs)
        rc = rc_obs["undist"] if rc_obs.get("ok") else None
        sc = sc_obs["undist"] if sc_obs.get("ok") else None
        should_refine = False
        with self._lock:
            if rc is not None:
                self.fresh_corners[ref_d] = rc
                self.fresh_ts[ref_d] = now
            if sc is not None:
                self.fresh_corners[slave_d] = sc
                self.fresh_ts[slave_d] = now
            max_age = float(self.ext_cfg["seam_fresh_max_age"])
            ref_fresh = (now - self.fresh_ts.get(ref_d, 0)) <= max_age \
                and ref_d in self.fresh_corners
            slave_fresh = (now - self.fresh_ts.get(slave_d, 0)) <= max_age \
                and slave_d in self.fresh_corners
            if ref_fresh and slave_fresh:
                self.seam_streak += 1
            else:
                self.seam_streak = 0
            if self.seam_streak >= 3:
                self.seam_streak = 0
                self.seam_last_refine = now
                should_refine = True
        if should_refine:
            self.log(f"[seam {ref_d}+{slave_d}] 同步达标，精修...")
            self.ext_task = threading.Thread(
                target=self._ext_do_seam, args=(ref_d, slave_d), daemon=True)
            self.ext_task.start()

    def _ext_do_seam(self, ref_d, slave_d):
        advance_pair = True
        try:
            ref = self.fresh_corners.get(ref_d)
            slave = self.fresh_corners.get(slave_d)
            stats = HOM.evaluate_seam_alignment(
                ref, slave, self.H[ref_d], self.H[slave_d],
                self.cols, self.rows)
            if stats.get("error"):
                self.log(f"[seam {ref_d}+{slave_d}] 诊断失败: {stats['error']}")
                return
            with self._lock:
                meta = {"ref": ref_d, "slave": slave_d, "ts": time.time(),
                        "read_only": True}
                meta.update(stats)
                self.seam_diagnostics.append(meta)
            if not stats.get("same_target", True):
                advance_pair = False
            self.log(f"[seam {ref_d}+{slave_d}] 只读诊断："
                     f"RMS={stats['rms_px']:.2f}px，"
                     f"max={stats['max_error_px']:.2f}px，"
                     f"中心差={stats.get('center_delta_px', 0):.2f}px；{stats['reason']}；"
                     "正式 H 未修改")
        except Exception as exc:  # noqa: BLE001
            self.log(f"[seam {ref_d}+{slave_d}] 异常: {exc}")
        finally:
            self.ext_task = None
            with self._lock:
                if not advance_pair:
                    self.seam_last_refine = time.time()
                    self.log(f"→ 保留当前 {ref_d}+{slave_d}：移走多余棋盘后再诊断")
                elif self.seam_pair_i + 1 < len(SEAM_PAIRS):
                    self.seam_pair_i += 1
                    pair = SEAM_PAIRS[self.seam_pair_i]
                    self.log(f"→ 下一对: {pair[0]}+{pair[1]}")
                else:
                    self.seam_mode = False
                    self.seam_complete = True
                    self.log("接缝诊断全部完成，正式 H 未修改")
                self.seam_streak = 0

    def ext_start_seam(self):
        with self._lock:
            if self.ext_pending:
                return False, "本轮单点外参尚未四路齐全，不能用旧外参做接缝诊断"
            if len(self.H) < 4:
                return False, "接缝诊断需四路 H 齐备"
            if self.ext_task:
                return False, "标定计算中，稍后重试"
            self.mode = "extrinsics"
            self.seam_mode = True
            self.seam_complete = False
            self.ext_running = True
            self.seam_pair_i = 0
            self.seam_streak = 0
            self.seam_last_refine = 0.0
            self.fresh_corners.clear()
            self.fresh_ts.clear()
        pair = SEAM_PAIRS[0]
        self.log(f"进入只读接缝诊断：把棋盘放到 {pair[0]}+{pair[1]} 重叠区；正式 H 不会修改")
        return True, f"接缝诊断：{pair[0]}+{pair[1]}"

    def ext_verify(self, d, near, lateral):
        if d not in self.H:
            return False, f"[{d}] 尚未标定 H"
        if self.ext_task:
            return False, "标定计算中，稍后重试"
        self.ext_task = threading.Thread(
            target=self._ext_verify, args=(d, near, lateral), daemon=True)
        self.ext_task.start()
        return True, f"verify {d} near={near} lateral={lateral} 计算中..."

    def _ext_verify(self, d, near, lateral):
        try:
            views = self._ext_burst_frames(d, int(self.ext_cfg["burst_frames"]))
            if not views:
                self.log(f"[verify {d}] 未检出棋盘")
                return
            mean, _, _ = HOM.average_corners(views, self.cols, self.rows)
            orient = str(self.placements[d].get("orient", "long-lateral"))
            g4, _ = HOM.ground_corners(d, near, lateral, orient,
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
            err = np.linalg.norm(meas - expected, axis=1)
            rec = {
                "direction": d, "near_m": float(near),
                "lateral_m": float(lateral),
                "error_mean_px": float(np.mean(err)),
                "error_max_px": float(np.max(err)),
                "error_mean_m": float(np.mean(err) / scale),
                "n_views": len(views), "ts": time.time(),
            }
            with self._lock:
                self.verifications.append(rec)
            self._ext_save_merged(d, self.poses.get(d, {}))
            ok = rec["error_mean_px"] < 5.0
            self.log(f"[verify {d}] near={near}m lat={lateral}m -> "
                     f"均值 {rec['error_mean_px']:.2f}px "
                     f"({rec['error_mean_m']*100:.1f}cm) "
                     f"{'✅' if ok else '❌'}")
        except Exception as exc:  # noqa: BLE001
            self.log(f"[verify {d}] 异常: {exc}")
        finally:
            self.ext_task = None

    def _ext_save_merged(self, d, pose, seam_meta=None):
        bev = dict(self.avm_bev)
        bev["square_size_m"] = self.square
        place = self.placements[d]
        HOM.save_extrinsics(
            results_dir() / "extrinsics.json", d, self.H[d],
            rms=self.rms_errors[d], qc=self.qc[d],
            burst_stats=self.burst_stats.get(d, {}),
            pose=pose or {},
            placements_used={
                "near_m": float(place["near_m"]),
                "lateral_m": float(place.get("lateral_m", 0)),
                "orient": str(place.get("orient", "long-lateral")),
                "board_w_m": (self.cols + 1) * self.square,
                "board_h_m": (self.rows + 1) * self.square,
            },
            bev_cfg=bev, pattern=(self.cols, self.rows),
            seam_meta=seam_meta)
        # save_extrinsics 不覆盖 seam_refined/verifications/board_measured_bev
        p = results_dir() / "extrinsics.json"
        with p.open("r") as f:
            data = json.load(f)
        with self._lock:
            data["seam_refined"] = self.seam_history
            data["verifications"] = self.verifications
            data.setdefault("board_measured_bev", {})[d] = \
                self.measured_bev.get(d)
        tmp = p.with_suffix(p.suffix + ".tmp")
        with tmp.open("w") as f:
            json.dump(data, f, indent=2, ensure_ascii=False)
            f.flush(); os.fsync(f.fileno())
        os.replace(tmp, p)

    def ext_set_placement(self, d, near, lateral):
        """设置当前这一次外参的棋盘摆位，不改写 YAML 默认配置。

        H 与地面摆位一一对应，因此参数变更会作废该方向未保存候选和连拍证据，
        用户必须重新检测，避免“显示的距离”和实际求 H 的距离不一致。
        """
        if d not in DIRECTIONS:
            return False, "方向无效"
        if d != self.target:
            return False, "只能设置当前需要摆板的方向"
        if self.ext_running or self.bursting or self.ext_task:
            return False, "请先暂停检测，不能在检测或计算中修改摆位"
        if d in self.H:
            return False, "该方向已保存；如需重标定，请先重新检测该方向"
        if not np.isfinite(near) or not (0.05 <= near <= 10.0):
            return False, "近边距离需在 0.05–10m 之间"
        if not np.isfinite(lateral) or abs(lateral) > 5.0:
            return False, "横向偏移需在 ±5m 之间"
        with self._lock:
            place = dict(self.placements[d])
            place["near_m"] = float(near)
            place["lateral_m"] = float(lateral)
            self.placements[d] = place
            self.cfg["placements"][d] = place
            self.ext_candidate.pop(d, None)
            self.ext_burst_evidence[d] = []
            self.ext_burst_state[d] = {"phase": "idle", "accepted": 0,
                                       "rejected": 0, "expected": 0}
            self.ext_stable_ref = None
            self.streak = 0
        self.ext_task_msg = (f"[{d}] 本轮摆位已设为：近边 {near:.3f}m，"
                             f"横向偏移 {lateral:.3f}m；现在可开始自动检测")
        self.log(self.ext_task_msg)
        return True, self.ext_task_msg

    def ext_select_target(self, d):
        """明确选择要标定的相机方向，而不是只切换观看画面。"""
        if d not in DIRECTIONS:
            return False, "方向无效"
        if self.ext_task or self.bursting:
            return False, "正在连拍或计算，完成后才能切换标定方向"
        with self._lock:
            self.mode = "extrinsics"
            self.ext_running = False
            self.target = d
            self.streak = 0
            self.ext_stable_ref = None
            self.ext_last_frame_id[d] = -1
        state = ("本轮已暂存；可重新检测替换候选" if d in self.ext_pending else
                 "已保存；可点重新检测以重标" if d in self.H else
                 "请确认摆位后开始检测")
        self.ext_task_msg = f"当前标定方向已切换为 {d}；{state}"
        self.log(self.ext_task_msg)
        return True, self.ext_task_msg

    def ext_cmd(self, cmd):
        """web 按钮映射到 skip/relock/seam/verify。返回 (ok, msg)。"""
        if cmd == "skip":
            self.streak = 0
            if self.seam_mode:
                return False, "接缝诊断请使用专用的跳过当前对操作"
            pending = [d for d in DIRECTIONS if d not in self.ext_pending]
            if len(pending) <= 1:
                return False, "没有其他未保存方向可切换"
            i = DIRECTIONS.index(self.target) if self.target in DIRECTIONS else -1
            next_d = next(d for off in range(1, len(DIRECTIONS) + 1)
                          if (d := DIRECTIONS[(i + off) % len(DIRECTIONS)]) in pending
                          and d != self.target)
            self.ext_select_target(next_d)
            return True, f"已切换到 {next_d}；尚未保存的方向可稍后再选回"
        if cmd == "start":
            if self.target is None:
                self._ext_advance_target()
            self.ext_stable_ref = None
            self.streak = 0
            self.ext_running = True
            self.ext_task_msg = f"请把棋盘贴地放在 {self.target or '—'} 方向标注距离处，保持不动"
            return True, f"开始自动检测 {self.target or '—'}"
        if cmd == "stop":
            self.ext_running = False
            self.streak = 0
            return True, "已暂停外参采集"
        if cmd == "relock":
            if self.target is None:
                return False, "当前没有外参目标，请先开始检测"
            self.ext_pending.pop(self.target, None)
            ADV.save_single_pending(
                results_dir() / "extrinsics.single_pending.json",
                self.ext_pending, DIRECTIONS)
            self.streak = 0
            self.ext_candidate.pop(self.target, None)
            self.ext_stable_ref = None
            self.ext_last_frame_id[self.target] = -1
            self.ext_burst_evidence[self.target] = []
            self.ext_burst_state[self.target] = {"phase": "idle", "accepted": 0,
                                                 "rejected": 0, "expected": 0}
            self.ext_running = True
            self.ext_task_msg = f"[{self.target}] 已清除本轮候选与连拍证据，重新检测"
            return True, self.ext_task_msg
        if cmd == "seam":
            return self.ext_start_seam()
        return False, f"未知命令 {cmd}"

    def seam_skip(self):
        """Skip only the active seam pair; never change the extrinsic target."""
        with self._lock:
            if not self.seam_mode:
                return False, "接缝诊断尚未开始"
            if self.seam_pair_i + 1 < len(SEAM_PAIRS):
                self.seam_pair_i += 1
                self.seam_streak = 0
                self.seam_last_refine = 0.0
                pair = SEAM_PAIRS[self.seam_pair_i]
                return True, f"跳到 {pair[0]}+{pair[1]}"
            self.seam_mode = False
            self.seam_complete = True
            return True, "接缝诊断已结束，正式 H 未修改"

    # ---------- 多位置外参标定 ----------
    def _intrinsics_fingerprint(self):
        with self._lock:
            if any(d not in self.intr_results for d in DIRECTIONS):
                return ""
            payload = {
                d: {k: self.intr_results[d].get(k) for k in ("K", "D", "image_size")}
                for d in DIRECTIONS
            }
        raw = json.dumps(payload, sort_keys=True, separators=(",", ":"),
                         ensure_ascii=True).encode("ascii")
        return hashlib.sha256(raw).hexdigest()

    def _multi_stale_reason(self):
        current = self._intrinsics_fingerprint()
        expected = str(self.multi_session.get("intrinsics_fingerprint") or "")
        has_data = any(self.multi_session.get("observations", {}).get(d)
                       for d in DIRECTIONS) or bool(self.multi_session.get("solutions"))
        if not current:
            return "四路正式内参未全部就绪"
        if not expected:
            return "旧多点会话没有内参指纹" if has_data else ""
        if expected != current:
            return "多点会话使用的内参与当前正式内参不一致"
        return ""

    def _mark_multi_session_stale(self, reason):
        with self._lock:
            if not any(self.multi_session.get("observations", {}).get(d)
                       for d in DIRECTIONS) and not self.multi_session.get("solutions"):
                self.multi_session["intrinsics_fingerprint"] = self._intrinsics_fingerprint()
                ADV.save_session(self.multi_session_dir, self.multi_session)
                return
            self.multi_session["state"] = "stale"
            self.multi_session["stale_reason"] = str(reason)
            self.multi_session["last_error"] = str(reason)
            ADV.save_session(self.multi_session_dir, self.multi_session)

    def sync_multi_session_intrinsics(self):
        """Adopt current intrinsics only for an empty legacy session."""
        with self._lock:
            has_data = any(self.multi_session.get("observations", {}).get(d)
                           for d in DIRECTIONS) or bool(self.multi_session.get("solutions"))
            if not has_data and not self.multi_session.get("intrinsics_fingerprint"):
                fingerprint = self._intrinsics_fingerprint()
                if fingerprint:
                    self.multi_session["intrinsics_fingerprint"] = fingerprint
                    ADV.save_session(self.multi_session_dir, self.multi_session)

    def multi_snapshot(self):
        """Return a compact session view; corner arrays remain on disk only."""
        with self._lock:
            session = self.multi_session
            observations = {
                d: [{k: item.get(k) for k in (
                    "id", "position_id", "placement", "captured_at",
                    "frame_count", "corner_jitter_px", "evidence")}
                    for item in session.get("observations", {}).get(d, [])]
                for d in DIRECTIONS
            }
            solutions = {
                d: {k: value for k, value in solution.items()
                    if k not in ("H", "pose")}
                for d, solution in (session.get("solutions") or {}).items()
            }
            stale_reason = self._multi_stale_reason()
            return {
                "id": session.get("id"), "state": session.get("state"),
                "created_at": session.get("created_at"),
                "updated_at": session.get("updated_at"),
                "layout": session.get("layout", {}),
                "observations": observations, "solutions": solutions,
                "task_running": self.multi_task is not None,
                "task_message": self.multi_task_msg,
                "last_error": session.get("last_error", ""),
                "intrinsics_fingerprint": session.get("intrinsics_fingerprint", ""),
                "stale": bool(stale_reason),
                "stale_reason": stale_reason,
            }

    def multi_create(self):
        if self.multi_task is not None:
            return False, "采集或解算正在进行"
        current = self.multi_session_dir / "session.json"
        if current.is_file():
            archive = self.multi_session_dir.parent / "archive" / str(
                self.multi_session.get("id", datetime.now().strftime("%Y%m%d-%H%M%S")))
            archive.parent.mkdir(parents=True, exist_ok=True)
            if archive.exists():
                archive = archive.with_name(archive.name + "-" + datetime.now().strftime("%H%M%S"))
            shutil.move(str(self.multi_session_dir), str(archive))
        self.multi_session_dir.mkdir(parents=True, exist_ok=True)
        with self._lock:
            self.multi_session = ADV.new_session(
                DIRECTIONS, self._intrinsics_fingerprint())
            self.multi_task_msg = "新会话已创建；按预设位置逐项采集"
        ADV.save_session(self.multi_session_dir, self.multi_session)
        return True, self.multi_task_msg

    def _multi_capture(self, d, position_id, placement):
        try:
            raw_views, undist_views, evidence = [], [], []
            last_id = -1
            evidence_dir = self.multi_session_dir / "evidence"
            evidence_dir.mkdir(parents=True, exist_ok=True)
            for slot in range(8):
                deadline = time.monotonic() + 1.2
                obs = None
                while time.monotonic() < deadline:
                    probe = self.ext_detect(d, update_live=False)
                    if probe.get("frame_id", -1) > last_id:
                        obs = probe
                        last_id = int(probe.get("frame_id", -1))
                        break
                    time.sleep(0.025)
                if not obs or not obs.get("ok"):
                    raise RuntimeError(
                        f"第 {slot + 1}/8 帧没有完整稳定棋盘：" +
                        self._ext_reason_text((obs or {}).get("reason", "no_frame")))
                raw_views.append(np.asarray(obs["raw"], np.float64))
                undist_views.append(np.asarray(obs["undist"], np.float64))
                jpeg = self._ext_evidence_jpeg(obs, slot, True)
                name = f"{d}-{position_id}-{slot + 1}.jpg"
                if jpeg:
                    (evidence_dir / name).write_bytes(jpeg)
                    evidence.append(name)
                self.multi_task_msg = f"[{d}] {position_id} 正在采集 {slot + 1}/8"
                time.sleep(0.06)
            mean_undist, used, stats = HOM.average_corners(
                undist_views, self.cols, self.rows,
                outlier_rms_px=float(self.ext_cfg["corner_outlier_rms_px"]),
                align_max_px=float(self.ext_cfg["corner_align_max_px"]))
            mean_raw, _, _ = HOM.average_corners(
                raw_views, self.cols, self.rows,
                outlier_rms_px=float(self.ext_cfg["corner_outlier_rms_px"]),
                align_max_px=float(self.ext_cfg["corner_align_max_px"]))
            if used < 6:
                raise RuntimeError(f"离群过滤后只有 {used}/8 帧；请固定棋盘后重采")
            record = {
                "id": f"{d}-{int(time.time() * 1000)}", "position_id": position_id,
                "placement": placement, "captured_at": datetime.now().isoformat(timespec="seconds"),
                "frame_count": used, "corner_jitter_px": stats.get("corner_jitter_px"),
                "raw_corners": mean_raw.tolist(),
                "undist_corners": mean_undist.tolist(), "evidence": evidence,
            }
            with self._lock:
                items = self.multi_session["observations"][d]
                items[:] = [item for item in items if item.get("position_id") != position_id]
                items.append(record)
                self.multi_session["solutions"].pop(d, None)
                self.multi_session["state"] = "capturing"
                self.multi_session["last_error"] = ""
                self.multi_task_msg = f"✅ [{d}] {position_id} 已保存 {used}/8 帧"
                ADV.save_session(self.multi_session_dir, self.multi_session)
        except Exception as exc:  # noqa: BLE001
            with self._lock:
                self.multi_session["last_error"] = str(exc)
                self.multi_task_msg = f"❌ [{d}] 采集失败：{exc}"
                ADV.save_session(self.multi_session_dir, self.multi_session)
        finally:
            with self._lock:
                self.multi_task = None

    def multi_capture_start(self, d, position_id, near, lateral, orient="long-lateral"):
        if d not in DIRECTIONS:
            return False, "方向无效"
        stale_reason = self._multi_stale_reason()
        if stale_reason:
            return False, stale_reason + "；请新建多点会话"
        if self.multi_task is not None or self.ext_task is not None or self.bursting:
            return False, "已有采集或解算任务正在进行"
        expected = {item["id"]: item for item in self.multi_session["layout"][d]}
        if position_id not in expected:
            return False, "位置编号不属于当前标定计划"
        preset = expected[position_id]
        if abs(float(near) - float(preset["near_m"])) > 0.03 or abs(
                float(lateral) - float(preset["lateral_m"])) > 0.03:
            return False, "实测摆位与预设值偏差超过 3cm；请重新测量"
        placement = {"near_m": float(near), "lateral_m": float(lateral),
                     "orient": str(orient)}
        self.mode = "multi_extrinsics"
        self.ext_running = False
        self.multi_task_msg = f"[{d}] {position_id} 准备采集 8 帧"
        self.multi_task = threading.Thread(
            target=self._multi_capture, args=(d, position_id, placement), daemon=True)
        self.multi_task.start()
        return True, self.multi_task_msg

    def multi_remove(self, d, observation_id):
        if d not in DIRECTIONS or self.multi_task is not None:
            return False, "方向无效或任务正在进行"
        with self._lock:
            items = self.multi_session["observations"][d]
            remaining = [item for item in items if item.get("id") != observation_id]
            if len(remaining) == len(items):
                return False, "未找到该观测"
            self.multi_session["observations"][d] = remaining
            self.multi_session["solutions"].pop(d, None)
            self.multi_session["state"] = "capturing"
            ADV.save_session(self.multi_session_dir, self.multi_session)
        return True, "观测已移除；正式标定未改变"

    def multi_solve(self):
        if self.multi_task is not None:
            return False, "任务正在进行"
        stale_reason = self._multi_stale_reason()
        if stale_reason:
            return False, stale_reason + "；请新建多点会话"
        try:
            solutions = {}
            for d in DIRECTIONS:
                observations = self.multi_session["observations"][d]
                required = {item["id"] for item in self.multi_session["layout"][d]}
                captured = {item.get("position_id") for item in observations}
                missing = sorted(required - captured)
                if missing:
                    raise ValueError(f"{d} 还缺 {len(missing)} 个预设位置")
                K, D = self._ext_camera_KD(d)
                if K is None:
                    raise ValueError(f"{d} 缺正式内参")
                new_k = fisheye_math.undistort_new_K(
                    K, self.width, self.height, float(self.avm_bev["balance"]))
                solution = ADV.solve_multi_position(
                    HOM, observations, d, self.cols, self.rows, self.square,
                    float(self.avm_bev["scale_px_per_meter"]),
                    tuple(self.avm_bev["canvas_size"]), new_k)
                H = np.asarray(solution["H"], np.float64)
                board_metrics = HOM.board_quad_metrics(
                    np.asarray(observations[0]["undist_corners"]), self.cols, self.rows)
                solution["qc"] = HOM.analyze_homography(
                    H, d, (self.width, self.height), tuple(self.avm_bev["canvas_size"]),
                    board_metrics,
                    svd_min_thresh=float(self.qc_cfg["h_svd_min"]),
                    edge_span_min=float(self.qc_cfg["h_edge_span_min_px"]),
                    center_tol=float(self.qc_cfg["h_center_tol_px"]),
                    center_flip_tol=float(self.qc_cfg["h_center_flip_tol_px"]),
                    board_edge_ratio_max=float(self.qc_cfg["board_edge_ratio_max"]))
                outer = np.asarray(observations[0]["undist_corners"])[
                    HOM.grid_outer_idx(self.cols, self.rows)]
                solution["board_measured_bev"] = cv2.perspectiveTransform(
                    outer.reshape(-1, 1, 2).astype(np.float64), H).reshape(-1, 2).tolist()
                solution["passed"] = bool(solution["passed"] and
                                           solution["qc"].get("status") != "bad")
                solutions[d] = solution
            with self._lock:
                self.multi_session["solutions"] = solutions
                self.multi_session["state"] = (
                    "ready" if all(x["passed"] for x in solutions.values()) else "failed")
                self.multi_session["last_error"] = ""
                self.multi_task_msg = ("✅ 四路全局解算通过，可提交" if
                                       self.multi_session["state"] == "ready" else
                                       "❌ 有相机未通过质量门槛，请检查各位置误差")
                ADV.save_session(self.multi_session_dir, self.multi_session)
            return self.multi_session["state"] == "ready", self.multi_task_msg
        except Exception as exc:  # noqa: BLE001
            with self._lock:
                self.multi_session["state"] = "failed"
                self.multi_session["last_error"] = str(exc)
                self.multi_task_msg = f"❌ 全局解算失败：{exc}"
                ADV.save_session(self.multi_session_dir, self.multi_session)
            return False, self.multi_task_msg

    def multi_commit(self):
        if self.multi_task is not None:
            return False, "任务正在进行"
        stale_reason = self._multi_stale_reason()
        if stale_reason:
            return False, stale_reason + "；禁止提交"
        try:
            backup = ADV.commit_multi_session(results_dir(), self.multi_session, DIRECTIONS)
            self.multi_session["state"] = "committed"
            self.multi_session["commit_backup"] = str(backup)
            ADV.save_session(self.multi_session_dir, self.multi_session)
            load_existing_results(self)
            ok = self.bev_load()
            self.multi_task_msg = "✅ 四路多点外参已原子提交；旧接缝因标定指纹变化自动失效"
            return ok, self.multi_task_msg if ok else self.bev_error
        except Exception as exc:  # noqa: BLE001
            return False, f"提交失败：{exc}"

    def multi_rollback(self):
        backup = self.multi_session.get("commit_backup")
        if not backup or not Path(backup).is_file():
            return False, "当前会话没有可回滚的提交备份"
        target = results_dir() / "extrinsics.json"
        tmp = target.with_suffix(".json.rollback.tmp")
        shutil.copy2(backup, tmp)
        os.replace(tmp, target)
        self.multi_session["state"] = "rolled_back"
        ADV.save_session(self.multi_session_dir, self.multi_session)
        load_existing_results(self)
        ok = self.bev_load()
        return ok, "已恢复多点提交前的正式外参" if ok else self.bev_error

    # ---------- BEV 渲染 ----------
    @staticmethod
    def _bev_preview_settings_path() -> Path:
        """会话预览配置，与正式标定结果分离。"""
        return results_dir() / BEV_PREVIEW_SETTINGS

    def load_bev_preview_settings(self):
        """恢复车体预览设置；坏文件不影响标定或 BEV 启动。"""
        path = self._bev_preview_settings_path()
        try:
            with path.open(encoding="utf-8") as f:
                saved = json.load(f)
            body = saved.get("body") or {}
            if not body.get("enabled", True):
                self.bev_body_size_m = None
                return
            width = float(body.get("width_m", DEFAULT_BODY_SIZE_M[0]))
            length = float(body.get("length_m", DEFAULT_BODY_SIZE_M[1]))
            if 0.10 <= width <= 3.0 and 0.10 <= length <= 5.0:
                self.bev_body_size_m = (width, length)
        except (OSError, ValueError, TypeError, json.JSONDecodeError):
            pass

    def _save_bev_preview_settings(self):
        """原子保存预览偏好，绝不触碰 extrinsics.json。"""
        size = self.bev_body_size_m or DEFAULT_BODY_SIZE_M
        data = {"version": 1, "body": {"enabled": self.bev_body_size_m is not None,
                                           "width_m": float(size[0]),
                                           "length_m": float(size[1])}}
        path = self._bev_preview_settings_path()
        tmp = path.with_suffix(path.suffix + ".tmp")
        try:
            path.parent.mkdir(parents=True, exist_ok=True)
            with tmp.open("w", encoding="utf-8") as f:
                json.dump(data, f, ensure_ascii=False, indent=2)
            os.replace(tmp, path)
        except OSError as exc:
            self.log(f"[bev] 保存预览设置失败：{exc}")

    @staticmethod
    def _bev_valid_homography_branch(H_dst_from_src, reference_dst, xx, yy):
        """保留与已保存棋盘同侧的单应分支；对 H 乘 -1 不敏感。"""
        inv_h = np.linalg.inv(np.asarray(H_dst_from_src, dtype=np.float64))
        rx, ry = float(reference_dst[0]), float(reference_dst[1])
        ref_den = float(inv_h[2, 0] * rx + inv_h[2, 1] * ry + inv_h[2, 2])
        if not np.isfinite(ref_den) or abs(ref_den) < 1e-10:
            raise ValueError("棋盘参考点落在投影奇异线，不能建立安全 BEV 映射")
        den = (inv_h[2, 0] * xx + inv_h[2, 1] * yy + inv_h[2, 2])
        epsilon = max(abs(ref_den) * 1e-6, 1e-10)
        return np.isfinite(den) & ((den * ref_den) > epsilon)

    @staticmethod
    def _bev_body_rect(shape, scale, size_m):
        if size_m is None:
            return None
        h, w = shape[:2]
        width_px = max(1, int(round(scale * float(size_m[0]))))
        length_px = max(1, int(round(scale * float(size_m[1]))))
        cx, cy = w // 2, h // 2
        x0 = max(0, cx - width_px // 2)
        y0 = max(0, cy - length_px // 2)
        return x0, y0, min(w - 1, x0 + width_px - 1), min(h - 1, y0 + length_px - 1)

    @classmethod
    def _bev_body_mask(cls, shape, scale, size_m):
        mask = np.zeros(shape[:2], dtype=bool)
        rect = cls._bev_body_rect(shape, scale, size_m)
        if rect is None:
            return mask
        x0, y0, x1, y1 = rect
        mask[y0:y1+1, x0:x1+1] = True
        return mask

    def _bev_update_coverage(self, cache):
        valid_any = np.any(np.stack([cache["supports"][d] for d in DIRECTIONS]), axis=0)
        body = self._bev_body_mask((cache["canvas"][1], cache["canvas"][0]),
                                   cache["scale"], self.bev_body_size_m)
        outside = ~body
        cache["valid_any"] = valid_any
        cache["unobserved"] = ~valid_any
        cache["coverage"] = {
            d: {"valid_px": int(cache["supports"][d].sum()),
                "reverse_rejected_px": int(cache["reverse_rejected"][d].sum()),
                "direction_rejected_px": int(cache["direction_rejected"][d].sum())}
            for d in DIRECTIONS
        }
        cache["coverage"]["outside_body_uncovered_pct"] = round(
            100.0 * float((~valid_any & outside).sum()) / max(int(outside.sum()), 1), 2)

    def bev_set_view(self, view_m, grid_m=0.0, mode="blend", transition_m=None,
                     show_seams=None, body_w_m=None, body_l_m=None, body_enabled=None):
        """设置会话级 BEV 视图；不改任何正式标定文件。"""
        if not np.isfinite(view_m) or not (2.0 <= float(view_m) <= 10.0):
            return False, "地面范围需在 2–10 米之间"
        if float(grid_m) not in (0.0, 0.5, 1.0):
            return False, "网格仅支持关闭、0.5m 或 1m"
        if mode not in ("blend", "surround", "front", "back", "left", "right", "coverage", "ownership"):
            return False, "查看模式无效"
        if transition_m is not None and (not np.isfinite(transition_m)
                                         or not 0.0 <= float(transition_m) <= 0.10):
            return False, "接缝过渡宽度需在 0–0.10 米之间"
        if body_enabled and (body_w_m is None or body_l_m is None):
            return False, "车体遮罩需同时填写长和宽"
        if body_enabled and (not 0.10 <= float(body_w_m) <= 3.0
                                     or not 0.10 <= float(body_l_m) <= 5.0):
            return False, "车体尺寸超出允许范围"
        rebuild = abs(self.bev_view_m - float(view_m)) > 1e-6
        self.bev_view_m, self.bev_grid_m, self.bev_display_mode = float(view_m), float(grid_m), mode
        if transition_m is not None:
            self.bev_transition_m = float(transition_m)
        if show_seams is not None:
            self.bev_show_seams = bool(show_seams)
        if body_enabled:
            self.bev_body_size_m = (float(body_w_m), float(body_l_m))
        elif body_enabled is False:
            self.bev_body_size_m = None
        self._save_bev_preview_settings()
        self.bev_preview_cache = None
        if not rebuild and self.bev_loaded_ok:
            # Do not mutate a cache which an MJPEG request may already hold.
            # Build all CPU/GPU assets first, then exchange one dictionary.
            with self._lock:
                previous = self.bev_cache
                previous_version = self.bev_cache_version
            replacement = dict(previous)
            self._bev_update_coverage(replacement)
            self._bev_build_weights(replacement)
            with self._lock:
                if self.bev_cache is previous and self.bev_cache_version == previous_version:
                    self.bev_cache = replacement
                    self.bev_cache_version += 1
            return True, f"BEV 视图已更新：{mode}"
        ok = self.bev_load()
        return ok, (f"BEV 地面范围已设为 {view_m:.1f}×{view_m:.1f}m"
                    if ok else self.bev_error)

    @staticmethod
    def _bev_angle_weight(direction, gx, gy):
        """限制每路相机的地面扇区，阻止单应矩阵跨视线外推到对侧。"""
        return ADV.angle_weight(direction, gx, gy, CAM_AXIS)

    @staticmethod
    def _bev_owner_and_weights(supports, scores, transition_px, allowed=None,
                               hard_mask=None):
        """先硬选一路，再只在很窄的边界带过渡，避免宽区域平均造成双影。"""
        return ADV.owner_and_weights(
            supports, scores, DIRECTIONS, transition_px, allowed, hard_mask)

    def _bev_upload_gpu(self, cache):
        if cache["backend"] != "cuda":
            cache["gpu"] = {}
            return
        gpu = {}
        for d in DIRECTIONS:
            gm1, gm2, gw1, gw3, ggain3 = (cv2.cuda_GpuMat() for _ in range(5))
            gm1.upload(cache["mapx"][d]); gm2.upload(cache["mapy"][d])
            gw1.upload(cache["weights"][d])
            gw3.upload(np.dstack([cache["weights"][d]] * 3))
            gain = float(cache["gains"].get(d, 1.0))
            ggain3.upload(np.full((cache["canvas"][1], cache["canvas"][0], 3),
                                  gain, np.float32))
            gpu[d] = (gm1, gm2, gw1, gw3, ggain3)
        # Assignment occurs only after all four uploads completed.  This is
        # critical because renderers retain a cache snapshot while views change.
        cache["gpu"] = gpu

    def _bev_build_weights(self, cache, allowed=None):
        if allowed is None:
            allowed = cache.get("seam_allow")
        transition_px = int(round(self.bev_transition_m * cache["scale"]))
        owner, weights = self._bev_owner_and_weights(cache["supports"], cache["scores"],
                                                      transition_px, allowed,
                                                      cache.get("hard_seam_mask"))
        cache["owner"] = owner
        cache["weights"] = weights
        self._bev_upload_gpu(cache)

    def _bev_direct_warp(self, frame, cache, d):
        return cv2.remap(frame, cache["mapx"][d], cache["mapy"][d],
                         cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)

    def _bev_cpu_compose(self, frames, cache, weights=None, gains=None):
        weights = cache["weights"] if weights is None else weights
        gains = cache["gains"] if gains is None else gains
        h, w = cache["canvas"][1], cache["canvas"][0]
        acc = np.zeros((h, w, 3), np.float32)
        wsum = np.zeros((h, w), np.float32)
        for d, fr in frames.items():
            weight = weights[d]
            acc += self._bev_direct_warp(fr, cache, d).astype(np.float32) * weight[..., None] * float(gains.get(d, 1.0))
            wsum += weight
        nz = wsum > 1e-6
        acc[nz] /= wsum[nz, None]
        acc[~nz] = (35, 39, 44)
        return np.clip(acc, 0, 255).astype(np.uint8)

    def bev_load(self):
        """用保存 H 的原坐标系生成当前地面范围的 GPU/CPU 映射与有效权重。"""
        rd = results_dir()
        # Keep the prior complete cache readable while the replacement is being
        # built.  A cache is never cleared or modified in place.
        with self._lock:
            build_version = self.bev_cache_version + 1
        try:
            intr = {}
            for d in DIRECTIONS:
                jp = rd / f"{d}.json"
                if not jp.is_file():
                    self.bev_error = f"缺内参 {d}.json（先做内参标定）"
                    return False
                with jp.open() as f:
                    intr[d] = json.load(f)
            ep = rd / "extrinsics.json"
            if (rd / "extrinsics.stale.json").exists():
                self.bev_error = "外参依赖的内参已更新，需重新完成四路外参标定"
                return False
            if not ep.is_file():
                self.bev_error = "缺 extrinsics.json（先做外参标定）"
                return False
            with ep.open() as f:
                extr = json.load(f)
            sources = extr.get("homography_source") or {}
            pose_rebuilt = any(
                item.get("source") == "saved_6dof_poses"
                for item in (extr.get("recovery_history") or []))
            invalid_legacy = []
            if pose_rebuilt:
                for d, pose in (extr.get("poses") or {}).items():
                    if sources.get(d) in ("single_position_direct_h",
                                           "multi_position_direct_h"):
                        continue
                    quality = HOM.analyze_camera_pose(pose, d, self.pose_qc)
                    if quality.get("status") == "bad":
                        invalid_legacy.append(d)
            if invalid_legacy:
                self.bev_error = (
                    "旧外参由异常朝内位姿重建，已拒绝加载；请重新标定四路外参："
                    + "、".join(invalid_legacy))
                return False
            H_all = {d: np.asarray(M, dtype=np.float64)
                     for d, M in (extr.get("homographies") or {}).items()}
            if set(H_all) != set(DIRECTIONS):
                self.bev_error = "BEV 需要四路正式外参"
                return False
            balance = float(extr.get("balance", self.avm_bev["balance"]))
            if abs(balance - float(self.avm_bev["balance"])) > 1e-6:
                self.bev_error = "去畸变 balance 与外参保存时不一致，不能安全拼接"
                return False
            src_scale = float(extr.get("scale_px_per_meter", self.avm_bev["scale_px_per_meter"]))
            src_canvas = tuple(int(x) for x in extr.get("canvas_size", self.avm_bev["canvas_size"]))
            src_center = tuple(float(x) for x in extr.get("vehicle_center", [src_canvas[0] / 2, src_canvas[1] / 2]))
            canvas_px = int(self.bev_canvas_px)
            if not 400 <= canvas_px <= 1000:
                raise ValueError("BEV 输出尺寸需在 400–1000 像素之间")
            canvas = (canvas_px, canvas_px)
            out_scale = float(canvas[0]) / self.bev_view_m
            ratio = out_scale / src_scale
            view_T = np.array([[ratio, 0.0, canvas[0] / 2 - ratio * src_center[0]],
                               [0.0, ratio, canvas[1] / 2 - ratio * src_center[1]],
                               [0.0, 0.0, 1.0]], dtype=np.float64)
            xx, yy = np.meshgrid(np.arange(canvas[0], dtype=np.float32),
                                 np.arange(canvas[1], dtype=np.float32))
            gx = (xx - canvas[0] / 2) / out_scale
            gy = -(yy - canvas[1] / 2) / out_scale
            # 直接反向映射：BEV 像素 → 单应逆变换 → 归一化相机射线 → 原始鱼眼。
            # 不再经过有限大小的矫正画布，避免把可用鱼眼画面裁掉。
            measured_boards = extr.get("board_measured_bev") or {}
            saved_poses = extr.get("poses") or {}
            mapx, mapy, supports, scores, reverse_rejected, direction_rejected = {}, {}, {}, {}, {}, {}
            pose_consistency = {}
            ground_envelopes = extr.get("ground_envelopes") or {}
            use_cuda = cuda_backend_available()
            dst_homogeneous = np.stack((xx.ravel(), yy.ravel(),
                                         np.ones(xx.size, dtype=np.float32)), axis=0).astype(np.float64)
            for d in DIRECTIONS:
                w, h = intr[d].get("image_size", [self.width, self.height])
                w, h = int(w), int(h)
                K = np.asarray(intr[d]["K"], dtype=np.float64)
                D = np.asarray(intr[d]["D"], dtype=np.float64)
                new_k = fisheye_math.undistort_new_K(K, w, h, balance)
                qc_consistency = HOM.homography_matches_saved_qc(
                    H_all[d], d, (w, h), src_canvas,
                    (extr.get("homography_qc") or {}).get(d))
                if not qc_consistency["ok"]:
                    raise ValueError(
                        f"{d} 外参完整性检查失败：{qc_consistency['reason']}。"
                        "拒绝加载，避免画面拉伸")
                if d in saved_poses:
                    H_pose = HOM.homography_from_pose(
                        saved_poses[d], new_k, src_scale, src_center)
                    pose_consistency[d] = HOM.homography_pose_consistency(
                        H_all[d], H_pose, (w, h), src_canvas)
                Hs = view_T @ H_all[d]
                board = np.asarray(measured_boards.get(d), dtype=np.float64)
                if board.shape != (4, 2) or not np.isfinite(board).all():
                    raise ValueError(f"{d} 缺少已保存棋盘参考点，不能安全排除反向投影")
                board_dst = cv2.perspectiveTransform(
                    board.reshape(1, -1, 2).astype(np.float64), view_T).reshape(-1, 2)
                branch_valid = self._bev_valid_homography_branch(
                    Hs, board_dst.mean(axis=0), xx, yy)
                inv_h = np.linalg.inv(Hs)
                undist_h = inv_h @ dst_homogeneous
                den = undist_h[2]
                with np.errstate(divide="ignore", invalid="ignore"):
                    rays = np.linalg.inv(new_k) @ undist_h
                    normalized = (rays[:2] / rays[2]).T.reshape(-1, 1, 2)
                raw = cv2.fisheye.distortPoints(normalized.astype(np.float64), K, D)
                raw = raw.reshape(canvas[1], canvas[0], 2).astype(np.float32)
                bx, by = raw[..., 0], raw[..., 1]
                raw_support = (np.isfinite(bx) & np.isfinite(by)
                               & (bx >= 0) & (bx < w - 1) & (by >= 0) & (by < h - 1))
                # raw_support 只说明能从鱼眼取到颜色；branch_valid 才说明这条
                # 单应逆映射在棋盘所处的正向地面分支，二者缺一不可。
                geometric_support = raw_support & branch_valid
                direction_score = self._bev_angle_weight(d, gx, gy)
                direction_valid = direction_score > 0.0
                envelope = ground_envelopes.get(d)
                if isinstance(envelope, dict):
                    ax, ay = CAM_AXIS[d]
                    forward = gx * ax + gy * ay
                    lateral = gx * (-ay) + gy * ax
                    direction_valid &= (
                        (forward >= float(envelope.get("forward_min_m", 0.0)))
                        & (forward <= float(envelope.get("forward_max_m", self.bev_view_m)))
                        & (np.abs(lateral) <= float(envelope.get(
                            "lateral_abs_max_m", self.bev_view_m))))
                support = geometric_support & direction_valid
                mapx[d], mapy[d], supports[d] = bx.astype(np.float32), by.astype(np.float32), support
                reverse_rejected[d] = raw_support & ~branch_valid
                direction_rejected[d] = geometric_support & ~direction_valid
                scores[d] = direction_score * support.astype(np.float32)
            cache = {"mapx": mapx, "mapy": mapy, "supports": supports, "scores": scores,
                     "canvas": canvas, "scale": out_scale, "view_m": self.bev_view_m,
                     "source_canvas": src_canvas, "backend": "cuda" if use_cuda else "cpu",
                     "gains": {d: 1.0 for d in DIRECTIONS}, "seam_allow": None,
                     "reverse_rejected": reverse_rejected,
                     "direction_rejected": direction_rejected,
                     "pose_consistency": pose_consistency}
            self._bev_update_coverage(cache)
            self._bev_build_weights(cache)
            fingerprint = ADV.calibration_fingerprint(rd, DIRECTIONS)
            seam_lut, seam_reason = ADV.load_seam_lut(
                rd / "seam_lut.npz", fingerprint, cache, DIRECTIONS)
            if seam_lut is not None:
                cache["seam_allow"] = seam_lut["allowed"]
                cache["gains"] = seam_lut["gains"]
                cache["hard_seam_mask"] = seam_lut.get("hard_mask")
                cache["seam_quality"] = seam_lut.get("quality", {})
                self._bev_build_weights(cache, allowed=cache["seam_allow"])
                seam_state = {"state": "ready", "reason_code": "loaded",
                              "message": "已加载与当前标定匹配的持久接缝",
                              "updated_at": datetime.now().isoformat(timespec="seconds"),
                              "gains": dict(cache["gains"]), "persistent": True,
                              "fingerprint": fingerprint[:12],
                              **dict(cache.get("seam_quality", {}))}
            else:
                seam_state = {"state": "pending", "reason_code": "lut_unavailable",
                              "message": (
                    "使用固定方向选路；持久接缝不可用：" + seam_reason),
                              "updated_at": None, "gains": dict(cache["gains"]),
                              "persistent": False, "fingerprint": fingerprint[:12]}
            with self._lock:
                self.bev_backend = "cuda" if use_cuda else "cpu"
                self.bev_cache = cache
                self.bev_cache_version = build_version
                self.bev_fingerprint = fingerprint
                self.bev_seam = seam_state
                self.bev_preview_cache = None
            with self._lock:
                self.bev_loaded_ok = True
                self.bev_error = ""
            self.log(f"[bev] {self.bev_backend} 映射就绪：{self.bev_view_m:.1f}m，{canvas}")
            return True
        except Exception as exc:  # noqa: BLE001
            with self._lock:
                self.bev_error = str(exc)
                self.bev_loaded_ok = False
            return False

    def _bev_finish(self, acc, wsum, active, started, newest_ts):
        if not active:
            return None
        nz = wsum > 1e-6
        acc[nz] /= wsum[nz][:, None]
        acc[~nz] = (35, 39, 44)
        now = time.monotonic()
        self.bev_stats = {"fps": round(1.0 / max(now - started, 1e-6), 1),
                          "render_ms": round((now - started) * 1000, 1),
                          "frame_age_ms": round(max(0.0, now - newest_ts) * 1000, 1) if newest_ts else None,
                          "active_dirs": active, "backend": self.bev_backend}
        return np.clip(acc, 0, 255).astype(np.uint8)

    def bev_render(self):
        # 取完整缓存快照，避免“重新加载”与旧 MJPEG 请求交叉时读到半状态。
        with self._lock:
            if not self.bev_loaded_ok:
                return None
            c = self.bev_cache
            mode = self.bev_display_mode
        started = time.monotonic()
        canvas = c["canvas"]
        if mode in ("coverage", "ownership"):
            colors = {"front": (65, 190, 65), "back": (220, 155, 45),
                      "left": (215, 75, 180), "right": (65, 145, 230)}
            # 覆盖诊断显示可见范围；接缝归属诊断显示每个像素最终由哪一路提供。
            img = np.full((canvas[1], canvas[0], 3), (20, 24, 30), np.uint8)
            for d in DIRECTIONS:
                mask = c["supports"][d] if mode == "coverage" else (c["owner"] == DIRECTIONS.index(d))
                img[mask] = np.maximum(img[mask], np.asarray(colors[d], dtype=np.uint8))
            self.bev_stats = {"fps": 0.0, "render_ms": 0.0, "frame_age_ms": None,
                              "active_dirs": list(DIRECTIONS), "backend": self.bev_backend}
            return img
        frames = []
        for d in DIRECTIONS:
            fr, ts = self.latest_frame(d)
            if fr is not None and (mode in ("blend", "surround") or mode == d):
                frames.append((d, fr, ts))
        if not frames:
            return None
        if c["backend"] == "cuda":
            try:
                gacc, gsum = cv2.cuda_GpuMat(), cv2.cuda_GpuMat()
                gacc.upload(np.zeros((canvas[1], canvas[0], 3), np.float32))
                gsum.upload(np.zeros((canvas[1], canvas[0]), np.float32))
                active, newest_ts = [], 0.0
                for d, fr, ts in frames:
                    gs = cv2.cuda_GpuMat(); gs.upload(fr)
                    gm1, gm2, gw1, gw3, ggain3 = c["gpu"][d]
                    warped = cv2.cuda.remap(gs, gm1, gm2, cv2.INTER_LINEAR)
                    weighted = cv2.cuda.multiply(warped.convertTo(cv2.CV_32F), gw3)
                    weighted = cv2.cuda.multiply(weighted, ggain3)
                    gacc = cv2.cuda.add(gacc, weighted)
                    gsum = cv2.cuda.add(gsum, gw1)
                    active.append(d); newest_ts = max(newest_ts, ts)
                self.bev_backend = "cuda"
                result = self._bev_finish(gacc.download(), gsum.download(), active, started, newest_ts)
                return self.bev_make_surround(result) if mode == "surround" else result
            except cv2.error as exc:
                c["backend"] = "cpu"
                self.bev_backend = "cpu"
                self.log(f"[bev] CUDA 回退 CPU：{exc}")
        acc = np.zeros((canvas[1], canvas[0], 3), dtype=np.float32)
        wsum = np.zeros((canvas[1], canvas[0]), dtype=np.float32)
        active, newest_ts = [], 0.0
        for d, fr, ts in frames:
            warped = self._bev_direct_warp(fr, c, d)
            weight = c["weights"][d]
            acc += warped.astype(np.float32) * weight[..., None] * float(c["gains"].get(d, 1.0))
            wsum += weight
            active.append(d); newest_ts = max(newest_ts, ts)
        self.bev_backend = "cpu"
        result = self._bev_finish(acc, wsum, active, started, newest_ts)
        return self.bev_make_surround(result) if mode == "surround" else result

    def bev_make_surround(self, metric):
        """Create a non-metric bowl-style observation view from the metric image."""
        if metric is None:
            return None
        h, w = metric.shape[:2]
        flat_m, outer_m, lift_m, pitch_deg = 0.75, 2.0, 0.8, 55.0
        key = (w, h, flat_m, outer_m, lift_m, pitch_deg)
        cache = self.bev_cache.get("bowl_map")
        if cache is None or cache[0] != key:
            yy, xx = np.indices((h, w), dtype=np.float32)
            cx, cy = w / 2.0, h / 2.0
            dx = xx - cx
            # Virtual pitch compresses depth while leaving the vehicle center stable.
            dy = (yy - cy) / max(np.sin(np.deg2rad(pitch_deg)), 0.2)
            radius = np.hypot(dx, dy)
            flat_px = flat_m * w / max(self.bev_view_m, 1e-6)
            outer_px = outer_m * w / max(self.bev_view_m, 1e-6)
            norm = np.clip((radius - flat_px) / max(outer_px - flat_px, 1.0), 0.0, 1.0)
            curve_power = 1.0 / (1.0 + lift_m / max(outer_m, 1e-6))
            # Pull distant source pixels inward according to the virtual lift.
            source_radius = radius.copy()
            source_radius += ((outer_px - flat_px)
                              * (np.power(norm, curve_power) - norm))
            factor = source_radius / np.maximum(radius, 1e-6)
            mapx = (cx + dx * factor).astype(np.float32)
            mapy = (cy + dy * factor).astype(np.float32)
            self.bev_cache["bowl_map"] = (key, mapx, mapy)
        else:
            _, mapx, mapy = cache
        bowl = cv2.remap(metric, mapx, mapy, cv2.INTER_LINEAR,
                         borderMode=cv2.BORDER_CONSTANT, borderValue=(25, 28, 32))
        cv2.putText(bowl, "SURROUND VIEW / NON-METRIC", (16, h - 18),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (185, 205, 220), 1, cv2.LINE_AA)
        return bowl

    @staticmethod
    def _bev_estimate_gains(warps, masks):
        """只从有效且未饱和的重叠地面估计整路曝光差。"""
        equations, rhs = [], []
        grays = {d: cv2.cvtColor(warps[d], cv2.COLOR_BGR2GRAY).astype(np.float32)
                 for d in warps}
        for ia, a in enumerate(DIRECTIONS):
            for b in DIRECTIONS[ia + 1:]:
                if a not in warps or b not in warps:
                    continue
                good = masks[a] & masks[b] & (grays[a] > 28) & (grays[a] < 225) \
                       & (grays[b] > 28) & (grays[b] < 225)
                if int(good.sum()) < 1200:
                    continue
                ratio = np.median(grays[b][good] / np.maximum(grays[a][good], 1.0))
                if not np.isfinite(ratio) or not 0.55 < ratio < 1.8:
                    continue
                row = np.zeros(len(DIRECTIONS), np.float32)
                row[DIRECTIONS.index(a)] = 1.0
                row[DIRECTIONS.index(b)] = -1.0
                equations.append(row); rhs.append(float(np.log(ratio)))
        if not equations:
            return {d: 1.0 for d in DIRECTIONS}
        anchor = np.zeros(len(DIRECTIONS), np.float32); anchor[0] = 1.0
        equations.append(anchor); rhs.append(0.0)
        x, *_ = np.linalg.lstsq(np.asarray(equations), np.asarray(rhs), rcond=None)
        return {d: float(np.clip(np.exp(x[i]), 0.85, 1.15)) for i, d in enumerate(DIRECTIONS)}

    def _bev_capture_stable_frames(self):
        """仅用连续的新帧计算接缝，避免把移动中的人或椅子锁入接缝。"""
        samples, previous_ids = [], None
        deadline = time.monotonic() + 6.0
        while len(samples) < 4:
            frames, ids, newest = {}, {}, 0.0
            for d in DIRECTIONS:
                frame, ts, frame_id = self.latest_frame_info(d)
                if frame is None:
                    raise RuntimeError(f"{d} 相机没有可用画面")
                frames[d], ids[d], newest = frame.copy(), int(frame_id), max(newest, ts)
            if previous_ids is not None and any(ids[d] == previous_ids[d] for d in DIRECTIONS):
                if time.monotonic() >= deadline:
                    raise RuntimeError("没有收到四路连续新帧")
                time.sleep(0.04)
                continue
            previous_ids = ids
            preview = {d: cv2.resize(cv2.cvtColor(frames[d], cv2.COLOR_BGR2GRAY),
                                     (160, 128), interpolation=cv2.INTER_AREA).astype(np.float32)
                       for d in DIRECTIONS}
            samples.append((frames, preview, newest))
            with self._lock:
                self.bev_seam.update({"state": "sampling", "sample_count": len(samples),
                                      "sample_required": 4,
                                      "message": f"正在采集稳定画面 {len(samples)}/4"})
            time.sleep(0.08)
        deltas = [np.mean(np.abs(samples[i][1][d] - samples[i - 1][1][d]))
                  for i in range(1, len(samples)) for d in DIRECTIONS]
        motion_delta = float(max(deltas, default=0.0))
        if motion_delta > 3.0:
            raise RuntimeError(f"场景仍在移动（帧差 {motion_delta:.1f}），请保持现场静止后重试")
        # The four samples are deliberately collapsed per camera.  Returning
        # only the last image made the stability check ineffective and allowed
        # a person/cable edge to decide the persisted seam.
        median_frames = {
            d: np.median(np.stack([sample[0][d] for sample in samples], axis=0),
                         axis=0).astype(np.uint8)
            for d in DIRECTIONS
        }
        return median_frames, max(sample[2] for sample in samples), motion_delta

    def _bev_compute_seam(self, task_version, task_cache, task_fingerprint):
        try:
            cache = task_cache
            frames, newest, motion_delta = self._bev_capture_stable_frames()
            # 只在静止帧上估一次曝光增益；实时路径保持 GPU 直接采样与几何扇区权重。
            size = (500, 500)
            images, supports_small = {}, {}
            for d in DIRECTIONS:
                mx = cv2.resize(cache["mapx"][d], size, interpolation=cv2.INTER_LINEAR)
                my = cv2.resize(cache["mapy"][d], size, interpolation=cv2.INTER_LINEAR)
                images[d] = cv2.remap(frames[d], mx, my, cv2.INTER_LINEAR).astype(np.float32)
                supports_small[d] = cv2.resize(cache["supports"][d].astype(np.uint8), size,
                                               interpolation=cv2.INTER_NEAREST).astype(bool)
            online = self._bev_cpu_compose(frames, cache)
            gains = self._bev_estimate_gains(images, supports_small)
            transition_px = int(round(self.bev_transition_m * cache["scale"]))
            allowed, hard_mask, _, _ = ADV.geometric_seam(
                cache["supports"], cache["scores"], DIRECTIONS, transition_px)
            fixed_owner, fixed_weights = self._bev_owner_and_weights(
                cache["supports"], cache["scores"], transition_px)
            new_cache = dict(cache)
            new_cache["seam_allow"] = allowed
            new_cache["gains"] = gains
            new_cache["hard_seam_mask"] = hard_mask
            self._bev_build_weights(new_cache, allowed=allowed)
            quality = ADV.geometric_seam_quality(
                new_cache["owner"], fixed_owner, cache["supports"], SEAM_PAIRS)
            old_owner = cache.get("owner")
            if old_owner is not None:
                quality["changed_px"] = int(np.count_nonzero(new_cache["owner"] != old_owner))
            pairs = quality["pairs"]
            changed_px = int(quality["changed_px"])
            fixed = self._bev_cpu_compose(frames, cache, weights=fixed_weights, gains=gains)
            seam = self._bev_cpu_compose(frames, new_cache)
            new_cache["seam_quality"] = quality
            encoded = {}
            for name, image in {"online": online, "fixed": fixed, "seam": seam}.items():
                ok, png = cv2.imencode(".png", image)
                if ok:
                    encoded[name] = png.tobytes()
            encoded["before"] = encoded.get("fixed", b"")
            encoded["after"] = encoded.get("seam", b"")
            with self._lock:
                if (cache is not self.bev_cache or self.bev_cache_version != task_version
                        or self.bev_fingerprint != task_fingerprint):
                    self.bev_seam = {"state": "pending", "reason_code": "stale_task",
                                     "message": "视图或标定已更新，已丢弃旧接缝任务结果",
                                     "updated_at": None, "gains": {d: 1.0 for d in DIRECTIONS}}
                    return
                # Persist only after every quality gate and stale-cache check.
                ADV.save_seam_lut(results_dir() / "seam_lut.npz",
                                  task_fingerprint, new_cache, DIRECTIONS)
                self.bev_cache = new_cache
                self.bev_cache_version += 1
                self.bev_compare = {k: v for k, v in encoded.items() if v}
                self.bev_seam = {
                    "state": "ready", "reason_code": "optimized",
                    "message": "已应用几何接缝",
                    "updated_at": datetime.now().isoformat(timespec="seconds"), "gains": gains,
                    "sample_count": 4, "sample_required": 4, "motion_delta": round(motion_delta, 2),
                    "changed_px": changed_px, "pairs": pairs,
                    "persistent": True, "fingerprint": task_fingerprint[:12], **quality,
                }
                self.bev_preview_cache = None
                self.log(f"[bev] 几何接缝已锁定：{', '.join(pairs)}，相对旧选路改动 {changed_px} 像素")
        except Exception as exc:  # noqa: BLE001
            with self._lock:
                self.bev_seam = {"state": "failed", "reason_code": "quality_or_runtime_failed",
                                 "message": f"接缝优化失败：{exc}",
                                 "updated_at": None, "gains": {d: 1.0 for d in DIRECTIONS}}
                self.log(f"[bev] 接缝优化失败：{exc}")
        finally:
            with self._lock:
                self.bev_seam_task = None

    def bev_start_seam_optimize(self):
        with self._lock:
            if not self.bev_loaded_ok:
                return False, "请先加载标定结果"
            if self.bev_seam_task is not None:
                return False, "接缝优化正在进行"
            # A recent valid same-board diagnostic blocks seam optimization when
            # it exposes global geometry error.  Different-board observations
            # are not treated as geometry evidence and are retried on /seam.
            latest = {}
            for item in self.seam_diagnostics:
                if item.get("same_target"):
                    latest[f"{item.get('ref')}+{item.get('slave')}"] = item
            bad = [key for key, item in latest.items()
                   if float(item.get("rms_px", 0.0)) > 5.0
                   or float(item.get("scale_ratio", 1.0)) > 1.35]
            if bad:
                return False, "同一棋盘外参残差过大（%s）；请使用多位置外参，不能用融合掩盖" % "、".join(bad)
            self.bev_seam = {"state": "sampling", "message": "正在采集连续稳定画面 0/4",
                             "reason_code": "sampling", "updated_at": None,
                             "gains": {d: 1.0 for d in DIRECTIONS}, "sample_count": 0,
                             "sample_required": 4, "fingerprint": self.bev_fingerprint[:12]}
            self.bev_seam_task = threading.Thread(
                target=self._bev_compute_seam,
                args=(self.bev_cache_version, self.bev_cache, self.bev_fingerprint),
                name="bev-seam", daemon=True)
            self.bev_seam_task.start()
        return True, "已开始计算稳定接缝"

    def bev_png(self, kind):
        with self._lock:
            return self.bev_compare.get(kind)

    def bev_jpeg(self):
        """多个浏览器共享一帧编码结果，避免每个 MJPEG 客户端重复 GPU 融合。"""
        with self.bev_preview_lock:
            now = time.monotonic()
            if self.bev_preview_cache and now - self.bev_preview_cache[0] < 0.085:
                return self.bev_preview_cache[1]
            img = self.bev_render()
            if img is None:
                return None
            self.bev_overlay(img)
            data = _encode_jpeg(img)
            if data:
                self.bev_preview_cache = (now, data)
            return data

    def bev_overlay(self, img):
        with self._lock:
            c = self.bev_cache
            mode = self.bev_display_mode
        grid = self.bev_grid_m
        scale = float(c.get("scale", img.shape[1] / max(self.bev_view_m, 1e-6)))
        # 真正未被任何正向地面分支看到的区域，用低对比斜纹标出；
        # 这与车体占地是两个概念，不能用更大的车体方块掩盖它。
        blind = c.get("unobserved")
        if blind is not None and mode != "surround":
            body_for_blind = self._bev_body_mask(img.shape, scale, self.bev_body_size_m)
            blind = blind & ~body_for_blind
            yy, xx = np.indices(blind.shape)
            stripes = blind & (((xx + yy) % 12) < 2)
            img[stripes] = (58, 67, 76)
        if grid > 0 and mode != "surround":
            step_px = max(1, int(round(scale * grid)))
            for gx in range(0, img.shape[1], step_px):
                cv2.line(img, (gx, 0), (gx, img.shape[0]), (70, 70, 70), 1)
            for gy in range(0, img.shape[0], step_px):
                cv2.line(img, (0, gy), (img.shape[1], gy), (70, 70, 70), 1)
        cx, cy = img.shape[1] // 2, img.shape[0] // 2
        rect = self._bev_body_rect(img.shape, scale, self.bev_body_size_m)
        if rect is not None:
            x0, y0, x1, y1 = rect
            if mode != "coverage":
                cv2.rectangle(img, (x0, y0), (x1, y1), (28, 34, 43), -1)
            cv2.rectangle(img, (x0, y0), (x1, y1), (130, 147, 165), 1)
            cv2.arrowedLine(img, (cx, cy), (cx, min(y0 + 4, cy - 10)),
                            (130, 147, 165), 1, tipLength=0.28)
        if self.bev_show_seams and "owner" in c and mode != "surround":
            o = c["owner"]
            edge = (np.diff(o, axis=0, prepend=o[:1]) != 0) | (np.diff(o, axis=1, prepend=o[:, :1]) != 0)
            img[edge] = (0, 230, 255)
        cv2.circle(img, (cx, cy), 5, (0, 0, 255), -1)
        return img

    # ---------- 状态快照（给 /api/status）----------
    def snapshot(self):
        with self._lock:
            intr_done = {d: (d in self.intr_results) for d in DIRECTIONS}
            intr_audits = {d: self._audit_intr_samples(self.intr_samples[d])
                           for d in DIRECTIONS}
            intr_usable = {
                d: [sample for _, sample in intr_audits[d][0]]
                for d in DIRECTIONS
            }
            intr_counts = {d: len(intr_usable[d]) for d in DIRECTIONS}
            intr_sample_stats = {}
            for d in DIRECTIONS:
                pending_rejected = intr_audits[d][1]
                stored_rejected = self.intr_rejected[d]
                reasons = {}
                for item in stored_rejected:
                    reason = str(item.get("reason", "unknown"))
                    reasons[reason] = reasons.get(reason, 0) + 1
                for _, _, reason, _ in pending_rejected:
                    reasons[reason] = reasons.get(reason, 0) + 1
                intr_sample_stats[d] = {
                    "captured": len(self.intr_samples[d]) + len(stored_rejected),
                    "usable": len(intr_usable[d]),
                    "quarantined": len(stored_rejected) + len(pending_rejected),
                    "pending_quarantine": len(pending_rejected),
                    "reasons": reasons,
                    "target": 25,
                    "minimum": MIN_INTR_SAMPLES,
                }
            intr_solution = {d: (self.intr_candidates.get(d) or self.intr_results.get(d))
                             for d in DIRECTIONS}
            intr_metrics = {
                d: {
                    "source": ("candidate" if d in self.intr_candidates
                               else ("saved" if d in self.intr_results else None)),
                    "rms": (round(float(sol.get("rms")), 4)
                            if sol and sol.get("rms") is not None else None),
                    "sample_count": (int(sol.get("sample_count"))
                                     if sol and sol.get("sample_count") is not None
                                     else intr_counts[d]),
                }
                for d, sol in intr_solution.items()
            }
            intr_quality = {
                d: ((sol.get("evaluation") or {}).get("status") if sol else None)
                for d, sol in intr_solution.items()
            }
            ext_done = ({d: (d in self.ext_pending) for d in DIRECTIONS}
                        if self.ext_pending else
                        {d: (d in self.H) for d in DIRECTIONS})
            ext_rms = {d: round(self.rms_errors.get(d, 0.0), 3)
                       for d in DIRECTIONS}
            ext_live = {}
            for d in DIRECTIONS:
                live = self.ext_live.get(d, {})
                ext_live[d] = {
                    "phase": live.get("phase", "idle"),
                    "reason": live.get("reason", ""),
                    "reason_text": self._ext_reason_text(live.get("reason", "")),
                    "frame_id": int(live.get("frame_id", -1)),
                    "corners_detected": int(live.get("corners_detected", 0)),
                    "expected": int(live.get("expected", self.cols * self.rows)),
                    "inview_count": int(live.get("inview_count", 0)),
                    "motion_rms_px": live.get("motion_rms_px"),
                    "detected_pattern": live.get("detected_pattern"),
                }
            ext_burst = {}
            for d in DIRECTIONS:
                item = dict(self.ext_burst_state.get(d, {}))
                item["evidence"] = [
                    {k: v for k, v in shot.items() if k != "jpeg"}
                    for shot in self.ext_burst_evidence.get(d, [])
                ]
                ext_burst[d] = item
            ext_candidate_detail = {}
            for d, c in self.ext_candidate.items():
                q = c.get("qc", {})
                ext_candidate_detail[d] = {
                    "rms": round(float(c.get("rms", 0.0)), 4),
                    "quality": q.get("status"),
                    "warnings": list(q.get("warnings", [])),
                    "failures": list(q.get("failures", [])),
                    "n_used": int(c.get("burst", {}).get("n_used", 0)),
                    "corner_jitter_px": c.get("burst", {}).get("corner_jitter_px"),
                }
            camera_status = {}
            for d in DIRECTIONS:
                g = self.grabbers.get(d)
                fr, _, frame_id = self.latest_frame_info(d)
                camera_status[d] = {
                    "available": g is not None and fr is not None,
                    "device": (g.device if g is not None
                               else f"/dev/video{int(self.cfg['capture']['cameras'][d]['device'])}"),
                    "frame_id": int(frame_id),
                    "fps": round(float(g.actual_fps), 1) if g is not None else 0.0,
                }
            prog = {d: [round(p, 2) for p in self.intr_progress(intr_usable[d])]
                    for d in DIRECTIONS}
            return {
                "mode": self.mode,
                "active_dir": self.active_dir,
                "target": self.target,
                "ext_running": self.ext_running,
                "ext_candidate": {d: d in self.ext_candidate for d in DIRECTIONS},
                "streak": self.streak,
                "bursting": self.bursting,
                "ext_stable_required": int(self.ext_cfg["stable_frames"]),
                "ext_live": ext_live,
                "ext_burst": ext_burst,
                "ext_candidate_detail": ext_candidate_detail,
                "ext_pending": {
                    d: ({"rms": round(float(self.ext_pending[d]["rms"]), 4),
                         "quality": self.ext_pending[d]["qc"].get("status")}
                        if d in self.ext_pending else None)
                    for d in DIRECTIONS},
                "ext_saved": {d: dict(self.ext_saved_catalog.get(
                    d, {"state": "missing"})) for d in DIRECTIONS},
                "ext_results_stale": self.ext_results_stale,
                "camera_status": camera_status,
                "seam_mode": self.seam_mode,
                "seam_complete": self.seam_complete,
                "seam_pair": (list(SEAM_PAIRS[self.seam_pair_i])
                              if self.seam_pair_i < len(SEAM_PAIRS)
                              else None),
                "seam_pair_index": self.seam_pair_i,
                "seam_pairs": [list(x) for x in SEAM_PAIRS],
                "seam_pair_count": len(SEAM_PAIRS),
                "seam_streak": self.seam_streak,
                "seam_last": (dict(self.seam_diagnostics[-1])
                              if self.seam_diagnostics else None),
                "intr_done": intr_done,
                "intr_counts": intr_counts,
                "intr_sample_stats": intr_sample_stats,
                "intr_progress": prog,
                "intr_collecting": dict(self.intr_collecting),
                "intr_candidate": {d: d in self.intr_candidates for d in DIRECTIONS},
                "intr_can_calibrate": {d: intr_counts[d] >= MIN_INTR_SAMPLES
                                        for d in DIRECTIONS},
                "intr_quality": intr_quality,
                "intr_metrics": intr_metrics,
                "intr_preview_available": {d: intr_solution[d] is not None
                                             for d in DIRECTIONS},
                "intr_preview_balance": dict(self.intr_preview_balance),
                "board": {"cols": self.cols, "rows": self.rows,
                          "square_m": self.square},
                "placements": self.placements,
                "intr_task_running": self.intr_task is not None,
                "intr_task_msg": self.intr_task_msg,
                "ui_messages": {
                    "intrinsics": dict(self.intr_task_ui)
                    if self.intr_task_ui else None,
                },
                "ext_done": ext_done,
                "ext_rms": ext_rms,
                "ext_task_running": self.ext_task is not None,
                "ext_task_msg": self.ext_task_msg,
                "extrinsics_session": self.multi_snapshot(),
                "bev_loaded": self.bev_loaded_ok,
                "bev_error": self.bev_error,
                "bev": {
                    "view_m": self.bev_view_m,
                    "grid_m": self.bev_grid_m,
                    "display_mode": self.bev_display_mode,
                    "backend": self.bev_backend,
                    "fps": self.bev_stats.get("fps", 0.0),
                    "render_ms": self.bev_stats.get("render_ms", 0.0),
                    "frame_age_ms": self.bev_stats.get("frame_age_ms"),
                    "active_dirs": list(self.bev_stats.get("active_dirs", [])),
                    "transition_m": self.bev_transition_m,
                    "show_seams": self.bev_show_seams,
                    "body_size_m": (list(self.bev_body_size_m)
                                    if self.bev_body_size_m is not None else None),
                    "coverage": dict(self.bev_cache.get("coverage", {})),
                    "seam": self._public_seam_state(self.bev_seam),
                    "compare_available": sorted(self.bev_compare),
                },
                "last_log": self.last_log,
                "directions": list(DIRECTIONS),
                "seam_pairs": [list(p) for p in SEAM_PAIRS],
            }


# =========================================================================
#  MJPEG 编码
# =========================================================================

_JPEG_CACHE: dict[tuple, tuple] = {}   # key=(dir, overlay) → (ts, bytes)
_PLACEHOLDER_CACHE: dict[tuple, bytes] = {}


def _encode_jpeg(frame, quality=90):
    ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, quality])
    return buf.tobytes() if ok else None


def _placeholder_jpeg(title: str, detail: str = "", size=(960, 768)):
    key = (title, detail, tuple(size))
    cached = _PLACEHOLDER_CACHE.get(key)
    if cached is not None:
        return cached
    width, height = (int(size[0]), int(size[1]))
    frame = np.full((height, width, 3), (22, 27, 35), dtype=np.uint8)
    cv2.putText(frame, title, (48, height // 2 - 12),
                cv2.FONT_HERSHEY_SIMPLEX, 1.15, (225, 232, 240), 2,
                cv2.LINE_AA)
    if detail:
        cv2.putText(frame, detail, (48, height // 2 + 38),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (125, 160, 195), 2,
                    cv2.LINE_AA)
    data = _encode_jpeg(frame, quality=82)
    if data is not None:
        _PLACEHOLDER_CACHE[key] = data
    return data


def stream_jpeg_camera(state: CalibState, d: str, overlay: str = ""):
    """单路相机 MJPEG 帧（可选棋盘角点 overlay）。"""
    fr, ts, frame_id = state.latest_frame_info(d)
    if fr is None:
        grabber = state.grabbers.get(d)
        device = grabber.device if grabber is not None else "camera unavailable"
        return _placeholder_jpeg(f"{d}: NO LIVE FRAME", device)
    # 缓存键含 overlay；候选参数变化也必须使矫正流失效。
    preview_revision = state.intr_preview_revision.get(d, 0) if overlay == "undist" else 0
    key = (d, overlay, preview_revision)
    cached = _JPEG_CACHE.get(key)
    if cached is not None and cached[0] == ts:
        return cached[1]
    pv = cv2.resize(fr, (max(1, int(fr.shape[1] * 0.5)),
                         max(1, int(fr.shape[0] * 0.5))))
    ov = pv.copy()
    if overlay == "undist":
        und = state.intr_undistort_preview(d, fr)
        ov = cv2.resize(und, (pv.shape[1], pv.shape[0]))
    elif overlay == "intr" and d == state.active_dir:
        corners = state.intr_last_corners.get(d)
        if corners is not None:
            cc = np.asarray(corners).reshape(-1, 2)
            sx = pv.shape[1] / fr.shape[1]
            sy = pv.shape[0] / fr.shape[0]
            for (x, y) in cc:
                cv2.circle(ov, (int(x * sx), int(y * sy)), 4, (0, 255, 0), -1)
        else:
            cv2.putText(ov, "NO BOARD", (12, 32), cv2.FONT_HERSHEY_SIMPLEX,
                        0.8, (0, 0, 255), 2)
    elif overlay == "ext":
        with state._lock:
            live = dict(state.ext_live.get(d, {}))
        phase = live.get("phase", "idle")
        reason = live.get("reason", "")
        # 只在检测结果属于当前视频帧时画角点，避免旧角点误导用户。
        raw = live.get("raw") if live.get("frame_id") == frame_id else None
        color = ((30, 210, 30) if phase in ("detected", "stable", "ready", "candidate")
                 else ((0, 210, 255) if phase in ("moving", "waiting", "bursting")
                       else (40, 60, 235)))
        if raw is not None:
            pts = np.asarray(raw).reshape(-1, 2)
            sx = pv.shape[1] / fr.shape[1]
            sy = pv.shape[0] / fr.shape[0]
            for x, y in pts:
                cv2.circle(ov, (int(x * sx), int(y * sy)), 3, color, -1)
            if len(pts) == state.cols * state.rows:
                outer = pts[HOM.grid_outer_idx(state.cols, state.rows)]
                quad = np.column_stack((outer[:, 0] * sx, outer[:, 1] * sy))
                cv2.polylines(ov, [np.int32(quad).reshape(-1, 1, 2)],
                              True, color, 2)
        else:
            cv2.putText(ov, "NO COMPLETE BOARD", (12, 32),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
        # cv2.putText 不支持中文，视频帧统一使用 ASCII；中文解释保留在网页右栏。
        overlay_reason = {
            "no_frame": "no frame", "board_not_found": "no complete board",
            "partial_board": "partial board only", "intrinsics_missing": "missing intrinsics",
            "out_of_view": "board out of view", "moving": "board moving",
            "stable": "board stable", "ready": "starting burst",
            "bursting": "capturing burst", "candidate": "candidate ready",
            "duplicate_frame": "waiting next frame",
        }.get(reason, reason or "waiting")
        detected = live.get("detected_pattern")
        size_hint = (f" detected {detected[0]}x{detected[1]}"
                     if detected else "")
        line = (f"{phase}: {overlay_reason}{size_hint}; corners "
                f"{live.get('corners_detected', 0)}/{state.cols * state.rows}")
        cv2.rectangle(ov, (0, 0), (min(ov.shape[1], 730), 32), (0, 0, 0), -1)
        cv2.putText(ov, line[:90], (10, 22), cv2.FONT_HERSHEY_SIMPLEX,
                    0.48, color, 1, cv2.LINE_AA)
    g = state.grabbers.get(d)
    fps = g.actual_fps if g else 0.0
    cv2.putText(ov, f"{d} {fr.shape[1]}x{fr.shape[0]} {fps:.1f}fps",
                (12, pv.shape[0] - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                (0, 255, 255), 1)
    data = _encode_jpeg(ov)
    if data:
        _JPEG_CACHE[key] = (ts, data)
    return data


def stream_jpeg_bev(state: CalibState):
    if not state.bev_loaded_ok:
        return _placeholder_jpeg("BEV UNAVAILABLE", "Complete valid extrinsics first",
                                 (1000, 1000))
    return state.bev_jpeg()


# =========================================================================
#  共享 UI（BASE_CSS + I18N + 渲染辅助）
# =========================================================================

BASE_CSS = (
    ":root{"
    "--bg:#0f1320;--surface:#1c2230;--surface-2:#252c3c;--surface-3:#2e3648;"
    "--border:#344255;--border-strong:#46546d;--text:#e8edf5;--muted:#aeb9c8;"
    "--accent:#3b82f6;--accent-hover:#5a9bff;--success:#22c55e;--warn:#f59e0b;"
    "--danger:#ef4444;--shadow:0 4px 16px rgba(0,0,0,.25);--radius:12px;"
    "--radius-sm:8px;"
    "}"
    "*{box-sizing:border-box}"
    "html,body{margin:0;padding:0}"
    "body{"
    "font-family:-apple-system,BlinkMacSystemFont,'Segoe UI',Roboto,"
    "'PingFang SC','Microsoft YaHei',sans-serif;"
    "background:var(--bg);color:var(--text);"
    "padding:clamp(12px,2.4vw,28px);min-height:100vh;"
    "}"
    "a{color:var(--accent);text-decoration:none}"
    "a:hover{color:var(--accent-hover);text-decoration:underline}"
    "h1,h2,h3{margin:0 0 12px;color:var(--text)}"
    "h1{font-size:clamp(20px,2vw,28px);font-weight:650}"
    "h2{font-size:17px;font-weight:650}"
    "h3{font-size:14px;font-weight:650;color:var(--muted);text-transform:uppercase;"
    "letter-spacing:.5px}"
    ".panel{background:var(--surface);border:1px solid var(--border);"
    "border-radius:var(--radius);padding:14px;box-shadow:var(--shadow)}"
    ".panel+ .panel{margin-top:14px}"
    ".panel-title{display:flex;justify-content:space-between;align-items:center;"
    "margin-bottom:10px;gap:8px;flex-wrap:wrap}"
    ".toolbar{display:flex;gap:8px;flex-wrap:wrap;align-items:center;"
    "padding:12px;background:var(--surface-2);border:1px solid var(--border);"
    "border-radius:var(--radius-sm);margin-bottom:14px}"
    ".toolbar label{display:flex;gap:6px;align-items:center;font-size:13px;"
    "color:var(--muted)}"
    ".toolbar select,.toolbar input[type=number],.toolbar input[type=text]"
    "{background:var(--bg);color:var(--text);border:1px solid var(--border-strong);"
    "border-radius:6px;padding:6px 8px;font-size:13px}"
    ".btn{display:inline-flex;align-items:center;gap:6px;padding:8px 14px;"
    "background:var(--surface-3);color:var(--text);border:1px solid var(--border-strong);"
    "border-radius:6px;cursor:pointer;font-size:13px;font-weight:600;"
    "transition:background .15s,transform .05s;user-select:none}"
    ".btn:hover{background:#3a4356}"
    ".btn:active{transform:translateY(1px)}"
    ".btn:disabled{opacity:.45;cursor:not-allowed}"
    ".btn-primary{background:var(--success);border-color:#1f7a3e;color:#fff}"
    ".btn-primary:hover{background:#2db854}"
    ".btn-secondary{background:#38506b;border-color:#3a5a7e;color:#fff}"
    ".btn-secondary:hover{background:#42628a}"
    ".btn-danger{background:var(--danger);border-color:#b03030;color:#fff}"
    ".btn-danger:hover{background:#d83838}"
    ".btn-sm{padding:5px 10px;font-size:12px}"
    ".nav{display:flex;gap:6px;align-items:center;margin-bottom:18px;flex-wrap:wrap;"
    "padding:10px 12px;background:var(--surface);border:1px solid var(--border);"
    "border-radius:var(--radius);box-shadow:var(--shadow)}"
    ".nav a{padding:8px 14px;background:var(--surface-2);color:var(--text);"
    "border-radius:6px;font-size:13px;font-weight:600;border:1px solid transparent}"
    ".nav a:hover{background:var(--surface-3);text-decoration:none}"
    ".nav a.active{background:var(--accent);border-color:var(--accent);color:#fff}"
    ".nav .spacer{flex:1}"
    ".lang-btn{display:inline-flex;align-items:center;gap:4px;padding:6px 10px;"
    "background:transparent;color:var(--muted);border:1px solid var(--border-strong);"
    "border-radius:6px;cursor:pointer;font-size:12px;font-weight:600}"
    ".lang-btn:hover{color:var(--text);border-color:var(--accent)}"
    ".layout{display:grid;gap:16px;align-items:start}"
    ".layout-2col{grid-template-columns:minmax(0,1fr) 340px}"
    ".layout-3col{grid-template-columns:minmax(0,1fr) 380px}"
    "@media(max-width:980px){.layout-2col,.layout-3col{grid-template-columns:1fr}}"
    ".video-wrap{background:#000;border-radius:var(--radius-sm);overflow:hidden;"
    "display:flex;align-items:center;justify-content:center;border:1px solid var(--border)}"
    ".video-wrap img,.video-wrap video{width:100%;height:100%;display:block;"
    "object-fit:contain}"
    "input,select{font:inherit;color:inherit}"
    "input[type=text],input[type=number]{background:var(--bg);color:var(--text);"
    "border:1px solid var(--border-strong);border-radius:6px;padding:6px 8px}"
    ".badge{display:inline-block;padding:3px 10px;border-radius:999px;"
    "font-size:12px;font-weight:600;letter-spacing:.2px}"
    ".badge.ok{background:rgba(34,197,94,.18);color:#7ee2a3;"
    "border:1px solid rgba(34,197,94,.4)}"
    ".badge.no{background:rgba(239,68,68,.18);color:#ffa4a4;"
    "border:1px solid rgba(239,68,68,.4)}"
    ".badge.warn{background:rgba(245,158,11,.18);color:#fcd58a;"
    "border:1px solid rgba(245,158,11,.4)}"
    ".badge.muted{background:var(--surface-3);color:var(--muted);border:1px solid var(--border)}"
    ".chip{display:inline-flex;align-items:center;gap:6px;background:var(--surface-2);"
    "border-left:3px solid var(--accent);padding:7px 11px;border-radius:6px;"
    "font-size:13px;color:var(--text)}"
    ".step-card{border-left:4px solid var(--border-strong);"
    "background:var(--surface-2);border-radius:6px;padding:10px 12px;margin:10px 0;"
    "transition:border-color .2s}"
    ".step-card.active{border-left-color:var(--success)}"
    ".step-card.warn{border-left-color:var(--warn)}"
    ".step-card.bad{border-left-color:var(--danger)}"
    ".step-card .step-title{font-weight:650;margin-bottom:6px;font-size:13px}"
    ".small{font-size:12px;color:var(--muted);line-height:1.55}"
    ".value{font-family:ui-monospace,'SF Mono',Menlo,monospace;font-size:13px;"
    "line-height:1.55;color:var(--text)}"
    ".log{font-family:ui-monospace,monospace;font-size:12px;color:#7ee2a3;"
    "white-space:pre-wrap;min-height:20px}"
    ".log.err{color:#ffa4a4}"
    ".alert{background:rgba(239,68,68,.12);border:1px dashed var(--danger);"
    "border-radius:8px;padding:24px 16px;color:#ffd1c9;text-align:center}"
    ".grid-auto{display:grid;grid-template-columns:repeat(auto-fit,minmax(320px,1fr));"
    "gap:14px}"
    ".tile{background:var(--surface);border:1px solid var(--border);"
    "border-radius:var(--radius);padding:12px;box-shadow:var(--shadow)}"
    ".tile h3{font-size:14px;color:var(--accent);margin-bottom:6px;"
    "text-transform:none;letter-spacing:0}"
    ".tile .stream{background:#000;border-radius:var(--radius-sm);overflow:hidden;"
    "aspect-ratio:16/10}"
    ".tile .stream img{width:100%;height:100%;object-fit:contain}"
    ".status-row{display:flex;gap:6px;margin-top:8px;flex-wrap:wrap}"
    ".gallery{display:grid;grid-template-columns:repeat(4,minmax(0,1fr));gap:8px;"
    "margin-top:10px}"
    "@media(max-width:680px){.gallery{grid-template-columns:repeat(2,minmax(0,1fr))}}"
    ".shot{min-width:0;border:1px solid var(--border);border-radius:6px;"
    "overflow:hidden;background:#0a0e15}"
    ".shot.bad{border-color:rgba(239,68,68,.5)}"
    ".shot img{width:100%;display:block;background:#000}"
    ".shot .cap{font-size:11px;padding:4px;color:var(--muted);"
    "white-space:nowrap;overflow:hidden;text-overflow:ellipsis}"
    ".progress{height:10px;background:var(--surface-3);border-radius:999px;"
    "overflow:hidden;margin:3px 0 8px}"
    ".progress>i{display:block;height:100%;background:var(--success);"
    "transition:width .3s ease}"
    ".technical-details{margin-top:7px;color:var(--muted);font-size:12px}"
    ".technical-details summary{cursor:pointer;color:var(--muted)}"
    ".technical-details pre{max-height:150px;overflow:auto;white-space:pre-wrap;"
    "overflow-wrap:anywhere;margin:6px 0 0;padding:8px;background:var(--bg);"
    "border:1px solid var(--border);border-radius:6px}"
    "::-webkit-scrollbar{width:8px;height:8px}"
    "::-webkit-scrollbar-thumb{background:var(--surface-3);border-radius:4px}"
    "::-webkit-scrollbar-track{background:transparent}"
)


I18N: dict[str, dict[str, str]] = {
    "zh": {
        "lang.toggle": "EN",
        "nav.home": "首页",
        "nav.intr": "内参",
        "nav.ext": "外参",
        "nav.ext_multi": "多点外参",
        "nav.seam": "接缝",
        "nav.bev": "BEV 预览",
        "common.start": "开始",
        "common.pause": "暂停",
        "common.stop": "停止",
        "common.save": "保存",
        "common.cancel": "取消",
        "common.refresh": "刷新",
        "common.loading": "读取中",
        "common.network_interrupted": "网络暂时中断",
        "common.operation_failed": "操作失败",
        "common.status_updated": "状态已更新",
        "common.technical_details": "技术详情",
        "intr.camera_selected": "已切换到 {direction}；采集保持暂停",
        "intr.capturing": "正在采集 {direction}；已有 {count} 张有效样本",
        "intr.capture_paused": "{direction} 采集已暂停",
        "intr.calibrating": "正在标定 {direction}：{usable} 张可用，{quarantined} 张待隔离",
        "intr.calibrated": "{direction} 标定完成：使用 {used} 张，隔离 {quarantined} 张，RMS {rms}px（{quality}）",
        "intr.calibration_failed": "{direction} 标定失败；请查看技术详情",
        "intr.not_enough_usable": "可用样本不足：{usable}/{minimum}，已隔离 {quarantined} 张",
        "intr.sample_quarantined": "检测到异常棋盘网格，样本已隔离（{reason}）",
        "intr.samples_cleared": "{direction} 本轮样本已清空；正式结果保留",
        "intr.saved": "{direction} 正式内参已保存",
        "intr.preview_balance": "{direction} 预览缩放已设为 {balance}；不会写入标定结果",
        "intr.usable_samples": "可用样本",
        "intr.captured": "已采集",
        "intr.target": "目标",
        "intr.minimum": "最少",
        "intr.quarantined": "已隔离",
        "reason.invalid_shape": "角点数量或形状错误",
        "reason.nonfinite": "角点包含无效数值",
        "reason.out_of_bounds": "角点超出图像边界",
        "reason.collapsed_edge": "相邻角点发生重叠",
        "reason.collapsed_cell": "棋盘网格单元塌缩",
        "reason.folded_grid": "棋盘角点顺序折叠",
        "reason.opencv_ill_conditioned": "OpenCV 判定该视图病态",
        "reason.no_frame": "尚未收到相机画面",
        "reason.board_not_found": "未检测到完整棋盘",
        "reason.partial_board": "仅识别到部分棋盘",
        "reason.intrinsics_missing": "该方向缺少正式内参",
        "reason.out_of_view": "角点太靠近画面边缘",
        "reason.seam_edge_valid": "边缘棋盘已完整检出",
        "reason.seam_raw_edge": "棋盘太靠近原图边缘",
        "reason.seam_nonfinite": "去畸变坐标无效",
        "reason.seam_outside_bev": "棋盘不在 BEV 有效范围",
        "reason.seam_too_small": "棋盘在 BEV 中跨度过小",
        "reason.moving": "棋盘仍在移动",
        "reason.stable": "棋盘稳定，正在确认",
        "reason.ready": "棋盘稳定，可以连拍",
        "reason.bursting": "正在自动连拍",
        "reason.candidate": "候选结果已生成",
        "reason.duplicate_frame": "等待相机新画面",
        "reason.burst_timeout": "等待新画面超时",
    },
    "en": {
        "lang.toggle": "中文",
        "nav.home": "Home",
        "nav.intr": "Intrinsics",
        "nav.ext": "Extrinsics",
        "nav.ext_multi": "Multi-position",
        "nav.seam": "Seam",
        "nav.bev": "BEV",
        "common.start": "Start",
        "common.pause": "Pause",
        "common.stop": "Stop",
        "common.save": "Save",
        "common.cancel": "Cancel",
        "common.refresh": "Refresh",
        "common.loading": "Loading",
        "common.network_interrupted": "Network temporarily interrupted",
        "common.operation_failed": "Operation failed",
        "common.status_updated": "Status updated",
        "common.technical_details": "Technical details",
        "intr.camera_selected": "Switched to {direction}; capture remains paused",
        "intr.capturing": "Capturing {direction}; {count} usable samples saved",
        "intr.capture_paused": "Capture paused for {direction}",
        "intr.calibrating": "Calibrating {direction}: {usable} usable, {quarantined} pending quarantine",
        "intr.calibrated": "{direction} calibrated: {used} used, {quarantined} quarantined, RMS {rms}px ({quality})",
        "intr.calibration_failed": "Calibration failed for {direction}; see technical details",
        "intr.not_enough_usable": "Not enough usable samples: {usable}/{minimum}; {quarantined} quarantined",
        "intr.sample_quarantined": "Invalid checkerboard geometry detected; sample quarantined ({reason})",
        "intr.samples_cleared": "Current samples cleared for {direction}; saved calibration retained",
        "intr.saved": "Saved intrinsics for {direction}",
        "intr.preview_balance": "Preview scale for {direction} set to {balance}; calibration files are unchanged",
        "intr.usable_samples": "Usable samples",
        "intr.captured": "Captured",
        "intr.target": "target",
        "intr.minimum": "minimum",
        "intr.quarantined": "quarantined",
        "reason.invalid_shape": "invalid corner count or shape",
        "reason.nonfinite": "non-finite corner coordinates",
        "reason.out_of_bounds": "corners outside the image",
        "reason.collapsed_edge": "overlapping adjacent corners",
        "reason.collapsed_cell": "collapsed checkerboard cell",
        "reason.folded_grid": "folded checkerboard ordering",
        "reason.opencv_ill_conditioned": "ill-conditioned OpenCV view",
        "reason.no_frame": "No camera frame received",
        "reason.board_not_found": "Full checkerboard not detected",
        "reason.partial_board": "Only part of the checkerboard was detected",
        "reason.intrinsics_missing": "Saved intrinsics are missing for this camera",
        "reason.out_of_view": "Corners are too close to the image edge",
        "reason.seam_edge_valid": "Complete checkerboard detected near the edge",
        "reason.seam_raw_edge": "Checkerboard is too close to the raw image edge",
        "reason.seam_nonfinite": "Undistorted coordinates are invalid",
        "reason.seam_outside_bev": "Checkerboard is outside the valid BEV area",
        "reason.seam_too_small": "Checkerboard span is too small in BEV",
        "reason.moving": "Checkerboard is still moving",
        "reason.stable": "Checkerboard stable; confirming",
        "reason.ready": "Checkerboard stable; burst ready",
        "reason.bursting": "Capturing burst frames",
        "reason.candidate": "Candidate generated",
        "reason.duplicate_frame": "Waiting for a new camera frame",
        "reason.burst_timeout": "Timed out waiting for a new frame",
    },
}


def render_nav(active: str) -> str:
    """统一导航条（含语言切换）。active 与 I18N key 对应：home/intr/ext/ext_multi/seam/bev。"""
    links = (
        ("home",     "/",              "home"),
        ("intr",     "/intrinsics",    "intr"),
        ("ext",      "/extrinsics",    "ext"),
        ("ext_multi","/extrinsics/multi", "ext_multi"),
        ("seam",     "/seam",          "seam"),
        ("bev",      "/bev",           "bev"),
    )
    items = "".join(
        f'<a href="{href}" class="{"active" if active==k else ""}" '
        f'data-i18n="nav.{ik}">{label_zh}</a>'
        for label_zh, (k, href, ik) in zip(
            ("首页", "内参", "外参", "多点外参", "接缝", "BEV 预览"), links
        )
    )
    return (
        f'<nav class="nav">{items}'
        f'<span class="spacer"></span>'
        f'<button id="langToggle" class="lang-btn" onclick="toggleLang()" '
        f'title="Switch language">EN</button>'
        f'</nav>'
    )


_I18N_JS = r"""
<script>
(function(){
  const I18N = __I18N_JSON_PLACEHOLDER__;
  // 元素级翻译（data-i18n="key"）：使用 I18N[lang][key]
  // 整段内置中文（data-i18n-zh / data-i18n-en）：直接覆盖 textContent
  const KEY = 'calib_lang';
  function cur(){
    const s = localStorage.getItem(KEY);
    if (s === 'zh' || s === 'en') return s;
    return (navigator.language || 'zh').toLowerCase().startsWith('zh') ? 'zh' : 'en';
  }
  function format(value, params){
    return String(value == null ? '' : value).replace(/\{([A-Za-z0-9_]+)\}/g,
      (_, key) => params && params[key] !== undefined ? String(params[key]) : '{' + key + '}');
  }
  function t(key, params){
    const lang = cur();
    const table = I18N[lang] || I18N.zh;
    return format(table[key] !== undefined ? table[key] : key, params || {});
  }
  function tr(zh, en){ return cur() === 'en' ? en : zh; }
  function reason(code, fallback){
    const key = 'reason.' + String(code || '');
    const table = I18N[cur()] || {};
    return table[key] !== undefined ? format(table[key], {}) : (fallback || code || '');
  }
  function renderMessage(target, message, raw, fallbackKey){
    const el = typeof target === 'string' ? document.getElementById(target) : target;
    if (!el) return;
    const lang = cur();
    const fallback = fallbackKey ? t(fallbackKey) : '';
    const summary = message && message.code ? t(message.code, message.params) :
      (lang === 'en' && /[\u3400-\u9fff]/.test(String(raw || '')) ? fallback : String(raw || fallback));
    el.replaceChildren();
    const line = document.createElement('span');
    line.textContent = summary;
    el.appendChild(line);
    const detail = message && message.detail ? String(message.detail) :
      (lang === 'en' && raw && summary !== String(raw) ? String(raw) : '');
    if (detail){
      const details = document.createElement('details');
      details.className = 'technical-details';
      const label = document.createElement('summary');
      label.textContent = t('common.technical_details');
      const pre = document.createElement('pre');
      pre.textContent = detail;
      details.append(label, pre);
      el.appendChild(details);
    }
  }
  function apply(lang){
    document.documentElement.lang = lang;
    document.querySelectorAll('[data-i18n]').forEach(el => {
      const k = el.dataset.i18n;
      const v = I18N[lang] && I18N[lang][k];
      if (v !== undefined) el.textContent = v;
    });
    document.querySelectorAll('[data-i18n-' + lang + ']').forEach(el => {
      const v = el.getAttribute('data-i18n-' + lang);
      if (v != null) el.textContent = v;
    });
    document.querySelectorAll('[data-i18n-title-' + lang + ']').forEach(el => {
      el.title = el.getAttribute('data-i18n-title-' + lang);
    });
    const btn = document.getElementById('langToggle');
    if (btn){
      btn.textContent = (I18N[lang] && I18N[lang]['lang.toggle']) || 'EN';
    }
    localStorage.setItem(KEY, lang);
    document.dispatchEvent(new CustomEvent('langchange', {detail:{lang}}));
  }
  window.toggleLang = () => apply(cur() === 'zh' ? 'en' : 'zh');
  window.applyLang = apply;
  window.langGet = cur;
  window.t = t;
  window.tr = tr;
  window.reasonText = reason;
  window.renderUiMessage = renderMessage;
  document.addEventListener('DOMContentLoaded', () => apply(cur()));
  if (document.readyState !== 'loading') apply(cur());
})();
</script>
"""


def _inject_common(html: str) -> str:
    """为单页 HTML 注入 BASE_CSS 与 i18n 脚本（在 head 末尾追加 style，在 </body> 前追加 script）。

    保持页面原有内联 style 不变；BASE_CSS 因为选择器更具特异性或更靠后会自然覆盖（同选择器则后定义胜出）。
    """
    i18n_json = json.dumps(I18N, ensure_ascii=False)
    script = _I18N_JS.replace("__I18N_JSON_PLACEHOLDER__", i18n_json)
    # 注入 BASE_CSS 至 head 末尾
    if "</head>" in html:
        html = html.replace(
            "</head>",
            f"<style>{BASE_CSS}</style>{script}</head>",
            1,
        )
    return html


# =========================================================================
#  HTML 页面
# =========================================================================

DASHBOARD_HTML = """<!doctype html><html lang="zh"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title data-i18n-zh="J501 标定台" data-i18n-en="J501 Calibration">J501 标定台</title>
<style>
*{box-sizing:border-box}body{font-family:system-ui,sans-serif;margin:0;padding:clamp(12px,3vw,32px);background:#10141b;color:#e8edf5}
h1{font-size:20px} .grid{display:flex;flex-wrap:wrap;gap:10px}
.tile{flex:1 1 420px;max-width:calc(50% - 10px);background:#1c212a;border:1px solid #2c3442;border-radius:12px;padding:8px}
.dev{font-weight:bold;color:#9cf;margin-bottom:4px}
img{width:100%;height:auto;background:#000;border-radius:8px;display:block}
.badge{display:inline-block;padding:2px 8px;border-radius:4px;font-size:12px;margin-left:6px}
.ok{background:#2a7d2a}.no{background:#7d2a2a}.warn{background:#7d5a2a}
.nav{display:flex;gap:10px;margin:14px 0}
.nav a{padding:10px 18px;background:#2a4d6a;color:#fff;border-radius:6px;text-decoration:none}
.nav a:hover{background:#356a8a}
#log{margin-top:10px;font-family:monospace;font-size:12px;color:#8f8;white-space:pre-wrap;min-height:20px}
@media(max-width:900px){.tile{max-width:100%}.nav{flex-wrap:wrap}}
</style></head><body>
<div class="nav" style="justify-content:flex-end;margin:0 0 12px"><button id="langToggle" class="lang-btn" onclick="toggleLang()" data-i18n-zh="EN" data-i18n-en="中文">EN</button></div>
<h1><span data-i18n-zh="J501 四路鱼眼标定" data-i18n-en="J501 4-Camera Fisheye Calibration">J501 四路鱼眼标定</span></h1>
<div class="nav">
  <a href="/intrinsics" data-i18n-zh="内参标定" data-i18n-en="Intrinsics">内参标定</a>
  <a href="/extrinsics" data-i18n-zh="外参标定" data-i18n-en="Extrinsics">外参标定</a>
  <a href="/extrinsics/multi" data-i18n-zh="多点外参" data-i18n-en="Multi-position">多点外参</a>
  <a href="/seam" data-i18n-zh="接缝诊断" data-i18n-en="Seam Diagnostics">接缝诊断</a>
  <a href="/bev" data-i18n-zh="BEV 俯视预览" data-i18n-en="BEV Preview">BEV 俯视预览</a>
</div>
<div class="grid" id="g"></div>
<div id="log"></div>
<script>
const dirs=['front','back','left','right'];
function mk(d){const t=document.createElement('div');t.className='tile';
 t.innerHTML=`<div class="dev">${d} <span id="s_${d}"></span></div><img src="/stream?dir=${d}">`;return t;}
dirs.forEach(d=>document.getElementById('g').appendChild(mk(d)));
async function poll(){try{const r=await fetch('/api/status');const s=await r.json();
 const lang=localStorage.getItem('calib_lang')||((navigator.language||'zh').startsWith('zh')?'zh':'en');
 const tIntrYes=lang==='en'?'Intrinsics ✓':'内参✓';
 const tIntrNo=lang==='en'?'Intrinsics ✗':'内参✗';
 const tExtYes=lang==='en'?'Extrinsics ✓':'外参✓';
 const tExtNo=lang==='en'?'Extrinsics ✗':'外参✗';
 dirs.forEach(d=>{const e=document.getElementById('s_'+d);if(!e)return;
   const i=s.intr_done[d]?`<span class="badge ok">${tIntrYes}</span>`:`<span class="badge no">${tIntrNo}</span>`;
   const x=s.ext_done[d]?`<span class="badge ok">${tExtYes}</span>`:`<span class="badge no">${tExtNo}</span>`;
   e.innerHTML=i+x;});
 renderUiMessage('log',null,s.last_log,'common.status_updated');}catch(e){renderUiMessage('log',null,e.message,'common.network_interrupted')}}
document.addEventListener('langchange',poll);setInterval(poll,1000);poll();
</script></body></html>"""


INTRINSICS_HTML = """<!doctype html><html lang="zh"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1"><title data-i18n-zh="内参标定" data-i18n-en="Intrinsics">内参标定</title><style>
*{box-sizing:border-box}body{font-family:system-ui,sans-serif;margin:0;background:#10141b;color:#e8edf5;padding:clamp(12px,3vw,28px)}
.nav{display:flex;gap:8px;margin:0 0 14px;flex-wrap:wrap;align-items:center}.nav a{padding:8px 14px;background:#2a4d6a;color:#fff;border-radius:6px;text-decoration:none}
h1{font-size:clamp(20px,2vw,28px);margin:0 0 14px}.layout{display:grid;grid-template-columns:minmax(0,1fr) clamp(360px,26vw,420px);gap:16px;align-items:start;width:100%}
.panel{background:#1b2430;border:1px solid #344254;border-radius:12px;padding:14px;box-shadow:0 8px 24px #0003}.video-panel{min-width:0}.side{width:auto;min-width:0}
#v{border-radius:8px;background:#000;width:100%;height:auto;max-height:calc(100vh - 170px);object-fit:contain;display:block}.preview-row,.actions{display:flex;gap:6px;flex-wrap:wrap;margin:10px 0}
button{padding:8px 12px;background:#2a7d2a;color:#fff;border:0;border-radius:6px;cursor:pointer}button:hover{background:#359a35}button:disabled{background:#555;cursor:not-allowed}.secondary{background:#2a4d6a}.secondary.active{background:#3978a7}.danger{background:#7d2a2a;margin-top:16px}.danger:hover{background:#a03939}
.bar{height:12px;background:#333;border-radius:6px;margin:3px 0 8px;overflow:hidden}.bar>i{display:block;height:100%;background:#2a9d2a}.metric{font:13px ui-monospace,monospace;line-height:1.5}.quality{display:inline-block;padding:2px 8px;border-radius:999px;font-size:12px}.pass{background:#247a45}.warn{background:#8a6422}.fail{background:#8a3232}.muted{font-size:12px;color:#aeb9c8;line-height:1.5}select{background:#12151a;color:#eee;border:1px solid #3a4;padding:6px;border-radius:5px}#msg{color:#9fe3a4;font:12px ui-monospace,monospace;min-height:46px;white-space:pre-wrap}
.lang-btn{margin-left:auto;padding:6px 10px;background:transparent;color:#aeb9c8;border:1px solid #46556a;border-radius:6px;cursor:pointer;font-size:12px;font-weight:600}
.lang-btn:hover{color:#fff;border-color:#3b82f6}
@media(max-width:900px){.layout{grid-template-columns:1fr}.side{width:auto}#v{max-height:none}}
</style></head><body>
<div class="nav"><a href="/" data-i18n-zh="首页" data-i18n-en="Home">首页</a><a href="/intrinsics" data-i18n-zh="内参" data-i18n-en="Intrinsics">内参</a><a href="/extrinsics" data-i18n-zh="外参" data-i18n-en="Extrinsics">外参</a><a href="/extrinsics/multi" data-i18n-zh="多点外参" data-i18n-en="Multi-position">多点外参</a><a href="/seam" data-i18n-zh="接缝" data-i18n-en="Seam">接缝</a><a href="/bev" data-i18n-zh="BEV" data-i18n-en="BEV">BEV</a><button id="langToggle" class="lang-btn" onclick="toggleLang()">EN</button></div>
<h1><span data-i18n-zh="内参标定 — " data-i18n-en="Intrinsics — ">内参标定 — </span><span id="boardSpec" data-i18n-zh="读取棋盘规格中" data-i18n-en="Loading board specification">读取棋盘规格中</span></h1>
<main class="layout"><section class="panel video-panel"><div id="videoState" class="muted" style="margin-bottom:7px" data-i18n-zh="正在连接 front" data-i18n-en="Connecting to front">正在连接 front</div><img id="v" alt="相机实时画面" data-i18n-title-zh="相机实时画面" data-i18n-title-en="Live camera image"><div class="preview-row"><button id="raw" class="secondary active" data-i18n-zh="原始画面" data-i18n-en="Raw">原始画面</button><button id="undist" class="secondary" disabled data-i18n-zh="矫正画面" data-i18n-en="Undistorted">矫正画面</button><label class="muted"><span data-i18n-zh="去畸变预览缩放" data-i18n-en="Undistort preview scale">去畸变预览缩放</span> <input id="previewBalance" type="range" min="0.50" max="1.50" step="0.05" value="0.80"> <b id="previewBalanceValue">0.80</b></label><span id="previewState" class="muted" data-i18n-zh="尚无可预览结果" data-i18n-en="No preview available">尚无可预览结果</span></div><div class="muted" data-i18n-zh="该滑杆仅改变当前矫正画面的视野与裁切：数值越大越放大、裁切越多。不会保存，也不会改变已标定的外参或 BEV 拼接。" data-i18n-en="This slider only changes the current undistorted preview's field of view: larger values zoom in more and crop more. It does not save anything and does not affect extrinsics or BEV.">该滑杆仅改变当前矫正画面的视野与裁切：数值越大越放大、裁切越多。不会保存，也不会改变已标定的外参或 BEV 拼接。</div></section>
<aside class="panel side"><label><span data-i18n-zh="方向：" data-i18n-en="Direction:">方向：</span><select id="dir"><option>front</option><option>back</option><option>left</option><option>right</option></select></label>
<div class="metric" style="margin-top:12px">X<div class="bar"><i id="b0" style="width:0"></i></div>Y<div class="bar"><i id="b1" style="width:0"></i></div>Size<div class="bar"><i id="b2" style="width:0"></i></div>Skew<div class="bar"><i id="b3" style="width:0"></i></div></div>
<div class="metric"><span data-i18n-zh="可用样本：" data-i18n-en="Usable samples:">可用样本：</span><b id="cnt">0</b><span id="sampleMeta"></span></div><div class="metric" id="metrics" data-i18n-zh="结果：—" data-i18n-en="Result: —">结果：—</div>
<div class="actions"><button id="collect" data-i18n-zh="开始采集" data-i18n-en="Start Capture">开始采集</button><button id="cal" data-i18n-zh="开始计算" data-i18n-en="Calibrate">开始计算</button><button id="save" disabled data-i18n-zh="保存结果" data-i18n-en="Save">保存结果</button></div><div id="msg"></div>
<div class="muted" data-i18n-zh="采集时请缓慢移动棋盘，覆盖画面边缘、中心、远近和倾斜角度。计算完成会自动进入候选矫正预览；满意后再保存。" data-i18n-en="Slowly move the chessboard to cover edges, center, near/far and tilt angles. After compute, an undistorted preview appears; save only when satisfied.">采集时请缓慢移动棋盘，覆盖画面边缘、中心、远近和倾斜角度。计算完成会自动进入候选矫正预览；满意后再保存。</div>
<button id="clr" class="danger" data-i18n-zh="清空本轮样本" data-i18n-en="Clear Samples">清空本轮样本</button></aside></main>
<script>
const v=document.getElementById('v'),dirEl=document.getElementById('dir'),videoState=document.getElementById('videoState'),previewBalance=document.getElementById('previewBalance'),previewBalanceValue=document.getElementById('previewBalanceValue');let viewMode='raw',lastStatus=null,shownDir='front',switchToken=0;const seenCandidate={};
function streamUrl(d){return '/stream?dir='+encodeURIComponent(d)+'&overlay='+(viewMode==='undist'?'undist':'intr')+'&t='+Date.now();}
function stopStream(){v.removeAttribute('src');v.style.visibility='hidden';}
function refreshStream(){stopStream();v.onload=()=>{v.style.visibility='visible'};v.src=streamUrl(shownDir);document.getElementById('raw').classList.toggle('active',viewMode==='raw');document.getElementById('undist').classList.toggle('active',viewMode==='undist');}
async function post(path,body){const r=await fetch(path,{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body});const j=await r.json();if(!r.ok&&!j.error&&!j.msg)j.error='HTTP '+r.status;return j;}
function showApiMessage(j){renderUiMessage('msg',j.message,j.error||j.msg,j.ok?null:'common.operation_failed');}
async function selectDir(d){const token=++switchToken,previous=shownDir;dirEl.disabled=true;stopStream();videoState.textContent=tr('正在切换到 ','Switching to ')+d+'…';try{const j=await post('/api/intrinsics/select','dir='+encodeURIComponent(d));if(!j.ok)throw Object.assign(new Error(j.error||j.msg||tr('切换失败','Switch failed')),{payload:j});if(token!==switchToken)return;shownDir=j.active_dir||d;dirEl.value=shownDir;viewMode='raw';videoState.textContent=tr('当前画面：','Live camera: ')+shownDir+' · '+(j.device||'');showApiMessage(j);refreshStream();}catch(e){if(token!==switchToken)return;shownDir=previous;dirEl.value=previous;videoState.textContent=tr('切换失败，仍显示 ','Switch failed; still showing ')+previous;showApiMessage(e.payload||{ok:false,error:e.message});refreshStream();}finally{if(token===switchToken)dirEl.disabled=false;}}
dirEl.onchange=e=>selectDir(e.target.value);document.getElementById('raw').onclick=()=>{viewMode='raw';refreshStream()};document.getElementById('undist').onclick=()=>{viewMode='undist';refreshStream()};previewBalance.oninput=()=>{previewBalanceValue.textContent=Number(previewBalance.value).toFixed(2)};previewBalance.onchange=async()=>{const d=dirEl.value,j=await post('/api/intrinsics/preview-balance','dir='+encodeURIComponent(d)+'&balance='+encodeURIComponent(previewBalance.value));showApiMessage(j);if(j.ok){viewMode='undist';refreshStream();}};
document.getElementById('collect').onclick=async()=>{const d=dirEl.value,on=document.getElementById('collect').dataset.on==='1',j=await post('/api/intrinsics/collect','dir='+d+'&enabled='+(on?'0':'1'));showApiMessage(j);};
document.getElementById('cal').onclick=async()=>{const d=dirEl.value,j=await post('/api/intrinsics/calibrate','dir='+d);showApiMessage(j);};
document.getElementById('save').onclick=async()=>{const d=dirEl.value;let j=await post('/api/intrinsics/save','dir='+d);if(!j.ok&&String(j.msg||'').includes('warn')&&confirm(tr('质量评估为警告，仍要保存吗？','Quality has warnings. Save anyway?')))j=await post('/api/intrinsics/save','dir='+d+'&confirm=1');showApiMessage(j);};
document.getElementById('clr').onclick=async()=>{const d=dirEl.value,stats=lastStatus?.intr_sample_stats?.[d]||{},n=stats.captured||0,has=lastStatus?.intr_candidate?.[d];if(!confirm(tr('确认清空 '+d+' 的 '+n+' 张采集记录'+(has?' 和未保存候选结果':'')+'？正式保存的结果不会删除。','Clear '+n+' captured records for '+d+(has?' and its unsaved candidate':'')+'? Saved calibration will be retained.')))return;const j=await post('/api/intrinsics/clear','dir='+d+'&confirm=1');showApiMessage(j);if(j.ok){viewMode='raw';refreshStream();}};
function qualityText(q){return q==='pass'?tr('通过','Pass'):q==='warn'?tr('警告','Warning'):q==='fail'?tr('不合格','Failed'):'—'}
function renderStatus(s){lastStatus=s;const d=dirEl.value;if(s.board)document.getElementById('boardSpec').textContent=`fisheye (${s.board.cols}×${s.board.rows}, ${(s.board.square_m*1000).toFixed(0)} mm)`;const pb=s.intr_preview_balance?.[d];if(pb!==undefined&&document.activeElement!==previewBalance){previewBalance.value=pb;previewBalanceValue.textContent=Number(pb).toFixed(2);}const p=s.intr_progress[d]||[0,0,0,0];for(let i=0;i<4;i++)document.getElementById('b'+i).style.width=(p[i]*100)+'%';const stats=s.intr_sample_stats?.[d]||{captured:0,usable:0,quarantined:0,target:25,minimum:15};document.getElementById('cnt').textContent=stats.usable||0;document.getElementById('sampleMeta').textContent=' · '+t('intr.captured')+' '+(stats.captured||0)+' · '+t('intr.target')+' '+(stats.target||25)+' · '+t('intr.minimum')+' '+(stats.minimum||15)+' · '+t('intr.quarantined')+' '+(stats.quarantined||0);const collecting=!!s.intr_collecting[d],candidate=!!s.intr_candidate[d],available=!!s.intr_preview_available[d],q=s.intr_quality[d],m=s.intr_metrics[d]||{};document.getElementById('collect').dataset.on=collecting?'1':'0';document.getElementById('collect').textContent=collecting?tr('暂停采集','Pause Capture'):tr('开始采集','Start Capture');document.getElementById('cal').disabled=!s.intr_can_calibrate[d]||s.intr_task_running;document.getElementById('save').disabled=s.intr_task_running||!candidate||q==='fail';document.getElementById('clr').disabled=s.intr_task_running;document.getElementById('undist').disabled=!available;document.getElementById('metrics').innerHTML=tr('结果：','Result: ')+(q?`<span class="quality ${q}">${qualityText(q)}</span>`:'—')+(m.rms!==null&&m.rms!==undefined?' · RMS '+m.rms+' px':'')+(m.sample_count?' · '+m.sample_count+' '+tr('张用于结果','images in result'):'');document.getElementById('previewState').textContent=available?(candidate?tr('候选参数预览（尚未保存）','Candidate preview (unsaved)'):tr('正式参数预览','Saved preview')):tr('尚无可预览结果','No preview available');if(candidate&&!seenCandidate[d]){viewMode='undist';refreshStream()}seenCandidate[d]=candidate;if(!available&&viewMode==='undist'){viewMode='raw';refreshStream()}renderUiMessage('msg',s.ui_messages?.intrinsics,s.intr_task_msg||s.last_log);}
async function poll(){try{const r=await fetch('/api/status');renderStatus(await r.json());}catch(e){renderUiMessage('msg',null,e.message,'common.network_interrupted');}}
document.addEventListener('langchange',()=>{if(lastStatus)renderStatus(lastStatus)});setInterval(poll,800);selectDir('front');poll();
</script></body></html>"""


EXTRINSICS_HTML = """<!doctype html><html lang="zh"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title data-i18n-zh="外参标定" data-i18n-en="Extrinsics">外参标定</title><style>
.saved-chip.invalid{border-left:3px solid #df5a57;color:#ffd1c9}
*{box-sizing:border-box}body{font-family:system-ui,sans-serif;margin:0;padding:clamp(12px,2vw,24px);background:#10141b;color:#e8edf5}.nav{display:flex;gap:8px;margin:0 0 14px;flex-wrap:wrap;align-items:center}.nav a{padding:8px 14px;background:#2a4d6a;color:#fff;border-radius:6px;text-decoration:none}.layout{display:grid;grid-template-columns:minmax(0,1fr) 340px;gap:16px;align-items:start}.panel{background:#1c212a;border:1px solid #344255;border-radius:12px;padding:14px}.video{width:100%;max-height:calc(100vh - 130px);object-fit:contain;background:#000;border-radius:8px;display:block}.camera-alert{min-height:280px;padding:32px 18px;display:grid;place-items:center;text-align:center;background:#151a21;border:1px dashed #a65c4e;border-radius:8px;color:#ffd1c9}.control{position:sticky;top:12px}.step{border-left:4px solid #596879;background:#171d26;border-radius:6px;padding:9px 10px;margin:9px 0}.step.active{border-left-color:#2fbe48}.step.warn{border-left-color:#e9ad3d}.step.bad{border-left-color:#df5a57}.next-step{border-left-color:#35bd4f;background:#14271a}.next-step h2{margin:0 0 5px;font-size:17px;color:#b9f7c1}.next-step button{width:100%;margin:5px 0;text-align:left}.small{font-size:12px;color:#b8c3d1;line-height:1.5}.value{font-family:ui-monospace,monospace;font-size:13px}.badge{display:inline-block;padding:2px 8px;border-radius:999px;font-size:12px;margin-left:6px}.ok{background:#1e742e}.warn{background:#7b5719}.bad{background:#8b302f}.muted{color:#9ba8b9}button{padding:9px 12px;margin:3px 2px;background:#287f32;color:#fff;border:0;border-radius:7px;cursor:pointer;font-weight:650}button:hover{background:#35a143}button:disabled{opacity:.45;cursor:not-allowed}.secondary{background:#38506b}.danger{background:#9f3a36}select,input{background:#121820;color:#eee;border:1px solid #527045;border-radius:5px;padding:6px}.saved-grid{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:6px;margin-top:7px}.saved-chip{font-size:12px;border-radius:6px;padding:5px 6px;background:#202934;color:#b8c3d1}.saved-chip.active{border-left:3px solid #34b84b;color:#b9f7c1}.saved-chip.stale{border-left:3px solid #e9ad3d;color:#ffdf99}.saved-chip.missing{border-left:3px solid #637181}.gallery{display:grid;grid-template-columns:repeat(4,minmax(0,1fr));gap:8px;margin-top:10px}.shot{min-width:0;border:1px solid #3a4656;border-radius:7px;overflow:hidden;background:#111820}.shot.bad{border-color:#a54a48}.shot img{width:100%;display:block;background:#000}.shot div{font-size:11px;padding:4px;color:#cbd4df;white-space:nowrap;overflow:hidden;text-overflow:ellipsis}#msg{margin-top:9px;color:#9fe8a8;font-family:ui-monospace,monospace;font-size:12px;line-height:1.45}.quality{padding:2px 7px;border-radius:9px}.quality.ok{background:#1e742e}.quality.warn{background:#7b5719}.quality.bad{background:#8b302f}.lang-btn{margin-left:auto;padding:6px 10px;background:transparent;color:#b8c3d1;border:1px solid #46556a;border-radius:6px;cursor:pointer;font-size:12px;font-weight:600}.lang-btn:hover{color:#fff;border-color:#3b82f6}@media(max-width:900px){.layout{grid-template-columns:1fr}.control{position:static}.video{max-height:none}.gallery{grid-template-columns:repeat(2,minmax(0,1fr))}}</style></head><body>
<div class="nav"><a href="/" data-i18n-zh="首页" data-i18n-en="Home">首页</a><a href="/intrinsics" data-i18n-zh="内参" data-i18n-en="Intrinsics">内参</a><a href="/extrinsics" data-i18n-zh="外参" data-i18n-en="Extrinsics">外参</a><a href="/seam" data-i18n-zh="接缝" data-i18n-en="Seam">接缝</a><a href="/bev" data-i18n-zh="BEV" data-i18n-en="BEV">BEV</a><button id="langToggle" class="lang-btn" onclick="toggleLang()">EN</button></div>
<h1 style="margin:0 0 14px" data-i18n-zh="外参标定：看见角点，再保存结果" data-i18n-en="Extrinsics: See corners, then save">外参标定：看见角点，再保存结果</h1>
<div class="layout"><main><section class="panel"><div style="display:flex;justify-content:space-between;gap:8px;align-items:center;margin-bottom:9px"><b data-i18n-zh="实时检测画面" data-i18n-en="Live Detection">实时检测画面</b><label><b data-i18n-zh="标定方向" data-i18n-en="Direction">标定方向</b> <select id="dir"><option>front</option><option>back</option><option>left</option><option>right</option></select></label></div><img class="video" id="v"><div class="camera-alert" id="cameraAlert" hidden></div><div class="small" style="margin-top:8px" data-i18n-zh="选择方向会同时切换标定目标和检测相机。绿色角点＝可用；黄色＝正在等稳定；红色＝本帧不能用于标定。" data-i18n-en="Choosing a direction switches both the target and the camera. Green corners = usable; yellow = waiting for stability; red = not usable for this frame.">选择方向会同时切换标定目标和检测相机。绿色角点＝可用；黄色＝正在等稳定；红色＝本帧不能用于标定。</div></section><section class="panel" style="margin-top:14px"><b data-i18n-zh="自动连拍证据（8 张）" data-i18n-en="Burst evidence (8 frames)">自动连拍证据（8 张）</b><span class="small" data-i18n-zh="每张都是程序实际拍到并检测过的画面" data-i18n-en="Each frame is what the program actually captured and detected">每张都是程序实际拍到并检测过的画面</span><div class="gallery" id="gallery"><span class="muted small" data-i18n-zh="等待自动连拍…" data-i18n-en="Waiting for burst…">等待自动连拍…</span></div></section></main>
<aside class="panel control"><div class="step next-step" id="nextStep" hidden><h2 data-i18n-zh="✅ 四路外参已完成" data-i18n-en="All 4 extrinsics done">✅ 四路外参已完成</h2><div id="nextHint" class="small"></div><button id="seamGo" class="secondary" onclick="goSeam()" data-i18n-zh="下一步：进入接缝诊断页（推荐）" data-i18n-en="Next: Seam (recommended)">下一步：进入接缝诊断页（推荐）</button><button onclick="goBev()" data-i18n-zh="跳过诊断，直接进 BEV →" data-i18n-en="Skip, go BEV →">跳过诊断，直接进 BEV →</button><div class="small" data-i18n-zh="接缝诊断会同时显示相邻两路并测量局部误差，但绝不会修改正式外参。" data-i18n-en="Seam diagnostics displays both adjacent cameras and measures local error, never modifying extrinsics.">接缝诊断会同时显示相邻两路并测量局部误差，但绝不会修改正式外参。</div></div><div class="step" id="savedState"><b data-i18n-zh="正式外参保存状态" data-i18n-en="Saved extrinsics status">正式外参保存状态</b><div id="saved" class="saved-grid"><span data-i18n-zh="读取中…" data-i18n-en="Loading…">读取中…</span></div><div id="savedNote" class="small" style="margin-top:6px"></div></div><div class="step" id="step1"><b>1. <span data-i18n-zh="先填写本轮摆位" data-i18n-en="Set placement">先填写本轮摆位</span></b><div id="placement" class="small"><span data-i18n-zh="等待选择方向" data-i18n-en="Waiting for direction">等待选择方向</span></div><div style="margin-top:6px"><span data-i18n-zh="近边距离(m)" data-i18n-en="Near edge (m)">近边距离(m)</span> <input id="nearInput" type="number" min="0.05" max="10" step="0.01" style="width:72px"> <span data-i18n-zh="横向偏移(m)" data-i18n-en="Lateral (m)">横向偏移(m)</span> <input id="lateralInput" type="number" min="-5" max="5" step="0.01" style="width:68px"></div><button onclick="applyPlacement()" data-i18n-zh="确认本次摆位" data-i18n-en="Apply Placement">确认本次摆位</button><div class="small" data-i18n-zh="近边距离是棋盘物理近边到车体中心的距离；右为正、左为负。" data-i18n-en="Near edge is the distance from the board's near edge to vehicle center; +right / -left.">近边距离是棋盘物理近边到车体中心的距离；右为正、左为负。</div></div><div class="step" id="step2"><b>2. <span data-i18n-zh="稳定检测" data-i18n-en="Stable Detection">稳定检测</span></b><div id="live" class="value"><span data-i18n-zh="尚未开始" data-i18n-en="Not started">尚未开始</span></div></div><div class="step" id="step3"><b>3. <span data-i18n-zh="自动连拍" data-i18n-en="Burst Capture">自动连拍</span></b><div id="burst" class="value"><span data-i18n-zh="等待稳定检测" data-i18n-en="Waiting for stable detection">等待稳定检测</span></div></div><div class="step" id="step4"><b>4. <span data-i18n-zh="审核并保存" data-i18n-en="Review & Save">审核并保存</span></b><div id="candidate" class="small"><span data-i18n-zh="尚无候选结果" data-i18n-en="No candidate yet">尚无候选结果</span></div></div><div style="margin-top:10px"><button onclick="startDetection()" data-i18n-zh="开始自动检测" data-i18n-en="Start Detection">开始自动检测</button><button class="secondary" onclick="act('stop')" data-i18n-zh="暂停" data-i18n-en="Pause">暂停</button><button id="saveExt" onclick="saveExt()" disabled data-i18n-zh="保存候选" data-i18n-en="Save Candidate">保存候选</button><button class="danger" onclick="relock()" data-i18n-zh="重新检测当前方向" data-i18n-en="Re-detect">重新检测当前方向</button><button class="secondary" onclick="act('skip')" data-i18n-zh="跳过当前方向" data-i18n-en="Skip">跳过当前方向</button></div><div style="margin-top:10px;border-top:1px solid #344255;padding-top:9px"><b data-i18n-zh="多点验证" data-i18n-en="Multi-point Verify">多点验证</b><div class="small" data-i18n-zh="保存后，把棋盘移到新位置核验误差。" data-i18n-en="After saving, move the board to a new position to verify error.">保存后，把棋盘移到新位置核验误差。</div><div style="margin-top:5px"><span data-i18n-zh="方向" data-i18n-en="Dir">方向</span> <input id="vd" value="front" style="width:52px"> <span data-i18n-zh="距离(m)" data-i18n-en="Dist (m)">距离(m)</span> <input id="vn" value="0.5" style="width:58px"> <span data-i18n-zh="横移(m)" data-i18n-en="Lat (m)">横移(m)</span> <input id="vl" value="0" style="width:50px"></div><button onclick="verify()" data-i18n-zh="开始多点验证" data-i18n-en="Start Verify">开始多点验证</button></div><div id="locked" class="small" style="margin-top:8px"></div><div id="msg"></div></aside></div>
<script>
const v=document.getElementById('v'),dir=document.getElementById('dir'),nearInput=document.getElementById('nearInput'),lateralInput=document.getElementById('lateralInput'),cameraAlert=document.getElementById('cameraAlert');let lastGallery='',placementTarget=null,lastStatus=null,shownDir=null;
function showVideo(d,cam){shownDir=d;if(cam&&!cam.available){v.removeAttribute('src');v.style.display='none';cameraAlert.hidden=false;cameraAlert.innerHTML=`<div><b>${tr(d+' 相机当前没有可用画面','Camera '+d+' has no live image')}</b><br><span class="small">${tr('预期设备：'+cam.device+'。已隐藏视频，避免误把上一方向的画面用于标定。请先恢复该路相机。','Expected device: '+cam.device+'. The video is hidden to prevent calibrating with the previous camera image. Restore this camera first.')}</span></div>`;return;}cameraAlert.hidden=true;v.style.display='block';if(v.dataset.dir!==d){v.dataset.dir=d;v.src='/stream?dir='+encodeURIComponent(d)+'&overlay=ext';}}
function renderSaved(s){const L=langGet()==='en'?{active:'Saved (current)',invalid:'Needs recalibration',stale:'Saved but stale',missing:'— Unsaved',pending:'Staged this run'}:{active:'✓ 已保存（当前可用）',invalid:'❌ 几何异常，需重标',stale:'⚠ 已保存但已过期',missing:'— 未保存',pending:'本轮已暂存'};const saved=s.ext_saved||{},pending=s.ext_pending||{},label=L;document.getElementById('saved').innerHTML=['front','back','left','right'].map(d=>{const x=pending[d]?{...pending[d],state:'pending'}:(saved[d]||{state:'missing'}),r=x.rms!==undefined?` · ${Number(x.rms).toFixed(3)}px`:'';return `<div class="saved-chip ${x.state||'missing'}"><b>${d}</b><br>${label[x.state]||label.missing}${x.state!=='missing'?r:''}</div>`}).join('');document.getElementById('savedNote').textContent=Object.values(pending).some(Boolean)?(langGet()==='en'?'Blue staged results do not replace formal extrinsics until all four cameras are ready.':'本轮暂存结果不会改写正式外参；四路全部完成后才会原子提交。'):Object.values(saved).some(x=>x.state==='invalid')?(langGet()==='en'?'The legacy pose-rebuilt geometry points inward and is blocked. Recalibrate all four cameras.':'旧结果由异常朝内位姿重建，已禁止用于 BEV；请依次重新标定四路。'):s.ext_results_stale?(langGet()==='en'?'Yellow "stale" means this direction was saved but intrinsics were updated later; the original file is kept but cannot be used for current BEV — recalibrate this direction.':'黄色“已过期”表示该方向曾保存，但内参后来更新；原文件仍保留，不能用于当前 BEV，需重新标定该路。'):(langGet()==='en'?'Green results are usable for the current calibration. Once all four are green, enter the seam page, then BEV.':'绿色结果可用于当前标定；四路均为绿色后进入独立接缝页，再进 BEV。');}
dir.onchange=async()=>{const wanted=dir.value;showVideo(wanted,null);const j=await post('/api/extrinsics/target','dir='+wanted);if(!j.ok){renderUiMessage('msg',null,j.error||j.msg||tr('无法切换方向','Unable to switch direction'),'common.operation_failed');dir.value=lastStatus?.target||'front';showVideo(dir.value,lastStatus?.camera_status?.[dir.value]);return;}placementTarget=null;renderUiMessage('msg',null,j.msg,'common.status_updated');showVideo(wanted,lastStatus?.camera_status?.[wanted]);};
fetch('/api/extrinsics/start',{method:'POST'});
async function post(url,body){const r=await fetch(url,{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body});return r.json();}
async function act(c){const j=await post('/api/extrinsics/cmd','cmd='+c);renderUiMessage('msg',null,j.error||j.msg,j.ok?'common.status_updated':'common.operation_failed');}
function goSeam(){window.location.assign('/seam');}
function goBev(){window.location.assign('/bev');}
function renderNext(s){const allDone=['front','back','left','right'].every(d=>!!s.ext_done?.[d]);const lang=langGet();const box=document.getElementById('nextStep');box.hidden=!allDone;if(!allDone)return;const btn=document.getElementById('seamGo'),pair=s.seam_pair;if(s.seam_complete){document.getElementById('nextHint').textContent=lang==='en'?'Seam done. Extrinsics unchanged. Review or proceed to BEV.':'接缝诊断已完成，正式 H 未修改。可复查或进入 BEV。';btn.disabled=false;btn.textContent=lang==='en'?'Open Seam Page':'打开接缝诊断页';}else if(s.seam_mode&&pair){document.getElementById('nextHint').textContent=lang==='en'?`Seam in progress: now ${pair[0]} + ${pair[1]}.`:`接缝诊断进行中：当前是 ${pair[0]} + ${pair[1]}。`;btn.disabled=false;btn.textContent=lang==='en'?'Open Seam Page':'打开接缝诊断页';}else{document.getElementById('nextHint').textContent=lang==='en'?'All 4 saved. Enter seam page to measure adjacency, or jump to BEV.':'四路已保存。可进入接缝页只读测量相邻误差，或直接检查 BEV。';btn.disabled=false;btn.textContent=lang==='en'?'Next: Seam (recommended)':'下一步：进入接缝诊断页（推荐）';}}
async function applyPlacement(){const d=lastStatus?.target;if(!d){renderUiMessage('msg',null,tr('尚无待标定方向','No calibration target selected'),'common.operation_failed');return false;}const j=await post('/api/extrinsics/placement',`dir=${d}&near=${encodeURIComponent(nearInput.value)}&lateral=${encodeURIComponent(lateralInput.value)}`);renderUiMessage('msg',null,j.error||j.msg,j.ok?'common.status_updated':'common.operation_failed');return !!j.ok;}
async function startDetection(){if(await applyPlacement())await act('start');}
async function relock(){if(!confirm(langGet()==='en'?'Re-detection will clear this direction\\'s unsaved candidate and the 8 burst frames; saved extrinsics are not deleted. Continue?':'重新检测会清除当前方向尚未保存的候选和本轮 8 张连拍证据；正式保存的外参不会删除。继续吗？'))return;await act('relock');}
async function saveExt(){const s=await fetch('/api/status').then(r=>r.json()),d=s.target||'front';let j=await post('/api/extrinsics/save','dir='+d);if(!j.ok&&String(j.msg||'').includes('warn')&&confirm(tr('候选有警告，仍要保存吗？','Candidate has warnings. Save anyway?')))j=await post('/api/extrinsics/save','dir='+d+'&confirm=1');renderUiMessage('msg',null,j.error||j.msg,j.ok?'common.status_updated':'common.operation_failed');}
async function verify(){const d=document.getElementById('vd').value,n=document.getElementById('vn').value,l=document.getElementById('vl').value;const j=await post('/api/extrinsics/verify',`dir=${d}&near=${n}&lateral=${l}`);renderUiMessage('msg',null,j.error||j.msg,j.ok?'common.status_updated':'common.operation_failed');}
function cls(el,name){el.className='step '+name;}function quality(q){return q==='ok'?(langGet()==='en'?'Pass':'通过'):q==='warn'?(langGet()==='en'?'Warning':'警告'):(langGet()==='en'?'Failed':'不合格');}
function gallery(d,b){const lang=langGet(),shots=b.evidence||[],sig=[lang,d,b.expected,b.accepted,b.rejected,b.phase,...shots.map(x=>x.frame_id+':'+x.accepted+':'+x.reason)].join(':');if(sig===lastGallery)return;lastGallery=sig;const root=document.getElementById('gallery');if(!b.expected){root.innerHTML='<span class="muted small">'+tr('棋盘稳定后会自动拍 8 张；每张都会显示检测到的角点。','Once the board is stable, 8 frames are captured automatically; each shows detected corners.')+'</span>';return;}let h='';for(let i=0;i<b.expected;i++){const shot=shots.find(x=>x.slot===i),valid=!!shot?.accepted,label=shot?reasonText(shot.reason,shot.reason_text||tr('未通过检测','Not detected')):tr('等待拍摄','Waiting');h+=`<div class="shot ${valid?'':'bad'}">${shot?`<img src="/api/extrinsics/burst-frame?dir=${d}&slot=${i}&t=${shot.frame_id}" onerror="this.style.display='none'">`:''}<div>${tr('第','Frame')} ${i+1} · ${valid?tr('完整角点','Full corners'):label}</div></div>`;}root.innerHTML=h;}
async function poll(){try{const s=await fetch('/api/status').then(r=>r.json()),target=s.target||'front',p=s.placements?.[target],live=(s.ext_live||{})[target]||{},b=(s.ext_burst||{})[target]||{},c=(s.ext_candidate_detail||{})[target];lastStatus=s;renderSaved(s);renderNext(s);if(dir.value!==target)dir.value=target;if(shownDir!==target||((s.camera_status||{})[target]&&!((s.camera_status||{})[target].available)))showVideo(target,(s.camera_status||{})[target]);const near=(p&&p.near_m!==undefined)?p.near_m:0.35,lat=(p&&p.lateral_m!==undefined)?p.lateral_m:0;if(placementTarget!==target){placementTarget=target;nearInput.value=near;lateralInput.value=lat;}document.getElementById('placement').textContent=tr(`当前标定方向：${target}；本轮按下方的近边距离和横向偏移计算。`,`Calibration direction: ${target}; this run uses the near-edge distance and lateral offset below.`);const running=s.ext_running?tr('正在检测','Detecting'):tr('已暂停','Paused'),hint=live.detected_pattern?tr(` · 当前棋盘约 ${live.detected_pattern[0]}×${live.detected_pattern[1]} 内角点（要求 ${live.expected||48} 点）`,` · detected about ${live.detected_pattern[0]}×${live.detected_pattern[1]} inner corners (${live.expected||48} required)`):'';document.getElementById('live').textContent=`${running} · ${reasonText(live.reason,tr('等待画面','Waiting for image'))}${hint} · ${tr('角点','corners')} ${live.corners_detected||0}/${live.expected||48}${live.motion_rms_px!==null&&live.motion_rms_px!==undefined?' · '+tr('位移','motion')+' '+live.motion_rms_px+' px':''} · ${tr('稳定','stable')} ${s.streak||0}/${s.ext_stable_required||10}`;cls(document.getElementById('step2'),live.phase==='failed'?'bad':(s.ext_running?'active':''));document.getElementById('burst').textContent=b.expected?`${b.phase||tr('连拍','burst')} · ${tr('有效','accepted')} ${b.accepted||0}/${b.expected} · ${tr('淘汰','rejected')} ${b.rejected||0}`:tr('等待棋盘稳定后自动拍 8 张','Waiting for a stable board before the automatic 8-frame burst');cls(document.getElementById('step3'),s.bursting?'warn':(b.expected?'active':''));if(c){const reasons=[...(c.failures||[]),...(c.warnings||[])],technical=reasons.length?(langGet()==='en'?`<details class="technical-details"><summary>${t('common.technical_details')}</summary><pre>${reasons.join(String.fromCharCode(10))}</pre></details>`:'<br><span class="small">'+reasons.join('；')+'</span>'):'';document.getElementById('candidate').innerHTML=`${tr('结果：','Result: ')}<span class="quality ${c.quality}">${quality(c.quality)}</span> · RMS ${c.rms} px · ${c.n_used||0} ${tr('张有效','valid frames')}${technical}`;cls(document.getElementById('step4'),c.quality==='bad'?'bad':(c.quality==='warn'?'warn':'active'));}else{document.getElementById('candidate').textContent=tr('尚无候选结果','No candidate yet');cls(document.getElementById('step4'),'');}document.getElementById('saveExt').disabled=!c||c.quality==='bad';document.getElementById('locked').textContent=tr(`本轮标定目标：${target}；请以顶部“正式外参保存状态”为准。`,`Current target: ${target}. Use the saved-extrinsics status above as the source of truth.`);renderUiMessage('msg',null,s.ext_task_msg||s.last_log,s.ext_task_running?'common.status_updated':null);gallery(target,b);}catch(e){renderUiMessage('msg',null,e.message,'common.network_interrupted');}}
document.addEventListener('langchange',()=>{lastGallery='';poll()});setInterval(poll,700);poll();
</script></body></html>"""


MULTI_EXTRINSICS_HTML = """<!doctype html><html lang="zh"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1"><title data-i18n-zh="多位置外参标定" data-i18n-en="Multi-position Extrinsics">多位置外参标定</title><style>
*{box-sizing:border-box}body{font-family:system-ui,sans-serif;margin:0;padding:20px;background:#10141b;color:#e8edf5}.nav,.actions,.dir-tabs{display:flex;gap:8px;flex-wrap:wrap;margin-bottom:12px;align-items:center}.nav a{padding:8px 12px;color:#cfe4ff;border:1px solid #344255;border-radius:6px;text-decoration:none}.layout{display:grid;grid-template-columns:minmax(0,1fr) 380px;gap:16px}.panel{background:#1c212a;border:1px solid #344255;border-radius:8px;padding:14px}.video{width:100%;max-height:58vh;object-fit:contain;background:#000;border-radius:6px}.position{display:grid;grid-template-columns:1fr auto;gap:6px;padding:9px;border-bottom:1px solid #303a48}.position.selected{background:#233349}.done{color:#7be38c}.bad{color:#ff8888}.muted{color:#aab5c3;font-size:12px}.metrics{font:12px ui-monospace,monospace;line-height:1.55;white-space:pre-wrap}button,select,input{padding:8px;border-radius:6px;border:1px solid #465568;background:#151b24;color:#fff}button{background:#287d3a;cursor:pointer}button.secondary{background:#365b7d}button.danger{background:#8a3636}button:disabled{opacity:.45;cursor:not-allowed}.evidence{display:flex;gap:5px;overflow:auto;margin-top:8px}.evidence img{height:72px;border-radius:4px}.quality{display:grid;grid-template-columns:70px 1fr;gap:6px;margin-top:8px}#msg{min-height:42px;color:#8ee49d;font:12px ui-monospace,monospace;white-space:pre-wrap}.lang-btn{margin-left:auto;padding:6px 10px;background:transparent;color:#aab5c3;border:1px solid #46556a;border-radius:6px;cursor:pointer;font-size:12px;font-weight:600}.lang-btn:hover{color:#fff;border-color:#3b82f6}@media(max-width:900px){.layout{grid-template-columns:1fr}}
</style></head><body>
<div class="nav"><a href="/" data-i18n-zh="首页" data-i18n-en="Home">首页</a><a href="/extrinsics" data-i18n-zh="单点外参" data-i18n-en="Single Ext">单点外参</a><a href="/extrinsics/multi" data-i18n-zh="多点外参" data-i18n-en="Multi Ext">多点外参</a><a href="/bev" data-i18n-zh="BEV" data-i18n-en="BEV">BEV</a><button id="langToggle" class="lang-btn" onclick="toggleLang()">EN</button></div>
<h1 data-i18n-zh="多位置外参标定" data-i18n-en="Multi-position Extrinsics">多位置外参标定</h1>
<div class="actions"><button class="secondary" onclick="createSession()" data-i18n-zh="新建会话" data-i18n-en="New Session">新建会话</button><button id="solve" onclick="solveSession()" data-i18n-zh="全局解算" data-i18n-en="Global Solve">全局解算</button><button id="commit" onclick="commitSession()" disabled data-i18n-zh="提交四路结果" data-i18n-en="Commit 4">提交四路结果</button><button class="danger" onclick="rollbackSession()" data-i18n-zh="回滚本次提交" data-i18n-en="Rollback">回滚本次提交</button><span id="state"></span></div>
<main class="layout"><section class="panel"><div class="dir-tabs"><label><span data-i18n-zh="相机" data-i18n-en="Camera">相机</span> <select id="dir"><option>front</option><option>back</option><option>left</option><option>right</option></select></label></div><img id="video" class="video" src="/stream?dir=front&overlay=ext"><div class="evidence" id="evidence"></div></section>
<aside class="panel"><h2 id="dirTitle">front · 0/6</h2><div id="positions"></div><div style="margin-top:12px"><label><span data-i18n-zh="近边距离(m)" data-i18n-en="Near (m)">近边距离(m)</span> <input id="near" type="number" step="0.01"></label> <label><span data-i18n-zh="横向(m)" data-i18n-en="Lat (m)">横向(m)</span> <input id="lateral" type="number" step="0.01"></label></div><button id="capture" style="margin-top:10px" onclick="capture()" data-i18n-zh="采集当前位置 8 帧" data-i18n-en="Capture 8 frames">采集当前位置 8 帧</button><div class="quality" id="quality"></div><div id="msg"></div></aside></main>
<script>
const dir=document.getElementById('dir'),near=document.getElementById('near'),lateral=document.getElementById('lateral');let session=null,selected='';
async function post(path,data=''){const r=await fetch(path,{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:data});return r.json()}
function choose(id){selected=id;const item=(session.layout[dir.value]||[]).find(x=>x.id===id);if(item){near.value=Number(item.near_m).toFixed(2);lateral.value=Number(item.lateral_m).toFixed(2)}render()}
function render(){if(!session)return;const d=dir.value,layout=session.layout[d]||[],obs=session.observations[d]||[],done=new Map(obs.map(x=>[x.position_id,x]));document.getElementById('dirTitle').textContent=d+' · '+done.size+'/'+layout.length;document.getElementById('positions').innerHTML=layout.map(p=>{const o=done.get(p.id),cls=(p.id===selected?'selected ':'')+(o?'done':'');return `<div class="position ${cls}" onclick="choose('${p.id}')"><span>${p.near_m.toFixed(2)} m · ${tr('横向','lateral')} ${p.lateral_m>=0?'+':''}${p.lateral_m.toFixed(2)} m</span><span>${o?tr('已采 ','Captured ')+o.frame_count+'/8':tr('待采','Pending')}</span>${o?`<button class="danger" onclick="event.stopPropagation();removeObs('${o.id}')">${tr('删除','Delete')}</button>`:''}</div>`}).join('');if(!selected||!layout.some(x=>x.id===selected)){const next=layout.find(x=>!done.has(x.id))||layout[0];if(next){selected=next.id;near.value=next.near_m;lateral.value=next.lateral_m}}const current=obs.find(x=>x.position_id===selected);document.getElementById('evidence').innerHTML=(current?.evidence||[]).map(n=>`<img src="/api/extrinsics/session/evidence?name=${encodeURIComponent(n)}">`).join('');const sol=session.solutions[d];document.getElementById('quality').innerHTML=sol?`<b class="${sol.passed?'done':'bad'}">${sol.passed?tr('通过','Pass'):tr('未通过','Failed')}</b><div class="metrics">${tr('训练','Train')} RMS ${sol.train_rms_px.toFixed(2)} px / 3.00\n${tr('位置','Position')} P95 ${sol.max_position_p95_px.toFixed(2)} px / 6.00\n${tr('留一','Holdout')} P95 ${sol.max_holdout_p95_px.toFixed(2)} px / 8.00</div>`:`<b>${tr('候选','Candidate')}</b><span class="muted">${tr('完成四路各 6 个位置后解算','Solve after capturing 6 positions for each camera')}</span>`;document.getElementById('state').textContent=tr('会话 ','Session ')+session.id+' · '+session.state;document.getElementById('capture').disabled=session.task_running;document.getElementById('solve').disabled=session.task_running;document.getElementById('commit').disabled=session.state!=='ready';renderUiMessage('msg',null,session.task_message||session.last_error,session.task_running?'common.status_updated':null);}
const renderSession=render;
render=()=>{renderSession();if(!session)return;const stale=!!session.stale;document.getElementById('state').textContent+=''+(stale?' · '+tr('已过期','Stale'):'');document.getElementById('capture').disabled=session.task_running||stale;document.getElementById('solve').disabled=session.task_running||stale;document.getElementById('commit').disabled=session.state!=='ready'||stale;if(stale)renderUiMessage('msg',null,session.stale_reason,tr('请新建会话','Create a new session'));};
dir.onchange=()=>{selected='';document.getElementById('video').src='/stream?dir='+dir.value+'&overlay=ext&t='+Date.now();render()};
async function capture(){if(!selected)return;const data=new URLSearchParams({dir:dir.value,position_id:selected,near:near.value,lateral:lateral.value,orient:'long-lateral'});const j=await post('/api/extrinsics/session/capture',data);renderUiMessage('msg',null,j.msg||j.error,j.ok?'common.status_updated':'common.operation_failed')}
async function removeObs(id){const j=await post('/api/extrinsics/session/remove',new URLSearchParams({dir:dir.value,observation_id:id}));renderUiMessage('msg',null,j.msg||j.error,j.ok?'common.status_updated':'common.operation_failed');poll()}
async function createSession(){if(!confirm(tr('新建会话会归档当前候选采集，正式外参不变。继续吗？','Creating a session archives the current candidate captures. Saved extrinsics remain unchanged. Continue?')))return;const j=await post('/api/extrinsics/session/create');renderUiMessage('msg',null,j.msg||j.error,j.ok?'common.status_updated':'common.operation_failed');selected='';poll()}
async function solveSession(){const j=await post('/api/extrinsics/session/solve');renderUiMessage('msg',null,j.msg||j.error,j.ok?'common.status_updated':'common.operation_failed');poll()}
async function commitSession(){if(!confirm(tr('确认原子替换四路正式外参？当前正式文件会自动备份。','Atomically replace all four saved extrinsics? Current files will be backed up.')))return;const j=await post('/api/extrinsics/session/commit');renderUiMessage('msg',null,j.msg||j.error,j.ok?'common.status_updated':'common.operation_failed');poll()}
async function rollbackSession(){if(!confirm(tr('恢复本次多点提交前的四路外参？','Restore the four extrinsics from before this multi-position commit?')))return;const j=await post('/api/extrinsics/session/rollback');renderUiMessage('msg',null,j.msg||j.error,j.ok?'common.status_updated':'common.operation_failed');poll()}
async function poll(){try{session=await fetch('/api/extrinsics/session').then(r=>r.json());render()}catch(e){renderUiMessage('msg',null,e.message,'common.network_interrupted')}}document.addEventListener('langchange',render);setInterval(poll,800);poll();
</script></body></html>"""


BEV_HTML = """<!doctype html><html lang="zh"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title data-i18n-zh="BEV 俯视预览" data-i18n-en="BEV Bird's-Eye Preview">BEV 俯视预览</title><style>
*{box-sizing:border-box}body{font-family:system-ui,sans-serif;margin:0;padding:clamp(12px,3vw,32px);background:#10141b;color:#e8edf5}.nav{display:flex;gap:8px;margin-bottom:16px;align-items:center;flex-wrap:wrap}.nav a{color:#b9d9ff;text-decoration:none;padding:6px 10px;border:1px solid #344255;border-radius:6px}button,select{padding:8px 13px;background:#287d2b;color:#fff;border:0;border-radius:6px;cursor:pointer;font-size:14px}button.secondary{background:#385a7d}.muted{font-size:13px;color:#aeb9c8}.toolbar{display:flex;gap:10px;align-items:center;flex-wrap:wrap;padding:12px;background:#19212d;border:1px solid #344255;border-radius:9px}.toolbar label{display:flex;gap:6px;align-items:center}.toolbar select{background:#10141b;border:1px solid #4b754c}.bev-viewport{width:min(100%,1000px);aspect-ratio:1;background:#000;border:1px solid #2c3442;border-radius:8px;overflow:hidden;margin-top:12px;display:grid;place-items:center}.bev-viewport img{width:100%;height:100%;object-fit:contain;grid-area:1/1}.bev-placeholder{grid-area:1/1;color:#aeb9c8;text-align:center;padding:24px}.stats{display:flex;gap:10px;flex-wrap:wrap;margin:10px 0}.chip{background:#19212d;border-left:3px solid #4f8f5a;padding:7px 10px;border-radius:5px;font-size:13px}.hint{max-width:1000px;margin-top:10px;padding:10px;border-left:3px solid #5486bb;background:#182332;color:#c8d6e8;font-size:13px}#err{color:#ff9797;font-family:monospace;margin-top:8px}#log{color:#8fdb9a;font-family:monospace;font-size:12px;margin-top:8px}.lang-btn{margin-left:auto;padding:6px 10px;background:transparent;color:#b9d9ff;border:1px solid #46556a;border-radius:6px;cursor:pointer;font-size:12px;font-weight:600}.lang-btn:hover{color:#fff;border-color:#3b82f6}.spacer{flex:1}
</style></head><body>
<h1 data-i18n-zh="BEV 俯视拼接预览" data-i18n-en="BEV Bird's-Eye Preview">BEV 俯视拼接预览</h1>
<div class="nav" style="margin-bottom:16px"><a href="/" data-i18n-zh="首页" data-i18n-en="Home">首页</a><a href="/intrinsics" data-i18n-zh="内参" data-i18n-en="Intrinsics">内参</a><a href="/extrinsics" data-i18n-zh="返回外参" data-i18n-en="Extrinsics">返回外参</a><a href="/seam" data-i18n-zh="接缝" data-i18n-en="Seam">接缝</a><a href="/bev" data-i18n-zh="BEV" data-i18n-en="BEV">BEV</a><span class="spacer" style="flex:1"></span><button id="langToggle" class="lang-btn" onclick="toggleLang()">EN</button></div>
<div class="toolbar"><button onclick="reloadCalibration()" data-i18n-zh="重新加载标定结果" data-i18n-en="Reload Calibration">重新加载标定结果</button><label><span data-i18n-zh="查看模式" data-i18n-en="View mode">查看模式</span> <select id="mode"><option value="blend" data-i18n-zh="米制俯视（4×4m）" data-i18n-en="Metric bird's-eye">米制俯视（4×4m）</option><option value="surround" data-i18n-zh="碗形环视（非米制）" data-i18n-en="Bowl surround (non-metric)">碗形环视（非米制）</option><option value="front" data-i18n-zh="仅前路" data-i18n-en="Front only">仅前路</option><option value="back" data-i18n-zh="仅后路" data-i18n-en="Back only">仅后路</option><option value="left" data-i18n-zh="仅左路" data-i18n-en="Left only">仅左路</option><option value="right" data-i18n-zh="仅右路" data-i18n-en="Right only">仅右路</option><option value="coverage" data-i18n-zh="可信覆盖诊断" data-i18n-en="Coverage diagnostics">可信覆盖诊断</option><option value="ownership" data-i18n-zh="相机归属诊断" data-i18n-en="Camera ownership">相机归属诊断</option></select></label><label data-i18n-title-zh="改变整个 BEV 的观察范围；当前没有单独的顶部裁切设置。" data-i18n-title-en="Changes the complete BEV range; there is no separate top crop."><span data-i18n-zh="地面范围（整体缩放）" data-i18n-en="Ground range">地面范围（整体缩放）</span> <select id="view"><option value="2">2×2 m</option><option value="3">3×3 m</option><option value="4" selected data-i18n-zh="4×4 m（推荐）" data-i18n-en="4×4 m (recommended)">4×4 m（推荐）</option><option value="5">5×5 m</option><option value="6">6×6 m</option><option value="8">8×8 m</option><option value="10">10×10 m</option></select></label><label><span data-i18n-zh="网格" data-i18n-en="Grid">网格</span> <select id="grid"><option value="0" selected data-i18n-zh="关闭" data-i18n-en="Off">关闭</option><option value="0.5">0.5 m</option><option value="1">1 m</option></select></label><label data-i18n-title-zh="只在相邻相机的接缝处混合，不会裁切顶部或改变画面范围。" data-i18n-title-en="Blends only adjacent-camera boundaries without cropping or changing the range."><span data-i18n-zh="接缝混合（仅交界）" data-i18n-en="Seam blend (boundaries only)">接缝混合（仅交界）</span> <select id="transition"><option value="0" data-i18n-zh="0 cm（硬接缝）" data-i18n-en="0 cm (hard seam)">0 cm（硬接缝）</option><option value="0.02">2 cm</option><option value="0.04" selected data-i18n-zh="4 cm（推荐）" data-i18n-en="4 cm (recommended)">4 cm（推荐）</option><option value="0.06">6 cm</option><option value="0.10">10 cm</option></select></label><span class="muted" id="transitionHint"></span><label><input id="showSeams" type="checkbox"> <span data-i18n-zh="显示接缝" data-i18n-en="Show seams">显示接缝</span></label><button onclick="optimizeSeam()" data-i18n-zh="重新优化接缝" data-i18n-en="Optimize Seams">重新优化接缝</button><button class="secondary" onclick="showCompare()" data-i18n-zh="同帧三图对照" data-i18n-en="Compare Same Frame">同帧三图对照</button><button class="secondary" onclick="resetView()" data-i18n-zh="恢复默认视图" data-i18n-en="Reset View">恢复默认视图</button><span id="st"></span></div>
<div class="toolbar" style="margin-top:8px"><label><input id="bodyEnabled" type="checkbox" checked> <span data-i18n-zh="显示车体（实测）" data-i18n-en="Show measured body">显示车体（实测）</span></label><label><span data-i18n-zh="宽(m)" data-i18n-en="Width (m)">宽(m)</span> <input id="bodyW" type="number" min="0.1" max="3" step="0.01" value="0.46" style="width:96px"></label><label><span data-i18n-zh="长(m)" data-i18n-en="Length (m)">长(m)</span> <input id="bodyL" type="number" min="0.1" max="5" step="0.01" value="0.46" style="width:96px"></label></div>
<div class="hint" data-i18n-zh="/seam 是只读的棋盘对齐诊断；这里的“重新优化接缝”才会采集四帧并保存真正的 BEV 几何接缝。黄色线只用于调试，默认关闭。接缝沿四路扇区对角线，只做窄带混合和曝光增益，不再绕开物体或沿地面纹理爬行。" data-i18n-en="/seam is a read-only checkerboard alignment diagnostic. Optimize Seams here samples four frames and saves the actual BEV geometric seam. Yellow lines are debug-only and off by default. Seams follow the four camera-sector diagonals with a narrow blend and exposure gains; they no longer detour around objects or crawl along ground texture.">/seam 是只读的棋盘对齐诊断；这里的“重新优化接缝”才会采集四帧并保存真正的 BEV 几何接缝。黄色线只用于调试，默认关闭。接缝沿四路扇区对角线，只做窄带混合和曝光增益，不再绕开物体或沿地面纹理爬行。</div>
<div id="err"></div>
<div class="bev-viewport"><div id="bevPlaceholder" class="bev-placeholder"><span data-i18n-zh="BEV 尚不可用" data-i18n-en="BEV unavailable">BEV 尚不可用</span><br><span class="muted" data-i18n-zh="请先完成有效的四路外参" data-i18n-en="Complete valid extrinsics for all four cameras first">请先完成有效的四路外参</span></div><img id="bevImage" alt="BEV preview"></div>
<div class="stats"><span class="chip" id="backend">—</span><span class="chip" id="render">—</span><span class="chip" id="active">—</span><span class="chip" id="coverage">—</span></div>
<div id="log"></div>
<script>
const mode=document.getElementById('mode'),view=document.getElementById('view'),grid=document.getElementById('grid'),transition=document.getElementById('transition'),showSeams=document.getElementById('showSeams'),bodyEnabled=document.getElementById('bodyEnabled'),bodyW=document.getElementById('bodyW'),bodyL=document.getElementById('bodyL'),bevImage=document.getElementById('bevImage'),bevPlaceholder=document.getElementById('bevPlaceholder');let bevStreaming=false;
function stopStream(){bevImage.removeAttribute('src');bevImage.style.display='none';bevStreaming=false;bevPlaceholder.hidden=false;}
function refreshStream(){bevImage.removeAttribute('src');bevImage.style.display='block';bevImage.src='/bev/stream?t='+Date.now();bevStreaming=true;bevPlaceholder.hidden=true;}
function updateTransitionHint(){const px=Math.round(Number(transition.value)*1000/Number(view.value));document.getElementById('transitionHint').textContent=tr('仅交界约 '+px+' 像素；不裁切画面','About '+px+' px at boundaries only; no cropping');}
async function setView(){const body=new URLSearchParams({mode:mode.value,view_m:view.value,grid_m:grid.value,transition_m:transition.value,show_seams:showSeams.checked?'1':'0',body_enabled:bodyEnabled.checked?'1':'0',body_w_m:bodyW.value,body_l_m:bodyL.value});const r=await fetch('/api/bev/view',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body});const j=await r.json();renderUiMessage('err',null,j.ok?'':(j.error||tr('设置失败','Settings failed')),j.ok?null:'common.operation_failed');updateTransitionHint();if(j.ok)refreshStream();}
async function reloadCalibration(){try{const r=await fetch('/api/bev/load',{method:'POST'}),j=await r.json();renderUiMessage('err',null,j.ok?'':j.error,j.ok?null:'common.operation_failed');document.getElementById('st').textContent=j.ok?tr('已重新加载','Reloaded'):tr('未加载','Not loaded');if(j.ok)refreshStream();else stopStream();}catch(e){stopStream();renderUiMessage('err',null,e.message,'common.network_interrupted');}}
async function optimizeSeam(){const j=await fetch('/api/bev/seam/optimize',{method:'POST'}).then(r=>r.json());renderUiMessage('err',null,j.ok?'':j.error,j.ok?null:'common.operation_failed');renderUiMessage('st',null,j.msg,j.ok?'common.status_updated':'common.operation_failed');}
function showCompare(){window.open('/api/bev/frame?kind=online','online');window.open('/api/bev/frame?kind=fixed','fixed');window.open('/api/bev/frame?kind=seam','seam');}
function resetView(){mode.value='blend';view.value='4';grid.value='0';transition.value='0.04';showSeams.checked=false;bodyEnabled.checked=true;bodyW.value='0.46';bodyL.value='0.46';setView();}
mode.onchange=setView;view.onchange=setView;grid.onchange=setView;transition.onchange=setView;showSeams.onchange=setView;bodyEnabled.onchange=setView;bodyW.onchange=setView;bodyL.onchange=setView;
async function poll(){try{const s=await fetch('/api/status').then(r=>r.json()),b=s.bev||{},seam=b.seam||{},cover=b.coverage||{},items=Object.values(cover).filter(x=>x&&typeof x==='object');mode.value=b.display_mode||mode.value;view.value=String(b.view_m||4);grid.value=String(b.grid_m||0);transition.value=String(b.transition_m??.04);showSeams.checked=!!b.show_seams;bodyEnabled.checked=!!b.body_size_m;if(b.body_size_m){bodyW.value=b.body_size_m[0];bodyL.value=b.body_size_m[1];}updateTransitionHint();renderUiMessage('err',null,s.bev_error,s.bev_error?'common.operation_failed':null);const seamState=seam.state==='ready'?tr('已优化','Optimized'):seam.state==='sampling'?tr('采集中','Sampling'):seam.state==='failed'?tr('失败：','Failed: ')+(seam.reason_code||''):tr('待优化','Pending');document.getElementById('st').textContent=s.bev_loaded?tr('已加载 · 接缝：','Loaded · seams: ')+seamState+(seam.sample_required?` ${seam.sample_count||0}/${seam.sample_required}`:''):tr('未加载','Not loaded');document.getElementById('backend').textContent=tr('后端：','Backend: ')+(b.backend||'—').toUpperCase();document.getElementById('render').textContent=tr('渲染：','Render: ')+(b.render_ms??'—')+' ms · '+(b.fps??'—')+' fps';document.getElementById('active').textContent=tr('参与相机：','Active cameras: ')+((b.active_dirs||[]).join(', ')||tr('暂无画面','none'))+(b.frame_age_ms!==null&&b.frame_age_ms!==undefined?' · '+tr('最新帧 ','latest ') +b.frame_age_ms+' ms':'');document.getElementById('coverage').textContent=tr('可信区未覆盖：','Uncovered trusted area: ')+(cover.outside_body_uncovered_pct??'—')+'% · '+tr('排除反向 ','reverse rejected ')+items.reduce((n,x)=>n+(x.reverse_rejected_px||0),0)+' / '+tr('错向 ','direction rejected ')+items.reduce((n,x)=>n+(x.direction_rejected_px||0),0)+' px';renderUiMessage('log',null,s.last_log,s.last_log?'common.status_updated':null);}catch(e){renderUiMessage('err',null,e.message,'common.network_interrupted')}}
document.addEventListener('langchange',poll);reloadCalibration();setInterval(poll,1200);
</script></body></html>"""


SEAM_HTML = """<!doctype html><html lang="zh"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title data-i18n-zh="接缝诊断" data-i18n-en="Seam Diagnostics">接缝诊断</title><style>
*{box-sizing:border-box}body{font-family:system-ui,sans-serif;margin:0;padding:clamp(12px,2vw,24px);background:#10141b;color:#e8edf5}
.nav{display:flex;gap:8px;margin:0 0 14px;flex-wrap:wrap;align-items:center}.nav a{padding:8px 14px;background:#2a4d6a;color:#fff;border-radius:6px;text-decoration:none}
h1{font-size:clamp(20px,2vw,28px);margin:0 0 8px}.layout{display:grid;grid-template-columns:minmax(0,1fr) 320px;gap:16px;align-items:start}
.panel{background:#1c212a;border:1px solid #344255;border-radius:12px;padding:14px}.videos{display:grid;grid-template-columns:1fr 1fr;gap:10px}
.video-card{min-width:0}.video{width:100%;max-height:calc(100vh - 210px);object-fit:contain;background:#000;border-radius:8px;display:block}
.cam-title{display:flex;justify-content:space-between;align-items:center;margin-bottom:6px;font-weight:650}
.badge{display:inline-block;padding:2px 8px;border-radius:999px;font-size:12px}.ok{background:#1e742e}.warn{background:#7b5719}.bad{background:#8b302f}.muted{color:#9ba8b9}
.step{border-left:4px solid #596879;background:#171d26;border-radius:6px;padding:9px 10px;margin:9px 0}.step.active{border-left-color:#2fbe48}.step.warn{border-left-color:#e9ad3d}.step.bad{border-left-color:#df5a57}
.pair{padding:8px;border-radius:7px;background:#202934;margin:5px 0;font-size:13px}.pair.current{border-left:3px solid #34b84b}.pair.done{color:#b9f7c1}.pair.todo{color:#b8c3d1}
button{padding:9px 12px;margin:3px 2px;background:#287f32;color:#fff;border:0;border-radius:7px;cursor:pointer;font-weight:650}button:hover{background:#35a143}button:disabled{opacity:.45;cursor:not-allowed}.secondary{background:#38506b}
.small{font-size:12px;color:#b8c3d1;line-height:1.5}.value{font-family:ui-monospace,monospace;font-size:13px;line-height:1.5;white-space:pre-wrap}
#msg{margin-top:9px;color:#9fe8a8;font-family:ui-monospace,monospace;font-size:12px;line-height:1.45;white-space:pre-wrap}
@media(max-width:980px){.layout,.videos{grid-template-columns:1fr}.video{max-height:none}}
.lang-btn{margin-left:auto;padding:6px 10px;background:transparent;color:#b8c3d1;border:1px solid #46556a;border-radius:6px;cursor:pointer;font-size:12px;font-weight:600}.lang-btn:hover{color:#fff;border-color:#3b82f6}.spacer{flex:1}
</style></head><body>
<div class="nav"><a href="/" data-i18n-zh="首页" data-i18n-en="Home">首页</a><a href="/intrinsics" data-i18n-zh="内参" data-i18n-en="Intrinsics">内参</a><a href="/extrinsics" data-i18n-zh="外参" data-i18n-en="Extrinsics">外参</a><a href="/seam" data-i18n-zh="接缝" data-i18n-en="Seam">接缝</a><a href="/bev" data-i18n-zh="BEV" data-i18n-en="BEV">BEV</a><span class="spacer" style="flex:1"></span><button id="langToggle" class="lang-btn" onclick="toggleLang()">EN</button></div>
<h1 data-i18n-zh="接缝诊断：同时看两路重叠区" data-i18n-en="Seam: View Overlap of Two Cameras">接缝诊断：同时看两路重叠区</h1>
<div class="small" style="margin-bottom:12px" data-i18n-zh="把棋盘放到当前高亮的两路相机都能看见的地面重叠区，保持不动。系统只测量两路 BEV 对齐误差，不会修改或保存正式外参。" data-i18n-en="Place the checkerboard motionless on ground visible to both highlighted cameras. The system only measures local BEV alignment and never changes saved extrinsics.">把棋盘放到当前高亮的两路相机都能看见的地面重叠区，保持不动。系统只测量两路 BEV 对齐误差，不会修改或保存正式外参。</div>
<div class="layout"><main class="panel"><div class="videos">
  <div class="video-card"><div class="cam-title"><span id="refTitle" data-i18n-zh="参考路" data-i18n-en="Reference">参考路</span><span id="refBadge" class="badge muted" data-i18n-zh="等待" data-i18n-en="Waiting">等待</span></div><img class="video" id="refVideo" alt="Reference camera"></div>
  <div class="video-card"><div class="cam-title"><span id="slaveTitle" data-i18n-zh="从路" data-i18n-en="Secondary">从路</span><span id="slaveBadge" class="badge muted" data-i18n-zh="等待" data-i18n-en="Waiting">等待</span></div><img class="video" id="slaveVideo" alt="Secondary camera"></div>
</div><div class="small" style="margin-top:8px" data-i18n-zh="绿色角点＝该路已检出完整棋盘；红色＝当前帧不可用。两路同步稳定后只生成诊断结果，正式 H 始终保持不变。" data-i18n-en="Green corners mean a complete checkerboard; red means the frame is unusable. Once both cameras are stable, only a diagnostic result is generated; saved H matrices remain unchanged.">绿色角点＝该路已检出完整棋盘；红色＝当前帧不可用。两路同步稳定后只生成诊断结果，正式 H 始终保持不变。</div></main>
<aside class="panel">
  <div class="step next-step" id="statusBox"><b id="statusTitle" data-i18n-zh="等待四路外参" data-i18n-en="Waiting for 4 extrinsics">等待四路外参</b><div id="statusHint" class="small"></div></div>
  <div class="step" id="liveBox"><b data-i18n-zh="当前同步状态" data-i18n-en="Live synchronization">当前同步状态</b><div id="live" class="value" data-i18n-zh="尚未开始" data-i18n-en="Not started">尚未开始</div></div>
  <div class="step"><b data-i18n-zh="相邻相机对" data-i18n-en="Adjacent camera pairs">相邻相机对</b><div id="pairs"></div></div>
  <div class="step"><b data-i18n-zh="最近一次诊断" data-i18n-en="Latest diagnostic">最近一次诊断</b><div id="last" class="small" data-i18n-zh="尚无结果" data-i18n-en="No result yet">尚无结果</div></div>
  <div style="margin-top:10px">
    <button id="startBtn" onclick="startSeam()" data-i18n-zh="开始接缝诊断" data-i18n-en="Start Seam Diagnostics">开始接缝诊断</button>
    <button id="skipBtn" class="secondary" onclick="skipSeam()" data-i18n-zh="跳过当前对" data-i18n-en="Skip Current Pair">跳过当前对</button>
    <button onclick="goBev()" data-i18n-zh="完成，进入 BEV →" data-i18n-en="Finish, Open BEV →">完成，进入 BEV →</button>
  </div>
  <div id="msg"></div>
</aside></div>
<script>
const refVideo=document.getElementById('refVideo'),slaveVideo=document.getElementById('slaveVideo');
let shownPair='';
async function post(url,body){const r=await fetch(url,{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body});return r.json();}
async function act(c){const j=await post('/api/extrinsics/cmd','cmd='+c);renderUiMessage('msg',null,j.error||j.msg,j.ok?'common.status_updated':'common.operation_failed');return j;}
function goBev(){window.location.assign('/bev');}
function showPair(pair){const key=(pair||[]).join('+');if(!pair||pair.length<2){refVideo.removeAttribute('src');slaveVideo.removeAttribute('src');shownPair='';return;}shownPair=key;document.getElementById('refTitle').textContent=tr('参考路 ','Reference ')+pair[0];document.getElementById('slaveTitle').textContent=tr('从路 ','Secondary ')+pair[1];if(refVideo.dataset.pair!==key){refVideo.dataset.pair=key;refVideo.src='/stream?dir='+encodeURIComponent(pair[0])+'&overlay=ext';slaveVideo.src='/stream?dir='+encodeURIComponent(pair[1])+'&overlay=ext';}}
function badge(el,live){const ok=!!live&&live.phase&&['detected','stable','ready','candidate'].includes(live.phase);const moving=live&&['moving','waiting','bursting'].includes(live.phase);el.className='badge '+(ok?'ok':(moving?'warn':'bad'));el.textContent=ok?tr('已检出棋盘','Checkerboard detected'):reasonText(live?.reason,tr('无完整棋盘','No complete checkerboard'));}
async function startSeam(){const j=await act('seam');if(j.ok)document.getElementById('startBtn').textContent=tr('接缝诊断进行中…','Seam diagnostics running…');}
async function skipSeam(){const j=await post('/api/seam/skip','');renderUiMessage('msg',null,j.error||j.msg,j.ok?'common.status_updated':'common.operation_failed');}
function renderPairs(s){const pairs=s.seam_pairs||[],idx=s.seam_pair_index||0,done=s.seam_complete,mode=s.seam_mode;document.getElementById('pairs').innerHTML=pairs.map((p,i)=>{const cur=mode&&i===idx,finished=done||i<idx,cls=finished?'done':(cur?'current':'todo'),label=finished?tr('已完成','Done'):(cur?tr('当前','Current'):tr('等待','Waiting'));return '<div class="pair '+cls+'"><b>'+p[0]+' + '+p[1]+'</b> · '+label+'</div>';}).join('')||'<div class="small muted">'+tr('未读取到相邻相机对','No adjacent camera pairs found')+'</div>';}
async function poll(){try{const s=await fetch('/api/status').then(r=>r.json()),pair=s.seam_pair||(s.seam_pairs||[])[s.seam_pair_index||0],allDone=['front','back','left','right'].every(d=>!!s.ext_done?.[d]),live=s.ext_live||{},ref=pair?live[pair[0]]||{}:{},slave=pair?live[pair[1]]||{}:{},last=s.seam_last;showPair(pair);if(pair){badge(document.getElementById('refBadge'),ref);badge(document.getElementById('slaveBadge'),slave);}renderPairs(s);const title=document.getElementById('statusTitle'),hint=document.getElementById('statusHint'),btn=document.getElementById('startBtn');document.getElementById('skipBtn').disabled=!s.seam_mode;if(!allDone){title.textContent=tr('还不能开始接缝诊断','Seam diagnostics unavailable');hint.textContent=tr('请先在外参页保存四路结果。','Save valid extrinsics for all four cameras first.');btn.disabled=true;btn.textContent=tr('等待四路外参','Waiting for 4 extrinsics');}else if(s.seam_complete){title.textContent=tr('接缝诊断已完成','Seam diagnostics complete');hint.textContent=tr('正式 H 未修改。可以进入 BEV，或重新诊断。','Saved H matrices are unchanged. Open BEV or run diagnostics again.');btn.disabled=false;btn.textContent=tr('重新开始接缝诊断','Restart Seam Diagnostics');}else if(s.seam_mode&&pair){title.textContent=tr('当前诊断 ','Diagnosing ')+pair[0]+' + '+pair[1];hint.textContent=tr('第 '+((s.seam_pair_index||0)+1)+'/'+(s.seam_pair_count||4)+' 对。把棋盘放到两路重叠区并保持不动。','Pair '+((s.seam_pair_index||0)+1)+'/'+(s.seam_pair_count||4)+'. Keep the checkerboard motionless in the overlap.');btn.disabled=true;btn.textContent=tr('接缝诊断进行中…','Seam diagnostics running…');}else{title.textContent=tr('四路外参已就绪','All 4 extrinsics ready');hint.textContent=tr('点开始后逐对测量局部对齐误差，不会修改外参。','Start to measure local alignment pair by pair without changing extrinsics.');btn.disabled=false;btn.textContent=tr('开始接缝诊断','Start Seam Diagnostics');}const sync=s.seam_streak||0;document.getElementById('live').textContent=s.seam_mode&&pair?([pair[0]+': '+reasonText(ref.reason,tr('等待','Waiting'))+' · '+tr('角点','corners')+' '+(ref.corners_detected||0)+'/'+(ref.expected||48),pair[1]+': '+reasonText(slave.reason,tr('等待','Waiting'))+' · '+tr('角点','corners')+' '+(slave.corners_detected||0)+'/'+(slave.expected||48),tr('同步稳定 ','Synchronized stable ')+sync+'/3'].join(String.fromCharCode(10))):(s.seam_complete?tr('本轮四对诊断已完成','All four pairs completed'):tr('尚未开始','Not started'));document.getElementById('last').textContent=last?(last.error?tr('失败','Failed')+': '+(langGet()==='en'?tr('见技术详情','See technical details'):last.error):(last.ref+'+'+last.slave+': RMS '+Number(last.rms_px||0).toFixed(2)+' px · '+tr('最大 ','max ')+Number(last.max_error_px||0).toFixed(2)+' px · '+tr('只读，H 未修改','read-only; H unchanged'))):tr('尚无结果','No result yet');if(s.ext_task_msg||s.last_log)renderUiMessage('msg',null,s.ext_task_msg||s.last_log,'common.status_updated');}catch(e){renderUiMessage('msg',null,e.message,'common.network_interrupted');}}
document.addEventListener('langchange',poll);setInterval(poll,700);poll();
</script></body></html>"""


# =========================================================================
#  HTTP
# =========================================================================

class Handler(BaseHTTPRequestHandler):
    server_version = "CalibWeb/1.0"

    @property
    def ctx(self) -> SimpleNamespace:
        return self.server.ctx  # type: ignore[attr-defined]

    def log_message(self, fmt, *args):
        pass

    def _send(self, code, body, ctype, extra=None):
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        if extra:
            for k, v in extra.items():
                self.send_header(k, v)
        self.end_headers()
        self.wfile.write(body)

    def _json(self, code, obj):
        self._send(code, json.dumps(obj, ensure_ascii=False, default=str).encode(),
                   "application/json; charset=utf-8")

    def do_GET(self):
        st: CalibState = self.ctx.state
        url = urlparse(self.path)
        q = {k: v[0] for k, v in parse_qs(url.query).items()}
        try:
            if url.path == "/":
                self._send(HTTPStatus.OK, _inject_common(DASHBOARD_HTML).encode(),
                           "text/html; charset=utf-8")
            elif url.path == "/intrinsics":
                self._send(HTTPStatus.OK, _inject_common(INTRINSICS_HTML).encode(),
                           "text/html; charset=utf-8")
            elif url.path == "/extrinsics":
                self._send(HTTPStatus.OK, _inject_common(EXTRINSICS_HTML).encode(),
                           "text/html; charset=utf-8")
            elif url.path in ("/extrinsics/multi", "/extrinsics/multi/"):
                self._send(HTTPStatus.OK, _inject_common(MULTI_EXTRINSICS_HTML).encode(),
                           "text/html; charset=utf-8")
            # Keep the short URL compatible with links/bookmarks from the
            # earlier multi-position calibration UI.
            elif url.path in ("/multi", "/multi/"):
                self._send(HTTPStatus.OK, _inject_common(MULTI_EXTRINSICS_HTML).encode(),
                           "text/html; charset=utf-8")
            elif url.path == "/seam":
                self._send(HTTPStatus.OK, _inject_common(SEAM_HTML).encode(),
                           "text/html; charset=utf-8")
            elif url.path == "/bev":
                self._send(HTTPStatus.OK, _inject_common(BEV_HTML).encode(),
                           "text/html; charset=utf-8")
            elif url.path == "/favicon.ico":
                self._send(HTTPStatus.NO_CONTENT, b"", "image/x-icon",
                           {"Cache-Control": "public, max-age=86400"})
            elif url.path == "/stream":
                d = q.get("dir", "front")
                if d not in DIRECTIONS:
                    self._json(HTTPStatus.BAD_REQUEST, {"error": f"bad dir {d}"})
                    return
                self._stream_mjpeg(lambda: stream_jpeg_camera(st, d, q.get("overlay", "")))
            elif url.path == "/bev/stream":
                self._stream_mjpeg(lambda: stream_jpeg_bev(st))
            elif url.path == "/api/status":
                self._json(HTTPStatus.OK, st.snapshot())
            elif url.path == "/api/extrinsics/session":
                self._json(HTTPStatus.OK, st.multi_snapshot())
            elif url.path == "/api/extrinsics/session/evidence":
                name = Path(q.get("name", "")).name
                path = st.multi_session_dir / "evidence" / name
                if not name or not path.is_file():
                    self._json(HTTPStatus.NOT_FOUND, {"error": "evidence not found"})
                    return
                self._send(HTTPStatus.OK, path.read_bytes(), "image/jpeg",
                           {"Cache-Control": "no-store"})
            elif url.path == "/api/bev/frame":
                kind = q.get("kind", "after")
                if kind not in ("before", "after", "online", "fixed", "seam"):
                    self._json(HTTPStatus.BAD_REQUEST, {"error": "bad frame kind"})
                    return
                data = st.bev_png(kind)
                if not data:
                    self._json(HTTPStatus.NOT_FOUND,
                               {"error": "请先执行一次接缝优化以生成同帧对照"})
                    return
                self._send(HTTPStatus.OK, data, "image/png",
                           {"Cache-Control": "no-store"})
            elif url.path == "/api/extrinsics/burst-frame":
                d = q.get("dir", "")
                try:
                    slot = int(q.get("slot", "-1"))
                except ValueError:
                    slot = -1
                if d not in DIRECTIONS or slot < 0:
                    self._json(HTTPStatus.BAD_REQUEST, {"error": "bad dir or slot"})
                    return
                with st._lock:
                    evidence = st.ext_burst_evidence.get(d, [])
                    data = (evidence[slot].get("jpeg")
                            if slot < len(evidence) else None)
                if not data:
                    self._json(HTTPStatus.NOT_FOUND, {"error": "burst frame unavailable"})
                    return
                self._send(HTTPStatus.OK, data, "image/jpeg",
                           {"Cache-Control": "no-store"})
            else:
                self._json(HTTPStatus.NOT_FOUND, {"error": "not found"})
        except (BrokenPipeError, ConnectionResetError):
            pass

    def do_POST(self):
        st: CalibState = self.ctx.state
        url = urlparse(self.path)
        n = int(self.headers.get("Content-Length") or 0)
        body = self.rfile.read(n) if n else b""
        form = {k: v[0] for k, v in parse_qs(body.decode(errors="ignore"),
                                             keep_blank_values=True).items()}
        try:
            if url.path == "/api/intrinsics/select":
                d = form.get("dir", "front")
                if d not in DIRECTIONS:
                    self._json(HTTPStatus.BAD_REQUEST,
                               {"ok": False, "error": "bad dir"})
                    return
                ok, detail = st.intr_select(d)
                payload = ({"ok": True, "mode": st.mode, "active_dir": d,
                            "message": st.intr_task_ui, **detail}
                           if ok else {"ok": False, "error": detail})
                self._json(HTTPStatus.OK if ok else HTTPStatus.CONFLICT, payload)
            elif url.path == "/api/intrinsics/preview-balance":
                d = form.get("dir", "front")
                try:
                    balance = float(form.get("balance", ""))
                except ValueError:
                    self._json(HTTPStatus.BAD_REQUEST,
                               {"ok": False, "error": "缩放必须是数字"})
                    return
                ok, msg = st.intr_set_preview_balance(d, balance)
                self._json(HTTPStatus.OK if ok else HTTPStatus.BAD_REQUEST,
                           {"ok": ok, "msg": msg, "error": None if ok else msg,
                            "message": st.intr_task_ui})
            elif url.path == "/api/intrinsics/collect":
                d = form.get("dir", "front")
                if d not in DIRECTIONS:
                    self._json(HTTPStatus.BAD_REQUEST, {"ok": False, "error": "bad dir"})
                    return
                enabled = form.get("enabled", "1").lower() in ("1", "true", "yes", "on")
                ok, msg = st.intr_collect(d, enabled)
                self._json(HTTPStatus.OK if ok else HTTPStatus.CONFLICT,
                           {"ok": ok, "msg": msg, "error": None if ok else msg,
                            "message": st.intr_task_ui})
            elif url.path == "/api/extrinsics/start":
                st.mode = "extrinsics"
                if st.target is None:
                    st._ext_advance_target()
                st.ext_running = False
                self._json(HTTPStatus.OK, {"ok": True, "mode": st.mode, "target": st.target, "running": False})
            elif url.path == "/api/intrinsics/calibrate":
                d = form.get("dir", "front")
                if d not in DIRECTIONS:
                    self._json(HTTPStatus.BAD_REQUEST, {"error": "bad dir"})
                    return
                st.mode = "intrinsics"
                st.active_dir = d
                ok, msg = st.intr_start_calibrate(d)
                self._json(HTTPStatus.OK if ok else HTTPStatus.CONFLICT,
                           {"ok": ok, "msg": msg, "error": None if ok else msg,
                            "message": st.intr_task_ui})
            elif url.path == "/api/intrinsics/save":
                d = form.get("dir", "front")
                if d not in DIRECTIONS:
                    self._json(HTTPStatus.BAD_REQUEST, {"ok": False, "error": "bad dir"})
                    return
                confirm = form.get("confirm", "0").lower() in ("1", "true", "yes", "on")
                ok, msg = st.intr_save_candidate(d, confirm)
                self._json(HTTPStatus.OK if ok else HTTPStatus.CONFLICT,
                           {"ok": ok, "msg": msg, "error": None if ok else msg,
                            "message": st.intr_task_ui})
            elif url.path == "/api/intrinsics/clear":
                d = form.get("dir", "front")
                if d not in DIRECTIONS:
                    self._json(HTTPStatus.BAD_REQUEST, {"ok": False, "error": "bad dir"})
                    return
                confirm = form.get("confirm", "0").lower() in ("1", "true", "yes", "on")
                ok, msg = st.intr_clear(d, confirm)
                self._json(HTTPStatus.OK if ok else HTTPStatus.CONFLICT,
                           {"ok": ok, "msg": msg, "error": None if ok else msg,
                            "message": st.intr_task_ui})
            elif url.path == "/api/extrinsics/cmd":
                cmd = form.get("cmd", "")
                ok, msg = st.ext_cmd(cmd)
                self._json(HTTPStatus.OK if ok else HTTPStatus.BAD_REQUEST,
                           {"ok": ok, "msg": msg} if ok else {"ok": False, "error": msg})
            elif url.path == "/api/seam/skip":
                ok, msg = st.seam_skip()
                self._json(HTTPStatus.OK if ok else HTTPStatus.CONFLICT,
                           {"ok": ok, "msg": msg, "error": None if ok else msg})
            elif url.path == "/api/extrinsics/placement":
                d = form.get("dir", st.target or "")
                try:
                    near = float(form.get("near", ""))
                    lateral = float(form.get("lateral", "0"))
                except ValueError:
                    self._json(HTTPStatus.BAD_REQUEST,
                               {"ok": False, "error": "距离或横向偏移必须是数字"})
                    return
                ok, msg = st.ext_set_placement(d, near, lateral)
                self._json(HTTPStatus.OK if ok else HTTPStatus.CONFLICT,
                           {"ok": ok, "msg": msg, "error": None if ok else msg})
            elif url.path == "/api/extrinsics/target":
                d = form.get("dir", "")
                ok, msg = st.ext_select_target(d)
                self._json(HTTPStatus.OK if ok else HTTPStatus.CONFLICT,
                           {"ok": ok, "msg": msg, "error": None if ok else msg,
                            "target": st.target})
            elif url.path == "/api/extrinsics/save":
                d = form.get("dir", st.target or "front")
                if d not in DIRECTIONS:
                    self._json(HTTPStatus.BAD_REQUEST, {"ok": False, "error": "bad dir"})
                    return
                confirm = form.get("confirm", "0").lower() in ("1", "true", "yes", "on")
                ok, msg = st.ext_save_candidate(d, confirm)
                self._json(HTTPStatus.OK if ok else HTTPStatus.CONFLICT, {"ok": ok, "msg": msg, "error": None if ok else msg})
            elif url.path == "/api/extrinsics/verify":
                d = form.get("dir", "front")
                try:
                    near = float(form.get("near", "0.5"))
                    lateral = float(form.get("lateral", "0"))
                except ValueError:
                    self._json(HTTPStatus.BAD_REQUEST, {"ok": False, "error": "bad near/lateral"})
                    return
                if d not in DIRECTIONS:
                    self._json(HTTPStatus.BAD_REQUEST, {"ok": False, "error": "bad dir"})
                    return
                ok, msg = st.ext_verify(d, near, lateral)
                self._json(HTTPStatus.OK, {"ok": ok, "msg": msg})
            elif url.path == "/api/extrinsics/session/create":
                ok, msg = st.multi_create()
                self._json(HTTPStatus.OK if ok else HTTPStatus.CONFLICT,
                           {"ok": ok, "msg": msg, "error": None if ok else msg})
            elif url.path == "/api/extrinsics/session/capture":
                d = form.get("dir", "")
                try:
                    near = float(form.get("near", ""))
                    lateral = float(form.get("lateral", ""))
                except ValueError:
                    self._json(HTTPStatus.BAD_REQUEST,
                               {"ok": False, "error": "距离必须是数字"})
                    return
                ok, msg = st.multi_capture_start(
                    d, form.get("position_id", ""), near, lateral,
                    form.get("orient", "long-lateral"))
                self._json(HTTPStatus.OK if ok else HTTPStatus.CONFLICT,
                           {"ok": ok, "msg": msg, "error": None if ok else msg})
            elif url.path == "/api/extrinsics/session/remove":
                ok, msg = st.multi_remove(form.get("dir", ""),
                                          form.get("observation_id", ""))
                self._json(HTTPStatus.OK if ok else HTTPStatus.CONFLICT,
                           {"ok": ok, "msg": msg, "error": None if ok else msg})
            elif url.path == "/api/extrinsics/session/solve":
                ok, msg = st.multi_solve()
                self._json(HTTPStatus.OK if ok else HTTPStatus.CONFLICT,
                           {"ok": ok, "msg": msg, "error": None if ok else msg})
            elif url.path == "/api/extrinsics/session/commit":
                ok, msg = st.multi_commit()
                self._json(HTTPStatus.OK if ok else HTTPStatus.CONFLICT,
                           {"ok": ok, "msg": msg, "error": None if ok else msg})
            elif url.path == "/api/extrinsics/session/rollback":
                ok, msg = st.multi_rollback()
                self._json(HTTPStatus.OK if ok else HTTPStatus.CONFLICT,
                           {"ok": ok, "msg": msg, "error": None if ok else msg})
            elif url.path == "/api/bev/load":
                ok = st.bev_load()
                self._json(HTTPStatus.OK, {"ok": ok, "error": st.bev_error})
            elif url.path == "/api/bev/view":
                try:
                    view_m = float(form.get("view_m", st.bev_view_m))
                    grid_m = float(form.get("grid_m", st.bev_grid_m))
                    transition_m = float(form.get("transition_m", st.bev_transition_m))
                    body_w_m = (float(form["body_w_m"])
                                if form.get("body_w_m", "").strip() else None)
                    body_l_m = (float(form["body_l_m"])
                                if form.get("body_l_m", "").strip() else None)
                except (TypeError, ValueError):
                    self._json(HTTPStatus.BAD_REQUEST,
                               {"ok": False, "error": "地面范围和网格必须是数字"})
                    return
                enabled = form.get("body_enabled", "0").lower() in ("1", "true", "yes", "on")
                show_seams = form.get("show_seams", "0").lower() in ("1", "true", "yes", "on")
                ok, msg = st.bev_set_view(view_m, grid_m, form.get("mode", "blend"),
                                           transition_m, show_seams, body_w_m, body_l_m,
                                           enabled)
                self._json(HTTPStatus.OK if ok else HTTPStatus.CONFLICT,
                           {"ok": ok, "msg": msg, "error": None if ok else msg,
                            "bev": st.snapshot().get("bev", {})})
            elif url.path == "/api/bev/seam/optimize":
                ok, msg = st.bev_start_seam_optimize()
                self._json(HTTPStatus.OK if ok else HTTPStatus.CONFLICT,
                           {"ok": ok, "msg": msg, "error": None if ok else msg})
            else:
                self._json(HTTPStatus.NOT_FOUND, {"error": "not found"})
        except Exception as exc:  # noqa: BLE001
            self._json(HTTPStatus.INTERNAL_SERVER_ERROR, {"error": str(exc)})

    def _stream_mjpeg(self, getter):
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type",
                         f"multipart/x-mixed-replace; boundary={BOUNDARY}")
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        last_ts = 0.0
        try:
            while True:
                data = getter()
                if data is not None:
                    self.wfile.write(
                        f"--{BOUNDARY}\r\nContent-Type: image/jpeg\r\n"
                        f"Content-Length: {len(data)}\r\n\r\n".encode())
                    self.wfile.write(data)
                    self.wfile.write(b"\r\n")
                else:
                    time.sleep(0.05)
                time.sleep(0.08)
        except (OSError, ConnectionError):
            pass
        finally:
            self.close_connection = True


def lan_ips():
    """枚举所有接口 IPv4（含 tailscale0），Tailscale(100.64/10) 优先打印。

    UDP-connect 只取默认路由接口 IP（这里=enp3s0=10.42.0.86），会漏掉
    tailscale0(100.114.170.49)——而外部 PC 经 Tailscale 连 Jetson 恰恰要用它。
    改用 `hostname -I` 全枚举。"""
    ips = []
    try:
        import subprocess
        out = subprocess.run(["hostname", "-I"], capture_output=True,
                             text=True, timeout=2).stdout
        for ip in out.split():
            # 跳过 loopback + docker bridge（172.16/12）
            if ip.startswith("127.") or ip.startswith("172."):
                continue
            if ip not in ips:
                ips.append(ip)
    except Exception:
        pass
    def rank(ip):
        if ip.startswith("100."): return 0              # Tailscale 优先
        if ip.startswith(("192.168.", "10.")): return 1  # LAN
        return 2
    ips.sort(key=rank)
    return ips


# =========================================================================
#  State 线程 + main
# =========================================================================

def state_loop(state: CalibState):
    """2.5Hz：按当前模式驱动检测/状态机。
    标定不需要 5Hz（棋盘/摆板都是慢动作）；2.5Hz 省 CPU 且 streak=10 约 4s 可达。"""
    interval = 0.4
    next_t = time.monotonic()
    while not state._stop:
        try:
            if state.mode == "intrinsics":
                state.intr_tick()
            elif state.mode == "extrinsics":
                state.ext_tick()
            # bev 模式渲染由 MJPEG 流按需调用，无需 tick
        except Exception as exc:  # noqa: BLE001
            state.log(f"[state] tick 异常: {exc}")
        next_t += interval
        delay = next_t - time.monotonic()
        time.sleep(delay if delay > 0 else 0)


def open_cameras(width, height, target_fps, open_timeout):
    print(f"探测相机（串行打开，每路超时 {open_timeout:.0f}s）...", flush=True)
    results = probe_devices(width, height, open_timeout)
    grabbers = {}
    for idx, wh, be, err in results:
        dev = f"/dev/video{idx}"
        if err:
            print(f"  {dev}: ❌ {err}")
            continue
        print(f"  {dev}: ✅ {be} {wh[0]}x{wh[1]}")
        g = CamGrabber(idx, width, height, target_fps=target_fps)
        g.start()
        grabbers[f"video{idx}"] = g
    # device→方向 映射（从 calib_config.yaml）
    cfg = load_config()
    cam = cfg["capture"]["cameras"]
    dir_grabbers = {}
    for d in DIRECTIONS:
        idx = int(cam[d]["device"])
        g = grabbers.get(f"video{idx}")
        if g is None:
            print(f"  ⚠ {d}: /dev/video{idx} 未探测到，该方向不可用")
            continue
        dir_grabbers[d] = g
    if len(dir_grabbers) < len(DIRECTIONS):
        print("\n  ⚠ 有相机未就绪。若是 video0/1「已打开但未读到帧」，多为 VI 通道卡死"
              "\n    （反复 kill camera_driver 致 dmesg 'enqueue kthread already initialized'）。"
              "\n    max967 模块 in use 无法 rmmod → 重启 Jetson 即恢复 4 路，再跑本脚本。")
    return dir_grabbers, cfg


def load_existing_results(state: CalibState):
    """启动时加载已有内参/外参结果（便于继续标定）。"""
    rd = results_dir()
    for d in DIRECTIONS:
        sp = rd / f"{d}.intr_samples.json"
        if sp.is_file():
            try:
                with sp.open(encoding="utf-8") as f:
                    sd = json.load(f)
                if sd.get("image_size") == [state.width, state.height] and sd.get("pattern_size") == [state.cols, state.rows]:
                    state.intr_samples[d] = [(list(map(float, x["params"])), np.asarray(x["corners"], dtype=np.float64)) for x in (sd.get("samples") or [])]
                    state.intr_rejected[d] = list(sd.get("rejected_samples") or [])
                    print(f"  恢复内参样本 {d} ({len(state.intr_samples[d])} 张)", flush=True)
                else:
                    print(f"  ⚠ 忽略 {d}.intr_samples.json：图像/棋盘配置不匹配", flush=True)
            except Exception as exc:
                print(f"  ⚠ 载入 {d}.intr_samples.json 失败: {exc}", flush=True)
        jp = rd / f"{d}.json"
        if jp.is_file():
            try:
                with jp.open() as f:
                    j = json.load(f)
                state.intr_results[d] = {
                    "K": j["K"], "D": j["D"],
                    "image_size": j.get("image_size", [state.width, state.height]),
                    "rms": j.get("rms"), "model": j.get("model", "equidistant"),
                    "evaluation": j.get("evaluation"),
                    "sample_count": j.get("sample_count"),
                }
                print(f"  载入已有内参 {d}.json (rms={j.get('rms')})", flush=True)
            except Exception as exc:  # noqa: BLE001
                print(f"  ⚠ 载入 {d}.json 失败: {exc}")
    ep = rd / "extrinsics.json"
    stale_marker = rd / "extrinsics.stale.json"
    state.ext_results_stale = stale_marker.exists()
    fresh_directions = set()
    if state.ext_results_stale:
        try:
            with stale_marker.open(encoding="utf-8") as f:
                stale_info = json.load(f)
            fresh_directions = {d for d in stale_info.get("fresh_directions", [])
                                if d in DIRECTIONS}
        except (OSError, ValueError, TypeError):
            # 兼容旧标记文件：其中没有逐路新鲜度时，保守地全部视为过期。
            fresh_directions = set()
    # 无论是否过期，都读取“保存目录”供网页提示；但只有未过期结果
    # 才进入 self.H（BEV/继续标定实际使用的结果）。
    if ep.is_file():
        try:
            data, H = HOM.load_extrinsics_file(ep)
            qcs = data.get("homography_qc") or {}
            rms = data.get("rms_errors") or {}
            poses = data.get("poses") or {}
            sources = data.get("homography_source") or {}
            pose_rebuilt = any(
                item.get("source") == "saved_6dof_poses"
                for item in (data.get("recovery_history") or []))
            for d, M in H.items():
                pose_quality = (HOM.analyze_camera_pose(
                    poses[d], d, state.pose_qc) if poses.get(d) else None)
                invalid_legacy = bool(
                    pose_rebuilt and sources.get(d) not in (
                        "single_position_direct_h", "multi_position_direct_h")
                    and pose_quality and pose_quality.get("status") == "bad")
                usable = ((not state.ext_results_stale or d in fresh_directions)
                          and not invalid_legacy)
                state.ext_saved_catalog[d] = {
                    "state": ("active" if usable else
                              "invalid" if invalid_legacy else "stale"),
                    "rms": float(rms.get(d, 0.0)),
                    "quality": (qcs.get(d) or {}).get("status"),
                    "pose_qc": pose_quality,
                }
                if not usable:
                    continue
                state.H[d] = M
                state.qc[d] = qcs.get(d, {})
                state.rms_errors[d] = rms.get(d, 0.0)
                state.burst_stats[d] = (data.get("burst") or {}).get(d, {})
                state.poses[d] = poses.get(d, {})
                state.measured_bev[d] = (data.get("board_measured_bev") or {}).get(d)
            if not state.ext_results_stale:
                state.seam_history = list(data.get("seam_refined") or [])
                state.verifications = list(data.get("verifications") or [])
                loaded = sum(1 for d in H if d in state.H)
                print(f"  载入已有外参 extrinsics.json ({loaded}/{len(H)} 路可用)",
                      flush=True)
            else:
                print(f"  ⚠ 外参文件因内参更新未全量就绪（已按当前内参重标 "
                      f"{len(fresh_directions)} 路，历史记录 {len(H)} 路）",
                      flush=True)
        except Exception as exc:  # noqa: BLE001
            print(f"  ⚠ 载入 extrinsics.json 失败: {exc}")
    pending_path = rd / "extrinsics.single_pending.json"
    state.ext_pending = ADV.load_single_pending(pending_path, DIRECTIONS)
    if state.ext_pending:
        print(f"  恢复本轮单点外参候选 ({len(state.ext_pending)}/4 路)，"
              "正式外参尚未改变", flush=True)


def main():
    ap = argparse.ArgumentParser(description="Web 标定 pipeline（内参+外参+BEV）")
    ap.add_argument("--port", type=int, default=8090)
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--width", type=int, default=DEFAULT_WIDTH)
    ap.add_argument("--height", type=int, default=DEFAULT_HEIGHT)
    ap.add_argument("--target-fps", type=float, default=10.0,
                    help="每路抓帧上限 (0=不限)")
    ap.add_argument("--open-timeout-s", type=float, default=6.0)
    args = ap.parse_args()

    # 检测 camera_driver 占用
    try:
        r = os.popen("pgrep -x camera_driver 2>/dev/null").read().strip()
        if r:
            print(f"⚠ 检测到 camera_driver (PID {r}) 在跑，会独占相机导致 device busy！")
            print(f"  先停: pkill -x camera_driver  （或 ros2 launch 终端 Ctrl+C）")
    except Exception:
        pass

    dir_grabbers, cfg = open_cameras(args.width, args.height,
                                     args.target_fps, args.open_timeout_s)
    if not dir_grabbers:
        print("❌ 未探测到任何相机：检查接线/驱动 (lsmod | grep max967)，"
              "并确保 camera_driver 未在跑。", flush=True)
        return 2

    state = CalibState(dir_grabbers, cfg)
    state.load_bev_preview_settings()
    state._stop = False
    load_existing_results(state)
    state.sync_multi_session_intrinsics()

    # state 线程默认 idle，页面切换时设 mode（见 POST handler）
    t = threading.Thread(target=state_loop, args=(state,), daemon=True)
    t.start()

    srv = ThreadingHTTPServer((args.host, args.port), Handler)
    srv.ctx = SimpleNamespace(state=state)

    urls = [f"http://127.0.0.1:{args.port}"]
    for ip in lan_ips():
        urls.append(f"http://{ip}:{args.port}")
    print(f"\n✅ 标定台已启动: {'  '.join(urls)}", flush=True)
    print("  页面: 仪表盘 → 内参 → 外参 → /seam 接缝诊断（只读）→ BEV 预览")
    # Mac 浏览器走 Clash 系统代理会拦 100.x 直连；ssh 隧道绕过（SSH 走 Tailscale，
    # 浏览器开 localhost 绕 Clash/ACL）。用真实 Tailscale IP，可直接复制粘贴。
    ts_ip = next((u.split("//")[1].split(":")[0] for u in urls
                  if u.startswith("http://100.")), None)
    if ts_ip:
        print("\n  ⚠ 浏览器直连 Tailscale IP 打不开？Mac 的 Clash 代理会拦 100.x。")
        print("  最稳——Mac 终端跑这条 ssh 隧道，然后浏览器开 http://127.0.0.1:8090：")
        print(f"    ssh -L 8090:127.0.0.1:8090 seeed@{ts_ip}")
        print("  （SSH 走 Tailscale，localhost 绕过 Clash/ACL，100% 通）")
    print("  Ctrl+C 退出\n", flush=True)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        print("\n正在退出...", flush=True)
    finally:
        state._stop = True
        srv.shutdown()
        srv.server_close()
        for g in dir_grabbers.values():
            g.stop()
        for g in dir_grabbers.values():
            g.join(timeout=3.0)
    return 0


if __name__ == "__main__":
    sys.exit(main())
