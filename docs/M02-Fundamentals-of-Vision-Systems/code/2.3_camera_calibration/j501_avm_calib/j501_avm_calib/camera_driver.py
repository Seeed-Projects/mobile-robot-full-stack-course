# -*- coding: utf-8 -*-
"""四路相机驱动节点：V4L2/GStreamer 采集 -> /cameras/<dir>/image_raw +
camera_info，并提供 set_camera_info 服务（cameracalibrator COMMIT 的落盘入口）。

每个方向独立采集线程只做 grab+格式转换，节点主定时器统一发布，
避免 rclpy 回调里阻塞。QV4L2 与 GStreamer backend 逻辑端口自参考项目
avm/camera_io.py。

用法:
    ros2 run j501_avm_calib camera_driver            # 全四路
    ros2 run j501_avm_calib camera_driver --probe    # 只探测并打印
参数见 calib_config.yaml 的 capture 段。
"""
from __future__ import annotations

import array
import os
import threading
import time
from pathlib import Path

import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CameraInfo
from sensor_msgs.srv import SetCameraInfo

from j501_avm_calib.config import load_config, camera_info_dir, DIRECTIONS
from j501_avm_calib import cam_info_io

SENSOR_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST, depth=5,
    reliability=ReliabilityPolicy.RELIABLE,
)

# NVIDIA 相机栈并发 open 不安全：所有相机打开串行化
_OPEN_LOCK = threading.Lock()


def _fourcc_code(fourcc: str) -> int:
    s = (fourcc or "YUYV").upper()
    if len(s) != 4:
        s = "YUYV"
    return cv2.VideoWriter_fourcc(*s)


def _gst_pipeline(index, width, height, fourcc, template, with_videoconvert):
    device = f"/dev/video{index}"
    if template.strip():
        return (template.replace("{device}", device)
                .replace("{width}", str(width))
                .replace("{height}", str(height))
                .replace("{fourcc}", fourcc))
    fmt = "YUY2" if fourcc in ("YUYV", "YUY2") else fourcc
    head = (
        f"v4l2src device={device} io-mode=2 do-timestamp=true ! "
        f"video/x-raw,format={fmt},width={width},height={height} ! "
        f"nvvidconv ! video/x-raw,format=BGRx,width={width},height={height}"
    )
    if with_videoconvert:
        return (f"{head} ! videoconvert ! video/x-raw,format=BGR ! "
                "appsink drop=true max-buffers=1 sync=false")
    return f"{head} ! appsink drop=true max-buffers=1 sync=false"


class CamGrabber(threading.Thread):
    """单路相机采集线程：持续 grab 最新帧，供主定时器读取。"""

    def __init__(self, direction: str, index: int, width: int, height: int,
                 fourcc: str, backend: str, gst_template: str,
                 frame_rate: float, on_event, publish_scale: float = 1.0):
        super().__init__(name=f"cam-{direction}", daemon=True)
        self.direction = direction
        self.index = index
        self.width, self.height = width, height
        self.fourcc = fourcc
        self.backend = backend
        self.gst_template = gst_template
        self.frame_rate = max(1.0, frame_rate)
        # 发布前降采样：全分辨率 8.4MB bgr8 消息会压垮 DDS + 主线程序列化
        # (标定不需要 1920×1536)。<1.0 时缩到 publish_scale 倍后再发布。
        self.publish_scale = float(publish_scale) if publish_scale else 1.0
        self.on_event = on_event  # (level, msg) -> None
        self._lock = threading.Lock()
        self._frame: np.ndarray | None = None
        self._updated_at = 0.0
        self._stamp_ns = 0
        self._stop_event = threading.Event()
        self.cap = None
        self.error: str | None = None
        self.actual_wh = (0, 0)
        self.backend_used = ""

    def latest(self):
        with self._lock:
            return (self._frame.copy() if self._frame is not None else None,
                    self._stamp_ns, self._updated_at)

    def _open_cap(self):
        with _OPEN_LOCK:
            return self._open_cap_locked()

    def _open_cap_locked(self):
        cap = None
        errs = []
        device = f"/dev/video{self.index}"
        if not os.path.exists(device):
            raise RuntimeError(
                f"设备不存在: {device}（检查相机接线/calib_config.yaml "
                f"capture.cameras.{self.direction}.device）")
        if self.backend == "gstreamer":
            for use_vc in (False, True):
                pipe = _gst_pipeline(self.index, self.width, self.height,
                                     self.fourcc, self.gst_template, use_vc)
                try:
                    cap = cv2.VideoCapture(pipe, cv2.CAP_GSTREAMER)
                    if cap.isOpened():
                        ok, frame = cap.read()
                        if ok and frame is not None and frame.size > 0:
                            self._normalize(frame)
                            return cap, f"gst{'+vc' if use_vc else ''}"
                        cap.release()
                        cap = None
                except Exception as exc:  # noqa: BLE001
                    errs.append(str(exc))
        try:
            cap = cv2.VideoCapture(self.index, cv2.CAP_V4L2)
        except Exception as exc:  # noqa: BLE001
            errs.append(str(exc))
            cap = None
        if cap is not None and not cap.isOpened():
            try:
                cap = cv2.VideoCapture(self.index, cv2.CAP_GSTREAMER)
            except Exception:
                cap = None
        if cap is None or not cap.isOpened():
            for use_vc in (False, True):
                pipe = _gst_pipeline(self.index, self.width, self.height,
                                     self.fourcc, self.gst_template, use_vc)
                try:
                    cap = cv2.VideoCapture(pipe, cv2.CAP_GSTREAMER)
                    if cap is not None and cap.isOpened():
                        ok, frame = cap.read()
                        if ok and frame is not None and frame.size > 0:
                            self._normalize(frame)
                            return cap, f"gst{'+vc' if use_vc else ''}"
                    if cap is not None:
                        cap.release()
                    cap = None
                except Exception as exc:  # noqa: BLE001
                    errs.append(str(exc))
                    cap = None
            raise RuntimeError(
                f"cannot open /dev/video{self.index} ({'; '.join(errs)})")
        cap.set(cv2.CAP_PROP_FOURCC, _fourcc_code(self.fourcc))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, int(self.width))
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(self.height))
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        return cap, "v4l2"

    @staticmethod
    def _normalize(frame: np.ndarray) -> np.ndarray:
        if frame.ndim == 3 and frame.shape[2] == 4:
            return frame[:, :, :3].copy()
        return frame

    def run(self):
        try:
            self.cap, self.backend_used = self._open_cap()
        except Exception as exc:  # noqa: BLE001
            self.error = str(exc)
            self.on_event("error", f"[{self.direction}] 打开失败: {exc}")
            return
        for _ in range(3):
            self.cap.grab()
        aw = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH) or 0)
        ah = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT) or 0)
        self.on_event("info",
                      f"[{self.direction}] /dev/video{self.index} "
                      f"({self.backend_used}) "
                      f"{aw}x{ah} 目标速率 {self.frame_rate:.0f}Hz")
        interval = 1.0 / self.frame_rate
        next_t = time.monotonic()
        while not self._stop_event.is_set():
            ok = self.cap.grab()
            if not ok:
                time.sleep(0.02)
                next_t = time.monotonic() + interval
                continue
            ok, frame = self.cap.retrieve()
            if not ok or frame is None:
                continue
            frame = self._normalize(frame)
            if self.publish_scale < 1.0:
                frame = cv2.resize(
                    frame,
                    (max(1, int(frame.shape[1] * self.publish_scale)),
                     max(1, int(frame.shape[0] * self.publish_scale))))
            now = time.time()
            stamp = int(now * 1e9)
            with self._lock:
                self._frame = frame
                self._stamp_ns = stamp
                self._updated_at = now
            self.actual_wh = (frame.shape[1], frame.shape[0])
            # 节流：睡到下一个 tick。始终 grab+retrieve 配对（绝不跳过 retrieve），
            # 否则 CAP_PROP_BUFFERSIZE=1 的 V4L2 单 buffer 队列被饿死 → 空转/卡死。
            next_t += interval
            delay = next_t - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            else:
                next_t = time.monotonic()
        try:
            self.cap.release()
        except Exception:
            pass

    def stop(self):
        self._stop_event.set()


def _yaml_path(direction: str) -> Path:
    return camera_info_dir() / f"{direction}.yaml"


class CameraDriverNode(Node):
    def __init__(self, probe_only: bool = False):
        super().__init__("camera_driver")
        self.cfg = load_config()
        cap = self.cfg["capture"]
        self.declare_parameter("frame_rate_hz", float(cap["frame_rate_hz"]))
        rate = self.get_parameter("frame_rate_hz").value

        self.pubs = {}
        self.srvs = {}
        self.state: dict[str, dict] = {}
        for d in DIRECTIONS:
            cam = cap["cameras"][d]
            self._setup_direction(d, cam)

        self.grabbers: dict[str, CamGrabber] = {}
        if not probe_only:
            for d in DIRECTIONS:
                cam = cap["cameras"][d]
                self.grabbers[d] = CamGrabber(
                    d, int(cam["device"]), int(cap["width"]),
                    int(cap["height"]), str(cap["fourcc"]),
                    str(cap["backend"]), str(cap.get("gst_pipeline_template") or ""),
                    rate, self._on_event,
                    publish_scale=float(cap.get("publish_scale") or 0.5))
                self.grabbers[d].start()
            self.pub_timer = self.create_timer(1.0 / max(rate, 1.0),
                                               self._publish_tick)
            self.get_logger().info("四路相机驱动已启动，发布 /cameras/<dir>/image_raw")

    def _on_event(self, level, msg):
        fn = {"error": self.get_logger().error,
              "warn": self.get_logger().warn,
              "info": self.get_logger().info}.get(level,
                                                  self.get_logger().info)
        fn(msg)

    def _setup_direction(self, d: str, cam: dict):
        """发布器 + set_camera_info 服务（命名空间 /cameras/<d>）。"""
        ns = f"cameras/{d}"
        self.pubs[d] = {
            "image": self.create_publisher(
                Image, f"{ns}/image_raw", SENSOR_QOS),
            "info": self.create_publisher(
                CameraInfo, f"{ns}/camera_info", SENSOR_QOS),
        }
        self.srvs[d] = self.create_service(
            SetCameraInfo, f"{ns}/set_camera_info",
            lambda req, resp, dd=d: self._set_camera_info_cb(req, resp, dd))
        data = self._load_state(d)
        self.state[d] = data
        self.get_logger().info(
            f"[/cameras/{d}] device={cam['device']} "
            f"camera_info={camera_info_dir() / (d + '.yaml')} "
            f"已标定={'是' if data['is_calibrated'] else '否'}")

    def _load_state(self, d: str) -> dict:
        p = _yaml_path(d)
        cap = self.cfg["capture"]
        if p.is_file():
            data = cam_info_io.read_calibration_yaml(p)
            state = cam_info_io.camera_info_to_yaml_dict(
                d, data["width"], data["height"], data["K"], data["D"],
                data["model"])
        else:
            p.parent.mkdir(parents=True, exist_ok=True)
            state = cam_info_io.empty_calibration_yaml(
                d, int(cap["width"]), int(cap["height"]))
            with p.open("w", encoding="utf-8") as f:
                import yaml
                yaml.safe_dump(state, f, default_flow_style=False,
                               sort_keys=False)
        state["_path"] = str(p)
        state["is_calibrated"] = cam_info_io.is_calibrated(state)
        return state

    def _set_camera_info_cb(self, req, resp, d: str):
        """cameracalibrator COMMIT 会调这里：落盘 YAML 并立即生效发布。"""
        try:
            info = req.camera_info
            if not info.k or float(info.k[0]) <= 0.0:
                resp.success = False
                resp.status_message = "拒绝空标定（K 未初始化）"
                return resp
            data = cam_info_io.from_camera_info_msg(info)
            data["camera_name"] = d
            p = Path(self.state[d]["_path"])
            import yaml
            with p.open("w", encoding="utf-8") as f:
                yaml.safe_dump(data, f, default_flow_style=False,
                               sort_keys=False)
            data["_path"] = str(p)
            data["is_calibrated"] = True
            self.state[d] = data
            resp.success = True
            resp.status_message = f"{d} CameraInfo 已保存到 {p}"
            self.get_logger().info(
                f"[/cameras/{d}] 收到标定结果(model={data['distortion_model']}"
                f", D[{len(data['distortion_coefficients']['data'])}]) "
                f"-> {p}")
        except Exception as exc:  # noqa: BLE001
            resp.success = False
            resp.status_message = f"写入失败: {exc}"
            self.get_logger().error(f"[/cameras/{d}] set_camera_info 失败: {exc}")
        return resp

    def _publish_tick(self):
        for d, g in self.grabbers.items():
            frame, stamp, _ = g.latest()
            if frame is None:
                continue
            msg = Image()
            msg.header.stamp.sec = stamp // 10**9
            msg.header.stamp.nanosec = stamp % 10**9
            msg.header.frame_id = f"camera_{d}_optical"
            msg.height = frame.shape[0]
            msg.width = frame.shape[1]
            msg.encoding = "bgr8"
            msg.is_bigendian = False
            msg.step = int(frame.strides[0])
            msg.data = array.array('B', frame.tobytes())
            self.pubs[d]["image"].publish(msg)
            info = cam_info_io.to_camera_info_msg(
                msg.header.stamp, f"camera_{d}_optical", self.state[d])
            self.pubs[d]["info"].publish(info)


def _probe_all(node: Node, cfg) -> bool:
    """启动时探测四路（打开片刻读一帧）。返回是否全部 OK。"""
    cap_cfg = cfg["capture"]
    all_ok = True
    for d in DIRECTIONS:
        cam = cap_cfg["cameras"][d]
        g = CamGrabber(d, int(cam["device"]), int(cap_cfg["width"]),
                       int(cap_cfg["height"]), str(cap_cfg["fourcc"]),
                       str(cap_cfg["backend"]),
                       str(cap_cfg.get("gst_pipeline_template") or ""),
                       1.0, node._on_event)
        g.start()
        g.join(timeout=8.0)
        if g.is_alive():
            g.stop()
            node.get_logger().error(f"[{d}] /dev/video{cam['device']} 打开超时")
            all_ok = False
        elif g.error:
            node.get_logger().error(f"[{d}] 探测失败: {g.error}")
            all_ok = False
        elif g.actual_wh != (0, 0):
            node.get_logger().info(
                f"[{d}] 探测成功 {g.actual_wh[0]}x{g.actual_wh[1]} "
                f"({g.backend_used})")
        else:
            node.get_logger().warn(f"[{d}] 已打开但未读到帧")
            all_ok = False
        g.stop()
    return all_ok


def main(args=None):
    rclpy.init(args=args)
    import sys
    probe_only = "--probe" in (sys.argv[1:] if sys.argv else [])
    node = CameraDriverNode(probe_only=False)
    if probe_only:
        ok = _probe_all(node, node.cfg)
        node.get_logger().info("probe 结果: " + ("✅ 四路全部可用" if ok
                                                 else "❌ 存在不可用相机"))
        if rclpy.ok():
            rclpy.shutdown()
        sys.exit(0 if ok else 1)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except rclpy.executors.ExternalShutdownException:
        pass
    for g in node.grabbers.values():
        g.stop()
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()