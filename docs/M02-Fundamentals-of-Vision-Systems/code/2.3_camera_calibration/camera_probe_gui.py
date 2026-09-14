#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""多路相机调试启动器（自适应探测 + 窗口排列命名 + 映射落盘）

用法:
    python3 tools/camera_probe_gui.py                 # 打开全部相机, 拖好窗口按 s 保存
    python3 tools/camera_probe_gui.py --probe-only    # 无 GUI 探测(打印设备号/分辨率)
    python3 tools/camera_probe_gui.py --names "front back left right" \\
        --auto-save-ms 8000                           # 自动化测试: 定时自动保存退出

行为:
  - 自适应枚举 /dev/video[0-9]*(V4L2 逐路串行打开, NVIDIA 相机栈并发 open 不安全);
    当前 4 路 GMSL 鱼眼, 未来接 6 路或更多同样适用。
  - 每路一个独立 OpenCV 窗口(可自由拖动), 标题含设备号/分辨率/已记忆的方向名;
    预览半分辨率 960x768。
  - 把窗口拖成实车布局后按 s(或回车): 按屏幕位置(从上到下、从左到右)排序,
    交互式输入方向名 -> 映射写入:
      config/camera_indices.json          (设备 <-> 方向 <-> 窗口位置, 下次启动恢复)
      ~/ros2_ws/.../j501_avm_calib/.../calib_config.yaml  (front/back/left/right 的
          device 值, 写前备份 *.bak-<时间戳>, 注释保留, 写后 YAML 重新解析校验)
    同时终端打印可直接粘贴的 YAML 片段。
  - Esc / q: 仅退出不保存。
"""
from __future__ import annotations

import argparse
import glob
import json
import os
import re
import shutil
import sys
import threading
import time
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np

DEFAULT_NAMES = ("front", "back", "left", "right")
J501_DIRECTIONS = ("front", "back", "left", "right")  # j501 标定包固定四向
PREVIEW_SCALE = 0.5          # 1920x1536 -> 960x768 预览
DISPLAY_HZ = 20.0            # 主循环刷新率
DEFAULT_WIDTH, DEFAULT_HEIGHT = 1920, 1536

# NVIDIA 相机栈并发 open 不安全: 所有相机打开串行化 (j501_avm_calib 同款约束)
_OPEN_LOCK = threading.Lock()


def _fourcc_code(fourcc: str) -> int:
    s = (fourcc or "YUYV").upper().strip()
    return cv2.VideoWriter_fourcc(*s) if len(s) == 4 else 0


def _gst_pipeline(index: int, width: int, height: int) -> str:
    return (f"v4l2src device=/dev/video{index} io-mode=2 do-timestamp=true ! "
            f"video/x-raw,format=YUY2,width={width},height={height} ! "
            f"videoconvert ! video/x-raw,format=BGR ! "
            f"appsink drop=true max-buffers=1 sync=false")


class CamGrabber(threading.Thread):
    """单路采集线程: 持续 grab 最新帧, 主线程定时取用 (顺串行打开)。"""

    def __init__(self, index: int, width: int, height: int,
                 target_fps: float = 0.0):
        super().__init__(name=f"cam-{index}", daemon=True)
        self.index = index
        self.device = f"/dev/video{index}"
        self.width, self.height = width, height
        self.cap = None
        self.backend = ""
        self.error = ""
        self.actual_wh = (0, 0)
        self.actual_fps = 0.0
        # 帧率节流: >0 时每路最多处理 target_fps 帧/秒 (0=不限)。
        # 靠「处理后 sleep 到下一个 tick」降频, 始终保持 grab+retrieve 配对;
        # 绝不在 grab 后跳过 retrieve(否则 BUFFERSIZE=1 的 V4L2 队列被饿死)。
        self._target_fps = max(0.0, float(target_fps))
        self._interval = (1.0 / self._target_fps
                          if self._target_fps > 0 else 0.0)
        self._stop_evt = threading.Event()
        self._lock = threading.Lock()
        self._frame = None
        self._updated_at = 0.0
        # 单调递增的采集序号。标定状态机用它确认拿到的是新画面，
        # 不能把同一帧误算为“稳定了很多帧”。
        self._frame_id = 0

    def _open_cap(self):
        errs = []
        with _OPEN_LOCK:
            try:
                cap = cv2.VideoCapture(self.device, cv2.CAP_V4L2)
                if not cap.isOpened():
                    raise RuntimeError("v4l2 open failed")
                cap.set(cv2.CAP_PROP_FOURCC, _fourcc_code("YUYV"))
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
                cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                return cap, "v4l2"
            except Exception as exc:  # noqa: BLE001
                errs.append(f"v4l2: {exc}")
            try:
                cap = cv2.VideoCapture(_gst_pipeline(self.index, self.width,
                                                     self.height),
                                       cv2.CAP_GSTREAMER)
                if not cap.isOpened():
                    raise RuntimeError("gstreamer open failed")
                return cap, "gstreamer"
            except Exception as exc:  # noqa: BLE001
                errs.append(f"gstreamer: {exc}")
        raise RuntimeError("; ".join(errs) or "cannot open")

    def run(self):
        try:
            self.cap, self.backend = self._open_cap()
        except Exception as exc:  # noqa: BLE001
            self.error = str(exc)
            return
        for _ in range(3):
            if not self.cap.grab():
                time.sleep(0.05)
        aw = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH) or 0)
        ah = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT) or 0)
        with self._lock:
            self.actual_wh = (aw, ah)
        t0 = time.monotonic()
        frames = 0
        next_t = time.monotonic()
        while not self._stop_evt.is_set():
            ok = self.cap.grab()
            if not ok:
                time.sleep(0.02)
                next_t = time.monotonic() + self._interval
                continue
            ok, frame = self.cap.retrieve()
            if not ok or frame is None:
                continue
            if frame.ndim == 3 and frame.shape[2] == 4:
                frame = frame[:, :, :3]
            now = time.monotonic()
            with self._lock:
                self._frame = frame
                self._updated_at = now
                self._frame_id += 1
            frames += 1
            dt = now - t0
            if dt >= 1.0:
                with self._lock:
                    self.actual_fps = frames / dt
                t0, frames = now, 0
            if self._interval <= 0:
                continue
            next_t += self._interval
            delay = next_t - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            else:
                next_t = time.monotonic()   # 防累计漂移导致多帧卡死
        try:
            self.cap.release()
        except Exception:
            pass

    def latest(self):
        with self._lock:
            return self._frame, self._updated_at

    def latest_with_id(self):
        """返回最新帧、时间戳和递增帧号；不改变原 latest() 兼容性。"""
        with self._lock:
            return self._frame, self._updated_at, self._frame_id

    def stop(self):
        self._stop_evt.set()


def enumerate_devices() -> list[int]:
    """高危设备枚举: /dev/video 数字后缀升序。"""
    devs = []
    for p in sorted(glob.glob("/dev/video*")):
        m = re.match(r"/dev/video(\d+)$", p)
        if m:
            devs.append(int(m.group(1)))
    return devs


def probe_devices(width: int, height: int, timeout_s: float = 6.0):
    """逐路串行打开 + 读一帧, 返回 [(index, actual_wh, backend, error)]。"""
    out = []
    for idx in enumerate_devices():
        g = CamGrabber(idx, width, height)
        g.start()
        g.join(timeout=timeout_s)
        if g.error:
            out.append((idx, (0, 0), "", f"打开失败: {g.error}"))
            g.stop()
            g.join(timeout=3.0)
            continue
        # 等待读帧 (最多 timeout_s)
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if g.latest()[0] is not None:
                break
            time.sleep(0.05)
        fr, _ = g.latest()
        g.stop()
        g.join(timeout=3.0)
        if fr is None:
            out.append((idx, g.actual_wh, g.backend, "已打开但未读到帧"))
        else:
            out.append((idx, (int(fr.shape[1]), int(fr.shape[0])),
                        g.backend, ""))
    return out


# ---------------- 窗口几何 ----------------

def window_rect(name: str):
    """返回 (x, y, w, h); 失败返回 None。"""
    try:
        x, y, w, h = cv2.getWindowImageRect(name)
        if w > 0 and h > 0:
            return (int(x), int(y), int(w), int(h))
    except Exception:
        pass
    return None


def sort_by_screen(windows: dict[str, tuple]):
    """按屏幕位置 (y, x) 排序; 几何失败的回退设备号并记警告。"""
    ok = [(n, r) for n, r in windows.items() if r]
    if len(ok) == len(windows):
        return sorted(ok, key=lambda nr: (nr[1][1], nr[1][0])), True
    fallback = sorted(windows.items(), key=lambda kv: kv[0])
    return fallback, False


# ---------------- 配置落盘 ----------------

def repo_root() -> Path:
    return Path(__file__).resolve().parents[1]


def default_json_path() -> Path:
    return repo_root() / "config" / "camera_indices.json"


def load_json(path: Path):
    if path.exists():
        try:
            return json.loads(path.read_text())
        except Exception:
            return {}
    return {}


def save_json(path: Path, mapping: dict[str, dict]) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    doc = {
        "updated_at": datetime.now().isoformat(timespec="seconds"),
        "devices": dict(sorted(mapping.items())),
    }
    path.write_text(json.dumps(doc, indent=2, ensure_ascii=False) + "\n")
    return path


def find_j501_yaml() -> Path | None:
    """j501_avm_calib 的 calib_config.yaml 真源 (解析 install/build 软链)。"""
    cands = [
        Path.home() / "ros2_ws/src/j501_avm_calib/config/calib_config.yaml",
        Path.home() / "ros2_ws/install/j501_avm_calib/share/j501_avm_calib/"
                     "config/calib_config.yaml",
        Path.home() / "ros2_ws/build/j501_avm_calib/config/calib_config.yaml",
    ]
    seen = set()
    for c in cands:
        if not c.exists():
            continue
        real = c.resolve()
        if real not in seen:
            seen.add(real)
            yield real


def update_j501_yaml(path: Path, device_of: dict[str, int]) -> bool:
    """仅替换 capture.cameras 段内 front/back/left/right 的 device 值。

    行级正则 -> 保留注释; 写前备份 *.bak-<时间戳>; 写后 yaml 重新解析校验。
    """
    import yaml  # 延迟导入: --probe-only 不需要
    lines = path.read_text().splitlines()
    in_cameras = False
    block_indent = None
    n_updated = 0
    out = []
    for ln in lines:
        m_block = re.match(r"^(\s*)cameras\s*:\s*$", ln)
        if m_block:
            in_cameras = True
            block_indent = len(m_block.group(1))
            out.append(ln)
            continue
        if in_cameras:
            stripped = ln.strip()
            if stripped and not stripped.startswith("#"):
                indent = len(ln) - len(ln.lstrip())
                if indent <= block_indent:   # 块结束 (同级或更浅)
                    in_cameras = False
                else:
                    for name in J501_DIRECTIONS:
                        if name not in device_of:
                            continue
                        m = re.match(
                            rf"^(\s*{name}\s*:\s*\{{device:\s*)\d+(\}}\s*)$",
                            ln)
                        if m:
                            ln = f"{m.group(1)}{device_of[name]}{m.group(2)}"
                            n_updated += 1
                            break
        out.append(ln)
    new_text = "\n".join(out) + "\n"
    if n_updated == 0:
        return False
    try:
        parsed = yaml.safe_load(new_text)
        assert isinstance(parsed, dict)
    except Exception:
        return False
    bak = path.with_name(path.name + f".bak-{datetime.now():%Y%m%d%H%M%S}")
    shutil.copy2(path, bak)
    path.write_text(new_text)
    return True


def print_yaml_snippet(device_of: dict[str, int]):
    print("\n可直接粘贴到 j501_avm_calib config/calib_config.yaml:")
    print("capture:")
    print("  cameras:")
    for name in J501_DIRECTIONS:
        if name in device_of:
            print(f"    {name}: {{device: {device_of[name]}}}")
    print()


# ---------------- 交互命名 ----------------

def prompt_names(sorted_devs: list[str], remembered: dict[str, str]):
    """交互式输入方向名; 输入 q 返回 None(放弃保存)。"""
    print("\n窗口已按 (从上到下, 从左到右) 排序:")
    for i, dev in enumerate(sorted_devs, 1):
        old = remembered.get(dev, "")
        print(f"  {i}) {dev}" + (f"  (上次: {old})" if old else ""))
    n = len(sorted_devs)
    def_names = [remembered.get(d, "") or
                 (DEFAULT_NAMES[i] if i < len(DEFAULT_NAMES) else f"cam{i + 1}")
                 for i, d in enumerate(sorted_devs)]
    hint = " ".join(def_names)
    while True:
        raw = input(
            f"\n请按上述顺序输入 {n} 个方向名(空格分隔),\n"
            f"直接回车使用默认 [{hint}], 或输入 q 放弃保存: ").strip()
        if raw.lower() in ("q", "quit"):
            return None
        names = (hint if not raw else raw).split()
        if len(names) != n:
            print(f"方向名数量 {len(names)} != 相机数 {n}, 请重输")
            continue
        if len(set(names)) != n:
            print("方向名需互不重复, 请重输")
            continue
        if all(re.fullmatch(r"[A-Za-z][A-Za-z0-9_]*", nm) for nm in names):
            return names
        print("方向名仅允许字母开头、字母数字下划线, 请重输")


# ---------------- 主流程 ----------------

def shutdown(windows, grabbers):
    """统一收尾: 停线程 -> join -> 销毁窗口(顺序反过来可能崩 GTK)。"""
    for g in grabbers.values():
        g.stop()
    for g in grabbers.values():
        g.join(timeout=3.0)
    for t in windows:
        try:
            cv2.destroyWindow(t)
        except cv2.error:
            pass


def run_gui(args):
    if os.environ.get("DISPLAY", "").strip() == "":
        os.environ["DISPLAY"] = args.display

    remembered = {}
    old_doc = load_json(Path(args.config_json))
    remembered = {dev: v.get("name", "")
                  for dev, v in old_doc.get("devices", {}).items()}

    print(f"探测相机 (串行打开, 每路超时 {args.open_timeout_s:.0f}s)...",
          flush=True)
    results = probe_devices(args.width, args.height, args.open_timeout_s)
    ok_devs = [(i, wh, be) for i, wh, be, err in results if not err]
    for i, wh, be, err in results:
        if err:
            print(f"  /dev/video{i}: ❌ {err}")
        else:
            print(f"  /dev/video{i}: ✅ {be} {wh[0]}x{wh[1]}")
    if not ok_devs:
        print("未探测到可用相机: 请检查接线/驱动 (lsmod | grep max967)。")
        return 2

    grabbers = {}
    for idx, _, _ in ok_devs:
        g = CamGrabber(idx, args.width, args.height)
        g.start()
        grabbers[idx] = g
    # 等首批帧
    deadline = time.monotonic() + args.open_timeout_s
    while time.monotonic() < deadline:
        if all(g.latest()[0] is not None for g in grabbers.values()):
            break
        time.sleep(0.05)

    # 窗口
    title = {}
    win_names = {}
    prev_pos = old_doc.get("devices", {})
    try:
        for idx, _, backend in ok_devs:
            dev = f"/dev/video{idx}"
            nm = remembered.get(dev, "未标定")
            t = f"{dev} [{nm}]"
            title[dev] = t
            win_names[dev] = t
            cv2.namedWindow(t, cv2.WINDOW_NORMAL | cv2.WINDOW_GUI_NORMAL)
            pos = prev_pos.get(dev, {}).get("pos")
            if pos and len(pos) == 4:
                try:
                    cv2.moveWindow(t, int(pos[0]), int(pos[1]))
                except Exception:
                    pass
    except cv2.error as exc:
        for g in grabbers.values():
            g.stop()
        print(f"❌ GUI 初始化失败: {exc}", file=sys.stderr)
        print("提示: 请在有桌面会话的终端运行 (export DISPLAY=:0 或桌面终端直接运行);"
              "仅探测可用 --probe-only。", file=sys.stderr)
        return 2

    print("\n窗口已打开。把窗口拖动成实车布局后: s=保存映射退出, q/Esc=不保存退出")

    scale = args.preview_scale
    save_requested = False
    auto_deadline = (time.monotonic() + args.auto_save_ms / 1000.0
                     if args.auto_save_ms > 0 else None)

    while True:
        now = time.monotonic()
        for dev, t in title.items():
            idx = int(dev.rsplit("video", 1)[1])
            g = grabbers[idx]
            fr, _ = g.latest()
            if fr is not None:
                pv = cv2.resize(fr, (int(fr.shape[1] * scale),
                                     int(fr.shape[0] * scale)))
                ov = pv.copy()
                cv2.putText(ov, f"{dev} {fr.shape[1]}x{fr.shape[0]} "
                                f"{g.actual_fps:.1f}fps ({g.backend})",
                            (12, 34), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                            (0, 255, 0), 2, cv2.LINE_AA)
                cv2.imshow(t, ov)
        key = cv2.waitKey(max(1, int(1000 / DISPLAY_HZ)))
        if key in (27, ord("q")):
            break
        if key in (ord("s"), 13):
            save_requested = True
            break
        if auto_deadline and now >= auto_deadline:
            save_requested = True
            break

    if not save_requested:
        shutdown(list(win_names.values()), grabbers)
        print("未保存退出 (q/Esc)")
        return 1

    # ---- 保存流程: 读取窗口几何 -> 排序 -> 命名 -> 落盘 ----
    print("正在读取窗口位置...", flush=True)
    rects = {}
    for dev, t in title.items():
        rects[dev] = window_rect(t)
        if rects[dev] is None:
            print(f"  警告: 无法读取窗口位置 {dev}, 将按设备号顺序回退")
    ordered, geom_ok = sort_by_screen({d: rects[d] for d in title})
    sorted_devs = [d for d, _ in ordered]
    if not geom_ok:
        print("  警告: 部分窗口几何不可用, 按设备号顺序命名")

    if args.names:
        names = args.names.split()
        if len(names) != len(sorted_devs):
            print(f"--names 数量 {len(names)} != 相机数 {len(sorted_devs)}, 放弃保存")
            shutdown(list(win_names.values()), grabbers)
            return 3
        print(f"使用命令行方向名: {names}")
    else:
        names = prompt_names(sorted_devs, remembered)
        if names is None:
            print("已放弃保存")
            shutdown(list(win_names.values()), grabbers)
            return 1

    mapping: dict[str, dict] = {}
    for dev, nm in zip(sorted_devs, names):
        entry = {"name": nm}
        r = rects.get(dev)
        if r:
            entry["pos"] = list(r)
        mapping[dev] = entry

    jp = save_json(Path(args.config_json), mapping)
    print(f"✅ 映射已写入 {jp}")

    device_of = {v["name"]: int(dev.rsplit("video", 1)[1])
                 for dev, v in mapping.items()}
    j501_res = []
    for yp in find_j501_yaml():
        if update_j501_yaml(yp, device_of):
            j501_res.append(str(yp))
    device_of = {n: d for n, d in device_of.items() if n in J501_DIRECTIONS}
    print_yaml_snippet(device_of)
    if j501_res:
        print(f"✅ j501_avm_calib 配置已更新: {', '.join(j501_res)}")
    else:
        print("⚠️ 未找到/未更新 j501 calib_config.yaml (新映射只写入 JSON)")

    # ---- 收尾 ----
    shutdown(list(win_names.values()), grabbers)
    return 0


def main():
    ap = argparse.ArgumentParser(description="多路相机调试启动器")
    ap.add_argument("--probe-only", action="store_true",
                    help="仅探测打印 (无 GUI)")
    ap.add_argument("--width", type=int, default=DEFAULT_WIDTH)
    ap.add_argument("--height", type=int, default=DEFAULT_HEIGHT)
    ap.add_argument("--config-json", default=str(default_json_path()))
    ap.add_argument("--names", default="",
                    help="方向名(空格分隔, 个数=相机数); 提供后保存不再交互")
    ap.add_argument("--preview-scale", type=float, default=PREVIEW_SCALE)
    ap.add_argument("--open-timeout-s", type=float, default=6.0)
    ap.add_argument("--auto-save-ms", type=float, default=0.0,
                    help="自动化测试: N 毫秒后自动保存退出")
    ap.add_argument("--display", default=":0")
    args = ap.parse_args()

    if args.probe_only:
        print("=== 相机探测 ===")
        for idx, wh, be, err in probe_devices(args.width, args.height,
                                              args.open_timeout_s):
            if err:
                print(f"/dev/video{idx}: ❌ {err}")
            else:
                print(f"/dev/video{idx}: ✅ {be} {wh[0]}x{wh[1]}")
        return 0

    return run_gui(args)


if __name__ == "__main__":
    sys.exit(main())
