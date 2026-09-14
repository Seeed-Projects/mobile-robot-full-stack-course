#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""远程相机调试页: Web MJPEG 观看全部相机(拖拽排序 + 每块命名 + 落盘)

Jetson 上无需连接显示器; 客户端只需浏览器:
    python3 tools/camera_web_viewer.py
    # 打开 http://192.168.2.18:8080 (终端会打印所有可访问 URL)

与 tools/camera_probe_gui.py 复用同一套抓帧(CamGrabber, NVIDIA 栈串行打开)和
落盘逻辑(save_json + update_j501_yaml, 备份 + YAML 校验):
页面拖拽排序 + 每块方向名 -> POST /save -> 同一保存链。

只依赖 Python 标准库 + cv2, 无外部 CDN/依赖。

安全: 默认绑定 0.0.0.0(内网调试); 需更安全可 --host 127.0.0.1 后走
      ssh -L 8080:127.0.0.1:8080 seeed@192.168.2.18 隧道后再开浏览器。
"""
from __future__ import annotations

import argparse
import json
import re
import socket
import sys
import time
import threading
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from types import SimpleNamespace
from urllib.parse import parse_qs, urlparse

import cv2

# 桌面版同源复用: 抓帧线程 / 探测 / 落盘
sys.path.insert(0, str(Path(__file__).resolve().parent))
from camera_probe_gui import (  # noqa: E402
    DEFAULT_NAMES,
    DEFAULT_HEIGHT,
    DEFAULT_WIDTH,
    J501_DIRECTIONS,
    PREVIEW_SCALE,
    CamGrabber,
    find_j501_yaml,
    load_json,
    probe_devices,
    save_json,
    update_j501_yaml,
)

BOUNDARY = "frame"


# ---------------- MJPEG 编码 ----------------

# 每路缓存最近一次编码结果 (device -> (ts, bytes)): 多个浏览器连上来时,
# 同一帧只做一次 resize + imencode, 之后按 _updated_at 命中缓存。
_JPEG_CACHE: dict[str, tuple] = {}


def build_jpeg(grabber: CamGrabber, scale: float, quality: int):
    """取最新帧 -> 缩放 -> 叠加信息 -> JPEG。无新帧返回 None。

    按帧时间戳缓存: 同一帧多浏览器共享一次编码, 降低缩放/软编 CPU。
    """
    fr, ts = grabber.latest()
    if fr is None:
        return None
    cached = _JPEG_CACHE.get(grabber.device)
    if cached is not None and cached[0] == ts:
        return cached[1]
    pv = cv2.resize(fr, (max(1, int(fr.shape[1] * scale)),
                         max(1, int(fr.shape[0] * scale))))
    ov = pv.copy()
    cv2.putText(ov, f"{grabber.device} {fr.shape[1]}x{fr.shape[0]} "
                    f"{grabber.actual_fps:.1f}fps ({grabber.backend})",
                (12, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                (0, 255, 0), 2, cv2.LINE_AA)
    ok, buf = cv2.imencode(".jpg", ov, [cv2.IMWRITE_JPEG_QUALITY, quality])
    if not ok:
        return None
    data = buf.tobytes()
    _JPEG_CACHE[grabber.device] = (ts, data)
    return data


# ---------------- HTTP ----------------

INDEX_HTML = """<!doctype html>
<html lang="zh"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>相机调试页</title>
<style>
 body{font-family:sans-serif;margin:12px;background:#12151a;color:#ddd}
 h1{font-size:18px}
 #grid{display:flex;flex-wrap:wrap;gap:10px}
 .tile{width:505px;background:#1c212a;border:1px solid #2c3442;border-radius:8px;
       padding:6px;cursor:grab}
 .tile.dragging{opacity:.5}
 .bar{display:flex;align-items:center;gap:6px;margin-bottom:4px}
 .handle{color:#7a8;cursor:grab;user-select:none}
 .dev{font-weight:bold;color:#9cf}
 .tile input{flex:1;background:#12151a;color:#eee;border:1px solid #3a4;padding:3px 6px;}
 img{width:100%;height:auto;background:#000;border-radius:4px}
 #bar2{display:flex;gap:10px;align-items:center;margin:0 0 14px;
       position:sticky;top:0;background:#12151a;padding:8px 0;z-index:10}
 #msg{margin-left:10px;color:#8f8}
 #msg.err{color:#e66;margin-left:10px}
 #btn{padding:10px 26px;font-size:16px;background:#2a7d2a;color:#fff;border:0;
     border-radius:6px;cursor:pointer}
 #btn:hover{background:#359a35}
 .err{color:#e66}
</style></head><body>
<h1>多路相机调试 — 拖拽排序, 每块输入方向名, 点顶部保存</h1>
<div id="bar2"><button id="btn">保存映射</button><span id="msg"></span></div>
<div id="grid"></div>
<script>
const grid=document.getElementById('grid');
const msgEl=document.getElementById('msg');
let dragging=null;

function collect(){return [...grid.querySelectorAll('.tile')].map(t=>({
  dev:t.dataset.dev, name:t.querySelector('input').value.trim()}))}

async function loadState(){
  try{
    const r=await fetch('/state'); const s=await r.json();
    const names=s.devices||{};
    for(const t of grid.children){
      const nm=names[t.dataset.dev];
      if(nm) t.querySelector('input').value=nm;
    }
  }catch(e){}
}
async function init(){
  const r=await fetch('/devices'); const devs=await r.json();
  for(const d of devs){
    const tile=document.createElement('div');
    tile.className='tile'; tile.draggable=true; tile.dataset.dev=d.dev;
    const name=(d.name)||(d.error?'':'未标定');
    tile.innerHTML=`<div class="bar"><span class="handle">≡</span>
      <span class="dev ${d.error?'err':''}">${d.dev}</span>
      <span>${d.error?('❌ '+d.error):(d.wh+' '+d.backend)}</span></div>
      <input value="${name}" placeholder="方向名(如 front)">`+
      (d.error?`<div class="err">打开失败: ${d.error}</div>`
              :`<img src="/stream?dev=${encodeURIComponent(d.dev)}">`);
    if(!d.error){tile.querySelector('input').disabled=false;}
    grid.appendChild(tile);
    tile.addEventListener('dragstart',e=>{dragging=tile;tile.classList.add('dragging')});
    tile.addEventListener('dragend',()=>{tile.classList.remove('dragging');dragging=null});
  }
  grid.addEventListener('dragover',e=>{
    e.preventDefault();
    if(!dragging)return;
    const after=[...grid.children].find(t=>t!==dragging&&
      e.clientY < t.getBoundingClientRect().top+t.offsetHeight/2);
    grid.insertBefore(dragging, after||null);
  });
  grid.addEventListener('drop',e=>e.preventDefault());
  await loadState();
}
document.getElementById('btn').addEventListener('click',async()=>{
  const order=collect();
  for(const o of order){ if(!/^[A-Za-z][A-Za-z0-9_]*$/.test(o.name)){
    msgEl.textContent='方向名格式错误(字母开头、字母数字下划线): '+o.dev;return;}}
  const names=order.map(o=>o.name);
  if(new Set(names).size!==names.length){msgEl.textContent='方向名需互不重复';return;}
  msgEl.textContent='保存中...';
  msgEl.classList.remove('err');
  const r=await fetch('/save',{method:'POST',headers:{'Content-Type':'application/json'},
    body:JSON.stringify({order})});
  const s=await r.json();
  if(!s.ok) msgEl.classList.add('err');
  msgEl.textContent=s.ok?('✅ '+JSON.stringify(s.detail)):('❌ '+s.error);
});
init();
</script></body></html>"""


class Handler(BaseHTTPRequestHandler):
    server_version = "CamWebViewer/1.0"

    @property
    def ctx(self) -> SimpleNamespace:
        return self.server.ctx  # type: ignore[attr-defined]

    def log_message(self, fmt, *args):  # 静音默认访问日志
        pass

    def _send(self, code: int, body: bytes, ctype: str, extra: dict | None = None):
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        if extra:
            for k, v in extra.items():
                self.send_header(k, v)
        self.end_headers()
        self.wfile.write(body)

    def _json(self, code: int, obj):
        body = json.dumps(obj).encode()
        self._send(code, body, "application/json; charset=utf-8")

    def do_GET(self):
        url = urlparse(self.path)
        q = {k: v[0] for k, v in parse_qs(url.query).items()}
        try:
            if url.path == "/":
                self._send(HTTPStatus.OK, INDEX_HTML.encode(),
                           "text/html; charset=utf-8")
            elif url.path == "/devices":
                out = []
                for d in self.ctx.devices:
                    ent = {"dev": d["dev"],
                           "wh": f"{d['wh'][0]}x{d['wh'][1]}" if not d["error"] else "-",
                           "backend": d["backend"], "error": d["error"]}
                    remembered = self.ctx.remembered.get(d["dev"], {})
                    if remembered.get("name"):
                        ent["name"] = remembered["name"]
                    out.append(ent)
                self._json(HTTPStatus.OK, out)
            elif url.path == "/state":
                self._json(HTTPStatus.OK,
                           {"devices": self.ctx.remembered})
            elif url.path == "/snapshot":
                dev = q.get("dev", "")
                g = self.ctx.grabbers.get(dev)
                if g is None:
                    self._json(HTTPStatus.NOT_FOUND,
                               {"error": f"unknown dev {dev}"})
                    return
                jpg = build_jpeg(g, self.ctx.scale, self.ctx.quality)
                if jpg is None:
                    self._json(HTTPStatus.SERVICE_UNAVAILABLE,
                               {"error": "no frame yet"})
                    return
                self._send(HTTPStatus.OK, jpg, "image/jpeg",
                           {"Cache-Control": "no-store"})
            elif url.path == "/stream":
                dev = q.get("dev", "")
                g = self.ctx.grabbers.get(dev)
                if g is None:
                    self._json(HTTPStatus.NOT_FOUND,
                               {"error": f"unknown dev {dev}"})
                    return
                self._stream_mjpeg(g)
            else:
                self._json(HTTPStatus.NOT_FOUND, {"error": "not found"})
        except (BrokenPipeError, ConnectionResetError):
            pass

    def do_POST(self):
        url = urlparse(self.path)
        if url.path != "/save":
            self._json(HTTPStatus.NOT_FOUND, {"error": "not found"})
            return
        try:
            n = int(self.headers.get("Content-Length") or 0)
            raw = self.rfile.read(n) if n else b""
            req = json.loads(raw or b"{}")
        except Exception as exc:
            self._json(HTTPStatus.BAD_REQUEST, {"error": f"bad json: {exc}"})
            return
        order = req.get("order") or []
        mapping: dict[str, dict] = {}
        seen_names: set[str] = set()
        try:
            for item in order:
                dev, name = item["dev"], item["name"]
                if dev not in self.ctx.grabbers:
                    raise ValueError(f"unknown dev {dev}")
                if not re.fullmatch(r"[A-Za-z][A-Za-z0-9_]*", name):
                    raise ValueError(f"bad name {name!r}")
                if name in seen_names:
                    raise ValueError(f"duplicate name {name!r}")
                seen_names.add(name)
                mapping[dev] = {"name": name}
            if not mapping:
                raise ValueError("empty order")
        except (KeyError, TypeError, ValueError) as exc:
            self._json(HTTPStatus.BAD_REQUEST, {"error": str(exc)})
            return

        jp = save_json(Path(self.ctx.config_json), mapping)
        device_of = {v["name"]: int(dev.rsplit("video", 1)[1])
                     for dev, v in mapping.items()}
        j501_res = []
        for yp in find_j501_yaml():
            if update_j501_yaml(yp, device_of):
                j501_res.append(str(yp))
        detail = {"json": str(jp), "j501_updated": j501_res}
        self._json(HTTPStatus.OK, {"ok": True, "detail": detail})
        # 同步内存状态, /state 预填即时生效
        self.ctx.remembered = {d: {"name": v["name"]}
                               for d, v in mapping.items()}

    def _stream_mjpeg(self, g: CamGrabber):
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type",
                         f"multipart/x-mixed-replace; boundary={BOUNDARY}")
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        last_ts = 0.0
        try:
            while True:
                fr, ts = g.latest()
                if fr is not None and ts != last_ts:
                    last_ts = ts
                    jpg = build_jpeg(g, self.ctx.scale, self.ctx.quality)
                    if jpg:
                        self.wfile.write(
                            f"--{BOUNDARY}\r\nContent-Type: image/jpeg\r\n"
                            f"Content-Length: {len(jpg)}\r\n\r\n".encode())
                        self.wfile.write(jpg)
                        self.wfile.write(b"\r\n")
                else:
                    time.sleep(0.03)
        except (BrokenPipeError, ConnectionResetError):
            pass


def lan_ips() -> list[str]:
    ips: list[str] = []
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ips.append(s.getsockname()[0])
        s.close()
    except OSError:
        pass
    try:
        for info in socket.getaddrinfo(socket.gethostname(), None,
                                       socket.AF_INET):
            ip = info[4][0]
            if ip not in ips and not ip.startswith("127."):
                ips.append(ip)
    except OSError:
        pass
    return ips


def main():
    ap = argparse.ArgumentParser(description="远程相机调试页 (Web MJPEG)")
    ap.add_argument("--port", type=int, default=8080)
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--width", type=int, default=DEFAULT_WIDTH)
    ap.add_argument("--height", type=int, default=DEFAULT_HEIGHT)
    ap.add_argument("--preview-scale", type=float, default=PREVIEW_SCALE)
    ap.add_argument("--jpg-quality", type=int, default=70)
    ap.add_argument("--target-fps", type=float, default=10.0,
                    help="每路相机处理帧率上限 (0=不限); 降频省 CPU")
    ap.add_argument("--open-timeout-s", type=float, default=6.0)
    ap.add_argument("--config-json",
                    default=str(Path(__file__).resolve().parents[1] / "config"
                                 / "camera_indices.json"))
    args = ap.parse_args()

    remembered = {}
    cfg = Path(args.config_json)
    if cfg.exists():
        remembered = load_json(cfg).get("devices", {})

    print("探测相机 (串行打开)...", flush=True)
    results = probe_devices(args.width, args.height, args.open_timeout_s)
    devices = []
    grabbers: dict[str, CamGrabber] = {}
    for idx, wh, be, err in results:
        dev = f"/dev/video{idx}"
        if err:
            print(f"  {dev}: ❌ {err}")
            devices.append({"dev": dev, "wh": (0, 0), "backend": "",
                            "error": err})
            continue
        print(f"  {dev}: ✅ {be} {wh[0]}x{wh[1]}")
        devices.append({"dev": dev, "wh": wh, "backend": be, "error": ""})
        g = CamGrabber(idx, args.width, args.height,
                       target_fps=args.target_fps)
        g.start()
        grabbers[dev] = g

    srv = ThreadingHTTPServer((args.host, args.port), Handler)
    srv.ctx = SimpleNamespace(devices=devices, grabbers=grabbers,
                              remembered=remembered,
                              config_json=args.config_json,
                              scale=args.preview_scale,
                              quality=args.jpg_quality)

    urls = [f"http://127.0.0.1:{args.port}"]
    for ip in lan_ips():
        urls.append(f"http://{ip}:{args.port}")
    print(f"\n✅ 相机调试页已启动: {'  '.join(urls)}")
    print("  页面: 拖拽画面块排序, 每块输入方向名, 点『保存映射』")
    print("  安全提示: 内网无鉴权; 更安全可 --host 127.0.0.1 后加 ssh 隧道:")
    print(f"    ssh -L 8080:127.0.0.1:{args.port} seeed@<jetson-ip>")
    print("  Ctrl+C 退出\n", flush=True)

    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        print("\n正在退出...", flush=True)
    finally:
        srv.shutdown()
        srv.server_close()
        for g in grabbers.values():
            g.stop()
        for g in grabbers.values():
            g.join(timeout=3.0)
    return 0


if __name__ == "__main__":
    sys.exit(main())