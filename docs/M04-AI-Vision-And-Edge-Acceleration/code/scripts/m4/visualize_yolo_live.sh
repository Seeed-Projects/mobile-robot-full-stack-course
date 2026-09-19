#!/usr/bin/env bash
# M4.1 实时摄像头目标检测 — 永久运行 + Ctrl-C 优雅退出。
#
# 同时跑：
#   1. csi_camera_publisher  (实时摄像头 → /perception/cameras/front/image)
#   2. yolo_trt_node         (TensorRT YOLO → /perception/detections + debug image)
#   3. cv_viewer             (持续把 debug image 显示在 OpenCV 窗口里 — VNC 上 :10)
#      或 (SHOW_WINDOW=0) screenshot_saver (落盘 PNG)
#
# 默认 DURATION=0 → 永久运行，直到 Ctrl-C。
# Ctrl-C 时优雅退出：SIGINT 子节点 → 等 5s → SIGKILL → 验证 /dev/video* 已释放。
#
# 用法：
#   bash scripts/m4/visualize_yolo_live.sh
#   CAMERA_SOURCE=test bash scripts/m4/visualize_yolo_live.sh
#   CAMERA_SOURCE=usb CAMERA_DEVICE=/dev/video2 bash scripts/m4/visualize_yolo_live.sh
#   DURATION=60 bash scripts/m4/visualize_yolo_live.sh     # 60 秒后自动停
#   SHOW_WINDOW=0 bash scripts/m4/visualize_yolo_live.sh   # 不弹窗，落盘 PNG
#   VIEWER_DISPLAY=:10 bash scripts/m4/visualize_yolo_live.sh   # VNC 上 :10 显示
#
set -u
# Note: deliberately NOT using 'set -e' or 'pipefail'. We rely on explicit
# error handling and signal traps, because:
#   - SIGINT during 'wait' returns exit code 130 (not a real error)
#   - pipefail triggers on grep finding 0 lines, which is normal for empty logs
#   - We want cleanup() to run from EXIT/INT/TERM traps, not from per-line ERR

REPO="/home/seeed/workspace/ros2_bev"
ENGINE="$REPO/models/m4/detection/engines/yolo11n_fp16.engine"
LABELS="$REPO/models/m4/detection/labels/coco.names"
OUT="$REPO/output/m4/4.1"
LOG_DIR="$OUT/visualize_live"

CAMERA_SOURCE="${CAMERA_SOURCE:-auto}"
CAMERA_DEVICE="${CAMERA_DEVICE:-/dev/video0}"
CAMERA_WIDTH="${CAMERA_WIDTH:-1280}"
CAMERA_HEIGHT="${CAMERA_HEIGHT:-720}"
CAMERA_FPS="${CAMERA_FPS:-15}"
DURATION="${DURATION:-0}"                  # 0 = infinite
SCREENSHOT_INTERVAL="${SCREENSHOT_INTERVAL:-2}"
SHOW_WINDOW="${SHOW_WINDOW:-1}"             # 1 = OpenCV 弹窗 (默认), 0 = 回退 PNG 落盘
VIEWER_DISPLAY="${VIEWER_DISPLAY:-:10}"     # X11 显示（VNC 默认 :10）
VIEWER_WIDTH="${VIEWER_WIDTH:-960}"         # 弹窗最大宽度（像素），超过自动缩放

mkdir -p "$OUT" "$LOG_DIR"
rm -f "$LOG_DIR"/*.log

GREEN=$'\e[0;32m'
YELLOW=$'\e[1;33m'
RED=$'\e[0;31m'
CYAN=$'\e[0;36m'
NC=$'\e[0m'
section() { echo "${YELLOW}== $* ==${NC}"; }
ok()      { echo "${GREEN}  [OK] $*${NC}"; }
fail()    { echo "${RED}  [FAIL] $*${NC}"; }
note()    { echo "${CYAN}  [INFO] $*${NC}"; }

# ---- PID tracking ----
YOLO_PID=""
CAM_PID=""
VIEW_PID=""
SNAP_DIR=""
STATUS_PID=""
CLEANED_UP=0

# ---- Helpers ----
show_video_holders() {
  local devs=("$@")
  local any=0
  note "current /dev/video* holders:"
  for d in "${devs[@]}"; do
    [ -e "$d" ] || continue
    local holders
    holders=$(lsof -t "$d" 2>/dev/null | tr '\n' ' ')
    if [ -n "$holders" ]; then
      echo "    $d : PIDs=[$holders]"
      for pid in $holders; do
        local cmd
        cmd=$(ps -p "$pid" -o args= 2>/dev/null | head -c 100)
        echo "        PID $pid : $cmd"
      done
      any=1
    else
      echo "    $d : free"
    fi
  done
  return $any
}

force_release_cameras() {
  local d pids pid cmd
  for d in /dev/video0 /dev/video1 /dev/video2 /dev/video3; do
    [ -e "$d" ] || continue
    pids=$(lsof -t "$d" 2>/dev/null)
    for pid in $pids; do
      cmd=$(ps -p "$pid" -o args= 2>/dev/null || true)
      if echo "$cmd" | grep -qE "csi_camera_publisher|gst-launch|yolo_trt_node|ros2 launch"; then
        note "  SIGKILL PID $pid : ${cmd:0:80}"
        kill -9 "$pid" 2>/dev/null || true
      fi
    done
  done
}

cleanup() {
  if [ "$CLEANED_UP" = "1" ]; then
    return
  fi
  CLEANED_UP=1
  local reason="${1:-exit}"
  section "Cleanup (reason: $reason)"

  # Kill the status loop first
  [ -n "$STATUS_PID" ] && kill "$STATUS_PID" 2>/dev/null
  wait $STATUS_PID 2>/dev/null

  # SIGINT to all children (so python/GStreamer finally blocks run)
  for label_pid in "CAM:$CAM_PID" "YOLO:$YOLO_PID" "VIEW:$VIEW_PID"; do
    local label="${label_pid%%:*}"
    local pid="${label_pid##*:}"
    if [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null; then
      note "SIGINT -> $label PID $pid"
      kill -INT "$pid" 2>/dev/null || true
    fi
  done

  # Wait up to 5s for graceful exit
  local waited=0
  while [ $waited -lt 5 ]; do
    local alive=0
    for pid in "$YOLO_PID" "$CAM_PID" "$VIEW_PID"; do
      [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null && alive=1
    done
    [ $alive -eq 0 ] && break
    sleep 1
    waited=$((waited + 1))
  done

  # Anything still alive → SIGKILL the process group (most reliable)
  for pid in "$YOLO_PID" "$CAM_PID" "$VIEW_PID"; do
    if [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null; then
      note "SIGKILL -> PID $pid (unresponsive)"
      kill -9 "$pid" 2>/dev/null || true
    fi
  done

  # Belt-and-braces: any straggling cv_viewer.py that escaped
  local stragglers
  stragglers=$(pgrep -f "cv_viewer\.py" 2>/dev/null || true)
  if [ -n "$stragglers" ]; then
    note "straggling cv_viewer.py PIDs: $stragglers — SIGKILL"
    kill -9 $stragglers 2>/dev/null || true
  fi

  # Make sure no descendants left
  pkill -9 -P $$ 2>/dev/null || true

  wait 2>/dev/null
  sleep 0.5

  # Verify cameras are released
  echo ""
  note "post-cleanup /dev/video* state:"
  if show_video_holders /dev/video0 /dev/video1 /dev/video2 /dev/video3 > /tmp/holders.txt 2>&1; then
    cat /tmp/holders.txt
    note "some holders remain — forcing release"
    force_release_cameras
    sleep 1
    show_video_holders /dev/video0 /dev/video1 /dev/video2 /dev/video3
  else
    cat /tmp/holders.txt
    ok "all /dev/video* devices are FREE"
  fi

  # Summary
  echo ""
  section "Summary"
  if [ -n "$SNAP_DIR" ] && [ -d "$SNAP_DIR" ]; then
    local total
    total=$(ls "$SNAP_DIR" 2>/dev/null | wc -l)
    echo "  total snapshots : $total"
    echo "  snapshot dir    : $SNAP_DIR"
    [ "$total" -gt 0 ] && \
      echo "  latest          : $SNAP_DIR/$(ls -t "$SNAP_DIR" | head -1)"
  fi
  # Window viewer stats: parse the last "N frames shown (X.X fps)" line
  if [ -f "$LOG_DIR/viewer.log" ]; then
    local last_frame_line last_frames last_fps
    # Match only the periodic 2s "frames shown (X.X fps)" report, not the
    # final DONE: "N frames shown over ..." line (which has no fps).
    last_frame_line=$(grep -E "frames shown \([0-9.]+ fps\)" "$LOG_DIR/viewer.log" 2>/dev/null | tail -1 || true)
    if [ -n "$last_frame_line" ]; then
      last_frames=$(echo "$last_frame_line" | grep -oE "[0-9]+ frames shown" | grep -oE "[0-9]+" || echo "?")
      last_fps=$(echo "$last_frame_line" | grep -oE "[0-9.]+ fps" | grep -oE "[0-9.]+" || echo "?")
      echo "  window frames   : ${last_frames} (last reported: ${last_fps} fps)"
    else
      # Fall back to total count from the final DONE: line
      local done_total
      done_total=$(grep -oE "DONE: [0-9]+ frames" "$LOG_DIR/viewer.log" 2>/dev/null | grep -oE "[0-9]+" || echo "?")
      echo "  window frames   : ${done_total} (no periodic fps report captured)"
    fi
  fi
  echo "  logs            : $LOG_DIR/"
  ok "DONE"
}

trap 'cleanup "SIGINT (Ctrl-C)"; exit 130' INT
trap 'cleanup "SIGTERM"; exit 143' TERM
trap 'cleanup "EXIT"' EXIT
# Note: no ERR trap — we want normal flow to continue.

# ---- 0. Environment ----
section "Step 0/4 — ROS environment"
set +u
source /opt/ros/humble/setup.bash
cd "$REPO/ros2_ws"
source install/setup.bash
set -u
ok "ros2 + bev_detection sourced"
export LD_LIBRARY_PATH="$REPO/ros2_ws/install/bev_detection/lib:/home/seeed/src/opencv-4.14.0-cuda-build/lib:/usr/local/cuda-12.6/lib64:/usr/lib/aarch64-linux-gnu:$LD_LIBRARY_PATH"
ok "LD_LIBRARY_PATH configured"

if [ ! -f "$ENGINE" ]; then
  fail "engine not found: $ENGINE"
  exit 1
fi

echo ""
show_video_holders /dev/video0 /dev/video1 /dev/video2 /dev/video3
note "if any of the above is 'PID xxx : python3 calib_web.py', stop it first:"
note "    pkill -f calib_web.py   (it holds all 4 CSI video nodes)"
echo ""

# ---- 1. yolo_trt_node ----
section "Step 1/4 — yolo_trt_node"
ros2 launch bev_detection yolo.launch.py \
  model_path:="$ENGINE" \
  class_names_path:="$LABELS" \
  publish_debug_image:=true \
  > "$LOG_DIR/yolo.log" 2>&1 &
YOLO_PID=$!
note "yolo_trt_node PID=$YOLO_PID  (log: $LOG_DIR/yolo.log)"

# ---- 2. camera publisher ----
section "Step 2/4 — camera publisher  (source=$CAMERA_SOURCE device=$CAMERA_DEVICE)"
python3 "$REPO/ros2_ws/src/bev_detection/test/csi_camera_publisher.py" \
  --source "$CAMERA_SOURCE" \
  --device "$CAMERA_DEVICE" \
  --width "$CAMERA_WIDTH" \
  --height "$CAMERA_HEIGHT" \
  --fps "$CAMERA_FPS" \
  --timeout 0 \
  --topic /perception/cameras/front/image \
  > "$LOG_DIR/camera.log" 2>&1 &
CAM_PID=$!
note "camera publisher PID=$CAM_PID  (log: $LOG_DIR/camera.log)"

# ---- 3. wait for topics ----
section "Step 3/4 — wait for topics"
for i in 1 2 3 4 5 6 7 8 9 10; do
  if ros2 topic list 2>/dev/null | grep -q "/perception/detections"; then
    ok "topics up after ${i}s"
    break
  fi
  sleep 1
done
ros2 topic list 2>/dev/null | grep -E "perception" | sed 's/^/    /'

# ---- 4. viewer / screenshot ----
section "Step 4/4 — live detection"
if [ "$SHOW_WINDOW" = "1" ]; then
  # OpenCV 弹窗 — 把 yolo debug image 实时显示在 VNC :10 上
  export DISPLAY="$VIEWER_DISPLAY"

  # 探测当前用户的 xauth cookie 并 merge，让 ssh 子进程能连上 VNC X server
  # (cookie 留作可选：vncserver 是 seeed 用户启的，ssh 也是 seeed，所以通常能直接连)
  if [ -z "${XAUTHORITY:-}" ] && [ -f "$HOME/.Xauthority" ]; then
    export XAUTHORITY="$HOME/.Xauthority"
  fi
  if command -v xdpyinfo >/dev/null 2>&1; then
    if DISPLAY="$VIEWER_DISPLAY" xdpyinfo >/dev/null 2>&1; then
      ok "DISPLAY=$VIEWER_DISPLAY reachable (window will appear in VNC)"
    else
      note "DISPLAY=$VIEWER_DISPLAY not reachable yet — window will appear once VNC connects"
    fi
  fi

  SNAP_DIR=""   # no PNG dump when window mode
  # stderr is filtered to drop the Qt font warnings that OpenCV writes on
  # startup (they don't affect rendering). rclpy logs go to stderr; we keep
  # everything else and just strip the Qt noise.
  python3 "$REPO/ros2_ws/src/bev_detection/test/cv_viewer.py" \
    --topic /perception/debug/detection_image \
    --display "$VIEWER_DISPLAY" \
    --width "$VIEWER_WIDTH" \
    --window "YOLO live" \
    --timeout 0 \
    > "$LOG_DIR/viewer.log" \
    2> >(grep --line-buffered -v -E "QFontDatabase|Qt no longer ships fonts|qt\.qpa\.plugin" >> "$LOG_DIR/viewer.log") &
  VIEW_PID=$!
  note "cv_viewer PID=$VIEW_PID  (log: $LOG_DIR/viewer.log)  window='YOLO live'"
else
  # 回退到 PNG 落盘 — 远程 ssh / 不想弹窗时用
  SNAP_DIR="$OUT/live_snaps_$(date +%Y%m%d_%H%M%S)"
  mkdir -p "$SNAP_DIR"
  ok "SHOW_WINDOW=0 → saving PNG every frame to: $SNAP_DIR"
  python3 "$REPO/ros2_ws/src/bev_detection/test/screenshot_saver.py" \
    --out "$SNAP_DIR/snapshot.png" \
    --topic /perception/debug/detection_image \
    --timeout 0 \
    --min-frames 1 \
    --all \
    > "$LOG_DIR/screenshot.log" 2>&1 &
  VIEW_PID=$!
  note "screenshot saver PID=$VIEW_PID  (log: $LOG_DIR/screenshot.log)"
fi

if [ "$DURATION" = "0" ]; then
  note "DURATION=0 → running forever (Ctrl-C to stop)"
else
  note "DURATION=${DURATION}s → auto-stop"
fi

# ---- Periodic live status (foreground, killed by cleanup) ----
START_TS=$(date +%s)
echo ""
echo "  --- live status (updates every 5s, Ctrl-C to stop) ---"

(
  PARENT_PID=$$
  while true; do
    sleep 5
    NOW=$(date +%s)
    ELAPSED=$((NOW - START_TS))
    if [ "$DURATION" != "0" ] && [ $ELAPSED -ge $DURATION ]; then
      note "DURATION=${DURATION}s reached, asking parent to clean up"
      kill -INT $PARENT_PID 2>/dev/null
      exit 0
    fi
    if ! kill -0 "$YOLO_PID" 2>/dev/null || ! kill -0 "$CAM_PID" 2>/dev/null; then
      note "a child process died, asking parent to clean up"
      kill -INT $PARENT_PID 2>/dev/null
      exit 0
    fi
    YOLO_BOXES=$(grep -oE "boxes=[0-9]+" "$LOG_DIR/yolo.log" 2>/dev/null | tail -1)
    YOLO_FPS=$(grep -oE "fps=[0-9.]+" "$LOG_DIR/yolo.log" 2>/dev/null | tail -1)
    CAM_FPS=$(grep -oE "[0-9]+\.[0-9]+ fps\)" "$LOG_DIR/camera.log" 2>/dev/null | tail -1)
    SNAP_COUNT=$(ls "$SNAP_DIR" 2>/dev/null | wc -l)
    echo "  [${ELAPSED}s] yolo=${YOLO_FPS:-?} ${YOLO_BOXES:-?}  cam=${CAM_FPS:-?}  snaps=${SNAP_COUNT:-0}"
  done
) &
STATUS_PID=$!

# Wait for any child process to terminate (or status loop itself)
# Using 'wait -n' so we react to the first exit and trigger cleanup
wait -n $STATUS_PID $YOLO_PID $CAM_PID $VIEW_PID 2>/dev/null
note "a child terminated, running cleanup"
cleanup "child terminated"
exit 0
