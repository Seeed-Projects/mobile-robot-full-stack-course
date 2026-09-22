#!/usr/bin/env bash
# scripts/m4/run_m4_2_demo.sh — one-click student entry for M4.2 (ByteTrack).
#
# Pipeline (CAMERA_SOURCE=gmsl — course hardware):
#   nvargus-daemon (root, external)
#     → /camera/front/image_raw/compressed
#   camera_sync_node (may already be running externally)
#     → /bev/frameset
#   camera_adapter_node  ← started by this demo when CAMERA_SOURCE=gmsl
#     → /perception/cameras/front/image
#   yolo_trt_node
#     → /perception/detections
#   tracking_node
#     → /perception/tracks
#   tracking_visualizer (from tracking_demo.launch.py)
#     → /perception/demo/m4_2

# --- module anchors: derived from this script's own location, never hardcoded ---
# M4_ROOT is this M04 module; WS_ROOT is the repository root.
M4_ROOT="${M4_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)}"
WS_ROOT="${WS_ROOT:-$(cd "$M4_ROOT/../.." && pwd)}"
export M4_ROOT WS_ROOT

set -u

LIB_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/lib"
# shellcheck disable=SC1091
source "$LIB_DIR/m4_demo_lib.sh"

DEMO_NAME="4.2"
m4_lib_init "$DEMO_NAME"

# ---- CLI / env overrides ----
: "${CAMERA_SOURCE:=auto}"
: "${CAMERA_DEVICE:=/dev/video0}"
# /dev/video0 enumerates 1920x1536, 1920x1080 and 3840x2160, all at 30 fps;
# the course material specifies 1080p, so every M4 demo shares that default.
: "${CAMERA_WIDTH:=1920}"
: "${CAMERA_HEIGHT:=1080}"
: "${CAMERA_FPS:=30}"
: "${VIEWER:=auto}"
: "${DURATION:=0}"

export CAMERA_SOURCE CAMERA_DEVICE CAMERA_WIDTH CAMERA_HEIGHT CAMERA_FPS VIEWER DURATION
m4_parse_cli "$DEMO_NAME" "$@"
m4_setup_env

m4_log_open

# ---- preflight ----
M4_PREFLIGHT_PATHS="$M4_ROOT/models/m4/detection/engines/yolo11n_fp16.engine \
  $WS_ROOT/install/bev_detection/lib/bev_detection/yolo_trt_node \
  $WS_ROOT/install/bev_detection/lib/bev_detection/camera_adapter_node \
  $WS_ROOT/install/bev_tracking/lib/bev_tracking/tracking_node \
  $WS_ROOT/install/bev_tracking/lib/bev_tracking/tracking_visualizer"
m4_preflight
m4_section "Preflight OK"

# Resolve camera source early so dry-run can report the resolved mode.
m4_camera_probe_and_choose

if [ "$M4_DRY_RUN" = "1" ]; then
    cat <<EOF
[DRY-RUN] $DEMO_NAME would:
  1. camera source: ${M4_CAMERA_SOURCE:-auto} → owned=$M4_CAMERA_OWNED mode=$M4_CAMERA_MODE
  2. (if owned && mode=csi/usb/test) spawn csi_camera_publisher
  3. (if owned && mode=gmsl) launch includes camera_adapter_node
  4. duplicate-probe: /perception/detections, /perception/tracks
  5. spawn via setsid: ros2 launch m4_demo_bringup m4_2_demo.launch.py
  6. staged readiness gate (gmsl: FrameSet→image→detections→tracks→overlay)
  7. spawn viewer ($VIEWER) on /perception/demo/m4_2
  8. start status reporter; wait for Ctrl-C
  9. on EXIT/INT/TERM: SIGINT to launch PGID, wait 5s, SIGKILL

[DRY-RUN] Camera source: $M4_CAMERA_SOURCE  resolved: $M4_CAMERA_MODE
[DRY-RUN] Camera owned: $M4_CAMERA_OWNED
[DRY-RUN] Viewer: $VIEWER  DURATION=$DURATION
EOF
    exit 0
fi

m4_install_traps
m4_algorithm_duplicate_probe "yolo_trt_node,tracking_node" \
    /perception/detections /perception/tracks

# ---- camera bringup (non-gmsl owned modes only) ----
if [ "$M4_CAMERA_OWNED" = "true" ] && [ "$M4_CAMERA_MODE" != "gmsl" ]; then
    m4_section "Camera ($M4_CAMERA_MODE) starting"
    m4_spawn_camera_publisher "$M4_CAMERA_MODE"
    sleep 1
fi

# ---- launch the demo ----
m4_section "Launching m4_2_demo.launch.py (camera_mode=$M4_CAMERA_MODE)"
m4_launch_ros ros2 launch m4_demo_bringup m4_2_demo.launch.py \
    camera_source:="$M4_CAMERA_MODE"
m4_record_meta

# ---- staged readiness gate ----
m4_section "Readiness gate"

# Stage 1: GMSL→FrameSet (only for gmsl mode)
if [ "$M4_CAMERA_MODE" = "gmsl" ]; then
    m4_wait_for_stage "/bev/frameset"         5.0 "GMSL→FrameSet"   || true
fi

# Stage 2: camera front image
m4_wait_for_stage "/perception/cameras/front/image" 5.0 "camera front image" || true

# Stage 3: YOLO detections
m4_wait_for_stage "/perception/detections"     8.0 "YOLO detections"    || true

# Stage 4: tracks
m4_wait_for_stage "/perception/tracks"         5.0 "tracking tracks"     || true

# Stage 5: demo overlay
m4_wait_for_stage "/perception/demo/m4_2"     5.0 "demo overlay"       || true

# ---- optional viewer ----
# M4 web preview replaces the legacy local GUI. The bash supervisor
# owns the web server in a separate PGID via m4_launch_web_server.
# --no-gui / --viewer=none still suppress it (matches old behaviour).
if [ "$M4_NO_GUI" = "1" ] || [ "$M4_VIEWER" = "none" ]; then
    m4_note "viewer (local GUI): disabled (--no-gui or --viewer=none)"
    m4_note "web server: disabled (same flag)"
else
    m4_section "Web server"
    m4_launch_web_server "m4_2" "2" "/perception/demo/m4_2" || \
        m4_warn "web server failed to start; continuing without preview"
    if [ "$M4_WEB_PID" != "0" ]; then
        m4_report_web_url "2" || m4_warn "web server health gate not satisfied"
    fi
fi

# ---- status reporter ----
m4_start_status_reporter

m4_note "demo is live. Press Ctrl-C to stop."
wait "$M4_LAUNCH_PID" 2>/dev/null
rc=$?
if [ $rc -ne 0 ]; then
    m4_report_launch_failure
fi
m4_note "launch process exited (rc=$rc); cleanup follows via EXIT trap"
exit 0
