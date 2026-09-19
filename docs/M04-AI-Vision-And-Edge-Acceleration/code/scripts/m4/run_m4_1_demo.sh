#!/usr/bin/env bash
# scripts/m4/run_m4_1_demo.sh — one-click student entry for M4.1 (YOLO11n).
#
# Pipeline (CAMERA_SOURCE=csi — GMSL cameras via V4L2):
#   /dev/video0 (GMSL/BSP driver exposes as V4L2 device)
#     → csi_camera_publisher.py (v4l2src)  ← started by this demo when CAMERA_SOURCE=csi
#     → /perception/cameras/front/image
#   yolo_trt_node
#     → /perception/detections
#   (upstream debug image remapped to) → /perception/demo/m4_1

set -u

LIB_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/lib"
# shellcheck disable=SC1091
source "$LIB_DIR/m4_demo_lib.sh"

DEMO_NAME="4.1"
m4_lib_init "$DEMO_NAME"

# ---- CLI defaults ----
: "${CAMERA_SOURCE:=csi}"
: "${CAMERA_DEVICE:=/dev/video0}"
: "${CAMERA_WIDTH:=1920}"
: "${CAMERA_HEIGHT:=1536}"
: "${CAMERA_FPS:=30}"
: "${VIEWER:=auto}"
: "${DURATION:=0}"

export CAMERA_SOURCE CAMERA_DEVICE CAMERA_WIDTH CAMERA_HEIGHT CAMERA_FPS VIEWER DURATION
m4_parse_cli "$DEMO_NAME" "$@"
m4_setup_env

m4_log_open

# Resolve camera source BEFORE preflight and dry-run so the report is accurate.
# This calls wait_for_first_message to probe /image then /bev/frameset.
# It sets M4_CAMERA_MODE and M4_CAMERA_OWNED globals.
m4_camera_probe_and_choose

# ---- preflight ----
M4_PREFLIGHT_PATHS="$REPO/models/m4/detection/engines/yolo11n_fp16.engine \
  $REPO/ros2_ws/install/bev_detection/lib/bev_detection/yolo_trt_node"
m4_preflight
m4_section "Preflight OK"

if [ "$M4_DRY_RUN" = "1" ]; then
    cat <<EOF
[DRY-RUN] $DEMO_NAME would:
  1. camera source: $M4_CAMERA_SOURCE → owned=$M4_CAMERA_OWNED mode=$M4_CAMERA_MODE
  2. (if owned && mode=csi/usb/test) spawn csi_camera_publisher
  3. (if owned && mode=gmsl) launch includes camera_adapter_node
  4. duplicate-probe: /perception/detections
  5. spawn via setsid: ros2 launch m4_demo_bringup m4_1_demo.launch.py
  6. staged readiness gate (gmsl: FrameSet→image→detections→overlay)
  7. spawn viewer ($VIEWER) on /perception/demo/m4_1
  8. start status reporter; wait for Ctrl-C
  9. on EXIT/INT/TERM: SIGINT to launch PGID, wait 5s, SIGKILL

[DRY-RUN] Camera source: $M4_CAMERA_SOURCE  resolved: $M4_CAMERA_MODE
[DRY-RUN] Camera owned: $M4_CAMERA_OWNED
[DRY-RUN] Viewer: $VIEWER  DURATION=$DURATION
EOF
    exit 0
fi

m4_install_traps
m4_algorithm_duplicate_probe "yolo_trt_node" /perception/detections

# ---- camera bringup: csi/usb/test only; gmsl starts camera_adapter_node via launch ----
if [ "$M4_CAMERA_OWNED" = "true" ] && [ "$M4_CAMERA_MODE" != "gmsl" ]; then
    m4_section "Camera ($M4_CAMERA_MODE) starting"
    case "$M4_CAMERA_MODE" in
        csi|usb)
            setsid python3 "$REPO/ros2_ws/src/bev_detection/test/csi_camera_publisher.py" \
                --source "$M4_CAMERA_MODE" \
                --device "$M4_CAMERA_DEVICE" \
                --width  "$M4_CAMERA_WIDTH" \
                --height "$M4_CAMERA_HEIGHT" \
                --fps    "$M4_CAMERA_FPS" \
                --timeout 0 \
                --topic /perception/cameras/front/image \
                > "$M4_LOG_DIR/camera.log" 2>&1 &
            ;;
        test)
            setsid python3 "$REPO/ros2_ws/src/bev_detection/test/csi_camera_publisher.py" \
                --source test \
                --width  "$M4_CAMERA_WIDTH" \
                --height "$M4_CAMERA_HEIGHT" \
                --fps    "$M4_CAMERA_FPS" \
                --timeout 0 \
                --topic /perception/cameras/front/image \
                > "$M4_LOG_DIR/camera.log" 2>&1 &
            ;;
    esac
    sleep 1
    m4_note "camera publisher spawned (see $M4_LOG_DIR/camera.log)"
fi

# ---- launch ----
m4_section "Launching m4_1_demo.launch.py (camera_mode=$M4_CAMERA_MODE)"
m4_launch_ros ros2 launch m4_demo_bringup m4_1_demo.launch.py \
    camera_source:="$M4_CAMERA_MODE"
m4_record_meta

# ---- staged readiness gate ----
m4_section "Readiness gate"

m4_wait_for_stage "/perception/cameras/front/image"     5.0 "camera front image" || true
m4_wait_for_stage "/perception/detections"             8.0 "YOLO detections"    || true
m4_wait_for_stage "/perception/demo/m4_1"              5.0 "demo overlay"       || true

# ---- viewer ----
# M4 web preview replaces the legacy local GUI. The bash supervisor
# owns the web server in a separate PGID via m4_launch_web_server.
# --no-gui / --viewer=none still suppress it (matches old behaviour).
if [ "$M4_NO_GUI" = "1" ] || [ "$M4_VIEWER" = "none" ]; then
    m4_note "viewer (local GUI): disabled (--no-gui or --viewer=none)"
    m4_note "web server: disabled (same flag)"
else
    m4_section "Web server"
    m4_launch_web_server "m4_1" "1" "/perception/demo/m4_1" || \
        m4_warn "web server failed to start; continuing without preview"
    if [ "$M4_WEB_PID" != "0" ]; then
        m4_report_web_url "1" || m4_warn "web server health gate not satisfied"
    fi
fi

# ---- status reporter ----
m4_start_status_reporter

m4_note "demo is live. Press Ctrl-C to stop."
wait "$M4_LAUNCH_PID" 2>/dev/null
rc=$?
[ $rc -ne 0 ] && m4_report_launch_failure
m4_note "launch process exited (rc=$rc); cleanup follows via EXIT trap"
exit 0
