#!/usr/bin/env bash
# scripts/m4/run_m4_4_demo.sh — one-click launch for M4.4 (FoundationPose 6D pose).
#
# Pipeline (when Orbbec Gemini 2 is plugged in):
#   orbbec_camera_node (driver, owned)
#     → /perception/cameras/front/{image, depth, camera_info}
#   object_mask_node (P0 single-object init helper)
#     → /perception/object_mask
#   foundationpose_node
#     → /perception/object_pose          (PoseStamped, primary)
#     → /perception/object_poses_3d      (Detection3DArray, optional)
#     → /tf                              (camera_front → <frame_id>)
#   pose_visualizer
#     → /perception/demo/m4_4            (BGR overlay)
#   m4_web_demo_server (WebRTC hub page)
#
# Hard requirements (from plan):
#   * Source scripts/m4/lib/m4_demo_lib.sh
#     (PID ownership / PGID cleanup / metadata / graceful shutdown)
#   * Do NOT call kill / pkill directly here
#   * Do NOT bypass PGID cleanup
#
# Smoke-test path: when no Orbbec device is detected, the camera launch
# still goes through but the visualizer will see no frames; the user can
# pair this with CAMERA_SOURCE=test to use csi_camera_publisher.

# --- module anchors: derived from this script's own location, never hardcoded ---
# M4_ROOT is this M04 module; WS_ROOT is the repository root.
M4_ROOT="${M4_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)}"
WS_ROOT="${WS_ROOT:-$(cd "$M4_ROOT/../.." && pwd)}"
export M4_ROOT WS_ROOT

set -u

LIB_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/lib"
# shellcheck disable=SC1091
source "$LIB_DIR/m4_demo_lib.sh"

DEMO_NAME="4.4"
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
: "${ORBBEC_SERIAL:=}"
: "${ORBBEC_USB_PORT:=}"
: "${MESH_NPZ_PATH:=$M4_ROOT/models/m4/pose/processed/cup.npz}"
: "${MESH_OBJ:=$M4_ROOT/models/m4/pose/obj_models/cup.obj}"
: "${FRAME_ID:=cup}"

export CAMERA_SOURCE CAMERA_DEVICE CAMERA_WIDTH CAMERA_HEIGHT CAMERA_FPS \
       VIEWER DURATION ORBBEC_SERIAL ORBBEC_USB_PORT \
       MESH_NPZ_PATH MESH_OBJ FRAME_ID

m4_parse_cli "$DEMO_NAME" "$@"
m4_setup_env

m4_log_open

# ---- preflight ----
M4_PREFLIGHT_PATHS="$WS_ROOT/install/bev_pose/lib/bev_pose/foundationpose_node \
  $WS_ROOT/install/bev_pose/lib/bev_pose/object_mask_node \
  $WS_ROOT/install/m4_demo_bringup/lib/m4_demo_bringup/pose_visualizer \
  $MESH_NPZ_PATH"
m4_preflight
m4_section "Preflight OK"

# ---- camera resolution ----
# For M4.4 the "camera" is Orbbec. We probe for the device; if missing we
# log a warning and continue (the driver will retry / the user can
# re-export the device). Unlike the bare csi mode we do not own the
# camera cycle here.
M4_CAMERA_MODE="orbbec"
if [ -n "${ORBBEC_SERIAL}" ] || [ -n "${ORBBEC_USB_PORT}" ]; then
    M4_CAMERA_OWNED=true
    m4_ok "[M4.4] Camera source: orbbec (serial=${ORBBEC_SERIAL:-auto}, "
        "usb_port=${ORBBEC_USB_PORT:-auto})"
    m4_ok "[M4.4] Camera ownership: demo"
elif lsusb 2>/dev/null | grep -qi orbbec; then
    M4_CAMERA_OWNED=true
    m4_ok "[M4.4] Camera source: orbbec (auto-discovered)"
    m4_ok "[M4.4] Camera ownership: demo"
else
    # No Orbbec visible. Continue with auto-select in the launch file;
    # demo will still start, visualizer will idle, no frames registered.
    M4_CAMERA_OWNED=false
    m4_warn "[M4.4] No Orbbec device on lsusb — driver may retry on connect"
fi

if [ "$M4_DRY_RUN" = "1" ]; then
    cat <<EOF
[DRY-RUN] $DEMO_NAME would:
  1. launch orbbec_camera_node (serial=${ORBBEC_SERIAL:-auto} usb=${ORBBEC_USB_PORT:-auto})
  2. launch object_mask_node + foundationpose_node (bev_pose/launch/m4_pose_estimation.launch.py)
  3. launch pose_visualizer (m4_demo_bringup)
  4. aggregate m4_4_demo.launch.py
  5. duplicate-probe: /perception/object_pose, /perception/object_poses_3d
  6. spawn via setsid: ros2 launch m4_demo_bringup m4_4_demo.launch.py
  7. staged readiness gate (orbbec→/image→/depth→camera_info→/object_mask→/object_pose→demo)
  8. start web server (m4_4) on port \$M4_WEB_PORT
  9. start status reporter; wait for Ctrl-C
 10. on EXIT/INT/TERM: SIGINT to launch PGID, wait 5s, SIGKILL

[DRY-RUN] Camera source: orbbec (owned=$M4_CAMERA_OWNED)
[DRY-RUN] mesh_npz_path: $MESH_NPZ_PATH
[DRY-RUN] frame_id:     $FRAME_ID
[DRY-RUN] Viewer:       $VIEWER  DURATION=$DURATION
EOF
    exit 0
fi

m4_install_traps
m4_algorithm_duplicate_probe "foundationpose_node" \
    /perception/object_pose /perception/object_poses_3d

# ---- launch the demo ----
m4_section "Launching m4_4_demo.launch.py (orbbec + FoundationPose)"
LAUNCH_ARGS=(
    "ros2 launch m4_demo_bringup m4_4_demo.launch.py"
    "mesh_npz_path:=$MESH_NPZ_PATH"
    "frame_id:=$FRAME_ID"
)
if [ -n "$ORBBEC_SERIAL" ]; then
    LAUNCH_ARGS+=("orbbec_serial_number:=$ORBBEC_SERIAL")
fi
if [ -n "$ORBBEC_USB_PORT" ]; then
    LAUNCH_ARGS+=("orbbec_usb_port:=$ORBBEC_USB_PORT")
fi

m4_launch_ros "${LAUNCH_ARGS[@]}"
m4_record_meta

# ---- staged readiness gate ----
m4_section "Readiness gate"

# Stage 1: Orbbec RGB
m4_wait_for_stage "/perception/cameras/front/image"  8.0 "orbbec rgb image" || true
# Stage 2: Orbbec Depth
m4_wait_for_stage "/perception/cameras/front/depth"  8.0 "orbbec depth" \
    "sensor_msgs.msg.Image" || true
# Stage 3: CameraInfo
m4_wait_for_stage "/perception/cameras/front/camera_info"  5.0 "orbbec camera_info" "sensor_msgs.msg.CameraInfo" || true
# Stage 4: object_mask (P0 helper publishes this)
m4_wait_for_stage "/perception/object_mask"        5.0 "object mask" \
    "sensor_msgs.msg.Image" || true
# Stage 5: object_pose (primary output)
m4_wait_for_stage "/perception/object_pose"        8.0 "object pose (PoseStamped)" \
    "geometry_msgs.msg.PoseStamped" || true
# Stage 6: demo overlay
m4_wait_for_stage "/perception/demo/m4_4"          5.0 "demo overlay m4_4" \
    "sensor_msgs.msg.Image" || true

# ---- web server (in its own PGID; cleaned by m4_cleanup) ----
if [ "$M4_NO_GUI" = "1" ] || [ "$M4_VIEWER" = "none" ]; then
    m4_note "viewer + web: disabled (--no-gui or --viewer=none)"
else
    m4_section "Web server"
    m4_launch_web_server "m4_4" "4" "/perception/demo/m4_4" || \
        m4_warn "web server failed to start; continuing without preview"
    if [ "${M4_WEB_PID:-0}" != "0" ]; then
        m4_report_web_url "4" || m4_warn "web server health gate not satisfied"
    fi
fi

# ---- status reporter ----
m4_start_status_reporter

m4_note "M4.4 demo is live. Press Ctrl-C to stop."
m4_note "WebRTC preview: http://\$(m4_jetson_ip):$M4_WEB_PORT/m4/4"
wait "$M4_LAUNCH_PID" 2>/dev/null
rc=$?
if [ $rc -ne 0 ]; then
    m4_report_launch_failure
fi
m4_note "launch process exited (rc=$rc); cleanup follows via EXIT trap"
exit 0
