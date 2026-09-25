#!/usr/bin/env bash
set -euo pipefail

# official: NVIDIA FP32/252 graph; adapted: project-owned FP32/42 graph.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
M4_CODE_ROOT="${M4_CODE_ROOT:-$(cd "$SCRIPT_DIR/../.." && pwd)}"
M44_MODE="${M44_MODE:-official}"
CONTAINER="${ISAAC_ROS_CONTAINER:-m4-isaacros-foundationpose}"
ASSET_ROOT="${ISAAC_ROS_ASSET_ROOT:-/workspaces/isaac_ros-dev/isaac_ros_assets/isaac_ros_foundationpose}"
MODEL_ROOT="${FOUNDATIONPOSE_MODEL_ROOT:-/workspaces/isaac_ros-dev/isaac_ros_assets/models/foundationpose}"
TRTEXEC="${TRTEXEC:-/usr/src/tensorrt/bin/trtexec}"
HOST_TRTEXEC="${HOST_TRTEXEC:-/usr/src/tensorrt/bin/trtexec}"
HOST_MODEL_ROOT="${HOST_MODEL_ROOT:-${ISAAC_ROS_HOST_ASSET_ROOT:+$ISAAC_ROS_HOST_ASSET_ROOT/models/foundationpose}}"
RTDETR_ENGINE="${RTDETR_ENGINE:-/workspaces/isaac_ros-dev/isaac_ros_assets/models/synthetica_detr/sdetr_grasp.plan}"
POSE_TOPIC="${POSE_TOPIC:-/output}"
M44_POSE_TIMEOUT="${M44_POSE_TIMEOUT:-240}"
# 0 exits after the first verified pose; N>0 keeps the launch and looping bag
# alive for N extra seconds; -1 holds them until SIGTERM (web-hub mode).
M44_VIEW_SECONDS="${M44_VIEW_SECONDS:-0}"
M44_RUN_TOKEN="${M44_RUN_TOKEN:-quickstart-$$}"
M44_SOURCE_ROOT="/tmp/m44-course-source-${M44_RUN_TOKEN}"

case "$M44_MODE" in
  official|adapted) ;;
  *) echo "ERROR: M44_MODE must be official or adapted." >&2; exit 2 ;;
esac
case "$M44_VIEW_SECONDS" in
  -1) ;;
  ''|*[!0-9]*) echo 'ERROR: M44_VIEW_SECONDS must be -1 or a nonnegative integer.' >&2; exit 2 ;;
esac
case "$M44_RUN_TOKEN" in
  ''|*[!a-zA-Z0-9_-]*) echo 'ERROR: invalid M44_RUN_TOKEN.' >&2; exit 2 ;;
esac
if ! command -v docker >/dev/null || ! docker inspect "$CONTAINER" >/dev/null 2>&1; then
  echo "ERROR: Isaac ROS container '$CONTAINER' is not available." >&2
  exit 2
fi
for required_source in \
  "$M4_CODE_ROOT/scripts/m4/verify_m4_4_pose.py" \
  "$M4_CODE_ROOT/4.4-isaac-ros-foundationpose/config/foundationpose_42.yaml" \
  "$M4_CODE_ROOT/4.4-isaac-ros-foundationpose/launch/m4_4_foundationpose_42.launch.py"; do
  if [ ! -f "$required_source" ]; then
    echo "ERROR: course source file is missing: $required_source" >&2
    exit 4
  fi
done
docker exec "$CONTAINER" rm -rf "$M44_SOURCE_ROOT"
docker exec "$CONTAINER" mkdir -p "$M44_SOURCE_ROOT"
tar -C "$M4_CODE_ROOT" -cf - \
  scripts/m4/verify_m4_4_pose.py \
  4.4-isaac-ros-foundationpose/config/foundationpose_42.yaml \
  4.4-isaac-ros-foundationpose/launch/m4_4_foundationpose_42.launch.py \
  | docker exec -i "$CONTAINER" tar -xf - -C "$M44_SOURCE_ROOT"
for active_node in '/install/bev_detection/lib/bev_detection/yolo_trt_node' '/install/bev_segmentation/lib/bev_segmentation/segmentation_node'; do
  if pgrep -f "$active_node" >/dev/null; then
    echo "ERROR: shared Hub inference node '$active_node' is active. Stop its run through the owning Hub before using the GPU." >&2
    exit 3
  fi
done
docker exec "$CONTAINER" bash -lc "test -x '$TRTEXEC' && test -f '$MODEL_ROOT/refine_model.onnx' && test -f '$MODEL_ROOT/score_model.onnx' && test -f '$ASSET_ROOT/quickstart_interface_specs.json' && test -f '$ASSET_ROOT/quickstart.bag/metadata.yaml' && test -s '$RTDETR_ENGINE' && test -f '$M44_SOURCE_ROOT/scripts/m4/verify_m4_4_pose.py'" || {
  echo 'ERROR: required FoundationPose assets are missing.' >&2
  exit 4
}
if [ "$M44_MODE" = adapted ]; then
  docker exec "$CONTAINER" test -f "$M44_SOURCE_ROOT/4.4-isaac-ros-foundationpose/config/foundationpose_42.yaml" || exit 4
  docker exec "$CONTAINER" test -f "$M44_SOURCE_ROOT/4.4-isaac-ros-foundationpose/launch/m4_4_foundationpose_42.launch.py" || exit 4
fi

# docker exec may leave the container-side shell running if the desktop
# terminal closes. This state file names only the processes started by this run.
stop_owned_run() {
  docker exec -e M44_RUN_TOKEN="$M44_RUN_TOKEN" "$CONTAINER" bash -lc '
    state_file="/tmp/m44-run-$M44_RUN_TOKEN.state"
    test -f "$state_file" || exit 0
    mapfile -t owned < "$state_file"
    inner_pid="${owned[0]:-}"
    launch_pid="${owned[1]:-}"
    bag_pid="${owned[2]:-}"
    for pid in "$bag_pid" "$launch_pid"; do
      if [[ "$pid" =~ ^[0-9]+$ ]]; then kill -TERM -- "-$pid" 2>/dev/null || true; fi
    done
    if [[ "$inner_pid" =~ ^[0-9]+$ ]]; then
      pkill -TERM -P "$inner_pid" 2>/dev/null || true
      kill -TERM "$inner_pid" 2>/dev/null || true
    fi
    sleep 2
    for pid in "$bag_pid" "$launch_pid"; do
      if [[ "$pid" =~ ^[0-9]+$ ]] && kill -0 -- "-$pid" 2>/dev/null; then
        kill -KILL -- "-$pid" 2>/dev/null || true
      fi
    done
    rm -f "$state_file"
  ' >/dev/null 2>&1 || true
}
trap stop_owned_run EXIT
trap 'exit 129' HUP
trap 'exit 130' INT
trap 'exit 143' TERM

# On this Jetson the official 252-profile build fits when trtexec runs on the
# host; the same TensorRT 10.3 build exhausted device memory in the container.
if [ "$M44_MODE" = official ] && [ ! -s "${HOST_MODEL_ROOT:-}/score_trt_engine.plan" ]; then
  if [ -z "$HOST_MODEL_ROOT" ]; then
    echo 'ERROR: set HOST_MODEL_ROOT or ISAAC_ROS_HOST_ASSET_ROOT before building the official score engine.' >&2
    exit 4
  fi
  if [ ! -x "$HOST_TRTEXEC" ] || [ ! -f "$HOST_MODEL_ROOT/score_model.onnx" ] || ! sudo -n true; then
    echo 'ERROR: host TensorRT, score ONNX, or passwordless sudo is unavailable.' >&2
    exit 4
  fi
  host_build_log="$(mktemp /tmp/m44-score-host.XXXXXX.log)"
  if ! (cd "$HOST_MODEL_ROOT" && sudo -n "$HOST_TRTEXEC" \
    --onnx=score_model.onnx --saveEngine=score_trt_engine.plan \
    --minShapes=input1:1x160x160x6,input2:1x160x160x6 \
    --optShapes=input1:1x160x160x6,input2:1x160x160x6 \
    --maxShapes=input1:252x160x160x6,input2:252x160x160x6 \
    --skipInference) >"$host_build_log" 2>&1; then
    sudo -n cp "$host_build_log" "$HOST_MODEL_ROOT/score_trtexec_252_host_fp32.log"
    tail -n 40 "$host_build_log" >&2
    rm -f "$host_build_log"
    exit 5
  fi
  sudo -n cp "$host_build_log" "$HOST_MODEL_ROOT/score_trtexec_252_host_fp32.log"
  rm -f "$host_build_log"
fi

docker exec -i -e M44_MODE="$M44_MODE" -e ASSET_ROOT="$ASSET_ROOT" \
  -e MODEL_ROOT="$MODEL_ROOT" -e TRTEXEC="$TRTEXEC" \
  -e RTDETR_ENGINE="$RTDETR_ENGINE" -e POSE_TOPIC="$POSE_TOPIC" \
  -e M44_POSE_TIMEOUT="$M44_POSE_TIMEOUT" \
  -e M44_VIEW_SECONDS="$M44_VIEW_SECONDS" \
  -e M44_RUN_TOKEN="$M44_RUN_TOKEN" \
  -e M44_SOURCE_ROOT="$M44_SOURCE_ROOT" "$CONTAINER" bash -s <<'INNER'
set -Eeo pipefail
source /opt/ros/humble/setup.bash
set -u
log_root=/workspaces/isaac_ros-dev/isaac_ros_assets/m4_4_logs
mkdir -p "$log_root"
run_id="$(date +%Y%m%d-%H%M%S)-$$-$M44_MODE"
launch_log="$log_root/$run_id-launch.log"
bag_log="$log_root/$run_id-bag.log"
pose_log="$log_root/$run_id-pose.json"
launch_pid=
bag_pid=
state_file="/tmp/m44-run-$M44_RUN_TOKEN.state"
write_state() {
  printf '%s\n%s\n%s\n' "$$" "$launch_pid" "$bag_pid" >"$state_file"
}
write_state
cleanup() {
  for pid in "$bag_pid" "$launch_pid"; do
    if [ -n "$pid" ]; then kill -TERM -- "-$pid" 2>/dev/null || true; fi
  done
  sleep 2
  for pid in "$bag_pid" "$launch_pid"; do
    if [ -n "$pid" ] && kill -0 -- "-$pid" 2>/dev/null; then
      kill -KILL -- "-$pid" 2>/dev/null || true
    fi
  done
  wait 2>/dev/null || true
  rm -f "$state_file"
}
trap cleanup EXIT
trap 'exit 129' HUP
trap 'exit 130' INT
trap 'exit 143' TERM

cd "$MODEL_ROOT"
if [ ! -s refine_trt_engine.plan ]; then
  "$TRTEXEC" --onnx=refine_model.onnx --saveEngine=refine_trt_engine.plan \
    --minShapes=input1:1x160x160x6,input2:1x160x160x6 \
    --optShapes=input1:1x160x160x6,input2:1x160x160x6 \
    --maxShapes=input1:42x160x160x6,input2:42x160x160x6 \
    >"$log_root/$run_id-refine-build.log" 2>&1
fi
if [ "$M44_MODE" = official ]; then
  score_engine="$MODEL_ROOT/score_trt_engine.plan"
  max_batch=252
  launch_target=(isaac_ros_examples isaac_ros_examples.launch.py)
else
  score_engine="$MODEL_ROOT/score_trt_engine_42_fp32.plan"
  max_batch=42
  launch_target=("$M44_SOURCE_ROOT/4.4-isaac-ros-foundationpose/launch/m4_4_foundationpose_42.launch.py")
fi
if [ ! -s "$score_engine" ]; then
  if [ "$M44_MODE" = official ]; then
    echo "ERROR: host-built 252-profile score engine is not visible at $score_engine" >&2
    exit 5
  fi
  score_log="$log_root/$run_id-score-build.log"
  if ! "$TRTEXEC" --onnx=score_model.onnx --saveEngine="$score_engine" \
    --minShapes=input1:1x160x160x6,input2:1x160x160x6 \
    --optShapes=input1:1x160x160x6,input2:1x160x160x6 \
    --maxShapes=input1:${max_batch}x160x160x6,input2:${max_batch}x160x160x6 \
    >"$score_log" 2>&1; then
    tail -n 40 "$score_log" >&2
    exit 5
  fi
fi
test -s "$score_engine"
echo "M4.4 mode=$M44_MODE score_max_batch=$max_batch score_engine=$score_engine"

launch_args=(
  interface_specs_file:="$ASSET_ROOT/quickstart_interface_specs.json"
  mesh_file_path:="$ASSET_ROOT/Mustard/textured_simple.obj"
  texture_path:="$ASSET_ROOT/Mustard/texture_map.png"
  refine_engine_file_path:="$MODEL_ROOT/refine_trt_engine.plan"
  score_engine_file_path:="$score_engine"
  rt_detr_engine_file_path:="$RTDETR_ENGINE"
)
if [ "$M44_MODE" = official ]; then
  launch_args=(launch_fragments:=foundationpose "${launch_args[@]}")
else
  launch_args+=(
    configuration_file:="$M44_SOURCE_ROOT/4.4-isaac-ros-foundationpose/config/foundationpose_42.yaml"
    input_images_drop_freq:=0
  )
fi
setsid ros2 launch "${launch_target[@]}" "${launch_args[@]}" >"$launch_log" 2>&1 &
launch_pid=$!
write_state
sleep 20
if ! kill -0 "$launch_pid" 2>/dev/null; then
  tail -n 100 "$launch_log" >&2
  exit 7
fi
setsid ros2 bag play "$ASSET_ROOT/quickstart.bag" --loop >"$bag_log" 2>&1 &
bag_pid=$!
write_state
if ! python3 "$M44_SOURCE_ROOT/scripts/m4/verify_m4_4_pose.py" \
    --topic "$POSE_TOPIC" --timeout "$M44_POSE_TIMEOUT" | tee "$pose_log"; then
  echo "ERROR: no valid Detection3DArray pose; logs: $launch_log $bag_log" >&2
  tail -n 100 "$launch_log" >&2
  tail -n 30 "$bag_log" >&2
  exit 8
fi
echo "M4.4 logs: $launch_log $bag_log $pose_log"
if [ "$M44_VIEW_SECONDS" -eq -1 ]; then
  echo "M4.4 hub hold: launch and looping bag stay up until SIGTERM (token=$M44_RUN_TOKEN)"
  while :; do sleep 3600; done
elif [ "$M44_VIEW_SECONDS" -gt 0 ]; then
  echo "M4.4 viewer hold: keeping launch and looping bag for ${M44_VIEW_SECONDS}s"
  sleep "$M44_VIEW_SECONDS"
fi
INNER
