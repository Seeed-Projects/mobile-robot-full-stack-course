#!/usr/bin/env bash
set -euo pipefail

CONTAINER="${ISAAC_ROS_CONTAINER:-m4-isaacros-foundationpose}"
ASSET_ROOT="${ISAAC_ROS_ASSET_ROOT:-/workspaces/isaac_ros-dev/isaac_ros_assets/isaac_ros_foundationpose}"
MODEL_ROOT="${FOUNDATIONPOSE_MODEL_ROOT:-/workspaces/isaac_ros-dev/isaac_ros_assets/models/foundationpose}"
TRTEXEC="${TRTEXEC:-/usr/src/tensorrt/bin/trtexec}"
RTDETR_ENGINE="${RTDETR_ENGINE:-/workspaces/isaac_ros-dev/isaac_ros_assets/models/synthetica_detr/sdetr_grasp.plan}"
POSE_TOPIC="${POSE_TOPIC:-/isaac_ros_examples/output}"

if ! command -v docker >/dev/null || ! docker inspect "$CONTAINER" >/dev/null 2>&1; then
  echo "ERROR: Isaac ROS container '$CONTAINER' is not available." >&2
  exit 2
fi

for active_node in '/install/bev_detection/lib/bev_detection/yolo_trt_node' '/install/bev_segmentation/lib/bev_segmentation/segmentation_node'; do
  if pgrep -f "$active_node" >/dev/null; then
    echo "ERROR: shared Hub inference node '$active_node' is active. Stop its run through the owning Hub before using the GPU." >&2
    exit 3
  fi
done

docker exec "$CONTAINER" bash -lc "test -x '$TRTEXEC' && test -f '$MODEL_ROOT/refine_model.onnx' && test -f '$MODEL_ROOT/score_model.onnx' && test -f '$ASSET_ROOT/quickstart_interface_specs.json' && test -f '$ASSET_ROOT/quickstart.bag/metadata.yaml'" || {
  echo "ERROR: required Isaac ROS 3.2 FoundationPose quickstart assets are missing." >&2
  exit 4
}

docker exec "$CONTAINER" test -f "$RTDETR_ENGINE" || {
  echo "ERROR: RT-DETR engine not found in container: $RTDETR_ENGINE" >&2
  exit 6
}

docker exec "$CONTAINER" bash -lc "
  set -euo pipefail
  source /opt/ros/humble/setup.bash
  launch_log=/tmp/m4_4_foundationpose_launch.log
  bag_log=/tmp/m4_4_foundationpose_bag.log
  launch_pid=
  bag_pid=
  cleanup() {
    [ -z \"\$bag_pid\" ] || kill \"\$bag_pid\" 2>/dev/null || true
    [ -z \"\$launch_pid\" ] || kill \"\$launch_pid\" 2>/dev/null || true
  }
  trap cleanup EXIT INT TERM
  cd '$MODEL_ROOT'
  for model in refine score; do
    onnx='${MODEL_ROOT}'/'\$model'_model.onnx
    engine='${MODEL_ROOT}'/'\$model'_trt_engine.plan
    if [ ! -s \"\$engine\" ]; then
      if [ \"\$model\" = refine ]; then
        max_batch=42
      else
        max_batch=252
      fi
      '$TRTEXEC' --onnx=\"\$onnx\" --saveEngine=\"\$engine\" \\
        --minShapes=input1:1x160x160x6,input2:1x160x160x6 \\
        --optShapes=input1:1x160x160x6,input2:1x160x160x6 \\
        --maxShapes=input1:\${max_batch}x160x160x6,input2:\${max_batch}x160x160x6 \\
        --maxAuxStreams=0 --builderOptimizationLevel=0 --memPoolSize=workspace:4096
    fi
  done
  test -s '$MODEL_ROOT/refine_trt_engine.plan'
  test -s '$MODEL_ROOT/score_trt_engine.plan'
  ros2 launch isaac_ros_examples isaac_ros_examples.launch.py \\
    launch_fragments:=foundationpose \\
    interface_specs_file:='$ASSET_ROOT/quickstart_interface_specs.json' \\
    mesh_file_path:='$ASSET_ROOT/Mustard/textured_simple.obj' \\
    texture_path:='$ASSET_ROOT/Mustard/texture_map.png' \\
    refine_engine_file_path:='$MODEL_ROOT/refine_trt_engine.plan' \\
    score_engine_file_path:='$MODEL_ROOT/score_trt_engine.plan' \\
    rt_detr_engine_file_path:='$RTDETR_ENGINE' >\"\$launch_log\" 2>&1 &
  launch_pid=\$!
  sleep 20
  if ! kill -0 \"\$launch_pid\" 2>/dev/null; then
    cat \"\$launch_log\"
    exit 7
  fi
  ros2 bag play '$ASSET_ROOT/quickstart.bag' --loop --delay 1 >\"\$bag_log\" 2>&1 &
  bag_pid=\$!
  if timeout 240 ros2 topic echo --once '$POSE_TOPIC'; then
    echo 'Official FoundationPose quickstart published one pose message on $POSE_TOPIC'
  else
    echo 'No pose message received on $POSE_TOPIC; launch and bag logs follow.' >&2
    cat \"\$launch_log\" >&2
    cat \"\$bag_log\" >&2
    exit 8
  fi
"
