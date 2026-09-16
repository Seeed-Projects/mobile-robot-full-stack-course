#!/usr/bin/env bash
# 一键启动 Web 标定 pipeline（内参+外参+BEV 预览，无需显示器）
# 用法: bash scripts/run_calib_web.sh [port]
set -euo pipefail

PORT="${1:-8090}"
REPO="/home/seeed/workspace/ros2_bev"

cd "$REPO"

# 检测 camera_driver 占用（会独占相机导致 device busy）
if pgrep -x camera_driver >/dev/null 2>&1; then
  PID=$(pgrep -x camera_driver | head -1)
  echo "⚠ 检测到 camera_driver (PID $PID) 在跑，会独占相机！"
  echo "  停掉它再启动:  pkill -x camera_driver"
  echo "  （或在其 ros2 launch 终端 Ctrl+C）"
  echo "继续启动会因 device busy 抓不到帧。是否先停掉？[y/N]"
  read -r ans
  if [[ "$ans" == "y" || "$ans" == "Y" ]]; then
    pkill -x camera_driver || true
    sleep 8   # teardown max967 寄存器复位需数秒
    echo "已停 camera_driver，等待 teardown 完成。"
  else
    echo "未停，继续（可能 device busy）。"
  fi
fi

# 固定结果/配置目录（与 ROS2 pipeline 共用；绕开 config.py ament fallback）
export J501_AVM_CALIB_RESULTS_DIR="$REPO/calib_results"
export J501_AVM_CALIB_CONFIG_DIR="/home/seeed/ros2_ws/src/j501_avm_calib/config"

# 系统 python3（有 cv2/numpy/yaml；.venv-tools 缺 cv2 不用）
export PYTHONPATH="/home/seeed/ros2_ws/src/j501_avm_calib:${PYTHONPATH:-}"

echo "启动标定台 http://0.0.0.0:$PORT ..."
exec python3 "$REPO/tools/calib_web.py" --port "$PORT"
