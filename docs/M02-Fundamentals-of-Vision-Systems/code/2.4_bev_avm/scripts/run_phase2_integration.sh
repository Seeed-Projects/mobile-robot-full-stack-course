#!/usr/bin/env bash
# Phase 2 integration: bag playback + perception stack in one sourced environment.
# Usage: scripts/run_phase2_integration.sh [bag_rate] [--once]
#   --once: play the bag a single time (clean 60-frame validation pass)
set -eo pipefail
REPO=/home/seeed/workspace/ros2_bev
BAG_RATE="${1:-1.0}"
LOOP="--loop"
if [[ "$2" == "--once" ]]; then LOOP=""; fi

# clean stale nodes from previous runs
pkill -f "ros2 bag play" 2>/dev/null || true
pkill -f bevdet_node 2>/dev/null || true
pkill -f camera_sync_node 2>/dev/null || true
pkill -f bev_system_monitor 2>/dev/null || true
pkill -f bev_visualization 2>/dev/null || true
pkill -f "ros2 launch bev_bringup" 2>/dev/null || true
sleep 2

source /opt/ros/humble/setup.bash
source "$REPO/ros2_ws/install/setup.bash"
export PATH=/usr/local/cuda-12.6/bin:$PATH

cd "$REPO"
(setsid ros2 bag play datasets/bags/nuscenes_mini_60f $LOOP --rate "$BAG_RATE" \
  > /tmp/bagplay.log 2>&1 &)
sleep 4

exec ros2 launch bev_bringup perception.launch.py