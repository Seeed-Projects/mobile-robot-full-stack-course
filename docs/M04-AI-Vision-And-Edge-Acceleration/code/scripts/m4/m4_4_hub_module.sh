#!/usr/bin/env bash
set -euo pipefail

# Hub module wrapper for M4.4 — one subprocess, started by the web hub's
# ModuleRuntimeManager (start_new_session + killpg SIGTERM contract).
#
# It owns exactly two children:
#   1. the host overlay node (ros2 launch m4_demo_bringup m4_4_web.launch.py,
#      own process group) -> publishes /perception/demo/m4_4 + .../stats
#   2. run_m4_4_isaacros_quickstart.sh in hub-hold mode (M44_VIEW_SECONDS=-1)
#      -> Isaac ROS FoundationPose launch + looping quickstart.bag in the
#      container, held until SIGTERM
#
# The quickstart runs in the FOREGROUND so the wrapper's exit code mirrors
# it (the manager records that in the module runtime log). On SIGTERM the
# manager kills the whole wrapper group: the docker exec client dies, the
# quickstart's deferred TERM trap fires and stop_owned_run tears down the
# container launch + bag (state-file scoped), then this script's EXIT trap
# stops the overlay node.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# scripts/m4 -> module -> modules -> repo root (the colcon workspace).
WS_ROOT="$(cd "$SCRIPT_DIR/../../../.." && pwd)"
QUICKSTART="$SCRIPT_DIR/run_m4_4_isaacros_quickstart.sh"
CONTAINER="${ISAAC_ROS_CONTAINER:-m4-isaacros-foundationpose}"
LAUNCH_INSTALLED="$WS_ROOT/install/m4_demo_bringup/share/m4_demo_bringup/launch/m4_4_web.launch.py"
# The overlay node subscribes to topics published by root-owned container
# processes; without the UDP-only profile Fast DDS local-peer SHM routing
# silently drops every message (root's /dev/shm segments are unreadable by
# seeed). See the comment inside the profile.
UDP_PROFILE="${M44_UDP_PROFILE:-$SCRIPT_DIR/../../4.4-isaac-ros-foundationpose/config/fastdds_udp_only.xml}"
VIZ_LOG="${M4_4_VIZ_LOG:-/tmp/m4_4_web_visualizer.log}"
VIZ_PID=

command -v docker >/dev/null 2>&1 || { echo 'ERROR: docker unavailable.' >&2; exit 2; }
docker inspect "$CONTAINER" >/dev/null 2>&1 || { echo "ERROR: Isaac ROS container '$CONTAINER' is not running." >&2; exit 2; }
[ -f "$QUICKSTART" ] || { echo "ERROR: missing $QUICKSTART" >&2; exit 2; }
[ -f "$LAUNCH_INSTALLED" ] || { echo 'ERROR: installed m4_4_web.launch.py missing (rebuild m4_demo_bringup).' >&2; exit 2; }
[ -f "$UDP_PROFILE" ] || { echo "ERROR: missing Fast DDS UDP profile: $UDP_PROFILE" >&2; exit 2; }
if ! command -v ros2 >/dev/null 2>&1; then
  # Manual invocation without the hub supervisor's environment.
  # shellcheck disable=SC1091
  . "$WS_ROOT/scripts/setup_env.sh" >/dev/null 2>&1 || true
fi
command -v ros2 >/dev/null 2>&1 || { echo 'ERROR: ros2 not on PATH (source scripts/setup_env.sh first).' >&2; exit 2; }

# Scoped sweep: ONLY runs named by hub-* state files, killed via the PIDs
# recorded in those files — never a blanket pkill. Covers a previous wrapper
# whose manager SIGKILL beat the docker teardown and leaked the container
# launch + bag.
sweep_stale_hub_runs() {
  docker exec "$CONTAINER" bash -lc '
    shopt -s nullglob
    for state_file in /tmp/m44-run-hub-*.state; do
      mapfile -t owned < "$state_file"
      inner="${owned[0]:-}"; launch="${owned[1]:-}"; bag="${owned[2]:-}"
      for pid in "$bag" "$launch"; do
        [[ "$pid" =~ ^[0-9]+$ ]] && kill -TERM -- "-$pid" 2>/dev/null || true
      done
      if [[ "$inner" =~ ^[0-9]+$ ]]; then
        pkill -TERM -P "$inner" 2>/dev/null || true
        kill -TERM "$inner" 2>/dev/null || true
      fi
      sleep 2
      for pid in "$bag" "$launch"; do
        [[ "$pid" =~ ^[0-9]+$ ]] && kill -KILL -- "-$pid" 2>/dev/null || true
      done
      rm -f "$state_file"
    done
  ' >/dev/null 2>&1 || true
}

cleanup() {
  if [ -n "$VIZ_PID" ]; then
    kill -TERM -- "-$VIZ_PID" 2>/dev/null || true
    sleep 3
    kill -KILL -- "-$VIZ_PID" 2>/dev/null || true
  fi
}
trap cleanup EXIT
trap 'exit 129' HUP
trap 'exit 130' INT
trap 'exit 143' TERM

sweep_stale_hub_runs

# Scope the profile to the overlay node only: the quickstart below talks to
# docker exec with an explicit env list, so this never leaks into the
# container, and the container keeps its own (root↔root) SHM transport.
FASTRTPS_DEFAULT_PROFILES_FILE="$UDP_PROFILE" \
  setsid ros2 launch m4_demo_bringup m4_4_web.launch.py >"$VIZ_LOG" 2>&1 &
VIZ_PID=$!

echo "m4_4 hub module: overlay launch pid=$VIZ_PID (log $VIZ_LOG); starting quickstart in hub-hold mode"
M44_VIEW_SECONDS=-1 M44_RUN_TOKEN="hub-$(date +%Y%m%d%H%M%S)-$$" "$QUICKSTART"
