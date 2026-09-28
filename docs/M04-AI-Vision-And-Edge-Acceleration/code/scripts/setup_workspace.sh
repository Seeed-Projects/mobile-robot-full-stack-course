#!/usr/bin/env bash
# setup_workspace.sh — assemble the M4 colcon workspace from canonical chapter sources.
#
# Canonical package source lives in the chapter directories:
#   4.1-yolo-object-detection/ros2/bev_detection
#   4.2-multi-object-tracking/ros2/bev_tracking
#   4.3-semantic-segmentation/ros2/bev_segmentation
#   4.5-native-foundationpose/ros2/bev_pose
#   common/ros2/m4_demo_bringup
#
# This script creates ros2_ws/src/<pkg> SYMLINKS pointing at those directories.
# There is exactly ONE editable copy of each package. Re-run after a fresh clone
# (git does not track the symlinks themselves - see .gitignore).
#
# Usage:
#   ./scripts/setup_workspace.sh          # create/refresh symlinks
#   ./scripts/setup_workspace.sh --clean  # remove the workspace build dirs
set -euo pipefail

M4_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WS="$M4_DIR/ros2_ws"
SRC="$WS/src"

declare -A PKGS=(
  [bev_detection]="../../4.1-yolo-object-detection/ros2/bev_detection"
  [bev_tracking]="../../4.2-multi-object-tracking/ros2/bev_tracking"
  [bev_segmentation]="../../4.3-semantic-segmentation/ros2/bev_segmentation"
  [bev_pose]="../../4.5-native-foundationpose/ros2/bev_pose"
  [m4_demo_bringup]="../../common/ros2/m4_demo_bringup"
  [bev_interfaces]="../../common/ros2/bev_interfaces"
)

if [ "${1:-}" = "--clean" ]; then
  rm -rf "$WS/build" "$WS/install" "$WS/log"
  echo "[setup_workspace] cleaned $WS/{build,install,log}"
  exit 0
fi

mkdir -p "$SRC"
for pkg in "${!PKGS[@]}"; do
  # Targets are relative to src/ (where the link lives); resolve for validation.
  target="$(realpath -m "$SRC/${PKGS[$pkg]}")"
  [ -d "$target" ] || { echo "[setup_workspace] ERROR: missing $target" >&2; exit 1; }
  ln -sfn "${PKGS[$pkg]}" "$SRC/$pkg"
  printf "  %-18s -> %s\n" "$pkg" "${PKGS[$pkg]}"
done

cat <<'MSG'

[setup_workspace] done. Build with:
  source /opt/ros/humble/setup.bash
  cd <this module>/ros2_ws
  colcon build --symlink-install --packages-select bev_detection bev_tracking bev_segmentation bev_pose m4_demo_bringup
MSG
