#!/usr/bin/env bash
set -euo pipefail

# View the verified Mustard bag on the Jetson desktop. The temporary viewer
# container connects to the local X socket; the existing container runs pose
# inference. Direct X access is needed for the physical NVIDIA display.
CONTAINER="${ISAAC_ROS_CONTAINER:-m4-isaacros-foundationpose}"
VIEWER_IMAGE="${M44_VIEWER_IMAGE:-m44-foundationpose-viewer:local}"
M44_MODE="${M44_MODE:-official}"
M44_VIEW_SECONDS="${M44_VIEW_SECONDS:-120}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [ -z "${DISPLAY:-}" ]; then
  echo 'ERROR: start this from a Jetson graphical desktop terminal (DISPLAY is empty).' >&2
  exit 2
fi
for required in docker xauth flock realpath; do
  command -v "$required" >/dev/null || { echo "ERROR: $required is unavailable." >&2; exit 2; }
done
exec {lock_fd}>"/tmp/m44-isaacros-visual-${UID}.lock"
if ! flock -n "$lock_fd"; then
  echo 'ERROR: the M4.4 visual demo is already running on this Jetson.' >&2
  exit 3
fi
if ! docker inspect "$CONTAINER" >/dev/null 2>&1; then
  echo "ERROR: Isaac ROS container '$CONTAINER' is unavailable." >&2
  exit 2
fi
if ! docker image inspect "$VIEWER_IMAGE" >/dev/null 2>&1; then
  echo "ERROR: viewer image '$VIEWER_IMAGE' is missing. Run: docker commit $CONTAINER $VIEWER_IMAGE" >&2
  exit 2
fi
rviz_config="${M44_RVIZ_CONFIG:-$SCRIPT_DIR/../../4.4-isaac-ros-foundationpose/rviz/m44_touchscreen.rviz}"
if [ ! -r "$rviz_config" ]; then
  echo "ERROR: M4.4 touch-screen RViz configuration '$rviz_config' is missing." >&2
  exit 2
fi
rviz_config="$(realpath "$rviz_config")"

display_num="${DISPLAY##*:}"
display_num="${display_num%%.*}"
case "$display_num" in
  ''|*[!0-9]*) echo "ERROR: unsupported DISPLAY=$DISPLAY" >&2; exit 2 ;;
esac
if [ ! -S "/tmp/.X11-unix/X${display_num}" ]; then
  echo "ERROR: X11 socket for DISPLAY=$DISPLAY is missing." >&2
  exit 2
fi
host_auth="${XAUTHORITY:-$HOME/.Xauthority}"
if [ ! -r "$host_auth" ]; then
  echo "ERROR: X11 authorization file '$host_auth' is unreadable." >&2
  exit 2
fi
if [ -z "$(xauth -f "$host_auth" list ":${display_num}" | awk 'NR==1 {print $3}')" ]; then
  echo "ERROR: no X11 authorization cookie for DISPLAY=$DISPLAY." >&2
  exit 2
fi

viewer="m44-rviz-$$"
quickstart_pid=
cleanup() {
  if [ -n "$quickstart_pid" ]; then
    kill -TERM "$quickstart_pid" 2>/dev/null || true
    wait "$quickstart_pid" 2>/dev/null || true
  fi
  docker rm -f "$viewer" >/dev/null 2>&1 || true
}
trap cleanup EXIT
trap 'exit 129' HUP
trap 'exit 130' INT
trap 'exit 143' TERM

docker run -d --name "$viewer" --runtime nvidia --network host --ipc host \
  -e DISPLAY=":${display_num}" -e XAUTHORITY=/tmp/m44-host.Xauthority \
  -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
  -v "$host_auth:/tmp/m44-host.Xauthority:ro" \
  -v "$rviz_config:/tmp/m44-view.rviz:ro" \
  --entrypoint bash "$VIEWER_IMAGE" -lc \
  'source /opt/ros/humble/setup.bash; exec rviz2 -d /tmp/m44-view.rviz' >/dev/null
sleep 5
if [ "$(docker inspect -f '{{.State.Running}}' "$viewer")" != true ]; then
  echo 'ERROR: RViz did not start.' >&2
  docker logs --tail 60 "$viewer" >&2 || true
  exit 7
fi

echo "RViz is open on DISPLAY=$DISPLAY."
echo 'Mustard RGB is on the left and the 3D pose is on the right.'
echo 'Use the RViz Panels menu to restore hidden settings panes.'
echo "The Mustard graph will remain active for ${M44_VIEW_SECONDS}s after valid_pose."
M44_MODE="$M44_MODE" M44_VIEW_SECONDS="$M44_VIEW_SECONDS" \
  M44_RUN_TOKEN="visual-$$" "$SCRIPT_DIR/run_m4_4_isaacros_quickstart.sh" &
quickstart_pid=$!
wait "$quickstart_pid"
