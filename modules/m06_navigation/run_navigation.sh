#!/usr/bin/env bash
set -euo pipefail
m6_repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
if [[ ! -f "${m6_repo_dir}/install/m06_navigation/setup.bash" ]]; then
  "${m6_repo_dir}/modules/m06_navigation/build.sh"
fi
unset ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION ROS_PACKAGE_PATH PYTHONPATH AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH
set +u
source "${m6_repo_dir}/modules/m06_navigation/setup.bash"
set -u
exec ros2 launch j501_navigation navigation.launch.py map:="${m6_repo_dir}/build/m06_maps/office.yaml" \
  view_output:="${m6_repo_dir}/build/m06_results/algorithm_view" "$@"
