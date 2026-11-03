#!/usr/bin/env bash
set -euo pipefail
m6_repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
unset ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION ROS_PACKAGE_PATH PYTHONPATH AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH
set +u
source /opt/ros/humble/setup.bash
set -u
if [[ (! -f /opt/ros/humble/share/nav2_bringup/package.xml || ! -f /opt/ros/humble/share/slam_toolbox/package.xml) && ! -f "${m6_repo_dir}/build/m06_nav2_deps/setup.bash" ]]; then
  /usr/bin/python3 "${m6_repo_dir}/modules/m06_navigation/j501_navigation/tools/bootstrap_nav2.py" \
    --output "${m6_repo_dir}/build/m06_nav2_deps"
fi
if [[ -f "${m6_repo_dir}/build/m06_nav2_deps/setup.bash" ]]; then
  source "${m6_repo_dir}/build/m06_nav2_deps/setup.bash"
fi
if ! /usr/bin/python3 - <<'PY'
from ament_index_python.packages import get_package_share_directory
import scipy
import matplotlib
for package in ['nav2_bringup', 'slam_toolbox', 'robot_localization']:
    get_package_share_directory(package)
PY
then
  /usr/bin/python3 "${m6_repo_dir}/modules/m06_navigation/j501_navigation/tools/bootstrap_nav2.py" \
    --output "${m6_repo_dir}/build/m06_nav2_deps"
  source "${m6_repo_dir}/build/m06_nav2_deps/setup.bash"
fi
colcon --log-base "${m6_repo_dir}/log/m06_navigation" build \
  --base-paths "${m6_repo_dir}/modules/m06_navigation" \
  --packages-select j501_chassis_description j501_navigation --symlink-install \
  --build-base "${m6_repo_dir}/build/m06_navigation" \
  --install-base "${m6_repo_dir}/install/m06_navigation" \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
