# Source this file in the terminal used for ROS 2 commands.
_m6_repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
source /opt/ros/humble/setup.bash
if [[ -f "${_m6_repo_dir}/build/m06_nav2_deps/setup.bash" ]]; then
  source "${_m6_repo_dir}/build/m06_nav2_deps/setup.bash"
fi
if [[ -f "${_m6_repo_dir}/install/m06_navigation/setup.bash" ]]; then
  source "${_m6_repo_dir}/install/m06_navigation/setup.bash"
else
  printf 'Run %s/modules/m06_navigation/build.sh first.\n' "${_m6_repo_dir}"
  unset _m6_repo_dir
  return 1
fi
unset _m6_repo_dir
