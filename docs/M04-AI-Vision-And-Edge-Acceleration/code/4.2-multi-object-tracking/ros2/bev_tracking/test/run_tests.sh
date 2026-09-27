#!/usr/bin/env bash
# run_tests.sh - run bev_tracking pytest suite with the host's plugin shims.
#
# Why does this script exist?
#   - The system pytest (6.2.5) on this host is incompatible with the
#     launch_testing_ros entry-point (which targets pytest >= 7 hooks).
#     Without a workaround pytest raises INTERNALERROR on collection.
#   - We work around it by pre-stubbing the broken entrypoint in
#     `sys.modules` BEFORE pytest collects plugins.
#   - We also stub `anyio.pytest_plugin` because the user's local
#     site-packages contains a newer anyio (4.15) that depends on
#     `_pytest.scope` (pytest >= 7), causing a second INTERNALERROR.
#
# This script wraps both shims in a single command. It is the canonical
# way to run the bev_tracking tests on this host. CI on a fresh Ubuntu
# image (which has matching pytest / launch_testing versions) does NOT
# need the shims and can call pytest directly.
#
# Usage:
#   cd <repo-root>
#   bash modules/m04-ai-vision-and-edge-acceleration/4.2-multi-object-tracking/ros2/bev_tracking/test/run_tests.sh [extra pytest args]
set -eo pipefail

# 1) Source ROS + workspace overlays.
if [[ -f /opt/ros/humble/setup.bash ]]; then
    # shellcheck disable=SC1091
    source /opt/ros/humble/setup.bash
fi
if [[ -f install/setup.bash ]]; then
    # shellcheck disable=SC1091
    source install/setup.bash
fi

# 2) Run pytest through a small Python bootstrap that installs the shims.
exec python3 -c "
import sys, types

# Pre-stub anyio so the newer anyio plugin doesn't crash pytest 6.
for name in ('anyio', 'anyio.pytest_plugin'):
    if name not in sys.modules:
        sys.modules[name] = types.ModuleType(name)

# Pre-stub launch_testing_ros_pytest_entrypoint: pytest 6 doesn't know
# the pytest_launch_collect_makemodule hook that the entrypoint dutifully
# declares. The shim declares only pytest_configure, which is harmless.
if 'launch_testing_ros_pytest_entrypoint' not in sys.modules:
    shim = types.ModuleType('launch_testing_ros_pytest_entrypoint')
    shim.pytest_configure = lambda *a, **k: None
    sys.modules['launch_testing_ros_pytest_entrypoint'] = shim

import pytest
sys.exit(pytest.main(['-v', 'src/bev_tracking/test/'] + sys.argv[1:]))
" "$@"
