"""Conftest for bev_tracking tests.

Marks the directory as a test root for ament_python's pytest runner,
AND works around a known incompatibility between the system pytest
(6.2.5 on this host) and the launch_testing_ros pytest entry-point.

The launch_testing_ros entry-point registers a `pytest_launch_collect_makemodule`
hook that pytest >= 7 renamed. With pytest 6 the plugin declares a hook
that does not exist in the host pytest, causing a `PluginValidationError`
on every collection.

We avoid the problem by inserting a small shim that re-exports the
launch_testing_ros entrypoint module but with its offending hook
removed. The shim is installed at conftest load time, which runs BEFORE
pytest collects plugins, so the broken entrypoint is masked entirely.

If pytest is upgraded to >= 7 and the underlying plugin starts working,
this shim becomes a no-op: the entrypoint itself will load successfully
and our shim just exposes an empty namespace.
"""

import sys
import types


def _install_launch_testing_shim():
    """Mask the broken launch_testing_ros pytest entrypoint.

    The shim pretends to be the real module, but does not register any
    hooks. It MUST be installed before pytest collects plugins.
    """
    target = 'launch_testing_ros_pytest_entrypoint'
    if target in sys.modules:
        return  # already installed (real one loaded fine, nothing to do)

    shim = types.ModuleType(target)

    def _noop(*_args, **_kwargs):
        return None

    def pytest_configure(config):
        # Real entrypoint only adds a marker line; harmless to skip in
        # the shim because we don't use rostest markers in this package.
        return None

    shim.pytest_configure = pytest_configure
    sys.modules[target] = shim


_install_launch_testing_shim()
