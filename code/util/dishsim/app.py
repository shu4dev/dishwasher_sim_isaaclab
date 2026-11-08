# Copyright (c) 2026, dishsim project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Kit boot for the entry scripts: Isaac Lab 3.0's launcher behind the repo's launcher CLI.

Isaac Lab 3.0 starts Isaac Sim through :func:`isaaclab.app.launch_simulation`, and its parser no
longer carries ``--headless`` or ``--enable_cameras``. The repo's scripts, the sampler wrappers and
the docs keep passing both, so :func:`add_args` adds them back beside the launcher's own flags
(``--device``, ``--viz``, ``--kit_args``, ...). :func:`launch` starts Kit (headless unless
``--viz kit``) with ``enable_cameras`` set before the launch (3.0 only auto-enables cameras for a
config it can scan, and the scripts pass none) and returns the running app; ``app.close()`` ends it.

Only launcher keys reach the launcher, never a script's own flags: a script's ``--video``
would otherwise make Kit ask for its MoviePy recorder.

Boot-first: import this module and call :func:`launch` before importing ``isaaclab`` scene, asset
or sim modules, ``pxr``, or the Kit-only ``dishsim`` modules. Nothing here imports Isaac at module
scope.
"""

import argparse
import contextlib

#: The KitLauncher's arguments (isaaclab_physx/app/kit_launcher.py) and the two restored here.
_LAUNCHER_KEYS = ("headless", "enable_cameras", "device", "device_explicit", "livestream", "xr",
                  "visualizer", "verbose", "info", "experience", "deterministic", "kit_args",
                  "anim_recording_enabled", "anim_recording_start_time", "anim_recording_stop_time",
                  "max_visible_envs")

#: The launched runtime, held for the process so it outlives the app object (as 3.0's own shim does).
_RUNTIME = contextlib.ExitStack()


def add_args(parser: argparse.ArgumentParser) -> None:
    """Add the launcher arguments plus ``--headless`` and ``--enable_cameras`` to *parser*."""
    from isaaclab.app import add_launcher_args  # noqa: PLC0415

    add_launcher_args(parser)
    group = parser.add_argument_group("dishsim launcher arguments")
    group.add_argument("--headless", action="store_true", default=False,
                       help="Run without a viewport (Isaac Lab 3.0 runs headless unless --viz kit).")
    group.add_argument("--enable_cameras", action="store_true", default=False,
                       help="Enable the RTX rendering pipeline for camera sensors.")


class _App:
    """The running Kit app; ``close()`` stops the runtime :func:`launch` started."""

    def __init__(self, app):
        self._app = app

    def close(self, exit_code: int = 0) -> None:
        # unwind the launch as a ``sys.exit(exit_code)`` so the runtime ends with that status
        _RUNTIME.__exit__(SystemExit, SystemExit(exit_code), None)

    def __getattr__(self, name):
        return getattr(self._app, name)


def launch(args) -> _App:
    """Start Isaac Sim / Kit for the parsed *args* (a namespace or dict); returns the app."""
    from isaaclab.app import launch_simulation  # noqa: PLC0415

    values = vars(args) if isinstance(args, argparse.Namespace) else dict(args)
    launcher_args = {key: values[key] for key in _LAUNCHER_KEYS if key in values}
    launcher_args["enable_cameras"] = bool(values.get("enable_cameras", False))
    launcher_args["require_kit"] = True
    _RUNTIME.enter_context(launch_simulation(None, launcher_args))

    import omni.kit.app  # noqa: PLC0415  (importable once Kit runs)

    return _App(omni.kit.app.get_app())
