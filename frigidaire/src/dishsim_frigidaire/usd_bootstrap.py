"""Load the bundled Isaac USD libraries for standalone, Kit-free asset tools.

Isaac Sim 4.5's ``python.sh`` omits USD extension directories before Kit starts.
Call :func:`ensure_usd` from a standalone entrypoint, before importing pxr. This
restarts that same Python command once with the bundled libraries on its paths;
it does not install packages or launch Kit. Inside a running Kit app it is a no-op.
"""

from __future__ import annotations

import os
from pathlib import Path
import sys


_BOOTSTRAP_MARKER = "DISHSIM_FRIGIDAIRE_USD_BOOTSTRAPPED"


def bundled_usd_environment(isaac_root: str | Path = "/isaac-sim") -> dict[str, str]:
    """Return an environment using the Isaac runtime's own USD/PhysX schemas."""
    root = Path(isaac_root)
    candidates = sorted((root / "extscache").glob("omni.usd.libs-*/pxr/Usd"))
    if not candidates:
        raise RuntimeError(
            "Bundled USD was not found. Run this tool with scripts/run_py.sh "
            "inside the dishsim-isaac container, or use a Python with USD installed."
        )
    usd_extension = candidates[-1].parents[1]
    physx_extension = root / "extsPhysics/omni.usd.schema.physx"
    if not (physx_extension / "pxr/PhysxSchema").is_dir():
        raise RuntimeError(f"Missing Isaac PhysX USD schema: {physx_extension}")

    env = dict(os.environ)
    additions = {
        "PYTHONPATH": [usd_extension, physx_extension],
        "LD_LIBRARY_PATH": [usd_extension / "bin", physx_extension / "bin"],
        "PXR_PLUGINPATH_NAME": [physx_extension / "plugins/PhysxSchema/resources"],
    }
    for key, paths in additions.items():
        existing = env.get(key, "")
        env[key] = os.pathsep.join([*(str(p) for p in paths), *([existing] if existing else [])])
    env[_BOOTSTRAP_MARKER] = "1"
    return env


def ensure_usd() -> None:
    """Make USD and PhysX schemas importable, restarting this entrypoint if needed.

    Do not call this after creating a Kit application: its own extension loader
    supplies the same modules. A plain ``usd-core`` install without PhysxSchema
    still needs the Isaac runtime for this asset's articulation attributes.
    """
    try:
        from pxr import PhysxSchema, Usd, UsdGeom, UsdPhysics  # noqa: F401

        return
    except ImportError as error:
        if os.environ.get(_BOOTSTRAP_MARKER) == "1":
            raise RuntimeError("Isaac USD bootstrap failed after restart") from error
        env = bundled_usd_environment()
        # orig_argv preserves `python -m ...` and `python -c ...`; sys.argv alone
        # loses the interpreter switches. Python 3.10 is the supported runtime.
        original = getattr(sys, "orig_argv", None)
        args = [sys.executable, *(original[1:] if original else sys.argv)]
        os.execve(sys.executable, args, env)
