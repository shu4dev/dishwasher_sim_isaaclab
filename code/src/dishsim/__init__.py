# Copyright (c) 2026, dishsim project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""dishsim: arrangement planning for dishwasher loading, physics-validated in Isaac Sim.

Scope: decide where each object goes in the machine — feasible = collision-free (Kit-free FCL
world) + physically stable (Isaac settle validation). Object motion is teleportation: a runner
writes root poses and lets physics settle; there is no robot arm and no motion planning. The
collision world is a standalone, Kit-free module so a rearrangement planner can run thousands
of fast placement queries in a plain Python process.

Frame convention (asserted throughout): base frame, meters, Z-up, quaternions XYZW.
"""

import os

# Path to the repository root (three levels up from this file: code/src/dishsim -> code/src -> code -> root).
PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))

# Downloaded assets live here (gitignored; see README for sources).
ASSETS_DIR = os.path.join(PROJECT_ROOT, "data", "assets")

# The seven data roots moved under data/ on 2026-10-03; records written before then keep the old paths.
_DATA_NAMES = ("assets", "results", "media", "logs", "outputs", "build", "artifacts")


def legacy_data_path(p):
    """A path read from a stored record, as a string: ``/workspace/dishsim/<name>/...`` or ``<name>/...`` (one of
    the seven data roots) maps to the same path under ``data/``; any other path comes back unchanged."""
    s = str(p)
    for prefix in ("/workspace/dishsim/", ""):
        if s.startswith(tuple(f"{prefix}{name}/" for name in _DATA_NAMES)):
            return f"{prefix}data/{s[len(prefix):]}"
    return s
