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

# Path to the repository root (two levels up from this file: src/dishsim -> src -> root).
PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))

# Downloaded assets live here (gitignored; see README for sources).
ASSETS_DIR = os.path.join(PROJECT_ROOT, "assets")
