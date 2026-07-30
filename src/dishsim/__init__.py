# Copyright (c) 2026, dishsim project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""dishsim: classical motion planning for dishwasher loading (UR5e + Robotiq 2F-85, Isaac Sim).

v0 scope: the object starts rigidly attached to the gripper TCP; OMPL (RRT-Connect) plans a
collision-free joint-space path that places it at a valid pose in the dishwasher's lower rack.
The collision world is a standalone, Kit-free module so a future MCTS rearrangement planner can
reuse it for thousands of fast queries.

Frame convention (asserted throughout): robot-base frame, meters, Z-up, quaternions XYZW.
"""

import os

# Path to the repository root (two levels up from this file: src/dishsim -> src -> root).
PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))

# Downloaded assets live here (gitignored; see README for sources).
ASSETS_DIR = os.path.join(PROJECT_ROOT, "assets")
