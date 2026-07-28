# Copyright (c) 2026, dishwasher_tasks project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Dishwasher-manipulation tasks for Isaac Lab: UR5e + Robotiq 2F-85 vs. an ArtVIP dishwasher."""

import os

# Path to the repository root (four levels up from this file).
PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))

# Downloaded assets live here (gitignored; see README for sources).
ASSETS_DIR = os.path.join(PROJECT_ROOT, "assets")
