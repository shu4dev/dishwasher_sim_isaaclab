# Copyright (c) 2026, dishsim project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""The few shared constants the Kit-free collision world and the camera rig still read.

Trimmed on 2026-11-01 (plans/2026-11-01-merged-benchmark.md) to what ``collision_world.py`` (directly and through
``geometry.py``) and ``media.py`` read; the Bosch 800 slot pipeline's machine, object, placement and robot-era
tables left with it (git history, tag isaac45-final). Frame convention everywhere: **base frame, meters, Z-up,
quaternions XYZW**.

This module must stay importable without Kit *and* without the planning venv (standard library only).
"""

# frame convention (asserted by the collision-world manifest reader, geometry.load_manifest)
FRAME_CONVENTION = {"frame": "robot_base", "units": "m", "up": "+z", "quat": "xyzw"}

# Hull inflation for candidate objects in FCL queries. Demoted from safety-certificate to
# pre-filter (2026-08-28): the Isaac settle gates are the certificate for placements, so this
# only needs to keep obviously-touching poses out of the settle budget. At 5 mm it vetoed
# real clearances of 1.6-3.3 mm (plate gap_right4, the rear bowl row, and the 120 mm two-step
# bowl spacing whose true hull-to-hull need is 111.2 mm) — halving certified capacity.
COLLISION_MARGIN_M = 0.002

# CoACD parameters per body (fallback key "default"); threshold is the concavity tolerance —
# lower = finer decomposition. geometry.coacd_dir_for keys each body's piece folder on them.
COACD = {
    "default": {"threshold": 0.05, "max_convex_hull": 32},
    "object": {"threshold": 0.03, "max_convex_hull": 32},  # keep a cup's or mug's cavity open
}

# Per-camera lens default of media.CameraRig (a camera spec may carry its own).
#
# The VERTICAL field of view is the binding constraint, not the horizontal one: the frame is
# 16:9 and `vertical_aperture` defaults to `horizontal_aperture * height/width`, so
#
#     focal 24 mm -> hFOV 47.2 deg, vFOV 27.6 deg
CAMERA_LENS_DEFAULT = {"focal_length": 24.0, "horizontal_aperture": 20.955}
