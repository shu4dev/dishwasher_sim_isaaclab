# Copyright (c) 2026, dishsim project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Kit-free sanity checks on the FROZEN CACHE ANCHOR grasp constants in dishsim.config.

These robot-era constants feed geometry.config_hash per class, so this file is part of the
zero-rebake tripwire: a drifted anchor fails here before it bricks the shipped caches.
Run with the project venv:

    PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /workspace/isaaclab/env_isaaclab/bin/python -m pytest \
        tests/test_grasp_config.py -v
"""

import numpy as np
import pytest

from dishsim import config
from dishsim.transforms import make_T

CALIBRATED = (
    config.GRIPPER_APERTURE_GRASP_RAD is not None and config.GRASP_RIM_TCP_Z_M is not None
)
needs_calibration = pytest.mark.skipif(
    not CALIBRATED, reason="frozen robot-era grasp anchors absent (should never happen)"
)


def test_jaw_clearance_geometry():
    # the mug's outer diameter must fit inside the fully-open 85 mm jaw with real clearance
    assert 2.0 * config.OBJECT_RIM_RADIUS_M < 0.085


def test_grasp_quat_normalized():
    assert abs(np.linalg.norm(config.GRASP_TCP_OBJ_QUAT) - 1.0) < 1e-6


@needs_calibration
def test_grasp_aperture_within_joint_limits():
    # finger_joint live limits are [0, 0.82] rad; a pinch on the ~80 mm mug must sit well
    # below the clearance-carry era's 0.78 (which meant a nearly-shut jaw)
    assert 0.0 < config.GRIPPER_APERTURE_GRASP_RAD < 0.82


@needs_calibration
def test_grasp_chain_maps_rim_center_to_tcp_axis():
    # the documented z-up derivation: the rim-center axis point (u, v, h2)_obj lands on the
    # tool axis at z = GRASP_RIM_TCP_Z_M
    T_tcp_obj = make_T(config.GRASP_TCP_OBJ_POS, config.GRASP_TCP_OBJ_QUAT)
    rim_center_obj = np.array(
        [config.OBJECT_BODY_CENTER_XZ[0], config.OBJECT_BODY_CENTER_XZ[1], config.OBJECT_BBOX_HALF[2], 1.0]
    )
    p_tcp = (T_tcp_obj @ rim_center_obj)[:3]
    assert np.allclose(p_tcp, (0.0, 0.0, config.GRASP_RIM_TCP_Z_M), atol=1e-6), p_tcp


