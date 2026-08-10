# Copyright (c) 2026, dishsim project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Kit-free sanity checks on the contact-pinch grasp constants in dishsim.config.

Pure numpy + config/transforms imports (no Kit, no scene module). The calibrated-constant
tests skip cleanly while the constants are still ``None`` (pre-calibration bootstrap state);
they activate once ``scripts/setup/calibrate_grasp.py`` results are frozen into config. Run with
the project venv:

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
    not CALIBRATED, reason="grasp constants not calibrated yet (run scripts/setup/calibrate_grasp.py)"
)


def test_jaw_clearance_geometry():
    # the mug's outer diameter must fit inside the fully-open 85 mm jaw with real clearance
    assert 2.0 * config.OBJECT_RIM_RADIUS_M < 0.085


def test_grasp_quat_normalized():
    assert abs(np.linalg.norm(config.GRASP_TCP_OBJ_QUAT) - 1.0) < 1e-6


def test_force_band_ordering():
    # static band is ordered; the dynamic cap must clear the band floor (a motion dip to the
    # static minimum must not abort) — but it is measured independently (wiggle peak x1.5)
    # and may sit below the generous static maximum
    assert 0.0 < config.GRIP_FORCE_MIN_N < config.GRIP_FORCE_MAX_N
    assert config.GRIP_FORCE_MIN_N < config.GRIP_FORCE_EXEC_MAX_N


def test_open_aperture_is_fully_open():
    assert config.GRIPPER_APERTURE_OPEN_RAD == 0.0


def test_pad_bodies_are_the_inner_finger_pair():
    assert set(config.GRIP_PAD_BODIES) == {"left_inner_finger", "right_inner_finger"}


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


@needs_calibration
def test_inner_finger_signs_shape():
    signs = config.GRIPPER_INNER_FINGER_SIGNS
    assert signs is not None
    assert set(signs) == {"left_inner_finger_joint", "right_inner_finger_joint"}
    assert all(v in (-1.0, 1.0) for v in signs.values())
