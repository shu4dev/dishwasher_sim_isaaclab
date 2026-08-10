# Copyright (c) 2026, dishsim project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""World <-> robot-base conversions under a NON-identity base placement.

Historically every world<->base conversion was a bare vector add/subtract of
``config.ROBOT_BASE_POS_W``, which silently assumed an identity base rotation. The named base
placements (``config.BASE_PLACEMENTS``) introduce yawed bases, so every conversion site now goes
through the full homogeneous transform. These tests pin both halves of that contract:

- under a yawed placement (bosch800 / side_high, +45 deg about z) the rotation is actually
  applied — translation-only math would be wrong by centimetres;
- at the frozen identity placement the full-transform math reproduces the legacy subtraction
  BYTE-EXACTLY (``np.array_equal``, not ``allclose``) — scipy's identity quaternion yields an
  exact identity matrix, so the v1 baseline numbers cannot drift.

Run with:
    PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /workspace/isaaclab/env_isaaclab/bin/python -m pytest \
        tests/test_base_transforms.py -v
"""

import numpy as np
import pytest

from dishsim import config
from dishsim.base_sweep import _bottom_offset, _stand_R
from dishsim.task import layout as L
from dishsim.transforms import T_inv, make_T

# ``dishsim.scene`` imports ``isaaclab.*`` at module scope and cannot be imported without a
# booted Kit app (the import raises ImportError from isaaclab.sim). The scene helpers are
# therefore mirrored here EXPRESSION-FOR-EXPRESSION so the math is covered Kit-free; the
# skip-guarded test at the bottom exercises the real functions whenever Kit is importable.


def _world_to_base(pos_w) -> np.ndarray:
    """Mirror of :func:`dishsim.scene.world_to_base` (kept identical by the Kit test below)."""
    T_base_w = T_inv(make_T(config.ROBOT_BASE_POS_W, config.ROBOT_BASE_QUAT_W))
    return T_base_w[:3, :3] @ np.asarray(pos_w, dtype=float) + T_base_w[:3, 3]


def _base_to_world(pos_b) -> np.ndarray:
    """Mirror of :func:`dishsim.scene.base_to_world`."""
    T_w_base = make_T(config.ROBOT_BASE_POS_W, config.ROBOT_BASE_QUAT_W)
    return T_w_base[:3, :3] @ np.asarray(pos_b, dtype=float) + T_w_base[:3, 3]


@pytest.fixture
def side_high():
    """bosch800 machine at its 'side_high' placement: base yawed +45 deg about world z."""
    config.apply_machine("bosch800")
    config.apply_base_placement("side_high")
    assert config.ROBOT_BASE_QUAT_W[2] != 0.0  # the placement this suite exists for
    yield
    # restore the frozen v1 baseline byte-identically (documented apply_machine contract)
    config.apply_machine("artvip_compact")


@pytest.fixture
def baseline():
    """The frozen v1 baseline: artvip_compact / 'front', identity base quaternion."""
    config.apply_machine("artvip_compact")
    assert tuple(config.ROBOT_BASE_QUAT_W) == (0.0, 0.0, 0.0, 1.0)
    yield
    config.apply_machine("artvip_compact")


# ---- world <-> base helpers --------------------------------------------------------------------


def test_round_trip_under_yawed_base(side_high):
    for p_w in ([0.9, -0.3, 0.85], [0.45, -0.55, 0.75], [1.2, 0.4, 0.0]):
        p_b = _world_to_base(p_w)
        np.testing.assert_allclose(_base_to_world(p_b), p_w, atol=1e-12)


def test_yawed_base_actually_rotates(side_high):
    # a point off the base origin: subtraction-only conversion would land elsewhere
    p_w = np.array([1.0, -0.2, 0.9])
    p_b = _world_to_base(p_w)
    shift_only = p_w - np.asarray(config.ROBOT_BASE_POS_W)
    assert not np.allclose(p_b, shift_only, atol=1e-3)
    # z is untouched by a yaw about z (what task/support.py's z-only conversion relies on)
    assert np.isclose(p_b[2], shift_only[2], atol=1e-12)


def test_identity_helpers_reproduce_exact_subtraction(baseline):
    p_w = np.array([0.731, -0.204, 0.912])
    assert np.array_equal(_world_to_base(p_w), p_w - np.asarray(config.ROBOT_BASE_POS_W))
    p_b = np.array([0.5, 0.25, 0.66])
    assert np.array_equal(_base_to_world(p_b), p_b + np.asarray(config.ROBOT_BASE_POS_W))


# ---- layout.surface_pose -----------------------------------------------------------------------


@pytest.mark.parametrize("cls", ["mug", "cup"])  # legacy Y-up stand and Z-up stand
def test_surface_pose_lands_at_the_requested_world_point(side_high, cls):
    spec = config.OBJECTS[cls]
    xy_w, yaw_deg, z_w, hover = (0.9, -0.3), 30.0, 0.87, 0.002
    T_base_obj = L.surface_pose(spec, xy_w, yaw_deg, z_w, hover=hover)

    # lift back to the world through the full base transform
    T_w_obj = make_T(config.ROBOT_BASE_POS_W, config.ROBOT_BASE_QUAT_W) @ T_base_obj
    R_stand = L.stand_rotation(spec, yaw_deg)  # the WORLD-frame stand rotation
    expected_pos_w = [xy_w[0], xy_w[1], z_w + hover + L.bottom_offset(spec, R_stand)]
    np.testing.assert_allclose(T_w_obj[:3, 3], expected_pos_w, atol=1e-12)
    # rotation must be conjugated into the base frame, not stored as-is: lifting it back must
    # recover the world stand rotation exactly (a translation-only port would be off by 45 deg)
    np.testing.assert_allclose(T_w_obj[:3, :3], R_stand, atol=1e-12)
    assert not np.allclose(T_base_obj[:3, :3], R_stand, atol=1e-3)


def test_surface_pose_identity_is_byte_exact(baseline):
    spec = config.OBJECTS["mug"]
    xy_w, yaw_deg, z_w, hover = (0.75, -0.1), 25.0, 0.87, 0.002
    R = L.stand_rotation(spec, yaw_deg)
    # the legacy (pre-yaw) formula, verbatim
    legacy = np.eye(4)
    legacy[:3, :3] = R
    pos_w = np.array([xy_w[0], xy_w[1], z_w + hover + L.bottom_offset(spec, R)])
    legacy[:3, 3] = pos_w - np.asarray(config.ROBOT_BASE_POS_W, dtype=float)
    assert np.array_equal(L.surface_pose(spec, xy_w, yaw_deg, z_w, hover=hover), legacy)


def test_top_z_is_invariant_across_base_placements():
    """The same physical placement reports the same world top-z from any base placement."""
    spec_args = ("cup", (0.9, -0.35), 10.0, 0.87)

    def item_here() -> L.LayoutItem:
        cls, xy_w, yaw, z_w = spec_args
        return L.LayoutItem(
            item_id=f"{cls}_00", object_class=cls, instance=0,
            T_base_obj=L.surface_pose(config.OBJECTS[cls], xy_w, yaw, z_w),
            xy_w=np.asarray(xy_w), yaw_deg=yaw, radius_m=0.05,
        )

    config.apply_machine("artvip_compact")
    try:
        top_front = L._top_z_w(item_here())
        config.apply_machine("bosch800")
        config.apply_base_placement("side_high")
        top_side = L._top_z_w(item_here())
    finally:
        config.apply_machine("artvip_compact")
    assert np.isclose(top_front, top_side, atol=1e-12)


# ---- reach_map / layout consistency ------------------------------------------------------------


def test_reach_map_idiom_matches_layout_surface_pose(side_high):
    """reach_map.py builds T_base_obj by conjugating a world pose (base_sweep's pure helpers);
    for a Z-up class its stand convention coincides with layout's, so the two constructions
    must agree pose-for-pose under a yawed base."""
    spec = config.OBJECTS["cup"]
    assert tuple(spec.axis_obj) == (0.0, 0.0, 1.0)  # conventions only coincide for Z-up
    x, y, yaw, top_z_w = 0.95, -0.45, 90.0, 0.914

    # reach_map.py's construction (post-fix), using base_sweep's pure functions
    R = _stand_R(spec, yaw)
    T_w_obj = np.eye(4)
    T_w_obj[:3, :3] = R
    T_w_obj[:3, 3] = (x, y, top_z_w + _bottom_offset(spec, R))
    T_base_w = T_inv(make_T(config.ROBOT_BASE_POS_W, config.ROBOT_BASE_QUAT_W))
    reach_map_pose = T_base_w @ T_w_obj

    layout_pose = L.surface_pose(spec, (x, y), yaw, top_z_w)
    np.testing.assert_allclose(reach_map_pose, layout_pose, atol=1e-12)


# ---- the real scene helpers (Kit only) ---------------------------------------------------------


def test_scene_helpers_round_trip_when_kit_is_available(side_high):
    # skip-marked: dishsim.scene needs a booted Kit app (module-scope isaaclab imports raise a
    # plain ImportError, not ModuleNotFoundError, so pytest.importorskip cannot be used); the
    # Kit-free mirrors above cover the identical expressions.
    try:
        from dishsim import scene as dscene
    except ImportError:
        pytest.skip("dishsim.scene needs a booted Kit app")
    p_w = np.array([0.9, -0.3, 0.85])
    p_b = dscene.world_to_base(p_w)
    np.testing.assert_allclose(p_b, _world_to_base(p_w), atol=0.0)
    np.testing.assert_allclose(dscene.base_to_world(p_b), p_w, atol=1e-12)
