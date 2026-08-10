# Copyright (c) 2026, dishsim project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Kit-free checks on the mode-aware slot derivation + goal poses (multi-object v1).

Uses a synthetic rack transform (identity at a fixed offset) instead of the geometry cache,
so these run before any Kit extraction. The geometry numbers come from config.RACK_GEN.
"""

import numpy as np
import pytest

from dishsim import config
from dishsim import placement
from dishsim import rack_gen

LOWER = "E_shelf_1_04"


@pytest.fixture(autouse=True)
def _restore():
    yield
    config.set_active_object("mug")


@pytest.fixture()
def rack_T(monkeypatch):
    T = np.eye(4)
    T[:3, 3] = (0.4, -0.1, -0.15)
    monkeypatch.setattr(placement, "_rack_T", lambda cache_dir: T)
    return T


def test_plate_slots_robot_bank_gaps(rack_T):
    """v4: the ROBOT bank (main plate keys) carries 3 gaps at the 40 mm pitch; robot discs
    release at the corridor midpoint between the two tine rows (seat_y 0.134)."""
    config.set_active_object("plate")
    slots = placement.derive_plate_slots("unused")
    assert len(slots) == 3
    p = config.RACK_GEN[LOWER]
    xs = np.asarray(rack_gen._plate_tine_xs(p))
    gaps = (xs[:-1] + xs[1:]) / 2.0
    for s, gx in zip(slots, gaps):
        assert s.mode == "plate_slot" and s.rack == "lower"
        assert abs(s.width_m - p["plate_tine_pitch"]) < 1e-9
        d = s.T_base_slot[:3, 3] - rack_T[:3, 3]
        assert abs(d[0] - gx) < 1e-9  # gap centers on the tine-pitch arithmetic
        assert abs(d[1] - 0.134) < 1e-9  # corridor midpoint between the rows (0.108/0.160)


def test_plate_goal_pose_is_vertical_disc(rack_T):
    config.set_active_object("plate")
    slots = placement.derive_plate_slots("unused")
    rng = np.random.default_rng(0)
    for T in placement.sample_goal_poses(slots[1], 8, rng):
        axis = T[:3, :3] @ np.array([0.0, 0.0, 1.0])  # disc face normal
        # the disc stands on edge: its normal is near-horizontal (~83-90 deg from vertical)
        ang_from_z = np.degrees(np.arccos(abs(axis[2])))
        assert ang_from_z > 75.0, ang_from_z
        # in-rack bounds: the disc bottom stays near the slot origin
        d = T[:3, 3] - slots[1].T_base_slot[:3, 3]
        assert abs(d[0]) < 0.02 and 0.0 < d[2] < 0.12


def test_basket_slots_inside_bays(rack_T):
    config.set_active_object("fork")
    slots = placement.derive_basket_slots("unused")
    p = config.RACK_GEN[LOWER]
    bays = rack_gen.basket_bays(p)
    assert len(slots) == len(bays) == 3
    for s, (x0, x1, y0, y1) in zip(slots, bays):
        d = s.T_base_slot[:3, 3] - rack_T[:3, 3]
        assert x0 < d[0] < x1 and y0 < d[1] < y1
        assert s.mode == "basket_drop" and s.rack == "basket"
    # goal poses hang the cutlery head-down well above the bay floor
    rng = np.random.default_rng(2)
    for T in placement.sample_goal_poses(slots[1], 6, rng):
        axis = T[:3, :3] @ np.array(config.OBJECT_AXIS_OBJ)
        assert axis[2] < -0.95  # head (+x_obj) points down
        d = T[:3, 3] - slots[1].T_base_slot[:3, 3]
        assert d[2] > 0.05  # release hover above the bay


def test_derive_slots_dispatch(rack_T, monkeypatch):
    # v4: 3 robot plate gaps; the bowl dispatches to floor_stand (its grid derivation reads
    # the cached rack mesh, so only the policy is asserted here)
    for name, expect in (("plate", 3), ("fork", 3)):
        config.set_active_object(name)
        assert len(placement.derive_slots("unused")) == expect
    assert config.OBJECTS["bowl"].placement.mode == "floor_stand"


def test_evaluate_placement_floor_stand_matches_v0():
    """The mode-aware evaluator reproduces the v0 standing criteria for the mug."""
    config.set_active_object("mug")
    T_slot = np.eye(4)
    T_slot[:3, 3] = (0.55, 0.0, -0.15)
    slot = placement.SlotFrame(0, T_slot, 0.117, "derived")
    # perfect standing pose: mug axis (+y_obj) up, bottom on the floor
    from scipy.spatial.transform import Rotation

    T = np.eye(4)
    T[:3, :3] = Rotation.from_euler("x", np.pi / 2).as_matrix()
    u, v = config.OBJECT_BODY_CENTER_XZ
    p_bottom_obj = np.array([u, -config.OBJECT_BBOX_HALF[1], v])
    T[:3, 3] = T_slot[:3, 3] - T[:3, :3] @ p_bottom_obj
    ev = placement.evaluate_placement(slot, T)
    assert ev["ok"] and ev["lateral_m"] < 1e-6 and ev["tilt_deg"] < 1e-4

    # 3 cm off laterally must fail
    T2 = T.copy()
    T2[0, 3] += 0.03
    assert not placement.evaluate_placement(slot, T2)["ok"]


def test_evaluate_placement_basket_inside_outside(rack_T):
    config.set_active_object("fork")
    slots = placement.derive_basket_slots("unused")
    slot = slots[0]
    from scipy.spatial.transform import Rotation

    R = Rotation.from_euler("y", np.pi / 2).as_matrix()  # head down
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = slot.T_base_slot[:3, 3] + np.array([0.0, 0.0, 0.03])
    assert placement.evaluate_placement(slot, T)["ok"]
    T_out = T.copy()
    T_out[1, 3] += 0.08  # outside the bay
    assert not placement.evaluate_placement(slot, T_out)["ok"]
