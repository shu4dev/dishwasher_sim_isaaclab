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
    # perfect standing pose: the z-up scan mug stands as-authored, bottom on the floor
    T = np.eye(4)
    u, v = config.OBJECT_BODY_CENTER_XZ
    p_bottom_obj = np.array([u, v, -config.OBJECT_BBOX_HALF[2]])
    T[:3, 3] = T_slot[:3, 3] - p_bottom_obj
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


# ---------------------------------------------------------------------------------------------
# flat_lay_third (Bosch third-rack tray; apply_machine mutates config globals -> fixture)
# ---------------------------------------------------------------------------------------------

THIRD = "E_shelf_third"


@pytest.fixture()
def bosch_third(monkeypatch):
    """Bosch machine + a synthetic third-rack transform; restores the v1 baseline."""
    config.apply_machine("bosch800")
    T = np.eye(4)
    T[:3, 3] = (0.5, -0.2, 0.3)
    monkeypatch.setattr(placement, "_third_rack_T", lambda cache_dir: T)
    yield T
    config.apply_machine(config.MACHINE_BASELINE_NAME)


def test_flat_lay_slots_inside_tray(bosch_third):
    """Slot columns sit at each tray zone's x-center (long axis along rack x), rows inside
    the tray footprint, origins ON the zone's floor plane — channel slots exactly
    ``channel.drop`` below the wing plane."""
    config.set_active_object("fork")
    slots = placement.derive_flat_lay_slots("unused")
    p = config.RACK_GEN[THIRD]
    zones = {name: (span, z) for name, span, z in rack_gen.tray_zones(p)}
    assert {s.params["zone"] for s in slots} == {"wing_l", "channel", "wing_r"}
    z_wing = rack_gen.tray_floor_z(p)
    for s in slots:
        assert s.mode == "flat_lay_third" and s.rack == "third" and s.source == "derived"
        d = s.T_base_slot[:3, 3] - bosch_third[:3, 3]
        assert 0.0 < d[0] < p["footprint"][0] and 0.0 < d[1] < p["footprint"][1]
        (x0, x1, y0, y1), z_floor = zones[s.params["zone"]]
        assert abs(d[0] - (x0 + x1) / 2.0) < 1e-9  # zone x-center
        assert y0 < d[1] < y1
        assert abs(d[2] - z_floor) < 1e-9
        assert abs(s.width_m - (x1 - x0)) < 1e-9  # usable long-axis extent
        if s.params["zone"] == "channel":
            assert abs((z_wing - d[2]) - p["channel"]["drop"]) < 1e-9
    counts = {z: sum(1 for s in slots if s.params["zone"] == z) for z in zones}
    assert min(counts.values()) >= 3  # a genuine row per zone, channel included
    # geometry-derived labels stay unique for the 3-column grid
    assert len(placement.slot_names(slots)) == len(slots)


def test_flat_lay_dispatch_and_goal_pose(bosch_third, monkeypatch):
    """derive_slots dispatches on the flat_lay_third mode; sampled goal poses lie the piece
    FLAT (long axis near slot x, horizontal) at a low hover above the floor plane."""
    import dataclasses

    config.set_active_object("fork")
    spec = config.OBJECTS["fork"]
    monkeypatch.setitem(
        config.OBJECTS,
        "fork",
        dataclasses.replace(
            spec, placement=dataclasses.replace(spec.placement, mode="flat_lay_third", rack="third")
        ),
    )
    slots = placement.derive_slots("unused")
    assert len(slots) == len(placement.derive_flat_lay_slots("unused")) > 0
    slot = slots[len(slots) // 2]
    hover = config.placement_mode_params("flat_lay_third")["release_hover_m"]
    rng = np.random.default_rng(3)
    for T in placement.sample_goal_poses(slot, 8, rng):
        axis = T[:3, :3] @ np.array(spec.axis_obj)
        assert abs(axis[2]) < 0.1  # long axis near horizontal
        assert abs(axis[0]) > 0.99  # ... and along the rack x (width) direction
        d = T[:3, 3] - slot.T_base_slot[:3, 3]
        assert abs(d[0]) < 0.01 and abs(d[1]) < 0.01
        # center = hover + the piece's flat half-thickness (+/- the sampled pitch)
        assert abs(d[2] - (hover + spec.bbox_half[2])) < 0.01


def test_evaluate_placement_flat_lay(bosch_third):
    """Per-mode verdict: settled center near the slot, long axis near horizontal, bottom on
    the zone's floor plane — with the channel datum sitting ``drop`` below the wings."""
    config.set_active_object("fork")
    spec = config.OBJECTS["fork"]
    slots = placement.derive_flat_lay_slots("unused")
    wing = next(s for s in slots if s.params["zone"] == "wing_l")
    chan = next(s for s in slots if s.params["zone"] == "channel")
    from scipy.spatial.transform import Rotation

    for slot in (wing, chan):
        # perfect flat lay: as-authored orientation, resting on the slot's own floor plane
        T = np.eye(4)
        T[:3, 3] = slot.T_base_slot[:3, 3] + np.array([0.0, 0.0, spec.bbox_half[2]])
        ev = placement.evaluate_placement(slot, T)
        assert ev["ok"] and ev["lateral_m"] < 1e-6 and ev["tilt_deg"] < 1e-4
        assert abs(ev["bottom_height_m"]) < 1e-6

        T_off = T.copy()
        T_off[0, 3] += 0.05  # 5 cm off the slot center
        assert not placement.evaluate_placement(slot, T_off)["ok"]

        T_tilt = T.copy()  # 30 deg pitch: the long axis leaves horizontal (tol is 15)
        T_tilt[:3, :3] = Rotation.from_euler("y", np.radians(30.0)).as_matrix()
        assert not placement.evaluate_placement(slot, T_tilt)["ok"]

    # a piece stranded at WING height over a channel slot fails the bottom criterion
    # (drop 28 mm >> tol_bottom 15 mm): the two floor datums are genuinely distinct
    T_hi = np.eye(4)
    T_hi[:3, 3] = chan.T_base_slot[:3, 3] + np.array(
        [0.0, 0.0, config.RACK_GEN[THIRD]["channel"]["drop"] + spec.bbox_half[2]]
    )
    assert not placement.evaluate_placement(chan, T_hi)["ok"]
