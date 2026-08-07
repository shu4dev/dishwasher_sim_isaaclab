# Copyright (c) 2026, dishsim project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""The three configuration surfaces: spawn composition, rack state, named goal slots, cameras.

The properties worth protecting here are the ones a casual implementation gets wrong in ways
that stay invisible: a composition that silently changes when a config key is reordered, slot
names that quietly re-point to different cells when the grid is retuned, and a lens change that
alters framing without anyone noticing.

Run with:
    PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /workspace/isaaclab/env_isaaclab/bin/python -m pytest \
        tests/test_task_config.py -v
"""

import math

import numpy as np
import pytest

from dishsim import config, placement
from dishsim.task import layout as L


def rng(seed=0):
    return np.random.default_rng(seed)


# ---- spawn composition ---------------------------------------------------------------------------


def test_exact_counts_are_honoured():
    draw = L.resolve_spawn_counts({"cup": 3, "mug": 1}, rng())
    assert sorted(draw) == ["cup", "cup", "cup", "mug"]


def test_a_range_stays_within_its_bounds_and_is_inclusive_at_both_ends():
    seen = set()
    for seed in range(60):
        n = len(L.resolve_spawn_counts({"cup": (2, 4)}, rng(seed)))
        assert 2 <= n <= 4
        seen.add(n)
    assert seen == {2, 3, 4}, f"range never reached its ends: {sorted(seen)}"


def test_a_degenerate_range_is_an_exact_count():
    assert len(L.resolve_spawn_counts({"cup": (3, 3)}, rng())) == 3


def test_zero_is_allowed_and_contributes_nothing():
    assert L.resolve_spawn_counts({"cup": 2, "mug": 0}, rng()) == ["cup", "cup"]


def test_the_same_seed_reproduces_composition_and_order():
    a = L.resolve_spawn_counts({"cup": (1, 4), "mug": (1, 2)}, rng(11))
    b = L.resolve_spawn_counts({"cup": (1, 4), "mug": (1, 2)}, rng(11))
    assert a == b


def test_key_order_does_not_change_the_draw():
    """A spec is a set of constraints. `--spawn a=1,b=2` must equal `--spawn b=2,a=1`.

    With a range the count is itself an rng draw, so iterating in dict/CLI order would mean
    adding a key — or typing the terms in another order — silently changed every existing seed.
    """
    for seed in range(12):
        a = L.resolve_spawn_counts({"cup": (2, 4), "mug": 1, "tumbler": (0, 2)}, rng(seed))
        b = L.resolve_spawn_counts({"tumbler": (0, 2), "cup": (2, 4), "mug": 1}, rng(seed))
        assert a == b, f"seed {seed}: {a} != {b}"


def test_the_expanded_list_is_shuffled_not_grouped_by_class():
    """Placement order is draw order, so an unshuffled list always favours one class."""
    orders = {tuple(L.resolve_spawn_counts({"cup": 3, "mug": 3}, rng(s))) for s in range(25)}
    grouped = tuple(["cup"] * 3 + ["mug"] * 3)
    assert len(orders) > 1, "composition never varies in order — the shuffle is not running"
    assert not (len(orders) == 1 and grouped in orders)


def test_counts_survive_the_shuffle():
    for seed in range(10):
        draw = L.resolve_spawn_counts({"cup": 3, "mug": 2}, rng(seed))
        assert draw.count("cup") == 3 and draw.count("mug") == 2


@pytest.mark.parametrize("bad", [
    {"nonexistent_class": 1},
    {"cup": -1},
    {"cup": (4, 2)},
    {"cup": (1, 2, 3)},
    {},
])
def test_malformed_specs_fail_loudly(bad):
    with pytest.raises(ValueError):
        L.resolve_spawn_counts(bad, rng())


def test_parse_spawn_arg_handles_counts_and_ranges():
    assert L.parse_spawn_arg("cup=2-4, mug=1") == {"cup": (2, 4), "mug": 1}


@pytest.mark.parametrize("bad", ["cup", "cup=x", "cup=1-x", ""])
def test_parse_spawn_arg_rejects_junk(bad):
    with pytest.raises(ValueError):
        L.parse_spawn_arg(bad)


# ---- rack state ------------------------------------------------------------------------------------


def test_a_state_name_resolves_to_itself():
    assert config.resolve_rack_state("placement") == "placement"


def test_extensions_resolve_to_the_matching_named_state():
    assert config.resolve_rack_state({"lower_m": -0.20, "upper_m": 0.0}) == "placement"


def test_both_racks_are_addressable_independently():
    """The two prismatic joints are separate, so the two fields must select different states."""
    a = config.resolve_rack_state({"lower_m": -0.20, "upper_m": 0.0})
    b = config.resolve_rack_state({"lower_m": 0.0, "upper_m": 0.0})
    assert a != b


def test_an_ambiguous_extension_prefers_the_usable_internal_state():
    """(-0.20, -0.20) matches both `both_out` and `placement_open`, which even share a hash.

    `both_out` carries a rack action and is rejected by the multi-object runner, so the internal
    state is the only useful answer — but the tie must be resolved deliberately, not by dict order.
    """
    assert config.resolve_rack_state({"lower_m": -0.20, "upper_m": -0.20}) == "placement_open"


def test_an_unbaked_combination_says_how_to_bake_it():
    with pytest.raises(RuntimeError) as e:
        config.resolve_rack_state({"lower_m": -0.20, "upper_m": -0.10})
    msg = str(e.value)
    assert "INTERNAL_STATES" in msg and "build_state.py" in msg
    assert "-0.1" in msg


@pytest.mark.parametrize("bad", [
    {"lower_m": -0.5, "upper_m": 0.0},
    {"lower_m": 0.0, "upper_m": 0.5},
])
def test_extensions_outside_the_joint_limits_are_rejected(bad):
    with pytest.raises(ValueError, match="joint limits"):
        config.resolve_rack_state(bad)


@pytest.mark.parametrize("bad", ["not_a_state", {"lower_m": -0.2}, 42])
def test_malformed_rack_specs_fail_loudly(bad):
    with pytest.raises(ValueError):
        config.resolve_rack_state(bad)


def test_the_configured_default_rack_state_resolves():
    assert config.resolve_rack_state() in {**config.SCENARIOS, **config.INTERNAL_STATES}


# ---- named slots -------------------------------------------------------------------------------------


def make_grid_slots(n_depth=3, n_lateral=5, pitch=0.06):
    """Synthetic floor_stand slots on a rack-aligned grid (row-major, like the real derivation)."""
    R = np.array([[0.0, 1.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])
    slots = []
    for j in range(n_depth):
        for i in range(n_lateral):
            T = np.eye(4)
            T[:3, :3] = R
            T[:3, 3] = R @ np.array([i * pitch, j * pitch, 0.0])
            slots.append(placement.SlotFrame(slot_id=len(slots), T_base_slot=T,
                                             width_m=0.1, source="derived", mode="floor_stand"))
    return slots


def test_names_are_unique_and_cover_every_slot():
    slots = make_grid_slots()
    names = placement.slot_names(slots)
    assert len(names) == len(slots)
    assert set(names.values()) == {s.slot_id for s in slots}


def test_the_grid_is_labelled_by_depth_and_side():
    names = placement.slot_names(make_grid_slots())
    assert names["near_centre"] == 2
    assert names["mid_centre"] == 7
    assert {"near_left1", "near_right1", "far_left2", "far_right2"} <= set(names)


def test_names_survive_a_changed_grid_pitch_where_ids_would_not():
    """The whole reason names are derived: retuning the grid silently re-points ids.

    ``near_centre`` must keep meaning the middle of the nearest row at any pitch.
    """
    for pitch in (0.05, 0.06, 0.08):
        names = placement.slot_names(make_grid_slots(pitch=pitch))
        assert names["near_centre"] == 2


def test_names_survive_float_noise_in_a_row():
    """Real slot coordinates within one row differ in the last ulp; an exact sort interleaves
    the rows and mislabels them."""
    slots = make_grid_slots()
    for k, s in enumerate(slots):
        s.T_base_slot[0, 3] = np.nextafter(s.T_base_slot[0, 3], -np.inf if k % 2 else np.inf)
    names = placement.slot_names(slots)
    assert names["near_centre"] == 2 and names["mid_centre"] == 7


def test_an_even_column_count_has_no_centre():
    names = placement.slot_names(make_grid_slots(n_lateral=4))
    assert "near_centre" not in names
    assert {"near_left2", "near_left1", "near_right1", "near_right2"} <= set(names)


def test_one_dimensional_modes_get_their_own_vocabulary():
    """A 3x5 vocabulary would lie about three of the four modes."""
    R = np.eye(3)
    gaps = []
    for i in range(11):  # plate tine gaps vary along the rack's lateral axis only
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = (i * 0.03, 0.0, 0.0)
        gaps.append(placement.SlotFrame(slot_id=i, T_base_slot=T, width_m=0.03,
                                        source="derived", mode="plate_slot"))
    names = placement.slot_names(gaps)
    assert names["gap_centre"] == 5
    assert all(n.startswith("gap_") for n in names)


def test_slot_names_stamps_the_label_onto_the_slot():
    slots = make_grid_slots()
    placement.slot_names(slots)
    assert slots[2].name == "near_centre"


def test_slot_frame_name_round_trips_through_json():
    slots = make_grid_slots()
    placement.slot_names(slots)
    assert placement.SlotFrame.from_json(slots[2].to_json()).name == "near_centre"


def test_empty_slot_list_is_handled():
    assert placement.slot_names([]) == {}


# ---- cameras -------------------------------------------------------------------------------------------


def test_the_default_lens_reproduces_the_previous_hardcoded_optics():
    lens = config.camera_lens("front")
    assert lens["focal_length"] == 24.0 and lens["horizontal_aperture"] == 20.955


def test_every_configured_camera_resolves_a_lens():
    for name in config.CAMERAS:
        lens = config.camera_lens(name)
        assert lens["focal_length"] > 0 and lens["horizontal_aperture"] > 0


def _vfov_deg(lens, hw=config.CAMERA_HW):
    return 2 * math.degrees(math.atan((lens["horizontal_aperture"] * hw[0] / hw[1])
                                      / (2 * lens["focal_length"])))


def test_the_episode_camera_is_wider_than_the_default():
    assert _vfov_deg(config.EPISODE_CAMERA["lens"]) > _vfov_deg(config.CAMERA_LENS_DEFAULT)


#: The content an episode video must never crop: robot working volume, the whole spawn
#: rectangle, the machine, its open door and the extended racks.
MUST_FRAME_LO = np.array([-0.20, -0.62, 0.0])
MUST_FRAME_HI = np.array([1.06, 0.50, 1.00])


def _frame_fractions(eye, target, lens, hw=config.CAMERA_HW):
    """Normalised image extent of the must-frame box: |u|,|v| <= 1 means fully in frame."""
    tan_h = lens["horizontal_aperture"] / (2 * lens["focal_length"])
    tan_v = (lens["horizontal_aperture"] * hw[0] / hw[1]) / (2 * lens["focal_length"])
    eye, target = np.asarray(eye, dtype=float), np.asarray(target, dtype=float)
    fwd = target - eye
    fwd /= np.linalg.norm(fwd)
    right = np.cross(fwd, [0.0, 0.0, 1.0])
    right /= np.linalg.norm(right)
    up = np.cross(right, fwd)
    us, vs = [], []
    for x in (MUST_FRAME_LO[0], MUST_FRAME_HI[0]):
        for y in (MUST_FRAME_LO[1], MUST_FRAME_HI[1]):
            for z in (MUST_FRAME_LO[2], MUST_FRAME_HI[2]):
                v = np.array([x, y, z]) - eye
                depth = float(v @ fwd)
                assert depth > 0, "the must-frame box is behind the camera"
                us.append(abs(float(v @ right)) / depth / tan_h)
                vs.append(abs(float(v @ up)) / depth / tan_v)
    return max(us), max(vs)


def test_the_episode_camera_contains_the_whole_scene():
    """Projected corners, not a bounding sphere: the sphere is orientation-agnostic and lands
    the eye ~15% further out than necessary, wasting frame on empty floor."""
    u, v = _frame_fractions(config.EPISODE_CAMERA["eye"], config.EPISODE_CAMERA["target"],
                            config.EPISODE_CAMERA["lens"])
    assert u <= 1.0 and v <= 1.0, f"episode view clips the scene (u={u:.2f}, v={v:.2f})"


def test_the_episode_camera_is_not_needlessly_far_back():
    """A view that contains everything with room to spare wastes pixels on the floor."""
    u, v = _frame_fractions(config.EPISODE_CAMERA["eye"], config.EPISODE_CAMERA["target"],
                            config.EPISODE_CAMERA["lens"])
    assert max(u, v) >= 0.80, (
        f"the scene fills only {100 * max(u, v):.0f}% of the frame — move the eye closer"
    )


def test_the_old_front_camera_demonstrably_crops_the_scene():
    """Guards the diagnosis itself: `front` is the view that cropped the countertop out."""
    eye, target = config.CAMERAS["front"]
    u, v = _frame_fractions(eye, target, config.camera_lens("front"))
    assert max(u, v) > 1.0, "front no longer crops — the reported problem has changed"


def test_the_configured_video_camera_exists():
    assert config.TASK["video_camera"] in set(config.CAMERAS) | {"episode"}
