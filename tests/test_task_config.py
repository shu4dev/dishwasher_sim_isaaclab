# Copyright (c) 2026, dishsim project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Two configuration surfaces: rack-state resolution and named goal slots.

The properties worth protecting here are the ones a casual implementation gets wrong in ways
that stay invisible: slot names that quietly re-point to different cells when the grid is
retuned, and a rack-state spec resolving to the wrong named cache.

Run with:
    PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /workspace/isaaclab/env_isaaclab/bin/python -m pytest \
        tests/test_task_config.py -v
"""

import numpy as np
import pytest

from dishsim import config, placement



# ---- rack state ------------------------------------------------------------------------------------


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


# ---- cameras -------------------------------------------------------------------------------------------


def test_the_configured_video_camera_exists():
    assert config.TASK["video_camera"] in set(config.CAMERAS) | {"episode"}
