# Copyright (c) 2026, dishsim project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Kit-free tests for the machine selector (apply_machine / apply_base_placement).

Covers: round-trip byte-stability of every machine-mutable global and of config_hash,
per-machine / per-state / per-placement hash separation, the conditional "machine" and
"base_placement" hash keys against the frozen v1 manifest, the machine-aware cache and
media layout vs. the v1 legacy carve-outs, named base placements, third-rack plumbing
through apply_scenario / resolve_rack_state, and the per-machine scalar overrides
(RACK_SLIDE_STEPS, RACK_TRAVEL_LIMITS_M). Every test restores the baseline machine —
apply_machine("artvip_compact") rewrites all mutated globals back, including the active
scenario and base placement.
"""

import json
import os

import pytest

from dishsim import config
from dishsim.geometry import config_hash

BASELINE = config.MACHINE_BASELINE_NAME  # "artvip_compact"
BOSCH = "bosch800"

from conftest import needs_cache

MANIFEST = os.path.join(config.CACHE_DIR, "scene_state.json")

#: base-placement globals rewritten by apply_base_placement + the scenario-owned targets;
#: together with config._MACHINE_MUTABLE this is every global the selector touches.
_PLACEMENT_MUTABLE = (
    "ROBOT_BASE_POS_W",
    "ROBOT_BASE_QUAT_W",
    "PEDESTAL_SIZE",
    "PEDESTAL_POS_W",
    "RACK_JOINT_TARGETS",
)


def _selector_state() -> dict:
    return {k: getattr(config, k) for k in config._MACHINE_MUTABLE + _PLACEMENT_MUTABLE}


@pytest.fixture(autouse=True)
def _restore_machine():
    yield
    config.apply_machine(BASELINE)
    config.apply_scenario("both_out")  # apply_machine keeps a surviving scenario name


# ---------------------------------------------------------------------------------------------
# 1. round-trip byte-stability
# ---------------------------------------------------------------------------------------------


def test_apply_machine_round_trip_is_byte_stable():
    """Switching to bosch800 and back must reproduce every mutated global and the hash."""
    h0 = config_hash()
    before = _selector_state()
    config.apply_machine(BOSCH)
    assert config.MACHINE == BOSCH
    assert config_hash() != h0
    config.apply_machine(BASELINE)
    assert config.MACHINE == BASELINE
    assert config_hash() == h0, "baseline machine must reproduce the v1 hash byte-identically"
    after = _selector_state()
    for key, value in before.items():
        assert after[key] == value, f"{key} not restored by apply_machine({BASELINE!r})"


# ---------------------------------------------------------------------------------------------
# 2. hash separation per machine / state / MACHINE_GEN value
# ---------------------------------------------------------------------------------------------


def test_config_hash_distinct_per_machine():
    h_base = config_hash()
    config.apply_machine(BOSCH)
    assert config_hash() != h_base


def test_config_hash_distinct_per_bosch_state():
    config.apply_machine(BOSCH)
    hashes = {}
    for state in ("both_out", "placement", "third_out"):
        config.apply_scenario(state)
        hashes[state] = config_hash()
    assert len(set(hashes.values())) == 3, f"hash collision: {hashes}"


def test_machine_gen_perturbation_changes_hash():
    """The "machine" hash key must cover the parametric geometry, not just the name."""
    config.apply_machine(BOSCH)
    before = config_hash()
    old_body = config.MACHINE_GEN[BOSCH]["body"]
    config.MACHINE_GEN[BOSCH]["body"] = {**old_body, "w": old_body["w"] + 0.001}
    try:
        assert config_hash() != before, "config_hash blind to MACHINE_GEN geometry"
    finally:
        config.MACHINE_GEN[BOSCH]["body"] = old_body
    assert config_hash() == before


# ---------------------------------------------------------------------------------------------
# 3. conditional keys: the baseline hash is the pre-selector frozen value
# ---------------------------------------------------------------------------------------------


@needs_cache
def test_baseline_hash_matches_frozen_manifest():
    """With the baseline machine + "front" placement neither conditional key fires, so the
    live hash must equal the value frozen in the baked v1 cache manifest."""
    assert config.MACHINE == BASELINE
    assert config.BASE_PLACEMENT == "front"
    with open(MANIFEST) as f:
        manifest = json.load(f)
    assert config_hash() == manifest["config_hash"]


# ---------------------------------------------------------------------------------------------
# 4. cache/media layout: fully regular under bosch800, v1 carve-outs after restore
# ---------------------------------------------------------------------------------------------


def test_bosch_cache_dirs_fully_regular_for_every_object():
    config.apply_machine(BOSCH)
    root = os.path.join(config.ASSETS_DIR, "cache", "machines", BOSCH) + os.sep
    for obj in sorted(config.OBJECTS):
        for state in ("both_out", "placement"):
            d = config.scenario_cache_dir(state, object_name=obj)
            assert d.startswith(root), (obj, state, d)
            assert d.endswith(os.path.join("objects", obj, state))


def test_restored_cache_dirs_keep_v1_carveouts():
    config.apply_machine(BOSCH)
    config.apply_machine(BASELINE)
    assert config.scenario_cache_dir("both_out", object_name="mug") == config.CACHE_DIR
    assert config.scenario_cache_dir("placement", object_name="mug") == os.path.join(
        config.ASSETS_DIR, "cache", "scenarios", "placement"
    )
    assert config.scenario_cache_dir("both_out", object_name="bowl") == os.path.join(
        config.ASSETS_DIR, "cache", "objects", "bowl", "both_out"
    )


def test_media_dirs_machine_aware_and_restored():
    config.apply_machine(BOSCH)
    assert config.scenario_media_dir("trials").endswith(
        os.path.join("media", "trials", BOSCH, "both_out")
    )
    assert config.scenario_media_dir("trials", "third_out").endswith(
        os.path.join("media", "trials", BOSCH, "third_out")
    )
    config.apply_machine(BASELINE)
    assert config.scenario_media_dir("trials").endswith(os.path.join("media", "trials"))
    assert config.scenario_media_dir("trials", "placement").endswith(
        os.path.join("media", "trials", "placement")
    )


# ---------------------------------------------------------------------------------------------
# 5. named base placements
# ---------------------------------------------------------------------------------------------


def test_apply_base_placement_rewrites_base_and_pedestal():
    config.apply_machine(BOSCH)
    # bosch default is the neutral BAKE REFERENCE (v1-identical "front"); the elevated
    # mounts are sweep candidates applied explicitly
    assert config.BASE_PLACEMENT == "front"
    assert config.ROBOT_BASE_POS_W == config.BASE_PLACEMENTS[BOSCH]["front"]["base_pos_w"]
    config.apply_base_placement("side_high")
    side = config.BASE_PLACEMENTS[BOSCH]["side_high"]
    assert config.ROBOT_BASE_POS_W == side["base_pos_w"]
    config.apply_base_placement("front_high")
    front_high = config.BASE_PLACEMENTS[BOSCH]["front_high"]
    assert config.BASE_PLACEMENT == "front_high"
    assert config.ROBOT_BASE_POS_W == front_high["base_pos_w"]
    assert config.ROBOT_BASE_QUAT_W == front_high["base_quat_w"]
    assert config.PEDESTAL_SIZE == front_high["pedestal_size"]
    assert config.PEDESTAL_POS_W == front_high["pedestal_pos_w"]


def test_unknown_base_placement_raises_listing_choices():
    with pytest.raises(ValueError, match="unknown base placement.*choices"):
        config.apply_base_placement("roof")
    assert config.BASE_PLACEMENT == "front", "a failed apply must not change state"


def test_baseline_machine_has_only_the_frozen_front_placement():
    assert set(config.BASE_PLACEMENTS[BASELINE]) == {"front"}
    with pytest.raises(ValueError, match="choices"):
        config.apply_base_placement("side_high")  # bosch-only name under the baseline


def test_config_hash_distinct_per_base_placement():
    """Non-"front" placements key the hash (name + quaternion), so two bosch placements
    with identical rack state must still hash apart."""
    config.apply_machine(BOSCH)
    h_side = config_hash()
    config.apply_base_placement("front_high")
    assert config_hash() != h_side


# ---------------------------------------------------------------------------------------------
# 6. third rack
# ---------------------------------------------------------------------------------------------


def test_third_rack_joint_targets_per_machine():
    assert config.HAS_THIRD_RACK is False
    assert config.RACK_THIRD_JOINT not in config.RACK_JOINT_TARGETS
    config.apply_machine(BOSCH)
    assert config.HAS_THIRD_RACK is True
    assert config.RACK_JOINT_TARGETS[config.RACK_THIRD_JOINT] == 0.0  # both_out: no third_m
    config.apply_scenario("third_out")
    assert config.RACK_JOINT_TARGETS[config.RACK_THIRD_JOINT] == -0.51
    assert config.RACK_JOINT_TARGETS["PrismaticJoint_dishwasher_2_down"] == 0.0
    assert config.RACK_JOINT_TARGETS["PrismaticJoint_dishwasher_2_up"] == 0.0
    config.apply_machine(BASELINE)
    assert config.RACK_THIRD_JOINT not in config.RACK_JOINT_TARGETS


def test_resolve_rack_state_with_third_extension():
    config.apply_machine(BOSCH)
    spec = {"lower_m": 0.0, "upper_m": 0.0, "third_m": -0.51}
    assert config.resolve_rack_state(spec) == "third_out"
    # omitted third defaults to stowed (0.0) and matches the bosch both_in scenario
    assert config.resolve_rack_state({"lower_m": 0.0, "upper_m": 0.0}) == "both_in"
    with pytest.raises(ValueError, match="outside the prismatic joint limits"):
        config.resolve_rack_state({"lower_m": 0.0, "upper_m": 0.0, "third_m": -0.60})


# ---------------------------------------------------------------------------------------------
# 7. selector validation
# ---------------------------------------------------------------------------------------------


def test_unknown_machine_raises_listing_choices():
    with pytest.raises(ValueError, match="unknown machine.*choices"):
        config.apply_machine("nope")
    assert config.MACHINE == BASELINE, "a failed apply must not change state"


def test_machine_override_keys_are_validated():
    """An override targeting a non-machine-mutable global must be rejected before any
    global is rewritten (the check runs ahead of the overlay)."""
    config.MACHINES["_bogus"] = {"NOT_A_GLOBAL": 1}
    try:
        with pytest.raises(ValueError, match="unknown globals"):
            config.apply_machine("_bogus")
    finally:
        del config.MACHINES["_bogus"]
    assert config.MACHINE == BASELINE


# ---------------------------------------------------------------------------------------------
# 8. per-machine scalar overrides
# ---------------------------------------------------------------------------------------------


def test_rack_slide_steps_and_travel_limits_override():
    config.apply_machine(BOSCH)
    assert config.RACK_SLIDE_STEPS == 672
    assert config.RACK_TRAVEL_LIMITS_M == (-0.56, 0.0)
    assert config.RACK_TRAVEL_LIMITS_BY_JOINT_M == {
        "PrismaticJoint_dishwasher_2_down": (-0.56, 0.0),
        "PrismaticJoint_dishwasher_2_up": (-0.51, 0.0),
        "PrismaticJoint_dishwasher_2_third": (-0.51, 0.0),
    }
    config.apply_machine(BASELINE)
    assert config.RACK_SLIDE_STEPS == 240
    assert config.RACK_TRAVEL_LIMITS_M == (-0.20, 0.0)
    assert set(config.RACK_TRAVEL_LIMITS_BY_JOINT_M) == {
        "PrismaticJoint_dishwasher_2_down",
        "PrismaticJoint_dishwasher_2_up",
    }
    assert all(
        v == config.RACK_TRAVEL_LIMITS_M for v in config.RACK_TRAVEL_LIMITS_BY_JOINT_M.values()
    )
