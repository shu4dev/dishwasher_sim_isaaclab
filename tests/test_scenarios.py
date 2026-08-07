# Copyright (c) 2026, dishsim project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Rack-state scenario machinery: config mutation, cache-dir mapping, hash coupling.

Kit-free (config + geometry only). The scenario set is exactly the two real robot-facing rack
states (both racks fully out / fully in); "both_out" is the baseline mapping to the legacy
cache/media dirs. Every test restores the baseline scenario in a finally block —
apply_scenario mutates module state that other tests (and the live cache contract) depend on.
"""

import os

import pytest

from dishsim import config
from dishsim.geometry import config_hash

BASELINE = "both_out"


def test_scenario_set_is_exactly_the_two_states():
    assert set(config.SCENARIOS) == {"both_out", "both_in"}
    assert config.SCENARIOS["both_out"]["rack_lower_m"] == -0.20
    assert config.SCENARIOS["both_out"]["rack_upper_m"] == -0.20
    assert config.SCENARIOS["both_in"]["rack_lower_m"] == 0.0
    assert config.SCENARIOS["both_in"]["rack_upper_m"] == 0.0


def test_rack_actions_converge_to_the_placement_state():
    """Each scenario's rack_action must transform its initial state into the shared placement
    state (lower out, upper in) — that is the whole point of the choreography."""
    place = config.INTERNAL_STATES[config.PLACEMENT_STATE]
    for name, sc in config.SCENARIOS.items():
        state = {"rack_lower_m": sc["rack_lower_m"], "rack_upper_m": sc["rack_upper_m"]}
        action = sc["rack_action"]
        key = "rack_upper_m" if action["joint"].endswith("_up") else "rack_lower_m"
        state[key] = action["to"]
        assert state == {k: place[k] for k in state}, f"{name}: rack_action does not reach placement"
        expected_mode = "pull" if action["to"] < state["rack_lower_m"] or action["to"] < 0 else "push"
        assert action["mode"] in ("pull", "push")


def test_internal_placement_state():
    try:
        config.apply_scenario(config.PLACEMENT_STATE)
        assert config.RACK_LOWER_EXT_M == -0.20
        assert config.RACK_UPPER_EXT_M == 0.0
        assert config.state_params()["min_feasible_slots"] == 2  # v3: basket eats a column
        assert config.scenario_cache_dir().endswith(os.path.join("cache", "scenarios", "placement"))
    finally:
        config.apply_scenario(BASELINE)


def test_apply_scenario_mutates_and_restores():
    baseline_targets = dict(config.RACK_JOINT_TARGETS)
    try:
        config.apply_scenario("both_in")
        assert config.SCENARIO_NAME == "both_in"
        assert config.RACK_LOWER_EXT_M == 0.0
        assert config.RACK_UPPER_EXT_M == 0.0
        assert config.RACK_JOINT_TARGETS["PrismaticJoint_dishwasher_2_down"] == 0.0
        config.apply_scenario("both_out")
        assert config.RACK_LOWER_EXT_M == -0.20
        assert config.RACK_JOINT_TARGETS["PrismaticJoint_dishwasher_2_up"] == -0.20
    finally:
        config.apply_scenario(BASELINE)
    assert config.SCENARIO_NAME == BASELINE
    assert config.RACK_JOINT_TARGETS == baseline_targets


def test_unknown_scenario_raises():
    with pytest.raises(ValueError, match="unknown scenario"):
        config.apply_scenario("lower_out")  # the old baseline no longer exists
    assert config.SCENARIO_NAME == BASELINE, "a failed apply must not change state"


def test_scenario_cache_dir_mapping():
    try:
        assert config.scenario_cache_dir() == config.CACHE_DIR  # baseline -> legacy cache
        assert config.scenario_cache_dir("both_in").endswith(os.path.join("cache", "scenarios", "both_in"))
        config.apply_scenario("both_in")
        assert config.scenario_cache_dir().endswith(os.path.join("cache", "scenarios", "both_in"))
    finally:
        config.apply_scenario(BASELINE)


def test_scenario_media_dir_mapping():
    try:
        assert config.scenario_media_dir("trials").endswith(os.path.join("media", "trials"))
        config.apply_scenario("both_in")
        assert config.scenario_media_dir("trials").endswith(os.path.join("media", "trials", "both_in"))
    finally:
        config.apply_scenario(BASELINE)


def test_config_hash_distinct_per_scenario_and_baseline_stable():
    before = config_hash()
    hashes = {}
    try:
        for name in config.SCENARIOS:
            config.apply_scenario(name)
            hashes[name] = config_hash()
    finally:
        config.apply_scenario(BASELINE)
    assert len(set(hashes.values())) == len(config.SCENARIOS), f"hash collision: {hashes}"
    assert hashes[BASELINE] == before, "baseline scenario must reproduce the pre-test hash"
    assert config_hash() == before, "restore must reproduce the live cache's hash"


def test_config_hash_covers_the_countertop():
    before = config_hash()
    old = config.COUNTERTOP_CENTER_W
    config.COUNTERTOP_CENTER_W = (old[0], old[1], old[2] + 0.001)
    try:
        assert config_hash() != before, "config_hash blind to the countertop geometry"
    finally:
        config.COUNTERTOP_CENTER_W = old
    assert config_hash() == before
