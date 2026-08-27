# Copyright (c) 2026, dishsim project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Rack-state scenario machinery: config mutation + the config-hash tripwires.

Kit-free (config + geometry only). Every test restores the baseline scenario in a finally
block — apply_scenario mutates module state that other tests (and the live cache contract)
depend on. The frozen-hash test runs FIRST, against pristine import-time defaults.
"""

import os

import pytest

from dishsim import config
from dishsim.geometry import config_hash

BASELINE = "both_out"


def test_baseline_config_hash_is_frozen():
    # the exact value stamped in every shipped v1 baseline cache (scene_state.json);
    # a drifted FROZEN CACHE ANCHOR fails HERE, in any process, with no assets restored
    assert config_hash() == "3f66d1ac2c369f74"


def test_internal_placement_state():
    try:
        config.apply_scenario(config.PLACEMENT_STATE)
        assert config.RACK_LOWER_EXT_M == -0.20
        assert config.RACK_UPPER_EXT_M == 0.0
        assert config.state_params()["min_feasible_slots"] == 2  # shared >=2-slot choice bar
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
