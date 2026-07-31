# Copyright (c) 2026, dishsim project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Rack-state scenario machinery: config mutation, cache-dir mapping, hash coupling.

Kit-free (config + geometry only). Every test restores the baseline scenario in a finally
block — apply_scenario mutates module state that other tests (and the live cache contract)
depend on.
"""

import os

import pytest

from dishsim import config
from dishsim.geometry import config_hash

BASELINE = "lower_out"


def test_apply_scenario_mutates_and_restores():
    baseline_targets = dict(config.RACK_JOINT_TARGETS)
    try:
        config.apply_scenario("both_out")
        assert config.SCENARIO_NAME == "both_out"
        assert config.RACK_LOWER_EXT_M == -0.20
        assert config.RACK_UPPER_EXT_M == -0.20
        assert config.RACK_JOINT_TARGETS["PrismaticJoint_dishwasher_2_up"] == -0.20
        config.apply_scenario("both_in")
        assert config.RACK_LOWER_EXT_M == 0.0
        assert config.RACK_JOINT_TARGETS["PrismaticJoint_dishwasher_2_down"] == 0.0
    finally:
        config.apply_scenario(BASELINE)
    assert config.SCENARIO_NAME == BASELINE
    assert config.RACK_JOINT_TARGETS == baseline_targets


def test_unknown_scenario_raises():
    with pytest.raises(ValueError, match="unknown scenario"):
        config.apply_scenario("racks_sideways")
    assert config.SCENARIO_NAME == BASELINE, "a failed apply must not change state"


def test_scenario_cache_dir_mapping():
    try:
        assert config.scenario_cache_dir() == config.CACHE_DIR
        assert config.scenario_cache_dir("both_in").endswith(os.path.join("cache", "scenarios", "both_in"))
        config.apply_scenario("both_out")
        assert config.scenario_cache_dir().endswith(os.path.join("cache", "scenarios", "both_out"))
    finally:
        config.apply_scenario(BASELINE)


def test_scenario_media_dir_mapping():
    try:
        assert config.scenario_media_dir("F").endswith(os.path.join("media", "F"))
        config.apply_scenario("both_in")
        assert config.scenario_media_dir("F").endswith(os.path.join("media", "F", "both_in"))
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
