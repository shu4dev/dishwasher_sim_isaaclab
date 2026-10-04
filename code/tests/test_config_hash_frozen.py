# Copyright (c) 2026, dishsim project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""The frozen config-hash pin — one of the repo's two frozen-invariant tripwires.

Runs cache-free in any process. A drifted FROZEN CACHE ANCHOR in config.py (base frame,
HOME_Q, grasp/aperture anchors, rack params, machine geometry) fails HERE immediately,
instead of surfacing later as "cache is stale — rebake" on a box with restored assets.
"""

from dishsim import config
from dishsim.geometry import config_hash


def test_baseline_config_hash_is_frozen():
    config.apply_machine(config.MACHINE_BASELINE_NAME)  # hermetic: never trust test order
    config.set_active_object("mug")
    config.apply_scenario("both_out")
    # the exact value stamped in every shipped v1 baseline cache (scene_state.json)
    assert config_hash() == "3f66d1ac2c369f74"
