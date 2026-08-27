# Copyright (c) 2026, dishsim project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""The teleport-feasibility primitive: CollisionWorld.object_in_collision.

Runs against the baseline cache (mug @ both_out), so it needs the restored assets.
"""

import numpy as np
from conftest import needs_cache

from dishsim import config, placement
from dishsim.collision_world import CollisionWorld, load_object_pieces


def _release_pose(slot, hover_extra=0.0):
    hover = float(config.placement_mode_params(slot.mode)["release_hover_m"]) + hover_extra
    return placement.object_pose_for_mode(slot, 0.0, np.zeros(2), np.zeros(2), hover)


@needs_cache
def test_release_pose_is_free_and_buried_pose_is_not():
    world = CollisionWorld()
    pieces = load_object_pieces(config.CACHE_DIR)
    slots = placement.derive_slots()
    assert slots, "baseline cache derived no slots"
    free = [s for s in slots if not world.object_in_collision(pieces, _release_pose(s))]
    assert free, "no slot accepts the object at its release pose in the empty machine"
    # buried 10 cm below the wire floor the object must hit the rack/statics
    buried = _release_pose(free[0], hover_extra=-0.10)
    hit, pairs = world.object_in_collision(pieces, buried, return_pairs=True)
    assert hit and pairs


@needs_cache
def test_placed_neighbour_blocks_the_same_slot():
    world = CollisionWorld()
    pieces = load_object_pieces(config.CACHE_DIR)
    slots = placement.derive_slots()
    free = [s for s in slots if not world.object_in_collision(pieces, _release_pose(s))]
    assert free
    T = _release_pose(free[0])
    world.add_object("occupant", pieces, T)
    assert world.object_in_collision(pieces, T)
    world.remove_object("occupant")
    assert not world.object_in_collision(pieces, T)
