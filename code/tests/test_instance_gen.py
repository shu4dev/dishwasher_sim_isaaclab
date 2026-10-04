# Copyright (c) 2026, dishsim project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""The instance sampler's tier knobs (cycles, goal-squat, counter cap) — Kit-free, stubbed.

``_nominal_release_pose`` is monkeypatched to identity-on-the-slot so the stub tables can
store 4x4 poses directly; the real geometry path is exercised by the Kit generator.
"""

import types

import numpy as np
import pytest

from dishsim import capacity, instance_gen
from dishsim.transforms import make_T


def T(x):
    return make_T((float(x), 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))


class ToyWorld:
    """1-D: a commanded pose collides when any OTHER item sits within 0.5 of its x."""

    def __init__(self):
        self.poses = {}

    def clear(self):
        self.poses = {}

    def sync(self, poses, classes):
        self.poses.update({k: np.asarray(v) for k, v in poses.items()})

    def move_collides(self, item_id, T_cmd, object_class=None):
        x = np.asarray(T_cmd)[0, 3]
        return any(k != item_id and abs(p[0, 3] - x) < 0.5 for k, p in self.poses.items())

    def buffer_poses(self, object_class):
        return [T(100.0), T(101.0), T(102.0)]


def _roster(n, cls="bowl"):
    # slot i sits at x=2i; every item's target is its own slot
    return [types.SimpleNamespace(item_id=f"{cls}_{i:02d}", object_class=cls, slot_id=i,
                                  T_base_obj=T(2 * i)) for i in range(n)]


def _tables(roster):
    slots = {}
    placeable = {}
    for it in roster:
        slots.setdefault(it.object_class, {})[it.slot_id] = T(2 * it.slot_id)
        placeable.setdefault(it.object_class, {})[it.slot_id] = True
    return types.SimpleNamespace(slots=slots, placeable=placeable)


@pytest.fixture(autouse=True)
def _slot_pose_is_slot(monkeypatch):
    monkeypatch.setattr(capacity, "_nominal_release_pose", lambda slot: slot)


def test_author_cycles_builds_disjoint_same_class_derangements():
    roster = _roster(6)
    rng = np.random.default_rng(0)
    forced, cycles = instance_gen.author_cycles(roster, (2, 3), rng)
    assert forced is not None and len(cycles) == 2
    assert sorted(len(c) for c in cycles) == [2, 3]
    members = [i for c in cycles for i in c]
    assert len(members) == len(set(members)) == 5  # disjoint
    by_id = {it.item_id: it for it in roster}
    for cyc in cycles:
        for k, item_id in enumerate(cyc):
            nxt = by_id[cyc[(k + 1) % len(cyc)]]
            assert forced[item_id] == nxt.slot_id          # k starts on k+1's goal slot
            assert forced[item_id] != by_id[item_id].slot_id  # true derangement
    # determinism
    forced2, cycles2 = instance_gen.author_cycles(roster, (2, 3), np.random.default_rng(0))
    assert (forced2, cycles2) == (forced, cycles)


def test_author_cycles_refuses_undersized_pools():
    assert instance_gen.author_cycles(_roster(2), (3,), np.random.default_rng(0)) == (None, None)


def test_max_counter_zero_never_parks_on_the_buffer():
    roster = _roster(4)
    rng = np.random.default_rng(1)
    initials, displaced = instance_gen.sample_initials(
        roster, _tables(roster), ToyWorld(), rng, 2, max_counter=0)
    assert initials is not None
    for item_id in displaced:
        assert initials[item_id][0, 3] < 90.0  # in-rack slots only, never the band


def test_avoid_goal_slots_keeps_displaced_off_every_goal():
    roster = _roster(3)
    goal_xs = {2 * it.slot_id for it in roster}
    for seed in range(5):
        initials, displaced = instance_gen.sample_initials(
            roster, _tables(roster), ToyWorld(), np.random.default_rng(seed), 2,
            avoid_goal_slots=True)
        assert initials is not None
        for item_id in displaced:
            assert float(initials[item_id][0, 3]) not in goal_xs  # buffer band only here


def test_forced_slots_land_exactly_where_authored():
    roster = _roster(4)
    forced = {"bowl_00": 1, "bowl_01": 0}  # a 2-cycle
    initials, displaced = instance_gen.sample_initials(
        roster, _tables(roster), ToyWorld(), np.random.default_rng(2), 2,
        forced_slots=forced)
    assert initials is not None and {"bowl_00", "bowl_01"} <= displaced
    assert float(initials["bowl_00"][0, 3]) == 2.0  # slot 1
    assert float(initials["bowl_01"][0, 3]) == 0.0  # slot 0
