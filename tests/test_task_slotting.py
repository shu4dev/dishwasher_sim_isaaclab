# Copyright (c) 2026, dishsim project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""The live packing rules: candidate ordering + mode-aware occupancy (dishsim/task/slotting.py).

Both functions are called on every capacity placement (`capacity._try_place_one`); the
plate-gap rule below is the one whose silent regression would plan one plate per rack.
"""

import numpy as np

from dishsim.placement import SlotFrame
from dishsim.task import slotting


def _slot(sid, xyz, mode="floor_stand"):
    T = np.eye(4)
    T[:3, 3] = xyz
    return SlotFrame(sid, T, 0.06, "derived", mode=mode)


def _centre(xyz):
    return np.asarray(xyz, dtype=float)


class TestOccupancyConflict:
    def test_adjacent_plate_gaps_never_conflict(self):
        # a tine bank exists to hold parallel discs one gap apart — the rack pitch, not the
        # disc radius, is the separation (the documented v2 change)
        assert not slotting.occupancy_conflict(
            "plate_slot", _centre((0.30, 0.10, 0.0)), 0.070,
            "plate_slot", _centre((0.30, 0.14, 0.0)), 0.070, 0.010)

    def test_the_same_plate_gap_is_one_receptacle(self):
        c = _centre((0.30, 0.10, 0.0))
        assert slotting.occupancy_conflict("plate_slot", c, 0.070,
                                           "plate_slot", c.copy(), 0.070, 0.010)

    def test_plate_vs_standing_neighbour_keeps_the_circle_rule(self):
        # 40 mm apart with radii 70 + 40 mm: circles overlap -> conflict
        assert slotting.occupancy_conflict(
            "plate_slot", _centre((0.30, 0.10, 0.0)), 0.070,
            "floor_stand", _centre((0.30, 0.14, 0.0)), 0.040, 0.010)

    def test_standing_circles_clear_when_separated(self):
        assert not slotting.occupancy_conflict(
            "floor_stand", _centre((0.30, 0.00, 0.0)), 0.040,
            "floor_stand", _centre((0.30, 0.12, 0.0)), 0.040, 0.010)


class TestCandidateSlotIds:
    SLOTS = {0: _slot(0, (0.3, 0.0, 0.0)), 1: _slot(1, (0.4, 0.0, 0.0)),
             2: _slot(2, (0.5, 0.0, 0.0))}
    NAMES = {"near": 0, "mid": 1, "far": 2}

    def test_feasibility_map_filters_and_pool_orders(self):
        # slot 1 infeasible (empty entry — the capacity planner's placeable=False shim)
        feasible = {0: (1,), 1: (), 2: (1,)}
        ids = slotting.candidate_slot_ids(
            "cup", "floor_stand", self.SLOTS, self.NAMES, feasible,
            type_slots=None, slot_pools={"floor_stand": (2, 1, 0)})
        assert ids == [2, 0]  # pool order kept, infeasible dropped

    def test_type_slots_is_an_ordered_preference(self):
        feasible = {0: (1,), 1: (1,), 2: (1,)}
        ids = slotting.candidate_slot_ids(
            "cup", "floor_stand", self.SLOTS, self.NAMES, feasible,
            type_slots={"cup": ("far", "near")}, slot_pools={"floor_stand": None})
        assert ids == [2, 0]

    def test_unknown_type_slot_name_fails_loudly(self):
        import pytest

        with pytest.raises(SystemExit, match="bogus"):
            slotting.candidate_slot_ids(
                "cup", "floor_stand", self.SLOTS, self.NAMES, {0: (1,)},
                type_slots={"cup": ("bogus",)}, slot_pools={"floor_stand": None})
