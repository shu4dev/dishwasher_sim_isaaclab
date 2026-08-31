# Copyright (c) 2026, dishsim project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""The optimal solver's search, on synthetic tables — Kit-free and geometry-free.

The A* in :mod:`dishsim.compat` is the benchmark's ground truth, so it is worth testing apart
from the collision geometry that feeds it: a stub table makes the expected move counts
arithmetic rather than a matter of measurement. The geometry side (does the table agree with
live FCL?) is checked separately against a real cache, which needs the asset archive.
"""

import os
import types

import numpy as np
import pytest

from dishsim import compat

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_INSTANCES = os.path.join(PROJECT_ROOT, "results", "instances", "bosch800", "placement")
#: Ground truth for the LOCAL instances, cross-validated two ways: the solver reproduces the
#: harness's own ``at_goal_initial`` exactly, and greedy's recorded solutions bound the optima
#: from above (corallab 2026-08-30 run: greedy solved s0/s1/s2 in 9/9/9 moves — optimal on
#: all three). Instances are per-machine artifacts (PhysX settle fixed points differ across
#: engines), so these pins belong to the instances generated on THIS box. The retired
#: Brev-era instances (never archived) measured at_goal 7/5/6 with optima {9, 10, 9}.
#: Re-pinned 2026-08-31: the earlier s1/s2 pins (8/8) were computed against s0's initial
#: poses through the table-reuse bug that CompatTable.set_instance now fixes.
_KNOWN_OPTIMA = {"perturbed_s0": 9, "perturbed_s1": 9, "perturbed_s2": 9}


def _T(x: float) -> np.ndarray:
    T = np.eye(4)
    T[0, 3] = x
    return T


class StubTable:
    """Cells on a line, one item per cell; distinct positions never conflict."""

    def __init__(self, items, inits, n_cells=3, n_buffers=1):
        self.poses = {("b", i): _T(float(i)) for i in range(n_cells)}
        for j in range(n_buffers):
            self.poses[("b", ("buf", j))] = _T(90.0 + j)
        for item, x in zip(items, inits):
            self.poses[("b", ("init", item))] = _T(x)
        self.static_ok = {k: True for k in self.poses}
        # counter-band membership: buffer cells by convention; tests mutate to add inits
        self.counter_locs = {k for k in self.poses
                             if isinstance(k[1], tuple) and k[1][0] == "buf"}

    def compatible(self, a, b):
        if a == b:
            return False
        return abs(float(self.poses[a][0, 3]) - float(self.poses[b][0, 3])) > 1e-9


def _instance(items, inits, targets):
    return types.SimpleNamespace(
        state="placement",
        items=[{"item_id": i, "object_class": "b", "T_base_init": _T(x),
                "target": {"T_base_obj": _T(t)}}
               for i, x, t in zip(items, inits, targets)])


def test_items_already_at_goal_cost_nothing():
    """An item starting ON its target cell must not be charged a move to stay there."""
    table = StubTable(["A", "B"], [0.0, 1.0])
    n, _, _ = compat.optimal_moves(_instance(["A", "B"], [0.0, 1.0], [0.0, 1.0]), table=table)
    assert n == 0


def test_each_displaced_item_costs_one_move_when_unobstructed():
    table = StubTable(["A", "B"], [10.0, 11.0])
    n, _, _ = compat.optimal_moves(_instance(["A", "B"], [10.0, 11.0], [0.0, 1.0]), table=table)
    assert n == 2


def test_two_cycle_forces_a_buffer_trip():
    """A on B's target and B on A's: the swap cannot be done in two moves."""
    table = StubTable(["A", "B"], [0.0, 1.0])
    n, _, _ = compat.optimal_moves(_instance(["A", "B"], [0.0, 1.0], [1.0, 0.0]), table=table)
    assert n == 3


def test_three_cycle_costs_four_moves():
    table = StubTable(["A", "B", "C"], [0.0, 1.0, 2.0])
    n, _, _ = compat.optimal_moves(
        _instance(["A", "B", "C"], [0.0, 1.0, 2.0], [1.0, 2.0, 0.0]), table=table)
    assert n == 4


def test_unsolvable_without_a_buffer_returns_none():
    """With every cell occupied and nowhere to park, a swap has no solution."""
    table = StubTable(["A", "B"], [0.0, 1.0], n_cells=2, n_buffers=0)
    n, status, _ = compat.optimal_moves(_instance(["A", "B"], [0.0, 1.0], [1.0, 0.0]),
                                        table=table)
    assert n is None and status == "unsolvable"


def test_status_reporting():
    """solved / bound / unsolvable are distinguishable — a capped benchmark depends on it."""
    swap = _instance(["A", "B"], [0.0, 1.0], [1.0, 0.0])
    n, status, _ = compat.optimal_moves(swap, table=StubTable(["A", "B"], [0.0, 1.0]))
    assert (n, status) == (3, "solved")
    n, status, _ = compat.optimal_moves(swap, table=StubTable(["A", "B"], [0.0, 1.0]),
                                        max_expansions=0)
    assert (n, status) == (None, "bound")


def test_counter_cap_zero_makes_buffered_swap_unsolvable():
    """With no spare rack cell, the swap needs the buffer: infeasible at cap 0, 3 at cap 1."""
    swap = _instance(["A", "B"], [0.0, 1.0], [1.0, 0.0])
    n, status, _ = compat.optimal_moves(
        swap, table=StubTable(["A", "B"], [0.0, 1.0], n_cells=2), counter_cap=0)
    assert (n, status) == (None, "unsolvable")
    n, status, _ = compat.optimal_moves(
        swap, table=StubTable(["A", "B"], [0.0, 1.0], n_cells=2), counter_cap=1)
    assert (n, status) == (3, "solved")


def test_leaving_the_band_is_legal_at_cap_zero():
    """The cap gates only moves INTO the band: an item starting there may always leave."""
    table = StubTable(["A"], [95.0])  # init pose in the band region
    table.counter_locs.add(("b", ("init", "A")))
    n, status, _ = compat.optimal_moves(_instance(["A"], [95.0], [0.0]), table=table,
                                        counter_cap=0)
    assert (n, status) == (1, "solved")


def test_items_starting_on_the_band_count_against_the_cap():
    """An item STARTING on the band consumes cap capacity: with A parked on the band and
    B's buffer trip barred while any other item shares the band, cap 2 solves directly in 3
    (B->buf, A->c0, B->c1); cap 1 forces a 4-move squat-shuffle. If init-band members were
    NOT counted against the cap, cap 1 would also give 3 — that count is what this pins."""

    class Conflicted(StubTable):
        def __init__(self):
            super().__init__(["A", "B"], [95.0, 10.0], n_cells=2, n_buffers=1)
            self.counter_locs.add(("b", ("init", "A")))  # A starts ON the band
            # ordering locks: A's goal c0 blocked while B sits on its start; B's goal c1
            # blocked while A sits on its band start; the buffer blocked while B sits on
            # its start — which closes A's band->band escape but NOT B's own buffer trip
            # (a pair never binds against the mover's own start cell)
            self.extra = {(("b", 0), ("b", ("init", "B"))),
                          (("b", 1), ("b", ("init", "A"))),
                          (("b", ("buf", 0)), ("b", ("init", "B")))}

        def compatible(self, a, b):
            if (a, b) in self.extra or (b, a) in self.extra:
                return False
            return super().compatible(a, b)

    inst = _instance(["A", "B"], [95.0, 10.0], [0.0, 1.0])
    n, status, _ = compat.optimal_moves(inst, table=Conflicted(), counter_cap=2)
    assert (n, status) == (3, "solved")   # B->buf, A->c0, B->c1
    n, status, _ = compat.optimal_moves(inst, table=Conflicted(), counter_cap=1)
    assert (n, status) == (4, "solved")   # buffer barred while A on band: squat-shuffle


@pytest.mark.skipif(not os.path.isdir(_INSTANCES),
                    reason="needs the restored asset archive + generated instances")
def test_known_optima_on_shipped_instances():
    """The solver's ground truth on the real instances, and its agreement with the harness.

    Two independent checks, because a wrong optimum silently corrupts every gap this
    benchmark reports: the start state must contain exactly the items the harness itself
    counts as at-goal, and the optima must match the values greedy's recorded runs bound.
    """
    from dishsim import config

    config.apply_machine("bosch800")
    config.apply_scenario("placement")
    config.apply_base_placement("side_winner")
    from dishsim import rearrange

    recorded_at_goal = {"perturbed_s0": 7, "perturbed_s1": 7, "perturbed_s2": 7}
    table = None
    for name, expected in _KNOWN_OPTIMA.items():
        inst = rearrange.Instance.load(os.path.join(_INSTANCES, f"{name}.json"))
        n_home = sum(1 for it in inst.items
                     if rearrange.at_goal(it, np.asarray(it["T_base_init"], dtype=float)))
        assert n_home == recorded_at_goal[name], f"{name}: start state disagrees with harness"
        n, status, table = compat.optimal_moves(inst, table=table)
        assert status == "solved"
        assert n == expected, f"{name}: optimum {n}, expected {expected}"
        # regression for the table-reuse bug: a FRESH table must agree with the reused one
        # (before set_instance existed, s1/s2 were silently solved against s0's initials)
        if name == "perturbed_s1":
            n_fresh, _, _ = compat.optimal_moves(inst, table=None)
            assert n_fresh == n, f"{name}: reused-table optimum {n} != fresh {n_fresh}"
