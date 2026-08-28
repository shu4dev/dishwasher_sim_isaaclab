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
#: Ground truth for the shipped instances, cross-validated two ways: the solver reproduces the
#: harness's own ``at_goal_initial`` (7/5/6) exactly, and greedy's recorded 9-move solutions of
#: s0 and s2 prove those optima cannot exceed 9.
_KNOWN_OPTIMA = {"perturbed_s0": 9, "perturbed_s1": 10, "perturbed_s2": 9}


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
    n, _ = compat.optimal_moves(_instance(["A", "B"], [0.0, 1.0], [0.0, 1.0]), table=table)
    assert n == 0


def test_each_displaced_item_costs_one_move_when_unobstructed():
    table = StubTable(["A", "B"], [10.0, 11.0])
    n, _ = compat.optimal_moves(_instance(["A", "B"], [10.0, 11.0], [0.0, 1.0]), table=table)
    assert n == 2


def test_two_cycle_forces_a_buffer_trip():
    """A on B's target and B on A's: the swap cannot be done in two moves."""
    table = StubTable(["A", "B"], [0.0, 1.0])
    n, _ = compat.optimal_moves(_instance(["A", "B"], [0.0, 1.0], [1.0, 0.0]), table=table)
    assert n == 3


def test_three_cycle_costs_four_moves():
    table = StubTable(["A", "B", "C"], [0.0, 1.0, 2.0])
    n, _ = compat.optimal_moves(
        _instance(["A", "B", "C"], [0.0, 1.0, 2.0], [1.0, 2.0, 0.0]), table=table)
    assert n == 4


def test_unsolvable_without_a_buffer_returns_none():
    """With every cell occupied and nowhere to park, a swap has no solution."""
    table = StubTable(["A", "B"], [0.0, 1.0], n_cells=2, n_buffers=0)
    n, _ = compat.optimal_moves(_instance(["A", "B"], [0.0, 1.0], [1.0, 0.0]), table=table)
    assert n is None


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

    recorded_at_goal = {"perturbed_s0": 7, "perturbed_s1": 5, "perturbed_s2": 6}
    table = None
    for name, expected in _KNOWN_OPTIMA.items():
        inst = rearrange.Instance.load(os.path.join(_INSTANCES, f"{name}.json"))
        n_home = sum(1 for it in inst.items
                     if rearrange.at_goal(it, np.asarray(it["T_base_init"], dtype=float)))
        assert n_home == recorded_at_goal[name], f"{name}: start state disagrees with harness"
        n, table = compat.optimal_moves(inst, table=table)
        assert n == expected, f"{name}: optimum {n}, expected {expected}"
