# Copyright (c) 2026, dishsim project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""The arrangement-RRT planners through the toy oracle — Kit-free, seeded, fast."""

import numpy as np
import pytest

from dishsim import rrt
from dishsim.rearrange import run_episode

from test_rearrange import T, ToyOracle, ToyWorld, swap_instance


@pytest.mark.parametrize("cls", [rrt.RRT, rrt.RRTConnect, rrt.RRTStar])
def test_planner_solves_the_swap(cls):
    inst = swap_instance()
    rec = run_episode(inst, cls(seed=0), ToyWorld(), ToyOracle(inst), budget=10,
                      algorithm_name=cls.name, time_budget_s=20.0)
    assert rec["solved"] and rec["abort"] is None, rec["abort"]
    assert rec["moves_used"] >= 3          # the swap provably needs a buffer trip
    assert rec["algo_stats"]["nodes"] > 0
    assert rec["infeasible_commands"] == 0  # the planner pre-checks with the same oracle


@pytest.mark.parametrize("cls", [rrt.RRT, rrt.RRTConnect, rrt.RRTStar])
def test_planner_gives_up_cleanly_at_cap_zero(cls, monkeypatch):
    """With the buffer barred and no spare cell, the toy swap is infeasible: the planner
    must give up (bounded by iterations here, not wall clock)."""
    monkeypatch.setattr(rrt, "MAX_ITERS", 2000)
    inst = swap_instance()
    rec = run_episode(inst, cls(seed=0), ToyWorld(), ToyOracle(inst), budget=10,
                      counter_cap=0, time_budget_s=20.0)
    assert not rec["solved"] and rec["abort"] == "give-up"
    assert rec["moves_used"] == 0 and rec["counter_full_refusals"] == 0


def test_planner_replans_after_a_failed_settle():
    inst = swap_instance()
    algo = rrt.RRTConnect(seed=0)
    rec = run_episode(inst, algo, ToyWorld(), ToyOracle(inst, fail_settle_at=2), budget=10,
                      time_budget_s=20.0)
    assert rec["solved"] and rec["failed_settles"] == 1
    assert rec["algo_stats"]["replans"] == 1


def test_seed_reproducibility():
    inst = swap_instance()
    recs = []
    for _ in range(2):
        recs.append(run_episode(inst, rrt.RRT(seed=7), ToyWorld(), ToyOracle(inst),
                                budget=10, time_budget_s=20.0))
    assert recs[0]["moves_used"] == recs[1]["moves_used"]
    assert [m["item_id"] for m in recs[0]["moves"]] == \
           [m["item_id"] for m in recs[1]["moves"]]


def test_rrt_star_is_no_worse_than_rrt_on_the_swap():
    inst = swap_instance()
    star = run_episode(inst, rrt.RRTStar(seed=3), ToyWorld(), ToyOracle(inst), budget=10,
                       time_budget_s=20.0)
    base = run_episode(inst, rrt.RRT(seed=3), ToyWorld(), ToyOracle(inst), budget=10,
                       time_budget_s=20.0)
    assert star["solved"] and base["solved"]
    assert star["moves_used"] <= base["moves_used"]


def test_mirror_is_restored_after_planning():
    """The planner must hand the FCL mirror back at the observed arrangement — the driver's
    own pre-check depends on it."""
    inst = swap_instance()
    world = ToyWorld()
    rec = run_episode(inst, rrt.RRT(seed=0), world, ToyOracle(inst), budget=10,
                      time_budget_s=20.0)
    assert rec["solved"]
    # after the episode the mirror holds the final (solved) arrangement
    assert abs(world.poses["A"][0, 3] - 1.0) < 1e-6
    assert abs(world.poses["B"][0, 3] - 0.0) < 1e-6
