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


class OrderedWorld(ToyWorld):
    """ToyWorld plus one goal-order constraint: B's goal (x = 0) waits until A is at its goal (x = 1)."""

    def goal_waits(self, item_id, T_cmd):
        if item_id == "B" and abs(np.asarray(T_cmd)[0, 3]) < 1e-6:
            a = self.poses.get("A")
            return [] if a is not None and abs(a[0, 3] - 1.0) < 1e-6 else ["A"]
        return []

    def blockers(self, item_id, T_cmd, object_class=None):
        return sorted(set(super().blockers(item_id, T_cmd)) | set(self.goal_waits(item_id, T_cmd)))


def ordered_instance():
    items = [{"item_id": "A", "object_class": "toy", "T_base_init": T(5), "target": {"T_base_obj": T(1)}},
             {"item_id": "B", "object_class": "toy", "T_base_init": T(6), "target": {"T_base_obj": T(0)}}]
    from dishsim.rearrange import Instance
    return Instance(name="ordered", machine="toy", base_placement="toy", state="toy", items=items, meta={})


def test_rrt_connect_honours_the_goal_order_on_the_goal_tree():
    """The goal tree's edges replay backwards, so a constraint 'B's goal waits for A' must be checked on the
    replayed move: every found plan sends A home before B (2026-09-28; before, the goal tree was checked
    forwards and could emit B first)."""
    for seed in range(6):
        inst = ordered_instance()
        planner = rrt.RRTConnect(seed=seed)
        planner.reset(inst, OrderedWorld())
        start = {it["item_id"]: np.asarray(it["T_base_init"]) for it in inst.items}
        path = planner._grow(start, None, __import__("time").perf_counter() + 5.0)
        assert path, f"seed {seed}: no plan"
        a_home = [k for k, (item, T_) in enumerate(path) if item == "A" and abs(np.asarray(T_)[0, 3] - 1.0) < 1e-6]
        b_home = [k for k, (item, T_) in enumerate(path) if item == "B" and abs(np.asarray(T_)[0, 3]) < 1e-6]
        assert a_home and b_home and max(a_home) < b_home[-1], f"seed {seed}: {[(i, float(np.asarray(t)[0, 3])) for i, t in path]}"


def test_nearest_neighbour_matrix_matches_the_item_count_distance():
    inst = swap_instance()
    planner = rrt.RRTConnect(seed=1)
    planner.reset(inst, ToyWorld())
    start = {it["item_id"]: np.asarray(it["T_base_init"]) for it in inst.items}
    tree = [rrt._Node(dict(start))]
    keys = {planner._key(start)}
    for _ in range(30):
        planner._extend(tree, keys, planner._sample(), planner.goal, None)
    sample = planner._sample()
    d = (planner._matrix(tree) != planner._codes_of(sample)[None, :]).sum(axis=1)
    assert [int(x) for x in d] == [len(planner._diff(n.poses, sample)) for n in tree]
    assert len(tree) > 1 and planner._matrix(tree).shape == (len(tree), 2)


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
