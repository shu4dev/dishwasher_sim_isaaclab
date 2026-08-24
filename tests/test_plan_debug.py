# Copyright (c) 2026, dishsim project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""PlanDebug instrumentation of Planner.plan: checker capture and the PlannerData tree.

Kit-free and asset-free: the collision world is stubbed, so only ompl + numpy are exercised.
Run with the project venv:

    PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /workspace/isaaclab/env_isaaclab/bin/python -m pytest \
        tests/test_plan_debug.py -v
"""

import numpy as np

from dishsim import config
from dishsim.planners import PlanDebug, make_planner
from dishsim.ur5e_kin import JOINT_LIMITS

from conftest import FreeWorld, WallWorld

START = np.array(config.HOME_Q)
FREE_GOALS = np.array([START + [0.4, 0.2, -0.2, 0.3, -0.3, 0.5], START + [-0.3, 0.1, 0.2, -0.2, 0.2, -0.4]])


def test_debug_capture_free_world():
    dbg = PlanDebug()
    res = make_planner("rrt_connect", budget_s=2.0).plan(FreeWorld(), START, FREE_GOALS, seed=1, debug=dbg)
    assert res.status == "solved"
    assert dbg.n_checks > 0
    assert dbg.checked_q.shape == (dbg.n_checks, 6)
    assert dbg.checked_valid.shape == (dbg.n_checks,)
    assert dbg.checked_valid.all(), "free world must never record an invalid verdict"
    assert dbg.raw_path_q is not None and np.allclose(dbg.raw_path_q[0], START)
    raw_len = float(np.sum(np.linalg.norm(np.diff(dbg.raw_path_q, axis=0), axis=1)))
    assert res.path_len_rad <= raw_len + 1e-6, "simplification must not lengthen the path"


def test_planner_data_tree():
    dbg = PlanDebug()
    res = make_planner("rrt_connect", budget_s=2.0).plan(FreeWorld(), START, FREE_GOALS, seed=1, debug=dbg)
    assert res.status == "solved"
    assert dbg.planner_data_ok, dbg.planner_data_error
    n = len(dbg.tree_q)
    assert n >= 2 and dbg.tree_q.shape == (n, 6)
    assert dbg.tree_tag.shape == (n,)
    tags = set(dbg.tree_tag.tolist())
    assert tags <= {1, 2}, f"unexpected RRT-Connect tags {tags}"
    assert tags == {1, 2}, "a solved run must have vertices in both trees"
    assert dbg.tree_edges.ndim == 2 and dbg.tree_edges.shape[1] == 2
    assert (dbg.tree_edges >= 0).all() and (dbg.tree_edges < n).all()
    lo = JOINT_LIMITS[:, 0] + config.PLAN_JOINT_BOUNDS_MARGIN_RAD - 1e-9
    hi = JOINT_LIMITS[:, 1] - config.PLAN_JOINT_BOUNDS_MARGIN_RAD + 1e-9
    assert (dbg.tree_q >= lo).all() and (dbg.tree_q <= hi).all(), "tree vertices outside planning bounds"
    start_dists = np.linalg.norm(dbg.tree_q[dbg.tree_tag == 1] - START, axis=1)
    assert start_dists.min() < 1e-6, "start tree must contain the start state"


def test_invalid_states_captured():
    goal = START.copy()
    goal[1] = -0.3  # across the slab from HOME's q1 = -1.712
    dbg = PlanDebug()
    res = make_planner("rrt_connect", budget_s=5.0).plan(WallWorld(), START, goal[None, :], seed=1, debug=dbg)
    assert res.status == "solved"
    invalid = dbg.checked_q[~dbg.checked_valid]
    assert len(invalid) > 0, "the slab must reject at least one sampled state"
    world = WallWorld()
    assert all(world.in_collision(q) for q in invalid), "recorded verdicts disagree with the world"


def test_no_debug_is_noop():
    res = make_planner("rrt_connect", budget_s=2.0).plan(FreeWorld(), START, FREE_GOALS, seed=1)
    assert res.status == "solved" and res.path_q is not None
