# Copyright (c) 2026, dishsim project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""The planner registry contract — every registered algorithm must be interchangeable.

These run against stub worlds (the only thing a planner needs is ``in_collision(q) -> bool``),
so they are Kit-free and fast. A new planner added to the registry is covered automatically.
"""

import numpy as np
import pytest

from dishsim import config
from dishsim.planners import PLANNERS, PlanDebug, PlanResult, Planner, available, make_planner
from dishsim.ur5e_kin import JOINT_LIMITS

from conftest import FreeWorld, WallWorld

START = np.array([0.0, -1.2, 1.2, -1.5, -1.5, 0.0])
GOALS = np.array([[0.6, -1.0, 1.0, -1.4, -1.5, 0.2], [-0.6, -1.4, 1.4, -1.6, -1.5, -0.2]])


def _budget(name: str) -> float:
    """Keep the optimizing planners' test cost bounded (they always use the whole budget)."""
    return 2.0


@pytest.mark.parametrize("name", available())
def test_registered_planner_solves_free_world(name):
    planner = make_planner(name, budget_s=_budget(name))
    res = planner.plan(FreeWorld(), START, GOALS, seed=1)
    assert isinstance(res, PlanResult)
    assert res.status == "solved", f"{name} failed on an obstacle-free query"
    assert res.path_q is not None and res.path_q.shape[1] == 6
    assert np.allclose(res.path_q[0], START), "path must start at the start configuration"
    assert res.path_len_rad > 0 and res.plan_time_s >= 0
    assert 0 <= res.goal_index < len(GOALS)
    assert np.linalg.norm(res.path_q[-1] - GOALS[res.goal_index]) < 1e-6


@pytest.mark.parametrize("name", available())
def test_registered_planner_respects_joint_bounds(name):
    res = make_planner(name, budget_s=_budget(name)).plan(FreeWorld(), START, GOALS, seed=2)
    lo = JOINT_LIMITS[:, 0] + config.PLAN_JOINT_BOUNDS_MARGIN_RAD - 1e-9
    hi = JOINT_LIMITS[:, 1] - config.PLAN_JOINT_BOUNDS_MARGIN_RAD + 1e-9
    assert (res.path_q >= lo).all() and (res.path_q <= hi).all()


@pytest.mark.parametrize("name", available())
def test_registered_planner_never_returns_a_colliding_path(name):
    """The safety invariant: whatever a planner returns must be valid in the world it planned
    against. Whether it finds a solution at all inside a small budget is algorithm-dependent
    (the optimizing planners are slower to a first solution), so that is asserted separately
    for the default planner below."""
    world = WallWorld()
    res = make_planner(name, budget_s=_budget(name)).plan(world, START, GOALS[:1], seed=3)
    assert res.status in ("solved", "timeout")
    if res.status == "solved":
        assert not any(world.in_collision(q) for q in res.path_q)


def test_default_planner_routes_around_an_obstacle():
    """The planner the runner actually uses must solve a query needing a detour."""
    world = WallWorld()
    res = make_planner(config.PLANNER, **config.PLANNER_PARAMS.get(config.PLANNER, {})).plan(
        world, START, GOALS[:1], seed=3
    )
    assert res.status == "solved", f"{config.PLANNER} could not route around the slab"
    assert not any(world.in_collision(q) for q in res.path_q)


@pytest.mark.parametrize("name", available())
def test_registered_planner_reports_no_goals(name):
    res = make_planner(name, budget_s=_budget(name)).plan(FreeWorld(), START, np.zeros((0, 6)))
    assert res.status == "no_goals" and res.path_q is None


@pytest.mark.parametrize("name", available())
def test_registered_planner_describes_itself(name):
    """The provenance block recorded with every trial must name the algorithm and its params."""
    desc = make_planner(name, budget_s=_budget(name)).describe()
    assert desc["name"] == name
    assert desc["budget_s"] == _budget(name)
    assert "resolution_rad" in desc and "simplify" in desc


@pytest.mark.parametrize("name", available())
def test_registered_planner_fills_debug(name):
    """PlanDebug is filled in place by any planner that supports instrumentation."""
    dbg = PlanDebug()
    make_planner(name, budget_s=_budget(name)).plan(FreeWorld(), START, GOALS, seed=4, debug=dbg)
    assert dbg.n_checks > 0
    assert dbg.checked_q.shape == (dbg.n_checks, 6)
    assert dbg.checked_valid.shape == (dbg.n_checks,)


def test_registry_is_populated_and_typed():
    assert set(available()) >= {"rrt_connect", "rrt_star", "prm"}
    for name, cls in PLANNERS.items():
        assert issubclass(cls, Planner), name
        assert cls.name == name, f"{cls.__name__}.name must match its registry key"


def test_unknown_planner_raises():
    with pytest.raises(ValueError, match="unknown planner"):
        make_planner("dijkstra_on_vibes")


def test_config_default_planner_is_registered():
    """The runner's default must exist, and every params entry must name a real planner."""
    assert config.PLANNER in PLANNERS
    for name in config.PLANNER_PARAMS:
        assert name in PLANNERS, f"config.PLANNER_PARAMS names unknown planner {name!r}"


def test_planner_params_are_accepted():
    """Documented per-algorithm params must construct without error."""
    for name, params in config.PLANNER_PARAMS.items():
        planner = make_planner(name, **params)
        res = planner.plan(FreeWorld(), START, GOALS, seed=5)
        assert res.status == "solved", f"{name} failed with its configured params"
