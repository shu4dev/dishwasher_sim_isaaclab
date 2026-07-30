# Copyright (c) 2026, dishsim project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""OMPL planning over the FCL collision world (Kit-free).

RRT-Connect in a 6-D ``RealVectorStateSpace`` bounded by the UR5e joint limits (minus a
margin), validity = ``not CollisionWorld.in_collision(q)`` with the carried object attached
and self-checking on, goals = ``ob.GoalStates`` built from a Phase E goal set. OMPL >= 2.0
nanobind API (plain-callable validity checkers, ``space.allocState()``).
"""

import time
from dataclasses import dataclass

import numpy as np

from . import config
from .ur5e_kin import JOINT_LIMITS


@dataclass
class PlanResult:
    """Outcome of one planning query."""

    path_q: np.ndarray | None  # [N, 6] waypoints, or None
    plan_time_s: float
    path_len_rad: float  # sum of L2 segment lengths (after simplification)
    status: str  # "solved" | "timeout" | "no_goals"
    goal_index: int = -1  # index (into the passed goal array) the path terminated at


def plan_to_goals(
    world,
    start_q: np.ndarray,
    goal_qs: np.ndarray,
    budget_s: float = config.PLAN_TIME_BUDGET_S,
    resolution_rad: float = config.PLAN_VALIDITY_RESOLUTION_RAD,
    simplify: bool = True,
    seed: int | None = None,
) -> PlanResult:
    """Plan a collision-free joint path from ``start_q`` to any of ``goal_qs``.

    Args:
        world: :class:`dishsim.collision_world.CollisionWorld` (validity oracle).
        start_q: Start configuration, shape [6].
        goal_qs: Goal set, shape [K, 6] (K >= 1).
        budget_s: Planner time budget [s].
        resolution_rad: Motion-validation step [rad] (converted to OMPL's fraction-of-extent).
        simplify: Run OMPL path simplification on the solution.
        seed: OMPL RNG seed (global; set before planner construction for reproducibility).

    Returns:
        A :class:`PlanResult`; ``path_q`` includes the start as row 0.
    """
    from ompl import base as ob  # noqa: PLC0415
    from ompl import geometric as og  # noqa: PLC0415
    from ompl import util as ou  # noqa: PLC0415

    goal_qs = np.atleast_2d(np.asarray(goal_qs, dtype=float))
    if len(goal_qs) == 0:
        return PlanResult(None, 0.0, 0.0, "no_goals")

    ou.setLogLevel(ou.LOG_WARN)
    if seed is not None:
        try:
            ou.RNG.setSeed(int(seed) if int(seed) != 0 else 1)  # OMPL treats 0 as "randomize"
        except Exception:
            pass  # older bindings: seeding unavailable; trials still vary via goal subsets

    space = ob.RealVectorStateSpace(6)
    bounds = ob.RealVectorBounds(6)
    margin = config.PLAN_JOINT_BOUNDS_MARGIN_RAD
    for i in range(6):
        bounds.setLow(i, float(JOINT_LIMITS[i, 0] + margin))
        bounds.setHigh(i, float(JOINT_LIMITS[i, 1] - margin))
    space.setBounds(bounds)

    ss = og.SimpleSetup(space)

    def is_valid(state) -> bool:
        q = np.array([state[i] for i in range(6)])
        return not world.in_collision(q)

    ss.setStateValidityChecker(is_valid)
    si = ss.getSpaceInformation()
    si.setStateValidityCheckingResolution(float(resolution_rad / space.getMaximumExtent()))

    start = space.allocState()
    for i in range(6):
        start[i] = float(start_q[i])
    ss.setStartState(start)

    goal = ob.GoalStates(si)
    for g in goal_qs:
        st = space.allocState()
        for i in range(6):
            st[i] = float(g[i])
        goal.addState(st)
    ss.setGoal(goal)
    planner = og.RRTConnect(si)
    planner.setRange(config.PLAN_RRT_RANGE_RAD)
    ss.setPlanner(planner)

    t0 = time.perf_counter()
    solved = ss.solve(float(budget_s))
    plan_time = time.perf_counter() - t0
    if not solved or not ss.haveExactSolutionPath():
        return PlanResult(None, plan_time, 0.0, "timeout")

    if simplify:
        ss.simplifySolution()
    path = ss.getSolutionPath()
    waypoints = np.array([[path.getState(i)[j] for j in range(6)] for i in range(path.getStateCount())])
    length = float(np.sum(np.linalg.norm(np.diff(waypoints, axis=0), axis=1)))
    goal_index = int(np.argmin(np.linalg.norm(goal_qs - waypoints[-1], axis=1)))
    return PlanResult(waypoints, plan_time, length, "solved", goal_index)


def time_parameterize(
    path_q: np.ndarray,
    speed_rad_s: float = config.EXEC_JOINT_SPEED_RAD_S,
    dt: float = config.SIM_DT,
) -> np.ndarray:
    """Resample a waypoint path at a constant joint-speed cap.

    Args:
        path_q: Waypoints, shape [N, 6].
        speed_rad_s: Max per-joint speed [rad/s].
        dt: Control period [s].

    Returns:
        Dense joint targets, shape [T, 6] (one row per physics step, ends exactly at the goal).
    """
    path_q = np.asarray(path_q, dtype=float)
    out = [path_q[0]]
    step = speed_rad_s * dt
    for a, b in zip(path_q[:-1], path_q[1:]):
        span = float(np.max(np.abs(b - a)))
        n = max(1, int(np.ceil(span / step)))
        for k in range(1, n + 1):
            out.append(a + (b - a) * (k / n))
    return np.array(out)
