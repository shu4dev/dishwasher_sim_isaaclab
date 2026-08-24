# Copyright (c) 2026, dishsim project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Shared OMPL machinery: everything an OMPL planner needs except the algorithm itself.

:class:`OMPLPlanner` owns the whole query — a 6-D ``RealVectorStateSpace`` bounded by the UR5e
joint limits (minus a margin), validity = ``not world.in_collision(q)``, goals as an
``ob.GoalStates`` set, solve, simplify, and :class:`~dishsim.planners.base.PlanDebug` capture.
Subclasses override exactly one method, :meth:`OMPLPlanner._make_planner`, because the OMPL
planners do not share a parameter interface (``RRTConnect`` has ``setRange`` only, ``RRTstar``
adds ``setGoalBias``, ``PRM`` has neither) — so each algorithm applies its own parameters
rather than a generic loop guessing at setters.

OMPL >= 2.0 nanobind API (plain-callable validity checkers, ``space.allocState()``).
"""

import time

import numpy as np

from .. import config
from ..ur5e_kin import JOINT_LIMITS
from .base import PlanDebug, PlanResult, Planner


def _extract_planner_data(planner, si, debug: PlanDebug) -> None:
    """Fill ``debug.tree_*`` from the planner's ``PlannerData`` (best effort, never raises)."""
    from ompl import base as ob  # noqa: PLC0415

    try:
        pd = ob.PlannerData(si)
        planner.getPlannerData(pd)
        n = pd.numVertices()
        debug.tree_tag = np.array([pd.getVertex(i).getTag() for i in range(n)], dtype=int)
        edges = [(i, j) for i in range(n) for j in pd.getEdges(i)]
        debug.tree_edges = np.array(edges, dtype=int).reshape(-1, 2)
        debug.tree_q = np.array(
            [[pd.getVertex(i).getState()[k] for k in range(6)] for i in range(n)]
        ).reshape(n, 6)
        debug.planner_data_ok = True
    except Exception as e:  # noqa: BLE001 — instrumentation must never break planning
        debug.planner_data_error = f"{type(e).__name__}: {e}"


class OMPLPlanner(Planner):
    """Base for OMPL geometric planners over the FCL collision world."""

    def _make_planner(self, si):
        """Construct and configure the OMPL planner instance for this algorithm.

        Args:
            si: ``ob.SpaceInformation`` for the bounded 6-D joint space.

        Returns:
            A configured ``ompl.geometric`` planner.
        """
        raise NotImplementedError

    def plan(
        self,
        world,
        start_q: np.ndarray,
        goal_qs: np.ndarray,
        *,
        seed: int | None = None,
        debug: PlanDebug | None = None,
    ) -> PlanResult:
        from ompl import base as ob  # noqa: PLC0415
        from ompl import geometric as og  # noqa: PLC0415
        from ompl import util as ou  # noqa: PLC0415

        goal_qs = np.atleast_2d(np.asarray(goal_qs, dtype=float))
        if len(goal_qs) == 0:
            return PlanResult(None, 0.0, 0.0, "no_goals")

        # Planners that cannot take a goal set get the goal nearest the start. The final
        # goal_index is still resolved against the caller's full array, so the trial record
        # keeps pointing at the right goal configuration either way.
        ompl_goals = goal_qs
        if not self.supports_multi_goal and len(goal_qs) > 1:
            nearest = int(np.argmin(np.linalg.norm(goal_qs - np.asarray(start_q, dtype=float), axis=1)))
            ompl_goals = goal_qs[nearest : nearest + 1]

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
        checked_q: list[np.ndarray] = []
        checked_valid: list[bool] = []

        def is_valid(state) -> bool:
            q = np.array([state[i] for i in range(6)])
            ok = not world.in_collision(q)
            if debug is not None:
                checked_q.append(q)
                checked_valid.append(ok)
            return ok

        ss.setStateValidityChecker(is_valid)
        si = ss.getSpaceInformation()
        si.setStateValidityCheckingResolution(float(self.resolution_rad / space.getMaximumExtent()))

        start = space.allocState()
        for i in range(6):
            start[i] = float(start_q[i])
        ss.setStartState(start)

        goal = ob.GoalStates(si)
        for g in ompl_goals:
            st = space.allocState()
            for i in range(6):
                st[i] = float(g[i])
            goal.addState(st)
        ss.setGoal(goal)
        planner = self._make_planner(si)
        ss.setPlanner(planner)

        def fill_debug() -> None:
            if debug is None:
                return
            debug.n_checks = len(checked_q)
            debug.checked_q = np.array(checked_q).reshape(len(checked_q), 6)
            debug.checked_valid = np.array(checked_valid, dtype=bool)
            _extract_planner_data(planner, si, debug)

        t0 = time.perf_counter()
        solved = ss.solve(float(self.budget_s))
        plan_time = time.perf_counter() - t0
        if not solved or not ss.haveExactSolutionPath():
            fill_debug()  # a timeout tree is exactly what the visual diagnoses
            return PlanResult(None, plan_time, 0.0, "timeout")

        if debug is not None:
            raw = ss.getSolutionPath()
            debug.raw_path_q = np.array(
                [[raw.getState(i)[j] for j in range(6)] for i in range(raw.getStateCount())]
            )

        if self.simplify:
            ss.simplifySolution()
        path = ss.getSolutionPath()
        waypoints = np.array([[path.getState(i)[j] for j in range(6)] for i in range(path.getStateCount())])
        length = float(np.sum(np.linalg.norm(np.diff(waypoints, axis=0), axis=1)))
        goal_index = int(np.argmin(np.linalg.norm(goal_qs - waypoints[-1], axis=1)))
        fill_debug()
        return PlanResult(waypoints, plan_time, length, "solved", goal_index)
