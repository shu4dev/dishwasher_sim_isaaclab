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


@dataclass
class PlanDebug:
    """Opt-in instrumentation for one :func:`plan_to_goals` call (pass an instance; filled in place).

    Populated on every exit path, including timeouts. The ``tree_*`` fields come from
    ``ob.PlannerData`` when the bindings expose it (``planner_data_ok``); otherwise they stay
    ``None`` and ``planner_data_error`` records the reason.
    """

    checked_q: np.ndarray | None = None  # [M, 6] every validity-checked state [rad], call order
    checked_valid: np.ndarray | None = None  # [M] bool verdict per check
    n_checks: int = 0
    tree_q: np.ndarray | None = None  # [V, 6] planner tree vertex states [rad]
    tree_tag: np.ndarray | None = None  # [V] int; RRT-Connect: 1 = start tree, 2 = goal tree
    tree_edges: np.ndarray | None = None  # [E, 2] int (from, to), includes the connect bridge
    raw_path_q: np.ndarray | None = None  # [N0, 6] solution before simplification [rad]
    planner_data_ok: bool = False
    planner_data_error: str = ""


def _coords_from_graphml(xml_text: str, n_expected: int) -> np.ndarray:
    """Recover planner-tree vertex coordinates from ``PlannerData.printGraphML()`` output.

    Fallback for OMPL builds whose ``PlannerDataVertex.getState()`` returns the method-less
    base state: the GraphML serialization carries the per-vertex reals in a ``coords`` node
    attribute instead.

    Args:
        xml_text: GraphML document produced by ``ob.PlannerData.printGraphML()``.
        n_expected: Vertex count the document must contain (``pd.numVertices()``).

    Returns:
        Vertex coordinates ordered by graph index, shape [n_expected, dim].
    """
    import xml.etree.ElementTree as ET  # noqa: PLC0415

    def local(tag: str) -> str:
        return tag.rsplit("}", 1)[-1]  # strip the GraphML xmlns prefix

    root = ET.fromstring(xml_text)
    coords_key = next(
        (el.get("id") for el in root.iter() if local(el.tag) == "key" and el.get("attr.name") == "coords"),
        None,
    )
    if coords_key is None:
        raise ValueError("GraphML has no 'coords' node attribute")

    rows: list[tuple[int, list[float]]] = []
    for el in root.iter():
        if local(el.tag) != "node":
            continue
        for d in el:
            if local(d.tag) == "data" and d.get("key") == coords_key:
                rows.append((int(el.get("id").lstrip("n")), [float(v) for v in d.text.split(",")]))
                break
    if len(rows) != n_expected:
        raise ValueError(f"GraphML has {len(rows)} coords rows, expected {n_expected}")
    rows.sort(key=lambda r: r[0])
    return np.array([r[1] for r in rows])


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
        try:
            debug.tree_q = np.array(
                [[pd.getVertex(i).getState()[k] for k in range(6)] for i in range(n)]
            ).reshape(n, 6)
        except Exception:  # method-less base ob.State: fall back to the GraphML serialization
            debug.tree_q = _coords_from_graphml(pd.printGraphML(), n)
        debug.planner_data_ok = True
    except Exception as e:  # noqa: BLE001 — instrumentation must never break planning
        debug.planner_data_error = f"{type(e).__name__}: {e}"


def plan_to_goals(
    world,
    start_q: np.ndarray,
    goal_qs: np.ndarray,
    budget_s: float = config.PLAN_TIME_BUDGET_S,
    resolution_rad: float = config.PLAN_VALIDITY_RESOLUTION_RAD,
    simplify: bool = True,
    seed: int | None = None,
    debug: PlanDebug | None = None,
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
        debug: Optional :class:`PlanDebug` filled in place with checker samples, the planner
            tree, and the pre-simplification path (captured even on timeout).

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

    def fill_debug() -> None:
        if debug is None:
            return
        debug.n_checks = len(checked_q)
        debug.checked_q = np.array(checked_q).reshape(len(checked_q), 6)
        debug.checked_valid = np.array(checked_valid, dtype=bool)
        _extract_planner_data(planner, si, debug)

    t0 = time.perf_counter()
    solved = ss.solve(float(budget_s))
    plan_time = time.perf_counter() - t0
    if not solved or not ss.haveExactSolutionPath():
        fill_debug()  # a timeout tree is exactly what the visual diagnoses
        return PlanResult(None, plan_time, 0.0, "timeout")

    if debug is not None:
        raw = ss.getSolutionPath()
        debug.raw_path_q = np.array(
            [[raw.getState(i)[j] for j in range(6)] for i in range(raw.getStateCount())]
        )

    if simplify:
        ss.simplifySolution()
    path = ss.getSolutionPath()
    waypoints = np.array([[path.getState(i)[j] for j in range(6)] for i in range(path.getStateCount())])
    length = float(np.sum(np.linalg.norm(np.diff(waypoints, axis=0), axis=1)))
    goal_index = int(np.argmin(np.linalg.norm(goal_qs - waypoints[-1], axis=1)))
    fill_debug()
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
