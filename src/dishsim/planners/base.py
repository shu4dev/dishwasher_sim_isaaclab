# Copyright (c) 2026, dishsim project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""The planner interface every motion-planning algorithm in this repo implements.

A planner answers one question: *given a collision world, a start configuration and a set of
acceptable goal configurations, find a collision-free joint path.* Everything algorithm-specific
lives behind :meth:`Planner.plan`; the experiment runner selects an implementation by name and
never mentions an algorithm.

Two deliberate choices in the signature:

- **The world and the seed are per-call, not per-instance.** One trial plans against several
  different collision worlds (rack approach, pick approach, place) with different seeds, so a
  planner instance is a configured *algorithm*, not a bound problem.
- **The only contract a planner needs from a world is** ``in_collision(q) -> bool``. Anything
  that answers that — :class:`dishsim.collision_world.CollisionWorld` or a test stub — is a
  valid world.
"""

from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import ClassVar

import numpy as np

from .. import config


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
    """Opt-in instrumentation for one :meth:`Planner.plan` call (pass an instance; filled in place).

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


class Planner(ABC):
    """Base class for joint-space planners over a collision world.

    Subclasses set :attr:`name` and implement :meth:`plan`. Register them in
    :mod:`dishsim.planners.registry` to make them selectable by name from the CLI.
    """

    #: Registry key (also what lands in the trial record).
    name: ClassVar[str] = ""

    #: Whether the algorithm accepts a goal *set*. Planners that cannot are given the goal
    #: nearest the start instead, and say so in :attr:`PlanResult.status` via ``goal_index``.
    #: (Measured, not assumed: the roadmap planners in this OMPL build never terminate with a
    #: multi-state ``ob.GoalStates`` — see :class:`~dishsim.planners.prm.PRMPlanner`.)
    supports_multi_goal: ClassVar[bool] = True

    def __init__(
        self,
        *,
        budget_s: float = config.PLAN_TIME_BUDGET_S,
        resolution_rad: float = config.PLAN_VALIDITY_RESOLUTION_RAD,
        simplify: bool = True,
        **params,
    ) -> None:
        """
        Args:
            budget_s: Planning time budget [s].
            resolution_rad: Motion-validation step [rad].
            simplify: Run path simplification on the solution.
            **params: Algorithm-specific parameters (see each subclass).
        """
        self.budget_s = float(budget_s)
        self.resolution_rad = float(resolution_rad)
        self.simplify = bool(simplify)
        self.params = dict(params)

    @abstractmethod
    def plan(
        self,
        world,
        start_q: np.ndarray,
        goal_qs: np.ndarray,
        *,
        seed: int | None = None,
        debug: PlanDebug | None = None,
    ) -> PlanResult:
        """Plan a collision-free joint path from ``start_q`` to any of ``goal_qs``.

        Args:
            world: Validity oracle exposing ``in_collision(q) -> bool``.
            start_q: Start configuration, shape [6].
            goal_qs: Goal set, shape [K, 6] (K >= 1).
            seed: Planner RNG seed, for reproducibility.
            debug: Optional :class:`PlanDebug` filled in place.

        Returns:
            A :class:`PlanResult`; ``path_q`` includes the start as row 0.
        """

    def describe(self) -> dict:
        """Provenance block recorded with every trial planned by this instance."""
        return {
            "name": self.name,
            "budget_s": self.budget_s,
            "resolution_rad": self.resolution_rad,
            "simplify": self.simplify,
            "supports_multi_goal": self.supports_multi_goal,
            **{k: v for k, v in self.params.items()},
        }

    def __repr__(self) -> str:  # pragma: no cover - debugging aid
        return f"{type(self).__name__}({self.describe()})"
