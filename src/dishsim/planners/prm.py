# Copyright (c) 2026, dishsim project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""PRM — probabilistic roadmap.

A multi-query method used here single-query: each call builds a fresh roadmap, so it is
typically slower than RRT-Connect on this problem. It earns its place as the structural proof
that the planner interface is not RRT-shaped — PRM exposes neither ``setRange`` nor
``setGoalBias``, which is why parameters are applied per algorithm rather than generically.

**Single-goal only, measured.** The roadmap planners in this OMPL build (``PRM`` and
``PRMstar``) never terminate when the goal is a multi-state ``ob.GoalStates`` — verified at the
raw OMPL level with a trivial validity checker, where ``PRM`` solves a 1-goal query in 0.1 s
and hangs indefinitely on 2 goals, while ``BITstar``/``RRTstar``/``KPIECE1`` solve the same
2-goal query in under a second. Every real query in this project is multi-goal (a slot's IK
goal set), so :class:`PRMPlanner` declares ``supports_multi_goal = False`` and
:class:`~dishsim.planners.ompl_base.OMPLPlanner` hands it the goal nearest the start.
"""

from .ompl_base import OMPLPlanner


class PRMPlanner(OMPLPlanner):
    """Probabilistic roadmap (single-goal; see the module docstring).

    Params:
        max_nearest_neighbors: Cap on roadmap connection attempts per milestone (OMPL default
            is an unbounded k-strategy).
    """

    name = "prm"
    supports_multi_goal = False

    def _make_planner(self, si):
        from ompl import geometric as og  # noqa: PLC0415

        planner = og.PRM(si)
        if "max_nearest_neighbors" in self.params:
            planner.setMaxNearestNeighbors(int(self.params["max_nearest_neighbors"]))
        return planner
