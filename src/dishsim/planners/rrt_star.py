# Copyright (c) 2026, dishsim project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""RRT* — asymptotically optimal single-tree RRT.

Unlike RRT-Connect this is an *optimizing* planner: it keeps improving the path until the time
budget expires, so a query always costs the full ``budget_s``. Give it a smaller budget than
the default when running sweeps (``--planner_param budget_s=5``).
"""

from .. import config
from .ompl_base import OMPLPlanner


class RRTStarPlanner(OMPLPlanner):
    """Asymptotically optimal RRT.

    Params:
        range_rad: Maximum extension step [rad].
        goal_bias: Probability of sampling the goal directly (OMPL default 0.05).
    """

    name = "rrt_star"

    def _make_planner(self, si):
        from ompl import geometric as og  # noqa: PLC0415

        planner = og.RRTstar(si)
        planner.setRange(float(self.params.get("range_rad", config.PLAN_RRT_RANGE_RAD)))
        if "goal_bias" in self.params:
            planner.setGoalBias(float(self.params["goal_bias"]))
        return planner
