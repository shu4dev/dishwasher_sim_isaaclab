# Copyright (c) 2026, dishsim project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""RRT-Connect — the project's default planner.

Bidirectional, non-optimizing: it returns as soon as the two trees connect, which is what makes
it fast enough for the cluttered machine front. The tree tags in
:class:`~dishsim.planners.base.PlanDebug` are RRT-Connect specific (1 = start tree,
2 = goal tree).
"""

from .. import config
from .ompl_base import OMPLPlanner


class RRTConnectPlanner(OMPLPlanner):
    """Bidirectional RRT.

    Params:
        range_rad: Maximum extension step [rad]; smaller values negotiate the cluttered
            machine front more reliably (default :data:`dishsim.config.PLAN_RRT_RANGE_RAD`).
    """

    name = "rrt_connect"

    def _make_planner(self, si):
        from ompl import geometric as og  # noqa: PLC0415

        planner = og.RRTConnect(si)
        planner.setRange(float(self.params.get("range_rad", config.PLAN_RRT_RANGE_RAD)))
        return planner
