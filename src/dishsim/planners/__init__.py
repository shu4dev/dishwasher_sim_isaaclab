# Copyright (c) 2026, dishsim project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Pluggable joint-space motion planners.

The experiment runner selects an algorithm by name (``config.PLANNER`` or ``--planner``) and
calls :meth:`~dishsim.planners.base.Planner.plan`; no experiment code names an algorithm.
See :mod:`dishsim.planners.registry` for how to add one.
"""

from .base import PlanDebug, Planner, PlanResult
from .bit_star import BITStarPlanner
from .ompl_base import OMPLPlanner
from .prm import PRMPlanner
from .registry import PLANNERS, available, make_planner
from .rrt_connect import RRTConnectPlanner
from .rrt_star import RRTStarPlanner

__all__ = [
    "PLANNERS",
    "BITStarPlanner",
    "OMPLPlanner",
    "PRMPlanner",
    "PlanDebug",
    "PlanResult",
    "Planner",
    "RRTConnectPlanner",
    "RRTStarPlanner",
    "available",
    "make_planner",
]
