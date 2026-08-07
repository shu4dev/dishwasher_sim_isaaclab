# Copyright (c) 2026, dishsim project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Planner registry — how the experiment runner turns ``--planner <name>`` into an algorithm.

Adding a planner is a two-step change that never touches the runner:

1. Write a module implementing :class:`~dishsim.planners.base.Planner` (subclass
   :class:`~dishsim.planners.ompl_base.OMPLPlanner` for anything OMPL provides, or implement
   ``Planner`` directly for a non-OMPL method such as a lattice A* or CHOMP).
2. Add it to :data:`PLANNERS` below.
"""

from .base import Planner
from .bit_star import BITStarPlanner
from .prm import PRMPlanner
from .rrt_connect import RRTConnectPlanner
from .rrt_star import RRTStarPlanner

#: Registry key -> planner class. The key is what ``--planner`` accepts and what is recorded
#: in every trial's provenance block.
PLANNERS: dict[str, type[Planner]] = {
    RRTConnectPlanner.name: RRTConnectPlanner,
    RRTStarPlanner.name: RRTStarPlanner,
    BITStarPlanner.name: BITStarPlanner,
    PRMPlanner.name: PRMPlanner,
}


def available() -> list[str]:
    """Registered planner names, sorted."""
    return sorted(PLANNERS)


def make_planner(name: str, **params) -> Planner:
    """Construct a registered planner.

    Args:
        name: Registry key (see :func:`available`).
        **params: Forwarded to the planner; ``budget_s``, ``resolution_rad`` and ``simplify``
            are understood by every planner, the rest are algorithm-specific.

    Returns:
        A configured :class:`~dishsim.planners.base.Planner`.
    """
    if name not in PLANNERS:
        raise ValueError(f"unknown planner {name!r} (choices: {available()})")
    return PLANNERS[name](**params)
