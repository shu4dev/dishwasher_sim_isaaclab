# Copyright (c) 2026, dishsim project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""BIT* — Batch Informed Trees.

An optimizing planner that alternates between sampling a batch of states in the informed
ellipsoid and searching the implicit graph heuristically — algorithmically unlike both the
RRT family and roadmaps, which makes it the useful third data point when comparing planners
on this task. Like RRT* it consumes its full time budget.
"""

from .ompl_base import OMPLPlanner


class BITStarPlanner(OMPLPlanner):
    """Batch Informed Trees.

    Params:
        samples_per_batch: States sampled per batch (OMPL default 100).
    """

    name = "bit_star"

    def _make_planner(self, si):
        from ompl import geometric as og  # noqa: PLC0415

        planner = og.BITstar(si)
        if "samples_per_batch" in self.params:
            planner.setSamplesPerBatch(int(self.params["samples_per_batch"]))
        return planner
