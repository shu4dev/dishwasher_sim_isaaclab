# Copyright (c) 2026, dishsim project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Shared test stubs and fixtures (import the stubs/markers with ``from conftest import ...``)."""

import os

import pytest

from dishsim import config


class FreeWorld:
    """Stub collision world: everything is valid."""

    def in_collision(self, q):
        return False


class StubStats:
    """MotionStats stand-in: just the field the code under test reads."""

    plan_time_s = 0.0


class StubPlanResult:
    """Minimal PlanResult stand-in for stub motion layers."""

    def __init__(self, status, path_q=None):
        self.status = status
        self.path_q = path_q
        self.plan_time_s = 0.0


class WallWorld:
    """Joint-space slab: q1 in (-1.2, -0.8) is blocked unless q0 > 0.5 (forces a detour)."""

    def in_collision(self, q):
        return (-1.2 < q[1] < -0.8) and (q[0] <= 0.5)


#: Skip when the baseline geometry cache has not been baked on this machine.
needs_cache = pytest.mark.skipif(
    not os.path.exists(os.path.join(config.CACHE_DIR, "scene_state.json")),
    reason="baseline geometry cache not built yet (run scripts/setup/extract_geometry.py)",
)


@pytest.fixture
def bosch():
    """Bosch 800 twin at the A2-winner mount; ALWAYS restores the byte-stable v1 baseline."""
    config.apply_machine("bosch800")
    config.apply_base_placement("side_winner")
    yield
    config.apply_machine(config.MACHINE_BASELINE_NAME)
    config.apply_scenario("both_out")  # apply_machine keeps a surviving scenario name
