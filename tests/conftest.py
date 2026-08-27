# Copyright (c) 2026, dishsim project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Shared test fixtures (import the markers with ``from conftest import ...``)."""

import os

import pytest

from dishsim import config


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
