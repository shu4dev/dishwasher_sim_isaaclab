# Copyright (c) 2026, dishsim project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Venv checks on the deterministic full-load plan (skips cleanly before the caches exist)."""

import os

import numpy as np
import pytest

from dishsim import config
from dishsim import fill_plan

needs_cache = pytest.mark.skipif(
    not os.path.exists(os.path.join(config.CACHE_DIR, "scene_state.json")),
    reason="baseline geometry cache not built yet (run scripts/setup/extract_geometry.py)",
)

needs_meshes = pytest.mark.skipif(
    not os.path.isdir(os.path.join(config.ASSETS_DIR, "props", "meshes")),
    reason="object meshes not built yet (run scripts/setup/build_object_assets.py)",
)


@needs_cache
def test_plan_deterministic_and_ordered():
    a = fill_plan.plan_full_load(config.CACHE_DIR)
    b = fill_plan.plan_full_load(config.CACHE_DIR)
    assert [i.item_id for i in a] == [i.item_id for i in b]
    assert [i.spawn_order for i in a] == list(range(len(a)))
    for x, y in zip(a, b):
        assert np.allclose(x.T_base_obj, y.T_base_obj)


@needs_cache
def test_plan_class_minimums():
    items = fill_plan.plan_full_load(config.CACHE_DIR)
    counts = {}
    for it in items:
        counts[it.name] = counts.get(it.name, 0) + 1
    assert counts["plate"] >= 7
    assert counts["fork"] + counts["spoon"] + counts["knife"] >= 10
    assert counts["mug"] + counts["cup"] + counts["tumbler"] + counts["wine_glass"] >= 6
    assert counts["bowl"] >= 2 and counts["saucer"] >= 3
    assert len(items) >= 30


@needs_cache
@needs_meshes
def test_plan_validates_no_overlap_and_closable():
    items = fill_plan.plan_full_load(config.CACHE_DIR)
    report = fill_plan.validate_plan(items, config.CACHE_DIR)  # asserts pairwise no-overlap
    assert report["n_items"] == len(items)
    assert report["closability_violations"] == []
