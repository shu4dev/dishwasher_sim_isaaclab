# Copyright (c) 2026, dishsim project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Randomized countertop layouts: reproducible, separated, supported, reachable.

The layout generator is the only randomness in an episode, and a seed is the only thing the
episode record stores that can reproduce a scene — so determinism here is a correctness property,
not a nicety.

Run with:
    PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /workspace/isaaclab/env_isaaclab/bin/python -m pytest \
        tests/test_task_layout.py -v
"""

import numpy as np
import pytest

from dishsim import config
from dishsim.task import layout as L

CLASSES = list(config.TASK["classes"])


def counter_bounds():
    cx, cy = config.COUNTERTOP_CENTER_W[0], config.COUNTERTOP_CENTER_W[1]
    sx, sy = config.COUNTERTOP_SIZE[0], config.COUNTERTOP_SIZE[1]
    return cx - sx / 2, cx + sx / 2, cy - sy / 2, cy + sy / 2


# ---- determinism -------------------------------------------------------------------------------


def test_same_seed_gives_an_identical_layout():
    a, _ = L.plan_countertop_layout(CLASSES, seed=7)
    b, _ = L.plan_countertop_layout(CLASSES, seed=7)
    assert [i.item_id for i in a] == [i.item_id for i in b]
    for x, y in zip(a, b):
        assert np.array_equal(x.T_base_obj, y.T_base_obj)
        assert x.yaw_deg == y.yaw_deg and np.array_equal(x.xy_w, y.xy_w)


def test_different_seeds_give_different_layouts():
    a, _ = L.plan_countertop_layout(CLASSES, seed=1)
    b, _ = L.plan_countertop_layout(CLASSES, seed=2)
    assert any(not np.allclose(x.T_base_obj, y.T_base_obj) for x, y in zip(a, b))


def test_item_ids_are_stable_and_per_class_indexed():
    items, _ = L.plan_countertop_layout(["mug", "cup", "mug"], seed=0)
    assert [i.item_id for i in items] == ["mug_00", "cup_00", "mug_01"]
    assert [i.instance for i in items] == [0, 0, 1]


# ---- separation, bounds, support ----------------------------------------------------------------


@pytest.mark.parametrize("seed", range(8))
def test_footprints_never_come_closer_than_the_minimum_separation(seed):
    items, _ = L.plan_countertop_layout(CLASSES, seed=seed)
    sep = config.TASK["min_separation_m"]
    for i, p in enumerate(items):
        for q in items[i + 1:]:
            gap = float(np.linalg.norm(p.xy_w - q.xy_w)) - (p.radius_m + q.radius_m)
            assert gap >= sep - 1e-9, f"{p.item_id}/{q.item_id} gap {gap:.4f} < {sep}"


@pytest.mark.parametrize("seed", range(8))
def test_every_footprint_lies_fully_on_the_countertop(seed):
    """Sampling a centre inside the reach rectangle is not enough — the object must be supported."""
    x0, x1, y0, y1 = counter_bounds()
    items, _ = L.plan_countertop_layout(CLASSES, seed=seed)
    for it in items:
        assert x0 <= it.xy_w[0] - it.radius_m and it.xy_w[0] + it.radius_m <= x1, it.item_id
        assert y0 <= it.xy_w[1] - it.radius_m and it.xy_w[1] + it.radius_m <= y1, it.item_id


@pytest.mark.parametrize("seed", range(4))
def test_centres_stay_inside_the_measured_reach_rectangle(seed):
    rect = config.TASK["spawn_rect_w"]
    items, _ = L.plan_countertop_layout(CLASSES, seed=seed)
    for it in items:
        assert rect["x_min"] <= it.xy_w[0] <= rect["x_max"], it.item_id
        assert rect["y_min"] <= it.xy_w[1] <= rect["y_max"], it.item_id


@pytest.mark.parametrize("name", CLASSES)
def test_objects_are_seated_on_the_surface_not_floating_or_sunk(name):
    """Bottom of the bbox sits exactly one spawn hover above the counter, for every yaw."""
    spec = config.OBJECTS[name]
    top = L.countertop_top_z_w()
    for yaw in (0.0, 37.0, 90.0, 180.0, 271.0):
        T = L.surface_pose(spec, (0.75, 0.0), yaw, top, hover=config.TASK["spawn_hover_m"])
        z_center_w = T[2, 3] + config.ROBOT_BASE_POS_W[2]
        bottom_w = z_center_w - L.bottom_offset(spec, T[:3, :3])
        assert bottom_w == pytest.approx(top + config.TASK["spawn_hover_m"], abs=1e-9)


@pytest.mark.parametrize("name", CLASSES)
def test_staging_matches_the_frozen_registry_pose(name):
    """The layout's standing convention must be the COUNTERTOP one, not the rack one.

    ``fill_plan._stand_R`` stands a fork head-up so it can drop into a basket bay; on a counter
    a fork lies flat. Using the rack convention here would spawn cutlery balanced on end.
    """
    spec = config.OBJECTS[name]
    T = L.surface_pose(spec, (0.665, -0.185), 0.0, L.countertop_top_z_w())
    z_center_w = T[2, 3] + config.ROBOT_BASE_POS_W[2]
    assert z_center_w == pytest.approx(spec.countertop_poses_w[0][0][2], abs=2e-3)


# ---- the reachability predicate -----------------------------------------------------------------


def test_unreachable_poses_are_resampled_not_returned():
    """Rejects must be resampled rather than returned, and the funnel must record them.

    The predicate rejects a fixed number of draws instead of a region, so the test does not
    depend on where the sampler happens to land for a given seed.
    """
    n_rejects = 4
    calls = {"n": 0}

    def reach(object_class, T):
        calls["n"] += 1
        return calls["n"] > n_rejects

    items, rej = L.plan_countertop_layout(CLASSES, seed=3, reach_fn=reach)
    assert len(items) == len(CLASSES), "every requested object must still be placed"
    assert rej.unreachable == n_rejects
    assert calls["n"] == n_rejects + len(CLASSES)


def test_accepted_poses_all_satisfy_the_reach_predicate():
    items, rej = L.plan_countertop_layout(CLASSES * 2, seed=3,
                                          reach_fn=lambda c, T: T[1, 3] < 0.0)
    assert all(it.T_base_obj[1, 3] < 0.0 for it in items)
    assert rej.unreachable > 0, "a half-plane predicate should reject at least one draw"


def test_an_impossible_reach_predicate_fails_loudly():
    with pytest.raises(RuntimeError, match="could not place"):
        L.plan_countertop_layout(["mug"], seed=0, reach_fn=lambda c, T: False, max_resamples=25)


def test_the_failure_message_names_the_cause():
    with pytest.raises(RuntimeError) as e:
        L.plan_countertop_layout(["mug"], seed=0, reach_fn=lambda c, T: False, max_resamples=10)
    msg = str(e.value)
    assert "mug" in msg and "unreachable" in msg and "10 draws" in msg


# ---- footprint radius ---------------------------------------------------------------------------


def test_round_objects_use_their_radius_not_the_bbox_diagonal():
    """A plate is a disc: the diagonal over-estimates it by 41% and shrinks its usable area."""
    r = L.footprint_radius(config.OBJECTS["plate"])
    h = config.OBJECTS["plate"].bbox_half
    assert r == pytest.approx(max(h[0], h[1]), abs=1e-9)
    assert r < np.hypot(h[0], h[1]) * 0.95


def test_elongated_objects_use_the_diagonal_so_yaw_cannot_escape_it():
    spec = config.OBJECTS["fork"]
    r = L.footprint_radius(spec)
    h = spec.bbox_half
    assert r == pytest.approx(float(np.hypot(h[0], h[1])), abs=1e-9)
    # a yawed fork's corners must stay inside the circle
    for yaw in np.linspace(0, 360, 24):
        R = L.stand_rotation(spec, float(yaw))
        corners = np.array([[sx * h[0], sy * h[1], 0.0] for sx in (-1, 1) for sy in (-1, 1)])
        assert np.max(np.linalg.norm((R @ corners.T).T[:, :2], axis=1)) <= r + 1e-9


def test_effective_rect_shrinks_for_bigger_objects():
    small = L.effective_rect(config.OBJECTS["cup"], config.TASK["spawn_rect_w"])
    big = L.effective_rect(config.OBJECTS["mug"], config.TASK["spawn_rect_w"])
    assert (big["x_max"] - big["x_min"]) < (small["x_max"] - small["x_min"])
    assert (big["y_max"] - big["y_min"]) < (small["y_max"] - small["y_min"])


# ---- Stage B: stacking ---------------------------------------------------------------------------


def test_stacking_off_by_default_leaves_everything_on_the_counter():
    items, _ = L.plan_countertop_layout(CLASSES, seed=5)
    assert all(it.support is None for it in items)


def test_stacking_records_what_each_object_rests_on():
    # a negative separation forces overlap, so the stacker has something to bite on
    items, _ = L.plan_countertop_layout(["mug"] * 6, seed=11, allow_stacking=True,
                                        min_separation_m=-0.2)
    supported = [it for it in items if it.support is not None]
    assert supported, "no object was stacked despite forced overlap"
    ids = {it.item_id for it in items}
    for it in supported:
        assert it.support in ids and it.support != it.item_id


def test_a_stacked_object_sits_above_the_one_it_rests_on():
    items, _ = L.plan_countertop_layout(["mug"] * 6, seed=11, allow_stacking=True,
                                        min_separation_m=-0.2)
    by_id = {it.item_id: it for it in items}
    for it in items:
        if it.support is None:
            continue
        assert it.T_base_obj[2, 3] > by_id[it.support].T_base_obj[2, 3]


# ---- Stage B: deliberate stacking ----------------------------------------------------------------


def test_stacking_actually_produces_stacks_across_seeds():
    """Relying on a uniform draw to land one footprint on another almost never stacks.

    Without deliberate aiming the support constraint would go untested on most seeds, so
    "stacking enabled" has to mean stacks actually appear.
    """
    stacked_seeds = 0
    for seed in range(12):
        items, _ = L.plan_countertop_layout(["cup"] * 3, seed=seed, allow_stacking=True)
        if any(it.support is not None for it in items):
            stacked_seeds += 1
    assert stacked_seeds >= 4, f"only {stacked_seeds}/12 seeds produced a stack"


def test_stack_fraction_controls_how_often_stacks_appear():
    """``stack_fraction`` governs DELIBERATE aiming, not whether stacking is possible at all.

    With stacking enabled the separation rule is off, so two objects can still overlap by chance
    and are correctly recorded as stacked. What the knob must do is make stacks common rather
    than incidental.
    """
    def n_stacked(fraction):
        return sum(
            any(it.support is not None
                for it in L.plan_countertop_layout(["cup"] * 3, seed=s, allow_stacking=True,
                                                   stack_fraction=fraction)[0])
            for s in range(12)
        )

    assert n_stacked(0.0) < n_stacked(1.0)
    assert n_stacked(1.0) == 12, "aiming every object should stack on every seed"


def test_stack_fraction_one_stacks_every_object_after_the_first():
    items, _ = L.plan_countertop_layout(["cup"] * 4, seed=1, allow_stacking=True,
                                        stack_fraction=1.0)
    assert items[0].support is None, "the first object has nothing to rest on"
    assert all(it.support is not None for it in items[1:])


def test_deliberate_stacking_stays_deterministic():
    a, _ = L.plan_countertop_layout(["cup"] * 3, seed=5, allow_stacking=True)
    b, _ = L.plan_countertop_layout(["cup"] * 3, seed=5, allow_stacking=True)
    assert [i.support for i in a] == [i.support for i in b]
    for x, y in zip(a, b):
        assert np.array_equal(x.T_base_obj, y.T_base_obj)


def test_a_stacked_layout_produces_a_support_graph_that_matches_the_intent():
    """The layout's own record of what it stacked must agree with what the graph derives."""
    from dishsim.task import support as S
    from dishsim.task.sequencer import TaskItem

    items, _ = L.plan_countertop_layout(["cup"] * 3, seed=2, allow_stacking=True,
                                        stack_fraction=1.0)
    tasks = [TaskItem(item_id=i.item_id, object_class=i.object_class, instance=i.instance,
                      T_base_obj=i.T_base_obj) for i in items]
    g = S.build_support_graph(tasks, backend="geometric")
    for it in items:
        if it.support is not None:
            assert it.support in g.rests_on[it.item_id], (
                f"{it.item_id} was spawned on {it.support} but the graph says "
                f"{g.rests_on[it.item_id]}")
