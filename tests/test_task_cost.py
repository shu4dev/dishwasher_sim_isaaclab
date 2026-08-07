# Copyright (c) 2026, dishsim project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""The pick-order cost registry — the same contract the planner registry has.

Parametrizing over ``available_costs()`` means a newly registered heuristic is covered the
moment it is added, without anyone remembering to write a test for it.

Run with:
    PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /workspace/isaaclab/env_isaaclab/bin/python -m pytest \
        tests/test_task_cost.py -v
"""

from dataclasses import dataclass

import numpy as np
import pytest

from dishsim import config
from dishsim.task.cost import COST_FUNCTIONS, available_costs, make_cost


@dataclass
class Cand:
    item_id: str
    object_class: str
    T_base_obj: np.ndarray
    T_base_grasp: np.ndarray
    grasp_q: np.ndarray | None = None


def cand(x, q=None, item_id="a"):
    T = np.eye(4)
    T[:3, 3] = (x, 0.0, 0.0)
    return Cand(item_id, "mug", T, T, None if q is None else np.asarray(q, dtype=float))


def tcp_at(x=0.0):
    T = np.eye(4)
    T[:3, 3] = (x, 0.0, 0.0)
    return T


# ---- registry contract --------------------------------------------------------------------------


def test_registry_is_non_empty_and_sorted():
    names = available_costs()
    assert names == sorted(names) and names
    assert set(names) == set(COST_FUNCTIONS)


@pytest.mark.parametrize("name", available_costs())
def test_every_registered_cost_returns_a_finite_float(name):
    fn = make_cost(name)
    out = fn(cand(0.3, q=np.zeros(6)), np.zeros(6), tcp_at(0.0))
    assert isinstance(out, float) and np.isfinite(out)


@pytest.mark.parametrize("name", available_costs())
def test_every_registered_cost_is_deterministic(name):
    fn = make_cost(name)
    c, q, T = cand(0.3, q=np.ones(6)), np.zeros(6), tcp_at(0.1)
    assert fn(c, q, T) == fn(c, q, T)


@pytest.mark.parametrize("name", available_costs())
def test_every_registered_cost_tolerates_a_missing_grasp_configuration(name):
    """A candidate may be known only in Cartesian terms; no heuristic may crash on that."""
    fn = make_cost(name)
    assert np.isfinite(fn(cand(0.3, q=None), np.zeros(6), tcp_at(0.0)))


def test_unknown_name_raises_with_the_choices():
    with pytest.raises(ValueError, match="unknown cost function"):
        make_cost("does_not_exist")


def test_the_configured_default_is_registered():
    assert config.TASK["cost_fn"] in available_costs()


# ---- the individual heuristics -------------------------------------------------------------------


def test_nearest_first_is_the_cartesian_distance_to_the_grasp_point():
    fn = make_cost("nearest_first")
    assert fn(cand(0.3), np.zeros(6), tcp_at(0.0)) == pytest.approx(0.3)
    assert fn(cand(0.3), np.zeros(6), tcp_at(0.25)) == pytest.approx(0.05)


def test_nearest_first_ignores_the_arm_configuration():
    """Two candidates in the same place must score identically however the arm would get there."""
    fn = make_cost("nearest_first")
    a = fn(cand(0.3, q=np.zeros(6)), np.zeros(6), tcp_at(0.0))
    b = fn(cand(0.3, q=np.full(6, 3.0)), np.zeros(6), tcp_at(0.0))
    assert a == b


def test_shortest_ik_is_the_joint_space_distance():
    fn = make_cost("shortest_ik")
    assert fn(cand(0.9, q=np.array([0.1] * 6)), np.zeros(6), tcp_at(0.0)) == pytest.approx(
        float(np.linalg.norm(np.full(6, 0.1)))
    )


def test_shortest_ik_can_disagree_with_nearest_first():
    """The reason both exist: a short Cartesian hop can be a long joint-space move."""
    near, ik = make_cost("nearest_first"), make_cost("shortest_ik")
    close_but_awkward = cand(0.10, q=np.full(6, 2.5), item_id="awkward")
    far_but_easy = cand(0.40, q=np.full(6, 0.05), item_id="easy")
    q, T = np.zeros(6), tcp_at(0.0)
    assert near(close_but_awkward, q, T) < near(far_but_easy, q, T)
    assert ik(close_but_awkward, q, T) > ik(far_but_easy, q, T)


def test_shortest_ik_falls_back_to_cartesian_without_a_configuration():
    near, ik = make_cost("nearest_first"), make_cost("shortest_ik")
    c, q, T = cand(0.3, q=None), np.zeros(6), tcp_at(0.0)
    assert ik(c, q, T) == near(c, q, T)


def test_farthest_first_reverses_nearest_first():
    near, far = make_cost("nearest_first"), make_cost("farthest_first")
    cands = [cand(0.1, item_id="a"), cand(0.5, item_id="b"), cand(0.3, item_id="c")]
    q, T = np.zeros(6), tcp_at(0.0)
    assert ([c.item_id for c in sorted(cands, key=lambda c: near(c, q, T))]
            == list(reversed([c.item_id for c in sorted(cands, key=lambda c: far(c, q, T))])))
