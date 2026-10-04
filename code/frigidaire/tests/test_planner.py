# Copyright (c) 2026, dishsim project.
# SPDX-License-Identifier: BSD-3-Clause
"""Bench planner helpers: counter band, support gate, sequencer, driver integration (Kit-free)."""
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "code/frigidaire" / "src"))
sys.path.insert(0, str(ROOT / "code/src"))

from dishsim import rearrange  # noqa: E402
from dishsim.transforms import make_T  # noqa: E402
from dishsim_frigidaire import planner as P  # noqa: E402

TOP = P.COUNTER["top_z_m"]


def T(x, y, z):
    return make_T((x, y, z), (0., 0., 0., 1.))


class FakeWorld:
    """Planner-world duck-type in a flat model: two objects collide when closer than 0.1 m in xy."""

    def __init__(self, initial, support=()):
        self.initial = {k: np.asarray(v, float) for k, v in initial.items()}
        self.support = [tuple(e) for e in support]
        self.classes, self._poses, self.certified, self.n_queries = {}, {}, set(), 0
        self.frames = {}

    def sync(self, poses, classes):
        self.classes.update(classes)
        self._poses.update({k: np.asarray(v, float) for k, v in poses.items()})

    def snapshot(self):
        return {k: v.copy() for k, v in self._poses.items()}

    def certify(self, T):
        pass

    def blockers(self, item_id, T, object_class=None):
        self.n_queries += 1
        p = np.asarray(T)[:3, 3]
        return sorted(k for k, q in self._poses.items()
                      if k != item_id and np.linalg.norm(q[:2, 3] - p[:2]) < .1 and abs(q[2, 3] - p[2]) < .1)

    def move_collides(self, item_id, T, object_class=None):
        return bool(self.blockers(item_id, T))

    def buffer_poses(self, kind):
        return [T(-.5 + .15 * i, 0., TOP + .05) for i in range(6)]

    def in_counter(self, T_):
        return P.in_counter_band(T_)

    def resting_on(self, item_id):
        return [b for a, b in self.support if a == item_id and np.allclose(self._poses[b], self.initial[b], atol=1e-6)]


class FakeOracle:
    def __init__(self, world, targets):
        self.world, self.targets, self.poses = world, targets, dict(world.initial)

    def at_goal(self, item, T_):
        t = self.targets.get(item["item_id"])
        return t is not None and bool(np.allclose(T_, t, atol=1e-6))

    def execute(self, move):
        disturbed = self.world.resting_on(move.item_id)
        self.poses[move.item_id] = np.asarray(move.T_base_obj, float)
        return dict(self.poses), ("disturbed" if disturbed else None), {"disturbed": disturbed}


def scripted(instance, moves):
    class Scripted:
        def reset(self, inst, world):
            self.moves = list(moves)

        def next_move(self, obs):
            return self.moves.pop(0) if self.moves else None
    return Scripted()


def instance_of(poses, targets, cap):
    items = [{"item_id": k, "object_class": "bowl", "T_base_init": np.asarray(v), "target": {"T_base_obj": targets.get(k)}}
             for k, v in poses.items()]
    return rearrange.Instance(name="toy", machine="frigidaire", base_placement="none", state="toy", items=items,
                              meta={"counter_cap": cap})


def test_counter_band():
    assert P.in_counter_band(T(0., 0., TOP + .03))
    assert P.in_counter_band(T(.59, -.29, TOP + .001))
    assert P.in_counter_band(T(0., 0., TOP))                     # a bowl resting flat: origin exactly at the top
    assert not P.in_counter_band(T(0., 0., TOP - .02))           # clearly below the top: not on the counter
    assert not P.in_counter_band(T(.61, 0., TOP + .03))          # off the slab
    assert not P.in_counter_band(T(0., .008, .59))               # an upper-rack pose


def test_support_edges_lower_supports_higher():
    poses = {"a": {"position_m": [0, 0, .95]}, "b": {"position_m": [0, 0, 1.0]}, "c": {"position_m": [.1, 0, .952]},
             "LowerRack": {"position_m": [0, 0, .2]}}
    edges = P.support_edges([["a", "b"], ["a", "c"], ["a", "LowerRack"], ["b", "a"]], poses, ["a", "b", "c"])
    assert edges == [["a", "b"]]                                 # 5 cm higher: supported; 2 mm: not; racks ignored


def test_bottom_first_unstack_is_disturbed_top_first_solves():
    init = {"bottom": T(0., 0., TOP + .03), "top": T(0., 0., TOP + .09)}
    targets = {"bottom": T(0., 0., .3), "top": T(.3, 0., .3)}
    for order, expect in ((["bottom", "top"], "disturbed"), (["top", "bottom"], None)):
        world = FakeWorld(init, support=[("bottom", "top")])
        inst = instance_of(init, targets, cap=2)
        rec = rearrange.run_episode(inst, scripted(inst, [rearrange.Move(k, targets[k]) for k in order]), world,
                                    FakeOracle(world, targets), budget=None, counter_cap=2)
        assert rec["abort"] == expect
        assert rec["solved"] == (expect is None)
        if expect:
            assert rec["moves"][-1]["disturbed"] == ["top"]


def test_sequencer_respects_cap_and_buffers_one_blocker():
    init = {"c": T(0., 0., TOP + .03), "a": T(0., 0., .3), "b": T(.3, 0., .3)}   # a and b must swap cells
    goals = {"c": T(.6, 0., .3), "a": T(.3, 0., .3), "b": T(0., 0., .3)}
    world = FakeWorld(init)
    world.sync(init, {k: "bowl" for k in init})
    assert P.sequence(world, goals, ["c", "a", "b"], cap=0) is None          # no parking allowed: the swap is impossible
    plan = P.sequence(world, goals, ["c", "a", "b"], cap=1)
    assert plan is not None and [m.item_id for m in plan][0] == "c"        # the counter object goes home first
    assert sum(P.in_counter_band(m.T_base_obj) for m in plan) == 1           # exactly one buffer trip for the swap
    assert world.snapshot()["c"][2, 3] == pytest.approx(TOP + .03)          # dry run restored the mirror
    inst = instance_of(init, goals, cap=1)
    rec = rearrange.run_episode(inst, scripted(inst, plan), world, FakeOracle(world, goals), budget=None, counter_cap=1)
    assert rec["solved"] and rec["counter_full_refusals"] == 0 and rec["infeasible_commands"] == 0
    assert rec["buffer_moves"] == 1 and rec["moves_used"] == 4


def test_sequencer_counter_objects_first_and_support_gate():
    init = {"low": T(0., 0., TOP + .03), "high": T(0., 0., TOP + .09), "in": T(.4, 0., .3)}
    goals = {"low": T(0., .2, .3), "high": T(.2, .2, .3), "in": T(.4, 0., .3)}
    world = FakeWorld(init, support=[("low", "high")])
    world.sync(init, {k: "bowl" for k in init})
    plan = P.sequence(world, goals, ["high", "low", "in"], cap=2)
    assert [m.item_id for m in plan] == ["high", "low"]                      # top first; "in" already at its goal


def test_item_order_and_to_rearrange_instance():
    inst = {"instance_id": "x", "inventory": {"bowl": 2}, "seed": 1, "counter": {"cap": 1},
            "objects": [{"object_id": "i", "kind": "bowl", "start": "LowerRack", "pose_world": {"position_m": [0, 0, .3], "quaternion_xyzw": [0, 0, 0, 1]}},
                        {"object_id": "lo", "kind": "bowl", "start": "Counter", "pose_world": {"position_m": [0, 0, .93], "quaternion_xyzw": [0, 0, 0, 1]}},
                        {"object_id": "hi", "kind": "bowl", "start": "Counter", "pose_world": {"position_m": [0, 0, .99], "quaternion_xyzw": [0, 0, 0, 1]}}]}
    assert P.item_order(inst) == ["hi", "lo", "i"]
    r = P.to_rearrange_instance(inst)
    assert r.items[0]["target"]["T_base_obj"] is None and r.meta["counter_cap"] == 1 and r.planner_instance is inst


def test_allowed_kinds_do_not_widen_racks():
    from dishsim_frigidaire import random_poses as R
    from dishsim_frigidaire.initial_state_runtime import normalize_candidates
    assert R.RACKS == ("LowerRack", "UpperRack") and "SilverwareBasket" in R.OBJECT_RACKS and "fork" in R.OBJECT_KINDS
    entry = {"kind": "fork", "rack": "SilverwareBasket", "object_id": "fork_a",
             "rack_local_pose": {"position_m": [0., 0., .1], "quaternion_xyzw": [0., 0., 0., 1.]}}
    assert normalize_candidates([entry])[0]["object_id"] == "fork_a"
    with pytest.raises(ValueError):
        normalize_candidates([dict(entry, rack="Door")])
