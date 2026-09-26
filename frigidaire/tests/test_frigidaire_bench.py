"""Kit-free checks of the HOTEC benchmark library (frigidaire/scripts/experiment/frigidaire_bench.py)."""
from __future__ import annotations

import importlib.util
import math
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[2]


@pytest.fixture(scope="module")
def B():
    spec = importlib.util.spec_from_file_location("frigidaire_bench", ROOT / "frigidaire/scripts/experiment/frigidaire_bench.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_tiers_counts_caps_and_n_ranges(B):
    assert [len(B.roster(t)) for t in B.TIERS] == [7, 15, 23]
    assert {k for _, k in B.roster("hard")} == {"plate", "bowl", "cup"}
    for tier, (lo, hi), slack in (("easy", (2, 5), 3), ("medium", (4, 11), 1), ("hard", (6, 17), 0)):
        ns = {B.draw_n(tier, s) for s in range(200)}
        assert min(ns) >= lo and max(ns) <= hi and len(ns) > 3
        assert B.cap_of(tier, 5) == 5 + slack
    assert B.draw_n("hard", 3) == B.draw_n("hard", 3)            # from (tier, seed) only


def test_tub_wall_and_goal_tolerances(B):
    pts = np.array([[0., 0., 0.], [.01, 0., 0.]])
    assert B.tub_clear(pts, [.26, 0., 0.], [0., 0., 0., 1.])
    assert not B.tub_clear(pts, [.271, 0., 0.], [0., 0., 0., 1.])      # a point at x = .281 > .277
    eye = np.eye(4)
    centroid = np.array([0., 0., .03])
    moved = eye.copy(); moved[0, 3] = .014
    assert B.within_goal("bowl", B.goal_distance(centroid, moved, eye))
    moved[0, 3] = .016
    assert not B.within_goal("bowl", B.goal_distance(centroid, moved, eye))
    assert B.within_goal("plate", B.goal_distance(centroid, moved, eye))    # plates allow 18 mm
    tilt = eye.copy()
    a = math.radians(16.)
    tilt[:3, :3] = [[1, 0, 0], [0, math.cos(a), -math.sin(a)], [0, math.sin(a), math.cos(a)]]
    assert not B.within_goal("cup", B.goal_distance(np.zeros(3), tilt, eye))


def test_family_filters_drop_the_mid_zone_and_the_outer_front_right(B):
    pts = np.array([[-.07, 0., 0.], [.07, 0., 0.]])                  # a 140 mm bowl
    c = lambda slot, x: {"slot": slot, "position": [x, 0., 0.], "quaternion_xyzw": [0., 0., 0., 1.]}
    assert not B.family_ok(c("lower_mid", 0.), pts)
    assert B.family_ok(c("lower_frontright", .165), pts)
    assert not B.family_ok(c("lower_frontright", .170), pts)          # settles past the tub wall (goal gates)
    assert B.family_ok(c("lower_rear", .205), pts)                    # other slots: the wall itself (lip .275)
    assert not B.family_ok(c("lower_rear", .208), pts)


def test_racked_in_uses_the_rack_boxes(B):
    pts = np.zeros((1, 3))
    frames = {"LowerRack": {"position_m": [0., -.49, .2], "quaternion_xyzw": [0., 0., 0., 1.]},
              "UpperRack": {"position_m": [0., -.44, .5], "quaternion_xyzw": [0., 0., 0., 1.]}}
    assert B.racked_in(pts, {"position_m": [0., -.49, .25], "quaternion_xyzw": [0., 0., 0., 1.]}, frames) == "LowerRack"
    assert B.racked_in(pts, {"position_m": [.5, -.49, .25], "quaternion_xyzw": [0., 0., 0., 1.]}, frames) is None


class _StubDishes:
    def collides(self, c, skip=()):
        return False
    def add(self, oid, c):
        pass
    def remove(self, oid):
        pass


def test_ascend_stops_without_gain_and_offers_keeps_only_to_their_dish(B, monkeypatch):
    fam = {"cup": [({"kind": "cup", "rack": "UpperRack", "slot": f"s{i}", "variant": "v", "position": [i * .1, 0., 0.],
                     "quaternion_xyzw": [0., 0., 0., 1.]}, None) for i in range(3)]}
    masks = {"cup": np.ones(3, bool)}
    monkeypatch.setattr(B, "DishSet", lambda parts, points: _StubDishes())
    entries = [B.entry_of("cup_01", fam["cup"][0][0]), B.entry_of("cup_02", fam["cup"][1][0])]
    keep = {"cup_01": {"kind": "cup", "rack": "UpperRack", "slot": "keep_cup_01", "variant": "keep",
                       "position": [.9, 0., 0.], "quaternion_xyzw": [0., 0., 0., 1.]}}
    seen = []

    def score(es):
        seen.append([e["slot"] for e in es])
        return {"score": 1. if es[0]["slot"] == "keep_cup_01" else .5, "worst": 0., "feasible": True}

    iso = lambda cands: np.array([1. if c["variant"] == "keep" else 0. for c in cands])
    out, res, hist, sweeps, stop = B.ascend(entries, fam, masks, None, None, score, iso, np.random.default_rng(0), keep)
    assert out[0]["slot"] == "keep_cup_01" and out[0]["keep"] and res["score"] == 1.
    assert not any("keep_cup_01" == slots[1] for slots in seen)      # never offered to cup_02
    assert sweeps == 2 and stop == "no_gain"                          # gain in sweep 1, none in sweep 2


def test_fixed_goal_sequencer_follows_targets(B):
    from dishsim.rearrange import Instance, Move

    class World:
        def snapshot(self):
            return {}
    import types
    P = B._planner()
    calls = {}

    def fake_sequence(world, goals, order, cap, max_moves=None):
        calls.update(goals=goals, order=order, cap=cap)
        return [Move(k, v) for k, v in goals.items()]
    orig, P.sequence = P.sequence, fake_sequence
    try:
        inst = Instance(name="x", machine="m", base_placement="b", state="s", meta={"counter_cap": 4},
                        items=[{"item_id": "bowl_01", "object_class": "bowl", "T_base_init": np.eye(4),
                                "target": {"T_base_obj": np.eye(4) * 2}}])
        inst.planner_instance = {"objects": [{"object_id": "bowl_01", "start": "Counter",
                                              "pose_world": {"position_m": [0, 0, 1], "quaternion_xyzw": [0, 0, 0, 1]}}]}
        algo = B.FixedGoalSequencer()
        algo.reset(inst, World())
        assert calls["cap"] == 4 and calls["order"] == ["bowl_01"]
        assert algo.next_move({}).item_id == "bowl_01" and algo.next_move({}) is None
    finally:
        P.sequence = orig


def test_rrt_candidates_fall_back_to_goals_and_cells(B):
    from dishsim.rearrange import Instance
    from dishsim import rrt

    class World:
        def buffer_poses(self, cls):
            return [np.eye(4)]
    inst = Instance(name="x", machine="frigidaire", base_placement="none", state="racks_out", meta={},
                    items=[{"item_id": "bowl_01", "object_class": "bowl", "T_base_init": np.eye(4),
                            "target": {"T_base_obj": np.eye(4) * 3}}])
    cands = rrt.RRTConnect(seed=0)._candidates(inst, World())
    assert len(cands["bowl"]) == 2 and np.allclose(cands["bowl"][0], np.eye(4) * 3)


def test_rel_maps_host_paths_to_container_paths(B):
    assert B.rel(B.OUT / "baseline") == "results/benchmark/frigidaire_hotec/baseline"       # symlinked root stays relative
    assert B.rel(B.ROOT / "frigidaire/scripts") == "frigidaire/scripts"
    assert B.rel("/media/corallab-s1/2tbhdd/brianshu/dishsim/x") == "/media/corallab-s1/2tbhdd/brianshu/dishsim/x"
    with pytest.raises(ValueError):
        B.rel("/tmp/elsewhere")


def test_nests_accepts_keep_candidates_without_an_orient_matrix(B):
    pytest.importorskip("pxr")
    from dishsim_frigidaire.loading import visual_points
    points = {"bowl": visual_points(B.ASSETS / "bowl.usda")}
    up = [0., 0., 0., 1.]
    keep = {"kind": "bowl", "rack": "LowerRack", "slot": "keep_bowl_01", "variant": "keep", "position": [0., 0., .1], "quaternion_xyzw": up}
    same = [{"id": "bowl_02", "kind": "bowl", "rack": "LowerRack", "position": [0., 0., .12], "quaternion_xyzw": up}]
    far = [{"id": "bowl_02", "kind": "bowl", "rack": "LowerRack", "position": [0., 0., .4], "quaternion_xyzw": up}]
    beside = [{"id": "bowl_02", "kind": "bowl", "rack": "LowerRack", "position": [.3, 0., .1], "quaternion_xyzw": up}]
    assert B.nests(points, keep, None, same) and not B.nests(points, keep, None, far)
    assert not B.nests(points, keep, None, beside)                     # same height, 30 cm apart: cannot nest


def test_banned_pairs_block_only_the_pair_and_poses_across_slot_names(B):
    q = [0., 0., 0., 1.]
    a = {"rack": "LowerRack", "slot": "lower_rear", "variant": "Y120", "position": [-.065, .155, .17], "quaternion_xyzw": q}
    b = {"rack": "LowerRack", "slot": "lower_rear", "variant": "X150", "position": [.05, .115, .13], "quaternion_xyzw": q}
    c = {"rack": "LowerRack", "slot": "lower_rear", "variant": "X150b", "position": [-.17, .155, .17], "quaternion_xyzw": q}
    bans = {frozenset((B.cand_key(a), B.cand_key(b)))}
    assert B.banned(b, [a], bans) and B.banned(a, [c, b], bans)
    assert not B.banned(c, [a], bans) and not B.banned(b, [a], set())
    same_pose = dict(c, slot="lower_rearright", variant="other")            # one pose under two slot names
    assert B.banned(same_pose, [], {frozenset((B.cand_key(c),))})
    old = {frozenset((f"{c['rack']}|{c['slot']}|{c['variant']}",))}          # an old slot-keyed ban still applies
    assert B.banned(c, [], old)


def test_neighbour_order_keeps_the_build_order_between_close_dishes_only(B):
    poses = {"a": {"position_m": [0., 0., .1]}, "b": {"position_m": [.05, 0., .1]}, "c": {"position_m": [.5, 0., .1]},
             "d": {"position_m": [.06, 0., .6]}}
    racks = {"a": "LowerRack", "b": "LowerRack", "c": "LowerRack", "d": "UpperRack"}
    assert B.neighbour_order(["b", "a", "c", "d"], poses, racks) == [("b", "a")]
