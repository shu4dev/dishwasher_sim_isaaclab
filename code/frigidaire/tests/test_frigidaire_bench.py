"""Kit-free checks of the HOTEC benchmark library (code/planner/frigidaire/frigidaire_bench.py)."""
from __future__ import annotations

import importlib.util
import math
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[3]


@pytest.fixture(scope="module")
def B():
    spec = importlib.util.spec_from_file_location("frigidaire_bench", ROOT / "code/planner/frigidaire/frigidaire_bench.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_tiers_counts_caps_and_n_ranges(B):
    assert [len(B.roster(t)) for t in B.TIERS] == [7, 15, 23]
    assert {k for _, k in B.roster("hard")} == {"plate", "bowl", "cup"}
    for tier, (lo, hi), slack in (("easy", (2, 5), 3), ("medium", (4, 11), 2), ("hard", (6, 17), 1)):   # medium/hard +1 since 2026-09-28
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


def test_every_plate_gap_stays_in_the_families(B):
    """The every-second-gap plate rule was reverted (2026-09-29): it left no room for the 7th bowl."""
    assert not hasattr(B, "plate_slot_ok")
    pts = np.array([[-.07, 0., 0.], [.07, 0., 0.]])
    plate = lambda slot: {"kind": "plate", "slot": slot, "position": [0., 0., 0.], "quaternion_xyzw": [0., 0., 0., 1.]}
    assert B.family_ok(plate("lower_front_03"), pts) and B.family_ok(plate("lower_front_04"), pts)

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
    assert B.rel(B.OUT / "baseline") == "data/results/benchmark/frigidaire_hotec/baseline"       # symlinked root stays relative
    assert B.rel(B.ROOT / "code/planner/frigidaire") == "code/planner/frigidaire"
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


def _episode(out, track, tier, algo, **kw):
    import json
    rec = {"track": track, "tier": tier, "instance": f"{tier}_s0", "algorithm": algo, "solved": True, "abort": None,
           "end_check": {"outcome": "accepted"}, "moves_used": 8, "gap": 1, "S_ref": .3, "failed_settles": 0,
           "planning_time_total_s": .1, "planning_cpu_s": .1, **kw}
    p = out / "episodes" / track / tier / f"{tier}_s0__{algo}.json"
    p.parent.mkdir(parents=True, exist_ok=True)
    p.write_text(json.dumps(rec))
    return p


def test_episode_rows_have_no_copies_and_open_success_needs_a_pooling_free_load(B, tmp_path):
    import json
    _episode(tmp_path, "goal", "easy", "greedy_offline")
    _episode(tmp_path, "goal", "easy", "rrt_connect", solved=False, abort="give-up")
    _episode(tmp_path, "goal", "easy", "planner")                                   # retired row: ignored
    _episode(tmp_path, "open", "easy", "planner", solved=False, abort="give-up", gap=None)   # retired 2026-09-29: ignored
    p = _episode(tmp_path, "open", "easy", "mcts", solved=False, abort="give-up", gap=None)
    p.with_suffix(".analysis.json").write_text(json.dumps({"S_final": .26, "feasible_final": False}))
    q = _episode(tmp_path, "open", "easy", "baseline", solved=False, abort="give-up", gap=None,
                 end_check={"outcome": "not_all_racked"})
    q.with_suffix(".analysis.json").write_text(json.dumps({"S_final": .35, "feasible_final": True}))
    rows = B.episode_rows(tmp_path)
    assert [r["algorithm"] for r in rows["goal"]] == ["greedy_offline", "rrt_connect"]
    assert [(r["algorithm"], r["success"], r["failure"]) for r in rows["open"]] == \
        [("baseline", False, "not_all_racked"), ("mcts", False, "pooling")]
    assert [(r["success"], r["failure"]) for r in rows["goal"]] == [(True, None), (False, "give-up")]
    s = B.summarize(rows["open"])
    assert s["solved"] == 0 and s["S_final"] is None and s["aborts"] == {"not_all_racked": 1, "pooling": 1}
    ok = [r for r in rows["open"] if r["algorithm"] == "mcts"][0]
    ok["feasible_final"], ok["failure"], ok["success"] = True, None, True          # the same load, no puddle
    assert B.summarize([ok])["S_over_S_ref"] == pytest.approx(.26 / .3)


def test_replay_marks_its_load_certified_only_when_the_plan_is(B):
    class World:
        def __init__(self):
            self.marked = None
        def in_counter(self, T):
            return T[2, 3] > .9
        def mark_goals(self, targets, centroids=None, certified=True):
            self.marked = (sorted(targets), certified)
    hi, lo = np.eye(4), np.eye(4)
    hi[2, 3], lo[2, 3] = .95, .3
    moves = [{"item_id": "bowl_01", "T_base_obj": hi.tolist()}, {"item_id": "bowl_01", "T_base_obj": lo.tolist()},
             {"item_id": "bowl_02", "T_base_obj": lo.tolist()}]
    w = World()
    B.Replay({"moves": moves, "certified": True}).reset(None, w)
    assert w.marked == (["bowl_01", "bowl_02"], True)                  # the parked pose is not a goal
    w = World()
    B.Replay({"moves": moves}).reset(None, w)
    assert w.marked == (["bowl_01", "bowl_02"], False)


def test_failure_of_reports_the_uncertified_load_and_pooling(B):
    base = {"track": "open", "solved": False, "end_check": {"outcome": "accepted"}}
    assert B.failure_of({**base, "abort": "load-not-buildable", "end_check": {"outcome": "aborted"}}) == "load-not-buildable"
    assert B.failure_of({**base, "abort": "give-up", "feasible_final": False}) == "pooling"
    assert B.failure_of({**base, "abort": "give-up", "feasible_final": True}) is None
    assert B.failure_of({"track": "goal", "solved": False, "abort": "give-up", "end_check": {"outcome": "accepted"}}) == "give-up"
    assert B.failure_of({"track": "goal", "solved": True, "abort": None, "end_check": {"outcome": "closure_failure"}}) == "closure_failure"


def test_neighbour_order_keeps_the_build_order_between_close_dishes_only(B):
    poses = {"a": {"position_m": [0., 0., .1]}, "b": {"position_m": [.05, 0., .1]}, "c": {"position_m": [.5, 0., .1]},
             "d": {"position_m": [.06, 0., .6]}}
    racks = {"a": "LowerRack", "b": "LowerRack", "c": "LowerRack", "d": "UpperRack"}
    assert B.neighbour_order(["b", "a", "c", "d"], poses, racks) == [("b", "a")]


def test_first_fit_re_plans_shuffle_the_family_order_reproducibly(B, monkeypatch):
    """Attempt 0 packs in family order; a re-plan (rng given) takes a seeded random order that differs from the
    deterministic pack and repeats for the same seed (2026-09-28: deterministic re-plans never became buildable)."""
    fam = {"cup": [({"kind": "cup", "rack": "UpperRack", "slot": f"s{i}", "variant": "v", "position": [i * .1, 0., 0.],
                     "quaternion_xyzw": [0., 0., 0., 1.]}, None) for i in range(8)]}
    masks = {"cup": np.ones(8, bool)}
    monkeypatch.setattr(B, "DishSet", lambda parts, points: _StubDishes())
    ids = [(f"cup_{i:02d}", "cup") for i in range(3)]
    slots = lambda entries: [e["slot"] for e in entries]
    plain = slots(B.first_fit(ids, fam, masks, None, None))
    assert plain == ["s0", "s1", "s2"]
    shuffled = slots(B.first_fit(ids, fam, masks, None, None, rng=np.random.default_rng(7)))
    again = slots(B.first_fit(ids, fam, masks, None, None, rng=np.random.default_rng(7)))
    assert shuffled != plain and shuffled == again and len(set(shuffled)) == 3
    assert B.PLAN_ATTEMPTS == 6


def test_own_order_keeps_a_legal_plan_and_rejects_an_illegal_one(B):
    """The MCTS replay order is kept only when every re-targeted move is legal and the load ends complete."""
    class World:
        def __init__(self):
            self._poses, self.classes, self.blocked = {"a": np.eye(4), "b": np.eye(4)}, {"a": "bowl", "b": "bowl"}, set()
        def snapshot(self):
            return {k: v.copy() for k, v in self._poses.items()}
        def sync(self, poses, classes):
            self._poses.update({k: np.asarray(v, dtype=float).copy() for k, v in poses.items()})
        def in_counter(self, T):
            return float(np.asarray(T)[2, 3]) > 5.
        def move_collides(self, oid, T):
            return oid in self.blocked
    w = World()
    ta, tb = np.eye(4), np.eye(4)
    ta[0, 3], tb[0, 3] = 1., 2.
    planned = [{"item_id": "a", "T_base_obj": (ta + .01).tolist()}, {"item_id": "b", "T_base_obj": tb.tolist()}]
    moves = B.own_order(w, planned, {"a": ta, "b": tb}, cap=1)
    assert [m.item_id for m in moves] == ["a", "b"] and np.allclose(moves[0].T_base_obj, ta)   # re-targeted to the build
    assert np.allclose(w._poses["a"], np.eye(4))                                              # world restored
    w.blocked = {"b"}
    assert B.own_order(w, planned, {"a": ta, "b": tb}, cap=1) is None
    w.blocked = set()
    assert B.own_order(w, planned[:1], {"a": ta, "b": tb}, cap=1) is None                    # load incomplete


def test_open_track_is_first_fit_and_mcts(B):
    assert B.TRACK_ALGORITHMS["open"] == ("baseline", "mcts")
