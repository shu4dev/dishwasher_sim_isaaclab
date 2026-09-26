# Copyright (c) 2026, dishsim project.
# SPDX-License-Identifier: BSD-3-Clause
"""Arrangement planner: counter band, support gate, sequencer, keep-in-place, driver integration (Kit-free)."""
import json
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "frigidaire" / "src"))
sys.path.insert(0, str(ROOT / "src"))

from dishsim import rearrange  # noqa: E402
from dishsim.transforms import make_T  # noqa: E402
from dishsim_frigidaire import planner as P  # noqa: E402

TOP = P.COUNTER["top_z_m"]
STATE = ROOT / "results/initial_states/frigidaire/packing_20260911_seed20260911/states/random_00.json"


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


@pytest.fixture(scope="module")
def pool():
    # The cached pool (POOL_RUN/candidates_screened.json) was screened on the v3 geometry
    # (52/72-tine racks, photo-fitted rims, earlier basket seat). It records the usdc hashes and
    # the settled body positions it was screened against; the pool tests run only while those
    # still describe the live build, and skip as stale until the organized candidates are
    # regenerated and re-screened on the tape-measured racks.
    pytest.importorskip("fcl")
    pytest.importorskip("pxr")
    if not (P.ASSET_DIR / "fdpc4221as.usdc").exists() or not (P.POOL_RUN / "candidates_screened.json").exists():
        pytest.skip("Frigidaire assets or the organized candidate pool are not present")
    stale = pool_staleness(json.loads((P.POOL_RUN / "candidates_screened.json").read_text()))
    if stale:
        pytest.skip("organized candidate pool is stale: " + "; ".join(stale))
    return P.build_geometric_pool()


def pool_staleness(catalog):
    """Reasons the cached pool no longer describes the live appliance (empty when it still does)."""
    import hashlib
    reasons = []
    for name, recorded in catalog.get("asset_sha256", {}).items():
        path = P.ASSET_DIR / name
        if not path.exists():
            reasons.append(f"{name} missing")
        elif hashlib.sha256(path.read_bytes()).hexdigest() != recorded:
            reasons.append(f"{name} rebuilt since the pool was screened")
    # The screening scene settles with both racks pulled out, so only the basket's seat in the
    # lower rack (its offset from the settled LowerRack body) is comparable with the source.
    bodies = catalog.get("baseline_components", {})
    if "SilverwareBasket" in bodies and "LowerRack" in bodies:
        recorded = np.asarray(bodies["SilverwareBasket"]["position_m"]) - np.asarray(bodies["LowerRack"]["position_m"])
        authored = np.asarray(P.E.BODY_POSITIONS["SilverwareBasket"]) - np.asarray(P.E.BODY_POSITIONS["LowerRack"])
        if np.linalg.norm(recorded - authored) > .005:
            reasons.append("basket seat moved (pool %s vs source %s m in the lower rack)"
                           % (np.round(recorded, 4).tolist(), np.round(authored, 4).tolist()))
    return reasons


def test_geometric_pool_drops_only_opening_edges(pool):
    assert set(pool["conflict_reasons"]) <= {"separation", "nesting", "same_slot", "overlap"}
    assert {c["kind"] for c in pool["candidates"]} == set(P.KINDS)
    dish = [i for i, c in enumerate(pool["candidates"]) if c["kind"] in ("bowl", "mug")]
    assert 0 < sum(a in dish and b in dish for a, b in pool["conflict_pairs"]) <= pool["stored_pairs"]
    assert pool["cross_kind_overlaps"] == 0


def test_plate_pool_entries_drain_and_share_trial_slots(pool):
    plates = [(i, c) for i, c in enumerate(pool["candidates"]) if c["kind"] == "dinner_plate"]
    assert 0 < len(plates) <= pool["plates"]["allowed"] and pool["plates"]["kept"] == len(plates)
    for _, c in plates:
        assert c["source"] == "packing" and c["row_metadata"]["row_id"] == f"plate_{c['rack']}"
        assert not P.E.pools("dinner_plate", *P.racks_in_pose(c["rack"], c["rack_local_pose"]).values())
    edges = {tuple(e) for e in pool["conflict_pairs"]}
    by_trial = {}
    for i, c in plates:
        by_trial.setdefault(c["slot_id"], []).append(i)
    twins = [v for v in by_trial.values() if len(v) > 1]
    assert twins and all((v[0], v[1]) in edges for v in twins)


def test_cutlery_pool_slots_and_edges(pool):
    edges = {tuple(e) for e in pool["conflict_pairs"]}
    cut = [(i, c) for i, c in enumerate(pool["candidates"]) if c["kind"] in P.CUTLERY_KINDS]
    counts = {}
    for _, c in cut:
        counts[c["kind"]] = counts.get(c["kind"], 0) + 1
        assert c["rack"] == "SilverwareBasket" and c["slot_id"].startswith("cutlery_")
    assert counts == {k: 9 * P.CUTLERY_PER_SLOT for k in P.CUTLERY_KINDS}
    for a in range(len(cut)):
        for b in range(a + 1, len(cut)):
            if cut[a][1]["slot_id"] == cut[b][1]["slot_id"]:
                assert (cut[a][0], cut[b][0]) in edges
    dish = {i for i, c in enumerate(pool["candidates"]) if c["kind"] not in P.CUTLERY_KINDS}
    cutset = {i for i, _ in cut}
    assert not any((a in dish) != (b in dish) for a, b in edges if a in dish | cutset and b in dish | cutset)


def test_search_proposals_extends_with_cutlery(pool):
    inventory = {"bowl": 1, "dinner_plate": 1, "fork": 1, "knife": 1}
    adjacency = P._adjacency(len(pool["candidates"]), pool["conflict_pairs"])
    proposals = P.search_proposals(pool, inventory, deadline=1e18, seed=1, random_attempts=5, milp_rounds=1)
    assert proposals
    for _, chosen in proposals:
        kinds = [pool["candidates"][i]["kind"] for i in chosen]
        assert sorted(kinds) == sorted(inventory)
        assert not any(b in adjacency[a] for a in chosen for b in chosen)


def test_racks_in_pose_basket_requires_the_settled_frame():
    local = {"position_m": [0., 0., .1], "quaternion_xyzw": [0., 0., 0., 1.]}
    with pytest.raises(ValueError):
        P.racks_in_pose("SilverwareBasket", local)
    if STATE.exists():
        snap = json.loads(STATE.read_text())["initial_snapshot"]
        basket = P.basket_in(snap)
        posed = P.racks_in_pose("SilverwareBasket", local, basket)
        assert np.allclose(posed["position_m"], basket[0] + [0., 0., .1], atol=1e-3)   # the settled basket is tilted 0.14 deg


def test_allowed_kinds_do_not_widen_racks():
    from dishsim_frigidaire import random_poses as R
    from dishsim_frigidaire.initial_state_runtime import normalize_candidates
    assert R.RACKS == ("LowerRack", "UpperRack") and "SilverwareBasket" in R.OBJECT_RACKS and "fork" in R.OBJECT_KINDS
    entry = {"kind": "fork", "rack": "SilverwareBasket", "object_id": "fork_a",
             "rack_local_pose": {"position_m": [0., 0., .1], "quaternion_xyzw": [0., 0., 0., 1.]}}
    assert normalize_candidates([entry])[0]["object_id"] == "fork_a"
    with pytest.raises(ValueError):
        normalize_candidates([dict(entry, rack="Door")])


def test_inventories_sum_over_the_seven_kinds():
    for n in (12, 24):
        assert sum(P.INVENTORY[n].values()) == n and set(P.INVENTORY[n]) == set(P.KINDS)


def test_keep_candidates_skip_supported_and_pooling(pool):
    if not STATE.exists():
        pytest.skip("packing state not present")
    inst = P.instance_from_state(STATE, cap=4)
    world = P.PlannerWorld(inst)
    merged = P.merged_pool(pool, inst, world.geometry)
    keeps = [c for c in merged["candidates"] if c.get("keep_for")]
    pooling = [o["object_id"] for o in inst["objects"]
               if P.E.pools(o["kind"], *P.racks_in_pose(o["rack"], o["rack_local_pose"], P.basket_in(inst["initial_snapshot"])).values())]
    assert not (set(k["keep_for"] for k in keeps) & set(pooling))
    inst["support"] = [[inst["objects"][0]["object_id"], inst["objects"][1]["object_id"]]]
    merged2 = P.merged_pool(pool, inst, world.geometry)
    assert inst["objects"][1]["object_id"] not in {c.get("keep_for") for c in merged2["candidates"]}


def test_racks_in_composition_matches_load_state():
    if not STATE.exists():
        pytest.skip("packing state not present")
    state = json.loads(STATE.read_text())
    arr = P.E.load_state(STATE)
    for src, obj in zip(state["objects"], arr.objects):
        posed = P.racks_in_pose(src["rack"], src["rack_local_pose"])
        assert np.allclose(posed["position_m"], obj["position_m"], atol=1e-9)
    assert np.allclose(P.basket_in(state["initial_snapshot"])[0], arr.basket[0], atol=1e-9)


def test_first_fit_is_deterministic(pool):
    if not STATE.exists():
        pytest.skip("packing state not present")
    inst = P.instance_from_state(STATE, cap=4)
    world = P.PlannerWorld(inst)
    goals = []
    for _ in range(2):
        rinst = P.to_rearrange_instance(inst)
        algo = P.FirstFitBaseline(seed=3, budget_s=20., pool=pool)
        algo.reset(rinst, world)
        world.sync({it["item_id"]: it["T_base_init"] for it in rinst.items}, {it["item_id"]: it["object_class"] for it in rinst.items})
        _, scored = algo.candidates(deadline=1e18)
        goals.append(scored[0][3])
    assert goals[0] == goals[1] and goals[0] == sorted(goals[0])
