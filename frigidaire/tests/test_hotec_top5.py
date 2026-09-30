"""Kit-free checks of the top-5 HOTEC arrangement search (frigidaire/scripts/evaluation/frigidaire_hotec_top5.py)."""
from __future__ import annotations

from collections import Counter
import importlib.util
import json
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[2]


@pytest.fixture(scope="module")
def T():
    spec = importlib.util.spec_from_file_location("hotec_top5", ROOT / "frigidaire/scripts/evaluation/frigidaire_hotec_top5.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def entry(oid, kind, slot, x=0., y=0., rack="LowerRack"):
    return {"id": oid, "kind": kind, "rack": rack, "slot": slot, "variant": "v", "position": [x, y, 0.],
            "quaternion_xyzw": [0., 0., 0., 1.]}


def test_roster_has_all_24_dishes_and_both_placement_orders(T):
    assert len(T.ROSTER) == 24 and Counter(k for _, k in T.ROSTER) == {"plate": 8, "bowl": 8, "cup": 8}
    for order, first in (("bowls_first", "bowl"), ("plates_first", "plate")):
        ids = T.order_ids(order)
        assert sorted(ids) == sorted(T.ROSTER)
        assert [k for _, k in ids[:8]] == [first] * 8


def test_distance_ignores_which_same_kind_dish_sits_in_a_slot(T):
    a = [entry("plate_01", "plate", "lower_front_00"), entry("plate_02", "plate", "lower_front_02")]
    b = [entry("plate_01", "plate", "lower_front_02"), entry("plate_02", "plate", "lower_front_00")]
    assert T.distance(a, b) == 0
    c = [entry("plate_01", "plate", "lower_front_00"), entry("plate_02", "plate", "lower_front_04")]
    assert T.distance(a, c) == 1
    lean = [dict(a[0], variant="lean-28"), a[1]]                       # a different lean is the same slot
    assert T.distance(a, lean) == 0


def test_lower_bowls_match_by_position_and_upper_bowls_by_gap(T):
    a = [entry("bowl_01", "bowl", "lower_rear", x=.100)]
    assert T.distance(a, [entry("bowl_02", "bowl", "lower_rear", x=.104)]) == 0
    assert T.distance(a, [entry("bowl_01", "bowl", "lower_rear", x=.160)]) == 1
    assert T.distance(a, [entry("bowl_01", "bowl", "lower_rearright", x=.100)]) == 0      # the same pose, alias zone
    assert T.distance(a, [entry("bowl_01", "bowl", "upper_bowl_00", x=.100, rack="UpperRack")]) == 1
    up = [entry("bowl_01", "bowl", "upper_bowl_07", y=.052, rack="UpperRack")]
    assert T.distance(up, [entry("bowl_03", "bowl", "upper_bowl_07", y=.082, rack="UpperRack")]) == 0
    two = [entry("bowl_01", "bowl", "lower_rear", x=.000), entry("bowl_02", "bowl", "lower_rear", x=.090)]
    assert T.distance(two, [entry("bowl_05", "bowl", "lower_rear", x=.010), entry("bowl_06", "bowl", "lower_mid", x=.200)]) == 1


def test_settled_pooling_or_a_loose_dish_disqualifies_an_accepted_gate(T):
    ok = {"feasible_settled": True, "racked": 24}
    base = {"start": 0, "S": .2, "entries": [], "feasible": True, "gate": "accepted"}
    assert T.accepted([dict(base, settled=ok)])
    assert not T.accepted([dict(base, settled=dict(ok, feasible_settled=False))])
    assert not T.accepted([dict(base, settled=dict(ok, racked=23))])
    assert not T.accepted([dict(base, settled=None)]) and not T.accepted([dict(base, gate="penetration_failure", settled=ok)])


def test_select_top_keeps_the_best_pairwise_distinct_loads(T):
    base = [entry(f"plate_{i:02d}", "plate", f"lower_front_{i:02d}") for i in range(1, 9)]

    def moved(n):
        es = [dict(e) for e in base]
        for i in range(n):
            es[i] = dict(es[i], slot=f"lower_rear_{i:02d}")
        return es
    cands = [{"start": 0, "S": .30, "entries": base}, {"start": 1, "S": .29, "entries": moved(2)},
             {"start": 2, "S": .28, "entries": moved(6)}, {"start": 3, "S": .27, "entries": moved(8)}]
    chosen = T.select_top(cands, top=5, dmin=6)
    assert [c["start"] for c in chosen] == [0, 2]      # 1 differs from 0 in 2 dishes; 3 differs from 2 in 2 dishes
    assert [c["start"] for c in T.select_top(cands, top=1, dmin=6)] == [0]


def test_level_filters_allow_every_plate_gap_and_the_mid_bowl_only_from_level_1(T):
    pts = np.array([[-.07, 0., 0.], [.07, 0., 0.]])
    bowl = lambda slot, x=0.: {"kind": "bowl", "slot": slot, "position": [x, 0., 0.], "quaternion_xyzw": [0., 0., 0., 1.]}
    assert not T.family_ok(bowl("lower_mid"), pts, 0)
    assert T.family_ok(bowl("lower_mid"), pts, 1) and T.family_ok(bowl("lower_mid"), pts, 2)
    assert T.family_ok(bowl("lower_frontright", .165), pts, 1)
    assert not T.family_ok(bowl("lower_frontright", .170), pts, 2)          # settles past the tub wall (benchmark)
    assert not T.family_ok(bowl("lower_rear", .208), pts, 0)                # lip past x .277
    plate = {"kind": "plate", "slot": "lower_front_03", "position": [0., 0., 0.], "quaternion_xyzw": [0., 0., 0., 1.]}
    assert T.family_ok(plate, pts, 0)                                       # odd gaps too (no even-gap rule)


def test_level_2_starts_put_every_plate_in_the_front_bank(T):
    fam = {"plate": [({"slot": "lower_front_00"}, None), ({"slot": "lower_rear_00"}, None), ({"slot": "lower_front_05"}, None)],
           "bowl": [({"slot": "lower_rear"}, None)], "cup": []}
    masks = {"plate": np.array([True, True, False]), "bowl": np.array([True]), "cup": np.zeros(0, bool)}
    f1, m1 = T.start_families(fam, masks, 1)
    assert f1 is fam and m1 is masks
    f2, m2 = T.start_families(fam, masks, 2)
    assert [c["slot"] for c, _ in f2["plate"]] == ["lower_front_00", "lower_front_05"]
    assert m2["plate"].tolist() == [True, False] and f2["bowl"] == fam["bowl"]


def test_only_joint_gate_penetration_bans_are_loaded(T, tmp_path):
    def write(rel, data):
        p = tmp_path / rel
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_text(json.dumps(data))
    write("instances/attempts/hard_s0_a0/bans.json", {"outcome": "penetration_failure", "pairs": [["a", "b"]]})
    write("instances/attempts/hard_s0_a1/bans.json", {"outcome": "sequence", "pairs": [["c", "d"]]})
    write("plans/open/medium/m__baseline.a0.bans.json", {"outcome": "sequence_plan", "gate": "penetration_failure", "pairs": [["e"]]})
    write("plans/open/medium/m__baseline.a1.bans.json", {"outcome": "sequence_plan", "gate": "accepted", "pairs": [["f", "g"]]})
    assert T.gate_bans(tmp_path) == {frozenset(("a", "b")), frozenset(("e",))}


def test_seeds_are_stable_and_distinct(T):
    assert T.seed_of("start", 0, 3, 1) == T.seed_of("start", 0, 3, 1)
    assert len({T.seed_of("ascent", 0, k) for k in range(40)}) == 40


def test_layout_carries_fresh_hashes_and_all_24_objects(T):
    from dishsim_frigidaire.loading import geometry_hashes
    from dishsim_frigidaire.paths import ASSET_DIR
    if not (ASSET_DIR / "fdpc4221as.usdc").is_file():
        pytest.skip("no Frigidaire build")
    entries = [entry(oid, kind, f"slot_{oid}") for oid, kind in T.ROSTER]
    layout = T.layout_of(entries)
    assert layout["appliance_sha256"] == geometry_hashes(ASSET_DIR)
    assert layout["counts"]["total"] == 24 and layout["counts"]["fitted"] == {"plate": 8, "bowl": 8, "cup": 8}
    assert {o["color"] for o in layout["objects"]} <= set(T.COLOURS)
    assert all("rack_local_pose" in o and o["usd"] == f"{o['kind']}.usda" for o in layout["objects"])
    json.dumps(layout)


def test_front_plates_order_places_plates_first_and_nesting_is_scored_not_forbidden(T):
    ids = T.order_ids("front_plates")
    assert [k for _, k in ids[:8]] == ["plate"] * 8 and sorted(ids) == sorted(T.ROSTER)
    assert set(T.ORDERS) == {"front_plates", "bowls_first", "plates_first"}
    assert T.B.nests(None, None, None, [{"kind": "bowl"}]) is False     # the module copy only; reported via nested_pairs
    assert T.B.DishSet is T.TolerantDishSet and 0 < T.DISH_TOL_M < .001 and T.APPLIANCE_TOL_M == 0   # gate preflight: 1 mm
    assert T.OUT.name.endswith("_" + T.VARIANT) and T.OUT != T.STRICT_OUT

