#!/usr/bin/env python3
"""Generate one planner instance: a settled messy counter pile plus unorganized racks (one Kit process).

    scripts/run_kit.sh frigidaire/scripts/experiment/frigidaire_planner_instances.py \
        --seed 0 --n-objects 9 --attempt 0 --out-dir results/planner/frigidaire/instances --headless --device cpu

Inside set: a random feasible subset of the packing run's candidate catalog (bowls and mugs,
the FCL-screened settled single-dish poses the packing states were built from). Counter set:
random poses (uniform rotation) in a box above a worktop slab, FCL-screened against the slab
and each other, released together with the inside set in ONE Isaac session so piles and
stacking arise physically. The backend's rest windows, rack retraction and containment gates
apply to the in-rack objects; counter objects only have to come to rest on the slab.
Writes <out-dir>/s<seed>_n<n>.json (a planner instance, not a state file) and
<out-dir>/attempts/s<seed>_n<n>_a<attempt>/physics.json; prints [RESULT] PASS | REROLL <reason>.
References and the sequenceability certificate are added Kit-free by
frigidaire_planner_run.py --certify.
"""
from __future__ import annotations

import argparse
from collections import Counter
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import sys
import time
import traceback

ROOT = Path(__file__).resolve().parents[3]
PILE_BOX_XY_M = (.16, .10)              # half extents of the pile's sampling box for FOUR objects; scales with sqrt(count/4)
PILE_DROP_M = (.015, .04)               # sampled bbox-centre height above the slab top before lifting
PILE_LIFT_STEP_M, PILE_LIFT_MAX_M = .01, .06   # a pose that overlaps what is already there is lifted until it clears: low
                                         # stacks only (objects lifted higher land hard enough to trip the 2 mm peak-penetration gate)
PILE_TRIES = 1500                        # form from centimetre drops instead of decimetre ones (the 2 mm peak-penetration gate)
SLAB_MARGIN_M = .05


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def generate(args, report):
    import numpy as np
    from dishsim_frigidaire import planner as P
    from dishsim_frigidaire.initial_state_candidates import InitialCollisionChecker
    from dishsim_frigidaire.initial_state_runtime import IsaacInitialStateBackend
    from dishsim_frigidaire.loading import visual_points
    from dishsim_frigidaire.random_poses import sample_pose, source_geometry_domains

    rng = np.random.default_rng(1000 * args.seed + args.attempt)
    inventory = dict(P.INVENTORY[args.n_objects])
    cutlery_start = "basket_only" if args.cutlery_start == "basket" or (args.cutlery_start == "auto" and args.attempt >= 6) else "pile_or_basket"
    kinds = [k for k, n in inventory.items() for _ in range(n)]
    drawable = [i for i, k in enumerate(kinds) if cutlery_start == "pile_or_basket" or k in P.DISH_KINDS]
    order = rng.permutation(len(drawable))
    counter_kinds = [kinds[drawable[i]] for i in order[:int(round(args.counter_share * args.n_objects))]]
    inside_inventory = Counter(kinds) - Counter(counter_kinds)
    report.update(inventory=inventory, counter_kinds=counter_kinds, inside_inventory=dict(inside_inventory),
                  cutlery_start=cutlery_start)
    checker = InitialCollisionChecker(args.usd.parent)

    # inside dishes: random feasible subset of the packing catalog
    catalog = json.loads((args.packing_dir / "candidates.json").read_text())
    graph = json.loads((args.packing_dir / "compatibility.json").read_text())
    allowed = [i for i in graph["allowed_indices"] if catalog["candidates"][i]["kind"] in P.DISH_KINDS]
    adjacency = P._adjacency(len(catalog["candidates"]), graph["conflict_pairs"])
    inside_dishes = {k: v for k, v in inside_inventory.items() if k in P.DISH_KINDS}
    chosen = P.random_greedy(catalog, allowed, adjacency, inside_dishes, rng, attempts=200) if inside_dishes else []
    if chosen is None:
        raise RuntimeError("no feasible inside subset in the packing catalog")
    entries = []
    for i in chosen:
        c = catalog["candidates"][i]
        if "rack_local_pose" not in c:
            raise RuntimeError(f"packing candidate {i} carries no rack_local_pose")
        entries.append({"kind": c["kind"], "rack": c["rack"], "object_id": c["candidate_id"], "candidate_id": c["candidate_id"],
                        "rack_local_pose": c["rack_local_pose"], "source": {"packing_candidate_index": int(i)}})
    # inside cutlery: a random compatible subset of the frozen basket patterns (same slot + 1 mm FCL edges), released
    # from a few mm above its pattern pose in the basket frame (the full-load evidence path's release hover)
    inside_cutlery = {k: v for k, v in inside_inventory.items() if k in P.CUTLERY_KINDS}
    if inside_cutlery:
        ccands, cedges, _, _ = P.cutlery_pool(checker)
        cpool = {"candidates": ccands, "conflict_pairs": cedges}
        cchosen = P.random_greedy(cpool, list(range(len(ccands))), P._adjacency(len(ccands), cedges), inside_cutlery, rng, attempts=200)
        if cchosen is None:
            raise RuntimeError("no compatible inside cutlery subset")
        for i in cchosen:
            c = ccands[i]
            local = {"position_m": list(c["rack_local_pose"]["position_m"]), "quaternion_xyzw": list(c["rack_local_pose"]["quaternion_xyzw"])}
            local["position_m"][2] += .001                        # pool poses already sit 1 mm above contact (settle_down)
            entries.append({"kind": c["kind"], "rack": "SilverwareBasket", "object_id": c["candidate_id"], "candidate_id": c["candidate_id"],
                            "rack_local_pose": local, "source": {"cutlery_slot": c["slot_id"], "pattern_index": c["source_index"]}})

    # counter set: random poses above the slab, FCL-screened against the slab and each other
    slab = P.slab_body(checker)
    vertices = {kind: visual_points(args.usd.parent / "tableware" / f"{kind}.usdc") for kind in set(counter_kinds)}
    top = P.COUNTER["top_z_m"]
    scale = max(1., 1.8 * (len(counter_kinds) / 4.) ** .5)    # big piles spread over the whole slab: fewer lifts, softer landings
    hx, hy = min(.5, PILE_BOX_XY_M[0] * scale), min(.25, PILE_BOX_XY_M[1] * scale)
    box = {"lower_m": [-hx, -hy, top + PILE_DROP_M[0]], "upper_m": [hx, hy, top + PILE_DROP_M[1]]}
    counter_entries, bodies = [], []
    def clear(body):
        return checker.pair(body, slab)["valid"] and all(checker.pair(body, other)["valid"] for other in bodies)

    for k, kind in enumerate(counter_kinds):
        for _ in range(PILE_TRIES):
            sampled = sample_pose(rng, vertices[kind], box)
            pose, lifted = {"position_m": list(sampled["position_m"]), "quaternion_xyzw": sampled["quaternion_xyzw"]}, 0.
            body = checker._body(kind, pose, f"counter_{kind}_{k}")
            while not clear(body) and lifted < PILE_LIFT_MAX_M:            # lift over whatever it overlaps
                lifted += PILE_LIFT_STEP_M
                pose["position_m"][2] += PILE_LIFT_STEP_M
                body = checker._body(kind, pose, f"counter_{kind}_{k}")
            if clear(body):
                bodies.append(body)
                counter_entries.append({"kind": kind, "rack": "LowerRack", "object_id": f"counter_{kind}_{k}",
                                        "pose_world": pose, "start": "Counter",
                                        "source": {"sampled_pose": sampled, "lifted_m": round(lifted, 3)}})
                break
        else:
            raise RuntimeError(f"could not place counter object {k} ({kind}) without overlap")
    counter_ids = [e["object_id"] for e in counter_entries]

    class CounterBackend(IsaacInitialStateBackend):
        """Counter objects ride along: no rack support required, allowed outside the tub, contacts recorded."""

        def __init__(self, *a, **kw):
            super().__init__(*a, **kw)
            self.assignments = {k: v for k, v in self.assignments.items() if k not in set(counter_ids)}

        def snapshot(self):
            s = super().snapshot()
            s["contact_pairs"] = sorted(list(p) for p in self.latest_contact["pairs"])
            return s

    baseline = json.loads((args.packing_dir / "baseline/result.json").read_text())["snapshot"]
    slab_spec = ("/World/Counter", P.COUNTER["size_m"], P.COUNTER["center_m"])
    backend = CounterBackend(args.usd, args.attempt_dir, device=args.device, domains=source_geometry_domains(),
                             deadline=args.started + args.max_wall_seconds, app=args.app,
                             candidates=entries + counter_entries, extra_statics=[slab_spec])
    outcomes = []
    for order in (("UpperRack", "LowerRack"), ("LowerRack", "UpperRack")):   # the rack-speed gate flakes; the other order usually passes
        record = backend.evaluate(order=order, baseline=baseline)
        outcomes.append({"order": list(order), "outcome": record["outcome"], "reason": record.get("reason")})
        if record["outcome"] != "closure_failure":
            break
    report["orders_tried"] = outcomes
    outcome = record["outcome"]
    contained = record.get("final_containment", {})
    if (outcome == "outside_dishwasher" and contained
            and all(contained[e["object_id"]]["contained"] for e in entries)
            and record.get("maximum_cycle_penetration_m", 1.) < .002):
        outcome = "accepted"
        record["outcome_reclassified"] = "in-rack objects contained; counter objects are outside the tub by design"
    report.update(outcome=outcome, backend_reason=record.get("reason"), wall_seconds_physics=record.get("wall_seconds"))
    if outcome != "accepted":
        report.update(result="REROLL", reason=f"backend {outcome}: {record.get('reason')}")
        return
    snap = record["initial_snapshot"]
    hx, hy = P.COUNTER["size_m"][0] / 2 - SLAB_MARGIN_M, P.COUNTER["size_m"][1] / 2 - SLAB_MARGIN_M
    for oid in counter_ids:
        x, y, z = snap["poses"][oid]["position_m"]
        if abs(x) > hx or abs(y) > hy or z < top - P.BAND_TOLERANCE_M:
            report.update(result="REROLL", reason=f"{oid} left the slab: {np.round([x, y, z], 3).tolist()}")
            return

    objects = []
    for e in entries:
        objects.append({"object_id": e["object_id"], "kind": e["kind"], "start": e["rack"], "rack": e["rack"],
                        "rack_local_pose": P.local_from_world(snap["poses"][e["rack"]], snap["poses"][e["object_id"]]),
                        "pose_world": snap["poses"][e["object_id"]], "source": {**e["source"], "candidate_id": e["candidate_id"]}})
    for e in counter_entries:
        objects.append({"object_id": e["object_id"], "kind": e["kind"], "start": "Counter",
                        "pose_world": snap["poses"][e["object_id"]], "source": e["source"]})
    ids = [o["object_id"] for o in objects]
    support = P.support_edges(snap["contact_pairs"], snap["poses"], ids)
    instance = {
        "schema_version": 1, "purpose": "planner_instance", "instance_id": f"s{args.seed}_n{args.n_objects}",
        "seed": args.seed, "attempt": args.attempt, "n_objects": len(objects), "inventory": inventory,
        "counter": {**P.COUNTER, "cap": len(counter_ids), "start_count": len(counter_ids), "share": args.counter_share,
                    "pile_box": box, "buffer_pitch_m": P.BUFFER_PITCH_M},
        "objects": objects,
        "initial_snapshot": {"joints": snap["joints"], "poses": snap["poses"], "step": snap["step"],
                             "contact_pairs": snap["contact_pairs"]},
        "support": support, "baseline": baseline,
        "validation": {"outcome": outcome, "reason": record.get("reason"), "reclassified": record.get("outcome_reclassified"),
                       "settled": {k: record["settled"].get(k) for k in ("passed", "reason", "simulated_seconds")},
                       "loaded_rack_motions": [{"rack": m["rack"], "passed": m.get("passed")} for m in record["loaded_rack_motions"]],
                       "final_containment": {k: v["contained"] for k, v in contained.items()},
                       "maximum_settle_penetration_m": record.get("maximum_settle_penetration_m"),
                       "maximum_cycle_penetration_m": record.get("maximum_cycle_penetration_m"),
                       "wall_seconds": record.get("wall_seconds"), "attempt_dir": str(args.attempt_dir.relative_to(ROOT))},
        "cutlery_start": cutlery_start,
        "hashes": {"packing_catalog": digest(args.packing_dir / "candidates.json"),
                   "cutlery_patterns": digest(P.CUTLERY_PATTERNS),
                   "packing_graph": digest(args.packing_dir / "compatibility.json"),
                   "assets": checker.asset_sha256,
                   "sources": {str(p.relative_to(ROOT)): digest(p) for p in
                               (Path(__file__), ROOT / "frigidaire/src/dishsim_frigidaire/planner.py",
                                ROOT / "frigidaire/src/dishsim_frigidaire/initial_state_runtime.py")}},
        "generated_utc": datetime.now(timezone.utc).isoformat()}
    path = args.out_dir / f"{instance['instance_id']}.json"
    path.write_text(json.dumps(instance, indent=1) + "\n")
    report.update(result="PASS", instance=str(path), support_edges=support, counter_ids=counter_ids)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seed", type=int, required=True)
    parser.add_argument("--n-objects", type=int, choices=(9, 18, 12, 24), required=True)
    parser.add_argument("--counter-share", type=float, default=.5,
                        help="fraction of the inventory that starts on the counter (the cap); 1/3 for 24-object piles")
    parser.add_argument("--cutlery-start", choices=("auto", "basket"), default="auto",
                        help="auto: cutlery may start in the pile (basket-only from attempt 6); basket: cutlery starts inside")
    parser.add_argument("--attempt", type=int, default=0)
    parser.add_argument("--out-dir", type=Path, default=ROOT / "results/planner/frigidaire/instances")
    parser.add_argument("--usd", type=Path, default=ROOT / "build/frigidaire_collection/usd/fdpc4221as.usdc")
    parser.add_argument("--packing-dir", type=Path, default=ROOT / "results/initial_states/frigidaire/packing_20260911_seed20260911")
    parser.add_argument("--max-wall-seconds", type=float, default=300)
    from isaaclab.app import AppLauncher
    AppLauncher.add_app_launcher_args(parser)
    parser.set_defaults(device="cpu", headless=True)
    args = parser.parse_args()
    args.started = time.monotonic()
    args.out_dir.mkdir(parents=True, exist_ok=True)
    args.attempt_dir = args.out_dir / "attempts" / f"s{args.seed}_n{args.n_objects}_a{args.attempt}"
    args.attempt_dir.mkdir(parents=True, exist_ok=True)
    report = {"result": "REROLL", "reason": None, "seed": args.seed, "n_objects": args.n_objects, "attempt": args.attempt,
              "started_utc": datetime.now(timezone.utc).isoformat()}
    app = None
    try:
        app = AppLauncher(args).app
        args.app = app
        sys.path[:0] = [str(ROOT / "src"), str(ROOT / "frigidaire/src")]
        generate(args, report)
    except Exception as exc:
        traceback.print_exc()
        report.update(result="REROLL", reason=f"exception {exc!r}")
    finally:
        report.update(wall_seconds=time.monotonic() - args.started, finished_utc=datetime.now(timezone.utc).isoformat())
        (args.attempt_dir / "generation.json").write_text(json.dumps(report, indent=2, allow_nan=False, default=str) + "\n")
        print(f"[RESULT] {report['result']} {report.get('reason') or report.get('instance', '')}", flush=True)
        if app is not None:
            from dishsim.media import release_sim_for_close
            release_sim_for_close()
            app.close()
    return 0 if report["result"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
