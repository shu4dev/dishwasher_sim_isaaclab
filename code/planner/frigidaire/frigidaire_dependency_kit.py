#!/usr/bin/env python3
# Copyright (c) 2026, dishsim project.
# SPDX-License-Identifier: BSD-3-Clause
"""Isaac dependency graph of one 24-dish load (boot-first; one load per Kit process).

--edges pairs (the default; procedure v20, 2026-10-14, research/2026-10-14-newj-curve-and-leaning-pairs.md; the
first version, v18, wrote pairs/, pairs_more/, pairs_rerun/, pairs_perturbed/ and is in git history): ordered-pair
tests on the goal load alone. Candidate pairs: every dish pair whose collision pieces (frigidaire_bench.bench_checker's
convex parts) come within PAIR_M = 20 mm at the target poses, any rack (FCL distance; overlap recorded as 0 mm).
A test parks its absent set S off-scene (as build_sequence parks), teleports every other dish to its exact target
and re-settles that context 150 ticks; a context dish then outside its at-goal tolerance of its target has fallen:
it is parked too and the context re-settled again (FALL_ROUNDS at most; recorded per test as parked_fallen, with every
dish's lateral / dz / tilt from its target per round). The dropped dish B is released at its LOWEST CLEAR HOVER
(frigidaire_bench.clear_release: its target raised in 1 mm steps along its rack's up axis until the pose passes the
bench checker's 1 mm penetration allowance against every context dish at its re-settled pose and the appliance; past
60 mm "no clear release": not landed, nothing dropped) and settled 150 + 60 ticks: landed = within the at-goal
tolerance of its target and contact depth < 2 mm; disturbed = context dishes that end outside their own at-goal
tolerance of their targets. L(B): S = {B}, for every dish; R(A, B): S = {A, B}, both orientations of every candidate
pair. Each drop stores its margin to the five thresholds (lateral, height, tilt, contact, disturbance = the moved
context dishes' goal distances); a drop within MARGIN = 20 % of any is rerun under 3 perturbed releases (3 mm / 3 deg,
--perturb-* mechanism, seeds 1, 2, 3; 10 S + 1..3 with --perturb-seed S) and the majority is its verdict. Edges
(a before b): need (B, X) for every X fallen in the first re-settle of L(B)'s context; knock (B, X) for every X the
verdict of L(B) disturbed; block (B, A) when L(B)'s verdict did not land and R(A, B)'s did; one edge per (a, b) with
its reasons. A two-way pair with need both ways is mutual support ("place together or re-roll"); any other two-way
pair has its drops (L(a), L(b), R(a, b), R(b, a)) retested under the 3 perturbed releases and, if it stays, is a
cycle. Status: cycle (a cycle pair, or a cycle once each mutual pair is one node; no build); mutual_support (a
diagnostic build_sequence with each mutual pair released together at its targets + one clear hover, the rest one at a
time: not a benchmark move); otherwise build_sequence started from the edges certifies the graph: ok | no_order |
landing | moved, a failed build names the failing dish. Every constraint a build learns beyond the edges joins the
graph with reason "learned" (extra_constraints). Writes <out>/pairs/<load_id>.json (the random sampler: pairs_random/).

--goal built (with --edges pairs, 2026-10-14; the default gate tests the targets as they are): a jointly settled goal
may hold a two-way pair, two dishes that only settle that way together, so it cannot be built one dish at a time.
The targets are first re-settled by build_sequence under the own-load rule (settle deviation <= DEV_MAX_M of the
commanded pose = target + the goal hover, contact depth < 2 mm, no placed dish moved > 10 mm / 20 deg; up to
SEQUENCE_BUILDS builds), its learned set started from the edges of the load's <out>/pairs/<load_id>.json minus its
two-way pairs (and any edge still on a longer cycle), else empty. No build = status 'resettle_failed' (the builds
kept, no tests). Otherwise the built poses become the targets (targets_source 'sequential_build'; goal_delta =
B.goal_distance from the previous target to the built pose per dish, and its max) and everything above runs on them,
the certifying build under the at-goal tolerance of the built poses. Writes <out>/pairs_resettled/<load_id>.json
(pairs_resettled_random/): the pairs record plus goal_source 'sequential_build', goal_delta, resettle_builds,
resettle_initial, resettle_rule and original (the pairs record read, or null); entries_settled = the built goal.
--out-dir PATH (any --edges and --goal): the record and _kit/<load_id>/ go under PATH instead.

--perturb-mm X --perturb-deg Y --perturb-seed S (--edges pairs; default 0 0 0 = unperturbed): the settle is
deterministic (a rerun reproduced every drop bit for bit), so a plain rerun measures nothing. With X or Y > 0 each L
and R drop commands the dropped dish at its release pose (v20: the lowest clear hover found at the unperturbed
target; v18: target + the goal hover), yawed U(-Y, Y) deg about the world Z axis
through its visual centroid, then shifted by a point uniform in the disc of radius X mm in its rack's plane (numpy
default_rng((S, test number)), reproducible); the context dishes, the resettle and the certifying build are never
perturbed, and landing is judged against the unperturbed target (and command) with the same tolerances. Per test
dx_mm, dy_mm (rack axes) and yaw_deg as applied; per record perturbation {lateral_mm, yaw_deg, seed} (zeros when off).

--edges build (2026-10-13): the load is built one dish at a time in the baseline's extended racks by the bench's
sequence gate (frigidaire_bench_kit.build_sequence: lowest centre first, 150 + 60 ticks per dish, up to 8 builds while
it learns the order). Edges (a before b): support (final contacts, the higher dish rests on the lower), learned (from
failed builds) and neighbour (same-rack pairs within 15 cm keep the successful build's order,
frigidaire_bench.neighbour_order). Writes <out>/<sampler>/<load_id>.json.
Waves (both modes) = longest-path layers (sources at 0).
Targets (2026-10-14: r05_00014's packed poses did not build, its shingled plates landed 10-13 mm deep): with --gate
the load has passed the top-5 joint gate (all 24 teleported and settled at once, 5 mm momentary peak, both racks
retracted, containment) and every dish must land within the at-goal tolerance of its JOINTLY SETTLED pose, the
benchmark's goal path (frigidaire_bench_kit.sequence_plan --gate); a load that failed its gate gets status 'gate',
no edges and no waves, written without booting Kit. Without --gate (the fallback) restricted loads are built toward
their commanded packed poses (lifted up to 60 mm) within DEV_MAX_M of the command (the own-load rule), random farm
loads toward their farm-settled poses within the at-goal tolerance.

Per load, from the repo root (<id> e.g. r05_00014):
(a) the joint-gate manifest, Kit-free (isaaclab is never imported; run it through run_py.sh, inside the container,
    since frigidaire_hotec_top5.manifest_of writes the container's absolute tableware paths):
    code/util/run_py.sh code/planner/frigidaire/frigidaire_dependency_kit.py --sampler restricted --load <id> --manifest-out data/results/dot_rules/dependency/restricted/_gate/<id>/manifest.json
(b) the joint gate with frigidaire_hotec_top5.gate_unit's flags; it prints [RESULT] <outcome> (accepted passes):
    code/util/run_kit.sh code/initialization/frigidaire/frigidaire_initial_state_validate.py --manifest data/results/dot_rules/dependency/restricted/_gate/<id>/manifest.json --out-dir data/results/dot_rules/dependency/restricted/_gate/<id> --max-wall-seconds 600 --peak-penetration-m 0.005 --headless --device cpu
    on closure_failure (the rack-speed flake) gate_unit reruns it once lower first: the same command with
    --order lower_first --out-dir data/results/dot_rules/dependency/restricted/_gate/<id>_lower_first, then give
    (c) that directory if it is accepted
(c) the graph (24 + 2 x pairs drops of 150 + 210 ticks, the fallen-context, marginal and two-way reruns, then the
    certifying build: about 35-70 min of Kit per load):
    code/util/run_kit.sh code/planner/frigidaire/frigidaire_dependency_kit.py --sampler restricted --load <id> --gate data/results/dot_rules/dependency/restricted/_gate/<id> --edges pairs --max-wall-seconds 7200 --headless
    (--goal built: the goal re-settled one dish at a time first, 1-8 more builds; --edges build --force: the
    2026-10-13 build-learned graph)

Writes the <load_id>.json above and <folder>/_kit/<load_id>/report.json.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import sys
import time
import traceback

ROOT = Path(__file__).resolve().parents[3]
RACK_FRAMES = ("LowerRack", "UpperRack", "SilverwareBasket")
PAIR_M = .02                      # --edges pairs: candidate pairs' collision pieces within 20 mm at the target poses
REASONS = ("knock", "need", "block", "learned")
MARGIN = .2                       # a drop within 20 % of any of its five thresholds is rerun under 3 perturbed releases
RERUN_MM, RERUN_DEG, RERUN_SEEDS = 3., 3., (1, 2, 3)   # seeds 10 S + 1..3 with --perturb-seed S (1, 2, 3 unperturbed)
MOVED_MM, MOVED_DEG = 1., 1.      # a context dish the drop moved by more: its goal distance enters the disturbance margin
FALL_ROUNDS = 4                   # context re-settles: the fallen are parked and the context re-settled, at most 4 rounds
LANDING_TAIL = ("contact depth < 2 mm; released at the lowest clear hover; no placed dish at its goal ends outside its "
                "at-goal tolerance (a placed dish not at its goal: moved > 10 mm / 20 deg)")


class Refused(Exception):
    """An expected stop (no traceback): a non-accepted farm load, a baseline that does not hold."""


def rel(path):
    p = Path(path).absolute()
    try:
        return str(p.relative_to(ROOT))
    except ValueError:
        return str(p)


def plain_kind(kind):
    return kind[6:] if kind.startswith("hotec_") else kind


def read_load(args):
    """(record path, entries): rack-local entries (racks out = the rack body frame), kinds plate | bowl | cup."""
    if args.sampler == "restricted":
        path = Path(args.restricted_dir) / f"{args.load}.json"
        data = json.loads(path.read_text())
        entries = [{"id": e["id"], "kind": plain_kind(e["kind"]), "rack": e["rack"], "slot": e.get("slot"),
                    "variant": e.get("variant"), "position": [float(v) for v in e["position"]],
                    "quaternion_xyzw": [float(v) for v in e["quaternion_xyzw"]], "keep": bool(e.get("keep", False))}
                   for e in data["entries"]]
    else:
        path = Path(args.farm_dir) / "loads" / args.load / "result.json"
        data = json.loads(path.read_text())
        if data.get("outcome") != "accepted":
            raise Refused(f"farm load outcome {data.get('outcome')!r}, not accepted")
        entries = [{"id": o["object_id"], "kind": plain_kind(o["kind"]), "rack": data["racks"][o["object_id"]],
                    "slot": "random", "variant": "random",
                    "position": [float(v) for v in o["rack_local_pose"]["position_m"]],
                    "quaternion_xyzw": [float(v) for v in o["rack_local_pose"]["quaternion_xyzw"]], "keep": False}
                   for o in data["objects"]]
    return path, entries


def read_gate(args, entries):
    """The load's finished joint gate (frigidaire_initial_state_validate.py --out-dir), read Kit-free."""
    path = Path(args.gate) / "result.json"
    if not path.is_file():
        raise Refused(f"no joint gate result {rel(path)}")
    gate = json.loads(path.read_text())
    planned = {o["object_id"]: o["rack_local_pose"]["position_m"] for o in gate.get("candidate_objects") or []}
    if planned and planned != {e["id"]: e["position"] for e in entries}:    # every load has the same 24 dish ids
        raise Refused(f"the joint gate {rel(args.gate)} ran on another load")
    return gate


def gate_info(args, gate):
    return {"dir": rel(args.gate), "outcome": gate.get("outcome"), "reason": gate.get("reason") or gate.get("error"),
            "peak_penetration_limit_m": (gate.get("limits") or {}).get("peak_penetration_m"),
            **{k: gate[k] for k in ("maximum_settle_penetration_m", "maximum_cycle_penetration_m", "wall_seconds")
               if k in gate}}


def layers(ids, edges):
    """Longest-path layer of every dish over (a before b) edges, sources at 0; None when the edges hold a cycle."""
    succ, indeg = {i: [] for i in ids}, {i: 0 for i in ids}
    for a, b in set(map(tuple, edges)):
        succ[a].append(b)
        indeg[b] += 1
    wave, todo, done = {i: 0 for i in ids}, [i for i in ids if indeg[i] == 0], 0
    while todo:
        i = todo.pop()
        done += 1
        for j in succ[i]:
            wave[j] = max(wave[j], wave[i] + 1)
            indeg[j] -= 1
            if indeg[j] == 0:
                todo.append(j)
    return wave if done == len(ids) else None


def local_entries(P, entries, frames, poses):
    """``entries`` at the world ``poses``, rack-local in the rack ``frames`` (the score script's input)."""
    out = []
    for e in entries:
        local = P.local_from_world(frames[e["rack"]], poses[e["id"]])
        out.append({**e, "position": local["position_m"], "quaternion_xyzw": local["quaternion_xyzw"]})
    return out


def pairs_folder(args, name="pairs"):
    return args.out / (name if args.sampler == "restricted" else f"{name}_{args.sampler}")


def record_folder(args):
    """--out-dir, else <out>/<sampler>/ (--edges build), <out>/pairs/ or, --goal built, <out>/pairs_resettled/ (the
    random sampler: pairs_random/, pairs_resettled_random/)."""
    if args.out_dir:
        return args.out_dir
    if args.edges == "build":
        return args.out / args.sampler
    return pairs_folder(args, "pairs_resettled" if args.goal == "built" else "pairs")


def write_record(args, out_path, dep):
    if out_path.exists() and not args.force:
        raise Refused(f"refusing to overwrite {rel(out_path)} (--force)")
    out_path.parent.mkdir(parents=True, exist_ok=True)
    tmp = out_path.with_name(out_path.name + ".tmp")
    tmp.write_text(json.dumps(dep, indent=1, default=lambda o: o.tolist() if hasattr(o, "tolist") else str(o)) + "\n")
    os.replace(tmp, out_path)


def goal_setup(args, entries, gate, B, K, P):
    """The module docstring's targets: (baseline snapshot, its rack frames, {id: world target pose}, landing(kind,
    dist, dev), the rule's text, targets_source, entries_settled). Kit-free."""
    base = json.loads(Path(args.baseline).read_text())
    snapshot = base.get("snapshot", base)
    frames = {r: snapshot["poses"][r] for r in RACK_FRAMES}
    targets = {e["id"]: P.world_from_local(frames[e["rack"]], {"position_m": e["position"],
                                                               "quaternion_xyzw": e["quaternion_xyzw"]}) for e in entries}
    tail = LANDING_TAIL
    if args.sampler == "restricted":
        landing = lambda kind, dist, dev: dev <= K.DEV_MAX_M                          # noqa: E731
        rule = (f"own load: settle deviation <= {K.DEV_MAX_M * 1e3:.0f} mm of the commanded packed pose "
                f"(released {B.GOAL_HOVER_M * 1e3:.0f} mm above it), {tail}")
    else:
        landing = lambda kind, dist, dev: B.within_goal(kind, dist)                   # noqa: E731
        tol = ", ".join(f"{k} {v[0] * 1e3:.0f} mm lateral / {v[1] * 1e3:.0f} mm dz / {v[2]:.0f} deg"
                        for k, v in B.AT_GOAL.items())
        rule = f"at-goal tolerance of the settled pose ({tol}), {tail}"
    targets_source = "commanded" if args.sampler == "restricted" else "farm_settled"
    entries_settled = []
    if gate is not None:          # accepted joint gate: its settled world poses (racks extended, the same baseline)
        joint = gate["initial_snapshot"]["poses"]
        targets = {e["id"]: joint[e["id"]] for e in entries}
        landing = lambda kind, dist, dev: B.within_goal(kind, dist)                   # noqa: E731
        targets_source, rule = "joint_gate", f"at-goal tolerance of the jointly settled pose, {tail}"
        entries_settled = local_entries(P, entries, joint, joint)    # rack-local in the gate's measured racks
    return snapshot, frames, targets, landing, rule, targets_source, entries_settled


def dependency(args, source, entries, gate, out_path, kit_dir, report, started):
    """--edges build: the 2026-10-13 graph (support + learned + neighbour edges of the build)."""
    import frigidaire_bench as B
    import frigidaire_bench_kit as K
    from dishsim_frigidaire import planner as P
    from dishsim_frigidaire.loading import visual_points

    snapshot, frames, targets, landing, rule, targets_source, entries_settled = goal_setup(args, entries, gate, B, K, P)
    centroids = B.kind_centroids({kind: visual_points(B.ASSETS / f"{kind}.usda") for kind in B.KINDS})
    print(f"building {args.load} ({args.sampler}, targets {targets_source}): {len(entries)} dishes; "
          f"landing = {rule}", flush=True)
    backend = K._sequence_scene(args, B, entries, snapshot, kit_dir, report)
    if backend is None:
        raise Refused("the extended baseline did not hold")
    by_id = {e["id"]: e for e in entries}
    built, record, _ = K.build_sequence(backend, B, P, by_id, targets, centroids, landing)
    builds = record["builds"]
    for k, b in enumerate(builds):
        print(f"build {k + 1}: {b['status']} {b['dish'] or ''} {b['moved'] or ''}", flush=True)

    ids = [e["id"] for e in entries]
    learned = sorted(tuple(x) for x in record["learned"])
    ok = built == "ok"
    if ok:
        status = "ok"
        support = sorted(set(map(tuple, record["goal_order"])) - set(learned))
        neighbour = [tuple(x) for x in B.neighbour_order(record["order"], record["poses_world"],
                                                          {e["id"]: e["rack"] for e in entries})]
        poses_world = record["poses_world"]
        built_entries = local_entries(P, entries, frames, poses_world)
    else:
        support, neighbour, poses_world, built_entries = [], [], {}, []
        status = ("cycle" if layers(ids, learned) is None else "no_order" if len(builds) >= K.SEQUENCE_BUILDS
                  else builds[-1]["status"])
    deps = sorted(set(support) | set(learned))
    waves = layers(ids, sorted(set(deps) | set(neighbour)) if ok else learned)
    dep_waves = layers(ids, deps) if ok else None
    n_waves = max(waves.values()) + 1 if waves is not None else None
    reason = record["reason"] + ("; the learned order constraints form a cycle" if status == "cycle" else "")
    dep = {"load_id": args.load, "sampler": args.sampler, "source": rel(source), "status": status, "reason": reason,
           "builds": builds, "learned": [list(x) for x in learned], "support": [list(x) for x in support],
           "neighbour": [list(x) for x in neighbour], "goal_order": [list(x) for x in deps],
           "order": record.get("order") or builds[-1]["order"], "entries": entries, "targets_source": targets_source,
           "targets_world": targets, "poses_world": poses_world, "entries_settled": entries_settled,
           "built_entries": built_entries, "frames": frames,
           **({"waves": waves} if waves is not None else {}), "n_waves": n_waves,
           "n_waves_dependencies": max(dep_waves.values()) + 1 if dep_waves is not None else None,
           "acyclic": waves is not None, "failed_dish": None if ok else builds[-1]["dish"], "landing_rule": rule,
           "gate": gate_info(args, gate) if gate is not None else None,
           "kit": {"wall_seconds": round(time.monotonic() - started, 1), "builds_run": len(builds)},
           "generated_utc": datetime.now(timezone.utc).isoformat()}
    write_record(args, out_path, dep)
    report.update(result="PASS", status=status, dependency=rel(out_path),
                  reason=f"status {status} builds {len(builds)} support {len(support)} learned {len(learned)} "
                         f"neighbour {len(neighbour)} waves {n_waves if n_waves is not None else 'none'} "
                         f"targets {targets_source}")


def candidate_pairs(checker, entries, targets, max_m=PAIR_M):
    """[a, b, distance_mm] (a < b, sorted) for every dish pair whose collision pieces come within ``max_m`` at the
    world ``targets``, any rack: the checker's convex parts per dish (InitialCollisionChecker._body), an AABB prefilter
    on the authored collision bounds, then one FCL distance over the two dishes' pieces (overlap, FCL's -1, is 0).
    Kit-free (pxr reads the parts: run_py.sh, or after the Kit boot)."""
    import numpy as np
    fcl = checker.fcl
    bodies = [checker._body(e["kind"], targets[e["id"]], e["id"]) for e in sorted(entries, key=lambda e: e["id"])]
    out = []
    for n, a in enumerate(bodies):
        for b in bodies[n + 1:]:
            if np.any(a["bounds"][0] > b["bounds"][1] + max_m) or np.any(b["bounds"][0] > a["bounds"][1] + max_m):
                continue
            data = fcl.DistanceData(request=fcl.DistanceRequest(), result=fcl.DistanceResult())
            a["manager"].distance(b["manager"], data, fcl.defaultDistanceCallback)
            d = max(float(data.result.min_distance), 0.)
            if d < max_m:
                out.append([a["id"], b["id"], round(d * 1e3, 2)])
    return out


def graph_edges(tests):
    """{(a, b): [reasons]} from the v20 tests (module docstring): knock (B, X) for every X the verdict of L(B) disturbed
    (ended outside its at-goal tolerance), need (B, X) for every X outside its tolerance after the first re-settle of
    L(B)'s context, block (B, A) when L(B)'s verdict did not land and R(A, B)'s did."""
    last = {t["dish"]: t for t in tests if not t["removed"]}
    reasons = {}
    for b, t in last.items():
        for x in t["verdict"]["disturbed"]:
            reasons.setdefault((b, x), set()).add("knock")
        for x in t["context_rounds"][0]["fallen"]:
            reasons.setdefault((b, x), set()).add("need")
    for t in tests:
        if t["removed"]:
            a, b = t["removed"][0], t["dish"]
            if not last[b]["verdict"]["landed"] and t["verdict"]["landed"]:
                reasons.setdefault((b, a), set()).add("block")
    return {e: [r for r in REASONS if r in rs] for e, rs in sorted(reasons.items())}


def two_way(edges):
    """[[a, b], class] for every pair with edges both ways (a < b): mutual_support when both carry need, else cycle."""
    return [[[a, b], "mutual_support" if "need" in edges[(a, b)] and "need" in edges[(b, a)] else "cycle"]
            for a, b in sorted(edges) if a < b and (b, a) in edges]


def groups_of(pairs):
    """{dish: representative} merging the given pairs (union-find; representative = smallest id)."""
    rep = {}

    def find(i):
        while rep.get(i, i) != i:
            i = rep[i]
        return i
    for a, b in pairs:
        ra, rb = find(a), find(b)
        if ra != rb:
            rep[max(ra, rb)] = min(ra, rb)
    return {i: find(i) for i in {x for p in pairs for x in p}}


def first_order(path):
    """--goal built: (the (a before b) edges of the load's --edges pairs record at ``path`` minus its two-way pairs and
    any edge still on a longer cycle (b reaches a), so build_sequence starts acyclic; the record's relative path),
    ([], None) without a record."""
    if not path.is_file():
        return [], None
    edges = {(e["a"], e["b"]) for e in json.loads(path.read_text()).get("edges") or []}
    edges -= {(b, a) for a, b in edges}
    succ = {}
    for a, b in edges:
        succ.setdefault(a, set()).add(b)

    def reach(i):
        seen, todo = set(), [i]
        while todo:
            for j in succ.get(todo.pop(), ()):
                if j not in seen:
                    seen.add(j)
                    todo.append(j)
        return seen
    return sorted((a, b) for a, b in edges if a not in reach(b)), rel(path)


def goal_delta(B, P, entries, centroids, built, targets):
    """--goal built: {per_dish: {id: {lateral_mm, dz_mm, tilt_deg}}, max: the same over dishes} from the previous
    target to the built pose of every dish (B.goal_distance, as the drops judge landing)."""
    per = {}
    for e in entries:
        d = B.goal_distance(centroids[e["kind"]], P.pose_T(built[e["id"]]), P.pose_T(targets[e["id"]]))
        per[e["id"]] = {"lateral_mm": round(d[0] * 1e3, 1), "dz_mm": round(d[1] * 1e3, 1), "tilt_deg": round(d[2], 1)}
    return {"per_dish": per, "max": {k: max(v[k] for v in per.values()) for k in ("lateral_mm", "dz_mm", "tilt_deg")}}


def perturbation(args):
    return {"lateral_mm": args.perturb_mm, "yaw_deg": args.perturb_deg, "seed": args.perturb_seed}


def perturb(T_cmd, centroid, R_rack, lateral_m, yaw_deg, seed, index):
    """--perturb-*: (``T_cmd`` yawed U(-yaw_deg, yaw_deg) about the world Z axis through the dish's visual ``centroid``,
    then shifted by a point uniform in the disc of radius ``lateral_m`` in the rack plane (``R_rack``'s x/y axes),
    (dx mm, dy mm, yaw deg) as applied); numpy default_rng((seed, index)), so a run is reproducible."""
    import numpy as np
    rng = np.random.default_rng((seed, index))
    r, th, yaw = lateral_m * np.sqrt(rng.random()), 2. * np.pi * rng.random(), float(rng.uniform(-yaw_deg, yaw_deg))
    dx, dy, a = float(r * np.cos(th)), float(r * np.sin(th)), np.radians(yaw)
    Rz = np.array([[np.cos(a), -np.sin(a), 0.], [np.sin(a), np.cos(a), 0.], [0., 0., 1.]])
    c = T_cmd[:3, :3] @ np.asarray(centroid, dtype=float) + T_cmd[:3, 3]     # the centroid stays put under the yaw
    T = T_cmd.copy()
    T[:3, :3] = Rz @ T_cmd[:3, :3]
    T[:3, 3] = c + Rz @ (T_cmd[:3, 3] - c) + np.asarray(R_rack, dtype=float) @ np.array([dx, dy, 0.])
    return T, (dx * 1e3, dy * 1e3, yaw)


def pairs(args, source, entries, gate, out_path, kit_dir, report, started):
    """--edges pairs: the v20 ordered-pair test (module docstring), its graph and its certifying (or, for mutual
    support, diagnostic) build; with --goal built the goal is first re-settled one dish at a time and the tests run on
    the built poses."""
    import numpy as np
    import frigidaire_bench as B
    import frigidaire_bench_kit as K
    from dishsim import rearrange as R
    from dishsim_frigidaire import planner as P
    from dishsim_frigidaire.loading import visual_points
    from dishsim_frigidaire.random_poses import LIMITS

    snapshot, frames, targets, landing, rule, targets_source, entries_settled = goal_setup(args, entries, gate, B, K, P)
    ids = [e["id"] for e in entries]
    by_id = {e["id"]: e for e in entries}
    centroids = B.kind_centroids({kind: visual_points(B.ASSETS / f"{kind}.usda") for kind in B.KINDS})
    backend, resettle, kit_resettle, head = None, {}, {}, ""
    if args.goal == "built":                                        # the goal re-settled one dish at a time first
        backend = K._sequence_scene(args, B, entries, snapshot, kit_dir, report)
        if backend is None:
            raise Refused("the extended baseline did not hold")
        initial, original = first_order(pairs_folder(args) / f"{args.load}.json")
        own = (f"own load: settle deviation <= {K.DEV_MAX_M * 1e3:.0f} mm of the commanded pose (the {targets_source} "
               f"pose + {B.GOAL_HOVER_M * 1e3:.0f} mm), {LANDING_TAIL}")
        print(f"resettle {args.load} ({args.sampler}, targets {targets_source}): {len(entries)} dishes one at a time, "
              f"{len(initial)} initial constraints from {original or 'no pairs record'}; landing = {own}", flush=True)
        t_resettle = time.monotonic()
        built, record, _ = K.build_sequence(backend, B, P, by_id, targets, centroids,
                                            lambda kind, dist, dev: dev <= K.DEV_MAX_M, initial=initial)
        for k, b in enumerate(record["builds"], 1):
            print(f"resettle build {k}: {b['status']} {b['dish'] or ''} {b['moved'] or ''}", flush=True)
        resettle = {"goal_source": "sequential_build", "goal_delta": None, "resettle_builds": record["builds"],
                    "resettle_initial": [list(x) for x in initial], "resettle_rule": own, "original": original}
        kit_resettle = {"resettle_wall_seconds": round(time.monotonic() - t_resettle, 1),
                        "resettle_builds_run": len(record["builds"])}
        if built != "ok":
            return gate_failed(args, source, entries, gate, out_path, report, "resettle_failed",
                               "the goal did not build one dish at a time: " + record["reason"],
                               targets_source=targets_source, targets_world=targets, entries_settled=entries_settled,
                               frames=frames, failed_dish=record["builds"][-1]["dish"], **resettle,
                               kit={"wall_seconds": round(time.monotonic() - started, 1), **kit_resettle})
        resettle["goal_delta"] = goal_delta(B, P, entries, centroids, record["poses_world"], targets)
        top = resettle["goal_delta"]["max"]
        head = (f"resettle builds {len(record['builds'])} goal delta max lateral {top['lateral_mm']} "
                f"dz {top['dz_mm']} mm tilt {top['tilt_deg']} deg ")
        print(f"goal built from the {targets_source} pose: {head}({kit_resettle['resettle_wall_seconds']:.0f} s)",
              flush=True)
        targets, targets_source = record["poses_world"], "sequential_build"
        landing = lambda kind, dist, dev: B.within_goal(kind, dist)                   # noqa: E731
        rule = f"at-goal tolerance of the sequentially built pose, {LANDING_TAIL}"
        entries_settled = local_entries(P, entries, frames, targets)
    pair_list = candidate_pairs(B.bench_checker(), entries, targets)
    runs = [(i, []) for i in ids]                                   # L(B), then R(A, B) both ways per candidate pair
    for a, b, _ in pair_list:
        runs += [(b, [a]), (a, [b])]
    on = bool(args.perturb_mm or args.perturb_deg)                 # --perturb-*: the L and R drops
    shaken = (f" perturb {args.perturb_mm:g} mm {args.perturb_deg:g} deg seed {args.perturb_seed}" if on else "")
    seeds = tuple(10 * args.perturb_seed + s for s in RERUN_SEEDS)
    print(f"pairs {args.load} ({args.sampler}, targets {targets_source}): {len(entries)} dishes, {len(pair_list)} "
          f"candidate pairs within {PAIR_M * 1e3:.0f} mm, {len(runs)} drops; landing = {rule}; release = lowest clear "
          f"hover ({B.RELEASE_RULE}, {B.RELEASE_STEP_M * 1e3:g} mm steps, cap {B.RELEASE_CAP_M * 1e3:g} mm); marginal "
          f"reruns {RERUN_MM:g} mm {RERUN_DEG:g} deg seeds {list(seeds)}"
          + (f";{shaken} (offset uniform in the disc, yaw uniform)" if on else ""), flush=True)
    if backend is None:
        backend = K._sequence_scene(args, B, entries, snapshot, kit_dir, report)
        if backend is None:
            raise Refused("the extended baseline did not hold")
    park = {i: {"position_m": [3. + .4 * j, 2., .3], "quaternion_xyzw": [0., 0., 0., 1.]} for j, i in enumerate(by_id)}
    limit = LIMITS["peak_penetration_m"]
    checker = B.release_checker()
    kind = {i: by_id[i]["kind"] for i in ids}
    goals = {i: P.pose_T(targets[i]) for i in ids}
    tol = {i: B.AT_GOAL[kind[i]] for i in ids}
    rack_R = {i: P.pose_T(frames[by_id[i]["rack"]])[:3, :3] for i in ids}
    state = {}

    def gd(i, T):
        return B.goal_distance(centroids[kind[i]], T, goals[i])

    def mm3(d):
        return [round(d[0] * 1e3, 1), round(d[1] * 1e3, 1), round(d[2], 1)]

    def context(absent0):
        """Park ``absent0``, every other dish at its target, re-settle 150 ticks; while a context dish ends outside its
        at-goal tolerance (fallen), park it too and re-settle again (FALL_ROUNDS at most). (poses, absent, rounds)."""
        absent, rounds = set(absent0), []
        while True:
            for i in ids:
                backend.set_rigid_pose(backend.objects[i], park[i] if i in absent else targets[i])
            for _ in range(K.SETTLE_TICKS):
                backend.tick()
            now = backend.poses()
            drift = {i: gd(i, P.pose_T(now[i])) for i in ids if i not in absent}
            fallen = sorted(i for i, d in drift.items() if not B.within_goal(kind[i], d))
            rounds.append({"parked": sorted(absent - set(absent0)), "fallen": fallen,
                           "drift": {i: mm3(d) for i, d in drift.items()}})
            if not fallen or len(rounds) >= FALL_ROUNDS:
                return now, absent, rounds
            absent |= set(fallen)

    def drop(dish, removed, index, seed=None, mm=0., deg=0.):
        """One drop of ``dish`` with ``removed`` absent (module docstring), released at its lowest clear hover above
        the target (with ``seed``: that release pose perturbed by ``mm`` / ``deg``, as --perturb-* does)."""
        now, absent, rounds = context({dish, *removed})
        ctx = [i for i in ids if i not in absent]
        pre = {i: P.pose_T(now[i]) for i in ctx}
        if "appliance" not in state:
            state["appliance"] = B.appliance_bodies(checker, now)
        off = (0., 0., 0.)
        close = B.near_poses({i: now[i] for i in ctx}, kind, targets[dish]["position_m"])
        bodies = [checker._body(kind[i], pose, i) for i, pose in close.items()]
        release, hover, blockers = B.clear_release(checker, [(kind[dish], targets[dish], dish)], bodies,
                                                   state["appliance"], rack_R[dish][:, 2])
        if release is not None and seed is not None:                # the release pose perturbed (the --perturb-*
            T, off = perturb(P.pose_T(release[0]), centroids[kind[dish]], rack_R[dish], mm * 1e-3, deg, seed, index)
            release = [P.pose_dict(T)]                              # mechanism); landing judged against the target
        out = {"dish": dish, "removed": list(removed), "index": index, "seed": seed,
               "parked_fallen": sorted(absent - {dish, *removed}), "context_rounds": rounds,
               "context_valid": not rounds[-1]["fallen"],
               "context_drift_mm": round(max((float(np.linalg.norm(pre[i][:3, 3] - goals[i][:3, 3])) * 1e3 for i in ctx),
                                             default=0.), 1),
               "dx_mm": round(off[0], 2), "dy_mm": round(off[1], 2), "yaw_deg": round(off[2], 2)}
        if release is None:                                         # no clear release: not landed, nothing dropped
            out.update(landed=False, no_clear_release=True, hover_mm=None, blockers=blockers, lateral_mm=None,
                       dz_mm=None, tilt_deg=None, contact_mm=None, disturbed=[], moved={}, margins={}, marginal=[])
            return out
        backend.set_rigid_pose(backend.objects[dish], release[0])
        for _ in range(K.SETTLE_TICKS + K.WINDOW_TICKS):
            backend.tick()
        now = backend.poses()
        post = {i: P.pose_T(now[i]) for i in [*ctx, dish]}
        dist = gd(dish, post[dish])
        dev = float(np.linalg.norm(post[dish][:3, 3] - np.asarray(release[0]["position_m"], dtype=float)))
        deep = float(max((d for name, d in backend.latest_contact["depths_m"].items() if dish in name.split("|")),
                         default=0.))
        after = {i: gd(i, post[i]) for i in ctx}
        moved = {}
        for i in ctx:
            dp, dr = float(np.linalg.norm(post[i][:3, 3] - pre[i][:3, 3])) * 1e3, R.rot_angle_deg(pre[i], post[i])
            if dp > MOVED_MM or dr > MOVED_DEG:
                moved[i] = {"moved_mm": round(dp, 1), "moved_deg": round(dr, 1), "goal": mm3(after[i])}
        t = tol[dish]
        margins = {"lateral": (t[0] - dist[0]) / t[0], "height": (t[1] - dist[1]) / t[1],
                   "tilt": (t[2] - dist[2]) / t[2], "contact": (limit - deep) / limit}
        near = [(tol[i][c] - after[i][c]) / tol[i][c] for i in moved for c in range(3)]
        margins["disturbance"] = min(near, key=abs) if near else None
        out.update(landed=bool(landing(kind[dish], dist, dev)) and deep < limit, no_clear_release=False,
                   hover_mm=round(hover * 1e3), blockers=[], lateral_mm=round(dist[0] * 1e3, 1),
                   dz_mm=round(dist[1] * 1e3, 1), tilt_deg=round(dist[2], 1), contact_mm=round(deep * 1e3, 2),
                   disturbed=sorted(i for i in ctx if not B.within_goal(kind[i], after[i])), moved=moved,
                   margins={k: None if v is None else round(v, 3) for k, v in margins.items()},
                   marginal=[k for k, v in margins.items() if v is not None and abs(v) <= MARGIN])
        return out

    def verdict(t):
        """The single drop's verdict, or the majority of its 3 perturbed reruns."""
        rs = t.get("reruns")
        if not rs:
            t["verdict"] = {"landed": t["landed"], "disturbed": t["disturbed"], "source": "single"}
        else:
            hits = [x for r in rs for x in r["disturbed"]]
            t["verdict"] = {"landed": sum(r["landed"] for r in rs) * 2 > len(rs),
                            "disturbed": sorted(x for x in set(hits) if hits.count(x) * 2 > len(rs)),
                            "source": f"majority of {len(rs)} perturbed releases"}
        return t

    def label(t):
        return f"{t['dish']} " + (f"without {t['removed'][0]}" if t["removed"] else "last")

    def show(t, head):
        print(f"{head} {label(t)}" + (f" seed {t['seed']} offset {t['dx_mm']} {t['dy_mm']} mm yaw {t['yaw_deg']} deg"
                                      if t["seed"] is not None else "")
              + (f" parked fallen {t['parked_fallen']}" if t["parked_fallen"] else "")
              + (f": no clear release (blockers {t['blockers']})" if t["no_clear_release"] else
                 f": hover {t['hover_mm']} mm landed {t['landed']} lateral {t['lateral_mm']} dz {t['dz_mm']} mm tilt "
                 f"{t['tilt_deg']} deg contact {t['contact_mm']} mm disturbed {t['disturbed'] or '-'} marginal "
                 f"{t['marginal'] or '-'}")
              + f" need {t['context_rounds'][0]['fallen'] or '-'} drift {t['context_drift_mm']} mm "
                f"({time.monotonic() - t0:.0f} s)", flush=True)

    def rerun(t, why):
        if t.get("reruns"):
            return False
        t["reruns"], t["rerun_reason"] = [], why
        for sd in seeds:
            r = drop(t["dish"], t["removed"], t["index"], sd, RERUN_MM, RERUN_DEG)
            t["reruns"].append(r)
            show(r, f"  rerun ({why})")
        verdict(t)
        print(f"  verdict {label(t)}: landed {t['verdict']['landed']} disturbed {t['verdict']['disturbed'] or '-'} "
              f"({t['verdict']['source']})", flush=True)
        return True

    tests, t0 = [], time.monotonic()
    for k, (dish, removed) in enumerate(runs, 1):
        t = drop(dish, removed, k, args.perturb_seed if on else None, args.perturb_mm, args.perturb_deg)
        tests.append(verdict(t))
        show(t, f"test {k}/{len(runs)}")
    drop_seconds = time.monotonic() - t0
    for t in tests:                                                 # margins: 3 perturbed releases, majority
        if t["marginal"]:
            rerun(t, "margin " + "+".join(t["marginal"]))
    margin_seconds = time.monotonic() - t0 - drop_seconds
    edges = graph_edges(tests)
    for _ in range(3):                                              # two-way pairs without need both ways: retested
        todo = [p for p, cls in two_way(edges) if cls == "cycle"]
        fresh = [rerun(t, "two-way " + "-".join(p)) for p in todo for t in tests
                 if (not t["removed"] and t["dish"] in p) or (t["removed"] and {t["dish"], t["removed"][0]} == set(p))]
        if not any(fresh):
            break
        edges = graph_edges(tests)
    retest_seconds = time.monotonic() - t0 - drop_seconds - margin_seconds
    pairs2 = two_way(edges)
    mutual = [p for p, cls in pairs2 if cls == "mutual_support"]
    cycles = [p for p, cls in pairs2 if cls == "cycle"]
    goal_order = sorted(edges)
    count = {r: sum(r in rs for rs in edges.values()) for r in REASONS[:3]}
    rep = groups_of(mutual)
    node = {i: rep.get(i, i) for i in ids}
    contracted = sorted({(node[a], node[b]) for a, b in goal_order if node[a] != node[b]})
    open_order = [e for e in goal_order if node[e[0]] != node[e[1]]]   # the graph minus the mutual pairs' own edges
    waves = layers(ids, open_order) if layers(sorted(set(node.values())), contracted) is not None else None
    n_waves = max(waves.values()) + 1 if waves is not None else None
    print(f"graph: {len(edges)} edges ({', '.join(f'{r} {n}' for r, n in count.items())}), cycles {cycles or '-'}, "
          f"mutual support {mutual or '-'}, waves {n_waves if n_waves is not None else 'none'}", flush=True)

    builds, extra, order, poses_world, built_entries, failed, diagnostic = [], [], [], targets, [], None, False
    if waves is None:
        status = "cycle"
        reason = "no valid one-at-a-time order: " + (
            "two-way pairs " + ", ".join(f"{a} <-> {b}" for a, b in cycles) if cycles
            else "the edges form a cycle through more than two dishes") + (
            f"; mutual support {', '.join(f'{a} <-> {b}' for a, b in mutual)}" if mutual else "")
    else:
        together = [sorted(i for i in ids if node[i] == r) for r in sorted(set(rep.values()))]
        diagnostic = bool(together)
        built, record, _ = K.build_sequence(backend, B, P, by_id, targets, centroids, landing, initial=open_order,
                                            together=together)
        builds = record["builds"]
        for k, b in enumerate(builds):
            print(f"{'diagnostic ' if diagnostic else ''}build {k + 1}: {b['status']} {b['dish'] or ''} "
                  f"{b['moved'] or ''}", flush=True)
        learned = sorted(tuple(x) for x in record["learned"])
        extra = sorted(set(learned) - set(open_order))
        order = record.get("order") or builds[-1]["order"]
        if built == "ok":
            poses_world = record["poses_world"]
            built_entries = local_entries(P, entries, frames, poses_world)
        else:
            failed = builds[-1]["dish"]
        tail = (f"; {len(extra)} constraint(s) learned beyond the graph's {len(open_order)} edges"
                + ("; the learned constraints close a cycle" if layers(ids, learned) is None else ""))
        if diagnostic:
            status = "mutual_support"
            reason = (f"place together or re-roll: mutual support {', '.join(f'{a} <-> {b}' for a, b in mutual)}; "
                      f"diagnostic build (each pair released together, not a benchmark move): {record['reason']}{tail}")
        else:
            status = "ok" if built == "ok" else "no_order" if len(builds) >= K.SEQUENCE_BUILDS else builds[-1]["status"]
            reason = record["reason"] + tail
        for a, b in extra:                                          # learned constraints join the graph
            edges[(a, b)] = edges.get((a, b), []) + ["learned"]
        if built == "ok" and layers(ids, open_order + extra) is not None:
            waves = layers(ids, open_order + extra)
            n_waves = max(waves.values()) + 1
    dep = {"load_id": args.load, "sampler": args.sampler, "source": rel(source), "procedure": "v20", "status": status,
           "reason": reason, "pairs_tested": pair_list, "tests": tests,
           "edges": [{"a": a, "b": b, "reasons": rs} for (a, b), rs in sorted(edges.items())],
           "two_way": [{"pair": p, "class": cls, "reasons_ab": edges.get(tuple(p), []),
                        "reasons_ba": edges.get(tuple(p[::-1]), [])} for p, cls in pairs2],
           "cycles": cycles, "mutual_support": mutual, "goal_order": [list(x) for x in goal_order], "order": order,
           "builds": builds, "diagnostic": diagnostic, "extra_constraints": [list(x) for x in extra], "waves": waves,
           "n_waves": n_waves, "acyclic": waves is not None, "failed_dish": failed, "entries": entries,
           "targets_source": targets_source, "targets_world": targets, "poses_world": poses_world,
           "entries_settled": entries_settled, "built_entries": built_entries, "frames": frames, "landing_rule": rule,
           "thresholds": {"at_goal": {k: {"lateral_mm": round(v[0] * 1e3, 3), "dz_mm": round(v[1] * 1e3, 3),
                                          "tilt_deg": v[2]} for k, v in B.AT_GOAL.items()},
                          "disturbed": "a context dish outside its at-goal tolerance of its target after the drop",
                          "contact_limit_mm": round(limit * 1e3, 3),
                          "release": {"rule": B.RELEASE_RULE, "allowance_mm": round(checker.penetration_limit_m * 1e3, 3),
                                      "step_mm": round(B.RELEASE_STEP_M * 1e3, 3), "cap_mm": round(B.RELEASE_CAP_M * 1e3, 3),
                                      "near_m": B.RELEASE_NEAR_M},
                          "margin": MARGIN, "rerun": {"lateral_mm": RERUN_MM, "yaw_deg": RERUN_DEG, "seeds": list(seeds)},
                          "moved_mm": MOVED_MM, "moved_deg": MOVED_DEG, "fall_rounds": FALL_ROUNDS,
                          "context_ticks": K.SETTLE_TICKS, "drop_ticks": K.SETTLE_TICKS + K.WINDOW_TICKS,
                          "physics_hz": 120, "candidate_mm": round(PAIR_M * 1e3, 3)},
           "perturbation": perturbation(args),
           "gate": gate_info(args, gate) if gate is not None else None, **resettle,
           "kit": {"wall_seconds": round(time.monotonic() - started, 1), "drops": len(tests),
                   "reruns": sum(len(t.get("reruns") or []) for t in tests),
                   "drop_wall_seconds": round(drop_seconds, 1), "margin_wall_seconds": round(margin_seconds, 1),
                   "retest_wall_seconds": round(retest_seconds, 1), "builds_run": len(builds), **kit_resettle},
           "generated_utc": datetime.now(timezone.utc).isoformat()}
    write_record(args, out_path, dep)
    reruns = sum(bool(t.get("reruns")) for t in tests)
    report.update(result="PASS", status=status, dependency=rel(out_path),
                  reason=head + f"status {status} pairs {len(pair_list)} drops {len(tests)} reruns {reruns} edges "
                         f"{len(edges)} (knock {count['knock']} need {count['need']} block {count['block']} learned "
                         f"{len(extra)}) cycles {len(cycles)} mutual {len(mutual)} builds {len(builds)}"
                         f"{' diagnostic' if diagnostic else ''} extra {len(extra)} waves "
                         f"{n_waves if n_waves is not None else 'none'} targets {targets_source}{shaken}")


def gate_failed(args, source, entries, gate, out_path, report, status="gate", reason=None, **fields):
    """A record without a graph: status 'gate' (the load failed its joint gate; no Kit boot) or, with --goal built,
    'resettle_failed' (the goal did not build one dish at a time; ``fields`` = what the run measured)."""
    info = gate_info(args, gate) if gate is not None else None
    reason = reason or f"joint gate {info['outcome']}: {info['reason']}"
    dep = {"load_id": args.load, "sampler": args.sampler, "source": rel(source), "status": status, "reason": reason,
           "builds": [], "learned": [], "support": [], "neighbour": [], "goal_order": [], "order": [],
           "entries": entries, "targets_source": "joint_gate", "targets_world": {}, "poses_world": {},
           "entries_settled": [], "built_entries": [], "frames": None, "n_waves": None, "n_waves_dependencies": None,
           "acyclic": None, "failed_dish": None, "landing_rule": None, "gate": info, "kit": None,
           "generated_utc": datetime.now(timezone.utc).isoformat()}
    if args.edges == "pairs":     # the pairs record's keys, empty (no build classes)
        dep = {k: v for k, v in dep.items() if k not in ("learned", "support", "neighbour", "n_waves_dependencies")}
        dep.update(pairs_tested=[], tests=[], edges=[], cycles=[], extra_constraints=[], waves=None, thresholds=None,
                   perturbation=perturbation(args))
    if args.goal == "built":
        dep.update(goal_source="sequential_build", goal_delta=None, resettle_builds=[], resettle_initial=[],
                   resettle_rule=None, original=None)
    dep.update(fields)
    write_record(args, out_path, dep)
    report.update(result="PASS", status=status, dependency=rel(out_path), reason=f"status {status} {reason}")


def write_manifest(args):
    """--manifest-out: the load's joint-gate manifest, frigidaire_hotec_top5.manifest_of(entries) (rack-local poses,
    the benchmark baseline = the default --baseline, the HOTEC v2 tableware). Kit-free: isaaclab is never imported."""
    import importlib.util
    spec = importlib.util.spec_from_file_location("top5", ROOT / "code/planner/frigidaire/frigidaire_hotec_top5.py")
    T = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(T)    # Kit-free at import: numpy, dishsim and a private copy of frigidaire_bench
    try:
        _, entries = read_load(args)
    except Refused as exc:
        print(f"[RESULT] FAIL {args.load} {exc}", flush=True)
        return
    args.manifest_out.parent.mkdir(parents=True, exist_ok=True)
    args.manifest_out.write_text(json.dumps(T.manifest_of(entries), indent=1) + "\n")
    print(f"[RESULT] PASS manifest {rel(args.manifest_out)}", flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--sampler", choices=("restricted", "random"), required=True)
    parser.add_argument("--load", "--loads", dest="load", required=True, metavar="ID", help="one load id per Kit process")
    parser.add_argument("--restricted-dir", type=Path, default=ROOT / "data/results/dot_rules/restricted/loads")
    parser.add_argument("--farm-dir", type=Path, default=ROOT / "data/results/random_loads/rl_20261009")
    parser.add_argument("--baseline", type=Path, default=ROOT / "data/results/benchmark/frigidaire_hotec/baseline/result.json")
    parser.add_argument("--usd", type=Path, default=ROOT / "data/build/frigidaire_collection/usd/fdpc4221as.usdc")
    parser.add_argument("--out", type=Path, default=ROOT / "data/results/dot_rules/dependency")
    parser.add_argument("--out-dir", type=Path, metavar="PATH",
                        help="write <load_id>.json and _kit/<load_id>/ under PATH instead of the folder under --out")
    parser.add_argument("--gate", type=Path, metavar="DIR",
                        help="the load's finished joint gate (its --out-dir): build toward the jointly settled poses")
    parser.add_argument("--edges", choices=("pairs", "build"), default="pairs",
                        help="pairs: the ordered-pair drop-test graph, <out>/pairs/ (default); "
                             "build: the 2026-10-13 build-learned graph, <out>/<sampler>/")
    parser.add_argument("--goal", choices=("gate", "built"), default="gate",
                        help="--edges pairs: gate = test the targets as they are (default); built = re-settle the "
                             "goal one dish at a time first, then test the built poses, <out>/pairs_resettled/")
    parser.add_argument("--perturb-mm", type=float, default=0., metavar="X",
                        help="--edges pairs: release each L/R drop's dish at a random lateral offset, uniform in the "
                             "disc of radius X mm in its rack plane (default 0: unperturbed)")
    parser.add_argument("--perturb-deg", type=float, default=0., metavar="Y",
                        help="--edges pairs: and at a random yaw about world Z through its centroid, uniform in "
                             "+-Y deg (default 0)")
    parser.add_argument("--perturb-seed", type=int, default=0, metavar="S",
                        help="the offsets' numpy default_rng((S, test number)) (default 0)")
    parser.add_argument("--manifest-out", type=Path, metavar="PATH", help="Kit-free: write the joint-gate manifest, exit")
    parser.add_argument("--max-wall-seconds", type=float, default=3600.)
    parser.add_argument("--force", action="store_true", help="overwrite an existing output")
    args, _ = parser.parse_known_args()                       # --manifest-out exits before isaaclab is imported
    if args.manifest_out:
        return write_manifest(args)
    if args.goal == "built" and args.edges != "pairs":
        parser.error("--goal built needs --edges pairs")
    if (args.perturb_mm or args.perturb_deg) and args.edges != "pairs":
        parser.error("--perturb-mm / --perturb-deg need --edges pairs")
    if min(args.perturb_mm, args.perturb_deg, args.perturb_seed) < 0:
        parser.error("--perturb-mm, --perturb-deg and --perturb-seed must be >= 0")
    from isaaclab.app import AppLauncher
    AppLauncher.add_app_launcher_args(parser)
    parser.set_defaults(device="cpu", headless=True)
    args = parser.parse_args()
    folder = record_folder(args)
    out_path = folder / f"{args.load}.json"
    if out_path.exists() and not args.force:
        print(f"[RESULT] FAIL {args.load} refusing to overwrite {rel(out_path)} (--force)", flush=True)
        return
    kit_dir = folder / "_kit" / args.load
    started = time.monotonic()
    report = {"result": "FAIL", "reason": None, "load_id": args.load, "sampler": args.sampler,
              "started_utc": datetime.now(timezone.utc).isoformat()}
    app = None
    try:
        source, entries = read_load(args)
        report["source"] = rel(source)
        gate = read_gate(args, entries) if args.gate else None
        if gate is not None and gate.get("outcome") != "accepted":
            return gate_failed(args, source, entries, gate, out_path, report)      # (finally: report, [RESULT])
        app = AppLauncher(args).app
        args.app = app
        sys.path[:0] = [str(ROOT / "code/src"), str(ROOT / "code/frigidaire/src"), str(ROOT / "code/planner/frigidaire")]
        (pairs if args.edges == "pairs" else dependency)(args, source, entries, gate, out_path, kit_dir, report, started)
    except Refused as exc:
        report.update(result="FAIL", reason=str(exc))
    except Exception as exc:                                   # noqa: BLE001
        traceback.print_exc()
        report.update(result="FAIL", reason=f"exception {exc!r}")
    finally:
        report.update(wall_seconds=time.monotonic() - started, finished_utc=datetime.now(timezone.utc).isoformat())
        kit_dir.mkdir(parents=True, exist_ok=True)
        (kit_dir / "report.json").write_text(json.dumps(report, indent=2, default=str) + "\n")
        goal = "goal built " if args.goal == "built" else ""
        print(f"[RESULT] {report['result']} {args.load} {goal}{report['reason']}", flush=True)
        if app is not None:
            from dishsim.media import release_sim_for_close
            release_sim_for_close()
            app.close()


if __name__ == "__main__":
    main()
