#!/usr/bin/env python3
# Copyright (c) 2026, dishsim project.
# SPDX-License-Identifier: BSD-3-Clause
"""Isaac dependency graph of one 24-dish load (boot-first; one load per Kit process).

--edges pairs (the default, 2026-10-14): one edge type from ordered-pair drop tests on the goal load alone. Candidate
pairs: every dish pair whose collision pieces (frigidaire_bench.bench_checker's convex parts) come within PAIR_M =
20 mm at the target poses, any rack (FCL distance; FCL reports overlapping pieces as -1, recorded as 0 mm). A test
parks its absent set S off-scene (as build_sequence parks), teleports every other dish to its exact target and
re-settles that context 150 ticks (its poses = pre), then releases the dropped dish B at its target + the goal hover
and settles 150 + 60 ticks: landed = the landing rule below and contact depth < 2 mm; disturbed = context dishes moved
> 10 mm / 20 deg from pre. L(B): S = {B} (B dropped last into the full load), for every dish; R(A, B): S = {A, B},
for both orientations of every candidate pair; every test runs once. Edges (a before b): knock (B, X) for every X
that L(B) disturbed; need (A, B) when L(B) landed and R(A, B) did not; block (B, A) when L(B) did not land and R(A, B)
did; one edge per (a, b) with its reasons. Edges both ways = a cycle (no valid one-at-a-time order): status 'cycle',
no build. An acyclic graph is certified by build_sequence started from its edges (extra_constraints = what the build
learned beyond them): status ok | no_order | landing | moved; a failed build keeps the complete graph and names the
failing dish. Writes <out>/pairs/<load_id>.json (the random sampler: pairs_random/).

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
and R drop commands the dropped dish at its target + the goal hover, yawed U(-Y, Y) deg about the world Z axis
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
(c) the graph (24 + 2 x pairs drops of 150 + 210 ticks, then the certifying build: about 25-40 min of Kit per load):
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
REASONS = ("knock", "need", "block")
LANDING_TAIL = "contact depth < 2 mm; no placed dish moved > 10 mm / 20 deg"


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
    """({(a, b): [reasons]}, two-way pairs [a, b] with a < b) from the drop tests (module docstring)."""
    last = {t["dish"]: t for t in tests if not t["removed"]}
    reasons = {}
    for b, t in last.items():
        for x in t["disturbed"]:
            reasons.setdefault((b, x), set()).add("knock")
    for t in tests:
        if t["removed"]:
            a, b = t["removed"][0], t["dish"]
            if last[b]["landed"] and not t["landed"]:
                reasons.setdefault((a, b), set()).add("need")
            elif not last[b]["landed"] and t["landed"]:
                reasons.setdefault((b, a), set()).add("block")
    edges = {e: [r for r in REASONS if r in rs] for e, rs in sorted(reasons.items())}
    return edges, sorted([a, b] for a, b in edges if a < b and (b, a) in edges)


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
    """--edges pairs: the ordered-pair drop tests, their graph and its certifying build (module docstring); with
    --goal built the goal is first re-settled one dish at a time and the tests run on the built poses."""
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
    on = bool(args.perturb_mm or args.perturb_deg)                 # --perturb-*: the L and R drops only
    shaken = (f" perturb {args.perturb_mm:g} mm {args.perturb_deg:g} deg seed {args.perturb_seed}" if on else "")
    print(f"pairs {args.load} ({args.sampler}, targets {targets_source}): {len(entries)} dishes, {len(pair_list)} "
          f"candidate pairs within {PAIR_M * 1e3:.0f} mm, {len(runs)} drops; landing = {rule}"
          + (f";{shaken} (offset uniform in the disc, yaw uniform)" if on else ""), flush=True)
    if backend is None:
        backend = K._sequence_scene(args, B, entries, snapshot, kit_dir, report)
        if backend is None:
            raise Refused("the extended baseline did not hold")
    park = {i: {"position_m": [3. + .4 * j, 2., .3], "quaternion_xyzw": [0., 0., 0., 1.]} for j, i in enumerate(by_id)}
    limit = LIMITS["peak_penetration_m"]

    def drop(dish, removed, index):
        absent = {dish, *removed}
        for i in ids:                                               # the context: S parked, the rest at the targets
            backend.set_rigid_pose(backend.objects[i], park[i] if i in absent else targets[i])
        for _ in range(K.SETTLE_TICKS):
            backend.tick()
        context = [i for i in ids if i not in absent]
        now = backend.poses()
        pre = {i: P.pose_T(now[i]) for i in context}
        drift = max((float(np.linalg.norm(pre[i][:3, 3] - np.asarray(targets[i]["position_m"], dtype=float)))
                     for i in context), default=0.)
        T_goal = P.pose_T(targets[dish])
        T_cmd = T_goal.copy()
        T_cmd[2, 3] += B.GOAL_HOVER_M
        kind = by_id[dish]["kind"]
        T_drop, off = T_cmd, (0., 0., 0.)
        if on:                                                      # landing stays judged against T_goal and T_cmd
            T_drop, off = perturb(T_cmd, centroids[kind], P.pose_T(frames[by_id[dish]["rack"]])[:3, :3],
                                  args.perturb_mm * 1e-3, args.perturb_deg, args.perturb_seed, index)
        backend.set_rigid_pose(backend.objects[dish], P.pose_dict(T_drop))
        for _ in range(K.SETTLE_TICKS + K.WINDOW_TICKS):
            backend.tick()
        now = backend.poses()
        post = {i: P.pose_T(now[i]) for i in [*context, dish]}
        dist = B.goal_distance(centroids[kind], post[dish], T_goal)
        dev = float(np.linalg.norm(post[dish][:3, 3] - T_cmd[:3, 3]))
        deep = float(max((d for name, d in backend.latest_contact["depths_m"].items() if dish in name.split("|")),
                         default=0.))
        moved = R.disturbed_items({**pre, dish: T_goal}, post, dish)
        return {"dish": dish, "removed": list(removed), "landed": bool(landing(kind, dist, dev)) and deep < limit,
                "lateral_mm": round(dist[0] * 1e3, 1), "dz_mm": round(dist[1] * 1e3, 1), "tilt_deg": round(dist[2], 1),
                "contact_mm": round(deep * 1e3, 2),
                "disturbed": {i: [round(float(np.linalg.norm(post[i][:3, 3] - pre[i][:3, 3])) * 1e3, 1),
                                  round(R.rot_angle_deg(pre[i], post[i]), 1)] for i in moved},
                "context_drift_mm": round(drift * 1e3, 1),
                "dx_mm": round(off[0], 2), "dy_mm": round(off[1], 2), "yaw_deg": round(off[2], 2)}

    tests, t0 = [], time.monotonic()
    for k, (dish, removed) in enumerate(runs, 1):
        t = drop(dish, removed, k)
        tests.append(t)
        print(f"test {k}/{len(runs)} {dish} " + (f"without {removed[0]}" if removed else "last") +
              (f" offset {t['dx_mm']} {t['dy_mm']} mm yaw {t['yaw_deg']} deg" if on else "") +
              f": landed {t['landed']} lateral {t['lateral_mm']} dz {t['dz_mm']} mm tilt {t['tilt_deg']} deg "
              f"contact {t['contact_mm']} mm drift {t['context_drift_mm']} mm disturbed {t['disturbed'] or '-'} "
              f"({time.monotonic() - t0:.0f} s)", flush=True)
    drop_seconds = time.monotonic() - t0
    edges, cycles = graph_edges(tests)
    goal_order = sorted(edges)
    count = {r: sum(r in rs for rs in edges.values()) for r in REASONS}
    waves = layers(ids, goal_order)
    n_waves = max(waves.values()) + 1 if waves is not None else None
    print(f"graph: {len(edges)} edges ({', '.join(f'{r} {n}' for r, n in count.items())}), cycles {cycles or '-'}, "
          f"waves {n_waves if n_waves is not None else 'none'}", flush=True)

    builds, extra, order, poses_world, built_entries, failed = [], [], [], targets, [], None
    if waves is None:
        status = "cycle"
        reason = "no valid one-at-a-time order: " + (
            "two-way pairs " + ", ".join(f"{a} <-> {b}" for a, b in cycles) if cycles
            else "the edges form a cycle through more than two dishes")
    else:
        built, record, _ = K.build_sequence(backend, B, P, by_id, targets, centroids, landing, initial=goal_order)
        builds = record["builds"]
        for k, b in enumerate(builds):
            print(f"build {k + 1}: {b['status']} {b['dish'] or ''} {b['moved'] or ''}", flush=True)
        learned = sorted(tuple(x) for x in record["learned"])
        extra = sorted(set(learned) - set(goal_order))
        order = record.get("order") or builds[-1]["order"]
        if built == "ok":
            status = "ok"
            poses_world = record["poses_world"]
            built_entries = local_entries(P, entries, frames, poses_world)
        else:
            status = "no_order" if len(builds) >= K.SEQUENCE_BUILDS else builds[-1]["status"]
            failed = builds[-1]["dish"]
        reason = (record["reason"] + f"; {len(extra)} constraint(s) learned beyond the graph's {len(goal_order)} edges"
                  + ("; the learned constraints close a cycle" if layers(ids, learned) is None else ""))
    dep = {"load_id": args.load, "sampler": args.sampler, "source": rel(source), "status": status, "reason": reason,
           "pairs_tested": pair_list, "tests": tests,
           "edges": [{"a": a, "b": b, "reasons": rs} for (a, b), rs in edges.items()], "cycles": cycles,
           "goal_order": [list(x) for x in goal_order], "order": order, "builds": builds,
           "extra_constraints": [list(x) for x in extra], "waves": waves, "n_waves": n_waves,
           "acyclic": waves is not None, "failed_dish": failed, "entries": entries, "targets_source": targets_source,
           "targets_world": targets, "poses_world": poses_world, "entries_settled": entries_settled,
           "built_entries": built_entries, "frames": frames, "landing_rule": rule,
           "thresholds": {"at_goal": {k: {"lateral_mm": round(v[0] * 1e3, 3), "dz_mm": round(v[1] * 1e3, 3),
                                          "tilt_deg": v[2]} for k, v in B.AT_GOAL.items()},
                          "disturb_mm": round(R.DISTURB_POS_M * 1e3, 3), "disturb_deg": R.DISTURB_ROT_DEG,
                          "contact_limit_mm": round(limit * 1e3, 3), "hover_mm": round(B.GOAL_HOVER_M * 1e3, 3),
                          "context_ticks": K.SETTLE_TICKS, "drop_ticks": K.SETTLE_TICKS + K.WINDOW_TICKS,
                          "physics_hz": 120, "candidate_mm": round(PAIR_M * 1e3, 3)},
           "perturbation": perturbation(args),
           "gate": gate_info(args, gate) if gate is not None else None, **resettle,
           "kit": {"wall_seconds": round(time.monotonic() - started, 1), "drops": len(tests),
                   "drop_wall_seconds": round(drop_seconds, 1), "builds_run": len(builds), **kit_resettle},
           "generated_utc": datetime.now(timezone.utc).isoformat()}
    write_record(args, out_path, dep)
    report.update(result="PASS", status=status, dependency=rel(out_path),
                  reason=head + f"status {status} pairs {len(pair_list)} drops {len(tests)} edges {len(edges)} "
                         f"(knock {count['knock']} need {count['need']} block {count['block']}) cycles {len(cycles)} "
                         f"builds {len(builds)} extra {len(extra)} waves {n_waves if n_waves is not None else 'none'} "
                         f"targets {targets_source}{shaken}")


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
