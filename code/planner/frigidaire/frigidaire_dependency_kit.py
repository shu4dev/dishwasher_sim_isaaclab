#!/usr/bin/env python3
# Copyright (c) 2026, dishsim project.
# SPDX-License-Identifier: BSD-3-Clause
"""Isaac dependency graph of one 24-dish load by physical cause (boot-first; one load per Kit process).

The method (plans/2026-10-16-physical-cause-graph.md; the overlap rule, plans/2026-10-16-labelled-views-overlap-rule.md;
reference code/frigidaire/docs/dependency.md; the study's rounds 1-6, ordered-pair tests, swap tests, the order library
and the learning loop, are in git history before 2026-10-22): a valid sample is (1) built poses that pass the bench's
joint gate, (2) a witness order that rebuilds all 24 in a fresh Kit process and (3) a graph whose every edge is a
physical cause in the finished load. Per load, one Kit process each:

The sampler (the default run; the packing <restricted-dir>/<load_id>.json of frigidaire_dot_rules_restricted.py): the
packing is built one dish at a time from empty extended racks (the benchmark baseline) by the insertion move,
frigidaire_bench.insertion_release toward the dish's MOVE POSE (a plate upright in its slot, 3 mm toward its packed
lean's side; a bowl or cup at its packed pose): it starts with its lowest collision point 20 mm above its rack's rim wire
centre (its highest collision point under the tub ceiling), descends along the rack's up axis in 5 mm steps, each checked
with the bench checker's 1 mm allowance against every placed dish and the appliance; a placed dish in the way or a blocked
start blocks the move (no release), the appliance alone stops the descent, and the dish is released at rest 20 mm above
the lowest clear step, then settles 150 + 60 ticks. The descent is geometric (no ticks): the first frames of each
recorded trajectory. The order is a seeded random permutation whose plate positions are refilled per bank in chain order,
the slot furthest on the lean side first (an upright plate settles leaning ~30 deg toward its lean side and overhangs the
next 3-4 slots there; frigidaire_bench_kit.chain_order); bowls and cups stay where the permutation put them. A dish goes
in when it lands within DEV_MAX_M of its target with contact under 2 mm, disturbs no placed dish and leaves no pair, among
the placed dishes and against the appliance, over the joint gate's 1 mm preflight; one that does not is undone and
tried at up to REPOSE_TRIES other free poses of its kind with a clear descent (Reposer; a plate along its chain only, its
bank's slots past its own, nearest first), else it is deferred (a plate after the plates still queued, any other dish to
the back of the order); the load fails only when no dish left goes in. Writes <out-dir>/<load_id>.json with procedure
'physical': reposing {seed, order, plate_order, attempts, reposed, repose_calls, entries_chosen, chains}, entries_chosen
and moves_world (the pose and move each dish went in with), the witness {order, status, support, steps, traj, plate_order,
chains: plate -> bank, index, slot, placed_index, placed_slot} and its tick poses _kit/<load_id>/traj/witness.json, the
built poses as entries_settled; or status 'resettle_failed' when the packing did not build.

--manifest-out PATH (Kit-free: run it through run_py.sh, inside the container, since frigidaire_hotec_top5.manifest_of
writes the container's absolute tableware paths): the joint-gate manifest of the record's built poses (entries_settled);
frigidaire_initial_state_validate.py gates them (--peak-penetration-m 0.005, lower first once on closure_failure).

--certify --gate DIR (one fresh Kit process): rebuilds the witness order from empty racks (build_sequence, fixed order, the
at-goal tolerance of the built pose: certified = 24 of 24 land), then: SUPPORT from the rebuild's resting contacts (the net
contact normal force on b from a, sign calibrated on the dish-rack contacts, pointing up by more than FORCE_MIN_N; the
higher-centre rule when the calibration fails), one candidate set per supported dish; ACCESS (geometric, Kit-free in
itself): b before a when a's final pose meets b's own insertion path (insertion_release against a alone and the
appliance, start down to the lowest clear step); CONFIRMATION drops per support candidate set S of b, each b dropped by
the move into the finished load (also removed: the dishes that cover b's path and everything resting, transitively, on b
or a removed support; anything that falls in the re-settle is parked; landed = the at-goal tolerance of b's rebuilt pose,
contact under the limit): without S (it must not land, else 'free'), with each support alone (which ones suffice, kept as
the disjunction), and with several supports, with all of S (the control) and with only one removed. Only HARD
requirements become edges (coordinator, 2026-10-16): a single support when b fails without it and lands with it; one of
several when b fails without that one alone. Graph (physical_graph): hard supports and every access hit; a pair with arcs
both ways loses both (conflicts), an edge the witness breaks is not drawn (against_witness), a dish in no edge is free,
layers for the DAG drawing. Writes gate, certify, contacts, access, confirmations, graph, valid into the record and the
drops' trajectories to _kit/<load_id>/traj/ (certify_witness, conf_<b>_<none|a|all|no_a>).

--sample (host python on the box; physical_loop) schedules it all over the packings of
data/logs/dot_rules/sequential_fresh_order.txt; --resume-from DIR imports the built samples of an earlier run first.

Per load, from the repo root (<id> e.g. r05_00027, <dir> the run's folder); the scheduler runs exactly these:
    code/util/run_kit.sh code/planner/frigidaire/frigidaire_dependency_kit.py --load <id> --out-dir <dir> --headless
    code/util/run_py.sh code/planner/frigidaire/frigidaire_dependency_kit.py --load <id> --out-dir <dir> --manifest-out <dir>/_gate/<id>/manifest.json
    code/util/run_kit.sh code/initialization/frigidaire/frigidaire_initial_state_validate.py --manifest <dir>/_gate/<id>/manifest.json --out-dir <dir>/_gate/<id> --max-wall-seconds 600 --peak-penetration-m 0.005 --headless --device cpu
    code/util/run_kit.sh code/planner/frigidaire/frigidaire_dependency_kit.py --load <id> --out-dir <dir> --certify --gate <dir>/_gate/<id> --headless
The whole loop (stdlib only): python3 code/planner/frigidaire/frigidaire_dependency_kit.py --sample --out-dir <dir>
--target-valid 5 --max-packings 20 --max-wall-seconds 10800 [--resume-from <earlier dir>].

Writes the <load_id>.json above and <out-dir>/_kit/<load_id>/report.json (report_certify.json for --certify).
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import random
import sys
import time
import traceback

ROOT = Path(__file__).resolve().parents[3]
RACK_FRAMES = ("LowerRack", "UpperRack", "SilverwareBasket")
FALL_ROUNDS = 4                   # confirmation re-settles: the fallen are parked and the rest re-settled, at most 4 rounds
LANDING_TAIL = ("contact depth < 2 mm; released at the lowest clear hover; no placed dish at its goal ends outside its "
                "at-goal tolerance (a placed dish not at its goal: moved > 10 mm / 20 deg)")
TRAJ_EVERY, PARK_X = 4, 2.        # tick poses every 4 ticks; a dish beyond x = 2 m is parked
REPOSE_TRIES = 5                  # the sampler: other free poses with a clear descent tried per failed dish
FORCE_MIN_N = .05                 # --certify: a dish-dish contact force whose up component exceeds 0.05 N is support
CALIBRATE_MIN = .8                # its sign: >= 80 % of the dish-rack contacts must push their dish up, else the centre rule
GATE_PEAK_M = .005                # the joint gate's peak penetration (frigidaire_hotec_top5.gate_unit's flag)


class Refused(Exception):
    """An expected stop (no traceback): a baseline that does not hold, a record that is not there."""


def rel(path):
    p = Path(path).absolute()
    try:
        return str(p.relative_to(ROOT))
    except ValueError:
        return str(p)


def plain_kind(kind):
    return kind[6:] if kind.startswith("hotec_") else kind


def read_load(args):
    """(record path, entries) of the restricted packing: rack-local entries (racks out = the rack body frame), kinds
    plate | bowl | cup."""
    path = Path(args.restricted_dir) / f"{args.load}.json"
    data = json.loads(path.read_text())
    entries = [{"id": e["id"], "kind": plain_kind(e["kind"]), "rack": e["rack"], "slot": e.get("slot"),
                "variant": e.get("variant"), "position": [float(v) for v in e["position"]],
                "quaternion_xyzw": [float(v) for v in e["quaternion_xyzw"]], "keep": bool(e.get("keep", False))}
               for e in data["entries"]]
    return path, entries


def local_entries(P, entries, frames, poses):
    """``entries`` at the world ``poses``, rack-local in the rack ``frames`` (the score script's input)."""
    out = []
    for e in entries:
        local = P.local_from_world(frames[e["rack"]], poses[e["id"]])
        out.append({**e, "position": local["position_m"], "quaternion_xyzw": local["quaternion_xyzw"]})
    return out


def write_record(args, out_path, dep):
    if out_path.exists() and not args.force:
        raise Refused(f"refusing to overwrite {rel(out_path)} (--force)")
    out_path.parent.mkdir(parents=True, exist_ok=True)
    tmp = out_path.with_name(out_path.name + ".tmp")
    tmp.write_text(json.dumps(dep, indent=1, default=lambda o: o.tolist() if hasattr(o, "tolist") else str(o)) + "\n")
    os.replace(tmp, out_path)


def goal_setup(args, entries, B, P):
    """(baseline snapshot, its rack frames, {id: world target pose}): the packing's commanded poses in the baseline's
    extended racks. Kit-free."""
    base = json.loads(Path(args.baseline).read_text())
    snapshot = base.get("snapshot", base)
    frames = {r: snapshot["poses"][r] for r in RACK_FRAMES}
    targets = {e["id"]: P.world_from_local(frames[e["rack"]], {"position_m": e["position"],
                                                               "quaternion_xyzw": e["quaternion_xyzw"]}) for e in entries}
    return snapshot, frames, targets


def goal_delta(B, P, entries, centroids, built, targets):
    """{per_dish: {id: {lateral_mm, dz_mm, tilt_deg}}, max: the same over dishes} from the commanded target to the built
    pose of every dish (B.goal_distance, as the drops judge landing)."""
    per = {}
    for e in entries:
        d = B.goal_distance(centroids[e["kind"]], P.pose_T(built[e["id"]]), P.pose_T(targets[e["id"]]))
        per[e["id"]] = {"lateral_mm": round(d[0] * 1e3, 1), "dz_mm": round(d[1] * 1e3, 1), "tilt_deg": round(d[2], 1)}
    return {"per_dish": per, "max": {k: max(v[k] for v in per.values()) for k in ("lateral_mm", "dz_mm", "tilt_deg")}}


def release_info(B):
    """The record's release: the insertion move (2026-10-15 b)."""
    return {"mode": "insertion", "version": "2026-10-15b", "step_mm": round(B.INSERT_STEP_M * 1e3, 3),
            "above_rim_mm": round(B.INSERT_ABOVE_RIM_M * 1e3, 3), "gap_mm": round(B.INSERT_GAP_M * 1e3, 3),
            "lean_bias_mm": round(B.INSERT_LEAN_BIAS_M * 1e3, 3),
            "move_pose": "a plate upright in its slot (lean 0, the gap's mid x) moved 3 mm toward its packed lean's side; "
                         "a bowl or cup at its packed pose",
            "start": "the move pose raised until its lowest collision point is 20 mm above the rack's rim wire centre, "
                     "its highest collision point capped under the tub ceiling (geometry.py TubCeiling), never below "
                     "the move pose",
            "check": "each 5 mm step down passes the bench checker's 1 mm allowance against all placed dishes and the "
                     "appliance; a placed dish in the way (or a blocked start) blocks the move, the appliance alone "
                     "stops the descent",
            "release": "at rest 20 mm above the lowest clear pose (never above the start), then 150 + 60 ticks"}


def tolerance_of(B, limit=.002):
    """(at-goal table, contact-depth limit m, text) of the bench tolerance (the records' "tolerance": "bench"):
    B.AT_GOAL with the gates' contact limit ``limit`` (LIMITS, 2 mm)."""
    text = ", ".join(f"{k} {v[0] * 1e3:g} mm lateral / {v[1] * 1e3:g} mm dz / {v[2]:g} deg" for k, v in B.AT_GOAL.items())
    return B.AT_GOAL, limit, text


def landing_tail(rinfo, contact_m=None):
    """LANDING_TAIL with the record's insertion move (and, given, its contact-depth limit)."""
    tail = LANDING_TAIL if contact_m is None else LANDING_TAIL.replace("< 2 mm", f"< {contact_m * 1e3:g} mm")
    return tail.replace("released at the lowest clear hover", (
        f"lowered from {rinfo['above_rim_mm']:g} mm above the rim in {rinfo['step_mm']:g} mm steps (plates upright) "
        f"until the appliance stops it, a placed dish in the way blocking it, released {rinfo['gap_mm']:g} mm above that"))


def parked(pose):
    return float(pose["position_m"][0]) > PARK_X


class Recorder:
    """Every dish's world pose (x y z qx qy qz qw, flat per frame) each TRAJ_EVERY ticks while a segment is open,
    written to <folder>/<name>.json (with ``empty_resets`` the segment restarts whenever every dish is parked: only the
    last build is kept). Wraps backend.tick."""

    def __init__(self, backend, ids, folder):
        self.ids, self.folder, self.frames, self.empty_resets = list(ids), Path(folder), None, False
        self.backend = backend
        tick = backend.tick

        def recorded_tick():
            tick()
            now = backend.previous_frames
            if self.frames is not None and backend.tick_index % TRAJ_EVERY == 0:
                if self.empty_resets and all(parked(now[i]) for i in self.ids):
                    self.frames = []
                self.frames.append([round(float(v), 5) for i in self.ids
                                    for v in (*now[i]["position_m"], *now[i]["quaternion_xyzw"])])
        backend.tick = recorded_tick

    def begin(self, empty_resets=False):
        self.frames, self.empty_resets = [], empty_resets

    def mark(self):
        """The open segment's length (build_sequence's ``trace``: an undone attempt is cut back to it)."""
        return len(self.frames) if self.frames is not None else 0

    def cut(self, n):
        if self.frames is not None and len(self.frames) > n:
            del self.frames[n:]

    def descent(self, dishes, steps, now=None):
        """The insertion move: one frame per descent step appended to the open segment, ``dishes`` at that step's
        poses (``steps`` = [[pose per dish] per step], start first), every other dish where it is ``now`` (default: the
        last tick). The descent is geometric (no ticks), so these frames are its only trace."""
        if self.frames is None:
            return
        now = now or self.backend.previous_frames
        for poses in steps:
            at = {**now, **dict(zip(dishes, poses))}
            self.frames.append([round(float(v), 5) for i in self.ids
                                for v in (*at[i]["position_m"], *at[i]["quaternion_xyzw"])])

    def save(self, name, poses=None, **meta):
        """Write the open segment (or, with ``poses``, that one frame) as <name>.json; returns the file name."""
        frames = self.frames or []
        if poses is not None:
            frames = [[round(float(v), 5) for i in self.ids
                       for v in (*poses[i]["position_m"], *poses[i]["quaternion_xyzw"])]]
        self.frames, self.empty_resets = None, False
        self.folder.mkdir(parents=True, exist_ok=True)
        (self.folder / f"{name}.json").write_text(json.dumps({"ids": self.ids, "every": TRAJ_EVERY, "hz": 120,
                                                             "frames": frames, **meta}) + "\n")
        return f"{name}.json"


def seed_of(*parts):
    """A stable 32-bit seed from strings (the sampler's order and re-posing draws per load)."""
    import hashlib
    return int(hashlib.sha256("|".join(map(str, parts)).encode()).hexdigest()[:8], 16)


class Reposer:
    """build_sequence's ``repose`` for the sampler (plans/2026-10-15-reposing-sampler-five-samples.md,
    Kit-free): up to REPOSE_TRIES other poses of the dish's kind from the packing's candidate list (the restricted
    sampler's: frigidaire_hotec_top5 level-0 families, masks and gate bans), drawn in a seeded random order, each FREE as
    the packing defines it, against the dishes placed so far (free_iter: no collision past the 0.8 mm touch with the
    placed dishes at their settled poses, one plate or cup per slot among them, no gate ban; the unplaced dishes' poses
    are not reserved, so a dish whose pose gets taken is re-posed in its turn) and with a CLEAR DESCENT under the
    insertion move against the placed dishes at their settled poses and the appliance (no placed dish blocks it, the
    start is clear). A plate's move depends only on its slot, so a plate gets at most one candidate per slot, never its
    own slot. ``calls`` keeps one line per call (dish, placed, screened, found, seconds). ``chain`` (the lean-side order,
    plans/2026-10-16-lean-side-plate-order.md: entry -> (bank, rank), frigidaire_bench_kit.plate_bank): a plate walks its
    chain instead, only the slots of its own bank past its own along the chain, nearest first (a random variant per slot)."""

    def __init__(self, B, P, frames, seed, chain=None):
        import importlib.util
        spec = importlib.util.spec_from_file_location("top5_repose", ROOT / "code/planner/frigidaire/frigidaire_hotec_top5.py")
        T = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(T)    # its private frigidaire_bench copy: TolerantDishSet, nesting allowed (the packing's rules)
        self.T, self.B, self.P, self.frames, self.chain = T, B, P, frames, chain
        self.fam, self.parts, self.points = T.level_families(0)
        self.masks = T.level_masks(self.fam, self.parts, self.points, 0)
        self.bans = T.all_bans()
        self.rng, self.calls = random.Random(seed), []

    def target_move(self, entry):
        """(world target, world move pose) of a rack-local entry."""
        f = self.frames[entry["rack"]]
        target = self.P.world_from_local(f, {"position_m": list(entry["position"]),
                                             "quaternion_xyzw": list(entry["quaternion_xyzw"])})
        return target, self.P.world_from_local(f, self.B.move_local(entry))

    def __call__(self, dish, entries, placed, now):
        B, T, P, t0 = self.B, self.T, self.P, time.monotonic()
        own = entries[dish]
        kind, others = own["kind"], []
        for i in placed:                                     # the placed dishes at their settled poses, rack-local
            e = entries[i]
            local = P.local_from_world(self.frames[e["rack"]], now[i])
            others.append({**e, "position": local["position_m"], "quaternion_xyzw": local["quaternion_xyzw"]})
        dishes = T.TolerantDishSet(self.parts, self.points)
        for e in others:
            dishes.add(e["id"], e)
        used = {(e["rack"], e["slot"]) for e in others}
        checker = B.release_checker()
        appliance = B.appliance_bodies(checker, now)
        bodies = [checker._body(entries[i]["kind"], now[i], i) for i in placed]
        heights = {r: B.rack_heights(now, r) for r in ("LowerRack", "UpperRack")}
        seen, out, screened = ({own["slot"]} if kind == "plate" else set()), [], 0
        index = list(range(len(self.fam[kind])))
        self.rng.shuffle(index)
        walk = self.chain is not None and kind == "plate"
        if walk:                                             # its bank's slots past its own, nearest first (stable sort:
            bank, rank = self.chain(own)                     # the shuffled variants stay random within a slot)
            ranks = {k: self.chain(self.fam[kind][k][0]) for k in index}
            index = sorted((k for k in index if ranks[k][0] == bank and ranks[k][1] > rank), key=lambda k: ranks[k][1])
        for k in index:
            if len(out) >= REPOSE_TRIES:
                break
            c = self.fam[kind][k][0]
            if (not self.masks[kind][k] or (kind != "bowl" and (c["rack"], c["slot"]) in used)
                    or (kind == "plate" and c["slot"] in seen) or B.cand_key(c) == B.cand_key(own)
                    or T.B.banned(c, others, self.bans) or dishes.collides(c, skip=(dish,))):
                continue
            seen.add(c["slot"])
            entry = T.B.entry_of(dish, c)
            target, move = self.target_move(entry)
            up, rim_z, top_z = heights[c["rack"]]
            screened += 1
            if B.insertion_release(checker, [(kind, move, dish)], bodies, appliance, up, rim_z, top_z)["poses"] is None:
                continue
            out.append({"entry": entry, "target": target, "move": move})
        self.calls.append({"dish": dish, "placed": len(placed), "screened": screened, "found": len(out),
                           "seconds": round(time.monotonic() - t0, 2), **({"chain": True} if walk else {})})
        print(f"  repose {dish} ({len(placed)} placed{', chain walk' if walk else ''}): {len(out)} clear of {screened} free poses screened "
              f"({time.monotonic() - t0:.1f} s): " + ", ".join(f"{o['entry']['slot']} {o['entry']['variant']}" for o in out),
              flush=True)
        return out


def sampler(args, source, entries, out_path, kit_dir, report, started):
    """The re-posing sampler (module docstring): the packing built one dish at a time, the record written with procedure
    'physical' (gate, certify and valid still None), or status 'resettle_failed' when no dish left goes in."""
    import frigidaire_bench as B
    import frigidaire_bench_kit as K
    from dishsim_frigidaire import planner as P
    from dishsim_frigidaire.loading import visual_points
    from dishsim_frigidaire.random_poses import LIMITS

    snapshot, frames, targets = goal_setup(args, entries, B, P)
    ids = [e["id"] for e in entries]
    by_id = {e["id"]: e for e in entries}
    centroids = B.kind_centroids({kind: visual_points(B.ASSETS / f"{kind}.usda") for kind in B.KINDS})
    rinfo = release_info(B)
    _, climit, tol_text = tolerance_of(B, LIMITS["peak_penetration_m"])
    backend = K._sequence_scene(args, B, entries, snapshot, kit_dir, report)
    if backend is None:
        raise Refused("the extended baseline did not hold")
    rec = Recorder(backend, ids, kit_dir / "traj")                 # the last build's ticks kept: the witness clip
    rec.begin(empty_resets=True)
    own = (f"own load: settle deviation <= {K.DEV_MAX_M * 1e3:.0f} mm of the commanded pose (the target, not the "
           f"release pose), {landing_tail(rinfo)}")
    print(f"resettle {args.load} (restricted, targets commanded): {len(entries)} dishes one at a time; landing = {own}",
          flush=True)
    t_resettle = time.monotonic()
    seed = seed_of("reposing", args.load)
    order0 = list(ids)
    random.Random(seed).shuffle(order0)
    order0, chains = K.chain_order(order0, by_id, B.lean_side)  # the same permutation, each bank's plates in chain order
    moves = {i: P.world_from_local(frames[by_id[i]["rack"]], B.move_local(by_id[i])) for i in ids}
    reposer = Reposer(B, P, frames, seed + 1, chain=lambda e: K.plate_bank(e, B.lean_side))
    links = ", ".join(f"{i} {c['bank']} {c['index']}" for i, c in chains.items())
    print(f"re-posing sampler: candidates ready ({time.monotonic() - t_resettle:.0f} s); lean-side order "
          f"{' '.join(order0)}" + (f"; chains {links}" if chains else ""), flush=True)
    built, record, _ = K.build_sequence(backend, B, P, by_id, targets, centroids,
                                        lambda kind, dist, dev: dev <= K.DEV_MAX_M, release="insertion",
                                        on_path=rec.descent, moves=moves, order=order0, repose=reposer, trace=rec,
                                        chains=chains)
    for a in record["attempts"]:
        s = a["step"]
        print(f"  attempt {a['dish']} try {a['try']} {a['slot']} {a['variant']} ({a['placed']} placed): "
              f"{a['status']}" + (f" blocked ({s.get('blocked')}) at {s.get('blocked_lift_mm')} mm by "
                                  f"{s.get('blockers')}" if s.get("no_clear_release") else
                                  f" dev {s.get('settle_dev_mm')} mm contact {s.get('contact_mm')} mm stop "
                                  f"{s.get('stop')} {s.get('lowest_lift_mm')} mm moved {a['moved'] or '-'}"
                                  + (f" overlap {s['overlap']}" if s.get("overlap") else "")),
              flush=True)
    moves, chosen = record["moves"], [record["entries"][i] for i in ids]
    sampled = {"seed": seed, "order": order0, "plate_order": "lean-side", "attempts": record["attempts"],
               "reposed": record["reposed"], "repose_calls": reposer.calls, "entries_chosen": chosen}
    if chains:                    # per plate: its bank, chain index, slot planned, slot and place in its bank built
        ok, per = {a["dish"] for a in record["attempts"] if a["status"] == "ok"}, {}
        for i in record["builds"][-1]["order"]:             # the placed dishes in placement order
            if i in chains and i in ok:
                chains[i]["placed_index"] = per.get(chains[i]["bank"], 0)
                per[chains[i]["bank"]] = chains[i]["placed_index"] + 1
        for i, c in chains.items():
            c["placed_slot"] = record["entries"][i]["slot"] if "placed_index" in c else None
        sampled["chains"] = chains
    for k, b in enumerate(record["builds"], 1):
        print(f"resettle build {k}: {b['status']} {b['dish'] or ''} {b['moved'] or ''}", flush=True)
    resettle = {"goal_source": "sequential_build", "goal_delta": None, "resettle_builds": record["builds"],
                "resettle_rule": own, "reposing": sampled, "moves_world": moves, "entries_chosen": chosen}
    kit_resettle = {"resettle_wall_seconds": round(time.monotonic() - t_resettle, 1),
                    "resettle_builds_run": len(record["builds"])}
    last = record["builds"][-1]
    resettle["witness"] = {"order": record.get("order") or last["order"], "status": last["status"],
                           "support": [list(x) for x in record.get("support", [])], "steps": last["steps"],
                           "traj": rec.save("witness", order=last["order"], status=last["status"]),
                           "plate_order": sampled["plate_order"], "chains": sampled.get("chains")}
    if built != "ok":                                       # a record without a goal: why the packing did not build
        reason = "the goal did not build one dish at a time: " + record["reason"]
        dep = {"load_id": args.load, "sampler": "restricted", "source": rel(source), "status": "resettle_failed",
               "reason": reason, "entries": entries, "targets_source": "commanded", "targets_world": targets,
               "entries_settled": [], "frames": frames, "failed_dish": last["dish"], "landing_rule": None, "gate": None,
               **resettle, "release": rinfo, "kit": {"wall_seconds": round(time.monotonic() - started, 1), **kit_resettle},
               "generated_utc": datetime.now(timezone.utc).isoformat()}
        write_record(args, out_path, dep)
        report.update(result="PASS", status="resettle_failed", dependency=rel(out_path),
                      reason=f"status resettle_failed {reason}")
        return
    resettle["goal_delta"] = goal_delta(B, P, entries, centroids, record["poses_world"], record["targets"])
    top = resettle["goal_delta"]["max"]
    head = (f"release insertion resettle builds {len(record['builds'])} goal delta max lateral {top['lateral_mm']} "
            f"dz {top['dz_mm']} mm tilt {top['tilt_deg']} deg ")
    print(f"goal built from the commanded pose: {head}({kit_resettle['resettle_wall_seconds']:.0f} s)", flush=True)
    targets = record["poses_world"]
    rule = f"at-goal tolerance of the sequentially built pose (bench: {tol_text}), {landing_tail(rinfo, climit)}"
    dep = {"load_id": args.load, "sampler": "restricted", "source": rel(source), "procedure": "physical",
           "status": "ok", "reason": f"goal built one dish at a time in {len(resettle['resettle_builds'])} build(s) by "
           "the re-posing sampler", "release": rinfo, **resettle, "entries": entries,
           "targets_source": "sequential_build", "targets_world": targets,
           "entries_settled": local_entries(P, chosen, frames, targets), "frames": frames, "landing_rule": rule,
           "tolerance": "bench", "gate": None, "certify": None, "valid": None,
           "kit": {"wall_seconds": round(time.monotonic() - started, 1), **kit_resettle},
           "generated_utc": datetime.now(timezone.utc).isoformat()}
    write_record(args, out_path, dep)
    report.update(result="PASS", status="ok", dependency=rel(out_path), reason=head + "status ok (sampler only)")


SAMPLE_STAGGER_S, SAMPLE_PILOT = 10., "r07_00012"
KIT_CORES, PY_CORES = tuple(range(14)), (14, 15)


def judge_log(path):
    """(ok, text) of a job log: [RESULT] PASS, no Traceback, no free(): invalid pointer (the exit code says nothing)."""
    text = path.read_text(errors="replace") if path.is_file() else ""
    return ("[RESULT] PASS" in text and "Traceback" not in text and "free(): invalid pointer" not in text), text


def write_manifest(args):
    """--manifest-out: the joint-gate manifest of the record's built poses (entries_settled, rack-local in the
    baseline's extended racks), frigidaire_hotec_top5.manifest_of (the benchmark baseline = the default --baseline, the
    HOTEC v2 tableware). Kit-free: isaaclab is never imported."""
    import importlib.util
    spec = importlib.util.spec_from_file_location("top5", ROOT / "code/planner/frigidaire/frigidaire_hotec_top5.py")
    T = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(T)    # Kit-free at import: numpy, dishsim and a private copy of frigidaire_bench
    path = args.out_dir / f"{args.load}.json"
    dep = json.loads(path.read_text()) if path.is_file() else {}
    if dep.get("status") != "ok" or not dep.get("entries_settled"):
        print(f"[RESULT] FAIL {args.load} no built sampler record at {rel(path)}", flush=True)
        return
    args.manifest_out.parent.mkdir(parents=True, exist_ok=True)
    args.manifest_out.write_text(json.dumps(T.manifest_of(dep["entries_settled"]), indent=1) + "\n")
    print(f"[RESULT] PASS manifest {rel(args.manifest_out)}", flush=True)


# --------------------------------------------------------------------------- --certify: the physical graph

class ForceLog:
    """Per ordered body pair (sensor, filter) of the backend's contact view, the net contact normal force sum(f n) of its
    rows at the last tick they were reported (world N, sign convention as PhysX reports it; support_candidates calibrates
    it): wraps the backend's contact tracker. A sleeping pair is no longer reported, and the tracker keeps it only while
    neither surface moved, so a resting pair's last rows are its resting force."""

    def __init__(self, backend):
        import numpy as np
        self.raw = {}
        tracker = backend.tracker
        names, update = tracker.names, tracker.update

        def logged(data, poses):
            out = update(data, poses)
            arr = [v.detach().cpu().numpy() if hasattr(v, "detach") else np.asarray(v) for v in data]
            f, n = arr[0].reshape(-1), arr[2].reshape(-1, 3)
            counts = arr[4].astype(np.int64).reshape(len(names), -1)
            starts = arr[5].astype(np.int64).reshape(len(names), -1)
            for i, j in np.argwhere(counts > 0):
                if i != j:
                    b, c = starts[i, j], counts[i, j]
                    self.raw[(names[i], names[j])] = (f[b:b + c, None] * n[b:b + c]).sum(axis=0)
            return out
        tracker.update = logged


def support_candidates(raw, pairs, poses, ids, racks=("LowerRack", "UpperRack", "SilverwareBasket")):
    """(supports {b: [a...]}, contacts [{pair, on_b_from_a_n, on_a_from_b_n}], calibration) of the finished load, standard
    library only. ``raw`` = ForceLog.raw, ``pairs`` = the resting contact pairs (sorted name tuples), ``poses`` = the final
    world poses. The sign that turns a row into the force ON its sensor is calibrated on the dish-rack contacts (a rack
    pushes its dishes up): calibration {sign, agree, n, rule}; with no such contact or fewer than CALIBRATE_MIN agreeing,
    rule 'centre' (planner.support_edges: the dish whose centre is 5 mm higher rests on the other) decides instead of the
    force. Rule 'force': a supports b when the force on b from a has an up (world z) component above FORCE_MIN_N."""
    dish = set(ids)
    votes = [float(v[2]) for (i, j), v in raw.items() if i in dish and j in racks and tuple(sorted((i, j))) in pairs]
    votes += [-float(v[2]) for (i, j), v in raw.items() if j in dish and i in racks and tuple(sorted((i, j))) in pairs]
    sign = 1. if sum(votes) >= 0 else -1.
    agree = sum(sign * v > 0 for v in votes) / len(votes) if votes else 0.
    cal = {"sign": sign, "agree": round(agree, 3), "n": len(votes),
           "rule": "force" if votes and agree >= CALIBRATE_MIN else "centre"}

    def force_on(b, a):                                       # on b from a [N]
        if (b, a) in raw:
            return [sign * float(x) for x in raw[(b, a)]]
        if (a, b) in raw:
            return [-sign * float(x) for x in raw[(a, b)]]
        return None
    supports, contacts = {i: [] for i in ids}, []
    for a, b in sorted(p for p in pairs if p[0] in dish and p[1] in dish and p[0] != p[1]):
        fb, fa = force_on(b, a), force_on(a, b)
        contacts.append({"pair": [a, b], "on_b_from_a_n": fb and [round(x, 4) for x in fb],
                         "on_a_from_b_n": fa and [round(x, 4) for x in fa]})
        if cal["rule"] == "force":
            if fb is not None and fb[2] > FORCE_MIN_N:
                supports[b].append(a)
            if fa is not None and fa[2] > FORCE_MIN_N:
                supports[a].append(b)
        else:                                                 # planner.SUPPORT_MIN_DZ_M = 5 mm
            dz = float(poses[b]["position_m"][2]) - float(poses[a]["position_m"][2])
            if dz > .005:
                supports[b].append(a)
            elif -dz > .005:
                supports[a].append(b)
    return {b: sorted(s) for b, s in supports.items() if s}, contacts, cal


def access_hits(B, checker, kinds, racks, moves, poses, ids):
    """(hits, paths) of the finished load (Kit-free, FCL): per dish a, its insertion path (insertion_release against the
    appliance alone at the final ``poses``: the start down to the lowest clear step), then every other dish b whose final
    pose fails the checker's 1 mm allowance against a at some step: hit {a, b, step, steps, lift_mm, start_mm, pose (a's
    world pose at the first such step)}, i.e. a must go in before b. paths {a: start_mm, steps, stop, blocked}."""
    appliance = B.appliance_bodies(checker, poses)
    bodies = {i: checker._body(kinds[i], poses[i], i) for i in ids}
    hits, paths = [], {}
    for a in ids:
        up, rim_z, top_z = B.rack_heights(poses, racks[a])
        m = B.insertion_release(checker, [(kinds[a], moves[a], a)], [], appliance, up, rim_z, top_z)
        clear = [] if m["blocked"] == "start" else m["lifts"][:-1] if m["blockers"] else m["lifts"]
        paths[a] = {"start_mm": round(m["start"] * 1e3, 1), "steps": len(clear), "stop": m["stop"],
                    "blocked": m["blocked"], "lowest_mm": None if m["lowest"] is None else round(m["lowest"] * 1e3, 1)}
        left = [b for b in ids if b != a]
        for k, h in enumerate(clear):
            pose = B.lifted(moves[a], h, up)
            body = checker._body(kinds[a], pose, a)
            for b in list(left):
                if not checker.pair(body, bodies[b])["valid"]:
                    hits.append({"a": a, "b": b, "step": k + 1, "steps": len(clear), "lift_mm": round(h * 1e3, 1),
                                 "start_mm": paths[a]["start_mm"], "pose": pose})
                    left.remove(b)
            if not left:
                break
    return hits, paths


def physical_graph(*a, **k): from sampler.graph import physical_graph as f; return f(*a, **k)   # noqa: E704  moved: code/src/sampler/graph.py (lazy: code/src joins sys.path after AppLauncher)


def physical_gate(gate_dir, dep):
    """The record's joint-gate numbers from ``gate_dir``/result.json (frigidaire_initial_state_validate.py on the built
    poses); a lower-first rerun also names the upper-first outcome. Standard library only."""
    path = Path(gate_dir) / "result.json"
    if not path.is_file():
        raise Refused(f"no joint gate result {rel(path)}")
    g = json.loads(path.read_text())
    planned = {o["object_id"]: [round(float(v), 6) for v in o["rack_local_pose"]["position_m"]]
               for o in g.get("candidate_objects") or []}
    if planned and planned != {e["id"]: [round(float(v), 6) for v in e["position"]] for e in dep["entries_settled"]}:
        raise Refused(f"the joint gate {rel(gate_dir)} ran on other poses than this record's built goal")
    mm = lambda v: None if v is None else round(float(v) * 1e3, 2)                 # noqa: E731
    settled, lim = g.get("settled") or {}, g.get("limits") or {}
    out = {"dir": rel(gate_dir), "outcome": g.get("outcome"), "reason": g.get("reason") or g.get("error"),
           "order": g.get("order"), "peak_limit_mm": mm(lim.get("peak_penetration_m")),
           "median_limit_mm": mm(lim.get("median_max_penetration_m")),
           "settle_peak_mm": mm(g.get("maximum_settle_penetration_m")),
           "cycle_peak_mm": mm(g.get("maximum_cycle_penetration_m")),
           "window_peak_mm": mm(settled.get("peak_penetration_m")),
           "window_median_mm": mm(settled.get("median_max_penetration_m")),
           "contained": (all(v["contained"] for v in g["final_containment"].values())
                         if g.get("final_containment") else None),
           "wall_seconds": round(float(g.get("wall_seconds") or 0.), 1)}
    first = Path(str(gate_dir)[:-len("_lower_first")]) / "result.json" if str(gate_dir).endswith("_lower_first") else None
    if first is not None and first.is_file():
        f = json.loads(first.read_text())
        out["first_try"] = {"order": f.get("order"), "outcome": f.get("outcome"), "reason": f.get("reason")}
    return out


def physical_certify(args, source, entries, out_path, kit_dir, report, started):
    """--certify (module docstring): the record's gate, the fresh-process rebuild of its witness, the support candidates
    from the rebuild's resting contact forces, the access hits, the support confirmation drops and the graph, all written
    into the record."""
    import numpy as np
    import frigidaire_bench as B
    import frigidaire_bench_kit as K
    from dishsim_frigidaire import planner as P
    from dishsim_frigidaire.loading import visual_points
    from dishsim_frigidaire.random_poses import LIMITS
    dep = json.loads(out_path.read_text()) if out_path.is_file() else {}
    if dep.get("procedure") != "physical" or dep.get("status") != "ok":
        raise Refused(f"no built sampler record at {rel(out_path)}")
    if not args.gate:
        raise Refused("--certify needs --gate DIR (the joint gate of the built poses)")
    args.force = True
    dep["gate"] = ginfo = physical_gate(args.gate, dep)
    if ginfo["outcome"] != "accepted":
        dep.update(valid=False, verdict={"stage": "invalid", "reason": f"joint gate {ginfo['outcome']}: {ginfo['reason']}"})
        write_record(args, out_path, dep)
        report.update(result="PASS", status="gate", dependency=rel(out_path), reason=dep["verdict"]["reason"])
        return
    tol_name = "bench"
    table, climit, tol_text = tolerance_of(B, LIMITS["peak_penetration_m"])
    rule = f"at-goal tolerance of the sequentially built pose ({tol_name}: {tol_text}), {landing_tail(dep['release'], climit)}"
    snapshot = goal_setup(args, entries, B, P)[0]
    chosen = dep["entries_chosen"]
    by_id = {e["id"]: e for e in chosen}
    ids, order, moves = list(by_id), list(dep["witness"]["order"]), dep["moves_world"]
    kind, rack = {i: by_id[i]["kind"] for i in ids}, {i: by_id[i]["rack"] for i in ids}
    centroids = B.kind_centroids({k: visual_points(B.ASSETS / f"{k}.usda") for k in B.KINDS})
    backend = K._sequence_scene(args, B, chosen, snapshot, kit_dir / "certify_scene", report)
    if backend is None:
        raise Refused("the extended baseline did not hold")
    forces = ForceLog(backend)
    rec = Recorder(backend, ids, kit_dir / "traj")
    t0 = time.monotonic()
    print(f"certify {args.load}: gate {ginfo['outcome']} ({ginfo['order']}, settle peak {ginfo['settle_peak_mm']} mm, cycle "
          f"peak {ginfo['cycle_peak_mm']} mm); rebuilding the witness {' '.join(order)}; landing = {rule}", flush=True)
    rec.begin(empty_resets=True)
    status, rb, _ = K.build_sequence(backend, B, P, by_id, dep["targets_world"], centroids,
                                     lambda k_, dist, dev: B.within_goal(k_, dist, table), fixed=order,
                                     release="insertion", on_path=rec.descent, moves=moves)
    final = backend.poses()
    last = rb["builds"][-1]
    certified = status == "ok"
    cert = {"status": status, "certified": certified, "dish": last["dish"], "moved": last["moved"],
            "placed": len(last["steps"]) - (1 if last["dish"] else 0), "reason": rb["reason"], "steps": last["steps"],
            "traj": rec.save("certify_witness", order=order, status=status), "tolerance": tol_name,
            "poses_world": {i: final[i] for i in ids} if certified else None,
            "rebuild_seconds": round(time.monotonic() - t0, 1)}
    print(f"certify {args.load}: rebuild {status} {last['dish'] or ''} {last['moved'] or ''} ({cert['placed']} of "
          f"{len(ids)} landed, {cert['rebuild_seconds']:.0f} s): {rb['reason']}", flush=True)
    thresholds = {"tolerance": tol_name, "contact_limit_mm": round(climit * 1e3, 3),
                  "at_goal": {k: {"lateral_mm": round(v[0] * 1e3, 3), "dz_mm": round(v[1] * 1e3, 3), "tilt_deg": v[2]}
                              for k, v in table.items()}, "force_min_n": FORCE_MIN_N}
    dep.update(certify=cert, landing_rule=rule, thresholds=thresholds)
    if not certified:
        dep.update(valid=False, verdict={"stage": "invalid", "reason": (
            f"the witness did not rebuild in a fresh Kit process: {status} at {last['dish']}"
            + (f" (moved {', '.join(last['moved'])})" if last["moved"] else "") + f", {cert['placed']} of {len(ids)} landed")})
        write_record(args, out_path, dep)
        report.update(result="PASS", status="not_certified", dependency=rel(out_path), reason=dep["verdict"]["reason"])
        return
    pairs = set()
    for _ in range(30):                                      # the resting contacts (they flicker: a union over 0.25 s)
        backend.tick()
        pairs |= {tuple(p) for p in backend.latest_contact["pairs"]}
    supports, contacts, cal = support_candidates(forces.raw, pairs, final, ids)
    print(f"certify {args.load}: {len(contacts)} dish-dish contacts, force sign {cal['sign']:+.0f} from {cal['n']} dish-rack "
          f"contacts ({cal['agree']:.0%} agree, rule {cal['rule']}); support candidates "
          + ("; ".join(f"{b} on {'+'.join(s)}" for b, s in supports.items()) or "none"), flush=True)
    checker = B.release_checker()
    t1 = time.monotonic()
    hits, paths = access_hits(B, checker, kind, rack, moves, final, ids)
    print(f"certify {args.load}: {len(hits)} access hits ({time.monotonic() - t1:.0f} s): "
          + ("; ".join(f"{h['a']} before {h['b']} (step {h['step']} of {h['steps']}, {h['lift_mm']} mm)" for h in hits)
             or "none"), flush=True)
    cover, resting = {}, {}
    for h in hits:
        cover.setdefault(h["a"], set()).add(h["b"])
    for b, s in supports.items():
        for a in s:
            resting.setdefault(a, set()).add(b)

    def above(start):                                        # everything resting, transitively, on ``start``
        out, todo = set(), list(start)
        while todo:
            for y in resting.get(todo.pop(), ()):
                if y not in out:
                    out.add(y)
                    todo.append(y)
        return out
    park = {i: {"position_m": [3. + .4 * j, 2., .3], "quaternion_xyzw": [0., 0., 0., 1.]} for j, i in enumerate(by_id)}
    goals = {i: P.pose_T(final[i]) for i in ids}

    def settle(absent):
        """Park ``absent``, every other dish at its final pose, 150 ticks; a dish then outside its tolerance of its final
        pose has fallen: parked too, re-settled (FALL_ROUNDS at most). (poses, absent, fallen per round)."""
        absent, rounds = set(absent), []
        while True:
            for i in ids:
                backend.set_rigid_pose(backend.objects[i], park[i] if i in absent else final[i])
            for _ in range(K.SETTLE_TICKS):
                backend.tick()
            now = backend.poses()
            fallen = sorted(i for i in ids if i not in absent
                            and B.off_goal(kind[i], centroids[kind[i]], P.pose_T(now[i]), goals[i], table))
            rounds.append(fallen)
            if not fallen or len(rounds) >= FALL_ROUNDS:
                return now, absent, rounds
            absent |= set(fallen)

    def drop(b, removed, name):
        """b by the move into the finished load without ``removed``: {landed, lateral_mm, dz_mm, tilt_deg, contact_mm
        (from b's final pose), disturbed, blocked, ...}; its ticks to traj/<name>.json."""
        now, absent, rounds = settle({b, *removed})
        ctx = [i for i in ids if i not in absent]
        bodies = [checker._body(kind[i], now[i], i) for i in ctx]
        up, rim_z, top_z = B.rack_heights(now, rack[b])
        m = B.insertion_release(checker, [(kind[b], moves[b], b)], bodies, B.appliance_bodies(checker, now), up, rim_z,
                                top_z)
        out = {"dish": b, "removed": sorted(removed), "fallen": sorted(absent - {b, *removed}), "rounds": rounds,
               **B.path_info(m)}
        rec.begin()
        rec.descent([b], [[B.lifted(moves[b], h, up)] for h in B.shown_lifts(m)], now)
        if m["poses"] is None:                                # a context dish (or the start) blocks the descent
            out.update(landed=False, blocked_by=m["blockers"], disturbed=[],
                       traj=rec.save(name, dish=b, path_blocked=True, blockers=m["blockers"], removed=sorted(removed)))
            return out
        pre = {i: P.pose_T(now[i]) for i in ctx}
        backend.set_rigid_pose(backend.objects[b], m["poses"][0])
        for _ in range(K.SETTLE_TICKS + K.WINDOW_TICKS):
            backend.tick()
        out["traj"] = rec.save(name, dish=b, release=m["poses"][0], removed=sorted(removed))
        now = backend.poses()
        d = B.goal_distance(centroids[kind[b]], P.pose_T(now[b]), goals[b])
        deep = float(max((v for k_, v in backend.latest_contact["depths_m"].items() if b in k_.split("|")), default=0.))
        out.update(landed=bool(B.within_goal(kind[b], d, table) and deep < climit), lateral_mm=round(d[0] * 1e3, 1),
                   dz_mm=round(d[1] * 1e3, 1), tilt_deg=round(d[2], 1), contact_mm=round(deep * 1e3, 2),
                   disturbed=sorted(i for i in ctx if B.off_goal(kind[i], centroids[kind[i]], P.pose_T(now[i]), pre[i],
                                                                 table)), pose=now[b])
        return out

    def said(x):
        return ("blocked by " + ", ".join(x.get("blocked_by") or []) if x.get("blocked_by") is not None else
                f"{'LANDS' if x['landed'] else 'fails'} {x['lateral_mm']} mm / {x['dz_mm']} mm / {x['tilt_deg']} deg, "
                f"contact {x['contact_mm']} mm") + (f", fell {x['fallen']}" if x["fallen"] else "")
    confirmations = []
    for b in [i for i in order if i in supports]:
        S, cov = supports[b], cover.get(b, set())

        def removed_for(present):                            # (closes over this b's S and cov)
            gone = set(S) - set(present)
            return sorted((gone | cov | above({b} | gone)) - {b} - set(present))
        c = {"dish": b, "supports": S, "kind": "plain" if len(S) == 1 else "several", "coverers": sorted(cov),
             "without": drop(b, removed_for(()), f"conf_{b}_none"), "alone": {}, "all": None, "loo": {}, "hard": [],
             "suffice": []}
        print(f"  confirm {b} on {'+'.join(S)}: without them {said(c['without'])} "
              f"({time.monotonic() - t0:.0f} s)", flush=True)
        if c["without"]["landed"]:
            c["outcome"] = "free"                            # b lands without its supports: no physical cause found
        elif c["without"].get("blocked_by") is not None:
            c["outcome"] = "blocked"
        else:
            for a in S:                                      # each support alone present: which ones suffice
                c["alone"][a] = drop(b, removed_for((a,)), f"conf_{b}_{a}")
                print(f"  confirm {b} with {a} alone: {said(c['alone'][a])}", flush=True)
            c["suffice"] = [a for a in S if c["alone"][a]["landed"]]
            if len(S) == 1:                                  # hard: b fails without a and lands with it
                c["hard"] = list(c["suffice"])
                c["outcome"] = "hard" if c["hard"] else "control_failed"
            else:                                            # hard: b fails without that support alone (the others in)
                c["all"] = drop(b, removed_for(S), f"conf_{b}_all")
                print(f"  confirm {b} with all of {'+'.join(S)}: {said(c['all'])}", flush=True)
                for a in S:
                    rest = [x for x in S if x != a]
                    c["loo"][a] = (c["alone"][rest[0]] if len(rest) == 1 else
                                   drop(b, removed_for(rest), f"conf_{b}_no_{a}"))
                    if len(rest) > 1:
                        print(f"  confirm {b} without {a} only: {said(c['loo'][a])}", flush=True)
                c["hard"] = [a for a in S if not c["loo"][a]["landed"] and c["loo"][a].get("blocked_by") is None]
                c["outcome"] = ("control_failed" if not c["all"]["landed"] else "hard" if c["hard"]
                                else "disjunctive")          # needs one of c["suffice"] (or several): not drawn
        confirmations.append(c)
    graph = physical_graph(order, confirmations, hits)
    n = graph["counts"]
    dep.update(contacts={"pairs": contacts, "calibration": cal, "supports": supports}, access={"hits": hits, "paths": paths},
               confirmations=confirmations, graph=graph, valid=True,
               verdict={"stage": "valid", "reason": (
                   f"joint gate accepted ({ginfo['order']}), witness rebuilt 24 of 24 in a fresh Kit process; "
                   f"{n['support']} hard support and {n['access']} access edge(s) drawn; {len(confirmations)} support "
                   f"candidate(s): " + ", ".join(f"{k} {sum(c['outcome'] == k for c in confirmations)}" for k in
                                                 ("hard", "disjunctive", "free", "blocked", "control_failed")))})
    dep["kit"] = {**(dep.get("kit") or {}), "certify_wall_seconds": round(time.monotonic() - started, 1),
                  "confirmation_drops": len({x["traj"] for c in confirmations
                                             for x in [c["without"], c["all"], *c["alone"].values(), *c["loo"].values()]
                                             if x})}
    write_record(args, out_path, dep)
    report.update(result="PASS", status="valid", dependency=rel(out_path),
                  reason=f"{dep['verdict']['reason']}; conflicts {n['conflicts']}, against the witness "
                         f"{n['against_witness']}, free {len(graph['free'])}")


PHYSICAL_KEEP = ("load_id", "sampler", "source", "release", "goal_source", "goal_delta", "resettle_builds",
                 "resettle_rule", "reposing", "moves_world", "entries_chosen", "witness", "entries", "targets_source",
                 "targets_world", "entries_settled", "frames")


def import_sample(src, out_dir):
    """--sample --resume-from: an earlier run's built sample's sampler output (witness, built goal, moves, chosen entries;
    nothing else it holds) as a sampler record (procedure 'physical', tolerance bench) in ``out_dir``, with the sampler's
    witness trajectory and S (.scores.json). Standard library only; an existing record is kept (False)."""
    import shutil
    src = Path(src)
    rec = json.loads(src.read_text())
    lid = rec["load_id"]
    dst = Path(out_dir) / f"{lid}.json"
    if dst.is_file():
        return False
    dep = {k: rec[k] for k in PHYSICAL_KEEP if k in rec}
    dep.update(procedure="physical", status="ok", reason=f"the re-posing sampler's build, imported from {rel(src)}",
               landing_rule=None, tolerance="bench", gate=None, certify=None, valid=None, imported_from=rel(src),
               kit={"resettle_wall_seconds": (rec.get("kit") or {}).get("resettle_wall_seconds")},
               generated_utc=datetime.now(timezone.utc).isoformat())
    traj = Path(out_dir) / "_kit" / lid / "traj"
    traj.mkdir(parents=True, exist_ok=True)
    w = src.parent / "_kit" / lid / "traj" / rec["witness"]["traj"]
    if w.is_file():
        shutil.copy2(w, traj / w.name)
    if (src.parent / f"{lid}.scores.json").is_file():
        shutil.copy2(src.parent / f"{lid}.scores.json", Path(out_dir) / f"{lid}.scores.json")
    tmp = dst.with_name(dst.name + ".tmp")
    tmp.write_text(json.dumps(dep, indent=1) + "\n")
    os.replace(tmp, dst)
    return True


def physical_loop(args):
    """--sample (host python on the box, stdlib only; plans/2026-10-16-physical-cause-graph.md): the packings of
    sequential_fresh_order.txt from r07_00012 (at most --max-packings; --resume-from DIR imports that folder's built
    samples first and skips every packing their parent runs attempted: a record's "rejudged" from names its parent run's
    folder). Per packing, Kit jobs: main (the sampler; not
    for an imported sample), gate (--manifest-out, then frigidaire_initial_state_validate.py at GATE_PEAK_M), gate_lower
    (the same lower rack first, once, on closure_failure), certify (one fresh Kit process, --gate the accepted dir); then
    clips (every certify that ran, valid first) and the score (valid, unscored) on cores 14-15. Kit jobs on cores 0-13, a
    packing's pipeline jobs before new packings, SAMPLE_STAGGER_S apart. Stops at --target-valid valid samples (Kit jobs
    killed, clips finish), when every packing is resolved, or --max-wall-seconds from now (everything killed). A Kit job
    is judged by its log (judge_log; a gate by its result.json); a failed one halts new launches. Log and state:
    data/logs/dot_rules/<--out-dir name>/, clips data/media/dot_rules/dependency_<name>/<load_id>/."""
    import shlex
    import subprocess
    t_end = time.time() + args.max_wall_seconds
    out = rel(args.out_dir)
    tag = Path(out).name
    logs, media = ROOT / "data/logs/dot_rules" / tag, f"data/media/dot_rules/dependency_{tag}"
    logs.mkdir(parents=True, exist_ok=True)
    (ROOT / out).mkdir(parents=True, exist_ok=True)
    order = (ROOT / "data/logs/dot_rules/sequential_fresh_order.txt").read_text().split()
    fresh = order[order.index(SAMPLE_PILOT):]
    todo, resumed = fresh[:args.max_packings], []
    if args.resume_from:
        rdir = ROOT / args.resume_from
        recs = {p.name[:-5]: json.loads(p.read_text()) for p in sorted(rdir.glob("r*.json"))
                if not p.name.endswith(".scores.json")}
        resumed = [k for k in order if (recs.get(k) or {}).get("status") == "ok"]
        seen = set(recs)
        for d in recs.values():
            src = (d.get("rejudged") or {}).get("from")
            if src:
                seen |= {p.name.split(".")[0] for p in (ROOT / src).parent.glob("r*.json")}
        todo = (resumed + [k for k in fresh if k not in seen])[:args.max_packings]
        for k in resumed:
            import_sample(rdir / f"{k}.json", ROOT / out)
    kit = "code/planner/frigidaire/frigidaire_dependency_kit.py"
    base = [kit, "--out-dir", out]
    st = {k: {"stage": "queued", "reason": None, "started": None, "finished": None, "resumed": k in resumed}
          for k in todo}
    kit_q, py_q, running, free_kit, free_py = [], [], [], list(KIT_CORES), list(PY_CORES)
    flags = {"halt": False, "last_kit": 0., "seq": 0}
    rank = {"certify": 0, "gate_lower": 1, "gate": 2, "main": 3}

    def say(msg):
        line = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()) + " " + msg
        with (logs / "scheduler.log").open("a") as f:
            f.write(line + "\n")
        print(line, flush=True)

    def save_state():
        (logs / "scheduler_state.json").write_text(json.dumps({
            "procedure": "physical", "updated_utc": datetime.now(timezone.utc).isoformat(),
            "deadline_utc": datetime.fromtimestamp(t_end, timezone.utc).isoformat(), "resumed": resumed,
            "target_valid": args.target_valid, "max_packings": args.max_packings, "valid": [k for k, s in st.items() if s["stage"] == "valid"],
            "packings": st, "running": [[j.kind, j.load, j.core] for j in running],
            "queued": [[j.kind, j.load] for j in kit_q + py_q]}, indent=1) + "\n")

    class Job:
        def __init__(self, kind, load, argv, wrapper="code/util/run_kit.sh", pattern=None, deadline=False):
            flags["seq"] += 1
            self.kind, self.load, self.argv, self.wrapper, self.pattern = kind, load, argv, wrapper, pattern
            self.deadline, self.seq, self.proc, self.core, self.t0, self.killed = deadline, flags["seq"], None, None, None, False
            self.log = logs / f"{load}_{kind}.log"

        def start(self, core):
            self.core, self.t0 = core, time.time()
            argv = list(self.argv)
            if self.deadline:                               # a dependency-kit Kit session ends by the deadline
                argv += ["--max-wall-seconds", str(int(max(120, t_end - time.time() + 300))), "--headless"]
            cmd = ["docker", "exec", "-w", "/workspace/dishsim", "-e", "OMP_NUM_THREADS=2", "dishsim-isaac", "taskset",
                   "-c", f"{core},{core + 16}", "nice", "-n", "10", self.wrapper, *argv]
            self.proc = subprocess.Popen(cmd, cwd=ROOT, stdout=self.log.open("w"), stderr=subprocess.STDOUT,
                                         start_new_session=True)
            say(f"start {self.kind} {self.load} core {core}")

        def kill(self):
            if self.killed:
                return False
            self.killed = True
            if self.pattern:
                subprocess.run(["docker", "exec", "dishsim-isaac", "pkill", "-KILL", "-f", "--", self.pattern],
                               capture_output=True)
            if self.proc is not None and self.proc.poll() is None:
                self.proc.kill()
            return True

    def gdir(lid, lower=False):
        return f"{out}/_gate/{lid}" + ("_lower_first" if lower else "")

    def kit_job(kind, lid, extra=()):
        return Job(kind, lid, base + ["--load", lid, *extra], deadline=True,
                   pattern=f"--load {lid} " + "".join(f"{x} " for x in extra) + "--max-wall-seconds")

    def gate_job(lid, lower=False):
        g = gdir(lid, lower)
        val = ["code/initialization/frigidaire/frigidaire_initial_state_validate.py", "--manifest",
               f"{gdir(lid)}/manifest.json", "--out-dir", g, "--max-wall-seconds", "600", "--peak-penetration-m",
               str(GATE_PEAK_M), "--headless", "--device", "cpu"] + (["--order", "lower_first"] if lower else [])
        if lower:
            return Job("gate_lower", lid, val, pattern=f"--out-dir {g} ")
        man = ["code/util/run_py.sh", *base, "--load", lid, "--manifest-out", f"{g}/manifest.json"]
        cmd = " ".join(map(shlex.quote, man)) + " && code/util/run_kit.sh " + " ".join(map(shlex.quote, val))
        return Job("gate", lid, ["-c", cmd], wrapper="bash", pattern=f"--out-dir {g} ")

    def py_jobs(lid, valid):
        clips = ["code/planner/frigidaire/frigidaire_dependency_clips.py", "--record", f"{out}/{lid}.json", "--out",
                 f"{media}/{lid}"]
        jobs = [Job("clips", lid, clips, wrapper="code/util/run_py.sh", pattern=" ".join(clips))]
        if valid and not (ROOT / out / f"{lid}.scores.json").is_file():
            score = ["code/planner/frigidaire/frigidaire_dependency_score.py", "--dependency-dir", out, "--loads", lid]
            jobs.append(Job("score", lid, score, wrapper="code/util/run_py.sh", pattern=" ".join(score)))
        if valid:
            py_q[:0] = jobs
        else:
            py_q.extend(jobs)

    def record(lid):
        p = ROOT / out / f"{lid}.json"
        return json.loads(p.read_text()) if p.is_file() else {}

    def done(j, ok, text):
        """What ``j`` finishing means for its packing, and the next job."""
        lid, s = j.load, st[j.load]
        if j.kind in ("clips", "score"):
            s[j.kind] = ok
            if not ok:
                say(f"ERROR {j.kind} {lid}: see {rel(j.log)} (the page needs it)")
            return
        if s["stage"] in ("valid", "invalid", "failed", "killed", "error"):
            return
        if j.kind in ("gate", "gate_lower"):
            res = ROOT / gdir(lid, j.kind == "gate_lower") / "result.json"
            g = json.loads(res.read_text()) if res.is_file() else None
            if g is None:
                s.update(stage="error", reason=f"{j.kind}: no result.json (see {rel(j.log)})", finished=time.time())
                flags["halt"] = True
                say(f"ERROR {j.kind} {lid}: no result.json, see {rel(j.log)}; HALT (no new launches)")
                return
            s.setdefault("gates", []).append(g.get("outcome"))
            if g.get("outcome") == "accepted":
                s["stage"] = "certify"
                kit_q.append(kit_job("certify", lid, ["--certify", "--gate", gdir(lid, j.kind == "gate_lower")]))
            elif g.get("outcome") == "closure_failure" and j.kind == "gate":
                s["stage"] = "gate_lower"
                kit_q.append(gate_job(lid, lower=True))
            else:
                s.update(stage="invalid", reason=f"joint gate {g.get('outcome')}: {g.get('reason') or g.get('error')}",
                         finished=time.time())
            say(f"{lid} {j.kind}: {g.get('outcome')} -> {s['stage']}")
            return
        if not ok:
            s.update(stage="error", reason=f"{j.kind} failed: see {rel(j.log)}", finished=time.time())
            flags["halt"] = True
            say(f"ERROR {j.kind} {lid}: see {rel(j.log)}; HALT (no new launches)")
            return
        dep = record(lid)
        if j.kind == "main":
            if dep.get("status") != "ok":
                s.update(stage="failed", reason=dep.get("reason"), finished=time.time())
                say(f"{lid} FAILED in the sampler: {dep.get('reason')}")
                return
            s["stage"] = "gate"
            kit_q.append(gate_job(lid))
            say(f"{lid} built all {len(dep.get('targets_world') or {})}: gate queued")
            return
        cert = dep.get("certify") or {}                       # certify
        s.update(certified=cert.get("certified"), edges=(dep.get("graph") or {}).get("counts"),
                 gate=(dep.get("gate") or {}).get("outcome"), finished=time.time(),
                 stage="valid" if dep.get("valid") else "invalid", reason=(dep.get("verdict") or {}).get("reason"))
        say(f"{lid} {s['stage'].upper()}: {s['reason']}")
        if cert:
            py_jobs(lid, s["stage"] == "valid")

    decided = []
    for lid in todo:                                         # a restart: packings already decided on disk stay decided
        dep, gates = record(lid), [ROOT / gdir(lid, low) / "result.json" for low in (True, False)]
        g = next((json.loads(p.read_text()) for p in gates if p.is_file()), None)
        if dep.get("valid") is not None:
            st[lid].update(stage="valid" if dep["valid"] else "invalid", started=time.time(), finished=time.time(),
                           reason=(dep.get("verdict") or {}).get("reason"), gate=(dep.get("gate") or {}).get("outcome"),
                           certified=(dep.get("certify") or {}).get("certified"),
                           edges=(dep.get("graph") or {}).get("counts"))
        elif dep.get("status") not in (None, "ok"):
            st[lid].update(stage="failed", reason=dep.get("reason"), started=time.time(), finished=time.time())
        elif g is not None and g.get("outcome") not in ("accepted", "closure_failure"):
            st[lid].update(stage="invalid", started=time.time(), finished=time.time(), gates=[g.get("outcome")],
                           reason=f"joint gate {g.get('outcome')}: {g.get('reason') or g.get('error')}")
        else:
            continue
        decided.append(lid)
    for lid in resumed:
        if lid not in decided:
            st[lid].update(stage="gate", started=time.time())
            kit_q.append(gate_job(lid))
    dtext = ", ".join(k + " " + st[k]["stage"] for k in decided) or "-"
    say(f"--sample up: {len(todo)} packings ({', '.join(todo)}), resumed {resumed or '-'}, decided on "
        f"disk {dtext}, target {args.target_valid} valid, deadline {datetime.fromtimestamp(t_end, timezone.utc):%H:%M:%SZ}")
    while True:
        now = time.time()
        for j in list(running):
            if j.proc.poll() is None:
                continue
            running.remove(j)
            (free_kit if j.core in KIT_CORES else free_py).append(j.core)
            ok, text = judge_log(j.log)
            res = [ln for ln in text.splitlines() if ln.startswith("[RESULT]")]
            say(f"done {j.kind} {j.load} ({(now - j.t0) / 60:.1f} min) log_ok {ok}: {res[-1] if res else 'no RESULT'}")
            if not j.killed:
                done(j, ok, text)
        valid = [k for k, s in st.items() if s["stage"] == "valid"]
        target = len(valid) >= args.target_valid
        if target or now >= t_end:
            why = f"{len(valid)} valid" if target else "deadline"
            for j in [x for x in running if x.deadline or x.kind.startswith("gate") or now >= t_end]:
                if j.kill():
                    say(f"killed {j.kind} {j.load}: {why}")
            kit_q.clear()
            if now >= t_end:
                py_q.clear()
            for k, s in st.items():
                if s["started"] is not None and s["stage"] in ("main", "gate", "gate_lower", "certify"):
                    s.update(stage="killed", reason=f"stopped ({why}) in stage {s['stage']}", finished=now)
        if not flags["halt"] and not target and now < t_end and free_kit and now - flags["last_kit"] >= SAMPLE_STAGGER_S:
            job = min(kit_q, key=lambda j: (rank[j.kind], todo.index(j.load), j.seq), default=None)
            if job is None:
                lid = next((k for k in todo if st[k]["stage"] == "queued"), None)
                if lid is not None:
                    st[lid].update(stage="main", started=now)
                    job = kit_job("main", lid)
                    kit_q.append(job)
            if job is not None:
                kit_q.remove(job)
                free_kit.sort()
                job.start(free_kit.pop(0))
                running.append(job)
                flags["last_kit"] = now
        while py_q and free_py and now < t_end:
            free_py.sort()
            job = py_q.pop(0)
            job.start(free_py.pop(0))
            running.append(job)
        save_state()
        resolved = all(s["stage"] in ("valid", "invalid", "failed", "error", "killed") for s in st.values())
        if not (running or kit_q or py_q) and (target or resolved or flags["halt"] or now >= t_end):
            n_att = sum(s["started"] is not None for s in st.values())
            say(f"[RESULT] {'PASS' if target else 'STOP'} --sample: {len(valid)} valid of {n_att} "
                f"attempted ({', '.join(valid) or 'none'}); "
                + ", ".join(f"{k} {s['stage']}" for k, s in st.items() if s["started"] is not None))
            return
        time.sleep(5)


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--load", "--loads", dest="load", metavar="ID", help="one load id per Kit process (required but with --sample)")
    parser.add_argument("--restricted-dir", type=Path, default=ROOT / "data/results/dot_rules/restricted/loads")
    parser.add_argument("--baseline", type=Path, default=ROOT / "data/results/benchmark/frigidaire_hotec/baseline/result.json")
    parser.add_argument("--usd", type=Path, default=ROOT / "data/assets/frigidaire_fdpc4221as/usd/fdpc4221as.usdc")
    parser.add_argument("--out-dir", type=Path, metavar="PATH", required=True,
                        help="the run's folder: <load_id>.json and _kit/<load_id>/ (with --sample: the records' folder)")
    parser.add_argument("--gate", type=Path, metavar="DIR",
                        help="--certify: the accepted joint gate of the built poses (frigidaire_initial_state_validate.py "
                             "--out-dir)")
    parser.add_argument("--certify", action="store_true",
                        help="one fresh Kit process: rebuild the record's witness from empty racks, then its support and "
                             "access edges and the support confirmation drops; needs --gate DIR")
    parser.add_argument("--sample", action="store_true",
                        help="host python on the box (stdlib): the sampling loop over the packings of "
                             "sequential_fresh_order.txt from r07_00012, Kit sessions on cores 0-13, clips on 14-15")
    parser.add_argument("--resume-from", type=Path, metavar="DIR",
                        help="--sample: an earlier run's records folder (relative to the repo root): its built samples "
                             "are imported first, new packings are the ones its runs had not attempted")
    parser.add_argument("--target-valid", type=int, default=5, metavar="N", help="--sample: stop at N valid samples")
    parser.add_argument("--max-packings", type=int, default=20, metavar="N", help="--sample: attempt at most N packings")
    parser.add_argument("--manifest-out", type=Path, metavar="PATH", help="Kit-free: write the joint-gate manifest, exit")
    parser.add_argument("--max-wall-seconds", type=float, default=3600.)
    parser.add_argument("--force", action="store_true", help="overwrite an existing output")
    args, _ = parser.parse_known_args()                       # --manifest-out exits before isaaclab is imported
    if args.manifest_out:
        return write_manifest(args)
    if args.sample:
        return physical_loop(args)
    if not args.load:
        parser.error("--load is required")
    from isaaclab.app import AppLauncher
    AppLauncher.add_app_launcher_args(parser)
    parser.set_defaults(device="cpu", headless=True)
    args = parser.parse_args()
    out_path = args.out_dir / f"{args.load}.json"
    if out_path.exists() and not args.force and not args.certify:
        print(f"[RESULT] FAIL {args.load} refusing to overwrite {rel(out_path)} (--force)", flush=True)
        return
    kit_dir = args.out_dir / "_kit" / args.load
    report_name = "report_certify.json" if args.certify else "report.json"
    started = time.monotonic()
    report = {"result": "FAIL", "reason": None, "load_id": args.load, "sampler": "restricted",
              "started_utc": datetime.now(timezone.utc).isoformat()}
    app = None
    try:
        source, entries = read_load(args)
        report["source"] = rel(source)
        app = AppLauncher(args).app
        args.app = app
        sys.path[:0] = [str(ROOT / "code/src"), str(ROOT / "code/frigidaire/src"), str(ROOT / "code/planner/frigidaire")]
        (physical_certify if args.certify else sampler)(args, source, entries, out_path, kit_dir, report, started)
    except Refused as exc:
        report.update(result="FAIL", reason=str(exc))
    except Exception as exc:                                   # noqa: BLE001
        traceback.print_exc()
        report.update(result="FAIL", reason=f"exception {exc!r}")
    finally:
        report.update(wall_seconds=time.monotonic() - started, finished_utc=datetime.now(timezone.utc).isoformat())
        kit_dir.mkdir(parents=True, exist_ok=True)
        (kit_dir / report_name).write_text(json.dumps(report, indent=2, default=str) + "\n")
        print(f"[RESULT] {report['result']} {args.load} {report['reason']}", flush=True)
        if app is not None:
            from dishsim.media import release_sim_for_close
            release_sim_for_close()
            app.close()


if __name__ == "__main__":
    main()
