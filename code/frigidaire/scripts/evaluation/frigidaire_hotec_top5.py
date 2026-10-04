#!/usr/bin/env python3
# Copyright (c) 2026, dishsim project.
# SPDX-License-Identifier: BSD-3-Clause
"""Top-5 exposure arrangements of all 24 HOTEC dishes in the Frigidaire FDPC4221AS twin (no planning).

Every arrangement is a complete rack load of the user's set (8 plates, 8 bowls, 8 cups, v2 massed assets) on the
re-measured racks (upper_tines_4x13_v5 / lower_tines_6x12_v4). No counter start, no rearrangement, no loading order.

Candidates come from a multi-start search: random complete packings (first-fit in a seeded shuffled family order)
improved by the HOTEC benchmark's coordinate ascent on the revision-5 exposure score S. Rules = the benchmark's
Isaac-learned families (frigidaire_bench.py: no mid-zone bowl, front-right bowls x <= 0.165, tub wall, rack-retraction
sweep, 3 mm dish clearance, joint-gate penetration bans) with EVERY plate gap allowed (the paused benchmark's even-gap rule is
not used), and no pooling dish. Each feasible candidate gets the benchmark's joint Isaac gate (all 24 at once: settle,
retract both racks, containment); the top 5 accepted candidates by planned S that differ pairwise in >= 6 of 24 dish
slots are scored on their settled poses and rendered (stills + orbit video).

If 24 dishes do not fit at level 0, the capacity probe relaxes in a fixed order (user, 2026-09-29): level 1 admits the
bowl between the plate banks (lower_mid), level 2 also seeds the starts with every plate in the front bank.

    python3 code/frigidaire/scripts/evaluation/frigidaire_hotec_top5.py --all                 # host: everything, resumable
    code/scripts/run_py.sh code/frigidaire/scripts/evaluation/frigidaire_hotec_top5.py --prepare --level 0
    code/scripts/run_py.sh code/frigidaire/scripts/evaluation/frigidaire_hotec_top5.py --capacity --level 0 --part 0 --parts 2
    code/scripts/run_py.sh code/frigidaire/scripts/evaluation/frigidaire_hotec_top5.py --search-start 3 --level 0
    code/scripts/run_py.sh code/frigidaire/scripts/evaluation/frigidaire_hotec_top5.py --settle-score 3
    code/scripts/run_py.sh code/frigidaire/scripts/evaluation/frigidaire_hotec_top5.py --collect

Container stages write under data/results/hotec/frigidaire/top5_<DATE>/ and data/media/hotec_wheatstraw/top5_<DATE>/ (root-owned);
the host orchestrator only reads there and logs to data/logs/hotec_top5/. Judge Kit runs by their [RESULT] lines.
"""
from __future__ import annotations

import argparse
from collections import Counter
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import time

import numpy as np

ROOT = Path(__file__).resolve().parents[4]
sys.path[:0] = [str(ROOT / "code/src"), str(ROOT / "code/frigidaire/src")]
from dishsim import legacy_data_path  # noqa: E402
_spec = importlib.util.spec_from_file_location("frigidaire_bench", ROOT / "code/frigidaire/scripts/experiment/frigidaire_bench.py")
B = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(B)

DATE = "20260929"
# Contact rules (user, 2026-09-29: "teleport all dishes and allow some collision"; the set is known to fit). The first
# run (top5_20260929) demanded >= 3 mm between dishes (the benchmark's one-dish-at-a-time rule) and forbade nesting, and
# racked all 24 in 0 of 40 packings at every relaxation level. Now dishes may touch and press <= DISH_TOL_M into each
# other, and nesting is scored (below). DISH_TOL_M stays under the joint gate's preflight limit (InitialCollisionChecker:
# <= 1 mm dish-dish and dish-appliance before physics; a 2 mm variant failed it as initial_collision, 1.7 mm into the
# lower rack). No appliance contact is needed: with touching dishes 6/6 front-bank packings rack all 24.
APPLIANCE_TOL_M = 0.
DISH_TOL_M = .0008
VARIANT = "touch08"
# The joint gate aborts on any momentary contact peak >= 2 mm (random_poses.LIMITS). Every one of the first four
# touch08 gates failed that way within 20 s (bowl-bowl 4.0 / 4.5 mm, plate-plate 2.2, bowl-rack 2.3: 24 teleported dishes
# dropping up to 60 mm onto tines and each other). User: "allow some collision" -> our gates allow a 5 mm momentary peak;
# the median-penetration limit (1 mm, so lasting interpenetration still fails), the 1 mm preflight, the rest windows,
# retraction and containment are unchanged. The 2 mm-limit results stay under gates/ as evidence.
GATE_PEAK_PEN_M = .005
STRICT_OUT = ROOT / "data/results/hotec/frigidaire" / f"top5_{DATE}"
OUT = ROOT / "data/results/hotec/frigidaire" / f"top5_{DATE}_{VARIANT}"
MEDIA = ROOT / "data/media/hotec_wheatstraw" / f"top5_{DATE}_{VARIANT}"
LOGS = ROOT / "data/logs/hotec_top5" / VARIANT
SCRIPT = "code/frigidaire/scripts/evaluation/frigidaire_hotec_top5.py"
ASSETS_REL = "data/assets/models/hotec_wheatstraw/v2"
COLOURS = ("blue_grey", "teal", "coral", "mustard")          # frigidaire_hotec_load.py: the assets' colour variants
KIND_ORDER = ("plate", "bowl", "cup")
ROSTER = [(f"{kind}_{k:02d}", kind) for kind in KIND_ORDER for k in range(1, 9)]
LEVELS = {0: "benchmark rules, every plate gap",
          1: "+ the bowl between the plate banks (lower_mid)",
          2: "+ start packings with every plate in the front bank"}
TOP, DISTINCT_MIN = 5, 6                  # user: 5 arrangements, >= 6 of 24 dishes in a different slot
DISTINCT_FALLBACK = 4                     # 2026-09-29: capacity forces every plate into the front bank and every cup into the
                                          # same 8 glass slots, so loads can differ only in their 8 bowls; ranks the >= 6 rule
                                          # cannot fill take loads >= 4 dishes (half the bowls) from every chosen one
STARTS, EXTRA_STARTS, MAX_STARTS = 20, 10, 40
CAPACITY_TRIES, CAPACITY_NEED, PARTS = 40, 1, 2   # >= 1 complete packing = "24 fit" at that level (user Q6)
SHUFFLES_PER_START = 10                   # fresh packings per start before reusing a capacity packing (a try: 12-33 s)
MAX_SWEEPS = 4                            # plan: up to 4 sweeps (the benchmark goal search uses 2); stops on no gain
BOWL_MATCH_M = .020                       # lower-rack bowls within 20 mm (xy) share a slot; one load's bowls sit >= 82 mm apart
STOPPED_BANS = B.OUT / "stopped_20260928"  # the new-rack gate bans of the stopped benchmark run (2026-09-28)


# --------------------------------------------------------------------------- small helpers

def seed_of(*parts):
    return int(hashlib.sha256("|".join(map(str, parts)).encode()).hexdigest()[:8], 16)


def atomic_write(path, text):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_name(f"{path.name}.tmp{os.getpid()}")
    tmp.write_text(text)
    os.replace(tmp, path)


def tag_of(k):
    return f"start_{k:02d}"


ORDERS = ("front_plates", "bowls_first", "plates_first")


def order_ids(order):
    """Placement order for a first-fit packing: 'front_plates' (every plate in the front bank, gaps filled in order,
    then bowls: the only way 24 fit on the first tape racks, HOTEC v8), 'bowls_first' (the scarce kind) or
    'plates_first'."""
    first = "bowl" if order == "bowls_first" else "plate"
    kinds = [first] + [kind for kind in KIND_ORDER if kind != first]
    return [(oid, kind) for kind in kinds for oid, k in ROSTER if k == kind]


# --------------------------------------------------------------------------- families (per relaxation level)

def family_ok(c, points, level):
    """The benchmark's family filters (frigidaire_bench.family_ok) with every plate gap allowed; level >= 1 admits
    the between-banks bowl zone the benchmark drops."""
    dropped = set(B.DROPPED_SLOTS) - ({"lower_mid"} if level >= 1 else set())
    if c["slot"] in dropped:
        return False
    if c["slot"] == "lower_frontright" and c["position"][0] > B.FRONT_RIGHT_MAX_X_M + 1e-9:
        return False
    return B.tub_clear(points, c["position"], c["quaternion_xyzw"])


def level_families(level):
    fam, parts, points = B.HX().candidate_families(B.ASSETS)
    fam = {kind: [(c, o) for c, o in lst if family_ok(c, points[kind], level)] for kind, lst in fam.items()}
    return fam, parts, points


def appliance_pen(world, c, cap):
    """Deepest contact of a candidate with the closed appliance (FCL contacts; stops once past ``cap``)."""
    fcl, worst = world.fcl, 0.
    for obj in world.candidate_objects(c):
        data = fcl.CollisionData(request=fcl.CollisionRequest(num_max_contacts=64, enable_contact=True))
        world.manager.collide(obj, data, fcl.defaultCollisionCallback)
        for ct in data.result.contacts:
            worst = max(worst, float(ct.penetration_depth))
        if worst > cap:
            break
    return worst


class TolerantDishSet(B.DishSet):
    """Dishes may touch and press up to DISH_TOL_M into each other. B.DishSet keeps >= 3 mm between dishes, which the
    benchmark needs to build a load one dish at a time; a joint teleport + settle does not."""

    def collides(self, c, skip=(), clearance=None):
        m, lo, hi, _ = self._manager(c)
        fcl = self.fcl
        for oid, (other, olo, ohi, _, _) in self.items.items():
            if oid in skip or np.any(hi < olo) or np.any(ohi < lo):
                continue
            data = fcl.CollisionData(request=fcl.CollisionRequest(num_max_contacts=200, enable_contact=True))
            m.collide(other, data, fcl.defaultCollisionCallback)
            if any(float(ct.penetration_depth) > DISH_TOL_M for ct in data.result.contacts):
                return True
        return False


B.DishSet = TolerantDishSet          # our module copy only: B.ascend / B.free_iter build and query dish sets through it

# Nesting is scored, not forbidden (2026-09-29). The benchmark's rule (frigidaire_bench.nests: same-rack bowls whose
# rack-frame outlines overlap AND whose depth slabs overlap along the bowl axis) rejected EVERY remaining bowl pose once
# 6-7 bowls were placed (15 cm bowls: two fill the rear zone, three the upper channel), capping the set at 23 of 24 in
# 0/40 packings at every level. Nested bowls do not collide; the exposure score already penalises a bowl hidden in
# another (its inside is not sprayed), and every record lists the nested pairs.
_BENCH_NESTS = B.nests
B.nests = lambda points, c, o, others: False


def nested_pairs(entries, points):
    """Pairs of same-rack bowls the benchmark's nesting rule would call nested (reported, not forbidden)."""
    bowls = [e for e in entries if e["kind"] == "bowl"]
    return [[a["id"], b["id"]] for i, a in enumerate(bowls) for b in bowls[i + 1:]
            if a["rack"] == b["rack"] and _BENCH_NESTS(points, b, None, [a])]


def level_masks(fam, parts, points, level, compute=False):
    """Closed-appliance FCL mask with APPLIANCE_TOL_M contact allowed, AND the rack-retraction sweep
    (frigidaire_bench.family_masks), cached per level in OUR folder so the benchmark's caches are left alone."""
    d = OUT / "cache" / f"level{level}"
    key = f"{B.family_digest(fam)}|sweep{B.SWEEP_STEPS}" + (f"|appliance_tol{APPLIANCE_TOL_M}" if APPLIANCE_TOL_M > 0 else "")
    path = d / "masks.json"
    for cached in (path, *sorted((OUT / "cache").glob("level*/masks.json")),     # levels 1 and 2 share families;
                   *sorted((STRICT_OUT / "cache").glob("level*/masks.json"))):    # no rack contact = the strict masks
        if cached.is_file():
            data = json.loads(cached.read_text())
            if data.get("key") == key:
                return {k: np.asarray(v, dtype=bool) for k, v in data["masks"].items()}
    if not compute:
        raise SystemExit(f"[RESULT] FAIL masks for level {level} missing or stale: run --prepare --level {level}")
    d.mkdir(parents=True, exist_ok=True)
    strict = STRICT_OUT / "cache" / f"level{min(level, 1)}" / "appliance_free.json"     # the strict run's no-contact mask
    base = B.appliance_free(fam, parts, points, cache=strict if strict.is_file() else d / "appliance_free.json")
    world = B.hotec_world(parts, points)
    tol = {kind: np.array([bool(ok) or (APPLIANCE_TOL_M > 0 and appliance_pen(world, c, APPLIANCE_TOL_M) <= APPLIANCE_TOL_M)
                           for (c, _), ok in zip(fam[kind], base[kind])], dtype=bool) for kind in base}
    masks = {kind: np.array([bool(ok) and not B.sweep_hits(c) for (c, _), ok in zip(fam[kind], tol[kind])], dtype=bool)
             for kind in tol}
    atomic_write(path, json.dumps({"key": key, "masks": {k: v.tolist() for k, v in masks.items()}}) + "\n")
    return masks


def start_families(fam, masks, level):
    """Level 2 seeds the packings with every plate in the front bank (the ascent may still move them)."""
    if level < 2:
        return fam, masks
    keep = [i for i, (c, _) in enumerate(fam["plate"]) if c["slot"].startswith("lower_front")]
    return {**fam, "plate": [fam["plate"][i] for i in keep]}, {**masks, "plate": masks["plate"][keep]}


def gate_bans(out):
    """Pairs/poses that PENETRATED in a joint Isaac gate (user rule Q5), from one benchmark results folder: goal-gate
    bans (outcome penetration_failure) and own-load bans whose gate failed on penetration (the build never ran then,
    so every pair is a gate pair). Bans learned by one-dish-at-a-time builds (outcome sequence, or sequence_plan after
    an accepted gate) are not loaded: this task validates by a joint settle only."""
    bans = set()
    paths = [*(Path(out) / "instances" / "attempts").glob("*/bans.json"), *(Path(out) / "bans").glob("*.json"),
             *(Path(out) / "plans" / "open").glob("*/*.bans.json")]
    for path in paths:
        data = json.loads(path.read_text())
        if data.get("outcome") == "penetration_failure" or (data.get("outcome") == "sequence_plan"
                                                              and data.get("gate") == "penetration_failure"):
            bans |= {frozenset(pair) for pair in data["pairs"]}
    return bans


def all_bans():
    """New-rack gate-penetration bans: the live benchmark folder, the stopped run moved aside on 2026-09-28 and this
    run's own failed gates (OUT/bans, written by --ban); old-rack bans under pre_rebuild_20260928/ are not loaded."""
    return frozenset(gate_bans(B.OUT) | gate_bans(STOPPED_BANS) | gate_bans(OUT))


def banned_ids(entries, bans):
    """Dishes of a load that sit in a banned pose or form a banned pair with another dish of the load."""
    return [e["id"] for e in entries if B.banned(e, [o for o in entries if o["id"] != e["id"]], bans)]


def pack(fam, masks, parts, points, level, bans, seed, order):
    """One first-fit packing in a seeded shuffled family order (frigidaire_bench.first_fit without keeps), recording
    the dish that found no free pose. Returns (entries or None, failing dish id or None)."""
    fam, masks = start_families(fam, masks, 2 if order == "front_plates" else level)
    rng = np.random.default_rng(seed)
    perm = {k: rng.permutation(len(fam[k])) for k in fam}
    if order == "front_plates":                              # gaps in order, a random free variant within each gap
        gap = lambda i: int(fam["plate"][i][0]["slot"].rsplit("_", 1)[1])
        perm["plate"] = np.array(sorted(perm["plate"], key=gap), dtype=int)
    fam = {k: [fam[k][i] for i in perm[k]] for k in fam}
    masks = {k: masks[k][perm[k]] for k in masks}
    dishes, entries = TolerantDishSet(parts, points), []
    for oid, kind in order_ids(order):
        pick = next(B.free_iter({"id": oid, "kind": kind}, entries, fam, masks, parts, points, dishes, bans), None)
        if pick is None:
            return None, oid
        entries.append(B.entry_of(oid, pick[0]))
        dishes.add(oid, pick[0])
    return entries, None


# --------------------------------------------------------------------------- distinctness

def dish_key(e):
    """Slot of one dish, ignoring which same-kind dish sits there: (kind, rack, slot) for plates, cups and upper
    bowls (one per slot/gap; lean, offset, tilt and lift variants ignored). Lower-rack bowls get None: their zones are
    wide and alias (candidate_families lists every lower_rearright pose again under lower_rear), so distance() pairs
    them by position instead."""
    if e["kind"] == "bowl" and e["rack"] == "LowerRack":
        return None
    return (e["kind"], e["rack"], e["slot"])


def distance(a, b):
    """Number of dishes of load ``a`` without a matching slot in load ``b``: a multiset match on dish_key, plus
    lower-rack bowls paired nearest-first by xy distance <= BOWL_MATCH_M (each bowl has at most one partner)."""
    ka, kb = [dish_key(e) for e in a], [dish_key(e) for e in b]
    matched = sum((Counter(k for k in ka if k is not None) & Counter(k for k in kb if k is not None)).values())
    la = [e["position"][:2] for e, k in zip(a, ka) if k is None]
    lb = [e["position"][:2] for e, k in zip(b, kb) if k is None]
    used_a, used_b = set(), set()
    for d, i, j in sorted((float(np.hypot(p[0] - q[0], p[1] - q[1])), i, j)
                          for i, p in enumerate(la) for j, q in enumerate(lb)):
        if d <= BOWL_MATCH_M + 1e-12 and i not in used_a and j not in used_b:
            used_a.add(i)
            used_b.add(j)
            matched += 1
    return max(len(a), len(b)) - matched


def select_top(cands, top=TOP, dmin=DISTINCT_MIN, fallback=DISTINCT_FALLBACK):
    """Greedy by planned S: a candidate joins when it differs from every chosen one in >= dmin dishes; if that leaves
    ranks empty, a second pass fills them with candidates >= ``fallback`` dishes from every chosen one. Returned in
    planned-S order."""
    order = sorted(cands, key=lambda c: (-c["S"], c["start"]))
    chosen = []
    for limit in (dmin,) + ((fallback,) if fallback is not None and fallback < dmin else ()):
        for c in order:
            if len(chosen) == top:
                break
            if all(c is not o for o in chosen) and all(distance(c["entries"], o["entries"]) >= limit for o in chosen):
                chosen.append(c)
    return sorted(chosen, key=lambda c: (-c["S"], c["start"]))


# --------------------------------------------------------------------------- records (read on host and in container)

def search_path(k):
    return OUT / "search" / f"{tag_of(k)}.json"


def gate_dir(k):
    return OUT / f"gates_pen{GATE_PEAK_PEN_M * 1000:g}mm" / tag_of(k)


def settled_path(k):
    return OUT / "settled" / f"{tag_of(k)}.json"


def settled_layout_path(k):
    return OUT / "settled" / f"{tag_of(k)}.layout.json"


def ban_path(k):
    return OUT / "bans" / f"{tag_of(k)}.json"


def read(path):
    path = Path(path)
    return json.loads(path.read_text()) if path.is_file() else None


def gate_outcome(k):
    res = read(gate_dir(k) / "result.json")
    return None if res is None else res.get("outcome")


def capacity_packings(level):
    out = []
    for p in sorted((OUT / "capacity").glob(f"level{level}_part*.json")):
        out += [t for t in read(p)["tries"] if t["complete"]]
    return sorted(out, key=lambda t: t["try"])


def candidates():
    """Every search record with its gate outcome and settled score (if any)."""
    out = []
    for path in sorted((OUT / "search").glob("start_*.json")):
        if path.name.count(".") > 1:                        # start_KK.gate_manifest.json / .layout.json
            continue
        rec = read(path)
        k = rec["start"]
        out.append({**rec, "gate": gate_outcome(k), "settled": read(settled_path(k))})
    return out


def settled_ok(c):
    """The settled load keeps every rule: no dish pools after the settle, all 24 racked."""
    st = c["settled"]
    return bool(st) and bool(st["feasible_settled"]) and st["racked"] == len(ROSTER)


def accepted(cands):
    return [c for c in cands if c.get("feasible") and c["gate"] == "accepted" and settled_ok(c)]


# --------------------------------------------------------------------------- container stages

def stage_prepare(level):
    fam, parts, points = level_families(level)
    masks = level_masks(fam, parts, points, level, compute=True)
    bans = all_bans()
    counts = {kind: (len(fam[kind]), int(masks[kind].sum())) for kind in fam}
    slots = {kind: len({c["slot"] for (c, _), ok in zip(fam[kind], masks[kind]) if ok}) for kind in fam}
    print(f"[INFO] level {level} ({LEVELS[level]}): candidates (all, free) {counts}, free slots {slots}, bans {len(bans)}",
          flush=True)
    print(f"[RESULT] PASS prepare level {level}", flush=True)
    return 0


def stage_capacity(level, part, parts_n):
    path = OUT / "capacity" / f"level{level}_part{part}.json"
    if path.is_file():
        print(f"[RESULT] PASS capacity level {level} part {part}: exists", flush=True)
        return 0
    fam, parts, points = level_families(level)
    masks = level_masks(fam, parts, points, level)
    bans = all_bans()
    tries = []
    for t in range(part, CAPACITY_TRIES, parts_n):
        order = ORDERS[(t // parts_n) % len(ORDERS)]
        t0 = time.monotonic()
        entries, failed = pack(fam, masks, parts, points, level, bans, seed_of("capacity", level, t), order)
        tries.append({"try": t, "order": order, "complete": entries is not None, "failed_at": failed,
                      "seconds": round(time.monotonic() - t0, 1), "entries": entries})
        print(f"[INFO] level {level} try {t} ({order}): {'complete' if entries else 'stuck at ' + failed}", flush=True)
    atomic_write(path, json.dumps({"level": level, "part": part, "parts": parts_n, "tries": tries}, indent=1) + "\n")
    n = sum(t["complete"] for t in tries)
    stuck = Counter(t["failed_at"].rsplit("_", 1)[0] for t in tries if t["failed_at"])
    print(f"[RESULT] PASS capacity level {level} part {part}: {n}/{len(tries)} complete, stuck by kind {dict(stuck)}",
          flush=True)
    return 0


def manifest_of(entries):
    base = read(B.OUT / "baseline" / "result.json")
    return {"objects": [{"object_id": e["id"], "kind": e["kind"], "rack": e["rack"],
                         "rack_local_pose": {"position_m": e["position"], "quaternion_xyzw": e["quaternion_xyzw"]}}
                        for e in entries],
            "baseline": base.get("snapshot", base), "tableware": B.tableware()}


def layout_of(entries):
    """A frigidaire_hotec_load.py layout with FRESH asset/appliance hashes (its Kit stage refuses stale ones)."""
    from dishsim_frigidaire.asset import BODY_POSITIONS
    from dishsim_frigidaire.loading import geometry_hashes
    from dishsim_frigidaire.paths import ASSET_DIR
    catalog = read(B.ASSETS / "catalog.json")
    objects = []
    for e in sorted(entries, key=lambda e: e["id"]):
        n = int(e["id"].rsplit("_", 1)[1])
        objects.append({"object_id": e["id"], "kind": e["kind"], "usd": f"{e['kind']}.usda",
                        "color": COLOURS[(n - 1) % len(COLOURS)], "rack": e["rack"], "slot": e["slot"],
                        "variant": e["variant"], "item": e["slot"],
                        "rack_local_pose": {"position_m": e["position"], "quaternion_xyzw": e["quaternion_xyzw"]}})
    by_rack = {}
    for o in objects:
        by_rack.setdefault(o["rack"], Counter())[o["kind"]] += 1
    return {"schema_version": 1, "purpose": "hotec_load_layout", "assets_dir": str(B.ASSETS),
            "asset_sha256": {f"{kind}.usda": catalog["items"][kind]["sha256"] for kind in B.KINDS},
            "parameters_sha256": catalog["parameters_sha256"], "appliance_dir": str(ASSET_DIR),
            "appliance_sha256": geometry_hashes(ASSET_DIR),
            "body_positions_m": {k: [float(v) for v in pos] for k, pos in BODY_POSITIONS.items()},
            "coordinate_system": "rack objects: rack-local metres + XYZW in the closed-appliance FCL frame",
            "priority": "top-5 exposure search (frigidaire_hotec_top5.py); every dish racked",
            "objects": objects,
            "counts": {"requested": {k: 8 for k in B.KINDS}, "fitted": dict(Counter(o["kind"] for o in objects)),
                       "counter": {k: 0 for k in B.KINDS}, "by_rack": {r: dict(c) for r, c in by_rack.items()},
                       "total": len(objects)},
            "fcl": {"status": "FCL: closed appliance free, retraction sweep clear, dishes >= 3 mm apart; "
                              "physics: the joint Isaac gate"}}


def stage_search(k, level):
    path = search_path(k)
    if path.is_file():
        print(f"[RESULT] PASS search {tag_of(k)}: exists", flush=True)
        return 0
    t0, c0 = time.monotonic(), time.process_time()
    fam, parts, points = level_families(level)
    masks = level_masks(fam, parts, points, level)
    bans = all_bans()
    packs = [t for t in capacity_packings(level) if not banned_ids(t["entries"], bans)]   # ascend never drops a ban
    start, source, stuck = None, None, Counter()
    if k < len(packs):
        start, source = packs[k]["entries"], f"capacity packing (try {packs[k]['try']}, {packs[k]['order']})"
    else:
        for t in range(SHUFFLES_PER_START):
            order = ORDERS[t % len(ORDERS)]
            start, failed = pack(fam, masks, parts, points, level, bans, seed_of("start", level, k, t), order)
            if start is not None:
                source = f"shuffle {t} ({order})"
                break
            stuck[failed.rsplit("_", 1)[0]] += 1
        if start is None and packs:                          # low packing rate: reuse one, the ascent seed differs
            j = k % len(packs)
            start, source = packs[j]["entries"], f"capacity packing (try {packs[j]['try']}) reused, ascent seed {k}"
    rec = {"start": k, "tag": tag_of(k), "level": level, "rules": LEVELS[level], "source": source,
           "stuck_by_kind": dict(stuck)}
    if start is None:
        rec.update(feasible=False, reason=f"no complete packing in {SHUFFLES_PER_START} shuffles",
                   seconds=time.monotonic() - t0)
        atomic_write(path, json.dumps(rec, indent=1) + "\n")
        print(f"[RESULT] REROLL search {tag_of(k)}: no complete packing ({dict(stuck)})", flush=True)
        return 0
    scorer = B.Scorer()
    s0 = scorer.score(start)
    rng = np.random.default_rng(seed_of("ascent", level, k))
    entries, _, history, sweeps, stop = B.ascend([dict(e) for e in start], fam, masks, parts, points, scorer.score,
                                                 scorer.iso, rng, max_sweeps=MAX_SWEEPS, log=lambda *a: None, bans=bans)
    final = scorer.score(entries)
    pooling = B.pooling_entries(entries)
    hit = banned_ids(entries, bans)
    feasible = bool(final["feasible"]) and not pooling and not hit and len(entries) == len(ROSTER)
    entries = sorted(entries, key=lambda e: e["id"])
    reason = (None if feasible else "pooling " + ", ".join(pooling) if pooling else
              "banned pose/pair " + ", ".join(hit) if hit else "score infeasible")
    rec.update(S_start=s0["score"], S=final["score"], W=final["worst"], feasible=feasible, pooling=pooling,
               reason=reason, max_sweeps=MAX_SWEEPS, appliance_tol_m=APPLIANCE_TOL_M, dish_tol_m=DISH_TOL_M,
               nested_pairs=nested_pairs(entries, points),
               sweeps=sweeps, stop=stop, moves=len(history) - 1, history=history, score_calls=scorer.calls,
               per_object=[{key: o[key] for key in ("id", "kind", "rack", "exposure", "pools")} for o in final["objects"]],
               entries=entries, seconds=time.monotonic() - t0, cpu_seconds=time.process_time() - c0)
    if feasible:
        atomic_write(path.with_name(f"{tag_of(k)}.gate_manifest.json"), json.dumps(manifest_of(entries), indent=1) + "\n")
        atomic_write(path.with_name(f"{tag_of(k)}.layout.json"), json.dumps(layout_of(entries), indent=1) + "\n")
    atomic_write(path, json.dumps(rec, indent=1) + "\n")
    print(f"[RESULT] {'PASS' if feasible else 'REROLL'} search {tag_of(k)}: S {rec['S_start']:.4f} -> {rec['S']:.4f}, "
          f"W {rec['W']:.4f}, {rec['moves']} moves, {sweeps} sweeps ({stop}), {rec['seconds']:.0f} s, "
          f"feasible {feasible}", flush=True)
    return 0


def figure(path, final, title, lines):
    """Per-rack exposure heat maps and per-dish bars (the benchmark's score-detail figure without the move trace)."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    hx = B.HX()
    from dishsim_frigidaire import exposure as E
    fig = plt.figure(figsize=(17, 7.5), facecolor="white")
    gs = fig.add_gridspec(1, 4, width_ratios=(1, 1, 1.1, .8), wspace=.28)
    s = final["samples"]
    owner = s["owner"]
    racks = [o["rack"] for o in final["objects"]]
    sc = None
    for col, rack in enumerate(("LowerRack", "UpperRack")):
        ax = fig.add_subplot(gs[0, col])
        mask = np.isin(owner, [i for i, r in enumerate(racks) if r == rack])
        pts, ex = s["points"][mask], s["exposure"][mask]
        order = np.argsort(ex)
        sc = ax.scatter(pts[order, 0] * 1000, pts[order, 1] * 1000, c=ex[order], s=2, cmap="viridis", vmin=0, vmax=1,
                        rasterized=True)
        for wire in E.rack_wires(rack):
            w = wire + np.asarray(hx.BODY_POSITIONS[rack])
            ax.plot(w[:, 0] * 1000, w[:, 1] * 1000, color="#b8c0c8", lw=.4, zorder=0)
        ax.set_aspect("equal")
        ax.set_xlim(-300, 300)
        ax.set_ylim(-300, 300)
        ax.set_title(f"{rack}: food-contact exposure", fontsize=10)
        ax.set_xlabel("x [mm]")
        ax.set_ylabel("y [mm] (front at the bottom)" if col == 0 else "")
    if sc is not None:
        fig.colorbar(sc, ax=fig.axes[:2], shrink=.8, label="share of spray-arm rays reaching the surface")
    ax = fig.add_subplot(gs[0, 2])
    objs = sorted(final["objects"], key=lambda o: o["id"])
    colours = ["#4c72b0" if o["kind"].endswith("plate") else "#dd8452" if o["kind"].endswith("bowl") else "#55a868"
               for o in objs]
    ax.barh([o["id"] + (" (pools)" if o["pools"] else "") for o in objs], [o["exposure"] for o in objs], color=colours)
    ax.set_xlim(0, 1)
    ax.invert_yaxis()
    ax.tick_params(axis="y", labelsize=7)
    ax.set_xlabel("exposure of the dish (area-weighted mean)")
    ax.set_title("per dish (blue plates, orange bowls, green cups)", fontsize=10)
    ax = fig.add_subplot(gs[0, 3])
    ax.axis("off")
    ax.text(0, 1, "\n".join(lines), va="top", fontsize=9, family="monospace")
    fig.suptitle(title, fontsize=12)
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=110)
    plt.close(fig)


def stage_settle_score(k):
    out = settled_path(k)
    if out.is_file():
        print(f"[RESULT] PASS settled {tag_of(k)}: exists", flush=True)
        return 0
    gate = read(gate_dir(k) / "result.json")
    if gate is None or gate.get("outcome") != "accepted":
        print(f"[RESULT] FAIL settled {tag_of(k)}: gate {None if gate is None else gate.get('outcome')}", flush=True)
        return 1
    from dishsim_frigidaire.loading import visual_points
    rec = read(search_path(k))
    poses = gate["initial_snapshot"]["poses"]                # settled, racks extended (final_snapshot = retracted)
    frames = {r: poses[r] for r in ("LowerRack", "UpperRack")}
    kinds = dict(ROSTER)
    points = {kind: visual_points(B.ASSETS / f"{kind}.usda") for kind in B.KINDS}
    entries, loose = B.racked_entries({oid: poses[oid] for oid in kinds}, frames, kinds, points)
    if not loose:                                            # the render starts from the SCORED (gate-settled) poses
        planned = {e["id"]: e for e in rec["entries"]}
        settled = [{**planned[e["id"]], "rack": e["rack"], "position": [float(v) for v in e["position"]],
                    "quaternion_xyzw": [float(v) for v in e["quaternion_xyzw"]]} for e in entries]
        atomic_write(settled_layout_path(k), json.dumps(layout_of(settled), indent=1) + "\n")
    scorer = B.Scorer()
    E, hx = scorer.E, scorer.hx
    res = scorer.score(entries)                              # the same function as the planned S
    basket = (np.asarray(hx.BODY_POSITIONS["SilverwareBasket"], dtype=float), np.asarray(E.IDENTITY, dtype=float))
    final = E.score_arrangement(E.Arrangement(tag_of(k), "", "", hx.world_objects(entries), basket), device=scorer.device,
                                samples=E.DEFAULTS["samples_per_object"], directions=E.DEFAULTS["directions"],
                                baselines=False)
    fig_path = MEDIA / "exposure" / f"{tag_of(k)}_settled_exposure.png"
    lines = [f"{tag_of(k)}  ·  rules level {rec['level']}", f"  {rec['rules']}", "",
             f"planned S  {rec['S']:.3f}   worst {rec['W']:.3f}", f"settled S  {res['score']:.3f}   worst {res['worst']:.3f}",
             f"pooling (settled): {sum(o['pools'] for o in res['objects'])}", f"racked {len(entries)}/24, loose {loose}",
             f"gate: {gate['outcome']}, {gate.get('wall_seconds', 0):.0f} s wall",
             f"max settle penetration {1000 * (gate.get('maximum_settle_penetration_m') or 0):.2f} mm", "",
             "S = sum(A_o E_o) / sum(A_o)", "E_o: share of spray-arm rays reaching", "the food-contact samples of dish o;",
             "A_o: its food-contact area;", "pooling = infeasible."]
    figure(fig_path, final, f"Settled score detail: {tag_of(k)} (joint Isaac gate, racks extended)", lines)
    out_rec = {"start": k, "tag": tag_of(k), "S_planned": rec["S"], "W_planned": rec["W"], "S_settled": res["score"],
               "W_settled": res["worst"], "feasible_settled": bool(res["feasible"]),
               "S_settled_arrangement": final["score"], "racked": len(entries), "loose": loose,
               "per_object": [{key: o[key] for key in ("id", "kind", "rack", "exposure", "pools")} for o in res["objects"]],
               "nested_pairs_settled": nested_pairs(entries, points),
               "gate": {"outcome": gate["outcome"], "reason": gate.get("reason"), "wall_seconds": gate.get("wall_seconds"),
                        "order": gate.get("order"),
                        "maximum_settle_penetration_m": gate.get("maximum_settle_penetration_m"),
                        "maximum_cycle_penetration_m": gate.get("maximum_cycle_penetration_m")},
               "figure": str(fig_path.relative_to(ROOT))}
    atomic_write(out, json.dumps(out_rec, indent=1) + "\n")
    verdict = ("PASS" if res["feasible"] and not loose else
               "REROLL settled pooling" if not res["feasible"] else "REROLL loose dish")
    print(f"[RESULT] {verdict} settled {tag_of(k)}: S planned {rec['S']:.4f} -> settled {res['score']:.4f}, "
          f"worst {res['worst']:.4f}, racked {len(entries)}/24", flush=True)
    return 0


def stage_ban(k):
    """The benchmark's --ban rule on one of this run's gates: the pose/pair its penetration blames -> OUT/bans."""
    gate, rec = read(gate_dir(k) / "result.json"), read(search_path(k))
    pairs = B.penetration_bans(gate, rec["entries"]) if gate and gate.get("outcome") == "penetration_failure" else []
    atomic_write(ban_path(k), json.dumps({"outcome": None if gate is None else gate.get("outcome"), "pairs": pairs},
                                         indent=1) + "\n")
    print(f"[RESULT] PASS ban {tag_of(k)}: {len(pairs)} pair(s) {pairs}", flush=True)
    return 0


def gate_unit(sched, k):
    """The joint Isaac gate (frigidaire_bench.gate_unit) with our peak-penetration limit; an existing result is reused;
    a rack-speed flake (closure_failure) is retried lower-first once. Returns the outcome."""
    gd = gate_dir(k)
    if (gd / "result.json").is_file():
        return gate_outcome(k)
    argv = ["code/frigidaire/scripts/experiment/frigidaire_initial_state_validate.py",
            "--manifest", B.rel(search_path(k).with_name(f"{tag_of(k)}.gate_manifest.json")), "--out-dir", B.rel(gd),
            "--max-wall-seconds", "600", "--peak-penetration-m", str(GATE_PEAK_PEN_M), "--headless", "--device", "cpu"]
    sched.run("kit", argv, f"gate_{tag_of(k)}")
    outcome = gate_outcome(k)
    if outcome == "closure_failure":
        alt = Path(str(gd) + "_lower_first")
        argv2 = argv[:-3] + ["--order", "lower_first", "--headless", "--device", "cpu"]
        argv2[argv2.index("--out-dir") + 1] = B.rel(alt)
        sched.run("kit", argv2, f"gate_{tag_of(k)}_lower_first")
        res = read(alt / "result.json")
        if res and res.get("outcome") == "accepted":
            sched.container_sh(["mv", str(gd.resolve()), str(gd.resolve()) + "_upper_first"])
            sched.container_sh(["cp", "-a", str(alt.resolve()), str(gd.resolve())])
            outcome = "accepted"
    return outcome


def live_search(k):
    r = subprocess.run(["docker", "exec", "dishsim-isaac", "ps", "-eo", "args"], capture_output=True, text=True)
    return any(f"frigidaire_hotec_top5.py --search-start {k} --level" in line and "python3" in line
               for line in r.stdout.splitlines())


def render_dir(k):
    return MEDIA / "renders" / tag_of(k)


def stage_collect():
    cands = candidates()
    chosen = select_top(accepted(cands))
    chosen_ids = {c["start"] for c in chosen}
    level = max((c["level"] for c in cands), default=None)
    rows = []
    for rank, c in enumerate(chosen, 1):
        k, st = c["start"], c["settled"]
        rd = render_dir(k)
        ev = read(rd / f"{tag_of(k)}_evidence.json") or {}
        others = [o for o in chosen if o["start"] != k]
        rows.append({"rank": rank, "start": k, "S_planned": c["S"], "W_planned": c["W"], "S_settled": st["S_settled"],
                     "W_settled": st["W_settled"], "pooling_settled": sum(o["pools"] for o in st["per_object"]),
                     "gate_wall_s": st["gate"]["wall_seconds"] or 0.,
                     "nested_pairs": st.get("nested_pairs_settled", []),
                     "min_distance_to_others": min((distance(c["entries"], o["entries"]) for o in others), default=None),
                     "by_rack": dict(Counter(f"{e['rack']}:{e['kind']}" for e in c["entries"])),
                     "render": {"result": ev.get("result"), "settle_reason": ev.get("settle_reason"),
                                "all_contained": ev.get("all_contained"), "error": ev.get("error")},
                     "files": {"layout_planned": str(search_path(k).with_name(f"{tag_of(k)}.layout.json").relative_to(ROOT)),
                               "layout_settled": str(settled_layout_path(k).relative_to(ROOT)),
                               "gate": str((gate_dir(k) / "result.json").relative_to(ROOT)),
                               "exposure_figure": legacy_data_path(st["figure"]),
                               **{name: str((rd / f"{tag_of(k)}_{name}").relative_to(ROOT))
                                  for name in ("placed.png", "settled.png", "collision.png", "orbit.mp4", "settle.json",
                                               "evidence.json")
                                  if ev.get("result") == "PASS" and (rd / f"{tag_of(k)}_{name}").is_file()}}})
    others = []
    for c in sorted(cands, key=lambda c: (-(c.get("S") or 0), c["start"])):
        if c["start"] in chosen_ids:
            continue
        if not c.get("feasible"):
            why = c.get("reason") or "infeasible"
        elif c["gate"] != "accepted":
            why = f"gate {c['gate']}"
        elif not c["settled"]:
            why = "no settled score"
        elif not c["settled"]["feasible_settled"]:
            why = "settled pooling: " + ", ".join(o["id"] for o in c["settled"]["per_object"] if o["pools"])
        elif c["settled"]["racked"] != len(ROSTER):
            why = "loose dish after the settle: " + ", ".join(c["settled"]["loose"])
        else:
            near = min(chosen, key=lambda o: distance(c["entries"], o["entries"]))
            d = distance(c["entries"], near["entries"])
            why = (f"only {d} dishes differ from start_{near['start']:02d}" if d < DISTINCT_FALLBACK
                   else f"distinct enough, ranked below the top {TOP}")
        others.append({"start": c["start"], "S_planned": c.get("S"), "why_not": why})
    capacity = {}
    for cp in sorted((OUT / "capacity").glob("level*_part*.json")):
        d = read(cp)
        c2 = capacity.setdefault(str(d["level"]), [0, 0])
        c2[0] += sum(t["complete"] for t in d["tries"])
        c2[1] += len(d["tries"])
    summary = {"purpose": "hotec_top5", "date": DATE, "level": level, "rules": LEVELS.get(level),
               "capacity": {lv: f"{a}/{b}" for lv, (a, b) in capacity.items()}, "max_sweeps": MAX_SWEEPS,
               "variant": VARIANT, "appliance_tol_m": APPLIANCE_TOL_M, "dish_tol_m": DISH_TOL_M,
               "gate_peak_penetration_m": GATE_PEAK_PEN_M,
               "nesting": "scored, not forbidden (nested pairs listed per load)",
               "distinct_min_dishes": DISTINCT_MIN, "distinct_fallback_dishes": DISTINCT_FALLBACK, "starts": len(cands),
               "feasible": sum(bool(c.get("feasible")) for c in cands),
               "gated": sum(c["gate"] is not None for c in cands), "accepted": len(accepted(cands)),
               "top": rows, "not_chosen": others}
    atomic_write(OUT / "summary.json", json.dumps(summary, indent=1) + "\n")
    lines = [f"# Top-{TOP} exposure arrangements, all 24 HOTEC dishes ({DATE})", "",
             f"Rules level {level}: {LEVELS.get(level)}. Distinct = at least {DISTINCT_MIN} of 24 dishes in a different slot; "
             f"ranks that rule cannot fill take loads at least {DISTINCT_FALLBACK} apart (plates and cups are forced, only bowls vary).",
             f"Capacity (complete / tried first-fit packings of all 24, per level): {summary['capacity']}. "
             f"Ascent: up to {MAX_SWEEPS} sweeps.",
             f"Starts {summary['starts']}, feasible {summary['feasible']}, gated {summary['gated']}, "
             f"accepted {summary['accepted']}.", "",
             "| rank | start | planned S | settled S | worst dish (settled) | pooling | nested bowl pairs | gate s | min distance | render |",
             "|---|---|---|---|---|---|---|---|---|---|"]
    for r in rows:
        lines.append(f"| {r['rank']} | start_{r['start']:02d} | {r['S_planned']:.3f} | {r['S_settled']:.3f} | "
                     f"{r['W_settled']:.3f} | {r['pooling_settled']} | {len(r['nested_pairs'])} | {r['gate_wall_s']:.0f} | "
                     f"{r['min_distance_to_others']} | "
                     f"{r['render']['result']} |")
    lines += ["", "Not chosen:", ""] + [f"- start_{o['start']:02d} (S {o['S_planned'] if o['S_planned'] is None else round(o['S_planned'], 3)}): "
                                          f"{o['why_not']}" for o in others]
    atomic_write(OUT / "summary.md", "\n".join(lines) + "\n")
    print("\n".join(lines), flush=True)
    ok = len(rows) == TOP and all(r["render"]["result"] == "PASS" for r in rows)
    print(f"[RESULT] {'PASS' if ok else 'FAIL'} collect: {len(rows)}/{TOP} arrangements, "
          f"{sum(r['render']['result'] == 'PASS' for r in rows)}/{len(rows)} renders PASS", flush=True)
    return 0


# --------------------------------------------------------------------------- host orchestration

LIVE_RE = re.compile(rf"frigidaire_hotec_top5\.py --(prepare|capacity|search-start|settle-score|ban|collect)|top5_{DATE}_{VARIANT}/")


def live_jobs(container):
    """Stages of this run still alive in the container: stopping the host orchestrator does not stop the jobs it
    started with docker exec, and a resume next to them would exceed the Kit slots and double-write gate dirs."""
    r = subprocess.run(["docker", "exec", container, "ps", "-eo", "pid,args"], capture_output=True, text=True)
    if r.returncode != 0:
        raise SystemExit(f"[RESULT] FAIL liveness check: {r.stderr.strip()}")
    return [line.strip() for line in r.stdout.splitlines()[1:]
            if LIVE_RE.search(line) and ("python3" in line or "isaaclab.sh" in line)]


def main_all(args):
    B.LOGS = LOGS                                            # the Scheduler's per-stage logs (our module copy only)
    if args.kit_cores:                                       # a core/slot partition when another scheduler runs next to
        B.KIT_CORES = tuple(args.kit_cores)                  # this one (total Kit jobs on GPU 1 stay <= 3)
    if args.py_cores:
        B.PY_CORES = tuple(args.py_cores)
    kit_n, py_n = min(args.kit_jobs, len(B.KIT_CORES), 3), min(args.py_jobs, len(B.PY_CORES), 2)
    sched = B.Scheduler(kit_n, py_n)
    live = [j for j in live_jobs(sched.containers[0][0]) if "--search-start" not in j]   # helper searches are waited on
    if live:
        print("[RESULT] FAIL jobs of an earlier top5 run are still live in the container; let them finish or stop them "
              "by PID, then resume:\n" + "\n".join(live), flush=True)
        return 1
    signal.signal(signal.SIGINT, lambda *_: (print("[RESULT] FAIL interrupted: container jobs keep running; resume "
                                                    "--all only after they end", flush=True), os._exit(130)))
    py = lambda argv, name: sched.run("py", [SCRIPT, *map(str, argv)], name)
    level = None
    for lv in LEVELS:
        line = py(["--prepare", "--level", lv], f"prepare_L{lv}")
        print(f"[INFO] prepare level {lv}: {line}", flush=True)
        if "PASS" not in line:
            print(f"[RESULT] FAIL prepare level {lv}", flush=True)
            return 1
        lines = B.orchestrate(range(PARTS), lambda p: py(["--capacity", "--level", lv, "--part", p, "--parts", PARTS],
                                                         f"capacity_L{lv}_p{p}"), jobs=PARTS)
        bad = [p for p, ln in zip(range(PARTS), lines)
               if "PASS" not in ln or not (OUT / "capacity" / f"level{lv}_part{p}.json").is_file()]
        if bad:                                              # a crashed part must not silently relax the rules
            print(f"[RESULT] FAIL capacity level {lv}: part(s) {bad} did not finish (see {LOGS}); resume --all, "
                  "finished parts are kept", flush=True)
            return 1
        n = len(capacity_packings(lv))
        print(f"[INFO] level {lv} ({LEVELS[lv]}): {n}/{CAPACITY_TRIES} complete packings", flush=True)
        if n >= CAPACITY_NEED:
            level = lv
            break
    if level is None:
        print("[RESULT] FAIL capacity: no rules level racks all 24 dishes; see the capacity records", flush=True)
        return 1

    def dominated_by(k, rec):
        """An earlier start with the same slots (distance 0) that already PASSED Isaac with planned S >= this one's:
        select_top always prefers it, so gating this start is redundant. A failed, pending or lower-S twin never
        suppresses a gate (Q4 promotion)."""
        for j in range(k):
            other = read(search_path(j))
            if (other and other.get("feasible") and settled_path(j).is_file() and other["S"] >= rec["S"]
                    and distance(rec["entries"], other["entries"]) == 0):
                return j
        return None

    def chain(k):
        tag = tag_of(k)
        while not search_path(k).is_file() and live_search(k):   # a helper loop is searching this start: wait for it
            time.sleep(20)
        if not search_path(k).is_file():
            py(["--search-start", k, "--level", level], f"search_{tag}")
        rec = read(search_path(k))
        if rec is None:
            return f"{tag}: search failed"
        if rec.get("level") != level:
            return f"{tag}: record made at rules level {rec.get('level')}, not {level}"
        if not rec.get("feasible"):
            return f"{tag}: {rec.get('reason')}"
        dup = dominated_by(k, rec)
        if dup is not None:
            return f"{tag}: same slots as {tag_of(dup)} (accepted, planned S >= this one); not gated"
        if gate_outcome(k) is None:
            better = select_top([c for c in accepted(candidates()) if c["S"] > rec["S"]])
            if len(better) >= TOP:                          # 5 distinct higher-S loads already passed Isaac: this one
                return f"{tag}: not gated (5 distinct accepted loads plan a higher S)"   # can never enter the top 5
            gate_unit(sched, k)
        outcome = gate_outcome(k)
        if outcome == "penetration_failure" and not ban_path(k).is_file():
            py(["--ban", k], f"ban_{tag}")                  # later starts avoid the pose/pair it blames
        if outcome != "accepted":
            return f"{tag}: gate {outcome}"
        if not settled_path(k).is_file():
            py(["--settle-score", k], f"settle_{tag}")
        st = read(settled_path(k))
        if not st:
            return f"{tag}: settled score failed"
        if not st["feasible_settled"] or st["racked"] != len(ROSTER):
            return f"{tag}: gate accepted but the settled load breaks a rule (pooling or loose dish)"
        return f"{tag}: accepted, S planned {rec['S']:.4f}, settled {st['S_settled']:.4f}"

    n = STARTS
    by_s = lambda ks: sorted(ks, key=lambda k: -((read(search_path(k)) or {}).get("S") or 0.))   # best planned S first
    status = B.orchestrate(by_s(range(n)), chain, jobs=kit_n + py_n)
    chosen = select_top(accepted(candidates()))
    while len(chosen) < TOP and n < MAX_STARTS:
        more = range(n, min(n + EXTRA_STARTS, MAX_STARTS))
        n = more.stop
        print(f"[INFO] {len(chosen)}/{TOP} distinct accepted after {more.start} starts: {len(more)} more", flush=True)
        status += B.orchestrate(by_s(more), chain, jobs=kit_n + py_n)
        chosen = select_top(accepted(candidates()))
    print("\n".join(status), flush=True)

    def render(c):
        k = c["start"]
        tag, rd = tag_of(k), render_dir(k)
        ev = read(rd / f"{tag}_evidence.json")
        if ev and ev.get("result") == "PASS":
            return f"{tag}: render exists (PASS)"
        if rd.exists():                                      # a failed or partial render refuses to overwrite: move it aside
            sched.container_sh(["mv", str(rd.resolve()), f"{rd.resolve()}_failed_{int(time.time())}"])
        line = sched.run("kit", ["code/frigidaire/scripts/evaluation/frigidaire_hotec_load.py",
                                 "--layout", B.rel(settled_layout_path(k)), "--assets", ASSETS_REL,
                                 "--out-dir", B.rel(rd), "--tag", tag, "--headless", "--enable_cameras", "--device", "cpu"],
                         f"render_{tag}")
        return f"{tag}: {line}"
    print("\n".join(B.orchestrate(chosen, render, jobs=kit_n)), flush=True)
    line = py(["--collect"], "collect")
    print((OUT / "summary.md").read_text() if (OUT / "summary.md").is_file() else "(no summary)", flush=True)
    print(f"[RESULT] {'PASS' if 'PASS' in line else 'FAIL'} top5 ({len(chosen)}/{TOP} arrangements, level {level})",
          flush=True)
    return 0


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    mode = ap.add_mutually_exclusive_group(required=True)
    mode.add_argument("--all", action="store_true", help="host: capacity -> search -> gates -> settled scores -> renders")
    mode.add_argument("--prepare", action="store_true", help="families + masks of --level (cached)")
    mode.add_argument("--capacity", action="store_true", help="shuffled first-fit packings of all 24 at --level")
    mode.add_argument("--search-start", type=int, metavar="K", help="one start: packing -> ascent -> record")
    mode.add_argument("--settle-score", type=int, metavar="K", help="score an accepted gate's settled poses + figure")
    mode.add_argument("--ban", type=int, metavar="K", help="ban the pose/pair a failed gate's penetration blames")
    mode.add_argument("--collect", action="store_true", help="select the top 5 and write summary.{json,md}")
    ap.add_argument("--level", type=int, choices=sorted(LEVELS), default=0)
    ap.add_argument("--part", type=int, default=0)
    ap.add_argument("--parts", type=int, default=PARTS)
    ap.add_argument("--kit-jobs", type=int, default=3)
    ap.add_argument("--py-jobs", type=int, default=2)
    ap.add_argument("--kit-cores", nargs="*", default=None, help='host: Kit core sets, e.g. "16-23" (partition)')
    ap.add_argument("--py-cores", nargs="*", default=None, help='host: Kit-free core sets, e.g. "28-31" "32-35"')
    args = ap.parse_args(argv)
    if args.all:
        return main_all(args)
    if args.prepare:
        return stage_prepare(args.level)
    if args.capacity:
        return stage_capacity(args.level, args.part, args.parts)
    if args.search_start is not None:
        return stage_search(args.search_start, args.level)
    if args.settle_score is not None:
        return stage_settle_score(args.settle_score)
    if args.ban is not None:
        return stage_ban(args.ban)
    return stage_collect()


if __name__ == "__main__":
    raise SystemExit(main())
