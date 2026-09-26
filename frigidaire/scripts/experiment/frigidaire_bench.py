#!/usr/bin/env python3
# Copyright (c) 2026, dishsim project.
# SPDX-License-Identifier: BSD-3-Clause
"""HOTEC rearrangement benchmark on the Frigidaire FDPC4221AS twin: Kit-free library and CLI.

Tiers (the user's design, 2026-09-23, with 7 bowls: the 8th did not fit physically, see TIERS):
easy = 7 bowls, medium = 8 plates + 7 bowls, hard = 8 plates + 7 bowls + 8 cups. An instance starts with
n dishes in messy stacks on a 1.8 x 0.6 m counter slab (n uniform in [N/4, 3N/4]) and the rest dropped
messily into the racks; the counter allowance is
n + 3 / n + 1 / n + 0. There is no hand-given goal: the goal is the highest-exposure arrangement
found per instance by coordinate ascent (revision-5 score S, HOTEC kinds registered), where rack
dishes may stay where they start (keep-in-place). Two tracks: A reaches that goal exactly; B has no
goal, success = every dish racked, scored by S. Physics (start settle, goal gate, per-move settle,
end check) runs in frigidaire_bench_kit.py; everything here is Kit-free.

    scripts/run_py.sh frigidaire/scripts/experiment/frigidaire_bench.py --capacity
    scripts/run_py.sh frigidaire/scripts/experiment/frigidaire_bench.py --generate --tiers easy medium hard --seeds 0 --jobs 3
"""
from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import math
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
sys.path[:0] = [str(ROOT / "src"), str(ROOT / "frigidaire/src")]

ASSETS = ROOT / "assets/models/hotec_wheatstraw/v2"
KINDS = ("plate", "bowl", "cup")
# 7 bowls in every tier: the 8th-bowl poses settle past the tub wall (outer front-right) or wedge (third rear pair)
# in Isaac goal gates (2026-09-23); without plates an 8-bowl packing exists but first-fit found it once in 300
# shuffles, so open-track planners could not rack easy at all within 60 s
TIERS = {"easy": {"inventory": {"bowl": 7}, "slack": 3},
         "medium": {"inventory": {"plate": 8, "bowl": 7}, "slack": 1},
         "hard": {"inventory": {"plate": 8, "bowl": 7, "cup": 8}, "slack": 0}}
TIER_INDEX = {name: i for i, name in enumerate(TIERS)}
COUNTER = {"size_m": (1.8, .6, .04), "center_m": (0., 0., .894), "top_z_m": .914}
CELL_PITCH_M = .25                  # parking cells: a 229 mm plate fits one cell
CELL_HOVER_M = .002
TUB_INNER_X_M = .277                # TubRight inner face in the rack frame (geometry.py: box .284, half width .007)
FRONT_RIGHT_MAX_X_M = .165          # front-right lower bowls (user). Poses at x .170-.190 clear the wall as COMMANDED but
                                    # settle outward off their 60 mm lift: lip at x .286 > .277, caught the cabinet on
                                    # retraction in every medium/hard goal gate (2026-09-23)
DROPPED_SLOTS = ("lower_mid",)      # the unstable mid-zone bowl (runs v10, v13)
GOAL_HOVER_M = .003                 # goal-track commands release 3 mm above the settled goal
AT_GOAL = {"bowl": (.015, .020, 15.), "cup": (.015, .020, 15.), "plate": (.018, .020, 16.)}  # lateral, dz, tilt
SEARCH = {"max_sweeps": 2, "top": 6, "random": 2, "min_gain": 1e-4}
OUT = ROOT / "results/benchmark/frigidaire_hotec"
MEDIA = ROOT / "media/benchmark/frigidaire_hotec"
BENCH_CAMERA = {"initial": ((2.05, -2.65, 2.05), (0., -.22, .62),        # frames the counter pile and both extended racks
                            {"focal_length": 50., "horizontal_aperture": 36.})}
COUNTER_COLOR = (.72, .70, .66)                                            # frigidaire_planner_video.py slab colour


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def tableware():
    return {kind: str(ASSETS / f"{kind}.usda") for kind in KINDS}


def roster(tier):
    """Canonical object ids of a tier, e.g. plate_01..08, bowl_01..08."""
    return [(f"{kind}_{k:02d}", kind) for kind, n in TIERS[tier]["inventory"].items() for k in range(1, n + 1)]


def draw_n(tier, seed):
    """Dishes on the counter at the start: uniform in [N/4, 3N/4], drawn from (tier, seed) only."""
    total = sum(TIERS[tier]["inventory"].values())
    rng = np.random.default_rng([int(seed), TIER_INDEX[tier], 7])
    return int(rng.integers(math.ceil(total / 4), math.floor(3 * total / 4) + 1))


def cap_of(tier, n):
    return int(n) + TIERS[tier]["slack"]


# --------------------------------------------------------------------------- lazy modules

_HX = None


def HX():
    """The HOTEC exposure-search script as a module (families, scorer registration, full_score)."""
    global _HX
    if _HX is None:
        spec = importlib.util.spec_from_file_location("hotec_exposure_search",
                                                      ROOT / "frigidaire/scripts/evaluation/frigidaire_hotec_exposure_search.py")
        _HX = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(_HX)
    return _HX


def quat_matrix(q):
    x, y, z, w = (float(v) for v in q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def posed_points(points, position, quaternion):
    return np.asarray(points) @ quat_matrix(quaternion).T + np.asarray(position, dtype=float)


def tub_clear(points, position, quaternion, margin=0.):
    """Every visual point inside the tub side walls (rack frame |x| <= TUB_INNER_X_M - margin)."""
    return bool(np.abs(posed_points(points, position, quaternion)[:, 0]).max() <= TUB_INNER_X_M - margin)


def family_ok(c, points):
    """The benchmark's family filters: no mid-zone bowl, front-right bowls at x <= 0.165, tub-wall clear."""
    if c["slot"] in DROPPED_SLOTS:
        return False
    if c["slot"] == "lower_frontright" and c["position"][0] > FRONT_RIGHT_MAX_X_M + 1e-9:
        return False
    return tub_clear(points, c["position"], c["quaternion_xyzw"])


# --------------------------------------------------------------------------- families

def families(assets=ASSETS):
    """HOTEC candidate families (rack-local, closed-appliance frame) with the benchmark's filters."""
    fam, parts, points = HX().candidate_families(assets)
    return {kind: [(c, o) for c, o in lst if family_ok(c, points[kind])] for kind, lst in fam.items()}, parts, points


def family_digest(fam):
    h = hashlib.sha256()
    for kind in KINDS:
        for c, _ in fam.get(kind, []):
            h.update(json.dumps([c["kind"], c["rack"], c["slot"], c["variant"],
                                 np.round(c["position"], 6).tolist(), np.round(c["quaternion_xyzw"], 6).tolist()]).encode())
    return h.hexdigest()


def appliance_free(fam, parts, points, cache=OUT / "families_appliance_free.json"):
    """Per kind, the mask of candidates that clear the empty closed appliance (cached by family digest)."""
    key = family_digest(fam)
    if Path(cache).is_file():
        data = json.loads(Path(cache).read_text())
        if data.get("family_digest") == key:
            return {k: np.asarray(v, dtype=bool) for k, v in data["masks"].items()}
    world = hotec_world(parts, points)
    masks = {kind: np.array([not world.collides(world.candidate_objects(c)) for c, _ in fam[kind]], dtype=bool)
             for kind in KINDS if kind in fam}
    Path(cache).parent.mkdir(parents=True, exist_ok=True)
    Path(cache).write_text(json.dumps({"family_digest": key, "masks": {k: v.tolist() for k, v in masks.items()}}) + "\n")
    return masks


SWEEP_STEPS = 25                        # rack positions checked from extended to closed (2 cm apart on the lower rack)


_SWEEP = None


def sweep_components():
    """(checker, [(rack, rack frame, {"Cabinet", "UpperRack"} bodies)]) at SWEEP_STEPS slide positions in the goal
    gate's upper-first order: the upper rack retracts with the lower one extended, then the lower rack (with the
    basket) under the closed upper rack. Frames come from the extended baseline; the slides are linear in y."""
    global _SWEEP
    if _SWEEP is None:
        from dishsim_frigidaire.asset import BODY_POSITIONS
        from dishsim_frigidaire.initial_state_candidates import COMPONENT_NAMES
        checker = bench_checker()
        base = json.loads((OUT / "baseline" / "result.json").read_text())
        base = base.get("snapshot", base)["poses"]
        shifted = lambda pose, dy: {"position_m": [pose["position_m"][0], pose["position_m"][1] + dy, pose["position_m"][2]],
                                    "quaternion_xyzw": list(pose["quaternion_xyzw"])}
        steps = []
        for rack in ("UpperRack", "LowerRack"):
            travel = BODY_POSITIONS[rack][1] - base[rack]["position_m"][1]
            for f in np.linspace(0., 1., SWEEP_STEPS):
                frames = {name: base[name] for name in COMPONENT_NAMES}
                if rack == "LowerRack":
                    frames["UpperRack"] = shifted(base["UpperRack"], BODY_POSITIONS["UpperRack"][1] - base["UpperRack"]["position_m"][1])
                    frames["SilverwareBasket"] = shifted(base["SilverwareBasket"], f * travel)
                frames[rack] = shifted(base[rack], f * travel)
                checker.update_components(frames)
                steps.append((rack, frames[rack], {"Cabinet": checker.components["Cabinet"], "UpperRack": checker.components["UpperRack"]}))
        _SWEEP = (checker, steps)
    return _SWEEP


def sweep_hits(c):
    """True when a rack-local candidate hits the cabinet (or, in the lower rack, the closed upper rack) anywhere on
    the rack's way in: a pose can clear the closed appliance and still catch the tub opening (the outer
    front-right bowl, x .175, caught the cabinet at lower_slide -0.155 in every medium/hard goal gate, 2026-09-23)."""
    from dishsim_frigidaire.random_poses import compose_pose
    checker, steps = sweep_components()
    for rack, frame, comps in steps:
        if rack != c["rack"]:
            continue
        p, q = compose_pose(frame["position_m"], frame["quaternion_xyzw"], list(c["position"]), list(c["quaternion_xyzw"]))
        body = checker._body(c["kind"], {"position_m": p.tolist(), "quaternion_xyzw": q.tolist()}, "sweep")
        if not checker.pair(body, comps["Cabinet"])["valid"] or (rack == "LowerRack" and not checker.pair(body, comps["UpperRack"])["valid"]):
            return True
    return False


def family_masks(fam, parts, points, cache=OUT / "families_retraction_free.json"):
    """appliance_free AND sweep-clear, per kind (the sweep cached by family digest)."""
    masks = appliance_free(fam, parts, points)
    key = f"{family_digest(fam)}|sweep{SWEEP_STEPS}"
    if Path(cache).is_file():
        data = json.loads(Path(cache).read_text())
        if data.get("key") == key:
            return {k: masks[k] & np.asarray(v, dtype=bool) for k, v in data["masks"].items()}
    sweep = {kind: np.array([bool(ok) and not sweep_hits(c) for (c, _), ok in zip(fam[kind], masks[kind])], dtype=bool)
             for kind in masks}
    Path(cache).write_text(json.dumps({"key": key, "masks": {k: v.tolist() for k, v in sweep.items()}}) + "\n")
    return {k: masks[k] & sweep[k] for k in masks}


def hotec_world(parts, points):
    """loading.CollisionWorld (closed appliance statics) with the HOTEC parts under the kind names."""
    from dishsim_frigidaire.loading import CollisionWorld
    from dishsim_frigidaire.paths import ASSET_DIR
    world = CollisionWorld(ASSET_DIR)
    for kind in KINDS:
        world.parts[kind], world.points[kind] = parts[kind], points[kind]
    return world


GOAL_CLEARANCE_M = .003   # goal dishes keep >= 3 mm apart (planner.md asks 5 mm; 5 mm leaves every tier a bowl short)


class DishSet:
    """FCL of placed dishes only (no appliance): AABB prefilter, then a pair counts as colliding below
    GOAL_CLEARANCE_M -- goals built one dish at a time must not touch their neighbours."""

    def __init__(self, parts, points):
        import fcl
        self.fcl, self.parts, self.points = fcl, parts, points
        self.items = {}                                    # id -> (manager, lo, hi, entry)

    def _manager(self, c):
        from dishsim_frigidaire.asset import BODY_POSITIONS
        fcl = self.fcl
        rot = quat_matrix(c["quaternion_xyzw"])
        pos = np.asarray(c["position"], dtype=float) + BODY_POSITIONS[c["rack"]]
        objs = [fcl.CollisionObject(p.geometry, fcl.Transform(rot @ p.rotation, rot @ p.translation + pos))
                for p in self.parts[c["kind"]]]
        m = fcl.DynamicAABBTreeCollisionManager()
        m.registerObjects(objs)
        m.setup()
        pts = self.points[c["kind"]] @ rot.T + pos
        return m, pts.min(0) - .003, pts.max(0) + .003, objs

    def add(self, oid, c):
        m, lo, hi, objs = self._manager(c)
        self.items[oid] = (m, lo, hi, c, objs)

    def remove(self, oid):
        self.items.pop(oid, None)

    def collides(self, c, skip=(), clearance=GOAL_CLEARANCE_M):
        m, lo, hi, _ = self._manager(c)
        fcl = self.fcl
        for oid, (other, olo, ohi, _, _) in self.items.items():
            if oid in skip or np.any(hi + clearance < olo) or np.any(ohi + clearance < lo):
                continue
            data = fcl.DistanceData(request=fcl.DistanceRequest(), result=fcl.DistanceResult())
            m.distance(other, data, fcl.defaultDistanceCallback)
            if data.result.min_distance < clearance:
                return True
        return False


def nests(points, c, o, others):
    """True when bowl candidate c would nest with a same-rack bowl in ``others``: their depth slabs overlap along
    the bowl axis (claims.slabs_disjoint) AND their rack-frame xy footprints overlap. The claims rule alone calls two
    bowls side by side "nested" whatever their distance, which left the lower rack one bowl short (G2 7 / 15 / 23)."""
    from dishsim_frigidaire.claims import slabs_disjoint
    o = quat_matrix(c["quaternion_xyzw"]) if o is None else o          # keep-in-place candidates carry no matrix

    def xy(e):
        P = posed_points(points["bowl"], e["position"], e["quaternion_xyzw"])
        return P[:, :2].min(0), P[:, :2].max(0)
    lo, hi = xy(c)
    for e in others:
        if e["kind"] != "bowl" or e["rack"] != c["rack"]:
            continue
        elo, ehi = xy(e)
        if np.any(hi < elo) or np.any(ehi < lo):
            continue
        if not slabs_disjoint(points["bowl"], (e, quat_matrix(e["quaternion_xyzw"])), (c, o)):
            return True
    return False


def cand_key(c):
    """A candidate POSE: rack + position (mm) + quaternion. The families list one pose under several slot names
    (lower_rear / lower_rearright), so a slot-keyed ban missed the same pose (2026-09-23)."""
    p = ",".join(f"{v * 1e3:.0f}" for v in c["position"])
    q = ",".join(f"{v:.3f}" for v in c["quaternion_xyzw"])
    return f"{c['rack']}|{p}|{q}"


def cand_keys(c):
    """The pose key, plus the older slot key that the first ban files used."""
    return (cand_key(c), f"{c['rack']}|{c['slot']}|{c['variant']}")


def load_bans(out=OUT):
    """Candidate pairs that penetrated together in an Isaac goal gate (one bans.json per failed attempt)."""
    bans = set()
    paths = [*(Path(out) / "instances" / "attempts").glob("*/bans.json"), *(Path(out) / "bans").glob("*.json")]
    for path in paths:                                      # bans/ keeps those of attempts moved aside
        data = json.loads(path.read_text())
        if data.get("outcome") in ("solo", "solo_overlap"):  # the retired any-order rule: those bans are wrong
            continue
        bans |= {frozenset(pair) for pair in data["pairs"]}
    return bans


def banned(c, others, bans):
    """A banned pose (a singleton: not self-supporting) or a banned pair with any placed dish."""
    if not bans:
        return False
    keys = cand_keys(c)
    return (any(frozenset((k,)) in bans for k in keys)
            or any(frozenset((k, ke)) in bans for e in others for k in keys for ke in cand_keys(e)))


def validated_goals(tier, out=OUT):
    """Commanded goal entries of the tier's accepted instances (each passed an Isaac goal gate)."""
    out_list = []
    for path in sorted((Path(out) / "instances" / tier).glob(f"{tier}_s*.json")):
        inst = json.loads(path.read_text())
        kinds = {o["object_id"]: o["kind"] for o in inst["objects"]}
        out_list.append([{"id": oid, "kind": kinds[oid], "rack": g["rack"], "slot": g["slot"], "variant": g["variant"],
                          "position": list(g["command_rack_local_pose"]["position_m"]),
                          "quaternion_xyzw": list(g["command_rack_local_pose"]["quaternion_xyzw"])}
                         for oid, g in inst["goal"]["objects"].items() if not g.get("keep")])
    return [t for t in out_list if len(t) == len(roster(tier))]


def stage_ban(attempt_dir):
    """After a goal gate failed on dish-on-dish penetration: ban the offending pair of candidate poses."""
    from dishsim_frigidaire.random_poses import LIMITS
    attempt_dir = Path(attempt_dir)
    gate = json.loads((attempt_dir / "gate" / "result.json").read_text())
    goal = {e["id"]: e for e in json.loads((attempt_dir / "goal.json").read_text())["entries"]}
    event = gate.get("maximum_settle_penetration_event") or {}
    pairs = []
    for name, depth in (event.get("contact_pair_penetration_m") or {}).items():
        a, b = name.split("|")
        if depth < LIMITS["peak_penetration_m"]:
            continue
        if a in goal and b in goal:
            pairs.append(sorted((cand_key(goal[a]), cand_key(goal[b]))))
        elif (a in goal) != (b in goal):                 # a dish into the appliance: the pose alone is bad
            dish = goal[a if a in goal else b]
            if dish.get("variant") != "keep":
                pairs.append([cand_key(dish)])
    (attempt_dir / "bans.json").write_text(json.dumps({"outcome": gate.get("outcome"), "pairs": pairs}, indent=1) + "\n")
    print(f"[RESULT] PASS ban {len(pairs)} pair(s): {pairs}", flush=True)
    return 0


def free_iter(dish, entries, fam, masks, parts, points, dishes, bans=frozenset()):
    """Family candidates of ``dish``'s kind that clear the appliance (cached), the other dishes (``dishes``
    must hold every entry), the one-per-slot rule (plates, cups) and the non-nesting rule (bowls)."""
    kind = dish["kind"]
    others = [e for e in entries if e["id"] != dish["id"]]
    used = {(e["rack"], e["slot"]) for e in others}
    for (c, o), ok in zip(fam[kind], masks[kind]):
        if not ok:
            continue
        if kind != "bowl" and (c["rack"], c["slot"]) in used:
            continue
        if kind == "bowl" and nests(points, c, o, others):
            continue
        if banned(c, others, bans):
            continue
        if dishes.collides(c, skip=(dish["id"],)):
            continue
        yield c, o


# --------------------------------------------------------------------------- goal search

def keep_candidates(instance, points):
    """Rack dishes that may stay where they start: not pooling, tub-clear, not resting on another dish."""
    from dishsim_frigidaire import exposure as E
    from dishsim_frigidaire.asset import BODY_POSITIONS
    supported = {b for _, b in instance.get("support", [])}
    out = {}
    for o in instance["objects"]:
        if o["start"] == "Counter" or o["object_id"] in supported:
            continue
        local = o["rack_local_pose"]
        if not tub_clear(points[o["kind"]], local["position_m"], local["quaternion_xyzw"]):
            continue
        p, q = E.compose_pose(BODY_POSITIONS[o["rack"]], E.IDENTITY, local["position_m"], local["quaternion_xyzw"])
        if E.pools(HX().SCORER_KIND[o["kind"]], p, q):
            continue
        keep = {"kind": o["kind"], "rack": o["rack"], "slot": f"keep_{o['object_id']}", "variant": "keep",
                "position": [local["position_m"][0], local["position_m"][1], local["position_m"][2] + KEEP_HOVER_M],
                "quaternion_xyzw": list(local["quaternion_xyzw"])}
        if not keep_clear(keep) or sweep_hits(keep):
            continue
        out[o["object_id"]] = keep
    return out


KEEP_HOVER_M = .002        # a kept dish is commanded 2 mm above its resting start pose: resting hulls overlap by more
                           # than the goal gate's 1 mm FCL preflight allows (a keep failed it, medium_s0_a3)


def keep_clear(c):
    """The goal gate's preflight for one rack-local pose: FCL (1 mm allowance) vs the extended appliance."""
    from dishsim_frigidaire.random_poses import compose_pose
    checker, _ = sweep_components()
    base = json.loads((OUT / "baseline" / "result.json").read_text())
    base = base.get("snapshot", base)["poses"]
    from dishsim_frigidaire.initial_state_candidates import COMPONENT_NAMES
    checker.update_components({name: base[name] for name in COMPONENT_NAMES})
    f = base[c["rack"]]
    p, q = compose_pose(f["position_m"], f["quaternion_xyzw"], list(c["position"]), list(c["quaternion_xyzw"]))
    return bool(checker.against_components(checker._body(c["kind"], {"position_m": p.tolist(), "quaternion_xyzw": q.tolist()}, "keep"))["valid"])


def entry_of(oid, c, keep=False):
    return {"id": oid, "kind": c["kind"], "rack": c["rack"], "slot": c["slot"], "variant": c["variant"],
            "position": [float(v) for v in c["position"]], "quaternion_xyzw": [float(v) for v in c["quaternion_xyzw"]],
            "keep": bool(keep)}


def first_fit(ids, fam, masks, parts, points, keeps=None, keep_ids=(), rng=None, bans=frozenset()):
    """Keeps first, then every other dish takes its kind's first free candidate in family order (shuffled per kind
    when ``rng`` is given); None if one fails."""
    if rng is not None:
        order = {k: rng.permutation(len(fam[k])) for k in fam}
        fam = {k: [fam[k][i] for i in order[k]] for k in fam}
        masks = {k: masks[k][order[k]] for k in masks}
    dishes, entries = DishSet(parts, points), []
    keeps = keeps or {}
    for oid in keep_ids:
        c = keeps[oid]
        if dishes.collides(c) or (c["kind"] == "bowl" and nests(points, c, None, entries)):
            continue
        entries.append(entry_of(oid, c, keep=True))
        dishes.add(oid, c)
    placed = {e["id"] for e in entries}
    for oid, kind in ids:
        if oid in placed:
            continue
        pick = next(free_iter({"id": oid, "kind": kind}, entries, fam, masks, parts, points, dishes, bans), None)
        if pick is None:
            return None
        entries.append(entry_of(oid, pick[0]))
        dishes.add(oid, pick[0])
    return entries


def ascend(entries, fam, masks, parts, points, score, isolated, rng, keeps=None, deadline=float("inf"),
           max_sweeps=SEARCH["max_sweeps"], top=SEARCH["top"], n_random=SEARCH["random"], log=print, bans=frozenset()):
    """Coordinate ascent: per dish, rank its free alternatives (plus its own keep pose) by isolated exposure,
    score the best ``top`` plus ``n_random`` random ones in full, keep a move when S rises by > min_gain.
    Stops after a sweep with no gain or ``max_sweeps`` sweeps or at ``deadline``."""
    keeps = keeps or {}
    current = score(entries)
    history = [{"sweep": 0, "piece": None, "score": current["score"]}]
    dishes = DishSet(parts, points)
    for e in entries:
        dishes.add(e["id"], e)
    sweeps = 0
    for sweep in range(1, max_sweeps + 1):
        sweeps, improved = sweep, False
        for i in range(len(entries)):
            if time.monotonic() > deadline:
                return entries, current, history, sweeps, "deadline"
            dish = entries[i]
            alts = list(free_iter(dish, entries, fam, masks, parts, points, dishes, bans))
            k = keeps.get(dish["id"])
            if k is not None and not dish.get("keep") and not dishes.collides(k, skip=(dish["id"],)) \
                    and not (k["kind"] == "bowl" and nests(points, k, None, [e for e in entries if e["id"] != dish["id"]])):
                alts.append((k, None))
            if not alts:
                continue
            iso = isolated([c for c, _ in alts])
            ranked = list(np.argsort(-iso)[:top])
            extra = [int(j) for j in rng.choice(len(alts), size=min(n_random, len(alts)), replace=False) if j not in ranked]
            best, best_entry, best_res = current["score"], None, None
            for j in ranked + extra:
                c = alts[j][0]
                trial = list(entries)
                trial[i] = entry_of(dish["id"], c, keep=c.get("variant") == "keep")
                res = score(trial)
                if res["feasible"] and res["score"] > best + SEARCH["min_gain"]:
                    best, best_entry, best_res = res["score"], trial[i], res
            if best_entry is not None:
                entries[i], current, improved = best_entry, best_res, True
                dishes.remove(dish["id"])
                dishes.add(dish["id"], best_entry)
                history.append({"sweep": sweep, "piece": dish["id"], "score": current["score"], "slot": best_entry["slot"]})
                log(f"[INFO] sweep {sweep} {dish['id']}: S {history[-2]['score']:.4f} -> {current['score']:.4f} ({best_entry['slot']})")
        if not improved:
            return entries, current, history, sweeps, "no_gain"
    return entries, current, history, sweeps, "max_sweeps"


class Scorer:
    """The revision-5 scorer with the HOTEC kinds registered (Warp; Kit-free processes only)."""

    def __init__(self, device=None):
        from dishsim_frigidaire import exposure as E
        self.E, self.hx = E, HX()
        self.info = self.hx.register_hotec(ASSETS)
        self.device = device or E.DEVICE
        self.isolated = self.hx.IsolatedExposure(self.device, 200, 32)
        self.calls = 0

    def score(self, entries):
        self.calls += 1
        return self.hx.full_score(entries, "bench", self.device, self.E.DEFAULTS["samples_per_object"], self.E.DEFAULTS["directions"])

    def iso(self, candidates):
        return self.isolated.evaluate(candidates)


def goal_search(instance, scorer, fam, masks, parts, points, seed=0, deadline=float("inf"), use_keeps=True,
                max_sweeps=SEARCH["max_sweeps"], shuffled_start=False, log=print, bans=frozenset(), templates=()):
    """Best-exposure complete load for an instance: keeps + first-fit, then coordinate ascent. ``shuffled_start``
    (re-rolls): the first-fit takes the families in a seeded random order (up to 20 tries), so a re-roll starts from
    another packing -- the deterministic packing of medium/hard wedged two rear bowls in every goal gate."""
    ids = [(o["object_id"], o["kind"]) for o in instance["objects"]]
    keeps = keep_candidates(instance, points) if use_keeps else {}
    rng = np.random.default_rng(seed)
    start = None
    for _ in range(20 if shuffled_start else 1):
        start = first_fit(ids, fam, masks, parts, points, keeps, list(keeps), rng=rng if shuffled_start else None, bans=bans)
        if start is None and keeps:
            start = first_fit(ids, fam, masks, parts, points, rng=rng if shuffled_start else None, bans=bans)
        if start is not None:
            break
    source = "shuffled first-fit" if shuffled_start else "first-fit"
    if start is None and shuffled_start:                     # the plain packing, then the validated goals
        start, source = first_fit(ids, fam, masks, parts, points, bans=bans), "first-fit"
    for template in templates if start is None else ():     # an Isaac-validated goal of the same tier
        dishes, ok = DishSet(parts, points), True
        for e in template:
            if dishes.collides(e) or banned(e, template, bans):
                ok = False
                break
            dishes.add(e["id"], e)
        if ok and sorted(e["id"] for e in template) == sorted(i for i, _ in ids):
            start, source = [dict(e, keep=False) for e in template], "validated goal"
            break
    if start is None:
        return None
    started = time.monotonic()
    entries, res, history, sweeps, stop = ascend(start, fam, masks, parts, points, scorer.score, scorer.iso,
                                                 np.random.default_rng(seed), keeps, deadline, max_sweeps, log=log, bans=bans)
    return {"method": f"start: {source}; coordinate ascent (top 6 + 2 random, full score)", "seed": seed,
            "sweeps": sweeps, "stop": stop, "seconds": time.monotonic() - started, "score_calls": scorer.calls,
            "S_planned": res["score"], "W_planned": res["worst"], "feasible": res["feasible"],
            "keep": sorted(e["id"] for e in entries if e.get("keep")), "keep_candidates": sorted(keeps),
            "entries": entries, "history": history}


# --------------------------------------------------------------------------- episode world and predicates

RACK_BOX = {"LowerRack": ((-.28, .28), (-.30, .30), (-.03, .32)),      # rack-local containment boxes [m]
            "UpperRack": ((-.26, .26), (-.29, .29), (-.04, .27))}       # (frigidaire_hotec_load.py RACK_BOX)


def bench_checker(asset_dir=None):
    """InitialCollisionChecker (1 mm allowance) with the HOTEC kinds under their kind names."""
    from dishsim_frigidaire.initial_state_candidates import InitialCollisionChecker
    from dishsim_frigidaire.paths import ASSET_DIR
    return InitialCollisionChecker(asset_dir or ASSET_DIR, tableware=tableware())


def _planner():
    from dishsim_frigidaire import planner as P
    return P


def make_bench_world(instance, checker=None):
    """BenchWorld (a PlannerWorld on the 1.8 m counter with plate-sized cells and a support rule)."""
    P = _planner()

    class BenchWorld(P.PlannerWorld):
        def __init__(self, instance, checker=None):
            self.counter = instance["counter"]
            self._cache = {}
            super().__init__(instance, checker=checker or bench_checker())
            self.goal_keys, self.goal_of, self.centroids, self.commanded_goal = set(), {}, {}, {}

        def _slab_body(self):
            return P.slab_body(self.checker, self.counter)

        def body(self, kind, T, key):
            k = (kind, P._key(T))
            if k not in self._cache:
                self._cache[k] = self.checker._body(kind, P.pose_dict(T), key)
            return dict(self._cache[k], id=key)

        def buffer_poses(self, kind):
            if kind not in self._buffer:
                lower = self.checker.bounds[kind][0]
                z = self.counter["top_z_m"] + CELL_HOVER_M - float(lower[2])
                cx, cy, _ = self.counter["center_m"]
                nx = int(self.counter["size_m"][0] // CELL_PITCH_M)
                xs = (np.arange(nx) - (nx - 1) / 2) * CELL_PITCH_M
                ys = (np.arange(2) - .5) * CELL_PITCH_M
                self._buffer[kind] = [P.make_T((cx + x, cy + y, z), (0., 0., 0., 1.)) for y in ys for x in xs]
            return self._buffer[kind]

        def mark_goals(self, targets, centroids=None):
            """Goal poses were certified together (jointly settled on track A, mutually FCL-free as planned on
            track B) and may touch: a command to a goal pose skips the pair check against a dish that rests at
            ITS goal -- its last executed command was that goal (it may have settled up to ~4 cm off a lifted
            family pose), or it lies within the at-goal tolerance. The per-move Isaac settle stays the arbiter
            (a real intrusion ends as `disturbed`)."""
            for oid, T in targets.items():
                self.certify(T)
                self.goal_keys.add(P._key(T))
                self.goal_of[oid] = np.asarray(T, dtype=float)
            self.centroids.update(centroids or {})

        def note_command(self, item_id, T):
            """Called by the oracle after an executed (not put-back) move."""
            self.commanded_goal[item_id] = item_id in self.goal_of and P._key(T) == P._key(self.goal_of[item_id])

        def at_own_goal(self, k):
            if k not in self.goal_of:
                return False
            if self.commanded_goal.get(k):
                return True
            kind = self.classes[k]
            return within_goal(kind, goal_distance(self.centroids.get(kind, np.zeros(3)), self._poses[k], self.goal_of[k]))

        def _collisions(self, item_id, T):
            body = self.body(self.classes[item_id], T, item_id)
            self.n_queries += 1
            key, hits = P._key(T), []
            if key not in self.certified and not self.checker.against_components(body)["valid"]:
                hits.append("appliance")
            if not self.checker.pair(body, self.slab)["valid"]:
                hits.append("Counter")
            for k, other in self._bodies.items():
                if k == item_id or (key in self.goal_keys and self.at_own_goal(k)):
                    continue
                if not self.checker.pair(body, other)["valid"]:
                    hits.append(k)
            return hits

        def resting_on(self, item_id):
            """The support rule is for the counter pile: a dish resting on ``item_id`` there must go first. In the
            racks an upright plate touching a lying bowl has the higher centre, which reads as "support" (medium_s0:
            every track aborted on it); rack neighbours are judged by the settle's disturbance check instead."""
            return [b for b in super().resting_on(item_id) if b in self._poses and self.in_counter(self._poses[b])]

        def harness_collides(self, item_id, T, object_class=None):
            """FCL only: the driver's refusal check (moving a supporting dish is left to the oracle)."""
            return bool(self._collisions(item_id, T))

        def goal_waits(self, item_id, T):
            """Goal-order constraints (instance goal "order", a before b): a command to b's goal waits until every
            dish b leans on in the certified build is at its own goal."""
            if item_id not in self.goal_of or P._key(T) != P._key(self.goal_of[item_id]):
                return []
            return [a for a, b in getattr(self, "goal_order", ()) if b == item_id and not self.at_own_goal(a)]

        def move_collides(self, item_id, T, object_class=None):
            return bool(self.resting_on(item_id)) or bool(self.goal_waits(item_id, T)) or bool(self._collisions(item_id, T))

        def blockers(self, item_id, T, object_class=None):
            return sorted(set(self.resting_on(item_id)) | set(self.goal_waits(item_id, T)) | set(self._collisions(item_id, T)))

    return BenchWorld(instance, checker)


def kind_centroids(points):
    return {kind: np.asarray(p).mean(axis=0) for kind, p in points.items()}


def goal_distance(centroid, T_actual, T_goal):
    """(lateral m, |dz| m, tilt deg) between two poses of the same dish, via its visual centroid and +Z."""
    Ta, Tg = np.asarray(T_actual, dtype=float), np.asarray(T_goal, dtype=float)
    ca, cg = Ta[:3, :3] @ centroid + Ta[:3, 3], Tg[:3, :3] @ centroid + Tg[:3, 3]
    d = ca - cg
    cos = float(np.clip(Ta[:3, 2] @ Tg[:3, 2], -1., 1.))
    return float(np.hypot(d[0], d[1])), float(abs(d[2])), float(np.degrees(np.arccos(cos)))


def within_goal(kind, distance):
    lat, dz, tilt = AT_GOAL[kind]
    return distance[0] <= lat and distance[1] <= dz and distance[2] <= tilt


def racked_in(points, pose_world, frames):
    """The rack whose containment box holds the dish's visual centroid (10 mm tolerance), else None."""
    from dishsim_frigidaire.random_poses import relative_pose
    c = posed_points(points, pose_world["position_m"], pose_world["quaternion_xyzw"]).mean(axis=0)
    for rack, box in RACK_BOX.items():
        f = frames[rack]
        rel = relative_pose(c, [0., 0., 0., 1.], f["position_m"], f["quaternion_xyzw"])[0]
        if all(lo - .01 <= v <= hi + .01 for v, (lo, hi) in zip(rel, box)):
            return rack
    return None


def goal_targets(instance, world, hover=GOAL_HOVER_M):
    """{id: world 4x4} of the settled goal (+ hover) in the instance's racks-out frames."""
    P = _planner()
    out = {}
    for oid, g in instance["goal"]["objects"].items():
        T = P.goal_T(world, g["rack"], g["settled_rack_local_pose"])
        T[2, 3] += hover
        out[oid] = T
    return out


class FixedGoalSequencer:
    """Planner pair on the goal track: the planner's greedy-with-buffering sequencer on the fixed goal."""

    def __init__(self, seed=None):
        self.plan = []

    def reset(self, instance, world):
        P = _planner()
        targets = {it["item_id"]: np.asarray(it["target"]["T_base_obj"]) for it in instance.items}
        order = P.item_order(instance.planner_instance)
        self.plan = P.sequence(world, targets, order, instance.meta["counter_cap"]) or []

    def next_move(self, obs):
        return self.plan.pop(0) if self.plan else None


class Replay:
    """Replays a Kit-free plan file (the open-track planner pair: the scorer needs Warp, not Kit)."""

    def __init__(self, plan):
        self.moves, self.planned_s = list(plan["moves"]), plan.get("planning_time_s")

    def reset(self, instance, world):
        final = {}
        for m in self.moves:
            T = np.asarray(m["T_base_obj"], dtype=float)
            world.certify(T)
            if not world.in_counter(T):
                final[m["item_id"]] = T
        world.mark_goals(final)                                 # the plan's own load is this episode's goal

    def next_move(self, obs):
        if not self.moves:
            return None
        from dishsim.rearrange import Move
        m = self.moves.pop(0)
        return Move(m["item_id"], np.asarray(m["T_base_obj"], dtype=float))


def plan_open(instance, algorithm, scorer, fam, masks, parts, points, budget_s=60., seed=0, log=print):
    """Open-track plan (Kit-free): choose a complete load, then sequence it on the counter cap.

    planner: keeps + first-fit + coordinate ascent within the budget (15 s kept for sequencing);
    baseline: first-fit in family order, no keeps, no ranking. Returns a plan dict (moves as world 4x4)."""
    P = _planner()
    t0, c0 = time.monotonic(), time.process_time()
    world = make_bench_world(instance)
    rinst = P.to_rearrange_instance(instance)
    world.sync({it["item_id"]: it["T_base_init"] for it in rinst.items}, {it["item_id"]: it["object_class"] for it in rinst.items})
    ids = [(o["object_id"], o["kind"]) for o in instance["objects"]]
    tries = []
    if algorithm == "planner":
        g = goal_search(instance, scorer, fam, masks, parts, points, seed=seed, deadline=t0 + budget_s - 15., log=log, bans=load_bans())
        if g is not None:
            tries.append((g["entries"], g["S_planned"]))
    else:
        e = first_fit(ids, fam, masks, parts, points, bans=load_bans())
        if e is not None:
            tries.append((e, None))
    moves, chosen = None, None
    for entries, S in tries:
        targets = {}
        for e in entries:
            T = P.goal_T(world, e["rack"], {"position_m": e["position"], "quaternion_xyzw": e["quaternion_xyzw"]})
            world.certify(T)
            targets[e["id"]] = T
        moves = P.sequence(world, targets, P.item_order(instance), instance["counter"]["cap"])
        if moves is not None:
            chosen = (entries, S)
            break
    return {"instance": instance["instance_id"], "track": "open", "algorithm": algorithm, "seed": seed,
            "budget_s": budget_s, "planning_time_s": time.monotonic() - t0, "planning_cpu_s": time.process_time() - c0,
            "moves": [{"item_id": m.item_id, "T_base_obj": np.asarray(m.T_base_obj).tolist()} for m in (moves or [])],
            "goal": None if chosen is None else chosen[0], "S_planned": None if chosen is None else chosen[1],
            "sequenced": moves is not None}


# --------------------------------------------------------------------------- capacity check (gate G2)

def capacity(log=print):
    fam, parts, points = families()
    masks = family_masks(fam, parts, points)
    counts = {kind: (len(fam[kind]), int(masks[kind].sum())) for kind in KINDS}
    log(f"[INFO] families after filters (all, appliance-free): {counts}")
    ok = True
    for tier in TIERS:
        entries = first_fit(roster(tier), fam, masks, parts, points)
        placed = 0 if entries is None else len(entries)
        need = len(roster(tier))
        log(f"[INFO] {tier}: first-fit racks {placed if entries else 'FAILED'} of {need}")
        if entries is None:
            ok = False
    print(f"[RESULT] {'PASS' if ok else 'FAIL'} capacity (first-fit racks every tier with the benchmark families)", flush=True)
    return ok


# --------------------------------------------------------------------------- pipeline stages (Kit-free)

def stage_goal(attempt_dir, device=None):
    """Goal search for a start record; writes goal.json and the goal-gate manifest."""
    attempt_dir = Path(attempt_dir)
    start = json.loads((attempt_dir / "start.json").read_text())
    if (attempt_dir / "goal.json").exists():
        raise SystemExit(f"[RESULT] FAIL refusing to overwrite {attempt_dir / 'goal.json'}")
    fam, parts, points = families()
    masks = family_masks(fam, parts, points)
    scorer = Scorer(device)
    # per attempt: with no keeps the goal would otherwise repeat on every re-roll, and a goal load that fails the
    # gate (e.g. a rack jolt on retraction) would fail every attempt of the instance
    seed = int(hashlib.sha256(f"goal|{start['instance_id']}|a{start['attempt']}".encode()).hexdigest()[:8], 16)
    bans = load_bans()
    g = goal_search(start, scorer, fam, masks, parts, points, seed=seed, shuffled_start=start["attempt"] > 0, bans=bans,
                    templates=validated_goals(start["tier"]))
    if g is not None:
        g["bans_applied"] = len(bans)
    if g is None:
        print("[RESULT] REROLL no complete load (keeps + first-fit failed)", flush=True)
        return 1
    (attempt_dir / "goal.json").write_text(json.dumps(g, indent=1) + "\n")
    manifest = {"objects": [{"object_id": e["id"], "kind": e["kind"], "rack": e["rack"],
                             "rack_local_pose": {"position_m": e["position"], "quaternion_xyzw": e["quaternion_xyzw"]}}
                            for e in g["entries"]],
                "baseline": start["baseline"], "tableware": tableware()}
    (attempt_dir / "gate_manifest.json").write_text(json.dumps(manifest, indent=1) + "\n")
    print(f"[RESULT] PASS goal S {g['S_planned']:.4f}, {len(g['keep'])} kept, {g['sweeps']} sweeps ({g['stop']}), "
          f"{g['seconds']:.0f} s", flush=True)
    return 0


NEIGHBOUR_ORDER_M = .15   # goal dishes this close (same rack) keep the certified build's relative order


def neighbour_order(build_order, poses_world, racks):
    """(a before b) for every same-rack pair of goal dishes within NEIGHBOUR_ORDER_M, in the certified build's
    order: a build that succeeds first time learns nothing, yet neighbours still depend on it (medium_s0: the
    build placed bowl_03 before bowl_01; greedy did the reverse and bowl_03 shoved bowl_01 43.8 mm)."""
    pos = {oid: n for n, oid in enumerate(build_order)}
    xyz = {oid: np.asarray(poses_world[oid]["position_m"], dtype=float) for oid in build_order}
    return sorted((a, b) for a in build_order for b in build_order
                  if pos[a] < pos[b] and racks[a] == racks[b] and float(np.linalg.norm(xyz[a] - xyz[b])) < NEIGHBOUR_ORDER_M)


def goal_order_of(seq, goal_entries):
    racks = {e["id"]: e["rack"] for e in goal_entries}
    return sorted({tuple(e) for e in seq["goal_order"]} | set(neighbour_order(seq["order"], seq["poses_world"], racks)))


def stage_reorder(instance_path):
    """Schema upgrade of an accepted instance: goal order = contact/learned edges + the neighbour order."""
    path = Path(instance_path)
    inst = json.loads(path.read_text())
    seq = {"order": inst["goal"]["build_order"], "goal_order": inst["goal"]["order"],
           "poses_world": {k: g["settled_pose_world"] for k, g in inst["goal"]["objects"].items()}}
    entries = [{"id": k, "rack": g["rack"]} for k, g in inst["goal"]["objects"].items()]
    before = len(inst["goal"]["order"])
    inst["goal"]["order"] = [list(e) for e in goal_order_of(seq, entries)]
    inst["goal"]["order_version"] = 2
    path.write_text(json.dumps(inst, indent=1) + "\n")
    print(f"[RESULT] PASS reorder {path.name}: {before} -> {len(inst['goal']['order'])} order constraints", flush=True)
    return 0


def stage_finalize(attempt_dir, out=OUT):
    """Accepted gate -> the benchmark instance: settled goal, S_ref, lower bound, sequencing certificate."""
    P = _planner()
    from dishsim_frigidaire.loading import visual_points
    attempt_dir = Path(attempt_dir)
    start = json.loads((attempt_dir / "start.json").read_text())
    goal = json.loads((attempt_dir / "goal.json").read_text())
    gate = json.loads((attempt_dir / "gate" / "result.json").read_text())
    if gate.get("outcome") != "accepted":
        print(f"[RESULT] REROLL goal gate {gate.get('outcome')}: {gate.get('reason')}", flush=True)
        return 1
    points = {kind: visual_points(ASSETS / f"{kind}.usda") for kind in KINDS}
    settled = gate["initial_snapshot"]["poses"]
    frames = {r: settled[r] for r in ("LowerRack", "UpperRack")}
    # Track A's goal = the poses of the Isaac SEQUENTIAL build (frigidaire_bench_kit.py --sequence: one dish at a
    # time, lowest first), and its support edges = the goal-order constraints (a before b) the planners see.
    seq = json.loads((attempt_dir / "sequence.json").read_text())
    if "poses_world" not in seq:
        print("[RESULT] REROLL the goal did not build one dish at a time", flush=True)
        return 1
    built = seq["poses_world"]
    objects, entries = {}, []
    for e in goal["entries"]:
        world = built[e["id"]]
        local = P.local_from_world(frames[e["rack"]], world)
        if not tub_clear(points[e["kind"]], local["position_m"], local["quaternion_xyzw"]):
            print(f"[RESULT] REROLL settled goal {e['id']} crosses the tub wall", flush=True)
            return 1
        objects[e["id"]] = {"rack": e["rack"], "slot": e["slot"], "variant": e["variant"], "keep": e.get("keep", False),
                            "command_rack_local_pose": {"position_m": e["position"], "quaternion_xyzw": e["quaternion_xyzw"]},
                            "settled_pose_world": world, "settled_rack_local_pose": local}
        entries.append({**e, "position": local["position_m"], "quaternion_xyzw": local["quaternion_xyzw"]})
    scorer = Scorer()
    ref = scorer.score(entries)
    n_counter = sum(o["start"] == "Counter" for o in start["objects"])
    lower = n_counter + sum(1 for o in start["objects"] if o["start"] != "Counter" and not objects[o["object_id"]]["keep"])
    inst = {**start, "purpose": "hotec_bench_instance",
            "goal": {**{k: goal[k] for k in ("method", "seed", "sweeps", "stop", "seconds", "S_planned", "keep")},
                     "S_ref": ref["score"], "W_ref": ref["worst"], "feasible_ref": ref["feasible"],
                     "per_object_ref": [{k: o[k] for k in ("id", "kind", "rack", "exposure", "pools")} for o in ref["objects"]],
                     "lower_bound": lower, "objects": objects, "order": [list(e) for e in goal_order_of(seq, goal["entries"])],
                     "order_version": 2, "build_order": seq["order"]},
            "validation": {**start["validation"], "gate": {"outcome": gate["outcome"], "reason": gate.get("reason"),
                                                           "wall_seconds": gate.get("wall_seconds"),
                                                           "maximum_cycle_penetration_m": gate.get("maximum_cycle_penetration_m")}}}
    world = make_bench_world(inst)
    rinst = P.to_rearrange_instance(inst)
    world.sync({it["item_id"]: it["T_base_init"] for it in rinst.items}, {it["item_id"]: it["object_class"] for it in rinst.items})
    targets = goal_targets(inst, world)
    world.mark_goals(targets, kind_centroids(points))
    world.goal_order = [tuple(e) for e in inst["goal"]["order"]]          # the certificate honours the build order
    plan = P.sequence(world, targets, P.item_order(inst), inst["counter"]["cap"])
    inst["certificate"] = {"sequenceable": plan is not None, "moves": None if plan is None else len(plan),
                           "method": "planner.sequence on the FCL mirror (greedy with buffering), not gated"}
    path = Path(out) / "instances" / inst["tier"] / f"{inst['instance_id']}.json"
    if path.exists():
        raise SystemExit(f"[RESULT] FAIL refusing to overwrite {path}")
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(inst, indent=1) + "\n")
    print(f"[RESULT] PASS instance {path.name}: S_ref {ref['score']:.4f}, lower bound {lower}, "
          f"certificate {inst['certificate']['moves']}", flush=True)
    return 0


def stage_plan(instance_path, out=OUT, device=None):
    """Open-track plans for the planner and the first-fit baseline (Kit-free, one pinned job per instance)."""
    inst = json.loads(Path(instance_path).read_text())
    fam, parts, points = families()
    masks = family_masks(fam, parts, points)
    scorer = Scorer(device)
    for name in ("planner", "baseline"):
        path = Path(out) / "plans" / "open" / inst["tier"] / f"{inst['instance_id']}__{name}.json"
        if path.exists():
            continue
        seed = int(hashlib.sha256(f"0|{inst['instance_id']}|{name}".encode()).hexdigest()[:8], 16)
        plan = plan_open(inst, name, scorer, fam, masks, parts, points, seed=seed, log=lambda *a: None)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(plan, indent=1) + "\n")
        print(f"[INFO] {path.name}: {len(plan['moves'])} moves, sequenced {plan['sequenced']}, "
              f"S_planned {plan['S_planned']}, {plan['planning_time_s']:.1f} s wall / {plan['planning_cpu_s']:.1f} s CPU", flush=True)
    print("[RESULT] PASS plans", flush=True)
    return 0


def racked_entries(poses, frames, kinds, points):
    """Scorer entries (rack-local) of the dishes racked in a set of world poses; the rest are returned apart."""
    P = _planner()
    entries, loose = [], []
    for oid, pose in poses.items():
        rack = racked_in(points[kinds[oid]], pose, frames)
        if rack is None:
            loose.append(oid)
            continue
        local = P.local_from_world(frames[rack], pose)
        entries.append({"id": oid, "kind": kinds[oid], "rack": rack, "position": local["position_m"],
                        "quaternion_xyzw": local["quaternion_xyzw"]})
    return entries, loose


def stage_analyze(episode_path, out=OUT, device=None, trace_samples=120, trace_directions=32, media=MEDIA):
    """Score detail of one episode: final S (full resolution, samples kept), S after every move (low
    resolution), and the figure: per-rack exposure heat maps, per-dish bars, S and racked count over moves."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import matplotlib.ticker
    from dishsim_frigidaire.loading import visual_points
    episode_path = Path(episode_path)
    rec = json.loads(episode_path.read_text())
    inst = json.loads((Path(out) / "instances" / rec["tier"] / f"{rec['instance']}.json").read_text())
    kinds = {o["object_id"]: o["kind"] for o in inst["objects"]}
    points = {kind: visual_points(ASSETS / f"{kind}.usda") for kind in KINDS}
    frames = rec.get("final_components") or {r: inst["initial_snapshot"]["poses"][r] for r in ("LowerRack", "UpperRack")}
    scorer = Scorer(device)
    E, hx = scorer.E, scorer.hx
    entries, loose = racked_entries(rec.get("final_poses", {}), frames, kinds, points)
    basket = (np.asarray(hx.BODY_POSITIONS["SilverwareBasket"], dtype=float), np.asarray(E.IDENTITY, dtype=float))
    final = E.score_arrangement(E.Arrangement("final", "", "", hx.world_objects(entries), basket), device=scorer.device,
                                samples=E.DEFAULTS["samples_per_object"], directions=E.DEFAULTS["directions"], baselines=False) if entries else None
    def low(es):
        return float(hx.full_score(es, "trace", scorer.device, trace_samples, trace_directions)["score"]) if es else 0.
    start_entries, _ = racked_entries({oid: inst["initial_snapshot"]["poses"][oid] for oid in kinds}, frames, kinds, points)
    trace = [{"move": 0, "racked": len(start_entries), "S_racked": low(start_entries)}]
    k = 0
    for m in rec.get("moves", []):
        after = m.get("poses_after")
        if not after:
            continue
        k += 1
        es, _ = racked_entries(after, frames, kinds, points)
        trace.append({"move": k, "item": m.get("item_id"), "racked": len(es), "S_racked": low(es)})
    analysis = {"episode": str(episode_path.relative_to(Path(out))) if str(episode_path).startswith(str(Path(out))) else str(episode_path),
                "S_final": None if final is None else final["score"], "W_final": None if final is None else final["worst"],
                "feasible_final": None if final is None else final["feasible"], "racked_final": len(entries), "loose_final": loose,
                "n_objects": len(kinds), "S_ref": inst["goal"]["S_ref"], "trace": trace,
                "per_object": [] if final is None else [{k2: o[k2] for k2 in ("id", "kind", "rack", "exposure", "pools")} for o in final["objects"]],
                "trace_resolution": {"samples": trace_samples, "directions": trace_directions}}
    apath = episode_path.with_suffix(".analysis.json")
    apath.write_text(json.dumps(analysis, indent=1) + "\n")
    # ---- figure
    fig = plt.figure(figsize=(17, 10), facecolor="white")
    gs = fig.add_gridspec(2, 3, width_ratios=(1, 1, 1.1), height_ratios=(1, .8), hspace=.32, wspace=.25)
    if final is not None:
        s = final["samples"]; owner = s["owner"]; racks = [o["rack"] for o in final["objects"]]
        for col, rack in enumerate(("LowerRack", "UpperRack")):
            ax = fig.add_subplot(gs[0, col])
            mask = np.isin(owner, [i for i, r in enumerate(racks) if r == rack])
            pts, ex = s["points"][mask], s["exposure"][mask]
            order = np.argsort(ex)
            sc = ax.scatter(pts[order, 0] * 1000, pts[order, 1] * 1000, c=ex[order], s=2, cmap="viridis", vmin=0, vmax=1, rasterized=True)
            for wire in E.rack_wires(rack):
                w = wire + np.asarray(hx.BODY_POSITIONS[rack])
                ax.plot(w[:, 0] * 1000, w[:, 1] * 1000, color="#b8c0c8", lw=.4, zorder=0)
            ax.set_aspect("equal"); ax.set_xlim(-300, 300); ax.set_ylim(-300, 300)
            ax.set_title(f"finished load, {rack}: food-contact exposure", fontsize=10)
            ax.set_xlabel("x [mm]"); ax.set_ylabel("y [mm] (front at the bottom)" if col == 0 else "")
        fig.colorbar(sc, ax=fig.axes[:2], shrink=.8, label="share of spray-arm rays reaching the surface")
        ax = fig.add_subplot(gs[0, 2])
        objs = sorted(final["objects"], key=lambda o: o["id"])
        colours = ["#4c72b0" if o["kind"].endswith("plate") else "#dd8452" if o["kind"].endswith("bowl") else "#55a868" for o in objs]
        ax.barh([o["id"] + (" (pools)" if o["pools"] else "") for o in objs], [o["exposure"] for o in objs], color=colours)
        ax.set_xlim(0, 1); ax.invert_yaxis(); ax.tick_params(axis="y", labelsize=7)
        ax.set_xlabel("exposure of the dish (area-weighted mean)")
        ax.set_title(f"per dish: S {final['score']:.3f} (S_ref {inst['goal']['S_ref']:.3f}), worst {final['worst']:.3f}", fontsize=10)
    ax = fig.add_subplot(gs[1, :2])
    xs = [t["move"] for t in trace]
    ax.plot(xs, [t["S_racked"] for t in trace], "-o", ms=3, color="#1f77b4", label="S over the racked dishes (low-res re-score)")
    ax.axhline(inst["goal"]["S_ref"], color="#888", ls="--", lw=1, label="S_ref (sampled goal)")
    ax.xaxis.set_major_locator(matplotlib.ticker.MaxNLocator(integer=True))
    ax.set_xlabel("executed move"); ax.set_ylabel("S"); ax.set_ylim(0, max(.35, max([t["S_racked"] for t in trace] + [0]) + .05))
    ax2 = ax.twinx()
    ax2.step(xs, [t["racked"] for t in trace], where="post", color="#d62728", alpha=.6, label="dishes racked")
    ax2.set_ylabel("dishes racked", color="#d62728"); ax2.set_ylim(0, len(kinds) + 1)
    ax.legend(loc="lower right", fontsize=8); ax.set_title("score change over the episode", fontsize=10)
    ax = fig.add_subplot(gs[1, 2]); ax.axis("off")
    end = rec.get("end_check", {})
    text = [f"{rec['instance']}  ·  {rec['track']} track  ·  {rec['algorithm']}",
            f"success: {rec.get('success')}   (abort: {rec.get('abort')}, end check: {end.get('outcome')})",
            f"moves: {rec.get('moves_used')}   lower bound: {rec.get('lower_bound')}   gap: {rec.get('gap')}",
            f"planning: {rec.get('planning_time_total_s', 0):.1f} s wall" + (f", {rec['planning_cpu_s']:.1f} s CPU" if rec.get("planning_cpu_s") else ""),
            f"counter: {inst['counter']['start_count']} at start, allowance {inst['counter']['cap']}",
            f"failed settles: {rec.get('failed_settles')}   refusals: {rec.get('infeasible_commands')} FCL, {rec.get('counter_full_refusals')} counter-full",
            "", "S = sum(A_o E_o) / sum(A_o)", "E_o: share of spray-arm rays reaching the", "food-contact samples of dish o (revision 5);",
            "A_o: its food-contact area; pooling = infeasible."]
    ax.text(0, 1, "\n".join(text), va="top", fontsize=9, family="monospace")
    fig.suptitle(f"Score detail: {rec['instance']}, {rec['track']} track, {rec['algorithm']}", fontsize=12)
    fpath = Path(media) / "analysis" / rec["tier"] / f"{rec['instance']}__{rec['track']}__{rec['algorithm']}.png"
    fpath.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(fpath, dpi=110); plt.close(fig)
    print(f"[RESULT] PASS analysis {fpath.name}: S_final {analysis['S_final']}, {len(trace) - 1} moves traced", flush=True)
    return 0


# --------------------------------------------------------------------------- scheduler (host side)

KIT_CORES = ("0-7", "8-15", "16-23")
PY_CORES = ("24-27", "28-31")
CONTAINERS = (("dishsim-isaac", 1),)      # (container, host GPU): one container on GPU 1 (user, 2026-09-23: no second
                                          # container -- its pip layer would land on the root disk); GPU 0/2 are never used
KIT_STAGGER_S = 90.                       # a Kit job reaches its GPU footprint in about a minute; the next launch waits
LOGS = ROOT / "logs/benchmark_frigidaire_hotec"


class Scheduler:
    """At most 3 Kit and 2 Kit-free jobs, each niced and pinned to its own cores; Kit launches are staggered and
    each waits for free GPU memory, so a check never races a job that is still growing."""

    def __init__(self, kit_jobs=3, py_jobs=2, min_free_mb=6000):
        import queue
        import subprocess
        self.subprocess = subprocess
        import threading
        running = [c for c in CONTAINERS if self._up(c[0])]
        if not running:
            raise SystemExit("no dishsim container is running")
        self.kit, self.py = queue.Queue(), queue.Queue()
        for i in range(min(kit_jobs, 3)):
            self.kit.put((running[i % len(running)], KIT_CORES[i]))
        self.launch_lock, self.last_kit_launch = threading.Lock(), 0.
        for i in range(min(py_jobs, 2)):
            self.py.put((running[0], PY_CORES[i]))
        self.min_free_mb = min_free_mb
        self.containers = running
        LOGS.mkdir(parents=True, exist_ok=True)

    def container_sh(self, argv):
        """A file operation on container-written (root-owned) outputs, run inside the container."""
        self.subprocess.run(["docker", "exec", self.containers[0][0], *argv], check=True)

    def _up(self, name):
        r = self.subprocess.run(["docker", "ps", "--filter", f"name=^{name}$", "--format", "{{.Names}}"],
                                capture_output=True, text=True)
        return r.stdout.strip() == name

    def _free_mb(self, gpu):
        r = self.subprocess.run(["nvidia-smi", "-i", str(gpu), "--query-gpu=memory.free", "--format=csv,noheader,nounits"],
                                capture_output=True, text=True)
        return int(r.stdout.strip() or 0)

    def run(self, kind, argv, log_name):
        """Run one stage in a free slot; returns the [RESULT] line ('' if none)."""
        q = self.kit if kind == "kit" else self.py
        (container, gpu), cores = q.get()
        try:
            if kind == "kit":
                with self.launch_lock:
                    time.sleep(max(0., self.last_kit_launch + KIT_STAGGER_S - time.monotonic()))
                    while self._free_mb(gpu) < self.min_free_mb:
                        time.sleep(30)
                    self.last_kit_launch = time.monotonic()
            launcher = "/workspace/dishsim/scripts/run_kit.sh" if kind == "kit" else "/workspace/dishsim/scripts/run_py.sh"
            lo, hi = (int(v) for v in cores.split("-"))
            threads = [f"{k}={hi - lo + 1}" for k in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS")]
            cmd = ["docker", "exec", "-w", "/workspace/dishsim", "-e", "PYTEST_DISABLE_PLUGIN_AUTOLOAD=1",
                   *[x for t in threads for x in ("-e", t)], container,
                   "taskset", "-c", cores, "nice", "-n", "10", launcher, *argv]     # BLAS pools sized to the pinned cores
            log = LOGS / f"{log_name}.log"
            with open(log, "w") as fh:
                fh.write(f"# {datetime_now()} {container} gpu{gpu} cores {cores}\n# {' '.join(cmd)}\n")
                fh.flush()
                self.subprocess.run(cmd, stdout=fh, stderr=self.subprocess.STDOUT)
            lines = [l for l in log.read_text(errors="replace").splitlines() if l.startswith("[RESULT]")]
            return lines[-1] if lines else ""
        finally:
            q.put(((container, gpu), cores))


def datetime_now():
    from datetime import datetime, timezone
    return datetime.now(timezone.utc).isoformat(timespec="seconds")


DATA_MOUNT = "/media/corallab-s1/2tbhdd/brianshu"   # mounted at the same path in the container (docker/compose.yaml)


def rel(path):
    """A host path as the container sees it: repo-relative (the data symlinks resolve identically inside), or an
    absolute path on the shared 2 TB mount; anything else would land in the container's writable layer."""
    import os
    path = os.path.abspath(str(path))                        # no symlink resolution: results/ etc. stay repo-relative
    if path == str(ROOT) or path.startswith(str(ROOT) + os.sep):
        return os.path.relpath(path, str(ROOT))
    if path.startswith(DATA_MOUNT + os.sep):
        return path
    raise ValueError(f"path not visible inside the container: {path}")


def generate_unit(sched, tier, seed, max_attempts=6, out=OUT, log=print):
    """One (tier, seed): attempts until an instance is accepted (start -> goal -> gate -> finalize)."""
    final = Path(out) / "instances" / tier / f"{tier}_s{seed}.json"
    if final.exists():
        return {"tier": tier, "seed": seed, "result": "exists"}
    trail = []
    for attempt in range(max_attempts):
        tag = f"{tier}_s{seed}_a{attempt}"
        adir = Path(out) / "instances" / "attempts" / tag
        steps = [("kit", ["frigidaire/scripts/experiment/frigidaire_bench_kit.py", "--start", "--tier", tier, "--seed", str(seed),
                          "--attempt", str(attempt), "--headless"], "start"),
                 ("py", ["frigidaire/scripts/experiment/frigidaire_bench.py", "--goal", rel(adir)], "goal"),
                 ("kit", ["frigidaire/scripts/experiment/frigidaire_initial_state_validate.py", "--manifest", rel(adir / "gate_manifest.json"),
                          "--out-dir", rel(adir / "gate"), "--max-wall-seconds", "600", "--headless", "--device", "cpu"], "gate"),
                 ("kit", ["frigidaire/scripts/experiment/frigidaire_bench_kit.py", "--sequence", rel(adir), "--headless"], "sequence"),
                 ("py", ["frigidaire/scripts/experiment/frigidaire_bench.py", "--finalize", rel(adir)], "finalize")]
        verdicts = []
        for kind, argv, name in steps:
            line = sched.run(kind, argv, f"{tag}_{name}")
            if name == "gate":
                gate = adir / "gate" / "result.json"
                outcome = json.loads(gate.read_text()).get("outcome") if gate.exists() else None
                ok = outcome == "accepted"
                if not ok and outcome == "closure_failure":         # the rack-speed flake: the other order once
                    argv2 = argv[:-3] + ["--order", "lower_first", "--headless", "--device", "cpu"]   # drops --headless --device cpu
                    argv2[argv2.index("--out-dir") + 1] = rel(adir / "gate_lower_first")
                    sched.run(kind, argv2, f"{tag}_gate_lower_first")
                    alt = adir / "gate_lower_first" / "result.json"
                    if alt.exists() and json.loads(alt.read_text()).get("outcome") == "accepted":
                        g = str((adir / "gate").resolve())                 # root-owned: moved inside the container
                        sched.container_sh(["mv", g, g + "_upper_first"])
                        sched.container_sh(["cp", "-a", str((adir / "gate_lower_first").resolve()), g])
                        ok = True
                if outcome == "penetration_failure":
                    sched.run("py", ["frigidaire/scripts/experiment/frigidaire_bench.py", "--ban", rel(adir)], f"{tag}_ban")
                line = f"[RESULT] {'PASS' if ok else 'REROLL'} gate {outcome}"
            verdicts.append((name, line))
            log(f"[INFO] {tag} {name}: {line}")
            if "PASS" not in line:
                break
        trail.append({"attempt": attempt, "verdicts": verdicts})
        if final.exists():
            return {"tier": tier, "seed": seed, "result": "accepted", "attempt": attempt, "trail": trail}
    return {"tier": tier, "seed": seed, "result": "failed", "trail": trail}


def orchestrate(units, fn, jobs):
    from concurrent.futures import ThreadPoolExecutor
    with ThreadPoolExecutor(max_workers=jobs) as pool:
        return list(pool.map(fn, units))


def ensure_baseline(sched, out=OUT):
    path = Path(out) / "baseline" / "result.json"
    if path.exists() and json.loads(path.read_text()).get("outcome") == "baseline_ready":
        return True
    line = sched.run("kit", ["frigidaire/scripts/experiment/frigidaire_initial_state_validate.py", "--out-dir", rel(Path(out) / "baseline"),
                             "--headless", "--device", "cpu"], "baseline")
    return path.exists() and json.loads(path.read_text()).get("outcome") == "baseline_ready"


# --------------------------------------------------------------------------- tables

ALGORITHMS = ("greedy_offline", "rrt_connect", "planner", "baseline")


def episode_rows(out=OUT):
    """Every table row's episode, per track: the 5 Isaac episodes of an instance fill 8 rows -- on track A the
    planner and the baseline are the same fixed-goal sequencer (one run), and on track B the Bosch pair's input
    is the sampled goal, so their track-A runs are scored again as open-track rows (success = all racked)."""
    rows = {"goal": [], "open": []}
    for path in sorted((Path(out) / "episodes").glob("*/*/*.json")):
        if path.parent.parent.name not in rows or path.name.endswith(".analysis.json"):
            continue
        rec = json.loads(path.read_text())
        analysis = path.with_suffix(".analysis.json")
        rec["S_final"] = json.loads(analysis.read_text())["S_final"] if analysis.is_file() else None
        rec["source_episode"] = f"{rec['track']}/{rec['tier']}/{path.stem}"
        rows[rec["track"]].append(rec)
        if rec["track"] == "goal" and rec["algorithm"] == "planner":
            rows["goal"].append({**rec, "algorithm": "baseline", "shared_from": "planner"})
        if rec["track"] == "goal" and rec["algorithm"] in ("greedy_offline", "rrt_connect"):
            rows["open"].append({**rec, "track": "open", "shared_from": "goal track",
                                 "success": rec["end_check"].get("outcome") == "accepted"})
    return rows


def summarize(recs):
    ok = [r for r in recs if r["success"]]
    gaps = [r["gap"] for r in ok if r.get("gap") is not None]
    cpu = [r["planning_cpu_s"] for r in recs if r.get("planning_cpu_s") is not None]
    ratios = [r["S_final"] / r["S_ref"] for r in ok if r.get("S_final") is not None and r.get("S_ref")]
    aborts = {}
    for r in recs:
        if r["success"]:                                     # an open-track replay "gives up" when its plan ends
            continue
        a = r.get("abort") or r.get("end_check", {}).get("outcome")
        if a:
            aborts[a] = aborts.get(a, 0) + 1
    mean = lambda v: float(np.mean(v)) if v else None
    return {"n": len(recs), "solved": len(ok), "moves": mean([r["moves_used"] for r in ok]), "gap": mean(gaps),
            "planning_wall_s": mean([r.get("planning_time_total_s", 0.) for r in recs]), "planning_cpu_s": mean(cpu),
            "S_final": mean([r["S_final"] for r in ok if r.get("S_final") is not None]), "S_over_S_ref": mean(ratios),
            "failed_settles": int(sum(r.get("failed_settles", 0) for r in recs)), "aborts": aborts}


def collect(out=OUT):
    """The two result tables (tier x algorithm per track) as summary.json and summary.md."""
    rows = episode_rows(out)
    table = {track: [{"tier": tier, "algorithm": algo, "shared_from": next((r.get("shared_from") for r in g if r.get("shared_from")), None),
                      **summarize(g)}
                     for tier in TIERS for algo in ALGORITHMS
                     for g in [[r for r in rows[track] if r["tier"] == tier and r["algorithm"] == algo]] if g]
             for track in ("goal", "open")}
    f = lambda v, d=1: "-" if v is None else f"{v:.{d}f}"
    lines = []
    for track, title in (("goal", "Track A: reach the sampled goal"), ("open", "Track B: open (every dish racked, scored by S)")):
        lines += [f"## {title}", "", "| tier | algorithm | success | moves | gap | planning s (wall / CPU) | S | S / S_ref | aborts |",
                  "|---|---|---|---|---|---|---|---|---|"]
        for t in table[track]:
            lines.append(f"| {t['tier']} | {t['algorithm']}{' (shared run)' if t['shared_from'] else ''} | {t['solved']}/{t['n']} | "
                         f"{f(t['moves'])} | {f(t['gap'])} | {f(t['planning_wall_s'])} / {f(t['planning_cpu_s'])} | "
                         f"{f(t['S_final'], 3)} | {f(t['S_over_S_ref'], 3)} | "
                         f"{', '.join(f'{k} {v}' for k, v in sorted(t['aborts'].items())) or '-'} |")
        lines.append("")
    path = Path(out) / "compare" / "summary.md"
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines) + "\n")
    (path.parent / "summary.json").write_text(json.dumps({"tables": table, "generated_utc": datetime_now()}, indent=1) + "\n")
    print("\n".join(lines))
    return path


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--capacity", action="store_true", help="gate G2: first-fit racks 8, 16 and 24 dishes")
    ap.add_argument("--goal", type=Path, metavar="ATTEMPT_DIR")
    ap.add_argument("--finalize", type=Path, metavar="ATTEMPT_DIR")
    ap.add_argument("--plan", type=Path, metavar="INSTANCE")
    ap.add_argument("--analyze", type=Path, metavar="EPISODE", help="score detail of one episode record (+ figure)")
    ap.add_argument("--ban", type=Path, metavar="ATTEMPT_DIR", help="ban the pair that penetrated in a failed goal gate")
    ap.add_argument("--reorder", type=Path, metavar="INSTANCE", help="upgrade an instance's goal order (neighbour order)")
    ap.add_argument("--collect", action="store_true")
    ap.add_argument("--generate", action="store_true", help="host: generate instances (parallel)")
    ap.add_argument("--plan-all", action="store_true", help="host: open-track plans for every instance (parallel)")
    ap.add_argument("--run-all", action="store_true", help="host: episodes for every instance (parallel)")
    ap.add_argument("--analyze-all", action="store_true", help="host: score detail of every episode record (parallel)")
    ap.add_argument("--videos", action="store_true", help="host: stop-motion videos of the first instance per tier (parallel)")
    ap.add_argument("--tiers", nargs="*", default=list(TIERS))
    ap.add_argument("--seeds", nargs="*", type=int, default=[0])
    ap.add_argument("--max-attempts", type=int, default=6)
    ap.add_argument("--kit-jobs", type=int, default=3)
    ap.add_argument("--py-jobs", type=int, default=2)
    ap.add_argument("--cameras", action="store_true", help="--run-all: render the initial/finished/goal stills")
    args = ap.parse_args(argv)
    if args.capacity:
        return 0 if capacity() else 1
    if args.goal:
        return stage_goal(args.goal)
    if args.finalize:
        return stage_finalize(args.finalize)
    if args.plan:
        return stage_plan(args.plan)
    if args.analyze:
        return stage_analyze(args.analyze)
    if args.ban:
        return stage_ban(args.ban)
    if args.reorder:
        return stage_reorder(args.reorder)
    if args.collect:
        collect()
        return 0
    if args.analyze_all:
        sched = Scheduler(0, args.py_jobs)
        paths = [p for p in sorted((OUT / "episodes").glob("*/*/*.json")) if p.parent.parent.name in ("goal", "open")
                 and not p.name.endswith(".analysis.json") and p.parent.name in args.tiers
                 and not p.with_suffix(".analysis.json").exists()]
        res = orchestrate(paths, lambda p: sched.run("py", ["frigidaire/scripts/experiment/frigidaire_bench.py", "--analyze", rel(p)],
                                                     f"analyze_{p.parent.parent.name}_{p.stem}"), jobs=args.py_jobs)
        print(f"[RESULT] {'PASS' if all('PASS' in r for r in res) else 'FAIL'} analyze ({len(paths)} episodes)", flush=True)
        return 0
    if args.videos:
        sched = Scheduler(args.kit_jobs, 0)
        eps = []
        for tier in args.tiers:
            insts = sorted((OUT / "instances" / tier).glob(f"{tier}_s*.json"), key=lambda p: int(p.stem.split("_s")[1]))
            if insts:
                eps += [(insts[0], e) for track in ("goal", "open")
                        for e in sorted((OUT / "episodes" / track / tier).glob(f"{insts[0].stem}__*.json")) if not e.name.endswith(".analysis.json")]
        res = orchestrate(eps, lambda ie: sched.run("kit", [
            "frigidaire/scripts/evaluation/frigidaire_planner_video.py", "--instance", rel(ie[0]), "--episode", rel(ie[1]),
            "--out-dir", rel(MEDIA / "video" / ie[0].parent.name), "--video-width", "640", "--headless", "--enable_cameras"],
            f"video_{ie[1].parent.parent.name}_{ie[1].stem}"), jobs=args.kit_jobs)
        print("\n".join(f"{e.parent.parent.name}/{e.stem}: {r}" for (_, e), r in zip(eps, res)), flush=True)
        print(f"[RESULT] {'PASS' if eps and all('PASS' in r for r in res) else 'FAIL'} videos ({len(eps)})", flush=True)
        return 0
    if args.generate or args.plan_all or args.run_all:
        sched = Scheduler(args.kit_jobs, args.py_jobs)
        if args.generate:
            if not ensure_baseline(sched):
                print("[RESULT] FAIL empty baseline", flush=True)
                return 1
            units = [(t, s) for s in args.seeds for t in args.tiers]
            res = orchestrate(units, lambda u: generate_unit(sched, u[0], u[1], args.max_attempts), jobs=args.kit_jobs + args.py_jobs)
            print(json.dumps([{k: r[k] for k in ("tier", "seed", "result")} for r in res]), flush=True)
            print(f"[RESULT] {'PASS' if all(r['result'] in ('accepted', 'exists') for r in res) else 'FAIL'} generate", flush=True)
            return 0
        paths = sorted(p for t in args.tiers for p in (OUT / "instances" / t).glob(f"{t}_s*.json")
                       if int(p.stem.split("_s")[1]) in args.seeds)
        if args.plan_all:
            res = orchestrate(paths, lambda p: sched.run("py", ["frigidaire/scripts/experiment/frigidaire_bench.py", "--plan", rel(p)],
                                                         f"plan_{p.stem}"), jobs=args.py_jobs)
        else:
            extra = ["--enable_cameras"] if args.cameras else []
            res = orchestrate(paths, lambda p: sched.run("kit", ["frigidaire/scripts/experiment/frigidaire_bench_kit.py", "--run",
                                                                 "--instance", rel(p), "--headless", *extra], f"run_{p.stem}"),
                              jobs=args.kit_jobs)
        print("\n".join(f"{p.stem}: {r}" for p, r in zip(paths, res)), flush=True)
        print(f"[RESULT] {'PASS' if all('PASS' in r for r in res) else 'FAIL'} {'plan' if args.plan_all else 'run'}", flush=True)
        return 0
    ap.print_help()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
