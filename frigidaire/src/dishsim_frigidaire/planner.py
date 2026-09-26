# Copyright (c) 2026, dishsim project.
# SPDX-License-Identifier: BSD-3-Clause
"""Arrangement planner for the Frigidaire FDPC4221AS over the revision-5 exposure objective.

Kit-free. Objects start in a messy pile on a countertop above the machine and unorganized
in the racks; a plan is a sequence of teleport moves (one object at a time) that ends with
every object inside, maximising the exposure score S with pooling as a hard gate
(lexicographic: feasibility, then S; moves reported). Runs on the benchmark driver
``dishsim.rearrange.run_episode``: this module supplies the world (an FCL mirror of the
appliance, the counter slab and every object), the oracle (geometric: a move lands as
commanded; pulling a supporting object from under another is the fatal "disturbed" fault),
the goal choosers (best-exposure proposal over the settled candidate pool plus keep-in-place,
or first-fit) and the sequencer (the benchmark's greedy rule with counter objects first, the
support gate and the counter cap mirrored so no refused command is ever emitted).

Frames: an instance stores racks-OUT world poses (the Isaac scene); FCL checks run there with
the instance's measured component frames. Scoring recomposes in-rack objects racks-IN from
rack-local poses, as ``exposure.load_state`` does. Counter objects are neither scored nor
occluders. Goal poses (pool candidates settled one by one in Isaac, keep-in-place poses
settled in this instance) are certified, so only their pairwise clearance is re-checked per
move; buffer cells are checked against everything.
"""
from __future__ import annotations
import hashlib
import json
import time
from collections import Counter, defaultdict
from pathlib import Path

import numpy as np

from dishsim.rearrange import Instance, Move
from dishsim.transforms import make_T, T_to_pos_quat
from . import exposure as E
from .paths import ASSET_DIR, REPO_ROOT
from .random_poses import compose_pose, quaternion_matrix_xyzw, relative_pose

COUNTER = {"size_m": (1.2, .6, .04), "center_m": (0., 0., .894), "top_z_m": .914}   # worktop over the cabinet (top .8509)
BUFFER_PITCH_M = .13          # bowl diameter .14: neighbouring cells cannot both hold a bowl, FCL decides
BUFFER_HOVER_M = .002
BAND_TOLERANCE_M = .01        # an object resting flat on the slab has its origin AT the top; the band starts 1 cm lower
SUPPORT_MIN_DZ_M = .005       # a dish-dish contact whose upper centre sits this much higher is a support edge
DISH_KINDS = ("bowl", "mug", "dinner_plate")
CUTLERY_KINDS = ("fork", "knife", "tablespoon", "teaspoon")
KINDS = DISH_KINDS + CUTLERY_KINDS
CUTLERY_PATTERNS = Path(__file__).with_name("cutlery_candidates.json")   # frozen head-down basket poses, static basket frame
CUTLERY_PER_SLOT = 5          # first variants per slot per kind: 45 per kind in the pool
CUTLERY_RELEASE_HOVER_M = .003   # inside-set starts release from this height, as the full-load evidence path does
POOL_RUN = REPO_ROOT / "results/initial_states/frigidaire/organized_20260911_seed20260911"
PACKING_RUN = REPO_ROOT / "results/initial_states/frigidaire/packing_20260911_seed20260911"
STATES = REPO_ROOT / "results/initial_states/frigidaire"
POOL_CACHE = REPO_ROOT / "results/planner/frigidaire/pool_geometric_v2.json"   # v1 (bowls+mugs) stays for the old records
ORGANIZED_REFERENCE = {(5, 4): "random_00", (9, 9): "random_06"}   # (bowls, mugs) -> organized state with that inventory
INVENTORY = {9: {"bowl": 5, "mug": 4}, 18: {"bowl": 9, "mug": 9},
             12: {"bowl": 3, "mug": 3, "dinner_plate": 2, "fork": 1, "knife": 1, "tablespoon": 1, "teaspoon": 1},
             24: {"bowl": 6, "mug": 6, "dinner_plate": 4, "fork": 2, "knife": 2, "tablespoon": 2, "teaspoon": 2}}
# The reference search (plain pool, no keep): deterministic, and exactly the planner's first stage, so
# S_reference is the best arrangement the search finds IGNORING how to get there; gap > 0 = cost of sequencing.
REFERENCE_SEARCH = {"seed": 0, "random_attempts": 50, "milp_rounds": 3, "seconds": 25.}


# ----------------------------------------------------------------------------- poses
def pose_dict(T):
    p, q = T_to_pos_quat(T)
    return {"position_m": p.tolist(), "quaternion_xyzw": q.tolist()}


def pose_T(pose):
    return make_T(pose["position_m"], pose["quaternion_xyzw"])


def world_from_local(frame, local):
    p, q = compose_pose(frame["position_m"], frame["quaternion_xyzw"], local["position_m"], local["quaternion_xyzw"])
    return {"position_m": p.tolist(), "quaternion_xyzw": q.tolist()}


def local_from_world(frame, world):
    p, q = relative_pose(world["position_m"], world["quaternion_xyzw"], frame["position_m"], frame["quaternion_xyzw"])
    return {"position_m": p.tolist(), "quaternion_xyzw": q.tolist()}


def racks_in_pose(rack, local, basket=None):
    """Racks-in world pose of a rack-local pose (the scorer's frame).

    Basket poses compose on the instance's SETTLED basket (``basket_in``), which sits ~7 mm below the
    nominal seat; passing none for the basket fails closed rather than mis-posing cutlery.
    """
    if rack == "SilverwareBasket":
        if basket is None:
            raise ValueError("SilverwareBasket poses need the instance's settled basket frame (basket_in)")
        frame = basket
    else:
        frame = (E.BODY_POSITIONS[rack], E.IDENTITY)
    p, q = compose_pose(frame[0], frame[1], local["position_m"], local["quaternion_xyzw"])
    return {"position_m": p.tolist(), "quaternion_xyzw": q.tolist()}


def basket_in(snapshot):
    lower, basket = snapshot["poses"]["LowerRack"], snapshot["poses"]["SilverwareBasket"]
    bp, bq = relative_pose(basket["position_m"], basket["quaternion_xyzw"], lower["position_m"], lower["quaternion_xyzw"])
    return compose_pose(E.BODY_POSITIONS["LowerRack"], E.IDENTITY, bp, bq)


def in_counter_band(T, counter=None):
    """Over the slab and above its top: the benchmark's counter predicate for this machine."""
    counter = counter or COUNTER
    x, y, z = np.asarray(T, dtype=float)[:3, 3]
    cx, cy, _ = counter["center_m"]
    hx, hy = counter["size_m"][0] / 2, counter["size_m"][1] / 2
    return bool(abs(x - cx) <= hx and abs(y - cy) <= hy and z > counter["top_z_m"] - BAND_TOLERANCE_M)


def _key(T):
    return tuple(np.round(np.asarray(T, dtype=float)[:3].ravel(), 7))


def score_objects(objects, basket, device=E.DEVICE):
    """Exposure of in-rack objects given as {id, kind, rack, rack_local_pose}; (S, W, feasible, pooling)."""
    if not objects:
        return {"score": 0., "worst": 0., "feasible": True, "pooling_count": 0, "n_objects": 0}
    posed = [{"id": o["id"], "kind": o["kind"], "rack": o["rack"], **racks_in_pose(o["rack"], o["rack_local_pose"], basket)}
             for o in objects]
    r = E.score_arrangement(E.Arrangement("planner", "", "", posed, basket), device=device, baselines=False)
    return {k: r[k] for k in ("score", "worst", "feasible", "pooling_count", "n_objects")}


# ----------------------------------------------------------------------------- support graph
def support_edges(contact_pairs, poses, ids):
    """Directed edges (supporter, supported) from settled dish-dish contacts: the higher centre is supported."""
    ids, edges = set(ids), set()
    for a, b in contact_pairs:
        if a not in ids or b not in ids or a == b:
            continue
        dz = float(poses[b]["position_m"][2]) - float(poses[a]["position_m"][2])
        if dz > SUPPORT_MIN_DZ_M:
            edges.add((a, b))
        elif -dz > SUPPORT_MIN_DZ_M:
            edges.add((b, a))
    return sorted(list(e) for e in edges)


# ----------------------------------------------------------------------------- world
def slab_body(checker, counter=None):
    """The counter slab as a body the checker's ``pair`` accepts."""
    fcl = checker.fcl
    counter = counter or COUNTER
    size, center = np.asarray(counter["size_m"], dtype=float), np.asarray(counter["center_m"], dtype=float)
    obj = fcl.CollisionObject(fcl.Box(*size), fcl.Transform(np.eye(3), center))
    manager = fcl.DynamicAABBTreeCollisionManager()
    manager.registerObjects([obj])
    manager.setup()
    return {"id": "Counter", "kind": "Counter", "objects": [obj], "manager": manager,
            "bounds": (center - size / 2, center + size / 2)}


class PlannerWorld:
    """FCL mirror with the benchmark's world duck-type (sync/move_collides/blockers/buffer_poses/in_counter)."""

    def __init__(self, instance, asset_dir=ASSET_DIR, checker=None):
        from .initial_state_candidates import COMPONENT_NAMES, InitialCollisionChecker
        from .organization import OrganizationGeometry
        self.checker = checker or InitialCollisionChecker(asset_dir)
        snap = instance["initial_snapshot"]["poses"]
        self.frames = {name: snap[name] for name in COMPONENT_NAMES}
        self.checker.update_components(self.frames)
        self.geometry = OrganizationGeometry(checker=self.checker)
        self.slab = self._slab_body()
        self.classes, self._poses, self._bodies, self._buffer = {}, {}, {}, {}
        self.initial = {o["object_id"]: pose_T(o["pose_world"]) for o in instance["objects"]}
        self.support = [tuple(e) for e in instance.get("support", [])]
        self.certified = set()
        self.n_queries = 0

    def _slab_body(self):
        return slab_body(self.checker)

    def body(self, kind, T, key):
        return self.checker._body(kind, pose_dict(T), key)

    def sync(self, poses, classes):
        self.classes.update(classes)
        for k, T in poses.items():
            T = np.asarray(T, dtype=float)
            if k not in self._poses or not np.allclose(self._poses[k], T, atol=1e-9):
                self._poses[k] = T.copy()
                self._bodies[k] = self.body(self.classes[k], T, k)

    def snapshot(self):
        return {k: v.copy() for k, v in self._poses.items()}

    def clear(self):
        self._poses.clear()
        self._bodies.clear()

    def certify(self, T):
        """Mark a pose as settled-certified: the appliance check is skipped for it (pairs are not)."""
        self.certified.add(_key(T))

    def _collisions(self, item_id, T):
        body = self.body(self.classes[item_id], T, item_id)
        self.n_queries += 1
        hits = []
        if _key(T) not in self.certified and not self.checker.against_components(body)["valid"]:
            hits.append("appliance")
        if not self.checker.pair(body, self.slab)["valid"]:
            hits.append("Counter")
        for k, other in self._bodies.items():
            if k != item_id and not self.checker.pair(body, other)["valid"]:
                hits.append(k)
        return hits

    def move_collides(self, item_id, T, object_class=None):
        return bool(self._collisions(item_id, T))

    def blockers(self, item_id, T, object_class=None):
        return sorted(self._collisions(item_id, T))

    def buffer_poses(self, kind):
        if kind not in self._buffer:
            lower = self.checker.bounds[kind][0]
            z = COUNTER["top_z_m"] + BUFFER_HOVER_M - float(lower[2])
            cx, cy, _ = COUNTER["center_m"]
            hx, hy = COUNTER["size_m"][0] / 2 - .05, COUNTER["size_m"][1] / 2 - .05
            xs, ys = np.arange(-hx, hx + 1e-9, BUFFER_PITCH_M), np.arange(-hy, hy + 1e-9, BUFFER_PITCH_M)
            self._buffer[kind] = [make_T((cx + x, cy + y, z), (0., 0., 0., 1.)) for y in ys for x in xs]
        return self._buffer[kind]

    def in_counter(self, T):
        return in_counter_band(T, getattr(self, "counter", None))

    def resting_on(self, item_id):
        """Objects still resting on ``item_id`` (support edge and the supported object never moved)."""
        return [b for a, b in self.support
                if a == item_id and b in self._poses and np.allclose(self._poses[b], self.initial[b], atol=1e-6)]


# ----------------------------------------------------------------------------- oracle
class GeometricOracle:
    """Moves land as commanded; the support gate is the only fault; every move is scored."""

    def __init__(self, instance, rinstance, world, device=E.DEVICE):
        self.instance, self.rinstance, self.world, self.device = instance, rinstance, world, device
        self.kinds = {o["object_id"]: o["kind"] for o in instance["objects"]}
        self.poses = {o["object_id"]: pose_T(o["pose_world"]) for o in instance["objects"]}
        self.location = {o["object_id"]: (None if o["start"] == "Counter" else (o["rack"], o["rack_local_pose"]))
                         for o in instance["objects"]}
        self.basket = basket_in(instance["initial_snapshot"])
        self.trace = []

    def at_goal(self, item, T):
        target = item["target"].get("T_base_obj")
        return target is not None and bool(np.allclose(np.asarray(T), np.asarray(target), atol=1e-6))

    def inside(self):
        return [{"id": i, "kind": self.kinds[i], "rack": loc[0], "rack_local_pose": loc[1]}
                for i, loc in self.location.items() if loc is not None]

    def execute(self, move):
        disturbed = self.world.resting_on(move.item_id)
        T = np.asarray(move.T_base_obj, dtype=float)
        self.poses[move.item_id] = T
        item = self.rinstance.item(move.item_id)
        target = item["target"]
        if target.get("T_base_obj") is not None and np.allclose(T, target["T_base_obj"], atol=1e-6):
            self.location[move.item_id] = (target["rack"], target["rack_local_pose"])
        elif in_counter_band(T):
            self.location[move.item_id] = None
        else:   # never commanded by this module's planners; keep the bookkeeping honest
            frame_rack = target.get("rack") or "LowerRack"
            self.location[move.item_id] = (frame_rack, local_from_world(self.world.frames[frame_rack], pose_dict(T)))
        s = score_objects(self.inside(), self.basket, self.device)
        info = {"disturbed": disturbed, "inside_count": s["n_objects"], "score_after": s["score"],
                "worst_after": s["worst"], "feasible_after": s["feasible"], "pooling_after": s["pooling_count"]}
        self.trace.append({"item_id": move.item_id, "kind": self.kinds[move.item_id], **info})
        return dict(self.poses), ("disturbed" if disturbed else None), info


# ----------------------------------------------------------------------------- pool
def _sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def candidate_fields(c, index):
    return {"kind": c["kind"], "rack": c["rack"], "slot_id": c["slot_id"], "position_xy_m": list(c["position_xy_m"]),
            "quaternion_xyzw": list(c["quaternion_xyzw"]), "row_metadata": dict(c["row_metadata"]),
            "candidate_id": c["candidate_id"], "rack_local_pose": c["rack_local_pose"], "source_index": int(index),
            "source": "organized"}


def plate_fields(c, index):
    """A packing-catalog plate as a pool candidate: slot = its source trial, so variants of one trial conflict."""
    local = c["rack_local_pose"]
    R = quaternion_matrix_xyzw(local["quaternion_xyzw"])
    return {"kind": c["kind"], "rack": c["rack"], "slot_id": c["source_trial_id"], "position_xy_m": list(local["position_m"][:2]),
            "quaternion_xyzw": list(local["quaternion_xyzw"]),
            "row_metadata": {"row_id": f"plate_{c['rack']}", "coordinate_m": float(local["position_m"][1]),
                             "opening_normal_rack": R[:, 2].tolist(), "handle_direction_rack": R[:, 0].tolist()},
            "candidate_id": c["candidate_id"], "rack_local_pose": local, "source_index": int(index), "source": "packing"}


def candidate_body(geometry, c, basket):
    """FCL body of a pool candidate in the racks-in frame: organization geometry for dishes, the checker for cutlery."""
    posed = racks_in_pose(c["rack"], c["rack_local_pose"], basket)
    if c["kind"] in CUTLERY_KINDS:
        return geometry.checker._body(c["kind"], posed, c["candidate_id"])
    return geometry.body(c["kind"], posed, c["candidate_id"])


def candidate_conflict(geometry, ci, bi, cj, bj):
    """(ok, reason): same slot; dish-dish separation/nesting; cutlery-cutlery overlap (1 mm); dish vs cutlery never."""
    from .organized_candidates import pair_compatible
    if ci["slot_id"] == cj["slot_id"]:
        return False, "same_slot"
    cut_i, cut_j = ci["kind"] in CUTLERY_KINDS, cj["kind"] in CUTLERY_KINDS
    if cut_i and cut_j:
        return (True, None) if geometry.checker.pair(bi, bj)["valid"] else (False, "overlap")
    if cut_i or cut_j:
        return True, None                     # the basket volume is disjoint from every rack slot (measured: cross_kind_overlaps)
    return pair_compatible(geometry, bi, bj, check_opening=False)


def _safe_id(text):
    """Object ids must be USD/file identifiers; pattern variants carry '-' and '.'."""
    import re
    return re.sub(r"[^A-Za-z0-9_]", "_", text)


def static_frames():
    """Component frames of the closed appliance at the authored origins (basket-relative contact is frame-invariant)."""
    from .initial_state_candidates import COMPONENT_NAMES
    return {name: {"position_m": list(E.BODY_POSITIONS[name]), "quaternion_xyzw": [0., 0., 0., 1.]} for name in COMPONENT_NAMES}


def settle_down(checker, kind, local, basket, step=.001, max_steps=20, body_id="probe"):
    """Lower a basket-local pose until the checker (1 mm allowance) reports contact with the appliance, then back
    off one step: cutlery then drops a millimetre at release instead of the patterns' ~7 mm (which lands hard
    enough to trip the 2 mm peak-penetration gate or leave a knife rocking past the settle window)."""
    if checker.components is None:
        checker.update_components(static_frames())
    pose = {"position_m": list(local["position_m"]), "quaternion_xyzw": list(local["quaternion_xyzw"])}
    lowered = 0.
    for _ in range(max_steps):
        trial = dict(pose, position_m=[pose["position_m"][0], pose["position_m"][1], pose["position_m"][2] - step])
        body = checker._body(kind, racks_in_pose("SilverwareBasket", trial, basket), body_id)
        if not checker.against_components(body)["valid"]:
            break
        pose, lowered = trial, lowered + step
    return pose, lowered


def cutlery_pool(checker, per_slot=CUTLERY_PER_SLOT, patterns=CUTLERY_PATTERNS):
    """Cutlery candidates: the first ``per_slot`` variants of every basket slot per kind (kind-fixed compartments)."""
    data = json.loads(Path(patterns).read_text())
    static = (np.asarray(E.BODY_POSITIONS["SilverwareBasket"], dtype=float), np.asarray(E.IDENTITY, dtype=float))
    cands = []
    for kind in CUTLERY_KINDS:
        # Per slot, round-robin over the yaw families so every slot offers each orientation: the file sweeps
        # yaw 90 first, and a spoon bowl facing the tub wall is worth nothing to the score.
        by_slot = defaultdict(lambda: defaultdict(list))
        for k, pat in enumerate(data["patterns"][kind]):
            by_slot[pat["slot"]][pat["variant"].split("_")[0]].append((k, pat))
        picked = []
        for slot, families in by_slot.items():
            queues = [list(v) for v in families.values()]
            while len([1 for _ in picked if _[1]["slot"] == slot]) < per_slot and any(queues):
                for q in queues:
                    if q and sum(1 for _ in picked if _[1]["slot"] == slot) < per_slot:
                        picked.append(q.pop(0))
        for k, pat in sorted(picked, key=lambda kp: kp[0]):
            local = {"position_m": [float(v) for v in pat["position"]], "quaternion_xyzw": [float(v) for v in pat["quaternion_xyzw"]]}
            local, lowered = settle_down(checker, kind, local, static)
            cands.append({"kind": kind, "rack": "SilverwareBasket", "slot_id": pat["slot"], "position_xy_m": local["position_m"][:2],
                          "quaternion_xyzw": local["quaternion_xyzw"], "row_metadata": {"row_id": pat["slot"]},
                          "candidate_id": _safe_id(f"{kind}_{pat['slot']}_{pat['variant']}"), "rack_local_pose": local, "source_index": k,
                          "source": "cutlery_patterns", "release_hover_m": pat.get("release_hover_m"), "lowered_m": round(lowered, 4)})
    bodies = [checker._body(c["kind"], racks_in_pose("SilverwareBasket", c["rack_local_pose"], static), c["candidate_id"]) for c in cands]
    edges, reasons = [], Counter()
    for i in range(len(cands)):
        for j in range(i + 1, len(cands)):
            if cands[i]["slot_id"] == cands[j]["slot_id"]:
                edges.append([i, j]); reasons["same_slot"] += 1
            elif not checker.pair(bodies[i], bodies[j])["valid"]:
                edges.append([i, j]); reasons["overlap"] += 1
    return cands, edges, dict(reasons), bodies


def build_geometric_pool(pool_run=POOL_RUN, packing_run=PACKING_RUN, asset_dir=ASSET_DIR):
    """Bowls and mugs of the screened pool, packing-catalog plates that pass its screen and drain, and cutlery
    from the frozen basket patterns, with GEOMETRIC conflicts only (separation, nesting, same slot, overlap)."""
    from .initial_state_candidates import InitialCollisionChecker
    from .organization import OrganizationGeometry
    from .organized_candidates import pair_compatible
    catalog_path, graph_path = pool_run / "candidates_screened.json", pool_run / "compatibility_screened.json"
    catalog, graph = json.loads(catalog_path.read_text()), json.loads(graph_path.read_text())
    keep = [i for i, c in enumerate(catalog["candidates"]) if c["kind"] in ("bowl", "mug") and i in set(graph["allowed_indices"])]
    remap = {old: new for new, old in enumerate(keep)}
    cands = [candidate_fields(catalog["candidates"][i], i) for i in keep]
    checker = InitialCollisionChecker(asset_dir)
    geometry = OrganizationGeometry(checker=checker)
    static = (np.asarray(E.BODY_POSITIONS["SilverwareBasket"], dtype=float), np.asarray(E.IDENTITY, dtype=float))
    bodies = [candidate_body(geometry, c, static) for c in cands]
    edges, reasons = [], Counter()
    for a, b in graph["conflict_pairs"]:            # only stored pairs can be closer than the ray-expanded broadphase
        if a not in remap or b not in remap:
            continue
        i, j = remap[a], remap[b]
        ok, reason = candidate_conflict(geometry, cands[i], bodies[i], cands[j], bodies[j])
        if not ok:
            edges.append([i, j]); reasons[reason] += 1
    # plates: the packing catalog's plates that passed its rack-penetration screen and do not pool
    pcat_path, pgraph_path = packing_run / "candidates.json", packing_run / "compatibility.json"
    pcat, pgraph = json.loads(pcat_path.read_text()), json.loads(pgraph_path.read_text())
    allowed = set(pgraph["allowed_indices"])
    plates = {"total": sum(c["kind"] == "dinner_plate" for c in pcat["candidates"]), "allowed": 0, "kept": 0}
    for i, c in enumerate(pcat["candidates"]):
        if c["kind"] != "dinner_plate" or i not in allowed:
            continue
        plates["allowed"] += 1
        posed = racks_in_pose(c["rack"], c["rack_local_pose"])
        if E.pools(c["kind"], posed["position_m"], posed["quaternion_xyzw"]):
            continue
        plates["kept"] += 1
        pc = plate_fields(c, i)
        pb = candidate_body(geometry, pc, static)
        j = len(cands)
        for k in range(len(cands)):
            ok, reason = candidate_conflict(geometry, cands[k], bodies[k], pc, pb)
            if not ok:
                edges.append([k, j]); reasons[reason] += 1
        cands.append(pc); bodies.append(pb)
    n_dish = len(cands)
    ccands, cedges, creasons, cbodies = cutlery_pool(checker)
    cross = sum(not checker.pair(bd, bc)["valid"] for bd in bodies for bc in cbodies)   # expected 0: measured, not assumed
    edges += [[a + n_dish, b + n_dish] for a, b in cedges]
    reasons.update(creasons)
    cands += ccands
    return {"schema_version": 2, "kinds": list(KINDS), "candidates": cands, "conflict_pairs": edges,
            "conflict_reasons": dict(reasons), "counts_by_kind": dict(Counter(c["kind"] for c in cands)),
            "stored_pairs": len(graph["conflict_pairs"]), "plates": plates, "cutlery_per_slot": CUTLERY_PER_SLOT,
            "cross_kind_overlaps": int(cross),
            "source": {"catalog": str(catalog_path.relative_to(REPO_ROOT)), "catalog_sha256": _sha(catalog_path),
                       "graph": str(graph_path.relative_to(REPO_ROOT)), "graph_sha256": _sha(graph_path),
                       "packing_catalog": str(pcat_path.relative_to(REPO_ROOT)), "packing_catalog_sha256": _sha(pcat_path),
                       "packing_graph": str(pgraph_path.relative_to(REPO_ROOT)), "packing_graph_sha256": _sha(pgraph_path),
                       "cutlery_patterns": str(Path(CUTLERY_PATTERNS).relative_to(REPO_ROOT)), "cutlery_patterns_sha256": _sha(CUTLERY_PATTERNS)},
            "rule": "dishes: separation < 5 mm or nesting or same slot (opening-exposure edges dropped); cutlery: same slot or "
                    "FCL overlap with the 1 mm allowance; dish vs cutlery never (basket disjoint from rack slots, measured)"}


def load_pool(path=POOL_CACHE):
    return json.loads(Path(path).read_text())


def _adjacency(n, edges):
    adjacency = defaultdict(set)
    for a, b in edges:
        adjacency[a].add(b)
        adjacency[b].add(a)
    return {i: adjacency[i] for i in range(n)}


def random_greedy(catalog, allowed, adjacency, inventory, rng, attempts=50):
    """A random feasible set with exact per-kind counts (frigidaire_exposure_search's sampler)."""
    for _ in range(attempts):
        chosen, counts = [], Counter()
        for i in rng.permutation(allowed):
            kind = catalog["candidates"][i]["kind"]
            if counts[kind] >= inventory.get(kind, 0) or adjacency[i] & set(chosen):
                continue
            chosen.append(int(i))
            counts[kind] += 1
            if counts == inventory:
                return sorted(chosen)
    return None


def merged_pool(pool, instance, geometry):
    """Pool + one keep-in-place candidate per inside object that rests on the rack alone and does not pool."""
    basket = basket_in(instance["initial_snapshot"])
    cands = [dict(c) for c in pool["candidates"]]
    edges = [tuple(e) for e in pool["conflict_pairs"]]
    supported = {b for _, b in instance.get("support", [])}
    bodies = [candidate_body(geometry, c, basket) for c in cands]
    for o in instance["objects"]:
        if o["start"] == "Counter" or o["object_id"] in supported:
            continue
        posed = racks_in_pose(o["rack"], o["rack_local_pose"], basket)
        if E.pools(o["kind"], posed["position_m"], posed["quaternion_xyzw"]):
            continue
        R = quaternion_matrix_xyzw(o["rack_local_pose"]["quaternion_xyzw"])
        slot = (o.get("source") or {}).get("cutlery_slot", f"keep_{o['object_id']}")   # a kept utensil blocks its own slot
        k = {"kind": o["kind"], "rack": o["rack"], "slot_id": slot,
             "position_xy_m": list(o["rack_local_pose"]["position_m"][:2]),
             "quaternion_xyzw": list(o["rack_local_pose"]["quaternion_xyzw"]),
             "row_metadata": {"row_id": f"keep_{o['object_id']}", "coordinate_m": float(o["rack_local_pose"]["position_m"][1]),
                              "opening_normal_rack": R[:, 2].tolist(), "handle_direction_rack": R[:, 0].tolist()},
             "candidate_id": f"keep_{o['object_id']}", "keep_for": o["object_id"],
             "rack_local_pose": o["rack_local_pose"], "source_index": None, "source": "keep"}
        kb = candidate_body(geometry, k, basket)
        j = len(cands)
        for i, b in enumerate(bodies):
            if not candidate_conflict(geometry, cands[i], b, k, kb)[0]:
                edges.append((i, j))
        cands.append(k)
        bodies.append(kb)
    return {"candidates": cands, "conflict_pairs": [list(e) for e in edges]}


def proposal_score(pool, indices, basket, device=E.DEVICE):
    objects = [{"id": pool["candidates"][i]["candidate_id"], "kind": pool["candidates"][i]["kind"],
                "rack": pool["candidates"][i]["rack"], "rack_local_pose": pool["candidates"][i]["rack_local_pose"]}
               for i in indices]
    return score_objects(objects, basket, device)


def search_proposals(pool, inventory, deadline, seed=0, random_attempts=100, milp_limit_s=5., milp_rounds=10, log=None):
    """Distinct feasible sets with exact per-kind counts: random greedy, then up to ``milp_rounds`` MILP rounds with
    no-good cuts, within the deadline. The MILP knows only dish kinds, so proposals are drawn over the dish sub-pool
    and each is extended by a random-greedy cutlery assignment over the cutlery sub-pool (same adjacency)."""
    from .organized_candidates import solve_inventory
    n = len(pool["candidates"])
    adjacency = _adjacency(n, pool["conflict_pairs"])
    dish_inv = {k: int(v) for k, v in inventory.items() if k in DISH_KINDS and v}
    cut_inv = {k: int(v) for k, v in inventory.items() if k in CUTLERY_KINDS and v}
    dish_idx = [i for i, c in enumerate(pool["candidates"]) if c["kind"] in DISH_KINDS]
    cut_idx = [i for i, c in enumerate(pool["candidates"]) if c["kind"] in CUTLERY_KINDS]
    dish_set = set(dish_idx)
    graph = {"complete": True, "allowed_indices": dish_idx,
             "conflict_pairs": [list(e) for e in pool["conflict_pairs"] if e[0] in dish_set and e[1] in dish_set]}
    rng = np.random.default_rng(seed)
    seen, proposals, failed = set(), [], 0

    def extend(chosen):
        if not cut_inv:
            return chosen
        extra = random_greedy(pool, cut_idx, adjacency, cut_inv, rng)
        return None if extra is None else sorted(chosen + extra)

    for _ in range(random_attempts):
        if time.monotonic() > deadline:
            break
        chosen = random_greedy(pool, dish_idx, adjacency, dish_inv, rng) if dish_inv else []
        chosen = extend(chosen) if chosen is not None else None
        if chosen is None:
            failed += 1
        elif chosen and tuple(chosen) not in seen:
            seen.add(tuple(chosen))
            proposals.append(("random", chosen))
    excluded, round_ = [], 0
    while dish_inv and time.monotonic() + milp_limit_s < deadline and round_ < milp_rounds:
        sol = solve_inventory(pool, graph, dict(dish_inv), seed=round_, time_limit_s=milp_limit_s, excluded_sets=excluded)
        chosen = sol.get("selected_indices")
        if chosen is None:
            break
        excluded.append(frozenset(chosen))
        chosen = extend(sorted(chosen))
        if chosen is None:
            failed += 1
        elif tuple(chosen) not in seen:
            seen.add(tuple(chosen))
            proposals.append(("milp", chosen))
        round_ += 1
    if log:
        log(f"proposals: {len(proposals)} ({sum(m == 'random' for m, _ in proposals)} random, {round_} milp rounds"
            + (f", {failed} cutlery extensions failed" if failed else "") + ")")
    return proposals


# ----------------------------------------------------------------------------- instance <-> driver
def load_instance(path):
    return json.loads(Path(path).read_text())


def item_order(instance):
    """Counter objects first, top of the pile first, then inside objects in file order."""
    counter = [o for o in instance["objects"] if o["start"] == "Counter"]
    inside = [o for o in instance["objects"] if o["start"] != "Counter"]
    counter.sort(key=lambda o: -float(o["pose_world"]["position_m"][2]))
    return [o["object_id"] for o in counter + inside]


def to_rearrange_instance(instance):
    items = [{"item_id": o["object_id"], "object_class": o["kind"], "T_base_init": pose_T(o["pose_world"]),
              "target": {"T_base_obj": None}} for o in instance["objects"]]
    r = Instance(name=instance["instance_id"], machine="frigidaire", base_placement="none", state="racks_out",
                 items=items, meta={"n_objects": len(items), "counter_cap": instance["counter"]["cap"],
                                    "inventory": instance["inventory"], "seed": instance.get("seed")})
    r.planner_instance = instance
    return r


def goal_T(world, rack, local):
    return pose_T(world_from_local(world.frames[rack], local))


# ----------------------------------------------------------------------------- sequencer
def sequence(world, goals, order, cap, max_moves=None):
    """Greedy with buffering, dry-run on the mirror; a Move list, or None when it cannot finish.

    Pass 1 sends home any misplaced, unsupported object whose goal is free. Pass 2 parks one
    blocker on the counter, only while the band holds fewer than ``cap`` objects (the driver
    would refuse it otherwise). Objects that still support another never move.
    """
    saved = world.snapshot()
    poses = dict(saved)
    kinds = dict(world.classes)
    relocated, plan = set(), []
    limit = max_moves or 10 * max(1, len(order))

    def at(i):
        return bool(np.allclose(poses[i], goals[i], atol=1e-6))

    try:
        while not all(at(i) for i in order) and len(plan) < limit:
            step = None
            for i in order:
                if at(i) or world.resting_on(i):
                    continue
                if not world.move_collides(i, goals[i]):
                    step = (i, goals[i])
                    break
            if step is None and sum(world.in_counter(T) for T in poses.values()) < cap:
                for i in order:
                    if at(i):
                        continue
                    for b in world.blockers(i, goals[i]):
                        if b not in goals or at(b) or b in relocated or world.resting_on(b):
                            continue
                        T_buf = next((T for T in world.buffer_poses(kinds[b]) if not world.move_collides(b, T)), None)
                        if T_buf is not None:
                            relocated.add(b)
                            step = (b, T_buf)
                            break
                    if step is not None:
                        break
            if step is None:
                break
            plan.append(Move(step[0], np.asarray(step[1], dtype=float)))
            poses[step[0]] = np.asarray(step[1], dtype=float)
            world.sync({step[0]: poses[step[0]]}, kinds)
        done = all(at(i) for i in order)
    finally:
        world.sync(saved, kinds)
    return plan if done else None


def assign(pool, indices, instance, rng=None):
    """Bind keep candidates to their object, the rest per kind to the remaining objects (optionally shuffled)."""
    goals = {}
    free = defaultdict(list)
    for i in indices:
        c = pool["candidates"][i]
        if c.get("keep_for"):
            goals[c["keep_for"]] = i
        else:
            free[c["kind"]].append(i)
    if rng is not None:
        for kind in free:
            rng.shuffle(free[kind])
    for oid in item_order(instance):
        if oid in goals:
            continue
        kind = next(o["kind"] for o in instance["objects"] if o["object_id"] == oid)
        if not free[kind]:
            return None
        goals[oid] = free[kind].pop(0)
    return goals


def write_targets(rinstance, world, pool, goals):
    """Goals as driver targets (racks-out world 4x4) plus the rack-local bookkeeping the oracle needs."""
    targets = {}
    for it in rinstance.items:
        c = pool["candidates"][goals[it["item_id"]]]
        T = goal_T(world, c["rack"], c["rack_local_pose"])
        world.certify(T)
        it["target"] = {"T_base_obj": T, "rack": c["rack"], "rack_local_pose": c["rack_local_pose"],
                        "candidate_id": c["candidate_id"], "candidate_index": c.get("source_index"),
                        "keep": bool(c.get("keep_for"))}
        targets[it["item_id"]] = T
    return targets


class ExposurePlanner:
    """Goal = best-exposure feasible proposal (pool + keep-in-place) that the sequencer can complete."""

    name = "planner"
    pool = None            # set by the runner (load_pool)

    def __init__(self, seed=0, budget_s=60., pool=None, device=E.DEVICE, log=print):
        self.seed, self.budget_s, self.device, self.log = int(seed), float(budget_s), device, log
        if pool is not None:
            self.pool = pool

    def reset(self, instance, world):
        self.rinstance, self.world = instance, world
        self.instance = instance.planner_instance
        self.plan = None
        self._stats = {"planner": self.name, "proposals": 0, "scored": 0, "feasible": 0, "sequenced_rank": None,
                       "best_score": None, "chosen_score": None, "keeps": 0, "plan_moves": 0, "planning_s": 0.}

    def candidates(self, deadline):
        """Proposals from two searches: the plain pool with the reference's seed (so the planner sees every
        proposal the reference could), then the merged pool (keep-in-place) with its own seed."""
        merged = merged_pool(self.pool, self.instance, self.world.geometry)
        inventory = Counter(o["kind"] for o in self.instance["objects"])
        proposals = reference_search(self.pool, inventory, log=self.log)      # stage 1 = the reference search, verbatim
        seen = {tuple(c) for _, c in proposals}
        for method, chosen in search_proposals(merged, inventory, deadline - 20., seed=self.seed, random_attempts=50, log=self.log):
            if tuple(chosen) not in seen:                         # pool indices coincide: keeps are appended after the pool
                seen.add(tuple(chosen))
                proposals.append((method, chosen))
        basket = basket_in(self.instance["initial_snapshot"])
        scored = []
        for method, chosen in proposals:
            s = proposal_score(merged, chosen, basket, self.device)
            self._stats["scored"] += 1
            if s["feasible"]:
                scored.append((s["score"], s["worst"], method, chosen))
        scored.sort(key=lambda t: -t[0])
        self._stats.update(proposals=len(proposals), feasible=len(scored), best_score=scored[0][0] if scored else None)
        return merged, scored

    def _plan(self):
        started = time.monotonic()
        deadline = started + self.budget_s
        merged, scored = self.candidates(deadline)
        order, cap = item_order(self.instance), self.instance["counter"]["cap"]
        rng = np.random.default_rng(self.seed)
        for rank, (score, worst, method, chosen) in enumerate(scored):
            if time.monotonic() > deadline:
                break
            for attempt in range(5):
                goals = assign(merged, chosen, self.instance, None if attempt == 0 else rng)
                if goals is None:
                    break
                targets = write_targets(self.rinstance, self.world, merged, goals)
                plan = sequence(self.world, targets, order, cap)
                if plan is not None:
                    self._stats.update(sequenced_rank=rank, chosen_score=score, chosen_worst=worst, chosen_method=method,
                                       keeps=sum(bool(merged["candidates"][i].get("keep_for")) for i in chosen),
                                       plan_moves=len(plan), planning_s=round(time.monotonic() - started, 3))
                    self.log(f"[OK] {self.name}: proposal rank {rank} ({method}) S={score:.3f} W={worst:.3f}, "
                             f"{len(plan)} moves, {self._stats['keeps']} kept, {self._stats['planning_s']} s")
                    return plan
        for it in self.rinstance.items:      # nothing sequenced: leave no half-written targets behind
            it["target"] = {"T_base_obj": None}
        self._stats["planning_s"] = round(time.monotonic() - started, 3)
        return []

    def next_move(self, obs):
        if self.plan is None:
            self.plan = self._plan()
        return self.plan.pop(0) if self.plan else None

    def stats(self):
        return dict(self._stats)


class FirstFitBaseline(ExposurePlanner):
    """Goal = first compatible pool candidate per object in pool order; no keep, no scoring."""

    name = "baseline"

    def candidates(self, deadline):
        pool = {"candidates": [dict(c) for c in self.pool["candidates"]], "conflict_pairs": [list(e) for e in self.pool["conflict_pairs"]]}
        adjacency = _adjacency(len(pool["candidates"]), pool["conflict_pairs"])
        inventory = Counter(o["kind"] for o in self.instance["objects"])
        basket = basket_in(self.instance["initial_snapshot"])
        scored = []
        by_kind = Counter(c["kind"] for c in pool["candidates"])
        orders = {"pool": list(range(len(pool["candidates"]))),
                  "scarce_first": sorted(range(len(pool["candidates"])), key=lambda i: (by_kind[pool["candidates"][i]["kind"]], i))}
        for name, order in orders.items():           # pool order first; scarce kinds first only when that finds nothing
            for skip in range(10):                   # deterministic fallbacks: skip the first k candidates
                chosen, counts = [], Counter()
                for i in order[skip:]:
                    c = pool["candidates"][i]
                    if counts[c["kind"]] >= inventory[c["kind"]] or adjacency[i] & set(chosen):
                        continue
                    chosen.append(i)
                    counts[c["kind"]] += 1
                if counts == inventory:
                    s = proposal_score(pool, chosen, basket, self.device)
                    self._stats["scored"] += 1
                    if s["feasible"]:
                        scored.append((s["score"], s["worst"], f"first_fit_{name}_skip{skip}", sorted(chosen)))
            if scored:
                break
        if not scored:                               # first-fit cannot reach the counts: first feasible random draws, unranked
            for method, chosen in search_proposals(pool, inventory, time.monotonic() + 20., seed=0, random_attempts=10, milp_rounds=0):
                s = proposal_score(pool, chosen, basket, self.device)
                self._stats["scored"] += 1
                if s["feasible"]:
                    scored.append((s["score"], s["worst"], f"first_feasible_{method}", chosen))
        self._stats.update(proposals=len(scored), feasible=len(scored), best_score=scored[0][0] if scored else None)
        return pool, scored          # in first-fit / draw order, not by score


ALGORITHMS = {"planner": ExposurePlanner, "baseline": FirstFitBaseline}


# ----------------------------------------------------------------------------- references, gate, records
def organized_reference(inventory, device=E.DEVICE):
    key = (int(inventory.get("bowl", 0)), int(inventory.get("mug", 0)))
    name = ORGANIZED_REFERENCE.get(key)
    if name is None:
        return None
    path = STATES / f"organized_20260911_seed20260911/states/{name}.json"
    r = E.score_state(path, device=device, baselines=False)
    return {"state": name, "score": r["score"], "worst": r["worst"]}


def reference_search(pool, inventory, log=None):
    r = REFERENCE_SEARCH
    return search_proposals(pool, inventory, time.monotonic() + r["seconds"], seed=r["seed"],
                            random_attempts=r["random_attempts"], milp_rounds=r["milp_rounds"], log=log)


def certify(instance, pool, budget_s=60., seed=0, device=E.DEVICE, log=print):
    """References and a sequenceability certificate; Kit-free."""
    inventory = Counter(o["kind"] for o in instance["objects"])
    basket = basket_in(instance["initial_snapshot"])
    started = time.monotonic()
    proposals = reference_search(pool, inventory, log=log)
    best = None
    for method, chosen in proposals:
        s = proposal_score(pool, chosen, basket, device)
        if s["feasible"] and (best is None or s["score"] > best["score"]):
            best = {"score": s["score"], "worst": s["worst"], "method": method, "indices": chosen}
    world = PlannerWorld(instance)
    rinst = to_rearrange_instance(instance)
    planner = ExposurePlanner(seed=seed, budget_s=budget_s, pool=pool, device=device, log=log)
    planner.reset(rinst, world)
    world.sync({it["item_id"]: it["T_base_init"] for it in rinst.items}, {it["item_id"]: it["object_class"] for it in rinst.items})
    plan = planner._plan()
    return {"reference": {**(best or {}), "proposals": len(proposals), "search": dict(REFERENCE_SEARCH),
                          "pool": f"geometric {'+'.join(sorted(set(c['kind'] for c in pool['candidates'])))}, no keep"},
            "organized": organized_reference(inventory, device),
            "certificate": {"sequenceable": bool(plan), "moves": len(plan), **planner.stats()},
            "seconds": round(time.monotonic() - started, 1)}


def gate_manifest(instance, rinstance):
    """Final-gate manifest for frigidaire_initial_state_validate.py: every object at its goal, in-rack only."""
    objects = []
    for it in rinstance.items:
        t = it["target"]
        objects.append({"object_id": it["item_id"], "kind": it["object_class"], "rack": t["rack"],
                        "rack_local_pose": t["rack_local_pose"], "goal_candidate_id": t.get("candidate_id"),
                        "keep": bool(t.get("keep"))})
    return {"schema_version": 1, "purpose": "planner_final_gate", "instance_id": instance["instance_id"],
            "objects": objects, "baseline": instance["baseline"]}


def settled_score(result, manifest, device=E.DEVICE):
    """Score the gate's settled snapshot the way the plan was scored (racks-in from rack-local poses)."""
    snap = result["initial_snapshot"]
    objects, moved = [], []
    for o in manifest["objects"]:
        measured = snap["poses"][o["object_id"]]
        local = local_from_world(snap["poses"][o["rack"]], measured)
        objects.append({"id": o["object_id"], "kind": o["kind"], "rack": o["rack"], "rack_local_pose": local})
        moved.append(float(np.linalg.norm(np.asarray(local["position_m"]) - np.asarray(o["rack_local_pose"]["position_m"]))))
    s = score_objects(objects, basket_in(snap), device)
    return {**s, "max_settle_displacement_m": max(moved) if moved else 0.}


def instance_from_state(path, cap, counter_ids=()):
    """A planner instance from a settled state file (no counter pile): for tests and dry runs."""
    state = load_instance(path)
    snap = state["initial_snapshot"]
    objects = []
    for o in state["objects"]:
        start = "Counter" if o["object_id"] in counter_ids else o["rack"]
        entry = {"object_id": o["object_id"], "kind": o["kind"], "start": start, "pose_world": o["pose_world"]}
        if start != "Counter":
            entry.update(rack=o["rack"], rack_local_pose=o["rack_local_pose"])
        objects.append(entry)
    inventory = dict(Counter(o["kind"] for o in objects))
    return {"schema_version": 1, "purpose": "planner_instance", "instance_id": f"state_{Path(path).stem}",
            "seed": state.get("seed"), "n_objects": len(objects), "inventory": inventory,
            "counter": {**COUNTER, "cap": int(cap), "start_count": len(counter_ids)},
            "objects": objects, "initial_snapshot": {k: snap[k] for k in ("joints", "poses")},
            "support": [], "baseline": state["baseline"], "source_state": str(path)}
