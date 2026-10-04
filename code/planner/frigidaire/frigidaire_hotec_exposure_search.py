#!/usr/bin/env python3
# Copyright (c) 2026, dishsim project.
# SPDX-License-Identifier: BSD-3-Clause
"""Highest-exposure full load of the HOTEC set under the rev-5 scorer, Kit-free.

The rev-5 scorer (dishsim_frigidaire.exposure) keys dish kinds by name and derives food-contact
surfaces from the prototype lathe meshes, so the HOTEC plate, bowl and cup are registered here as
``hotec_plate`` / ``hotec_bowl`` / ``hotec_cup`` from their v2 USD visual meshes: a food-contact face
looks inward (toward the lathe axis) or up, and lies above the foot plane (bowl and cup interiors,
plate top; undersides, outer walls and the foot recess excluded); the mouth ring is the interior's
highest ring; pooling applies as for the prototypes.

Search: coordinate ascent from a complete load (run v8: 8 plates, 8 bowls, 8 cups racked). Every
piece owns the realistic candidate family of its kind (the planner's slot families: plates in every
front and rear gap x leans x offsets, bowls in the lower zones and every upper centre gap x tilts x
lifts, cups in the ten ladder slots x variants). One move re-poses one piece to an FCL-free,
non-nesting alternative given the other 23; alternatives are ranked by their cached ISOLATED
exposure (appliance occluders only) and the best few plus random picks are scored in FULL (all 24
as occluders); a move is kept when S rises. Sweeps repeat until no piece improves. The result is a
sampled maximum, not a proof.

    code/util/run_py.sh code/planner/frigidaire/frigidaire_hotec_exposure_search.py \\
        --assets data/assets/models/hotec_wheatstraw/v2 --sweeps 3 --seed 0
"""
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
sys.path[:0] = [str(ROOT / "code/src"), str(ROOT / "code/frigidaire/src")]
from dishsim_frigidaire.usd_bootstrap import ensure_usd            # noqa: E402
ensure_usd()
from dishsim_frigidaire import exposure as E                         # noqa: E402
from dishsim_frigidaire import geometry                              # noqa: E402
from dishsim_frigidaire.asset import BODY_POSITIONS, COMPONENT_FILES  # noqa: E402
from dishsim_frigidaire.claims import (LOWER_BOWL_FLOOR, LOWER_BOWL_LIFTS, LOWER_PLATE_FLOOR, UPPER_GLASS_FLOOR,  # noqa: E402
                                       _tilted, lower_bowl_variants, slabs_disjoint, upper_bowl_variants)
from dishsim_frigidaire.geometry import lower_tine_positions, upper_tine_positions   # noqa: E402
from dishsim_frigidaire.loading import CollisionWorld, _candidate, collision_parts, rotation, visual_points  # noqa: E402
from dishsim_frigidaire.paths import ASSET_DIR, REPO_ROOT            # noqa: E402

_spec = importlib.util.spec_from_file_location("hotec_load", Path(__file__).with_name("frigidaire_hotec_load.py"))
HL = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(HL)

KINDS = ("plate", "bowl", "cup")
SCORER_KIND = {kind: f"hotec_{kind}" for kind in KINDS}
RIM_BAND_M = .001
OUT = REPO_ROOT / "data/results/exposure/frigidaire/hotec"


# --------------------------------------------------------------------------- HOTEC kinds in the scorer

def visual_triangles(filename):
    """(T, 3, 3) object-frame triangles of every visual (non-collision) mesh of a HOTEC USD."""
    from pxr import Usd, UsdGeom, UsdPhysics
    stage = Usd.Stage.Open(str(filename))
    root = stage.GetDefaultPrim()
    cache = UsdGeom.XformCache()
    root_inverse = cache.GetLocalToWorldTransform(root).GetInverse()
    tris = []
    for prim in Usd.PrimRange(root):
        if not prim.IsA(UsdGeom.Mesh) or prim.HasAPI(UsdPhysics.CollisionAPI):
            continue
        mesh = UsdGeom.Mesh(prim)
        matrix = np.asarray(cache.GetLocalToWorldTransform(prim) * root_inverse).T
        points = np.asarray(mesh.GetPointsAttr().Get(), dtype=float) @ matrix[:3, :3].T + matrix[:3, 3]
        counts = np.asarray(mesh.GetFaceVertexCountsAttr().Get())
        indices = np.asarray(mesh.GetFaceVertexIndicesAttr().Get())
        faces, k = [], 0
        for n in counts:
            poly = indices[k:k + n]; k += n
            for j in range(1, n - 1):
                faces.append((poly[0], poly[j], poly[j + 1]))
        tris.append(points[np.asarray(faces)])
    if not tris:
        raise ValueError(f"No visual meshes in {filename}")
    return np.concatenate(tris)


def register_hotec(assets_dir):
    """Register the three HOTEC kinds in the scorer (visual occluders + food-contact surfaces)."""
    assets_dir = Path(assets_dir)
    catalog = json.loads((assets_dir / "catalog.json").read_text())
    info = {}
    for kind in KINDS:
        name = SCORER_KIND[kind]
        tri = visual_triangles(assets_dir / f"{kind}.usda")
        cross = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
        areas = 0.5 * np.linalg.norm(cross, axis=1)
        keep = areas > 1e-14
        tri, cross, areas = tri[keep], cross[keep], areas[keep]
        normals = cross / (2. * areas[:, None])
        centroid = tri.mean(axis=1)
        radial = centroid.copy(); radial[:, 2] = 0.
        radial /= np.maximum(np.linalg.norm(radial, axis=1, keepdims=True), 1e-9)
        inward = np.einsum("ij,ij->i", normals, radial) < -.1          # faces the lathe axis: vessel interior wall
        up = normals[:, 2] > .5                                          # faces up: plate top, vessel floor
        floor_top = float(catalog["items"][kind]["usd_bounds_m"][0][2]) + .003
        food = (inward | up) & (centroid[:, 2] > floor_top)
        ftri, fnorm, farea = tri[food], normals[food], areas[food]
        inner = ftri.reshape(-1, 3)
        top = float(inner[:, 2].max())
        rim = inner[inner[:, 2] >= top - RIM_BAND_M]
        E._VISUALS[name] = [tri]
        E._FOOD_CONTACT[name] = E.FoodContact(name, ftri, fnorm, farea, float(farea.sum()), rim, inner)
        info[name] = {"visual_triangles": int(len(tri)), "food_contact_triangles": int(len(ftri)),
                      "food_contact_area_m2": float(farea.sum()), "rim_z_m": top, "rim_vertices": int(len(rim)),
                      "rule": "faces looking inward (toward the axis) or up, above the foot plane + 3 mm",
                      "usd_sha256": catalog["items"][kind]["sha256"], "mass_kg": catalog["items"][kind].get("mass_kg")}
    if not all(k in E.POOLING_KINDS for k in SCORER_KIND.values()):
        E.POOLING_KINDS = tuple(E.POOLING_KINDS) + tuple(SCORER_KIND.values())
    return info


# --------------------------------------------------------------------------- candidate families

def candidate_families(assets_dir):
    """Every realistic pose of each kind (planner families, all slots), plus the collision parts/points."""
    assets_dir = Path(assets_dir)
    parts, points = {}, {}
    for kind in KINDS:
        path = assets_dir / f"{kind}.usda"
        parts[kind], points[kind] = collision_parts(path), visual_points(path)
    heights = {kind: float(points[kind][:, 2].max()) for kind in KINDS}
    teeth, rows = lower_tine_positions()
    mids = (teeth[:-1] + teeth[1:]) / 2
    front_y, rear_y = float((rows[0] + rows[1]) / 2), float((rows[-2] + rows[-1]) / 2)
    fam = {"plate": [], "bowl": [], "cup": []}
    plate_centre = np.array([0., 0., heights["plate"] / 2])
    for bank, y, gaps in (("front", front_y + HL.FRONT_PLATE_INSET_Y, tuple(HL._FRONT_USABLE)),
                          ("rear", rear_y, tuple(geometry.lower_plate_gaps("rear")))):
        for gap in gaps:
            for lean in HL.PLATE_LEANS:
                orient = rotation("Y", math.radians(90 - lean))
                shift = orient @ plate_centre
                for off in HL.PLATE_OFFSETS:
                    fam["plate"].append((_candidate("plate", "LowerRack", f"lower_{bank}_{gap:02d}", float(mids[gap]) + off - shift[0],
                                                    y - shift[1], orient, points["plate"], LOWER_PLATE_FLOOR,
                                                    f"lean{lean}_offset{off}"), orient))
    fam["bowl"] += list(lower_bowl_variants(points["bowl"], front_y, "lower_frontright"))
    mid_y = float((rows[2] + rows[3]) / 2)
    x_free = geometry.PARAMETERS["lower_rack"]["basket_reserved_x"][0] - .005
    families = [("Y", t) for t in (120, 135, 105, 150)] + [("X", t) for t in (120, 135, 105, 150)]
    zones = [("rear", rear_y, (0., .010, -.010, float(rows[-2] - rear_y), float(rows[-1] - rear_y)), np.arange(-.200, .0701, .005)),
             ("mid", mid_y, (0., .010, -.010, .020, -.020), np.arange(-.200, .0651, .005)),
             ("rearright", rear_y, (0., .010, -.010, float(rows[-1] - rear_y)), np.arange(-.020, .0701, .005))]
    for zone, zone_y, dys, xs in zones:
        for axis, tilt in families:
            orient = rotation(axis, math.radians(tilt))
            for dy in dys:
                for x in xs:
                    base, _ = _tilted("bowl", "LowerRack", f"lower_{zone}", float(x), zone_y + dy, orient,
                                      points["bowl"], LOWER_BOWL_FLOOR, f"{axis}{tilt}_y{zone_y + dy:.4f}_x{x:.3f}")
                    wp = points["bowl"] @ orient.T + np.asarray(base["position"])
                    if wp[:, 0].max() > x_free or np.abs(wp[:, 1]).max() > .270:
                        continue
                    for lift in LOWER_BOWL_LIFTS:
                        fam["bowl"].append((dict(base, position=[base["position"][0], base["position"][1], base["position"][2] + lift],
                                                 variant=f'{base["variant"]}_lift{lift}'), orient))
    _, upper_ys = upper_tine_positions()
    for a, b in HL.UPPER_BOWL_GAPS:
        y = float((upper_ys[a] + upper_ys[b]) / 2)
        fam["bowl"] += list(upper_bowl_variants(a, y, points["bowl"]))
    for side, index in HL.CUP_SLOTS + tuple((side, index) for index in (0, 5) for side in (-1, 1)):
        y = -.210 + .085 * index
        for x, lean, lift in HL.CUP_VARIANTS:
            c = HL.sloped_cup(f"glass_{'left' if side < 0 else 'right'}_{index}", side * x, y, lean, points["cup"],
                              UPPER_GLASS_FLOOR, heights["cup"])
            c["position"][2] += lift
            c["variant"] += f"_x{x}_lift{lift}"
            fam["cup"].append((c, None))
    return fam, parts, points


# --------------------------------------------------------------------------- scoring helpers

def world_objects(entries):
    objs = []
    for e in entries:
        p, q = E.compose_pose(BODY_POSITIONS[e["rack"]], E.IDENTITY, e["position"], e["quaternion_xyzw"])
        objs.append({"id": e["id"], "kind": SCORER_KIND[e["kind"]], "rack": e["rack"], "position_m": p, "quaternion_xyzw": q})
    return objs


def full_score(entries, name, device, samples, directions):
    basket = (np.asarray(BODY_POSITIONS["SilverwareBasket"], dtype=float), np.asarray(E.IDENTITY, dtype=float))
    res = E.score_arrangement(E.Arrangement(name, "", "", world_objects(entries), basket), device=device,
                              samples=samples, directions=directions, baselines=False)
    return E.strip_samples(res)


class IsolatedExposure:
    """Cached isolated exposure (appliance occluders only), evaluated in batches per rack."""
    def __init__(self, device, samples, directions):
        self.device, self.samples, self.directions = device, samples, directions
        basket = (np.asarray(BODY_POSITIONS["SilverwareBasket"], dtype=float), np.asarray(E.IDENTITY, dtype=float))
        self.soup = E.occluder_soup(E.Arrangement("appliance", "", "", [], basket), appliance=True)
        self.sources = {rack: E.rack_sources(rack, directions, E.DEFAULTS["ceiling_weight"])[:2] for rack in ("LowerRack", "UpperRack")}
        self.cache = {}

    @staticmethod
    def key(c):
        return (c["kind"], c["rack"], tuple(np.round(c["position"], 4)), tuple(np.round(c["quaternion_xyzw"], 4)))

    def evaluate(self, candidates):
        todo = [c for c in candidates if self.key(c) not in self.cache]
        for rack in ("LowerRack", "UpperRack"):
            batch = [c for c in todo if c["rack"] == rack]
            for start in range(0, len(batch), 64):
                chunk = batch[start:start + 64]
                pts, nrm = [], []
                for c in chunk:
                    fc = E.food_contact(SCORER_KIND[c["kind"]])
                    s = E.surface_samples(fc, self.samples)
                    p, q = E.compose_pose(BODY_POSITIONS[rack], E.IDENTITY, list(c["position"]), list(c["quaternion_xyzw"]))
                    rot = E.quaternion_matrix_xyzw(q)
                    pts.append(s.points @ rot.T + np.asarray(p)); nrm.append(s.normals @ rot.T)
                ex = E.exposure_of(np.concatenate(pts), np.concatenate(nrm), self.soup, self.device, self.directions,
                                   sources=self.sources[rack]).reshape(len(chunk), self.samples)
                for c, e in zip(chunk, ex.mean(axis=1)):
                    self.cache[self.key(c)] = float(e)
        return np.array([self.cache[self.key(c)] for c in candidates])


# --------------------------------------------------------------------------- insertion-order gate

DESCENT_STEP_M, DESCENT_HEIGHT_M = .010, .300      # a hand lowers the piece straight down from 30 cm above its rest pose
LEAN_STEP_DEG = 4.                                 # plates: lowered UPRIGHT, then rotated about their bottom edge to the final lean
SWEEP_STATICS = {"LowerRack": ("LowerRack", "SilverwareBasket"), "UpperRack": ("UpperRack",)}   # the OTHER rack is pushed in
# Plates thread their thin rim between the tines by hand (the coupe body is deeper than the 32 mm clear gap), so
# rack contact along a plate's insertion path is not a gate; piece-to-piece crossings are.
RACK_GATED_KINDS = ("bowl", "cup")


class SweepWorld:
    """FCL manager holding one rack's static parts only (that rack is pulled out while it is loaded)."""
    _parts = {}

    def __init__(self, rack, parts, points):
        import fcl
        self.fcl, self.parts, self.points = fcl, parts, points
        self.manager = fcl.DynamicAABBTreeCollisionManager()
        objs = []
        for body in SWEEP_STATICS[rack]:
            if body not in SweepWorld._parts:
                SweepWorld._parts[body] = collision_parts(ASSET_DIR / COMPONENT_FILES[body])
            objs.extend(self.transform(SweepWorld._parts[body], np.eye(3), BODY_POSITIONS[body]))
        self.manager.registerObjects(objs)
        self.manager.setup()

    def transform(self, parts, orient, position):
        fcl = self.fcl
        return [fcl.CollisionObject(part.geometry, fcl.Transform(orient @ part.rotation, orient @ part.translation + position))
                for part in parts]

    def posed(self, entry, dz=0., orient=None, position=None):
        position = (np.asarray(entry["position"], dtype=float) if position is None else np.asarray(position, dtype=float)) \
            + BODY_POSITIONS[entry["rack"]] + [0., 0., dz]
        orient = E.quaternion_matrix_xyzw(entry["quaternion_xyzw"]) if orient is None else orient
        return self.transform(self.parts[entry["kind"]], orient, position)

    def path(self, entry):
        """Insertion sweep poses (orient, position in rack frame) from far above down to the rest pose."""
        rot_final = E.quaternion_matrix_xyzw(entry["quaternion_xyzw"])
        pos_final = np.asarray(entry["position"], dtype=float)
        poses = []
        if entry["kind"] == "plate":
            # phase 2 (reversed): rotate about the plate's lowest rest point from the final lean back to upright ...
            pts = self.points["plate"] @ rot_final.T + pos_final
            pivot = pts[np.argmin(pts[:, 2])]
            pivot_local = rot_final.T @ (pivot - pos_final)
            lean = math.degrees(math.atan2(rot_final[0, 2], rot_final[2, 2]))      # tilt of the plate's +Z (face normal) from vertical... sign kept
            n = max(1, int(round(abs(lean) / LEAN_STEP_DEG)))
            upright = None
            for k in range(1, n + 1):
                theta = math.radians(lean * (1 - k / n))
                rot = rotation("Y", math.pi / 2 - theta) if False else None
                # rotate the final orientation about the world Y axis by -lean*k/n (undo the lean progressively)
                c, s_ = math.cos(-math.radians(lean) * k / n), math.sin(-math.radians(lean) * k / n)
                ry = np.array([[c, 0, s_], [0, 1, 0], [-s_, 0, c]])
                rot = ry @ rot_final
                pos = pivot - rot @ pivot_local
                poses.append((rot, pos))
                upright = (rot, pos)
            # ... then phase 1 (reversed): lift the upright plate straight up
            for dz in np.arange(DESCENT_STEP_M, DESCENT_HEIGHT_M + 1e-9, DESCENT_STEP_M):
                poses.append((upright[0], upright[1] + [0., 0., dz]))
        else:
            for dz in np.arange(DESCENT_STEP_M, DESCENT_HEIGHT_M + 1e-9, DESCENT_STEP_M):
                poses.append((rot_final, pos_final + [0., 0., dz]))
        return poses

    def hits(self, objects, manager=None):
        fcl = self.fcl
        data = fcl.CollisionData(request=fcl.CollisionRequest(num_max_contacts=1))
        for obj in objects:
            (manager or self.manager).collide(obj, data, fcl.defaultCollisionCallback)
            if data.result.is_collision:
                return True
        return False

    def path_hits_rack(self, entry):
        if entry["kind"] not in RACK_GATED_KINDS:
            return False
        return any(self.hits(self.posed(entry, orient=o, position=p)) for o, p in self.path(entry))

    def path_hits_piece(self, entry, other):
        import fcl
        m = fcl.DynamicAABBTreeCollisionManager()
        m.registerObjects(self.posed(other)); m.setup()
        return any(self.hits(self.posed(entry, orient=o, position=p), m) for o, p in self.path(entry))


_SWEEP = {}


def sweep_world(rack, parts, points):
    if rack not in _SWEEP:
        _SWEEP[rack] = SweepWorld(rack, parts, points)
    return _SWEEP[rack]


def insertion_graph(entries, parts, points, cache=None):
    """Edges (i before j): i's descent path crosses j's rest pose. Also the pieces whose path hits the rack itself."""
    cache = {} if cache is None else cache
    rack_hits, edges = [], []
    for i, a in enumerate(entries):
        w = sweep_world(a["rack"], parts, points)
        ka = (a["id"], tuple(np.round(a["position"], 4)), tuple(np.round(a["quaternion_xyzw"], 4)))
        if ("rack", ka) not in cache:
            cache[("rack", ka)] = w.path_hits_rack(a)
        if cache[("rack", ka)]:
            rack_hits.append(a["id"])
        for j, b in enumerate(entries):
            if i == j or b["rack"] != a["rack"]:
                continue
            kb = (b["id"], tuple(np.round(b["position"], 4)), tuple(np.round(b["quaternion_xyzw"], 4)))
            if ("pair", ka, kb) not in cache:
                cache[("pair", ka, kb)] = w.path_hits_piece(a, b)
            if cache[("pair", ka, kb)]:
                edges.append((a["id"], b["id"]))          # a must be placed before b
    return edges, rack_hits


def insertion_order(entries, parts, points, cache=None):
    """Topological order (lower rack first, rear to front, left to right as the tie-break) or None."""
    edges, rack_hits = insertion_graph(entries, parts, points, cache)
    if rack_hits:
        return None, edges, rack_hits
    ids = [e["id"] for e in entries]
    after = {i: set() for i in ids}; indeg = {i: 0 for i in ids}
    for a, b in edges:
        if b not in after[a]:
            after[a].add(b); indeg[b] += 1
    key = {e["id"]: (0 if e["rack"] == "LowerRack" else 1, -e["position"][1], e["position"][0]) for e in entries}
    ready = sorted([i for i in ids if indeg[i] == 0], key=lambda i: key[i])
    order = []
    while ready:
        i = ready.pop(0); order.append(i)
        for b in after[i]:
            indeg[b] -= 1
            if indeg[b] == 0:
                ready.append(b); ready.sort(key=lambda x: key[x])
    if len(order) != len(ids):
        return None, edges, rack_hits          # a cycle: no hand can insert this set
    return order, edges, rack_hits


def describe_placement(entry):
    """Human placement wording: tine gaps from the LEFT, rows and upper positions from the FRONT."""
    import re
    xs, ys = lower_tine_positions(); _, uys = upper_tine_positions()
    p = np.asarray(entry["position"]) * 1000.
    rot = E.quaternion_matrix_xyzw(entry["quaternion_xyzw"]); axis = rot @ np.array([0., 0., 1.])
    side = lambda v: "right" if v > 0 else "left"
    fb = lambda v: "rear" if v > 0 else "front"
    kind, slot, variant = entry["kind"], entry["slot"], entry["variant"]
    if kind == "plate":
        gap = int(re.search(r"_(\d+)$", slot).group(1)); bank = "front" if "front" in slot else "rear"
        lean = abs(int(re.search(r"lean(-?\d+)", variant).group(1)))
        return (f"LOWER rack, {bank} bank (between rows {'1 and 2' if bank == 'front' else '5 and 6'}): stand the plate on edge between "
                f"tines {gap + 1} and {gap + 2} from the left, eating face toward the {side(axis[0])}, top leaning about {lean} deg to the {side(axis[0])}")
    if kind == "bowl" and entry["rack"] == "UpperRack":
        a = int(re.search(r"_(\d+)$", slot).group(1))
        return (f"UPPER rack, centre column: bowl mouth down over the gap between tine positions {a + 1} and {a + 2} from the front, "
                f"opening tilted toward the {fb(axis[1])}")
    if kind == "bowl":
        col = int(np.argmin(np.abs(xs * 1000 - p[0]))) + 1; row = int(np.argmin(np.abs(ys * 1000 - p[1]))) + 1
        rows = {1: "rows 1 and 2", 2: "rows 1 and 2", 3: "rows 3 and 4", 4: "rows 3 and 4", 5: "rows 5 and 6", 6: "rows 5 and 6"}[row]
        toward = side(axis[0]) if abs(axis[0]) > abs(axis[1]) else fb(axis[1])
        return (f"LOWER rack: bowl mouth down over the tines of {rows}, centred on tine column {col} from the left, "
                f"opening tilted toward the {toward}, resting on the tine tips")
    m = re.match(r"glass_(left|right)_(\d)", slot)
    pos = int(np.argmin(np.abs(uys * 1000 - p[1]))) + 1
    return (f"UPPER rack, {m.group(1)} glass shelf (outer channel): cup upside down beside tine position {pos} from the front "
            f"of the outer column, leaning about 8 deg toward the wall")


def load_entries(layout):
    entries = []
    for o in layout["objects"]:
        if o["rack"] == "Counter":
            raise ValueError("the start layout must rack every piece")
        entries.append({"id": o["object_id"], "kind": o["kind"], "rack": o["rack"], "slot": o["slot"], "variant": o["variant"],
                        "position": list(o["rack_local_pose"]["position_m"]), "quaternion_xyzw": list(o["rack_local_pose"]["quaternion_xyzw"])})
    return entries


def orient_of(entry):
    return E.quaternion_matrix_xyzw(entry["quaternion_xyzw"])


def free_alternatives(i, entries, fam, parts, points, world_factory):
    """Candidates of piece i's kind that are FCL-free against the other 23 and non-nesting with same-rack bowls."""
    others = [e for j, e in enumerate(entries) if j != i]
    world = world_factory()
    for e in others:
        world.add({"kind": e["kind"], "rack": e["rack"], "position": e["position"], "quaternion_xyzw": e["quaternion_xyzw"]})
    kind = entries[i]["kind"]
    used = {(e["rack"], e["slot"]) for e in others}
    out = []
    for c, o in fam[kind]:
        if (c["rack"], c["slot"]) in used and kind != "bowl":
            continue                                        # plates/cups: one per slot; bowls carry zone names, FCL decides
        if kind == "bowl":
            ok = True
            for e in others:
                if e["kind"] == "bowl" and e["rack"] == c["rack"] and not slabs_disjoint(points["bowl"], (e, orient_of(e)), (c, o)):
                    ok = False; break
            if not ok:
                continue
        if world.collides(world.candidate_objects(c)):
            continue
        out.append((c, o))
    return out


def layout_from(entries, template, name, score):
    objects = []
    for e in entries:
        t = next(o for o in template["objects"] if o["object_id"] == e["id"])
        objects.append({**t, "rack": e["rack"], "slot": e["slot"], "variant": e["variant"],
                        "rack_local_pose": {"position_m": [float(v) for v in e["position"]], "quaternion_xyzw": [float(v) for v in e["quaternion_xyzw"]]}})
    by_rack = {}
    for o in objects:
        by_rack.setdefault(o["rack"], {}).setdefault(o["kind"], 0); by_rack[o["rack"]][o["kind"]] += 1
    counts = dict(template["counts"], fitted={k: sum(o["kind"] == k for o in objects) for k in KINDS},
                  counter={k: 0 for k in KINDS}, by_rack=by_rack, total=len(objects))
    return {**template, "objects": objects, "counts": counts, "purpose": "hotec_load_layout",
            "exposure_search": {"selected": name, "score": score}}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--assets", type=Path, default=ROOT / "data/assets/models/hotec_wheatstraw/v2")
    parser.add_argument("--start-layout", type=Path, default=REPO_ROOT / "data/results/hotec/frigidaire/v8/layout.json")
    parser.add_argument("--sweeps", type=int, default=3)
    parser.add_argument("--top", type=int, default=6, help="alternatives per move scored in full, by isolated exposure")
    parser.add_argument("--random", type=int, default=2, help="extra random alternatives per move scored in full")
    parser.add_argument("--pieces", type=int, default=0, help="debug: only the first N pieces per sweep (0 = all)")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--device", default=E.DEVICE)
    parser.add_argument("--isolated-samples", type=int, default=200)
    parser.add_argument("--isolated-directions", type=int, default=32)
    parser.add_argument("--samples", type=int, default=E.DEFAULTS["samples_per_object"])
    parser.add_argument("--directions", type=int, default=E.DEFAULTS["directions"])
    parser.add_argument("--out", type=Path, default=OUT)
    parser.add_argument("--insertion-gate", action="store_true", help="accept a move only if a vertical-lowering insertion order exists")
    args = parser.parse_args(argv)
    t0 = time.time()
    args.out.mkdir(parents=True, exist_ok=True)
    info = register_hotec(args.assets)
    for name, i in info.items():
        print(f"[INFO] {name}: food-contact {i['food_contact_triangles']} faces, {i['food_contact_area_m2']*1e4:.1f} cm2, "
              f"rim z {i['rim_z_m']*1000:.1f} mm ({i['rim_vertices']} vertices)", flush=True)
    assert E.pools("hotec_bowl", (0, 0, .3), (0., 0., 0., 1.)) is True and E.pools("hotec_bowl", (0, 0, .3), (1., 0., 0., 0.)) is False
    fam, parts, points = candidate_families(args.assets)
    print("[INFO] candidate family sizes", {k: len(v) for k, v in fam.items()}, flush=True)
    template = json.loads(args.start_layout.read_text())
    entries = load_entries(template)
    isolated = IsolatedExposure(args.device, args.isolated_samples, args.isolated_directions)
    rng = np.random.default_rng(args.seed)

    def world_factory():
        w = CollisionWorld(ASSET_DIR)
        for kind in KINDS:
            w.parts[kind], w.points[kind] = parts[kind], points[kind]
        return w

    gate_cache = {}
    if args.insertion_gate:
        order0, edges0, rack0 = insertion_order(entries, parts, points, gate_cache)
        if order0 is None:
            raise SystemExit(f"[RESULT] FAIL the start layout has no vertical insertion order (rack hits {rack0}, edges {edges0})")
        print(f"[INFO] start layout insertion order OK: {len(edges0)} dependencies", flush=True)
    current = full_score(entries, "start", args.device, args.samples, args.directions)
    start_score = current["score"]
    print(f"[INFO] start ({args.start_layout}): S {start_score:.4f} worst {current['worst']:.4f} pooling {current['pooling_count']}", flush=True)
    history, moves = [{"sweep": 0, "piece": None, "score": start_score}], 0
    for sweep in range(1, args.sweeps + 1):
        improved = False
        order = list(range(len(entries)))
        if args.pieces:
            order = order[:args.pieces]
        for i in order:
            alts = free_alternatives(i, entries, fam, parts, points, world_factory)
            if not alts:
                continue
            iso = isolated.evaluate([c for c, _ in alts])
            ranked = list(np.argsort(-iso)[:args.top])
            extra = [int(j) for j in rng.choice(len(alts), size=min(args.random, len(alts)), replace=False) if j not in ranked]
            best_local, best_entry = current["score"], None
            for j in ranked + extra:
                c, _ = alts[j]
                trial = list(entries)
                trial[i] = {**entries[i], "rack": c["rack"], "slot": c["slot"], "variant": c["variant"],
                            "position": [float(v) for v in c["position"]], "quaternion_xyzw": [float(v) for v in c["quaternion_xyzw"]]}
                res = full_score(trial, f"s{sweep}_p{i}_a{j}", args.device, args.samples, args.directions)
                if args.insertion_gate and res["feasible"] and res["score"] > best_local + 1e-4:
                    if insertion_order(trial, parts, points, gate_cache)[0] is None:
                        continue
                if res["feasible"] and res["score"] > best_local + 1e-4:
                    best_local, best_entry, best_res = res["score"], trial[i], res
            if best_entry is not None:
                entries[i], current, improved, moves = best_entry, best_res, True, moves + 1
                print(f"[INFO] sweep {sweep} piece {entries[i]['id']}: S {history[-1]['score']:.4f} -> {current['score']:.4f} "
                      f"({entries[i]['rack']} {entries[i]['slot']} {entries[i]['variant'][:24]}; {len(alts)} alternatives)", flush=True)
                history.append({"sweep": sweep, "piece": entries[i]["id"], "score": current["score"], "slot": entries[i]["slot"], "variant": entries[i]["variant"]})
        print(f"[INFO] sweep {sweep} done: S {current['score']:.4f}, {moves} moves so far, isolated cache {len(isolated.cache)}, {time.time() - t0:.0f} s", flush=True)
        if not improved:
            break
    best_layout = layout_from(entries, template, "coordinate_ascent", current["score"])
    order, edges, rack_hits = insertion_order(entries, parts, points, gate_cache if args.insertion_gate else None)
    best_layout["insertion_order"] = order
    best_layout["insertion_dependencies"] = edges
    best_layout["insertion_rack_hits"] = rack_hits
    best_layout["placement_instructions"] = ([f"{k + 1}. {next(e for e in entries if e['id'] == oid)['id']}: {describe_placement(next(e for e in entries if e['id'] == oid))}"
                                             for k, oid in enumerate(order)] if order else None)
    (args.out / "best_layout.json").write_text(json.dumps(best_layout, indent=1) + "\n")
    record = {"schema_version": 1, "method": "coordinate ascent over the planner's realistic candidate families; "
                                             "alternatives ranked by cached isolated exposure, best few plus random scored in full",
              "scorer": {**E.DEFAULTS, "samples_per_object": args.samples, "directions": args.directions,
                         "isolated_samples": args.isolated_samples, "isolated_directions": args.isolated_directions},
              "hotec_kinds": info, "assets": str(args.assets), "start_layout": str(args.start_layout), "seed": args.seed,
              "families": {k: len(v) for k, v in fam.items()}, "sweeps_run": history[-1]["sweep"], "moves": moves,
              "insertion_gate": bool(args.insertion_gate), "insertion_order": order, "insertion_dependencies": edges, "insertion_rack_hits": rack_hits,
              "start_score": start_score, "best_score": current["score"], "best_worst": current["worst"],
              "best_objects": current["objects"], "history": history, "elapsed_s": time.time() - t0}
    (args.out / "search.json").write_text(json.dumps(record, indent=1) + "\n")
    print(f"[RESULT] PASS S {current['score']:.4f} (start {args.start_layout.parent.name} {start_score:.4f}, {moves} moves, "
          f"{history[-1]['sweep']} sweeps) {time.time() - t0:.0f} s -> {args.out}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
