# Copyright (c) 2026, dishsim project.
# SPDX-License-Identifier: BSD-3-Clause
"""Minimal ambient-occlusion (AO) exposure scorer for Frigidaire arrangements.

Definition (settled 2026-09-17, revision 2): for every sample on a FOOD-CONTACT
surface (the lathed inner wall of a bowl / mug, the top face of a plate) cast a
fixed Fibonacci set of 64 rays toward the LOWER half-space of the world frame,
where the spray arms sit; exposure = share of rays that hit NOTHING in the
load-only occluder set (every dish including the object itself, both racks, the
silverware basket). Tub, door and cabinet never occlude. Per object:
area-weighted mean exposure. Arrangement: area-weighted mean over objects
(primary) and the worst object (secondary). No threshold.

``source="hemisphere"`` keeps revision 1 (uniform hemisphere around the surface
normal), which prefers mouth-up vessels; the mouth-up count from the organized
policy is reported beside every score either way. No water model. Kit-free:
numpy at import, warp for the ray queries.
"""
from __future__ import annotations
from dataclasses import dataclass
from pathlib import Path
import hashlib
import json
import math

import numpy as np

from .asset import BODY_POSITIONS
from .paths import REPO_ROOT
from .random_poses import compose_pose, quaternion_matrix_xyzw, relative_pose

try:
    import warp as wp
    wp.config.kernel_cache_dir = str(REPO_ROOT / "outputs" / "warp_cache")
    wp.config.quiet = True
except ImportError:  # pragma: no cover - warp ships with Isaac's python
    wp = None

DEFAULTS = {"schema_version": 4, "directions": 64, "samples_per_object": 500,
            "max_distance_m": 2.0, "origin_offset_m": 1e-4, "source": "per-rack", "ceiling_weight": 0.,
            "occluders": "load only: every dish (self included), LowerRack, UpperRack, SilverwareBasket",
            "definition": "load-induced occlusion of food-contact surfaces for water from source points: "
                          "a disc under each rack (the spray arm's sweep) and an optional ceiling point "
                          "above the upper rack; rays weighted by impingement cosine; no water model; "
                          "not measured cleaning"}
# Spray-arm sweeps as source discs (world frame, metres). Heights and radii come from the
# asset (tub floor 0.152, lower rack floor wires 0.215, upper rack floor wires ~0.572) and
# the rack widths; they are ASSUMPTIONS recorded in every score, not measurements.
ARM_SOURCES = {"lower_arm": {"center": (0., .008, .185), "radius": .245, "racks": ("LowerRack", "SilverwareBasket")},
               "middle_arm": {"center": (0., .008, .540), "radius": .205, "racks": ("UpperRack",)}}
CEILING_POINT = (0., .018, .817)     # upper spray nozzle, assumed at the tub ceiling centre
POOL_TOLERANCE_M = .002
# Direction-set source per rack under the legacy source="per-rack-directions" (revision 3).
RACK_SOURCE = {"UpperRack": "below+above"}
RING = 96  # tableware._SECTORS (24) * 4 vertices per lathe ring
# Lathe section counts per kind (tableware.tableware_geometry); the first section
# of every supported kind sits on the axis, so the mesh is apex, n-1 outer rings,
# n-1 inner rings, inner apex. Inner = food contact (vessel interior, plate top).
SECTIONS = {"bowl": 6, "mug": 5, "tumbler": 5, "dinner_plate": 5, "salad_plate": 5, "saucer": 5}
IDENTITY = (0., 0., 0., 1.)


@dataclass
class FoodContact:
    kind: str
    triangles: np.ndarray   # (T, 3, 3) object frame
    normals: np.ndarray     # (T, 3) outward (into the cavity / above the plate)
    areas: np.ndarray       # (T,)
    area_m2: float
    rim: np.ndarray         # (96, 3) the mouth ring (first inner lathe ring), object frame
    inner_points: np.ndarray  # every inner-surface vertex, object frame


@dataclass
class Samples:
    points: np.ndarray      # (N, 3)
    normals: np.ndarray     # (N, 3)
    weights: np.ndarray     # (N,) sums to the food-contact area
    face: np.ndarray        # (N,) index into FoodContact.triangles


_FOOD_CONTACT: dict[str, FoodContact] = {}
_VISUALS: dict[str, list[np.ndarray]] = {}


def dish_visuals(kind):
    """Every visual triangle soup of a kind (mug handle included), object frame."""
    if kind not in _VISUALS:
        from .tableware import tableware_geometry
        _VISUALS[kind] = [np.asarray(p)[np.asarray(f)] for p, _, f in tableware_geometry(kind)["visuals"]]
    return _VISUALS[kind]


def food_contact(kind):
    if kind in _FOOD_CONTACT:
        return _FOOD_CONTACT[kind]
    if kind not in SECTIONS:
        raise NotImplementedError(f"No food-contact rule for {kind!r} in this minimal version")
    from .tableware import tableware_geometry
    points, _, faces = tableware_geometry(kind)["visuals"][0]
    points, faces = np.asarray(points), np.asarray(faces)
    n = SECTIONS[kind]
    expected = 2 + 2 * (n - 1) * RING
    if len(points) != expected:
        raise ValueError(f"{kind}: {len(points)} vertices, expected the lathe layout with {expected}")
    start = 1 + (n - 1) * RING           # first inner-ring vertex
    inner = np.all(faces >= start, axis=1)  # rim band (mixed rings) excluded
    tri = points[faces[inner]]
    cross = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
    areas = 0.5 * np.linalg.norm(cross, axis=1)
    normals = cross / (2. * areas[:, None])
    _FOOD_CONTACT[kind] = FoodContact(kind, tri, normals, areas, float(areas.sum()),
                                      points[start:start + RING].copy(), points[start:].copy())
    return _FOOD_CONTACT[kind]


def surface_samples(fc, n):
    """Systematic area sampling on the face-area CDF; face centroids, no RNG."""
    cdf = np.cumsum(fc.areas)
    cdf /= cdf[-1]
    face = np.minimum(np.searchsorted(cdf, (np.arange(n) + .5) / n), len(fc.areas) - 1)
    return Samples(fc.triangles[face].mean(axis=1), fc.normals[face], np.full(n, fc.area_m2 / n), face)


def hemisphere_directions(m):
    """Deterministic Fibonacci lattice on the +Z hemisphere."""
    k = np.arange(m)
    z = (k + .5) / m
    r = np.sqrt(1. - z * z)
    phi = k * math.pi * (3. - math.sqrt(5.))
    return np.column_stack((r * np.cos(phi), r * np.sin(phi), z))


def lower_directions(m):
    """Deterministic Fibonacci lattice on the world -Z hemisphere (spray-arm side)."""
    d = hemisphere_directions(m)
    d[:, 2] *= -1.
    return d


def source_for(rack, source=DEFAULTS["source"]):
    """Resolve the ray-source name for an object in ``rack``."""
    if source == "per-rack":
        return next(name for name, spec in ARM_SOURCES.items() if rack in spec["racks"])
    if source == "per-rack-directions":
        return RACK_SOURCE.get(rack, "below")
    return source


def disk_points(center, radius, k):
    """Equal-area Fibonacci disk (same construction as organization.opening_rays)."""
    i = np.arange(k)
    r = radius * np.sqrt((i + .5) / k)
    a = i * math.pi * (3. - math.sqrt(5.))
    return np.column_stack((r * np.cos(a), r * np.sin(a), np.zeros(k))) + np.asarray(center, dtype=float)


def rack_sources(rack, k=DEFAULTS["directions"], ceiling_weight=DEFAULTS["ceiling_weight"]):
    """Source points and weights (sum 1) for an object in ``rack``; returns (points, weights, arm)."""
    arm = source_for(rack, "per-rack")
    spec = ARM_SOURCES[arm]
    points = disk_points(spec["center"], spec["radius"], k)
    if rack == "UpperRack" and ceiling_weight > 0:
        weights = np.append(np.full(k, (1. - ceiling_weight) / k), ceiling_weight)
        points = np.vstack((points, np.asarray(CEILING_POINT)))
    else:
        weights = np.full(k, 1. / k)
    return points, weights, arm


def pools(kind, position_m, quaternion_xyzw, tolerance=POOL_TOLERANCE_M):
    """True when the vessel cannot drain: some interior point sits below the rim's lowest point."""
    if kind not in ("bowl", "mug", "tumbler"):
        return False
    fc = food_contact(kind)
    rot = quaternion_matrix_xyzw(quaternion_xyzw)
    z = float(np.asarray(position_m, dtype=float)[2])
    rim_z = (fc.rim @ rot.T)[:, 2] + z
    inner_z = (fc.inner_points @ rot.T)[:, 2] + z
    return bool(inner_z.min() < rim_z.min() - tolerance)


def source_directions(m, source):
    """World-frame direction set for a resolved source name."""
    if source == "below":
        return lower_directions(m)
    if source == "above":
        return hemisphere_directions(m)
    if source == "below+above":
        return np.concatenate((lower_directions(m), hemisphere_directions(m)))
    raise ValueError(f"unknown ray source {source!r}")


def tangent_frames(normals):
    a = np.where(np.abs(normals[:, :1]) < .9, [[1., 0., 0.]], [[0., 1., 0.]])
    t = np.cross(normals, a)
    t /= np.linalg.norm(t, axis=1, keepdims=True)
    b = np.cross(normals, t)
    return np.stack((t, b, normals), axis=1)  # (N, 3, 3) rows = local x, y, z


def sample_rays(points, normals, m, offset, source="below"):
    """Rays for every sample; ``source`` must already be resolved (not "per-rack")."""
    if source == "hemisphere":
        dirs = np.einsum("mk,nkj->nmj", hemisphere_directions(m), tangent_frames(normals)).reshape(-1, 3)
    else:
        dirs = np.tile(source_directions(m, source), (len(points), 1))
    rays_per_sample = len(dirs) // max(len(points), 1)
    origins = np.repeat(points + offset * normals, rays_per_sample, axis=0)
    return origins, dirs


def posed(triangles, position, quaternion_xyzw):
    rot = quaternion_matrix_xyzw(quaternion_xyzw)
    return triangles @ rot.T + np.asarray(position, dtype=float)


@dataclass
class Arrangement:
    name: str
    path: str
    sha256: str
    objects: list          # dicts: id, kind, rack, position_m, quaternion_xyzw (racks IN)
    basket: tuple          # (position_m, quaternion_xyzw) racks IN


def load_state(path):
    """Racks-in poses from a settled initial-state JSON (stored with racks OUT)."""
    path = Path(path)
    raw = path.read_bytes()
    data = json.loads(raw)
    snap = data["initial_snapshot"]["poses"]
    objects = []
    for o in data["objects"]:
        rack, local, world, frame = o["rack"], o["rack_local_pose"], o["pose_world"], snap[o["rack"]]
        check, _ = relative_pose(world["position_m"], world["quaternion_xyzw"],
                                 frame["position_m"], frame["quaternion_xyzw"])
        if np.linalg.norm(check - np.asarray(local["position_m"])) > 1e-5:
            raise ValueError(f"{path.name}: rack_local_pose of {o['object_id']} disagrees with pose_world")
        p, q = compose_pose(BODY_POSITIONS[rack], IDENTITY, local["position_m"], local["quaternion_xyzw"])
        objects.append({"id": o["object_id"], "kind": o["kind"], "rack": rack,
                        "position_m": p, "quaternion_xyzw": q})
    lower, basket = snap["LowerRack"], snap["SilverwareBasket"]
    bp, bq = relative_pose(basket["position_m"], basket["quaternion_xyzw"], lower["position_m"], lower["quaternion_xyzw"])
    return Arrangement(path.stem, str(path), hashlib.sha256(raw).hexdigest(), objects,
                       compose_pose(BODY_POSITIONS["LowerRack"], IDENTITY, bp, bq))


_COMPONENTS = None


def component_triangles(name):
    """Component-frame wire triangles of a rack or the basket (solids skipped)."""
    global _COMPONENTS
    if _COMPONENTS is None:
        from .geometry import build_components
        _COMPONENTS = build_components()
    meshes = _COMPONENTS[name]["meshes"].values()
    return np.concatenate([np.asarray(m.points)[np.asarray(m.faces)] for m in meshes])


def rack_wires(name):
    """Simplified wire centrelines for drawing (component frame)."""
    global _COMPONENTS
    if _COMPONENTS is None:
        from .geometry import build_components
        _COMPONENTS = build_components()
    return [np.asarray(path) for _, path, _ in _COMPONENTS[name]["wires"]]


def occluder_soup(arrangement, appliance=True):
    parts = []
    if appliance:
        for name in ("LowerRack", "UpperRack"):
            parts.append(component_triangles(name) + np.asarray(BODY_POSITIONS[name]))
        parts.append(posed(component_triangles("SilverwareBasket"), *arrangement.basket))
    for obj in arrangement.objects:
        for tri in dish_visuals(obj["kind"]):
            parts.append(posed(tri, obj["position_m"], obj["quaternion_xyzw"]))
    return np.concatenate(parts).astype(np.float32)


if wp is not None:
    @wp.kernel
    def _hit_kernel(mesh: wp.uint64, origins: wp.array(dtype=wp.vec3), dirs: wp.array(dtype=wp.vec3),
                    max_t: wp.array(dtype=float), hit: wp.array(dtype=wp.int32), t_hit: wp.array(dtype=float)):
        i = wp.tid()
        q = wp.mesh_query_ray(mesh, origins[i], dirs[i], max_t[i])
        if q.result:
            hit[i] = 1
            t_hit[i] = q.t
        else:
            t_hit[i] = max_t[i]


def cast(soup, origins, dirs, device="cpu", max_t=DEFAULTS["max_distance_m"]):
    """(hit, t): hit is True where the ray hits any soup triangle within max_t (scalar or per ray);
    t is the hit distance, or max_t when nothing was hit."""
    if wp is None:
        raise RuntimeError("warp is required for the ray queries")
    max_t = np.broadcast_to(np.asarray(max_t, dtype=np.float32), (len(origins),)).astype(np.float32)
    if not len(soup) or not len(origins):
        return np.zeros(len(origins), dtype=bool), max_t.copy()
    with wp.ScopedDevice(device):
        mesh = wp.Mesh(points=wp.array(soup.reshape(-1, 3), dtype=wp.vec3),
                       indices=wp.array(np.arange(soup.size // 3, dtype=np.int32), dtype=wp.int32))
        o = wp.array(np.ascontiguousarray(origins, dtype=np.float32), dtype=wp.vec3)
        d = wp.array(np.ascontiguousarray(dirs, dtype=np.float32), dtype=wp.vec3)
        m = wp.array(np.ascontiguousarray(max_t), dtype=float)
        hit = wp.zeros(len(origins), dtype=wp.int32)
        t = wp.zeros(len(origins), dtype=float)
        wp.launch(_hit_kernel, dim=len(origins), inputs=[mesh.id, o, d, m, hit, t])
        return hit.numpy().astype(bool), t.numpy()


def exposure_of(points, normals, soup, device="cpu", directions=DEFAULTS["directions"],
                offset=DEFAULTS["origin_offset_m"], source="below", sources=None):
    """Per-sample exposure in [0, 1].

    With ``sources=(points, weights)``: rays run from each sample to each source point;
    each ray is weighted by the source weight times max(cos(angle to the normal), 0), and
    exposure = weighted share of unobstructed rays (0 when no source faces the sample).
    Without ``sources``: legacy direction-set modes, unweighted share of open rays.
    """
    if not len(points):
        return np.zeros(0)
    if sources is None:
        origins, dirs = sample_rays(points, normals, directions, offset, source)
        hits = cast(soup, origins, dirs, device)[0].reshape(len(points), -1)
        return 1. - hits.mean(axis=1)
    q, u = sources
    starts = points + offset * normals
    d = q[None, :, :] - starts[:, None, :]                       # (N, M, 3)
    dist = np.linalg.norm(d, axis=2)                              # (N, M)
    dirs = d / dist[..., None]
    cosine = np.maximum(np.einsum("nj,nmj->nm", normals, dirs), 0.)
    hit, _ = cast(soup, np.repeat(starts, len(q), axis=0), dirs.reshape(-1, 3), device, dist.reshape(-1))
    open_ = ~hit.reshape(len(points), len(q))
    w = cosine * np.asarray(u)[None, :]
    denom = w.sum(axis=1)
    return np.where(denom > 0, (w * open_).sum(axis=1) / np.where(denom > 0, denom, 1.), 0.)


def isolated_baseline(kind, position_m, quaternion_xyzw, device="cpu",
                      samples=DEFAULTS["samples_per_object"], directions=DEFAULTS["directions"],
                      source="below", sources=None):
    """Mean exposure of the object alone at its pose (self-occlusion only)."""
    fc = food_contact(kind)
    s = surface_samples(fc, samples)
    rot = quaternion_matrix_xyzw(quaternion_xyzw)
    soup = np.concatenate([posed(t, position_m, quaternion_xyzw) for t in dish_visuals(kind)]).astype(np.float32)
    return float(exposure_of(s.points @ rot.T + np.asarray(position_m), s.normals @ rot.T,
                             soup, device, directions, source=source, sources=sources).mean())


def mouth_up(kind, rack, quaternion_xyzw):
    """Organized-policy orientation gate for bowls and mugs; None when not applicable."""
    if kind not in ("bowl", "mug"):
        return None
    from .organization import orientation_metrics
    return not orientation_metrics(kind, rack, {"quaternion_xyzw": list(map(float, quaternion_xyzw))})["valid"]


def score_arrangement(arrangement, device="cpu", samples=DEFAULTS["samples_per_object"],
                      directions=DEFAULTS["directions"], appliance=True, source=DEFAULTS["source"],
                      ceiling_weight=DEFAULTS["ceiling_weight"]):
    soup = occluder_soup(arrangement, appliance)
    points, normals, owner, records = [], [], [], []
    for i, obj in enumerate(arrangement.objects):
        fc = food_contact(obj["kind"])
        s = surface_samples(fc, samples)
        rot = quaternion_matrix_xyzw(obj["quaternion_xyzw"])
        points.append(s.points @ rot.T + np.asarray(obj["position_m"]))
        normals.append(s.normals @ rot.T)
        owner.append(np.full(samples, i))
        records.append({"id": obj["id"], "kind": obj["kind"], "rack": obj["rack"], "area_m2": fc.area_m2,
                        "ray_source": source_for(obj["rack"], source),
                        "mouth_up": mouth_up(obj["kind"], obj["rack"], obj["quaternion_xyzw"]),
                        "pools": pools(obj["kind"], obj["position_m"], obj["quaternion_xyzw"]),
                        "position_m": list(map(float, obj["position_m"])),
                        "quaternion_xyzw": list(map(float, obj["quaternion_xyzw"]))})
    points, normals, owner = np.concatenate(points), np.concatenate(normals), np.concatenate(owner)
    per_sample = np.zeros(len(points))
    point_sources = {}
    for src in sorted({r["ray_source"] for r in records}):     # one cast per distinct source
        idx = [i for i, r in enumerate(records) if r["ray_source"] == src]
        mask = np.isin(owner, idx)
        if source == "per-rack":
            q, u, _ = rack_sources(records[idx[0]]["rack"], directions, ceiling_weight)
            point_sources[src] = (q, u)
            per_sample[mask] = exposure_of(points[mask], normals[mask], soup, device, directions, sources=(q, u))
        else:
            per_sample[mask] = exposure_of(points[mask], normals[mask], soup, device, directions, source=src)
    for i, rec in enumerate(records):
        rec["exposure"] = float(per_sample[owner == i].mean())
        rec["baseline"] = isolated_baseline(rec["kind"], rec["position_m"], rec["quaternion_xyzw"],
                                            device, samples, directions, rec["ray_source"],
                                            point_sources.get(rec["ray_source"]))
        rec["relative"] = rec["exposure"] / rec["baseline"] if rec["baseline"] > 0 else None
    areas = np.array([r["area_m2"] for r in records])
    exposures = np.array([r["exposure"] for r in records])
    violations = [r["id"] for r in records if r["pools"]]
    return {"name": arrangement.name, "source": arrangement.path, "source_sha256": arrangement.sha256,
            "n_objects": len(records), "score": float((areas * exposures).sum() / areas.sum()),
            "worst": float(exposures.min()), "mouth_up_count": int(sum(bool(r["mouth_up"]) for r in records)),
            "pooling_count": len(violations), "feasible": not violations, "violations": violations,
            "parameters": {**DEFAULTS, "directions": directions, "samples_per_object": samples,
                           "source": source, "ceiling_weight": ceiling_weight,
                           "arm_sources": ARM_SOURCES if source == "per-rack" else None,
                           "ceiling_point": CEILING_POINT if source == "per-rack" else None,
                           "rack_direction_sets": {**RACK_SOURCE, "other": "below"} if source == "per-rack-directions" else None,
                           "pool_tolerance_m": POOL_TOLERANCE_M,
                           "device": device, "appliance_occluders": appliance},
            "objects": records,
            "samples": {"points": points, "normals": normals, "owner": owner, "exposure": per_sample},
            "triangles": int(len(soup))}


def score_state(path, **kwargs):
    return score_arrangement(load_state(path), **kwargs)


def sanity_pair(device="cpu", samples=DEFAULTS["samples_per_object"], directions=DEFAULTS["directions"],
                gap_m=.015, source=DEFAULTS["source"]):
    """A bowl mouth-DOWN on the lower rack floor, alone versus with a dinner plate below its rim.

    The bowl is flipped 180 deg about X so its rim sits at the lower rack floor height
    (z = 0.215) and its base at z = 0.28; the plate's top face sits gap_m below the rim,
    between the bowl and the lower arm (z = 0.185).
    """
    flipped = (1., 0., 0., 0.)
    rim_z = BODY_POSITIONS["LowerRack"][2]
    bowl = {"id": "bowl", "kind": "bowl", "rack": "LowerRack", "position_m": np.array([0., .008, rim_z + .065]),
            "quaternion_xyzw": flipped}
    plate = {"id": "plate", "kind": "dinner_plate", "rack": "LowerRack",
             "position_m": np.array([0., .008, rim_z - gap_m - .010]), "quaternion_xyzw": IDENTITY}
    out = {}
    for label, objects in (("alone", [bowl]), ("covered", [bowl, plate])):
        arr = Arrangement(label, "", "", objects, (np.zeros(3), IDENTITY))
        result = score_arrangement(arr, device, samples, directions, appliance=False, source=source)
        out[label] = {"exposure": result["objects"][0]["exposure"], "samples": result["samples"],
                      "objects": objects}
    return out


def strip_samples(result):
    """JSON-serialisable copy of a score record."""
    return {k: v for k, v in result.items() if k != "samples"}
