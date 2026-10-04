# Copyright (c) 2026, dishsim project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""HOTEC wheat-straw dinnerware (plate, bowl, cup): parametric lathe -> Isaac USD, Kit-free.

Every shape is a 2-D cross-section built from NAMED physical parameters (rim diameter, height,
wall, floor, foot, bulge, grooves, flutes), each tagged ``nominal`` (listing target),
``estimated`` (modelling estimate, documented) or ``measured`` (user override). The sparse
control rows are revolved into convex collision pieces; a dense PCHIP curve through the SAME
rows is revolved into the visual shell, so the collider never sits outside the visual by more
than the decorative feature depth. Metres, Z up, origin at the base centre (foot plane z = 0).

No mass is authored anywhere (no ``PhysicsMassAPI``, no ``physics:density``); the user
measures the pieces. Capacity is the exact volume of revolution of the inner profile up to the
lowest rim point (a temporary horizontal closure that is never written into the asset).

The lathe primitives ``_hull``/``_normals``/``_closed_mesh``/``_lathe`` are copied verbatim
from ``code/frigidaire/src/dishsim_frigidaire/tableware.py:49-136`` and the USD helpers
``_attr``/``_bind``/``_collision``/``_mesh`` from ``code/frigidaire/src/dishsim_frigidaire/asset.py``
(lines 41-42, 84-97, 117-130); this tracked module must never import that package.
``pxr`` and ``matplotlib`` are imported function-locally (architecture layering rule).

    code/scripts/run_py.sh -m dishsim.hotec_gen --out data/assets/models/hotec_wheatstraw/v1
    code/scripts/run_py.sh -m dishsim.hotec_gen --out .../v2 --set cup.base_diameter_m=0.060
"""
from __future__ import annotations

import argparse
import copy
import hashlib
import json
import math
from pathlib import Path

import numpy as np

KINDS = ("plate", "bowl", "cup")
ROOT_PRIMS = {"plate": "HotecPlate", "bowl": "HotecBowl", "cup": "HotecCup"}
_SECTORS = 24                     # collision sectors (tableware.py:49); visual ring = 4 x sectors
RING = 4 * _SECTORS
SAMPLES_PER_INTERVAL = 6          # dense visual rows between consecutive control rows
HEADSPACE_M = .005
US_FLOZ_ML = 29.5735295625
ADVERTISED_ML = {"bowl": 26 * US_FLOZ_ML, "cup": 12 * US_FLOZ_ML}   # 769.0 / 354.9 mL
RIM_FLAT_INSET_M = .0004          # the rim row keeps a flat >= 0.6 mm so the lip band is never degenerate
COLOURS = {"blue_grey": (.42, .50, .56), "teal": (.18, .55, .52),
           "coral": (.86, .42, .30), "mustard": (.82, .64, .20)}     # disposable appearance
DEFAULT_COLOUR = "blue_grey"
CONTACT = {"static_friction": .45, "dynamic_friction": .35, "restitution": 0.}   # estimated; no density
PHYSX = {"solverPositionIterationCount": 16, "solverVelocityIterationCount": 4,
         "enableCCD": True, "maxDepenetrationVelocity": .5}
MASS_STATUS = ("not authored: no PhysicsMassAPI and no physics:density on any prim; "
               "the user measures the pieces")
MASS_PROVENANCE = ("average of 8 pieces on a kitchen scale, 2026-09-22: 8 bowls 539 g, 8 cups 476 g, 8 plates 757 g")


def mass_status(table, kind):
    """Mass provenance string for one kind: MASS_STATUS until ``mass_kg`` is set, then the authored value."""
    mass = table[kind]["mass_kg"]["value"]
    if mass is None:
        return MASS_STATUS
    return (f"measured: physics:mass = {mass:.6g} kg on the rigid-body prim ({MASS_PROVENANCE}); "
            "no density, centre of mass or inertia authored (PhysX derives them from the collision shells)")
DIMENSION_STATUS = "nominal listing targets plus documented estimates; not measured"
ORIGIN_STATUS = {"base": "base centre: z = 0 at the foot plane, +Z up, opening toward +Z",
                 "centre": "centre of the outer bounds, +Z up, opening toward +Z"}


def _p(value, status, note):
    return {"value": value, "status": status, "note": note}


PARAMETERS = {
    "plate": {
        "rim_diameter_m": _p(.2286, "nominal", "listing: 9 in maximum outer diameter"),
        "height_m": _p(.0400, "nominal", "listing: 1.6 in overall height, foot plane to rim top"),
        "wall_m": _p(.0035, "estimated", "wheat-straw PP mouldings run 2.5-4 mm"),
        "floor_m": _p(.0040, "estimated", "floor slightly thicker than the wall"),
        "foot_diameter_m": _p(.140, "estimated", "about 61 % of the rim; a coupe plate carries a wide foot ring"),
        "foot_width_m": _p(.006, "estimated", "moulded ring width"),
        "foot_height_m": _p(.004, "estimated", "recess depth under the base centre"),
        "side_bulge_m": _p(.006, "estimated", "sagitta of the convex underside arc foot -> rim (0 = straight cone); "
                                              "the flat eating well is DERIVED: where the wall offset rises above the floor"),
        "side_rows": _p(2, "estimated", "collision rows along the side arc (2 -> 8 bands, 192 pieces): measured "
                                        "cavity intrusion 0.59 mm; 3 rows (216 pieces) would give 0.42 mm"),
        "grooves": _p({"heights_m": [], "depth_m": .0006, "half_width_m": .00125}, "estimated",
                      "no decorative grooves seen on the plate"),
        "mass_kg": _p(None, "nominal", "not authored until measured: --set kind.mass_kg=<kg> authors physics:mass on the rigid-body prim"),
    },
    "bowl": {
        "rim_diameter_m": _p(.148, "nominal", "listing: 5.8 in maximum diameter"),
        "height_m": _p(.075, "nominal", "listing: 2.95 in overall height"),
        "wall_m": _p(.0035, "estimated", "as the plate"),
        "floor_m": _p(.0040, "estimated", ""),
        "foot_diameter_m": _p(.070, "estimated", "about 47 % of the rim, middle of the photo's 45-50 %"),
        "foot_width_m": _p(.005, "estimated", ""),
        "foot_height_m": _p(.004, "estimated", "distinct foot ring in the photo"),
        "side_bulge_m": _p(.008, "estimated", "rounded flank (NOT a hemisphere): sagitta of the side arc; "
                                              "the dominant capacity uncertainty"),
        "side_rows": _p(2, "estimated", "collision rows along the side arc (2 -> 8 bands, 192 pieces): measured "
                                        "cavity intrusion 0.85 mm; 3 rows (216 pieces) would give 0.54 mm"),
        "grooves": _p({"heights_m": [.054, .060, .066], "depth_m": .0006, "half_width_m": .00125}, "estimated",
                      "three horizontal grooves on the upper body, 6 mm pitch; visual only, collision smooth"),
        "mass_kg": _p(None, "nominal", "not authored until measured: --set kind.mass_kg=<kg> authors physics:mass on the rigid-body prim"),
    },
    "cup": {
        "rim_diameter_m": _p(.0711, "nominal", "listing: 2.8 in maximum width, PROVISIONAL "
                                               "(the listing mixes the 20-pack fluted cup and the 24-pack ringed cup)"),
        "height_m": _p(.1049, "nominal", "listing: 4.13 in, PROVISIONAL"),
        "wall_m": _p(.0025, "estimated", "cups are moulded thinner than plates"),
        "floor_m": _p(.0040, "estimated", ""),
        "base_diameter_m": _p(.056, "estimated", "about 79 % of the rim: the mild taper seen in the photos"),
        "foot_width_m": _p(.004, "estimated", "base ring width"),
        "foot_height_m": _p(.002, "estimated", "shallow recess under the base"),
        "flutes": _p({"count": 12, "shape": "facet", "depth_m": .0006, "fade_z_m": [.012, .020, .090, .098]},
                     "estimated", "vertical facets of the original cup; count and depth to be measured; "
                                  "count must divide the 96-vertex ring; outer surface only"),
        "rings": _p({"heights_m": [], "depth_m": .0006, "half_width_m": .00125}, "estimated",
                    "the ringed-cup alternative: set heights_m to e.g. [0.030, 0.050, 0.070] and flutes.count to 0"),
        "mass_kg": _p(None, "nominal", "not authored until measured: --set kind.mass_kg=<kg> authors physics:mass on the rigid-body prim"),
    },
}
STATUSES = ("nominal", "estimated", "measured")


# --------------------------------------------------------------------------- parameters

def resolve(overrides=None):
    """Deep copy of PARAMETERS with ``{"kind.name[.sub]": value}`` overrides applied as measured."""
    table = copy.deepcopy(PARAMETERS)
    for path, value in (overrides or {}).items():
        kind, name, *sub = path.split(".")
        entry = table[kind][name]
        if sub:
            entry["value"][sub[0]] = value
        else:
            entry["value"] = value
        entry["status"] = "measured"
        entry["note"] = f"user override: {path}={value!r}"
    return table


def parameters_sha256(table):
    return hashlib.sha256(json.dumps(table, sort_keys=True, separators=(",", ":")).encode()).hexdigest()


def values(table, kind):
    return {name: entry["value"] for name, entry in table[kind].items()}


# --------------------------------------------------------------------------- vendored lathe (tableware.py:49-136)

def _hull(points):
    """Small exact convex pieces; no approximate decomposition or cavity filling."""
    from scipy.spatial import ConvexHull
    p = np.unique(np.round(np.asarray(points, dtype=float), 12), axis=0)
    hull = ConvexHull(p)
    faces = hull.simplices.copy()
    for face, equation in zip(faces, hull.equations):
        a, b, c = p[face]
        if np.dot(np.cross(b-a, c-a), equation[:3]) < 0:
            face[1], face[2] = face[2], face[1]
    return p, faces


def _normals(points, faces):
    p, f = np.asarray(points), np.asarray(faces, dtype=int)
    normals = np.zeros_like(p, dtype=float)
    area = np.cross(p[f[:, 1]]-p[f[:, 0]], p[f[:, 2]]-p[f[:, 0]])
    for index in range(3):
        np.add.at(normals, f[:, index], area)
    lengths = np.linalg.norm(normals, axis=1)
    if (lengths < 1e-15).any():
        raise ValueError("A tableware mesh contains an unused or degenerate vertex")
    return normals / lengths[:, None]


def _closed_mesh(points, faces):
    p, f = np.asarray(points, dtype=float), np.asarray(faces, dtype=int)
    area = np.cross(p[f[:, 1]]-p[f[:, 0]], p[f[:, 2]]-p[f[:, 0]])
    keep = np.linalg.norm(area, axis=1) > 1e-14
    f = f[keep]
    volume = np.sum(np.einsum("ij,ij->i", p[f[:, 0]],
                            np.cross(p[f[:, 1]], p[f[:, 2]]))) / 6
    if volume < 0:
        f = f[:, ::-1]
    return p, _normals(p, f), f


def _lathe(sections, sectors=_SECTORS, transform=None):
    """Revolve a shell represented by paired cross-section boundary points.

    Each section is (outside radius, outside Z, inside radius, inside Z).
    Neighboring sections define a shell band, which becomes one convex piece per
    angular sector. Neither a full-vessel hull nor a full-plate cylinder is used.
    """
    sections = np.asarray(sections, dtype=float)
    profile = np.concatenate((sections[:, :2], sections[::-1, 2:]))
    count = sectors * 4
    points, rings, faces = [], [], []
    for radius, z in profile:
        if radius < 1e-10:
            rings.append([len(points)])
            points.append([0, 0, z])
        else:
            rings.append(list(range(len(points), len(points)+count)))
            points.extend([[radius*math.cos(t), radius*math.sin(t), z]
                           for t in np.arange(count)*2*math.pi/count])
    for a, b in zip(rings, rings[1:]+rings[:1]):
        if len(a) == len(b) == 1:
            continue
        for i in range(count):
            j = (i+1) % count
            if len(a) == 1:
                faces.append([a[0], b[i], b[j]])
            elif len(b) == 1:
                faces.append([a[i], b[0], a[j]])
            else:
                faces.extend([[a[i], b[i], b[j]], [a[i], b[j], a[j]]])
    p = np.asarray(points)
    if transform is not None:
        p = transform(p)
    visual = _closed_mesh(p, faces)
    pieces = []
    for lower, upper in zip(sections[:-1], sections[1:]):
        cross = [lower[:2], lower[2:], upper[:2], upper[2:]]
        for sector in range(sectors):
            # Mid-angle vertices keep the outer silhouette discrepancy below
            # 0.3 mm at a 130 mm plate rim and preserve the open inner wall.
            angles = (sector+np.array([0., .5, 1.]))*2*math.pi/sectors
            vertices = [[r*math.cos(t), r*math.sin(t), z]
                        for r, z in cross for t in angles]
            p = np.asarray(vertices)
            if transform is not None:
                p = transform(p)
            pieces.append(_hull(p))
    return visual, pieces


def signed_volume(points, faces):
    p, f = np.asarray(points, dtype=float), np.asarray(faces, dtype=int)
    return float(np.sum(np.einsum("ij,ij->i", p[f[:, 0]], np.cross(p[f[:, 1]], p[f[:, 2]]))) / 6)


# --------------------------------------------------------------------------- 2-D profile

def _arc(p0, p1, sagitta):
    """Circular arc p0 -> p1 bulging by ``sagitta`` to the right of the chord (outward for a
    wall travelled bottom -> top). Returns (point(u), unit_tangent(u)) for u in [0, 1]."""
    p0, p1 = np.asarray(p0, dtype=float), np.asarray(p1, dtype=float)
    d = p1 - p0
    length = float(np.linalg.norm(d))
    t = d / length
    if abs(sagitta) < 1e-9:
        return (lambda u: p0 + d * u), (lambda u: t)
    n = np.array([t[1], -t[0]])
    radius = (length * length / 4 + sagitta * sagitta) / (2 * sagitta)
    centre = (p0 + p1) / 2 - n * (radius - sagitta)
    a0 = math.atan2(*(p0 - centre)[::-1])
    a1 = math.atan2(*(p1 - centre)[::-1])
    da = (a1 - a0 + math.pi) % (2 * math.pi) - math.pi
    sign = 1. if da >= 0 else -1.

    def point(u):
        a = a0 + da * u
        return centre + radius * np.array([math.cos(a), math.sin(a)])

    def tangent(u):
        a = a0 + da * u
        return sign * np.array([-math.sin(a), math.cos(a)])
    return point, tangent


def _inward(tangent):
    """Unit normal on the material side of an outer wall travelled bottom -> top."""
    return np.array([-tangent[1], tangent[0]])


def _bisect(fn, lo, hi, target, steps=60):
    """u in [lo, hi] with fn(u) == target for a monotone increasing fn."""
    for _ in range(steps):
        mid = (lo + hi) / 2
        if fn(mid) < target:
            lo = mid
        else:
            hi = mid
    return (lo + hi) / 2


def rim_inset(wall):
    """Horizontal round-over of the rim; the remaining top flat is >= 0.6 mm (never a knife edge)."""
    return min(max(RIM_FLAT_INSET_M, .35 * wall), (wall - .0006) / 2)


class Curves:
    """Exact 2-D construction of one kind: foot rows, the outer wall curve, its inward offset.

    The interior wall is the exact offset of the outer wall by the wall thickness; where that
    offset dips below the cavity floor it is projected up onto the floor plane, which makes the
    flat floor (the plate's eating well) a DERIVED radius rather than an estimate.
    """

    def __init__(self, kind, v):
        self.kind, self.v = kind, v
        R, H = v["rim_diameter_m"] / 2, v["height_m"]
        w, fl, fw, fh = v["wall_m"], v["floor_m"], v["foot_width_m"], v["foot_height_m"]
        self.R, self.H, self.w, self.ft, self.fh = R, H, w, fh + fl, fh
        self.lr = w / 2
        self.inset = rim_inset(w)
        if kind == "cup":
            rb = v["base_diameter_m"] / 2
            rbi = rb - fw
            taper0 = np.array([rb + .0015, fh + .002])          # end of the base fillet
            taper1 = np.array([R, H - self.lr])                  # the rim is the widest point
            d = taper1 - taper0
            t = d / np.linalg.norm(d)
            self.outer, self.tangent = (lambda u: taper0 + d * u), (lambda u: t)
            fillet = taper0 - np.array([rb, 0.])
            n_f = _inward(fillet / np.linalg.norm(fillet))
            self.foot = [((0., fh), (0., self.ft)), ((rbi - .002, fh), (rbi - .002, self.ft)),
                         ((rbi, 0.), (rbi, self.ft)),
                         ((rb, 0.), self._floor(np.array([rb, 0.]) + w * n_f))]
            self.side_rows = 0
            self.foot_outer_radius = rb
        else:
            rfo = v["foot_diameter_m"] / 2
            rfi = rfo - fw
            self.outer, self.tangent = _arc((rfo, 0.), (R, H - self.lr), v["side_bulge_m"])
            self.foot = [((0., fh), (0., self.ft)), ((rfi - .003, fh), (rfi - .003, self.ft)),
                         ((rfi, 0.), (rfi, self.ft))]
            self.side_rows = int(v["side_rows"])
            self.foot_outer_radius = rfo
        self.u_emerge = (_bisect(lambda u: self.offset(u)[1], 0., 1., self.ft)
                         if self.offset(0.)[1] < self.ft else 0.)
        self.well_radius = float(self.inner_at(self.u_emerge)[0])

    def offset(self, u):
        return self.outer(u) + self.w * _inward(self.tangent(u))

    def _floor(self, q):
        return np.array([q[0], self.ft]) if q[1] < self.ft else np.asarray(q, dtype=float)

    def inner_at(self, u):
        return self._floor(self.offset(u))

    def u_for_outer_z(self, z):
        lo, hi = self.outer(0.)[1], self.outer(1.)[1]
        if not lo < z < hi:
            return None
        return _bisect(lambda u: self.outer(u)[1], 0., 1., z)

    def sparse_us(self):
        us = [0.]
        if self.u_emerge > 1e-6:
            us.append(self.u_emerge)
        us += [self.u_emerge + (1 - self.u_emerge) * k / (self.side_rows + 1) for k in range(1, self.side_rows + 1)]
        us.append(1.)
        return us

    def lip_rows(self, samples):
        """Quarter-ellipse round-over from the wall tops to the rim flat, b in (0, 90] degrees."""
        R, H, w, inset = self.R, self.H, self.w, self.inset
        xo, zo = self.outer(1.)
        xi, zi = self.inner_at(1.)
        rows = []
        for b in np.linspace(0., np.pi / 2, samples + 1)[1:]:
            rows.append(((R - inset) + (xo - (R - inset)) * math.cos(b), zo + (H - zo) * math.sin(b),
                         (R - w + inset) - ((R - w + inset) - xi) * math.cos(b), zi + (H - zi) * math.sin(b)))
        return rows

    def rows(self, us, lip_samples):
        rows = [(*o, *i) for o, i in self.foot]
        rows += [(*self.outer(u), *self.inner_at(u)) for u in us]
        rows += self.lip_rows(lip_samples)
        return np.asarray(rows, dtype=float)


def _check_control_rows(rows):
    if not (abs(rows[0, 0]) < 1e-12 and abs(rows[0, 2]) < 1e-12):
        raise ValueError("row 0 must be on the axis for both curves")
    if (rows[1:, 0] < 1e-6).any() or (rows[1:, 2] < 1e-6).any():
        raise ValueError("only row 0 may touch the axis")
    if (np.diff(rows[:, 3]) < -1e-9).any():
        raise ValueError("inner z must be non-decreasing bottom -> top")
    if (np.linalg.norm(np.diff(rows[:, :2], axis=0), axis=1) < 1e-7).any():
        raise ValueError("duplicate consecutive outer rows")
    if (np.linalg.norm(np.diff(rows[:, 2:], axis=0), axis=1) < 1e-7).any():
        raise ValueError("duplicate consecutive inner rows")
    if rows[-1, 0] - rows[-1, 2] < .0006 - 1e-9:
        raise ValueError("rim flat narrower than 0.6 mm")


def _bump(u):
    return np.where(np.abs(u) < 1, np.cos(np.pi * u / 2) ** 2, 0.)


def _apply_grooves(dense, grooves, wall):
    """Outer-only axisymmetric grooves (raised-cosine cuts) on the dense rows."""
    heights = list(grooves.get("heights_m", []))
    if not heights:
        return dense
    depth, hw = float(grooves["depth_m"]), float(grooves["half_width_m"])
    if depth >= wall - .0005:
        raise ValueError("groove depth must stay below wall - 0.5 mm")
    out = dense.copy()
    for z in heights:
        out[:, 0] -= depth * _bump((out[:, 1] - z) / hw)
    return out


def groove_rows(grooves):
    heights = list(grooves.get("heights_m", []))
    hw = float(grooves.get("half_width_m", 0.))
    return [z + f * hw for z in heights for f in (-1., -.5, 0., .5, 1.)]


def profile(kind, v, wall_samples=36, lip_samples=3):
    """Return (control_rows, dense_rows, meta): sparse collision rows and the dense visual rows.

    The sparse rows are a subset of the dense rows; dense rows additionally carry the grooves.
    """
    curves = Curves(kind, v)
    sparse_us = curves.sparse_us()
    rows = curves.rows(sparse_us, lip_samples=1)
    _check_control_rows(rows)
    grooves = v.get("grooves", v.get("rings", {}))
    extra = groove_rows(grooves)
    if kind == "cup" and int(v["flutes"]["count"]) > 0:
        extra += list(v["flutes"]["fade_z_m"])
    us = set(np.linspace(0., 1., wall_samples + 1).tolist()) | set(sparse_us)
    us |= {u for u in (curves.u_for_outer_z(z) for z in extra) if u is not None}
    dense = curves.rows(sorted(us), lip_samples=lip_samples)
    keep = [0] + [i for i in range(1, len(dense)) if np.linalg.norm(dense[i] - dense[i - 1]) > 1e-7]
    dense = dense[keep]
    _check_control_rows(dense)
    for row in rows:                                        # subset guarantee
        if np.abs(dense - row).sum(axis=1).min() > 1e-9:
            raise AssertionError("a control row is missing from the dense rows")
    dense = _apply_grooves(dense, grooves, v["wall_m"])
    meta = {"well_diameter_m": 2 * curves.well_radius, "u_emerge": curves.u_emerge,
            "rim_inset_m": curves.inset, "rim_flat_m": float(rows[-1, 0] - rows[-1, 2]),
            "lip_radius_m": curves.lr}
    return rows, dense, meta


# --------------------------------------------------------------------------- 3-D geometry

def _smoothstep(z, a, b):
    u = np.clip((z - a) / (b - a), 0., 1.)
    return u * u * (3 - 2 * u)


def apply_flutes(points, n_rows, flutes, ring=RING):
    """Vertical facets on the OUTER rings only (indices 1 .. (n_rows-1)*ring of the lathe layout)."""
    k = int(flutes["count"])
    if k <= 0:
        return points
    if ring % k:
        raise ValueError(f"flute count {k} must divide the visual ring of {ring} vertices")
    depth = float(flutes["depth_m"])
    z0, z1, z2, z3 = flutes["fade_z_m"]
    theta = 2 * np.pi * np.arange(ring) / ring
    if flutes.get("shape", "facet") == "facet":
        step = 2 * np.pi / k
        g = (1 - math.cos(np.pi / k) / np.cos((theta % step) - np.pi / k)) / (1 - math.cos(np.pi / k))
    else:
        g = (1 - np.cos(k * theta)) / 2
    p = np.array(points, dtype=float)
    for row in range(1, n_rows):
        idx = slice(1 + (row - 1) * ring, 1 + row * ring)
        z = float(p[idx, 2][0])
        fade = float(_smoothstep(z, z0, z1) * (1 - _smoothstep(z, z2, z3)))
        if fade <= 0:
            continue
        r = np.hypot(p[idx, 0], p[idx, 1])
        scale = (r - depth * fade * g) / r
        p[idx, 0] *= scale
        p[idx, 1] *= scale
    return p


def geometry(kind, v, sectors=_SECTORS):
    """Dense visual shell + sparse convex collision pieces from one parameter set."""
    rows, dense, meta = profile(kind, v)
    (p, n, f), _ = _lathe(dense)                    # visual: ring = 96
    _, pieces = _lathe(rows, sectors=sectors)       # collision: the control rows only
    n_rows = len(dense)
    if len(p) != 2 + 2 * (n_rows - 1) * RING:
        raise AssertionError("unexpected lathe vertex layout")
    fluted = False
    if kind == "cup" and int(v["flutes"]["count"]) > 0:
        p = apply_flutes(p, n_rows, v["flutes"])
        n = _normals(p, f)
        fluted = True
    if signed_volume(p, f) <= 0:
        raise ValueError("visual shell is inverted")
    return {"visual": (p, n, f), "pieces": pieces, "dense": dense, "rows": rows, "meta": meta,
            "bands": len(rows) - 1, "sectors": sectors, "ring": RING, "fluted": fluted,
            "bounds_m": np.array([p.min(0), p.max(0)])}


def inner_ring_indices(n_rows, ring=RING):
    """Visual-mesh index ranges of the inner rings, top (rim) -> bottom, plus the inner apex."""
    start = 1 + (n_rows - 1) * ring
    rings = [range(start + j * ring, start + (j + 1) * ring) for j in range(n_rows - 1)]
    return rings, start + (n_rows - 1) * ring


def rim_z(geom):
    """Lowest rim point: minimum z over the last outer ring and the first inner ring."""
    p, n_rows = geom["visual"][0], len(geom["dense"])
    rings, _ = inner_ring_indices(n_rows)
    outer_top = range(1 + (n_rows - 2) * RING, 1 + (n_rows - 1) * RING)
    return float(min(p[list(outer_top), 2].min(), p[list(rings[0]), 2].min()))


# --------------------------------------------------------------------------- capacity

def _frustum_sum(r, z):
    dz = np.diff(z)
    return float(np.sum(np.pi / 3 * dz * (r[:-1] ** 2 + r[:-1] * r[1:] + r[1:] ** 2)))


def cavity_volume_m3(dense, top_z):
    """Exact volume of revolution of the inner polyline from the floor up to ``top_z``."""
    r, z = dense[:, 2].copy(), dense[:, 3].copy()
    if (np.diff(z) < -1e-9).any():
        raise ValueError("inner z not monotone")
    mask = z <= top_z + 1e-12
    r, z = r[mask], z[mask]
    if z[-1] < top_z - 1e-12:                       # interpolate the closure height
        i = int(np.sum(mask))
        r_top = np.interp(top_z, dense[i - 1:i + 1, 3], dense[i - 1:i + 1, 2])
        r, z = np.append(r, r_top), np.append(z, top_z)
    return _frustum_sum(r, z)


def mesh_cavity_volume_m3(geom, top_z):
    """Signed volume of the delivered inner surface closed by a temporary fan at ``top_z``."""
    p, n_rows = geom["visual"][0], len(geom["dense"])
    rings, apex = inner_ring_indices(n_rows)
    pts = [np.array([0., 0., top_z])]
    ring_ids = [[0]]
    for ring in rings:
        ring_ids.append(list(range(len(pts), len(pts) + len(ring))))
        pts.extend(p[list(ring)])
    ring_ids.append([len(pts)])
    pts.append(p[apex])
    faces = []
    count = RING
    for a, b in zip(ring_ids[:-1], ring_ids[1:]):
        for i in range(count):
            j = (i + 1) % count
            if len(a) == 1:
                faces.append([a[0], b[i], b[j]])
            elif len(b) == 1:
                faces.append([a[i], b[0], a[j]])
            else:
                faces.extend([[a[i], b[i], b[j]], [a[i], b[j], a[j]]])
    return abs(signed_volume(np.asarray(pts), faces))


def brimful_capacity(kind, v, geom, headspace_m=HEADSPACE_M):
    """Brimful (to the lowest rim point) and headspace capacities with the mesh cross-check."""
    top = rim_z(geom)
    H = v["height_m"]
    if top < H - 1e-9:
        raise ValueError(f"rim is not level: lowest rim point {top} < height {H}")
    dense = geom["dense"]
    brimful = cavity_volume_m3(dense, top)
    mesh = mesh_cavity_volume_m3(geom, top)
    result = {"rim_z_m": top, "headspace_z_m": top - headspace_m, "brimful_ml": brimful * 1e6,
              "brimful_us_floz": brimful * 1e6 / US_FLOZ_ML,
              "headspace_ml": cavity_volume_m3(dense, top - headspace_m) * 1e6,
              "mesh_brimful_ml": mesh * 1e6, "mesh_over_polyline": mesh / brimful,
              "polygon_area_factor": RING / (2 * np.pi) * math.sin(2 * np.pi / RING),
              "method": "pi * integral r_in(z)^2 dz of the delivered inner polyline from the cavity floor to "
                        "the lowest rim point (temporary horizontal closure; asset not capped); mesh cross-check "
                        "= signed volume of the inner surface closed by a fan at the rim"}
    if kind in ADVERTISED_ML:
        adv = ADVERTISED_ML[kind]
        result.update(advertised_ml=adv, advertised_us_floz=adv / US_FLOZ_ML,
                      difference_ml=result["brimful_ml"] - adv,
                      difference_pct=100 * (result["brimful_ml"] - adv) / adv)
    else:
        result.update(advertised_ml=None, note="no plate capacity target; well volume is informational")
    return result


def sensitivity(kind, table):
    """Brimful capacity over wall +-1 mm x foot/base +-10 mm (and the bowl side bulge); never used to tune."""
    if kind not in ADVERTISED_ML:
        return []
    base = values(table, kind)
    foot_key = "base_diameter_m" if kind == "cup" else "foot_diameter_m"
    out = []
    for dw in (-.001, 0., .001):
        for df in (-.010, 0., .010):
            v = dict(base, wall_m=base["wall_m"] + dw, **{foot_key: base[foot_key] + df})
            _, dense, _ = profile(kind, v)
            out.append({"wall_m": v["wall_m"], foot_key: v[foot_key],
                        "brimful_ml": cavity_volume_m3(dense, v["height_m"]) * 1e6})
    if kind == "bowl":
        for sb in (.004, .006, .008, .010):
            v = dict(base, side_bulge_m=sb)
            _, dense, _ = profile(kind, v)
            out.append({"side_bulge_m": sb, "brimful_ml": cavity_volume_m3(dense, v["height_m"]) * 1e6})
    return out


# --------------------------------------------------------------------------- checks

def _segments_intersect(a, b, c, d):
    def orient(p, q, r):
        return (q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0])
    o1, o2, o3, o4 = orient(a, b, c), orient(a, b, d), orient(c, d, a), orient(c, d, b)
    return (o1 * o2 < 0) and (o3 * o4 < 0)


def _point_segment_distance(p, a, b):
    ab = b - a
    t = np.clip(np.dot(p - a, ab) / max(np.dot(ab, ab), 1e-18), 0., 1.)
    return float(np.linalg.norm(p - (a + t * ab)))


def profile_checks(kind, v, dense):
    """Simple closed polygon, monotone inner z, wall thickness, rim flat."""
    loop = np.concatenate((dense[:, :2], dense[::-1, 2:]))
    n = len(loop)
    crossings = 0
    for i in range(n):
        a, b = loop[i], loop[(i + 1) % n]
        for j in range(i + 2, n):
            if i == 0 and j == n - 1:
                continue
            if _segments_intersect(a, b, loop[j], loop[(j + 1) % n]):
                crossings += 1
    w, H = v["wall_m"], v["height_m"]
    lr = w / 2
    outer = dense[:, :2]
    inner_wall = dense[(dense[:, 3] > v["foot_height_m"] + v["floor_m"] + 1e-6) & (dense[:, 3] < H - lr - 1e-6), 2:]
    min_wall = min(min(_point_segment_distance(p, outer[i], outer[i + 1]) for i in range(len(outer) - 1))
                   for p in inner_wall) if len(inner_wall) else float("nan")
    return {"simple_polygon": crossings == 0, "polygon_crossings": crossings,
            "inner_z_monotone": bool((np.diff(dense[:, 3]) >= -1e-9).all()),
            "min_wall_m": min_wall, "min_wall_ok": bool(min_wall >= .6 * w),
            "rim_flat_m": float(dense[-1, 0] - dense[-1, 2]),
            "rim_flat_ok": bool(dense[-1, 0] - dense[-1, 2] >= .0006 - 1e-9)}


def closed_and_oriented(faces):
    """Every undirected edge used twice, every directed edge matched by its reverse."""
    from collections import Counter
    undirected, directed = Counter(), Counter()
    for face in faces:
        for a, b in zip(face, np.roll(face, -1)):
            undirected[tuple(sorted((int(a), int(b))))] += 1
            directed[(int(a), int(b))] += 1
    return (all(count == 2 for count in undirected.values())
            and all(count == directed[(b, a)] for (a, b), count in directed.items()))


def mesh_checks(geom):
    p, n, f = geom["visual"]
    area = np.linalg.norm(np.cross(p[f[:, 1]] - p[f[:, 0]], p[f[:, 2]] - p[f[:, 0]]), axis=1)
    return {"closed_and_oriented": closed_and_oriented(f), "unit_normals": bool(np.allclose(np.linalg.norm(n, axis=1), 1)),
            "min_face_area_m2": float(area.min()), "material_volume_ml": signed_volume(p, f) * 1e6,
            "points": int(len(p)), "triangles": int(len(f))}


class PieceSet:
    """Half-space form of every convex collision piece for fast containment tests."""

    def __init__(self, pieces):
        from scipy.spatial import ConvexHull
        eqs, self.volumes = [], []
        for pts, _ in pieces:
            hull = ConvexHull(pts)
            eqs.append(hull.equations)
            self.volumes.append(float(hull.volume))
        self.sizes = [len(e) for e in eqs]
        self.equations = np.concatenate(eqs)
        self.owner = np.repeat(np.arange(len(pieces)), self.sizes)

    def contains(self, point, tol=1e-10):
        inside = self.equations[:, :3] @ np.asarray(point, dtype=float) + self.equations[:, 3] <= tol
        return bool(np.bincount(self.owner, weights=~inside, minlength=len(self.sizes)).min() == 0)


def _signed_cavity_depth(points_rz, polyline):
    """Distance of (r, z) points from the inner polyline, positive on the cavity side (left of bottom -> top)."""
    a, b = polyline[:-1], polyline[1:]
    ab = b - a
    lengths = np.maximum(np.einsum("ij,ij->i", ab, ab), 1e-18)
    out = np.empty(len(points_rz))
    for i, q in enumerate(points_rz):
        t = np.clip(np.einsum("ij,j->i", ab, q) - np.einsum("ij,ij->i", ab, a), 0., lengths) / lengths
        foot = a + t[:, None] * ab
        d = np.linalg.norm(q - foot, axis=1)
        k = int(np.argmin(d))
        cross = ab[k, 0] * (q[1] - a[k, 1]) - ab[k, 1] * (q[0] - a[k, 0])
        out[i] = d[k] if cross > 0 else -d[k]
    return out


def cavity_intrusion_m(geom, row_samples=9, angle_fractions=(0., .25, .5, .75)):
    """Deepest point of any collider inside the visual interior, normal to that surface.

    Every convex piece spans one band (two consecutive control rows) and one sector whose
    vertices sit at the sector edges and the mid-angle; the deepest points are therefore the
    straight chords between those vertices, along the row (profile chord) and around the
    sector (angular chord). Both are sampled here and measured against the dense inner
    polyline; the result is a lower bound that the hull's flat faces cannot exceed materially.
    """
    rows, dense, sectors = geom["rows"], geom["dense"], geom["sectors"]
    step = 2 * np.pi / sectors
    samples = []
    for k in range(len(rows) - 1):
        lo, hi = rows[k, 2:], rows[k + 1, 2:]
        for s in np.linspace(0., 1., row_samples):
            rho, z = (1 - s) * lo[0] + s * hi[0], (1 - s) * lo[1] + s * hi[1]
            if rho < 1e-9:
                samples.append((0., z))
                continue
            for f in angle_fractions:                       # chords between vertices at 0, step/2, step
                if f <= .5:
                    a, b, g = 0., step / 2, 2 * f
                else:
                    a, b, g = step / 2, step, 2 * f - 1
                ea, eb = np.array([math.cos(a), math.sin(a)]), np.array([math.cos(b), math.sin(b)])
                samples.append((rho * float(np.linalg.norm((1 - g) * ea + g * eb)), z))
    depth = _signed_cavity_depth(np.asarray(samples), dense[:, 2:])
    return float(max(depth.max(), 0.))


def collision_checks(kind, v, geom, clearance_m=.001):
    """Open cavity, solid foot and floor, no protrusion past the crest, collider intrusion depth."""
    pieces = PieceSet(geom["pieces"])
    dense = geom["dense"]
    fh, ft, H = v["foot_height_m"], v["foot_height_m"] + v["floor_m"], v["height_m"]
    intrusion = cavity_intrusion_m(geom)
    axis_open = all(not pieces.contains([0., 0., z]) for z in np.linspace(ft + .002, H - .002, 5))
    inner = dense[:, 2:]
    mid_open = True
    for z in np.linspace(ft + .005, H - .005, 4):           # half-radius probes at several heights
        r = float(np.interp(z, inner[:, 1], inner[:, 0])) / 2
        mid_open = mid_open and not pieces.contains([r, 0., z]) and not pieces.contains([r * math.cos(np.pi / geom["sectors"]), r * math.sin(np.pi / geom["sectors"]), z])
    rfo = (v["base_diameter_m"] if kind == "cup" else v["foot_diameter_m"]) / 2
    foot_probe = [rfo - v["foot_width_m"] / 2, 0., fh / 2]
    floor_probe = [0., 0., fh + v["floor_m"] / 2]
    crest = float(dense[:, 0].max())
    max_radius = max(float(np.hypot(pts[:, 0], pts[:, 1]).max()) for pts, _ in geom["pieces"])
    return {"cavity_probes_open": axis_open and mid_open, "axis_open": axis_open, "mid_radius_open": mid_open,
            "foot_solid": pieces.contains(foot_probe), "floor_solid": pieces.contains(floor_probe),
            "max_cavity_intrusion_m": intrusion, "intrusion_ok": intrusion <= clearance_m,
            "min_piece_volume_m3": min(pieces.volumes), "pieces": len(geom["pieces"]),
            "max_collider_radius_m": max_radius, "crest_radius_m": crest,
            "no_protrusion": max_radius <= crest + 1e-6}


# --------------------------------------------------------------------------- USD (vendored helpers asset.py:41-130)

def _attr(prim, name, kind, value):
    return prim.CreateAttribute(name, kind, custom=False).Set(value)


def _bind(stage, prim, root, material, physics=False):
    from pxr import UsdShade
    UsdShade.MaterialBindingAPI.Apply(prim).Bind(
        UsdShade.Material.Get(stage, root + "/Looks/" + material),
        materialPurpose="physics" if physics else UsdShade.Tokens.allPurpose)


def _collision(stage, prim, root, material="Contact"):
    from pxr import Sdf, UsdPhysics
    UsdPhysics.CollisionAPI.Apply(prim)
    prim.AddAppliedSchema("PhysxCollisionAPI")
    _attr(prim, "physxCollision:contactOffset", Sdf.ValueTypeNames.Float, .0005)
    _attr(prim, "physxCollision:restOffset", Sdf.ValueTypeNames.Float, 0.)
    _bind(stage, prim, root, material, physics=True)


def _mesh(stage, path, points, normals, faces):
    from pxr import UsdGeom, Vt
    p = np.asarray(points, dtype=np.float32)
    faces = np.asarray(faces, dtype=np.int32)
    mesh = UsdGeom.Mesh.Define(stage, path)
    mesh.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(p))
    mesh.CreateFaceVertexCountsAttr(Vt.IntArray.FromNumpy(np.full(len(faces), 3, dtype=np.int32)))
    mesh.CreateFaceVertexIndicesAttr(Vt.IntArray.FromNumpy(faces.reshape(-1)))
    if normals is not None:
        mesh.CreateNormalsAttr(Vt.Vec3fArray.FromNumpy(np.asarray(normals, dtype=np.float32)))
        mesh.SetNormalsInterpolation("vertex")
    mesh.CreateSubdivisionSchemeAttr("none")
    mesh.CreateExtentAttr(Vt.Vec3fArray.FromNumpy(np.array([p.min(0), p.max(0)], dtype=np.float32)))
    return mesh.GetPrim()


def _import_pxr():
    try:
        import pxr  # noqa: F401
    except ImportError as exc:
        raise ImportError("pxr is not importable: run through code/scripts/run_py.sh (its omni.usd.libs shim) or inside "
                          "an environment prepared like dishsim_frigidaire.usd_bootstrap.bundled_usd_environment()") from exc


def write_asset(kind, out_dir, table, geom, capacity, checks, origin="base"):
    """Write ``<kind>.usda``: default prim, materials + colour variants, visual, convex colliders; physics:mass
    on the root prim only when the kind's ``mass_kg`` parameter is set (measured), nothing mass-related otherwise."""
    _import_pxr()
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade
    path = Path(out_dir) / f"{kind}.usda"
    p, n, f = geom["visual"]
    shift = np.array([0., 0., values(table, kind)["height_m"] / 2]) if origin == "centre" else np.zeros(3)
    stage = Usd.Stage.CreateNew(str(path))
    root = f"/{ROOT_PRIMS[kind]}"
    body = UsdGeom.Xform.Define(stage, root).GetPrim()
    stage.SetDefaultPrim(body)
    UsdGeom.SetStageMetersPerUnit(stage, 1.)
    UsdGeom.SetStageUpAxis(stage, "Z")
    stage.GetRootLayer().customLayerData = {           # strings only: nested lists do not round-trip
        "generator": "dishsim.hotec_gen", "generator_sha256": generator_sha256(),
        "hotec_kind": kind, "hotec_parameters_json": json.dumps(table[kind], sort_keys=True),
        "hotec_parameters_sha256": parameters_sha256(table), "origin": ORIGIN_STATUS[origin],
        "mass_status": mass_status(table, kind), "dimension_status": DIMENSION_STATUS}
    UsdPhysics.RigidBodyAPI.Apply(body)
    body.AddAppliedSchema("PhysxRigidBodyAPI")
    mass = table[kind]["mass_kg"]["value"]
    if mass is not None:                               # measured: the mass itself, nothing else (PhysX derives inertia)
        UsdPhysics.MassAPI.Apply(body).CreateMassAttr(float(mass))
    _attr(body, "physxRigidBody:solverPositionIterationCount", Sdf.ValueTypeNames.Int, PHYSX["solverPositionIterationCount"])
    _attr(body, "physxRigidBody:solverVelocityIterationCount", Sdf.ValueTypeNames.Int, PHYSX["solverVelocityIterationCount"])
    _attr(body, "physxRigidBody:enableCCD", Sdf.ValueTypeNames.Bool, PHYSX["enableCCD"])
    _attr(body, "physxRigidBody:maxDepenetrationVelocity", Sdf.ValueTypeNames.Float, PHYSX["maxDepenetrationVelocity"])
    for name, value in (("hotec:kind", kind), ("hotec:parametersSha256", parameters_sha256(table)),
                        ("hotec:massStatus", mass_status(table, kind)), ("hotec:origin", ORIGIN_STATUS[origin]),
                        ("hotec:dimensionStatus", DIMENSION_STATUS)):
        body.CreateAttribute(name, Sdf.ValueTypeNames.String).Set(value)
    material = UsdShade.Material.Define(stage, root + "/Looks/Surface")
    shader = UsdShade.Shader.Define(stage, material.GetPath().AppendChild("Shader"))
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.)
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(.30)
    material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    vset = body.GetVariantSets().AddVariantSet("color")
    for name, rgb in COLOURS.items():                 # diffuseColor ONLY inside the variants (LIVRPS)
        vset.AddVariant(name)
        vset.SetVariantSelection(name)
        with vset.GetVariantEditContext():
            shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*rgb))
    vset.SetVariantSelection(DEFAULT_COLOUR)
    contact = UsdShade.Material.Define(stage, root + "/Looks/Contact")
    physics = UsdPhysics.MaterialAPI.Apply(contact.GetPrim())
    physics.CreateStaticFrictionAttr(CONTACT["static_friction"])
    physics.CreateDynamicFrictionAttr(CONTACT["dynamic_friction"])
    physics.CreateRestitutionAttr(CONTACT["restitution"])
    UsdGeom.Scope.Define(stage, root + "/Visuals")
    scope = UsdGeom.Scope.Define(stage, root + "/Collisions").GetPrim()
    UsdGeom.Imageable(scope).CreateVisibilityAttr("invisible")
    prim = _mesh(stage, root + "/Visuals/Body", p - shift, n, f)
    _bind(stage, prim, root, "Surface")
    for i, (pts, faces) in enumerate(geom["pieces"]):
        prim = _mesh(stage, root + f"/Collisions/Shell_{i:03d}", pts - shift, None, faces)
        UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr("convexHull")
        _collision(stage, prim, root)
    stage.GetRootLayer().Save()
    return path


def usd_bounds(path):
    """World bound of the default prim after re-opening the saved file (visuals only; collisions are invisible)."""
    _import_pxr()
    from pxr import Usd, UsdGeom
    stage = Usd.Stage.Open(str(path))
    root = stage.GetDefaultPrim()
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
    box = cache.ComputeWorldBound(root).ComputeAlignedRange()
    lo, hi = np.array(box.GetMin()), np.array(box.GetMax())
    return {"prim": str(root.GetPath()), "meters_per_unit": UsdGeom.GetStageMetersPerUnit(stage),
            "up_axis": UsdGeom.GetStageUpAxis(stage), "min_m": lo.tolist(), "max_m": hi.tolist(),
            "size_m": (hi - lo).tolist()}


# --------------------------------------------------------------------------- figures

def write_figures(kind, v, geom, capacity, checks, out_dir, ax_pair=None):
    """Cross-section with the dense curves, control rows, collision band quads, fill, rim and headspace lines."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Polygon
    dense, rows = geom["dense"], geom["rows"]
    own = ax_pair is None
    if own:
        fig, (ax, side) = plt.subplots(1, 2, figsize=(11, 5), gridspec_kw={"width_ratios": [3, 2]})
    else:
        ax, side = ax_pair
    mm = 1000.
    for sign in (1, -1):
        ax.plot(sign * dense[:, 0] * mm, dense[:, 1] * mm, color="#1f4e79", lw=1.6, label="outer (visual)" if sign == 1 else None)
        ax.plot(sign * dense[:, 2] * mm, dense[:, 3] * mm, color="#c0392b", lw=1.6, label="inner (visual)" if sign == 1 else None)
        for a, b in zip(rows[:-1], rows[1:]):
            quad = np.array([[a[0], a[1]], [b[0], b[1]], [b[2], b[3]], [a[2], a[3]]]) * mm
            quad[:, 0] *= sign
            ax.add_patch(Polygon(quad, closed=True, facecolor="#f39c12", alpha=.18, edgecolor="#b9770e", lw=.6))
        ax.plot(sign * rows[:, 0] * mm, rows[:, 1] * mm, "o", color="#1f4e79", ms=3.5)
        ax.plot(sign * rows[:, 2] * mm, rows[:, 3] * mm, "s", color="#c0392b", ms=3.0)
    if capacity is not None:
        top = capacity["rim_z_m"]
        inner = dense[dense[:, 3] <= top + 1e-12]
        fill = np.concatenate((inner[:, 2:][::-1] * [[-1, 1]], inner[:, 2:]))
        ax.add_patch(Polygon(fill * mm, closed=True, facecolor="#5dade2", alpha=.25, hatch="//", edgecolor="none"))
        ax.axhline(top * mm, ls="--", color="#2e86c1", lw=1, label="brimful (lowest rim point)")
        ax.axhline(capacity["headspace_z_m"] * mm, ls=":", color="#2e86c1", lw=1, label="5 mm headspace")
        text = (f"brimful {capacity['brimful_ml']:.0f} mL ({capacity['brimful_us_floz']:.1f} fl oz)\n"
                f"5 mm headspace {capacity['headspace_ml']:.0f} mL\n"
                f"advertised {capacity['advertised_ml']:.0f} mL ({capacity['advertised_us_floz']:.0f} fl oz)\n"
                f"mesh/polyline {capacity['mesh_over_polyline']:.4f}")
    else:
        text = f"well volume {cavity_volume_m3(dense, v['height_m']) * 1e6:.0f} mL (no target)"
    text += (f"\nmin wall {checks['profile']['min_wall_m'] * mm:.2f} mm\n"
             f"cavity intrusion {checks['collision']['max_cavity_intrusion_m'] * mm:.2f} mm\n"
             f"{checks['collision']['pieces']} convex pieces ({geom['bands']} bands x {geom['sectors']})")
    ax.text(.02, .98, text, transform=ax.transAxes, va="top", ha="left", fontsize=8,
            bbox={"boxstyle": "round", "facecolor": "white", "alpha": .85})
    R, H = v["rim_diameter_m"] / 2 * mm, v["height_m"] * mm
    ax.set_xlim(-R * 1.08, R * 1.08)
    ax.set_ylim(-.06 * H - 2, H * 1.15 + 2)
    ax.set_aspect("equal")
    ax.set_xlabel("radius [mm]")
    ax.set_ylabel("z [mm]  (0 = foot plane)")
    ax.set_title(f"HOTEC {kind}: {v['rim_diameter_m'] * mm:.1f} mm x {H:.1f} mm")
    ax.legend(loc="lower right", fontsize=7)
    p = geom["visual"][0]
    n_rows = len(dense)
    if kind == "cup":
        target = v["height_m"] / 2
        best = min(range(1, n_rows), key=lambda r: abs(p[1 + (r - 1) * RING, 2] - target))
        ring = p[1 + (best - 1) * RING: 1 + best * RING]
        theta = np.arctan2(ring[:, 1], ring[:, 0])
        r = np.hypot(ring[:, 0], ring[:, 1]) * mm
        side.plot(np.degrees(theta) % 360, r, ".-", color="#1f4e79", ms=3, lw=.8)
        side.set_xlabel("theta [deg]")
        side.set_ylabel(f"outer radius at z = {p[1 + (best - 1) * RING, 2] * mm:.0f} mm [mm]")
        side.set_title(f"facets: {v['flutes']['count']} x {v['flutes']['depth_m'] * mm:.2f} mm (outer only)")
    elif kind == "bowl":
        g = v["grooves"]
        zs = g["heights_m"]
        zoom = dense[(dense[:, 1] > (min(zs) - .006 if zs else H / mm * .6)) & (dense[:, 1] < (max(zs) + .006 if zs else H / mm))]
        side.plot(zoom[:, 0] * mm, zoom[:, 1] * mm, ".-", color="#1f4e79", ms=3)
        side.plot(zoom[:, 2] * mm, zoom[:, 3] * mm, ".-", color="#c0392b", ms=3)
        side.set_aspect("equal")
        side.set_title(f"grooves: {len(zs)} x {g['depth_m'] * mm:.1f} mm deep (visual only)")
        side.set_xlabel("radius [mm]")
    else:
        zoom = dense[dense[:, 0] < v["foot_diameter_m"] / 2 + .02]
        side.plot(zoom[:, 0] * mm, zoom[:, 1] * mm, ".-", color="#1f4e79", ms=3)
        side.plot(zoom[:, 2] * mm, zoom[:, 3] * mm, ".-", color="#c0392b", ms=3)
        side.set_aspect("equal")
        side.set_title("foot ring, recess and well edge")
        side.set_xlabel("radius [mm]")
    side.grid(alpha=.3)
    if own:
        fig.tight_layout()
        path = Path(out_dir) / f"{kind}_profile.png"
        fig.savefig(path, dpi=110)
        plt.close(fig)
        return path
    return None


def write_combined_figure(kinds, out_png):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(2, 3, figsize=(16, 9), gridspec_kw={"height_ratios": [3, 2]})
    for col, (kind, v, geom, capacity, checks) in enumerate(kinds):
        write_figures(kind, v, geom, capacity, checks, None, ax_pair=(axes[0, col], axes[1, col]))
    fig.suptitle("HOTEC wheat-straw set: cross-sections (blue outer, red inner, orange collision bands, hatched brimful)", fontsize=11)
    fig.tight_layout()
    fig.savefig(out_png, dpi=90)
    plt.close(fig)
    return out_png


# --------------------------------------------------------------------------- build

def generator_sha256():
    return hashlib.sha256(Path(__file__).read_bytes()).hexdigest()


def _sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _json_ready(obj):
    if isinstance(obj, dict):
        return {k: _json_ready(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [_json_ready(v) for v in obj]
    if isinstance(obj, np.ndarray):
        return obj.tolist()
    if isinstance(obj, (np.floating, np.integer, np.bool_)):
        return obj.item()
    return obj


def evaluate(kind, table, sectors=_SECTORS):
    """Geometry, capacity and all checks for one kind, without touching the filesystem."""
    v = values(table, kind)
    geom = geometry(kind, v, sectors=sectors)
    capacity = brimful_capacity(kind, v, geom) if kind in ADVERTISED_ML else None
    checks = {"profile": profile_checks(kind, v, geom["dense"]), "mesh": mesh_checks(geom),
              "collision": collision_checks(kind, v, geom)}
    size = geom["bounds_m"][1] - geom["bounds_m"][0]
    target = np.array([v["rim_diameter_m"], v["rim_diameter_m"], v["height_m"]])
    checks["dimensions"] = {"target_size_m": target.tolist(), "actual_size_m": size.tolist(),
                            "size_error_m": (size - target).tolist(), "min_z_m": float(geom["bounds_m"][0, 2]),
                            "ok": bool(np.abs(size - target).max() <= .0005 and abs(geom["bounds_m"][0, 2]) < 1e-9)}
    checks["all_ok"] = bool(checks["dimensions"]["ok"] and checks["profile"]["simple_polygon"]
                            and checks["profile"]["inner_z_monotone"] and checks["profile"]["min_wall_ok"]
                            and checks["profile"]["rim_flat_ok"] and checks["mesh"]["closed_and_oriented"]
                            and checks["mesh"]["unit_normals"] and checks["mesh"]["min_face_area_m2"] > 1e-14
                            and checks["collision"]["cavity_probes_open"] and checks["collision"]["foot_solid"]
                            and checks["collision"]["floor_solid"] and checks["collision"]["no_protrusion"]
                            and checks["collision"]["intrusion_ok"] and checks["collision"]["min_piece_volume_m3"] > 1e-12
                            and (capacity is None or .998 <= capacity["mesh_over_polyline"] <= 1.0005))
    return v, geom, capacity, checks


MEASUREMENTS_REQUESTED = [
    "brimful water mass of one bowl and one cup (kitchen scale, +-1 g, water at ~20 C: 1 g ~ 1 mL)",
    "rim outer diameter, foot/base outer diameter and overall height of each kind (calipers or ruler)",
    "wall thickness at the rim and floor thickness (calipers)",
    "cup: number of vertical facets (or rings) and their depth; whether facets reach the rim",
    "bowl: heights of the three grooves above the foot plane",
    "which pack the cup belongs to (20-pack fluted or 24-pack ringed)",
    "mass of each piece (measured 2026-09-22 as 8-piece totals; authored as physics:mass via --set kind.mass_kg)",
]


def build(out_dir, overrides=None, origin="base", sectors=_SECTORS, force=False, figures=True):
    """Write the three assets, parameters.json, catalog.json, figures and README into ``out_dir``."""
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    existing = [k for k in KINDS if (out_dir / f"{k}.usda").exists()]
    if existing and not force:
        raise FileExistsError(f"Refusing to overwrite {existing} in {out_dir}; use --force or a new version directory")
    table = resolve(overrides)
    table_hash = parameters_sha256(table)
    catalog = {"schema_version": 1,
               "set": "HOTEC wheat-straw dinnerware: 8 plates, 8 bowls, 8 cups; four colours x two pieces per kind",
               "units": "metres", "up_axis": "Z", "origin": ORIGIN_STATUS[origin],
               "quaternion_order": "XYZW in project records; WXYZ at the USD/Isaac boundary (no poses are authored here)",
               "generator": "dishsim.hotec_gen", "generator_sha256": generator_sha256(), "parameters_sha256": table_hash,
               "mass_status": {kind: mass_status(table, kind) for kind in KINDS}, "dimension_status": DIMENSION_STATUS,
               "collision_method": f"one convex hull per control-row band per {360 // sectors}-degree sector (mid-angle vertices); "
                                   "the control rows are a subset of the dense visual rows; cavities and foot recess open; "
                                   "decorative grooves/flutes are visual only",
               "variant_set": {"name": "color", "variants": list(COLOURS), "default": DEFAULT_COLOUR, "colours_rgb": COLOURS},
               "items": {}, "sha256": {}}
    parameters = {"schema_version": 1, "parameters_sha256": table_hash, "generator_sha256": catalog["generator_sha256"],
                  "kinds": table, "measurements_requested": MEASUREMENTS_REQUESTED, "capacity": {}, "sensitivity": {}, "checks": {}}
    figure_dir = out_dir / "figures"
    combined = []
    for kind in KINDS:
        v, geom, capacity, checks = evaluate(kind, table, sectors=sectors)
        path = write_asset(kind, out_dir, table, geom, capacity, checks, origin=origin)
        bounds = usd_bounds(path)
        target = np.array(checks["dimensions"]["target_size_m"])
        usd_error = np.abs(np.array(bounds["size_m"]) - target).max()
        if usd_error > .0005 or bounds["meters_per_unit"] != 1. or bounds["up_axis"] != "Z":
            raise ValueError(f"{path}: re-read bound/metadata mismatch: {bounds}")
        item = {"usd": path.name, "default_prim": bounds["prim"], "sha256": _sha256(path),
                "target_size_m": target.tolist(), "actual_size_m": checks["dimensions"]["actual_size_m"],
                "usd_size_m": bounds["size_m"], "usd_bounds_m": [bounds["min_m"], bounds["max_m"]],
                "size_error_m": checks["dimensions"]["size_error_m"], "convex_colliders": len(geom["pieces"]),
                "collision_bands": geom["bands"], "collision_sectors": sectors,
                "visual_points": int(len(geom["visual"][0])), "visual_triangles": int(len(geom["visual"][2])),
                "fluted": geom["fluted"], "capacity": capacity, "checks_all_ok": checks["all_ok"],
                "mass_kg": table[kind]["mass_kg"]["value"], "mass_status": mass_status(table, kind)}
        catalog["items"][kind] = _json_ready(item)
        parameters["capacity"][kind] = _json_ready(capacity)
        parameters["sensitivity"][kind] = _json_ready(sensitivity(kind, table))
        parameters["checks"][kind] = _json_ready(checks)
        combined.append((kind, v, geom, capacity, checks))
        if figures:
            figure_dir.mkdir(exist_ok=True)
            write_figures(kind, v, geom, capacity, checks, figure_dir)
    if figures:
        write_combined_figure(combined, figure_dir / "hotec_profiles.png")
    (out_dir / "parameters.json").write_text(json.dumps(parameters, indent=2, allow_nan=False) + "\n")
    (out_dir / "README.md").write_text(_readme(catalog, parameters))
    for path in sorted(out_dir.rglob("*")):
        if path.is_file() and path.name != "catalog.json":
            catalog["sha256"][str(path.relative_to(out_dir))] = _sha256(path)
    (out_dir / "catalog.json").write_text(json.dumps(catalog, indent=2, allow_nan=False) + "\n")
    return catalog, parameters


def _readme(catalog, parameters):
    lines = ["# HOTEC wheat-straw dinnerware assets", "",
             "Open plate.usda, bowl.usda or cup.usda (ASCII USD). Metres, Z up, origin at the base centre "
             "(foot plane z = 0), default prims /HotecPlate, /HotecBowl, /HotecCup; four colour variants on the "
             "default prim (variantSet `color`).", "",
             (("Masses are authored as physics:mass on each default prim (" + MASS_PROVENANCE + "); no density or inertia. ")
              if all(parameters["kinds"][kind]["mass_kg"]["value"] is not None for kind in KINDS) else
              "No mass is authored (no PhysicsMassAPI, no density): measure the real pieces before assigning one. ")
             + "Dimensions are nominal listing targets plus documented estimates (parameters.json carries every value "
             "with its status and note); see docs/hotec_wheatstraw_asset.md in the source repository.", "",
             "| kind | size [m] | pieces | brimful [mL] | advertised [mL] |", "|---|---|---:|---:|---:|"]
    for kind, item in catalog["items"].items():
        cap = item["capacity"]
        lines.append(f"| {kind} | {item['actual_size_m'][0]:.4f} x {item['actual_size_m'][1]:.4f} x {item['actual_size_m'][2]:.4f} "
                     f"| {item['convex_colliders']} | {cap['brimful_ml']:.0f} | {cap['advertised_ml']:.0f} |"
                     if cap else f"| {kind} | {item['actual_size_m'][0]:.4f} x {item['actual_size_m'][1]:.4f} x "
                                 f"{item['actual_size_m'][2]:.4f} | {item['convex_colliders']} | - | - |")
    lines += ["", f"generator sha256 {catalog['generator_sha256'][:16]}..., parameters sha256 {catalog['parameters_sha256'][:16]}...", ""]
    return "\n".join(lines)


def _parse_override(text):
    path, _, raw = text.partition("=")
    if not raw:
        raise argparse.ArgumentTypeError("--set expects kind.name[.sub]=value")
    try:
        value = json.loads(raw)
    except json.JSONDecodeError:
        value = raw
    return path.strip(), value


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--out", type=Path, required=True, help="version directory, e.g. data/assets/models/hotec_wheatstraw/v1")
    parser.add_argument("--set", action="append", default=[], type=_parse_override, metavar="kind.name=value",
                        help="override one parameter (tagged measured), repeatable; JSON for lists/dicts")
    parser.add_argument("--origin", choices=("base", "centre"), default="base")
    parser.add_argument("--collision-sectors", type=int, default=_SECTORS, dest="sectors")
    parser.add_argument("--force", action="store_true", help="overwrite existing .usda files in --out")
    parser.add_argument("--no-figures", action="store_true")
    args = parser.parse_args(argv)
    catalog, parameters = build(args.out, overrides=dict(args.set), origin=args.origin, sectors=args.sectors,
                                force=args.force, figures=not args.no_figures)
    ok = True
    for kind, item in catalog["items"].items():
        cap = item["capacity"]
        ok = ok and item["checks_all_ok"]
        line = (f"[INFO] {kind}: size {np.round(item['actual_size_m'], 4).tolist()} m, {item['convex_colliders']} pieces "
                f"({item['collision_bands']} bands), checks {'ok' if item['checks_all_ok'] else 'FAILED'}")
        if cap:
            line += (f", brimful {cap['brimful_ml']:.1f} mL vs advertised {cap['advertised_ml']:.1f} mL "
                     f"({cap['difference_pct']:+.1f} %), headspace {cap['headspace_ml']:.1f} mL")
        print(line)
    print(f"[RESULT] {'PASS' if ok else 'FAIL'} {args.out}", flush=True)
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
