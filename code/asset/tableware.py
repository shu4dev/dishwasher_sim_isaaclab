# Copyright (c) 2026, dishsim project.
# SPDX-License-Identifier: BSD-3-Clause
"""Fixed-size everyday tableware for the Frigidaire mixed-load experiment.

The catalog is independent of benchmark props and imports neither USD nor Kit at
module import. All dimensions are explicit modeling defaults in metres, not
dimensions inferred from the reference photographs. The USD writer moved to the asset
folder (``data/assets/frigidaire_prototypes/source/write_tableware.py``); the prototypes
it wrote live in ``data/assets/frigidaire_prototypes/usd`` (``paths.PROTOTYPES_DIR``).
Plate normals are local +Z; hollow vessels open toward +Z; mug handles face +X.
Cutlery is longitudinal in Z, with fork tines and spoon bowls at the +Z end.
"""
from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path

import numpy as np


CATALOG = {
    "dinner_plate": {"size_m": [.260, .260, .020], "diameter_m": .260,
                     "mass_kg": .65, "origin": "center of outer bounds", "rack": "lower"},
    "salad_plate": {"size_m": [.205, .205, .018], "diameter_m": .205,
                    "mass_kg": .40, "origin": "center of outer bounds", "rack": "lower"},
    "saucer": {"size_m": [.150, .150, .018], "diameter_m": .150,
               "mass_kg": .20, "origin": "center of outer bounds", "rack": "upper"},
    "bowl": {"size_m": [.140, .140, .065], "diameter_m": .140,
             "mass_kg": .35, "origin": "base center; bottom z=0", "rack": ["lower", "upper"]},
    "mug": {"size_m": [.120, .085, .100], "body_diameter_m": .085,
            "mass_kg": .30, "origin": "body center; handle +X", "rack": "upper"},
    "tumbler": {"size_m": [.080, .080, .160], "rim_diameter_m": .080,
                "base_diameter_m": .075, "mass_kg": .25,
                "origin": "body center", "rack": "upper"},
    "fork": {"size_m": [.025, .012, .195], "mass_kg": .045,
             "origin": "longitudinal center; tines +Z", "rack": "basket"},
    "knife": {"size_m": [.018, .006, .215], "mass_kg": .055,
              "origin": "longitudinal center; blade +Z", "rack": "basket"},
    "tablespoon": {"size_m": [.040, .018, .190], "mass_kg": .050,
                   "origin": "longitudinal center; bowl +Z opens +Y", "rack": "basket"},
    "teaspoon": {"size_m": [.030, .012, .155], "mass_kg": .025,
                 "origin": "longitudinal center; bowl +Z opens +Y", "rack": "basket"},
}

_COLORS = {"dinner_plate": (.72, .81, .83), "salad_plate": (.91, .87, .73),
           "saucer": (.66, .78, .69), "bowl": (.75, .43, .31),
           "mug": (.37, .60, .71), "tumbler": (.78, .87, .90)}
_SECTORS = 24


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


def _loft(sections, sides=12):
    """Rounded cutlery cross sections (Z, X center, Y center, X/Y radii)."""
    points, faces, rings = [], [], []
    for z, x, y, rx, ry in sections:
        rings.append(list(range(len(points), len(points)+sides)))
        points.extend([[x+rx*math.cos(t), y+ry*math.sin(t), z]
                       for t in np.arange(sides)*2*math.pi/sides])
    for a, b in zip(rings[:-1], rings[1:]):
        for i in range(sides):
            j = (i+1) % sides
            faces.extend([[a[i], b[j], b[i]], [a[i], a[j], b[j]]])
    for ring, reverse in [(rings[0], True), (rings[-1], False)]:
        for i in range(1, sides-1):
            face = [ring[0], ring[i], ring[i+1]]
            faces.append(face[::-1] if reverse else face)
    p = np.asarray(points)
    pieces = [_hull(np.concatenate((p[a], p[b]))) for a, b in zip(rings[:-1], rings[1:])]
    return _closed_mesh(p, faces), pieces


def tableware_geometry(kind):
    """Return independent numpy visual meshes, convex shell pieces, and capsules.

    This inspection seam intentionally exposes actual authored collider geometry
    for reproducible packing checks without running Kit.
    """
    spec = CATALOG[kind]
    result = {"visuals": [], "convexes": [], "capsules": []}

    def add(pair):
        visual, colliders = pair
        result["visuals"].append(visual)
        result["convexes"].extend(colliders)

    if kind in ("dinner_plate", "salad_plate", "saucer"):
        radius, height = spec["diameter_m"]/2, spec["size_m"][2]
        # A shallow eating well, foot, sloping shoulder, and rounded raised lip.
        half = height/2
        add(_lathe([(0, -.70*half, 0, -.30*half),
                    (.36*radius, -half, .36*radius, -.48*half),
                    (.59*radius, -.80*half, .59*radius, -.27*half),
                    (.83*radius, .19*half, .83*radius, .65*half),
                    (radius, .73*half, radius-.0008, half)]))
    elif kind == "bowl":
        add(_lathe([(0, 0, 0, .006), (.028, 0, .024, .006),
                    (.045, .020, .040, .025), (.065, .050, .060, .052),
                    (.070, .062, .066, .062), (.0695, .065, .0665, .065)]))
    elif kind in ("mug", "tumbler"):
        height = spec["size_m"][2]
        radius = spec.get("body_diameter_m", spec.get("rim_diameter_m"))/2
        base = spec.get("base_diameter_m", .079)/2
        wall = .0035 if kind == "mug" else .0022
        bottom = .006 if kind == "mug" else .005
        lo, hi = -height/2, height/2
        add(_lathe([(0, lo, 0, lo+bottom), (base, lo, base-wall, lo+bottom),
                    (radius-.001, lo+.014, radius-.001-wall, lo+.015),
                    (radius, hi-.002, radius-wall, hi-.002),
                    (radius-.0003, hi, radius-wall+.0003, hi)]))
        if kind == "mug":
            from .geometry import Mesh
            theta = np.linspace(-math.pi/2, math.pi/2, 21)
            path = np.column_stack((.0405+.0325*np.cos(theta),
                                    np.zeros_like(theta), .0335*np.sin(theta)))
            mesh = Mesh("Surface")
            mesh.tube(path, .0045, sides=16)
            result["visuals"].append((np.asarray(mesh.points), np.asarray(mesh.normals), np.asarray(mesh.faces)))
            result["capsules"].extend((a, b, .0045) for a, b in zip(path[:-1], path[1:]))
    elif kind == "fork":
        add(_loft([(-.0975, 0, 0, .001, .0005), (-.094, 0, 0, .0055, .0025),
                   (-.040, 0, -.001, .0045, .002), (.025, 0, -.004, .0033, .002),
                   (.052, 0, .001, .009, .002), (.065, 0, .004, .0125, .002)]))
        for x in (-.0104, -.0034666667, .0034666667, .0104):
            add(_loft([(.0635, x, .004, .0021, .0015),
                       (.078, x, .003, .0017, .0012),
                       (.095, x, .001, .0011, .0008),
                       (.0975, x, .0007, .00025, .00025)]))
    elif kind == "knife":
        add(_loft([(-.1075, 0, 0, .001, .0005), (-.104, 0, 0, .0055, .003),
                   (-.025, 0, 0, .0048, .0028), (.004, 0, 0, .004, .002),
                   (.025, 0, 0, .008, .0016), (.082, 0, 0, .009, .0012),
                   (.101, -.002, 0, .0065, .001), (.1075, -.004, 0, .001, .0004)]))
    elif kind in ("tablespoon", "teaspoon"):
        width, depth, length = spec["size_m"]
        a, b = width/2, .030 if kind == "tablespoon" else .022
        center = length/2-b
        def spoon_transform(p):
            return np.column_stack((p[:, 0]*a, p[:, 2], center-p[:, 1]*b))
        add(_lathe([(0, -depth/2, 0, -depth/2+.002),
                    (.45, -.37*depth, .45, -.37*depth+.002),
                    (.80, .02*depth, .80, .02*depth+.0017),
                    (1., .42*depth, .995, depth/2)], sectors=20, transform=spoon_transform))
        add(_loft([(-length/2, 0, .002, .001, .0005),
                   (-length/2+.004, 0, .002, .005, .0015),
                   (-.020, 0, .0015, .0038, .0014),
                   (center-b-.007, 0, depth*.35, .003, .0012),
                   (center-b+.006, 0, depth*.35, .0035, .0012)]))
    else:
        raise KeyError(kind)
    return result
