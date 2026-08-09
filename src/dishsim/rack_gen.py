# Copyright (c) 2026, dishsim project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Procedural realistic dishwasher-rack geometry — the single source of truth for both racks.

v2 is styled after three reference appliances (Whirlpool WDTA50SAKZ, Bosch 800 Series,
Frigidaire FDPC4314AS), from parts-listing/manual research: a 3-gauge wire hierarchy
(frame >> runners > tines, real tine-dia/pitch ratio ~0.07), 30 mm Whirlpool-pattern tine rows
with candy-cane end hooks, base fillets / mid-height tie wires / bead tip caps, a dark
fold-down insert row (own mesh + material), roller wheels, a dipped front rail with a grab
handle, and fold-down cup shelves + RackMatic lever blocks on the upper rack.

Kit-free by design (numpy + trimesh only, no ``pxr``): the same builder feeds three consumers —
:mod:`dishsim.usd_prep` authors the merged per-group meshes into the derived v0 USD (PhysX/SDF
side), ``scripts/setup/decompose_meshes.py`` writes the exact convex parts as FCL pieces (no CoACD
for the racks), and the Kit-free tests/preview validate the shape before any Kit run.

Design space is the world-metric rack BODY frame: X = width, Y = depth with y=0 the front edge
(the end that extends toward the robot), Z up with 0 at the lowest wire surface. Geometry spans
exactly ``[0, W] x [0, D]``, z >= 0 — same convention as the ArtVIP mesh it replaces, so the
prismatic-joint anchors and the slot-grid arithmetic keep working. The rack Xforms in the ArtVIP
USD carry a non-uniform x-scale (0.9191 lower / 0.94 upper) that children inherit;
:func:`mesh_arrays_usd` pre-divides authored x-coordinates by that scale so the world-space
result equals the design geometry exactly and the Phase-D extraction round-trips it.

Every part is convex and watertight by construction (ring-stack prisms; corner arcs, wall
ramps, candy canes, fillets, scallops and the V-profile cradle ribs are discretized into short
straight segments), so the parts double as the exact FCL convex decomposition. No RNG anywhere —
identical params, identical geometry.

Wire cross-sections are regular 12-gons: 12 divides both 90 and 180 degrees, so axis-aligned
rods have vertices pointing exactly along the negative axes and the assembled bounding box lands
on the configured footprint to float precision (the slot grid derives from these bounds).
"""

import hashlib
import json
import math
from dataclasses import dataclass, field

import numpy as np
import trimesh

from . import config

#: preview colors per zone (hex, matplotlib-compatible)
ZONE_COLORS = {
    "perimeter": "#4d4d4d",
    "guard": "#1a80bb",
    "floor_open": "#8a8a8a",
    "floor_dense": "#b8642d",
    "channel": "#2ca089",
    "slope": "#7b52ab",
    "plate_tines": "#c23b22",
    "bowl_tines": "#d29c2f",
    "divider_tines": "#c23b22",
    "ribs": "#5c8a3a",
    "tie": "#96694a",
    "insert": "#33343a",
    "wheels": "#2b2b2b",
    "handle": "#1a80bb",
    "cup_shelf": "#7b52ab",
    "rackmatic": "#33343a",
    "basket": "#9aa0a6",
    "basket_handle": "#7d838a",
}


@dataclass(frozen=True)
class RackPart:
    """One convex, watertight wire segment of the rack, in design space.

    ``group`` selects the authored USD mesh: ``frame`` (main rack material) or ``insert``
    (the dark fold-down tine-row insert, authored as a second mesh with its own material).
    """

    name: str
    zone: str
    mesh: trimesh.Trimesh
    group: str = field(default="frame")


def parts_by_group(parts: list[RackPart]) -> dict[str, list[RackPart]]:
    groups: dict[str, list[RackPart]] = {}
    for p in parts:
        groups.setdefault(p.group, []).append(p)
    return groups


# ---------------------------------------------------------------------------------------------
# primitives (explicit numpy faces — no trimesh.creation.*: the venv has trimesh 5.x while Kit
# bundles 4.11, and the creation API surface differs between them)
# ---------------------------------------------------------------------------------------------


def _basis_for(axis: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Orthonormal (u, v) perpendicular to ``axis``, world-axis-aligned for axis-aligned rods."""
    a = np.asarray(axis, dtype=float)
    a = a / np.linalg.norm(a)
    if abs(a[2]) > 0.999:  # vertical rod: ring in xy
        u = np.array([1.0, 0.0, 0.0])
    elif abs(a[0]) > 0.999:  # x rod: ring in yz
        u = np.array([0.0, 1.0, 0.0])
    elif abs(a[1]) > 0.999:  # y rod: ring in xz
        u = np.array([1.0, 0.0, 0.0])
    else:
        u = np.cross(np.array([0.0, 0.0, 1.0]), a)
        u = u / np.linalg.norm(u)
    v = np.cross(a, u)
    return u, v / np.linalg.norm(v)


def _ring_stack(
    centers: list[np.ndarray], radii: list[float], u: np.ndarray, v: np.ndarray, n: int
) -> trimesh.Trimesh:
    """Closed prism/frustum stack through ``centers`` with per-ring ``radii``.

    Convex whenever the radius profile is concave along the stack and the center path is
    straight or linearly sheared — which every caller guarantees.
    """
    angles = np.arange(n) * (2.0 * np.pi / n)
    ring_dirs = np.cos(angles)[:, None] * u[None, :] + np.sin(angles)[:, None] * v[None, :]
    k = len(centers)
    verts = np.concatenate(
        [np.asarray(c, dtype=float)[None, :] + r * ring_dirs for c, r in zip(centers, radii)]
        + [np.asarray(centers[0], dtype=float)[None, :], np.asarray(centers[-1], dtype=float)[None, :]]
    )
    cb, ct = k * n, k * n + 1
    faces = []
    for j in range(k - 1):
        a, b = j * n, (j + 1) * n
        for i in range(n):
            i2 = (i + 1) % n
            faces.append((a + i, a + i2, b + i2))
            faces.append((a + i, b + i2, b + i))
    for i in range(n):
        i2 = (i + 1) % n
        faces.append((cb, i2, i))  # bottom cap, faces -axis
        faces.append((ct, (k - 1) * n + i, (k - 1) * n + i2))  # top cap, faces +axis
    mesh = trimesh.Trimesh(vertices=verts, faces=np.asarray(faces, dtype=np.int64), process=False)
    if mesh.volume < 0:
        mesh.invert()
    return mesh


def _rod(p0, p1, dia: float, n: int) -> trimesh.Trimesh:
    """Straight wire segment (regular-n-gon prism) from ``p0`` to ``p1``."""
    p0 = np.asarray(p0, dtype=float)
    p1 = np.asarray(p1, dtype=float)
    u, v = _basis_for(p1 - p0)
    return _ring_stack([p0, p1], [dia / 2.0, dia / 2.0], u, v, n)


def _tine(base, h: float, dia: float, lean_deg: float, n: int) -> trimesh.Trimesh:
    """Vertical prong with a tapered cap, leaning toward +y (bowl / divider tines)."""
    base = np.asarray(base, dtype=float)
    r = dia / 2.0
    shear = math.tan(math.radians(lean_deg))

    def c(z_rel: float) -> np.ndarray:
        return base + np.array([0.0, shear * z_rel, z_rel])

    zs = (0.0, 0.72 * h, 0.90 * h, h)
    radii = [r, r, 0.62 * r, 0.30 * r]
    u = np.array([1.0, 0.0, 0.0])
    v = np.array([0.0, 1.0, 0.0])
    return _ring_stack([c(z) for z in zs], radii, u, v, n)


def _sheared_rod(base, z0: float, z1: float, dia: float, lean_deg: float, n: int) -> trimesh.Trimesh:
    """Straight vertical wire segment sheared by tan(lean)*z toward +y (tine shafts, beads)."""
    base = np.asarray(base, dtype=float)
    shear = math.tan(math.radians(lean_deg))
    u = np.array([1.0, 0.0, 0.0])
    v = np.array([0.0, 1.0, 0.0])
    c0 = base + np.array([0.0, shear * z0, z0])
    c1 = base + np.array([0.0, shear * z1, z1])
    return _ring_stack([c0, c1], [dia / 2.0, dia / 2.0], u, v, n)


def _arc_points(center_xy, radius: float, a0_deg: float, a1_deg: float, segments: int, z: float) -> np.ndarray:
    """Horizontal arc polyline (xy-plane) — perimeter corner arcs."""
    ang = np.radians(np.linspace(a0_deg, a1_deg, segments + 1))
    return np.stack(
        [center_xy[0] + radius * np.cos(ang), center_xy[1] + radius * np.sin(ang), np.full(segments + 1, z)],
        axis=1,
    )


def _arc_points_3d(center, radius: float, u, v, a0_deg: float, a1_deg: float, segments: int) -> np.ndarray:
    """Arc polyline in the plane spanned by (u, v) about ``center`` — canes, fillets, scallops."""
    center = np.asarray(center, dtype=float)
    u = np.asarray(u, dtype=float)
    v = np.asarray(v, dtype=float)
    ang = np.radians(np.linspace(a0_deg, a1_deg, segments + 1))
    return center[None, :] + radius * (np.cos(ang)[:, None] * u[None, :] + np.sin(ang)[:, None] * v[None, :])


def _block(center, extents, n_unused: int = 0) -> trimesh.Trimesh:
    """Axis-aligned box part (RackMatic lever caps, insert stop clip)."""
    c = np.asarray(center, dtype=float)
    e = np.asarray(extents, dtype=float) / 2.0
    signs = np.array(
        [[-1, -1, -1], [1, -1, -1], [1, 1, -1], [-1, 1, -1], [-1, -1, 1], [1, -1, 1], [1, 1, 1], [-1, 1, 1]],
        dtype=float,
    )
    verts = c[None, :] + signs * e[None, :]
    faces = np.array(
        [
            (0, 2, 1), (0, 3, 2),  # bottom (-z)
            (4, 5, 6), (4, 6, 7),  # top (+z)
            (0, 1, 5), (0, 5, 4),  # -y
            (2, 3, 7), (2, 7, 6),  # +y
            (1, 2, 6), (1, 6, 5),  # +x
            (3, 0, 4), (3, 4, 7),  # -x
        ],
        dtype=np.int64,
    )
    mesh = trimesh.Trimesh(vertices=verts, faces=faces, process=False)
    if mesh.volume < 0:
        mesh.invert()
    return mesh


# ---------------------------------------------------------------------------------------------
# derived layout shared by the builder, the probe specs, and the tests
# ---------------------------------------------------------------------------------------------


def _z_levels(p: dict) -> dict:
    r_heavy = p["wire_dia_heavy"] / 2.0
    r_load = p["wire_dia_load"] / 2.0
    r_light = p["wire_dia_light"] / 2.0
    ch = p.get("channel")
    z_runner = r_load + (ch["drop"] if ch else 0.0)  # channel runners sit at r_load (global z-min 0)
    z_cross = z_runner + r_load + r_light  # crossbars rest tangent on the runners
    return {
        "r_heavy": r_heavy,
        "r_load": r_load,
        "r_light": r_light,
        "z_runner": z_runner,
        # heavy outermost runners: top-aligned with the load runners where possible, but never
        # below z=0 (the upper rack has no channel, so bottom-aligning at r_heavy wins there)
        "z_runner_heavy": max(z_runner + r_load - r_heavy, r_heavy),
        "z_cross": z_cross,
        "z_bot_rail": max(z_runner, r_heavy),  # bottom perimeter rail center
        "floor_top": z_cross + r_light,  # what the object stands on; the slot-datum level
        "tine_base": z_cross + r_light,
    }


def _dense_runner_xs(p: dict) -> list[float]:
    """One light runner at each load-runner span midpoint (skipping the channel bridge)."""
    xs = []
    ch = p.get("channel")
    for a, b in zip(p["runner_xs"][:-1], p["runner_xs"][1:]):
        if ch and not (b < ch["x"][0] or a > ch["x"][1]):  # span overlaps the recessed channel
            continue
        xs.append((a + b) / 2.0)
    return xs


def _plate_tine_xs(p: dict) -> np.ndarray:
    x0, x1 = p["plate_tine_xspan"]
    n = int(round((x1 - x0) / p["plate_tine_pitch"])) + 1
    return x0 + np.arange(n) * p["plate_tine_pitch"]


def _bar_segments(p: dict, y: float, z: float, with_slopes: bool) -> list[tuple[np.ndarray, np.ndarray, str]]:
    """Segment list (p0, p1, zone) for one transverse bar at depth ``y``, wire centers at ``z``.

    Handles the recessed tracking channel (dip by ``channel.drop`` over short diagonal ramps)
    and, when ``with_slopes``, the drainage ramps attached to the side walls (``slope_sign``
    -1 = floor descends toward the wall, +1 = ascends).
    """
    W = p["footprint"][0]
    zl = _z_levels(p)
    x_lo, x_hi = zl["r_load"] + 0.0015, W - zl["r_load"] - 0.0015
    tan_s = math.tan(math.radians(p["slope_deg"])) * p.get("slope_sign", -1.0)
    slopes = sorted(p.get("slope_zones") or ()) if with_slopes else []
    ch = p.get("channel")
    ramp_run = 0.006

    def pt(x: float, zz: float) -> np.ndarray:
        return np.array([x, y, zz])

    events: list[tuple[float, float, str]] = [(max(s0, x_lo), min(s1, x_hi), "slope") for s0, s1 in slopes]
    if ch:
        events.append((ch["x"][0], ch["x"][1], "channel"))
    events.sort()

    segs: list[tuple[np.ndarray, np.ndarray, str]] = []
    cursor = x_lo
    for x0, x1, kind in events:
        if x0 > cursor + 0.002:  # skip degenerate slivers where a zone starts at the bar end
            segs.append((pt(cursor, z), pt(x0, z), "floor_open"))
        if kind == "slope":
            dz = tan_s * (x1 - x0)
            if (x0 + x1) / 2.0 < W / 2.0:  # left-wall zone: wall at x0 carries the offset
                segs.append((pt(x0, z + dz), pt(x1, z), "slope"))
            else:  # right-wall zone: wall at x1
                segs.append((pt(x0, z), pt(x1, z + dz), "slope"))
        else:
            drop = ch["drop"]
            segs.append((pt(x0, z), pt(x0 + ramp_run, z - drop), "channel"))
            segs.append((pt(x0 + ramp_run, z - drop), pt(x1 - ramp_run, z - drop), "channel"))
            segs.append((pt(x1 - ramp_run, z - drop), pt(x1, z), "channel"))
        cursor = x1
    if cursor < x_hi - 0.002:
        segs.append((pt(cursor, z), pt(x_hi, z), "floor_open"))
    return segs


def _front_rail_z(p: dict, x: float) -> float:
    """Front top-rail wire-center height at ``x`` (loading dip with diagonal transitions)."""
    zl = _z_levels(p)
    z_side = p["rim_side_h"] - zl["r_heavy"]
    dip = p.get("front_dip_x")
    if not dip:
        return z_side
    z_dip = p["rim_front_h"] - zl["r_heavy"]
    d0, d1 = dip
    run = 0.025
    if x <= d0 or x >= d1:
        return z_side
    if x < d0 + run:
        return z_side + (z_dip - z_side) * (x - d0) / run
    if x > d1 - run:
        return z_side + (z_dip - z_side) * (d1 - x) / run
    return z_dip


# ---------------------------------------------------------------------------------------------
# builder
# ---------------------------------------------------------------------------------------------


def _perimeter(parts: list[RackPart], p: dict) -> None:
    """Multi-height guard frame: heavy-gauge rail loops with rounded corners, a dipped front
    rail with a grab handle (lower rack), step rods at the rear-guard transition, balusters,
    rear mid guard rail."""
    W, D = p["footprint"]
    zl = _z_levels(p)
    n = p["wire_sides"]
    d_heavy, d_load, d_light = p["wire_dia_heavy"], p["wire_dia_load"], p["wire_dia_light"]
    r_w = zl["r_heavy"]  # rail centerline inset: outer wire surface exactly on the bbox edge
    R = p["corner_r"]
    m = p["corner_segments"]
    z_bot = zl["z_bot_rail"]
    z_side = p["rim_side_h"] - r_w
    z_rear = p["rim_rear_h"] - r_w
    y_split = p["rear_zone_y0"]
    ax = R + r_w  # arc-end offset from each corner

    corners = {  # arc center, sweep (degrees), wall -> wall
        "fl": ((ax, ax), 180.0, 270.0),
        "fr": ((W - ax, ax), 270.0, 360.0),
        "rr": ((W - ax, D - ax), 0.0, 90.0),
        "rl": ((ax, D - ax), 90.0, 180.0),
    }

    def chain(pts: np.ndarray, dia: float, zone: str, tag: str) -> None:
        for i in range(len(pts) - 1):
            parts.append(RackPart(f"{tag}_{i:02d}", zone, _rod(pts[i], pts[i + 1], dia, n)))

    def arc(key: str, z: float, dia: float, zone: str, tag: str) -> None:
        c, a0, a1 = corners[key]
        chain(_arc_points(c, R, a0, a1, m, z), dia, zone, tag)

    def loop_pieces(z: float, dia: float, zone: str, tag: str, which: str) -> None:
        if which in ("full", "front"):
            arc("fl", z, dia, zone, f"{tag}_arc_fl")
            arc("fr", z, dia, zone, f"{tag}_arc_fr")
            if which == "front" and p.get("front_dip_x"):
                # dipped loading rail: side-height stubs, diagonal drops, flat dip center
                d0, d1 = p["front_dip_x"]
                run = 0.025
                xs = (ax, d0, d0 + run, d1 - run, d1, W - ax)
                for i in range(len(xs) - 1):
                    z0, z1 = _front_rail_z(p, xs[i]), _front_rail_z(p, xs[i + 1])
                    parts.append(
                        RackPart(f"{tag}_front_{i}", zone, _rod((xs[i], r_w, z0), (xs[i + 1], r_w, z1), dia, n))
                    )
            else:
                parts.append(RackPart(f"{tag}_front", zone, _rod((ax, r_w, z), (W - ax, r_w, z), dia, n)))
        if which in ("full", "rear"):
            arc("rr", z, dia, zone, f"{tag}_arc_rr")
            arc("rl", z, dia, zone, f"{tag}_arc_rl")
            parts.append(RackPart(f"{tag}_rear", zone, _rod((ax, D - r_w, z), (W - ax, D - r_w, z), dia, n)))
        y0s, y1s = (ax, D - ax) if which == "full" else ((ax, y_split) if which == "front" else (y_split, D - ax))
        for side, x in (("l", r_w), ("r", W - r_w)):
            parts.append(RackPart(f"{tag}_side_{side}", zone, _rod((x, y0s, z), (x, y1s, z), dia, n)))

    loop_pieces(z_bot, d_heavy, "perimeter", "rail_bot", "full")
    loop_pieces(z_side, d_heavy, "perimeter", "rail_top_f", "front")
    loop_pieces(z_rear, d_heavy, "guard", "rail_top_r", "rear")
    # height-transition step rods where the rear guard begins
    for side, x in (("l", r_w), ("r", W - r_w)):
        parts.append(
            RackPart(f"rail_step_{side}", "guard", _rod((x, y_split, z_side), (x, y_split, z_rear), d_heavy, n))
        )
    # rear guard mid rail (retention rail halfway up the raised rear section)
    if p.get("guard_mid_rail"):
        z_mid = (z_bot + z_rear) / 2.0
        parts.append(
            RackPart("rail_mid_rear", "guard", _rod((ax, D - r_w, z_mid), (W - ax, D - r_w, z_mid), d_light, n))
        )
        for side, x in (("l", r_w), ("r", W - r_w)):
            parts.append(
                RackPart(f"rail_mid_{side}", "guard", _rod((x, y_split, z_mid), (x, D - ax, z_mid), d_light, n))
            )
    # center-front grab handle rising over the dipped rail (real racks: molded grip here)
    if p.get("handle"):
        hx0, hx1 = p["handle"]["x"]
        gz = p["handle"]["grip_z"]
        for i, hx in enumerate((hx0, hx1)):
            z0 = _front_rail_z(p, hx)
            parts.append(RackPart(f"handle_riser_{i}", "handle", _rod((hx, r_w, z0), (hx, r_w, gz), d_load, n)))
        parts.append(RackPart("handle_grip", "handle", _rod((hx0, r_w, gz), (hx1, r_w, gz), d_load, n)))

    # balusters (vertical infill wires; front ones follow the dip profile)
    pitch = p["baluster_pitch"]
    margin = ax + 0.004
    for i, x in enumerate(np.arange(margin, W - margin + 1e-9, pitch)):
        parts.append(
            RackPart(f"bal_f_{i:02d}", "perimeter", _rod((x, r_w, z_bot), (x, r_w, _front_rail_z(p, x)), d_light, n))
        )
        parts.append(RackPart(f"bal_r_{i:02d}", "guard", _rod((x, D - r_w, z_bot), (x, D - r_w, z_rear), d_light, n)))
    for i, y in enumerate(np.arange(margin, D - margin + 1e-9, pitch)):
        z_top = z_side if y < y_split else z_rear
        zone = "perimeter" if y < y_split else "guard"
        for side, x in (("l", r_w), ("r", W - r_w)):
            parts.append(RackPart(f"bal_s{side}_{i:02d}", zone, _rod((x, y, z_bot), (x, y, z_top), d_light, n)))


def _plate_bank(parts: list[RackPart], p: dict) -> None:
    """Rear plate bank: dense grid, V-ribs, two tine rows (candy-cane ends, beads, fillets,
    tie wires); the configured row becomes the dark fold-down INSERT group with spine/bosses/clip."""
    W, D = p["footprint"]
    zl = _z_levels(p)
    n = p["wire_sides"]
    d_light = p["wire_dia_light"]
    r_light = zl["r_light"]
    ch = p.get("channel")
    by0, by1 = p["plate_zone_y"]
    y_hi = D - zl["r_load"] - 0.0015
    tine_xs = _plate_tine_xs(p)
    lean = p["plate_tine_lean_deg"]
    shear = math.tan(math.radians(lean))
    h = p["plate_tine_h"]
    base_z = zl["tine_base"]
    insert_row = (p.get("insert") or {}).get("row", -1)

    # dense short runners (one per load-runner span)
    for di, x in enumerate(_dense_runner_xs(p)):
        parts.append(
            RackPart(
                f"dense_runner_{di:02d}",
                "floor_dense",
                _rod((x, by0, zl["z_runner"]), (x, min(by1, y_hi), zl["z_runner"]), d_light, n),
            )
        )
    # bank support bars (transverse; the rear zone is flat — no wall ramps)
    for j, y in enumerate(p["bank_bar_ys"]):
        for k, (p0, p1, zone) in enumerate(_bar_segments(p, y, zl["z_cross"], with_slopes=False)):
            parts.append(
                RackPart(f"bank_bar_{j}_{k:02d}", "floor_dense" if zone == "floor_open" else zone, _rod(p0, p1, d_light, n))
            )
    # corrugated cradle ribs: V-profile, peaks under the tines, troughs at the gap centers
    z_peak, z_trough = zl["z_cross"], zl["z_cross"] - 2.0 * p["rib_amplitude"]
    for r_i, y in enumerate(p["plate_rows_y"]):
        pts: list[tuple[float, float]] = []
        for k, xp in enumerate(tine_xs):
            pts.append((float(xp), z_peak))
            if k < len(tine_xs) - 1:
                pts.append((float(xp + tine_xs[k + 1]) / 2.0, z_trough))
        for k in range(len(pts) - 1):
            (xa, za), (xb, zb) = pts[k], pts[k + 1]
            parts.append(RackPart(f"rib_{r_i}_{k:02d}", "ribs", _rod((xa, y, za), (xb, y, zb), d_light, n)))

    # Tine rows: ends are candy canes, middles straight shafts + bead caps; every base
    # filleted. The cane is optional (None -> straight end tines): its inboard-curling hook
    # arcs INTO the adjacent gap and rejects disc goal poses there. Both v4 banks keep their
    # canes — measured on the robot bank: end gap 0 is behind the arm-access boundary
    # regardless, and end gap 2 still bakes accepted goals with the cane in place.
    cane = p.get("candy_cane")
    bead = p["tine_bead"]
    fil = p["tine_fillet"]
    ux, uz = np.array([1.0, 0.0, 0.0]), np.array([0.0, 0.0, 1.0])
    for r_i, y in enumerate(p["plate_rows_y"]):
        group = "insert" if r_i == insert_row else "frame"
        for k, x in enumerate(tine_xs):
            x = float(x)
            is_cane = cane is not None and k in (0, len(tine_xs) - 1)
            base = (x, y, base_z)
            if is_cane:
                r_c = cane["r"]
                # apex WIRE SURFACE lands exactly at h (bbox-top contract shared with the beads)
                shaft_top = h - r_c - r_light
                parts.append(
                    RackPart(f"cane_shaft_r{r_i}_c{k:02d}", "plate_tines", _sheared_rod(base, 0.0, shaft_top, d_light, lean, n), group)
                )
                inboard = 1.0 if k == 0 else -1.0
                center = np.array([x + inboard * r_c, y + shear * shaft_top, base_z + shaft_top])
                a0, a1 = (180.0, 180.0 - cane["sweep_deg"]) if inboard > 0 else (0.0, cane["sweep_deg"])
                pts3 = _arc_points_3d(center, r_c, ux, uz, a0, a1, cane["segments"])
                pts3[:, 1] += shear * (pts3[:, 2] - (base_z + shaft_top))  # keep the lean along the hook
                for s in range(len(pts3) - 1):
                    parts.append(
                        RackPart(f"cane_arc_r{r_i}_c{k:02d}_{s}", "plate_tines", _rod(pts3[s], pts3[s + 1], d_light, n), group)
                    )
            else:
                shaft_top = h - bead["len"]
                parts.append(
                    RackPart(f"plate_tine_r{r_i}_c{k:02d}", "plate_tines", _sheared_rod(base, 0.0, shaft_top, d_light, lean, n), group)
                )
                parts.append(
                    RackPart(
                        f"bead_r{r_i}_c{k:02d}",
                        "plate_tines",
                        _sheared_rod(base, h - bead["len"] - 0.002, h, d_light * bead["dia_factor"], lean, n),
                        group,
                    )
                )
            # base fillet: quarter arc curving DOWN from the shaft base into the floor grid
            # (real U-bend wires dive into the floor; curving upward would also pollute the
            # slot-datum percentile band above floor_top — the down-arc's top ring is exactly
            # perpendicular to z, so no fillet vertex exceeds the floor-top level)
            direction = 1.0 if k % 2 == 0 else -1.0
            f_c = np.array([x + direction * fil["r"], y, base_z])
            a0, a1 = (180.0, 270.0) if direction > 0 else (0.0, -90.0)
            fpts = _arc_points_3d(f_c, fil["r"], ux, uz, a0, a1, fil["segments"])
            for s in range(len(fpts) - 1):
                parts.append(
                    RackPart(f"fillet_r{r_i}_c{k:02d}_{s}", "plate_tines", _rod(fpts[s], fpts[s + 1], d_light, n), group)
                )
        # Mid-height tie wire threaded through the row. Optional (None skips): the tie crosses
        # every gap THROUGH the standing disc's plane, so a robot-loaded bank cannot have one —
        # measured 2026-08-09: it single-handedly rejected 100% of plate goal poses. Fill-only
        # rear banks keep it for realism (teleported discs simply rest on it).
        if p.get("tine_tie_frac") is not None:
            tz = base_z + p["tine_tie_frac"] * h
            ty = y + shear * p["tine_tie_frac"] * h
            parts.append(
                RackPart(
                    f"tine_tie_{r_i}",
                    "tie",
                    _rod((float(tine_xs[0]), ty, tz), (float(tine_xs[-1]), ty, tz), d_light, n),
                    group,
                )
            )

    # fold-down insert hardware (dark second mesh): spine rod, hinge bosses, stop clip
    ins = p.get("insert")
    if ins is not None:
        y_row = p["plate_rows_y"][ins["row"]]
        spine_z = base_z + ins["spine_dia"] / 2.0
        x0, x1 = p["plate_tine_xspan"]
        parts.append(
            RackPart("insert_spine", "insert", _rod((x0, y_row, spine_z), (x1, y_row, spine_z), ins["spine_dia"], n), "insert")
        )
        for side, bx in (("l", x0 - ins["boss_len"]), ("r", x1)):
            parts.append(
                RackPart(
                    f"insert_boss_{side}",
                    "insert",
                    _rod((bx, y_row, spine_z), (bx + ins["boss_len"], y_row, spine_z), ins["boss_dia"], n),
                    "insert",
                )
            )
        parts.append(
            RackPart("insert_clip", "insert", _block((x0 + 0.008, y_row - 0.010, spine_z), ins["clip_size"]), "insert")
        )


def _cutlery_basket(parts: list[RackPart], p: dict) -> None:
    """3-compartment plastic cutlery basket in the lower rack's open zone (v3).

    Rides the rack (fixed part of the rack body mesh, group ``basket`` -> own USD mesh +
    material): a free-standing basket would skate during the 0.2 m rack slide. Convex parts
    only: floor slab, 4 walls, 2 y-dividers (3 bays), and an arch carry handle (2 posts +
    bar). The floor slab contributes 8 vertices to the bottom 15 mm percentile band —
    negligible against the thousands of floor-wire vertices, so the slot datum contract
    holds (asserted in tests).
    """
    b = p["basket"]
    zl = _z_levels(p)
    x0, x1 = b["x"]
    y0, y1 = b["y"]
    z0 = zl["floor_top"]
    h, t, ft = b["h"], b["wall_t"], b["floor_t"]
    n = p["wire_sides"]
    cx = (x0 + x1) / 2.0

    parts.append(RackPart("basket_floor", "basket", group="basket", mesh=_block(((x0 + x1) / 2.0, (y0 + y1) / 2.0, z0 + ft / 2.0), (x1 - x0, y1 - y0, ft))))
    z_wall = z0 + ft + (h - ft) / 2.0
    wall_h = h - ft
    parts.append(RackPart("basket_wall_x0", "basket", group="basket", mesh=_block((x0 + t / 2.0, (y0 + y1) / 2.0, z_wall), (t, y1 - y0, wall_h))))
    parts.append(RackPart("basket_wall_x1", "basket", group="basket", mesh=_block((x1 - t / 2.0, (y0 + y1) / 2.0, z_wall), (t, y1 - y0, wall_h))))
    parts.append(RackPart("basket_wall_y0", "basket", group="basket", mesh=_block(((x0 + x1) / 2.0, y0 + t / 2.0, z_wall), (x1 - x0 - 2 * t, t, wall_h))))
    parts.append(RackPart("basket_wall_y1", "basket", group="basket", mesh=_block(((x0 + x1) / 2.0, y1 - t / 2.0, z_wall), (x1 - x0 - 2 * t, t, wall_h))))
    cy = (y0 + y1) / 2.0
    # The arch carry handle is optional (side-grip baskets). Measured 2026-08-09: an arch bar
    # over rotated (x-split) bays runs along every drop column's dodge axis and leaves no
    # collision-free drop window for cutlery — a rotated robot-loaded basket must go handleless.
    hd = b.get("handle")
    z_top = z0 + h
    if "dividers_x" in b:
        # rotated variant: bays split along x — keeps every drop column at the basket's
        # front-band y, where the fork-drop stack clears the machine mouth (measured: bays at
        # y >= 0.126 are capped by the shell top)
        for di, dx in enumerate(b["dividers_x"]):
            parts.append(RackPart(f"basket_divider_{di}", "basket", group="basket", mesh=_block((dx, cy, z_wall), (t, y1 - y0 - 2 * t, wall_h))))
        if hd is not None:
            z_bar = z_top + hd["clearance"]
            parts.append(RackPart("basket_handle_post_0", "basket_handle", group="basket", mesh=_rod((x0 + t / 2.0, cy, z_top - 0.010), (x0 + t / 2.0, cy, z_bar), hd["post_dia"], n)))
            parts.append(RackPart("basket_handle_post_1", "basket_handle", group="basket", mesh=_rod((x1 - t / 2.0, cy, z_top - 0.010), (x1 - t / 2.0, cy, z_bar), hd["post_dia"], n)))
            parts.append(RackPart("basket_handle_bar", "basket_handle", group="basket", mesh=_rod((x0 + t / 2.0, cy, z_bar), (x1 - t / 2.0, cy, z_bar), hd["bar_dia"], n)))
        return
    for di, dy in enumerate(b["dividers_y"]):
        parts.append(RackPart(f"basket_divider_{di}", "basket", group="basket", mesh=_block(((x0 + x1) / 2.0, dy, z_wall), (x1 - x0 - 2 * t, t, wall_h))))
    if hd is None:
        return
    z_bar = z_top + hd["clearance"]
    # arch carry handle over the y-centerline (posts at the two end walls, bar along y)
    parts.append(RackPart("basket_handle_post_0", "basket_handle", group="basket", mesh=_rod((cx, y0 + t / 2.0, z_top - 0.010), (cx, y0 + t / 2.0, z_bar), hd["post_dia"], n)))
    parts.append(RackPart("basket_handle_post_1", "basket_handle", group="basket", mesh=_rod((cx, y1 - t / 2.0, z_top - 0.010), (cx, y1 - t / 2.0, z_bar), hd["post_dia"], n)))
    parts.append(RackPart("basket_handle_bar", "basket_handle", group="basket", mesh=_rod((cx, y0 + t / 2.0, z_bar), (cx, y1 - t / 2.0, z_bar), hd["bar_dia"], n)))


def basket_bays(params: dict) -> list[tuple[float, float, float, float]]:
    """Interior (x0, x1, y0, y1) of each basket compartment (design frame)."""
    b = params["basket"]
    t = b["wall_t"]
    if "dividers_x" in b:
        y0, y1 = b["y"][0] + t, b["y"][1] - t
        lo_edges = [b["x"][0] + t] + [d + t / 2.0 for d in b["dividers_x"]]
        hi_edges = [d - t / 2.0 for d in b["dividers_x"]] + [b["x"][1] - t]
        return [(lo, hi, y0, y1) for lo, hi in zip(lo_edges, hi_edges)]
    x0, x1 = b["x"][0] + t, b["x"][1] - t
    lo_edges = [b["y"][0] + t] + [d + t / 2.0 for d in b["dividers_y"]]
    hi_edges = [d - t / 2.0 for d in b["dividers_y"]] + [b["y"][1] - t]
    return [(x0, x1, lo, hi) for lo, hi in zip(lo_edges, hi_edges)]


def basket_probes(params: dict) -> list[tuple[tuple[float, float, float], tuple[float, float, float]]]:
    """A cutlery-sized box inside each basket bay (must be FCL-free), inset 4 mm from the
    bay walls and spanning from just above the basket floor to just under the handle bar."""
    b = params["basket"]
    zl = _z_levels(params)
    z0 = zl["floor_top"] + b["floor_t"] + 0.003
    z1 = zl["floor_top"] + b["h"] - 0.003
    probes = []
    for x0, x1, y0, y1 in basket_bays(params):
        ext = (x1 - x0 - 0.008, y1 - y0 - 0.008, z1 - z0)
        probes.append((ext, ((x0 + x1) / 2.0, (y0 + y1) / 2.0, (z0 + z1) / 2.0)))
    return probes


def basket_divider_negative_probe(params: dict) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    """A box straddling the first divider — MUST collide (piece-transform sanity control)."""
    b = params["basket"]
    zl = _z_levels(params)
    z_c = zl["floor_top"] + b["h"] / 2.0
    if "dividers_x" in b:
        cy = (b["y"][0] + b["y"][1]) / 2.0
        return (0.020, 0.030, 0.030), (float(b["dividers_x"][0]), cy, z_c)
    cx = (b["x"][0] + b["x"][1]) / 2.0
    return (0.030, 0.020, 0.030), (cx, float(b["dividers_y"][0]), z_c)


def open_zones_effective(params: dict) -> list[tuple[float, float, float, float]]:
    """Open zones clipped against the basket footprint (standing-slot feasibility space)."""
    zones = list(params["open_zones"])
    b = params.get("basket")
    if not b:
        return zones
    bx0 = b["x"][0] - 0.002
    out = []
    for x0, x1, y0, y1 in zones:
        if x1 > bx0 >= x0:
            x1 = bx0
        if x1 - x0 > 0.02:
            out.append((x0, x1, y0, y1))
    return out


def _upper_features(parts: list[RackPart], p: dict) -> None:
    """Upper-rack signatures: fold-down cup shelves with stemware scallops over the slope
    zones, and RackMatic lever end-cap blocks on the side rails."""
    W, D = p["footprint"]
    zl = _z_levels(p)
    n = p["wire_sides"]
    d_light = p["wire_dia_light"]
    r_w = zl["r_heavy"]

    cs = p.get("cup_shelf")
    if cs:
        y0, y1 = cs["y_span"]
        drop = cs["depth"] * math.tan(math.radians(cs["tilt_deg"]))
        z_hinge, z_inner = cs["mount_z"], cs["mount_z"] - drop
        for side in ("l", "r"):
            hinge_x = r_w + 0.0015 if side == "l" else W - r_w - 0.0015
            inner_x = hinge_x + cs["depth"] if side == "l" else hinge_x - cs["depth"]
            parts.append(
                RackPart(f"shelf_{side}_hinge", "cup_shelf", _rod((hinge_x, y0, z_hinge), (hinge_x, y1, z_hinge), d_light, n))
            )
            parts.append(
                RackPart(f"shelf_{side}_edge", "cup_shelf", _rod((inner_x, y0, z_inner), (inner_x, y1, z_inner), d_light, n))
            )
            for i, y in enumerate(np.linspace(y0, y1, cs["rungs"])):
                parts.append(
                    RackPart(f"shelf_{side}_rung_{i}", "cup_shelf", _rod((hinge_x, y, z_hinge), (inner_x, y, z_inner), d_light, n))
                )
            for i, y in enumerate(np.linspace(y0, y1, cs["scallops"] + 2)[1:-1]):
                pts = _arc_points_3d(
                    (inner_x, y, z_inner), cs["scallop_r"], np.array([0.0, 1.0, 0.0]), np.array([0.0, 0.0, 1.0]), 180.0, 0.0, 3
                )
                for s in range(len(pts) - 1):
                    parts.append(RackPart(f"shelf_{side}_scallop_{i}_{s}", "cup_shelf", _rod(pts[s], pts[s + 1], d_light, n)))
            rail_x = r_w if side == "l" else W - r_w
            stub_end = hinge_x + 0.002 if side == "l" else hinge_x - 0.002
            for i, y in enumerate((y0, y1)):  # hinge stubs to the side rail
                parts.append(
                    RackPart(f"shelf_{side}_stub_{i}", "cup_shelf", _rod((rail_x, y, z_hinge), (stub_end, y, z_hinge), d_light, n))
                )

    rb = p.get("rackmatic_blocks")
    if rb:
        ex = rb["size"]
        for side in ("l", "r"):
            cx = ex[0] / 2.0 if side == "l" else W - ex[0] / 2.0
            for i, y in enumerate(rb["ys"]):
                parts.append(RackPart(f"rackmatic_{side}_{i}", "rackmatic", _block((cx, y, rb["z"]), ex)))


def build_rack(params: dict) -> list[RackPart]:
    """Build every wire part of one rack in design space. Deterministic; every part convex."""
    p = params
    W, D = p["footprint"]
    zl = _z_levels(p)
    n = p["wire_sides"]
    d_heavy, d_load, d_light = p["wire_dia_heavy"], p["wire_dia_load"], p["wire_dia_light"]
    parts: list[RackPart] = []
    y_lo, y_hi = zl["r_load"] + 0.0015, D - zl["r_load"] - 0.0015
    ch = p.get("channel")

    # -- floor: depth-axis load runners; the two outermost are heavy gauge ---------------------
    last = len(p["runner_xs"]) - 1
    for i, x in enumerate(p["runner_xs"]):
        heavy = i in (0, last)
        z = zl["z_runner_heavy"] if heavy else zl["z_runner"]
        parts.append(
            RackPart(f"runner_{i:02d}", "floor_open", _rod((x, y_lo, z), (x, y_hi, z), d_heavy if heavy else d_load, n))
        )
    if ch:
        for i, x in enumerate(ch["runner_xs"]):
            parts.append(
                RackPart(f"channel_runner_{i}", "channel", _rod((x, y_lo, zl["r_load"]), (x, y_hi, zl["r_load"]), d_load, n))
            )

    # -- floor: transverse crossbars (light) with channel dip + wall ramps ---------------------
    for j, y in enumerate(p["crossbar_ys"]):
        for k, (p0, p1, zone) in enumerate(_bar_segments(p, y, zl["z_cross"], with_slopes=True)):
            parts.append(RackPart(f"cross_{j}_{k:02d}", zone, _rod(p0, p1, d_light, n)))
    # longitudinal support rod mid-ramp so tilted drinkware rests on two lines
    for si, (s0, s1) in enumerate(p.get("slope_zones") or ()):
        xm = (s0 + s1) / 2.0
        run = (s1 - xm) if xm < W / 2.0 else (xm - s0)
        z = zl["z_cross"] + p.get("slope_sign", -1.0) * math.tan(math.radians(p["slope_deg"])) * run
        y0s, y1s = min(p["crossbar_ys"]) - 0.008, max(p["crossbar_ys"]) + 0.008
        parts.append(RackPart(f"slope_runner_{si}", "slope", _rod((xm, y0s, z), (xm, y1s, z), d_light, n)))

    # -- rear plate bank (lower rack) -----------------------------------------------------------
    if "plate_rows_y" in p:
        _plate_bank(parts, p)
    if p.get("plate_bank2"):
        # A second, independently-parameterized tine bank (e.g. a fill-only rear bank behind a
        # robot-facing front bank). Its sub-dict shadows the plate_* keys of the main bank;
        # set "insert"/"tine_tie_frac" explicitly to control that hardware per bank.
        _plate_bank(parts, {**p, **p["plate_bank2"]})

    # -- cutlery basket (lower rack, v3) ---------------------------------------------------------
    if "basket" in p:
        _cutlery_basket(parts, p)

    # -- bowl tines (sparser, shorter) -----------------------------------------------------------
    if "bowl_tine_xs" in p:
        for xi, x in enumerate(p["bowl_tine_xs"]):
            for yi, y in enumerate(p["bowl_tine_ys"]):
                parts.append(
                    RackPart(f"bowl_tine_{xi}_{yi}", "bowl_tines", _tine((x, y, zl["tine_base"]), p["bowl_tine_h"], d_light, 0.0, n))
                )

    # -- upper-rack center divider (glass row) ---------------------------------------------------
    if "divider_tine_x" in p:
        for yi, y in enumerate(p["divider_tine_ys"]):
            parts.append(
                RackPart(
                    f"divider_tine_{yi}", "divider_tines", _tine((p["divider_tine_x"], y, zl["tine_base"]), p["divider_tine_h"], d_light, 0.0, n)
                )
            )

    # -- roller wheels: outer face exactly on the footprint plane, bottoms exactly z=0 ----------
    wh = p.get("wheels")
    if wh:
        r = wh["dia"] / 2.0
        for side in ("l", "r"):
            x0, x1 = (0.0, wh["width"]) if side == "l" else (W - wh["width"], W)
            hx0, hx1 = (wh["width"], wh["width"] + 0.012) if side == "l" else (W - wh["width"] - 0.012, W - wh["width"])
            for i, y in enumerate((wh["y_inset"], D - wh["y_inset"])):
                parts.append(RackPart(f"wheel_{side}_{i}", "wheels", _rod((x0, y, r), (x1, y, r), wh["dia"], n)))
                parts.append(RackPart(f"wheel_hub_{side}_{i}", "wheels", _rod((hx0, y, r), (hx1, y, r), 0.010, n)))

    _upper_features(parts, p)
    _perimeter(parts, p)
    return parts


def merged_mesh(parts: list[RackPart]) -> trimesh.Trimesh:
    """All parts (both groups) concatenated into one triangle-soup mesh."""
    return trimesh.util.concatenate([p.mesh for p in parts])


def mesh_arrays_usd(parts: list[RackPart], scale_x: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """(points, faceVertexCounts, faceVertexIndices) for USD authoring under the scaled Xform.

    The rack body Xform's non-uniform x-scale is inherited by children and baked back in at
    Phase-D extraction, so authored x-coordinates are pre-divided by ``scale_x`` here: the
    world-space (and extracted) geometry then equals the design geometry exactly. Callers pass
    a group-filtered part list to author the frame and the insert as separate meshes.
    """
    merged = merged_mesh(parts)
    points = np.asarray(merged.vertices, dtype=np.float64).copy()
    points[:, 0] /= float(scale_x)
    faces = np.asarray(merged.faces, dtype=np.int32)
    counts = np.full(len(faces), 3, dtype=np.int32)
    return points.astype(np.float32), counts, faces.reshape(-1)


def params_hash(rack_gen_cfg: dict, version: int) -> str:
    """Stable digest of the full generator config — the derived-USD staleness stamp."""
    payload = {"rack_gen": rack_gen_cfg, "version": version}
    return hashlib.sha256(json.dumps(payload, sort_keys=True).encode()).hexdigest()[:16]


# ---------------------------------------------------------------------------------------------
# probe specs (geometry only; FCL execution lives with the consumers)
# ---------------------------------------------------------------------------------------------


def floor_top_z(params: dict) -> float:
    """Top of the open-zone floor wires (crossbar tops).

    For the LOWER rack this is also the level placement.py's 95th-percentile slot datum lands
    on (asserted in tests). The upper rack's ascending wall ramps sit higher inside the 15 mm
    percentile band, so its datum would come out a few mm above this — nothing derives an
    upper-rack datum today, but don't extend slot derivation there without revisiting that.
    """
    return _z_levels(params)["floor_top"]


def mug_probes(params: dict) -> list[tuple[tuple[float, float, float], tuple[float, float, float]]]:
    """(extents, center) standing-object boxes, one per open zone, long side along the zone's
    long axis.

    The mug asset lies on its side in the object frame: bbox half index 1 is the ALONG-AXIS
    half, so the standing footprint comes from halves 0 (handle direction) and 2 (body
    diameter), matching placement.py's footprint formula. Each horizontal face carries the
    5 mm carried-hull inflation the planner applies (statics are uninflated), and the box
    bottom sits below ``floor_top + RELEASE_HOVER_M`` by that same margin — exactly bounding
    what goal_configs releases — replacing the old bbox-center/flat-floor probe that tines break.
    """
    margin = config.COLLISION_MARGIN_M
    long_e = 2.0 * (config.OBJECT_BBOX_HALF[0] + margin)
    short_e = 2.0 * (config.OBJECT_BBOX_HALF[2] + margin)
    h = config.OBJECT_HEIGHT_M + 2.0 * margin
    z_c = floor_top_z(params) + config.RELEASE_HOVER_M + config.OBJECT_HEIGHT_M / 2.0
    probes = []
    for x0, x1, y0, y1 in open_zones_effective(params):  # v3: clipped against the basket
        ext = (long_e, short_e, h) if (x1 - x0) >= (y1 - y0) else (short_e, long_e, h)
        probes.append((ext, ((x0 + x1) / 2.0, (y0 + y1) / 2.0, z_c)))
    return probes


def plate_gap_probes(params: dict, count: int = 3) -> list[tuple[tuple[float, float, float], tuple[float, float, float]]]:
    """Thin plate-shaped boxes centered in sampled gaps between adjacent plate tines (must be
    free). Sized for the v2 bank: the 52 mm sheared inter-row corridor bounds y, and the box
    top stays below the mid-height tie wires."""
    xs = _plate_tine_xs(params)
    gaps = (xs[:-1] + xs[1:]) / 2.0
    idx = [len(gaps) // 4, len(gaps) // 2, (3 * len(gaps)) // 4][:count]
    y_c = float(np.mean(params["plate_rows_y"])) + 0.003  # shear-centered between the rows
    z0 = floor_top_z(params) + 0.002
    ext = (0.010, 0.040, 0.036)
    return [(ext, (float(gaps[i]), y_c, z0 + ext[2] / 2.0)) for i in idx]


def plate_tine_negative_probe(params: dict) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    """A box centered ON a tine (mid-column of the FRONT row, lean-tracked) — MUST collide
    (piece-transform sanity control; with the 52 mm row split the inter-row corridor no longer
    touches any tine, so the control sits on the row itself)."""
    xs = _plate_tine_xs(params)
    z0 = floor_top_z(params) + 0.002
    ext = (0.010, 0.020, 0.036)
    z_c = z0 + ext[2] / 2.0
    shear = math.tan(math.radians(params["plate_tine_lean_deg"]))
    y_c = params["plate_rows_y"][0] + shear * (z_c - _z_levels(params)["tine_base"])
    return ext, (float(xs[len(xs) // 2]), float(y_c), z_c)


# ---------------------------------------------------------------------------------------------
# preview (venv-side evidence; matplotlib imported lazily)
# ---------------------------------------------------------------------------------------------


def preview_png(parts: list[RackPart], out_png: str, title: str) -> None:
    """Zone-colored 3-view render (iso/top/side), same style as the Phase-D overlays."""
    import os  # noqa: PLC0415

    import matplotlib  # noqa: PLC0415

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt  # noqa: PLC0415
    from matplotlib.patches import Patch  # noqa: PLC0415
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection  # noqa: PLC0415

    merged = merged_mesh(parts)
    mn, mx = merged.bounds
    zones_present = sorted({p.zone for p in parts})
    fig = plt.figure(figsize=(17, 5.8))
    for k, (elev, azim, label) in enumerate([(24, -55, "iso"), (88, -90, "top"), (4, -90, "side")]):
        ax = fig.add_subplot(1, 3, k + 1, projection="3d")
        for part in parts:
            ax.add_collection3d(
                Poly3DCollection(
                    part.mesh.vertices[part.mesh.faces],
                    facecolor=ZONE_COLORS.get(part.zone, "#999999"),
                    edgecolor="none",
                    alpha=0.9,
                )
            )
        ax.set_xlim(mn[0], mx[0])
        ax.set_ylim(mn[1], mx[1])
        ax.set_zlim(mn[2], mx[2])
        ax.set_box_aspect(tuple(mx - mn))
        ax.view_init(elev=elev, azim=azim)
        ax.set_title(f"{title} — {label} ({len(parts)} parts)")
        ax.set_axis_off()
    fig.legend(
        handles=[Patch(facecolor=ZONE_COLORS.get(z, "#999999"), label=z) for z in zones_present],
        loc="lower center",
        ncol=min(len(zones_present), 10),
        frameon=False,
    )
    fig.tight_layout(rect=(0, 0.05, 1, 1))
    os.makedirs(os.path.dirname(out_png), exist_ok=True)
    fig.savefig(out_png, dpi=110)
    plt.close(fig)
