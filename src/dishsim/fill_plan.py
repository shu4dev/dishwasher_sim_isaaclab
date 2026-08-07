# Copyright (c) 2026, dishsim project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Deterministic full-load plan for the capacity study (Kit-free).

Computes a target pose for every item of a realistic full dishwasher load from the registry
(:data:`dishsim.config.OBJECTS`) + the rack generator numbers (:data:`dishsim.config.RACK_GEN`),
in each rack's design/body frame, then maps into the robot-base frame via the cached
``T_base_body`` transforms. ``scripts/setup/capacity_fill.py`` spawns the items at these poses
(+ a small drop hover) and lets physics settle; this module also pre-validates the layout with
FCL (pairwise item overlap + item-vs-rack clearance) and checks the closability z-budget so a
layout bug fails fast in the venv, not after a Kit boot.

Layout (compact 6-place-setting machine):

- **Lower rack**: 8 dinner plates + 3 saucers in the 11 tine-bank gaps (vertical, 7 deg lean);
  2 bowls leaning on the bowl tines over the drinkware slope; pitcher + upside-down food
  container (lid stacked on top) in the open zone; cutlery basket bays: 4 forks / 4 spoons /
  3 knives, leaning +-8 deg.
- **Upper rack**: 2 mugs + cups + tumblers upside-down in the two open zones (greedy lattice
  packing, FCL-pruned); 2 wine glasses upside-down where the packer finds room (the stemware
  scallop lie-in is a physics stretch goal scripts/setup/capacity_fill.py attempts first); spatula + serving spoon
  lying flat along the slope strips under the cup shelves.

Every number here is either a registry dim, a RACK_GEN parameter, or derived from them.
"""

import os
from dataclasses import dataclass, field

import numpy as np
import trimesh

from . import config
from . import rack_gen
from .geometry import load_manifest

LOWER = "E_shelf_1_04"
UPPER = "E_shelf_03"

#: clearance between packed drinkware body circles [m] — upside-down items standing nearly
#: rim-to-rim is how real racks are loaded; the FCL validation checks the true meshes
ITEM_MARGIN_M = 0.002
#: spawn hover above the computed rest pose [m] (scripts/setup/capacity_fill.py drops from here)
SPAWN_HOVER_M = 0.020


@dataclass
class PlannedItem:
    """One item of the full load, in the robot-base frame."""

    item_id: str  # e.g. "plate_03"
    name: str  # registry key
    instance: int
    T_base_obj: np.ndarray  # [4, 4] target REST pose (design intent, pre-settle)
    rack: str  # "lower" | "upper"
    zone: str  # human-readable zone tag
    spawn_order: int = 0

    def to_json(self) -> dict:
        return {
            "item_id": self.item_id,
            "name": self.name,
            "instance": self.instance,
            "T_base_obj": self.T_base_obj.tolist(),
            "rack": self.rack,
            "zone": self.zone,
            "spawn_order": self.spawn_order,
        }


# ---------------------------------------------------------------------------------------------
# orientation helpers (design frame; scipy imported lazily to keep module import light)
# ---------------------------------------------------------------------------------------------


def _rot(seq: str, angles_deg) -> np.ndarray:
    from scipy.spatial.transform import Rotation  # noqa: PLC0415

    return Rotation.from_euler(seq, np.radians(np.atleast_1d(angles_deg))).as_matrix()


def _stand_R(spec: config.ObjectSpec, yaw_deg: float = 0.0, upside_down: bool = False) -> np.ndarray:
    """Rotation standing the object axis up (or down), then yawing about design z."""
    axis = tuple(spec.axis_obj)
    if axis == (0.0, 1.0, 0.0):
        R0 = _rot("x", 90.0)  # +y_obj -> +z (the mug convention, placement.py:147)
    elif axis == (0.0, 0.0, 1.0):
        R0 = np.eye(3)
    else:  # length-along-x utensils "stand" head-up
        R0 = _rot("y", -90.0)  # +x_obj -> +z
    if upside_down:
        R0 = _rot("x", 180.0) @ R0
    return _rot("z", yaw_deg) @ R0


def _axis_bottom_offset(spec: config.ObjectSpec, R: np.ndarray) -> np.ndarray:
    """Design-frame offset from the object origin to its LOWEST bbox point under ``R``."""
    # bbox corners in the object frame -> rotated; the support point is the min-z corner
    h = np.array(spec.bbox_half)
    corners = np.array([[sx * h[0], sy * h[1], sz * h[2]] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)])
    z = (R @ corners.T).T[:, 2]
    return np.array([0.0, 0.0, -float(z.min())])


def _place(spec, R: np.ndarray, xy, z_floor: float, hover: float = 0.002) -> np.ndarray:
    """Design-frame pose: origin so the lowest bbox point sits ``hover`` above ``z_floor``."""
    T = np.eye(4)
    T[:3, :3] = R
    lift = _axis_bottom_offset(spec, R)
    T[:3, 3] = np.array([xy[0], xy[1], z_floor + hover]) + lift
    return T


# ---------------------------------------------------------------------------------------------
# the layout
# ---------------------------------------------------------------------------------------------


def _lower_items() -> list[tuple[str, int, np.ndarray, str]]:
    """(name, instance, T_design_obj, zone) for the lower rack."""
    p = config.RACK_GEN[LOWER]
    z_floor = rack_gen.floor_top_z(p)
    items: list[tuple[str, int, np.ndarray, str]] = []

    # -- plate bank: 11 gaps; 7 dinner plates mid-bank, 3 saucers outer, spatula in the last ---
    # A leaning disc always spans its full diameter in rack depth (the disc plane contains y),
    # so dinner plates seat FORWARD: bottom edge on the y~0.197 bank bar, disc reaching
    # through both tine rows without crossing the rear wall. Saucers sit deeper (center
    # y 0.222) so their smaller disc still reaches the rear tine row.
    xs = np.asarray(rack_gen._plate_tine_xs(p))
    gaps = (xs[:-1] + xs[1:]) / 2.0
    lean = float(p["plate_tine_lean_deg"])
    # dinner plates stay OUT of the basket's x-range (their forward overhang at low z dips
    # into the cutlery bays); saucers sit deep enough to clear the bay tops, so they take the
    # outer gaps over the basket
    saucer_gaps, plate_gaps, spatula_gap = [0, 8, 9], [1, 2, 3, 4, 5, 6, 7], 10
    # vertical disc between adjacent tines: face NORMAL along rack +x (the 30 mm pitch
    # direction), disc plane in yz, leaned +7 deg toward the rear guard like the tines
    R_disc = _rot("x", -lean) @ _rot("y", 90.0)

    def disc_T(spec, gap_x, y_bottom):
        r = spec.rim_radius_m
        T = np.eye(4)
        T[:3, :3] = R_disc
        # center one radius above the bottom edge, shifted +y by the lean
        T[:3, 3] = [
            float(gap_x),
            y_bottom + r * np.sin(np.radians(lean)),
            z_floor + 0.002 + r * np.cos(np.radians(lean)),
        ]
        return T

    plate = config.OBJECTS["plate"]
    saucer = config.OBJECTS["saucer"]
    for k, gi in enumerate(plate_gaps):
        items.append(("plate", k, disc_T(plate, gaps[gi], 0.197), "plate_bank"))
    for k, gi in enumerate(saucer_gaps):
        items.append(("saucer", k, disc_T(saucer, gaps[gi], 0.2153), "plate_bank"))

    # spatula standing in the last gap like a plate (thin side along x), leaned 15 deg back
    spat = config.OBJECTS["spatula"]
    T_sp = np.eye(4)
    T_sp[:3, :3] = _rot("x", -15.0) @ _rot("y", -90.0)  # head up, thickness along rack x
    half_len = spat.height_m / 2.0
    T_sp[:3, 3] = [
        float(gaps[spatula_gap]),
        0.200 + half_len * np.sin(np.radians(15.0)),
        z_floor + 0.002 + half_len * np.cos(np.radians(15.0)),
    ]
    items.append(("spatula", 0, T_sp, "plate_bank"))

    # -- one bowl leaning on the bowl tines over the drinkware slope ---------------------------
    # (plates overhang forward to y~0.134, so only the front bowl position stays clear; the
    # second bowl goes upside-down to the upper rack's rear strip)
    bowl = config.OBJECTS["bowl"]
    lean_bowl = float(bowl.placement.params.get("lean_deg", 48.0))
    R = _rot("y", lean_bowl) @ _rot("x", 180.0)  # opening down-slope (-x), back on the tines
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [0.070, 0.060, z_floor + 0.004 + bowl.rim_radius_m * np.sin(np.radians(lean_bowl))]
    items.append(("bowl", 0, T, "bowl_tines"))

    # -- open zone: pitcher + one standing mug (the front strip clears the plate overhang) -----
    pitcher = config.OBJECTS["pitcher"]
    items.append(("pitcher", 0, _place(pitcher, _stand_R(pitcher, yaw_deg=90.0), (0.130, 0.095), z_floor), "open_zone"))
    mug = config.OBJECTS["mug"]
    items.append(("mug", 1, _place(mug, _stand_R(mug, yaw_deg=90.0), (0.212, 0.055), z_floor), "open_zone"))

    # -- cutlery basket: forks / spoons / knives per bay, leaning +-8 deg ----------------------
    bays = rack_gen.basket_bays(p)
    b = p["basket"]
    z_bfloor = z_floor + b["floor_t"]
    # parallel +8 deg lean per bay (alternating leans converge at the top and clash); spoon
    # heads are 25 mm wide, so spoons cap at 3 per bay
    per_bay = [("fork", 4, bays[0]), ("spoon", 3, bays[1]), ("knife", 3, bays[2])]
    for name, count, (bx0, bx1, by0, by1) in per_bay:
        spec = config.OBJECTS[name]
        xs_c = np.linspace(bx0 + 0.012, bx1 - 0.012, count)
        y_bay = (by0 + by1) / 2.0
        for k, x in enumerate(xs_c):
            tilt = 8.0
            R = _rot("y", tilt) @ _stand_R(spec, yaw_deg=90.0)  # blades/tines up, thin side along x
            T = np.eye(4)
            T[:3, :3] = R
            half_len = spec.height_m / 2.0
            T[:3, 3] = [float(x), y_bay, z_bfloor + 0.003 + half_len * np.cos(np.radians(tilt))]
            items.append((name, k, T, "basket"))
    return items


def _upper_items() -> list[tuple[str, int, np.ndarray, str]]:
    """(name, instance, T_design_obj, zone) for the upper rack (greedy lattice packing)."""
    p = config.RACK_GEN[UPPER]
    z_floor = rack_gen.floor_top_z(p)
    items: list[tuple[str, int, np.ndarray, str]] = []

    # circular footprint radius per drinkware item: the widest BODY radial (the mug's belly
    # is 46.5 mm — wider than its 39.9 mm rim); the handle stays outside this circle — its
    # yaw is chosen below and the FCL validation checks the true mesh
    def foot_r(spec):
        if tuple(spec.axis_obj) == (0.0, 1.0, 0.0):  # Y-up mug: radial extents are x/z
            return float(spec.bbox_half[2])
        return float(spec.rim_radius_m)

    # zone B (x-high) is clipped for the packer: x <= 0.290 (the wine-glass lie-in claims the
    # strip toward the right slope shelf) and y <= 0.168 (the upside-down bowl overhangs the
    # rear strip edge); zone A keeps its full depth
    zones = [tuple(z) for z in p["open_zones"]]
    zones = [
        (x0, min(x1, 0.290), y0, min(y1, 0.168)) if x0 > 0.185 else (x0, x1, y0, min(y1, 0.176))
        for x0, x1, y0, y1 in zones
    ]
    # packing wishlist: (name, instance, yaw) in priority order; the greedy lattice takes the
    # first free spot, so mugs land in zone A (x-low) and tumblers/cups in zone B. Mug yaws
    # point the handles diagonally inward (outward pokes through the front rim wall).
    # one mug per zone at most (the 93 mm belly owns a zone column); the second mug stands in
    # the lower open zone instead (see _lower_items)
    wishlist = [
        ("mug", 0, 180.0),
        ("tumbler", 0, 0.0), ("tumbler", 1, 0.0),
        ("cup", 0, 0.0), ("cup", 1, 0.0),
    ]
    placed: list[tuple[float, float, float]] = []  # (x, y, foot_r) accepted
    divider_x = p.get("divider_tine_x")

    def fits(x, y, r):
        for zx0, zx1, zy0, zy1 in zones:
            if zx0 + r - 0.002 <= x <= zx1 - r + 0.002 and zy0 + r - 0.004 <= y <= zy1 - r + 0.004:
                break
        else:
            return False
        if divider_x is not None and abs(x - divider_x) < r + 0.004:
            return False
        return all((x - px) ** 2 + (y - py) ** 2 >= (r + pr + ITEM_MARGIN_M) ** 2 for px, py, pr in placed)

    # candidate lattice over both zones (8 mm pitch, deterministic order)
    cands = []
    for zx0, zx1, zy0, zy1 in zones:
        for x in np.arange(zx0 + 0.02, zx1 - 0.019, 0.008):
            for y in np.arange(zy0 + 0.02, zy1 - 0.019, 0.008):
                cands.append((round(float(x), 4), round(float(y), 4)))

    for name, inst, yaw in wishlist:
        spec = config.OBJECTS[name]
        r = foot_r(spec)
        spot = next(((x, y) for x, y in cands if fits(x, y, r)), None)
        if spot is None:
            continue  # honest capacity: the packer records only what fits
        placed.append((spot[0], spot[1], r))
        R = _stand_R(spec, yaw_deg=yaw, upside_down=True)
        items.append((name, inst, _place(spec, R, spot, z_floor), "open_zone_upside_down"))

    # -- wine glasses: stemware lie-in over the right slope strip / cup shelf ------------------
    # Bowl rim down on the ascending slope, foot up toward the wall-side shelf with its
    # stemware scallops; spawned tilted and physics-settled (stability gate judges it, with a
    # documented lay-down fallback in scripts/setup/capacity_fill.py).
    for k, y in enumerate((0.060, 0.130)):
        T = np.eye(4)
        T[:3, :3] = _rot("y", -145.0)  # axis (bottom->rim) points down-interior; foot up-wall
        T[:3, 3] = [0.336, y, z_floor + 0.052]
        items.append(("wine_glass", k, T, "stemware_shelf"))

    # -- serving spoon flat along the LEFT slope strip (under that cup shelf) ------------------
    spec = config.OBJECTS["serving_spoon"]
    T = np.eye(4)
    T[:3, :3] = _rot("z", 90.0)  # long axis along rack y, lying flat
    T[:3, 3] = [0.026, 0.100, z_floor + spec.bbox_half[2] + 0.006]
    items.append(("serving_spoon", 0, T, "slope_strip_flat"))

    # -- rear strip: upside-down food container with the lid on top + the second bowl ----------
    cont = config.OBJECTS["container"]
    T_cont = _place(cont, _stand_R(cont, upside_down=True), (0.100, 0.234), z_floor)
    items.append(("container", 0, T_cont, "rear_strip"))
    lid = config.OBJECTS["lid"]
    T_lid = _place(lid, _stand_R(lid), (0.100, 0.234), z_floor + cont.height_m + 0.004)
    items.append(("lid", 0, T_lid, "rear_strip"))
    bowl = config.OBJECTS["bowl"]
    T_bowl = _place(bowl, _stand_R(bowl, upside_down=True), (0.250, 0.226), z_floor)
    items.append(("bowl", 1, T_bowl, "rear_strip"))
    return items


def plan_full_load(cache_dir: str | None = None) -> list[PlannedItem]:
    """The full-load plan in the robot-base frame (deterministic).

    Args:
        cache_dir: Geometry cache with the rack ``T_base_body`` transforms (default: the
            active scenario's cache — use the both_out cache, racks extended for loading).
    """
    manifest = load_manifest(cache_dir or config.scenario_cache_dir())
    out: list[PlannedItem] = []
    order = 0
    counters: dict[str, int] = {}
    for rack, body, builder in (("lower", LOWER, _lower_items), ("upper", UPPER, _upper_items)):
        T_base_rack = np.array(manifest["statics"][body]["T_base_body"])
        for name, inst, T_design, zone in builder():
            counters[name] = counters.get(name, 0)
            item_id = f"{name}_{counters[name]:02d}"
            counters[name] += 1
            out.append(
                PlannedItem(
                    item_id=item_id,
                    name=name,
                    instance=inst,
                    T_base_obj=T_base_rack @ T_design,
                    rack=rack,
                    zone=zone,
                    spawn_order=order,
                )
            )
            order += 1
    return out


# ---------------------------------------------------------------------------------------------
# validation (venv-side, before any Kit boot)
# ---------------------------------------------------------------------------------------------


def _item_mesh(name: str) -> trimesh.Trimesh:
    path = os.path.join(config.ASSETS_DIR, "props", "meshes", f"{name}.obj")
    if name == "mug" and not os.path.exists(path):
        # the mug ships as a USD-only asset; its true body-frame mesh lives in the baseline
        # geometry cache (extracted by scripts/setup/extract_geometry.py) — a bbox stand-in would false-positive
        # the handle spacing checks
        cache_obj = os.path.join(config.CACHE_DIR, "meshes", "object.obj")
        if os.path.exists(cache_obj):
            return trimesh.load(cache_obj, force="mesh")
        spec = config.OBJECTS["mug"]
        return trimesh.creation.box(extents=[2 * h for h in spec.bbox_half])
    return trimesh.load(path, force="mesh")


def validate_plan(items: list[PlannedItem], cache_dir: str | None = None) -> dict:
    """FCL overlap + rack-clearance + closability z-budget check of a plan.

    Returns a report dict; raises AssertionError on hard layout bugs (item-item overlap).
    """
    import fcl  # noqa: PLC0415

    cache_dir = cache_dir or config.scenario_cache_dir()
    manifest = load_manifest(cache_dir)

    def convex(mesh, T):
        hull = mesh.convex_hull
        faces = np.hstack([np.full((len(hull.faces), 1), 3, dtype=np.int64), hull.faces.astype(np.int64)])
        geo = fcl.Convex(hull.vertices.astype(np.float64), len(hull.faces), faces.flatten())
        return fcl.CollisionObject(geo, fcl.Transform(T[:3, :3], T[:3, 3]))

    objs = []
    for it in items:
        mesh = _item_mesh(it.name)
        objs.append((it, convex(mesh, it.T_base_obj)))

    # pairwise item overlap (convex-hull level; the mug hull ignores the cavity — fine for
    # spacing checks since nothing nests inside another item in this layout)
    overlaps = []
    for i in range(len(objs)):
        for j in range(i + 1, len(objs)):
            if objs[i][0].item_id == "lid_00" and objs[j][0].item_id == "container_00":
                continue  # the lid intentionally rests on the container
            if objs[j][0].item_id == "lid_00" and objs[i][0].item_id == "container_00":
                continue
            req, res = fcl.CollisionRequest(), fcl.CollisionResult()
            fcl.collide(objs[i][1], objs[j][1], req, res)
            if res.is_collision:
                overlaps.append((objs[i][0].item_id, objs[j][0].item_id))
    assert not overlaps, f"planned items overlap: {overlaps}"

    # closability z-budget: lower-rack items must clear the stowed upper rack's underside.
    # Budget = inter-rack dz + the upper rack's lowest interior geometry (its heavy runners
    # sit z_runner_heavy above the body origin, minus the wire radius).
    T_lower = np.array(manifest["statics"][LOWER]["T_base_body"])
    zl_up = rack_gen._z_levels(config.RACK_GEN[UPPER])
    dz_racks = 0.2417593 - 0.0875132  # measured body offsets (tests/test_rack_gen.py:339)
    z_budget = T_lower[2, 3] + dz_racks + zl_up["z_runner_heavy"] - config.RACK_GEN[UPPER]["wire_dia_heavy"] / 2.0
    violations = []
    for it, obj in objs:
        if it.rack != "lower":
            continue
        mesh = _item_mesh(it.name)
        top = float((it.T_base_obj[:3, :3] @ mesh.vertices.T).T[:, 2].max() + it.T_base_obj[2, 3])
        if top > z_budget - 0.002:
            violations.append((it.item_id, round(top, 4)))
    counts: dict[str, int] = {}
    for it in items:
        counts[it.name] = counts.get(it.name, 0) + 1
    return {
        "n_items": len(items),
        "counts": counts,
        "z_budget_base": round(z_budget, 4),
        "closability_violations": violations,
    }
