# Copyright (c) 2026, dishsim project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Procedural dishware props (Kit-free): tumbler, wine glass, container + lid, serving spoon.

Each builder returns a list of **convex, watertight** trimesh pieces in the object's canonical
frame (Z-up for drinkware/containers, length-along-X for utensils, origin at the bbox center).
The pieces serve two consumers with zero decomposition slop:

- The authoring pipeline (``build_object_assets.py``, in git history) authored them as per-part
  USD mesh prims, each with a convex-hull collider (PhysX side);
- ``scripts/setup/decompose_meshes.py`` writes them verbatim as the FCL pieces (no CoACD — CoACD
  would seal thin open shells like the glass walls).

Deterministic by construction (fixed segment counts, no RNG): scripts/setup/decompose_meshes.py must regenerate
byte-identical geometry from the registry dims alone. Dims come from
:data:`dishsim.config.OBJECTS` — edit there, not here.
"""

import numpy as np
import trimesh

from . import config

#: radial segments for shell walls (each wall segment is one convex piece)
WALL_SEGMENTS = 12
#: polygon segments for solid lathe pieces (discs, stems)
SOLID_SIDES = 24


def _ring_wall(
    r_in_bot: float,
    r_out_bot: float,
    r_in_top: float,
    r_out_top: float,
    z_bot: float,
    z_top: float,
    segments: int = WALL_SEGMENTS,
) -> list[trimesh.Trimesh]:
    """An open conical shell wall as ``segments`` convex hull pieces (arc slabs)."""
    parts = []
    ang = np.linspace(0.0, 2.0 * np.pi, segments + 1)
    # sub-sample each arc so the hull follows the curve (3 points per arc face)
    for i in range(segments):
        a = np.linspace(ang[i], ang[i + 1], 4)
        ca, sa = np.cos(a), np.sin(a)
        pts = []
        for r, z in ((r_in_bot, z_bot), (r_out_bot, z_bot), (r_in_top, z_top), (r_out_top, z_top)):
            pts.append(np.column_stack([r * ca, r * sa, np.full_like(a, z)]))
        hull = trimesh.convex.convex_hull(np.vstack(pts))
        parts.append(hull)
    return parts


def _disc(r: float, z_bot: float, z_top: float, sides: int = SOLID_SIDES) -> trimesh.Trimesh:
    """A solid cylinder (convex) between two z planes."""
    m = trimesh.creation.cylinder(radius=r, height=z_top - z_bot, sections=sides)
    m.apply_translation([0.0, 0.0, (z_bot + z_top) / 2.0])
    return m


def _box(size, center) -> trimesh.Trimesh:
    m = trimesh.creation.box(extents=size)
    m.apply_translation(center)
    return m


def tumbler() -> list[trimesh.Trimesh]:
    """Open drinking tumbler: tapered shell, dia 52 -> 60 mm, h 105 mm, wall 2.5 mm."""
    spec = config.OBJECTS["tumbler"]
    h = spec.height_m
    r_top = spec.rim_radius_m
    r_bot = 0.026
    wall = 0.0025
    floor_t = 0.004
    z0, z1 = -h / 2.0, h / 2.0
    parts = [_disc(r_bot, z0, z0 + floor_t)]
    parts += _ring_wall(r_bot - wall, r_bot, r_top - wall, r_top, z0 + floor_t, z1)
    return parts


def wine_glass() -> list[trimesh.Trimesh]:
    """Wine glass: foot disc + stem + open tulip bowl; h 120 mm, bowl dia 58 mm.

    The stem is 8 mm to seat in the upper rack's stemware scallops (r 8 mm).
    """
    spec = config.OBJECTS["wine_glass"]
    h = spec.height_m
    r_bowl = spec.rim_radius_m
    z0 = -h / 2.0
    foot_t = 0.004
    stem_h = 0.046
    stem_r = 0.004
    bowl_h = h - foot_t - stem_h  # 0.070
    wall = 0.002
    parts = [
        _disc(0.0275, z0, z0 + foot_t),  # foot
        _disc(stem_r, z0 + foot_t, z0 + foot_t + stem_h),  # stem
        _disc(0.012, z0 + foot_t + stem_h, z0 + foot_t + stem_h + 0.008),  # bowl base cap
    ]
    zb0 = z0 + foot_t + stem_h
    # tulip profile: radius widens then eases at the rim (3 stacked wall bands)
    prof = [(0.012, zb0 + 0.008), (0.026, zb0 + 0.030), (r_bowl, zb0 + 0.052), (0.0275, zb0 + bowl_h)]
    for (r_a, z_a), (r_b, z_b) in zip(prof[:-1], prof[1:]):
        parts += _ring_wall(r_a - wall, r_a, r_b - wall, r_b, z_a, z_b)
    return parts


def container() -> list[trimesh.Trimesh]:
    """Open food-storage box 120 x 90 x 55 mm, wall 2.5 mm, floor 3 mm."""
    spec = config.OBJECTS["container"]
    L, W = 2 * spec.bbox_half[0], 2 * spec.bbox_half[1]
    H = spec.height_m
    wall, floor_t = 0.0025, 0.003
    z0 = -H / 2.0
    parts = [
        _box((L, W, floor_t), (0, 0, z0 + floor_t / 2.0)),
        _box((L, wall, H - floor_t), (0, -(W - wall) / 2.0, floor_t / 2.0)),
        _box((L, wall, H - floor_t), (0, (W - wall) / 2.0, floor_t / 2.0)),
        _box((wall, W - 2 * wall, H - floor_t), (-(L - wall) / 2.0, 0, floor_t / 2.0)),
        _box((wall, W - 2 * wall, H - floor_t), ((L - wall) / 2.0, 0, floor_t / 2.0)),
    ]
    return parts


def lid() -> list[trimesh.Trimesh]:
    """Container lid 126 x 96 x 8 mm: top plate + retaining lip."""
    spec = config.OBJECTS["lid"]
    L, W = 2 * spec.bbox_half[0], 2 * spec.bbox_half[1]
    H = spec.height_m
    plate_t = 0.003
    lip = 0.003
    parts = [
        _box((L, W, plate_t), (0, 0, H / 2.0 - plate_t / 2.0)),
        _box((L, lip, H - plate_t), (0, -(W - lip) / 2.0, -plate_t / 2.0)),
        _box((L, lip, H - plate_t), (0, (W - lip) / 2.0, -plate_t / 2.0)),
        _box((lip, W - 2 * lip, H - plate_t), (-(L - lip) / 2.0, 0, -plate_t / 2.0)),
        _box((lip, W - 2 * lip, H - plate_t), ((L - lip) / 2.0, 0, -plate_t / 2.0)),
    ]
    return parts


def serving_spoon() -> list[trimesh.Trimesh]:
    """Serving spoon 180 mm: handle bar + solid shallow scoop (convex — fine for collision)."""
    spec = config.OBJECTS["serving_spoon"]
    L = spec.height_m  # long axis = x
    handle_l = 0.105
    handle = _box((handle_l, 0.014, 0.005), (-(L / 2.0) + handle_l / 2.0, 0, 0.002))
    # scoop: squashed half-ellipsoid, dished side up; +2 mm z shift centers the merged bbox
    scoop = trimesh.creation.icosphere(subdivisions=2, radius=1.0)
    scoop.apply_scale([0.0375, 0.020, 0.008])
    scoop.vertices[:, 2] = np.minimum(scoop.vertices[:, 2], 0.004)
    scoop = trimesh.convex.convex_hull(scoop.vertices)
    scoop.apply_translation([L / 2.0 - 0.0375, 0, 0.002])
    return [handle, scoop]


BUILDERS = {
    "tumbler": tumbler,
    "wine_glass": wine_glass,
    "container": container,
    "lid": lid,
    "serving_spoon": serving_spoon,
}


def build(name: str) -> list[trimesh.Trimesh]:
    """Convex parts for a procedural object (registry key or builder name)."""
    if name not in BUILDERS:
        raise ValueError(f"no procedural builder {name!r} (choices: {sorted(BUILDERS)})")
    return BUILDERS[name]()

