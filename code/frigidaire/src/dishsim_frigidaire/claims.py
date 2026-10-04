"""Explicit rack-capacity claim layouts for the current Frigidaire geometry.

Unlike ``loading.plan_full_load`` (greedy saturation of a finite pattern), a claim layout
is a fixed statement of WHAT goes WHERE, derived from the tape-measured tine grid
(``geometry.PARAMETERS``, 2026-09-21: 12 x 6 lower tines minus the 8 under the basket,
4 x 13 upper tines minus the 4 absent centre positions):

- Upper rack: 5 inverted tumblers per side in the outer glass channels, 2 saucers at the
  very front of the centre gap, then as many tilted bowls as fit rearward through the
  remaining centre gaps, the wide one included (a measured count).
- Lower rack, two sideways-plate banks: the front bank has 11 gaps, the rear bank 9 (the
  basket bay omits columns 11-12). Variant A: front bank 6 dinner + 2 salad + 2 bowls
  (front-right, ahead of the basket); rear bank 6 dinner + 2 salad. Variant B (both rows
  6 dinner + 5 salad) was retired with this grid: no bank has 11 usable gaps any more.
- Basket: 4 each of fork, knife, tablespoon, teaspoon, one kind per compartment of the
  tape-measured 320 x 95 x 130 mm (L x W x H) basket (1x4). ``BASKET_KEYS`` names the first four poses
  of each kind's greedy simultaneous pattern from the Kit-free FCL pose search
  (frigidaire_cutlery_pose_search.py, 2026-09-23, v3-design basket basket_1x4_320x95_v4, 130 mm outside); no physics refinement yet.

Every claimed item takes the first FCL-free pose among a short list of variants (v2-accepted
variant first; plates carry a rearward-inset variant next, because a 260 mm dinner plate
standing on the bank centreline cuts the rim wire). An item with no free variant is
recorded as ``unplaced``; the geometry verdict is PASS only when nothing is unplaced. Bowls
are additionally kept from nesting: consecutive bowls must occupy disjoint depth slabs along
the mouth axis. Output is the same schema-1 manifest ``load_validation.validate_manifest``
and the full-load evidence script consume.

The recorded claim results (data/build/frigidaire_collection/validation/claims) were produced on
the previous 52/72-tine geometry and are stale until regenerated.
"""
from __future__ import annotations

from collections import Counter
import hashlib
import json
import math
from pathlib import Path

import numpy as np

from . import loading
from .geometry import (PARAMETERS, lower_plate_gaps, lower_tine_positions, upper_tine_gaps,
                       upper_tine_positions)
from .loading import CUTLERY, CollisionWorld, _candidate, _sloped_candidate, geometry_hashes, rotation

VARIANTS = ("A",)            # B (6 dinner + 5 salad per bank) retired: the rear bank has 9 gaps
LOWER_FRONT = {"A": ("dinner_plate",) * 6 + ("salad_plate",) * 2}
LOWER_REAR = ("dinner_plate",) * 6 + ("salad_plate",) * 2
LOWER_BOWLS = {"A": 2}
UPPER_TUMBLERS_PER_SIDE = 5
UPPER_SAUCERS = 2
TUMBLER_LADDER_Y = tuple(-.170 + .085 * i for i in range(UPPER_TUMBLERS_PER_SIDE))
BASKET_KEYS = (
    "fork:cutlery_0_0_0:yaw90_X-4_shift0_0.002",
    "knife:cutlery_1_0_2:yaw90_X-4_shift0_0.002",
    "tablespoon:cutlery_2_0_0:yaw90_X-4_shift0.002_0",
    "teaspoon:cutlery_3_0_0:yaw90_X8_shift0_0.002",
    "fork:cutlery_0_1_0:yaw90_X-4_shift0_0.002",
    "knife:cutlery_1_1_2:yaw90_X-4_shift0_0.002",
    "tablespoon:cutlery_2_1_0:yaw90_X-4_shift0_0",
    "teaspoon:cutlery_3_0_2:yaw90_X8_shift0_-0.002",
    "fork:cutlery_0_2_0:yaw90_X-4_shift0_0.002",
    "knife:cutlery_1_2_2:yaw90_X-4_shift0_0.002",
    "tablespoon:cutlery_2_2_0:yaw90_X-4_shift0_0",
    "teaspoon:cutlery_3_1_0:yaw90_X8_shift0_0.002",
    "fork:cutlery_0_2_2:yaw90_Y4_shift-0.002_0",
    "knife:cutlery_1_0_0:yaw90_X8_shift0_-0.002",
    "tablespoon:cutlery_2_1_2:yaw270_X4_shift0.002_0",
    "teaspoon:cutlery_3_1_2:yaw90_X8_shift0_-0.002",
)
LOWER_PLATE_FLOOR = .006
LOWER_BOWL_FLOOR = .007
UPPER_CENTRE_FLOOR = -.0081      # centre rib top (-10 mm) + 1.9 mm wire radius
UPPER_GLASS_FLOOR = -.002         # wire top at the tumbler mouth's higher (outer, wall-foot) contact; the inner rim point hovers over the ridge flank
# Rearward inset per bank (m): a 260 mm dinner plate standing on the bank centreline would
# cut the rim wire (rows 80 mm apart in a 563 mm rack), so plates try this seat before the valley.
PLATE_INSET_Y = {"front": .025, "rear": -.015}


def _rear_valley_y():
    """Mid-valley between the two lower floor cross-wires just in front of the rear plate bank.

    The cross-wire run mirrors geometry._lower_rack (21 U-ribs inset 18.43 mm from the rim
    centreline), so the seat follows the tape-measured depth (v2 fix, re-derived).
    """
    p = PARAMETERS["lower_rack"]
    hy = p["wire_depth"] / 2 - p["rim_diameter"] / 2
    cross = np.linspace(-(hy - .01843), hy - .01843, p["floor_cross_ribs"])
    _, rows = lower_tine_positions()
    valleys = (cross[:-1] + cross[1:]) / 2
    return float(valleys[valleys < (rows[-2] + rows[-1]) / 2].max())


def _glass_channel_x():
    """Centre of the outer sloped glass channel (geometry._upper_rack's *_glass_channel site): the
    middle of the slope from the low ridge outside the outer tine column down to the trough at the wall."""
    p = PARAMETERS["upper_rack"]
    profile = p["channel_profile"]
    crest = max(p["tine_bank_x"]) + profile["ridge_offset_from_column"]
    trough = p["wire_width"] / 2 - p["rim_diameter"] / 2 - profile["trough_inset_from_rim"]
    return (crest + trough) / 2


REAR_VALLEY_Y = _rear_valley_y()
GLASS_CHANNEL_X = _glass_channel_x()
# (mouth x, lean deg, lift m): near-upright tumblers over the low ridge, mouth between the ridge
# flank and the wall foot; the 125 mm rim wall caps a 160 mm tumbler's outward lean at a few degrees
TUMBLER_VARIANTS = tuple((round(GLASS_CHANNEL_X - inset, 5), lean, lift) for inset, lean, lift
                         in ((0., 0, .010), (.003, 4, .010), (-.003, 8, .012), (.005, 0, .012)))
BASKET_FRONT_Y = PARAMETERS["lower_rack"]["basket_reserved_y"][0] - .004   # lower bowls stay ahead of the basket bay
BOWL_SLAB_CLEARANCE = .003
LOWER_BOWL_LIFTS = (0., .020, .040, .060)
FORBIDDEN_SHORTCUTS = ["resizing props", "nested bowls", "stacked cups", "fixed dishes",
                       "invisible supports"]


def candidate_key(c):
    return f'{c["kind"]}:{c["slot"]}:{c["variant"]}'


def _tilted(kind, rack, slot, x, y, orient, points, floor, variant):
    """Candidate plus the rotation matrix, kept for the nesting check."""
    return _candidate(kind, rack, slot, x, y, orient, points, floor, variant), orient


# ----------------------------------------------------------------------------- pose builders

def lower_plate_variants(kind, bank, gap, x, y, points):
    """Sideways plate in one tine gap; v2-accepted seat first, rearward inset next, valley last (rear)."""
    out = []
    ys = [("", y), ("_inset", y + PLATE_INSET_Y[bank])] + ([("_valley", REAR_VALLEY_Y)] if bank == "rear" else [])
    for suffix, yy in ys:
        for lean, offset in ((-4, .008 if kind == "dinner_plate" else .004), (-8, .010)):
            orient = rotation("Y", math.radians(90 - lean))
            out.append(_tilted(kind, "LowerRack", f"lower_{bank}_{gap:02d}", x + offset, yy, orient,
                               points, LOWER_PLATE_FLOOR, f"lean{lean}_offset{offset}{suffix}"))
    return out


def lower_bowl_variants(points, y_bank, slot, x_start=.060):
    """Bowls in the front-right zone ahead of the basket, scanning x rightward from ``x_start``.

    Vertical tines cannot straddle a 140 mm bowl, so the bowl goes OVER them: mouth facing
    down and sideways (rotation about Y past 90 deg) or down and forward (about X), rim on
    the floor, tines entering the cavity from below. The v2 mouth-up family (65 deg about Y)
    is kept last as a fallback; it only fit the old 150 mm row pitch.
    """
    out = []
    _, rows = lower_tine_positions()
    families = ([("Y", t) for t in (120, 135, 105, 150)] + [("X", t) for t in (120, 135, 105, 150)]
                + [("Y", t) for t in (65, 55, 75)])
    for axis, tilt in families:
        orient = rotation(axis, math.radians(tilt))
        for base_y in (y_bank, float(rows[1]), float(rows[0])):   # bank centreline, second row, front row
            for dy in (0., .010, -.010):
                y = base_y + dy
                for x in np.arange(x_start, .2251, .005):
                    base, _ = _tilted("bowl", "LowerRack", slot, x, y, orient, points,
                                      LOWER_BOWL_FLOOR, f"{axis}{tilt}_y{y:.4f}_x{x:.3f}")
                    world_pts = points @ orient.T + np.asarray(base["position"])
                    if world_pts[:, 1].max() > BASKET_FRONT_Y:
                        continue
                    # lifts let the bowl ride on tine tips instead of the floor rib
                    for lift in LOWER_BOWL_LIFTS:
                        c = dict(base, position=[base["position"][0], base["position"][1],
                                                 base["position"][2] + lift],
                                 variant=f'{base["variant"]}_lift{lift}')
                        out.append((c, orient))
    return out


def upper_tumbler_variants(side, index, y, points):
    out = []
    for x, lean, lift in TUMBLER_VARIANTS:
        c = _sloped_candidate("tumbler", f"glass_{side}_{index}", side * x, y, lean, points,
                              UPPER_GLASS_FLOOR)
        c["position"][2] += lift
        c["variant"] += f"_x{x}_lift{lift}"
        out.append((c, None))
    return out


def upper_saucer_variants(gap, y, points):
    out = []
    for lean, offset in ((8, .010), (4, .008)):
        orient = rotation("X", math.radians(90 - lean))
        out.append(_tilted("saucer", "UpperRack", f"saucer_{gap:02d}", 0., y + offset, orient,
                           points, UPPER_CENTRE_FLOOR, f"lean{lean}_offset{offset}"))
    return out


def upper_bowl_variants(gap, y, points):
    """Bowl in one centre gap (index of its front tine, or "wide" for the gap at the absent
    centre positions), mouth facing down and forward, tines inside the cavity.

    Rotation about X past 90 deg turns the mouth toward the front and the floor; the rim
    rests on the centre floor rib and the base leans on the tine row behind. Mouth-up
    poses (tilt < 90) cannot clear the inner tine columns and are kept only as a fallback.
    """
    out = []
    slot = f"upper_bowl_{gap:02d}" if isinstance(gap, int) else f"upper_bowl_{gap}"
    for tilt in (120, 135, 110, 150, 100, 82, 70):
        orient = rotation("X", math.radians(tilt))
        for offset in (0., .010, -.010, .020, -.020):
            out.append(_tilted("bowl", "UpperRack", slot, 0., y + offset, orient,
                               points, UPPER_CENTRE_FLOOR, f"tilt{tilt}_offset{offset}"))
    return out


def _slab(points, orient, position, normal):
    projected = (points @ orient.T + np.asarray(position)) @ normal
    return float(projected.min()), float(projected.max())


def slabs_disjoint(points, first, second, clearance=BOWL_SLAB_CLEARANCE):
    """Two bowls cannot nest when their depth slabs along the first bowl's mouth axis
    do not overlap. ``first``/``second`` are (candidate, orient) pairs."""
    normal = first[1] @ np.array([0., 0., 1.])
    a = _slab(points, first[1], first[0]["position"], normal)
    b = _slab(points, second[1], second[0]["position"], normal)
    return b[0] >= a[1] + clearance or a[0] >= b[1] + clearance


def cutlery_pattern():
    text = Path(loading.__file__).with_name("cutlery_candidates.json").read_text()
    return json.loads(text)["patterns"]


# ----------------------------------------------------------------------------- generator

def generate(asset_dir, variant, world=None, banned=()):
    """Build the claim manifest for ``variant`` against the tableware in ``asset_dir``.

    ``banned`` lists candidate keys rejected by an earlier physics run; a banned item takes
    its next free variant instead (recorded as ``physics_rejected``), exactly like
    ``plan_full_load``'s ban file.
    """
    from .tableware import CATALOG
    from .asset import BODY_POSITIONS
    if variant not in VARIANTS:
        raise ValueError(f"Unknown claim variant {variant!r}; expected one of {VARIANTS}")
    asset_dir = Path(asset_dir)
    world = world or CollisionWorld(asset_dir)
    points = world.points
    banned = set(banned)
    accepted, rejections, unplaced = [], [], []
    counters = Counter()

    def place(item, options, required=True):
        """First collision-free option wins. Returns (entry, option) or (None, None)."""
        for option in options:
            c = option[0]
            if candidate_key(c) in banned:
                rejections.append({"item": item, "candidate": candidate_key(c),
                                   "reason": "physics_rejected"})
                continue
            if world.collides(world.candidate_objects(c)):
                rejections.append({"item": item, "candidate": candidate_key(c),
                                   "reason": "authored_collider_overlap"})
                continue
            kind = c["kind"]
            counters[kind] += 1
            entry = dict(c, id=f"{kind}_{counters[kind]:03d}", candidate_key=candidate_key(c),
                         claim=item)
            accepted.append(entry)
            world.add(c)
            return entry, option
        if required:
            unplaced.append({"item": item, "kind": options[0][0]["kind"] if options else None,
                             "tried": len(options)})
        return None, None

    # 1. basket: the v2-validated cutlery poses, by key; free same-kind poses as fallback
    patterns = cutlery_pattern()
    by_key = {candidate_key(c): c for kind in CUTLERY for c in patterns[kind]}
    used_slots = set()
    for n, key in enumerate(BASKET_KEYS):
        kind = key.split(":")[0]
        primary = by_key[key]
        others = [c for c in patterns[kind] if c is not primary]
        fallbacks = ([c for c in others if c["slot"] == primary["slot"]]
                     + [c for c in others if c["slot"] != primary["slot"] and c["slot"] not in used_slots])
        entry, _ = place(f"basket_{kind}_{n // 4 + 1}",
                         [(primary, None)] + [(c, None) for c in fallbacks])
        if entry:
            used_slots.add(entry["slot"])

    # 2. lower rows, left -> right, then the front-right bowls (variant A)
    teeth, rows = lower_tine_positions()
    mids = (teeth[:-1] + teeth[1:]) / 2
    front_y = float((rows[0] + rows[1]) / 2)
    rear_y = float((rows[-2] + rows[-1]) / 2)
    claimed = {"lower_front": dict(Counter(LOWER_FRONT[variant])),
               "lower_rear": dict(Counter(LOWER_REAR)),
               "lower_bowls": LOWER_BOWLS[variant],
               "upper_tumblers": 2 * UPPER_TUMBLERS_PER_SIDE, "upper_saucers": UPPER_SAUCERS,
               "basket": {kind: 4 for kind in CUTLERY}}
    for bank, y, row in (("front", front_y, LOWER_FRONT[variant]), ("rear", rear_y, LOWER_REAR)):
        gaps = lower_plate_gaps(bank)        # gaps between tines present in both rows, left to right
        if len(row) > len(gaps):
            raise ValueError(f"{bank} bank claims {len(row)} plates but has {len(gaps)} gaps")
        for gap, kind in zip(gaps, row):
            place(f"lower_{bank}_gap{gap}_{kind}",
                  lower_plate_variants(kind, bank, gap, float(mids[gap]), y, points[kind]))
    # Bowls scan rightward from the first tine column right of the claimed front plates.
    bowl_x_start = float(teeth[len(LOWER_FRONT[variant])])
    if LOWER_BOWLS[variant] == 2:
        # Joint search: the first bowl is only accepted if a non-nesting, collision-free
        # second bowl exists to its right with it in place.
        first_options = lower_bowl_variants(points["bowl"], front_y, "bowl_front_0", bowl_x_start)
        second_options = [(dict(c, slot="bowl_front_1"), o) for c, o in first_options]
        pair = None
        for c1, o1 in first_options:
            if world.collides(world.candidate_objects(c1)):
                rejections.append({"item": "lower_bowl_1", "candidate": candidate_key(c1),
                                   "reason": "authored_collider_overlap"})
                continue
            world.add(c1)
            for c2, o2 in second_options:
                if c2["position"][0] <= c1["position"][0]:
                    continue
                if not slabs_disjoint(points["bowl"], (c1, o1), (c2, o2)):
                    continue
                if not world.collides(world.candidate_objects(c2)):
                    pair = ((c1, o1), (c2, o2))
                    break
            world.reset_load(accepted)
            if pair:
                break
            rejections.append({"item": "lower_bowl_2", "candidate": candidate_key(c1),
                               "reason": "no_non_nesting_partner"})
        if pair:
            place("lower_bowl_1", [pair[0]])
            place("lower_bowl_2", [pair[1]])
        else:
            unplaced.append({"item": "lower_bowl_pair", "kind": "bowl", "tried": len(first_options)})
    elif LOWER_BOWLS[variant]:
        place("lower_bowl_1", lower_bowl_variants(points["bowl"], front_y, "bowl_front_0", bowl_x_start))

    # 3. upper rack: tumblers, then the two front saucers, then bowls filling rearward
    for side in (-1, 1):
        for index, y in enumerate(TUMBLER_LADDER_Y):
            place(f"upper_tumbler_{'left' if side < 0 else 'right'}_{index + 1}",
                  upper_tumbler_variants(side, index, y, points["tumbler"]))
    # Centre column gaps between present tines: the first regular gaps take the saucers and
    # every remaining gap, the wide one at the absent centre positions included, a bowl.
    _, upper_ys = upper_tine_positions()
    saucer_gaps = upper_tine_gaps(1)[:UPPER_SAUCERS]
    for a, b in saucer_gaps:
        place(f"upper_saucer_{a + 1}",
              upper_saucer_variants(a, float((upper_ys[a] + upper_ys[b]) / 2), points["saucer"]))
    upper_bowls = []
    for a, b in upper_tine_gaps(1, regular_only=False):
        if (a, b) in saucer_gaps:
            continue
        gap, item = (a, f"upper_bowl_gap{a}") if b - a == 1 else ("wide", "upper_bowl_wide")
        options = [(c, o) for c, o in upper_bowl_variants(gap, float((upper_ys[a] + upper_ys[b]) / 2), points["bowl"])
                   if all(slabs_disjoint(points["bowl"], p, (c, o)) for p in upper_bowls)]
        # a gap without a free bowl pose is a measurement, not a claim failure
        _, option = place(item, options, required=False)
        if option:
            upper_bowls.append(option)

    counts = dict(Counter(item["kind"] for item in accepted))
    by_rack = {rack: dict(Counter(item["kind"] for item in accepted if item["rack"] == rack))
               for rack in ("LowerRack", "UpperRack", "SilverwareBasket")}
    from .paths import REPO_ROOT
    try:
        recorded_dir = str(asset_dir.resolve().relative_to(REPO_ROOT))
    except ValueError:
        recorded_dir = str(asset_dir)
    manifest = {
        "schema_version": 1, "asset_dir": recorded_dir,
        "coordinate_system": "component-local metres; quaternion XYZW; initial closed appliance",
        "objects": accepted, "catalog": CATALOG, "counts": counts,
        "body_positions_m": BODY_POSITIONS, "geometry_hashes": geometry_hashes(asset_dir),
        "packing": {
            "layout": f"claim_{variant}",
            "status": "FCL claim layout; physics validation required",
            "claim": ("explicit rack-capacity claim layout derived from the tine grid; "
                      "every claimed item takes its first collision-free variant"),
            "claimed": claimed, "by_rack": by_rack, "upper_bowl_fill": len(upper_bowls),
            "unplaced": unplaced, "rejections": rejections, "banned": sorted(banned),
            "geometry_result": "PASS" if not unplaced else "FAIL",
            "candidate_pattern_sha256": hashlib.sha256(
                Path(loading.__file__).with_name("cutlery_candidates.json").read_bytes()).hexdigest(),
            "forbidden_shortcuts": FORBIDDEN_SHORTCUTS,
        },
    }
    return manifest


def geometry_report(manifest):
    packing = manifest["packing"]
    return {"layout": packing["layout"], "result": packing["geometry_result"],
            "object_count": len(manifest["objects"]), "counts": manifest["counts"],
            "by_rack": packing["by_rack"], "claimed": packing["claimed"],
            "upper_bowl_fill": packing["upper_bowl_fill"], "unplaced": packing["unplaced"],
            "banned": packing["banned"], "rejection_count": len(packing["rejections"])}


def write(manifest, out_dir, label):
    """``label`` names the attempt (default: the variant letter; e.g. ``A2`` for a retry)."""
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    manifest_path = out_dir / f"claim_{label}_manifest.json"
    report_path = out_dir / f"claim_{label}_geometry.json"
    manifest_path.write_text(json.dumps(manifest, indent=2) + "\n")
    report_path.write_text(json.dumps(geometry_report(manifest), indent=2) + "\n")
    return manifest_path, report_path
