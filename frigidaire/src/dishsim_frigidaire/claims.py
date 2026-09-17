"""Explicit rack-capacity claim layouts for the current Frigidaire geometry.

Unlike ``loading.plan_full_load`` (greedy saturation of a finite pattern), a claim layout
is a fixed statement of WHAT goes WHERE, derived from the tine grid:

- Upper rack: 6 inverted tumblers per side in the outer channels, 2 saucers at the very
  front of the centre gap, then as many tilted bowls as fit rearward (a measured count).
- Lower rack, two sideways-plate rows of 11 gaps. Variant A: front row 6 dinner + 2 salad
  + 2 bowls (front-right, ahead of the basket); rear row 6 dinner + 5 salad.
  Variant B: both rows 6 dinner + 5 salad, no bowls.
- Basket: 4 each of fork, knife, tablespoon, teaspoon (the cutlery poses that survived the
  v2 physics campaign; basket geometry and frame are unchanged).

Every claimed item takes the first FCL-free pose among a short list of variants (v2-accepted
variant first). An item with no free variant is recorded as ``unplaced``; the geometry verdict
is PASS only when nothing is unplaced. Bowls are additionally kept from nesting: consecutive
bowls must occupy disjoint depth slabs along the mouth axis. Output is the same schema-1
manifest ``load_validation.validate_manifest`` and the full-load evidence script consume.
"""
from __future__ import annotations

from collections import Counter
import hashlib
import json
import math
from pathlib import Path

import numpy as np

from . import loading
from .loading import CUTLERY, CollisionWorld, _candidate, _sloped_candidate, geometry_hashes, rotation

VARIANTS = ("A", "B")
LOWER_FRONT = {"A": ("dinner_plate",) * 6 + ("salad_plate",) * 2,
               "B": ("dinner_plate",) * 6 + ("salad_plate",) * 5}
LOWER_REAR = ("dinner_plate",) * 6 + ("salad_plate",) * 5
LOWER_BOWLS = {"A": 2, "B": 0}
UPPER_TUMBLERS_PER_SIDE = 6
UPPER_SAUCERS = 2
BASKET_KEYS = (
    "fork:cutlery_0_0_2:yaw90_X-4_shift0_0",
    "knife:cutlery_1_0_2:yaw90_X-4_shift0_-0.002",
    "tablespoon:cutlery_2_2_0:yaw90_Y4_shift0_0.002",
    "teaspoon:cutlery_3_0_0:observed_ccd_closed_dx0_dy0_dz0.0005",
    "fork:cutlery_0_1_2:yaw90_X-4_shift0_0",
    "knife:cutlery_1_1_2:yaw90_X-4_shift0_-0.002",
    "tablespoon:cutlery_2_0_0:yaw270_Y-8_shift0_0.002",
    "teaspoon:cutlery_3_0_2:observed_ccd_closed_dx0_dy0.0005_dz0.0005",
    "fork:cutlery_0_2_2:yaw90_X-4_shift-0.002_0",
    "knife:cutlery_1_2_2:yaw90_X-4_shift0_-0.002",
    "tablespoon:cutlery_2_1_2:yaw0_X-4_shift0_-0.002",
    "teaspoon:cutlery_3_2_0:observed_ccd_closed_dx0.0005_dy0_dz0.0005",
    "fork:cutlery_0_0_0:yaw90_X8_shift0_-0.002",
    "knife:cutlery_1_0_0:yaw90_Y-4_shift-0.002_0",
    "tablespoon:cutlery_2_1_1:yaw0_X-4_shift0_0.002",
    "teaspoon:cutlery_3_2_2:observed_ccd_closed_dx-0.0015_dy0.0005_dz0.001",
)
LOWER_PLATE_FLOOR = .006
LOWER_BOWL_FLOOR = .007
UPPER_CENTRE_FLOOR = -.0081      # centre rib top (-10 mm) + 1.9 mm wire radius
UPPER_GLASS_FLOOR = .001233
REAR_VALLEY_Y = .1485            # mid-valley between lower floor cross-wires (v2 fix)
BASKET_FRONT_Y = -.040           # lower bowls must stay ahead of the basket body
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
    """Sideways plate in one tine gap; v2-accepted lean first, valley seating last (rear)."""
    out = []
    ys = [("", y)] + ([("_valley", REAR_VALLEY_Y)] if bank == "rear" else [])
    for suffix, yy in ys:
        for lean, offset in ((-4, .008 if kind == "dinner_plate" else .004), (-8, .010)):
            orient = rotation("Y", math.radians(90 - lean))
            out.append(_tilted(kind, "LowerRack", f"lower_{bank}_{gap:02d}", x + offset, yy, orient,
                               points, LOWER_PLATE_FLOOR, f"lean{lean}_offset{offset}{suffix}"))
    return out


def lower_bowl_variants(points, y_bank, slot):
    """Bowls in the front-right zone ahead of the basket.

    Vertical tines cannot straddle a 140 mm bowl, so the bowl goes OVER them: mouth facing
    down and sideways (rotation about Y past 90 deg) or down and forward (about X), rim on
    the floor, tines entering the cavity from below. The v2 mouth-up family (65 deg about Y)
    is kept last as a fallback; it only fit the old 150 mm row pitch.
    """
    out = []
    families = ([("Y", t) for t in (120, 135, 105, 150)] + [("X", t) for t in (120, 135, 105, 150)]
                + [("Y", t) for t in (65, 55, 75)])
    for axis, tilt in families:
        orient = rotation(axis, math.radians(tilt))
        for base_y in (y_bank, -.1125, -.186):
            for dy in (0., .010, -.010):
                y = base_y + dy
                for x in np.arange(.060, .2251, .005):
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
    for x, lean, lift in ((.190, 8, .010), (.180, 16, .010), (.190, 4, .012), (.160, 24, .014)):
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
    """Bowl in the centre gap, mouth facing down and forward, tines inside the cavity.

    Rotation about X past 90 deg turns the mouth toward the front and the floor; the rim
    rests on the centre floor rib and the base leans on the tine row behind. Mouth-up
    poses (tilt < 90) cannot clear the inner tine columns and are kept only as a fallback.
    """
    out = []
    for tilt in (120, 135, 110, 150, 100, 82, 70):
        orient = rotation("X", math.radians(tilt))
        for offset in (0., .010, -.010, .020, -.020):
            out.append(_tilted("bowl", "UpperRack", f"upper_bowl_{gap:02d}", 0., y + offset, orient,
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
    from .geometry import lower_tine_positions, upper_tine_positions
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
        for gap, kind in enumerate(row):
            place(f"lower_{bank}_gap{gap}_{kind}",
                  lower_plate_variants(kind, bank, gap, float(mids[gap]), y, points[kind]))
    if LOWER_BOWLS[variant] == 2:
        # Joint search: the first bowl is only accepted if a non-nesting, collision-free
        # second bowl exists to its right with it in place.
        first_options = lower_bowl_variants(points["bowl"], front_y, "bowl_front_0")
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
        place("lower_bowl_1", lower_bowl_variants(points["bowl"], front_y, "bowl_front_0"))

    # 3. upper rack: tumblers, then the two front saucers, then bowls filling rearward
    for side in (-1, 1):
        for index in range(UPPER_TUMBLERS_PER_SIDE):
            y = -.210 + .085 * index
            place(f"upper_tumbler_{'left' if side < 0 else 'right'}_{index + 1}",
                  upper_tumbler_variants(side, index, y, points["tumbler"]))
    _, upper_ys = upper_tine_positions()
    gap_mids = (upper_ys[:-1] + upper_ys[1:]) / 2
    for gap in range(UPPER_SAUCERS):
        place(f"upper_saucer_{gap + 1}", upper_saucer_variants(gap, float(gap_mids[gap]), points["saucer"]))
    upper_bowls = []
    for gap in range(UPPER_SAUCERS, len(gap_mids)):
        options = [(c, o) for c, o in upper_bowl_variants(gap, float(gap_mids[gap]), points["bowl"])
                   if all(slabs_disjoint(points["bowl"], p, (c, o)) for p in upper_bowls)]
        # a gap without a free bowl pose is a measurement, not a claim failure
        _, option = place(f"upper_bowl_gap{gap}", options, required=False)
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
