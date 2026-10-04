#!/usr/bin/env python3
"""Load the HOTEC wheat-straw set (8 plates, 8 bowls, 8 cups) into the Frigidaire FDPC4221AS.

Two stages, one file:

    # 1. Kit-free FCL layout (seconds): rack slots in priority plates > bowls > cups, overflow to the counter slab
    code/util/run_py.sh code/planner/frigidaire/frigidaire_hotec_load.py --layout-only \\
        --assets data/assets/models/hotec_wheatstraw/v1 --out-dir data/results/hotec/frigidaire/v1

    # 2. One Isaac session: spawn, open door, extend racks, teleport, settle, stills, orbit video
    code/util/run_kit.sh code/planner/frigidaire/frigidaire_hotec_load.py \\
        --layout data/results/hotec/frigidaire/v1/layout.json --assets data/assets/models/hotec_wheatstraw/v1 \\
        --out-dir data/results/hotec/frigidaire/v1 --tag hotec_v1 --headless --enable_cameras --device cpu

Rack poses are rack-local metres with XYZW quaternions (the claims/loading convention) and are
composed with the MEASURED extended-rack frames in Isaac; counter poses are world. Every slot is
derived from the tape-measured rack geometry in ``dishsim_frigidaire.geometry`` (lower rack 6 x 12
tines at 31.8 mm columns and 81 / 73 / 67 / 73 / 81 mm rows in a 525 x 561 mm rim, the basket bay removing
columns 11-12 of rows 3-6 so the rear bank keeps 9 gaps; upper rack 4 x 13 at 33 mm with the wide centre gap; cup mouths on
the upper glass channel site), never from hard-coded gap indices. Mass: a massless asset (v1) must
carry no MassAPI and PhysX's derived masses are logged; a measured asset (v2+, catalog ``mass_kg``)
must carry exactly that physics:mass on its root and PhysX must report it back. This script adds none. Judge
the Kit run by its ``[RESULT]`` line, never by the exit code.
"""
from __future__ import annotations

import argparse
from collections import deque
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import sys
import time
import traceback

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
sys.path[:0] = [str(ROOT / "code/src"), str(ROOT / "code/frigidaire/src")]
from dishsim_frigidaire import geometry                                # noqa: E402  numpy-only, safe before AppLauncher

KINDS = ("plate", "bowl", "cup")
COLOURS = ("blue_grey", "teal", "coral", "mustard")
COUNTER = {"size_m": (1.2, .6, .04), "center_m": (0., 0., .894), "top_z_m": .914}   # planner.py:38 (runtime slab)
PLATE_LEANS = (-24, -20, -28, -16, -32, -12)                           # a 40 mm plate in a 32.1 mm clear tine gap (36 - 3.9):
PLATE_OFFSETS = tuple(round(x, 3) for x in np.arange(-.012, .0121, .002))   # FCL-measured window ~ lean -24, +4 mm
FRONT_PLATE_INSET_Y = .010                 # rearward: a 228.6 mm plate between the front rows (80 mm pitch, 563 mm rack) would touch the front rim
_LOWER_XS, _LOWER_YS = geometry.lower_tine_positions()
_LOWER_MIDS = (_LOWER_XS[:-1] + _LOWER_XS[1:]) / 2
_FRONT_USABLE = [g for g in geometry.lower_plate_gaps("front")        # gaps whose mid x stays 20 mm ahead of the basket bay
                 if _LOWER_MIDS[g] <= geometry.lower_basket_footprint()["x"][0] - .020]
FRONT_GAPS = tuple(_FRONT_USABLE[::2][:4])                             # every 2nd usable front gap: no shingling
REAR_GAPS = tuple(geometry.lower_plate_gaps("rear")[::2][:4])          # every 2nd of the 9 gaps the basket bay leaves
UPPER_BOWL_GAPS = tuple(geometry.upper_tine_gaps(1, regular_only=False))   # (a, b) tine pairs of a centre column, front to rear, incl. the wide (4, 7)
_GLASS = abs(geometry._upper_rack()["sites"]["right_glass_channel"][0])    # upper-rack sloped glass channel |x| (site, .18775)
CUP_VARIANTS = tuple((round(_GLASS + dx, 5), lean, lift) for dx, lean, lift in
                     ((0., 0, .010), (.005, 4, .010), (-.005, 8, .012), (-.010, 0, .014)))   # near-upright like claims.TUMBLER_VARIANTS; the 125 mm wall caps the outward lean
CUP_SLOTS = tuple((side, index) for index in (1, 2, 3, 4) for side in (-1, 1))
COUNTER_CELLS = tuple(sorted(((round(x, 3), round(y, 3)) for x in np.arange(-.50, .501, .05) for y in np.arange(-.20, .201, .05)),
                            key=lambda c: (abs(c[1]), c[0])))        # 0.05 m grid, FCL decides what fits
ORBIT_YAW_DEG, ORBIT_YAW_SWING_DEG = -90., 70.      # frigidaire_exposure_scene_render.py:27-39
ORBIT_ELEV_DEG, ORBIT_ELEV_SWING_DEG = 32., 16.
ORBIT_DISTANCE_SCALE = .82
RACK_BOX = {"LowerRack": ((-.28, .28), (-.30, .30), (-.03, .32)),      # rack-local containment boxes [m]
            "UpperRack": ((-.26, .26), (-.29, .29), (-.04, .27))}


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def orbit_eye(k, n, target, distance):
    """Camera position for frame k of n: one closed loop across the front, smooth at the seam."""
    u = 2 * math.pi * k / n
    yaw = math.radians(ORBIT_YAW_DEG + ORBIT_YAW_SWING_DEG * math.sin(u))
    elev = math.radians(ORBIT_ELEV_DEG + ORBIT_ELEV_SWING_DEG * math.cos(u))
    return (target[0] + distance * math.cos(elev) * math.cos(yaw),
            target[1] + distance * math.cos(elev) * math.sin(yaw),
            target[2] + distance * math.sin(elev))


# ----------------------------------------------------------------------------- Kit-free layout

def sloped_cup(slot, mouth_x, y, lean, points, floor, height):
    """Inverted cup on the sloped glass channel: loading._sloped_candidate with the mouth at +height
    (base-centred origin) instead of the library's body-centred half-height literal."""
    from dishsim_frigidaire.loading import quaternion_xyzw, rotation
    side = np.sign(mouth_x)
    angle = math.radians(side * lean)
    orient = rotation("Y", angle) @ rotation("X", math.pi)
    normal = np.array([math.sin(angle), 0., math.cos(angle)])
    x = mouth_x + height * math.sin(angle)
    z = (floor * normal[2] + .002 - ((points @ orient.T) @ normal).min() - normal[0] * (x - mouth_x)) / normal[2]
    return {"kind": "cup", "rack": "UpperRack", "slot": slot, "variant": f"lean{lean}",
            "position": [float(x), float(y), float(z)], "quaternion_xyzw": quaternion_xyzw(orient)}


def plan_layout(assets_dir, bowls_upper=4, search=False, plates="split", chooser=None):
    """``search``: capacity mode. Besides the fixed slot lists, bowls may also lie mouth-down over the short
    middle tine rows left of the basket bay (all bowls non-nesting), every upper centre gap is open and the
    cup ladder gains its two end slots. Realistic families only; FCL decides.
    ``plates``: "split" = 4 + 4 over both banks (runs v1 to v7); "front" = every usable front gap, the rear
    bank left free for bowls mouth-down over the rear tines.
    ``chooser(item, options) -> options``: reorders a slot's candidate poses before the first FCL-free one is
    taken (None = the authored order, i.e. the deterministic v1 to v8 behaviour); the exposure search uses it."""
    from dishsim_frigidaire.asset import BODY_POSITIONS
    from dishsim_frigidaire.claims import (LOWER_BOWL_FLOOR, LOWER_BOWL_LIFTS, LOWER_PLATE_FLOOR, UPPER_GLASS_FLOOR,
                                           _tilted, lower_bowl_variants, slabs_disjoint, upper_bowl_variants)
    from dishsim_frigidaire.geometry import lower_tine_positions, upper_tine_positions
    from dishsim_frigidaire.loading import CollisionWorld, _candidate, collision_parts, geometry_hashes, rotation, visual_points
    from dishsim_frigidaire.paths import ASSET_DIR

    assets_dir = Path(assets_dir)
    catalog = json.loads((assets_dir / "catalog.json").read_text())
    world = CollisionWorld(ASSET_DIR)
    parts, points = {}, {}
    for kind in KINDS:                       # "bowl" deliberately replaces the stock bowl: claims' builders use that literal
        path = assets_dir / f"{kind}.usda"
        if digest(path) != catalog["items"][kind]["sha256"]:
            raise ValueError(f"{path} differs from catalog.json")
        parts[kind], points[kind] = collision_parts(path), visual_points(path)
        world.parts[kind], world.points[kind] = parts[kind], points[kind]
    heights = {kind: float(points[kind][:, 2].max()) for kind in KINDS}
    accepted, occupied, rejected = [], set(), []

    def try_place(item, options):
        if chooser is not None:
            options = chooser(item, list(options))
        for c, o in options:
            if c["slot"] in occupied:
                continue
            if world.collides(world.candidate_objects(c)):
                rejected.append({"item": item, "slot": c["slot"], "variant": c["variant"]})
                continue
            world.add(c)
            occupied.add(c["slot"])
            entry = dict(c, item=item, rack_local_pose={"position_m": [float(x) for x in c["position"]],
                                                        "quaternion_xyzw": [float(x) for x in c["quaternion_xyzw"]]})
            accepted.append(entry)
            return entry, o
        return None, None

    teeth, rows = lower_tine_positions()
    mids = (teeth[:-1] + teeth[1:]) / 2
    front_y, rear_y = float((rows[0] + rows[1]) / 2), float((rows[-2] + rows[-1]) / 2)
    plate_centre = np.array([0., 0., heights["plate"] / 2])       # base-centred origin: put the MID-PLANE in the gap
    banks = ((("front", front_y + FRONT_PLATE_INSET_Y, tuple(_FRONT_USABLE)),) if plates == "front"
             else (("front", front_y + FRONT_PLATE_INSET_Y, FRONT_GAPS), ("rear", rear_y, REAR_GAPS)))
    for bank, y, gaps in banks:
        for gap in gaps:
            if sum(e["kind"] == "plate" for e in accepted) >= 8:      # the set has eight plates
                break
            options = []
            for lean in PLATE_LEANS:
                orient = rotation("Y", math.radians(90 - lean))
                shift = orient @ plate_centre
                for off in PLATE_OFFSETS:
                    options.append((_candidate("plate", "LowerRack", f"lower_{bank}_{gap:02d}", float(mids[gap]) + off - shift[0],
                                               y - shift[1], orient, points["plate"], LOWER_PLATE_FLOOR,
                                               f"lean{lean}_offset{off}"), None))
            try_place(f"plate_{bank}_{gap:02d}", options)
    bowls = []
    first = lower_bowl_variants(points["bowl"], front_y, "lower_bowl_0")
    c1, o1 = try_place("lower_bowl_1", first)
    if c1 is not None:
        bowls.append((c1, o1))
        second = [(dict(c, slot="lower_bowl_1"), o) for c, o in first
                  if abs(c["position"][0] - c1["position"][0]) > .02 and slabs_disjoint(points["bowl"], (c1, o1), (c, o))]
        c2, o2 = try_place("lower_bowl_2", second)
        if c2 is not None:
            bowls.append((c2, o2))
    if search:
        # Capacity mode: bowls mouth-down over the SHORT middle rows (rows 3-4), left of the basket bay,
        # between the two plate banks; the same tilt families as the front-right pair, non-nesting.
        mid_y = float((rows[2] + rows[3]) / 2)
        x_free = geometry.PARAMETERS["lower_rack"]["basket_reserved_x"][0] - .005
        families = [("Y", t) for t in (120, 135, 105, 150)] + [("X", t) for t in (120, 135, 105, 150)]
        zones = [("mid", mid_y, (0., .010, -.010, .020, -.020), np.arange(-.200, .0651, .005)),          # between the banks
                 ("rearright", rear_y, (0., .010, -.010, float(rows[-1] - rear_y)), np.arange(-.020, .0701, .005))]  # beside the basket, behind the rear plates
        if plates == "front":                                     # the whole rear bank is free: bowls over the rear rows
            zones.insert(0, ("rear", rear_y, (0., .010, -.010, float(rows[-2] - rear_y), float(rows[-1] - rear_y)),
                             np.arange(-.200, .0701, .005)))
        n = 0
        while len(bowls) < 8:
            options = []
            for zone, zone_y, dys, xs in zones:
              for axis, tilt in families:
                orient = rotation(axis, math.radians(tilt))
                for dy in dys:
                    for x in xs:
                        base, _ = _tilted("bowl", "LowerRack", f"lower_{zone}_bowl_{n}", float(x), zone_y + dy, orient,
                                          points["bowl"], LOWER_BOWL_FLOOR, f"{axis}{tilt}_y{zone_y + dy:.4f}_x{x:.3f}")
                        world_pts = points["bowl"] @ orient.T + np.asarray(base["position"])
                        if world_pts[:, 0].max() > x_free or np.abs(world_pts[:, 1]).max() > .270:
                            continue
                        if not all(slabs_disjoint(points["bowl"], p, (base, orient)) for p in bowls if p[0]["rack"] == "LowerRack"):
                            continue
                        for lift in LOWER_BOWL_LIFTS:
                            options.append((dict(base, position=[base["position"][0], base["position"][1], base["position"][2] + lift],
                                                 variant=f'{base["variant"]}_lift{lift}'), orient))
            c, o = try_place(f"lower_extra_bowl_{n}", options)
            if c is None:
                break
            bowls.append((c, o))
            n += 1
    _, upper_ys = upper_tine_positions()
    for a, b in UPPER_BOWL_GAPS:                            # slot upper_bowl_{a:02d}: gap between present tines a and b
        if sum(p[0]["rack"] == "UpperRack" for p in bowls) >= max(0, int(bowls_upper)):
            break
        y = float((upper_ys[a] + upper_ys[b]) / 2)
        options = [(c, o) for c, o in upper_bowl_variants(a, y, points["bowl"])
                   if all(slabs_disjoint(points["bowl"], p, (c, o)) for p in bowls if p[0]["rack"] == "UpperRack")]
        c, o = try_place(f"upper_bowl_{a:02d}", options)
        if c is not None:
            bowls.append((c, o))
    cup_slots = CUP_SLOTS + (tuple((side, index) for index in (0, 5) for side in (-1, 1)) if search else ())
    for side, index in cup_slots:
        y = -.210 + .085 * index
        options = []
        for x, lean, lift in CUP_VARIANTS:
            c = sloped_cup(f"glass_{'left' if side < 0 else 'right'}_{index}", side * x, y, lean, points["cup"],
                           UPPER_GLASS_FLOOR, heights["cup"])
            c["position"][2] += lift
            c["variant"] += f"_x{x}_lift{lift}"
            options.append((c, None))
        try_place(f"cup_{'left' if side < 0 else 'right'}_{index}", options)
    counter = []
    fitted = {kind: sum(e["kind"] == kind for e in accepted) for kind in KINDS}
    for kind in KINDS:                                    # priority order is the loop order
        for _ in range(8 - fitted[kind]):
            for x, y in COUNTER_CELLS:
                z = COUNTER["top_z_m"] + .002 - float(points[kind][:, 2].min())
                if any(c["kind"] == kind and abs(c["pose_world"]["position_m"][0] - x) < 1e-9
                       and abs(c["pose_world"]["position_m"][1] - y) < 1e-9 for c in counter):
                    continue
                objects = world.transform(parts[kind], np.eye(3), np.array([x, y, z]))
                if world.collides(objects):
                    continue
                world.manager.registerObjects(objects)
                world.manager.update()
                counter.append({"kind": kind, "rack": "Counter", "slot": f"counter_{x:+.2f}_{y:+.2f}",
                                "variant": "upright", "item": f"counter_{kind}",
                                "pose_world": {"position_m": [x, y, z], "quaternion_xyzw": [0., 0., 0., 1.]}})
                break
    objects, counters = [], {kind: 0 for kind in KINDS}
    for entry in accepted + counter:
        kind = entry["kind"]
        counters[kind] += 1
        objects.append({"object_id": f"{kind}_{counters[kind]:02d}", "kind": kind, "usd": f"{kind}.usda",
                        "color": COLOURS[(counters[kind] - 1) % len(COLOURS)], "rack": entry["rack"],
                        "slot": entry["slot"], "variant": entry["variant"], "item": entry["item"],
                        **({"rack_local_pose": entry["rack_local_pose"]} if "rack_local_pose" in entry
                           else {"pose_world": entry["pose_world"]})})
    by_rack = {}
    for obj in objects:
        by_rack.setdefault(obj["rack"], {}).setdefault(obj["kind"], 0)
        by_rack[obj["rack"]][obj["kind"]] += 1
    from collections import Counter as _Counter
    rejection_summary = dict(_Counter(r["item"] for r in rejected))
    layout = {"schema_version": 1, "purpose": "hotec_load_layout", "assets_dir": str(assets_dir.resolve()),
              "asset_sha256": {f"{kind}.usda": catalog["items"][kind]["sha256"] for kind in KINDS},
              "parameters_sha256": catalog["parameters_sha256"], "appliance_dir": str(ASSET_DIR),
              "appliance_sha256": geometry_hashes(ASSET_DIR), "body_positions_m": {k: list(v) for k, v in BODY_POSITIONS.items()},
              "coordinate_system": "rack objects: rack-local metres + XYZW in the closed-appliance FCL frame; "
                                   "counter objects: world metres + XYZW (runtime slab, top z = 0.914)",
              "priority": "plates > bowls > cups; lower-rack bowls capped at 2 by the basket gate; "
                          "upper-rack centre gap counts as fitted; leftovers upright on the counter slab",
              "plates_mode": plates, "search": bool(search),
              "rejected_per_item": {item: n for item, n in sorted(__import__("collections").Counter(r["item"] for r in rejected).items())},
              "objects": objects, "counts": {"requested": {kind: 8 for kind in KINDS}, "fitted": fitted,
                                            "counter": {kind: sum(c["kind"] == kind for c in counter) for kind in KINDS},
                                            "by_rack": by_rack, "total": len(objects)},
              "fcl": {"rejections": len(rejected), "rejections_by_item": rejection_summary,
                      "status": "FCL non-overlap only; physics validation required"}}
    return layout


def layout_main(argv):
    parser = argparse.ArgumentParser(description="Kit-free HOTEC layout")
    parser.add_argument("--layout-only", action="store_true")
    parser.add_argument("--assets", type=Path, default=ROOT / "data/assets/models/hotec_wheatstraw/v1")
    parser.add_argument("--out-dir", type=Path, required=True)
    parser.add_argument("--bowls-upper", type=int, default=4)
    parser.add_argument("--search", action="store_true", help="capacity mode: middle-row lower bowls, every upper gap, two extra cup slots")
    parser.add_argument("--plates", choices=("split", "front"), default="split", help="split: 4 + 4 over both banks; front: every usable front gap, rear bank free for bowls")
    args = parser.parse_args(argv)
    sys.path[:0] = [str(ROOT / "code/src"), str(ROOT / "code/frigidaire/src")]
    args.out_dir.mkdir(parents=True, exist_ok=True)
    output = args.out_dir / "layout.json"
    if output.exists():
        raise FileExistsError(f"Refusing to overwrite layout: {output}")
    layout = plan_layout(args.assets, bowls_upper=args.bowls_upper, search=args.search, plates=args.plates)
    output.write_text(json.dumps(layout, indent=2, allow_nan=False) + "\n")
    counts = layout["counts"]
    ok = counts["total"] == 24 and counts["fitted"]["plate"] == 8      # every plate racked, nothing on the counter
    print(f"[RESULT] {'PASS' if ok else 'FAIL'} plates {counts['fitted']['plate']}/8 bowls {counts['fitted']['bowl']}/8 "
          f"cups {counts['fitted']['cup']}/8 counter {sum(counts['counter'].values())} -> {output}", flush=True)
    return 0 if ok else 1


# ----------------------------------------------------------------------------- Kit session

def run(args, report):
    # All Kit/USD/project imports happen after AppLauncher initialized Kit.
    import numpy as np
    import torch
    import omni.usd
    import isaaclab.sim as sim_utils
    from isaaclab.sim import SimulationContext
    from isaaclab.assets import RigidObject, RigidObjectCfg
    from pxr import PhysxSchema, Usd, UsdGeom, UsdPhysics
    from PIL import Image, ImageDraw
    from dishsim.media import CameraRig, VideoWriter
    from dishsim.quats import wxyz_to_xyzw, xyzw_to_wxyz
    from dishsim_frigidaire.asset import apply_mode, spawn
    from dishsim_frigidaire.initial_state_runtime import RACK_COMMAND_SPEED_M_S, all_vertex_step_speed, window_motion_metrics
    from dishsim_frigidaire.loading import geometry_hashes, visual_points
    from dishsim_frigidaire.random_pose_runtime import COMPONENTS, EXTENSION, JOINTS
    from dishsim_frigidaire.random_poses import LIMITS, compose_pose, motion_is_settled, relative_pose, transform_vertices
    from frigidaire_initial_state_render import CAMERA, HEIGHT, WIDTH, caption_fonts

    layout = json.loads(args.layout.read_text())
    assets = args.assets
    for name, expected in layout["asset_sha256"].items():
        if digest(assets / name) != expected:
            raise ValueError(f"asset changed since the layout was planned: {name}")
    if geometry_hashes(args.usd.parent) != layout["appliance_sha256"]:
        raise ValueError("appliance USD changed since the layout was planned")
    outputs = {key: args.out_dir / f"{args.tag}_{key}" for key in
               ("placed.png", "settled.png", "collision.png", "orbit.mp4", "settle.json")}
    for path in outputs.values():
        if path.exists():
            raise FileExistsError(f"Refusing to overwrite: {path}")
    entries = layout["objects"]
    dt = LIMITS["physics_dt_s"]
    catalog = json.loads((assets / "catalog.json").read_text())
    report.update(layout_path=str(args.layout), layout_sha256=digest(args.layout), parameters_sha256=layout["parameters_sha256"],
                  asset_sha256_before=dict(layout["asset_sha256"]), counts=layout["counts"], camera=CAMERA,
                  runtime={"isaac_sim": Path("/isaac-sim/VERSION").read_text().strip(), "physics_hz": round(1 / dt),
                           "device": args.device, "scene_ccd": True, "thresholds": dict(LIMITS),
                           "observation_seconds": args.observation, "settle_timeout_s": args.settle_timeout},
                  scope="Isaac PhysX settle of an FCL layout with both racks extended; no rack retraction; "
                        "no contact/penetration gate; asset colour variants, no per-item tint override")

    sim = SimulationContext(sim_utils.SimulationCfg(dt=dt, device=args.device, use_fabric=True,
                                                    physx=sim_utils.PhysxCfg(enable_ccd=True)))
    stage = omni.usd.get_context().get_stage()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.)
    ground = sim_utils.CuboidCfg(size=(200., 200., .05), collision_props=sim_utils.CollisionPropertiesCfg(),
                                 visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(.20, .225, .255), roughness=.8))
    ground.func("/World/Ground", ground, translation=(0., 0., -.025))
    slab = sim_utils.CuboidCfg(size=tuple(COUNTER["size_m"]), collision_props=sim_utils.CollisionPropertiesCfg(),
                               visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(.72, .70, .66), roughness=.5))
    slab.func("/World/Counter", slab, translation=tuple(COUNTER["center_m"]))
    fill = sim_utils.DomeLightCfg(intensity=1200, color=(.92, .95, 1.))
    fill.func("/World/Fill", fill)
    key = sim_utils.DistantLightCfg(intensity=2400, angle=15, color=(1., .97, .93))
    key.func("/World/Key", key, orientation=(.9238795, .3826834, 0, 0))
    rim = sim_utils.DistantLightCfg(intensity=1100, angle=20, color=(.86, .92, 1.))
    rim.func("/World/Rim", rim, orientation=(.7071068, -.5, .5, 0))
    dishwasher, basket = spawn("/World/HotecDishwasher", mode="scripted", usd_path=str(args.usd))

    objects, selections, spawned_sizes = {}, {}, {}
    for index, entry in enumerate(entries):
        oid = entry["object_id"]
        objects[oid] = RigidObject(RigidObjectCfg(
            prim_path=f"/World/HotecDishes/{oid}",
            spawn=sim_utils.UsdFileCfg(usd_path=str((assets / entry["usd"]).resolve())),
            init_state=RigidObjectCfg.InitialStateCfg(pos=(3. + index * .4, 0., .3))))      # parking row
        prim = stage.GetPrimAtPath(f"/World/HotecDishes/{oid}")
        vset = prim.GetVariantSets().GetVariantSet("color")
        vset.SetVariantSelection(entry["color"])
        selections[oid] = vset.GetVariantSelection()
        box = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render]).ComputeWorldBound(prim).ComputeAlignedRange()
        size = (np.array(box.GetMax()) - np.array(box.GetMin())).tolist()
        spawned_sizes[oid] = size
        if np.abs(np.array(size) - np.array(catalog["items"][entry["kind"]]["usd_size_m"])).max() > 5e-4:
            raise ValueError(f"{oid}: spawned size {size} differs from the catalog")
        rigid, physx = UsdPhysics.RigidBodyAPI(prim), PhysxSchema.PhysxRigidBodyAPI(prim)
        if (not rigid.GetRigidBodyEnabledAttr().Get() or rigid.GetKinematicEnabledAttr().Get()
                or physx.GetDisableGravityAttr().Get() or not physx.GetEnableCCDAttr().Get()):
            raise ValueError("Expected dynamic gravity-enabled CCD dish: " + oid)
        mass_attr = prim.GetAttribute("physics:mass")
        authored = prim.HasAPI(UsdPhysics.MassAPI) or (mass_attr.IsValid() and mass_attr.HasAuthoredValue())
        expected_mass = catalog["items"][entry["kind"]].get("mass_kg")
        if expected_mass is None and authored:
            raise ValueError("Unexpected authored mass on " + oid)
        if expected_mass is not None and (not authored or abs(float(mass_attr.Get()) - expected_mass) > 1e-6):
            raise ValueError(f"{oid}: authored physics:mass {mass_attr.Get()} differs from the catalog's {expected_mass} kg")
    report.update(variant_selections=selections, spawned_sizes_m=spawned_sizes)
    points = {kind: visual_points(assets / f"{kind}.usda") for kind in KINDS}
    kinds = {e["object_id"]: e["kind"] for e in entries}

    rig = CameraRig(CAMERA, hw=(HEIGHT, WIDTH))
    sim.reset()
    apply_mode(dishwasher, "scripted")
    rig.apply_poses(sim.device)
    masses = {}
    for oid, obj in objects.items():
        try:
            masses[oid] = float(obj.data.default_mass.reshape(-1)[0])
        except AttributeError:
            masses[oid] = float(obj.root_physx_view.get_masses().reshape(-1)[0])
    measured = {kind: catalog["items"][kind].get("mass_kg") for kind in KINDS}
    if all(m is not None for m in measured.values()):
        for oid, m in masses.items():
            if abs(m - measured[kinds[oid]]) > 1e-3:
                raise ValueError(f"{oid}: PhysX reports {m:.4f} kg, the authored measured mass is {measured[kinds[oid]]:.4f} kg")
        report.update(masses_kg=masses, mass_source="authored physics:mass on each asset root (measured average of 8 pieces "
                                                    "on a kitchen scale, 2026-09-22); PhysX reports it back; this script adds none")
    else:
        report.update(masses_kg=masses, mass_source="PhysX-derived from the authored collision shells at its default density; "
                                                    "no MassAPI in the assets and none authored by this script")
    if any(not math.isfinite(m) or abs(m - 1.) < 1e-9 or m <= 0. for m in masses.values()):
        raise ValueError(f"PhysX mass derivation failed for some piece: {masses}")

    indices = {name: dishwasher.joint_names.index(name) for name in JOINTS}
    body_indices = {name: dishwasher.body_names.index(name) for name in COMPONENTS[:-1]}
    target = dishwasher.data.default_joint_pos.clone()

    def _pose(position, quaternion_wxyz):
        return {"position_m": position.detach().cpu().numpy().tolist(),
                "quaternion_xyzw": wxyz_to_xyzw(quaternion_wxyz.detach().cpu().numpy().copy()).tolist()}

    def poses():
        result = {name: _pose(dishwasher.data.body_pos_w[0, i], dishwasher.data.body_quat_w[0, i]) for name, i in body_indices.items()}
        result["SilverwareBasket"] = _pose(basket.data.root_pos_w[0], basket.data.root_quat_w[0])
        result.update({oid: _pose(obj.data.root_pos_w[0], obj.data.root_quat_w[0]) for oid, obj in objects.items()})
        return result

    def joints():
        values = dishwasher.data.joint_pos[0].detach().cpu().numpy()
        return {name: float(values[i]) for name, i in indices.items()}

    def tick():
        dishwasher.set_joint_position_target(target)
        dishwasher.write_data_to_sim()
        sim.step(render=False)
        dishwasher.update(dt)
        basket.update(dt)
        for obj in objects.values():
            obj.update(dt)
        return poses()

    def ramp(goal):
        initial = target.clone()
        duration = max(3.5, *(1.5 * abs(float(initial[0, indices[name]]) - value) / (.35 if name == "door_hinge" else RACK_COMMAND_SPEED_M_S)
                              for name, value in goal.items()))
        steps = math.ceil(duration / dt)
        for step in range(1, steps + 1):
            fraction = step / steps
            blend = fraction * fraction * (3. - 2. * fraction)
            for name, value in goal.items():
                target[0, indices[name]] = initial[0, indices[name]] * (1 - blend) + value * blend
            tick()
        return steps * dt

    def settle(names, motion_points, timeout, observation):
        """The backend's hold(): a 1 s trailing window at 120 Hz, qualify within timeout, then observe."""
        window = deque(maxlen=round(LIMITS["rest_window_s"] / dt) + 1)
        previous, first_pass, restarts = None, None, []
        metrics, frames = {}, None
        for step in range(math.ceil((timeout + observation) / dt) + 1):
            frames = tick()
            speeds = {name: 0. if previous is None else all_vertex_step_speed(motion_points[name], previous[name], frames[name], dt)
                      for name in names}
            previous = frames
            window.append({"poses": frames, "speeds": speeds})
            if len(window) < window.maxlen:
                continue
            metrics = {name: window_motion_metrics(window, name, dt) for name in names}
            passed = all(motion_is_settled(m) for m in metrics.values())
            if passed and first_pass is None:
                if (step + 1) * dt > timeout + 1e-12:
                    return {"passed": False, "reason": "no qualifying rest window before settle timeout",
                            "simulated_seconds": (step + 1) * dt, "motion": metrics, "restarts": restarts}, frames
                first_pass = step
            elif not passed and first_pass is not None:
                restarts.append({"failed_at_simulated_seconds": (step + 1) * dt,
                                 "failed": [n for n, m in metrics.items() if not motion_is_settled(m)]})
                first_pass = None
                if (step + 1) * dt >= timeout:
                    return {"passed": False, "reason": "continuous observation failed after the settle allowance",
                            "simulated_seconds": (step + 1) * dt, "motion": metrics, "restarts": restarts}, frames
            if first_pass is not None and step - first_pass >= round(observation / dt):
                return {"passed": True, "reason": "all required trailing rest windows passed",
                        "simulated_seconds": (step + 1) * dt, "qualification_simulated_seconds": (first_pass + 1) * dt,
                        "motion": metrics, "restarts": restarts}, frames
        return {"passed": False, "reason": "continuous observation did not complete", "simulated_seconds": (step + 1) * dt,
                "motion": metrics, "restarts": restarts}, frames

    # --- empty baseline: closed hold, door open, both racks out, basket settled -----------------
    basket_points = {"SilverwareBasket": visual_points(args.usd.parent / "silverware_basket.usdc")}
    for _ in range(round(2. / dt)):
        tick()
    baseline = {"door_ramp_s": ramp({"door_hinge": math.pi / 2}),
                "racks_ramp_s": ramp({"upper_slide": EXTENSION["UpperRack"], "lower_slide": EXTENSION["LowerRack"]})}
    hold, frames = settle(("SilverwareBasket",), basket_points, timeout=args.settle_timeout, observation=1.)
    baseline.update(hold=hold, joints=joints(), rack_frames={name: frames[name] for name in ("LowerRack", "UpperRack")})
    report["baseline"] = baseline
    if not hold["passed"]:
        raise RuntimeError("empty extended baseline did not settle: " + hold["reason"])

    # --- layout poses in the measured extended-rack frames ------------------------------------
    placed = {}
    for entry in entries:
        oid = entry["object_id"]
        if "rack_local_pose" in entry:
            frame = frames[entry["rack"]]
            position, quaternion = compose_pose(frame["position_m"], frame["quaternion_xyzw"],
                                                entry["rack_local_pose"]["position_m"], entry["rack_local_pose"]["quaternion_xyzw"])
            pose = {"position_m": position.tolist(), "quaternion_xyzw": quaternion.tolist()}
        else:
            pose = entry["pose_world"]
        placed[oid] = pose

    def teleport(oid, pose):
        values = [*pose["position_m"], *xyzw_to_wxyz(pose["quaternion_xyzw"])]
        obj = objects[oid]
        obj.write_root_pose_to_sim(torch.tensor([values], dtype=torch.float32, device=sim.device))
        obj.write_root_velocity_to_sim(torch.zeros((1, 6), device=sim.device))
        obj.reset()

    order = layout.get("insertion_order") if args.sequential else None
    if args.sequential and not order:
        raise ValueError("--sequential needs a layout with insertion_order (see frigidaire_hotec_exposure_search.py)")
    steps = []
    if order:
        # every piece starts upright on the floor beside the machine (0.30 m stash grid, out of the camera's
        # framing and wider than the 229 mm plate), then enters one at a time
        cells = [(x, y) for y in (-.6, -.3, 0., .3, .6) for x in (1.2, 1.5, 1.8, 2.1, 2.4)]
        for k, entry in enumerate(entries):
            oid = entry["object_id"]
            x, y = cells[k]
            teleport(oid, {"position_m": [float(x), float(y), .002 - float(points[kinds[oid]][:, 2].min())],
                           "quaternion_xyzw": [0., 0., 0., 1.]})
    else:
        for oid, pose in placed.items():
            teleport(oid, pose)
    tick()                                                   # one physics step before any render (Fabric)
    fonts, font_evidence = caption_fonts()

    def still(path, title, detail):
        for _ in range(16):
            sim.render()
            rig.update(sim.get_physics_dt())
        pixels = rig.grab_one("initial")
        if pixels.shape != (HEIGHT, WIDTH, 3) or float(pixels.std()) <= 5 or int(pixels.max()) <= 60:
            raise ValueError("Isaac camera produced an empty or malformed RGB frame")
        picture = Image.fromarray(pixels)
        draw = ImageDraw.Draw(picture)
        draw.rectangle((0, 0, WIDTH, 160), fill=(18, 28, 40))
        draw.text((34, 18), title, fill=(246, 249, 252), font=fonts["title"])
        draw.text((36, 80), detail, fill=(218, 229, 239), font=fonts["detail"])
        draw.text((36, 118), "both racks extended · door open · Isaac RTX · " + ("measured mass authored" if "authored physics:mass" in report["mass_source"] else "no authored mass (PhysX-derived)"),
                  fill=(190, 207, 222), font=fonts["detail"])
        picture.save(path)
        return pixels

    counts = layout["counts"]
    detail = (f"{counts['total']} pieces  |  racks: {counts['fitted']['plate']} plates, {counts['fitted']['bowl']} bowls, "
              f"{counts['fitted']['cup']} cups  |  counter: {sum(counts['counter'].values())}")
    if order:
        # --- sequential insertion in the gated order: teleport one piece, short settle, still ------
        instructions = layout.get("placement_instructions") or [f"{k + 1}. {oid}" for k, oid in enumerate(order)]
        still(args.out_dir / f"{args.tag}_step_00.png", "HOTEC wheat-straw set - all pieces on the counter, nothing placed yet", detail)
        inserted, last_frames = [], {}
        for n, oid in enumerate(order, 1):
            teleport(oid, placed[oid])
            tick()
            inserted.append(oid)
            step_points = {o: points[kinds[o]] for o in inserted}
            step_points["SilverwareBasket"] = basket_points["SilverwareBasket"]
            hold_n, frames_n = settle((*inserted, "SilverwareBasket"), step_points, timeout=4., observation=1.)
            disturbed = {}
            for o in inserted[:-1]:
                d = float(np.linalg.norm(np.asarray(frames_n[o]["position_m"]) - np.asarray(last_frames[o]["position_m"])))
                disturbed[o] = d
            worst_prev = max(disturbed.values()) if disturbed else 0.
            last_frames = {o: frames_n[o] for o in inserted}
            text = instructions[n - 1]
            still(args.out_dir / f"{args.tag}_step_{n:02d}.png", f"step {n}/{len(order)}: {text[:150]}",
                  f"placed so far {n}  |  this step settled {'yes' if hold_n['passed'] else 'NO'} in {hold_n['simulated_seconds']:.1f} s  |  "
                  f"largest nudge of an earlier piece {worst_prev * 1000:.0f} mm")
            steps.append({"step": n, "object_id": oid, "instruction": text, "settle_passed": hold_n["passed"], "settle_reason": hold_n["reason"],
                          "simulated_seconds": hold_n["simulated_seconds"], "earlier_piece_disturbance_m": disturbed,
                          "worst_earlier_disturbance_m": worst_prev})
            print(f"[INFO] step {n}/{len(order)} {oid}: settle {'ok' if hold_n['passed'] else 'FAILED'}, earlier pieces nudged <= {worst_prev * 1000:.0f} mm", flush=True)
        still(outputs["placed.png"], "HOTEC wheat-straw set in the FDPC4221AS - after sequential insertion (before the final settle)", detail)
    else:
        still(outputs["placed.png"], "HOTEC wheat-straw set in the FDPC4221AS - as placed (FCL layout)", detail)

    # --- loaded settle ---------------------------------------------------------------------------
    motion_points = {oid: points[kinds[oid]] for oid in objects}
    motion_points["SilverwareBasket"] = basket_points["SilverwareBasket"]
    hold, frames = settle((*objects, "SilverwareBasket"), motion_points, timeout=args.settle_timeout, observation=args.observation)
    per_object, worst = {}, {"translation_m": 0., "rotation_deg": 0.}
    for entry in entries:
        oid = entry["object_id"]
        before, after = placed[oid], frames[oid]
        cosine = abs(float(np.dot(before["quaternion_xyzw"], after["quaternion_xyzw"])))
        cosine /= float(np.linalg.norm(before["quaternion_xyzw"]) * np.linalg.norm(after["quaternion_xyzw"]))
        row = {"kind": kinds[oid], "color": entry["color"], "rack": entry["rack"], "slot": entry["slot"], "variant": entry["variant"],
               "placed_pose_world": before, "final_pose_world": after,
               "translation_m": float(np.linalg.norm(np.asarray(after["position_m"]) - before["position_m"])),
               "rotation_deg": float(np.degrees(2 * np.arccos(np.clip(cosine, 0., 1.))))}
        world_points = transform_vertices(points[kinds[oid]], after["position_m"], after["quaternion_xyzw"])
        if entry["rack"] in RACK_BOX:
            frame = frames[entry["rack"]]
            local, _ = relative_pose(after["position_m"], after["quaternion_xyzw"], frame["position_m"], frame["quaternion_xyzw"])
            local_placed, _ = relative_pose(before["position_m"], before["quaternion_xyzw"], baseline["rack_frames"][entry["rack"]]["position_m"],
                                            baseline["rack_frames"][entry["rack"]]["quaternion_xyzw"])
            row["rack_relative_drift_m"] = float(np.linalg.norm(local - local_placed))
            rel = relative_pose(world_points.mean(axis=0), [0., 0., 0., 1.], frame["position_m"], frame["quaternion_xyzw"])[0]
            box = RACK_BOX[entry["rack"]]
            row["in_assigned_rack"] = bool(all(lo - .01 <= c <= hi + .01 for c, (lo, hi) in zip(rel, box)))
        else:
            row["on_slab"] = bool(abs(after["position_m"][0]) <= .6 and abs(after["position_m"][1]) <= .3
                                  and float(world_points[:, 2].min()) >= COUNTER["top_z_m"] - .01)
        per_object[oid] = row
        worst = {"translation_m": max(worst["translation_m"], row["translation_m"]),
                 "rotation_deg": max(worst["rotation_deg"], row["rotation_deg"])}
    contained = all(row.get("in_assigned_rack", row.get("on_slab", False)) for row in per_object.values())
    settle_record = {"schema_version": 1, "settle": hold, "objects": per_object, "max_displacement": worst,
                     "sequential": {"order": order, "steps": steps} if order else None,
                     "all_contained": contained, "counts": counts, "masses_kg": masses,
                     "final_joints": joints(), "rack_frames": {name: frames[name] for name in ("LowerRack", "UpperRack")}}
    outputs["settle.json"].write_text(json.dumps(settle_record, indent=2, allow_nan=False) + "\n")
    report.update(settle_file=outputs["settle.json"].name, settle_passed=hold["passed"], settle_reason=hold["reason"],
                  settle_simulated_seconds=hold["simulated_seconds"], all_contained=contained, max_displacement=worst)

    # --- stills, collision view, orbit ------------------------------------------------------------
    pixels = still(outputs["settled.png"], "HOTEC wheat-straw set in the FDPC4221AS - settled",
                   detail + f"  |  settled {hold['simulated_seconds']:.1f} s, max drift {worst['translation_m'] * 1000:.0f} mm / {worst['rotation_deg']:.0f} deg")
    report.update(image_files={k: outputs[k].name for k in ("placed.png", "settled.png")},
                  image_sha256={k: digest(outputs[k]) for k in ("placed.png", "settled.png")},
                  resolution=[WIDTH, HEIGHT], rgb_std=float(pixels.std()), caption_fonts=font_evidence)
    try:
        for oid in objects:
            root = stage.GetPrimAtPath(f"/World/HotecDishes/{oid}")
            UsdGeom.Imageable(stage.GetPrimAtPath(root.GetPath().AppendChild("Visuals"))).GetVisibilityAttr().Set("invisible")
            UsdGeom.Imageable(stage.GetPrimAtPath(root.GetPath().AppendChild("Collisions"))).GetVisibilityAttr().Set("inherited")
        collision_pixels = still(outputs["collision.png"], "HOTEC wheat-straw set - convex collision pieces", detail)
        report["collision_view"] = ("unavailable: identical to the settled frame" if np.array_equal(collision_pixels, pixels)
                                    else outputs["collision.png"].name)
    except Exception as exc:  # best effort only
        report["collision_view"] = f"unavailable: {exc!r}"
    finally:
        for oid in objects:
            root = stage.GetPrimAtPath(f"/World/HotecDishes/{oid}")
            UsdGeom.Imageable(stage.GetPrimAtPath(root.GetPath().AppendChild("Visuals"))).GetVisibilityAttr().Set("inherited")
            UsdGeom.Imageable(stage.GetPrimAtPath(root.GetPath().AppendChild("Collisions"))).GetVisibilityAttr().Set("invisible")
    eye0, target_point = CAMERA["initial"][0], CAMERA["initial"][1]
    distance = ORBIT_DISTANCE_SCALE * math.dist(eye0, target_point)
    n, size = int(round(args.orbit_seconds * args.fps)), (args.video_width, args.video_width * HEIGHT // WIDTH)
    cam, targets = rig.cams["initial"], torch.tensor([target_point], dtype=torch.float32, device=sim.device)
    video, bar = VideoWriter(str(outputs["orbit.mp4"]), fps=args.fps), size[1] // 16
    try:
        for k in range(n):
            cam.set_world_poses_from_view(eyes=torch.tensor([orbit_eye(k, n, target_point, distance)], dtype=torch.float32, device=sim.device),
                                          targets=targets)
            for _ in range(args.renders_per_frame):
                sim.render()
                rig.update(sim.get_physics_dt())
            frame = Image.fromarray(rig.grab_one("initial")).resize(size, Image.LANCZOS)
            draw = ImageDraw.Draw(frame)
            draw.rectangle((0, 0, size[0], bar), fill=(18, 28, 40))
            draw.text((12, bar // 5), "HOTEC wheat-straw set, settled in the Frigidaire FDPC4221AS · Isaac RTX",
                      fill=(218, 229, 239), font=fonts["detail"].font_variant(size=max(12, bar // 2)))
            video.add(np.asarray(frame))
            if k % (2 * args.fps) == 0:
                print(f"[INFO] orbit frame {k}/{n}", flush=True)
    finally:
        video.close()
    if video.frames != n or outputs["orbit.mp4"].stat().st_size < 100_000:
        raise ValueError(f"Orbit video incomplete: {video.frames}/{n} frames")
    report.update(video_file=outputs["orbit.mp4"].name, video_sha256=digest(outputs["orbit.mp4"]), video_frames=n, video_fps=args.fps,
                  video_resolution=list(size), orbit={"seconds": args.orbit_seconds, "distance_m": distance,
                  "renders_per_frame": args.renders_per_frame})
    after = {name: digest(assets / name) for name in layout["asset_sha256"]}
    report.update(asset_sha256_after=after, assets_unchanged=after == layout["asset_sha256"],
                  result="PASS" if hold["passed"] and contained and after == layout["asset_sha256"] else "FAIL")


def main():
    if "--layout-only" in sys.argv[1:]:
        return layout_main(sys.argv[1:])
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--layout", type=Path, required=True)
    parser.add_argument("--assets", type=Path, default=ROOT / "data/assets/models/hotec_wheatstraw/v1")
    parser.add_argument("--usd", type=Path, default=ROOT / "data/build/frigidaire_collection/usd/fdpc4221as.usdc")
    parser.add_argument("--out-dir", type=Path, required=True)
    parser.add_argument("--tag", default="hotec_v1")
    parser.add_argument("--sequential", action="store_true", help="insert the pieces one by one in the layout's insertion_order (stash on the counter first)")
    parser.add_argument("--settle-timeout", type=float, default=12., dest="settle_timeout")
    parser.add_argument("--observation", type=float, default=5.)
    parser.add_argument("--orbit-seconds", type=float, default=12.)
    parser.add_argument("--fps", type=int, default=24)
    parser.add_argument("--video-width", type=int, default=1280)
    parser.add_argument("--renders-per-frame", type=int, default=3, dest="renders_per_frame")
    from isaaclab.app import AppLauncher
    AppLauncher.add_app_launcher_args(parser)
    parser.set_defaults(headless=True, device="cpu")
    args = parser.parse_args()
    if not args.enable_cameras:
        raise SystemExit("Rendering requires --enable_cameras")
    if args.device != "cpu":
        raise SystemExit("This experiment requires CPU physics")
    args.layout, args.assets, args.usd, args.out_dir = (p.resolve() for p in (args.layout, args.assets, args.usd, args.out_dir))
    args.out_dir.mkdir(parents=True, exist_ok=True)
    report = {"schema_version": 1, "result": "FAIL", "started_utc": datetime.now(timezone.utc).isoformat()}
    app, started = None, time.monotonic()
    evidence = args.out_dir / f"{args.tag}_evidence.json"
    if evidence.exists():
        raise FileExistsError(f"Refusing to overwrite evidence: {evidence}")
    try:
        app = AppLauncher(args).app
        sys.path[:0] = [str(ROOT / "code/src"), str(ROOT / "code/frigidaire/src"), str(ROOT / "code/util/frigidaire")]
        run(args, report)
    except Exception as exc:
        traceback.print_exc()
        report.update(result="FAIL", error=repr(exc))
    finally:
        report.update(finished_utc=datetime.now(timezone.utc).isoformat(), wall_seconds=time.monotonic() - started)
        evidence.write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
        print(f"[RESULT] {report['result']}: {evidence}", flush=True)
        if app is not None:
            from dishsim.media import release_sim_for_close
            release_sim_for_close()
            app.close()
    return 0 if report["result"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
