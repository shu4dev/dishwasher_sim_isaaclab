#!/usr/bin/env python3
# Copyright (c) 2026, dishsim project.
# SPDX-License-Identifier: BSD-3-Clause
"""Kit side of the HOTEC rearrangement benchmark (boot-first; one Kit process per call).

--start: one generation attempt for (tier, seed, attempt): n dishes in a messy pile on the 1.8 m counter
slab, the rest dropped messily into the racks, one joint settle with the racks extended, then the
reproduction gate (teleport back to the settled poses, re-settle 2.5 s, every dish within 10 mm / 15 deg).
Writes <out>/instances/attempts/<tier>_s<seed>_a<attempt>/start.json; [RESULT] PASS | REROLL <reason>.

--run: every episode of one accepted instance. Goal track: greedy_offline, rrt_connect and the planner
pair's sequencer on the sampled goal. Open track: the planner and first-fit baseline plans made Kit-free
(frigidaire_bench.py --plan), replayed here. Every move is teleported and settled (150 ticks + 60-tick
drift window at 120 Hz, the Bosch oracle verdicts, settle-deviation limit 0.08 m); an all-racked end state
gets the end check (tub wall, both racks retracted, containment). Stills of the initial, finished and goal
configurations go to data/media/benchmark/frigidaire_hotec/stills/ when --enable_cameras is set.

    code/util/run_kit.sh code/planner/frigidaire/frigidaire_bench_kit.py --start --tier easy --seed 0 --attempt 0 --headless
    code/util/run_kit.sh code/planner/frigidaire/frigidaire_bench_kit.py --run --instance <json> --headless --enable_cameras
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import sys
import time
import traceback

ROOT = Path(__file__).resolve().parents[3]
STACK_MAX, STACK_OFFSET_M, STACK_TILT_DEG = 4, .015, 4.  # messy stacks on the counter: height, offset, tilt
STACK_DROP_M, STACK_LIFT_STEP_M, STACK_LIFT_MAX_M = .005, .005, .15
PILE_EDGE_M = .02                                       # every dish rim at least 2 cm inside the slab edge
PILE_COVERAGE, PILE_ASPECT = .5, 1.6                    # stack bases: summed base footprint / box area, box x:y
LIFT_STEP_M, RACK_LIFT_MAX_M = .01, .16
RACK_DROP_Z_M = (.02, .06)                              # bbox centre above the rack floor datum before lifting clear
RACK_MARGIN_M = .09                                     # keep drops off the rack walls
TRIES = 1500
SETTLE_TICKS, WINDOW_TICKS, RESET_TICKS = 150, 60, 300  # the Bosch 75 / 30 / 150 steps at 60 Hz, as seconds at 120 Hz
STILL_RENDERS = 48
SEQUENCE_BUILDS, SEQUENCE_NEIGHBOUR_M = 8, .15          # sequence gate: rebuilds while it learns the order; a dish
                                                        # that does not land goes in after its neighbours (< 15 cm)
BUILD_TICKS = 120                                       # 1 s settle after each dish while the start is built
DEV_MAX_M = .08                                         # measured HOTEC drops reach 66.5 mm (the Bosch limit is 0.06)
EPISODES = (("goal", "greedy_offline"), ("goal", "rrt_connect"), ("open", "baseline"), ("open", "mcts"))
# (the goal-track "planner" row, the fixed-goal sequencer, was dropped 2026-09-28: track A compares the executors)


def pose_distance(a, b):
    import numpy as np
    dp = float(np.linalg.norm(np.asarray(a["position_m"]) - b["position_m"]))
    qa, qb = np.asarray(a["quaternion_xyzw"]), np.asarray(b["quaternion_xyzw"])
    c = abs(float(qa @ qb) / (np.linalg.norm(qa) * np.linalg.norm(qb)))
    return dp, float(np.degrees(2 * np.arccos(min(1., c))))


# --------------------------------------------------------------------------- scene helpers

def make_backend(args, B, entries, out_dir, counter_ids=(), cameras=False):
    from dishsim_frigidaire.initial_state_runtime import IsaacInitialStateBackend
    from dishsim_frigidaire.random_poses import source_geometry_domains
    rig = {}

    class Backend(IsaacInitialStateBackend):
        """Counter dishes ride along (no rack support required); snapshots carry the contact pairs."""

        def __init__(self, *a, **kw):
            super().__init__(*a, **kw)
            self.assignments = {k: v for k, v in self.assignments.items() if k not in set(counter_ids)}

        def snapshot(self):
            s = super().snapshot()
            s["contact_pairs"] = sorted(list(p) for p in self.latest_contact["pairs"])
            return s

    def before_reset(backend):
        if cameras:
            from dishsim.media import CameraRig
            sys.path.insert(0, str(ROOT / "code/util/frigidaire"))
            from frigidaire_initial_state_render import HEIGHT, WIDTH, item_tints
            tint_scene(item_tints([{"object_id": oid} for oid in backend.objects]), B.COUNTER_COLOR)
            rig["rig"], rig["hw"] = CameraRig(B.BENCH_CAMERA, hw=(HEIGHT, WIDTH)), (HEIGHT, WIDTH)

    slab = ("/World/Counter", tuple(B.COUNTER["size_m"]), tuple(B.COUNTER["center_m"]))
    backend = Backend(args.usd, out_dir, device="cpu", domains=source_geometry_domains(),
                      deadline=time.monotonic() + args.max_wall_seconds, app=args.app, candidates=entries,
                      extra_statics=[slab], tableware=B.tableware(), before_reset=before_reset)
    if "rig" in rig:
        rig["rig"].apply_poses(backend.sim.device)
        backend.tick()                                  # one physics step before any render (Fabric)
    return backend, rig


def tint_scene(tints, counter_color):
    """Per-item dish tints (CLAUDE.md: one colour per item in multi-object renders) and a visible counter slab."""
    import omni.usd
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdShade
    stage = omni.usd.get_context().get_stage()

    def material(name, color, roughness):
        m = UsdShade.Material.Define(stage, f"/World/BenchMaterials/{name}")
        shader = UsdShade.Shader.Define(stage, m.GetPath().AppendChild("Surface"))
        shader.CreateIdAttr("UsdPreviewSurface")
        shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
        shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(roughness)
        m.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
        return m

    def bind(path, m):
        for child in Usd.PrimRange(stage.GetPrimAtPath(path)):
            if child.IsA(UsdGeom.Gprim):
                UsdShade.MaterialBindingAPI.Apply(child).Bind(m, bindingStrength=UsdShade.Tokens.strongerThanDescendants)

    for oid, color in tints.items():
        bind(f"/World/InitialStateDishes/{oid}", material(oid, color, .36))
    bind("/World/Counter", material("Counter", counter_color, .5))


def still(backend, rig, path, title, detail):
    if "rig" not in rig:
        return None
    from PIL import Image, ImageDraw
    sys.path.insert(0, str(ROOT / "code/util/frigidaire"))
    from frigidaire_initial_state_render import caption_fonts
    for _ in range(STILL_RENDERS):              # RTX accumulation: fewer renders leave ghosts of moved dishes
        backend.sim.render()
        rig["rig"].update(backend.sim.get_physics_dt())
    pixels = rig["rig"].grab_one("initial")
    picture = Image.fromarray(pixels)
    fonts, _ = caption_fonts()
    draw = ImageDraw.Draw(picture)
    draw.rectangle((0, 0, picture.width, 150), fill=(18, 28, 40))
    draw.text((34, 18), title, fill=(246, 249, 252), font=fonts["title"])
    draw.text((36, 86), detail, fill=(218, 229, 239), font=fonts["detail"])
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    picture.save(path)
    return str(path)


# --------------------------------------------------------------------------- --start

def unstack_rehearsal(backend, ids, P, B=None, points=None, kinds=None, tries_per_step=3):
    """Clear the settled start top-down (user, 2026-09-23): repeatedly lift away a dish nothing rests on --
    dishes resting on another first -- park it off-scene, settle the per-move 150 + 60 ticks and judge the
    remaining dishes by the EPISODE's rule (2026-09-28): a dish displaced > 10 mm / 20 deg is a recorded nudge
    unless it leaves the counter band and the racks, which is what would abort an episode ("disturbed"; nothing
    is at its goal during generation). A removal that disturbs a neighbour is undone (every dish teleported back
    to the pre-removal state and re-settled, which must reproduce it) and the next free dish is tried, up to
    ``tries_per_step``: accepted = some clearing order exists. Returns (ok, record). Before, any displaced dish
    failed the rehearsal, so starts the episode would accept were re-rolled."""
    from dishsim.rearrange import DISTURB_POS_M, DISTURB_ROT_DEG, INIT_MATCH_POS_M, INIT_MATCH_ROT_DEG

    def off_scene(oid, pose):
        """Left the counter and the racks (the episode's fatal displacement); any displacement when no geometry."""
        if B is None or points is None or kinds is None:
            return True
        now = backend.poses()
        frames = {r: now[r] for r in ("LowerRack", "UpperRack")}
        return not P.in_counter_band(P.pose_T(pose), B.COUNTER) and B.racked_in(points[kinds[oid]], pose, frames) is None
    remaining, order, worst, undone, nudged = list(ids), [], (0., 0.), [], []
    for step in range(len(ids)):
        snap = backend.snapshot()
        support = P.support_edges(sorted(backend.latest_contact["pairs"]), snap["poses"], remaining)
        supporters = {a for a, _ in support}
        supported = {b for _, b in support}
        free = sorted((i for i in remaining if i not in supporters), key=lambda i: (i not in supported, i))
        if not free:
            return False, {"order": order, "undone": undone, "stuck": remaining, "reason": "every remaining dish supports another"}
        for oid in free[:tries_per_step]:
            pre = {i: snap["poses"][i] for i in remaining}
            backend.set_rigid_pose(backend.objects[oid], {"position_m": [3. + .4 * step, 2., .3], "quaternion_xyzw": [0., 0., 0., 1.]})
            for _ in range(SETTLE_TICKS + WINDOW_TICKS):
                backend.tick()
            post = backend.poses()
            moved = {i: pose_distance(pre[i], post[i]) for i in pre if i != oid}
            displaced = sorted(i for i, (dp, dr) in moved.items() if dp > DISTURB_POS_M or dr > DISTURB_ROT_DEG)
            bad = [i for i in displaced if off_scene(i, post[i])]
            top = max(moved.values(), key=lambda d: d[0] + d[1] / 2000, default=(0., 0.))
            if not bad:
                nudged += [{"step": step, "lifted": oid, "nudged": i, "mm": round(moved[i][0] * 1e3, 1), "deg": round(moved[i][1], 1)}
                           for i in displaced]
                break
            undone.append({"step": step, "lifted": oid, "disturbed": bad, "max_mm": round(max(moved[i][0] for i in bad) * 1e3, 1)})
            for i in remaining:                          # undo: the whole pre-removal state, then re-settle
                backend.set_rigid_pose(backend.objects[i], pre[i])
            for _ in range(RESET_TICKS):
                backend.tick()
            back = backend.poses()
            if any(pose_distance(pre[i], back[i])[0] > INIT_MATCH_POS_M or pose_distance(pre[i], back[i])[1] > INIT_MATCH_ROT_DEG
                   for i in remaining):
                return False, {"order": order, "undone": undone, "reason": f"undoing the removal of {oid} did not reproduce the pile"}
        else:
            last = undone[-1]
            return False, {"order": order, "undone": undone,
                           "reason": f"no free dish lifts cleanly at step {step} (last: {last['lifted']} disturbs {last['disturbed']}, {last['max_mm']} mm)"}
        worst = max(worst, top, key=lambda d: d[0] + d[1] / 2000)
        order.append({"removed": oid, "was_resting": oid in supported, "max_neighbour_mm": round(top[0] * 1e3, 1),
                      "max_neighbour_deg": round(top[1], 1)})
        remaining.remove(oid)
    return True, {"order": order, "undone": undone, "nudged": nudged, "rule": "episode: displaced and off the counter/racks",
                  "max_neighbour_mm": round(worst[0] * 1e3, 1), "max_neighbour_deg": round(worst[1], 1)}


def start(args, B, report):
    import numpy as np
    from dishsim_frigidaire import planner as P
    from dishsim_frigidaire.initial_state_candidates import COMPONENT_NAMES
    from dishsim_frigidaire.loading import visual_points
    from dishsim_frigidaire.random_poses import compose_pose, relative_pose, sample_pose, source_geometry_domains

    tier, seed, attempt = args.tier, args.seed, args.attempt
    rng = np.random.default_rng([seed, B.TIER_INDEX[tier], 1000 + attempt])
    n = B.draw_n(tier, seed)
    cap = B.cap_of(tier, n)
    ids = B.roster(tier)
    kinds = dict(ids)
    counter_ids = sorted(ids[i][0] for i in rng.permutation(len(ids))[:n])
    baseline = json.loads(Path(args.baseline).read_text())
    baseline = baseline.get("snapshot", baseline)
    frames = {name: baseline["poses"][name] for name in COMPONENT_NAMES}
    report.update(tier=tier, seed=seed, attempt=attempt, n=n, cap=cap, counter_ids=counter_ids)
    checker = B.bench_checker()
    checker.update_components(frames)
    slab = P.slab_body(checker, B.COUNTER)
    points = {kind: visual_points(B.ASSETS / f"{kind}.usda") for kind in B.KINDS}
    rack_of = {oid: "Counter" if oid in counter_ids else "LowerRack" if kind == "plate" else str(rng.choice(["LowerRack", "UpperRack"]))
               for oid, kind in ids}
    order = [oid for oid, _ in ids if oid not in counter_ids] + list(counter_ids)    # rack drops, then the pile
    park = {"position_m": [3., 0., .3], "quaternion_xyzw": [0., 0., 0., 1.]}
    entries = [{"object_id": oid, "kind": kinds[oid], "rack": "LowerRack" if rack_of[oid] == "Counter" else rack_of[oid],
                "pose_world": park} for oid in order]
    backend, _ = make_backend(args, B, entries, args.attempt_dir, counter_ids)
    if backend.prepare_baseline(baseline)["result"] != "PASS":
        report.update(result="REROLL", reason="the extended baseline did not hold")
        return
    backend.loaded, backend.previous_frames, backend.phase = True, None, "bench_build"
    backend.tracker.clear()
    backend.maximum_penetration_m, backend.peak_event, backend.global_peak_event = 0., None, None
    bodies, drops = [], {}

    def clear(body, statics):
        return ((not statics or checker.against_components(body)["valid"]) and checker.pair(body, slab)["valid"]
                and all(checker.pair(body, other)["valid"] for other in bodies))

    domains = source_geometry_domains()["sampling_bounds_by_rack"]
    top = B.COUNTER["top_z_m"]
    # Counter dishes as messy STACKS (a random heap of these thin shells interlocks: 0 of 6 heaps at coverage
    # 0.5-1.0 could be cleared without moving a neighbour > 10 mm, 2026-09-23): one kind per stack -- plates on
    # plates, bowls nested (a bowl stack face-down half the time) -- 1..STACK_MAX high, cups alone (up or down).
    # Every dish gets its own yaw, an offset <= STACK_OFFSET_M and a tilt <= STACK_TILT_DEG.
    stacks = []
    for kind in B.KINDS:
        pool = [oid for oid in counter_ids if kinds[oid] == kind]
        pool = [pool[i] for i in rng.permutation(len(pool))]
        while pool:
            h = 1 if kind == "cup" else int(rng.integers(1, min(STACK_MAX, len(pool)) + 1))
            stacks.append({"kind": kind, "ids": pool[:h], "inverted": kind != "plate" and bool(rng.random() < .5)})
            pool = pool[h:]
    stacks = [stacks[i] for i in rng.permutation(len(stacks))]
    radius = {kind: float(np.abs(points[kind][:, :2]).max()) for kind in B.KINDS}
    rim = max((radius[s["kind"]] for s in stacks), default=0.) + PILE_EDGE_M
    footprint = sum(np.pi * radius[s["kind"]] ** 2 for s in stacks)
    cov = args.pile_coverage
    hy = min(B.COUNTER["size_m"][1] / 2 - rim, (footprint / (cov * 4 * PILE_ASPECT)) ** .5)   # stack bases cluster;
    hx = min(B.COUNTER["size_m"][0] / 2 - rim, footprint / (cov * 4 * hy))                     # every rim on the slab
    report["pile"] = {"stacks": [{"kind": s["kind"], "ids": s["ids"], "inverted": s["inverted"]} for s in stacks],
                      "base_footprint_m2": round(footprint, 4), "box_half_m": [round(hx, 3), round(hy, 3)],
                      "base_coverage": round(footprint / (4 * hx * hy), 2), "offset_m": STACK_OFFSET_M, "tilt_deg": STACK_TILT_DEG,
                      "built": f"one dish at a time, {BUILD_TICKS} ticks each"}

    def dish_quat(inverted):
        yaw, axis, tilt = rng.uniform(0, 2 * np.pi), rng.uniform(0, 2 * np.pi), np.radians(rng.uniform(0, STACK_TILT_DEG))
        q = [0., 0., float(np.sin(yaw / 2)), float(np.cos(yaw / 2))]
        q = compose_pose([0, 0, 0], q, [0, 0, 0], [float(np.cos(axis) * np.sin(tilt / 2)), float(np.sin(axis) * np.sin(tilt / 2)), 0.,
                                                    float(np.cos(tilt / 2))])[1]
        return list(compose_pose([0, 0, 0], q, [0, 0, 0], [1., 0., 0., 0.] if inverted else [0., 0., 0., 1.])[1])

    def lowest(kind, q):
        return float((points[kind] @ B.quat_matrix(q).T)[:, 2].min())

    placements = [(oid, None) for oid in order if rack_of[oid] != "Counter"] + \
                 [(oid, (si, level)) for si, s in enumerate(stacks) for level, oid in enumerate(s["ids"])]
    base_xy = {}
    for oid, where in placements:            # each dish is dropped onto the SETTLED scene
        kind, rack = kinds[oid], rack_of[oid]
        now = backend.poses()
        bodies[:] = [checker._body(kinds[p], now[p], p) for p in drops]
        lifted, ok = 0., False
        if where is None:                    # a messy random drop into its rack
            lo, hi = np.asarray(domains[rack]["lower_m"]), np.asarray(domains[rack]["upper_m"])
            box = {"lower_m": [lo[0] + RACK_MARGIN_M, lo[1] + RACK_MARGIN_M, lo[2] + RACK_DROP_Z_M[0]],
                   "upper_m": [hi[0] - RACK_MARGIN_M, hi[1] - RACK_MARGIN_M, lo[2] + RACK_DROP_Z_M[1]]}
            frame = frames[rack]
            for _ in range(TRIES):
                sampled = sample_pose(rng, points[kind], box)
                local = {"position_m": list(sampled["position_m"]), "quaternion_xyzw": sampled["quaternion_xyzw"]}
                lifted = 0.
                while True:
                    p, q = compose_pose(frame["position_m"], frame["quaternion_xyzw"], local["position_m"], local["quaternion_xyzw"])
                    pose = {"position_m": p.tolist(), "quaternion_xyzw": q.tolist()}
                    ok = clear(checker._body(kind, pose, oid), True)
                    if ok or lifted >= RACK_LIFT_MAX_M:
                        break
                    lifted += LIFT_STEP_M
                    local["position_m"][2] += LIFT_STEP_M
                if ok:
                    break
        else:
            si, level = where
            s = stacks[si]
            for _ in range(TRIES):
                q = dish_quat(s["inverted"])
                if level == 0:               # the stack's base: on the slab, clear of every other stack
                    xy = rng.uniform([-hx, -hy], [hx, hy])
                    pose = {"position_m": [float(xy[0]), float(xy[1]), top + STACK_DROP_M - lowest(kind, q)], "quaternion_xyzw": q}
                    ok = clear(checker._body(kind, pose, oid), False)
                    if ok:
                        base_xy[si] = xy
                else:                        # onto the settled dish below: start low, lift until clear, drop
                    below = now[s["ids"][level - 1]]
                    r, a = STACK_OFFSET_M * np.sqrt(rng.random()), rng.uniform(0, 2 * np.pi)
                    xy = base_xy[si] + [r * np.cos(a), r * np.sin(a)]
                    z = float(below["position_m"][2]) + STACK_DROP_M
                    lifted = 0.
                    while True:
                        pose = {"position_m": [float(xy[0]), float(xy[1]), z + lifted], "quaternion_xyzw": q}
                        ok = clear(checker._body(kind, pose, oid), False)
                        if ok or lifted >= STACK_LIFT_MAX_M:
                            break
                        lifted += STACK_LIFT_STEP_M
                if ok:
                    break
        if not ok:
            raise RuntimeError(f"could not place {oid} ({rack}{'' if where is None else f', stack {where}'}) without overlap")
        backend.set_rigid_pose(backend.objects[oid], pose)
        for _ in range(BUILD_TICKS):
            backend.tick()
        drops[oid] = {"rack": rack, "lifted_m": round(lifted, 3)} if where is None else \
                     {"rack": "Counter", "stack": where[0], "level": where[1], "inverted": stacks[where[0]]["inverted"], "lifted_m": round(lifted, 3)}
    for _ in range(BUILD_TICKS):
        backend.tick()
    # The scene PhysX built is the start. No FCL preflight (its 1 mm allowance is for sampled poses; resting hulls
    # overlap slightly more), and landing transients are recorded, not gated: a thin HOTEC shell landing on another
    # measured 6 mm contact depth for a few ticks (2026-09-23). The gate is the state itself: the backend's
    # settle-and-observation hold, whose 1 s rest windows over 5 s must keep peak AND median contact depth under the
    # LIMITS (the same thresholds), then reproduction and the unstack rehearsal below.
    from dishsim_frigidaire.initial_state_runtime import OBSERVATION_SECONDS, RACKS
    build_peak, build_event = backend.maximum_penetration_m, backend.global_peak_event
    backend.maximum_penetration_m, backend.peak_event, backend.global_peak_event = 0., None, None
    hold = backend.hold(phase="bench_start_settle_and_observation", goal=backend.goal(RACKS), timeout=12.,
                        observation=OBSERVATION_SECONDS, abort_on_peak=False)
    report["build_transient_penetration_m"], report["build_transient_event"] = build_peak, build_event
    from dishsim_frigidaire.random_poses import motion_is_settled
    report["hold"] = {**{key: hold.get(key) for key in ("passed", "settled", "contacts_ok", "dish_supported", "endpoints_ok",
                                                        "unsupported_object_ids", "peak_penetration_m", "simulated_seconds")},
                      "moving": sorted(name for name, v in (hold.get("motion") or {}).items() if not motion_is_settled(v))}
    record = {"settled": hold, "maximum_settle_penetration_m": backend.maximum_penetration_m,
              "maximum_settle_penetration_event": backend.global_peak_event, "initial_snapshot": backend.snapshot()}
    if not hold["passed"]:
        record.update(outcome="settle_failure", reason=hold.get("reason"))
    else:
        record.update(outcome="settled", reason="built one dish at a time; 5 s rest windows passed (contacts, motion, support)")
    report.update(outcome=record["outcome"], backend_reason=record.get("reason"),
                  maximum_settle_penetration_m=record.get("maximum_settle_penetration_m"),
                  maximum_settle_penetration_event=record.get("maximum_settle_penetration_event"))
    if record["outcome"] != "settled":
        report.update(result="REROLL", reason=f"backend {record['outcome']}: {record.get('reason')}")
        return
    settled = record["initial_snapshot"]["poses"]
    # reproduction gate: teleport back to the settled poses and re-settle (the Bosch reset semantics)
    for oid, _ in ids:
        backend.set_rigid_pose(backend.objects[oid], settled[oid])
    for _ in range(RESET_TICKS):
        backend.tick()
    snap = backend.snapshot()
    worst = max((pose_distance(settled[oid], snap["poses"][oid]) for oid, _ in ids), key=lambda d: d[0] + d[1] / 1500)
    report["reproduction"] = {"max_pos_mm": round(worst[0] * 1e3, 2), "max_rot_deg": round(worst[1], 2)}
    if any(pose_distance(settled[oid], snap["poses"][oid])[0] > .010 or pose_distance(settled[oid], snap["poses"][oid])[1] > 15.
           for oid, _ in ids):
        report.update(result="REROLL", reason=f"start does not reproduce: {report['reproduction']}")
        return
    rack_frames = {rack: snap["poses"][rack] for rack in ("LowerRack", "UpperRack")}
    objects = []
    for oid, kind in ids:
        pose = snap["poses"][oid]
        T = P.pose_T(pose)
        on_counter = P.in_counter_band(T, B.COUNTER)
        if oid in counter_ids and not on_counter:
            report.update(result="REROLL", reason=f"{oid} left the counter: {np.round(pose['position_m'], 3).tolist()}")
            return
        rack = None if on_counter else B.racked_in(points[kind], pose, rack_frames)
        if not on_counter and rack is None:
            report.update(result="REROLL", reason=f"{oid} is neither racked nor on the counter")
            return
        o = {"object_id": oid, "kind": kind, "start": "Counter" if on_counter else rack, "pose_world": pose,
             "source": drops[oid]}
        if rack is not None:
            o["rack"] = rack
            o["rack_local_pose"] = P.local_from_world(rack_frames[rack], pose)
        objects.append(o)
    band = sum(o["start"] == "Counter" for o in objects)
    if band != n:
        report.update(result="REROLL", reason=f"{band} dishes in the counter band, drew n = {n}")
        return
    support = P.support_edges(snap["contact_pairs"], snap["poses"], [o["object_id"] for o in objects])
    ok, unstack = unstack_rehearsal(backend, [oid for oid, _ in ids], P, B, points, kinds)   # mutates the scene: snap kept above
    report["unstack"] = unstack
    if not ok:
        report.update(result="REROLL", reason=f"unstack rehearsal: {unstack['reason']}")
        return
    start_record = {
        "schema_version": 1, "purpose": "hotec_bench_start", "instance_id": f"{tier}_s{seed}", "tier": tier, "seed": seed,
        "attempt": attempt, "n_objects": len(objects), "inventory": B.TIERS[tier]["inventory"],
        "tableware": B.tableware(), "tableware_sha256": {k: B.digest(v) for k, v in B.tableware().items()},
        "counter": {**{k: list(v) if isinstance(v, tuple) else v for k, v in B.COUNTER.items()}, "start_count": n,
                    "slack": B.TIERS[tier]["slack"], "cap": cap, "cells_pitch_m": B.CELL_PITCH_M, "pile": report["pile"]},
        "objects": objects,
        "initial_snapshot": {"joints": snap["joints"], "poses": snap["poses"], "step": snap["step"], "contact_pairs": snap["contact_pairs"]},
        "support": support, "baseline": baseline,
        "validation": {"start": {"outcome": record["outcome"], "settled": {k: record["settled"].get(k) for k in ("passed", "reason", "simulated_seconds")},
                                 "maximum_settle_penetration_m": record.get("maximum_settle_penetration_m")},
                       "reproduction": report["reproduction"], "unstack": unstack},
        "hashes": {"appliance": checker.asset_sha256},
        "generated_utc": datetime.now(timezone.utc).isoformat()}
    path = args.attempt_dir / "start.json"
    path.write_text(json.dumps(start_record, indent=1) + "\n")
    report.update(result="PASS", start=str(path), support_edges=len(support), stacked=sorted({b for _, b in support}))


# --------------------------------------------------------------------------- --sequence

def build_sequence(backend, B, P, by_id, targets, centroids, landing):
    """Build a load in Isaac one dish at a time, lowest centre first (a valid order for "rests on"), each commanded
    at its target pose + the goal hover and settled like a move (150 + 60 ticks): every dish must pass ``landing``
    (the goal: within the at-goal tolerance of its jointly settled pose; an own load: within the episode's settle
    deviation of its commanded pose), with < 2 mm contact depth, and no dish already placed may move
    > 10 mm / 20 deg. Dense HOTEC loads lean on each other (shingled plates, tilted upper bowls), so "any order" is
    impossible (a solo-clearance rule banned the standard upper bowl pair and adjacent plates); this certifies ONE
    order. The built poses become the targets, and their support edges become the goal-order constraints the
    planners see. ``targets`` = {id: world pose dict}; returns (status, record, bans)."""
    import numpy as np
    from dishsim import rearrange as R
    from dishsim_frigidaire.random_poses import LIMITS
    height = {i: float(targets[i]["position_m"][2]) for i in by_id}
    centre = {i: np.asarray(targets[i]["position_m"], dtype=float) for i in by_id}
    near = {i: [j for j in by_id if j != i and by_id[j]["rack"] == by_id[i]["rack"]
                and float(np.linalg.norm(centre[j] - centre[i])) < SEQUENCE_NEIGHBOUR_M] for i in by_id}

    def topo(constraints):
        """Kahn's order: every learned (a before b) holds, ties lowest centre first; None on a cycle."""
        import heapq
        indeg = {i: 0 for i in by_id}
        for a, b in constraints:
            indeg[b] += 1
        heap = [(height[i], i) for i, d in indeg.items() if d == 0]
        heapq.heapify(heap)
        out = []
        while heap:
            _, i = heapq.heappop(heap)
            out.append(i)
            for a, b in constraints:
                if a == i:
                    indeg[b] -= 1
                    if indeg[b] == 0:
                        heapq.heappush(heap, (height[b], b))
        return out if len(out) == len(by_id) else None

    def build(order):
        for j, i in enumerate(by_id):                      # clear the racks: every dish parked apart
            backend.set_rigid_pose(backend.objects[i], {"position_m": [3. + .4 * j, 2., .3], "quaternion_xyzw": [0., 0., 0., 1.]})
        for _ in range(60):
            backend.tick()
        placed, steps = [], []
        for oid in order:
            e = by_id[oid]
            T_goal = P.pose_T(targets[oid])
            T_cmd = T_goal.copy()
            T_cmd[2, 3] += B.GOAL_HOVER_M
            pre = {i: P.pose_T(backend.poses()[i]) for i in placed}
            backend.set_rigid_pose(backend.objects[oid], P.pose_dict(T_cmd))
            for _ in range(SETTLE_TICKS + WINDOW_TICKS):
                backend.tick()
            now = backend.poses()
            dist = B.goal_distance(centroids[e["kind"]], P.pose_T(now[oid]), T_goal)
            dev = float(np.linalg.norm(np.asarray(now[oid]["position_m"]) - T_cmd[:3, 3]))
            deep = max((d for name, d in backend.latest_contact["depths_m"].items() if oid in name.split("|")), default=0.)
            moved = R.disturbed_items({**pre, oid: T_goal}, {i: P.pose_T(now[i]) for i in [*placed, oid]}, oid)
            steps.append({"id": oid, "lateral_mm": round(dist[0] * 1e3, 1), "dz_mm": round(dist[1] * 1e3, 1),
                          "tilt_deg": round(dist[2], 1), "settle_dev_mm": round(dev * 1e3, 1),
                          "contact_mm": round(deep * 1e3, 2), "disturbed": moved})
            if moved:
                return "moved", oid, moved, steps
            if not landing(e["kind"], dist, dev) or deep >= LIMITS["peak_penetration_m"]:
                return "landing", oid, None, steps
            placed.append(oid)
        return "ok", None, None, steps

    learned, builds, status, bans = set(), [], None, []
    order = topo(learned)
    for _ in range(SEQUENCE_BUILDS):
        status, x, moved, steps = build(order)
        builds.append({"order": order, "status": status, "dish": x, "moved": moved, "steps": steps})
        if status == "ok":
            break
        if status == "moved":                               # x must go in before what it knocked
            learned |= {(x, y) for y in moved}
        else:                                               # x did not land: it leans on its neighbours, so
            later = [y for y in near[x] if (x, y) not in learned and (y, x) not in learned]   # they go in first
            if not later:
                break
            learned |= {(y, x) for y in later}
        order = topo(learned)
        if order is None:
            break
    record = {"status": status, "builds": builds, "learned": sorted(learned)}
    if status == "ok":
        pairs = set()
        for _ in range(60):                                  # contacts flicker: the union over 0.5 s
            backend.tick()
            pairs.update(tuple(p) for p in backend.latest_contact["pairs"])
        final = backend.poses()
        pos = {i: n for n, i in enumerate(order)}
        contact = [tuple(e) for e in P.support_edges(sorted(pairs), final, order) if pos[e[0]] < pos[e[1]]]
        record.update(order=order, poses_world={oid: final[oid] for oid in order},
                      goal_order=sorted(set(contact) | learned),
                      reason=f"built in {len(builds)} build(s), {len(set(contact) | learned)} order constraints "
                             f"({len(learned)} learned), max lateral {max(s['lateral_mm'] for s in steps)} mm")
    else:
        last = builds[-1]
        e = by_id[last["dish"]] if last["dish"] else None
        if last["status"] == "moved" and e is not None:
            bans += [sorted((B.cand_key(e), B.cand_key(by_id[m]))) for m in last["moved"] if not by_id[m].get("keep") and not e.get("keep")]
        elif e is not None and not e.get("keep"):
            bans.append([B.cand_key(e)])
        record["reason"] = (f"no order built the load in {len(builds)} build(s): last {last['status']} "
                            f"at {last['dish']} {last['moved'] or ''}")
    return status, record, bans


def _sequence_scene(args, B, entries, baseline, scene_dir, report):
    """Every dish parked apart in an extended, empty baseline; None when the baseline does not hold."""
    park = {"position_m": [3., 0., .3], "quaternion_xyzw": [0., 0., 0., 1.]}
    backend, _ = make_backend(args, B, [{"object_id": e["id"], "kind": e["kind"], "rack": e["rack"], "pose_world": park}
                                        for e in entries], scene_dir)
    if backend.prepare_baseline(baseline)["result"] != "PASS":
        report.update(result="REROLL", reason="the extended baseline did not hold")
        return None
    backend.loaded = True
    return backend


def sequence_gate(args, B, report):
    """--sequence ATTEMPT_DIR: certify the instance GOAL (the goal gate's jointly settled poses are the targets;
    landing = the at-goal tolerance). Writes sequence.json and, on failure, the banned poses/pairs (bans.json)."""
    from dishsim_frigidaire import planner as P
    from dishsim_frigidaire.loading import visual_points
    adir = Path(args.attempt_dir)
    goal = json.loads((adir / "goal.json").read_text())
    gate = json.loads((adir / "gate" / "result.json").read_text())
    start_rec = json.loads((adir / "start.json").read_text())
    joint = gate["initial_snapshot"]["poses"]
    centroids = B.kind_centroids({kind: visual_points(B.ASSETS / f"{kind}.usda") for kind in B.KINDS})
    backend = _sequence_scene(args, B, goal["entries"], start_rec["baseline"], adir / "sequence_scene", report)
    if backend is None:
        return
    by_id = {e["id"]: e for e in goal["entries"]}
    status, record, bans = build_sequence(backend, B, P, by_id, {i: joint[i] for i in by_id}, centroids,
                                          lambda kind, dist, dev: B.within_goal(kind, dist))
    report.update(result="PASS" if status == "ok" else "REROLL", reason=record["reason"])
    (adir / "sequence.json").write_text(json.dumps(record, indent=1) + "\n")
    if bans:
        path = adir / "bans.json"
        data = json.loads(path.read_text()) if path.is_file() else {"outcome": "sequence", "pairs": []}
        data["pairs"] += bans
        path.write_text(json.dumps(data, indent=1) + "\n")


def sequence_plan(args, B, report):
    """--sequence-plan PLAN --instance INSTANCE: certify an open-track algorithm's OWN load the way the goal is
    certified (2026-09-28): its commanded rack-local poses are built one dish at a time in the instance's
    extended racks; landing = the episode's settle deviation (DEV_MAX_M) and contact depth, since a commanded
    family pose (lifted up to 60 mm) settles well past the at-goal tolerance. The built poses and the learned
    order go to <plan>.sequence.json; the Kit-free --resequence stage turns them into the certified plan. Before
    this, own loads were scored on paper and replayed blind: a plate leaning 24 deg was pulled through a placed
    neighbour and every medium/hard open-track row failed 'disturbed' (pilot, 2026-09-23)."""
    from dishsim_frigidaire import planner as P
    from dishsim_frigidaire.loading import visual_points
    plan_path = Path(args.sequence_plan)
    plan = json.loads(plan_path.read_text())
    inst = json.loads(Path(args.instance).read_text())
    out = plan_path.with_name(plan_path.name[:-len(".json")] + ".sequence.json")
    entries = plan.get("goal") or []
    if not entries or not plan.get("sequenced"):
        record = {"status": "no_load", "reason": f"{plan['algorithm']} found no complete, sequenceable load"}
        out.write_text(json.dumps(record, indent=1) + "\n")
        report.update(result="REROLL", reason=record["reason"])
        return
    if args.gate:                        # the load passed the joint Isaac gate: build from its SETTLED poses, like the goal
        gate = json.loads((Path(args.gate) / "result.json").read_text())
        if gate.get("outcome") != "accepted":
            record = {"status": "gate", "reason": f"the load's joint gate was {gate.get('outcome')}: {gate.get('reason')}"}
            out.write_text(json.dumps(record, indent=1) + "\n")
            report.update(result="REROLL", reason=record["reason"])
            return
        joint = gate["initial_snapshot"]["poses"]
        targets = {e["id"]: joint[e["id"]] for e in entries}
        landing, rule = (lambda kind, dist, dev: B.within_goal(kind, dist)), "at-goal tolerance of the jointly settled pose"
    else:                                # no joint gate: the commanded family poses themselves (a lifted pose drops)
        frames = {r: inst["initial_snapshot"]["poses"][r] for r in ("LowerRack", "UpperRack")}
        targets = {e["id"]: P.world_from_local(frames[e["rack"]], {"position_m": e["position"], "quaternion_xyzw": e["quaternion_xyzw"]})
                   for e in entries}
        landing, rule = (lambda kind, dist, dev: dev <= DEV_MAX_M), "settle deviation <= DEV_MAX_M of the commanded pose"
    centroids = B.kind_centroids({kind: visual_points(B.ASSETS / f"{kind}.usda") for kind in B.KINDS})
    backend = _sequence_scene(args, B, entries, inst["baseline"], args.out / "plans" / "_kit" / plan_path.stem, report)
    if backend is None:
        return
    by_id = {e["id"]: e for e in entries}
    status, record, bans = build_sequence(backend, B, P, by_id, targets, centroids, landing)
    record.update(plan=plan_path.name, instance=inst["instance_id"], algorithm=plan["algorithm"], bans=bans,
                  gate=str(args.gate) if args.gate else None,
                  landing=f"{rule}, contact depth < 2 mm; no placed dish moved > 10 mm / 20 deg")
    out.write_text(json.dumps(record, indent=1) + "\n")
    report.update(result="PASS" if status == "ok" else "REROLL", reason=record["reason"], sequence=str(out))


# --------------------------------------------------------------------------- --run

class Timed:
    """Wall and CPU time of an algorithm's reset (where offline planners plan) plus every next_move; the driver
    itself times next_move only. CPU = the calling thread's (Kit's own threads run meanwhile)."""

    def __init__(self, algo):
        self.algo, self.wall, self.cpu, self.reset_wall = algo, 0., 0., 0.

    def _timed(self, fn, *a):
        w, c = time.perf_counter(), time.thread_time()
        out = fn(*a)
        self.wall += time.perf_counter() - w
        self.cpu += time.thread_time() - c
        return out

    def reset(self, instance, world):
        before = self.wall
        self._timed(self.algo.reset, instance, world)
        self.reset_wall = self.wall - before

    def next_move(self, obs):
        return self._timed(self.algo.next_move, obs)

    def stats(self):
        """The wrapped planner's search statistics (rrt: nodes, samples, extends, replans) reach the record."""
        return self.algo.stats() if hasattr(self.algo, "stats") else {}


class BenchOracle:
    """Teleport, settle, judge on the IsaacInitialStateBackend scene: rearrange.settle_move's verdicts, thresholds
    and put-back, except that a displaced neighbour is FATAL only when it was at its goal before the move or ends
    outside the counter and the racks; any other displaced dish (a messy start dish) is a recorded, non-fatal
    "nudge". Messy rack drops lean on each other, so the Bosch rule (any neighbour > 10 mm / 20 deg is fatal)
    aborted nearly every episode within 1-6 moves on interactions no algorithm can see (pilot, 2026-09-23)."""

    def __init__(self, backend, ids, kinds, world, centroids, goal=None, points=None):
        self.backend, self.ids, self.kinds, self.world, self.centroids, self.goal = backend, ids, kinds, world, centroids, goal
        self.points = points

    def _poses(self):
        from dishsim_frigidaire import planner as P
        p = self.backend.poses()
        return {i: P.pose_T(p[i]) for i in self.ids}

    def poses(self):
        return self._poses()

    def measured(self, item_id):
        from dishsim_frigidaire import planner as P
        return P.pose_T(self.backend._pose(self.backend.objects[item_id].data.root_pos_w[0],
                                           self.backend.objects[item_id].data.root_quat_w[0]))

    def teleport(self, item_id, T):
        from dishsim_frigidaire import planner as P
        self.backend.set_rigid_pose(self.backend.objects[item_id], P.pose_dict(T))

    def step(self, n):
        for _ in range(n):
            self.backend.tick()

    def at_goal(self, item, T):
        import frigidaire_bench as B
        if self.goal is None:
            return False
        kind = self.kinds[item["item_id"]]
        return B.within_goal(kind, B.goal_distance(self.centroids[kind], T, self.goal[item["item_id"]]))

    def _fatal(self, moved, protected, poses):
        import frigidaire_bench as B
        from dishsim_frigidaire import planner as P
        snap = self.backend.poses()
        frames = {r: snap[r] for r in ("LowerRack", "UpperRack")}
        return [k for k in moved if k in protected or (B.racked_in(self.points[self.kinds[k]], P.pose_dict(poses[k]), frames) is None
                                                        and not self.world.in_counter(poses[k]))]

    def settle(self, move):
        import numpy as np
        from dishsim import rearrange as R
        protected = {k for k in self.ids if k != move.item_id and self.world.at_own_goal(k)}
        pre = self._poses()
        self.teleport(move.item_id, move.T_base_obj)
        hist = []
        for s in range(SETTLE_TICKS):
            self.step(1)
            if s >= SETTLE_TICKS - WINDOW_TICKS:
                hist.append(self.measured(move.item_id))
        poses = self._poses()
        drift_p = float(np.linalg.norm(hist[-1][:3, 3] - hist[0][:3, 3]))
        drift_deg = R.rot_angle_deg(hist[0], hist[-1])
        dev_p = float(np.linalg.norm(poses[move.item_id][:3, 3] - np.asarray(move.T_base_obj)[:3, 3]))
        moved = R.disturbed_items(pre, poses, move.item_id)
        fatal = self._fatal(moved, protected, poses)
        # the gates' contact-depth limit, per move: a placement that leaves the moved dish wedged > 2 mm into
        # anything is put back (every finished load failed the end check with 7-9 mm resting depth otherwise)
        from dishsim_frigidaire.random_poses import LIMITS
        deep = max((d for name, d in self.backend.latest_contact["depths_m"].items() if move.item_id in name.split("|")),
                   default=0.)
        info = {"settle_dev_mm": round(dev_p * 1e3, 1), "drift_mm": round(drift_p * 1e3, 1), "contact_mm": round(deep * 1e3, 2),
                "disturbed": fatal, "nudged": [k for k in moved if k not in fatal]}
        if fatal:
            return poses, "disturbed", info
        if (drift_p > R.STABLE_POS_M or drift_deg > R.STABLE_ROT_DEG or dev_p > DEV_MAX_M
                or deep >= LIMITS["peak_penetration_m"]):
            self.teleport(move.item_id, pre[move.item_id])          # the Bosch put-back
            for _ in range(SETTLE_TICKS):
                self.step(1)
            poses = self._poses()
            back_dp = float(np.linalg.norm(poses[move.item_id][:3, 3] - pre[move.item_id][:3, 3]))
            back_dr = R.rot_angle_deg(pre[move.item_id], poses[move.item_id])
            info["teleport_back_mm"] = round(back_dp * 1e3, 1)
            again = R.disturbed_items(pre, poses, move.item_id)
            refatal = self._fatal(again, protected, poses)
            info["nudged"] += [k for k in again if k not in refatal and k not in info["nudged"]]
            if refatal:
                info["disturbed"] = refatal
                return poses, "disturbed", info
            if back_dp > R.INIT_MATCH_POS_M or back_dr > R.INIT_MATCH_ROT_DEG:
                return poses, "unstable-settle", info
            return poses, "failed-settle", info
        return poses, None, info

    def execute(self, move):
        from dishsim_frigidaire import planner as P
        resting = self.world.resting_on(move.item_id)
        if resting:
            return self._poses(), "disturbed", {"reason": "support", "resting": resting}
        poses, fault, info = self.settle(move)
        if fault != "failed-settle":                            # a put-back dish keeps its previous command
            self.world.note_command(move.item_id, move.T_base_obj)
        snap = self.backend.poses()
        self.world.support = [tuple(e) for e in P.support_edges(self.backend.latest_contact["pairs"], snap, self.ids)]
        self.world.initial = {k: v.copy() for k, v in poses.items()}
        info["poses_after"] = {i: P.pose_dict(poses[i]) for i in self.ids}
        return poses, fault, info


def run(args, B, report):
    import numpy as np
    from dishsim import rearrange
    from dishsim_frigidaire import planner as P
    from dishsim_frigidaire.loading import visual_points
    from dishsim_frigidaire.random_pose_runtime import EXTENSION, RACK_JOINT

    inst = json.loads(Path(args.instance).read_text())
    iid, tier = inst["instance_id"], inst["tier"]
    ids = [o["object_id"] for o in inst["objects"]]
    kinds = {o["object_id"]: o["kind"] for o in inst["objects"]}
    counter_ids = [o["object_id"] for o in inst["objects"] if o["start"] == "Counter"]
    points = {kind: visual_points(B.ASSETS / f"{kind}.usda") for kind in B.KINDS}
    centroids = B.kind_centroids(points)
    entries = [{"object_id": o["object_id"], "kind": o["kind"], "rack": o.get("rack", "LowerRack"), "pose_world": o["pose_world"]}
               for o in inst["objects"]]
    work = args.out / "episodes" / "_kit" / iid
    backend, rig = make_backend(args, B, entries, work, counter_ids, cameras=args.cameras)
    stills = ROOT / "data/media/benchmark/frigidaire_hotec/stills" / tier / iid
    checker = B.bench_checker()
    baseline = inst["baseline"]
    start_poses = inst["initial_snapshot"]["poses"]
    report.update(instance=iid, tier=tier, episodes=[])

    reset_pairs = set()

    def reset():
        backend.restore(baseline)
        for _ in range(12):
            backend.tick()
        for oid in ids:
            backend.set_rigid_pose(backend.objects[oid], start_poses[oid])
        backend.loaded = True
        backend.assignments = {o["object_id"]: o["rack"] for o in inst["objects"] if o["start"] != "Counter"}
        reset_pairs.clear()
        for tick in range(RESET_TICKS):
            backend.tick()
            if tick >= RESET_TICKS - 60:                    # contacts flicker: take the union of the last 0.5 s
                reset_pairs.update(tuple(p) for p in backend.latest_contact["pairs"])
        now = backend.poses()
        worst = max(max(pose_distance(start_poses[oid], now[oid])[0] * 1e3, pose_distance(start_poses[oid], now[oid])[1] / 1.5)
                    for oid in ids)
        ok = all(pose_distance(start_poses[oid], now[oid])[0] <= .010 and pose_distance(start_poses[oid], now[oid])[1] <= 15. for oid in ids)
        return ok, round(worst, 2)

    def end_check():
        """Tub wall on the extended state, then both racks retracted and whole-mesh containment."""
        now = backend.poses()
        frames = {r: now[r] for r in ("LowerRack", "UpperRack")}
        racked = {oid: B.racked_in(points[kinds[oid]], now[oid], frames) for oid in ids}
        if any(v is None for v in racked.values()):
            return {"ran": False, "outcome": "not_all_racked", "racked": racked}
        tub = {oid: B.tub_clear(points[kinds[oid]], *(lambda l: (l["position_m"], l["quaternion_xyzw"]))(
            P.local_from_world(frames[racked[oid]], now[oid]))) for oid in ids}
        if not all(tub.values()):
            return {"ran": True, "outcome": "tub_wall", "racked": racked, "tub_clear": tub}
        backend.assignments = dict(racked)
        prepared = [{"object_id": oid, "kind": kinds[oid]} for oid in ids]
        pre = {oid: now[oid] for oid in ids}
        tried = []
        for order in (("UpperRack", "LowerRack"), ("LowerRack", "UpperRack")):
            rec = {"loaded_rack_motions": []}
            backend.maximum_penetration_m, backend.peak_event, backend.global_peak_event = 0., None, None  # clean record
            backend.close_and_contain(rec, order, prepared)
            deepest = sorted(backend.latest_contact["depths_m"].items(), key=lambda kv: -kv[1])[:3]
            tried.append({"order": list(order), "outcome": rec.get("outcome"), "reason": rec.get("reason"),
                          "deepest_contacts_mm": [[name, round(depth * 1e3, 2)] for name, depth in deepest],
                          "motions": [{key: m.get(key) for key in ("rack", "passed", "reason", "failed_step", "unsupported_object_ids",
                                                                    "maximum_penetration_m", "peak_measured_rack_speed_m_s")}
                                      for m in rec.get("loaded_rack_motions", [])]})
            if rec.get("outcome") != "closure_failure":
                break
            backend.ramp({RACK_JOINT[r]: EXTENSION[r] for r in ("LowerRack", "UpperRack")}, "reextend_after_flake")
            for oid in ids:
                backend.set_rigid_pose(backend.objects[oid], pre[oid])
            for _ in range(RESET_TICKS):
                backend.tick()
        return {"ran": True, "outcome": rec.get("outcome"), "reason": rec.get("reason"), "orders_tried": tried,
                "racked": racked, "tub_clear": tub,
                "containment": {k: v["contained"] for k, v in rec.get("final_containment", {}).items()},
                "rack_speed_flakes": sum(t["outcome"] == "closure_failure" for t in tried[:-1])}

    ok, worst = reset()
    report["reset_worst"] = worst
    still(backend, rig, stills / "initial.png", f"{iid}: initial configuration",
          f"{len(counter_ids)} dishes on the counter (allowance {inst['counter']['cap']}), {len(ids) - len(counter_ids)} in the racks")
    goal_world = {oid: g["settled_pose_world"] for oid, g in inst["goal"]["objects"].items()}
    for oid in ids:
        backend.set_rigid_pose(backend.objects[oid], goal_world[oid])
    for _ in range(60):
        backend.tick()
    still(backend, rig, stills / "goal.png", f"{iid}: sampled goal (S_ref {inst['goal']['S_ref']:.3f})",
          f"highest-exposure load found by coordinate ascent; {len(inst['goal']['keep'])} dishes kept in place")
    algorithms = {"greedy_offline": rearrange.OfflineGreedy}
    try:
        from dishsim import rrt
        algorithms["rrt_connect"] = rrt.RRTConnect
    except Exception as exc:                                   # noqa: BLE001
        report["rrt_import_error"] = repr(exc)
    for track, name in EPISODES:
        if args.algorithms and name not in args.algorithms:
            continue
        out = args.out / "episodes" / track / tier / f"{iid}__{name}.json"
        if out.exists():
            report["episodes"].append({"track": track, "algorithm": name, "skipped": "exists"})
            continue
        ok, worst = reset()
        if not ok:                                          # a stacked start can land ~10 mm off once; retry once
            ok, worst = reset()
        rinst = P.to_rearrange_instance(inst)
        world = B.make_bench_world(inst, checker)
        live = backend.poses()                              # the reset's live support (a stack edge can be missing
        world.support = sorted({tuple(e) for e in inst.get("support", [])} |   # from the one-tick start snapshot)
                               {tuple(e) for e in P.support_edges(sorted(reset_pairs), live, ids)})
        world.initial = {oid: P.pose_T(live[oid]) for oid in ids}
        world.sync(dict(world.initial), dict(kinds))       # the planners' support rule compares _poses with initial
        for it in rinst.items:                              # run_episode re-syncs the world to T_base_init: make that
            it["T_base_init"] = world.initial[it["item_id"]].copy()     # the live reset state as well
        targets = B.goal_targets(inst, world)
        for it in rinst.items:
            it["target"] = {"T_base_obj": targets[it["item_id"]]}
        world.mark_goals(targets, centroids)
        seed = int(__import__("hashlib").sha256(f"0|{iid}|{name}".encode()).hexdigest()[:8], 16)
        plan = None
        if track == "open":
            plan_path = args.out / "plans" / "open" / tier / f"{iid}__{name}.json"
            plan = json.loads(plan_path.read_text())
            if not plan.get("certified"):                   # the load never built one dish at a time in Isaac:
                rec = {"solved": False, "abort": "load-not-buildable", "moves": [], "moves_used": 0,   # the algorithm failed
                       "reason": plan.get("certification", {}).get("reason"), "planning_time_total_s": plan.get("planning_time_s"),
                       "planning_cpu_s": plan.get("planning_cpu_s"), "plan_goal": plan.get("goal"), "end_check": {"ran": False, "outcome": "aborted"}}
                rec.update(track=track, tier=tier, instance=iid, algorithm=name, seed=seed, counter_cap=inst["counter"]["cap"],
                           lower_bound=inst["goal"]["lower_bound"], S_ref=inst["goal"]["S_ref"], success=False, nudges=0)
                out.parent.mkdir(parents=True, exist_ok=True)
                out.write_text(json.dumps(rec, indent=1) + "\n")
                report["episodes"].append({"track": track, "algorithm": name, "success": False, "abort": rec["abort"]})
                continue
            world.goal_order = [tuple(e) for e in plan.get("goal_order", [])]   # the certified build's order
            algo = B.Replay(plan)
        elif name == "planner":
            world.goal_order = [tuple(e) for e in inst["goal"].get("order", [])]
            algo = B.FixedGoalSequencer()
        else:
            world.goal_order = [tuple(e) for e in inst["goal"].get("order", [])]
            cls = algorithms[name]
            try:
                algo = cls(seed=seed)
            except TypeError:
                algo = cls()
        goal_T = {oid: B.goal_targets(inst, world, hover=0.)[oid] for oid in ids} if track == "goal" else None
        oracle = BenchOracle(backend, ids, kinds, world, centroids, goal_T, points)
        if not ok:
            rec = {"solved": False, "abort": "init-mismatch", "reset_worst": worst}
        else:
            t0, timed = time.monotonic(), Timed(algo)
            rec = rearrange.run_episode(rinst, timed, world, oracle, budget=None, algorithm_name=name,
                                        time_budget_s=60., counter_cap=inst["counter"]["cap"])
            rec["wall_s"] = time.monotonic() - t0
            rec["planning_time_total_s"], rec["planning_cpu_s"], rec["planning_reset_s"] = timed.wall, timed.cpu, timed.reset_wall
        if plan is not None:
            rec["planning_time_total_s"], rec["planning_cpu_s"] = plan["planning_time_s"], plan.get("planning_cpu_s")
            rec["plan_goal"], rec["plan_sequenced"] = plan.get("goal"), plan.get("sequenced")
        final = backend.poses()
        rec["nudges"] = sum(len(m.get("nudged") or []) for m in rec.get("moves", []))
        rec["final_poses"] = {oid: final[oid] for oid in ids}
        rec["final_components"] = {r: final[r] for r in ("LowerRack", "UpperRack", "SilverwareBasket")}
        rec["still_finished"] = still(backend, rig, stills / f"finished__{track}__{name}.png",
                                      f"{iid}: finished, {track} track, {name}",
                                      f"{rec.get('moves_used', 0)} moves, abort {rec.get('abort')}")
        rec["end_check"] = end_check() if rec.get("abort") in (None, "give-up") else {"ran": False, "outcome": "aborted"}
        rec.update(track=track, tier=tier, instance=iid, algorithm=name, seed=seed, counter_cap=inst["counter"]["cap"],
                   lower_bound=inst["goal"]["lower_bound"], S_ref=inst["goal"]["S_ref"])
        end_ok = rec["end_check"].get("outcome") == "accepted"
        rec["success"] = bool(rec.get("solved") and end_ok) if track == "goal" else bool(end_ok)
        if track == "goal":
            rec["gap"] = rec.get("moves_used", 0) - inst["goal"]["lower_bound"] if rec["success"] else None
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(json.dumps(rec, indent=1, default=lambda o: o.tolist() if hasattr(o, "tolist") else str(o)) + "\n")
        report["episodes"].append({"track": track, "algorithm": name, "success": rec["success"], "abort": rec.get("abort"),
                                   "moves": rec.get("moves_used"), "end": rec["end_check"].get("outcome")})
    report["result"] = "PASS"


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--start", action="store_true")
    mode.add_argument("--run", action="store_true")
    mode.add_argument("--sequence", type=Path, metavar="ATTEMPT_DIR", help="build the goal one dish at a time, certify the order")
    mode.add_argument("--sequence-plan", type=Path, metavar="PLAN", help="build an open-track plan's own load one dish at a time (needs --instance)")
    parser.add_argument("--gate", type=Path, metavar="DIR", help="--sequence-plan: the load's joint gate (frigidaire_initial_state_validate.py --out-dir)")
    parser.add_argument("--tier", choices=("easy", "medium", "hard"))
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--attempt", type=int, default=0)
    parser.add_argument("--instance", type=Path)
    parser.add_argument("--algorithms", nargs="*", default=None)
    parser.add_argument("--baseline", type=Path, default=ROOT / "data/results/benchmark/frigidaire_hotec/baseline/result.json")
    parser.add_argument("--out", type=Path, default=ROOT / "data/results/benchmark/frigidaire_hotec")
    parser.add_argument("--usd", type=Path, default=ROOT / "data/build/frigidaire_collection/usd/fdpc4221as.usdc")
    parser.add_argument("--max-wall-seconds", type=float, default=7200.)
    parser.add_argument("--pile-coverage", type=float, default=PILE_COVERAGE, help="summed dish footprint / pile box area")
    from isaaclab.app import AppLauncher
    AppLauncher.add_app_launcher_args(parser)
    parser.set_defaults(device="cpu", headless=True)
    args = parser.parse_args()
    args.cameras = bool(getattr(args, "enable_cameras", False))     # AppLauncher consumes its own flags at boot
    started = time.monotonic()
    reroll = bool(args.start or args.sequence or args.sequence_plan)
    report = {"result": "REROLL" if reroll else "FAIL", "reason": None, "started_utc": datetime.now(timezone.utc).isoformat()}
    app = None
    if args.sequence:
        args.attempt_dir = args.sequence
    if args.sequence_plan and not args.instance:
        raise SystemExit("[RESULT] FAIL --sequence-plan needs --instance")
    if args.start:
        args.attempt_dir = args.out / "instances" / "attempts" / f"{args.tier}_s{args.seed}_a{args.attempt}"
        if (args.attempt_dir / "start.json").exists():
            raise SystemExit(f"[RESULT] FAIL refusing to overwrite {args.attempt_dir / 'start.json'}")
        args.attempt_dir.mkdir(parents=True, exist_ok=True)
    try:
        app = AppLauncher(args).app
        args.app = app
        sys.path[:0] = [str(ROOT / "code/src"), str(ROOT / "code/frigidaire/src"), str(ROOT / "code/planner/frigidaire")]
        import frigidaire_bench as B
        (start if args.start else sequence_gate if args.sequence else sequence_plan if args.sequence_plan else run)(args, B, report)
    except Exception as exc:                                   # noqa: BLE001
        traceback.print_exc()
        report.update(result="REROLL" if reroll else "FAIL", reason=f"exception {exc!r}")
    finally:
        report.update(wall_seconds=time.monotonic() - started, finished_utc=datetime.now(timezone.utc).isoformat())
        target = (args.attempt_dir / "generation.json") if args.start else (args.attempt_dir / "sequence_report.json") if args.sequence \
            else args.sequence_plan.with_name(args.sequence_plan.name[:-len(".json")] + ".sequence_report.json") if args.sequence_plan \
            else (args.out / "episodes" / "_kit" / f"{Path(args.instance).stem}.json")
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_text(json.dumps(report, indent=2, default=str) + "\n")
        print(f"[RESULT] {report['result']} {report.get('reason') or report.get('start', '') or report.get('instance', '')}", flush=True)
        if app is not None:
            from dishsim.media import release_sim_for_close
            release_sim_for_close()
            app.close()


if __name__ == "__main__":
    main()
