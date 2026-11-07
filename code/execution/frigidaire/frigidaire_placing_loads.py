#!/usr/bin/env python3
"""Part B goal loads (plans/2026-10-26-placing-goal.md): three single-kind subsets of certified sampler goals ->
data/results/robot/placing_v1/<load>/load.json; ``--summarize`` writes each load's summary.json from its dishes.jsonl
and subset_settle.json. Stdlib only (runs on the Mac).

    python3 code/execution/frigidaire/frigidaire_placing_loads.py [--summarize]

Goal pose = the goal record's ``targets_world`` (the sequentially built, settled poses; base frame, metres, Z-up,
XYZW). Order = the certify witness order restricted to the subset; edges = the graph's arrows within the subset.

``--sample <sample> --fixes`` (plans/2026-11-04-bowl-carry-clearance.md, -plate-pick-presentation.md, -deep-sites-gate.md, phase
2) writes the three per-kind test loads of results/2026-11-04-three-fixes.md: data/results/robot/samples_v1/<sample_id>__fixes/
{carry,plates,sites}/load.json (FIXES: the dishes, their moves, the plates' presentation, the build release of the sites from the
goal record's moves_world and witness steps), each with its media folder and the s0 bowls' seated-test supports.

``--sample <sample> --final`` (plans/2026-11-04-s0-rerun.md) writes data/results/robot/samples_v1/<sample_id>__final/load.json: every
dish of the sample in the dag order with its measured move (FINAL_MOVES; cups D-side-down, cup_05 by the build's release; bowls
D-ring, bowl_02 and bowl_03 by the build's release; plates I4-wall20 from the presentation.json written beside it out of the
phase 1b sweep's facing-0 row, FINAL_PLATE_ROW) and the three reject rows of FINAL_REJECTS (no Kit process for them).

``--sample <sample> --fifteen`` (plans/2026-11-04-fifteen-of-fifteen.md) writes data/results/robot/samples_v1/<sample_id>__fifteen/
load.json from the access order beside it (order.json of frigidaire_access_order.py): per dish its chain, hold, drive and move (the
build's sites D-ring-insert and D-side-insert), the plates' pinch spin at the goal, the start and the gate verdict; the order's rejects;
the chains in the order's executed order (with its reversed access edges, --dag soft). ``--order <dir>/order.json``: build from that
order instead and write load.json beside it (a scratch copy; the fifteen dir untouched). Third round: an order run --no-basket gives
the load "basket": false and every row the episode flag --no-basket (its "episode_flags"); a plate whose step passed by the
tine-guided drop gets the move I4-wall20-drop and "release" {mode, release_mm, guided_mm}; the "round1" block names, per dish no
Isaac run placed yet, the scene the box runner restores before it (round1_block).

``--sample <four-block sample>`` (plans/2026-11-04-grasp-place-s0.md) writes data/results/robot/samples_v1/<sample_id>/load.json
instead: every dish of the sample (all kinds) with its goal pose (goal.poses), slot (goal.slots), dag order and edges, the
rack frames of the empty-machine baseline (checked against goal.rack_joints), the pick orientation per kind and the move per
kind. ``--rows <samples_v1/<sample_id>>`` writes its rows.jsonl: one row per dish in the dag order from the Kit-free gate
(gate.json), the Isaac rows (dishes.jsonl) and the media folder.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
OUT = ROOT / "data/results/robot/placing_v1"
LOADS = {"easy_bowls": ("rest_easy", 5, "bowl"), "easy_plates": ("bp_medium", 5, "plate"), "easy_cups": ("rest_hard", 2, "cup"),
         "easy_bowls_v4": ("rest_easy_v4", 5, "bowl")}   # plans/2026-10-29-sampler-v4.md: the easy bowl preset re-sampled on v4
GRASP = {"bowl": "rim pinch", "plate": "idealized rim hold", "cup": "wall pinch"}
STEPS = ("R1", "R2", "S1", "S2", "S3", "S4", "S5", "S6")
SAMPLES = ROOT / "data/results/robot/samples_v1"
BASELINE = ROOT / "data/results/benchmark/frigidaire_hotec/baseline/result.json"
PICK_SPOT_XY = (.20, -.12)              # the study's counter spot (frigidaire_robot_episode.LIMITS_COUNTER_XY)
PICK_ORIENT = {"plate": "face up", "bowl": "mouth down", "cup": "mouth up"}
SAMPLE_MOVES = {"bowl": "D-ring", "cup": "D-side-mouth", "plate": "I4"}   # cups: the coordinator's option B (2026-11-04)
CUPS_DOWN_GRASP = ("side pinch 60 mm under the rim (45 mm from the base), the cup mouth down, the wrist toward the base (above "
                   "the cup); plans/2026-11-04-cups-mouth-down.md")
SAMPLE_GRASP = {"bowl": "foot ring, the pads on the ring's outer face (F-out)",
                "cup": "side pinch 45 mm under the rim, the wrist toward the mouth (placing_v1/partA_cup_side_h45, held 10/10)",
                "plate": "rim pinch as the grasp gate measured it (limits_v1/gate2_plate, the dish clearance waived)"}


def build(name, tier, seed, kind):
    pair_rel = f"data/results/sampler/{tier}/frigidaire_fdpc4221as__hotec_wheatstraw__{seed}.json"
    pair = json.loads((ROOT / pair_rel).read_text())
    rec = json.loads((ROOT / pair["goal"]["record"]).read_text())
    order = [d for d in pair["goal"]["certify"]["order"] if d.startswith(kind + "_")]
    assert order == [d for d in rec["witness"]["order"] if d.startswith(kind + "_")], "witness orders disagree"
    meta = {e["id"]: e for e in rec["entries_settled"]}
    dishes = []
    for k, d in enumerate(order):
        p = rec["targets_world"][d]
        dishes.append({"name": d, "kind": kind, "grasp": GRASP[kind], "rack": meta[d]["rack"], "slot": meta[d]["slot"],
                       "variant": meta[d]["variant"], "order_index": k,
                       "goal_pose": {"pos": p["position_m"], "quat_xyzw": p["quaternion_xyzw"]}})
    sub, edges = set(order), []
    for e in rec["graph"]["edges"]:
        if e["a"] in sub and e["b"] in sub and [e["a"], e["b"]] not in edges:
            edges.append([e["a"], e["b"]])
    load = {"load": name, "source_pair": pair_rel, "goal_record": pair["goal"]["record"], "kind": kind,
            "pose_convention": "base frame (the twin's world), metres, Z-up, quaternion XYZW; targets_world of the goal record",
            "frames": rec["frames"], "dishes": dishes, "order": order, "edges": edges}
    (OUT / name).mkdir(parents=True, exist_ok=True)
    (OUT / name / "load.json").write_text(json.dumps(load, indent=1) + "\n")
    print(f"{name}: {len(dishes)} {kind}s, order {order}, edges {edges}, racks {sorted({d['rack'] for d in dishes})}")


def summarize(name):
    d = OUT / name
    rows = {}
    if (d / "dishes.jsonl").exists():
        for line in (d / "dishes.jsonl").read_text().splitlines():
            if line.strip():
                r = json.loads(line)
                rows[(r["dish"], r["variant"])] = r              # a rerun's row replaces the earlier one
    load = json.loads((d / "load.json").read_text())
    counts = lambda v: {s: sum(r["failed_at"] == s for (_, vv), r in rows.items() if vv == v) for s in STEPS
                        if any(r["failed_at"] == s for (_, vv), r in rows.items() if vv == v)}
    ss = json.loads((d / "subset_settle.json").read_text()) if (d / "subset_settle.json").exists() else None
    out = {"load": name, "n_dishes": len(load["dishes"]),
           "placed_W": sum(r["failed_at"] is None for (_, v), r in rows.items() if v == "W"),
           "placed_R": sum(r["failed_at"] is None for (_, v), r in rows.items() if v == "R"),
           "placed_R_ideal": sum(r["failed_at"] is None for (_, v), r in rows.items() if v == "R-ideal"),
           "placed_S": sum(r["failed_at"] is None for (_, v), r in rows.items() if v == "S"),
           "rows_W": sum(v == "W" for _, v in rows), "rows_R": sum(v == "R" for _, v in rows),
           "rows_R_ideal": sum(v == "R-ideal" for _, v in rows), "rows_S": sum(v == "S" for _, v in rows),
           "failed_at_counts": counts("W"), "failed_at_counts_R": counts("R"), "failed_at_counts_R_ideal": counts("R-ideal"),
           "failed_at_counts_S": counts("S"),
           "subset_settle": None if ss is None else {k: ss[k] for k in ("ok", "max_drift_mm", "max_drift_deg")}}
    (d / "summary.json").write_text(json.dumps(out, indent=1) + "\n")
    print(json.dumps(out))


def build_sample(path, cups_down=False):
    """plans/2026-11-04-grasp-place-s0.md: the four-block sample -> samples_v1/<sample_id>/load.json (every dish, all kinds).
    ``cups_down`` (plans/2026-11-04-cups-mouth-down.md): the cups alone in the dag order, picked mouth down, move D-side-down,
    -> samples_v1/<sample_id>__cups_down/load.json."""
    path = path if path.is_absolute() else ROOT / path
    smp = json.loads(path.read_text())
    sid, goal, dag = smp["metadata"]["sample_id"], smp["goal"], smp["dag"]
    moves, orient, grasp = dict(SAMPLE_MOVES), dict(PICK_ORIENT), dict(SAMPLE_GRASP)
    if cups_down:
        moves["cup"], orient["cup"], grasp["cup"] = "D-side-down", "mouth down", CUPS_DOWN_GRASP
    base = json.loads(BASELINE.read_text())
    snap = base.get("snapshot", base)
    off = {j: abs(float(snap["joints"][j]) - float(v)) for j, v in goal["rack_joints"].items()}
    if max(off.values()) > 1e-6:
        raise SystemExit(f"[RESULT] FAIL {sid}: the baseline's rack joints differ from goal.rack_joints {off}")
    kind = lambda n: n.split("_")[0]
    order = [d for d in dag["order"] if not cups_down or kind(d) == "cup"]
    dishes = []
    for k, d in enumerate(order):
        s_, p = goal["slots"][d], goal["poses"][d]
        dishes.append({"name": d, "kind": kind(d), "grasp": grasp[kind(d)], "rack": s_["rack"], "slot": s_["slot"],
                       "variant": s_["variant"], "order_index": k, "move": moves[kind(d)],
                       "pick": {"spot_xy": list(PICK_SPOT_XY), "orientation": orient[kind(d)]},
                       "goal_pose": {"pos": p["position_m"], "quat_xyzw": p["quaternion_xyzw"]}})
    name = f"{sid}__cups_down" if cups_down else sid
    load = {"load": name, "source_sample": str(path.relative_to(ROOT)), "kind": "cup" if cups_down else "mixed",
            "kinds": sorted({d["kind"] for d in dishes}), "moves": moves, "pick_orientation": orient,
            "pick_spot_xy": list(PICK_SPOT_XY),
            "pose_convention": "base frame (the twin's world), metres, Z-up, quaternion XYZW; goal.poses of the sample",
            "frames": {r: snap["poses"][r] for r in ("LowerRack", "UpperRack", "SilverwareBasket")},
            "frames_source": f"{BASELINE.relative_to(ROOT)} (both racks extended, joints = goal.rack_joints)",
            "rack_joints": goal["rack_joints"], "dishes": dishes, "order": order,
            "edges": [[e["a"], e["b"]] for e in dag["edges"] if e["a"] in order and e["b"] in order],
            "edge_kinds": [e.get("kind") for e in dag["edges"] if e["a"] in order and e["b"] in order]}
    if cups_down:
        load["subset"] = "the sample's cups alone (plans/2026-11-04-cups-mouth-down.md); the other dishes are not in the scene"
    (SAMPLES / name).mkdir(parents=True, exist_ok=True)
    (SAMPLES / name / "load.json").write_text(json.dumps(load, indent=1) + "\n")
    print(f"[RESULT] PASS sample load {name}: {len(dishes)} dishes {dict((k_, sum(d['kind'] == k_ for d in dishes)) for k_ in load['kinds'])}, "
          f"order {load['order']}, edges {load['edges']} -> {(SAMPLES / name / 'load.json').relative_to(ROOT)}")


FIXES = {"carry": {"dishes": ("bowl_05", "bowl_01", "cup_03"), "moves": {"bowl": "D-ring", "cup": "D-side-down"},
                   "plan": "plans/2026-11-04-bowl-carry-clearance.md"},
         "plates": {"dishes": ("plate_05", "plate_03", "plate_04"), "moves": {"plate": "I4-lean30"},
                    "plan": "plans/2026-11-04-plate-pick-presentation.md"},
         "sites": {"dishes": ("bowl_02", "bowl_03", "cup_05"), "moves": {"bowl": "D-ring-build", "cup": "D-side-down-build"},
                   "plan": "plans/2026-11-04-deep-sites-gate.md"}}
FIXES_UNREACHABLE = {"bowl_04": "the gripper and wrists 2 and 3 against the Cabinet at every option (phase 1, sites/reach.json)",
                     "plate_01": "wrists 1 and 2 and the upper arm against the Cabinet and the basket at every option",
                     "plate_02": "wrists 1 to 3 and the forearm against the pedestal at every option"}
FIX_PICK = {"bowl": {"orientation": "mouth down"}, "cup": {"orientation": "mouth down"},
            "plate": {"orientation": "leaning 30 deg on a backstop (the edge pinch)", "presentation": "presentation.json"}}
FIX_GRASP = {"D-ring": SAMPLE_GRASP["bowl"], "D-ring-build": SAMPLE_GRASP["bowl"], "D-side-down": CUPS_DOWN_GRASP.replace(
    "60 mm under the rim (45 mm from the base)", "11 mm from the base (93.9 mm under the rim)"),
             "D-side-down-build": CUPS_DOWN_GRASP.replace("60 mm under the rim (45 mm from the base)",
                                                          "11 mm from the base (93.9 mm under the rim)"),
             "I4-lean30": "the top-down edge pinch at the top edge (TCP 3 mm outside the rim, pads on the mouth and base faces)"}


def build_fixes(path):
    """--sample <sample> --fixes: the three per-kind loads of the three-fixes window (FIXES) -> samples_v1/<sample_id>__fixes/
    <chain>/load.json; the dishes in the dag order, goal poses = goal.poses, the frames of the baseline; the sites' build
    release = the goal record's moves_world pose and witness step of the dish."""
    import shutil
    path = path if path.is_absolute() else ROOT / path
    smp = json.loads(path.read_text())
    sid, goal, dag = smp["metadata"]["sample_id"], smp["goal"], smp["dag"]
    rec_path = smp["metadata"]["provenance"]["goal_record"]
    rec = json.loads((ROOT / rec_path).read_text())
    steps = {s["id"]: s for s in rec["witness"]["steps"]}
    base = json.loads(BASELINE.read_text())
    snap = base.get("snapshot", base)
    kind = lambda n: n.split("_")[0]
    for chain, spec in FIXES.items():
        order = [n for n in dag["order"] if n in spec["dishes"]]
        dishes = []
        for k, n in enumerate(order):
            s_, p = goal["slots"][n], goal["poses"][n]
            mv = spec["moves"][kind(n)]
            row = {"name": n, "kind": kind(n), "grasp": FIX_GRASP[mv], "rack": s_["rack"], "slot": s_["slot"],
                   "variant": s_["variant"], "order_index": k, "move": mv,
                   "pick": {"spot_xy": list(PICK_SPOT_XY), **FIX_PICK[kind(n)]},
                   "goal_pose": {"pos": p["position_m"], "quat_xyzw": p["quaternion_xyzw"]}}
            if mv.endswith("-build"):
                st = steps[n]
                row["build_release"] = {"move_pose": rec["moves_world"][n], "record": rec_path,
                                        **{k_: st.get(k_) for k_ in ("start_lift_mm", "lowest_lift_mm", "release_lift_mm", "stop",
                                                                     "hover_mm", "lateral_mm", "dz_mm", "tilt_deg", "settle_dev_mm")},
                                        "rule": rec["release"].get("release")}
            dishes.append(row)
        name = f"{sid}__fixes_{chain}"
        d_out = SAMPLES / f"{sid}__fixes" / chain
        load = {"load": name, "chain": chain, "plan": spec["plan"], "source_sample": str(path.relative_to(ROOT)),
                "kind": "mixed" if len({d["kind"] for d in dishes}) > 1 else dishes[0]["kind"],
                "kinds": sorted({d["kind"] for d in dishes}), "moves": dict(spec["moves"]),
                "pick_spot_xy": list(PICK_SPOT_XY),
                "pose_convention": "base frame (the twin's world), metres, Z-up, quaternion XYZW; goal.poses of the sample",
                "frames": {r: snap["poses"][r] for r in ("LowerRack", "UpperRack", "SilverwareBasket")},
                "frames_source": f"{BASELINE.relative_to(ROOT)} (both racks extended, joints = goal.rack_joints)",
                "rack_joints": goal["rack_joints"], "dishes": dishes, "order": order,
                "edges": [[e["a"], e["b"]] for e in dag["edges"] if e["a"] in order and e["b"] in order],
                "subset": f"the sample's {', '.join(order)} alone in the dag order; the other dishes are not in the scene",
                "media_dir": f"data/media/robot/samples_v1/{sid}__fixes/{chain}",
                "unreachable_on_this_mount": FIXES_UNREACHABLE}
        d_out.mkdir(parents=True, exist_ok=True)
        (d_out / "load.json").write_text(json.dumps(load, indent=1) + "\n")
        if any(d["kind"] == "bowl" for d in dishes):  # the s0 bowls' goal supports (the same goals: kitfree_v1/insert_bowl)
            shutil.copyfile(SAMPLES / sid / "bowl_supports.json", d_out / "bowl_supports.json")
        print(f"[RESULT] PASS fixes load {name}: {[(d['name'], d['move']) for d in dishes]} -> {(d_out / 'load.json').relative_to(ROOT)}")


FINAL_MOVES = {"cup": "D-side-down", "bowl": "D-ring", "plate": "I4-wall20",   # plans/2026-11-04-s0-rerun.md: per kind the
               "cup_05": "D-side-down-build", "bowl_02": "D-ring-build", "bowl_03": "D-ring-build"}   # move that measured best
FINAL_REJECTS = {   # the three sites no option reaches (plans/2026-11-04-deep-sites-gate.md phase 1, <sample_id>__fixes/sites/kitfree.md)
    "bowl_04": ["at every descent start the gripper, wrist 3 and wrist 2 against the Cabinet; no free side beside the site (the "
                "held bowl against the UpperRack)", ["gripper|Cabinet", "wrist_3_link|Cabinet", "wrist_2_link|Cabinet", "held|UpperRack"]],
    "plate_01": ["at every descent start the upper arm, wrist 1 and wrist 2 against the Cabinet; no free side beside the site (the "
                 "held plate against the tines)", ["upper_arm_link|Cabinet", "wrist_1_link|Cabinet", "wrist_2_link|Cabinet",
                                                    "held|tine row5 col5"]],
    "plate_02": ["at every descent start the held plate against the forearm, wrist 2 against the basket and the pedestal",
                 ["held|forearm_link", "wrist_2_link|SilverwareBasket", "wrist_2_link|Pedestal"]]}
FINAL_LOWER_ORDER = ("bowl_02", "plate_03", "plate_05", "plate_04")   # the lower chain as run: the dag order with plate_03 moved
                                          # before plate_05 (the coordinator, 2026-11-04): plate_05 at its goal blocks plate_03's
                                          # descent (wrist 1, the forearm, the hand), plate_03 at its goal leaves plate_05 a 115 mm
                                          # start (data/logs/robot_final/kitfree/plate_order.log); plate_03 has no dag edges
FINAL_PLATE_SWEEP = "data/results/robot/samples_v1/{sid}__fixes/plates_wall2/goal_lean.json"   # phase 1b of
FINAL_PLATE_ROW = {"facing_deg": 0, "cand": "180/+0"}   # plans/2026-11-04-plate-wall-pinch.md: the coordinator's row (the plate
                                          # turns 0.3 to 2.4 deg from the pick to the goals; I4 G2 at all three, leg 1 straight)
FINAL_GRASP = {"D-ring": SAMPLE_GRASP["bowl"], "D-ring-build": SAMPLE_GRASP["bowl"], "D-side-down": FIX_GRASP["D-side-down"],
               "D-side-down-build": FIX_GRASP["D-side-down-build"],
               "I4-wall20": "the wall-normal pinch 20 mm down the plate's wall (grasp.plate_wall_grasps), roll 180, spin 0"}


def final_presentation(sid):
    """presentation.json of the final load from the phase 1b sweep's FINAL_PLATE_ROW: the plate's spawn pose, the backstop as
    the sweep placed it (its axes: a turn about z, so the USD quaternion is that turn), the wall hold and the chosen pick."""
    import math
    src = ROOT / FINAL_PLATE_SWEEP.format(sid=sid)
    gl = json.loads(src.read_text())
    row = next(r for r in gl["rows"] if r["facing_deg"] == FINAL_PLATE_ROW["facing_deg"])
    c = row["cands"][FINAL_PLATE_ROW["cand"]]
    if not c["ok"]:
        raise SystemExit(f"[RESULT] FAIL {src}: the row {FINAL_PLATE_ROW} has no passing pick")
    bx = row["fixture"]["boxes"]["Backstop"]
    ax = bx["axes"]
    th = math.atan2(ax[1][0], ax[0][0])                 # the box's local x (its 20 mm depth) in the world: a turn about z
    i4 = {n: {k: (gl["i4"].get(f"{row['facing_deg']}/{FINAL_PLATE_ROW['cand']}/{n}") or {}).get(k)
              for k in ("pass", "gate", "start_mm", "joint_move_deg", "why")} for n in ("plate_05", "plate_03", "plate_04")}
    roll, spin = int(c["roll"]), float(c["spin_deg"])
    return {"name": f"goal-lean facing {row['facing_deg']}", "facing_deg": row["facing_deg"], "yaw_k": None,
            "lean_deg": round(gl["goal_orientation"]["axis_below_horizontal_deg"], 2), "plate_pose": row["spawn_pose"],
            "backstop": {"center_m": bx["center_m"], "size_m": bx["size_m"], "axes": ax,
                         "quaternion_xyzw": [0., 0., round(math.sin(th / 2), 8), round(math.cos(th / 2), 8)]},
            "edge": None,
            "wall": {"depth_mm": gl["hold"]["depth_mm"], "T_hold": gl["hold"]["record"]["T_tcp_in_plate"]["0"],
                     "spins_deg": [spin], "rolls": [roll, roll], "pad": gl["hold"]["pad"],
                     "rule": "grasp.plate_wall_grasps(T_obj, T_hold, spins_deg, rolls): the hold turned to the rim point highest at "
                             "the plate's pose plus the spin, the roll; the candidate twice, so a run's re-grasp (PICK_TRIES) "
                             "retries the same pinch"},
            "fixture": row["fixture"],
            "pick_kitfree": {k: c.get(k) for k in ("T_tcp", "M", "q_hover", "q_grasp", "q_lift", "T_lift", "facing", "move_rad",
                                                   "moment_Nm", "approach_last60", "turn_to_goal_deg")},
            "i4_kitfree": i4, "source": f"{src.relative_to(ROOT)} rows[facing {row['facing_deg']}].cands[{FINAL_PLATE_ROW['cand']}] "
            "(the coordinator's choice, 2026-11-04: the top-level chosen was facing 330)",
            "sweep_sha256_16": gl.get("imported_sha256_16")}


def build_final(path):
    """--sample <sample> --final: samples_v1/<sample_id>__final/{load.json, presentation.json}, plans/2026-11-04-s0-rerun.md."""
    path = path if path.is_absolute() else ROOT / path
    smp = json.loads(path.read_text())
    sid, goal, dag = smp["metadata"]["sample_id"], smp["goal"], smp["dag"]
    rec_path = smp["metadata"]["provenance"]["goal_record"]
    rec = json.loads((ROOT / rec_path).read_text())
    steps = {s_["id"]: s_ for s_ in rec["witness"]["steps"]}
    base = json.loads(BASELINE.read_text())
    snap = base.get("snapshot", base)
    off = {j: abs(float(snap["joints"][j]) - float(v)) for j, v in goal["rack_joints"].items()}
    if max(off.values()) > 1e-6:
        raise SystemExit(f"[RESULT] FAIL {sid}: the baseline's rack joints differ from goal.rack_joints {off}")
    kind = lambda n: n.split("_")[0]
    name = f"{sid}__final"
    d_out = SAMPLES / name
    d_out.mkdir(parents=True, exist_ok=True)
    pres = final_presentation(sid)
    (d_out / "presentation.json").write_text(json.dumps(pres, indent=1) + "\n")
    dishes = []
    for k, n in enumerate(dag["order"]):
        s_, p = goal["slots"][n], goal["poses"][n]
        mv = FINAL_MOVES.get(n, FINAL_MOVES[kind(n)])
        row = {"name": n, "kind": kind(n), "grasp": FINAL_GRASP[mv], "rack": s_["rack"], "slot": s_["slot"],
               "variant": s_["variant"], "order_index": k, "move": mv,
               "pick": {"spot_xy": list(PICK_SPOT_XY),
                        "orientation": "mouth down" if kind(n) in ("bowl", "cup") else
                        f"standing in its goal orientation against the backstop ({pres['name']})",
                        **({"presentation": "presentation.json"} if kind(n) == "plate" else {})},
               "goal_pose": {"pos": p["position_m"], "quat_xyzw": p["quaternion_xyzw"]}}
        if mv.endswith("-build"):
            st = steps[n]
            row["build_release"] = {"move_pose": rec["moves_world"][n], "record": rec_path,
                                    **{k_: st.get(k_) for k_ in ("start_lift_mm", "lowest_lift_mm", "release_lift_mm", "stop",
                                                                 "hover_mm", "lateral_mm", "dz_mm", "tilt_deg", "settle_dev_mm")},
                                    "rule": rec["release"].get("release")}
        if n in FINAL_REJECTS:
            row["reject"] = {"cause": "unreachable on this mount", "detail": FINAL_REJECTS[n][0],
                             "blockers": FINAL_REJECTS[n][1],
                             "source": f"data/results/robot/samples_v1/{sid}__fixes/sites/kitfree.md (phase 1 of "
                                       "plans/2026-11-04-deep-sites-gate.md)"}
        dishes.append(row)
    chains = {"upper": [d["name"] for d in dishes if d["rack"] == "UpperRack" and "reject" not in d],
              "lower": list(FINAL_LOWER_ORDER)}
    load = {"load": name, "plan": "plans/2026-11-04-s0-rerun.md", "source_sample": str(path.relative_to(ROOT)), "kind": "mixed",
            "kinds": sorted({d["kind"] for d in dishes}), "moves": {"cup": "D-side-down", "bowl": "D-ring", "plate": "I4-wall20"},
            "move_overrides": {n: m for n, m in FINAL_MOVES.items() if "_" in n}, "pick_spot_xy": list(PICK_SPOT_XY),
            "pose_convention": "base frame (the twin's world), metres, Z-up, quaternion XYZW; goal.poses of the sample",
            "frames": {r: snap["poses"][r] for r in ("LowerRack", "UpperRack", "SilverwareBasket")},
            "frames_source": f"{BASELINE.relative_to(ROOT)} (both racks extended, joints = goal.rack_joints)",
            "rack_joints": goal["rack_joints"], "dishes": dishes, "order": list(dag["order"]),
            "edges": [[e["a"], e["b"]] for e in dag["edges"]], "edge_kinds": [e.get("kind") for e in dag["edges"]],
            "chains": chains, "executed_order": chains["upper"] + chains["lower"],
            "executed_order_rule": "each rack's chain in the dag order, the lower chain with plate_03 moved before plate_05 (a valid "
                                   "linearization of the dag: plate_03 has no edges) for the arm's access; dag.order's own "
                                   "sequence makes plate_03 a predicted no-reach (data/logs/robot_final/kitfree/plate_order.log)",
            "rejects": sorted(FINAL_REJECTS), "media_dir": f"data/media/robot/samples_v1/{name}",
            "presentation": "presentation.json"}
    (d_out / "load.json").write_text(json.dumps(load, indent=1) + "\n")
    import shutil                                       # the s0 bowls' seated-test supports (the same goals)
    shutil.copyfile(SAMPLES / sid / "bowl_supports.json", d_out / "bowl_supports.json")
    print(f"[RESULT] PASS final load {name}: {[(d['name'], d['move'], 'reject' if 'reject' in d else 'run') for d in dishes]}; "
          f"chains {chains} -> {(d_out / 'load.json').relative_to(ROOT)}")


RING_SIDE_ISAAC = ("bowl_04", "bowl_03", "bowl_02")   # plans/2026-11-04-ring-side-hold.md: the Isaac order (each bowl alone)
GRIP_DRIVE_BEFORE = {"gripper_effort": 8., "gripper_stiffness": 400., "gripper_damping": 1., "finger_effort": 10.,
                     "finger_stiffness": 10., "finger_damping": .05}   # flags R2 and the fixed robot_cfg gains before the
                                          # grip-force round (plans/2026-11-04-grip-force.md): a row's default drive
RING_SIDE_GRASP = ("side ring hold: the approach horizontal at the ring's height, the pads on the ring's outer face at two "
                   "opposite points, the pad's lower edge 1 mm above the ring's base plane, the fingertips at the bowl's axis "
                   "(grasp.foot_side_grasps); the bowl mouth down")


def build_ring_side(path):
    """--sample <sample> --ring-side (plans/2026-11-04-ring-side-hold.md): samples_v1/<sample_id>__ring_side/load.json, the sample's
    five bowls in the dag order with move D-ring-side, each run alone (no state chain: the bowl spawned at the counter spot, no other
    dish, its rack out and the other pushed in, the racks at the goal frames); isaac_order RING_SIDE_ISAAC, the others Kit-free only."""
    import shutil
    path = path if path.is_absolute() else ROOT / path
    smp = json.loads(path.read_text())
    sid, goal, dag = smp["metadata"]["sample_id"], smp["goal"], smp["dag"]
    base = json.loads(BASELINE.read_text())
    snap = base.get("snapshot", base)
    off = {j: abs(float(snap["joints"][j]) - float(v)) for j, v in goal["rack_joints"].items()}
    if max(off.values()) > 1e-6:
        raise SystemExit(f"[RESULT] FAIL {sid}: the baseline's rack joints differ from goal.rack_joints {off}")
    order = [n for n in dag["order"] if n.startswith("bowl_")]
    name = f"{sid}__ring_side"
    d_out = SAMPLES / name
    d_out.mkdir(parents=True, exist_ok=True)
    dishes = []
    for k, n in enumerate(order):
        s_, p = goal["slots"][n], goal["poses"][n]
        dishes.append({"name": n, "kind": "bowl", "grasp": RING_SIDE_GRASP, "rack": s_["rack"], "slot": s_["slot"],
                       "variant": s_["variant"], "order_index": k, "move": "D-ring-side",
                       "pick": {"spot_xy": list(PICK_SPOT_XY), "orientation": "mouth down"},
                       "goal_pose": {"pos": p["position_m"], "quat_xyzw": p["quaternion_xyzw"]}})
    load = {"load": name, "plan": "plans/2026-11-04-ring-side-hold.md", "source_sample": str(path.relative_to(ROOT)), "kind": "bowl",
            "kinds": ["bowl"], "moves": {"bowl": "D-ring-side"}, "pick_spot_xy": list(PICK_SPOT_XY),
            "pose_convention": "base frame (the twin's world), metres, Z-up, quaternion XYZW; goal.poses of the sample",
            "frames": {r: snap["poses"][r] for r in ("LowerRack", "UpperRack", "SilverwareBasket")},
            "frames_source": f"{BASELINE.relative_to(ROOT)} (both racks extended, joints = goal.rack_joints)",
            "rack_joints": goal["rack_joints"], "dishes": dishes, "order": order,
            "edges": [[e["a"], e["b"]] for e in dag["edges"] if e["a"] in order and e["b"] in order],
            "isaac_order": list(RING_SIDE_ISAAC), "kitfree_only": [n for n in order if n not in RING_SIDE_ISAAC],
            "subset": "each bowl alone (no other dish in the scene, no state chain), its rack out and the other pushed in",
            "media_dir": f"data/media/robot/samples_v1/{name}"}
    (d_out / "load.json").write_text(json.dumps(load, indent=1) + "\n")
    shutil.copyfile(SAMPLES / sid / "bowl_supports.json", d_out / "bowl_supports.json")   # the s0 bowls' seated-test supports
    print(f"[RESULT] PASS ring-side load {name}: {[(d['name'], d['slot']) for d in dishes]}; Isaac {list(RING_SIDE_ISAAC)} -> "
          f"{(d_out / 'load.json').relative_to(ROOT)}")


FIFTEEN_MOVE = {"D-ring-build": "D-ring-insert", "D-side-down-build": "D-side-insert"}   # plans/2026-11-04-fifteen-of-fifteen.md:
                                          # the build's sites by guarded insertion along the build's move (the gate's -> the episode's)


def fifteen_move(gate_move, build_site, release=None):
    """The episode's move of a gate move: a build site's -build move its guarded insertion, the side ring hold there
    D-ring-side-insert (the gate checks it as D-ring-side with the build release); the wall pinch released by the tine-guided
    drop (``release``: the step's release block, mode "drop"; plans/2026-11-04-fifteen-of-fifteen.md, the user 20:05 EST)
    I4-wall20-drop, a seat release I4-wall20."""
    if gate_move == "I4-wall20" and (release or {}).get("mode") == "drop":
        return DROP_MOVE
    return "D-ring-side-insert" if build_site and gate_move == "D-ring-side" else FIFTEEN_MOVE.get(gate_move, gate_move)


DROP_MOVE = "I4-wall20-drop"              # the wall pinch's tine-guided release (the episode's move): lowered along the tine axis
                                          # to the lowest height with the arm 2 mm clear, the rim already 20 mm or more below the
                                          # slot's tine tips there, the jaws open, the rise along the tine axis (no back-off)
NO_BASKET_FLAG = "--no-basket"            # the episode's flag of a load whose order ran without the SilverwareBasket
ROUND1_RULE = ("per dish no Isaac run placed yet (no state file of the order's state dirs has it placed), the scene the full run "
               "gives it, to restore before its round-1 process: its chain predecessors in the executed order. state = the "
               "state file (<s0>__final/state/after_<dag index>_<dish>.json) of the nearest predecessor that run placed (ok, the "
               "dish among placed: the scene as the robot left it); none placed there: that rack's first __final state with "
               "nothing placed (the chain's empty start, its rack joints); compose = the predecessors the state lacks, at their "
               "predicted poses (the order's poses: the dish's own landed pose where a run placed it, the ring-side round's "
               "bowl_04 among them, else its goal pose), as world poses at the state's rack frame of their rack; remove = the "
               "state's placed dishes that are not predecessors (parked). The composed --state file: the state's placed plus "
               "compose minus remove, its rack_joints, its failed list without the composed dishes")
BOWL_CUP_DRIVE = {"gripper_effort": 8., "gripper_stiffness": 4., "gripper_damping": 2.5}   # results/2026-11-04-grip-force.md gf_07
PLATE_DRIVE = {"gripper_effort": 8., "gripper_stiffness": 400., "gripper_damping": 1.}     # (22.5 N per pad); the plates' wall pinch
FIFTEEN_DRIVE = {"foot-out": BOWL_CUP_DRIVE, "foot-side": BOWL_CUP_DRIVE, "side-down": BOWL_CUP_DRIVE, "plate-wall": PLATE_DRIVE}
FIFTEEN_GRASP = {"foot-out": SAMPLE_GRASP["bowl"], "foot-side": RING_SIDE_GRASP, "side-down": FIX_GRASP["D-side-down"],
                 "plate-wall": "the wall-normal pinch 20 mm down the plate's wall (grasp.plate_wall_grasps), roll 180, spin 0 on the "
                               "plank; at the goal the pinch's rim point pinch_spin_deg from the rim point highest (the access order)"}


def build_releases(smp, names):
    """{dish: build_release} for ``names`` from the sample's goal record (its moves_world pose and witness step), the block build_final
    writes for a -build site (plans/2026-11-04-deep-sites-gate.md, amended). No names, or no goal record (a hardware-aware
    candidate, plans/2026-11-05-hardware-aware-sampler.md: goal_record null): {}."""
    rec_path = (smp["metadata"].get("provenance") or {}).get("goal_record")
    if not names or not rec_path:
        return {}
    rec = json.loads((ROOT / rec_path).read_text())
    steps = {s_["id"]: s_ for s_ in rec["witness"]["steps"]}
    return {n: {"move_pose": rec["moves_world"][n], "record": rec_path,
                **{k_: steps[n].get(k_) for k_ in ("start_lift_mm", "lowest_lift_mm", "release_lift_mm", "stop", "hover_mm",
                                                   "lateral_mm", "dz_mm", "tilt_deg", "settle_dev_mm")},
                "rule": rec["release"].get("release")} for n in names}


def _rel(p):
    """``p`` relative to the repo root when it lies under it, else its absolute path (a scratch copy)."""
    try:
        return str(Path(p).resolve().relative_to(ROOT.resolve()))
    except ValueError:
        return str(Path(p).resolve())


def _qn(q):
    s = sum(float(v) * float(v) for v in q) ** .5
    return [float(v) / s for v in q]


def _qmul(a, b):
    """XYZW quaternion product a b."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return [aw * bx + ax * bw + ay * bz - az * by, aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw, aw * bw - ax * bx - ay * by - az * bz]


def _qrot(q, v):
    """The vector v turned by the unit XYZW quaternion q."""
    x, y, z, w = q
    t = [2. * (y * v[2] - z * v[1]), 2. * (z * v[0] - x * v[2]), 2. * (x * v[1] - y * v[0])]
    return [v[0] + w * t[0] + y * t[2] - z * t[1], v[1] + w * t[1] + z * t[0] - x * t[2], v[2] + w * t[2] + x * t[1] - y * t[0]]


def _in_frame(fr, p):
    """The pose p (world {position_m, quaternion_xyzw}) in the frame fr (world): fr^-1 p."""
    qf = _qn(fr["quaternion_xyzw"])
    qc = [-qf[0], -qf[1], -qf[2], qf[3]]
    d = [float(a) - float(b) for a, b in zip(p["position_m"], fr["position_m"])]
    return {"position_m": _qrot(qc, d), "quaternion_xyzw": _qn(_qmul(qc, _qn(p["quaternion_xyzw"])))}


def _at_frame(fr, r):
    """The pose r (in the frame fr) in the world: fr r."""
    qf = _qn(fr["quaternion_xyzw"])
    return {"position_m": [float(a) + b for a, b in zip(fr["position_m"], _qrot(qf, r["position_m"]))],
            "quaternion_xyzw": _qn(_qmul(qf, r["quaternion_xyzw"]))}


def round1_block(sid, od, smp, snap, chains):
    """The load's "round1" block (ROUND1_RULE): per dish of the executed order that no Isaac run placed yet, the state to restore
    (the nearest predecessor's __final state, else the rack's empty start), the predecessors to compose in at their predicted
    poses (world poses at that state's rack frame) and the state's dishes to remove."""
    ab = lambda p: Path(p) if Path(p).is_absolute() else ROOT / p
    placed_in = {}                                     # dish -> the first state file (the order's dirs in order) that has it placed
    for sd in [ab(s) for s in od.get("states") or []]:
        for f in sorted(sd.glob("after_*_*.json")):
            st = json.loads(f.read_text())
            if st.get("after") in (st.get("placed") or {}) and st["after"] not in placed_in:
                placed_in[st["after"]] = _rel(f)
    fin_dir = SAMPLES / f"{sid}__final" / "state"
    fin, empty = {}, {}                                # __final: dish -> (file, state) placed there; rack -> its first empty state
    for f in sorted(fin_dir.glob("after_*_*.json")):
        st = json.loads(f.read_text())
        n = st.get("after")
        if st.get("ok") and n in (st.get("placed") or {}):
            fin[n] = (f, st)
        elif not st.get("placed") and n in smp["goal"]["slots"]:
            empty.setdefault(smp["goal"]["slots"][n]["rack"], (f, st))
    out = {}
    for c, seq in chains.items():
        for i, n in enumerate(seq):
            if n in placed_in:
                continue
            preds = seq[:i]
            rack = smp["goal"]["slots"][n]["rack"]
            base = next((fin[p] for p in reversed(preds) if p in fin), None) or empty.get(rack)
            b_placed = sorted((base[1].get("placed") or {})) if base else []
            comp = {}
            for p in [p for p in preds if p not in b_placed]:
                src, rk = od["poses"][p]["source"], od["poses"][p]["rack"]
                if src.startswith("goal"):
                    rel_ = _in_frame(snap["poses"][rk], smp["goal"]["poses"][p])
                else:
                    st_ = json.loads(ab(src).read_text())
                    rel_ = _in_frame(st_["rack_frames"][rk], st_["placed"][p])
                w_ = _at_frame(base[1]["rack_frames"][rk], rel_) if base else None
                comp[p] = {"rack": rk, "pose_source": src, "pose_in_rack": rel_,
                           **({"position_m": w_["position_m"], "quaternion_xyzw": w_["quaternion_xyzw"]} if w_ else {})}
            out[n] = {"chain": c, "chain_index": i, "predecessor": preds[-1] if preds else None,
                      "predecessor_placed_in_isaac": placed_in.get(preds[-1]) if preds else None, "scene": preds,
                      "state": _rel(base[0]) if base else None, "state_after": base[1].get("after") if base else None,
                      "state_ok": base[1].get("ok") if base else None, "state_placed": b_placed, "compose": comp,
                      "remove": [m for m in b_placed if m not in preds]}
    return {"rule": ROUND1_RULE, "dishes": list(out), "placed_in_isaac": placed_in, "state_dir": _rel(fin_dir), "per_dish": out}


def build_fifteen(path, order=None):
    """--sample <sample> --fifteen (plans/2026-11-04-fifteen-of-fifteen.md): samples_v1/<sample_id>__fifteen/load.json from the access
    order beside it (order.json, frigidaire_access_order.py): every dish in the dag order with its chain, hold, drive, move (bowls
    D-ring or D-ring-side, the build's sites D-ring-insert and D-side-insert, cups D-side-down, plates I4-wall20 with the pinch spin at
    the goal), descent start and gate verdict; the chains in the access order (a dish the order predicts blocked runs after its chain's
    placed dishes; the order's reversed access edges, --dag soft, in reversed_edges), the order's rejects as reject rows (no Kit
    process), the mount file and its counter spot; presentation.json (the plank) and bowl_supports.json beside it. ``order``
    (--order): that order.json instead, the files written beside it. Third round: "basket" (the order's; false: every row's
    "episode_flags" carry --no-basket), a plate passed by the tine-guided drop the move I4-wall20-drop with "release" {mode,
    release_mm, guided_mm} (a seat release keeps I4-wall20), and "round1" (round1_block: the scene each dish no Isaac run placed
    yet restores before its process)."""
    import shutil
    path = path if path.is_absolute() else ROOT / path
    smp = json.loads(path.read_text())
    sid, goal, dag = smp["metadata"]["sample_id"], smp["goal"], smp["dag"]
    name = f"{sid}__fifteen"
    o_path = (order if order.is_absolute() else ROOT / order) if order is not None else SAMPLES / name / "order.json"
    d_out = o_path.parent
    od = json.loads(o_path.read_text())
    if od["sample_id"] != sid:
        raise SystemExit(f"[RESULT] FAIL {o_path} is the order of {od['sample_id']}, not {sid}")
    base = json.loads(BASELINE.read_text())
    snap = base.get("snapshot", base)
    off = {j: abs(float(snap["joints"][j]) - float(v)) for j, v in goal["rack_joints"].items()}
    if max(off.values()) > 1e-6:
        raise SystemExit(f"[RESULT] FAIL {sid}: the baseline's rack joints differ from goal.rack_joints {off}")
    kind = lambda n: n.split("_")[0]
    steps = {s_["dish"]: (c, s_) for c, ch in od["chains"].items() for s_ in ch["steps"] + list(ch["blocked"].values())}
    rejects = od["rejects"]
    miss = [n for n in dag["order"] if n not in steps and n not in rejects]
    if miss:
        raise SystemExit(f"[RESULT] FAIL {o_path} has no step for {miss} (run the access order on both chains)")
    chains = {c: list(ch["order"]) + list(ch["blocked"]) for c, ch in od["chains"].items()}
    builds = build_releases(smp, [n for n in od["insert_sites"] if n in goal["slots"]])   # as the access order's gate had them
    mf = Path(od["mount_file"]) if Path(od["mount_file"]).is_absolute() else ROOT / od["mount_file"]
    mb = json.loads(mf.read_text())["best"] if mf.is_file() else {}
    spot = [float(v) for v in (od.get("spot_xy") or mb.get("spot_xy") or PICK_SPOT_XY)]   # the mount's counter spot, else the study's
    if mb.get("spot_xy") is not None and max(abs(float(u) - v) for u, v in zip(mb["spot_xy"], spot)) > 1e-9:
        raise SystemExit(f"[RESULT] FAIL {o_path}: the order's counter spot {spot} is not the mount's {mb['spot_xy']} ({mf})")
    basket = bool(od.get("basket", True))                # --no-basket in the order: the robot rounds load the twin without it
    pres = od.get("presentation_used") or json.loads((ROOT / od["presentation"]).read_text())   # the plank as the order used it
    (d_out / "presentation.json").write_text(json.dumps(pres, indent=1) + "\n")   # (moved with the mount's spot): the plates' pick
    dishes = []
    for k, n in enumerate(dag["order"]):
        s_, p = goal["slots"][n], goal["poses"][n]
        c, st = steps.get(n) or (None, rejects[n])
        rb = (goal.get("robot") or {}).get(n)            # the hardware-aware sampler's move, hold and spin (goal.robot), when present
        if rb:
            st = dict(st, hold=rb["hold"], move=rb["move"], **({"goal_spin_deg": rb.get("spin_deg")} if kind(n) == "plate" else {}))
        hold, drive = st["hold"], FIFTEEN_DRIVE[st["hold"]]
        drop = kind(n) == "plate" and bool(st.get("pass")) and (st.get("release") or {}).get("mode") == "drop"
        row = {"name": n, "kind": kind(n), "grasp": FIFTEEN_GRASP[hold], "rack": s_["rack"], "slot": s_["slot"], "variant": s_["variant"],
               "order_index": k, "chain": c, "chain_index": chains[c].index(n) if c else None,
               "move": DROP_MOVE if drop else st["move"], "hold": hold, "episode_flags": [] if basket else [NO_BASKET_FLAG],
               "grip_drive": drive, "grip_drive_arg": ",".join(f"{k_}={v_:g}" for k_, v_ in drive.items()),
               "pick": {"spot_xy": spot,
                        "orientation": "mouth down" if kind(n) in ("bowl", "cup") else
                        f"standing in its goal orientation on the plank ({pres['name']})",
                        **({"presentation": "presentation.json"} if kind(n) == "plate" else {})},
               "goal_pose": {"pos": p["position_m"], "quat_xyzw": p["quaternion_xyzw"]},
               "insert_axis": ("the build's move (its moves_world pose), down to the build's stop" if n in builds else
                               "the tine axis" if kind(n) == "plate" else "the rack's up axis"),
               "access": {**{k_: st.get(k_) for k_ in ("gate", "pass", "why", "scene", "start")},
                          **({"reversed_access": st["reversed_access"]} if st.get("reversed_access") else {}),
                          **({"seat_at_goal": st["seat_at_goal"]} if st.get("seat_at_goal") else {})}}
        if n in builds:
            row["build_release"] = builds[n]
        if kind(n) == "plate":
            row["pinch_spin_deg"] = st.get("goal_spin_deg")
        if drop:                                          # the tine-guided release: the jaws open release_mm above the seat
            row["release"] = {"mode": "drop", **{k_: st["release"].get(k_) for k_ in ("release_mm", "guided_mm")}}
        if hold == "foot-side":
            row["ring_side_order"] = "straight"           # the episode's --ring-side-order straight (bowl_04 try c's start order)
        if c is not None and n in od["chains"][c]["blocked"]:
            row["predicted_blocked"] = {"scene": st.get("scene"), "why": st.get("why"), "culprits": st.get("culprits")}
        if n in rejects:
            row["reject"] = {"cause": "unreachable on this mount", "detail": st.get("why"), "blockers": st.get("blocker") or [],
                             "source": f"{_rel(o_path)} (the access order's hardware-only gate)"}
        dishes.append(row)
    moves = {"cup": "D-side-down", "bowl": "D-ring", "plate": "I4-wall20"}
    load = {"load": name, "plan": "plans/2026-11-04-fifteen-of-fifteen.md", "source_sample": str(path.relative_to(ROOT)),
            "kind": "mixed", "kinds": sorted({d["kind"] for d in dishes}), "moves": moves,
            "move_overrides": {d["name"]: d["move"] for d in dishes if d["move"] != moves[d["kind"]]},
            "pick_spot_xy": spot, "mount_file": od["mount_file"], "mount": {k_: mb.get(k_) for k_ in ("base", "yaw_rad", "spot_xy")},
            "pose_convention": "base frame (the twin's world), metres, Z-up, quaternion XYZW; goal.poses of the sample",
            "frames": {r: snap["poses"][r] for r in ("LowerRack", "UpperRack", "SilverwareBasket")},
            "frames_source": f"{BASELINE.relative_to(ROOT)} (both racks extended, joints = goal.rack_joints)",
            "rack_joints": goal["rack_joints"], "dishes": dishes, "order": list(dag["order"]),
            "edges": [[e["a"], e["b"]] for e in dag["edges"]], "edge_kinds": [e.get("kind") for e in dag["edges"]],
            "chains": chains, "executed_order": [n for c in ("upper", "lower") for n in chains.get(c, [])],
            "executed_order_rule": "each rack's chain in the access order (order.json: a linearization of the dag, with --dag soft its "
                                   "access edges reversible for the arm's chain (reversed_edges; the sample's dag untouched), whose "
                                   "every step passes the Kit-free gate with the chain's earlier dishes at their predicted landed "
                                   "poses), then the dishes the order predicts blocked; the rejects run nowhere",
            "dag": od.get("dag", "hard"),
            "reversed_edges": [{"chain": c, "edge": f_["edge"], "now": f_["now"], "kind": f_["kind"],
                                "gate": {k_: (f_.get("c_with_x") or {}).get(k_) for k_ in ("dish", "scene", "choice", "gate", "pass")},
                                "rule": f_.get("rule")}
                               for c, fl_ in (od.get("reversed") or {}).items() for f_ in fl_],
            "rejects": sorted(rejects), "grip_drives": {h: FIFTEEN_DRIVE[h] for h in FIFTEEN_DRIVE},
            "media_dir": f"data/media/robot/samples_v1/{name}", "presentation": "presentation.json", "order_file": "order.json",
            "gate_file": "gate.json", "basket": basket,
            "episode_flags_rule": ("every row's episode_flags go on its episode command line (--no-basket: the order ran "
                                   "without the SilverwareBasket, the robot rounds load the twin without it; s0's goal poses "
                                   "and S_ref keep it)"),
            "round1": round1_block(sid, od, smp, snap, chains)}
    if od.get("executed_order") is not None and load["executed_order"] != od["executed_order"]:
        raise SystemExit(f"[RESULT] FAIL the load's executed order {load['executed_order']} is not the order's {od['executed_order']}")
    (d_out / "load.json").write_text(json.dumps(load, indent=1) + "\n")
    shutil.copyfile(SAMPLES / sid / "bowl_supports.json", d_out / "bowl_supports.json")   # the s0 bowls' seated-test supports
    print(f"[RESULT] PASS fifteen load {name}: {[(d['name'], d['move'], d.get('pinch_spin_deg')) for d in dishes if 'reject' not in d]}; "
          f"rejects {load['rejects']}; chains {chains}; reversed {[(e['chain'], e['edge']) for e in load['reversed_edges']]}; "
          f"basket {basket}; round1 { {n: (v['state_after'], sorted(v['compose']), v['remove']) for n, v in load['round1']['per_dish'].items()} } -> "
          f"{_rel(d_out / 'load.json')}")


def sample_rows(d):
    """--rows: rows.jsonl of a samples_v1 load, one row per dish in the dag order: the Kit-free gate (gate.json), the last
    Isaac row of the dish (dishes.jsonl) and its media (the media folder's clip_/release_/landed_<dish> files)."""
    d = d if d.is_absolute() else ROOT / d
    load = json.loads((d / "load.json").read_text())
    gate = json.loads((d / "gate.json").read_text())["dishes"]
    isaac, tries = {}, {}
    if (d / "dishes.jsonl").is_file():
        for line in (d / "dishes.jsonl").read_text().splitlines():
            if line.strip():
                r = json.loads(line)
                isaac[r["dish"]] = r                       # the last row of a dish counts
                tries.setdefault(r["dish"], []).append(r)  # (the ring_side load: one row per try)
    media = ROOT / load["media_dir"] if load.get("media_dir") else ROOT / "data/media/robot/samples_v1" / load["load"]
    rel = lambda p: str(p.relative_to(ROOT)) if p.is_file() else None
    ig = None                                          # the plates' Isaac grasp gate (plans/2026-11-04-plate-pick-presentation.md;
    gf = d / "partA_plate_wall20" / "trials.jsonl"     # the wall pinch's of plans/2026-11-04-plate-wall-pinch.md first)
    gf = gf if gf.is_file() else d / "partA_plate_edge_lean30" / "trials.jsonl"
    if gf.is_file():
        tr = [json.loads(x) for x in gf.read_text().splitlines() if x.strip()]
        held_ = sum(bool(t.get("g3")) for t in tr)
        ig = {"run": gf.parent.name, "held": held_, "trials": len(tr),
              "outcomes": {o: sum(t.get("outcome") == o for t in tr) for o in sorted({t.get("outcome") for t in tr})}}
    c300 = (json.loads((d / "carry300.json").read_text()).get("bowls") or {}) if (d / "carry300.json").is_file() else {}
    out = []
    per_try = bool(load.get("isaac_order"))           # plans/2026-11-04-ring-side-hold.md: every Isaac try keeps its own row
    todo = [(x, r_) for x in sorted(load["dishes"], key=lambda x: x["order_index"])
            for r_ in ((tries.get(x["name"]) or [None]) if per_try else [isaac.get(x["name"])])]
    for x, r in todo:
        n, g = x["name"], gate.get(x["name"]) or {}
        rid = Path(r["record"]).parent.name if r and r.get("record") else None
        sfx = "" if not per_try or r is None or r is tries[n][0] else "_" + rid.rsplit("_", 1)[-1]   # try b's media: <name>_b
        row = {"dish": n, "kind": x["kind"], "slot": x["slot"], "rack": x["rack"], "order_index": x["order_index"],
               "move": x["move"], "gate": g.get("gate"), "gate_pass": bool(g.get("pass")), "gate_why": g.get("why"),
               "gate_blocker": g.get("blocker"), "ran_isaac": r is not None,
               "gate_release_check": None if "release_check" not in g else {k_: g["release_check"].get(k_) for k_ in
                                                                           ("ok", "spin_deg", "stages")}}
        if r is None and n in (load.get("kitfree_only") or ()):   # plans/2026-11-04-ring-side-hold.md: Kit-free rows only
            row.update(picked=False, carried=False, seated=False, at_goal=False,
                       cause="Kit-free only (the plan: placed today by the axial hold); gate " + str(g.get("gate"))
                             + ("" if g.get("pass") else ": " + str(g.get("why"))))
        elif r is None and c300.get(n) is not None and not c300[n].get("found"):   # the coordinator's option C (300 s search)
            row.update(picked=False, carried=False, seated=False, at_goal=False, cause="no Kit-free carry path at 300 s",
                       carry300={k_: c300[n].get(k_) for k_ in ("seconds", "found", "legs", "start", "release_check")})
        elif r is None and x.get("reject"):            # plans/2026-11-04-s0-rerun.md: a reject row (no Kit process)
            row.update(picked=False, carried=False, seated=False, at_goal=False, cause=x["reject"]["cause"],
                       cause_detail=x["reject"].get("detail"), blockers=x["reject"].get("blockers"))
        elif r is None and ig and ig["held"] < 8 and x["kind"] == "plate":   # the plates' Isaac gate held fewer than 8
            row.update(picked=False, carried=False, seated=False, at_goal=False,
                       cause=f"no measured pick: the Isaac grasp gate {ig['run']} held {ig['held']}/{ig['trials']} {ig['outcomes']}",
                       isaac_grasp_gate=ig)
        elif r is None:
            row.update(picked=False, carried=False, seated=False, at_goal=False,
                       cause=("gate: " + str(g.get("why"))) if not g.get("pass") else
                       (f"not run: the Isaac grasp gate {ig['run']} held {ig['held']}/{ig['trials']} {ig['outcomes']} (fewer "
                        f"than 8 held ends the chain)" if ig and ig["held"] < 8 and x["kind"] == "plate" else
                        "not run (no Isaac row)"))
            if ig and x["kind"] == "plate":
                row["isaac_grasp_gate"] = ig
        else:
            s = r.get("summary") or {}
            if per_try:                                # the try, its start order and the carry's leg plans
                row["try"] = rid.rsplit("_", 1)[-1] if rid else None
                row.update(run_id=rid, carry_plans=(r.get("legs") or {}).get("plans"),
                           start_choice={k_: (r.get("start_choice") or {}).get(k_) for k_ in (
                               "ring_side_order", "straight_candidates", "first_all_straight", "rule1_rank_of_first")})
            row.update(picked=bool(s.get("picked")), carried=bool(s.get("carried")), seated=bool(s.get("seated")),
                       at_goal=bool(s.get("at_goal")), cause=s.get("cause"), failed_at=r.get("failed_at"),
                       why=r.get("why"), pick=s.get("pick"), carry_drift_mm=s.get("carry_drift_mm"),
                       final_pose=r.get("final_pose"), error=r.get("error"), wall_s=r.get("wall_s"), sim_s=r.get("sim_s"),
                       record=r.get("record"), state_in=r.get("state_in"), state_out=r.get("state_out"), traj=r.get("traj"),
                       landing=s.get("goal_distance"), carry_legs=s.get("carry_legs"), carry_clean=s.get("carry_clean"),
                       tilt_cap=r.get("tilt_cap"), carry_retry=r.get("carry_retry"), stop=r.get("stop"),
                       stop_check=r.get("stop_check"), build_release=r.get("build_release"))
            rec_ = ROOT / r["record"] if r.get("record") else None   # plans/2026-11-04-grip-force.md: the finger drive the row ran at
            if rec_ is not None and rec_.is_file():                  # (the run record's flags R2; fields older records lack are the
                fl_ = json.loads(rec_.read_text()).get("flags") or {}   # robot_cfg constants of their day)
                row["grip_drive"] = {k_: fl_.get(k_, v_) for k_, v_ in GRIP_DRIVE_BEFORE.items()}
            if ig and x["kind"] == "plate":
                row["isaac_grasp_gate"] = ig
        if g.get("tilt_cap"):
            row["gate_tilt_cap"] = g["tilt_cap"]
        if x.get("descent_bias_mm") is not None:      # round 3 (the user, 2026-11-05 00:05 EST): the row's tine-axis lateral bias,
            row["descent_bias_mm"] = x["descent_bias_mm"]   # the gate's drop retreat and the episode's rest wait and retreat
        if g.get("drop_retreat"):
            row["gate_drop_retreat"] = {k_: g["drop_retreat"].get(k_) for k_ in ("variant", "back_mm", "rise_mm")}
        if (g.get("release") or {}).get("stop_mode"):
            row["gate_stop_mode"] = g["release"]["stop_mode"]
        for k_ in ("drop_rest", "drop_retreat"):
            if (r or {}).get(k_) is not None:
                row[k_] = ({kk: r[k_].get(kk) for kk in ("variant", "back_mm", "rise_mm", "release", "checks")}
                           if k_ == "drop_retreat" else r[k_])
        if x.get("move") == "D-ring-side":            # the carry columns: the chosen start's tilt split over the hand
            cb_ = (r or {}).get("combo") or {}
            src_ = cb_ if cb_.get("pitch_deg") is not None else ((g.get("release") or {}).get("best") or {})
            row["gate_pick"] = {k_: (g.get("pick") or {}).get(k_) for k_ in ("pass", "of", "roll", "best_yaw_k", "ranked")}
            row["carry_start"] = {"spin_deg": src_.get("spin_deg"), "start_mm": src_.get("descent_start_above_goal_mm",
                                                                                         src_.get("start_mm")),
                                  **{k_: src_.get(k_) for k_ in ("pitch_deg", "roll_deg", "tilt_deg")},
                                  "source": "the Isaac row's combo" if cb_.get("pitch_deg") is not None else "the Kit-free gate"}
            if src_.get("pitch_deg") is not None and src_["pitch_deg"] > (src_.get("roll_deg") or 0.):
                row["carry_risk"] = (f"pitch-dominant start: {src_['pitch_deg']} of the {src_.get('tilt_deg')} deg goal tilt about "
                                     f"the hand's closing axis (torsional friction only, the creep direction), {src_.get('roll_deg')} "
                                     f"deg about the approach")
        row.update(clip=rel(media / f"clip_{n}{sfx}.mp4"), release_still=rel(media / f"release_{n}{sfx}.png"),
                   landed_still=rel(media / f"landed_{n}{sfx}.png"))
        out.append(row)
    (d / "rows.jsonl").write_text("".join(json.dumps(r_) + "\n" for r_ in out))
    k_ = sorted({r_["kind"] for r_ in out})
    rate = {k: {c: f"{sum(r_[c] for r_ in out if r_['kind'] == k)}/{sum(r_['kind'] == k for r_ in out)}"
                for c in ("gate_pass", "picked", "carried", "seated", "at_goal")} for k in k_}
    print(f"[RESULT] PASS rows {load['load']}: {len(out)} rows, Isaac {sum(r_['ran_isaac'] for r_ in out)}; per kind {rate} -> "
          f"{(d / 'rows.jsonl').relative_to(ROOT)}")


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--summarize", action="store_true")
    ap.add_argument("--sample", type=Path, default=None, help="a four-block sample -> samples_v1/<sample_id>/load.json")
    ap.add_argument("--rows", type=Path, default=None, help="a samples_v1/<sample_id> folder -> its rows.jsonl")
    ap.add_argument("--cups-down", action="store_true",
                    help="--sample: the cups alone, picked mouth down, move D-side-down (plans/2026-11-04-cups-mouth-down.md) "
                         "-> samples_v1/<sample_id>__cups_down/load.json")
    ap.add_argument("--fixes", action="store_true",
                    help="--sample: the three per-kind loads of the three-fixes window (FIXES) -> samples_v1/<sample_id>__fixes/"
                         "{carry,plates,sites}/load.json")
    ap.add_argument("--final", action="store_true",
                    help="--sample: the s0 rerun's load (plans/2026-11-04-s0-rerun.md) -> samples_v1/<sample_id>__final/load.json")
    ap.add_argument("--ring-side", action="store_true",
                    help="--sample: the bowls with the side ring hold (plans/2026-11-04-ring-side-hold.md) -> samples_v1/"
                         "<sample_id>__ring_side/load.json")
    ap.add_argument("--fifteen", action="store_true",
                    help="--sample: the fifteen-of-fifteen load from samples_v1/<sample_id>__fifteen/order.json (frigidaire_access_order"
                         ".py; plans/2026-11-04-fifteen-of-fifteen.md) -> load.json beside it")
    ap.add_argument("--order", type=Path, default=None,
                    help="--fifteen: build from this order.json (default samples_v1/<sample_id>__fifteen/order.json) and write "
                         "load.json beside it")
    ap.add_argument("loads", nargs="*", help="only these loads (default: all; a named load leaves the others untouched)")
    a = ap.parse_args()
    if a.sample is not None and a.fifteen:
        return build_fifteen(a.sample, a.order)
    if a.sample is not None and a.ring_side:
        return build_ring_side(a.sample)
    if a.sample is not None and a.fixes:
        return build_fixes(a.sample)
    if a.sample is not None and a.final:
        return build_final(a.sample)
    if a.sample is not None:
        return build_sample(a.sample, cups_down=a.cups_down)
    if a.rows is not None:
        return sample_rows(a.rows)
    for name, spec in LOADS.items():
        if a.loads and name not in a.loads:
            continue
        summarize(name) if a.summarize else build(name, *spec)


if __name__ == "__main__":
    main()
