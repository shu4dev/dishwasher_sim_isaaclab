#!/usr/bin/env python3
"""Kit-free sweep of the plate picked in its own goal orientation (plans/2026-11-04-plate-wall-pinch.md, amendment phase 1b).

    PYTHONPATH=code/src .venv-kitfree/bin/python code/execution/frigidaire/frigidaire_plate_goal_lean.py --workers 6

The plate stands on its rim edge at the counter spot frigidaire_robot_reach.COUNTER_XY in the s0 plate goals' orientation
(the sample's goal.slots rack-local quaternion of plate_05, plate_03 and plate_04, composed with the arm world's LowerRack
frame), turned about world z through the spot by the facing (12 facings, 30 deg steps; facing 0 = the goal orientation
itself), the rim circle's centre over the spot and its lowest point 2 mm above the slab (frigidaire_plate_presentations.
edge_pose's rule). The backstop stands on the side the plate's top edge overhangs: a 20 x 160 x 100 mm box, its long side
along the lean axis, slid toward the plate until the leaning face is 1 mm from it (FCL bisection, as phase 1's backstop).
The hold is frigidaire_plate_wall_pinch's 20 mm wall-normal pinch (imported, both rolls), its pinch at the rim point highest
at the pose spun by 0, 30 or -30 deg (EP.sampler_bowl_tcp, as frigidaire_site_sweep.py). Per (facing, roll, spin): phase 1's
pick gate with the backstop in the scene (frigidaire_plate_presentations.pick: the 15 mm arm margin, the 1 mm held rule, the
open gripper's closest approach), the pivot moment, the plate's turn from the pick to each goal at the same spin, then the
I4 check at plate_05, plate_03 and plate_04 (frigidaire_plate_wall_pinch.i4_check from the 0.15 m lift: the descent start
50 to 150 mm, D-seat's 2 mm rule on the final descent, leg 1 straight else RRT-Connect with the rerun's 60 s budget) and
the joint move (the largest joint travel from the lift to the descent start). I4 jobs run facing 0 first, then the facings
nearest it; jobs not started by the deadline are recorded as not run. Chosen: the (facing, roll, spin) with the most I4
passes over the three goals, ties by the smallest total joint move; null when nothing passes.
Writes data/results/robot/samples_v1/<s0>__fixes/plates_wall2/{goal_lean.json,kitfree.md}. Imports the phase 1 scripts
as libraries only.
"""
from __future__ import annotations

import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
import sys
import time

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
sys.path[:0] = [str(ROOT / "code/execution/frigidaire"), str(ROOT / "code/src"), str(ROOT / "code/frigidaire/src")]
import frigidaire_plate_wall_pinch as WP            # noqa: E402  the 20 mm wall-normal hold, its world, the I4 check
PP, R, EP = WP.PP, WP.R, WP.EP
PP.CARRY_RRT_S = 60.                                # the rerun's leg-1 RRT-Connect budget (read by WP.i4_check at call time;
                                                    # this process only, no file is changed)

SAMPLE = WP.SAMPLE
SAMPLE_JSON = ROOT / "data/results/bench/samples" / f"{SAMPLE}.json"
OUT = ROOT / "data/results/robot/samples_v1" / f"{SAMPLE}__fixes" / "plates_wall2"
FACINGS_DEG = tuple(30 * k for k in range(12))
SPINS = (0., 30., -30.)
ROLLS = (0, 180)
PLATES = WP.PLATES                                  # plate_05, plate_03, plate_04
BACK_H_M, BACK_T_M, BACK_W_M, BACK_TOUCH_M = .100, PP.BACK_T_M, PP.BACK_W_M, PP.BACK_TOUCH_M
I4_DEADLINE_S = 22 * 60.                            # no I4 job starts later than this after the run's start
IMPORTED = WP.IMPORTED + ("code/execution/frigidaire/frigidaire_plate_wall_pinch.py",)
_W = {}


def world():
    W = WP.world()
    if "goal_rot" not in _W:
        _W["goal_rot"] = goal_rotation(W)
    return W


def goal_rotation(W):
    """The goal orientation in the arm world: the sample's slot quaternion (equal for PLATES, checked) composed with the arm
    world's LowerRack frame; with that frame's angle from the world axes."""
    from dishsim_frigidaire.robot.rig import T_of
    S = json.loads(SAMPLE_JSON.read_text())
    qs = {n: S["goal"]["slots"][n]["rack_local_pose"]["quaternion_xyzw"] for n in PLATES}
    q0 = np.asarray(qs[PLATES[0]], dtype=float)
    spread = max(PP.rot_deg(T_of((0, 0, 0), q0)[:3, :3], T_of((0, 0, 0), q)[:3, :3]) for q in qs.values())
    fr = R.frames_for(W.B, "LowerRack")["LowerRack"]
    F_r = T_of(fr["position_m"], fr["quaternion_xyzw"])
    Rw = F_r[:3, :3] @ T_of((0, 0, 0), q0)[:3, :3]
    return {"R": Rw, "slot_quats": qs, "slot_quat_spread_deg": spread,
            "rack_frame_vs_world_deg": PP.rot_deg(np.eye(3), F_r[:3, :3]),
            "axis_world": np.round(Rw[:, 2], 5).tolist(),
            "axis_below_horizontal_deg": round(math.degrees(math.asin(-float(Rw[2, 2]))), 2)}


def rot_z(a):
    T = np.eye(3)
    T[:2, :2] = [[math.cos(a), -math.sin(a)], [math.sin(a), math.cos(a)]]
    return T


def spawn_pose(W, facing_deg):
    """The plate at the spot in the goal orientation turned by facing_deg about world z: the rim circle's centre over the
    spot, the lowest point COUNTER_LIFT_M above the slab (edge_pose's rule)."""
    g = W.geom
    Rw = rot_z(math.radians(facing_deg)) @ _W["goal_rot"]["R"]
    a = Rw[:, 2]
    T = np.eye(4)
    T[:3, :3] = Rw
    T[:3, 3] = np.array([R.COUNTER_XY[0], R.COUNTER_XY[1], 0.]) - g["height"] * a
    T[2, 3] += .914 + R.COUNTER_LIFT_M - float((g["points"] @ T[:3, :3].T + T[:3, 3])[:, 2].min())
    return T


def backstop(W, T, body):
    """The backstop on the side the plate's top edge overhangs (d: the horizontal part of the in-plane up): a BACK_T_M x
    BACK_W_M x BACK_H_M box on the slab, its long side along the lean axis z x d, slid from 0.40 m out along d toward the
    plate until the plate is BACK_TOUCH_M from it (FCL bisection). ([body], info)"""
    fcl = W.arm.fcl
    Pw = W.geom["points"] @ T[:3, :3].T + T[:3, 3]
    low = Pw[int(np.argmin(Pw[:, 2]))]
    a = T[:3, 2]
    n = np.array([0., 0., 1.]) - a[2] * a
    d = np.array([n[0], n[1], 0.]) / np.linalg.norm(n[:2])
    t = np.cross([0., 0., 1.], d)
    Rm = np.column_stack([d, t, [0., 0., 1.]])

    def at(u):                                       # the box's near face u along d from the plate's lowest point
        c = np.array([low[0], low[1], .914 + BACK_H_M / 2]) + (u + BACK_T_M / 2) * d
        return PP.box_body(fcl, "Backstop", c, (BACK_T_M, BACK_W_M, BACK_H_M), Rm)
    clear, hit = .40, float(((Pw - low) @ d).min())
    if PP.dist(fcl, body, at(clear)) <= BACK_TOUCH_M:
        raise RuntimeError("the backstop's far position already touches the plate")
    for _ in range(40):
        mid = (clear + hit) / 2
        clear, hit = (mid, hit) if PP.dist(fcl, body, at(mid)) > BACK_TOUCH_M else (clear, mid)
    b = at(clear)
    rows = (Pw - low) @ d
    data = fcl.DistanceData(request=fcl.DistanceRequest(enable_nearest_points=True))
    body["manager"].distance(b["objects"][0], data, fcl.defaultDistanceCallback)
    nps = [np.asarray(x, dtype=float) for x in data.result.nearest_points]
    near = min(nps, key=lambda p_: abs(float(p_[2]) - .914 - BACK_H_M)) if nps else None   # the box's point nearest the plate
    loc = None if near is None else np.linalg.inv(T) @ np.r_[near, 1.]
    info = {"height_m": BACK_H_M, "near_face_from_lowest_point_m": round(clear, 4),
            "touch_point_world": None if near is None else np.round(near, 4).tolist(),
            "touch_point_above_slab_mm": None if near is None else round((float(near[2]) - .914) * 1e3, 1),
            "touch_point_plate_frame_r_z_mm": None if loc is None else [round(float(np.hypot(loc[0], loc[1])) * 1e3, 1),
                                                                         round(float(loc[2]) * 1e3, 1)],
            "gap_mm": round(PP.dist(fcl, body, b) * 1e3, 2), "overhang_side": np.round(d, 5).tolist(),
            "plate_top_overhang_m": round(float(rows.max()), 4), "boxes": {"Backstop": b["box"]},
            "rule": "a 20 x 160 x 100 mm box on the slab on the side the plate's top edge overhangs, its long side along the "
                    "lean axis, slid toward the plate until the plate is 1 mm from it (FCL bisection)"}
    return [b], info


def pick_tcp(T_p, spin, roll, T_hold):
    """The wall pinch (the hold in the plate frame, T_hold) at the rim point highest at T_p spun by ``spin`` deg, rolled."""
    return EP.sampler_bowl_tcp(T_p, np.linalg.inv(T_hold), spin, np) @ (WP.RZ_PI if roll else np.eye(4))


def presentation(W, facing_deg):
    T_p = spawn_pose(W, facing_deg)
    body = W.arm.checker._body("plate", W.P.pose_dict(T_p), "dish")
    fx, info = backstop(W, T_p, body)
    return T_p, fx, info


def sweep_job(fi):
    """One facing: the backstop, then every (roll, spin) pick -> a row."""
    W = world()
    arm = W.arm
    t0 = time.monotonic()
    f_deg = FACINGS_DEG[fi]
    T_p, fx, info = presentation(W, f_deg)
    for b in fx:
        arm.statics[b["id"]] = b
    T_hold = W.holds[WP.DEPTH_MM]
    row = {"facing_deg": f_deg, "plate_pose": np.round(T_p, 6).tolist(), "spawn_pose": W.P.pose_dict(T_p), "fixture": info,
           "cands": {}}
    try:
        dish = arm.checker._body("plate", W.P.pose_dict(T_p), "dish")
        for roll in ROLLS:
            for spin in SPINS:
                T_g = pick_tcp(T_p, spin, roll, T_hold)
                rr = PP.pick(W, arm, dish, T_p, T_g, True)
                M = np.linalg.inv(T_g) @ T_p
                rr.pop("turn_to_goal_deg", None)         # phase 1's is at spin 0; the same spin's turn below
                rr.update(roll=roll, spin_deg=spin, T_tcp=np.round(T_g, 6).tolist(), M=np.round(M, 6).tolist(),
                          moment_Nm=dict(zip(("closing_axis", "pads"), PP.moment(W, T_g, M))),
                          moment_tcp_axes_Nm=WP.moment_split(W, T_g, M),
                          turn_to_goal_deg={nm: PP.rot_deg(T_p[:3, :3], (EP.sampler_bowl_tcp(g[0], M, spin, np) @ M)[:3, :3])
                                            for nm, g in W.goals.items()})
                row["cands"][f"{roll}/{spin:+.0f}"] = rr
    finally:
        for b in fx:
            arm.statics.pop(b["id"], None)
        arm.set_dishes({})
    row["seconds"] = round(time.monotonic() - t0, 1)
    return row


def i4_job(args):
    """One (facing, roll, spin, goal): WP.i4_check from that pick's lift with the facing's backstop in the scene; not run
    after the deadline."""
    fi, key, name, cand, deadline = args
    out = {"facing_deg": FACINGS_DEG[fi], "cand": key, "dish": name}
    if time.time() > deadline:
        out.update(run=False, pass_=False, gate="not run")
        out["pass"] = out.pop("pass_")
        return out
    W = world()
    arm = W.arm
    t0 = time.monotonic()
    T_goal, slot, F_r, _ = W.goals[name]
    _, fx, _ = presentation(W, FACINGS_DEG[fi])
    for b in fx:
        arm.statics[b["id"]] = b
    try:
        r = WP.i4_check(W, arm, T_goal, F_r, np.asarray(cand["M"]), np.asarray(cand["q_lift"]), np.asarray(cand["T_lift"]),
                        cand["spin_deg"])
    finally:
        for b in fx:
            arm.statics.pop(b["id"], None)
    if r.get("pass"):
        r["joint_move_deg"] = round(float(np.degrees(np.abs(np.asarray(r["q_start"]) - np.asarray(cand["q_lift"]))).max()), 1)
    out.update(r, run=True, slot=slot, seconds=round(time.monotonic() - t0, 1))
    return out


# -------------------------------------------------------------------------------------------------------------- report
def i4_cell(r, turn):
    """gate / start (mm) / leg 1 / turn (deg) / joint move (deg)."""
    if r is None:
        return "no pick"
    if not r.get("run"):
        return "not run (deadline)"
    if r["pass"]:
        return f"G2 / {r['start_h_mm']} / {r['leg1']['path']} / {turn} / {r['joint_move_deg']}"
    if r["gate"] == "G1":
        why = (r.get("why") or "").replace("G2: carry: ", "").replace("no leg 1 from the lift to the top of the tine-axis line",
                                                                       "no leg 1").replace("|", " vs ")
        return f"G1 / {r.get('start_mm')} / {why} / {turn} / -"
    blk = Counter()
    blk.update(r.get("start_blockers") or {})
    blk.update(r.get("descent_blockers") or {})
    top = ", ".join(k for k, _ in blk.most_common(2)).replace("|", " vs ") or (r.get("why") or "")[:60]
    return f"{r['gate']} / - / - / {turn} / - ({top})"


def write_md(res, path, cmd):
    gr, rows, i4 = res["goal_orientation"], res["rows"], res["i4"]
    L = ["# Plate picked in its goal orientation, Kit-free (phase 1b of plans/2026-11-04-plate-wall-pinch.md)", "",
         f"Command (Mac, {res['seconds']:.0f} s on {res['workers']} workers):", "", "```", cmd, "```", "",
         f"The plate stands on its rim edge at the spot in the s0 plate goals' orientation, the rack-local quaternion "
         f"{np.round(gr['slot_quats'][PLATES[0]], 4).tolist()} of plate_05, plate_03 and plate_04 (spread "
         f"{gr['slot_quat_spread_deg']} deg). The LowerRack frame is {gr['rack_frame_vs_world_deg']} deg from the world "
         f"axes, so the plate's axis points {gr['axis_world']} in the world, {gr['axis_below_horizontal_deg']} deg below "
         "horizontal, and its top edge overhangs the mouth face. Each facing turns that pose about world z through the spot; "
         "facing 0 is the goal orientation itself. The rim circle's centre is over the spot and the lowest point 2 mm above "
         "the slab. The backstop is a 20 x 160 x 100 mm box on the overhang side, its long side along the lean axis, with "
         "the leaning face 1 mm from it. The hold is frigidaire_plate_wall_pinch.py's 20 mm wall-normal pinch at the rim "
         "point highest at the pose, spun by the spin column. Roll 180 turns the hand 180 deg about the approach.", "",
         "The pick follows phase 1's rules with the backstop in the scene: the arm and gripper at the 15 mm margin, the gripper "
         "allowed to touch the plate in the last 60 mm and at the grasp, the held plate under the 1 mm overlap rule and 2 mm "
         "from the links, G1 at the hover, the grasp and the 0.15 m lift, G2 along the approach line, the lift line and the "
         f"home transit. Pivot compares the plate's weight moment on the pads with phase 1's a ({res['a_pads_Nm']} N m). "
         "I4 runs from the pick's lift at the same spin: a descent start 50 to 150 mm above the seat with a free IK at the "
         "15 mm carry margin, D-seat's 2 mm rule on the final descent, the line up the tine axis, leg 1 straight or by "
         "RRT-Connect in 60 s, and the line down. An I4 cell reads gate / start (mm) / leg 1 / the plate's turn from the "
         "pick to the goal (deg) / the joint move from the lift to the descent start (deg).", ""]
    ran = sorted({r["facing_deg"] for r in i4.values() if r.get("run")})
    L += [f"I4 ran at facings {', '.join(map(str, ran)) or 'none'} deg"
          + (f"; the deadline stopped {sum(1 for r in i4.values() if not r.get('run'))} jobs." if
             any(not r.get("run") for r in i4.values()) else "; every passing pick was checked.") , ""]
    for roll in ROLLS:
        L += [f"Roll {roll}:", "",
              "| facing (deg) | spin (deg) | pick | commonest blockers | pivot (pads N m) | gripper to plate, last 60 mm (mm) "
              "| plate_05 | plate_03 | plate_04 |", "|---|---|---|---|---|---|---|---|---|"]
        for row in rows:
            for spin in SPINS:
                key = f"{roll}/{spin:+.0f}"
                c = row["cands"][key]
                bl = ("-" if c["ok"] else "; ".join(f"{k} ({v})" for k, v in list((c.get("blockers") or {}).items())[:2])
                      .replace("|", " vs ") or c.get("stage") or "-")
                cells = [i4_cell(i4.get(f"{row['facing_deg']}/{key}/{n}"), c["turn_to_goal_deg"][n]) if c["ok"] else "no pick"
                         for n in PLATES]
                L.append(f"| {row['facing_deg']} | {spin:+.0f} | {'G2' if c['ok'] else c['gate'] + ' (' + str(c['stage']) + ')'} "
                         f"| {bl} | {c['moment_Nm']['pads']} | "
                         f"{c['approach_last60']['gripper_plate_min_mm'] if c['ok'] else '-'} | " + " | ".join(cells) + " |")
        L.append("")
    ch = res["chosen"]
    if ch:
        L += [f"Chosen: facing {ch['facing_deg']} deg, roll {ch['roll']}, spin {ch['spin_deg']:+.0f} deg, with I4 passes at "
              f"{', '.join(ch['passes'])} ({len(ch['passes'])} of 3) and a total joint move of {ch['joint_move_total_deg']} "
              f"deg. Its pick (TCP, hold, plate pose, backstop, joint states, lift) and per-goal I4 records are under chosen "
              "in goal_lean.json.", ""]
    else:
        L += ["Chosen: none. No (facing, roll, spin) passes I4 at any of the three goals; goal_lean.json has chosen null.", ""]
    rk = res["ranking"][:5]
    if rk:
        L += ["Best rows by I4 passes, then the smallest total joint move: " + "; ".join(
              f"facing {x['facing_deg']}, roll {x['roll']}, spin {x['spin_deg']:+.0f}: {x['n_pass']} passes "
              f"({', '.join(x['passes']) or 'none'}), furthest gates {x['gates']}" for x in rk) + ".", ""]
    path.write_text("\n".join(L) + "\n")


def sha16(rel):
    return hashlib.sha256((ROOT / rel).read_bytes()).hexdigest()[:16]


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--workers", type=int, default=6)
    a = ap.parse_args()
    import multiprocessing as mp
    t0, t0_wall = time.monotonic(), time.time()
    OUT.mkdir(parents=True, exist_ok=True)
    shas = {f: sha16(f) for f in IMPORTED}
    a_pads = json.loads(WP.PHASE1.read_text())["summary"]["a"]["moment_Nm"]["pads"]
    print(f"[INFO] {len(FACINGS_DEG)} facings x {len(ROLLS)} rolls x {len(SPINS)} spins on {a.workers} workers; leg 1 RRT "
          f"{PP.CARRY_RRT_S:.0f} s; I4 deadline {I4_DEADLINE_S / 60:.0f} min", flush=True)
    rows, i4 = [], {}
    with mp.get_context("fork").Pool(a.workers) as pool:  # the World is built in the workers only (Accelerate and fork)
        for r in pool.imap_unordered(sweep_job, range(len(FACINGS_DEG)), chunksize=1):
            rows.append(r)
            print(f"[INFO] facing {r['facing_deg']:3d}: " + ", ".join(f"{k} {'ok' if c['ok'] else c['stage']}"
                                                                    for k, c in r["cands"].items()) + f" {r['seconds']} s",
                  flush=True)
        rows.sort(key=lambda r: r["facing_deg"])
        order = sorted(range(len(FACINGS_DEG)), key=lambda i: (min(i, len(FACINGS_DEG) - i), i))
        jobs = [(fi, key, n, rows[fi]["cands"][key], t0_wall + I4_DEADLINE_S)
                for fi in order for key in rows[fi]["cands"] if rows[fi]["cands"][key]["ok"] for n in PLATES]
        print(f"[INFO] {len(jobs)} I4 jobs at {time.monotonic() - t0:.0f} s (facing order {[FACINGS_DEG[i] for i in order]})",
              flush=True)
        for r in pool.imap(i4_job, jobs, chunksize=1):
            i4[f"{r['facing_deg']}/{r['cand']}/{r['dish']}"] = r
            if r.get("run"):
                print(f"[INFO] I4 facing {r['facing_deg']} {r['cand']} {r['dish']}: pass {r['pass']} gate {r['gate']} "
                      f"{r['seconds']} s {str(r.get('why') or '')[:140]}", flush=True)
    W = world()
    gr = {k: v for k, v in _W["goal_rot"].items() if k != "R"}
    gr["R"] = np.round(_W["goal_rot"]["R"], 6).tolist()
    rank = []
    for row in rows:
        for key, c in row["cands"].items():
            if not c["ok"]:
                continue
            rs = {n: i4.get(f"{row['facing_deg']}/{key}/{n}") for n in PLATES}
            ps = [n for n, r in rs.items() if r and r["pass"]]
            rank.append({"facing_deg": row["facing_deg"], "roll": c["roll"], "spin_deg": c["spin_deg"], "n_pass": len(ps),
                         "passes": ps, "joint_move_total_deg": round(sum(rs[n]["joint_move_deg"] for n in ps), 1),
                         "gates": {n: (r or {}).get("gate") for n, r in rs.items()}, "key": key})
    rank.sort(key=lambda x: (-x["n_pass"], x["joint_move_total_deg"], abs(((x["facing_deg"] + 180) % 360) - 180)))
    chosen = None
    if rank and rank[0]["n_pass"]:
        x = rank[0]
        row = next(r for r in rows if r["facing_deg"] == x["facing_deg"])
        c = row["cands"][x["key"]]
        chosen = {**{k: x[k] for k in ("facing_deg", "roll", "spin_deg", "n_pass", "passes", "joint_move_total_deg")},
                  "hold": {"depth_mm": WP.DEPTH_MM, "T_tcp_in_plate": {"0": np.round(W.holds[WP.DEPTH_MM], 6).tolist(),
                                                                       "180": np.round(W.holds[WP.DEPTH_MM] @ WP.RZ_PI, 6).tolist()},
                           "source": "frigidaire_plate_wall_pinch.hold_in_plate(0.020), spun by EP.sampler_bowl_tcp"},
                  "plate_pose": row["spawn_pose"], "fixture": row["fixture"],
                  "pick": {k: c.get(k) for k in ("T_tcp", "M", "q_hover", "q_grasp", "q_lift", "T_lift", "facing", "move_rad",
                                                 "moment_Nm", "approach_last60", "transit", "turn_to_goal_deg")},
                  "i4": {n: i4.get(f"{x['facing_deg']}/{x['key']}/{n}") for n in PLATES}}
    print(f"[INFO] chosen {None if not chosen else (chosen['facing_deg'], chosen['roll'], chosen['spin_deg'], chosen['passes'])}",
          flush=True)
    res = {"plan": "plans/2026-11-04-plate-wall-pinch.md", "phase": "1b", "sample": str(SAMPLE_JSON.relative_to(ROOT)),
           "load": str(WP.LOAD.relative_to(ROOT)), "mount_file": str(R.SAMPLE_MOUNT.relative_to(ROOT)), "mount": W.mount,
           "counter_xy": list(R.COUNTER_XY), "goal_orientation": gr, "facings_deg": list(FACINGS_DEG), "spins_deg": list(SPINS),
           "hold": {"depth_mm": WP.DEPTH_MM, "record": WP.hold_record(WP.DEPTH_MM / 1e3, W.pad, W.holds[WP.DEPTH_MM]),
                    "pad": W.pad}, "a_pads_Nm": a_pads, "leg1_rrt_s": PP.CARRY_RRT_S,
           "i4_deadline_min": I4_DEADLINE_S / 60, "rule": "phase 1's pick gate with the backstop in the scene; I4 = "
           "frigidaire_plate_wall_pinch.i4_check at the same spin from the pick's 0.15 m lift, leg 1 RRT-Connect 60 s; chosen = "
           "the most I4 passes over plate_05, plate_03, plate_04, ties by the smallest total joint move (lift to descent start)",
           "rows": rows, "i4": i4, "ranking": rank, "chosen": chosen,
           "imported_sha256_16": shas, "workers": a.workers, "seconds": round(time.monotonic() - t0, 1)}
    (OUT / "goal_lean.json").write_text(json.dumps(res, indent=1, default=lambda o: o.tolist() if hasattr(o, "tolist")
                                                   else str(o)) + "\n")
    cmd = ("PYTHONPATH=code/src .venv-kitfree/bin/python code/execution/frigidaire/frigidaire_plate_goal_lean.py "
           f"--workers {a.workers}")
    write_md(res, OUT / "kitfree.md", cmd)
    n_ok = sum(c["ok"] for r in rows for c in r["cands"].values())
    print(f"[RESULT] PASS plate goal lean: picks {n_ok}/{len(rows) * len(ROLLS) * len(SPINS)}; I4 run "
          f"{sum(1 for r in i4.values() if r.get('run'))}/{len(i4)}, passes {sum(1 for r in i4.values() if r['pass'])}; chosen "
          f"{None if not chosen else (chosen['facing_deg'], chosen['roll'], chosen['spin_deg'])} in {res['seconds']} s -> "
          f"{(OUT / 'goal_lean.json').relative_to(ROOT)}", flush=True)


if __name__ == "__main__":
    main()
