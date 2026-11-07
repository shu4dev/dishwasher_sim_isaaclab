#!/usr/bin/env python3
"""Kit-free check of the plate in its goal orientation on a plank and a lip (plans/2026-11-04-plate-wall-pinch.md, phase 1c).

    PYTHONPATH=code/src .venv-kitfree/bin/python code/execution/frigidaire/frigidaire_plate_goal_plank.py --workers 6

The plate in the s0 goals' orientation (frigidaire_plate_goal_lean.spawn_pose: facings 0, 330 and 30 deg about world z, the
rim circle's centre over the spot, the lowest point 2 mm above the slab) rests on a plank whose top face is parallel to the
plate plane and touches the plate's extreme surface on one side, with a lip bar on the plank's downhill edge that the
bottom rim rests against (1 mm gap). Two sides are checked statically: "foot" (the plank's top face in the foot plane, under
the foot ring, the brief's fixture: 160 x 190 x 20 mm) and "rim" (the plank's top face in the rim plane, under the rim flat
on the mouth side). Static: gravity's component along the plank normal must press the plate onto the plank, the downhill
component must point at the lip, and the weight line (the vertical through the plate's visual centroid, phase 1's centre of
mass) must meet the plank plane inside the convex hull of the plate's contacts on the plank (visual points within 0.5 mm of
the plank's face and over it) and on the lip (within 1.5 mm of its face). When a fixture's lowest corner is below the slab,
the fixture and the plate rise together until it sits on the slab. The pick and the I4 check run on the static fixture:
frigidaire_plate_goal_lean's hold and checks (the 20 mm wall pinch, roll 180, spin 0; phase 1's pick gate; I4 at
plate_05, plate_03, plate_04 with leg 1 RRT-Connect 60 s) with the plank and the lip in the scene.
Writes data/results/robot/samples_v1/<s0>__fixes/plates_wall3/{goal_plank.json,kitfree.md}.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import sys
import time

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
sys.path[:0] = [str(ROOT / "code/execution/frigidaire"), str(ROOT / "code/src"), str(ROOT / "code/frigidaire/src")]
import frigidaire_plate_goal_lean as GL             # noqa: E402  the goal orientation, the hold, its pick (leg 1 RRT 60 s)
WP, PP, R, EP = GL.WP, GL.PP, GL.R, GL.EP

SAMPLE = GL.SAMPLE
OUT = ROOT / "data/results/robot/samples_v1" / f"{SAMPLE}__fixes" / "plates_wall3"
FACINGS_DEG = (0, 330, 30)
CANDS = ((180, 0.),)                                # (roll, spin): the phase-1b chosen hold
PLATES = GL.PLATES
PLANK_T_M, LIP_W_M, LIP_H_M, LIP_L_M, LIP_GAP_M = .020, .015, .015, .160, .001
BRIEF = {"side": "foot", "width_m": .160, "up_m": .190}           # the brief's plank: 160 wide, 190 up the plate plane
RIM_WIDTHS_M = (.160, .240)                         # the rim side: the brief's width, and one that covers the rim's sides
RIM_UPS_M = (.190, .170, .150, .130)                # the plank's reach up the plate plane, longest first (finger clearance)
CONTACT_M, LIP_CONTACT_M = .0005, .0015
_W = {}


def frame(T_p, side):
    """The plank frame of the plate at T_p: u = the in-plane up (uphill), v = the lean axis, w = the plank's normal toward the
    plate; the plank plane through the plate's extreme visual point along -w ("foot": the foot plane side, w = +a; "rim":
    the rim plane side, w = -a). (origin on the plane under the plate's lowest point along -u, u, v, w)"""
    W = GL.world()
    a = T_p[:3, 2]
    n = np.array([0., 0., 1.]) - a[2] * a
    u = n / np.linalg.norm(n)
    w = a.copy() if side == "foot" else -a
    v = np.cross(w, u)
    P = W.geom["points"] @ T_p[:3, :3].T + T_p[:3, 3]
    w0 = float((P @ w).min())                       # the plate's extreme toward the plank
    u0 = float((P @ u).min())                       # its lowest extent downhill
    c = T_p[:3, 3]
    o = c + (w0 - c @ w) * w + (u0 - c @ u) * u
    return o, u, v, w, P


def boxes(T_p, side, width, up):
    """The plank (top face on the plane, from under the lip to ``up`` along u, ``width`` along v, PLANK_T_M thick away from
    the plate) and the lip (LIP_W x LIP_H on the plank's face, its uphill face LIP_GAP_M below the plate's lowest extent)."""
    o, u, v, w, P = frame(T_p, side)
    vc = float((P @ v).mean() - o @ v)               # centred on the plate along v
    Rm = np.column_stack([u, v, w])
    u_lo = -LIP_GAP_M - LIP_W_M
    pc = o + ((u_lo + up) / 2) * u + vc * v - PLANK_T_M / 2 * w
    lc = o + (-LIP_GAP_M - LIP_W_M / 2) * u + vc * v + LIP_H_M / 2 * w
    return [("Plank", pc, (up - u_lo, width, PLANK_T_M), Rm), ("Lip", lc, (LIP_W_M, LIP_L_M, LIP_H_M), Rm)]


def corners(c, size, Rm):
    return np.array([[x, y, z] for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)]) * np.asarray(size) / 2 @ Rm.T + c


def hull_contains(pts2, q):
    """(inside, the signed distance from q to the 2-D convex hull's boundary, + inside)."""
    from scipy.spatial import ConvexHull
    h = ConvexHull(pts2)
    d = -np.max(h.equations[:, :2] @ q + h.equations[:, 2])
    return bool(d >= 0), float(d)


def static_check(T_p, side, width, up):
    """The rest of the plate on the fixture (see the module doc)."""
    W = GL.world()
    o, u, v, w, P = frame(T_p, side)
    g = np.array([0., 0., -1.])
    gw, gu = float(g @ w), float(g @ u)
    L = (P - o)
    pu, pv, pw = L @ u, L @ v, L @ w
    vc = float((P @ v).mean() - o @ v)
    on_plank = (pw <= CONTACT_M) & (pu >= -LIP_GAP_M - LIP_W_M) & (pu <= up) & (np.abs(pv - vc) <= width / 2)
    on_lip = (pu <= float(pu.min()) + LIP_CONTACT_M) & (np.abs(pv - vc) <= LIP_L_M / 2) & (pw <= LIP_H_M)
    com = T_p[:3, :3] @ W.geom["centroid"] + T_p[:3, 3]
    out = {"side": side, "width_m": width, "up_m": up, "inclination_deg": round(math.degrees(math.acos(abs(float(w[2])))), 2),
           "g_along_plank_normal": round(gw, 4), "g_downhill": round(-gu, 4),
           "presses_on_plank": gw < 0, "downhill_to_lip": gu < 0,
           "plank_contacts": int(on_plank.sum()), "lip_contacts": int(on_lip.sum())}
    if gw >= 0:
        out.update(static=False, why=f"gravity pulls the plate off the plank ({gw:+.3f} g along the plank's normal toward the "
                                     "plate: the plank lies above the plate)")
        return out
    s = -float((com - o) @ w) / gw                   # down the vertical from the centroid to the plank plane
    hit = com + s * g - o
    q = np.array([hit @ u, hit @ v - vc])
    pts = np.c_[pu, pv - vc][on_plank | on_lip]
    if len(pts) < 3:
        out.update(static=False, why="fewer than 3 contacts")
        return out
    inside, margin = hull_contains(pts, q)
    up_max = float(pts[:, 0].max())
    out.update(weight_line_uv_mm=np.round(q * 1e3, 1).tolist(), contact_u_range_mm=[round(float(pts[:, 0].min()) * 1e3, 1),
                                                                                    round(up_max * 1e3, 1)],
               contact_v_range_mm=[round(float(pts[:, 1].min()) * 1e3, 1), round(float(pts[:, 1].max()) * 1e3, 1)],
               hull_margin_mm=round(margin * 1e3, 1), static=bool(inside and gu < 0),
               why=None if inside and gu < 0 else ("the weight line misses the support polygon" if not inside else
                                                   "the downhill component does not point at the lip"))
    return out


def placed(W, facing_deg, fx_spec):
    """The plate's spawn pose and the fixture boxes, raised together when a box corner is below the slab: (T_p, specs, raise)."""
    T_p = GL.spawn_pose(W, facing_deg)
    specs = boxes(T_p, fx_spec["side"], fx_spec["width_m"], fx_spec["up_m"])
    low = min(float(corners(c, s, Rm)[:, 2].min()) for _, c, s, Rm in specs)
    lift = max(0., .914 - low)
    if lift > 0:
        T_p = T_p.copy()
        T_p[2, 3] += lift
        specs = [(nm, c + np.array([0., 0., lift]), s, Rm) for nm, c, s, Rm in specs]
    return T_p, specs, lift


def bodies(W, specs):
    return [PP.box_body(W.arm.fcl, nm, c, s, Rm) for nm, c, s, Rm in specs]


def box_record(specs):
    from dishsim_frigidaire import planner as P
    out = []
    for nm, c, s, Rm in specs:
        T = np.eye(4)
        T[:3, :3], T[:3, 3] = Rm, c
        out.append({"id": nm, "center_m": np.round(c, 5).tolist(), "size_m": [round(float(x), 4) for x in s],
                    "axes": np.round(Rm, 5).tolist(), "pose": P.pose_dict(T)})
    return out


def pick_job(args):
    """One facing x (roll, spin) on the fixture: phase 1's pick with the boxes in the scene."""
    fi, fx_spec = args
    W = GL.world()
    arm = W.arm
    T_p, specs, lift = placed(W, FACINGS_DEG[fi], fx_spec)
    bs = bodies(W, specs)
    for b in bs:
        arm.statics[b["id"]] = b
    row = {"facing_deg": FACINGS_DEG[fi], "plate_pose": np.round(T_p, 6).tolist(), "spawn_pose": W.P.pose_dict(T_p),
           "raised_mm": round(lift * 1e3, 1), "fixture": box_record(specs),
           "static": static_check(T_p, fx_spec["side"], fx_spec["width_m"], fx_spec["up_m"]), "cands": {}}
    try:
        dish = arm.checker._body("plate", W.P.pose_dict(T_p), "dish")
        row["plate_gap_mm"] = {b["id"]: round(EP.depth_of(arm.fcl, dish["manager"], b["objects"][0]) * 1e3, 2) for b in bs}
        # signed: > 0 the gap, <= 0 minus the deepest convex-piece penetration (0 = touching, the plank's face at the plate)
        for roll, spin in CANDS:
            T_g = GL.pick_tcp(T_p, spin, roll, W.holds[WP.DEPTH_MM])
            rr = PP.pick(W, arm, dish, T_p, T_g, True)
            M = np.linalg.inv(T_g) @ T_p
            rr.pop("turn_to_goal_deg", None)
            rr.update(roll=roll, spin_deg=spin, T_tcp=np.round(T_g, 6).tolist(), M=np.round(M, 6).tolist(),
                      moment_Nm=dict(zip(("closing_axis", "pads"), PP.moment(W, T_g, M))),
                      turn_to_goal_deg={nm: PP.rot_deg(T_p[:3, :3], (EP.sampler_bowl_tcp(g[0], M, spin, np) @ M)[:3, :3])
                                        for nm, g in W.goals.items()})
            row["cands"][f"{roll}/{spin:+.0f}"] = rr
    finally:
        for b in bs:
            arm.statics.pop(b["id"], None)
        arm.set_dishes({})
    return row


def i4_job(args):
    fi, key, name, cand, fx_spec = args
    W = GL.world()
    arm = W.arm
    t0 = time.monotonic()
    T_goal, slot, F_r, _ = W.goals[name]
    _, specs, _ = placed(W, FACINGS_DEG[fi], fx_spec)
    bs = bodies(W, specs)
    for b in bs:
        arm.statics[b["id"]] = b
    try:
        r = WP.i4_check(W, arm, T_goal, F_r, np.asarray(cand["M"]), np.asarray(cand["q_lift"]), np.asarray(cand["T_lift"]),
                        cand["spin_deg"])
    finally:
        for b in bs:
            arm.statics.pop(b["id"], None)
    if r.get("pass"):
        r["joint_move_deg"] = round(float(np.degrees(np.abs(np.asarray(r["q_start"]) - np.asarray(cand["q_lift"]))).max()), 1)
    r.update(run=True, facing_deg=FACINGS_DEG[fi], cand=key, dish=name, slot=slot, seconds=round(time.monotonic() - t0, 1))
    return r


def sha16(rel):
    return hashlib.sha256((ROOT / rel).read_bytes()).hexdigest()[:16]


def write_md(res, path, cmd):
    L = ["# Plate on a plank and a lip in its goal orientation, Kit-free (phase 1c of plans/2026-11-04-plate-wall-pinch.md)", "",
         f"Command (Mac, {res['seconds']:.0f} s on {res['workers']} workers):", "", "```", cmd, "```", "",
         "The plate is in the s0 goals' orientation (facing 0 is the goal orientation itself; 330 and 30 turn it about world "
         "z through the spot), with the rim circle's centre over the spot. The plank's top face is parallel to the plate plane "
         "and touches the plate's extreme surface on its side, and the lip bar (15 x 15 mm, 160 mm long) on the plank's "
         "downhill edge sits 1 mm below the bottom rim. Foot is the plank in the foot plane under the foot ring. Rim is the "
         "plank in the rim plane under the rim flat on the mouth side. The weight line is the vertical through the plate's "
         "visual centroid. It must meet the plank plane inside the hull of the contacts on the plank and the lip, gravity "
         "must press the plate onto the plank, and the downhill component must point at the lip.", "",
         "Static check at facing 0 (u up the plank from the plate's lowest extent, v along the lean axis; mm):", "",
         "| plank side | width x reach up (mm) | inclination (deg) | g along the plank normal toward the plate | g downhill "
         "| contacts plank / lip | weight line u, v | contact u range | hull margin | static |",
         "|---|---|---|---|---|---|---|---|---|---|"]
    for s in res["static_checks"]:
        L.append(f"| {s['side']} | {s['width_m'] * 1e3:.0f} x {s['up_m'] * 1e3:.0f} | {s['inclination_deg']} | "
                 f"{s['g_along_plank_normal']} | {s['g_downhill']} | {s['plank_contacts']} / {s['lip_contacts']} | "
                 f"{s.get('weight_line_uv_mm', '-')} | {s.get('contact_u_range_mm', '-')} | {s.get('hull_margin_mm', '-')} | "
                 f"{'yes' if s['static'] else 'no: ' + str(s['why'])} |")
    fx = res["fixture_used"]
    L += ["", (f"Fixture used for the sweep: the {fx['side']} side, {fx['width_m'] * 1e3:.0f} mm wide, reaching "
               f"{fx['up_m'] * 1e3:.0f} mm up the plate plane, the longest reach in {', '.join(f'{x * 1e3:.0f}' for x in RIM_UPS_M)} "
               "mm whose facing-0 pick passes and whose rest is static. " + fx["why"]) if fx else
          "No fixture is static, so the sweep did not run.", ""]
    if res["rows"]:
        L += ["| facing (deg) | raised (mm) | plate gap to plank / lip (mm) | pick | blockers | pivot (pads N m) | gripper to plate, "
              "last 60 mm (mm) | plate_05 | plate_03 | plate_04 |", "|---|---|---|---|---|---|---|---|---|---|"]
        for row in res["rows"]:
            for key, c in row["cands"].items():
                bl = "-" if c["ok"] else "; ".join(f"{k} ({v})" for k, v in list((c.get("blockers") or {}).items())[:2]).replace("|", " vs ")
                cells = [GL.i4_cell(res["i4"].get(f"{row['facing_deg']}/{key}/{n}"), c["turn_to_goal_deg"][n]) if c["ok"]
                         else "no pick" for n in PLATES]
                L.append(f"| {row['facing_deg']} | {row['raised_mm']} | {row['plate_gap_mm'].get('Plank')} / "
                         f"{row['plate_gap_mm'].get('Lip')} | {'G2' if c['ok'] else c['gate'] + ' (' + str(c['stage']) + ')'} | "
                         f"{bl or '-'} | {c['moment_Nm']['pads']} | "
                         f"{c['approach_last60']['gripper_plate_min_mm'] if c['ok'] else '-'} | " + " | ".join(cells) + " |")
        L += ["", "An I4 cell reads gate / start (mm) / leg 1 / the plate's turn from the pick to the goal (deg) / the joint "
              "move from the lift to the descent start (deg).", ""]
    ch = res["chosen"]
    L += [(f"Chosen: facing {ch['facing_deg']} deg, roll {ch['roll']}, spin {ch['spin_deg']:+.0f} deg, I4 passes at "
           f"{', '.join(ch['passes'])}, total joint move {ch['joint_move_total_deg']} deg." if ch else
           "Chosen: none (no tuple passes I4 at any goal on a static fixture)."), ""]
    path.write_text("\n".join(L) + "\n")


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--workers", type=int, default=6)
    a = ap.parse_args()
    import multiprocessing as mp
    t0 = time.monotonic()
    OUT.mkdir(parents=True, exist_ok=True)
    shas = {f: sha16(f) for f in GL.IMPORTED + ("code/execution/frigidaire/frigidaire_plate_goal_lean.py",)}
    specs = [BRIEF] + [{"side": "rim", "width_m": wd, "up_m": up} for wd in RIM_WIDTHS_M for up in RIM_UPS_M]
    with mp.get_context("fork").Pool(a.workers) as pool:  # the World in the workers only (Accelerate and fork)
        probe = pool.map(pick_job, [(0, s) for s in specs], chunksize=1)
        checks = [p["static"] for p in probe]
        for s, p in zip(specs, probe):
            c = p["cands"]["180/+0"]
            print(f"[INFO] fixture {s['side']} {s['width_m'] * 1e3:.0f} x {s['up_m'] * 1e3:.0f}: static {p['static']['static']} "
                  f"({p['static'].get('why')}), raised {p['raised_mm']} mm, facing-0 pick {'ok' if c['ok'] else c['stage']} "
                  f"{dict(list((c.get('blockers') or {}).items())[:2])}", flush=True)
        ok = [(s, p) for s, p in zip(specs, probe) if p["static"]["static"] and p["cands"]["180/+0"]["ok"]]
        fx = None
        if ok:
            s, p = max(ok, key=lambda x: (x[0]["width_m"] == RIM_WIDTHS_M[0], x[0]["up_m"]))
            fx = dict(s, why=(f"Its rest: inclination {p['static']['inclination_deg']} deg, the weight line {p['static']['hull_margin_mm']}"
                              f" mm inside the support polygon, {p['static']['g_along_plank_normal']} g along the plank's normal "
                              f"and {p['static']['g_downhill']} g downhill onto the lip; the plate and the fixture rise "
                              f"{p['raised_mm']} mm."))
        rows, i4 = [], {}
        if fx:
            rows = pool.map(pick_job, [(fi, fx) for fi in range(len(FACINGS_DEG))], chunksize=1)
            jobs = [(fi, key, n, rows[fi]["cands"][key], fx) for fi in range(len(FACINGS_DEG)) for key in rows[fi]["cands"]
                    if rows[fi]["cands"][key]["ok"] for n in PLATES]
            print(f"[INFO] fixture {fx['side']} {fx['width_m'] * 1e3:.0f} x {fx['up_m'] * 1e3:.0f}; {len(jobs)} I4 jobs at "
                  f"{time.monotonic() - t0:.0f} s", flush=True)
            for r in pool.imap(i4_job, jobs, chunksize=1):
                i4[f"{r['facing_deg']}/{r['cand']}/{r['dish']}"] = r
                print(f"[INFO] I4 facing {r['facing_deg']} {r['cand']} {r['dish']}: pass {r['pass']} gate {r['gate']} "
                      f"{r['seconds']} s {str(r.get('why') or '')[:140]}", flush=True)
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
    rank.sort(key=lambda x: (-x["n_pass"], x["joint_move_total_deg"]))
    chosen = None
    if rank and rank[0]["n_pass"]:
        x = rank[0]
        row = next(r for r in rows if r["facing_deg"] == x["facing_deg"])
        c = row["cands"][x["key"]]
        chosen = {**{k: x[k] for k in ("facing_deg", "roll", "spin_deg", "n_pass", "passes", "joint_move_total_deg")},
                  "hold": {"depth_mm": WP.DEPTH_MM, "source": "frigidaire_plate_wall_pinch.hold_in_plate(0.020), spun by "
                                                               "EP.sampler_bowl_tcp (frigidaire_plate_goal_lean.pick_tcp)"},
                  "plate_pose": row["spawn_pose"], "raised_mm": row["raised_mm"], "fixture": row["fixture"],
                  "static": row["static"],
                  "pick": {k: c.get(k) for k in ("T_tcp", "M", "q_hover", "q_grasp", "q_lift", "T_lift", "facing", "move_rad",
                                                 "moment_Nm", "approach_last60", "transit", "turn_to_goal_deg")},
                  "i4": {n: i4.get(f"{x['facing_deg']}/{x['key']}/{n}") for n in PLATES}}
    res = {"plan": "plans/2026-11-04-plate-wall-pinch.md", "phase": "1c", "load": str(WP.LOAD.relative_to(ROOT)),
           "counter_xy": list(R.COUNTER_XY), "facings_deg": list(FACINGS_DEG), "cands": [list(c) for c in CANDS],
           "static_checks": checks, "fixture_specs_probed": specs, "fixture_used": fx, "leg1_rrt_s": PP.CARRY_RRT_S,
           "rows": rows, "i4": i4, "ranking": rank, "chosen": chosen,
           "imported_sha256_16": shas, "workers": a.workers, "seconds": round(time.monotonic() - t0, 1)}
    (OUT / "goal_plank.json").write_text(json.dumps(res, indent=1, default=lambda o: o.tolist() if hasattr(o, "tolist")
                                                    else str(o)) + "\n")
    cmd = ("PYTHONPATH=code/src .venv-kitfree/bin/python code/execution/frigidaire/frigidaire_plate_goal_plank.py "
           f"--workers {a.workers}")
    write_md(res, OUT / "kitfree.md", cmd)
    print(f"[RESULT] PASS plate goal plank: fixture {None if not fx else (fx['side'], fx['width_m'], fx['up_m'])}; I4 passes "
          f"{sum(1 for r in i4.values() if r['pass'])}/{len(i4)}; chosen "
          f"{None if not chosen else (chosen['facing_deg'], chosen['roll'], chosen['spin_deg'], chosen['passes'])} in "
          f"{res['seconds']} s -> {(OUT / 'goal_plank.json').relative_to(ROOT)}", flush=True)


if __name__ == "__main__":
    main()
