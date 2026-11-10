#!/usr/bin/env python3
"""Kit-free check of the EPick holds at sample s0's goals and the random counter spawns (plans/2026-11-10-suction-random-spawn.md).

    code/util/run_py.sh code/sampler/robot/scripts/frigidaire_suction_check.py --sample data/results/robot/suction_s0/s0.json \\
        --mount data/results/robot/mount/posture_v2.json --out data/results/robot/suction_s0

Per rack chain (the dag order; the dish's rack out at its slide limit, the other pushed in, the basket out, the counter's front
edge at y -0.17) and per dish, with the chain's earlier dishes at their goals, three holds of the 40 mm cup on the FCL arm model
(data/results/robot/arm_model.json: the UR5e links and the EPick body; the real cup added as a 40 x 24 mm cylinder):
  base    the cup on the outer bottom centre, the tool along the dish axis (the dish mouth down on the counter);
  side    the cup on the outer side wall SIDE_H[kind] above the base, the tool along the inward wall normal, 12 azimuths
          (bowls and cups; a plate has no side wall);
  inside  the cup on the inner floor centre, the tool down the axis from the mouth (the dish mouth up).
A hold passes at the goal when the tool keeps out of its own dish, the release (the goal raised 3 mm) has an elbow-up IK solution
with the arm and the tool 2 mm clear, an approach start 100 (else 150, 50) mm straight above it is clear under the carry rule (the
arm and the tool 5 mm, the held dish 5 mm), the straight approach line is clear (2 mm; the held dish by the checker's 1 mm pair
rule) and the 50 mm retreat back along the tool axis is clear with the dish at its goal. Run D holds by the base, run U by the
side wall, else by the inside. Spawns: per dish (x, y, yaw) uniform on the counter with the dish inside its edges (seed 0),
redrawn until every run with a hold can pick there (the pre-pick 100 mm back along the tool axis under the carry rule, the line
to the pick, the 100 mm lift and the lifted pose under the carry rule), the draws counted. Writes <out>/check.json.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys
import time

import numpy as np

ROOT = Path(__file__).resolve().parents[4]
sys.path[:0] = [str(ROOT / "code"), str(ROOT / "code/util")]
from sampler.robot import epick                     # noqa: E402  (Kit-free)
from sampler.robot.rig import Rig, T_of             # noqa: E402

ARM_MODEL = ROOT / "data/results/robot/arm_model.json"
DISH_USD = ROOT / "data/assets/hotec_wheatstraw/usd"
COUNTER_FRONT_Y, COUNTER_TOP_M = -.17, .914        # round 3's pulled counter (frigidaire_robot_replay.pull_counter)
SPAWN_LIFT_M = .002                                 # dishes spawn 2 mm above the counter
SIDE_H = {"bowl": .060, "cup": .065}                # the side hold's contact height above the base: the cup's lowest height
                                                    # whose pre-pick keeps the body 5 mm off the counter mouth up
N_AZ = 12
RELEASE_UP_M, APPROACH_M, RETREAT_M = .003, (.10, .15, .05), .05     # APPROACH_M per approach direction (approach_dirs)
PRE_PICK_M, LIFT_M = .10, .10
LINE_MARGIN_M, TRANSIT_MARGIN_M, CARRY_CLEAR_M = .002, .005, .005
HOME_TCP = (.45, -.30, 1.15)                         # the tool straight down above the counter's front edge, in reach
MAX_DRAWS, N_SHARE = 500, 200                        # spawn draws per dish; every dish evaluates at least N_SHARE for the share
STAGES = ("own dish", "release", "approach start", "approach line", "retreat")


class KinRig(Rig):
    """The rig's IK and line math without a simulator (the TCP at the cup tip)."""

    def __init__(self, T_wb):
        self.T_wb, self.T_bw, self.T_w3_tcp = T_wb, np.linalg.inv(T_wb), epick.T_W3_TIP


def bench():
    """frigidaire_bench with the counter's front edge at COUNTER_FRONT_Y (the slab's back edge and top kept)."""
    from arrangement_optimization import frigidaire_bench as B
    (cx, cy, cz), (sx, sy, sz) = B.COUNTER["center_m"], B.COUNTER["size_m"]
    back = cy + sy / 2
    B.COUNTER = {**B.COUNTER, "size_m": (sx, back - COUNTER_FRONT_Y, sz), "center_m": (cx, (back + COUNTER_FRONT_Y) / 2, cz)}
    return B


def frames_for(B, rack):
    """Component frames: the baseline, ``rack`` out at its slide limit, the other rack pushed in (the basket rides on the lower)."""
    from asset.twin import BODY_POSITIONS
    from sampler.initial_state_candidates import COMPONENT_NAMES
    from sampler.random_pose_runtime import EXTENSION
    base = json.loads((B.OUT / "baseline" / "result.json").read_text())
    base = base.get("snapshot", base)["poses"]
    frames = {name: base[name] for name in COMPONENT_NAMES}
    other = "UpperRack" if rack == "LowerRack" else "LowerRack"
    for r, y in ((other, BODY_POSITIONS[other][1]), (rack, BODY_POSITIONS[rack][1] + EXTENSION[rack])):
        dy = y - base[r]["position_m"][1]
        for name in (r, "SilverwareBasket") if r == "LowerRack" else (r,):
            p = base[name]["position_m"]
            frames[name] = {"position_m": [p[0], p[1] + dy, p[2]], "quaternion_xyzw": list(base[name]["quaternion_xyzw"])}
    return frames


def mount_T(path):
    m = json.loads(Path(path).read_text())["best"]
    c, s = math.cos(m["yaw_rad"]), math.sin(m["yaw_rad"])
    T = np.eye(4)
    T[:3, :3] = [[c, -s, 0], [s, c, 0], [0, 0, 1]]
    T[:3, 3] = m["base"]
    return T, m


def pose_of(T):
    from scipy.spatial.transform import Rotation
    return {"position_m": [float(v) for v in T[:3, 3]], "quaternion_xyzw": Rotation.from_matrix(T[:3, :3]).as_quat().tolist()}


def T_pose(p):
    return T_of(p["position_m"], p["quaternion_xyzw"])


def up(T, h):
    T = np.array(T, dtype=float)
    T[2, 3] += h
    return T


def along(T, d):
    """``T`` moved by ``d`` along its own z (the tool axis)."""
    T = np.array(T, dtype=float)
    T[:3, 3] += d * T[:3, 2]
    return T


def rot(axis, a):
    c, s = math.cos(a), math.sin(a)
    i, j = {"x": (1, 2), "z": (0, 1)}[axis]
    R = np.eye(4)
    R[i, i], R[i, j], R[j, i], R[j, j] = c, -s, s, c
    return R


def hold_T(p, z):
    """The TCP in the dish frame: at ``p``, its z (the cup axis, toward the dish) along ``z``; the spin is free (wrist 3 only)."""
    z = np.asarray(z, dtype=float) / np.linalg.norm(z)
    x = np.cross([0., 0., 1.] if abs(z[2]) < .9 else [0., 1., 0.], z)
    x /= np.linalg.norm(x)
    T = np.eye(4)
    T[:3, :3] = np.c_[x, np.cross(z, x), z]
    T[:3, 3] = p
    return T


def dish_geometry(kind):
    """{height, radius, holds: {name: [(label, H)]}} from the dish's collision vertices (patches) and visual points (wall)."""
    from pxr import Usd, UsdGeom, UsdPhysics
    from asset.loading import visual_points
    st = Usd.Stage.Open(str(DISH_USD / f"{kind}.usda"))
    root, cache = st.GetDefaultPrim(), UsdGeom.XformCache()
    inv = np.linalg.inv(np.asarray(cache.GetLocalToWorldTransform(root)).T)
    P = np.vstack([((inv @ np.asarray(cache.GetLocalToWorldTransform(p)).T)
                    @ np.c_[np.asarray(UsdGeom.Mesh(p).GetPointsAttr().Get(), dtype=float), np.ones(len(UsdGeom.Mesh(p).GetPointsAttr().Get()))].T).T[:, :3]
                   for p in Usd.PrimRange(root) if p.HasAPI(UsdPhysics.CollisionAPI) and p.IsA(UsdGeom.Mesh)])
    r, z = np.hypot(P[:, 0], P[:, 1]), P[:, 2]
    near = z[r <= epick.CUP_R_M]
    holds = {"base": [("base", hold_T((0., 0., near.min()), (0., 0., 1.)))],
             "inside": [("inside", hold_T((0., 0., near.max()), (0., 0., -1.)))], "side": []}
    if kind != "plate":                               # a plate has no side wall
        V = np.asarray(visual_points(DISH_USD / f"{kind}.usda"))
        rv, zv = np.hypot(V[:, 0], V[:, 1]), V[:, 2]

        def r_out(h):
            return float(rv[np.abs(zv - h) <= .0015].max())
        h = SIDE_H[kind]
        ro, s = r_out(h), (r_out(h + .005) - r_out(h - .005)) / .01
        nr, nz = 1. / math.hypot(1., s), -s / math.hypot(1., s)      # the outward wall normal in (radial, z)
        for k in range(N_AZ):
            a = 2 * math.pi * k / N_AZ
            holds["side"].append((f"side@{30 * k}", hold_T((ro * math.cos(a), ro * math.sin(a), h),
                                                           (-nr * math.cos(a), -nr * math.sin(a), -nz))))
    return {"height": float(z.max()), "radius": float(r.max()), "holds": holds}


def cup_points(n=32):
    """The real 40 mm cup as points in wrist_3's frame, ending 1 mm short of the tip (the hold's patch is not a collision)."""
    a = np.linspace(0., 2 * np.pi, n, endpoint=False)
    ring = np.c_[epick.CUP_R_M * np.cos(a), epick.CUP_R_M * np.sin(a)]
    return np.vstack([np.c_[ring, np.full(n, z)] for z in (epick.BODY_LEN_M, epick.TIP_M - .001)])


class World:
    """The FCL arm world of one rack chain: the appliance at the chain's rack state with the basket out, the counter, the
    pedestal and the floor; ``kr`` the analytic IK."""

    def __init__(self, B, rack, mount, frames=None, model=ARM_MODEL):
        from motion_planning import planner as P
        from sampler.robot import collide as C
        self.B, self.C = B, C
        self.T_wb, m = mount_T(mount)
        self.checker = B.bench_checker()
        self.set_frames(frames or frames_for(B, rack))
        hulls, X = C.load_model(model)
        hulls["gripper"] = list(hulls["gripper"]) + [C._hull(cup_points())]
        self.arm = C.ArmWorld(self.checker, P.slab_body(self.checker, B.COUNTER), hulls, X, self.T_wb, epick.T_W3_TIP,
                              held_margin=CARRY_CLEAR_M)
        bx, by, bz = m["base"]
        self.arm.add_static_box("Pedestal", (bx, by, bz / 2 - .01), (.18, .18, bz - .02))
        self.arm.add_static_box("Floor", (0., 0., -.05), (6., 6., .1))
        self.kr = KinRig(self.T_wb)

    def set_frames(self, frames):
        self.checker.update_components(frames)
        self.checker.components.pop("SilverwareBasket", None)      # basket out (the robot rounds' rule)

    def body(self, kind, T, oid):
        return self.checker._body(kind, pose_of(T), oid)

    def hits(self, q, mode, held=None, may=()):
        """[] when clear. mode "line": the arm and the tool 2 mm, a held dish by the 1 mm pair rule; "transit": 5 mm, a held
        dish 5 mm from the scene. ``may``: dishes only the tool may touch."""
        a = self.arm
        a.margin, a.held_clear_m = (LINE_MARGIN_M, None) if mode == "line" else (TRANSIT_MARGIN_M, CARRY_CLEAR_M)
        return a.collisions(q, held=held, gripper_may_touch=may)

    def own_dish(self, kind, H):
        """The tool's hulls at hold ``H`` against its own dish: None when clear, else the touching hull's name."""
        fcl, body = self.arm.fcl, self.body(kind, np.eye(4), "self")
        W3 = H @ np.linalg.inv(epick.T_W3_TIP)
        for i, (g, _) in enumerate(self.arm.hulls["gripper"]):
            o = fcl.CollisionObject(g, fcl.Transform(W3[:3, :3], W3[:3, 3]))
            if any(fcl.collide(o, other, fcl.CollisionRequest(), fcl.CollisionResult()) for other in body["objects"]):
                return "cup" if i == len(self.arm.hulls["gripper"]) - 1 else "body"
        return None


def line_hit(W, qs, mode, held=None, may=()):
    """(waypoint index, hits) of the first waypoint that is not clear, else None."""
    for i, q in enumerate(qs):
        h = W.hits(q, mode, held, may)
        if h:
            return i, h
    return None


def seat(W, kind, H):
    """``H`` backed off along the tool axis in 0.5 mm steps, up to 5 mm, until the tool (the body and the cup) clears its own
    dish (the gripper's ray reaches 10 mm): (H, standoff m), else (None, the part still inside)."""
    for k in range(11):
        own = W.own_dish(kind, along(H, -.0005 * k))
        if own is None:
            return along(H, -.0005 * k), .0005 * k
    return None, own


def approach_dirs(T_goal):
    """The approach's motion directions: straight down, then down the slope of the dish's rim plane (a leaning plate slides
    in its own plane between its tines)."""
    n = T_goal[:3, 2]
    d = -(np.array([0., 0., 1.]) - n[2] * n)
    return [("vertical", np.array([0., 0., -1.]))] + ([("in-plane", d / np.linalg.norm(d))] if np.linalg.norm(d) > .2 else [])


def goal_check(W, kind, oid, T_goal, H, seed):
    """One hold candidate at the goal -> {"pass", "stage", "cause", "H" (seated), "standoff_mm", q_rel, q_app, approach_m}."""
    Hs, so = seat(W, kind, H)
    if Hs is None:
        return {"pass": False, "stage": "own dish", "cause": f"own dish: the tool's {so} inside the dish even 5 mm off",
                "hits": [], "H": np.round(H, 6).tolist()}
    H, worst = Hs, (-1, None, ())
    seated = {"H": np.round(H, 6).tolist(), "standoff_mm": round(so * 1e3, 1)}
    held = (kind, np.linalg.inv(H), oid)
    T_rel = up(T_goal, RELEASE_UP_M) @ H
    sols = W.kr.ik_all(T_rel, seed)
    if not sols:
        return {"pass": False, "stage": "release", "cause": "release: no elbow-up IK solution", "hits": [], **seated}

    def note(stage, cause, hits=()):
        nonlocal worst
        if STAGES.index(stage) > worst[0]:
            worst = (STAGES.index(stage), cause, hits)
    for q_rel in sols:
        h_ = W.hits(q_rel, "line")
        if h_:
            note("release", f"release: {h_[0][0]} against {h_[0][1]}", h_)
            continue
        for (way, v), h in ((w, h) for w in approach_dirs(T_goal) for h in APPROACH_M):
            T_app = np.array(T_rel)
            T_app[:3, 3] -= h * v
            tag = f"{way} approach from {h * 1e3:.0f} mm"
            qa = W.kr.ik_all(T_app, q_rel)
            if not qa:
                note("approach start", f"{tag}: no IK at its start")
                continue
            h_ = W.hits(qa[0], "transit", held)
            if h_:
                note("approach start", f"{tag}: {h_[0][0]} against {h_[0][1]} at its start", h_)
                continue
            qs = W.kr.line_q(T_app, T_rel, 40, qa[0])
            if qs is None:
                note("approach line", f"{tag}: IK branch jump")
                continue
            bad = line_hit(W, qs, "line", held)
            if bad:
                note("approach line", f"{tag}: {bad[1][0][0]} against {bad[1][0][1]} {(1 - bad[0] / 39) * h * 1e3:.0f} mm "
                                      f"before the release", bad[1])
                continue
            W.arm.dishes[oid] = W.body(kind, T_goal, oid)          # the dish at its goal; the tool leaves it
            try:
                qr = W.kr.line_q(T_rel, along(T_rel, -RETREAT_M), 20, qs[-1])
                bad = None if qr is None else line_hit(W, qr, "line", may=(oid,))
            finally:
                W.arm.dishes.pop(oid, None)
            if qr is None or bad:
                note("retreat", "retreat: IK branch jump" if qr is None else f"retreat: {bad[1][0][0]} against {bad[1][0][1]}",
                     () if qr is None else bad[1])
                continue
            return {"pass": True, "stage": "retreat", "cause": f"clear: release, {tag}, retreat", "q_rel": qs[-1].tolist(),
                    "q_app": qa[0].tolist(), "approach": way, "approach_m": h, "approach_dir": v.tolist(),
                    "q_retreat": qr[-1].tolist(), "hits": [], **seated}
    return {"pass": False, "stage": STAGES[worst[0]], "cause": worst[1], "hits": [list(x) for x in worst[2]], **seated}


def spawn_T(x, y, yaw, mouth_down, height):
    """The dish on the counter: mouth up on its base, or mouth down on its rim (base-centred origin on top), 2 mm up."""
    T = rot("z", yaw) @ (rot("x", math.pi) if mouth_down else np.eye(4))
    T[:3, 3] = (x, y, COUNTER_TOP_M + (height if mouth_down else 0.) + SPAWN_LIFT_M)
    return T


def pick_check(W, kind, oid, T_dish, H, seed):
    """The pick at a spawn -> {q_pre, q_pick, q_lift} or None."""
    held = (kind, np.linalg.inv(H), oid)
    T_pick = T_dish @ H
    T_pre, T_lift = along(T_pick, -PRE_PICK_M), up(T_pick, LIFT_M)
    body = W.body(kind, T_dish, oid)
    try:
        for q_pick in W.kr.ik_all(T_pick, seed):
            W.arm.dishes[oid] = body
            if W.hits(q_pick, "line", may=(oid,)):
                continue
            qp = W.kr.ik_all(T_pre, q_pick)
            if not qp or W.hits(qp[0], "transit"):
                continue
            qs = W.kr.line_q(T_pre, T_pick, 30, qp[0])
            if qs is None or line_hit(W, qs, "line", may=(oid,)):
                continue
            W.arm.dishes.pop(oid, None)                           # held from here on
            ql = W.kr.line_q(T_pick, T_lift, 30, qs[-1])
            if ql is None or line_hit(W, ql[1:], "line", held) or W.hits(ql[-1], "transit", held):
                continue
            return {"q_pre": qp[0].tolist(), "q_pick": qs[-1].tolist(), "q_lift": ql[-1].tolist()}
        return None
    finally:
        W.arm.dishes.pop(oid, None)


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--sample", type=Path, required=True)
    ap.add_argument("--mount", type=Path, required=True)
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--arm-model", type=Path, default=ARM_MODEL)
    args = ap.parse_args()
    t0 = time.monotonic()
    s = json.loads(args.sample.read_text())
    goal, order = s["goal"], s["dag"]["order"]
    kind = {oid: oid.rsplit("_", 1)[0] for oid in order}
    rack = {oid: goal["slots"][oid]["rack"] for oid in order}
    chains = {"upper": [o for o in order if rack[o] == "UpperRack"], "lower": [o for o in order if rack[o] == "LowerRack"]}
    B = bench()
    geo = {k: dish_geometry(k) for k in sorted(set(kind.values()))}
    out = {"sample": str(args.sample), "mount": str(args.mount), "arm_model": str(args.arm_model), "seed": args.seed,
           "counter": {"front_y": COUNTER_FRONT_Y, "slab": {k: list(v) if isinstance(v, tuple) else v for k, v in B.COUNTER.items()}},
           "rules": {"release_up_m": RELEASE_UP_M, "approach_m": APPROACH_M, "retreat_m": RETREAT_M, "pre_pick_m": PRE_PICK_M,
                     "lift_m": LIFT_M, "line_margin_m": LINE_MARGIN_M, "transit_margin_m": TRANSIT_MARGIN_M,
                     "carry_clear_m": CARRY_CLEAR_M, "side_h_m": SIDE_H, "n_azimuths": N_AZ, "max_draws": MAX_DRAWS,
                     "spawn_lift_m": SPAWN_LIFT_M, "basket": False},
           "geometry": {k: {"height": g["height"], "radius": g["radius"]} for k, g in geo.items()},
           "chains": chains, "dishes": {}}
    worlds = {c: World(B, "UpperRack" if c == "upper" else "LowerRack", args.mount, model=args.arm_model) for c in chains}
    T_home = np.diag([1., -1., -1., 1.])                 # the cup axis straight down
    T_home[:3, 3] = HOME_TCP
    q_home = worlds["upper"].kr.ik(T_home, np.array([0., -1.712, 1.712, -1.571, -1.571, 0.]))
    if q_home is None or any(w.hits(q_home, "transit") for w in worlds.values()):
        raise SystemExit(f"[RESULT] FAIL suction check: the home pose {HOME_TCP} has no clear IK")
    out["q_home"] = q_home.tolist()
    share = {run: {k: [0, 0] for k in geo} for run in ("D", "U")}     # [draws the run picks from, draws] per kind
    for chain, ids in chains.items():
        W = worlds[chain]
        for i, oid in enumerate(ids):
            k, T_goal = kind[oid], T_pose(goal["poses"][oid])
            W.arm.set_dishes({e: W.body(kind[e], T_pose(goal["poses"][e]), e) for e in ids[:i]})
            row = {"kind": k, "rack": rack[oid], "chain": chain, "index": i, "slot": goal["slots"][oid].get("slot"),
                   "goal": goal["poses"][oid], "holds": {}}
            for name, cands in geo[k]["holds"].items():
                if not cands:
                    row["holds"][name] = {"verdict": "fail", "cause": "a plate has no side wall", "passing": [], "blockers": [],
                                          "candidates": []}
                    continue
                res = [{"label": lab, **goal_check(W, k, oid, T_goal, H, q_home)} for lab, H in cands]
                ok = [r for r in res if r["pass"]]
                best = ok[0] if ok else max(res, key=lambda r: STAGES.index(r["stage"]))
                row["holds"][name] = {"verdict": "pass" if ok else "fail", "cause": best["cause"] + (f" ({best['label']})" if name == "side" else ""),
                                      "passing": [r["label"] for r in ok],
                                      "blockers": sorted({h_[1] for r in ([best] if not ok else []) for h_ in r["hits"] if h_[1] in ids[:i]}),
                                      "candidates": res}
            hb, hs, hi = (row["holds"][n] for n in ("base", "side", "inside"))
            row["runs"] = {"D": {"hold": "base" if hb["verdict"] == "pass" else None, "cause": hb["cause"]},
                           "U": {"hold": (hs["passing"][0] if hs["passing"] else "inside" if hi["verdict"] == "pass" else None),
                                 "cause": hs["cause"] if hs["passing"] else hi["cause"] if hi["verdict"] == "pass"
                                 else f"side: {hs['cause']}; inside: {hi['cause']}"}}
            # spawns: the same (x, y, yaw) for both runs, redrawn until every run with a hold picks there
            runs = {"D": [c for c in hb["candidates"] if c["pass"]],
                    "U": [c for c in hs["candidates"] if c["pass"]] + [c for c in hi["candidates"] if c["pass"]]}
            R = geo[k]["radius"] + .005
            rng = np.random.default_rng([args.seed, order.index(oid)])
            first, spawn, n, hits = {}, None, 0, {"D": 0, "U": 0}
            while any(runs.values()) and n < MAX_DRAWS and (spawn is None or n < N_SHARE):
                n += 1
                x, y, yaw = (rng.uniform(-.9 + R, .9 - R), rng.uniform(COUNTER_FRONT_Y + R, .30 - R), rng.uniform(0., 2 * math.pi))
                picks = {}
                for run, cands in runs.items():
                    T_d = spawn_T(x, y, yaw, run == "D", geo[k]["height"])
                    for c in cands:
                        p = pick_check(W, k, oid, T_d, np.asarray(c["H"]), q_home)
                        if p:
                            picks[run] = {"hold": c["label"], "spawn_pose": pose_of(T_d), **p}
                            hits[run] += 1
                            first.setdefault(run, (n, (x, y, yaw), picks[run]))
                            break
                if spawn is None and all(run in picks for run, c in runs.items() if c):
                    spawn = {"xy_yaw": [x, y, yaw], "draws": n, "picks": picks}
            if spawn is None and "D" in first:                   # no draw served both runs: run D's first pick, U without one
                n0, xyz, p = first["D"]
                spawn = {"xy_yaw": list(xyz), "draws": n0, "picks": {"D": p}, "note": f"no draw in {n} served both runs"}
            row["spawn"] = spawn
            row["share"] = {run: {"picks": hits[run], "draws": n} for run, c in runs.items() if c}
            for run, c in runs.items():
                if c:
                    share[run][k][0] += hits[run]
                    share[run][k][1] += n
            for run in ("D", "U"):
                if row["runs"][run]["hold"] and not (spawn and run in spawn["picks"]):
                    row["runs"][run].update(hold=None, cause=f"no counter pick in {n} draws")
                elif row["runs"][run]["hold"]:
                    row["runs"][run]["hold"] = spawn["picks"][run]["hold"]
            out["dishes"][oid] = row
            print(f"[INFO] {chain} {i} {oid}: base {hb['verdict']} ({hb['cause']}); side {hs['verdict']} ({hs['cause']}); inside "
                  f"{hi['verdict']} ({hi['cause']}); D {row['runs']['D']['hold']} U {row['runs']['U']['hold']}; spawn "
                  f"{None if not spawn else [round(v, 3) for v in spawn['xy_yaw']]} at draw {None if not spawn else spawn['draws']}, picks "
                  f"{ {r: v['picks'] for r, v in row['share'].items()} } of {n} draws ({time.monotonic() - t0:.0f} s)", flush=True)
    out["flip"] = [o for o in order if not out["dishes"][o]["runs"]["U"]["hold"]]
    out["share"] = {run: {k: {"picks": a, "draws": b, "share": round(a / b, 4) if b else None} for k, (a, b) in v.items()}
                    for run, v in share.items()}
    out["wall_s"] = round(time.monotonic() - t0, 1)
    args.out.mkdir(parents=True, exist_ok=True)
    (args.out / "check.json").write_text(json.dumps(out, indent=1) + "\n")
    nD = sum(1 for o in order if out["dishes"][o]["runs"]["D"]["hold"])
    nU = sum(1 for o in order if out["dishes"][o]["runs"]["U"]["hold"])
    print(f"[RESULT] PASS suction check: run D {nD}/15 with a hold and a spawn, run U {nU}/15, flip {out['flip']}, share "
          f"{ {r: {k: x['share'] for k, x in v.items()} for r, v in out['share'].items()} }, {out['wall_s']} s", flush=True)


if __name__ == "__main__":
    main()
