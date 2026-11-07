#!/usr/bin/env python3
"""Kit-free: a carried bowl's executed path against its planned carry leg, from the Isaac trial's own logs.

    PYTHONPATH=code/src .venv-kitfree/bin/python code/execution/frigidaire/frigidaire_carry_deviation.py

plans/2026-11-04-bowl-carry-clearance.md, phase 1. Reads the s0 load dir (never writes there) and writes only under --out.
Per run of --runs (runs/<run>/trials/trial.jsonl: the 20 Hz samples of q, q_cmd and the dish poses, the PhysX contact
events; trial_physics.jsonl: the scene's body poses; traj_*.npz: the logged wrist 3 poses, a check of the FK; the placing
record: the pick's hold M and the descent start):
  legs     with the dish held by the pads alone: the lift (the 0.15 m TCP line) and the carry (transit_to into the descent
           start, one RRT-Connect leg in both s0 bowl runs); the planned leg = its commanded waypoints (q_cmd), each straight
           joint edge sampled at STEP_RAD, its TCP and dish paths by FK through the arm model (collide.ArmWorld on
           data/results/robot/arm_model.json, the run's mount, the scene at the leg's start);
  samples  the lateral deviation (the executed TCP and dish origin to the nearest point of the planned path, searched in a
           window behind the commanded waypoint), the tracking error (TCP at q against TCP at q_cmd), the joint lag
           max |q - q_cmd|, the in-hand offset (the dish against TCP @ M, the episode's carry watch, with its rotation and
           the largest displacement over the dish's visual points) and the executed dish clearance to the scene (FCL signed
           distance, negative = overlap);
  events   the contact tick (the held dish's first scene contact in the carry) and the closest point (the smallest executed
           dish clearance in the carry): those numbers plus the plan's clearance at the matching planned point (the held dish
           and the arm against the scene, and ArmWorld.collisions with the dish held, the planner's own check);
  knob     the plan's rule on the dropped bowl's contact tick.
Writes <out>/carry_deviation.json and <out>/kitfree.md.
"""
from __future__ import annotations

import argparse
import importlib.util
import json
import math
from pathlib import Path
import shlex
import sys
import time

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
sys.path[:0] = [str(ROOT / "code/src"), str(ROOT / "code/frigidaire/src")]

LOAD = "data/results/robot/samples_v1/frigidaire_fdpc4221as__hotec_wheatstraw__plate5-bowl5-cup5_a3__s0"
RUNS = "s0_bowl_05_b,s0_bowl_01_b"
ARM_MODEL = "data/results/robot/arm_model.json"
LIFT, CARRY = "lift", "carry to the descent start"
STEP_RAD = .02                     # the planned leg sampled at 0.02 rad (the RRT path's own waypoint spacing)
FINE_RAD = .002                    # and re-scanned at 0.002 rad next to every planned sample with the dish under CLEAR_MM
CUTOFF_M = .10                     # a target farther than this (AABB gap) is not resolved
BACK, AHEAD = 60, 10               # the nearest-point window, in planned samples behind / ahead of the commanded waypoint
HAND = ("pad", "finger", "palm")   # the hand's contact categories (the hold itself)
DEV_SLOW_MM, CLEAR_MM, IN_HAND_MM = 15., 20., 2.     # the plan's rule (Decisions)
TICK_HZ = 120.


# ----------------------------------------------------------------------------------------------- small helpers
def load_bench(front_y):
    from dishsim_frigidaire.robot import grasp as G
    spec = importlib.util.spec_from_file_location("frigidaire_bench", ROOT / "code/planner/frigidaire/frigidaire_bench.py")
    B = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(B)
    G.pull_counter(B, front_y)                     # the run's counter edge (the records' counter_front_y)
    return B


def mount_T(m):
    """The robot base frame from the record's mount [x, y, z, yaw]."""
    c, s = math.cos(m[3]), math.sin(m[3])
    T = np.eye(4)
    T[:3, :3] = [[c, -s, 0.], [s, c, 0.], [0., 0., 1.]]
    T[:3, 3] = m[:3]
    return T


def T7(v):
    from dishsim_frigidaire.robot.rig import T_of
    return T_of(v[:3], v[3:7])


def pose_dict(T):
    from scipy.spatial.transform import Rotation
    return {"position_m": [float(x) for x in T[:3, 3]], "quaternion_xyzw": Rotation.from_matrix(T[:3, :3]).as_quat().tolist()}


def norm_pose(p):
    q = np.asarray(p["quaternion_xyzw"], float)
    return {"position_m": [float(x) for x in p["position_m"]], "quaternion_xyzw": (q / np.linalg.norm(q)).tolist()}


def orthonormal(T):
    T = np.array(T, dtype=float)
    U, _, Vt = np.linalg.svd(T[:3, :3])
    T[:3, :3] = U @ Vt
    return T


def rot_deg(Ra, Rb):
    return float(np.degrees(np.arccos(np.clip((np.trace(Ra.T @ Rb) - 1.) / 2., -1., 1.))))


def r2(x, n=2):
    return None if x is None else round(float(x), n)


def distinct(qs):
    out = [np.asarray(qs[0], float)]
    for q in qs[1:]:
        q = np.asarray(q, float)
        if np.abs(q - out[-1]).max() > 1e-6:
            out.append(q)
    return np.array(out)


def densify(W, step=STEP_RAD):
    """Each straight joint edge of W sampled at <= step; also the fractional waypoint index of every sample."""
    out, src = [W[0]], [0.]
    for i, (a, b) in enumerate(zip(W[:-1], W[1:])):
        n = max(1, int(math.ceil(float(np.abs(b - a).max()) / step - 1e-9)))
        for s in np.linspace(0., 1., n + 1)[1:]:
            out.append(a + (b - a) * s)
            src.append(i + s)
    return np.array(out), np.array(src)


def corners(W, cos_max=.9999):
    d = np.diff(W, axis=0)
    n = d / np.linalg.norm(d, axis=1, keepdims=True)
    return [int(i) + 1 for i in np.nonzero((n[1:] * n[:-1]).sum(1) < cos_max)[0]]


def nearest_on(P, p, lo, hi):
    """(distance, segment i, fraction t) of point p to the polyline P[lo..hi]."""
    lo, hi = max(0, int(lo)), min(len(P) - 1, int(hi))
    if hi <= lo:
        return float(np.linalg.norm(P[lo] - p)), lo, 0.
    a, ab = P[lo:hi], P[lo + 1:hi + 1] - P[lo:hi]
    L2 = (ab * ab).sum(1)
    t = np.clip(((p - a) * ab).sum(1) / np.where(L2 > 0, L2, 1.), 0., 1.)
    d = np.linalg.norm(a + ab * t[:, None] - p, axis=1)
    k = int(np.argmin(d))
    return float(d[k]), lo + k, float(t[k])


def gap(A, Bb):
    (alo, ahi), (blo, bhi) = A, Bb
    return float(np.linalg.norm(np.maximum(0., np.maximum(np.asarray(blo) - ahi, np.asarray(alo) - bhi))))


# ----------------------------------------------------------------------------------------------- the FCL world
class World:
    """The episode's planning world rebuilt Kit-free: collide.ArmWorld with the run's mount, pedestal and floor, the scene
    (the five components and the other dishes) at their logged poses, the held dish (kind, M, oid)."""

    def __init__(self, B, checker, model, rec, phys_row, oid, kind, M):
        from dishsim_frigidaire import planner as P
        from dishsim_frigidaire.initial_state_candidates import COMPONENT_NAMES
        from dishsim_frigidaire.robot import collide as C, rig as RG, ur5e
        checker.update_components({n: norm_pose(phys_row["poses"][n]) for n in COMPONENT_NAMES})
        hulls, X = model
        self.arm = C.ArmWorld(checker, P.slab_body(checker, B.COUNTER), hulls, X, mount_T(rec["mount"]),
                              RG.T_of(ur5e.T_WRIST3_TCP_POS, ur5e.T_WRIST3_TCP_QUAT_XYZW), held_margin=C.HELD_ARM_MARGIN_M)
        bx, by, bz = rec["mount"][:3]
        self.arm.add_static_box("Pedestal", (bx, by, bz / 2 - .01), (.18, .18, bz - .02))   # as the episode
        self.arm.add_static_box("Floor", (0., 0., -.05), (6., 6., .1))
        others = {d: norm_pose(p) for d, p in phys_row["poses"].items()
                  if d not in COMPONENT_NAMES and d != oid and abs(p["position_m"][0]) < 2.}   # the parked line (x >= 3 m) left out
        self.arm.set_dishes({d: checker._body(d.split("_")[0], p, d) for d, p in others.items()})
        self.others = sorted(others)
        self.fcl, self.checker, self.oid, self.kind, self.M = checker.fcl, checker, oid, kind, M
        self.targets = (list(checker.components.items()) + [("Counter", self.arm.slab)] + list(self.arm.statics.items())
                        + [(d, b) for d, b in self.arm.dishes.items()])

    def tcp(self, q):
        return self.arm.link_poses(q)["wrist_3_link"] @ self.arm.T_w3_tcp

    def _sd(self, a, b_mgr, single=False):
        data = self.fcl.DistanceData(request=self.fcl.DistanceRequest(enable_nearest_points=True, enable_signed_distance=True))
        if single:
            b_mgr.distance(a, data, self.fcl.defaultDistanceCallback)
        else:
            a.distance(b_mgr, data, self.fcl.defaultDistanceCallback)
        return float(data.result.min_distance), [np.asarray(p, float) for p in data.result.nearest_points]

    def dish_clear(self, T_d, parts=False):
        """The dish at T_d against the scene: the smallest signed distance (m), its target, the per-target distances (mm),
        the bench checker's pair rule (1 mm penetration allowed: the planner's held-dish test)."""
        body = self.checker._body(self.kind, pose_dict(T_d), "held")
        best, per, ok, pen = (math.inf, None, None), {}, True, 0.
        for name, b in self.targets:
            if gap(body["bounds"], b["bounds"]) > CUTOFF_M:
                continue
            d, pts = self._sd(body["manager"], b["manager"])
            per[name] = round(d * 1e3, 2)
            if d < best[0]:
                best = (d, name, pts)
            r = self.checker.pair(body, b)
            ok, pen = ok and r["valid"], max(pen, r["max_penetration_m"])
        out = {"mm": None if best[1] is None else round(best[0] * 1e3, 2), "target": best[1], "per_target_mm": per,
               "checker_pair_ok": ok, "checker_penetration_mm": round(pen * 1e3, 3)}
        if parts and best[1] is not None:
            out.update(self.parts_at(body, best[1], best[2]))
        return out, best[0], body

    def arm_clear(self, q):
        """The arm links (shoulder skipped, as ArmWorld.collisions) and the gripper hulls against the scene."""
        best = (math.inf, None, None)
        for link, o, lo, hi in self.arm.arm_objects(q, ("shoulder_link",)):
            for name, b in self.targets:
                if gap((lo, hi), b["bounds"]) > CUTOFF_M:
                    continue
                d, _ = self._sd(o, b["manager"], single=True)
                if d < best[0]:
                    best = (d, link, name)
        return {"mm": None if best[1] is None else round(best[0] * 1e3, 2), "link": best[1], "target": best[2]}

    def hits(self, q):
        """The planner's check at q with the dish held (links and gripper 15 mm from the scene, the held dish by the pair
        rule, the held dish 2 mm from the links)."""
        return sorted({f"{a}|{b}" for a, b in self.arm.collisions(q, held=(self.kind, self.M, self.oid), ignore=(self.oid,),
                                                                    first=False)})

    def parts_at(self, body, target, pts):
        """The dish part and the target part nearest the FCL nearest points (prim names, as the PhysX colliders)."""
        fcl = self.fcl
        tb = dict(self.targets)[target]
        t_paths = ([p.name for p in self.checker.parts[target]] if target in self.checker.parts else
                   [p.name for p in self.checker.parts[target.split("_")[0]]] if target in self.arm.dishes else [target])
        d_paths = [p.name for p in self.checker.parts[self.kind]]

        def nearest(objs, paths, point):
            sph = fcl.CollisionObject(fcl.Sphere(1e-6), fcl.Transform(np.eye(3), np.asarray(point, float)))
            best = (math.inf, None)
            for o, pth in zip(objs, paths):
                d = fcl.distance(sph, o, fcl.DistanceRequest(enable_signed_distance=True), fcl.DistanceResult())
                if d < best[0]:
                    best = (d, pth)
            return best
        p0, p1 = pts
        a = (nearest(body["objects"], d_paths, p0), nearest(tb["objects"], t_paths, p1), p0, p1)
        b = (nearest(body["objects"], d_paths, p1), nearest(tb["objects"], t_paths, p0), p1, p0)
        (_, dn), (_, tn), dpt, tpt = min((a, b), key=lambda x: x[0][0] + x[1][0])
        return {"dish_part": str(dn).rsplit("/", 1)[-1], "target_part": str(tn).rsplit("/", 1)[-1],
                "dish_point_m": [round(float(v), 5) for v in dpt], "target_point_m": [round(float(v), 5) for v in tpt]}


# ----------------------------------------------------------------------------------------------- one run
def read_run(run_dir):
    rows = [json.loads(x) for x in (run_dir / "trials/trial.jsonl").read_text().splitlines() if x.strip()]
    phys = [json.loads(x) for x in (run_dir / "trials/trial_physics.jsonl").read_text().splitlines() if x.strip()]
    rec = json.loads(next(run_dir.glob("placing_*.json")).read_text())
    row = rec["rows"][0]
    traj = np.load(run_dir / "trials" / Path(row["traj"]).name)
    return rows, phys, rec, row, traj


def leg_of(S, k0, k1, prepend=None, tail=None):
    """The planned waypoints of samples S[k0..k1] (their distinct q_cmd; ``prepend`` = the leg's start command, ``tail`` =
    the plan's end when the leg was cut), densified."""
    qs = ([prepend] if prepend is not None else []) + [S[k]["payload"]["q_cmd"] for k in range(k0, k1 + 1)]
    W = distinct(qs)
    n_cmd = len(W)
    if tail is not None and np.abs(np.asarray(tail) - W[-1]).max() > 1e-6:
        W = np.vstack([W, tail])
    Pq, src = densify(W)
    return W, n_cmd, Pq, src


def analyse(name, run_dir, B, checker, model, dish_pts):
    t0 = time.monotonic()
    rows, phys, rec, row, traj = read_run(run_dir)
    oid, kind = row["dish"], row["kind"]
    S = [r for r in rows if r["event"] == "sample"]
    ev = [r for r in rows if r["event"] in ("contact_begin", "contact_end")]
    M = orthonormal(np.asarray(row["in_hand"]["M"], float))
    drop = (row.get("dropped") or {}).get("tick")
    ks = {ph: [k for k, r in enumerate(S) if r["phase"] == ph] for ph in (LIFT, CARRY)}
    kc = [k for k in ks[CARRY] if drop is None or S[k]["tick"] <= drop]
    phys_at = {p["step"]: p for p in phys}
    start_step = min(phys_at, key=lambda s: abs(s - S[kc[0]]["tick"]))
    W_ = World(B, checker, model, rec, phys_at[start_step], oid, kind, M)
    q_ds = np.radians(row["reset"]["ring_plan"]["chosen"]["descent_start_q_deg"])   # the plan's end (0.1 deg in the record)

    # the FK check: TCP by FK of the logged q against the logged wrist 3 body pose (the traj npz, the same sim times)
    w3 = [p.endswith("/wrist_3_link") for p in json.loads((run_dir / "trials/robot_meshes.json").read_text())["body_paths"]]
    i_w3 = w3.index(True)
    t_np = np.asarray(traj["t"])
    fk_pos, fk_rot = [], []
    for k in ks[LIFT] + kc:
        j = int(np.argmin(np.abs(t_np - S[k]["sim_s"])))
        if abs(t_np[j] - S[k]["sim_s"]) > 1e-4:
            continue
        w, x, y, z = traj["body_quat_wxyz"][j, i_w3]
        T_log = T7(np.r_[traj["body_pos"][j, i_w3], [x, y, z, w]]) @ W_.arm.T_w3_tcp
        T_fk = W_.tcp(np.asarray(S[k]["payload"]["q"]))
        fk_pos.append(np.linalg.norm(T_fk[:3, 3] - T_log[:3, 3]) * 1e3)
        fk_rot.append(rot_deg(T_fk[:3, :3], T_log[:3, :3]))

    legs, per_sample = {}, {}
    for leg, idx in (("lift", ks[LIFT]), ("carry", kc)):
        cut = leg == "carry" and drop is not None
        W, n_cmd, Pq, src = leg_of(S, idx[0], idx[-1], prepend=S[idx[0] - 1]["payload"]["q_cmd"] if leg == "lift" else None,
                                   tail=q_ds if cut else None)
        PT = np.array([W_.tcp(q) for q in Pq])
        P_tcp, P_dish = PT[:, :3, 3], (PT @ M)[:, :3, 3]
        cols = {c: [] for c in ("tick", "t_s", "tcp_lateral_mm", "dish_lateral_mm", "tcp_tracking_mm", "lag_rad",
                                "in_hand_mm", "in_hand_deg", "in_hand_surface_mm", "dish_clearance_mm", "dish_clearance_target",
                                "plan_s")}
        feet = {}
        for k in idx:
            p = S[k]["payload"]
            q, qc = np.asarray(p["q"], float), np.asarray(p["q_cmd"], float)
            T_e, T_c = W_.tcp(q), W_.tcp(qc)
            T_h, T_d = T_e @ M, T7(p["dishes"][oid])
            j_c = int(np.argmin(np.abs(Pq - qc).max(1)))
            lt, it, tt = nearest_on(P_tcp, T_e[:3, 3], j_c - BACK, j_c + AHEAD)
            ld, i_d, t_d = nearest_on(P_dish, T_d[:3, 3], j_c - BACK, j_c + AHEAD)
            dR = T_d[:3, :3] - T_h[:3, :3]
            surf = float(np.linalg.norm(dish_pts @ dR.T + (T_d[:3, 3] - T_h[:3, 3]), axis=1).max())
            ex, _, _ = W_.dish_clear(T_d)
            cols["tick"].append(S[k]["tick"])
            cols["t_s"].append(round(S[k]["sim_s"], 3))
            cols["tcp_lateral_mm"].append(r2(lt * 1e3))
            cols["dish_lateral_mm"].append(r2(ld * 1e3))
            cols["tcp_tracking_mm"].append(r2(np.linalg.norm(T_e[:3, 3] - T_c[:3, 3]) * 1e3))
            cols["lag_rad"].append(r2(np.abs(q - qc).max(), 4))
            cols["in_hand_mm"].append(r2(np.linalg.norm(T_d[:3, 3] - T_h[:3, 3]) * 1e3))
            cols["in_hand_deg"].append(r2(rot_deg(T_h[:3, :3], T_d[:3, :3])))
            cols["in_hand_surface_mm"].append(r2(surf * 1e3))
            cols["dish_clearance_mm"].append(ex["mm"])
            cols["dish_clearance_target"].append(ex["target"])
            cols["plan_s"].append(r2(src[i_d] + t_d * (src[min(i_d + 1, len(src) - 1)] - src[i_d]), 3))
            feet[S[k]["tick"]] = (i_d, t_d)
        # the plan's clearance at every planned sample (0.02 rad)
        pd, pa, ph = [], [], []
        for q in Pq:
            c_, _, _ = W_.dish_clear(W_.tcp(q) @ M)
            pd.append(c_)
            pa.append(W_.arm_clear(q))
            ph.append(W_.hits(q))
        lag = np.array([np.asarray(S[k]["payload"]["q"]) - np.asarray(S[k]["payload"]["q_cmd"]) for k in idx])
        dq = np.array([np.asarray(S[k + 1]["payload"]["q"]) - np.asarray(S[k]["payload"]["q"]) for k in idx[:-1]])
        dt_s = (S[idx[-1]]["tick"] - S[idx[0]]["tick"]) / TICK_HZ
        fin = lambda xs: [x for x in xs if x is not None]
        ex_c = [(c, cols["tick"][i], cols["dish_clearance_target"][i]) for i, c in enumerate(cols["dish_clearance_mm"])
                if c is not None]
        pmin_d = min(((c["mm"], i, c["target"]) for i, c in enumerate(pd) if c["mm"] is not None), default=(None, None, None))
        pmin_a = min(((c["mm"], i, c["link"], c["target"]) for i, c in enumerate(pa) if c["mm"] is not None),
                     default=(None, None, None, None))
        per_t = {}
        for c in pd:
            for t_, v in c["per_target_mm"].items():
                per_t[t_] = min(per_t.get(t_, math.inf), v)
        bad = [i for i, h in enumerate(ph) if h]
        steps = np.abs(np.diff(Pq, axis=0)).max(1) if len(Pq) > 1 else np.zeros(0)
        to_end = np.r_[np.cumsum(steps[::-1])[::-1], 0.]
        under = [i for i, c in enumerate(pd) if c["mm"] is not None and c["mm"] < CLEAR_MM]
        fine = None                                # the 0.002 rad re-scan of the edges next to those samples
        for j in sorted({j for i in under for j in (i - 1, i) if 0 <= j < len(Pq) - 1}):
            a_, b_ = Pq[j], Pq[j + 1]
            n = max(1, int(math.ceil(float(np.abs(b_ - a_).max()) / FINE_RAD)))
            for s_ in np.linspace(0., 1., n + 1):
                q_ = a_ + (b_ - a_) * s_
                c_, _, _ = W_.dish_clear(W_.tcp(q_) @ M)
                if c_["mm"] is not None and (fine is None or c_["mm"] < fine[0]["mm"]):
                    fine = (c_, q_, j, s_)
        if fine is not None:
            c_, q_, j, s_ = fine
            c_, _, _ = W_.dish_clear(W_.tcp(q_) @ M, parts=True)
            fine = {"mm": c_["mm"], "target": c_["target"], "dish_part": c_.get("dish_part"),
                    "target_part": c_.get("target_part"), "checker_pair_ok": c_["checker_pair_ok"],
                    "checker_penetration_mm": c_["checker_penetration_mm"], "planner_hits": W_.hits(q_),
                    "to_end_rad": r2(to_end[j] - s_ * float(np.abs(Pq[j + 1] - Pq[j]).max()), 3),
                    "q_deg": [round(float(v), 2) for v in np.degrees(q_)]}
        legs[leg] = {
            "ticks": [S[idx[0]]["tick"], S[idx[-1]]["tick"]], "samples": len(idx), "duration_s": round(dt_s, 2),
            "path": ("RRT-Connect (rig.follow, 10 ticks per <= 0.02 rad waypoint)" if leg == "carry" else
                     "straight TCP line up (rig.follow, 2 ticks per IK waypoint)"),
            "cut_at_drop_tick": drop if cut else None,
            "commanded_waypoints": n_cmd, "planned_samples": len(Pq),
            "tail_from_record": bool(cut),
            "edges": ([{"from_waypoint": a, "q_deg": [round(float(v), 1) for v in np.degrees(W[a])]}
                       for a in [0] + corners(W[:n_cmd])] if leg == "carry" else None),
            "end_q_deg": [round(float(v), 1) for v in np.degrees(W[-1])],
            "joint_speed_rad_s": {"commanded_mean": r2(np.abs(np.diff(W[:n_cmd], axis=0)).max(1).sum() / max(dt_s, 1e-9), 3),
                                  "executed_max": r2(np.abs(dq).max(1).max() / (6. / TICK_HZ), 3) if len(dq) else None},
            "tcp_lateral_mm": {"max": max(fin(cols["tcp_lateral_mm"])), "mean": r2(np.mean(fin(cols["tcp_lateral_mm"])))},
            "dish_lateral_mm": {"max": max(fin(cols["dish_lateral_mm"])), "mean": r2(np.mean(fin(cols["dish_lateral_mm"])))},
            "tcp_tracking_mm": {"max": max(fin(cols["tcp_tracking_mm"])), "mean": r2(np.mean(fin(cols["tcp_tracking_mm"])))},
            "lag_rad": {"max": r2(np.abs(lag).max(), 4), "per_joint_max": [r2(v, 4) for v in np.abs(lag).max(0)]},
            "in_hand": {"max_mm": max(cols["in_hand_mm"]), "max_deg": max(cols["in_hand_deg"]),
                        "max_surface_mm": max(cols["in_hand_surface_mm"])},
            "executed_dish_clearance_min": None if not ex_c else
                dict(zip(("mm", "tick", "target"), min(ex_c, key=lambda x: x[0]))),
            "planned_dish_clearance_min": {"mm": pmin_d[0], "at_planned_sample": pmin_d[1], "target": pmin_d[2],
                                           "per_target_mm": {t_: round(v, 2) for t_, v in sorted(per_t.items())}},
            "planned_arm_clearance_min": dict(zip(("mm", "at_planned_sample", "link", "target"), pmin_a)),
            "planner_check_at_0p02rad": {"samples_failing": len(bad), "of": len(Pq),
                                         "first": None if not bad else {"planned_sample": bad[0], "hits": ph[bad[0]],
                                                                        "to_end_rad": r2(to_end[bad[0]], 3)},
                                         "hits": sorted({h for i in bad for h in ph[i]})},
            "planned_dish_clearance_at_end_mm": pd[-1]["mm"], "planned_dish_target_at_end": pd[-1]["target"],
            "planned_dish_under_20mm": {"samples": len(under), "of": len(Pq),
                                        "by_target": {t_: {"samples": len(ii), "min_mm": min(pd[i]["mm"] for i in ii),
                                                           "to_end_rad": [r2(to_end[ii[0]], 3), r2(to_end[ii[-1]], 3)]}
                                                      for t_ in sorted({pd[i]["target"] for i in under})
                                                      for ii in [[i for i in under if pd[i]["target"] == t_]]}},
            "planned_arm_under_20mm": sum(1 for c in pa if c["mm"] is not None and c["mm"] < CLEAR_MM),
            "planned_dish_clearance_fine_min": fine,
            "planned": {"waypoint": [r2(v, 3) for v in src], "to_end_rad": [r2(v, 3) for v in to_end],
                        "dish_clearance_mm": [c["mm"] for c in pd], "dish_target": [c["target"] for c in pd],
                        "arm_clearance_mm": [c["mm"] for c in pa], "arm_link": [c["link"] for c in pa],
                        "arm_target": [c["target"] for c in pa], "planner_hits": ph},
        }
        per_sample[leg] = cols
        legs[leg]["_aux"] = (Pq, src, P_tcp, P_dish, pd, pa, ph, feet)

    # transit_to's via-point: the descent start's TCP raised to z_safe, IK near the descent start (one leg when it fails)
    from dishsim_frigidaire.robot import rig as RG

    class KinRig(RG.Rig):                          # the rig's IK without a simulator (as frigidaire_robot_reach.World)
        def __init__(s, T_wb, T_w3_tcp):
            s.T_wb, s.T_bw, s.T_w3_tcp = T_wb, np.linalg.inv(T_wb), T_w3_tcp
    z_safe = float(rec["flags"]["z_safe_m"])
    above = W_.tcp(q_ds)
    above[2, 3] = z_safe
    q_a = KinRig(W_.arm.T_wb, W_.arm.T_w3_tcp).ik(above, q_ds)
    via = {"z_m": z_safe, "ik": q_a is not None,
           "rule": "transit_to: rig.ik of the descent start's TCP at z_safe near q_start, then arm.free with the bowl held "
                   "(the carry rule, bowl mouth down and hand above it, not checked here)"}
    if q_a is not None:
        via.update(q_deg=[round(float(v), 1) for v in np.degrees(q_a)], planner_hits=W_.hits(q_a))

    # the contact tick and the closest point (carry)
    car = legs["carry"]
    cols = per_sample["carry"]
    lo_t, hi_t = car["ticks"]
    scene_ev = []
    for e in ev:
        p = e["payload"]
        if not (lo_t - 6 <= e["tick"] <= hi_t + 6) or oid not in p["pair"]:
            continue
        other_cat = p["cats"][1 - p["pair"].index(oid)]
        if other_cat in HAND:
            continue
        scene_ev.append({"event": e["event"], "tick": e["tick"], "pair": "|".join(p["pair"]), "cats": p["cats"],
                         **{k: v for k, v in p.items() if k not in ("pair", "cats")}})
    begins = [e for e in scene_ev if e["event"] == "contact_begin"]
    contact_tick = begins[0]["tick"] if begins else None
    i_close = int(np.argmin([c if c is not None else math.inf for c in cols["dish_clearance_mm"]]))
    Pq, src, P_tcp, P_dish, pd, pa, ph, feet = car["_aux"]

    def at(i_s, label):
        """The numbers at carry sample i_s, the plan's clearance at the planned point matching the executed dish."""
        k = kc[i_s]
        p = S[k]["payload"]
        q = np.asarray(p["q"], float)
        T_e = W_.tcp(q)
        T_d = T7(p["dishes"][oid])
        i_d, t_d = feet[S[k]["tick"]]
        j1 = min(i_d + 1, len(Pq) - 1)
        q_p = Pq[i_d] + t_d * (Pq[j1] - Pq[i_d])
        T_p = W_.tcp(q_p)
        pl, _, _ = W_.dish_clear(T_p @ M, parts=True)
        ex, _, _ = W_.dish_clear(T_d, parts=True)
        eh, _, _ = W_.dish_clear(T_e @ M)
        # the bowl point nearest the scene at the executed pose, carried with the bowl to its planned pose
        disp = None
        if ex.get("dish_point_m"):
            x_loc = np.linalg.inv(T_d) @ np.r_[ex["dish_point_m"], 1.]
            disp = float(np.linalg.norm((T_d @ x_loc)[:3] - ((T_p @ M) @ x_loc)[:3]) * 1e3)
        dR = T_d[:3, :3] - (T_p @ M)[:3, :3]
        surf = float(np.linalg.norm(dish_pts @ dR.T + (T_d[:3, 3] - (T_p @ M)[:3, 3]), axis=1).max())
        before = [v for t_, v in zip(cols["tick"], cols["in_hand_mm"]) if t_ <= S[k]["tick"]]
        rem = float(np.abs(np.diff(Pq[i_d:], axis=0)).max(1).sum()) - t_d * float(np.abs(Pq[j1] - Pq[i_d]).max())
        return {"what": label, "sample_tick": S[k]["tick"], "t_s": round(S[k]["sim_s"], 3),
                "tcp_lateral_mm": cols["tcp_lateral_mm"][i_s], "dish_lateral_mm": cols["dish_lateral_mm"][i_s],
                "dish_surface_from_plan_mm": r2(surf * 1e3),
                "contact_point_from_plan_mm": r2(disp),
                "tcp_tracking_mm": cols["tcp_tracking_mm"][i_s], "lag_rad": cols["lag_rad"][i_s],
                "lag_per_joint_rad": [r2(v, 4) for v in q - np.asarray(p["q_cmd"], float)],
                "in_hand_mm": cols["in_hand_mm"][i_s], "in_hand_deg": cols["in_hand_deg"][i_s],
                "in_hand_surface_mm": cols["in_hand_surface_mm"][i_s],
                "in_hand_before_max_mm": max(before) if before else None,
                "pad_N": p.get("pad_N"),
                "planned": {"plan_s": cols["plan_s"][i_s], "to_end_rad": r2(rem, 3),
                            "q_deg": [round(float(v), 2) for v in np.degrees(q_p)],
                            "dish_clearance_mm": pl["mm"], "dish_target": pl["target"], "dish_part": pl.get("dish_part"),
                            "target_part": pl.get("target_part"), "dish_per_target_mm": pl["per_target_mm"],
                            "checker_pair_ok": pl["checker_pair_ok"], "checker_penetration_mm": pl["checker_penetration_mm"],
                            "arm": W_.arm_clear(q_p), "planner_hits": W_.hits(q_p)},
                "executed": {"dish_clearance_mm": ex["mm"], "dish_target": ex["target"], "dish_part": ex.get("dish_part"),
                             "target_part": ex.get("target_part"), "dish_point_m": ex.get("dish_point_m"),
                             "target_point_m": ex.get("target_point_m"),
                             "dish_at_hold_clearance_mm": eh["mm"], "dish_at_hold_target": eh["target"],
                             "arm": W_.arm_clear(q)}}

    contact = None
    if contact_tick is not None:
        i_s = max(i for i, t_ in enumerate(cols["tick"]) if t_ <= contact_tick)
        contact = at(i_s, "the last sample at or before the contact tick")
        contact.update(tick=contact_tick, pairs=[e for e in scene_ev if e["tick"] == contact_tick],
                       pair_ends=[e for e in scene_ev if e["event"] == "contact_end"],
                       next_samples=[{"tick": cols["tick"][j], "in_hand_mm": cols["in_hand_mm"][j],
                                      "dish_lateral_mm": cols["dish_lateral_mm"][j], "lag_rad": cols["lag_rad"][j],
                                      "dish_clearance_mm": cols["dish_clearance_mm"][j]}
                                     for j in range(i_s + 1, min(i_s + 3, len(cols["tick"])))])
    closest = at(i_close, "the carry sample with the smallest executed dish clearance")

    for leg in legs.values():
        leg.pop("_aux")
    fk = {"samples": len(fk_pos), "max_mm": r2(max(fk_pos) if fk_pos else None, 3),
          "max_deg": r2(max(fk_rot) if fk_rot else None, 3),
          "rule": "TCP by FK of the logged q through the arm model against the logged wrist 3 body pose @ T_w3_tcp (traj npz)"}
    out = {"run": name, "dish": oid, "kind": kind, "slot": row.get("slot"), "failed_at": row.get("failed_at"),
           "why": row.get("why"), "carry_watch": row.get("carry_watch"),
           "hold": {"M": np.round(M, 6).tolist(), "dish_from_tcp_mm": r2(np.linalg.norm(M[:3, 3]) * 1e3, 1),
                    "source": row["in_hand"]["source"]},
           "descent_start_q_deg": row["reset"]["ring_plan"]["chosen"]["descent_start_q_deg"],
           "scene": {"physics_step": start_step, "others": W_.others, "racks": phys_at[start_step]["joints"]},
           "fk_check": fk, "via_above_descent_start": via, "legs": legs, "contact": contact, "closest": closest,
           "carry_contacts": scene_ev, "per_sample": per_sample, "seconds": round(time.monotonic() - t0, 1)}
    return out


# ----------------------------------------------------------------------------------------------- the knob and the note
def knob(b):
    """The plan's rule on the dropped bowl's contact tick."""
    c = b["contact"]
    dev = max(c["tcp_lateral_mm"], c["dish_lateral_mm"])
    clr = c["planned"]["dish_clearance_mm"]
    off = c["in_hand_before_max_mm"]
    conds = {"deviation_over_15mm": dev > DEV_SLOW_MM,
             "deviation_under_15mm_and_planned_clearance_under_20mm": dev <= DEV_SLOW_MM and clr is not None and clr < CLEAR_MM,
             "in_hand_offset_over_2mm": off is not None and off > IN_HAND_MM}
    names = {"deviation_over_15mm": ("the last carry leg slowed", "rig max_step 0.004 rad (0.25 rad/s)"),
             "deviation_under_15mm_and_planned_clearance_under_20mm": ("the carry legs' scene margin", "arm.margin 0.020 m"),
             "in_hand_offset_over_2mm": ("the carry planned from the measured in-hand pose after the pick", "M re-measured")}
    first = next((k for k, v in conds.items() if v), None)
    return {"inputs": {"deviation_mm": dev, "tcp_lateral_mm": c["tcp_lateral_mm"], "dish_lateral_mm": c["dish_lateral_mm"],
                       "planned_dish_clearance_mm": clr, "planned_arm_clearance_mm": c["planned"]["arm"]["mm"],
                       "in_hand_before_contact_max_mm": off, "at_tick": c["sample_tick"], "dish": b["dish"]},
            "conditions": conds, "named": None if first is None else {"rule": first, "knob": names[first][0],
                                                                      "value": names[first][1]},
            "also_true": [k for k, v in conds.items() if v and k != first],
            "rule": "plans/2026-11-04-bowl-carry-clearance.md: deviation over 15 mm = the last leg slowed (rig max_step 0.004 "
                    "rad, 0.25 rad/s); deviation under 15 mm with the plan's clearance under 20 mm = the carry legs' scene "
                    "margin (arm.margin) to 20 mm; an in-hand offset over 2 mm = the carry planned from the measured in-hand "
                    "pose after the pick (deviation = the larger of the TCP and dish lateral deviations; the plan's clearance "
                    "= the held dish's at the planned point matching the executed dish; the in-hand offset = the largest "
                    "before the contact)"}


def mm(v):
    return "over 100" if v is None else v


def note(res, kn, cmd, out_json):
    L = ["# Bowl carry deviation, Kit-free (phase 1 of plans/2026-11-04-bowl-carry-clearance.md)", "",
         f"Command (Mac, from the repo root): `{cmd}`", "",
         f"Output: `{out_json}`. The planned leg is the rig's commanded waypoints (q_cmd), each straight joint edge sampled "
         f"at 0.02 rad, with the TCP and the bowl by FK through `{ARM_MODEL}` and the bowl at the pick's measured hold M. "
         "An RRT-Connect leg is a polyline of straight joint edges, so the plan's straight joint line is taken per edge. "
         "Lateral deviation is the distance to the nearest point of the planned path, and the tracking error compares TCP "
         "at q with TCP at q_cmd at the same tick. Clearances are FCL signed distances in mm (negative means overlap). The "
         "planner's check is ArmWorld.collisions with the bowl held: links and gripper 15 mm from the scene, the held bowl "
         "by the bench checker's pair rule (up to 1 mm penetration passes), the held bowl 2 mm from the links.", ""]
    for b in res:
        L.append(f"## {b['dish']} ({b['run']}, {b['slot']}): {b['failed_at'] or 'carried and seated'}")
        lf, cr, via = b["legs"]["lift"], b["legs"]["carry"], b["via_above_descent_start"]
        L.append(f"FK check against the logged wrist 3: {b['fk_check']['max_mm']} mm, {b['fk_check']['max_deg']} deg over "
                 f"{b['fk_check']['samples']} samples. Lift: {lf['commanded_waypoints']} commanded waypoints over "
                 f"{lf['duration_s']} s, mean commanded joint speed {lf['joint_speed_rad_s']['commanded_mean']} rad/s. Carry: one "
                 f"RRT-Connect leg of {len(cr['edges'])} straight joint edges (corners at waypoints "
                 f"{', '.join(str(e['from_waypoint']) for e in cr['edges'][1:]) or 'none'}), {cr['commanded_waypoints']} "
                 f"waypoints over {cr['duration_s']} s, mean commanded joint speed {cr['joint_speed_rad_s']['commanded_mean']} "
                 f"rad/s" + (f", cut at the drop tick {cr['cut_at_drop_tick']} (the tail to the descent start from the "
                             f"record's q, rounded to 0.1 deg)" if cr["cut_at_drop_tick"] else "")
                 + f". transit_to's via-point at z {via['z_m']} m above the descent start: "
                 + ("no IK, so the carry was the one leg" if not via["ik"] else
                    f"IK at {via['q_deg']} deg, planner hits {via['planner_hits'] or 'none'}") + ".")
        L.append("")
        L.append("| leg | samples | TCP lateral max mm | bowl lateral max mm | tracking max mm | lag max rad | in-hand max mm "
                 "(deg, surface mm) | executed bowl clearance min mm | planned bowl clearance min mm | planned bowl "
                 "clearance at the end mm | planned arm clearance min mm | planner check fails at 0.02 rad |")
        L.append("|---|---|---|---|---|---|---|---|---|---|---|---|")
        for leg, g in b["legs"].items():
            ex = g["executed_dish_clearance_min"] or {}
            pd, pa, pc = g["planned_dish_clearance_min"], g["planned_arm_clearance_min"], g["planner_check_at_0p02rad"]
            fails = f"{pc['samples_failing']} of {pc['of']}" + (
                f" ({', '.join(pc['hits'])}, {pc['first']['to_end_rad']} rad before the end)" if pc["first"] else "")
            L.append(f"| {leg} | {g['samples']} | {g['tcp_lateral_mm']['max']} | {g['dish_lateral_mm']['max']} | "
                     f"{g['tcp_tracking_mm']['max']} | {g['lag_rad']['max']} | {g['in_hand']['max_mm']} "
                     f"({g['in_hand']['max_deg']}, {g['in_hand']['max_surface_mm']}) | {ex.get('mm')} ({ex.get('target')}) | "
                     f"{mm(pd['mm'])} ({pd['target']}) | {mm(g['planned_dish_clearance_at_end_mm'])} "
                     f"({g['planned_dish_target_at_end'] or 'nothing within 100 mm'}) | {mm(pa['mm'])} ({pa['link']} vs "
                     f"{pa['target']}) | {fails} |")
        u = cr["planned_dish_under_20mm"]
        L.append("")
        L.append(f"On the planned carry the bowl is under 20 mm from the scene at {u['samples']} of {u['of']} planned samples ("
                 + "; ".join(f"{t_} {g_['samples']}, at least {g_['min_mm']} mm, from {g_['to_end_rad'][0]} to "
                             f"{g_['to_end_rad'][1]} rad before the descent start" for t_, g_ in u["by_target"].items())
                 + f"), and the arm at {cr['planned_arm_under_20mm']}."
                 + ("" if not cr["planned_dish_clearance_fine_min"] else
                    " Re-scanned at 0.002 rad next to those samples, the planned bowl comes to "
                    f"{cr['planned_dish_clearance_fine_min']['mm']} mm from {cr['planned_dish_clearance_fine_min']['target']} "
                    f"({cr['planned_dish_clearance_fine_min']['dish_part']} against "
                    f"{cr['planned_dish_clearance_fine_min']['target_part']}) "
                    f"{cr['planned_dish_clearance_fine_min']['to_end_rad']} rad before the descent start, where the checker's "
                    f"pair rule {'passes' if cr['planned_dish_clearance_fine_min']['checker_pair_ok'] else 'fails'} "
                    f"(contact depth {cr['planned_dish_clearance_fine_min']['checker_penetration_mm']} mm)."))
        L.append("")
        for what, c in (("Contact tick", b["contact"]), ("Closest point", b["closest"])):
            if c is None:
                continue
            pl, ex = c["planned"], c["executed"]
            head = (f"{what} {c['tick']} (sample {c['sample_tick']}): " if what == "Contact tick" else
                    f"{what} (sample {c['sample_tick']}): ")
            L.append(head + f"TCP lateral {c['tcp_lateral_mm']} mm, bowl lateral {c['dish_lateral_mm']} mm (its surface "
                     f"{c['dish_surface_from_plan_mm']} mm and its point nearest the scene {c['contact_point_from_plan_mm']} "
                     f"mm from the planned pose), tracking {c['tcp_tracking_mm']} mm, lag {c['lag_rad']} rad, in-hand offset "
                     f"{c['in_hand_mm']} mm and {c['in_hand_deg']} deg (at most {c['in_hand_before_max_mm']} mm up to this "
                     f"sample), {pl['to_end_rad']} rad of planned path left to the descent start. The plan's bowl clearance "
                     f"there is {pl['dish_clearance_mm']} mm to {pl['dish_target']} ({pl['dish_part']} against "
                     f"{pl['target_part']}, the checker's pair rule {'passes' if pl['checker_pair_ok'] else 'fails'}), the "
                     f"plan's arm clearance {pl['arm']['mm']} mm ({pl['arm']['link']} vs {pl['arm']['target']}), and the "
                     f"planner's check flags {', '.join(pl['planner_hits']) if pl['planner_hits'] else 'nothing'}. Executed: "
                     f"bowl clearance {ex['dish_clearance_mm']} mm to {ex['dish_target']} ({ex['dish_part']} against "
                     f"{ex['target_part']}), {ex['dish_at_hold_clearance_mm']} mm for the bowl at the hold on the executed "
                     f"arm, and the arm {ex['arm']['mm']} mm ({ex['arm']['link']} vs {ex['arm']['target']}).")
            if what == "Contact tick":
                prs = "; ".join(f"{e['event']} {e['pair']} at tick {e['tick']} (colliders {', '.join(e['colliders'])}"
                                + (f", {e['normal_N']} N" if 'normal_N' in e else f", peak {e['peak_N']} N for "
                                   f"{e['duration_s']} s") + ")" for e in c["pairs"] + c["pair_ends"])
                nxt = "; ".join(f"tick {x['tick']} in-hand {x['in_hand_mm']} mm, bowl lateral {x['dish_lateral_mm']} mm"
                                for x in c["next_samples"])
                L.append(f"Contact pairs: {prs}. Pad normal forces at the sample (N): {c['pad_N']}. After it: {nxt}.")
            L.append("")
    nm = kn["named"]
    i = kn["inputs"]
    L.append("## Knob")
    L.append(f"On {i['dish']} at the contact sample {i['at_tick']} the deviation is {i['deviation_mm']} mm, the plan's bowl "
             f"clearance {i['planned_dish_clearance_mm']} mm (arm {i['planned_arm_clearance_mm']} mm) and the in-hand offset "
             f"up to the contact at most {i['in_hand_before_contact_max_mm']} mm. Of the rule's three conditions "
             + ", ".join(f"{k.replace('in_hand', 'in-hand').replace('_', ' ').replace('15mm', '15 mm').replace('20mm', '20 mm').replace('2mm', '2 mm')} "
                         f"is {'true' if v else 'false'}" for k, v in kn["conditions"].items()) + ". "
             + (f"The rule names {nm['knob']}, {nm['value']}." if nm else "The rule names no knob.")
             + (f" Also true: {', '.join(kn['also_true'])}." if kn["also_true"] else ""))
    L.append("")
    reach = []
    for b in res:
        cr = b["legs"]["carry"]
        reach.append(f"{b['dish']} planned arm clearance at least {cr['planned_arm_clearance_min']['mm']} mm over the carry "
                     f"({cr['planned_arm_under_20mm']} samples under 20 mm), planned bowl clearance at the descent start "
                     f"{cr['planned_dish_clearance_at_end_mm']} mm")
    L.append("For the integration: arm.margin in collide.ArmWorld covers the arm links and the gripper only, and the held "
             "bowl passes on the checker's pair rule. " + "; ".join(reach) + ". So arm.margin 0.020 m alone leaves both "
             "carry paths as planned. A 20 mm margin that reaches the held bowl would also reject both descent starts as "
             "planned today. The carry legs here are RRT-Connect paths that rig.follow runs at 10 ticks per 0.02 rad "
             "waypoint, about 0.24 rad/s already, and max_step applies to straight move_joint legs only. The RRT-Connect "
             "edges are checked at 0.03 rad (collide.rrt_connect_np resolution) and followed at 0.02 rad waypoints.")
    return "\n".join(L) + "\n"


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--load-dir", default=LOAD)
    ap.add_argument("--runs", default=RUNS, help="comma list of run ids under <load-dir>/runs")
    ap.add_argument("--out", default=None, help="default <load-dir>__fixes/carry")
    ap.add_argument("--arm-model", default=ARM_MODEL)
    a = ap.parse_args()
    from dishsim_frigidaire.loading import visual_points
    from dishsim_frigidaire.robot import collide as C
    load = ROOT / a.load_dir
    out = ROOT / (a.out or f"{a.load_dir}__fixes/carry")
    out.mkdir(parents=True, exist_ok=True)
    runs = [r for r in a.runs.split(",") if r]
    rec0 = json.loads(next((load / "runs" / runs[0]).glob("placing_*.json")).read_text())
    B = load_bench(rec0["rows"][0]["counter_front_y"])
    checker = B.bench_checker()
    model = C.load_model(ROOT / a.arm_model)
    res = []
    for r in runs:
        kind = json.loads(next((load / "runs" / r).glob("placing_*.json")).read_text())["rows"][0]["kind"]
        pts = np.asarray(visual_points(B.ASSETS / f"{kind}.usda"), float)
        pts = pts[:: max(1, len(pts) // 4000)]
        res.append(analyse(r, load / "runs" / r, B, checker, model, pts))
        b = res[-1]
        print(f"[INFO] {r}: {b['dish']} carry {b['legs']['carry']['samples']} samples, contact "
              f"{(b['contact'] or {}).get('tick')}, {b['seconds']} s", flush=True)
    dropped = [b for b in res if b["contact"] is not None] or res[:1]
    kn = knob(dropped[0]) if dropped[0]["contact"] is not None else {"named": None, "inputs": {}, "conditions": {}}
    cmd = "PYTHONPATH=code/src .venv-kitfree/bin/python code/execution/frigidaire/frigidaire_carry_deviation.py" + "".join(
        f" {shlex.quote(x)}" for x in sys.argv[1:])
    rel = out.relative_to(ROOT)
    doc = {"plan": "plans/2026-11-04-bowl-carry-clearance.md (phase 1)", "command": cmd, "load_dir": a.load_dir,
           "arm_model": a.arm_model, "step_rad": STEP_RAD, "knob": kn, "bowls": {b["dish"]: b for b in res}}
    (out / "carry_deviation.json").write_text(json.dumps(doc, indent=1) + "\n")
    (out / "kitfree.md").write_text(note(res, kn, cmd, rel / "carry_deviation.json"))
    nm = kn.get("named") or {}
    print(f"[RESULT] PASS carry deviation: {', '.join(b['dish'] for b in res)}; knob {nm.get('knob')} ({nm.get('value')}); "
          f"{rel}/carry_deviation.json, kitfree.md", flush=True)


if __name__ == "__main__":
    main()
