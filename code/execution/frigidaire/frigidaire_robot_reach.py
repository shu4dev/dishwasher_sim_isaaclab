#!/usr/bin/env python3
"""Kit-free reach sweep (gates G0-G2) of the UR5e + 2F-85 mouth-up rim pinch: counter grasp -> every rack site.

    code/util/run_py.sh code/execution/frigidaire/frigidaire_robot_reach.py --workers 2

Plan plans/2026-10-23-hardware-limits-mouth-up.md, Experiment 2 (Kit-free half). One dish of a kind stands mouth-up
on the counter at COUNTER_XY; its 12 rim-pinch grasps (robot/grasp.py KIND_RIM) give 12 in-hand poses ("yaws").
Per (kind, site, target, yaw), on the FCL arm model (data/results/robot/arm_model.json) with the appliance, the
counter, the pedestal and the floor in the world, the dish carried with the TCP after the grasp:
  G0  an analytic IK solution on any of the 8 branches at the release pose (no collisions);
  G1  a collision-free IK solution at pre-grasp, grasp, lift (+0.15 m), pre-place (0.10 m back along the approach)
      and release (the site pose + 6 mm);
  G2  the approach and lift lines, RRT-Connect lift -> pre-place within BUDGET_S, and the straight insertion line
      pre-place -> release, all collision-free (an RRT timeout is counted as ``budget``, apart from line failures).
Targets: ``family`` = the packer's pose at the site (plates: the upright-with-lean-bias pose ``upright_pose(slot, +1)``,
bowls / cups: a representative family variant); ``level`` (bowls, cups) = the dish as held, mouth up and level, at
the site's x, y, raised in 5 mm steps until it clears the appliance (the orientation-free reach check).
Rack state per cell: a lower-rack site has the upper rack pushed in, an upper-rack site the lower rack pushed in.
Writes data/results/robot/limits_v1/reach.jsonl (one row per job, appended as they finish) and reach.json (sites).
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys
import time

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
sys.path[:0] = [str(ROOT / "code/src"), str(ROOT / "code/frigidaire/src")]
OUT = ROOT / "data/results/robot/limits_v1"
MOUNT = ROOT / "data/results/robot/mount/easy_s0_any.json"
ARM_MODEL = ROOT / "data/results/robot/arm_model.json"
COUNTER_XY = (.20, -.16)            # the dish's counter spot (shared with frigidaire_robot_episode.py --grasp-gate)
COUNTER_LIFT_M = .002               # resting 2 mm above the slab (the upright3 test case)
HOVER_M, LIFT_M, RELEASE_ABOVE_M = .10, .15, .006
BUDGET_S = 3.
N_YAW = 12
KIND_ORDER = ("bowl", "cup", "plate")
_W = {}


def bench():
    import importlib.util
    spec = importlib.util.spec_from_file_location("frigidaire_bench", ROOT / "code/planner/frigidaire/frigidaire_bench.py")
    B = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(B)
    return B


def mount_T():
    m = json.loads(MOUNT.read_text())["best"]
    c, s = math.cos(m["yaw_rad"]), math.sin(m["yaw_rad"])
    T = np.eye(4)
    T[:3, :3] = [[c, -s, 0], [s, c, 0], [0, 0, 1]]
    T[:3, 3] = m["base"]
    return T, m


def frames_for(B, rack):
    """Component frames of a cell: the baseline (both racks extended), the OTHER rack pushed in."""
    from dishsim_frigidaire.asset import BODY_POSITIONS
    from dishsim_frigidaire.initial_state_candidates import COMPONENT_NAMES
    base = json.loads((B.OUT / "baseline" / "result.json").read_text())
    base = base.get("snapshot", base)["poses"]
    frames = {name: base[name] for name in COMPONENT_NAMES}
    other = "UpperRack" if rack == "LowerRack" else "LowerRack"
    dy = BODY_POSITIONS[other][1] - base[other]["position_m"][1]
    shift = lambda p: {"position_m": [p["position_m"][0], p["position_m"][1] + dy, p["position_m"][2]],
                       "quaternion_xyzw": list(p["quaternion_xyzw"])}
    frames[other] = shift(base[other])
    if other == "LowerRack":
        frames["SilverwareBasket"] = shift(base["SilverwareBasket"])
    return frames


def T_pose(p):
    from dishsim_frigidaire.robot.rig import T_of
    return T_of(p["position_m"], p["quaternion_xyzw"])


def sites(B):
    """{kind: [{site, slot, rack, local}]} from the packer's families (the benchmark filters)."""
    fam, _, _ = B.families()
    out = {k: [] for k in KIND_ORDER}
    for slot in sorted({c["slot"] for c, _ in fam["plate"]}):
        out["plate"].append({"site": slot, "slot": slot, "rack": "LowerRack", "variant": "upright+1",
                             "local": B.upright_pose(slot, +1)})
    by = {}
    for c, _ in fam["bowl"]:
        by.setdefault(c["slot"], []).append(c)
    for slot, lst in sorted(by.items()):
        if slot.startswith("upper"):
            c = lst[0]
            out["bowl"].append({"site": slot, "slot": slot, "rack": c["rack"], "variant": c["variant"],
                                "local": {"position_m": list(c["position"]), "quaternion_xyzw": list(c["quaternion_xyzw"])}})
            continue
        q0 = np.round(lst[0]["quaternion_xyzw"], 3)
        same = [c for c in lst if np.allclose(np.round(c["quaternion_xyzw"], 3), q0)]
        ymed = float(np.median([c["position"][1] for c in same]))
        bins = {}
        for c in same:                                   # one site per 5 cm of x: lowest lift, then y nearest the zone's
            b = int(round(c["position"][0] / .05))
            k = (round(c["position"][2], 4), abs(c["position"][1] - ymed))
            if b not in bins or k < bins[b][0]:
                bins[b] = (k, c)
        for b, (_, c) in sorted(bins.items()):
            out["bowl"].append({"site": f"{slot}@x{c['position'][0]:+.2f}", "slot": slot, "rack": c["rack"],
                                "variant": c["variant"],
                                "local": {"position_m": list(c["position"]), "quaternion_xyzw": list(c["quaternion_xyzw"])}})
    seen = set()
    for c, _ in fam["cup"]:
        if c["slot"] in seen:
            continue
        seen.add(c["slot"])
        out["cup"].append({"site": c["slot"], "slot": c["slot"], "rack": c["rack"], "variant": c["variant"],
                           "local": {"position_m": list(c["position"]), "quaternion_xyzw": list(c["quaternion_xyzw"])}})
    return out


def world_pose(local, frame):
    from dishsim_frigidaire.random_poses import compose_pose
    p, q = compose_pose(frame["position_m"], frame["quaternion_xyzw"], local["position_m"], local["quaternion_xyzw"])
    return {"position_m": [float(v) for v in p], "quaternion_xyzw": [float(v) for v in q]}


def counter_obj():
    T = np.eye(4)
    T[:3, 3] = (COUNTER_XY[0], COUNTER_XY[1], .914 + COUNTER_LIFT_M)
    return T


def level_pose(checker, kind, site_world):
    """The dish as held (mouth up, level) at the site's x, y: the lowest z (5 mm steps) that clears the appliance."""
    x, y, z0 = site_world["position_m"]
    for k in range(80):
        p = {"position_m": [x, y, z0 - .05 + .005 * k], "quaternion_xyzw": [0., 0., 0., 1.]}
        if checker.against_components(checker._body(kind, p, "level"))["valid"]:
            return p
    return None


class World:
    """Per worker: the arm model and one FCL world per rack state."""

    def __init__(self):
        from dishsim_frigidaire import planner as P
        from dishsim_frigidaire.robot import collide as C, rig as RG, ur5e
        self.B = B = bench()
        self.C, self.P = C, P
        self.T_wb, _ = mount_T()
        hulls, X = C.load_model(ARM_MODEL)
        self.T_w3_tcp = RG.T_of(ur5e.T_WRIST3_TCP_POS, ur5e.T_WRIST3_TCP_QUAT_XYZW)

        class KinRig(RG.Rig):                        # the rig's IK/line math without a simulator
            def __init__(s, T_wb):
                s.T_wb, s.T_bw, s.T_w3_tcp = T_wb, np.linalg.inv(T_wb), self.T_w3_tcp
        self.kr = KinRig(self.T_wb)
        self.arms = {}
        for rack in ("LowerRack", "UpperRack"):
            checker = B.bench_checker()
            checker.update_components(frames_for(B, rack))
            arm = C.ArmWorld(checker, P.slab_body(checker, B.COUNTER), hulls, X, self.T_wb, self.T_w3_tcp)
            bx, by, bz = json.loads(MOUNT.read_text())["best"]["base"]
            arm.add_static_box("Pedestal", (bx, by, bz / 2 - .01), (.18, .18, bz - .02))
            arm.add_static_box("Floor", (0., 0., -.05), (6., 6., .1))
            self.arms[rack] = arm
        self.seed = np.array([0.0, -1.712, 1.712, -1.571, -1.571, 0.0])


def branch(q):
    return f"e{'+' if q[2] > 0 else '-'}w{'+' if q[4] > 0 else '-'}"


def free_qs(W, arm, T, kw, seed=None, check=True):
    qs = W.kr.ik_all(T, W.seed if seed is None else seed)
    up = lambda q: -round(float(arm.link_poses(q)["wrist_1_link"][2, 3]), 2)      # wrist-up first (robot.md)
    return sorted([q for q in qs if not check or arm.free(q, **kw)], key=up), len(qs)


def line_ok(W, arm, T0, T1, q0, n, kw, allow_last=0., dish=None):
    qs = W.kr.line_q(T0, T1, n, q0)
    if qs is None:
        return None, "IK branch jump"
    d = float(np.linalg.norm(np.asarray(T1)[:3, 3] - np.asarray(T0)[:3, 3]))
    for i, q in enumerate(qs):
        left = d * (1 - i / max(1, len(qs) - 1))
        extra = {"gripper_may_touch": (dish,)} if (dish and left < allow_last) else {}
        if not arm.free(q, **kw, **extra):
            return None, f"collides {round(left * 1e3)} mm from the end: {arm.collisions(q, **kw, **extra)[:2]}"
    return qs, None


def job(args):
    """One (kind, site, target, yaw) cell -> a row with the gate reached."""
    kind, site, target, k, waive = args
    if "w" not in _W:
        _W["w"] = World()
    W = _W["w"]
    from dishsim_frigidaire.robot import grasp as G
    t0 = time.monotonic()
    arm = W.arms[site["rack"]]
    T_o = counter_obj()
    grasps = G.tilted_grasps(T_o, N_YAW, kind)
    row = {"kind": kind, "site": site["site"], "slot": site["slot"], "rack": site["rack"], "target": target, "yaw": k,
           "gate": "none", "why": None, "waived": waive}
    if k >= len(grasps):
        row["why"] = "no grasp candidate"
        return row
    g = grasps[k]
    T_g = g["T_tcp"]
    M = np.linalg.inv(T_g) @ T_o
    pose = site["world"][target]
    if pose is None:
        row["why"] = "no clear level pose"
        return row
    T_goal = T_pose(pose)
    T_rel = T_goal @ np.linalg.inv(M)
    T_rel[2, 3] += RELEASE_ABOVE_M
    T_pre = G.hover_along(T_g, HOVER_M)
    T_lift = T_g.copy()
    T_lift[2, 3] += LIFT_M
    T_pp = G.hover_along(T_rel, HOVER_M)
    row["release_tcp_z_axis"] = np.round(T_rel[:3, 2], 3).tolist()
    # G0
    n_ik = len(W.kr.ik_all(T_rel, W.seed))
    row["ik_release"] = n_ik
    if n_ik == 0:
        row["why"] = "G0: no IK at the release pose"
        return row
    row["gate"] = "G0"
    # G1 (dish on the counter for the empty-handed waypoints, carried for the rest)
    dish_body = arm.checker._body(kind, W.P.pose_dict(T_o), "dish")
    held = {"held": (kind, M, "dish")}
    sols = {}
    for name, T, kw in (("pre-grasp", T_pre, {}), ("grasp", T_g, {"gripper_may_touch": ("dish",)}),
                        ("lift", T_lift, held), ("pre-place", T_pp, held), ("release", T_rel, held)):
        arm.set_dishes({"dish": dish_body} if name in ("pre-grasp", "grasp") else {})
        qs, n = free_qs(W, arm, T, kw, check=not (waive and name in ("pre-grasp", "grasp")))
        if not qs:
            arm.set_dishes({"dish": dish_body} if name in ("pre-grasp", "grasp") else {})
            any_q = W.kr.ik_all(T, W.seed)
            row["why"] = (f"G1: no collision-free IK at the {name} ({n} IK solutions; first hits "
                          f"{arm.collisions(any_q[0], first=False, **kw)[:3] if any_q else None})")
            arm.set_dishes({})
            return row
        sols[name] = qs
    row["gate"] = "G1"
    row["branch"] = branch(sols["release"][0])
    # G2: approach line, lift line, RRT lift -> pre-place, insertion line
    arm.set_dishes({"dish": dish_body})
    q_pre = sols["pre-grasp"][0]
    if waive:                                            # the counter approach is not collision-checked (the plate)
        qs_a, why = W.kr.line_q(T_pre, T_g, 60, q_pre), "IK branch jump"
    else:
        qs_a, why = line_ok(W, arm, T_pre, T_g, q_pre, 60, {}, allow_last=.06, dish="dish")
    if qs_a is None:
        row["why"] = f"G2: approach line {why}"
        arm.set_dishes({})
        return row
    arm.set_dishes({})
    qs_l, why = line_ok(W, arm, T_g, T_lift, qs_a[-1], 40, held)
    if qs_l is None:
        row["why"] = f"G2: lift line {why}"
        return row
    best = None
    for q_pp in sols["pre-place"][:2]:
        qs_i, why = line_ok(W, arm, T_pp, T_rel, q_pp, 40, held)
        if qs_i is None:
            best = best or f"G2: insertion line {why}"
            continue
        t = time.monotonic()
        path = W.C.rrt_connect(arm, qs_l[-1], [q_pp], time_s=BUDGET_S, collide_kw=held, seed=1 + k)
        row["rrt_s"] = round(time.monotonic() - t, 2)
        if path is None:
            best = "G2: budget (RRT-Connect found no path in %.0f s)" % BUDGET_S
            continue
        row.update(gate="G2", branch=branch(qs_i[-1]), why=None, path_waypoints=len(path))
        row["q"] = {"pre_grasp": q_pre.tolist(), "lift": qs_l[-1].tolist(), "pre_place": q_pp.tolist(),
                    "release": qs_i[-1].tolist()}
        break
    if row["gate"] != "G2":
        row["why"] = best
    row["seconds"] = round(time.monotonic() - t0, 2)
    return row


# ------------------------------------------------------------------ round 3: the orientation sweep
TILTS = (0., 30., 60., 90., 120., 150.)
REPRESENTATIVE = {"bowl": ("upper_bowl_02", "lower_rear@x-0.07", "lower_frontright@x+0.08", "lower_rearright@x-0.02"),
                  "cup": ("glass_left_1", "glass_right_1")}


def rodrigues(k, ang):
    k = np.asarray(k, dtype=float) / np.linalg.norm(k)
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + np.sin(ang) * K + (1 - np.cos(ang)) * K @ K


def measured_M(kind):
    """TCP -> dish after the lift and the 5 s hold, from the grasp-gate trajectories (the trial with the median pivot)."""
    from dishsim_frigidaire.robot import rig as RG, ur5e
    run = OUT / f"gate2_{kind}"
    rows = [json.loads(l) for l in (run / "trials.jsonl").read_text().splitlines() if l.strip()]
    rows = sorted([r for r in rows if r["g3"] and r.get("traj")], key=lambda r: r["hold"]["settle_rotation_deg"])
    r = rows[len(rows) // 2]
    paths = json.loads((run / "trials" / "robot_meshes.json").read_text())["body_paths"]
    i = next(j for j, q in enumerate(paths) if q.endswith("/wrist_3_link"))
    z = np.load(ROOT / r["traj"])
    w = z["body_quat_wxyz"][-1, i]
    T_w3 = RG.T_of(z["body_pos"][-1, i], (w[1], w[2], w[3], w[0]))
    T_tcp = T_w3 @ RG.T_of(ur5e.T_WRIST3_TCP_POS, ur5e.T_WRIST3_TCP_QUAT_XYZW)
    d = z["dish"][-1]
    return np.linalg.inv(T_tcp) @ RG.T_of(d[:3], d[3:7]), {"trial": r["trial"], "pivot_deg": r["hold"]["settle_rotation_deg"]}


def tilt_pose(checker, kind, site, tilt):
    """The dish tilted ``tilt`` deg about the horizontal axis toward the site's lean side (the horizontal part of the
    packer pose's dish axis; the rack front, -y, when that axis is vertical), its origin at the level pose's x, y, at the
    lowest height (5 mm steps) that clears the appliance."""
    from dishsim_frigidaire.robot.rig import T_of
    fam = T_of(site["world"]["family"]["position_m"], site["world"]["family"]["quaternion_xyzw"])
    d = np.array([fam[0, 2], fam[1, 2], 0.])
    d = d / np.linalg.norm(d) if np.linalg.norm(d) > .1 else np.array([0., -1., 0.])
    R = np.eye(3) if tilt == 0 else rodrigues(np.cross([0., 0., 1.], d), np.radians(tilt))
    from scipy.spatial.transform import Rotation
    q = Rotation.from_matrix(R).as_quat().tolist()
    x, y, z0 = site["world"]["level"]["position_m"]
    for k in range(120):                               # from the level pose's height up (Isaac lowers it until contact)
        p = {"position_m": [x, y, z0 + .005 * k], "quaternion_xyzw": q}
        if checker.against_components(checker._body(kind, p, "tilt"))["valid"]:
            return p, d.tolist()
    return None, d.tolist()


_C = {}


def counter_part(W, arm, kind, k, M):
    """G1 + G2 of the counter waypoints for grasp k (cached per worker): the lift line carries the dish as grasped
    (it pivots into M only once it is off the counter), the lifted waypoint carries it at the measured M."""
    key = (kind, k, id(arm))
    if key in _C:
        return _C[key]
    from dishsim_frigidaire.robot import grasp as G
    T_o = counter_obj()
    g = G.tilted_grasps(T_o, N_YAW, kind)[k]
    T_g = g["T_tcp"]
    T_pre, T_lift = G.hover_along(T_g, HOVER_M), T_g.copy()
    T_lift[2, 3] += LIFT_M
    dish_body = arm.checker._body(kind, W.P.pose_dict(T_o), "dish")
    held = {"held": (kind, M, "dish")}
    out = {"ok": False}
    for name, T, kw, dish in (("pre-grasp", T_pre, {}, True), ("grasp", T_g, {"gripper_may_touch": ("dish",)}, True),
                              ("lift", T_lift, held, False)):
        arm.set_dishes({"dish": dish_body} if dish else {})
        qs, n = free_qs(W, arm, T, kw)
        if not qs:
            out["why"] = f"G1: no collision-free IK at the {name} ({n} IK solutions)"
            arm.set_dishes({})
            _C[key] = out
            return out
        out[name] = qs
    arm.set_dishes({"dish": dish_body})
    qs_a, why = line_ok(W, arm, T_pre, T_g, out["pre-grasp"][0], 60, {}, allow_last=.06, dish="dish")
    arm.set_dishes({})
    if qs_a is None:
        out["why"] = f"G2: approach line {why}"
    else:
        qs_l, why = line_ok(W, arm, T_g, T_lift, qs_a[-1], 40, {"held": (kind, np.linalg.inv(T_g) @ T_o, "dish")})
        out.update(ok=qs_l is not None, why=None if qs_l is not None else f"G2: lift line {why}",
                   q_lift=None if qs_l is None else qs_l[-1])
    _C[key] = out
    return out


def orient_job(args):
    kind, site, tilt, k, pose, M = args
    if "w" not in _W:
        _W["w"] = World()
    W = _W["w"]
    from dishsim_frigidaire.robot import grasp as G
    arm = W.arms[site["rack"]]
    row = {"kind": kind, "site": site["site"], "rack": site["rack"], "tilt_deg": tilt, "yaw": k, "gate": "none", "why": None}
    if pose is None:
        row["why"] = "no clear pose"
        return row
    from dishsim_frigidaire.robot.rig import T_of
    phi = 2 * np.pi * k / N_YAW
    T_target = T_of(pose["position_m"], pose["quaternion_xyzw"])
    T_target[:3, :3] = T_target[:3, :3] @ rodrigues([0., 0., 1.], phi)
    T_rel = T_target @ np.linalg.inv(M)
    T_rel[2, 3] += RELEASE_ABOVE_M
    T_pp = G.hover_along(T_rel, HOVER_M)
    row["phi_rad"] = phi
    n_ik = len(W.kr.ik_all(T_rel, W.seed))
    if n_ik == 0:
        row["why"] = "G0: no IK at the release pose"
        return row
    row["gate"] = "G0"
    held = {"held": (kind, M, "dish")}
    arm.set_dishes({})
    sol = {}
    for name, T in (("pre-place", T_pp), ("release", T_rel)):
        qs, n = free_qs(W, arm, T, held)
        if not qs:
            any_q = W.kr.ik_all(T, W.seed)
            row["why"] = (f"G1: no collision-free IK at the {name} ({n} IK solutions; first hits "
                          f"{arm.collisions(any_q[0], first=False, **held)[:2] if any_q else None})")
            return row
        sol[name] = qs
    c = counter_part(W, arm, kind, k, M)
    if "lift" not in c:
        row["why"] = c["why"]
        return row
    row["gate"] = "G1"
    row["branch"] = branch(sol["release"][0])
    if not c["ok"]:
        row["why"] = c["why"]
        return row
    best = None
    for q_pp in sol["pre-place"][:2]:
        qs_i, why = line_ok(W, arm, T_pp, T_rel, q_pp, 40, held)
        if qs_i is None:
            best = best or f"G2: insertion line {why}"
            continue
        path = W.C.rrt_connect(arm, c["q_lift"], [q_pp], time_s=BUDGET_S, collide_kw=held, seed=1 + k)
        if path is None:
            best = "G2: budget (RRT-Connect found no path in %.0f s)" % BUDGET_S
            continue
        row.update(gate="G2", branch=branch(qs_i[-1]), why=None)
        break
    if row["gate"] != "G2":
        row["why"] = best
    return row


def orient_table(rows, poses, leans):
    """Per (kind, site, tilt): the best gate over yaws, the first G2 yaw's phi, the commonest stop, the target pose."""
    order = {"none": 0, "G0": 1, "G1": 2, "G2": 3}
    cells = {}
    for r in rows:
        key = (r["kind"], r["site"], r["tilt_deg"])
        c = cells.setdefault(key, {"kind": r["kind"], "site": r["site"], "rack": r["rack"], "tilt_deg": r["tilt_deg"],
                                   "gate": "none", "phi_rad": None, "n": 0, "G0": 0, "G1": 0, "G2": 0, "whys": {}})
        c["n"] += 1
        for g in ("G0", "G1", "G2"):
            c[g] += order[r["gate"]] >= order[g]
        if order[r["gate"]] > order[c["gate"]]:
            c["gate"] = r["gate"]
        if r["gate"] == "G2" and c["phi_rad"] is None:
            c["phi_rad"] = r["phi_rad"]
        if r.get("why"):
            w = r["why"].split(" (")[0]
            c["whys"][w] = c["whys"].get(w, 0) + 1
    out = {"bowl": [], "cup": []}
    for (kind, site, tilt), c in sorted(cells.items(), key=lambda kv: (kv[0][0], kv[0][1], kv[0][2])):
        c["why"] = max(c["whys"].items(), key=lambda kv: kv[1])[0] if c["whys"] else None
        c["world"] = poses.get((kind, site, tilt))
        c["lean_dir"] = leans.get((kind, site))
        out[kind].append(c)
    return out


def orient_main(a):
    B = bench()
    reach = json.loads((OUT / "reach.json").read_text())
    rows1 = [json.loads(l) for l in (OUT / "reach.jsonl").read_text().splitlines() if l.strip()]
    g2 = {(r["kind"], r["site"]) for r in rows1 if r["target"] == "level" and r["gate"] == "G2"}
    checkers = {}
    for rack in ("LowerRack", "UpperRack"):
        checkers[rack] = B.bench_checker()
        checkers[rack].update_components(frames_for(B, rack))
    jobs, poses, leans, meta = [], {}, {}, {"tilts": TILTS, "measured_M": {}}
    from dishsim_frigidaire.robot import grasp as G
    base_xy = np.asarray(reach["mount"]["base"][:2])
    T_o = counter_obj()
    for kind in ("bowl", "cup"):
        M, src = measured_M(kind)
        meta["measured_M"][kind] = {"M": M.tolist(), **src}
        sites = [s for s in reach["sites"][kind] if (kind, s["site"]) in g2]
        sites.sort(key=lambda s: (s["site"] not in REPRESENTATIVE[kind], REPRESENTATIVE[kind].index(s["site"])
                                  if s["site"] in REPRESENTATIVE[kind] else 0, s["site"]))
        gs = G.tilted_grasps(T_o, N_YAW, kind)
        yaws = [int(i) for i in np.argsort([-float(np.dot(g["T_tcp"][:2, 3] - T_o[:2, 3], base_xy - T_o[:2, 3])) for g in gs])]
        for s in sites:
            for tilt in TILTS:
                pose, lean = tilt_pose(checkers[s["rack"]], kind, s, tilt)
                poses[(kind, s["site"], tilt)], leans[(kind, s["site"])] = pose, lean
                jobs += [(kind, s, tilt, k, pose, M) for k in yaws]
    rep = [j for j in jobs if j[1]["site"] in REPRESENTATIVE[j[0]]]
    jobs = rep + [j for j in jobs if j[1]["site"] not in REPRESENTATIVE[j[0]]]
    print(f"[INFO] orientation sweep: {len(jobs)} jobs ({len(rep)} representative first), M pivots "
          f"{ {k: v['pivot_deg'] for k, v in meta['measured_M'].items()} }", flush=True)
    rows_path = a.out / "orient.jsonl"
    rows, t0 = [], time.monotonic()

    def write_table():
        tab = orient_table(rows, poses, leans)
        (a.out / "orient.json").write_text(json.dumps({**meta, "done": len(rows), "of": len(jobs), "cells": tab}) + "\n")

    import multiprocessing as mp
    with open(rows_path, "a") as f, mp.get_context("fork").Pool(a.workers) as pool:
        for r in pool.imap(orient_job, jobs, chunksize=1):
            rows.append(r)
            f.write(json.dumps(r) + "\n")
            f.flush()
            if len(rows) % 24 == 0:
                write_table()
                print(f"[INFO] {len(rows)}/{len(jobs)} jobs, {time.monotonic() - t0:.0f} s", flush=True)
    write_table()
    print(f"[RESULT] PASS orientation sweep {len(rows)} jobs in {time.monotonic() - t0:.0f} s", flush=True)


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--workers", type=int, default=2)
    ap.add_argument("--kinds", default=",".join(KIND_ORDER))
    ap.add_argument("--targets", default="family,level")
    ap.add_argument("--limit", type=int, default=None, help="dry run: first N jobs only")
    ap.add_argument("--waive-counter-grasp", action="store_true",
                    help="no collision check at the counter pre-grasp, grasp and approach (the plate: the 15 mm margin rejects "
                         "every plate pinch); the rack side is gated as usual; rows go to reach_waived.jsonl")
    ap.add_argument("--out", type=Path, default=OUT)
    ap.add_argument("--orient", action="store_true", help="round 3: the orientation sweep (orient.jsonl, orient.json)")
    a = ap.parse_args()
    a.out = a.out if a.out.is_absolute() else ROOT / a.out
    a.out.mkdir(parents=True, exist_ok=True)
    if a.orient:
        return orient_main(a)
    t0 = time.monotonic()
    B = bench()
    kinds = [k for k in KIND_ORDER if k in a.kinds.split(",")]
    allsites = sites(B)
    checkers = {}
    for rack in ("LowerRack", "UpperRack"):
        checkers[rack] = B.bench_checker()
        checkers[rack].update_components(frames_for(B, rack))
    for kind in kinds:
        for s in allsites[kind]:
            frame = frames_for(B, s["rack"])[s["rack"]]
            fam = world_pose(s["local"], frame)
            s["world"] = {"family": fam, "level": None if kind == "plate" else level_pose(checkers[s["rack"]], kind, fam)}
    meta = {"mount": json.loads(MOUNT.read_text())["best"], "counter_xy": COUNTER_XY, "hover_m": HOVER_M, "lift_m": LIFT_M,
            "release_above_m": RELEASE_ABOVE_M, "budget_s": BUDGET_S, "n_yaw": N_YAW, "arm_model": str(ARM_MODEL.relative_to(ROOT)),
            "sites": {k: allsites[k] for k in kinds}, "started": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())}
    targets = a.targets.split(",")
    from dishsim_frigidaire.robot import grasp as G
    base_xy = np.asarray(meta["mount"]["base"][:2])
    T_o = counter_obj()
    order = {}
    for kind in kinds:                                   # yaws facing the robot first (the episode's pick order)
        gs = G.tilted_grasps(T_o, N_YAW, kind)
        face = [-float(np.dot(g["T_tcp"][:2, 3] - T_o[:2, 3], base_xy - T_o[:2, 3])) for g in gs]
        order[kind] = [int(i) for i in np.argsort(face)]
    meta["yaw_order"] = order
    if not a.waive_counter_grasp:
        (a.out / "reach.json").write_text(json.dumps(meta, indent=1) + "\n")
    jobs = [(kind, s, t, k, a.waive_counter_grasp) for t in targets for kind in kinds for k in order[kind]
            for s in allsites[kind] if not (t == "level" and kind == "plate")]
    if a.limit:
        jobs = jobs[:a.limit]
    print(f"[INFO] {sum(len(allsites[k]) for k in kinds)} sites {[(k, len(allsites[k])) for k in kinds]}, {len(jobs)} jobs, "
          f"{a.workers} workers", flush=True)
    rows = a.out / ("reach_waived.jsonl" if a.waive_counter_grasp else "reach.jsonl")
    done = 0
    with open(rows, "a") as f:
        if a.workers > 1:
            import multiprocessing as mp
            with mp.get_context("fork").Pool(a.workers) as pool:
                for r in pool.imap(job, jobs, chunksize=1):
                    f.write(json.dumps(r) + "\n")
                    f.flush()
                    done += 1
                    if done % 25 == 0:
                        print(f"[INFO] {done}/{len(jobs)} jobs, {time.monotonic() - t0:.0f} s", flush=True)
        else:
            for j in jobs:
                r = job(j)
                print(json.dumps({k: r.get(k) for k in ("kind", "site", "target", "yaw", "gate", "why", "seconds")}), flush=True)
                f.write(json.dumps(r) + "\n")
                done += 1
    print(f"[RESULT] PASS reach sweep {done} jobs in {time.monotonic() - t0:.0f} s -> {rows.relative_to(ROOT)}", flush=True)


if __name__ == "__main__":
    main()
