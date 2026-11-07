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
COUNTER_XY = (.20, -.12)            # the dish's counter spot (shared with frigidaire_robot_episode.py --grasp-gate): the
                                    # --side-spot sweep's spot at the -0.17 edge (2026-10-27; (.20, -.16) at the -0.30 edge)
COUNTER_LIFT_M = .002               # resting 2 mm above the slab (the upright3 test case)
HOVER_M, LIFT_M, RELEASE_ABOVE_M = .10, .15, .006
BUDGET_S = 3.
N_YAW = 12
KIND_ORDER = ("bowl", "cup", "plate")
SEED_FREE = False                   # --seed-free: IK keeps every 2 pi wrap inside the limits (kin.expand_2pi_wraps)
GRID_DX, GRID_EDGE_M = .05, .005    # --counter-grid: x step; the dish stays this far inside the slab's front edge
COUNTER_FRONT_Y = -.17              # --counter-front-y (grasp.pull_counter; the earlier rounds ran at the benchmark's -0.30)
BASKET = True                       # the SilverwareBasket in every FCL world; --no-basket / use_basket(False) (plans/2026-11-04-
                                    # fifteen-of-fifteen.md, the user's basket-out decision for the robot rounds: the real basket lifts
                                    # out) pops it from each rack-state checker after update_components (strip_basket); the frames
                                    # keep all five components, and s0's goals and the benchmark keep the basket
_W = {}


def use_basket(on=True):
    """The basket switch (--no-basket = use_basket(False)): every World built after the call, in this process and in the workers
    it forks, has the SilverwareBasket in its FCL world (``on``) or not. Call it before the first World (the order tool: in the
    parent, before its pool forks). -> the setting"""
    global BASKET
    BASKET = bool(on)
    return BASKET


def strip_basket(checker, on=None):
    """``checker`` (a bench checker after update_components) without the SilverwareBasket's body when the basket is out (``on``
    False; None = BASKET): every check that walks checker.components (the arm's collisions and clearances, the scene targets, the
    seat search, the open-jaw check) then no longer sees it. -> checker"""
    if not (BASKET if on is None else on) and checker.components:
        checker.components.pop("SilverwareBasket", None)
    return checker


def bench():
    import importlib.util
    from dishsim_frigidaire.robot import grasp as G
    spec = importlib.util.spec_from_file_location("frigidaire_bench", ROOT / "code/planner/frigidaire/frigidaire_bench.py")
    B = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(B)
    G.pull_counter(B, COUNTER_FRONT_Y)
    return B


def mount_T(m=None):
    m = m or json.loads(MOUNT.read_text())["best"]
    c, s = math.cos(m["yaw_rad"]), math.sin(m["yaw_rad"])
    T = np.eye(4)
    T[:3, :3] = [[c, -s, 0], [s, c, 0], [0, 0, 1]]
    T[:3, 3] = m["base"]
    return T, m


def frames_for(B, rack):
    """Component frames of a cell: the baseline (both racks extended), ``rack`` out at the twin's slide limit (EXTENSION: its
    rear wall flush with the tub's front face, plans/2026-10-29-racks-out.md; the baseline snapshot was recorded at the old
    -0.49 / -0.44 travel, so the rack is moved along y to it), the OTHER rack pushed in; the basket rides on the lower rack."""
    from dishsim_frigidaire.asset import BODY_POSITIONS
    from dishsim_frigidaire.initial_state_candidates import COMPONENT_NAMES
    from dishsim_frigidaire.random_pose_runtime import EXTENSION
    base = json.loads((B.OUT / "baseline" / "result.json").read_text())
    base = base.get("snapshot", base)["poses"]
    frames = {name: base[name] for name in COMPONENT_NAMES}
    other = "UpperRack" if rack == "LowerRack" else "LowerRack"

    def shift(p, dy):
        return {"position_m": [p["position_m"][0], p["position_m"][1] + dy, p["position_m"][2]],
                "quaternion_xyzw": list(p["quaternion_xyzw"])}
    for r, y in ((other, BODY_POSITIONS[other][1]), (rack, BODY_POSITIONS[rack][1] + EXTENSION[rack])):
        dy = y - base[r]["position_m"][1]
        frames[r] = shift(base[r], dy)
        if r == "LowerRack":
            frames["SilverwareBasket"] = shift(base["SilverwareBasket"], dy)
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


def counter_obj(xy=None):
    xy = COUNTER_XY if xy is None else xy           # the spot at call time (--mount's spot_xy moves it)
    T = np.eye(4)
    T[:3, 3] = (xy[0], xy[1], .914 + COUNTER_LIFT_M)
    return T


def cup_down_obj(xy=None):
    """plans/2026-11-04-cups-mouth-down.md: the cup mouth down at the spot, its rim COUNTER_LIFT_M above the slab and its
    base-centred origin on top (= frigidaire_robot_episode.counter_pose(mouth_down=True, kind="cup"))."""
    from dishsim_frigidaire.robot import grasp as G
    xy = COUNTER_XY if xy is None else xy
    T = np.eye(4)
    T[:3, :3] = np.diag([1., -1., -1.])
    T[:3, 3] = (xy[0], xy[1], .914 + COUNTER_LIFT_M + G.KIND_RIM["cup"][2])
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

    def __init__(self, mount=None, basket=None):
        """``mount`` = {"base", "yaw_rad"} (default: the MOUNT file's best); ``basket`` False: no SilverwareBasket in the FCL
        worlds (strip_basket; None = BASKET, use_basket's setting)."""
        from dishsim_frigidaire import planner as P
        from dishsim_frigidaire.robot import collide as C, rig as RG, ur5e
        self.B = B = bench()
        self.C, self.P = C, P
        self.basket = BASKET if basket is None else bool(basket)
        self.T_wb, m = mount_T(mount)
        hulls, X = C.load_model(ARM_MODEL)
        self.T_w3_tcp = RG.T_of(ur5e.T_WRIST3_TCP_POS, ur5e.T_WRIST3_TCP_QUAT_XYZW)

        class KinRig(RG.Rig):                        # the rig's IK/line math without a simulator
            def __init__(s, T_wb):
                s.T_wb, s.T_bw, s.T_w3_tcp = T_wb, np.linalg.inv(T_wb), self.T_w3_tcp

            def ik_all(s, T, seed):
                if not SEED_FREE:
                    return RG.Rig.ik_all(s, T, seed)
                from dishsim_frigidaire.robot import kin   # every analytic branch x every in-limit 2 pi wrap, nearest first
                sols = kin.ik_wrist3_all(s.T_bw @ np.asarray(T) @ np.linalg.inv(s.T_w3_tcp), q_seed=np.asarray(seed))
                ok = [q for q in kin.expand_2pi_wraps(sols, margin=.02, max_out=4096) if s.within_limits(q) and s.branch_ok(q)]
                return sorted(ok, key=lambda q: float(np.abs(q - np.asarray(seed)).max()))
        self.kr = KinRig(self.T_wb)
        self.arms = {}
        for rack in ("LowerRack", "UpperRack"):
            checker = B.bench_checker()
            checker.update_components(frames_for(B, rack))
            strip_basket(checker, self.basket)
            arm = C.ArmWorld(checker, P.slab_body(checker, B.COUNTER), hulls, X, self.T_wb, self.T_w3_tcp)
            bx, by, bz = m["base"]
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


def measured_M(kind, run=None):
    """TCP -> dish after the lift and the 5 s hold, from the grasp-gate trajectories (the trial with the median pivot);
    ``run`` = another gate run's folder (default limits_v1/gate2_<kind>)."""
    from dishsim_frigidaire.robot import rig as RG, ur5e
    run = Path(run) if run else OUT / f"gate2_{kind}"
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
    return np.linalg.inv(T_tcp) @ RG.T_of(d[:3], d[3:7]), {"trial": r["trial"], "pivot_deg": r["hold"]["settle_rotation_deg"],
                                                          "T_tcp": T_tcp.tolist()}


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


def counter_part(W, arm, kind, k, M, xy=None, family="rim"):
    """G1 + G2 of the counter waypoints for grasp k (cached per worker): the lift line carries the dish as grasped
    (it pivots into M only once it is off the counter), the lifted waypoint carries it at the measured M. ``family`` wall =
    the cup wall pinch (grasp.wall_grasps; the open pads straddle the cup over the whole approach, as the episode's pick)."""
    xy = COUNTER_XY if xy is None else xy
    key = (kind, k, id(arm), tuple(xy), family)
    if key in _C:
        return _C[key]
    from dishsim_frigidaire.robot import grasp as G
    T_o = counter_obj(xy)
    g = (G.wall_grasps(T_o, N_YAW, kind) if family == "wall" else G.tilted_grasps(T_o, N_YAW, kind))[k]
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
    qs_a, why = line_ok(W, arm, T_pre, T_g, out["pre-grasp"][0], 60, {}, allow_last=.12 if family == "wall" else .06,
                        dish="dish")
    arm.set_dishes({})
    if qs_a is None:
        out["why"] = f"G2: approach line {why}"
    else:
        qs_l, why = line_ok(W, arm, T_g, T_lift, qs_a[-1], 40, {"held": (kind, np.linalg.inv(T_g) @ T_o, "dish")})
        out.update(ok=qs_l is not None, why=None if qs_l is not None else f"G2: lift line {why}",
                   q_lift=None if qs_l is None else qs_l[-1])
    _C[key] = out
    return out


def grid_spots(B, kind):
    """5 x 5 pick spots around COUNTER_XY: x step GRID_DX, y step so the dish (its visual radius) stays GRID_EDGE_M
    inside the slab's front edge (y = COUNTER centre - half depth, the robot side) at the outer row."""
    from dishsim_frigidaire.loading import visual_points
    r = float(np.linalg.norm(visual_points(B.ASSETS / f"{kind}.usda")[:, :2], axis=1).max())
    edge = B.COUNTER["center_m"][1] - B.COUNTER["size_m"][1] / 2
    dy = min(GRID_DX, (COUNTER_XY[1] - (edge + r + GRID_EDGE_M)) / 2)
    return r, dy, [(i, j, (round(COUNTER_XY[0] + (i - 2) * GRID_DX, 4), round(COUNTER_XY[1] + (j - 2) * dy, 4)))
                   for i in range(5) for j in range(5)]


def grid_job(args):
    """One (kind, spot, yaw) cell of the counter grid: G1/G2 of the pick (pre-grasp, grasp, lift; approach and lift
    lines) with the upper rack pushed in (the grasp gate's rack state)."""
    kind, i, j, xy, k = args
    if "w" not in _W:
        _W["w"] = World()
    W = _W["w"]
    Ms = _W.setdefault("M", {})
    if kind not in Ms:
        Ms[kind] = measured_M(kind)[0]
    t0 = time.monotonic()
    out = counter_part(W, W.arms["LowerRack"], kind, k, Ms[kind], xy=xy)
    return {"kind": kind, "i": i, "j": j, "xy": list(xy), "yaw": k,
            "gate": "G2" if out.get("ok") else "G1" if "lift" in out else "none", "why": out.get("why"),
            "seconds": round(time.monotonic() - t0, 2)}


def grid_main(a):
    from dishsim_frigidaire.robot import grasp as G
    t0 = time.monotonic()
    B = bench()
    kinds = [k for k in KIND_ORDER if k in a.kinds.split(",")]
    meta = {"counter_xy": COUNTER_XY, "dx": GRID_DX, "edge_m": GRID_EDGE_M, "rack_state": "upper rack pushed in",
            "seed_free": SEED_FREE, "kinds": {}}
    jobs = []
    for kind in kinds:
        r, dy, spots = grid_spots(B, kind)
        meta["kinds"][kind] = {"radius_m": round(r, 4), "dy": round(dy, 4), "spots": []}
        n = len(G.tilted_grasps(counter_obj(), N_YAW, kind))
        jobs += [(kind, i, j, xy, k) for i, j, xy in spots for k in range(n)]
    print(f"[INFO] counter grid {len(jobs)} jobs, {a.workers} workers", flush=True)
    rows = []
    import multiprocessing as mp
    with open(a.out / "counter_grid.jsonl", "a") as f, mp.get_context("fork").Pool(a.workers) as pool:
        for r_ in pool.imap(grid_job, jobs, chunksize=1):
            f.write(json.dumps(r_) + "\n")
            f.flush()
            rows.append(r_)
            if len(rows) % 100 == 0:
                print(f"[INFO] {len(rows)}/{len(jobs)} jobs, {time.monotonic() - t0:.0f} s", flush=True)
    rank = {"none": 0, "G1": 1, "G2": 2}
    for kind in kinds:
        _, _, spots = grid_spots(B, kind)
        for i, j, xy in spots:
            rs = [r_ for r_ in rows if r_["kind"] == kind and r_["i"] == i and r_["j"] == j]
            best = max((r_["gate"] for r_ in rs), key=rank.get, default="none")
            meta["kinds"][kind]["spots"].append({"i": i, "j": j, "xy": list(xy), "gate": best,
                                                 "yaws_g2": [r_["yaw"] for r_ in rs if r_["gate"] == "G2"],
                                                 "yaws_g1": [r_["yaw"] for r_ in rs if r_["gate"] in ("G1", "G2")],
                                                 "why_first": next((r_["why"] for r_ in rs if r_["why"]), None)})
        g = [s_["gate"] for s_ in meta["kinds"][kind]["spots"]]
        print(f"[INFO] {kind}: G2 {g.count('G2')}/25, G1+ {g.count('G1') + g.count('G2')}/25", flush=True)
    (a.out / "counter_grid.json").write_text(json.dumps(meta, indent=1) + "\n")
    print(f"[RESULT] PASS counter grid {len(rows)} jobs in {time.monotonic() - t0:.0f} s -> {a.out.relative_to(ROOT)}", flush=True)


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
        out.setdefault(kind, []).append(c)
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
        strip_basket(checkers[rack])
    tilts = tuple(float(x) for x in a.tilts.split(","))
    jobs, poses, leans, meta = [], {}, {}, {"tilts": tilts, "measured_M": {}}
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
            for tilt in tilts:
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


# ------------------------------------------------------------------ window 2: the plate placed from the hand
HOME_TCP = (.25, -.35, 1.25)        # the episode's home TCP (straight down at flags.z_safe_m above the work area)

def inhand_job(args):
    """One (plate gap, yaw) cell, the plate already in the hand at the measured in-hand pose M (no counter grasp): the
    target is the gap's upright family pose turned ``2 pi k / 12`` about the plate's own axis (where the jaws sit on the
    rim); the start is the home TCP on its first IK branch that is free with the plate held (the measured lift pose and
    the home branch nearest HOME_Q are both within the 15 mm margin; the episode spawns at this start); G0 an IK at the
    release; G1 collision-free IK at the pre-place and the release, carried; G2 the RRT start -> pre-place and the
    insertion line."""
    site, k, pose, M = args
    if "w" not in _W:
        _W["w"] = World()
    W = _W["w"]
    from dishsim_frigidaire.robot import grasp as G
    from dishsim_frigidaire.robot.rig import T_of
    arm = W.arms[site["rack"]]
    phi = 2 * np.pi * k / N_YAW
    row = {"kind": "plate", "site": site["site"], "rack": site["rack"], "target": "family-inhand", "tilt_deg": 0., "yaw": k,
           "phi_rad": phi, "gate": "none", "why": None, "grasp_family": "in hand"}
    T_target = T_of(pose["position_m"], pose["quaternion_xyzw"])
    T_target[:3, :3] = T_target[:3, :3] @ rodrigues([0., 0., 1.], phi)
    T_rel = T_target @ np.linalg.inv(M)
    T_rel[2, 3] += RELEASE_ABOVE_M
    T_pp = G.hover_along(T_rel, HOVER_M)
    if not W.kr.ik_all(T_rel, W.seed):
        row["why"] = "G0: no IK at the release pose"
        return row
    row["gate"] = "G0"
    held = {"held": ("plate", M, "dish")}
    arm.set_dishes({})
    starts, _ = free_qs(W, arm, G.tcp_frame(np.array(HOME_TCP), [1., 0., 0.]), held)
    if not starts:
        row["why"] = "G1: no home IK branch is free with the plate in hand"
        return row
    q_home = starts[0]
    row["q_start"] = q_home.tolist()
    sol = {}
    for name, T in (("pre-place", T_pp), ("release", T_rel)):
        qs, n = free_qs(W, arm, T, held)
        if not qs:
            any_q = W.kr.ik_all(T, W.seed)
            row["why"] = (f"G1: no collision-free IK at the {name} ({n} IK solutions; first hits "
                          f"{arm.collisions(any_q[0], first=False, **held)[:2] if any_q else None})")
            return row
        sol[name] = qs
    row.update(gate="G1", branch=branch(sol["release"][0]))
    best = None
    for q_pp in sol["pre-place"][:2]:
        qs_i, why = line_ok(W, arm, T_pp, T_rel, q_pp, 40, held)
        if qs_i is None:
            best = best or f"G2: insertion line {why}"
            continue
        path = W.C.rrt_connect(arm, q_home, [q_pp], time_s=BUDGET_S, collide_kw=held, seed=1 + k)
        if path is None:
            best = "G2: budget (RRT-Connect found no path in %.0f s)" % BUDGET_S
            continue
        row.update(gate="G2", branch=branch(qs_i[-1]), why=None)
        break
    if row["gate"] != "G2":
        row["why"] = best
    return row


def inhand_main(a):
    t0 = time.monotonic()
    reach = json.loads((OUT / "reach.json").read_text())
    M, src = measured_M("plate")
    gaps = reach["sites"]["plate"]
    jobs = [(s, k, s["world"]["family"], M) for s in gaps for k in range(N_YAW)]
    print(f"[INFO] plate in hand: {len(gaps)} gaps x {N_YAW} yaws = {len(jobs)} jobs, M from gate2_plate {src}", flush=True)
    rows = []
    import multiprocessing as mp
    with open(a.out / "inhand.jsonl", "a") as f, mp.get_context("fork").Pool(a.workers) as pool:
        for r in pool.imap(inhand_job, jobs, chunksize=1):
            rows.append(r)
            f.write(json.dumps(r) + "\n")
            f.flush()
    tab = orient_table(rows, {("plate", s["site"], 0.): s["world"]["family"] for s in gaps}, {})
    q_start = next((r["q_start"] for r in rows if r.get("q_start")), None)
    (a.out / "inhand.json").write_text(json.dumps({"target": "family-inhand", "home_tcp": HOME_TCP, "q_start": q_start,
                                                   "measured_M": {"plate": {"M": M.tolist(), **src}}, "cells": tab}) + "\n")
    g2 = [c["site"] for c in tab["plate"] if c["gate"] == "G2"]
    print(f"[RESULT] PASS plate in hand {len(rows)} jobs in {time.monotonic() - t0:.0f} s: G2 at {len(g2)}/{len(gaps)} gaps {g2}",
          flush=True)


# ------------------------------------------------------------------ the cup side pinch (plans/2026-10-26-side-pinch.md)
PLACING = ROOT / "data/results/robot/placing_v1"
YAW_NOTE = ("yaw k = the approach direction 2 pi k / 12 around the cup: at the counter grasp.side_grasps' yaw k; at the goal the "
            "cup held at its height's hold turned 2 pi k / 12 about its own axis (the symmetric cup and the tilt-only "
            "tolerance allow any turn), so a height's counter pick and a goal yaw combine freely; the wrist side (the 180 deg "
            "roll about the approach, grasp.WRIST_SIDES) is fixed in the cup's frame by the hold: both = some counter yaw "
            "passes at that height and wrist side AND this goal yaw passes; both_same_yaw = the same k passes at both")


def rot_z(phi):
    T = np.eye(4)
    T[:3, :3] = rodrigues([0., 0., 1.], phi)
    return T


def side_M(M0, k, side):
    """The hold at yaw k and wrist side ``side`` from the yaw-0 "mouth" hold M0: the cup turned about its own axis (right),
    the TCP rolled 180 deg about the approach for "base" (left; the gripper is symmetric)."""
    return (rot_z(np.pi) if side == "base" else np.eye(4)) @ np.asarray(M0) @ rot_z(-2 * np.pi * k / N_YAW)


def side_job(args):
    """One cup: S1 / S2 of every (height, wrist side, yaw) on the arm model, the earlier cups at their goals, the lower
    rack in; yaw k turns the goal 2 pi k / 12 about the cup's own axis, held at the yaw-0 hold of that wrist side."""
    d, earlier, holds = args
    if "w" not in _W:
        _W["w"] = World()
    W = _W["w"]
    from dishsim_frigidaire.robot import grasp as G
    from dishsim_frigidaire.robot.rig import T_of
    sys.path.insert(0, str(ROOT / "code/execution/frigidaire"))
    import frigidaire_robot_episode as EP                 # constants only (stdlib at module level)
    arm = W.arms[d["rack"]]
    arm.set_dishes({e["name"]: arm.checker._body("cup", {"position_m": e["goal_pose"]["pos"],
                                                        "quaternion_xyzw": e["goal_pose"]["quat_xyzw"]}, e["name"]) for e in earlier})
    T_goal = T_of(d["goal_pose"]["pos"], d["goal_pose"]["quat_xyzw"])
    q_home = W.kr.ik(G.tcp_frame(np.array(HOME_TCP), [1., 0., 0.]), np.array([0.0, -1.712, 1.712, -1.571, -1.571, 0.0]))
    out = []
    for (h, side), hold in holds.items():
        M = side_M(hold["M"], 0, side)
        held = {"held": ("cup", M, "dish")}
        # the episode's start: the home TCP on its first IK branch free with the cup held (reset() in place_goal_trials)
        seed = q_home if arm.free(q_home, **held) else next(
            (q for q in W.kr.ik_all(G.tcp_frame(np.array(HOME_TCP), [1., 0., 0.]), q_home) if arm.free(q, **held)), q_home)
        for k in range(N_YAW):
            T_g = T_goal @ rot_z(2 * np.pi * k / N_YAW)
            c = {"below_top_m": float(h), "wrist_side": side, "yaw_k": k, "s1": False, "s2": False, "why": None}
            for r in EP.RAISES_M:
                T_d = T_g.copy()
                T_d[2, 3] += r
                if free_qs(W, arm, T_d @ np.linalg.inv(M), held, seed=seed)[0]:
                    c.update(s1=True, s1_raise_m=r)
                    break
            if not c["s1"]:
                T_d = T_g.copy()
                T_d[2, 3] += EP.RAISES_M[0]
                qs = W.kr.ik_all(T_d @ np.linalg.inv(M), seed)
                c["hits"] = sorted({b for _, b in arm.collisions(qs[0], first=False, **held)}) if qs else []
                c["why"] = (f"S1: no collision-free IK with the cup at its goal orientation 0.10-0.35 m above its goal "
                            f"(+0.10 m: {len(qs)} IK solutions, first hits {arm.collisions(qs[0], first=False, **held)[:3] if qs else None})")
                out.append(c)
                continue
            T_rel = T_g @ np.linalg.inv(M)
            T_rel[2, 3] += RELEASE_ABOVE_M
            T_pp = T_rel.copy()
            T_pp[2, 3] += HOVER_M
            q_pp, q_rel = free_qs(W, arm, T_pp, held, seed=seed)[0], free_qs(W, arm, T_rel, held, seed=seed)[0]
            c.update(n_free_preplace=len(q_pp), n_free_release=len(q_rel), s2=bool(q_pp and q_rel))
            if not c["s2"]:
                bad = [(w, T) for w, q_, T in (("pre-place", q_pp, T_pp), ("release", q_rel, T_rel)) if not q_]
                hits = set()
                for _, T in bad:
                    qs = W.kr.ik_all(T, seed)
                    hits |= {b for _, b in arm.collisions(qs[0], first=False, **held)} if qs else set()
                c["hits"] = sorted(hits)
                c["why"] = "S2: " + "; ".join(f"no collision-free IK at the {w}" for w, _ in bad)
            out.append(c)
    arm.set_dishes({})
    return d["name"], out


def side_pick(W, arm, dish, g, M, q_home):
    """One side-pinch candidate's counter pick on the arm model: a collision-free pre-grasp hover IK, the approach line
    (the open pads may straddle the cup all along), the 0.15 m lift line with the cup held at M. Leaves the cup body set."""
    from dishsim_frigidaire.robot import grasp as G
    sys.path.insert(0, str(ROOT / "code/execution/frigidaire"))
    import frigidaire_robot_episode as EP
    T_g, H = g["T_tcp"], G.hover_along(g["T_tcp"], HOVER_M)
    T_up = G.hover(T_g, LIFT_M)
    r = {"ok": False, "why": None}
    arm.set_dishes({"dish": dish})
    hov, _ = free_qs(W, arm, H, {}, seed=q_home)
    r["n_free_hover"] = len(hov)
    if not hov:
        qs = W.kr.ik_all(H, q_home)
        r["why"] = f"no collision-free IK at the pre-grasp hover ({len(qs)} IK, first hits {arm.collisions(qs[0], first=False)[:3] if qs else None})"
    for q_h in hov[:4]:
        qs, why = line_ok(W, arm, H, T_g, q_h, 100, {}, allow_last=EP.ALLOW_STRADDLE_M, dish="dish")
        if qs is None:
            r["why"] = f"approach line {why}"
            continue
        arm.set_dishes({})
        up, why = line_ok(W, arm, T_g, T_up, qs[-1], 40, {"held": ("cup", M, "dish")})
        arm.set_dishes({"dish": dish})
        if up is None:
            r["why"] = f"lift line {why}"
            continue
        r.update(ok=True, why=None, q_hover=[round(float(v), 5) for v in q_h], q_lift=[round(float(v), 5) for v in up[-1]])
        break
    return r


# ------------------------------------------------------------------ plans/2026-10-27-cup-side-drop.md: the base-side spot
SPOT_X, SPOT_BELOW_TOP = .20, .045
SPOT_BEHIND_EDGE_M = tuple(round(.05 + .01 * i, 3) for i in range(8))   # y = the front edge + 0.05 ... + 0.12 m


def home_q(W):
    from dishsim_frigidaire.robot import grasp as G
    return W.kr.ik(G.tcp_frame(np.array(HOME_TCP), [1., 0., 0.]), np.array([0.0, -1.712, 1.712, -1.571, -1.571, 0.0]))


def spot_job(args):
    """One (y, yaw) cell: the base-side pinch at SPOT_BELOW_TOP picked from (SPOT_X, y) at its nominal hold."""
    y, k = args
    if "w" not in _W:
        _W["w"] = World()
    W = _W["w"]
    from dishsim_frigidaire.robot import grasp as G
    arm = W.arms["LowerRack"]                             # the grasp gate's racks: the upper rack pushed in
    T_o = counter_obj((SPOT_X, y))
    g = G.side_grasps(T_o, N_YAW, SPOT_BELOW_TOP, wrist_side="base")[k]
    r = side_pick(W, arm, arm.checker._body("cup", W.P.pose_dict(T_o), "dish"), g, np.linalg.inv(g["T_tcp"]) @ T_o, home_q(W))
    arm.set_dishes({})
    return {"y": y, "yaw_k": k, **r}


def preflight_job(args):
    """One easy_cups cup (information): per drop height, the yaws with a collision-free IK of the drop pose (the hold
    flipped, the cup centroid over the goal centroid), the earlier cups at their goals, the lower rack pushed in."""
    d, earlier, M, R0 = args
    if "w" not in _W:
        _W["w"] = World()
    W = _W["w"]
    from dishsim_frigidaire.loading import visual_points
    from dishsim_frigidaire.robot import grasp as G
    from dishsim_frigidaire.robot.rig import T_of
    sys.path.insert(0, str(ROOT / "code/execution/frigidaire"))
    import frigidaire_robot_episode as EP
    arm = W.arms[d["rack"]]
    arm.set_dishes({e["name"]: arm.checker._body("cup", {"position_m": e["goal_pose"]["pos"],
                                                        "quaternion_xyzw": e["goal_pose"]["quat_xyzw"]}, e["name"]) for e in earlier})
    held = {"held": ("cup", M, "dish")}
    q0 = home_q(W)
    seed = q0 if arm.free(q0, **held) else next(
        (q for q in W.kr.ik_all(G.tcp_frame(np.array(HOME_TCP), [1., 0., 0.]), q0) if arm.free(q, **held)), q0)
    c = np.r_[visual_points(W.B.ASSETS / "cup.usda").mean(axis=0), 1.]
    g_c, m_c = (T_of(d["goal_pose"]["pos"], d["goal_pose"]["quat_xyzw"]) @ c)[:3], (M @ c)[:3]
    Rf = EP.flip_down(R0, np)
    n = [sum(bool(free_qs(W, arm, EP.drop_tcp(Rf, m_c, g_c, h, k, np), held, seed=seed)[0]) for k in range(N_YAW))
         for h in EP.DROP_HEIGHTS_M]
    arm.set_dishes({})
    low = next((h for h, x in zip(EP.DROP_HEIGHTS_M, n) if x), None)
    return d["name"], {"slot": d["slot"], "drop_ik": low is not None, "lowest_m": low, "yaws_per_height": n}


def spot_main(a):
    """--side-spot: the base-side pick at x SPOT_X and y = the front edge + 0.05 ... + 0.12 m (12 yaws each); the spot is the
    first y (nearest the edge) with the most passing yaws. Then the drop-IK pre-flight per easy_cups cup with the nominal
    hold of the best-facing passing yaw there (the episode's pick order). Writes placing_v1/side/spot.json."""
    t0 = time.monotonic()
    from collections import Counter
    import multiprocessing as mp
    from dishsim_frigidaire.robot import grasp as G
    sys.path.insert(0, str(ROOT / "code/execution/frigidaire"))
    import frigidaire_robot_episode as EP
    B = bench()
    edge = B.COUNTER["center_m"][1] - B.COUNTER["size_m"][1] / 2
    ys = [round(edge + dy, 3) for dy in SPOT_BEHIND_EDGE_M]
    with mp.get_context("fork").Pool(a.workers) as pool:
        rows = list(pool.imap(spot_job, [(y, k) for y in ys for k in range(N_YAW)], chunksize=1))
    per = {y: [r for r in rows if r["y"] == y] for y in ys}
    npass = {y: sum(r["ok"] for r in rs) for y, rs in per.items()}
    y_spot = next(y for y in ys if npass[y] == max(npass.values()))
    for y in ys:
        whys = Counter(str(r["why"]).split(" (")[0].split(":")[0] for r in per[y] if not r["ok"])
        print(f"[INFO] y {y:+.3f}: {npass[y]}/{N_YAW} yaws pass {[r['yaw_k'] for r in per[y] if r['ok']]} {dict(whys)}", flush=True)
    T_o = counter_obj((SPOT_X, y_spot))
    gs = G.side_grasps(T_o, N_YAW, SPOT_BELOW_TOP, wrist_side="base")
    to_base = np.asarray(mount_T()[1]["base"][:2]) - T_o[:2, 3]
    ok = [r["yaw_k"] for r in per[y_spot] if r["ok"]] or list(range(N_YAW))
    k_best = max(ok, key=lambda k: float(np.dot(gs[k]["T_tcp"][:2, 3] - T_o[:2, 3], to_base)))
    M, R0 = np.linalg.inv(gs[k_best]["T_tcp"]) @ T_o, gs[k_best]["T_tcp"][:3, :3]
    ds = json.loads(a.side_load.read_text())["dishes"]
    with mp.get_context("fork").Pool(a.workers) as pool:
        pre = dict(pool.imap(preflight_job, [(d, ds[:i], M, R0) for i, d in enumerate(ds)], chunksize=1))
    for n_, v in pre.items():
        print(f"[INFO] pre-flight {n_} {v['slot']}: drop IK {v['drop_ik']} lowest {v['lowest_m']} yaws/height {v['yaws_per_height']}",
              flush=True)
    res = {"plan": "plans/2026-10-27-cup-side-drop.md", "counter_front_y": COUNTER_FRONT_Y, "x": SPOT_X,
           "below_top_m": SPOT_BELOW_TOP, "wrist_side": "base", "ys": ys, "spot_xy": [SPOT_X, y_spot],
           "rule": "the first y (from the edge back) with the most yaws passing hover IK, approach line and lift line",
           "per_y": {f"{y:.3f}": {"pass": npass[y], "rows": per[y]} for y in ys},
           "preflight": {"hold": "nominal base-side hold of yaw %d at the spot (the best-facing passing yaw, else the best-facing "
                                 "of all 12)" % k_best,
                         "earlier_cups": "at their goals", "heights_m": list(EP.DROP_HEIGHTS_M),
                         "cups": pre},
           "seconds": round(time.monotonic() - t0, 1)}
    a.spot_out.parent.mkdir(parents=True, exist_ok=True)
    a.spot_out.write_text(json.dumps(res, indent=1) + "\n")
    print(f"[RESULT] PASS side spot y {y_spot:+.3f} ({npass[y_spot]}/{N_YAW} yaws); pre-flight drop IK at "
          f"{sum(v['drop_ik'] for v in pre.values())}/{len(pre)} cups in {res['seconds']} s -> {a.spot_out.relative_to(ROOT)}",
          flush=True)


def side_main(a):
    """Kit-free candidates of the side pinch: the counter pick per (height, yaw) (pre-grasp hover, approach line, lift
    0.15 m with the cup held; the gate's racks), then per easy_cups cup S1 / S2 per (height, yaw) with each height's hold
    (the median hold of placing_v1/partA_cup_side_h<mm> when that gate held, else the nominal geometric hold)."""
    from dishsim_frigidaire.robot import grasp as G
    sys.path.insert(0, str(ROOT / "code/execution/frigidaire"))
    import frigidaire_robot_episode as EP
    t0 = time.monotonic()
    W = World()
    arm = W.arms["LowerRack"]                             # the grasp gate's rack state: the upper rack pushed in
    T_o = counter_obj()
    dish = arm.checker._body("cup", W.P.pose_dict(T_o), "dish")
    q_home = W.kr.ik(G.tcp_frame(np.array(HOME_TCP), [1., 0., 0.]), np.array([0.0, -1.712, 1.712, -1.571, -1.571, 0.0]))
    holds, pick = {}, {}
    for h in G.SIDE_BELOW_TOP_M:
        gs = G.side_grasps(T_o, N_YAW, h)
        run = PLACING / f"partA_cup_side_h{round(h * 1e3):02d}"
        rows = [json.loads(l) for l in (run / "trials.jsonl").read_text().splitlines() if l.strip()] if (run / "trials.jsonl").is_file() else []
        if any(r["g3"] and r.get("traj") for r in rows):
            M0, src = measured_M("cup", run)
            r = next(r for r in rows if r["trial"] == src["trial"])
            k0, s0 = int(round(r["grasp"]["yaw"] / (2 * np.pi / N_YAW))) % N_YAW, r["grasp"].get("wrist_side", "mouth")
            hold = {"measured": True, "gate_run": str(run.relative_to(ROOT)), "trial": src["trial"], "pivot_deg": src["pivot_deg"],
                    "gate_yaw_k": k0, "gate_wrist_side": s0, "held": f"{sum(bool(x['g3']) for x in rows)}/{len(rows)}"}
        else:
            k0, s0, M0 = 0, "mouth", np.linalg.inv(gs[0]["T_tcp"]) @ T_o
            hold = {"measured": False, "gate_run": str(run.relative_to(ROOT)) if rows else None, "trial": None, "pivot_deg": None,
                    "gate_yaw_k": None, "gate_wrist_side": None, "held": f"0/{len(rows)}" if rows else "not run"}
        # the yaw-0 "mouth" hold: undo the gate's roll (left) and turn the cup back to yaw 0 (right)
        hold["M"] = (rot_z(np.pi) if s0 == "base" else np.eye(4)) @ M0 @ rot_z(2 * np.pi * k0 / N_YAW)
        holds[h] = hold
        res = []
        for k, g in ((k, g) for side in G.WRIST_SIDES for k, g in enumerate(G.side_grasps(T_o, N_YAW, h, wrist_side=side))):
            res.append({"yaw_k": k, "wrist_side": g["wrist_side"],
                        **side_pick(W, arm, dish, g, side_M(hold["M"], k, g["wrist_side"]), q_home)})
        arm.set_dishes({})
        pick[h] = res
        print(f"[INFO] counter pick, pads {h * 1e3:.0f} mm under the rim: {sum(x['ok'] for x in res)}/{len(res)} (yaw, wrist side) "
              f"{[(x['yaw_k'], x['wrist_side']) for x in res if x['ok']]}; hold {hold['held']} measured {hold['measured']}", flush=True)
    load = json.loads(a.side_load.read_text())
    ds = load["dishes"]
    jobs = [(d, ds[:i], {(h, side): {"M": v["M"].tolist()} for h, v in holds.items() for side in G.WRIST_SIDES})
            for i, d in enumerate(ds)]
    import multiprocessing as mp
    with mp.get_context("fork").Pool(a.workers) as pool:
        goal = dict(pool.imap(side_job, jobs, chunksize=1))
    from collections import Counter
    cups = {}
    for i, d in enumerate(ds):
        cs = goal[d["name"]]
        for c in cs:
            h, side = c["below_top_m"], c["wrist_side"]
            c["pick_same_yaw"] = any(x["ok"] for x in pick[h] if x["yaw_k"] == c["yaw_k"] and x["wrist_side"] == side)
            c["pick"] = any(x["ok"] for x in pick[h] if x["wrist_side"] == side)   # some counter yaw (the cup turns at the goal)
            c["both"] = c["pick"] and c["s1"] and c["s2"]
        ok = [c for c in cs if c["both"]]
        summ = {"tried": len(cs), "pass_pick": sum(c["pick"] for c in cs), "pass_goal": sum(c["s1"] and c["s2"] for c in cs),
                "pass_both": len(ok), "pass_both_same_yaw": sum(c["both"] and c["pick_same_yaw"] for c in cs),
                "pass_s1": sum(c["s1"] for c in cs)}
        best, failed_at, why = None, None, None
        if ok:
            c = max(ok, key=lambda c: (holds[c["below_top_m"]]["measured"], c["n_free_release"] + c["n_free_preplace"], -c["yaw_k"]))
            hold = holds[c["below_top_m"]]
            phi = 2 * np.pi * c["yaw_k"] / N_YAW
            from dishsim_frigidaire.robot.rig import T_of
            best = {"below_top_m": c["below_top_m"], "yaw_k": c["yaw_k"], "wrist_side": c["wrist_side"],
                    "goal_turn_deg": round(float(np.degrees(phi)), 1), "M": side_M(hold["M"], 0, c["wrist_side"]).tolist(),
                    "T_goal": (T_of(d["goal_pose"]["pos"], d["goal_pose"]["quat_xyzw"]) @ rot_z(phi)).tolist(),
                    "gate_run": hold["gate_run"], "trial": hold["trial"], "pivot_deg": hold["pivot_deg"], "measured": hold["measured"],
                    "s1_raise_m": c["s1_raise_m"], "n_free_preplace": c["n_free_preplace"], "n_free_release": c["n_free_release"]}
        else:
            pk = [c for c in cs if c["pick"]]
            hs = sorted({f"{round(c['below_top_m'] * 1e3)} ({c['wrist_side']})" for c in pk})
            step = "S1" if not any(c["s1"] for c in pk) else "S2"
            bad = [c for c in pk if (not c["s1"]) == (step == "S1") and not (c["s1"] and c["s2"])]
            hits = Counter(b for c in bad for b in c.get("hits", []))
            waived = [c for c in cs if c["s1"] and c["s2"]]
            failed_at = step
            why = ((f"no side-pinch candidate is collision-free at the counter pick ({len(cs)} candidates)" if not pk else
                    f"none of the {len(pk)} candidates with a collision-free counter pick (pads {', '.join(hs)} mm under "
                    f"the rim, wrist side, x {N_YAW} yaws) has a collision-free IK " + (
                        "with the cup at its goal orientation 0.10-0.35 m above its goal" if step == "S1" else
                        "at the pre-place and the release (S1 passed at " + f"{sum(c['s1'] for c in pk)})"))
                   + f"; commonest hits {hits.most_common(4)}"
                   + (f"; with the counter pick waived {len(waived)} candidates pass S1 and S2 (pads "
                      f"{', '.join(sorted({str(round(c['below_top_m'] * 1e3)) + ' mm ' + c['wrist_side'] for c in waived}))})"
                      if waived else ""))
        cups[d["name"]] = {"order_index": i, "slot": d["slot"], "rack": d["rack"], "summary": summ, "best": best,
                           "failed_at": failed_at, "why": why, "candidates": cs}
        print(f"[INFO] {d['name']} {d['slot']}: {summ} best {None if best is None else (best['below_top_m'], best['wrist_side'], best['yaw_k'])} "
              f"{failed_at or ''} {str(why or '')[:160]}", flush=True)
    res = {"plan": "plans/2026-10-26-side-pinch.md", "load": str(a.side_load.relative_to(ROOT)), "counter_xy": COUNTER_XY,
           "pick_racks": "upper rack pushed in (the grasp gate's)", "goal_racks": "lower rack pushed in (an upper-rack goal)",
           "yaw_note": YAW_NOTE, "heights_m": list(G.SIDE_BELOW_TOP_M), "axis_beyond_tcp_m": G.SIDE_AXIS_BEYOND_TCP_M,
           "holds": {f"{h:.3f}": {**v, "M": v["M"].tolist()} for h, v in holds.items()},
           "wrist_sides": list(G.WRIST_SIDES), "pick": {f"{h:.3f}": v for h, v in pick.items()}, "cups": cups,
           "seconds": round(time.monotonic() - t0, 1)}
    a.side_out.parent.mkdir(parents=True, exist_ok=True)
    a.side_out.write_text(json.dumps(res, indent=1) + "\n")
    print(f"[RESULT] PASS side candidates {sum(c['best'] is not None for c in cups.values())}/{len(cups)} cups with a candidate "
          f"in {res['seconds']} s -> {a.side_out.relative_to(ROOT)}", flush=True)


# ------------------------------------------------------------------ plans/2026-10-27-posture-fix.md: the mount search
MOUNT_XS, MOUNT_YS, MOUNT_ZS = (.45, .60), (-.55, -.70, -.85), (.45, .60, .75, .90)
TODAY_MOUNT = {"base": [.45, -.55, .90], "yaw_rad": 2.60}         # data/results/robot/mount/easy_s0_any.json (the baseline)
SEARCH_LOADS = ("easy_bowls", "easy_plates", "easy_cups")
SEARCH_CAP_S = 1800.                                               # the plan's 30 min cap: the best so far wins
MOUNT_OUT = ROOT / "data/results/robot/mount/posture_v1.json"
SEARCH_RULE = ("score = the 23 easy goals with an un-wound elbow-up collision-free drop IK at any height (25-350 mm) and yaw "
               "(12; the earlier dishes of the load at their goals, the other rack pushed in; early exit) + the two measured "
               "counter picks at the spot (bowl rim pinch, cup wall pinch: a yaw with collision-free hover, grasp and lift IK "
               "and free approach and lift lines); ties: the cup base-side pick passes a yaw, then the lowest pedestal; only "
               "mounts where the episode can start (an elbow-up home IK free with the cup held) are picked")


def search_holds():
    """{kind: (M, R0, source)}: the drop test's holds (bowl gate2 rim pinch, plate gate2 assumed rim hold, TCP orientation
    as held) and round 6's cup hold (the h45 mouth hold rolled 180 deg, flipped mouth down for the drop)."""
    sys.path.insert(0, str(ROOT / "code/execution/frigidaire"))
    import frigidaire_robot_episode as EP
    out = {}
    for kind in ("bowl", "plate"):
        M, src = measured_M(kind)
        out[kind] = (M, np.asarray(src["T_tcp"])[:3, :3], f"limits_v1/gate2_{kind} median hold ({src['trial']}, "
                                                            f"pivot {src['pivot_deg']} deg)")
    M0, src = measured_M("cup", PLACING / EP.SIDE_MOUTH_RUN)
    out["cup"] = (side_M(M0, 0, "base"), EP.flip_down(np.asarray(src["T_tcp"])[:3, :3], np),
                  f"{EP.SIDE_MOUTH_RUN} median hold ({src['trial']}) rolled 180 deg about the approach, flipped mouth down")
    return out


def any_free(W, arm, T, kw, seed):
    return next((q for q in W.kr.ik_all(T, seed) if arm.free(q, **kw)), None)


def mount_job(args):
    """One mount: the episode's start, the 23 drop IKs, the 2 measured counter picks, the base-side pre-check."""
    m, holds, loads, M_wall = args
    t0 = time.monotonic()
    _C.clear()                                           # counter_part's cache keys on id(arm): a new World per mount
    W = World(m)
    from dishsim_frigidaire.robot import grasp as G
    from dishsim_frigidaire.robot.rig import T_of
    sys.path.insert(0, str(ROOT / "code/execution/frigidaire"))
    import frigidaire_robot_episode as EP
    deg = lambda q: [round(float(v), 1) for v in np.degrees(q)]
    row = {**m, "status": "done"}
    T_home = G.tcp_frame(np.array(HOME_TCP), [1., 0., 0.])
    q_home = home_q(W)                                   # rig.ik: elbow up, un-wound, nearest HOME_Q

    def start(kind, arm):
        held = {"held": (kind, holds[kind][0], "dish")}
        if q_home is None:
            return None
        return q_home if arm.free(q_home, **held) else any_free(W, arm, T_home, held, q_home)
    q_cup = start("cup", W.arms["UpperRack"])
    row.update(home_ik=q_home is not None, home_deg=None if q_home is None else deg(q_home),
               start_free_with_cup=q_cup is not None, eligible=q_cup is not None)
    cent = W.B.kind_centroids({k: visual_points_of(W.B, k) for k in ("bowl", "plate", "cup")})
    goals = {}
    for L, ds in loads:
        for i, d in enumerate(ds):
            kind = d["kind"]
            M, R0 = np.asarray(holds[kind][0]), np.asarray(holds[kind][1])
            arm = W.arms[d["rack"]]
            arm.set_dishes({e["name"]: arm.checker._body(kind, {"position_m": e["goal_pose"]["pos"],
                                                               "quaternion_xyzw": e["goal_pose"]["quat_xyzw"]}, e["name"])
                            for e in ds[:i]})
            held = {"held": (kind, M, "dish")}
            seed = start(kind, arm)
            seed = q_home if seed is None else seed
            c = np.r_[cent[kind], 1.]
            g_c, m_c = (T_of(d["goal_pose"]["pos"], d["goal_pose"]["quat_xyzw"]) @ c)[:3], (M @ c)[:3]
            hit = None
            if seed is not None:
                for h in EP.DROP_HEIGHTS_M:
                    for k in range(N_YAW):
                        q = any_free(W, arm, EP.drop_tcp(R0, m_c, g_c, h, k, np), held, seed)
                        if q is not None:
                            hit = (h, k, q)
                            break
                    if hit:
                        break
            arm.set_dishes({})
            goals[d["name"]] = {"load": L, "rack": d["rack"], "slot": d["slot"], "ok": hit is not None,
                                "height_m": hit[0] if hit else None, "yaw_k": hit[1] if hit else None,
                                "max_abs_deg": round(float(np.degrees(np.abs(hit[2]).max())), 1) if hit else None}
    arm = W.arms["LowerRack"]                            # the grasp gates' racks: the upper rack pushed in
    picks = {}
    for name, kind, fam, M in (("bowl rim pinch", "bowl", "rim", holds["bowl"][0]), ("cup wall pinch", "cup", "wall", M_wall)):
        T_o = counter_obj(COUNTER_XY)
        n = len(G.wall_grasps(T_o, N_YAW, kind) if fam == "wall" else G.tilted_grasps(T_o, N_YAW, kind))
        k_ok = next((k for k in range(n) if counter_part(W, arm, kind, k, np.asarray(M), COUNTER_XY, fam)["ok"]), None)
        picks[name] = {"ok": k_ok is not None, "yaw_k": k_ok}
        arm.set_dishes({})
    T_o = counter_obj(COUNTER_XY)
    cup = arm.checker._body("cup", W.P.pose_dict(T_o), "dish")
    base_ok = [k for k, g in enumerate(G.side_grasps(T_o, N_YAW, SPOT_BELOW_TOP, wrist_side="base"))
               if side_pick(W, arm, cup, g, np.linalg.inv(g["T_tcp"]) @ T_o, W.seed if q_home is None else q_home)["ok"]]
    arm.set_dishes({})
    n_goal = {L: f"{sum(goals[d['name']]['ok'] for d in ds)}/{len(ds)}" for L, ds in loads}
    row.update(goals_ok=sum(v["ok"] for v in goals.values()), goals_n=len(goals), goals_per_load=n_goal,
               picks=picks, picks_ok=sum(v["ok"] for v in picks.values()), base_side_yaws=base_ok,
               score=sum(v["ok"] for v in goals.values()) + sum(v["ok"] for v in picks.values()), goals=goals,
               seconds=round(time.monotonic() - t0, 1))
    return row


def visual_points_of(B, kind):
    from dishsim_frigidaire.loading import visual_points
    return visual_points(B.ASSETS / f"{kind}.usda")


def mount_search_main(a):
    """--mount-search: the 24 grid mounts (yaw facing the work centre: the mean x, y of the 23 goals and the 2 picks) and
    today's mount, lowest pedestal first, scored by SEARCH_RULE on --workers processes; at the cap the best so far wins
    and the skipped mounts are marked. Writes data/results/robot/mount/posture_v1.json (the episode's --mount)."""
    import multiprocessing as mp
    t0 = time.monotonic()
    loads = [(L, json.loads((PLACING / L / "load.json").read_text())) for L in SEARCH_LOADS]
    loads = [(L, sorted(ld["dishes"], key=lambda x: x["order_index"])) for L, ld in loads]
    pts = [d["goal_pose"]["pos"][:2] for _, ds in loads for d in ds] + [list(COUNTER_XY)] * 2
    cx, cy = (float(v) for v in np.mean(pts, axis=0))
    mounts = []
    for z in MOUNT_ZS:
        if z == TODAY_MOUNT["base"][2]:
            mounts.append({"base": list(TODAY_MOUNT["base"]), "yaw_rad": TODAY_MOUNT["yaw_rad"], "baseline": True})
        for x in MOUNT_XS:
            for y in MOUNT_YS:
                mounts.append({"base": [x, y, z], "yaw_rad": round(math.atan2(cy - y, cx - x), 4), "baseline": False})
    for i, m in enumerate(mounts):
        m["visit"] = i
    holds = search_holds()
    M_wall, src_w = measured_M("cup", PLACING / "partA_cup_wall")
    hold_src = {k: v[2] for k, v in holds.items()}
    hold_src["cup wall pinch (pick)"] = f"partA_cup_wall median hold ({src_w['trial']})"
    jobs = [(m, {k: (np.asarray(v[0]), np.asarray(v[1])) for k, v in holds.items()}, loads, np.asarray(M_wall)) for m in mounts]
    print(f"[INFO] mount search: {len(mounts)} mounts, work centre ({cx:.3f}, {cy:.3f}), {a.workers} workers, cap "
          f"{SEARCH_CAP_S:.0f} s; holds {hold_src}", flush=True)

    def done_(r):
        print(f"[INFO] mount {r['visit']:2d} {r['base']} yaw {r['yaw_rad']:.3f}{' (today)' if r['baseline'] else ''}: score "
              f"{r['score']} (goals {r['goals_ok']}/{r['goals_n']} {r['goals_per_load']}, picks {r['picks_ok']}/2), base-side "
              f"yaws {r['base_side_yaws']}, start free {r['start_free_with_cup']}, {r['seconds']} s, "
              f"{time.monotonic() - t0:.0f} s in", flush=True)

    def err_(e):
        print(f"[ERROR] mount job: {e!r} {getattr(e, '__cause__', None)}", flush=True)
    pool = mp.get_context("fork").Pool(a.workers)
    res = [pool.apply_async(mount_job, (j,), callback=done_, error_callback=err_) for j in jobs]
    while time.monotonic() - t0 < SEARCH_CAP_S and not all(r.ready() for r in res):
        time.sleep(5)
    rows = []
    for m, r in zip(mounts, res):
        if not r.ready():
            rows.append({**m, "status": f"skipped ({SEARCH_CAP_S / 60:.0f} min cap)"})
            continue
        try:
            rows.append(r.get())
        except Exception as e:                           # noqa: BLE001
            rows.append({**m, "status": f"error: {e!r}"})
    pool.terminate()
    ok = [r for r in rows if r["status"] == "done" and r["eligible"]]
    if not ok:
        raise SystemExit(f"[RESULT] FAIL mount search: no scored eligible mount ({[r['status'] for r in rows]})")
    best = max(ok, key=lambda r: (r["score"], bool(r["base_side_yaws"]), -r["base"][2], -r["visit"]))
    for r in rows:
        r["picked"] = r is best
    base = next(r for r in rows if r["baseline"])
    res = {"plan": "plans/2026-10-27-posture-fix.md", "rule": SEARCH_RULE, "work_centre": [round(cx, 4), round(cy, 4)],
           "grid": {"x": MOUNT_XS, "y": MOUNT_YS, "z": MOUNT_ZS, "yaw": "facing the work centre"},
           "baseline": {k: base.get(k) for k in ("base", "yaw_rad", "status", "score", "base_side_yaws")},
           "counter_xy": list(COUNTER_XY), "counter_front_y": COUNTER_FRONT_Y, "holds": hold_src,
           "heights_m": None, "cap_s": SEARCH_CAP_S, "workers": a.workers,
           "best": {k: best[k] for k in ("base", "yaw_rad", "score", "goals_ok", "picks_ok", "base_side_yaws", "visit",
                                         "baseline", "goals_per_load", "picks")},
           "table": rows, "seconds": round(time.monotonic() - t0, 1)}
    sys.path.insert(0, str(ROOT / "code/execution/frigidaire"))
    import frigidaire_robot_episode as EP
    res["heights_m"] = list(EP.DROP_HEIGHTS_M)
    a.mount_out.parent.mkdir(parents=True, exist_ok=True)
    a.mount_out.write_text(json.dumps(res, indent=1) + "\n")
    n_skip = sum(r["status"].startswith("skipped") for r in rows)
    print(f"[RESULT] PASS mount search {len(rows) - n_skip}/{len(rows)} mounts scored in {res['seconds']} s: picked "
          f"{best['base']} yaw {best['yaw_rad']:.3f} score {best['score']} (base-side yaws {best['base_side_yaws']}); today's "
          f"{base['base']} score {base.get('score')} ({base['status']}) -> {a.mount_out.relative_to(ROOT)}", flush=True)


# ------------------------------------------------------------------ plans/2026-10-29-foot-ring-grasp.md: the foot ring pick
FOOT_MOUNT = ROOT / "data/results/robot/mount/posture_v1.json"
FOOT_TRANSIT_S = 15.                  # the episode's transit_to RRT budget (--transit-rrt-s)
FOOT_GATE_N = 10                      # the gate's trials: each passing yaw is re-checked at their ten +-5 mm spots


def foot_obj(xy=None):
    """The bowl mouth down at the counter spot (the rim on the slab, the ring on top), COUNTER_LIFT_M up as the gate spawns it."""
    from dishsim_frigidaire.robot import grasp as G
    xy = COUNTER_XY if xy is None else xy
    T = np.eye(4)
    T[:3, :3] = np.diag([1., -1., -1.])                      # 180 deg about x: the bowl axis (mouth) points down
    T[:3, 3] = (xy[0], xy[1], .914 + G.foot_ring()["bowl_height"] + COUNTER_LIFT_M)
    return T


def foot_clear(arm, q, np_, held=None):
    """(arm links vs the scene: d, link, target; the hand (gripper + wrist 3) vs the counter: d) at joint state q, FCL."""
    sys.path.insert(0, str(ROOT / "code/execution/frigidaire"))
    import frigidaire_robot_episode as EP
    objs = arm.arm_objects(q, ("shoulder_link",))
    links = [o for o in objs if o[0] != "gripper"]
    a = EP.min_clear(arm.fcl, links, EP.scene_targets(arm), np_, cutoff=.50)
    hand = [o for o in objs if o[0] in ("gripper", "wrist_3_link")]
    c = EP.min_clear(arm.fcl, hand, [("Counter", arm.slab)], np_, cutoff=.30)
    return a, c[0]


def foot_pick(W, arm, dish, g, M, q_home):
    """One foot candidate's counter pick on the arm model, the episode's pick order: the collision-free hover IK sorted by
    the smallest largest joint move from home (goal_qs), the 100-step approach line (the gripper may touch the bowl in the
    last ALLOW_CONTACT_M), the 80-step 0.15 m lift line with the bowl held at M (the bowl body off the scene)."""
    from dishsim_frigidaire.robot import grasp as G
    sys.path.insert(0, str(ROOT / "code/execution/frigidaire"))
    import frigidaire_robot_episode as EP
    T_g = g["T_tcp"]
    H, T_up = G.hover_along(T_g, HOVER_M), G.hover_along(T_g, LIFT_M)
    r = {"yaw_k": g["yaw_k"], "ok": False, "why": None}
    arm.set_dishes({"dish": dish})
    ik = W.kr.ik_all(H, q_home)
    hov = sorted([q for q in ik if arm.free(q)], key=lambda q: float(np.abs(q - q_home).max()))
    r.update(n_ik_hover=len(ik), n_free_hover=len(hov))
    if not hov:
        r["why"] = f"no collision-free IK at the pre-grasp hover ({len(ik)} IK, first hits {arm.collisions(ik[0], first=False)[:3] if ik else None})"
    for q_h in hov[:4]:
        qs, why = line_ok(W, arm, H, T_g, q_h, 100, {}, allow_last=EP.ALLOW_CONTACT_M, dish="dish")
        if qs is None:
            r["why"] = f"approach line {why}"
            continue
        clear_a = [foot_clear(arm, q, np) for q in qs[::10] + [qs[-1]]]
        arm.set_dishes({})
        up, why = line_ok(W, arm, T_g, T_up, qs[-1], 80, {"held": ("bowl", M, "dish")})
        if up is None:
            arm.set_dishes({"dish": dish})
            r["why"] = f"lift line {why}"
            continue
        clear_u = [foot_clear(arm, q, np) for q in up[::10] + [up[-1]]]
        arm.set_dishes({"dish": dish})
        worst = min(clear_a + clear_u, key=lambda c: c[0][0])
        r.update(ok=True, why=None, q_hover=[round(float(v), 5) for v in q_h], q_grasp=[round(float(v), 5) for v in qs[-1]],
                 q_lift=[round(float(v), 5) for v in up[-1]],
                 arm_clear_mm=round(float(worst[0][0]) * 1e3, 1), arm_clear_link=worst[0][1], arm_clear_target=worst[0][2],
                 hand_counter_mm=round(float(min(c[1] for c in clear_a + clear_u)) * 1e3, 1),
                 hand_counter_at_grasp_mm=round(float(clear_a[-1][1]) * 1e3, 1),
                 max_joint_move_rad=round(float(np.abs(q_h - q_home).max()), 3))
        break
    arm.set_dishes({})
    return r


def hand_dish_clear(arm, q, dish):
    """The open gripper and wrist 3 at joint state q vs the dish body: the smallest FCL distance (m; inf = nothing within 50 mm)."""
    sys.path.insert(0, str(ROOT / "code/execution/frigidaire"))
    import frigidaire_robot_episode as EP
    objs = [o for o in arm.arm_objects(q, ("shoulder_link",)) if o[0] in EP.JAW_LINKS]
    return float(EP.min_clear(arm.fcl, objs, [("dish", dish)], np, cutoff=.05)[0])


def foot_side_pick(W, arm, dish, g, M, q_home):
    """One side ring hold candidate's counter pick on the arm model (plans/2026-11-04-ring-side-hold.md), the cup side pinch's
    rules: the collision-free hover IK (back along the horizontal approach) sorted by the smallest largest joint move from home
    (goal_qs), the 100-step approach line (the open gripper may come near the bowl all along, ALLOW_STRADDLE_M) on which the open
    gripper and wrist 3 never touch the bowl (FCL distance > 0, a tightening of that rule), the 0.15 m lift straight up (40 steps)
    with the bowl held at M (the bowl body off the scene)."""
    from dishsim_frigidaire.robot import grasp as G
    sys.path.insert(0, str(ROOT / "code/execution/frigidaire"))
    import frigidaire_robot_episode as EP
    T_g = g["T_tcp"]
    H, T_up = G.hover_along(T_g, HOVER_M), G.hover(T_g, LIFT_M)
    r = {"yaw_k": g["yaw_k"], "wrist_side": g["wrist_side"], "ok": False, "why": None}
    arm.set_dishes({"dish": dish})
    ik = W.kr.ik_all(H, q_home)
    hov = sorted([q for q in ik if arm.free(q)], key=lambda q: float(np.abs(q - q_home).max()))
    r.update(n_ik_hover=len(ik), n_free_hover=len(hov))
    if not hov:
        r["why"] = f"no collision-free IK at the pre-grasp hover ({len(ik)} IK, first hits {arm.collisions(ik[0], first=False)[:3] if ik else None})"
    for q_h in hov[:4]:
        qs, why = line_ok(W, arm, H, T_g, q_h, 100, {}, allow_last=EP.ALLOW_STRADDLE_M, dish="dish")
        if qs is None:
            r["why"] = f"approach line {why}"
            continue
        gd = [hand_dish_clear(arm, q, dish) for q in qs[::10] + [qs[-1]]]
        if min(gd) <= 0.:
            r["why"] = f"approach line: the open gripper touches the bowl ({min(gd) * 1e3:.1f} mm)"
            continue
        clear_a = [foot_clear(arm, q, np) for q in qs[::10] + [qs[-1]]]
        arm.set_dishes({})
        up, why = line_ok(W, arm, T_g, T_up, qs[-1], 40, {"held": ("bowl", M, "dish")})
        arm.set_dishes({"dish": dish})
        if up is None:
            r["why"] = f"lift line {why}"
            continue
        clear_u = [foot_clear(arm, q, np) for q in up[::10] + [up[-1]]]
        worst = min(clear_a + clear_u, key=lambda c: c[0][0])
        r.update(ok=True, why=None, q_hover=[round(float(v), 5) for v in q_h], q_grasp=[round(float(v), 5) for v in qs[-1]],
                 q_lift=[round(float(v), 5) for v in up[-1]],
                 arm_clear_mm=round(float(worst[0][0]) * 1e3, 1), arm_clear_link=worst[0][1], arm_clear_target=worst[0][2],
                 hand_counter_mm=round(float(min(c[1] for c in clear_a + clear_u)) * 1e3, 1),
                 hand_bowl_mm=round(min(gd) * 1e3, 1) if np.isfinite(min(gd)) else None,
                 hand_bowl_at_grasp_mm=round(gd[-1] * 1e3, 1) if np.isfinite(gd[-1]) else None,
                 max_joint_move_rad=round(float(np.abs(q_h - q_home).max()), 3))
        break
    arm.set_dishes({})
    return r


def foot_transit(W, arm, q_home, q_hover, T_hover, z_safe=1.25):
    """transit_to's legs from the gate's home (the TCP already at z_safe): home -> the hover raised to z_safe (same IK branch
    as q_hover) -> the hover; each leg a straight joint move (checked every 0.02 rad) else RRT-Connect (FOOT_TRANSIT_S)."""
    C = W.C
    above = np.array(T_hover, copy=True)
    above[2, 3] = z_safe
    q_above = W.kr.ik(above, q_hover)
    if q_above is not None and not arm.free(q_above):
        q_above = None                                    # transit_to then plans home -> the hover directly
    plan = ([("home -> above", q_home, q_above), ("above -> hover", q_above, np.asarray(q_hover))] if q_above is not None
            else [("home -> hover (no free IK above the hover)", q_home, np.asarray(q_hover))])
    legs = []
    for name, q0, q1 in plan:
        if not arm.free(q1):
            return {"ok": False, "legs": legs, "why": f"{name}: the end pose collides {arm.collisions(q1, first=False)[:3]}"}
        if arm.path_free(q0, q1, step=.02):
            legs.append({"leg": name, "path": "straight"})
            continue
        t0 = time.monotonic()
        path = C.rrt_connect(arm, q0, [q1], time_s=FOOT_TRANSIT_S, shortcut_s=2.)
        legs.append({"leg": name, "path": "rrt" if path is not None else "rrt: none", "s": round(time.monotonic() - t0, 1)})
        if path is None:
            return {"ok": False, "legs": legs, "why": f"{name}: no RRT-Connect path in {FOOT_TRANSIT_S:.0f} s"}
    return {"ok": True, "legs": legs, "why": None}


def foot_main(a):
    """--foot-candidates (plans/2026-10-29-foot-ring-grasp.md): the bowl mouth down at the counter spot (the gate's racks: the
    upper rack pushed in; the posture_v1 mount), per family (out: the open pads close on the ring's outer face; in: the
    closed fingertips enter the recess and open on its inner face, checked with the closed jaws' hulls) and yaw (12) the
    pick of ``foot_pick``; per family the best candidate = the most collision-free hover IK, then the largest smallest
    arm clearance over the approach and the lift; its transit legs from the gate's home are planned too. -> --foot-out."""
    from dishsim_frigidaire.robot import grasp as G
    sys.path.insert(0, str(ROOT / "code/execution/frigidaire"))
    import frigidaire_robot_episode as EP
    t0 = time.monotonic()
    m = json.loads(FOOT_MOUNT.read_text())["best"]
    W = World({"base": m["base"], "yaw_rad": m["yaw_rad"]})
    arm = W.arms["LowerRack"]                             # the grasp gate's rack state: the upper rack pushed in
    T_o = foot_obj()
    dish = arm.checker._body("bowl", W.P.pose_dict(T_o), "dish")
    q_home = W.kr.ik(G.tcp_frame(np.array(HOME_TCP), [1., 0., 0.]), np.array([0.0, -1.712, 1.712, -1.571, -1.571, 0.0]))
    open_hulls, closed = arm.hulls["gripper"], EP.closed_gripper_hulls(W.C, np)
    res = {"plan": "plans/2026-10-29-foot-ring-grasp.md", "mount": m, "mount_file": rel_path(FOOT_MOUNT),
           "counter_xy": list(COUNTER_XY), "counter_front_y": COUNTER_FRONT_Y, "basket": W.basket, "bowl_pose": W.P.pose_dict(T_o),
           "racks": "upper rack pushed in (the grasp gate's)", "hover_m": HOVER_M, "lift_m": LIFT_M, "n_yaw": N_YAW,
           "ring": G.foot_ring(), "asset": str(G.FOOT_ASSET.relative_to(ROOT)),
           "q_home": [round(float(v), 5) for v in q_home],
           "rule": "pass = a collision-free hover IK (15 mm), a free approach line (the gripper may touch the bowl in the last "
                   f"{EP.ALLOW_CONTACT_M * 1e3:.0f} mm) and a free lift line with the bowl held; ranked (the gate tries them in "
                   "this order per trial) by the gate's ten +-5 mm spots passing, then the most free hover IK, then the largest "
                   "smallest arm-link clearance over the approach and the lift; best = the first", "families": {}}
    rng = np.random.default_rng(0)                        # the gate's +-5 mm spot draws (--seed 0, trials 0-9, in order)
    spots = [tuple(np.asarray(COUNTER_XY) + rng.uniform(-.005, .005, 2)) for _ in range(FOOT_GATE_N)]
    res["gate_spots_mm"] = [[round((x - COUNTER_XY[0]) * 1e3, 2), round((y - COUNTER_XY[1]) * 1e3, 2)] for x, y in spots]
    to_base = np.asarray(W.T_wb[:2, 3]) - T_o[:2, 3]
    for side in [x for x in a.foot_families.split(",") if x]:
        arm.hulls["gripper"] = closed if side == "in" else open_hulls
        gs = G.foot_side_grasps(T_o, N_YAW) if side == "side" else G.foot_grasps(T_o, N_YAW, side)
        pick_fn = foot_side_pick if side == "side" else foot_pick   # side (plans/2026-11-04-ring-side-hold.md): both rolls
        rows = []
        for i_g, g in enumerate(gs):
            M = np.linalg.inv(g["T_tcp"]) @ T_o
            rows.append(pick_fn(W, arm, dish, g, M, q_home))
            sd_ = g["T_tcp"][:2, 3] - T_o[:2, 3]          # the episode's pick order: the candidate facing the robot most
            rows[-1]["facing"] = round(float(np.dot(sd_, to_base) / (np.linalg.norm(sd_) * np.linalg.norm(to_base) + 1e-9)), 3)
            if rows[-1]["ok"]:                            # the same yaw at the gate's ten perturbed spots
                per = []
                for xy in spots:
                    T_s = foot_obj(xy)
                    d_s = arm.checker._body("bowl", W.P.pose_dict(T_s), "dish")
                    g_s = (G.foot_side_grasps(T_s, N_YAW) if side == "side" else G.foot_grasps(T_s, N_YAW, side))[i_g]
                    per.append(pick_fn(W, arm, d_s, g_s, np.linalg.inv(g_s["T_tcp"]) @ T_s, q_home)["ok"])
                rows[-1].update(gate_spots_pass=int(sum(per)), gate_spots=[bool(x) for x in per])
            print(f"[INFO] foot-{side} yaw {g['yaw_k']:2d}{' ' + g['wrist_side'] if side == 'side' else ''}: "
                  f"{'PASS' if rows[-1]['ok'] else 'fail'} "
                  f"hover IK {rows[-1]['n_free_hover']}/{rows[-1]['n_ik_hover']} "
                  f"{'' if not rows[-1]['ok'] else 'arm %.1f mm (%s vs %s), hand-counter %.1f mm, gate spots %d/%d' % (rows[-1]['arm_clear_mm'], rows[-1]['arm_clear_link'], rows[-1]['arm_clear_target'], rows[-1]['hand_counter_mm'], rows[-1]['gate_spots_pass'], FOOT_GATE_N)}"
                  f"{'' if rows[-1]['ok'] else str(rows[-1]['why'])[:150]}", flush=True)
        ok = [r for r in rows if r["ok"]]
        best, transit = None, None
        ranked = sorted(ok, key=lambda r: (-r["gate_spots_pass"], -r["n_free_hover"], -r["arm_clear_mm"]) if side != "side"
                        else (-r["gate_spots_pass"], -r["facing"], -r["n_free_hover"], -r["arm_clear_mm"]))
        if ok:
            best = ranked[0]
            g = gs[rows.index(best)]
            arm.set_dishes({"dish": dish})
            transit = foot_transit(W, arm, q_home, np.array(best["q_hover"]), G.hover_along(g["T_tcp"], HOVER_M))
            arm.set_dishes({})
        plan = {k: (v if not isinstance(v, (np.floating, float)) else round(float(v), 6)) for k, v in gs[0]["plan"].items()}
        res["families"][side] = {"kind": f"foot-{side}", "plan": plan, "rows": rows, "pass": len(ok),
                                 "ranked_yaws": [r["yaw_k"] for r in ranked],
                                 "best": None if best is None else {"yaw_rad": gs[rows.index(best)]["yaw"],
                                                                    "T_tcp": gs[rows.index(best)]["T_tcp"].tolist(), **best},
                                 "transit": transit}
        if side == "side":                                # the (yaw, wrist side) pairs in the gate's order; the rule
            res["families"][side].update(
                ranked=[[r["yaw_k"], r["wrist_side"]] for r in ranked],
                rule="pass = the cup side pinch's pick rules on the bowl's ring (foot_side_pick: a free hover IK, the approach "
                     "line with the open gripper never touching the bowl, the 0.15 m lift straight up with the bowl held); ranked "
                     "by the gate's ten +-5 mm spots passing, then facing the robot, then the most free hover IK, then the largest "
                     "smallest arm-link clearance")
        print(f"[INFO] foot-{side}: {len(ok)}/{len(gs)} candidates pass "
              f"{[(r['yaw_k'], r['wrist_side']) if side == 'side' else r['yaw_k'] for r in ok]}; best "
              f"{None if best is None else (best['yaw_k'], best.get('wrist_side'))}; transit "
              f"{None if transit is None else (transit['ok'], transit['legs'], transit['why'])}", flush=True)
    arm.hulls["gripper"] = open_hulls
    if "side" in res["families"]:
        res["plan"] = "plans/2026-11-04-ring-side-hold.md (the side family; plans/2026-10-29-foot-ring-grasp.md for out / in)"
    res["seconds"] = round(time.monotonic() - t0, 1)
    a.foot_out.parent.mkdir(parents=True, exist_ok=True)
    a.foot_out.write_text(json.dumps(res, indent=1, default=lambda o: o.tolist() if hasattr(o, "tolist") else str(o)) + "\n")
    good = bool(res["families"]) and all(f["best"] is not None and f["transit"]["ok"] for f in res["families"].values())
    print(f"[RESULT] {'PASS' if good else 'FAIL'} foot candidates "
          f"{ {s: (f['pass'], None if f['best'] is None else f['best']['yaw_k']) for s, f in res['families'].items()} } "
          f"in {res['seconds']} s -> {a.foot_out.relative_to(ROOT)}", flush=True)


# ------------------------------------------------------------------ plans/2026-11-04-grasp-place-s0.md: the sample's gate
SAMPLE_MOUNT = ROOT / "data/results/robot/mount/posture_v1.json"
SAMPLE_TRANSIT_S = 60.                # the episode's transit_to RRT-Connect budget per leg (--transit-rrt-s; 60 s since
                                      # plans/2026-11-04-s0-rerun.md, was 15)
SAMPLE_SHORTCUT_S = 10.               # the episode's --shortcut-s of the study's rows
SAMPLE_DZ = .002                      # transit_to's rise waypoints with a held dish (2 mm)
PRESENTATION_XY = (.20, -.12)         # the counter spot of every samples_v1 presentation.json without "spot_xy" (COUNTER_XY)
SIDE_MOVES = ("D-ring-side", "D-ring-side-insert")   # the side ring hold's moves (plans/2026-11-04-ring-side-hold.md; -insert:
                                      # its guarded insertion, plans/2026-11-04-fifteen-of-fifteen.md)
WALL_MOVES = ("I4-wall20", "I4-wall20-drop")   # the wall pinch's moves: -drop is the episode's name of an I4-wall20 plate passed by
                                      # D2's tine-guided release (frigidaire_placing_loads.DROP_MOVE); gated as I4-wall20, the drop allowed
INSERT_MOVES = ("D-ring-insert", "D-side-insert", "D-ring-side-insert")   # the guarded insertion's moves (= the episode's
                                      # INSERT_MOVES when it has them): the release at the stop, the back-off, the rise


def rel_path(p):
    """``p`` relative to the repo root when it lies under it (a record's path), else as given."""
    try:
        return str(Path(p).resolve().relative_to(ROOT))
    except ValueError:
        return str(p)


def use_mount(path):
    """--mount (plans/2026-11-04-fifteen-of-fifteen.md): gate at the mount file's best (SAMPLE_MOUNT, FOOT_MOUNT); its "spot_xy"
    (frigidaire_mount_sweep.py's posture_v2.json: the spawn spot moved with the arm along the counter's front edge) moves the
    counter spot COUNTER_XY, and with it a load's plate presentation (shift_presentation). Returns the mount's best."""
    global SAMPLE_MOUNT, FOOT_MOUNT, COUNTER_XY
    path = Path(path) if Path(path).is_absolute() else ROOT / path
    best = json.loads(path.read_text())["best"]
    SAMPLE_MOUNT = FOOT_MOUNT = path
    if best.get("spot_xy") is not None:
        COUNTER_XY = (float(best["spot_xy"][0]), float(best["spot_xy"][1]))
    return best


def shift_presentation(pres, xy=None):
    """A plate presentation moved with the counter spot ``xy`` (default COUNTER_XY): the plate, its backstop, its fixtures and
    any absolute grasp frames translated by xy minus the spot it was built at (its "spot_xy", else PRESENTATION_XY), the
    Kit-free pick record at the old spot dropped, "spot_xy" set. Unmoved: ``pres`` itself."""
    xy = COUNTER_XY if xy is None else xy
    x0, y0 = pres.get("spot_xy") or PRESENTATION_XY
    dx, dy = float(xy[0]) - float(x0), float(xy[1]) - float(y0)
    if abs(dx) < 1e-9 and abs(dy) < 1e-9:
        return pres
    out = json.loads(json.dumps(pres))

    def mv(p):
        return [float(p[0]) + dx, float(p[1]) + dy, float(p[2])]
    out["plate_pose"]["position_m"] = mv(out["plate_pose"]["position_m"])
    for b_ in [out.get("backstop")] + list(out.get("fixtures") or []):
        if b_:
            b_["center_m"] = mv(b_["center_m"])
    for g_ in out.get("grasps") or []:                # the edge pinch's absolute TCP frames (4 x 4, the translation column)
        g_["T_tcp"][0][3] += dx
        g_["T_tcp"][1][3] += dy
    out.pop("pick_kitfree", None)
    out["spot_xy"] = [float(xy[0]), float(xy[1])]
    out["moved_from_xy"] = [float(x0), float(y0)]
    return out


def gate_world(load_dir, m=None, basket=None):
    """The sample gate's World (sample_gate_job, one per worker): the mount ``m`` ({"base", "yaw_rad"}; default SAMPLE_MOUNT's
    best), the kind centroids, the planned holds, the hands of the load dir's Isaac gates (else the earlier rounds'), its plate
    presentation moved with the counter spot (shift_presentation) and the side ring hold's candidate order; ``basket`` False: no
    SilverwareBasket in its FCL worlds (None = BASKET, use_basket's setting)."""
    sys.path.insert(0, str(ROOT / "code/execution/frigidaire"))
    import frigidaire_robot_episode as EP
    from dishsim_frigidaire.robot import collide as C_
    m = m or json.loads(SAMPLE_MOUNT.read_text())["best"]
    W = World({"base": m["base"], "yaw_rad": m["yaw_rad"]}, basket=basket)
    W.cent = W.B.kind_centroids({k: visual_points_of(W.B, k) for k in ("bowl", "plate", "cup")})
    W.holds = sample_holds()
    gd_ = Path(load_dir) / EP.CUP_DOWN_GATE_RUN           # D-side-down's hand: the load's cup gate's closed jaws, else the cups
    W.cup_hulls = EP.cup_gate_hulls(C_, np, gd_ if (gd_ / "trials.jsonl").is_file() else EP.CUP_DOWN_GATE_DIR)   # round's
    pg_ = EP.gate_trial(Path(load_dir) / (EP.PLATE_WALL_GATE_RUN if (Path(load_dir) / EP.PLATE_WALL_GATE_RUN).is_dir() else
                                          EP.PLATE_EDGE_GATE_RUN), fallback=EP.PLATE_GATE)   # the pinch's jaws, else the
    W.plate_closed = (EP.closed_gripper_hulls(C_, np, run=pg_[0], traj=pg_[1]), pg_[2])   # rim gate's (a proxy)
    pf_ = Path(load_dir) / "presentation.json"
    W.pres = shift_presentation(json.loads(pf_.read_text())) if pf_.is_file() else None
    W.bowl_side = EP.ring_tool_hulls(C_, np, load_dir, PLACING / "partA_bowl_foot_out/inhand.json")   # D-ring-side's hand;
    fs_ = Path(load_dir) / "foot_side_candidates.json"                    # the hold sweep's (yaw, wrist side) order
    W.side_rank = (json.loads(fs_.read_text())["families"]["side"].get("ranked") if fs_.is_file() else None)
    return W


def sample_holds():
    """The planned hold per kind (TCP -> dish) and its source: bowl = the F-out gate's median hold (placing_v1/
    partA_bowl_foot_out/inhand.json), cup = the h45 mouth-side median hold (frigidaire_robot_episode.SIDE_MOUTH_RUN; the
    coordinator's option B: picked as measured, the cup turned mouth down with the wrist below it at the release, R0 = flip_up
    of the hold's TCP), plate = the rim-pinch gate's median hold (limits_v1/gate2_plate)."""
    sys.path.insert(0, str(ROOT / "code/execution/frigidaire"))
    import frigidaire_robot_episode as EP
    fh = json.loads((PLACING / "partA_bowl_foot_out/inhand.json").read_text())
    M0, src_c = measured_M("cup", PLACING / EP.SIDE_MOUTH_RUN)
    M_p, src_p = measured_M("plate")
    return {"cup_down": cup_down_hold(),
            "bowl": (np.asarray(fh["inhand"]["M_tcp_to_bowl"], dtype=float), None,
                     f"partA_bowl_foot_out/inhand.json (trial {fh['inhand']['trial']}, held {fh['held']}/{fh['trials']})"),
            "cup": (np.asarray(M0, dtype=float), EP.flip_up(np.asarray(src_c["T_tcp"])[:3, :3], np),
                    f"{EP.SIDE_MOUTH_RUN} median hold (trial {src_c['trial']}), the wrist toward the mouth"),
            "plate": (M_p, None, f"limits_v1/gate2_plate median hold (trial {src_p['trial']}, pivot {src_p['pivot_deg']} deg)")}


def cup_down_hold(below_top=None):
    """The planned hold (TCP -> cup, None, source) of the mouth-down cup: the nominal side_grasps hold with the pads ``below_top``
    under the rim (None: EP.SAMPLE_MOVE_ARGS["D-side-down"]'s; round 3: a load row's side_below_top_m), the wrist toward the
    base."""
    sys.path.insert(0, str(ROOT / "code/execution/frigidaire"))
    import frigidaire_robot_episode as EP
    from dishsim_frigidaire.robot import grasp as G
    bt = EP.SAMPLE_MOVE_ARGS["D-side-down"]["side_below_top"] if below_top is None else float(below_top)
    T_d = cup_down_obj()
    g_d = G.side_grasps(T_d, N_YAW, bt, wrist_side="base")[0]
    return (np.linalg.inv(g_d["T_tcp"]) @ T_d, None,
            f"the nominal side_grasps hold of the mouth-down cup (pads {bt * 1e3:.1f} mm under the rim, the wrist toward the "
            "base; plans/2026-11-04-cups-mouth-down.md)" + ("" if below_top is None else "; the load row's side_below_top_m"))


def row_cup_hulls(W, load_dir, run):
    """Round 3 (cup_05): the closed jaws of a load row's ``hold_gate_run`` (its Isaac cup gate in the load dir; no held trial
    there: the cups round's, as gate_world's W.cup_hulls), cached on W per run."""
    sys.path.insert(0, str(ROOT / "code/execution/frigidaire"))
    import frigidaire_robot_episode as EP
    from dishsim_frigidaire.robot import collide as C_
    cache = W.__dict__.setdefault("_row_cup_hulls", {})
    if run not in cache:
        gd_ = Path(load_dir) / run
        cache[run] = EP.cup_gate_hulls(C_, np, gd_ if (gd_ / "trials.jsonl").is_file() else EP.CUP_DOWN_GATE_DIR)
    return cache[run]


def pairs_of(arm, q, **kw):
    return sorted({f"{a}|{b}" for a, b in arm.collisions(q, first=False, **kw)})


def leg_ok(W, arm, q0, q1, kw, budget):
    """One transit leg: a straight joint move checked every 0.02 rad, else RRT-Connect (seed 1). (path kind or None, s)"""
    if arm.path_free(q0, q1, step=.02, **kw):
        return "straight", 0.
    t0 = time.monotonic()
    path = W.C.rrt_connect(arm, np.asarray(q0), [np.asarray(q1)], time_s=budget, collide_kw=kw, seed=1, shortcut_s=SAMPLE_SHORTCUT_S)
    return ("rrt" if path is not None else None), round(time.monotonic() - t0, 1)


def transit_check(W, arm, q_from, T_from, q_to, T_to, kw, rise_to=None, budget=SAMPLE_TRANSIT_S, z_safe=1.25):
    """frigidaire_robot_episode.transit_to's legs, Kit-free: the straight rise (``rise_to`` = the TCP height to rise to, None =
    none), the target raised to z_safe in the target's IK branch when free, then down to it; each leg straight else RRT."""
    legs, q, T = [], np.asarray(q_from, dtype=float), np.asarray(T_from, dtype=float)
    if rise_to is not None and T[2, 3] < rise_to - .01:
        up = T.copy()
        up[2, 3] = rise_to
        qs, why = line_ok(W, arm, T, up, q, max(20, int((rise_to - T[2, 3]) / SAMPLE_DZ)), kw)
        if qs is None:
            return {"ok": False, "legs": legs, "why": f"the rise to {rise_to:.2f} m: {why}"}
        legs.append({"leg": "rise", "path": "line"})
        q = qs[-1]
    above = np.array(T_to, dtype=float, copy=True)
    above[2, 3] = z_safe
    q_above = W.kr.ik(above, q_to)
    if q_above is not None and not arm.free(q_above, **kw):
        q_above = None
    plan = ([("above the target", q, q_above), ("down to the target", q_above, np.asarray(q_to))] if q_above is not None
            else [("to the target (no free IK above it)", q, np.asarray(q_to))])
    for name, q0, q1 in plan:
        kind_, s_ = leg_ok(W, arm, q0, q1, kw, budget)
        legs.append({"leg": name, "path": kind_ or "rrt: none", "s": s_})
        if kind_ is None:
            return {"ok": False, "legs": legs, "why": f"no path {name} (RRT-Connect {budget:.0f} s)"}
    return {"ok": True, "legs": legs, "why": None}


def sample_pick(W, arm, kind, q_home, down=False, below_top=None):
    """The episode's counter pick per kind on the arm model, every yaw: bowl = foot_pick (F-out), cup = side_pick (45 mm,
    wrist toward the mouth; ``down``: the cup mouth down, 60 mm under the rim, the wrist toward the base), plate = the rim
    pinch as the gate ran it (the dish clearance waived: the hover free of everything
    but the plate, the approach line IK-continuous only, the 0.15 m lift line checked with the plate held); the passing yaw
    facing the robot most (the episode's order), its home -> pre-grasp transit (transit_to's legs, empty-handed)."""
    from dishsim_frigidaire.robot import grasp as G
    sys.path.insert(0, str(ROOT / "code/execution/frigidaire"))
    import frigidaire_robot_episode as EP
    T_o = foot_obj() if kind == "bowl" else cup_down_obj() if down else counter_obj()
    dish = arm.checker._body(kind, W.P.pose_dict(T_o), "dish")
    sd_ = EP.SAMPLE_MOVE_ARGS["D-side-down"]
    gs = (G.foot_grasps(T_o, N_YAW, "out") if kind == "bowl" else
          G.side_grasps(T_o, N_YAW, sd_["side_below_top"] if below_top is None else float(below_top),   # round 3: the row's
                        wrist_side=sd_["side_wrist_side"]) if kind == "cup" and down else                    # side_below_top_m
          G.side_grasps(T_o, N_YAW, .045, wrist_side="mouth") if kind == "cup" else G.tilted_grasps(T_o, N_YAW, "plate"))
    rows = []
    for k, g in enumerate(gs):
        M_g = np.linalg.inv(g["T_tcp"]) @ T_o
        if kind == "bowl":
            r = foot_pick(W, arm, dish, g, M_g, q_home)
        elif kind == "cup":
            r = side_pick(W, arm, dish, g, M_g, q_home)
            arm.set_dishes({})
        else:
            r = {"ok": False, "why": None}
            H, T_up = G.hover_along(g["T_tcp"], HOVER_M), G.hover_along(g["T_tcp"], LIFT_M)
            arm.set_dishes({"dish": dish})
            hov = sorted([q for q in W.kr.ik_all(H, q_home) if arm.free(q, ignore=("dish",))],
                         key=lambda q: float(np.abs(q - q_home).max()))
            if not hov:
                qs_ = W.kr.ik_all(H, q_home)
                r["why"] = (f"no collision-free IK at the pre-grasp hover ({len(qs_)} IK, first hits "
                            f"{pairs_of(arm, qs_[0], ignore=('dish',))[:3] if qs_ else None})")
            arm.set_dishes({})
            for q_h in hov[:4]:
                line = W.kr.line_q(H, g["T_tcp"], 100, q_h)
                if line is None:
                    r["why"] = "approach line: IK branch jump"
                    continue
                up, why = line_ok(W, arm, g["T_tcp"], T_up, line[-1], 80, {"held": ("plate", M_g, "dish")})
                if up is None:
                    r["why"] = f"lift line {why}"
                    continue
                r.update(ok=True, why=None, q_hover=[round(float(v), 5) for v in q_h], q_lift=[round(float(v), 5) for v in up[-1]])
                break
        to_base = np.asarray(W.T_wb[:2, 3]) - T_o[:2, 3]
        side_ = g["T_tcp"][:2, 3] - T_o[:2, 3]
        r.update(yaw_k=k, facing=round(float(np.dot(side_, to_base) / (np.linalg.norm(side_) * np.linalg.norm(to_base) + 1e-9)), 3))
        rows.append(r)
    arm.set_dishes({})
    ok = sorted([r for r in rows if r["ok"]], key=lambda r: -r["facing"])
    out = {"pass": len(ok), "of": len(rows), "yaws": [r["yaw_k"] for r in ok], "ok": bool(ok), "rows": rows}
    if not ok:
        whys = sorted({str(r["why"]) for r in rows})
        out["why"] = f"no collision-free pick at the spot ({len(rows)} yaws): " + "; ".join(whys[:3])
        out["blocker"] = sorted({b for r in rows for b in __import__("re").findall(r"\('(\w+)', '(\w+)'\)", str(r["why"]))
                                 for b in ["|".join(b)]})
        return out
    best = ok[0]
    g = gs[best["yaw_k"]]
    H = G.hover_along(g["T_tcp"], HOVER_M)
    arm.set_dishes({"dish": dish})
    tr = transit_check(W, arm, q_home, G.tcp_frame(np.array(HOME_TCP), [1., 0., 0.]), np.array(best["q_hover"]), H,
                       {"ignore": ("dish",)} if kind == "plate" else {})
    arm.set_dishes({})
    T_lift = G.hover_along(g["T_tcp"], LIFT_M) if kind != "cup" else G.hover(g["T_tcp"], LIFT_M)
    out.update(best_yaw_k=best["yaw_k"], q_lift=best["q_lift"], T_lift=T_lift.tolist(), transit_home=tr,
               ok=bool(tr["ok"]), why=None if tr["ok"] else f"home -> pre-grasp: {tr['why']}")
    return out


def foot_side_sample_pick(W, arm, q_home, roll, rank=None):
    """D-ring-side (plans/2026-11-04-ring-side-hold.md): the side ring hold's counter pick of the wrist side ``roll`` at every yaw
    (foot_side_pick), the passing yaws in the hold sweep's order (``rank``: foot_side_candidates.json's [[yaw_k, wrist_side], ...];
    unranked ones after, facing the robot most), the first one's home -> pre-grasp transit (transit_to's legs, empty-handed) and its
    nominal hold M (TCP -> bowl). The keys of sample_pick plus "roll", "M" and "rank"."""
    from dishsim_frigidaire.robot import grasp as G
    T_o = foot_obj()
    dish = arm.checker._body("bowl", W.P.pose_dict(T_o), "dish")
    gs = [g for g in G.foot_side_grasps(T_o, N_YAW) if g["wrist_side"] == roll]
    to_base = np.asarray(W.T_wb[:2, 3]) - T_o[:2, 3]
    rk = [tuple(x) for x in (rank or [])]
    rows = []
    for g in gs:
        r = foot_side_pick(W, arm, dish, g, np.linalg.inv(g["T_tcp"]) @ T_o, q_home)
        sd_ = g["T_tcp"][:2, 3] - T_o[:2, 3]
        r["facing"] = round(float(np.dot(sd_, to_base) / (np.linalg.norm(sd_) * np.linalg.norm(to_base) + 1e-9)), 3)
        rows.append(r)
    arm.set_dishes({})
    ok = sorted([r for r in rows if r["ok"]], key=lambda r: (rk.index((r["yaw_k"], roll)) if (r["yaw_k"], roll) in rk else len(rk),
                                                            -r["facing"]))
    out = {"roll": roll, "pass": len(ok), "of": len(rows), "yaws": [r["yaw_k"] for r in ok], "ok": bool(ok), "rows": rows,
           "rank": "foot_side_candidates.json (the hold sweep's order)" if rk else "facing the robot most"}
    if not ok:
        whys = sorted({str(r["why"]) for r in rows})
        out["why"] = f"no collision-free pick at the spot ({len(rows)} yaws, wrist toward the {roll}): " + "; ".join(whys[:3])
        out["blocker"] = sorted({b for r in rows for b in __import__("re").findall(r"\('(\w+)', '(\w+)'\)", str(r["why"]))
                                 for b in ["|".join(b)]})
        return out
    best = ok[0]
    g = next(g_ for g_ in gs if g_["yaw_k"] == best["yaw_k"])
    H = G.hover_along(g["T_tcp"], HOVER_M)
    arm.set_dishes({"dish": dish})
    tr = transit_check(W, arm, q_home, G.tcp_frame(np.array(HOME_TCP), [1., 0., 0.]), np.array(best["q_hover"]), H, {})
    arm.set_dishes({})
    out.update(best_yaw_k=best["yaw_k"], q_lift=best["q_lift"], T_lift=G.hover(g["T_tcp"], LIFT_M).tolist(), transit_home=tr,
               M=np.linalg.inv(g["T_tcp"]) @ T_o, ok=bool(tr["ok"]), why=None if tr["ok"] else f"home -> pre-grasp: {tr['why']}")
    return out


def ring_release(W, arm, d, M, q_seed, F_goal, kind="bowl", aboves=None, seat_tool=None, rule1=True, pitch=False, insert=False,
                 side_order="pitch"):
    """D-ring (frigidaire_robot_episode ring_plan, Kit-free): per spin (24) the TCP at the goal turned about the bowl axis, 50 mm
    up the rack's axis = the descent start: G0 an IK at the goal; G1 a free IK at the descent start with the bowl held, the
    bowl mouth down and the TCP above its centroid; G2 the 11-point line down IK-continuous with the hand clear (hand_hits).
    ``kind`` cup, ``aboves`` (D-side-down, plans/2026-11-04-cups-mouth-down.md): the mouth-down cup's hold, the descent start the
    lowest of ``aboves`` that passes per spin (5 mm waypoints down), ranked by that height first (the episode's ring_plan);
    ``seat_tool`` (the closed jaws + wrist 3): the line down checked by D-seat's final-descent rule instead (EP.seat_line_hits,
    the hand and the links SEAT_ARM_MARGIN_M from the scene). ``rule1`` (plans/2026-11-04-bowl-carry-clearance.md, amended; the
    episode's --attach none ring_plan): every start of EP.DESCENT_STARTS_M with the held dish EP.CARRY_CLEAR_M from the scene
    (arm.held_clear_m, set by the caller), ranked by the held dish's clearance there. A dish with "build_release" (plans/2026-11-
    04-deep-sites-gate.md, amended): the build's move pose instead of the goal, the line down to the build's stop, the hand
    above the dish at the release EP.BUILD_UP_M above the stop. ``pitch`` (D-ring-side, plans/2026-11-04-ring-side-hold.md): each
    passing start carries the goal's tilt split over the hand (EP.side_pitch) and the smallest pitch about the closing axis ranks
    first (a roll of the hand where one exists), then rule 1's order; ``side_order`` "straight" (the load row's ring_side_order,
    the episode's --ring-side-order straight) keeps rule 1's order with the pitch breaking its ties (the episode's ring_plan).
    ``insert`` (the guarded insertion of INSERT_MOVES, plans/2026-11-04-fifteen-of-fifteen.md; the episode's ring_plan): the same
    starts and line, the release at the stop itself (the build's stop, else the goal; no BUILD_UP_M raise)."""
    from dishsim_frigidaire.robot.rig import T_of
    sys.path.insert(0, str(ROOT / "code/execution/frigidaire"))
    import frigidaire_robot_episode as EP
    aboves = EP.DESCENT_STARTS_M if rule1 else (aboves or (EP.RING_ABOVE_M,))
    build = d.get("build_release")
    fr = frames_for(W.B, d["rack"])[d["rack"]]
    F_r = T_of(fr["position_m"], fr["quaternion_xyzw"])
    T_g0 = F_r @ np.linalg.inv(F_goal) @ T_of(d["goal_pose"]["pos"], d["goal_pose"]["quat_xyzw"])
    T_m0 = (F_r @ np.linalg.inv(F_goal) @ T_of(build["move_pose"]["position_m"], build["move_pose"]["quaternion_xyzw"])
            if build else T_g0)
    lift = float(build["lowest_lift_mm"]) / 1e3 if build else 0.
    up_rel = EP.BUILD_UP_M if build and not insert else 0.   # the build: let go 20 mm above its stop; the insertion: at the stop
    up = F_r[:3, 2] / np.linalg.norm(F_r[:3, 2])
    c = np.r_[W.cent[kind], 1.]
    held = {"held": (kind, M, "dish"), "ignore": ("dish",)}
    seat_targets = EP.scene_targets(arm, ignore=("dish",)) if seat_tool is not None else None
    counts, first, ranked, g0 = {}, None, [], False
    for k in range(EP.RING_SPINS):
        sp = 2 * np.pi * k / EP.RING_SPINS
        Rz = np.eye(4)
        Rz[:2, :2] = [[np.cos(sp), -np.sin(sp)], [np.sin(sp), np.cos(sp)]]
        T_g = T_m0 @ Rz
        T_e = T_g.copy()
        T_e[:3, 3] += lift * up                        # the descent line's end: the goal, or the build's stop
        T_tg = T_e @ np.linalg.inv(M)
        T_r = T_e.copy()
        T_r[:3, 3] += up_rel * up                      # the dish at the release
        T_tr = T_r @ np.linalg.inv(M)
        g0 = g0 or bool(W.kr.ik_all(T_tg, q_seed))
        why0 = None
        if float(T_tr[2, 3] - (T_r @ c)[2]) <= 0.:
            why0 = "hand not above"
        elif T_g[2, 2] >= 0.:
            why0 = "mouth not down"
        whys = [why0] if why0 else []
        for ab in aboves if why0 is None else ():
            T_ds = T_tg.copy()
            T_ds[:3, 3] += ab * up
            n_line = int(round(ab / .005)) + 1         # 5 mm waypoints (EP.RING_LINE_N at 50 mm)
            qs = W.kr.ik_all(T_ds, q_seed)
            why = None if qs else f"no IK at the descent start (+{ab * 1e3:.0f} mm)"
            hit = None
            for q in qs:
                if not arm.free(q, **held):
                    why = f"the descent start collides (+{ab * 1e3:.0f} mm) {pairs_of(arm, q, **held)[:3]}"
                    continue
                line = W.kr.line_q(T_ds, T_tg, n_line, q)
                if line is None:
                    why = "IK branch jump on the descent line"
                    continue
                bad = next(((j, h_) for j, qq in enumerate(line) for h_ in
                            [EP.seat_line_hits(arm, qq, seat_tool, seat_targets, np) if seat_tool is not None else
                             [f"{a}|{b}" for a, b in arm.collisions(qq, first=False, held_scene=False, **held)
                              if a != "held" or b in W.C.ARM_LINKS]]
                            if h_), None)
                if bad is not None:
                    why = (f"the hand on the descent line {round((lift + ab * (1 - bad[0] / (n_line - 1))) * 1e3)} mm above the "
                           f"{'move pose' if build else 'goal'}: {bad[1][:3]}")
                    continue
                cl = arm.held_clearance(q, held["held"], ignore=("dish",), cutoff=EP.CLEAR_CUTOFF_M) if rule1 else (None, None)
                hit = {"spin_deg": round(360. * k / EP.RING_SPINS, 1), "q_start": q, "T_start": T_ds, "above_m": ab,
                       "move": float(np.abs(q - q_seed).max()), "T_goal_tcp": T_tg, "T_goal": T_g, "q_goal": line[-1],
                       "T_end": T_e, "T_rel": T_r, "T_tcp_rel": T_tr, "build": bool(build), "insert": bool(insert),
                       "clear_m": None if cl[0] is None else min(float(cl[0]), EP.CLEAR_CUTOFF_M), "clear_to": cl[1],
                       "axis": up,                    # the insertion axis (the rack's up axis): D4's retreat (b) rises along it
                       "T_g0": T_g0}                  # the goal now (insert_stop's heights: the dish centroid above the goal's)
                if pitch:
                    hit["pitch_deg"], hit["roll_deg"], hit["tilt_deg"] = (round(v_, 2) for v_ in EP.side_pitch(T_g, T_tg))
                why = None
                break
            whys.append(why)
            if hit is not None:
                ranked.append(hit)
                if not rule1:
                    break
        for why in whys:
            key = why.split(" ")[0] + " " + why.split(" ")[1] if why else "pass"
            counts[key] = counts.get(key, 0) + 1
            first = first or (f"spin {360. * k / EP.RING_SPINS:.0f}: {why}" if why else None)
    if rule1:
        ranked.sort(key=lambda c_: (-c_["clear_m"], c_["move"], c_["above_m"], c_["spin_deg"]))
    else:
        ranked.sort(key=lambda c_: (c_["above_m"], c_["move"], c_["spin_deg"]))   # D-ring: every start at 50 mm
    if pitch and side_order == "pitch":                # the smallest pitch first, rule 1's order within (EP.pitch_order)
        ranked = [ranked[i_] for i_ in EP.pitch_order(ranked)]
    elif pitch:                                        # --ring-side-order straight: rule 1's order, the pitch breaking its ties
        ranked.sort(key=lambda c_: (-c_["clear_m"], c_["move"], c_["pitch_deg"], c_["above_m"], c_["spin_deg"]))
    return {"G0": g0, "G1": bool(ranked), "ranked": ranked, "counts": counts, "first_fail": first,
            "why": None if ranked else (f"no passing spin ({counts}); {first}" if g0 else "G0: no IK at the goal for any spin"),
            "best": None if not ranked else {"spin_deg": ranked[0]["spin_deg"], "start_mm": round(ranked[0]["above_m"] * 1e3),
                                             "held_clear_mm": None if ranked[0]["clear_m"] is None else round(ranked[0]["clear_m"] * 1e3, 1),
                                             "held_clear_to": ranked[0]["clear_to"],
                                             **({k_: ranked[0][k_] for k_ in ("pitch_deg", "roll_deg", "tilt_deg")} if pitch else {})},
            "descent_starts": None if not pitch else [{"spin_deg": c_["spin_deg"], "start_mm": round(c_["above_m"] * 1e3),
                                               "held_clear_mm": None if c_["clear_m"] is None else round(c_["clear_m"] * 1e3, 1),
                                               "pitch_deg": c_["pitch_deg"], "roll_deg": c_["roll_deg"]} for c_ in ranked],
            "build_release": None if not build else {k_: build.get(k_) for k_ in ("lowest_lift_mm", "release_lift_mm", "stop")}}


PARTIAL_OPEN_CLEAR_M = .005           # the partial open (the user, 2026-11-04 22:20 EST; = frigidaire_robot_episode.PARTIAL_OPEN_CLEAR_M):
                                      # the release width the pad gap holding the dish at the pinch plus this much per side (10 mm in all)
OPEN_MODES = ("full", "partial")      # the open at a guarded insertion's release, in this order (insert_release_check): the first clear
                                      # of OPEN_MODES x RETREATS wins
RETREATS = ("a", "b", "c")            # D4 (plans/2026-11-04-fifteen-of-fifteen.md, the coordinator): the guarded insertion's retreat
RETREAT_RULES = {                     # after the open, tried in this order at each start (insert_release_check); the first clear wins
    "a": "back-off then rise: CUP_BACK_M (20 mm) along -TCP z in 5 mm steps, then RING_RISE_M (100 mm) straight up in 10 mm steps",
    "b": "rise along the insertion axis then back-off: RING_RISE_M (100 mm) along the rack's up axis (the line down's axis) in "
         "10 mm steps, then CUP_BACK_M (20 mm) along -TCP z in 5 mm steps",
    "c": "rise only: RING_RISE_M (100 mm) straight up in 10 mm steps"}


def open_release_check(W, arm, kind, c_, back=True, variant=None, others=False):
    """The supported-release rule, Kit-free (plans/2026-11-04-cups-mouth-down.md): the dish at its release (the descent line's
    end; the build release: BUILD_UP_M above the stop; the guarded insertion, c_["insert"]: the stop itself), the TCP there, the
    jaws OPEN (the arm model's open gripper hulls and wrist 3, frigidaire_robot_episode.JAW_LINKS) against the dish and against
    the appliance (racks, basket, tub, cabinet) at the release, along the back-off (``back``: CUP_BACK_M along -TCP z, 5 mm steps:
    the side holds and the guarded insertion, as the episode's ring_drop; the D-ring / -build bowls have none) and along the rise
    (RING_RISE_M straight up, 10 mm steps), each pose IK-continuous from the last; ok = every FCL distance >= JAW_MARGIN_M
    (2 mm, the jaw check's margin) and no IK jump. ``variant`` (D4, RETREAT_RULES): "a" = back=True's path, "b" = the rise along
    the insertion axis (c_["axis"]) then the back-off, "c" = back=False's path; None = ``back``'s (today's). ``others`` (D4, as the
    episode's retreat_check): the open jaws also against the counter, the static boxes and the scene's other dishes ("others_mm")
    and the links SEAT_LINKS against all of it and the dish ("links_mm", SEAT_ARM_MARGIN_M)."""
    sys.path.insert(0, str(ROOT / "code/execution/frigidaire"))
    import frigidaire_robot_episode as EP
    T0 = np.asarray(c_.get("T_tcp_rel", c_["T_goal_tcp"]))
    q = np.asarray(c_["q_goal"])
    if c_.get("build") and not c_.get("insert"):       # the build: up BUILD_UP_M from the stop with the dish, then open (the
        q = W.kr.ik(T0, q)                             # guarded insertion opens at the stop: q_goal itself)
    body = arm.checker._body(kind, W.P.pose_dict(np.asarray(c_.get("T_rel", c_["T_goal"]))), "dish")
    comps = list((arm.checker.components or {}).items())
    oth = (([("Counter", arm.slab)] if arm.slab is not None else []) + list(arm.statics.items())
           + [(n_, b_) for n_, b_ in arm.dishes.items() if n_ != "dish"]) if others else []
    v_ = variant or ("a" if back else "c")
    poses = [("release", 0., T0)]

    def back_off(T_from):
        for j in range(1, int(round(EP.CUP_BACK_M / .005)) + 1):
            T = T_from.copy()
            T[:3, 3] -= .005 * j * T0[:3, 2]
            poses.append(("back-off", .005 * j, T))

    def rise(T_from, ax):
        for j in range(1, int(round(EP.RING_RISE_M / .010)) + 1):
            T = T_from.copy()
            T[:3, 3] += .010 * j * ax
            poses.append(("rise", .010 * j, T))
    z_ = np.array([0., 0., 1.])
    if v_ == "b":                                      # back out along the insertion first, then away from the dish
        rise(T0, np.asarray(c_.get("axis", z_), dtype=float))
        back_off(poses[-1][2])
    else:
        if v_ == "a":
            back_off(T0)
        rise(poses[-1][2], z_)
    out, prev = {}, q
    if prev is None:
        return {"ok": False, "stages": {"release": {"ik_jump_at_mm": 0}}, "spin_deg": c_["spin_deg"], "limit_mm": EP.JAW_MARGIN_M * 1e3,
                "variant": v_, "rule": "no IK continuous from the stop to the release"}
    lims_ = {"dish_mm": EP.JAW_MARGIN_M * 1e3, "appliance_mm": EP.JAW_MARGIN_M * 1e3,
             **({"others_mm": EP.JAW_MARGIN_M * 1e3, "links_mm": EP.SEAT_ARM_MARGIN_M * 1e3} if others else {})}
    names_ = {"appliance_mm": "appliance", "others_mm": "others", "links_mm": "links"}
    for stage, s_, T in poses:
        q_ = prev if stage == "release" else W.kr.ik(T, prev)
        if q_ is None or float(np.abs(q_ - prev).max()) > .1:
            out.setdefault(stage, {})["ik_jump_at_mm"] = round(s_ * 1e3)
            break
        prev = q_
        arm_objs = arm.arm_objects(q_, ("shoulder_link",))
        objs = [o for o in arm_objs if o[0] in EP.JAW_LINKS]
        meas = {"dish_mm": EP.min_clear(arm.fcl, objs, [(kind, body)], np, cutoff=.05),
                "appliance_mm": EP.min_clear(arm.fcl, objs, comps, np, cutoff=.05)}
        if others:
            meas["others_mm"] = EP.min_clear(arm.fcl, objs, oth, np, cutoff=.05) if oth else (np.inf, None, None)
            meas["links_mm"] = EP.min_clear(arm.fcl, [o for o in arm_objs if o[0] in EP.SEAT_LINKS],
                                            [(kind, body)] + comps + oth, np, cutoff=.05)
        r_ = out.setdefault(stage, {"dish_mm": None, "appliance_mm": None, "appliance": None,
                                    **({"others_mm": None, "others": None, "links_mm": None, "links": None} if others else {})})
        for key_, (d_, nm_, tn_) in meas.items():
            vv_ = round(float(d_) * 1e3, 1) if np.isfinite(d_) else None      # None: nothing within 50 mm
            if vv_ is not None and (r_[key_] is None or vv_ < r_[key_]):
                r_[key_] = vv_
                if key_ in names_:
                    r_[names_[key_]] = f"{nm_}|{tn_}"
    lim = EP.JAW_MARGIN_M * 1e3
    want = {"release", "back-off", "rise"} if v_ in ("a", "b") else {"release", "rise"}
    ok = all("ik_jump_at_mm" not in r_ and all(r_.get(k_) is None or r_[k_] >= l_ for k_, l_ in lims_.items())
             for r_ in out.values()) and set(out) == want
    where = ((" (the guarded insertion: at the predicted sensor stop)" if c_.get("stop_pred") else
              " (the guarded insertion: at the build's stop)" if c_.get("build") else " (the guarded insertion: at the goal)")
             if c_.get("insert") else " (the build: 20 mm above its stop)" if c_.get("build") else " (the goal)")
    return {"ok": bool(ok), "stages": out, "spin_deg": c_["spin_deg"], "limit_mm": lim, "variant": v_,
            "rule": ("the open gripper and wrist 3 at the release" + where
                     + (", along the 20 mm back-off" if v_ == "a" else "") + " and along the 100 mm rise: the FCL distance to "
                     "the dish at its release pose and to the appliance >= 2 mm (None = nothing within 50 mm)") if variant is None else
                    (f"the open gripper and wrist 3 at the release{where}, then retreat ({v_}) {RETREAT_RULES[v_]}: the FCL distance "
                     "to the dish at its release pose and to the appliance" + (
                         ", the counter, the static boxes and the scene's other dishes, and the links "
                         f"{', '.join(EP.SEAT_LINKS)} to all of them," if others else "")
                     + " >= 2 mm at every pose (None = nothing within 50 mm), each pose IK-continuous")}


def insert_stop(W, arm, kind, c_, tool):
    """The guarded insertion's predicted sensor stop at one start, as the episode's ring_plan predicts it (its insert_stop; plans/
    2026-11-04-fifteen-of-fifteen.md: the gate and the episode judge the release at the same stop): the dish at the descent line's
    end (c_["T_end"]) and the closed tool (``tool``: the hold's closed jaws + wrist 3, rigid with the TCP at c_["T_goal_tcp"]) lowered
    together along the rack's up axis (c_["axis"]): the dish's first touch of the scene's non-static bodies, the tool's of any scene
    body when it comes first, down to EP.DESCENT_BELOW_M under the goal (steps EP.clear_step_mm). -> {"dz_m": below the line's end
    (0: touching there, or nothing touched), "h_mm": the dish centroid above the goal's there, "body": "dish" | "hand" | None,
    "target"}"""
    sys.path.insert(0, str(ROOT / "code/execution/frigidaire"))
    import frigidaire_robot_episode as EP
    up = np.asarray(c_["axis"], dtype=float)
    T_e, T_tg = np.asarray(c_["T_end"], dtype=float), np.asarray(c_["T_goal_tcp"], dtype=float)
    cc = np.r_[W.cent[kind], 1.]
    h0 = float((T_e @ cc)[2] - (np.asarray(c_["T_g0"], dtype=float) @ cc)[2])
    targets = EP.scene_targets(arm, ignore=("dish",))
    W3i = np.linalg.inv(arm.T_w3_tcp)

    def at(T, h):
        T2 = np.array(T, dtype=float, copy=True)
        T2[:3, 3] += (h - h0) / max(float(up[2]), 1e-6) * up
        return T2

    def first_touch(objs_at, tg_, h_floor):
        h = h0
        while h > h_floor:
            d, _, tn = EP.min_clear(arm.fcl, objs_at(h), tg_, np)
            if d <= 0:
                return h, tn
            h = round(h - EP.clear_step_mm(d) / 1e3, 4)
        return None

    def dish_objs(h):
        b_ = arm.checker._body(kind, EP._pose_of(at(T_e, h)), "dish")
        return [("dish", b_["manager"], *b_["bounds"])]

    def tool_objs(h):
        W3 = at(T_tg, h) @ W3i
        o_ = []
        for g, v in tool:
            p = np.asarray(v) @ W3[:3, :3].T + W3[:3, 3]
            o_.append(("tool", arm.fcl.CollisionObject(g, arm.fcl.Transform(W3[:3, :3], W3[:3, 3])), p.min(0), p.max(0)))
        return o_
    hd = first_touch(dish_objs, [t for t in targets if t[0] not in arm.statics], -EP.DESCENT_BELOW_M)
    ht = first_touch(tool_objs, targets, (hd[0] if hd else -EP.DESCENT_BELOW_M) - 1e-6)
    hit = (ht[0], "hand", ht[1]) if ht is not None and (hd is None or ht[0] > hd[0]) else (
        (hd[0], "dish", hd[1]) if hd is not None else None)
    if hit is None:
        return {"dz_m": 0., "h_mm": round(h0 * 1e3, 1), "body": None, "target": None}
    return {"dz_m": max(0., h0 - hit[0]), "h_mm": round(hit[0] * 1e3, 1), "body": hit[1], "target": hit[2]}


def gate_axial_hulls(W, load_dir):
    """The axial ring hold's closed jaws (D-ring-insert's tool: the episode's ring_hulls, frigidaire_robot_episode.axial_ring_hulls:
    the load dir's F-out gate when it has a held trial, else placing_v1's), cached on the World. (hulls, source)"""
    if getattr(W, "bowl_axial", None) is None:
        sys.path.insert(0, str(ROOT / "code/execution/frigidaire"))
        import frigidaire_robot_episode as EP
        W.bowl_axial = EP.axial_ring_hulls(W.C, np, load_dir, PLACING / "partA_bowl_foot_out/inhand.json")
    return W.bowl_axial


def insert_release_check(W, arm, kind, c_, closed=None):
    """D4 (plans/2026-11-04-fifteen-of-fifteen.md): the guarded insertion's open-jaw check at one start: the retreats of RETREATS in
    order (open_release_check with that variant, the scene's other dishes as targets too), the first clear one used. -> the chosen
    variant's record (none clear: "variant" None, "ok" False and variant a's stages) with "start_mm" and every tried variant's
    verdict and stages ("variants"). ``closed`` (the hold's closed jaws, a HullList; the partial open, the user's decision 2026-11-04
    22:20 EST): judged where the episode judges it, at the predicted sensor stop (insert_stop with EP.tool_hulls(arm, closed); the
    line from the descent line's end down to it IK-continuous), the open there the first clear of OPEN_MODES x RETREATS: the full
    open (the arm model's open gripper hulls), else the partial open (EP.partial_open: the pad gap holding the dish at the hold
    trial's finger angle plus PARTIAL_OPEN_CLEAR_M a side, its hull the trial's 2F-85 linkage there); the record adds "open" (the
    chosen open, None when none is clear), "modes" (every open tried with its variants), "stop" and "judged_at". Without ``closed``:
    at the descent line's end, the full open only (the earlier rule)."""
    if closed is not None:
        return insert_release_open(W, arm, kind, c_, closed)
    tried = {}
    for v_ in RETREATS:
        tried[v_] = open_release_check(W, arm, kind, c_, variant=v_, others=True)
        if tried[v_]["ok"]:
            break
    ok_ = next((v_ for v_, r_ in tried.items() if r_["ok"]), None)
    out = dict(tried[ok_ or RETREATS[0]])
    out.update(ok=ok_ is not None, variant=ok_, start_mm=round(float(c_["above_m"]) * 1e3),
               variants={v_: {"ok": r_["ok"], "stages": r_["stages"]} for v_, r_ in tried.items()},
               retreat_rule="D4: the first clear of " + "; ".join(f"({v_}) {RETREAT_RULES[v_]}" for v_ in RETREATS)
               + " (the open jaws 2 mm from the dish, the appliance, the counter, the static boxes and the scene's other dishes, "
                 "the links SEAT_LINKS 2 mm from all of them; = the episode's retreat_check)")
    return out


def insert_drop_gate(W, arm, kind, c_, closed, rack):
    """The guided release at one start (frigidaire_robot_episode.INSERT_DROP_RULE; the user, 2026-11-04 23:02 EST), as the episode's
    ring_plan runs it: the predicted sensor stop (insert_stop with the hold's closed jaws ``closed``), the hand walked up from there
    (EP.insert_drop_walk: checkpoints every EP.DROP_CHECK_MM and the build's stop, the full open then the partial open, retreats a, b,
    c, the guidance under the wire tips of ``rack`` inside the dish's goal footprint, EP.footprint_tips). -> the walk's record (no
    joints or poses) with "spin_deg", "start_mm", "stop", "tips", "judged_at", "mode": "drop"."""
    sys.path.insert(0, str(ROOT / "code/execution/frigidaire"))
    import frigidaire_robot_episode as EP
    st_ = insert_stop(W, arm, kind, c_, EP.tool_hulls(arm, closed))
    up = np.asarray(c_["axis"], dtype=float)
    T_st = np.array(c_["T_goal_tcp"], dtype=float, copy=True)
    T_st[:3, 3] -= st_["dz_m"] * up
    q_st = np.asarray(c_["q_goal"], dtype=float)
    if st_["dz_m"] > 1e-4:
        ln_ = W.kr.line_q(np.asarray(c_["T_goal_tcp"], dtype=float), T_st, max(3, int(round(st_["dz_m"] / .005)) + 1), q_st)
        q_st = None if ln_ is None else np.asarray(ln_[-1], dtype=float)
    base = {"spin_deg": c_["spin_deg"], "start_mm": round(float(c_["above_m"]) * 1e3), "stop": st_, "mode": "drop",
            "judged_at": "the guided release: the lowest clear checkpoint above the predicted sensor stop (EP.INSERT_DROP_RULE)",
            "limit_mm": EP.JAW_MARGIN_M * 1e3}
    if q_st is None:
        return {**base, "ok": False, "variant": None, "open": None, "variants": {}, "release": None,
                "why": f"IK branch jump between the descent line's end and the predicted stop {st_['h_mm']} mm"}
    T_dst = np.array(c_["T_end"], dtype=float, copy=True)
    T_dst[:3, 3] -= st_["dz_m"] * up
    cc = np.r_[W.cent[kind], 1.]
    h_line = float((np.asarray(c_["T_end"], dtype=float) @ cc)[2] - (np.asarray(c_["T_g0"], dtype=float) @ cc)[2])
    if not hasattr(W, "vpts"):
        W.vpts = {}
    if kind not in W.vpts:
        W.vpts[kind] = np.asarray(visual_points_of(W.B, kind), dtype=float)
    tips = EP.footprint_tips(arm, rack, c_["T_g0"], W.vpts[kind], np)
    dw = EP.insert_drop_walk(arm, W.kr.ik, kind, "dish", T_st, q_st, T_dst, up, h_line - st_["dz_m"], h_line,
                             h_line + float(c_["above_m"]), EP.scene_targets(arm, ignore=("dish",)),
                             lambda: EP.partial_open(W.C, np, closed, clear=PARTIAL_OPEN_CLEAR_M), tips, W.vpts[kind], np)
    out = {k_: v_ for k_, v_ in dw.items() if k_ not in ("q_end", "T_tcp_end", "T_dish_end")}
    out.update(base, tips=tips, stages={"release": dw["release"]} if dw["release"] else {})
    return out


def insert_release_open(W, arm, kind, c_, closed):
    """insert_release_check with the hold's closed jaws ``closed``: the predicted sensor stop (insert_stop), the release there, the
    first clear of OPEN_MODES x RETREATS (the partial open's hulls in place of the arm model's open hulls while it is checked)."""
    sys.path.insert(0, str(ROOT / "code/execution/frigidaire"))
    import frigidaire_robot_episode as EP
    rule_ = ("D4 with the open (the user, 2026-11-04 22:20 EST): at the predicted sensor stop (the episode's ring_plan insert_stop), "
             "the first clear of (full, partial) x (" + "; ".join(f"({v_}) {RETREAT_RULES[v_]}" for v_ in RETREATS) + ") (the open "
             "jaws 2 mm from the dish, the appliance, the counter, the static boxes and the scene's other dishes, the links SEAT_LINKS "
             "2 mm from all of them; = the episode's retreat_check_open); the partial open: the pad gap holding the dish at the hold "
             f"trial's finger angle plus {PARTIAL_OPEN_CLEAR_M * 1e3:.0f} mm a side (grasp.pad_at / theta_at_inner), its hull the "
             "trial's 2F-85 linkage there")
    st_ = insert_stop(W, arm, kind, c_, EP.tool_hulls(arm, closed))
    up = np.asarray(c_["axis"], dtype=float)
    T_st = np.array(c_["T_goal_tcp"], dtype=float, copy=True)
    T_st[:3, 3] -= st_["dz_m"] * up
    q_st = np.asarray(c_["q_goal"], dtype=float)
    if st_["dz_m"] > 1e-4:                            # IK-continuous from the line's end down to the predicted stop (the episode's)
        ln_ = W.kr.line_q(np.asarray(c_["T_goal_tcp"], dtype=float), T_st, max(3, int(round(st_["dz_m"] / .005)) + 1), q_st)
        q_st = None if ln_ is None else np.asarray(ln_[-1], dtype=float)
    base = {"spin_deg": c_["spin_deg"], "start_mm": round(float(c_["above_m"]) * 1e3), "stop": st_, "limit_mm": EP.JAW_MARGIN_M * 1e3,
            "judged_at": "the predicted sensor stop (the episode's ring_plan insert_stop: the dish and the closed jaws lowered from "
                         "the descent line's end along the rack's up axis to their first touch)", "retreat_rule": rule_}
    if q_st is None:
        return {**base, "ok": False, "variant": None, "open": None, "stages": {}, "variants": {}, "modes": {},
                "why": f"IK branch jump between the descent line's end and the predicted stop {st_['h_mm']} mm"}
    T_dst = np.array(c_["T_end"], dtype=float, copy=True)
    T_dst[:3, 3] -= st_["dz_m"] * up
    c2 = dict(c_, T_tcp_rel=T_st, q_goal=q_st, T_rel=T_dst, stop_pred=True)
    modes, chosen, partial = {}, None, None
    for mode in OPEN_MODES:
        if mode == "partial":
            partial = EP.partial_open(W.C, np, closed, clear=PARTIAL_OPEN_CLEAR_M)
            if not partial.get("hulls"):
                modes["partial"] = {"ok": False, "variant": None, "why": partial.get("why"), "variants": {},
                                    "open": EP.open_record(partial), "tried": {}}
                break
        keep, tried = arm.hulls["gripper"], {}
        if mode == "partial":
            arm.hulls["gripper"] = partial["hulls"]
        try:
            for v_ in RETREATS:
                tried[v_] = open_release_check(W, arm, kind, c2, variant=v_, others=True)
                if tried[v_]["ok"]:
                    chosen = (mode, v_)
                    break
        finally:
            arm.hulls["gripper"] = keep
        modes[mode] = {"ok": chosen is not None, "variant": chosen[1] if chosen else None, "tried": tried,
                       "variants": {v_: {"ok": r_["ok"], "stages": r_["stages"]} for v_, r_ in tried.items()},
                       "open": EP.open_full_record() if mode == "full" else EP.open_record(partial)}
        if chosen:
            break
    m_, v_ = chosen or ("full", None)
    out = dict(modes[m_]["tried"][v_ or RETREATS[0]])
    out.update(base, ok=chosen is not None, variant=v_, open=modes[m_]["open"] if chosen else None,
               variants=modes[m_]["variants"], modes={k_: {kk: vv for kk, vv in m.items() if kk != "tried"} for k_, m in modes.items()})
    return out


def cup_release(W, arm, d, M, R0, q_seed):
    """D-side-mouth (frigidaire_robot_episode drop, --hold side-mouth): the mouth-side hold turned mouth down (TCP x up, the
    wrist below the cup), the cup centroid over the goal centroid at DROP_HEIGHTS_M x 12 yaws: G0 an IK at a drop pose; G1 the lowest height with a
    collision-free IK (the drop tries: that height's yaws by the smallest joint move, up to DROP_TRIES)."""
    from dishsim_frigidaire.robot.rig import T_of
    sys.path.insert(0, str(ROOT / "code/execution/frigidaire"))
    import frigidaire_robot_episode as EP
    c = np.r_[W.cent["cup"], 1.]
    g_c, m_c = (T_of(d["goal_pose"]["pos"], d["goal_pose"]["quat_xyzw"]) @ c)[:3], (M @ c)[:3]
    held = {"held": ("cup", M, "dish"), "ignore": ("dish",)}
    g0, tries, hits = False, [], {}
    for h in EP.DROP_HEIGHTS_M:
        cand = []
        for k in range(N_YAW):
            T = EP.drop_tcp(R0, m_c, g_c, h, k, np)
            qs = W.kr.ik_all(T, q_seed)
            g0 = g0 or bool(qs)
            free = [q for q in qs if arm.free(q, **held)]
            if free:
                cand.append((float(np.abs(free[0] - q_seed).max()), k, T, free[0]))
            elif qs:
                for b in pairs_of(arm, qs[0], **held):
                    hits[b] = hits.get(b, 0) + 1
        if cand:
            tries = [{"h_m": h, "yaw_k": k, "T": T, "q": q, "move": mv} for mv, k, T, q in sorted(cand, key=lambda x: x[0])][:EP.DROP_TRIES]
            break
    return {"G0": g0, "G1": bool(tries), "ranked": tries, "lowest_m": tries[0]["h_m"] if tries else None,
            "why": None if tries else (f"no collision-free IK at any drop height (25-350 mm) and yaw; commonest hits "
                                       f"{sorted(hits.items(), key=lambda x: -x[1])[:4]}" if g0 else "G0: no IK at any drop pose"),
            "blocker": sorted(hits, key=lambda b: -hits[b])[:4]}


SEAT_AT_GOAL_M = .020                 # D3 (plans/2026-11-04-fifteen-of-fifteen.md, the user): a predicted or gated seat counts only
                                      # within this of the goal along the descent axis, with rack wires as its only supports


def seat_at_goal(sk, rack, labels, limit_m=SEAT_AT_GOAL_M):
    """D3: a seat_search record ``sk`` is at the goal when |seat_mm| <= ``limit_m`` along the descent axis (the seat search's
    heights are measured from the goal, plates moved SEAT_DX_M toward the basket) and every support is a part of ``rack`` (the
    LowerRack: its wire labels, ``labels`` = EP.rack_labels; another rack: its body name); a dish or any other body among the
    supports is a perch. {"ok", "seat_mm", "supports", "not_rack", "on_dish", "limit_mm", "why", "rule"}"""
    sup = [str(s) for s in sk.get("supports") or []]
    mine = set(labels) if rack == "LowerRack" else {rack}
    not_rack = [s for s in sup if s not in mine]
    seat = sk.get("seat_mm")
    ok = seat is not None and abs(float(seat)) <= limit_m * 1e3 + 1e-9 and not not_rack
    return {"ok": bool(ok), "seat_mm": seat, "supports": sup, "not_rack": not_rack, "on_dish": list(sk.get("on_dish") or []),
            "limit_mm": round(limit_m * 1e3, 1),
            "why": None if ok else f"seat {seat} mm up the axis on {sup}: not at the goal",
            "rule": f"D3: the seat within {limit_m * 1e3:.0f} mm of the goal along the descent axis and on {rack} parts only (a "
                    "dish or another body among its supports is a perch: G0, never attempted)"}


def plate_release(W, arm, d, M, q_seed):
    """I4 (frigidaire_robot_episode seat_plan, Kit-free): the goal turned about the plate axis so the pinch is the rim point
    highest at the goal (the top edge), moved 1 mm toward the basket; the plate alone lowered along the tine axis to its seat;
    G0 an IK at the seat; G1 a descent start 50..150 mm above the seat with a free IK continuous down to 30 mm under the seat;
    then the line up the tine axis toward z_safe (>= 100 mm free) whose top is leg 1's end. D3: a seat off its goal (seat_at_goal)
    is G0, not attempted."""
    from dishsim_frigidaire.robot.rig import T_of
    sys.path.insert(0, str(ROOT / "code/execution/frigidaire"))
    import frigidaire_robot_episode as EP
    T_goal = T_of(d["goal_pose"]["pos"], d["goal_pose"]["quat_xyzw"])
    T_g = EP.sampler_bowl_tcp(T_goal, M, 0., np) @ M                 # the plate at its goal, spun: the pinch at the top edge
    fr = frames_for(W.B, "LowerRack")["LowerRack"]
    x_dir, v, lean_t = EP.tine_axis(T_of(fr["position_m"], fr["quaternion_xyzw"]), np)
    T_c = T_g.copy()
    T_c[:3, 3] += (EP.descent_bias_mm(d) if hasattr(EP, "descent_bias_mm") else EP.SEAT_DX_M * 1e3) / 1e3 * x_dir   # round 3
    labels = EP.rack_labels(len(arm.checker.parts["LowerRack"]))
    sk = EP.seat_search(arm, "plate", "dish", T_c, v, EP.scene_targets(arm, ignore=("dish",), statics=False), labels, np)
    if sk.get("seat_mm") is None:
        return {"G0": False, "G1": False, "why": f"no seat: the plate alone along the tine axis touches nothing ({sk})"}
    sg = seat_at_goal(sk, "LowerRack", labels)
    if not sg["ok"]:                                   # D3: a perch is never attempted
        return {"G0": True, "G1": False, "why": sg["why"], "seat_mm": sk.get("seat_mm"), "supports": sk.get("supports"),
                "seat_at_goal": sg, "blocker": sg["not_rack"]}
    T_ts = sk.pop("T_seat") @ np.linalg.inv(M)
    held = {"held": ("plate", M, "dish"), "ignore": ("dish",)}
    g0 = bool(W.kr.ik_all(T_ts, q_seed))
    starts, hits = [], {}
    h_s = EP.SEAT_START_ABOVE_M
    while h_s <= EP.SEAT_START_MAX_M + 1e-9 and not starts:
        T_s = T_ts.copy()
        T_s[:3, 3] += h_s * v
        for q in W.kr.ik_all(T_s, q_seed):
            if not arm.free(q, **held):
                for b in pairs_of(arm, q, **held):
                    hits[b] = hits.get(b, 0) + 1
                continue
            prev, brk = q, None
            for j in range(1, int(round((h_s + EP.DESCENT_BELOW_M) * 1e3)) + 1):
                T_j = T_s.copy()
                T_j[:3, 3] -= j * 1e-3 * v
                qq = W.kr.ik(T_j, prev)
                if qq is None or float(np.abs(qq - prev).max()) > .1:
                    brk = j
                    break
                prev = qq
            if brk is None:
                starts.append((h_s, T_s, q))
        if not starts:
            h_s = round(h_s + EP.SEAT_START_STEP_M, 3)
    out = {"G0": g0, "G1": bool(starts), "seat_mm": sk.get("seat_mm"), "supports": sk.get("supports"), "seat_at_goal": sg,
           "spin": "the pinch at the rim point highest at the goal", "starts": [], "blocker": sorted(hits, key=lambda b: -hits[b])[:4]}
    if not starts:
        out["why"] = (f"no collision-free IK continuous to the bound at any descent start {EP.SEAT_START_ABOVE_M * 1e3:.0f}.."
                      f"{EP.SEAT_START_MAX_M * 1e3:.0f} mm above the seat; commonest hits {sorted(hits.items(), key=lambda x: -x[1])[:4]}"
                      if g0 else "G0: no IK at the seat")
        return out
    for h_s, T_s, q_s in starts:
        h_full = (1.25 - float(T_s[2, 3])) / float(v[2])
        n_up = max(3, int(round(h_full / EP.SEAT_LINE_STEP_M)))
        prev, q_top, h_top, why_up = q_s, None, 0., None
        for j in range(1, n_up + 1):
            T_j = T_s.copy()
            T_j[:3, 3] += h_full * j / n_up * v
            q = W.kr.ik(T_j, prev)
            if q is None or float(np.abs(q - prev).max()) > EP.SEAT_LINE_JUMP_RAD:
                why_up = f"IK branch jump {h_full * j / n_up * 1e3:.0f} mm up the axis"
                break
            if not arm.free(q, **held):
                why_up = f"collides {h_full * j / n_up * 1e3:.0f} mm up the axis: {pairs_of(arm, q, **held)[:2]}"
                break
            prev, q_top, h_top = q, q, h_full * j / n_up
        T_top = T_s.copy()
        T_top[:3, 3] += h_top * v
        out["starts"].append({"h_s_mm": round(h_s * 1e3), "line_up_mm": round(h_top * 1e3), "stopped": why_up,
                              "q_s": q_s, "T_s": T_s, "q_top": q_top, "T_top": T_top, "v": v})
    good = [s_ for s_ in out["starts"] if s_["q_top"] is not None and s_["line_up_mm"] >= EP.SEAT_LINE_MIN_M * 1e3 - 1e-6]
    out["ok_starts"] = good
    if not good:
        out["G1"] = False
        out["why"] = f"the line up the tine axis is under {EP.SEAT_LINE_MIN_M * 1e3:.0f} mm free: {[s_['stopped'] for s_ in out['starts']]}"
    return out


PRESENTATION = {"name": "lean30", "yaw_k": 4, "lean_deg": 30.}   # plans/2026-11-04-plate-pick-presentation.md (amended): phase 1's
                                      # d30-top at yaw 4 (the mouth facing azimuth 120 deg) passed the pick at both rolls and the
                                      # I4 check at plate_05 and plate_03 (presentations.json i4_all d30-top, try (4, 0))
PINCH_SPINS_DEG = (0., 15., 30., 45., 60., 75., 90.)   # the plate's pinch moved round the rim from the rim point highest toward
                                      # the front (phase 1's pitch relaxation, plans/2026-11-04-deep-sites-gate.md); the first passes


def plate_world(W):
    """reach's World made usable by frigidaire_plate_presentations' helpers: the plate geometry (rim radius, height, visual
    points, the edge pinch's height edge_z), the lower rack's arm, the home joints."""
    if not hasattr(W, "geom"):
        sys.path.insert(0, str(ROOT / "code/execution/frigidaire"))
        import frigidaire_plate_presentations as PP
        P = visual_points_of(W.B, "plate")
        r = np.linalg.norm(P[:, :2], axis=1)
        near = P[r >= r.max() - PP.EDGE_REACH_M]
        W.geom = {"rim_r": float(r.max()), "height": float(P[:, 2].max()), "points": P, "centroid": P.mean(axis=0),
                  "edge_z": float((near[:, 2].min() + near[:, 2].max()) / 2)}
        W.arm, W.q_home, W.goals = W.arms["LowerRack"], home_q(W), {}
    return W


def plate_presentation(W):
    """The plate leaning PRESENTATION["lean_deg"] on its backstop at the counter spot (frigidaire_plate_presentations.edge_pose
    and .backstop, phase 1's construction), the edge pinch's parameters and both rolls -> the presentation json the episode
    reads (the Isaac spawn, the /World/Backstop box, the arm model's static box, grasp.edge_grasps)."""
    sys.path.insert(0, str(ROOT / "code/execution/frigidaire"))
    import frigidaire_plate_presentations as PP
    from scipy.spatial.transform import Rotation
    from dishsim_frigidaire.robot import grasp as G
    W = plate_world(W)
    T_p, ah, a, n = PP.edge_pose(W, PRESENTATION["yaw_k"], PRESENTATION["lean_deg"])
    body = W.arm.checker._body("plate", W.P.pose_dict(T_p), "dish")
    fx, info = PP.backstop(W, T_p, ah, body)
    bx = fx[0]["box"]
    Rm = np.asarray(bx["axes"], dtype=float)
    gs = G.edge_grasps(T_p, W.geom["rim_r"], W.geom["edge_z"], G.EDGE_OUT_M)
    return {**PRESENTATION, "plate_pose": W.P.pose_dict(T_p),
            "backstop": {"center_m": bx["center_m"], "size_m": bx["size_m"], "axes": bx["axes"],
                         "quaternion_xyzw": [round(float(v), 8) for v in Rotation.from_matrix(Rm).as_quat()]},
            "edge": {"rim_r_m": round(W.geom["rim_r"], 6), "edge_z_m": round(W.geom["edge_z"], 6), "out_m": G.EDGE_OUT_M},
            "grasps": [{"roll_deg": g["roll_deg"], "T_tcp": np.round(g["T_tcp"], 6).tolist()} for g in gs],
            "fixture": info, "mouth_dir": np.round(ah, 6).tolist(),
            "rule": "the plate on its edge, its mouth facing azimuth 2 pi yaw_k / 12, its axis lean_deg up from horizontal (leaning "
                    "back onto its base side), the rim circle's centre over the counter spot, its lowest point 2 mm above the "
                    "slab; the backstop a 20 x 160 mm box half as tall as the plate's top above the slab, behind the plate, its "
                    "top edge 1 mm from the plate's base face (FCL bisection); the edge pinch at the rim point highest (TCP z "
                    "radially inward, TCP y along the plate axis, the TCP 3 mm outside the rim edge at edge_z), both rolls",
            "source": "frigidaire_plate_presentations.py (phase 1) edge_pose / backstop at yaw_k, lean_deg; grasp.edge_grasps"}


def plate_presentation_job():
    """plate_presentation on a World of its own (not the worker's cached one, which reads presentation.json)."""
    m = json.loads(SAMPLE_MOUNT.read_text())["best"]
    out = json.loads(json.dumps(plate_presentation(World({"base": m["base"], "yaw_rad": m["yaw_rad"]})),
                                default=lambda o: o.tolist() if hasattr(o, "tolist") else str(o)))
    out["spot_xy"] = [float(v) for v in COUNTER_XY]   # built at today's spot: shift_presentation leaves it there
    return out


def extra_fixtures(pres):
    """A presentation's fixtures other than its backstop (the plank presentation's lip): [{id, center_m, size_m, axes}]."""
    b_id = (pres.get("backstop") or {}).get("id", "Backstop")
    return [f_ for f_ in pres.get("fixtures") or [] if f_.get("id") not in (b_id, "Backstop")]


def edge_pick(W, pres):
    """The leaning plate's edge pinch at the counter spot, Kit-free: per roll frigidaire_plate_presentations.pick (G1 hover,
    grasp and 0.15 m lift with the plate held; G2 the approach line with the gripper free to touch the plate in its last
    60 mm, the straight-up lift line and the home transit; full 15 mm clearance), the backstop a static box, and so is every
    other fixture of the presentation (the plank's lip; sample_gate_job removes them after the dish, as the backstop). ->
    sample_pick's shape with the passing rolls (facing the robot first) and their nominal holds M."""
    sys.path.insert(0, str(ROOT / "code/execution/frigidaire"))
    import frigidaire_plate_presentations as PP
    import frigidaire_robot_episode as EP
    from dishsim_frigidaire.robot.rig import T_of
    W = plate_world(W)
    arm = W.arm
    T_p = T_of(pres["plate_pose"]["position_m"], pres["plate_pose"]["quaternion_xyzw"])
    b_ = pres["backstop"]
    arm.statics["Backstop"] = EP.box_body(arm.fcl, "Backstop", b_["center_m"], b_["size_m"], b_["axes"], np)
    for f_ in extra_fixtures(pres):                   # = data/logs/robot_final/kitfree/plank_gate.py's lip (the s0 rerun's rows)
        arm.statics[f_["id"]] = EP.box_body(arm.fcl, f_["id"], f_["center_m"], f_["size_m"], f_["axes"], np)
    dish = arm.checker._body("plate", W.P.pose_dict(T_p), "dish")
    rows = []
    if pres.get("wall"):                              # plans/2026-11-04-plate-wall-pinch.md phase 2: the wall pinch, each (spin, roll)
        from dishsim_frigidaire.robot import grasp as G   # once (the episode retries the same candidate on a miss)
        wl = pres["wall"]
        pres = {**pres, "grasps": [{"roll_deg": g_["roll_deg"], "spin_deg": g_["spin_deg"], "T_tcp": g_["T_tcp"]} for g_ in
                                   G.plate_wall_grasps(T_p, np.asarray(wl["T_hold"], dtype=float), wl["spins_deg"],
                                                       sorted(set(wl["rolls"]), key=list(wl["rolls"]).index))]}
    for g in pres["grasps"]:
        r = PP.pick(W, arm, dish, T_p, np.asarray(g["T_tcp"], dtype=float), True)
        r.update(roll_deg=g["roll_deg"], yaw_k=pres["yaw_k"], T_tcp=g["T_tcp"])
        rows.append(r)
    arm.set_dishes({})
    ok = sorted([r for r in rows if r["ok"]], key=lambda r: (-r["facing"], r["move_rad"]))
    out = {"pass": len(ok), "of": len(rows), "rolls": [r["roll_deg"] for r in ok], "ok": bool(ok), "rows": rows,
           "presentation": pres["name"], "candidates": ok}
    if not ok:
        out["why"] = "no collision-free edge pinch at the presentation: " + "; ".join(sorted({str(r.get("why")) for r in rows}))[:400]
        out["blocker"] = sorted({b for r in rows for b in (r.get("blockers") or {})})[:6]
        return out
    out.update(best_roll=ok[0]["roll_deg"], q_lift=ok[0]["q_lift"], T_lift=ok[0]["T_lift"], transit_home=ok[0].get("transit"))
    return out


PLATE_DROP_GUIDE_M = .020             # D2 (plans/2026-11-04-fifteen-of-fifteen.md, the user's tine-guided plate release): a plate is let
                                      # go above its seat only with its lowest rim point at least this far below its slot's tine tips
DROP_KEYS = ("release_mm", "guided_mm", "first_violation", "gap_mm", "arm_clear_mm", "start_mm", "drop_retreat")   # a drop start's release record
PLATE_DROP_RULE = ("D2, the tine-guided plate release (I4-wall20; the user, 2026-11-04 20:05 EST): when no descent start reaches "
                   "the seat under D-seat's final-descent rule and the starts that pass fail it only against a dish or the rack, "
                   "the plate is lowered along the tine axis to h_free (release_mm above the seat), the lowest checkpoint (the "
                   "start, then every 5 mm, IK-continuous in 1 mm steps) down to which the closed hand, the open gripper hull and "
                   "SEAT_LINKS stay SEAT_ARM_MARGIN_M (2 mm) from the scene; there the jaws open and the open hand rises back along "
                   "the tine axis (no back-off). Allowed only when the held plate's lowest rim point there is at least "
                   "PLATE_DROP_GUIDE_M (20 mm) below its slot's tine tips (guided_mm: the plate already between its tines, no free "
                   "fall), else G0 'not tine-guided'. The landing is the episode's to judge (the plate rule: dz 20 mm, tilt 16 deg, "
                   "and the seated test); plates only")


def slot_tine_tips(arm, slot, labels):
    """The tine tips of a LowerRack plate slot (its two tine columns g and g + 1, both rows of its bank: rows 0, 1 in front, 4, 5 at
    the rear, as frigidaire_robot_episode.seated_test): per tine its tip's world z (the higher capsule end plus the radius, the
    rack's FCL parts at the gate's frames) and the lowest of them. {"tips_z_m": {label: z}, "tip_z_m": z or None}"""
    g_ = int(slot.split("_")[-1])
    want = {f"tine row{r} col{c}" for r in ((0, 1) if "_front_" in slot else (4, 5)) for c in (g_, g_ + 1)}
    tips = {}
    for i, (o, p) in enumerate(zip(arm.checker.components["LowerRack"]["objects"], arm.checker.parts["LowerRack"])):
        if i >= len(labels) or labels[i] not in want or type(p.geometry).__name__ != "Capsule":
            continue
        c = np.asarray(o.getTranslation(), dtype=float)
        h = abs(float(np.asarray(o.getRotation(), dtype=float)[2, 2])) * p.geometry.lz / 2
        tips[labels[i]] = max(tips.get(labels[i], -np.inf), float(c[2] + h + p.geometry.radius))
    return {"tips_z_m": {k: round(z, 5) for k, z in sorted(tips.items())}, "tip_z_m": min(tips.values()) if tips else None}


def rim_low_z(W, T):
    """The lowest world z of the plate's rim circle (W.geom, plate_world: radius rim_r at the height ``height`` of its base-centred
    frame) at the pose T."""
    T = np.asarray(T, dtype=float)
    return float(T[2, 2] * W.geom["height"] + T[2, 3] - W.geom["rim_r"] * math.hypot(T[2, 0], T[2, 1]))


def drop_hits(EP, arm, q, tool, targets):
    """D2's rule at joint state q: EP.seat_line_hits (the closed hand ``tool`` and SEAT_LINKS) and the open gripper's hulls (the jaws
    open at the release and rise open), each at least EP.SEAT_ARM_MARGIN_M from every target. [] when clear, else
    ["<part>|<target> <d> mm", ...]"""
    out = list(EP.seat_line_hits(arm, q, tool, targets, np))
    objs = [o for o in arm.arm_objects(q, ("shoulder_link",)) if o[0] == "gripper"]
    d, _, tn = EP.min_clear(arm.fcl, objs, targets, np, cutoff=max(.05, 2 * EP.SEAT_ARM_MARGIN_M))
    if d < EP.SEAT_ARM_MARGIN_M:
        out.append(f"gripper|{tn} {d * 1e3:.1f} mm")
    return out


def drop_clear(EP, arm, q, tool, targets):
    """[d mm, "part|target"]: the smallest FCL distance of the closed hand, the open gripper and SEAT_LINKS at q (0.10 m cutoff)."""
    W3 = arm.link_poses(q)["wrist_3_link"]
    objs = [o for o in arm.arm_objects(q, ("shoulder_link",)) if o[0] in tuple(EP.SEAT_LINKS) + ("gripper",)]
    for g, vv in tool:
        p = np.asarray(vv) @ W3[:3, :3].T + W3[:3, 3]
        objs.append(("hand", arm.fcl.CollisionObject(g, arm.fcl.Transform(W3[:3, :3], W3[:3, 3])), p.min(0), p.max(0)))
    d, nm, tn = EP.min_clear(arm.fcl, objs, targets, np, cutoff=.10)
    return [round(float(d) * 1e3, 1) if np.isfinite(d) else None, f"{nm}|{tn}"]


def drop_retreat_at(EP, W, arm, q_rel, T_ts, v, h_rel_mm, h_start_mm, T_seat, targets):
    """Round 3 (the user, 2026-11-05 00:05 EST; EP.DROP_RETREAT_RULE): the drop's retreat judged Kit-free at the release
    (the TCP ``h_rel_mm`` above the seat along the tine axis ``v``, joints ``q_rel``) with the released plate at its seat ``T_seat``
    among the targets: the back-off 20 mm along -TCP z then the rise up the tine axis back to the start height (``h_start_mm``),
    else the rise alone. -> EP.drop_retreat_check's record (variant, back_mm, rise_mm, checks, release, rule)."""
    T_rel = np.asarray(T_ts, dtype=float).copy()
    T_rel[:3, 3] += h_rel_mm / 1e3 * np.asarray(v, dtype=float)
    tg = list(targets) + [("dish", arm.checker._body("plate", W.P.pose_dict(T_seat), "dish"))]
    rt = EP.drop_retreat_check(arm, W.kr.ik, np.asarray(q_rel, dtype=float), T_rel, v, (h_start_mm - h_rel_mm) / 1e3, tg, np,
                               dish="dish")
    rt["judged_at"] = "Kit-free: the planned release, the released plate at its seat"
    return rt


def tine_guided(EP, W, arm, d, T_ts, M, v, fails, tool, targets, held):
    """D2 (PLATE_DROP_RULE): per start height of ``fails`` ({h_s m: [q, ...]}: the IK solutions whose start passes and whose full
    descent failed only on D-seat's rule against a dish or the rack) the walk down the tine axis again from the start (IK-continuous
    in 1 mm steps) under drop_hits at the start and every 5 mm: h_free = the lowest clear checkpoint (mm above the seat), the IK
    solution with the lowest h_free per height; guided_mm = the held plate's lowest rim point there below the slot's tine tips.
    -> {"cands": [(h_s, T_s, q, clear m, clear to, info)] of the heights with guided_mm >= PLATE_DROP_GUIDE_M, the lowest h_free
    first, then rule 1 (the held plate's clearance at the start); "tried": per height its record; "tips": slot_tine_tips}"""
    W = plate_world(W)
    tips = slot_tine_tips(arm, d["slot"], EP.rack_labels(len(arm.checker.parts["LowerRack"])))
    T_seat = T_ts @ np.asarray(M, dtype=float)

    def plate_at(h_mm):
        T = T_seat.copy()
        T[:3, 3] += h_mm / 1e3 * v
        return T
    tried, cands = [], []
    for h_s, qs in sorted(fails.items()):
        T_s = T_ts.copy()
        T_s[:3, 3] += h_s * v
        n_dn, best = int(round(h_s * 1e3)), None
        for q in qs:
            hh = drop_hits(EP, arm, q, tool, targets)
            h_free, q_free, viol = (None, None, {"at_mm": n_dn, "hits": hh[:2]}) if hh else (n_dn, q, None)
            prev = q
            for j in range(1, n_dn + 1) if not hh else ():
                T_j = T_s.copy()
                T_j[:3, 3] -= j * 1e-3 * v
                qq = W.kr.ik(T_j, prev)
                if qq is None or float(np.abs(qq - prev).max()) > .1:
                    viol = {"at_mm": n_dn - j, "hits": ["IK branch jump"]}
                    break
                if j % 5 == 0 or j == n_dn:
                    hh = drop_hits(EP, arm, qq, tool, targets)
                    if hh:
                        viol = {"at_mm": n_dn - j, "hits": hh[:2]}
                        break
                    h_free, q_free = n_dn - j, qq
                prev = qq
            if best is None or (h_free is not None and (best[0] is None or h_free < best[0])):
                best = (h_free, q_free, viol, q)
        h_free, q_free, viol, q = best
        rec = {"start_mm": n_dn, "release_mm": h_free, "guided_mm": None, "first_violation": viol, "pass": False}
        if viol is not None and viol["hits"]:
            viol["pair"] = viol["hits"][0].split(" ")[0]
        if h_free is not None and tips["tip_z_m"] is not None:
            rec["guided_mm"] = round((tips["tip_z_m"] - rim_low_z(W, plate_at(h_free))) * 1e3, 1)
            rec["pass"] = rec["guided_mm"] >= PLATE_DROP_GUIDE_M * 1e3 - 1e-9
        tried.append(rec)
        if not rec["pass"]:
            continue
        b = arm.checker._body("plate", W.P.pose_dict(plate_at(h_free)), "dish")
        gap = {n_: round(float(EP.depth_of(arm.fcl, b["manager"], b_["manager"])) * 1e3, 1)
               for n_, b_ in [("LowerRack", arm.checker.components["LowerRack"])] + [x for x in arm.dishes.items() if x[0] != "dish"]}
        cl = arm.held_clearance(q, held["held"], ignore=("dish",), cutoff=EP.CLEAR_CUTOFF_M)
        cands.append((h_s, T_s, q, min(float(cl[0]), EP.CLEAR_CUTOFF_M), cl[1],
                      {"mode": "drop", "start_mm": n_dn, "release_mm": h_free, "guided_mm": rec["guided_mm"],
                       "first_violation": viol, "gap_mm": gap, "arm_clear_mm": drop_clear(EP, arm, q_free, tool, targets),
                       "drop_retreat": drop_retreat_at(EP, W, arm, q_free, T_ts, v, h_free, n_dn, T_seat, targets)}))
    cands.sort(key=lambda c_: (c_[5]["release_mm"], -c_[3], c_[0]))
    return {"cands": cands, "tried": tried, "tips": tips}


def i4_release(W, arm, d, M, q_seed, F_goal, tool, spin_deg=0., guided=False):
    """I4 under rule 1, Kit-free (the episode's --attach none seat_plan): the goal turned about the plate axis so the pinch is
    ``spin_deg`` from the rim point highest at the goal, moved SEAT_DX_M toward the basket, the plate alone lowered along the
    tine axis to its seat (seat_search); D3: a seat off its goal (seat_at_goal: more than 20 mm up the axis, or not on rack wires
    alone) is G0, never attempted; G0 an IK at the seat; G1 every descent start 50..150 mm above the seat (5 mm steps)
    with a free IK (the arm 15 mm, the held plate arm.held_clear_m from the scene), continuous to DESCENT_BELOW_M under the seat
    and with the hand (``tool``: the closed jaws + wrist 3) and SEAT_LINKS 2 mm from the scene every 5 mm down to the seat,
    ranked by the held plate's clearance at the start ("mode" seat, "release_mm" 0); ``guided`` (I4-wall20, D2): no such start,
    and the passing starts failing that rule only against a dish or the rack: the tine-guided release of tine_guided ("mode"
    drop, PLATE_DROP_RULE), else G0 "not tine-guided"; then, for up to DROP_TRIES starts, the line up the tine axis toward
    z_safe (>= 100 mm free) whose top is leg 1's end (leg 1 and the line down are the caller's carry)."""
    from dishsim_frigidaire.robot.rig import T_of
    sys.path.insert(0, str(ROOT / "code/execution/frigidaire"))
    import frigidaire_robot_episode as EP
    fr = frames_for(W.B, "LowerRack")["LowerRack"]
    F_r = T_of(fr["position_m"], fr["quaternion_xyzw"])
    T_goal = F_r @ np.linalg.inv(F_goal) @ T_of(d["goal_pose"]["pos"], d["goal_pose"]["quat_xyzw"])
    T_g = EP.sampler_bowl_tcp(T_goal, M, spin_deg, np) @ M
    x_dir, v, lean_t = EP.tine_axis(F_r, np)
    bias_mm = (EP.descent_bias_mm(d) if hasattr(EP, "descent_bias_mm") else EP.SEAT_DX_M * 1e3)   # round 3: the load row's
    T_c = T_g.copy()                                                                               # descent_bias_mm (default 1)
    T_c[:3, 3] += bias_mm / 1e3 * x_dir
    labels = EP.rack_labels(len(arm.checker.parts["LowerRack"]))
    sk = EP.seat_search(arm, "plate", "dish", T_c, v, EP.scene_targets(arm, ignore=("dish",), statics=False), labels, np)
    out = {"G0": False, "G1": False, "spin_deg": spin_deg, "seat_mm": sk.get("seat_mm"), "supports": sk.get("supports"),
           "starts": [], "ok_starts": [], "descent_bias_mm": round(float(bias_mm), 2)}
    if sk.get("seat_mm") is None:
        out["why"] = "no seat: the plate alone along the tine axis touches nothing"
        return out
    out["seat_at_goal"] = sg = seat_at_goal(sk, "LowerRack", labels)
    if not sg["ok"]:                                   # D3: a perch (a dish among the supports) or a seat off its goal
        out.update(G0=True, why=sg["why"], start_blockers=[], descent_blockers=[])
        return out
    T_ts = sk.pop("T_seat") @ np.linalg.inv(M)
    held = {"held": ("plate", M, "dish"), "ignore": ("dish",)}
    out["G0"] = bool(W.kr.ik_all(T_ts, q_seed))
    if not out["G0"]:
        out["why"] = "G0: no IK at the seat"
        return out
    targets = EP.scene_targets(arm, ignore=("dish",))
    drop_to = {"LowerRack"} | {n_ for n_ in arm.dishes if n_ != "dish"}   # D2: the rule's targets a drop may stop above
    cands, hits, dbreaks, fails = [], {}, {}, {}
    h_s = EP.SEAT_START_ABOVE_M
    while h_s <= EP.SEAT_START_MAX_M + 1e-9:
        T_s = T_ts.copy()
        T_s[:3, 3] += h_s * v
        for q in W.kr.ik_all(T_s, q_seed):
            if not arm.free(q, **held):
                for b in pairs_of(arm, q, **held):
                    hits[b] = hits.get(b, 0) + 1
                continue
            prev, brk, n_dn, hh = q, None, int(round(h_s * 1e3)), []
            for j in range(1, n_dn + int(round(EP.DESCENT_BELOW_M * 1e3)) + 1):
                T_j = T_s.copy()
                T_j[:3, 3] -= j * 1e-3 * v
                qq = W.kr.ik(T_j, prev)
                if qq is None or float(np.abs(qq - prev).max()) > .1:
                    brk = "IK branch jump"
                    break
                if j <= n_dn and (j % 5 == 0 or j == n_dn):
                    hh = EP.seat_line_hits(arm, qq, tool, targets, np)
                    if hh:
                        brk = f"final descent {n_dn - j} mm above the seat: {hh[0]}"
                        break
                prev = qq
            if brk is None:
                cl = arm.held_clearance(q, held["held"], ignore=("dish",), cutoff=EP.CLEAR_CUTOFF_M)
                cands.append((h_s, T_s, q, min(float(cl[0]), EP.CLEAR_CUTOFF_M), cl[1]))
                break
            k_ = brk if brk.startswith("IK") else brk.split(": ")[-1].rsplit(" ", 2)[0]
            dbreaks[k_] = dbreaks.get(k_, 0) + 1
            if guided and not brk.startswith("IK") and all(h_.split(" ")[0].split("|")[-1] in drop_to for h_ in hh):
                fails.setdefault(h_s, []).append(q)    # D2: a passing start stopped only by the 2 mm rule at a dish or the rack
        h_s = round(h_s + EP.SEAT_START_STEP_M, 3)
    cands.sort(key=lambda c_: (-c_[3], c_[0]))
    out.update(G1=bool(cands), start_blockers=sorted(hits.items(), key=lambda x: -x[1])[:4],
               descent_blockers=sorted(dbreaks.items(), key=lambda x: -x[1])[:4],
               start_clear={f"{round(c_[0] * 1e3)}": [round(c_[3] * 1e3, 1), c_[4]] for c_ in cands}, mode="seat", release_mm=0)
    if not cands and fails:                            # D2: the tine-guided release above the seat
        dr = tine_guided(EP, W, arm, d, T_ts, M, v, fails, tool, targets, held)
        out["drop"] = {"tried": dr["tried"], "tine_tips_z_m": dr["tips"]["tips_z_m"], "tip_z_m": dr["tips"]["tip_z_m"],
                       "guide_min_mm": round(PLATE_DROP_GUIDE_M * 1e3, 1), "rule": PLATE_DROP_RULE}
        cands = dr["cands"]
        if not cands:
            low = min((r_ for r_ in dr["tried"] if r_["release_mm"] is not None), key=lambda r_: r_["release_mm"], default=None)
            out.update(mode=None, release_mm=None, why=(
                f"not tine-guided: no start height ({len(dr['tried'])} tried) keeps the closed hand, the open gripper and SEAT_LINKS "
                f"2 mm from the scene at any release height ({[r_['first_violation'] for r_ in dr['tried']][:2]})" if low is None else
                f"not tine-guided: the lowest clear release, {low['release_mm']} mm above the seat (start +{low['start_mm']} mm), "
                f"leaves the plate's lowest rim point {low['guided_mm']} mm below the slot's tine tips (under "
                f"{PLATE_DROP_GUIDE_M * 1e3:.0f} mm: no free fall); below it {low['first_violation']}"))
            return out
        out.update(G1=True, mode="drop", **{k: cands[0][5].get(k) for k in DROP_KEYS},
                   start_clear={f"{round(c_[0] * 1e3)}": [round(c_[3] * 1e3, 1), c_[4]] for c_ in cands})
    if not cands:
        out["why"] = (f"G1: no descent start {EP.SEAT_START_ABOVE_M * 1e3:.0f}..{EP.SEAT_START_MAX_M * 1e3:.0f} mm above the seat "
                      f"with a free IK (the held plate {EP.CARRY_CLEAR_M * 1e3:.0f} mm from the scene) and the final descent under "
                      f"the 2 mm rule; start hits {out['start_blockers'][:3]}, descent {out['descent_blockers'][:3]}")
        return out
    for c_ in cands[:EP.DROP_TRIES]:
        h_s, T_s, q_s, cl_m, cl_to = c_[:5]
        h_full = (1.25 - float(T_s[2, 3])) / float(v[2])
        n_up = max(3, int(round(h_full / EP.SEAT_LINE_STEP_M)))
        prev, q_top, h_top, why_up = q_s, None, 0., None
        for j in range(1, n_up + 1):
            T_j = T_s.copy()
            T_j[:3, 3] += h_full * j / n_up * v
            q = W.kr.ik(T_j, prev)
            if q is None or float(np.abs(q - prev).max()) > EP.SEAT_LINE_JUMP_RAD:
                why_up = f"IK branch jump {h_full * j / n_up * 1e3:.0f} mm up the axis"
                break
            if not arm.free(q, **held):
                why_up = f"collides {h_full * j / n_up * 1e3:.0f} mm up the axis: {pairs_of(arm, q, **held)[:2]}"
                break
            prev, q_top, h_top = q, q, h_full * j / n_up
        T_top = T_s.copy()
        T_top[:3, 3] += h_top * v
        st = {"h_s_mm": round(h_s * 1e3), "held_clear_mm": round(cl_m * 1e3, 1), "held_clear_to": cl_to,
              "line_up_mm": round(h_top * 1e3), "stopped": why_up, "q_s": q_s, "T_s": T_s, "q_top": q_top, "T_top": T_top, "v": v,
              **(c_[5] if len(c_) > 5 else {"mode": "seat", "release_mm": 0})}   # D2: the drop's release record per start
        out["starts"].append(st)
        if q_top is not None and h_top >= EP.SEAT_LINE_MIN_M - 1e-6:
            out["ok_starts"].append(st)
    if not out["ok_starts"]:
        out["G1"] = False
        out["why"] = f"the line up the tine axis is under {EP.SEAT_LINE_MIN_M * 1e3:.0f} mm free: {[s_['stopped'] for s_ in out['starts']]}"
    return out


def carried_drop(rel, tr):
    """D2: the release record (DROP_KEYS) of the drop start the I4 carry ``tr`` reached (its h_s_mm), else of the first ok start."""
    ss = rel.get("ok_starts") or []
    s_ = next((x for x in ss if tr and tr.get("ok") and x.get("h_s_mm") == tr.get("h_s_mm")), ss[0] if ss else None)
    return {} if s_ is None or s_.get("mode") != "drop" else {k: s_.get(k) for k in DROP_KEYS}


def i4_carry(W, arm, rel, q_lift, held, retry_s=0.):
    """The I4 carry from the lift: leg 1 to the top of the tine-axis line (straight, else RRT-Connect 45 s) and the line down the
    tine axis to the descent start, for each ok start in turn (the held dish under arm.held_clear_m throughout). ``retry_s`` > 0
    (plans/2026-11-04-fifteen-of-fifteen.md, the narrow passages: frigidaire_mount_sweep.py gives collide.NARROW_RETRY_S to the
    plates it adds): no start passed and some starts' leg 1 found no path -> collide.rrt_connect's one retry of ``retry_s`` s
    rooted at the top of every such start whose line down is clear (the per-start 45 s searches were its first search, so
    time_s 0; the same held clearance, margins and 0.02 rad edges), the carry ending at the start the path reached; the record
    "retry" (budget, seconds, roots, found). 0 (the default) = no retry, as before."""
    tr, lost = None, []
    for s_ in rel["ok_starts"]:
        kind_, sec = leg_ok(W, arm, q_lift, s_["q_top"], held, 45.)
        legs = [{"leg": "leg 1 to the top of the tine-axis line", "path": kind_ or "rrt: none", "s": sec}]
        tr = {"ok": False, "legs": legs, "why": None, "h_s_mm": s_["h_s_mm"], "held_clear_mm": s_["held_clear_mm"]}
        if kind_ is None:
            tr["why"] = "no leg 1 to the top of the tine-axis line (RRT-Connect 45 s)"
            lost.append(s_)
            continue
        n_dn = max(3, int(round(float((s_["T_top"][:3, 3] - s_["T_s"][:3, 3]) @ s_["v"]) / .005)) + 1)
        dn, why = line_ok(W, arm, s_["T_top"], s_["T_s"], s_["q_top"], n_dn, held)
        legs.append({"leg": "down the tine axis", "path": "line" if dn is not None else "blocked"})
        tr.update(ok=dn is not None, why=None if dn is not None else f"the line down the tine axis: {why}")
        if tr["ok"]:
            break
    if retry_s > 0. and lost and not (tr and tr["ok"]):
        roots = []                                     # the lost starts whose line down the tine axis is clear (it does not
        for s_ in lost:                                # depend on leg 1)
            n_dn = max(3, int(round(float((s_["T_top"][:3, 3] - s_["T_s"][:3, 3]) @ s_["v"]) / .005)) + 1)
            if line_ok(W, arm, s_["T_top"], s_["T_s"], s_["q_top"], n_dn, held)[0] is not None:
                roots.append(s_)
        t0 = time.monotonic()
        path = (W.C.rrt_connect(arm, np.asarray(q_lift, dtype=float), [np.asarray(s_["q_top"], dtype=float) for s_ in roots],
                                time_s=0., collide_kw=held, seed=1, shortcut_s=SAMPLE_SHORTCUT_S, retry_s=float(retry_s))
                if roots else None)
        sec = round(time.monotonic() - t0, 1)
        k = (None if path is None else
             int(np.argmin([float(np.abs(np.asarray(path[-1]) - np.asarray(s_["q_top"])).max()) for s_ in roots])))
        rec = {"budget_s": float(retry_s), "seconds": sec, "lost": len(lost), "roots": len(roots),
               "roots_start_mm": [s_["h_s_mm"] for s_ in roots], "found": path is not None,
               "reached_start_mm": None if k is None else roots[k]["h_s_mm"],
               "rule": "collide.rrt_connect's narrow-passage retry (seed 2) rooted at the top of every passing descent start whose "
                       "leg 1 found no path in 45 s and whose line down the tine axis is clear; the same held clearance, margins "
                       "and edge resolution"}
        if path is not None:
            s_ = roots[k]
            tr = {"ok": True, "why": None, "h_s_mm": s_["h_s_mm"], "held_clear_mm": s_["held_clear_mm"], "retry": rec,
                  "legs": [{"leg": "leg 1 to the top of the tine-axis line (the narrow retry)", "path": "rrt", "s": sec},
                           {"leg": "down the tine axis", "path": "line"}]}
        else:
            tr = dict(tr, retry=rec, why=f"{tr.get('why') or 'no carry'}; the narrow retry (RRT-Connect {retry_s:.0f} s rooted at "
                                         f"{len(roots)} start(s)) found no path either")
    return tr or {"ok": False, "legs": [], "why": "no descent start"}


def sample_gate_job(args):
    """One dish of the sample load: the pick at the spot (G1, G2 lines and the home transit), the kind's release (G0, G1,
    the descent line) and the carry from the lift to the descent start (G2), hardware only (no other dish in the scene).
    Rule 1 (plans/2026-11-04-bowl-carry-clearance.md, amended): from the pick's lift on, the held dish EP.CARRY_CLEAR_M from the
    scene (arm.held_clear_m) at every descent start candidate and on every carry sample, the RRT-Connect edges at 0.02 rad;
    the descent start = the passing one with the largest held-dish clearance. A load dish's move I4-lean30: the plate leaning
    on its presentation's backstop, the edge pinch (both rolls), the I4 seat under the 2 mm final-descent rule with the pinch
    spun PINCH_SPINS_DEG from the rim point highest (the first that passes); I4-wall20: the pinch at the load row's
    "pinch_spin_deg" at the goal (the access order's), else the pick's; D-ring-build / D-side-down-build: the build's own release
    (ring_release with its move pose and stop) and the open-jaw release check there. The guarded insertion (INSERT_MOVES, plans/
    2026-11-04-fifteen-of-fifteen.md: D-ring-insert the axial hold's D-ring path, D-side-insert the cup's, D-ring-side-insert the
    side ring hold's; the build's move and stop where the row has a build_release): the release at the stop, the open-jaw check
    there along the CUP_BACK_M back-off (-TCP z) and the rise, which a pass needs (the episode's ring_drop); D4: the retreat after
    the open is the first clear of RETREATS over the passing starts in the gate's order (insert_release_check; the row's
    release_check.variant and retreat_choice). A D-ring-side row's "ring_side_order" orders its starts as the episode's
    --ring-side-order; a plate row's "_retry_s" (the mount sweep's) gives i4_carry its narrow-passage retry. D3: a plate's seat
    off its goal (seat_at_goal) is G0; D2: an I4-wall20 plate whose descent stops only on the 2 mm rule at a dish or the rack is
    released above its seat when tine-guided (release.mode "drop", release_mm, guided_mm; else G0 "not tine-guided"; a seat
    release is release.mode "seat"). The row's "basket": the World's (use_basket, --no-basket)."""
    d, frames, load_dir = args
    sys.path.insert(0, str(ROOT / "code/execution/frigidaire"))
    import frigidaire_robot_episode as EP
    if "w" not in _W:
        _W["w"] = gate_world(load_dir)
    W = _W["w"]
    from dishsim_frigidaire.robot import collide as C
    from dishsim_frigidaire.robot.rig import T_of
    t0 = time.monotonic()
    kind, arm = d["kind"], W.arms[d["rack"]]
    if d.get("reject"):                               # plans/2026-11-04-s0-rerun.md: a reject row of the load (no check, no Kit run)
        return {"dish": d["name"], "kind": kind, "rack": d["rack"], "slot": d["slot"], "order_index": d["order_index"],
                "move": d.get("move"), "gate": "reject", "pass": False, "why": d["reject"]["cause"],
                "blocker": d["reject"].get("blockers") or [], "reject": d["reject"], "pick": {"pass": 0, "of": 0},
                "release": {}, "basket": bool(W.basket), "seconds": 0.}
    arm.set_dishes({})
    arm.held_clear_m = None                           # the pick's checks; rule 1 from the lift on
    arm.held_margin = C.HELD_ARM_MARGIN_M if kind in ("bowl", "plate") else C.MARGIN_M   # the episode's D foot-out / I rule
    down = d.get("move") in ("D-side-down", "D-side-down-build", "D-side-insert")   # plans/2026-11-04-cups-mouth-down.md: the
                                                      # D-ring path, the cup (D-side-insert: its guarded insertion, ``insert``)
    edge = d.get("move") in ("I4-lean30",) + WALL_MOVES   # plans/2026-11-04-plate-pick-presentation.md (amended); the wall pinch
    build = d.get("build_release")                    # plans/2026-11-04-deep-sites-gate.md (amended)
    ring_side = d.get("move") in SIDE_MOVES           # plans/2026-11-04-ring-side-hold.md: the side ring hold, D-side-down's path
    insert = d.get("move") in getattr(EP, "INSERT_MOVES", INSERT_MOVES)   # the guarded insertion (the episode's ring_plan and
                                                      # ring_drop): released at the stop, then the back-off and the rise
    side_order = d.get("ring_side_order") or "pitch"  # D-ring-side's start order (the episode's --ring-side-order)
    M, R0, src = W.holds["cup_down" if down else kind]
    bt_row = d.get("side_below_top_m") if down else None   # round 3 (cup_05, the user 2026-11-05 00:05 EST): the row's hold height
    if bt_row is not None:                                # and its Isaac gate run (the closed jaws), else today's
        M, R0, src = cup_down_hold(bt_row)
    cup_hulls = row_cup_hulls(W, load_dir, d["hold_gate_run"]) if down and d.get("hold_gate_run") else W.cup_hulls
    q_home = home_q(W)
    row = {"dish": d["name"], "kind": kind, "rack": d["rack"], "slot": d["slot"], "order_index": d["order_index"],
           "move": d["move"], "hold": src, "held_margin_mm": round(arm.held_margin * 1e3, 1),
           "carry_clear_mm": EP.CARRY_CLEAR_M * 1e3, "edge_res_rad": C.EDGE_RES_RAD, "basket": bool(W.basket),
           "scene": "hardware only: the appliance with the dish's rack out and the other pushed in"
                    + ("" if W.basket else " and no SilverwareBasket (--no-basket)")
                    + f", the counter (front edge {COUNTER_FRONT_Y:+.2f} m), the pedestal and the floor; no other dish"}
    if down:                                          # round 3: the cup's hold height and the jaws' gate run (the row's, else today's)
        row.update(side_below_top_m=round(float(EP.SAMPLE_MOVE_ARGS["D-side-down"]["side_below_top"] if bt_row is None else bt_row), 5),
                   hold_gate_run=d.get("hold_gate_run") or EP.CUP_DOWN_GATE_RUN, hold_jaws=cup_hulls[1],
                   hold_fields_from="the load row" if (bt_row is not None or d.get("hold_gate_run")) else "the defaults")
    F_goal = T_of(frames[d["rack"]]["position_m"], frames[d["rack"]]["quaternion_xyzw"])
    if edge:                                          # the leaning plate: the edge pinch per roll, then the I4 per pick and spin
        if W.pres is None:
            raise RuntimeError(f"{d['name']}: move {d['move']} needs {Path(load_dir) / 'presentation.json'}")
        pk = edge_pick(W, W.pres)
        row["pick"] = {k: v for k, v in pk.items() if k not in ("rows", "candidates")}
        row["pick_rows"] = [{k: v for k, v in r.items() if k in ("roll_deg", "ok", "why", "facing", "stage", "move_rad")}
                            for r in pk["rows"]]
        arm = W.arm                                   # the lower rack's arm, the backstop among its statics
        row["release"] = {"G0": False, "G1": False, "why": "no pick"}
        tool = EP.tool_hulls(arm, W.plate_closed[0])
        row["hand"] = W.plate_closed[1]
        tries, best, first, per_roll = [], None, None, {}
        sp_ = d.get("pinch_spin_deg")                 # the wall pinch at the goal: the load row's spin (the access order's), else
        for c_ in pk["candidates"]:                   # the pick's; every passing pick (roll), each over the spins until one passes
            M_c = np.asarray(c_["M"], dtype=float)
            held = {"held": ("plate", M_c, "dish"), "ignore": ("dish",)}
            arm.held_clear_m = EP.CARRY_CLEAR_M
            ok_c = None
            for spin in (PINCH_SPINS_DEG if d["move"] == "I4-lean30" else
                         (float(sp_) if sp_ is not None else c_.get("spin_deg", 0.),)):
                rel = i4_release(W, arm, d, M_c, np.asarray(c_["q_lift"]), F_goal, tool, spin,
                                 guided=d["move"] in WALL_MOVES and not d.get("no_guided"))   # D2: the tine-guided release, the wall pinch's plates (off: --direct-only)
                t_ = {"roll_deg": c_["roll_deg"], "spin_deg": spin, "G0": rel["G0"], "G1": rel["G1"], "why": rel.get("why"),
                      "seat_mm": rel.get("seat_mm"), "start_clear": rel.get("start_clear"), "mode": rel.get("mode")}
                tr = None
                if rel["G1"]:
                    tr = i4_carry(W, arm, rel, np.asarray(c_["q_lift"]), held, retry_s=float(d.get("_retry_s") or 0.))
                    t_["carry"] = {k: tr.get(k) for k in ("ok", "legs", "why", "h_s_mm", "held_clear_mm")}
                    t_.update(carried_drop(rel, tr))
                tries.append(t_)
                first = first or (rel, c_, spin, tr)
                if rel["G1"] and tr and tr["ok"]:
                    ok_c = (rel, c_, spin, tr)
                    break
            arm.held_clear_m = None
            per_roll[str(c_["roll_deg"])] = ({"pass": True, "spin_deg": ok_c[2], "start_mm": ok_c[3].get("h_s_mm"),
                                              "held_clear_mm": ok_c[3].get("held_clear_mm"), "mode": ok_c[0].get("mode"),
                                              **carried_drop(ok_c[0], ok_c[3])} if ok_c else
                                             {"pass": False, "spin_deg": None, "why": tries[-1].get("why") or
                                              (tries[-1].get("carry") or {}).get("why")})
            best = best or ok_c
        arm.statics.pop("Backstop", None)
        for f_ in extra_fixtures(W.pres):             # the presentation's other fixtures leave with the plate
            arm.statics.pop(f_["id"], None)
        row["i4_tries"] = tries
        if best or first:
            rel, c_, spin, tr = best or first
            row["release"] = {k: v for k, v in rel.items() if k not in ("starts", "ok_starts")}
            row["release"].update(pinch_spin_deg=spin, roll_deg=c_["roll_deg"], candidates=len(rel.get("ok_starts") or []),
                                  per_roll=per_roll, roll_order=[r_ for r_, v_ in per_roll.items() if v_["pass"]]
                                  + [r_ for r_, v_ in per_roll.items() if not v_["pass"]],
                                  descent_rule=f"D-seat's final-descent rule ({EP.SEAT_ARM_MARGIN_M * 1e3:.0f} mm, the hand: "
                                               f"{W.plate_closed[1]}) from the start to the seat")
            if rel.get("mode") == "drop":             # D2: the carried start's release (release_mm above the seat, guided_mm, ...)
                row["release"].update(carried_drop(rel, tr), release_rule=PLATE_DROP_RULE,
                                      descent_rule=f"D2's drop rule (the closed hand: {W.plate_closed[1]}, the open gripper hull, "
                                                   f"SEAT_LINKS, {EP.SEAT_ARM_MARGIN_M * 1e3:.0f} mm) from the start to release_mm")
                row["drop_retreat"] = row["release"].get("drop_retreat")   # round 3: EP.DROP_RETREAT_RULE at the carried start
            row["descent_bias_mm"] = rel.get("descent_bias_mm")
            row["transit"] = tr
        rel = row["release"]
        g0 = bool(rel.get("G0"))
        g1 = g0 and bool(rel.get("G1")) and bool(pk.get("pass"))
        g2 = g1 and bool(pk["ok"]) and bool(row.get("transit") and row["transit"]["ok"])
        row.update(gate="G2" if g2 else "G1" if g1 else "G0" if g0 else "none", **{"pass": bool(g2)})
        if not pk["ok"]:
            row.update(why=f"pick: {pk['why']}", blocker=pk.get("blocker") or [])
        elif not g1:
            row.update(why=f"release: {rel.get('why')}", blocker=[b for b, _ in (rel.get("start_blockers") or [])]
                       or list((rel.get("seat_at_goal") or {}).get("not_rack") or []))
        elif not g2:
            row.update(why=f"carry: {(row.get('transit') or {}).get('why') or 'not planned'}", blocker=[])
        else:
            row.update(why=None, blocker=[])
        row["seconds"] = round(time.monotonic() - t0, 1)
        return row
    if ring_side:                                     # one wrist side per job (sample_gate_main runs both and merges them)
        pk = foot_side_sample_pick(W, arm, q_home, d.get("_roll", "base"), W.side_rank)
        if pk.get("M") is not None:
            M = pk.pop("M")
            src = (f"the nominal foot_side_grasps hold (yaw {pk['best_yaw_k']}, the wrist toward the {pk['roll']}; the pads on the "
                   "ring's outer face, no Isaac gate before the window)")
            row["hold"] = src
    else:
        pk = sample_pick(W, arm, kind, q_home, down=down, below_top=bt_row)
    row["pick"] = {k: v for k, v in pk.items() if k != "rows"}
    row["pick_rows"] = [{k: v for k, v in r.items() if k in ("yaw_k", "wrist_side", "ok", "why", "facing", "hand_bowl_mm",
                                                             "hand_bowl_at_grasp_mm", "arm_clear_mm", "hand_counter_mm")}
                        for r in pk["rows"]]
    q_seed = np.asarray(pk["q_lift"]) if pk.get("q_lift") is not None else q_home
    arm.held_clear_m = EP.CARRY_CLEAR_M               # rule 1: the release search and the carry
    rel = (ring_release(W, arm, d, M, q_seed, F_goal, seat_tool=EP.tool_hulls(arm, W.bowl_side[0]), pitch=True, insert=insert,
                        side_order=side_order) if ring_side else
           ring_release(W, arm, d, M, q_seed, F_goal, insert=insert) if kind == "bowl" else
           ring_release(W, arm, d, M, q_seed, F_goal, kind="cup", aboves=EP.CUP_DOWN_ABOVES_M,
                        seat_tool=EP.tool_hulls(arm, cup_hulls[0]), insert=insert) if down else
           cup_release(W, arm, d, M, R0, q_seed) if kind == "cup" else plate_release(W, arm, d, M, q_seed))
    row["release"] = {k: v for k, v in rel.items() if k not in ("ranked", "starts", "ok_starts")}
    if insert:
        row["release"]["guarded_insert"] = (("released at the build's stop" if build else "released at the goal")
                                            + " (no raise), then the open jaws' first clear retreat (D4, release_check.variant: "
                                              f"a {EP.CUP_BACK_M * 1e3:.0f} mm along -TCP z then up {EP.RING_RISE_M * 1e3:.0f} mm, "
                                              "b up along the insertion axis then the back-off, c up only)")
    if ring_side:
        row["release"]["ring_side_order"] = side_order
    if down or ring_side:
        row["release"]["descent_rule"] = ("the descent start at the 15 mm carry margins; the line down to the goal: D-seat's "
                                          "final-descent rule, the hand (closed jaws + wrist 3) and the links SEAT_LINKS "
                                          f"{EP.SEAT_ARM_MARGIN_M * 1e3:.0f} mm from the scene; the hand: "
                                          f"{(W.bowl_side if ring_side else cup_hulls)[1]}")
    if ring_side:
        row["release"]["rank_rule"] = ("D-ring-side: the passing starts (rule 1, every 50..150 mm start x 24 spins) ordered by the "
                                       "goal tilt's pitch about the hand's closing axis (smallest first, 0.5 deg bins: a roll of "
                                       "the hand where one exists), then straight carry legs first, then rule 1's order")
    row["release"]["candidates"] = len(rel.get("ranked") or rel.get("ok_starts") or [])
    tr = None
    if pk["ok"] and rel["G1"]:                         # G2: the carry from the lift pose to the descent start
        held = {"held": (kind, M, "dish"), "ignore": ("dish",)}
        T_lift = np.asarray(pk["T_lift"])
        if kind == "bowl" or down:
            c_c = np.r_[W.cent[kind], 1.]

            cap_ = {"z": 0.}                          # plans/2026-11-04-s0-rerun.md: the episode's bowl tilt cap per start

            def carry_ok(q):                           # the episode's ring_drop rule: mouth down, the hand above the dish
                T_t = arm.link_poses(q)["wrist_3_link"] @ arm.T_w3_tcp
                T_b = T_t @ M
                return bool(T_b[2, 2] < 0. and T_t[2, 3] > (T_b @ c_c)[2] and (kind != "bowl" or T_b[2, 2] <= cap_["z"]))
            arm.carry_ok = carry_ok
            ok_, n_chk = [], 0                        # the coordinator's preference (2026-11-04, the episode's straight_first):
            for i_, cd_ in enumerate(rel["ranked"]):  # the passing starts whose carry legs are all straight joint moves first
                if kind == "bowl":
                    cap_["z"] = EP.bowl_tilt_cap_z(cd_["T_goal"])
                ab_ = np.array(cd_["T_start"], dtype=float, copy=True)
                ab_[2, 3] = 1.25
                qa_ = W.kr.ik(ab_, cd_["q_start"])
                qa_ = qa_ if qa_ is not None and arm.free(qa_, **held) else None
                lg_ = [(q_seed, qa_), (qa_, cd_["q_start"])] if qa_ is not None else [(q_seed, cd_["q_start"])]
                n_chk += 1
                if all(arm.path_free(np.asarray(a_), np.asarray(b_), step=.02, **held) for a_, b_ in lg_):
                    ok_.append(i_)
                    if len(ok_) >= EP_DROP_TRIES:
                        break
            order_ = (EP.pitch_order(rel["ranked"], ok_) if ring_side and side_order == "pitch" else
                      ok_ + [i_ for i_ in range(len(rel["ranked"])) if i_ not in ok_])
            row["start_choice"] = {"straight_candidates": len(ok_), "checked": n_chk, "ranked": len(rel["ranked"]),
                                   "rule1_rank_of_first": (order_[0] + 1) if order_ else None,
                                   "first_all_straight": bool(order_ and order_[0] in ok_)}
            ordered_, rchk_ = [rel["ranked"][i_] for i_ in order_], {}
            closed_ins = None                         # the partial open (the user, 2026-11-04 22:20 EST): the hold's closed jaws, the
            if insert:                                # predicted stop's tool and the partial open's trial (the episode's ring_hulls)
                closed_ins = (cup_hulls[0] if down else W.bowl_side[0] if ring_side else gate_axial_hulls(W, load_dir)[0])
            if insert:                                # D4: the starts in the gate's order whose open-jaw retreat is clear
                clear_ = []                           # (insert_release_check: a, b, c), up to EP_DROP_TRIES; the carry tries
                for x in ordered_:                    # those (none clear: the gate's order, the release check then fails)
                    rchk_[id(x)] = insert_release_check(W, arm, kind, x, closed=closed_ins)
                    if rchk_[id(x)]["ok"]:
                        clear_.append(x)
                        if len(clear_) >= EP_DROP_TRIES:
                            break
                drop_ok = []
                if not clear_ and closed_ins is not None:   # EP.INSERT_DROP_RULE: no start clears the sensor-trip release
                    dr_tried = []
                    for x in ordered_:
                        dr_ = insert_drop_gate(W, arm, kind, x, closed_ins, d["rack"])
                        dr_tried.append({k_: dr_.get(k_) for k_ in ("spin_deg", "start_mm", "ok", "release_mm", "stop_mm", "drop_mm",
                                                                     "guided_mm", "variant", "first_violation", "why")})
                        dr_tried[-1]["open_mode"] = (dr_.get("open") or {}).get("open_mode")
                        if dr_["ok"]:
                            rchk_[id(x)] = dr_
                            drop_ok.append(x)
                    drop_ok.sort(key=lambda x: (rchk_[id(x)]["release_mm"],
                                                0 if (rchk_[id(x)]["open"] or {}).get("open_mode") == "full" else 1,
                                                -(x.get("clear_m") or 0.), x["above_m"], x["spin_deg"]))
                    row["drop_choice"] = {"candidates": len(drop_ok), "tried": dr_tried, "rule": EP.INSERT_DROP_RULE,
                                          "tips": (rchk_[id(drop_ok[0])] if drop_ok else {}).get("tips")}
                row["retreat_choice"] = {
                    "checked": len(rchk_), "clear": len(clear_), "ranked": len(ordered_), "drop_candidates": len(drop_ok),
                    "first_clear_rank": next((i_ + 1 for i_, x in enumerate(ordered_) if clear_ and x is clear_[0]), None),
                    "variants": {v_: sum(1 for r_ in rchk_.values() if r_.get("variant") == v_) for v_ in RETREATS},
                    "open_modes": {m_: sum(1 for r_ in rchk_.values() if (r_.get("open") or {}).get("open_mode") == m_)
                                   for m_ in OPEN_MODES},
                    "rule": "D4: the passing starts in the gate's order checked for a clear retreat after the open (the first clear "
                            "of RETREATS: a back-off then rise, b rise along the insertion axis then back-off, c rise only) until "
                            f"{EP_DROP_TRIES} are clear; the carry tries those in order (none clear: the gate's order, and the "
                            "release check records all three variants)"}
                ordered_ = clear_ or drop_ok[:EP_DROP_TRIES] or ordered_   # the guided release's candidates in its order
            tries_, retry_, tried_ = ordered_[:EP_DROP_TRIES], None, []
            while tries_:
                c_ = tries_.pop(0)
                tried_.append(c_)
                if kind == "bowl":
                    cap_["z"] = EP.bowl_tilt_cap_z(c_["T_goal"])
                bud_ = EP.RING_RETRY_RRT_S if retry_ else SAMPLE_TRANSIT_S
                tr = transit_check(W, arm, q_seed, T_lift, c_["q_start"], c_["T_start"], held, rise_to=None, budget=bud_)
                tr.update(spin_deg=c_["spin_deg"], descent_start_mm=round(c_["above_m"] * 1e3),
                          held_clear_mm=None if c_.get("clear_m") is None else round(c_["clear_m"] * 1e3, 1),
                          held_clear_to=c_.get("clear_to"), rrt_budget_s=bud_,
                          **({k_: c_[k_] for k_ in ("pitch_deg", "roll_deg", "tilt_deg")} if ring_side else {}))
                if kind == "bowl":
                    tr["tilt_cap_deg"] = round(math.degrees(math.acos(min(1., -(cap_["z"] - 1e-4)))), 2)
                    tr["goal_tilt_deg"] = round(math.degrees(math.acos(min(1., -float(c_["T_goal"][2, 2])))), 2)
                if tr["ok"]:
                    break
                row.setdefault("carry_fails", []).append({k_: tr.get(k_) for k_ in ("spin_deg", "descent_start_mm", "why", "legs")})
                if not tries_ and kind == "bowl" and retry_ is None:   # the episode's one retry at RING_RETRY_RRT_S (compute only):
                    retry_ = next((x for x, f_ in zip(tried_, row["carry_fails"])          # the first start whose RRT found nothing
                                   if any(l_.get("path") == "rrt: none" for l_ in f_.get("legs") or [])), None)
                    tries_ = [retry_] if retry_ is not None else []
            if kind == "bowl" and rel["ranked"]:
                c1_ = next((x for x in rel["ranked"] if tr and tr["ok"] and x["spin_deg"] == tr.get("spin_deg")
                            and round(x["above_m"] * 1e3) == tr.get("descent_start_mm")), rel["ranked"][0])
                row["tilt_cap"] = {"cap_deg": round(math.degrees(math.acos(min(1., -(EP.bowl_tilt_cap_z(c1_["T_goal"]) - 1e-4)))), 2),
                                   "goal_tilt_deg": round(math.degrees(math.acos(min(1., -float(c1_["T_goal"][2, 2])))), 2),
                                   "flat_cap_deg": EP.BOWL_CARRY_TILT_MAX_DEG, "retry": None if retry_ is None else
                                   {"spin_deg": retry_["spin_deg"], "budget_s": EP.RING_RETRY_RRT_S, "ok": bool(tr and tr["ok"])},
                                   "rule": "the held bowl's tilt from straight down at most max(20 deg, its tilt at the descent "
                                           "start) on every carry configuration (the coordinator's option 1, 2026-11-04)"}
            arm.carry_ok = None
            if down or build or ring_side or insert:    # the supported-release check at the carried spin (else the first)
                c_ = next((x for x in rel["ranked"] if tr and x["spin_deg"] == tr.get("spin_deg")
                           and round(x["above_m"] * 1e3) == tr.get("descent_start_mm") and tr["ok"]), rel["ranked"][0])
                row["release_check"] = ((rchk_.get(id(c_)) or insert_release_check(W, arm, kind, c_, closed=closed_ins)) if insert
                                        else open_release_check(W, arm, kind, c_, back=down or ring_side or insert))   # D4
                if insert:                            # the row's open (the user, 2026-11-04 22:20 EST): the carried start's
                    op_ = row["release_check"].get("open") or {}
                    row["release"].update(open_mode=op_.get("open_mode"), open_mm=op_.get("open_mm"), open_hull=op_.get("open_hull"),
                                          open_theta_rad=op_.get("open_theta_rad"), release_stop=row["release_check"].get("stop"),
                                          release_judged_at=row["release_check"].get("judged_at"),
                                          mode="drop" if row["release_check"].get("mode") == "drop" else "seat",
                                          stop_mode="guided" if row["release_check"].get("mode") == "drop" else "predicted",
                                          stop_rule=getattr(EP, "INSERT_STOP_RULE", None))   # round 3: the position stop there
                    if row["release_check"].get("mode") == "drop":   # EP.INSERT_DROP_RULE: the guided release's record
                        rc_d = row["release_check"]
                        row["release"].update({k_: rc_d.get(k_) for k_ in ("release_mm", "stop_mm", "drop_mm", "guided_mm",
                                                                             "first_violation", "variant", "tips_z_m", "rim_low_z_m")},
                                              rule=EP.INSERT_DROP_RULE)
        elif kind == "cup":
            for c_ in rel["ranked"]:
                tr = transit_check(W, arm, q_seed, T_lift, c_["q"], c_["T"], held, rise_to=1.25)
                tr.update(h_m=c_["h_m"], yaw_k=c_["yaw_k"])
                if tr["ok"]:
                    break
        else:
            tr = i4_carry(W, arm, rel, q_seed, held)
        row["transit"] = tr
    arm.held_clear_m = None
    g0 = bool(rel["G0"])
    g1 = g0 and bool(rel["G1"]) and bool(pk.get("pass"))
    g2 = g1 and bool(pk["ok"]) and bool(tr and tr["ok"])
    if (build or insert or ring_side or down) and g2 and not (row.get("release_check") or {}).get("ok"):
        g2 = False                                     # the build release and the guarded insertion also need the open jaws
                                                       # clear at their release; since 2026-11-05 (plans/2026-11-05-hardware-aware-
                                                       # sampler.md) D-ring-side and D-side-down too (the open-jaw check at the goal)
    row.update(gate="G2" if g2 else "G1" if g1 else "G0" if g0 else "none", **{"pass": bool(g2)})
    if not pk["ok"]:
        row.update(why=f"pick: {pk['why']}", blocker=pk.get("blocker") or [])
    elif not rel["G1"] or not g0:
        row.update(why=f"release: {rel['why']}", blocker=rel.get("blocker") or [])
    elif not (tr and tr["ok"]):
        row.update(why=f"carry: {tr['why'] if tr else 'not planned'}", blocker=[])
    elif not g2 and insert:                            # D4: no retreat clear at the carried start
        rc_ = row["release_check"]
        pm_ = (rc_.get("modes") or {}).get("partial")
        dc_ = row.get("drop_choice")
        dtxt_ = "" if not dc_ else (f"; the guided release: {dc_['candidates']} candidates (first tried: "
                                    f"{(dc_['tried'] or [{}])[0].get('why')})")
        row.update(why=f"release check: the open jaws at the guarded insertion release, no clear retreat at spin {rc_['spin_deg']} "
                       f"(+{rc_.get('start_mm')} mm{', the predicted stop ' + str((rc_.get('stop') or {}).get('h_mm')) + ' mm' if rc_.get('stop') else ''}): "
                       + (rc_.get("why") or "; ".join(f"({v_}) {r_['stages']}" for v_, r_ in (rc_.get("variants") or {}).items()))
                       + ("" if not pm_ else f"; the partial open ({(pm_.get('open') or {}).get('open_mm')} mm): "
                          + (pm_.get("why") or "; ".join(f"({v_}) {r_['stages']}" for v_, r_ in (pm_.get("variants") or {}).items())))
                       + dtxt_, blocker=[])
    elif not g2:
        row.update(why=f"release check: the open jaws at the {'guarded insertion' if insert else 'build' if build else 'goal'} release "
                       f"{row['release_check']['stages']}", blocker=[])
    else:
        row.update(why=None, blocker=[])
    row["seconds"] = round(time.monotonic() - t0, 1)
    return row


EP_DROP_TRIES = 6                     # = frigidaire_robot_episode.DROP_TRIES (the descent starts tried per dish)
WRIST_SIDES = ("base", "mouth")       # = grasp.WRIST_SIDES (the side ring hold's two rolls)
GATE_RANK = {"G2": 3, "G1": 2, "G0": 1, "none": 0, "reject": -1}


def merge_rolls(rows):
    """D-ring-side (plans/2026-11-04-ring-side-hold.md): the two wrist-side rows of a bowl merged into one, the first passing side
    (the order of WRIST_SIDES), else the side with the highest gate; "per_roll" keeps both, and pick.ranked lists the chosen side's
    passing yaws first, then the other side's ([yaw_k, wrist_side], the order the episode's pick tries)."""
    out, by = [], {}
    for r in rows:
        if r.get("move") not in SIDE_MOVES:
            out.append(r)
            continue
        if r["dish"] not in by:
            by[r["dish"]] = []
            out.append(r["dish"])
        by[r["dish"]].append(r)
    res = []
    for x in out:
        if not isinstance(x, str):
            res.append(x)
            continue
        rs = by[x]
        best = next((r for r in rs if r.get("pass")), None) or max(rs, key=lambda r: GATE_RANK.get(r.get("gate"), 0))
        others = [r for r in rs if r is not best]
        row = dict(best)
        row["roll"] = best["pick"].get("roll")
        row["pick"] = dict(best["pick"], ranked=[[k, r["pick"].get("roll")] for r in [best] + others for k in r["pick"].get("yaws") or []])
        row["per_roll"] = {r["pick"].get("roll"): {
            "gate": r.get("gate"), "pass": r.get("pass"), "why": r.get("why"), "blocker": r.get("blocker"),
            "pick_pass": f"{r['pick'].get('pass')}/{r['pick'].get('of')}", "pick_yaws": r["pick"].get("yaws"),
            "release_best": (r.get("release") or {}).get("best"), "release_counts": (r.get("release") or {}).get("counts"),
            "release_first_fail": (r.get("release") or {}).get("first_fail"),
            "carry": None if not r.get("transit") else {k_: r["transit"].get(k_) for k_ in ("ok", "legs", "why", "spin_deg",
                                                                                           "descent_start_mm", "pitch_deg",
                                                                                           "roll_deg", "tilt_deg")},
            "release_check_ok": (r.get("release_check") or {}).get("ok"), "seconds": r.get("seconds")} for r in rs}
        row["seconds"] = round(sum(r.get("seconds") or 0. for r in rs), 1)
        res.append(row)
    return res


def sample_gate_main(a):
    """--sample-gate <samples_v1/<id>/load.json>: sample_gate_job per dish on --workers processes -> <load dir>/gate.json, or the
    file --out names (a.gate_out; the access order's gate.json beside the load then stays)."""
    import multiprocessing as mp
    t0 = time.monotonic()
    load = json.loads(a.sample_gate.read_text())
    ds = sorted(load["dishes"], key=lambda x: x["order_index"])
    holds = sample_holds()
    pres = None
    with mp.get_context("fork").Pool(a.workers) as pool:
        if any(d.get("move") == "I4-lean30" for d in ds):   # the plates' presentation, beside the load before the dish jobs (in
            pres = pool.apply(plate_presentation_job)    # a worker: a World built in the parent deadlocks the forked workers)
            (a.sample_gate.parent / "presentation.json").write_text(json.dumps(pres, indent=1) + "\n")
            print(f"[INFO] presentation {pres['name']} yaw {pres['yaw_k']}: plate at {np.round(pres['plate_pose']['position_m'], 4).tolist()}, "
                  f"backstop {pres['backstop']['center_m']} {pres['backstop']['size_m']}, edge {pres['edge']} -> "
                  f"{(a.sample_gate.parent / 'presentation.json').relative_to(ROOT)}", flush=True)
        jobs = [(dict(d, _roll=w) if d.get("move") in SIDE_MOVES else d, load["frames"], a.sample_gate.parent)
                for d in ds for w in (WRIST_SIDES if d.get("move") in SIDE_MOVES else (None,))]
        rows = list(pool.imap(sample_gate_job, jobs, chunksize=1))
    rows = merge_rolls(rows)                         # D-ring-side: one row per bowl from its two wrist sides
    for r in rows:
        print(f"[INFO] {r['dish']} {r['slot']}: gate {r['gate']} pass {r['pass']} pick {r['pick']['pass']}/{r['pick']['of']} "
              f"release G0 {r['release'].get('G0')} G1 {r['release'].get('G1')} ({r['release'].get('candidates')}) "
              f"best {r['release'].get('best') or {k_: r['release'].get(k_) for k_ in ('pinch_spin_deg', 'roll_deg', 'start_clear') if k_ in r['release']}} "
              f"carry {None if not r.get('transit') else (r['transit']['ok'], [l_['path'] for l_ in r['transit']['legs']])} "
              f"{'' if 'release_check' not in r else 'open-jaw release ' + str({k_: r['release_check'].get(k_) for k_ in ('ok', 'variant', 'spin_deg', 'stages')}) + ' '}"
              f"{'' if (r['release'].get('mode') or 'seat') == 'seat' else 'mode ' + str(r['release'].get('mode')) + ' ' + str({k_: r['release'].get(k_) for k_ in ('release_mm', 'guided_mm', 'first_violation')}) + ' '}"
              f"{r['seconds']} s why {str(r['why'])[:220]}", flush=True)
    res = {"plan": "plans/2026-11-04-grasp-place-s0.md", "load": rel_path(a.sample_gate),
           "plans": ["plans/2026-11-04-bowl-carry-clearance.md", "plans/2026-11-04-plate-pick-presentation.md",
                     "plans/2026-11-04-deep-sites-gate.md"],
           "mount_file": rel_path(SAMPLE_MOUNT), "arm_model": str(ARM_MODEL.relative_to(ROOT)),
           "mount": {k: json.loads(SAMPLE_MOUNT.read_text())["best"].get(k) for k in ("base", "yaw_rad", "spot_xy")},
           "counter_xy": list(COUNTER_XY), "counter_front_y": COUNTER_FRONT_Y, "basket": bool(BASKET),
           "basket_rule": "the SilverwareBasket in the FCL world" if BASKET else
                          "--no-basket (the user's basket-out decision, 2026-11-04 19:20 EST): the SilverwareBasket popped from "
                          "every rack-state checker (strip_basket); the frames keep it, s0's goals and the benchmark keep it",
           "rule_seat": "D3: a plate's predicted seat counts only within 20 mm of its goal along the tine axis with LowerRack "
                        "wires as its only supports (seat_at_goal); else G0, never attempted",
           "holds": {k: v[2] for k, v in holds.items()}, "presentation": None if pres is None else "presentation.json",
           "rule1": "from the pick's lift on, the held dish at least 5 mm (FCL distance) from the scene at every descent start "
                    "candidate and on every carry sample (straight legs checked every 0.02 rad, RRT-Connect edges at 0.02 rad); "
                    "the descent start = the passing candidate 50..150 mm above the seat (the build release: above its stop) "
                    "with the largest held-dish clearance; no passing start = a gate reject",
           "rule": "G0 = an analytic IK at the release pose; G1 = collision-free IK at the pick's pre-grasp, grasp and 0.15 m lift "
                   "and at the kind's descent start (bowl D-ring, plate I4, cup D-side-down: 50..150 mm above the goal) or drop "
                   "pose (cup D-side-mouth) with the dish held; "
                   "G2 = the approach and lift lines, the home -> pre-grasp transit, the carry from the lift to the descent "
                   "start (straight joint moves, else RRT-Connect 15 s per leg, 45 s for the plate's leg 1) and the descent "
                   "line; hardware only (no other dish). pass = G2.",
           "dishes": {r["dish"]: r for r in rows}, "seconds": round(time.monotonic() - t0, 1)}
    if any(d.get("move") in SIDE_MOVES for d in ds):  # the Isaac grasp gate's candidate order: the first Isaac bowl that passes
        res["plan"] = "plans/2026-11-04-ring-side-hold.md"
        res["rule_ring_side"] = ("D-ring-side: per bowl both wrist sides of the side ring hold (12 yaws each, foot_side_pick) and per "
                                 "passing side the D-ring release under D-side-down's final-descent rule ranked by the goal "
                                 "tilt's pitch about the closing axis, the carry under the bowls' carry rule, the open-jaw release "
                                 "check (the open jaws 2 mm from the bowl over the release, the 20 mm back-off and the rise); the "
                                 "bowl's row = its first passing side")
        first = next((res["dishes"][n] for n in load.get("isaac_order") or [] if (res["dishes"].get(n) or {}).get("pass")), None)
        res["gate_pick"] = None if first is None else {
            "dish": first["dish"], "ranked": first["pick"]["ranked"],
            "rule": "the grasp gate (--grasp-family foot-side) tries the first Isaac bowl's pick order: its passing wrist side's yaws "
                    "in the hold sweep's order (the hold is the same contact for every yaw and both sides)"}
    if any(d.get("move") in INSERT_MOVES for d in ds):   # plans/2026-11-04-fifteen-of-fifteen.md
        res["rule_insert"] = ("the guarded insertion (D-ring-insert, D-side-insert, D-ring-side-insert): the descent starts and line "
                              "of the hold's path (the build's move and stop where the row has a build_release), the release AT the "
                              "stop (no raise), the open-jaw check there and along the retreat, which a pass needs; D4: the retreat "
                              "is the first clear of " + "; ".join(f"({v_}) {RETREAT_RULES[v_]}" for v_ in RETREATS)
                              + " (the open jaws 2 mm from the dish, the appliance, the counter, the statics and the scene's other "
                              "dishes, the links SEAT_LINKS 2 mm from all of them) over the passing starts in the gate's order "
                              "(release_check.variant, retreat_choice); none clear: G1")
    if any(d.get("move") in WALL_MOVES and d.get("pinch_spin_deg") is not None for d in ds):
        res["rule_plate_spin"] = "I4-wall20: the wall pinch at the load row's pinch_spin_deg at the goal (the access order's)"
    if any(d.get("move") in WALL_MOVES for d in ds):
        res["rule_plate_drop"] = PLATE_DROP_RULE + " (release.mode seat or drop)"
    out = getattr(a, "gate_out", None) or a.sample_gate.parent / "gate.json"   # --out: another file (the checker's gate_hardware)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(res, indent=1, default=lambda o: o.tolist() if hasattr(o, "tolist") else str(o)) + "\n")
    print(f"[RESULT] PASS sample gate {load['load']}: {sum(r['pass'] for r in rows)}/{len(rows)} pass "
          f"{[r['dish'] for r in rows if r['pass']]} in {res['seconds']} s -> {rel_path(out)}", flush=True)


def main():
    global SEED_FREE, COUNTER_FRONT_Y
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--workers", type=int, default=2)
    ap.add_argument("--kinds", default=",".join(KIND_ORDER))
    ap.add_argument("--targets", default="family,level")
    ap.add_argument("--limit", type=int, default=None, help="dry run: first N jobs only")
    ap.add_argument("--waive-counter-grasp", action="store_true",
                    help="no collision check at the counter pre-grasp, grasp and approach (the plate: the 15 mm margin rejects "
                         "every plate pinch); the rack side is gated as usual; rows go to reach_waived.jsonl")
    ap.add_argument("--out", type=Path, default=None,
                    help=f"the sweeps' output dir (default {OUT.relative_to(ROOT)}); with --sample-gate the gate FILE to write "
                         "(default gate.json beside the load)")
    ap.add_argument("--orient", action="store_true", help="round 3: the orientation sweep (orient.jsonl, orient.json)")
    ap.add_argument("--tilts", default="0,30,60,90,120,150", help="--orient: the tilts swept (deg)")
    ap.add_argument("--plate-in-hand", action="store_true",
                    help="G1/G2 of the plate placed from the hand (measured in-hand pose, no counter grasp) at the plate gaps' "
                         "upright family pose, 12 yaws (inhand.jsonl, inhand.json)")
    ap.add_argument("--seed-free", action="store_true",
                    help="IK: every analytic solution expanded by 2 pi inside the joint limits (kin.expand_2pi_wraps), "
                         "nearest the HOME_Q seed first, instead of one wrap unwrapped toward the seed")
    ap.add_argument("--counter-grid", action="store_true",
                    help="G1/G2 of the pick on a 5 x 5 grid of counter spots per kind (counter_grid.jsonl, counter_grid.json)")
    ap.add_argument("--side-candidates", action="store_true",
                    help="the cup side pinch's candidates (heights x yaws): counter pick, then S1 / S2 per cup of --side-load")
    ap.add_argument("--side-load", type=Path, default=PLACING / "easy_cups/load.json")
    ap.add_argument("--side-out", type=Path, default=PLACING / "side/candidates.json")
    ap.add_argument("--side-spot", action="store_true",
                    help="plans/2026-10-27-cup-side-drop.md: the base-side pick's counter spot sweep and the per-cup drop-IK "
                         "pre-flight of --side-load (--spot-out)")
    ap.add_argument("--spot-out", type=Path, default=PLACING / "side/spot.json")
    ap.add_argument("--counter-front-y", type=float, default=COUNTER_FRONT_Y, help="the counter slab's front edge y (m)")
    ap.add_argument("--foot-candidates", action="store_true",
                    help="plans/2026-10-29-foot-ring-grasp.md: the mouth-down bowl's foot ring pick (F-out, F-in) x 12 yaws at "
                         "the counter spot on the posture_v1 mount, the best candidate per family and its transit (--foot-out)")
    ap.add_argument("--foot-out", type=Path, default=PLACING / "foot/candidates.json")
    ap.add_argument("--foot-families", default="out,in",
                    help="--foot-candidates: the families (out, in; side = the side ring hold of plans/2026-11-04-ring-side-hold.md, "
                         "12 yaws x both wrist sides)")
    ap.add_argument("--mount-search", action="store_true",
                    help="plans/2026-10-27-posture-fix.md: score 25 pedestal mounts (SEARCH_RULE), write --mount-out")
    ap.add_argument("--mount-out", type=Path, default=MOUNT_OUT)
    ap.add_argument("--sample-gate", type=Path, default=None,
                    help="plans/2026-11-04-grasp-place-s0.md: per dish of a samples_v1 load.json the pick at the spot, the kind's "
                         "release and the carry on the posture_v1 mount (G0-G2, hardware only) -> gate.json beside it (--out: "
                         "this file instead, e.g. gate_hardware.json, the access order's gate.json kept)")
    ap.add_argument("--mount", type=Path, default=None,
                    help="plans/2026-11-04-fifteen-of-fifteen.md: --sample-gate and --foot-candidates at this mount file's best "
                         "(default posture_v1.json); its spot_xy (frigidaire_mount_sweep.py's posture_v2.json) moves the counter "
                         "spot and the load's plate presentation with it")
    ap.add_argument("--no-basket", action="store_true",
                    help="plans/2026-11-04-fifteen-of-fifteen.md (the user's basket-out decision): no SilverwareBasket in the FCL "
                         "worlds (use_basket(False): popped from every rack-state checker; the frames keep it); gate.json and the "
                         "rows record \"basket\": false. Default: the basket in")
    a = ap.parse_args()
    a.gate_out = None if a.out is None else a.out if a.out.is_absolute() else ROOT / a.out   # --sample-gate: the gate file
    a.out = OUT if a.out is None else a.gate_out
    SEED_FREE, COUNTER_FRONT_Y = a.seed_free, a.counter_front_y
    if a.no_basket:                                   # before any World (the pools fork after this)
        use_basket(False)
        print("[INFO] --no-basket: no SilverwareBasket in the FCL worlds", flush=True)
    if a.mount is not None:
        m_ = use_mount(a.mount)
        print(f"[INFO] mount {rel_path(SAMPLE_MOUNT)}: base {m_['base']} yaw {m_['yaw_rad']} spot {list(COUNTER_XY)}", flush=True)
    if a.sample_gate is not None:
        a.sample_gate = a.sample_gate if a.sample_gate.is_absolute() else ROOT / a.sample_gate
        return sample_gate_main(a)
    a.out.mkdir(parents=True, exist_ok=True)
    if a.mount_search:
        a.mount_out = a.mount_out if a.mount_out.is_absolute() else ROOT / a.mount_out
        return mount_search_main(a)
    if a.side_spot:
        a.side_load, a.spot_out = (x if x.is_absolute() else ROOT / x for x in (a.side_load, a.spot_out))
        return spot_main(a)
    if a.foot_candidates:
        a.foot_out = a.foot_out if a.foot_out.is_absolute() else ROOT / a.foot_out
        return foot_main(a)
    if a.side_candidates:
        a.side_load, a.side_out = (x if x.is_absolute() else ROOT / x for x in (a.side_load, a.side_out))
        return side_main(a)
    if a.counter_grid:
        return grid_main(a)
    if a.orient:
        return orient_main(a)
    if a.plate_in_hand:
        return inhand_main(a)
    t0 = time.monotonic()
    B = bench()
    kinds = [k for k in KIND_ORDER if k in a.kinds.split(",")]
    allsites = sites(B)
    checkers = {}
    for rack in ("LowerRack", "UpperRack"):
        checkers[rack] = B.bench_checker()
        checkers[rack].update_components(frames_for(B, rack))
        strip_basket(checkers[rack])
    for kind in kinds:
        for s in allsites[kind]:
            frame = frames_for(B, s["rack"])[s["rack"]]
            fam = world_pose(s["local"], frame)
            s["world"] = {"family": fam, "level": None if kind == "plate" else level_pose(checkers[s["rack"]], kind, fam)}
    meta = {"mount": json.loads(MOUNT.read_text())["best"], "counter_xy": COUNTER_XY, "hover_m": HOVER_M, "lift_m": LIFT_M,
            "release_above_m": RELEASE_ABOVE_M, "budget_s": BUDGET_S, "n_yaw": N_YAW, "arm_model": str(ARM_MODEL.relative_to(ROOT)),
            "sites": {k: allsites[k] for k in kinds}, "started": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
            "seed_free": SEED_FREE}
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
