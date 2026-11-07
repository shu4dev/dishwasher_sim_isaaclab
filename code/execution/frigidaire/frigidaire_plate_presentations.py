#!/usr/bin/env python3
"""Kit-free sweep of four plate presentations at the counter spot (plans/2026-11-04-plate-pick-presentation.md, phase 1).

    PYTHONPATH=code/src .venv-kitfree/bin/python code/execution/frigidaire/frigidaire_plate_presentations.py --workers 6

One HOTEC plate at the counter spot frigidaire_robot_reach.COUNTER_XY on the posture_v1 mount, the FCL arm model
(data/results/robot/arm_model.json), the lower rack out and the upper rack pushed in (the grasp gate's racks), the counter
(front edge -0.17 m), the pedestal and the floor. Full clearance (no waiver): the arm and the gripper keep the planner's
15 mm margin from the scene and from the plate; only the gripper may touch the plate, in the last ALLOW_CONTACT_M (60 mm)
of the approach and at the grasp. The held plate keeps the checker's 1 mm overlap rule against the scene and
HELD_ARM_MARGIN_M (2 mm) against the arm links (the episode's plate rule). Presentations, 12 yaws each, both rolls of the
symmetric gripper about its approach per yaw (a yaw passes when either roll passes):
  a        flat at the spot, the KIND_RIM rim pinch from above (grasp.tilted_grasps' tilt rule): today's pick;
  b        flat, its centre on the counter edge (y -0.17, the rim 114 mm past it), the edge pinch at the rim point of yaw k:
           approach radially inward (horizontal), closing along the plate axis, one pad on the rim top and one under the rim;
  c-top    upright on edge between two rails RAIL_GAP_M apart, the edge pinch at the top rim point (from above);
  c-I4     the same stand, frigidaire_robot_episode.plate_holds' mouth hold at the top rim point (the I4 placing hold:
           the KIND_RIM pinch closing radially, approach along the plate's -axis from its mouth face);
  d30-*, d45-*  leaning 30 / 45 deg back onto a backstop box (base face on its top edge), the two top-rim holds as in c;
  c-I4m, d30-I4m, d45-I4m  extra rows (not in the choice of the best): round 12 window 2's I4 hold, the mouth hold mirrored
           through the rim wall (sampler_mirror_tcp, rim 6 mm into the pads, the hand on the base face).
Yaw k: a, b = the rim point of grasp.bowl_grasps' yaw k (k 0 = +y, k 6 = -y, the counter's front); c, d = the plate's mouth
facing azimuth 2 pi k / 12 (k 0 = +x), the stand or backstop turned with it.
Per (presentation, yaw, roll): G1 = a collision-free IK at the pre-grasp hover (0.10 m back along the approach), at the grasp
and at the 0.15 m lift (world up; a: back along its approach, as the episode lifts the rim pinch) with the plate held as
grasped; G2 = the approach line (100 waypoints), the lift line (80) and the home -> hover transit (transit_check, RRT-Connect
15 s a leg). Best = the most passing yaws over the brief's eight rows (ties: the I4 placing hold itself, then c and d). Then
the I4 descent check at plate_05's and plate_03's goals of the s0 load with the best's hold, and with every other presentation
that has a passing pick (frigidaire_robot_reach.plate_release's seat and starts, the start's IK at the 15 mm carry margin, the
line from the start down to the seat under D-seat's final-descent rule SEAT_ARM_MARGIN_M, the line up the tine axis, leg 1
from the pick's lift), plus a calibration with the s0 gate's own hold and pick (with and without the D-seat rule).
Writes data/results/robot/samples_v1/<s0>__fixes/plates/presentations.json and kitfree.md. Reads the reach and episode
scripts as libraries only.
"""
from __future__ import annotations

import argparse
from collections import Counter
import json
import math
from pathlib import Path
import re
import sys
import time

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
sys.path[:0] = [str(ROOT / "code/execution/frigidaire"), str(ROOT / "code/src"), str(ROOT / "code/frigidaire/src")]
import frigidaire_robot_reach as R                  # noqa: E402  read-only helpers (World, home_q, transit_check, leg_ok, ...)
import frigidaire_robot_episode as EP               # noqa: E402  constants and Kit-free helpers (stdlib at module level)

SAMPLE = "frigidaire_fdpc4221as__hotec_wheatstraw__plate5-bowl5-cup5_a3__s0"
LOAD = ROOT / "data/results/robot/samples_v1" / SAMPLE / "load.json"
OUT = ROOT / "data/results/robot/samples_v1" / f"{SAMPLE}__fixes" / "plates"
I4_PLATES = ("plate_05", "plate_03")
N_YAW = R.N_YAW
ROLLS = (0, 180)
RZ_PI = np.diag([-1., -1., 1., 1.])                  # the TCP turned 180 deg about its approach (the same pinch, wrist across)
BRIEF_ROWS = ("a", "b", "c-top", "c-I4", "d30-top", "d30-I4", "d45-top", "d45-I4")
EXTRA_ROWS = ("c-I4m", "d30-I4m", "d45-I4m")         # round 12 window 2's I4 hold (sampler_mirror_tcp 'mirror d6': the mouth
                                                     # hold mirrored through the rim wall, the hand on the base face), the
                                                     # hold that seated the front four in Isaac; extra rows, not in "best"
PRESENTATIONS = BRIEF_ROWS + EXTRA_ROWS
MIRROR_D_MM = 6                                      # the rim 6 mm into the pads (round 12's depth option B)
I4_HOLD_ROWS = ("c-I4", "d30-I4", "d45-I4")          # the tie-break: the hold that is the I4 placing hold itself
B_CENTER_Y = -.17                                    # b: the plate centre on the counter's front edge
RAIL_GAP_M, RAIL_THICK_M, RAIL_LEN_M, RAIL_H_MAX_M, RAIL_CLEAR_M = .012, .010, .120, .020, .0005
BACK_W_M, BACK_T_M, BACK_TOUCH_M = .160, .020, .001  # backstop: width along the plate, depth, the plate's gap to its edge
EDGE_OUT_M, EDGE_REACH_M = .003, .015                # edge pinch: the TCP 3 mm outside the rim edge (the open pads' tips,
                                                     # 18.4 mm beyond the TCP, reach 15 mm inside it); its height = the
                                                     # mid-height along the axis of the plate within those 15 mm
PLATE_MASS_KG = .094625
I4_TRIES_PER_ROLL = 2                                # passing picks tried per roll as the carry's start (best facing first)
CARRY_RRT_S = EP.SEAT_PLAN_RRT_S                     # 45 s: leg 1 to the top of the tine-axis line (the s0 gate's budget)
_W = {}


# ---------------------------------------------------------------------------------------------------------------- world
def world():
    """Per worker: the reach World on the posture_v1 mount, the lower rack's arm with the episode's held-plate margin, the
    plate geometry and the plate gate's closed jaws (the I4 hand)."""
    if "w" in _W:
        return _W["w"]
    from dishsim_frigidaire.robot import collide as C
    m = json.loads(R.SAMPLE_MOUNT.read_text())["best"]
    W = R.World({"base": m["base"], "yaw_rad": m["yaw_rad"]})
    W.mount = {"base": m["base"], "yaw_rad": m["yaw_rad"]}
    W.arm = W.arms["LowerRack"]
    W.arm.held_margin = C.HELD_ARM_MARGIN_M
    P = R.visual_points_of(W.B, "plate")
    r = np.linalg.norm(P[:, :2], axis=1)
    near = P[r >= r.max() - EDGE_REACH_M]
    W.geom = {"rim_r": float(r.max()), "height": float(P[:, 2].max()), "points": P, "centroid": P.mean(axis=0),
              "edge_z": float((near[:, 2].min() + near[:, 2].max()) / 2)}
    W.plate_closed = EP.closed_gripper_hulls(C, np, run=EP.PLATE_GATE[0], traj=EP.PLATE_GATE[1])
    W.q_home = R.home_q(W)
    W.goals = goals(W)
    _W["w"] = W
    return W


def goals(W):
    """{plate: (goal pose in the arm world's LowerRack frame, slot, the rack frame F_r)} of I4_PLATES (the load's goal pose
    carried from the load's rack frame into frames_for's, as ring_release does)."""
    from dishsim_frigidaire.robot.rig import T_of
    load = json.loads(LOAD.read_text())
    F_goal = T_of(load["frames"]["LowerRack"]["position_m"], load["frames"]["LowerRack"]["quaternion_xyzw"])
    fr = R.frames_for(W.B, "LowerRack")["LowerRack"]
    F_r = T_of(fr["position_m"], fr["quaternion_xyzw"])
    out = {}
    for d in load["dishes"]:
        if d["name"] in I4_PLATES:
            out[d["name"]] = (F_r @ np.linalg.inv(F_goal) @ T_of(d["goal_pose"]["pos"], d["goal_pose"]["quat_xyzw"]), d["slot"], F_r,
                              round(float(np.linalg.norm(F_r[:3, 3] - F_goal[:3, 3])) * 1e3, 2))
    return out


def box_body(fcl, name, center, size, Rm):
    """A static box in the checker's body format (ArmWorld.add_static_box, with a rotation)."""
    Rm, c = np.asarray(Rm, dtype=float), np.asarray(center, dtype=float)
    o = fcl.CollisionObject(fcl.Box(*size), fcl.Transform(Rm, c))
    mg = fcl.DynamicAABBTreeCollisionManager()
    mg.registerObjects([o])
    mg.setup()
    corners = np.array([[x, y, z] for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)]) * np.asarray(size) / 2 @ Rm.T + c
    return {"id": name, "kind": "static", "objects": [o], "manager": mg, "bounds": (corners.min(0), corners.max(0)),
            "box": {"center_m": np.round(c, 5).tolist(), "size_m": [round(float(s), 4) for s in size],
                    "axes": np.round(Rm, 5).tolist()}}


def dist(fcl, body, box):
    data = fcl.DistanceData(request=fcl.DistanceRequest())
    body["manager"].distance(box["objects"][0], data, fcl.defaultDistanceCallback)
    return float(data.result.min_distance)


# --------------------------------------------------------------------------------------------------------- presentations
def flat_pose(xy):
    return R.counter_obj(xy)


def edge_pose(W, k, lean_deg):
    """The plate on edge: its mouth facing azimuth 2 pi k / 12, its axis tilted ``lean_deg`` up from horizontal (the plate
    leans back by that much onto its base side), the rim circle's centre over the spot, its lowest point COUNTER_LIFT_M above
    the slab. (T, ah = the mouth's horizontal direction, a = the axis, n = the in-plane up toward the top rim point)"""
    g, psi, ph = W.geom, 2 * np.pi * k / N_YAW, math.radians(lean_deg)
    ah = np.array([math.cos(psi), math.sin(psi), 0.])
    a = math.cos(ph) * ah + math.sin(ph) * np.array([0., 0., 1.])
    n = np.array([0., 0., 1.]) - a[2] * a
    n /= np.linalg.norm(n)
    T = np.eye(4)
    T[:3, 0], T[:3, 1], T[:3, 2] = n, np.cross(a, n), a
    T[:3, 3] = np.array([R.COUNTER_XY[0], R.COUNTER_XY[1], 0.]) - g["height"] * a
    T[2, 3] += .914 + R.COUNTER_LIFT_M - float((g["points"] @ T[:3, :3].T + T[:3, 3])[:, 2].min())
    return T, ah, a, n


def lowest_rim_point(W, T, Pw):
    """The rim circle's lowest point of the plate on edge (x = the in-plane up), at the height of the plate's lowest point."""
    p = T[:3, 3] + W.geom["height"] * T[:3, 2] - W.geom["rim_r"] * T[:3, 0]
    return np.array([p[0], p[1], float(Pw[:, 2].min())])


def rails(W, T, ah, body):
    """c's stand: two rails on the counter at the plate's lowest point, long along the bottom tangent, inner faces RAIL_GAP_M
    apart; the mouth-side inner face 2 RAIL_CLEAR_M beyond the plate's farthest point toward its mouth within RAIL_H_MAX_M of
    the bottom; the height the largest whole mm up to RAIL_H_MAX_M that keeps both rails RAIL_CLEAR_M clear of the plate."""
    fcl = W.arm.fcl
    Pw = W.geom["points"] @ T[:3, :3].T + T[:3, 3]
    low = lowest_rim_point(W, T, Pw)
    near = Pw[Pw[:, 2] < low[2] + RAIL_H_MAX_M]
    s_m = float(((near - low) @ ah).max()) + 2 * RAIL_CLEAR_M   # 1 mm, so the 0.5 mm test is not decided by float noise
    t = np.cross([0., 0., 1.], ah)
    Rm = np.column_stack([ah, t, [0., 0., 1.]])
    found, d_ = None, None
    for h_mm in range(int(round(RAIL_H_MAX_M * 1e3)), 0, -1):
        h = h_mm / 1e3
        bs = []
        for name, s_c in (("RailMouth", s_m + RAIL_THICK_M / 2), ("RailBase", s_m - RAIL_GAP_M - RAIL_THICK_M / 2)):
            c = np.array([low[0], low[1], .914 + h / 2]) + s_c * ah
            bs.append(box_body(fcl, name, c, (RAIL_THICK_M, RAIL_LEN_M, h), Rm))
        d_ = [dist(fcl, body, b) for b in bs]
        if min(d_) >= RAIL_CLEAR_M:
            found = (h, bs)
            break
    if found is None:
        raise RuntimeError(f"no rail height keeps the stand clear of the plate (distances {d_})")
    h, bs = found
    info = {"gap_m": RAIL_GAP_M, "height_m": h, "held_depth_m": round(h - R.COUNTER_LIFT_M, 4),
            "clear_mm": [round(x * 1e3, 2) for x in d_], "boxes": {b["id"]: b["box"] for b in bs},
            "rule": "inner faces 12 mm apart, the mouth-side one 1 mm beyond the plate's farthest point toward its mouth near "
                    "the bottom; height = the largest whole mm (<= 20) with both rails >= 0.5 mm from the plate (FCL)"}
    return bs, info


def backstop(W, T, ah, body):
    """d's backstop: a box on the counter on the plate's base side, long along the bottom tangent, half as tall as the plate's
    top above the slab; its front face moved toward the plate until the plate's base face rests BACK_TOUCH_M from it."""
    fcl = W.arm.fcl
    Pw = W.geom["points"] @ T[:3, :3].T + T[:3, 3]
    low = lowest_rim_point(W, T, Pw)
    h = round(float(Pw[:, 2].max() - .914) / 2, 3)
    t = np.cross([0., 0., 1.], ah)
    Rm = np.column_stack([ah, t, [0., 0., 1.]])

    def at(u):                                       # the front face u along ah from the lowest point (u < 0: behind it)
        c = np.array([low[0], low[1], .914 + h / 2]) + (u - BACK_T_M / 2) * ah
        return box_body(fcl, "Backstop", c, (BACK_T_M, BACK_W_M, h), Rm)
    lo, hi = -.40, float(((Pw - low) @ ah).max())    # lo: far behind (clear); hi: through the plate
    if dist(fcl, body, at(lo)) <= BACK_TOUCH_M:
        raise RuntimeError("the backstop's far position already touches the plate")
    for _ in range(40):
        mid = (lo + hi) / 2
        lo, hi = (mid, hi) if dist(fcl, body, at(mid)) > BACK_TOUCH_M else (lo, mid)
    b = at(lo)
    info = {"height_m": h, "front_face_behind_lowest_point_m": round(-lo, 4), "gap_mm": round(dist(fcl, body, b) * 1e3, 2),
            "boxes": {"Backstop": b["box"]},
            "rule": "a 20 mm deep, 160 mm wide box half as tall as the plate's top above the slab, behind the plate, moved toward "
                    "it until the plate's base face is 1 mm from its top edge (FCL bisection)"}
    return [b], info


def kind_rim(W, T_p, k, roll):
    """a: grasp.bowl_grasps' rim pinch of yaw k (rolled), leaned by the least grasp.TILTS_DEG that keeps the gripper capsule and
    its transit line off the counter (tilted_grasps' rule); None when no tilt does."""
    from dishsim_frigidaire.robot import grasp as G
    T0 = G.bowl_grasps(T_p, N_YAW, "plate")[k]["T_tcp"] @ (RZ_PI if roll else np.eye(4))
    for deg in G.TILTS_DEG:
        T = G.tilt(T0, deg)
        if not G.hits_counter(T):
            return T, deg
    return None, None


def edge_pinch(W, T_p, n):
    """The edge pinch at the rim point along ``n`` (unit, in the plate's plane): TCP z = -n (radially inward), y = the plate
    axis (the pads on the mouth face and the base face), the TCP EDGE_OUT_M outside the rim edge at the height edge_z."""
    a = T_p[:3, 2]
    T = np.eye(4)
    T[:3, 2], T[:3, 1] = -n, a
    T[:3, 0] = np.cross(a, -n)
    T[:3, 3] = T_p[:3, 3] + W.geom["edge_z"] * a + (W.geom["rim_r"] + EDGE_OUT_M) * n
    return T


def presentation(W, pres, k):
    """(plate pose, [(roll, T_tcp, tilt_deg)], fixtures [bodies], fixture info, lift straight up?) of one presentation yaw."""
    from dishsim_frigidaire.robot import grasp as G
    rolls = []
    if pres in ("a", "b"):
        T_p = flat_pose(R.COUNTER_XY if pres == "a" else (R.COUNTER_XY[0], B_CENTER_Y))
        if pres == "a":
            for roll in ROLLS:
                T, deg = kind_rim(W, T_p, k, roll)
                rolls.append((roll, T, deg))
        else:
            n = G.bowl_grasps(T_p, N_YAW, "plate")[k]["T_tcp"][:3, 1]      # the radial direction of yaw k's rim point
            for roll in ROLLS:
                rolls.append((roll, edge_pinch(W, T_p, n) @ (RZ_PI if roll else np.eye(4)), None))
        return T_p, rolls, [], None, pres != "a"
    lean = 0. if pres.startswith("c") else float(pres[1:3])
    T_p, ah, a, n = edge_pose(W, k, lean)
    body = W.arm.checker._body("plate", W.P.pose_dict(T_p), "dish")
    fx, info = rails(W, T_p, ah, body) if pres.startswith("c") else backstop(W, T_p, ah, body)
    T0 = (edge_pinch(W, T_p, n) if pres.endswith("top") else EP.sampler_mirror_tcp(T_p, MIRROR_D_MM, G, np) if
          pres.endswith("I4m") else EP.plate_holds(T_p, G, np)["mouth"][1])
    for roll in ROLLS:
        rolls.append((roll, T0 @ (RZ_PI if roll else np.eye(4)), None))
    return T_p, rolls, fx, info, True


# ------------------------------------------------------------------------------------------------------------- the pick
def pairs_all(arm, qs, kw):
    """Blocking pairs counted once per IK solution."""
    c = Counter()
    for q in qs:
        c.update(R.pairs_of(arm, q, **kw))
    return c


def line(W, arm, T0, T1, q0, n, kw, allow_last=0., dish=None):
    """frigidaire_robot_reach.line_ok with the blocking pairs: (qs, why, pairs)."""
    qs = W.kr.line_q(T0, T1, n, q0)
    if qs is None:
        return None, "IK branch jump", ["IK branch jump"]
    d = float(np.linalg.norm(np.asarray(T1)[:3, 3] - np.asarray(T0)[:3, 3]))
    for i, q in enumerate(qs):
        left = d * (1 - i / max(1, len(qs) - 1))
        extra = {"gripper_may_touch": (dish,)} if (dish and left < allow_last) else {}
        if not arm.free(q, **kw, **extra):
            ps = R.pairs_of(arm, q, **kw, **extra)
            return None, f"collides {round(left * 1e3)} mm from the end: {ps[:3]}", ps
    return qs, None, []


def pick(W, arm, dish, T_p, T_g, lift_up):
    """G1 and G2 of one pick candidate (see the module doc). The plate body ``dish`` is in the scene while empty-handed."""
    from dishsim_frigidaire.robot import grasp as G
    q_home = W.q_home
    H = G.hover_along(T_g, R.HOVER_M)
    T_up = G.hover(T_g, R.LIFT_M) if lift_up else G.hover_along(T_g, R.LIFT_M)
    M = np.linalg.inv(T_g) @ T_p
    held = {"held": ("plate", M, "dish")}
    r = {"ok": False, "gate": "none", "stage": None, "why": None, "blockers": {}}

    def fail(stage, why, blockers):
        arm.set_dishes({})
        r.update(stage=stage, why=why, blockers=dict(blockers))
        return r
    arm.set_dishes({"dish": dish})
    ik_h = W.kr.ik_all(H, q_home)
    hov = sorted([q for q in ik_h if arm.free(q)], key=lambda q: float(np.abs(q - q_home).max()))
    if not hov:
        return fail("G1 hover", f"no collision-free IK at the pre-grasp hover ({len(ik_h)} IK)",
                    pairs_all(arm, ik_h, {}) if ik_h else {"no IK": 1})
    ik_g = W.kr.ik_all(T_g, q_home)
    if not any(arm.free(q, gripper_may_touch=("dish",)) for q in ik_g):
        return fail("G1 grasp", f"no collision-free IK at the grasp ({len(ik_g)} IK)",
                    pairs_all(arm, ik_g, {"gripper_may_touch": ("dish",)}) if ik_g else {"no IK": 1})
    arm.set_dishes({})
    ik_u = W.kr.ik_all(T_up, q_home)
    if not any(arm.free(q, **held) for q in ik_u):
        return fail("G1 lift", f"no collision-free IK at the 0.15 m lift with the plate held ({len(ik_u)} IK)",
                    pairs_all(arm, ik_u, held) if ik_u else {"no IK": 1})
    r["gate"] = "G1"
    tries = []
    for q_h in hov[:4]:
        arm.set_dishes({"dish": dish})
        qs_a, why, ps = line(W, arm, H, T_g, q_h, 100, {}, allow_last=EP.ALLOW_CONTACT_M, dish="dish")
        if qs_a is None:
            tries.append(("G2 approach line", f"approach line {why}", ps))
            continue
        arm.set_dishes({})
        qs_u, why, ps = line(W, arm, T_g, T_up, qs_a[-1], 80, held)
        if qs_u is None:
            tries.append(("G2 lift line", f"lift line {why}", ps))
            continue
        arm.set_dishes({"dish": dish})
        tr = R.transit_check(W, arm, q_home, G.tcp_frame(np.array(R.HOME_TCP), [1., 0., 0.]), q_h, H, {})
        arm.set_dishes({})
        if not tr["ok"]:
            tries.append(("G2 home transit", f"home -> hover: {tr['why']}", [f"transit: {tr['why']}"]))
            continue
        to_base = np.asarray(W.T_wb[:2, 3]) - T_p[:2, 3]
        off = (H - T_g)[:2, 3]                       # where the gripper comes from; a vertical approach: the grasp point's
        if np.linalg.norm(off) < .01:                # side of the plate (the episode's order)
            off = T_g[:2, 3] - T_p[:2, 3]
        facing = (float(off @ to_base / (np.linalg.norm(off) * np.linalg.norm(to_base))) if np.linalg.norm(off) > 1e-3 else 0.)
        d_line, close = float(np.linalg.norm(T_g[:3, 3] - H[:3, 3])), (np.inf, None)
        for i, q in enumerate(qs_a):                 # the open gripper + wrist 3 vs the plate where the gate lets them touch
            left = d_line * (1 - i / max(1, len(qs_a) - 1))
            if left < EP.ALLOW_CONTACT_M:
                objs = [o for o in arm.arm_objects(q, ("shoulder_link",)) if o[0] in ("gripper", "wrist_3_link")]
                dd = EP.min_clear(arm.fcl, objs, [("dish", dish)], np, cutoff=.05)[0]
                if dd < close[0]:
                    close = (dd, left)
        r["approach_last60"] = {"gripper_plate_min_mm": round(close[0] * 1e3, 1) if np.isfinite(close[0]) else None,
                                "at_mm_from_end": None if close[1] is None else round(close[1] * 1e3)}
        r["turn_to_goal_deg"] = {nm: rot_deg(T_p[:3, :3], (EP.sampler_bowl_tcp(g[0], M, 0., np) @ M)[:3, :3])
                                 for nm, g in W.goals.items()}
        r.update(ok=True, gate="G2", stage=None, why=None, transit=tr, facing=round(facing, 3),
                 move_rad=round(float(np.abs(q_h - q_home).max()), 3), q_hover=np.round(q_h, 5).tolist(),
                 q_grasp=np.round(qs_a[-1], 5).tolist(), q_lift=np.round(qs_u[-1], 5).tolist(),
                 T_lift=np.round(T_up, 6).tolist(), M=np.round(M, 6).tolist())
        return r
    st, why, ps = tries[0]
    arm.set_dishes({})
    r.update(stage=st, why=why, blockers=dict(Counter(ps)), tries=[t[1][:200] for t in tries])
    return r


def moment(W, T_tcp, M):
    """The plate's weight about the pinch at the lift (N m, the visual centroid as its centre of mass): the part about the
    closing axis (a free swing for a two-pad pinch) and the rest (carried by the pads' pressure)."""
    T_plate = T_tcp @ M
    com = T_plate[:3, :3] @ W.geom["centroid"] + T_plate[:3, 3]
    tau = np.cross(com - T_tcp[:3, 3], np.array([0., 0., -9.81 * PLATE_MASS_KG]))
    y = T_tcp[:3, 1]
    return round(float(abs(tau @ y)), 4), round(float(np.linalg.norm(tau - (tau @ y) * y)), 4)


def sweep_job(args):
    """One (presentation, yaw): both rolls -> a row."""
    pres, k = args
    W = world()
    arm = W.arm
    t0 = time.monotonic()
    T_p, rolls, fx, info, lift_up = presentation(W, pres, k)
    for b in fx:
        arm.statics[b["id"]] = b
    try:
        dish = arm.checker._body("plate", W.P.pose_dict(T_p), "dish")
        row = {"presentation": pres, "yaw_k": k, "plate_pose": np.round(T_p, 6).tolist(), "fixture": info, "rolls": {}}
        for roll, T_g, deg in rolls:
            if T_g is None:
                rr = {"ok": False, "gate": "none", "stage": "candidate", "blockers": {"gripper capsule|Counter": 1},
                      "why": "no grasp.TILTS_DEG lean keeps the gripper capsule and its transit line off the counter"}
            else:
                rr = pick(W, arm, dish, T_p, T_g, lift_up)
                rr["tilt_deg"] = deg
                rr["T_tcp"] = np.round(T_g, 6).tolist()
                rr["moment_Nm"] = dict(zip(("closing_axis", "pads"), moment(W, T_g, np.linalg.inv(T_g) @ T_p)))
            row["rolls"][str(roll)] = rr
        row["pass"] = any(v["ok"] for v in row["rolls"].values())
        row["gate"] = max((v["gate"] for v in row["rolls"].values()), key=("none", "G1", "G2").index)
    finally:
        for b in fx:
            arm.statics.pop(b["id"], None)
        arm.set_dishes({})
    row["seconds"] = round(time.monotonic() - t0, 1)
    return row


# ------------------------------------------------------------------------------------------------------------ the I4 check
def rot_deg(Ra, Rb):
    return round(math.degrees(math.acos(float(np.clip((np.trace(Ra.T @ Rb) - 1) / 2, -1., 1.)))), 1)


def descent_ok(W, arm, T_s, q, h_s, v, tool, targets, seat_rule=True):
    """The final descent from the start ``h_s`` above the seat: IK continuous in 1 mm steps (at most 0.1 rad a step) down to
    DESCENT_BELOW_M under the seat, and D-seat's rule (seat_line_hits: SEAT_LINKS and the closed jaws + wrist 3 at least
    SEAT_ARM_MARGIN_M from the scene) every 5 mm from the start to the seat. (ok, why)"""
    prev, n_dn = q, int(round(h_s * 1e3))
    for j in range(1, n_dn + int(round(EP.DESCENT_BELOW_M * 1e3)) + 1):
        T_j = T_s.copy()
        T_j[:3, 3] -= j * 1e-3 * v
        qq = W.kr.ik(T_j, prev)
        if qq is None or float(np.abs(qq - prev).max()) > .1:
            return False, f"IK branch jump {n_dn - j:+d} mm from the seat"
        if seat_rule and j <= n_dn and (j % 5 == 0 or j == n_dn):
            hits = EP.seat_line_hits(arm, qq, tool, targets, np)
            if hits:
                return False, f"final descent {n_dn - j} mm above the seat: {hits[0]}"
        prev = qq
    return True, None


def i4_check(W, arm, T_goal, F_r, M, q_lift, T_lift, seat_rule=True):
    """The I4 descent check with the hold M (TCP -> plate), from the pick's lift (q_lift, T_lift); hardware only."""
    from dishsim_frigidaire.robot import collide as C
    arm.set_dishes({})
    held = {"held": ("plate", M, "dish"), "ignore": ("dish",)}
    T_g = EP.sampler_bowl_tcp(T_goal, M, 0., np) @ M                   # the plate at its goal, the pinch at the top edge
    x_dir, v, lean_t = EP.tine_axis(F_r, np)
    T_c = T_g.copy()
    T_c[:3, 3] += EP.SEAT_DX_M * x_dir
    labels = EP.rack_labels(len(arm.checker.parts["LowerRack"]))
    sk = EP.seat_search(arm, "plate", "dish", T_c, v, EP.scene_targets(arm, ignore=("dish",), statics=False), labels, np)
    out = {"pass": False, "gate": "none", "seat_mm": sk.get("seat_mm"), "supports": sk.get("supports")}
    if sk.get("seat_mm") is None:
        out["why"] = f"no seat along the tine axis ({ {k: v_ for k, v_ in sk.items() if k != 'T_seat'} })"
        return out
    T_ts = sk.pop("T_seat") @ np.linalg.inv(M)
    if not W.kr.ik_all(T_ts, q_lift):
        out["why"] = "G0: no IK at the seat"
        return out
    out["gate"] = "G0"
    tool = EP.tool_hulls(arm, W.plate_closed)
    targets = EP.scene_targets(arm, ignore=("dish",))
    starts, hits, dhits, h_s, examples = [], Counter(), Counter(), EP.SEAT_START_ABOVE_M, []
    while h_s <= EP.SEAT_START_MAX_M + 1e-9 and not starts:
        T_s = T_ts.copy()
        T_s[:3, 3] += h_s * v
        for q in W.kr.ik_all(T_s, q_lift):
            if not arm.free(q, **held):
                hits.update(R.pairs_of(arm, q, **held))
                continue
            ok, why = descent_ok(W, arm, T_s, q, h_s, v, tool, targets, seat_rule)
            if not ok:
                if why not in examples and len(examples) < 4:
                    examples.append(why)
                dhits[why.split(": ")[-1].rsplit(" ", 2)[0] if ": " in why else why] += 1
                continue
            starts.append((h_s, T_s, q))
        if not starts:
            h_s = round(h_s + EP.SEAT_START_STEP_M, 3)
    out.update(start_blockers=dict(hits.most_common(4)), descent_blockers=dict(dhits.most_common(4)), descent_examples=examples)
    if not starts:
        out["why"] = (f"G1: no descent start {EP.SEAT_START_ABOVE_M * 1e3:.0f}..{EP.SEAT_START_MAX_M * 1e3:.0f} mm above the "
                      f"seat with a free IK at the 15 mm carry margin and a final descent under the 2 mm rule; start hits "
                      f"{hits.most_common(3)}, descent {dhits.most_common(3)}")
        return out
    out.update(gate="G1", start_mm=round(starts[0][0] * 1e3), n_starts=len(starts))
    tries = []
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
                why_up = f"collides {h_full * j / n_up * 1e3:.0f} mm up the axis: {R.pairs_of(arm, q, **held)[:2]}"
                break
            prev, q_top, h_top = q, q, h_full * j / n_up
        if q_top is None or h_top < EP.SEAT_LINE_MIN_M - 1e-6:
            tries.append(f"line up the tine axis {h_top * 1e3:.0f} mm free (< {EP.SEAT_LINE_MIN_M * 1e3:.0f}): {why_up}")
            continue
        T_top = T_s.copy()
        T_top[:3, 3] += h_top * v
        path, sec = R.leg_ok(W, arm, q_lift, q_top, held, CARRY_RRT_S)
        if path is None:
            tries.append(f"no leg 1 from the lift to the top of the tine-axis line (RRT-Connect {CARRY_RRT_S:.0f} s)")
            continue
        n_dn = max(3, int(round(float((T_top[:3, 3] - T_s[:3, 3]) @ v) / .005)) + 1)
        dn, why, ps = line(W, arm, T_top, T_s, q_top, n_dn, held)
        if dn is None:
            tries.append(f"the line down the tine axis: {why}")
            continue
        out.update(pass_=True, gate="G2", why=None, line_up_mm=round(h_top * 1e3), leg1={"path": path, "s": sec},
                   start_h_mm=round(h_s * 1e3), plate_rotation_lift_to_start_deg=rot_deg((T_lift @ M)[:3, :3], (T_s @ M)[:3, :3]),
                   tcp_rotation_lift_to_start_deg=rot_deg(T_lift[:3, :3], T_s[:3, :3]),
                   q_start=np.round(q_s, 5).tolist(), q_top=np.round(q_top, 5).tolist())
        out["pass"] = out.pop("pass_")
        break
    out["carry_tries"] = tries[:6]
    if not out["pass"]:
        out["why"] = f"G2: carry: {tries[0] if tries else 'no start'}"
    return out


def i4_job(args):
    """One goal plate: the I4 check from each pick candidate in turn (the fixture of that candidate's yaw in the scene)."""
    name, pres, cands, seat_rule = args
    W = world()
    arm = W.arm
    t0 = time.monotonic()
    T_goal, slot, F_r, shift_mm = W.goals[name]
    res = {"dish": name, "slot": slot, "presentation": pres, "seat_rule": seat_rule, "rack_frame_shift_mm": shift_mm, "tries": []}
    for c in cands:
        _, _, fx, _, _ = presentation(W, pres if pres in PRESENTATIONS else "a", c["yaw_k"])
        for b in fx:
            arm.statics[b["id"]] = b
        try:
            r = i4_check(W, arm, T_goal, F_r, np.asarray(c["M"]), np.asarray(c["q_lift"]), np.asarray(c["T_lift"]), seat_rule)
        finally:
            for b in fx:
                arm.statics.pop(b["id"], None)
        r.update(yaw_k=c["yaw_k"], roll=c["roll"])
        res["tries"].append(r)
        if r["pass"]:
            break
    best = next((t for t in res["tries"] if t["pass"]), None)
    if best is None:
        best = max(res["tries"], key=lambda t: ("none", "G0", "G1", "G2").index(t["gate"])) if res["tries"] else {"gate": "none"}
    res.update(pass_=bool(best.get("pass")), gate=best.get("gate"), why=best.get("why"), seconds=round(time.monotonic() - t0, 1))
    res["pass"] = res.pop("pass_")
    return res


# -------------------------------------------------------------------------------------------------------------- report
def summarize(rows):
    """{presentation: summary} from the sweep rows."""
    out = {}
    for pres in PRESENTATIONS:
        rs = sorted([r for r in rows if r["presentation"] == pres], key=lambda r: r["yaw_k"])
        bl = Counter()
        for r in rs:
            for rr in r["rolls"].values():
                if not rr["ok"]:
                    bl.update(rr.get("blockers") or {})
        stages = Counter(rr["stage"] for r in rs for rr in r["rolls"].values() if not rr["ok"])
        mom = [rr["moment_Nm"] for r in rs for rr in r["rolls"].values() if rr.get("moment_Nm")]
        oks = [rr for r in rs for rr in r["rolls"].values() if rr["ok"]]
        close = [rr["approach_last60"]["gripper_plate_min_mm"] for rr in oks
                 if rr["approach_last60"]["gripper_plate_min_mm"] is not None]
        out[pres] = {"pass": sum(r["pass"] for r in rs), "of": len(rs), "yaws": [r["yaw_k"] for r in rs if r["pass"]],
                     "pass_roll0": sum(r["rolls"]["0"]["ok"] for r in rs), "pass_roll180": sum(r["rolls"]["180"]["ok"] for r in rs),
                     "g1": sum(r["gate"] in ("G1", "G2") for r in rs), "fail_stages": dict(stages.most_common()),
                     "blockers": dict(bl.most_common(6)),
                     "moment_Nm": {"closing_axis": max((m["closing_axis"] for m in mom), default=None),
                                   "pads": max((m["pads"] for m in mom), default=None)},
                     "approach_last60_gripper_plate_min_mm": min(close) if close else None,
                     "turn_to_goal_deg_min": {nm: min((rr["turn_to_goal_deg"][nm] for rr in oks), default=None) for nm in I4_PLATES},
                     "fixture": rs[0]["fixture"] if rs else None}
    a_pads = out["a"]["moment_Nm"]["pads"]
    for pres, s in out.items():                      # the pivot column from the pads' moment relative to a's (38 deg in Isaac)
        f = (s["moment_Nm"]["pads"] or 0.) / a_pads if a_pads else None
        s["pivot_on_lift"] = ("yes (38 deg measured, limits_v1/gate2_plate)" if pres == "a" else
                              None if f is None else f"likely ({f:.0%} of a's moment)" if f >= .75 else
                              f"possible ({f:.0%} of a's moment)" if f >= .25 else f"no ({f:.0%} of a's moment)")
    return out


def pick_best(summ):
    """Most passing yaws; ties: the I4 placing hold itself (c-I4, d-I4), then the pivot-free (c, d), then G1 yaws."""
    def key(p):
        s = summ[p]
        return (s["pass"], p in I4_HOLD_ROWS, p[0] in "cd", s["g1"], -PRESENTATIONS.index(p))
    return max(BRIEF_ROWS, key=key)


HOLDS = {"rim pinch": ("a", "c-I4", "d30-I4", "d45-I4"), "edge pinch": ("b", "c-top", "d30-top", "d45-top"),
         "mirrored hold": EXTRA_ROWS}


def i4_fail(r):
    """The commonest blockers over every try of a failed I4 result and its first descent reason, short."""
    blk, ex = Counter(), []
    for t in r["tries"]:
        blk.update(t.get("start_blockers") or {})
        blk.update(t.get("descent_blockers") or {})
        ex += [e for e in t.get("descent_examples") or [] if e not in ex]
    top = ", ".join(f"{k} ({v} IK)" for k, v in blk.most_common(2)).replace("|", " vs ")
    ex = [re.sub(r" -\d+\.\d mm", " intersecting", e).replace("|", " vs ") for e in ex]
    return f"{r['gate']}: {top or (r.get('why') or '')[:80]}" + (f"; first descent fail: {ex[0]}" if ex else "")


def i4_cell(r):
    if r["pass"]:
        t = next(x for x in r["tries"] if x["pass"])
        return f"G2, start {t['start_h_mm']} mm, leg 1 {t['leg1']['path']}, turn {t['plate_rotation_lift_to_start_deg']} deg"
    return i4_fail(r)


def winner_note(res, p):
    """Whether the carry needs a change of the plate in the hand, the wrist turn and the pivot, for presentation p."""
    s, ia = res["summary"], res["i4_all"]
    fam = next(k for k, v in HOLDS.items() if p in v)
    turns = [t["plate_rotation_lift_to_start_deg"] for x in ia.get(p, {}).values() for t in x["tries"] if t.get("pass")]
    return (f"{p} holds the plate by the {fam}" + (", which is the I4 placing hold itself (plate_holds' mouth hold), so the "
            "carry to the descent start needs no change of the plate in the hand." if p in I4_HOLD_ROWS else
            ", which differs from plate_holds' I4 hold. The I4 check ran with the plate held as picked, so the carry needs "
            "no regrasp either.") +
            (f" The wrist turns the rigidly held plate by {' and '.join(str(t) for t in turns)} deg between the lift and the "
             "descent start." if turns else " No descent start passed, so the wrist turn has no value.") +
            f" Pivot on the lift: {s[p]['pivot_on_lift']}, {s[p]['moment_Nm']['pads']} N m on the pads against a's "
            f"{s['a']['moment_Nm']['pads']} N m.")


def write_md(res, path, cmd):
    s, i4, b, ia = res["summary"], res["i4"], res["best"], res["i4_all"]
    L = ["# Plate presentations at the counter spot, Kit-free (phase 1 of plans/2026-11-04-plate-pick-presentation.md)", "",
         f"Command (Mac, {res['seconds']:.0f} s on {res['workers']} workers):", "", "```", cmd, "```", "",
         "The scene is the FCL arm model on the posture_v1 mount, the lower rack out and the upper rack pushed in, the counter "
         "(front edge -0.17 m), the pedestal and the floor, plus the plate's fixture as static boxes and no other dish. The "
         "arm and the gripper keep the full 15 mm margin. Only the gripper may touch the plate, in the last 60 mm of the "
         "approach and at the grasp. The held plate keeps the checker's 1 mm overlap rule against the scene and 2 mm against "
         "the arm links. A yaw passes when either roll of the gripper about its approach passes G1 (a free IK at the hover, "
         "the grasp and the 0.15 m lift with the plate held) and G2 (the approach line, the lift line and the home transit).", "",
         "Pivot on lift compares the plate's weight moment on the pads at the lift with a's, which pivoted 38 deg in Isaac. "
         "The moment about the closing axis is 0 for every hold. The gripper column gives the open gripper's closest approach "
         "to the plate in the last 60 mm of the approach over the passing candidates, where 0 would mean touching. The turn "
         "column gives the smallest rotation of the plate from the pick to the goals of plate_05 and plate_03 over the passing "
         "candidates. Rows marked extra use round 12 window 2's mirrored hold (sampler_mirror_tcp, rim 6 mm into the pads, "
         "the hand on the base face), which seated the front four in Isaac, and they stay out of the choice of the best.", "",
         "| presentation | yaws passing of 12 (roll 0 / roll 180) | passing yaws | commonest blockers | pivot on lift "
         "(pads N m) | gripper to plate, last 60 mm (mm) | turn to goals 05 / 03 (deg) |",
         "|---|---|---|---|---|---|---|"]
    for p in PRESENTATIONS:
        x = s[p]
        bl = "; ".join(f"{k} ({v})" for k, v in list(x["blockers"].items())[:3]).replace("|", " vs ") or "none"
        tg = x["turn_to_goal_deg_min"]
        L.append(f"| {p}{' (extra)' if p in EXTRA_ROWS else ''} | {x['pass']} ({x['pass_roll0']} / {x['pass_roll180']}) | "
                 f"{', '.join(map(str, x['yaws'])) or '-'} | {bl} | {x['pivot_on_lift']} ({x['moment_Nm']['pads']}) | "
                 f"{x['approach_last60_gripper_plate_min_mm'] if x['approach_last60_gripper_plate_min_mm'] is not None else '-'} | "
                 f"{tg['plate_05'] if tg['plate_05'] is not None else '-'} / {tg['plate_03'] if tg['plate_03'] is not None else '-'} |")
    hc = {r["fixture"]["height_m"] for r in res["rows"] if r["presentation"] == "c-top"}
    hd = {r["fixture"]["held_depth_m"] for r in res["rows"] if r["presentation"] == "c-top"}
    fx_c, fx_d, fx_e = s["c-top"]["fixture"], s["d30-top"]["fixture"], s["d45-top"]["fixture"]
    L += ["", "The rows use three holds. a, c-I4, d30-I4 and d45-I4 hold the plate by the KIND_RIM rim pinch (closing "
          "radially across the rim wall, the approach along the plate's -axis). b, c-top, d30-top and d45-top use the edge "
          "pinch (the approach radially inward, the pads on the mouth face and the base face, closing along the plate axis). "
          "The extra rows use the mirrored hold. The I4 check depends on the hold and the carry's start only, so the rows of "
          "one hold share their seat and descent verdicts.", "",
          f"By the brief's rule over its eight rows the best presentation is {b['presentation']}, with {b['why']}.", "",
          f"a passes yaw {', '.join(map(str, s['a']['yaws'])) or 'none'} at full clearance, the same yaw the s0 gate passed "
          "with the waiver, because the spot's front rim hangs 64 mm past the -0.17 m edge and the gripper there is clear of "
          "the counter. In Isaac that pick stopped at waypoint 75 of 100 with the palm on the rim. The model keeps the open "
          f"gripper {s['a']['approach_last60_gripper_plate_min_mm']} mm from the plate over the last 60 mm, so this gate "
          "cannot see that contact.", "",
          f"The stand's rails, inner faces 12 mm apart, can be at most {'/'.join(f'{h * 1e3:.0f}' for h in sorted(hc))} mm "
          f"tall and hold the upright plate {'/'.join(f'{h * 1e3:.0f}' for h in sorted(hd))} mm deep (the same at every yaw "
          f"when one value is given). The coupe rim spreads along the axis as it rises from its lowest point, so a taller rail "
          f"cuts into it. The backstop is {fx_d['height_m'] * 1e3:.0f} mm tall for d30 and "
          f"{fx_e['height_m'] * 1e3:.0f} mm for d45 (half the plate's top height), and the plate's base face rests 1 mm from "
          "its top edge.", "",
          "The I4 descent check runs hardware only, from the pick's 0.15 m lift. The goal is spun so the pinch is the rim "
          "point highest at the goal, and the plate alone is lowered along the tine axis to its seat. A descent start lies 50 "
          "to 150 mm above the seat with a free IK at the 15 mm carry margin. The line from the start to the seat follows "
          "D-seat's 2 mm rule (SEAT_LINKS and the closed jaws plus wrist 3), and the IK stays continuous 30 mm below the seat. "
          "The line up the tine axis must be free for at least 100 mm. Leg 1 from the lift (straight, else RRT-Connect 45 s) "
          "and the line down the axis complete the carry. Up to two passing picks per roll serve as the carry's start, the best "
          "facing first.", "",
          f"I4 with the best presentation's hold ({b['presentation']}):", "",
          "| plate | slot | pass | gate | seat (mm along the axis) | start (mm above the seat) | leg 1 | plate turn, lift to "
          "start (deg) | picks tried (yaw, roll) | why |",
          "|---|---|---|---|---|---|---|---|---|---|"]
    for name in I4_PLATES:
        r = i4[name]
        t = next((x for x in r["tries"] if x["pass"]), r["tries"][-1] if r["tries"] else {})
        tried = " ".join(f"({x['yaw_k']}, {x['roll']})" for x in r["tries"]) or "-"
        L.append(f"| {name} | {r['slot']} | {'yes' if r['pass'] else 'no'} | {r['gate']} | {t.get('seat_mm')} | "
                 f"{t.get('start_h_mm', '-')} | {(t.get('leg1') or {}).get('path', '-')} | "
                 f"{t.get('plate_rotation_lift_to_start_deg', '-')} | {tried} | "
                 f"{'all stages clear' if r['pass'] else i4_fail(r)} |")
    L += ["", "I4 with every presentation that has a passing pick (the same check):", "",
          "| presentation | plate_05 (lower_front_04) | plate_03 (lower_front_00) |", "|---|---|---|"]
    for p in [p for p in PRESENTATIONS if p in ia]:
        L.append(f"| {p}{' (extra)' if p in EXTRA_ROWS else ''} | {i4_cell(ia[p]['plate_05'])} | {i4_cell(ia[p]['plate_03'])} |")
    c1, c0 = ia.get("s0 hold"), ia.get("s0 hold (continuity only)")
    if c1 and c0:
        L += ["", "Calibration against the s0 gate (gate.json) uses its measured hold (limits_v1/gate2_plate, pivot 38 deg) "
              "from its pick at a's yaw 6. With the descent checked for IK continuity only, as plate_release does, plate_05 "
              f"gives {i4_cell(c0['plate_05'])} and plate_03 gives {i4_cell(c0['plate_03'])}, the gate's own 80 and 50 mm "
              f"starts. With D-seat's 2 mm rule added, plate_05 gives {i4_cell(c1['plate_05'])} and plate_03 gives "
              f"{i4_cell(c1['plate_03'])}."]
    bb = res.get("best_both")
    L += ["", winner_note(res, b["presentation"]) + " " +
          (f"The best presentation with a pick and an I4 pass at both goals is {bb}, with {s[bb]['pass']} of 12 yaws"
           + (", an extra row outside the brief's eight" if bb in EXTRA_ROWS else "") + ". " + winner_note(res, bb)
           if bb and bb != b["presentation"] else "No presentation has both a pick and an I4 pass at both goals." if not bb else ""), ""]
    path.write_text("\n".join(L) + "\n")


def candidates(rows, pres):
    """The passing picks of a presentation as I4 carry starts: per roll the I4_TRIES_PER_ROLL best facing (then the smallest
    joint move from home), the rolls interleaved."""
    per = []
    for roll in ("0", "180"):
        ok = [dict(r["rolls"][roll], yaw_k=r["yaw_k"], roll=int(roll)) for r in rows
              if r["presentation"] == pres and r["rolls"][roll]["ok"]]
        ok.sort(key=lambda c: (-c["facing"], c["move_rad"]))
        per.append(ok[:I4_TRIES_PER_ROLL])
    return [c for i in range(I4_TRIES_PER_ROLL) for x in per if i < len(x) for c in [x[i]]]


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--workers", type=int, default=6)
    a = ap.parse_args()
    import multiprocessing as mp
    t0 = time.monotonic()
    OUT.mkdir(parents=True, exist_ok=True)
    jobs = [(p, k) for p in PRESENTATIONS for k in range(N_YAW)]
    print(f"[INFO] {len(PRESENTATIONS)} presentations x {N_YAW} yaws x {len(ROLLS)} rolls on {a.workers} workers", flush=True)
    rows = []
    with mp.get_context("fork").Pool(a.workers) as pool:
        for r in pool.imap_unordered(sweep_job, jobs, chunksize=1):
            rows.append(r)
            rr = r["rolls"]
            print(f"[INFO] {r['presentation']:8s} yaw {r['yaw_k']:2d}: pass {r['pass']} gate {r['gate']} "
                  f"r0 {rr['0']['stage'] or 'ok'} r180 {rr['180']['stage'] or 'ok'} {r['seconds']} s", flush=True)
        summ = summarize(rows)
        best = pick_best(summ)
        tie = [p for p in BRIEF_ROWS if summ[p]["pass"] == summ[best]["pass"] and p != best]
        why = (f"{summ[best]['pass']} of 12 yaws passing at full clearance" +
               (f" (tied with {', '.join(tie)}; the tie goes to the I4 placing hold itself, then to the pivot-free c and d)"
                if tie else ""))
        orders = {p: candidates(rows, p) for p in PRESENTATIONS if summ[p]["pass"]}
        print(f"[INFO] best {best}: {why}; I4 carry starts {[(c['yaw_k'], c['roll']) for c in orders.get(best, [])]}", flush=True)
        gate = json.loads((LOAD.parent / "gate.json").read_text())         # the calibration: the s0 gate's hold and pick
        M_p, src_p = R.measured_M("plate")
        calib = {n: [{"yaw_k": gate["dishes"][n]["pick"]["best_yaw_k"], "roll": 0, "M": M_p.tolist(),
                      "q_lift": gate["dishes"][n]["pick"]["q_lift"], "T_lift": gate["dishes"][n]["pick"]["T_lift"]}] for n in I4_PLATES}
        i4_jobs = ([(n, p, orders[p], True) for p in orders for n in I4_PLATES] +
                   [(n, "s0 hold", calib[n], rule) for n in I4_PLATES for rule in (True, False)])
        i4_all = {}
        for r in pool.imap_unordered(i4_job, i4_jobs, chunksize=1):
            key = r["presentation"] + ("" if r["seat_rule"] else " (continuity only)")
            i4_all.setdefault(key, {})[r["dish"]] = r
            print(f"[INFO] I4 {key} {r['dish']} {r['slot']}: pass {r['pass']} gate {r['gate']} {r['seconds']} s why "
                  f"{str(r['why'])[:160]}", flush=True)
    W = world()
    i4 = i4_all.get(best) or {n: {"dish": n, "slot": W.goals[n][1], "pass": False, "gate": "none", "tries": [],
                                  "why": f"no pick of {best} passes, so the carry has no start"} for n in I4_PLATES}
    both = [p for p in PRESENTATIONS if p in i4_all and all(i4_all[p][n]["pass"] for n in I4_PLATES)]
    best_both = max(both, key=lambda p: (summ[p]["pass"], p in BRIEF_ROWS, p in I4_HOLD_ROWS)) if both else None
    res = {"plan": "plans/2026-11-04-plate-pick-presentation.md", "phase": 1, "load": str(LOAD.relative_to(ROOT)),
           "mount_file": str(R.SAMPLE_MOUNT.relative_to(ROOT)), "mount": W.mount, "arm_model": str(R.ARM_MODEL.relative_to(ROOT)),
           "counter_xy": list(R.COUNTER_XY), "counter_front_y": R.COUNTER_FRONT_Y,
           "plate": {k: round(float(W.geom[k]), 5) for k in ("rim_r", "height", "edge_z")},
           "rule": "full clearance: the arm and gripper 15 mm from the scene and the plate (the gripper may touch the plate in the "
                   "last 60 mm of the approach and at the grasp); the held plate 1 mm overlap vs the scene, 2 mm vs the arm links; "
                   "G1 = free IK at the hover (0.10 m back along the approach), the grasp and the 0.15 m lift with the plate "
                   "held as grasped; G2 = the approach line (100), the lift line (80) and the home -> hover transit; a yaw "
                   "passes when either roll (the gripper turned 180 deg about its approach) passes",
           "brief_rows": list(BRIEF_ROWS), "extra_rows": list(EXTRA_ROWS),
           "summary": summ, "best": {"presentation": best, "why": why}, "best_both": best_both,
           "i4": i4, "i4_all": i4_all, "i4_candidates": {p: [{k: c[k] for k in ("yaw_k", "roll", "facing", "move_rad")} for c in o]
                                                         for p, o in orders.items()},
           "calibration_hold": f"limits_v1/gate2_plate median hold (trial {src_p['trial']}, pivot {src_p['pivot_deg']} deg)",
           "rows": sorted(rows, key=lambda r: (PRESENTATIONS.index(r["presentation"]), r["yaw_k"])),
           "workers": a.workers, "seconds": round(time.monotonic() - t0, 1)}
    (OUT / "presentations.json").write_text(json.dumps(res, indent=1, default=lambda o: o.tolist() if hasattr(o, "tolist")
                                                       else str(o)) + "\n")
    cmd = ("PYTHONPATH=code/src .venv-kitfree/bin/python code/execution/frigidaire/frigidaire_plate_presentations.py "
           f"--workers {a.workers}")
    write_md(res, OUT / "kitfree.md", cmd)
    print(f"[RESULT] PASS plate presentations: " + ", ".join(f"{p} {summ[p]['pass']}/12" for p in PRESENTATIONS) +
          f"; best {best}; I4 " + ", ".join(f"{n} {'pass' if i4[n]['pass'] else 'fail ' + str(i4[n]['gate'])}" for n in I4_PLATES) +
          f"; pick + I4 at both goals: {both or 'none'} in {res['seconds']} s -> {(OUT / 'presentations.json').relative_to(ROOT)}",
          flush=True)


if __name__ == "__main__":
    main()
