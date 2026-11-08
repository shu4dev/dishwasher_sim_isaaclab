#!/usr/bin/env python3
"""Kit-free sweep of the plate's wall-normal pinch at the counter spot (plans/2026-11-04-plate-wall-pinch.md, phase 1).

    PYTHONPATH=code:code/util .venv-kitfree/bin/python code/sampler/robot/scripts/frigidaire_plate_wall_pinch.py --workers 6

The hold: hotec_gen's outer wall arc of the plate (foot ring edge to the rim edge, with the side bulge; the generator in
data/assets/hotec_wheatstraw/source/, the values from parameters.json), walked DEPTH_MM down from the rim edge (the arc's
top end, r = rim_diameter / 2, where the lip's round-over starts) along the arc. There the outward normal of the outer
surface is the closing axis (TCP y), the arc's tangent toward the foot is the approach (TCP z: the fingers slide down the
wall from outside the rim edge), TCP x = y x z (the rim's circumferential direction), and the TCP sits so the closed pads'
centre (the plate gate's closed jaws, EP.PLATE_GATE, finger-pad hulls in the TCP frame) is at that point, the wall's
mid-surface between the pads. Both rolls: the 180 deg turn about the approach (RZ_PI, the fingers swapped).
Presentations, 12 yaws each, as phase 1 of plans/2026-11-04-plate-pick-presentation.md did:
  face-up  the plate flat at the counter spot frigidaire_robot_reach.COUNTER_XY, no fixture; yaw k = the rim point of
           grasp.bowl_grasps' yaw k (k 0 = +y, k 6 = -y, the counter's front);
  lean30   leaning 30 deg back onto the phase-1 backstop (frigidaire_plate_presentations.edge_pose / backstop: 100 mm tall,
           20 x 160 mm, the plate's base face 1 mm from its top edge), the mouth facing azimuth 2 pi k / 12, the pinch at the
           top rim point.
Rows at 20 mm down the wall are the plan's hold; rows at 15 and 25 mm are extra and stay out of the choice.
Per (row, yaw, roll) the phase-1 pick gate (frigidaire_plate_presentations.pick: full clearance, the arm and the gripper at
the planner's 15 mm margin, the gripper may touch the plate in the last 60 mm of the approach and at the grasp, the held
plate 1 mm overlap vs the scene and 2 mm vs the links; G1 = free IK at the hover, the grasp and the 0.15 m lift straight up
with the plate held; G2 = the approach line, the lift line and the home transit), the pivot moment at the lift
(frigidaire_plate_presentations.moment, against phase 1's a), the open gripper's closest approach to the plate over the last
60 mm. Then the I4 descent check (a copy of frigidaire_plate_presentations.i4_check with a spin argument: the seat along the
tine axis, the start's IK at the 15 mm carry margin, D-seat's 2 mm rule on the final descent with the plate gate's closed
jaws, the line up the tine axis, leg 1 from the pick's lift, the line down) at plate_05, plate_03 and plate_04, plate_04 also
with the pinch spun 30 deg toward the front (EP.sampler_bowl_tcp spin 30, as frigidaire_site_sweep.py's elbow-up pass).
Chosen: the most passing yaws among the 20 mm rows whose I4 passes at least one goal (ties: more goals, then no fixture).
Writes data/results/robot/samples_v1/<s0>__fixes/plates_wall/{wall_pinch.json,kitfree.md}. Imports the reach, episode and
phase-1 scripts as libraries only.
"""
from __future__ import annotations

import argparse
from collections import Counter
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import sys
import time

import numpy as np

ROOT = Path(__file__).resolve().parents[4]
sys.path[:0] = [str(ROOT / "code/sampler/robot/scripts"), str(ROOT / "code"), str(ROOT / "code/util")]
import frigidaire_plate_presentations as PP         # noqa: E402  phase 1: the scene, fixtures, pick gate, moment, I4 pieces
R, EP = PP.R, PP.EP

SAMPLE = PP.SAMPLE
LOAD = PP.LOAD
OUT = ROOT / "data/results/robot/samples_v1" / f"{SAMPLE}__fixes" / "plates_wall"
PHASE1 = PP.OUT / "presentations.json"              # phase 1's table: a's pads moment for the pivot column
HOTEC = ROOT / "data/assets/hotec_wheatstraw"
PRESENTATIONS = ("face-up", "lean30")
LEAN_DEG = 30.
DEPTH_MM = 20                                       # the plan's hold: the pads' centre 20 mm down the wall
EXTRA_DEPTHS_MM = (15, 25)                          # extra rows, not in the choice
ROWS = tuple(f"{p}/{d}" for d in (DEPTH_MM,) + EXTRA_DEPTHS_MM for p in PRESENTATIONS)
GOALS = (("plate_05", 0.), ("plate_03", 0.), ("plate_04", 0.), ("plate_04", 30.))   # (dish, pinch spin toward the front)
PLATES = ("plate_05", "plate_03", "plate_04")
ROLLS = PP.ROLLS
RZ_PI = PP.RZ_PI
N_YAW = PP.N_YAW
IMPORTED = ("code/sampler/robot/scripts/frigidaire_plate_presentations.py", "code/sampler/robot/scripts/frigidaire_robot_reach.py",
            "code/sampler/robot/scripts/frigidaire_robot_episode.py", "code/sampler/robot/grasp.py",
            "code/sampler/robot/collide.py", "data/assets/hotec_wheatstraw/source/hotec_gen.py",
            "data/assets/hotec_wheatstraw/parameters.json")
_W = {}


# ------------------------------------------------------------------------------------------------- the plate's profile
def hotec_gen():
    spec = importlib.util.spec_from_file_location("hotec_gen", HOTEC / "source/hotec_gen.py")
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


def profile():
    """hotec_gen's plate: the Curves (outer wall arc, its inward offset), the dense visual rows and the sparse collision rows
    ((outer r, z, inner r, z) in m), and the outer arc's arc length down from the rim edge."""
    if "prof" in _W:
        return _W["prof"]
    HG = hotec_gen()
    v = {k: e["value"] for k, e in json.loads((HOTEC / "parameters.json").read_text())["kinds"]["plate"].items()}
    cv = HG.Curves("plate", v)
    rows, dense, meta = HG.profile("plate", v)
    us = np.linspace(0., 1., 20001)
    P = np.array([cv.outer(u) for u in us])
    s_down = np.r_[np.cumsum(np.linalg.norm(np.diff(P, axis=0), axis=1)[::-1])[::-1], 0.]
    _W["prof"] = {"v": v, "cv": cv, "rows": np.asarray(rows), "dense": np.asarray(dense), "meta": meta, "us": us,
                  "s_down": s_down, "wall": float(v["wall_m"])}
    return _W["prof"]


def wall_point(depth_m):
    """The point ``depth_m`` down the outer wall arc from the rim edge: (u, outer point, unit tangent bottom -> top, outward
    normal, the inner surface along the normal) in the profile plane (r, z)."""
    pr = profile()
    u = float(np.interp(depth_m, pr["s_down"][::-1], pr["us"][::-1]))
    p, t = np.asarray(pr["cv"].outer(u), dtype=float), np.asarray(pr["cv"].tangent(u), dtype=float)
    n = np.array([t[1], -t[0]])                      # minus hotec_gen._inward: away from the material
    return u, p, t, n, np.asarray(pr["cv"].offset(u), dtype=float)


def pad_geometry(W):
    """The plate gate's closed finger pads (EP.closed_hulls_named, the last frame of EP.PLATE_GATE) in the TCP frame: their
    centre along the approach, half length (z), half width (x) and the gap between their inner faces."""
    from sampler.robot import collide as C
    Ti = np.linalg.inv(W.T_w3_tcp)
    pads = [np.asarray(v_) @ Ti[:3, :3].T + Ti[:3, 3]
            for nm, _, v_ in EP.closed_hulls_named(C, np, EP.PLATE_GATE[0], EP.PLATE_GATE[1]) if nm.endswith("finger_pad")]
    if len(pads) != 2:
        raise RuntimeError(f"expected 2 finger pads in the plate gate's closed jaws, got {len(pads)}")
    P = np.vstack(pads)
    lo, hi = sorted(pads, key=lambda p: float(p[:, 1].mean()))
    return {"centre_z_m": float((P[:, 2].min() + P[:, 2].max()) / 2), "half_len_m": float((P[:, 2].max() - P[:, 2].min()) / 2),
            "half_width_m": float(np.abs(P[:, 0]).max()), "gap_m": float(hi[:, 1].min() - lo[:, 1].max()),
            "z_range_m": [float(P[:, 2].min()), float(P[:, 2].max())],
            "source": f"{EP.PLATE_GATE[0].relative_to(ROOT)}/trials/{EP.PLATE_GATE[1]} (last frame), finger-pad hulls"}


def hold_in_plate(depth_m, pad):
    """The wall pinch's TCP in the plate frame, the pinch on the plate frame's +x meridian, roll 0."""
    pr = profile()
    _, p, t, n, _ = wall_point(depth_m)
    e, a = np.array([1., 0., 0.]), np.array([0., 0., 1.])
    T = np.eye(4)
    T[:3, 1] = n[0] * e + n[1] * a                   # closing axis: the outer surface's outward normal
    T[:3, 2] = -(t[0] * e + t[1] * a)                # approach: down the wall toward the foot
    T[:3, 0] = np.cross(T[:3, 1], T[:3, 2])          # the circumferential direction
    mid = p[0] * e + p[1] * a - pr["wall"] / 2 * T[:3, 1]
    T[:3, 3] = mid - pad["centre_z_m"] * T[:3, 2]
    return T


def wall_fit(depth_m, pad, T_hold):
    """How the wall sits between the closed pads (the pads flat in the TCP's y = const planes, over z = centre +- half length
    and x = +- half width): per profile (the dense visual rows, the sparse collision rows revolved smoothly) the outer
    surface's largest y and the inner surface's smallest y over the pads' rectangle; the gap the pads close to (both touching),
    the wall's centre off the pads' mid plane, the pads' contact positions along z; the arc's chord sagitta over the pad
    length; the collision facet's angle to the arc's tangent at the pinch point."""
    pr = profile()
    u, p, t, n, _ = wall_point(depth_m)
    Ti = np.linalg.inv(T_hold)
    zc, h, hw = pad["centre_z_m"], pad["half_len_m"], pad["half_width_m"]

    def dense_poly(P2):                               # 0.1 mm samples along the polyline
        out = []
        for a_, b_ in zip(P2[:-1], P2[1:]):
            k = max(2, int(np.linalg.norm(b_ - a_) / 1e-4) + 1)
            out.append(a_ + np.linspace(0., 1., k)[:, None] * (b_ - a_))
        return np.vstack(out)

    def env(P2):
        """(the largest / smallest TCP y within the pads' rectangle, the z where it is) over the surface of revolution of
        the profile polyline P2 (r, z), at the meridian and at the pads' side edges."""
        Q = dense_poly(P2)
        pts = []
        for s in (0., 1., -1.):
            ph = np.arcsin(np.clip(s * hw / np.maximum(Q[:, 0], hw), -1., 1.))
            X = np.c_[Q[:, 0] * np.cos(ph), Q[:, 0] * np.sin(ph), Q[:, 1], np.ones(len(Q))]
            pts.append((X @ Ti.T)[:, :3])
        Y = np.vstack(pts)
        return Y[(Y[:, 2] >= zc - h) & (Y[:, 2] <= zc + h) & (np.abs(Y[:, 0]) <= hw + 1e-4)]

    out = {}
    for nm, rows in (("visual", pr["dense"]), ("collision", pr["rows"])):
        Yo, Yi = env(rows[:, :2]), env(rows[:, 2:])
        yo, yi = float(Yo[:, 1].max()), float(Yi[:, 1].min())
        zo, zi = Yo[Yo[:, 1] >= yo - 1e-4][:, 2], Yi[Yi[:, 1] <= yi + 1e-4][:, 2]
        out[nm] = {"gap_mm": round((yo - yi) * 1e3, 2), "wall_centre_off_mid_mm": round((yo + yi) / 2 * 1e3, 2),
                   "outer_contact_z_mm": [round(float(zo.min()) * 1e3, 1), round(float(zo.max()) * 1e3, 1)],
                   "inner_contact_z_mm": [round(float(zi.min()) * 1e3, 1), round(float(zi.max()) * 1e3, 1)]}
    sd = pr["s_down"]                                 # the arc's chord sagitta over the pad's length (arc length +- h)
    s0, s1 = max(0., depth_m - h), depth_m + h
    m = (sd >= s0) & (sd <= s1)
    A = np.array([pr["cv"].outer(uu) for uu in pr["us"][m]])
    c0, c1 = A[0], A[-1]
    d = (c1 - c0) / np.linalg.norm(c1 - c0)
    sag = float(np.max(np.abs((A - c0) @ np.array([-d[1], d[0]]))))
    so = pr["rows"][:, :2]                            # the collision facet whose span along the profile holds the pinch point
    j = next(i for i, (a_, b_) in enumerate(zip(so[:-1], so[1:]))
             if 0. <= float((p - a_) @ (b_ - a_)) <= float((b_ - a_) @ (b_ - a_)))
    fd = (so[j + 1] - so[j]) / np.linalg.norm(so[j + 1] - so[j])
    out.update(arc_sagitta_over_pad_mm=round(sag * 1e3, 2), pad_arc_span_mm=[round(s0 * 1e3, 1), round(s1 * 1e3, 1)],
               pad_beyond_rim_edge_mm=round(max(0., h - depth_m) * 1e3, 1),
               collision_facet_vs_arc_deg=round(math.degrees(math.acos(float(np.clip(fd @ t, -1., 1.)))), 2),
               collision_facet_rz_mm=[np.round(so[j] * 1e3, 2).tolist(), np.round(so[j + 1] * 1e3, 2).tolist()],
               circ_sag_at_pad_edge_mm=round(float(p[0] * (1 - math.cos(math.asin(hw / p[0]))) * abs(n[0])) * 1e3, 2))
    return out


def hold_record(depth_m, pad, T_hold):
    """The hold's numbers for the json (profile plane r, z in mm; angles in deg)."""
    u, p, t, n, q = wall_point(depth_m)
    e, a = np.array([1., 0., 0.]), np.array([0., 0., 1.])
    y = T_hold[:3, 1]
    return {"depth_mm": round(depth_m * 1e3), "u": round(u, 5), "pinch_outer_rz_mm": np.round(p * 1e3, 3).tolist(),
            "pinch_inner_rz_mm": np.round(q * 1e3, 3).tolist(), "wall_slope_deg": round(math.degrees(math.atan2(t[1], t[0])), 2),
            "normal_rz": np.round(n, 5).tolist(), "approach_rz": np.round(-t, 5).tolist(),
            "closing_vs_rim_pinch_radial_deg": round(math.degrees(math.acos(min(1., abs(float(y @ e))))), 2),
            "closing_vs_edge_pinch_axial_deg": round(math.degrees(math.acos(min(1., abs(float(y @ a))))), 2),
            "T_tcp_in_plate": {"0": np.round(T_hold, 6).tolist(), "180": np.round(T_hold @ RZ_PI, 6).tolist()},
            "fit": wall_fit(depth_m, pad, T_hold)}


# ------------------------------------------------------------------------------------------------------------ the world
def goals(W):
    """{plate: (goal pose in the arm world's LowerRack frame, slot, the rack frame F_r, shift mm)} of PLATES (phase 1's
    goals() over three plates)."""
    from sampler.robot.rig import T_of
    load = json.loads(LOAD.read_text())
    F_goal = T_of(load["frames"]["LowerRack"]["position_m"], load["frames"]["LowerRack"]["quaternion_xyzw"])
    fr = R.frames_for(W.B, "LowerRack")["LowerRack"]
    F_r = T_of(fr["position_m"], fr["quaternion_xyzw"])
    out = {}
    for d in load["dishes"]:
        if d["name"] in PLATES:
            out[d["name"]] = (F_r @ np.linalg.inv(F_goal) @ T_of(d["goal_pose"]["pos"], d["goal_pose"]["quat_xyzw"]), d["slot"], F_r,
                              round(float(np.linalg.norm(F_r[:3, 3] - F_goal[:3, 3])) * 1e3, 2))
    return out


def world():
    """Per worker: phase 1's World (posture_v1 mount, the lower rack's arm with the held-plate margin, the plate geometry,
    the plate gate's closed jaws), the three goals, the closed pads and the holds per depth."""
    if "w" in _W:
        return _W["w"]
    from sampler.robot import collide as C
    m = json.loads(R.SAMPLE_MOUNT.read_text())["best"]
    W = R.World({"base": m["base"], "yaw_rad": m["yaw_rad"]})
    W.mount = {"base": m["base"], "yaw_rad": m["yaw_rad"]}
    W.arm = W.arms["LowerRack"]
    W.arm.held_margin = C.HELD_ARM_MARGIN_M
    P = R.visual_points_of(W.B, "plate")
    r = np.linalg.norm(P[:, :2], axis=1)
    near = P[r >= r.max() - PP.EDGE_REACH_M]
    W.geom = {"rim_r": float(r.max()), "height": float(P[:, 2].max()), "points": P, "centroid": P.mean(axis=0),
              "edge_z": float((near[:, 2].min() + near[:, 2].max()) / 2)}
    W.plate_closed = EP.closed_gripper_hulls(C, np, run=EP.PLATE_GATE[0], traj=EP.PLATE_GATE[1])
    W.q_home = R.home_q(W)
    W.goals = goals(W)
    W.pad = pad_geometry(W)
    W.holds = {d: hold_in_plate(d / 1e3, W.pad) for d in (DEPTH_MM,) + EXTRA_DEPTHS_MM}
    _W["w"] = W
    return W


def rot_z(a):
    T = np.eye(4)
    T[:2, :2] = [[math.cos(a), -math.sin(a)], [math.sin(a), math.cos(a)]]
    return T


def presentation(W, pres, k):
    """(plate pose, the pinch meridian in the world (unit, perpendicular to the plate axis), fixtures [bodies], fixture
    info) of one presentation yaw."""
    from sampler.robot import grasp as G
    if pres == "face-up":
        T_p = PP.flat_pose(R.COUNTER_XY)
        return T_p, G.bowl_grasps(T_p, N_YAW, "plate")[k]["T_tcp"][:3, 1], [], None
    T_p, ah, a, n = PP.edge_pose(W, k, LEAN_DEG)
    body = W.arm.checker._body("plate", W.P.pose_dict(T_p), "dish")
    fx, info = PP.backstop(W, T_p, ah, body)
    return T_p, n, fx, info


def wall_tcp(T_p, e, T_hold, roll):
    """The hold T_hold (plate frame, +x meridian) turned about the plate axis onto the meridian e, rolled."""
    e_p = np.asarray(T_p)[:3, :3].T @ np.asarray(e, dtype=float)
    return np.asarray(T_p) @ rot_z(math.atan2(e_p[1], e_p[0])) @ T_hold @ (RZ_PI if roll else np.eye(4))


def moment_split(W, T_tcp, M):
    """The plate's weight about the TCP at the lift (N m, the visual centroid as its centre of mass) in the TCP frame:
    about x (circumferential, in the pads' plane across their length), y (the closing axis), z (along the pads' length)."""
    T_plate = T_tcp @ M
    com = T_plate[:3, :3] @ W.geom["centroid"] + T_plate[:3, 3]
    tau = np.cross(com - T_tcp[:3, 3], np.array([0., 0., -9.81 * PP.PLATE_MASS_KG]))
    return {ax: round(float(abs(tau @ T_tcp[:3, i])), 4) for i, ax in enumerate("xyz")}


# ------------------------------------------------------------------------------------------------------------- the sweep
def sweep_job(args):
    """One (row, yaw): both rolls -> a row."""
    key, k = args
    pres, depth = key.split("/")
    W = world()
    arm = W.arm
    t0 = time.monotonic()
    T_p, e, fx, info = presentation(W, pres, k)
    for b in fx:
        arm.statics[b["id"]] = b
    try:
        dish = arm.checker._body("plate", W.P.pose_dict(T_p), "dish")
        row = {"presentation": key, "yaw_k": k, "plate_pose": np.round(T_p, 6).tolist(), "fixture": info, "rolls": {}}
        for roll in ROLLS:
            T_g = wall_tcp(T_p, e, W.holds[int(depth)], roll)
            rr = PP.pick(W, arm, dish, T_p, T_g, True)
            M = np.linalg.inv(T_g) @ T_p
            rr["T_tcp"] = np.round(T_g, 6).tolist()
            rr["M"] = np.round(M, 6).tolist()
            rr["moment_Nm"] = dict(zip(("closing_axis", "pads"), PP.moment(W, T_g, M)))
            rr["moment_tcp_axes_Nm"] = moment_split(W, T_g, M)
            row["rolls"][str(roll)] = rr
        row["pass"] = any(v["ok"] for v in row["rolls"].values())
        row["gate"] = max((v["gate"] for v in row["rolls"].values()), key=("none", "G1", "G2").index)
    finally:
        for b in fx:
            arm.statics.pop(b["id"], None)
        arm.set_dishes({})
    row["seconds"] = round(time.monotonic() - t0, 1)
    return row


# ---------------------------------------------------------------------------------------------------------- the I4 check
def i4_check(W, arm, T_goal, F_r, M, q_lift, T_lift, spin_deg=0., seat_rule=True):
    """frigidaire_plate_presentations.i4_check with the pinch spun ``spin_deg`` round the rim from the point highest at the
    goal (EP.sampler_bowl_tcp); hardware only. The rest is phase 1's code unchanged."""
    arm.set_dishes({})
    held = {"held": ("plate", M, "dish"), "ignore": ("dish",)}
    T_pin0 = EP.sampler_bowl_tcp(T_goal, M, 0., np)
    T_pin = EP.sampler_bowl_tcp(T_goal, M, spin_deg, np)
    T_g = T_pin @ M                                                    # the plate at its goal, the pinch spun round the rim
    x_dir, v, lean_t = EP.tine_axis(F_r, np)
    T_c = T_g.copy()
    T_c[:3, 3] += EP.SEAT_DX_M * x_dir
    labels = EP.rack_labels(len(arm.checker.parts["LowerRack"]))
    sk = EP.seat_search(arm, "plate", "dish", T_c, v, EP.scene_targets(arm, ignore=("dish",), statics=False), labels, np)
    out = {"pass": False, "gate": "none", "spin_deg": spin_deg, "seat_mm": sk.get("seat_mm"), "supports": sk.get("supports"),
           "pinch_shift_from_highest_mm": np.round((T_pin[:3, 3] - T_pin0[:3, 3]) * 1e3, 1).tolist()}
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
            ok, why = PP.descent_ok(W, arm, T_s, q, h_s, v, tool, targets, seat_rule)
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
        path, sec = R.leg_ok(W, arm, q_lift, q_top, held, PP.CARRY_RRT_S)
        if path is None:
            tries.append(f"no leg 1 from the lift to the top of the tine-axis line (RRT-Connect {PP.CARRY_RRT_S:.0f} s)")
            out.setdefault("leg1_diag", []).append(leg_diag(arm, q_lift, q_top, held, h_s))
            continue
        n_dn = max(3, int(round(float((T_top[:3, 3] - T_s[:3, 3]) @ v) / .005)) + 1)
        dn, why, ps = PP.line(W, arm, T_top, T_s, q_top, n_dn, held)
        if dn is None:
            tries.append(f"the line down the tine axis: {why}")
            continue
        out.update(pass_=True, gate="G2", why=None, line_up_mm=round(h_top * 1e3), leg1={"path": path, "s": sec},
                   start_h_mm=round(h_s * 1e3), plate_rotation_lift_to_start_deg=PP.rot_deg((T_lift @ M)[:3, :3], (T_s @ M)[:3, :3]),
                   tcp_rotation_lift_to_start_deg=PP.rot_deg(T_lift[:3, :3], T_s[:3, :3]),
                   q_start=np.round(q_s, 5).tolist(), q_top=np.round(q_top, 5).tolist(), T_start=np.round(T_s, 6).tolist())
        out["pass"] = out.pop("pass_")
        break
    out["carry_tries"] = tries[:6]
    if not out["pass"]:
        out["why"] = f"G2: carry: {tries[0] if tries else 'no start'}"
    return out


def leg_diag(arm, q0, q1, held, h_s):
    """A failed leg 1, described (not judged): the straight joint move's first blocking pairs (0.02 rad steps) and the
    largest joint travel."""
    q0, q1 = np.asarray(q0, dtype=float), np.asarray(q1, dtype=float)
    n = max(2, int(np.ceil(float(np.abs(q1 - q0).max()) / .02)) + 1)
    hit = next(((i / (n - 1), R.pairs_of(arm, q0 + (q1 - q0) * i / (n - 1), **held)) for i in range(n)
                if not arm.free(q0 + (q1 - q0) * i / (n - 1), **held)), (None, []))
    d = np.degrees(q1 - q0)
    return {"start_h_mm": round(h_s * 1e3), "straight_first_hit_at": None if hit[0] is None else round(hit[0], 3),
            "straight_first_hit_pairs": hit[1][:4], "joint_travel_deg": np.round(d, 1).tolist(),
            "largest_travel_deg": round(float(np.abs(d).max()), 1), "q_top": np.round(q1, 5).tolist()}


def i4_job(args):
    """One (row, roll, goal plate, spin): the I4 check from that roll's pick candidates in turn (the candidate's fixture in
    the scene), up to the first pass."""
    key, roll, name, spin, cands = args
    W = world()
    arm = W.arm
    t0 = time.monotonic()
    T_goal, slot, F_r, shift_mm = W.goals[name]
    res = {"dish": name, "slot": slot, "spin_deg": spin, "presentation": key, "roll": roll, "rack_frame_shift_mm": shift_mm,
           "tries": []}
    for c in cands:
        _, _, fx, _ = presentation(W, key.split("/")[0], c["yaw_k"])
        for b in fx:
            arm.statics[b["id"]] = b
        try:
            r = i4_check(W, arm, T_goal, F_r, np.asarray(c["M"]), np.asarray(c["q_lift"]), np.asarray(c["T_lift"]), spin)
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
    res.update(pass_=bool(best.get("pass")), gate=best.get("gate"), why=best.get("why"), start_mm=best.get("start_mm"),
               seconds=round(time.monotonic() - t0, 1))
    res["pass"] = res.pop("pass_")
    return res


def goal_key(name, spin):
    return name if not spin else f"{name} spin {spin:.0f}"


def i4_cell(r):
    """One I4 verdict, short: a pass as phase 1 wrote it; a carry failure after a descent start with the start and the
    carry's stop; else phase 1's blockers."""
    if r is None:
        return "no passing pick"
    if r["pass"]:
        return PP.i4_cell(r)
    t = max(r["tries"], key=lambda t: ("none", "G0", "G1", "G2").index(t["gate"])) if r["tries"] else None
    if t is not None and t["gate"] == "G1":
        why = (t.get("why") or "").replace("G2: carry: ", "")
        why = why.replace("no leg 1 from the lift to the top of the tine-axis line", "no leg 1 from the lift").replace("|", " vs ")
        dg = (t.get("leg1_diag") or [None])[0]
        if dg:
            why += (f"; the straight joint move is blocked at {dg['straight_first_hit_at']:.0%} by "
                    f"{', '.join(dg['straight_first_hit_pairs'][:2]).replace('|', ' vs ')}" if dg["straight_first_hit_at"] is not None
                    else "; the straight joint move is free at 0.02 rad steps") + f", largest joint travel {dg['largest_travel_deg']} deg"
        return f"G1, start {t['start_mm']} mm (yaw {t['yaw_k']}); carry fails: {why}"
    return PP.i4_fail(r)


# -------------------------------------------------------------------------------------------------------------- report
def pivot_label(pads, a_pads):
    f = pads / a_pads if (pads is not None and a_pads) else None
    return (None if f is None else f"likely ({f:.0%} of a's moment)" if f >= .75 else
            f"possible ({f:.0%} of a's moment)" if f >= .25 else f"no ({f:.0%} of a's moment)")


def summarize(rows, a_pads):
    """{row: summary} from the sweep rows (phase 1's columns), with the same per roll."""
    out = {}
    for key in ROWS:
        rs = sorted([r for r in rows if r["presentation"] == key], key=lambda r: r["yaw_k"])
        per = {}
        for roll in ("0", "180"):
            rr_ = [(r["yaw_k"], r["rolls"][roll]) for r in rs]
            bl, stages = Counter(), Counter()
            for _, rr in rr_:
                if not rr["ok"]:
                    bl.update(rr.get("blockers") or {})
                    stages[rr["stage"]] += 1
            oks = [rr for _, rr in rr_ if rr["ok"]]
            close = [rr["approach_last60"]["gripper_plate_min_mm"] for rr in oks
                     if rr["approach_last60"]["gripper_plate_min_mm"] is not None]
            per[roll] = {"pass": len(oks), "yaws": [k for k, rr in rr_ if rr["ok"]], "blockers": dict(bl.most_common(6)),
                         "fail_stages": dict(stages.most_common()),
                         "moment_pads_Nm": max((rr["moment_Nm"]["pads"] for _, rr in rr_), default=None),
                         "approach_last60_gripper_plate_min_mm": min(close) if close else None,
                         "turn_to_goal_deg_min": {nm: min((rr["turn_to_goal_deg"][nm] for rr in oks), default=None)
                                                  for nm in PLATES}}
        mom = [rr["moment_Nm"] for r in rs for rr in r["rolls"].values()]
        mx = [rr["moment_tcp_axes_Nm"] for r in rs for rr in r["rolls"].values()]
        pads = max((m["pads"] for m in mom), default=None)
        out[key] = {"pass": sum(r["pass"] for r in rs), "of": len(rs), "yaws": [r["yaw_k"] for r in rs if r["pass"]],
                    "pass_roll0": per["0"]["pass"], "pass_roll180": per["180"]["pass"], "per_roll": per,
                    "g1": sum(r["gate"] in ("G1", "G2") for r in rs),
                    "moment_Nm": {"closing_axis": max((m["closing_axis"] for m in mom), default=None), "pads": pads},
                    "moment_tcp_axes_Nm": {ax: max((m[ax] for m in mx), default=None) for ax in "xyz"},
                    "pivot_on_lift": pivot_label(pads, a_pads), "fixture": rs[0]["fixture"] if rs else None,
                    "extra": not key.endswith(f"/{DEPTH_MM}")}
    return out


def goals_passed(i4_row):
    """The goal plates (plate_04 by either spin, either roll) whose I4 passes."""
    return sorted({g.split(" ")[0] for per in (i4_row or {}).values() for g, r in per.items() if r["pass"]})


def choose(summ, i4_all):
    """The plan's rule over the 20 mm rows: the most passing yaws among the rows whose I4 passes at least one goal; ties:
    more goals passed, then no fixture (face-up)."""
    ok = [k for k in ROWS if not summ[k]["extra"] and goals_passed(i4_all.get(k))]
    if not ok:
        return None
    return max(ok, key=lambda k: (summ[k]["pass"], len(goals_passed(i4_all.get(k))), k.startswith("face-up")))


def fmt(x, dash="-"):
    return dash if x is None else x


def write_md(res, path, cmd):
    s, ia, h, pad = res["summary"], res["i4"], res["holds"], res["pad"]
    hz = h[str(DEPTH_MM)]
    fz = hz["fit"]
    a_p = res["a_pads_Nm"]
    L = ["# Plate wall pinch at the counter spot, Kit-free (phase 1 of plans/2026-11-04-plate-wall-pinch.md)", "",
         f"Command (Mac, {res['seconds']:.0f} s on {res['workers']} workers):", "", "```", cmd, "```", "",
         "The hold comes from hotec_gen's outer wall arc of the plate (data/assets/hotec_wheatstraw/source/hotec_gen.py with "
         f"parameters.json). The pinch point lies {DEPTH_MM} mm down that arc from the rim edge, the arc's top end at r "
         f"{res['rim_edge_rz_mm'][0]:.1f} mm, z {res['rim_edge_rz_mm'][1]:.2f} mm, where the lip's round-over starts. The "
         "closing axis (TCP y) is the outer surface's outward normal there. The approach (TCP z) is the arc's tangent toward "
         "the foot, so the fingers come down the wall from outside the rim edge. TCP x is the rim's circumferential "
         f"direction. The TCP sits so the closed pads' centre, {pad['centre_z_mm']} mm beyond the TCP in the plate gate's "
         "closed jaws, is at the pinch point, with the wall's mid surface between the pads. Roll 180 is the hand turned "
         "180 deg about the approach. A yaw passes when either roll passes. Rows at 15 and 25 mm are extra and stay out of "
         "the choice.", "",
         "The scene and the rules are phase 1's: the FCL arm model on the posture_v1 mount, the lower rack out and the upper "
         "rack pushed in, the counter (front edge -0.17 m), the pedestal, the floor and the backstop for lean30. The arm and "
         "the gripper keep the full 15 mm margin. Only the gripper may touch the plate, in the last 60 mm of the approach and "
         "at the grasp. The held plate keeps the 1 mm overlap rule against the scene and 2 mm against the links. G1 needs a "
         "free IK at the hover (0.10 m back along the approach), the grasp and the 0.15 m lift straight up with the plate "
         "held. G2 adds the approach line, the lift line and the home transit.", "",
         f"Pivot on lift is phase 1's ratio of the plate's weight moment on the pads at the lift to a's {a_p} N m (the flat "
         "rim pinch that pivoted 38 deg in Isaac). The gripper column is the open gripper's closest approach to the plate "
         "over the last 60 mm of the approach, over the passing picks. The turn column is the smallest rotation of the plate "
         "from the pick to the goals of plate_05, plate_03 and plate_04 (spin 0) over the passing picks.", "",
         "| presentation | down the wall (mm) | roll | yaws passing of 12 | passing yaws | commonest blockers "
         f"| pivot on lift (pads N m; a {a_p}) | gripper to plate, last 60 mm (mm) | turn to goals 05 / 03 / 04 (deg) |",
         "|---|---|---|---|---|---|---|---|---|"]
    for key in ROWS:
        x = s[key]
        pres, d = key.split("/")
        for roll in ("0", "180"):
            y = x["per_roll"][roll]
            bl = "; ".join(f"{k} ({v})" for k, v in list(y["blockers"].items())[:3]).replace("|", " vs ") or "none"
            tg = y["turn_to_goal_deg_min"]
            L.append(f"| {pres}{' (extra)' if x['extra'] else ''} | {d} | {roll} | {y['pass']} | "
                     f"{', '.join(map(str, y['yaws'])) or '-'} | {bl} | {pivot_label(y['moment_pads_Nm'], a_p)} "
                     f"({y['moment_pads_Nm']}) | {fmt(y['approach_last60_gripper_plate_min_mm'])} | "
                     f"{fmt(tg['plate_05'])} / {fmt(tg['plate_03'])} / {fmt(tg['plate_04'])} |")
    L += ["", "Yaws passing of 12 with either roll: " + "; ".join(
          f"{k.split('/')[0]} {k.split('/')[1]} mm{' (extra)' if s[k]['extra'] else ''} {s[k]['pass']}" for k in ROWS) + ".", "",
          "The I4 descent check runs from each roll's passing picks (up to two, the best facing first), hardware only: the "
          "seat along the tine axis, a descent start 50 to 150 mm above it with a free IK at the 15 mm carry margin, D-seat's "
          "2 mm rule on the final descent (the links and the plate gate's closed jaws plus wrist 3), the IK continuous 30 mm "
          "below the seat, the line up the tine axis at least 100 mm, leg 1 from the pick's lift (straight, else RRT-Connect "
          "45 s) and the line down. The pinch sits at the rim point highest at the goal. The last column spins it 30 deg "
          "toward the front at plate_04, as the site sweep's elbow-up pass did. G0 is an IK at the seat, G1 a descent start "
          "with a clear final descent, G2 the whole carry.", "",
          "| presentation | down the wall (mm) | roll | plate_05 (lower_front_04) | plate_03 (lower_front_00) "
          "| plate_04 (lower_rear_00) | plate_04 spun 30 deg to the front |", "|---|---|---|---|---|---|---|"]
    for key in ROWS:
        pres, d = key.split("/")
        for roll in ("0", "180"):
            per = (ia.get(key) or {}).get(roll) or {}
            L.append(f"| {pres}{' (extra)' if s[key]['extra'] else ''} | {d} | {roll} | " +
                     " | ".join(i4_cell(per.get(goal_key(n, sp))) for n, sp in GOALS) + " |")
    ch = res["chosen"]
    L += ["", ("The chosen presentation is " + ch["presentation"].split("/")[0] + f": {ch['why']}. Its pick candidates "
               "(yaw, roll, TCP, hold, plate pose, fixture, joint states and the lift) are under chosen in wall_pinch.json: "
               + ", ".join(f"({c['yaw_k']}, {c['roll']})" for c in ch['candidates']) + "."
               if ch.get("presentation") else f"No presentation is chosen: {ch['why']} in wall_pinch.json ("
               + ", ".join(f"({c['yaw_k']}, {c['roll']})" for c in ch['not_chosen']['candidates']) + ")."), ""]
    L += [f"The hold in numbers (r and z in mm from the plate's base centre; the fit uses the closed pads' "
          f"{2 * pad['half_len_mm']:.0f} by {2 * pad['half_width_mm']:.0f} mm rectangle):", "",
          "| down the wall (mm) | pinch point r, z (mm) | wall slope (deg) | closing axis vs the rim pinch's radial axis (deg) "
          "| vs the edge pinch's axial axis (deg) | arc chord sagitta over the pad (mm) | closed gap, visual / collision (mm) "
          "| wall off the pads' mid plane, visual / collision (mm) | collision facet vs the arc (deg) |",
          "|---|---|---|---|---|---|---|---|---|"]
    for d in (DEPTH_MM,) + EXTRA_DEPTHS_MM:
        x = h[str(d)]
        f = x["fit"]
        L.append(f"| {d}{' (extra)' if d != DEPTH_MM else ''} | {x['pinch_outer_rz_mm'][0]:.1f}, {x['pinch_outer_rz_mm'][1]:.1f} | "
                 f"{x['wall_slope_deg']} | {x['closing_vs_rim_pinch_radial_deg']} | {x['closing_vs_edge_pinch_axial_deg']} | "
                 f"{f['arc_sagitta_over_pad_mm']} | {f['visual']['gap_mm']} / {f['collision']['gap_mm']} | "
                 f"{f['visual']['wall_centre_off_mid_mm']} / {f['collision']['wall_centre_off_mid_mm']} | "
                 f"{f['collision_facet_vs_arc_deg']} |")
    L += ["", f"At {DEPTH_MM} mm the closing axis is {hz['closing_vs_rim_pinch_radial_deg']} deg from the rim pinch's radial "
          f"axis and {hz['closing_vs_edge_pinch_axial_deg']} deg from the edge pinch's axial axis. The pad's "
          f"{2 * pad['half_len_mm']:.0f} mm along the wall covers {fz['pad_arc_span_mm'][0]} to {fz['pad_arc_span_mm'][1]} mm "
          f"down the arc, and the arc's chord sagitta over that length is {fz['arc_sagitta_over_pad_mm']} mm. The outer "
          "surface bulges toward the outer pad and the inner surface curves toward the inner pad at the pad's ends, so the "
          f"flat pads close to {fz['visual']['gap_mm']} mm on the visual profile and {fz['collision']['gap_mm']} mm on the "
          f"collision rows, against the {res['wall_mm']} mm wall. On the visual profile the outer pad touches at z "
          f"{fz['visual']['outer_contact_z_mm'][0]} to {fz['visual']['outer_contact_z_mm'][1]} mm and the inner pad at z "
          f"{fz['visual']['inner_contact_z_mm'][0]} and {fz['visual']['inner_contact_z_mm'][1]} mm (TCP frame; the pad spans "
          f"{pad['z_range_mm'][0]} to {pad['z_range_mm'][1]} mm); on the collision rows the outer pad touches at z "
          f"{fz['collision']['outer_contact_z_mm'][0]} to {fz['collision']['outer_contact_z_mm'][1]} mm and the inner pad at "
          f"z {fz['collision']['inner_contact_z_mm'][0]} to {fz['collision']['inner_contact_z_mm'][1]} mm. The wall's "
          f"envelope sits {abs(fz['visual']['wall_centre_off_mid_mm'])} mm toward the inner pad, so a centred close pushes "
          "the plate that far along the closing axis toward the outer pad. Across the pad's width the rim's curvature adds "
          f"{fz['circ_sag_at_pad_edge_mm']} mm at the pad's edges. The collision facet under the pinch point runs from "
          f"r, z {fz['collision_facet_rz_mm'][0]} to {fz['collision_facet_rz_mm'][1]} mm.", "",
          f"The plate gate's closed jaws, which the descent check uses for the hand, hold the pads {pad['gap_mm']} mm apart. "
          f"The wall pinch closes them to about {fz['visual']['gap_mm']} mm, so each pad sits about "
          f"{(fz['visual']['gap_mm'] - pad['gap_mm']) / 2:.1f} mm further out than in those hulls. The 2F-85's pads closed on "
          f"air stop {res['pad_min_gap_mm']} mm apart in the recorded partA_bowl_foot_in trials, so the gap is inside the "
          "stroke.", ""]
    path.write_text("\n".join(L) + "\n")


def pad_min_gap_mm():
    """The smallest gap between the finger pads' inner faces when the jaws close on air: the partA_bowl_foot_in trials' entry
    frame (the closed fingertips entering the recess, nothing between the pads), the frame the trial records first."""
    from sampler.robot.rig import T_of
    from sampler.robot import ur5e
    run = ROOT / "data/results/robot/placing_v1/partA_bowl_foot_in"
    rm = json.loads((run / "trials/robot_meshes.json").read_text())
    paths = rm["body_paths"]
    idx = {k: next(j for j, p in enumerate(paths) if p.endswith(k)) for k in ("/wrist_3_link", "/left_inner_finger",
                                                                              "/right_inner_finger")}
    T_w3_tcp = T_of(ur5e.T_WRIST3_TCP_POS, ur5e.T_WRIST3_TCP_QUAT_XYZW)
    pads = {k: np.c_[np.asarray(rm["hulls"][str(idx[k])][1], dtype=float), np.ones(len(rm["hulls"][str(idx[k])][1]))]
            for k in ("/left_inner_finger", "/right_inner_finger")}
    gaps = []
    for f in sorted((run / "trials").glob("*.npz")):
        z = np.load(f)

        def Tb(t, i):
            w = z["body_quat_wxyz"][t, i]
            return T_of(z["body_pos"][t, i], (w[1], w[2], w[3], w[0]))
        for t in range(z["body_pos"].shape[0]):          # every frame commanded closed (0.8 rad): the smallest gap is
            if float(z["jaw"][t]) < .8 - 1e-6:           # the fingertips closed on air
                continue
            Ti = np.linalg.inv(Tb(t, idx["/wrist_3_link"]) @ T_w3_tcp)
            ys = {k: (Ti @ Tb(t, idx[k]) @ P.T).T[:, 1] for k, P in pads.items()}
            gaps.append(float(ys["/right_inner_finger"].min() - ys["/left_inner_finger"].max()))
    return round(min(gaps) * 1e3, 2) if gaps else None


def chosen_record(rows, summ, i4_all, holds, W):
    """The chosen row (choose) and its pick candidates for the s0 rerun; with no chosen row, the row with the most
    passing picks, recorded under not_chosen."""
    best = choose(summ, i4_all)
    top = max((k for k in ROWS if not summ[k]["extra"]), key=lambda k: (summ[k]["pass"], k.startswith("face-up")))
    pick = best or top                               # no chosen row: the most passing picks, recorded as not chosen
    cands = []
    for r in [r for r in rows if r["presentation"] == pick]:
        for roll in ("0", "180"):
            c = r["rolls"][roll]
            if not c["ok"]:
                continue
            tried = {g: next(({k: t.get(k) for k in ("pass", "gate", "start_mm", "start_h_mm", "why")} for t in x["tries"]
                              if t["yaw_k"] == r["yaw_k"]), None)
                     for g, x in ((i4_all.get(pick) or {}).get(roll) or {}).items()}
            cands.append({"yaw_k": r["yaw_k"], "roll": int(roll), "facing": c["facing"], "move_rad": c["move_rad"],
                          "T_tcp": c["T_tcp"], "M": c["M"], "plate_pose": W.P.pose_dict(np.asarray(r["plate_pose"])),
                          "fixture": r["fixture"], "q_hover": c["q_hover"], "q_grasp": c["q_grasp"], "q_lift": c["q_lift"],
                          "T_lift": c["T_lift"], "moment_Nm": c["moment_Nm"], "approach_last60": c["approach_last60"],
                          "transit": c["transit"], "i4": tried})
    cands.sort(key=lambda c: (-c["facing"], c["move_rad"]))
    i4_pick = {roll: {g: {k: x[k] for k in ("pass", "gate", "start_mm", "why")} for g, x in per.items()}
               for roll, per in (i4_all.get(pick) or {}).items()}
    if best:
        why = (f"{summ[best]['pass']} of 12 yaws passing at full clearance and an I4 pass at "
               f"{', '.join(goals_passed(i4_all[best]))}")
        others = [k for k in ROWS if not summ[k]["extra"] and k != best]
        why += "; " + "; ".join(f"{k.split('/')[0]} has {summ[k]['pass']} yaws and I4 passes at "
                                f"{', '.join(goals_passed(i4_all.get(k))) or 'none'}" for k in others)
        chosen = {"presentation": best, "why": why, "hold": holds[best.split("/")[1]], "candidates": cands, "i4": i4_pick}
    else:
        chosen = {"presentation": None, "why": (f"no {DEPTH_MM} mm row has an I4 pass at any goal; the most passing picks "
                                                f"are {top.split('/')[0]}'s {summ[top]['pass']} of 12 yaws, recorded under "
                                                f"not_chosen"),
                  "not_chosen": {"presentation": top, "hold": holds[top.split("/")[1]], "candidates": cands, "i4": i4_pick}}
    return chosen


def sha16(rel):
    return hashlib.sha256((ROOT / rel).read_bytes()).hexdigest()[:16]


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--workers", type=int, default=6)
    a = ap.parse_args()
    import multiprocessing as mp
    t0 = time.monotonic()
    OUT.mkdir(parents=True, exist_ok=True)
    shas = {f: sha16(f) for f in IMPORTED}
    a_pads = json.loads(PHASE1.read_text())["summary"]["a"]["moment_Nm"]["pads"]
    jobs = [(key, k) for key in ROWS for k in range(N_YAW)]     # the World is built in the workers only: numpy's Accelerate
                                                                # does not survive a fork after the parent's LAPACK calls
    print(f"[INFO] {len(ROWS)} rows x {N_YAW} yaws x {len(ROLLS)} rolls on {a.workers} workers", flush=True)
    rows = []
    with mp.get_context("fork").Pool(a.workers) as pool:
        for r in pool.imap_unordered(sweep_job, jobs, chunksize=1):
            rows.append(r)
            rr = r["rolls"]
            print(f"[INFO] {r['presentation']:12s} yaw {r['yaw_k']:2d}: pass {r['pass']} gate {r['gate']} "
                  f"r0 {rr['0']['stage'] or 'ok'} r180 {rr['180']['stage'] or 'ok'} {r['seconds']} s", flush=True)
        summ = summarize(rows, a_pads)
        orders = {key: PP.candidates(rows, key) for key in ROWS if summ[key]["pass"]}
        i4_jobs = [(key, roll, n, sp, [c for c in orders[key] if c["roll"] == roll])
                   for key in orders for roll in ROLLS for n, sp in GOALS
                   if any(c["roll"] == roll for c in orders[key])]
        print(f"[INFO] {len(i4_jobs)} I4 jobs at {time.monotonic() - t0:.0f} s", flush=True)
        i4_all = {}
        for r in pool.imap_unordered(i4_job, i4_jobs, chunksize=1):
            i4_all.setdefault(r["presentation"], {}).setdefault(str(r["roll"]), {})[goal_key(r["dish"], r["spin_deg"])] = r
            print(f"[INFO] I4 {r['presentation']} roll {r['roll']} {goal_key(r['dish'], r['spin_deg'])} {r['slot']}: pass "
                  f"{r['pass']} gate {r['gate']} {r['seconds']} s why {str(r['why'])[:160]}", flush=True)
    W = world()                                      # the parent's own World, after the pool: the hold records, the pads
    pad = W.pad
    holds = {str(d): hold_record(d / 1e3, pad, W.holds[d]) for d in (DEPTH_MM,) + EXTRA_DEPTHS_MM}
    print(f"[INFO] pads (closed, TCP frame): centre z {pad['centre_z_m'] * 1e3:.1f} mm, half length {pad['half_len_m'] * 1e3:.1f}, "
          f"half width {pad['half_width_m'] * 1e3:.1f}, gap {pad['gap_m'] * 1e3:.2f} mm", flush=True)
    for d, x in holds.items():
        print(f"[INFO] hold {d} mm: pinch r, z {x['pinch_outer_rz_mm']} mm, slope {x['wall_slope_deg']} deg, closing vs "
              f"radial {x['closing_vs_rim_pinch_radial_deg']}, vs axial {x['closing_vs_edge_pinch_axial_deg']}; fit "
              f"{json.dumps(x['fit'])}", flush=True)
    chosen = chosen_record(rows, summ, i4_all, holds, W)
    print(f"[INFO] chosen {chosen['presentation']}: {chosen['why']}", flush=True)
    pr = profile()
    res = {"plan": "plans/2026-11-04-plate-wall-pinch.md", "phase": 1, "load": str(LOAD.relative_to(ROOT)),
           "mount_file": str(R.SAMPLE_MOUNT.relative_to(ROOT)), "mount": W.mount, "arm_model": str(R.ARM_MODEL.relative_to(ROOT)),
           "counter_xy": list(R.COUNTER_XY), "counter_front_y": R.COUNTER_FRONT_Y, "lean_deg": LEAN_DEG,
           "wall_mm": round(pr["wall"] * 1e3, 2), "rim_edge_rz_mm": np.round(np.asarray(pr["cv"].outer(1.)) * 1e3, 3).tolist(),
           "outer_arc_length_mm": round(float(pr["s_down"][0]) * 1e3, 2),
           "pad": {"centre_z_mm": round(pad["centre_z_m"] * 1e3, 2), "half_len_mm": round(pad["half_len_m"] * 1e3, 2),
                   "half_width_mm": round(pad["half_width_m"] * 1e3, 2), "gap_mm": round(pad["gap_m"] * 1e3, 2),
                   "z_range_mm": [round(z * 1e3, 1) for z in pad["z_range_m"]], "source": pad["source"]},
           "pad_min_gap_mm": pad_min_gap_mm(),
           "holds": holds, "a_pads_Nm": a_pads, "a_source": str(PHASE1.relative_to(ROOT)),
           "rule": "full clearance (phase 1's pick gate): the arm and gripper 15 mm from the scene and the plate (the gripper may "
                   "touch the plate in the last 60 mm of the approach and at the grasp); the held plate 1 mm overlap vs the scene, "
                   "2 mm vs the arm links; G1 = free IK at the hover (0.10 m back along the approach), the grasp and the 0.15 m "
                   "lift straight up with the plate held; G2 = the approach line (100), the lift line (80) and the home -> hover "
                   "transit; a yaw passes when either roll passes. I4: phase 1's check (D-seat's 2 mm rule on the final descent, "
                   "the plate gate's closed jaws) per roll from up to two passing picks, the pinch at the rim point highest at "
                   "the goal (plate_04 also spun 30 deg toward the front)",
           "rows_order": list(ROWS), "extra_rows": [k for k in ROWS if summ[k]["extra"]],
           "summary": summ, "i4": i4_all, "chosen": chosen,
           "i4_candidates": {k: [{x: c[x] for x in ("yaw_k", "roll", "facing", "move_rad")} for c in o] for k, o in orders.items()},
           "rows": sorted(rows, key=lambda r: (ROWS.index(r["presentation"]), r["yaw_k"])),
           "imported_sha256_16": shas, "workers": a.workers, "seconds": round(time.monotonic() - t0, 1)}
    (OUT / "wall_pinch.json").write_text(json.dumps(res, indent=1, default=lambda o: o.tolist() if hasattr(o, "tolist")
                                                     else str(o)) + "\n")
    cmd = ("PYTHONPATH=code:code/util .venv-kitfree/bin/python code/sampler/robot/scripts/frigidaire_plate_wall_pinch.py "
           f"--workers {a.workers}")
    write_md(res, OUT / "kitfree.md", cmd)
    print("[RESULT] PASS plate wall pinch: " + ", ".join(f"{k} {summ[k]['pass']}/12" for k in ROWS) + "; I4 " +
          "; ".join(f"{k}: {', '.join(goals_passed(i4_all.get(k))) or 'none'}" for k in ROWS if k in i4_all) +
          f"; chosen {chosen['presentation']} in {res['seconds']} s -> {(OUT / 'wall_pinch.json').relative_to(ROOT)}", flush=True)


if __name__ == "__main__":
    main()
