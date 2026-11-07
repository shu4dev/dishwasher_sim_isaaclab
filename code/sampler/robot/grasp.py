"""Top-down 2F-85 grasps of a HOTEC bowl in any resting pose (Kit-free).

Measured on the grip probe (code/sampler/robot/scripts/frigidaire_grip_probe.py, 2026-09-29), with the TCP of
``ur5e.T_WRIST3_TCP`` = the fingertip ENDS (the pads reach ~6 mm beyond and ~32 mm behind it), closing along the
TCP y axis, TCP z = approach (world down here):

- ``rim``   upright bowl (mouth up): pinch the rim wall; the bowl pivots ~30 mm about the pinch, then hangs stably
            (drift 0.3 mm / 3 s) -> the in-hand pose is MEASURED after the lift, never assumed;
- ``foot``  bowl mouth down: clamp the 4 mm foot ring off-centre (TCP on the ring wall, 6 mm deep): held, drift 0;
            a centred clamp is blocked (the open pads land on the widening body);
- ``side``  bowl on its side: pinch the rim where its wall is vertical (the rim point level with the axis),
            pads centred 8 mm inside the rim edge.
Poses are world 4x4 of the TCP (x, y = closing axis, z = approach).
"""
from __future__ import annotations

from pathlib import Path

import numpy as np

from . import ur5e

RIM_R, WALL, HEIGHT = .074, .0035, .075          # HOTEC bowl (data/assets/hotec_wheatstraw/parameters.json)
FOOT_R, FOOT_W = .035, .005
RIM_INSERT_M = .025                              # rim top above the TCP
FOOT_INSERT_M = .006                             # ring top above the TCP
SIDE_INSET_M, SIDE_DROP_M = .008, .016           # pads 8 mm inside the rim edge, pad height centred on the rim point
# mouth-up rim pinch per HOTEC kind: (rim radius, wall, height, rim top above the TCP); parameters.json nominal/estimated
# values (2026-10-23 limits round). Plate: the TCP 10 mm under the rim top so the fingertips stay ~24 mm above the counter.
KIND_RIM = {"bowl": (RIM_R, WALL, HEIGHT, RIM_INSERT_M), "cup": (.0711 / 2, .0025, .1049, .025),
            "plate": (.2286 / 2, .0035, .040, .010)}
# mouth-down foot-ring clamp per kind: (ring radius, ring width); plate = parameters.json foot_diameter_m / foot_width_m
KIND_FOOT = {"bowl": (FOOT_R, FOOT_W), "plate": (.14 / 2, .006)}
# cup wall pinch (2026-10-26, the cup's main grasp): the TCP on the cup axis pointing down, the pads straddling the cup
# OUTSIDE its wall and closing across its diameter. The pads span ~6 mm beyond to ~32 mm behind the TCP (robot.md 2):
# their centre is PAD_MID_BEHIND_TCP_M behind it. The brief's 35 mm pad depth (TCP 48 mm under the rim) jammed the cup's
# rim between the inner knuckles (32 N each, jaws stuck at 0 rad, 2026-10-26 run): measured on the open gripper's logged
# link hulls, the palm reaches 40 mm behind the TCP and the inner knuckles converge to 7 mm off the centreline 20-75 mm
# behind it. With the TCP 24 mm under the top the pads keep the smaller clearance (9.2 mm vs the knuckles' 11.9 mm, so
# the pads touch first); 28 mm already reverses it (9.5 vs 6.5). Pad centres 11 mm below the rim: the pads cover the top
# 30 mm of the wall (outer diameter ~67-71 mm on the linear 56 -> 71.1 mm taper). Mouth down (variant R re-pick) the same
# pinch goes across the base (56-60 mm).
PAD_MID_BEHIND_TCP_M = .013
WALL_PAD_BELOW_TOP_M = BASE_PAD_BELOW_TOP_M = .011
WALL_JAW_RAD = (.08, .45)                        # jaw window of a pinch across ~58-67 mm (0.8 rad = closed on air)
UPRIGHT, INVERTED = .7, -.7                      # bowl axis z component thresholds
AXIS_EPS = 1e-6                                  # a direction shorter than this is degenerate (plan Phase 0.5)


def unit(v, what="axis"):
    """``v`` normalised; raises ValueError when it is not finite or degenerate (never divides by ~0: a NaN closing
    axis from two coincident points was one of the previous session's process errors)."""
    v = np.asarray(v, dtype=float)
    n = float(np.linalg.norm(v))
    if not np.isfinite(v).all() or not np.isfinite(n) or n < AXIS_EPS:
        raise ValueError(f"degenerate {what}: {v.tolist()} (norm {n:.3g})")
    return v / n


def tcp_frame(p, closing):
    """TCP pose at ``p`` approaching straight down with the closing axis ``closing`` (made horizontal)."""
    y = unit([closing[0], closing[1], 0.], "closing axis (horizontal part)")
    z = np.array([0., 0., -1.])
    x = np.cross(y, z)
    T = np.eye(4)
    T[:3, 0], T[:3, 1], T[:3, 2], T[:3, 3] = x, y, z, p
    return T


def bowl_class(T_obj):
    a = np.asarray(T_obj)[:3, 2]
    return "rim" if a[2] > UPRIGHT else "foot" if a[2] < INVERTED else "side"


def bowl_grasps(T_obj, n_yaw=12, dish="bowl"):
    """Candidate grasps [{"kind", "T_tcp", "yaw"}] for a bowl whose base-centred origin pose is ``T_obj``; ``dish``
    cup/plate = the same mouth-up rim pinch with that kind's KIND_RIM (upright only)."""
    T_obj = np.asarray(T_obj, dtype=float)
    if not np.isfinite(T_obj).all():
        raise ValueError("non-finite bowl pose")
    o, a = T_obj[:3, 3], T_obj[:3, 2]
    unit(a, "bowl axis")                         # validated only: the values stay bit-identical to the PASS run
    kind = bowl_class(T_obj)
    if dish != "bowl" and kind != "rim" and not (dish == "plate" and kind == "foot"):
        raise ValueError(f"{dish}: only the mouth-up rim pinch exists (got {kind})")
    rim_r, wall, height, insert = KIND_RIM[dish]
    out = []
    if kind in ("rim", "foot"):
        # a direction perpendicular to the axis to start the yaw sweep
        e1 = unit(np.cross(a, [1., 0., 0.]) if abs(a[0]) < .9 else np.cross(a, [0., 1., 0.]), "yaw-sweep start")
        e2 = np.cross(a, e1)
        for k in range(n_yaw):
            phi = 2 * np.pi * k / n_yaw
            n = np.cos(phi) * e1 + np.sin(phi) * e2                  # radial direction in the ring plane
            if kind == "rim":
                c = o + height * a
                p = c + (rim_r - wall / 2) * n
                top = p[2]
                p = np.array([p[0], p[1], top - insert])
            else:
                p = o + (KIND_FOOT[dish][0] - KIND_FOOT[dish][1] / 2) * n   # the ring top is the base plane (mouth down)
                p = np.array([p[0], p[1], o[2] - FOOT_INSERT_M])
            if np.linalg.norm(n[:2]) < .5:                           # the wall is not vertical enough here
                continue
            out.append({"kind": kind, "T_tcp": tcp_frame(p, n), "yaw": float(phi)})
    else:
        c = o + HEIGHT * a
        side = unit(np.cross(a, [0., 0., 1.]), "side-grasp direction (bowl axis must not be vertical)")
        for s in (1., -1.):
            p = c + s * (RIM_R - WALL / 2) * side - SIDE_INSET_M * a
            p = np.array([p[0], p[1], p[2] - SIDE_DROP_M])
            out.append({"kind": kind, "T_tcp": tcp_frame(p, s * side), "yaw": float(s)})
    return out


def wall_grasps(T_obj, n_yaw=12, dish="cup", below_top=None):
    """Cup wall pinch: TCP on the cup axis pointing down, pads outside the wall closing across the diameter, pad
    centres ``below_top`` under the cup's top (the rim mouth up, the base mouth down); n_yaw closing directions."""
    T_obj = np.asarray(T_obj, dtype=float)
    o, a = T_obj[:3, 3], unit(T_obj[:3, 2], "cup axis")
    up = a[2] > 0
    if below_top is None:
        below_top = WALL_PAD_BELOW_TOP_M if up else BASE_PAD_BELOW_TOP_M
    top = o + KIND_RIM[dish][2] * a if up else o
    p = top - (below_top + PAD_MID_BEHIND_TCP_M) * (a if up else -a)       # on the axis, down the cup
    e1 = unit(np.cross(a, [1., 0., 0.]) if abs(a[0]) < .9 else np.cross(a, [0., 1., 0.]), "yaw-sweep start")
    e2 = np.cross(a, e1)
    out = []
    for k in range(n_yaw):
        phi = 2 * np.pi * k / n_yaw
        n = np.cos(phi) * e1 + np.sin(phi) * e2
        if np.linalg.norm(n[:2]) < .5:
            continue
        out.append({"kind": "wall" if up else "base", "T_tcp": tcp_frame(p, n), "yaw": float(phi)})
    return out


# cup side pinch (2026-10-26, plans/2026-10-26-side-pinch.md): the TCP approach HORIZONTAL (perpendicular to the cup
# axis), the closing axis horizontal, both pads outside the wall across the diameter, the pad centres ``below_top`` under
# the rim; TCP x along the cup axis, so a roll about the approach turns the cup mouth down, the gripper beside it. Depth
# from the open gripper's collider hulls (data/results/robot/arm_model.json, TCP frame): the pad hulls span 20 mm behind to
# 18 mm beyond the TCP (inner faces 87 mm apart) and move ~9 mm forward while closing on 60-70 mm; the palm face is 40 mm
# behind the TCP. With the cup axis SIDE_AXIS_BEYOND_TCP_M beyond the TCP the pads stay the closest part (9-14 mm open
# clearance at the three heights, palm 16-21 mm, inner knuckles 21-26 mm); with the axis at the old 13 mm-behind pad
# centre the palm would sit 7 mm inside the wall. The wrist: wrist_3's joint axis is TCP x (kin.py DH frame 6 z; the TCP
# sits 0.13 m along wrist_3's y), so wrist_2 hangs 0.10 m from wrist_3 along -TCP x. The 180 deg roll about the approach
# (the same pinch: the gripper is symmetric) moves the whole wrist across: ``wrist_side`` "mouth" puts it toward the cup's
# mouth (above the TCP for an upright cup), "base" toward its base (above the TCP for a mouth-down cup). Measured
# 2026-10-27: with "base" at the counter wrist_1/2 sit 0.10 m under the TCP, inside the slab, for every IK branch.
SIDE_BELOW_TOP_M = (.011, .045, .080)            # the swept pad heights under the rim
SIDE_AXIS_BEYOND_TCP_M = .010
WRIST_SIDES = ("mouth", "base")


def side_grasps(T_obj, n_yaw=12, below_top=SIDE_BELOW_TOP_M[0], dish="cup", wrist_side="mouth"):
    """Cup side pinch candidates [{"kind": "side", "T_tcp", "yaw", "below_top", "wrist_side"}]: n_yaw approach
    directions perpendicular to the cup axis (horizontal for an upright cup), pad centres ``below_top`` under the rim."""
    T_obj = np.asarray(T_obj, dtype=float)
    o, a = T_obj[:3, 3], unit(T_obj[:3, 2], "cup axis")
    x = -a if wrist_side == "mouth" else a                                # TCP x; the wrist sits along -x
    c = o + (KIND_RIM[dish][2] - below_top) * a                           # the axis point at the pad centres' height
    e1 = unit(np.cross(a, [1., 0., 0.]) if abs(a[0]) < .9 else np.cross(a, [0., 1., 0.]), "yaw-sweep start")
    e2 = np.cross(a, e1)
    out = []
    for k in range(n_yaw):
        phi = 2 * np.pi * k / n_yaw
        z = np.cos(phi) * e1 + np.sin(phi) * e2                         # approach: toward the cup axis
        T = np.eye(4)
        T[:3, 0], T[:3, 1], T[:3, 2], T[:3, 3] = x, np.cross(z, x), z, c - SIDE_AXIS_BEYOND_TCP_M * z
        out.append({"kind": "side", "T_tcp": T, "yaw": float(phi), "below_top": float(below_top), "wrist_side": wrist_side})
    return out


# bowl foot ring grasps (plans/2026-10-29-foot-ring-grasp.md): the bowl stands MOUTH DOWN on the counter, its vertical foot
# ring on top (the v3 asset of 2026-10-29: outer face r 32.86, inner face r 29.70 mm, 5.57 mm tall, read from parameters.json).
# The TCP straight down on the bowl axis, the closing axis horizontal at n_yaw yaws. "out" (F-out): the open pads close on the
# ring's outer face; "in" (F-in): the closed fingertips enter the recess and OPEN against its inner face. Depth (user): the
# fingertips reach the ring's base plane and no further (the wall flares wider just below the ring): the deepest pad bottom
# of the close (or of the descent and the open) sits FOOT_MARGIN_M above the base plane, so the pads engage the ring over
# its height minus the margin. The 1 mm margin covers the pad model's error (the cup side pinch's measured jaws: 0.8 mm).
# Pad model (TCP frame, y = the closing axis, z = the approach): the 2F-85's pads TRANSLATE on a circle about the inner
# knuckle's pivot by the finger joint angle (the parallelogram); measured on the quasi-static frames of the limits_v1 gate
# trajectories (49518 frames, 2026-10-29): open (0 rad) the right pad's inner face 43.55 mm from the axis and its bottom
# 18.42 mm beyond the TCP, closed on air (0.8 rad) 2.85 / 32.05 mm; thickness 19.2 mm and half width 11.0 mm (TCP x) from
# the arm model's open hulls (data/results/robot/arm_model.json).
FOOT_ASSET = Path(ur5e.ROOT) / "data/assets/hotec_wheatstraw/parameters.json"
PAD_OPEN_M, PAD_CLOSED_M = (.04355, .01842), (.00285, .03205)       # (inner face y, bottom z) at 0 and JAW_CLOSED_RAD
PAD_THICK_M, PAD_HALF_WIDTH_M = .0192, .011
JAW_CLOSED_RAD = .8
FOOT_MARGIN_M = .001
FOOT_SIDES = ("out", "in")
FOOT_JAW_RAD = {"out": (.10, .45), "in": (.30, .78)}               # blocked on the ring: closing (out) / opening (in); in from .45 to .30 (window 3: the loaded open stops at 0.43-0.44, the inner fingers give)


def _pad_arc():
    """The pad's offset from the pivot at 0 rad (y, z): (Rot(0.8) - I) v = closed - open."""
    c, s = np.cos(JAW_CLOSED_RAD), np.sin(JAW_CLOSED_RAD)
    d = np.subtract(PAD_CLOSED_M, PAD_OPEN_M)
    return np.linalg.solve(np.array([[c - 1., -s], [s, c - 1.]]), d)


def pad_at(theta):
    """(inner face y, bottom z beyond the TCP) of the right pad at finger angle ``theta`` (the left pad mirrors y)."""
    v, c, s = _pad_arc(), np.cos(theta), np.sin(theta)
    return float(PAD_OPEN_M[0] + (c - 1.) * v[0] - s * v[1]), float(PAD_OPEN_M[1] + s * v[0] + (c - 1.) * v[1])


def theta_at_inner(y):
    """The finger angle whose right pad's inner face is ``y`` from the axis (inner y falls monotonically 0 -> 0.8 rad)."""
    lo, hi = 0., JAW_CLOSED_RAD
    for _ in range(60):
        mid = (lo + hi) / 2
        lo, hi = (mid, hi) if pad_at(mid)[0] > y else (lo, mid)
    return (lo + hi) / 2


def foot_ring(path=None):
    """The bowl's foot ring from the asset's parameters.json: outer / inner face radius, height, bowl height (m)."""
    import json
    k = json.loads(Path(path or FOOT_ASSET).read_text())["kinds"]["bowl"]
    d, w, h, H = (float(k[n]["value"]) for n in ("foot_diameter_m", "foot_width_m", "foot_height_m", "height_m"))
    return {"outer_r": d / 2, "inner_r": d / 2 - w, "height": h, "bowl_height": H}


def foot_plan(side, ring=None, margin=FOOT_MARGIN_M):
    """The jaw angles and the TCP depth of one family: the contact angle, the entry angle (open for out, closed for in),
    the deepest pad bottom beyond the TCP over the close / the descent and the open, the engaged ring height at contact."""
    ring = ring or foot_ring()
    if side == "out":
        th_c, th_entry = theta_at_inner(ring["outer_r"]), 0.
    else:                                       # the flat pad's corners (TCP x +-half width) meet the inner face first
        corner = np.sqrt(ring["inner_r"] ** 2 - PAD_HALF_WIDTH_M ** 2)
        th_c, th_entry = theta_at_inner(corner - PAD_THICK_M), JAW_CLOSED_RAD
    ths = np.linspace(min(th_c, th_entry), max(th_c, th_entry), 81)
    deepest = max(pad_at(t)[1] for t in ths)
    bottom_c = pad_at(th_c)[1]
    return {"side": side, "theta_contact": float(th_c), "theta_entry": float(th_entry), "pad_bottom_contact_m": bottom_c,
            "pad_bottom_deepest_m": float(deepest), "margin_m": margin,
            "engaged_m": float(ring["height"] - margin - (deepest - bottom_c)), "ring": ring,
            "jaw_window_rad": FOOT_JAW_RAD[side]}


def foot_grasps(T_obj, n_yaw=12, side="out", margin=FOOT_MARGIN_M, ring=None):
    """Foot ring candidates [{"kind": "foot-out" | "foot-in", "T_tcp", "yaw", "side", "plan"}] for a mouth-down bowl whose
    base-centred origin pose is ``T_obj`` (the origin is the ring's end face centre, on top): the TCP on the bowl axis, the
    deepest pad bottom ``margin`` above the ring's base plane, n_yaw closing directions."""
    T_obj = np.asarray(T_obj, dtype=float)
    o, a = T_obj[:3, 3], unit(T_obj[:3, 2], "bowl axis")
    if a[2] > INVERTED:
        raise ValueError(f"foot grasps need the bowl mouth down (axis z {a[2]:.2f})")
    plan = foot_plan(side, ring, margin)
    p = o + (plan["ring"]["height"] - margin - plan["pad_bottom_deepest_m"]) * a      # a points down: up from the base plane
    e1 = unit(np.cross(a, [1., 0., 0.]) if abs(a[0]) < .9 else np.cross(a, [0., 1., 0.]), "yaw-sweep start")
    e2 = np.cross(a, e1)
    out = []
    for k in range(n_yaw):
        phi = 2 * np.pi * k / n_yaw
        n = np.cos(phi) * e1 + np.sin(phi) * e2
        out.append({"kind": f"foot-{side}", "T_tcp": tcp_frame(p, n), "yaw": float(phi), "yaw_k": k, "side": side, "plan": plan})
    return out


# the side ring hold (plans/2026-11-04-ring-side-hold.md): the cup side pinch's geometry on the foot ring of a mouth-down bowl. The
# approach (TCP z) horizontal toward the bowl axis at the ring's height, the closing axis (TCP y) horizontal across the ring, so the
# pads close on its outer face at two opposite points (F-out's contact and jaw window); TCP x along the bowl axis, both rolls
# (wrist_side "base": x = the axis, the wrist toward the bowl's base, above a mouth-down bowl; "mouth": the wrist below). The pad's
# edge toward the body (PAD_HALF_WIDTH_M along TCP x from the TCP) FOOT_MARGIN_M above the ring's base plane, so the pads engage the
# ring over its height minus the margin and stand above the bowl's back; the fingertips (the pad model's pad bottom at the contact
# angle) reach the bowl's axis and no further.
FOOT_JAW_RAD["side"] = FOOT_JAW_RAD["out"]          # the same contact on the ring's outer face


def foot_side_grasps(T_obj, n_yaw=12, rolls=WRIST_SIDES, margin=FOOT_MARGIN_M, ring=None):
    """Side ring hold candidates [{"kind": "foot-side", "T_tcp", "yaw", "yaw_k", "wrist_side", "side", "plan"}] for a mouth-down
    bowl whose base-centred origin pose is ``T_obj`` (the ring's end face centre, on top): per roll in ``rolls`` (wrist sides),
    n_yaw horizontal approach directions toward the axis; the TCP PAD_HALF_WIDTH_M - (ring height - margin) beyond the end face
    along the axis and the F-out contact's pad bottom short of the axis along the approach."""
    T_obj = np.asarray(T_obj, dtype=float)
    o, a = T_obj[:3, 3], unit(T_obj[:3, 2], "bowl axis")
    if a[2] > INVERTED:
        raise ValueError(f"foot side grasps need the bowl mouth down (axis z {a[2]:.2f})")
    plan = dict(foot_plan("out", ring, margin))
    tip = plan["pad_bottom_contact_m"]                # the fingertips at the contact angle (the deepest over the close)
    c = o + (plan["ring"]["height"] - margin - PAD_HALF_WIDTH_M) * a     # the TCP's level on the axis
    plan.update(side="side", tip_beyond_tcp_m=float(tip), pad_low_edge_above_base_m=float(margin),
                tcp_beyond_end_face_m=float(PAD_HALF_WIDTH_M - (plan["ring"]["height"] - margin)),
                jaw_window_rad=FOOT_JAW_RAD["side"])
    e1 = unit(np.cross(a, [1., 0., 0.]) if abs(a[0]) < .9 else np.cross(a, [0., 1., 0.]), "yaw-sweep start")
    e2 = np.cross(a, e1)
    out = []
    for w in rolls:
        x = a if w == "base" else -a                  # the wrist sits along -TCP x
        for k in range(n_yaw):
            phi = 2 * np.pi * k / n_yaw
            z = np.cos(phi) * e1 + np.sin(phi) * e2  # the approach: toward the axis
            T = np.eye(4)
            T[:3, 0], T[:3, 1], T[:3, 2], T[:3, 3] = x, np.cross(z, x), z, c - tip * z
            out.append({"kind": "foot-side", "T_tcp": T, "yaw": float(phi), "yaw_k": k, "wrist_side": w, "side": "side",
                        "plan": plan})
    return out


EDGE_OUT_M = .003                                 # the plate's edge pinch: the TCP this far outside the rim edge (the open
                                                  # pads' tips reach 15 mm inside it; plans/2026-11-04-plate-pick-presentation.md)
RZ_PI = np.diag([-1., -1., 1., 1.])               # the TCP turned 180 deg about its approach (the same pinch, wrist across)


def edge_grasps(T_obj, rim_r, edge_z, out=EDGE_OUT_M):
    """The top-down edge pinch of a plate standing or leaning on its edge (plans/2026-11-04-plate-pick-presentation.md,
    amended after phase 1): at the rim point highest (n = the in-plane up, the world up made perpendicular to the plate
    axis a), TCP z = -n (radially inward), TCP y = a (the pads on the mouth face and the base face, closing along the
    axis), the TCP ``out`` outside the rim edge (radius ``rim_r``) at ``edge_z`` up the axis from the base-centred origin;
    both 180 deg rolls about the approach. [{"kind": "edge", "T_tcp", "yaw", "roll_deg"}]"""
    T_obj = np.asarray(T_obj, dtype=float)
    o, a = T_obj[:3, 3], unit(T_obj[:3, 2], "plate axis")
    n = unit(np.array([0., 0., 1.]) - a[2] * a, "in-plane up (a plate lying flat has no top edge)")
    T = np.eye(4)
    T[:3, 0], T[:3, 1], T[:3, 2] = np.cross(a, -n), a, -n
    T[:3, 3] = o + edge_z * a + (rim_r + out) * n
    return [{"kind": "edge", "T_tcp": T @ Rr, "yaw": float(np.radians(r)), "roll_deg": r}
            for r, Rr in ((0, np.eye(4)), (180, RZ_PI))]


# the plate's wall-normal pinch (plans/2026-11-04-plate-wall-pinch.md; the hold of code/sampler/robot/scripts/
# frigidaire_plate_wall_pinch.py, ported for its phase 2): hotec_gen's outer wall arc of the plate (foot ring edge to the rim
# edge, from the asset's parameters.json), walked PLATE_WALL_DEPTH_M down from the rim edge; there the closing axis (TCP y) is
# the outer surface's outward normal, the approach (TCP z) the arc's tangent toward the foot, TCP x = y x z, and the TCP sits
# so the closed pads' centre (PLATE_WALL_PAD_Z_M beyond the TCP: the plate gate's closed finger pads, wall_pinch.json pad) is
# on the wall's mid surface. The pinch is turned to the rim point highest at the plate's pose plus a spin; both rolls.
PLATE_WALL_DEPTH_M = .020
PLATE_WALL_PAD_Z_M = .01297
HOTEC_GEN = Path(ur5e.ROOT) / "data/assets/hotec_wheatstraw/source/hotec_gen.py"


def plate_wall_hold(depth_m=PLATE_WALL_DEPTH_M, pad_z_m=PLATE_WALL_PAD_Z_M, params=None):
    """The wall pinch's TCP in the plate's base-centred frame on its +x meridian, roll 0 (4x4)."""
    import importlib.util
    import json
    spec = importlib.util.spec_from_file_location("hotec_gen", HOTEC_GEN)
    HG = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(HG)
    v = {k: e["value"] for k, e in json.loads(Path(params or FOOT_ASSET).read_text())["kinds"]["plate"].items()}
    cv = HG.Curves("plate", v)
    us = np.linspace(0., 1., 20001)
    P = np.array([cv.outer(u) for u in us])
    s_down = np.r_[np.cumsum(np.linalg.norm(np.diff(P, axis=0), axis=1)[::-1])[::-1], 0.]
    u = float(np.interp(depth_m, s_down[::-1], us[::-1]))
    p, t = np.asarray(cv.outer(u), dtype=float), np.asarray(cv.tangent(u), dtype=float)
    n = np.array([t[1], -t[0]])                       # the outward normal (away from the material)
    e, a = np.array([1., 0., 0.]), np.array([0., 0., 1.])
    T = np.eye(4)
    T[:3, 1] = n[0] * e + n[1] * a
    T[:3, 2] = -(t[0] * e + t[1] * a)
    T[:3, 0] = np.cross(T[:3, 1], T[:3, 2])
    T[:3, 3] = p[0] * e + p[1] * a - float(v["wall_m"]) / 2 * T[:3, 1] - pad_z_m * T[:3, 2]
    return T


def plate_wall_grasps(T_obj, T_hold, spins_deg=(0.,), rolls=(0, 180)):
    """The wall pinch on a plate at ``T_obj``: the hold ``T_hold`` (plate frame) turned about the plate axis so its meridian is
    the rim point highest at T_obj plus each spin, each roll (180 = RZ_PI). [{"kind": "plate-wall", "T_tcp", "yaw",
    "spin_deg", "roll_deg"}] in the order given."""
    T_obj, T_hold = np.asarray(T_obj, dtype=float), np.asarray(T_hold, dtype=float)
    a = unit(T_obj[:3, 2], "plate axis")
    n = unit(np.array([0., 0., 1.]) - a[2] * a, "in-plane up (a plate lying flat has no rim point highest)")
    n_d = T_obj[:3, :3].T @ n
    r_p = unit(np.array([T_hold[0, 3], T_hold[1, 3], 0.]), "the hold's meridian")
    out = []
    for s in spins_deg:
        phi = np.arctan2(np.cross(r_p, n_d)[2], r_p @ n_d) + np.radians(s)
        Rz = np.eye(4)
        Rz[:2, :2] = [[np.cos(phi), -np.sin(phi)], [np.sin(phi), np.cos(phi)]]
        for r in rolls:
            out.append({"kind": "plate-wall", "T_tcp": T_obj @ Rz @ T_hold @ (RZ_PI if r else np.eye(4)),
                        "yaw": float(np.radians(s)), "spin_deg": float(s), "roll_deg": int(r)})
    return out


def hover(T_tcp, dz=.08):
    H = np.array(T_tcp, dtype=float, copy=True)
    H[2, 3] += dz
    return H


# ---------------------------------------------------------------------------- tilted approaches under the counter
# The pulled-out lower rack's rear ~12 cm lies under the 0.914 m counter's front edge (y = -0.30): a straight-down
# gripper there hits the slab (easy_s0 bowl_01 at y -0.291, goals at -0.289 / -0.306). Tilting the approach toward
# the front (-y) reaches under the overhang; the gripper body is checked against the slab as a capsule.
COUNTER_BOX = ((-.90, .90), (-.30, .30), (.874, .914))
COUNTER_FRONT_Y = -.17                            # robot runs since 2026-10-27 (plans/2026-10-27-cup-side-drop.md)


def pull_counter(B, front_y=COUNTER_FRONT_Y):
    """The counter slab's front edge (the robot side) moved to ``front_y``, its back edge and top kept, in the bench module
    ``B`` (B.COUNTER: the Kit slab, the FCL slab, the counter band) and in COUNTER_BOX; frigidaire_bench.py keeps -0.30."""
    global COUNTER_BOX
    (x0, x1), (_, y1), zs = COUNTER_BOX
    COUNTER_BOX = ((x0, x1), (front_y, y1), zs)
    (cx, cy, cz), (sx, sy, sz) = B.COUNTER["center_m"], B.COUNTER["size_m"]
    back = cy + sy / 2
    B.COUNTER = {**B.COUNTER, "size_m": (sx, back - front_y, sz), "center_m": (cx, (back + front_y) / 2, cz)}
    return B.COUNTER
TILTS_DEG = (0., 15., 25., 35.)
GRIPPER_RADIUS_M, GRIPPER_LENGTH_M = .09, .30     # the 2F-85 + wrist behind the TCP (open fingers reach +-65 mm), as a capsule


def tilt(T_tcp, deg):
    """Rotate the TCP frame about the grasp point so the approach leans toward -y (the gripper body moves to -y)."""
    b = np.radians(deg)
    R = np.array([[1, 0, 0], [0, np.cos(b), -np.sin(b)], [0, np.sin(b), np.cos(b)]])   # about world +x
    T = np.array(T_tcp, dtype=float, copy=True)
    T[:3, :3] = R @ T[:3, :3]
    return T


def hover_along(T_tcp, d=.10):
    """Back off along the approach axis (TCP -z), where the arm comes from."""
    H = np.array(T_tcp, dtype=float, copy=True)
    H[:3, 3] -= d * H[:3, 2]
    return H


def hits_counter(T_tcp, box=None, radius=GRIPPER_RADIUS_M, length=GRIPPER_LENGTH_M, above_to=1.3):
    """Gripper capsule (TCP back along -approach) or the vertical transit line above its hover vs the slab."""
    box = COUNTER_BOX if box is None else box                                 # pull_counter's edge
    lo = np.array([box[0][0] - radius, box[1][0] - radius, box[2][0] - .01])
    hi = np.array([box[0][1] + radius, box[1][1] + radius, box[2][1] + .01])   # sideways by the radius, 1 cm on top
    pts = [T_tcp[:3, 3] - s * T_tcp[:3, 2] for s in np.linspace(.04, length, 12)]   # behind the fingertips
    h = hover_along(T_tcp)[:3, 3]
    pts += [np.array([h[0], h[1], z]) for z in np.linspace(h[2], above_to, 12)]
    return any(np.all(p > lo) and np.all(p < hi) for p in pts)


def tilted_grasps(T_obj, n_yaw=12, dish="bowl"):
    """``bowl_grasps`` with the least tilt that keeps the gripper and its transit line off the counter slab."""
    out = []
    for g in bowl_grasps(T_obj, n_yaw, dish):
        for deg in TILTS_DEG:
            T = tilt(g["T_tcp"], deg)
            if not hits_counter(T):
                out.append({**g, "T_tcp": T, "tilt_deg": deg})
                break
    return out
