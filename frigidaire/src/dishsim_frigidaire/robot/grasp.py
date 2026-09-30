"""Top-down 2F-85 grasps of a HOTEC bowl in any resting pose (Kit-free).

Measured on the grip probe (frigidaire/scripts/setup/frigidaire_grip_probe.py, 2026-09-29), with the TCP of
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

import numpy as np

RIM_R, WALL, HEIGHT = .074, .0035, .075          # HOTEC bowl (assets/models/hotec_wheatstraw/v2/parameters.json)
FOOT_R, FOOT_W = .035, .005
RIM_INSERT_M = .025                              # rim top above the TCP
FOOT_INSERT_M = .006                             # ring top above the TCP
SIDE_INSET_M, SIDE_DROP_M = .008, .016           # pads 8 mm inside the rim edge, pad height centred on the rim point
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


def bowl_grasps(T_obj, n_yaw=12):
    """Candidate grasps [{"kind", "T_tcp", "yaw"}] for a bowl whose base-centred origin pose is ``T_obj``."""
    T_obj = np.asarray(T_obj, dtype=float)
    if not np.isfinite(T_obj).all():
        raise ValueError("non-finite bowl pose")
    o, a = T_obj[:3, 3], T_obj[:3, 2]
    unit(a, "bowl axis")                         # validated only: the values stay bit-identical to the PASS run
    kind = bowl_class(T_obj)
    out = []
    if kind in ("rim", "foot"):
        # a direction perpendicular to the axis to start the yaw sweep
        e1 = unit(np.cross(a, [1., 0., 0.]) if abs(a[0]) < .9 else np.cross(a, [0., 1., 0.]), "yaw-sweep start")
        e2 = np.cross(a, e1)
        for k in range(n_yaw):
            phi = 2 * np.pi * k / n_yaw
            n = np.cos(phi) * e1 + np.sin(phi) * e2                  # radial direction in the ring plane
            if kind == "rim":
                c = o + HEIGHT * a
                p = c + (RIM_R - WALL / 2) * n
                top = p[2]
                p = np.array([p[0], p[1], top - RIM_INSERT_M])
            else:
                p = o + (FOOT_R - FOOT_W / 2) * n                     # the ring top is the base plane (mouth down)
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


def hover(T_tcp, dz=.08):
    H = np.array(T_tcp, dtype=float, copy=True)
    H[2, 3] += dz
    return H


# ---------------------------------------------------------------------------- tilted approaches under the counter
# The pulled-out lower rack's rear ~12 cm lies under the 0.914 m counter's front edge (y = -0.30): a straight-down
# gripper there hits the slab (easy_s0 bowl_01 at y -0.291, goals at -0.289 / -0.306). Tilting the approach toward
# the front (-y) reaches under the overhang; the gripper body is checked against the slab as a capsule.
COUNTER_BOX = ((-.90, .90), (-.30, .30), (.874, .914))
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


def hits_counter(T_tcp, box=COUNTER_BOX, radius=GRIPPER_RADIUS_M, length=GRIPPER_LENGTH_M, above_to=1.3):
    """Gripper capsule (TCP back along -approach) or the vertical transit line above its hover vs the slab."""
    lo = np.array([box[0][0] - radius, box[1][0] - radius, box[2][0] - .01])
    hi = np.array([box[0][1] + radius, box[1][1] + radius, box[2][1] + .01])   # sideways by the radius, 1 cm on top
    pts = [T_tcp[:3, 3] - s * T_tcp[:3, 2] for s in np.linspace(.04, length, 12)]   # behind the fingertips
    h = hover_along(T_tcp)[:3, 3]
    pts += [np.array([h[0], h[1], z]) for z in np.linspace(h[2], above_to, 12)]
    return any(np.all(p > lo) and np.all(p < hi) for p in pts)


def tilted_grasps(T_obj, n_yaw=12):
    """``bowl_grasps`` with the least tilt that keeps the gripper and its transit line off the counter slab."""
    out = []
    for g in bowl_grasps(T_obj, n_yaw):
        for deg in TILTS_DEG:
            T = tilt(g["T_tcp"], deg)
            if not hits_counter(T):
                out.append({**g, "T_tcp": T, "tilt_deg": deg})
                break
    return out
