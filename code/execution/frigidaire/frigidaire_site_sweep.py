#!/usr/bin/env python3
"""Kit-free sweep of four relaxations of the vertical descent at the six goal sites the s0 gate rejected.

    PYTHONPATH=code/src .venv-kitfree/bin/python code/execution/frigidaire/frigidaire_site_sweep.py --workers 6

Plan plans/2026-11-04-deep-sites-gate.md, phase 1 (no Isaac, no box). The scene, the arm model, the posture_v1 mount, the
racks (the dish's rack out, the other pushed in), the holds and the moves are the s0 round's (frigidaire_robot_reach.py
--sample-gate, imported read-only): bowls D-ring with the F-out foot-ring hold (placing_v1/partA_bowl_foot_out trial 9),
plates I4 with the rim-pinch hold (limits_v1/gate2_plate trial 1). The hold is rigid: the pick at the counter spot stays the
s0 round's, so every relaxation re-poses the held dish and the hand together at the descent.
  pitch  the TCP's approach tilted toward the front (the wrist toward -y), 0-90 deg. Plates: the pinch point spun about the
         plate's axis from the s0 rim point (the highest at the goal) toward the plate's front edge; the plate stays at its
         goal. Bowls: the foot-ring hold is axial (the approach is the bowl axis), so the pitch tilts the bowl with the hand
         about world x through its lowest point (mouth toward the back) and the bowl is re-seated.
  tilt   the dish turned from its goal's lean by -30..30 deg about the horizontal axis through its lowest point (bowls:
         world -x, + = mouth toward the front; plates: the lean axis, + = toward upright), then lowered along the descent
         axis (bowls the rack's up axis, plates the tine axis) to its Kit-free seat (EP.seat_search, 1 mm overlap).
         Bowls: pitch p and tilt t give one bowl pose, the net tilt t - p, evaluated once.
  slide  the descent ends 20/30/40 mm beside the release pose (8 horizontal directions; free = the held dish within 1 mm of
         the scene at the shifted pose, down the descent axis to it and along the slide), then a horizontal slide into it.
  elbow  every IK branch: up = the episode's elbow-up family (rig.Rig.branch_ok), down = the rest; 24 spins (bowls about
         their axis as D-ring; plates the pinch around the rim, whose front half is the pitch).
A combination passes when the s0 pick at the spot passes, the descent start is free at the carry margins (15 mm arm, the
held dish 2 mm from the links and within 1 mm of the scene), the final descent (and the slide) is IK-continuous with the
hand (closed jaws + wrist 3) and the links SEAT_LINKS 2 mm from the scene (D-seat's rule, EP.seat_line_hits), the jaws open
2 mm clear of the appliance and the dish at the release and on the retreat (bowls: up 100 mm; plates: back up to 100 mm
along -TCP z, truncated at the appliance, then up 100 mm from a pose 10 mm from the plate), and the carry from the pick's lift to the descent start is found (bowls:
transit_check with the D-ring carry rule, mouth down and the hand above; plates: the line up the tine axis, leg 1 by
RRT-Connect 45 s and the line down, the s0 gate's plate rule). Cost order: pitch, |tilt|, slide, elbow up first, joint move.
A predicted seat (a tilted dish's, a plate's I4 seat) counts only within LANDING_LIFT_MM of its pivot along the descent axis with
its rack's wires as its only supports (plans/2026-11-04-fifteen-of-fifteen.md D3, frigidaire_robot_reach.seat_at_goal), else the
cell is perched. --no-basket: no SilverwareBasket in the FCL world (frigidaire_robot_reach.use_basket(False); "basket" false).
Writes data/results/robot/samples_v1/<s0>__fixes/sites/reach.json and kitfree.md.
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
import frigidaire_robot_reach as R              # noqa: E402  read-only: World, frames, the pick, transits
import frigidaire_robot_episode as EP           # noqa: E402  read-only: the D-ring, I4 and D-seat constants and checks

S0_ID = "frigidaire_fdpc4221as__hotec_wheatstraw__plate5-bowl5-cup5_a3__s0"
S0 = ROOT / "data/results/robot/samples_v1" / S0_ID
OUT = ROOT / "data/results/robot/samples_v1" / f"{S0_ID}__fixes" / "sites"
SITES = ("bowl_04", "bowl_02", "bowl_03", "plate_01", "plate_02", "plate_04")
PITCHES = tuple(range(0, 91, 15))
TILTS = tuple(range(-30, 31, 10))
SLIDES_MM = (20, 30, 40)
SLIDE_DIRS = 8                                  # world horizontal directions, 45 deg apart from +x
SPINS = EP.RING_SPINS                           # 24
CARRY_TRIES = EP.DROP_TRIES                     # 6 candidates per cell and branch, as the episode's transit tries
LANDING_LIFT_MM = 20                            # a tilted dish must seat within this of its pivot along the descent axis (else
                                                # it perches above the site: recorded, never carried; the plate landing rule's dz)
IMPORTED = ("code/execution/frigidaire/frigidaire_robot_reach.py", "code/execution/frigidaire/frigidaire_robot_episode.py",
            "code/frigidaire/src/dishsim_frigidaire/robot/collide.py", "code/frigidaire/src/dishsim_frigidaire/robot/rig.py")
_S = {}


# ------------------------------------------------------------------ geometry helpers
def unit(v):
    v = np.asarray(v, dtype=float)
    return v / np.linalg.norm(v)


def rot_z(a):
    T = np.eye(4)
    T[:2, :2] = [[math.cos(a), -math.sin(a)], [math.sin(a), math.cos(a)]]
    return T


def rot_about(T, R3, p):
    out = np.array(T, dtype=float, copy=True)
    out[:3, :3] = R3 @ out[:3, :3]
    out[:3, 3] = p + R3 @ (out[:3, 3] - p)
    return out


def elbow_of(q):
    """'up' = rig.Rig.branch_ok's elbow-up family (shoulder lift in (-pi, 0), elbow in (0, pi)), else 'down'."""
    lift = (q[1] + np.pi) % (2 * np.pi) - np.pi
    el = (q[2] + np.pi) % (2 * np.pi) - np.pi
    return "up" if (-np.pi < lift < 0. and 0. < el < np.pi) else "down"


def all_branches(W, on=True):
    """IK over every branch (True) or the episode's elbow-up family only (False); W.kr's class is World's own subclass."""
    type(W.kr).ELBOW_UP_ONLY = not on


def pairs_of_hits(hits):
    """EP.seat_line_hits strings 'a|b 1.2 mm' -> 'a|b'."""
    return sorted({h.split(" ")[0] for h in hits})


def tally(d, gate, pairs):
    g = d.setdefault(gate, {"n": 0, "pairs": {}})
    g["n"] += 1
    for p in pairs:
        g["pairs"][p] = g["pairs"].get(p, 0) + 1


def top_pairs(fails, k=4):
    out = {}
    for gate, g in fails.items():
        out[gate] = {"n": g["n"], "pairs": [p for p, _ in sorted(g["pairs"].items(), key=lambda x: -x[1])[:k]]}
    return out


# ------------------------------------------------------------------ the world and the sites (built in the parent, inherited)
def world():
    if "W" not in _S:
        from dishsim_frigidaire.robot import collide as C
        m = json.loads(R.SAMPLE_MOUNT.read_text())["best"]
        W = R.World({"base": m["base"], "yaw_rad": m["yaw_rad"]})
        W.cent = W.B.kind_centroids({k: R.visual_points_of(W.B, k) for k in ("bowl", "plate", "cup")})
        W.holds = R.sample_holds()
        W.pts = {k: np.asarray(R.visual_points_of(W.B, k), dtype=float) for k in ("bowl", "plate")}
        fh = json.loads((R.PLACING / "partA_bowl_foot_out/inhand.json").read_text())
        run = R.PLACING / "partA_bowl_foot_out"
        rows = [json.loads(x) for x in (run / "trials.jsonl").read_text().splitlines() if x.strip()]
        r_ = next(x for x in rows if x["trial"] == fh["inhand"]["trial"])
        W.closed = {"bowl": EP.closed_gripper_hulls(C, np, run=run, traj=Path(r_["traj"]).name),
                    "plate": EP.closed_gripper_hulls(C, np, run=EP.PLATE_GATE[0], traj=EP.PLATE_GATE[1])}
        W.closed_src = {"bowl": str(r_["traj"]), "plate": f"{EP.PLATE_GATE[0].relative_to(ROOT)}/trials/{EP.PLATE_GATE[1]}"}
        _S["W"] = W
    return _S["W"]


def site_ctx(name):
    """Everything per site: the dish, its rack's arm world, the hold, the s0 pick (re-run, elbow-up as the episode), the goal
    in the gate's frames, the descent axis, the D-seat tool and targets."""
    if ("site", name) in _S:
        return _S[("site", name)]
    from dishsim_frigidaire.robot import collide as C
    from dishsim_frigidaire.robot.rig import T_of
    W = world()
    load = json.loads((S0 / "load.json").read_text())
    gate = json.loads((S0 / "gate.json").read_text())
    d = next(x for x in load["dishes"] if x["name"] == name)
    kind, rack = d["kind"], d["rack"]
    arm = W.arms[rack]
    arm.set_dishes({})
    arm.held_margin, arm.carry_ok = C.HELD_ARM_MARGIN_M, None
    M = np.asarray(W.holds[kind][0], dtype=float)
    all_branches(W, False)
    t0 = time.monotonic()
    pk = R.sample_pick(W, arm, kind, R.home_q(W))
    pk_s = round(time.monotonic() - t0, 1)
    fr = R.frames_for(W.B, rack)[rack]
    F_r = T_of(fr["position_m"], fr["quaternion_xyzw"])
    F_goal = T_of(load["frames"][rack]["position_m"], load["frames"][rack]["quaternion_xyzw"])
    T_goal = F_r @ np.linalg.inv(F_goal) @ T_of(d["goal_pose"]["pos"], d["goal_pose"]["quat_xyzw"])
    if kind == "bowl":
        axis, x_dir, T_base = unit(F_r[:3, 2]), None, T_goal
    else:
        x_dir, axis, _ = EP.tine_axis(F_r, np)
        T_base = EP.sampler_bowl_tcp(T_goal, M, 0., np) @ M        # the s0 spin: the pinch at the rim point highest
    ctx = {"name": name, "d": d, "kind": kind, "rack": rack, "arm": arm, "M": M, "pick": pk, "pick_s": pk_s,
           "q_lift": np.asarray(pk["q_lift"], dtype=float) if pk.get("q_lift") is not None else None,
           "T_lift": np.asarray(pk["T_lift"], dtype=float) if pk.get("T_lift") is not None else None,
           "T_goal": T_goal, "T_base": T_base, "axis": np.asarray(axis, dtype=float), "x_dir": x_dir,
           "cent": np.r_[W.cent[kind], 1.], "pts": W.pts[kind],
           "tool": EP.tool_hulls(arm, W.closed[kind]),
           "scene": EP.scene_targets(arm, ignore=("dish",)),
           "scene_seat": EP.scene_targets(arm, ignore=("dish",), statics=False),
           "labels": EP.rack_labels(len(arm.checker.parts["LowerRack"])),
           "s0": {k: gate["dishes"][name].get(k) for k in ("gate", "why", "blocker")}}
    _S[("site", name)] = ctx
    return ctx


def lowest(ctx, T):
    P = ctx["pts"] @ np.asarray(T)[:3, :3].T + np.asarray(T)[:3, 3]
    return P[int(np.argmin(P[:, 2]))]


def landing(ctx, T):
    """The release pose vs the goal: the centroid's dz and lateral offset (mm), the axis angle (deg)."""
    c, g = (np.asarray(T) @ ctx["cent"])[:3], (ctx["T_goal"] @ ctx["cent"])[:3]
    ang = math.degrees(math.acos(float(np.clip(np.asarray(T)[:3, 2] @ ctx["T_goal"][:3, 2], -1., 1.))))
    return {"dz_mm": round(float(c[2] - g[2]) * 1e3, 1), "lateral_mm": round(float(np.linalg.norm(c[:2] - g[:2])) * 1e3, 1),
            "axis_deg": round(ang, 1)}


def release_pose(ctx, tilt):
    """The dish at its release at spin 0 for the tilt key (bowls: the net tilt t - p, deg, + = mouth toward the front;
    plates: t, + = toward upright): the s0 pose (bowls the goal, plates the I4 seat) at 0, else turned about its lowest
    point and lowered along the descent axis to its seat. (T or None, info)"""
    key = ("rel", ctx["name"], tilt)
    if key in _S:
        return _S[key]
    kind, arm = ctx["kind"], ctx["arm"]
    T0 = ctx["T_base"]
    if kind == "bowl" and tilt == 0:
        sk = EP.seat_search(arm, kind, "dish", T0, ctx["axis"], ctx["scene_seat"], ctx["labels"], np)
        info = {"rule": "the goal (D-ring)", "goal_seat_mm": sk.get("seat_mm"), "goal_first_mm": sk.get("first_mm"),
                "goal_first_contacts": sk.get("first_contacts"),
                "note": "the bowl lowered alone along the rack's up axis at its goal orientation: first touch and seat (mm "
                        "above the goal); a first touch above 0 = the goal is not reachable straight down",
                "landing": landing(ctx, T0)}
        _S[key] = (T0, info)
        return _S[key]
    T = np.array(T0, dtype=float, copy=True)
    if tilt != 0:
        u = np.array([-1., 0., 0.]) if kind == "bowl" else unit(np.cross(T0[:3, 2], [0., 0., 1.]))
        T = rot_about(T0, R.rodrigues(u, math.radians(tilt)), lowest(ctx, T0))
    if kind == "plate":
        T[:3, 3] += EP.SEAT_DX_M * ctx["x_dir"]
    sk = EP.seat_search(arm, kind, "dish", T, ctx["axis"], ctx["scene_seat"], ctx["labels"], np)
    if sk.get("seat_mm") is None:
        _S[key] = (None, {"rule": "no seat along the descent axis", "seat": {k: v for k, v in sk.items() if k != "T_seat"}})
        return _S[key]
    T_seat = sk.pop("T_seat")
    info = {"rule": "the I4 seat" if tilt == 0 else "turned about its lowest point, then seated along the descent axis",
            "seat_mm": sk["seat_mm"], "first_mm": sk["first_mm"], "supports": sk.get("supports"),
            "seat_at_goal": R.seat_at_goal(sk, ctx["rack"], ctx["labels"], LANDING_LIFT_MM / 1e3),   # D3 (perched)
            "landing": landing(ctx, T_seat)}
    _S[key] = (T_seat, info)
    return _S[key]


def spun(ctx, T_rel, k):
    """The dish at its release turned by spin k: bowls about their own axis (D-ring); plates the pinch moved round the rim
    by 15 k deg from the rim point highest at this pose (sampler_bowl_tcp, the plate in place)."""
    if ctx["kind"] == "bowl":
        return np.asarray(T_rel) @ rot_z(2 * math.pi * k / SPINS)
    return EP.sampler_bowl_tcp(T_rel, ctx["M"], 360. * k / SPINS, np) @ ctx["M"]


def plate_pitch(ctx, T_rel, T_k):
    """The pinch's angle from the rim point highest (deg, + toward the plate's front edge, -y)."""
    Mi = np.linalg.inv(ctx["M"])
    a = unit(np.asarray(T_rel)[:3, 2])

    def rdir(T_d):
        r = (T_d @ Mi)[:3, 3] - T_d[:3, 3]
        return unit(r - (r @ a) * a)
    r0, r = rdir(spun(ctx, T_rel, 0)), rdir(T_k)
    th = math.degrees(math.atan2(float(np.cross(r0, r) @ a), float(r0 @ r)))
    return round(th if float(np.cross(a, r0)[1]) < 0. else -th)


def slide_vec(sl):
    if sl is None:
        return np.zeros(3)
    s_mm, k = sl
    a = 2 * math.pi * k / SLIDE_DIRS
    return s_mm / 1e3 * np.array([math.cos(a), math.sin(a), 0.])


def dish_free(ctx, T):
    """The held dish rule (ArmWorld.collisions): the checker's pair within its 1 mm allowance against the appliance, the
    counter and the static boxes. (ok, [bodies])"""
    arm = ctx["arm"]
    body = arm.checker._body(ctx["kind"], ctx_pose(T), "dish")
    bad = [nm for nm, b in ctx["scene"] if not arm.checker.pair(body, b)["valid"]]
    return not bad, bad


def ctx_pose(T):
    from dishsim_frigidaire import planner as P
    return P.pose_dict(np.asarray(T, dtype=float))


def dish_blockers(ctx, T):
    """The scene parts the dish overlaps at T over the 1 mm allowance (LowerRack wires by group, other bodies by name)."""
    if "aabbs" not in ctx:
        ctx["aabbs"] = EP.rack_aabbs(ctx["arm"], np)
    cs = EP.dish_contacts(ctx["arm"], ctx["kind"], T, "dish", ctx["scene"], ctx["labels"], ctx["aabbs"], np)
    return sorted({f"held|{lab}" for d_, _, lab in cs if d_ < -EP.SEAT_PEN_MAX_M})


def slide_dish_check(ctx, T_rel, sl):
    """The slide's dish-only path at spin 0 (the dish is axisymmetric): the shifted pose, the descent axis from the highest
    descent start down to it (5 mm), the horizontal slide into the release (2 mm)."""
    v = slide_vec(sl)
    T_end = np.array(T_rel, dtype=float, copy=True)
    T_end[:3, 3] += v
    top = EP.RING_ABOVE_M if ctx["kind"] == "bowl" else EP.SEAT_START_MAX_M
    for j in range(int(round(top / .005)), -1, -1):
        T = T_end.copy()
        T[:3, 3] += .005 * j * ctx["axis"]
        ok, bad = dish_free(ctx, T)
        if not ok:
            return {"ok": False, "why": f"the dish {j * 5} mm up the descent axis beside the site",
                    "pairs": dish_blockers(ctx, T) or [f"held|{b}" for b in bad]}
    n = max(2, int(round(np.linalg.norm(v) / .002)))
    for j in range(1, n):
        T = T_end.copy()
        T[:3, 3] -= v * j / n
        ok, bad = dish_free(ctx, T)
        if not ok:
            return {"ok": False, "why": f"the dish {round(np.linalg.norm(v) * (1 - j / n) * 1e3)} mm before the end of the slide",
                    "pairs": dish_blockers(ctx, T) or [f"held|{b}" for b in bad]}
    return {"ok": True, "why": None, "pairs": []}


# ------------------------------------------------------------------ the release side per spin
def release_check(ctx, T_tcp, q, T_dish):
    """The jaws open (the arm model's open gripper hulls + wrist 3, EP.JAW_LINKS) at least EP.JAW_MARGIN_M from the dish at
    its release and from the appliance, at the release and on the retreat (bowls: D-ring's RING_RISE_M straight up; plates:
    I4's back-off along -TCP z in BACK_STEP_M steps up to RELEASE_BACK_MAX_M, truncated at its last pose clear of the
    appliance, then the rise from there when the open jaws are RIM_CLEAR_M from the plate), each pose IK-continuous (0.1
    rad). (ok, why, pairs)"""
    W, arm, kind = world(), ctx["arm"], ctx["kind"]
    body = arm.checker._body(kind, ctx_pose(T_dish), "dish")
    comps = list((arm.checker.components or {}).items())
    lim = EP.JAW_MARGIN_M
    prev, T_last = np.asarray(q, dtype=float), np.asarray(T_tcp, dtype=float)

    def clear(q_):
        objs = [o for o in arm.arm_objects(q_, ("shoulder_link",)) if o[0] in EP.JAW_LINKS]
        d_c, n_c, _ = EP.min_clear(arm.fcl, objs, [(kind, body)], np, cutoff=.05)
        d_a, n_a, t_a = EP.min_clear(arm.fcl, objs, comps, np, cutoff=.05)
        return d_c, n_c, d_a, n_a, t_a

    def step(T, stage):
        nonlocal prev, T_last
        q_ = W.kr.ik(T, prev)
        if q_ is None or float(np.abs(q_ - prev).max()) > .1:
            return f"IK jump on the {stage}", []
        prev, T_last = q_, T
        return None, []
    d_c, n_c, d_a, n_a, t_a = clear(prev)
    if d_c < lim or d_a < lim:
        bad = ([f"{n_c}|dish"] if d_c < lim else []) + ([f"{n_a}|{t_a}"] if d_a < lim else [])
        return False, "the open jaws at the release", bad
    if kind == "plate":                               # I4's release(back=True): the back-off truncated at its last free pose,
        d_rim, stop = d_c, None                        # the rise only from a pose RIM_CLEAR_M from the plate
        for j in range(1, int(round(EP.RELEASE_BACK_MAX_M / EP.BACK_STEP_M)) + 1):
            T = np.asarray(T_tcp, dtype=float).copy()
            T[:3, 3] -= EP.BACK_STEP_M * j * T[:3, 2]
            q_b = W.kr.ik(T, prev)
            if q_b is None or float(np.abs(q_b - prev).max()) > .1:
                stop = f"IK jump {j * EP.BACK_STEP_M * 1e3:.0f} mm into the back-off"
                break
            d_c, n_c, d_a, n_a, t_a = clear(q_b)
            if d_c < lim:
                return False, f"the open jaws {j * EP.BACK_STEP_M * 1e3:.0f} mm into the back-off", [f"{n_c}|dish"]
            if d_a < lim:
                stop = f"the open jaws at the appliance {j * EP.BACK_STEP_M * 1e3:.0f} mm into the back-off ({n_a}|{t_a})"
                break
            prev, T_last, d_rim = q_b, T, d_c
        if d_rim < EP.RIM_CLEAR_M:
            return False, (f"the open jaws {d_rim * 1e3:.1f} mm from the plate where the back-off ends "
                           f"(under {EP.RIM_CLEAR_M * 1e3:.0f} mm: no rise; {stop})"), []
    T_b = T_last.copy()
    for j in range(1, int(round(EP.RING_RISE_M / .010)) + 1):
        T = T_b.copy()
        T[2, 3] += .010 * j
        why, _ = step(T, "rise")
        if why:
            return False, why, []
        d_c, n_c, d_a, n_a, t_a = clear(prev)
        if d_a < lim or d_c < lim:
            bad = ([f"{n_c}|dish"] if d_c < lim else []) + ([f"{n_a}|{t_a}"] if d_a < lim else [])
            return False, f"the open jaws {j * 10} mm into the rise", bad
    return True, None, []


def eval_spin(ctx, T_dish, sl, fails, found, k, pitch, by_spin):
    """One spin of one cell: per IK branch (found[branch] keeps every candidate) the descent start, the final descent (and
    the slide) under D-seat's rule and the release; fails tallies the gate and pairs per branch, by_spin[branch][k] keeps
    this spin's own verdict."""
    W, arm, kind, M = world(), ctx["arm"], ctx["kind"], ctx["M"]
    Mi = np.linalg.inv(M)
    held = {"held": (kind, M, "dish"), "ignore": ("dish",)}
    T_end = np.array(T_dish, dtype=float, copy=True)
    T_end[:3, 3] += slide_vec(sl)
    T_tr, T_te = np.asarray(T_dish) @ Mi, T_end @ Mi
    if kind == "bowl":
        c = np.asarray(T_dish) @ ctx["cent"]
        rule = ("hand not above the bowl (D-ring rule)" if T_tr[2, 3] <= c[2] else
                "mouth not down (D-ring rule)" if T_dish[2, 2] >= 0. else None)
        if rule:
            for e in ("up", "down"):
                tally(fails.setdefault(e, {}), rule, [])
                by_spin.setdefault(e, {})[k] = {"gate": rule, "pairs": []}
            return
    heights = ((EP.RING_ABOVE_M,) if kind == "bowl" else
               tuple(round(EP.SEAT_START_ABOVE_M + EP.SEAT_START_STEP_M * i, 3)
                     for i in range(int(round((EP.SEAT_START_MAX_M - EP.SEAT_START_ABOVE_M) / EP.SEAT_START_STEP_M)) + 1)))
    got, seen_ik = set(), {"up": False, "down": False}
    last = {}
    for h in heights:
        T_ds = T_te.copy()
        T_ds[:3, 3] += h * ctx["axis"]
        for q in W.kr.ik_all(T_ds, ctx["q_lift"]):
            e = elbow_of(q)
            seen_ik[e] = True
            if e in got:
                continue
            if not arm.free(q, **held):
                last[e] = ("descent start", q)             # the pairs only for the spin's last failure (19 ms each)
                continue
            n = int(round(h / .005)) + 1
            line = W.kr.line_q(T_ds, T_te, n, q)
            if line is None:
                last[e] = ("descent line IK jump", [])
                continue
            if kind == "plate" and sl is None:          # I4: IK-continuous down to DESCENT_BELOW_M under the seat
                prev, brk = line[-1], False
                for j in range(1, int(round(EP.DESCENT_BELOW_M * 1e3)) + 1):
                    T_j = T_te.copy()
                    T_j[:3, 3] -= j * 1e-3 * ctx["axis"]
                    qq = W.kr.ik(T_j, prev)
                    if qq is None or float(np.abs(qq - prev).max()) > .1:
                        brk = True
                        break
                    prev = qq
                if brk:
                    last[e] = ("IK not continuous to 30 mm under the seat", [])
                    continue
            bad = next((hh for qq in line for hh in [EP.seat_line_hits(arm, qq, ctx["tool"], ctx["scene"], np)] if hh), None)
            if bad:
                last[e] = ("final descent (D-seat 2 mm)", pairs_of_hits(bad))
                continue
            q_end = line[-1]
            if sl is not None:
                n_s = max(3, int(round(sl[0] / 2.)) + 1)
                sline = W.kr.line_q(T_te, T_tr, n_s, q_end)
                if sline is None:
                    last[e] = ("slide IK jump", [])
                    continue
                bad = next((hh for qq in sline for hh in [EP.seat_line_hits(arm, qq, ctx["tool"], ctx["scene"], np)] if hh), None)
                if bad:
                    last[e] = ("slide (D-seat 2 mm)", pairs_of_hits(bad))
                    continue
                q_end = sline[-1]
            ok, why, bad = release_check(ctx, T_tr, q_end, T_dish)
            if not ok:
                last[e] = (f"release: {why}", bad)
                continue
            found.setdefault(e, []).append({"spin": k, "pitch": pitch, "h_start_mm": round(h * 1e3), "q_start": q,
                                            "T_start": T_ds, "q_release": q_end, "T_dish": np.asarray(T_dish),
                                            "move": float(np.abs(q - ctx["q_lift"]).max())})
            got.add(e)
        if len(got) == 2:
            break
    for e in ("up", "down"):
        if e in got:
            by_spin.setdefault(e, {})[k] = {"gate": "pass", "pairs": []}
            continue
        if not seen_ik[e]:
            gate, pr = "no IK at the descent start", []
        elif e in last:
            gate, pr = last[e]
            if isinstance(pr, np.ndarray):
                pr = R.pairs_of(arm, pr, **held)
        else:
            continue
        tally(fails.setdefault(e, {}), gate, pr)
        by_spin.setdefault(e, {})[k] = {"gate": gate, "pairs": list(pr)[:4]}


def perched(tilt, rel):
    """D3 (plans/2026-11-04-fifteen-of-fifteen.md, frigidaire_robot_reach.seat_at_goal): a predicted seat (a tilted dish's, a
    plate's I4 seat at tilt 0) counts only within LANDING_LIFT_MM of its pivot along the descent axis with its rack's wires as its
    only supports; the bowl's goal at tilt 0 is no prediction (a record without the rule: the old tilted-dish height rule)."""
    sg = rel.get("seat_at_goal")
    if sg is None:
        return tilt != 0 and abs(rel.get("seat_mm") or 0) > LANDING_LIFT_MM
    return not sg["ok"]


def rel_job(args):
    """(site, tilt key) -> the release pose at spin 0 and its seat record."""
    name, tilt = args
    t0 = time.monotonic()
    T, info = release_pose(site_ctx(name), tilt)
    return name, tilt, T, {**info, "seconds": round(time.monotonic() - t0, 1)}


def cell_job(args):
    """(site, tilt key, slide[, release pose, its record]) -> the release side over the 24 spins: candidates and failures
    per branch."""
    name, tilt, sl = args[:3]
    t0 = time.monotonic()
    ctx = site_ctx(name)
    W = world()
    all_branches(W, True)
    ctx["arm"].set_dishes({})
    ctx["arm"].carry_ok = None
    T_rel, rel = args[3:5] if len(args) > 3 else release_pose(ctx, tilt)
    row = {"site": name, "kind": ctx["kind"], "tilt": tilt, "slide": sl, "release": rel, "dish_slide": None,
           "found": {}, "fails": {}, "seconds": None}
    if T_rel is None:
        row.update(why=rel["rule"], seconds=round(time.monotonic() - t0, 1))
        return row
    if perched(tilt, rel):
        row.update(why=f"perched: the {'tilted ' if tilt else ''}dish seats {rel['seat_mm']} mm up the descent axis from its pivot "
                       f"on {rel.get('supports')} (D3: within {LANDING_LIFT_MM} mm, on its rack's wires alone)")
        if sl is None:                                 # a note: does the arm reach the perched pose (never a pass)
            found = {}
            for k in range(SPINS):
                T_k = spun(ctx, T_rel, k)
                eval_spin(ctx, T_k, None, {}, found, k, plate_pitch(ctx, T_rel, T_k) if ctx["kind"] == "plate" else None, {})
            row["perched_reach"] = {e: len(v) for e, v in found.items()}
        row["seconds"] = round(time.monotonic() - t0, 1)
        return row
    if sl is not None:
        row["dish_slide"] = slide_dish_check(ctx, T_rel, sl)
        if not row["dish_slide"]["ok"]:
            row.update(why=f"slide side not free: {row['dish_slide']['why']}", seconds=round(time.monotonic() - t0, 1))
            return row
    found, fails, by_spin, pitches = {}, {}, {}, {}
    for k in range(SPINS):
        T_k = spun(ctx, T_rel, k)
        pitch = plate_pitch(ctx, T_rel, T_k) if ctx["kind"] == "plate" else None
        pitches[k] = pitch
        eval_spin(ctx, T_k, sl, fails, found, k, pitch, by_spin)
    row["found"] = {e: sorted(v, key=lambda c: c["move"]) for e, v in found.items()}
    row["fails"] = {e: top_pairs(f) for e, f in fails.items()}
    row["by_spin"] = {e: {str(k): {**v, "pitch": pitches[k]} for k, v in sorted(d.items())} for e, d in by_spin.items()}
    row["seconds"] = round(time.monotonic() - t0, 1)
    return row


# ------------------------------------------------------------------ the carry
def carry_ok_fn(ctx):
    from dishsim_frigidaire.robot import kin
    W, M, c_ = world(), ctx["M"], ctx["cent"]

    def ok(q):                                      # frigidaire_robot_episode ring_drop's carry_ok: mouth down, hand above
        T_t = W.kr.T_wb @ kin.fk_all_links(np.asarray(q, dtype=float))["wrist_3_link"] @ W.kr.T_w3_tcp
        T_b = T_t @ M
        return bool(T_b[2, 2] < 0. and T_t[2, 3] > (T_b @ c_)[2])
    return ok


def plate_carry(ctx, c):
    """The s0 gate's plate carry from the pick's lift: the line up the tine axis from the descent start toward z_safe (5 mm,
    IK jumps under SEAT_LINE_JUMP_RAD, free at the carry margins, at least SEAT_LINE_MIN_M), leg 1 to its top (straight
    joint move else RRT-Connect 45 s), the line down the tine axis to the descent start."""
    W, arm, v = world(), ctx["arm"], ctx["axis"]
    held = {"held": ("plate", ctx["M"], "dish"), "ignore": ("dish",)}
    T_s, q_s = np.asarray(c["T_start"]), np.asarray(c["q_start"])
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
            why_up = f"collides {h_full * j / n_up * 1e3:.0f} mm up the axis: {R.pairs_of(arm, q, **held)[:3]}"
            break
        prev, q_top, h_top = q, q, h_full * j / n_up
    legs = [{"leg": "the line up the tine axis", "free_mm": round(h_top * 1e3), "stopped": why_up}]
    if q_top is None or h_top < EP.SEAT_LINE_MIN_M - 1e-9:
        return {"ok": False, "legs": legs, "why": f"the line up the tine axis is under {EP.SEAT_LINE_MIN_M * 1e3:.0f} mm free: {why_up}"}
    T_top = T_s.copy()
    T_top[:3, 3] += h_top * v
    kind_, sec = R.leg_ok(W, arm, ctx["q_lift"], q_top, held, EP.SEAT_PLAN_RRT_S)
    legs.append({"leg": "leg 1 to the top of the tine-axis line", "path": kind_ or "rrt: none", "s": sec})
    if kind_ is None:
        return {"ok": False, "legs": legs, "why": f"no leg 1 to the top of the tine-axis line (RRT-Connect {EP.SEAT_PLAN_RRT_S:.0f} s)"}
    n_dn = max(3, int(round(h_top / .005)) + 1)
    dn, why = R.line_ok(W, arm, T_top, T_s, q_top, n_dn, held)
    legs.append({"leg": "down the tine axis", "path": "line" if dn is not None else "blocked"})
    return {"ok": dn is not None, "legs": legs, "why": None if dn is not None else f"the line down the tine axis: {why}"}


def carry_job(args):
    """(key = (site, tilt, slide, branch, pitch), candidates) -> the first candidate whose carry passes (up to CARRY_TRIES)."""
    key, cands = args
    name, tilt, sl, e, _ = key
    t0 = time.monotonic()
    ctx = site_ctx(name)
    W = world()
    all_branches(W, True)
    arm = ctx["arm"]
    arm.set_dishes({})
    held = {"held": (ctx["kind"], ctx["M"], "dish"), "ignore": ("dish",)}
    tries = []
    out = {"key": key, "site": name, "tilt": tilt, "slide": sl, "elbow": e, "ok": False, "pass": None, "tries": tries}
    for c in cands[:CARRY_TRIES]:
        if ctx["kind"] == "bowl":
            arm.carry_ok = carry_ok_fn(ctx)
            try:
                tr = R.transit_check(W, arm, ctx["q_lift"], ctx["T_lift"], c["q_start"], c["T_start"], held, rise_to=None)
            finally:
                arm.carry_ok = None
        else:
            tr = plate_carry(ctx, c)
        tries.append({"spin": c["spin"], "pitch": c["pitch"], "h_start_mm": c["h_start_mm"], "ok": tr["ok"], "why": tr["why"],
                      "legs": tr["legs"]})
        if tr["ok"]:
            out.update(ok=True, **{"pass": {"spin": c["spin"], "spin_deg": round(360. * c["spin"] / SPINS, 1),
                                            "pitch": c["pitch"], "h_start_mm": c["h_start_mm"],
                                            "q_start_deg": [round(float(x), 1) for x in np.degrees(c["q_start"])],
                                            "q_release_deg": [round(float(x), 1) for x in np.degrees(c["q_release"])],
                                            "move_deg": round(math.degrees(c["move"]), 1), "legs": tr["legs"]}})
            break
    out["seconds"] = round(time.monotonic() - t0, 1)
    return out


# ------------------------------------------------------------------ options, costs and the run
def bowl_pt(net):
    """The cheapest (pitch, tilt) of PITCHES x TILTS whose net tilt t - p is ``net`` (None if none)."""
    best = None
    for p in PITCHES:
        for t in TILTS:
            if t - p == net and (best is None or (p, abs(t)) < (best[0], abs(best[1]))):
                best = (p, t)
    return best


def cost(kind, tilt, sl, e, c):
    """Cost order: pitch, |tilt|, slide (none first), elbow up first, joint move."""
    if kind == "bowl":
        p, t = bowl_pt(tilt)
    else:
        p, t = (abs(c["pitch"]) + (0. if c["pitch"] >= 0 else .5)), tilt
    return (p, abs(t), 0 if sl is None else sl[0], 0 if e == "up" else 1, c["move"])


def jsonable(o):
    if isinstance(o, np.ndarray):
        return o.tolist()
    if isinstance(o, (np.floating, np.integer)):
        return o.item()
    return str(o)


def cand_rec(c):
    return {"spin": c["spin"], "spin_deg": round(360. * c["spin"] / SPINS, 1), "pitch": c["pitch"], "h_start_mm": c["h_start_mm"],
            "move_deg": round(math.degrees(c["move"]), 1), "q_start_deg": [round(float(x), 1) for x in np.degrees(c["q_start"])]}


def sha(rel):
    return hashlib.sha256((ROOT / rel).read_bytes()).hexdigest()[:16]


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--workers", type=int, default=6)
    ap.add_argument("--sites", default=",".join(SITES))
    ap.add_argument("--smoke", action="store_true", help="the s0 baseline cell of each site only (no carry search)")
    ap.add_argument("--out", type=Path, default=OUT)
    ap.add_argument("--mount", type=Path, default=None,
                    help="plans/2026-11-04-fifteen-of-fifteen.md: the sweep at this mount file's best (default posture_v1.json, "
                         "R.SAMPLE_MOUNT) and its spot_xy (frigidaire_robot_reach.use_mount); give it its own --out")
    ap.add_argument("--no-basket", action="store_true",
                    help="plans/2026-11-04-fifteen-of-fifteen.md (the user's basket-out decision): no SilverwareBasket in the FCL "
                         "world (frigidaire_robot_reach.use_basket(False)); reach.json records \"basket\": false")
    a = ap.parse_args()
    if a.mount is not None:                             # before world(): the parent builds it and the workers inherit it
        R.use_mount(a.mount)
    if a.no_basket:                                     # likewise
        R.use_basket(False)
    import multiprocessing as mp
    t_start = time.monotonic()
    out = a.out if a.out.is_absolute() else ROOT / a.out
    out.mkdir(parents=True, exist_ok=True)
    sites = [s for s in a.sites.split(",") if s]
    hashes = {p: sha(p) for p in IMPORTED}
    W = world()
    for s in sites:                                     # the picks in the parent: the workers inherit them by fork
        ctx = site_ctx(s)
        print(f"[INFO] {s} {ctx['d']['slot']} pick {ctx['pick']['pass']}/{ctx['pick']['of']} ok {ctx['pick']['ok']} "
              f"yaw {ctx['pick'].get('best_yaw_k')} ({ctx['pick_s']} s)", flush=True)
    nets = sorted({t - p for p in PITCHES for t in TILTS})
    keys = {s: ((0,) if a.smoke else tuple(nets) if _S[("site", s)]["kind"] == "bowl" else TILTS) for s in sites}
    cells = {}
    with mp.get_context("fork").Pool(a.workers) as pool:
        rels = {}                                       # stage A: the release pose per tilt key (the seat searches)
        for name, tilt, T, info in pool.imap_unordered(rel_job, [(s, k) for s in sites for k in keys[s]], chunksize=1):
            rels[(name, tilt)] = (T, info)
        jobs = []                                       # stage B: every key without a slide; the slides where the dish fits
        for s in sites:
            for k in keys[s]:
                T, info = rels[(s, k)]
                jobs.append((s, k, None, T, info))
                if a.smoke or T is None or perched(k, info):
                    continue
                jobs += [(s, k, (s_mm, dk), T, info) for s_mm in SLIDES_MM for dk in range(SLIDE_DIRS)]
        print(f"[INFO] {len(rels)} release poses ({sum(1 for (s, k), (T, i) in rels.items() if T is not None and not perched(k, i))} "
              f"fit), {len(jobs)} release-side cells on {a.workers} workers at {time.monotonic() - t_start:.0f} s", flush=True)
        for i, r in enumerate(pool.imap_unordered(cell_job, jobs, chunksize=1)):
            cells[(r["site"], r["tilt"], None if r["slide"] is None else tuple(r["slide"]))] = r
            if (i + 1) % 20 == 0 or i + 1 == len(jobs):
                print(f"[INFO] cells {i + 1}/{len(jobs)} at {time.monotonic() - t_start:.0f} s", flush=True)
        # the carry: every option cell (one relaxation at a time, both branches), then the cost order until a pass
        carry_res = {}

        def groups_of(s, only=None):
            """(cost, key, branch, candidates) per cell and branch (plates: per pinch spin) of site s."""
            gs = []
            for (s_, tilt, sl), r in cells.items():
                if s_ != s:
                    continue
                for e, cs in (r.get("found") or {}).items():
                    if r["kind"] == "plate":
                        by = {}
                        for c in cs:
                            by.setdefault(c["pitch"], []).append(c)
                        for pi, cc in by.items():
                            cc = sorted(cc, key=lambda c: c["move"])
                            gs.append((cost("plate", tilt, sl, e, cc[0]), (s, tilt, sl, e, pi), cc))
                    else:
                        gs.append((cost("bowl", tilt, sl, e, cs[0]), (s, tilt, sl, e, None), cs))
            return sorted(gs, key=lambda g: g[0])

        def run_carries(keys_cands):
            todo = [(k, cs) for k, cs in keys_cands if k not in carry_res]
            for res in pool.imap_unordered(carry_job, todo, chunksize=1):
                carry_res[tuple(res["key"])] = res
        if not a.smoke:
            opt = []
            for s in sites:
                kind = _S[("site", s)]["kind"]
                for _, key, cs in groups_of(s):
                    _, tilt, sl, e, pi = key
                    if kind == "bowl":
                        pt = bowl_pt(tilt)
                        one = sl is None and pt is not None and (pt[0] == 0 or pt[1] == 0)
                        one = one or (sl is not None and tilt == 0)
                    else:
                        one = (sl is None and (tilt == 0 or pi == 0) and (pi is not None and 0 <= pi <= 90)) or \
                              (sl is not None and tilt == 0 and pi == 0)
                    if one:
                        opt.append((key, cs))
            print(f"[INFO] carry for {len(opt)} one-at-a-time option groups at {time.monotonic() - t_start:.0f} s", flush=True)
            run_carries(opt)
            for s in sites:                             # the cost order, a round of `workers` groups at a time; then the
                for only_up in (False, True):           # elbow-up groups alone when the cheapest pass is elbow down
                    gs = [g for g in groups_of(s) if not only_up or g[1][3] == "up"]
                    if only_up and (not gs or not any(v["ok"] for k, v in carry_res.items() if k[0] == s)
                                    or any(v["ok"] for k, v in carry_res.items() if k[0] == s and k[3] == "up")):
                        continue
                    i = 0
                    while i < len(gs) and not any(carry_res.get(g[1], {}).get("ok") for g in gs[:i]):
                        run_carries([(g[1], g[2]) for g in gs[i:i + a.workers]])
                        i += a.workers
                        if time.monotonic() - t_start > 1800:
                            print("[INFO] the 30 min budget: the cost search stops here", flush=True)
                            break
                    print(f"[INFO] {s}: cost search{' (elbow up)' if only_up else ''} over {min(i, len(gs))}/{len(gs)} groups "
                          f"at {time.monotonic() - t_start:.0f} s", flush=True)
    res = summarize(a, sites, cells, carry_res if not a.smoke else {}, hashes, time.monotonic() - t_start)
    (out / "reach.json").write_text(json.dumps(res, indent=1, default=jsonable) + "\n")
    write_md(out / "kitfree.md", res)
    shown = (out / "reach.json").relative_to(ROOT) if (out / "reach.json").is_relative_to(ROOT) else out / "reach.json"
    print(f"[RESULT] PASS site sweep: {res['passing_sites']}/{len(sites)} sites pass "
          f"{[s for s in sites if res['sites'][s]['best']]} in {res['seconds']} s -> {shown}", flush=True)


def summarize(a, sites, cells, carry_res, hashes, secs):
    res = {"plan": "plans/2026-11-04-deep-sites-gate.md (phase 1)", "s0": str(S0.relative_to(ROOT)),
           "mount_file": R.rel_path(R.SAMPLE_MOUNT), "counter_xy": list(R.COUNTER_XY), "basket": bool(world().basket),
           "seat_rule": f"D3: a predicted seat within {LANDING_LIFT_MM} mm of its pivot along the descent axis on its rack's "
                        "wires alone (frigidaire_robot_reach.seat_at_goal), else perched",
           "arm_model": str(R.ARM_MODEL.relative_to(ROOT)),
           "holds": {k: world().holds[k][2] for k in ("bowl", "plate")}, "closed_jaws": world().closed_src,
           "imported_sha256_16": hashes, "rule": __doc__.split("\n\n", 2)[2].strip(),
           "grid": {"pitch_deg": PITCHES, "tilt_deg": TILTS, "slide_mm": SLIDES_MM, "slide_dirs": SLIDE_DIRS, "spins": SPINS,
                    "carry_tries": CARRY_TRIES, "landing_lift_mm": LANDING_LIFT_MM,
                    "slides_evaluated": "20, 30 and 40 mm x 8 directions at every tilt key whose dish fits (tilt 0 and the "
                                        "keys not perched)", "bowl_net_tilts": sorted({t - p for p in PITCHES for t in TILTS})},
           "sites": {}, "seconds": round(secs, 1), "smoke": bool(a.smoke), "n_cells": len(cells),
           "n_carry": len(carry_res)}
    for s in sites:
        ctx = _S[("site", s)]
        kind = ctx["kind"]
        rows = {k: r for k, r in cells.items() if k[0] == s}
        cell_out = []
        for (s_, tilt, sl), r in sorted(rows.items(), key=lambda x: (x[0][1], x[0][2] is not None, x[0][2] or (0, 0))):
            rec = {"tilt": tilt, "pitch_tilt": bowl_pt(tilt) if kind == "bowl" else None, "slide": sl,
                   "release": r.get("release"), "dish_slide": r.get("dish_slide"), "why": r.get("why"),
                   "seconds": r.get("seconds"), "by_spin": r.get("by_spin"), "perched_reach": r.get("perched_reach"),
                   "branches": {}}
            for e in ("up", "down"):
                cs = (r.get("found") or {}).get(e, [])
                b = {"candidates": len(cs), "best": [cand_rec(c) for c in cs[:CARRY_TRIES]],
                     "fails": (r.get("fails") or {}).get(e, {})}
                if kind == "plate":
                    b["pitches"] = sorted({c["pitch"] for c in cs})
                carries = [v for k, v in carry_res.items() if k[0] == s and k[1] == tilt and k[2] == sl and k[3] == e]
                if carries:
                    b["carry"] = [{k: v for k, v in c.items() if k not in ("key", "site", "tilt", "slide", "elbow")} for c in carries]
                rec["branches"][e] = b
            cell_out.append(rec)
        # the verdict of every option, one relaxation at a time from the s0 move (both branches, the 24 spins)
        def verdict(tilt, sl, e, pitch=None):
            r = rows.get((s, tilt, sl))
            if r is None:
                return {"verdict": "not evaluated"}
            if r.get("why"):
                return {"verdict": "fail", "at": r["why"], "pairs": (r.get("dish_slide") or {}).get("pairs", [])}
            cs = (r.get("found") or {}).get(e, [])
            if pitch is not None:
                cs = [c for c in cs if c["pitch"] == pitch]
            if not cs:
                f = (r.get("fails") or {}).get(e, {})
                if kind == "plate" and pitch is not None:
                    own = [v for v in (r.get("by_spin") or {}).get(e, {}).values() if v.get("pitch") == pitch]
                    f = {own[0]["gate"]: {"n": 1, "pairs": own[0]["pairs"]}} if own else {}
                return {"verdict": "fail", "at": "release side", "fails": f}
            cr = [v for k, v in carry_res.items() if k[0] == s and k[1] == tilt and k[2] == sl and k[3] == e
                  and (k[4] is None or pitch is None or k[4] == pitch)]
            if not cr:
                return {"verdict": "release side ok, carry not run", "candidates": len(cs)}
            ok = [c for c in cr if c["ok"]]
            if ok:
                return {"verdict": "pass", "candidates": len(cs), "combo": ok[0]["pass"]}
            return {"verdict": "fail", "at": "carry", "candidates": len(cs),
                    "why": [t_["why"] for c in cr for t_ in c["tries"]][:6]}
        opts = {"pitch": {}, "tilt": {}, "slide": {}, "elbow": {}}
        for p in PITCHES:
            for e in ("up", "down"):
                opts["pitch"].setdefault(str(p), {})[e] = verdict(-p, None, e) if kind == "bowl" else verdict(0, None, e, pitch=p)
        for t in TILTS:
            for e in ("up", "down"):
                opts["tilt"].setdefault(str(t), {})[e] = verdict(t, None, e) if kind == "bowl" else verdict(t, None, e, pitch=0)
        for s_mm in SLIDES_MM:
            for dk in range(SLIDE_DIRS):
                for e in ("up", "down"):
                    v = verdict(0, (s_mm, dk), e) if kind == "bowl" else verdict(0, (s_mm, dk), e, pitch=0)
                    opts["slide"].setdefault(f"{s_mm}mm@{dk * 45}deg", {})[e] = v
        opts["elbow"]["s0 move"] = {e: verdict(0, None, e) if kind == "bowl" else verdict(0, None, e, pitch=0)
                                    for e in ("up", "down")}
        # the cheapest passing combination (the plan's order), and the cheapest with the elbow up (the episode's branch rule)
        best, best_up, n_pass = None, None, 0
        for k, v in carry_res.items():
            if k[0] != s or not v["ok"]:
                continue
            n_pass += 1
            c = v["pass"]
            key = cost(kind, k[1], k[2], k[3], {"pitch": c["pitch"], "move": math.radians(c["move_deg"])})
            pt = bowl_pt(k[1]) if kind == "bowl" else (c["pitch"], k[1])
            rel = rows[(s, k[1], k[2])]["release"]
            rec = (key, {"pitch_deg": pt[0], "tilt_deg": pt[1], "net_tilt_deg": k[1] if kind == "bowl" else None,
                         "slide": None if k[2] is None else {"mm": k[2][0], "dir_deg": k[2][1] * 45},
                         "elbow": k[3], "spin_deg": c["spin_deg"], "descent_start_mm": c["h_start_mm"],
                         "move_deg": c["move_deg"], "q_start_deg": c["q_start_deg"], "q_release_deg": c["q_release_deg"],
                         "release": rel, "legs": c["legs"]})
            if best is None or key < best[0]:
                best = rec
            if k[3] == "up" and (best_up is None or key < best_up[0]):
                best_up = rec
        # the commonest blockers over every cell (per gate) when nothing passes
        agg = {}
        for r in rows.values():
            for e, f in (r.get("fails") or {}).items():
                for gate, g in f.items():
                    a_ = agg.setdefault(gate, {"n": 0, "pairs": {}})
                    a_["n"] += g["n"]
                    for p_ in g["pairs"]:
                        a_["pairs"][p_] = a_["pairs"].get(p_, 0) + 1
            if r.get("why"):
                a_ = agg.setdefault(r["why"].split(":")[0], {"n": 0, "pairs": {}})
                a_["n"] += 1
                for p_ in (r.get("dish_slide") or {}).get("pairs", []):
                    a_["pairs"][p_] = a_["pairs"].get(p_, 0) + 1
        for k, v in carry_res.items():
            if k[0] == s and not v["ok"]:
                a_ = agg.setdefault("carry", {"n": 0, "pairs": {}})
                a_["n"] += 1
        res["sites"][s] = {"slot": ctx["d"]["slot"], "kind": kind, "rack": ctx["rack"], "s0": ctx["s0"],
                           "pick": {k: ctx["pick"].get(k) for k in ("pass", "of", "yaws", "ok", "best_yaw_k", "why")},
                           "options": opts, "best": None if best is None else best[1],
                           "best_elbow_up": None if best_up is None else best_up[1], "passing_groups": n_pass,
                           "perched_reach": {str(k[1]): {"seat_mm": r["release"].get("seat_mm"), **(r.get("perched_reach") or {})}
                                             for k, r in sorted(rows.items(), key=lambda x: x[0][1])
                                             if k[2] is None and (r.get("perched_reach") or {})},
                           "blockers": {g: {"n": v["n"], "pairs": [p for p, _ in sorted(v["pairs"].items(), key=lambda x: -x[1])[:4]]}
                                        for g, v in sorted(agg.items(), key=lambda x: -x[1]["n"])},
                           "cells": cell_out}
    res["passing_sites"] = sum(1 for s in sites if res["sites"][s]["best"])
    return res


def write_md(path, res):
    """kitfree.md: the table (one relaxation at a time per column, the cheapest passing combination), how it was judged,
    the exact command and the run time."""
    import re

    def pairs_short(ps, k=3):
        return ", ".join(ps[:k]) if ps else "none recorded"

    def s0_cause(v):
        why = str(v["s0"].get("why") or "")
        if v["kind"] == "bowl":
            ps = re.findall(r"'([\w|]+)'", why.split("collides", 1)[-1])
            return f"held bowl or hand at every descent start (24 spins): {pairs_short(ps)}"
        return f"no descent start 50 to 150 mm above the seat: {pairs_short(v['s0'].get('blocker') or [])}"

    def why_of(v):
        """(gate label, perch height or None, pairs) of one failed option value."""
        at = str(v.get("at") or "")
        if at.startswith("perched"):
            m = re.search(r"seats (-?\d+) mm", at)
            return "perched", int(m.group(1)) if m else None, []
        if at.startswith("slide side not free"):
            return "no free side beside the site", None, list(v.get("pairs") or [])
        if at.startswith("no seat"):
            return "no seat", None, []
        if at.startswith("carry"):
            return "carry", None, [str(w) for w in (v.get("why") or [])][:1]
        if at.startswith("release side"):
            f = v.get("fails") or {}
            if not f:
                return "release side", None, []
            g, x = max(f.items(), key=lambda kv: kv[1]["n"])
            return g, None, list(x["pairs"])
        return str(v.get("verdict", "?")), None, []

    def opt_text(v, name):
        o = v["options"][name]
        passes = {}
        for val, by in o.items():
            for e, x in by.items():
                if x.get("verdict") == "pass":
                    passes.setdefault(e, []).append(val)
        if passes:
            return "pass: " + "; ".join(f"{e} {', '.join(vals)}" for e, vals in sorted(passes.items(), key=lambda kv: kv[0] != "up"))
        gates, seats, pairs = {}, [], {}
        for val, by in o.items():
            for e, x in by.items():
                if x.get("verdict") == "not evaluated":
                    continue
                w, n, ps = why_of(x)
                gates[w] = gates.get(w, 0) + 1
                for p_ in ps:
                    pairs.setdefault(w, {})[p_] = pairs.setdefault(w, {}).get(p_, 0) + 1
                if n is not None:
                    seats.append(n)
        if not gates:
            return "not evaluated"
        w = max(gates, key=gates.get)
        if w == "perched":
            return f"no pass: the dish perches {min(seats)} to {max(seats)} mm up"
        top = [p_ for p_, _ in sorted(pairs.get(w, {}).items(), key=lambda kv: -kv[1])[:2]]
        return f"no pass: {w}" + (f" ({', '.join(top)})" if top else "")

    def combo(b):
        sl = "no slide" if b["slide"] is None else f"slide {b['slide']['mm']} mm toward {b['slide']['dir_deg']} deg"
        legs = ", ".join(f"{l_['leg']} {l_.get('path', '')}".strip() for l_ in b["legs"] if l_.get("path"))
        return (f"pitch {b['pitch_deg']}, tilt {b['tilt_deg']}, {sl}, elbow {b['elbow']}, spin {b['spin_deg']} deg, descent "
                f"start +{b['descent_start_mm']} mm, joint move {b['move_deg']} deg ({legs})")
    lines = ["# Deep sites: Kit-free sweep at the six s0 gate rejects", "",
             f"Plan plans/2026-11-04-deep-sites-gate.md, phase 1. The sweep takes the six rejects of {res['s0']}/gate.json and "
             f"checks each site on the FCL arm model ({res['arm_model']}) at the mount {res['mount_file']}, with the dish's "
             "rack out, the other rack pushed in and no other dish. code/execution/frigidaire/frigidaire_site_sweep.py wrote "
             "this table and reach.json beside it.", "",
             "| site | cause in the s0 round | pitch | tilt | slide | elbow | best passing combination or the blockers |",
             "|---|---|---|---|---|---|---|"]
    for s, v in res["sites"].items():
        b, bu = v["best"], v.get("best_elbow_up")
        if b:
            best = combo(b) + ("" if b["elbow"] == "up" or not bu else f"; elbow up: {combo(bu)}")
        else:
            names = {"slide side not free": "no free side beside the site", "perched": "perched"}
            best = "none; " + "; ".join(f"{names.get(g, g)} ({x['n']})" + (f": {pairs_short(x['pairs'])}" if x["pairs"] else "")
                                        for g, x in list(v["blockers"].items())[:3])
        lines.append(f"| {s} {v['slot']} | {s0_cause(v)} | {opt_text(v, 'pitch')} | {opt_text(v, 'tilt')} | "
                     f"{opt_text(v, 'slide')} | {opt_text(v, 'elbow')} | {best} |")
    ok = [s for s, v in res["sites"].items() if v["best"]]
    lines += ["", f"Sites with a passing combination: {len(ok)} of {len(res['sites'])}" + (f" ({', '.join(ok)})." if ok else ".")]
    blocked, pr_lines = [], []
    for s, v in res["sites"].items():
        c0 = next((c for c in v["cells"] if c["tilt"] == 0 and c["slide"] is None), None)
        g = ((c0 or {}).get("release") or {}).get("goal_first_mm")
        if g is not None and g > 1:
            blocked.append((s, g))
        pr = v.get("perched_reach") or {}
        keys = [k for k, x in pr.items() if x.get("up") or x.get("down")]
        if keys:
            seats, ks = [pr[k]["seat_mm"] for k in keys], sorted(int(k) for k in keys)
            lab = "net tilts" if v["kind"] == "bowl" else "tilts"
            pr_lines.append(f"{s} at {len(ks)} {lab} from {ks[0]} to {ks[-1]} deg, the dish {min(seats)} to {max(seats)} mm up")
    if blocked:
        txt = " and ".join(f"{s} first touches the rack {g} mm above its goal" if i == 0 else f"{s} {g} mm above its goal"
                           for i, (s, g) in enumerate(blocked))
        lines += ["", f"Lowered alone at its goal orientation along the rack's up axis, {txt}, so a D-ring descent from above "
                      f"cannot reach {'either goal' if len(blocked) == 2 else 'these goals' if len(blocked) > 2 else 'that goal'}."]
    if pr_lines:
        lines += ["", "The arm does reach some perched poses, recorded in reach.json and never counted as a pass: " +
                  "; ".join(pr_lines) + "."]
    lines += ["",
              "Each option column is that relaxation alone from the s0 move, over the 24 spins and both elbow branches (up is "
              "the episode's elbow-up family, down is every other IK branch); a failing column names its commonest gate and "
              "blocking pairs. The counts in the last column tally the spins and branches that stopped at each gate over every "
              "cell, with a perched cell or a blocked slide counted once. Pitch tilts the hand toward the front in 15 deg steps up to 90. For a plate it moves the pinch point "
              "round the rim from the highest rim point toward the plate's front edge, so the plate stays at its goal. The "
              "bowl's foot-ring hold is axial, so a bowl pitch turns the bowl with the hand (mouth toward the back) and is the "
              "same pose as a negative tilt. Tilt turns the dish by -30 to 30 deg about its lowest point (bowls about world x, "
              "plus is the mouth toward the front; plates about the lean axis, plus is toward upright) and lowers it along the "
              f"descent axis to its seat. A tilted dish that seats more than {res['grid'].get('landing_lift_mm', 20)} mm up that axis "
              "is perched on the tines and fails. Slide ends the descent 20, 30 or 40 mm beside the release pose in one of 8 "
              "horizontal directions, then slides in; the held dish must stay within the 1 mm allowance of the scene on the "
              "way down beside the site and along the slide.", "",
              "A combination passes when the s0 pick at the counter spot passes, the descent start is free at the carry "
              "margins (15 mm for the arm, 2 mm for the held dish against the links, 1 mm overlap with the scene), the final "
              "descent and the slide keep the hand and the links 2 mm from the scene (D-seat's rule), the open jaws stay 2 mm "
              "clear of the appliance and the dish at the release and on the retreat, and the carry from the pick's lift to "
              "the descent start is found (bowls with the D-ring carry rule, plates with the s0 gate's leg 1 and tine-axis "
              "lines). The best combination follows the plan's order: the smallest pitch, then the smallest tilt, then no "
              "slide before a slide, then elbow up, then the smallest joint move. When it needs the elbow down, the cheapest "
              "elbow-up combination is given after it. A passing plate's spin is the spin_deg of EP.sampler_bowl_tcp at the "
              "I4 seat, the pinch that far round the rim from its highest point; Isaac has measured the rim-pinch hold only "
              "at the highest point.", "",
              "Command (Mac, repo root):", "",
              "    PYTHONPATH=code/src .venv-kitfree/bin/python code/execution/frigidaire/frigidaire_site_sweep.py --workers 6"
              + ("" if res.get("basket", True) else " --no-basket"),
              "", f"Run time: {res['seconds']} s on 6 workers ({res.get('n_cells', '?')} release-side cells, "
                  f"{res.get('n_carry', '?')} carry groups)."]
    path.write_text("\n".join(lines) + "\n")


if __name__ == "__main__":
    main()
