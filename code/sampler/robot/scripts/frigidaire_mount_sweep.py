#!/usr/bin/env python3
"""Kit-free pedestal mount sweep over all 15 dishes of s0 with today's holds (plans/2026-11-04-fifteen-of-fifteen.md, B).

    PYTHONPATH=code:code/util .venv-kitfree/bin/python code/sampler/robot/scripts/frigidaire_mount_sweep.py --workers 6

The grid: base x 0.36 / 0.45 / 0.60 / 0.75 / 0.90, y -0.70 / -0.80 / -0.90 / -1.00, z 0.45 / 0.60 / 0.75 (60 mounts; x 0.75
and 0.90 since round 1's winner sat on the 0.60 edge), the base +x facing posture_v1's work centre (posture_v1's rule; its own
base is the grid point 0.45, -0.70, 0.60). A mount is out when its base
lies in the open door and extended racks footprint (x in (-0.36, 0.36) and y in (-0.95, -0.12)) or the episode's pedestal
column (a cylinder of radius 0.08 m from the floor to the base) meets the appliance. The spawn spot and the plate's plank
presentation follow the base along the counter's front edge: the spot keeps its x offset from the base (posture_v1: spot x 0.20
at base x 0.45) and the counter's y (-0.12, 50 mm behind the -0.17 edge); the plank, its lip and the plate move by the same
dx. The base-frame offset cannot be kept, because a base moved back would put the spot off the counter. The episode's home
TCP stays where it is.
One mount per worker process, its FCL world built in that worker. Each site runs frigidaire_robot_reach.sample_gate_job,
the full sample gate (the pick at the spot, the carry under the rules, the descent at the site, the open-jaw release), on
the hardware only, with today's holds: the load's move (bowls the axial foot ring F-out, D-ring / D-ring-build; cups the
mouth-down side pinch, D-side-down / -build; plates the wall pinch on the plank, I4-wall20; a fifteen load's guarded insertion
D-ring-insert / D-side-insert / D-ring-side-insert as the gate checks it). A bowl whose axial hold fails then tries the side
ring hold (D-ring-side, D-ring-side-insert for an insert move with its build release kept: the wrist toward the base, then
toward the mouth; a row already on the side ring hold tries both wrist sides). The order and the early exits:
the home IK (no IK, or a collision with either rack out, ends the mount because the episode cannot start). Then the plates
to add (plate_02, plate_01); when neither passes, the mount cannot count. A plate to add that reaches G1 but whose leg 1 finds
no path in 45 s at every start gets collide.rrt_connect's one narrow-passage retry (NARROW_RETRY_S, rooted at every passing
descent start; reach.i4_carry, its budget and seconds recorded) before the leg is lost. Then the kept sites, plates, cups and
bowls, and the first kept site lost ends the mount. Then any other site to add (bowl_04 when --keep is the rerun's gate alone).
--full turns the last two exits off. Then the chain pairs (PAIRS: the s0 dag's access edge plate_02 -> plate_05 records the
build's vertical insertion, not the arm's tine-axis descent): at a mount that adds plate_02, plate_05 gated with plate_02 at its
goal pose in the scene and plate_02 with plate_05 at its goal pose in the scene (the narrow retry on both); pair_ok = either
order passes with its plates' seats at their goals (the gate's D3 rule, release.seat_at_goal: within 20 mm along the tine axis on
LowerRack wires alone, so plate_02 perched on plate_05 never counts; a plate placed by the gate's D2 tine-guided release, release.mode
"drop", counts), and a mount whose pair fails keeps plate_02 but loses plate_05 ("placeable" one less). --no-basket: no
SilverwareBasket in the FCL worlds (frigidaire_robot_reach.use_basket(False) before the pool forks; rows and file "basket": false).
--plate-spins tries the plates' wall pinch at other spins from the rim point highest (default: the presentation's 0).
A candidate keeps every site reached today (--keep: the G2 rows of the s0 rerun's gate.json, 12, and of the ring_side round's,
which adds bowl_04 by the side ring hold, placed in the grip-force round; 13) and adds plate_02 or plate_01. The
winner has the most sites placeable counting the pairs, then moves least from posture_v1 (the base distance, then the yaw).
It is written to --out (default posture_v2.json; posture_v3.json when --out names it; in posture_v1.json's shape, so the
episode's --mount reads its best), with the per-site tables (<out>.rows.jsonl), the spot and the moved presentation
(<out stem>_presentation.json) beside it, but an existing --out file is replaced only by a strictly better best (rank_key: the
most placeable, then the smaller move, then the smaller yaw change; the old best re-scored by this sweep's row of the same
mount when it ran it), else <out stem>_stays.json records the verdict and the new best. Without a candidate,
posture_v1_stays.json beside --out records the verdict with the nearest miss's table. Rows stream to <out>.rows.partial.jsonl
and move beside the file written. At --cap-min the best finished mount wins and the unfinished ones are marked skipped.
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
sys.path[:0] = [str(ROOT / "code/sampler/robot/scripts"), str(ROOT / "code"), str(ROOT / "code/util")]
import frigidaire_robot_reach as R              # noqa: E402  the sample gate (sample_gate_job, gate_world, the spot)

S0_ID = "frigidaire_fdpc4221as__hotec_wheatstraw__plate5-bowl5-cup5_a3__s0"
FINAL = ROOT / "data/results/robot/samples_v1" / f"{S0_ID}__final"
RING_SIDE = ROOT / "data/results/robot/samples_v1" / f"{S0_ID}__ring_side"
V1 = ROOT / "data/results/robot/mount/posture_v1.json"
OUT = ROOT / "data/results/robot/mount/posture_v2.json"
XS, YS, ZS = (.36, .45, .60, .75, .90), (-.70, -.80, -.90, -1.00), (.45, .60, .75)
FORBIDDEN = {"x": (-.36, .36), "y": (-.95, -.12)}   # the open door / extended racks footprint (frigidaire_robot_mount.py)
PEDESTAL_R_M = .08                                   # the episode's pedestal collider (a cylinder, the floor to the base)
KIND_RANK = {"plate": 0, "cup": 1, "bowl": 2}        # the kept sites' order: the cheap checks first
CAP_MIN = 90.
PAIRS = (("plate_02", "plate_05"),)                  # the chain pairs gated both ways at a mount that adds the first: the s0
                                                     # dag's access edge plate_02 -> plate_05 (the build's vertical insertion)
RULE = ("candidate = an eligible mount (its base outside the door and racks footprint, the pedestal column clear of the "
        "appliance, a collision-free home IK with either rack out) whose full sample gate (frigidaire_robot_reach."
        "sample_gate_job, hardware only, today's holds; a bowl's side ring hold where its axial hold fails; a plate to add "
        "with collide.rrt_connect's narrow-passage retry on leg 1) passes every kept site and plate_02 or plate_01; "
        "placeable = the sites passed, less the second dish of every chain pair (PAIRS) that fails both ways (plate_05 gated "
        "with plate_02 at its goal in the scene, plate_02 with plate_05 at its goal in the scene; a pass counts with its seat at "
        "its goal, the gate's D3 rule); winner = the most "
        "placeable, then the smallest base move from posture_v1, then the smallest yaw change; an existing --out file is "
        "replaced only by a strictly better winner")


def short(s, n=300):
    return None if s is None else str(s)[:n]


def mounts_of(v1, only=None):
    """The grid (or ``only``: [(x, y, z)]) as mount dicts: yaw facing posture_v1's work centre, the spot along the counter's edge,
    the move from posture_v1, in the visit order (the smallest move first); forbidden ones kept with their status."""
    cx, cy = v1["work_centre"]
    b1, y1 = v1["best"]["base"], v1["best"]["yaw_rad"]
    pts = only or [(x, y, z) for x in XS for y in YS for z in ZS]
    out = []
    for x, y, z in pts:
        yaw = round(math.atan2(cy - y, cx - x), 4)
        mv = float(np.linalg.norm(np.subtract((x, y, z), b1)))
        out.append({"base": [x, y, z], "yaw_rad": yaw, "spot_xy": [round(R.COUNTER_XY[0] + (x - b1[0]), 4), R.COUNTER_XY[1]],
                    "move_m": round(mv, 4), "dyaw_rad": round(abs(yaw - y1), 4),
                    "baseline": bool(np.allclose((x, y, z), b1) and abs(yaw - y1) < 1e-4),
                    "forbidden": FORBIDDEN["x"][0] < x < FORBIDDEN["x"][1] and FORBIDDEN["y"][0] < y < FORBIDDEN["y"][1]})
    out.sort(key=lambda m: (m["move_m"], m["dyaw_rad"], m["base"]))
    for i, m in enumerate(out):
        m["visit"] = i
    return out


def site_plan(ds, keep):
    """[(name, role)]: the plates to add ("add-plate"), the kept sites by kind (plates, cups, bowls) and load order ("keep"),
    the other sites to add ("add")."""
    adds = [d for d in ds if d["name"] not in keep]
    key = lambda d: d["order_index"]
    return ([(d["name"], "add-plate") for d in sorted(adds, key=key) if d["kind"] == "plate"]
            + [(d["name"], "keep") for d in sorted([d for d in ds if d["name"] in keep], key=lambda d: (KIND_RANK[d["kind"]], key(d)))]
            + [(d["name"], "add") for d in sorted(adds, key=key) if d["kind"] != "plate"])


def pedestal_clear(W, m):
    """The episode's pedestal column (radius PEDESTAL_R_M, the floor to the base) against the appliance with either rack out:
    (smallest FCL distance m, the component; inf = nothing within 50 mm)."""
    import frigidaire_robot_episode as EP
    bx, by, bz = m["base"]
    best = (np.inf, None)
    for arm in W.arms.values():
        o = arm.fcl.CollisionObject(arm.fcl.Cylinder(PEDESTAL_R_M, bz), arm.fcl.Transform(np.eye(3), np.array([bx, by, bz / 2])))
        lo, hi = np.array([bx - PEDESTAL_R_M, by - PEDESTAL_R_M, 0.]), np.array([bx + PEDESTAL_R_M, by + PEDESTAL_R_M, bz])
        d, _, tn = EP.min_clear(arm.fcl, [("Pedestal", o, lo, hi)], list((arm.checker.components or {}).items()), np, cutoff=.05)
        if d < best[0]:
            best = (float(d), tn)
    return best


def detail(r):
    """The compact record of one sample_gate_job row: the pick, the release, the carry, the open-jaw check."""
    pk, rl, tr = r.get("pick") or {}, r.get("release") or {}, r.get("transit") or {}
    out = {"pick": {k: pk.get(k) for k in ("pass", "of", "yaws", "rolls", "best_yaw_k", "best_roll", "roll") if k in pk},
           "release": {k: rl.get(k) for k in ("G0", "G1", "candidates", "best", "pinch_spin_deg", "roll_deg", "seat_mm",
                                              "seat_at_goal", "mode", "release_mm", "guided_mm", "first_violation") if k in rl}}
    if not pk.get("ok", True) or not pk.get("pass", 1):
        out["pick"]["why"] = short(pk.get("why"), 400)
    if not rl.get("G1"):
        out["release"].update(why=short(rl.get("why"), 400), counts=rl.get("counts"), first_fail=short(rl.get("first_fail")),
                              start_blockers=rl.get("start_blockers"), descent_blockers=rl.get("descent_blockers"))
    if tr:
        out["carry"] = {"ok": tr.get("ok"), "legs": [l_.get("path") for l_ in tr.get("legs") or []], "why": short(tr.get("why")),
                        **{k: tr.get(k) for k in ("spin_deg", "descent_start_mm", "h_s_mm", "held_clear_mm", "retry") if k in tr}}
    if r.get("carry_fails"):
        out["carry_fails"] = [{k: short(f_.get(k), 160) if k == "why" else f_.get(k) for k in ("spin_deg", "descent_start_mm", "why")}
                              for f_ in r["carry_fails"][:3]]
    if r.get("release_check") is not None:
        out["release_check_ok"] = r["release_check"].get("ok")
        if r["release_check"].get("variants") is not None:   # D4: the guarded insertion's retreat
            out["release_check_variant"] = r["release_check"].get("variant")
    if r.get("retreat_choice"):
        out["retreat_choice"] = {k: r["retreat_choice"].get(k) for k in ("checked", "clear", "first_clear_rank", "variants")}
    if r.get("start_choice"):
        out["start_choice"] = r["start_choice"]
    return out


def seat_ok(s):
    """D3 for a chain pair: the site passes and, a plate, with its seat at its goal (the gate's release.seat_at_goal; a row without
    one, an older gate's: its pass)."""
    sg = ((s.get("detail") or {}).get("release") or {}).get("seat_at_goal")
    return bool(s.get("pass")) and (sg is None or bool(sg.get("ok")))


def build_world(m, load_dir, spins=None):
    """The gate's World at mount ``m`` (frigidaire_robot_reach.gate_world, the spot already set), the plates' wall pinch at
    ``spins`` (deg; None = the presentation's own, today's 0) when given."""
    W = R.gate_world(load_dir, m)
    if spins is not None and W.pres is not None and W.pres.get("wall"):
        W.pres = dict(W.pres, wall=dict(W.pres["wall"], spins_deg=[float(v) for v in spins]))
    W.mount_, W.spins_ = m, spins
    return W


def _set_dishes(self, bodies):
    """collide.ArmWorld.set_dishes with the scene's dishes always in (frigidaire_access_order's wrapper: the episode's arm model
    holds every placed dish)."""
    self.dishes = {**self._scene, **dict(bodies)}


def set_scene(W, scene, frames):
    """The load rows ``scene`` at their goal poses (the load's frames carried to the gate's rack frames, as the gate carries a
    goal) in their rack's arm model through every set_dishes; () empties every arm's scene."""
    import types
    from sampler.robot.rig import T_of
    for arm in W.arms.values():
        if "_scene" not in vars(arm):
            arm.set_dishes = types.MethodType(_set_dishes, arm)
        arm._scene = {}
    for e in scene:
        rk, fr = e["rack"], frames[e["rack"]]
        f_ = R.frames_for(W.B, rk)[rk]
        T = (T_of(f_["position_m"], f_["quaternion_xyzw"]) @ np.linalg.inv(T_of(fr["position_m"], fr["quaternion_xyzw"]))
             @ T_of(e["goal_pose"]["pos"], e["goal_pose"]["quat_xyzw"]))
        W.arms[rk]._scene[e["name"]] = W.arms[rk].checker._body(e["kind"], W.P.pose_dict(T), e["name"])
    for arm in W.arms.values():
        arm.set_dishes({})


def gate_site(W, d, frames, load_dir, log, scene=(), retry_s=0.):
    """One site with today's holds: the load's move, and for a bowl whose axial hold fails the side ring hold (D-ring-side, or
    D-ring-side-insert with the row's build release for an insert move; each wrist side of R.WRIST_SIDES in turn until one
    passes; a row on the side ring hold already: both sides). ``scene``: load rows at their goal poses in the arm model (the
    chain pairs); ``retry_s`` > 0: a plate's narrow-passage retry of leg 1 (reach.i4_carry). A failed check (an exception) counts
    as a fail and rebuilds the World."""
    d0 = {k: v for k, v in d.items() if k != "reject"}
    if retry_s > 0. and d["kind"] == "plate":
        d0["_retry_s"] = float(retry_s)
    tries_in = [d0]
    if d["kind"] == "bowl" and d0.get("move") in R.SIDE_MOVES:
        tries_in = [dict(d0, _roll=w) for w in R.WRIST_SIDES]             # as sample_gate_main: both wrist sides
    elif d["kind"] == "bowl" and d0.get("move") in R.INSERT_MOVES:       # the guarded insertion's side ring hold at its site
        tries_in += [dict(d0, move="D-ring-side-insert", _roll=w) for w in R.WRIST_SIDES]
    elif d["kind"] == "bowl":
        side = {k: v for k, v in d0.items() if k != "build_release"}      # the ring_side load's row: the goal pose, no build
        tries_in += [dict(side, move="D-ring-side", _roll=w) for w in R.WRIST_SIDES]
    tries = []
    with_ = f" with {','.join(e['name'] for e in scene)}" if scene else ""
    for t_in in tries_in:
        t0 = time.monotonic()
        try:
            set_scene(R._W["w"], scene, frames)
            r = R.sample_gate_job((t_in, frames, load_dir))
        except Exception as e:                          # noqa: BLE001
            r = {"move": t_in.get("move"), "gate": "error", "pass": False, "why": f"error: {e!r}", "blocker": [], "pick": {},
                 "seconds": round(time.monotonic() - t0, 1)}
            R._W["w"] = W = build_world(W.mount_, load_dir, W.spins_)   # a clean World for the next check
        r["_roll"] = t_in.get("_roll")
        tries.append(r)
        log(f"{d['name']}{with_} {t_in.get('move')}{' ' + t_in['_roll'] if t_in.get('_roll') else ''}: {r.get('gate')} "
            f"{r.get('seconds')} s{'' if r.get('pass') else ' ' + str(short(r.get('why'), 160))}")
        if r.get("pass"):
            break
    set_scene(R._W["w"], (), frames)
    best = next((t for t in tries if t.get("pass")), None) or max(tries, key=lambda t: R.GATE_RANK.get(t.get("gate"), 0))
    return {"slot": d["slot"], "kind": d["kind"], "rack": d["rack"], "move": best.get("move"), "roll": best.get("_roll"),
            "gate": best.get("gate"), "pass": bool(best.get("pass")), "why": short(best.get("why")),
            "blocker": list(best.get("blocker") or [])[:8], "seconds": round(sum(t.get("seconds") or 0. for t in tries), 1),
            "detail": detail(best),
            "tries": [{"move": t.get("move"), "roll": t.get("_roll"), "gate": t.get("gate"), "why": short(t.get("why"), 200),
                       "blocker": list(t.get("blocker") or [])[:6], "seconds": t.get("seconds")} for t in tries]}


def mount_job(args):
    """One mount, in its own worker process: the spot, the gate's World at this mount (built here), the eligibility checks,
    then the sites in site_plan's order with the early exits (none with ``full`` but the home's)."""
    m, ds, frames, keep, load_dir, full, spins = args
    t0 = time.monotonic()
    tag = f"mount {m['visit']:2d} {m['base']} yaw {m['yaw_rad']:.4f}"

    def log(msg):
        print(f"[INFO] {tag}: {msg} ({time.monotonic() - t0:.0f} s)", flush=True)
    row = {k: m[k] for k in ("base", "yaw_rad", "spot_xy", "move_m", "dyaw_rad", "baseline", "visit")}
    row.update(status="done", eligible=False, stopped=None, sites={}, order=[], basket=bool(R.BASKET))
    if m["forbidden"]:
        row["stopped"] = "the base in the open door and racks footprint"
        return finish(row, t0, keep)
    R.COUNTER_XY = tuple(m["spot_xy"])                  # this worker's spot: reach reads it at call time
    W = build_world(m, load_dir, spins)                 # the FCL world of this mount (never built before the fork)
    R._W["w"] = W                                       # sample_gate_job's per-worker World
    row["sources"] = {"holds": {k: v[2] for k, v in W.holds.items()},
                      "hands": {"cup_down": W.cup_hulls[1], "plate_wall": W.plate_closed[1], "bowl_side": W.bowl_side[1]},
                      "presentation": None if W.pres is None else {**{k: W.pres.get(k) for k in ("name", "spot_xy", "moved_from_xy")},
                                                                   "spins_deg": (W.pres.get("wall") or {}).get("spins_deg")}}
    d_p, c_p = pedestal_clear(W, m)
    row["pedestal_clear_mm"] = None if not np.isfinite(d_p) else round(d_p * 1e3, 1)
    if d_p <= 0.:
        row["stopped"] = f"the pedestal column inside the {c_p} ({d_p * 1e3:.1f} mm)"
        log(row["stopped"])
        return finish(row, t0, keep)
    q_home = R.home_q(W)
    hits = {} if q_home is None else {rk: R.pairs_of(a_, q_home)[:4] for rk, a_ in W.arms.items() if not a_.free(q_home)}
    row["home"] = {"ik": q_home is not None, "collides": hits,
                   "deg": None if q_home is None else [round(float(v), 1) for v in np.degrees(q_home)]}
    if q_home is None or hits:
        row["stopped"] = ("home: no elbow-up IK at the episode's home TCP" if q_home is None else
                          f"home: the arm collides with the rack out {hits}")
        log(row["stopped"])
        return finish(row, t0, keep)
    row["eligible"] = True
    from sampler.robot import collide as C
    by = {d["name"]: d for d in ds}
    plan = site_plan(ds, keep)
    add_plates = [n for n, role in plan if role == "add-plate"]
    for name, role in plan:                             # a plate to add: the narrow-passage retry of leg 1
        s = gate_site(W, by[name], frames, load_dir, log, retry_s=C.NARROW_RETRY_S if role == "add-plate" else 0.)
        W = R._W["w"]
        row["sites"][name] = s
        row["order"].append(name)
        if full:
            continue
        if role == "keep" and not s["pass"]:
            row["stopped"] = f"lost {name} ({s['gate']}: {short(s['why'], 160)})"
            break
        if role == "add-plate" and name == add_plates[-1] and not any(row["sites"][n]["pass"] for n in add_plates):
            row["stopped"] = "adds no plate: the mount cannot count"
            break
    if row["stopped"]:
        log(row["stopped"])
    row["pairs"] = {}
    for x, y in PAIRS:                                  # the chain pairs at a mount that adds x (x passes alone)
        if x not in by or y not in by or x in keep or not (row["sites"].get(x) or {}).get("pass"):
            continue
        key = f"{x}|{y}"
        if row["stopped"]:
            row["pairs"][key] = {"checked": False, "why": f"the mount stopped: {short(row['stopped'], 160)}"}
            continue
        alone = {n: seat_ok(row["sites"].get(n) or {}) for n in (x, y)}
        orders = {}
        for a_, b_ in ((x, y), (y, x)):                # a_ placed (at its goal pose), then b_ gated with a_ in the scene
            s = gate_site(W, by[b_], frames, load_dir, log, scene=(by[a_],), retry_s=C.NARROW_RETRY_S)
            W = R._W["w"]
            orders[f"{a_} then {b_}"] = {k: s[k] for k in ("gate", "pass", "why", "blocker", "seconds", "detail")}
            orders[f"{a_} then {b_}"]["seat_at_goal"] = seat_ok(s)   # D3: a perch on the first dish never counts
        ok = bool((alone[x] and orders[f"{x} then {y}"]["seat_at_goal"]) or (alone[y] and orders[f"{y} then {x}"]["seat_at_goal"]))
        row["pairs"][key] = {"checked": True, "alone": alone, "orders": orders, "pair_ok": ok,
                             "lost": None if ok or not all(alone.values()) else y}
        log(f"pair {key}: {'ok' if ok else 'fails both ways'} "
            f"({', '.join(f'{o}: {v_['gate']}' for o, v_ in orders.items())})")
    return finish(row, t0, keep)


def finish(row, t0, keep):
    sites = row["sites"]
    adds = [n for n in sites if n not in keep]
    row["kept"] = [n for n in keep if (sites.get(n) or {}).get("pass")]
    row["lost"] = [n for n in keep if n in sites and not sites[n]["pass"]]
    row["added"] = [n for n in adds if sites[n]["pass"]]
    row["score"] = len(row["kept"]) + len(row["added"])
    row["chain_lost"] = [p["lost"] for p in (row.get("pairs") or {}).values() if p.get("lost")]
    row["placeable"] = row["score"] - len(row["chain_lost"])   # a failed pair keeps its first dish and loses the second
    row["seconds"] = round(time.monotonic() - t0, 1)
    return row


def rank_key(r, candidate=True):
    """A mount's standing, larger is better: a candidate, the most sites placeable counting the chain pairs (a file without
    "placeable": its score), the smallest base move from posture_v1, the smallest yaw change."""
    p = r.get("placeable", r.get("score"))
    return (bool(candidate), -1 if p is None else int(p), -float(r.get("move_m") or 0.), -float(r.get("dyaw_rad") or 0.))


def prior_best(path, rows):
    """The existing --out file's best and its standing (rank_key): this sweep's own row of the same base and yaw when it ran
    it (the old best re-scored by today's rules), else as the file records it; None = unreadable, or not this sweep's shape."""
    try:
        b = json.loads(Path(path).read_text()).get("best") or {}
    except (OSError, ValueError, AttributeError):
        return None
    if b.get("base") is None or b.get("yaw_rad") is None:
        return None
    same = next((r for r in rows if r.get("status") == "done" and np.allclose(r["base"], b["base"], atol=1e-6)
                 and abs(float(r["yaw_rad"]) - float(b["yaw_rad"])) < 1e-4), None)
    if same is not None:
        return {"best": b, "key": rank_key(same, same.get("candidate")), "rescored": True, "visit": same["visit"],
                "placeable_now": same.get("placeable"), "candidate_now": bool(same.get("candidate"))}
    if b.get("move_m") is None or b.get("dyaw_rad") is None:
        return None
    return {"best": b, "key": rank_key(b), "rescored": False}


def judge(rows, keep, add_plates, names):
    """Flags each row (candidate, the sites left unchecked); the winner (or None) and the nearest miss."""
    for r in rows:
        r["unchecked"] = [n for n in names if n not in (r.get("sites") or {})] if r["status"] == "done" else None
        r["candidate"] = bool(r["status"] == "done" and r.get("eligible") and len(r.get("kept") or []) == len(keep)
                              and any(n in (r.get("added") or []) for n in add_plates))
    cands = [r for r in rows if r["candidate"]]
    win = min(cands, key=lambda r: (-r["placeable"], r["move_m"], r["dyaw_rad"], r["visit"])) if cands else None
    done = [r for r in rows if r["status"] == "done" and r.get("eligible")]
    plate_rank = lambda r: max([R.GATE_RANK.get((r["sites"].get(n) or {}).get("gate"), 0) for n in add_plates] or [0])
    near = (max(done, key=lambda r: (any(n in r["added"] for n in add_plates), len(r["kept"]), len(r["added"]), plate_rank(r),
                                     -r["move_m"])) if done else None)
    return win, near


def main():
    import multiprocessing as mp
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--workers", type=int, default=6)
    ap.add_argument("--load", type=Path, default=FINAL / "load.json",
                    help="the 15 dishes with their moves and frames (its dir: presentation.json and the Isaac gates' hands)")
    ap.add_argument("--keep", type=Path, nargs="+", default=[FINAL / "gate.json", RING_SIDE / "gate.json"],
                    help="the sites reached today: the rows with pass true in any of these gate.json files (default the s0 "
                         "rerun's and the ring_side round's: 13; the rerun's alone: its 12)")
    ap.add_argument("--v1", type=Path, default=V1, help="posture_v1.json: the work centre, the move's origin")
    ap.add_argument("--mounts", nargs="*", default=None, help="only these bases, each x,y,z (default the 60-mount grid)")
    ap.add_argument("--full", action="store_true", help="no early exit but the home's: every site at every eligible mount")
    ap.add_argument("--cap-min", type=float, default=CAP_MIN, help="at the cap the best finished mount wins (default 90)")
    ap.add_argument("--plate-spins", default=None,
                    help="the plates' wall pinch at these spins from the rim point highest, deg, comma-separated (default the "
                         "presentation's own: 0, today's hold); every plate tries them, the first passing pick and seat wins")
    ap.add_argument("--out", type=Path, default=OUT,
                    help="the winner's mount file (default posture_v2.json; e.g. posture_v3.json): an existing one is replaced only "
                         "by a strictly better best, else <stem>_stays.json beside it; posture_v1_stays.json without a candidate")
    ap.add_argument("--no-basket", action="store_true",
                    help="no SilverwareBasket in the FCL worlds (frigidaire_robot_reach.use_basket(False), the user's basket-out "
                         "decision of plans/2026-11-04-fifteen-of-fifteen.md); rows and file record \"basket\": false")
    a = ap.parse_args()
    if a.no_basket:                                     # before the pool forks: every mount's World inherits it
        R.use_basket(False)
    a.load, a.v1, a.out = (p if p.is_absolute() else ROOT / p for p in (a.load, a.v1, a.out))
    a.keep = [p if p.is_absolute() else ROOT / p for p in a.keep]
    t_start = time.monotonic()
    load = json.loads(a.load.read_text())
    ds = sorted(load["dishes"], key=lambda x: x["order_index"])
    gates = [json.loads(p.read_text())["dishes"] for p in a.keep]
    keep = [d["name"] for d in ds if any((g.get(d["name"]) or {}).get("pass") for g in gates)]
    add_plates = [n for n, role in site_plan(ds, keep) if role == "add-plate"]
    v1 = json.loads(a.v1.read_text())
    only = [tuple(float(v) for v in s.split(",")) for s in a.mounts] if a.mounts else None
    ms = mounts_of(v1, only)
    rows_f = a.out.with_suffix(".rows.partial.jsonl")   # streamed here, moved beside the file written at the end (an existing
    rows_f.parent.mkdir(parents=True, exist_ok=True)    # <out>.rows.jsonl stays unless its mount file is replaced)
    rows_f.write_text("")
    print(f"[INFO] mount sweep: {len(ms)} mounts, {len(ds)} sites (keep {len(keep)}: {keep}; add {[d['name'] for d in ds if d['name'] not in keep]}), "
          f"{a.workers} workers, cap {a.cap_min:.0f} min{', full' if a.full else ''}{', no basket' if not R.BASKET else ''} -> "
          f"{R.rel_path(a.out)}", flush=True)

    def done_(r):
        with open(rows_f, "a") as f:
            f.write(json.dumps(r, default=str) + "\n")
        print(f"[INFO] mount {r['visit']:2d} {r['base']} yaw {r['yaw_rad']:.4f} spot {r['spot_xy']}: score {r['score']} "
              f"placeable {r.get('placeable')} (kept {len(r['kept'])}/{len(keep)}, added {r['added']}, lost {r['lost']}, "
              f"pairs { {k_: v_.get('pair_ok') for k_, v_ in (r.get('pairs') or {}).items()} }) {r['stopped'] or 'all checked'}, "
              f"{r['seconds']} s, {time.monotonic() - t_start:.0f} s in", flush=True)

    def err_(e):
        print(f"[ERROR] mount job: {e!r}", flush=True)
    spins = None if a.plate_spins is None else [float(v) for v in a.plate_spins.split(",") if v.strip()]
    jobs = [(m, ds, load["frames"], keep, a.load.parent, a.full, spins) for m in ms]
    pool = mp.get_context("fork").Pool(a.workers, maxtasksperchild=1)   # a fresh process (and World) per mount
    res = [pool.apply_async(mount_job, (j,), callback=done_, error_callback=err_) for j in jobs]
    while time.monotonic() - t_start < a.cap_min * 60. and not all(r.ready() for r in res):
        time.sleep(5)
    rows = []
    for m, r in zip(ms, res):
        keys = ("base", "yaw_rad", "spot_xy", "move_m", "dyaw_rad", "baseline", "visit")
        if not r.ready():
            rows.append({**{k: m[k] for k in keys}, "status": f"skipped ({a.cap_min:.0f} min cap)", "sites": {}, "kept": [],
                         "lost": [], "added": [], "score": None})
            continue
        try:
            rows.append(r.get())
        except Exception as e:                          # noqa: BLE001
            rows.append({**{k: m[k] for k in keys}, "status": f"error: {e!r}", "sites": {}, "kept": [], "lost": [], "added": [],
                         "score": None})
    pool.terminate()
    win, near = judge(rows, keep, add_plates, [d["name"] for d in ds])
    import frigidaire_robot_episode as EP
    src = next((r["sources"] for r in rows if r.get("sources")), {})
    for r in rows:                                      # posture_v1's table shape: the winner's row picked
        r.pop("sources", None)
        r["picked"] = r is win
    pf = a.load.parent / "presentation.json"
    base = next((r for r in rows if r.get("baseline")), None)
    res = {"plan": "plans/2026-11-04-fifteen-of-fifteen.md", "rule": RULE, "work_centre": v1["work_centre"],
           "grid": {"x": list(XS), "y": list(YS), "z": list(ZS), "yaw": "facing posture_v1's work centre",
                    "forbidden": FORBIDDEN, "pedestal_r_m": PEDESTAL_R_M, "mounts": None if only is None else [list(p) for p in only],
                    "pedestal_note": "pedestal_clear_mm: the column's FCL distance to the appliance (None: nothing within 50 mm)"},
           "baseline": None if base is None else {k: base.get(k) for k in ("base", "yaw_rad", "status", "score", "placeable",
                                                                           "kept", "added", "lost", "stopped")},
           "counter_xy": None, "counter_front_y": R.COUNTER_FRONT_Y, "basket": bool(R.BASKET),
           "holds": src.get("holds"), "hands": src.get("hands"),
           "heights_m": list(EP.DESCENT_STARTS_M), "cap_s": a.cap_min * 60., "workers": a.workers, "full": bool(a.full),
           "plate_spins_deg": spins if spins is not None else (src.get("presentation") or {}).get("spins_deg"),
           "load": R.rel_path(a.load), "keep_source": [R.rel_path(p) for p in a.keep], "keep": keep,
           "adds": [d["name"] for d in ds if d["name"] not in keep], "add_plates": add_plates,
           "order_rule": "per mount: the pedestal and the home IK; the plates to add (neither passes: the mount cannot count); "
                         "the kept plates, cups and bowls (a bowl's side ring hold where its axial hold fails; the first kept site "
                         "lost ends the mount); the other sites to add" + (" (--full: no early exit but the home's)" if a.full else "")
                         + "; then the chain pairs of a mount that adds plate_02 (none once the mount stopped)",
           "retry_rule": f"a plate to add (and either dish of a chain pair) whose every passing descent start's leg 1 finds no "
                         f"RRT-Connect path in 45 s: one collide.rrt_connect retry of NARROW_RETRY_S s rooted at every such start "
                         f"whose line down the tine axis is clear (frigidaire_robot_reach.i4_carry; carry.retry in the site's detail)",
           "pairs_rule": {"pairs": [list(p) for p in PAIRS],
                          "rule": "at a mount that adds the first dish of a pair: the second gated with the first at its goal pose "
                                  "in the scene (the first then the second) and the first with the second at its goal pose (the "
                                  "second then the first); pair_ok = an order whose first dish passes alone and whose second "
                                  "passes with it, each with its seat at its goal (D3, the gate's release.seat_at_goal: a perch on "
                                  "the other plate never counts; the gate's D2 tine-guided drop does); a failed pair loses its "
                                  "second dish from placeable (the access edge records the build's vertical insertion, not the "
                                  "arm's tine-axis descent)"},
           "spot_rule": "the spot keeps its x offset from the base along the counter's front edge (spot x = 0.20 + base x - 0.45) "
                        "and the counter's y (-0.12); the plank presentation (the plate, the plank, its lip) moves by the same dx; "
                        "the episode's home TCP stays",
           "table": sorted(rows, key=lambda r: r["visit"]), "seconds": round(time.monotonic() - t_start, 1)}
    prior = None
    if win is None:
        out = a.out.with_name("posture_v1_stays.json")
    elif not a.out.exists():
        out = a.out
    else:                                               # never over an existing mount file unless strictly better
        prior = prior_best(a.out, rows)
        out = (a.out if prior is not None and rank_key(win) > prior["key"] else a.out.with_name(a.out.stem + "_stays.json"))

    def best_of(r):
        return {**{k: r[k] for k in ("base", "yaw_rad", "spot_xy", "score", "placeable", "move_m", "dyaw_rad", "visit", "baseline")},
                "kept": len(r["kept"]), "added": r["added"], "chain_lost": r["chain_lost"],
                "pairs": {k_: {"pair_ok": v_.get("pair_ok"), "lost": v_.get("lost"),
                               "orders": {o: v["gate"] for o, v in (v_.get("orders") or {}).items()}} if v_.get("checked") else v_
                          for k_, v_ in (r.get("pairs") or {}).items()},
                "sites_gate": {n: f"{s['gate']} {s['move']}{' ' + s['roll'] if s.get('roll') else ''}" for n, s in r["sites"].items()}}
    if win is not None and out == a.out:
        dx = round(win["spot_xy"][0] - R.PRESENTATION_XY[0], 4)
        pres_out = None
        if pf.is_file() and abs(dx) > 1e-9:
            pres_out = a.out.with_name(a.out.stem + "_presentation.json")
            pres_out.write_text(json.dumps(R.shift_presentation(json.loads(pf.read_text()), win["spot_xy"]), indent=1) + "\n")
        res.update(verdict=f"{a.out.stem}: the winner adds {win['added']}, placeable {win['placeable']}"
                           + (f" (the chain pairs lose {win['chain_lost']})" if win["chain_lost"] else ""),
                   counter_xy=win["spot_xy"], best=best_of(win), sites=win["sites"], pairs=win.get("pairs"),
                   replaced=None if prior is None else {"best": {k: prior["best"].get(k) for k in ("base", "yaw_rad", "spot_xy",
                                                                                                  "score", "placeable")},
                                                        "key": prior["key"], "rescored": prior["rescored"]},
                   spot={"xy": win["spot_xy"], "offset_m": [dx, 0.], "rule": res["spot_rule"],
                         "presentation": None if pres_out is None else R.rel_path(pres_out),
                         "presentation_rule": "the load's presentation.json moved by the spot's dx (frigidaire_robot_reach."
                                              "shift_presentation); the sample gate given --mount moves it the same way"})
    elif win is not None:                               # the existing file stays: its best is this file's best
        res.update(verdict=f"{a.out.stem} stays: the new best {win['base']} (placeable {win['placeable']}, move {win['move_m']} m) "
                           f"is not strictly better than its best"
                           + ("" if prior is None else f" {prior['best'].get('base')} (rank {list(prior['key'])}"
                                                      f"{', re-scored by this sweep' if prior['rescored'] else ', as recorded'})"),
                   counter_xy=None if prior is None else prior["best"].get("spot_xy"),
                   best=None if prior is None else {**prior["best"], "source": R.rel_path(a.out)},
                   prior=None if prior is None else {k: prior.get(k) for k in ("key", "rescored", "visit", "placeable_now",
                                                                               "candidate_now")},
                   new_best={**best_of(win), "key": list(rank_key(win))}, sites=win["sites"], pairs=win.get("pairs"),
                   stays_rule="an existing --out file is replaced only by a strictly better winner (rank_key: a candidate, the "
                              "most placeable, the smaller base move, the smaller yaw change; its best re-scored by this sweep's "
                              "row of the same mount when it ran it; an unreadable or foreign file is never replaced)")
    else:
        b1 = v1["best"]
        res.update(verdict="posture_v1 stays: no mount keeps every kept site and adds plate_02 or plate_01",
                   counter_xy=list(R.COUNTER_XY), best={"base": b1["base"], "yaw_rad": b1["yaw_rad"], "spot_xy": list(R.COUNTER_XY),
                                                        "source": R.rel_path(a.v1)},
                   nearest_miss=None if near is None else {k: near.get(k) for k in ("base", "yaw_rad", "spot_xy", "move_m", "score",
                                                                                     "placeable", "kept", "lost", "added",
                                                                                     "stopped", "sites", "pairs")})
    out.write_text(json.dumps(res, indent=1, default=lambda o: o.tolist() if hasattr(o, "tolist") else str(o)) + "\n")
    rows_out = out.with_suffix(".rows.jsonl")           # the rows beside the file written
    rows_f.replace(rows_out)
    n_skip = sum(str(r["status"]).startswith("skipped") for r in rows)
    print(f"[RESULT] PASS mount sweep {len(rows) - n_skip}/{len(rows)} mounts in {res['seconds']} s: "
          + (f"winner {win['base']} yaw {win['yaw_rad']:.4f} spot {win['spot_xy']} adds {win['added']} placeable "
             f"{win['placeable']} pairs { {k_: v_.get('pair_ok') for k_, v_ in (win.get('pairs') or {}).items()} } "
             f"(move {win['move_m']} m){'' if out == a.out else '; not strictly better: ' + a.out.name + ' stays'}"
             if win else f"posture_v1 stays; nearest miss {None if near is None else (near['base'], near['added'], near['lost'], near['stopped'])}")
          + f" -> {R.rel_path(out)}, {R.rel_path(rows_out)}", flush=True)

if __name__ == "__main__":
    main()
