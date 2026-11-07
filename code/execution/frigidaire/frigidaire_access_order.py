#!/usr/bin/env python3
"""The access order of plans/2026-11-04-fifteen-of-fifteen.md, Kit-free: per rack chain of a four-block sample (s0) a linearization
of its dag (--dag soft: an access edge may be reversed, REVERSE_RULE) in which every dish's step passes the Kit-free sample gate
(frigidaire_robot_reach.sample_gate_job: the pick at the
counter spot, the descent start and the line down, the carry, the open-jaw release) with the EARLIER dishes of its chain in the scene
at their predicted landed poses: the dish's own state file where a run placed it (the s0 rerun's, then the ring-side round's), its
goal pose otherwise.

    PYTHONPATH=code/src .venv-kitfree/bin/python code/execution/frigidaire/frigidaire_access_order.py \\
        [--sample data/results/bench/samples/<s0>.json] [--mount data/results/robot/mount/posture_v1.json] [--workers 6] \\
        [--dag soft|hard] [--narrow-retry-s 0] [--no-basket] [--out <dir>/order.json]

Choices per dish, in the order of preference: a bowl the axial foot ring (D-ring; the build's sites D-ring-insert) where it passes,
else the side ring hold (D-ring-side, at a build site D-ring-side-insert; grasp.foot_side_grasps, its starts in the episode's
--ring-side-order straight order); a cup
the side-down hold (D-side-down; cup_05 D-side-insert); a plate the wall pinch with its spin AT THE GOAL (the pinch's rim point from
the rim point highest of the goal; the pick on the plank keeps the presentation's spin), the smallest |spin| of PLATE_SPINS_DEG that
passes. The build's sites are gated with the episode's guarded-insertion moves (D-ring-insert, D-side-insert; the side ring hold
there D-ring-side-insert where the gate checks that move as the side ring hold, else D-ring-side with the build release, round 1's
check). The descent start is the gate's (straight carry legs first, then rule 1). First every dish alone (hardware only): a dish no
choice passes is a reject. Then per chain the dag order greedily: a dish that fails gets the scene dishes its failure names (else the
last placed) as learned "place before" edges and the chain runs again, until it is complete or no edge can be added without a cycle
(the best run is kept; the pair of PAIRS is reported per spin both ways). --dag soft (the default; the coordinator, 2026-11-04
18:05 EST): a learned "x before c" that conflicts with an ACCESS edge c -> x of the dag is tested by REVERSE_RULE and the edge is
reversed for this chain when c is a plate and passes with x in the scene with its seat at its goal (seat_at_goal: the gate's I4 seat
search stops at the first contact, so a bare pass can be a perch on x); a bowl's or a cup's access edge stays, a support edge is never
reversed, and the sample's dag is untouched; --dag hard keeps every dag edge (round 1). Every plate row carries seat_at_goal, and a
placed plate step seated off its goal is listed in order.json's off_goal_seats (a warning). Writes <load dir>/order.json (the chains, per dish the hold, the move, the spin, the start and
the gate verdict, the reversed edges with both gate rows, the rejects, the pair checks) and gate.json beside it (each dish's chosen
row in the sample gate's format, which the episode reads for the plates' pinch spin and the side ring hold's pick order). --out may
lie outside the repo (a scratch copy): its records then keep absolute paths.

Third round (the plan's Decisions, the user 19:20 and 20:05 EST). --no-basket: every worker's gate world without the
SilverwareBasket (the real basket lifts out; s0's goal poses and S_ref keep it): frigidaire_robot_reach.use_basket(False) in each
worker before its world is built (a reach without it: the Kit-free measurement's wrap, InitialCollisionChecker.update_components
popping the basket from the checker's components right after building them; the frames keep all five), checked on both arms;
"basket": false in order.json and gate.json. A step passes on the gate's pass and, for a plate, its seat at its goal (seat_at_goal,
SEAT_RULE: within AT_GOAL["plate"] of the goal and at most 20 mm from it along the tine axis, rack wires its only supports; a dish
among them is a perch, a failed step, so plate_02 perched on plate_05 is a conflict, never a pass), whatever the gate's release
mode: "seat" (released at the seat, I4-wall20) or "drop" (the tine-guided release: the jaws open release_mm above the seat with the
plate's rim guided_mm below the slot's tine tips, the episode's I4-wall20-drop; a spin that seats the plate goes before one that
drops it, and after the search a dropping plate is tried before the scene dish that forces its drop, DROP_PREF_RULE: the best run
places the most dishes, then drops the fewest). Every plate step carries its release mode; the open-jaw check's retreat variant
(cup_05's D-side-insert) is copied into a step's start where the gate gives one.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import multiprocessing as mp
import os
import re
import shutil
import sys
import threading
import time
import traceback
import types
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
sys.path[:0] = [str(ROOT / "code/execution/frigidaire"), str(ROOT / "code/src"), str(ROOT / "code/frigidaire/src")]
import frigidaire_placing_loads as PL  # noqa: E402  (stdlib only)
import frigidaire_robot_reach as R  # noqa: E402  (Kit-free)

SID = "frigidaire_fdpc4221as__hotec_wheatstraw__plate5-bowl5-cup5_a3__s0"
SAMPLE = ROOT / "data/results/bench/samples" / f"{SID}.json"
MOUNT = ROOT / "data/results/robot/mount/posture_v1.json"
STATE_DIRS = (PL.SAMPLES / f"{SID}__final/state", PL.SAMPLES / f"{SID}__ring_side/state")   # measured landings (the first wins)
PRESENTATION = PL.SAMPLES / f"{SID}__final/presentation.json"   # the plank (goal-plank facing 0): the plates' pick
GATES = {   # the Isaac grasp gates of record (their median held trial's closed jaws: the descent checks' hand); the load dir's run first
    "plate": PL.SAMPLES / f"{SID}__final/partA_plate_wall20",                 # 8 / 400 / 1 on the plank, held 10 of 10
    "cup": PL.SAMPLES / f"{SID}__cups_down/partA_cup_side_down11_gf",          # 8 / 4 / 2.5, held 10 of 10 at 20.4 N
    "bowl_side": PL.SAMPLES / f"{SID}__ring_side/partA_bowl_foot_side_gf"}     # 8 / 4 / 2.5, held 10 of 10 at 22.3 to 23.2 N
SIDE_RANK = PL.SAMPLES / f"{SID}__ring_side/foot_side_candidates.json"   # the side ring hold's (yaw, wrist side) order
PLATE_SPINS_DEG = (0., 15., -15., 30., -30., 45., -45., 60., -60., 75., -75., 90., -90.)   # the pinch at the goal, 0 = the gated
INSERT_SITES = {n: m for n, m in PL.FINAL_MOVES.items() if m.endswith("-build")}   # the build's sites (bowl_02, bowl_03, cup_05)
PAIRS = (("plate_03", "plate_05"),)       # the s0 rerun's pair: plate_05 at its goal blocks plate_03's descent, plate_03 as it landed
                                          # blocks plate_05's line (results/2026-11-04-s0-rerun.md): both orders reported per spin
MAX_RUNS = 12                             # the greedy runs per chain (each adds at least one learned edge)
REVERSE_RULE = ("--dag soft: an access edge c -> x of the sample's dag (x at its goal covers c's build insertion path, the build's "
                "vertical insertion) is reversed for the arm's chain when x fails the gate with c in the scene, c is a plate (the "
                "arm lowers it along the tine axis, a different sweep) and c, gated at its own scene of that run with x added at "
                "its predicted pose (every pinch spin of c), passes WITH ITS SEAT AT ITS GOAL (seat_at_goal: the gate's I4 seat "
                "search stops the plate at its first contact, so a bare pass can be a perch on x): x goes before c (the learned "
                "edge x -> c); else the conflict stays. A bowl's or a cup's access edge stays (its descent is along the rack's up "
                "axis, the path the edge records, and the gate's line down does not sweep the held dish against the scene); a "
                "support edge (any other kind) is never reversed; the sample's dag is untouched")
REVERSIBLE_KINDS = ("plate",)             # the dishes whose descent is not the build's vertical insertion (the I4 tine axis)
DROP_PREF_RULE = ("the tine-guided drop relaxes the supported release only where the order cannot avoid it (the user's plate pair, "
                  "2026-11-04 20:05 EST): after the search, a placed plate that drops because a scene dish c stops its seat descent "
                  "(c named by its drop's first violation or descent blockers) is tried before c (the preference edge x -> c, never "
                  "against the dag or a learned edge; each edge once), one edge per run; a run is kept when it places more dishes, or "
                  "as many with fewer drops")
DISH_RE = re.compile(r"\b(?:plate|bowl|cup)_\d{2}\b")
INIT_DISH = {"name": "_init", "kind": "bowl", "rack": "UpperRack", "slot": "-", "order_index": -1, "move": None,
             "reject": {"cause": "the access order's world init (no check)"}}
CFG = {}                                  # set in the parent before the pool forks; the workers inherit it
BASKET = "SilverwareBasket"               # --no-basket: the component the robot rounds leave out (the user, 2026-11-04 19:20 EST)
SEAT_RULE = ("a plate's seat (the gate's seat search: the plate alone lowered along the tine axis from its goal moved SEAT_DX_M "
             "toward the basket, its first contact; the landing of the tine-guided drop too) counts only within AT_GOAL['plate'] "
             "of the goal (lateral, dz), at most AT_GOAL's 20 mm from it along the tine axis and with rack wires (the LowerRack's "
             "wire groups) its only supports; a dish among them is a perch: the step fails (the user, 2026-11-04 20:05 EST)")
_S = {}                                   # per worker: the gate's world
# plans/2026-11-05-hardware-aware-sampler.md (--candidate, --direct-only, --gate-one): the hardware-aware sampler's gate
DIRECT_MOVES = ("D-ring", "D-ring-side", "D-side-down", "I4-wall20")   # the four direct releases (the user's Decisions)
DIRECT_SPINS_DEG = (0., 15., -15., 30., -30.)                         # the wall pinch's spins at the goal under --direct-only
CUP5 = {"side_below_top_m": .0999, "hold_gate_run": "partA_cup_side_down5"}   # round 3's 5 mm cup hold (cup_05), a second cup choice
FIFTEEN_DIR = PL.SAMPLES / f"{SID}__fifteen"                          # the gates of record at posture_v2 (partA_*, presentation.json)
FIFTEEN_R3 = PL.SAMPLES / f"{SID}__fifteen_r3"                        # partA_cup_side_down5
DIRECT_RULE = ("direct-only (plans/2026-11-05-hardware-aware-sampler.md): every dish gated in the dag order with the earlier dishes "
               "of its rack in the scene at their candidate poses (no insert sites, no state dirs, the basket in), over its direct "
               "choices only (bowls D-ring then D-ring-side, cups D-side-down at the 11 mm then the 5 mm hold, plates I4-wall20 at "
               "the spins 0, +-15, +-30 with the tine-guided release off); direct_ok = the gate's pass, a move of DIRECT_MOVES, no "
               "build release, no stop mode, a plate released at its seat (release.mode seat) with seat_at_goal ok, and the "
               "open-jaw release check ok where the gate computes one (D-ring-side, D-side-down; D-ring and I4-wall20 judge their "
               "release inside ring_release / i4_release)")


def _gates_side_insert():
    """True when the gate checks D-ring-side-insert as the side ring hold (frigidaire_robot_reach.merge_rolls merges the move's two
    wrist sides, as it does D-ring-side's); round 1's gate does not."""
    r_ = {"dish": "_", "move": "D-ring-side-insert", "gate": "none", "pass": False, "pick": {"roll": "base", "yaws": []}}
    try:
        return len(R.merge_rolls([r_, dict(r_, pick={"roll": "mouth", "yaws": []})])) == 1
    except Exception:                                    # noqa: BLE001  (an older gate: round 1's check)
        return False


SIDE_INSERT_MOVE = "D-ring-side-insert" if _gates_side_insert() else "D-ring-side"   # the side ring hold's gate move at a build
                                          # site (D-ring-side: with the dish row's build release; the episode runs D-ring-side-insert)


def kind_of(n):
    return n.split("_")[0]


def choices(n):
    """[(key, hold, the gate's move, the plate's spin at the goal)] of a dish in the order of preference (a build site's moves:
    the episode's guarded insertion, PL.fifteen_move; the side ring hold there SIDE_INSERT_MOVE)."""
    if CFG.get("direct"):                                # --direct-only: the four direct releases, no insertion or drop
        if kind_of(n) == "bowl":
            return [("foot-out", "foot-out", "D-ring", None), ("foot-side", "foot-side", "D-ring-side", None)]
        if kind_of(n) == "cup":
            return [("side-down", "side-down", "D-side-down", None), ("side-down5", "side-down", "D-side-down", None)]
        return [(f"plate-wall@{s:+.0f}", "plate-wall", "I4-wall20", s) for s in DIRECT_SPINS_DEG]
    site = INSERT_SITES.get(n)
    if kind_of(n) == "bowl":
        return [("foot-out", "foot-out", PL.fifteen_move(site, True) if site else "D-ring", None),
                ("foot-side", "foot-side", SIDE_INSERT_MOVE if site else "D-ring-side", None)]
    if kind_of(n) == "cup":
        return [("side-down", "side-down", PL.fifteen_move(site, True) if site else "D-side-down", None)]
    return [(f"plate-wall@{s:+.0f}", "plate-wall", "I4-wall20", s) for s in PLATE_SPINS_DEG]


def choice(n, key):
    return next(c for c in choices(n) if c[0] == key)


def release_of(row):
    """A plate row's release from the gate row's release block (frigidaire_robot_reach.i4_release, D2): {"mode": "seat" (released
    at its seat, release_mm 0) or "drop" (the tine-guided release) or None (no release: not tine-guided, or no seat), "release_mm"
    (above the seat along the tine axis, where the jaws open), "guided_mm" (the plate's lowest rim point there below its slot's
    tine tips)}; a gate without modes: "seat" for a passing row, else None."""
    rel_ = row.get("release") or {}
    mode = rel_["mode"] if "mode" in rel_ else ("seat" if row.get("pass") else None)
    return {"mode": mode, "release_mm": rel_.get("release_mm"), "guided_mm": rel_.get("guided_mm")}


def ok_row(n, row):
    """A step's verdict: the gate's pass and, for a plate, its seat at its goal under SEAT_RULE (seat_at_goal), whatever the
    release mode (the tine-guided drop counts as a pass: the plate lands on that seat)."""
    return bool(row.get("pass")) and (kind_of(n) != "plate" or bool((row.get("seat_at_goal") or {}).get("ok")))


# ------------------------------------------------------------------ the worker side (the gate's world, one per process)
_EDGE_PICK = R.edge_pick


def _straight_order(cands, straight=()):
    """frigidaire_robot_episode.pitch_order as --ring-side-order straight orders D-ring-side's starts: the straight-leg ones first,
    then rule 1's order (ring_release ranks the starts by rule 1 before it calls this)."""
    s_ = [int(i) for i in straight]
    return s_ + [i for i in range(len(cands)) if i not in set(s_)]


def _set_dishes(self, bodies):
    """collide.ArmWorld.set_dishes with the chain's earlier dishes always in (the episode's arm model holds every dish)."""
    self.dishes = {**self._scene, **dict(bodies)}


def seat_at_goal(W, row):
    """A plate row's I4 seat under SEAT_RULE (the gate's seat search lowers the plate alone along the tine axis from its goal
    moved SEAT_DX_M toward the basket and stops at the first contact; the tine-guided drop lands there too): the seat's offset
    from the goal (lateral, |dz|; no tilt, the search translates) within frigidaire_bench.AT_GOAL["plate"], |seat_mm| along the
    tine axis at most its dz bound (20 mm), and every support a LowerRack wire group (EP.rack_labels): a dish among them is a
    perch (on_dish), any other body (other) fails too. None without a seat."""
    import frigidaire_robot_episode as EP
    from dishsim_frigidaire.robot.rig import T_of
    rel_ = row.get("release") or {}
    if rel_.get("seat_mm") is None:
        return None
    fr = R.frames_for(W.B, "LowerRack")["LowerRack"]
    x_dir, v, _ = EP.tine_axis(T_of(fr["position_m"], fr["quaternion_xyzw"]), np)
    off = EP.SEAT_DX_M * np.asarray(x_dir, dtype=float) + float(rel_["seat_mm"]) / 1e3 * np.asarray(v, dtype=float)
    dist = (float(np.hypot(off[0], off[1])), abs(float(off[2])), 0.)
    sup = sorted({str(s) for s in rel_.get("supports") or []})
    wires = set(EP.rack_labels(len(W.arms["LowerRack"].checker.parts["LowerRack"])))
    on = [s for s in sup if DISH_RE.fullmatch(s)]
    other = [s for s in sup if s not in wires and s not in on]
    along = abs(float(rel_["seat_mm"])) <= W.B.AT_GOAL["plate"][1] * 1e3 + 1e-9
    return {"ok": bool(W.B.within_goal("plate", dist)) and along and bool(sup) and not on and not other,
            "seat_mm": rel_["seat_mm"], "lateral_mm": round(dist[0] * 1e3, 1), "dz_mm": round(dist[1] * 1e3, 1),
            "along_ok": bool(along), "on_dish": on, "other": other, "supports": rel_.get("supports"),
            "tolerance": list(W.B.AT_GOAL["plate"]), "rule": SEAT_RULE}


def _edge_pick(W, pres):
    """reach.edge_pick with the presentation's other fixtures (the plank's lip) as statics of the arm model too (the s0 rerun's
    plank_gate.py) and the job's goal spin on every passing pick (the gate's I4-wall20 evaluates a pick at its "spin_deg")."""
    import frigidaire_robot_episode as EP
    W_ = R.plate_world(W)
    bid = (pres.get("backstop") or {}).get("id")
    for b in pres.get("fixtures") or []:
        if b.get("id") != bid:
            W_.arm.statics[b["id"]] = EP.box_body(W_.arm.fcl, b["id"], b["center_m"], b["size_m"], b["axes"], np)
    pk = _EDGE_PICK(W, pres)
    for c in pk.get("candidates") or []:
        c["spin_deg"] = _S.get("goal_spin", 0.)
    return pk


def _basket_out():
    """--no-basket in this worker, before its gate world is built: frigidaire_robot_reach.use_basket(False) (every World built
    after it strips BASKET from its checkers, reach.strip_basket); a reach without it: the wrap of the Kit-free measurement
    (scratchpad basket/basket_gate.py), InitialCollisionChecker.update_components popping BASKET from the checker's components
    right after the original builds them, so every check that walks the components (collide.ArmWorld, frigidaire_robot_episode.
    scene_targets, seat_search, against_components) no longer sees it; the frames keep all five (_frames requires them).
    -> what removed it"""
    if callable(getattr(R, "use_basket", None)):
        R.use_basket(False)
        return "frigidaire_robot_reach.use_basket(False)"
    from dishsim_frigidaire import initial_state_candidates as ISC
    cls = ISC.InitialCollisionChecker
    if not getattr(cls.update_components, "_no_basket", False):
        orig = cls.update_components

        def update_components(self, component_frames):
            orig(self, component_frames)
            if self.components and BASKET in self.components:
                self.components.pop(BASKET)
        update_components._no_basket = True
        cls.update_components = update_components
    return ("the access order's wrap of InitialCollisionChecker.update_components (the basket popped from the components "
            "after each build, as the Kit-free measurement)")


def _check_basket(W):
    """Both arms' checkers hold BASKET as CFG["basket"] says (a world built before the switch would keep it)."""
    want = bool(CFG.get("basket", True))
    has = {rk: BASKET in (arm.checker.components or {}) for rk, arm in W.arms.items()}
    if any(v != want for v in has.values()):
        raise RuntimeError(f"the gate world's {BASKET} per rack checker {has}, the run wants it {'in' if want else 'out'}")
    return {rk: sorted(arm.checker.components or {}) for rk, arm in W.arms.items()}


def _gate_dir(cands):
    return next((Path(p) for p in cands if p is not None and (Path(p) / "trials.jsonl").is_file()), None)


def _use_mount(mount):
    """frigidaire_robot_reach.use_mount (its --mount: the best mount and its counter spot "spot_xy"), else the mount paths alone."""
    if hasattr(R, "use_mount"):
        R.use_mount(mount)
    else:
        R.SAMPLE_MOUNT = R.FOOT_MOUNT = Path(mount)


def _world():
    """The gate's world on CFG["mount"], built by the gate's own init (sample_gate_job on a reject row), with the gates of record's
    closed jaws, the plank presentation, the scene wrapper on both arms and the straight start order of D-ring-side."""
    if "W" in _S:
        return _S["W"]
    import frigidaire_robot_episode as EP
    from dishsim_frigidaire.robot import collide as C
    mount = Path(CFG["mount"])
    _use_mount(mount)                                  # the gate reads its mount from SAMPLE_MOUNT at the world init
    removal = None if CFG.get("basket", True) else _basket_out()   # --no-basket: before the world's checkers are built
    R._W.clear()
    R.sample_gate_job((INIT_DISH, CFG["frames"], CFG["load_dir"]))
    W = R._W["w"]
    m = json.loads(mount.read_text())["best"]
    if not np.allclose(W.T_wb, R.mount_T(m)[0]):        # the gate took its mount from elsewhere: a world on ours, its resources kept
        W2 = R.World({"base": m["base"], "yaw_rad": m["yaw_rad"]})
        for k, v in vars(W).items():
            vars(W2).setdefault(k, v)
        W = R._W["w"] = W2
    ld = Path(CFG["load_dir"])
    pg = _gate_dir((ld / EP.PLATE_WALL_GATE_RUN, GATES["plate"]))
    if pg is not None:
        run, traj, src = EP.gate_trial(pg, fallback=EP.PLATE_GATE)
        W.plate_closed = (EP.closed_gripper_hulls(C, np, run=run, traj=traj), src)
    cg = _gate_dir((ld / EP.CUP_DOWN_GATE_RUN, GATES["cup"], EP.CUP_DOWN_GATE_DIR))
    if cg is not None:
        W.cup_hulls = EP.cup_gate_hulls(C, np, cg)
    bg = _gate_dir((ld / EP.FOOT_SIDE_GATE_RUN, GATES["bowl_side"]))
    if bg is not None:
        try:
            run, traj, src = EP.gate_trial(bg)
            W.bowl_side = (EP.closed_gripper_hulls(C, np, run=run, traj=traj), src)
        except RuntimeError:
            pass
    fs = next((p for p in (ld / "foot_side_candidates.json", SIDE_RANK) if p.is_file()), None)
    if fs is not None:
        W.side_rank = json.loads(fs.read_text())["families"]["side"].get("ranked")
    W.pres = CFG["pres"]
    comps = _check_basket(W)
    for arm in W.arms.values():
        arm._scene = {}
        arm.set_dishes = types.MethodType(_set_dishes, arm)
    EP.pitch_order = _straight_order
    R.edge_pick = _edge_pick
    _S.update(W=W, resources={"plate_hand": W.plate_closed[1], "cup_hand": W.cup_hulls[1], "bowl_side_hand": W.bowl_side[1],
                              "side_rank": None if fs is None else str(fs.relative_to(ROOT)),
                              "mount": {"base": m["base"], "yaw_rad": m["yaw_rad"]}, "T_wb": np.round(W.T_wb, 6).tolist(),
                              "basket": bool(CFG.get("basket", True)), "basket_removal": removal, "components": comps})
    return W


def _scene(W, rack, names):
    """The chain's earlier dishes ``names`` in W.arms[rack] at their predicted poses (CFG["poses"]: in the rack's frame), the
    other arm's scene empty."""
    from dishsim_frigidaire.robot.rig import T_of
    for arm in W.arms.values():
        arm._scene = {}
    if names:
        arm = W.arms[rack]
        fr = R.frames_for(W.B, rack)[rack]
        F = T_of(fr["position_m"], fr["quaternion_xyzw"])
        arm._scene = {m: arm.checker._body(kind_of(m), W.P.pose_dict(F @ np.asarray(CFG["poses"][m]["T_rel"], dtype=float)), m)
                      for m in names}
    for arm in W.arms.values():
        arm.set_dishes({})


def _job(job):
    """One (dish, choice, scene) gate row: frigidaire_robot_reach.sample_gate_job with the scene in (the side ring hold: both wrist
    sides, merged as sample_gate_main merges them). -> (dish, key, scene, row)."""
    n, key, scene = job
    W = _world()
    _, hold, gmove, spin = choice(n, key)
    d = dict(CFG["dishes"][n], move=gmove)
    if key == "side-down5":                              # the 5 mm cup hold (round 3's cup_05): its hold height and Isaac gate run
        d.update(CUP5)
    if CFG.get("direct"):                                # --direct-only: the tine-guided release off
        d["no_guided"] = True
    if spin is not None:                                 # the plate's pinch spin at the goal in the dish row too (a gate that
        d["pinch_spin_deg"] = float(spin)                # reads the load row's pinch_spin_deg, not the pick's spin_deg)
        if CFG.get("retry_s"):                           # --narrow-retry-s: the plate's leg-1 retry (the gate's i4_carry)
            d["_retry_s"] = float(CFG["retry_s"])
    if hold == "foot-side":                              # the load row's start order (a gate that reads it; an older gate: the
        d["ring_side_order"] = "straight"                # patched EP.pitch_order, rule 1's order with the straight-leg starts first)
    t0 = time.monotonic()
    _scene(W, d["rack"], scene)
    _S["goal_spin"] = 0. if spin is None else float(spin)
    try:
        if hold == "foot-side":
            rows = []
            for roll in R.WRIST_SIDES:
                rows.append(R.sample_gate_job((dict(d, _roll=roll), CFG["frames"], CFG["load_dir"])))
                if rows[-1].get("pass"):
                    break
            row = R.merge_rolls(rows)[0]
        else:
            row = R.sample_gate_job((d, CFG["frames"], CFG["load_dir"]))
    except Exception:                                    # noqa: BLE001  (a failed check is a row, never a dead search)
        row = {"dish": n, "gate": "error", "pass": False, "why": traceback.format_exc()[-800:], "blocker": []}
    finally:
        _scene(W, d["rack"], ())
        for b in CFG["pres"].get("fixtures") or []:
            W.arms["LowerRack"].statics.pop(b["id"], None)
        W.arms["LowerRack"].statics.pop("Backstop", None)
    row = json.loads(json.dumps(row, default=lambda o: o.tolist() if hasattr(o, "tolist") else str(o)))
    row.update(choice=key, hold=hold, gate_move=gmove, scene=list(scene), job_s=round(time.monotonic() - t0, 1),
               resources=_S.get("resources"))
    if spin is not None:
        row["goal_spin_deg"] = spin
        ps_ = (row.get("release") or {}).get("pinch_spin_deg")
        if ps_ is not None and abs(float(ps_) - float(spin)) > 1e-6:   # the gate checked another spin than the choice's: no verdict
            row.update(gate="error", why=f"the gate evaluated the pinch spin {ps_} deg, not the choice's {spin} deg", **{"pass": False})
        sg_ = seat_at_goal(W, row) if row.get("gate") != "error" else None
        if sg_ is not None:
            row["seat_at_goal"] = sg_
        row["release_mode"] = release_of(row)["mode"]
    return n, key, tuple(scene), row


# ------------------------------------------------------------------ the parent side (the search; no world here)
def culprits(row, scene):
    """The scene dishes a failing row names (its pairs, blockers and whys)."""
    txt = json.dumps({k: v for k, v in row.items() if k not in ("dish", "scene", "resources")})
    return sorted(set(DISH_RE.findall(txt)) & set(scene))


def start_of(row):
    """The descent start the gate carried to (or its first candidate), its carry legs and its release check."""
    tr, rel = row.get("transit") or {}, row.get("release") or {}
    legs = [l_.get("path") for l_ in tr.get("legs") or []]
    out = {"legs": legs, "all_straight": bool(legs) and all(p in ("straight", "line") for p in legs)}
    if kind_of(str(row.get("dish"))) == "plate":
        out.update(start_mm=tr.get("h_s_mm"), held_clear_mm=tr.get("held_clear_mm"), goal_spin_deg=rel.get("pinch_spin_deg"),
                   roll_deg=rel.get("roll_deg"), seat_mm=rel.get("seat_mm"),   # the seat search's height (0 = at the goal)
                   **{f"release_{k}" if k == "mode" else k: v for k, v in release_of(row).items()})
    else:
        out.update(spin_deg=tr.get("spin_deg"), start_mm=tr.get("descent_start_mm"), held_clear_mm=tr.get("held_clear_mm"),
                   held_clear_to=tr.get("held_clear_to"), start_choice=row.get("start_choice"),
                   **{k: tr[k] for k in ("pitch_deg", "roll_deg", "tilt_deg", "tilt_cap_deg", "rrt_budget_s") if k in tr})
    if row.get("release_check") is not None:
        out["release_check_ok"] = row["release_check"].get("ok")
        if row["release_check"].get("variant") is not None:   # D4: the retreat after the jaws open (the gate's first clear one)
            out["release_check_variant"] = row["release_check"]["variant"]
    return out


def brief(row, scene=()):
    out = {"gate": row.get("gate"), "pass": bool(row.get("pass")), "why": None if row.get("pass") else str(row.get("why"))[:300],
           "culprits": culprits(row, scene), "start": start_of(row), "s": row.get("job_s")}
    if row.get("seat_at_goal") is not None:            # SEAT_RULE: a plate's gate pass counts only with its seat at its goal
        out["seat_ok"] = bool(row["seat_at_goal"].get("ok"))
    return out


class Gate:
    """The memo {(dish, choice key, scene set): row} and the pool that fills it (thread-safe: the chains search side by side)."""

    def __init__(self, pool):
        self.pool, self.rows, self.lock, self.n, self.t0 = pool, {}, threading.Lock(), 0, time.monotonic()

    def run(self, todo):
        """[(dish, key, scene)] -> their rows, the missing ones on the pool."""
        key = lambda t: (t[0], t[1], frozenset(t[2]))
        with self.lock:
            miss = [t for t in dict.fromkeys((t[0], t[1], tuple(t[2])) for t in todo) if key(t) not in self.rows]
        if miss:
            for n, k, sc, row in self.pool.imap_unordered(_job, miss, chunksize=1):
                with self.lock:
                    self.rows[key((n, k, sc))] = row
                    self.n += 1
                drop_ = (f" drop {(row.get('release') or {}).get('release_mm')} mm" if row.get("release_mode") == "drop" else "") + (
                    " (seat off its goal)" if row.get("pass") and not ok_row(n, row) else "")
                print(f"[INFO] {time.monotonic() - self.t0:6.0f} s  job {self.n:3d}  {n} {k} scene {list(sc)}: {row.get('gate')} "
                      f"{'PASS' if row.get('pass') else 'fail'}{drop_} {row.get('job_s')} s"
                      f"{'' if row.get('pass') else '  ' + str(row.get('why'))[:160]}", flush=True)
        with self.lock:
            return [self.rows[key(t)] for t in todo]

    def step(self, n, scene, keys):
        """The dish's step at the scene over its choices ``keys`` (preference order): a plate's spins at once (the first that seats
        it before the first tine-guided drop: the supported release relaxed only where no spin seats), a bowl's axial hold before
        the side ring hold. {"pass", "key", "row", "tried": [(key, row)]}"""
        groups = [list(keys)] if kind_of(n) == "plate" else [[k] for k in keys]
        tried = []
        for g in groups:
            rows = self.run([(n, k, tuple(scene)) for k in g])
            tried += list(zip(g, rows))
            good = [(k, r) for k, r in zip(g, rows) if ok_row(n, r)]   # a plate: its seat at its goal (SEAT_RULE)
            ok = next((x for x in good if kind_of(n) != "plate" or release_of(x[1])["mode"] != "drop"), None) or next(iter(good), None)
            if ok is not None:
                return {"pass": True, "key": ok[0], "row": ok[1], "tried": tried}
        return {"pass": False, "key": None, "row": None, "tried": tried}


def drop_culprits(row, scene):
    """The scene dishes that stopped a drop step's seat descent (its release's first_violation and descent_blockers)."""
    rel_ = row.get("release") or {}
    return sorted(set(DISH_RE.findall(json.dumps([rel_.get("first_violation"), rel_.get("descent_blockers")]))) & set(scene))


def fix_culprits(st, scene):
    """A failed step's culprits: of its choices that name scene dishes, the fewest (the preference order breaks ties); none named:
    the last placed dish (chronological). (culprits, the choice key whose row they come from)"""
    named = [(culprits(r, scene), k) for k, r in st["tried"]]
    named = [x for x in named if x[0]]
    if named:
        return min(named, key=lambda x: len(x[0]))
    return ([scene[-1]] if scene else []), (st["tried"][0][0] if st["tried"] else None)


def reaches(a, b, edges):
    """True when ``a`` must come before ``b`` along ``edges`` ((u, v): u before v)."""
    seen, todo = set(), [a]
    while todo:
        u = todo.pop()
        if u == b:
            return True
        if u in seen:
            continue
        seen.add(u)
        todo += [v for u_, v in edges if u_ == u]
    return False


def step_record(n, st, scene, chain_index):
    """The order.json record of a dish's step (placed or blocked)."""
    k = st["key"] or fix_culprits(st, scene)[1]
    row = st["row"] or dict(st["tried"]).get(k) or {}
    _, hold, gmove, spin = choice(n, k)
    rel_ = release_of(row) if spin is not None else None   # a plate: its release mode (seat, or the tine-guided drop, whose
    mv_ = PL.fifteen_move(gmove, n in INSERT_SITES, rel_ if st["pass"] else None)   # episode move I4-wall20-drop a pass names)
    rec = {"dish": n, "kind": kind_of(n), "rack": CFG["dishes"][n]["rack"], "slot": CFG["dishes"][n]["slot"],
           "chain_index": chain_index, "order_index": CFG["dishes"][n]["order_index"], "hold": hold,
           "move": mv_, "gate_move": gmove, "scene": list(scene),
           "scene_poses": {m: CFG["poses"][m]["source"] for m in scene}, "gate": row.get("gate"), "pass": bool(st["pass"]),
           "why": row.get("why"), "blocker": row.get("blocker"), "start": start_of(row),
           "tried": {k_: brief(r_, scene) for k_, r_ in st["tried"]}, "seconds": row.get("job_s")}
    if spin is not None:
        rec["goal_spin_deg"] = spin
        rec["seat_at_goal"] = row.get("seat_at_goal")
        rec["release"] = rel_
    if not st["pass"]:
        rec["culprits"] = fix_culprits(st, scene)[0]
    return rec, row


def eff_edges(dag, flips, learned):
    """The chain's edges ((u, v): u before v): the dag's without the reversed access edges, then the learned ((x, c, why))."""
    rv = {tuple(f_["edge"]) for f_ in flips}
    return [e for e in dag if e not in rv] + [(a, b) for a, b, _ in learned]


def reverse_test(gate, x, c, sc, st, k_, keys, run):
    """REVERSE_RULE's test of the dag access edge c -> x (c a plate), x having failed (choice k_) with c in its scene ``sc``: c
    gated at its own scene of the run (the dishes placed before it) with x added at its predicted pose, every pinch spin of c
    (Gate.step). -> the record ("pass": some spin of c passes with its seat at its goal, seat_at_goal; "gate_pass": the gate's
    bare pass; both gate rows: x with c, c with x)."""
    scene = [m for m in sc[:sc.index(c)] if m != x] + [x]
    st_c = gate.step(c, scene, keys[c])                  # a plate: every spin at once (Gate.step), all of them tried
    good = [(k, r) for k, r in st_c["tried"] if ok_row(c, r)]
    kc = good[0][0] if good else next((k for k, r in st_c["tried"] if r.get("pass")), None) or fix_culprits(st_c, scene)[1]
    row_c, row_x = dict(st_c["tried"]).get(kc) or {}, dict(st["tried"]).get(k_) or {}
    return {"edge": [c, x], "kind": "access", "now": [x, c], "run": run, "pass": bool(good),
            "gate_pass": any(bool(r.get("pass")) for _, r in st_c["tried"]),
            "seat_at_goal": row_c.get("seat_at_goal"), "rule": REVERSE_RULE,
            "x_with_c": {"dish": x, "scene": list(sc), "choice": k_, "blocker": row_x.get("blocker"), **brief(row_x, sc)},
            "c_with_x": {"dish": c, "scene": scene, "choice": kc, "blocker": row_c.get("blocker"), **brief(row_c, scene),
                         "tried": {k: {**brief(r, scene), "seat_at_goal": r.get("seat_at_goal")} for k, r in st_c["tried"]}},
            "scene_poses": {m: CFG["poses"][m]["source"] for m in scene}}


def chain_search(gate, chain, keys, log, soft=False):
    """The chain's order: greedy runs in the dag order with the learned edges (see the module doc); ``soft`` (--dag soft): a
    conflict with a dag access edge is tested by REVERSE_RULE; then the drop preference (DROP_PREF_RULE). -> the best run's record
    (the most dishes placed, then the fewest drops)."""
    prefer = [n for n in CFG["dag_order"] if n in chain]
    dag = [(a, b) for a, b in CFG["dag_edges"] if a in chain and b in chain]
    access = {(a, b) for a, b in CFG.get("access_edges") or () if a in chain and b in chain}
    extra, best, conflicts, flips = [], None, [], []

    def greedy(edges):
        """One run under ``edges``: (order, blocked {x: (its scene, its step)}, the plates placed by the drop)."""
        preds = {x: {a for a, b in edges if b == x} for x in chain}
        order, blocked, drops = [], {}, []
        while True:
            left = [x for x in prefer if x not in order and x not in blocked]
            if not left:
                break
            free = [x for x in left if preds[x] <= set(order) | set(blocked)] or left[:1]
            x = free[0]
            st = gate.step(x, order, keys[x])
            if st["pass"]:
                order.append(x)
                if kind_of(x) == "plate" and release_of(st["row"])["mode"] == "drop":
                    drops.append(x)
            else:
                blocked[x] = (list(order), st)
        return order, blocked, drops
    score = lambda b_: (len(b_["order"]), -len(b_["drops"]))
    for run in range(1, MAX_RUNS + 1):
        order, blocked, drops = greedy(eff_edges(dag, flips, extra))
        log(f"run {run}: order {order}, blocked {list(blocked)}, learned {[(a, b) for a, b, _ in extra]}"
            + (f", reversed {[f_['edge'] for f_ in flips]}" if flips else "") + (f", drops {drops}" if drops else ""))
        cur_ = {"order": list(order), "blocked": dict(blocked), "drops": drops, "run": run, "learned": list(extra),
                "reversed": list(flips), "prefer": []}
        if best is None or score(cur_) > score(best):
            best = cur_
        if not blocked:
            break
        new = []
        for x, (sc, st) in blocked.items():
            cul, k_ = fix_culprits(st, sc)
            for c in cul:
                if any((x, c) == (a, b) for a, b, _ in extra + new):
                    continue
                cur = eff_edges(dag, flips, extra + new)
                if reaches(c, x, cur):
                    test, why_ = None, None
                    if soft and (c, x) in access and (c, x) in cur and kind_of(c) not in REVERSIBLE_KINDS:
                        why_ = (f"not reversible: {c} is a {kind_of(c)} (its descent is along the rack's up axis, the path the "
                                "access edge records)")
                    elif soft and (c, x) in access and (c, x) in cur and not reaches(c, x, [e for e in cur if e != (c, x)]):
                        test = reverse_test(gate, x, c, sc, st, k_, keys, run)
                        sg_ = test.get("seat_at_goal") or {}
                        log(f"run {run}: the access edge {c} -> {x}: {c} with {x} in the scene {test['c_with_x']['scene']} "
                            f"{test['c_with_x']['gate']} at {test['c_with_x']['choice']}, seat {sg_.get('seat_mm')} mm on "
                            f"{sg_.get('on_dish') or sg_.get('supports')} (at goal {sg_.get('ok')}): "
                            f"{'reversed' if test['pass'] else 'kept'}")
                        if test["pass"]:
                            flips.append(test)
                            new.append((x, c, f"run {run}: {x} failed ({k_}) with {c} in the scene; the access edge {c} -> {x} "
                                              f"reversed ({c} passes with {x} in the scene)"))
                            continue
                    conflicts.append({"dish": x, "before": c, "scene": sc, "choice": k_, "run": run,
                                      "why": "c already precedes x (the dag or a learned edge): the pair blocks each other",
                                      **({"reversal_test": test} if test is not None else {}),
                                      **({"reversal": why_} if why_ else {})})
                    continue
                new.append((x, c, f"run {run}: {x} failed ({k_}) with {c} in the scene"))
        if not new:
            break
        extra += new
    pref_tried = []                                     # DROP_PREF_RULE: one preference edge per run, each edge once
    while best["drops"] and len(pref_tried) < MAX_RUNS:
        base = eff_edges(dag, best["reversed"], best["learned"]) + best["prefer"]
        cand = next(((x, c) for x in best["drops"] for c in drop_culprits(
            gate.step(x, best["order"][:best["order"].index(x)], keys[x])["row"], best["order"][:best["order"].index(x)])
                     if [x, c] not in [t_["edge"] for t_ in pref_tried] and not reaches(c, x, base)), None)
        if cand is None:
            break
        run += 1
        order, blocked, drops = greedy(base + [cand])
        cur_ = {"order": list(order), "blocked": dict(blocked), "drops": drops, "run": run, "learned": list(best["learned"]),
                "reversed": list(best["reversed"]), "prefer": best["prefer"] + [cand]}
        kept = score(cur_) > score(best)
        pref_tried.append({"edge": list(cand), "run": run, "order": list(order), "blocked": list(blocked), "drops": drops,
                           "kept": kept})
        log(f"run {run} (drop preference: {cand[0]} before {cand[1]}): order {order}, blocked {list(blocked)}, drops {drops}: "
            f"{'kept' if kept else 'not kept'}")
        if kept:
            best = cur_
    steps, rows = [], {}
    for i, x in enumerate(best["order"]):
        sc = best["order"][:i]
        rec, rows[x] = step_record(x, gate.step(x, sc, keys[x]), sc, i)
        steps.append(rec)
    blocked = {}
    for x, (sc, st) in best["blocked"].items():
        blocked[x], rows[x] = step_record(x, st, sc, len(best["order"]) + len(blocked))
    for f_ in best["reversed"]:                         # the steps of a reversed edge's two dishes name it
        for s_ in steps + list(blocked.values()):
            if s_["dish"] in f_["edge"]:
                s_.setdefault("reversed_access", []).append(f_["edge"])
    return {"order": best["order"], "complete": not best["blocked"], "steps": steps, "blocked": blocked, "best_run": best["run"],
            "runs": run, "learned": [list(e) for e in best["learned"]], "learned_all": [list(e) for e in extra],
            "conflicts": conflicts, "dag": "soft" if soft else "hard", "reversed": best["reversed"], "reversed_all": flips,
            "drops": best["drops"], "drop_preference": [list(e) for e in best["prefer"]], "drop_tried": pref_tried,
            "drop_rule": DROP_PREF_RULE}, rows


def pair_report(gate, a, b, ch, keys):
    """Both orders of a pair per choice (the plates: per spin at the goal): the first on the scene before the pair in the chain's
    order, the second with the first added."""
    order = ch["order"] + list(ch["blocked"])
    idx = [order.index(x) for x in (a, b) if x in order]
    p0 = [x for x in order[:min(idx)] if x not in (a, b)] if idx else []
    out = {"scene_before": p0, "orders": {}}
    for x, y in ((a, b), (b, a)):
        rx = gate.run([(x, k, tuple(p0)) for k in keys[x]])
        ry = gate.run([(y, k, tuple(p0 + [x])) for k in keys[y]])
        kx = next((k for k, r in zip(keys[x], rx) if ok_row(x, r)), None)
        ky = next((k for k, r in zip(keys[y], ry) if ok_row(y, r)), None)
        out["orders"][f"{x} then {y}"] = {
            "fits": kx is not None and ky is not None, "choices": [kx, ky],
            x: {k: brief(r, p0) for k, r in zip(keys[x], rx)}, y: {k: brief(r, p0 + [x]) for k, r in zip(keys[y], ry)}}
    fit = [o for o, v in out["orders"].items() if v["fits"]]
    out["verdict"] = (f"both fit: {fit[0]} at {out['orders'][fit[0]]['choices']}" if fit else
                      "blocked both ways: no choice of the second passes with the first in the scene (blockers per choice above)")
    return out


def pose_table(smp, frames, state_dirs):
    """{dish: {"rack", "T_rel" (the pose in its rack's frame), "source"}}: the dish's own state file where a run placed it
    (after_*_<dish>.json with the dish placed; the first of ``state_dirs`` that has one), else its goal pose."""
    from dishsim_frigidaire.robot.rig import T_of
    T = lambda p: T_of(p["position_m"], p["quaternion_xyzw"])
    out = {}
    for n, s_ in smp["goal"]["slots"].items():
        rk, hit = s_["rack"], None
        for sd in state_dirs:
            for f in sorted(Path(sd).glob(f"after_*_{n}.json")):
                st = json.loads(f.read_text())
                if st.get("after") == n and n in (st.get("placed") or {}):
                    hit = (f, st["placed"][n], st["rack_frames"][rk])
            if hit:
                break
        if hit:
            f, p, fr = hit
            out[n] = {"rack": rk, "T_rel": (np.linalg.inv(T(fr)) @ T(p)).tolist(), "source": rel(f)}
        else:
            p = smp["goal"]["poses"][n]
            out[n] = {"rack": rk, "T_rel": (np.linalg.inv(T(frames[rk])) @ T(p)).tolist(), "source": "goal.poses (never placed)"}
    return out


def sha16(p):
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()[:16]


def rel(p):
    """``p`` relative to the repo root when it lies under it, else its absolute path (an --out outside the repo: a scratch copy)."""
    for a_, b_ in ((Path(p), ROOT), (Path(p).resolve(), ROOT.resolve())):
        try:
            return str(a_.relative_to(b_))
        except ValueError:
            pass
    return str(Path(p).resolve())


def config(sample, mount, states, presentation, out):
    """CFG for the workers (set before the pool forks): the mount, the load's frames (the baseline's), the load dir, the plank, the
    gate's dish rows (the build's sites with their build release), the predicted poses and the dag. -> the sample"""
    smp = json.loads(sample.read_text())
    goal, dag = smp["goal"], smp["dag"]
    base = json.loads(PL.BASELINE.read_text())
    snap = base.get("snapshot", base)
    frames = {r: snap["poses"][r] for r in ("LowerRack", "UpperRack", "SilverwareBasket")}
    builds = PL.build_releases(smp, [n for n in INSERT_SITES if n in goal["slots"]])
    dishes = {}
    for k, n in enumerate(dag["order"]):
        s_, p = goal["slots"][n], goal["poses"][n]
        dishes[n] = {"name": n, "kind": kind_of(n), "rack": s_["rack"], "slot": s_["slot"], "variant": s_["variant"], "order_index": k,
                     "goal_pose": {"pos": p["position_m"], "quat_xyzw": p["quaternion_xyzw"]},
                     **({"build_release": builds[n]} if n in builds else {})}
    _use_mount(mount)                                  # the mount's counter spot moves the pick and the plank with it
    pres = json.loads(presentation.read_text())
    pres = R.shift_presentation(pres) if hasattr(R, "shift_presentation") else pres
    CFG.update(mount=str(mount), frames=frames, load_dir=str(out.parent), pres=pres, spot_xy=[float(v) for v in R.COUNTER_XY],
               dishes=dishes, poses=pose_table(smp, frames, states), dag_order=list(dag["order"]),
               dag_edges=[(e["a"], e["b"]) for e in dag["edges"]],
               access_edges=[(e["a"], e["b"]) for e in dag["edges"] if e.get("kind") == "access"])
    return smp


def direct_ok(n, row):
    """--direct-only: the gate's pass with a direct release (DIRECT_RULE)."""
    rel_, rc = row.get("release") or {}, row.get("release_check")
    return (bool(row.get("pass")) and row.get("move") in DIRECT_MOVES and rel_.get("build_release") is None
            and not rel_.get("stop_mode") and (rc is None or bool(rc.get("ok")))
            and (kind_of(n) != "plate" or (release_of(row)["mode"] == "seat" and bool((row.get("seat_at_goal") or {}).get("ok")))))


def direct_why(n, row):
    """Why a row is not a direct pass (None when it is)."""
    if direct_ok(n, row):
        return None
    rel_, rc = row.get("release") or {}, row.get("release_check")
    if not row.get("pass"):
        return str(row.get("why"))[:400]
    if row.get("move") not in DIRECT_MOVES:
        return f"move {row.get('move')} is not direct"
    if rel_.get("build_release") is not None or rel_.get("stop_mode"):
        return f"build release / stop mode {rel_.get('stop_mode')}"
    if rc is not None and not rc.get("ok"):
        return f"release check: {rc.get('why') or rc.get('stages')}"[:400]
    if release_of(row)["mode"] != "seat":
        return f"plate release mode {release_of(row)['mode']} (not at the seat)"
    sg = row.get("seat_at_goal") or {}
    return f"plate seat off its goal: lateral {sg.get('lateral_mm')} mm, dz {sg.get('dz_mm')} mm, on {sg.get('on_dish')}, other {sg.get('other')}"


def rrt_timeout(row):
    """The row failed on a carry RRT that found no path in its budget."""
    w = str(row.get("why") or "")
    return w.startswith("carry:") and ("no path" in w or "budget" in w)


def direct_verdict(n, scene, tried):
    """A dish's --direct-only record from its choices' rows ``tried`` [(key, row)] in preference order."""
    good = [(k, r) for k, r in tried if direct_ok(n, r)]
    k, r = good[0] if good else tried[0]
    _, hold, gmove, spin = choice(n, k)
    st = start_of(r)
    cul = sorted(set().union(*[set(culprits(r_, scene)) | set(drop_culprits(r_, scene)) for _, r_ in tried]))
    out = {"dish": n, "kind": kind_of(n), "rack": CFG["dishes"][n]["rack"], "slot": CFG["dishes"][n]["slot"], "scene": list(scene),
           "direct_ok": bool(good), "direct_why": None if good else "; ".join(f"{k_}: {direct_why(n, r_)}" for k_, r_ in tried),
           "culprits": [] if good else cul, "carry_timeout": (not good) and any(rrt_timeout(r_) for _, r_ in tried),
           "choice": k, "move": gmove, "hold": hold,
           "spin_deg": spin if spin is not None else st.get("spin_deg"), "start_mm": st.get("start_mm"), "start": st,
           "gate": r.get("gate"), "pick_ok": (r.get("pick") or {}).get("ok", bool((r.get("pick") or {}).get("pass"))),
           "tried": {k_: dict(brief(r_, scene), direct_ok=direct_ok(n, r_), direct_why=direct_why(n, r_)) for k_, r_ in tried}}
    if k == "side-down5":
        out.update(CUP5)
    return out, {"pass": bool(good), "key": k if good else None, "row": r if good else None, "tried": tried}


def candidate_load_dir(ld):
    """The load dir of a --candidate / --direct-only run: the gates of record at posture_v2 (<s0>__fifteen/partA_*, and
    partA_cup_side_down5 of <s0>__fifteen_r3) as relative symlinks, and a copy of <s0>__fifteen/presentation.json."""
    ld = Path(ld).absolute()
    ld.mkdir(parents=True, exist_ok=True)
    links = {q.name: q for q in sorted(FIFTEEN_DIR.glob("partA_*"))}
    links.setdefault(CUP5["hold_gate_run"], FIFTEEN_R3 / CUP5["hold_gate_run"])
    for name, tgt in links.items():
        if not (ld / name).is_symlink() and not (ld / name).exists():
            (ld / name).symlink_to(os.path.relpath(Path(tgt).absolute(), ld))
    if not (ld / "presentation.json").is_file():
        shutil.copyfile(FIFTEEN_DIR / "presentation.json", ld / "presentation.json")
    return ld


def direct_main(a, out, smp, sid, t0):
    """--direct-only / --gate-one: the dishes (every dish of the dag order with the earlier dishes of its rack, or the --gate-one
    dishes with their --scene) gated over their direct choices -> order.json (per dish its verdict; chains in the fifteen format)
    and gate.json beside it."""
    order_, dishes = list(smp["dag"]["order"]), CFG["dishes"]
    if a.gate_one:
        sc_ = list(a.scene or [])
        pairs = [(n, [m for m in (sc_[i] if i < len(sc_) else "").split(",") if m]) for i, n in enumerate(a.gate_one)]
    else:
        pairs = [(n, [m for m in order_[:i] if dishes[m]["rack"] == dishes[n]["rack"]]) for i, n in enumerate(order_)]
    print(f"[INFO] {sid}: direct-only, mount {rel(a.mount)}, basket {'in' if CFG['basket'] else 'OUT'}, {len(pairs)} dishes, "
          f"narrow retry {CFG['retry_s']} s", flush=True)
    with mp.get_context("fork").Pool(a.workers) as pool:
        gate = Gate(pool)
        gate.run([(n, k, tuple(sc)) for n, sc in pairs for k, *_ in choices(n)])
    per, steps, rows_out = {}, {}, {}
    for n, sc in pairs:
        tried = [(k, gate.rows[(n, k, frozenset(sc))]) for k, *_ in choices(n)]
        per[n], st = direct_verdict(n, sc, tried)
        steps[n], rows_out[n] = step_record(n, st, sc, None)
    chains = {}
    for c, rk in (("upper", "UpperRack"), ("lower", "LowerRack")):
        mine = [n for n, _ in pairs if dishes[n]["rack"] == rk]
        chains[c] = {"order": [n for n in mine if per[n]["direct_ok"]], "steps": [steps[n] for n in mine if per[n]["direct_ok"]],
                     "blocked": {n: steps[n] for n in mine if not per[n]["direct_ok"]}, "reversed": [], "learned": []}
    resources = next((r.get("resources") for r in gate.rows.values() if r.get("resources")), None)
    failed = [n for n in per if not per[n]["direct_ok"]]
    order = {"plan": "plans/2026-11-05-hardware-aware-sampler.md", "mode": "gate-one" if a.gate_one else "direct-only",
             "sample": rel(a.sample), "sample_id": sid, "mount_file": rel(a.mount), "basket": CFG["basket"], "resources": resources,
             "spot_xy": CFG["spot_xy"], "presentation": rel(a.presentation), "presentation_used": CFG["pres"],
             "load_dir": rel(out.parent), "rule": DIRECT_RULE, "seat_rule": SEAT_RULE, "narrow_retry_s": CFG["retry_s"],
             "dag": "hard", "order": order_, "dishes": per, "all_direct_ok": not failed, "failed": failed,
             "chains": chains, "rejects": {}, "insert_sites": {}, "reversed": {},
             "executed_order": [n for c in ("upper", "lower") for n in chains[c]["order"] + list(chains[c]["blocked"])],
             "evaluations": gate.n, "seconds": round(time.monotonic() - t0, 1)}
    out.write_text(json.dumps(order, indent=1, default=lambda o: o.tolist() if hasattr(o, "tolist") else str(o)) + "\n")
    for n, r_ in rows_out.items():
        r_["move"] = PL.fifteen_move(r_.get("gate_move"), False, release_of(r_) if kind_of(n) == "plate" else None)
        if kind_of(n) == "plate":
            r_["pinch_spin_deg"] = r_.get("goal_spin_deg", (r_.get("release") or {}).get("pinch_spin_deg"))
        if r_.get("hold") == "foot-side":
            r_["ring_side_order"] = "straight"
    (out.parent / "gate.json").write_text(json.dumps(
        {"plan": order["plan"], "order": rel(out), "mount_file": rel(a.mount), "basket": CFG["basket"], "rule": DIRECT_RULE,
         "dishes": rows_out, "seconds": order["seconds"]}, indent=1, default=lambda o: o.tolist() if hasattr(o, "tolist") else str(o))
        + "\n")
    print(f"[RESULT] PASS {order['mode']} {sid}: {len(per) - len(failed)}/{len(per)} direct; failed "
          f"{ {n: (per[n]['culprits'], str(per[n]['direct_why'])[:120]) for n in failed} }; {gate.n} gate jobs in "
          f"{order['seconds']} s -> {rel(out)}", flush=True)


def main():
    global INSERT_SITES
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--sample", type=Path, default=SAMPLE)
    ap.add_argument("--mount", type=Path, default=MOUNT, help="the pedestal mount file (its best), default posture_v1.json")
    ap.add_argument("--workers", type=int, default=6)
    ap.add_argument("--states", type=Path, nargs="*", default=list(STATE_DIRS),
                    help="state dirs of earlier runs (the first that placed a dish gives its landed pose)")
    ap.add_argument("--presentation", type=Path, default=PRESENTATION, help="the plates' presentation (the plank)")
    ap.add_argument("--chains", default="upper,lower")
    ap.add_argument("--narrow-retry-s", type=float, default=0.,
                    help="the plates' narrow-passage carry retry passed to the gate as the dish row's _retry_s (frigidaire_robot_"
                         "reach.i4_carry, collide.rrt_connect retry_s; 180 = collide.NARROW_RETRY_S as the mount sweep's plates); "
                         "default 0: none (round 1)")
    ap.add_argument("--dag", choices=("soft", "hard"), default="soft",
                    help="soft (default): a plate's dag access edge may be reversed for the arm's chain when the plate passes "
                         "with the later dish in the scene and its seat at its goal (REVERSE_RULE); hard: every dag edge kept "
                         "(round 1)")
    ap.add_argument("--no-basket", action="store_true",
                    help="the gate's world without the SilverwareBasket in every worker (the user, 2026-11-04 19:20 EST: the robot "
                         "rounds load the twin without it, the real basket lifts out); order.json and gate.json record \"basket\": "
                         "false. Default: the basket in (rounds 1 and 2)")
    ap.add_argument("--out", type=Path, default=None, help="default samples_v1/<sample_id>__fifteen/order.json (gate.json beside "
                                                          "it; a path outside the repo keeps absolute paths in the records)")
    ap.add_argument("--candidate", type=Path, help="a four-block candidate load (the hardware-aware sampler's: goal.slots, goal.poses, "
                                                   "dag.order and edges; goal_record null) in place of --sample: no insert sites, no "
                                                   "state dirs, the basket in, --out required, its load dir beside --out")
    ap.add_argument("--direct-only", action="store_true", help="DIRECT_RULE: the dag order with the earlier dishes of each rack, the "
                                                               "direct choices only, per dish direct_ok, direct_why, culprits")
    ap.add_argument("--gate-one", action="append", default=[], metavar="DISH",
                    help="--direct-only for this dish alone with its --scene (repeatable, paired with --scene by position)")
    ap.add_argument("--scene", action="append", default=[], metavar="D1,D2", help="the scene of the matching --gate-one ('' none)")
    a = ap.parse_args()
    ab = lambda p: p if p.is_absolute() else ROOT / p
    direct = a.direct_only or bool(a.gate_one)
    if a.candidate is not None:
        a.sample = a.candidate
    if (a.candidate is not None or direct) and a.out is None:
        raise SystemExit("[RESULT] FAIL --candidate / --direct-only / --gate-one need --out")
    if a.candidate is not None or direct:               # the hardware-aware sampler: no insert sites, no state dirs, the basket in
        INSERT_SITES, a.states = {}, []                 # --no-basket as given (the hardware-aware sampler's basket-out variant)
        CFG["direct"] = direct
        ld = candidate_load_dir(ab(a.out).parent)
        if a.presentation == PRESENTATION:
            a.presentation = ld / "presentation.json"
    a.sample, a.mount, a.presentation = ab(a.sample), ab(a.mount), ab(a.presentation)
    t0 = time.monotonic()
    sid = json.loads(a.sample.read_text())["metadata"]["sample_id"]
    out = ab(a.out) if a.out else PL.SAMPLES / f"{sid}__fifteen/order.json"
    out.parent.mkdir(parents=True, exist_ok=True)
    states = [ab(s) for s in a.states]
    smp = config(a.sample, a.mount, states, a.presentation, out)
    CFG["retry_s"] = float(a.narrow_retry_s)
    CFG["basket"] = not a.no_basket                    # before the pool forks: each worker's world reads it (_world)
    if direct:
        return direct_main(a, out, smp, sid, t0)
    dag, dishes = smp["dag"], CFG["dishes"]
    print(f"[INFO] {sid}: mount {rel(a.mount)}, {len(dishes)} dishes, --dag {a.dag}, basket {'in' if CFG['basket'] else 'OUT'}, "
          f"the side ring hold at a build site gated as "
          f"{SIDE_INSERT_MOVE}, landed poses "
          f"{ {n: v['source'].split('/')[-1] for n, v in CFG['poses'].items() if not v['source'].startswith('goal')} }", flush=True)
    log_lines = []

    def log(msg):
        log_lines.append(msg)
        print(f"[INFO] {msg}", flush=True)
    with mp.get_context("fork").Pool(a.workers) as pool:
        gate = Gate(pool)
        first = [(n, k, ()) for n in dag["order"] for k, *_ in (choices(n) if kind_of(n) == "plate" else choices(n)[:1])]
        hw = dict(zip([(n, k) for n, k, _ in first], gate.run(first)))           # every dish alone: hardware only
        side = [(n, "foot-side", ()) for n in dag["order"] if kind_of(n) == "bowl" and not ok_row(n, hw[(n, "foot-out")])]
        hw.update(zip([(n, k) for n, k, _ in side], gate.run(side)))
        keys, rejects, gate_rows = {}, {}, {}
        for n in dag["order"]:
            ks = [k for k, *_ in choices(n) if (n, k) not in hw or ok_row(n, hw[(n, k)])]
            if kind_of(n) == "bowl" and not ok_row(n, hw[(n, "foot-out")]):
                ks = [k for k in ks if k != "foot-out"]
            keys[n] = ks
            if not ks:
                k0 = choices(n)[0][0]
                st = {"pass": False, "key": None, "row": None, "tried": [(k, hw[(n, k)]) for k, *_ in choices(n) if (n, k) in hw]}
                rejects[n], gate_rows[n] = step_record(n, st, [], None)
                rejects[n]["key"] = k0
        log(f"hardware only: rejects {sorted(rejects)}; choices {keys}")
        chains = {"upper": [n for n in dag["order"] if dishes[n]["rack"] == "UpperRack" and n not in rejects],
                  "lower": [n for n in dag["order"] if dishes[n]["rack"] == "LowerRack" and n not in rejects]}
        res, threads = {}, []
        for c in [c for c in a.chains.split(",") if c]:
            th = threading.Thread(target=lambda c=c: res.__setitem__(c, chain_search(gate, chains[c], keys,
                                                                                     lambda m, c=c: log(f"{c}: {m}"),
                                                                                     soft=a.dag == "soft")))
            th.start()
            threads.append(th)
        for th in threads:
            th.join()
        pairs = {}
        for x, y in PAIRS:
            c = next((c for c in res if x in chains[c] and y in chains[c]), None)
            if c is not None:
                pairs[f"{x}|{y}"] = pair_report(gate, x, y, res[c][0], keys)
                log(f"pair {x}|{y}: {pairs[f'{x}|{y}']['verdict']}")
        n_jobs = gate.n
        resources = next((r.get("resources") for r in gate.rows.values() if r.get("resources")), None)
    for c, (_, rows) in res.items():
        gate_rows.update(rows)
    chains_out = {c: v[0] for c, v in res.items()}
    order = {"plan": "plans/2026-11-04-fifteen-of-fifteen.md", "sample": rel(a.sample), "sample_id": sid,
             "mount_file": rel(a.mount), "basket": CFG["basket"], "resources": resources, "spot_xy": CFG["spot_xy"],
             "presentation": rel(a.presentation), "presentation_used": CFG["pres"],
             "states": [rel(s) for s in states],
             "rule": "per rack chain a linearization of the dag (greedy in the dag order; a dish that fails gets the scene dishes its "
                     "failure names, else the last placed, as learned place-before edges, and the chain runs again; the best run is "
                     "kept; --dag soft: a conflicting access edge reversed by reverse_rule) in which every step passes "
                     "frigidaire_robot_reach.sample_gate_job (pick, descent start and line, carry, "
                     "open-jaw release; every margin, cap and budget the gate's) with the chain's earlier dishes in the scene at "
                     "their predicted landed poses; bowls the axial foot ring where it passes, else the side ring hold (its starts "
                     "straight legs first, then rule 1); plates the smallest |spin| at the goal that passes; a dish no choice passes "
                     "alone (hardware only) is a reject; a plate's step passes only with its seat at its goal (seat_rule), "
                     "released at the seat or by the tine-guided drop (its release mode; the smallest |spin| that seats it before "
                     "the smallest that drops it)",
             "seat_rule": SEAT_RULE, "drop_rule": DROP_PREF_RULE,
             "basket_rule": ("the gate's world without the SilverwareBasket in every worker (--no-basket; the user, 2026-11-04 "
                             "19:20 EST; resources.basket_removal says how); s0's goal poses and S_ref keep it")
             if not CFG["basket"] else "the twin with its SilverwareBasket (the default)",
             "dag": a.dag, "reverse_rule": REVERSE_RULE, "narrow_retry_s": CFG["retry_s"],
             "reversed": {c: v["reversed"] for c, v in chains_out.items() if v.get("reversed")},
             "gate_moves": {"insert_sites": {n: [c_[2] for c_ in choices(n)] for n in INSERT_SITES},
                            "side_insert_move": SIDE_INSERT_MOVE},
             "plate_spins_deg": list(PLATE_SPINS_DEG), "pick_spin_deg": 0., "insert_sites": INSERT_SITES,
             "poses": {n: {k: v[k] for k in ("rack", "source")} for n, v in CFG["poses"].items()},
             "rejects": rejects, "chains": chains_out,
             "executed_order": [n for c in ("upper", "lower") if c in chains_out
                                for n in chains_out[c]["order"] + list(chains_out[c]["blocked"])],
             "off_goal_seats": {s_["dish"]: s_["seat_at_goal"] for v in chains_out.values() for s_ in v["steps"]
                                if s_.get("seat_at_goal") and not s_["seat_at_goal"].get("ok")},
             "off_goal_rule": "a placed plate step whose gate pass seats it off its goal (seat_at_goal: beyond AT_GOAL or on a "
                              "dish) is listed here; since the third round a step needs its seat at its goal (seat_rule), so the "
                              "list stays empty (a consistency check)",
             "plate_releases": {s_["dish"]: dict(s_["release"], spin_deg=s_.get("goal_spin_deg"), move=s_["move"])
                                for v in chains_out.values() for s_ in v["steps"] if s_.get("release")},
             "pairs": pairs, "log": log_lines, "evaluations": n_jobs, "seconds": round(time.monotonic() - t0, 1),
             "code_sha256_16": {str(p.relative_to(ROOT)): sha16(p) for p in (
                 Path(__file__), ROOT / "code/execution/frigidaire/frigidaire_robot_reach.py",
                 ROOT / "code/execution/frigidaire/frigidaire_placing_loads.py",
                 ROOT / "code/execution/frigidaire/frigidaire_robot_episode.py",
                 ROOT / "code/frigidaire/src/dishsim_frigidaire/robot/collide.py",
                 ROOT / "code/frigidaire/src/dishsim_frigidaire/robot/grasp.py")}}
    out.write_text(json.dumps(order, indent=1, default=lambda o: o.tolist() if hasattr(o, "tolist") else str(o)) + "\n")
    for n, r_ in gate_rows.items():                       # the sample gate's format: the episode's and --rows' gate.json
        r_["move"] = PL.fifteen_move(r_.get("gate_move"), n in INSERT_SITES,   # a passing plate's drop: I4-wall20-drop
                                     release_of(r_) if kind_of(n) == "plate" and ok_row(n, r_) else None)
        if kind_of(n) == "plate":                         # the pinch spin at the goal (= release.pinch_spin_deg, the gate's)
            r_["pinch_spin_deg"] = r_.get("goal_spin_deg", (r_.get("release") or {}).get("pinch_spin_deg"))
        if r_.get("hold") == "foot-side":                 # the side ring hold's start order as gated (EP.pitch_order patched)
            r_["ring_side_order"] = "straight"
    g_out = {"plan": "plans/2026-11-04-fifteen-of-fifteen.md", "load": rel(out.parent / "load.json"),
             "order": rel(out), "mount_file": rel(a.mount), "basket": CFG["basket"],
             "arm_model": rel(R.ARM_MODEL), "counter_xy": list(R.COUNTER_XY), "dag": a.dag,
             "rule": "each dish's row of the access order (order.json): frigidaire_robot_reach.sample_gate_job with the dishes before it "
                     "in its chain in the scene at their predicted landed poses (a reject: alone); the plates' pinch spin at the goal "
                     "is release.pinch_spin_deg",
             "dishes": {n: gate_rows[n] for n in dag["order"] if n in gate_rows}, "seconds": order["seconds"]}
    (out.parent / "gate.json").write_text(json.dumps(g_out, indent=1, default=lambda o: o.tolist() if hasattr(o, "tolist") else str(o))
                                          + "\n")
    summ = {c: f"{len(v['order'])}/{len(v['order']) + len(v['blocked'])} {v['order']} blocked {list(v['blocked'])}"
            for c, v in chains_out.items()}
    print(f"[RESULT] PASS access order {sid} at {a.mount.name} (--dag {a.dag}, basket {'in' if CFG['basket'] else 'out'}): "
          f"{summ}; plate releases { {n: (v_['mode'], v_['release_mm'], v_['spin_deg']) for n, v_ in order['plate_releases'].items()} }; "
          f"reversed "
          f"{ {c: [f_['edge'] for f_ in v] for c, v in order['reversed'].items()} }; off-goal seats (warning) "
          f"{ {n: (v_['seat_mm'], v_['on_dish']) for n, v_ in order['off_goal_seats'].items()} }; rejects {sorted(rejects)}; "
          f"pairs { {k: v['verdict'] for k, v in pairs.items()} }; {n_jobs} gate jobs in {order['seconds']} s -> "
          f"{rel(out)}, {rel(out.parent / 'gate.json')}", flush=True)


if __name__ == "__main__":
    main()
