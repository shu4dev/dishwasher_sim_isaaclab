"""build stage: the fit-score-prove goal (plans/2026-11-02-fit-score-prove.md, Decisions and its Revision), then its
Isaac build.

Fit (Kit-free, FCL + CP-SAT): an exact packing of the roster over the packer's candidates (frigidaire_hotec_top5 level
families and masks: slots, gaps, leans, tub wall, the closed appliance, the rack-retraction sweep), the lower-rack bowl
zones thinned to the first pose per FIT_CELL_M cell per (slot, tilt family, lift). One Boolean per candidate pose (the
dishes of a kind are the same shape, so a pose literal stands for any dish of its kind): each kind takes exactly its count
of poses, plates and cups one per slot, no two poses that collide past the packer's 0.8 mm touch (FCL, nesting allowed;
cached per family digest next to the packer's masks), gate bans as conflicts. Objective: the sum of isolated exposure
(HX().IsolatedExposure, ascend's ranking) plus seeded noise, a tiebreak; FIT_SOLVE_S per solve. The packing's access and
support DAG (sampler/order.py, packed model) must be acyclic: a cycle adds a nogood over its poses and the model is solved
again, up to FIT_CUTS times. A level the roster does not fit reports the most bowls it can hold beside the other kinds
(the relaxed model), then the next level of the ladder (1 adds the lower_mid bowl zone).

Score (GPU): the coordinate ascent of frigidaire_bench.ascend from the fit: per dish its free alternatives ranked by
isolated exposure, the top 6 plus 2 random scored by the fair S (score.fair: per dish min(1, NJ / ceiling[arm][kind]), 0
when it pools; plain mean; NJ cast by dot_rules over the whole load at N_FULL / GAIN_RAY_FRACTION rays, the 6.2
ceilings), 2 sweeps; the best move of a dish is kept when S rises by > 1e-4 and the load's DAG stays acyclic. The chosen
load is scored once at the full budget by the score stage (S_ref).

Prove: the DAG's topological order is the build order (frigidaire_dependency_kit.py --load --order, by subprocess: the
insertion move one dish at a time, re-posing on a physical miss and deferral (strict false), the joint gate's 1 mm
overlap rule after each drop; plate_order "lean-side" refills each bank's plates in chain order, which the DAG order's
tie rule already follows; when that refill would break an edge the file says "dag" and the kit keeps the order as it
is). Then gate_goal, certify, score and graph. The goal block comes from the build record (sample.goal_of), the maker
summary goes to metadata.maker; the full maker record and the packed entries stay in <work>/goal/loads/<sample>.json.
"""
import importlib.util
import json
from pathlib import Path
import time

from . import ROOT, rel, run
from .scorer import score as SC
from .difficulty import roster
from .sample import MAKER_KEYS, goal_of

DEP_KIT = "code/sampler/scripts/frigidaire_dependency_kit.py"
GAIN_RAY_FRACTION = 16          # candidates ranked at about 1/16 of the score stage's rays (Decisions, plan grill)
KINDS = ("plate", "bowl", "cup")
FIT_CELL_M = .020               # lower-rack bowls: the first pose per 2 cm cell per (slot, tilt family, lift)
FIT_SOLVE_S = 60.               # CP-SAT time limit per solve
FIT_CUTS = 20                   # DAG-cycle nogoods per level
FIT_LEVELS = (0, 1)             # the packer's ladder: 1 admits the lower_mid bowl zone (frigidaire_hotec_top5.LEVELS)
FIT_WORKERS = 2                 # CP-SAT search workers (a roster runs on one core pair)


def restricted():
    """frigidaire_dot_rules_restricted (its top-5 module T and T's frigidaire_bench copy, with the packer's tolerant
    dish set and nesting allowed)."""
    spec = importlib.util.spec_from_file_location("restricted", ROOT / "code/sampler/scorer/scripts/frigidaire_dot_rules_restricted.py")
    R = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(R)
    return R


def packer(R, level=0):
    """(fam, masks, parts, points, bans): the packer's families, masks and gate bans at ``level``, kept on ``R``."""
    cache = R.__dict__.setdefault("_fsp_packer", {})
    if level not in cache:
        T = R.T
        fam, parts, points = T.level_families(level)
        cache[level] = (fam, T.level_masks(fam, parts, points, level), parts, points, T.all_bans())
    return cache[level]


def tilt_of(c):
    """The tilt token of a family candidate's variant: lean-24, X135, Y150, tilt100, lean0 ..."""
    return str(c["variant"]).split("_")[0]


class GainScorer:
    """The fair S of a partial load, NJ cast at ``n_rays`` (dot_rules: J lines only, as the restricted scorer), the
    racks and basket soup built once."""

    def __init__(self, B, ceil, n_rays, theta=SC.THETA_DEG):
        import numpy as np
        B.HX().register_hotec(B.ASSETS)
        from sampler.scorer import dot_rules as DR
        from sampler.scorer import exposure as E
        from asset.twin import BODY_POSITIONS
        self.np, self.DR, self.E, self.ceil, self.theta = np, DR, E, ceil, float(theta)
        self.eng = DR.DotRuleEngine(int(n_rays))
        self.basket = (np.asarray(BODY_POSITIONS["SilverwareBasket"], dtype=float), np.asarray(E.IDENTITY, dtype=float))
        self.static = E.occluder_soup(E.Arrangement("load", "", "", [], self.basket), appliance=True)
        self.calls, self.seconds = 0, 0.

    def __call__(self, entries):
        if not entries:
            return 0.
        t0 = time.perf_counter()
        DR, E, np = self.DR, self.E, self.np
        dots, owner, recs, objs = DR.load_dots(entries)
        soup = np.concatenate([self.static, E.occluder_soup(E.Arrangement("load", "", "", objs, self.basket), appliance=False)])
        mesh = self.eng.build_mesh(soup)               # = DR.load_soup(entries): racks, basket, then the dishes
        sums = self.eng.ray_sums(mesh, dots, [self.theta], fixed=False)
        WF, WG, WJ, _, _ = self.eng.wstar(dots, sums, [self.theta])
        nj = self.eng.scores(sums, WF, WG, WJ)[0]["NJ"][:, 0].cpu().numpy()
        del mesh
        per = [{"id": r["id"], "kind": r["kind"], "arm": r["arm"], "area_m2": r["area_m2"], "NJ": float(nj[owner == i].mean())}
               for i, r in enumerate(recs)]
        s = SC.fair(per, [r["id"] for r in recs if r["pools"]], self.ceil)["S"]
        self.calls += 1
        self.seconds += time.perf_counter() - t0
        return s


# --------------------------------------------------------------------------- fit: candidates and conflicts

def fit_candidates(R, level):
    """[(kind, family index)]: every masked pose, the lower-rack bowls thinned to the first per FIT_CELL_M cell per
    (slot, tilt family, lift) in family order (seed-free, so the conflicts cache per family digest)."""
    import numpy as np
    fam, masks, _, _, _ = packer(R, level)
    out, cells = [], set()
    for kind in KINDS:
        for i, ((c, _), ok) in enumerate(zip(fam[kind], masks[kind])):
            if not ok:
                continue
            if kind == "bowl" and c["rack"] == "LowerRack":
                v = str(c["variant"])
                cell = (c["slot"], v.split("_")[0], v.rsplit("_lift", 1)[-1],
                        int(np.floor(c["position"][0] / FIT_CELL_M + 1e-9)), int(np.floor(c["position"][1] / FIT_CELL_M + 1e-9)))
                if cell in cells:
                    continue
                cells.add(cell)
            out.append((kind, i))
    return out


_W = {}


def _conflict_init(level, cands):
    _W.update(R=restricted(), level=level, cands=cands)


def _conflict_chunk(chunk):
    """The pairs of ``chunk`` (candidate index pairs) that collide past the packer's touch (TolerantDishSet)."""
    R, cands = _W["R"], _W["cands"]
    fam, _, parts, points, _ = packer(R, _W["level"])
    out = []
    for a, b in chunk:
        d = R.T.TolerantDishSet(parts, points)
        d.add("a", fam[cands[a][0]][cands[a][1]][0])
        if d.collides(fam[cands[b][0]][cands[b][1]][0]):
            out.append((a, b))
    return out


def fit_conflicts(R, level, log=print, workers=None):
    """(cands, pairs, seconds): fit_candidates and the index pairs that collide past the packer's 0.8 mm touch
    (TolerantDishSet: FCL contacts, nesting allowed), AABB-prefiltered, cached per family digest in the packer's mask
    folder (frigidaire_hotec_top5 OUT/cache/level<L>/fit_conflicts.json). Computed in a spawn pool (FCL objects do not
    pickle, and the build process may hold a CUDA context)."""
    import multiprocessing as mp
    import os
    import numpy as np
    T, B = R.T, R.T.B
    fam, _, parts, points, _ = packer(R, level)
    key = f"{B.family_digest(fam)}|sweep{B.SWEEP_STEPS}|tol{T.DISH_TOL_M}|cell{FIT_CELL_M}"
    path = T.OUT / "cache" / f"level{level}" / "fit_conflicts.json"
    if path.is_file():
        data = json.loads(path.read_text())
        if data.get("key") == key:
            return [tuple(c) for c in data["cands"]], [tuple(p) for p in data["pairs"]], data.get("seconds")
    t0 = time.monotonic()
    cands = fit_candidates(R, level)
    d = T.TolerantDishSet(parts, points)
    lo, hi = map(np.array, zip(*(d._bounds(fam[k][i][0]) for k, i in cands)))
    near = np.all((lo[:, None, :] <= hi[None, :, :]) & (lo[None, :, :] <= hi[:, None, :]), axis=2)
    a, b = np.nonzero(np.triu(near, 1))
    todo = list(zip(a.tolist(), b.tolist()))
    workers = workers or max(1, min(8, (os.cpu_count() or 2)))
    log(f"  fit conflicts, level {level}: {len(cands)} poses, {len(todo)} AABB pairs, {workers} workers")
    chunks = [todo[k:k + 2000] for k in range(0, len(todo), 2000)]
    with mp.get_context("spawn").Pool(workers, initializer=_conflict_init, initargs=(level, cands)) as pool:
        pairs = sorted(p for part in pool.imap_unordered(_conflict_chunk, chunks) for p in part)
    seconds = round(time.monotonic() - t0, 1)
    T.atomic_write(path, json.dumps({"key": key, "cands": cands, "pairs": pairs, "aabb_pairs": len(todo),
                                     "seconds": seconds}) + "\n")
    log(f"  fit conflicts, level {level}: {len(pairs)} colliding pairs in {seconds} s -> {rel(path)}")
    return cands, pairs, seconds


def _edge_init(level, cands):
    from motion_planning import planner as P
    from .order import BASELINE
    R = restricted()
    B = R.T.B
    now = json.loads(BASELINE.read_text())["snapshot"]["poses"]
    checker = B.release_checker()
    _W.update(R=R, level=level, cands=cands, P=P, now=now, checker=checker, appliance=B.appliance_bodies(checker, now),
              heights={r: B.rack_heights(now, r) for r in ("LowerRack", "UpperRack")}, at={}, low={})


def _edge_entry(n):
    R = _W["R"]
    k, i = _W["cands"][n]
    return R.T.B.entry_of(f"c{n}", packer(R, _W["level"])[0][k][i][0])


def _edge_bodies(n):
    """(packed body, body lowered by the support drop) of candidate n in the baseline's extended racks (cached)."""
    if n not in _W["at"]:
        from .order import SUPPORT_GAP_M
        B, P, checker, e = _W["R"].T.B, _W["P"], _W["checker"], _edge_entry(n)
        pose = P.world_from_local(_W["now"][e["rack"]], {"position_m": list(e["position"]),
                                                         "quaternion_xyzw": list(e["quaternion_xyzw"])})
        _W["at"][n] = checker._body(e["kind"], pose, n)
        _W["low"][n] = checker._body(e["kind"], B.lifted(pose, -(SUPPORT_GAP_M + checker.penetration_limit_m),
                                                         (0., 0., 1.)), n)
    return _W["at"][n], _W["low"][n]


def _edge_chunk(task):
    """The pairwise DAG edges (a before b) of sampler/order.py for each path owner a of ``task`` = [(a, [b for its
    path], [b for support])]: a's insertion path (against the appliance alone) meets b's packed pose, or b rests on a."""
    B, P, checker = _W["R"].T.B, _W["P"], _W["checker"]
    out = []
    for a, path_bs, sup_bs in task:
        e = _edge_entry(a)
        move = P.world_from_local(_W["now"][e["rack"]], B.move_local(e))
        up, rim_z, top_z = _W["heights"][e["rack"]]
        m = B.insertion_release(checker, [(e["kind"], move, a)], [], _W["appliance"], up, rim_z, top_z)
        clear = [] if m["blocked"] == "start" else m["lifts"][:-1] if m["blockers"] else m["lifts"]
        steps = [checker._body(e["kind"], B.lifted(move, h, up), a) for h in clear]
        for b in path_bs:
            at_b = _edge_bodies(b)[0]
            if any(not checker.pair(s, at_b)["valid"] for s in steps):
                out.append((a, b))
        at_a = _edge_bodies(a)[0]
        for b in sup_bs:
            at_b, low_b = _edge_bodies(b)
            if checker.pair(at_b, at_a)["valid"] and not checker.pair(low_b, at_a)["valid"]:
                out.append((a, b))
    return out


def fit_cycles(R, level, cands, pairs, log=print, workers=None):
    """(two-cycles, edges, seconds): the candidate pairs whose sampler/order.py edges run both ways (each one's insertion
    path meets the other's packed pose, or one rests on the other: no insertion order for the two), for the pairs that
    do not collide; such edges depend on the two poses alone, so these are exact pairwise conflicts and the lazy DAG cuts
    are left with the longer cycles. AABB-prefiltered on the checker's bounds (the path swept from its move pose up to its
    insertion start), cached like fit_conflicts (fit_cycles.json)."""
    import multiprocessing as mp
    import os
    import numpy as np
    from motion_planning import planner as P
    from .order import BASELINE
    T, B = R.T, R.T.B
    fam = packer(R, level)[0]
    key = f"{B.family_digest(fam)}|sweep{B.SWEEP_STEPS}|tol{T.DISH_TOL_M}|cell{FIT_CELL_M}|insert{B.INSERT_ABOVE_RIM_M}"
    path = T.OUT / "cache" / f"level{level}" / "fit_cycles.json"
    if path.is_file():
        data = json.loads(path.read_text())
        if data.get("key") == key:
            return [tuple(c) for c in data["cycles"]], data["edges"], data.get("seconds")
    t0 = time.monotonic()
    now = json.loads(BASELINE.read_text())["snapshot"]["poses"]
    checker = B.release_checker()
    heights = {r: B.rack_heights(now, r) for r in ("LowerRack", "UpperRack")}

    def box(kind, pose):
        return P_box(checker, kind, pose)

    pose_lo, pose_hi, path_lo, path_hi = [], [], [], []
    for k, i in cands:
        e = B.entry_of("x", fam[k][i][0])
        f = now[e["rack"]]
        pose = P.world_from_local(f, {"position_m": list(e["position"]), "quaternion_xyzw": list(e["quaternion_xyzw"])})
        move = P.world_from_local(f, B.move_local(e))
        up, rim_z, top_z = heights[e["rack"]]
        along = B.posed_points(B.collision_points(k), move["position_m"], move["quaternion_xyzw"]) @ up
        start = rim_z + B.INSERT_ABOVE_RIM_M - float(along.min())
        start = max(min(start, top_z - float(along.max())), 0.)
        lo, hi = box(k, pose)
        mlo, mhi = box(k, move)
        top = np.asarray(up) * start
        pose_lo.append(lo), pose_hi.append(hi)
        path_lo.append(np.minimum(mlo, mlo + top)), path_hi.append(np.maximum(mhi, mhi + top))
    pose_lo, pose_hi, path_lo, path_hi = map(np.array, (pose_lo, pose_hi, path_lo, path_hi))
    collide = np.zeros((len(cands), len(cands)), dtype=bool)
    for a, b in pairs:
        collide[a, b] = collide[b, a] = True
    np.fill_diagonal(collide, True)
    reach = np.all((path_lo[:, None, :] <= pose_hi[None, :, :]) & (pose_lo[None, :, :] <= path_hi[:, None, :]), axis=2) & ~collide
    m = .003
    touch = np.all((pose_lo[:, None, :] - m <= pose_hi[None, :, :]) & (pose_lo[None, :, :] - m <= pose_hi[:, None, :]), axis=2) & ~collide
    # a two-cycle needs an edge each way: keep a's targets b only where b could also come back to a
    back = reach | touch
    need = back & back.T
    tasks = [(a, np.nonzero(reach[a] & need[a])[0].tolist(), np.nonzero(touch[a] & need[a])[0].tolist())
             for a in range(len(cands))]
    tasks = [t for t in tasks if t[1] or t[2]]
    workers = workers or max(1, min(8, (os.cpu_count() or 2)))
    log(f"  fit cycles, level {level}: {len(tasks)} path owners, {int(sum(len(t[1]) for t in tasks))} path and "
        f"{int(sum(len(t[2]) for t in tasks))} support checks, {workers} workers")
    chunks = [tasks[k::workers * 4] for k in range(workers * 4)]
    with mp.get_context("spawn").Pool(workers, initializer=_edge_init, initargs=(level, cands)) as pool:
        edges = {e for part in pool.imap_unordered(_edge_chunk, chunks) for e in part}
    cycles = sorted({(min(a, b), max(a, b)) for a, b in edges if (b, a) in edges})
    seconds = round(time.monotonic() - t0, 1)
    T.atomic_write(path, json.dumps({"key": key, "cycles": cycles, "edges": len(edges), "seconds": seconds}) + "\n")
    log(f"  fit cycles, level {level}: {len(edges)} pairwise edges, {len(cycles)} two-cycles in {seconds} s -> {rel(path)}")
    return cycles, len(edges), seconds


def P_box(checker, kind, pose):
    """The checker's AABB of a dish body at a world pose (InitialCollisionChecker._body's bounds)."""
    import numpy as np
    from sampler.initial_state_candidates import _corners, _read_pose
    p, _, orient = _read_pose(pose)
    corners = _corners(*checker.bounds[kind]) @ orient.T + p
    return corners.min(axis=0), corners.max(axis=0)


# --------------------------------------------------------------------------- fit: CP-SAT

def fit_solve(R, level, count, cands, pairs, weights, cuts, seed, relax=False, time_limit=FIT_SOLVE_S, extra_bans=(),
              key_cuts=()):
    """(status, chosen candidate indices or None): the CP-SAT model of the module docstring over the roster's kinds;
    ``relax`` lets the bowls fall short and maximizes their number instead of the exposure tiebreak. ``extra_bans`` (pose-key
    frozensets of one or two keys, as the gate bans) and ``key_cuts`` (lists of pose keys, never all together) are the
    hardware-aware loop's cuts (sampler/hwgate.py)."""
    from collections import defaultdict
    from ortools.sat.python import cp_model
    fam, _, _, _, bans = packer(R, level)
    bans = [*bans, *extra_bans]
    B = R.T.B
    use = [n for n, (k, _) in enumerate(cands) if count.get(k, 0) > 0]
    m = cp_model.CpModel()
    y = {n: m.NewBoolVar(f"y{n}") for n in use}
    for k in KINDS:
        if count.get(k, 0) > 0:
            total = sum(y[n] for n in use if cands[n][0] == k)
            if relax and k == "bowl":
                m.Add(total <= count[k])
            else:
                m.Add(total == count[k])
    slots = defaultdict(list)
    for n in use:
        k, i = cands[n]
        if k != "bowl":
            slots[(fam[k][i][0]["rack"], fam[k][i][0]["slot"])].append(n)
    for ns in slots.values():
        if len(ns) > 1:
            m.AddAtMostOne(y[n] for n in ns)
    for a, b in pairs:
        if a in y and b in y:
            m.AddBoolOr([y[a].Not(), y[b].Not()])
    if bans or key_cuts:                                      # gate bans: a banned pose, or a banned pair of poses
        keyed = defaultdict(list)
        for n in use:
            for kk in B.cand_keys(fam[cands[n][0]][cands[n][1]][0]):
                keyed[kk].append(n)
        for ban in bans:
            ks = list(ban)
            if len(ks) == 1:
                for n in keyed.get(ks[0], ()):
                    m.Add(y[n] == 0)
            else:
                for a in keyed.get(ks[0], ()):
                    for b in keyed.get(ks[1], ()):
                        if a != b:
                            m.AddBoolOr([y[a].Not(), y[b].Not()])
        for kc in key_cuts:                                   # a no-good over poses (or pose groups: any pose of a group
            zs = []                                           # occupies it): never every one of them at once
            for kk in kc:
                ns = sorted({n for k_ in ([kk] if isinstance(kk, str) else kk) for n in keyed.get(k_, ())})
                if not ns:                                    # a pose of the no-good absent at this level: it cannot bind
                    zs = None
                    break
                z = m.NewBoolVar(f"z{len(zs)}")
                m.AddMaxEquality(z, [y[n] for n in ns])
                zs.append(z)
            if zs:
                m.AddBoolOr([z.Not() for z in zs])
    for cut in cuts:
        m.AddBoolOr([y[n].Not() for n in cut])
    if relax:
        m.Maximize(sum(y[n] for n in use if cands[n][0] == "bowl"))
    else:
        m.Maximize(sum(int(weights[n]) * y[n] for n in use))
    s = cp_model.CpSolver()
    s.parameters.max_time_in_seconds = float(time_limit)
    s.parameters.num_workers = FIT_WORKERS
    s.parameters.random_seed = int(seed) % (2 ** 31)
    status = s.StatusName(s.Solve(m))
    chosen = [n for n in use if s.Value(y[n])] if status in ("OPTIMAL", "FEASIBLE") else None
    return status, chosen


def fit_pack(count, seed, R=None, log=print, isolated=None, level=0, extra_bans=(), key_cuts=(), relaxed=True):
    """(entries or None, info): the CP-SAT fit of the roster ``count`` at ``level`` with the lazy DAG cuts (module
    docstring). ``isolated`` (candidates -> isolated exposure, Warp) weights the tiebreak; without it only the seeded
    noise orders the optima (the Mac's Kit-free proof). info = {level, poses, conflicts, conflicts_s, cuts, solves,
    status, solve_s, dag_s, max_bowls (when the roster does not fit), seconds}."""
    import numpy as np
    from . import order as O
    R = R or restricted()
    B = R.T.B
    t0 = time.monotonic()
    fam = packer(R, level)[0]
    cands, pairs, conflicts_s = fit_conflicts(R, level, log)
    cycles, edges, cycles_s = fit_cycles(R, level, cands, pairs, log)
    rng = np.random.default_rng([int(seed), 7])
    noise = rng.integers(0, 10, len(cands))
    iso = np.zeros(len(cands)) if isolated is None else np.asarray(isolated([fam[k][i][0] for k, i in cands]))
    weights = np.round(1000. * iso).astype(int) + noise
    info = {"level": level, "poses": sum(count.get(k, 0) > 0 for k, _ in cands), "conflicts": len(pairs),
            "conflicts_s": conflicts_s, "two_cycles": len(cycles), "pair_edges": edges, "cycles_s": cycles_s,
            "cuts": 0, "solves": 0, "status": None, "solve_s": 0., "dag_s": 0.,
            "max_bowls": None, "tiebreak": "isolated exposure + noise" if isolated is not None else "noise"}
    pairs = pairs + cycles                                    # two-cycles are pairwise conflicts like collisions
    cuts = []
    while True:
        t1 = time.monotonic()
        status, chosen = fit_solve(R, level, count, cands, pairs, weights, cuts, seed, extra_bans=extra_bans,
                                   key_cuts=key_cuts)
        info["solves"] += 1
        info["solve_s"] = round(info["solve_s"] + time.monotonic() - t1, 1)
        info["status"] = status
        if chosen is None:
            break
        entries, pose_of = [], {}
        for k in KINDS:                                       # the kind's dishes, in a seeded order, onto its poses
            mine = [n for n in chosen if cands[n][0] == k]
            ids = [o for o, kk in roster(count) if kk == k]
            for oid, j in zip([ids[j] for j in rng.permutation(len(ids))], rng.permutation(len(mine))):
                entries.append(B.entry_of(oid, fam[k][cands[mine[j]][1]][0]))
                pose_of[oid] = mine[j]
        t2 = time.monotonic()
        dag = O.dag_order(entries, seed)
        info["dag_s"] = round(info["dag_s"] + time.monotonic() - t2, 1)
        if dag["order"] is not None:
            info["pose_key"] = {oid: B.cand_key(fam[cands[n][0]][cands[n][1]][0]) for oid, n in pose_of.items()}
            info.update(seconds=round(time.monotonic() - t0, 1), dag={"access": len(dag["access"]),
                        "support": len(dag["support"]), "layers": dag["layers"]})
            log(f"  fit level {level}: packed at solve {info['solves']} ({info['cuts']} DAG cuts), {info['poses']} poses, "
                f"{info['conflicts']} conflicts, {info['seconds']} s")
            return entries, info
        if info["cuts"] >= FIT_CUTS:
            info["status"] = "cuts exhausted"
            break
        cuts.append(sorted({pose_of[i] for i in dag["cycle"]}))      # never all of these poses together again
        info["cuts"] += 1
        log(f"  fit level {level}: DAG cycle {'>'.join(dag['cycle'])}, cut {info['cuts']}")
    if info["status"] != "cuts exhausted" and relaxed:        # the roster does not fit: how many bowls do
        t1 = time.monotonic()
        status, chosen = fit_solve(R, level, count, cands, pairs, weights, cuts, seed, relax=True)
        info["max_bowls"] = None if chosen is None else sum(cands[n][0] == "bowl" for n in chosen)
        info["relaxed_status"] = status
        info["solve_s"] = round(info["solve_s"] + time.monotonic() - t1, 1)
    info["seconds"] = round(time.monotonic() - t0, 1)
    log(f"  fit level {level}: no packing ({info['status']}); max bowls {info['max_bowls']} of {count.get('bowl', 0)}, "
        f"{info['seconds']} s")
    return None, info


# --------------------------------------------------------------------------- score and prove

def ascend_pack(entries, R, score, seed, log=print, isolated=None, level=0):
    """(entries, info): frigidaire_bench.ascend from the fit with the gain scorer ``score`` (entries -> S) and the DAG
    hook (module docstring). info = {S_first_fit, S_ascent, sweeps, stop, history, accept: {calls, refused, seconds}}."""
    import numpy as np
    from . import order as O
    B = R.T.B
    fam, masks, parts, points, bans = packer(R, level)
    if isolated is None:
        from sampler.scorer import exposure as E
        isolated = B.HX().IsolatedExposure(E.DEVICE, 200, 32).evaluate
    acc = {"calls": 0, "refused": 0, "seconds": 0.}

    def accept(trial):
        t0 = time.monotonic()
        ok = O.dag_order(trial, seed)["order"] is not None
        acc["calls"] += 1
        acc["refused"] += not ok
        acc["seconds"] += time.monotonic() - t0
        return ok

    S = B.SEARCH
    out, current, history, sweeps, stop = B.ascend(
        [dict(e) for e in entries], fam, masks, parts, points, lambda es: {"feasible": True, "score": score(es)},
        isolated, np.random.default_rng([int(seed), 1]), max_sweeps=S["max_sweeps"], top=S["top"],
        n_random=S["random"], log=log, bans=bans, accept=accept)
    acc["seconds"] = round(acc["seconds"], 1)
    return out, {"S_first_fit": round(history[0]["score"], 5), "S_ascent": round(current["score"], 5), "sweeps": sweeps,
                 "stop": stop, "history": history, "accept": acc}


def plan_goal(count, seed, R, score, log=print, isolated=None, levels=FIT_LEVELS):
    """(entries or None, dag or None, info): the fit at the first level of the ladder that packs the roster, the ascent
    from it, then the final DAG (acyclic by construction; a cycle would fall back to the fit's packing)."""
    from . import order as O
    t0, tried = time.monotonic(), []
    info = {"method": "fit-score-prove", "seed": seed, "levels": tried}
    for level in levels:
        entries, fit = fit_pack(count, seed, R, log, isolated=isolated, level=level)
        tried.append(fit)
        if entries is not None:
            break
    else:
        return None, None, {**info, "level": None, "seconds": round(time.monotonic() - t0, 1)}
    out, asc = ascend_pack(entries, R, score, seed, log, isolated, level)
    t1 = time.monotonic()
    dag = O.dag_order(out, seed)
    if dag["order"] is None:                                  # cannot happen: every committed move kept the DAG acyclic
        log(f"  final DAG has a cycle {dag['cycle']}: the fit's packing is kept")
        asc["dropped"] = {"cycle": dag["cycle"]}
        out, dag = entries, O.dag_order(entries, seed)
    info.update(asc, level=level, fit=fit, dag_seconds=round(time.monotonic() - t1, 1),
                seconds=round(time.monotonic() - t0, 1))
    return out, dag, info


def plate_order_of(dag, entries):
    """"lean-side" when the kit's chain refill (frigidaire_bench_kit.chain_order) of the DAG order keeps every edge,
    else "dag" (the kit then keeps the order as it is)."""
    from . import order as O
    B, K, _, _ = O._mods()
    chained = K.chain_order(dag["order"], {e["id"]: e for e in entries}, B.lean_side)[0]
    pos = {i: n for n, i in enumerate(chained)}
    return "lean-side" if all(pos[a] < pos[b] for a, b in [*dag["access"], *dag["support"]]) else "dag"


def stage_fit(state, params):
    """The Kit-free half of the old build stage: the goal's packing (plan_goal: the fit and the score ascent; --hardware:
    sampler/hwgate.py's lazy loop, no ascent) and its loads files <work>/goal/loads/<id>.json and <id>.order.json, which the
    build stage builds in Isaac."""
    meta = state["metadata"]
    prov = meta["provenance"]
    work = Path(params["work"]) / "goal"
    load_id = Path(params["work"]).name
    R = restricted()
    t0 = time.monotonic()
    for key in ("goal_record", "gate_goal", "certify", "score", "hwgate"):   # a new goal: the downstream records are gone
        prov.pop(key, None)
    prov["load_id"], state["goal"], state["dag"] = load_id, None, None
    hw_note = None
    if params.get("hardware"):
        from . import hwgate
        entries, dag, info = hwgate.hw_goal(meta["params"]["count"], meta["seed"], R, ROOT / params["hardware"],
                                            Path(params["work"]), basket=params.get("hardware_basket", "in") != "out",
                                            max_iter=int(params.get("hardware_max_iter") or hwgate.HW_MAX_ITER))
        prov["hwgate"] = info["hwgate"]
        meta["maker"] = {k: info.get(k) for k in MAKER_KEYS}
        last = json.loads((ROOT / info["order_file"]).read_text()) if info.get("order_file") else None
        if last is None and info["hwgate"]["iterations"]:      # the last gated iteration's rows for the counter picks
            k_ = max((it["k"] for it in info["hwgate"]["iterations"] if it.get("level") is not None), default=None)
            f_ = Path(params["work"]) / "hwgate" / f"iter_{k_}" / "order.json"
            last = json.loads(f_.read_text()) if k_ is not None and f_.is_file() else None
        _, hw_note = hwgate.initial_check(state, params, last)
        its = info["hwgate"]["iterations"]
        fits = (f"{len(its)} iteration(s): " + "; ".join(
            f"{it['k']}: level {it['level']}, failed {sorted(it['failed'])}, cuts {[c['type'] for c in it['cuts']]}" for it in its))
        if entries is None:
            return state, "FAIL", f"hwgate: {info.get('stop')} ({fits}); {hw_note}"
    else:
        ceil = SC.ceilings(params.get("ceilings", SC.CEILINGS), params.get("ceilings_wait_s", 0.))
        n_rays = SC.N_FULL // GAIN_RAY_FRACTION
        score = GainScorer(R.T.B, ceil, n_rays)
        from sampler.scorer import exposure as E
        isolated = R.T.B.HX().IsolatedExposure(E.DEVICE, 200, 32).evaluate      # one cache for the fit and the ascent
        entries, dag, info = plan_goal(meta["params"]["count"], meta["seed"], R, score, isolated=isolated)
        info.update(rays_N=n_rays, theta_deg=SC.THETA_DEG, ceilings=rel(params.get("ceilings", SC.CEILINGS)),
                    score_calls=score.calls, score_seconds=round(score.seconds, 1), seconds=round(time.monotonic() - t0, 1),
                    S_planned=info.get("S_ascent") if entries else None)
        fits = "; ".join(f"level {f['level']}: {f['poses']} poses, {f['conflicts']} conflicts, {f['cuts']} DAG cuts, "
                         f"{f['seconds']} s" + ("" if f.get("dag") else f", no packing ({f['status']}, max bowls "
                                                                           f"{f['max_bowls']})") for f in info["levels"])
        meta["maker"] = {k: info.get(k) for k in MAKER_KEYS}
        if entries is None:
            return state, "FAIL", f"fit: no packing at any level ({fits})"
    info["dag"] = {"access": dag["access"], "support": dag["support"], "layers": dag["layers"], "order": dag["order"]}
    info["plate_order"] = plate_order_of(dag, entries)
    loads = work / "loads"
    loads.mkdir(parents=True, exist_ok=True)
    (loads / f"{load_id}.json").write_text(json.dumps({"load_id": load_id, "seed": meta["seed"], "order": dag["order"],
                                                        "entries": entries, "maker": info}, indent=1) + "\n")
    (loads / f"{load_id}.order.json").write_text(json.dumps({"order": dag["order"], "plate_order": info["plate_order"],
                                                              "strict": False, "pose_model": "packed",
                                                              "source": "fit-score-prove (code/sampler/build.py): the DAG "
                                                                        "order"}, indent=1) + "\n")
    if params.get("hardware"):
        return state, "PASS", (f"hwgate {fits}; {info['hwgate'].get('direct')}; DAG {len(dag['access'])} access + "
                               f"{len(dag['support'])} support edges, {dag['layers']} layers, plate order "
                               f"{info['plate_order']}; {hw_note} -> {rel(loads / (load_id + '.json'))}")
    return state, "PASS", (
        f"fit {fits}; ascent {info['sweeps']} sweeps S {info['S_first_fit']:.4f} -> {info['S_ascent']:.4f} (N {n_rays}), "
        f"{score.calls} casts in {score.seconds:.0f} s; DAG {len(dag['access'])} access + {len(dag['support'])} support "
        f"edges, {dag['layers']} layers, plate order {info['plate_order']}")


def stage_build(state, params):
    """The Isaac build of the fit stage's loads files (frigidaire_dependency_kit.py --load --order); the goal block from its
    record (sample.goal_of), goal.robot from a hardware-aware fit."""
    meta = state["metadata"]
    prov = meta["provenance"]
    work = Path(params["work"]) / "goal"
    load_id = Path(params["work"]).name
    loads = work / "loads"
    lf, order_file = loads / f"{load_id}.json", loads / f"{load_id}.order.json"
    if not (lf.is_file() and order_file.is_file()):
        return state, "FAIL", f"no fit: {rel(lf)} or its order file is missing (run the fit stage)"
    info = json.loads(lf.read_text())["maker"]
    for key in ("goal_record", "gate_goal", "certify", "score"):     # a new build: the downstream records are gone
        prov.pop(key, None)
    prov["load_id"], state["goal"], state["dag"] = load_id, None, None
    ok, _ = run("kit", [DEP_KIT, "--load", load_id, "--restricted-dir", rel(loads), "--order", rel(order_file),
                        "--out-dir", rel(work), "--headless", "--force"],       # --force: a rerun replaces the last build
                work / "logs" / "build.log")
    record = work / f"{load_id}.json"
    dep = json.loads(record.read_text()) if record.is_file() else {}
    prov["goal_record"] = rel(record)
    if dep.get("targets_world"):
        state["goal"] = goal_of(dep, json.loads((ROOT / prov["initial_record"]).read_text()))
        if info.get("robot"):
            state["goal"]["robot"] = info["robot"]
    return state, "PASS" if ok and dep.get("status") == "ok" else "FAIL", (
        f"build of the {info.get('method')} fit (level {info.get('level')}, {len(info['dag']['order'])} dishes, plate order "
        f"{info.get('plate_order')}): status {dep.get('status')}" + (f"; {dep.get('reason')}" if dep.get("status") != "ok" else ""))
