#!/usr/bin/env python3
# Copyright (c) 2026, dishsim project.
# SPDX-License-Identifier: BSD-3-Clause
"""Relative S (2026-10-14, research/2026-10-14-newj-curve-and-leaning-pairs.md; a scored column, not adopted).

A dish's reference is the same engine run on that dish alone (its own visual mesh the only blocker): axis straight
down (tilt 0, the grid's "facing the arm"), its food-contact centroid where it sits in the load relative to its own
arm (centre and radius from the score record), New+J at the record's theta and N. relative = New+J / max(reference,
FLOOR = 0.05), and S_rel = the area-weighted mean of relative over the load's dishes, like S. Kit-free, GPU (Warp).

Reads dot_rules.score_load records (per_dish with kind, arm, area_m2, position_m, quaternion_xyzw, E) and writes
<name>.relative.json beside each input, never touching the score files:
  --loads-dir DIR          <id>.json farm loads (one state, "settled")
  --dependency-dir DIR     <id>.scores.json dependency loads (states packed / settled / built when present)
  --restricted-dir DIR     DIR/scores_N0262144/<id>.json restricted sampler loads (one state, "packed"; New+J only)

    code/util/run_py.sh code/planner/frigidaire/frigidaire_dot_rules_relative.py \\
        --loads-dir data/results/dot_rules/loads/N0262144 --dependency-dir data/results/dot_rules/dependency/pairs
Prints a check line first: the plate / bowl / cup reference at 14 cm over the lower arm's centre, theta 30 (the grid
gives 0.814 / 0.504 / 0.183 there), then [RESULT] PASS with the counts and the Spearman rank correlation of S_rel
against S_NJ over the farm loads.

--reference best (2026-10-16, plans/2026-10-16-object-level-rescale.md): the reference becomes the best the dish alone
reaches at its spot over tilt = the facing reference above times the grid's tilt gain at the nearest (height, distance,
theta) cell of --grid-obj-dir (max over tilt of <kind>_obj New+J over its 0 deg value; the grid's lower-arm disc, assumed
to carry over to the middle arm). relative_best = min(1, New+J / max(facing x gain, FLOOR)): clipped at 1 per dish
(decided 2026-10-16; the grid has no inward tilts, so a few cups beat their reference), the unclipped ratio kept as
relative_best_unclipped. S_rel_best = the area-weighted mean of the clipped values (S_rel_best_unclipped: of the
unclipped ones), written next to the existing fields of every .relative.json (a load without one gets its facing
reference first). A cell whose 0 deg pose is skipped or scores 0, or a kind with no _obj grid, keeps gain 1
(gain_fallback). Standard library only when every load already has its facing reference. --summary prints the Spearman
of S_rel_best against S_NJ over the farm loads, their top and bottom five, and the floored / fallback / clipped counts
per group.

--reference kind (2026-10-21, plans/2026-10-21-kind-ceiling-score.md): one ceiling per kind = the largest <kind>_obj
New+J of the grid over every tilt, height and distance at the records' theta (lower-arm disc, used for the middle arm too),
read from --grid-obj-dir or from the window.GRID payload of a saved Dot Score Rules page (--grid-payload). Per dish
S_kind = min(1, NJ / ceiling[kind]), 0 for a dish that pools (the pooling list of the score record beside the
.relative.json, exposure.pools: interior more than 2 mm below the rim; a state whose score record is not there is scored
with pooling NOT checked, n_pooling None, and the [RESULT] says PARTIAL); ratio_kind = NJ / ceiling unclipped. Per state
S = the plain mean of S_kind, S_area = its area-weighted mean ("surface reached"), S_by_kind, n_pooling, n_clipped_kind;
written next to the existing fields of every saved .relative.json of the given dirs. Standard library only. --summary
prints the ceilings, the Spearman of S and S_area against S_NJ and against S_rel_best, the farm top and bottom five and
the pooling / clipped counts.
"""
from __future__ import annotations

import argparse
import datetime as dt
import json
import os
from pathlib import Path
import statistics
import sys
import time

ROOT = Path(__file__).resolve().parents[3]
sys.path[:0] = [str(ROOT / "code/src"), str(ROOT / "code/frigidaire/src"), str(Path(__file__).resolve().parent)]
try:                          # the facing pass needs numpy and Warp (the box); the best / kind passes are stdlib only
    import numpy as np
    import frigidaire_bench as B  # noqa: E402
except ImportError:
    np = B = None

FLOOR = .05
BATCH = 256                   # dishes per engine call (500 dots each)
RESTRICTED_SCORES = "scores_N0262144"     # the restricted loads rescored at the farm's and the grid's N
BEST_TEXT = ("the facing reference times the grid's tilt gain at the nearest (height, distance, theta) cell: max over "
             "tilt of <kind>_obj New+J over its 0 deg value (lower-arm disc); relative_best = min(1, NJ / max(that, "
             "floor)), clipped at 1 per dish (relative_best_unclipped without the min)")
KIND_TEXT = ("one ceiling per kind: the largest <kind>_obj New+J of the grid over every tilt, height and distance at the "
             "load's theta (lower-arm disc, used for the middle arm too); S_kind = min(1, NJ / ceiling), 0 for a dish "
             "that pools (interior more than 2 mm below the rim); S = mean of S_kind, S_area = area-weighted mean")


def log(*a):
    print(f"[{dt.datetime.utcnow():%H:%M:%S}]", *a, flush=True)


def quat_matrix(q):
    x, y, z, w = (float(v) for v in q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


class Reference:
    """New+J of one dish kind alone at tilt 0 with its food-contact centroid at given points over its arm."""

    def __init__(self, DR, eng, theta):
        self.DR, self.eng, self.theta, self.geom, self.mesh = DR, eng, float(theta), {}, {}

    def _kind(self, kind):
        if kind not in self.geom:
            self.geom[kind] = self.DR.dish_geometry(kind)
            self.mesh[kind] = self.eng.build_mesh(self.geom[kind]["tris"])
        return self.geom[kind], self.mesh[kind]

    def centroid_rel(self, kind, position, quaternion, arm_center):
        geom, _ = self._kind(kind)
        return quat_matrix(quaternion) @ geom["centroid"] + np.asarray(position, dtype=float) - np.asarray(arm_center)

    def __call__(self, kind, cells):
        """cells = [(centroid relative to the arm centre (3,), arm radius)] -> reference New+J per cell."""
        DR, eng = self.DR, self.eng
        geom, mesh = self._kind(kind)
        R0 = DR.grid_rotation(0.)
        st_loc = geom["p_loc"] + DR.OFFSET_M * geom["n_loc"]
        st, nrm, cz = st_loc @ R0.T, geom["n_loc"] @ R0.T, R0 @ geom["centroid"]
        out = []
        for b in range(0, len(cells), BATCH):
            chunk = cells[b:b + BATCH]
            n = len(chunk)
            rel = np.concatenate([st + (np.asarray(c, dtype=float) - cz) for c, _ in chunk])
            rad = np.concatenate([np.full(len(st), float(r)) for _, r in chunk])
            dots = DR.Dots(rel, np.tile(nrm, (n, 1)), np.tile(st_loc, (n, 1)), rad, R0.T.copy())
            sums = eng.ray_sums(mesh, dots, [self.theta], fixed=False)
            WF, WG, WJ, _, _ = eng.wstar(dots, sums, [self.theta])
            nj = eng.scores(sums, WF, WG, WJ)[0]["NJ"][:, 0].cpu().numpy().reshape(n, -1)
            out += [float(v) for v in nj.mean(axis=1)]
        return out


def restricted_record(DR, rec):
    """A restricted score record (frigidaire_dot_rules_restricted.py: packed poses, New+J per dish only) in score_load's
    shape: per dish kind, arm, area, world pose and E.NJ, as dot_rules.load_dots derives them; the per-rack arms."""
    nj = {d["id"]: float(d["NJ"]) for d in rec["per_dish"]}
    per = [{"id": o["id"], "kind": o["kind"], "arm": DR.E.source_for(o["rack"]),
            "area_m2": DR.E.food_contact(o["kind"]).area_m2, "position_m": [float(v) for v in o["position_m"]],
            "quaternion_xyzw": [float(v) for v in o["quaternion_xyzw"]], "E": {"NJ": nj[o["id"]]}}
           for o in DR.world_objects(rec["entries"])]
    return {"load_id": rec["load_id"], "N": rec["N"], "theta_deg": rec["theta_deg"], "per_dish": per,
            "arms": {k: {"center": list(v["center"]), "radius": v["radius"]} for k, v in DR.E.ARM_SOURCES.items()}}


def jobs_of(args):
    """[(input path, output path, load_id, {state: score record})]."""
    out = []
    if args.loads_dir:
        for p in sorted(Path(args.loads_dir).glob("*.json")):
            if p.name.endswith(".relative.json"):
                continue
            rec = json.loads(p.read_text())
            if "per_dish" in rec:
                out.append((p, p.with_name(p.stem + ".relative.json"), rec["load_id"], {"settled": rec}))
    for d in args.dependency_dir or []:
        for p in sorted(Path(d).glob("*.scores.json")):
            rec = json.loads(p.read_text())
            states = {k: rec[k] for k in ("packed", "settled", "built") if rec.get(k)}
            out.append((p, p.with_name(p.name[:-len(".scores.json")] + ".relative.json"), rec["load_id"], states))
    if args.restricted_dir:
        for p in sorted((Path(args.restricted_dir) / RESTRICTED_SCORES).glob("*.json")):
            if p.name.endswith(".relative.json"):
                continue
            rec = json.loads(p.read_text())
            out.append((p, p.with_name(p.stem + ".relative.json"), rec["load_id"], {"packed": rec}))
    return out


def spearman(a, b):
    """Rank correlation (ties: mean rank), standard library only."""
    def ranks(x):
        order = sorted(range(len(x)), key=lambda i: x[i])
        r = [0.] * len(x)
        i = 0
        while i < len(order):
            j = i
            while j + 1 < len(order) and x[order[j + 1]] == x[order[i]]:
                j += 1
            for k in range(i, j + 1):
                r[order[k]] = (i + j) / 2.
            i = j + 1
        return r
    ra, rb = ranks(list(a)), ranks(list(b))
    ma, mb = sum(ra) / len(ra), sum(rb) / len(rb)
    cov = sum((x - ma) * (y - mb) for x, y in zip(ra, rb))
    return cov / (sum((x - ma) ** 2 for x in ra) * sum((y - mb) ** 2 for y in rb)) ** .5


def facing(jobs, args, t0):
    """The 2026-10-14 pass: the facing reference of every dish of ``jobs`` from the engine, one .relative.json each."""
    if B is None:
        raise SystemExit(f"{len(jobs)} loads have no .relative.json: the facing pass needs numpy and Warp (the box)")
    B.HX().register_hotec(B.ASSETS)
    from dishsim_frigidaire import dot_rules as DR
    for _, _, _, states in jobs:
        for state in list(states):
            if "arms" not in states[state]:              # a restricted score record
                states[state] = restricted_record(DR, states[state])
    Ns = {r["N"] for _, _, _, st in jobs for r in st.values()}
    thetas = {r["theta_deg"] for _, _, _, st in jobs for r in st.values()}
    if len(Ns) > 1 or len(thetas) > 1:
        raise SystemExit(f"mixed N {Ns} or theta {thetas}")
    N, theta = (Ns.pop(), thetas.pop()) if jobs else (262144, 30.)
    eng = DR.DotRuleEngine(N)
    ref = Reference(DR, eng, theta)
    lower = DR.E.ARM_SOURCES["lower_arm"]
    check = {k: ref(k, [((0., 0., .14), lower["radius"])])[0] for k in ("plate", "bowl", "cup")}
    log("check: tilt 0, 14 cm over the lower arm's centre, theta %g: plate %.3f bowl %.3f cup %.3f"
        % (theta, check["plate"], check["bowl"], check["cup"]))
    # every (job, state, dish) -> one cell; grouped by kind
    cells, where = {}, []
    for j, (_, _, _, states) in enumerate(jobs):
        for state, rec in states.items():
            for k, dish in enumerate(rec["per_dish"]):
                arm = rec["arms"][dish["arm"]]
                c = ref.centroid_rel(dish["kind"], dish["position_m"], dish["quaternion_xyzw"], arm["center"])
                cells.setdefault(dish["kind"], []).append((c, arm["radius"]))
                where.append((j, state, k, dish["kind"], len(cells[dish["kind"]]) - 1, c))
    log(f"{len(where)} dishes: " + ", ".join(f"{k} {len(v)}" for k, v in cells.items()))
    refs = {}
    for kind, cl in cells.items():
        t1 = time.time()
        refs[kind] = ref(kind, cl)
        log(f"{kind}: {len(cl)} references in {time.time() - t1:.0f} s")
    farm = []
    for j, (src, dst, lid, states) in enumerate(jobs):
        out_states = {}
        for state, rec in states.items():
            per = []
            for (jj, st, k, kind, ci, c) in where:
                if jj != j or st != state:
                    continue
                dish = rec["per_dish"][k]
                r, nj = refs[kind][ci], float(dish["E"]["NJ"])
                per.append({"id": dish["id"], "kind": kind, "arm": dish["arm"], "area_m2": dish["area_m2"],
                            "h_m": round(float(c[2]), 5), "d_m": round(float(np.hypot(c[0], c[1])), 5),
                            "reference_NJ": r, "NJ": nj, "relative": nj / max(r, FLOOR), "floored": r < FLOOR})
            a = np.array([p["area_m2"] for p in per])
            out_states[state] = {"S_NJ": float((a * np.array([p["NJ"] for p in per])).sum() / a.sum()),
                                 "S_rel": float((a * np.array([p["relative"] for p in per])).sum() / a.sum()),
                                 "n_floored": int(sum(p["floored"] for p in per)), "per_dish": per}
        rec = {"load_id": lid, "N": N, "theta_deg": theta, "floor": FLOOR, "source": os.path.relpath(src, ROOT),
               "reference": "the dish alone (own visual mesh the only blocker), tilt 0, its food-contact centroid at "
                            "its place relative to its own arm; New+J, J lines only",
               "states": out_states, "generated_utc": dt.datetime.utcnow().isoformat() + "Z"}
        tmp = dst.with_name(dst.name + ".tmp")
        tmp.write_text(json.dumps(rec, indent=1) + "\n")
        os.replace(tmp, dst)
        if args.loads_dir and str(src).startswith(str(Path(args.loads_dir))):
            farm.append((out_states["settled"]["S_NJ"], out_states["settled"]["S_rel"]))
    rho = spearman([f[0] for f in farm], [f[1] for f in farm]) if len(farm) > 2 else None
    log(f"[RESULT] PASS {len(jobs)} files, {len(where)} dish references in {time.time() - t0:.0f} s"
        + (f"; farm loads {len(farm)}: Spearman(S_rel, S_NJ) {rho:.3f}" if rho is not None else ""))


def nearest(xs, v):
    """Index of the grid node nearest to v (the first of two equally near, as the page's explorer)."""
    return min(range(len(xs)), key=lambda i: abs(xs[i] - v))


def grid_gains(grid_dir):
    """(grid, {kind: {(hi, di, qi): (gain, fallback)}}): max over tilt of <kind>_obj New+J (skipped poses left out) over
    its 0 deg value per (height, distance, theta) cell; gain 1 and fallback where the 0 deg pose is skipped or scores 0."""
    g = json.loads((Path(grid_dir) / "grid.json").read_text())
    if g["tilts"][0] != 0:
        raise SystemExit(f"{grid_dir}: the first tilt is {g['tilts'][0]}, not 0")
    nT, nH, nD, nQ = (len(g[k]) for k in ("tilts", "heights", "distances", "thetas"))
    out = {}
    for kind in ("plate", "bowl", "cup"):
        a = (g.get(f"{kind}_obj") or {}).get("NJ")
        if a is None:
            continue
        out[kind] = {}
        for hi in range(nH):
            for di in range(nD):
                for qi in range(nQ):
                    v = [a[((t * nH + hi) * nD + di) * nQ + qi] for t in range(nT)]
                    out[kind][hi, di, qi] = (max(v) / v[0], False) if v[0] > 0 else (1., True)
    return g, out


def best_state(s):
    """One state from its dishes' reference_best_NJ: relative_best = min(1, NJ / max(reference_best_NJ, FLOOR)) and
    relative_best_unclipped per dish; S_rel_best (clipped) and S_rel_best_unclipped, the area-weighted means, and
    n_floored_best, n_gain_fallback, n_clipped_best."""
    per = s["per_dish"]
    for p in per:
        raw = p["NJ"] / max(p["reference_best_NJ"], FLOOR)
        p.update({"relative_best": min(1., raw), "relative_best_unclipped": raw})
    a = sum(p["area_m2"] for p in per)
    s["S_rel_best"] = sum(p["area_m2"] * p["relative_best"] for p in per) / a
    s["S_rel_best_unclipped"] = sum(p["area_m2"] * p["relative_best_unclipped"] for p in per) / a
    s["n_floored_best"] = sum(p["floored_best"] for p in per)
    s["n_gain_fallback"] = sum(p["gain_fallback"] for p in per)
    s["n_clipped_best"] = sum(p["relative_best_unclipped"] > 1 for p in per)


def best_pass(jobs, grid_dir):
    """--reference best: gain, reference_best_NJ, floored_best, gain_fallback per dish, then best_state per state,
    written into each job's .relative.json next to the facing fields."""
    g, gains = grid_gains(grid_dir)
    for _, dst, _, _ in jobs:
        rec = json.loads(dst.read_text())
        qi = nearest(g["thetas"], float(rec["theta_deg"]))
        for s in rec["states"].values():
            for p in s["per_dish"]:
                kind = p["kind"].replace("hotec_", "")
                cell = (nearest(g["heights"], 100 * p["h_m"]), nearest(g["distances"], 100 * p["d_m"]), qi)
                gain, fb = gains[kind][cell] if kind in gains else (1., True)
                r = p["reference_NJ"] * gain
                p.update({"gain": gain, "gain_cell_cm": [g["heights"][cell[0]], g["distances"][cell[1]],
                                                         g["thetas"][qi]],
                          "reference_best_NJ": r, "floored_best": r < FLOOR, "gain_fallback": fb})
            best_state(s)
        rec["reference_best"] = BEST_TEXT
        rec["grid_obj"] = os.path.relpath(Path(grid_dir).resolve(), ROOT)
        rec["best_utc"] = dt.datetime.utcnow().isoformat() + "Z"
        tmp = dst.with_name(dst.name + ".tmp")
        tmp.write_text(json.dumps(rec, indent=1) + "\n")
        os.replace(tmp, dst)


def summary(jobs, args):
    """--summary: per group (farm, restricted, each dependency dir) the counts, the farm Spearman and top / bottom five."""
    def group(src):
        if args.loads_dir and str(src).startswith(str(Path(args.loads_dir))):
            return "farm"
        if args.restricted_dir and str(src).startswith(str(Path(args.restricted_dir))):
            return "restricted"
        return os.path.relpath(src.parent, ROOT)
    groups = {}
    for src, dst, lid, _ in jobs:
        groups.setdefault(group(src), []).append((lid, json.loads(dst.read_text())))
    for name, recs in groups.items():
        dishes = [p for _, r in recs for s in r["states"].values() for p in s["per_dish"]]
        over = [p["relative_best_unclipped"] for p in dishes if p["relative_best_unclipped"] > 1]
        s_all = [s["S_rel_best"] for _, r in recs for s in r["states"].values()]
        n_cl = sum(s["n_clipped_best"] > 0 for _, r in recs for s in r["states"].values())
        log(f"{name}: {len(recs)} loads, {len(s_all)} states, {len(dishes)} dishes; floored (best reference < {FLOOR}) "
            f"{sum(p['floored_best'] for p in dishes)}, gain fallback {sum(p['gain_fallback'] for p in dishes)}, "
            f"clipped at 1: {len(over)} dishes in {n_cl} states"
            + (f" (unclipped {min(over):.3f} to {max(over):.3f})" if over else "")
            + f"; S_rel_best min / median / max {min(s_all):.4f} / {statistics.median(s_all):.4f} / {max(s_all):.4f}")
        if name in ("farm", "restricted"):
            st = "settled" if name == "farm" else "packed"
            rows = [(lid, r["states"][st]["S_rel_best"], r["states"][st]["S_NJ"], r["states"][st]["S_rel"])
                    for lid, r in recs]
            by_abs = {x[0]: n + 1 for n, x in enumerate(sorted(rows, key=lambda x: -x[2]))}
            by_best = sorted(rows, key=lambda x: -x[1])
            if len(rows) > 2:
                log(f"{name}: Spearman(S_rel_best, S_NJ) {spearman([x[1] for x in rows], [x[2] for x in rows]):.3f}, "
                    f"Spearman(S_rel_best, S_rel facing) {spearman([x[1] for x in rows], [x[3] for x in rows]):.3f} "
                    f"over {len(rows)} loads")
            top5_abs = {lid for lid, n in by_abs.items() if n <= 5}
            bot5_abs = {lid for lid, n in by_abs.items() if n > len(rows) - 5}
            for word, five, ref5 in (("top", by_best[:5], top5_abs), ("bottom", by_best[-5:], bot5_abs)):
                log(f"{name} {word} five by S_rel_best: " + "; ".join(
                    f"{lid} {b:.4f} (S_NJ {a:.4f}, rank {by_abs[lid]} of {len(rows)})" for lid, b, a, _ in five)
                    + f"; {len({x[0] for x in five} & ref5)} of them in the {word} five by S_NJ")
        else:
            for lid, r in recs:
                log(f"{name} {lid}: " + ", ".join(f"{k} S_rel_best {s['S_rel_best']:.4f} (unclipped "
                                                    f"{s['S_rel_best_unclipped']:.4f}, S_NJ {s['S_NJ']:.4f}, floored "
                                                    f"{s['n_floored_best']}, clipped {s['n_clipped_best']})"
                                                    for k, s in r["states"].items()))


def read_grid(args):
    """The object-level grid: --grid-obj-dir's grid.json, else the window.GRID payload of a saved Dot Score Rules page."""
    if args.grid_obj_dir:
        return json.loads((Path(args.grid_obj_dir) / "grid.json").read_text())
    text = Path(args.grid_payload).read_text()
    i = text.index("window.GRID =") + len("window.GRID =")
    return json.JSONDecoder().raw_decode(text[i:].lstrip())[0]


def kind_ceilings(g, theta):
    """{kind: the largest <kind>_obj New+J over every tilt, height and distance at theta} (skipped poses are negative)."""
    nT, nH, nD, nQ = (len(g[k]) for k in ("tilts", "heights", "distances", "thetas"))
    qi = g["thetas"].index(theta)                  # the loads' theta must be a grid node
    return {kind: max(g[f"{kind}_obj"]["NJ"][((t * nH + hi) * nD + di) * nQ + qi]
                      for t in range(nT) for hi in range(nH) for di in range(nD)) / float(g.get("scale", 1))
            for kind in ("plate", "bowl", "cup")}


def kind_jobs(args):
    """[(score record path, .relative.json path)] for --reference kind: every saved .relative.json of the given dirs and
    the score record beside it (farm <id>.json, dependency <id>.scores.json, restricted <id>.json), present or not."""
    dirs = ([(Path(args.loads_dir), ".json")] if args.loads_dir else []) \
        + [(Path(d), ".scores.json") for d in args.dependency_dir or []] \
        + ([(Path(args.restricted_dir) / RESTRICTED_SCORES, ".json")] if args.restricted_dir else [])
    return [(p.with_name(p.name[:-len(".relative.json")] + ext), p) for d, ext in dirs
            for p in sorted(d.glob("*.relative.json"))]


def pooling_of(src, state):
    """The pooling ids of one state from its score record (dependency: per state; farm, restricted: one state), or None
    when the record is not here (its states are then scored with pooling not checked)."""
    if not src.is_file():
        return None
    rec = json.loads(src.read_text())
    rec = rec[state] if isinstance(rec.get(state), dict) else rec
    if "pooling" not in rec:
        raise SystemExit(f"{src} {state}: the score record has no pooling list")
    return rec["pooling"]


def kind_state(s, ceil, pooling):
    """One state over the kind ceilings: per dish ratio_kind = NJ / ceiling, S_kind = min(1, ratio_kind) or 0 when the
    dish pools; S (plain mean), S_area (area-weighted), S_by_kind, pooling, n_pooling (both None when not checked),
    n_clipped_kind."""
    per = s["per_dish"]
    pset = set(pooling or ())
    unknown = pset - {p["id"] for p in per}
    if unknown:
        raise SystemExit(f"pooling ids not among the dishes: {sorted(unknown)}")
    for p in per:
        k, pools = p["kind"].replace("hotec_", ""), p["id"] in pset
        ratio = p["NJ"] / ceil[k]
        p.update({"pools": pools, "ratio_kind": ratio, "S_kind": 0. if pools else min(1., ratio)})
    a = sum(p["area_m2"] for p in per)
    s["S"] = sum(p["S_kind"] for p in per) / len(per)
    s["S_area"] = sum(p["area_m2"] * p["S_kind"] for p in per) / a
    kinds = sorted({p["kind"].replace("hotec_", "") for p in per})
    s["S_by_kind"] = {k: statistics.mean(p["S_kind"] for p in per if p["kind"].replace("hotec_", "") == k) for k in kinds}
    s["pooling"] = None if pooling is None else sorted(pset)
    s["n_pooling"] = None if pooling is None else len(pset)
    s["n_clipped_kind"] = sum(p["ratio_kind"] > 1 and not p["pools"] for p in per)


def kind_pass(kj, ceil, theta, source):
    """--reference kind: kind_state per state of every .relative.json, the pooling ids from the score record beside it."""
    for src, dst in kj:
        rec = json.loads(dst.read_text())
        if float(rec["theta_deg"]) != theta:
            raise SystemExit(f"{dst}: theta {rec['theta_deg']}, the ceilings are at {theta}")
        for st, s in rec["states"].items():
            kind_state(s, ceil, pooling_of(src, st))
        rec.update({"reference_kind": KIND_TEXT, "ceilings_kind": ceil, "ceilings_source": source,
                    "kind_utc": dt.datetime.utcnow().isoformat() + "Z"})
        tmp = dst.with_name(dst.name + ".tmp")
        tmp.write_text(json.dumps(rec, indent=1) + "\n")
        os.replace(tmp, dst)


def summary_kind(kj, args, ceil):
    """--summary of the kind pass: per group the counts and S range; farm and restricted: Spearman of S and S_area
    against S_NJ and S_rel_best, top and bottom five by S; dependency loads one line per state."""
    def group(dst):
        if args.loads_dir and str(dst).startswith(str(Path(args.loads_dir))):
            return "farm"
        if args.restricted_dir and str(dst).startswith(str(Path(args.restricted_dir))):
            return "restricted"
        return os.path.relpath(dst.parent, ROOT)
    log("ceilings (" + ", ".join(f"{k} {v:.4f}" for k, v in ceil.items()) + ")")
    groups = {}
    for _, dst in kj:
        r = json.loads(dst.read_text())
        groups.setdefault(group(dst), []).append((r["load_id"], r))
    for name, recs in groups.items():
        sts = [s for _, r in recs for s in r["states"].values()]
        dishes = [p for s in sts for p in s["per_dish"]]
        over = [p["ratio_kind"] for p in dishes if p["ratio_kind"] > 1 and not p["pools"]]
        pk = {}
        for p in dishes:
            if p["pools"]:
                pk[p["kind"]] = pk.get(p["kind"], 0) + 1
        unchecked = sum(s["n_pooling"] is None for s in sts)
        log(f"{name}: {len(recs)} loads, {len(sts)} states, {len(dishes)} dishes; pooling "
            + (f"NOT CHECKED in {unchecked} states (no score record here), " if unchecked else "")
            + f"{sum(pk.values())} pooling dishes ({', '.join(f'{k} {v}' for k, v in sorted(pk.items())) or 'none'}) in "
            f"{sum((s['n_pooling'] or 0) > 0 for s in sts)} states; clipped at 1: {len(over)} dishes"
            + (f" ({min(over):.3f} to {max(over):.3f})" if over else "")
            + f"; S min / median / max {min(s['S'] for s in sts):.4f} / {statistics.median(s['S'] for s in sts):.4f} / "
            f"{max(s['S'] for s in sts):.4f}")
        if name not in ("farm", "restricted"):
            for lid, r in recs:
                for k, s in r["states"].items():
                    log(f"{name} {lid} {k}: S {s['S']:.4f}, S_area {s['S_area']:.4f}, "
                        + ", ".join(f"{kk} {v:.4f}" for kk, v in s["S_by_kind"].items())
                        + f"; pooling {s['n_pooling']}, clipped {s['n_clipped_kind']}; S_NJ {s['S_NJ']:.4f}, "
                        f"S_rel_best {s['S_rel_best']:.4f}")
            continue
        st = "settled" if name == "farm" else "packed"
        rows = [(lid, r["states"][st]) for lid, r in recs]
        rank = {key: {x[0]: n + 1 for n, x in enumerate(sorted(rows, key=lambda x: -x[1][key]))}
                for key in ("S_NJ", "S_rel_best")}
        for key in ("S", "S_area"):
            log(f"{name}: Spearman({key}, S_NJ) {spearman([x[1][key] for x in rows], [x[1]['S_NJ'] for x in rows]):.3f}, "
                f"Spearman({key}, S_rel_best) "
                f"{spearman([x[1][key] for x in rows], [x[1]['S_rel_best'] for x in rows]):.3f} over {len(rows)} loads"
                + (f"; Spearman(S, S_area) {spearman([x[1]['S'] for x in rows], [x[1]['S_area'] for x in rows]):.3f}"
                   if key == "S" else ""))
        by_s = sorted(rows, key=lambda x: -x[1]["S"])
        for word, five in (("top", by_s[:5]), ("bottom", by_s[-5:])):
            log(f"{name} {word} five by S: " + "; ".join(
                f"{lid} {s['S']:.4f} (S_area {s['S_area']:.4f}; "
                + ", ".join(f"{k} {v:.3f}" for k, v in s["S_by_kind"].items())
                + f"; pooling {s['n_pooling']}; S_NJ {s['S_NJ']:.4f} rank {rank['S_NJ'][lid]}, S_rel_best rank "
                f"{rank['S_rel_best'][lid]} of {len(rows)})" for lid, s in five))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--loads-dir", default=None)
    ap.add_argument("--dependency-dir", action="append", default=None)
    ap.add_argument("--restricted-dir", default=None, help=f"restricted sampler dir: its {RESTRICTED_SCORES}/<id>.json")
    ap.add_argument("--reference", choices=("facing", "best", "kind"), default="facing",
                    help="best: the facing reference times the grid's tilt gain; kind: one ceiling per kind (docstring)")
    ap.add_argument("--grid-obj-dir", default=None, help="--reference best / kind: the object-level grid level (grid.json)")
    ap.add_argument("--grid-payload", default=None,
                    help="--reference kind: a saved Dot Score Rules index.html (its window.GRID) instead of --grid-obj-dir")
    ap.add_argument("--summary", action="store_true", help="--reference best / kind: Spearman, top / bottom five, counts")
    ap.add_argument("--force", action="store_true", help="recompute loads that already have a .relative.json")
    args = ap.parse_args()
    if args.reference == "best" and not args.grid_obj_dir:
        raise SystemExit("--reference best needs --grid-obj-dir")
    if args.reference == "kind" and not (args.grid_obj_dir or args.grid_payload):
        raise SystemExit("--reference kind needs --grid-obj-dir or --grid-payload")
    t0 = time.time()
    all_jobs = jobs_of(args)
    jobs = [j for j in all_jobs if args.force or not j[1].is_file()]
    log(f"{len(jobs)} load files to reference")
    if jobs or args.reference == "facing":
        facing(jobs, args, t0)
    if args.reference == "best":
        best_pass(all_jobs, args.grid_obj_dir)
        dishes = [p for _, dst, _, _ in all_jobs for s in json.loads(dst.read_text())["states"].values()
                  for p in s["per_dish"]]
        if args.summary:
            summary(all_jobs, args)
        over = sum(p["relative_best_unclipped"] > 1 for p in dishes)
        log(f"[RESULT] PASS --reference best: {len(all_jobs)} files, {len(dishes)} dishes, "
            f"{sum(p['floored_best'] for p in dishes)} floored, {over} clipped at 1, in {time.time() - t0:.0f} s")
    if args.reference == "kind":
        kj = kind_jobs(args)
        thetas = {float(json.loads(dst.read_text())["theta_deg"]) for _, dst in kj}
        if len(thetas) != 1:
            raise SystemExit(f"mixed theta {thetas}")
        theta = thetas.pop()
        source = os.path.relpath(Path(args.grid_obj_dir or args.grid_payload).resolve(), ROOT)
        ceil = kind_ceilings(read_grid(args), theta)
        kind_pass(kj, ceil, theta, source)
        if args.summary:
            summary_kind(kj, args, ceil)
        sts = [s for _, dst in kj for s in json.loads(dst.read_text())["states"].values()]
        dishes = [p for s in sts for p in s["per_dish"]]
        bad = [p["id"] for p in dishes if not 0. <= p["S_kind"] <= 1.]
        unchecked = sum(s["n_pooling"] is None for s in sts)
        log(f"[RESULT] {'FAIL' if bad else 'PARTIAL' if unchecked else 'PASS'} --reference kind: {len(kj)} files, "
            f"{len(sts)} states, {len(dishes)} dishes, ceilings at theta {theta:g} "
            + " / ".join(f"{k} {v:.4f}" for k, v in ceil.items()) + f" ({source}); "
            + (f"pooling NOT CHECKED in {unchecked} states (no score record beside their .relative.json); " if unchecked
               else "") + f"{sum(p['pools'] for p in dishes)} pooling (scored 0), "
            f"{sum(p['ratio_kind'] > 1 and not p['pools'] for p in dishes)} clipped at 1"
            + (f"; S_kind outside [0, 1]: {bad}" if bad else "") + f"; in {time.time() - t0:.0f} s")


if __name__ == "__main__":
    sys.exit(main())
