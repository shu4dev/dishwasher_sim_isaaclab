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

    code/util/run_py.sh code/planner/frigidaire/frigidaire_dot_rules_relative.py \\
        --loads-dir data/results/dot_rules/loads/N0262144 --dependency-dir data/results/dot_rules/dependency/pairs
Prints a check line first: the plate / bowl / cup reference at 14 cm over the lower arm's centre, theta 30 (the grid
gives 0.814 / 0.504 / 0.183 there), then [RESULT] PASS with the counts and the Spearman rank correlation of S_rel
against S_NJ over the farm loads.
"""
from __future__ import annotations

import argparse
import datetime as dt
import json
import os
from pathlib import Path
import sys
import time

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
sys.path[:0] = [str(ROOT / "code/src"), str(ROOT / "code/frigidaire/src"), str(Path(__file__).resolve().parent)]
import frigidaire_bench as B  # noqa: E402

FLOOR = .05
BATCH = 256                   # dishes per engine call (500 dots each)


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
    return out


def spearman(a, b):
    def ranks(x):
        x = np.asarray(x, dtype=float)
        order = np.argsort(x, kind="mergesort")
        r = np.empty(len(x))
        r[order] = np.arange(len(x))
        for v in np.unique(x):            # ties: mean rank
            m = x == v
            r[m] = r[m].mean()
        return r
    ra, rb = ranks(a), ranks(b)
    return float(np.corrcoef(ra, rb)[0, 1])


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--loads-dir", default=None)
    ap.add_argument("--dependency-dir", action="append", default=None)
    ap.add_argument("--force", action="store_true", help="recompute loads that already have a .relative.json")
    args = ap.parse_args()
    t0 = time.time()
    jobs = [j for j in jobs_of(args) if args.force or not j[1].is_file()]
    log(f"{len(jobs)} load files to reference")
    B.HX().register_hotec(B.ASSETS)
    from dishsim_frigidaire import dot_rules as DR
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


if __name__ == "__main__":
    sys.exit(main())
