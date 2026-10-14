#!/usr/bin/env python3
# Copyright (c) 2026, dishsim project.
# SPDX-License-Identifier: BSD-3-Clause
"""The slowest-converging grid cells beyond the sweep: the cells with the largest |New+G - New+J| at the top grid
level, recomputed alone (their own theta only) at larger N, to show that the extreme cells meet too.

    code/util/run_py.sh code/planner/frigidaire/frigidaire_dot_rules_tail.py --from-level 65536 --levels 262144 1048576 4194304

Writes data/results/dot_rules/grid/tail.json: one row per (cell, N) with the dish means of NG, NJ, DG, DJ, the gap,
the largest per-dot gap and the mean G and J line counts. The from-level row is copied from its grid.json.
"""
from __future__ import annotations

import argparse
import datetime as dt
import json
import os
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
sys.path[:0] = [str(ROOT / "code/src"), str(ROOT / "code/frigidaire/src"), str(Path(__file__).resolve().parent)]
import frigidaire_bench as B  # noqa: E402

GRID = ROOT / "data/results/dot_rules/grid"


def log(*a):
    print(f"[{dt.datetime.utcnow():%H:%M:%S}]", *a, flush=True)


def worst_cells(grid, top):
    """(dish, ti, hi, di, qi) of the `top` largest dish-mean |NG - NJ|, plus the worst cell of every theta."""
    nT, nH, nD, Q = len(grid["tilts"]), len(grid["heights"]), len(grid["distances"]), len(grid["thetas"])
    rows = []
    for dish in ("plate", "bowl"):
        ng = np.asarray(grid[dish]["NG"], float).reshape(nT, nH, nD, Q)
        nj = np.asarray(grid[dish]["NJ"], float).reshape(nT, nH, nD, Q)
        ok = ng >= 0
        gap = np.where(ok, np.abs(ng - nj) / grid["scale"], -1.)
        for idx in zip(*np.nonzero(ok)):
            rows.append((float(gap[idx]), dish) + tuple(int(i) for i in idx))
    rows.sort(reverse=True)
    pick = rows[:top]
    for qi in range(Q):
        best = max((r for r in rows if r[5] == qi), default=None)
        if best is not None and best not in pick:
            pick.append(best)
    return pick


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--from-level", type=int, default=65536)
    ap.add_argument("--levels", type=int, nargs="+", default=[262144, 1048576, 4194304])
    ap.add_argument("--top", type=int, default=6)
    ap.add_argument("--out", type=Path, default=GRID / "tail.json")
    args = ap.parse_args()

    grid = json.loads((GRID / f"N{args.from_level:07d}" / "grid.json").read_text())
    picks = worst_cells(grid, args.top)
    B.HX().register_hotec(B.ASSETS)
    from dishsim_frigidaire import dot_rules as DR

    TH = list(grid["thetas"])
    geoms = {d: DR.dish_geometry(d) for d in ("plate", "bowl")}
    rows = []
    for gap0, dish, ti, hi, di, qi in picks:
        flat = ((ti * len(grid["heights"]) + hi) * len(grid["distances"]) + di) * len(TH) + qi
        cell = {"dish": dish, "tilt": grid["tilts"][ti], "h_cm": grid["heights"][hi], "d_cm": grid["distances"][di],
                "theta": TH[qi]}
        g = grid[dish]
        rows.append({**cell, "N": args.from_level, "NG": g["NG"][flat] / grid["scale"], "NJ": g["NJ"][flat] / grid["scale"],
                     "DG": g["DG"][flat] / grid["scale"], "DJ": g["DJ"][flat] / grid["scale"], "gap": gap0,
                     "source": "grid"})
        log("cell", cell, f"gap at N={args.from_level}: {gap0:.4g}")
    for N in args.levels:
        eng = DR.DotRuleEngine(N)
        meshes = {d: eng.build_mesh(geoms[d]["tris"]) for d in geoms}
        for gap0, dish, ti, hi, di, qi in picks:
            geom, tilt, th = geoms[dish], grid["tilts"][ti], TH[qi]
            cells = [c for c in DR.grid_cells(geom, tilt) if (c[0], c[1]) == (hi, di)]
            dots = DR.grid_dots(geom, tilt, cells)
            t0 = time.perf_counter()
            s = eng.ray_sums(meshes[dish], dots, [th])
            WF, WG, WJ, mF, diag = eng.wstar(dots, s, [th])
            sc, zG, zJ, above = eng.scores(s, WF, WG, WJ)
            A = {k: v.cpu().numpy().reshape(500, -1)[:, -1] for k, v in sc.items()}
            row = {"dish": dish, "tilt": tilt, "h_cm": grid["heights"][hi], "d_cm": grid["distances"][di], "theta": th,
                   "N": N, "NG": float(A["NG"].mean()), "NJ": float(A["NJ"].mean()), "DG": float(A["DG"].mean()),
                   "DJ": float(A["DJ"].mean()), "gap": float(abs(A["NG"].mean() - A["NJ"].mean())),
                   "dot_gap_max": float(np.abs(A["NG"] - A["NJ"]).max()),
                   "nG_mean": float(s.g_n[:, 0].double().mean()), "nJ_mean": float(s.j_n[:, 0].double().mean()),
                   "binds": diag.get("binds", 0), "above1": int(sum(int(v) for v in above.values()) if isinstance(above, dict) else int(above)),
                   "seconds": round(time.perf_counter() - t0, 2), "source": "tail"}
            rows.append(row)
            log(f"N={N} {dish} t{tilt} h{row['h_cm']} d{row['d_cm']} th{th}: NG {row['NG']:.5f} NJ {row['NJ']:.5f} "
                f"gap {row['gap']:.3g} (dot max {row['dot_gap_max']:.3g}), |G| {row['nG_mean']:.0f} |J| {row['nJ_mean']:.0f}, "
                f"{row['seconds']} s")
            tmp = args.out.with_suffix(".tmp")
            tmp.write_text(json.dumps({"generated_utc": dt.datetime.utcnow().isoformat() + "Z",
                                       "from_level": args.from_level, "levels": args.levels, "rows": rows}, indent=1))
            os.replace(tmp, args.out)
    print(f"[RESULT] PASS tail: {len(picks)} cells x {len(args.levels)} levels -> {args.out}", flush=True)


if __name__ == "__main__":
    main()
