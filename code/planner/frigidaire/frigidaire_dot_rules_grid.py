#!/usr/bin/env python3
# Copyright (c) 2026, dishsim project.
# SPDX-License-Identifier: BSD-3-Clause
"""Dot-rule dish grid at large N (Kit-free, GPU): the page GRID (plate, bowl, cup, the unblocked 'dot' entity),
the object-level grid of every dish (<dish>_obj: per cell, sum over its 500 dots of a rule's numerator / sum of its
per-dot denominator), per-dot figure arrays and the New+G vs New+J convergence statistics, for one N or a sweep.

    code/util/run_py.sh code/planner/frigidaire/frigidaire_dot_rules_grid.py --sweep 64 262144 --deadline-utc 11:45

Tilt check (plans/2026-10-22-tilt-score-check.md): --tilts=A:B:S (B included; < 0 leans the food side toward the disc
centre), --spots "h,d ..." (cm, grid nodes; only those cells are cast), --theta 30 (comma list), always with its own --out.

Outputs per N in OUT/N{N:07d}/ (OUT = --out, default data/results/dot_rules/grid): grid.json, perdot_{dish}.npz,
convergence.json, partial/{dish}_t{tilt:03d}.npz (resume checkpoints; one without obj_means is stale, deleted and
recomputed); after every level OUT/convergence_sweep.{json,csv}. Every level is complete or absent: the sweep stops
before a level whose projection overruns the deadline.
"""
from __future__ import annotations

import argparse
import datetime as dt
import json
import math
import os
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
sys.path[:0] = [str(ROOT / "code/src"), str(ROOT / "code/frigidaire/src"), str(Path(__file__).resolve().parent)]
import frigidaire_bench as B  # noqa: E402

OUT = ROOT / "data/results/dot_rules/grid"
PERDOT_TILTS = (0, 45, 90, 135, 180)
PERDOT_CELL_CM = (14, 0)
PERDOT_THETA = 30


def log(*a):
    print(f"[{dt.datetime.utcnow():%H:%M:%S}]", *a, flush=True)


def atomic_write(path, text):
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(path.suffix + ".tmp")
    tmp.write_text(text)
    os.replace(tmp, path)


def atomic_npz(path, **arrays):
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_name(path.stem + ".tmp.npz")
    np.savez_compressed(tmp, **arrays)
    os.replace(tmp, path)


def deadline_of(hhmm):
    if not hhmm:
        return None
    h, m = (int(v) for v in hhmm.split(":"))
    now = dt.datetime.now(dt.timezone.utc)
    return now.replace(hour=h, minute=m, second=0, microsecond=0).timestamp()


def run_tilt(DR, eng, geom, mesh, tilt, path, spots=None):
    """One (dish, tilt) batch: every non-skipped (h, d) cell, 500 dots each, all 10 theta."""
    TH = DR.THETAS_DEG
    Q = len(TH)
    cells = DR.grid_cells(geom, tilt)
    if spots:
        cells = [c for c in cells if (DR.GRID_H_CM[c[0]], DR.GRID_D_CM[c[1]]) in spots]
    if not cells:
        atomic_npz(path, cells=np.zeros((0, 2), np.int32), obj_means=np.zeros((0, len(DR.RULES), Q)))
        return {"cells": 0}
    dots = DR.grid_dots(geom, tilt, cells)
    t0 = time.perf_counter()
    s = eng.ray_sums(mesh, dots, TH)
    t1 = time.perf_counter()
    WF, WG, WJ, mF, diag = eng.wstar(dots, s, TH, all_sets=True)     # W* of every dot of a non-empty set
    t2 = time.perf_counter()
    sc, zG, zJ, above = eng.scores(s, WF, WG, WJ)
    C = len(cells)
    A = {k: v.cpu().numpy().reshape(C, 500, -1) for k, v in sc.items()}          # (C, 500, 1 or Q)
    A = {k: np.broadcast_to(v, (C, 500, Q)) for k, v in A.items()}
    means = np.stack([A[k].mean(axis=1) for k in DR.RULES], axis=1)               # (C, 7, Q)
    # object level: per cell, sum over its 500 dots of the numerator / sum of the per-dot denominator (0 if 0)
    cs = {k: v.cpu().numpy().reshape(C, 500, -1).sum(axis=1) for k, v in (       # (C, 1 or Q)
        ("fcv", s.f_cv), ("fc", s.f_c), ("gcv", s.g_cv), ("jcv", s.j_cv), ("WF", WF), ("WG", WG), ("WJ", WJ))}

    def div0(a, b):
        return np.where(b > 0, a / np.where(b > 0, b, 1.), 0.)

    nN = 500. * eng.N
    obj = {"O": div0(cs["fcv"], cs["fc"]), "D": cs["fcv"] / nN, "DG": cs["gcv"] / nN, "DJ": cs["jcv"] / nN,
           "N": div0(cs["fcv"], cs["WF"]), "NG": div0(cs["gcv"], cs["WG"]), "NJ": div0(cs["jcv"], cs["WJ"])}
    obj_means = np.stack([np.broadcast_to(obj[k], (C, Q)) for k in DR.RULES], axis=1)   # (C, 7, Q)
    gap = np.abs(A["NG"] - A["NJ"])
    h_dot = dots.rel[:, 2].reshape(C, 500)
    ratio = (np.maximum(h_dot, 0.)[:, :, None] * np.tan(np.radians(TH))[None, None, :] / DR.GRID_ARM_R) ** 2
    djlim = (A["DJ"] * ratio).mean(axis=1)                                        # (C, Q)
    rays = s.rays.cpu().numpy().reshape(C, 500).sum(axis=1)
    out = {"cells": np.array([(hi, di) for hi, di, _ in cells], np.int32), "means": means, "obj_means": obj_means,
           "zG": zG.cpu().numpy().reshape(C, 500, Q).sum(axis=1).astype(np.int32),
           "zJ": zJ.cpu().numpy().reshape(C, 500, Q).sum(axis=1).astype(np.int32),
           "gap_dot_max": gap.max(axis=1), "gap_dot_mean": gap.mean(axis=1), "djlim": djlim,
           "maxdot": np.stack([A[k].max(axis=(1, 2)) for k in DR.RULES], axis=1),
           "above1": np.stack([(A[k] > 1 + 1e-9).sum(axis=(1, 2)) for k in DR.RULES], axis=1).astype(np.int32),
           "rays": rays, "t_rays": t1 - t0, "t_wstar": t2 - t1, "t_total": time.perf_counter() - t0,
           "diag": json.dumps(diag)}
    if tilt in PERDOT_TILTS and PERDOT_THETA in TH:
        hi_t, di_t = DR.GRID_H_CM.index(PERDOT_CELL_CM[0]), DR.GRID_D_CM.index(PERDOT_CELL_CM[1])
        ci = next((j for j, (hi, di, _) in enumerate(cells) if (hi, di) == (hi_t, di_t)), None)
        if ci is not None:
            q = TH.index(PERDOT_THETA)
            sl = slice(ci * 500, (ci + 1) * 500)
            Rm = DR.grid_rotation(tilt)
            p = cells[ci][2]
            out.update({"pd_R": Rm, "pd_p": p, "pd_pos": geom["p_loc"] @ Rm.T + p, "pd_nrm": geom["n_loc"] @ Rm.T,
                        "pd_scores": np.stack([A[k][ci, :, q] for k in DR.RULES]).astype(np.float32),
                        "pd_sumc": np.stack([s.f_c[sl].cpu().numpy(), s.g_c[sl, q].cpu().numpy(),
                                             s.j_c[sl, q].cpu().numpy()]),
                        "pd_nlines": np.stack([s.g_n[sl, q].cpu().numpy(), s.j_n[sl, q].cpu().numpy()]).astype(np.int32),
                        "pd_empty": np.stack([zG[sl, q].cpu().numpy(), zJ[sl, q].cpu().numpy()])})
    atomic_npz(path, **out)
    return {"cells": C, "rays": float(rays.sum()), "t_rays": t1 - t0, "t_wstar": t2 - t1, "diag": diag,
            "above": above}


def dot_entity(DR, eng, sign):
    TH = DR.THETAS_DEG
    dots = DR.dot_entity_dots(tilts=DR.GRID_TILTS, sign=sign)
    s = eng.ray_sums(None, dots, TH, cast=False)
    WF, WG, WJ, mF, diag = eng.wstar(dots, s, TH, all_f=True)
    sc, zG, zJ, above = eng.scores(s, WF, WG, WJ)
    P, Q = len(dots.radius), len(TH)
    out = {}
    for k in DR.RULES:
        v = sc[k].cpu().numpy()
        v = np.broadcast_to(v.reshape(P, -1), (P, Q))
        out[k] = np.rint(v * 1e6).astype(np.int64).reshape(-1).tolist()
    out["zG"] = zG.cpu().numpy().astype(int).reshape(-1).tolist()
    out["zJ"] = zJ.cpu().numpy().astype(int).reshape(-1).tolist()
    out["nG"] = s.g_n.cpu().numpy().astype(int).reshape(-1).tolist()
    out["nJ"] = s.j_n.cpu().numpy().astype(int).reshape(-1).tolist()
    m = mF.cpu().numpy().astype(np.float64)
    m /= np.maximum(np.linalg.norm(m, axis=1, keepdims=True), 1e-300)
    down = np.degrees(np.arccos(np.clip(-m[:, 2], -1, 1)))
    gapn = np.degrees(np.arccos(np.clip((m * dots.nrm).sum(axis=1), -1, 1)))
    out["mDown"] = np.repeat(np.rint(down * 10).astype(int), Q).tolist()
    out["mGap"] = np.repeat(np.rint(gapn * 10).astype(int), Q).tolist()
    return out, {"diag": diag, "above": above}


def assemble(DR, N, level, dishes, meta):
    """grid.json, perdot npz and convergence.json from the partial checkpoints of one level."""
    TH = DR.THETAS_DEG
    nT, nH, nD, Q = len(DR.GRID_TILTS), len(DR.GRID_H_CM), len(DR.GRID_D_CM), len(TH)
    grid = {"tilts": list(DR.GRID_TILTS), "heights": list(DR.GRID_H_CM), "distances": list(DR.GRID_D_CM),
            "thetas": list(TH), "scale": 1000000, "dots": 500, "n_points": N}
    conv_cells = []           # (dish, tilt, hi, di, means (7,Q), gap_dot_max (Q), gap_dot_mean (Q), djlim (Q))
    tot = {"rays": 0., "t_rays": 0., "t_wstar": 0., "certified": 0, "uncertified": 0, "skipped": 0, "reused": 0,
           "binds": 0, "max_bind_rel": 0., "above1": np.zeros(7, np.int64), "maxdot": np.zeros(7)}
    for dish in dishes:
        arr = {k: np.full(nT * nH * nD * Q, -1, np.int64) for k in list(DR.RULES) + ["zG", "zJ"]}
        obj = {k: np.full(nT * nH * nD * Q, -1, np.int64) for k in DR.RULES}
        perdot = {"tilts": [], "R": [], "p": [], "pos": [], "nrm": [], "scores": [], "sumc": [], "nlines": [],
                  "empty": []}
        for ti, tilt in enumerate(DR.GRID_TILTS):
            z = np.load(level / "partial" / f"{dish}_t{tilt:03d}.npz")
            if len(z["cells"]) == 0:
                continue
            om = z["obj_means"]
            d = json.loads(str(z["diag"]))
            for k in ("certified", "uncertified", "skipped", "reused", "binds"):
                tot[k] += d[k]
            tot["max_bind_rel"] = max(tot["max_bind_rel"], d["max_bind_rel"])
            tot["rays"] += float(z["rays"].sum())
            tot["t_rays"] += float(z["t_rays"])
            tot["t_wstar"] += float(z["t_wstar"])
            tot["above1"] += z["above1"].sum(axis=0)
            tot["maxdot"] = np.maximum(tot["maxdot"], z["maxdot"].max(axis=0))
            for ci, (hi, di) in enumerate(z["cells"]):
                base = ((ti * nH + hi) * nD + di) * Q
                for j, k in enumerate(DR.RULES):
                    arr[k][base:base + Q] = np.rint(z["means"][ci, j] * 1e6)
                    obj[k][base:base + Q] = np.rint(om[ci, j] * 1e6)
                arr["zG"][base:base + Q] = z["zG"][ci]
                arr["zJ"][base:base + Q] = z["zJ"][ci]
                conv_cells.append((dish, tilt, int(hi), int(di), z["means"][ci], z["gap_dot_max"][ci],
                                   z["gap_dot_mean"][ci], z["djlim"][ci]))
            if "pd_scores" in z.files:
                perdot["tilts"].append(tilt)
                for k in ("R", "p", "pos", "nrm", "scores", "sumc", "nlines", "empty"):
                    perdot[k].append(z["pd_" + k])
        grid[dish] = {k: v.tolist() for k, v in arr.items()}
        grid[f"{dish}_obj"] = {k: v.tolist() for k, v in obj.items()}
        if perdot["tilts"]:
            sc = np.stack(perdot["scores"])
            atomic_npz(level / f"perdot_{dish}.npz", tilts=np.array(perdot["tilts"]), R=np.stack(perdot["R"]),
                       p=np.stack(perdot["p"]), pos=np.stack(perdot["pos"]), nrm=np.stack(perdot["nrm"]), scores=sc,
                       sumc=np.stack(perdot["sumc"]), nlines=np.stack(perdot["nlines"]),
                       empty=np.stack(perdot["empty"]), means=sc.mean(axis=2),
                       n_above1=(sc > 1 + 1e-9).sum(axis=2), rules=np.array(DR.RULES), N=N, theta_deg=PERDOT_THETA,
                       h_cm=PERDOT_CELL_CM[0], d_cm=PERDOT_CELL_CM[1])
    # convergence statistics per theta over every non-skipped (dish, cell)
    jN, jNG, jNJ, jDG, jDJ = (DR.RULES.index(k) for k in ("N", "NG", "NJ", "DG", "DJ"))
    per_theta = []
    for q, th in enumerate(TH):
        g = np.array([abs(c[4][jNG, q] - c[4][jNJ, q]) for c in conv_cells])
        am = int(np.argmax(g))
        c = conv_cells[am]
        dmax = np.array([c_[5][q] for c_ in conv_cells])
        dmean = np.array([c_[6][q] for c_ in conv_cells])
        hcell = np.array([DR.GRID_H_CM[c_[2]] / 100. for c_ in conv_cells])
        DG = np.array([c_[4][jDG, q] for c_ in conv_cells])
        DJ = np.array([c_[4][jDJ, q] for c_ in conv_cells])
        lim = np.array([c_[7][q] for c_ in conv_cells])
        ok = (DG > 0) & (DJ > 0)
        aR2 = (hcell * math.tan(math.radians(th)) / DR.GRID_ARM_R) ** 2
        rr = np.abs(DG[ok] / DJ[ok] / aR2[ok] - 1.) if ok.any() else np.zeros(1)
        okl = DG > 0
        ll = np.abs(DG[okl] - lim[okl]) / DG[okl] if okl.any() else np.zeros(1)
        per_theta.append({"theta_deg": th, "n_cells": len(g), "gap_max": float(g.max()), "gap_mean": float(g.mean()),
                          "gap_p95": float(np.percentile(g, 95)),
                          "gap_argmax": {"dish": c[0], "tilt": c[1], "h_cm": DR.GRID_H_CM[c[2]],
                                         "d_cm": DR.GRID_D_CM[c[3]], "NG": float(c[4][jNG, q]),
                                         "NJ": float(c[4][jNJ, q])},
                          "dot_gap_max": float(dmax.max()), "dot_gap_mean": float(dmean.mean()),
                          "dgdj_vs_aR2": {"median": float(np.median(rr)), "max": float(rr.max()), "n": int(ok.sum())},
                          "dg_vs_dj_limit": {"median": float(np.median(ll)), "max": float(ll.max()), "n": int(okl.sum())}})
    conv = {"N": N, "per_theta": per_theta, "rays": tot["rays"], "seconds_rays": tot["t_rays"],
            "seconds_wstar": tot["t_wstar"], "rays_per_s": tot["rays"] / max(tot["t_rays"], 1e-9),
            "kappa_rays_per_dotpose_over_N": tot["rays"] / (len(conv_cells) * 500.) / N,
            "wstar": {k: tot[k] for k in ("certified", "uncertified", "skipped", "reused", "binds", "max_bind_rel")},
            "n_above_1": dict(zip(DR.RULES, tot["above1"].tolist())),
            "max_dot_score": dict(zip(DR.RULES, tot["maxdot"].tolist())), **meta}
    return grid, conv


def sweep_append(out, N, conv):
    path = out / "convergence_sweep.json"
    data = json.loads(path.read_text()) if path.is_file() else {"levels": []}
    data["levels"] = [lv for lv in data["levels"] if lv["N"] != N] + [conv]
    data["levels"].sort(key=lambda lv: lv["N"])
    data["updated_utc"] = dt.datetime.utcnow().isoformat() + "Z"
    atomic_write(path, json.dumps(data, indent=1) + "\n")
    rows = ["N,theta_deg,gap_max,gap_mean,gap_p95,dot_gap_max,dot_gap_mean,dgdj_median,dgdj_max,limit_median,limit_max"]
    for lv in data["levels"]:
        for t in lv["per_theta"]:
            rows.append(",".join(str(v) for v in (lv["N"], t["theta_deg"], t["gap_max"], t["gap_mean"], t["gap_p95"],
                                                  t["dot_gap_max"], t["dot_gap_mean"], t["dgdj_vs_aR2"]["median"],
                                                  t["dgdj_vs_aR2"]["max"], t["dg_vs_dj_limit"]["median"],
                                                  t["dg_vs_dj_limit"]["max"])))
    atomic_write(out / "convergence_sweep.csv", "\n".join(rows) + "\n")


def run_level(DR, N, args, deadline):
    from dishsim_frigidaire import dot_rules  # noqa: F401  (engine module already imported)
    level = args.out / f"N{N:07d}"
    if (level / "grid.json").is_file() and (level / "convergence.json").is_file():
        have = json.loads((level / "grid.json").read_text())
        if all(k in have for d in args.dishes for k in (d, f"{d}_obj")):
            log(f"N={N}: complete, skipped")
            return json.loads((level / "convergence.json").read_text()), 0.
        log(f"N={N}: grid.json lacks a requested dish or its _obj entity; level recomputed")
    started = time.time()
    eng = DR.DotRuleEngine(N, ray_chunk=args.chunk, n_seeds=args.seeds, iters=args.iters)
    for dish in args.dishes:
        geom = DR.dish_geometry(dish)
        mesh = eng.build_mesh(geom["tris"])
        for tilt in DR.GRID_TILTS:
            path = level / "partial" / f"{dish}_t{tilt:03d}.npz"
            if path.is_file():
                with np.load(path) as z:
                    fresh = "obj_means" in z.files
                if fresh:
                    continue
                path.unlink()                     # a pre-object-level partial is stale for this run
                log(f"N={N} {dish} t{tilt:03d}: stale partial (no obj_means) deleted, recomputed")
            if deadline and time.time() > deadline:
                log(f"N={N}: deadline reached inside the level ({dish} t{tilt}); partials kept")
                return None, time.time() - started
            r = run_tilt(DR, eng, geom, mesh, tilt, path, args.spots)
            if r["cells"]:
                log(f"N={N} {dish} t{tilt:03d}: {r['cells']} cells, rays {r['rays']:.3e} in {r['t_rays']:.1f}s "
                    f"({r['rays'] / max(r['t_rays'], 1e-9):.3e}/s), W* {r['t_wstar']:.1f}s "
                    f"(cert {r['diag']['certified']}, unc {r['diag']['uncertified']}, binds {r['diag']['binds']}), "
                    f"above1 {sum(r['above'].values())}")
        del mesh
    t_ent = time.perf_counter()
    dot, dot_info = dot_entity(DR, eng, 1.)
    dot_neg, _ = dot_entity(DR, eng, -1.)
    t_ent = time.perf_counter() - t_ent
    meta = {"generated_utc": dt.datetime.utcnow().isoformat() + "Z", "n_points": N, "arm_radius_m": DR.GRID_ARM_R,
            "dots_per_dish": 500, "offset_m": DR.OFFSET_M, "pose": "R_y(180-tilt) about the food-contact centroid, "
            "centroid at (d, 0, h) over the disc centre; skip when any visual vertex dips below the disc plane; "
            "tilt < 0 leans the food side toward the disc centre",
            "spots_cm": [list(s) for s in args.spots] if args.spots else "every (h, d) cell",
            "occluder": "the dish's own visual mesh", "wstar_method": "exact |sum d| when every line lies within 45 deg of "
            "down; else 256x512 screen + 8 seeds (own normal, down, top-4, +-120 deg) x <= %d ascent passes, "
            "every line of the set as a seed when N <= %d, then a polish round (rotations of the best direction "
            "by +-1/2/4/8 deg and every 30 deg about z, +-2/5 deg in elevation); safety net W* >= sum c" % (
                args.iters, eng.every_line_max), "device": eng.device,
            "warp": __import__("warp").__version__, "seconds_dot_entity": t_ent, "dot_entity": dot_info,
            "object_level": "ratio of sums over the 500 dots: sum numerator / sum per-dot denominator; W* computed "
            "for every dot of a non-empty set (all_sets)"}
    grid, conv = assemble(DR, N, level, args.dishes, meta)
    grid["dot"], grid["dot_neg"] = dot, dot_neg
    grid["meta"] = meta
    conv["wall_s"] = time.time() - started
    atomic_write(level / "grid.json", json.dumps(grid, separators=(",", ":")) + "\n")
    atomic_write(level / "convergence.json", json.dumps(conv, indent=1) + "\n")
    sweep_append(args.out, N, conv)
    nC = len(DR.GRID_TILTS) * len(DR.GRID_H_CM) * len(DR.GRID_D_CM)
    for dish in args.dishes:                      # self-check: D_obj == D up to rounding; O_obj != O in general
        dl, ol = grid[dish], grid[f"{dish}_obj"]
        live = np.asarray(dl["D"]).reshape(nC, -1)[:, 0] >= 0
        dD = np.abs(np.asarray(ol["D"]) - np.asarray(dl["D"])).reshape(nC, -1)[live]
        dO = np.abs(np.asarray(ol["O"]) - np.asarray(dl["O"])).reshape(nC, -1)[live, 0]
        log(f"self-check N={N} {dish}: max |D_obj - D| = {int(dD.max()) if dD.size else 0} scaled (expected 0 or 1); "
            f"O_obj differs from O by > 1 scaled in {int((dO > 1).sum())} of {int(live.sum())} cells")
    t30 = next((t for t in conv["per_theta"] if t["theta_deg"] == 30), conv["per_theta"][0])
    log(f"[RESULT] PASS level N={N}: wall {conv['wall_s']:.0f}s, rays {conv['rays']:.3e} at {conv['rays_per_s']:.3e}/s, "
        f"kappa {conv['kappa_rays_per_dotpose_over_N']:.2f}, W* {conv['seconds_wstar']:.0f}s, binds {conv['wstar']['binds']}, "
        f"above1 {sum(conv['n_above_1'].values())}; theta 30 |NG-NJ| max {t30['gap_max']:.3e} mean {t30['gap_mean']:.3e}")
    return conv, conv["wall_s"]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--sweep", nargs=2, type=int, metavar=("N_MIN", "N_MAX"))
    ap.add_argument("--N", type=int)
    ap.add_argument("--dishes", "--kinds", nargs="+", default=["plate", "bowl", "cup"], help="names or comma lists")
    ap.add_argument("--tilts", help="A:B:S degrees, B included (a negative A needs --tilts=A:B:S); default 0:180:5")
    ap.add_argument("--spots", help='"h,d h,d ..." in cm, each a grid node; default every (h, d) cell')
    ap.add_argument("--theta", help="comma list of cone half-angles in degrees; default all ten")
    ap.add_argument("--out", type=Path, default=OUT)
    ap.add_argument("--chunk", type=int, default=1024)
    ap.add_argument("--seeds", type=int, default=8)
    ap.add_argument("--iters", type=int, default=12)
    ap.add_argument("--deadline-utc", default=None)
    args = ap.parse_args()
    args.dishes = [k for s in args.dishes for k in s.split(",") if k]
    args.spots = [tuple(int(v) for v in s.split(",")) for s in args.spots.split()] if args.spots else None
    if args.sweep:
        levels, n = [], args.sweep[0]
        while n <= args.sweep[1]:
            levels.append(n)
            n *= 4
    elif args.N:
        levels = [args.N]
    else:
        ap.error("--sweep or --N")
    B.HX().register_hotec(B.ASSETS)
    from dishsim_frigidaire import dot_rules as DR
    if args.tilts:                         # ponytail: overrides the module's grid axes for this process only
        a, b, st = (int(v) for v in args.tilts.split(":"))
        DR.GRID_TILTS = tuple(range(a, b + 1, st))
    if args.theta:
        DR.THETAS_DEG = tuple(int(v) for v in args.theta.split(","))
    off = [s for s in args.spots or () if s[0] not in DR.GRID_H_CM or s[1] not in DR.GRID_D_CM]
    if off:
        ap.error(f"--spots off the grid nodes {DR.GRID_H_CM} x {DR.GRID_D_CM}: {off}")
    deadline = deadline_of(args.deadline_utc)
    log(f"levels {levels}, dishes {args.dishes}, tilts {DR.GRID_TILTS[0]}..{DR.GRID_TILTS[-1]} ({len(DR.GRID_TILTS)}), "
        f"spots {args.spots or 'all'}, thetas {DR.THETAS_DEG}, out {args.out}, deadline {args.deadline_utc}")
    prev = None
    for N in levels:
        if prev is not None and deadline:
            proj = 4.2 * prev
            if time.time() + proj > deadline:
                log(f"[RESULT] STOP before N={N}: projected {proj / 60:.0f} min overruns the deadline")
                break
        conv, wall = run_level(DR, N, args, deadline)
        if conv is None:
            log(f"[RESULT] STOP inside N={N}: deadline")
            break
        prev = wall if wall > 0 else (conv.get("wall_s") or 0.)
    log("[RESULT] PASS sweep done")


if __name__ == "__main__":
    main()
