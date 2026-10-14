#!/usr/bin/env python3
# Copyright (c) 2026, dishsim project.
# SPDX-License-Identifier: BSD-3-Clause
"""Dot-rule engine: two one-shot sanity checks, throughput calibration and the N_max choice (Kit-free, GPU).

R1  revision-5 Original equality on the smoke load at N=64, theta 30 (S_O 0.1604257 +- 1e-6, per dish 1e-6).
R4  W* search >= a numpy port of the page's every-line best() on ~1,000 uncertified problems at N=256
    (the engine's direction evaluated in fp64 on the same line set; no shortfall > 1e-9), no dot above 1 + 1e-9.
Then rho_dish, rho_load, rho_W at N=16,384 and N_max = the largest of {65536, 262144, 1048576} with
T_sweep(N_max) + 300 T_load(N_max) <= 0.85 (hours until 11:45 UTC), written to data/results/dot_rules/calibration.json.

    code/util/run_py.sh code/planner/frigidaire/frigidaire_dot_rules_bench.py
"""
from __future__ import annotations

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
import frigidaire_dot_rules_loads as L  # noqa: E402

SMOKE = ROOT / "data/results/hotec/frigidaire/top5_20260929_touch08/settled"
OUT = ROOT / "data/results/dot_rules/calibration.json"
GRID64 = ROOT / "data/results/dot_rules/grid/N0000064/convergence.json"
CANDIDATES = (65536, 262144, 1048576)
DOTPOSES = 2032000
CAL_N = 16384
GOLD = math.pi * (3. - math.sqrt(5.))


def log(*a):
    print(f"[{dt.datetime.utcnow():%H:%M:%S}]", *a, flush=True)


# --------------------------------------------------------------------------- R1

def r1(DR):
    eng = DR.DotRuleEngine(64)
    layout = json.loads((SMOKE / "start_00.layout.json").read_text())
    ref = json.loads((SMOKE / "start_00.json").read_text())
    rec = DR.score_load(eng, L.entries_of(layout), 30., "start_00")
    refd = {o["id"]: o["exposure"] for o in ref["per_object"]}
    dev = max(abs(d["E"]["O"] - refd[d["id"]]) for d in rec["per_dish"])
    s_dev = abs(rec["S"]["O"] - 0.1604257)
    s_ref = abs(rec["S"]["O"] - ref["S_settled"])
    ok = bool(s_dev <= 1e-6 and dev <= 1e-6 and len(rec["per_dish"]) == len(refd))
    log(f"[{'PASS' if ok else 'FAIL'}] R1 S_O {rec['S']['O']:.9f} (target 0.1604257, |d| {s_dev:.2e}; vs file "
        f"{ref['S_settled']:.9f} |d| {s_ref:.2e}), max per-dish |d| {dev:.2e}")
    return ok, {"S_O": rec["S"]["O"], "S_O_reference": ref["S_settled"], "abs_dev_S_vs_0.1604257": s_dev,
                "max_abs_dev_per_dish": dev, "n_dishes": len(rec["per_dish"]), "S_all_rules": rec["S"], "pass": ok}


# --------------------------------------------------------------------------- R4

def sphere400():
    i = np.arange(400)
    z = 1 - 2 * (i + .5) / 400
    r = np.sqrt(1 - z * z)
    return np.column_stack((r * np.cos(i * GOLD), r * np.sin(i * GOLD), z))


def page_best(D, S400):
    """numpy port of the page's best(): starts = mean, every line, top-3 of 400 sphere directions; <= 50 steps."""
    if not len(D):
        return 0.
    s = D.sum(axis=0)
    starts = [s / np.linalg.norm(s)] if np.linalg.norm(s) > 0 else []
    w400 = np.maximum(S400 @ D.T, 0).sum(axis=1)
    top = np.argsort(-w400, kind="stable")[:3]
    m = np.vstack(starts + [D, S400[top]])
    prev = None
    done = np.zeros(len(m), bool)
    for _ in range(50):
        F = (m @ D.T) > 0
        if prev is not None:
            done |= (F == prev).all(axis=1)
        if done.all():
            break
        V = F.astype(float) @ D
        Ln = np.linalg.norm(V, axis=1)
        upd = ~done & (Ln > 0)
        done |= ~done & (Ln == 0)
        m[upd] = V[upd] / Ln[upd, None]
        prev = F
    return float(np.maximum(m @ D.T, 0).sum(axis=1).max())


def refine(D, m):
    """W at the fp64 fixed point of the engine's basin: m <- V_S/|V_S| from the engine's fp32 direction."""
    m = np.asarray(m, float)
    m = m / max(np.linalg.norm(m), 1e-300)
    w = np.maximum(D @ m, 0).sum()
    for _ in range(50):
        V = D[(D @ m) > 0].sum(axis=0)
        if not np.linalg.norm(V):
            break
        m2 = V / np.linalg.norm(V)
        w = max(w, np.maximum(D @ m2, 0).sum())
        if np.array_equal((D @ m2) > 0, (D @ m) > 0):
            break
        m = m2
    return float(w)


def line_set(DR, U32, U64, rel, R, kind, th):
    """The exact lines of a set: membership in fp32 as the kernels, directions in fp64."""
    r32 = rel.astype(np.float32)
    R32 = np.float32(R)
    h32 = r32[2]
    if kind == 2:
        a32 = h32 * np.float32(math.tan(math.radians(th)))
        x32, y32 = a32 * U32[:, 0], a32 * U32[:, 1]
        keep = (r32[0] + x32) ** 2 + (r32[1] + y32) ** 2 <= R32 * R32
        x, y = float(a32) * U64[:, 0], float(a32) * U64[:, 1]
    else:
        x32, y32 = R32 * U32[:, 0] - r32[0], R32 * U32[:, 1] - r32[1]
        L32 = np.sqrt(x32 * x32 + y32 * y32 + h32 * h32)
        keep = np.ones(len(U32), bool) if kind == 0 else (h32 / L32 >= np.float32(math.cos(math.radians(th))))
        x, y = R * U64[:, 0] - rel[0], R * U64[:, 1] - rel[1]
    h = rel[2]
    D = np.column_stack((x, y, np.full(len(x), -h)))
    D /= np.linalg.norm(D, axis=1, keepdims=True)
    return D[keep]


def r4(DR, n=256, per_kind=350, seed=0, every_line_max=256):
    import torch
    eng = DR.DotRuleEngine(n, every_line_max=every_line_max)
    TH = DR.THETAS_DEG
    U64 = DR.E.disk_points((0., 0., 0.), 1., n)[:, :2]
    U32 = U64.astype(np.float32)
    S400 = sphere400()
    rng = np.random.default_rng(seed)
    probs, above_total = [], {k: 0 for k in DR.RULES}
    for dish, tilts in (("plate", (45, 135)), ("bowl", (90, 160))):
        geom = DR.dish_geometry(dish)
        mesh = eng.build_mesh(geom["tris"])
        for tilt in tilts:
            cells = DR.grid_cells(geom, tilt)
            dots = DR.grid_dots(geom, tilt, cells)
            s = eng.ray_sums(mesh, dots, TH)                        # with blocking: realistic score check
            WF, WG, WJ, mF, _ = eng.wstar(dots, s, TH)
            _, mG, mJ = eng.last_m
            _, _, _, above = eng.scores(s, WF, WG, WJ)
            for k in DR.RULES:
                above_total[k] += above[k]
            sub = rng.choice(len(dots.radius), size=min(len(dots.radius), 400), replace=False)
            f_c, g_n, j_n = s.f_cv.cpu().numpy(), s.g_n.cpu().numpy(), s.j_n.cpu().numpy()
            g_cv, j_cv = s.g_cv.cpu().numpy(), s.j_cv.cpu().numpy()
            WFn, WGn, WJn = WF.cpu().numpy(), WG.cpu().numpy(), WJ.cpu().numpy()
            mFn, mGn, mJn = mF.cpu().numpy(), mG.cpu().numpy(), mJ.cpu().numpy()
            for i in sub:
                rel, R, nrm = dots.rel[i], dots.radius[i], dots.nrm[i]
                h, rho = rel[2], math.hypot(rel[0], rel[1])
                if f_c[i] > 0 and R + rho > h:                       # solved problems only (cv > 0)
                    probs.append((0, None, rel, R, nrm, WFn[i], mFn[i], n))
                for q in range(6, 10):
                    a = h * math.tan(math.radians(TH[q]))
                    unc = min(a, R + rho) > h
                    if unc and g_cv[i, q] > 0 and 0 < g_n[i, q] < n:
                        probs.append((1, TH[q], rel, R, nrm, WGn[i, q], mGn[i, q], g_n[i, q]))
                    if unc and j_cv[i, q] > 0 and j_n[i, q] > 0:
                        probs.append((2, TH[q], rel, R, nrm, WJn[i, q], mJn[i, q], j_n[i, q]))
        del mesh
    by_kind = {k: [p for p in probs if p[0] == k] for k in (0, 1, 2)}
    chosen = []
    for k, lst in by_kind.items():
        idx = rng.choice(len(lst), size=min(per_kind, len(lst)), replace=False)
        chosen += [lst[j] for j in idx]
    worst, mism, rel_sum_dev, n_cmp, t0 = 0., 0, 0., 0, time.time()
    for kind, th, rel, R, nrm, W_eng, m_eng, cnt in chosen:
        D = line_set(DR, U32, U64, rel, R, kind, th)
        if len(D) != cnt:
            mism += 1
            continue
        Wp = page_best(D, S400)
        We = max(refine(D, m_eng), np.maximum(D @ nrm, 0).sum())
        if Wp > 0:
            worst = max(worst, (Wp - We) / Wp)
            rel_sum_dev = max(rel_sum_dev, abs(W_eng - Wp) / Wp)
        n_cmp += 1
    above = sum(above_total.values())
    ok = bool(worst <= 1e-9 and above == 0 and n_cmp >= 0.9 * len(chosen))
    if os.environ.get("DOT_RULES_R4_ONLY"):
        sf = []
        for kind, th, rel, R, nrm, W_eng, m_eng, cnt in chosen:
            D = line_set(DR, U32, U64, rel, R, kind, th)
            Wp = page_best(D, S400)
            sf.append(((Wp - max(refine(D, m_eng), np.maximum(D @ nrm, 0).sum())) / Wp, kind, th, rel[2]))
        sf.sort(key=lambda t: -t[0])
        log("worst", [(f"{a:.1e}", k, t, round(float(h), 3)) for a, k, t, h in sf[:12]],
            "n>1e-9", sum(a > 1e-9 for a, *_ in sf), "n>1e-6", sum(a > 1e-6 for a, *_ in sf))
    log(f"[{'PASS' if ok else 'FAIL'}] R4{'' if every_line_max >= n else ' (diagnostic: large-N search only)'} N={n} {n_cmp} problems (F {len(by_kind[0])}, G {len(by_kind[1])}, J {len(by_kind[2])} "
        f"available; {mism} set-size mismatches skipped): max shortfall vs page best() {worst:.2e}, "
        f"max |W_engine(fp32 sums) - W_page|/W_page {rel_sum_dev:.2e}, dots above 1: {above} ({time.time() - t0:.0f}s)")
    return ok, {"N": n, "every_line_seeds": every_line_max >= n, "problems_compared": n_cmp, "set_size_mismatches": mism, "max_shortfall_rel": worst,
                "max_abs_rel_dev_fp32_sums": rel_sum_dev, "dots_above_1": above_total, "pass": ok}


# --------------------------------------------------------------------------- throughput

def throughput(DR):
    import warp as wp
    eng = DR.DotRuleEngine(CAL_N)
    batches, rays, t_rays, t_w, n_dp, t_host = [], 0., 0., 0., 0, 0.
    for dish in ("plate", "bowl"):
        geom = DR.dish_geometry(dish)
        mesh = eng.build_mesh(geom["tris"])
        for tilt in (0, 45, 90, 135):
            t0 = time.perf_counter()
            cells = DR.grid_cells(geom, tilt)
            dots = DR.grid_dots(geom, tilt, cells)
            th0 = time.perf_counter()
            s = eng.ray_sums(mesh, dots, DR.THETAS_DEG)
            t1 = time.perf_counter()
            WF, WG, WJ, _, diag = eng.wstar(dots, s, DR.THETAS_DEG)
            t2 = time.perf_counter()
            sc = eng.scores(s, WF, WG, WJ)
            _ = {k: v.cpu().numpy() for k, v in sc[0].items()}
            t3 = time.perf_counter()
            r = float(s.rays.sum())
            batches.append({"dish": dish, "tilt": tilt, "dotposes": len(dots.radius), "rays": r, "t_rays": t1 - th0,
                            "t_wstar": t2 - t1, "kappa": r / len(dots.radius) / CAL_N})
            rays += r
            t_rays += t1 - th0
            t_w += t2 - t1
            n_dp += len(dots.radius)
            t_host += (th0 - t0) + (t3 - t2)
            log(f"  {dish} t{tilt}: {len(dots.radius)} dot-poses, rays {r:.3e} in {t1 - th0:.2f}s "
                f"({r / (t1 - th0):.3e}/s, kappa {r / len(dots.radius) / CAL_N:.2f}), W* {t2 - t1:.2f}s")
        del mesh
    rho_dish = rays / t_rays
    kappa_s = rays / n_dp / CAL_N
    tw_dp = t_w / n_dp / CAL_N                                   # W* seconds per dot-pose per unit N
    t_batch = t_host / len(batches) + 0.3
    rho_w = eng.stats["vsum_terms"] / max(eng.stats["vsum_s"], 1e-9)
    # load: smoke soup (602k triangles), BVH constructors, rays and W* at theta 30
    layout = json.loads((SMOKE / "start_00.layout.json").read_text())
    entries = L.entries_of(layout)
    t0 = time.perf_counter()
    dots, owner, recs, objs = DR.load_dots(entries)
    soup = DR.load_soup(entries, objs)
    t_soup = time.perf_counter() - t0
    bvh = {}
    for cons in (None, "sah", "lbvh"):
        t0 = time.perf_counter()
        mesh = eng.build_mesh(soup, cons)
        wp.synchronize_device(eng.device)
        bvh[str(cons)] = time.perf_counter() - t0
        del mesh
    rec = DR.score_load(eng, entries, 30., "start_00")
    rho_load = rec["rays"] / rec["seconds"]["rays"]
    kappa_load = rec["rays"] / 12000. / CAL_N
    tw_load = rec["seconds"]["wstar"] / CAL_N
    t_load_fixed = rec["seconds"]["soup"] + rec["seconds"]["bvh"] + 1.0     # + pools, I/O, conversion
    log(f"  load: rays {rec['rays']:.3e} in {rec['seconds']['rays']:.2f}s ({rho_load:.3e}/s, kappa {kappa_load:.2f}), "
        f"W* {rec['seconds']['wstar']:.2f}s, soup {rec['seconds']['soup']:.2f}s, bvh {rec['seconds']['bvh']:.2f}s, "
        f"bvh builds {bvh}")
    kappa_grid = kappa_s
    if GRID64.is_file():
        kappa_grid = json.loads(GRID64.read_text())["kappa_rays_per_dotpose_over_N"]

    def T_grid(N):
        return (kappa_grid * N * DOTPOSES / rho_dish + tw_dp * N * DOTPOSES + 74 * t_batch + 10.)

    def T_sweep(Nmax):
        done = {64} if GRID64.is_file() else set()
        return sum(T_grid(N) for N in (64 * 4 ** k for k in range(12)) if N <= Nmax and N not in done)

    def T_load(N):
        return kappa_load * 12000 * N / rho_load + tw_load * N + t_load_fixed

    now = dt.datetime.now(dt.timezone.utc)
    end = now.replace(hour=11, minute=45, second=0, microsecond=0)
    budget = 0.85 * (end - now).total_seconds()
    proj = {N: {"T_grid_s": T_grid(N), "T_sweep_s": T_sweep(N), "T_load_s": T_load(N),
                "T_loads_300_s": 300 * T_load(N), "T_total_s": T_sweep(N) + 300 * T_load(N)} for N in CANDIDATES}
    fits = [N for N in CANDIDATES if proj[N]["T_total_s"] <= budget]
    n_max = max(fits) if fits else CANDIDATES[0]
    for N in CANDIDATES:
        p = proj[N]
        log(f"  N={N}: sweep {p['T_sweep_s'] / 60:.1f} min, load {p['T_load_s']:.1f}s, 300 loads "
            f"{p['T_loads_300_s'] / 60:.1f} min, total {p['T_total_s'] / 60:.1f} min vs budget {budget / 60:.1f} min")
    return n_max, {"cal_N": CAL_N, "rho_dish_rays_per_s": rho_dish, "kappa_sample": kappa_s, "kappa_grid_N64": kappa_grid,
                   "wstar_s_per_dotpose_per_N": tw_dp, "rho_W_term_seeds_per_s": rho_w, "t_batch_host_s": t_batch,
                   "batches": batches, "rho_load_rays_per_s": rho_load, "kappa_load": kappa_load,
                   "wstar_load_s_per_N": tw_load, "t_load_fixed_s": t_load_fixed, "t_soup_s": t_soup,
                   "bvh_build_s": bvh, "load_seconds_at_cal_N": rec["seconds"], "load_rays_at_cal_N": rec["rays"],
                   "budget_s": budget, "budget_rule": "0.85 * seconds from now until 11:45 UTC",
                   "now_utc": now.isoformat(), "projection": {str(k): v for k, v in proj.items()},
                   "candidates": list(CANDIDATES), "fits": fits}


def main():
    B.HX().register_hotec(B.ASSETS)
    from dishsim_frigidaire import dot_rules as DR
    t0 = time.time()
    ok1, rep1 = r1(DR)
    ok4, rep4 = r4(DR)
    _, rep4d = r4(DR, every_line_max=0)                         # non-gating: what the large-N search alone achieves
    _, rep4e = r4(DR, n=1024, per_kind=60, every_line_max=0)
    rep4["diagnostic_search_only"] = {"N256": rep4d, "N1024": rep4e}
    if os.environ.get("DOT_RULES_R4_ONLY"):
        return
    n_max, cal = throughput(DR)
    import warp as wp
    out = {"generated_utc": dt.datetime.utcnow().isoformat() + "Z", "N_max": n_max, "sanity": {"R1": rep1, "R4": rep4},
           "throughput": cal, "warp": wp.__version__, "device": "cuda:0", "wall_s": time.time() - t0}
    OUT.parent.mkdir(parents=True, exist_ok=True)
    tmp = OUT.with_suffix(".json.tmp")
    tmp.write_text(json.dumps(out, indent=1, default=lambda o: o.item() if hasattr(o, "item") else str(o)) + "\n")
    os.replace(tmp, OUT)
    ok = ok1 and ok4
    print(f"[RESULT] {'PASS' if ok else 'FAIL'} bench: R1 {'pass' if ok1 else 'FAIL'}, R4 {'pass' if ok4 else 'FAIL'}, "
          f"N_max {n_max}, rho_dish {cal['rho_dish_rays_per_s']:.3e}, rho_load {cal['rho_load_rays_per_s']:.3e}, "
          f"rho_W {cal['rho_W_term_seeds_per_s']:.3e} -> {OUT.relative_to(ROOT)}", flush=True)


if __name__ == "__main__":
    main()
