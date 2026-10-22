#!/usr/bin/env python3
# Copyright (c) 2026, dishsim project.
# SPDX-License-Identifier: BSD-3-Clause
"""Figures for the Dot Score Rules page at large N (Kit-free, matplotlib Agg): the per-dot figures.

Reads the dot-rule grid (perdot_<dish>.npz, grid.json; the object-level grid's when given) and writes into --out
  dots_<dish>.png                             500 dots x 3 rules (Original / New / New+J) x 5 tilts, dish in plate bowl cup
The page builder (frigidaire_dot_rules_page.py) imports the readers below. Run from the repo root:
  code/util/run_py.sh code/planner/frigidaire/frigidaire_dot_rules_figures.py \
      --grid-dir data/results/dot_rules/grid/N0262144 \
      --grid-obj-dir data/results/dot_rules/grid_obj/N0262144 --out data/media/dot_rules/figures
A grid dir may also be the grid script's --out: its largest N*/ level holding grid.json is used. The figures of the
retired sections (convergence, random loads, object level, dependency graphs) are in git history before 2026-10-22.
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

try:                          # the figures need numpy (the box); the readers the page builder uses are stdlib only
    import numpy as np
except ImportError:
    np = None

ROOT = Path(__file__).resolve().parents[3]
RULES = ("O", "D", "DG", "DJ", "N", "NG", "NJ")
NAMES = {"O": "Original", "D": "D", "DG": "D+G", "DJ": "D+J", "N": "New", "NG": "New+G", "NJ": "New+J"}
ALIASES = {"o": "O", "orig": "O", "original": "O", "rev5": "O", "d": "D", "dg": "DG", "d+g": "DG", "gd": "DG",
           "dj": "DJ", "d+j": "DJ", "jd": "DJ", "n": "N", "new": "N", "ng": "NG", "new+g": "NG", "newg": "NG",
           "nj": "NJ", "new+j": "NJ", "newj": "NJ"}
TILTS_PERDOT = (0, 45, 90, 135, 180)
R_LOWER = 0.233                                        # exposure.ARM_SOURCES lower-arm radius [m]
RULES3 = ("O", "N", "NJ")                              # the rules shown outside the explorer
DEFAULT_ASSETS = ROOT / "data/assets/hotec_wheatstraw/usd"


def log(*a):
    print("[figures]", *a, flush=True)


def resolve(path):
    p = Path(path)
    if p.is_absolute() or p.exists():
        return p
    return ROOT / p


def read_json(path):
    with open(path) as f:
        return json.load(f)


def rule_dict(d):
    """{rule: float} from a dict keyed O..NJ (or the page's names); None if it has no rule keys."""
    if not isinstance(d, dict):
        return None
    out = {}
    for k, v in d.items():
        r = ALIASES.get(str(k).strip().lower().replace(" ", ""), k if k in RULES else None)
        if r in RULES and isinstance(v, (int, float)):
            out[r] = float(v)
    return out if out else None


# ------------------------------------------------------------------ grid / per-dot readers
def load_grid(grid_dir):
    return read_json(resolve(grid_dir) / "grid.json")


def level_dir(path):
    """The grid dir itself when it holds grid.json, else its largest N*/ level that does (the grid script's --out)."""
    p = resolve(path)
    if (p / "grid.json").is_file():
        return p
    levels = sorted(p.glob("N*/grid.json"))
    if not levels:
        raise SystemExit(f"no grid.json in {p} or in its N*/ levels")
    log("grid level", levels[-1].parent)
    return levels[-1].parent


def flat_index(grid, tilt, h, d, theta):
    ti, hi = grid["tilts"].index(tilt), grid["heights"].index(h)
    di, qi = grid["distances"].index(d), grid["thetas"].index(theta)
    return ((ti * len(grid["heights"]) + hi) * len(grid["distances"]) + di) * len(grid["thetas"]) + qi


def grid_at(grid, entity, tilt, h, d, theta):
    g, i = grid[entity], flat_index(grid, tilt, h, d, theta)
    sc = float(grid.get("scale", 1e6))
    out = {r: g[r][i] / sc for r in RULES if r in g}
    for k in ("zG", "zJ", "nG", "nJ"):
        if k in g:
            out[k] = g[k][i]
    for k in ("mDown", "mGap"):
        if k in g:
            out[k] = g[k][i] / 10.0
    return out


def load_perdot(grid_dir, dish):
    """perdot_<dish>.npz with the rule axis (scores, means, n_above1) put in RULES order."""
    z = np.load(resolve(grid_dir) / f"perdot_{dish}.npz")
    d = {k: z[k] for k in z.files}
    if "rules" in d:
        order = [list(map(str, d["rules"])).index(r) for r in RULES]
        d["scores"] = d["scores"][:, order]
        for k in ("means", "n_above1"):
            if k in d:
                d[k] = d[k][:, order]
    return d


def _find(obj, key):
    """First value of `key` anywhere in a nested dict / list (None if absent)."""
    if isinstance(obj, dict):
        if key in obj:
            return obj[key]
        for v in obj.values():
            r = _find(v, key)
            if r is not None:
                return r
    elif isinstance(obj, list):
        for v in obj:
            r = _find(v, key)
            if r is not None:
                return r
    return None


# ------------------------------------------------------------------ geometry for outlines
def kind_points(assets):
    """3-D convex-hull vertices of each HOTEC visual mesh (asset frame); projections keep the same 2-D hull."""
    sys.path.insert(0, str(ROOT / "code/frigidaire/src"))
    from dishsim_frigidaire import loading
    from scipy.spatial import ConvexHull
    pts = {}
    for k in ("plate", "bowl", "cup"):
        p = loading.visual_points(Path(assets) / f"{k}.usda")
        pts[k] = p[ConvexHull(p).vertices]
    return pts


def hull2(xy):
    from scipy.spatial import ConvexHull
    try:
        return xy[ConvexHull(xy).vertices]
    except Exception:                                      # degenerate (edge-on) projection
        return xy


# ------------------------------------------------------------------ figures
def _mpl():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams.update({"font.size": 10, "axes.titlesize": 10.5, "figure.dpi": 100, "savefig.dpi": 100})
    return plt


def fig_perdot(npz, dish, verts_local, n_points, out, rules=RULES3):
    plt = _mpl()
    from matplotlib.colors import Normalize
    from matplotlib.colorbar import ColorbarBase
    cmap, norm = plt.get_cmap("viridis"), Normalize(0.0, 1.0)
    tilts = [int(t) for t in npz["tilts"]]
    scores, sumc, nl = npz["scores"].astype(float), npz["sumc"], npz["nlines"]
    means = npz["means"] if "means" in npz else scores.mean(axis=2)
    above = npz["n_above1"] if "n_above1" in npz else (scores > 1 + 1e-9).sum(axis=2)
    h_cm = int(npz["h_cm"]) if "h_cm" in npz else 14
    d_cm = int(npz["d_cm"]) if "d_cm" in npz else 0
    theta = float(npz["theta_deg"]) if "theta_deg" in npz else 30.0
    nfmt = f"{int(n_points):,}"
    posed, sil = [], []
    for ti in range(len(tilts)):
        R, p = npz["R"][ti], npz["p"][ti]
        pos = npz["pos"][ti]
        if abs(pos[:, 2].mean() - h_cm / 100) > 0.03:     # local dots: pose them like the dish
            pos = pos @ R.T + p
        posed.append(pos)
        V = verts_local @ R.T + p
        sil.append(hull2(V[:, [0, 2]]))
    allx = np.concatenate([np.abs(s[:, 0]) for s in sil] + [np.abs(q[:, 0]) for q in posed])
    allz = np.concatenate([s[:, 1] for s in sil] + [q[:, 2] for q in posed])
    xr, zt = max(0.13, allx.max() + 0.012), allz.max() + 0.015
    names = [NAMES[r] for r in rules]
    which = ("the seven rules" if len(rules) == len(RULES) else names[0] if len(rules) == 1
             else ", ".join(names[:-1]) + " and " + names[-1])
    where = "centre" if d_cm == 0 else f"centre + {d_cm} cm"
    lines = [f"Alone, own mesh the only blocker; food-contact centre {h_cm} cm over the lower-arm disc {where}; "
             f"θ {theta:g}°, N = {nfmt} lines per dot.",
             "Seen along world Y (nearer dots on top); black bar: the disc. Tilt 180°: the dish's own orientation."]
    meaning = {"O": "Original: Σ c·open ÷ the dot's own weight Σ c over all the fixed lines.",
               "N": f"New: Σ c·open over all {nfmt} fixed lines ÷ the best-facing weight over the same lines.",
               "NG": "New+G: New over the fixed lines within θ of vertical below the dot.",
               "NJ": "New+J: New over the dot's own cone lines on the disc (its θ-circle, radius h·tan θ)."}
    lines += [meaning[r] for r in rules if r in meaning]
    if any(r in ("D", "DG", "DJ") for r in rules):
        lines.append("D, D+G, D+J: Σ c·open over all fixed lines / those within θ / the dot's own cone lines, ÷ N.")
    lines.append("One 0-to-1 scale; dark outline: a dot above 1 (counted in the title); hollow ring: a dot with no "
                 "weight (Σ c = 0, or no line in its set).")
    nc = len(rules)
    head = 0.62 + len(lines) * 10 * 1.4 / 72 + 0.7      # title, subtitle and the first row's two-line titles [in]
    W, H = 2.0 + 3.0 * nc, head + 3.05 * len(tilts)
    fig, axes = plt.subplots(len(tilts), nc, figsize=(W, H), squeeze=False)
    fig.subplots_adjust(left=0.9 / W, right=1 - 1.1 / W, top=1 - head / H, bottom=0.2 / H, wspace=0.12, hspace=0.3)
    colt = {"O": "Original (revision 5)", "D": f"D: divide by all {nfmt}", "DG": f"D+G: within θ, ÷ {nfmt}",
            "DJ": f"D+J: cone on disc, ÷ {nfmt}", "N": f"New: all {nfmt} fixed lines", "NG": "New+G: within θ",
            "NJ": "New+J: cone on disc"}
    short = {"O": "original", "D": "D", "DG": "D+G", "DJ": "D+J", "N": "New", "NG": "New+G", "NJ": "New+J"}
    rowlab = {0: "\nfacing the arm", 90: "\non edge", 180: "\nfood side up"}
    for ti, t in enumerate(tilts):
        pos = posed[ti]
        order = np.argsort(-pos[:, 1], kind="stable")      # seen along +Y: far (large y) first, near on top
        for ci, r in enumerate(rules):
            ri = RULES.index(r)                            # the npz rule axis is in RULES order (load_perdot)
            ax = axes[ti, ci]
            s = scores[ti, ri]
            if r in ("O", "D"):
                empty = sumc[ti, 0] <= 0
            elif r == "DG":
                empty = sumc[ti, 1] <= 0
            elif r == "DJ":
                empty = sumc[ti, 2] <= 0
            elif r == "NG":
                empty = nl[ti, 0] <= 0
            elif r == "NJ":
                empty = nl[ti, 1] <= 0
            else:
                empty = np.zeros(len(s), bool)
            hi = s > 1 + 1e-9
            fc = cmap(norm(np.clip(s, 0, 1)))
            fc[empty] = (0, 0, 0, 0)
            ec = fc.copy()
            ec[empty] = cmap(0.0)
            ec[hi] = (0.1, 0.05, 0.0, 1.0)
            lw = np.where(empty | hi, 0.9, 0.0)
            ax.fill(sil[ti][:, 0], sil[ti][:, 1], color="#d6d6d6", zorder=0, lw=0)
            ax.plot([-R_LOWER, R_LOWER], [0, 0], color="k", lw=3.2, solid_capstyle="butt", zorder=1)
            ax.scatter(pos[order, 0], pos[order, 2], s=11, c=fc[order], edgecolors=ec[order],
                       linewidths=lw[order], zorder=2)
            ax.set_xlim(-xr, xr)
            ax.set_ylim(-0.012, zt)
            ax.set_aspect("equal")
            ax.set_xticks([])
            ax.set_yticks([])
            for sp in ax.spines.values():
                sp.set_color("#cccccc")
            title = f"{short[r]}: {means[ti, ri]:.3f}"
            if int(above[ti, ri]) > 0:
                title += f"  ({int(above[ti, ri])} above 1)"
            ax.set_title((colt[r] + "\n" + title) if ti == 0 else title, fontsize=10.5)
        axes[ti, 0].set_ylabel(f"tilt {t}°" + rowlab.get(t, ""), fontsize=11)
    cax = fig.add_axes([1 - 0.85 / W, 0.12, 0.17 / W, 0.62])
    ColorbarBase(cax, cmap=cmap, norm=norm, extend="max", label="dot score")
    fig.text(0.9 / W, 1 - 0.2 / H, f"HOTEC v2 {dish}: the 500 dot scores under {which}, tilt 0° to 180°",
             fontsize=15, va="top")
    fig.text(0.9 / W, 1 - 0.62 / H, "\n".join(lines), fontsize=10, va="top", color="#555555", linespacing=1.4)
    fig.savefig(out)
    plt.close(fig)
    log("wrote", out)


# ------------------------------------------------------------------ driver
def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--grid-dir", default=None)
    ap.add_argument("--grid-obj-dir", default=None,
                    help="object-level grid (<dish>_obj entities, cup): the per-dot figures use its perdot_<dish>.npz "
                         "instead of --grid-dir's")
    ap.add_argument("--assets", default=str(DEFAULT_ASSETS))
    ap.add_argument("--out", default="data/media/dot_rules/figures")
    a = ap.parse_args()
    out = resolve(a.out)
    out.mkdir(parents=True, exist_ok=True)
    kpts = kind_points(a.assets)
    gsrc = a.grid_obj_dir or a.grid_dir
    if gsrc:
        pdir = level_dir(gsrc)
        grid = load_grid(pdir)
        for dish in ("plate", "bowl", "cup"):
            if (pdir / f"perdot_{dish}.npz").is_file():
                fig_perdot(load_perdot(pdir, dish), dish, kpts[dish], grid["n_points"], out / f"dots_{dish}.png")
    print(f"[RESULT] PASS figures in {out}", flush=True)


def run_main(main, name):
    """Run a script's main so that the last stdout line is [RESULT] PASS (main's own) or [RESULT] FAIL."""
    import traceback
    try:
        main()
    except SystemExit as e:
        if e.code in (None, 0):
            raise
        if not isinstance(e.code, int):
            print(e.code, file=sys.stderr, flush=True)
        print(f"[RESULT] FAIL {name}: {e.code}", flush=True)
        sys.exit(e.code if isinstance(e.code, int) else 1)
    except Exception as e:                                  # report it, then exit non-zero
        traceback.print_exc()
        print(f"[RESULT] FAIL {name}: {type(e).__name__}: {e}", flush=True)
        sys.exit(1)


if __name__ == "__main__":
    run_main(main, "figures")
