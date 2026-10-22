#!/usr/bin/env python3
# Copyright (c) 2026, dishsim project.
# SPDX-License-Identifier: BSD-3-Clause
"""Tilt check figure and table (stdlib only, runs on the Mac; plans/2026-10-22-tilt-score-check.md).

Reads the tilt-check grid (frigidaire_dot_rules_grid.py with --tilts/--spots; --grid is its --out or one N*/ level)
and writes into --out:
  tilt_check.svg / .png   one row per spot, one column per entity (plate, bowl, cup, the single dot): object-level
                          New, New+J and score(τ) = New+J(τ) / max_τ′ New+J(τ′) over the sweep at that spot, the argmax marked; the
                          dot-level score dashed; the plan's 0.08-0.20 band at 90 degrees. PNG via rsvg-convert.
  tilt_check.md           the plan's checks per dish and spot, then the values at -30 ... 180 degrees.

    python3 code/planner/frigidaire/frigidaire_tilt_check_figure.py --grid data/results/dot_rules/tilt_check \\
        --out data/media/dot_rules/tilt_check
Trimmed view (the page's): --spots "14,0" --tilts 0:180 --curves NJ,score draws one row, the outward tilts only and
New+J with the score (the dot-level score follows it); the score still divides by the max over the whole sweep.
"""
from __future__ import annotations

import argparse
import html
import json
import shutil
import subprocess
from pathlib import Path

TABLE_TILTS = (-30, 0, 30, 45, 60, 90, 120, 150, 180)
ENTITIES = ("plate", "bowl", "cup", "dot")
COLOUR = {"N": "#d55e00", "NJ": "#0072b2", "score": "#111111"}
PW, PH, ML, MT, CG, RG = 270, 150, 74, 128, 46, 58          # panel size and margins [px]
YMAX = 1.08


def level(path):
    p = Path(path)
    return p if (p / "grid.json").is_file() else sorted(p.glob("N*/grid.json"))[-1].parent


def curve(g, ent, rule, h, d, theta):
    """Values over g['tilts'] at one spot; None where the pose was skipped (-1)."""
    nH, nD, nQ = len(g["heights"]), len(g["distances"]), len(g["thetas"])
    hi, di, qi = g["heights"].index(h), g["distances"].index(d), g["thetas"].index(theta)
    a = [g[ent][rule][((ti * nH + hi) * nD + di) * nQ + qi] for ti in range(len(g["tilts"]))]
    return [None if v < 0 else v / g["scale"] for v in a]


def score(nj):
    m = max((v for v in nj if v is not None), default=0.)
    return [None if v is None or m <= 0 else v / m for v in nj], m


def check(tilts, nj, centre=False):
    s, m = score(nj)
    at = dict(zip(tilts, s))
    best = [t for t, v in zip(tilts, nj) if v is not None and v == m]
    nj0 = dict(zip(tilts, nj)).get(0)
    mirror = [abs(at[t] - at[-t]) for t in tilts if centre and t > 0 and -t in at and None not in (at[t], at[-t])]
    c = {"argmax": best[0] if best else None, "n_at_max": len(best), "max": m, "s0": at.get(0), "s90": at.get(90),
         "s180": at.get(180), "gain0": (m / nj0 - 1.) if nj0 else None, "mirror": max(mirror) if mirror else None}
    c["pass"] = (len(best) == 1 and c["s180"] is not None and c["s180"] < 0.005
                 and c["s90"] is not None and 0.08 <= c["s90"] <= 0.20)
    return c, s


def f(v, n=3):
    return "–" if v is None else f"{v:.{n}f}"


def spot_name(h, d):
    return f"{h} cm up, {d} cm out"


def svg_panel(x0, y0, tilts, curves, marks, title, band=True):
    X = lambda t: x0 + (t - tilts[0]) / (tilts[-1] - tilts[0]) * PW
    Y = lambda v: y0 + PH - v / YMAX * PH
    o = [f'<rect x="{x0}" y="{y0}" width="{PW}" height="{PH}" fill="#ffffff" stroke="#bbbbbb"/>']
    for t in range(-60, 181, 30):
        if tilts[0] <= t <= tilts[-1]:
            dark = t in (0, 90)
            o.append(f'<line x1="{X(t):.1f}" y1="{y0}" x2="{X(t):.1f}" y2="{y0 + PH}" stroke="{"#c8c8c8" if dark else "#ececec"}"/>')
            o.append(f'<text x="{X(t):.1f}" y="{y0 + PH + 13}" font-size="10" text-anchor="middle" fill="#555">{t}</text>')
    for v in (0, .2, .4, .6, .8, 1.):
        o.append(f'<line x1="{x0}" y1="{Y(v):.1f}" x2="{x0 + PW}" y2="{Y(v):.1f}" stroke="#ececec"/>')
        o.append(f'<text x="{x0 - 4}" y="{Y(v) + 3.5:.1f}" font-size="10" text-anchor="end" fill="#555">{v:g}</text>')
    if band:
        o.append(f'<rect x="{X(87.5):.1f}" y="{Y(.20):.1f}" width="{X(92.5) - X(87.5):.1f}" height="{Y(.08) - Y(.20):.1f}" '
                 f'fill="#2ca02c" opacity="0.35"/>')
    for vals, colour, width, dash in curves:
        seg = []
        for t, v in list(zip(tilts, vals)) + [(None, None)]:
            if v is None:
                if len(seg) > 1:
                    pts = " ".join(f"{X(a):.1f},{Y(b):.1f}" for a, b in seg)
                    da = f' stroke-dasharray="{dash}"' if dash else ""
                    o.append(f'<polyline points="{pts}" fill="none" stroke="{colour}" stroke-width="{width}"{da}/>')
                seg = []
            else:
                seg.append((t, min(v, YMAX)))
    for t in marks:
        o.append(f'<circle cx="{X(t):.1f}" cy="{Y(1.):.1f}" r="4.2" fill="#d00000" stroke="#ffffff" stroke-width="1"/>')
    o.append(f'<text x="{x0}" y="{y0 - 6}" font-size="11.5" fill="#222">{html.escape(title)}</text>')
    return o


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--grid", default="data/results/dot_rules/tilt_check")
    ap.add_argument("--out", default="data/media/dot_rules/tilt_check")
    ap.add_argument("--theta", type=int, default=30)
    ap.add_argument("--spots", default=None, help='rows to draw, e.g. "14,0 14,20" (default: every spot of the grid)')
    ap.add_argument("--tilts", default=None, help="tilt range to draw, e.g. 0:180 (default: the whole sweep)")
    ap.add_argument("--curves", default="N,NJ,score", help="object-level curves to draw, of N, NJ, score")
    ap.add_argument("--no-band", action="store_true", help="leave out the plan's 0.08-0.20 band at 90 degrees")
    a = ap.parse_args()
    lv = level(a.grid)
    g = json.loads((lv / "grid.json").read_text())
    spots = [tuple(s) for s in g["meta"]["spots_cm"]]
    if a.spots:
        spots = [tuple(int(x) for x in sp.split(",")) for sp in a.spots.split()]
        assert all(sp in [tuple(x) for x in g["meta"]["spots_cm"]] for sp in spots), spots
    tilts, th = g["tilts"], a.theta
    lo, hi = (float(x) for x in a.tilts.split(":")) if a.tilts else (tilts[0], tilts[-1])
    keep = [i for i, t in enumerate(tilts) if lo <= t <= hi]
    crop = lambda v: [v[i] for i in keep]
    tilts_draw, draw = crop(tilts), set(a.curves.split(","))
    names = [n for k, n in (("N", "New(τ)"), ("NJ", "New+J(τ)"),
                            ("score", 'score(τ) = New+J(τ) / max<tspan dy="4" font-size="11">τ′</tspan>'
                                      '<tspan dy="-4" dx="4">New+J(τ′)</tspan>')) if k in draw]
    head = (", ".join(names[:-1]) + " and " if len(names) > 1 else "") + names[-1]
    lean = ("τ: tilt of the food side away from the arm (" + ("τ &lt; 0: leaned toward the arm\'s centre; " if lo < 0 else "")
            + "0: facing the arm; 90: on edge; 180: facing away); τ′ runs over the whole sweep at the spot.")
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    W, H = ML + 4 * PW + 3 * CG + 24, MT + len(spots) * PH + (len(spots) - 1) * RG + 64
    svg = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" font-family="Helvetica, Arial, sans-serif">',
           f'<rect width="{W}" height="{H}" fill="#fafafa"/>',
           f'<text x="{ML}" y="26" font-size="17" fill="#111">{head} '
           f'(θ {th}°, N = {g["n_points"]:,} lines per set, lower arm, each dish alone)</text>',
           f'<text x="{ML}" y="46" font-size="12.5" fill="#444">{lean}</text>',
           f'<text x="{ML}" y="63" font-size="12.5" fill="#444">Gaps: poses skipped (a vertex below the disc plane). '
           f'Red dot: the argmax of New+J' + ("." if a.no_band else "; green bar: the plan\'s 0.08-0.20 band at 90°.")
           + '</text>']
    legend = [("New, object level", COLOUR["N"], 1.6, None), ("New+J, object level", COLOUR["NJ"], 1.6, None),
              ("score (object level)", COLOUR["score"], 2.4, None), ("score, dot level", "#666666", 1.2, "5 3")]
    legend = [lg for k, lg in zip(("N", "NJ", "score", "score"), legend) if k in draw]
    lx = ML
    for name, colour, width, dash in legend:
        da = f' stroke-dasharray="{dash}"' if dash else ""
        svg.append(f'<line x1="{lx}" y1="86" x2="{lx + 26}" y2="86" stroke="{colour}" stroke-width="{width}"{da}/>')
        svg.append(f'<text x="{lx + 32}" y="90" font-size="12" fill="#222">{name}</text>')
        lx += 210
    rows, checks = [], []
    for ri, (h, d) in enumerate(spots):
        y0 = MT + ri * (PH + RG)
        svg.append(f'<text x="18" y="{y0 + PH / 2}" font-size="12.5" fill="#111" text-anchor="middle" '
                   f'transform="rotate(-90 18 {y0 + PH / 2})">{spot_name(h, d)}</text>')
        block = []
        for ci, ent in enumerate(ENTITIES):
            obj = ent if ent == "dot" else f"{ent}_obj"
            N, NJ = curve(g, obj, "N", h, d, th), curve(g, obj, "NJ", h, d, th)
            c, s = check(tilts, NJ, d == 0)
            curves = [(N, COLOUR["N"], 1.6, None), (NJ, COLOUR["NJ"], 1.6, None), (s, COLOUR["score"], 2.4, None)]
            curves = [cv for k, cv in zip(("N", "NJ", "score"), curves) if k in draw]
            sd = None
            if ent != "dot":
                cd, sd = check(tilts, curve(g, ent, "NJ", h, d, th), d == 0)
                if "score" in draw:
                    curves.insert(0, (sd, "#666666", 1.2, "5 3"))
                c["dot_level"] = cd
            gain = "" if c["argmax"] in (0, None) else f" (+{100 * c['gain0']:.1f} % over 0°)"
            label = "single dot" if ent == "dot" else ent
            svg += svg_panel(ML + ci * (PW + CG), y0, tilts_draw, [(crop(v), *rest) for v, *rest in curves],
                             [c["argmax"]] if c["argmax"] is not None and lo <= c["argmax"] <= hi else [],
                             f"{label}: best {c['argmax']}°{gain}", band=not a.no_band)
            checks.append(((h, d), ent, c))
            at = {k: dict(zip(tilts, v)) for k, v in (("New", N), ("New+J", NJ), ("score", s), ("score, dot level", sd))
                  if v is not None}
            for k, row in at.items():
                block.append(f"| {label} | {k} | " + " | ".join(f(row.get(t)) for t in TABLE_TILTS) + " |")
        rows.append((h, d, block))
    svg.append(f'<text x="{ML + (4 * PW + 3 * CG) / 2}" y="{H - 22}" font-size="12.5" text-anchor="middle" fill="#333">'
               f'tilt (°)</text></svg>')
    (out / "tilt_check.svg").write_text("\n".join(svg) + "\n")
    if shutil.which("rsvg-convert"):
        subprocess.run(["rsvg-convert", "-z", "1.5", "-o", str(out / "tilt_check.png"), str(out / "tilt_check.svg")],
                       check=True)
    md = [f"# Tilt check, θ {th}°, N = {g['n_points']:,}, lower arm ({lv})", "",
          "Plan checks (object level): score = 1 at exactly one tilt; score(180) < 0.005; 0.08 ≤ score(90) ≤ 0.20; "
          "score ≤ 1 by construction. Mirror gap (over the centre only): max |score(t) − score(−t)|, expect ≈ 0.", "",
          "| spot | entity | argmax ° | tilts at max | New+J max | score 0° | best over 0° | score 90° | score 180° | "
          "pass | dot level: argmax ° / 90° / 180° | mirror gap |", "|" + "---|" * 12]
    for (h, d), ent, c in checks:
        dl = c.get("dot_level")
        gain = "–" if c["gain0"] is None or c["argmax"] == 0 else f"+{100 * c['gain0']:.2f} %"
        dlv = "–" if dl is None else f"{dl['argmax']} / {f(dl['s90'], 4)} / {f(dl['s180'], 4)}"
        name = "single dot" if ent == "dot" else ent
        md.append(f"| {spot_name(h, d)} | {name} | {c['argmax']} | {c['n_at_max']} | {f(c['max'], 4)} | "
                  f"{f(c['s0'], 4)} | {gain} | {f(c['s90'], 4)} | {f(c['s180'], 4)} | "
                  f"{'yes' if c['pass'] else 'no'} | {dlv} | {f(c['mirror'], 4)} |")
    for h, d, block in rows:
        md += ["", f"## {spot_name(h, d)}", "", "| entity | curve | " + " | ".join(f"{t}°" for t in TABLE_TILTS) + " |",
               "|" + "---|" * (2 + len(TABLE_TILTS))] + block
    (out / "tilt_check.md").write_text("\n".join(md) + "\n")
    dish = [c for _, e, c in checks if e != "dot"]
    for (h, d), ent, c in checks:
        print(f"{spot_name(h, d):>20} {ent:>5}: argmax {c['argmax']} (n {c['n_at_max']}), s0 {f(c['s0'], 4)}, "
              f"s90 {f(c['s90'], 4)}, s180 {f(c['s180'], 4)}, pass {c['pass']}")
    print(f"[RESULT] PASS wrote {out}/tilt_check.{{svg,png,md}}; plan checks met in {sum(c['pass'] for c in dish)} of "
          f"{len(dish)} dish-spot pairs", flush=True)


if __name__ == "__main__":
    main()
