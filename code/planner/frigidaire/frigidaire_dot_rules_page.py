#!/usr/bin/env python3
# Copyright (c) 2026, dishsim project.
# SPDX-License-Identifier: BSD-3-Clause
"""Build the Dot Score Rules page at large N (Kit-free).

Template: the 8 October page (docs/Dot Score Rules/index.html). Kept: its <head>, CSS tokens (dark mode
included), the explorer markup and the chart JS (edited: the dot is read from the grid, N replaces 64, and a
dot | object level toggle and the cup when the grid has them). Rewritten from the result files: header tiles,
the rules (three: Original, New, New+J), per-dot, the object level, the random Isaac loads, the Isaac scenes,
the Isaac dependency graphs (the ordered-pair test), the G = J appendix.
Run from the repo root after frigidaire_dot_rules_figures.py (same --grid-obj-dir and --dependency-dir):
  code/util/run_py.sh code/planner/frigidaire/frigidaire_dot_rules_page.py \
      --grid-dir data/results/dot_rules/grid/N0262144 \
      --grid-obj-dir data/results/dot_rules/grid_obj/N0262144 \
      --sweep data/results/dot_rules/grid/convergence_sweep.json \
      --loads-dir data/results/dot_rules/loads/N0262144 \
      --dependency-dir data/results/dot_rules/dependency/pairs \
      --farm-dir data/results/random_loads/rl_20261009 \
      --figures-dir data/media/dot_rules/figures --out data/media/dot_rules/page
Without --grid-obj-dir the page reads --grid-dir's grid and the object-level section is a note; without
--dependency-dir the dependency section is left out. Each dependency JSON with edges is copied to <out>/data/ and
linked from its caption; a load the joint gate rejected has none, so it is neither copied nor linked. Optional:
--rerun-dir (a second pairwise run of the same loads: an edges-reproduced column and sentence) and --resettled-dir
(goals re-settled by a sequential build: a sub-section closing the dependency section; the figures script draws their
depgraph_<id>_resettled.png with the same flag).
"""
from __future__ import annotations

import argparse
import html
import json
import math
import shutil
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import frigidaire_dot_rules_figures as F  # noqa: E402

RULES, NAMES, RULES3 = F.RULES, F.NAMES, F.RULES3
TEMPLATE = F.ROOT / "docs/Dot Score Rules/index.html"
SUP = str.maketrans("-0123456789", "⁻⁰¹²³⁴⁵⁶⁷⁸⁹")
MB = 1024 * 1024


def sub1(text, old, new):
    n = text.count(old)
    if n != 1:
        raise SystemExit(f"template edit: expected one match, found {n}: {old[:80]!r}")
    return text.replace(old, new)


def between(text, start, end, include_end=True):
    i = text.index(start)
    j = text.index(end, i)
    return i, (j + len(end) if include_end else j)


def sci(v):
    """0.0123 -> 0.0123; 4.1e-6 -> 4.1·10⁻⁶ (old page style for small numbers)."""
    if v is None or not np.isfinite(v):
        return "–"
    if v == 0:
        return "0"
    if abs(v) >= 0.001:
        return f"{v:.4f}".rstrip("0").rstrip(".") if abs(v) < 0.01 else f"{v:.3f}"
    e = int(math.floor(math.log10(abs(v))))
    return f"{v / 10 ** e:.1f}·10{str(e).translate(SUP)}"


f3 = F.fmt3


def n_(v):
    return f"{int(v):,}"


def esc(s):
    return html.escape(str(s), quote=True)


# ------------------------------------------------------------------ data
def grid_payload(grid):
    keep = ("tilts", "heights", "distances", "thetas", "scale", "dots", "n_points")
    out = {k: grid[k] for k in keep}
    for ent in ("plate", "bowl") + (("cup",) if "cup" in grid else ()):
        out[ent] = {k: grid[ent][k] for k in RULES + ("zG", "zJ")}
        if f"{ent}_obj" in grid:                   # object level: the explorer toggle shows Original, New, New+J only
            out[f"{ent}_obj"] = {k: grid[f"{ent}_obj"][k] for k in RULES3}
    dot = grid.get("dot")
    if not dot:
        raise SystemExit("grid.json has no 'dot' entity")
    dkeys = RULES + ("zG", "zJ", "nG", "nJ", "mDown", "mGap")
    out["dot"] = {k: dot[k] for k in dkeys if k in dot}
    for alias in ("dot_neg", "dot_mirror", "dotneg", "dot_mirrored"):
        if grid.get(alias):
            out["dot_neg"] = {k: grid[alias][k] for k in dkeys if k in grid[alias]}
            break
    return out


def above_and_max(grid, grid_dir, perdot):
    """(dots above 1, largest New dot, scope words) from the grid's own diagnostics; per-dot poses otherwise."""
    sources = [grid.get("meta", {})]
    cj = F.resolve(grid_dir) / "convergence.json"
    if cj.exists():
        sources.append(F.read_json(cj))
    above = mx = None
    for src in sources:
        for key in ("n_above1", "dots_above_1", "n_above_1", "above1", "count_above_1"):
            v = F._find(src, key)
            if v is not None and above is None:
                above = int(sum(_flat_nums(v)))
        for key in ("max_dot", "max_dot_score", "max_score", "dot_max_score", "max_new_dot", "max_dot_N"):
            v = F._find(src, key)
            if v is not None and mx is None:
                if isinstance(v, dict):
                    v = (F.rule_dict(v) or {}).get("N")
                if isinstance(v, (int, float)):
                    mx = float(v)
    if above is not None and mx is not None:
        return above, mx, "grid"
    a = sum(int(np.sum(p["n_above1"])) for p in perdot.values())
    m = max(float(np.max(p["scores"][:, RULES.index("N")])) for p in perdot.values())
    return (above if above is not None else a), (mx if mx is not None else m), "perdot"


def _flat_nums(v):
    if isinstance(v, (int, float)):
        return [v]
    if isinstance(v, dict):
        return [x for u in v.values() for x in _flat_nums(u)]
    if isinstance(v, list):
        return [x for u in v for x in _flat_nums(u)]
    return []


def slope(xs, ys):
    xs, ys = np.asarray(xs, float), np.asarray(ys, float)
    ok = (xs > 0) & (ys > 0)
    if ok.sum() < 2:
        return float("nan")
    return float(np.polyfit(np.log(xs[ok]), np.log(ys[ok]), 1)[0])


# ------------------------------------------------------------------ sections
def dep_scope(d):
    """Who the dependency graphs cover: the five best restricted loads, or their count when the dependency dir holds
    loads the picks do not rank."""
    ids = [x["dep"]["load_id"] for x in d["deps"]]
    return "the five best restricted loads" if set(ids) <= set(d["dep_picks"]) else f"{len(ids)} restricted loads"


def header(d):
    P0, P90 = d["P0"], d["P90"]
    above, mx, scope = d["above"], d["maxN"], d["above_scope"]
    nposes = sum(int(p["scores"].shape[0] * p["scores"].shape[2]) for p in d["pd"].values())
    where = (f"in all seven rules over {n_(d['poses'])} dish poses" if scope == "grid"
             else f"in all seven rules over the {n_(nposes)} dot-poses of the per-dot figures")
    draft = " · DRAFT, synthetic inputs" if d["draft"] else ""
    if d["has_obj"]:
        obj = ("A dish has two scores: the dot level, the mean of its 500 dot scores, and the object level, the sum of "
               "its 500 numerators over the sum of its 500 denominators, a ratio of sums in which each dot counts in "
               "proportion to its denominator. The explorer switches between the two levels and adds the cup.")
    else:
        obj = ("A dish will also get an object-level score, the sum of its 500 numerators over the sum of its 500 "
               "denominators (a ratio of sums); that grid, with the cup, is being computed, and the explorer toggle "
               "comes with it.")
    n, ok = len(d["deps"]), sum(x["dep"].get("status") != "gate" for x in d["deps"])   # loads past the joint gate
    deps = (f" For {dep_scope(d)}, Isaac first settles all 24 dishes together (the joint gate), then "
            "drops close pairs of dishes in both orders into that settled load to find which dish must go in before "
            "which, and builds the load one dish at a time in that order: one dependency graph per load, with the "
            "settled and built poses rescored." if d["deps"] and ok == n else
            f" For {dep_scope(d)}, Isaac first settles all 24 dishes together (the joint gate); for the {ok} that pass "
            "it, Isaac then drops close pairs of dishes in both orders into that settled load to find which dish must go "
            "in before which, and builds the load one dish at a time in that order: one dependency graph per tested "
            "load, with the settled and built poses rescored." if d["deps"] else "")
    lead = ("A dot's score says how exposed it is to the lower spray arm. The original rule (revision 5) can rank a dot "
            "that sees one spray point above a dot that sees all of them. New divides by the best-facing dot at the "
            "same spot instead of by the dot's own weight, and New+J does the same over near-vertical jets only. The "
            "page compares these three rules; the explorer and the appendix keep all seven. Every result uses "
            f"N = {d['Nf']} lines per set. {obj} Then completely random 24-dish loads, each settled and gated in "
            "Isaac and ranked under the three rules, and the five best and five worst loads under New+J from two "
            f"samplers, shown in the Isaac scene.{deps} The appendix proves that New+G and New+J become the same "
            "rule as N grows.")
    return f"""<header class="top">
  <p class="eyebrow">Dishwasher exposure score · 14 October 2026{draft}</p>
  <h1>Dot score: Original, New and New+J</h1>
  <p class="lead">{esc(lead)}</p>
  <div class="tiles">
    <div class="tile"><p class="t-label">Plate on edge</p><p class="t-value">{P90['O']:.3f} → {P90['N']:.3f}</p><p class="t-sub">original → New. The edge case: rim dots that face few spray points no longer score near 1</p></div>
    <div class="tile"><p class="t-label">Plate facing the arm</p><p class="t-value">{P0['N']:.3f}</p><p class="t-sub">under New (original {P0['O']:.3f}). The best pose keeps a score near the top</p></div>
    <div class="tile"><p class="t-label">Dots above 1</p><p class="t-value">{above}</p><p class="t-sub">{where}; the largest New dot is {mx:.6f}</p></div>
  </div>
</header>"""


def rules(d):
    N, Nf, dt = d["N"], d["Nf"], d["dot14"]
    Dd, Nd = dt["D"], dt["N"]
    wstar = Dd * N / Nd if Nd > 0 else float("nan")
    return f"""<section>
  <p class="eyebrow">The rules</p>
  <h2>One numerator, three rules</h2>
  <p class="prose">Each dot sends lines to the spray arm's disc (radius R = 23.3 cm). Every rule adds up the same thing on top: the cosine of each open line against the dot's normal. Original and New differ only in what they divide by; New+J is New over a smaller set of lines, the near-vertical jets.</p>
  <div class="formula">d_k      unit direction from the dot to the end of line k
c_k      = max(0, n·d_k)          <i>cosine between line k and the dot's normal n; 0 from behind</i>
v_k      = 1 if line k is open, 0 if something blocks it
W(m)     = Σ_k max(0, m·d_k)      <i>the weight this spot would give a dot whose normal were m</i>

Original = Σ_k c_k v_k / W(n)            <i>÷ the dot's own weight</i>
New      = Σ_k c_k v_k / max_m W(m)      <i>÷ the best-facing dot at the same spot</i>
New+J    = New's formula over the J lines: both sums run over the same smaller set

lines k  Original, New: the N fixed spray points on the disc (a sunflower), N = {Nf}
         G: those of the N within θ of vertical below the dot (the explorer's G rules)
         J: the dot's own N points inside its θ-circle (radius h·tan θ), those that land on the disc
empty    a dot with no line, or W = 0, scores 0</div>
  <div class="plate">
    <div class="key-table">
      <span class="nm"><span class="line-key thick" style="color:var(--v-o)"></span>Original</span><span class="sw"><b>Of the spray that could reach me, how much is unblocked?</b> It divides by the dot's own weight, so it cannot see how little could reach. A dot facing one spray point and keeping it scores 1.</span>
      <span class="nm"><span class="line-key" style="color:var(--v-jd)"></span>New</span><span class="sw"><b>How much reaches me, as a share of the most that could reach this spot?</b> The reference is the same spot with the normal turned to face the arm as well as possible, with nothing in the way. A dot that faces the best way and is unblocked scores 1 at any height or position.</span>
      <span class="nm"><span class="line-key" style="color:var(--v-j)"></span>New+J</span><span class="sw">New, counting only near-vertical jets: {Nf} lines of the dot's own, spread inside its θ-circle. Where the circle lies on the disc, the best-facing dot faces straight down and the reference is a constant (N × {dt['DJ']:.3f} at 30°).</span>
    </div>
  </div>
  <div class="scroll-x">
    <table class="cases">
      <thead><tr><th>14 cm over the disc centre, θ 30°, N = {Nf}</th><th>Original</th><th>New</th></tr></thead>
      <tbody>
        <tr><td>Case 1: one spray point straight ahead, kept; the other {n_(N - 1)} behind</td><td>1</td><td>{sci(1.0 / wstar)}</td></tr>
        <tr><td>Case 2: facing the arm, all {Nf} in front, half of the weight blocked</td><td>0.5</td><td>{0.5 * Nd:.2f}</td></tr>
        <tr><td>Ideal: facing the arm, nothing blocked</td><td>1</td><td>{Nd:.3f}</td></tr>
      </tbody>
    </table>
  </div>
  <p class="prose">The original puts case 1 above case 2. New puts them in the right order and keeps the ideal dot at 1. With {Nf} lines one spray point is worth almost nothing under New: a dot counts as exposed only when a broad share of the arm reaches it. The maximum over m is pure geometry: it needs no extra ray cast. For θ ≤ 45° the New+J reference is exact with no search: every such line lies within 45° of straight down, so no two lines are more than 90° apart, and then the best-facing weight is the length of the sum of the line directions (appendix). Otherwise m is turned toward the mean direction of the lines it faces, from 8 starts per dot: the dot's own normal, which keeps every New score at or below 1, straight down, the 4 best of 256 directions screened on a coarse sample, and two 120° turns of the best one. A last round restarts from small turns around the best direction found (1° to 8° and every 30° about the vertical, 2° and 5° in elevation), to hop between the shallow ripples a finite sunflower leaves in W. For N ≤ 1,024 every line is also a start, as in the original search; at larger N the search without them already matches the original's every-line result.</p>
</section>"""


def explorer(tpl, d):
    i, j = between(tpl, '<section id="explorer">', "</section>")
    sec = tpl[i:j]
    who = "the plate, the bowl and the cup" if d["has_cup"] else "the plate and the bowl"
    sec = sub1(sec, "The dot is unblocked and computed live. The plate and the bowl are read from the grid:",
               f"The unblocked dot, {who} are all read from one grid computed with N = {d['Nf']} lines per set:")
    sec = sub1(sec, "The five dot-score rules", "The seven dot-score rules")
    extra = (" Four rules appear only here: D divides the same numerator by N instead of by a weight; D+G and D+J "
             "are D over the G or the J lines (G: the fixed spray points within θ of vertical below the dot); New+G "
             "is New over the G lines.")
    if d["has_cup"]:
        sec = sub1(sec, "at 180° the plate lies face up and the bowl mouth up.",
                   "at 180° the plate lies face up, the bowl and the cup mouth up.")
        sec = sub1(sec, '<button type="button" data-entity="bowl" aria-pressed="false">bowl</button>',
                   '<button type="button" data-entity="bowl" aria-pressed="false">bowl</button>'
                   '<button type="button" data-entity="cup" aria-pressed="false">cup</button>')
    if d["has_obj"]:
        extra += (" Level switches a dish between the dot level, the mean of its 500 dot scores, and the object level, "
                  "the sum of its 500 numerators over the sum of its 500 denominators. The object level shows "
                  "Original, New and New+J only (the D family divides every dot by the same N, so it is the same at "
                  "both levels), and a single dot has none.")
        sec = sub1(sec, '        </div>\n      </div>\n      <div class="sliders">',
                   '        </div>\n        <span class="ctl-label">Level</span>\n'
                   '        <div class="seg" id="seg-level" role="group" aria-label="Dot level or object level">\n'
                   '          <button type="button" data-level="dot" aria-pressed="true">dot</button>'
                   '<button type="button" data-level="object" aria-pressed="false">object</button>\n'
                   '        </div>\n      </div>\n      <div class="sliders">')
    sec = sub1(sec, "θ moves only the four cone rules.</p>", "θ moves only the four cone rules." + esc(extra) + "</p>")
    if not d["has_neg"]:
        sec = sub1(sec, '<input id="in-distance" type="range" min="-35"', '<input id="in-distance" type="range" min="0"')
        sec = sub1(sec, "the dish grid covers 0 to 35 cm only", "the grid covers 0 to 35 cm only")
    return sec


def perdot(d):
    out, dishes = [], list(d["pd"])
    for n, (dish, pd) in enumerate(d["pd"].items()):
        m = pd["means"]
        ti = {int(t): k for k, t in enumerate(pd["tilts"])}

        def at(t):
            return ", ".join(f"{NAMES[r]} {m[ti[t], RULES.index(r)]:.3f}" for r in RULES3) if t in ti else "–"
        nf = n_(int(pd["N"])) if "N" in pd else d["Nf"]
        h_cm = int(pd["h_cm"]) if "h_cm" in pd else 14
        th = float(pd["theta_deg"]) if "theta_deg" in pd else 30.0
        lead = (f"The {dish} alone, its food-contact centre {h_cm} cm over the disc centre, θ {th:g}°, {nf} lines per "
                "dot, seen along world Y. One 0-to-1 scale; hollow rings mark dots with no weight (Σ c = 0, or no line "
                "in their set)." if n == 0 else f"The {dish}, same pose and rules.")
        low = [NAMES[r] for r in RULES3[1:] if 180 in ti and m[ti[180], RULES.index(r)] < 0.01]
        cap = (f"{lead} Means facing the arm (tilt 0): {at(0)}. On edge (90°): {at(90)}. Food side away (180°): "
               f"{at(180)}" + (f"; {' and '.join(low)} below 0.01." if low else "."))
        out.append(f"""  <figure>
    <div class="media"><img src="media/dots_{dish}.png" alt="The {dish}, 500 dots, coloured by score under Original, New and New+J at tilt 0, 45, 90, 135 and 180 degrees" loading="lazy"></div>
    <figcaption>{esc(cap)}</figcaption>
  </figure>""")
    who = dishes[0] if len(dishes) == 1 else ", ".join(f"the {x}" for x in dishes[:-1]) + f" and the {dishes[-1]}"
    return f"""<section>
  <p class="eyebrow">Per dot</p>
  <h2>{who[0].upper() + who[1:]}, dot by dot</h2>
""" + "\n".join(out) + "\n</section>"


OBJ_ROWS = (("plate on edge (tilt 90°)", "plate", 90), ("plate facing the arm (tilt 0°)", "plate", 0),
            ("bowl facing the arm (tilt 0°)", "bowl", 0), ("cup facing the arm (tilt 0°)", "cup", 0),
            ("cup on edge (tilt 90°)", "cup", 90))


def objlevel_section(d):
    """Dot level against object level (ratio of sums over a dish's 500 dots) for Original, New and New+J."""
    head = """<section id="object-level">
  <p class="eyebrow">Object level</p>
  <h2>One score per dish: a ratio of sums</h2>
"""
    if not d["has_obj"]:
        return head + ('  <p class="note">The object-level grid (N = 262,144, plate, bowl and cup, about 5.5 h) is being '
                       "computed; this section and the explorer toggle fill in the next version.</p>\n</section>")
    g = d["grid"]
    sc = float(g.get("scale", 1e6))
    dishes = [k for k in ("plate", "bowl", "cup") if k in g and f"{k}_obj" in g]

    def v3(x):
        return "–" if x is None or x < 0 else f"{x:.3f}"

    def sgn(v):
        s = f"{v:+.3f}"
        return "0.000" if s in ("+0.000", "-0.000") else s.replace("-", "−")
    rows = []
    for lab, dish, tilt in OBJ_ROWS:
        if dish in dishes:
            a, b = F.grid_at(g, dish, tilt, 14, 0, 30), F.grid_at(g, f"{dish}_obj", tilt, 14, 0, 30)
            rows.append(f"<tr><td>{esc(lab)}</td>"
                        + "".join(f"<td>{v3(a.get(r))}</td><td>{v3(b.get(r))}</td>" for r in RULES3) + "</tr>")
    th = "".join(f"<th>{NAMES[r]}, dot</th><th>{NAMES[r]}, object</th>" for r in RULES3)
    diffs, ncell, nbig, top = [], 0, {}, None
    nT, nH, nD, Q = (len(g[k]) for k in ("tilts", "heights", "distances", "thetas"))
    for r in RULES3:
        x = np.concatenate([np.asarray(g[k][r], float) for k in dishes])
        y = np.concatenate([np.asarray(g[f"{k}_obj"][r], float) for k in dishes])
        ok = (x >= 0) & (y >= 0)
        dd = (y[ok] - x[ok]) / sc
        ncell = max(ncell, int(ok.sum()))
        nbig[r] = int((dd > 0.1).sum())                  # entries where the object level exceeds the dot level by > 0.1
        if dd.size:
            diffs.append(f"{NAMES[r]} median {sgn(float(np.median(dd)))}, from {sgn(float(dd.min()))} to "
                         f"{sgn(float(dd.max()))}")
            if r == "NJ":                                # back to (dish, tilt, height, distance, theta), grid layout
                i = int(np.flatnonzero(ok)[np.argmax(dd)])
                k, rem = divmod(i, nT * nH * nD * Q)
                rem, qi = divmod(rem, Q)
                rem, di = divmod(rem, nD)
                ti, hi = divmod(rem, nH)
                top = (float(dd.max()), dishes[k], g["tilts"][ti], g["heights"][hi], g["distances"][di],
                       g["thetas"][qi], x[i] / sc, y[i] / sc)
    who = ", ".join(dishes[:-1]) + f" and {dishes[-1]}" if len(dishes) > 1 else dishes[0]
    p1 = ("The dot level of a dish is the mean of its 500 dot scores: every dot counts the same, however much spray "
          "its spot could receive. The object level adds up the 500 numerators and divides by the sum of the 500 "
          "denominators, so each dot counts in proportion to its denominator: under New a dot at a spot the arm reaches "
          "well weighs more than a dot the arm can hardly reach, and a dot whose lines are all blocked still adds its "
          "whole denominator.")
    p2 = ("The D family divides every dot by the same N, so its ratio of sums is its mean: it is unchanged by "
          "construction, and only Original, New and New+J are shown. Every dot's best-facing weight enters the New "
          "sums, blocked dots included. "
          + (("Under New+J a dot has a weight only when at least one of its lines reaches the arm disc; a dot with no "
              "such line adds nothing to either sum, where the dot level counts it as a zero. Past the disc edge most "
              "dots are of that kind, so the object level rates only the dots that still see the disc: it exceeds the "
              f"dot level by more than 0.1 in {n_(nbig['NJ'])} of the cell–angle entries (Original {n_(nbig['O'])}, "
              f"New {n_(nbig['N'])}), at most by {sgn(top[0])} at the {top[1]} at tilt {top[2]:g}°, {top[3]:g} cm up, "
              f"{top[4]:g} cm out, jet angle {top[5]:g}° ({top[6]:.3f} at dot level, {top[7]:.3f} at object level). ")
             if top else "")
          + (f"Object minus dot level over the {n_(ncell)} non-skipped grid cells of the {who} (every pose and jet "
             f"angle): {'; '.join(diffs)}." if diffs else ""))
    return head + f"""  <p class="prose">{esc(p1)}</p>
  <div class="formula">E_dot  = (1/500) Σ_j ( Σ_k c_jk v_jk / denom_j )     <i>the dot level: the mean of the dot scores</i>
E_obj  = Σ_j Σ_k c_jk v_jk / Σ_j denom_j          <i>the object level; j runs over the dish's 500 dots</i>

denom_j  Original   W_j(n_j), the dot's own weight
         New        max_m W_j(m), the best-facing weight at the dot's spot
         New+J      max_m W_j(m) over the dot's J lines (the numerator too runs over them)</div>
  <p class="prose">{esc(p2)}</p>
  <div class="scroll-x">
    <table class="cases">
      <thead><tr><th>14 cm up, over the disc centre, θ 30°, N = {d['Nf']}</th>{th}</tr></thead>
      <tbody>
        {chr(10).join('        ' + r for r in rows).strip()}
      </tbody>
    </table>
  </div>
  <figure>
    <div class="media"><img src="media/objlevel_slices.png" alt="Object level and dot level under Original, New and New+J for each dish along tilt, height and distance, at θ 30 degrees" loading="lazy"></div>
    <figcaption><b>Object level against dot level.</b> Solid: the object level; dashed: the dot level; Original grey, New orange-red, New+J blue. Columns: tilt (14 cm up, over the disc centre), height (facing the arm, over the centre) and distance (facing the arm, 14 cm up), all at θ 30°. Gaps: poses skipped because the dish would dip below the disc plane; dotted: the disc edge.</figcaption>
  </figure>
</section>"""


def loads_section(d):
    s, summ, farm, recs = d, d["summ"], d["farm"], d["recs"]
    n = summ["n"]
    h = farm["harvest"]
    counts = h.get("counts") or h.get("outcomes") or {}
    if not recs:
        return (f"""<section id="random-loads">
  <p class="eyebrow">Goal loads</p>
  <h2>Goal loads, completely random and settled in Isaac</h2>
  <p class="prose">{esc(f"No accepted load was scored by the deadline: the farm finished {h.get('n_accepted', 0)} "
                        f"accepted of {h.get('attempted', 0)} attempted loads (outcomes {counts}).")}</p>
</section>""")
    n_acc = int(h.get("n_accepted") or counts.get("accepted") or len(farm["accepted"]) or n)
    nl = n_(recs[0]["N"]) if recs[0].get("N") else d["Nf"]                 # the loads' own N
    attempted = int(h.get("attempted") or (sum(counts.values()) if counts else n_acc))
    rate = n_acc / attempted if attempted else float("nan")
    p5 = 5.0 / (n + 1)
    R = summ["rules"]
    params = recs[0].get("params", {}) if recs else {}
    p_lower = params.get("p_lower", 0.636)
    tries = params.get("tries_per_dish", 8)
    walls = [a.get("wall_s") for a in farm["accepted"] if isinstance(a.get("wall_s"), (int, float))]
    wall = f" Median wall time per accepted load: {np.median(walls) / 60:.1f} min." if walls else ""
    fails = ", ".join(f"{k.replace('_', ' ')} {v}" for k, v in sorted(counts.items(), key=lambda kv: -kv[1])
                      if k != "accepted" and v)
    sw = {k: sum(1 for x in recs if x.get("switches", {}).get(k)) for k in
          ("precheck_enabled", "contact_undo_enabled", "failfast_contact_enabled")}

    def share(k):
        return "every load" if sw[k] == n else f"{sw[k]} of the {n} loads"
    pre = ""
    if sw["precheck_enabled"] or sw["contact_undo_enabled"] or sw["failfast_contact_enabled"]:
        n_pure = acc_pure = 0
        for f in sorted(Path(d["farm_dir"]).glob("loads/*/result.json")) if d.get("farm_dir") else []:
            try:
                r = json.loads(f.read_text())
            except (OSError, ValueError):
                continue
            if not (r.get("precheck_enabled") or r.get("contact_undo_enabled")):
                n_pure += 1
                acc_pure += r.get("outcome") == "accepted"
        if n_pure:
            pre += (f" The sampler started pure: of the {n_pure} loads built without per-drop checks, {acc_pure} passed "
                    f"({acc_pure / n_pure:.0%}), and nearly every load in which a dish came to rest pressed more than "
                    "1 mm into the rack wires failed the final gate. From 04:17 UTC the gate's own checks were also "
                    "applied drop by drop, the same restriction enforced earlier.")
    if sw["precheck_enabled"] or farm["precheck"]:
        n_contain = sum(1 for x in recs if x.get("switches", {}).get("precheck_enabled")
                        and not str(x.get("load_id", "")).split("_g", 1)[-1].startswith("0_"))
        pre += (f" A Kit-free closure pre-check was switched on for {share('precheck_enabled')}: a drop is also "
                "undone if any visual point leaves the tub's width or the dish hits the "
                "cabinet (or, in the lower rack, the closed upper rack) along the rack's retraction")
        pre += (f"; for {n_contain} of them, built by the workers restarted at 04:53 UTC, also if the dish would end up "
                "outside the tub's interior with the racks closed (the gate's containment box)." if n_contain else ".")
        pre += " This tilts the sampled distribution toward loads that close."
    if sw["contact_undo_enabled"]:
        pre += (f" A contact undo was on for {share('contact_undo_enabled')}: a drop whose rest window fails the "
                "gate's contact check (a dish pressed more than 1 mm into the racks or another dish) is undone like any "
                "other failed drop.")
    if sw["failfast_contact_enabled"]:
        pre += (f" A fail-fast rule was on for {share('failfast_contact_enabled')}: a build stops at the first rest "
                "window that fails the gate's contact check, since the final gate would most likely reject it.")
    scored = "" if n == n_acc else f" {n} of the {n_acc} accepted loads were scored in time and are ranked here."
    method = (
        "Each load is built in one Isaac run by dropping the 24 HOTEC dishes (8 plates, 8 bowls, 8 cups) one at a "
        f"time, in a random order. For each drop the rack is drawn in proportion to its box volume (lower {p_lower:.3f}), "
        "the orientation is uniform on SO(3), and the position is uniform over the places that keep the whole visual "
        "mesh inside the rack box. A release that comes within 1 mm of the appliance or a settled dish (FCL) is redrawn. "
        "The dish is released at rest and falls until a 1 s window passes the gate's motion thresholds. If it ends "
        "unsupported, leaves an earlier dish moving or unsupported, or unseats the basket, the drop is undone and "
        f"redrawn (at most {tries} tries, then the load is abandoned). A dish belongs to the rack it rests on; the basket "
        "counts as the lower rack." + pre + " When all 24 have landed, the gate's own checks run in the same process: "
        "the loaded hold (peak penetration under 5 mm, median under 1 mm, support, basket, rack endpoints), then both "
        "racks retract, upper first, and every dish must stay inside the tub, with one lower-first retry. Pooling is "
        "scored but does not reject a load. No score feeds back into the sampling, so the accepted loads are "
        "independent draws: the best of them carries no local-optimum bias, and one more load would enter the current "
        f"top five with chance 5/(n+1). Scores: Original, New and New+J at θ 30° and N = {nl}, per-rack arms (lower arm for "
        "the lower rack and basket, middle arm for the upper rack), every dish and both racks and the basket as "
        "blockers, at the settled rack-local poses with the racks in." + scored + wall)
    tiles = f"""  <div class="tiles two">
    <div class="tile"><p class="t-label">Accepted loads</p><p class="t-value">{n_(n_acc)} of {n_(attempted)}</p><p class="t-sub">built and gated{('; the others: ' + esc(fails)) if fails else ''}</p></div>
    <div class="tile"><p class="t-label">Acceptance rate</p><p class="t-value">{100 * rate:.0f}%</p><p class="t-sub">gate-accepted share of the loads attempted</p></div>
    <div class="tile"><p class="t-label">5/(n+1)</p><p class="t-value">{100 * p5:.1f}%</p><p class="t-sub">the chance that one more random load would enter the top five (n = {n})</p></div>
    <div class="tile"><p class="t-label">Best load, New / New+J</p><p class="t-value">{R['N']['max']:.3f} / {R['NJ']['max']:.3f}</p><p class="t-sub">{esc(R['N']['best'])}{' wins both' if R['N']['best'] == R['NJ']['best'] else ' / ' + esc(R['NJ']['best'])}</p></div>
  </div>"""
    head = "".join(f"<th>{NAMES[r]}</th>" for r in RULES3)
    rows = [f"<tr><td>{lab}</td>" + "".join(f"<td>{f3(R[r][k])}</td>" for r in RULES3) + "</tr>"
            for lab, k in (("min", "min"), ("median", "median"), ("max", "max"))]
    for rb in RULES3:
        S = summ["by_id"][R[rb]["best"]]["S"]
        rows.append(f"<tr><td>Best by {NAMES[rb]} ({esc(R[rb]['best'])})</td>"
                    + "".join(f"<td>{f3(S[r])}</td>" for r in RULES3) + "</tr>")
    table = f"""  <div class="scroll-x">
    <table class="cases">
      <thead><tr><th>S over {n} loads</th>{head}</tr></thead>
      <tbody>
        {chr(10).join('        ' + r for r in rows).strip()}
      </tbody>
    </table>
  </div>"""
    order = {x["load_id"]: x["order_index"] for x in recs}
    rec_cap = (f"<b>Record curves.</b> Best-so-far (solid) and worst-so-far (dashed) S as the accepted loads finished, "
               f"one panel per rule (Original, New, New+J); the last panel is 5/(n+1). The best load under New arrived as load "
               f"{order[R['N']['best']]} of {n}, under New+J as load {order[R['NJ']['best']]}.")
    figs = [f"""  <figure>
    <div class="media"><img src="media/rand_record.png" alt="Best-so-far and worst-so-far S against the number of accepted random loads, for each rule, and 5/(n+1)" loading="lazy"></div>
    <figcaption>{rec_cap}</figcaption>
  </figure>"""]
    partner = {"O": "N", "N": "NJ", "NJ": "N"}
    for r in RULES3:
        for top in (True, False):
            key = "top5" if top else "bottom5"
            ids = R[r][key]
            other = set(R[partner[r]][key])
            k = sum(1 for i in ids if i in other)
            up = []
            for i in ids:
                objs = summ["by_id"][i].get("objects", [])
                up.append(sum(1 for o in objs if o["kind"] == "bowl" and o["rack"] == "UpperRack"))
            word = "highest" if top else "lowest"
            ext = R[r]["max"] if top else R[r]["min"]
            pools = sum(1 for i in ids if summ["by_id"][i]["feasible"] is False)
            cap = (f"<b>Five {word} under {NAMES[r]}</b>{' (θ 30°)' if r in ('DG', 'DJ', 'NG', 'NJ') else ''}. "
                   f"{'Best' if top else 'Worst'} {esc(ids[0])} at {ext:.3f}. {k} of the five are also among the five "
                   f"{word} under {NAMES[partner[r]]}. Bowls in the upper rack: {', '.join(map(str, up))}."
                   + (f" {pools} of the five pool water." if pools else ""))
            figs.append(f"""  <figure>
    <div class="media"><img src="media/rand_{key}_{F.SLUG[r]}.png" alt="The five {word}-scoring random Isaac-settled 24-dish loads under {NAMES[r]}: top views of both racks and a side view" loading="lazy"></div>
    <figcaption>{cap}</figcaption>
  </figure>""")
    sp = summ["spearman"]
    spread_cap = (f"<b>How much the rules agree.</b> Histograms of S under New and New+J; S under Original against "
                  f"New and under New against New+J, coloured by New+J. Spearman rank correlation of S over {n} loads: "
                  f"Original vs New {sp.get('O-N', float('nan')):.2f}, New vs New+J {sp.get('N-NJ', float('nan')):.2f}, "
                  f"Original vs New+J {sp.get('O-NJ', float('nan')):.2f}." if n >= 3 else
                  f"<b>The spread of S</b> over {n} loads; too few loads for rank correlations.")
    if n < 10:
        spread_cap += f" With {n} loads the top five and the bottom five overlap."
    figs.append(f"""  <figure>
    <div class="media"><img src="media/rand_spread.png" alt="Histograms of S under New and New+J over the random loads, and scatters of Original against New and of New against New+J coloured by New+J" loading="lazy"></div>
    <figcaption>{spread_cap}</figcaption>
  </figure>""")
    cav = (f"Caveats: {summ['pooling_loads']} of the {n} loads have at least one dish that pools water; they are kept "
           "and flagged. The loads are scored with the racks in at their settled rack-local poses; the arms and the "
           "exposure model are assumptions, not measurements. PhysX runs are not bit-reproducible, so a load is "
           "reproduced from its stored poses, not from its seed. The gate filters the release distribution, so "
           "\"random\" means independent draws of the loads that pass the gate, not uniform packings.")
    return f"""<section id="random-loads">
  <p class="eyebrow">Goal loads</p>
  <h2>Goal loads, completely random and settled in Isaac</h2>
  <p class="prose">{esc(method)}</p>
{tiles}
{table}
{chr(10).join(figs)}
  <p class="note">{esc(cav)}</p>
</section>"""


def isaac_section(d):
    """The 5 best and 5 worst loads under New+J for both samplers, as ray-traced stills of the Isaac scene."""
    picks_dir, rdir = F.resolve(d["picks_dir"]), F.resolve(d["restricted_dir"])
    sets = {k: json.loads((picks_dir / f"{k}_picks.json").read_text()) for k in ("rand", "restr")
            if (picks_dir / f"{k}_picks.json").is_file()}
    if not sets:
        return ""
    tries = [json.loads(line) for p in sorted(rdir.glob("tries_w*.jsonl")) for line in p.read_text().splitlines()]
    n_try, n_fit = len(tries), sum(t["complete"] for t in tries)
    workers = len({t["worker"] for t in tries})
    lo = [json.loads(p.read_text()) for p in sorted((rdir / "scores_N0016384").glob("*.json"))]
    hi = {p.stem: json.loads(p.read_text()) for p in sorted((rdir / f"scores_N{d['N']:07d}").glob("*.json"))}
    rand_s = [x["S"]["NJ"] for x in d["recs"]]
    restr_s = [hi[x["load_id"]]["S_NJ"] if x["load_id"] in hi else x["S_NJ"] for x in lo]
    gap16 = max([abs(hi[x["load_id"]]["S_NJ"] - x["S_NJ"]) for x in lo if x["load_id"] in hi] or [0.])

    def stats(v):
        return f"{min(v):.3f} / {float(np.median(v)):.3f} / {max(v):.3f}" if v else "–"

    def upper_bowls(entries):
        return sum(1 for e in entries if e["kind"].endswith("bowl") and e["rack"] == "UpperRack")

    dsum = {x["dep"]["load_id"]: F.dep_summary(x["dep"], x["scores"]) for x in d["deps"]}   # graphs by load id

    def figs(key, side):
        out = []
        for p in [q for q in sets.get(key, []) if q["side"] == side]:
            if key == "rand":
                r = json.loads((F.resolve(d["loads_dir"]) / f"{p['load_id']}.json").read_text())
                ents = r.get("entries", [])
                extra = f"Original {r['S']['O']:.3f}, New {r['S']['N']:.3f}. "
                pools = len(r.get("pooling", []))
            else:
                r = hi.get(p["load_id"], {})
                ents, extra = r.get("entries", []), ""
                pools = len(r.get("pooling", []))
            word = "Best" if side == "top" else "Worst"
            cap = (f"<b>{word} {p['rank']}</b>: {esc(p['load_id'])}, S under New+J {p['S_NJ']:.3f}. {extra}"
                   f"Bowls in the upper rack: {upper_bowls(ents) if ents else '–'}. "
                   + (f"{pools} dish{'es' if pools != 1 else ''} pool water." if pools else "No dish pools water."))
            graph = ""
            sm = dsum.get(p["load_id"]) if key == "restr" else None
            if sm:                            # the load's dependency graph, the same figure as the dependency section
                graph = (f'\n    <div class="media"><img src="media/depgraph_{esc(p["load_id"])}.png" alt="Dependency '
                         f'graph of load {esc(p["load_id"])} from the ordered-pair test in Isaac, by wave, and top views '
                         f'of both racks with the same edges" loading="lazy"></div>')
                cap += (f" Below: its dependency graph from the ordered-pair test, {esc(sm['text'])}; the test, the "
                        f'edge list and the build are in <a href="#dependency-graphs">Dependency graphs</a>.')
            out.append(f"""  <figure>
    <div class="media"><img src="media/isaac_{p['tag']}.jpg" alt="{word} {p['rank']} under New+J ({'completely random' if key == 'rand' else 'restricted'} sampler): the load in the Isaac scene, both racks and the lower rack alone" loading="lazy"></div>{graph}
    <figcaption>{cap}</figcaption>
  </figure>""")
        return "\n".join(out)

    method = (f"Two samplers, one rule. Completely random: the {len(d['recs'])} loads above, each built by random drops, "
              "settled and gated in Isaac and scored at its settled poses. Restricted: the sampler of the 8 October page. "
              "All 24 dishes go in one shuffled order with the kinds mixed, and each takes a uniformly random free pose from "
              "its kind's list of physically checked rack poses (the top-5 run's slots, gaps and leans): no appliance or "
              "rack-retraction collision, dishes may touch by up to 0.8 mm, no nested bowls. A try where any dish finds no "
              f"free pose is thrown away. {workers} processes sampled for about 10 minutes, and {n_fit} of {n_try:,} tries fit "
              f"all 24 ({100 * n_fit / max(1, n_try):.0f}%). The 8 October run fit 27 of 3,745 under the same written rules; "
              "this machine holds none of the gate-penetration bans that run loaded from earlier benchmark gates, which "
              "likely explains most of the difference. Restricted loads are scored at their packed poses; they are not "
              f"settled in Isaac. Both samplers are ranked under New+J only (θ 30°, per-rack arms, every dish, both racks "
              f"and the basket as blockers). The restricted loads were ranked at N = 16,384, and the 12 highest and 12 "
              f"lowest rescored at N = {d['Nf']} (the two agree to within {gap16:.4f}); the ten shown, and the minimum "
              f"and maximum in the table, are at N = {d['Nf']}.")
    render = ("Isaac Sim's RTX renderer does not start on this machine: Kit crashes as it enables its Hydra renderer "
              "(NVIDIA driver 595.71 is newer than Isaac Sim 4.5 supports, and neither may change here). So each image is "
              "ray-traced on the GPU with Warp from the meshes the Isaac scene loads (the HOTEC visual meshes, both racks "
              "and the basket), at the scored poses with the racks in and the appliance shell hidden. Left: both racks. "
              "Right: the lower rack, with the upper rack and its dishes hidden. Plates are blue, bowls orange and cups "
              "green, one shade per dish.")
    table = f"""  <div class="scroll-x">
    <table class="cases">
      <thead><tr><th>S under New+J</th><th>loads</th><th>min / median / max</th></tr></thead>
      <tbody>
        <tr><td>Completely random (settled and gated in Isaac)</td><td>{len(rand_s)}</td><td>{stats(rand_s)}</td></tr>
        <tr><td>Restricted (packed poses, 8 October sampler)</td><td>{len(restr_s)}</td><td>{stats(restr_s)}</td></tr>
      </tbody>
    </table>
  </div>"""
    graphs_note = ('  <p class="prose">Each still is followed by the load&#8217;s dependency graph from the ordered-pair '
                   'test in Isaac: an arrow from A to B means A has to be placed before B, a two-way pair means that '
                   'no one-at-a-time order builds the load, and a load the joint gate rejected has no edges. The test, '
                   'the full table and the graphs of the other tested loads are in '
                   '<a href="#dependency-graphs">Dependency graphs</a>.</p>\n'
                   if any(q["load_id"] in dsum for q in sets.get("restr", []) if q["side"] == "top") else "")
    return f"""<section id="isaac-scenes">
  <p class="eyebrow">Loads in the Isaac scene</p>
  <h2>The five best and five worst loads under New+J, two samplers</h2>
  <p class="prose">{esc(method)}</p>
  <p class="prose">{esc(render)}</p>
{table}
  <h3>Completely random: the five best</h3>
{figs("rand", "top")}
  <h3>Completely random: the five worst</h3>
{figs("rand", "bottom")}
  <h3>Restricted: the five best</h3>
{graphs_note}{figs("restr", "top")}
  <h3>Restricted: the five worst</h3>
{figs("restr", "bottom")}
</section>"""


def s_chain(S, settled="settled"):
    """' S packed → settled → built: Original a → b → c; ...' over the sides that were scored ('' when none);
    settled: that side's label (re-settled for a re-settled goal)."""
    sides = [k for k in ("packed", "settled", "built") if S.get(k)]
    vals = "; ".join(f"{NAMES[r]} " + " → ".join(f"{S[k][r]:.3f}" for k in sides)
                     for r in RULES3 if all(r in S[k] for k in sides))
    lab = " → ".join(settled if k == "settled" else k for k in sides)
    return f" S {lab}: {vals}." if vals else ""


def mins(s):
    return f"{s / 60:.0f} min" if s >= 90 else f"{s:.0f} s"


def gate_words(dep):
    """(sentence, detail) from the record's gate field (never the gate directory). sentence: the outcome and the
    settle peak (the retraction peak too above 0.05 mm) against the limit. A load the gate rejected over the limit gets
    detail instead: its peak against the limit."""
    g = dep.get("gate") if isinstance(dep.get("gate"), dict) else {}
    st, cy, lim = (float(g[k]) * 1e3 if isinstance(g.get(k), (int, float)) else None for k in
                   ("maximum_settle_penetration_m", "maximum_cycle_penetration_m", "peak_penetration_limit_m"))
    pk = max((v for v in (st, cy) if v is not None), default=None)
    if dep.get("status") == "gate" and pk is not None and (lim is None or pk > lim):
        return "", f" ({pk:.2f} mm" + (f" against the {lim:.0f} mm limit)" if lim is not None else ")")
    parts = ([f"while settling {st:.1f} mm"] if st is not None else []) + (
        [f"during the rack retraction {cy:.1f} mm"] if cy is not None and cy > 0.05 else [])
    out = str(g.get("outcome") or "").replace("_", " ")
    if not (out or parts):
        return "", ""
    deep = ("deepest momentary contact " + " and ".join(parts) + (f" (limit {lim:.0f} mm)" if lim is not None else "")
            if parts else "")
    return " Joint gate: " + "; ".join(x for x in (out, deep) if x) + ".", ""


def candidate_mm(deps, default=20.0):
    """The candidate-pair distance [mm] from the records' thresholds (a key naming 'candidate'; *_m keys in metres)."""
    for x in deps:
        for k, v in (x["dep"].get("thresholds") or {}).items():
            if "candidate" in str(k) and isinstance(v, (int, float)) and not isinstance(v, bool):
                return float(v) * (1e3 if str(k).endswith("_m") else 1.0)
    return default


def rerun_agreement(dep, rerun):
    """(edges in both runs, only in the first, only in the second, two-way pairs equal, every drop repeated exactly)
    of two pairwise runs of one load; edges as (a, b), reasons ignored."""
    e1, t1, _ = F.dep_edges(dep)
    e2, t2, _ = F.dep_edges(rerun)
    s1, s2 = set(e1), set(e2)
    return s1 & s2, s1 - s2, s2 - s1, set(t1) == set(t2), bool(dep.get("tests")) and dep.get("tests") == rerun.get("tests")


def perturbation(deps):
    """(largest lateral offset [mm], largest yaw [deg]) of the releases of perturbed reruns, from their record-level
    perturbation field; None when no rerun was perturbed (field absent or zero)."""
    lat = yaw = 0.0
    for dep in deps:
        p = dep.get("perturbation") if isinstance(dep.get("perturbation"), dict) else {}
        lat = max(lat, abs(float(p.get("lateral_mm") or 0.0)))
        yaw = max(yaw, abs(float(p.get("yaw_deg") or 0.0)))
    return (lat, yaw) if lat or yaw else None


def _deg(v):
    return "?°" if v is None else f"{v:.0f}°" if v >= 10 else f"{v:.1f}°"


def resettled_block(d, rank):
    """The sub-section closing the dependency section: goals re-settled by a sequential build (--resettled-dir)."""
    rs = sorted(d.get("resettled") or [], key=lambda x: (rank.get(x["dep"]["load_id"], len(rank)), x["dep"]["load_id"]))
    if not rs:
        return ""
    failed = [x["dep"]["load_id"] for x in rs if x["dep"].get("status") == "resettle_failed"]
    prose = ("A jointly settled goal may hold a pair that only settles together: dropped one at a time, neither order "
             "brings both dishes back to where the joint settle left them. Building the goal one dish at a time with "
             "the own-load landing rule (each dish commanded at its jointly settled pose raised 3 mm and kept where it "
             "settles, within 8 cm of the command, with under 2 mm of contact and nothing already placed moved more "
             "than 10 mm or 20°) is meant to give a goal the move can reach, on which the ordered-pair test is then "
             "rerun" + ("; the table gives how far the goal moved." if len(failed) < len(rs) else "."))
    if failed:
        who = ("Both loads" if len(failed) == len(rs) == 2 else f"All {len(rs)} loads" if len(failed) == len(rs) > 2
               else "The load" if len(rs) == 1 else ", ".join(failed))
        prose += (f" {who} failed: the goal could not be re-settled by a sequential build under the own-load rule "
                  f"either, so {'their' if len(failed) > 1 else 'its'} two-way pairs must be placed together.")
    rows, figs, notes = [], [], []
    for x in rs:
        dep, lid = x["dep"], x["dep"]["load_id"]
        sm = F.dep_summary(dep, x["scores"])
        L, T = sm["goal_moved"] or (None, None)
        tag = f"Best {rank[lid] + 1}: " if lid in rank else ""
        none = sm["unsettled"] or sm["rejected"]
        nf, _, how = F.resettle_failure(dep)
        status = (f"resettle_failed: {how}" if sm["unsettled"] else sm["status"] if sm["ok"] or sm["cycle"] or none
                  else f"{sm['status']}: {sm['failed'] or '?'}")
        cells = [lid, sm["resettle_builds"], "–" if L is None and T is None else
                 f"{'–' if L is None else f'{L:.1f}'} / {'–' if T is None else f'{T:.1f}'}",
                 "–" if none else sm["n_pairs"],
                 "–" if none else "{} ({} / {} / {})".format(sm["n_edges"], *(sm["reasons"][r] for r in F.REASONS)),
                 "–" if none else sm["n_cycles"], "–" if none or sm["cycle"] or sm["n_waves"] is None else sm["n_waves"],
                 "–" if none or sm["cycle"] else sm["builds"], "–" if none or sm["cycle"] else len(sm["extra"]), status]
        shown = ("packed",) if sm["unsettled"] else ("packed", "settled", "built")   # not re-settled: its settled
        cells += [" / ".join(f"{sm['S'][k][r]:.3f}" if k in shown and (sm["S"][k] or {}).get(r) is not None    # scores
                             else "–" for k in ("packed", "settled", "built")) for r in RULES3]     # are the joint settle's
        rows.append("<tr>" + "".join(f"<td>{esc(str(c))}</td>" for c in cells) + "</tr>")
        if none:                              # no re-settled goal: a row and a line, no figure
            notes.append(f"{tag}{lid}: the goal could not be re-settled; {F._plural(nf, 'resettle build')} "
                         f"{'was' if nf == 1 else 'were'} tried, and the last stopped when {how}." if sm["unsettled"]
                         else f"{tag}{lid}: {sm['text']}.")
            continue
        ex = sm["extra"]
        exl = (" (" + ", ".join(f"{a} before {b}" for a, b in ex[:6]) + (", …" if len(ex) > 6 else "") + ")"
               if ex else "")
        dr = F.context_drift(dep)
        cap = (f", goal re-settled: {sm['text']}. Goal moved at most {F._num(L, 'mm')} and {_deg(T)}; "
               f"{F._plural(sm['resettle_builds'], 'resettle build')}. " + sm["counts"][:1].upper() + sm["counts"][1:]
               + exl + "." + (f" Context drift over {F.DRIFT_MM:g} mm in {dr[0]} of {dr[1]} drops, at most "
                              f"{dr[2]:.1f} mm." if dr else "") + s_chain(sm["S"], settled="re-settled"))
        link = (f' <a href="data/depgraph_{esc(lid)}_resettled.json">edge list (JSON)</a>' if sm["n_edges"] else "")
        figs.append(f"""  <figure>
    <div class="media"><img src="media/depgraph_{esc(lid)}_resettled.png" alt="Dependency graph of load {esc(lid)} on its goal re-settled by a sequential build, by wave, and top views of both racks with the same edges" loading="lazy"></div>
    <figcaption><b>{esc(tag + lid)}</b>{esc(cap)}{link}</figcaption>
  </figure>""")
    head = "".join(f"<th>{h}</th>" for h in (
        "load", "resettle builds", "goal moved: max mm / deg", "pairs tested", "edges (knock / need / block)", "cycles",
        "waves", "builds", "extra constraints", "status", *(f"S {NAMES[r]}: packed / re-settled / built" for r in RULES3)))
    note = "".join(f'  <p class="note">{esc(t)}</p>\n' for t in notes)
    return f"""
  <h3>Re-settled goals for the two-way loads</h3>
  <p class="prose">{esc(prose)}</p>
  <div class="scroll-x">
    <table class="cases">
      <thead><tr>{head}</tr></thead>
      <tbody>
        {chr(10).join('        ' + r for r in rows).strip()}
      </tbody>
    </table>
  </div>
{note}{chr(10).join(figs)}"""


def dependency_section(d):
    """The ordered-pair dependency graph of each of the five best restricted loads (figure, caption, JSON link)."""
    deps = d["deps"]
    if not deps:
        return ""
    rank = {lid: k for k, lid in enumerate(d["dep_picks"])}
    deps = sorted(deps, key=lambda x: (rank.get(x["dep"]["load_id"], len(rank)), x["dep"]["load_id"]))
    Ns = {int(x["scores"]["N"]) for x in deps if x["scores"] and x["scores"].get("N")}
    nf = n_(Ns.pop()) if len(Ns) == 1 else d["Nf"]
    sms = {x["dep"]["load_id"]: F.dep_summary(x["dep"], x["scores"]) for x in deps}
    reruns = d.get("reruns") or {}                       # --rerun-dir: edge agreement of a second run per load
    agree = {x["dep"]["load_id"]: rerun_agreement(x["dep"], reruns[x["dep"]["load_id"]]["dep"]) for x in deps
             if x["dep"]["load_id"] in reruns and not sms[x["dep"]["load_id"]]["rejected"]
             and reruns[x["dep"]["load_id"]]["dep"].get("status") != "gate"}
    tails = "".join(f"; in {x['dep']['load_id']} the context stayed displaced instead: its drift never returned below "
                    f"{t[1]:.1f} mm after test {t[0]}" + (f", even in the {t[2]} drops in which only cups were out of the "
                                                         "load" if t[2] else "")
                    for x in deps for t in [F.drift_tail(x["dep"])] if t)
    test = ("Each load first passes the joint gate of the top-five loads, in the extended racks of the empty baseline: "
            "all 24 dishes are placed at their packed poses at once and settled together, and the load passes when it "
            "comes to rest, both racks then retract with it, no contact gets deeper than 5 mm at any moment of the "
            "settle or the retraction, and every dish ends inside the cabinet. A load the gate rejects gets no graph. "
            "The graph then comes from the goal load alone, the jointly settled poses, and the benchmark's own move: a "
            "dish is teleported 3 mm above its goal pose and Isaac settles it (150 + 60 ticks). It lands when it comes "
            "to rest within the at-goal tolerance of its goal pose (bowls and cups 15 mm lateral, 20 mm in height and "
            "15°; plates 18 mm, 20 mm and 16°) with less than 2 mm of contact depth, and no other dish moves more than "
            "10 mm or 20°. The context is the rest of the load: the other dishes sit at their jointly settled poses, "
            "re-settled before each drop, and a dish counts as moved against those re-settled poses, as in the "
            "sequencer. Each dish is dropped last into the full load (24 drops), and each dish of a candidate pair is "
            "dropped with the other one removed (2 drops per pair). The candidates are the pairs whose collision "
            f"pieces come within {candidate_mm(deps):g} mm of each other at the goal poses (FCL, any rack); every "
            f"other pair is independent by geometry. A context drift above {F.DRIFT_MM:g} mm before a drop usually "
            "means a removed dish was holding a neighbour up: the neighbour falls, the dropped dish then pushes it and "
            "the test records a knock, an edge in the right direction that is really a lean (the neighbour leans on "
            "the dropped dish), and any edge measured in a context displaced like this is weaker evidence" + tails
            + ". The drift of every test is in the JSON (context_drift_mm), and each caption counts the drops above "
            f"{F.DRIFT_MM:g} mm.")
    pert = perturbation(reruns[lid]["dep"] for lid in agree)
    if agree:
        n, both = len(agree), sum(len(a[0]) for a in agree.values())
        first, new = sum(len(a[0]) + len(a[1]) for a in agree.values()), sum(len(a[2]) for a in agree.values())
        same = sum(a[4] for a in agree.values())         # reruns that repeated every drop to the last digit
        if pert:                                         # a perturbed-release rerun: a robustness measure
            test += (f" A second run perturbed every release by up to {pert[0]:g} mm laterally and {pert[1]:g}° in "
                     "yaw, the context and the certifying build left unperturbed, as a robustness measure: over the "
                     f"{F._plural(n, 'load')} run twice, {both} of {first} edges were reproduced, {first - both} lost "
                     f"and {new} new, and the two-way pairs agreed in {sum(a[3] for a in agree.values())} of {n}.")
        elif same == n:
            test += (f" A second run of the same {F._plural(n, 'load')} repeated every drop to the last digit: the Isaac "
                     "settle is deterministic here, so a rerun measures no settle noise, and all "
                     f"{first} edges and the two-way pairs came back unchanged.")
        else:
            test += (f" Each test runs once, so a second run of the same loads measures the settle noise: over the "
                     f"{F._plural(n, 'load')} run twice, {both} of {first} edges came back, "
                     f"{F._plural(new, 'new edge')} appeared, and the two-way pairs agreed in "
                     f"{sum(a[3] for a in agree.values())} of {n}"
                     + (f"; {same} of the reruns repeated every drop to the last digit." if same else "."))
    rule = ("The ordered-pair test reads these drops: A goes before B when only that order lands both dishes, for one "
            "of three reasons. Knock: A dropped last into the full load moves B, so A must go in first (a knock edge "
            "may point at any dish the drop moved, candidate or not). Need: B lands with A in place but not with A "
            "removed. Block: A lands only with B removed, so B in place would stop it. An edge can carry two reasons, "
            "so the knock, need and block counts in the captions and the table can add up to more than the edges. "
            "A two-way pair, each dish having "
            "to go in before the other, is a cycle: no one-at-a-time order builds the load, and the load gets no build. "
            "An acyclic graph is certified by the sequential build: the load is rebuilt one dish at a time in a "
            "topological order of the graph, lowest centre first among the dishes that are free, with the same move "
            "and the same landing test, and a failed build adds the order it teaches and tries again, up to 8 builds. "
            "The number of extra constraints, those the build had to add to the graph, is the graph's score: 0 means "
            "the graph alone orders the load. The waves are the layers of the graph: a dish's wave is the longest path "
            "to it from a dish nothing must precede, so every dish of a wave can go in once the earlier waves are in. "
            f"The jointly settled and the built poses are scored like the packed ones, under the three rules at "
            f"N = {nf}, so packed → settled → built shows what the joint settle, then the build one dish at a time, "
            "does to S.")
    ren = ("The construction follows Ren and Qureshi (IROS 2024, Multi-Stage Monte Carlo Tree Search for Non-Monotone "
           "Object Rearrangement Planning in Narrow Confined Environments): a graph over the goal arrangement, which "
           "must be a DAG, a topological order of it as the placing order, and a cycle meaning that no order exists. "
           "Their edge comes from the arm's placing tunnel, the space the gripper sweeps on its way to a goal pose; "
           "this benchmark has no arm, so the settle test of its own move takes the tunnel's place.")
    legend = ("Left: the graph, one column per wave (in a cyclic load the waves of the graph with its two-way pairs "
              "removed; one row per kind when there is one wave or none), the dishes no edge orders in a column of "
              "their own; plates blue, bowls orange, cups green. Every edge is the same solid dark arrow from the "
              "earlier dish to the later one, its reasons in the JSON edge list; a two-way pair is one double-headed "
              "edge marked two-way, and the failing dish of a failed build is ringed red. Right: top views of the "
              "lower and the upper rack at the built poses (the jointly settled poses, the targets, when there was no "
              "build; the packed poses when the joint gate rejected the load), with the same edges, slightly curved, "
              "between the dish centres; p, b and c mark plates, bowls and cups.")
    if not set(sms) <= set(rank):             # loads beyond the picks: say how many were tested
        nt = sum(not sm["rejected"] for sm in sms.values())
        nc = sum(bool(sm["cycle"]) for sm in sms.values())
        legend += (f" The table and the figures cover {F._plural(len(sms), 'load')}: {nt} passed the joint gate and "
                   f"{'was' if nt == 1 else 'were'} tested, {nc or 'none'} of them with a two-way pair.")
    figs, rows = [], []
    for x in deps:
        dep = x["dep"]
        lid = dep["load_id"]
        sm = sms[lid]
        rej = sm["rejected"]
        tag = f"Best {rank[lid] + 1}: " if lid in rank else ""
        kit = dep.get("kit") or {}
        ks, kd = (kit.get(k) if isinstance(kit.get(k), (int, float)) and not rej else None
                  for k in ("wall_seconds", "drop_wall_seconds"))
        nd = len(dep.get("tests") or [])
        wall = [f"joint gate {mins(sm['gate_s'])}"] if sm["gate_s"] is not None else []
        wall += [f"{nd} drops {mins(kd)}" if nd else f"drops {mins(kd)}"] if kd is not None else []
        wall += [f"Kit session {mins(ks)}"] if ks is not None else []
        wall = f" Isaac time: {', '.join(wall)}." if wall else ""
        pen, detail = gate_words(dep)
        st = s_chain(sm["S"])
        ex = sm["extra"]
        exl = (" (" + ", ".join(f"{a} before {b}" for a, b in ex[:6]) + (", …" if len(ex) > 6 else "") + ")"
               if ex else "")
        dr = F.context_drift(dep)
        drift = (f" Context drift over {F.DRIFT_MM:g} mm in {dr[0]} of {dr[1]} drops, at most {dr[2]:.1f} mm."
                 if dr else "")
        counts = sm["counts"][:1].upper() + sm["counts"][1:] + exl + "." + drift
        if rej:
            cap = f", the joint gate rejected this load: {sm['why'].strip().rstrip('.')}{detail}; {sm['counts']}."
            cap += pen + st + wall
        elif sm["ok"]:
            pool = sm["pooling_built"]
            pl = ("" if pool is None else " No dish pools water at the built poses." if not pool else
                  f" {len(pool)} dish{'es pool' if len(pool) != 1 else ' pools'} water at the built poses "
                  f"({', '.join(map(str, pool))}).")
            cap = f", {sm['text']}. " + counts + st + pl + pen + wall
        elif sm["cycle"]:
            cap = f", {sm['text']}. " + counts + st + pen + wall
        else:
            cap = f", {sm['text']} ({sm['failed'] or '?'} ringed red). " + counts + st + pen + wall
        link = f' <a href="data/depgraph_{esc(lid)}.json">edge list (JSON)</a>' if sm["n_edges"] else ""
        figs.append(f"""  <figure>
    <div class="media"><img src="media/depgraph_{esc(lid)}.png" alt="Dependency graph of load {esc(lid)} from the ordered-pair test in Isaac, by wave, and top views of both racks with the same edges" loading="lazy"></div>
    <figcaption><b>{esc(tag + lid)}</b>{esc(cap)}{link}</figcaption>
  </figure>""")
        nobuild = rej or sm["cycle"]           # no build: its numbers are dashes; a cyclic graph has no waves
        status = sm["status"] if sm["ok"] or nobuild else f"{sm['status']}: {sm['failed'] or '?'}"
        cells = [lid, sm["outcome"] or "–", "–" if rej else sm["n_pairs"],
                 "–" if rej else "{} ({} / {} / {})".format(sm["n_edges"], *(sm["reasons"][r] for r in F.REASONS))]
        if agree:                             # edges reproduced by the second run: "8 of 9, +1 new"
            a = agree.get(lid)
            cells.append("–" if a is None else f"{len(a[0])} of {len(a[0]) + len(a[1])}"
                         + (f", +{len(a[2])} new" if a[2] else "") + ("" if a[3] else ", cycles differ"))
        cells.append("–" if dr is None else f"{dr[0]} of {dr[1]}")
        cells += ["–" if rej else sm["n_cycles"], "–" if nobuild or sm["n_waves"] is None else sm["n_waves"],
                 "–" if nobuild else sm["builds"], "–" if nobuild else len(ex), status,
                 " / ".join(f"{sm['S'][s]['NJ']:.3f}" if (sm["S"][s] or {}).get("NJ") is not None else "–"
                            for s in ("packed", "settled", "built"))]
        rows.append("<tr>" + "".join(f"<td>{esc(str(c))}</td>" for c in cells) + "</tr>")
    missing = [p for p in d["dep_picks"] if p not in {x["dep"]["load_id"] for x in deps}]
    head = "".join(f"<th>{h}</th>" for h in ("load", "gate", "pairs tested", "edges (knock / need / block)",
                                             *(((f"edges under a {pert[0]:g} mm / {pert[1]:g}° release" if pert
                                                 else "edges reproduced"),) if agree else ()),
                                             f"drift > {F.DRIFT_MM:g} mm", "cycles",
                                             "waves", "builds", "extra constraints", "status",
                                             "S under New+J: packed / settled / built"))
    table = f"""  <div class="scroll-x">
    <table class="cases">
      <thead><tr>{head}</tr></thead>
      <tbody>
        {chr(10).join('        ' + r for r in rows).strip()}
      </tbody>
    </table>
  </div>
"""
    note = (f'  <p class="note">{esc("Not tested yet: " + ", ".join(missing) + ".")}</p>\n' if missing else "")
    return f"""<section id="dependency-graphs">
  <p class="eyebrow">Dependency graphs</p>
  <h2>Which dish must go in before which: the ordered-pair test in Isaac</h2>
  <p class="prose">{esc(test)}</p>
  <p class="prose">{esc(rule)}</p>
  <p class="prose">{esc(ren)}</p>
  <p class="prose">{esc(legend)}</p>
{table}{note}{chr(10).join(figs)}{resettled_block(d, rank)}
</section>"""


def appendix(d):
    sw = d["sweep"]
    Ns, rows = sw["N"], sw["rows"]
    show = [t for t in (5.0, 30.0, 60.0, 85.0) if t in sw["thetas"]]
    head = "".join(f"<th>θ {t:g}°: max / mean</th>" for t in show)
    body = []
    for N in Ns:
        cells = "".join(f"<td>{sci(rows.get((N, t), {}).get('gap_max'))} / {sci(rows.get((N, t), {}).get('gap_mean'))}</td>"
                        for t in show)
        body.append(f"<tr><td>{n_(N)}</td>{cells}</tr>")
    big = [n for n in Ns if n >= 1024] or Ns
    sl = [slope(big, [rows.get((n, t), {}).get("gap_max") for n in big]) for t in sw["thetas"]]
    slm = [slope(big, [rows.get((n, t), {}).get("gap_mean") for n in big]) for t in sw["thetas"]]
    sl, slm = [x for x in sl if np.isfinite(x)], [x for x in slm if np.isfinite(x)]
    tail_html = ""
    tail_path = Path(d.get("farm_dir", ".")).parents[2] / "results/dot_rules/grid/tail.json" if d.get("farm_dir") else None
    if tail_path is not None and tail_path.is_file():
        tl = json.loads(tail_path.read_text())
        cells_t, levels_t = {}, sorted({r["N"] for r in tl["rows"]})
        for r in tl["rows"]:
            key = (r["dish"], r["tilt"], r["h_cm"], r["d_cm"], r["theta"])
            cells_t.setdefault(key, {})[r["N"]] = r
        if len(levels_t) > 1:
            th_t = "".join(f"<th>N = {n_(n)}</th>" for n in levels_t)
            tr_t = []
            for (dish, tilt, h, dd, th), byN in sorted(cells_t.items(), key=lambda kv: -kv[1][levels_t[0]]["gap"]
                                                       if levels_t[0] in kv[1] else 0):
                tds = "".join(f"<td>{sci(byN[n]['gap']) if n in byN else '–'}</td>" for n in levels_t)
                last = byN.get(levels_t[-1], {})
                tr_t.append(f"<tr><td>{dish}, tilt {tilt}°, {h} cm up, {dd} cm out, θ {th:g}°</td>{tds}"
                            f"<td>{n_(round(last['nG_mean'])) if 'nG_mean' in last else '–'}</td></tr>")
            tail_html = (
                '  <p class="prose">The slowest cells keep converging past the sweep. The cells with the largest dish-mean '
                f"gap at N = {n_(levels_t[0])}, recomputed alone at larger N (their own jet angle only):</p>\n"
                '  <div class="scroll-x">\n    <table class="cases">\n'
                f"      <thead><tr><th>|New+G − New+J|, dish mean</th>{th_t}<th>G lines per dot at N = {n_(levels_t[-1])}</th></tr></thead>\n"
                "      <tbody>\n" + "\n".join("        " + r for r in tr_t) + "\n      </tbody>\n    </table>\n  </div>\n")
    last = {t: rows.get((Ns[-1], t), {}) for t in sw["thetas"]}
    worst_t = max(last, key=lambda t: last[t].get("gap_max", -1))
    dr0 = np.nanmedian([rows.get((Ns[0], t), {}).get("dl_med", np.nan) for t in sw["thetas"]])
    dr1 = np.nanmedian([rows.get((Ns[-1], t), {}).get("dl_med", np.nan) for t in sw["thetas"]])
    nc = [v.get("n_cells") for v in rows.values() if v.get("n_cells")]
    cells = f": {max(nc):,} dish poses per jet angle" if nc else ""
    rate = ""
    if sl:
        m_ = lambda v: f"{v:.2f}".replace("-", "−")
        rate = (f"From N = {n_(big[0])} to {n_(Ns[-1])} the largest dish-mean gap falls like N<sup>{m_(np.median(sl))}</sup> "
                f"(median slope over the {len(sl)} jet angles, range {m_(min(sl))} to {m_(max(sl))})"
                + (f" and the mean gap like N<sup>{m_(np.median(slm))}</sup>" if slm else "")
                + f". At N = {n_(Ns[-1])} the largest gap over all poses is {sci(last[worst_t].get('gap_max'))} "
                f"(θ {worst_t:g}°). ")
    if np.isfinite(dr0) and np.isfinite(dr1):
        rate += (f"The remark checks out dot by dot: the median relative gap between a dish's D+G and the mean of "
                 f"(a_i/R)²·D+J_i over its dots (a_i = h_i·tan θ) falls from {sci(dr0)} at N = {n_(Ns[0])} "
                 f"to {sci(dr1)}.")
    return f"""<section id="appendix">
  <p class="eyebrow">Appendix</p>
  <h2>Why G and J meet as N grows</h2>
  <p class="prose">Fix one dot: start s at height h &gt; 0 above the arm's disc D (centre o, radius R, in the plane z = 0), unit normal n, and blockers made of triangles (its own dish, or a whole load). A point q of the plane is reached along the unit direction d(q) = (q − s)/|q − s|. The line from s to q lies within θ of straight down exactly when q is inside the dot's θ-circle C: centred under the dot, radius a = h·tan θ. Let A = C ∩ D. Both cone sets are point samples of the same region A:</p>
  <div class="formula">U_k    = √((k + ½)/N) · (cos kφ, sin kφ),  φ = π(3 − √5),  k = 0 … N−1     <i>the unit sunflower</i>
G      = the fixed points q_k = o + R·U_k that lie in A                 <i>N points spread over D</i>
J      = the dot's own points p_k = foot + a·U_k that lie in D          <i>N points spread over C</i>
c(q)   = max(0, n·d(q))       v(q) = 1 if the segment s→q is open, else 0
g_m(q) = max(0, m·d(q))       for a unit vector m
New+X  = Σ_X c·v / max_m Σ_X g_m        <i>X = G or J; 0 if X is empty</i></div>
  <h3>Theorem</h3>
  <p class="prose">If A has positive area and the shadow boundary (the points of A where v jumps) has zero area, then as N → ∞ both New+G and New+J converge to the same limit</p>
  <div class="formula">L = ∫_A c·v dA / max_m ∫_A g_m dA</div>
  <p class="prose">The shadow condition always holds here: every blocker is a finite set of triangles, the shadow of a triangle seen from s is a polygon, so the shadow boundary is a finite union of segments.</p>
  <h3>Proof</h3>
  <ol class="proof">
    <li><b>The sunflower spreads area evenly.</b> Take an annular sector S of the unit disc: r₁ &lt; r ≤ r₂, angle in an arc I of length ℓ. U_k ∈ S needs r₁² &lt; (k + ½)/N ≤ r₂², which picks a run of consecutive k of length M = N(r₂² − r₁²) + O(1), and kφ mod 2π ∈ I. Because φ/2π is irrational, the angles kφ are equidistributed (Weyl), and uniformly over runs that start anywhere: for every integer t ≠ 0, |Σ<sub>k=k₀…k₀+M−1</sub> e<sup>itkφ</sup>| = |sin(tMφ/2)| / |sin(tφ/2)| ≤ 1/|sin(tφ/2)|, a bound that does not depend on k₀ and is finite because tφ/2π is never an integer. So the run holds (ℓ/2π)·M + o(M) points of the arc, and #{{k : U_k ∈ S}}/N → ℓ(r₂² − r₁²)/2π = |S|/π. A set E whose boundary has zero area (Jordan measurable) lies between two finite unions of sectors whose areas differ by less than any ε, so #{{k : U_k ∈ E}}/N → |E|/π. Scaling and translation keep area ratios: the fixed points fill D evenly (a fraction |E|/|D| of them lands in E) and the dot's own points fill C evenly (fraction |E|/|C|).</li>
    <li><b>The weights are Riemann integrable on A.</b> c and g_m are continuous in q (h &gt; 0 keeps q away from s). v jumps only on the shadow boundary, and the boundary of A is two circular arcs; both have zero area. A bounded function that is continuous outside a zero-area set is squeezed between step functions on small cells whose integrals differ by less than any ε, so step 1 turns averages into integrals: (1/N) Σ_G c·v → ∫_A c·v dA / |D| and (1/N) Σ_J c·v → ∫_A c·v dA / |C|, and the same for g_m at every fixed m.</li>
    <li><b>The maxima converge.</b> Each term is 1-Lipschitz in m: |max(0, m·d) − max(0, m′·d)| ≤ |m − m′|. So the normalized sums F_N(m) = (1/N) Σ_X g_m are 1-Lipschitz in m for every N, and so is their limit F(m). Pointwise convergence of a 1-Lipschitz family on the compact sphere is uniform (converge on a finite ε-net, fill the gaps with the Lipschitz bound), and |max F_N − max F| ≤ sup |F_N − F|, so max_m F_N(m) → max_m F(m).</li>
    <li><b>The limit denominator is positive.</b> Straight down, m = (0, 0, −1), gives g_m(q) = h/|q − s| &gt; 0 on all of A, and A has positive area, so max_m ∫_A g_m dA &gt; 0. For large N the finite denominators are positive too, so neither rule falls back to 0.</li>
    <li><b>The factors cancel.</b> Write New+X with the same 1/N on top and bottom (the maximum commutes with a positive factor). For G both limits carry the factor 1/|D|, for J both carry 1/|C|; inside each rule it cancels, and both tend to ∫_A c·v dA / max_m ∫_A g_m dA = L. ∎</li>
  </ol>
  <h3>Remark: the D pair does not meet</h3>
  <p class="prose">D+G = Σ_G c·v / N → ∫_A c·v dA / |D| = (|A|/|D|) × the mean of c·v over A, and D+J = Σ_J c·v / N → (|A|/|C|) × the same mean. Their ratio tends to |C|/|D| = (a/R)² = (h·tan θ / R)²: D+G counts the cone's share of the disc's points, D+J the cone's share of its own points.</p>
  <p class="prose">A dish score is the mean of its 500 dots and a load's S is an area-weighted mean of its dishes, so both are finite averages: New+G and New+J meet for dishes and loads too. Where A is empty (a dot at or below the disc plane, or a θ-circle that misses the disc) both rules are 0 at every N. The computed reference is exact for θ ≤ 45°: all G and J lines then lie within 45° of straight down, every pair has d_i·d_j ≥ 0, and max_m Σ max(0, m·d) = |Σ d| (for m with facing set S, m·Σ_S d ≤ |Σ_S d| ≤ |Σ d| because the cross terms are non-negative, and m = Σ d/|Σ d| faces every line). For θ &gt; 45° and for New it is a multi-start search that includes the dot's own normal, so it can fall short of the true maximum but never exceed it, and no dot scores above 1.</p>
  <figure>
    <div class="media"><img src="media/convergence.png" alt="Log-log plot of the largest and mean gap between New+G and New+J against N for each jet angle, and the D+G over D+J ratio error against N" loading="lazy"></div>
    <figcaption><b>The numerics.</b> The full grid (both dishes, every pose and jet angle{cells}) recomputed at N = {n_(Ns[0])} to {n_(Ns[-1])}, each N four times the last. Left: the largest (solid) and mean (dashed) |New+G − New+J| over dish means; middle: the same over single dots; right: the median relative gap between D+G and its limit, the dish mean of (a_i/R)²·D+J_i with a_i = h_i·tan θ per dot. The largest single-dot gap stays near 1 until N is very large: among the two million dot poses a few always sit where the θ-circle only grazes the disc, so at that N one of the two sets is still empty while the other is not. The theorem holds dot by dot, not uniformly over dots: each such dot converges once N puts enough points into its sliver, while the mean over dots falls steadily.</figcaption>
  </figure>
  <div class="scroll-x">
    <table class="cases">
      <thead><tr><th>N</th>{head}</tr></thead>
      <tbody>
        {chr(10).join('        ' + r for r in body).strip()}
      </tbody>
    </table>
  </div>
  <p class="prose">{rate}</p>
{tail_html}
</section>"""


def footer(d):
    obj = ("the object level, a dish's score as the sum of its 500 numerators over the sum of its 500 denominators, in "
           "its own section and behind a dot | object toggle in the explorer, which also gains the cup"
           if d["has_obj"] else
           "a placeholder for the object level (the sum of a dish's 500 numerators over the sum of its 500 "
           "denominators), whose grid with the cup is being computed")
    n, ok = len(d["deps"]), sum(x["dep"].get("status") != "gate" for x in d["deps"])   # loads past the joint gate
    deps = (f"dependency graphs for {dep_scope(d)}, each settled jointly in Isaac (the joint gate), "
            + ("its" if ok == n else f"the close dish pairs of the {ok} that pass it") + (" close dish pairs" if ok == n
                                                                                       else "")
            + " then dropped in both orders into that settled load (one edge type, A before B when "
            "only that order lands both dishes; a two-way pair means no order) and each acyclic graph certified by a "
            "build one dish at a time, the settled and built poses rescored under the three rules, and the edge lists "
            "published as JSON" if d["deps"] else "")
    changes = ("Outside the explorer and the appendix the page keeps three rules, Original, New and New+J: the rules "
               "key and cases, the per-dot figures" + (" (now with the cup)" if d["has_cup"] else "")
               + f" and the random-load tables and figures. Added: {obj}" + (f"; and {deps}" if deps else "") + ".")
    return f"""<footer>
  <p><b>What changed since 9 October.</b> {esc(changes)}</p>
  <p style="margin-top:8px">Code: <code>code/frigidaire/src/dishsim_frigidaire/dot_rules.py</code> (GPU engine), <code>code/planner/frigidaire/frigidaire_dot_rules_{{bench,grid,loads,tail,restricted,isaac,render,figures,page}}.py</code>, <code>code/planner/frigidaire/frigidaire_dependency_kit.py</code> (Isaac pair tests and builds), <code>code/planner/frigidaire/frigidaire_dependency_score.py</code> (packed{', settled' if d['dep_gate'] else ''} and built poses rescored), <code>code/initialization/frigidaire/frigidaire_random_load_{{kit,farm}}.py</code>. Results: <code>data/results/dot_rules/</code> (the dependency graphs in <code>dependency/pairs/</code>), <code>data/results/random_loads/rl_20261009/</code>. Revision 5 (the original) is still the live scorer; <code>exposure.py</code> is unchanged and nothing is committed.</p>
</footer>"""


EXTRA_CSS = """
.tiles.two{grid-template-columns:repeat(2,minmax(0,1fr))}
.proof{margin:0;padding-left:22px;display:flex;flex-direction:column;gap:12px;max-width:70ch}
.proof li{padding-left:4px}
#appendix h3{margin-top:6px}
@media (max-width:760px){ .tiles.two{grid-template-columns:1fr} }
.seg button:disabled{opacity:.4;cursor:not-allowed}
.seg button:disabled:hover{color:var(--ink-2)}
"""


LEVEL_JS = """if (byId("seg-level")) byId("seg-level").addEventListener("click", function(ev){
  var b = ev.target.closest("button"); if (!b) return;
  state.level = b.getAttribute("data-level"); setPressed("seg-level", "data-level", state.level);
  var one = byId("seg-entity").querySelector('[data-entity="dot"]');     /* no object level for a single dot */
  if (one) { one.disabled = state.level === "object"; one.title = state.level === "object" ? "no object level for a single dot" : ""; }
  if (state.level === "object" && state.entity === "dot") { state.entity = "plate"; setPressed("seg-entity", "data-entity", "plate"); }
  draw();
});"""


def edit_js(js):
    js = sub1(js, "var SCALE = G.scale, DOTS = G.dots;",
              'var SCALE = G.scale, DOTS = G.dots, NPTS = G.n_points || 64;\n'
              'function NFMT(v){ return Number(v).toLocaleString("en-US"); }')
    js = sub1(js, 'distance: {label: "distance from the disc centre (cm)",          min: -35,',
              'distance: {label: "distance from the disc centre (cm)",          min: (G.dot_neg ? -35 : 0),')
    js = sub1(js, 'var state = {x: "tilt", mode: "scores", entity: "plate", tilt: 45,',
              'var state = {x: "tilt", mode: "scores", entity: "plate", level: "dot", tilt: 45,')
    js = sub1(js, "var SHOWN = {O: true, D: true, DG: true, DJ: true, N: true, NG: true, NJ: true};",
              "var SHOWN = {O: true, D: true, DG: true, DJ: true, N: true, NG: true, NJ: true};\n"
              "var OBJ_RULES = {O: true, N: true, NJ: true};      /* the rules the object level shows */\n"
              'function inLevel(k){ return state.level !== "object" || !!OBJ_RULES[k]; }')
    js = sub1(js, 'return SHOWN[v.key] && (state.mode === "scores" || v.key !== "O");',
              'return SHOWN[v.key] && inLevel(v.key) && (state.mode === "scores" || v.key !== "O");')
    i = js.index("/* ---- the unit sunflower pattern")
    j = js.index("function withDiffs(s){")
    js = js[:i] + """/* ---- every entity from the grid, snapped to the nearest nodes; null where a dish pose was skipped.
   The dot is unblocked; its arrays add the line counts nG, nJ and New's best-facing normal (mDown, mGap, 0.1°).
   Level "object" reads a dish from G[kind + "_obj"]: Σ numerators / Σ denominators over its dots (O, N, NJ). ---- */
var RULE_KEYS = ["O", "D", "DG", "DJ", "N", "NG", "NJ"];
function hasNeg(){ return state.entity === "dot" && !!G.dot_neg; }
function gridScores(kind, tilt, hcm, dcm, theta){
  var obj = state.level === "object" && kind !== "dot", g = G[obj ? kind + "_obj" : kind];
  if (!g) return undefined;
  if (dcm < 0) { if (kind === "dot" && G.dot_neg) { g = G.dot_neg; dcm = -dcm; } else return undefined; }
  var ti = nearestIndex(G.tilts, tilt), hi = nearestIndex(G.heights, hcm), di = nearestIndex(G.distances, dcm), qi = nearestIndex(G.thetas, theta);
  var i = ((ti * G.heights.length + hi) * G.distances.length + di) * G.thetas.length + qi;
  if (g.O[i] < 0) return null;
  var s = {zG: g.zG ? g.zG[i] : null, zJ: g.zJ ? g.zJ[i] : null, n: kind === "dot" ? 1 : DOTS, obj: obj};
  RULE_KEYS.forEach(function(k){ s[k] = g[k] ? g[k][i] / SCALE : null; });
  if (kind === "dot") { s.nG = g.nG ? g.nG[i] : null; s.nJ = g.nJ ? g.nJ[i] : null; s.mDown = g.mDown ? g.mDown[i] / 10 : null; s.mGap = g.mGap ? g.mGap[i] / 10 : null; }
  return s;
}
function scoresAt(pose){ return gridScores(state.entity, pose.tilt, pose.height, pose.distance, pose.theta); }
function snapPose(pose){
  var d = pose.distance, ad = G.distances[nearestIndex(G.distances, Math.abs(d))];
  return {tilt: G.tilts[nearestIndex(G.tilts, pose.tilt)], height: G.heights[nearestIndex(G.heights, pose.height)],
          distance: d < 0 ? (hasNeg() ? -ad : d) : ad, theta: G.thetas[nearestIndex(G.thetas, pose.theta)]};
}
""" + js[j:]
    i, j = between(js, "function xValues(){", "\n}\n")
    js = js[:i] + """function xValues(){
  if (state.x === "distance" && hasNeg()) return G.distances.slice(1).reverse().map(function(d){ return -d; }).concat(G.distances);
  return AXES[state.x].nodes.slice();
}
""" + js[j:]
    js = sub1(js, 'half = state.entity === "dot" ? 0.5 : null', "half = null")
    js = sub1(js, 'if (state.x === "distance" && state.entity !== "dot") xt = [0, 10, 23.3, 35];',
              'if (state.x === "distance" && !hasNeg()) xt = [0, 10, 23.3, 35];')
    js = sub1(js, 'if (e < ax.min || (state.entity !== "dot" && e < 0)) return;',
              "if (e < ax.min || (!hasNeg() && e < 0)) return;")
    js = sub1(js, 'if (state.entity !== "dot") pts[i].forEach(function(p){', "pts[i].forEach(function(p){")
    js = sub1(js, 'if (!(state.entity !== "dot" && state.x === "distance" && mx < 0)) {',
              'if (!(!hasNeg() && state.x === "distance" && mx < 0)) {')
    js = sub1(js, '    if (state.mode !== "scores" && sr.key === "O") return;',
              '    if ((state.mode !== "scores" && sr.key === "O") || !inLevel(sr.key)) return;')
    js = sub1(js, 'chip(ro, (state.entity === "dot" ? "dot" : state.entity) + " at "',
              'chip(ro, (state.entity === "dot" ? "dot" : state.entity) + (state.level === "object" && state.entity !== "dot" ? ", object level," : "") + " at "')
    i, j = between(js, '  if (state.entity === "dot") {\n    if (s.m) {', 'of 64 on the disc");\n  }')
    js = js[:i] + """  if (state.entity === "dot") {
    if (s.mDown != null) chip(ro, "New's best-facing normal: " + s.mDown.toFixed(1) + "° from straight down, " + s.mGap.toFixed(1) + "° from this dot's normal");
    if (s.nG != null) chip(ro, "lines: G rules " + NFMT(s.nG) + " of " + NFMT(NPTS) + " fixed points, J rules " + NFMT(s.nJ) + " of " + NFMT(NPTS) + " on the disc");
  }""" + js[j:]
    js = sub1(js, ' else chip(ro, "dots with no line: G rules "', ' else if (s.zG != null) chip(ro, "dots with no line: G rules "')
    i, j = between(js, '  n.textContent = state.entity !== "dot"', "at its spot.\";\n")
    js = js[:i] + """  var nodes = "Sliders snap to the grid: tilt in " + (G.tilts[1] - G.tilts[0]) + "° steps from " + G.tilts[0] + " to " + G.tilts[G.tilts.length - 1] + "; heights " + G.heights.slice(0, -1).join(", ") + " and " + G.heights[G.heights.length - 1] + " cm; distance in " + (G.distances[1] - G.distances[0]) + " cm steps; θ " + G.thetas.slice(0, -1).join(", ") + " and " + G.thetas[G.thetas.length - 1] + "°; " + NFMT(NPTS) + " lines per set.";
  n.textContent = state.entity !== "dot"
    ? nodes + (state.level === "object" ? " Object level: sum over the " + DOTS + " dots of the numerator / sum of the per-dot denominators (Original, New and New+J)." : " Dish score = mean of its " + DOTS + " dots.")
    : "One unblocked dot, so every line is open: the original is 1 wherever a line carries weight, and New is the dot's own weight over the best-facing weight at its spot. " + nodes;
""" + js[j:]
    old = """    var inp = byId("in-" + state.x), v = Number(inp.value), nodes = AXES[state.x].nodes;
    if (state.entity !== "dot") { var i = nearestIndex(nodes, v) + (ev.key === "ArrowRight" ? 1 : -1); i = Math.max(0, Math.min(nodes.length - 1, i)); v = nodes[i]; }
    else v = v + (ev.key === "ArrowRight" ? 1 : -1);"""
    js = sub1(js, old, """    var inp = byId("in-" + state.x), v = Number(inp.value), nodes = xValues();
    var i = nearestIndex(nodes, v) + (ev.key === "ArrowRight" ? 1 : -1); i = Math.max(0, Math.min(nodes.length - 1, i)); v = nodes[i];""")
    seg = ('byId("seg-entity").addEventListener("click", function(ev){ var b = ev.target.closest("button"); if (!b) return; '
           'state.entity = b.getAttribute("data-entity"); setPressed("seg-entity", "data-entity", state.entity); draw(); });')
    js = sub1(js, seg, seg.replace("if (!b) return;", "if (!b || b.disabled) return;") + "\n" + LEVEL_JS)
    for gone in ("dotScores", "DOTCACHE", "of 64", "÷ 64"):
        if gone in js:
            raise SystemExit(f"JS still contains {gone!r}")
    return js


def copy_media(figdir, outdir, names):
    media = outdir / "media"
    media.mkdir(parents=True, exist_ok=True)
    for nm in names:
        src = figdir / nm
        if not src.exists():
            raise SystemExit(f"missing figure {src}")
        shutil.copyfile(src, media / nm)
    total = sum((media / nm).stat().st_size for nm in names) + (outdir / "index.html").stat().st_size
    if total > 10 * MB:                     # palette PNGs: about 3x smaller, no visible change on these plots
        from PIL import Image
        for nm in names:
            im = Image.open(media / nm).convert("RGB")
            im.quantize(colors=256, method=Image.Quantize.MEDIANCUT).save(media / nm, optimize=True)
        total = sum((media / nm).stat().st_size for nm in names) + (outdir / "index.html").stat().st_size
    return total


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--grid-dir", required=True)
    ap.add_argument("--grid-obj-dir", default=None,
                    help="object-level grid (<dish>_obj entities, cup); when given it is the page's grid: explorer "
                         "payload, grid numbers, per-dot captions and the object-level section")
    ap.add_argument("--sweep", required=True)
    ap.add_argument("--loads-dir", required=True)
    ap.add_argument("--farm-dir", default="data/results/random_loads/rl_20261009")
    ap.add_argument("--figures-dir", default="data/media/dot_rules/figures")
    ap.add_argument("--out", default="data/media/dot_rules/page")
    ap.add_argument("--template", default=str(TEMPLATE))
    ap.add_argument("--draft", action="store_true", help="mark the page as built from synthetic inputs")
    ap.add_argument("--picks-dir", default="data/media/dot_rules/isaac")
    ap.add_argument("--renders-dir", default="data/media/dot_rules/renders")
    ap.add_argument("--restricted-dir", default="data/results/dot_rules/restricted")
    ap.add_argument("--dependency-dir", default=None,
                    help="Isaac ordered-pair dependency graphs (data/results/dot_rules/dependency/pairs); omitted: "
                         "no section")
    ap.add_argument("--rerun-dir", default=None,
                    help="a second pairwise run of the same loads (<id>.json): edges reproduced; omitted: not shown")
    ap.add_argument("--resettled-dir", default=None,
                    help="goals re-settled by a sequential build (<id>.json, <id>.scores.json): a sub-section closing "
                         "the dependency section; omitted: none")
    ap.add_argument("--picks", default="data/media/dot_rules/isaac/restr_picks.json",
                    help="restricted picks: their top ranks order the dependency section")
    a = ap.parse_args()
    tpl = open(F.resolve(a.template), encoding="utf-8").read()
    gdir = F.level_dir(a.grid_obj_dir or a.grid_dir)
    grid = F.load_grid(gdir)
    N = int(grid["n_points"])
    pd = {k: F.load_perdot(gdir, k) for k in ("plate", "bowl", "cup") if (gdir / f"perdot_{k}.npz").is_file()}
    if not pd:
        raise SystemExit(f"no perdot_<dish>.npz in {gdir}")
    farm = F.load_farm(a.farm_dir)
    recs = F.attach_objects(F.load_records(a.loads_dir, farm))
    if not recs:
        print(f"[page] no scored loads in {a.loads_dir}: the random-load section reports the farm only", flush=True)
    payload = grid_payload(grid)
    above, mx, scope = above_and_max(grid, gdir, pd)
    deps = F.load_dependency(a.dependency_dir) if a.dependency_dir else []
    reruns = {x["dep"]["load_id"]: x for x in F.load_dependency(a.rerun_dir)} if a.rerun_dir else {}
    resettled = F.load_dependency(a.resettled_dir) if a.resettled_dir else []
    pk = F.resolve(a.picks)
    picks = json.loads(pk.read_text()) if pk.is_file() else []
    tops = sorted((p for p in picks if p.get("side") == "top"), key=lambda p: p.get("rank", 99))
    d = {"N": N, "Nf": n_(N), "draft": a.draft, "has_neg": "dot_neg" in payload, "grid": grid,
         "has_obj": any(f"{k}_obj" in payload for k in ("plate", "bowl", "cup")), "has_cup": "cup" in payload,
         "P0": F.grid_at(grid, "plate", 0, 14, 0, 30), "P90": F.grid_at(grid, "plate", 90, 14, 0, 30),
         "dot14": F.grid_at(grid, "dot", 0, 14, 0, 30), "dot3": F.grid_at(grid, "dot", 0, 3, 0, 30),
         "dot26": F.grid_at(grid, "dot", 0, 26, 0, 30),
         "poses": int(sum(np.sum(np.asarray(grid[e]["O"]) >= 0) for e in ("plate", "bowl", "cup") if e in grid)),
         "above": above, "maxN": mx, "above_scope": scope, "pd": pd,
         "sweep": F.load_sweep(a.sweep), "farm": farm, "recs": recs, "summ": F.summarize(recs),
         "farm_dir": str(F.resolve(a.farm_dir)), "picks_dir": a.picks_dir, "restricted_dir": a.restricted_dir,
         "loads_dir": a.loads_dir, "deps": deps, "dep_picks": [p["load_id"] for p in tops],
         "dep_gate": any(isinstance(x["dep"].get("gate"), dict) for x in deps), "reruns": reruns,
         "resettled": resettled if deps else []}
    head = tpl[:tpl.index('<main class="wrap">')]
    i = tpl.index("</main>") + len("</main>")
    tail_style = tpl[i:tpl.index("<script>", i)]
    tail_style = sub1(tail_style, "</style>", EXTRA_CSS + "</style>")
    k = tpl.index("<script>\n(function(){")
    js = edit_js(tpl[k + len("<script>\n"):tpl.rindex("</script>")])
    page = (head + '<main class="wrap">\n\n' + header(d) + "\n\n" + rules(d) + "\n\n" + explorer(tpl, d) + "\n\n"
            + perdot(d) + "\n\n" + objlevel_section(d) + "\n\n" + loads_section(d) + "\n\n" + isaac_section(d)
            + "\n\n" + dependency_section(d) + "\n\n" + appendix(d) + "\n\n" + footer(d) + "\n\n</main>\n"
            + tail_style + "<script>\nwindow.GRID = " + json.dumps(payload, separators=(",", ":")) + ";\n</script>\n"
            + "<script>\n" + js + "</script>\n\n</body></html>\n")
    out = F.resolve(a.out)
    out.mkdir(parents=True, exist_ok=True)
    (out / "index.html").write_text(page, encoding="utf-8")
    names = [f"dots_{k}.png" for k in pd] + ["convergence.png"]
    if recs:
        names += ["rand_record.png", "rand_spread.png"] + [f"rand_{w}5_{F.SLUG[r]}.png" for r in RULES3
                                                           for w in ("top", "bottom")]
    if d["has_obj"]:
        names.append("objlevel_slices.png")
    names += [f"depgraph_{x['dep']['load_id']}.png" for x in deps]
    names += [f"depgraph_{x['dep']['load_id']}_resettled.png" for x in d["resettled"]
              if x["dep"].get("status") != "resettle_failed"]
    total = copy_media(F.resolve(a.figures_dir), out, names)
    if deps:                                  # the edge lists, linked from the dependency captions
        (out / "data").mkdir(parents=True, exist_ok=True)
        for x in deps:
            dst = out / "data" / f"depgraph_{x['dep']['load_id']}.json"
            if not x["dep"].get("edges"):                 # no edges (a load the joint gate rejected): not linked
                dst.unlink(missing_ok=True)
                continue
            shutil.copyfile(x["path"], dst)
            total += dst.stat().st_size
        for x in d["resettled"]:                          # the re-settled goals' edge lists, linked the same way
            dst = out / "data" / f"depgraph_{x['dep']['load_id']}_resettled.json"
            if not x["dep"].get("edges"):
                dst.unlink(missing_ok=True)
                continue
            shutil.copyfile(x["path"], dst)
            total += dst.stat().st_size
    for src in sorted(F.resolve(a.renders_dir).glob("*.jpg")):     # the Isaac-scene stills, kept as JPEG
        shutil.copyfile(src, out / "media" / f"isaac_{src.name}")
        total += src.stat().st_size
    size = (out / "index.html").stat().st_size
    print(f"[page] N {N:,} from {gdir}; dishes {', '.join(pd)}; object level {'yes' if d['has_obj'] else 'no (note)'}; "
          f"{d['summ']['n']} scored loads; sweep {len(d['sweep']['N'])} levels; {len(deps)} dependency graphs; "
          + (f"{len(reruns)} reruns; " if reruns else "") + (f"{len(d['resettled'])} re-settled; " if d["resettled"] else "")
          + f"index.html {size / MB:.2f} MB; total {total / MB:.2f} MB", flush=True)
    if size > 15 * MB:
        raise SystemExit(f"index.html is {size / MB:.2f} MB, above the 15 MB limit")
    print(f"[RESULT] PASS page {out / 'index.html'}", flush=True)


if __name__ == "__main__":
    F.run_main(main, "page")
