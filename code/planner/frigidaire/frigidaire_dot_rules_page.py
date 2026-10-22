#!/usr/bin/env python3
# Copyright (c) 2026, dishsim project.
# SPDX-License-Identifier: BSD-3-Clause
"""Build the Dot Score Rules page at large N (Kit-free): the header, the three rules, the explorer, the per-dot figures.

Template: the 8 October page (docs/Dot Score Rules/index.html, a saved published page whose publish wrapper is dropped).
Kept: its <head>, CSS tokens (dark mode included) and the explorer markup (edited: the dot is read from the grid, N
replaces 64, a dot | object level toggle and the cup when the grid has them); the chart JS is EXPLORER_JS (New+J only,
one curve per entity, Show = score | relative to the kind's best). Rewritten from the data: the header tiles, the rules
(three: Original, New, New+J), the explorer's prose (the kind ceilings, kind_best) and the per-dot captions. This is the
page as published since Version 21 (15 October 2026, trimmed to these sections; the object level, goal loads, Isaac
scenes, dependency graphs and appendix of Version 20 are in git history before 2026-10-22).

Sources:
  --grid-obj-dir / --grid-dir (the box; the object-level grid when given): the grid level, grid.json with every rule,
      perdot_<dish>.npz and convergence.json: every number on the page, the explorer's payload (window.GRID) built
      from it.
  --grid-payload PAGE: the payload read from a saved page instead (its window.GRID). Alone, with no grid level (the
      Mac), the numbers only the full grid and the per-dot arrays hold print as nan or –: the header tiles but the dish
      pose count, the rules' cases and N x 0.928, the per-dot captions.
  --figures-dir: the per-dot PNGs, dots_<dish>.png (frigidaire_dot_rules_figures.py, same grid level).
Run from the repo root after frigidaire_dot_rules_figures.py:
  code/util/run_py.sh code/planner/frigidaire/frigidaire_dot_rules_page.py \
      --grid-dir data/results/dot_rules/grid/N0262144 --grid-obj-dir data/results/dot_rules/grid_obj/N0262144 \
      --figures-dir data/media/dot_rules/figures --out data/media/dot_rules/page
  python3 code/planner/frigidaire/frigidaire_dot_rules_page.py --grid-payload data/media/dot_rules/page_v22/index.html \
      --figures-dir data/media/dot_rules/page_v22/media --out <dir>
"""
from __future__ import annotations

import argparse
import html
import json
import math
import shutil
import sys
from pathlib import Path

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
    if v is None or not math.isfinite(v):
        return "–"
    if v == 0:
        return "0"
    if abs(v) >= 0.001:
        return f"{v:.4f}".rstrip("0").rstrip(".") if abs(v) < 0.01 else f"{v:.3f}"
    e = int(math.floor(math.log10(abs(v))))
    return f"{v / 10 ** e:.1f}·10{str(e).translate(SUP)}"


def n_(v):
    return f"{int(v):,}"


def esc(s):
    return html.escape(str(s), quote=True)


# ------------------------------------------------------------------ data
def grid_payload(grid):
    """The explorer's grid (2026-10-14: New+J only): the axes, each entity's NJ (negative = a skipped dish pose), the
    dishes' zJ and object-level NJ, the dot's J line count."""
    keep = ("tilts", "heights", "distances", "thetas", "scale", "dots", "n_points")
    out = {k: grid[k] for k in keep}
    for ent in ("plate", "bowl", "cup"):
        if ent in grid:
            out[ent] = {k: grid[ent][k] for k in ("NJ", "zJ")}
            if f"{ent}_obj" in grid:
                out[f"{ent}_obj"] = {"NJ": grid[f"{ent}_obj"]["NJ"]}
    dot = grid.get("dot")
    if not dot:
        raise SystemExit("grid.json has no 'dot' entity")
    out["dot"] = {k: dot[k] for k in ("NJ", "nJ") if k in dot}
    return out


def kind_best(grid, theta=30, key="NJ"):
    """Each entity's best New+J over every tilt, height and distance at ``theta`` (2026-10-21, the explorer's relative
    view; at the object level these are the load score's kind ceilings). Skipped poses are negative. Standard library."""
    nT, nH, nD, nQ = (len(grid[k]) for k in ("tilts", "heights", "distances", "thetas"))
    qi = grid["thetas"].index(theta)
    return {ent: max(grid[ent][key][((t * nH + hi) * nD + di) * nQ + qi] for t in range(nT) for hi in range(nH)
                     for di in range(nD)) / float(grid["scale"])
            for ent in ("dot", "plate", "bowl", "cup", "plate_obj", "bowl_obj", "cup_obj") if ent in grid}


REL_BUTTON = "relative to the kind's best"
REL_CAPTION = ("New+J scores, or each relative to its kind's best: the same entity's highest score over every tilt, "
               "height and distance of the grid at the chosen θ and level, one constant per curve except along θ. Gaps: "
               "a dish pose skipped because the dish would dip below the disc plane. On the θ axis, the dark bar marks 1° "
               "to 35°: the jets of the one lower arm with published angles. On the distance axis, the dotted line marks "
               "the disc edge.")


def relative_prose(grid):
    """The explorer prose on Show = relative to the kind's best (2026-10-21), the constants from kind_best."""
    kb = kind_best(grid)
    obj = ", ".join(f"{k} {kb[k + '_obj']:.3f}" for k in ("plate", "bowl", "cup") if k + "_obj" in kb)
    dot = ", ".join(f"{k} {kb[k]:.3f}" for k in ("plate", "bowl", "cup") if k in kb)
    at30 = (f"At θ 30° the object-level constants are {obj} (dot level: {dot}; the single dot's best is 1). They are "
            "the load score's ceilings: a dish in a load scores its object-level New+J over its kind's constant, "
            "clipped at 1 and 0 when it pools water, and the load scores the mean over its dishes. " if obj else
            f"At θ 30° the constants are {dot}; the single dot's best is 1. ")
    return ("Show switches between the score and the score relative to the kind's best: one constant per entity, level "
            "and θ, the entity's highest score over every tilt, height and distance of the grid at that θ. " + at30 +
            "Along tilt, height and distance a relative curve is the score curve rescaled: it keeps its shape, reaches 1 "
            "only at the kind's best spot and never passes it, so distance from the arm stays a penalty; along θ each "
            "point divides by its own θ's constant. ")


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
    a = sum(int(p["n_above1"].sum()) for p in perdot.values())
    m = max(float(p["scores"][:, RULES.index("N")].max()) for p in perdot.values())
    return (above if above is not None else a), (mx if mx is not None else m), "perdot"


def _flat_nums(v):
    if isinstance(v, (int, float)):
        return [v]
    if isinstance(v, dict):
        return [x for u in v.values() for x in _flat_nums(u)]
    if isinstance(v, list):
        return [x for u in v for x in _flat_nums(u)]
    return []


# ------------------------------------------------------------------ sections
def header(d):
    P0, P90 = d["P0"], d["P90"]
    above, mx, scope = d["above"], d["maxN"], d["above_scope"]
    where = (f"in all seven rules over {n_(d['poses'])} dish poses" if scope == "grid" else  # per-dot: no payload alone
             "in all seven rules over the "
             f"{n_(sum(int(p['scores'].shape[0] * p['scores'].shape[2]) for p in d['pd'].values()))} dot-poses of the "
             "per-dot figures")
    draft = " · DRAFT, synthetic inputs" if d["draft"] else ""
    if d["has_obj"]:
        obj = ("A dish has two scores: the dot level, the mean of its 500 dot scores, and the object level, the sum of "
               "its 500 numerators over the sum of its 500 denominators, a ratio of sums in which each dot counts in "
               "proportion to its denominator. The explorer switches between the two levels.")
    else:
        obj = ("A dish will also get an object-level score, the sum of its 500 numerators over the sum of its 500 "
               "denominators (a ratio of sums); that grid, with the cup, is being computed, and the explorer toggle "
               "comes with it.")
    lead = ("A dot's score says how exposed it is to the lower spray arm. The original rule (revision 5) can rank a dot "
            "that sees one spray point above a dot that sees all of them. New divides by the best-facing dot at the "
            "same spot instead of by the dot's own weight, and New+J does the same over near-vertical jets only. The "
            "page compares these three rules, and the explorer shows New+J alone, one curve per dot or dish, as a "
            f"score or relative to its kind's best. Every result uses N = {d['Nf']} lines per set. {obj}")
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
         G: those of the N within θ of vertical below the dot (the G rules, kept in the grid file)
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
  <p class="prose">The original puts case 1 above case 2. New puts them in the right order and keeps the ideal dot at 1. With {Nf} lines one spray point is worth almost nothing under New: a dot counts as exposed only when a broad share of the arm reaches it. The maximum over m is pure geometry: it needs no extra ray cast. For θ ≤ 45° the New+J reference is exact with no search: every such line lies within 45° of straight down, so no two lines are more than 90° apart, and then the best-facing weight is the length of the sum of the line directions. Otherwise m is turned toward the mean direction of the lines it faces, from 8 starts per dot: the dot's own normal, which keeps every New score at or below 1, straight down, the 4 best of 256 directions screened on a coarse sample, and two 120° turns of the best one. A last round restarts from small turns around the best direction found (1° to 8° and every 30° about the vertical, 2° and 5° in elevation), to hop between the shallow ripples a finite sunflower leaves in W. For N ≤ 1,024 every line is also a start, as in the original search; at larger N the search without them already matches the original's every-line result.</p>
</section>"""


def explorer(tpl, d):
    """The explorer (2026-10-14): New+J only, one curve per entity (dot, plate, bowl, cup), Show = score | relative to
    the kind's best (2026-10-21; was the best tilt at the spot, and before that 0 deg), Level = dot | object; the other
    six rules stay in the grid file."""
    i, j = between(tpl, '<section id="explorer">', "</section>")
    sec, g = tpl[i:j], d["grid"]
    dishes = [e for e in ("plate", "bowl", "cup") if e in g]
    who = ", ".join(f"the {e}" for e in dishes[:-1]) + f" and the {dishes[-1]}"
    prose = (f"One rule, New+J, one curve per entity: one unblocked dot, {who}, all read from one grid computed with "
             f"N = {d['Nf']} lines per set (tilt in 5° steps from 0° to 180°, nine heights, distance in 5 cm steps, ten "
             "jet angles). Pick which slider runs along the x-axis; the other three set the pose. Click a legend entry "
             "to hide or show an entity. Each dish is alone, with its own mesh as the only blocker, and its food-contact "
             "centre sits at the chosen height and distance; past 90° the food side turns away from the arm. "
             + relative_prose(g) +
             "Level switches a dish between the dot level, the mean of its 500 dot scores, and the object level, the "
             "sum of its 500 numerators over the sum of its 500 denominators; a single dot has no object level. "
             "Original, D, D+G, D+J, New and New+G are kept in the grid file.")
    a, b = between(sec, '<p class="prose">', "</p>")
    sec = sec[:a] + '<p class="prose">' + esc(prose) + "</p>" + sec[b:]
    sec = sub1(sec, '<button type="button" data-mode="scores" aria-pressed="true">scores</button><button type="button" '
                    'data-mode="diff" aria-pressed="false">difference from the original</button>',
               '<button type="button" data-mode="scores" aria-pressed="true">score</button><button type="button" '
               f'data-mode="rel" aria-pressed="false">{REL_BUTTON}</button>')
    a, b = between(sec, '        <span class="ctl-label">For</span>', "        </div>\n")
    level = ('        <span class="ctl-label">Level</span>\n'
             '        <div class="seg" id="seg-level" role="group" aria-label="Dot level or object level">\n'
             '          <button type="button" data-level="dot" aria-pressed="true">dot</button><button type="button" '
             'data-level="object" aria-pressed="false">object</button>\n        </div>\n') if d["has_obj"] else ""
    sec = sec[:a] + level + sec[b:]
    sec = sub1(sec, '<input id="in-distance" type="range" min="-35"', '<input id="in-distance" type="range" min="0"')
    sec = sub1(sec, 'aria-label="Rules in the chart"', 'aria-label="Entities in the chart"')
    sec = sub1(sec, 'aria-label="The five dot-score rules against the chosen slider.',
               'aria-label="New+J of the dot and the dishes against the chosen slider.')
    a, b = between(sec, "<figcaption>", "</figcaption>")
    sec = sec[:a] + "<figcaption>" + esc(REL_CAPTION) + "</figcaption>" + sec[b:]
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
        keep = ""
        if 0 in ti and 90 in ti:                  # the share of the 0 deg score kept on edge (2026-10-14)
            sh = {r: m[ti[90], RULES.index(r)] / m[ti[0], RULES.index(r)] for r in ("O", "NJ")}
            keep = (f" On edge the {dish} keeps {100 * sh['O']:.0f} % of its 0° score under Original and "
                    f"{100 * sh['NJ']:.1f} % under New+J.")
        cap = (f"{lead} Means facing the arm (tilt 0): {at(0)}. On edge (90°): {at(90)}. Food side away (180°): "
               f"{at(180)}" + (f"; {' and '.join(low)} below 0.01." if low else ".") + keep)
        out.append(f"""  <figure>
    <div class="media"><img src="media/dots_{dish}.png" alt="The {dish}, 500 dots, coloured by score under Original, New and New+J at tilt 0, 45, 90, 135 and 180 degrees" loading="lazy"></div>
    <figcaption>{esc(cap)}</figcaption>
  </figure>""")
    who = dishes[0] if len(dishes) == 1 else ", ".join(f"the {x}" for x in dishes[:-1]) + f" and the {dishes[-1]}"
    return f"""<section>
  <p class="eyebrow">Per dot</p>
  <h2>{who[0].upper() + who[1:]}, dot by dot</h2>
""" + "\n".join(out) + "\n</section>"


# The footer as published in Version 23 (21 October 2026); a new version adds its own line (and drops "bench," from the
# code list: that script was deleted on 2026-10-22).
FOOTER = """<footer>
  <p>Code: <code>code/frigidaire/src/dishsim_frigidaire/dot_rules.py</code> (GPU engine), <code>code/planner/frigidaire/frigidaire_dot_rules_{bench,grid,figures,page}.py</code>. Results: <code>data/results/dot_rules/grid/N0262144/</code> and <code>grid_obj/N0262144/</code>. Revision 5 (the original) is still the live scorer; <code>exposure.py</code> is unchanged. Trimmed 15 October 2026 to the rules, the explorer and the per-dot figures; the full Version 20 page (object level, goal loads, Isaac scenes, dependency graphs, appendix) is kept on the Brev box at <code>data/media/dot_rules/page_v20/</code>. Version 22 (16 October 2026): the explorer&#x27;s relative view divides by the best tilt at the spot instead of 0°. Version 23 (21 October 2026): it divides by one constant per kind and θ, the kind&#x27;s best over every tilt, height and distance (at θ 30° and the object level, the ceilings the load score divides by).</p>
</footer>"""


EXTRA_CSS = """
.tiles.two{grid-template-columns:repeat(2,minmax(0,1fr))}
@media (max-width:760px){ .tiles.two{grid-template-columns:1fr} }
.seg button:disabled{opacity:.4;cursor:not-allowed}
.seg button:disabled:hover{color:var(--ink-2)}
footer code{overflow-wrap:anywhere}
"""


EXPLORER_JS = r"""(function(){
"use strict";
/* The explorer (2026-10-14): New+J only, one curve per entity, read from the grid at the nearest nodes.
   Show "relative to the kind's best" (2026-10-21) divides by one constant per entity, level and θ: the entity's largest
   score over every tilt, height and distance of the grid at that θ (skipped poses left out), so a relative curve keeps
   the score curve's shape along tilt, height and distance and never passes 1; at θ 30° and the object level these are
   the load score's kind ceilings. Level "object" reads a dish from G[kind + "_obj"]. */
var G = window.GRID;
var NS = "http://www.w3.org/2000/svg";
var SCALE = G.scale, DOTS = G.dots, NPTS = G.n_points || 64;
function NFMT(v){ return Number(v).toLocaleString("en-US"); }
var ENTS = [
  {key: "dot",   name: "one dot", css: "--v-o",  width: 3},
  {key: "plate", name: "plate",   css: "--v-j",  width: 2.2},
  {key: "bowl",  name: "bowl",    css: "--v-jd", width: 2.2},
  {key: "cup",   name: "cup",     css: "--v-d",  width: 2.2}
].filter(function(e){ return !!G[e.key]; });
var SHOWN = {dot: true, plate: true, bowl: true, cup: true};
var AXES = {
  tilt:     {label: "tilt (°): 0 = facing the arm, 90 = on edge, 180 = facing away", min: 0, max: 180, unit: "°",   nodes: G.tilts},
  height:   {label: "height above the disc (cm)",                                  min: 3, max: 26,  unit: " cm", nodes: G.heights},
  distance: {label: "distance from the disc centre (cm)",                          min: 0, max: 35,  unit: " cm", nodes: G.distances},
  theta:    {label: "θ: jet angle from vertical (°)",                              min: 5, max: 85,  unit: "°",   nodes: G.thetas}
};
var state = {x: "tilt", mode: "scores", level: "dot", tilt: 45, height: 14, distance: 0, theta: 30};

function byId(id){ return document.getElementById(id); }
function mk(tag, attrs, parent){
  var e = document.createElementNS(NS, tag), k;
  for (k in attrs) if (Object.prototype.hasOwnProperty.call(attrs, k)) e.setAttribute(k, attrs[k]);
  if (parent) parent.appendChild(e);
  return e;
}
function clear(node){ while (node.firstChild) node.removeChild(node.firstChild); }
function fmt(v, d){ return v == null ? "–" : v.toFixed(d); }
function nearestIndex(list, v){ var b = 0, i; for (i = 1; i < list.length; i++) if (Math.abs(list[i] - v) < Math.abs(list[b] - v)) b = i; return b; }

/* ---- the grid: one flat array per entity, index ((tilt * H + height) * D + distance) * Q + theta ---- */
function source(key){
  if (state.level === "object") return key === "dot" ? null : (G[key + "_obj"] || null);
  return G[key] || null;
}
function flat(tilt, hcm, dcm, theta){
  var ti = nearestIndex(G.tilts, tilt), hi = nearestIndex(G.heights, hcm), di = nearestIndex(G.distances, dcm), qi = nearestIndex(G.thetas, theta);
  return ((ti * G.heights.length + hi) * G.distances.length + di) * G.thetas.length + qi;
}
function kindBest(g, theta){  /* the largest score over every tilt, height and distance at θ (the flat index's last axis) */
  var b = -1, i;
  for (i = nearestIndex(G.thetas, theta); i < g.NJ.length; i += G.thetas.length) if (g.NJ[i] > b) b = g.NJ[i];
  return b;
}
function cell(key, pose){      /* {score, ref, rel} or {skip}; null when the entity has no grid at this level */
  var g = source(key);
  if (!g) return null;
  var v = g.NJ[flat(pose.tilt, pose.height, pose.distance, pose.theta)];
  if (v < 0) return {skip: true};
  var r = kindBest(g, pose.theta);
  return {score: v / SCALE, ref: r / SCALE, rel: r > 0 ? v / r : null};
}
function valueOf(c){ if (!c || c.skip) return null; return state.mode === "scores" ? c.score : c.rel; }
function why(c){
  if (!c) return "no object level for a single dot";
  if (c.skip) return "skipped: the dish would dip below the disc plane";
  return null;
}
function available(e){ return !!source(e.key); }
function activeSeries(){ return ENTS.filter(function(e){ return SHOWN[e.key] && available(e); }); }
function snapPose(p){
  return {tilt: G.tilts[nearestIndex(G.tilts, p.tilt)], height: G.heights[nearestIndex(G.heights, p.height)],
          distance: G.distances[nearestIndex(G.distances, p.distance)], theta: G.thetas[nearestIndex(G.thetas, p.theta)]};
}
function buildCurve(){
  return AXES[state.x].nodes.map(function(x){
    var pose = {tilt: state.tilt, height: state.height, distance: state.distance, theta: state.theta}, c = {};
    pose[state.x] = x;
    ENTS.forEach(function(e){ c[e.key] = cell(e.key, pose); });
    return {x: x, c: c};
  });
}

/* ---- drawing ---- */
var W = 900, H = 430, L = 50, Rm = 118, T = 30, B = 58;
function sx(ax, x){ return L + (x - ax.min) / (ax.max - ax.min) * (W - L - Rm); }
function yScale(rows, series){   /* 0 to 1; grown in steps of 0.2 when a relative value passes 1 */
  var hi = 1, i, j, v, t, ticks = [];
  for (i = 0; i < rows.length; i++) for (j = 0; j < series.length; j++) { v = valueOf(rows[i].c[series[j].key]); if (v != null && v > hi) hi = v; }
  if (hi <= 1) return {min: 0, max: 1.05, ticks: [0, 0.2, 0.4, 0.6, 0.8, 1]};
  hi = Math.ceil(hi * 5 - 1e-9) / 5;
  for (t = 0; t <= hi + 1e-9; t += 0.2) ticks.push(Math.round(t * 10) / 10);
  return {min: 0, max: hi + 0.05, ticks: ticks};
}
function tickLabel(v){ if (v === 0) return "0"; var a = Math.abs(v), s = Math.abs(a - Math.round(a)) < 1e-9 ? String(Math.round(a)) : String(a).replace(/^0\./, "."); return (v < 0 ? "−" : "") + s; }
function sy(ys, v){ return T + (ys.max - v) / (ys.max - ys.min) * (H - T - B); }
var tipPos = null;

function draw(){
  var svg = byId("chart"), ax = AXES[state.x], rows = buildCurve(), series = activeSeries(), ys = yScale(rows, series);
  clear(svg);
  var plotW = W - L - Rm, plotH = H - T - B, i, j;
  for (i = 0; i < ys.ticks.length; i++) {
    var yy = sy(ys, ys.ticks[i]);
    mk("line", {x1: L, x2: W - Rm, y1: yy, y2: yy, stroke: state.mode !== "scores" && ys.ticks[i] === 1 ? "var(--axis)" : "var(--rule)", "stroke-width": "1"}, svg);
    var tl = mk("text", {x: L - 8, y: yy + 4, "text-anchor": "end", "font-size": "12"}, svg); tl.textContent = tickLabel(ys.ticks[i]);
  }
  var xt = {tilt: [0, 30, 60, 90, 120, 150, 180], height: [3, 5, 10, 15, 20, 26], distance: [0, 10, 23.3, 35], theta: [5, 20, 35, 50, 60, 70, 85]}[state.x];
  mk("line", {x1: L, x2: W - Rm, y1: T + plotH, y2: T + plotH, stroke: "var(--axis)", "stroke-width": "1"}, svg);
  for (i = 0; i < xt.length; i++) {
    var xx = sx(ax, xt[i]);
    mk("line", {x1: xx, x2: xx, y1: T + plotH, y2: T + plotH + 5, stroke: "var(--axis)", "stroke-width": "1"}, svg);
    var tx = mk("text", {x: xx, y: T + plotH + 20, "text-anchor": "middle", "font-size": "12"}, svg); tx.textContent = String(xt[i]);
  }
  var xl = mk("text", {x: L + plotW / 2, y: H - 12, "text-anchor": "middle", "font-size": "13"}, svg); xl.textContent = ax.label;
  var yl = mk("text", {x: 14, y: T + plotH / 2, "text-anchor": "middle", "font-size": "13", transform: "rotate(-90 14 " + (T + plotH / 2) + ")"}, svg);
  yl.textContent = state.mode === "scores" ? "New+J score" : "New+J relative to the kind's best";
  if (state.x === "distance") {
    var ex = sx(ax, 23.3);
    mk("line", {x1: ex, x2: ex, y1: T, y2: T + plotH, stroke: "var(--axis)", "stroke-width": "1", "stroke-dasharray": "2 4"}, svg);
    var el = mk("text", {x: ex + 5, y: T + 12, "font-size": "11.5"}, svg); el.textContent = "disc edge 23.3 cm";
  }
  if (state.x === "theta") {
    mk("rect", {x: sx(ax, 5), y: T - 10, width: sx(ax, 35) - sx(ax, 5), height: 5, fill: "var(--ink-2)", rx: 1}, svg);
    var bl = mk("text", {x: sx(ax, 35) + 6, y: T - 5, "font-size": "11.5"}, svg); bl.textContent = "measured arm's jets, 1° to 35°";
  }
  var ends = [];
  for (j = 0; j < series.length; j++) {
    var sr = series[j], pts = [], seg = [], last = null;
    for (i = 0; i < rows.length; i++) {
      var v = valueOf(rows[i].c[sr.key]);
      if (v == null || isNaN(v)) { if (seg.length) { pts.push(seg); seg = []; } continue; }
      seg.push([sx(ax, rows[i].x), sy(ys, Math.max(ys.min, Math.min(ys.max, v)))]); last = {x: rows[i].x, v: v};
    }
    if (seg.length) pts.push(seg);
    for (i = 0; i < pts.length; i++) {
      var d = pts[i].map(function(p, k){ return (k ? "L" : "M") + p[0].toFixed(1) + " " + p[1].toFixed(1); }).join("");
      mk("path", {d: d, fill: "none", stroke: "var(" + sr.css + ")", "stroke-width": sr.width, "stroke-linejoin": "round", "stroke-linecap": "round", opacity: ".95"}, svg);
      pts[i].forEach(function(p){ mk("circle", {cx: p[0], cy: p[1], r: 3.2, fill: "var(" + sr.css + ")", stroke: "var(--plate)", "stroke-width": "1.5"}, svg); });
    }
    if (last) ends.push({name: sr.name, y: sy(ys, Math.max(ys.min, Math.min(ys.max, last.v))), x: sx(ax, last.x), css: sr.css, v: last.v});
  }
  ends.sort(function(a, b){ return a.y - b.y; });
  for (i = 0; i < ends.length; i++) { ends[i].ly = ends[i].y; if (i && ends[i].ly - ends[i - 1].ly < 13) ends[i].ly = ends[i - 1].ly + 13; }
  var over = ends.length ? ends[ends.length - 1].ly - (T + plotH) : 0;
  if (over > 0) {
    for (i = 0; i < ends.length; i++) ends[i].ly -= over;
    for (i = ends.length - 1; i > 0; i--) if (ends[i].ly - ends[i - 1].ly < 13) ends[i - 1].ly = ends[i].ly - 13;
  }
  ends.forEach(function(e){
    if (Math.abs(e.ly - e.y) > 2) mk("line", {x1: e.x + 3, y1: e.y, x2: W - Rm + 10, y2: e.ly, stroke: "var(--axis)", "stroke-width": "1"}, svg);
    mk("line", {x1: W - Rm + 12, x2: W - Rm + 24, y1: e.ly, y2: e.ly, stroke: "var(" + e.css + ")", "stroke-width": "3"}, svg);
    var t = mk("text", {x: W - Rm + 30, y: e.ly + 4, "font-size": "12"}, svg);
    t.textContent = e.name + " " + fmt(e.v, 2);
  });
  var pose = snapPose({tilt: state.tilt, height: state.height, distance: state.distance, theta: state.theta}), mxx = sx(ax, pose[state.x]);
  mk("line", {x1: mxx, x2: mxx, y1: T, y2: T + plotH, stroke: "var(--ink)", "stroke-width": "1", "stroke-opacity": ".55"}, svg);
  series.forEach(function(sr){ var v = valueOf(cell(sr.key, pose)); if (v == null) return;
    mk("circle", {cx: mxx, cy: sy(ys, Math.max(ys.min, Math.min(ys.max, v))), r: 5, fill: "var(" + sr.css + ")", stroke: "var(--plate)", "stroke-width": "2"}, svg); });
  svg.__rows = rows; svg.__ax = ax; svg.__ys = ys; svg.__series = series;
  drawLegend(); drawReadout(pose, series); drawTable(rows, series); drawNote(rows, series);
  if (tipPos) hover(tipPos.px, tipPos.py, true);
}
function drawLegend(){
  var lg = byId("legend"); clear(lg);
  ENTS.forEach(function(e){
    var b = document.createElement("button"), k = document.createElement("span"), n = document.createElement("span"), ok = available(e);
    b.type = "button"; b.className = "lg-btn"; b.setAttribute("aria-pressed", SHOWN[e.key] && ok ? "true" : "false");
    b.disabled = !ok; b.title = ok ? "" : "no object level for a single dot";
    k.className = "line-key" + (e.key === "dot" ? " thick" : ""); k.style.color = "var(" + e.css + ")";
    n.className = "lg-name"; n.textContent = e.name;
    b.appendChild(k); b.appendChild(n);
    b.addEventListener("click", function(){ if (b.disabled) return; SHOWN[e.key] = !SHOWN[e.key]; draw(); });
    lg.appendChild(b);
  });
  var sk = document.createElement("span"); sk.className = "lg-static";
  sk.appendChild(document.createTextNode("gap: dish pose skipped (below the disc plane)"));
  lg.appendChild(sk);
}
function chip(parent, text){ var c = document.createElement("span"); c.className = "chip"; c.appendChild(document.createTextNode(text)); parent.appendChild(c); return c; }
function drawReadout(pose, series){
  var ro = byId("readout"); clear(ro);
  chip(ro, (state.level === "object" ? "object level, " : "") + "at " + pose.tilt + "°, " + pose.height + " cm up, " + pose.distance + " cm out, θ " + pose.theta + "°");
  series.forEach(function(sr){
    var c = cell(sr.key, pose), msg = why(c), ch = document.createElement("span"), k = document.createElement("span"), b = document.createElement("b");
    ch.className = "chip"; k.className = "line-key"; k.style.color = "var(" + sr.css + ")";
    b.textContent = msg ? "–" : fmt(valueOf(c), 3);
    ch.appendChild(k); ch.appendChild(b);
    ch.appendChild(document.createTextNode(" " + sr.name + (msg ? ": " + msg : (state.mode !== "scores" ? " (kind's best " + fmt(c.ref, 3) + ")" : ""))));
    ro.appendChild(ch);
  });
  if (SHOWN.dot && state.level !== "object" && G.dot && G.dot.nJ) {
    var nj = G.dot.nJ[flat(pose.tilt, pose.height, pose.distance, pose.theta)];
    chip(ro, "the dot's J lines: " + NFMT(nj) + " of " + NFMT(NPTS) + " on the disc");
  }
}
function drawTable(rows, series){
  var tbl = byId("tbl"); clear(tbl);
  var thead = document.createElement("thead"), tr = document.createElement("tr"), th = document.createElement("th");
  th.textContent = AXES[state.x].label; tr.appendChild(th);
  series.forEach(function(sr){ var h = document.createElement("th"); h.textContent = sr.name + (state.mode === "scores" ? "" : " (relative)"); tr.appendChild(h); });
  thead.appendChild(tr); tbl.appendChild(thead);
  var tb = document.createElement("tbody");
  rows.forEach(function(r){
    var row = document.createElement("tr"), td = document.createElement("td"); td.textContent = String(r.x); row.appendChild(td);
    series.forEach(function(sr){
      var c = r.c[sr.key], v = valueOf(c), cl = document.createElement("td");
      cl.textContent = v != null ? fmt(v, 3) : (c && c.skip ? "skipped" : "–");
      row.appendChild(cl);
    });
    tb.appendChild(row);
  });
  tbl.appendChild(tb);
}
function drawNote(rows, series){
  var n = byId("chart-note");
  var nodes = "Sliders snap to the grid: tilt in " + (G.tilts[1] - G.tilts[0]) + "° steps from " + G.tilts[0] + " to " + G.tilts[G.tilts.length - 1] + "; heights " + G.heights.slice(0, -1).join(", ") + " and " + G.heights[G.heights.length - 1] + " cm; distance in " + (G.distances[1] - G.distances[0]) + " cm steps; θ " + G.thetas.slice(0, -1).join(", ") + " and " + G.thetas[G.thetas.length - 1] + "°; " + NFMT(NPTS) + " lines per set.";
  n.textContent = nodes + (state.level === "object" ? " Object level: sum over the " + DOTS + " dots of the numerator / sum of the per-dot denominators." : " Dish score = mean of its " + DOTS + " dots.")
    + (state.mode !== "scores" ? " Relative = the score / the same entity's best score over every tilt, height and distance at this θ, at most 1." : "");
  ["tilt", "height", "distance", "theta"].forEach(function(k){
    byId("ctl-" + k).classList.toggle("is-x", k === state.x);
    byId("note-" + k).textContent = k === state.x ? "runs along the x-axis; its slider moves the marker" : "";
  });
}
/* ---- hover: a crosshair snapping to the nearest x, one tooltip with every entity ---- */
function hover(px, py, keep){
  var svg = byId("chart"), rows = svg.__rows, ax = svg.__ax, series = svg.__series, tip = byId("tip");
  if (!rows) return;
  var rect = svg.getBoundingClientRect(), xv = ax.min + (px / rect.width * W - L) / (W - L - Rm) * (ax.max - ax.min);
  var b = 0, i; for (i = 1; i < rows.length; i++) if (Math.abs(rows[i].x - xv) < Math.abs(rows[b].x - xv)) b = i;
  var r = rows[b];
  var old = svg.querySelector(".crosshair"); if (old) old.parentNode.removeChild(old);
  var cx = sx(ax, r.x) / W * rect.width;
  var ch = mk("line", {x1: sx(ax, r.x), x2: sx(ax, r.x), y1: T, y2: H - B, stroke: "var(--muted)", "stroke-width": "1", "class": "crosshair"}, svg);
  ch.style.pointerEvents = "none";
  clear(tip);
  var tt = document.createElement("div"); tt.className = "tt"; tt.textContent = AXES[state.x].label.split(":")[0].split(" (")[0] + " " + r.x + ax.unit; tip.appendChild(tt);
  series.forEach(function(sr){
    var c = r.c[sr.key], msg = why(c);
    var row = document.createElement("div"); row.className = "tr";
    var k = document.createElement("span"); k.className = "line-key"; k.style.color = "var(" + sr.css + ")"; k.style.width = "20px";
    var v = document.createElement("span"); v.className = "tv"; v.textContent = msg ? "–" : fmt(valueOf(c), 3);
    var n = document.createElement("span"); n.className = "tn"; n.textContent = sr.name + (msg ? ": " + msg : "");
    row.appendChild(k); row.appendChild(v); row.appendChild(n); tip.appendChild(row);
  });
  tip.hidden = false;
  var ww = byId("chart-wrap").clientWidth, tw = tip.offsetWidth;
  tip.style.left = Math.min(Math.max(0, cx + 14), ww - tw) + "px";
  tip.style.top = Math.max(0, Math.min(py - 20, rect.height - tip.offsetHeight - 8)) + "px";
  if (!keep) tipPos = {px: px, py: py};
}
function unhover(){ var svg = byId("chart"), old = svg.querySelector(".crosshair"); if (old) old.parentNode.removeChild(old); byId("tip").hidden = true; tipPos = null; }
/* ---- controls ---- */
function setPressed(group, attr, value){
  Array.prototype.forEach.call(byId(group).querySelectorAll("button"), function(b){ b.setAttribute("aria-pressed", b.getAttribute(attr) === value ? "true" : "false"); });
}
function onSeg(id, attr, key){
  if (!byId(id)) return;
  byId(id).addEventListener("click", function(ev){ var b = ev.target.closest("button"); if (!b || b.disabled) return; state[key] = b.getAttribute(attr); setPressed(id, attr, state[key]); draw(); });
}
onSeg("seg-x", "data-x", "x");
onSeg("seg-mode", "data-mode", "mode");
onSeg("seg-level", "data-level", "level");
["tilt", "height", "distance", "theta"].forEach(function(k){
  var inp = byId("in-" + k), out = byId("out-" + k);
  function upd(){ state[k] = Number(inp.value); out.textContent = inp.value + AXES[k].unit; draw(); }
  inp.addEventListener("input", upd);
  out.textContent = inp.value + AXES[k].unit;
});
(function(){
  var svg = byId("chart");
  svg.addEventListener("pointermove", function(ev){ var r = svg.getBoundingClientRect(); hover(ev.clientX - r.left, ev.clientY - r.top, false); });
  svg.addEventListener("pointerleave", unhover);
  svg.addEventListener("keydown", function(ev){
    if (ev.key !== "ArrowLeft" && ev.key !== "ArrowRight") return;
    ev.preventDefault();
    var inp = byId("in-" + state.x), nodes = AXES[state.x].nodes;
    var i = nearestIndex(nodes, Number(inp.value)) + (ev.key === "ArrowRight" ? 1 : -1);
    inp.value = String(nodes[Math.max(0, Math.min(nodes.length - 1, i))]); inp.dispatchEvent(new Event("input"));
  });
})();
draw();
})();
"""


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


def read_payload(path):
    """The explorer's grid payload (window.GRID) of a saved Dot Score Rules page."""
    text = Path(path).read_text(encoding="utf-8")
    i = text.index("window.GRID =") + len("window.GRID =")
    return json.JSONDecoder().raw_decode(text[i:].lstrip())[0]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--grid-dir", default=None, help="the dot-level grid level (grid.json, perdot_<dish>.npz)")
    ap.add_argument("--grid-obj-dir", default=None,
                    help="object-level grid (<dish>_obj entities, cup); when given it is the page's grid: explorer "
                         "payload, grid numbers and per-dot captions")
    ap.add_argument("--grid-payload", default=None,
                    help="a saved page (index.html): the explorer's payload from its window.GRID; with no grid level "
                         "the numbers only the full grid holds print as nan or – (module docstring)")
    ap.add_argument("--figures-dir", default="data/media/dot_rules/figures")
    ap.add_argument("--out", default="data/media/dot_rules/page")
    ap.add_argument("--template", default=str(TEMPLATE))
    ap.add_argument("--draft", action="store_true", help="mark the page as built from synthetic inputs")
    a = ap.parse_args()
    gsrc = a.grid_obj_dir or a.grid_dir
    if not (gsrc or a.grid_payload):
        raise SystemExit("needs --grid-obj-dir, --grid-dir or --grid-payload")
    tpl = open(F.resolve(a.template), encoding="utf-8").read()
    tpl = tpl[tpl.index("<body>\n") + len("<body>\n"):]          # a saved published page: its publish wrapper goes
    if gsrc:                                                      # the box: every number from the grid level
        gdir = F.level_dir(gsrc)
        grid = F.load_grid(gdir)
        pd = {k: F.load_perdot(gdir, k) for k in ("plate", "bowl", "cup") if (gdir / f"perdot_{k}.npz").is_file()}
        if not pd:
            raise SystemExit(f"no perdot_<dish>.npz in {gdir}")
        above, mx, scope = above_and_max(grid, gdir, pd)
        P0, P90 = F.grid_at(grid, "plate", 0, 14, 0, 30), F.grid_at(grid, "plate", 90, 14, 0, 30)
        dot14 = F.grid_at(grid, "dot", 0, 14, 0, 30)
    payload = read_payload(a.grid_payload) if a.grid_payload else grid_payload(grid)
    if not gsrc:                                                  # the saved page's payload alone
        nan = float("nan")
        gdir = a.grid_payload
        pd = {k: {"tilts": [], "means": None} for k in ("plate", "bowl", "cup") if k in payload}
        above, mx, scope = nan, nan, "grid"
        P0 = P90 = {"O": nan, "N": nan}
        dot14 = {"D": nan, "N": nan, "DJ": nan}
    N = int(payload["n_points"])
    d = {"N": N, "Nf": n_(N), "draft": a.draft, "grid": payload,
         "has_obj": any(f"{k}_obj" in payload for k in ("plate", "bowl", "cup")),
         "P0": P0, "P90": P90, "dot14": dot14,
         "poses": sum(v >= 0 for e in ("plate", "bowl", "cup") if e in payload for v in payload[e]["NJ"]),  # skipped: -1
         "above": above, "maxN": mx, "above_scope": scope, "pd": pd}
    head = tpl[:tpl.index('<main class="wrap">')]
    i = tpl.index("</main>") + len("</main>")
    tail_style = sub1(tpl[i:tpl.index("<script>", i)], "</style>", EXTRA_CSS + "</style>")
    page = (head + '<main class="wrap">\n\n' + header(d) + "\n\n" + rules(d) + "\n\n" + explorer(tpl, d) + "\n\n"
            + perdot(d) + "\n\n" + FOOTER + "\n\n</main>\n" + tail_style + "<script>\nwindow.GRID = "
            + json.dumps(payload, separators=(",", ":")) + ";\n</script>\n" + "<script>\n" + EXPLORER_JS + "</script>\n")
    out = F.resolve(a.out)
    out.mkdir(parents=True, exist_ok=True)
    (out / "index.html").write_text(page, encoding="utf-8")
    total = copy_media(F.resolve(a.figures_dir), out, [f"dots_{k}.png" for k in pd])
    size = (out / "index.html").stat().st_size
    print(f"[page] N {N:,} from {gdir}; dishes {', '.join(pd)}; object level {'yes' if d['has_obj'] else 'no'}"
          + ("" if gsrc else "; no grid level: the header tiles, the rules' cases and the per-dot captions print nan or –")
          + f"; index.html {size / MB:.2f} MB; total {total / MB:.2f} MB", flush=True)
    if size > 15 * MB:
        raise SystemExit(f"index.html is {size / MB:.2f} MB, above the 15 MB limit")
    print(f"[RESULT] PASS page {out / 'index.html'}", flush=True)


if __name__ == "__main__":
    F.run_main(main, "page")
