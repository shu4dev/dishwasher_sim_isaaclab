#!/usr/bin/env python3
# Copyright (c) 2026, dishsim project.
# SPDX-License-Identifier: BSD-3-Clause
"""Dimensioned drawings of the two FDPC4221AS racks, in centimetres (Kit-free).

One image per rack: plan view, elevations and a value table. The wires drawn are
the generated source geometry projected orthographically, and every number is read
from dishsim_frigidaire.geometry. Colour says where a value comes from: teal = the
user's tape measurement, orange = derived by the model from the tape values (the
user's tape figure in brackets), grey = estimate from the photo fit.

    python3 frigidaire/scripts/evaluation/frigidaire_rack_dimensions_cm.py [--out-dir media/frigidaire_rack_dimensions]

The curated copies in docs/figures/ are frigidaire_{upper,lower}_rack_cm.png.
"""
import argparse
from pathlib import Path
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "frigidaire/src"))
from dishsim_frigidaire import geometry as G

TAPE, DERIVED, EST = "#00707a", "#c0551a", "#7b8894"
INK, WIRE, TINE, BASKET, PAPER = "#23384b", "#c9d2d8", "#00707a", "#9a5b2c", "#fbfcfd"
CM = 100.0
P = G.PARAMETERS


def f(v):
    """Short cm text: 48, 51.5, 4.21."""
    return f"{v:.2f}".rstrip("0").rstrip(".")


def extent(comp, prefix):
    rows = [(np.asarray(p), r) for n, p, r in comp["wires"] if n.startswith(prefix)]
    lo = np.min([p.min(0) - r for p, r in rows], 0)
    hi = np.max([p.max(0) + r for p, r in rows], 0)
    return lo * CM, hi * CM


def tines(comp, key):
    """[(name, base_cm, tip_cm)] for the generated tine wires."""
    return [(n, np.asarray(p[0]) * CM, np.asarray(p[-1]) * CM)
            for n, p, _ in comp["wires"] if n.startswith(key) and "_Tooth" in n]


def z_span(comp, keep):
    """Lowest and highest wire surface z (cm) over the wires kept."""
    rows = [(np.asarray(q), r) for n, q, r in comp["wires"] if keep(n)]
    return min((q[:, 2] - r).min() for q, r in rows) * CM, max((q[:, 2] + r).max() for q, r in rows) * CM


def rib_extrema(rack):
    """Local extrema (x, z) in cm of the generated middle cross rib, right half, as drawn."""
    rib = [np.asarray(q) * CM for n, q, _ in rack["wires"] if n == "ContouredCrossU_10"][0]
    r = rib[rib[:, 0] > 0]
    r = r[np.argsort(r[:, 0])]
    z = r[:, 2]
    return [tuple(r[i, [0, 2]]) for i in range(1, len(z) - 1) if (z[i] - z[i - 1]) * (z[i + 1] - z[i]) < 0]


def wires(ax, comp, axes, shift=(0, 0, 0), keep=lambda n: True, color=WIRE, lw=.9, z=1):
    for name, path, _ in comp["wires"]:
        if keep(name):
            q = (np.asarray(path) + shift) * CM
            ax.plot(q[:, axes[0]], q[:, axes[1]], color=color, lw=lw, solid_capstyle="round", zorder=z)


def dim(ax, a, b, text, color=TAPE, off=(0, 0), rot=0, fs=9.5):
    a, b = np.asarray(a, float), np.asarray(b, float)
    ax.annotate("", xy=b, xytext=a, zorder=9,
                arrowprops={"arrowstyle": "<->", "color": color, "lw": 1.0, "shrinkA": 0, "shrinkB": 0})
    m = (a + b) / 2 + np.asarray(off, float)
    ax.text(*m, text, ha="center", va="center", color=color, fontsize=fs, rotation=rot, zorder=10,
            fontweight="bold" if color == TAPE else "normal",
            bbox={"facecolor": PAPER, "edgecolor": "none", "pad": 1.2})


def ext(ax, xs, ys):
    ax.plot(xs, ys, color="#aab6bf", lw=.6, ls=(0, (3, 2)), zorder=2)


def note(ax, x, y, text, color=EST, fs=9, **kw):
    ax.text(x, y, text, color=color, fontsize=fs, zorder=10,
            bbox={"facecolor": PAPER, "edgecolor": "none", "pad": 1.0}, **kw)


def view(ax, title):
    ax.set_aspect("equal")
    ax.axis("off")
    ax.set_title(title, loc="left", fontsize=12.5, color=INK, fontweight="bold", pad=8)


def table(ax, rows, title):
    ax.axis("off")
    ax.set_title(title, loc="left", fontsize=12.5, color=INK, fontweight="bold", pad=4)
    colour = {"tape": TAPE, "derived": DERIVED, "estimate": EST}
    t = ax.table(cellText=[r[:3] + (r[3],) for r in rows],
                 colLabels=("quantity", "your tape (cm)", "model (cm)", "source"),
                 colWidths=(.22, .17, .47, .1), loc="upper left", cellLoc="left")
    t.auto_set_font_size(False)
    t.set_fontsize(9.5)
    t.scale(1, 1.32)
    for (r, c), cell in t.get_celld().items():
        cell.set_edgecolor("#dbe2e7")
        cell.set_facecolor(PAPER if r else "#eef2f5")
        if r == 0:
            cell.set_text_props(color=INK, fontweight="bold")
        elif c in (2, 3):
            cell.set_text_props(color=colour[rows[r - 1][3]])


def legend(fig, y):
    for i, (label, c) in enumerate((("teal = your tape measurement", TAPE),
                                    ("orange = derived from your tape values (your figure in brackets)", DERIVED),
                                    ("grey = estimate from the photo fit", EST))):
        fig.text(.04 + (0, .22, .6)[i], y, "■ " + label, color=c, fontsize=10.5,
                 fontweight="bold" if c == TAPE else "normal")


# ---------------------------------------------------------------- upper rack

def upper(out):
    comps = G.build_components()
    rack, p = comps["UpperRack"], P["upper_rack"]
    lo, hi = extent(rack, "TopRim")
    W, D = hi[0] - lo[0], hi[1] - lo[1]
    ts = tines(rack, "BowlComb")
    xs, ys = (np.asarray(v) * CM for v in G.upper_tine_positions())
    mask = G.upper_tine_mask()                     # [position, column]
    pitch, cpitch = p["tine_spacing"] * CM, p["tine_column_pitch"] * CM
    side, front, rear = (p[k] * CM for k in ("tine_side_margin", "tine_front_margin", "tine_rear_margin"))
    tape = {k: v * CM for k, v in p["tine_margins_tape_m"].items()}
    rim_z = p["rim_height"] * CM
    rise, lean, dia = p["tine_height"] * CM, p["tine_tip_offset_y"] * CM, p["tine_diameter"] * CM
    prof = {k: v * CM for k, v in p["channel_profile"].items() if k != "note"}
    counts = mask.sum(0).tolist()
    absent = sorted({int(i) + 1 for i, _ in zip(*np.nonzero(~mask))})
    ex = rib_extrema(rack)
    near = lambda x0: min(ex, key=lambda e: abs(e[0] - x0))
    valley = near(xs[-1] + prof["mug_valley_offset_from_column"])
    crest = near(xs[-1] + prof["ridge_offset_from_column"])
    trough = near(hi[0] - p["rim_diameter"] / 2 * CM - prof["trough_inset_from_rim"])
    assert len(ex) == 4 and valley[1] < 0 < crest[1] and trough[1] < 0, ex
    z_lo, _ = z_span(rack, lambda n: n.startswith(("ContouredCrossU", "LongitudinalCradle")))
    z_top = z_span(rack, lambda n: n.startswith("TopRim"))[1]
    assert abs(z_top - z_lo - p["outer_height_tape"] * CM) < .05, z_top - z_lo
    # the drawing must show the generated geometry, not a stale parameter copy
    assert np.isclose(W, p["wire_width"] * CM) and np.isclose(D, p["wire_depth"] * CM), (W, D)
    assert len(ts) == sum(counts) and np.allclose(np.unique([b[0] for _, b, _ in ts]), xs), len(ts)

    fig = plt.figure(figsize=(21, 15.5), facecolor=PAPER)
    gs = fig.add_gridspec(3, 2, width_ratios=(1.12, 1), height_ratios=(1, 1, .62),
                          left=.03, right=.98, top=.905, bottom=.03, hspace=.16, wspace=.06)
    fig.text(.03, .965, "Upper rack, Frigidaire FDPC4221AS twin: every dimension in cm", fontsize=19,
             fontweight="bold", color=INK)
    fig.text(.03, .943, f"revision {p['geometry_revision']} · {sum(counts)} tines · wires are the generated "
             "source geometry, projected; front of the rack is -Y (bottom of the plan)", fontsize=11, color=INK)
    legend(fig, .925)

    # plan view
    ax = fig.add_subplot(gs[0:2, 0])
    view(ax, "Plan view (from above)")
    wires(ax, rack, (0, 1), keep=lambda n: "_Tooth" not in n and "Roller" not in n)
    for _, b, t in ts:
        ax.plot([b[0], t[0]], [b[1], t[1]], color=TINE, lw=2.2, zorder=4)
        ax.plot(b[0], b[1], "o", ms=5, mfc=TINE, mec="white", mew=.6, zorder=5)
    for i, j in zip(*np.nonzero(~mask)):
        ax.plot(xs[j], ys[i], "o", ms=7, mfc="none", mec=TINE, mew=1.3, zorder=5)
    for j, x in enumerate(xs):
        ax.text(x, hi[1] + 2.2, f"C{j + 1}\n{counts[j]} tines", ha="center", va="bottom",
                color=TAPE, fontsize=10, fontweight="bold")
    for i, y in enumerate(ys):
        ax.text(hi[0] + .9, y, f"{i + 1}", ha="left", va="center", color=INK, fontsize=8)
    ax.text(hi[0] + .9, hi[1] + .6, "position\n(front = 1)", ha="left", va="bottom", color=INK, fontsize=8)
    note(ax, xs[1] + 1.4, ys[absent[0] - 1] + pitch / 2 - .5, f"positions {absent[0]}-{absent[-1]}\nabsent in C2, C3",
         color=TAPE, fs=9.5, fontweight="bold")
    # outer size
    ext(ax, [lo[0], lo[0]], [hi[1], hi[1] + 9.5]); ext(ax, [hi[0], hi[0]], [hi[1], hi[1] + 9.5])
    dim(ax, (lo[0], hi[1] + 8.5), (hi[0], hi[1] + 8.5), f"{f(W)} outer rim width")
    ext(ax, [lo[0] - 7, lo[0]], [lo[1], lo[1]]); ext(ax, [lo[0] - 7, lo[0]], [hi[1], hi[1]])
    dim(ax, (lo[0] - 6, lo[1]), (lo[0] - 6, hi[1]), f"{f(D)} outer rim depth", rot=90)
    # column pitch
    for j in range(3):
        dim(ax, (xs[j], hi[1] + 1.2), (xs[j + 1], hi[1] + 1.2), f"{f(cpitch)}", fs=9)
    # side margins (tine base centre to outer rim)
    yl = (ys[1] + ys[2]) / 2
    dim(ax, (lo[0], yl), (xs[0], yl), f"side {f(side)}\n(tape {f(tape['side'])})", DERIVED, off=(0, 1.5), fs=9)
    dim(ax, (xs[-1], yl), (hi[0], yl), f"side {f(side)}\n(tape {f(tape['side'])})", DERIVED, off=(0, 1.5), fs=9)
    # front and rear margins beside column C2, text in the free strip between C2 and C3
    xm = xs[1] + 1.2
    dim(ax, (xm, lo[1]), (xm, ys[0]), "", DERIVED)
    note(ax, xm + .6, (lo[1] + ys[0]) / 2, f"front {f(front)} (tape {f(tape['front'])})", DERIVED, fs=9, va="center")
    dim(ax, (xm, ys[-1]), (xm, hi[1]), "", DERIVED)
    note(ax, xm + .6, (ys[-1] + hi[1]) / 2 + .2, f"rear {f(rear)} (tape {f(tape['rear'])})", DERIVED, fs=9, va="center")
    # tine pitch
    xp = xs[-1] + 3.2
    dim(ax, (xp, ys[4]), (xp, ys[5]), f"{f(pitch)}", off=(1.6, 0), fs=9.5)
    dim(ax, (xp + 5.2, ys[0]), (xp + 5.2, ys[-1]), f"{len(ys) - 1} x {f(pitch)} = {f(ys[-1] - ys[0])}",
        DERIVED, rot=90, off=(1.3, 0), fs=9.5)
    ax.text(0, lo[1] - 3.2, "FRONT", ha="center", color=INK, fontsize=11, fontweight="bold")
    ax.set_xlim(lo[0] - 15, hi[0] + 14)
    ax.set_ylim(lo[1] - 5, hi[1] + 13)

    # front section X-Z
    ax = fig.add_subplot(gs[0, 1])
    view(ax, "Front view, looking into the rack (cross-floor profile)")
    wires(ax, rack, (0, 2), keep=lambda n: n.startswith(("ContouredCrossU", "TopRim", "MidRim")) and "_Tooth" not in n)
    for _, b, t in ts:
        ax.plot([b[0], t[0]], [b[2], t[2]], color=TINE, lw=2.2, zorder=4)
    floor_lo = valley[1]
    ext(ax, [hi[0], hi[0] + 5], [z_top, z_top]); ext(ax, [hi[0], hi[0] + 5], [z_lo, z_lo])
    ext(ax, [lo[0] - 1, hi[0] + 5], [0, 0])
    dim(ax, (hi[0] + 4, z_lo), (hi[0] + 4, z_top), f"{f(z_top - z_lo)} outside", rot=90, off=(1.2, 0))
    ext(ax, [lo[0] - 5, lo[0]], [rim_z, rim_z])
    dim(ax, (lo[0] - 4, 0), (lo[0] - 4, rim_z), f"rim wire {f(rim_z)}", DERIVED, rot=90, off=(-1.1, 0), fs=9)
    b0 = [b for _, b, _ in ts if np.isclose(b[0], xs[0])][0]
    dim(ax, (xs[0] - 1.4, b0[2]), (xs[0] - 1.4, b0[2] + rise), f"tine {f(rise)}", EST, rot=90, off=(-.9, 0), fs=9)
    for j in range(3):
        dim(ax, (xs[j], rim_z + 2), (xs[j + 1], rim_z + 2), f"{f(cpitch)}", fs=9)
    note(ax, lo[0] + .8, .5, "z = 0 (rack datum)", fs=8.5)
    # channel profile (photo-fit estimates), right half, as generated
    hxc = hi[0] - p["rim_diameter"] / 2 * CM
    feats = [((prof["central_ridge_half_width"] / 2, -1.0), f"centre floor -1\n{f(2 * prof['central_ridge_half_width'])} wide (C2, C3)", (2.5, -6.6)),
             (valley, f"mug valley {f(valley[1])}\n{f(valley[0] - xs[-1])} outside C4", (10.2, -6.6)),
             (crest, f"low ridge +{f(crest[1])}\n{f(crest[0] - xs[-1])} outside C4", (18.4, 3.4)),
             (trough, f"glass trough {f(trough[1])}\n{f(hi[0] - trough[0])} inside the outer rim", (19.8, -6.6)),
             ((hxc - .3, 5.5), "mid rail 5.5", (19.6, 8.6))]
    for xy, text, at in feats:
        ax.annotate(text, xy=xy, xytext=at, ha="center", va="center", color=EST, fontsize=8.5, zorder=10,
                    arrowprops={"arrowstyle": "->", "color": EST, "lw": .7},
                    bbox={"facecolor": PAPER, "edgecolor": "none", "pad": 1.0})
    ax.set_xlim(lo[0] - 8, hi[0] + 8)
    ax.set_ylim(floor_lo - 7, rim_z + 4)

    # side view Y-Z, outer column C1
    ax = fig.add_subplot(gs[1, 1])
    view(ax, "Side view from the right (tines of column C4, leaning toward the rear)")
    wires(ax, rack, (1, 2), keep=lambda n: n.startswith(("LongitudinalCradle", "TopRim", "MidRim")))
    c1 = [(b, t) for _, b, t in ts if np.isclose(b[0], xs[-1])]
    for b, t in c1:
        ax.plot([b[1], t[1]], [b[2], t[2]], color=TINE, lw=2.2, zorder=4)
    b, t = c1[0]
    dim(ax, (b[1] - 1.2, b[2]), (b[1] - 1.2, t[2]), f"{f(rise)} rise", EST, rot=90, off=(-.9, 0), fs=9)
    ext(ax, [t[1], t[1]], [t[2], t[2] + 3]); ext(ax, [b[1], b[1]], [t[2], t[2] + 3])
    dim(ax, (b[1], t[2] + 2.4), (t[1], t[2] + 2.4), "", EST)
    note(ax, t[1] + .6, t[2] + 2.9, f"lean {f(lean)} to the rear, tine Ø{f(dia)}", fs=9)
    dim(ax, (ys[4], -3), (ys[5], -3), f"{f(pitch)}", fs=9)
    ext(ax, [hi[1], hi[1] + 4], [z_top, z_top]); ext(ax, [hi[1], hi[1] + 4], [z_lo, z_lo])
    dim(ax, (hi[1] + 3, z_lo), (hi[1] + 3, z_top), f"{f(z_top - z_lo)}", rot=90, off=(1, 0))
    dim(ax, (lo[1], -5.2), (hi[1], -5.2), f"{f(D)} depth")
    ax.text(lo[1], -7.4, "FRONT", ha="left", color=INK, fontsize=10, fontweight="bold")
    ax.text(hi[1], -7.4, "REAR", ha="right", color=INK, fontsize=10, fontweight="bold")
    ax.set_xlim(lo[1] - 4, hi[1] + 6)
    ax.set_ylim(-8.5, rim_z + 5)

    ax = fig.add_subplot(gs[2, :])
    over_x = 2 * tape["side"] + (len(xs) - 1) * cpitch - W
    over_y = tape["front"] + tape["rear"] + (len(ys) - 1) * pitch - D
    table(ax, [
        ("outer rim width", "48", f(W), "tape"),
        ("outer rim depth", "51.5", f(D), "tape"),
        ("height, outside", "12.5", f"{f(z_top - z_lo)} from the lowest floor wire to the rim top", "tape"),
        ("rim wire height", "-", f"{f(rim_z)} rim wire centre above the rack datum z = 0 (set so the outside is {f(z_top - z_lo)}; "
         f"the floor dips {f(-valley[1])} below z = 0)", "derived"),
        ("tine columns", "4, 9 apart", f"4, symmetric about the centre: x = {', '.join(f(x) for x in xs)}", "tape"),
        ("tines per column", "13 / 11 / 11 / 13", f"{' / '.join(map(str, counts))} = {sum(counts)}", "tape"),
        ("missing tines", "2 at the centre", f"positions {absent[0]} and {absent[-1]} of {len(ys)} (from the front) in C2 and C3", "tape"),
        ("tine pitch front-back", "3.7", f"{f(pitch)}", "tape"),
        ("side margin", "12", f"{f(side)} (3 x {f(cpitch)} = {f(3 * cpitch)} leaves {f(W - 3 * cpitch)} of the {f(W)}; "
         f"your margins exceed the rim by {f(over_x)})", "derived"),
        ("front / rear margin", "8 / 5.5", f"{f(front)} / {f(rear)} ({len(ys) - 1} x {f(pitch)} = {f(ys[-1] - ys[0])} leaves "
         f"{f(D - (len(ys) - 1) * pitch)} of the {f(D)}, split 8:5.5; your margins exceed it by {f(over_y)})", "derived"),
        ("tine", "-", f"{f(rise)} vertical rise, {f(lean)} rearward lean, Ø{f(dia)}", "estimate"),
        ("floor profile", "-", f"generated wire: centre -1, mug valley {f(valley[1])}, low ridge +{f(crest[1])}, glass trough "
         f"{f(trough[1])} (nominal corners {f(p['lowest_floor_center_z'] * CM)} / +{f(prof['ridge_z'])} before the bends)", "estimate"),
    ], "All values (cm; margins are tine base centre to the outer rim wire)")
    fig.savefig(out, dpi=110, facecolor=PAPER)
    plt.close(fig)


# ---------------------------------------------------------------- lower rack + basket

def lower(out):
    comps = G.build_components()
    rack, basket, p, bp = comps["LowerRack"], comps["SilverwareBasket"], P["lower_rack"], P["silverware_basket"]
    lo, hi = extent(rack, "UpperRim")
    W, D = hi[0] - lo[0], hi[1] - lo[1]
    ts = tines(rack, "TineBank")
    xs, ys = (np.asarray(v) * CM for v in G.lower_tine_positions())
    mask = G.lower_tine_mask()                     # [row, column]
    heights = G.lower_tine_heights() * CM
    px, py = p["tine_pitch"]["x"] * CM, p["tine_pitch"]["y"] * CM
    m = {k: v * CM for k, v in p["tine_margins"].items()}
    tape = {k: v * CM for k, v in p["tine_margins_tape_m"].items()}
    rim_z, lean, dia = p["rim_height"] * CM, p["tine_tip_offset_x"] * CM, p["tine_diameter"] * CM
    seat = np.asarray(bp["seat"]["origin_in_rack_m"])
    s = seat * CM
    L, Wb, Hb, Hh = (bp[k] * CM for k in ("length_y", "width_x", "body_height", "handle_top_z"))
    bx0, bx1, by0, by1 = s[0] - Wb / 2, s[0] + Wb / 2, s[1] - L / 2, s[1] + L / 2
    counts = mask.sum(1).tolist()
    removed = [(int(r), int(c)) for r, c in zip(*np.nonzero(~mask))]
    rm_rows = sorted({r + 1 for r, _ in removed}); rm_cols = sorted({c + 1 for _, c in removed})
    short = [r + 1 for r in p["tine_short_rows"]]
    hd = bp["handle"]
    base = [bb for _, bb, _ in ts][0][2]
    taper = bp["taper"]
    assert np.isclose(W, p["wire_width"] * CM) and np.isclose(D, p["wire_depth"] * CM), (W, D)
    assert len(ts) == sum(counts) and np.allclose(np.unique([b[1] for _, b, _ in ts]), ys), len(ts)
    blo, bhi = extent(basket, "TopRim")
    b_lo, b_top = z_span(basket, lambda n: True)
    r_lo = z_span(rack, lambda n: n.startswith(("FloorCrossU", "FloorLongU")))[0]
    r_top = z_span(rack, lambda n: n.startswith("UpperRim"))[1]
    assert abs(r_top - r_lo - p["outer_height_tape"] * CM) < .05 and abs(b_lo) < 1e-6, (r_top - r_lo, b_lo)
    assert np.allclose(bhi[:2] - blo[:2], (Wb, L)) and np.isclose(max(np.asarray(q)[:, 2].max() + r for n, q, r in basket["wires"]) * CM, Hh)

    fig = plt.figure(figsize=(22, 18.5), facecolor=PAPER)
    gs = fig.add_gridspec(3, 2, width_ratios=(1.05, 1), height_ratios=(1, .95, .86),
                          left=.03, right=.98, top=.905, bottom=.03, hspace=.14, wspace=.05)
    fig.text(.03, .968, "Lower rack and silverware basket, Frigidaire FDPC4221AS twin: every dimension in cm",
             fontsize=19, fontweight="bold", color=INK)
    fig.text(.03, .949, f"revisions {p['geometry_revision']} + {bp['geometry_revision']} · {sum(counts)} tines · "
             "wires are the generated source geometry, projected; front of the rack is -Y (bottom of the plan)",
             fontsize=11, color=INK)
    legend(fig, .933)

    # plan view
    ax = fig.add_subplot(gs[0:2, 0])
    view(ax, "Plan view (from above)")
    wires(ax, rack, (0, 1), keep=lambda n: "_Tooth" not in n and "Wheel" not in n)
    wires(ax, basket, (0, 1), shift=seat, keep=lambda n: n.startswith(("TopRim", "Partition", "Handle", "CornerPost")),
          color=BASKET, lw=1.3, z=3)
    for _, b, t in ts:
        ax.plot([b[0], t[0]], [b[1], t[1]], color=TINE, lw=2.4, zorder=4)
        ax.plot(b[0], b[1], "o", ms=4.5, mfc=TINE, mec="white", mew=.5, zorder=5)
    for r, c in removed:
        ax.plot(xs[c], ys[r], "o", ms=6.5, mfc="none", mec=TINE, mew=1.2, zorder=5)
    for r, y in enumerate(ys):
        ax.text(lo[0] - .8, y, f"R{r + 1}  {f(heights[r])} tall\n{counts[r]} tines", ha="right", va="center",
                color=TAPE, fontsize=9, fontweight="bold")
    for c, x in enumerate(xs):
        ax.text(x, hi[1] + .8, f"{c + 1}", ha="center", va="bottom", color=INK, fontsize=8)
    ax.text(xs[0] - 2.2, hi[1] + .8, "column", ha="right", va="bottom", color=INK, fontsize=8)
    ext(ax, [lo[0], lo[0]], [hi[1], hi[1] + 9]); ext(ax, [hi[0], hi[0]], [hi[1], hi[1] + 9])
    dim(ax, (lo[0], hi[1] + 8), (hi[0], hi[1] + 8), f"{f(W)} outer rim width")
    dim(ax, (xs[0], hi[1] + 4.6), (xs[-1], hi[1] + 4.6), f"{len(xs) - 1} x {f(px)} = {f(xs[-1] - xs[0])}", DERIVED, fs=9)
    dim(ax, (xs[4], hi[1] + 2.5), (xs[5], hi[1] + 2.5), f"{f(px)}", off=(0, 1.1), fs=9)
    ext(ax, [lo[0] - 14.5, lo[0]], [lo[1], lo[1]]); ext(ax, [lo[0] - 14.5, lo[0]], [hi[1], hi[1]])
    dim(ax, (lo[0] - 13.5, lo[1]), (lo[0] - 13.5, hi[1]), f"{f(D)} outer rim depth", rot=90)
    dim(ax, (xs[1] + 1.6, ys[0]), (xs[1] + 1.6, ys[1]), f"{f(py)}", off=(1.3, 0), fs=9.5)
    # margins
    yl = lo[1] + 1.5
    dim(ax, (lo[0], yl), (xs[0], yl), f"left {f(m['left'])}\n(tape {f(tape['left'])})", DERIVED, off=(1.4, 2.2), fs=9)
    dim(ax, (xs[-1], yl), (hi[0], yl), f"right {f(m['right'])}\n(tape {f(tape['right'])})", DERIVED, off=(-1.6, 2.2), fs=9)
    xm = xs[3] + 1.8
    dim(ax, (xm, lo[1]), (xm, ys[0]), f"front {f(m['front'])} (tape {f(tape['front'])})", DERIVED, off=(5.4, 0), fs=9)
    dim(ax, (xm, ys[-1]), (xm, hi[1]), f"rear {f(m['rear'])} (tape {f(tape['rear'])})", DERIVED, off=(5.2, 0), fs=9)
    # basket footprint and seat
    dim(ax, (bx0, by0 - 1.4), (bx1, by0 - 1.4), f"{f(Wb)}", fs=9.5)
    xb = (xs[-2] + xs[-1]) / 2                      # between the two omitted columns, clear of the circles
    dim(ax, (xb, by0), (xb, by1), f"basket {f(L)}", rot=90, off=(0, (ys[3] + ys[4]) / 2 - s[1]), fs=9.5)
    dim(ax, (bx1, by1 - 3), (hi[0], by1 - 3), f"{f(hi[0] - bx1)}", EST, off=(0, 1), fs=8.5)
    dim(ax, (bx1 - 1.5, by1), (bx1 - 1.5, hi[1]), f"{f(hi[1] - by1)}", EST, off=(1.3, 0), fs=8.5)
    ext(ax, [bx0 - 3, hi[0]], [by0, by0])
    note(ax, bx0 - 12.5, by0 - 3.4, f"basket front edge {f(ys[2] - by0)} ahead of row R3", fs=8.5)
    note(ax, xs[0], (ys[3] + ys[4]) / 2, f"{len(removed)} tines omitted under the basket:\ncolumns {rm_cols[0]}-{rm_cols[-1]}, "
         f"rows R{rm_rows[0]}-R{rm_rows[-1]} (open circles)", color=TAPE, fs=9, fontweight="bold", va="center")
    ax.text(0, lo[1] - 3, "FRONT", ha="center", color=INK, fontsize=11, fontweight="bold")
    ax.set_xlim(lo[0] - 18, hi[0] + 2.5)
    ax.set_ylim(lo[1] - 4.5, hi[1] + 11)

    # side view Y-Z with basket
    ax = fig.add_subplot(gs[0, 1])
    view(ax, "Side view from the right (tine rows and the basket in its seat)")
    wires(ax, rack, (1, 2), keep=lambda n: n.startswith(("FloorLongU", "UpperRim", "MidRail", "LowerReinforcement")))
    wires(ax, basket, (1, 2), shift=seat, keep=lambda n: not n.startswith(("BottomCrossRib", "LongWall-1", "EndWall")),
          color=BASKET, lw=.8, z=3)
    for r in range(len(ys)):
        b = [bb for _, bb, _ in ts if np.isclose(bb[1], ys[r])][0]
        ax.plot([ys[r], ys[r]], [b[2], b[2] + heights[r]], color=TINE, lw=3, zorder=4)
        ax.text(ys[r], b[2] + heights[r] + .5, f"R{r + 1}", ha="center", va="bottom", color=TAPE, fontsize=9, fontweight="bold",
                zorder=10, bbox={"facecolor": PAPER, "edgecolor": "none", "pad": .8})
    dim(ax, (ys[0] - 1.3, base), (ys[0] - 1.3, base + heights[0]), f"{f(heights[0])}", rot=90, off=(-.9, 0))
    dim(ax, (ys[2] - 1.3, base), (ys[2] - 1.3, base + heights[2]), f"{f(heights[2])}", rot=90, off=(-.9, 0))
    dim(ax, (ys[0], -2.6), (ys[1], -2.6), f"{f(py)}", fs=9)
    ext(ax, [lo[1] - 7.4, lo[1]], [r_lo, r_lo]); ext(ax, [lo[1] - 7.4, lo[1]], [r_top, r_top])
    dim(ax, (lo[1] - 6.6, r_lo), (lo[1] - 6.6, r_top), f"{f(r_top - r_lo)} outside", rot=90, off=(-1, 0))
    ext(ax, [lo[1] - 4, lo[1]], [rim_z, rim_z]); ext(ax, [lo[1] - 4, lo[1] + 1], [0, 0])
    dim(ax, (lo[1] - 3, 0), (lo[1] - 3, rim_z), f"rim wire {f(rim_z)}", DERIVED, rot=90, off=(-1, 0), fs=9)
    bz = s[2]
    ext(ax, [by1, hi[1] + 5], [bz + Hb, bz + Hb]); ext(ax, [by1 - 6, hi[1] + 5], [bz + Hh, bz + Hh])
    ext(ax, [by1, hi[1] + 5], [bz, bz])
    dim(ax, (hi[1] + 1.8, bz), (hi[1] + 1.8, bz + Hb), f"{f(Hb)} body", rot=90, off=(.9, 0))
    dim(ax, (hi[1] + 4.2, bz), (hi[1] + 4.2, bz + Hh), f"{f(Hh)} to handle top", rot=90, off=(.9, 0))
    dim(ax, (by0, bz + Hh + 1.6), (by1, bz + Hh + 1.6), f"basket {f(L)}")
    dim(ax, (lo[1], -5), (hi[1], -5), f"{f(D)} depth")
    ax.text(lo[1], -7.2, "FRONT", ha="left", color=INK, fontsize=10, fontweight="bold")
    ax.text(hi[1], -7.2, "REAR", ha="right", color=INK, fontsize=10, fontweight="bold")
    ax.set_xlim(lo[1] - 9.5, hi[1] + 7.5)
    ax.set_ylim(-8, bz + Hh + 4)

    # basket details (local frame, origin at the bottom-face centre)
    sub = gs[1, 1].subgridspec(1, 3, width_ratios=(2.3, .9, .9), wspace=.05)
    ax = fig.add_subplot(sub[0])
    view(ax, "Basket, long side (handle over the right wall)")
    wires(ax, basket, (1, 2), keep=lambda n: not n.startswith(("BottomCrossRib", "LongWall-1", "EndWall")), color=BASKET, lw=.8)
    parts = [-L / 4, 0, L / 4]
    for y in parts:
        ax.plot([y, y], [0, Hb], color=BASKET, lw=2, zorder=3)
    for i in range(4):
        dim(ax, (-L / 2 + i * L / 4, -1.6), (-L / 2 + (i + 1) * L / 4, -1.6), f"{f(L / 4)}", fs=9)
    dim(ax, (-L / 2, -4.2), (L / 2, -4.2), f"{f(L)} length (top rim)")
    ext(ax, [L / 2, L / 2 + 5.5], [Hb, Hb]); ext(ax, [L / 2 - 8, L / 2 + 5.5], [Hh, Hh]); ext(ax, [L / 2, L / 2 + 5.5], [0, 0])
    dim(ax, (L / 2 + 1.8, 0), (L / 2 + 1.8, Hb), f"{f(Hb)}", rot=90, off=(.8, 0))
    dim(ax, (L / 2 + 4.4, 0), (L / 2 + 4.4, Hh), f"{f(Hh)}", rot=90, off=(.8, 0))
    fy, ft, lh = hd["foot_y_abs"] * CM, hd["flat_top_half_span"] * CM, hd["loop_height"] * CM
    dim(ax, (-fy, Hh + 1.4), (fy, Hh + 1.4), f"legs {f(2 * fy)} apart", EST, fs=8.5)
    dim(ax, (-ft, Hh - 1.2), (ft, Hh - 1.2), f"flat top {f(2 * ft)}", EST, fs=8.5)
    note(ax, -L / 2, Hb + 2.2, "4 compartments\n(3 partitions)", color=TAPE, fs=9, fontweight="bold")
    note(ax, -L / 2, -6.6, f"heights from the lowest wire (z = {f(b_lo)}): outside heights", fs=8.5)
    ax.set_xlim(-L / 2 - 1, L / 2 + 6.5)
    ax.set_ylim(-7.4, Hh + 3.2)

    ax = fig.add_subplot(sub[1])
    view(ax, "End view")
    wires(ax, basket, (0, 2), keep=lambda n: not n.startswith(("BottomCrossRib", "BottomLongRib")), color=BASKET, lw=.8)
    fw = Wb * taper["floor_width_ratio"]
    dim(ax, (-Wb / 2, Hb + 1.2), (Wb / 2, Hb + 1.2), f"{f(Wb)} width", fs=9.5)
    dim(ax, (-fw / 2, -1.4), (fw / 2, -1.4), f"floor {f(fw)}", EST, fs=8.5)
    hx = (bp["width_x"] / 2 - bp["wire_radii"]["rim"] - hd["plane_inset_from_rim_x"]) * CM
    ax.annotate("handle over the\nright long wall", xy=(hx, Hh - 3), xytext=(-Wb / 2 - .3, Hh - 1), color=TAPE, fontsize=8.5,
                fontweight="bold", arrowprops={"arrowstyle": "->", "color": TAPE, "lw": .8})
    note(ax, -Wb / 2 - .3, -3.9, "rack's right wall →", color=TAPE, fs=8.5, fontweight="bold")
    ax.set_xlim(-Wb / 2 - 1.2, Wb / 2 + 1.2)
    ax.set_ylim(-4.6, Hh + 3.2)

    ax = fig.add_subplot(sub[2])
    view(ax, "Top view")
    wires(ax, basket, (0, 1), keep=lambda n: n.startswith(("TopRim", "Partition", "BottomLongRib")), color=BASKET, lw=.8)
    for i in range(4):
        ax.text(0, -L / 2 + (i + .5) * L / 4, f"{i + 1}", ha="center", va="center", color=TAPE, fontsize=11, fontweight="bold")
    dim(ax, (-Wb / 2, L / 2 + 1), (Wb / 2, L / 2 + 1), f"{f(Wb)}", fs=9.5)
    dim(ax, (Wb / 2 + 1, -L / 2), (Wb / 2 + 1, L / 2), f"{f(L)}", rot=90, off=(.8, 0), fs=9.5)
    ax.text(0, -L / 2 - 2, "FRONT", ha="center", color=INK, fontsize=9, fontweight="bold")
    ax.set_xlim(-Wb / 2 - 1, Wb / 2 + 2.6)
    ax.set_ylim(-L / 2 - 3, L / 2 + 2.6)

    ax = fig.add_subplot(gs[2, :])
    over_x = tape["left"] + tape["right"] + (len(xs) - 1) * px - W
    over_y = tape["front"] + tape["rear"] + (len(ys) - 1) * py - D
    table(ax, [
        ("outer rim width x depth", "52.5 x 56.3", f"{f(W)} x {f(D)}", "tape"),
        ("height, outside", "11.5", f"{f(r_top - r_lo)} from the floor wire underside to the rim top", "tape"),
        ("rim wire height", "-", f"{f(rim_z)} rim wire centre above the floor wire centre z = 0 (set so the outside is {f(r_top - r_lo)})", "derived"),
        ("tine grid", "12 columns x 6 rows", f"{len(xs)} x {len(ys)}; {' / '.join(map(str, counts))} per row = {sum(counts)} "
         f"({len(removed)} omitted under the basket)", "tape"),
        ("tine pitch left-right / front-back", "3.6 / 8", f"{f(px)} / {f(py)}", "tape"),
        ("tine height", "9.5; middle rows 4.5", f"rows R{short[0]}-R{short[-1]} {f(p['tine_short_height'] * CM)}, "
         f"the others {f(p['tine_height'] * CM)}", "tape"),
        ("left / right margin", "7.7 / 10.5", f"{f(m['left'])} / {f(m['right'])} ({len(xs) - 1} x {f(px)} = {f(xs[-1] - xs[0])} leaves "
         f"{f(W - (xs[-1] - xs[0]))} of the {f(W)}, split 7.7:10.5; your margins exceed it by {f(over_x)})", "derived"),
        ("front / rear margin", "10.5 / 12", f"{f(m['front'])} / {f(m['rear'])} ({len(ys) - 1} x {f(py)} = {f(ys[-1] - ys[0])} leaves "
         f"{f(D - (ys[-1] - ys[0]))} of the {f(D)}, split 10.5:12; your margins exceed it by {f(over_y)})", "derived"),
        ("tine lean, diameter, base", "-", f"{f(lean)} to the right, Ø{f(dia)}, base {f(base)} above the datum", "estimate"),
        ("basket top rim", "32 x 9.5", f"{f(L)} long x {f(Wb)} wide", "tape"),
        ("basket body / handle top", "13 / 22", f"{f(Hb - b_lo)} / {f(b_top - b_lo)} from the lowest wire (top rim, handle top)", "tape"),
        ("basket placement", "rear-right, handle to the right", "rear-right corner, length front to back; handle over the right long wall", "tape"),
        ("basket compartments", "4 in a row", f"4 x {f(L / 4)} (partitions at {f(-L / 4)}, 0, +{f(L / 4)} from the centre)", "tape"),
        ("basket floor, handle", "-", f"floor {f(fw)} x {f(L * taper['floor_length_ratio'])} (walls taper); handle legs {f(2 * fy)} "
         f"apart, flat top {f(2 * ft)}, loop {f(lh)} tall", "estimate"),
        ("basket seat", "-", f"centre {f(s[0])} right and {f(s[1])} rear of the rack centre; {f(hi[0] - bx1)} from the right rim, "
         f"{f(hi[1] - by1)} from the rear rim", "estimate"),
    ], "All values (cm; margins are tine base centre to the outer rim wire)")
    fig.savefig(out, dpi=110, facecolor=PAPER)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--out-dir", type=Path, default=ROOT / "media/frigidaire_rack_dimensions")
    args = ap.parse_args()
    args.out_dir.mkdir(parents=True, exist_ok=True)
    for name, draw in (("frigidaire_upper_rack_cm.png", upper), ("frigidaire_lower_rack_cm.png", lower)):
        draw(args.out_dir / name)
        print(f"[INFO] wrote {args.out_dir / name}")
    print("[RESULT] PASS")


if __name__ == "__main__":
    main()
