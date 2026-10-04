#!/usr/bin/env python3
# Copyright (c) 2026, dishsim project.
# SPDX-License-Identifier: BSD-3-Clause
"""The user's tape numbers drawn exactly as written, with every conflict and every guess marked (Kit-free).

Nothing here comes from the twin. Each rack is laid out from TAPE alone, starting at the left and
front margins, so a margin that cannot hold shows up at the right or back rim. Teal = your number,
red = your numbers contradict each other, orange = my reading of something the numbers leave open.
Correct TAPE below and rerun:

    python3 code/initialization/frigidaire/frigidaire_tape_check.py [--out-dir data/media/frigidaire_rack_dimensions]
"""
import argparse
import importlib.util
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import numpy as np

_spec = importlib.util.spec_from_file_location("dims", Path(__file__).resolve().parent / "frigidaire_rack_dimensions_cm.py")
D = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(D)                                   # reuse the cm drawing helpers (dim, ext, note, view, f)
f, dim, ext, note = D.f, D.dim, D.ext, D.note
RED, ORANGE, TEAL, INK, PAPER = "#c62828", "#c0551a", D.TAPE, D.INK, D.PAPER

# The user's recorded numbers, cm, as written on 2026-09-23.
TAPE = {
    "upper": {"width": 48, "depth": 51.5, "height": 12.5, "columns": 4, "tines_per_column": [13, 11, 11, 13],
              "margin_left": 12, "margin_right": 12, "margin_front": 8, "margin_back": 5.5,
              "column_pitch": 9, "row_pitch": 3.7},
    "lower": {"width": 52.5, "depth": 56.3, "height": 11.5, "columns": 12, "rows": 6,
              "tine_height": 9.5, "short_tine_height": 4.5,
              "margin_left": 7.7, "margin_right": 10.5, "margin_front": 10.5, "margin_back": 12,
              "column_pitch": 3.6, "row_pitch": 8},
}
BASKET = {"length": 32, "width": 9.5, "right_gap": 3.1, "back_gap": 2.55}   # 2026-09-22 tape size; gaps as in the twin


def mark(ax, x, y, n, color):
    ax.text(x, y, str(n), ha="center", va="center", color="white", fontsize=10, fontweight="bold", zorder=12,
            bbox={"boxstyle": "circle,pad=0.3", "facecolor": color, "edgecolor": "none"})


def issues(fig, rows, top):
    fig.text(.03, top, "Please check (the numbers match the circles on the drawing)", fontsize=13, fontweight="bold", color=INK)
    y = top - .03
    for n, color, text in rows:
        fig.text(.035, y, f" {n} ", fontsize=11, fontweight="bold", color="white",
                 bbox={"boxstyle": "circle,pad=0.25", "facecolor": color, "edgecolor": "none"})
        fig.text(.06, y, text, fontsize=11, color=INK, va="baseline")
        y -= .032


def header(fig, title, sub):
    fig.text(.03, .965, title, fontsize=19, fontweight="bold", color=INK)
    fig.text(.03, .943, sub, fontsize=11, color=INK)
    for i, (label, c) in enumerate((("teal = your number", TEAL), ("red = your numbers contradict each other", RED),
                                    ("orange = my reading, please confirm", ORANGE))):
        fig.text(.04 + (0, .2, .5)[i], .922, "■ " + label, color=c, fontsize=11, fontweight="bold")


def plan_grid(ax, xs, ys, W, Dp, absent=lambda i, j: False, short=lambda i: False):
    ax.add_patch(Rectangle((0, 0), W, Dp, fill=False, ec=INK, lw=2.4, zorder=3))
    for j, x in enumerate(xs):
        for i, y in enumerate(ys):
            outside = x > W or y > Dp
            c = RED if outside else (ORANGE if short(i) else TEAL)
            if absent(i, j):
                ax.plot(x, y, "o", ms=8, mfc="none", mec=ORANGE, mew=1.6, zorder=5)
            else:
                ax.plot(x, y, "o", ms=6.5, mfc=c, mec="white", mew=.6, zorder=5)


def upper(out):
    t = TAPE["upper"]
    W, Dp, H = t["width"], t["depth"], t["height"]
    n = max(t["tines_per_column"])
    xs = t["margin_left"] + t["column_pitch"] * np.arange(t["columns"])
    ys = t["margin_front"] + t["row_pitch"] * np.arange(n)
    right_left, back_left = W - xs[-1], Dp - ys[-1]
    missing = n - min(t["tines_per_column"])                  # 2
    gap = [n // 2 - 1, n // 2]                                # 0-based 5, 6 = positions 6-7 (13 has no centred pair)
    absent = lambda i, j: j in (1, 2) and i in gap

    fig = plt.figure(figsize=(20, 14.5), facecolor=PAPER)
    gs = fig.add_gridspec(2, 2, width_ratios=(1.3, 1), height_ratios=(1, .42), left=.03, right=.98, top=.885,
                          bottom=.02, hspace=.08, wspace=.06)
    header(fig, "Upper rack drawn from your numbers only", "laid out from the left and the front margin with your spacing; "
           "cm; the dark rectangle is the 48 x 51.5 rim; front at the bottom")
    ax = fig.add_subplot(gs[0, 0])
    D.view(ax, "Plan view (from above)")
    plan_grid(ax, xs, ys, W, Dp, absent)
    for j, x in enumerate(xs):
        ax.text(x, -4.3, f"C{j + 1}\n{t['tines_per_column'][j]}", ha="center", va="top", color=TEAL, fontsize=10, fontweight="bold")
    for i, y in enumerate(ys):
        ax.text(-1.2, y, f"{i + 1}", ha="right", va="center", color=INK, fontsize=8)
    dim(ax, (0, Dp + 4), (W, Dp + 4), f"{f(W)} rim width")
    dim(ax, (-5, 0), (-5, Dp), f"{f(Dp)} rim depth", rot=90)
    dim(ax, (0, ys[1] + 1.85), (xs[0], ys[1] + 1.85), f"{f(t['margin_left'])}", fs=9)
    mark(ax, xs[0] / 2, ys[1] - .3, 5, ORANGE)
    for j in range(len(xs) - 1):
        dim(ax, (xs[j], ys[3] + 1.85), (xs[j + 1], ys[3] + 1.85), f"{f(t['column_pitch'])}", fs=9)
    dim(ax, (xs[0] - 2.2, 0), (xs[0] - 2.2, ys[0]), f"{f(t['margin_front'])}", off=(-1.2, 0), fs=9)
    dim(ax, (xs[0] - 2.2, ys[4]), (xs[0] - 2.2, ys[5]), f"{f(t['row_pitch'])}", off=(-1.3, 0), fs=9)
    # conflicts
    dim(ax, (xs[-1], ys[1] + 1.85), (W, ys[1] + 1.85), f"only {f(right_left)}", RED, off=(0, 1.3), fs=9.5)
    note(ax, xs[-1] + .3, ys[1] - 2.2, f"you: {f(t['margin_right'])}", color=RED, fs=9.5, fontweight="bold")
    mark(ax, W + 2.2, ys[1] + 1.85, 1, RED)
    if back_left < 0:
        ax.axhline(Dp, xmin=0, xmax=1, color=RED, lw=.8, ls=(0, (4, 3)), zorder=2)
        note(ax, xs[1] + 1, ys[-1] + .9, f"position {n} lands {f(-back_left)} behind the rim; you: {f(t['margin_back'])} of margin",
             color=RED, fs=9.5, fontweight="bold")
    else:
        dim(ax, (xs[1] + 1.5, ys[-1]), (xs[1] + 1.5, Dp), f"only {f(back_left)}", RED, off=(2.5, 0), fs=9.5)
    mark(ax, W + 2.2, ys[-1], 2, RED)
    note(ax, (xs[1] + xs[2]) / 2, (ys[gap[0]] + ys[gap[1]]) / 2, f"{missing} missing:\npositions {gap[0] + 1}-{gap[1] + 1}?",
         color=ORANGE, fs=9.5, fontweight="bold", ha="center", va="center")
    mark(ax, xs[2] + 2.4, ys[gap[1]] + 1.4, 3, ORANGE)
    ax.text(W / 2, -9.6, "FRONT", ha="center", color=INK, fontsize=11, fontweight="bold")
    ax.set_xlim(-11, W + 5)
    ax.set_ylim(-11, max(Dp, ys[-1]) + 7)

    # side view, schematic
    ax = fig.add_subplot(gs[0, 1])
    D.view(ax, "Side view from the right (schematic)")
    ax.plot([0, Dp, Dp, 0, 0], [0, 0, H, H, 0], color=INK, lw=2.2)
    for y in ys:
        ax.plot([y, y], [0, 9.1], color=ORANGE, lw=2, ls=(0, (3, 2)), zorder=4, clip_on=False)
    dim(ax, (Dp + 2.5, 0), (Dp + 2.5, H), f"{f(H)} outside", rot=90, off=(1, 0))
    note(ax, ys[0], 9.9, "tine height ?", color=ORANGE, fs=10, fontweight="bold")
    mark(ax, ys[0] + 9.5, 10.5, 4, ORANGE)
    dim(ax, (0, -2.4), (Dp, -2.4), f"{f(Dp)} depth")
    ax.text(0, -4.6, "FRONT", ha="left", color=INK, fontsize=10, fontweight="bold")
    ax.text(Dp, -4.6, "BACK", ha="right", color=INK, fontsize=10, fontweight="bold")
    ax.set_xlim(-3, Dp + 6)
    ax.set_ylim(-6, H + 4)

    issues(fig, [
        (1, RED, f"Across: {f(t['margin_left'])} + 3 x {f(t['column_pitch'])} + {f(t['margin_right'])} = "
                 f"{f(t['margin_left'] + 3 * t['column_pitch'] + t['margin_right'])}, but the rack is {f(W)} wide. From your left margin only "
                 f"{f(right_left)} is left on the right. Which is off: a side margin, the {f(t['column_pitch'])} spacing, or the {f(W)} width?"),
        (2, RED, f"Front to back: {f(t['margin_front'])} + {n - 1} x {f(t['row_pitch'])} + {f(t['margin_back'])} = "
                 f"{f(t['margin_front'] + (n - 1) * t['row_pitch'] + t['margin_back'])}, but the rack is {f(Dp)} deep. From your front margin the "
                 f"{n}th tine is {f(-back_left)} past the back rim. Which is off: a margin, the {f(t['row_pitch'])} spacing, or the {f(Dp)} depth?"),
        (3, ORANGE, f"{n} positions have no exact centre pair, so I left out positions {gap[0] + 1} and {gap[1] + 1} (counting from the front) "
                    f"in C2 and C3. Is it {gap[0] + 1}-{gap[1] + 1} or {gap[1] + 1}-{gap[1] + 2}?"),
        (4, ORANGE, "The upper tine height was not measured (the twin guesses 9.1). How tall are they, and do they lean?"),
        (5, ORANGE, "I read each margin as outer edge of the rim wire to the centre of the nearest tine. Is that how you measured?"),
    ], .27)
    fig.savefig(out, dpi=110, facecolor=PAPER)
    plt.close(fig)


def lower(out):
    t = TAPE["lower"]
    W, Dp, H = t["width"], t["depth"], t["height"]
    xs = t["margin_left"] + t["column_pitch"] * np.arange(t["columns"])
    ys = t["margin_front"] + t["row_pitch"] * np.arange(t["rows"])
    right_left, back_left = W - xs[-1], Dp - ys[-1]
    mid = [t["rows"] // 2 - 1, t["rows"] // 2]               # rows 3-4
    short = lambda i: i in mid
    bx1, by1 = W - BASKET["right_gap"], Dp - BASKET["back_gap"]
    bx0, by0 = bx1 - BASKET["width"], by1 - BASKET["length"]
    under = [(i, j) for j, x in enumerate(xs) for i, y in enumerate(ys) if bx0 <= x <= bx1 and by0 <= y <= by1]

    fig = plt.figure(figsize=(20, 15), facecolor=PAPER)
    gs = fig.add_gridspec(2, 2, width_ratios=(1.3, 1), height_ratios=(1, .42), left=.03, right=.98, top=.885,
                          bottom=.02, hspace=.08, wspace=.06)
    header(fig, "Lower rack drawn from your numbers only", "laid out from the left and the front margin with your spacing; "
           "cm; the dark rectangle is the 52.5 x 56.3 rim; front at the bottom")
    ax = fig.add_subplot(gs[0, 0])
    D.view(ax, "Plan view (from above)")
    ax.add_patch(Rectangle((bx0, by0), BASKET["width"], BASKET["length"], fill=False, ec=D.BASKET, lw=1.6, ls=(0, (5, 3)), zorder=4))
    ax.text((bx0 + bx1) / 2, by1 - 1.2, "basket", ha="center", va="top", color=D.BASKET, fontsize=9, fontweight="bold")
    plan_grid(ax, xs, ys, W, Dp, short=short)
    for j, x in enumerate(xs):
        ax.text(x, -1.2, f"{j + 1}", ha="center", va="top", color=INK, fontsize=8)
    for i, y in enumerate(ys):
        ax.text(-1.2, y, f"R{i + 1}  {f(t['short_tine_height'] if short(i) else t['tine_height'])} tall", ha="right", va="center",
                color=ORANGE if short(i) else TEAL, fontsize=9, fontweight="bold")
    dim(ax, (0, Dp + 4), (W, Dp + 4), f"{f(W)} rim width")
    dim(ax, (-12, 0), (-12, Dp), f"{f(Dp)} rim depth", rot=90)
    dim(ax, (0, ys[0] + 3.2), (xs[0], ys[0] + 3.2), f"{f(t['margin_left'])}", off=(0, 1.2), fs=9)
    mark(ax, xs[0] / 2, ys[0] + 1.3, 5, ORANGE)
    dim(ax, (xs[3], ys[0] + 3.2), (xs[4], ys[0] + 3.2), f"{f(t['column_pitch'])}", off=(0, 1.2), fs=9)
    dim(ax, (xs[1] + 1.8, 0), (xs[1] + 1.8, ys[0]), f"{f(t['margin_front'])}", off=(1.3, 0), fs=9)
    dim(ax, (xs[1] + 1.8, ys[0]), (xs[1] + 1.8, ys[1]), f"{f(t['row_pitch'])}", off=(1.1, 0), fs=9)
    # conflicts
    dim(ax, (xs[-1], ys[0] + 3.2), (W, ys[0] + 3.2), f"only {f(right_left)}", RED, off=(0, 1.3), fs=9.5)
    note(ax, xs[-1] - 3.2, ys[0] + 5.6, f"you: {f(t['margin_right'])}", color=RED, fs=9.5, fontweight="bold")
    mark(ax, W + 2.2, ys[0] + 3.2, 1, RED)
    xb = xs[5] + 1.8
    dim(ax, (xb, ys[-1]), (xb, Dp), f"only {f(back_left)}", RED, off=(2.6, 0), fs=9.5)
    note(ax, xb + .6, ys[-1] - 2.2, f"you: {f(t['margin_back'])}", color=RED, fs=9.5, fontweight="bold")
    mark(ax, xb - 1.6, (ys[-1] + Dp) / 2, 2, RED)
    mark(ax, -10, (ys[mid[0]] + ys[mid[1]]) / 2, 3, ORANGE)
    mark(ax, bx0 - 1.6, by0 + 3, 4, ORANGE)
    ax.text(W / 2, -4.4, "FRONT", ha="center", color=INK, fontsize=11, fontweight="bold")
    ax.set_xlim(-16, W + 5)
    ax.set_ylim(-6, Dp + 7)

    ax = fig.add_subplot(gs[0, 1])
    D.view(ax, "Side view from the right (schematic)")
    ax.plot([0, Dp, Dp, 0, 0], [0, 0, H, H, 0], color=INK, lw=2.2)
    for i, y in enumerate(ys):
        h = t["short_tine_height"] if short(i) else t["tine_height"]
        ax.plot([y, y], [0, h], color=ORANGE if short(i) else TEAL, lw=3, zorder=4)
        ax.text(y, h + .4, f"R{i + 1}", ha="center", va="bottom", color=ORANGE if short(i) else TEAL, fontsize=9, fontweight="bold")
    dim(ax, (ys[0] - 1.4, 0), (ys[0] - 1.4, t["tine_height"]), f"{f(t['tine_height'])}", rot=90, off=(-.9, 0))
    dim(ax, (ys[mid[0]] - 1.4, 0), (ys[mid[0]] - 1.4, t["short_tine_height"]), f"{f(t['short_tine_height'])}", ORANGE, rot=90, off=(-.9, 0))
    dim(ax, (Dp + 2.5, 0), (Dp + 2.5, H), f"{f(H)} outside", rot=90, off=(1, 0))
    dim(ax, (ys[0], -2.4), (ys[1], -2.4), f"{f(t['row_pitch'])}", fs=9)
    dim(ax, (0, -4.6), (Dp, -4.6), f"{f(Dp)} depth")
    ax.text(0, -6.8, "FRONT", ha="left", color=INK, fontsize=10, fontweight="bold")
    ax.text(Dp, -6.8, "BACK", ha="right", color=INK, fontsize=10, fontweight="bold")
    ax.set_xlim(-3, Dp + 6)
    ax.set_ylim(-8, H + 3)

    cols = sorted({j + 1 for _, j in under}); rows_u = sorted({i + 1 for i, _ in under})
    issues(fig, [
        (1, RED, f"Across: {f(t['margin_left'])} + 11 x {f(t['column_pitch'])} + {f(t['margin_right'])} = "
                 f"{f(t['margin_left'] + 11 * t['column_pitch'] + t['margin_right'])}, but the rack is {f(W)} wide. From your left margin only "
                 f"{f(right_left)} is left on the right. Which is off: a side margin, the {f(t['column_pitch'])} spacing, or the {f(W)} width?"),
        (2, RED, f"Front to back: {f(t['margin_front'])} + 5 x {f(t['row_pitch'])} + {f(t['margin_back'])} = "
                 f"{f(t['margin_front'] + 5 * t['row_pitch'] + t['margin_back'])}, but the rack is {f(Dp)} deep. From your front margin only "
                 f"{f(back_left)} is left at the back. Which is off: a margin, the {f(t['row_pitch'])} spacing, or the {f(Dp)} depth?"),
        (3, ORANGE, "\"2 middle tine 4.5\": I read it as the two middle ROWS (R3 and R4, counting from the front). Or is it two middle columns?"),
        (4, ORANGE, f"12 x 6 = 72 tines, but the basket (32 x 9.5, rear right) stands in the corner: here it would cover columns "
                    f"{cols[0]}-{cols[-1]} of rows R{rows_u[0]}-R{rows_u[-1]}. Are there tines under the basket, and which ones?"),
        (5, ORANGE, "I read each margin as outer edge of the rim wire to the centre of the nearest tine. Is that how you measured?"),
    ], .27)
    fig.savefig(out, dpi=110, facecolor=PAPER)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--out-dir", type=Path, default=D.ROOT / "data/media/frigidaire_rack_dimensions")
    args = ap.parse_args()
    args.out_dir.mkdir(parents=True, exist_ok=True)
    for name, draw in (("tape_check_upper_rack.png", upper), ("tape_check_lower_rack.png", lower)):
        draw(args.out_dir / name)
        print(f"[INFO] wrote {args.out_dir / name}")
    print("[RESULT] PASS")


if __name__ == "__main__":
    main()
