"""Toy worked example of Step 2-3 (lines and dot score), drawn to scale in a side view.

    python3 docs/scoring_note/toy/make_toy.py        (host: numpy + matplotlib)
Writes toy_example.pdf (3 pages) and toy_example_p1..3.png next to this file. Not part of the note.
2D (x, z): a mouth-down bowl (inside = upper half of a circle, radius 0.07 m, mouth at z = 0.30) above a disc
at z = 0 with 5 spray points; one neighboring plate stands at x = 0.12 (z 0.05..0.35) as a blocker.
Same formulas as the note: s = p + eps n, d = (q - s)/l, gamma = max(0, n.d), v = 1 if the segment s + t d,
0 < t < l, touches no blocker, e = sum(gamma v) / sum(gamma) (equal weights u_k cancel).
"""
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.backends.backend_pdf import PdfPages  # noqa: E402
from matplotlib.patches import Wedge, Arc  # noqa: E402

HERE = Path(__file__).resolve().parent
plt.rcParams.update({"font.family": "serif", "font.serif": ["DejaVu Serif"], "mathtext.fontset": "cm",
                     "font.size": 9, "axes.linewidth": .6})
BLUE, RED, GRAY, INK, TAN, ORANGE = "#2a78d6", "#c0392b", "#9aa5ae", "#13212B", "#c9a36a", "#f0b27a"

R, C, MOUTH, EPS = .07, np.array([0., .30]), .30, 1e-4
PLATE_X, PLATE_Z = .12, (.05, .35)
QX = (-.25, -.10, 0., .10, .25)
Q = [np.array([x, 0.]) for x in QX]


def on_circle(deg):
    a = np.radians(deg)
    return C + R * np.array([-np.cos(a), np.sin(a)])


DOTS = {"A": np.array([0., .37]), "B": on_circle(45), "D": on_circle(25)}


def trace(p):
    """All 5 lines of one dot: dicts with d, l, gamma, status, hit point."""
    n = (C - p) / np.linalg.norm(C - p)
    s = p + EPS * n
    out = []
    for q in Q:
        v = q - s
        l = float(np.linalg.norm(v))
        d = v / l
        g = max(0., float(n @ d))
        rec = {"q": q, "d": d, "l": l, "gamma": g, "status": "open", "hit": None, "t_hit": None}
        if g == 0:
            rec["status"] = "behind"
        else:
            # the line runs inside the bowl's circle; where it leaves the circle decides mouth vs own wall
            b = float(d @ (s - C)); c = float((s - C) @ (s - C)) - R * R
            t_exit = -b + np.sqrt(b * b - c)
            x_exit = s + t_exit * d
            if x_exit[1] > MOUTH:                      # leaves through the wall above the mouth: own rim blocks it
                rec.update(status="blocked by own wall", hit=x_exit, t_hit=t_exit)
            elif d[0] != 0:
                t_p = (PLATE_X - s[0]) / d[0]
                z_p = (s + t_p * d)[1]
                if 0 < t_p < l and PLATE_Z[0] <= z_p <= PLATE_Z[1]:
                    rec.update(status="blocked by plate", hit=s + t_p * d, t_hit=t_p)
        out.append(rec)
    num = sum(r["gamma"] for r in out if r["status"] == "open")
    den = sum(r["gamma"] for r in out)
    return n, s, out, num, den


def scene(ax, show_dots=True, alpha=1.):
    ax.plot([-.30, .30], [0, 0], color=BLUE, lw=3, solid_capstyle="round", zorder=1)
    for q in Q:
        ax.plot(*q, "o", color=BLUE, ms=5, zorder=3)
    ax.add_patch(Wedge(tuple(C), R + .006, 0, 180, width=.006, color=TAN, zorder=2, alpha=alpha))
    ax.plot([PLATE_X, PLATE_X], PLATE_Z, color=ORANGE, lw=4, solid_capstyle="butt", zorder=2, alpha=alpha)
    if show_dots:
        for name, p in DOTS.items():
            ax.plot(*p, "o", color=INK, ms=4, zorder=5)
    ax.set_aspect("equal")
    ax.set_xlim(-.32, .32); ax.set_ylim(-.045, .43)
    for sp in ("top", "right"):
        ax.spines[sp].set_visible(False)
    ax.tick_params(labelsize=7, width=.5, length=2)


def page1(pdf):
    fig, ax = plt.subplots(figsize=(8.3, 6.2))
    scene(ax)
    for k, q in enumerate(Q):
        ax.annotate(f"$\\mathbf{{q}}_{k}$\n({q[0]:+.2f}, 0)", q, xytext=(0, -26), textcoords="offset points",
                    ha="center", fontsize=8)
    ax.text(-.30, .012, "disc $\\mathcal{D}_a$ (edge-on), 5 spray points $\\mathbf{q}_0 \\ldots \\mathbf{q}_4$", fontsize=8, color=BLUE)
    ax.annotate("bowl, mouth down\n(inside = half circle, radius 0.07,\nmouth at $z$ = 0.30)", (-.07, .30),
                xytext=(-.30, .36), fontsize=8, arrowprops=dict(arrowstyle="-", lw=.5, color=INK))
    ax.annotate("neighboring plate = a blocker\n($x$ = 0.12, $z$ from 0.05 to 0.35)", (PLATE_X, .30), xytext=(.15, .38),
                fontsize=8, arrowprops=dict(arrowstyle="-", lw=.5, color=INK))
    for name, p in DOTS.items():
        n = (C - p) / np.linalg.norm(C - p)
        ax.annotate("", xy=p + .045 * n, xytext=p, arrowprops=dict(arrowstyle="-|>", lw=1, color=INK))
        off = {"A": (6, 6), "B": (-20, 6), "D": (-22, -2)}[name]
        ax.annotate(name, p, xytext=off, textcoords="offset points", fontsize=10, weight="bold")
    txt = ("Three dots on the bowl's inside, each with its unit normal $\\mathbf{n}$ (arrow, into the cavity):\n"
           + "\n".join(f"  {k}: $\\mathbf{{p}}$ = ({p[0]:+.4f}, {p[1]:.4f}),  $\\mathbf{{n}}$ = ({v[0]:+.3f}, {v[1]:+.3f})"
                       for k, p in DOTS.items() for v in [(C - p) / np.linalg.norm(C - p)]))
    fig.text(.08, .06, txt, fontsize=8.5, va="bottom", linespacing=1.5)
    ax.set_title("Toy setup (side view, meters, to scale)", fontsize=11)
    fig.subplots_adjust(left=.08, right=.97, top=.93, bottom=.27)
    pdf.savefig(fig); fig.savefig(HERE / "toy_example_p1.png", dpi=150)
    plt.close(fig)


def page2(pdf):
    p = DOTS["A"]
    n, s, lines, num, den = trace(p)
    r = lines[4]
    d, l = r["d"], r["l"]
    fig, ax = plt.subplots(figsize=(8.3, 6.2))
    scene(ax, alpha=.9)
    for rr in lines[:4]:
        ax.plot([s[0], rr["q"][0]], [s[1], rr["q"][1]], color=GRAY, lw=.6, alpha=.5, zorder=1)
    ax.plot([s[0], r["hit"][0]], [s[1], r["hit"][1]], color=RED, lw=1.6, zorder=4)
    ax.plot([r["hit"][0], r["q"][0]], [r["hit"][1], r["q"][1]], color=RED, lw=1, ls=(0, (3, 3)), alpha=.6, zorder=4)
    ax.plot(*r["hit"], marker="X", color=RED, ms=9, zorder=6)
    ax.annotate("", xy=p + .06 * n, xytext=p, arrowprops=dict(arrowstyle="-|>", lw=1.2, color=INK))
    ax.text(p[0] - .012, p[1] + .045 * n[1] - .004, "$\\mathbf{n}$", fontsize=10, ha="right")
    ax.annotate("", xy=s + .06 * d, xytext=s, arrowprops=dict(arrowstyle="-|>", lw=2, color=BLUE), zorder=8)
    ax.text(*(s + .06 * d + np.array([.008, .006])), "$\\mathbf{d}$", fontsize=10, color=BLUE, zorder=8)
    ax.text(-.31, .36, "$\\mathbf{n}$ and $\\mathbf{d}$ are unit vectors,\nboth drawn 6 cm long", fontsize=8)
    ax.add_patch(Arc(tuple(s), .09, .09, theta1=-90, theta2=np.degrees(np.arctan2(d[1], d[0])), lw=.8, color=INK, zorder=7))
    ax.text(s[0] + .014, s[1] - .066, "$\\theta$", fontsize=10, zorder=7)
    for t in (.1, .2):
        pt = s + t * d
        ax.plot(*pt, "o", color=INK, ms=3.5, zorder=6)
        ax.annotate(f"$t$ = {t:.1f}", pt, xytext=(8, 2), textcoords="offset points", fontsize=8)
    ax.annotate(f"$t$ = {r['t_hit']:.3f}: on the plate  $\\Rightarrow$  $v$ = 0", r["hit"], xytext=(10, -2),
                textcoords="offset points", fontsize=8.5, color=RED)
    ax.annotate(f"$t$ = $\\ell$ = {l:.3f}: $\\mathbf{{q}}_4$", r["q"], xytext=(-8, 12), textcoords="offset points",
                fontsize=8.5, ha="right")
    ax.annotate("A: $\\mathbf{p}$ = (0, 0.3700)\n$\\mathbf{s}$ = (0, 0.3699), 0.1 mm below: too small to see",
                p, xytext=(-.31, .40), fontsize=8)
    steps = [r"1. $\mathbf{n}$ = (0, $-1$): the top of a mouth-down bowl faces straight down",
             r"2. $\mathbf{p}$ = (0, 0.3700)",
             r"3. $\mathbf{s} = \mathbf{p} + \epsilon\,\mathbf{n}$ = (0, 0.3700) + 0.0001$\cdot$(0, $-1$) = (0, 0.3699)",
             rf"4. $\mathbf{{q}}_4 - \mathbf{{s}}$ = (0.25, $-$0.3699),  $\ell$ = $\sqrt{{0.25^2 + 0.3699^2}}$ = {l:.3f} m",
             rf"5. $\mathbf{{d}} = (\mathbf{{q}}_4-\mathbf{{s}})/\ell$ = ({d[0]:.3f}, {d[1]:.3f})   (length 1)",
             rf"6. walk $\mathbf{{s}} + t\,\mathbf{{d}}$: $t$ = 0 is $\mathbf{{s}}$, $t$ = {l:.3f} is $\mathbf{{q}}_4$; at $t$ = {r['t_hit']:.3f} the point is ({r['hit'][0]:.3f}, {r['hit'][1]:.3f}), on the plate",
             r"7. so the set {points of the line} $\cap$ {blockers} is not empty $\Rightarrow$ $v$ = 0 (blocked)",
             rf"8. $\gamma = \max(0, \mathbf{{n}}^\top\mathbf{{d}})$ = 0$\cdot${d[0]:.3f} + ($-$1)($-${-d[1]:.3f}) = {r['gamma']:.3f} = $\cos\theta$"]
    fig.text(.08, .035, "\n".join(steps), fontsize=8.5, va="bottom", linespacing=1.55)
    ax.set_title("One line, step by step: dot A $\\rightarrow$ spray point $\\mathbf{q}_4$", fontsize=11)
    fig.subplots_adjust(left=.08, right=.97, top=.93, bottom=.36)
    pdf.savefig(fig); fig.savefig(HERE / "toy_example_p2.png", dpi=150)
    plt.close(fig)


def page3(pdf):
    fig, axes = plt.subplots(1, 3, figsize=(11.7, 5.6))
    es = {}
    for ax, (name, p) in zip(axes, DOTS.items()):
        n, s, lines, num, den = trace(p)
        es[name] = num / den
        scene(ax, show_dots=False, alpha=.85)
        ax.plot(*p, "o", color=INK, ms=5, zorder=7)
        ax.annotate("", xy=p + .045 * n, xytext=p, arrowprops=dict(arrowstyle="-|>", lw=1, color=INK))
        for k, r in enumerate(lines, 1):
            q = r["q"]
            if r["status"] == "open":
                ax.plot([s[0], q[0]], [s[1], q[1]], color=BLUE, lw=1.3, zorder=4)
            elif r["status"] == "behind":
                ax.plot([s[0], q[0]], [s[1], q[1]], color=GRAY, lw=1, ls=(0, (1, 2)), zorder=4)
            else:
                ax.plot([s[0], r["hit"][0]], [s[1], r["hit"][1]], color=RED, lw=1.3, ls=(0, (4, 2)), zorder=4)
                ax.plot(*r["hit"], marker="X", color=RED, ms=7, zorder=6)
            lab = "behind" if r["status"] == "behind" else f"{r['gamma']:.3f}"
            ax.annotate(lab, q, xytext=(0, -13), textcoords="offset points", ha="center", fontsize=7.5,
                        color={"open": BLUE, "behind": "#6f7d87"}.get(r["status"], RED))
        ax.set_title(f"dot {name}:  $e$ = {num:.3f} / {den:.3f} = {num / den:.3f}", fontsize=10)
        ax.set_xlim(-.30, .30); ax.set_ylim(-.04, .40)
    h = [plt.Line2D([], [], color=BLUE, lw=1.5, label="open: $v$ = 1"),
         plt.Line2D([], [], color=RED, lw=1.5, ls=(0, (4, 2)), marker="X", ms=6, label="blocked (plate or own wall): $v$ = 0"),
         plt.Line2D([], [], color=GRAY, lw=1.2, ls=(0, (1, 2)), label="behind the tangent plane: $\\gamma$ = 0 (ignored)")]
    fig.legend(handles=h, loc="upper center", ncol=3, frameon=False, fontsize=8.5, bbox_to_anchor=(.5, .93))
    E = sum(es.values()) / 3
    fig.text(.5, .06, "numbers under the disc points: $\\gamma$ of each line.   "
             "$e$ = (sum of $\\gamma$ over open lines) / (sum of $\\gamma$ over all front-facing lines)\n"
             f"If these 3 dots were the whole bowl (Step 4):  $E$ = ({es['A']:.3f} + {es['B']:.3f} + {es['D']:.3f}) / 3 = {E:.3f}",
             ha="center", fontsize=9, linespacing=1.6)
    fig.suptitle("All 15 lines: 3 dots $\\times$ 5 spray points", fontsize=12, y=.99)
    fig.subplots_adjust(left=.04, right=.99, top=.84, bottom=.2, wspace=.12)
    pdf.savefig(fig); fig.savefig(HERE / "toy_example_p3.png", dpi=150)
    plt.close(fig)


if __name__ == "__main__":
    for name, p in DOTS.items():
        n, s, lines, num, den = trace(p)
        print(name, [(round(r["gamma"], 3), r["status"]) for r in lines], f"e = {num:.3f}/{den:.3f} = {num / den:.3f}")
    with PdfPages(HERE / "toy_example.pdf") as pdf:
        page1(pdf); page2(pdf); page3(pdf)
    print("wrote", HERE / "toy_example.pdf")
