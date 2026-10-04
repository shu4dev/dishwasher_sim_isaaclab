"""Figures for the math section of the scoring note, one per step, labeled with its notation.

    python3 docs/scoring_note/figures/make_figures.py      (host: numpy + matplotlib)
Reads build/scoring_note/figdata.npz (made by extract_figdata.py in the container: the real hard_s0
goal load scored with the revision-5 scorer) and writes step1_dots.pdf, step2_disc.pdf,
step3_dot.pdf, step4_dish.pdf, step5_load.pdf next to this file. Steps 2-3 are exact arithmetic
with the scorer's disc formula; Fig. 2 right is tikz_exposure_geometry.tex.
"""
import math
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import Wedge, Arc  # noqa: E402
from mpl_toolkits.mplot3d import proj3d  # noqa: E402
from mpl_toolkits.mplot3d.art3d import Poly3DCollection  # noqa: E402

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
D = np.load(ROOT / "data/build/scoring_note/figdata.npz")

plt.rcParams.update({"font.family": "serif", "font.serif": ["DejaVu Serif"], "mathtext.fontset": "cm",
                     "font.size": 8, "axes.linewidth": 0.6, "savefig.dpi": 300, "pdf.fonttype": 42})
BLUE, RED, GRAY, INK = "#2a78d6", "#c0392b", "#9aa5ae", "#13212B"
KIND_COLOR = {"plate": "#2a78d6", "bowl": "#eb6834", "cup": "#1baf7a"}   # validated categorical slots 1-3
FOOD, SHELL = np.array([0.85, 0.72, 0.50]), np.array([0.80, 0.80, 0.80])
EMAP = plt.get_cmap("viridis")   # same map as the benchmark score-detail figures

# scorer constants (exposure.py): lower-arm disc, K points, line start offset
M_LOW, R_LOW, K, EPS = np.array([0., .008, .185]), .233, 64, 1e-4
PHI = math.pi * (3. - math.sqrt(5.))


def sunflower(center, radius, k=K):
    i = np.arange(k)
    r = radius * np.sqrt((i + .5) / k)
    return np.column_stack((r * np.cos(i * PHI), r * np.sin(i * PHI), np.zeros(k))) + center


def shade(tris, base, light=np.array([-.4, -.6, .8])):
    n = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
    n /= np.maximum(np.linalg.norm(n, axis=1, keepdims=True), 1e-12)
    lam = .55 + .45 * np.abs(n @ (light / np.linalg.norm(light)))
    return np.clip(np.asarray(base)[None, :] * lam[:, None], 0, 1)


def draw_last(coll, z=-1e9):
    """mplot3d sorts 3D collections by depth; make this one draw after the dish surface."""
    orig = coll.do_3d_projection

    def proj(renderer, orig=orig):
        orig(renderer)
        return z
    coll.do_3d_projection = proj


def cut_dish(ax, kind, dot_colors, elev, cut_deg=90., size=3.2, food_rgb=FOOD):
    """Half of the upright dish on the side cut_deg (dish frame), seen from the other side, so the inside shows.
    Food-contact surface tan (or food_rgb), the rest gray. Returns the mask of dots drawn."""
    vis, food = D[f"{kind}_visual"], D[f"{kind}_food"]
    u = np.array([math.cos(math.radians(cut_deg)), math.sin(math.radians(cut_deg)), 0.])
    keep = vis.mean(axis=1) @ u >= 0
    tris = vis[keep]
    food_c = {tuple(c) for c in np.round(food.mean(axis=1), 6)}
    is_food = np.array([tuple(c) in food_c for c in np.round(tris.mean(axis=1), 6)])
    cols = np.where(is_food[:, None], shade(tris, food_rgb), shade(tris, SHELL))
    pc = Poly3DCollection(tris, facecolors=cols, edgecolors="none", linewidths=0, alpha=.6)
    pc.set_rasterized(True)
    ax.add_collection3d(pc)
    pts = D[f"{kind}_pts"]
    back = pts @ u >= 0
    c = np.asarray(dot_colors)[back] if np.ndim(dot_colors) else dot_colors
    draw_last(ax.scatter(pts[back, 0], pts[back, 1], pts[back, 2], s=size, c=c, depthshade=False, linewidths=0))
    lo, hi = vis.reshape(-1, 3).min(0), vis.reshape(-1, 3).max(0)
    mid, h = (lo + hi) / 2, (hi - lo).max() / 2 * .78
    ax.set_xlim(mid[0] - h, mid[0] + h); ax.set_ylim(mid[1] - h, mid[1] + h); ax.set_zlim(mid[2] - h, mid[2] + h)
    ax.view_init(elev=elev, azim=cut_deg - 180.)
    ax.set_axis_off()
    return back


def to2d(ax, p):
    x, y, _ = proj3d.proj_transform(p[0], p[1], p[2], ax.get_proj())
    return x, y


def step1():
    fig = plt.figure(figsize=(3.4, 1.75))
    ax = fig.add_axes([-.08, -.5, .66, 1.6], projection="3d")
    back = cut_dish(ax, "bowl", BLUE, elev=30)
    pts, nrm = D["bowl_pts"], D["bowl_nrm"]
    rad = np.einsum('ij,ij->i', nrm[:, :2], pts[:, :2])
    cand = np.flatnonzero(back & (pts[:, 0] > .03) & (pts[:, 2] > .03) & (pts[:, 2] < .06) & (rad < 0))
    j = cand[np.argmin(nrm[cand, 0] + np.abs(nrm[cand, 1]))]   # side wall, normal pointing left in the picture
    p, n = pts[j], nrm[j]
    draw_last(ax.scatter([p[0]], [p[1]], [p[2]], s=22, c=RED, depthshade=False, linewidths=0), -2e9)
    L = .035
    draw_last(ax.quiver(p[0], p[1], p[2], n[0], n[1], n[2], length=L, color=INK, lw=1.2, arrow_length_ratio=.35), -3e9)
    fig.canvas.draw()
    ax.annotate(r"$\mathbf{p}_j$", to2d(ax, p), xytext=(4, -9), textcoords="offset points", fontsize=9, color=RED)
    ax.annotate(r"$\mathbf{n}_j$", to2d(ax, p + L * n), xytext=(-6, 4), textcoords="offset points", fontsize=9)
    fig.text(.53, .86, "dish $o$: a bowl, cut in half", fontsize=7.5, weight="bold")
    fig.text(.53, .71, "tan: food-contact surface,\narea $A_o$ (340 cm$^2$ for a bowl)", fontsize=7.5, va="top")
    fig.text(.53, .47, "blue: its $N=500$ dots, $j\\in\\mathcal{J}_o$;\neach stands for area $A_o/N$", fontsize=7.5, va="top")
    fig.text(.53, .23, "red: one dot $j$ at $\\mathbf{p}_j$, with\nunit normal $\\mathbf{n}_j$ into the cavity", fontsize=7.5, va="top")
    fig.savefig(HERE / "step1_dots.pdf")
    plt.close(fig)


def step2():
    q = sunflower(np.zeros(3), R_LOW)
    k = 30
    fig, ax = plt.subplots(figsize=(1.9, 1.9))
    ax.add_patch(plt.Circle((0, 0), R_LOW, fill=False, lw=.6, color=GRAY))
    r0, r1 = R_LOW * math.sqrt(k / K), R_LOW * math.sqrt((k + 1) / K)
    ax.add_patch(Wedge((0, 0), r1, 0, 360, width=r1 - r0, color="#f0b27a", lw=0))
    ax.scatter(q[:, 0], q[:, 1], s=5, color=BLUE, zorder=3, linewidths=0)
    for i, lab in ((k, r"$\mathbf{q}_k$"), (k + 1, r"$\mathbf{q}_{k+1}$")):
        ax.plot([0, q[i, 0]], [0, q[i, 1]], color=INK, lw=.5, zorder=2)
        ax.scatter([q[i, 0]], [q[i, 1]], s=16, color=RED, zorder=4, linewidths=0)
        ax.annotate(lab, q[i, :2], xytext=(3, 2), textcoords="offset points", fontsize=8)
    a0 = math.degrees(k * PHI) % 360
    ax.add_patch(Arc((0, 0), .09, .09, theta1=a0, theta2=a0 + math.degrees(PHI), lw=.7, color=INK))
    am = math.radians(a0 + math.degrees(PHI) / 2)
    ax.text(.07 * math.cos(am), .07 * math.sin(am), r"$\phi$", fontsize=8, ha="center", va="center")
    ax.annotate("", xy=(R_LOW * math.cos(math.radians(20)), R_LOW * math.sin(math.radians(20))), xytext=(0, 0),
                arrowprops=dict(arrowstyle="-|>", lw=.6, color=INK, shrinkA=0, shrinkB=0))
    ax.text(.13, .075, r"$\rho_a$", fontsize=8)
    ax.plot(0, 0, marker="+", color=INK, ms=6, mew=.9, zorder=5)
    ax.annotate(r"$\mathbf{m}_a$", (0, 0), xytext=(1, 5), textcoords="offset points", fontsize=8)
    ax.text(-.245, .215, r"$\mathcal{D}_a$", fontsize=8)
    ax.set_xlim(-.25, .25); ax.set_ylim(-.25, .25); ax.set_aspect("equal"); ax.set_axis_off()
    fig.savefig(HERE / "step2_disc.pdf", bbox_inches="tight", pad_inches=.02)
    plt.close(fig)


def dot_case(n, blocked):
    q = sunflower(M_LOW, R_LOW)
    s = np.array([0., .008, .300]) + EPS * n
    d = q - s
    d /= np.linalg.norm(d, axis=1, keepdims=True)
    g = np.maximum(d @ n, 0.)
    front, opn = g > 0, (g > 0) & ~blocked
    return q - M_LOW, g, front, opn, g.sum() / K, g[opn].sum() / K


def step3():
    q0 = sunflower(np.zeros(3), R_LOW)
    cases = [(np.array([0., 0., -1.]), np.arange(K) < K // 2, "(i) $\\mathbf{n}_j$ into the page,\ninner half blocked"),
             (np.array([1., 0., 0.]), np.zeros(K, bool), "(ii) $\\mathbf{n}_j$ points to $+x$,\nnone blocked")]
    fig, axes = plt.subplots(1, 2, figsize=(3.5, 2.7))
    for ax, (n, blk, title) in zip(axes, cases):
        q, g, front, opn, fw, ow = dot_case(n, blk)
        ax.add_patch(plt.Circle((0, 0), R_LOW + .01, color="#eef3f8", zorder=0))
        size = 4 + 38 * g
        ax.scatter(q[opn, 0], q[opn, 1], s=size[opn], color=BLUE, linewidths=0, zorder=3)
        bl = front & ~opn
        ax.scatter(q[bl, 0], q[bl, 1], s=size[bl], color=RED, marker="X", linewidths=0, zorder=3)
        ax.scatter(q[~front, 0], q[~front, 1], s=9, facecolors="none", edgecolors=GRAY, linewidths=.6, zorder=3)
        ax.plot(0, 0, marker=r"$\otimes$" if n[2] < 0 else "o", color=INK, ms=7 if n[2] < 0 else 3, zorder=4)
        if n[0] > 0:
            ax.annotate("", xy=(.11, 0), xytext=(0, 0), arrowprops=dict(arrowstyle="-|>", color=INK, lw=.8))
        ax.set_title(title, fontsize=7)
        ax.text(0, -.275, rf"$\sum_k u_k\gamma_{{jk}}v_{{jk}}$ = {ow:.3f} (blue)" "\n"
                rf"$\sum_k u_k\gamma_{{jk}}$ = {fw:.3f} (blue + red)" "\n"
                rf"$e_j$ = {ow / fw:.3f}", ha="center", va="top", fontsize=7, linespacing=1.4)
        ax.set_xlim(-.26, .26); ax.set_ylim(-.26, .26); ax.set_aspect("equal"); ax.set_axis_off()
    h = [plt.Line2D([], [], ls="", marker="o", color=BLUE, ms=4, label=r"open, $v_{jk}=1$"),
         plt.Line2D([], [], ls="", marker="X", color=RED, ms=4, label=r"blocked, $v_{jk}=0$"),
         plt.Line2D([], [], ls="", marker="o", mfc="none", mec=GRAY, ms=4, label=r"behind, $\gamma_{jk}=0$")]
    fig.legend(handles=h, loc="lower center", ncol=3, frameon=False, fontsize=7, bbox_to_anchor=(.5, -.01),
               handletextpad=.2, columnspacing=1.)
    fig.subplots_adjust(left=.01, right=.99, top=.87, bottom=.38, wspace=.08)
    fig.savefig(HERE / "step3_dot.pdf")
    plt.close(fig)


def step4():
    j = list(D["ids"]).index("bowl_05")
    e = D["e"][j]
    pts = D["bowl_pts"]
    cuts = np.arange(0, 360, 10.)
    gap = [abs(e[pts @ np.array([math.cos(math.radians(c)), math.sin(math.radians(c)), 0.]) >= 0].mean() - e.mean()) for c in cuts]
    cut = float(cuts[int(np.argmin(gap))])
    fig = plt.figure(figsize=(3.4, 1.6))
    ax = fig.add_axes([-.1, -.46, .62, 1.5], projection="3d")
    shown = cut_dish(ax, "bowl", EMAP(e), elev=35, cut_deg=cut, size=7, food_rgb=np.array([.9, .9, .9]))
    print(f"step4: cut {cut:.0f} deg, shown half mean {e[shown].mean():.3f} vs E {e.mean():.3f}")
    fig.text(.25, .93, "bowl_05, dots colored by $e_j$", ha="center", fontsize=7)
    hx = fig.add_axes([.63, .26, .35, .6])
    _, _, bars = hx.hist(e, bins=np.linspace(0, 1, 21), edgecolor="white", linewidth=.4)
    for b, c in zip(bars, np.linspace(.025, .975, 20)):
        b.set_facecolor(EMAP(c))
    Eo = float(e.mean())
    hx.axvline(Eo, color=INK, lw=1.)
    hx.set_title(rf"line: $E_o = \frac{{1}}{{N}}\sum_{{j\in\mathcal{{J}}_o}} e_j = {Eo:.3f}$", fontsize=7)
    hx.set_xticks([0, .5, 1])
    hx.set_xlabel(r"$e_j$ of its $N=500$ dots", fontsize=7)
    hx.set_ylabel("dots", fontsize=7)
    hx.set_xlim(0, 1)
    for sp in ("top", "right"):
        hx.spines[sp].set_visible(False)
    hx.tick_params(labelsize=6, width=.5, length=2)
    fig.savefig(HERE / "step4_dish.pdf")
    plt.close(fig)


def step5():
    kinds, A, E = np.array(D["kinds"]), D["areas"] * 1e4, D["E"]
    order = np.concatenate([np.flatnonzero(kinds == k)[np.argsort(-E[kinds == k])] for k in ("plate", "bowl", "cup")])
    S, W = float(D["S"]), float(D["W"])
    fig, ax = plt.subplots(figsize=(3.5, 2.0))
    x = 0.
    for i in order:
        ax.bar(x, E[i], width=A[i], align="edge", color=KIND_COLOR[kinds[i]], edgecolor="white", linewidth=.6)
        x += A[i]
    ax.axhline(S, color=INK, lw=1., ls=(0, (4, 2)), zorder=3)
    ax.text(x + 60, S, f"$S$ = {S:.3f}", ha="left", va="center", fontsize=7, clip_on=False)
    xs = np.cumsum([0] + [A[i] for i in order])
    # one column spelled out: width A_o, height E_o, area A_o E_o
    w0, h0 = A[order[0]], E[order[0]]
    ax.annotate("", xy=(0, h0 + .035), xytext=(w0, h0 + .035), arrowprops=dict(arrowstyle="<->", lw=.6, color=INK))
    ax.text(w0 / 2, h0 + .045, "$A_o$", ha="center", va="bottom", fontsize=7.5)
    ax.text(w0 / 2, h0 / 2, "$A_oE_o$", ha="center", va="center", fontsize=6.5, color="white", rotation=90)
    ax.annotate("", xy=(-160, 0), xytext=(-160, h0), arrowprops=dict(arrowstyle="<->", lw=.6, color=INK), annotation_clip=False)
    ax.text(-230, h0 * .55, "$E_o$", ha="right", va="center", fontsize=7.5)
    iw = int(np.argmin(E[order]))
    cx = xs[iw] + A[order[iw]] / 2
    ax.annotate(rf"$W=\min_o E_o$ = {W:.3f}", xy=(cx, W + .004), xytext=(cx, .45), ha="center", fontsize=7,
                arrowprops=dict(arrowstyle="-", lw=.5, color=INK))
    ax.set_xlim(-420, x); ax.set_ylim(0, .5)
    ax.spines['left'].set_position(('data', -420))
    ax.set_xlabel(r"the 23 dishes side by side: food-contact area (cm$^2$)")
    ax.set_ylabel("dish score $E_o$")
    for sp in ("top", "right"):
        ax.spines[sp].set_visible(False)
    ax.tick_params(width=.5, length=2)
    h = [plt.Rectangle((0, 0), 1, 1, color=KIND_COLOR[k]) for k in ("plate", "bowl", "cup")]
    ax.legend(h, ["plates", "bowls", "cups"], loc="upper right", ncol=3, frameon=False, fontsize=7,
              bbox_to_anchor=(1, 1.13), handlelength=1, columnspacing=.8)
    fig.subplots_adjust(left=.13, right=.83, top=.9, bottom=.22)
    fig.savefig(HERE / "step5_load.pdf")
    plt.close(fig)


if __name__ == "__main__":
    for f in (step1, step2, step3, step4, step5):
        f()
        print("wrote", f.__name__)
