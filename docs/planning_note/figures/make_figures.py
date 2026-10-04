"""Figures for the planning note.

    python3 docs/planning_note/figures/make_figures.py      (host: numpy + matplotlib + PIL)
Reads build/planning_note/catalogue.npz (made from extract_figdata.py's output, see the README) and the
benchmark records under results/benchmark/frigidaire_hotec/ (instances, goal searches, plans, episode
analyses), and writes next to this file:
  fig_stills.jpg     hard_s0 initial and goal stills (media/benchmark/frigidaire_hotec/stills/)
  fig_catalogue.pdf  usable catalogue poses, top view per rack, with the hard_s0 goal load
  fig_ascent.pdf     S over the accepted swaps of every instance's goal search
  fig_loads.pdf      easy_s0: the first-fit load vs the MCTS load
  fig_mcts.pdf       best low-res S found by MCTS over time, relative to first-fit
"""
import json
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import Rectangle  # noqa: E402
from PIL import Image  # noqa: E402

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
R = ROOT / "data/results/benchmark/frigidaire_hotec"
MEDIA = ROOT / "data/media/benchmark/frigidaire_hotec"

plt.rcParams.update({"font.family": "serif", "font.serif": ["DejaVu Serif"], "mathtext.fontset": "cm",
                     "font.size": 8, "axes.linewidth": 0.6, "savefig.dpi": 300, "pdf.fonttype": 42})
INK, GRAY = "#13212B", "#9aa5ae"
KIND_COLOR = {"plate": "#2a78d6", "bowl": "#eb6834", "cup": "#1baf7a"}     # same slots as the scoring note
TIER_COLOR = {"easy": "#2a78d6", "medium": "#eb6834", "hard": "#c0392b"}
RACK_BOX = {"LowerRack": ((-.28, .28), (-.30, .30)), "UpperRack": ((-.26, .26), (-.29, .29))}   # frigidaire_bench.RACK_BOX
RACK_NAME = {"LowerRack": "lower rack", "UpperRack": "upper rack"}
INSTANCES = [(t, f"{t}_s{s}") for t, n in (("easy", 6), ("medium", 3), ("hard", 3)) for s in range(n)]


def load(path):
    return json.loads(Path(path).read_text())


def instance(tier, iid):
    return load(R / "instances" / tier / f"{iid}.json")


def rack_axes(ax, rack, title):
    (x0, x1), (y0, y1) = RACK_BOX[rack]
    ax.add_patch(Rectangle((x0, y0), x1 - x0, y1 - y0, fill=False, lw=.6, ec=GRAY))
    ax.set_xlim(x0 - .02, x1 + .02)
    ax.set_ylim(y0 - .02, y1 + .02)
    ax.set_aspect("equal")
    ax.set_xticks([])
    ax.set_yticks([])
    for s in ax.spines.values():
        s.set_visible(False)
    ax.set_title(title, fontsize=8, pad=3)
    ax.text(0, y0 - .012, "door", ha="center", va="top", fontsize=6, color=GRAY)


def draw_load(ax, entries, rack, filled=True, size=26):
    for e in entries:
        if e["rack"] != rack:
            continue
        c = KIND_COLOR[e["kind"]]
        ax.scatter(e["position"][0], e["position"][1], s=size, marker="o", lw=.9,
                   fc=c if filled else "white", ec=INK if filled else c, zorder=3)


# ------------------------------------------------------------------ Fig. 1: stills
def fig_stills():
    ims = []
    for name in ("initial", "goal"):
        im = Image.open(MEDIA / "stills/hard/hard_s0" / f"{name}.png").convert("RGB")
        w, h = im.size
        im = im.crop((int(.2 * w), int(.19 * h), int(.96 * w), int(.97 * h)))   # the scene; the caption carries the title band
        ims.append(im.resize((im.width * 900 // im.height, 900)))
    gap = 30
    out = Image.new("RGB", (ims[0].width + ims[1].width + gap, 900), "white")
    out.paste(ims[0], (0, 0))
    out.paste(ims[1], (ims[0].width + gap, 0))
    out.save(HERE / "fig_stills.jpg", quality=82)


# ------------------------------------------------------------------ Fig. 2: catalogue
def fig_catalogue():
    D = np.load(ROOT / "data/build/planning_note/catalogue.npz")
    inst = instance("hard", "hard_s0")
    goal = load(R / "instances/attempts" / f"hard_s0_a{inst['attempt']}" / "goal.json")["entries"]
    fig, axes = plt.subplots(1, 2, figsize=(6.3, 2.75))
    for ax, rack in zip(axes, ("LowerRack", "UpperRack")):
        rack_axes(ax, rack, RACK_NAME[rack])
        for kind in ("plate", "bowl", "cup"):
            m = (D["kind"] == kind) & (D["rack"] == rack)
            if m.any():
                xy = D["xyz"][m]
                ax.scatter(xy[:, 0], xy[:, 1], s=2.6, c=KIND_COLOR[kind], alpha=.45, lw=0, zorder=1)
        draw_load(ax, goal, rack)
    handles = [plt.Line2D([], [], ls="", marker="o", ms=2.5, mfc=KIND_COLOR[k], mec="none", alpha=.6,
                          label=f"{k}: {int(D['n_usable'][i])} usable poses") for i, k in enumerate(("plate", "bowl", "cup"))]
    handles.append(plt.Line2D([], [], ls="", marker="o", ms=5, mfc=GRAY, mec=INK, label="hard_s0 goal load (23 dishes)"))
    fig.legend(handles=handles, loc="lower center", ncol=4, frameon=False, fontsize=7, handletextpad=.3)
    fig.subplots_adjust(left=.01, right=.99, top=.92, bottom=.12, wspace=.05)
    fig.savefig(HERE / "fig_catalogue.pdf")
    plt.close(fig)


# ------------------------------------------------------------------ Fig. 3: goal ascent
def fig_ascent():
    fig, ax = plt.subplots(figsize=(4.2, 2.2))
    for tier, iid in INSTANCES:
        inst = instance(tier, iid)
        h = load(R / "instances/attempts" / f"{iid}_a{inst['attempt']}" / "goal.json")["history"]
        s = np.array([x["score"] for x in h])
        ax.plot(np.arange(len(s)), s / s[0], color=TIER_COLOR[tier], lw=.9, marker="o", ms=1.8)
    for tier, c in TIER_COLOR.items():
        ax.plot([], [], color=c, lw=.9, label=tier)
    ax.axhline(1, color=GRAY, lw=.6, ls="--")
    ax.set_xlabel("accepted swaps $t$ (one dish moved to another free catalogue pose)")
    ax.set_ylabel(r"$S(L_t)\,/\,S(L_0)$")
    ax.legend(frameon=False, fontsize=7, loc="upper left")
    for s in ("top", "right"):
        ax.spines[s].set_visible(False)
    fig.tight_layout(pad=.3)
    fig.savefig(HERE / "fig_ascent.pdf")
    plt.close(fig)


# ------------------------------------------------------------------ Fig. 6: first-fit vs MCTS loads
def fig_loads():
    fig, axes = plt.subplots(1, 4, figsize=(6.6, 2.35))
    for col, algo in enumerate(("baseline", "mcts")):
        plan = load(R / "plans/open/easy" / f"easy_s0__{algo}.json")
        S = load(R / "episodes/open/easy" / f"easy_s0__{algo}.analysis.json")["S_final"]
        name = "first-fit" if algo == "baseline" else "MCTS"
        for k, rack in enumerate(("LowerRack", "UpperRack")):
            ax = axes[2 * col + k]
            rack_axes(ax, rack, RACK_NAME[rack])
            draw_load(ax, plan["goal"], rack, size=34)
        fig.text(.255 + .49 * col, .97, f"{name} load, settled $S$ = {S:.3f}", ha="center", va="top", fontsize=8.5,
                 fontweight="bold", color=INK)
    fig.subplots_adjust(left=.01, right=.99, top=.82, bottom=.04, wspace=.08)
    fig.savefig(HERE / "fig_loads.pdf")
    plt.close(fig)


# ------------------------------------------------------------------ Fig. 7b: MCTS progress
def fig_mcts():
    fig, ax = plt.subplots(figsize=(3.3, 2.3))
    for tier, iid in INSTANCES:
        st = load(R / "plans/open" / tier / f"{iid}__mcts.a0.json")["search"]
        h = np.array(st["history"], dtype=float)
        ff = st["first_fit_S_low"]
        t = np.concatenate([[0.], h[:, 0], [55.]])
        v = np.concatenate([[ff], h[:, 2], [h[-1, 2]]]) / ff
        ax.step(t, v, where="post", color=TIER_COLOR[tier], lw=.9)
    for tier, c in TIER_COLOR.items():
        ax.plot([], [], color=c, lw=.9, label=tier)
    ax.axvspan(0, 15, color=GRAY, alpha=.15, lw=0)
    ax.text(7.5, 1.45, "pool\n(≤ 15 s)", ha="center", va="bottom", fontsize=6.5, color=INK)
    ax.set_xlabel("planning time (s)")
    ax.set_ylabel("best $S_{\\rm low}$ / first-fit's")
    ax.set_xlim(0, 56)
    ax.legend(frameon=False, fontsize=7, loc="center right")
    for s in ("top", "right"):
        ax.spines[s].set_visible(False)
    fig.tight_layout(pad=.3)
    fig.savefig(HERE / "fig_mcts.pdf")
    plt.close(fig)


if __name__ == "__main__":
    fig_stills()
    fig_catalogue()
    fig_ascent()
    fig_loads()
    fig_mcts()
    print("[RESULT] PASS figures")
