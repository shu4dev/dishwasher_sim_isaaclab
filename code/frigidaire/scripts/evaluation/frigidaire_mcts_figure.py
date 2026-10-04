#!/usr/bin/env python3
"""Method figure of the track-B MCTS planner: how the search works, what it found in its 60 s, and S vs budget.

    code/scripts/run_py.sh code/frigidaire/scripts/evaluation/frigidaire_mcts_figure.py

Reads the certified MCTS plans (data/results/benchmark/frigidaire_hotec/plans/open/<tier>/<id>__mcts.json, their
``search`` statistics) and the Kit-free budget sweep (mcts_budget_sweep/*.json); writes
data/media/benchmark/frigidaire_hotec/mcts_method.png. Planned S only (before Isaac): the settled S of the same loads
is in the episode analyses.
"""
from __future__ import annotations

import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[4]
OUT = ROOT / "data/results/benchmark/frigidaire_hotec"
MEDIA = ROOT / "data/media/benchmark/frigidaire_hotec"
TIER_COLOR = {"easy": "#3b7dd8", "medium": "#d9822b", "hard": "#b33a3a"}


def main():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import FancyBboxPatch

    plans = []
    for p in sorted((OUT / "plans" / "open").glob("*/*__mcts.json")):
        d = json.loads(p.read_text())
        if d.get("search"):
            plans.append((p.parent.name, d["instance"], d))
    sweep = {}
    for p in sorted((OUT / "mcts_budget_sweep").glob("*.json")):
        d = json.loads(p.read_text())
        sweep.setdefault(d["instance"], {})[d["budget_s"]] = d

    fig = plt.figure(figsize=(15, 5.2))
    ax0 = fig.add_axes([0.01, 0.05, 0.33, 0.88])
    ax0.set_axis_off()
    ax0.set_xlim(0, 10)
    ax0.set_ylim(0, 10)
    steps = [("Pool", "first-fit + seeded packs;\nroot = best complete load"),
             ("Select", "UCT down the tree\n(progressive widening)"),
             ("Expand", "one move: place a dish on a\ntop pose, keep it, or park it"),
             ("Rollout", "repair the parent's load locally,\nscore it (low-res S)"),
             ("Back up", "S to every node on the path"),
             ("Answer", "best load: tree prefix + greedy\ncompletion -> Isaac certification")]
    for i, (title, text) in enumerate(steps):
        y = 9.0 - i * 1.55
        ax0.add_patch(FancyBboxPatch((0.3, y - 0.62), 9.4, 1.2, boxstyle="round,pad=0.05", fc="#eef3f8", ec="#6b7f93"))
        ax0.text(0.6, y, title, fontsize=11, weight="bold", va="center")
        ax0.text(3.0, y, text, fontsize=9, va="center")
        if i < len(steps) - 1:
            ax0.annotate("", xy=(5, y - 0.93), xytext=(5, y - 0.64), arrowprops=dict(arrowstyle="->", color="#6b7f93"))
    ax0.annotate("", xy=(9.85, 7.45), xytext=(9.85, 3.0), arrowprops=dict(arrowstyle="->", color="#6b7f93",
                 connectionstyle="arc3,rad=-0.3"))
    ax0.text(9.3, 5.2, "repeat\nuntil 55 s", fontsize=8, color="#6b7f93", ha="right")
    ax0.text(0.3, 9.95, "Move-level MCTS (track B, 60 s)", fontsize=12, weight="bold")

    ax1 = fig.add_axes([0.40, 0.14, 0.27, 0.74])
    for tier, iid, d in plans:
        s = d["search"]
        hist = s.get("history") or []
        if not hist:
            continue
        x0, base = hist[0][0], hist[0][2]                  # the best pooled packing = where the tree starts
        xs = [h[0] - x0 for h in hist]
        ys = [h[2] / base for h in hist]
        ax1.plot(xs, ys, color=TIER_COLOR.get(tier, "k"), lw=1.2, alpha=.8)
        ax1.text(xs[-1], ys[-1], iid.split("_")[1], fontsize=7, color=TIER_COLOR.get(tier, "k"))
    ax1.axhline(1., color="#888", ls="--", lw=1)
    ax1.set_xlabel("tree search time, s")
    ax1.set_ylabel("best S found / S of the starting load (low-res)")
    ax1.set_title("What the tree adds to the best pooled packing", fontsize=11)
    for tier, c in TIER_COLOR.items():
        ax1.plot([], [], color=c, label=tier)
    ax1.legend(fontsize=8, loc="lower right")

    ax2 = fig.add_axes([0.72, 0.14, 0.27, 0.74])
    for tier, iid, d in plans:
        if iid not in sweep:
            continue
        a0 = OUT / "plans" / "open" / tier / f"{iid}__mcts.a0.json"    # same seed and start as the sweep
        pts = {60.: json.loads(a0.read_text()).get("S_planned") if a0.is_file() else None}
        pts.update({b: r["S_planned"] for b, r in sweep[iid].items()})
        ff = next(iter(sweep[iid].values())).get("S_first_fit")
        xs = sorted(b for b, v in pts.items() if v)
        ax2.plot(xs, [pts[b] / ff for b in xs], "o-", color=TIER_COLOR.get(tier, "k"), lw=1.2, ms=4)
        ax2.text(xs[-1] * 1.05, pts[xs[-1]] / ff, iid.split("_")[1], fontsize=7, color=TIER_COLOR.get(tier, "k"))
    ax2.axhline(1., color="#888", ls="--", lw=1)
    ax2.set_xscale("log")
    ax2.set_xticks([60, 180, 600])
    ax2.set_xticklabels(["60", "180", "600"])
    ax2.set_xlabel("planning budget, s")
    ax2.set_ylabel("planned S / first-fit planned S")
    ax2.set_title("More budget (Kit-free sweep, medium and hard)", fontsize=11)
    MEDIA.mkdir(parents=True, exist_ok=True)
    out = MEDIA / "mcts_method.png"
    fig.savefig(out, dpi=130)
    print(f"[RESULT] PASS figure {out.relative_to(ROOT)} ({len(plans)} plans, {len(sweep)} swept instances)", flush=True)


if __name__ == "__main__":
    main()
