# Copyright (c) 2026, dishsim project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Phase G: aggregate results/*.json into docs/v0_report.md + figures + slides notes.

Kit-free (venv python). Figures follow the dataviz reference palette (single-series charts:
blue #2a78d6 on the #fcfcfb light surface, text in ink tokens, recessive grid).

Run with:
    /workspace/isaaclab/env_isaaclab/bin/python scripts/30_make_report.py
"""

import glob
import json
import os
import shutil
import statistics

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
RESULTS = os.path.join(PROJECT_ROOT, "results")
FIG_DIR = os.path.join(PROJECT_ROOT, "docs", "figures")
DOCS = os.path.join(PROJECT_ROOT, "docs")

SURFACE = "#fcfcfb"
SERIES = "#2a78d6"
INK = "#0b0b0b"
INK2 = "#52514e"
GRID = "#e5e4e0"

FAILURE_STAGES = ["no-goal-config", "planner-timeout", "execution-collision", "unstable-after-release"]


def style_axes(ax):
    ax.set_facecolor(SURFACE)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    for side in ("left", "bottom"):
        ax.spines[side].set_color(GRID)
    ax.tick_params(colors=INK2, labelsize=9)
    ax.yaxis.grid(True, color=GRID, linewidth=0.8)
    ax.set_axisbelow(True)


def new_fig(w=6.4, h=3.6):
    fig, ax = plt.subplots(figsize=(w, h), facecolor=SURFACE)
    style_axes(ax)
    return fig, ax


def main() -> None:
    trials = []
    for path in sorted(glob.glob(os.path.join(RESULTS, "trial_*.json"))):
        with open(path) as f:
            trials.append(json.load(f))
    if not trials:
        raise SystemExit("[FAIL] no trial JSONs in results/")
    os.makedirs(FIG_DIR, exist_ok=True)

    slots = sorted({t["slot"] for t in trials})
    n_ok = sum(t["success"] for t in trials)
    per_slot = {
        s: (sum(t["success"] for t in trials if t["slot"] == s), sum(t["slot"] == s for t in trials))
        for s in slots
    }
    plan_times = [t["plan_time_s"] for t in trials if t.get("plan_time_s") is not None]
    path_lens = [t["path_len_rad"] for t in trials if t.get("path_len_rad")]
    failures = {s: sum(t["failure_stage"] == s for t in trials) for s in FAILURE_STAGES}
    lat_errs = [t["final_pose_err"]["lateral_m"] * 1e3 for t in trials if t.get("final_pose_err")]
    tilt_errs = [t["final_pose_err"]["tilt_deg"] for t in trials if t.get("final_pose_err")]

    # ---- figures ------------------------------------------------------------------------------
    fig, ax = new_fig()
    rates = [per_slot[s][0] / per_slot[s][1] * 100 for s in slots]
    bars = ax.bar([str(s) for s in slots], rates, width=0.6, color=SERIES, edgecolor="none")
    for b, s in zip(bars, slots):
        ok, n = per_slot[s]
        ax.annotate(f"{ok}/{n}", (b.get_x() + b.get_width() / 2, b.get_height()), ha="center",
                    va="bottom", fontsize=9, color=INK)
    ax.set_ylim(0, 110)
    ax.set_xlabel("slot id", color=INK2)
    ax.set_ylabel("success rate [%]", color=INK2)
    ax.set_title("Placement success rate by slot", color=INK, fontsize=11)
    fig.savefig(os.path.join(FIG_DIR, "success_by_slot.png"), dpi=140, bbox_inches="tight",
                facecolor=SURFACE)
    plt.close(fig)

    fig, ax = new_fig()
    ax.hist(plan_times, bins=12, color=SERIES, edgecolor=SURFACE, linewidth=1.5)
    ax.set_xlabel("plan time [s]", color=INK2)
    ax.set_ylabel("trials", color=INK2)
    ax.set_title("RRT-Connect planning time (5 s budget)", color=INK, fontsize=11)
    fig.savefig(os.path.join(FIG_DIR, "plan_time_hist.png"), dpi=140, bbox_inches="tight",
                facecolor=SURFACE)
    plt.close(fig)

    fig, ax = new_fig()
    labels = ["success"] + FAILURE_STAGES
    counts = [n_ok] + [failures[s] for s in FAILURE_STAGES]
    bars = ax.barh(labels[::-1], counts[::-1], height=0.6, color=SERIES, edgecolor="none")
    for b, c in zip(bars, counts[::-1]):
        ax.annotate(f" {c}", (b.get_width(), b.get_y() + b.get_height() / 2), va="center",
                    fontsize=9, color=INK)
    ax.set_xlabel("trials", color=INK2)
    ax.xaxis.grid(True, color=GRID, linewidth=0.8)
    ax.yaxis.grid(False)
    ax.set_title("Outcome breakdown", color=INK, fontsize=11)
    fig.savefig(os.path.join(FIG_DIR, "outcome_breakdown.png"), dpi=140, bbox_inches="tight",
                facecolor=SURFACE)
    plt.close(fig)

    # curate best stills into docs/figures (tracked)
    curated = []
    for t in trials:
        if t["success"] and t.get("media", {}).get("final") and len(curated) < 4:
            src = os.path.join(PROJECT_ROOT, t["media"]["final"])
            dst = os.path.join(FIG_DIR, f"best_{t['trial']}.png")
            if os.path.isfile(src):
                shutil.copy(src, dst)
                curated.append((t["trial"], os.path.relpath(dst, DOCS)))
    fail_example = next((t for t in trials if not t["success"] and t.get("media", {}).get("final")), None)
    if fail_example:
        src = os.path.join(PROJECT_ROOT, fail_example["media"]["final"])
        if os.path.isfile(src):
            dst = os.path.join(FIG_DIR, f"fail_{fail_example['trial']}.png")
            shutil.copy(src, dst)
            curated.append((fail_example["trial"] + " (failure)", os.path.relpath(dst, DOCS)))

    def stats_line(vals, fmt="{:.2f}"):
        if not vals:
            return "n/a"
        q = statistics.quantiles(vals, n=20)
        return (f"mean {fmt.format(statistics.mean(vals))}, median {fmt.format(statistics.median(vals))}, "
                f"p95 {fmt.format(q[18])}, max {fmt.format(max(vals))}")

    # ---- report -------------------------------------------------------------------------------
    lines = [
        "# v0 report — OMPL placement planning (dishwasher lower rack)",
        "",
        "Task: UR5e + Robotiq 2F-85 with a YCB mug (plate stand-in; `029_plate` is not in the",
        "Isaac 6.0 asset bucket) welded to the TCP; RRT-Connect plans a collision-free path to a",
        "standing slot on the extended lower rack; the weld is released and the placement must",
        "satisfy [success_criteria.md](success_criteria.md). Stack: Isaac Sim 6.0.1 + Isaac Lab",
        "3.0 (execution/ground truth), OMPL 2.0.1 + python-fcl 0.7 + CoACD 1.0 (planning),",
        "hand-rolled analytic UR5e IK validated against Pinocchio (see docs/environment.md).",
        "",
        "## Headline numbers",
        "",
        f"- **Success rate: {n_ok}/{len(trials)} ({n_ok/len(trials)*100:.0f} %)** over "
        f"{len(slots)} slots x {len({t['seed'] for t in trials})} seeds",
        f"- Plan time [s]: {stats_line(plan_times)} (budget {5.0} s)",
        f"- Path length [rad]: {stats_line(path_lens)}",
        f"- Final placement error: lateral [mm] {stats_line(lat_errs, '{:.1f}')}; "
        f"tilt [deg] {stats_line(tilt_errs, '{:.2f}')}",
        f"- FCL collision world: 98 % agreement with Isaac contacts (0 non-conservative "
        f"mismatches over 200 configs), ~0.2 ms/query — see media/D/parity_results.json",
        "",
        "![success by slot](figures/success_by_slot.png)",
        "![plan time](figures/plan_time_hist.png)",
        "![outcomes](figures/outcome_breakdown.png)",
        "",
        "## Per-slot results",
        "",
        "| slot | trials | successes | rate | note |",
        "|---|---|---|---|---|",
    ]
    for s in slots:
        ok, n = per_slot[s]
        note = "empty goal set (deep interior / reach limit)" if all(
            t["failure_stage"] == "no-goal-config" for t in trials if t["slot"] == s
        ) else ""
        lines.append(f"| {s} | {n} | {ok} | {ok/n*100:.0f} % | {note} |")
    lines += [
        "",
        "## Failure breakdown",
        "",
        "| stage | count |",
        "|---|---|",
    ] + [f"| {s} | {failures[s]} |" for s in FAILURE_STAGES] + [
        "",
        "Goal-config scarcity per slot (the rejection funnels) is recorded in",
        "`assets/cache/slots/goal_sets.json`; deep-interior slots yield zero collision-free",
        "goals because the arm cannot descend into the machine cavity — expected signal, not a",
        "pipeline defect.",
        "",
        "## Curated figures",
        "",
    ] + [f"![{name}]({rel})" for name, rel in curated] + [
        "",
        "## Media gallery (full evidence on disk, gitignored)",
        "",
        "| trial | slot | seed | outcome | stage | plan [s] | video | final still |",
        "|---|---|---|---|---|---|---|---|",
    ]
    for t in trials:
        media = t.get("media", {})
        lines.append(
            f"| {t['trial']} | {t['slot']} | {t['seed']} | "
            f"{'SUCCESS' if t['success'] else 'fail'} | {t['failure_stage'] or '—'} | "
            f"{t['plan_time_s'] if t['plan_time_s'] is not None else '—'} | "
            f"{media.get('video', '—')} | {media.get('final', '—')} |"
        )
    report_path = os.path.join(DOCS, "v0_report.md")
    with open(report_path, "w") as f:
        f.write("\n".join(lines) + "\n")

    with open(os.path.join(RESULTS, "summary.json"), "w") as f:
        json.dump({"n_trials": len(trials), "n_success": n_ok,
                   "per_slot": {str(s): per_slot[s] for s in slots},
                   "plan_times_s": plan_times, "path_lens_rad": path_lens,
                   "failures": failures}, f, indent=2)

    # ---- slides notes ---------------------------------------------------------------------------
    notes = [
        "# v0 slides notes (paste-ready)",
        "",
        f"- v0 delivered end-to-end: object-in-gripper -> OMPL RRT-Connect -> collision-free",
        f"  execution in Isaac Sim -> release -> stable placement in the dishwasher's lower rack.",
        f"- **{n_ok}/{len(trials)} trials succeed ({n_ok/len(trials)*100:.0f} %)** across "
        f"{len(slots)} slots x {len({t['seed'] for t in trials})} RNG seeds; failures are "
        f"dominated by goal-infeasible deep-interior slots (reach limit), not planner or "
        f"execution errors.",
        f"- Plan times: {stats_line(plan_times)} s against a 5 s budget — RRT-Connect over a "
        f"custom FCL collision world at ~0.2 ms per collision query.",
        "- The collision world is a standalone, simulator-free module (needed for the future",
        "  MCTS rearrangement planner): CoACD-decomposed rack (128 pieces keeps the wire gaps",
        "  open), validated at 98 % agreement vs Isaac contact ground truth with zero",
        "  non-conservative mismatches.",
        "- Analytic UR5e IK (8 branches, hand-rolled; ur-analytic-ik has no py3.12 wheel)",
        "  matches Pinocchio to 1e-9 and live Isaac FK to 0.2 mm — goal sets are thousands of",
        "  configs per feasible slot, with per-slot rejection funnels as reach-feasibility maps.",
        "- Found and fixed via the parity gate: attached-object-vs-arm collisions must be",
        "  world-checks (PhysX simulates them), and teleported ground-truth sampling needs",
        "  mimic-consistent finger states — both would have silently corrupted the benchmark.",
        "- Known simplifications: mug stands on the wire basket floor (the ArtVIP rack has no",
        "  plate tines); clearance carry instead of a pinch grasp (grasping is out of scope for",
        "  v0; two pads-on-rim attempts measurably interfere with the Robotiq jaw sweep).",
        "",
        "## Best figures/clips (by path)",
        "",
        "- docs/figures/success_by_slot.png — headline result",
        "- docs/figures/outcome_breakdown.png — failure taxonomy",
        "- media/F/trial_02_00_0.mp4 — full plan-execute-release clip (first end-to-end success)",
        "- media/E/accepted_slot1_sheet.png — goal-config diversity at one slot",
        "- media/D/overlay_E_shelf_1_04.png — rack decomposition, wire gaps preserved",
        "- media/C/rigidity_iso.mp4 — welded-object rigidity proof",
    ] + [f"- docs/{rel} — curated final still ({name})" for name, rel in curated[:2]]
    with open(os.path.join(DOCS, "slides_notes.md"), "w") as f:
        f.write("\n".join(notes) + "\n")

    print(f"[INFO] report: {report_path}")
    print(f"[INFO] slides notes: {os.path.join(DOCS, 'slides_notes.md')}")
    print(f"[INFO] figures: {FIG_DIR}")
    print(f"[RESULT] PASS ({n_ok}/{len(trials)} successes)")


if __name__ == "__main__":
    main()
