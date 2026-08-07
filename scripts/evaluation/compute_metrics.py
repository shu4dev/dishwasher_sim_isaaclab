# Copyright (c) 2026, dishsim project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Compute metrics for one or more experiment runs — from disk, with no simulator.

Reads the trial records an experiment wrote and emits ``metrics.json`` plus figures. With
several runs it also prints the planner-comparison table, which is the point of making the
planner pluggable: run the same slots and seeds under different algorithms, then compare.

Venv-side, no Kit:
    env_isaaclab/bin/python scripts/evaluation/compute_metrics.py                 # the latest run
    env_isaaclab/bin/python scripts/evaluation/compute_metrics.py --run <run_id>
    env_isaaclab/bin/python scripts/evaluation/compute_metrics.py --all           # every run + comparison
"""

import argparse
import glob
import json
import os
import sys

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))  # scripts/<phase>/<file>.py
sys.path.insert(0, os.path.join(PROJECT_ROOT, "src"))

from dishsim import metrics as dmetrics  # noqa: E402

RESULTS = os.path.join(PROJECT_ROOT, "results")

parser = argparse.ArgumentParser(description="Aggregate experiment results into metrics + figures.")
parser.add_argument("--run", type=str, default=None, help="Run id (default: the latest run).")
parser.add_argument("--all", action="store_true", help="Summarize every run and compare them.")
parser.add_argument("--out", type=str, default=None,
                    help="Output dir (default: results/evaluation/<run_id>).")
parser.add_argument("--no_figures", action="store_true")
args = parser.parse_args()


def figures(summary: dict, out_dir: str) -> list[str]:
    """Success-by-slot and plan-time figures; returns the paths written."""
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    os.makedirs(out_dir, exist_ok=True)
    written = []

    by_slot = summary["by_slot"]
    if by_slot:
        fig, ax = plt.subplots(figsize=(7, 3.2))
        slots = list(by_slot)
        ax.bar(slots, [by_slot[s]["rate"] for s in slots], color="#1a80bb")
        ax.set_ylim(0, 1.05)
        ax.set_xlabel("slot")
        ax.set_ylabel("success rate")
        ax.set_title(f"{summary['n_success']}/{summary['n_trials']} trials succeeded")
        for i, s in enumerate(slots):
            ax.text(i, by_slot[s]["rate"] + 0.03, f"{by_slot[s]['success']}/{by_slot[s]['trials']}",
                    ha="center", fontsize=8)
        fig.tight_layout()
        path = os.path.join(out_dir, "success_by_slot.png")
        fig.savefig(path, dpi=140)
        plt.close(fig)
        written.append(path)

    by_planner = summary["by_planner"]
    if len(by_planner) > 1:
        fig, ax = plt.subplots(figsize=(6, 3.2))
        names = sorted(by_planner)
        ax.bar(names, [by_planner[n]["rate"] for n in names], color="#c23b22")
        ax.set_ylim(0, 1.05)
        ax.set_ylabel("success rate")
        ax.set_title("Success by planner")
        fig.tight_layout()
        path = os.path.join(out_dir, "success_by_planner.png")
        fig.savefig(path, dpi=140)
        plt.close(fig)
        written.append(path)
    return written


def summarize_run(run_dir: str) -> dict | None:
    manifest, trials = dmetrics.load_run(run_dir)
    if not trials:
        print(f"[WARN] {os.path.basename(run_dir)}: no trial records")
        return None
    summary = dmetrics.summarize(trials, manifest)
    run_id = summary["run_id"] or os.path.basename(run_dir)
    out_dir = args.out or os.path.join(RESULTS, "evaluation", run_id)
    os.makedirs(out_dir, exist_ok=True)
    if not args.no_figures:
        for path in figures(summary, out_dir):
            print(f"[INFO] figure: {os.path.relpath(path, PROJECT_ROOT)}")
    out_path = os.path.join(out_dir, "metrics.json")
    with open(out_path, "w") as f:
        json.dump(summary, f, indent=2)
    print(f"[INFO] {run_id}: {summary['n_success']}/{summary['n_trials']} succeeded "
          f"({summary['success_rate'] * 100:.0f} %), plan time median "
          f"{summary['plan_time_s'].get('median')} s -> {os.path.relpath(out_path, PROJECT_ROOT)}")
    if summary["failure_stages"]:
        print(f"[INFO]   failures: {summary['failure_stages']}")
    return summary


def main() -> None:
    if args.all:
        run_dirs = sorted(d for d in glob.glob(os.path.join(RESULTS, "experiments", "*"))
                          if os.path.isdir(d))
    elif args.run:
        run_dirs = [os.path.join(RESULTS, "experiments", args.run)]
    else:
        latest = dmetrics.latest_run(RESULTS)
        run_dirs = [latest] if latest else []

    if not run_dirs:
        print(f"[FAIL] no experiment runs found under {os.path.relpath(RESULTS, PROJECT_ROOT)}/experiments")
        print("[RESULT] FAIL")
        raise SystemExit(1)

    runs = {}
    for run_dir in run_dirs:
        if not os.path.isdir(run_dir):
            print(f"[FAIL] no such run: {run_dir}")
            print("[RESULT] FAIL")
            raise SystemExit(1)
        if summarize_run(run_dir) is not None:
            runs[os.path.basename(run_dir)] = dmetrics.load_run(run_dir)[1]

    if len(runs) > 1:
        rows = dmetrics.compare_runs(runs)
        print("\n[INFO] planner comparison")
        print(f"  {'run':46s} {'planner':12s} {'trials':>6s} {'success':>8s} {'plan med':>9s}")
        for r in rows:
            print(f"  {r['run_id'][:46]:46s} {str(r['planner'])[:12]:12s} {r['n_trials']:6d} "
                  f"{r['success_rate'] * 100:7.0f}% {str(r['plan_time_median_s']):>9s}")
        path = os.path.join(RESULTS, "evaluation", "comparison.json")
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, "w") as f:
            json.dump(rows, f, indent=2)
        print(f"[INFO] comparison: {os.path.relpath(path, PROJECT_ROOT)}")

    print("[RESULT] PASS")


if __name__ == "__main__":
    main()
