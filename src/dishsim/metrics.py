# Copyright (c) 2026, dishsim project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Aggregate experiment results into metrics (Kit-free, pure data).

Reads only what an experiment wrote to disk — trial JSONs and the run manifest — so metrics
can be recomputed at any time, on any machine, without a simulator. Kept as a library so the
aggregation is unit-testable and the CLI stays a thin wrapper.

Which numbers can live here and which cannot: geometric outcomes are recomputable from a
recorded pose, but the contact-derived gates (a trial "succeeds" only if no unexpected body
force exceeds the threshold) need live force sensors and therefore stay in the experiment.
Evaluation aggregates those verdicts; it never re-derives them.
"""

import glob
import json
import os
import statistics
from collections import Counter


def load_run(run_dir: str) -> tuple[dict, list[dict]]:
    """Load a run's manifest and every trial record it produced.

    Args:
        run_dir: A ``results/experiments/<run_id>`` directory.

    Returns:
        ``(manifest, trials)``; the manifest is ``{}`` if the run predates manifests.
    """
    manifest_path = os.path.join(run_dir, "manifest.json")
    manifest = json.load(open(manifest_path)) if os.path.exists(manifest_path) else {}
    trials = []
    for path in sorted(glob.glob(os.path.join(run_dir, "trials", "*.json"))):
        with open(path) as f:
            trials.append(json.load(f))
    return manifest, trials


def latest_run(results_root: str) -> str | None:
    """The most recent run directory, from the ``LATEST`` pointer or by mtime."""
    experiments = os.path.join(results_root, "experiments")
    pointer = os.path.join(experiments, "LATEST")
    if os.path.exists(pointer):
        run_id = open(pointer).read().strip()
        candidate = os.path.join(experiments, run_id)
        if os.path.isdir(candidate):
            return candidate
    runs = [d for d in glob.glob(os.path.join(experiments, "*")) if os.path.isdir(d)]
    return max(runs, key=os.path.getmtime) if runs else None


def _stats(values: list[float]) -> dict:
    values = [v for v in values if v is not None]
    if not values:
        return {"n": 0}
    out = {
        "n": len(values),
        "mean": round(statistics.fmean(values), 4),
        "median": round(statistics.median(values), 4),
        "min": round(min(values), 4),
        "max": round(max(values), 4),
    }
    if len(values) > 1:
        out["stdev"] = round(statistics.stdev(values), 4)
    return out


def summarize(trials: list[dict], manifest: dict | None = None) -> dict:
    """Aggregate trial records into a metrics dict.

    Args:
        trials: Trial records (as written by the experiment runner).
        manifest: Optional run manifest, echoed into the result for provenance.

    Returns:
        Metrics: overall success, per-planner / per-object / per-slot breakdowns, planning and
        execution statistics, the failure taxonomy, and placement-error statistics.
    """
    n = len(trials)
    n_ok = sum(bool(t.get("success")) for t in trials)
    by_slot: dict[str, dict] = {}
    for t in trials:
        key = str(t.get("slot"))
        entry = by_slot.setdefault(key, {"trials": 0, "success": 0})
        entry["trials"] += 1
        entry["success"] += bool(t.get("success"))

    def group(field: str) -> dict:
        out: dict[str, dict] = {}
        for t in trials:
            value = t.get(field)
            if isinstance(value, dict):  # e.g. the planner provenance block
                value = value.get("name")
            entry = out.setdefault(str(value), {"trials": 0, "success": 0})
            entry["trials"] += 1
            entry["success"] += bool(t.get("success"))
        for entry in out.values():
            entry["rate"] = round(entry["success"] / entry["trials"], 4)
        return out

    lateral = [t["final_pose_err"]["lateral_m"] for t in trials
               if isinstance(t.get("final_pose_err"), dict) and "lateral_m" in t["final_pose_err"]]
    tilt = [t["final_pose_err"]["tilt_deg"] for t in trials
            if isinstance(t.get("final_pose_err"), dict) and "tilt_deg" in t["final_pose_err"]]

    return {
        "run_id": (manifest or {}).get("run_id"),
        "n_trials": n,
        "n_success": n_ok,
        "success_rate": round(n_ok / n, 4) if n else 0.0,
        "by_planner": group("planner"),
        "by_object": group("object"),
        "by_scenario": group("scenario"),
        "by_slot": {k: {**v, "rate": round(v["success"] / v["trials"], 4)}
                    for k, v in sorted(by_slot.items(), key=lambda kv: int(kv[0]))},
        "plan_time_s": _stats([t.get("plan_time_s") for t in trials]),
        "path_len_rad": _stats([t.get("path_len_rad") for t in trials]),
        "exec_steps": _stats([t.get("exec_steps") for t in trials]),
        "placement_lateral_m": _stats(lateral),
        "placement_tilt_deg": _stats(tilt),
        "failure_stages": dict(Counter(t.get("failure_stage") for t in trials
                                       if t.get("failure_stage"))),
        "planner": (manifest or {}).get("planner"),
        "config_hash": (manifest or {}).get("config_hash"),
    }


def compare_runs(runs: dict[str, list[dict]]) -> list[dict]:
    """One summary row per run — the planner-vs-planner comparison table.

    Args:
        runs: ``{run_id: trials}``.

    Returns:
        Rows sorted by success rate (descending), then by median plan time.
    """
    rows = []
    for run_id, trials in runs.items():
        summary = summarize(trials)
        planners = sorted(summary["by_planner"])
        rows.append({
            "run_id": run_id,
            "planner": planners[0] if len(planners) == 1 else ",".join(planners),
            "n_trials": summary["n_trials"],
            "success_rate": summary["success_rate"],
            "plan_time_median_s": summary["plan_time_s"].get("median"),
            "path_len_median_rad": summary["path_len_rad"].get("median"),
        })
    return sorted(rows, key=lambda r: (-r["success_rate"], r["plan_time_median_s"] or 0.0))
