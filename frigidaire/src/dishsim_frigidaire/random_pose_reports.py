"""Durable records and honest accounting for the random single-dish experiment.

This module has no Isaac/Kit dependency. Poses are recorded as position in metres
and quaternion in xyzw order. Sampled poses are rack-local; observed poses use
the simulation world frame. Counts describe sampled proposals, not distinct
stable arrangements or the number of possible continuous poses.
"""
from __future__ import annotations

from collections import Counter
import csv
from io import StringIO
import json
import math
import os
from pathlib import Path
import tempfile


KINDS = ("dinner_plate", "bowl", "mug")
RACKS = ("LowerRack", "UpperRack")
OUTCOMES = (
    "accepted", "initial_collision", "settle_timeout", "closure_failure",
    "lost_support", "outside_dishwasher", "simulation_error", "interrupted",
)
UNRESOLVED_OUTCOMES = {"simulation_error", "interrupted"}
POSE_FRAMES = {"sampled_pose": "rack_local", "settled_pose": "world", "final_pose": "world"}


def _json_text(value):
    return json.dumps(value, indent=2, sort_keys=True, allow_nan=False) + "\n"


def _atomic_text(path, text):
    """Replace a report only after all of its content has reached the filesystem."""
    path = Path(path)
    name = None
    try:
        with tempfile.NamedTemporaryFile(mode="w", encoding="utf-8", dir=path.parent,
                                         prefix=f".{path.name}.", delete=False) as stream:
            name = stream.name
            stream.write(text)
            stream.flush()
            # Kit runs as root in the shared container. Reports must remain
            # readable by the host user after replacing the 0600 temporary file.
            os.fchmod(stream.fileno(), 0o644)
            os.fsync(stream.fileno())
        os.replace(name, path)
    finally:
        if name is not None and os.path.exists(name):
            os.unlink(name)


def _counts(trials, planned):
    counts = Counter(t["outcome"] for t in trials)
    attempted = len(trials)
    unresolved = sum(counts[name] for name in UNRESOLVED_OUTCOMES)
    classified = attempted - unresolved
    return {
        "planned": planned,
        "attempted": attempted,
        # A simulation error is not a completed physical trial.
        "completed": classified,
        "classified": classified,
        "unresolved": unresolved,
        "unattempted": max(0, planned - attempted),
        **{outcome: counts[outcome] for outcome in OUTCOMES},
        "confirmed_successes_per_attempt": counts["accepted"] / attempted if attempted else None,
    }


def build_summary(trials, *, kinds=KINDS, racks=RACKS, samples_per_cell=100,
                  baselines=None, status="complete"):
    """Summarize every proposed sample, leaving numerical/budget failures unresolved."""
    if not isinstance(samples_per_cell, int) or isinstance(samples_per_cell, bool) or samples_per_cell < 1:
        raise ValueError("samples_per_cell must be a positive integer")
    trials = list(trials)
    kinds, racks = tuple(kinds), tuple(racks)
    if not kinds or not racks or len(set(kinds)) != len(kinds) or len(set(racks)) != len(racks):
        raise ValueError("kinds and racks must be nonempty and unique")
    allowed = {(kind, rack) for kind in kinds for rack in racks}
    identities, sample_keys = set(), set()
    for trial in trials:
        if (trial["kind"], trial["rack"]) not in allowed or trial["outcome"] not in OUTCOMES:
            raise ValueError("unknown kind, rack, or outcome")
        index = trial["sample_index"]
        if not isinstance(index, int) or isinstance(index, bool) or not 0 <= index < samples_per_cell:
            raise ValueError("sample_index must be within the planned cell")
        sample_key = (trial["kind"], trial["rack"], index)
        if trial["trial_id"] in identities or sample_key in sample_keys:
            raise ValueError("duplicate trial ID or cell sample index")
        identities.add(trial["trial_id"])
        sample_keys.add(sample_key)
    baselines = {rack: dict((baselines or {}).get(rack, {"result": "NOT_RUN"})) for rack in racks}
    cells = []
    for kind in kinds:
        for rack in racks:
            rows = [trial for trial in trials if trial["kind"] == kind and trial["rack"] == rack]
            baseline_result = baselines[rack].get("result", "NOT_RUN")
            cells.append({
                "kind": kind, "rack": rack, "baseline_result": baseline_result,
                "blocked": baseline_result in {"FAIL", "ERROR"},
                **_counts(rows, samples_per_cell),
            })
    return {
        "schema_version": 1,
        "status": status,
        "simulation_run": bool(trials),
        "pose_frames": dict(POSE_FRAMES),
        "samples_per_cell": samples_per_cell,
        "baselines": baselines,
        "cells": cells,
        "totals": {**_counts(trials, samples_per_cell * len(cells)),
                   "blocked_cells": sum(cell["blocked"] for cell in cells)},
        "interpretation": (
            "Confirmed successes / attempted includes every generated proposal, including initial "
            "collisions. Simulation errors and interrupted trials are unresolved. Unattempted "
            "samples are not impossible placements. Completed/classified counts exclude unresolved "
            "trials. Counts are not unique arrangements or a finite total of continuous poses."
        ),
    }


def _validate_pose(pose, name):
    if pose is None:
        return
    for field, size in (("position_m", 3), ("quaternion_xyzw", 4)):
        values = pose.get(field)
        if not isinstance(values, (list, tuple)) or len(values) != size:
            raise ValueError(f"{name}.{field} must contain {size} numbers")
        if any(isinstance(value, bool) or not isinstance(value, (int, float))
               or not math.isfinite(value) for value in values):
            raise ValueError(f"{name}.{field} must contain finite numbers")
    norm = math.sqrt(sum(value * value for value in pose["quaternion_xyzw"]))
    if abs(norm - 1.0) > 1e-5:
        raise ValueError(f"{name}.quaternion_xyzw must be normalized")


def _markdown(summary):
    totals = summary["totals"]
    lines = ["# Random dish pose experiment", "", f"Run status: **{summary['status']}**.", ""]
    if not summary["simulation_run"]:
        lines.extend(["**No random-pose trials ran. No placement success was measured.**", ""])
    lines.extend([
        f"Confirmed successes: **{totals['accepted']} / {totals['attempted']} attempted proposals**. "
        f"{totals['classified']} classified, {totals['unresolved']} unresolved, "
        f"{totals['unattempted']} unattempted out of {totals['planned']} planned.", "",
        "| Dish | Rack | Baseline | Attempted | Accepted | Confirmed / attempted | Unresolved | Unattempted |",
        "| --- | --- | --- | ---: | ---: | ---: | ---: | ---: |",
    ])
    for cell in summary["cells"]:
        rate = cell["confirmed_successes_per_attempt"]
        rate_label = "—" if rate is None else f"{100 * rate:.1f}%"
        baseline = cell["baseline_result"] + (" (blocked)" if cell["blocked"] else "")
        lines.append(f"| {cell['kind']} | {cell['rack']} | {baseline} | {cell['attempted']} | "
                     f"{cell['accepted']} | {rate_label} | {cell['unresolved']} | {cell['unattempted']} |")
    lines.extend(["", "Outcome counts (all proposals):", ""])
    lines.extend(f"- {outcome}: {totals[outcome]}" for outcome in OUTCOMES)
    lines.extend([
        "", summary["interpretation"], "",
        "Sampled poses use rack-local coordinates. Settled and final poses use world coordinates. "
        "Positions are metres; quaternion component order is xyzw. Accepted poses are in "
        "`accepted_poses.json`; full trial records are in `trials.jsonl`. Replay requires the "
        "matching asset and runtime settings recorded in `metadata.json`.", "",
        "Door closure and multi-dish capacity are outside this experiment.", "",
    ])
    if summary.get("plots"):
        plots = summary["plots"]
        lines.extend([f"Plots: {plots['status']}.", ""])
        if plots.get("reason"):
            lines.extend([plots["reason"], ""])
        for filename in plots.get("files", []):
            lines.extend([f"![{filename}]({filename})", ""])
    return "\n".join(lines)


def _plots(directory, trials, kinds, racks):
    if not trials:
        return {"status": "not_run", "reason": "No sampled poses to plot.", "files": []}
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError as exc:
        return {"status": "unavailable", "reason": str(exc), "files": []}
    files = []
    figures = []
    try:
        # One row per dish/rack cell; all three position components remain visible.
        cells = [(kind, rack) for kind in kinds for rack in racks]
        position_fig, position_axes = plt.subplots(len(cells), 2, figsize=(10, 3 * len(cells)), squeeze=False)
        orientation_fig, orientation_axes = plt.subplots(len(cells), 2, figsize=(10, 3 * len(cells)), squeeze=False)
        figures = [position_fig, orientation_fig]
        groups = (("accepted", "#27804c"), ("rejected", "#a5a5a5"), ("unresolved", "#d58a00"))
        for row, (kind, rack) in enumerate(cells):
            rows = [t for t in trials if t["kind"] == kind and t["rack"] == rack and t.get("sampled_pose")]
            for label, color in groups:
                def category(trial):
                    return ("accepted" if trial["outcome"] == "accepted" else
                            "unresolved" if trial["outcome"] in UNRESOLVED_OUTCOMES else "rejected")
                selected = [t["sampled_pose"] for t in rows if category(t) == label]
                for col, (x, y) in enumerate(((0, 1), (0, 2))):
                    position_axes[row, col].scatter(
                        [p["position_m"][x] for p in selected], [p["position_m"][y] for p in selected],
                        label=label, color=color, s=15, alpha=.7)
                # Canonical hemisphere makes the double quaternion cover explicit and consistent.
                quaternions = [p["quaternion_xyzw"] for p in selected]
                quaternions = [[v * (-1 if q[3] < 0 else 1) for v in q] for q in quaternions]
                for col, (x, y) in enumerate(((0, 1), (2, 3))):
                    orientation_axes[row, col].scatter(
                        [q[x] for q in quaternions], [q[y] for q in quaternions],
                        label=label, color=color, s=15, alpha=.7)
            for col in range(2):
                ax = position_axes[row, col]
                ax.set(title=f"{kind} / {rack}", xlabel="rack-local x (m)",
                       ylabel=f"rack-local {'y' if col == 0 else 'z'} (m)")
                ax.grid(alpha=.2)
                ax.legend(fontsize="small")
                ax = orientation_axes[row, col]
                ax.set(title=f"{kind} / {rack}", xlabel="qx" if col == 0 else "qz",
                       ylabel="qy" if col == 0 else "qw", xlim=(-1, 1), ylim=(-1, 1))
                ax.grid(alpha=.2)
                ax.legend(fontsize="small")
        position_fig.suptitle("Sampled rack-local positions, colored by physical trial outcome")
        orientation_fig.suptitle("Sampled orientations: quaternion projections (q and −q equivalent; qw ≥ 0)")
        for figure, filename in zip(figures, ("sampled_positions.png", "sampled_orientations.png")):
            figure.tight_layout(rect=(0, 0, 1, .98))
            figure.savefig(directory / filename, dpi=130)
            files.append(filename)
        return {"status": "complete", "files": files}
    except Exception as exc:
        return {"status": "error", "reason": f"{type(exc).__name__}: {exc}", "files": files}
    finally:
        for figure in figures:
            plt.close(figure)


class ExperimentReport:
    """Own a new output directory and durably append trial results.

    ``finalize`` may be repeated within one instance to refresh derived reports.
    Reopening a prior run directory is intentionally prohibited. Caller metadata
    should include the asset hashes, runtime settings, seed, geometry bounds, and
    run timestamps; those values are not guessed by this reporting helper.
    """

    def __init__(self, out_dir, *, kinds=KINDS, racks=RACKS, samples_per_cell=100):
        self.out_dir = Path(out_dir)
        self.kinds, self.racks = tuple(kinds), tuple(racks)
        self.samples_per_cell = samples_per_cell
        build_summary([], kinds=self.kinds, racks=self.racks, samples_per_cell=samples_per_cell)
        self.out_dir.mkdir(parents=True, exist_ok=True)
        if any(self.out_dir.iterdir()):
            raise FileExistsError(f"Experiment output directory must be empty: {self.out_dir}")
        self._stream = (self.out_dir / "trials.jsonl").open("x", encoding="utf-8")
        self.trials = []

    def write_metadata(self, metadata):
        _atomic_text(self.out_dir / "metadata.json", _json_text({
            **metadata, "schema_version": 1, "pose_frames": dict(POSE_FRAMES),
            "kinds": list(self.kinds), "racks": list(self.racks),
            "samples_per_cell": self.samples_per_cell,
        }))

    def record_trial(self, trial):
        if self._stream.closed:
            raise ValueError("Experiment report is closed")
        # JSON round trip both validates serialization and isolates caller mutations.
        trial = json.loads(json.dumps(trial, allow_nan=False))
        for name in POSE_FRAMES:
            _validate_pose(trial.get(name), name)
        if trial.get("sampled_pose") is None:
            raise ValueError("Every attempted proposal requires a sampled_pose")
        if trial["outcome"] == "accepted" and any(trial.get(name) is None for name in POSE_FRAMES):
            raise ValueError("Accepted trials require sampled, settled, and final poses")
        summary = build_summary([*self.trials, trial], kinds=self.kinds, racks=self.racks,
                                samples_per_cell=self.samples_per_cell, status="running")
        self._stream.write(json.dumps(trial, sort_keys=True, allow_nan=False) + "\n")
        self._stream.flush()
        os.fsync(self._stream.fileno())
        self.trials.append(trial)
        _atomic_text(self.out_dir / "summary.json", _json_text(summary))

    def finalize(self, *, baselines=None, status="complete", extra=None, plots=True):
        summary = build_summary(self.trials, kinds=self.kinds, racks=self.racks,
                                samples_per_cell=self.samples_per_cell, baselines=baselines, status=status)
        if extra is not None:
            summary["extra"] = extra
        summary["plots"] = (_plots(self.out_dir, self.trials, self.kinds, self.racks) if plots else
                            {"status": "disabled", "files": []})
        _atomic_text(self.out_dir / "summary.json", _json_text(summary))
        stream = StringIO(newline="")
        writer = csv.DictWriter(stream, fieldnames=list(summary["cells"][0]))
        writer.writeheader()
        writer.writerows(summary["cells"])
        _atomic_text(self.out_dir / "summary.csv", stream.getvalue())
        _atomic_text(self.out_dir / "accepted_poses.json", _json_text({
            "schema_version": 1, "pose_frames": dict(POSE_FRAMES),
            "metadata_file": "metadata.json",
            "trials": [trial for trial in self.trials if trial["outcome"] == "accepted"],
        }))
        _atomic_text(self.out_dir / "report.md", _markdown(summary))
        return summary

    def close(self):
        self._stream.close()

    def __enter__(self):
        return self

    def __exit__(self, *_exc):
        self.close()
