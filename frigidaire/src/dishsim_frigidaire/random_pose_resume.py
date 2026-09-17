"""Validate and carry evidence into a new run without modifying the prior run.

Only classified proposals count toward the logical sample target. Prior unresolved
attempts remain archived evidence and are retried with the original seeded pose.
No USD or Isaac imports occur here; sampling validation imports NumPy lazily.
"""
from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
import math
import os
from pathlib import Path
import shutil

from .random_pose_reports import UNRESOLVED_OUTCOMES, build_summary


PHYSICS_SOURCE_PATHS = tuple("frigidaire/src/dishsim_frigidaire/" + name + ".py" for name in (
    "random_pose_runtime", "random_poses", "random_pose_assets", "geometry", "asset",
    "loading", "load_validation", "tableware",
)) + ("src/dishsim/quats.py",)


def _digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _reject_constant(value):
    raise ValueError("Nonfinite JSON value: " + value)


def _read_json(path):
    return json.loads(Path(path).read_text(), parse_constant=_reject_constant)


def _read_rows(path):
    return [json.loads(line, parse_constant=_reject_constant)
            for line in Path(path).read_text().splitlines() if line.strip()]


def _finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def _pose(value, label, sampled=False):
    if not isinstance(value, dict):
        raise ValueError(f"Missing {label}")
    fields = [("position_m", 3), ("quaternion_xyzw", 4)]
    if sampled:
        fields.append(("bbox_center_m", 3))
    for key, size in fields:
        values = value.get(key)
        if not isinstance(values, list) or len(values) != size or not all(_finite(x) for x in values):
            raise ValueError(f"Invalid {label}.{key}")
    if abs(sum(x*x for x in value["quaternion_xyzw"]) - 1.) > 2e-5:
        raise ValueError(f"Unnormalized {label} quaternion")


def _relative_trace(prior, trial):
    value = trial.get("trace_file")
    if value is None:
        if trial["outcome"] == "accepted":
            raise ValueError(f"Accepted trial has no trace: {trial['trial_id']}")
        return None
    path = Path(value)
    if path.is_absolute() or not path.parts or path.parts[0] != "traces" or ".." in path.parts:
        raise ValueError(f"Invalid trace path: {value}")
    source = prior / path
    if prior not in source.resolve().parents or not source.is_file():
        raise ValueError(f"Missing or external trace: {value}")
    # A missing final hold cannot be presented as a reusable accepted trial.
    if trial["outcome"] == "accepted":
        rows = _read_rows(source)
        if not any(row.get("phase") == "final_hold" and row.get("dish") is not None for row in rows):
            raise ValueError(f"Accepted trace lacks a final hold: {value}")
    return path


@dataclass
class ResumeState:
    prior_dir: Path
    metadata: dict
    summary: dict
    classified_trials: list
    unresolved_trials: list
    completed_trials: dict
    provenance: dict
    prior_wall_seconds: float
    prior_evaluation_attempts: int
    traces: dict


def load_resume(prior_dir, current_metadata):
    """Load a finalized run after current inputs/source hashes have been collected."""
    prior = Path(prior_dir).resolve()
    metadata = _read_json(prior / "metadata.json")
    summary = _read_json(prior / "summary.json")
    if not metadata.get("finished_utc") or summary.get("status") in {None, "running"}:
        raise ValueError("Resume source must be a finalized run")
    if metadata.get("run_status") != summary["status"]:
        raise ValueError("Prior metadata and summary statuses disagree")
    for key in ("seed", "kinds", "racks", "samples_per_cell", "domains", "thresholds"):
        if key not in metadata or key not in current_metadata or metadata[key] != current_metadata[key]:
            raise ValueError(f"Resume requires matching {key}")
    old_assets = metadata.get("inputs", {}).get("asset_hashes")
    new_assets = current_metadata.get("inputs", {}).get("asset_hashes")
    if not isinstance(old_assets, dict) or not old_assets or old_assets != new_assets:
        raise ValueError("Resume requires matching input asset hashes")
    old_sources, new_sources = metadata.get("source_hashes", {}), current_metadata.get("source_hashes", {})
    for path in PHYSICS_SOURCE_PATHS:
        if not old_sources.get(path) or old_sources[path] != new_sources.get(path):
            raise ValueError(f"Resume physics source differs or is missing: {path}")
    differences = {path: {"prior": old_sources.get(path), "current": new_sources.get(path)}
                   for path in sorted(set(old_sources) | set(new_sources)) if old_sources.get(path) != new_sources.get(path)}
    trials = _read_rows(prior / "trials.jsonl")
    rebuilt = build_summary(trials, kinds=metadata["kinds"], racks=metadata["racks"],
                            samples_per_cell=metadata["samples_per_cell"], baselines=summary.get("baselines"),
                            status=summary["status"])
    if rebuilt["cells"] != summary.get("cells") or rebuilt["totals"] != summary.get("totals"):
        raise ValueError("Prior summary counts disagree with trial records")
    if summary.get("samples_per_cell") != metadata["samples_per_cell"]:
        raise ValueError("Prior planned counts disagree")
    traces = {}
    for trial in trials:
        canonical_id = "{}_{:05d}".format(trial["kind"] + "_" + trial["rack"], trial["sample_index"])
        if trial["trial_id"] != canonical_id:
            raise ValueError(f"Out-of-scope trial ID: {trial['trial_id']}")
        _pose(trial.get("sampled_pose"), "sampled_pose", sampled=True)
        if trial["outcome"] == "accepted":
            _pose(trial.get("settled_pose"), "settled_pose")
            _pose(trial.get("final_pose"), "final_pose")
            if summary.get("baselines", {}).get(trial["rack"], {}).get("result") != "PASS":
                raise ValueError("Accepted prior trial has no passed rack baseline")
        path = _relative_trace(prior, trial)
        if path is not None:
            if path in traces.values():
                raise ValueError(f"Multiple trials reference the same trace: {path}")
            traces[trial["trial_id"]] = path
    accepted = [trial for trial in trials if trial["outcome"] == "accepted"]
    if _read_json(prior / "accepted_poses.json").get("trials") != accepted:
        raise ValueError("Prior accepted replay records disagree with trials")
    classified = [trial for trial in trials if trial["outcome"] not in UNRESOLVED_OUTCOMES]
    unresolved = [trial for trial in trials if trial["outcome"] in UNRESOLVED_OUTCOMES]
    wall = metadata.get("cumulative_wall_seconds", metadata.get("wall_seconds"))
    if not _finite(wall) or wall < 0:
        raise ValueError("Prior cumulative elapsed wall time is missing or invalid")
    physical_attempts = metadata.get("cumulative_evaluation_attempts", len(trials))
    if not isinstance(physical_attempts, int) or isinstance(physical_attempts, bool) or physical_attempts < len(trials):
        raise ValueError("Prior physical attempt history count is invalid")
    files = [Path(name) for name in ("metadata.json", "summary.json", "trials.jsonl", "accepted_poses.json")]
    files += sorted(traces.values())
    provenance = {
        "prior_run_dir": str(prior), "prior_status": summary["status"],
        "prior_file_hashes": {str(path): _digest(prior / path) for path in files},
        "prior_logical_attempts": len(trials), "prior_physical_attempts": physical_attempts,
        "prior_evaluation_attempts": physical_attempts,
        "prior_cumulative_wall_seconds": float(wall),
        "carried_classified_count": len(classified), "archived_unresolved_count": len(unresolved),
        "prior_outcome_counts": summary["totals"], "allowed_source_differences": differences,
        "physics_sources_validated": list(PHYSICS_SOURCE_PATHS),
        "history_directory": "resume_history/" + prior.name,
        "policy": "Reuse full classified rows; retry unresolved proposals with the original seeded pose. "
                  "CLI, scheduling, reporting and resume-helper source changes are recorded and allowed; "
                  "physics sources, inputs, thresholds, domains and runtime settings must match.",
    }
    return ResumeState(prior, metadata, summary, classified, unresolved,
                       {trial["trial_id"]: trial for trial in classified}, provenance, float(wall), physical_attempts, traces)


def validate_runtime(state, current_runtime):
    """Call after initializing the backend and collecting actual runtime versions."""
    previous = state.metadata.get("runtime")
    if not isinstance(previous, dict) or not previous or previous != current_runtime:
        raise ValueError("Resume requires identical recorded runtime settings and versions")


def validate_sample_stream(state, points_by_kind, domains):
    """Verify classified AND unresolved proposals against their original seeded draws."""
    from .random_poses import cell_generators, round_robin_cells, sample_pose
    generators = cell_generators(state.metadata["seed"])
    prior = {trial["trial_id"]: trial for trial in state.classified_trials + state.unresolved_trials}
    for kind, rack, index in round_robin_cells(state.metadata["samples_per_cell"]):
        pose = sample_pose(generators[(kind, rack)], points_by_kind[kind], domains["sampling_bounds_by_rack"][rack])
        trial_id = "{}_{}_{:05d}".format(kind, rack, index)
        if trial_id not in prior:
            continue
        recorded = prior[trial_id]["sampled_pose"]
        for key in ("position_m", "quaternion_xyzw", "bbox_center_m"):
            if len(recorded[key]) != len(pose[key]) or any(abs(a-b) > 1e-12 for a, b in zip(recorded[key], pose[key])):
                raise ValueError(f"Prior seeded sample differs: {trial_id}.{key}")


def _copy_exclusive(source, destination):
    destination.parent.mkdir(parents=True, exist_ok=True)
    with Path(source).open("rb") as source_stream, destination.open("xb") as output:
        shutil.copyfileobj(source_stream, output)
        output.flush()
        os.fchmod(output.fileno(), 0o644)
        os.fsync(output.fileno())


def seed_report(report, state):
    """Archive prior evidence and copy classified rows once into an empty new report."""
    destination = Path(report.out_dir).resolve()
    if destination == state.prior_dir or state.prior_dir in destination.parents:
        raise ValueError("Resume output must be outside the prior run directory")
    if report.trials:
        raise ValueError("Resume can seed only a report with no recorded trials")
    if (tuple(report.kinds), tuple(report.racks), report.samples_per_cell) != (
            tuple(state.metadata["kinds"]), tuple(state.metadata["racks"]), state.metadata["samples_per_cell"]):
        raise ValueError("New report configuration differs from resume source")
    history = destination / state.provenance["history_directory"]
    if history.exists():
        raise FileExistsError(f"Resume history already exists: {history}")
    for relative, digest in state.provenance["prior_file_hashes"].items():
        if _digest(state.prior_dir / relative) != digest:
            raise ValueError(f"Prior evidence changed after validation: {relative}")
    # Copies retain source bytes. Unresolved traces live only in history so that
    # retrying the same trial ID can create its new trace with open('x').
    for relative in state.provenance["prior_file_hashes"]:
        _copy_exclusive(state.prior_dir / relative, history / relative)
    previous_history = state.prior_dir / "resume_history"
    if previous_history.is_dir():
        for source in sorted(previous_history.rglob("*")):
            if source.is_symlink():
                raise ValueError("Prior resume history must not contain symlinks")
            if source.is_file():
                _copy_exclusive(source, history / source.relative_to(state.prior_dir))
    with (history / "unresolved_trials.jsonl").open("x", encoding="utf-8") as stream:
        for trial in state.unresolved_trials:
            stream.write(json.dumps(trial, sort_keys=True, allow_nan=False) + "\n")
        stream.flush()
        os.fsync(stream.fileno())
    for trial in state.classified_trials:
        trace = state.traces.get(trial["trial_id"])
        if trace is not None:
            _copy_exclusive(state.prior_dir / trace, destination / trace)
        report.record_trial(trial)
    return dict(state.provenance)
