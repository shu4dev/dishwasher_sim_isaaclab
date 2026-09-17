"""Audit and visualize one finished random-pose run without starting Isaac.

Usage: scripts/run_py.sh frigidaire/scripts/evaluation/frigidaire_random_pose_report.py
       --run-dir results/random_poses/frigidaire/<run>

Writes analysis.json, analysis.md and outcome_breakdown.png after the saved proposal
counts agree. This is an audit of saved evidence, not a new physical validation.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "src"))
from dishsim_frigidaire.random_pose_reports import OUTCOMES, build_summary, _atomic_text
from dishsim_frigidaire.random_pose_resume import PHYSICS_SOURCE_PATHS


def _finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def _json(path):
    with Path(path).open() as stream:
        return json.load(stream, parse_constant=lambda value: (_ for _ in ()).throw(ValueError(value)))


def _digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _child(directory, relative):
    path = Path(relative)
    if path.is_absolute() or ".." in path.parts:
        raise ValueError(f"Evidence path escapes directory: {relative}")
    result = (directory / path).resolve()
    if directory.resolve() not in result.parents:
        raise ValueError(f"Evidence path escapes directory: {relative}")
    return result


def _rows(path):
    return [json.loads(line) for line in path.read_text().splitlines() if line.strip()]


def audit_resume(directory, metadata, summary, trials):
    """Verify inherited evidence and count logical proposals separately from evaluations."""
    wall = metadata["wall_seconds"]
    accounting = {"session_wall_seconds": wall, "cumulative_wall_seconds": wall,
                  "inherited_classified_trials": 0, "new_evaluation_attempts": len(trials),
                  "cumulative_evaluation_attempts": len(trials), "logical_attempted": len(trials),
                  "logical_planned": summary["totals"]["planned"], "archived_unresolved_attempts": 0}
    resume = metadata.get("resume")
    audit = {"status": "NOT_APPLICABLE"}
    if resume is not None:
        if not isinstance(resume, dict) or not isinstance(resume.get("applied"), bool):
            raise ValueError("Resume application state is missing")
        if not resume["applied"]:
            if trials or metadata.get("inherited_classified_trials", 0):
                raise ValueError("Unapplied resume contains unverified carried trials")
            audit = {"status": "NOT_APPLIED"}
            accounting["cumulative_wall_seconds"] += resume["prior_cumulative_wall_seconds"]
            accounting["cumulative_evaluation_attempts"] += resume["prior_evaluation_attempts"]
        else:
            history = _child(directory, resume["history_directory"])
            if not history.is_dir():
                raise ValueError("Applied resume history is missing")
            hashes = resume.get("prior_file_hashes")
            required = {"metadata.json", "summary.json", "trials.jsonl", "accepted_poses.json"}
            if not isinstance(hashes, dict) or not required.issubset(hashes):
                raise ValueError("Resume history provenance hashes are incomplete")
            for relative, expected in hashes.items():
                path = _child(history, relative)
                if not path.is_file() or _digest(path) != expected:
                    raise ValueError(f"Resume history hash mismatch: {relative}")
            old_meta, old_summary = _json(history / "metadata.json"), _json(history / "summary.json")
            old_trials = _rows(history / "trials.jsonl")
            if not old_meta.get("finished_utc") or old_meta.get("run_status") != old_summary.get("status"):
                raise ValueError("Archived prior run was not finalized consistently")
            rebuilt = build_summary(old_trials, kinds=old_meta["kinds"], racks=old_meta["racks"],
                                    samples_per_cell=old_meta["samples_per_cell"], baselines=old_summary.get("baselines"),
                                    status=old_summary["status"])
            if rebuilt["totals"] != old_summary.get("totals") or rebuilt["cells"] != old_summary.get("cells"):
                raise ValueError("Archived prior summary counts disagree")
            if _json(history / "accepted_poses.json").get("trials") != [t for t in old_trials if t["outcome"] == "accepted"]:
                raise ValueError("Archived accepted replay rows disagree")
            if old_meta.get("resume") is not None:
                audit_resume(history, old_meta, old_summary, old_trials)
            for key in ("seed", "kinds", "racks", "samples_per_cell", "domains", "thresholds", "runtime"):
                if key not in old_meta or old_meta[key] != metadata.get(key):
                    raise ValueError(f"Resume changed {key}")
            old_assets = old_meta.get("inputs", {}).get("asset_hashes")
            if not isinstance(old_assets, dict) or not old_assets or old_assets != metadata.get("inputs", {}).get("asset_hashes"):
                raise ValueError("Resume changed asset hashes")
            old_sources, sources = old_meta.get("source_hashes", {}), metadata.get("source_hashes", {})
            if set(resume.get("physics_sources_validated", [])) != set(PHYSICS_SOURCE_PATHS):
                raise ValueError("Resume physics source validation coverage is incomplete")
            for path in PHYSICS_SOURCE_PATHS:
                if not old_sources.get(path) or old_sources[path] != sources.get(path):
                    raise ValueError(f"Resume physics source differs: {path}")
            differences = {path: {"prior": old_sources.get(path), "current": sources.get(path)}
                           for path in sorted(set(old_sources) | set(sources)) if old_sources.get(path) != sources.get(path)}
            if differences != resume.get("allowed_source_differences"):
                raise ValueError("Recorded resume source differences disagree")
            classified = [t for t in old_trials if t["outcome"] not in {"interrupted", "simulation_error"}]
            unresolved = [t for t in old_trials if t["outcome"] in {"interrupted", "simulation_error"}]
            if _rows(history / "unresolved_trials.jsonl") != unresolved:
                raise ValueError("Archived unresolved attempts differ from prior records")
            current = {t["trial_id"]: t for t in trials}
            for trial in classified:
                if current.get(trial["trial_id"]) != trial:
                    raise ValueError(f"Inherited classified row changed or missing: {trial['trial_id']}")
                if trial.get("trace_file"):
                    relative = trial["trace_file"]
                    if relative not in hashes or _digest(_child(directory, relative)) != hashes[relative]:
                        raise ValueError(f"Inherited trace bytes changed: {relative}")
            for trial in unresolved:
                relative = trial.get("trace_file")
                if relative and relative not in hashes:
                    raise ValueError(f"Archived unresolved trace lacks provenance: {relative}")
                retry = current.get(trial["trial_id"])
                if retry:
                    for key, length in (("position_m", 3), ("quaternion_xyzw", 4), ("bbox_center_m", 3)):
                        before, after = trial.get("sampled_pose", {}).get(key), retry.get("sampled_pose", {}).get(key)
                        if not isinstance(before, list) or not isinstance(after, list) or len(before) != length or len(after) != length:
                            raise ValueError("Retried proposal has missing sampled pose evidence")
                        if any(not _finite(a) or not _finite(b) or abs(a-b) > 1e-12 for a, b in zip(before, after)):
                            raise ValueError(f"Retried proposal changed seeded pose: {trial['trial_id']}")
            previous_wall = old_meta.get("cumulative_wall_seconds", old_meta.get("wall_seconds"))
            previous_attempts = old_meta.get("cumulative_evaluation_attempts", len(old_trials))
            if not _finite(previous_wall) or previous_wall < 0 or not isinstance(previous_attempts, int) or previous_attempts < len(old_trials):
                raise ValueError("Archived cumulative time or attempt count is invalid")
            expected_provenance = {"prior_status": old_summary["status"], "prior_logical_attempts": len(old_trials),
                                   "prior_evaluation_attempts": previous_attempts, "prior_cumulative_wall_seconds": previous_wall,
                                   "carried_classified_count": len(classified), "archived_unresolved_count": len(unresolved)}
            if any(resume.get(key) != value for key, value in expected_provenance.items()):
                raise ValueError("Resume provenance counts or wall time disagree with archive")
            if resume.get("prior_outcome_counts") != old_summary["totals"]:
                raise ValueError("Resume prior outcome counts disagree with archive")
            accounting.update(cumulative_wall_seconds=wall+previous_wall, inherited_classified_trials=len(classified),
                              new_evaluation_attempts=len(trials)-len(classified),
                              cumulative_evaluation_attempts=previous_attempts+len(trials)-len(classified),
                              archived_unresolved_attempts=previous_attempts-len(classified))
            audit = {"status": "PASS", "history_directory": resume["history_directory"],
                     "verified_prior_files": len(hashes), "inherited_rows_verified": len(classified),
                     "archived_unresolved_verified": len(unresolved),
                     "allowed_source_differences": differences}
    for key in ("cumulative_wall_seconds", "inherited_classified_trials", "new_evaluation_attempts", "cumulative_evaluation_attempts"):
        for document in (metadata, summary.get("extra", {})):
            if key not in document:
                if resume is not None:
                    raise ValueError(f"Resume accounting is missing {key}")
                continue
            value = document[key]
            if not _finite(value) or abs(value-accounting[key]) > (1e-6 if key.endswith("seconds") else 0):
                raise ValueError(f"Resume accounting disagrees: {key}")
    return audit, accounting


def _trace(directory, trial):
    filename = trial.get("trace_file")
    if not filename:
        return []
    path = (directory / filename).resolve()
    if directory.resolve() not in path.parents:
        raise ValueError(f"Trace escapes run directory: {filename}")
    if not path.is_file():
        return []
    return [json.loads(line) for line in path.read_text().splitlines() if line.strip()]


def physical_entry(trial, trace):
    """Return recorded evidence of entering dish physics, not just proposing a pose."""
    if trial["outcome"] == "initial_collision":
        return {"entered": False, "last_phase": "initial_collision"}
    active = [row for row in trace if row.get("dish") is not None
              and row.get("phase") in {"dish_settle", "loaded_retraction", "final_hold"}]
    measured = any(trial.get(key) is not None for key in
                   ("settle", "settled_pose", "rack_motion", "final_hold", "final_pose"))
    if active or measured:
        return {"entered": True, "last_phase": trace[-1].get("phase", "unknown") if trace else "saved_measurements"}
    # A reset-only trace can be followed by interruption before the next trace
    # sample. Do not infer that dish physics either started or did not start.
    return {"entered": None, "last_phase": trace[-1].get("phase", "unknown") if trace else "unrecorded"}


def audit_accepted(trial, metadata, trace):
    """Check recorded acceptance gates; missing measurements cannot pass an audit."""
    missing, failed = [], []
    limits = metadata.get("thresholds", {})

    def number(value, label):
        if not _finite(value):
            missing.append(label)
            return None
        return value

    def limit(key):
        value = number(limits.get(key), "thresholds." + key)
        if value is not None and value <= 0:
            missing.append("positive thresholds." + key)
            return None
        return value

    def less(value, threshold, label, inclusive=False):
        value = number(value, label)
        if value is not None and threshold is not None:
            if value < 0 or (value > threshold if inclusive else value >= threshold):
                failed.append(label)
        return value

    def true(value, label):
        if value is None or not isinstance(value, bool):
            missing.append(label)
        elif not value:
            failed.append(label)

    thresholds = {key: limit(key) for key in (
        "root_position_span_m", "quaternion_span_deg", "peak_mesh_point_speed_m_s",
        "peak_penetration_m", "median_max_penetration_m", "rack_endpoint_m",
        "containment_tolerance_m", "rest_window_s", "physics_dt_s",
    )}
    for name in ("sampled_pose", "settled_pose", "final_pose"):
        pose = trial.get(name) or {}
        for field, length in (("position_m", 3), ("quaternion_xyzw", 4)):
            values = pose.get(field)
            if not isinstance(values, list) or len(values) != length or not all(_finite(v) for v in values):
                missing.append(name + "." + field)
            elif field == "quaternion_xyzw" and abs(sum(v*v for v in values) - 1) > 2e-5:
                failed.append(name + ".normalized_quaternion")
    for phase in ("settle", "final_hold"):
        hold = trial.get(phase) or {}
        for key in ("settled", "endpoints_ok", "contacts_ok", "basket_supported", "dish_supported"):
            true(hold.get(key), phase + "." + key)
        for key in ("peak_penetration_m", "median_max_penetration_m"):
            less(hold.get(key), thresholds[key], phase + "." + key)
        for body in (trial["kind"], "SilverwareBasket"):
            metrics = hold.get("motion", {}).get(body, {})
            for key in ("root_position_span_m", "quaternion_span_deg", "peak_mesh_point_speed_m_s"):
                less(metrics.get(key), thresholds[key], phase + "." + body + "." + key)
            duration = number(metrics.get("sample_duration_s"), phase + "." + body + ".sample_duration_s")
            if duration is not None and thresholds["rest_window_s"] is not None and duration < thresholds["rest_window_s"]:
                failed.append(phase + "." + body + ".rest_window")
            count = number(metrics.get("sample_count"), phase + "." + body + ".sample_count")
            if count is not None and thresholds["rest_window_s"] and thresholds["physics_dt_s"]:
                if count < round(thresholds["rest_window_s"] / thresholds["physics_dt_s"]) + 1:
                    failed.append(phase + "." + body + ".sample_count")

    hold = trial.get("final_hold") or {}
    endpoint_errors = []
    for joint in ("lower_slide", "upper_slide"):
        value = number(hold.get("joints", {}).get(joint), "final_hold.joints." + joint)
        if value is not None:
            endpoint_errors.append(abs(value))
            less(abs(value), thresholds["rack_endpoint_m"], "final_hold.joints." + joint, inclusive=True)
    peak = less(trial.get("maximum_cycle_penetration_m"), thresholds["peak_penetration_m"],
                "maximum_cycle_penetration_m")
    envelope = metadata.get("domains", {}).get("interior_bounds", {})
    bounds = trial.get("final_mesh_bounds_m")
    arrays = [envelope.get("lower_m"), envelope.get("upper_m")]
    if not isinstance(bounds, list) or len(bounds) != 2:
        missing.append("final_mesh_bounds_m")
        bounds = [None, None]
    arrays += bounds
    clearance = None
    if any(not isinstance(a, list) or len(a) != 3 or not all(_finite(v) for v in a) for a in arrays):
        missing.append("finite interior and mesh bounds")
    else:
        lo, hi, mesh_lo, mesh_hi = arrays
        if any(lo[i] >= hi[i] or mesh_lo[i] > mesh_hi[i] for i in range(3)):
            failed.append("ordered interior and mesh bounds")
        clearance = min(*(mesh_lo[i] - lo[i] for i in range(3)), *(hi[i] - mesh_hi[i] for i in range(3)))
        tolerance = thresholds["containment_tolerance_m"]
        if tolerance is not None and clearance < -tolerance:
            failed.append("whole_mesh_containment")

    # Trace checks complement the full-rate one-second statistics saved above.
    # They cannot independently certify unsaved intervals between 10 Hz samples.
    final_trace = [row for row in trace if row.get("phase") == "final_hold"]
    trace_error = None
    if not final_trace:
        missing.append("final_hold trace")
    else:
        hz = metadata.get("runtime", {}).get("physics_hz")
        if not _finite(hz) or hz <= 0 or not all(_finite(row.get("step")) for row in final_trace):
            missing.append("final_hold trace timebase")
        else:
            end = final_trace[-1]["step"]
            window_s = thresholds["rest_window_s"]
            if window_s is not None:
                final_trace = [row for row in final_trace if row["step"] >= end - hz * window_s]
                trace_hz = metadata.get("runtime", {}).get("trace_hz")
                if not _finite(trace_hz) or trace_hz <= 0:
                    missing.append("runtime.trace_hz")
                elif len(final_trace) < math.floor(window_s * trace_hz):
                    missing.append("full final_hold trace window")
            errors = []
            for row in final_trace:
                for joint in ("lower_slide", "upper_slide"):
                    value = number(row.get("joints", {}).get(joint), "trace.joints." + joint)
                    if value is not None:
                        errors.append(abs(value))
                        less(abs(value), thresholds["rack_endpoint_m"], "trace.joints." + joint, inclusive=True)
                pairs = row.get("contact_pairs")
                if not isinstance(pairs, list):
                    missing.append("trace.contact_pairs")
                else:
                    pairs = {tuple(sorted(pair)) for pair in pairs if isinstance(pair, list) and len(pair) == 2}
                    edge = lambda a, b: tuple(sorted((a, b))) in pairs
                    support = edge(trial["kind"], trial["rack"]) or (
                        trial["rack"] == "LowerRack" and edge(trial["kind"], "SilverwareBasket")
                        and edge("SilverwareBasket", "LowerRack"))
                    if not support:
                        failed.append("trace.dish_supported")
            trace_error = max(errors) if errors else None
    return {"trial_id": trial["trial_id"], "result": "FAIL" if failed else "INCOMPLETE" if missing else "PASS",
            "failed": sorted(set(failed)), "missing": sorted(set(missing)),
            "final_saved_endpoint_error_m": max(endpoint_errors) if endpoint_errors else None,
            "trace_window_endpoint_error_m": trace_error,
            "cycle_peak_penetration_m": peak, "wall_clearance_m": clearance}


def analyse(directory):
    directory = Path(directory)
    metadata, summary = _json(directory / "metadata.json"), _json(directory / "summary.json")
    if summary.get("status") in {None, "running"} or not metadata.get("finished_utc"):
        raise ValueError("Run is not finalized; wait for completion before analysing its evidence")
    if metadata.get("run_status") != summary["status"]:
        raise ValueError("Metadata and summary run statuses disagree")
    trials = [json.loads(line) for line in (directory / "trials.jsonl").read_text().splitlines() if line.strip()]
    rebuilt = build_summary(trials, kinds=metadata["kinds"], racks=metadata["racks"],
                            samples_per_cell=metadata["samples_per_cell"], baselines=summary.get("baselines"),
                            status=summary["status"])
    if rebuilt["cells"] != summary.get("cells") or rebuilt["totals"] != summary.get("totals"):
        raise ValueError("Saved summary counts disagree with trial records; no report written")
    if metadata["samples_per_cell"] != summary.get("samples_per_cell"):
        raise ValueError("Metadata and summary planned sample counts disagree")
    accepted = [trial for trial in trials if trial["outcome"] == "accepted"]
    for trial in accepted:
        if summary.get("baselines", {}).get(trial["rack"], {}).get("result") != "PASS":
            raise ValueError("Accepted trial has no passed empty-rack baseline")
    replay = _json(directory / "accepted_poses.json")
    if replay.get("trials") != accepted:
        raise ValueError("Accepted replay poses disagree with the recorded trials")
    wall = metadata.get("wall_seconds")
    summary_wall = summary.get("extra", {}).get("wall_seconds")
    if not _finite(wall) or wall < 0:
        raise ValueError("Actual elapsed wall time is missing or invalid")
    if summary_wall is not None and (not _finite(summary_wall) or abs(wall-summary_wall) > 1e-6):
        raise ValueError("Metadata and summary elapsed wall times disagree")
    resume_audit, accounting = audit_resume(directory, metadata, summary, trials)
    stages, audits = {}, []
    for trial in trials:
        trace = _trace(directory, trial)
        stages[trial["trial_id"]] = physical_entry(trial, trace)
        if trial["outcome"] == "accepted":
            audits.append(audit_accepted(trial, metadata, trace))
    for cell in summary["cells"]:
        rows = [trial for trial in trials if (trial["kind"], trial["rack"]) == (cell["kind"], cell["rack"])]
        cell["physical_entered"] = sum(stages[t["trial_id"]]["entered"] is True for t in rows)
        cell["physical_entry_unknown"] = sum(stages[t["trial_id"]]["entered"] is None for t in rows)
    verdict = "FAIL" if any(a["result"] == "FAIL" for a in audits) else (
        "INCOMPLETE" if any(a["result"] == "INCOMPLETE" for a in audits) else "PASS" if audits else "NOT_RUN")
    return {"directory": directory, "metadata": metadata, "summary": summary, "trials": trials,
            "stages": stages, "audits": audits, "audit_result": verdict, "wall_seconds": wall,
            "resume_audit": resume_audit, "accounting": accounting}


def structured_report(result):
    """Stable, hashed audit evidence for the package builder; no source run mutation."""
    directory = result["directory"]
    return {"schema_version": 1, "status": result["audit_result"],
            "accepted_count": len(result["audits"]), "audits": result["audits"],
            "input_hashes": {name: _digest(directory / name) for name in (
                "metadata.json", "summary.json", "trials.jsonl", "accepted_poses.json")},
            "trace_hashes": {trial["trace_file"]: _digest(_child(directory, trial["trace_file"]))
                             for trial in result["trials"] if trial.get("trace_file")
                             and _child(directory, trial["trace_file"]).is_file()},
            "output_hashes": {name: _digest(directory / name) for name in ("analysis.md", "outcome_breakdown.png")
                              if (directory / name).is_file()},
            "resume_audit": result["resume_audit"], "accounting": result["accounting"],
            "source_sha256": _digest(Path(__file__)),
            "scope": "Audit of saved gate measurements and resume evidence; no new physics replay."}


def _ratio(numerator, denominator):
    return f"{numerator}/{denominator} ({100*numerator/denominator:.1f}%)" if denominator else "—"


def markdown(result):
    summary, meta = result["summary"], result["metadata"]
    total = summary["totals"]
    entered = sum(c["physical_entered"] for c in summary["cells"])
    unknown = sum(c["physical_entry_unknown"] for c in summary["cells"])
    accounting = result["accounting"]
    lines = ["# Random dish poses: measured results", "",
             f"Run `{result['directory'].name}`; seed {meta.get('seed', 'unrecorded')}; "
             f"status **{summary['status']}**; actual elapsed **{result['wall_seconds']:.1f} s** "
             f"({result['wall_seconds']/60:.1f} min).", "",
             f"**{total['accepted']} accepted / {total['attempted']} logical raw proposals** "
             f"{_ratio(total['accepted'], total['attempted']).split(' ', 1)[-1]}. "
             f"{total['unattempted']} of {total['planned']} planned proposals were untested; "
             f"{total['unresolved']} attempted proposals remain unresolved.", "",
             "| Dish / rack | Accepted / logical raw proposals | Known logical physics entries | Accepted / known logical physics entries | Untested | Unresolved |",
             "| --- | ---: | ---: | ---: | ---: | ---: |"]
    for cell in summary["cells"]:
        label = cell['kind'].replace('_', ' ') + " / " + cell['rack'].replace('Rack', '').lower()
        lines.append(f"| {label} | {_ratio(cell['accepted'], cell['attempted'])} | {cell['physical_entered']} | "
                     f"{_ratio(cell['accepted'], cell['physical_entered'])} | {cell['unattempted']} | {cell['unresolved']} |")
    lines.extend(["", f"Physics entry is evidenced for **{entered}** proposals; entry is unknown for **{unknown}**. "
                  "Initial collisions count in the raw-proposal denominator and are excluded from physics entries. "
                  "For interrupted/error trials, a saved physics measurement or active-dish trace is required to count an entry. "
                  "Each logical sample contributes at most once to this physical-entry denominator. "
                  "The conditional physics-entry rate describes this filtered subset, not arbitrary random poses.", "",
                  "Empty-rack baselines: " + "; ".join(f"{rack}: {value.get('result', 'NOT_RUN')}"
                                                     for rack, value in summary.get("baselines", {}).items()) + ".", "",
                  "Outcomes: " + "; ".join(f"{key.replace('_', ' ')} {total[key]}" for key in OUTCOMES) + ".", "",
                  "![Outcome counts per planned dish/rack cell](outcome_breakdown.png)", "",
                  f"Accepted-evidence audit: **{result['audit_result']}**, {len(result['audits'])} recorded accepted trials. "
                  "This checks saved rest, support, joint, whole-mesh containment, and penetration gates, "
                  "plus available final-hold trace samples. It is not a new simulation.", ""])
    if meta.get("resume") is not None:
        resume_lines = [
            f"Continuation accounting: **{accounting['session_wall_seconds']:.1f} s current session**, "
            f"**{accounting['cumulative_wall_seconds']:.1f} s cumulative** "
            f"({accounting['cumulative_wall_seconds']/60:.1f} min). "
            f"**{accounting['inherited_classified_trials']} classified records inherited unchanged**; "
            f"**{accounting['new_evaluation_attempts']} new evaluation attempts**.", "",
            f"There are **{accounting['logical_attempted']} attempted logical samples** out of "
            f"{accounting['logical_planned']} planned, versus **{accounting['cumulative_evaluation_attempts']} cumulative "
            f"evaluation attempts** including {accounting['archived_unresolved_attempts']} archived unresolved attempts. "
            "Evaluation attempts include FCL-rejected proposals; they are not a count of dishes released into physics. "
            "An interrupted attempt and its retry share one logical sample ID and are not counted twice in acceptance rates.", "",
            f"Resume provenance audit: **{result['resume_audit']['status']}**. " + (
                "Archived source reports and traces match their recorded hashes; inherited classified rows and trace bytes "
                "are unchanged; old unresolved records remain in `resume_history/`." if result['resume_audit']['status'] == "PASS"
                else "No prior evidence was imported; saved resume metadata is unapplied."), "",
        ]
        lines[4:4] = resume_lines
    if result["audits"]:
        for key, label, scale, unit in (
            ("final_saved_endpoint_error_m", "Maximum rack endpoint error at the final saved joint sample", 1000, "mm"),
            ("trace_window_endpoint_error_m", "Maximum sampled rack endpoint error in the trailing final-hold trace window", 1000, "mm"),
            ("cycle_peak_penetration_m", "Maximum penetration during accepted rack retraction/hold cycles", 1000, "mm"),
        ):
            values = [a[key] for a in result["audits"] if a[key] is not None]
            if values:
                lines.append(f"- {label}: **{max(values)*scale:.4f} {unit}**.")
        clearance = [a["wall_clearance_m"] for a in result["audits"] if a["wall_clearance_m"] is not None]
        if clearance:
            lines.append(f"- Minimum accepted whole-mesh clearance from the interior envelope: **{min(clearance)*1000:.4f} mm** "
                         "(negative values are within the recorded numerical tolerance, not physical clearance).")
        lines.append("")
    for audit in result["audits"]:
        if audit["result"] != "PASS":
            lines.append(f"- `{audit['trial_id']}`: {audit['result']}; failed={audit['failed']}; missing={audit['missing']}.")
    unresolved = [t for t in result["trials"] if t["outcome"] in {"interrupted", "simulation_error"}]
    if unresolved:
        lines.extend(["", "Unresolved trial stages:", ""])
        for trial in unresolved:
            stage = result["stages"][trial["trial_id"]]
            entry = "yes" if stage["entered"] is True else "no" if stage["entered"] is False else "unknown"
            lines.append(f"- `{trial['trial_id']}`: {trial['outcome']}; last saved phase `{stage['last_phase']}`; "
                         f"dish physics entered: {entry}. {trial.get('reason', '')}")
    thresholds = meta.get("thresholds", {})
    lines.extend(["", "Protocol: one dish at a time; uniformly sampled 3-D rotations and rack-local rotated-mesh "
                  "bounding-box centers; natural settling before retracting the selected rack. "
                  "The other rack stays retracted and the door stays open. "
                  f"Recorded tolerances: rack endpoint {thresholds.get('rack_endpoint_m', 'missing')} m, "
                  f"containment {thresholds.get('containment_tolerance_m', 'missing')} m, "
                  f"peak penetration {thresholds.get('peak_penetration_m', 'missing')} m. "
                  "Transient landing penetration is recorded by the experiment; the acceptance gates check "
                  "settled-window contacts and the entire retraction/hold cycle.", "",
                  "Counts describe accepted sampled drops, not unique resting arrangements, maximum capacity, "
                  "or a finite number of continuous poses. Untested and unresolved proposals are not impossible placements. "
                  "Door closure and multi-dish loading were not tested. This analysis uses this run and any "
                  "verified inherited continuation records; unrelated runs, including any duplicate-seed smoke test, are excluded.", "",
                  "See `metadata.json` for geometry hashes, seed, bounds, runtime and thresholds; "
                  "`trials.jsonl` for raw measurements; `accepted_poses.json` for replayable accepted poses.", ""])
    return "\n".join(lines)


def plot(result, destination):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.ticker import MaxNLocator

    cells = result["summary"]["cells"]
    colors = ("#22834f", "#a8adb5", "#e9b44c", "#d95f59", "#8d6cab", "#518eb7", "#9d3434", "#e3882b", "#f2f2f2")
    outcomes = (*OUTCOMES, "unattempted")
    figure, axis = plt.subplots(figsize=(12, max(5, .72*len(cells)+1.5)))
    try:
        left = [0] * len(cells)
        for outcome, color in zip(outcomes, colors):
            values = [cell[outcome] for cell in cells]
            axis.barh(range(len(cells)), values, left=left, color=color, edgecolor="white", linewidth=.5,
                      label=outcome.replace("_", " "), hatch="///" if outcome == "unattempted" else None)
            left = [a+b for a, b in zip(left, values)]
        planned = max(cell["planned"] for cell in cells)
        for row, cell in enumerate(cells):
            axis.text(planned*1.02, row, _ratio(cell["accepted"], cell["attempted"]), va="center", fontsize=9)
        axis.set_yticks(range(len(cells)))
        axis.set_yticklabels([c["kind"].replace("_", " ") + " / " + c["rack"].replace("Rack", "").lower() for c in cells])
        axis.invert_yaxis()
        axis.set_xlim(0, max(1, planned)*1.35)
        axis.xaxis.set_major_locator(MaxNLocator(integer=True))
        axis.set_xticks([value for value in axis.get_xticks() if 0 <= value <= planned])
        axis.set_xlabel("Number of planned proposals (including untested); right labels: accepted / attempted")
        axis.set_title(f"Random single-dish placements — {result['directory'].name}\n"
                       f"{result['summary']['status']}; seed {result['metadata'].get('seed', 'unrecorded')}")
        axis.xaxis.grid(alpha=.2)
        axis.set_axisbelow(True)
        axis.legend(loc="upper center", bbox_to_anchor=(.5, -.15), ncol=3, frameon=False, fontsize=9)
        figure.tight_layout()
        figure.savefig(destination, dpi=180, bbox_inches="tight")
    finally:
        plt.close(figure)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", required=True, type=Path)
    args = parser.parse_args(argv)
    try:
        result = analyse(args.run_dir)
        plot(result, args.run_dir / "outcome_breakdown.png")
        _atomic_text(args.run_dir / "analysis.md", markdown(result))
        _atomic_text(args.run_dir / "analysis.json", json.dumps(structured_report(result), indent=2, sort_keys=True, allow_nan=False) + "\n")
    except (OSError, ValueError, KeyError, TypeError, ImportError) as exc:
        print(f"REPORT ERROR: {exc}", file=sys.stderr)
        return 2
    print(f"Accepted-evidence audit: {result['audit_result']}; analysis.json, analysis.md and outcome_breakdown.png written")
    return 0 if result["audit_result"] in {"PASS", "NOT_RUN"} else 1


if __name__ == "__main__":
    raise SystemExit(main())
