#!/usr/bin/env python3
"""Package a finalized, audited random-dish experiment without changing raw data.

Requires analysis.json, independent_geometry_audit.json, and
accepted_pose_examples.png/.json from the reporting tools. The source files
recorded by the experiment must still match. No appliance assets are copied.
"""
from __future__ import annotations

import argparse
from collections import Counter
from copy import deepcopy
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import sys
import tempfile
import zipfile

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT/"frigidaire/src"))
from dishsim_frigidaire.random_pose_reports import OUTCOMES, build_summary

RAW_FILES = ("metadata.json", "summary.json", "trials.jsonl", "accepted_poses.json")
FINAL_STATUSES = {"complete", "budget_exhausted", "interrupted", "baseline_blocked", "error"}


def digest(path):
    result = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024*1024), b""):
            result.update(block)
    return result.hexdigest()


def read_json(path):
    value = json.loads(Path(path).read_text())
    json.dumps(value, allow_nan=False)
    return value


def safe_file(root, name):
    relative = Path(name)
    if relative.is_absolute() or not relative.parts or ".." in relative.parts:
        raise ValueError("Unsafe artifact path: "+str(name))
    path = root/relative
    if root not in path.resolve().parents or path.is_symlink() or not path.is_file():
        raise ValueError("Missing or external artifact: "+str(name))
    return path


def finite(value, label, nonnegative=False):
    if (isinstance(value, bool) or not isinstance(value, (int, float))
            or not math.isfinite(value) or (nonnegative and value < 0)):
        raise ValueError("Invalid numeric measurement: "+label)
    return value


def indexed(rows, label):
    result = {}
    for row in rows:
        identifier = row["trial_id"]
        if identifier in result:
            raise ValueError("Duplicate "+label+" trial ID: "+identifier)
        result[identifier] = row
    return result


def verify_hashes(directory, recorded, required, label):
    if not isinstance(recorded, dict):
        raise ValueError("Missing "+label+" hashes")
    for name in required:
        if recorded.get(name) != digest(safe_file(directory, name)):
            raise ValueError(label+" hash mismatch: "+name)


def close_numbers(first, second, label, tolerance=1e-9):
    if abs(finite(first, label)-finite(second, label)) > tolerance:
        raise ValueError("Inconsistent "+label)


def validate_run(run_dir, repo_root=ROOT):
    """Read and check all prerequisites before writing any derived artifact."""
    directory, repo = Path(run_dir).resolve(), Path(repo_root).resolve()
    meta, summary, replay, analysis, audit, views = (
        read_json(safe_file(directory, filename)) for filename in (
            "metadata.json", "summary.json", "accepted_poses.json", "analysis.json",
            "independent_geometry_audit.json", "accepted_pose_examples.json"))
    status = summary.get("status")
    if (status not in FINAL_STATUSES or meta.get("run_status") != status
            or not meta.get("finished_utc") or not summary.get("simulation_run")):
        raise ValueError("Package requires a finalized physical run with consistent statuses")
    rows = [json.loads(line) for line in safe_file(directory, "trials.jsonl").read_text().splitlines() if line.strip()]
    json.dumps(rows, allow_nan=False)
    rebuilt = build_summary(rows, kinds=meta["kinds"], racks=meta["racks"],
                            samples_per_cell=meta["samples_per_cell"], baselines=summary["baselines"], status=status)
    if (summary.get("samples_per_cell") != meta["samples_per_cell"]
            or rebuilt["cells"] != summary.get("cells") or rebuilt["totals"] != summary.get("totals")):
        raise ValueError("Summary counts disagree with raw trial records")
    for row in rows:
        if row["trial_id"] != "{}_{}_{:05d}".format(row["kind"], row["rack"], row["sample_index"]):
            raise ValueError("Noncanonical trial ID: "+row["trial_id"])
    accepted = [row for row in rows if row["outcome"] == "accepted"]
    if replay.get("trials") != accepted or not accepted:
        raise ValueError("Accepted replay must exactly match the nonempty accepted raw rows")
    accepted_by_id = indexed(accepted, "accepted")
    totals = summary["totals"]
    if status == "complete" and totals["attempted"] != totals["planned"]:
        raise ValueError("A complete sampling scan must attempt every planned proposal")
    for row in accepted:
        if summary["baselines"][row["rack"]]["result"] != "PASS":
            raise ValueError("Accepted pose lacks a passed rack baseline")

    if analysis.get("status") != "PASS" or analysis.get("accepted_count") != len(accepted):
        raise ValueError("Saved-evidence analysis must PASS every accepted trial")
    verify_hashes(directory, analysis.get("input_hashes"), RAW_FILES, "Saved-evidence input")
    trace_hashes = analysis.get("trace_hashes", {})
    verify_hashes(directory, trace_hashes, trace_hashes, "Saved-evidence trace")
    for row in accepted:
        if not row.get("trace_file") or row["trace_file"] not in trace_hashes:
            raise ValueError("Accepted trace was not hashed by saved-evidence analysis")
    evidence_by_id = indexed(analysis["audits"], "saved-evidence")
    if set(evidence_by_id) != set(accepted_by_id):
        raise ValueError("Saved-evidence IDs disagree with accepted replay")
    for row in evidence_by_id.values():
        if row.get("result") != "PASS" or row.get("failed") or row.get("missing"):
            raise ValueError("Saved-evidence accepted audit is incomplete or failed")
        for key in ("final_saved_endpoint_error_m", "trace_window_endpoint_error_m", "cycle_peak_penetration_m"):
            finite(row[key], key, nonnegative=True)
        finite(row["wall_clearance_m"], "wall_clearance_m")
    if analysis.get("output_hashes"):
        verify_hashes(directory, analysis["output_hashes"], analysis["output_hashes"], "Analysis output")

    if (audit.get("status") != "PASS" or audit.get("passed") is not True or audit.get("errors")
            or audit.get("accepted_count") != len(accepted) or audit.get("checked_count") != len(accepted)):
        raise ValueError("Independent geometry audit must PASS every accepted trial")
    verify_hashes(directory, audit.get("input_hashes"), ("metadata.json", "accepted_poses.json"), "Geometry input")
    geometry_by_id = indexed(audit["trials"], "geometry")
    if set(geometry_by_id) != set(accepted_by_id):
        raise ValueError("Geometry audit IDs disagree with accepted replay")
    tolerance = finite(meta["thresholds"]["containment_tolerance_m"], "containment tolerance", True)
    close_numbers(audit["containment_tolerance_m"], tolerance, "geometry containment tolerance", 1e-15)
    discrepancy_limit = finite(audit["bounds_discrepancy_limit_m"], "bounds discrepancy limit", True)
    if discrepancy_limit > 1e-7:
        raise ValueError("Geometry audit uses a bounds discrepancy limit above 1e-7 m")
    for identifier, row in geometry_by_id.items():
        reference = accepted_by_id[identifier]
        if (row.get("passed") is not True or row.get("world_contained") is not True
                or row.get("cabinet_contained") is not True
                or (row.get("kind"), row.get("rack")) != (reference["kind"], reference["rack"])):
            raise ValueError("Failed or inconsistent geometry trial: "+identifier)
        if finite(row["bounds_discrepancy_m"], "bounds discrepancy", True) > discrepancy_limit:
            raise ValueError("Geometry bounds discrepancy exceeds its limit")
        for frame in ("world", "cabinet"):
            if finite(row[frame+"_min_signed_interior_clearance_m"], frame+" clearance") < -tolerance:
                raise ValueError("Geometry clearance is outside the recorded tolerance")
    for key in ("world_contained_count", "cabinet_contained_count"):
        if audit.get(key) != len(accepted):
            raise ValueError("Geometry containment counts disagree")
    close_numbers(audit["max_bounds_discrepancy_m"], max(row["bounds_discrepancy_m"] for row in geometry_by_id.values()),
                  "maximum reconstructed bounds discrepancy", 1e-15)

    sources = meta.get("source_hashes")
    if not isinstance(sources, dict) or not sources:
        raise ValueError("Recorded source hashes are required for a reproducible package")
    verify_hashes(repo, sources, sources, "Source snapshot")
    for filename in ("geometry.py", "tableware.py"):
        name = "frigidaire/src/dishsim_frigidaire/"+filename
        recorded = audit.get("source_hashes", {}).get(name, {})
        if (recorded.get("matches") is not True or recorded.get("actual_sha256") != sources.get(name)
                or recorded.get("recorded_sha256") != sources.get(name)):
            raise ValueError("Geometry audit source provenance differs: "+filename)

    if (views.get("kind") != "saved_pose_reconstruction" or views.get("new_simulation") is not False
            or views.get("isaac_render") is not False):
        raise ValueError("Views must be labeled saved-pose reconstructions")
    verify_hashes(directory, views.get("input_sha256"), ("metadata.json", "summary.json", "accepted_poses.json"), "Views input")
    figure = safe_file(directory, "accepted_pose_examples.png")
    if views.get("figure_sha256") != digest(figure):
        raise ValueError("Views image hash mismatch")
    if views.get("asset_hashes") != meta["inputs"]["asset_hashes"]:
        raise ValueError("Views asset hashes differ from the experiment")
    for name in ("geometry", "tableware"):
        if views.get(name+"_source_sha256") != sources["frigidaire/src/dishsim_frigidaire/"+name+".py"]:
            raise ValueError("Views source provenance differs: "+name)
    panels = {}
    for panel in views["panels"]:
        key = panel["kind"], panel["rack"]
        if key in panels:
            raise ValueError("Duplicate views panel")
        panels[key] = panel
    cells = {(cell["kind"], cell["rack"]) for cell in summary["cells"]}
    if set(panels) != cells:
        raise ValueError("Views must represent every dish/rack combination")
    for key, panel in panels.items():
        candidates = [row for row in accepted if (row["kind"], row["rack"]) == key]
        expected = min(candidates, key=lambda row: (row["sample_index"], row["trial_id"])) if candidates else None
        if panel.get("trial_id") != (expected["trial_id"] if expected else None):
            raise ValueError("Views selection disagrees with accepted records")
        if expected and (panel.get("dish_pose") != expected["final_pose"]
                         or panel.get("rack_pose") != expected["final_hold"]["poses"][key[1]]):
            raise ValueError("Views transforms disagree with measured final poses")

    accounting = analysis["accounting"]
    expected_accounting = {
        "session_wall_seconds": meta["wall_seconds"],
        "cumulative_wall_seconds": meta.get("cumulative_wall_seconds", meta["wall_seconds"]),
        "inherited_classified_trials": meta.get("inherited_classified_trials", 0),
        "new_evaluation_attempts": meta.get("new_evaluation_attempts", len(rows)),
        "cumulative_evaluation_attempts": meta.get("cumulative_evaluation_attempts", len(rows)),
        "logical_attempted": len(rows), "logical_planned": totals["planned"],
    }
    for key, value in expected_accounting.items():
        close_numbers(accounting.get(key), value, "accounting."+key)
    if (accounting["new_evaluation_attempts"]+accounting["inherited_classified_trials"] != len(rows)
            or accounting["cumulative_evaluation_attempts"] < len(rows)
            or accounting["cumulative_wall_seconds"] < accounting["session_wall_seconds"]
            or accounting.get("archived_unresolved_attempts") != accounting["cumulative_evaluation_attempts"]-len(rows)):
        raise ValueError("Cumulative and logical accounting disagree")
    resumed = meta.get("resume", {}).get("applied") is True
    if analysis.get("resume_audit", {}).get("status") != ("PASS" if resumed else "NOT_APPLICABLE"):
        raise ValueError("Resume history audit is missing or unsuccessful")
    protected = {}
    for path in directory.rglob("*"):
        if path.is_symlink():
            raise ValueError("Package artifacts must not be symlinks: "+str(path))
        if path.is_file() and path.name not in {"experiment_report.md", "strictly_contained_poses.json", "checksums.sha256"}:
            protected[str(path.relative_to(directory))] = digest(path)
    return {"directory": directory, "repo": repo, "metadata": meta, "summary": summary,
            "replay": replay, "rows": rows, "accepted": accepted, "analysis": analysis,
            "audit": audit, "geometry": geometry_by_id, "evidence": evidence_by_id,
            "accounting": accounting, "protected_hashes": protected}


def strict_ids(result):
    return {identifier for identifier, row in result["geometry"].items()
            if row["world_min_signed_interior_clearance_m"] >= 0
            and row["cabinet_min_signed_interior_clearance_m"] >= 0}


def number(value):
    """Display derived floating-point values with useful, explicit precision."""
    return format(value, ".9g")


def rate(accepted, attempted):
    return "unmeasured" if not attempted else number(100*accepted/attempted)+"%"


def markdown(result):
    meta, summary = result["metadata"], result["summary"]
    totals, accounting, selected = summary["totals"], result["accounting"], strict_ids(result)
    limits, runtime = meta["thresholds"], meta.get("runtime", {})
    geometry, evidence = list(result["geometry"].values()), list(result["evidence"].values())
    worst_clearance = min(min(row["world_min_signed_interior_clearance_m"], row["cabinet_min_signed_interior_clearance_m"]) for row in geometry)
    status_label = (f"sampling complete; {totals['unresolved']} unresolved"
                    if summary["status"] == "complete" else summary["status"])
    lines = ["# Random dish placement experiment", "",
             f"Run `{result['directory'].name}`; finished `{meta['finished_utc']}`; status **{status_label}** "
             f"(recorded run status: `{summary['status']}`).", "",
             f"**{totals['accepted']} accepted placements from {totals['attempted']} distinct random proposals "
             f"({rate(totals['accepted'], totals['attempted'])}).** {totals['classified']} proposals were classified, "
             f"{totals['unresolved']} remain unresolved, and {totals['unattempted']} of {totals['planned']} planned proposals were untested.", "",
             f"**{len(selected)} accepted placements also satisfy zero boundary tolerance in both the world envelope "
             f"and measured Cabinet frame.** The other {totals['accepted']-len(selected)} use the configured "
             f"{number(1000*limits['containment_tolerance_m'])} mm containment tolerance. "
             f"The greatest nominal boundary overrun among accepted poses is {number(max(0., -worst_clearance)*1000)} mm.", "",
             "| Dish | Rack | Accepted / attempted | Rate | Also strictly contained |",
             "| --- | --- | ---: | ---: | ---: |"]
    for cell in summary["cells"]:
        count = sum(identifier in selected and row["kind"] == cell["kind"] and row["rack"] == cell["rack"]
                    for identifier, row in result["geometry"].items())
        lines.append(f"| {cell['kind'].replace('_', ' ')} | {cell['rack']} | {cell['accepted']}/{cell['attempted']} | "
                     f"{rate(cell['accepted'], cell['attempted'])} | {count} |")
    lines += ["", "## Attempts and elapsed time", "",
              f"This process evaluated {accounting['new_evaluation_attempts']} proposals and carried "
              f"{accounting['inherited_classified_trials']} prior classified rows. The combined record contains "
              f"{accounting['logical_attempted']} logical proposal IDs. Across all sessions there were "
              f"{accounting['cumulative_evaluation_attempts']} evaluation attempts, including "
              f"{accounting['archived_unresolved_attempts']} archived unresolved attempts; retrying their seeded poses "
              "does not increase the logical proposal count.", "",
              f"This session used {number(accounting['session_wall_seconds'])} s "
              f"({number(accounting['session_wall_seconds']/60)} min). Cumulative session wall time was "
              f"{number(accounting['cumulative_wall_seconds'])} s ({number(accounting['cumulative_wall_seconds']/60)} min). "
              "These times include initialization and baseline checks, not only individual dish trials.", "",
              "| Outcome | Proposals |", "| --- | ---: |"]
    lines += [f"| {outcome.replace('_', ' ')} | {totals[outcome]} |" for outcome in OUTCOMES]
    if (result["directory"]/"outcome_breakdown.png").is_file():
        lines += ["", "![Outcomes by dish/rack combination](outcome_breakdown.png)"]
    lines += ["", "## Method", "",
              f"Seed {meta['seed']}; {meta['samples_per_cell']} planned proposals per dish/rack cell. "
              "Each trial used one dish. Uniform 3-D rotations and uniform rotated visual bounding-box centers "
              "were sampled in the recorded rack-local domains. All dish/rack combinations were interleaved. "
              "Initial collisions consume proposals, and dishes may change pose while settling.", "",
              f"Isaac Sim `{runtime.get('isaac_sim', 'unrecorded')}`; physics device `{runtime.get('device', 'unrecorded')}`; "
              f"{number(runtime.get('physics_hz', 1/limits['physics_dt_s']))} Hz physics. "
              f"The selected rack physically retracted at a commanded speed capped at {number(limits['rack_speed_m_s'])} m/s. "
              "The other rack remained retracted and the door stayed open. "
              "Empty-rack baseline results: "+"; ".join(f"{rack}: {row['result']}" for rack, row in summary["baselines"].items())+".", "",
              f"Acceptance required rack closure within {number(1000*limits['rack_endpoint_m'])} mm, selected-rack support "
              "(or the supported lower-rack basket), whole-mesh containment, and a "
              f"{number(limits['rest_window_s'])} s rest window. Rest limits were position span below "
              f"{number(1000*limits['root_position_span_m'])} mm, orientation span below {number(limits['quaternion_span_deg'])} degrees, "
              f"and mesh-point speed below {number(limits['peak_mesh_point_speed_m_s'])} m/s. "
              f"Peak retraction/hold penetration had to remain below {number(1000*limits['peak_penetration_m'])} mm; "
              f"rest-window median maximum penetration below {number(1000*limits['median_max_penetration_m'])} mm.", "",
              "| Dish | Recorded overall size (m) |", "| --- | --- |"]
    for kind in meta["kinds"]:
        entry = meta["inputs"]["catalog"][kind]
        lines.append("| "+kind.replace("_", " ")+" | "+" × ".join(number(value) for value in entry.get("actual_size_m", entry["size_m"]))+" |")
    lines += ["", "| Rack-local center domain | Lower XYZ (m) | Upper XYZ (m) |",
              "| --- | --- | --- |"]
    for rack, bounds in meta["domains"]["sampling_bounds_by_rack"].items():
        lines.append("| "+rack+" | "+", ".join(number(v) for v in bounds["lower_m"])+" | "+", ".join(number(v) for v in bounds["upper_m"])+" |")
    lines += ["", "## Representative final placements", "",
              "Each panel reconstructs a different saved accepted trial using matching source visual meshes and measured final "
              "dish/rack/basket transforms. The shell and other rack are omitted. These are illustrations of saved simulation "
              "output, not additional simulations or Isaac camera renders.", "",
              "![Representative accepted placements](accepted_pose_examples.png)", "",
              "## Verification and limits", "",
              "Sampling completion describes completion of the planned proposal scan. "
              f"The {totals['unresolved']} unresolved logical trials retain their original classifications and are excluded "
              "from accepted placements and the strictly contained subset. A PASS from the saved-evidence or geometry "
              "audit applies only to accepted placements; it does not certify numerical validity of every attempted trial.", "",
              f"The [saved-evidence audit](analysis.json) passed all {len(evidence)} accepted records. "
              f"Maximum final saved rack endpoint error was {number(1000*max(row['final_saved_endpoint_error_m'] for row in evidence))} mm; "
              f"maximum sampled endpoint error in the trailing hold was {number(1000*max(row['trace_window_endpoint_error_m'] for row in evidence))} mm. "
              f"Maximum accepted retraction/hold penetration was {number(1000*max(row['cycle_peak_penetration_m'] for row in evidence))} mm. "
              "Full-rate saved rest statistics and available trajectory samples were audited; the samples do not independently "
              "measure unsaved intervals.", "",
              f"The [independent geometry audit](independent_geometry_audit.json) regenerated all accepted visual meshes, "
              f"including mug handles. Maximum discrepancy from saved world bounds was "
              f"{number(max(row['bounds_discrepancy_m'] for row in geometry))} m. Every accepted pose passed containment in "
              "both the original world envelope and measured Cabinet frame. Cabinet identity is descriptive; solver drift "
              "does not introduce an extra acceptance condition. This audit neither checks collisions nor reruns physics.", "",
              "The counts describe accepted random drops under this model and sampling protocol. They do not count unique "
              "resting arrangements, all possible continuous poses, or multi-dish capacity. Door closure was not tested. "
              "Unresolved or untested proposals are not impossible placements.", "",
              "## Saved data", "",
              f"- [Accepted poses and measurements](accepted_poses.json): {totals['accepted']} accepted records, with metre positions and XYZW quaternions.",
              f"- [Strictly contained poses](strictly_contained_poses.json): {len(selected)} accepted records meeting the additional zero-tolerance containment filter in both frames.",
              "- [Raw trials](trials.jsonl), [summary CSV](summary.csv), [runtime, geometry hashes, bounds and thresholds](metadata.json), and `traces/`.",
              "- [Saved-evidence analysis](analysis.md), [geometry audit](independent_geometry_audit.json), and [illustration provenance](accepted_pose_examples.json).",
              "- `source_snapshot/` contains only the experiment's recorded source files after hash verification. `reporting_tools/` contains the available reporting scripts and documentation.",
              "- [Checksums](checksums.sha256) cover every bundled file except the checksum list itself. Matching staged appliance assets and the original Isaac runtime are required; assets are not duplicated in this package.", ""]
    if meta.get("resume", {}).get("applied"):
        lines += ["`resume_history/` preserves prior metadata, raw logical records, and unresolved attempts. "
                  "Inherited classified rows retain their original measurements; restarted unresolved proposals use their original seeded poses.", ""]
    return "\n".join(lines)


def package_run(run_dir, repo_root=ROOT):
    result = validate_run(run_dir, repo_root)
    directory, repo = result["directory"], result["repo"]
    selected = strict_ids(result)
    strict = deepcopy(result["replay"])
    strict["trials"] = [row for row in result["accepted"] if row["trial_id"] in selected]
    strict["selection"] = "Subset of originally accepted trials with zero boundary tolerance in both world-envelope and measured Cabinet frames; all original criteria and raw records are unchanged."
    strict["source_accepted_poses_sha256"] = digest(directory/"accepted_poses.json")
    strict["source_geometry_audit_sha256"] = digest(directory/"independent_geometry_audit.json")
    (directory/"strictly_contained_poses.json").write_text(json.dumps(strict, indent=2, sort_keys=True, allow_nan=False)+"\n")
    (directory/"experiment_report.md").write_text(markdown(result))
    for relative, expected in result["metadata"]["source_hashes"].items():
        source = safe_file(repo, relative)
        if digest(source) != expected:
            raise ValueError("Source changed during packaging: "+relative)
        destination = directory/"source_snapshot"/relative
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, destination)
    optional = ["frigidaire/scripts/evaluation/"+name for name in (
        "frigidaire_random_pose_report.py", "frigidaire_random_pose_views.py",
        "frigidaire_random_pose_geometry_audit.py", "frigidaire_random_pose_package.py")]
    optional.append("frigidaire/docs/random_pose_experiment.md")
    for relative in optional:
        source = repo/relative
        if source.is_file():
            destination = directory/"reporting_tools"/relative
            destination.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(safe_file(repo, relative), destination)
    verify_hashes(directory, result["protected_hashes"], result["protected_hashes"], "Finalized artifact changed during packaging")
    files = sorted(path for path in directory.rglob("*") if path.is_file() and path.name != "checksums.sha256")
    for path in files:
        if path.is_symlink() or directory not in path.resolve().parents:
            raise ValueError("External package artifact")
    checksums = {str(path.relative_to(directory)): digest(path) for path in files}
    (directory/"checksums.sha256").write_text("".join(checksum+"  "+relative+"\n" for relative, checksum in checksums.items()))
    archive = directory.parent/(directory.name+".zip")
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(dir=directory.parent, prefix="."+directory.name+"_", suffix=".zip", delete=False) as stream:
            temporary = Path(stream.name)
        with zipfile.ZipFile(temporary, "w", zipfile.ZIP_DEFLATED, compresslevel=6) as bundle:
            for path in [*files, directory/"checksums.sha256"]:
                bundle.write(path, str(path.relative_to(directory.parent)))
        with zipfile.ZipFile(temporary) as bundle:
            if bundle.testzip() is not None:
                raise ValueError("ZIP integrity check failed")
            for relative, checksum in checksums.items():
                if hashlib.sha256(bundle.read(directory.name+"/"+relative)).hexdigest() != checksum:
                    raise ValueError("ZIP content checksum mismatch: "+relative)
        verify_hashes(directory, result["protected_hashes"], result["protected_hashes"], "Finalized artifact changed during archiving")
        os.chmod(temporary, 0o644)
        os.replace(temporary, archive)
        temporary = None
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)
    return {"report": str(directory/"experiment_report.md"), "strictly_contained": len(selected),
            "accepted": len(result["accepted"]), "logical_attempted": result["summary"]["totals"]["attempted"],
            "cumulative_evaluation_attempts": result["accounting"]["cumulative_evaluation_attempts"],
            "archive": str(archive), "archive_bytes": archive.stat().st_size,
            "checksummed_files": len(checksums), "archive_sha256": digest(archive)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", required=True, type=Path)
    parser.add_argument("--repo-root", type=Path, default=ROOT)
    args = parser.parse_args()
    try:
        result = package_run(args.run_dir, args.repo_root)
    except (OSError, ValueError, KeyError, TypeError) as exc:
        parser.exit(1, "[PACKAGE] FAILED: "+str(exc)+"\n")
    print(json.dumps(result, sort_keys=True))


if __name__ == "__main__":
    main()
