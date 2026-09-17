#!/usr/bin/env python3
"""Read-only cross-artifact audit of the completed Frigidaire packing delivery.

Only the explicitly selected audit JSON is written. --allow-incomplete permits
an ongoing run to exit successfully, but its verdict remains INCOMPLETE.
"""
from __future__ import annotations

import argparse
from collections import Counter, defaultdict
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import struct
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
sys.path[:0] = [str(ROOT/"src"), str(ROOT/"frigidaire/src")]
from dishsim_frigidaire.random_poses import compose_pose, relative_pose
from frigidaire_initial_state_report import (digest, image_records, load_experiment,
                                            pose_error, validate_pose, validate_state)


def read(path):
    return json.loads(Path(path).read_text())


def portable_path(value):
    path = Path(value)
    marker = "/workspace/dishsim/"
    if str(path).startswith(marker):
        return ROOT/str(path)[len(marker):]
    return path if path.is_absolute() else ROOT/path


def canonical_digest(document):
    return hashlib.sha256(json.dumps(document, sort_keys=True, separators=(",", ":")).encode()).hexdigest()


def png_dimensions(path):
    header = Path(path).read_bytes()[:24]
    if len(header) != 24 or header[:8] != b"\x89PNG\r\n\x1a\n" or header[12:16] != b"IHDR":
        raise ValueError("Image is not a PNG with an IHDR header")
    return list(struct.unpack(">II", header[16:24]))


def same_pose(first, second, position_tolerance=1e-10):
    error = pose_error(first, second)
    return error["position_error_m"] <= position_tolerance and error["orientation_error_deg"] <= 1e-4


def pose(position, quaternion):
    return {"position_m": np.asarray(position).tolist(), "quaternion_xyzw": np.asarray(quaternion).tolist()}


def candidate_errors(catalog, trials):
    """Check the actual perturbations and frame compositions, independently of RNG replay."""
    if not __debug__:
        raise ValueError("Delivery audit must run without Python -O; invariant assertions are required")
    errors, seen, variants = [], set(), defaultdict(set)
    by_id = {trial["trial_id"]: trial for trial in trials}
    for index, candidate in enumerate(catalog["candidates"]):
        try:
            identity, source_id = candidate["candidate_id"], candidate["source_trial_id"]
            source = by_id[source_id]
            variant = candidate["variant_index"]
            assert identity not in seen and candidate["candidate_index"] == index
            assert identity == f"{source_id}_v{variant:02d}" and 0 <= variant <= 8
            assert candidate["kind"] == source["kind"] and candidate["rack"] == source["rack"]
            assert candidate["source_final_pose"] == source["final_pose"]
            assert candidate["source_final_rack_pose"] == source["final_rack_pose"]
            seen.add(identity)
            variants[source_id].add(variant)
            perturbation = candidate["perturbation"]
            delta = np.asarray(perturbation["translation_m"], dtype=float)
            axis = np.asarray(perturbation["rotation_axis_rack"], dtype=float)
            angle = float(perturbation["rotation_angle_deg"])
            assert delta.shape == axis.shape == (3,) and np.isfinite(delta).all() and np.isfinite(axis).all()
            assert np.linalg.norm(delta) <= .010+1e-12
            assert abs(np.linalg.norm(delta)-perturbation["translation_norm_m"]) <= 1e-12
            assert abs(np.linalg.norm(axis)-1.) <= 1e-10 and math.isfinite(angle) and 0 <= angle <= 10.+1e-10
            half = math.radians(angle)/2
            expected_delta_q = np.r_[axis*math.sin(half), math.cos(half)]
            delta_q = perturbation["rotation_quaternion_xyzw"]
            assert np.allclose(expected_delta_q, delta_q, atol=1e-12, rtol=0.)
            if variant == 0:
                assert np.linalg.norm(delta) == 0 and angle == 0
            for key in ("source_rack_local_pose", "rack_local_pose", "pose_world"):
                validate_pose(candidate[key])
            source_p, source_q = relative_pose(source["final_pose"]["position_m"],
                source["final_pose"]["quaternion_xyzw"], source["final_rack_pose"]["position_m"],
                source["final_rack_pose"]["quaternion_xyzw"])
            assert same_pose(candidate["source_rack_local_pose"], pose(source_p, source_q))
            _, perturbed_q = compose_pose([0., 0., 0.], delta_q, [0., 0., 0.], source_q)
            assert same_pose(candidate["rack_local_pose"], pose(source_p+delta, perturbed_q))
            rack = catalog["baseline_components"][candidate["rack"]]
            p, q = compose_pose(rack["position_m"], rack["quaternion_xyzw"], source_p+delta, perturbed_q)
            assert same_pose(candidate["pose_world"], pose(p, q))
        except (AssertionError, KeyError, TypeError, ValueError) as exc:
            errors.append({"candidate_index": index, "error": str(exc) or "candidate invariant failed"})
    if set(variants) != set(by_id) or any(value != set(range(9)) for value in variants.values()):
        errors.append({"error": "Every source trial must supply its original plus eight variants"})
    return errors


def selection_errors(indices, objects, candidates, allowed, conflicts):
    errors = []
    if len(indices) != len(set(indices)) or not set(indices) <= allowed or len(indices) != len(objects):
        errors.append("Candidate indices are duplicated, ineligible, or disagree with object count")
    if any(first in indices and second in indices for first, second in conflicts):
        errors.append("Selected state contains a conflicting candidate pair")
    if {entry.get("candidate_index") for entry in objects} != set(indices):
        errors.append("Object candidate indices differ from state selection")
    for entry in objects:
        index = entry.get("candidate_index")
        if not isinstance(index, int) or not 0 <= index < len(candidates):
            errors.append("Object has an invalid candidate index")
            continue
        source = candidates[index]
        for key in ("candidate_id", "source_trial_id", "kind", "rack", "variant_index", "perturbation", "mass_kg", "size_m"):
            if entry.get(key) != source[key]:
                errors.append(f"{entry.get('object_id')}: {key} differs from source candidate")
        if entry.get("candidate_rack_local_pose") != source["rack_local_pose"]:
            errors.append(f"{entry.get('object_id')}: original proposed rack-local pose differs")
    return errors


def valid_full_graph_bound(record, expected=35):
    dual = record.get("mip_dual_bound")
    return (record.get("bound_scope") == "full finite compatibility graph"
            and record.get("target_count") is None and record.get("status") in (0, 1)
            and record.get("geometric_cardinality_upper_bound") == expected
            and isinstance(dual, (int, float)) and math.isfinite(dual)
            and math.floor(-dual+1e-7) <= expected
            and isinstance(record.get("count"), int) and record["count"] <= expected)


def source_coverage(directory, documents, summary):
    """Find each recorded executable content hash; missing baseline hashes stay explicit."""
    archived = defaultdict(list)
    archive_roots = [directory/"source_snapshot", directory/"source_revisions"]
    archive_roots += list((directory/"resume_history").glob("**/source_snapshot"))
    for root in archive_roots:
        for path in sorted(root.rglob("*.py")):
            archived[digest(path)].append(str(path.relative_to(directory)))
    references = []
    for label, document in documents:
        hashes = document.get("execution_source_hashes", document.get("source_hashes", {}))
        references.extend({"evidence": label, "source_file": name, "sha256": value}
                          for name, value in hashes.items())
    references += [{"evidence": "summary.json", "source_file": name, "sha256": value}
                   for name, value in summary.get("source_hashes", {}).items()]
    versions = {}
    for reference in references:
        key = (reference["source_file"], reference["sha256"])
        item = versions.setdefault(key, {"source_file": key[0], "sha256": key[1], "evidence": [],
                                        "archive_paths": archived.get(key[1], [])})
        item["evidence"].append(reference["evidence"])
    for item in versions.values():
        current = portable_path(item["source_file"])
        item["current_file_matches"] = current.is_file() and digest(current) == item["sha256"]
        item["coverage"] = "archived" if item["archive_paths"] else "current_only" if item["current_file_matches"] else "missing"
    return {"versions": list(versions.values()), "reference_count": len(references),
            "needs_archive": [item for item in versions.values() if item["coverage"] == "current_only"],
            "missing": [item for item in versions.values() if item["coverage"] == "missing"],
            "baseline_execution_hash_note": "Baseline result has no recorded execution_source_hashes; no exact execution hash is inferred retroactively."}


def audit(directory):
    directory = Path(directory).resolve()
    result = {"schema_version": 1, "created_utc": datetime.now(timezone.utc).isoformat(),
              "run_directory": str(directory), "gates": [], "scope":
              "Saved evidence consistency and finite candidate catalog optimum; not a continuous global dishwasher capacity proof."}

    def gate(name, ok, details=None, pending=False):
        result["gates"].append({"name": name, "status": "PASS" if ok else "PENDING" if pending else "FAIL", "details": details})

    summary, catalog, graph, inputs = (read(directory/name) for name in
        ("summary.json", "candidates.json", "compatibility.json", "input_validation.json"))
    source_path = portable_path(summary["source_pool"])
    source = read(source_path)
    source_hash = digest(source_path)
    trials = source["trials"]
    gate("original_176_accepted_pool_unchanged", len(trials) == 176
         and len({t["trial_id"] for t in trials}) == 176 and all(t["outcome"] == "accepted" for t in trials)
         and source_hash == summary["input_hashes"]["accepted_poses.json"] == catalog["source_accepted_sha256"]
         == graph["source_accepted_sha256"], {"path": str(source_path), "sha256": source_hash, "count": len(trials)})
    candidates = catalog["candidates"]
    errors = candidate_errors(catalog, trials)
    gate("1584_bounded_candidates_and_frame_compositions", len(candidates) == catalog["candidate_count"] == 1584
         and catalog["source_trial_count"] == 176 and catalog["variants_per_template"] == 8 and not errors,
         {"errors": errors[:40], "error_count": len(errors)})
    allowed = set(graph["allowed_indices"])
    conflicts = [tuple(pair) for pair in graph["conflict_pairs"]]
    graph_ok = (graph["complete"] is True and graph["unresolved_pairs"] == 0
                and graph["allowed_count"] == len(allowed) == len(graph["allowed_indices"]) == 334
                and graph["candidate_count"] == 1584 and graph["candidate_catalog_sha256"] == canonical_digest(catalog)
                and all(0 <= i < 1584 for i in allowed)
                and all(a != b and a in allowed and b in allowed for a, b in conflicts))
    gate("complete_334_candidate_graph_integrity", graph_ok,
         {"canonical_catalog_sha256": canonical_digest(catalog), "graph_sha256": digest(directory/"compatibility.json"),
          "allowed_count": len(allowed), "conflict_count": len(conflicts), "unresolved_pairs": graph["unresolved_pairs"]})
    asset_dir = portable_path(inputs["asset_path"]).parent
    asset_errors = [name for name, expected in inputs["asset_hashes"].items()
                    if not (asset_dir/name).is_file() or digest(asset_dir/name) != expected]
    gate("asset_inputs_unchanged", not asset_errors and inputs["asset_hashes"] == summary["input_hashes"]["asset_hashes"], asset_errors)
    gate("graph_asset_hashes_match_validated_assets", bool(graph["asset_sha256"])
         and all(inputs["asset_hashes"].get(name) == value for name, value in graph["asset_sha256"].items()))

    experiment = load_experiment(directory)
    gate("existing_report_saved_record_audit", not experiment["problems"], experiment["problems"])
    states = []
    all_state_errors, seen_sets = [], {}
    for row in experiment["states"]:
        state = row["state"]
        errors = selection_errors(state["candidate_indices"], state["objects"], candidates, allowed, conflicts)
        selected = frozenset(state["candidate_indices"])
        if selected in seen_sets:
            errors.append("Duplicate candidate set also published as "+seen_sets[selected])
        seen_sets[selected] = state["state_id"]
        if state.get("input_hashes") != summary["input_hashes"]:
            errors.append("State input hashes differ from run input hashes")
        source_result = read(directory/"attempts"/state["source_attempt"]/"result.json")
        if source_result != state["validation"]:
            errors.append("State validation differs from its actual saved source attempt")
        try:
            validate_state(state)
        except (ValueError, KeyError, TypeError) as exc:
            errors.append(str(exc))
        all_state_errors.extend({"state_id": state["state_id"], "error": error} for error in errors)
        states.append({"state_id": state["state_id"], "source_attempt": state["source_attempt"],
                       "purpose": state["purpose"], "counts": row["counts"], "candidate_indices": state["candidate_indices"],
                       "state_sha256": row["sha256"], "valid": not errors})
    result["states"] = states
    gate("state_candidate_sets_and_physics_evidence", not all_state_errors, all_state_errors)
    purposes = Counter(row["purpose"] for row in states)
    gate("eleven_deliverable_states", len(states) == 11 and purposes == {"highest": 1, "randomized": 10},
         dict(purposes), pending=len(states) < 11)
    highest_rows = [row for row in experiment["states"] if row["state"]["purpose"] == "highest"]
    valid_bounds = [row for row in summary["search"]["solver_records"] if valid_full_graph_bound(row)]
    highest_ok = len(highest_rows) == 1 and highest_rows[0]["counts"]["total"] == summary["highest_count"] == 35
    gate("highest_35_matches_recorded_full_graph_bound", highest_ok and bool(valid_bounds) and graph_ok,
         {"qualifying_solver_records": valid_bounds, "scope": result["scope"]})
    if highest_rows:
        state = highest_rows[0]["state"]
        validation = state["validation"]
        trace = directory/"attempts"/state["source_attempt"]/validation["trace_file"]
        final_paths = validation["final_hold"]["support_paths"]
        support_ok = all(final_paths.get(obj["object_id"], [None])[0] == obj["object_id"]
                         and final_paths[obj["object_id"]][-1] == obj["rack"] for obj in state["objects"])
        result["highest_physics"] = {"source_attempt": state["source_attempt"], "counts": highest_rows[0]["counts"],
            "trace_file": str(trace.relative_to(directory)), "trace_sha256": digest(trace) if trace.is_file() else None,
            "maximum_settle_penetration_m": validation["maximum_settle_penetration_m"],
            "maximum_cycle_penetration_m": validation["maximum_cycle_penetration_m"],
            "maximum_settle_penetration_event": validation.get("maximum_settle_penetration_event"),
            "maximum_cycle_penetration_event": validation.get("maximum_cycle_penetration_event"),
            "final_support_paths": final_paths, "all_final_contained": all(v["contained"] for v in validation["final_containment"].values())}
        gate("highest_trace_support_and_containment", trace.is_file() and trace.stat().st_size > 0 and support_ok
             and result["highest_physics"]["all_final_contained"])

    images = image_records(experiment)
    image_errors, image_rows = [], []
    for item in images:
        metadata = read(directory/item["evidence"])
        dimensions = png_dimensions(directory/item["path"])
        if dimensions != [1920, 1440]:
            image_errors.append(item["path"]+": PNG dimensions differ")
        image_rows.append({**item, "png_dimensions": dimensions, "image_sha256": metadata["image_sha256"],
                           "maximum_position_error_m": metadata["max_position_error_m"],
                           "maximum_orientation_error_deg": metadata["max_orientation_error_deg"]})
    result["images"] = image_rows
    gate("two_matching_1920x1440_Isaac_renders", len(images) == 2 and not image_errors
         and {item["purpose"] for item in images} == {"highest", "randomized"}, image_errors, pending=len(images) < 2)
    documents = [(row["path"], row["result"]) for row in experiment["attempts"]]
    documents += [(str(path.relative_to(directory)), read(path)) for path in sorted((directory/"renders").glob("*_render_evidence.json"))]
    report_path = directory/"report_audit.json"
    if report_path.is_file():
        report = read(report_path)
        report_errors = []
        for relative, expected in report.get("input_hashes", {}).items():
            path = (directory/relative).resolve()
            if directory not in path.parents or not path.is_file() or digest(path) != expected:
                report_errors.append(relative)
        gate("technical_report_inputs_current", bool(report.get("input_hashes")) and not report_errors
             and report.get("result") == "PASS" and report.get("state_count") == len(states)
             and report.get("highest_count") == summary["highest_count"], report_errors)
        documents.append(("report_audit.json", report))
    else:
        gate("technical_report_inputs_current", False, "Generate final report_audit.json before final delivery audit", pending=True)
    documents.append(("delivery_audit_execution", {"execution_source_hashes":
        {str(Path(__file__).relative_to(ROOT)): digest(__file__)}}))
    coverage = source_coverage(directory, documents, summary)
    result["execution_source_coverage"] = coverage
    gate("executed_source_hashes_available", not coverage["missing"], coverage["missing"])
    gate("executed_source_hashes_archived", not coverage["needs_archive"], coverage["needs_archive"], pending=True)
    statuses = {row["status"] for row in result["gates"]}
    result["result"] = "FAIL" if "FAIL" in statuses else "INCOMPLETE" if "PENDING" in statuses else "PASS"
    result["audit_source_sha256"] = digest(Path(__file__))
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out-dir", type=Path, required=True)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--allow-incomplete", action="store_true")
    args = parser.parse_args()
    output = args.output or args.out_dir/"delivery_audit.json"
    if output.exists():
        parser.error("Audit output already exists; choose a new --output path")
    try:
        result = audit(args.out_dir)
    except Exception as exc:
        import traceback
        result = {"result": "FAIL", "error": repr(exc), "traceback": traceback.format_exc()}
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, indent=2, allow_nan=False)+"\n")
    print("[RESULT] "+result["result"]+" "+str(output), flush=True)
    for gate in result.get("gates", []):
        if gate["status"] != "PASS":
            print(gate["status"]+": "+gate["name"], flush=True)
    return 0 if result["result"] == "PASS" or (args.allow_incomplete and result["result"] == "INCOMPLETE") else 1


if __name__ == "__main__":
    raise SystemExit(main())
