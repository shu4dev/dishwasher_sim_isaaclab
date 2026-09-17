#!/usr/bin/env python3
"""Audit and report inventory-preserving organized dishwasher counterparts.

    scripts/run_py.sh frigidaire/scripts/evaluation/frigidaire_organized_report.py \
        --out-dir RUN --pdf

The original packing can fail organization. An organized counterpart requires
joint physics, the additional organization/door gates, and a fresh reproduction.
"""
from __future__ import annotations

import argparse
from collections import Counter
from copy import deepcopy
from datetime import datetime, timezone
import hashlib
import html
import importlib.util
import json
import math
import os
from pathlib import Path
import struct
import sys
import tempfile
from urllib.parse import quote

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(Path(__file__).parent))
from frigidaire_initial_state_report import (COMPONENTS, KINDS, RACKS, digest, finite,
    pose_error, read_json, state_counts, validate_pose, validate_state as validate_source_state)

POLICY = {"mug_max_downward_angle_deg": 45., "bowl_max_downward_angle_deg": 75.,
          "plate_max_vertical_deviation_deg": 15., "minimum_dish_clearance_m": .005,
          "opening_samples": 64, "opening_ray_length_m": .100, "minimum_opening_exposure": .80,
          "require_direct_rack_support": True, "allow_nesting": False, "allow_stacking": False}


def portable_path(value, directory=None):
    path = Path(value)
    prefix = "/workspace/dishsim/"
    if str(path).startswith(prefix):
        path = ROOT / str(path)[len(prefix):]
    return path.resolve() if path.is_absolute() else ((Path(directory) if directory else ROOT) / path).resolve()


def local_path(directory, relative):
    path = (directory / relative).resolve()
    if directory.resolve() not in path.parents:
        raise ValueError(f"Output evidence path escapes run: {relative}")
    return path


def link_path(path, directory):
    return quote(os.path.relpath(Path(path).resolve(), Path(directory).resolve()), safe="/._-")


def inventory(state):
    result = {}
    for obj in state["objects"]:
        identity = obj["object_id"]
        if identity in result:
            raise ValueError("Inventory has duplicated object IDs")
        result[identity] = {key: obj.get(key) for key in ("kind", "mass_kg", "size_m")}
    return result


def validate_inventory(state, source):
    if inventory(state) != inventory(source):
        raise ValueError("Organized state does not preserve exact source object IDs, kinds, masses and dimensions")
    return state_counts(state)


def orientation_error(kind, pose):
    validate_pose(pose)
    x, y, z, w = pose["quaternion_xyzw"]
    norm = math.sqrt(x*x+y*y+z*z+w*w)
    x, y, z, w = x/norm, y/norm, z/norm, w/norm
    normal_z = 1.-2.*(x*x+y*y)
    if kind == "dinner_plate":
        return math.degrees(math.asin(min(1., abs(normal_z))))
    return math.degrees(math.acos(min(1., max(-1., -normal_z))))


def validate_organization(record, objects, poses):
    if not isinstance(record, dict) or record.get("valid") is not True or record.get("passed") is not True:
        raise ValueError("Organization phase is missing or failed")
    if record.get("violations") != []:
        raise ValueError("Accepted organization retains hard-rule violations")
    for key, value in POLICY.items():
        if record.get("policy", {}).get(key) != value:
            raise ValueError(f"Organization policy is missing or changed: {key}")
    identities = {obj["object_id"] for obj in objects}
    if set(record.get("per_object", {})) != identities:
        raise ValueError("Organization does not assess every inventory item")
    for obj in objects:
        identity, kind, rack = obj["object_id"], obj["kind"], obj["rack"]
        item = record["per_object"][identity]
        if item.get("kind") != kind or item.get("rack") != rack or item.get("direct_rack_support") is not True:
            raise ValueError(f"Missing measured direct assigned-rack support: {identity}")
        orient = item.get("orientation", {})
        limit = POLICY["plate_max_vertical_deviation_deg"] if kind == "dinner_plate" else POLICY[f"{kind}_max_downward_angle_deg"]
        error = orientation_error(kind, poses[identity])
        if (orient.get("valid") is not True or not finite(orient.get("orientation_error_deg"))
                or abs(orient["orientation_error_deg"]-error) > 1e-5 or error > limit+1e-7
                or (kind == "dinner_plate" and rack != "LowerRack")):
            raise ValueError(f"Orientation evidence disagrees with measured pose or policy: {identity}")
    clearance = record.get("separation", {}).get("minimum_certified_clearance_m")
    if len(objects) > 1 and (not finite(clearance) or clearance < .005-1e-9):
        raise ValueError("Whole-load dish separation is unassessed or below 5 mm")
    if record.get("nesting", {}).get("intrusions") != []:
        raise ValueError("Nesting exclusion is unassessed or failed")
    vessels = {obj["object_id"] for obj in objects if obj["kind"] in {"mug", "bowl"}}
    exposure = record.get("opening_exposure", [])
    if len(exposure) != len(vessels) or {item.get("object_id") for item in exposure} != vessels:
        raise ValueError("Opening exposure does not assess every vessel")
    for item in exposure:
        count, total, fraction = item.get("unobstructed_rays"), item.get("total_rays"), item.get("unobstructed_fraction")
        if (not isinstance(count, int) or isinstance(count, bool) or total != 64 or not 0 <= count <= total
                or not finite(fraction) or abs(fraction-count/total) > 1e-12 or fraction+1e-12 < .80):
            raise ValueError("Invalid or insufficient 64-ray opening exposure")
    return record


def validate_organized_result(result, objects):
    if result.get("outcome") != "accepted":
        raise ValueError("Organized physics result is not accepted")
    # The legacy validator's purpose labels describe the old experiment only.
    # Build an audit view; saved organized documents are never modified.
    snapshot = result.get("initial_snapshot", {})
    measured_objects = [dict(obj, pose_world=snapshot.get("poses", {}).get(obj["object_id"])) for obj in objects]
    physics_view = {"schema_version": 1, "purpose": "highest", "accepted": True, "objects": measured_objects,
                    "initial_snapshot": snapshot, "validation": result}
    physics_view["counts"] = state_counts(physics_view)
    validate_source_state(physics_view)
    for key, snapshot in (("organization_initial", "initial_snapshot"), ("organization_closed", "final_snapshot"),
                          ("organization_reopened", "reopened_snapshot")):
        poses = result.get(snapshot, {}).get("poses", {})
        validate_organization(result.get(key), objects, poses)
    for name in ("door_close_motion", "door_open_motion"):
        motion = result.get(name, {})
        if motion.get("passed") is not True or motion.get("measured_speed_ok") is not True:
            raise ValueError(f"Actual door cycle is missing or failed: {name}")
        if not finite(motion.get("peak_measured_door_speed_rad_s")) or not 0 <= motion["peak_measured_door_speed_rad_s"] <= .65001:
            raise ValueError(f"Measured door speed is missing or excessive: {name}")
    extensions = result.get("loaded_rack_extension_motions", [])
    if len(extensions) != 2 or {item.get("rack") for item in extensions} != set(RACKS) or any(item.get("passed") is not True for item in extensions):
        raise ValueError("Both loaded rack extensions must pass after reopening")
    for key in ("settled", "door_closed_hold", "reopened_hold"):
        hold = result.get(key, {})
        if hold.get("passed") is not True or hold.get("organization_valid") is not True:
            raise ValueError(f"Missing organized rest hold: {key}")
        if not finite(hold.get("observation_completed_s")) or hold["observation_completed_s"] < 5.-1e-8:
            raise ValueError(f"Insufficient continuous organization observation: {key}")
        if (hold.get("organization_geometry_hz") != 10 or hold.get("continuous_observation_geometry_samples", 0) < 50
                or hold.get("continuous_passing_window_count", 0) < 601 or hold.get("orientation_valid") is not True
                or hold.get("forbidden_contacts_ok") is not True):
            raise ValueError(f"Incomplete organization sampling or forbidden-contact evidence: {key}")
        support = hold.get("direct_rack_support", {})
        if set(support) != {obj["object_id"] for obj in objects} or not all(value is True for value in support.values()):
            raise ValueError(f"Direct support is incomplete: {key}")
        clearance = hold.get("minimum_observed_clearance_m")
        exposure = hold.get("minimum_observed_opening_exposure")
        if len(objects) > 1 and (not finite(clearance) or clearance < .005-1e-9):
            raise ValueError(f"Observed clearance is missing or insufficient: {key}")
        if any(obj["kind"] in {"mug", "bowl"} for obj in objects) and (not finite(exposure) or exposure < .80-1e-12):
            raise ValueError(f"Observed opening exposure is missing or insufficient: {key}")
    for snapshot, expected_door, extended in (("final_snapshot", 0., False), ("reopened_snapshot", math.pi/2, True)):
        joints = result.get(snapshot, {}).get("joints", {})
        targets = {"door_hinge": (expected_door, math.radians(.5)),
                   "lower_slide": (-.49 if extended else 0., .005), "upper_slide": (-.44 if extended else 0., .005)}
        for name, (target, tolerance) in targets.items():
            if not finite(joints.get(name)) or abs(joints[name]-target) > tolerance:
                raise ValueError(f"Measured {snapshot} endpoint is invalid: {name}")
    return result


def validate_organized_state(state, source, require_reproduction=True):
    validate_source_state(dict(state, purpose=source["purpose"]))
    counts = validate_inventory(state, source)
    if state.get("source_state_id") != source.get("state_id"):
        raise ValueError("Counterpart source state ID differs")
    validate_organized_result(state["validation"], state["objects"])
    if require_reproduction:
        reproduction = state.get("reproduction", {})
        if reproduction.get("result") != "PASS":
            raise ValueError("Fresh reproduction is missing or failed")
        validate_organized_result(reproduction.get("validation", {}), state["objects"])
    return counts


def initial_assessment(record):
    if not isinstance(record, dict):
        return {"status": "UNASSESSED", "violations": {}, "direct_support_assessed": False}
    violations = Counter(item.get("rule", "unrecorded") for item in record.get("violations", []))
    direct = [item.get("direct_rack_support") for item in record.get("per_object", {}).values()]
    return {"status": "PASS" if record.get("valid") is True else "FAIL" if record.get("valid") is False else "UNASSESSED",
            "violations": dict(violations), "direct_support_assessed": bool(direct) and all(isinstance(value, bool) for value in direct)}


def search_metadata(result):
    summary, directory = result["summary"], result["directory"]
    metadata = {"catalog": {}, "graph": {}, "screening_phases": [], "screen_catalog_sources": [], "artifact_hashes": {}}
    def document(relative):
        if not isinstance(relative, str):
            return None
        path = local_path(directory, relative)
        if not path.is_file():
            return None
        metadata["artifact_hashes"][relative] = digest(path)
        return read_json(path)
    catalog = document(summary.get("candidate_file")) or {}
    if catalog:
        candidates = catalog.get("candidates", [])
        metadata["catalog"] = {"path": summary["candidate_file"], "candidate_count": len(candidates),
            "count_by_kind": dict(Counter(item["kind"] for item in candidates)),
            "processed_patterns": catalog.get("processed_patterns"), "pattern_count": catalog.get("pattern_count"),
            "refinement_level": catalog.get("refinement_level"), "screening_only": catalog.get("screening_only", False)}
    graph = document(summary.get("compatibility_file")) or {}
    if graph:
        metadata["graph"] = {"path": summary["compatibility_file"], "allowed_count": len(graph.get("allowed_indices", [])),
            "conflict_count": len(graph.get("conflict_pairs", [])), "tested_pairs": graph.get("tested_pairs"),
            "status": graph.get("status"), "full_catalog_complete": graph.get("full_catalog_complete"),
            "bound_scope": graph.get("bound_scope"), "candidate_catalog_sha256": graph.get("candidate_catalog_sha256")}
    screening = summary.get("screening") or {}
    expansion = screening.get("expansion") or {}
    if not isinstance(expansion, dict):
        expansion = {"status": expansion}
    for label, relative, adopted in (("initial", screening.get("result"), screening.get("catalog_adopted", False)),
            ("expansion", screening.get("expansion_result", expansion.get("result")), screening.get("expansion_adopted", False))):
        recorded = document(relative)
        if recorded is not None:
            metadata["screening_phases"].append({"phase": label, "path": relative, "adopted": adopted,
                **{key: recorded.get(key) for key in ("outcome", "candidate_count", "attempt_count", "failure_count", "outcome_counts", "source_candidate_count", "skipped_count")}})
        elif label == "expansion" and expansion:
            metadata["screening_phases"].append({"phase": label, "adopted": adopted, "outcome": expansion.get("status", "unrecorded"),
                "source_candidate_count": expansion.get("candidate_count", expansion.get("source_candidate_count")),
                "candidate_count": None, "attempt_count": None})
    metadata["screen_catalog_sources"] = summary.get("screen_catalog_sources", catalog.get("screening_catalog_sources", catalog.get("screen_catalog_sources", [])))
    for item in metadata["screen_catalog_sources"]:
        if isinstance(item, dict) and isinstance(item.get("path"), str):
            saved = document(item["path"])
            if saved is not None and item.get("sha256") and metadata["artifact_hashes"][item["path"]] != item["sha256"]:
                result["problems"].append("Screen catalog provenance hash mismatch: " + item["path"])
    solvers = summary.get("solver_records", [])
    metadata["solver"] = {"calls": len(solvers), "status_counts": dict(Counter(str(item.get("status", "unrecorded")) for item in solvers)),
        "elapsed_seconds": sum(item["elapsed_s"] for item in solvers if finite(item.get("elapsed_s"))),
        "feasibility_counts": dict(Counter(item.get("geometric_feasibility", "unrecorded") for item in solvers))}
    precheck_domains = {}
    for entry in [*summary.get("geometric_prechecks", {}).values(), *summary.get("geometric_precheck_graph_history", [])]:
        precheck_domains[(entry.get("catalog_sha256"), entry.get("graph_sha256"))] = entry
    checked = [record for entry in precheck_domains.values() for record in entry.get("inventories", {}).values()]
    actual_calls = [record["solver"] for record in checked if finite(record.get("solver", {}).get("elapsed_s"))]
    canonical = lambda value: hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":")).encode()).hexdigest()
    current = precheck_domains.get((canonical(catalog), canonical(graph)), {}) if catalog and graph else {}
    metadata["geometric_prechecks"] = {"catalog_graph_domains": len(precheck_domains), "inventory_records": len(checked),
        "solver_calls": len(actual_calls), "unassessed_records": sum(record.get("solver", {}).get("status") == "unassessed" for record in checked),
        "solver_elapsed_seconds": sum(solver["elapsed_s"] for solver in actual_calls),
        "solver_status_counts": dict(Counter(str(solver.get("status", "unrecorded")) for solver in actual_calls)),
        "current_domain_proven_infeasible_inventories": sum(record.get("proven_infeasible") is True for record in current.get("inventories", {}).values())}
    return metadata


def auxiliary_search_evidence(result):
    records = []
    for entry in result["summary"].get("auxiliary_search_records", []):
        try:
            relative = entry["path"]
            path = local_path(result["directory"], relative)
            if digest(path) != entry.get("sha256"):
                raise ValueError("Referenced auxiliary search hash differs")
            document = read_json(path)
            result["input_hashes"][relative] = entry["sha256"]
            if entry.get("full_state_validation_claim") is not False:
                raise ValueError("Auxiliary search must explicitly remain outside full-state validation")
            if "exact_inventory_solutions_tested" in document:
                if document.get("aggregate_passing_manifest_created") is not False or document.get("no_physics_run") is not True:
                    raise ValueError("Unexpected hybrid-preparation verdict")
                description = (f"Hybrid preparation tested {document['exact_inventory_solutions_tested']} exact-inventory pairwise solutions; "
                    f"all failed {document.get('static_failure')}. Best minimum mouth exposure was {document.get('best_minimum_exposure'):.1%}, "
                    f"below the required {document.get('required_minimum_exposure'):.1%}. No passing manifest or physics run resulted.")
            elif "maximum_under_ceilings" in document:
                capacity = document["maximum_under_ceilings"]
                bound, dual = capacity.get("integer_upper_bound"), capacity.get("objective_dual_bound")
                if (document.get("graph_complete") is not True or capacity.get("status") != 0 or not finite(bound)
                        or not finite(dual) or bound != math.floor(-dual+1e-7) or capacity.get("count") != bound):
                    raise ValueError("Finite graph bound lacks consistent optimal solver evidence")
                ceilings = document.get("inventory_ceilings", {})
                affected = [f"{row['source_state_id']} ({row['counts']['total']} items)" for row in result["rows"]
                    if row.get("counts", {}).get("total", 0) > bound
                    and all(count <= ceilings.get(kind, 0) for kind, count in row["counts"]["by_kind"].items())]
                description = (f"The recorded {document.get('snapshot_candidate_count')}-node screened graph has a pairwise selection upper bound "
                    f"of {int(bound)} under type ceilings {ceilings}; the solver found {capacity['count']} and proved the same integer bound. "
                    "This is a finite geometric graph bound, not a physically validated maximum or a global dishwasher capacity. "
                    + ("Source inventories exceeding this bound while satisfying those ceilings: " + ", ".join(affected)
                       + ". They have no feasible selection in this recorded graph." if affected else ""))
            elif "maximum_swaps" in document:
                if document.get("success") is not False or document.get("no_physics_run") is not True:
                    raise ValueError("Unexpected local-repair verdict")
                description = (f"The local repair search failed after evaluating {document.get('evaluated_arrangements')} arrangements "
                    f"with beam width {document.get('beam_width')} and at most {document.get('maximum_swaps')} swaps. "
                    "It produced no accepted arrangement and launched no physics run. "
                    + str(document.get("scope", "This bounded local search does not prove global infeasibility.")))
            else:
                description = "Additional finite-search evidence; no full-state validation claim."
            records.append({"path": relative, "sha256": entry["sha256"], "description": description})
        except (OSError, ValueError, KeyError, TypeError) as exc:
            result["problems"].append(f"Auxiliary search {entry.get('path', 'unrecorded')}: {exc}")
    return records


def load_experiment(directory):
    directory = Path(directory).resolve()
    summary = read_json(directory / "summary.json")
    problems, rows, hashes = [], [], {"summary.json": digest(directory / "summary.json")}
    entries = summary.get("states", [])
    if len({entry.get("source_state_id") for entry in entries}) != len(entries):
        problems.append("Source state IDs are duplicated")
    for entry in entries:
        row = {"entry": entry, "source_state_id": entry.get("source_state_id", "unknown"), "status": "unresolved", "problems": []}
        try:
            source_path = portable_path(entry["source_state"], directory)
            source = read_json(source_path)
            source_counts = validate_source_state(source)
            if source["state_id"] != row["source_state_id"]:
                raise ValueError("Source file and source_state_id differ")
            if entry.get("counts", {}).get("total") != source_counts["total"]:
                raise ValueError("Summary source count differs from original inventory")
            for key in ("by_kind", "by_rack"):
                if {k: v for k, v in entry.get("counts", {}).get(key, {}).items() if v} != source_counts[key]:
                    raise ValueError(f"Summary source {key} differs from original inventory")
            source_hash = digest(source_path)
            if entry.get("source_state_sha256") != source_hash:
                raise ValueError("Original source state differs from its recorded hash")
            hashes[str(source_path)] = source_hash
            row.update(source=source, source_path=source_path, source_sha256=source_hash,
                       counts=source_counts, initial_assessment=initial_assessment(entry.get("initial_organization")))
            copied_source = local_path(directory, f"inputs/source_run/states/{row['source_state_id']}.json")
            if copied_source.is_file():
                if digest(copied_source) != source_hash:
                    raise ValueError("Bundled original-state copy differs from the unchanged source")
                row["display_source_path"] = copied_source
                hashes[str(copied_source.relative_to(directory))] = source_hash
            if entry.get("status") == "accepted":
                path = local_path(directory, entry["accepted_state"])
                state = read_json(path)
                validate_organized_state(state, source)
                if state.get("source_state_sha256") != source_hash:
                    raise ValueError("Accepted counterpart does not hash the unchanged original state")
                attempt = state.get("source_attempt")
                result_path = local_path(directory, f"attempts/{attempt}/result.json")
                if read_json(result_path) != state["validation"]:
                    raise ValueError("Accepted state validation differs from saved attempt")
                reproduction = state["reproduction"]
                retry = reproduction.get("source_attempt")
                if not retry or retry == attempt:
                    raise ValueError("Reproduction must reference a separate fresh attempt")
                replay_path = local_path(directory, f"attempts/{retry}/result.json")
                if read_json(replay_path) != reproduction["validation"]:
                    raise ValueError("Fresh reproduction differs from its saved result")
                hashes.update({str(path.relative_to(directory)): digest(path), str(result_path.relative_to(directory)): digest(result_path),
                               str(replay_path.relative_to(directory)): digest(replay_path)})
                row.update(status="accepted", state=state, accepted_path=path, state_sha256=digest(path))
            elif entry.get("accepted_state"):
                raise ValueError("An unresolved row must not advertise an accepted state")
        except (OSError, ValueError, KeyError, TypeError) as exc:
            row["problems"].append(str(exc))
            problems.append(f"{row['source_state_id']}: {exc}")
        rows.append(row)
    attempts = []
    for path in sorted((directory / "attempts").glob("*/result.json")):
        try:
            result = read_json(path)
            attempts.append({"attempt_id": path.parent.name, "outcome": result.get("outcome", "unresolved"), "path": path})
            hashes[str(path.relative_to(directory))] = digest(path)
        except (OSError, ValueError) as exc:
            problems.append(f"Attempt {path.parent.name}: {exc}")
    counts = Counter(row["status"] for row in rows)
    result = {"directory": directory, "summary": summary, "rows": rows, "attempts": attempts,
              "attempt_outcomes": dict(Counter(row["outcome"] for row in attempts)),
              "accepted_count": counts["accepted"], "unresolved_count": counts["unresolved"],
              "problems": problems, "input_hashes": hashes}
    controls = []
    for name, entry in summary.get("controls", {}).items():
        try:
            path = local_path(directory, entry["result"])
            control = read_json(path)
            if entry.get("outcome") != control.get("outcome"):
                raise ValueError("Control index and result outcome differ")
            hashes[str(path.relative_to(directory))] = digest(path)
            controls.append({"name": name, "outcome": control.get("outcome"), "path": str(path.relative_to(directory))})
        except (OSError, ValueError, KeyError, TypeError) as exc:
            problems.append(f"Control {name}: {exc}")
    result["controls"] = controls
    if counts["accepted"] and {item["name"] for item in controls if item["outcome"] == "control_passed"} != {"empty_cycle", "blocked_door"}:
        problems.append("Accepted counterparts require passing empty-cycle and blocked-door controls")
    preservation = summary.get("source_preservation")
    preservation_path = directory / "source_preservation_audit.json"
    if preservation is not None:
        if preservation.get("result") != "PASS":
            problems.append("Original source preservation check failed")
        if not preservation_path.is_file() or read_json(preservation_path) != preservation:
            problems.append("Source preservation summary differs from its saved audit")
        else:
            hashes[preservation_path.name] = digest(preservation_path)
    result["images"] = image_records(result)
    # status=complete means the bounded run finished; unresolved inventories are valid results.
    if summary.get("status") == "complete" and len(rows) != 11:
        problems.append("The finished experiment must account for all eleven original inventories")
    if summary.get("status") == "complete" and {row["source_state_id"] for row in rows} != {"highest", *(f"random_{i:02d}" for i in range(10))}:
        problems.append("Finished source IDs must be highest and random_00 through random_09")
    if summary.get("status") == "complete" and any(row["entry"].get("status") not in {"accepted", "unresolved"} for row in rows):
        problems.append("Finished inventories must have finalized accepted or unresolved status")
    expected_views = {(row["source_state_id"], "before") for row in rows}
    expected_views |= {(row["source_state_id"], "after") for row in rows if row["status"] == "accepted"}
    result["missing_views"] = sorted(expected_views-set(result["images"]))
    if summary.get("status") == "complete" and result["missing_views"]:
        problems.append(f"Finished presentation lacks {len(result['missing_views'])} measured before/after views")
    if "attempt_count" in summary and summary["attempt_count"] != len(attempts):
        problems.append("Summary attempt count differs from saved result files")
    if "outcomes" in summary and {key: value for key, value in summary["outcomes"].items() if value} != result["attempt_outcomes"]:
        problems.append("Summary outcome counts differ from saved results")
    if "accepted_count" in summary and summary["accepted_count"] != counts["accepted"]:
        problems.append("Summary accepted count differs from audited counterparts")
    result["search_metadata"] = search_metadata(result)
    hashes.update(result["search_metadata"]["artifact_hashes"])
    result["auxiliary_search_evidence"] = auxiliary_search_evidence(result)
    result["audit_result"] = "PASS" if not problems else "INCOMPLETE"
    return result


def png_dimensions(path):
    header = Path(path).read_bytes()[:24]
    if len(header) != 24 or header[:8] != b"\x89PNG\r\n\x1a\n" or header[12:16] != b"IHDR":
        raise ValueError("Render is not a PNG with an IHDR header")
    return list(struct.unpack(">II", header[16:24]))


def image_records(result):
    rows = {row["source_state_id"]: row for row in result["rows"]}
    images = {}
    for path in sorted((result["directory"] / "renders").glob("*_render_evidence.json")):
        try:
            record = read_json(path)
            identity, view = record["source_state_id"], record["view"]
            row = rows[identity]
            if view not in {"before", "after"} or (view == "after" and row["status"] != "accepted"):
                continue
            state = row["source"] if view == "before" else row["state"]
            expected_hash = row["source_sha256"] if view == "before" else row["state_sha256"]
            image = local_path(result["directory"], "renders/" + record["image_file"])
            if (record.get("result") != "PASS" or record.get("state_sha256") != expected_hash or record.get("state_unchanged") is not True
                    or record.get("counts") != state_counts(state) or record.get("image_sha256") != digest(image)
                    or record.get("resolution") != [1920, 1440] or png_dimensions(image) != [1920, 1440]):
                raise ValueError("Image dimensions, state identity or image hash disagree")
            identities = set(COMPONENTS) | {obj["object_id"] for obj in state["objects"]}
            rendered = record.get("rendered_poses", {})
            if set(rendered) != identities:
                raise ValueError("Render omits an object or appliance component")
            for name in identities:
                error = pose_error(state["initial_snapshot"]["poses"][name], rendered[name])
                if error["position_error_m"] > 1e-7 or error["orientation_error_deg"] > 1e-4:
                    raise ValueError("Render differs from measured saved transforms")
            if (identity, view) in images:
                raise ValueError("Duplicate render for one source and view")
            images[(identity, view)] = {"path": str(image.relative_to(result["directory"])), "evidence": str(path.relative_to(result["directory"])),
                                       "image_sha256": digest(image), "evidence_sha256": digest(path), "label": record["label"]}
        except (OSError, ValueError, KeyError, TypeError) as exc:
            result["problems"].append(f"Render {path.name}: {exc}")
    return images


def organization_numbers(record):
    if not isinstance(record, dict):
        return "Unassessed"
    clearance = record.get("separation", {}).get("minimum_certified_clearance_m")
    exposure = [item.get("unobstructed_fraction") for item in record.get("opening_exposure", [])]
    exposure = [value for value in exposure if finite(value)]
    direct = sum(item.get("direct_rack_support") is True for item in record.get("per_object", {}).values())
    return (f"minimum certified clearance {clearance*1000:.2f} mm" if finite(clearance) else "clearance unrecorded or single item") + (
        f"; minimum mouth exposure {min(exposure):.1%}" if exposure else "; no vessel exposure recorded") + f"; direct rack support {direct} items"


def intro(result):
    summary = result["summary"]
    wall = summary.get("wall_seconds")
    elapsed = f"{wall:.1f} seconds ({wall/60:.1f} minutes)" if finite(wall) else "unrecorded"
    highest = next((row for row in result["rows"] if row["source_state_id"] == "highest"), None)
    highest_text = (f"The highest source inventory contains {highest.get('counts', {}).get('total', 'unrecorded')} items and its organized counterpart is {highest['status']}. "
                    if highest else "The highest source inventory is not indexed. ")
    paragraphs = [
        f"Accepted organized counterparts: {result['accepted_count']} of {len(result['rows'])} original states; {result['unresolved_count']} unresolved. "
        + highest_text + "Every accepted counterpart preserves the exact original object identities, kinds, dimensions and masses. Rack assignment may change.",
        f"Run status: {summary.get('status', 'unrecorded')}. Saved-evidence audit: {result['audit_result']}. "
        f"Seed: {summary.get('seed', 'unrecorded')}. Elapsed wall time: {elapsed}; configured total budget: {summary.get('budget_seconds', 'unrecorded')} seconds. "
        "The original packing artifacts remain the before evidence; they are not required to satisfy the new organization rules.",
        "Hard orientation rules: mug openings within 45° of downward; bowl openings within 75° of downward; dinner plates in the lower rack with "
        "their planes within 15° of vertical. All dish pairs require at least 5 mm geometric clearance. No nesting or stacking is allowed, and every item "
        "must have measured direct support from its assigned rack.",
        "The opening-exposure proxy launches 64 deterministic rays from points across each mug or bowl mouth, extending 100 mm outward along its "
        "opening normal. At least 80% must be unblocked by other dish visual triangles (at least 52 of 64 rays). This is a geometric proxy, "
        "not a simulation of water jets, drainage, detergent, cleaning quality, or loading accessibility.",
        "Soft preferences encourage upper-rack mugs, fewer occupied type rows, shared orientation families and compact centers, with seeded tie breaking. "
        "Measured row fragments, normal alignment, handle alignment and clearance are reported separately. The proposal solver does not globally "
        "optimize actual geometric clearance or contiguous grouping, and these preferences do not certify visual neatness.",
        "Pose sampling uses finite rack-derived rows and slots. Lower-rack plates occupy tine gaps with 0° or ±8° lean from vertical. "
        "Mug slots use channel grids with downward tilt families 0°, 15° and 30° and paired handle yaw directions. Bowl slots follow tine gaps with "
        "30°, 45°, 60° and 75° downward tilt in opposite directions. Refinement adds 5 mm slot offsets and additional mug yaw families. "
        "For each fixed horizontal position and orientation, a conservative vertical FCL distance search finds rack support with a small positive gap, "
        "then checks appliance contact and rack footprint. These first-contact seeds still require physical settling.",
        "Binary MILP selection chooses combinations of candidate poses: the sum of selected candidates of each dish type must equal that original "
        "inventory's exact type count, and each incompatible pair obeys x_i + x_j <= 1. Fixed-inventory failed combinations are excluded. "
        "After selecting placements, a Hungarian assignment permutes the original identities within each identical dish type, preferring fewer rack "
        "transfers and then shorter displacement. Identity assignment changes which existing item uses a pose; it does not add, remove or resize items. "
        "This is neither exhaustive search over continuous positions and SO(3) rotations nor uniform sampling of valid loads, and solver infeasibility "
        "applies only to the recorded finite catalog and constraints.",
        "Near-pair separation uses authored FCL collision geometry; distant pairs may use conservative AABB distance lower bounds. "
        "Nesting is tested against an inscribed convex cavity proxy for each mug or bowl. The mouth rays use a deterministic equal-area Fibonacci disk "
        "and two-sided intersection tests against other dish visual triangles. These explicit geometric approximations are preserved in the source and policy records.",
        "Each accepted state passes joint settling and both loaded rack retractions, actual door closure, a closed-door rest observation, reopening "
        "and loaded rack extension, followed by organized rest observation. It also passes a fresh independent reproduction of the complete cycle. "
        "Simulation uses the pinned Isaac Sim 4.5 runtime. Orientation, direct support, forbidden contacts and physical rest are checked at 120 Hz; "
        "full clearance, nesting and opening geometry are checked at 10 Hz and at final snapshots. Each final five-second passing observation has at least "
        "50 geometric samples. Clearance between geometry samples is not guaranteed. Door endpoints allow 0.5° error, with measured door speed capped at 0.65 rad/s.",
        "Search is bounded by the recorded proposal settings and time allocation. An unresolved counterpart means this run did not establish a valid "
        "organized arrangement for that exact inventory. It does not prove impossibility or a global capacity limit. Before/after images replay "
        "measured saved poses in Isaac RTX without advancing physics; transform and image hashes accompany each view.",
    ]
    if summary.get("screening") is not None:
        paragraphs.insert(-1,
            "The search also screens individual candidates in a shared Isaac session with three reusable dish actors, one active candidate at a time "
            "and the other actors parked away. Each passing candidate must complete the same five-second organization, physical-rest and direct-rack-support "
            "observation. Its measured settled rack-local pose becomes a proposal in a new catalog. A screening_passed record validates only that isolated "
            "candidate; it is not acceptance of a jointly loaded state or a complete rack-and-door cycle. Every full inventory still requires a fresh "
            "primary cycle and a separate fresh reproduction. Screening consumes the original two-hour budget, preserves the highest 35-item target, "
            "and does not change the acceptance thresholds. Screening settings, counts and provenance remain in summary.json.")
    metadata = result.get("search_metadata", {})
    catalog, graph, solver = metadata.get("catalog", {}), metadata.get("graph", {}), metadata.get("solver", {})
    if catalog:
        paragraphs.append(f"Current adopted catalog: {catalog['candidate_count']} candidate poses; actual candidate types {catalog['count_by_kind']}. "
            f"Source pattern processing: {catalog.get('processed_patterns')} of {catalog.get('pattern_count')}. "
            f"Current graph: {graph.get('allowed_count', 'unrecorded')} allowed candidates, {graph.get('conflict_count', 'unrecorded')} "
            f"incompatible pairs; status {graph.get('status', 'unrecorded')}. Graph scope: {graph.get('bound_scope', 'unrecorded')}. "
            "Pairwise compatibility does not establish aggregate opening exposure or joint physical stability.")
    for phase in metadata.get("screening_phases", []):
        shown = lambda key: phase.get(key) if phase.get(key) is not None else "unrecorded"
        paragraphs.append(f"Recorded {phase['phase']} screening: status {shown('outcome')}; {shown('candidate_count')} passing proposals "
            f"from {shown('attempt_count')} isolated attempts in a source pool of {shown('source_candidate_count')}; "
            f"adopted into search: {phase.get('adopted')}. Unknown counts remain unrecorded; expansion passes are not added to the active catalog "
            "until its merged provenance is recorded, and none of these proposal counts are full-state acceptances.")
    if solver.get("calls"):
        paragraphs.append(f"Recorded proposal-search MILP calls: {solver['calls']}; solver time {solver['elapsed_seconds']:.3f} seconds; status counts "
            f"{solver['status_counts']}; finite-catalog feasibility outcomes {solver['feasibility_counts']}. Per-call seeds, type constraints, "
            "objectives, solver messages and graph provenance remain in summary.json and the indexed catalog files.")
    prechecks = metadata.get("geometric_prechecks", {})
    if prechecks.get("inventory_records"):
        paragraphs.append(f"Separate geometric prechecks: {prechecks['solver_calls']} actual solver calls across {prechecks['catalog_graph_domains']} "
            f"catalog/graph domains, {prechecks['solver_elapsed_seconds']:.3f} seconds, status counts {prechecks['solver_status_counts']}; "
            f"{prechecks['unassessed_records']} records were unassessed without a solve. The current domain has "
            f"{prechecks['current_domain_proven_infeasible_inventories']} distinct inventories recorded as proven infeasible. "
            "These calls are separate from proposal-search solver_records. Each distinct inventory receives at most ten seconds per current domain, "
            "using the complete geometric graph and no exclusions from failed physical trials. Only proven finite-catalog infeasibility skips further "
            "allocation; the inventory remains unresolved. Unknown results, timeouts and pending independent replays remain scheduled. "
            "Catalog and graph hashes bind each cached proof; a changed domain requires a new precheck.")
    return paragraphs


def markdown(result):
    lines = ["# Organized dishwasher counterparts", "", *sum(([text, ""] for text in intro(result)), [])]
    lines += ["| Source | Inventory | Original organization | Organized counterpart | Attempts |", "| --- | ---: | --- | --- | ---: |"]
    for row in result["rows"]:
        original = row.get("initial_assessment", {}).get("status", "UNASSESSED")
        source_link = link_path(row.get("display_source_path", row["source_path"]), result["directory"]) if "source_path" in row else "summary.json"
        status = f"[accepted]({link_path(row['accepted_path'], result['directory'])})" if row["status"] == "accepted" else "UNRESOLVED"
        lines.append(f"| [{row['source_state_id']}]({source_link}) | {row.get('counts', {}).get('total', 'unknown')} | {original} | {status} | {len(row['entry'].get('attempt_ids', []))} |")
    lines += ["", "Attempt outcomes: " + "; ".join(f"{key}: {value}" for key, value in result["attempt_outcomes"].items()) + ".", ""]
    lines += ["Controls: " + "; ".join(f"[{item['name']}]({item['path']}): {item['outcome']}" for item in result["controls"]) + ".", ""]
    for record in result.get("auxiliary_search_evidence", []):
        lines += [record["description"] + f" [Recorded evidence]({record['path']})", ""]
    for row in result["rows"]:
        lines += [f"## {row['source_state_id']}", ""]
        counts = row.get("counts", {})
        lines += [f"Original inventory: {counts.get('total', 'unknown')}; by kind: {counts.get('by_kind', {})}. "
                  f"Counterpart status: {row['status'].upper()}.", ""]
        if row["status"] != "accepted":
            lines += ["Unresolved reason: " + str(row["entry"].get("reason", "No reproduced full-inventory counterpart has been established.")), ""]
            if row["entry"].get("provisional_state"):
                lines += ["A provisional primary-cycle record exists; it is not an accepted counterpart without a passing independent reproduction.", ""]
        assessment = row.get("initial_assessment", {})
        lines += [f"Original hard-rule violations: {assessment.get('violations', {})}; direct support assessed: {assessment.get('direct_support_assessed', False)}.", ""]
        if row["status"] == "accepted":
            lines += ["Accepted measured initial organization: " + organization_numbers(row["state"]["validation"]["organization_initial"]) + ".", "",
                      "Soft preferences: " + json.dumps(row["state"]["validation"]["organization_initial"].get("preferences", {}), sort_keys=True) + ".", ""]
        for view in ("before", "after"):
            image = result["images"].get((row["source_state_id"], view))
            if image:
                lines += [f"![{image['label']}]({image['path']})", f"[{view.title()} render audit]({image['evidence']})", ""]
            elif view == "after" and row["status"] != "accepted":
                lines += ["After: unresolved; no accepted counterpart is shown.", ""]
            else:
                lines += [f"{view.title()} image: not available.", ""]
    if result["problems"]:
        lines += ["Saved-evidence issues:", "", *("- " + problem for problem in result["problems"]), ""]
    lines += ["[Run summary, search settings, limits and source hashes](summary.json) · [Saved-evidence audit](report_audit.json) · "
              "[PDF](technical_report.pdf). Attempt directories preserve proposals, runtime results, traces and logs. "
              "No failed or unresolved attempt is relabeled accepted.", ""]
    return "\n".join(lines)


def html_report(result):
    esc = html.escape
    content = ["<!doctype html><html lang='en'><meta charset='utf-8'><title>Organized dishwasher counterparts</title>",
               "<style>body{font:16px/1.5 system-ui;max-width:1200px;margin:36px auto;padding:0 24px;color:#172838;background:#f6f8fa}"
               "h1{font-size:34px}h2{font-size:25px}.pairs,.headlines{display:grid;grid-template-columns:1fr 1fr;gap:18px}"
               "figure{margin:0}img{width:100%;height:auto;display:block}.state{margin:30px 0;padding:20px;background:white;border:1px solid #dce4eb}"
               ".missing{padding:40px 18px;background:#edf1f5;color:#526477}table{width:100%;border-collapse:collapse}"
               "th,td{padding:8px;border-bottom:1px solid #dce4eb;text-align:left}a{color:#176390}code{overflow-wrap:anywhere}"
               "@media(max-width:700px){.pairs,.headlines{grid-template-columns:1fr}}"
               "@media print{@page{size:A4;margin:12mm}body{font-size:9pt;line-height:1.35;background:white;padding:0;margin:0;max-width:none}"
               "h1{font-size:22pt}h2{font-size:15pt}.state{break-inside:avoid;padding:8px;margin:12px 0}.pairs{gap:8px}"
               "th,td{padding:4px}table{font-size:8pt}p{orphans:3;widows:3}.headlines{break-inside:avoid}}</style>",
               "<h1>Organized dishwasher counterparts</h1>"]
    content.extend("<p>" + esc(text) + "</p>" for text in intro(result))
    headlines = [result["images"].get((identity, "after")) for identity in ("highest", "random_03")]
    if any(headlines):
        content.append("<h2>Validated organized examples</h2><div class='headlines'>")
        for image in headlines:
            if image:
                content.append(f"<figure><img src='{esc(image['path'], quote=True)}' alt='{esc(image['label'], quote=True)}'><figcaption>{esc(image['label'])}</figcaption></figure>")
        content.append("</div>")
    content.append("<h2>All source inventories</h2><table><thead><tr><th>Source</th><th>Items</th><th>Original organization</th><th>Counterpart</th><th>Attempts</th></tr></thead><tbody>")
    for row in result["rows"]:
        content.append(f"<tr><td><a href='#{esc(row['source_state_id'], quote=True)}'>{esc(row['source_state_id'])}</a></td>"
                       f"<td>{row.get('counts', {}).get('total', 'unknown')}</td><td>{esc(row.get('initial_assessment', {}).get('status', 'UNASSESSED'))}</td>"
                       f"<td>{esc(row['status'].upper())}</td><td>{len(row['entry'].get('attempt_ids', []))}</td></tr>")
    content.append("</tbody></table><p>Attempt outcomes: " + esc(str(result["attempt_outcomes"])) + ".</p>")
    content.append("<p>Controls: " + "; ".join(f"<a href='{esc(item['path'], quote=True)}'>{esc(item['name'])}</a>: {esc(item['outcome'])}" for item in result["controls"]) + ".</p>")
    for record in result.get("auxiliary_search_evidence", []):
        content.append("<p>" + esc(record["description"]) + f" <a href='{esc(record['path'], quote=True)}'>Recorded evidence</a></p>")
    for row in result["rows"]:
        identity = row["source_state_id"]
        content.append(f"<section class='state' id='{esc(identity, quote=True)}'><h2>{esc(identity)} · {esc(row['status'].upper())}</h2>")
        content.append("<p>Exact original inventory: " + esc(str(row.get("counts", {}))) + ".</p>")
        if row["status"] != "accepted":
            content.append("<p>Unresolved reason: " + esc(str(row["entry"].get("reason", "No reproduced full-inventory counterpart has been established."))) + "</p>")
            if row["entry"].get("provisional_state"):
                content.append("<p>A provisional primary-cycle record exists; it is not an accepted counterpart without a passing independent reproduction.</p>")
        assessment = row.get("initial_assessment", {})
        content.append("<p>Original organization: " + esc(assessment.get("status", "UNASSESSED")) + "; violations: " + esc(str(assessment.get("violations", {})))
                       + "; direct support assessed: " + str(assessment.get("direct_support_assessed", False)) + ".</p>")
        if row["status"] == "accepted":
            org = row["state"]["validation"]["organization_initial"]
            content.append("<p>Accepted measured initial organization: " + esc(organization_numbers(org)) + ".</p><p>Soft preferences: "
                           + esc(str(org.get("preferences", {}))) + ".</p>")
        content.append("<div class='pairs'>")
        for view in ("before", "after"):
            image = result["images"].get((identity, view))
            if image:
                content.append(f"<figure><img src='{esc(image['path'], quote=True)}' alt='{esc(image['label'], quote=True)}'>"
                               f"<figcaption><a href='{esc(image['evidence'], quote=True)}'>{esc(image['label'])} · transform audit</a></figcaption></figure>")
            else:
                text = "After: unresolved; no accepted counterpart is shown." if view == "after" and row["status"] != "accepted" else f"{view.title()} image not available."
                content.append("<div class='missing'>" + esc(text) + "</div>")
        content.append("</div><p>")
        if "source_path" in row:
            content.append(f"<a href='{link_path(row.get('display_source_path', row['source_path']), result['directory'])}'>Original measured state</a>")
        if row["status"] == "accepted":
            content.append(f" · <a href='{link_path(row['accepted_path'], result['directory'])}'>Accepted counterpart and reproduction</a>")
        content.append("</p></section>")
    if result["problems"]:
        content.append("<h2>Saved-evidence issues</h2><ul>" + "".join("<li>"+esc(value)+"</li>" for value in result["problems"]) + "</ul>")
    content.append("<p><a href='summary.json'>Run summary, search settings, limits and hashes</a> · <a href='report_audit.json'>Saved-evidence audit</a> · "
                   "<a href='technical_report.pdf'>Technical PDF</a>. Attempt directories preserve proposals, results, traces and logs. "
                   "The original artifacts and every failed or unresolved verdict are retained.</p></html>")
    return "\n".join(content)


def preload_pdf_greenlet():
    """Use Isaac's existing CPython 3.10 wheel without importing its other deps."""
    if sys.version_info[:2] != (3, 10) or "greenlet" in sys.modules:
        return
    package = Path("/isaac-sim/extscache/omni.services.pip_archive-0.13.6+lx64/pip_prebundle/greenlet")
    extension = package / "_greenlet.cpython-310-x86_64-linux-gnu.so"
    if not (package / "__init__.py").is_file() or not extension.is_file():
        raise RuntimeError("The existing Isaac CPython 3.10 greenlet dependency is unavailable")
    spec = importlib.util.spec_from_file_location(
        "greenlet", package / "__init__.py", submodule_search_locations=[str(package)])
    if spec is None or spec.loader is None:
        raise RuntimeError("Cannot load the existing Isaac greenlet package")
    module = importlib.util.module_from_spec(spec)
    previous_names = set(sys.modules)
    sys.modules["greenlet"] = module
    try:
        spec.loader.exec_module(module)
    except BaseException:
        for name in set(sys.modules) - previous_names:
            if name == "greenlet" or name.startswith("greenlet."):
                sys.modules.pop(name, None)
        raise


def export_pdf(directory, expected_images):
    support = ROOT / "outputs/viewer_validation"
    os.environ["PLAYWRIGHT_BROWSERS_PATH"] = str(support / "browsers")
    os.environ["XDG_CACHE_HOME"] = str(support / "runtime_cache")
    os.environ["TMPDIR"] = str(support / "tmp")
    sys.path.insert(0, str(support / "deps"))
    preload_pdf_greenlet()
    from playwright.sync_api import sync_playwright
    source, output = directory / "organized_report.html", directory / "technical_report.pdf"
    errors, requests = [], []
    with sync_playwright() as playwright:
        browser = playwright.chromium.launch(headless=True, args=["--no-sandbox"])
        page = browser.new_page(offline=True, viewport={"width": 1280, "height": 1100})
        page.on("pageerror", lambda error: errors.append(str(error)))
        page.on("request", lambda request: requests.append(request.url))
        page.goto(source.as_uri(), wait_until="load")
        if page.locator("h1").inner_text() != "Organized dishwasher counterparts":
            raise ValueError("Unexpected PDF source heading")
        if page.locator("img").count() != expected_images or not page.evaluate("Array.from(document.images).every(i=>i.complete&&i.naturalWidth===1920&&i.naturalHeight===1440)"):
            raise ValueError("PDF export has missing or incorrectly sized Isaac images")
        if not page.evaluate("document.documentElement.scrollWidth<=innerWidth"):
            raise ValueError("HTML report overflows its viewport")
        page.screenshot(path=str(directory / "report_preview.png"))
        page.pdf(path=str(output), format="A4", print_background=True, prefer_css_page_size=True, display_header_footer=True,
                 header_template="<span></span>", footer_template='<div style="font-size:8px;color:#667;width:100%;text-align:center">Organized dishwasher counterparts · <span class="pageNumber"></span> / <span class="totalPages"></span></div>')
        browser.close()
    remote = [url for url in requests if url.startswith(("http://", "https://"))]
    if errors or remote:
        raise ValueError(f"PDF export errors or remote requests: {errors}, {remote}")
    evidence = {"result": "PASS", "html_sha256": digest(source), "pdf_sha256": digest(output), "export_source_sha256": digest(__file__),
                "pdf_bytes": output.stat().st_size, "image_occurrences": expected_images, "all_images_loaded": True,
                "javascript_errors": errors, "remote_requests": remote}
    (directory / "pdf_validation.json").write_text(json.dumps(evidence, indent=2) + "\n")
    return evidence


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out-dir", type=Path, required=True)
    parser.add_argument("--pdf", action="store_true", help="Export with the existing offline Playwright installation")
    args = parser.parse_args(argv)
    directory = args.out_dir.resolve()
    audit = {"schema_version": 1, "result": "INCOMPLETE", "created_utc": datetime.now(timezone.utc).isoformat(),
             "problems": ["Report generation has not completed"],
             "source_hashes": {str(Path(__file__).relative_to(ROOT)): digest(__file__)}}
    try:
        # Invalidate a previous PASS before loading inputs or producing any output.
        # A failed PDF export or interrupted regeneration must not leave stale approval.
        write_audit(directory, audit)
        result = load_experiment(args.out_dir)
        directory = result["directory"]
        html_text = html_report(result)
        for filename, value in (("organized_report.md", markdown(result)), ("organized_report.html", html_text), ("index.html", html_text)):
            (directory / filename).write_text(value)
        audit = {"schema_version": 1, "result": result["audit_result"], "created_utc": datetime.now(timezone.utc).isoformat(),
                 "accepted_count": result["accepted_count"], "unresolved_count": result["unresolved_count"], "source_state_count": len(result["rows"]),
                 "accepted_state_ids": sorted(row["source_state_id"] for row in result["rows"] if row["status"] == "accepted"),
                 "unresolved_state_ids": sorted(row["source_state_id"] for row in result["rows"] if row["status"] != "accepted"),
                 "attempt_count": len(result["attempts"]), "attempt_outcomes": result["attempt_outcomes"], "problems": result["problems"],
                 "controls": result["controls"], "missing_views": result["missing_views"],
                 "search_metadata": result["search_metadata"],
                 "auxiliary_search_evidence": result["auxiliary_search_evidence"],
                 "input_hashes": result["input_hashes"], "render_evidence": [dict(source_state_id=key[0], view=key[1], **value) for key, value in result["images"].items()],
                 "source_hashes": {str(Path(__file__).relative_to(ROOT)): digest(__file__)},
                 "scope": "Saved counterpart identity, recorded organization/door/reproduction gates and render-transform integrity; no real-world cleaning or global capacity certificate."}
        if args.pdf:
            audit["pdf"] = export_pdf(directory, html_text.count("<img "))
        audit["output_hashes"] = {name: digest(directory / name) for name in ("organized_report.md", "organized_report.html", "index.html")}
        if args.pdf:
            audit["output_hashes"].update({name: digest(directory / name) for name in ("technical_report.pdf", "pdf_validation.json", "report_preview.png")})
        write_audit(directory, audit)
        print(f"[RESULT] {result['audit_result']}: {result['accepted_count']} accepted; {result['unresolved_count']} unresolved", flush=True)
        return 0 if result["audit_result"] == "PASS" else 1
    except Exception as exc:
        audit.update(result="INCOMPLETE", problems=[*audit.get("problems", []), f"Report generation failed: {exc}"])
        try:
            write_audit(directory, audit)
        except OSError:
            pass
        print(f"[RESULT] INCOMPLETE: {exc}", file=sys.stderr, flush=True)
        return 1


def write_audit(directory, audit):
    path = directory / "report_audit.json"
    temporary = None
    try:
        with tempfile.NamedTemporaryFile("w", dir=directory, prefix=".report_audit_", suffix=".json", delete=False) as stream:
            temporary = Path(stream.name)
            json.dump(audit, stream, indent=2, allow_nan=False)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        temporary.chmod(0o644)
        temporary.replace(path)
    finally:
        if temporary is not None and temporary.exists():
            temporary.unlink()


if __name__ == "__main__":
    raise SystemExit(main())
