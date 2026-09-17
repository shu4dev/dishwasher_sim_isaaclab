#!/usr/bin/env python3
"""Compose a factual report from saved multi-dish initial-state experiment records.

    scripts/run_py.sh frigidaire/scripts/evaluation/frigidaire_initial_state_report.py --out-dir <run>

Missing states or interrupted attempts remain explicit. A finite candidate search
does not establish a global maximum over continuous positions and orientations.
"""
from __future__ import annotations

import argparse
from collections import Counter
import hashlib
import html
import json
import math
from pathlib import Path
import sys


KINDS = ("dinner_plate", "bowl", "mug")
RACKS = ("LowerRack", "UpperRack")
COMPONENTS = ("Cabinet", "Door", "LowerRack", "UpperRack", "SilverwareBasket")
CLASSIFIED_FAILURES = {"initial_collision", "settle_failure", "penetration_failure", "closure_failure", "outside_dishwasher"}


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read_json(path):
    return json.loads(Path(path).read_text())


def contained_path(directory, relative):
    path = (directory / relative).resolve()
    if directory.resolve() not in path.parents:
        raise ValueError(f"Evidence path escapes experiment: {relative}")
    return path


def finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def validate_pose(pose):
    if not isinstance(pose, dict):
        raise ValueError("Missing measured pose")
    for key, length in (("position_m", 3), ("quaternion_xyzw", 4)):
        values = pose.get(key)
        if not isinstance(values, list) or len(values) != length or not all(finite(x) for x in values):
            raise ValueError(f"Invalid pose {key}")
    if abs(sum(x*x for x in pose["quaternion_xyzw"])-1.) > 2e-5:
        raise ValueError("Pose quaternion is not normalized")
    return pose


def pose_error(first, second):
    validate_pose(first)
    validate_pose(second)
    position = math.sqrt(sum((a-b)**2 for a, b in zip(first["position_m"], second["position_m"])))
    a, b = first["quaternion_xyzw"], second["quaternion_xyzw"]
    dot = abs(sum(x*y for x, y in zip(a, b))) / math.sqrt(sum(x*x for x in a)*sum(y*y for y in b))
    return {"position_error_m": position, "orientation_error_deg": math.degrees(2*math.acos(min(1., dot)))}


def state_counts(state):
    objects = state.get("objects")
    if not isinstance(objects, list):
        raise ValueError("State objects must be a list")
    return {"total": len(objects), "by_kind": dict(Counter(obj["kind"] for obj in objects)),
            "by_rack": dict(Counter(obj["rack"] for obj in objects))}


def validate_state(state):
    """Check state identity, measured open pose, and its explicit runtime verdict."""
    if state.get("schema_version") != 1 or state.get("accepted") is not True:
        raise ValueError("Rendering/reporting an accepted state requires schema 1 and accepted=true")
    if state.get("purpose") not in {"highest", "randomized"}:
        raise ValueError("Unknown initial-state purpose")
    validation = state.get("validation", {})
    if validation.get("outcome") != "accepted":
        raise ValueError("State lacks an accepted initial-state physics result")
    counts = state_counts(state)
    if counts["total"] < 1:
        raise ValueError("Accepted load must contain at least one dish")
    recorded = state.get("counts", {})
    if recorded.get("total") != counts["total"]:
        raise ValueError("Recorded state total differs from object count")
    for key in ("by_kind", "by_rack"):
        if {k: v for k, v in recorded.get(key, {}).items() if v} != counts[key]:
            raise ValueError(f"Recorded state {key} differs from objects")
    poses = state.get("initial_snapshot", {}).get("poses", {})
    for component in COMPONENTS:
        validate_pose(poses.get(component))
    identities = set()
    for obj in state["objects"]:
        identity = obj.get("object_id")
        if not isinstance(identity, str) or not identity or identity in identities or identity in COMPONENTS:
            raise ValueError("State object IDs must be nonempty and unique")
        identities.add(identity)
        if obj.get("kind") not in KINDS or obj.get("rack") not in RACKS:
            raise ValueError("Unknown dish kind or rack")
        validate_pose(obj.get("rack_local_pose"))
        error = pose_error(obj.get("pose_world"), poses.get(identity))
        if error["position_error_m"] > 1e-8 or error["orientation_error_deg"] > 1e-4:
            raise ValueError(f"Object pose and measured initial snapshot differ: {identity}")
    joints = state.get("initial_snapshot", {}).get("joints", {})
    for name, expected, tolerance in (("door_hinge", math.pi/2, math.radians(.5)),
                                       ("lower_slide", -.49, .005), ("upper_slide", -.44, .005)):
        if not finite(joints.get(name)) or abs(joints[name]-expected) > tolerance:
            raise ValueError(f"Measured initial state is not door-open/both-racks-extended: {name}")
    if validation.get("object_count") != counts["total"] or validation.get("initial_snapshot") != state["initial_snapshot"]:
        raise ValueError("State and runtime measured initial snapshot/object count differ")
    if validation.get("initial_geometry", {}).get("valid") is not True:
        raise ValueError("Initial geometry validation is missing or failed")
    motions = validation.get("loaded_rack_motions", [])
    if len(motions) != 2 or {motion.get("rack") for motion in motions} != set(RACKS):
        raise ValueError("Both loaded rack retractions must be recorded")
    holds = [("initial", validation.get("settled", {}), 5.), ("final", validation.get("final_hold", {}), 5.)]
    for motion in motions:
        if motion.get("passed") is not True:
            raise ValueError("A loaded rack retraction did not pass")
        speed = motion.get("peak_measured_rack_speed_m_s")
        if not finite(speed) or not 0 <= speed <= .10001:
            raise ValueError("Missing or excessive measured rack speed")
        holds.append((motion["rack"], motion.get("hold", {}), 0.))
    for label, hold, observation in holds:
        if not all(hold.get(key) is True for key in ("passed", "settled", "contacts_ok", "basket_supported", "dish_supported", "endpoints_ok")):
            raise ValueError(f"Missing or failed {label} physics hold")
        observed = hold.get("observation_completed_s")
        if not finite(observed) or observed+1e-8 < observation:
            raise ValueError(f"Insufficient continuous observation in {label} hold")
        if hold.get("rest_window_count", 0) < round(observation*120)+1:
            raise ValueError(f"Missing consecutive rest windows in {label} hold")
        if ("continuous_passing_window_count" in hold
                and hold["continuous_passing_window_count"] < round(observation*120)+1):
            raise ValueError(f"Interrupted passing-window sequence in {label} hold")
        for key, limit in (("peak_penetration_m", .002), ("median_max_penetration_m", .001)):
            if not finite(hold.get(key)) or not 0 <= hold[key] < limit:
                raise ValueError(f"Invalid {label} {key}")
        for identity in identities | {"SilverwareBasket"}:
            metrics = hold.get("motion", {}).get(identity, {})
            for key, limit in (("root_position_span_m", .005), ("quaternion_span_deg", 3.), ("peak_mesh_point_speed_m_s", .03)):
                if not finite(metrics.get(key)) or not 0 <= metrics[key] < limit:
                    raise ValueError(f"Invalid {label} rest evidence for {identity}: {key}")
            if metrics.get("sample_count", 0) < 121 or metrics.get("sample_duration_s", 0) < 1.:
                raise ValueError(f"Incomplete {label} rest window for {identity}")
    for key in ("maximum_settle_penetration_m", "maximum_cycle_penetration_m"):
        if not finite(validation.get(key)) or not 0 <= validation[key] < .002:
            raise ValueError(f"Invalid {key}")
    containment = validation.get("final_containment", {})
    if set(containment) != identities or any(any(value.get(key) is not True for key in
            ("contained", "world_contained", "cabinet_frame_contained")) for value in containment.values()):
        raise ValueError("Whole-mesh final containment evidence is incomplete or failed")
    return counts


def load_experiment(directory):
    directory = Path(directory).resolve()
    summary = read_json(directory / "summary.json")
    problems, states, attempts = [], [], []
    listed = summary.get("states", [])
    if len(set(listed)) != len(listed):
        problems.append("Summary lists a state path more than once")
    for relative in listed:
        try:
            path = contained_path(directory, relative)
            state = read_json(path)
            counts = validate_state(state)
            source_attempt = state.get("source_attempt")
            if not isinstance(source_attempt, str) or not source_attempt:
                raise ValueError("State has no source validation attempt")
            source_result = contained_path(directory, f"attempts/{source_attempt}/result.json")
            if read_json(source_result) != state["validation"]:
                raise ValueError("State validation differs from its saved source attempt")
            states.append({"path": str(path.relative_to(directory)), "state": state, "counts": counts,
                           "sha256": digest(path), "result": "PASS"})
        except (OSError, ValueError, KeyError, TypeError) as exc:
            problems.append(f"State {relative}: {exc}")
    for path in sorted((directory / "attempts").glob("*/result.json")):
        try:
            result = read_json(path)
            manifest_path = path.parent / "manifest.json"
            manifest = read_json(manifest_path) if manifest_path.is_file() else None
            attempts.append({"attempt_id": path.parent.name, "result": result, "manifest": manifest,
                             "path": str(path.relative_to(directory)), "sha256": digest(path)})
        except (OSError, ValueError, TypeError) as exc:
            problems.append(f"Attempt {path.parent.name}: {exc}")
    outcomes = dict(Counter(row["result"].get("outcome", "unrecorded") for row in attempts))
    if summary.get("attempt_count") != len(attempts):
        problems.append(f"Summary attempt_count={summary.get('attempt_count')} differs from {len(attempts)} saved results")
    if {k: v for k, v in summary.get("outcomes", {}).items() if v} != outcomes:
        problems.append("Summary outcome counts differ from saved attempt results")
    highest = [row for row in states if row["state"]["purpose"] == "highest"]
    randomized = [row for row in states if row["state"]["purpose"] == "randomized"]
    if len({row["state"].get("state_id") for row in states}) != len(states):
        problems.append("Published state IDs are not unique")
    if len(highest) > 1:
        problems.append("More than one highest-load state is published")
    highest_count = max((row["counts"]["total"] for row in highest), default=0)
    if summary.get("highest_count", 0) != highest_count:
        problems.append("Summary highest_count differs from saved highest state")
    if summary.get("randomized_count", 0) != len(randomized):
        problems.append("Summary randomized_count differs from saved randomized states")
    revisions = [{"path": str(path.relative_to(directory)), "sha256": digest(path), "record": read_json(path)}
                 for path in sorted((directory / "source_revisions").glob("*/revision.json"))]
    result = {"directory": directory, "summary": summary, "states": states, "attempts": attempts,
            "outcomes": outcomes, "problems": problems, "highest_count": highest_count,
            "randomized_count": len(randomized), "revisions": revisions,
            "audit_result": "PASS" if not problems else "INCOMPLETE"}
    result["catalog_metrics"] = catalog_metrics(result)
    result["solver_metrics"] = solver_metrics(result)
    result["runtime_profiles"] = runtime_profiles(result)
    result["audit_result"] = "PASS" if not problems else "INCOMPLETE"
    images = image_records(result)
    image_purposes = {record["purpose"] for record in images}
    if summary.get("status") == "complete" and (not highest or len(randomized) != 10 or len(images) != 2
                                                or image_purposes != {"highest", "randomized"}):
        problems.append("Complete experiment requires a highest state, ten randomized states, and two matching validated renders")
        result["audit_result"] = "INCOMPLETE"
    return result


def catalog_metrics(result):
    """Describe the saved candidate set without treating it as an inventory limit."""
    graph = result["summary"].get("catalog", {})
    metrics = {"candidate_count": graph.get("candidate_count"), "allowed_count": graph.get("allowed_count"),
               "conflict_count": graph.get("conflict_count"), "complete": graph.get("complete") is True,
               "unresolved_pairs": graph.get("unresolved_pairs"), "source_trial_count": None,
               "variants_per_template": None, "eligible_originals": None, "artifact_hashes": {}}
    path = result["directory"] / "candidates.json"
    if path.is_file():
        try:
            catalog = read_json(path)
            candidates = catalog["candidates"]
            metrics["artifact_hashes"]["candidates.json"] = digest(path)
            canonical_hash = hashlib.sha256(json.dumps(catalog, sort_keys=True, separators=(",", ":")).encode()).hexdigest()
            metrics["canonical_catalog_sha256"] = canonical_hash
            if graph.get("candidate_catalog_sha256") != canonical_hash:
                raise ValueError("Canonical candidate catalog differs from the graph's recorded hash")
            if catalog.get("candidate_count") != len(candidates) or len(candidates) != graph.get("candidate_count"):
                raise ValueError("Candidate catalog count differs from its graph")
            indices = graph.get("allowed_indices", [])
            if (len(set(indices)) != len(indices) or len(indices) != graph.get("allowed_count")
                    or any(not isinstance(i, int) or isinstance(i, bool) or not 0 <= i < len(candidates) for i in indices)):
                raise ValueError("Candidate graph has inconsistent eligible indices")
            metrics.update(source_trial_count=catalog.get("source_trial_count"),
                           variants_per_template=catalog.get("variants_per_template"),
                           eligible_originals=sum(candidates[i].get("variant_index") == 0 for i in indices))
        except (OSError, ValueError, KeyError, TypeError) as exc:
            result["problems"].append(f"Candidate catalog: {exc}")
    return metrics


def solver_metrics(result):
    """Aggregate recorded solver results; fixed-target bounds are never global graph bounds."""
    records = result["summary"].get("search", {}).get("solver_records", [])
    graph = result["catalog_metrics"]
    bounds = []
    for record in records:
        bound = record.get("geometric_cardinality_upper_bound")
        if (record.get("bound_scope") == "full finite compatibility graph" and record.get("target_count") is None
                and record.get("status") in (0, 1) and finite(bound) and bound >= 0 and bound == int(bound)
                and graph["complete"] and graph["unresolved_pairs"] == 0):
            count = record.get("count", 0)
            if not finite(count) or count > bound or (finite(graph["allowed_count"]) and bound > graph["allowed_count"]):
                result["problems"].append("A solver bound contradicts its recorded selection or eligible count")
                continue
            bounds.append(int(bound))
    counts = [row["count"] for row in records if finite(row.get("count"))]
    return {"record_count": len(records), "status_counts": dict(Counter(str(row.get("status", "unrecorded")) for row in records)),
            "elapsed_seconds": sum(row["elapsed_s"] for row in records if finite(row.get("elapsed_s")) and row["elapsed_s"] >= 0),
            "best_geometric_count": max(counts, default=None), "valid_full_graph_bound_count": len(bounds),
            "minimum_full_graph_bound": min(bounds, default=None), "maximum_full_graph_bound": max(bounds, default=None)}


def runtime_profiles(result):
    profiles = {}
    for row in result["attempts"]:
        runtime = row["result"].get("runtime", row["result"].get("runtime_settings", {}))
        hold = row["result"].get("settled", row["result"].get("last_hold", {}))
        profile = {key: runtime.get(key) for key in ("commanded_rack_speed_cap_m_s", "measured_rack_speed_cap_m_s",
                   "rack_speed_measurement_tolerance_m_s", "rest_checks_hz", "observation_seconds")}
        profile["observation_policy"] = ("requalification within 12-second allowance; failures recorded" if "observation_restarts" in hold
                                         else "first qualifying window starts observation" if hold else "not observed in saved result")
        key = json.dumps(profile, sort_keys=True)
        profiles.setdefault(key, {**profile, "attempt_ids": []})["attempt_ids"].append(row["attempt_id"])
    return list(profiles.values())


def revision_description(revision):
    record = revision["record"]
    name = record.get("revision", "Unspecified revision")
    if name == "commanded_rack_speed_095_to_080":
        return ("Commanded rack speed was reduced from 0.095 to 0.080 m/s after early tracking overshoot. "
                "The measured 0.10 m/s limit, its 0.00001 m/s numerical tolerance, and prior outcomes were unchanged.")
    if name == "permit_observation_restart_within_12_second_settle_allowance":
        return ("A failed observation can requalify within the original 12-second settling allowance. Each failed observation is retained; "
                "five uninterrupted passing seconds are still required, with a 17-second maximum and no restart after 12 seconds. "
                "Earlier failed attempts retain their original verdicts.")
    if name == "record_exact_120hz_peak_penetration_events":
        return ("Exact worst-penetration event snapshots were added because a 10 Hz regular trace can miss a brief peak measured at 120 Hz. "
                "The scalar penetration gates and earlier outcomes were unchanged.")
    return str(record.get("reason", name))


def image_records(result):
    records = []
    for path in sorted((result["directory"] / "renders").glob("*_render_evidence.json")):
        try:
            record = read_json(path)
            matching = [row for row in result["states"] if row["state"]["state_id"] == record.get("state_id")]
            image = contained_path(result["directory"], "renders/" + record["image_file"])
            if (record.get("result") == "PASS" and len(matching) == 1
                    and record.get("state_sha256") == matching[0]["sha256"]
                    and image.is_file() and digest(image) == record.get("image_sha256")
                    and record.get("state_unchanged") is True and record.get("resolution") == [1920, 1440]
                    and record.get("counts") == matching[0]["counts"]):
                state = matching[0]["state"]
                saved = state["initial_snapshot"]["poses"]
                identities = set(COMPONENTS) | {obj["object_id"] for obj in state["objects"]}
                rendered = record.get("rendered_poses", {})
                if set(rendered) != identities:
                    continue
                errors = [pose_error(saved[name], rendered[name]) for name in identities]
                if any(error["position_error_m"] > 1e-7 or error["orientation_error_deg"] > 1e-4 for error in errors):
                    continue
                records.append({"path": str(image.relative_to(result["directory"])), "label": record["label"],
                                "purpose": state["purpose"], "counts": matching[0]["counts"],
                                "evidence": str(path.relative_to(result["directory"])), "evidence_sha256": digest(path),
                                "image_sha256": record["image_sha256"]})
        except (OSError, ValueError, KeyError, TypeError):
            continue
    return records


def paragraphs(result):
    summary = result["summary"]
    wall = summary.get("wall_seconds")
    wall_label = f"{wall:.1f} seconds ({wall/60:.1f} minutes)" if finite(wall) else "not recorded"
    catalog, solver = result["catalog_metrics"], result["solver_metrics"]
    bound = solver["minimum_full_graph_bound"]
    source_count = catalog["source_trial_count"]
    variants = catalog["variants_per_template"]
    catalog_description = (f"The source catalog contains {source_count} reusable pose templates, each retained unchanged and expanded "
        f"with {variants} local perturbations ({catalog['candidate_count']:,} candidates before filtering). "
        if all(finite(value) for value in (source_count, variants, catalog["candidate_count"]))
        else "Candidate generation settings are preserved in the catalog and summary records. ")
    catalog_description += (f"Geometry filtering retained {catalog['allowed_count']} candidates, including {catalog['eligible_originals']} "
        f"unperturbed original templates, with {catalog['conflict_count']} incompatible candidate pairs. "
        if all(finite(catalog[key]) for key in ("allowed_count", "eligible_originals", "conflict_count")) else "")
    eligible = catalog["allowed_count"]
    subset_description = (f"There are 2^{eligible} possible subsets before conflicts, with independent sets in the graph specifying compatible selections. "
                          if isinstance(eligible, int) and eligible >= 0 else "Selections are independent sets of the finite conflict graph. ")
    unresolved = sum(count for outcome, count in result["outcomes"].items() if outcome not in CLASSIFIED_FAILURES | {"accepted"})
    supplemental = []
    if result['highest_count'] and result['solver_metrics']['minimum_full_graph_bound'] == result['highest_count']:
        supplemental.append('Capacity search ended early when the jointly validated load reached the finite-catalog geometric upper bound. '
            'Unused search time remained available for randomized-state validation, rendering and reporting; the two-hour budget was a maximum allowance.')
    supplemental.append('In each one-second rest window, position span is the largest coordinate range, angular deviation is measured '
        'from the first orientation in that window, and vertex speed is the maximum 120 Hz finite-difference speed over all authored visual vertices.')
    input_path = result['directory'] / 'input_validation.json'
    if input_path.is_file():
        catalog_specs = read_json(input_path).get('catalog', {})
        descriptions = []
        for kind in KINDS:
            spec = catalog_specs.get(kind, {})
            if spec.get('size_m') and finite(spec.get('mass_kg')):
                dimensions = ' × '.join(f'{dimension*1000:g}' for dimension in spec['size_m'])
                descriptions.append(f"{kind.replace('_', ' ')}: {dimensions} mm, {spec['mass_kg']:g} kg")
        if descriptions:
            supplemental.append('Fixed modeled tableware (actor-axis bounding dimensions and mass): ' + '; '.join(descriptions) +
                '. Both rack assignments were permitted regardless of catalog recommendations. These model dimensions, rigid collision shapes, '
                'friction and estimated mass properties have not been calibrated as a real dishwasher capacity measurement.')
    original_bound_path = result['directory'] / 'original_template_bound.json'
    if original_bound_path.is_file():
        original_bound = read_json(original_bound_path)
        if all(digest(result['directory'] / name) == expected
               for name, expected in original_bound.get('input_hashes', {}).items()) and original_bound.get('input_hashes'):
            solver = original_bound.get('solver', {})
            if solver.get('status') == 0 and solver.get('count') == solver.get('geometric_cardinality_upper_bound'):
                supplemental.append(f"Auxiliary geometric comparison: the {original_bound['eligible_originals']} eligible unperturbed source templates "
                    f"alone have a maximum compatible subset of {solver['count']} objects. This comparison is recorded in original_template_bound.json; "
                    "it is a geometric result, not an additional physics validation.")
    resumptions = result['summary'].get('resumptions', [])
    if resumptions:
        supplemental.append("The controller resumed from saved evidence after correcting premature exit on an empty greedy proposal. "
            "The original wall-clock deadline and completed trials were preserved. During resumed random-state generation, "
            "proposals alternate between uniformly drawn subsets of the highest validated candidate set and randomized graph greedy selection. "
            "Conditioning on uniqueness and joint physics acceptance makes the resulting states nonuniform. Resume records are linked in summary.json.")
    return [
        (f"Highest validated load found: {result['highest_count']} dishes. " if result['highest_count'] else "No validated loaded initial state was found. ") +
        (f"The smallest recorded full-graph upper bound is {bound} for the finite geometric candidate model. " if bound is not None
         else "No valid full-graph upper bound is recorded. ") +
        "These describe this modeled finite search; they do not establish the dishwasher's global or real-world capacity.",
        f"Validated randomized initial states: {result['randomized_count']} of 10 requested. "
        f"Saved validation attempts: {len(result['attempts'])}, including {unresolved} unresolved or infrastructure-error results. "
        f"Run status: {summary.get('status', 'unrecorded')}. Saved-record audit: {result['audit_result']}. "
        f"Elapsed wall time: {wall_label}; total budget: {summary.get('budget_seconds', 7200)} seconds. "
        f"Run seed: {summary.get('seed', 'unrecorded')}.",
        "Initial states have both racks extended and the door open. Each accepted state records a joint load, "
        "with all dishes present during validation. Stacks require a contact support chain reaching the selected rack; "
        "a seated silverware basket can provide a lower-rack support path.",
        "Search allocation: first 80 minutes for the highest-load search, 30 minutes for randomized states, "
        "and 10 minutes for rendering/reporting, within a total two-hour budget. " + catalog_description +
        "Position perturbations are uniform within a 10 mm ball; rotation axes are uniform and angles uniform from 0 to 10 degrees. "
        "These bounds apply to candidate generation. Physics settling may move a dish farther; measured initial poses are saved separately from proposals.",
        subset_description + "The search selects combinations of fixed position-and-orientation candidates. Greedy selection and MILP calls "
        "capped at 60 seconds propose loads. MILP uses a binary selection variable for each candidate and x_i + x_j <= 1 for every "
        "conflicting pair. A failed physical trial excludes only that exact candidate set; its supersets remain eligible because added dishes can provide support.",
        "Randomized-state targets are three loads near 25%, four near 50%, and three near 75% of the highest validated count, "
        "rounded upward with a minimum of one item. Targets decrease by one after every three failed tries when time permits. "
        "These seeded search results are not uniform samples over all valid arrangements; the state table records requested and actual sizes.",
        "Physics protocol: CPU simulation at 120 Hz with CCD; up to 12 seconds to settle, then five continuous seconds "
        "of passing one-second rest windows checked at every physics step. Retract the upper rack and then the lower rack; when needed, test the reverse "
        "order from a fresh baseline. Both racks must close successfully. Rack speed is capped at 0.10 m/s. "
        "Gates use 5 mm joint endpoint error, 1 mm whole-mesh containment tolerance, 2 mm peak penetration, "
        "1 mm median maximum rest penetration, root translation span below 5 mm, orientation span below 3 degrees, "
        "and peak speed below 0.03 m/s over every visual mesh point. Both world and measured Cabinet frames must satisfy whole-mesh containment.",
        "Initial placement is set directly in simulation. The experiment does not assess a dish-loading path, cleaning performance, "
        "or door closure. Geometry and physical simulation are model estimates, not a calibrated measurement of real appliance capacity. "
        "Missing, timed-out, or numerical-error validations remain unresolved; they are not proved impossible placements. "
        "Isaac renders reproduce the measured loaded initial snapshot and do not revalidate physics.",
    ] + supplemental


def evidence_rows(result):
    solver = result["solver_metrics"]
    return [
        ("Jointly validated load", str(result["highest_count"]), "Saved measured state and simultaneous physics checks; highest found in this search."),
        ("Finite-model upper bound", str(solver["minimum_full_graph_bound"]) if solver["minimum_full_graph_bound"] is not None else "Unrecorded",
         "Combinatorial bound for recorded fixed candidate geometry; no claim about all continuous poses or real capacity."),
        ("Solver calls", str(solver["record_count"]), f"Recorded MILP calls; {solver['elapsed_seconds']:.3f} seconds total solver time."),
        ("Best geometric selection", str(solver["best_geometric_count"]), "A candidate proposal; requires separate joint physical validation."),
        ("Valid full-graph bound range", f"{solver['minimum_full_graph_bound']} to {solver['maximum_full_graph_bound']}" if solver["valid_full_graph_bound_count"] else "Unrecorded",
         f"{solver['valid_full_graph_bound_count']} bounds; fixed-target bounds excluded."),
        ("Solver status counts", "; ".join(f"{key}: {value}" for key, value in solver["status_counts"].items()) or "None",
         "SciPy MILP: 0 = optimal, 1 = iteration/time limit; full messages remain in summary.json."),
    ]


def measurement_rows(result):
    rows = []
    for row in result["states"]:
        state = row["state"]
        validation = state["validation"]
        displacement = [value.get("translation_m") for value in validation.get("proposal_to_initial_displacement", {}).values()]
        displacement = [value for value in displacement if finite(value)]
        rows.append((state["state_id"], f"{validation['maximum_settle_penetration_m']*1000:.4f}",
                     f"{validation['maximum_cycle_penetration_m']*1000:.4f}",
                     f"{max(motion['peak_measured_rack_speed_m_s'] for motion in validation['loaded_rack_motions']):.6f}",
                     f"{validation['settled']['observation_completed_s']:.3f}",
                     f"{max(displacement)*1000:.3f}" if displacement else "Unrecorded"))
    return rows


def runtime_profile_description(profile):
    ids = profile["attempt_ids"]
    attempts = f"{len(ids)} attempts" + (f" ({', '.join(ids)})" if len(ids) <= 4 else f" (from {ids[0]} through {ids[-1]}; full membership in report_audit.json)")
    return (f"{attempts}: commanded rack cap {profile['commanded_rack_speed_cap_m_s']} m/s; measured cap "
            f"{profile['measured_rack_speed_cap_m_s']} m/s with {profile['rack_speed_measurement_tolerance_m_s']} m/s numerical tolerance; "
            f"rest checks {profile['rest_checks_hz']} Hz; observation {profile['observation_seconds']} seconds; "
            f"{profile['observation_policy']}.")


def markdown(result):
    lines = ["# Multi-dish initial-state experiment", ""]
    for text in paragraphs(result):
        lines.extend([text, ""])
    lines.extend(["| Quantity | Result | Interpretation |", "| --- | --- | --- |"])
    lines.extend("| " + " | ".join(row) + " |" for row in evidence_rows(result))
    lines.extend(["", "| State | Purpose | Requested target | Actual total | Dinner plates | Bowls | Mugs | Lower rack | Upper rack | Seed |",
                  "| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- |"])
    for row in result["states"]:
        state, counts = row["state"], row["counts"]
        cells = [f"[{state['state_id']}]({row['path']})", state["purpose"], str(state.get("requested_count", "—")), str(counts["total"]),
                 *(str(counts["by_kind"].get(kind, 0)) for kind in KINDS),
                 *(str(counts["by_rack"].get(rack, 0)) for rack in RACKS), str(state.get("seed", "unrecorded"))]
        lines.append("| " + " | ".join(cells) + " |")
    lines.extend(["", "Attempt outcomes: " + ("; ".join(f"{key}: {value}" for key, value in result["outcomes"].items()) or "none recorded") + ".", ""])
    lines.extend(["| State | Initial peak penetration (mm) | Retraction peak penetration (mm) | Peak measured rack speed (m/s) | Initial continuous observation (s) | Maximum proposal-to-initial movement (mm) |",
                  "| --- | ---: | ---: | ---: | ---: | ---: |"])
    lines.extend("| " + " | ".join(row) + " |" for row in measurement_rows(result))
    lines.append("")
    for image in image_records(result):
        lines.extend([f"![{image['label']}]({image['path']})", f"[{image['label']}: render transform audit]({image['evidence']})", ""])
    if result["problems"]:
        lines.extend(["Saved-evidence issues:", "", *("- " + value for value in result["problems"]), ""])
    if result["revisions"]:
        lines.extend(["Protocol revisions preserve the source before and after each change. Every attempt retains its actual runtime settings and original outcome.", ""])
        for revision in result["revisions"]:
            lines.extend([revision_description(revision) + f" [Revision record]({revision['path']})", ""])
    for profile in result["runtime_profiles"]:
        lines.extend([runtime_profile_description(profile), ""])
    lines.extend(["The [run index](summary.json) preserves all solver records, physics limits, input hashes, executable hashes, "
                  "seeds, timestamps and allocation settings. The [saved-record audit](report_audit.json) records artifact hashes, "
                  "compact solver statistics and runtime profile membership. [Candidate catalog](candidates.json) and "
                  "[compatibility graph](compatibility.json) preserve every candidate and tested geometric conflict.", "",
                  "Each linked state preserves measured initial poses and its validation result. Each `attempts/<id>/` "
                  "contains the proposed manifest, result, trace, and runtime log; the [attempt index](attempt_index.json) "
                  "includes failed and unresolved evaluations. The original single-dish experiment evidence is unchanged.", ""])
    return "\n".join(lines)


def html_report(result):
    escape = html.escape
    content = ["<!doctype html><html lang='en'><meta charset='utf-8'><title>Multi-dish initial-state experiment</title>",
               "<style>body{font:17px/1.55 system-ui;max-width:1120px;margin:40px auto;padding:0 24px;color:#182533;background:#f6f8fa}"
               "h1{font-size:36px}table{width:100%;border-collapse:collapse;background:white}th,td{padding:9px;border-bottom:1px solid #dce3e9;text-align:left}"
               "img{width:100%;height:auto}figure{margin:32px 0}pre{white-space:pre-wrap;background:#e8edf2;padding:18px;font-size:13px}a{color:#1263a0}</style>",
               "<h1>Multi-dish initial-state experiment</h1>"]
    content.extend("<p>" + escape(text) + "</p>" for text in paragraphs(result))
    content.append("<table><thead><tr><th>Quantity</th><th>Result</th><th>Interpretation</th></tr></thead><tbody>")
    content.extend("<tr>" + "".join("<td>"+escape(cell)+"</td>" for cell in row) + "</tr>" for row in evidence_rows(result))
    content.append("</tbody></table><h2>Validated states</h2><table><thead><tr><th>State</th><th>Purpose</th><th>Requested target</th><th>Actual total</th><th>By kind</th><th>By rack</th><th>Seed</th></tr></thead><tbody>")
    for row in result["states"]:
        state, counts = row["state"], row["counts"]
        content.append(f"<tr><td><a href='{escape(row['path'], quote=True)}'>{escape(state['state_id'])}</a></td>"
                       f"<td>{escape(state['purpose'])}</td><td>{state.get('requested_count', '—')}</td><td>{counts['total']}</td>"
                       f"<td>{escape(str(counts['by_kind']))}</td><td>{escape(str(counts['by_rack']))}</td><td>{escape(str(state.get('seed', 'unrecorded')))}</td></tr>")
    content.append("</tbody></table><p>Attempt outcomes: " + escape(str(result["outcomes"])) + "</p>")
    content.append("<table><thead><tr><th>State</th><th>Initial peak penetration (mm)</th><th>Retraction peak penetration (mm)</th>"
                   "<th>Peak measured rack speed (m/s)</th><th>Initial continuous observation (s)</th><th>Maximum proposal-to-initial movement (mm)</th></tr></thead><tbody>")
    content.extend("<tr>" + "".join("<td>"+escape(cell)+"</td>" for cell in row) + "</tr>" for row in measurement_rows(result))
    content.append("</tbody></table>")
    for image in image_records(result):
        content.append(f"<figure><img src='{escape(image['path'], quote=True)}' alt='{escape(image['label'], quote=True)}'>"
                       f"<figcaption><a href='{escape(image['evidence'], quote=True)}'>{escape(image['label'])} — render transform audit</a></figcaption></figure>")
    if result["problems"]:
        content.append("<h2>Saved-evidence issues</h2><ul>" + "".join("<li>"+escape(p)+"</li>" for p in result["problems"]) + "</ul>")
    if result["revisions"]:
        content.append("<h2>Recorded protocol revisions</h2><p>Source before and after every revision is preserved. "
                       "Each attempt retains its actual runtime settings and original outcome.</p>")
        for revision in result["revisions"]:
            content.append("<p>" + escape(revision_description(revision)) + f" <a href='{escape(revision['path'], quote=True)}'>Revision record</a></p>")
    content.extend("<p>"+escape(runtime_profile_description(profile))+"</p>" for profile in result["runtime_profiles"])
    content.append("<h2>Reproduction records</h2><p>The <a href='summary.json'>run index</a> preserves all solver records, physics limits, "
                   "input and executable hashes, seeds, timestamps and allocation settings. The <a href='report_audit.json'>saved-record audit</a> "
                   "contains artifact hashes and compact solver/runtime summaries. The <a href='candidates.json'>candidate catalog</a> and "
                   "<a href='compatibility.json'>compatibility graph</a> preserve every candidate and tested geometric conflict. "
                   "The <a href='attempt_index.json'>attempt index</a> retains failed and unresolved evaluations. Each attempt directory "
                   "contains its manifest, result, trace and runtime log. The original single-dish evidence is unchanged.</p></html>")
    return "\n".join(content)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out-dir", type=Path, required=True)
    args = parser.parse_args(argv)
    try:
        result = load_experiment(args.out_dir)
        (args.out_dir / "experiment_report.md").write_text(markdown(result))
        (args.out_dir / "experiment_report.html").write_text(html_report(result))
        audit = {"schema_version": 1, "result": result["audit_result"], "problems": result["problems"],
                 "state_count": len(result["states"]), "attempt_count": len(result["attempts"]),
                 "highest_count": result["highest_count"], "randomized_count": result["randomized_count"],
                 "catalog_metrics": result["catalog_metrics"], "solver_metrics": result["solver_metrics"],
                 "runtime_profiles": result["runtime_profiles"],
                 "validated_images": image_records(result),
                 "input_hashes": {"summary.json": digest(args.out_dir / "summary.json"),
                                  **result["catalog_metrics"]["artifact_hashes"],
                                  **{row["path"]: row["sha256"] for row in result["states"]},
                                  **{row["path"]: row["sha256"] for row in result["attempts"]},
                                  **{row["path"]: row["sha256"] for row in result["revisions"]}},
                 "source_hashes": {str(Path(__file__).resolve()): digest(__file__)},
                 "scope": "Saved-state record integrity; runtime acceptance remains in each state's validation record."}
        auxiliary = args.out_dir / 'original_template_bound.json'
        if auxiliary.is_file():
            audit['input_hashes']['original_template_bound.json'] = digest(auxiliary)
        (args.out_dir / "report_audit.json").write_text(json.dumps(audit, indent=2) + "\n")
        print(f"[RESULT] {result['audit_result']}: initial-state reports written", flush=True)
        return 0 if not result["problems"] else 1
    except (OSError, ValueError, KeyError, TypeError) as exc:
        print(f"[RESULT] INCOMPLETE: {exc}", file=sys.stderr, flush=True)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
