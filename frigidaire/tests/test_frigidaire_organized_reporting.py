"""Organized evidence cannot inherit acceptance from the original packing."""
from copy import deepcopy
import importlib.util
import json
import math
import os
from pathlib import Path
import struct
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "frigidaire/scripts/evaluation"))
from frigidaire_organized_report import (POLICY, digest, html_report, image_records,
    initial_assessment, inventory, load_experiment, main, markdown, orientation_error,
    portable_path, validate_inventory, validate_organization, validate_organized_state)
from frigidaire_organized_render import build_jobs, label_for

spec = importlib.util.spec_from_file_location("initial_fixture", Path(__file__).with_name("test_frigidaire_initial_state_reporting.py"))
fixture_module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(fixture_module)


def organization_fixture(objects):
    per_object = {obj["object_id"]: {"kind": obj["kind"], "rack": obj["rack"], "direct_rack_support": True,
                  "orientation": {"valid": True, "orientation_error_deg": 0.}} for obj in objects}
    exposure = [{"object_id": obj["object_id"], "unobstructed_rays": 64, "total_rays": 64,
                 "unobstructed_fraction": 1.} for obj in objects if obj["kind"] in {"mug", "bowl"}]
    return {"valid": True, "passed": True, "policy": deepcopy(POLICY), "violations": [],
            "per_object": per_object, "separation": {"minimum_certified_clearance_m": None},
            "nesting": {"intrusions": []}, "opening_exposure": exposure, "preferences": {"upper_rack_mugs": 0}}


def organized_fixture():
    source = fixture_module.state_fixture()
    state = deepcopy(source)
    state.update(purpose="organized", source_state_id="highest")
    for pose in (state["objects"][0]["pose_world"], state["objects"][0]["rack_local_pose"],
                 state["initial_snapshot"]["poses"]["mug_001"]):
        pose["quaternion_xyzw"] = [1., 0., 0., 0.]
    validation = state["validation"]
    validation["initial_snapshot"] = deepcopy(state["initial_snapshot"])
    closed = deepcopy(state["initial_snapshot"])
    closed["joints"] = {"door_hinge": 0., "lower_slide": 0., "upper_slide": 0.}
    validation.update(final_snapshot=closed, reopened_snapshot=deepcopy(state["initial_snapshot"]))
    organization = organization_fixture(state["objects"])
    for key in ("organization_initial", "organization_closed", "organization_reopened"):
        validation[key] = deepcopy(organization)
    for hold in [validation["settled"], validation["final_hold"]]:
        hold.update(organization_valid=True, organization_geometry_hz=10, continuous_observation_geometry_samples=50,
                    orientation_valid=True, forbidden_contacts_ok=True, direct_rack_support={"mug_001": True},
                    minimum_observed_clearance_m=None, minimum_observed_opening_exposure=1.)
    validation["door_closed_hold"] = deepcopy(validation["final_hold"])
    validation["reopened_hold"] = deepcopy(validation["settled"])
    validation["loaded_rack_extension_motions"] = [{"rack": rack, "passed": True} for rack in ("LowerRack", "UpperRack")]
    for key in ("door_close_motion", "door_open_motion"):
        validation[key] = {"passed": True, "measured_speed_ok": True, "peak_measured_door_speed_rad_s": .35}
    state["reproduction"] = {"result": "PASS", "source_attempt": "attempt_0001", "validation": deepcopy(validation)}
    return source, state


class OrganizedReportingTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory(prefix="organized-report-", dir=os.environ.get("INITIAL_STATE_TEST_OUTPUT_DIR"))
        self.addCleanup(temporary.cleanup)
        self.directory = Path(temporary.name)
        self.run = self.directory / "organized"
        self.run.mkdir()

    def save(self, path, data):
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(data, indent=2) + "\n")
        return path

    def write_run(self, accepted=True):
        source, state = organized_fixture()
        source_path = self.save(self.directory / "original/states/highest.json", source)
        state["source_state_sha256"] = digest(source_path)
        entry = {"source_state_id": "highest", "source_state": str(source_path), "source_state_sha256": digest(source_path),
                 "counts": source["counts"], "status": "accepted" if accepted else "unresolved",
                 "accepted_state": "states/highest.json" if accepted else None, "attempt_ids": [],
                 "initial_organization": {"valid": False, "violations": [{"rule": "orientation", "object_id": "mug_001"}],
                                          "per_object": {"mug_001": {"direct_rack_support": True}}}}
        summary = {"schema_version": 1, "source_run": str(source_path.parent.parent), "status": "running",
                   "seed": 5, "budget_seconds": 7200, "wall_seconds": 60, "states": [entry], "controls": {}}
        if accepted:
            self.save(self.run / "states/highest.json", state)
            self.save(self.run / "attempts/attempt_0000/result.json", state["validation"])
            self.save(self.run / "attempts/attempt_0001/result.json", state["reproduction"]["validation"])
            entry["attempt_ids"] = ["attempt_0000", "attempt_0001"]
            for control in ("empty_cycle", "blocked_door"):
                path = f"controls/{control}/result.json"
                self.save(self.run / path, {"outcome": "control_passed", "control": control})
                summary["controls"][control] = {"outcome": "control_passed", "result": path}
        self.save(self.run / "summary.json", summary)
        return source, state, summary

    def test_original_need_not_pass_organization(self):
        source, state = organized_fixture()
        self.assertEqual(orientation_error("mug", source["objects"][0]["pose_world"]), 180.)
        self.assertEqual(validate_organized_state(state, source)["total"], 1)

    def test_inventory_preserves_ids_kind_mass_and_size(self):
        source, state = organized_fixture()
        for key, value in (("object_id", "replacement_mug"), ("kind", "bowl"), ("mass_kg", .999), ("size_m", [1., 1., 1.])):
            with self.subTest(key=key):
                edited = deepcopy(state)
                edited["objects"][0][key] = value
                with self.assertRaisesRegex(ValueError, "exact source"):
                    validate_inventory(edited, source)

    def test_rack_transfer_is_allowed(self):
        source, state = organized_fixture()
        state["objects"][0]["rack"] = "UpperRack"
        self.assertEqual(inventory(state), inventory(source))

    def test_organization_flag_cannot_hide_wrong_measured_orientation(self):
        source, state = organized_fixture()
        with self.assertRaisesRegex(ValueError, "Orientation"):
            validate_organization(state["validation"]["organization_initial"], state["objects"], source["initial_snapshot"]["poses"])

    def test_changed_policy_rejected(self):
        source, state = organized_fixture()
        state["validation"]["organization_initial"]["policy"]["minimum_dish_clearance_m"] = .001
        with self.assertRaisesRegex(ValueError, "policy"):
            validate_organized_state(state, source)

    def test_insufficient_or_fabricated_exposure_rejected(self):
        source, state = organized_fixture()
        for count, fraction in ((51, 51/64), (52, 1.)):
            with self.subTest(count=count):
                edited = deepcopy(state)
                item = edited["validation"]["organization_initial"]["opening_exposure"][0]
                item.update(unobstructed_rays=count, unobstructed_fraction=fraction)
                with self.assertRaisesRegex(ValueError, "opening exposure"):
                    validate_organized_state(edited, source)

    def test_clearance_support_and_nesting_are_hard_gates(self):
        source, state = organized_fixture()
        objects = [state["objects"][0], dict(state["objects"][0], object_id="mug_002")]
        poses = {obj["object_id"]: obj["pose_world"] for obj in objects}
        for failure in ("clearance", "nesting", "support"):
            with self.subTest(failure=failure):
                record = organization_fixture(objects)
                record["separation"]["minimum_certified_clearance_m"] = .006
                if failure == "clearance":
                    record["separation"]["minimum_certified_clearance_m"] = .00499
                elif failure == "nesting":
                    record["nesting"]["intrusions"] = [{"vessel": "mug_001", "intruder": "mug_002"}]
                else:
                    record["per_object"]["mug_001"]["direct_rack_support"] = False
                with self.assertRaises(ValueError):
                    validate_organization(record, objects, poses)

    def test_50_geometry_samples_suffice_but_49_do_not(self):
        source, state = organized_fixture()
        validate_organized_state(state, source)
        state["validation"]["settled"]["continuous_observation_geometry_samples"] = 49
        with self.assertRaisesRegex(ValueError, "sampling"):
            validate_organized_state(state, source)

    def test_recorded_door_success_requires_actual_closed_endpoint(self):
        source, state = organized_fixture()
        state["validation"]["final_snapshot"]["joints"]["door_hinge"] = .1
        with self.assertRaisesRegex(ValueError, "endpoint"):
            validate_organized_state(state, source)

    def test_door_measured_speed_limit(self):
        source, state = organized_fixture()
        state["validation"]["door_close_motion"]["peak_measured_door_speed_rad_s"] = .66
        with self.assertRaisesRegex(ValueError, "door speed"):
            validate_organized_state(state, source)

    def test_fresh_reproduction_cannot_inherit_primary_acceptance(self):
        source, state = organized_fixture()
        state["reproduction"]["validation"].pop("organization_closed")
        with self.assertRaisesRegex(ValueError, "Organization"):
            validate_organized_state(state, source)

    def test_reproduction_also_requires_legacy_physical_gates(self):
        source, state = organized_fixture()
        state["reproduction"]["validation"]["maximum_cycle_penetration_m"] = .003
        with self.assertRaisesRegex(ValueError, "penetration"):
            validate_organized_state(state, source)

    def test_distinct_saved_reproduction_attempt_required(self):
        source, state, summary = self.write_run()
        state["reproduction"]["source_attempt"] = state["source_attempt"]
        self.save(self.run / "states/highest.json", state)
        self.assertIn("separate fresh attempt", str(load_experiment(self.run)["problems"]))

    def test_source_bytes_cannot_change(self):
        source, state, summary = self.write_run()
        source["seed"] = 999
        self.save(Path(summary["states"][0]["source_state"]), source)
        self.assertIn("recorded hash", str(load_experiment(self.run)["problems"]))

    def test_bundled_source_copy_gets_local_links_and_hash_binding(self):
        source, state, summary = self.write_run(accepted=False)
        copied = self.save(self.run / "inputs/source_run/states/highest.json", source)
        result = load_experiment(self.run)
        self.assertEqual(result["rows"][0]["display_source_path"], copied)
        self.assertIn("inputs/source_run/states/highest.json", markdown(result))
        self.assertIn("href='inputs/source_run/states/highest.json'", html_report(result))
        self.assertEqual(result["input_hashes"]["inputs/source_run/states/highest.json"], digest(copied))
        source["seed"] = 100
        self.save(copied, source)
        self.assertIn("Bundled original-state copy differs", str(load_experiment(self.run)["problems"]))

    def test_report_accepts_honest_unresolved_and_original_failure(self):
        self.write_run(accepted=False)
        result = load_experiment(self.run)
        self.assertEqual(result["audit_result"], "PASS")
        self.assertEqual(result["accepted_count"], 0)
        self.assertIn("UNRESOLVED", markdown(result))
        self.assertIn("no accepted counterpart is shown", html_report(result))
        self.assertNotIn("0 valid arrangements exist", markdown(result))

    def test_single_candidate_screening_is_not_full_load_acceptance(self):
        source, state, summary = self.write_run(accepted=False)
        summary["screening"] = {"status": "complete", "screening_passed": 35}
        self.save(self.run / "summary.json", summary)
        result = load_experiment(self.run)
        text = markdown(result)
        self.assertEqual(result["accepted_count"], 0)
        self.assertIn("shared Isaac session", text)
        self.assertIn("screening_passed record validates only that isolated candidate", text)
        self.assertIn("original two-hour budget", text)
        self.assertIn("fresh primary cycle and a separate fresh reproduction", text)

    def test_method_uses_actual_catalog_counts_and_keeps_expansion_separate(self):
        source, state, summary = self.write_run(accepted=False)
        self.save(self.run / "candidates_screened.json", {"candidates": [{"kind": "mug"}, {"kind": "bowl"}],
            "candidate_count": 2, "count_by_kind": {"mug": 999}, "processed_patterns": 20, "pattern_count": 100})
        self.save(self.run / "compatibility_screened.json", {"allowed_indices": [0, 1], "conflict_pairs": [[0, 1]], "status": "complete"})
        self.save(self.run / "candidate_screening/result.json", {"outcome": "screening_partial", "candidate_count": 2,
            "attempt_count": 3, "source_candidate_count": 10})
        summary.update(candidate_file="candidates_screened.json", compatibility_file="compatibility_screened.json",
            screening={"result": "candidate_screening/result.json", "catalog_adopted": True,
                       "expansion": {"status": "running", "source_candidate_count": 613}})
        self.save(self.run / "summary.json", summary)
        result = load_experiment(self.run)
        metadata = result["search_metadata"]
        self.assertEqual(metadata["catalog"]["count_by_kind"], {"mug": 1, "bowl": 1})
        self.assertEqual(metadata["catalog"]["candidate_count"], 2)
        self.assertFalse(metadata["screening_phases"][1]["adopted"])
        text = markdown(result)
        self.assertIn("Hungarian assignment", text)
        self.assertIn("x_i + x_j <= 1", text)
        self.assertIn("source pool of 613", text)
        self.assertEqual(result["accepted_count"], 0)

    def test_negative_search_evidence_binds_hashes_without_physics_claim(self):
        source, state, summary = self.write_run(accepted=False)
        path = self.save(self.run / "negative/capacity_result.json", {"graph_complete": True, "snapshot_candidate_count": 104,
            "inventory_ceilings": {"mug": 21, "bowl": 13, "dinner_plate": 1},
            "maximum_under_ceilings": {"status": 0, "count": 25, "integer_upper_bound": 25, "objective_dual_bound": -25.}})
        summary["auxiliary_search_records"] = [{"path": "negative/capacity_result.json", "sha256": digest(path), "full_state_validation_claim": False}]
        self.save(self.run / "summary.json", summary)
        result = load_experiment(self.run)
        self.assertIn("not a physically validated maximum", markdown(result))
        self.assertEqual(result["accepted_count"], 0)
        self.assertEqual(result["input_hashes"]["negative/capacity_result.json"], digest(path))
        path.write_text("{}")
        self.assertIn("auxiliary search hash differs", str(load_experiment(self.run)["problems"]))

    def test_precheck_calls_are_separate_and_unassessed_is_not_a_call(self):
        source, state, summary = self.write_run(accepted=False)
        summary["solver_records"] = [{"status": 0, "elapsed_s": 2.}]
        entry = {"catalog_sha256": "catalog", "graph_sha256": "graph", "inventories": {
            "one": {"proven_infeasible": True, "solver": {"status": 2, "elapsed_s": .2}},
            "two": {"proven_infeasible": False, "solver": {"status": 1, "elapsed_s": 10.}},
            "three": {"proven_infeasible": False, "solver": {"status": "unassessed"}}}}
        summary["geometric_prechecks"] = {"catalog": entry}
        summary["geometric_precheck_graph_history"] = [entry]
        self.save(self.run / "summary.json", summary)
        result = load_experiment(self.run)
        metadata = result["search_metadata"]
        self.assertEqual(metadata["solver"]["calls"], 1)
        self.assertEqual(metadata["geometric_prechecks"]["solver_calls"], 2)
        self.assertEqual(metadata["geometric_prechecks"]["unassessed_records"], 1)
        self.assertIn("Unknown results, timeouts and pending independent replays remain scheduled", markdown(result))

    def test_accepted_report_requires_both_controls(self):
        source, state, summary = self.write_run()
        summary["controls"].pop("blocked_door")
        self.save(self.run / "summary.json", summary)
        self.assertIn("blocked-door controls", str(load_experiment(self.run)["problems"]))

    def test_count_mismatch_rejected(self):
        source, state, summary = self.write_run()
        summary["attempt_count"] = 100
        self.save(self.run / "summary.json", summary)
        self.assertIn("attempt count", str(load_experiment(self.run)["problems"]))

    def test_batch_schedules_before_for_unresolved_without_after(self):
        self.write_run(accepted=False)
        jobs = build_jobs(SimpleNamespace(batch=self.run / "summary.json", state_ids=None))
        self.assertEqual([(job["source_state_id"], job["view"]) for job in jobs], [("highest", "before")])
        self.assertIn("Original packing", label_for(jobs[0]))

    def test_batch_schedules_both_views_for_accepted(self):
        self.write_run()
        jobs = build_jobs(SimpleNamespace(batch=self.run / "summary.json", state_ids=None))
        self.assertEqual([job["view"] for job in jobs], ["before", "after"])
        self.assertIn("Organized counterpart", label_for(jobs[1]))

    def test_portable_container_source_path(self):
        self.assertEqual(portable_path("/workspace/dishsim/frigidaire/docs/example.md"), ROOT / "frigidaire/docs/example.md")

    def test_original_unassessed_support_is_not_reported_pass(self):
        result = initial_assessment({"valid": False, "per_object": {"mug": {"direct_rack_support": None}}, "violations": []})
        self.assertFalse(result["direct_support_assessed"])

    def test_html_markdown_index_and_hash_audit_written(self):
        self.write_run()
        self.assertEqual(main(["--out-dir", str(self.run)]), 0)
        self.assertEqual((self.run / "organized_report.html").read_bytes(), (self.run / "index.html").read_bytes())
        self.assertIn("at 10 Hz", (self.run / "organized_report.md").read_text())
        audit = json.loads((self.run / "report_audit.json").read_text())
        self.assertEqual(audit["input_hashes"]["summary.json"], digest(self.run / "summary.json"))
        self.assertEqual(audit["accepted_state_ids"], ["highest"])
        self.assertEqual(audit["unresolved_state_ids"], [])

    def test_failed_pdf_regeneration_cannot_leave_stale_pass(self):
        self.write_run()
        self.assertEqual(main(["--out-dir", str(self.run)]), 0)
        with patch("frigidaire_organized_report.export_pdf", side_effect=RuntimeError("PDF browser failure")):
            self.assertEqual(main(["--out-dir", str(self.run), "--pdf"]), 1)
        audit = json.loads((self.run / "report_audit.json").read_text())
        self.assertEqual(audit["result"], "INCOMPLETE")
        self.assertIn("PDF browser failure", str(audit["problems"]))

    def test_broken_input_regeneration_cannot_leave_stale_pass(self):
        self.write_run()
        self.assertEqual(main(["--out-dir", str(self.run)]), 0)
        (self.run / "summary.json").write_text("invalid JSON")
        self.assertEqual(main(["--out-dir", str(self.run)]), 1)
        audit = json.loads((self.run / "report_audit.json").read_text())
        self.assertEqual(audit["result"], "INCOMPLETE")

    def test_provisional_primary_and_screening_do_not_claim_highest_accepted(self):
        source, state, summary = self.write_run(accepted=False)
        summary["screening"] = {"screening_passed": 35}
        row = summary["states"][0]
        row.update(provisional_state="provisional_states/highest_primary.json", reason="Independent reproduction failed")
        self.save(self.run / "summary.json", summary)
        result = load_experiment(self.run)
        text = markdown(result)
        self.assertIn("its organized counterpart is unresolved", text)
        self.assertIn("Independent reproduction failed", text)
        self.assertIn("not an accepted counterpart", text)
        self.assertEqual(main(["--out-dir", str(self.run)]), 0)
        audit = json.loads((self.run / "report_audit.json").read_text())
        self.assertEqual(audit["accepted_state_ids"], [])
        self.assertEqual(audit["unresolved_state_ids"], ["highest"])


if __name__ == "__main__":
    unittest.main()
