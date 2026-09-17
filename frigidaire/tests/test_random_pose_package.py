"""A small finished-run fixture exercises evidence binding and portable ZIPs."""
from copy import deepcopy
import hashlib
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest
import zipfile

from dishsim_frigidaire.random_pose_reports import build_summary
from dishsim_frigidaire.random_poses import KINDS, LIMITS, RACKS

SCRIPT = Path(__file__).resolve().parents[1]/"scripts/evaluation/frigidaire_random_pose_package.py"
SPEC = importlib.util.spec_from_file_location("random_pose_package_under_test", SCRIPT)
package = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(package)


def write_json(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, allow_nan=False)+"\n")


class PackageTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix="random_pose_package_test_")
        self.addCleanup(self.temporary.cleanup)
        self.repo = Path(self.temporary.name)/"repo"
        self.run = self.repo/"results/finished_fixture"
        self.run.mkdir(parents=True)
        self.source_hashes = {}
        for name in ("tableware", "geometry"):
            relative = "frigidaire/src/dishsim_frigidaire/"+name+".py"
            path = self.repo/relative
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text("# "+name+" source fixture\n")
            self.source_hashes[relative] = package.digest(path)
        pose = {"position_m": [0., 0., .2], "quaternion_xyzw": [0., 0., 0., 1.]}
        rows = []
        for index in range(2):
            for kind in KINDS:
                for rack in RACKS:
                    trial_id = "{}_{}_{:05d}".format(kind, rack, index)
                    rows.append({"trial_id": trial_id, "kind": kind, "rack": rack, "sample_index": index,
                                 "outcome": "accepted" if index == 0 else "initial_collision",
                                 "sampled_pose": dict(deepcopy(pose), bbox_center_m=[0., 0., .2]),
                                 "settled_pose": deepcopy(pose) if index == 0 else None,
                                 "final_pose": deepcopy(pose) if index == 0 else None,
                                 "final_hold": {"poses": {rack: deepcopy(pose)}},
                                 "trace_file": "traces/"+trial_id+".jsonl"})
                    path = self.run/rows[-1]["trace_file"]
                    path.parent.mkdir(exist_ok=True)
                    path.write_text(json.dumps({"phase": "final_hold", "dish": pose})+"\n")
        self.rows = rows
        self.accepted = rows[:6]
        self.summary = build_summary(rows, kinds=KINDS, racks=RACKS, samples_per_cell=2,
                                     baselines={rack: {"result": "PASS"} for rack in RACKS}, status="complete")
        self.meta = {
            "run_status": "complete", "finished_utc": "2026-09-10T22:00:00+00:00",
            "kinds": list(KINDS), "racks": list(RACKS), "samples_per_cell": 2, "seed": 0,
            "source_hashes": self.source_hashes, "wall_seconds": 10., "cumulative_wall_seconds": 10.,
            "new_evaluation_attempts": 12, "cumulative_evaluation_attempts": 12,
            "inherited_classified_trials": 0, "thresholds": deepcopy(LIMITS),
            "runtime": {"device": "cpu", "isaac_sim": "test-runtime", "physics_hz": 120},
            "domains": {"sampling_bounds_by_rack": {
                rack: {"lower_m": [-.2, -.2, 0.], "upper_m": [.2, .2, .3]} for rack in RACKS}},
            "inputs": {"asset_hashes": {"tableware_fixture.usdc": "assets-are-not-copied"},
                       "catalog": {kind: {"size_m": [.1, .1, .1], "actual_size_m": [.1, .1, .1]} for kind in KINDS}},
        }
        self.replay = {"schema_version": 1, "metadata_file": "metadata.json",
                       "pose_frames": {"final_pose": "world"}, "trials": deepcopy(self.accepted)}
        write_json(self.run/"metadata.json", self.meta)
        write_json(self.run/"summary.json", self.summary)
        write_json(self.run/"accepted_poses.json", self.replay)
        (self.run/"trials.jsonl").write_text("".join(json.dumps(row)+"\n" for row in rows))
        self.analysis = {
            "status": "PASS", "accepted_count": 6,
            "input_hashes": {name: package.digest(self.run/name) for name in package.RAW_FILES},
            "trace_hashes": {row["trace_file"]: package.digest(self.run/row["trace_file"]) for row in rows},
            "audits": [{"trial_id": row["trial_id"], "result": "PASS", "failed": [], "missing": [],
                        "final_saved_endpoint_error_m": .0001, "trace_window_endpoint_error_m": .0002,
                        "cycle_peak_penetration_m": .0003, "wall_clearance_m": .0001} for row in self.accepted],
            "resume_audit": {"status": "NOT_APPLICABLE"},
            "accounting": {"session_wall_seconds": 10., "cumulative_wall_seconds": 10.,
                           "inherited_classified_trials": 0, "new_evaluation_attempts": 12,
                           "cumulative_evaluation_attempts": 12, "logical_attempted": 12,
                           "logical_planned": 12, "archived_unresolved_attempts": 0},
        }
        self.audit = {
            "status": "PASS", "passed": True, "errors": [], "accepted_count": 6, "checked_count": 6,
            "input_hashes": {name: package.digest(self.run/name) for name in ("metadata.json", "accepted_poses.json")},
            "containment_tolerance_m": .001, "bounds_discrepancy_limit_m": 1e-7,
            "world_contained_count": 6, "cabinet_contained_count": 6, "max_bounds_discrepancy_m": 1e-9,
            "source_hashes": {name: {"matches": True, "actual_sha256": value, "recorded_sha256": value}
                              for name, value in self.source_hashes.items()},
            "trials": [{"trial_id": row["trial_id"], "kind": row["kind"], "rack": row["rack"],
                        "passed": True, "world_contained": True, "cabinet_contained": True,
                        "cabinet_world_identity": False, "bounds_discrepancy_m": 1e-9,
                        "world_min_signed_interior_clearance_m": -.0004 if index == 0 else .0002,
                        "cabinet_min_signed_interior_clearance_m": -.0003 if index == 1 else .0001}
                       for index, row in enumerate(self.accepted)],
        }
        (self.run/"accepted_pose_examples.png").write_bytes(b"fixture image; no renderer needed for evidence binding\n")
        self.views = {
            "kind": "saved_pose_reconstruction", "new_simulation": False, "isaac_render": False,
            "input_sha256": {name: package.digest(self.run/name) for name in ("metadata.json", "summary.json", "accepted_poses.json")},
            "figure_sha256": package.digest(self.run/"accepted_pose_examples.png"),
            "asset_hashes": self.meta["inputs"]["asset_hashes"],
            "geometry_source_sha256": self.source_hashes["frigidaire/src/dishsim_frigidaire/geometry.py"],
            "tableware_source_sha256": self.source_hashes["frigidaire/src/dishsim_frigidaire/tableware.py"],
            "panels": [{"kind": row["kind"], "rack": row["rack"], "trial_id": row["trial_id"],
                        "dish_pose": row["final_pose"], "rack_pose": row["final_hold"]["poses"][row["rack"]]}
                       for row in self.accepted],
        }
        self.write_derived_inputs()

    def write_derived_inputs(self):
        write_json(self.run/"analysis.json", self.analysis)
        write_json(self.run/"independent_geometry_audit.json", self.audit)
        write_json(self.run/"accepted_pose_examples.json", self.views)

    def refresh_metadata_binding(self):
        write_json(self.run/"metadata.json", self.meta)
        value = package.digest(self.run/"metadata.json")
        self.analysis["input_hashes"]["metadata.json"] = value
        self.audit["input_hashes"]["metadata.json"] = value
        self.views["input_sha256"]["metadata.json"] = value
        self.write_derived_inputs()

    def assert_no_package_outputs(self):
        self.assertFalse((self.run/"experiment_report.md").exists())
        self.assertFalse((self.run/"strictly_contained_poses.json").exists())
        self.assertFalse((self.run.parent/(self.run.name+".zip")).exists())

    def test_package_preserves_raw_evidence_and_verifies_small_archive(self):
        before = {name: (self.run/name).read_bytes() for name in package.RAW_FILES}
        result = package.package_run(self.run, self.repo)
        self.assertEqual(result["accepted"], 6)
        self.assertEqual(result["strictly_contained"], 4)
        selected = package.read_json(self.run/"strictly_contained_poses.json")["trials"]
        self.assertEqual(selected, self.accepted[2:])
        self.assertEqual({name: (self.run/name).read_bytes() for name in package.RAW_FILES}, before)
        report = (self.run/"experiment_report.md").read_text()
        self.assertIn("6 accepted placements from 12 distinct random proposals (50%)", report)
        self.assertIn("greatest nominal boundary overrun among accepted poses is 0.4 mm", report)
        self.assertIn("Maximum accepted retraction/hold penetration was 0.3 mm", report)
        self.assertIn("cabinet", report.lower())
        with zipfile.ZipFile(result["archive"]) as archive:
            self.assertIsNone(archive.testzip())
            for line in archive.read(self.run.name+"/checksums.sha256").decode().splitlines():
                checksum, relative = line.split("  ", 1)
                self.assertEqual(hashlib.sha256(archive.read(self.run.name+"/"+relative)).hexdigest(), checksum)
            self.assertFalse(any(name.endswith(".usdc") for name in archive.namelist()))
            for relative, checksum in self.source_hashes.items():
                data = archive.read(self.run.name+"/source_snapshot/"+relative)
                self.assertEqual(hashlib.sha256(data).hexdigest(), checksum)

    def test_resumed_counts_distinguish_logical_ids_and_evaluation_attempts(self):
        self.meta.update(resume={"applied": True}, cumulative_wall_seconds=30., inherited_classified_trials=6,
                         new_evaluation_attempts=6, cumulative_evaluation_attempts=13)
        self.analysis["resume_audit"] = {"status": "PASS"}
        self.analysis["accounting"].update(cumulative_wall_seconds=30., inherited_classified_trials=6,
                                             new_evaluation_attempts=6, cumulative_evaluation_attempts=13,
                                             archived_unresolved_attempts=1)
        self.refresh_metadata_binding()
        result = package.package_run(self.run, self.repo)
        self.assertEqual(result["logical_attempted"], 12)
        self.assertEqual(result["cumulative_evaluation_attempts"], 13)
        report = (self.run/"experiment_report.md").read_text()
        self.assertIn("This process evaluated 6 proposals and carried 6 prior classified rows", report)
        self.assertIn("13 evaluation attempts, including 1 archived unresolved attempts", report)
        self.assertIn("Cumulative session wall time was 30 s", report)

    def test_complete_sampling_with_numerical_unresolved_preserves_honest_counts(self):
        unresolved = self.rows[-1]
        unresolved["outcome"] = "simulation_error"
        unresolved["reason"] = "Invalid settled contacts"
        self.summary = build_summary(self.rows, kinds=KINDS, racks=RACKS, samples_per_cell=2,
                                     baselines={rack: {"result": "PASS"} for rack in RACKS}, status="complete")
        write_json(self.run/"summary.json", self.summary)
        (self.run/"trials.jsonl").write_text("".join(json.dumps(row)+"\n" for row in self.rows))
        for name in ("summary.json", "trials.jsonl"):
            self.analysis["input_hashes"][name] = package.digest(self.run/name)
        self.views["input_sha256"]["summary.json"] = package.digest(self.run/"summary.json")
        self.write_derived_inputs()
        raw_before = (self.run/"trials.jsonl").read_bytes()
        result = package.package_run(self.run, self.repo)
        self.assertEqual(result["logical_attempted"], 12)
        self.assertEqual(result["accepted"], 6)
        self.assertEqual(result["strictly_contained"], 4)
        self.assertEqual((self.run/"trials.jsonl").read_bytes(), raw_before)
        selected = package.read_json(self.run/"strictly_contained_poses.json")["trials"]
        self.assertNotIn(unresolved["trial_id"], {row["trial_id"] for row in selected})
        report = (self.run/"experiment_report.md").read_text()
        self.assertIn("sampling complete; 1 unresolved", report)
        self.assertIn("11 proposals were classified, 1 remain unresolved", report)
        self.assertIn("| simulation error | 1 |", report)
        self.assertIn("audit applies only to accepted placements", report)
        self.assertIn("does not certify numerical validity of every attempted trial", report)

    def test_complete_sampling_with_untested_proposals_is_rejected(self):
        remaining = self.rows[:-1]
        summary = build_summary(remaining, kinds=KINDS, racks=RACKS, samples_per_cell=2,
                                baselines={rack: {"result": "PASS"} for rack in RACKS}, status="complete")
        write_json(self.run/"summary.json", summary)
        (self.run/"trials.jsonl").write_text("".join(json.dumps(row)+"\n" for row in remaining))
        with self.assertRaisesRegex(ValueError, "attempt every planned proposal"):
            package.package_run(self.run, self.repo)
        self.assert_no_package_outputs()

    def test_live_run_is_rejected_before_writing(self):
        self.meta["run_status"] = "running"
        self.meta.pop("finished_utc")
        self.refresh_metadata_binding()
        with self.assertRaisesRegex(ValueError, "finalized"):
            package.package_run(self.run, self.repo)
        self.assert_no_package_outputs()

    def test_metadata_change_invalidates_existing_audits(self):
        self.meta["seed"] = 99
        write_json(self.run/"metadata.json", self.meta)
        with self.assertRaisesRegex(ValueError, "hash mismatch"):
            package.package_run(self.run, self.repo)
        self.assert_no_package_outputs()

    def test_changed_source_cannot_be_packaged_as_recorded_source(self):
        (self.repo/next(iter(self.source_hashes))).write_text("# newer source\n")
        with self.assertRaisesRegex(ValueError, "Source snapshot hash mismatch"):
            package.package_run(self.run, self.repo)
        self.assert_no_package_outputs()

    def test_geometry_audit_rejects_wrong_ids_or_impossible_claims(self):
        original = deepcopy(self.audit)
        for change in ("duplicate_id", "wrong_cell", "failed", "outside", "discrepancy"):
            with self.subTest(change=change):
                self.audit = deepcopy(original)
                row = self.audit["trials"][0]
                if change == "duplicate_id":
                    row["trial_id"] = self.audit["trials"][1]["trial_id"]
                elif change == "wrong_cell":
                    row["kind"] = "mug"
                elif change == "failed":
                    row["passed"] = False
                elif change == "outside":
                    row["world_min_signed_interior_clearance_m"] = -.0011
                else:
                    row["bounds_discrepancy_m"] = 2e-7
                self.write_derived_inputs()
                with self.assertRaises(ValueError):
                    package.package_run(self.run, self.repo)
                self.assert_no_package_outputs()

    def test_saved_evidence_trace_tamper_or_missing_binding_blocks_package(self):
        relative = self.accepted[0]["trace_file"]
        original = (self.run/relative).read_bytes()
        (self.run/relative).write_bytes(original+b"{}\n")
        with self.assertRaisesRegex(ValueError, "trace hash mismatch"):
            package.package_run(self.run, self.repo)
        (self.run/relative).write_bytes(original)
        self.analysis["trace_hashes"].pop(relative)
        self.write_derived_inputs()
        with self.assertRaisesRegex(ValueError, "not hashed"):
            package.package_run(self.run, self.repo)
        self.assert_no_package_outputs()

    def test_reconstruction_cannot_silently_show_different_poses(self):
        self.views["panels"][0]["dish_pose"] = {"position_m": [10., 0., 0.], "quaternion_xyzw": [0., 0., 0., 1.]}
        self.write_derived_inputs()
        with self.assertRaisesRegex(ValueError, "transforms disagree"):
            package.package_run(self.run, self.repo)
        self.assert_no_package_outputs()

    def test_evaluation_attempt_counts_must_include_archived_retries(self):
        self.analysis["accounting"]["archived_unresolved_attempts"] = 1
        self.write_derived_inputs()
        with self.assertRaisesRegex(ValueError, "accounting disagree"):
            package.package_run(self.run, self.repo)
        self.assert_no_package_outputs()

    def test_failed_or_incomplete_saved_evidence_cannot_be_packaged(self):
        self.analysis["audits"][0]["result"] = "INCOMPLETE"
        self.write_derived_inputs()
        with self.assertRaisesRegex(ValueError, "incomplete or failed"):
            package.package_run(self.run, self.repo)
        self.assert_no_package_outputs()


if __name__ == "__main__":
    unittest.main()
