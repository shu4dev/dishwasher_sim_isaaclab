"""Saved initial-state evidence, bounded summaries, and render caption helpers."""
from copy import deepcopy
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[2]


def load_tool(name):
    path = ROOT / "frigidaire/scripts/evaluation" / f"frigidaire_initial_state_{name}.py"
    spec = importlib.util.spec_from_file_location(f"initial_state_{name}", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


report, render = load_tool("report"), load_tool("render")


def state_fixture():
    pose = {"position_m": [0., -.48, .3], "quaternion_xyzw": [0., 0., 0., 1.]}
    snapshot = {"joints": {"door_hinge": math.pi/2, "lower_slide": -.49, "upper_slide": -.44},
                "poses": {name: deepcopy(pose) for name in (*report.COMPONENTS, "mug_001")}}
    metric = {"root_position_span_m": .0001, "quaternion_span_deg": .01,
              "peak_mesh_point_speed_m_s": .001, "sample_count": 121, "sample_duration_s": 1.}
    hold = {key: True for key in ("passed", "settled", "contacts_ok", "basket_supported", "dish_supported", "endpoints_ok")}
    hold.update(observation_completed_s=5., rest_window_count=601, continuous_passing_window_count=601,
                peak_penetration_m=.0001, median_max_penetration_m=.00001,
                motion={name: deepcopy(metric) for name in ("mug_001", "SilverwareBasket")})
    validation = {"outcome": "accepted", "object_count": 1, "initial_snapshot": deepcopy(snapshot),
                  "initial_geometry": {"valid": True}, "settled": deepcopy(hold), "final_hold": deepcopy(hold),
                  "maximum_settle_penetration_m": .0001, "maximum_cycle_penetration_m": .0001,
                  "final_containment": {"mug_001": {"contained": True, "world_contained": True, "cabinet_frame_contained": True}},
                  "loaded_rack_motions": [{"rack": rack, "passed": True, "hold": deepcopy(hold),
                                           "peak_measured_rack_speed_m_s": .099} for rack in report.RACKS]}
    return {"schema_version": 1, "state_id": "highest", "purpose": "highest", "accepted": True,
            "objects": [{"object_id": "mug_001", "kind": "mug", "rack": "LowerRack",
                         "pose_world": deepcopy(pose), "rack_local_pose": deepcopy(pose)}],
            "initial_snapshot": snapshot, "validation": validation, "source_attempt": "attempt_0000",
            "counts": {"total": 1, "by_kind": {"mug": 1}, "by_rack": {"LowerRack": 1}}, "seed": 17}


class InitialStateReportingTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory(prefix="initial-state-report-", dir=os.environ.get("INITIAL_STATE_TEST_OUTPUT_DIR"))
        self.addCleanup(temporary.cleanup)
        self.directory = Path(temporary.name)

    def save(self, relative, value):
        path = self.directory / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(value, indent=2) + "\n")
        return path

    def write_fixture(self, state=None):
        state = state or state_fixture()
        self.save("states/highest.json", state)
        self.save("attempts/attempt_0000/result.json", state["validation"])
        self.save("attempts/attempt_0000/manifest.json", {"objects": state["objects"]})
        summary = {"status": "incomplete", "wall_seconds": 120., "budget_seconds": 7200, "seed": 17,
                   "highest_count": 1, "randomized_count": 0, "attempt_count": 1, "outcomes": {"accepted": 1},
                   "states": ["states/highest.json"], "catalog": {}, "search": {}, "limits": {},
                   "input_hashes": {}, "source_hashes": {}}
        self.save("summary.json", summary)
        return summary

    def add_catalog(self, summary):
        catalog = {"source_trial_count": 2, "variants_per_template": 1, "candidate_count": 4,
                   "candidates": [{"variant_index": variant} for variant in (0, 1, 0, 1)]}
        self.save("candidates.json", catalog)
        canonical = hashlib.sha256(json.dumps(catalog, sort_keys=True, separators=(",", ":")).encode()).hexdigest()
        summary["catalog"] = {"candidate_count": 4, "allowed_count": 3, "allowed_indices": [0, 1, 2],
                              "conflict_count": 1, "complete": True, "unresolved_pairs": 0,
                              "candidate_catalog_sha256": canonical}
        self.save("summary.json", summary)
        return catalog

    def test_valid_state(self):
        self.assertEqual(report.validate_state(state_fixture())["total"], 1)

    def test_missing_physics_rejected(self):
        for key in ("settled", "final_hold", "loaded_rack_motions", "final_containment", "initial_geometry"):
            with self.subTest(key=key):
                state = state_fixture()
                state["validation"].pop(key)
                with self.assertRaises(ValueError):
                    report.validate_state(state)

    def test_measured_not_proposed_pose(self):
        state = state_fixture()
        state["objects"][0]["pose_world"]["position_m"][0] += .01
        with self.assertRaisesRegex(ValueError, "measured initial snapshot"):
            report.validate_state(state)

    def test_joint_state_both_racks_extended(self):
        state = state_fixture()
        state["initial_snapshot"]["joints"]["upper_slide"] = 0.
        with self.assertRaisesRegex(ValueError, "both-racks-extended"):
            report.validate_state(state)

    def test_insufficient_observation_rejected(self):
        state = state_fixture()
        state["validation"]["settled"]["observation_completed_s"] = 4.9
        with self.assertRaisesRegex(ValueError, "continuous observation"):
            report.validate_state(state)

    def test_noncontinuous_observation_rejected(self):
        state = state_fixture()
        state["validation"]["settled"]["continuous_passing_window_count"] = 600
        with self.assertRaisesRegex(ValueError, "passing-window sequence"):
            report.validate_state(state)

    def test_both_containment_frames_required(self):
        state = state_fixture()
        state["validation"]["final_containment"]["mug_001"]["cabinet_frame_contained"] = False
        with self.assertRaisesRegex(ValueError, "containment"):
            report.validate_state(state)

    def test_source_attempt_equality(self):
        self.write_fixture()
        self.save("attempts/attempt_0000/result.json", {})
        self.assertEqual(report.load_experiment(self.directory)["audit_result"], "INCOMPLETE")

    def test_report_html_and_markdown(self):
        self.write_fixture()
        self.assertEqual(report.main(["--out-dir", str(self.directory)]), 0)
        markdown = (self.directory / "experiment_report.md").read_text()
        self.assertIn("Highest validated load found: 1 dishes", markdown)
        self.assertIn("not uniform samples", markdown)
        self.assertIn("Physics settling may move a dish farther", markdown)
        self.assertIn("<table>", (self.directory / "experiment_report.html").read_text())
        audit = report.read_json(self.directory / "report_audit.json")
        self.assertEqual(audit["input_hashes"]["summary.json"], report.digest(self.directory / "summary.json"))

    def test_complete_claim_requires_all_artifacts(self):
        summary = self.write_fixture()
        summary["status"] = "complete"
        self.save("summary.json", summary)
        self.assertEqual(report.load_experiment(self.directory)["audit_result"], "INCOMPLETE")

    def test_duplicate_published_state_rejected(self):
        summary = self.write_fixture()
        summary["states"] *= 2
        self.save("summary.json", summary)
        self.assertEqual(report.load_experiment(self.directory)["audit_result"], "INCOMPLETE")

    def test_catalog_uses_canonical_hash_and_retains_file_hash(self):
        summary = self.write_fixture()
        self.add_catalog(summary)
        result = report.load_experiment(self.directory)
        self.assertEqual(result["audit_result"], "PASS")
        metrics = result["catalog_metrics"]
        self.assertEqual(metrics["eligible_originals"], 2)
        self.assertNotEqual(metrics["canonical_catalog_sha256"], metrics["artifact_hashes"]["candidates.json"])

    def test_changed_catalog_rejected(self):
        summary = self.write_fixture()
        catalog = self.add_catalog(summary)
        catalog["candidates"][0]["variant_index"] = 1
        self.save("candidates.json", catalog)
        self.assertEqual(report.load_experiment(self.directory)["audit_result"], "INCOMPLETE")

    def test_fixed_target_bounds_excluded_and_output_stays_compact(self):
        summary = self.write_fixture()
        self.add_catalog(summary)
        full = {"status": 0, "count": 3, "elapsed_s": .5, "geometric_cardinality_upper_bound": 3,
                "target_count": None, "bound_scope": "full finite compatibility graph", "message": "RAW_SOLVER_SENTINEL"}
        fixed = {**full, "count": 1, "geometric_cardinality_upper_bound": 1,
                 "target_count": 1, "bound_scope": "fixed target-count MILP only"}
        summary["search"]["solver_records"] = [full]*100 + [fixed]
        self.save("summary.json", summary)
        result = report.load_experiment(self.directory)
        self.assertEqual(result["solver_metrics"]["minimum_full_graph_bound"], 3)
        self.assertEqual(result["solver_metrics"]["elapsed_seconds"], 50.5)
        self.assertNotIn("RAW_SOLVER_SENTINEL", report.markdown(result))
        self.assertLess(len(report.html_report(result)), 15000)

    def test_incomplete_graph_has_no_bound(self):
        summary = self.write_fixture()
        self.add_catalog(summary)
        summary["catalog"]["complete"] = False
        summary["search"]["solver_records"] = [{"status": 0, "count": 3, "geometric_cardinality_upper_bound": 3,
                                                 "target_count": None, "bound_scope": "full finite compatibility graph"}]
        self.save("summary.json", summary)
        self.assertIsNone(report.load_experiment(self.directory)["solver_metrics"]["minimum_full_graph_bound"])

    def test_runtime_profiles_use_each_attempts_actual_settings(self):
        self.write_fixture()
        result = report.load_experiment(self.directory)
        result["attempts"] = [{"attempt_id": "attempt_0000", "result": {"runtime": {"commanded_rack_speed_cap_m_s": .095}, "settled": {"passed": False}}},
                              {"attempt_id": "attempt_0001", "result": {"runtime": {"commanded_rack_speed_cap_m_s": .08}, "settled": {"observation_restarts": []}}}]
        profiles = report.runtime_profiles(result)
        self.assertEqual([p["commanded_rack_speed_cap_m_s"] for p in profiles], [.095, .08])
        self.assertIn("requalification", profiles[1]["observation_policy"])

    def test_render_audit_recomputes_all_saved_transform_errors(self):
        state = state_fixture()
        self.write_fixture(state)
        image = self.directory / "renders/highest_initial.png"
        image.parent.mkdir()
        image.write_bytes(b"fixture image bytes")
        evidence = {"result": "PASS", "state_id": "highest", "image_file": image.name,
                    "state_sha256": report.digest(self.directory / "states/highest.json"),
                    "image_sha256": report.digest(image), "state_unchanged": True, "resolution": [1920, 1440],
                    "counts": state["counts"], "rendered_poses": deepcopy(state["initial_snapshot"]["poses"]),
                    "label": "Highest validated load found", "max_position_error_m": 0.}
        self.save("renders/highest_render_evidence.json", evidence)
        result = report.load_experiment(self.directory)
        self.assertEqual(len(report.image_records(result)), 1)
        evidence["rendered_poses"]["mug_001"]["position_m"][0] += .001
        self.save("renders/highest_render_evidence.json", evidence)
        self.assertEqual(report.image_records(result), [])

    def test_changed_render_pixels_rejected(self):
        self.write_fixture()
        result = report.load_experiment(self.directory)
        self.save("renders/highest_render_evidence.json", {"result": "PASS", "state_id": "highest",
                  "image_file": "highest_initial.png", "state_sha256": result["states"][0]["sha256"],
                  "image_sha256": "incorrect"})
        (self.directory / "renders/highest_initial.png").write_bytes(b"changed")
        self.assertEqual(report.image_records(result), [])

    def test_per_item_tints_and_exact_highest_label(self):
        state = state_fixture()
        state["objects"] = [{"object_id": f"dish_{i:03d}"} for i in range(50)]
        colors = render.item_tints(state["objects"])
        self.assertEqual(len(set(tuple(value) for value in colors.values())), 50)
        self.assertEqual(render.label_for(state), "Highest validated load found")

    def test_caption_fonts_are_large_and_hash_recorded(self):
        fonts, evidence = render.caption_fonts()
        self.assertEqual(fonts["title"].size, 42)
        self.assertEqual(fonts["detail"].size, 27)
        self.assertEqual(evidence["title"]["sha256"], render.digest(evidence["title"]["path"]))

    def test_quaternion_sign_pose_audit(self):
        first = state_fixture()["objects"][0]["pose_world"]
        second = deepcopy(first)
        second["quaternion_xyzw"] = [-x for x in second["quaternion_xyzw"]]
        self.assertEqual(report.pose_error(first, second)["orientation_error_deg"], 0)


if __name__ == "__main__":
    unittest.main()
