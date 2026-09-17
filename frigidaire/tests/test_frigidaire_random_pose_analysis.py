"""Saved-evidence audits and analysis plots, independent of Isaac/Kit."""
from copy import deepcopy
import importlib.util
import hashlib
import json
from pathlib import Path
import tempfile
import unittest
import shutil


SCRIPT = Path(__file__).resolve().parents[1] / "scripts/evaluation/frigidaire_random_pose_report.py"
SPEC = importlib.util.spec_from_file_location("frigidaire_random_pose_analysis", SCRIPT)
analysis = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(analysis)


def evidence():
    pose = {"position_m": [0., 0., .2], "quaternion_xyzw": [0., 0., 0., 1.], "bbox_center_m": [0., 0., .2]}
    metrics = {"root_position_span_m": .0001, "quaternion_span_deg": .01,
               "peak_mesh_point_speed_m_s": .002, "sample_count": 121, "sample_duration_s": 1.}
    hold = {"settled": True, "endpoints_ok": True, "contacts_ok": True,
            "basket_supported": True, "dish_supported": True,
            "peak_penetration_m": .00001, "median_max_penetration_m": .000001,
            "motion": {"mug": deepcopy(metrics), "SilverwareBasket": deepcopy(metrics)},
            "joints": {"lower_slide": -.000001, "upper_slide": -.000002}}
    trial = {"trial_id": "mug_LowerRack_00000", "sample_index": 0, "kind": "mug", "rack": "LowerRack",
             "outcome": "accepted", "sampled_pose": deepcopy(pose), "settled_pose": deepcopy(pose),
             "final_pose": deepcopy(pose), "settle": deepcopy(hold), "final_hold": deepcopy(hold),
             "final_mesh_bounds_m": [[-.1, -.1, .1], [.1, .1, .3]], "maximum_cycle_penetration_m": .00002,
             "trace_file": "traces/mug.jsonl"}
    metadata = {"thresholds": {"root_position_span_m": .005, "quaternion_span_deg": 3.,
                               "peak_mesh_point_speed_m_s": .03, "peak_penetration_m": .002,
                               "median_max_penetration_m": .001, "rack_endpoint_m": .005,
                               "containment_tolerance_m": .001, "rest_window_s": 1., "physics_dt_s": 1/120},
                "domains": {"interior_bounds": {"lower_m": [-.3, -.3, 0.], "upper_m": [.3, .3, 1.]}},
                "runtime": {"physics_hz": 120, "trace_hz": 10}, "seed": 0, "samples_per_cell": 3,
                "kinds": ["mug"], "racks": ["LowerRack"], "wall_seconds": 10.,
                "finished_utc": "2026-09-10T20:00:00+00:00", "run_status": "budget_exhausted"}
    trace = [{"step": 12*i, "phase": "final_hold", "dish": deepcopy(pose),
              "joints": deepcopy(hold["joints"]), "contact_pairs": [["LowerRack", "mug"], ["LowerRack", "SilverwareBasket"]]}
             for i in range(1, 11)]
    return trial, metadata, trace


class RandomPoseAnalysisTests(unittest.TestCase):
    def setUp(self):
        directory = tempfile.TemporaryDirectory(prefix="frigidaire-random-pose-analysis-")
        self.addCleanup(directory.cleanup)
        self.directory = Path(directory.name)

    def write_run(self):
        trial, metadata, trace = evidence()
        collision = {**deepcopy(trial), "trial_id": "mug_LowerRack_00001", "sample_index": 1,
                     "outcome": "initial_collision", "settled_pose": None, "final_pose": None}
        for key in ("settle", "final_hold", "final_mesh_bounds_m", "maximum_cycle_penetration_m", "trace_file"):
            collision.pop(key, None)
        trials = [trial, collision]
        summary = analysis.build_summary(trials, kinds=metadata["kinds"], racks=metadata["racks"],
                                         samples_per_cell=3, baselines={"LowerRack": {"result": "PASS"}},
                                         status=metadata["run_status"])
        summary["extra"] = {"wall_seconds": metadata["wall_seconds"]}
        for filename, value in (("summary.json", summary), ("metadata.json", metadata),
                                ("accepted_poses.json", {"trials": [trial]})):
            (self.directory / filename).write_text(json.dumps(value))
        (self.directory / "trials.jsonl").write_text("".join(json.dumps(t)+"\n" for t in trials))
        (self.directory / "traces").mkdir()
        (self.directory / trial["trace_file"]).write_text("".join(json.dumps(t)+"\n" for t in trace))
        return trial, metadata, trace

    def write_resumed_run(self):
        trial, metadata, _ = self.write_run()
        metadata["inputs"] = {"asset_hashes": {"fixture.usdc": "same-asset"}}
        metadata["source_hashes"] = {path: "same-source" for path in analysis.PHYSICS_SOURCE_PATHS}
        old_rows = [json.loads(line) for line in (self.directory / "trials.jsonl").read_text().splitlines()]
        interrupted = {**deepcopy(old_rows[-1]), "trial_id": "mug_LowerRack_00002", "sample_index": 2,
                       "outcome": "interrupted", "trace_file": "traces/interrupted.jsonl"}
        old_rows.append(interrupted)
        old_summary = analysis.build_summary(old_rows, kinds=metadata["kinds"], racks=metadata["racks"], samples_per_cell=3,
                                             baselines={"LowerRack": {"result": "PASS"}}, status="budget_exhausted")
        old_summary["extra"] = {"wall_seconds": metadata["wall_seconds"]}
        history = self.directory / "resume_history/prior"
        (history / "traces").mkdir(parents=True)
        for name, value in (("metadata.json", metadata), ("summary.json", old_summary),
                            ("accepted_poses.json", {"trials": [trial]})):
            (history / name).write_text(json.dumps(value))
        (history / "trials.jsonl").write_text("".join(json.dumps(t)+"\n" for t in old_rows))
        (history / "unresolved_trials.jsonl").write_text(json.dumps(interrupted)+"\n")
        shutil.copyfile(self.directory / trial["trace_file"], history / trial["trace_file"])
        (history / interrupted["trace_file"]).write_text(json.dumps({"phase": "loaded_retraction", "dish": trial["sampled_pose"]})+"\n")
        hashes = {str(p.relative_to(history)): hashlib.sha256(p.read_bytes()).hexdigest() for p in history.rglob("*")
                  if p.is_file() and p.name != "unresolved_trials.jsonl"}
        metadata["resume"] = {"applied": True, "history_directory": "resume_history/prior", "prior_file_hashes": hashes,
                              "prior_status": "budget_exhausted", "prior_logical_attempts": 3,
                              "prior_evaluation_attempts": 3, "prior_cumulative_wall_seconds": 10.,
                              "carried_classified_count": 2, "archived_unresolved_count": 1,
                              "prior_outcome_counts": old_summary["totals"], "allowed_source_differences": {},
                              "physics_sources_validated": list(analysis.PHYSICS_SOURCE_PATHS)}
        metadata.update(run_status="complete", wall_seconds=14., cumulative_wall_seconds=24.,
                        inherited_classified_trials=2, new_evaluation_attempts=1, cumulative_evaluation_attempts=4)
        retry = {**deepcopy(interrupted), "outcome": "initial_collision"}
        retry.pop("trace_file")
        rows = old_rows[:2] + [retry]
        summary = analysis.build_summary(rows, kinds=metadata["kinds"], racks=metadata["racks"], samples_per_cell=3,
                                         baselines={"LowerRack": {"result": "PASS"}}, status="complete")
        summary["extra"] = {key: metadata[key] for key in ("wall_seconds", "cumulative_wall_seconds", "inherited_classified_trials",
                                                           "new_evaluation_attempts", "cumulative_evaluation_attempts")}
        (self.directory / "metadata.json").write_text(json.dumps(metadata))
        (self.directory / "summary.json").write_text(json.dumps(summary))
        (self.directory / "trials.jsonl").write_text("".join(json.dumps(t)+"\n" for t in rows))
        return history

    def test_valid_saved_gates_and_conditional_denominator(self):
        self.write_run()
        result = analysis.analyse(self.directory)
        self.assertEqual(result["audit_result"], "PASS")
        cell = result["summary"]["cells"][0]
        self.assertEqual((cell["attempted"], cell["physical_entered"], cell["unattempted"]), (2, 1, 1))
        report = analysis.markdown(result)
        self.assertIn("1/2 (50.0%)", report)
        self.assertIn("1/1 (100.0%)", report)
        self.assertIn("10.0 s", report)
        self.assertIn("1 of 3 planned proposals were untested", report)

    def test_missing_evidence_never_passes(self):
        for missing in ("pose", "support", "motion", "bounds", "threshold", "trace"):
            with self.subTest(missing=missing):
                trial, metadata, trace = evidence()
                if missing == "pose":
                    trial.pop("final_pose")
                elif missing == "support":
                    trial["final_hold"].pop("dish_supported")
                elif missing == "motion":
                    trial["final_hold"]["motion"].pop("mug")
                elif missing == "bounds":
                    trial.pop("final_mesh_bounds_m")
                elif missing == "threshold":
                    metadata["thresholds"].pop("peak_penetration_m")
                else:
                    trace.clear()
                self.assertEqual(analysis.audit_accepted(trial, metadata, trace)["result"], "INCOMPLETE")

    def test_gate_failures_are_reported(self):
        for failure in ("endpoint", "support", "motion", "bounds", "penetration", "trace_support"):
            with self.subTest(failure=failure):
                trial, metadata, trace = evidence()
                if failure == "endpoint":
                    trial["final_hold"]["joints"]["lower_slide"] = .006
                elif failure == "support":
                    trial["final_hold"]["dish_supported"] = False
                elif failure == "motion":
                    trial["final_hold"]["motion"]["mug"]["root_position_span_m"] = .006
                elif failure == "bounds":
                    trial["final_mesh_bounds_m"][0][0] = -.302
                elif failure == "penetration":
                    trial["maximum_cycle_penetration_m"] = .002
                else:
                    trace[-1]["contact_pairs"] = []
                self.assertEqual(analysis.audit_accepted(trial, metadata, trace)["result"], "FAIL")

    def test_interruption_entry_uses_stage_evidence(self):
        trial, _, trace = evidence()
        partial = {"outcome": "interrupted"}
        self.assertIsNone(analysis.physical_entry(partial, [{"phase": "trial_reset", "dish": None}])["entered"])
        self.assertIs(analysis.physical_entry(partial, trace)["entered"], True)
        self.assertIs(analysis.physical_entry({"outcome": "initial_collision"}, trace)["entered"], False)
        self.assertIs(analysis.physical_entry({"outcome": "interrupted", "settle": trial["settle"]}, [])["entered"], True)

    def test_mismatched_counts_rejected_without_artifacts(self):
        self.write_run()
        path = self.directory / "summary.json"
        summary = json.loads(path.read_text())
        summary["totals"]["accepted"] += 1
        path.write_text(json.dumps(summary))
        with self.assertRaisesRegex(ValueError, "counts disagree"):
            analysis.analyse(self.directory)
        self.assertFalse((self.directory / "analysis.md").exists())

    def test_live_run_is_not_analysed(self):
        self.write_run()
        path = self.directory / "metadata.json"
        metadata = json.loads(path.read_text())
        metadata.pop("finished_utc")
        path.write_text(json.dumps(metadata))
        with self.assertRaisesRegex(ValueError, "not finalized"):
            analysis.analyse(self.directory)

    def test_trace_cannot_escape_run(self):
        self.write_run()
        with self.assertRaisesRegex(ValueError, "escapes run"):
            analysis._trace(self.directory, {"trace_file": "../unrelated.jsonl"})

    def test_command_writes_readable_plot_and_report(self):
        try:
            import matplotlib  # noqa: F401
        except ImportError:
            self.skipTest("matplotlib unavailable")
        self.write_run()
        self.assertEqual(analysis.main(["--run-dir", str(self.directory)]), 0)
        self.assertIn("Accepted-evidence audit: **PASS**", (self.directory / "analysis.md").read_text())
        self.assertTrue((self.directory / "outcome_breakdown.png").read_bytes().startswith(b"\x89PNG"))
        audit = json.loads((self.directory / "analysis.json").read_text())
        self.assertEqual(audit["status"], "PASS")
        self.assertEqual(audit["accepted_count"], 1)
        self.assertEqual(audit["input_hashes"]["trials.jsonl"], hashlib.sha256((self.directory / "trials.jsonl").read_bytes()).hexdigest())
        self.assertIn("analysis.md", audit["output_hashes"])

    def test_resume_rates_use_logical_samples_and_cumulative_attempts_are_separate(self):
        self.write_resumed_run()
        result = analysis.analyse(self.directory)
        self.assertEqual(result["resume_audit"]["status"], "PASS")
        self.assertEqual(result["accounting"], {"session_wall_seconds": 14., "cumulative_wall_seconds": 24.,
                                              "inherited_classified_trials": 2, "new_evaluation_attempts": 1,
                                              "cumulative_evaluation_attempts": 4, "logical_attempted": 3,
                                              "logical_planned": 3, "archived_unresolved_attempts": 1})
        text = analysis.markdown(result)
        self.assertIn("3 logical raw proposals", text)
        self.assertIn("4 cumulative evaluation attempts", text)
        self.assertIn("not a count of dishes released into physics", text)
        self.assertEqual(result["summary"]["cells"][0]["physical_entered"], 1)

    def test_resume_archive_hash_tamper_is_rejected(self):
        history = self.write_resumed_run()
        (history / "summary.json").write_text("{}")
        with self.assertRaisesRegex(ValueError, "history hash mismatch"):
            analysis.analyse(self.directory)

    def test_resume_inherited_trace_must_be_byte_identical(self):
        self.write_resumed_run()
        path = self.directory / "traces/mug.jsonl"
        path.write_text(path.read_text()+"\n")
        with self.assertRaisesRegex(ValueError, "Inherited trace bytes changed"):
            analysis.analyse(self.directory)

    def test_resume_unresolved_history_cannot_disappear(self):
        history = self.write_resumed_run()
        (history / "unresolved_trials.jsonl").write_text("")
        with self.assertRaisesRegex(ValueError, "Archived unresolved attempts differ"):
            analysis.analyse(self.directory)

    def test_resume_inherited_rows_cannot_change(self):
        self.write_resumed_run()
        path = self.directory / "trials.jsonl"
        rows = [json.loads(line) for line in path.read_text().splitlines()]
        rows[1]["new_detail"] = "not preserved"
        path.write_text("".join(json.dumps(t)+"\n" for t in rows))
        with self.assertRaisesRegex(ValueError, "Inherited classified row changed"):
            analysis.analyse(self.directory)

    def test_resume_accounting_cannot_double_count_copied_rows(self):
        self.write_resumed_run()
        path = self.directory / "metadata.json"
        metadata = json.loads(path.read_text())
        metadata["new_evaluation_attempts"] = 3
        metadata["cumulative_evaluation_attempts"] = 6
        path.write_text(json.dumps(metadata))
        with self.assertRaisesRegex(ValueError, "Resume accounting disagrees"):
            analysis.analyse(self.directory)

    def test_resume_retry_sample_cannot_change(self):
        self.write_resumed_run()
        path = self.directory / "trials.jsonl"
        rows = [json.loads(line) for line in path.read_text().splitlines()]
        rows[-1]["sampled_pose"]["bbox_center_m"][0] += .01
        path.write_text("".join(json.dumps(t)+"\n" for t in rows))
        with self.assertRaisesRegex(ValueError, "Retried proposal changed"):
            analysis.analyse(self.directory)


if __name__ == "__main__":
    unittest.main()
