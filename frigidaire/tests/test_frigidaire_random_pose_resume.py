"""Resume provenance, deterministic retry, and immutable evidence carry-over."""
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import tempfile
import unittest

import numpy as np

from dishsim_frigidaire.random_pose_reports import ExperimentReport, build_summary
from dishsim_frigidaire.random_pose_resume import (
    PHYSICS_SOURCE_PATHS, load_resume, seed_report, validate_runtime, validate_sample_stream,
)
from dishsim_frigidaire.random_poses import KINDS, RACKS, cell_generators, sample_pose


class RandomPoseResumeTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory(prefix="frigidaire-resume-")
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)
        self.prior = self.root / "prior"
        self.prior.mkdir()
        self.points = {kind: np.array([[-.01, -.02, -.03], [.01, .02, .03]]) for kind in KINDS}
        self.domains = {"sampling_bounds_by_rack": {rack: {"lower_m": [-.2, -.2, 0.], "upper_m": [.2, .2, .3]}
                                                   for rack in RACKS}}
        self.metadata = {"seed": 0, "kinds": list(KINDS), "racks": list(RACKS), "samples_per_cell": 2,
                         "domains": self.domains, "thresholds": {"rack_endpoint_m": .005},
                         "inputs": {"asset_hashes": {"fixture.usdc": "same-asset"}},
                         "source_hashes": {path: "same-source" for path in PHYSICS_SOURCE_PATHS},
                         "runtime": {"device": "cpu", "physics_hz": 120, "isaac_sim": "4.5"},
                         "run_status": "budget_exhausted", "finished_utc": "2026-09-10T00:00:00Z",
                         "wall_seconds": 1800.}
        self.metadata["source_hashes"]["frigidaire/src/dishsim_frigidaire/random_pose_experiment.py"] = "old-scheduler"
        generators = cell_generators(0)
        self.trials = []
        for kind, rack, outcome in (("dinner_plate", "LowerRack", "initial_collision"),
                                    ("dinner_plate", "UpperRack", "accepted"),
                                    ("bowl", "LowerRack", "interrupted")):
            pose = sample_pose(generators[(kind, rack)], self.points[kind], self.domains["sampling_bounds_by_rack"][rack])
            trial_id = f"{kind}_{rack}_00000"
            trial = {"trial_id": trial_id, "kind": kind, "rack": rack, "sample_index": 0,
                     "outcome": outcome, "sampled_pose": pose,
                     "settled_pose": deepcopy(pose) if outcome == "accepted" else None,
                     "final_pose": deepcopy(pose) if outcome == "accepted" else None,
                     "trace_file": f"traces/{trial_id}.jsonl", "original_extra": {"preserve": [1, 2, 3]}}
            self.trials.append(trial)
            (self.prior / "traces").mkdir(exist_ok=True)
            phase = "final_hold" if outcome == "accepted" else "loaded_retraction" if outcome == "interrupted" else "trial_reset"
            (self.prior / trial["trace_file"]).write_text(json.dumps({"phase": phase, "dish": pose}) + "\n")
        self.write_prior()
        self.current = deepcopy(self.metadata)
        self.current["source_hashes"]["frigidaire/src/dishsim_frigidaire/random_pose_experiment.py"] = "new-scheduler"

    def write_prior(self):
        summary = build_summary(self.trials, samples_per_cell=2,
                                baselines={rack: {"result": "PASS"} for rack in RACKS}, status="budget_exhausted")
        for name, value in (("metadata.json", self.metadata), ("summary.json", summary),
                            ("accepted_poses.json", {"trials": [t for t in self.trials if t["outcome"] == "accepted"]})):
            (self.prior / name).write_text(json.dumps(value))
        (self.prior / "trials.jsonl").write_text("".join(json.dumps(t) + "\n" for t in self.trials))

    def test_classified_carry_over_excludes_unresolved_and_records_source_change(self):
        state = load_resume(self.prior, self.current)
        self.assertEqual(len(state.completed_trials), 2)
        self.assertNotIn(self.trials[-1]["trial_id"], state.completed_trials)
        self.assertEqual(state.unresolved_trials, [self.trials[-1]])
        self.assertEqual(state.prior_evaluation_attempts, 3)
        self.assertEqual(state.prior_wall_seconds, 1800.)
        self.assertEqual(len(state.provenance["allowed_source_differences"]), 1)
        validate_runtime(state, self.current["runtime"])
        validate_sample_stream(state, self.points, self.domains)

    def test_physics_or_inputs_or_sampling_changes_rejected(self):
        for key in ("seed", "kinds", "racks", "samples_per_cell", "domains", "thresholds", "assets", "physics"):
            with self.subTest(key=key):
                current = deepcopy(self.current)
                if key == "assets":
                    current["inputs"]["asset_hashes"]["fixture.usdc"] = "modified"
                elif key == "physics":
                    current["source_hashes"][PHYSICS_SOURCE_PATHS[0]] = "modified"
                else:
                    current[key] = "modified"
                with self.assertRaises(ValueError):
                    load_resume(self.prior, current)

    def test_runtime_changes_rejected(self):
        state = load_resume(self.prior, self.current)
        with self.assertRaisesRegex(ValueError, "runtime"):
            validate_runtime(state, {**self.current["runtime"], "physics_hz": 60})

    def test_interrupted_sample_is_validated_before_retry(self):
        self.trials[-1]["sampled_pose"]["bbox_center_m"][0] += .001
        self.write_prior()
        state = load_resume(self.prior, self.current)
        with self.assertRaisesRegex(ValueError, "seeded sample differs.*bowl_LowerRack"):
            validate_sample_stream(state, self.points, self.domains)

    def test_copy_preserves_rows_traces_and_prior_and_archives_interruption(self):
        state = load_resume(self.prior, self.current)
        before = {str(p.relative_to(self.prior)): hashlib.sha256(p.read_bytes()).hexdigest()
                  for p in self.prior.rglob("*") if p.is_file()}
        output = self.root / "resumed"
        with ExperimentReport(output, samples_per_cell=2) as report:
            provenance = seed_report(report, state)
            self.assertEqual(report.trials, self.trials[:2])
            with self.assertRaisesRegex(ValueError, "no recorded trials"):
                seed_report(report, state)
        self.assertEqual([json.loads(line) for line in (output / "trials.jsonl").read_text().splitlines()], self.trials[:2])
        for trial in self.trials[:2]:
            self.assertEqual((output / trial["trace_file"]).read_bytes(), (self.prior / trial["trace_file"]).read_bytes())
        unresolved = self.trials[-1]
        self.assertFalse((output / unresolved["trace_file"]).exists())
        history = output / provenance["history_directory"]
        self.assertEqual(json.loads((history / "unresolved_trials.jsonl").read_text()), unresolved)
        self.assertEqual((history / unresolved["trace_file"]).read_bytes(), (self.prior / unresolved["trace_file"]).read_bytes())
        after = {str(p.relative_to(self.prior)): hashlib.sha256(p.read_bytes()).hexdigest()
                 for p in self.prior.rglob("*") if p.is_file()}
        self.assertEqual(before, after)

    def test_cumulative_history_counts_survive_resume_chains(self):
        self.metadata.update(cumulative_wall_seconds=3600., cumulative_evaluation_attempts=4)
        self.write_prior()
        state = load_resume(self.prior, self.current)
        self.assertEqual(state.prior_wall_seconds, 3600.)
        self.assertEqual(state.prior_evaluation_attempts, 4)

    def test_corrupt_summary_duplicate_ids_and_missing_accepted_trace_rejected(self):
        path = self.prior / "summary.json"
        summary = json.loads(path.read_text())
        summary["totals"]["attempted"] += 1
        path.write_text(json.dumps(summary))
        with self.assertRaisesRegex(ValueError, "counts disagree"):
            load_resume(self.prior, self.current)
        self.write_prior()
        accepted_trace = self.prior / self.trials[1]["trace_file"]
        accepted_trace.unlink()
        with self.assertRaisesRegex(ValueError, "Missing or external trace"):
            load_resume(self.prior, self.current)
        accepted_trace.write_text(json.dumps({"phase": "final_hold", "dish": {}}) + "\n")
        self.trials[2]["trial_id"] = self.trials[0]["trial_id"]
        (self.prior / "trials.jsonl").write_text("".join(json.dumps(t) + "\n" for t in self.trials))
        with self.assertRaisesRegex(ValueError, "duplicate"):
            load_resume(self.prior, self.current)

    def test_prior_mutation_between_load_and_copy_is_rejected(self):
        state = load_resume(self.prior, self.current)
        (self.prior / "metadata.json").write_text("{}")
        with ExperimentReport(self.root / "resumed", samples_per_cell=2) as report:
            with self.assertRaisesRegex(ValueError, "changed after validation"):
                seed_report(report, state)
            self.assertEqual(report.trials, [])


if __name__ == "__main__":
    unittest.main()
