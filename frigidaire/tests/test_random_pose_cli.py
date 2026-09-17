"""Launcher-independent failure reporting and Kit shutdown ordering."""
from contextlib import redirect_stdout
import importlib.util
from io import StringIO
import json
from pathlib import Path
import tempfile
import time
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import patch


ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT/"frigidaire/scripts/experiment/frigidaire_random_pose_experiment.py"
spec = importlib.util.spec_from_file_location("random_pose_cli_under_test", SCRIPT)
cli = importlib.util.module_from_spec(spec)
spec.loader.exec_module(cli)


class CLITests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.directory = Path(self.temp.name)/"run"
        parser = cli.parser_for_experiment()
        cli.add_minimal_launcher_arguments(parser)
        self.args = parser.parse_args(["--out-dir", str(self.directory), "--no-plots"])
        self.domains = {"sampling_bounds_by_rack": {}, "interior_bounds": {}}
        self.inputs = {"asset_hashes": {"fdpc4221as.usdc": "fixture"}}
        self.output = StringIO()
        for name, value in (("dishsim_frigidaire.random_pose_assets.validate_inputs", self.inputs),
                            ("dishsim_frigidaire.random_poses.source_geometry_domains", self.domains)):
            context = patch(name, return_value=value)
            context.start()
            self.addCleanup(context.stop)

    def run_cli(self, **kwargs):
        with redirect_stdout(self.output):
            result = cli.run(self.args, time.monotonic(), **kwargs)
        return result, json.loads((self.directory/"summary.json").read_text())

    def test_preflight_reports_no_physics_and_no_acceptance_rate(self):
        self.args.preflight_only = True
        result, summary = self.run_cli()
        self.assertEqual(result, 0)
        self.assertEqual(summary["status"], "preflight_only")
        self.assertEqual(summary["totals"]["unattempted"], 600)
        self.assertIsNone(summary["totals"]["confirmed_successes_per_attempt"])
        self.assertIn("[RESULT] NOT_RUN", self.output.getvalue())
        self.assertNotIn("[RESULT] PASS", self.output.getvalue())

    def test_unavailable_runtime_saves_honest_diagnostic(self):
        result, summary = self.run_cli(import_error=ModuleNotFoundError("isaaclab is unavailable"))
        self.assertEqual(result, 1)
        self.assertEqual(summary["status"], "runtime_unavailable")
        self.assertEqual(summary["totals"]["attempted"], 0)
        metadata = json.loads((self.directory/"metadata.json").read_text())
        self.assertEqual(metadata["runtime_status"], "NOT_RUN")
        self.assertIn("isaaclab is unavailable", metadata["runtime_error"])
        self.assertIn("[RESULT] NOT_RUN", self.output.getvalue())

    def test_result_is_flushed_before_app_close_can_exit_process(self):
        events = []

        def close():
            events.append("close")
            self.assertIn("[RESULT] INCOMPLETE", self.output.getvalue())
            self.assertTrue((self.directory/"summary.json").is_file())

        app = SimpleNamespace(close=close)
        launcher = lambda args: SimpleNamespace(app=app)
        backend = SimpleNamespace(runtime_settings={})
        # The existing media module targets Python 3.10; this suite also runs on
        # the host's Python 3.8 and only needs its Kit shutdown boundary here.
        media = ModuleType("dishsim.media")
        media.release_sim_for_close = lambda: events.append("release")
        failed = {"status": "baseline_blocked", "baselines": {
            rack: {"result": "FAIL"} for rack in ("LowerRack", "UpperRack")}}
        with patch("dishsim_frigidaire.random_pose_runtime.IsaacPoseBackend", return_value=backend), \
                patch("dishsim_frigidaire.random_pose_experiment.execute_trials", return_value=failed), \
                patch.object(cli.importlib.metadata, "version", return_value="fixture-version"), \
                patch.dict(cli.sys.modules, {"dishsim.media": media}):
            result, summary = self.run_cli(launcher_class=launcher)
        self.assertEqual(result, 1)
        self.assertEqual(summary["status"], "baseline_blocked")
        self.assertEqual(events, ["release", "close"])

    def test_resume_validates_before_import_and_accounts_for_prior_retry(self):
        self.args.resume_from = Path(self.temp.name)/"prior"
        pose = {"position_m": [0., 0., 0.], "quaternion_xyzw": [0., 0., 0., 1.]}
        previous = {"trial_id": "dinner_plate_LowerRack_00000", "kind": "dinner_plate",
                    "rack": "LowerRack", "sample_index": 0, "sampled_pose": pose,
                    "settled_pose": None, "final_pose": None, "outcome": "initial_collision"}
        retried = dict(previous, trial_id="dinner_plate_UpperRack_00000", rack="UpperRack")
        state = SimpleNamespace(provenance={"prior_run": "fixture"},
                                completed_trials={previous["trial_id"]: previous},
                                unresolved_trials=[dict(retried, outcome="interrupted")],
                                prior_wall_seconds=50., prior_evaluation_attempts=2)
        events = []

        def seed(report, resume):
            self.assertIs(resume, state)
            events.append("seed")
            report.record_trial(previous)

        def execute(backend, report, **kwargs):
            events.append("execute")
            self.assertEqual(kwargs["completed_trials"], state.completed_trials)
            self.assertEqual(report.trials, [previous])
            report.record_trial(retried)
            return {"status": "budget_exhausted", "baselines": {
                rack: {"result": "PASS"} for rack in ("LowerRack", "UpperRack")}}

        media = ModuleType("dishsim.media")
        media.release_sim_for_close = lambda: None
        launcher = lambda args: SimpleNamespace(app=SimpleNamespace(close=lambda: None))
        backend = SimpleNamespace(runtime_settings={}, points={})
        with patch("dishsim_frigidaire.random_pose_resume.load_resume", return_value=state), \
                patch("dishsim_frigidaire.random_pose_resume.validate_runtime", side_effect=lambda *_: events.append("runtime")), \
                patch("dishsim_frigidaire.random_pose_resume.validate_sample_stream", side_effect=lambda *_: events.append("samples")), \
                patch("dishsim_frigidaire.random_pose_resume.seed_report", side_effect=seed), \
                patch("dishsim_frigidaire.random_pose_runtime.IsaacPoseBackend", return_value=backend), \
                patch("dishsim_frigidaire.random_pose_experiment.execute_trials", side_effect=execute), \
                patch.object(cli.importlib.metadata, "version", return_value="fixture-version"), \
                patch.dict(cli.sys.modules, {"dishsim.media": media}):
            result, summary = self.run_cli(launcher_class=launcher)
        self.assertEqual(events, ["runtime", "samples", "seed", "execute"])
        self.assertEqual(result, 1)
        self.assertEqual(summary["totals"]["attempted"], 2)
        metadata = json.loads((self.directory/"metadata.json").read_text())
        self.assertTrue(metadata["resume"]["applied"])
        self.assertEqual(metadata["inherited_classified_trials"], 1)
        self.assertEqual(metadata["new_evaluation_attempts"], 1)
        self.assertEqual(metadata["cumulative_evaluation_attempts"], 3)
        self.assertAlmostEqual(metadata["cumulative_wall_seconds"], metadata["wall_seconds"]+50.)


if __name__ == "__main__":
    unittest.main()
