"""Random-pose evidence accounting and persistence without the Isaac runtime."""
from copy import deepcopy
import csv
import json
import math
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from dishsim_frigidaire.random_pose_reports import ExperimentReport, build_summary, OUTCOMES


def trial(index, outcome="accepted", kind="dinner_plate", rack="LowerRack"):
    pose = {"position_m": [.01, .02, .03], "quaternion_xyzw": [0., 0., 0., 1.]}
    return {"trial_id": f"{kind}:{rack}:{index}", "sample_index": index,
            "kind": kind, "rack": rack, "outcome": outcome,
            "sampled_pose": deepcopy(pose), "settled_pose": deepcopy(pose),
            "final_pose": deepcopy(pose), "timings": {"seconds": .1}}


def check_all_generated_proposals_count_but_errors_remain_unresolved():
    rows = [trial(index, outcome) for index, outcome in enumerate(OUTCOMES)]
    summary = build_summary(rows, samples_per_cell=10,
                            baselines={"LowerRack": {"result": "PASS"},
                                       "UpperRack": {"result": "FAIL", "reason": "empty rack jams"}})
    totals = summary["totals"]
    assert totals["planned"] == 60
    assert totals["attempted"] == 8
    assert totals["completed"] == totals["classified"] == 6
    assert totals["unresolved"] == 2
    assert totals["unattempted"] == 52
    assert totals["accepted"] == 1
    assert totals["confirmed_successes_per_attempt"] == 1 / 8
    assert totals["blocked_cells"] == 3
    assert all(totals[outcome] == 1 for outcome in OUTCOMES)
    upper = next(cell for cell in summary["cells"] if cell["rack"] == "UpperRack")
    assert upper["blocked"] is True
    assert upper["confirmed_successes_per_attempt"] is None


def check_zero_trials_preserve_not_run_status_and_no_acceptance_rate(tmp_path):
    with ExperimentReport(tmp_path) as report:
        report.write_metadata({"seed": 0})
        summary = report.finalize(status="runtime_unavailable", plots=False,
                                  baselines={"LowerRack": {"result": "ERROR", "reason": "No GPU"}})
    assert summary["simulation_run"] is False
    assert summary["status"] == "runtime_unavailable"
    assert summary["totals"]["confirmed_successes_per_attempt"] is None
    assert summary["totals"]["unattempted"] == 600
    markdown = (tmp_path / "report.md").read_text()
    assert "No random-pose trials ran" in markdown
    assert "runtime_unavailable" in markdown
    assert "ERROR (blocked)" in markdown


def check_durable_records_replay_and_reports_agree(tmp_path):
    rows = [trial(0), trial(1, "initial_collision"), trial(2, "interrupted")]
    with patch("dishsim_frigidaire.random_pose_reports.os.fsync") as fsync:
        with ExperimentReport(tmp_path, samples_per_cell=3) as report:
            report.write_metadata({"seed": 42, "asset_hashes": {"asset.usdc": "abc"}})
            for row in rows:
                report.record_trial(row)
                # The evidence is readable before close/finalize, even if the process stops now.
                disk_rows = [json.loads(line) for line in (tmp_path / "trials.jsonl").read_text().splitlines()]
                assert disk_rows[-1] == row
                checkpoint = json.loads((tmp_path / "summary.json").read_text())
                assert checkpoint["totals"]["attempted"] == len(disk_rows)
            rows[0]["final_pose"]["position_m"][0] = 999
            summary = report.finalize(status="budget_exhausted", plots=False)
        assert fsync.call_count >= len(rows)
    assert json.loads((tmp_path / "summary.json").read_text()) == summary
    assert (tmp_path / "summary.json").stat().st_mode & 0o444 == 0o444
    assert json.loads((tmp_path / "metadata.json").read_text())["seed"] == 42
    accepted = json.loads((tmp_path / "accepted_poses.json").read_text())
    assert len(accepted["trials"]) == 1
    assert accepted["trials"][0]["final_pose"]["position_m"][0] == .01
    assert accepted["pose_frames"]["sampled_pose"] == "rack_local"
    assert accepted["pose_frames"]["final_pose"] == "world"
    with (tmp_path / "summary.csv").open() as stream:
        cells = list(csv.DictReader(stream))
    assert len(cells) == 6
    assert sum(int(cell["attempted"]) for cell in cells) == 3
    assert "1 / 3 attempted proposals" in (tmp_path / "report.md").read_text()


def check_nonempty_output_is_never_overwritten(tmp_path):
    existing = tmp_path / "evidence.txt"
    existing.write_text("preserve")
    with unittest.TestCase().assertRaisesRegex(FileExistsError, "must be empty"):
        ExperimentReport(tmp_path)
    assert existing.read_text() == "preserve"
    assert list(tmp_path.iterdir()) == [existing]


def check_finalization_may_refresh_owned_files(tmp_path):
    with ExperimentReport(tmp_path, samples_per_cell=2) as report:
        report.record_trial(trial(0, "initial_collision"))
        report.finalize(status="running", plots=False)
        report.record_trial(trial(1))
        summary = report.finalize(status="complete", plots=False)
    assert summary["totals"]["attempted"] == 2
    assert json.loads((tmp_path / "summary.json").read_text())["status"] == "complete"


def check_invalid_records_do_not_append_or_inflate_counts(tmp_path, change):
    with ExperimentReport(tmp_path, samples_per_cell=2) as report:
        report.record_trial(trial(0))
        bad = trial(1)
        if change == "duplicate_id":
            bad["trial_id"] = trial(0)["trial_id"]
        elif change == "duplicate_sample":
            bad["sample_index"] = 0
        elif change == "unknown_outcome":
            bad["outcome"] = "passed"
        elif change == "unknown_cell":
            bad["rack"] = "Door"
        else:
            bad["sample_index"] = 2
        with unittest.TestCase().assertRaises(ValueError):
            report.record_trial(bad)
        assert len(report.trials) == 1
        assert len((tmp_path / "trials.jsonl").read_text().splitlines()) == 1


def check_poses_are_finite_complete_and_replayable(tmp_path, change):
    row = trial(0)
    if change == "nan":
        row["sampled_pose"]["position_m"][0] = math.nan
    elif change == "wrong_size":
        row["sampled_pose"]["position_m"] = [1, 2]
    elif change == "bad_quaternion":
        row["sampled_pose"]["quaternion_xyzw"] = [0, 0, 0, 0]
    elif change == "no_sample":
        row["sampled_pose"] = None
    else:
        row["final_pose"] = None
    with ExperimentReport(tmp_path) as report:
        with unittest.TestCase().assertRaises(ValueError):
            report.record_trial(row)
        assert (tmp_path / "trials.jsonl").read_text() == ""


def check_plot_artifacts_are_created_from_recorded_samples(tmp_path):
    try:
        import matplotlib  # noqa: F401
    except ImportError:
        raise unittest.SkipTest("matplotlib unavailable")
    with ExperimentReport(tmp_path, kinds=("dinner_plate",), racks=("LowerRack",), samples_per_cell=2) as report:
        report.record_trial(trial(0))
        report.record_trial(trial(1, "simulation_error"))
        summary = report.finalize(plots=True)
    assert summary["plots"]["status"] == "complete"
    assert len(summary["plots"]["files"]) == 2
    for filename in summary["plots"]["files"]:
        assert (tmp_path / filename).read_bytes().startswith(b"\x89PNG")


class RandomPoseReportTests(unittest.TestCase):
    def setUp(self):
        directory = tempfile.TemporaryDirectory(prefix="frigidaire-random-pose-report-")
        self.addCleanup(directory.cleanup)
        self.directory = Path(directory.name)

    def test_all_proposals_count_and_errors_are_unresolved(self):
        check_all_generated_proposals_count_but_errors_remain_unresolved()

    def test_no_trials_make_no_success_claim(self):
        check_zero_trials_preserve_not_run_status_and_no_acceptance_rate(self.directory)

    def test_durable_records_and_artifacts_agree(self):
        check_durable_records_replay_and_reports_agree(self.directory)

    def test_never_overwrite_existing_output(self):
        check_nonempty_output_is_never_overwritten(self.directory)

    def test_refresh_reports_within_owned_run(self):
        check_finalization_may_refresh_owned_files(self.directory)

    def test_reject_invalid_records_without_counting_them(self):
        for change in ("duplicate_id", "duplicate_sample", "unknown_outcome", "unknown_cell", "too_many"):
            with self.subTest(change=change):
                check_invalid_records_do_not_append_or_inflate_counts(self.directory / change, change)

    def test_require_replayable_finite_poses(self):
        for change in ("nan", "wrong_size", "bad_quaternion", "no_sample", "no_final"):
            with self.subTest(change=change):
                check_poses_are_finite_complete_and_replayable(self.directory / change, change)

    def test_plot_artifacts(self):
        check_plot_artifacts_are_created_from_recorded_samples(self.directory)


if __name__ == "__main__":
    unittest.main()
