# Copyright (c) 2026, dishsim project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Kit-free checks on the evaluation aggregation (Phase 3 reads artifacts, nothing else)."""

import json
import os

from dishsim import metrics as dmetrics


def _trial(tag, success, slot=2, planner="rrt_connect", plan_time=1.0, stage=None, lateral=0.005):
    return {
        "schema_version": 1, "trial": tag, "slot": slot, "seed": 0, "repeat": 0,
        "object": "mug", "scenario": "both_out", "placement_mode": "floor_stand",
        "planner": {"name": planner, "budget_s": 20.0},
        "success": success, "failure_stage": stage, "failure_detail": None,
        "plan_time_s": plan_time, "path_len_rad": 6.5, "exec_steps": 700,
        "final_pose_err": {"lateral_m": lateral, "tilt_deg": 0.5} if success else None,
    }


def _write_run(root, run_id, trials, manifest=None):
    run_dir = os.path.join(root, "experiments", run_id)
    os.makedirs(os.path.join(run_dir, "trials"), exist_ok=True)
    for t in trials:
        with open(os.path.join(run_dir, "trials", f"{t['trial']}.json"), "w") as f:
            json.dump(t, f)
    with open(os.path.join(run_dir, "manifest.json"), "w") as f:
        json.dump(manifest or {"run_id": run_id, "planner": {"name": "rrt_connect"}}, f)
    return run_dir


def test_load_run_reads_manifest_and_trials(tmp_path):
    run_dir = _write_run(str(tmp_path), "r1", [_trial("t0", True), _trial("t1", False, stage="planner-timeout")])
    manifest, trials = dmetrics.load_run(run_dir)
    assert manifest["run_id"] == "r1"
    assert len(trials) == 2 and {t["trial"] for t in trials} == {"t0", "t1"}


def test_summarize_counts_and_breakdowns():
    trials = [
        _trial("t0", True, slot=2),
        _trial("t1", False, slot=2, stage="planner-timeout"),
        _trial("t2", True, slot=7),
    ]
    s = dmetrics.summarize(trials, {"run_id": "r1"})
    assert s["n_trials"] == 3 and s["n_success"] == 2
    assert s["success_rate"] == round(2 / 3, 4)
    assert s["by_slot"]["2"] == {"trials": 2, "success": 1, "rate": 0.5}
    assert s["by_slot"]["7"]["rate"] == 1.0
    assert s["failure_stages"] == {"planner-timeout": 1}
    assert s["by_object"]["mug"]["trials"] == 3


def test_summarize_groups_by_planner_name():
    """The planner block is a dict; grouping must key on its name, not its repr."""
    trials = [_trial("a", True, planner="rrt_connect"), _trial("b", False, planner="prm")]
    s = dmetrics.summarize(trials)
    assert set(s["by_planner"]) == {"rrt_connect", "prm"}
    assert s["by_planner"]["rrt_connect"]["rate"] == 1.0
    assert s["by_planner"]["prm"]["rate"] == 0.0


def test_summarize_statistics_and_empty_fields():
    trials = [_trial("a", True, plan_time=1.0), _trial("b", True, plan_time=3.0)]
    s = dmetrics.summarize(trials)
    assert s["plan_time_s"]["mean"] == 2.0 and s["plan_time_s"]["median"] == 2.0
    assert s["plan_time_s"]["min"] == 1.0 and s["plan_time_s"]["max"] == 3.0
    assert s["placement_lateral_m"]["n"] == 2
    # a run where nothing succeeded has no placement errors to summarize
    s2 = dmetrics.summarize([_trial("c", False, stage="planner-timeout")])
    assert s2["placement_lateral_m"] == {"n": 0}


def test_summarize_handles_no_trials():
    s = dmetrics.summarize([])
    assert s["n_trials"] == 0 and s["success_rate"] == 0.0


def test_latest_run_prefers_the_pointer(tmp_path):
    root = str(tmp_path)
    _write_run(root, "old", [_trial("t0", True)])
    _write_run(root, "new", [_trial("t0", True)])
    with open(os.path.join(root, "experiments", "LATEST"), "w") as f:
        f.write("old\n")
    assert os.path.basename(dmetrics.latest_run(root)) == "old"
    # a stale pointer falls back to the most recent directory
    with open(os.path.join(root, "experiments", "LATEST"), "w") as f:
        f.write("deleted-run\n")
    assert os.path.basename(dmetrics.latest_run(root)) in {"old", "new"}


def test_compare_runs_ranks_by_success_then_speed():
    rows = dmetrics.compare_runs({
        "slow_perfect": [_trial("a", True, planner="prm", plan_time=9.0)],
        "fast_perfect": [_trial("a", True, planner="rrt_connect", plan_time=1.0)],
        "failing": [_trial("a", False, planner="rrt_star", plan_time=0.1, stage="planner-timeout")],
    })
    assert [r["run_id"] for r in rows] == ["fast_perfect", "slow_perfect", "failing"]
    assert rows[0]["planner"] == "rrt_connect"
