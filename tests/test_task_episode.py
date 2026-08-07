# Copyright (c) 2026, dishsim project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""The episode record — Phase 2's multi-object artifact, and its aggregation.

Per-pick records deliberately keep the existing trial schema so Phase 3 reads multi-object runs
unchanged; the episode record carries only what a per-pick record cannot express, which is the
order picks were made in and why the episode ended when it did.

Run with:
    PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /workspace/isaaclab/env_isaaclab/bin/python -m pytest \
        tests/test_task_episode.py -v
"""

import json

import numpy as np
import pytest

from dishsim.task.episode import (
    EPISODE_SCHEMA_VERSION,
    EpisodeResult,
    PickRecord,
    summarize_episodes,
    write_episode,
)


def make_result(status="cleared", picks=(("mug_00", True, None), ("cup_00", True, None))):
    r = EpisodeResult(episode_id="ep001", seed=7, classes=["mug", "cup"], cost_fn="nearest_first")
    for i, (item_id, ok, stage) in enumerate(picks):
        r.picks.append(PickRecord(item_id=item_id, object_class=item_id.rsplit("_", 1)[0],
                                  order=i, goal_slot=i + 1, success=ok, failure_stage=stage,
                                  cost=0.1 * (i + 1), n_candidates=len(picks) - i,
                                  plan_time_s=1.5))
    r.status = status
    r.unplaced = [p.item_id for p in r.picks if not p.success]
    return r


def test_record_round_trips_through_json(tmp_path):
    r = make_result()
    path = write_episode(str(tmp_path / "ep001.json"), r)
    doc = json.load(open(path))
    assert doc["schema_version"] == EPISODE_SCHEMA_VERSION
    assert doc["episode_id"] == "ep001" and doc["seed"] == 7
    assert doc["pick_order"] == ["mug_00", "cup_00"]
    assert doc["n_objects"] == 2 and doc["n_placed"] == 2
    assert doc["cost_fn"] == "nearest_first"


def test_extra_provenance_is_merged(tmp_path):
    path = write_episode(str(tmp_path / "e.json"), make_result(),
                         extra={"planner": {"name": "rrt_connect"}, "config_hash": "abc123"})
    doc = json.load(open(path))
    assert doc["planner"]["name"] == "rrt_connect" and doc["config_hash"] == "abc123"


def test_numpy_values_serialise(tmp_path):
    """Poses and counts arrive as numpy; a record that cannot be written is a lost episode."""
    r = make_result()
    r.support_graph = {"a": {"b"}}
    r.layout_rejection = {"draws": np.int64(12), "unreachable": np.int64(3)}
    doc = json.load(open(write_episode(str(tmp_path / "e.json"), r)))
    assert doc["support_graph"] == {"a": ["b"]}
    assert doc["layout_rejection"]["draws"] == 12


def test_a_deadlock_records_every_remaining_object_and_reason(tmp_path):
    r = make_result(status="deadlock", picks=(("mug_00", True, None),))
    r.unplaced = ["cup_00", "tumbler_00"]
    r.blocked_reasons = {"cup_00": "no collision-free grasp in the current world",
                         "tumbler_00": "supports ['cup_00'] — must be cleared first"}
    r.n_recoveries = 2
    doc = json.load(open(write_episode(str(tmp_path / "e.json"), r)))
    assert doc["status"] == "deadlock"
    assert sorted(doc["unplaced"]) == ["cup_00", "tumbler_00"]
    assert set(doc["blocked_reasons"]) == {"cup_00", "tumbler_00"}
    assert doc["n_recoveries"] == 2


def test_n_placed_counts_successes_only():
    r = make_result(status="partial",
                    picks=(("mug_00", True, None), ("cup_00", False, "pick-grasp-fault")))
    assert r.n_placed == 1 and len(r.picks) == 2


# ---- aggregation --------------------------------------------------------------------------------


def test_summarize_empty():
    s = summarize_episodes([])
    assert s["n_episodes"] == 0 and s["clear_rate"] == 0.0


def test_summarize_counts_clear_rate_and_pick_rate():
    docs = [
        make_result().to_json(),
        make_result(status="partial",
                    picks=(("mug_00", True, None), ("cup_00", False, "pick-grasp-fault"))).to_json(),
    ]
    s = summarize_episodes(docs)
    assert s["n_episodes"] == 2 and s["n_cleared"] == 1
    assert s["clear_rate"] == 0.5
    assert s["n_picks"] == 4 and s["n_pick_success"] == 3
    assert s["pick_success_rate"] == 0.75


def test_summarize_breaks_down_by_class_and_failure_stage():
    docs = [make_result(status="partial",
                        picks=(("mug_00", True, None), ("mug_01", False, "pick-grasp-fault"),
                               ("cup_00", False, "planner-timeout"))).to_json()]
    s = summarize_episodes(docs)
    assert s["by_class"]["mug"] == {"picks": 2, "success": 1, "rate": 0.5}
    assert s["by_class"]["cup"]["rate"] == 0.0
    assert s["failure_stages"] == {"pick-grasp-fault": 1, "planner-timeout": 1}


def test_summarize_counts_deadlocks_and_recoveries():
    a = make_result(status="deadlock", picks=(("mug_00", True, None),))
    a.n_recoveries = 2
    b = make_result()
    s = summarize_episodes([a.to_json(), b.to_json()])
    assert s["n_deadlocks"] == 1 and s["n_recoveries"] == 2
    assert s["status_counts"] == {"cleared": 1, "deadlock": 1}


@pytest.mark.parametrize("status", ["cleared", "partial", "deadlock", "error"])
def test_every_status_survives_aggregation(status):
    s = summarize_episodes([make_result(status=status).to_json()])
    assert s["status_counts"][status] == 1


# ---- home anchoring ------------------------------------------------------------------------------

def test_home_fields_round_trip():
    r = make_result()
    r.start_home_err_rad = 0.0
    r.end_home_err_rad = 1.4e-05
    r.home_return_status = "planned"
    r.post_home_displacement_mm = {"cup_00": 0.0221}
    d = json.loads(json.dumps(r.to_json()))
    assert d["start_home_err_rad"] == 0.0
    assert d["end_home_err_rad"] == pytest.approx(1.4e-05)
    assert d["home_return_status"] == "planned"
    assert d["post_home_displacement_mm"] == {"cup_00": 0.022}


def test_home_fields_default_to_none_not_missing():
    """Phase 3 reads these by key; a missing key and a null mean different things."""
    d = make_result().to_json()
    for k in ("start_home_err_rad", "end_home_err_rad", "home_return_status"):
        assert k in d and d[k] is None
    assert d["post_home_displacement_mm"] == {}


def test_summarize_reports_worst_parking_error_and_lerp_count():
    a, b = make_result(), make_result()
    a.end_home_err_rad, a.home_return_status = 0.001, "planned"
    b.end_home_err_rad, b.home_return_status = 0.04, "lerp"
    s = summarize_episodes([a.to_json(), b.to_json()])
    assert s["max_end_home_err_rad"] == pytest.approx(0.04)
    assert s["n_home_lerp"] == 1


def test_summarize_tolerates_episodes_without_home_fields():
    """Schema-1 records on disk predate the home fields and must not crash aggregation."""
    old = make_result().to_json()
    for k in ("start_home_err_rad", "end_home_err_rad", "home_return_status"):
        old.pop(k)
    s = summarize_episodes([old])
    assert s["max_end_home_err_rad"] is None and s["n_home_lerp"] == 0


def test_runner_error_path_construction_is_valid():
    """Mirrors run_task.py's fallback EpisodeResult exactly — guards those keyword names."""
    r = EpisodeResult(episode_id="ep004", seed=4, classes=["cup", "cup", "tumbler"],
                      cost_fn="nearest_first", status="error",
                      detail="RuntimeError: boom", unplaced=["cup_00", "cup_01", "tumbler_00"])
    d = json.loads(json.dumps(r.to_json()))
    assert d["status"] == "error" and d["n_placed"] == 0
    assert d["detail"].startswith("RuntimeError")
    assert d["unplaced"] == ["cup_00", "cup_01", "tumbler_00"]
