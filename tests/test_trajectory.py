# Copyright (c) 2026, dishsim project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Kit-free checks on the trial trajectory format (the Phase 2 -> Phase 3 handoff).

The recorder itself needs a live scene, so these tests exercise the on-disk contract: writing
a recording the way :class:`TrajectoryRecorder` does, then loading and remapping it the way
the replay renderer does.
"""

import json
import os

import numpy as np
import pytest

from dishsim import trajectory as traj

ROBOT_JOINTS = [f"j{i}" for i in range(12)]
DW_JOINTS = ["door", "rack_up", "rack_down"]
BODIES = [f"link{i}" for i in range(16)]


def _write(tmp_path, n=200, stride=2, check_stride=60) -> str:
    rng = np.random.default_rng(0)
    captured = np.zeros(n, dtype=bool)
    captured[stride - 1 :: stride] = True
    check_idx = np.arange(0, n, check_stride, dtype=np.int32)
    meta = {
        "schema_version": traj.SCHEMA_VERSION,
        "sim_dt": 1 / 60,
        "trial": "trial_02_00_0",
        "robot_joint_names": ROBOT_JOINTS,
        "dishwasher_joint_names": DW_JOINTS,
        "robot_body_names": BODIES,
        "phase_legend": {str(k): v for k, v in traj.PHASE_LEGEND.items()},
        "quat_convention": "xyzw",
        "n_steps": n,
        "n_captured": int(captured.sum()),
    }
    path = os.path.join(tmp_path, "trial_02_00_0.npz")
    np.savez_compressed(
        path,
        robot_joint_pos=rng.normal(size=(n, 12)).astype(np.float32),
        dishwasher_joint_pos=rng.normal(size=(n, 3)).astype(np.float32),
        object_root_pose_w=rng.normal(size=(n, 7)).astype(np.float32),
        phase=np.full(n, traj.phase_code("place-execute"), dtype=np.uint8),
        weld=np.ones(n, dtype=bool),
        captured=captured,
        check_step_idx=check_idx,
        check_body_pose_w=rng.normal(size=(len(check_idx), 16, 7)).astype(np.float32),
        meta=np.array(json.dumps(meta)),
    )
    return path


def test_round_trip_shapes_and_dtypes(tmp_path):
    rec = traj.TrajectoryRecording.load(_write(str(tmp_path)))
    assert len(rec) == 200
    assert rec.robot_joint_pos.shape == (200, 12) and rec.robot_joint_pos.dtype == np.float32
    assert rec.dishwasher_joint_pos.shape == (200, 3)
    assert rec.object_root_pose_w.shape == (200, 7)
    assert rec.phase.dtype == np.uint8 and rec.weld.dtype == bool
    assert rec.check_body_pose_w.shape[1:] == (16, 7)
    assert rec.check_step_idx.shape[0] == rec.check_body_pose_w.shape[0]
    assert rec.meta["trial"] == "trial_02_00_0"


def test_capture_stride_is_explicit(tmp_path):
    """The captured mask is the video contract: it must survive the round trip exactly."""
    rec = traj.TrajectoryRecording.load(_write(str(tmp_path), n=200, stride=2))
    steps = rec.captured_steps
    assert len(steps) == 100 == rec.meta["n_captured"]
    assert np.all(np.diff(steps) == 2)


def test_joint_order_remaps_by_name(tmp_path):
    """A scene whose joint order differs from the recording still replays correctly."""
    rec = traj.TrajectoryRecording.load(_write(str(tmp_path)))
    live = list(reversed(ROBOT_JOINTS))
    cols = rec.joint_order(live, kind="robot")
    remapped = rec.robot_joint_pos[:, cols]
    # column i of the remapped array is the recording's column for live[i]
    for i, name in enumerate(live):
        assert np.array_equal(remapped[:, i], rec.robot_joint_pos[:, ROBOT_JOINTS.index(name)])
    assert np.array_equal(rec.joint_order(ROBOT_JOINTS), np.arange(12))


def test_joint_order_rejects_mismatched_sets(tmp_path):
    rec = traj.TrajectoryRecording.load(_write(str(tmp_path)))
    with pytest.raises(ValueError, match="joint sets differ"):
        rec.joint_order(ROBOT_JOINTS[:-1] + ["surprise"], kind="robot")


def test_schema_version_is_enforced(tmp_path):
    path = _write(str(tmp_path))
    with np.load(path, allow_pickle=False) as z:
        arrays = {k: z[k] for k in z.files}
    meta = json.loads(str(arrays["meta"]))
    meta["schema_version"] = traj.SCHEMA_VERSION + 1
    arrays["meta"] = np.array(json.dumps(meta))
    np.savez_compressed(path, **arrays)
    with pytest.raises(ValueError, match="schema_version"):
        traj.TrajectoryRecording.load(path)


def test_phase_legend_round_trip(tmp_path):
    rec = traj.TrajectoryRecording.load(_write(str(tmp_path)))
    assert rec.phase_name(0) == "place-execute"
    assert traj.phase_code("place-execute") == traj.PHASE_CODE["place-execute"]
    assert traj.phase_code("not-a-real-stage") == 255
    # legend keys and codes agree in both directions
    assert {int(k) for k in rec.meta["phase_legend"]} == set(traj.PHASE_LEGEND)


def test_time_parameterize_moved_and_unchanged():
    """time_parameterize keeps its contract after moving out of planning.py."""
    path = np.array([[0.0] * 6, [1.0] + [0.0] * 5])
    dense = traj.time_parameterize(path, speed_rad_s=0.5, dt=1 / 60)
    assert np.allclose(dense[0], path[0]) and np.allclose(dense[-1], path[1])
    step = 0.5 / 60
    assert len(dense) == 1 + int(np.ceil(1.0 / step))
    assert np.all(np.diff(dense[:, 0]) > 0)
