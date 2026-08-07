# Copyright (c) 2026, dishsim project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Multi-object recordings stay readable by Phase 3.

The whole decoupling argument rests on Phase 3 reading Phase 2's artifacts without knowing how
they were produced. Widening a recording from one object to N is exactly the change that can
break that quietly: the array grows, an old reader slices the first 7 columns, and playback
renders one object correctly while the rest sit at the origin.

Run with:
    PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /workspace/isaaclab/env_isaaclab/bin/python -m pytest \
        tests/test_task_recording.py -v
"""

import json

import numpy as np
import pytest

from dishsim import config
from dishsim.replay import scene_objects_for
from dishsim.trajectory import TrajectoryRecording


def make_npz(tmp_path, n_steps=5, keys=("mug_00", "cup_00"), classes=None, name="ep.npz"):
    """Hand-build a recording with ``len(keys)`` object pose blocks."""
    n_obj = len(keys)
    meta = {
        "schema_version": 1, "sim_dt": config.SIM_DT,
        "robot_joint_names": ["a"], "dishwasher_joint_names": ["d"],
        "robot_body_names": ["base_link"], "quat_convention": "xyzw", "frame": "world",
        "phase_legend": {}, "check_stride": 60, "n_steps": n_steps, "n_captured": n_steps,
    }
    if keys:
        meta["object_keys"] = list(keys)
        meta["object_classes"] = classes if classes is not None else {
            k: k.rsplit("_", 1)[0] for k in keys
        }
    poses = np.zeros((n_steps, 7 * n_obj), dtype=np.float32)
    for i in range(n_obj):
        poses[:, 7 * i + 0] = 0.1 * (i + 1)  # distinct x per object
        poses[:, 7 * i + 6] = 1.0  # unit quaternion (XYZW)
    path = str(tmp_path / name)
    np.savez_compressed(
        path,
        robot_joint_pos=np.zeros((n_steps, 1), dtype=np.float32),
        dishwasher_joint_pos=np.zeros((n_steps, 1), dtype=np.float32),
        object_root_pose_w=poses,
        phase=np.zeros(n_steps, dtype=np.uint8),
        weld=np.zeros(n_steps, dtype=bool),
        captured=np.ones(n_steps, dtype=bool),
        check_step_idx=np.array([0], dtype=np.int32),
        check_body_pose_w=np.zeros((1, 1, 7), dtype=np.float32),
        meta=np.array(json.dumps(meta)),
    )
    return path


def test_object_pose_array_widens_by_seven_per_object(tmp_path):
    rec = TrajectoryRecording.load(make_npz(tmp_path, keys=("mug_00", "cup_00", "tumbler_00")))
    assert rec.object_root_pose_w.shape[1] == 21
    assert rec.meta["object_keys"] == ["mug_00", "cup_00", "tumbler_00"]


def test_each_object_block_is_distinct(tmp_path):
    """Guards the failure where every object is written from the first block."""
    rec = TrajectoryRecording.load(make_npz(tmp_path, keys=("mug_00", "cup_00")))
    p = rec.object_root_pose_w
    assert p[0, 0] == pytest.approx(0.1) and p[0, 7] == pytest.approx(0.2)


def test_scene_objects_for_builds_one_spec_per_item(tmp_path):
    rec = TrajectoryRecording.load(make_npz(tmp_path, keys=("mug_00", "cup_00")))
    specs = scene_objects_for(rec)
    assert [s["name"] for s in specs] == ["mug_00", "cup_00"]
    assert specs[0]["usd_path"] == config.OBJECTS["mug"].usd_path
    assert specs[1]["usd_path"] == config.OBJECTS["cup"].usd_path
    # spawn poses come from the recording's first frame, not from a layout seed
    assert specs[0]["pos"][0] == pytest.approx(0.1)
    assert specs[1]["pos"][0] == pytest.approx(0.2)


def test_single_object_recordings_keep_the_legacy_path(tmp_path):
    """No object_keys at all: the caller must fall back to the legacy carried_object prim."""
    rec = TrajectoryRecording.load(make_npz(tmp_path, keys=()))
    assert scene_objects_for(rec) == []


def test_an_explicit_single_carried_object_also_uses_the_legacy_path(tmp_path):
    rec = TrajectoryRecording.load(make_npz(tmp_path, keys=("carried_object",),
                                            classes={"carried_object": "mug"}))
    assert scene_objects_for(rec) == []


def test_a_recording_without_a_class_map_fails_loudly(tmp_path):
    """An old multi-object recording cannot be rebuilt — say so instead of guessing a USD."""
    rec = TrajectoryRecording.load(make_npz(tmp_path, keys=("mug_00", "cup_00"), classes={}))
    with pytest.raises(RuntimeError, match="object_classes"):
        scene_objects_for(rec)
