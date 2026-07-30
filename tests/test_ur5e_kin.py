# Copyright (c) 2026, dishsim project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Validate the hand-rolled UR5e analytic FK/IK against Pinocchio and itself.

Ground truth: the ur_description URDF that ships with Isaac Sim, loaded with Pinocchio
(preinstalled in the Isaac python env). Run with the project venv:

    /workspace/isaaclab/env_isaaclab/bin/pytest tests/test_ur5e_kin.py -v
"""

import collections

import numpy as np
import pytest

from dishsim import ur5e_kin as K

URDF = (
    "/isaac-sim/extsDeprecated/isaacsim.robot_motion.motion_generation/"
    "motion_policy_configs/universal_robots/ur5e/ur5e.urdf"
)
JOINTS = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]
N_SAMPLES = 1000
SEED = 42


def _random_configs(n: int, seed: int) -> np.ndarray:
    rng = np.random.default_rng(seed)
    return rng.uniform(K.JOINT_LIMITS[:, 0], K.JOINT_LIMITS[:, 1], size=(n, 6))


@pytest.fixture(scope="module")
def pin_fk():
    pin = pytest.importorskip("pinocchio")
    model = pin.buildModelFromUrdf(URDF)
    data = model.createData()
    idx_q = [model.joints[model.getJointId(n)].idx_q for n in JOINTS]
    fid_w3 = model.getFrameId("wrist_3_link")
    fid_base = model.getFrameId("base_link")

    def fk(q6: np.ndarray) -> np.ndarray:
        q = np.zeros(model.nq)
        q[idx_q] = q6
        pin.framesForwardKinematics(model, data, q)
        return (data.oMf[fid_base].inverse() * data.oMf[fid_w3]).homogeneous

    return fk


def test_fk_matches_pinocchio(pin_fk):
    worst_pos, worst_rot = 0.0, 0.0
    for q in _random_configs(N_SAMPLES, SEED):
        T_ours = K.fk_wrist3(q)
        T_pin = pin_fk(q)
        worst_pos = max(worst_pos, float(np.linalg.norm(T_ours[:3, 3] - T_pin[:3, 3])))
        worst_rot = max(worst_rot, float(np.linalg.norm(T_ours[:3, :3] - T_pin[:3, :3])))
    print(f"\nFK vs Pinocchio over {N_SAMPLES}: worst pos {worst_pos:.3e} m, worst rot {worst_rot:.3e}")
    assert worst_pos < 1e-6
    assert worst_rot < 1e-6


def test_ik_round_trip():
    hist = collections.Counter()
    misses = 0
    for q in _random_configs(N_SAMPLES, SEED + 1):
        q = K.wrap_to_pi(q)  # canonical branch for comparison
        T = K.fk_wrist3(q)
        sols = K.ik_wrist3_all(T, q_seed=q)
        hist[len(sols)] += 1
        if len(sols) == 0:
            misses += 1
            continue
        # (a) some branch recovers the original configuration
        dev = np.min(np.max(np.abs(K.wrap_to_pi(sols - q)), axis=1))
        if dev > 1e-6:
            misses += 1
        # (b) every branch reconstructs the pose (enforced internally, re-checked here)
        for s in sols:
            T_re = K.fk_wrist3(s)
            assert np.linalg.norm(T_re[:3, 3] - T[:3, 3]) < 1e-6
            assert np.linalg.norm(T_re[:3, :3] - T[:3, :3]) < 1e-6
    print(f"\nIK branch histogram over {N_SAMPLES}: {dict(sorted(hist.items()))}, misses: {misses}")
    # tolerate a tiny singular tail (wrist/shoulder singularities where the branch is degenerate)
    assert misses <= N_SAMPLES * 0.005
    # the generic pose has 8 branches; it must be the mode
    assert hist.most_common(1)[0][0] == 8


def test_wrap_expansion_within_limits():
    q = np.array([0.1, -1.0, 1.0, -1.5, 3.0, -3.0])
    expanded = K.expand_2pi_wraps(np.array([q]))
    assert len(expanded) >= 2  # at least the original plus some wraps
    for v in expanded:
        assert np.all(v >= K.JOINT_LIMITS[:, 0]) and np.all(v <= K.JOINT_LIMITS[:, 1])
        # each wrapped variant maps to the same pose
        assert np.linalg.norm(K.fk_wrist3(v)[:3, 3] - K.fk_wrist3(q)[:3, 3]) < 1e-9


def test_dls_fallback():
    rng = np.random.default_rng(SEED + 2)
    for _ in range(5):
        q = rng.uniform(-np.pi / 2, np.pi / 2, 6)
        T = K.fk_wrist3(q)
        q_num = K.dls_ik(T, q + rng.normal(0, 0.1, 6))
        assert q_num is not None
        T_re = K.fk_wrist3(q_num)
        assert np.linalg.norm(T_re[:3, 3] - T[:3, 3]) < 1e-5
