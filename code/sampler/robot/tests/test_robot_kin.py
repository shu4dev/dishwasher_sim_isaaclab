"""Kit-free tests of the UR5e kinematics (code/sampler/robot)."""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

sys.path[:0] = [str(Path(__file__).resolve().parents[4] / "code"), str(Path(__file__).resolve().parents[4] / "code/util")]
from sampler.robot import kin  # noqa: E402


def test_ik_round_trip_on_random_reachable_poses():
    rng = np.random.default_rng(0)
    for _ in range(20):
        q = rng.uniform(-np.pi, np.pi, 6)
        q[2] = rng.uniform(-2.5, 2.5)                     # stay off the elbow singularity
        T = kin.fk_wrist3(q)
        sols = kin.ik_wrist3_all(T, q_seed=q)
        assert len(sols) > 0
        assert all(np.allclose(kin.fk_wrist3(s), T, atol=1e-6) for s in sols)

