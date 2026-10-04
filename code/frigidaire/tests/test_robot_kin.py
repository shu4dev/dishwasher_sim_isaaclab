"""Kit-free tests of the UR5e kinematics and the 2F-85 bowl grasp geometry (code/frigidaire/src/dishsim_frigidaire/robot)."""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from dishsim_frigidaire.robot import grasp as G, kin  # noqa: E402


def test_ik_round_trip_on_random_reachable_poses():
    rng = np.random.default_rng(0)
    for _ in range(20):
        q = rng.uniform(-np.pi, np.pi, 6)
        q[2] = rng.uniform(-2.5, 2.5)                     # stay off the elbow singularity
        T = kin.fk_wrist3(q)
        sols = kin.ik_wrist3_all(T, q_seed=q)
        assert len(sols) > 0
        assert all(np.allclose(kin.fk_wrist3(s), T, atol=1e-6) for s in sols)


def pose(z_axis, origin=(0., 0., 0.)):
    a = np.asarray(z_axis, dtype=float) / np.linalg.norm(z_axis)
    x = np.cross([0., 1., 0.], a) if abs(a[1]) < .9 else np.cross([1., 0., 0.], a)
    x /= np.linalg.norm(x)
    T = np.eye(4)
    T[:3, 0], T[:3, 1], T[:3, 2], T[:3, 3] = x, np.cross(a, x), a, origin
    return T


def test_rim_grasps_sit_on_the_wall_and_approach_down():
    gs = G.bowl_grasps(pose([0, 0, 1], (.5, 0., .4)))
    assert gs and all(g["kind"] == "rim" for g in gs)
    for g in gs:
        T = g["T_tcp"]
        assert np.allclose(T[:3, 2], [0, 0, -1])                      # straight down
        r = np.linalg.norm(T[:2, 3] - [.5, 0.])
        assert r == pytest.approx(G.RIM_R - G.WALL / 2, abs=1e-9)      # on the wall's mid thickness
        assert T[2, 3] == pytest.approx(.4 + G.HEIGHT - G.RIM_INSERT_M)
        radial = (T[:2, 3] - [.5, 0.]) / r
        assert abs(float(np.dot(T[:2, 1], radial))) == pytest.approx(1.)   # the jaws close across the wall


def test_foot_and_side_grasps():
    foot = G.bowl_grasps(pose([0, 0, -1], (0., 0., .5)))
    assert foot and all(g["kind"] == "foot" for g in foot)
    assert all(g["T_tcp"][2, 3] == pytest.approx(.5 - G.FOOT_INSERT_M) for g in foot)
    side = G.bowl_grasps(pose([1, 0, 0], (0., 0., .5)))
    assert len(side) == 2 and all(g["kind"] == "side" for g in side)
    assert all(abs(g["T_tcp"][1, 3]) == pytest.approx(G.RIM_R - G.WALL / 2) for g in side)


def test_counter_check_and_tilt():
    over = G.tcp_frame(np.array([0., -.25, .6]), [1., 0., 0.])           # under the slab edge, straight down
    assert G.hits_counter(over)
    clear = G.tcp_frame(np.array([0., -.60, .6]), [1., 0., 0.])          # in front of the counter
    assert not G.hits_counter(clear)
    tilted = G.tilt(over, 35.)
    assert np.allclose(tilted[:3, 3], over[:3, 3])                       # tilting keeps the grasp point
    assert tilted[1, 2] > 0                                              # the approach now points toward +y ...
    assert (tilted[:3, 3] - .2 * tilted[:3, 2])[1] < over[1, 3]          # ... so the gripper body leans to -y
    above = G.tcp_frame(np.array([0., 0., .99]), [1., 0., 0.])           # a grasp on the counter top
    assert not G.hits_counter(above)
