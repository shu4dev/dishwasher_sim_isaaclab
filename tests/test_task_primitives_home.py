# Copyright (c) 2026, dishsim project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""The home-anchored episode invariant: every episode starts and ends at ``config.HOME_Q``.

The property under test is that :meth:`PickPlace.return_home` always leaves the arm at home and
always says HOW it got there. The "how" matters: the lerp fallback is not collision-checked, so a
caller that cannot distinguish it from a planned retreat cannot tell whether parking the arm may
have disturbed a placed load.

Everything runs against stubs — no collision cache, no simulator — the same way the grasp and
sequencer contracts are tested.

Run with:
    PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /workspace/isaaclab/env_isaaclab/bin/python -m pytest \
        tests/test_task_primitives_home.py -v
"""

import numpy as np
import pytest

from dishsim import config
from dishsim.task.primitives import GraspProfile, PickPlace
from dishsim.task.sequencer import TaskItem

HOME = np.array(config.HOME_Q, dtype=float)
TOL = float(config.TASK["home_tol_rad"])


class StubWorld:
    def __init__(self, in_coll=False):
        self._in_coll = in_coll

    def in_collision(self, q, return_pairs=False):
        return (self._in_coll, []) if return_pairs else self._in_coll


class StubStats:
    plan_time_s = 0.0


class StubPlanResult:
    def __init__(self, status, path_q=None):
        self.status = status
        self.path_q = path_q


class StubMotion:
    """The slice of MotionService that the home retreat uses, with a phase log."""

    def __init__(self, q, *, in_collision=False, plan_solves=True, payload=None):
        self._q = np.asarray(q, dtype=float)
        self.world = StubWorld(in_collision)
        self._plan_solves = plan_solves
        self._payload = payload
        self.stats = StubStats()
        self.log = []          # (kind, phase, detail)
        self.detached = False
        self.obstacles = {}

    # -- queried state
    def current_q(self):
        return self._q.copy()

    @property
    def payload(self):
        return self._payload

    # -- motion
    def plan(self, start_q, goal_qs, *, seed=None, debug=False):
        self.log.append(("plan", None, None))
        if not self._plan_solves:
            return StubPlanResult("failed")
        path = np.vstack([np.asarray(start_q, dtype=float), np.asarray(goal_qs)[0]])
        return StubPlanResult("solved", path)

    def execute(self, path_q, *, phase, aperture=None, exclude=(), on_step=None, gate=None):
        path = np.asarray(path_q, dtype=float)
        self.log.append(("execute", phase, len(path)))
        self._q = path[-1].copy()   # the arm actually ends where the path ends
        return None

    def hold(self, steps, *, phase, arm_q=None, aperture=None, on_step=None):
        self.log.append(("hold", phase, steps))
        if arm_q is not None:
            self._q = np.asarray(arm_q, dtype=float).copy()
        return None

    def set_aperture(self, aperture_rad, steps, *, phase, arm_q=None, on_step=None):
        self.log.append(("aperture", phase, aperture_rad))
        return aperture_rad

    def detach_payload(self):
        self.detached = True
        self._payload = None

    def set_obstacle(self, key, pieces, T):
        self.obstacles[key] = T


class StubScene:
    def __init__(self):
        self.welds = {}

    def set_weld(self, item_id, on):
        self.welds[item_id] = on

    def object_pose_base(self, item_id):
        return np.eye(4)


def make_pp(motion, scene=None):
    prof = GraspProfile(object_class="cup", T_tcp_obj=np.eye(4), aperture_grasp_rad=0.5,
                        aperture_open_rad=0.0, force_min_n=1.0, force_max_n=20.0,
                        force_exec_max_n=40.0, pieces=["g"])
    return PickPlace(motion, scene or StubScene(), profiles={"cup": prof},
                     goal_sets={}, T_w3_tcp=np.eye(4), seed=0), prof


def phases(motion, kind=None):
    return [p for k, p, _ in motion.log if kind is None or k == kind]


# ---- the "already there" case ------------------------------------------------------------------

def test_returns_skipped_when_already_at_home():
    m = StubMotion(HOME)
    pp, _ = make_pp(m)
    assert pp.return_home() == "skipped"
    assert m.log == [], "an arm already at home must not be moved"


def test_skips_just_inside_tolerance():
    q = HOME.copy()
    q[0] += TOL * 0.9
    m = StubMotion(q)
    pp, _ = make_pp(m)
    assert pp.return_home() == "skipped"


def test_moves_just_outside_tolerance():
    q = HOME.copy()
    q[0] += TOL * 1.1
    m = StubMotion(q)
    pp, _ = make_pp(m)
    assert pp.return_home() == "planned"


# ---- planned vs lerp ----------------------------------------------------------------------------

def test_planned_retreat_when_start_is_collision_free():
    m = StubMotion(np.zeros(6))
    pp, _ = make_pp(m)
    assert pp.return_home() == "planned"
    assert "plan" in [k for k, _, _ in m.log]
    assert np.allclose(m.current_q(), HOME)


def test_lerp_fallback_when_the_plan_fails():
    m = StubMotion(np.zeros(6), plan_solves=False)
    pp, _ = make_pp(m)
    assert pp.return_home() == "lerp"
    n = [d for k, _, d in m.log if k == "execute"][0]
    assert n == 60, "the fallback is a 60-step joint-space interpolation"
    assert np.allclose(m.current_q(), HOME)


def test_lerp_fallback_skips_the_planner_when_the_start_is_in_collision():
    m = StubMotion(np.zeros(6), in_collision=True)
    pp, _ = make_pp(m)
    assert pp.return_home() == "lerp"
    assert "plan" not in [k for k, _, _ in m.log], (
        "planning from a colliding start state is what produces the unexplained "
        "'start tree could not be initialized'"
    )


def test_both_paths_end_at_home():
    for kwargs in ({}, {"plan_solves": False}, {"in_collision": True}):
        m = StubMotion(np.zeros(6), **kwargs)
        pp, _ = make_pp(m)
        pp.return_home()
        assert pp.home_error() < TOL


# ---- the settle hold -----------------------------------------------------------------------------

def test_settle_hold_is_appended_at_home():
    m = StubMotion(np.zeros(6))
    pp, _ = make_pp(m)
    pp.return_home(phase="episode-home")
    holds = [(p, d) for k, p, d in m.log if k == "hold"]
    assert holds == [("episode-home-settle", config.SETTLE_STEPS)]


def test_settle_hold_can_be_disabled():
    m = StubMotion(np.zeros(6))
    pp, _ = make_pp(m)
    pp.return_home(settle_steps=0)
    assert [k for k, _, _ in m.log if k == "hold"] == []


def test_no_settle_hold_when_skipped():
    m = StubMotion(HOME)
    pp, _ = make_pp(m)
    pp.return_home()
    assert m.log == []


def test_phase_label_propagates_to_the_motion():
    m = StubMotion(np.zeros(6))
    pp, _ = make_pp(m)
    pp.return_home(phase="custom")
    assert phases(m, "execute") == ["custom"]
    assert phases(m, "hold") == ["custom-settle"]


# ---- the payload guard ---------------------------------------------------------------------------

def test_payload_is_released_before_the_arm_moves():
    m = StubMotion(np.zeros(6), payload="cup_00")
    scene = StubScene()
    pp, _ = make_pp(m, scene)
    pp.return_home(phase="episode-home")
    kinds = [k for k, _, _ in m.log]
    assert kinds.index("aperture") < kinds.index("execute"), (
        "the jaws must open before the retreat, or an interrupted carry is dragged home"
    )
    assert scene.welds["cup_00"] is False
    assert m.detached is True
    assert m.payload is None


def test_payload_release_opens_to_the_fully_open_aperture():
    m = StubMotion(np.zeros(6), payload="cup_00")
    pp, _ = make_pp(m)
    pp.return_home()
    ap = [d for k, _, d in m.log if k == "aperture"][0]
    assert ap == config.GRIPPER_APERTURE_OPEN_RAD


def test_no_release_phases_when_nothing_is_held():
    m = StubMotion(np.zeros(6))
    pp, _ = make_pp(m)
    pp.return_home()
    assert [k for k, _, _ in m.log if k == "aperture"] == []
    assert m.detached is False


def test_payload_released_even_when_already_at_home():
    """Parking is a no-op but the hand must still be empty afterwards."""
    m = StubMotion(HOME, payload="cup_00")
    pp, _ = make_pp(m)
    assert pp.return_home() == "skipped"
    assert m.payload is None


# ---- the abandon path must not shift ------------------------------------------------------------

def test_abandon_still_uses_the_abandon_home_phase():
    """The recording/metrics vocabulary is load-bearing — renaming it silently reclassifies runs."""
    m = StubMotion(np.zeros(6))
    pp, prof = make_pp(m)
    item = TaskItem(item_id="cup_00", object_class="cup", instance=0, T_base_obj=np.eye(4))
    pp._abandon(item, prof)
    assert "abandon-home" in phases(m, "execute")


def test_abandon_does_not_add_the_long_settle():
    """A full SETTLE_STEPS pause after every failed pick would stretch a bad episode for nothing."""
    m = StubMotion(np.zeros(6))
    pp, prof = make_pp(m)
    item = TaskItem(item_id="cup_00", object_class="cup", instance=0, T_base_obj=np.eye(4))
    pp._abandon(item, prof)
    assert "abandon-home-settle" not in phases(m, "hold")


# ---- home_error ----------------------------------------------------------------------------------

def test_home_error_is_the_max_per_joint_deviation():
    q = HOME.copy()
    q[2] += 0.3
    q[4] -= 0.1
    m = StubMotion(q)
    pp, _ = make_pp(m)
    assert pp.home_error() == pytest.approx(0.3)


def test_home_error_is_zero_at_home():
    pp, _ = make_pp(StubMotion(HOME))
    assert pp.home_error() == pytest.approx(0.0)


def test_home_tolerance_is_configurable_not_hardcoded():
    assert "home_tol_rad" in config.TASK
    assert 0.0 < float(config.TASK["home_tol_rad"]) < 0.5
