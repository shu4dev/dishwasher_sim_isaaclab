# Copyright (c) 2026, dishsim project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Rack manipulation as an episode prologue.

An episode starting from ``both_in`` has nowhere to place anything until the rack comes out, so
the properties that matter are the ones that keep a half-done rack action from poisoning the rest
of the episode: the drive override must always be released, a rack that settles off target must
fail loudly rather than let later picks plan against a world that no longer matches the machine,
and a failed action must stop the sequence instead of continuing to the next one.

The geometry (:mod:`dishsim.rack_ops`) is stubbed out — it is exercised for real by the
single-object runner. What is under test here is the choreography and the gating.

Run with:
    PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /workspace/isaaclab/env_isaaclab/bin/python -m pytest \
        tests/test_task_rack.py -v
"""

import numpy as np
import pytest

from dishsim import config
from dishsim.task import rack as R

from conftest import StubPlanResult, StubStats

ACTION = config.SCENARIOS["both_in"]["rack_action"]


class StubWorld:
    def __init__(self, in_coll=False):
        self._c = in_coll
        self.manifest = {
            "t_wrist3_tcp": np.eye(4).tolist(),
            "statics_state": {"rack_lower_m": 0.0, "rack_upper_m": 0.0},
            "statics": {b: {"T_base_body": np.eye(4).tolist()}
                        for b in (ACTION["body"], config.SCENARIOS["both_out"]["rack_action"]["body"])},
        }

    def in_collision(self, q, return_pairs=False):
        return (self._c, []) if return_pairs else self._c


class StubMotion:
    """The slice of MotionService a rack action uses, with a phase log."""

    def __init__(self, *, plan_solves=True, fail_phase=None):
        self.world = StubWorld()
        self.stats = StubStats()
        self._plan_solves = plan_solves
        self._fail_phase = fail_phase
        self.log = []            # (kind, phase)
        self.rack_targets = []   # every set_rack_target call, in order
        self._q = np.array(config.HOME_Q, dtype=float)

    def current_q(self):
        return self._q.copy()

    def tcp_pose(self, q=None):
        return np.eye(4)

    def plan(self, start_q, goal_qs, *, seed=None, debug=None):
        if not self._plan_solves:
            return StubPlanResult("failed")
        return StubPlanResult("solved", np.vstack([np.asarray(start_q, float), np.asarray(goal_qs)[0]]))

    def _result(self, phase, q_end):
        from dishsim.task.motion import ExecResult
        if phase == self._fail_phase:
            return ExecResult(False, f"{phase}: stub fault", 9.9, 1, q_end)
        return ExecResult(True, None, 0.0, 1, q_end)

    def execute(self, path_q, *, phase, aperture=None, exclude=(), on_step=None, gate=None,
                on_before_step=None):
        self.log.append(("execute", phase))
        path = np.atleast_2d(np.asarray(path_q, dtype=float))
        if on_before_step is not None:          # drive the rack exactly as the real loop would
            for i in range(len(path)):
                on_before_step(i / max(len(path) - 1, 1))
        self._q = path[-1].copy()
        return self._result(phase, self._q)

    def servo_line(self, T_from, T_to, n, q_seed, *, phase, aperture=None, exclude=(),
                   on_step=None):
        self.log.append(("servo", phase))
        return self._result(phase, np.asarray(q_seed, dtype=float))

    def set_aperture(self, aperture_rad, steps, *, phase, arm_q=None, on_step=None):
        self.log.append(("aperture", phase))
        return aperture_rad

    def hold(self, steps, *, phase, arm_q=None, aperture=None, on_step=None):
        self.log.append(("hold", phase))
        return self._result(phase, self._q)

    def set_rack_target(self, targets):
        self.rack_targets.append(targets)


@pytest.fixture
def geom(monkeypatch):
    """Canned rack geometry — the real thing needs a collision cache."""
    monkeypatch.setattr(R.rack_ops, "t_wrist3_tcp", lambda: np.eye(4))
    monkeypatch.setattr(R.rack_ops, "approach_dir", lambda T: np.array([0.0, 1.0, 0.0]))
    monkeypatch.setattr(R.rack_ops, "engage_pose",
                        lambda T, p, a, e, flip=False: np.eye(4))
    monkeypatch.setattr(R.rack_ops, "slide_arm_path",
                        lambda *a, **k: np.tile(np.array(config.HOME_Q, dtype=float), (21, 1)))
    monkeypatch.setattr(R, "ik_wrist3_all",
                        lambda T, q_seed=None: [np.array(config.HOME_Q, dtype=float)])


def make(motion, ext=-0.20):
    spec = R.RackSpec.from_action(ACTION, motion.world)
    act = R.RackAction(motion, gripper_bodies=("left_inner_finger",), measure_ext=lambda: ext)
    return spec, act


def phases(m, kind=None):
    return [p for k, p in m.log if kind is None or k == kind]


# ---- the happy path ------------------------------------------------------------------------------

def test_pull_runs_the_full_choreography_in_order(geom):
    m = StubMotion()
    spec, act = make(m)
    out = act.run(spec)
    assert out.ok, out.detail
    order = [p for _, p in m.log]
    for a, b in zip(["rack-approach", "rack-engage", "rack-grip-close", "rack-slide",
                     "rack-settle", "rack-release", "rack-disengage"], ["x"] * 7):
        assert a in order, f"missing phase {a}"
    assert order.index("rack-approach") < order.index("rack-engage") < order.index("rack-slide")
    assert order.index("rack-slide") < order.index("rack-disengage")


def test_spec_reads_the_cached_extension_from_the_manifest(geom):
    m = StubMotion()
    spec, _ = make(m)
    assert spec.cached_ext == 0.0 and spec.to == -0.20
    assert spec.joint == "PrismaticJoint_dishwasher_2_down"


def test_slide_ramps_the_rack_target_from_stowed_to_target(geom):
    m = StubMotion()
    spec, act = make(m)
    act.run(spec)
    ramp = [t[spec.joint] for t in m.rack_targets if t is not None]
    assert ramp[0] == pytest.approx(0.0)
    assert ramp[-1] == pytest.approx(-0.20)
    assert all(b <= a + 1e-12 for a, b in zip(ramp, ramp[1:])), "the ramp must be monotone"


def test_achieved_extension_is_measured_not_assumed(geom):
    m = StubMotion()
    spec, act = make(m, ext=-0.1985)
    out = act.run(spec)
    assert out.achieved_m == pytest.approx(-0.1985)
    assert out.error_m == pytest.approx(0.0015)


# ---- the gating that protects the rest of the episode --------------------------------------------

def test_settle_beyond_tolerance_fails(geom):
    """Later picks plan against a world posed at the TARGET; a rack that stopped short is a lie."""
    m = StubMotion()
    spec, act = make(m, ext=-0.20 + config.RACK_SLIDE_TOL_M * 2)
    out = act.run(spec)
    assert not out.ok and out.stage == "rack-slide-fault"
    assert "off target" in out.detail


def test_settle_just_inside_tolerance_passes(geom):
    m = StubMotion()
    spec, act = make(m, ext=-0.20 + config.RACK_SLIDE_TOL_M * 0.5)
    assert act.run(spec).ok


def test_override_stays_latched_after_a_successful_pull(geom):
    """The scene's standing targets come from the PRE-action scenario (racks stowed), so
    releasing the override would drive the rack straight back in behind the arm."""
    m = StubMotion()
    spec, act = make(m)
    act.run(spec)
    assert m.rack_targets[-1] == {spec.joint: pytest.approx(-0.20)}


def test_latched_targets_accumulate_across_a_sequence(geom):
    """A second action that REPLACED the override would let the first rack slide shut."""
    m = StubMotion()
    lower = R.RackSpec.from_action(ACTION, m.world)
    upper = R.RackSpec.from_action(
        {**config.SCENARIOS["both_out"]["rack_action"], "to": -0.20}, m.world)
    act = R.RackAction(m, measure_ext=lambda: -0.20)
    ph = R.run_sequence([lower, upper], act)
    assert ph.ok
    assert m.rack_targets[-1] == {lower.joint: pytest.approx(-0.20),
                                  upper.joint: pytest.approx(-0.20)}


@pytest.mark.parametrize("fail_at", ["rack-approach", "rack-engage", "rack-slide"])
def test_a_failed_action_leaves_no_partial_target_commanded(geom, fail_at):
    m = StubMotion(fail_phase=fail_at)
    spec, act = make(m)
    out = act.run(spec)
    assert not out.ok
    assert m.rack_targets[-1] is None, "nothing was latched, so nothing may stay commanded"


def test_a_failed_action_rolls_back_to_what_earlier_actions_latched(geom):
    m = StubMotion()
    lower = R.RackSpec.from_action(ACTION, m.world)
    act = R.RackAction(m, measure_ext=lambda: -0.20)
    act.run(lower)                       # succeeds and latches
    m._fail_phase = "rack-slide"         # the next one fails mid-slide
    upper = R.RackSpec.from_action(
        {**config.SCENARIOS["both_out"]["rack_action"], "to": -0.20}, m.world)
    assert not act.run(upper).ok
    assert m.rack_targets[-1] == {lower.joint: pytest.approx(-0.20)}


def test_override_not_latched_when_planning_fails(geom):
    m = StubMotion(plan_solves=False)
    spec, act = make(m)
    out = act.run(spec)
    assert not out.ok and out.stage == "rack-approach-plan"
    assert m.rack_targets == [] or m.rack_targets[-1] is None


# ---- failure taxonomy ----------------------------------------------------------------------------

def test_no_slide_path_reports_slide_plan(geom, monkeypatch):
    monkeypatch.setattr(R.rack_ops, "slide_arm_path", lambda *a, **k: None)
    m = StubMotion()
    spec, act = make(m)
    out = act.run(spec)
    assert out.stage == "rack-slide-plan"
    assert m.log == [], "nothing may move when the slide was never validated"


def test_no_reachable_pre_engage_reports_slide_plan(geom, monkeypatch):
    monkeypatch.setattr(R, "ik_wrist3_all", lambda T, q_seed=None: [])
    m = StubMotion()
    spec, act = make(m)
    assert act.run(spec).stage == "rack-slide-plan"


def test_approach_contact_reports_approach_plan(geom):
    m = StubMotion(fail_phase="rack-approach")
    spec, act = make(m)
    out = act.run(spec)
    assert out.stage == "rack-approach-plan" and "stub fault" in out.detail


def test_engage_contact_reports_slide_fault(geom):
    m = StubMotion(fail_phase="rack-engage")
    spec, act = make(m)
    assert act.run(spec).stage == "rack-slide-fault"


def test_disengage_failure_is_not_fatal(geom):
    """The rack is already placed; a blocked back-off is the return-home's problem."""
    m = StubMotion(fail_phase="rack-disengage")
    spec, act = make(m)
    assert act.run(spec).ok


# ---- push mode -----------------------------------------------------------------------------------

def test_push_mode_never_closes_or_releases_the_grip(geom):
    """A push is a closed-fist press — ramping the pinch would grab at nothing."""
    m = StubMotion()
    spec = R.RackSpec.from_action(config.SCENARIOS["both_out"]["rack_action"], m.world)
    out = R.RackAction(m, measure_ext=lambda: 0.0).run(spec)
    assert out.ok
    assert "rack-grip-close" not in phases(m)
    assert "rack-release" not in phases(m)


# ---- sequences -----------------------------------------------------------------------------------

def test_sequence_runs_every_action(geom):
    m = StubMotion()
    spec, act = make(m)
    ph = R.run_sequence([spec, spec], act)
    assert ph.ok and len(ph.outcomes) == 2
    assert phases(m).count("rack-slide") == 2


def test_sequence_stops_at_the_first_failure(geom):
    m = StubMotion(fail_phase="rack-slide")
    spec, act = make(m)
    ph = R.run_sequence([spec, spec], act)
    assert not ph.ok and len(ph.outcomes) == 1, "a failed pull must not be followed by another"


def test_rack_phase_serialises(geom):
    m = StubMotion()
    spec, act = make(m)
    doc = R.run_sequence([spec], act).to_json()
    assert doc[0]["joint"] == spec.joint
    assert doc[0]["target_m"] == pytest.approx(-0.20)
    assert doc[0]["ok"] is True


# ---- scripted transitions ------------------------------------------------------------------------
#
# The full-load episode's phase boundaries: drive ramps with the arm parked. The stub motion
# above already records every set_rack_target call; measured extensions are simulated as
# perfect drives (last commanded value) unless a test injects an error.

def _joints():
    return sorted(config.RACK_JOINT_TARGETS)


def _perfect_exts(m, initial):
    """measure_exts stub: drives are perfect — report the last commanded target."""
    def measure():
        return dict(m.rack_targets[-1]) if m.rack_targets else dict(initial)
    return measure


def _transition(m, initial, **kw):
    return R.ScriptedTransition(m, measure_exts=_perfect_exts(m, initial),
                                steps_per_meter=10, settle_steps=2, **kw)


def test_transition_commands_complete_dicts_every_step():
    down, up = _joints()[0], _joints()[-1]
    m = StubMotion()
    initial = {down: -0.20, up: 0.0}
    tr = _transition(m, initial)
    out = tr.run([{down: 0.0, up: 0.0}, {down: 0.0, up: -0.20}],
                 from_state="placement", to_state="x")
    assert out.ok
    assert m.rack_targets, "the ramp must command targets"
    for cmd in m.rack_targets:
        assert set(cmd) == set(_joints()), "every commanded dict must be complete"
    # the FINAL commanded dict is the destination state and stays commanded (never None)
    assert m.rack_targets[-1] == {down: 0.0, up: -0.20}
    assert None not in m.rack_targets


def test_transition_rejects_incomplete_dicts():
    down = _joints()[0]
    m = StubMotion()
    tr = _transition(m, {down: -0.20})
    with pytest.raises(ValueError, match="complete"):
        tr.run([{down: 0.0}], from_state="a", to_state="b")


def test_transition_ramp_is_monotone_per_joint():
    down, up = _joints()[0], _joints()[-1]
    m = StubMotion()
    tr = _transition(m, {down: -0.20, up: 0.0})
    tr.run([{down: 0.0, up: 0.0}], from_state="a", to_state="b")
    downs = [cmd[down] for cmd in m.rack_targets]
    assert downs == sorted(downs), "stowing ramp must be monotone"
    assert downs[-1] == 0.0


def test_transition_slide_fault_on_settle_error():
    down, up = _joints()[0], _joints()[-1]
    m = StubMotion()

    def bad_exts():
        # the drive stalls 20 mm short of whatever was last commanded on the down joint
        cur = dict(m.rack_targets[-1]) if m.rack_targets else {down: -0.20, up: 0.0}
        cur[down] = cur[down] - 0.020
        return cur

    tr = R.ScriptedTransition(m, measure_exts=bad_exts, steps_per_meter=10, settle_steps=2)
    out = tr.run([{down: 0.0, up: 0.0}], from_state="a", to_state="b")
    assert not out.ok and out.stage == "transition-slide-fault"
    assert out.per_joint[down]["error_m"] == pytest.approx(0.020)
    # the last commanded dict stays commanded even on failure
    assert m.rack_targets[-1] == {down: 0.0, up: 0.0}


def test_transition_rider_slip_subtracts_the_rack_ride():
    down, up = _joints()[0], _joints()[-1]
    m = StubMotion()
    T0 = np.eye(4)
    T0[:3, 3] = (0.5, 0.1, 0.2)
    T1 = np.eye(4)
    T1[:3, 3] = (0.5, 0.1 + 0.56, 0.2)      # the item moved exactly with its rack
    poses = {"n": 0}
    body = {"n": 0}

    def measure_items():
        poses["n"] += 1
        return {"cup_00": T0 if poses["n"] == 1 else T1}

    def measure_body(name):
        body["n"] += 1
        return np.array([0.0, 0.0, 0.0]) if body["n"] == 1 else np.array([0.0, 0.56, 0.0])

    tr = R.ScriptedTransition(m, measure_exts=_perfect_exts(m, {down: -0.20, up: 0.0}),
                              measure_items=measure_items, measure_body_pos=measure_body,
                              steps_per_meter=10, settle_steps=2)
    out = tr.run([{down: 0.0, up: 0.0}], from_state="a", to_state="b",
                 riders={"cup_00": "E_shelf_1_04"},
                 slip_gates={"cup_00": (0.010, 10.0)})
    assert out.ok
    assert out.item_slips["cup_00"]["slip_mm"] == pytest.approx(0.0, abs=1e-6)


def test_transition_rider_displaced_fails():
    down, up = _joints()[0], _joints()[-1]
    m = StubMotion()
    T0 = np.eye(4)
    T1 = np.eye(4)
    T1[:3, 3] = (0.05, 0.0, 0.0)            # 50 mm of real slip, no rack ride
    seen = {"n": 0}

    def measure_items():
        seen["n"] += 1
        return {"cup_00": T0 if seen["n"] == 1 else T1}

    tr = R.ScriptedTransition(m, measure_exts=_perfect_exts(m, {down: -0.20, up: 0.0}),
                              measure_items=measure_items,
                              measure_body_pos=lambda b: np.zeros(3),
                              steps_per_meter=10, settle_steps=2)
    out = tr.run([{down: 0.0, up: 0.0}], from_state="a", to_state="b",
                 riders={"cup_00": "E_shelf_1_04"},
                 slip_gates={"cup_00": (0.010, 10.0)})
    assert not out.ok and out.stage == "transition-rider-displaced"
    assert not out.item_slips["cup_00"]["ok"]
    assert out.item_slips["cup_00"]["slip_mm"] == pytest.approx(50.0)


def test_transition_outcome_serialises():
    down, up = _joints()[0], _joints()[-1]
    m = StubMotion()
    tr = _transition(m, {down: -0.20, up: 0.0})
    doc = tr.run([{down: 0.0, up: 0.0}], from_state="placement",
                 to_state="middle_out").to_json()
    assert doc["ok"] is True and doc["from_state"] == "placement"
    assert doc["per_joint"][down]["target_m"] == 0.0
