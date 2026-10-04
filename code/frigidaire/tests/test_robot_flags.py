"""Kit-free tests of the robot run flags (plan plans/2026-09-29-easy-s0.md, Section 3 and decisions D1-D19).

The legacy profile must freeze the literal values the 2026-09-29 upright3 PASS ran with (the regression test's
"old settings"), and the headline profile must carry no benchmark relaxation.
"""
from __future__ import annotations

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from dishsim_frigidaire.robot import flags as F, rig  # noqa: E402


def test_headline_has_no_benchmark_relaxation():
    h = F.resolve("headline")
    assert F.is_headline(h) and not F.deviations(h)
    assert (h.test_case, h.place_mode, h.judge, h.disturbance, h.end_check, h.move_settle) == \
           (None, "goal_pose", "at_goal", "benchmark", "benchmark", "benchmark")
    assert h.reextend_before_scoring and h.counter_cap and not h.continue_after_failed_dish
    assert h.invariants_abort and h.final_hold_observation_s == 2.
    # D8: pre-R7 limits = the rig defaults, never the widened call-site values
    assert (h.lag_max_rad, h.settled_lag_rad) == (rig.LAG_MAX_RAD, rig.SETTLED_LAG_RAD) == (.15, .03)
    assert all(getattr(h, f) is None for f in ("tol_approach", "tol_joint_move_held", "tol_rrt", "tol_rise", "tol_lift", "tol_reaim"))
    # D9: the drift gate
    assert (h.hold_gate, h.hold_window_s, h.hold_drift_max_m) == ("drift", 2., .005)


def test_legacy_profile_freezes_the_upright3_pass_values():
    lg = F.resolve("legacy_upright3")
    assert not F.is_headline(lg)
    assert (lg.test_case, lg.place_mode, lg.judge, lg.disturbance, lg.end_check) == \
           ("upright3", "lower_until_contact", "racked_in", "any_neighbour", "racked_in")
    # R7 call sites of the PASS run: approach .06, held joint move .08, RRT .08, rise .08, lift .08, re-aim .08, detector .05
    assert (lg.tol_approach, lg.tol_joint_move_held, lg.tol_rrt, lg.tol_rise, lg.tol_lift, lg.tol_reaim) == \
           (.06, .08, .08, .08, .08, .08)
    assert lg.place_down_contact_lag == .05
    # R1: rose > 5 cm and within 20 cm of the TCP after 90 ticks
    assert (lg.hold_gate, lg.legacy_rise_min_m, lg.legacy_near_tcp_m, lg.in_hand_settle_ticks) == ("legacy_rise", .05, .20, 90)
    # R2 and the geometry of the PASS run
    assert (lg.gripper_effort, lg.gripper_stiffness) == (8., 400.)
    assert lg.jaw_wall_rad == (.45, .795)
    assert (lg.z_safe_m, lg.hover_m, lg.release_above_m) == (1.25, .10, .006)
    assert lg.mount == "data/results/robot/mount/easy_s0_any.json"          # D17
    assert not lg.invariants_abort and not lg.diag_settle_active        # record-only harness, old timeline


def test_resolve_validates():
    with pytest.raises(ValueError):
        F.resolve("nope")
    with pytest.raises(ValueError):
        F.resolve("headline", not_a_flag=1)
    with pytest.raises(ValueError):
        F.resolve("headline", judge="whatever")
    d = F.resolve("headline", test_case="upright3")
    assert [row[0] for row in F.deviations(d)] == ["test_case"] and not F.is_headline(d)


def test_one_hold_rule_per_profile():
    h, lg = F.resolve("headline"), F.resolve("legacy_upright3")
    assert F.hold_verdict(h, in_hand=True, window_s=2., window_drift_m=.004)[0]
    assert not F.hold_verdict(h, in_hand=True, window_s=2., window_drift_m=.006)[0]
    assert not F.hold_verdict(h, in_hand=False, window_s=2., window_drift_m=0.)[0]      # a dish left on the counter
    assert not F.hold_verdict(h, in_hand=True, window_s=1., window_drift_m=0.)[0]       # window too short
    assert F.hold_verdict(lg, rise_m=.06, dist_to_tcp_m=.1)[0]
    assert not F.hold_verdict(lg, rise_m=.04, dist_to_tcp_m=.1)[0]
    assert not F.hold_verdict(lg, rise_m=.06, dist_to_tcp_m=.25)[0]
