"""Kit-free tests of the robot harness's pure rules (plan plans/2026-09-29-easy-s0.md, Phase 0.3-0.4).

The Kit-side parts (contact reports, sleeping, invariants on a live articulation) are exercised by the episode runs and
the conventions script; here the pure pieces: collider classification, the touching rule, the trial-log schema.
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from dishsim_frigidaire.robot import harness as H, triallog as TL  # noqa: E402

G = "/World/Robot/Gripper/Robotiq_2F_85/"


def test_classify_robot_parts_and_scene_bodies():
    assert H.classify(G + "left_inner_finger/visuals/Defeatured_2F_85_PAD_OPEN_fingertipsstep_01/Defeatured_2F_85_PAD_OPEN_fingertipsstep") == ("pad_L", "pad")
    assert H.classify(G + "right_inner_finger/visuals/Defeatured_2F_85_PAD_OPEN_finger4step_01/Defeatured_2F_85_PAD_OPEN_finger4step") == ("right_inner_finger", "finger")
    assert H.classify(G + "left_inner_knuckle/visuals/x/y") == ("left_inner_knuckle", "finger")
    assert H.classify(G + "base_link/visuals/x/y") == ("palm", "palm")
    assert H.classify("/World/Robot/wrist_3_link/collisions/wrist3/mesh") == ("wrist_3_link", "arm")
    assert H.classify("/World/Robot/base_link") == ("base_link", "arm")
    assert H.classify("/World/InitialStateDishes/bowl_03/Collisions/Shell_007") == ("bowl_03", "dish")
    assert H.classify("/World/InitialStateDishwasher/LowerRack/geom") == ("LowerRack", "rack")
    assert H.classify("/World/InitialStateDishwasher/SilverwareBasket/geom") == ("SilverwareBasket", "basket")
    assert H.classify("/World/InitialStateDishwasher/Cabinet/x") == ("Cabinet", "appliance")
    assert H.classify("/World/Counter/geometry/mesh") == ("Counter", "counter")
    assert H.classify("/World/Pedestal") == ("Pedestal", "pedestal")
    assert H.classify("/World/Ground/geometry") == ("Ground", "ground")
    # a second robot root gets a prefix (the conventions script runs two robots)
    assert H.classify("/World/RobotB/Gripper/Robotiq_2F_85/right_inner_finger/visuals/a_fingertipsstep_01/a_fingertipsstep",
                      roots=("/World/RobotA", "/World/RobotB")) == ("RobotB:pad_R", "pad")


def test_touching_rule_excludes_contact_offset_proximity():
    assert not H.is_touch(0., .0018)            # inside the contact offset, no impulse: the legacy log's 0 N "contacts"
    assert H.is_touch(21., -.00017)             # the knuckle load
    assert H.is_touch(0., -1e-5)                # penetrating without impulse this step: still a touch
    assert H.is_touch(2.4, .0005)               # impulse with a positive separation (contact offset): a touch
    assert not H.is_touch(0., float("inf"))


def test_carried_allowed_list_is_the_plan_default_plus_the_basket_interpretation():
    assert set(H.CARRIED_ALLOWED) == {"pad", "counter", "rack", "basket"}
    assert "finger" not in H.CARRIED_ALLOWED and "palm" not in H.CARRIED_ALLOWED and "arm" not in H.CARRIED_ALLOWED
    assert H.PAD_FORCE_MAX_N == 235.
    assert set(H.INVARIANTS) == {"arm_or_palm_contact", "carried_contact", "teleport_after_reset", "sleeping_body",
                                 "nan_frame", "joint_limit", "pad_force"}


def test_trial_log_rows_carry_sim_and_wall_time_separately(tmp_path):
    ticks = {"n": 0}
    log = TL.TrialLog(tmp_path / "trial.jsonl", run_id="r", trial_id="t00", config="abc", profile="headline",
                      clock=lambda: (ticks["n"], ticks["n"] / 120.))
    log.set(move=1, dish="bowl_01", attempt=0, phase="pick")
    ticks["n"] = 240
    line = log.write("note", value=float("nan"), arr=[1, 2])
    log.close()
    rows = TL.read(tmp_path / "trial.jsonl")
    assert line == 1 and len(rows) == 1
    n, row = rows[0]
    assert (row["tick"], row["sim_s"]) == (240, 2.0) and row["wall_s"] >= 0.
    assert (row["move"], row["dish"], row["attempt"], row["phase"]) == (1, "bowl_01", 0, "pick")
    assert row["payload"]["value"] == "NaN" and row["payload"]["arr"] == [1, 2]   # non-finite floats never crash a writer
    assert "sim_s" in row and "wall_s" in row and row["config_id"] == "abc"


def test_config_id_separates_configurations_not_attempts():
    a = {"bowl_01": {"position_m": [0., 0., .914], "quaternion_xyzw": [0., 0., 0., 1.]}}
    b = {"bowl_01": {"position_m": [.005, 0., .914], "quaternion_xyzw": [0., 0., 0., 1.]}}   # a +-5 mm perturbation
    assert TL.config_id(a, ["bowl_01"]) == TL.config_id(json.loads(json.dumps(a)), ["bowl_01"])
    assert TL.config_id(a, ["bowl_01"]) != TL.config_id(b, ["bowl_01"])
