"""Convention tests of the UR5e + 2F-85 on the LIVE asset (plan plans/2026-09-29-easy-s0.md, Phase 0.5; D7, D12).

The measurements come from code/frigidaire/scripts/setup/frigidaire_robot_conventions.py (Kit, Isaac Sim 4.5), trimmed
into code/frigidaire/tests/fixtures/robot/conventions.json (inputs only, see its README). These tests assert that the
Kit-free code's conventions (robot/grasp.py, robot/ur5e.py) match what the live asset does, and that every
computed axis is finite and non-degenerate before use.
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from dishsim_frigidaire.robot import grasp as G, ur5e  # noqa: E402

FIXTURE = Path(__file__).resolve().parent / "fixtures" / "robot" / "conventions.json"


@pytest.fixture(scope="module")
def conv():
    return json.loads(FIXTURE.read_text())


def test_measured_values_are_finite(conv):
    tcp = np.asarray(conv["tcp"]["axes_world"], dtype=float)
    assert np.isfinite(tcp).all() and np.allclose(tcp.T @ tcp, np.eye(3), atol=1e-3)     # a rotation
    for state in ("open", "closed"):
        for side in ("left", "right"):
            p = conv["pads"][state][side]
            assert np.isfinite(p["centroid_mm"]).all()
            n = np.asarray(p["face_normal_tcp"], dtype=float)
            assert np.isfinite(n).all() and np.linalg.norm(n) == pytest.approx(1., abs=1e-3)


def test_tcp_frame_is_the_commanded_frame(conv):
    assert np.allclose(conv["tcp"]["axes_world"], conv["tcp"]["commanded_axes"], atol=2e-3)


def test_approach_axis_points_from_the_wrist_to_the_fingertips(conv):
    # grasp.hover_along backs off along -z: +z (the approach) must point away from wrist 3 toward the fingers
    assert conv["tcp"]["z_dot_wrist3_to_tcp"] > .1
    v = np.asarray(conv["tcp"]["wrist3_to_tcp_world"], dtype=float)
    z = np.asarray(conv["tcp"]["axes_world"], dtype=float)[:, 2]
    assert float(v @ z) / np.linalg.norm(v) > .999


def test_jaw_axis_is_tcp_y(conv):
    # grasp.py: "closing along the TCP y axis" -- the left pad sits at -y, the right at +y, open and closed
    for state in ("open", "closed"):
        left = np.asarray(conv["pads"][state]["left"]["centroid_mm"], dtype=float)
        right = np.asarray(conv["pads"][state]["right"]["centroid_mm"], dtype=float)
        d = right - left
        assert d[1] > 0 and abs(d[1]) / np.linalg.norm(d) > .999
    gap_open = conv["pads"]["open"]["right"]["centroid_mm"][1] - conv["pads"]["open"]["left"]["centroid_mm"][1]
    gap_closed = conv["pads"]["closed"]["right"]["centroid_mm"][1] - conv["pads"]["closed"]["left"]["centroid_mm"][1]
    assert gap_closed < gap_open


def test_pad_normals_face_each_other_along_the_jaw_axis(conv):
    for state in ("open", "closed"):
        nl = np.asarray(conv["pads"][state]["left"]["face_normal_tcp"], dtype=float)
        nr = np.asarray(conv["pads"][state]["right"]["face_normal_tcp"], dtype=float)
        assert nl[1] > .99 and nr[1] < -.99


def test_finger_joint_signs_match_the_code_and_the_asset(conv):
    u = conv["unloaded_close"]
    assert u["finger_joint"] == pytest.approx(.8, abs=.01)
    for name, sign in ur5e.INNER_FINGER_SIGNS.items():            # what set_gripper commands
        assert np.sign(u["ratios"][name]) == sign
    for name, expected in u["expected_from_mimic"].items():         # PhysxMimicJointAPI gearings of the payload
        assert u["ratios"][name] == pytest.approx(expected, abs=.02)


def test_mimic_couplings_hold_during_a_loaded_close(conv):
    # D7 (the H2 runtime test): every coupled finger joint follows finger_joint with its declared gearing while the
    # pads squeeze a bowl rim (the rotX mimic instances on Z-axis joints included)
    l, u = conv["loaded_close"], conv["unloaded_close"]
    assert .45 < l["finger_joint"] < .8                             # stopped on the wall
    for name, expected in u["expected_from_mimic"].items():
        assert l["ratios"][name] == pytest.approx(expected, abs=.01)


def test_contact_monitor_matches_the_isaac_lab_sensor(conv):
    # harness.ContactMonitor (contact reports, per collider) vs ContactSensor (per body) on the same close
    monitor, sensor = conv["loaded_close"]["monitor_vs_sensor_total"]
    parts = conv["loaded_close"]["bowl_contact_N_by_robot_part"]
    assert sum(parts.values()) == pytest.approx(sensor, rel=.02)


def test_arm_colliders_make_contacts_on_isaac_45(conv):
    # D12 press test: a plate dropped on each arm rests on its links, instanced (A) and de-instanced (B) alike
    for name in ("A", "B"):
        p = conv["press"][name]
        assert p["verdict"] == "contact" and p["caught_by_arm"] and p["arm_links_touching"]


def test_degenerate_axes_raise_before_use():
    with pytest.raises(ValueError):
        G.tcp_frame(np.zeros(3), [0., 0., 1.])                      # a vertical closing axis has no horizontal part
    with pytest.raises(ValueError):
        G.unit([np.nan, 0., 0.])
    with pytest.raises(ValueError):
        G.unit([1e-9, 0., 0.])
    T = np.eye(4)
    T[0, 0] = np.nan
    with pytest.raises(ValueError):
        G.bowl_grasps(T)
    assert np.allclose(G.unit([0., 3., 4.]), [0., .6, .8])
