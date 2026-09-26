# Copyright (c) 2026, dishsim project.
# SPDX-License-Identifier: BSD-3-Clause
"""Analytic and layout checks for the minimal AO exposure scorer (Kit-free)."""
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "frigidaire" / "src"))
sys.path.insert(0, str(ROOT / "src"))

from dishsim_frigidaire import exposure as E  # noqa: E402

STATES = ROOT / "results/initial_states/frigidaire"


@pytest.mark.parametrize("kind", ["bowl", "mug", "dinner_plate"])
def test_food_contact_faces_are_the_inner_lathe_bands(kind):
    fc = E.food_contact(kind)
    n = E.SECTIONS[kind]
    ring = E.RING[kind]
    assert len(fc.areas) == (n - 2) * 2 * ring + ring   # inner bands + inner apex fan
    centroid = fc.triangles.mean(axis=1)
    radial = np.einsum("ij,ij->i", centroid[:, :2], fc.normals[:, :2])
    if kind == "dinner_plate":
        assert np.all(fc.normals[:, 2] > 0)                 # top face only
    else:
        assert np.all(radial < 1e-7)                        # walls face the axis, floor faces +Z
        assert np.all(fc.normals[np.abs(radial) < 1e-7, 2] > 0)
    assert 0 < fc.area_m2 < 0.1


def test_hemisphere_is_upper_and_unit():
    d = E.hemisphere_directions(64)
    assert np.allclose(np.linalg.norm(d, axis=1), 1) and np.all(d[:, 2] > 0)
    assert np.all(E.lower_directions(64)[:, 2] < 0)


def _square(z, x0=-10., x1=10.):
    p = np.array([[x0, -10, z], [x1, -10, z], [x1, 10, z], [x0, 10, z]], dtype=np.float32)
    return p[[[0, 1, 2], [0, 2, 3]]]


def test_analytic_exposure_from_below_plane_half_plane_and_nothing():
    point, normal = np.zeros((1, 3)), np.array([[0., 0., -1.]])   # downward-facing sample
    assert E.exposure_of(point, normal, np.zeros((0, 3, 3), np.float32))[0] == 1.0
    # plane 10 mm below: every downward ray reaches it within the 2 m cutoff
    assert E.exposure_of(point, normal, _square(-.01))[0] == 0.0
    half = E.exposure_of(point, normal, _square(-.01, 0., 10.))[0]
    assert abs(half - .5) < 4 / 64
    # an upward-facing sample sees nothing from below even with no load around it
    up = np.array([[0., 0., 1.]])
    assert E.exposure_of(point, up, _square(-.01))[0] == 0.0


def test_hemisphere_mode_still_available():
    point, normal = np.zeros((1, 3)), np.array([[0., 0., 1.]])
    assert E.exposure_of(point, normal, _square(.01), source="hemisphere")[0] == 0.0
    assert E.exposure_of(point, normal, np.zeros((0, 3, 3), np.float32), source="hemisphere")[0] == 1.0


def test_sanity_pair_orders_covered_below_alone():
    s = E.sanity_pair(samples=100, directions=32)
    assert s["covered"]["exposure"] < 0.5 * s["alone"]["exposure"]


def test_load_state_reseats_racks_in():
    path = STATES / "organized_20260911_seed20260911/states/random_06.json"
    if not path.exists():
        pytest.skip("settled Frigidaire states not present")
    import json
    data = json.loads(path.read_text())
    arr = E.load_state(path)
    shift = {"LowerRack": .49, "UpperRack": .44}
    for src, obj in zip(data["objects"], arr.objects):
        expected = np.asarray(src["pose_world"]["position_m"]) + [0., shift[src["rack"]], 0.]
        assert np.allclose(obj["position_m"], expected, atol=1e-5)
        assert np.allclose(obj["quaternion_xyzw"], src["pose_world"]["quaternion_xyzw"], atol=1e-6)
    assert abs(arr.basket[0][1] - E.BODY_POSITIONS["LowerRack"][1]) < .2


def test_source_sets_and_rack_rule():
    assert np.all(E.source_directions(64, "above")[:, 2] > 0)
    assert E.source_directions(64, "below+above").shape == (128, 3)
    assert E.source_for("UpperRack") == "middle_arm"
    assert E.source_for("LowerRack") == "lower_arm" and E.source_for("SilverwareBasket") == "lower_arm"
    assert E.source_for("UpperRack", "per-rack-directions") == "below+above"
    assert E.source_for("UpperRack", "below") == "below"


def test_upward_sample_under_plane_above():
    point, up = np.zeros((1, 3)), np.array([[0., 0., 1.]])
    assert E.exposure_of(point, up, _square(.01), source="above")[0] == 0.0
    assert E.exposure_of(point, up, _square(.01), source="below+above")[0] == 0.5   # below half all open


def test_rack_sources_geometry():
    q, u, arm = E.rack_sources("LowerRack")
    assert arm == "lower_arm" and len(q) == 64 and np.allclose(q[:, 2], .185) and abs(u.sum() - 1) < 1e-9
    q, u, arm = E.rack_sources("UpperRack", ceiling_weight=.25)
    assert arm == "middle_arm" and len(q) == 65 and abs(u[-1] - .25) < 1e-9 and abs(u.sum() - 1) < 1e-9
    assert np.allclose(q[-1], E.CEILING_POINT)


def test_source_point_exposure_facing_and_blocking():
    q, u, _ = E.rack_sources("LowerRack")
    p, down, up = np.array([[0., .008, .40]]), np.array([[0., 0., -1.]]), np.array([[0., 0., 1.]])
    empty = np.zeros((0, 3, 3), np.float32)
    assert E.exposure_of(p, down, empty, sources=(q, u))[0] == 1.0      # faces the disc, nothing in the way
    assert E.exposure_of(p, up, empty, sources=(q, u))[0] == 0.0        # faces away from every source
    assert E.exposure_of(p, down, _square(.30), sources=(q, u))[0] == 0.0   # plane between sample and disc
    assert E.exposure_of(p, down, _square(.10), sources=(q, u))[0] == 1.0   # plane beyond the disc: no effect


def test_cast_returns_hit_distance():
    hit, t = E.cast(_square(-.05), np.zeros((1, 3)), np.array([[0., 0., -1.]]), max_t=np.array([1.]))
    assert hit[0] and abs(t[0] - .05) < 1e-5


def _about_x(deg):
    a = np.radians(deg) / 2
    return (float(np.sin(a)), 0., 0., float(np.cos(a)))


def test_pooling_is_a_draining_test():
    z = [0., 0., .3]
    assert E.pools("bowl", z, (0., 0., 0., 1.))          # mouth up
    assert not E.pools("bowl", z, (1., 0., 0., 0.))      # mouth down
    assert not E.pools("bowl", z, _about_x(100))         # mouth 80 deg from down: policy flags it, water drains
    assert not E.pools("bowl", z, _about_x(80))          # mouth 100 deg from down: this shallow bowl traps ~1 mm only
    assert E.pools("bowl", z, _about_x(45))              # mouth 135 deg from down: holds ~12 mm
    assert not E.pools("mug", z, _about_x(90))           # mug on its side drains
    assert E.pools("mug", z, _about_x(60))               # mug tilted mouth-up holds water
    assert E.pools("dinner_plate", z, (0., 0., 0., 1.))       # flat plate holds water in its well
    assert not E.pools("dinner_plate", z, _about_x(90))     # on edge: drains
    assert not E.pools("dinner_plate", z, _about_x(94))     # the lower-rack lean
    for kind in ("fork", "knife", "tablespoon"):
        assert not E.pools(kind, z, (0., 0., 0., 1.))        # cutlery never pools


def test_feasibility_follows_mouth_up():
    packing = STATES / "packing_20260911_seed20260911/states/random_06.json"
    organized = STATES / "organized_20260911_seed20260911/states/random_06.json"
    if not (packing.exists() and organized.exists()):
        pytest.skip("settled Frigidaire states not present")
    a = E.score_state(packing, samples=50, directions=16)
    b = E.score_state(organized, samples=50, directions=16)
    assert a["feasible"] is False and len(a["violations"]) == a["pooling_count"] > 0
    assert b["feasible"] is True and b["violations"] == []
    assert {o["ray_source"] for o in b["objects"] if o["rack"] == "UpperRack"} == {"middle_arm"}
    assert {o["ray_source"] for o in b["objects"] if o["rack"] == "LowerRack"} == {"lower_arm"}


@pytest.mark.parametrize("kind", ["tablespoon", "teaspoon"])
def test_spoon_bowl_is_a_conforming_lathe(kind):
    fc = E.food_contact(kind)
    assert E.RING[kind] == 80 and E.SECTIONS[kind] == 4
    assert len(fc.areas) == 2 * 2 * 80 + 80 and fc.rim.shape == (80, 3)
    assert np.all(fc.normals[:, 1] > 0)                       # the bowl opens local +Y; interior faces into the cavity
    assert 0 < fc.area_m2 < .01


def test_fork_food_contact_is_the_tines():
    fc = E.food_contact("fork")
    assert len(fc.areas) == 4 * 92 and fc.area_m2 > 0 and fc.rim.shape == (0, 3)
    assert fc.triangles[:, :, 2].min() >= .0635 - 1e-9        # handle, shaft and neck excluded
    assert (fc.normals[:, 1] > .5).any() and (fc.normals[:, 1] < -.5).any()   # both flat faces count


def test_knife_food_contact_is_the_blade():
    fc = E.food_contact("knife")
    assert 0 < len(fc.areas) < len(E.dish_visuals("knife")[0]) and len(fc.areas) == 106
    assert fc.triangles[:, :, 2].min() >= E.KNIFE_BOLSTER_Z - 1e-9   # handle excluded
    assert (fc.normals[:, 1] > .5).any() and (fc.normals[:, 1] < -.5).any()


def test_cutlery_exposure_alone_follows_the_head():
    p, src = np.array([.2, .1, .33]), E.rack_sources("SilverwareBasket")[:2]
    assert E.isolated_baseline("fork", p, (0., 0., 0., 1.), sources=src) > .3     # tines up, both faces see the disc
    assert E.isolated_baseline("knife", p, (0., 0., 0., 1.), sources=src) > .5
    assert E.isolated_baseline("tablespoon", p, _about_x(-90), sources=src) > .7  # bowl faces the arm (its rim shades some)
    assert E.isolated_baseline("tablespoon", p, _about_x(90), sources=src) == 0.  # bowl faces the ceiling


def test_baselines_opt_in():
    bowl = {"id": "b", "kind": "bowl", "rack": "LowerRack", "position_m": np.array([0., .008, .30]),
            "quaternion_xyzw": (1., 0., 0., 0.)}
    arr = E.Arrangement("one", "", "", [bowl], (np.zeros(3), E.IDENTITY))
    a = E.score_arrangement(arr, samples=50, directions=16, appliance=False)
    b = E.score_arrangement(arr, samples=50, directions=16, appliance=False, baselines=False)
    assert a["score"] == b["score"] and "baseline" in a["objects"][0] and "baseline" not in b["objects"][0]
    assert a["parameters"]["baselines"] and not b["parameters"]["baselines"]


def test_ring_per_kind_and_default_sources():
    assert E.RING["bowl"] == 96 and E.RING["tablespoon"] == 80 and E.food_contact("bowl").rim.shape == (96, 3)
    q, u, arm = E.rack_sources("UpperRack")
    assert arm == "middle_arm" and len(q) == 64 and abs(u.sum() - 1) < 1e-9     # no ceiling point by default
    assert E.DEFAULTS["ceiling_weight"] == 0 and E.DEFAULTS["schema_version"] == 5
