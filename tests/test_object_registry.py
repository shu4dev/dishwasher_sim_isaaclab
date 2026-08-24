# Copyright (c) 2026, dishsim project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Kit-free sanity checks on the multi-object registry (config.OBJECTS).

Covers: per-spec geometry/mass sanity, jaw clearance at the grasp patch, the grasp-chain
math for every family (the grasped feature must land on the tool axis at rim_tcp_z), the
mug entry reproducing the frozen v0 constants bit-exactly (baseline cache protection),
set_active_object mutate/restore, per-object cache-dir separation, and the gripper-wide
grasp constants (force band, open aperture, pad bodies, inner-finger signs).
"""

import numpy as np
import pytest

from dishsim import config
from dishsim.geometry import config_hash
from dishsim.transforms import make_T

JAW_MAX_M = 0.085


@pytest.fixture(autouse=True)
def _restore_active_object():
    yield
    config.set_active_object("mug")
    config.apply_scenario("both_out")


# ---------------------------------------------------------------------------------------------
# 1. every spec is sane
# ---------------------------------------------------------------------------------------------


@pytest.mark.parametrize("name", sorted(config.OBJECTS))
def test_spec_sanity(name):
    spec = config.OBJECTS[name]
    assert spec.name == name
    assert spec.source in ("isaac_ycb", "ycb16k", "procedural")
    assert spec.scale > 0.0 and spec.mass_kg > 0.0
    assert all(h > 0.0 for h in spec.bbox_half)
    assert spec.height_m > 0.0 and spec.rim_radius_m > 0.0
    assert tuple(spec.axis_obj) in ((0.0, 1.0, 0.0), (0.0, 0.0, 1.0), (1.0, 0.0, 0.0))
    assert spec.placement.mode in (
        "floor_stand", "plate_slot", "basket_drop",
        "upside_down", "stem_scallop", "flat_lay",
    )
    assert spec.placement.rack in ("lower", "upper", "basket")
    if spec.robot_demo:
        assert len(spec.countertop_poses_w) >= 1, "demo classes need a staging pose"
        assert spec.grasp.family != "stem_pinch", "stem_pinch is registry-only"


@pytest.mark.parametrize("name", sorted(config.OBJECTS))
def test_jaw_clearance_at_grasp(name):
    spec = config.OBJECTS[name]
    # The object width between the pads must fit the 85 mm jaw with clearance. Floor = 3.5 mm:
    # the tightest measured-working pinch is the scan mug's 81.2 mm wall (calibrated
    # 2026-08-10 — zero contact at full open, clean hold and release at 3.8 mm clearance).
    assert spec.grasp.grasp_width_m + 0.0035 < JAW_MAX_M, name


@pytest.mark.parametrize("name", sorted(config.OBJECTS))
def test_calibrated_aperture_bounds(name):
    ap = config.OBJECTS[name].grasp.aperture_rad
    if ap is not None:
        assert 0.0 < ap < 0.82


# ---------------------------------------------------------------------------------------------
# 2. grasp-chain math per family: the grasped feature lands on the tool axis at rim_tcp_z
# ---------------------------------------------------------------------------------------------


def _grasped_point_obj(spec: config.ObjectSpec) -> np.ndarray:
    u, v = spec.body_center_uv
    fam = spec.grasp.family
    if tuple(spec.axis_obj) == (0.0, 1.0, 0.0):  # legacy Y-up mug
        return np.array([u, spec.bbox_half[1], v])
    h2 = spec.bbox_half[2]
    if fam in ("rim_diam", "stem_pinch"):
        return np.array([u, v, h2])
    if fam == "rim_edge":
        return np.array([u, v + spec.rim_radius_m, h2])
    if fam == "edge_pinch":
        return np.array([u + spec.rim_radius_m, v, 0.0])
    if fam == "handle_pinch":
        return np.array([spec.grasp.grasp_point_m, 0.0, 0.0])
    raise AssertionError(fam)


@pytest.mark.parametrize("name", sorted(config.OBJECTS))
def test_grasp_chain_maps_grasp_point_to_tcp_axis(name):
    spec = config.OBJECTS[name]
    pos, quat = config.grasp_transform(spec)
    assert abs(np.linalg.norm(quat) - 1.0) < 1e-6
    T_tcp_obj = make_T(pos, quat)
    p = np.ones(4)
    p[:3] = _grasped_point_obj(spec)
    p_tcp = (T_tcp_obj @ p)[:3]
    assert np.allclose(p_tcp, (0.0, 0.0, spec.grasp.rim_tcp_z_m), atol=1e-6), (name, p_tcp)


@pytest.mark.parametrize("name", sorted(config.OBJECTS))
def test_carry_orientation_per_family(name):
    """The carried object's axis direction in the TCP frame matches its family's carry."""
    spec = config.OBJECTS[name]
    _, quat = config.grasp_transform(spec)
    T = make_T((0.0, 0.0, 0.0), quat)
    axis_tcp = T[:3, :3] @ np.array(spec.axis_obj)
    fam = spec.grasp.family
    if fam in ("rim_diam", "rim_edge", "stem_pinch"):
        # upright carry: object axis anti-parallel to tool z (opening toward the gripper)
        assert np.allclose(axis_tcp, (0.0, 0.0, -1.0), atol=1e-6), (name, axis_tcp)
    elif fam == "edge_pinch":
        # on-edge carry: disc axis along the jaw axis (y_tcp), either sign
        assert abs(abs(axis_tcp[1]) - 1.0) < 1e-6, (name, axis_tcp)
    elif fam == "handle_pinch":
        # flat carry: long axis perpendicular to tool z
        assert abs(axis_tcp[2]) < 1e-6, (name, axis_tcp)


# ---------------------------------------------------------------------------------------------
# 3. the mug entry reproduces the frozen v0 constants (baseline cache protection)
# ---------------------------------------------------------------------------------------------


def test_mug_entry_matches_frozen_constants():
    spec = config.OBJECTS["mug"]
    assert spec.object_name == "025_mug"
    assert spec.usd_path == config.OBJECT_USD
    assert spec.mass_kg == config.OBJECT_MASS_KG
    assert spec.bbox_half == config.OBJECT_BBOX_HALF
    assert spec.axis_obj == config.OBJECT_AXIS_OBJ
    assert spec.body_center_uv == config.OBJECT_BODY_CENTER_XZ
    assert spec.rim_radius_m == config.OBJECT_RIM_RADIUS_M
    assert spec.height_m == config.OBJECT_HEIGHT_M
    assert spec.grasp.aperture_rad == config.GRIPPER_APERTURE_GRASP_RAD
    assert spec.grasp.force_min_n == config.GRIP_FORCE_MIN_N
    assert spec.grasp.force_max_n == config.GRIP_FORCE_MAX_N
    pos, quat = config.grasp_transform(spec)
    # bit-exact against the module literals: the cache hash JSON-serializes these floats
    assert pos == config.GRASP_TCP_OBJ_POS
    assert quat == config.GRASP_TCP_OBJ_QUAT


def test_set_active_object_mug_is_hash_stable():
    h0 = config_hash()
    config.set_active_object("mug")
    assert config_hash() == h0, "activating the mug must not perturb the baseline cache hash"


# ---------------------------------------------------------------------------------------------
# 4. set_active_object mutate/restore + cache separation
# ---------------------------------------------------------------------------------------------


def test_set_active_object_mutates_and_restores():
    h_mug = config_hash()
    usd_mug = config.OBJECT_USD
    config.set_active_object("bowl")
    assert config.ACTIVE_OBJECT == "bowl"
    assert config.OBJECT_NAME == "bowl"
    assert config.OBJECT_USD.endswith("bowl_physics.usd")
    assert config.OBJECT_AXIS_OBJ == (0.0, 0.0, 1.0)
    assert config.COACD["object"] == config.OBJECTS["bowl"].coacd
    h_bowl = config_hash()
    assert h_bowl != h_mug
    config.set_active_object("mug")
    assert config.OBJECT_USD == usd_mug
    assert config_hash() == h_mug


def test_unknown_object_raises():
    with pytest.raises(ValueError):
        config.set_active_object("chandelier")


def test_uncalibrated_aperture_estimate_sane():
    config.set_active_object("cup")  # not yet calibrated
    est = config.GRIPPER_APERTURE_GRASP_RAD
    assert 0.0 < est < 0.82
    # linear-jaw estimate: a 65 mm cup needs a smaller close than an 80 mm mug
    assert est > 0.058


def test_cache_dirs_distinct_per_object_and_state():
    dirs = set()
    for name in ("mug", "bowl", "plate"):
        config.set_active_object(name)
        for state in ("both_out", "placement"):
            dirs.add(config.scenario_cache_dir(state))
    assert len(dirs) == 6
    config.set_active_object("mug")
    assert config.scenario_cache_dir("both_out") == config.CACHE_DIR, "mug keeps the legacy root"


def test_object_spec_in_hash_for_non_mug():
    config.set_active_object("plate")
    h1 = config_hash()
    # a registry-geometry change must invalidate that object's cache
    import dataclasses

    spec = config.OBJECTS["plate"]
    config.OBJECTS["plate"] = dataclasses.replace(spec, scale=spec.scale * 1.01)
    try:
        h2 = config_hash()
    finally:
        config.OBJECTS["plate"] = spec
    assert h1 != h2


# ---------------------------------------------------------------------------------------------
# 5. gripper-wide grasp constants
# ---------------------------------------------------------------------------------------------


def test_force_band_ordering():
    # static band is ordered; the dynamic cap must clear the band floor (a motion dip to the
    # static minimum must not abort) — but it is measured independently (wiggle peak x1.5)
    # and may sit below the generous static maximum
    assert 0.0 < config.GRIP_FORCE_MIN_N < config.GRIP_FORCE_MAX_N
    assert config.GRIP_FORCE_MIN_N < config.GRIP_FORCE_EXEC_MAX_N


def test_open_aperture_is_fully_open():
    assert config.GRIPPER_APERTURE_OPEN_RAD == 0.0


def test_pad_bodies_are_the_inner_finger_pair():
    assert set(config.GRIP_PAD_BODIES) == {"left_inner_finger", "right_inner_finger"}


def test_inner_finger_signs_measured():
    signs = config.GRIPPER_INNER_FINGER_SIGNS
    assert signs is not None
    assert set(signs) == {"left_inner_finger_joint", "right_inner_finger_joint"}
    assert all(v in (-1.0, 1.0) for v in signs.values())
