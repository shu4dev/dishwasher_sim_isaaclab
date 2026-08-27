# Copyright (c) 2026, dishsim project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Kit-free sanity checks on the multi-object registry (config.OBJECTS).

Covers: per-spec geometry/mass sanity, the grasp-chain math for every family (its output IS
the config_hash "grasp" key — baseline cache protection), set_active_object mutate/restore,
and per-object cache-dir separation.
"""

import numpy as np
import pytest

from dishsim import config
from dishsim.geometry import config_hash
from dishsim.transforms import make_T

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


# ---------------------------------------------------------------------------------------------
# 3. baseline cache protection
# ---------------------------------------------------------------------------------------------


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
