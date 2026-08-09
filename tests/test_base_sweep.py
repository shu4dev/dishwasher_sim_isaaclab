# Copyright (c) 2026, dishsim project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Base-pose sweep: re-expression correctness on a synthetic cache.

The sweep's whole premise is that moving the robot base is a pure re-expression of the cached
statics (machine bodies get ``T_delta``, pedestal/ground stay base-anchored, the robot stays at
its own origin). These tests build a tiny synthetic collision cache — boxes instead of the real
meshes, manifest written with the live ``config_hash`` so ``load_manifest`` passes — and assert
the re-expression invariants directly, with no dependency on the baked asset caches:

- a known ``T_delta`` changes world verdicts the way the geometry says it must;
- the ground (base-anchored) keeps its verdicts through any rebase;
- an identity rebase restores every verdict exactly;
- ``goal_configs``'s ``early_exit_accepted`` stops sampling early without changing what
  "feasible" means.

Run with:
    PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /workspace/isaaclab/env_isaaclab/bin/python -m pytest \
        tests/test_base_sweep.py -v
"""

import json
import os

import numpy as np
import pytest
import trimesh

from dishsim import base_sweep, config, rack_ops
from dishsim.base_sweep import FRONT, Candidate
from dishsim.collision_world import CollisionWorld
from dishsim.geometry import config_hash
from dishsim.placement import SlotFrame, goal_configs
from dishsim.transforms import make_T

ARM_LINKS = ["base_link", "shoulder_link", "upper_arm_link", "forearm_link",
             "wrist_1_link", "wrist_2_link", "wrist_3_link"]

#: Big block in front of the robot: contains the home-pose wrist volume (base-frame x 0.2-1.2)
#: but not the shoulder column at x 0.
MACHINE_T = make_T((0.7, 0.0, 0.3), (0, 0, 0, 1))
MACHINE_EXTENTS = (1.0, 1.0, 1.0)

#: Thick slab whose top is the real ground plane (base-frame z -0.25): anything below hits it.
GROUND_T = make_T((0.4, 0.0, -0.875), (0, 0, 0, 1))
GROUND_EXTENTS = (4.0, 4.0, 1.25)

#: Arm pointing straight down (shoulder_lift +90 deg): forearm/wrist links end up below the
#: ground top.
Q_DOWN = np.array([0.0, 1.571, 0.0, 0.0, 0.0, 0.0])


@pytest.fixture(scope="module")
def synthetic_cache(tmp_path_factory):
    """A minimal, loadable collision cache: box statics + tiny box arm links."""
    config.set_active_object("mug")
    config.apply_scenario("both_out")
    root = tmp_path_factory.mktemp("cache")
    mesh_dir = os.path.join(root, "meshes")
    os.makedirs(mesh_dir)

    def save(name: str, mesh: trimesh.Trimesh) -> str:
        rel = f"meshes/{name}.obj"
        mesh.export(os.path.join(root, rel))
        return rel

    manifest = {
        "frame_convention": config.FRAME_CONVENTION,
        "config_hash": config_hash(),
        "object_name": config.OBJECT_NAME,
        "home_q": list(config.HOME_Q),
        "statics": {
            "E_body_5": {"mesh": save("machine", trimesh.creation.box(extents=MACHINE_EXTENTS)),
                         "T_base_body": MACHINE_T.tolist(), "coacd": False},
            "pedestal": {"mesh": save("pedestal", trimesh.creation.box(extents=config.PEDESTAL_SIZE)),
                         "T_base_body": make_T((0.0, 0.0, -0.125), (0, 0, 0, 1)).tolist(),
                         "coacd": False},
            "ground": {"mesh": save("ground", trimesh.creation.box(extents=GROUND_EXTENTS)),
                       "T_base_body": GROUND_T.tolist(), "coacd": False},
        },
        "arm_links": {
            name: {"mesh": save(f"link_{name}", trimesh.creation.box(extents=(0.04, 0.04, 0.04)))}
            for name in ARM_LINKS
        },
        "gripper_links": {
            "gripper_body": {"mesh": save("gripper", trimesh.creation.box(extents=(0.03, 0.03, 0.03))),
                             "T_wrist3_link": make_T((0.0, 0.05, 0.0), (0, 0, 0, 1)).tolist()},
        },
        "object": {
            "mesh": save("object", trimesh.creation.box(extents=(0.05, 0.05, 0.05))),
            "T_wrist3_obj": (rack_ops.t_wrist3_tcp()
                             @ make_T(*config.grasp_transform(config.OBJECTS["mug"]))).tolist(),
            "coacd": False,
        },
        "t_wrist3_tcp": rack_ops.t_wrist3_tcp().tolist(),
        "statics_state": {"rack_lower_m": -0.2, "rack_upper_m": -0.2, "door_deg": 89.05},
        "scenario": "both_out",
    }
    with open(os.path.join(root, "scene_state.json"), "w") as f:
        json.dump(manifest, f)
    return str(root)


@pytest.fixture()
def world(synthetic_cache):
    config.set_active_object("mug")
    config.apply_scenario("both_out")
    return CollisionWorld(cache_dir=synthetic_cache, self_check=False)


def statics0(world) -> dict:
    return {name: np.array(entry["T_base_body"])
            for name, entry in world.manifest["statics"].items()}


# ---------------------------------------------------------------------------------------------
# candidate transform math
# ---------------------------------------------------------------------------------------------


def test_front_candidate_is_the_identity_re_expression():
    assert np.allclose(FRONT.T_delta(), np.eye(4), atol=1e-12)


def test_candidate_re_expression_matches_hand_computed_geometry():
    """Base at (0, -0.5) yawed +90 deg: a world point re-expresses by rotate-then-shift."""
    cand = Candidate(0.0, -0.5, 90.0)
    p_w = np.array([0.65, 0.0, 0.25, 1.0])
    p_b1 = np.linalg.inv(cand.T_w_base()) @ p_w
    # p_w - t = (0.65, 0.5, 0.0); Rz(-90) maps (x, y) -> (y, -x)
    assert np.allclose(p_b1[:3], (0.5, -0.65, 0.0), atol=1e-9)

    # T_delta must agree: an old-base point maps through it to the same new-base coordinates
    p_b0 = p_w[:3] - np.asarray(config.ROBOT_BASE_POS_W)
    assert np.allclose((cand.T_delta() @ np.append(p_b0, 1.0))[:3], p_b1[:3], atol=1e-9)


# ---------------------------------------------------------------------------------------------
# world re-expression
# ---------------------------------------------------------------------------------------------


def test_set_static_transform_moves_a_static_and_its_verdicts(world):
    home = np.array(config.HOME_Q)
    assert world.in_collision(home), "home wrist volume must start inside the machine block"

    far = MACHINE_T.copy()
    far[0, 3] += 10.0
    world.set_static_transform("E_body_5", far)
    assert not world.in_collision(home)

    world.set_static_transform("E_body_5", MACHINE_T)
    assert world.in_collision(home)


def test_set_static_transform_composes_with_set_static_offset(world):
    """set_static_offset derives from the CURRENT static pose, so it must ride a rebase."""
    far = MACHINE_T.copy()
    far[0, 3] += 10.0
    world.set_static_transform("E_body_5", far)
    world.set_static_offset("E_body_5", -10.0)  # body +y is base +y here (identity rotation)
    # offset moved it along body y, not back to the origin pose: still free at home
    assert not world.in_collision(np.array(config.HOME_Q))
    world.set_static_offset("E_body_5", 0.0)
    assert not world.in_collision(np.array(config.HOME_Q))


def test_rebase_moves_the_machine_but_not_the_ground(world):
    s0 = statics0(world)
    home = np.array(config.HOME_Q)
    assert world.in_collision(home)
    assert world.in_collision(Q_DOWN), "the down-pointing arm must start inside the ground slab"

    # a base 5 m behind the machine: the machine block leaves the arm's volume entirely
    base_sweep.rebase_statics(world, s0, Candidate(-5.0, 0.0, 0.0).T_delta())
    assert not world.in_collision(home)
    # the ground is base-anchored: the down-pointing arm still hits it wherever the base stands
    assert world.in_collision(Q_DOWN)


def test_identity_rebase_restores_every_verdict(world):
    s0 = statics0(world)
    rng = np.random.default_rng(7)
    qs = [rng.uniform(-2.0, 2.0, 6) for _ in range(150)]
    before = [bool(world.in_collision(q)) for q in qs]

    base_sweep.rebase_statics(world, s0, Candidate(-1.0, -0.5, 45.0).T_delta())
    base_sweep.rebase_statics(world, s0, FRONT.T_delta())
    after = [bool(world.in_collision(q)) for q in qs]
    assert before == after


def test_rebased_machine_lands_where_the_world_frame_says(world):
    """Yaw the base: the machine's new base-frame pose must equal inv(T_w_b1) @ T_w_body."""
    s0 = statics0(world)
    cand = Candidate(0.1, -0.6, 30.0)
    T_w_b0 = make_T(config.ROBOT_BASE_POS_W, config.ROBOT_BASE_QUAT_W)
    T_w_body = T_w_b0 @ s0["E_body_5"]

    expected = np.linalg.inv(cand.T_w_base()) @ T_w_body
    assert np.allclose(cand.T_delta() @ s0["E_body_5"], expected, atol=1e-9)


# ---------------------------------------------------------------------------------------------
# goal-config early exit
# ---------------------------------------------------------------------------------------------


@pytest.fixture()
def open_world(world):
    """The synthetic world with the machine block parked far away (an unobstructed slot)."""
    far = MACHINE_T.copy()
    far[0, 3] += 10.0
    world.set_static_transform("E_body_5", far)
    return world


def _slot() -> SlotFrame:
    # a standing slot in known-feasible territory (the real slot grid's near column)
    return SlotFrame(slot_id=0, T_base_slot=make_T((0.55, 0.0, -0.15), (0, 0, 0, 1)),
                     width_m=0.08, source="manual")


def test_goal_configs_early_exit_stops_sampling(open_world):
    config.set_active_object("mug")
    full = goal_configs(_slot(), open_world, np.random.default_rng(3), n_samples=32)
    assert len(full.configs) > 0, "the unobstructed slot must accept configurations"
    assert full.n_pose_samples == 32

    early = goal_configs(_slot(), open_world, np.random.default_rng(3), n_samples=32,
                         early_exit_accepted=1)
    assert len(early.configs) > 0
    assert early.n_pose_samples < 32, "early exit must stop before the full sample budget"


def test_goal_configs_early_exit_default_changes_nothing(open_world):
    config.set_active_object("mug")
    a = goal_configs(_slot(), open_world, np.random.default_rng(11), n_samples=8)
    b = goal_configs(_slot(), open_world, np.random.default_rng(11), n_samples=8,
                     early_exit_accepted=None)
    assert np.array_equal(a.configs, b.configs)
    assert a.n_pose_samples == b.n_pose_samples
    assert a.n_collision_reject == b.n_collision_reject


# ---------------------------------------------------------------------------------------------
# metrics + ranking
# ---------------------------------------------------------------------------------------------


def test_largest_rectangle_finds_the_known_block():
    mask = np.zeros((5, 6), dtype=bool)
    mask[1:4, 2:5] = True
    assert base_sweep.largest_rectangle(mask) == (1, 3, 2, 4)
    assert base_sweep.largest_rectangle(np.zeros((3, 3), dtype=bool)) == (0, -1, 0, -1)


def test_rank_key_orders_infeasible_last_and_criteria_first():
    gated = {"feasible": False}
    weak = {"feasible": True, "n_criteria": 1, "weighted": 99.0}
    strong = {"feasible": True, "n_criteria": 3, "weighted": 10.0}
    ranked = sorted([weak, gated, strong], key=base_sweep.rank_key, reverse=True)
    assert ranked == [strong, weak, gated]


def test_seeded_rng_is_deterministic_across_calls():
    a = base_sweep._seeded_rng("x+0.1", "placement", "cup", 3, 16).integers(0, 1 << 30, 8)
    b = base_sweep._seeded_rng("x+0.1", "placement", "cup", 3, 16).integers(0, 1 << 30, 8)
    assert np.array_equal(a, b)
