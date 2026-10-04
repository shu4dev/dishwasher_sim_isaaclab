# Copyright (c) 2026, dishsim project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""The static scene: dishwasher locked open on the ground plane, plus manipulable objects.

Kit-only module (imports ``isaaclab.*`` at module scope) — import it only after ``AppLauncher``
has started the app. ``pxr`` is imported lazily inside the functions that touch USD.

Key invariants enforced here:

- One frame convention: the base frame is the world frame posed by
  :data:`dishsim.config.ROBOT_BASE_POS_W` / :data:`dishsim.config.ROBOT_BASE_QUAT_W` — the
  frozen cache anchor every cached coordinate is expressed in (a robot-era mount, kept so the
  shipped caches stay valid).
- Objects teleport: a runner writes root poses directly and lets physics settle; the only
  actuated degrees of freedom are the dishwasher's own rack/door drives
  (:func:`hold_targets` pins them every step).
"""

import math

import numpy as np

import isaaclab.sim as sim_utils
from isaaclab.assets import AssetBaseCfg, RigidObjectCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sensors import ContactSensorCfg
from isaaclab.utils.configclass import configclass

from . import config
from .machine import DISHWASHER_V0_CFG
from .quats import wxyz_to_xyzw, xyzw_to_wxyz


# ---------------------------------------------------------------------------------------------
# scene construction
# ---------------------------------------------------------------------------------------------


def make_scene_cfg(objects: list | None = None) -> InteractiveSceneCfg:
    """Build the scene config (single env): ground, light, pedestal, dishwasher, objects.

    Args:
        objects: Manipulable objects to spawn, one dict per item with keys ``name``,
            ``usd_path``, ``pos``, ``quat`` and optionally ``contact_filters`` (see
            :func:`_add_object`). Callers may also attach further ``RigidObjectCfg``
            attributes to the returned config directly.
    """

    @configclass
    class SceneCfg(InteractiveSceneCfg):
        ground = AssetBaseCfg(prim_path="/World/GroundPlane", spawn=sim_utils.GroundPlaneCfg())
        light = AssetBaseCfg(
            prim_path="/World/Light", spawn=sim_utils.DomeLightCfg(intensity=3000.0, color=(0.75, 0.75, 0.75))
        )
        pedestal = AssetBaseCfg(
            prim_path="{ENV_REGEX_NS}/Pedestal",
            spawn=sim_utils.CuboidCfg(
                size=config.PEDESTAL_SIZE,
                collision_props=sim_utils.CollisionPropertiesCfg(),
                visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.3, 0.3, 0.3)),
            ),
            init_state=AssetBaseCfg.InitialStateCfg(pos=config.PEDESTAL_POS_W),
        )
        dishwasher = DISHWASHER_V0_CFG.replace(prim_path="{ENV_REGEX_NS}/Dishwasher")

    scene_cfg = SceneCfg(num_envs=1, env_spacing=3.0)
    for spec in objects or ():
        _add_object(scene_cfg, spec)
    return scene_cfg


def _add_object(scene_cfg, spec: dict) -> None:
    """Attach one manipulable object to a scene config.

    Args:
        scene_cfg: Scene configclass instance to mutate.
        spec: ``{"name", "usd_path", "pos", "quat"}`` plus optional ``"contact_filters"``
            (prim paths to resolve per-partner forces against — an object's peers, so a
            support graph can be read from contacts) and optional ``"color"`` (linear RGB
            0-1 render tint, see :func:`~dishsim.config.display_color`; visual only).
    """
    name = spec["name"]
    color = spec.get("color")
    setattr(
        scene_cfg,
        name,
        RigidObjectCfg(
            prim_path="{ENV_REGEX_NS}/" + name,
            spawn=sim_utils.UsdFileCfg(
                usd_path=spec["usd_path"],
                rigid_props=sim_utils.RigidBodyPropertiesCfg(max_depenetration_velocity=5.0),
                activate_contact_sensors=True,
                # visual only: binds over the asset's own material so classes are tellable
                # apart on camera. Physics and collision geometry are untouched.
                visual_material=(sim_utils.PreviewSurfaceCfg(diffuse_color=tuple(color))
                                 if color is not None else None),
            ),
            init_state=RigidObjectCfg.InitialStateCfg(
                # spec quats are project-order XYZW; isaaclab 2.1 cfg tuples are WXYZ
                pos=tuple(spec["pos"]), rot=tuple(xyzw_to_wxyz(list(spec["quat"])))
            ),
        ),
    )
    filters = list(spec.get("contact_filters", []))
    if filters:
        setattr(
            scene_cfg,
            f"{name}_contact",
            ContactSensorCfg(
                prim_path="{ENV_REGEX_NS}/" + name,
                update_period=0.0,
                filter_prim_paths_expr=filters,
            ),
        )


# ---------------------------------------------------------------------------------------------
# state writes + assertions
# ---------------------------------------------------------------------------------------------


def hold_targets(scene) -> None:
    """(Re-)issue the standing dishwasher drive targets (racks pinned, door locked).

    Call after every ``scene.reset()`` (reset clears command buffers) — and it is cheap enough
    to call every step of a settle loop.
    """
    dw = scene["dishwasher"]
    dw.set_joint_position_target(target=dw.data.default_joint_pos.clone())


def write_default_states(scene) -> None:
    """Standalone-script reset dance: write default root/joint states, reset, re-arm targets."""
    dw = scene["dishwasher"]
    root_pose = dw.data.default_root_state[:, :7].clone()
    root_pose[:, :3] += scene.env_origins
    dw.write_root_pose_to_sim(root_pose=root_pose)
    dw.write_root_velocity_to_sim(root_velocity=dw.data.default_root_state[:, 7:].clone())
    dw.write_joint_position_to_sim(position=dw.data.default_joint_pos.clone())
    dw.write_joint_velocity_to_sim(velocity=dw.data.default_joint_vel.clone())
    # scene.reset() clears command buffers — targets must be re-armed AFTER it (a target set
    # before reset silently reverts; found the hard way during scene bring-up)
    scene.reset()
    hold_targets(scene)


def assert_frames(scene) -> None:
    """Assert the single frame convention this whole project relies on."""
    dw_pos = scene["dishwasher"].data.root_pos_w[0].cpu().numpy()
    dw_quat = wxyz_to_xyzw(scene["dishwasher"].data.root_quat_w[0].cpu().numpy())
    assert np.allclose(dw_pos, config.DISHWASHER_POS_W, atol=1e-4), f"dishwasher root at {dw_pos}"
    # sign-agnostic: q and -q are the same rotation, and the sim may hand back either sign
    assert np.allclose(dw_quat, config.DISHWASHER_QUAT_W, atol=1e-4) or np.allclose(
        -dw_quat, config.DISHWASHER_QUAT_W, atol=1e-4
    ), (
        f"dishwasher root rotation {dw_quat} != config.DISHWASHER_QUAT_W "
        f"{config.DISHWASHER_QUAT_W} — the frame convention is broken"
    )


def statics_report(scene) -> dict:
    """Door/rack state vs the configured lock targets (deviations mean something is pushing)."""
    dw = scene["dishwasher"]
    door_ids, _ = dw.find_joints("RevoluteJoint_dishwasher_2_middle")
    down_ids, _ = dw.find_joints("PrismaticJoint_dishwasher_2_down")
    up_ids, _ = dw.find_joints("PrismaticJoint_dishwasher_2_up")
    jp = dw.data.joint_pos[0]
    return {
        "door_deg": math.degrees(float(jp[door_ids[0]])),
        "rack_lower_m": float(jp[down_ids[0]]),
        "rack_upper_m": float(jp[up_ids[0]]),
        "door_err_deg": abs(math.degrees(float(jp[door_ids[0]])) - math.degrees(config.DOOR_INIT_RAD)),
        "rack_lower_err_m": abs(float(jp[down_ids[0]]) - config.RACK_LOWER_EXT_M),
        "rack_upper_err_m": abs(float(jp[up_ids[0]]) - config.RACK_UPPER_EXT_M),
    }
