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
  (:func:`set_rack_target_override` + :func:`hold_targets`).
"""

import math

import numpy as np
from scipy.spatial.transform import Rotation

import isaaclab.sim as sim_utils
from isaaclab.assets import AssetBaseCfg, RigidObjectCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sensors import ContactSensorCfg
from isaaclab.utils.configclass import configclass

from . import config
from .machine import DISHWASHER_V0_CFG
from .transforms import T_inv, make_T


# ---------------------------------------------------------------------------------------------
# frames
# ---------------------------------------------------------------------------------------------


def world_to_base(pos_w) -> np.ndarray:
    """World position -> base-frame position (full transform; the base may be yawed).

    Reduces to exact ``pos_w - ROBOT_BASE_POS_W`` at the identity base quaternion.

    Returns:
        Position in the base frame [m], shape [3].
    """
    T_base_w = T_inv(make_T(config.ROBOT_BASE_POS_W, config.ROBOT_BASE_QUAT_W))
    return T_base_w[:3, :3] @ np.asarray(pos_w, dtype=float) + T_base_w[:3, 3]


def base_to_world(pos_b) -> np.ndarray:
    """Base-frame position -> world position (full transform; the base may be yawed).

    Inverse of :func:`world_to_base`; reduces to exact ``pos_b + ROBOT_BASE_POS_W`` at the
    identity base quaternion.

    Returns:
        Position in the world frame [m], shape [3].
    """
    T_w_base = make_T(config.ROBOT_BASE_POS_W, config.ROBOT_BASE_QUAT_W)
    return T_w_base[:3, :3] @ np.asarray(pos_b, dtype=float) + T_w_base[:3, 3]


def countertop_pose_w(instance: int = 0) -> tuple[np.ndarray, np.ndarray]:
    """World staging pose of item ``instance`` on the countertop (axis-convention aware).

    Returns:
        (position [m] shape [3], XYZW quaternion shape [4]).
    """
    (pos_w, yaw_deg) = config.OBJECT_COUNTERTOP_POSES_W[instance]
    r_yaw = Rotation.from_euler("z", float(yaw_deg), degrees=True)
    if tuple(config.OBJECT_AXIS_OBJ) == (0.0, 1.0, 0.0):  # Y-up mug stands via Rx(+90)
        r_stand = Rotation.from_euler("x", 90.0, degrees=True)
    else:  # Z-up and X-up (flat) objects stage as authored
        r_stand = Rotation.identity()
    return np.asarray(pos_w, dtype=float), (r_yaw * r_stand).as_quat()  # XYZW


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
                pos=tuple(spec["pos"]), rot=tuple(spec["quat"])
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


#: persistent rack drive-target override consumed by hold_targets (rack transitions ramp
#: these each step; the scenario defaults apply when None)
_RACK_TARGET_OVERRIDE: dict[str, float] | None = None


def set_rack_target_override(targets: dict[str, float] | None) -> None:
    """Override the dishwasher rack drive targets that :func:`hold_targets` pins every step.

    A rack transition ramps the moved joint's target through this; pass ``None`` to restore
    the scenario defaults (do so only when the scenario's post-action state matches — after a
    completed transition the override must stay at the target for the rest of the run).
    """
    global _RACK_TARGET_OVERRIDE
    _RACK_TARGET_OVERRIDE = dict(targets) if targets else None


def hold_targets(scene) -> None:
    """(Re-)issue the standing dishwasher drive targets (racks pinned, door locked).

    Call after every ``scene.reset()`` (reset clears command buffers) — and it is cheap enough
    to call every step of a settle loop.
    """
    dw = scene["dishwasher"]
    dw_targets = dw.data.default_joint_pos.torch.clone()
    if _RACK_TARGET_OVERRIDE:
        for jname, val in _RACK_TARGET_OVERRIDE.items():
            jids, _ = dw.find_joints(jname)
            dw_targets[:, jids[0]] = float(val)
    dw.set_joint_position_target_index(target=dw_targets)


def write_default_states(scene) -> None:
    """Standalone-script reset dance: write default root/joint states, reset, re-arm targets."""
    dw = scene["dishwasher"]
    root_pose = dw.data.default_root_pose.torch.clone()
    root_pose[:, :3] += scene.env_origins
    dw.write_root_pose_to_sim_index(root_pose=root_pose)
    dw.write_root_velocity_to_sim_index(root_velocity=dw.data.default_root_vel.torch.clone())
    dw.write_joint_position_to_sim_index(position=dw.data.default_joint_pos.torch.clone())
    dw.write_joint_velocity_to_sim_index(velocity=dw.data.default_joint_vel.torch.clone())
    # scene.reset() clears command buffers — targets must be re-armed AFTER it (a target set
    # before reset silently reverts; found the hard way during scene bring-up)
    scene.reset()
    hold_targets(scene)


def assert_frames(scene) -> None:
    """Assert the single frame convention this whole project relies on."""
    dw_pos = scene["dishwasher"].data.root_pos_w.torch[0].cpu().numpy()
    dw_quat = scene["dishwasher"].data.root_quat_w.torch[0].cpu().numpy()
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
    jp = dw.data.joint_pos.torch[0]
    return {
        "door_deg": math.degrees(float(jp[door_ids[0]])),
        "rack_lower_m": float(jp[down_ids[0]]),
        "rack_upper_m": float(jp[up_ids[0]]),
        "door_err_deg": abs(math.degrees(float(jp[door_ids[0]])) - math.degrees(config.DOOR_INIT_RAD)),
        "rack_lower_err_m": abs(float(jp[down_ids[0]]) - config.RACK_LOWER_EXT_M),
        "rack_upper_err_m": abs(float(jp[up_ids[0]]) - config.RACK_UPPER_EXT_M),
    }
