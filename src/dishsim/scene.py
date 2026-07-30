# Copyright (c) 2026, dishsim project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""The static v0 scene: dishwasher locked open, UR5e with frozen gripper, object welded to TCP.

Kit-only module (imports ``isaaclab.*`` at module scope) — import it only after ``AppLauncher``
has started the app. ``pxr`` is imported lazily inside the functions that touch USD.

Key invariants enforced here:

- One frame convention: robot-base frame == world frame shifted by
  :data:`dishsim.config.ROBOT_BASE_POS_W`, identity base rotation (asserted).
- The gripper is never actuated beyond holding :data:`dishsim.config.GRIPPER_APERTURE_RAD` on
  ``finger_joint``; the five mimic joints are never touched.
- The carried object is welded to ``wrist_3_link`` by a USD ``FixedJoint`` whose local pose is
  authored *analytically* from the config grasp chain (never captured from live prim poses), so
  the weld is exactly consistent with the FK used by the collision world and the goal sampler:
  ``T_wrist3_obj = T_wrist3_tcp . T_tcp_obj``.
"""

import math

import numpy as np
import torch

import isaaclab.sim as sim_utils
from isaaclab.assets import AssetBaseCfg, RigidObjectCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sensors import ContactSensorCfg, FrameTransformerCfg
from isaaclab.sensors.frame_transformer.frame_transformer_cfg import OffsetCfg
from isaaclab.utils.configclass import configclass

from . import config
from .robots import DISHWASHER_V0_CFG, UR5E_ROBOTIQ_2F_85_CFG
from .transforms import T_to_pos_quat, make_T
from .ur5e_kin import fk_wrist3

WELD_PRIM_NAME = "WeldToWrist"


# ---------------------------------------------------------------------------------------------
# grasp chain (pure numpy — mirrored by the collision world)
# ---------------------------------------------------------------------------------------------


def t_wrist3_tcp() -> np.ndarray:
    """Fixed transform wrist_3_link -> TCP from config; raises until the rotation is measured.

    Returns:
        Homogeneous transform, shape [4, 4].
    """
    if config.T_WRIST3_TCP_QUAT is None:
        raise RuntimeError(
            "config.T_WRIST3_TCP_QUAT is not measured yet — run "
            "`scripts/run_kit.sh scripts/10_v0_scene.py --headless --measure` and freeze the "
            "printed constant into src/dishsim/config.py first."
        )
    return make_T(config.T_WRIST3_TCP_POS, config.T_WRIST3_TCP_QUAT)


def t_wrist3_obj() -> np.ndarray:
    """Fixed transform wrist_3_link -> carried-object frame (the weld's local pose)."""
    return t_wrist3_tcp() @ make_T(config.GRASP_TCP_OBJ_POS, config.GRASP_TCP_OBJ_QUAT)


def grasp_pose_w(q=None) -> tuple[np.ndarray, np.ndarray]:
    """World pose of the carried object when the arm is at ``q`` (default: home).

    Returns:
        (position [m] shape [3], XYZW quaternion shape [4]).
    """
    q = config.HOME_Q if q is None else q
    T_w_base = make_T(config.ROBOT_BASE_POS_W, config.ROBOT_BASE_QUAT_W)
    return T_to_pos_quat(T_w_base @ fk_wrist3(np.asarray(q)) @ t_wrist3_obj())


def world_to_base(pos_w) -> np.ndarray:
    """World position -> robot-base-frame position (identity base rotation, asserted at build)."""
    return np.asarray(pos_w, dtype=float) - np.asarray(config.ROBOT_BASE_POS_W)


# ---------------------------------------------------------------------------------------------
# scene construction
# ---------------------------------------------------------------------------------------------


def make_scene_cfg(with_object: bool = True, with_robot_contacts: bool = False) -> InteractiveSceneCfg:
    """Build the v0 scene config (single env).

    Args:
        with_object: Spawn the carried object at the home-configuration grasp pose. Set False
            for the ``--measure`` bootstrap run that derives the TCP rotation constant.
        with_robot_contacts: Add contact sensors over every robot body (arm + gripper) — used
            by the parity check and execution-time contact monitoring.
    """
    robot_joint_pos = dict(zip(config.ARM_JOINTS, config.HOME_Q))
    # gripper: drive joint starts open and is *held* at the frozen aperture by its position
    # target (write_default_states); mimic-driven joints follow physically and are never set.
    robot_joint_pos[config.GRIPPER_JOINT] = 0.0

    robot_cfg = UR5E_ROBOTIQ_2F_85_CFG.replace(
        prim_path="{ENV_REGEX_NS}/Robot",
        init_state=UR5E_ROBOTIQ_2F_85_CFG.init_state.replace(
            pos=config.ROBOT_BASE_POS_W,
            rot=config.ROBOT_BASE_QUAT_W,
            joint_pos={**UR5E_ROBOTIQ_2F_85_CFG.init_state.joint_pos, **robot_joint_pos},
        ),
    )

    @configclass
    class V0SceneCfg(InteractiveSceneCfg):
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
        robot = robot_cfg
        dishwasher = DISHWASHER_V0_CFG.replace(prim_path="{ENV_REGEX_NS}/Dishwasher")

        # TCP + fingertip frames (offsets measured, see docs/joint_report.md)
        ee_frame = FrameTransformerCfg(
            prim_path="{ENV_REGEX_NS}/Robot/base_link",
            debug_vis=False,
            target_frames=[
                FrameTransformerCfg.FrameCfg(
                    prim_path="{ENV_REGEX_NS}/Robot/Gripper/Robotiq_2F_85/base_link",
                    name="ee_tcp",
                    offset=OffsetCfg(pos=(0.0, 0.0, 0.13)),
                ),
                FrameTransformerCfg.FrameCfg(
                    prim_path="{ENV_REGEX_NS}/Robot/Gripper/Robotiq_2F_85/left_inner_finger",
                    name="tool_leftfinger",
                    offset=OffsetCfg(pos=(0.0, -0.0592, 0.1199)),
                ),
                FrameTransformerCfg.FrameCfg(
                    prim_path="{ENV_REGEX_NS}/Robot/Gripper/Robotiq_2F_85/right_inner_finger",
                    name="tool_rightfinger",
                    offset=OffsetCfg(pos=(0.0, 0.0592, 0.1199)),
                ),
            ],
        )

    scene_cfg = V0SceneCfg(num_envs=1, env_spacing=3.0)

    if with_robot_contacts:
        scene_cfg.robot_contacts_arm = ContactSensorCfg(
            prim_path="{ENV_REGEX_NS}/Robot/(base_link|shoulder_link|upper_arm_link|forearm_link"
            "|wrist_1_link|wrist_2_link|wrist_3_link)",
            update_period=0.0,
        )
        scene_cfg.robot_contacts_gripper = ContactSensorCfg(
            prim_path="{ENV_REGEX_NS}/Robot/Gripper/Robotiq_2F_85/.*",
            update_period=0.0,
        )

    if with_object:
        obj_pos, obj_quat = grasp_pose_w()
        scene_cfg.carried_object = RigidObjectCfg(
            prim_path="{ENV_REGEX_NS}/CarriedObject",
            spawn=sim_utils.UsdFileCfg(
                usd_path=config.OBJECT_USD,
                rigid_props=sim_utils.RigidBodyPropertiesCfg(max_depenetration_velocity=5.0),
                activate_contact_sensors=True,
            ),
            init_state=RigidObjectCfg.InitialStateCfg(pos=tuple(obj_pos), rot=tuple(obj_quat)),
        )
        # net contact force on the carried object — must stay ~0 while welded (any persistent
        # force means the fingers or the world are touching it). The filter list resolves
        # per-partner forces into force_matrix_w for diagnosis (sensor prim is a single prim
        # per env, so filtering is legal here).
        scene_cfg.object_contact = ContactSensorCfg(
            prim_path="{ENV_REGEX_NS}/CarriedObject",
            update_period=0.0,
            filter_prim_paths_expr=[
                "{ENV_REGEX_NS}/Robot/Gripper/Robotiq_2F_85/left_inner_finger",
                "{ENV_REGEX_NS}/Robot/Gripper/Robotiq_2F_85/right_inner_finger",
                "{ENV_REGEX_NS}/Robot/Gripper/Robotiq_2F_85/left_inner_knuckle",
                "{ENV_REGEX_NS}/Robot/Gripper/Robotiq_2F_85/right_inner_knuckle",
                "{ENV_REGEX_NS}/Robot/Gripper/Robotiq_2F_85/left_outer_knuckle",
                "{ENV_REGEX_NS}/Robot/Gripper/Robotiq_2F_85/right_outer_knuckle",
                "{ENV_REGEX_NS}/Robot/Gripper/Robotiq_2F_85/left_outer_finger",
                "{ENV_REGEX_NS}/Robot/Gripper/Robotiq_2F_85/right_outer_finger",
                "{ENV_REGEX_NS}/Robot/Gripper/Robotiq_2F_85/base_link",
                "{ENV_REGEX_NS}/Robot/wrist_3_link",
            ],
        )
    return scene_cfg


# ---------------------------------------------------------------------------------------------
# weld (USD FixedJoint, authored analytically, toggleable at runtime)
# ---------------------------------------------------------------------------------------------


def author_weld(stage, env_path: str = "/World/envs/env_0") -> str:
    """Author the wrist->object FixedJoint on the live stage BEFORE ``sim.reset()``.

    The joint's local pose on the wrist side is the analytic grasp chain, so after reset (which
    also teleports the object to the matching world pose) the constraint starts with zero
    error. ``physics:excludeFromArticulation`` keeps PhysX from absorbing the object into the
    robot articulation.

    Returns:
        The weld prim path (use with :func:`set_weld_enabled`).
    """
    from pxr import Gf, Sdf, UsdPhysics  # noqa: PLC0415

    pos, quat = T_to_pos_quat(t_wrist3_obj())  # wrist_3_link -> object
    weld_path = f"{env_path}/CarriedObject/{WELD_PRIM_NAME}"
    joint = UsdPhysics.FixedJoint.Define(stage, Sdf.Path(weld_path))
    joint.GetBody0Rel().SetTargets([Sdf.Path(f"{env_path}/Robot/wrist_3_link")])
    joint.GetBody1Rel().SetTargets([Sdf.Path(f"{env_path}/CarriedObject")])
    joint.GetLocalPos0Attr().Set(Gf.Vec3f(*[float(v) for v in pos]))
    # Gf.Quatf is (w, (x, y, z)); config quats are XYZW
    joint.GetLocalRot0Attr().Set(Gf.Quatf(float(quat[3]), Gf.Vec3f(float(quat[0]), float(quat[1]), float(quat[2]))))
    joint.GetLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
    joint.GetLocalRot1Attr().Set(Gf.Quatf(1.0, Gf.Vec3f(0.0, 0.0, 0.0)))
    UsdPhysics.Joint(joint.GetPrim()).CreateExcludeFromArticulationAttr(True)
    return weld_path


def set_weld_enabled(stage, weld_path: str, enabled: bool) -> None:
    """Toggle the weld constraint (release = False)."""
    prim = stage.GetPrimAtPath(weld_path)
    if not prim.IsValid():
        raise RuntimeError(f"weld prim not found at {weld_path}")
    prim.GetAttribute("physics:jointEnabled").Set(bool(enabled))


# ---------------------------------------------------------------------------------------------
# state writes + assertions
# ---------------------------------------------------------------------------------------------


def hold_targets(scene) -> None:
    """(Re-)issue the standing position targets: arm at home, gripper frozen, statics pinned.

    Call after every ``scene.reset()`` (reset clears command buffers) — and it is cheap enough
    to call every step of a settle loop.
    """
    robot = scene["robot"]
    targets = robot.data.default_joint_pos.torch.clone()
    ids, _ = robot.find_joints(config.GRIPPER_JOINT)
    targets[:, ids[0]] = config.GRIPPER_APERTURE_RAD
    robot.set_joint_position_target_index(target=targets)
    dw = scene["dishwasher"]
    dw.set_joint_position_target_index(target=dw.data.default_joint_pos.torch.clone())


def write_default_states(scene) -> None:
    """Standalone-script reset dance: write default root/joint states, reset, re-arm targets."""
    for name in ("robot", "dishwasher"):
        art = scene[name]
        root_pose = art.data.default_root_pose.torch.clone()
        root_pose[:, :3] += scene.env_origins
        art.write_root_pose_to_sim_index(root_pose=root_pose)
        art.write_root_velocity_to_sim_index(root_velocity=art.data.default_root_vel.torch.clone())
        art.write_joint_position_to_sim_index(position=art.data.default_joint_pos.torch.clone())
        art.write_joint_velocity_to_sim_index(velocity=art.data.default_joint_vel.torch.clone())
    if "carried_object" in scene.keys():
        obj = scene["carried_object"]
        root_pose = obj.data.default_root_pose.torch.clone()
        root_pose[:, :3] += scene.env_origins
        obj.write_root_pose_to_sim_index(root_pose=root_pose)
        obj.write_root_velocity_to_sim_index(root_velocity=obj.data.default_root_vel.torch.clone())
    # scene.reset() clears command buffers — targets must be re-armed AFTER it (a finger target
    # set before reset silently reverts to the open pose; found the hard way in Phase C)
    scene.reset()
    hold_targets(scene)


def assert_frames(scene) -> None:
    """Assert the single frame convention this whole project relies on."""
    base_pos = scene["robot"].data.root_pos_w.torch[0].cpu().numpy()
    base_quat = scene["robot"].data.root_quat_w.torch[0].cpu().numpy()
    assert np.allclose(base_pos, config.ROBOT_BASE_POS_W, atol=1e-4), f"robot base at {base_pos}"
    assert np.allclose(base_quat, config.ROBOT_BASE_QUAT_W, atol=1e-4) or np.allclose(
        -base_quat, config.ROBOT_BASE_QUAT_W, atol=1e-4
    ), f"robot base rotation {base_quat} != identity — the robot-base frame convention is broken"


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


def measure_tcp(scene) -> dict:
    """Live wrist_3 -> TCP transform (the constant frozen into config after ``--measure``)."""
    robot = scene["robot"]
    w3_ids, _ = robot.find_bodies("wrist_3_link")
    w3_pos = robot.data.body_link_pos_w.torch[0, w3_ids[0]].cpu().numpy()
    w3_quat = robot.data.body_link_quat_w.torch[0, w3_ids[0]].cpu().numpy()
    tcp_pos = scene["ee_frame"].data.target_pos_w.torch[0, 0].cpu().numpy()
    tcp_quat = scene["ee_frame"].data.target_quat_w.torch[0, 0].cpu().numpy()
    from .transforms import T_inv, mat_to_quat, quat_to_mat  # noqa: PLC0415

    T_w_w3 = make_T(w3_pos, w3_quat)
    T_w_tcp = make_T(tcp_pos, tcp_quat)
    T_w3_tcp = T_inv(T_w_w3) @ T_w_tcp
    return {
        "wrist3_pos_w": w3_pos.tolist(),
        "wrist3_quat_w": w3_quat.tolist(),
        "tcp_pos_w": tcp_pos.tolist(),
        "tcp_quat_w": tcp_quat.tolist(),
        "t_wrist3_tcp_pos": T_w3_tcp[:3, 3].tolist(),
        "t_wrist3_tcp_quat_xyzw": mat_to_quat(T_w3_tcp[:3, :3]).tolist(),
    }
