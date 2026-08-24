# Copyright (c) 2026, dishsim project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""The static v0 scene: dishwasher locked open, UR5e with frozen gripper, object welded to TCP.

Kit-only module (imports ``isaaclab.*`` at module scope) — import it only after ``AppLauncher``
has started the app. ``pxr`` is imported lazily inside the functions that touch USD.

Key invariants enforced here:

- One frame convention: robot-base frame == world frame posed by
  :data:`dishsim.config.ROBOT_BASE_POS_W` / :data:`dishsim.config.ROBOT_BASE_QUAT_W`
  (asserted against the live spawn at build).
- The gripper is actuated only via ``finger_joint``, between exactly two calibrated apertures
  (:data:`dishsim.config.GRIPPER_APERTURE_OPEN_RAD` at trial endpoints,
  :data:`dishsim.config.GRIPPER_APERTURE_GRASP_RAD` during all planned motion). The mimic
  joints are never position-written; the two stiff ``.*_inner_finger_joint`` drive *targets*
  are set mimic-consistently so they do not fight the mimic constraint (signs measured by
  ``scripts/setup/calibrate_grasp.py``); the three zero-stiffness knuckle joints stay untouched.
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
from .quats import wxyz_to_xyzw, xyzw_to_wxyz
from .robots import DISHWASHER_V0_CFG, UR5E_ROBOTIQ_2F_85_CFG
from .transforms import T_inv, T_to_pos_quat, make_T
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
            "`scripts/run_kit.sh scripts/setup/check_scene.py --headless --measure` and freeze the "
            "printed constant into src/dishsim/config.py first."
        )
    return make_T(config.T_WRIST3_TCP_POS, config.T_WRIST3_TCP_QUAT)


def t_wrist3_obj() -> np.ndarray:
    """Fixed transform wrist_3_link -> carried-object frame (the weld's local pose)."""
    if config.GRASP_TCP_OBJ_POS is None:
        raise RuntimeError(
            "config.GRASP_RIM_TCP_Z_M is not measured yet — run "
            "`scripts/run_kit.sh scripts/setup/calibrate_grasp.py --headless` and freeze the "
            "printed constants into src/dishsim/config.py first."
        )
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
    """World position -> robot-base-frame position (full transform; the base may be yawed).

    Reduces to exact ``pos_w - ROBOT_BASE_POS_W`` at the identity base quaternion.

    Returns:
        Position in the robot-base frame [m], shape [3].
    """
    T_base_w = T_inv(make_T(config.ROBOT_BASE_POS_W, config.ROBOT_BASE_QUAT_W))
    return T_base_w[:3, :3] @ np.asarray(pos_w, dtype=float) + T_base_w[:3, 3]


def base_to_world(pos_b) -> np.ndarray:
    """Robot-base-frame position -> world position (full transform; the base may be yawed).

    Inverse of :func:`world_to_base`; reduces to exact ``pos_b + ROBOT_BASE_POS_W`` at the
    identity base quaternion.

    Returns:
        Position in the world frame [m], shape [3].
    """
    T_w_base = make_T(config.ROBOT_BASE_POS_W, config.ROBOT_BASE_QUAT_W)
    return T_w_base[:3, :3] @ np.asarray(pos_b, dtype=float) + T_w_base[:3, 3]


# ---------------------------------------------------------------------------------------------
# scene construction
# ---------------------------------------------------------------------------------------------


def make_scene_cfg(
    with_object: bool = True, with_robot_contacts: bool = False,
    objects: list | None = None,
) -> InteractiveSceneCfg:
    """Build the v0 scene config (single env).

    Args:
        objects: Heterogeneous manipulable objects to spawn, one dict per item with keys
            ``name``, ``usd_path``, ``pos``, ``quat`` and optionally ``contact_filters``.
            Unlike the active-class ``with_object`` spawn, these may be different classes —
            what a multi-object task needs. Usually combined with ``with_object=False``.
        with_object: Spawn the carried object at the home-configuration grasp pose. Set False
            for the ``--measure`` bootstrap run that derives the TCP rotation constant.
        with_robot_contacts: Add contact sensors over every robot body (arm + gripper) — used
            by the parity check and execution-time contact monitoring.
    """
    robot_joint_pos = dict(zip(config.ARM_JOINTS, config.HOME_Q))
    # gripper: the drive joint starts fully open (the trial-start state — the visible close
    # onto the object is commanded later via ramp_gripper); mimic-driven joints follow
    # physically and are never position-written.
    robot_joint_pos[config.GRIPPER_JOINT] = config.GRIPPER_APERTURE_OPEN_RAD

    robot_cfg = UR5E_ROBOTIQ_2F_85_CFG.replace(
        prim_path="{ENV_REGEX_NS}/Robot",
        init_state=UR5E_ROBOTIQ_2F_85_CFG.init_state.replace(
            pos=config.ROBOT_BASE_POS_W,
            rot=xyzw_to_wxyz(config.ROBOT_BASE_QUAT_W),
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
        # source must NOT share a leaf name with any target: 2.1's FrameTransformer maps
        # frames by leaf body name, and the arm's base_link would swallow the gripper-base
        # ee_tcp target (both leaves are "base_link"). wrist_3_link is unique; only world-
        # frame outputs are consumed, so the source choice is otherwise free.
        ee_frame = FrameTransformerCfg(
            prim_path="{ENV_REGEX_NS}/Robot/wrist_3_link",
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
        # spawned at the home-configuration grasp pose (legacy: scripts 10-15 weld it there)
        pos, quat = grasp_pose_w()
        scene_cfg.carried_object = RigidObjectCfg(
            prim_path="{ENV_REGEX_NS}/CarriedObject",
            spawn=sim_utils.UsdFileCfg(
                usd_path=config.OBJECT_USD,
                rigid_props=sim_utils.RigidBodyPropertiesCfg(max_depenetration_velocity=5.0),
                activate_contact_sensors=True,
            ),
            init_state=RigidObjectCfg.InitialStateCfg(pos=tuple(pos), rot=tuple(xyzw_to_wxyz(quat))),
        )
        # net contact force on the carried object — must stay ~0 while welded (any
        # persistent force means the fingers or the world are touching it). The filter
        # list resolves per-partner forces into force_matrix_w for diagnosis (sensor prim
        # is a single prim per env, so filtering is legal here).
        scene_cfg.object_contact = ContactSensorCfg(
            prim_path="{ENV_REGEX_NS}/CarriedObject",
            update_period=0.0,
            filter_prim_paths_expr=list(_GRIPPER_FILTER_PATHS),
        )

    for spec in objects or ():
        _add_object(scene_cfg, spec, _GRIPPER_FILTER_PATHS)
    return scene_cfg


#: Gripper prims a manipulable object's contact sensor resolves per-partner forces against.
_GRIPPER_FILTER_PATHS = [
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
]


def _add_object(scene_cfg, spec: dict, gripper_filters: list) -> None:
    """Attach one heterogeneous manipulable object to a scene config.

    ``with_object`` spawns the ACTIVE class; a multi-object task needs a mug and a cup and a
    tumbler in one scene, each from its own USD, each with its own contact sensor.

    Args:
        scene_cfg: Scene configclass instance to mutate.
        spec: ``{"name", "usd_path", "pos", "quat"}`` plus optional ``"contact_filters"``
            (extra prim paths to resolve per-partner forces against — an object's peers, so a
            support graph can be read from contacts).
        gripper_filters: Robot prim paths every object filters against.
    """
    name = spec["name"]
    setattr(
        scene_cfg,
        name,
        RigidObjectCfg(
            prim_path="{ENV_REGEX_NS}/" + name,
            spawn=sim_utils.UsdFileCfg(
                usd_path=spec["usd_path"],
                rigid_props=sim_utils.RigidBodyPropertiesCfg(max_depenetration_velocity=5.0),
                activate_contact_sensors=True,
            ),
            init_state=RigidObjectCfg.InitialStateCfg(
                pos=tuple(spec["pos"]), rot=tuple(xyzw_to_wxyz(list(spec["quat"])))
            ),
        ),
    )
    setattr(
        scene_cfg,
        f"{name}_contact",
        ContactSensorCfg(
            prim_path="{ENV_REGEX_NS}/" + name,
            update_period=0.0,
            filter_prim_paths_expr=list(gripper_filters) + list(spec.get("contact_filters", [])),
        ),
    )


# ---------------------------------------------------------------------------------------------
# weld (USD FixedJoint, authored analytically, toggleable at runtime)
# ---------------------------------------------------------------------------------------------


def author_weld(stage, env_path: str = "/World/envs/env_0",
                prim_name: str | None = None, T_wrist3_obj: np.ndarray | None = None) -> str:
    """Author the wrist->object FixedJoint on the live stage BEFORE ``sim.reset()``.

    The joint's local pose on the wrist side is the analytic grasp chain, so after reset (which
    also teleports the object to the matching world pose) the constraint starts with zero
    error. ``physics:excludeFromArticulation`` keeps PhysX from absorbing the object into the
    robot articulation. With multiple objects, author one weld per item (disabled) and only
    ever enable the one being carried.

    Args:
        stage: Live USD stage.
        env_path: Environment prim path.
        prim_name: Explicit object prim name under ``env_path``, overriding the default
            ``CarriedObject``. A multi-object episode spawns one prim per item under its own
            name.
        T_wrist3_obj: Explicit wrist-to-object transform, shape [4, 4]. Required when the
            objects are of DIFFERENT classes: the module-level grasp chain describes only the
            active object, and welding a cup with a mug's transform silently offsets it.
            Defaults to the active object's chain.

    Returns:
        The weld prim path (use with :func:`set_weld_enabled`).
    """
    from pxr import Gf, Sdf, UsdPhysics  # noqa: PLC0415

    name = prim_name if prim_name is not None else "CarriedObject"
    T = t_wrist3_obj() if T_wrist3_obj is None else np.asarray(T_wrist3_obj)
    pos, quat = T_to_pos_quat(T)  # wrist_3_link -> object
    weld_path = f"{env_path}/{name}/{WELD_PRIM_NAME}"
    joint = UsdPhysics.FixedJoint.Define(stage, Sdf.Path(weld_path))
    joint.GetBody0Rel().SetTargets([Sdf.Path(f"{env_path}/Robot/wrist_3_link")])
    joint.GetBody1Rel().SetTargets([Sdf.Path(f"{env_path}/{name}")])
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


#: Newton's-third-law sign relating ``object_contact.force_matrix_w`` rows (force ON the mug
#: FROM each gripper body) to the reaction seen on the gripper bodies' own contact sensors.
#: Verified live by ``scripts/setup/calibrate_grasp.py`` (with the wrong sign the pad residual
#: doubles instead of cancelling — impossible to miss).
GRIP_REACTION_SIGN = -1.0


def _resolve_grasp_aperture() -> float:
    if config.GRIPPER_APERTURE_GRASP_RAD is None:
        raise RuntimeError(
            "config.GRIPPER_APERTURE_GRASP_RAD is not measured yet — run "
            "`scripts/run_kit.sh scripts/setup/calibrate_grasp.py --headless` and freeze the "
            "printed constants into src/dishsim/config.py first."
        )
    return config.GRIPPER_APERTURE_GRASP_RAD


#: persistent rack drive-target override consumed by hold_targets (the drive-synchronized rack
#: slide ramps these each step; the scenario defaults apply when None)
_RACK_TARGET_OVERRIDE: dict[str, float] | None = None


def set_rack_target_override(targets: dict[str, float] | None) -> None:
    """Override the dishwasher rack drive targets that :func:`hold_targets` pins every step.

    The rack-manipulation choreography ramps the moved joint's target through this while the
    gripper visibly engages the rack handle; pass ``None`` to restore the scenario defaults
    (do so only when the scenario's post-action state matches — after a completed rack_action
    the override must stay at the action target for the rest of the trial).
    """
    global _RACK_TARGET_OVERRIDE
    _RACK_TARGET_OVERRIDE = dict(targets) if targets else None


def hold_targets(scene, aperture: float | None = None, arm_q=None) -> None:
    """(Re-)issue the standing position targets: arm, gripper aperture, statics pinned.

    Call after every ``scene.reset()`` (reset clears command buffers) — and it is cheap enough
    to call every step of a settle loop.

    Args:
        aperture: ``finger_joint`` target [rad]; ``None`` uses the calibrated grasp aperture
            (raises until :data:`dishsim.config.GRIPPER_APERTURE_GRASP_RAD` is frozen).
        arm_q: optional 6-vector overriding the arm joints (default: the home pose). Pass the
            executed waypoint here during motion — copying *measured* positions as gripper
            targets would zero the drive torque and silently relax the pinch.
    """
    if aperture is None:
        aperture = _resolve_grasp_aperture()
    robot = scene["robot"]
    targets = robot.data.default_joint_pos.clone()
    if arm_q is not None:
        arm_ids, _ = robot.find_joints(config.ARM_JOINTS, preserve_order=True)
        targets[:, arm_ids] = torch.as_tensor(
            np.asarray(arm_q, dtype=float), dtype=targets.dtype, device=targets.device
        )
    ids, _ = robot.find_joints(config.GRIPPER_JOINT)
    targets[:, ids[0]] = aperture
    # the two stiff (k=10) inner-finger drives must not fight the mimic constraint: command
    # them mimic-consistently once the signs are measured, else follow the measured position
    # (zero spring torque, damping only)
    if_ids, if_names = robot.find_joints(".*_inner_finger_joint", preserve_order=True)
    if config.GRIPPER_INNER_FINGER_SIGNS is not None:
        for jid, jname in zip(if_ids, if_names):
            targets[:, jid] = config.GRIPPER_INNER_FINGER_SIGNS[jname] * aperture
    else:
        targets[:, if_ids] = robot.data.joint_pos[:, if_ids]
    robot.set_joint_position_target(target=targets)
    dw = scene["dishwasher"]
    dw_targets = dw.data.default_joint_pos.clone()
    if _RACK_TARGET_OVERRIDE:
        for jname, val in _RACK_TARGET_OVERRIDE.items():
            jids, _ = dw.find_joints(jname)
            dw_targets[:, jids[0]] = float(val)
    dw.set_joint_position_target(target=dw_targets)


def ramp_gripper(
    scene, sim, end_aperture: float, steps: int, arm_q=None, per_step=None, step_fn=None
) -> float:
    """Linearly ramp the ``finger_joint`` target to ``end_aperture`` — the visible close/open.

    Starts from the current *measured* finger position and calls :func:`hold_targets` with the
    interpolated target every physics step, so the motion is smooth on camera and the squeeze
    stays commanded throughout.

    Args:
        end_aperture: final ``finger_joint`` target [rad].
        steps: physics steps for the ramp (e.g. :data:`dishsim.config.GRIPPER_CLOSE_RAMP_STEPS`).
        arm_q: optional arm override passed through to :func:`hold_targets`.
        per_step: optional callback invoked after every step (e.g. video capture).
        step_fn: optional single-step function replacing the built-in
            write/step/update sequence. Callers that record trajectories pass their own
            stepper here so every physics step in a trial goes through one place and none can
            be missed.

    Returns:
        The measured ``finger_joint`` position [rad] after the ramp.
    """
    robot = scene["robot"]
    ids, _ = robot.find_joints(config.GRIPPER_JOINT)
    dt = sim.get_physics_dt()
    start = float(robot.data.joint_pos[0, ids[0]])
    for i in range(steps):
        frac = (i + 1) / steps
        hold_targets(scene, aperture=start + frac * (end_aperture - start), arm_q=arm_q)
        if step_fn is None:
            scene.write_data_to_sim()
            sim.step()
            scene.update(dt)
        else:
            step_fn(1)
        if per_step is not None:
            per_step(i)
    return float(robot.data.joint_pos[0, ids[0]])


def object_contact_partners(scene) -> list[str]:
    """Basenames of the ``object_contact`` filter bodies, in ``force_matrix_w`` row order."""
    return [p.rsplit("/", 1)[-1] for p in scene["object_contact"].cfg.filter_prim_paths_expr]


def grip_forces(scene) -> dict:
    """Per-partner contact forces on the carried object, plus the external residual.

    Returns:
        Dict with ``partners`` (body name -> force magnitude [N]), ``net_n`` (net force [N]),
        ``pads_n`` (pad magnitudes [N], :data:`dishsim.config.GRIP_PAD_BODIES` order), and
        ``external_n`` (net minus summed partner forces [N] — the mug's non-gripper contact
        estimate, ~0 unless the mug touches the world).
    """
    sensor = scene["object_contact"]
    names = object_contact_partners(scene)
    net_vec = sensor.data.net_forces_w[0].reshape(-1, 3).sum(dim=0)
    fm = sensor.data.force_matrix_w[0].reshape(-1, 3)
    partners = {n: float(m) for n, m in zip(names, fm.norm(dim=-1))}
    for pad in config.GRIP_PAD_BODIES:
        assert pad in partners, f"pad body {pad} missing from the object_contact filter list"
    return {
        "partners": partners,
        "net_n": float(net_vec.norm()),
        "pads_n": [partners[p] for p in config.GRIP_PAD_BODIES],
        "external_n": float((net_vec - fm.sum(dim=0)).norm()),
    }


def grip_gate(scene, during_motion: bool = False) -> tuple[bool, str]:
    """Check the calibrated grip-force band while the object is gripped.

    Static (default): each pad force within ``[GRIP_FORCE_MIN_N, GRIP_FORCE_MAX_N]``. During
    motion: pads bounded only above by ``GRIP_FORCE_EXEC_MAX_N`` (transients dip below the
    static band without meaning the grasp failed). Always: every non-pad partner and the
    external residual stay under ``CONTACT_FORCE_THRESH_N``.

    Returns:
        (ok, detail) — detail names the first violated condition.
    """
    f = grip_forces(scene)
    lo, hi = (
        (0.0, config.GRIP_FORCE_EXEC_MAX_N)
        if during_motion
        else (config.GRIP_FORCE_MIN_N, config.GRIP_FORCE_MAX_N)
    )
    for pad, mag in zip(config.GRIP_PAD_BODIES, f["pads_n"]):
        if not lo <= mag <= hi:
            return False, f"{pad} force {mag:.2f} N outside [{lo:.1f}, {hi:.1f}] N"
    for name, mag in f["partners"].items():
        if name not in config.GRIP_PAD_BODIES and mag > config.CONTACT_FORCE_THRESH_N:
            return False, f"non-pad partner {name} touches the object ({mag:.2f} N)"
    if f["external_n"] > config.CONTACT_FORCE_THRESH_N:
        return False, f"external (non-gripper) residual {f['external_n']:.2f} N"
    return True, ""


def unexpected_robot_contact(scene, sensors, sensor_names, exclude=(),
                             object_key: str | None = None) -> tuple[float, str]:
    """Peak *unexpected* contact force over the robot-body sensors.

    The two pad bodies are checked on their residual after subtracting the expected carried-
    object reaction (read from that object's ``force_matrix_w``); every other body reports its
    raw net force. Bodies in ``exclude`` plus :data:`dishsim.config.PARITY_BODY_EXCLUDE` are
    skipped.

    Args:
        scene: Live interactive scene.
        sensors: Robot-body contact sensors.
        sensor_names: Body names per sensor.
        exclude: Robot bodies whose contact is expected.
        object_key: Explicit scene key of the carried object's contact sensor, overriding the
            default ``object_contact``. A multi-object scene names sensors after their item,
            so without this the pad reaction is never found and the gripper's own grip on the
            object it is carrying is reported as an unexpected collision.

    Returns:
        (peak force [N], body name).
    """
    pad_rows = {}
    key = object_key if object_key is not None else "object_contact"
    if key in scene.keys():
        sensor_obj = scene[key]
        if sensor_obj.data.force_matrix_w is not None:
            names = [p.rsplit("/", 1)[-1] for p in sensor_obj.cfg.filter_prim_paths_expr]
            fm = sensor_obj.data.force_matrix_w[0].reshape(-1, 3)
            for pad in config.GRIP_PAD_BODIES:
                if pad in names:
                    pad_rows[pad] = fm[names.index(pad)]
    skip = set(config.PARITY_BODY_EXCLUDE) | set(exclude)
    peak, peak_body = 0.0, ""
    for si, sensor in enumerate(sensors):
        forces = sensor.data.net_forces_w[0]
        for bi in range(forces.shape[0]):
            name = sensor_names[si][bi] if bi < len(sensor_names[si]) else f"s{si}b{bi}"
            if name in skip:
                continue
            vec = forces[bi]
            if name in pad_rows:
                vec = vec - GRIP_REACTION_SIGN * pad_rows[name]
            mag = float(vec.norm())
            if mag > peak:
                peak, peak_body = mag, name
    return peak, peak_body


def write_default_states(scene, aperture: float | None = None) -> None:
    """Standalone-script reset dance: write default root/joint states, reset, re-arm targets."""
    for name in ("robot", "dishwasher"):
        art = scene[name]
        root_pose = art.data.default_root_state[:, :7].clone()
        root_pose[:, :3] += scene.env_origins
        art.write_root_pose_to_sim(root_pose=root_pose)
        art.write_root_velocity_to_sim(root_velocity=art.data.default_root_state[:, 7:].clone())
        art.write_joint_position_to_sim(position=art.data.default_joint_pos.clone())
        art.write_joint_velocity_to_sim(velocity=art.data.default_joint_vel.clone())
    if "carried_object" in scene.keys():
        obj = scene["carried_object"]
        root_pose = obj.data.default_root_state[:, :7].clone()
        root_pose[:, :3] += scene.env_origins
        obj.write_root_pose_to_sim(root_pose=root_pose)
        obj.write_root_velocity_to_sim(root_velocity=obj.data.default_root_state[:, 7:].clone())
    # scene.reset() clears command buffers — targets must be re-armed AFTER it (a finger target
    # set before reset silently reverts to the open pose; found the hard way during scene bring-up)
    scene.reset()
    hold_targets(scene, aperture=aperture)


def assert_frames(scene) -> None:
    """Assert the single frame convention this whole project relies on."""
    base_pos = scene["robot"].data.root_pos_w[0].cpu().numpy()
    base_quat = wxyz_to_xyzw(scene["robot"].data.root_quat_w[0].cpu().numpy())
    assert np.allclose(base_pos, config.ROBOT_BASE_POS_W, atol=1e-4), f"robot base at {base_pos}"
    # sign-agnostic: q and -q are the same rotation, and the sim may hand back either sign
    assert np.allclose(base_quat, config.ROBOT_BASE_QUAT_W, atol=1e-4) or np.allclose(
        -base_quat, config.ROBOT_BASE_QUAT_W, atol=1e-4
    ), (
        f"robot base rotation {base_quat} != config.ROBOT_BASE_QUAT_W "
        f"{config.ROBOT_BASE_QUAT_W} — the robot-base frame convention is broken"
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


def measure_tcp(scene) -> dict:
    """Live wrist_3 -> TCP transform (the constant frozen into config after ``--measure``)."""
    robot = scene["robot"]
    w3_ids, _ = robot.find_bodies("wrist_3_link")
    w3_pos = robot.data.body_link_pos_w[0, w3_ids[0]].cpu().numpy()
    w3_quat = wxyz_to_xyzw(robot.data.body_link_quat_w[0, w3_ids[0]].cpu().numpy())
    tcp_pos = scene["ee_frame"].data.target_pos_w[0, 0].cpu().numpy()
    tcp_quat = wxyz_to_xyzw(scene["ee_frame"].data.target_quat_w[0, 0].cpu().numpy())
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
