# Copyright (c) 2026, dishwasher_tasks project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Robot-agnostic base configuration for the dishwasher door-opening task.

Modeled on the in-tree cabinet task (``isaaclab_tasks/.../manipulation/cabinet``): the scene keeps
``robot`` and ``ee_frame`` as :obj:`~dataclasses.MISSING` so robot-specific configs (see
``config/ur5e``) fill them in. All dishwasher-side names and offsets come from the measured
values in ``docs/joint_report.md`` / ``docs/asset_survey.md``.

Scene geometry: the robot base sits on a 0.25 m pedestal at the origin; the dishwasher stands
0.85 m in front (front face at x ≈ 0.65, facing the robot), so the closed-door handle is ~0.65 m
from the robot base (UR5e reach ≈ 0.85 m) and the fully open door tip (x ≈ 0.27) clears the
pedestal edge (x = 0.125) by ~0.14 m.
"""

from dataclasses import MISSING

import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg, AssetBaseCfg
from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sensors import FrameTransformerCfg
from isaaclab.sensors.frame_transformer.frame_transformer_cfg import OffsetCfg
from isaaclab.sim import SimulationCfg
from isaaclab.utils.configclass import configclass
from isaaclab_physx.physics import PhysxCfg

from isaaclab_tasks.utils import PresetCfg

from dishwasher_tasks.robots.dishwasher import DISHWASHER_CFG, DISHWASHER_DOOR_JOINT

from . import mdp

##
# Simulation presets
##


@configclass
class DishwasherSimCfg(PresetCfg):
    """PhysX-only simulation presets.

    The launcher machinery (``launch_simulation`` / preset CLI) expects the ``sim`` field to be a
    :class:`~isaaclab_tasks.utils.PresetCfg` wrapper — a plain :class:`~isaaclab.sim.SimulationCfg`
    crashes the Kit boot in this Isaac Lab version.
    """

    default: SimulationCfg = SimulationCfg(
        dt=1 / 60,
        render_interval=1,
        physics=PhysxCfg(bounce_threshold_velocity=0.01, friction_correlation_distance=0.00625),
    )
    physx: SimulationCfg = SimulationCfg(
        dt=1 / 60,
        render_interval=1,
        physics=PhysxCfg(bounce_threshold_velocity=0.01, friction_correlation_distance=0.00625),
    )


##
# Scene definition
##


@configclass
class DishwasherSceneCfg(InteractiveSceneCfg):
    """Scene with the dishwasher, a pedestal-mounted robot, ground plane, and light.

    The robot and its end-effector frame are populated by the robot-specific env configs.
    """

    # robot and end-effector frame: populated by robot-specific config
    robot: ArticulationCfg = MISSING
    ee_frame: FrameTransformerCfg = MISSING

    # NOTE: at spawn the ROOT LINK frame (body E_body_5, at the machine's rear corner) is placed
    # at this pose — not the asset origin. This position puts the closed-door handle at
    # (0.64, 0.0, 0.36) in world, i.e. 0.65 m from the robot base, with the front face at
    # x ≈ 0.63 and the fully open door tip at x ≈ 0.27 (pedestal edge: 0.125).
    dishwasher = DISHWASHER_CFG.replace(
        prim_path="{ENV_REGEX_NS}/Dishwasher",
        init_state=DISHWASHER_CFG.init_state.replace(
            pos=(0.6498, 0.2423, 0.0),
            rot=(0.0, 0.0, -0.70710678, 0.70710678),  # yaw -90 deg: front face toward the robot
        ),
    )

    # handle frame on the door body: offset measured in the door hinge frame; x out of the door,
    # y along the handle bar (see docs/joint_report.md)
    handle_frame = FrameTransformerCfg(
        prim_path="{ENV_REGEX_NS}/Dishwasher/E_body_5",
        debug_vis=False,
        target_frames=[
            FrameTransformerCfg.FrameCfg(
                prim_path="{ENV_REGEX_NS}/Dishwasher/E_door_4",
                name="door_handle",
                offset=OffsetCfg(pos=(-0.0058, -0.0100, 0.3487), rot=(0.0, 0.0, -0.70710678, 0.70710678)),
            ),
        ],
    )

    pedestal = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/Pedestal",
        spawn=sim_utils.CuboidCfg(
            size=(0.25, 0.25, 0.25),
            collision_props=sim_utils.CollisionPropertiesCfg(),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.3, 0.3, 0.3)),
        ),
        init_state=AssetBaseCfg.InitialStateCfg(pos=(0.0, 0.0, 0.125)),
    )

    plane = AssetBaseCfg(
        prim_path="/World/GroundPlane",
        init_state=AssetBaseCfg.InitialStateCfg(),
        spawn=sim_utils.GroundPlaneCfg(),
        collision_group=-1,
    )

    light = AssetBaseCfg(
        prim_path="/World/light",
        spawn=sim_utils.DomeLightCfg(color=(0.75, 0.75, 0.75), intensity=3000.0),
    )


##
# MDP settings
##


@configclass
class ActionsCfg:
    """Action specifications for the MDP."""

    arm_action: mdp.JointPositionActionCfg = MISSING
    gripper_action: mdp.BinaryJointPositionActionCfg = MISSING


@configclass
class ObservationsCfg:
    """Observation specifications for the MDP."""

    @configclass
    class PolicyCfg(ObsGroup):
        """Observations for policy group."""

        joint_pos = ObsTerm(func=mdp.joint_pos_rel)
        joint_vel = ObsTerm(func=mdp.joint_vel_rel)
        door_joint_pos = ObsTerm(
            func=mdp.joint_pos,
            params={"asset_cfg": SceneEntityCfg("dishwasher", joint_names=[DISHWASHER_DOOR_JOINT])},
        )
        door_joint_vel = ObsTerm(
            func=mdp.joint_vel,
            params={"asset_cfg": SceneEntityCfg("dishwasher", joint_names=[DISHWASHER_DOOR_JOINT])},
        )
        rel_ee_handle = ObsTerm(func=mdp.rel_ee_handle_distance)
        actions = ObsTerm(func=mdp.last_action)

        def __post_init__(self):
            self.enable_corruption = True
            self.concatenate_terms = True

    policy: PolicyCfg = PolicyCfg()


@configclass
class EventCfg:
    """Configuration for events."""

    reset_all = EventTerm(func=mdp.reset_scene_to_default, mode="reset")

    reset_robot_joints = EventTerm(
        func=mdp.reset_joints_by_offset,
        mode="reset",
        params={
            "position_range": (-0.1, 0.1),
            "velocity_range": (0.0, 0.0),
        },
    )


@configclass
class RewardsCfg:
    """Reward terms for the MDP."""

    # 1. Approach the handle
    approach_ee_handle = RewTerm(func=mdp.approach_ee_handle, weight=2.0, params={"threshold": 0.2})
    align_ee_handle = RewTerm(func=mdp.align_ee_handle, weight=0.5)

    # 2. Grasp the handle
    approach_gripper_handle = RewTerm(func=mdp.approach_gripper_handle, weight=5.0, params={"offset": MISSING})
    align_grasp_around_handle = RewTerm(func=mdp.align_grasp_around_handle, weight=0.125)
    grasp_handle = RewTerm(
        func=mdp.grasp_handle,
        weight=0.5,
        params={
            "threshold": 0.03,
            "open_joint_pos": MISSING,
            "asset_cfg": SceneEntityCfg("robot", joint_names=MISSING),
        },
    )

    # 3. Open the door (angle in rad; success past 60 deg)
    open_door_bonus = RewTerm(
        func=mdp.open_door_bonus,
        weight=7.5,
        params={
            "asset_cfg": SceneEntityCfg("dishwasher", joint_names=[DISHWASHER_DOOR_JOINT]),
            "success_threshold": 1.047,
        },
    )
    multi_stage_open_door = RewTerm(
        func=mdp.multi_stage_open_door,
        weight=1.0,
        params={"asset_cfg": SceneEntityCfg("dishwasher", joint_names=[DISHWASHER_DOOR_JOINT])},
    )

    # 4. Penalize actions for cosmetic reasons
    action_rate_l2 = RewTerm(func=mdp.action_rate_l2, weight=-1e-2)
    joint_vel = RewTerm(func=mdp.joint_vel_l2, weight=-0.0001, params={"asset_cfg": SceneEntityCfg("robot")})


@configclass
class TerminationsCfg:
    """Termination terms for the MDP."""

    time_out = DoneTerm(func=mdp.time_out, time_out=True)

    success = DoneTerm(
        func=mdp.door_open_sustained,
        params={
            "asset_cfg": SceneEntityCfg("dishwasher", joint_names=[DISHWASHER_DOOR_JOINT]),
            "threshold": 1.047,
            "num_steps": 50,
        },
    )


##
# Environment configuration
##


@configclass
class OpenDishwasherEnvCfg(ManagerBasedRLEnvCfg):
    """Configuration for the dishwasher door-opening environment."""

    sim: DishwasherSimCfg = DishwasherSimCfg()
    scene: DishwasherSceneCfg = DishwasherSceneCfg(num_envs=512, env_spacing=3.0)
    observations: ObservationsCfg = ObservationsCfg()
    actions: ActionsCfg = ActionsCfg()
    rewards: RewardsCfg = RewardsCfg()
    terminations: TerminationsCfg = TerminationsCfg()
    events: EventCfg = EventCfg()

    def __post_init__(self):
        """Post initialization."""
        self.decimation = 1
        self.episode_length_s = 10.0
        self.viewer.eye = (-1.5, -1.5, 1.5)
        self.viewer.lookat = (0.6, 0.0, 0.4)
