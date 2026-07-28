# Copyright (c) 2026, dishwasher_tasks project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""UR5e + Robotiq 2F-85 configuration for the dishwasher door-opening task."""

from isaaclab.sensors import FrameTransformerCfg
from isaaclab.sensors.frame_transformer.frame_transformer_cfg import OffsetCfg
from isaaclab.utils.configclass import configclass

from dishwasher_tasks.robots.ur5e_robotiq_2f85 import UR5E_ROBOTIQ_2F_85_CFG
from dishwasher_tasks.tasks.manager_based.dishwasher import mdp
from dishwasher_tasks.tasks.manager_based.dishwasher.open_dishwasher_env_cfg import OpenDishwasherEnvCfg


@configclass
class UR5eOpenDishwasherEnvCfg(OpenDishwasherEnvCfg):
    def __post_init__(self):
        # post init of parent
        super().__post_init__()

        # robot on the pedestal (base at 0.25 m); elbow-up home pose facing the dishwasher
        self.scene.robot = UR5E_ROBOTIQ_2F_85_CFG.replace(
            prim_path="{ENV_REGEX_NS}/Robot",
            init_state=UR5E_ROBOTIQ_2F_85_CFG.init_state.replace(pos=(0.0, 0.0, 0.25)),
        )

        # actions: the 6 arm joints (position control) + binary gripper on the drive joint only.
        # The other five gripper joints follow through PhysX mimic constraints and stay out of
        # the action space.
        self.actions.arm_action = mdp.JointPositionActionCfg(
            asset_name="robot",
            joint_names=[
                "shoulder_pan_joint",
                "shoulder_lift_joint",
                "elbow_joint",
                "wrist_1_joint",
                "wrist_2_joint",
                "wrist_3_joint",
            ],
            # 0.5 rad per unit action: the UR5e arm gains are stiff (k up to 1320) and full
            # +-1 rad position swings at 60 Hz whip the wrist hard enough to destabilize the
            # gripper's mimic-joint cluster
            scale=0.5,
            use_default_offset=True,
        )
        self.actions.gripper_action = mdp.BinaryJointPositionActionCfg(
            asset_name="robot",
            joint_names=["finger_joint"],
            open_command_expr={"finger_joint": 0.0},
            close_command_expr={"finger_joint": 0.785},
        )

        # End-effector frames. IMPORTANT: order matters — [0] is the TCP, [1:] the fingertips
        # (consumed by the grasp rewards). Offsets measured from the asset (docs/joint_report.md):
        # the gripper links are authored coincident with the mount frame; the pad geometry sits
        # at z ≈ 0.12 along the tool axis with ∓0.059 m lateral offset when open.
        self.scene.ee_frame = FrameTransformerCfg(
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

        # robot-specific reward parameters (2F-85: finger_joint 0 = open, ~0.8 rad = closed)
        self.rewards.approach_gripper_handle.params["offset"] = 0.02
        self.rewards.grasp_handle.params["open_joint_pos"] = 0.0
        self.rewards.grasp_handle.params["asset_cfg"].joint_names = ["finger_joint"]


@configclass
class UR5eOpenDishwasherEnvCfg_PLAY(UR5eOpenDishwasherEnvCfg):
    def __post_init__(self):
        # post init of parent
        super().__post_init__()
        # make a smaller scene for play
        self.scene.num_envs = 16
        self.scene.env_spacing = 3.0
        # disable randomization for play
        self.observations.policy.enable_corruption = False
