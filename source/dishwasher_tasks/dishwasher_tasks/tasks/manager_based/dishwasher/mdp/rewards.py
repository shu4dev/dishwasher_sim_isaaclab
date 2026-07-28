# Copyright (c) 2026, dishwasher_tasks project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Reward terms for the dishwasher door-opening task.

Ported from the in-tree cabinet task (``isaaclab_tasks/.../cabinet/mdp/rewards.py``) with the
drawer replaced by a hinged door (door joint position is an angle [rad]) and the Franka gripper
replaced by a Robotiq 2F-85, whose ``finger_joint`` convention is inverted vs. the Franka fingers:
**0 = open, ~0.8 rad = closed** — see :func:`grasp_handle`.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

import torch

from isaaclab.managers import ManagerTermBase, RewardTermCfg, SceneEntityCfg
from isaaclab.utils.math import matrix_from_quat

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv


def approach_ee_handle(env: ManagerBasedRLEnv, threshold: float) -> torch.Tensor:
    r"""Reward the robot for reaching the door handle using an inverse-square law.

    .. math::

        reward = \begin{cases}
            2 * (1 / (1 + distance^2))^2 & \text{if } distance \leq threshold \\
            (1 / (1 + distance^2))^2 & \text{otherwise}
        \end{cases}
    """
    ee_tcp_pos = env.scene["ee_frame"].data.target_pos_w.torch[..., 0, :]
    handle_pos = env.scene["handle_frame"].data.target_pos_w.torch[..., 0, :]

    distance = torch.linalg.norm(handle_pos - ee_tcp_pos, dim=-1, ord=2)
    reward = 1.0 / (1.0 + distance**2)
    reward = torch.pow(reward, 2)
    return torch.where(distance <= threshold, 2 * reward, reward)


def align_ee_handle(env: ManagerBasedRLEnv) -> torch.Tensor:
    """Reward for aligning the end-effector with the handle.

    The handle frame is authored so that its x axis points out of the door (toward the robot when
    the door is closed) and its y axis runs along the handle bar. Alignment rewards the gripper
    approach axis (z) opposing the handle x axis and the gripper x axis opposing the handle y.
    """
    ee_tcp_quat = env.scene["ee_frame"].data.target_quat_w.torch[..., 0, :]
    handle_quat = env.scene["handle_frame"].data.target_quat_w.torch[..., 0, :]

    ee_tcp_rot_mat = matrix_from_quat(ee_tcp_quat)
    handle_mat = matrix_from_quat(handle_quat)

    handle_x, handle_y = handle_mat[..., 0], handle_mat[..., 1]
    ee_tcp_x, ee_tcp_z = ee_tcp_rot_mat[..., 0], ee_tcp_rot_mat[..., 2]

    align_z = torch.bmm(ee_tcp_z.unsqueeze(1), -handle_x.unsqueeze(-1)).squeeze(-1).squeeze(-1)
    align_x = torch.bmm(ee_tcp_x.unsqueeze(1), -handle_y.unsqueeze(-1)).squeeze(-1).squeeze(-1)
    return 0.5 * (torch.sign(align_z) * align_z**2 + torch.sign(align_x) * align_x**2)


def align_grasp_around_handle(env: ManagerBasedRLEnv) -> torch.Tensor:
    """Bonus for a graspable hand pose: one fingertip above the handle bar, the other below."""
    handle_pos = env.scene["handle_frame"].data.target_pos_w.torch[..., 0, :]
    ee_fingertips_w = env.scene["ee_frame"].data.target_pos_w.torch[..., 1:, :]
    lfinger_pos = ee_fingertips_w[..., 0, :]
    rfinger_pos = ee_fingertips_w[..., 1, :]

    is_graspable = (rfinger_pos[:, 2] < handle_pos[:, 2]) & (lfinger_pos[:, 2] > handle_pos[:, 2])
    return is_graspable


def approach_gripper_handle(env: ManagerBasedRLEnv, offset: float = 0.02) -> torch.Tensor:
    """Reward the fingertips closing on the handle bar while in a graspable pose."""
    handle_pos = env.scene["handle_frame"].data.target_pos_w.torch[..., 0, :]
    ee_fingertips_w = env.scene["ee_frame"].data.target_pos_w.torch[..., 1:, :]
    lfinger_pos = ee_fingertips_w[..., 0, :]
    rfinger_pos = ee_fingertips_w[..., 1, :]

    lfinger_dist = torch.abs(lfinger_pos[:, 2] - handle_pos[:, 2])
    rfinger_dist = torch.abs(rfinger_pos[:, 2] - handle_pos[:, 2])
    is_graspable = (rfinger_pos[:, 2] < handle_pos[:, 2]) & (lfinger_pos[:, 2] > handle_pos[:, 2])

    return is_graspable * ((offset - lfinger_dist) + (offset - rfinger_dist))


def grasp_handle(
    env: ManagerBasedRLEnv, threshold: float, open_joint_pos: float, asset_cfg: SceneEntityCfg
) -> torch.Tensor:
    """Reward for closing the gripper when the TCP is close to the handle.

    Note:
        The Robotiq 2F-85 ``finger_joint`` is **0 = open** and grows toward ~0.8 rad when closed —
        the opposite of the Franka fingers in the cabinet task — so this rewards
        ``joint_pos - open_joint_pos`` (positive when closing).
    """
    ee_tcp_pos = env.scene["ee_frame"].data.target_pos_w.torch[..., 0, :]
    handle_pos = env.scene["handle_frame"].data.target_pos_w.torch[..., 0, :]
    gripper_joint_pos = env.scene[asset_cfg.name].data.joint_pos.torch[:, asset_cfg.joint_ids]

    distance = torch.linalg.norm(handle_pos - ee_tcp_pos, dim=-1, ord=2)
    is_close = distance <= threshold

    return is_close * torch.sum(gripper_joint_pos - open_joint_pos, dim=-1)


class open_door_bonus(ManagerTermBase):
    """Bonus for opening the door, scaled by the door joint angle [rad].

    Doubled while the grasp is around the handle. Also tracks per-episode success (door ever
    opened past ``success_threshold``) and logs ``Metrics/success_rate`` on reset.
    """

    def __init__(self, cfg: RewardTermCfg, env: ManagerBasedRLEnv):
        super().__init__(cfg, env)
        self._track_success = cfg.params.get("success_threshold") is not None
        if self._track_success:
            self.succeeded = torch.zeros(env.num_envs, dtype=torch.bool, device=env.device)

    def reset(self, env_ids: torch.Tensor):
        if self._track_success:
            self._env.extras.setdefault("log", {})["Metrics/success_rate"] = (
                self.succeeded[env_ids].float().mean().item()
            )
            self.succeeded[env_ids] = False

    def __call__(
        self, env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg, success_threshold: float | None = None
    ) -> torch.Tensor:
        door_pos = env.scene[asset_cfg.name].data.joint_pos.torch[:, asset_cfg.joint_ids[0]]
        is_graspable = align_grasp_around_handle(env).float()
        if success_threshold is not None:
            self.succeeded |= door_pos > success_threshold
        return (is_graspable + 1.0) * door_pos


def multi_stage_open_door(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg) -> torch.Tensor:
    """Multi-stage bonus for opening the door (angles in rad: cracked > 3 deg, 30 deg, 60 deg)."""
    door_pos = env.scene[asset_cfg.name].data.joint_pos.torch[:, asset_cfg.joint_ids[0]]
    is_graspable = align_grasp_around_handle(env).float()

    open_easy = (door_pos > 0.05) * 0.5
    open_medium = (door_pos > 0.52) * is_graspable
    open_hard = (door_pos > 1.05) * is_graspable

    return open_easy + open_medium + open_hard
