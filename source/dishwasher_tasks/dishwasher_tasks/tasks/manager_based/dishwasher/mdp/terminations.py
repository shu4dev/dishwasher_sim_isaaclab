# Copyright (c) 2026, dishwasher_tasks project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Termination terms for the dishwasher door-opening task."""

from __future__ import annotations

from typing import TYPE_CHECKING

import torch

from isaaclab.managers import ManagerTermBase, SceneEntityCfg, TerminationTermCfg

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv


class door_open_sustained(ManagerTermBase):
    """Success when the door angle stays above ``threshold`` for ``num_steps`` consecutive steps."""

    def __init__(self, cfg: TerminationTermCfg, env: ManagerBasedRLEnv):
        super().__init__(cfg, env)
        self._counter = torch.zeros(env.num_envs, dtype=torch.long, device=env.device)

    def reset(self, env_ids: torch.Tensor):
        self._counter[env_ids] = 0

    def __call__(
        self, env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg, threshold: float, num_steps: int
    ) -> torch.Tensor:
        door_pos = env.scene[asset_cfg.name].data.joint_pos.torch[:, asset_cfg.joint_ids[0]]
        above = door_pos > threshold
        self._counter = torch.where(above, self._counter + 1, torch.zeros_like(self._counter))
        return self._counter >= num_steps
