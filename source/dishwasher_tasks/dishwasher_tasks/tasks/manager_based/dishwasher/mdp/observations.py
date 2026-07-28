# Copyright (c) 2026, dishwasher_tasks project.
#
# SPDX-License-Identifier: BSD-3-Clause

from __future__ import annotations

from typing import TYPE_CHECKING

import torch

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv
    from isaaclab.sensors import FrameTransformerData


def rel_ee_handle_distance(env: ManagerBasedRLEnv) -> torch.Tensor:
    """Vector from the end-effector TCP to the dishwasher door handle [m]."""
    ee_tf_data: FrameTransformerData = env.scene["ee_frame"].data
    handle_tf_data: FrameTransformerData = env.scene["handle_frame"].data

    return handle_tf_data.target_pos_w.torch[..., 0, :] - ee_tf_data.target_pos_w.torch[..., 0, :]
