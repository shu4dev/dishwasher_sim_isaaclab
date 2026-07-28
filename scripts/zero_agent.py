# Copyright (c) 2026, dishwasher_tasks project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Zero-action agent for the dishwasher tasks (boot-first pattern, headless-friendly).

Usage:
    /workspace/isaaclab/isaaclab.sh -p scripts/zero_agent.py \
        --task Isaac-Open-Dishwasher-UR5e-v0 --num_envs 4 --headless
"""

"""Launch Isaac Sim Simulator first."""

import argparse

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Zero agent for the dishwasher tasks.")
parser.add_argument("--task", type=str, required=True, help="Name of the task.")
parser.add_argument("--num_envs", type=int, default=None, help="Number of environments to simulate.")
parser.add_argument("--steps", type=int, default=300, help="Number of env steps to run.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""Rest everything follows."""

import gymnasium as gym
import torch

from isaaclab_tasks.utils.hydra import resolve_presets
from isaaclab_tasks.utils.parse_cfg import load_cfg_from_registry

import dishwasher_tasks.tasks  # noqa: F401


def main():
    """Zero-actions agent with the dishwasher environment."""
    torch.manual_seed(42)

    env_cfg = load_cfg_from_registry(args_cli.task, "env_cfg_entry_point")
    env_cfg = resolve_presets(env_cfg)
    if args_cli.num_envs is not None:
        env_cfg.scene.num_envs = args_cli.num_envs
    if args_cli.device is not None:
        env_cfg.sim.device = args_cli.device

    env = gym.make(args_cli.task, cfg=env_cfg)
    print(f"[INFO] Gym observation space: {env.observation_space}")
    print(f"[INFO] Gym action space: {env.action_space}")
    env.reset()

    actions = torch.zeros(env.action_space.shape, device=env.unwrapped.device)
    for _ in range(args_cli.steps):
        with torch.inference_mode():
            env.step(actions)
    print(f"[INFO] Ran {args_cli.steps} steps with zero actions without error.")
    env.close()


if __name__ == "__main__":
    main()
    simulation_app.close()
