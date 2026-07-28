# Copyright (c) 2026, dishwasher_tasks project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Train an RSL-RL PPO agent on the dishwasher tasks.

This script boots the simulator app *before* importing or resolving any task configuration
(the "boot-first" pattern). The stock template scripts resolve the env config and scan it in
``launch_simulation`` before Kit boots, which crashes natively (``free(): invalid pointer``)
for this project's config on this Isaac Lab build — see docs/environment.md.

Usage:
    /workspace/isaaclab/isaaclab.sh -p scripts/rsl_rl/train.py \
        --task Isaac-Open-Dishwasher-UR5e-v0 --num_envs 512 --headless
"""

"""Launch Isaac Sim Simulator first."""

import argparse

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Train an RSL-RL agent on a dishwasher task.")
parser.add_argument("--task", type=str, required=True, help="Name of the task.")
parser.add_argument("--num_envs", type=int, default=None, help="Number of environments to simulate.")
parser.add_argument("--max_iterations", type=int, default=None, help="RL policy training iterations.")
parser.add_argument("--seed", type=int, default=None, help="Seed used for the environment.")
parser.add_argument("--resume", action="store_true", default=False, help="Resume from the latest checkpoint.")
parser.add_argument("--load_run", type=str, default=None, help="Run folder to resume from (regex).")
parser.add_argument("--checkpoint", type=str, default=None, help="Checkpoint file to resume from (regex).")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""Rest everything follows."""

import importlib.metadata as metadata
import os
import time
from datetime import datetime

import gymnasium as gym
from rsl_rl.runners import OnPolicyRunner

from isaaclab.utils.io import dump_yaml

from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper, handle_deprecated_rsl_rl_cfg

from isaaclab_tasks.utils import get_checkpoint_path
from isaaclab_tasks.utils.hydra import resolve_presets
from isaaclab_tasks.utils.parse_cfg import load_cfg_from_registry

import dishwasher_tasks.tasks  # noqa: F401


def main():
    """Train with an RSL-RL agent."""
    # resolve configurations (post-boot on purpose; presets collapse to their defaults)
    env_cfg = load_cfg_from_registry(args_cli.task, "env_cfg_entry_point")
    env_cfg = resolve_presets(env_cfg)
    agent_cfg = load_cfg_from_registry(args_cli.task, "rsl_rl_cfg_entry_point")
    agent_cfg = handle_deprecated_rsl_rl_cfg(agent_cfg, metadata.version("rsl-rl-lib"))

    # CLI overrides
    if args_cli.num_envs is not None:
        env_cfg.scene.num_envs = args_cli.num_envs
    if args_cli.max_iterations is not None:
        agent_cfg.max_iterations = args_cli.max_iterations
    if args_cli.seed is not None:
        agent_cfg.seed = args_cli.seed
    if args_cli.device is not None:
        env_cfg.sim.device = args_cli.device
    if args_cli.resume:
        agent_cfg.resume = True
    if args_cli.load_run is not None:
        agent_cfg.load_run = args_cli.load_run
    if args_cli.checkpoint is not None:
        agent_cfg.load_checkpoint = args_cli.checkpoint
    env_cfg.seed = agent_cfg.seed

    # logging directory: logs/rsl_rl/<experiment>/<timestamp> (relative to the current directory)
    log_root_path = os.path.abspath(os.path.join("logs", "rsl_rl", agent_cfg.experiment_name))
    log_dir = os.path.join(log_root_path, datetime.now().strftime("%Y-%m-%d_%H-%M-%S"))
    print(f"[INFO] Logging experiment in directory: {log_dir}")
    env_cfg.log_dir = log_dir

    # create environment and wrap for rsl-rl
    env = gym.make(args_cli.task, cfg=env_cfg)
    env = RslRlVecEnvWrapper(env, clip_actions=agent_cfg.clip_actions)

    resume_path = None
    if agent_cfg.resume:
        resume_path = get_checkpoint_path(log_root_path, agent_cfg.load_run, agent_cfg.load_checkpoint)

    runner = OnPolicyRunner(env, agent_cfg.to_dict(), log_dir=log_dir, device=agent_cfg.device)
    runner.add_git_repo_to_log(__file__)
    if resume_path is not None:
        print(f"[INFO] Loading model checkpoint from: {resume_path}")
        runner.load(resume_path)

    dump_yaml(os.path.join(log_dir, "params", "env.yaml"), env_cfg)
    dump_yaml(os.path.join(log_dir, "params", "agent.yaml"), agent_cfg)

    start_time = time.time()
    try:
        runner.learn(num_learning_iterations=agent_cfg.max_iterations, init_at_random_ep_len=True)
        print(f"[INFO] Training time: {round(time.time() - start_time, 2)} seconds")
        env.close()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
    simulation_app.close()
