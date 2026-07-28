# Copyright (c) 2026, dishwasher_tasks project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Play a trained RSL-RL checkpoint on the dishwasher tasks.

Boot-first pattern: the simulator app starts before any task configuration is imported or
resolved (see scripts/rsl_rl/train.py for why).

Usage:
    /workspace/isaaclab/isaaclab.sh -p scripts/rsl_rl/play.py \
        --task Isaac-Open-Dishwasher-UR5e-Play-v0 --num_envs 16
"""

"""Launch Isaac Sim Simulator first."""

import argparse

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Play a trained RSL-RL checkpoint.")
parser.add_argument("--task", type=str, required=True, help="Name of the task.")
parser.add_argument("--num_envs", type=int, default=None, help="Number of environments to simulate.")
parser.add_argument("--load_run", type=str, default=None, help="Run folder to load from (regex; default latest).")
parser.add_argument("--checkpoint", type=str, default=None, help="Checkpoint file to load (regex; default latest).")
parser.add_argument("--video", action="store_true", default=False, help="Record a video (needs --enable_cameras).")
parser.add_argument("--video_length", type=int, default=300, help="Length of the recorded video (in steps).")
parser.add_argument("--max_steps", type=int, default=None, help="Stop after this many env steps (default: run forever).")
parser.add_argument("--real-time", action="store_true", default=False, help="Run in real-time, if possible.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""Rest everything follows."""

import importlib.metadata as metadata
import os
import time

import gymnasium as gym
import torch
from rsl_rl.runners import OnPolicyRunner

from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper, handle_deprecated_rsl_rl_cfg

from isaaclab_tasks.utils import get_checkpoint_path
from isaaclab_tasks.utils.hydra import resolve_presets
from isaaclab_tasks.utils.parse_cfg import load_cfg_from_registry

import dishwasher_tasks.tasks  # noqa: F401


def main():
    """Play with an RSL-RL agent."""
    env_cfg = load_cfg_from_registry(args_cli.task, "env_cfg_entry_point")
    env_cfg = resolve_presets(env_cfg)
    agent_cfg = load_cfg_from_registry(args_cli.task, "rsl_rl_cfg_entry_point")
    agent_cfg = handle_deprecated_rsl_rl_cfg(agent_cfg, metadata.version("rsl-rl-lib"))

    if args_cli.num_envs is not None:
        env_cfg.scene.num_envs = args_cli.num_envs
    if args_cli.device is not None:
        env_cfg.sim.device = args_cli.device
    if args_cli.load_run is not None:
        agent_cfg.load_run = args_cli.load_run
    if args_cli.checkpoint is not None:
        agent_cfg.load_checkpoint = args_cli.checkpoint
    env_cfg.seed = agent_cfg.seed

    # find the checkpoint
    log_root_path = os.path.abspath(os.path.join("logs", "rsl_rl", agent_cfg.experiment_name))
    resume_path = get_checkpoint_path(log_root_path, agent_cfg.load_run, agent_cfg.load_checkpoint)
    log_dir = os.path.dirname(resume_path)
    env_cfg.log_dir = log_dir

    env = gym.make(args_cli.task, cfg=env_cfg, render_mode="rgb_array" if args_cli.video else None)
    if args_cli.video:
        video_kwargs = {
            "video_folder": os.path.join(log_dir, "videos", "play"),
            "step_trigger": lambda step: step == 0,
            "video_length": args_cli.video_length,
            "disable_logger": True,
        }
        print(f"[INFO] Recording a video to {video_kwargs['video_folder']}")
        env = gym.wrappers.RecordVideo(env, **video_kwargs)
    env = RslRlVecEnvWrapper(env, clip_actions=agent_cfg.clip_actions)

    print(f"[INFO] Loading model checkpoint from: {resume_path}")
    runner = OnPolicyRunner(env, agent_cfg.to_dict(), log_dir=None, device=agent_cfg.device)
    runner.load(resume_path)
    policy = runner.get_inference_policy(device=env.unwrapped.device)

    # export the trained policy to JIT/ONNX next to the checkpoint
    export_model_dir = os.path.join(log_dir, "exported")
    runner.export_policy_to_jit(path=export_model_dir, filename="policy.pt")
    runner.export_policy_to_onnx(path=export_model_dir, filename="policy.onnx")

    dt = env.unwrapped.step_dt
    obs = env.get_observations()
    timestep = 0
    while simulation_app.is_running():
        start_time = time.time()
        with torch.inference_mode():
            actions = policy(obs)
            obs, _, _, _ = env.step(actions)
        timestep += 1
        if args_cli.video and timestep == args_cli.video_length:
            break
        if args_cli.max_steps is not None and timestep >= args_cli.max_steps:
            break
        if args_cli.real_time:
            sleep_time = dt - (time.time() - start_time)
            if sleep_time > 0:
                time.sleep(sleep_time)

    env.close()


if __name__ == "__main__":
    main()
    simulation_app.close()
