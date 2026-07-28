# How to play a trained checkpoint

All commands run from the project root (`/workspace/isaaclab/dishwasher_sim_isaaclab`) so the
`logs/` paths resolve.

## First training result (2026-07-28)

PPO, 512 envs, 200 iterations (~12 min on the L4): **`Metrics/success_rate` reached 0.92–0.94**
(fraction of episodes with the door held past 60°). Mean reward plateaued around iteration 100
at ~100–107. The policy opens the door by hooking/pushing the lip handle rather than a pinch
grasp (the grasp-shaping terms stayed near zero). Tensorboard logs:
`logs/rsl_rl/ur5e_open_dishwasher/` — the `Episode_Reward/open_door_bonus` and
`Metrics/success_rate` curves show the door-angle progress.

## Roll out the latest checkpoint (16 envs, `_PLAY` config)

```bash
cd /workspace/isaaclab/dishwasher_sim_isaaclab
/workspace/isaaclab/isaaclab.sh -p scripts/rsl_rl/play.py \
    --task Isaac-Open-Dishwasher-UR5e-Play-v0 --num_envs 16
```

`play.py` automatically picks the most recent run/checkpoint under
`logs/rsl_rl/ur5e_open_dishwasher/`. Use `--load_run <run_dir>` / `--checkpoint <model_N.pt>` to
select a specific one.

## Watching over the streaming client

The play config has 16 envs and observation noise disabled. To watch it live over the Kit App
Streaming client, add the livestream flag (then connect with the streaming client while the
script runs):

```bash
/workspace/isaaclab/isaaclab.sh -p scripts/rsl_rl/play.py \
    --task Isaac-Open-Dishwasher-UR5e-Play-v0 --num_envs 16 --livestream 2
```

Success looks like: the arm reaches to the slim lip handle on the door's upper edge, pinches or
hooks it, and pulls the bottom-hinged door down/open past 60° (the door then rests near 90°).

## Record a video headless (no streaming client)

```bash
/workspace/isaaclab/isaaclab.sh -p scripts/rsl_rl/play.py \
    --task Isaac-Open-Dishwasher-UR5e-Play-v0 --num_envs 16 --headless \
    --enable_cameras --video --video_length 300
```

The clip lands under the run's `videos/` folder.

## Tensorboard

```bash
/workspace/isaaclab/isaaclab.sh -p -m tensorboard.main --logdir logs/rsl_rl/ur5e_open_dishwasher --bind_all
```

Key curves: `Episode_Reward/open_door_bonus`, `Episode_Reward/multi_stage_open_door`,
`Metrics/success_rate` (fraction of episodes where the door passed 60°).
