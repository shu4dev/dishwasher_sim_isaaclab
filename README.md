# dishwasher_sim_isaaclab

Kitchen-manipulation RL environments for Isaac Lab 3.0 / Isaac Sim 6.0: a **UR5e + Robotiq 2F-85**
opens the door of an articulated **ArtVIP dishwasher** (`Isaac-Open-Dishwasher-UR5e-v0`), with a
stretch task of loading a YCB mug into the rack (`Isaac-Load-Dishwasher-UR5e-v0`).

See [docs/environment.md](docs/environment.md) for the exact machine/software stack and Isaac Lab
3.0 API notes, and [docs/joint_report.md](docs/joint_report.md) for the articulation details every
config in this repo is derived from.

## Layout

```
scripts/00_inspect_scene.py        scene inspection: joint report, stability + door tests
scripts/01_make_mug_physics_usd.py derive a physics-enabled YCB mug USD
scripts/rsl_rl/{train,play}.py     RSL-RL entry points (harvested Isaac Lab template shims)
source/dishwasher_sim_isaaclab/    the installable task package (env cfgs, mdp terms, robot cfgs)
assets/                            downloaded assets (gitignored — see below)
docs/                              environment, joint report, how-to-play
logs/                              training logs (gitignored)
```

## Setup

Everything runs through Isaac Lab's wrapper from the Isaac Lab root:

```bash
# install the task package (editable)
/workspace/isaaclab/isaaclab.sh -p -m pip install -e source/dishwasher_sim_isaaclab

# download the ArtVIP dishwasher assets (~82 MB) into assets/artvip/
/workspace/isaaclab/isaaclab.sh -p scripts/01_download_assets.py   # or see docs/environment.md
```

## Train / Play

```bash
cd /workspace/isaaclab/dishwasher_sim_isaaclab
/workspace/isaaclab/isaaclab.sh -p scripts/rsl_rl/train.py \
    --task Isaac-Open-Dishwasher-UR5e-v0 --num_envs 512 --headless
/workspace/isaaclab/isaaclab.sh -p scripts/rsl_rl/play.py \
    --task Isaac-Open-Dishwasher-UR5e-Play-v0 --num_envs 16
```

See [docs/how_to_play.md](docs/how_to_play.md) for checkpoint rollout over the streaming client.

## Asset sources and licenses

| Asset | Source | License | Notes |
|---|---|---|---|
| ArtVIP dishwasher (`assets/artvip/`) | HuggingFace dataset [`X-Humanoid/ArtVIP`](https://huggingface.co/datasets/X-Humanoid/ArtVIP), `Articulated_objects/major_appliances/dishwasher/` | **Apache-2.0** | articulated USD, door + sliding racks |
| UR5e + Robotiq 2F-85 | Isaac Sim 6.0 asset library, `Robots/UniversalRobots/ur5e/ur5e.usd` (`Gripper=Robotiq_2f_85` variant) | NVIDIA Omniverse asset EULA | fetched from Nucleus S3 at spawn |
| YCB mug (`assets/props/025_mug_physics.usd`) | Isaac Sim 6.0 asset library `Props/YCB/Axis_Aligned/025_mug.usd`, physics APIs added locally | YCB dataset terms / NVIDIA asset EULA | derived file, gitignored |

Downloaded assets are **never committed** (`assets/` is gitignored).
