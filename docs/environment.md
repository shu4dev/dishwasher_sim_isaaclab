# Environment

Discovered 2026-07-28 on an NVIDIA Brev Isaac Launchable instance.

## Hardware

| Item | Value |
|---|---|
| GPU | NVIDIA L4, 23 GB VRAM (Ada), driver 595.71.05, CUDA 13.2 |
| Disk | ~147 GB free on `/` (overlay) |
| OS | Linux 6.17.0-1019-aws |

## Software stack

| Item | Value |
|---|---|
| Isaac Sim | **6.0.1-rc.7** (`/isaac-sim`, symlinked as `/workspace/isaaclab/_isaac_sim`) |
| Isaac Lab | **3.0.0** (`/workspace/isaaclab`, source install, not a git checkout) |
| Python | kit Python 3.12.13 via `./isaaclab.sh -p` (no venv/conda; resolves to `_isaac_sim/python.sh`) |
| PyTorch | 2.10.0+cu128 |
| rsl-rl-lib | 5.0.1 |
| warp-lang | 1.13.0 |
| gymnasium | 1.2.1 |
| huggingface_hub | 0.36.2 (pre-installed) |

> **Note:** the project brief assumed Isaac Sim 5.x + Isaac Lab 2.x. The launchable actually ships
> Isaac Sim 6.0.1 + Isaac Lab 3.0.0. Per the ground rules (never up/downgrade), all code in this
> repo targets the 3.0 API.

## Isaac Lab 3.0 API notes (differences vs. 2.x tutorials)

- **Quaternions are XYZW** everywhere in configs (`rot=(0, 0, 0, 1)` is identity). 2.x used WXYZ.
- **Asset/sensor data buffers are `ProxyArray`** — append `.torch` to get a `torch.Tensor`,
  e.g. `articulation.data.joint_pos.torch[:, ids]`.
- **RSL-RL runner cfg** uses `actor=RslRlMLPModelCfg(...)` / `critic=...` with a
  `distribution_cfg` — not the 2.x `policy=RslRlPpoActorCriticCfg(...)` shape.
- **Physics backend split**: `SimulationCfg.physics = PhysxCfg(...)` with `PhysxCfg` imported from
  `isaaclab_physx.physics`. PhysX is the default; Newton (`physics=newton_mjwarp` CLI token) is
  opt-in. This project stays on PhysX.
- **Actuator limits**: use `effort_limit_sim` / `velocity_limit_sim` on `ImplicitActuatorCfg`.
- `scripts/reinforcement_learning/rsl_rl/train.py` / `play.py` still work but emit a
  `DeprecationWarning`; the current entry is `./isaaclab.sh train --rl_library rsl_rl --task <ID>`.

## Hard-won launcher findings (this project's scripts work around these)

1. **Boot-first requirement.** The stock template scripts resolve the env config *before* Kit
   boots (`hydra_task_config` / `resolve_task_config` + `launch_simulation(env_cfg, ...)`).
   For this project's config that pattern crashes natively (`free(): invalid pointer`) during
   Kit startup — deterministically, from any working directory, while the same flow works for
   in-tree tasks. This project's `scripts/rsl_rl/train.py`, `play.py`, and `zero_agent.py`
   therefore launch `AppLauncher` **first** and import/resolve everything afterwards (the same
   pattern as the in-tree tutorials), which is reliable.
2. **`sim` must be a `PresetCfg`.** The launcher machinery expects `env_cfg.sim` to be an
   `isaaclab_tasks.utils.PresetCfg` wrapper (as all in-tree tasks do). The env config uses a
   PhysX-only `DishwasherSimCfg(PresetCfg)`; standalone scripts resolve it via
   `isaaclab_tasks.utils.hydra.resolve_presets(env_cfg)`.
3. **Package name vs. repo name.** The Python package is `dishwasher_tasks`, not
   `dishwasher_sim_isaaclab`: Kit's extension scan turns a directory whose name matches an
   importable package into a shadowing namespace package (`unknown location` ImportErrors).
4. **`./isaaclab.sh -p` exits 0 even when the wrapped script crashes** — verify success from
   log content, never from the exit code.

## Asset root

Resolved from `apps/isaaclab.python.kit` by `isaaclab.utils.assets`:

```
ISAAC_NUCLEUS_DIR = https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/6.0/Isaac
```

No local asset cache exists — every Nucleus-path spawn is a live HTTPS fetch.

Notable findings in the 6.0 asset library:
- `Robots/UniversalRobots/ur5e/ur5e.usd` has variant sets
  `Gripper: [None, Robotiq_2f_85]`, `Physics: [None, PhysX]`, `Sensor` — i.e. a **pre-assembled
  UR5e + Robotiq 2F-85** is available by selecting `variants={"Gripper": "Robotiq_2f_85"}`.
- `Props/YCB/Axis_Aligned_Physics/` contains **only** 003_cracker_box, 004_sugar_box,
  005_tomato_soup_can, 006_mustard_bottle. The mug (025) and bowl (024) exist only in the plain
  `Axis_Aligned/` folder **without** physics APIs (and Isaac Lab's spawner cannot add a missing
  `RigidBodyAPI`). This project derives a local physics-enabled mug USD instead
  (`scripts/01_make_mug_physics_usd.py`).

## Smoke test (Phase 0)

```bash
cd /workspace/isaaclab
./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/train.py \
    --task Isaac-Reach-UR10-v0 --num_envs 64 --max_iterations 5 --headless
```

**Result: PASS** — 5 learning iterations logged, checkpoints written to
`/workspace/isaaclab/logs/rsl_rl/reach_ur10/`, no crash.

Two findings from the smoke-test process:

1. `Isaac-Open-Drawer-Franka-v0` (the first candidate) **fails on this machine**: the in-tree
   `FRANKA_PANDA_CFG` points at `{ISAACLAB_NUCLEUS_DIR}/Robots/FrankaEmika/panda_instanceable.usd`,
   which no longer exists in the 6.0 S3 bucket (the bucket now has
   `FrankaEmika/franka_panda.usda` + a `Legacy/` folder). The in-tree asset configs and the live
   bucket are slightly out of sync — every asset URL used by this project is verified against the
   bucket before use.
2. `./isaaclab.sh -p` **exits 0 even when the wrapped script crashes** — success must be verified
   from the log output (iteration lines / absence of tracebacks), never from the exit code.

## Train / play invocations for this project

```bash
cd /workspace/isaaclab/dishwasher_sim_isaaclab
# train
/workspace/isaaclab/isaaclab.sh -p scripts/rsl_rl/train.py \
    --task Isaac-Open-Dishwasher-UR5e-v0 --num_envs 512 --headless
# play
/workspace/isaaclab/isaaclab.sh -p scripts/rsl_rl/play.py \
    --task Isaac-Open-Dishwasher-UR5e-Play-v0 --num_envs 16
```
