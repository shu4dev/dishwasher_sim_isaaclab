# Environment

Measured on an NVIDIA Brev Isaac Launchable instance and re-verified on a fresh instance of
the same launchable. This is the **canonical home of the launcher landmines** — the README and
`CLAUDE.md` link here rather than restating them.

> The project's RL door-opening pipeline lives in git history (branch
> `archive/rl-door-opening`). OMPL planning is CPU-bound, so the vCPU count below matters more
> than the GPU.

## Hardware

| Item | Value |
|---|---|
| CPU | AMD EPYC 7R13, **8 vCPU** |
| RAM | **30 GiB** (no swap) |
| GPU | NVIDIA L4, 23 GB VRAM (Ada), driver 595.71.05, CUDA 13.2 |
| Disk | ~146 GB free on `/` (overlay) |
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
   For this project's config that pattern crashed natively (`free(): invalid pointer`) during
   Kit startup — deterministically, from any working directory, while the same flow works for
   in-tree tasks. Every Kit entry script in this repo therefore launches `AppLauncher`
   **first** and imports/resolves everything afterwards (the same pattern as the in-tree
   tutorials), which is reliable.
2. **`sim` must be a `PresetCfg`** — for the gym/manager-env workflow. The v0 standalone
   scripts sidestep the whole mechanism by constructing
   `SimulationContext(sim_utils.SimulationCfg(..., physics=PhysxCfg()))` directly (the pattern
   in `scripts/setup/inspect_scene.py`); only gym-registered tasks need the
   `PresetCfg`/`resolve_presets` dance.
3. **Package name vs. repo name.** The Python package is `dishsim` (under `src/`, previously
   `dishwasher_tasks`), never `dishwasher_sim_isaaclab`: Kit's extension scan turns a directory
   whose name matches an importable package into a shadowing namespace package
   (`unknown location` ImportErrors). Keep module-scope `pxr`/`omni` imports out of the package
   (lazy in-function imports, see `src/dishsim/usd_prep.py`).
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
  `RigidBodyAPI`). The project originally derived a local physics mug USD from that bucket
  asset; since the 2026-08-10 public-asset migration the mug builds from the public YCB
  google_16k scan instead (see `docs/asset_survey.md`).

> **Bucket drift warning:** the in-tree Isaac Lab asset configs and the live 6.0 S3 bucket are
> slightly out of sync (e.g. `panda_instanceable.usd` no longer exists) — every asset URL used
> by this project is verified against the bucket before use. And `./isaaclab.sh -p` **exits 0
> even when the wrapped script crashes** — success must be verified from log output, never
> from the exit code.

## Planning stack

The RL train/play entry points were removed in the v0 pivot (recoverable from the
`archive/rl-door-opening` branch). The v0 stack adds a venv plus CPU planning dependencies:

```bash
/isaac-sim/kit/python/bin/python3 -m venv --system-site-packages /workspace/isaaclab/env_isaaclab
/workspace/isaaclab/env_isaaclab/bin/pip install -r requirements-planning.txt
/workspace/isaaclab/env_isaaclab/bin/pip install -e /workspace/isaaclab/dishwasher_sim_isaaclab
```

The pins in `requirements-planning.txt` are exactly the measured working set in the table
below (the table stays the measurement of record).

| Item | Value |
|---|---|
| venv | `/workspace/isaaclab/env_isaaclab` (`--system-site-packages`, wrapper-native path) |
| ompl | 2.0.1 (cp312 manylinux wheel; **nanobind bindings** — see API notes below) |
| python-fcl | 0.7.0.11 |
| coacd | 1.0.11 |
| trimesh | 4.12.2 in the venv (in-Kit resolves to 4.11.1 from `omni.pip.compute` via PYTHONPATH precedence) |
| matplotlib | 3.11.1 |
| imageio | 2.37.4 in the venv (in-Kit: 2.37.2 from the prebundle) |
| imageio-ffmpeg | 0.6.0 (preinstalled, bundled static ffmpeg 7.0.2 — no system ffmpeg) |
| pin (Pinocchio) | 4.1.0 (preinstalled — validates the hand-rolled UR5e analytic IK in `tests/`) |
| pytest | 9.1.1 (system site) |

**OMPL 2.0 nanobind API notes** (differs from the old Py++ bindings all tutorials show):
`setStateValidityChecker` accepts a plain Python callable; there is no `ob.StateValidityCheckerFn`
and no `ob.State(space)` constructor — allocate states with `space.allocState()` and index them.
`ob.GoalStates` exists (dishsim.planners uses it). `ob.PlannerData(si)` + `planner.getPlannerData(pd)` are
bound and work (verified 2026-07-31, used by `scripts/evaluation/plan_visual.py`): `pd.getEdges(i)` returns
a plain `list[int]`, `pd.getVertex(i).getTag()` gives RRT-Connect's tree tags (1 = start tree,
2 = goal tree), vertex states support direct indexing, and `pd.printGraphML()` returns the GraphML
document as a string (per-vertex reals in its `coords` attribute — the readback fallback
`planning._coords_from_graphml` parses this).

### Venv/wrapper interactions (hard-won, 2026-07-29)

1. **Kit boot needs `scripts/run_kit.sh`.** With `env_isaaclab` present, `isaaclab.sh -p`
   resolves to the bare venv interpreter, which lacks the env `_isaac_sim/python.sh` exports
   (`EXP_PATH`/`CARB_APP_PATH`/`ISAAC_PATH`, kit `LD_LIBRARY_PATH`, kit `PYTHONPATH`);
   `AppLauncher` then dies with `KeyError: 'EXP_PATH'`. Every Kit entry script therefore runs
   through `scripts/run_kit.sh` (exports that env, then `exec isaaclab.sh -p "$@"`). Non-Kit
   invocations (`pip`, pxr-only scripts, pytest) work with the bare venv python directly.
2. **pytest needs `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1`.** The system site-packages carry hydra,
   whose auto-registered pytest plugin imports `yaml` — a module that only exists inside Kit's
   `pip_prebundle` paths, so collection crashes outside Kit. Disabling plugin autoload avoids
   the whole class of problem:
   `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /workspace/isaaclab/env_isaaclab/bin/python -m pytest tests/`
