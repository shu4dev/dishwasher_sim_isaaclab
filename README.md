# dishwasher_sim_isaaclab

**v0 — classical placement planning:** a UR5e + Robotiq 2F-85 starts with a plate already in its
gripper and uses **OMPL (RRT-Connect)** to find a collision-free joint-space path that places the
plate at a valid pose in the lower rack of an articulated **ArtVIP dishwasher** (door locked
open), then releases it and verifies the placement is stable. Runs on Isaac Lab 3.0 / Isaac Sim
6.0, fully headless; every result ships with PNG/MP4 evidence under `media/`.

**Why the pivot:** this repo previously implemented an RL door-opening pipeline (PPO,
`Isaac-Open-Dishwasher-UR5e-v0`, ~93 % success). The lab's direction changed to classical motion
planning as the v0 for a longer-term **MCTS rearrangement planner** — which is why the collision
world is built as a standalone, Kit-free module (`src/dishsim/collision_world.py`) capable of
thousands of fast queries, not planner-internal code. Grasping, perception, and path constraints
(e.g. keep-upright) are explicitly out of scope for v0. The RL pipeline is preserved on the
`archive/rl-door-opening` branch.

> **Pending decision gate:** this plan assumes Isaac Sim remains the simulator. Lab confirmation
> (PyBullet/MuJoCo vs Isaac) is outstanding; if the lab picks another simulator, work stops after
> Phase B and the portable pieces are `tests/`, `ur5e_kin.py`, `collision_world.py`,
> `placement.py`, `planning.py` (pure Python/OMPL/FCL — no Isaac imports).

See [docs/environment.md](docs/environment.md) for the machine/software stack, launcher
landmines, and pinned dependency versions, and [docs/joint_report.md](docs/joint_report.md) for
the measured articulation numbers every config derives from.

## Layout

```
src/dishsim/                       the project package (installable, `pip install -e .`)
  robots.py                        UR5e+2F85 and dishwasher ArticulationCfgs (measured values)
  usd_prep.py                      derived dishwasher USDs (world-weld removal, drive prep)
  ur5e_kin.py                      analytic UR5e FK/IK, 8 branches          (Phase B)
  config.py                        every tunable: grasp, tolerances, CoACD, budgets, cameras (C)
  scene.py, media.py               v0 scene + camera/video capture          (Phase C)
  geometry.py, collision_world.py  USD→FCL extraction + Kit-free collision world (Phase D)
  placement.py, planning.py        slot frames + IK goal sets, OMPL wrapper (Phases E/F)
scripts/                           numbered entry points (00–30), boot-first AppLauncher pattern
tests/                             venv pytest for the planning stack       (Phase B)
docs/                              environment, joint report, success criteria, v0 report
assets/                            downloaded + derived assets (gitignored — see below)
media/  results/                   visual evidence + trial JSONs (gitignored, stay on disk)
```

## Setup

Everything Isaac-side runs through `scripts/run_kit.sh` (a thin shim that exports the Isaac Sim
environment and hands off to `isaaclab.sh -p` — required once the venv exists, see
[docs/environment.md](docs/environment.md)), from this project root:

```bash
# 1. planning venv (the wrapper auto-detects this exact path)
/isaac-sim/kit/python/bin/python3 -m venv --system-site-packages /workspace/isaaclab/env_isaaclab
/workspace/isaaclab/env_isaaclab/bin/pip install ompl python-fcl coacd trimesh matplotlib imageio pytest
/workspace/isaaclab/env_isaaclab/bin/pip install -e .

# 2. download the ArtVIP dishwasher assets (~82 MB) into assets/artvip/
scripts/run_kit.sh -c "from huggingface_hub import snapshot_download; \
  snapshot_download(repo_id='X-Humanoid/ArtVIP', repo_type='dataset', \
  allow_patterns=['Articulated_objects/major_appliances/dishwasher/**'], local_dir='assets/artvip')"

# 3. generate the derived dishwasher USD + joint report (runs the stability/door tests)
scripts/run_kit.sh scripts/00_inspect_scene.py --headless --test_door

# 4. derive the physics-enabled YCB plate (or mug fallback: --object 025_mug)
scripts/run_kit.sh scripts/01_make_prop_physics_usd.py --object 029_plate

# 5. planning-stack tests (no Kit)
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /workspace/isaaclab/env_isaaclab/bin/python -m pytest tests/
```

> Note: the Python package is named `dishsim` (not `dishwasher_sim_isaaclab`) on purpose — a
> package with the same name as this repo directory gets shadowed by a namespace package when
> Kit scans extension paths, which breaks imports at app boot.

> `./isaaclab.sh -p` exits 0 even when the wrapped script crashes: judge success from log
> content (`[RESULT] PASS`, absence of tracebacks), never from the exit code.

## v0 pipeline (phases)

| Phase | Entry point | Output |
|---|---|---|
| B — dependency spike | `tests/`, `scripts/05_kit_smoke.py` | deps verified, media capture proven |
| C — static scene | `scripts/10_v0_scene.py` | scene stills/clip, pose log, welded plate |
| D — collision world | `scripts/12…14_*.py` | FCL world + Isaac parity report |
| E — placement goals | `scripts/15_goal_configs.py` | slot frames + IK goal sets, `docs/success_criteria.md` |
| F — plan & place | `scripts/20_plan_and_place.py` | per-trial JSON + MP4 in `results/`, `media/F/` |
| G — benchmark | `scripts/30_make_report.py` | `docs/v0_report.md`, `docs/slides_notes.md` |

## Asset sources and licenses

| Asset | Source | License | Notes |
|---|---|---|---|
| ArtVIP dishwasher (`assets/artvip/`) | HuggingFace dataset [`X-Humanoid/ArtVIP`](https://huggingface.co/datasets/X-Humanoid/ArtVIP), `Articulated_objects/major_appliances/dishwasher/` | **Apache-2.0** | articulated USD, door + sliding racks |
| UR5e + Robotiq 2F-85 | Isaac Sim 6.0 asset library, `Robots/UniversalRobots/ur5e/ur5e.usd` (`Gripper=Robotiq_2f_85` variant) | NVIDIA Omniverse asset EULA | fetched from Nucleus S3 at spawn |
| YCB plate / mug (`assets/props/*_physics.usd`) | Isaac Sim 6.0 asset library `Props/YCB/Axis_Aligned/{029_plate,025_mug}.usd`, physics APIs added locally | YCB dataset terms / NVIDIA asset EULA | derived files, gitignored |

Downloaded assets are **never committed** (`assets/` is gitignored); `media/` and `results/`
stay on disk for review, with curated figures copied into `docs/figures/`.
