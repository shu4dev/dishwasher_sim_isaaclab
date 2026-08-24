# Environment

Measured on the **corallab workstation** (2026-08-23), the project's primary machine since
the Brev launchable was retired. This is the **canonical home of the launcher landmines** —
the README and `CLAUDE.md` link here rather than restating them.

> The project's RL door-opening pipeline lives in git history (branch
> `archive/rl-door-opening`). OMPL planning is CPU-bound, so the core count below matters more
> than the GPU.

## Hardware (host)

| Item | Value |
|---|---|
| CPU | 36 cores |
| RAM | 125 GiB |
| GPU | 3× NVIDIA GeForce RTX 3090, 24 GB (Ampere), driver **535.230.02**, CUDA 12.2 |
| Disk | root NVMe ~95 % full — bulk data lives on `/media/corallab-s1/2tbhdd/brianshu/dishsim` |
| OS | Ubuntu 20.04.6 (glibc 2.31) — too old for any native Isaac install; Docker mandatory |

## Software stack (all inside the repo-owned container)

The runtime is the image built by `docker/Dockerfile` and run by `docker/compose.yaml`
(container `dishsim-isaac`, GPU 1 by default — `DISHSIM_GPU=<n>` overrides). The driver-535
host caps Isaac Sim at **4.5.0** (the newest release NVIDIA documents for this driver
series; 5.x/6.0 want ≥ 570/580).

| Item | Value |
|---|---|
| Isaac Sim | **4.5.0** (`nvcr.io/nvidia/isaac-sim:4.5.0` base, `/isaac-sim`) |
| Isaac Lab | **v2.1.1** (git checkout at `/workspace/isaaclab`, editable-installed into Kit python) |
| Python | Kit python **3.10.15** — no venv (the old venv was the sole cause of the `EXP_PATH` landmine) |
| PyTorch | 2.7.0+cu128 (runs on the 535 driver via CUDA minor-version compat — precompiled sm_86 kernels) |
| numpy | 1.26.0 (byte-hash digests in tests are pinned per numeric environment — see `tests/test_rack_gen_bosch.py`) |
| Repo | bind-mounted at `/workspace/dishsim`, `pip install -e` on every container start |

History: the project originally ran on an NVIDIA Brev launchable (L4, Isaac Sim 6.0.1-rc.7 +
Isaac Lab 3.0.0, driver 595). The `on-corrallab` branch ports the Kit boundary to the 2.1
API; the Kit-free planning stack and every collision cache are version-independent and
carried over unchanged (`config_hash` untouched).

## Isaac Lab 2.1 API notes (the port, reversed from the 3.0-era code)

- **Quaternions are WXYZ** in every isaaclab surface (`rot=` config tuples, `data.*_quat_w`
  buffers, 7-D poses for `write_root_pose_to_sim`). The project convention stays **XYZW**
  internally; `src/dishsim/quats.py` (`xyzw_to_wxyz` / `wxyz_to_xyzw`) converts at every
  boundary crossing and nowhere else.
- **Data buffers are plain `torch.Tensor`** — no `ProxyArray`, no `.torch` accessor.
- **PhysX is the only backend**: plain `sim_utils.SimulationCfg(...)` (no `physics=` kwarg,
  no `isaaclab_physx` import; `PhysxCfg` would come from `isaaclab.sim` if needed).
- **Kinematic writes lose the `_index` suffix**: `write_root_pose_to_sim(root_pose=...)`,
  `write_joint_position_to_sim(position=...)`, `set_joint_position_target(target=...)` etc.
  (same kwarg names, optional `env_ids`/`joint_ids`).
- **`data.default_root_state`** (13-D) replaces 3.0's split `default_root_pose` /
  `default_root_vel` — slice `[:, :7]` / `[:, 7:]`.
- **`isaaclab.utils.mesh` does not exist** — `dishsim.geometry` carries its own pxr-based
  extractor (`extract_prim_mesh`), including the mesh→body relative transform with scale.
- `articulation_root_prim_path`, `effort_limit_sim`/`velocity_limit_sim`,
  `find_bodies/find_joints(preserve_order=)`, sensors (`ContactSensorCfg`,
  `FrameTransformerCfg`, `Camera`) and `InteractiveScene` surface are unchanged vs 3.0 usage.

## Hard-won launcher findings (this project's scripts work around these)

1. **Boot-first requirement.** Every Kit entry script launches `AppLauncher` **first** and
   imports/resolves everything afterwards (`dishsim`, `isaaclab.*` scene modules, `pxr`).
2. **Package name vs. repo name.** The Python package is `dishsim` (under `src/`), never
   `dishwasher_sim_isaaclab`: Kit's extension scan turns a directory whose name matches an
   importable package into a shadowing namespace package (`unknown location` ImportErrors).
   Keep module-scope `pxr`/`omni` imports out of the package (lazy in-function imports, see
   `src/dishsim/usd_prep.py`).
3. **`./isaaclab.sh -p` exits 0 even when the wrapped script crashes** — verify success from
   log content, never from the exit code. `scripts/run_kit.sh` inherits this property.
4. **The standalone `SimulationContext(sim_utils.SimulationCfg(...))` pattern** (see
   `scripts/setup/inspect_scene.py`) is the reliable construction for this project's scenes.

## Launchers (docker era)

- `scripts/run_kit.sh <script> ...` — on the host, forwards itself into `dishsim-isaac` via
  `docker exec` at the mapped cwd; inside, execs `/workspace/isaaclab/isaaclab.sh -p`.
- `scripts/run_py.sh ...` — same forwarding, but execs Kit's python directly (pytest,
  planners, tools; no Kit boot). `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1` is baked in: hydra's
  auto-registered pytest plugin breaks collection outside Kit.
- `scripts/tools/bootstrap.sh` — image build (if absent) + `compose up -d` + archive restore.
- The base image's entrypoint launches the streaming sim — `docker/compose.yaml` overrides
  it (editable-install of the repo, then `sleep infinity` for `docker exec`).

## Asset root

Resolved by the 4.5 Kit apps:

```
https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/4.5
```

The UR5e spawns from the archive's local copy when present (`robots.UR5E_USD_PATH` probes
it first), falling back to `Robots/UniversalRobots/ur5e/ur5e.usd` under the 4.5 bucket.

## Planning stack

CPU planning deps are baked into the image from `requirements-planning.txt` — installed into
Kit's python, no venv:

| Item | Value |
|---|---|
| ompl | 2.0.1 (cp310 wheel; **nanobind bindings** — see API notes below) |
| python-fcl | 0.7.0.11 |
| coacd | 1.0.11 |
| trimesh | 4.12.2 |
| matplotlib | 3.10.9 (3.11.x needs python ≥ 3.11) |
| imageio | 2.37.4 |
| pytest | latest (NOT preinstalled in Isaac Sim 4.5's python, unlike 6.0) |

**OMPL 2.0 nanobind API notes** (differs from the old Py++ bindings all tutorials show):
`setStateValidityChecker` accepts a plain Python callable; there is no `ob.StateValidityCheckerFn`
and no `ob.State(space)` constructor — allocate states with `space.allocState()` and index them.
`ob.GoalStates` exists (dishsim.planners uses it). `ob.PlannerData(si)` + `planner.getPlannerData(pd)` are
bound and work (verified 2026-07-31, used by `scripts/evaluation/plan_visual.py`): `pd.getEdges(i)` returns
a plain `list[int]`, `pd.getVertex(i).getTag()` gives RRT-Connect's tree tags (1 = start tree,
2 = goal tree), and vertex states support direct indexing for every registered planner
(verified 2026-07-31 — the GraphML readback fallback this once needed is retired).

## Storage map

| Location | Contents |
|---|---|
| `/` (root disk) | the `dishsim-isaac:4.5.0` image (~24 GB) — the only root-disk artifact |
| `2tbhdd …/dishsim/repo_data/` | `assets/ media/ results/ logs/ outputs/` — the repo's dirs are symlinks here |
| `2tbhdd …/dishsim/kit_cache/ ov_data/ pip_cache/` | Kit shader/extension/pip caches (compose mounts) |
| `2tbhdd …/dishsim/hf_home/` | `HF_HOME` (asset-archive downloads) |

The 2tbhdd mount appears at the identical path inside the container so the symlinks resolve
on both sides. Python-3.10 note: `tarfile`'s `data` filter refuses extraction through those
symlinks — `restore_assets.py` extracts its prefix-vetted members with `filter="fully_trusted"`.
