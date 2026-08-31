# Environment

Measured on the **corallab workstation** (2026-08-30), where the `plan` branch now runs after
the Brev launchable was retired. This is the **canonical home of the launcher landmines** —
the README and `CLAUDE.md` link here rather than restating them.

> The benchmark is teleport-only (no robot, no motion planning on this branch); the FCL
> planning stack is CPU-bound, so the core count below matters more than the GPU.

## Hardware (host)

| Item | Value |
|---|---|
| CPU | 36 threads (i9-10980XE) |
| RAM | 125 GiB |
| GPU | 3× NVIDIA GeForce RTX 3090, 24 GB (Ampere), driver **535.230.02**, CUDA 12.2 |
| Disk | root NVMe ~99 % full — bulk data lives on `/media/corallab-s1/2tbhdd/brianshu/dishsim` |
| OS | Ubuntu 20.04.6 (glibc 2.31) — too old for any native Isaac install; Docker mandatory |

This is a **shared machine**: labmates' jobs move between the three GPUs. Pick the
least-loaded card per shell (`nvidia-smi`, then `DISHSIM_GPU=<n> docker compose ... up -d`),
and never touch other users' containers, images, or directories.

## Software stack (all inside the repo-owned container)

The runtime is the image built by `docker/Dockerfile` and run by `docker/compose.yaml`
(container `dishsim-isaac`). The driver-535 host caps Isaac Sim at **4.5.0** (the newest
release NVIDIA documents for this driver series; 5.x/6.0 want ≥ 570/580).

| Item | Value |
|---|---|
| Isaac Sim | **4.5.0** (`nvcr.io/nvidia/isaac-sim:4.5.0` base, `/isaac-sim`) |
| Isaac Lab | **v2.1.1** (git checkout at `/workspace/isaaclab`, editable-installed into Kit python) |
| Python | Kit python **3.10.15** — no venv (the old venv was the sole cause of the `EXP_PATH` landmine) |
| PyTorch | 2.7.0+cu128 (runs on the 535 driver via CUDA minor-version compat — precompiled sm_86 kernels) |
| numpy | 1.26.0 (byte-hash digests in tests are pinned per numeric environment — see `tests/test_rack_gen_frozen.py`) |
| Repo | bind-mounted at `/workspace/dishsim`, `pip install -e` on every container start |

**Local-image caveat**: the built `dishsim-isaac:4.5.0` on this box predates the Dockerfile
lines that bake the `isaaclab` core package and `pytest` — both installs had been applied by
hand in the (since-removed) original container's writable layer. The compose entrypoint now
re-applies them on every container start, so a recreated container self-heals; rebuilding the
image would also bake them but costs root disk for nothing.

History: the `plan` branch was built and validated (2026-08-28, `placement` green) on an
NVIDIA Brev launchable (L4, Isaac Sim 6.0.1-rc.7 + Isaac Lab 3.0.0, driver 595, venv
`env_isaaclab`). This port (2026-08-30) reverses the Kit boundary to the 2.1 API, following
the recipe the `on-corrallab` branch proved for the robot-era code; the Kit-free planning
stack and every collision cache are version-independent and carried over unchanged
(`config_hash` untouched — `tests/test_config_hash_frozen.py` passes unedited).

## Isaac Lab 2.1 API notes (the port, reversed from the 3.0-era code)

- **Quaternions are WXYZ** in every isaaclab surface (`rot=` config tuples, `data.*_quat_w`
  buffers, 7-D poses for `write_root_pose_to_sim`). The project convention stays **XYZW**
  internally (configs, caches, instance/episode JSON); `src/dishsim/quats.py`
  (`xyzw_to_wxyz` / `wxyz_to_xyzw`) converts at every boundary crossing and nowhere else.
  Boundary census on this branch: 7 pose-write sites (scene `_add_object`, machine cfgs,
  `gen_instances`/`run_rearrange` teleports, `instance_views`
  tableau writes) and 5 quat-read sites (`scene.assert_frames`, `geometry.body_pose_w`, the
  three scripts' `measured()`); `default_root_state` round-trips are isaaclab→isaaclab and
  must NOT convert.
- **Data buffers are plain `torch.Tensor`** — no `ProxyArray`, no `.torch` accessor.
- **PhysX is the only backend**: plain `sim_utils.SimulationCfg(...)` (no `physics=` kwarg,
  no `isaaclab_physx` import).
- **Kinematic writes lose the `_index` suffix**: `write_root_pose_to_sim(root_pose=...)`,
  `write_joint_position_to_sim(position=...)`, `set_joint_position_target(target=...)` etc.
  (same kwarg names, optional `env_ids`/`joint_ids`).
- **`data.default_root_state`** (13-D) replaces 3.0's split `default_root_pose` /
  `default_root_vel` — slice `[:, :7]` / `[:, 7:]`.
- **`isaaclab.utils.mesh` does not exist** — `dishsim.geometry` carries its own pxr-based
  extractor (`extract_prim_mesh`), including the mesh→body relative transform with scale.
- **2.1's `AppLauncher` pops `enable_cameras` off the args namespace** — scripts that read
  `args_cli.enable_cameras` after launch save/restore it around `AppLauncher(args_cli)`
  (`run_rearrange.py`; the retired capacity_fill.py did too).
- **4.5 solver behavioral delta** (measured): the ArtVIP passive door cannot hold its
  inverted-pendulum equilibrium at 0 deg (6.0's solver left it asleep there) — it falls open
  and rests against the as-shipped limit while the velocity readout chatters.
  the retired inspect_scene.py's stability/door gates were positional (tail-span) for this
  reason (git history).
- `effort_limit_sim`/`velocity_limit_sim`, `find_bodies`/`find_joints`, sensors
  (`ContactSensorCfg`, `FrameTransformerCfg`, `Camera`) and the `InteractiveScene` surface
  are unchanged vs 3.0 usage.

## Hard-won launcher findings (this project's scripts work around these)

1. **Boot-first requirement.** Every Kit entry script launches `AppLauncher` **first** and
   imports/resolves everything afterwards (`dishsim`, `isaaclab.*` scene modules, `pxr`).
2. **Package name vs. repo name.** The Python package is `dishsim` (under `src/`), never
   `dishwasher_sim_isaaclab`: Kit's extension scan turns a directory whose name matches an
   importable package into a shadowing namespace package (`unknown location` ImportErrors).
   Keep module-scope `pxr`/`omni` imports out of the package (lazy in-function imports, see
   `src/dishsim/usd_prep.py`).
3. **`./isaaclab.sh -p` exits 0 even when the wrapped script crashes** — verify success from
   log content (`[RESULT] PASS`, absence of tracebacks / `free(): invalid pointer`), never
   from the exit code. `scripts/run_kit.sh` inherits this property.
4. **The standalone `SimulationContext(sim_utils.SimulationCfg(...))` pattern** (see
   `scripts/setup/gen_instances.py`) is the reliable construction for this project's scenes.

## Launchers (docker era)

- `scripts/run_kit.sh <script> ...` — on the host, forwards itself into `dishsim-isaac` via
  `docker exec` at the mapped cwd; inside, execs `/workspace/isaaclab/isaaclab.sh -p`.
- `scripts/run_py.sh ...` — same forwarding, but execs Kit's python directly (pytest,
  planners, tools; no Kit boot). `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1` is baked in: hydra's
  auto-registered pytest plugin breaks collection outside Kit.
- `scripts/tools/bootstrap.sh` — image build (if absent) + `compose up -d` + archive restore
  + the kit_smoke install gate.
- The base image's entrypoint launches the streaming sim — `docker/compose.yaml` overrides
  it (isaaclab-core + pytest self-heal, editable-install of the repo, then `sleep infinity`
  for `docker exec`).

## Asset root

Resolved by the 4.5 Kit apps (stock scenery only — the Bosch twin, ArtVIP dishwasher and
every prop are local files):

```
https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/4.5
```

## Planning stack

CPU planning deps are baked into the image from `requirements-planning.txt` — installed into
Kit's python, no venv:

| Item | Value |
|---|---|
| python-fcl | 0.7.0.11 |
| coacd | 1.0.11 |
| trimesh | 4.12.2 |
| matplotlib | 3.10.9 (3.11.x needs python ≥ 3.11) |
| imageio | 2.37.4 |
| pytest | 9.1.1 (entrypoint-installed — see the local-image caveat above) |

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
The container runs as root, so files it writes on the drive/repo are root-owned — clean them
via `docker exec rm`, not host sudo.

Cache-anchor note: the shipped bosch800 caches were baked at base placement `side_winner`
(the restore log prints the matched anchor per cache) — Kit-free tools that recompute
`config_hash` (`decompose_meshes.py`, `extract_geometry.py`) need the matching
`--machine bosch800 --placement side_winner`, or they will mis-report the cache as stale.
