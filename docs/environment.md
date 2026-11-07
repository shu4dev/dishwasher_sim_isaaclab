# Environment

Two stacks since 2026-11-01. HEAD (branch `plan`) runs on the **Brev box** `dishsim-l4` with Isaac Sim
**6.2.0** + Isaac Lab **v3.0.0** (results/2026-11-01-stack-upgrade.md). The **corallab workstation**
(measured 2026-08-30) stays on Isaac Sim 4.5.0 + Isaac Lab v2.1.1 at the last pre-upgrade commit
(tag `isaac45-final`); its hardware, storage and shared-machine sections below describe that machine,
and the 2.1 API notes live in that commit's copy of this file. This is the **canonical home of the
launcher landmines** — the README and `CLAUDE.md` link here rather than restating them.

> The benchmark (the Frigidaire twin + the HOTEC set, `code/sampler/scripts/bench.py`) is teleport-only (no robot arm, no arm
> motion planning); an arm was studied separately in the Frigidaire twin
> ([code/sampler/robot/docs/robot.md](../code/sampler/robot/docs/robot.md), closed 2026-10-30). The FCL planning stack and the
> PhysX settles are CPU-bound; the score-greedy build, the score and the ceiling grids cast rays on the GPU (Warp).

## Hardware (corallab host, the 4.5 stack)

| Item | Value |
|---|---|
| CPU | 36 threads (i9-10980XE) |
| RAM | 125 GiB |
| GPU | 3× NVIDIA GeForce RTX 3090, 24 GB (Ampere), driver **535.230.02**, CUDA 12.2 |
| Disk | root NVMe 928 GiB, 80 % used with about 182 GiB free (2026-09-29) — bulk data lives on `/media/corallab-s1/2tbhdd/brianshu/dishsim` |
| OS | Ubuntu 20.04.6 (glibc 2.31) — too old for any native Isaac install; Docker mandatory |

The Brev box: g6.2xlarge, one L4 (24 GB) on driver 595.91.07 (Isaac Sim 6.2 is tested on 595.58.03),
AMD EPYC 7R13 with 4 cores / 8 threads, 30 GiB RAM (under 6.2's 32 GB minimum), 117 GB root disk, a
419 GB ephemeral NVMe for caches (results/2026-10-22-new-box-bringup.md).

This is a **shared machine**: labmates' jobs move between the three GPUs. Pick the
least-loaded card per shell (`nvidia-smi`, then `DISHSIM_GPU=<n> docker compose ... up -d`),
and never touch other users' containers, images, or directories.

## Software stack (all inside the repo-owned container)

The runtime is the image built by `code/util/docker/Dockerfile` (container `dishsim-isaac`). On the
Brev box the container is created by hand from the saved inspect of its predecessor (no compose):
`--gpus all`, `--user root`, `--restart unless-stopped`, the repo at `/workspace/dishsim`, Kit/pip/HF
caches and `/tmp` on `/opt/dlami/nvme/dishsim`, and an entrypoint that editable-installs
isaaclab, isaaclab_physx and the repo, then sleeps. Docker runs on overlay2 there (the containerd
image store kept compressed + unpacked layers: 61 GB for the 4.5 image).

| Item | Brev box (HEAD) | corallab (tag `isaac45-final`) |
|---|---|---|
| Isaac Sim | **6.2.0** (`nvcr.io/nvidia/isaac-sim:6.2.0`, `/isaac-sim`) | **4.5.0** (`nvcr.io/nvidia/isaac-sim:4.5.0`) |
| Isaac Lab | **v3.0.0** (`/workspace/isaaclab`; isaaclab, isaaclab_physx, isaaclab_newton, isaaclab_ov editable with `--no-deps`) | **v2.1.1** (`./isaaclab.sh --install none` + the core package) |
| Python | Kit python **3.12** — no venv | Kit python **3.10.15** — no venv |
| numpy / torch / warp | Kit's numpy 2.3.1; torch 2.11.0+cu130, warp-lang 1.17.0, newton 1.6.1 pip-installed (Isaac Lab's pins) | numpy 1.26.0, torch 2.7.0+cu128 |
| Image tag | `dishsim-isaac:6.2.0` (26.7 GB on overlay2) | `dishsim-isaac:4.5.0` (39.3 GB) |
| Repo | bind-mounted at `/workspace/dishsim`, `pip install -e` on every container start | same |

The old venv was the sole cause of the `EXP_PATH` landmine; neither image has one.

**Local-image caveat (corallab)**: the built `dishsim-isaac:4.5.0` there predates the Dockerfile
lines that baked the `isaaclab` core package and `pytest`; that commit's compose entrypoint
re-applies both on every start.

History: the `plan` branch was built and validated (2026-08-28, `placement` green) on an NVIDIA
Brev launchable (L4, Isaac Sim 6.0.1-rc.7 + Isaac Lab 3.0.0, driver 595, venv `env_isaaclab`),
ported down to 2.1 for corallab on 2026-08-30, and ported up to 6.2.0 + 3.0.0 for the Brev box on
2026-11-01 (plans/2026-11-01-stack-upgrade.md). The Kit-free planning stack is version-independent and
carried over unchanged. The Bosch 800 slot pipeline and its caches left the repo the same day
(plans/2026-11-01-merged-benchmark.md; git history, tag `isaac45-final`).

## Isaac Lab 3.0 up-port deltas (2026-11-01, from the 2.1 code)

- **Boot**: every entry script calls `dishsim.app.add_args(parser)` and `dishsim.app.launch(args)`
  (over `isaaclab.app.launch_simulation`, the `AppLauncher` shim is deprecated). 3.0's parser has no
  `--headless` / `--enable_cameras`; the helper adds both back, sets `enable_cameras` before the
  launch (3.0 auto-enables cameras only for a config it can scan), passes the launcher's own keys
  only (a script's `--video` would otherwise make Kit demand MoviePy) and returns the app, whose
  `close()` unwinds the launch.
- **Quaternions are XYZW** in 3.0, so `dishsim/quats.py`'s two functions are the identity and stay
  at every crossing. Hard-coded WXYZ literals were converted (light orientations, robot-era pose
  writes and mounts). `pxr` `Gf.Quat*` still takes the real part first: never route it through
  `quats.py`.
- **ProxyArray**: every `.data.*` read takes `.torch` (camera frames: `data.output["rgb"].torch`).
- **Writes**: `write_*_to_sim_index(root_pose=..., root_velocity=..., position=..., velocity=...)`,
  keyword-only; joint targets via `actuators.target_command.set_position_index(value=...)` (and the
  velocity / effort setters). `default_root_pose` / `default_root_vel` replace `default_root_state`,
  `root_link_pose_w` + `root_com_vel_w` replace `root_state_w`, `body_mass` replaces `default_mass`,
  `root_view` replaces `root_physx_view`, `actuators.applied_effort` and
  `actuators.target_command.position` replace `data.applied_torque` / `data.joint_pos_target`.
- **Physics config**: `SimulationCfg(physics=PhysxCfg(...))` with `PhysxCfg` from
  `isaaclab_physx.physics`, always with `enable_external_forces_every_iteration=False`: the 2.x
  meaning (2.1.1 never authored it, PhysX's default is off; 3.0's default is on). Every other
  `PhysxCfg` default equals 2.1.1's.
- **Schema fragments**: `rigid_props=[UsdPhysicsRigidBodyCfg(), PhysxRigidBodyCfg(...)]`,
  `collision_props=[UsdPhysicsCollisionCfg(), PhysxCollisionCfg()]`, `MassCfg`,
  `articulation_props=[PhysxArticulationCfg(...)]` with `fix_root_link` on the spawner cfg, and
  `PhysxRigidBodyMaterialCfg` (PhysX classes from `isaaclab_physx.sim.schemas` / `.sim.spawners.materials`).
  The Frigidaire twin welds itself (`Joints/base_fixed`, a sibling of the `Cabinet` root), so its
  spawner passes `fix_root_link=False`: no second world joint, and the weld is not under the root,
  so nothing is disabled.
- **Actuators**: `joint_effort_limit` / `joint_velocity_limit` replace `effort_limit_sim` /
  `velocity_limit_sim`; implicit-actuator gains are articulation-owned (no cache to mirror after
  `write_joint_stiffness_to_sim_index`).
- **Cameras**: `CameraCfg(renderer_cfg=IsaacRtxRendererCfg(enable_scene_partitioning=False))`, explicitly:
  that renderer type sets `/isaaclab/render/rtx_sensors`, which makes `sim.is_rendering` true, and
  frames are produced when a camera reads (`camera.update`), which pumps Kit once per step. 3.0
  partitions the RTX scene per env by default; a world-level camera (`/World/Cam_*`, no partition
  token) then sees only the ground and lights, not `env_0`'s dishwasher and dishes (found on the
  first 6.2 run: stills of the empty floor while the run printed PASS). The first RTX boot after a box
  start compiles shaders for about 4-5 min (the NVMe cache is wiped at every stop).
- **Contacts**: the Frigidaire runtimes' `isaacsim.core.api` `RigidContactView` became the PhysX
  tensor API's own view (`PhysxManager.get_physics_sim_view().create_rigid_contact_view`, i.e.
  `omni.physics.tensors.api`) plus contact reports at threshold 0 before reset, as the old class did;
  `get_contact_data` returns the same six buffers, separations included. ContactSensor force reads
  use `net_normal_forces_w` / `normal_force_matrix_w` (the 2.x meaning; on PhysX 6.2
  `net_forces_w` adds friction).
- **Shutdown**: `media.release_sim_for_close` only flushes stdio (3.0 has no on-stop handle to disarm).
- **Launchers**: `run_kit.sh` execs `/isaac-sim/python.sh` (isaaclab.sh is deprecated in 3.0);
  `run_py.sh` finds `omni.usd.libs` in `extscache/` or the 6.x `kit/extscore` / `exts` folders.
- **Install**: `--no-deps` editable installs (3.0 keeps its third-party pins in the root pyproject and
  Kit ships numpy/torch/warp); `isaaclab.app.sim_launcher` imports `isaaclab_newton` and `isaaclab_ov`
  config modules at module scope, so both packages are installed; `lazy_loader`, `rich` and
  `prettytable` are the light modules the core import path adds; `imageio-ffmpeg` backs
  `VideoWriter`.

## Hard-won launcher findings (this project's scripts work around these)

1. **Boot-first requirement.** Every Kit entry script boots Kit **first** (`dishsim.app.launch`;
   `AppLauncher` on the 4.5 commit) and imports/resolves everything afterwards (`isaaclab.*` scene
   modules, `pxr`, the Kit-only `dishsim` modules; `dishsim.app` itself is Kit-free).
2. **Package name vs. repo name.** The Python package is `dishsim` (under `code/util/`), never
   `dishwasher_sim_isaaclab`: Kit's extension scan turns a directory whose name matches an
   importable package into a shadowing namespace package (`unknown location` ImportErrors).
   Keep module-scope `pxr`/`omni` imports out of the package (lazy in-function imports, see
   `code/util/dishsim/media.py`).
3. **The exit code is not a verdict.** `./isaaclab.sh -p` (4.5) exits 0 even when the wrapped
   script crashes, and Kit's close can end the process with 0 on 6.2 too — verify success from log
   content (`[RESULT] PASS`, absence of tracebacks / `free(): invalid pointer`), never from the exit
   code. `code/util/run_kit.sh` inherits this property.
4. **The standalone `SimulationContext(sim_utils.SimulationCfg(...))` pattern** (see
   `code/sampler/initial_state_runtime.py`) is the reliable construction for this project's
   scenes.

## Launchers (docker era)

- `code/util/run_kit.sh <script> ...` — on the host, forwards itself into `dishsim-isaac` via
  `docker exec` at the mapped cwd; inside, execs `/isaac-sim/python.sh` (the 4.5 commit:
  `/workspace/isaaclab/isaaclab.sh -p`).
- `code/util/run_py.sh ...` — same forwarding, but execs Kit's python directly (pytest,
  planners, tools; no Kit boot). `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1` is baked in: hydra's
  auto-registered pytest plugin breaks collection outside Kit.
- `code/util/tools/bootstrap.sh` — image build (if absent) + `compose up -d` + the kit_smoke
  install gate. It restores nothing: `data/assets` is a copy of the HF dataset
  `shu4dev/dishsim-assets`, downloaded once beforehand with
  `hf download shu4dev/dishsim-assets --repo-type dataset --local-dir data/assets` (since 2026-10-21).
- The base image's entrypoint launches the streaming sim — `code/util/docker/compose.yaml` overrides
  it (isaaclab-core + pytest self-heal, editable-install of the repo, then `sleep infinity`
  for `docker exec`).

## Asset root

Resolved by the Kit apps (stock scenery only — the Frigidaire twin and the HOTEC set are local files);
the 4.5 apps used:

```
https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/4.5
```

## Planning stack

CPU planning deps are baked into the image from `requirements-planning.txt` — installed into
Kit's python, no venv (the same pins on 3.10 and 3.12; all ship cp312 or abi3 wheels):

| Item | Value |
|---|---|
| python-fcl | 0.7.0.11 |
| coacd | 1.0.11 |
| trimesh | 4.12.2 |
| matplotlib | 3.10.9 |
| imageio | 2.37.4 (+ imageio-ffmpeg 0.6.0 on the 6.2 image, the libx264 backend of `VideoWriter`) |
| ortools | 9.15.6755, `--no-deps` (CP-SAT for the bench's fit, 2026-11-02; Kit's numpy 2.3.1 and protobuf stay; `ortools.sat` imports pandas, installed 3.1.0 `--no-deps` only where Kit's python lacks it). The Mac's `.venv-kitfree` has the same ortools pin (`uv pip install`). |
| pytest | 9.1.1 (entrypoint-installed — see the local-image caveat above) |

## Storage map

| Location | Contents |
|---|---|
| `/` (root disk) | the `dishsim-isaac:4.5.0` image (~24 GB) plus the container's writable layer (0.93 GiB on 2026-09-29: the isaaclab and pytest installs the entrypoint re-applies at every start) |
| `2tbhdd …/dishsim/repo_data/` | `assets/ build/ media/ results/ logs/ outputs/` — the repo's `data/` dirs are symlinks here; `build/` held the only copy of the current Frigidaire twin build until it moved into the asset dataset (`data/assets/frigidaire_fdpc4221as/`, 2026-10-21) |
| `2tbhdd …/dishsim/kit_cache/ ov_data/ pip_cache/` | Kit shader/extension caches and the pip cache mount (compose mounts; `pip_cache/` stays empty, see below) |
| `2tbhdd …/dishsim/hf_home/` | `HF_HOME` (HF hub downloads; the asset-archive tarballs until 2026-10-21) |
| `2tbhdd …/dishsim/_trash_20260917/`, `_trash_20260929/` | 30-day hold folders of retired data, each with a `MANIFEST.txt` (delete after 2026-10-17 / 2026-10-29) |
| `2tbhdd …/dishsim/robot_era_evidence/` | robot-era `results/` and `media/` kept outside the repo roots (includes `results/plate_settle/`, the settle-reliability probe records) |

**The pip cache is disabled** (2026-09-29): `pip_cache/` is mounted at `/root/.cache/pip` and
is owned by uid 1029, and pip run as root refuses a cache directory it does not own, so every
`pip install` in the container warns `The cache has been disabled`, `pip cache info` errors
out, and the folder stays empty.

The 2tbhdd mount appears at the identical path inside the container so the symlinks resolve
on both sides. The container runs as root, so files it writes on the drive/repo are
root-owned — clean them via `docker exec rm`, not host sudo.

## Pausing & releasing the machine (shared box)

When no benchmark work is running, stop the project's container so its claim on the box is
zero (an idle container already holds ~0 CPU/GPU — compute is held by *processes*, not by
stopped reservations — but a stopped container is the polite, unambiguous state).

```bash
# 1. confirm nothing of OURS is mid-run (only the `sleep infinity` keeper should show)
docker top dishsim-isaac
# if a Kit job is live: let it finish, or kill it INSIDE the container first so
# records are not cut mid-write:  docker exec dishsim-isaac pkill -f 'isaac|kit'

# 2. stop (keeps the container + its entrypoint installs for a fast restart;
#    restart: unless-stopped means it STAYS stopped across daemon restarts/reboots)
docker compose -f code/util/docker/compose.yaml stop

# 3. verify the release
docker ps --filter name=dishsim-isaac        # empty
nvidia-smi --query-compute-apps=pid,process_name,used_memory --format=csv  # no dishsim rows
```

`Exited (137)` is normal: `sleep infinity` ignores SIGTERM, so docker SIGKILLs after the
10 s grace — there is no state to lose. Prefer `stop` over `down`: `down` removes the
container, and the next bring-up must recreate it (entrypoint pip reinstalls + the
readiness wait `bootstrap.sh` performs).

**Restart**: same GPU → `docker compose -f code/util/docker/compose.yaml start` (seconds). Different
GPU (check `nvidia-smi` first, pick the least-loaded) → `DISHSIM_GPU=<n> docker compose -f
code/util/docker/compose.yaml up -d` — the changed config recreates the container, so wait for the
entrypoint installs (or just run `code/util/tools/bootstrap.sh`, which waits for readiness).

Shared-box safety rules:

1. Act by NAME only — `docker compose -f code/util/docker/compose.yaml stop` / `docker stop
   dishsim-isaac`. Never `docker stop $(docker ps -q)`, never `docker system prune`
   (it can delete labmates' stopped containers and images), never `nvidia-smi --gpu-reset`.
2. `nvidia-smi` lists HOST PIDs with no container attribution; root-owned GPU PIDs are
   usually *other people's containers*. Attribute before judging:
   `cat /proc/<pid>/cgroup` and compare the container id against `docker ps --no-trunc`.
3. Kill order: the job inside the container first (`pkill` via `docker exec`), the
   container second. Never kill container PIDs from the host, never host `sudo`.
4. Verify after every stop: `docker ps` + `nvidia-smi`, as above.
