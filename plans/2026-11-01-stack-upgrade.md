# Upgrade the box to Isaac Sim 6.2.0 + Isaac Lab 3.0.0
Slug: 2026-11-01-stack-upgrade | From: results/2026-10-22-new-box-bringup.md (box recipe), docs/environment.md (2.1 vs 3.0 API notes)

## Goal
On dishsim-l4 the Bosch pipeline runs end to end with working cameras on Isaac Sim 6.2.0 + Isaac Lab 3.0.0 (python 3.12) and the Frigidaire Kit path boots; corallab stays on 4.5 at the last pre-upgrade commit.

## Decisions (from grilling)
- Box only; corallab is frozen at 4.5 on the pre-upgrade commit (suggested tag `isaac45-final`, yours); no dual-stack shims. Rejected retiring corallab from the docs (it still runs the Bosch pipeline). Upgrade before the generality round; rejected generality first.
- Disk: delete the 4.5 image on the box, switch Docker to overlay2 (about 39 GB per image instead of 61), build there; a new 200 GB box only if the image still does not fit. Rejected a new box first (the box's results and media are not in the dataset) and two boxes (double billing).
- Target `nvcr.io/nvidia/isaac-sim:6.2.0` + IsaacLab `v3.0.0`, pinned to the tag (main already removed `isaaclab.sh` and changed PhysicsManager). Rejected `6.2.0.1` (undocumented delta). No venv: the isaaclab packages go into Kit's python 3.12 and `run_kit.sh` execs `/isaac-sim/python.sh` directly. Rejected the uv venv recipe (the EXP_PATH landmine came from a venv) and the prebuilt isaac-lab image (16.7 GB, uid 1000).
- Boot through one repo helper that keeps the `--headless`, `--enable_cameras` and `--device` contract (3.0 dropped the first two from its parser). Rejected the deprecated `AppLauncher` shim (a second port when it goes) and dropping the flags from 26 scripts, the sampler wrappers and the docs.
- 3.0 names now: `write_*_to_sim_index`, `actuators.target_command`, schema fragments, `root_view`, `.torch` on every `.data` read, so nothing is left for the 3.1 and 3.2 removals. Rejected the deprecated forwarders. 2.x meanings kept: `PhysxCfg(enable_external_forces_every_iteration=False)` and `net_normal_forces_w` wherever a force threshold is read. Rejected the 3.0 defaults (friction in the net force would move every threshold).
- Re-pins: the three perturbed placement instances and `test_compat` only; the 27 pairs, fixtures, robot counts, S and the ceilings stay labelled Isaac Sim 4.5 (Warp 1.5); a Warp 1.17 re-score is a later round. Rejected re-certifying the 27 pairs and re-scoring in the window. One window, 4 h cap, hard halt armed at T0; experiment-runner implements. Rejected the refactorer (a worktree branch for a run that lives on the box).

## Change
- `code/util/docker/Dockerfile`: `FROM nvcr.io/nvidia/isaac-sim:6.2.0`; clone IsaacLab `v3.0.0`; `pip install --no-build-isolation -e source/isaaclab source/isaaclab_physx` into Kit's python; same planning pins (all ship cp312 or abi3 wheels); numpy is Kit's 2.3; tag `dishsim-isaac:6.2.0`. `compose.yaml`, `bootstrap.sh`: the tag. `requirements-planning.txt`: the python comment.
- `code/util/run_kit.sh`: `exec /isaac-sim/python.sh "$@"` (add `--no-ros-env` if 6.1's ROS setup leaks into the env). `run_py.sh`: re-find the USD extension folder (`extscache/omni.usd.libs-*`) in the 6.2 layout.
- New `code/src/dishsim/app.py`: `add_args(parser)` and `launch(args)` over `isaaclab.app.launch_simulation`, with `enable_cameras` set before launch; the 26 entry scripts swap their `AppLauncher` lines for it; `media.release_sim_for_close` becomes a no-op (3.0's SimulationContext has no stop-handle spin and 6.x flushes stdout on close).
- Kit boundary (35 files, reversing the 2.1 port in `docs/environment.md`): `quats.py`'s two functions become identity (3.0 is XYZW, `InitialStateCfg.rot` default `(0, 0, 0, 1)`), call sites untouched; `SimulationCfg(physics=PhysxCfg(...))` from `isaaclab_physx.physics`; `default_root_pose` / `default_root_vel`, `root_link_pose_w`, `body_link_pose_w`; schema fragments such as `[UsdPhysicsRigidBodyCfg, PhysxRigidBodyCfg]` with `fix_root_link` on the spawner cfg (the Frigidaire USD has `Joints/base_fixed`: pass `fix_root_link=False` there, since v3.0.0 raises on a fixed-rooted articulation); `RigidContactView` (2 files) and `omni.physics.tensors.impl.api` move to `ContactSensor.net_normal_forces_w` and `omni.physics.tensors.api`; `CameraCfg(renderer_cfg=IsaacRtxRendererCfg(...))` and `data.output["rgb"].torch`; `isaaclab.sim.create_prim`; pxr transforms take `Gf.Vec3d`.
- `CLAUDE.md` (lines 27-30, 42-54) and `docs/environment.md`: corallab stays at 4.5 on the tagged commit, the box runs 6.2.0 + 3.0.0; the stack table and the up-port deltas replace the 2.1 notes. `code/tests/test_compat.py`: `_KNOWN_OPTIMA` and `recorded_at_goal` re-pinned from the regenerated instances once the harness-agreement check passes.

## Run
Mac: `brev login` (yours), `brev start dishsim-l4`, `brev refresh`, ssh; T0 = RUNNING; `sudo shutdown -h +240` on the box. Then on the box, in order, each gate judged from its log:
```
docker inspect dishsim-isaac > ~/dishsim_container_4.5.json && docker rm -f dishsim-isaac && docker image rm dishsim-isaac:4.5.0 nvcr.io/nvidia/isaac-sim:4.5.0 && df -h /
sudo python3 -c 'import json;p="/etc/docker/daemon.json";d=json.load(open(p));d.setdefault("features",{})["containerd-snapshotter"]=False;json.dump(d,open(p,"w"),indent=1)' && sudo systemctl restart docker && df -h /
docker build -f code/util/docker/Dockerfile -t dishsim-isaac:6.2.0 . && <recreate dishsim-isaac from the saved inspect: mounts, env, --gpus all, --user root, restart unless-stopped>
code/util/run_kit.sh code/util/tools/kit_smoke.py --headless --enable_cameras && code/util/run_py.sh -m pytest code/tests/ code/frigidaire/tests/ -q
code/util/run_kit.sh code/initialization/bosch/gen_instances.py --headless --mode perturbed --state placement --n 3 --seed 0
code/util/run_kit.sh code/initialization/bosch/instance_views.py --headless --enable_cameras --instance data/results/instances/bosch800/placement/perturbed_s0.json
code/util/run_kit.sh code/planner/bosch/run_rearrange.py --headless --enable_cameras --video --instances "data/results/instances/bosch800/placement/*.json" --algorithms greedy
code/util/run_kit.sh code/initialization/frigidaire/frigidaire_initial_state_validate.py --manifest data/results/dot_rules/dependency/samples_v5/_gate/r05_00027/manifest.json --out-dir data/results/upgrade/gate_r05 --headless --device cpu
```

## Works when
`kit_smoke` prints `[RESULT] PASS` with a non-black `smoke.png` on the box (the gate 4.5 failed on this driver), pytest is green, and gen_instances, instance_views and run_rearrange with video all print `[RESULT] PASS` on 6.2.0, with `test_compat` re-pinned; results, media and logs are rsynced back, `brev stop` sent and STOPPED confirmed, results/2026-11-01-stack-upgrade.md written.

## Assumed / skipped
- Assumed: root in the container (the 6.2 image defaults to uid 1234) keeps the root-owned-file rules; 30 GiB RAM sits under the 32 GB minimum and 595.91.07 is newer than the tested 595.58.03. If Kit dies in the camera smoke, or the image does not fit after overlay2, the runner stops and reports; a 200 GB, 64 GiB box is then the next decision.
- Skipped this round: the 27 pairs, HOTEC fixtures, robot numbers, S and the ceilings (4.5 history); the rack_gen digest pin (a dataset test, re-pinned only if numpy 2.3 breaks it); a Dockerfile for both stacks; `isaacsim.core.api` uses beyond the two RigidContactView files.
