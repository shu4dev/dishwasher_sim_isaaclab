# corallab4 bring-up: run note, 2026-11-07 (driven over ssh from the Mac)
Plan: plans/2026-11-07-corallab4-bringup.md. Branch `worktree-agent-afdb8fb5e23a54661` at bf7bad1 (bf7bad1c0957c3efe861a68aa86dd4e0396614fc), the compose.yaml commit made on corallab4, pulled into the Mac worktree and pushed to origin. Logs: data/logs/bringup/ in ~/isaac-launchable/dishwasher_sim_isaaclab on corallab4. No log of this round holds a Traceback, `free(): invalid pointer` or `malloc(): corrupted top size`.

Passed:
1. Image: `docker build -f code/util/docker/Dockerfile -t dishsim-isaac:6.2.0 .` took 740 s (00:45:45Z to 00:58:05Z, `Successfully tagged dishsim-isaac:6.2.0`; the box took 1,470 s), on the legacy builder, with the base pulled from nvcr.io without a login. 41.5 GB on disk (13.9 GB content); the one pip conflict line is `osqp 1.0.5 requires joblib, which is not installed`.
2. Container: made by the docker run below. `docker logs dishsim-isaac` shows the two pip runs with no error, `pip show dishsim` gives `Editable project location: /workspace/dishsim`, and isaaclab 28.0.0 and isaaclab_physx 8.0.0 are editable under /workspace/isaaclab/source.
3. pytest, `code/util/run_py.sh -m pytest code -q`: `245 passed, 1 skipped, 24 subtests passed in 30.28s` before the baseline existed (test_bench_order.py skips as a module without it); rerun after the baseline, `1 failed, 246 passed, 1 skipped, 24 subtests passed in 27.11s`. The failure is test_r05_00027_access_edges_and_witness (failing since 2026-10-23), the skip test_r05_00027_rest_model_bowl_edges (its bowl rest table was not copied). 248 is this branch's test set, as the refactor note collected; the box's 263 passed was the 2026-11-01 set.
4. kit_smoke: `[OK] camera frame is non-black: shape (720, 1280, 3), mean 235.3, std 10.2` and `[RESULT] PASS` in 3 min 22 s, first shader compile included; data/media/smoke/smoke.png shows the checker ground and the cube.
5. Baseline: `[RESULT] baseline_ready data/results/benchmark/frigidaire_hotec/baseline/result.json` in 29 s; its 28 rack, basket and joint numbers equal the box's 2026-11-02 baseline exactly.
6. s0, the plan's `bench.py --from gate_initial --to certify`: `[RESULT] PASS 1 of 1 reruns passed` in 9.5 min, with gate_initial, fit, build, gate_goal and certify each PASS and certify's `witness rebuilt 15 of 15 in a fresh Kit process`. The fit hit the copied caches (1487 poses, 223694 conflicts) and took the box's path, `S 0.4071 -> 0.5033 (N 16384), 226 casts in 238 s` (336 to 345 s on the L4). The 15 built goal poses equal the box's bit for bit (position and quaternion difference 0.0).

Not done: the box copy. The Mac's brev CLI is logged out (`You are currently logged out, would you like to log in?` at 00:50Z and 01:21Z), so the box never started: no billed window, no `du -sh data/*`, no newer samples/_work, runs or caches. Copied from the Mac by rsync instead: the plan's nine folders, 1,002,105,159 bytes in 3,656 entries (symlinks all relative) in 2.6 min, plus data/results/hotec/frigidaire (10.9 MB; level 0 fit_conflicts.json dated 2026-11-02 01:52 EST and fit_cycles.json 02:00, level 1 02:11 and 02:13). corallab4's data/ now holds assets 13M and results 964M.

## Container command (compose.yaml at bf7bad1 as a plain docker run)
corallab4 has no `docker compose`, so on the coordinator's call this replaces the plan's `docker compose ... up -d`. compose.yaml sets no shm or ipc option, so the container has docker's defaults (64 MB shm, private ipc) on the default bridge network. The same text is in data/logs/bringup/docker_run.sh on corallab4.

```bash
docker run -d --name dishsim-isaac --restart unless-stopped --gpus all \
  -e ACCEPT_EULA=Y -e PRIVACY_CONSENT=Y -e HF_HOME=/media/corallab-s1/2tbhdd/brianshu/dishsim/hf_home \
  -v /home/shu4/isaac-launchable/dishwasher_sim_isaaclab:/workspace/dishsim \
  -v /home/shu4/dishsim-cache/kit_cache:/isaac-sim/kit/cache \
  -v /home/shu4/dishsim-cache/ov_data/cache_ov:/root/.cache/ov \
  -v /home/shu4/dishsim-cache/pip_cache:/root/.cache/pip \
  -v /home/shu4/dishsim-cache/ov_data/glcache:/root/.cache/nvidia/GLCache \
  -v /home/shu4/dishsim-cache/ov_data/computecache:/root/.nv/ComputeCache \
  -v /home/shu4/dishsim-cache/ov_data/logs:/root/.nvidia-omniverse/logs \
  -v /home/shu4/dishsim-cache/ov_data/data:/root/.local/share/ov/data \
  --entrypoint bash dishsim-isaac:6.2.0 \
  -c '/isaac-sim/python.sh -m pip install -q --no-build-isolation --no-deps -e /workspace/isaaclab/source/isaaclab -e /workspace/isaaclab/source/isaaclab_physx && /isaac-sim/python.sh -m pip install -q --no-deps -e /workspace/dishsim && exec sleep infinity'
```

## What the plan missed
1. docker.io 29.1.3 ships without the compose and buildx plugins. The user can add `docker-compose-v2` later (sudo); nothing in this round needs it. compose.yaml also defaults to GPU 1 (`DISHSIM_GPU`), and corallab4's only GPU is 0.
2. On this branch `--from gate_initial` runs fit too (STAGES: gate_initial, fit, build, ...), which plans the goal again from the seed. gate_initial judges the stored start record and boots no Kit, so the start was not re-settled on this machine.
3. The rerun rewrote s0 in place: fit cleared the goal's score and the dag, and score and graph lie outside `--to certify`, so corallab4's s0 has `dag: null` and no S_ref until `--from score --to graph` runs. The pre-run file is data/logs/bringup/s0_before.json.
4. HF_HOME still names the old drive path, which now lands in the container's writable layer. pip disables its cache because /home/shu4/dishsim-cache/pip_cache belongs to shu4 and pip runs as root (a warning in docker logs). data/results/sampler/bowl_rest (212 KB) is not on the copy list. The container's PID 1 is `sleep infinity`, which reaps no children, so each Kit run leaves a defunct omni.telemetry process (six after this round, harmless so far).
5. corallab4's git email is the placeholder `ID+shu4dev@users.noreply.github.com`, and bf7bad1 carries it.

Next: `brev login` on the Mac, then one box window for `du -sh data/*` and the newer samples/_work, runs and caches.
