# Bring the project up on corallab4
Slug: 2026-11-07-corallab4-bringup | From: results/2026-11-07-corallab4-check.md

## Goal
The refactor branch runs on corallab4 inside the same `dishsim-isaac:6.2.0` image as on the box: pytest, kit_smoke with cameras, a fresh baseline and one stored sample through its gates all pass there, with only the files the benchmark reads copied over.

## Decisions (from grilling, the user's 2026-11-07 message)
- Docker image, built from `code/util/docker/Dockerfile` on corallab4. Rejected a native pip install: a second stack, no sudo saved in practice.
- corallab4 holds only what reproduces the benchmark: the repo, data/assets, and the inputs the code reads (list in Change). Rejected mirroring the box's data/: media, logs and old run folders stay on the box until its deletion, archived to the Mac only if the user asks.
- The refactor branch `worktree-agent-afdb8fb5e23a54661` is tested directly; `plan` is not installed first. Rejected plan-first: one window, one test.
- The box is started once in the same window for what the Mac lacks, then stopped; it is deleted only when the user says so.
- The user runs the sudo part (Docker, the NVIDIA toolkit, the docker group); the runner does the rest over ssh from the Mac.

## Change (no code change beyond compose.yaml)
- Mac: `git push origin worktree-agent-afdb8fb5e23a54661` so corallab4 can clone it (public repo, no credential there); add the ssh remote `corallab4` (`corallab4:isaac-launchable/dishwasher_sim_isaaclab`) for commits made there, pulled and pushed from the Mac like the old `brev` remote.
- corallab4: `git clone -b worktree-agent-afdb8fb5e23a54661 https://github.com/shu4dev/dishwasher_sim_isaaclab.git ~/isaac-launchable/dishwasher_sim_isaaclab`; `mkdir -p ~/dishsim-cache/{kit_cache,pip_cache,ov_data/{cache_ov,glcache,computecache,logs,data}}`.
- code/util/docker/compose.yaml: drop the whole-drive bind `/media/corallab-s1/2tbhdd/brianshu`; the seven cache binds get the host prefix `/home/shu4/dishsim-cache` in place of `/media/corallab-s1/2tbhdd/brianshu/dishsim`; one commit on the branch.
- Image: `docker build -f code/util/docker/Dockerfile -t dishsim-isaac:6.2.0 .` under setsid nohup, log to data/logs/bringup/build.log (20 to 40 min; the box took 1,470 s on 4 cores). Then `docker compose -f code/util/docker/compose.yaml up -d`; `docker logs dishsim-isaac` must show the two editable installs.
- Inputs from the Mac by rsync (all small but the robot holds): data/assets (13 MB), data/results/dot_rules/ceiling_62, data/results/bench/samples and samples_hw, data/results/bench/v3, data/results/sampler/plate_rest_62, data/results/dot_rules/dependency/samples_v5, data/results/robot/samples_v1 (835 MB: the holds the episode reads), data/results/robot/mount.
- Inputs from the box (one `brev start`, while the image builds): the fit caches (data/results/hotec/frigidaire/top5_*/cache/level0/fit_conflicts.json and fit_cycles.json, per family digest) and the rest of data/results/hotec/frigidaire that the Mac lacks, anything under data/results/bench/samples/_work and runs newer than the Mac's copy; `du -sh data/*` recorded in the note for the deletion decision; `brev stop` in the same turn.
- Baseline re-recorded on this machine (per-stack): `code/util/run_kit.sh code/sampler/scripts/frigidaire_initial_state_validate.py --headless --out-dir data/results/benchmark/frigidaire_hotec/baseline` ends in `[RESULT] baseline_ready`.
- Kit workers here: cpus j and j+8 are siblings (`taskset -c j,j+8`), at most three workers (31 GiB); every Kit log judged by content, never by exit code.

## Run
`code/util/run_py.sh -m pytest code -q` then `code/util/run_kit.sh code/util/tools/kit_smoke.py --headless --enable_cameras`, the baseline above, then one stored sample through its gates: `code/util/run_py.sh code/sampler/scripts/bench.py --from gate_initial --to certify --input data/results/bench/samples/frigidaire_fdpc4221as__hotec_wheatstraw__plate5-bowl5-cup5_a3__s0.json --out data/results/bench/samples`

## Works when
pytest gives the box's counts (263 passed and 24 subtests on 2026-11-01, the packed-model test failing as before); kit_smoke prints `[RESULT] PASS` with a non-black still; `baseline_ready`; s0 passes gate_initial and certify again on this machine. Then results/2026-11-07-corallab4-bringup.md, and the merge of the branch into plan, the box deletion and the CLAUDE.md and repo map edits wait for the user's word.

## Assumed / skipped
- Assumed: the repo path and cache folder above (the box's path, so the notes' commands hold); anonymous pull of nvcr.io/nvidia/isaac-sim:6.2.0 works (else `docker login nvcr.io` with the user's NGC key); if `--from gate_initial` is refused for a stored sample, the fallback is a fresh easy roster, `--kinds bowl --count bowl=7 --counter-allowance 3 --seed 0 --to gate_goal`.
- Skipped this round: the robot chain, the algorithms' episodes, pages, the archive of the box's media and logs, the CPU governor.
