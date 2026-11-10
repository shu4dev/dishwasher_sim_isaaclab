# sampler/robot

Block 1b ROBOT of `docs/repo_map.txt` (pick and place, gate): the native Isaac Sim 6.2 UR5e with an EPick-sized suction
body on its flange, since 2026-11-09. The package is `sampler.robot` (`ur5e`, `epick`, `kin`, `collide`, `rig`,
`harness`, `flags`, `triallog`); entry scripts in `scripts/`, Kit-free tests in `tests/`, reference in
[docs/robot.md](docs/robot.md).

The pinch layer (grasp.py, the pinch gates and probes, the robot episode, the reach, mount and site sweeps, the access
order, the placing loads and the report pages) was deleted on 2026-11-09 (plans/2026-11-09-delete-pinch-layer.md); git
keeps it at 9f700e1 on plan. The suction episode is the next round's code, built from the EPick test's cycle, the rig and
the harness. Until its gate exists, the sampler's hardware gate (`code/sampler/hwgate.py`, `bench --hardware`) stops at
its missing subprocess target `frigidaire_access_order.py`.

- Input: the bare arm streamed from NVIDIA's 6.2 asset root (`ur5e.UR5E_USD`; every result records its ETag), the
  HOTEC dishes `data/assets/hotec_wheatstraw/usd/`, the Frigidaire twin `data/assets/frigidaire_fdpc4221as/usd/fdpc4221as.usdc`,
  logged trials for the replay.
- Output: `data/results/robot/epick/` (the EPick test), `data/results/robot/arm_model.json`, the replay's mp4s and
  stills, the drop-curve and tines records under `data/results/robot/`.

## Commands, in order

```bash
code/util/run_kit.sh code/sampler/robot/scripts/frigidaire_epick_test.py --headless --enable_cameras
code/util/run_kit.sh code/sampler/robot/scripts/frigidaire_arm_model.py --headless
```

## Scripts

The EPick test and the arm-model export for the new hardware, then the scripts kept from the hardware-limits study of
2026-10-23 to 2026-10-30 and the s0 rounds (reference `code/sampler/robot/docs/robot.md`). Defaults are defined in the
script itself unless another file is named. Kit = through `code/util/run_kit.sh ... --headless`; Kit-free =
`code/util/run_py.sh` or the Mac's python.

| Script | What it does | Key parameters (default) | Reads | Writes |
|---|---|---|---|---|
| `frigidaire_epick_test.py` | Kit (`--enable_cameras` for the stills; CPU physics, scene queries on). Plan A of plans/2026-11-09-native-ur5e-epick.md: step 0 records the streamed arm file's facts (ETag, layers, variant sets, articulation root, joints, bodies, every prim); step 1 teleports the arm to 20 joint poses and compares wrist_3 with `kin.fk_wrist3`, the fixed offset calibrated at HOME_Q; step 2 picks the plate face up, the bowl and the cup mouth down on their flat patches with the surface gripper (close 150 ms, lift 100 mm, hold 5 s, set down, open 180 ms) and takes an RTX still of each held dish. | none | the 6.2 arm, the HOTEC dishes | `data/results/robot/epick/epick.json`, `held_<dish>.png` |
| `frigidaire_arm_model.py` | Kit. Exports the arm collision model (hull vertices per link, link-frame offsets to the analytic FK) for Kit-free checks: the bare 6.2 arm's link hulls and the EPick body's hull under the `gripper` key (`epick.py`'s cylinder in wrist_3_link's frame). First run in the suction-gate round. | none | the 6.2 arm, `epick.py` | `data/results/robot/arm_model.json` |
| `frigidaire_robot_replay.py` | Kit-free (Warp). Replay of a logged robot trial, since the box's RTX renderer crashes with cameras: the arm and gripper as the convex hulls of their USD collision meshes at the logged body poses, the dish, racks, counter and pedestal; two views (the episode camera, one following the dish), 10 fps mp4. `--strip` / `--strip-only` a six-frame strip, `--still` a Kit-free still of a pre-place pose, `--release-still` the release frame, `--landed-still` the last frame (the dish as it landed; a samples_v1 trajectory also carries the other dishes' kinds, the pedestal and the counter edge, so `--kind` can be left out; a plate presentation's backstop is drawn from the trajectory's `fixtures`), `--load-dir` one mp4 of a placing load's drops in order plus the goal and as-landed stills. `--still` imports `frigidaire_robot_reach.py` (deleted on 2026-11-09 and in git at 9f700e1), so that mode stops at its import. | `--traj`, `--kind`, `--load-dir`, `--variant` (`D`; also `D-side`, `D-side-m2`, `D-flip`, `I`, `D-lip`, `I2`, `D-lip2`, `I3`, `D-perch`, `I4`, `D-seat`, `I5`, `D-sampler`, `D-roll`, `D-ring`, `D-ring-h100`), `--goals`, `--label`, `--out` (required), `--wh` (`640x480`), `--every` (1) | a trial's `traj_*.npz` and the `robot_meshes.json` beside it, or a `placing_v1/<load>/` folder (rows and trajectories); `--goals` (`goals_v4.json`) | the mp4 or PNG at `--out`; with `--load-dir` the `drop_`, `goal_`, `landed_<load>*` files in `--out` and entries upserted into `<out>/videos.json` |
| `frigidaire_drop_curve.py` | Kit (boot-first, one load per process, CPU physics). Each dish of a placing load ALONE (no robot) dropped from the sampler's release orientation at each gap above the rack stop, the earlier dishes at their goals, 150 + 60 settle ticks; judged seated on the goal's wires (cups: the goal pose within 2 mm / 5 deg), mouth, offsets and drift. `--goals` drops from the refreshed goals plus the gap; `--perch-roll` releases each bowl at its first tine contact per tilt; `--refresh-goals` settles each dish alone 2 mm above its recorded goal (the refreshed goal); `--rejudge` re-judges a `drops.jsonl` Kit-free. It imports `frigidaire_robot_episode.py` and `frigidaire_robot_reach.py` (deleted on 2026-11-09 and in git at 9f700e1), so every mode stops at that import. | `--load` (required), `--gap-mm` (`20,40,60,80,100,150`), `--repeats` (5), `--jitter` (`2mm,2deg`), `--dishes` (all), `--out-root` (`data/results/robot/drop_curve`), `--baseline` (`data/results/benchmark/frigidaire_hotec/baseline/result.json`), `--usd` (`data/assets/frigidaire_fdpc4221as/usd/fdpc4221as.usdc`), `--max-wall-seconds` (9000), `--goals`, `--bowl-supports` (`data/results/robot/kitfree_v1/insert_bowl/goal_supports.json`), `--perch-roll` (tilts), `--refresh-goals`, `--refresh-tag` (`all`), `--rejudge` | the load, the bench baseline, the twin USD, the bowl supports, `--goals` | `<out-root>/<load>/{drops.jsonl, summary.csv, highest.json, report.json}` (with `--perch-roll`: `best_tilt.json` under `data/results/robot/perch_roll/`); `--refresh-goals`: `<load dir>/goals_v4_<tag>.json` and `data/results/robot/goal_refresh/<load>_<tag>/`; refuses to overwrite |
| `frigidaire_tines_test.py` | Kit, no robot in the scene. Does the rack alone tilt a level bowl at the right-side lower sites? Per site and trial: the racks as in the robot trials, the bowl teleported mouth up and level to the site's clear-height pose plus the release offset with a seeded +-5 mm x, y perturbation, the benchmark's per-move settle, then a rest to 5 s; per site, a tilt past 15 deg means the rack, level means the robot. It imports `frigidaire_robot_episode.py` for `limits_instance` (deleted on 2026-11-09 and in git at 9f700e1), so it stops at that import. | `--n` (5), `--seed` (0), `--sites` (`lower_frontright@x+0.06`, `@x+0.08`, `@x+0.13`, `lower_rearright@x-0.02`), `--reach` (`data/results/robot/limits_v1/reach.json`), `--out` (`data/results/robot/exhaustive_w1/tines`; refuses to overwrite), `--usd` (`data/assets/frigidaire_fdpc4221as/usd/fdpc4221as.usdc`) | `--reach`, the twin USD | `<out>/trials.jsonl`, `<out>/tines.json` |
| `frigidaire_run_video.py` | Mac (PIL: `/opt/anaconda3/bin/python3`; ffmpeg). The s0 rerun's stitched video (plans/2026-11-04-s0-rerun.md): a 3 s title card per dish (dish, kind, slot, move, verdict, cause) in the executed order on the dag's 15 positions, each followed by its replay clip, concatenated into one h264 yuv420p 1280 x 536 10 fps mp4; a contact sheet of the landed stills. A reject row gets its card and no clip. | `--crf` (26) | `<load dir>/{load.json, rows.jsonl}`, the clips and stills of the rows | `<media dir>/s0_grasp_place.mp4`, `<media dir>/contact_sheet.png`, the cards in `<load dir>/_video/` |
