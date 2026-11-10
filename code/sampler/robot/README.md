# sampler/robot

Block 1b ROBOT of `docs/repo_map.txt` (pick and place, gate): the native Isaac Sim 6.2 UR5e with an EPick-sized suction
body on its flange, since 2026-11-09. The package is `sampler.robot` (`ur5e`, `epick`, `kin`, `collide`, `rig`,
`harness`, `flags`, `triallog`); entry scripts in `scripts/`, Kit-free tests in `tests/`, reference in
[docs/robot.md](docs/robot.md).

The pinch layer (grasp.py, the pinch gates and probes, the robot episode, the reach, mount and site sweeps, the access
order, the placing loads, the report pages, the drop curve and the tines test) was deleted on 2026-11-09
(plans/2026-11-09-delete-pinch-layer.md); git keeps it at 9f700e1 on plan. The suction episode is the next round's code,
built from the EPick test's cycle, the rig and the harness. Until its gate exists, the sampler's hardware gate
(`code/sampler/hwgate.py`, `bench --hardware`) stops at its missing subprocess target `frigidaire_access_order.py`.

- Input: the bare arm streamed from NVIDIA's 6.2 asset root (`ur5e.UR5E_USD`; every result records its ETag), the
  HOTEC dishes `data/assets/hotec_wheatstraw/usd/`, the Frigidaire twin's
  `data/assets/frigidaire_fdpc4221as/components.json` and logged trials for the replay.
- Output: `data/results/robot/epick/` (the EPick test), `data/results/robot/arm_model.json`, the replay's mp4s and
  stills.

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
| `frigidaire_robot_replay.py` | Kit-free (Warp). Replay of a logged robot trial, since the box's RTX renderer crashes with cameras: the arm and gripper as the convex hulls of their USD collision meshes at the logged body poses, the dish, racks, counter and pedestal; two views (the episode camera, one following the dish), 10 fps mp4. `--strip` / `--strip-only` a six-frame strip, `--release-still` the release frame, `--landed-still` the last frame (the dish as it landed; a samples_v1 trajectory also carries the other dishes' kinds, the pedestal and the counter edge, so `--kind` can be left out; a plate presentation's backstop is drawn from the trajectory's `fixtures`), `--load-dir` one mp4 of a placing load's drops in order plus the goal and as-landed stills. | `--traj`, `--kind`, `--load-dir`, `--variant` (`D`; also `D-side`, `D-side-m2`, `D-flip`, `I`, `D-lip`, `I2`, `D-lip2`, `I3`, `D-perch`, `I4`, `D-seat`, `I5`, `D-sampler`, `D-roll`, `D-ring`, `D-ring-h100`), `--goals`, `--label`, `--out` (required), `--wh` (`640x480`), `--every` (1) | a trial's `traj_*.npz` and the `robot_meshes.json` beside it, or a `placing_v1/<load>/` folder (rows and trajectories); `--goals` (`goals_v4.json`) | the mp4 or PNG at `--out`; with `--load-dir` the `drop_`, `goal_`, `landed_<load>*` files in `--out` and entries upserted into `<out>/videos.json` |
| `frigidaire_run_video.py` | Mac (PIL: `/opt/anaconda3/bin/python3`; ffmpeg). The s0 rerun's stitched video (plans/2026-11-04-s0-rerun.md): a 3 s title card per dish (dish, kind, slot, move, verdict, cause) in the executed order on the dag's 15 positions, each followed by its replay clip, concatenated into one h264 yuv420p 1280 x 536 10 fps mp4; a contact sheet of the landed stills. A reject row gets its card and no clip. | `--crf` (26) | `<load dir>/{load.json, rows.jsonl}`, the clips and stills of the rows | `<media dir>/s0_grasp_place.mp4`, `<media dir>/contact_sheet.png`, the cards in `<load dir>/_video/` |
