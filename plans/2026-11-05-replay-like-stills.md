# Replay clips that look like the benchmark stills
Slug: 2026-11-05-replay-like-stills | From: results/2026-11-05-fifteen-r3.md (round 3's 15 chain clips, data/results/robot/samples_v1/<s0>__fifteen_r3c)

## Goal
Round 3's 15 replay clips and the continuous video show the same scene as the goal and initial stills: the whole twin with the cabinet and door (the basket out, as in the runs), both racks with their dishes, each dish in the still's colour, seen from the still's camera, so the goal picture and the video read as one place.

## Decisions (the user, 2026-11-05 15:50 to 16:05 EST)
- The replay comes to the stills, not the stills to the replay (16:05 EST, with the basket and the other rack's timing decided at 16:10). Rejected re-rendering the stills in the replay's style (hulls on white) and a scene-only change: the user wants one look.
- Cabinet and door drawn from the twin's components.json (meshes plus solids, the asset's material colours: stainless, tub plastic), the door open at 90 deg, the racks in coated-wire grey; a dark ground and the stills' background and light. The basket is not drawn: it was lifted out for every robot run (`--no-basket`) and the hand crosses its volume in plate_01's clip; the header says the basket is out. Rejected drawing it for reference (a picture of a collision that never happened).
- Both racks with their dishes: the other rack stays at its logged pushed-in pose and shows what its own chain had placed at that point of the video order (time-consistent; the final state would show plates before their own clips). A dish that missed its seat stays visible at its landed pose (rows.jsonl final_pose) in the later clips of its rack, shifted with the rack.
- Colours: item_tints over the sorted dish ids, as the stills (bowl_01 #e69c67 ... plate_05 #9267e6); the current dish full, the earlier ones unchanged (no lightening); the arm and pedestal as today.
- Camera: the stills' BENCH_CAMERA (eye 2.05, -2.65, 2.05; look-at 0, -0.22, 0.62; vertical fov 30.23 deg at 4:3) as the main view at 1280 x 960, the dish-following view as a 320 x 240 inset bottom right, the 56 px header on top (1280 x 1016); the pedestal and arm stand right of the racks and hide nothing.
- Scope: round 3's 15 clips re-rendered in place (the old clips kept as clip_<dish>_v1.mp4), the continuous video rebuilt with no title cards in the chain order (upper rack, then lower) plus a copy under 30 MB; round 2 untouched. One box window of about 45 min (4x the pixels of today's 23 to 44 s per clip), started on the user's go after the Mac checks.

## Change (one runner, Opus)
- code/execution/frigidaire/frigidaire_robot_replay.py: statics() adds the cabinet and door (a solids-to-triangles helper for boxes and cylinders beside the meshes; `visual: False` skipped); the dish colours from frigidaire_initial_state_render.item_tints; the main camera and the inset layout; new inputs `--load <load.json> --rows <rows.jsonl> --states <state dir> --video-order <names>` to draw the other rack's dishes and the landed failed dishes; the header's second line names the composite ("basket out; the other rack's dishes as placed by their own chain").
- frigidaire_dot_rules_render.py's render(): background, light and a ground box as parameters (defaults unchanged).
- data/logs/robot_replay_v2/: the window scripts (start, sync by rsync with md5, the 15 renders on cores 3/7, the concat, copy-back, stop) and the Mac check (scene assembly without Warp: triangle counts per component, the camera projection of the rack centres and the pedestal, the tint table).

## Run
Mac: `PYTHONPATH=code/src .venv-kitfree/bin/python code/execution/frigidaire/frigidaire_robot_replay.py --check-scene <r3c args>` (no Warp: counts and projections). Box: for each of the 15 dishes `code/util/run_py.sh code/execution/frigidaire/frigidaire_robot_replay.py --traj <npz> --load ... --rows ... --states ... --out <r3c media>/clip_<dish>.mp4` on cores 3/7, then the concat to s0_fifteen_r3_continuous.mp4 and the small copy. Note results/2026-11-05-replay-like-stills.md.

## Works when
The 15 new clips and the continuous video show the cabinet, door, both racks' dishes and the stills' colours from the stills' camera, with no traceback; the goal still and the video's last frame of each rack match dish for dish.

## Assumed / skipped
- Assumed: the trial npz and rows.jsonl of round 3 hold every pose needed (the other rack's dishes come from the state files, the failed dishes' from rows final_pose); the counter stays at the runs' front edge (-0.17).
- Skipped: round 2's clips, the stills, new tests, a Mac Warp path.
