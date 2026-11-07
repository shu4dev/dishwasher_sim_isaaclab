# The stop-motion videos on Isaac Lab 3.0: the video script on the kit's scene path, a video stage, the v2 pair's three videos
Slug: 2026-11-01-video-port | From: results/2026-11-01-rack-wheels-visual.md

## Goal
The v2 pair's three recorded episodes (greedy_offline, rrt_connect, mcts) get stop-motion videos rendered on Isaac Sim 6.2 from their stored records, and `data/media/bench/v2/index.html` embeds them under the run's own name, with the episodes untouched.

## Decisions (from grilling)
- The video script builds its scene the way the bench kit does (`IsaacInitialStateBackend`: the spawned twin, the dishes as RigidObjects, the kit's tints and `BENCH_CAMERA`), restores the stored start (racks at the start joints, dishes at `initial_snapshot.poses`) and, per executed move, writes `poses_after` for the dishes that moved and renders; no physics moves a dish. Rejected a ClonePlan patch on the raw-USD scene (the 3.0 API is not readable on the Mac, and re-referencing moved prims was a 4.5 renderer workaround) and a Kit-free Warp video (a second renderer, a different look). The retired planner-instance branch goes.
- A new `video` stage between run and report renders the videos the run's episode records lack; `--from video --to report` on a stored pair fills them in without re-running the episodes, and the run stage stops rendering. Rejected re-running from run (new episodes, about 40 min, MCTS may land elsewhere) and a flag on the run stage (its ledger entry would mean two things).
- This round delivers the v2 pair's three videos (first_fit has no record) and the page title from the run folder's name. Rejected videos only and a second easy seed (two changes in one round).
- One window, 1 h cap, after the Mac rewrite; every Kit process runs alone on all 8 threads. Rejected 45 min (no room for one fix attempt).
- Plan grill: the video stage renders only the videos a pair lacks, so a resumed window skips what passed and deleting an mp4 forces a new render (rejected always re-rendering); captions are the start frame, one line per move (dish -> counter, rack, or failed settle, put back) and an end frame with S_final and S_ref from the run stage's `.fair.json` (rejected per-move S from a Warp cast inside Kit, and bare frames); inside the window one fix and one rerun, a second traceback ends it with the note (rejected three fixes and a ClonePlan fallback route); the stage list changes in CLAUDE.md, the two rows that describe the script and the repo map (rejected CLAUDE.md only).

## Change
- `code/planner/frigidaire/frigidaire_planner_video.py`: `render()` on the kit's path (`make_backend`, `tint_scene` and the camera rig from `frigidaire_bench_kit.py`); `restore()` with the instance's start snapshot; dishes kinematic so a written pose holds; per executed move `set_rigid_pose` for each dish whose `poses_after` differs, one tick, the kit's renders, `grab_one`, caption (start; move k/n: dish -> counter / rack / failed settle, put back; end with S_final and S_ref from the run stage's `.fair.json`), `VideoWriter`. The empty-frame check, the evidence JSON and the `[RESULT]` line stay; `--instance`, `--episode`, `--out-dir`, `--video-width` keep their meaning.
- `code/src/bench/`: `episodes.py` stage_run no longer renders (rows keep `video: null`); new `video.py` with `stage_video` (for each episode row with a record and no mp4, run the video script with the media folder from the pair's run record; fill `video`; PASS when every record has a video); `sample.py` STAGES and STAGE_FN gain `video` between run and report; `schema/pair.json` ledger enum gains it; `report.py` title and h1 read `Merged benchmark <run folder>`.
- Docs: CLAUDE.md ("What this is" stages, Validation status line on videos), `code/planner/frigidaire/README.md` and `code/frigidaire/docs/planner.md` rows for the script, `docs/repo_map.txt` stage list.

## Run
Mac first: `.venv-kitfree/bin/python -m pytest code/tests/ code/frigidaire/tests/ -q` and `bench.py --help` lists `video`. Box, one window, self-halt armed at T0 + 1 h, each Kit process on all 8 threads, judged from its log:
```
code/util/run_py.sh code/planner/bench.py --from video --to report --input data/results/bench/v2/frigidaire_fdpc4221as__hotec_wheatstraw__bowl7_a3__s0.json --out data/results/bench/v2 --media data/media/bench/v2
```

## Works when
Three mp4 files in `data/media/bench/v2/<pair>/`, one per recorded episode, each with a PASS evidence JSON and no empty frame; the pair's ledger gains a `video` PASS entry while the three episode records' sha256 are unchanged; `index.html` is titled with v2 and embeds the three videos; the Mac's Kit-free pytest unchanged (237 passed, the 11 Warp, PIL and packed-model failures); results/2026-11-01-video-port.md written.

## Assumed / skipped
- Assumed: 640 px width, the script's fps and hold-frame defaults, `BENCH_CAMERA`; the stored v2 pair is the only input and its episodes are not re-run; the box already holds the wheels-visual twin (the hub upload is yours); the twin's pytest is not rerun.
- Skipped this round: first_fit's own load, the medium and hard build gap (next round), per-move S captions, more seeds, the pages.
