# A pre-build plate pose: foot pivot to first rack contact, against the settled plates and r05_00027's plate hits
Slug: 2026-10-22-plate-pose-model | From: results/2026-10-22-dag-pose-model.md (diagnostic, no lit review)

## Goal
Decide whether a plate's rest pose can be predicted before the build well enough for the access DAG: the model must
reproduce r05_00027's 9 plate hits with no plate cycle and land every predicted plate within 3° of tilt and 10 mm of
position of its settled pose, on every built load whose record holds settled plates.

## Decisions (grilled 2026-10-22 13:40 ET)
- Why plates only: the pose-model run showed plates exact at their settled lean and bowls unpredictable; of the eight
  loads only r05_00027 has plates, so hits are judged on it alone and pose error carries the rest.
- Two rest modes exist on r05's settled plates (packed 114-122° of tilt): five lean 2-4° further than packed with a 28-41 mm
  shift in rack x (a pivot about the foot, not the centre), three stand at 83° and 59-60 mm higher (resting on the
  tines, not in the gap). A fixed-angle pivot cannot give both; a pivot to first contact can.
- Models: **F** foot-pivot to contact: the plate at `upright_pose` (lean bias included) lowered by the move's own descent
  against the appliance alone to where the rack stops it, then rotated about the horizontal axis through its lowest
  point (along the bank's y) toward its lean side in 1° steps until the first contact with the rack (the 1 mm allowance,
  FCL), or 60° at most. **A** the same pivot by the packed variant's lean angle (`lean-28` = 28°). **S** settled, the oracle
  (passes by construction). The three models give plate poses; bowls and cups take their settled poses in every model so
  that only the plates are under test. Rejected this round: angles fitted to the data, physics settles.
- Loads: hits on r05_00027 (`entries_chosen`, `entries_settled`, `access.hits`, 8 plates). Pose error on r05 and on every
  record in `data/results/dot_rules/dependency/samples_v5/`, `samples_v7/` (r02_00017) and `samples_v6b/` whose
  `entries_settled` holds plates (built loads, valid or not); list which were found.
- Pass (user's call): F or A passes when the 9 plate hits are reproduced exactly with no cycle among plates on r05, and
  every predicted plate is within 3° and 10 mm of its settled pose on every load found. Report per plate: predicted
  and settled tilt, rest mode (gap or tines), position error; per load: hits missed and spurious among plate pairs.
- One short box window (FCL and pxr are in the container), target 10 minutes.

## Change
- `code/src/sampler/order_check.py`: models F and A beside P, L, S, M, C (a `--plate-model` is fine), the per-plate pose
  error table, the plate-pair restriction of the hit comparison. No change to `order.py`, `build.py` or the kits.

## Run
Box: `code/util/run_py.sh code/src/sampler/order_check.py --records <r05 + every record with settled plates> --models
F,A,S --out data/results/sampler/plate_pose_model`; copy back; `brev stop`; confirm STOPPED. Note on the Mac.

## Works when
`results/2026-10-22-plate-pose-model.md` holds the per-plate table (model, tilt predicted/settled, mode, mm), the hit
comparison on r05 for F, A and S, the pass verdict per model, and one line on what the sampler should use for plates;
the box is STOPPED, window under 20 minutes.

## Assumed / skipped
Assumed: the lean side of each slot is `lean_side(variant)`; the rack stop for an upright plate is what L found
(85-90 mm up). Skipped: bowls and cups (settled poses in every model), any sampler run, the chain (a plate leaning on
its neighbour instead of the tine is a second-order effect this round does not model).
