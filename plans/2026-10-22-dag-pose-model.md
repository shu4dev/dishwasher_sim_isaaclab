# Which pose model gives the pre-build access DAG: packed, lowest-clear-step or settled
Slug: 2026-10-22-dag-pose-model | From: results/2026-10-22-dag-build-order.md (diagnostic, no lit review)

## Goal
Find the pose model at which the pre-build DAG (`order.access_edges` + `order.support_edges`) reproduces the certified
access hits exactly and is acyclic, on the eight loads that have packed, chosen and settled poses with certified hits,
so the next sampler run computes its DAG at poses that predict the build instead of contradicting it.

## Decisions (grilled 2026-10-22 12:30 ET)
- Loads (8): r05_00027 (`data/results/dot_rules/dependency/samples_v5/r05_00027.json`: `entries`, `entries_chosen`,
  `entries_settled`, `access.hits` = 11) and the seven valid easy builds of today's DAG run (seeds 4, 7, 8, 12, 13,
  15, 17: `data/results/sampler/dag_easy/_work/<pair>/goal/<load>.json`, same fields; their `*.order.json` beside the
  load files). Nothing else is sampled.
- Pose models, each a full set of poses for the load: **P** packed = `entries_chosen` (what the build was given);
  **L** lowest-clear-step = each dish alone, the move's own descent (from 20 mm above the rim, 5 mm steps, the 1 mm
  allowance) against the appliance only, stopped where the appliance stops it; plates at `upright_pose` with the 3 mm
  lean bias, bowls and cups at the packed orientation without the `lift`; **S** settled = `entries_settled`;
  **M** mixed = plates from S, bowls and cups from L (which kind breaks P). Any other model is a later round.
- Edges: the functions as they are in `code/src/sampler/order.py` (access = the kit's `access_hits` path check at the
  model's poses, support = the vertical probe), one run per model per load.
- Pass rule (user's call): a model passes a load when its access edge set equals the certified `access.hits` pairs
  (a, b) exactly and the DAG is acyclic; a model passes when all 8 loads pass. Report k of 8 per model, and per load
  the missed and spurious edges and any cycle. The witness order's consistency with each DAG is reported too, as
  information, not as the pass rule.
- Kit-free but container-bound: the FCL checker and `pxr` live in the container, so the run is one short box window
  (start, run, stop; about 10 minutes). No Isaac, no sampling, no sampler change.

## Change
- `code/src/sampler/order_check.py` (new, small): `--records` (kit records), `--models P,L,S,M`, `--out DIR`; builds
  each model's poses (L through the bench's descent helpers already used by the move), calls `access_edges` and
  `support_edges`, compares with the record's hits, writes `table.md` and one JSON per load and model.
- No change to `order.py`, `build.py` or the kit.

## Run
Box: `code/util/run_py.sh code/src/sampler/order_check.py --records <the 8> --models P,L,S,M --out
data/results/sampler/dag_pose_model`; copy the out dir back; `brev stop`; confirm STOPPED. Then on the Mac the note.

## Works when
`results/2026-10-22-dag-pose-model.md` holds the 4 x 8 table (pass / missed / spurious / cycle per cell), the k of 8
per model, the plate-versus-bowl diagnosis from M, and one line naming the model to use pre-build or saying none of
the four does and why; the box is STOPPED and the window is under 20 minutes.

## Assumed / skipped
Assumed: `entries_settled` and the hits in each record come from the same certified rebuild. Skipped: any rerun of
the sampler, Isaac settles of single dishes, new pose models.
