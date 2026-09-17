# Exposure scorer (Frigidaire FDPC4221AS)

Reference for agents working on `frigidaire/src/dishsim_frigidaire/exposure.py` and its
scripts. Human quickstart: [exposure_quickstart.md](exposure_quickstart.md).

## What it is

A Kit-free, ray-cast proxy for "how well would this arrangement wash". It ranks arrangements
of the SAME object set; absolute values are not comparable across different sets. It is not a
cleaning measurement and is never quoted as one. Chosen 2026-09-17 over (a) simulating the
wash (months, uncheckable without residue data) and (b) a measured spray-arm model (needs
the real unit); one idea only, no asset changes.

## Definition (revision 4)

Objects `o` with food-contact area `A_o`; samples `p_j` (N = 500 face centroids, weight
`A_o/N`) with outward normals `n_j`; source points `q_k` with weights `u_k` for the object's rack.

```
d_jk = (q_k - p_j) / |q_k - p_j|
v_jk = 1 if the segment p_j + eps*n_j -> q_k hits nothing in the load, else 0
c_jk = max(n_j . d_jk, 0)                      impingement weight
e_j  = sum_k u_k c_jk v_jk / sum_k u_k c_jk    0 when the denominator is 0
E_o  = mean_j e_j
S    = sum_o A_o E_o / sum_o A_o               primary;  W = min_o E_o  secondary
feasible iff no vessel pools: min z(interior vertices) >= min z(rim ring) - 2 mm
```

- Load = every dish including the object itself (mug handle included), both racks, the
  basket. Tub, door, cabinet never occlude. `eps` = 0.1 mm, rays die at 2 m.
- Sources: 64-point equal-area disc under each rack, the arm's sweep. Lower arm centre
  (0, 0.008, 0.185) r 0.245 for LowerRack and basket objects; middle arm (0, 0.008, 0.540)
  r 0.205 for UpperRack objects. Optional ceiling point (0, 0.018, 0.817) with
  `ceiling_weight` (default 0). All from asset dimensions: ASSUMPTIONS, recorded in every score.
- Food-contact surface = inner lathe rings of the tableware mesh (vessel interiors, plate
  top). Vertex index `>= 1 + (n_sections-1)*96` is inner; rim band and mug handle excluded.
  Only lathed kinds are supported (`SECTIONS` in `exposure.py`); cutlery raises.
- Legacy `source` modes kept for comparison: `hemisphere` (rev 1 uniform AO), `below`
  (rev 2), `per-rack-directions` (rev 3). `mouth_up` (organized-policy angle rule) is still
  recorded beside `pools`; feasibility follows `pools`.

`FORMULA.md` under `results/exposure/frigidaire/` is written by the summary script from the
`FORMULA` constant in `frigidaire_exposure_summary.py`; keep that constant and this section
in agreement.

## Files

| File | Role |
|---|---|
| `src/dishsim_frigidaire/exposure.py` | scorer: `food_contact`, `surface_samples`, `rack_sources`, `cast` (Warp), `exposure_of`, `pools`, `load_state`, `score_state`, `score_arrangement`, `sanity_pair` |
| `scripts/evaluation/frigidaire_exposure_demo.py` | one pair: sanity/pair/bars/evidence png, turntable.gif, rays.gif |
| `scripts/evaluation/frigidaire_exposure_summary.py` | all seven pairs, `--source`, `--ceiling-weight`, `--ceiling-sweep`, `--convergence <pair>`; writes FORMULA.md |
| `scripts/evaluation/frigidaire_exposure_search.py` | samples feasible alternatives from the organized candidate pool (random greedy + MILP), scores them |
| `scripts/evaluation/frigidaire_exposure_settled_best.py` | turns a settled attempt folder into `state.json`, re-scores, draws `settled_best.png` |
| `tests/test_exposure.py` | 15 Kit-free tests (analytic ray cases, lathe layout, loader re-seating, pooling, feasibility) |

Inputs are the settled initial states under `results/initial_states/frigidaire/*_20260911_seed20260911/states/`.
They store the racks OUT (door 90°, slides at their limits); `load_state` re-seats every
object from `rack_local_pose` onto the racks-in origins in `asset.BODY_POSITIONS` and
re-seats the basket by its settled offset to the LowerRack. Each organized state re-arranges
exactly the objects of the packing state with the same name (seven pairs, bowls and mugs).

## Commands

All Kit-free unless marked; run from the repo root. Warp runs on the CPU device by default
(`--device cuda` optional). Outputs land under `results/exposure/frigidaire/` (2 TB drive,
root-owned; clean with `docker exec dishsim-isaac rm`).

```bash
scripts/run_py.sh -m pytest frigidaire/tests/test_exposure.py                       # 3 s
scripts/run_py.sh frigidaire/scripts/evaluation/frigidaire_exposure_demo.py --pair random_06 --device cpu   # 3 min, GIFs dominate
scripts/run_py.sh frigidaire/scripts/evaluation/frigidaire_exposure_summary.py --device cpu --ceiling-sweep --convergence random_06   # 6 min
scripts/run_py.sh frigidaire/scripts/evaluation/frigidaire_exposure_search.py --pair random_06 --device cpu   # 5 min
```

Settling a search proposal (Kit, CPU physics, ~4 min per run). The claims evidence script
`frigidaire_full_load_evidence.py` REJECTS mugs in the lower rack; use the organized backend:

1. Build a manifest: candidates for the chosen `indices` from
   `organized_20260911_seed20260911/candidates_screened.json`, identities via
   `assign_identities(selected, source_state['objects'], source_state['baseline'])`
   (import from `frigidaire/scripts/experiment/frigidaire_organized_experiment.py` with
   `frigidaire/scripts/experiment` on `sys.path`), plus `policy` and `baseline` copied from
   the source organized state, `order: upper_first`, `schema_version: 1`.
2. `scripts/run_kit.sh frigidaire/scripts/experiment/frigidaire_organized_validate.py --usd build/frigidaire_collection/usd/fdpc4221as.usdc --out-dir <dir> --max-wall-seconds 480 --headless --device cpu --order upper_first --manifest <dir>/manifest.json`
   Judge by `[RESULT] accepted` in the log; `result.json` carries the measured snapshots.
3. `scripts/run_py.sh frigidaire/scripts/evaluation/frigidaire_exposure_settled_best.py <dir>` → `state.json`, `scores.json`, `settled_best.png`.
4. Reproduce: same manifest with `objects` = the settled `state.json` objects,
   `purpose: reproduction`, `reproduction_of: <first attempt_id>`; run step 2 into `<dir>/replay`;
   the state counts as accepted only when the replay is accepted too (the organized
   pipeline's standard). Record the verdict in `state.json["reproduction"]`.

Isaac render of any state (Kit, GPU, ~2 min): `frigidaire_initial_state_render.py --headless
--enable_cameras --state <run>/states/<id>.json --out-dir <out>`. It validates a run layout:
`<run>/attempts/<state.source_attempt>/result.json` equal to `state.validation`,
`state.input_hashes.asset_hashes` (copy from a packing state) or `<run>/summary.json`,
`purpose` in {`randomized`, `highest`}, `accepted: true`, and it refuses to overwrite an
existing image or evidence file: use a fresh `--out-dir`.

## Results on record (2026-09-17)

| Result | Value |
|---|---|
| seven pairs, organized minus packing | +0.121 to +0.215; organized wins 7/7; every packing state infeasible (pooling) |
| ceiling-weight sweep | organized wins 7/7 at weight 0, 0.25, 0.5; 1/7 at 1.0 |
| convergence (random_06, 32–128 directions × 200–1000 samples) | scores move ≤ 0.001 |
| sanity | bowl mouth-down alone 0.613; plate 15 mm below its rim 0.026 |
| search, random_06 (9 bowls + 9 mugs) | 110 proposals, 0.221–0.274, median 0.246; settled organized 0.242 |
| best proposal | 0.274, settled and independently reproduced (pose difference 0.0 mm / 0.018°), all gates passed |

Folders: `summary/` (per-rack default), `summary_below/` (rev-2 rays for comparison),
`demo_random_0N/` (per pair, with `isaac/` renders), `search/` and `search/settle_best/`.

## Landmines

- Warp kernels must be defined in a file; `python -c` strings fail at kernel compile. The
  kernel cache is redirected to `outputs/warp_cache` (2 TB drive) at import.
- The container was moved to GPU 1 on 2026-09-17 (`DISHSIM_GPU=1 docker compose ... up -d`)
  because foreign jobs fill GPU 2; RTX renders need a GPU with room, the scorer does not.
- `ffmpeg` is absent in the container: animations are GIFs via Pillow.
- Draining is looser than the organized policy's angle rule: this shallow bowl traps only
  about 1 mm at 100° from mouth-down, so `pools` is False there while `mouth_up` is True.
- Scores are only comparable across arrangements of the same objects; each object has its
  own self-occlusion ceiling (`baseline` in the record).

## Limitations to state with any result

Line-of-sight only (no splash, sheeting or pooling dynamics); direction-blind within a
source disc ("toward the centre" is invisible); arm discs, ceiling point and cosine weighting
are assumptions, not measurements; the ceiling nozzle's weight decides the raw ranking above
0.5; bowls and mugs only so far; search proposals are FCL-feasible, not settled, until run
through step 2 above.
