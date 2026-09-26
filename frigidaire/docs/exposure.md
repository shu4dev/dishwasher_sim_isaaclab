# Exposure scorer (Frigidaire FDPC4221AS)

Reference for agents working on `frigidaire/src/dishsim_frigidaire/exposure.py` and its
scripts. Human quickstart: [exposure_quickstart.md](exposure_quickstart.md).

## What it is

A Kit-free, ray-cast proxy for "how well would this arrangement wash". It ranks arrangements
of the SAME object set; absolute values are not comparable across different sets. It is not a
cleaning measurement and is never quoted as one. Chosen 2026-09-17 over (a) simulating the
wash (months, uncheckable without residue data) and (b) a measured spray-arm model (needs
the real unit); one idea only, no asset changes. Revision 5 (2026-09-20) is the closing
revision: it is the objective the arrangement planner consumes, and nothing in the
definition is tuned to a result.

## Definition (revision 5)

Objects `o` with food-contact area `A_o`; samples `p_j` (N = 500 face centroids, weight
`A_o/N`) with outward normals `n_j`; source points `q_k` with weights `u_k` for the object's rack.

```
d_jk = (q_k - p_j) / |q_k - p_j|
v_jk = 1 if the segment p_j + eps*n_j -> q_k hits nothing in the load, else 0
c_jk = max(n_j . d_jk, 0)                      impingement weight
e_j  = sum_k u_k c_jk v_jk / sum_k u_k c_jk    0 when the denominator is 0
E_o  = mean_j e_j
S    = sum_o A_o E_o / sum_o A_o               primary;  W = min_o E_o  secondary
feasible iff no vessel or plate pools: min z(interior vertices) >= min z(rim ring) - 2 mm
```

- Load = every dish including the object itself (mug handle included), both racks, the
  basket (a pure wire cage, fully occluding). Tub, door, cabinet never occlude. `eps` = 0.1 mm,
  rays die at 2 m.
- Sources: 64-point equal-area disc under each rack, the arm's sweep. Lower arm centre
  (0, 0.008, 0.185) r 0.233 for LowerRack and basket objects; middle arm (0, 0.008, 0.540)
  r 0.191 for UpperRack objects (`ARM_SOURCES`; since 2026-09-22 half the tape-measured rim
  width minus the previous margin, formerly 0.245 / 0.205). All from asset dimensions:
  ASSUMPTIONS, recorded in every score.
- Ceiling point (0, 0.018, 0.817) for UpperRack objects with weight `ceiling_weight`,
  **default 0** (arm discs only). The unit is not accessible, so the nozzle was never
  inspected; instead of a tuned weight the robustness claim is the sweep: organized wins 7/7 at
  weights 0, 0.25 and 0.5 (0.5 is the largest swept value that keeps 7/7) and 1/7 at 1.0,
  which is degenerate because it removes the middle arm and gives every mouth-down upper-rack
  vessel exposure 0 by construction. Only samples whose interior normal faces the ceiling
  ever gain from it, i.e. mouth-up vessels.
- Food-contact surface per kind (`SECTIONS`/`RING` in `exposure.py`): the inner lathe
  rings of bowls, mugs, tumblers, plates (top face) and spoon bowls (vertex index
  `>= 1 + (n_sections-1)*ring`, ring 96 for the 24-sector kinds, 80 for spoons; rim band and
  mug handle excluded); fork tines (the four tine meshes, both faces); the knife blade (faces
  wholly above the bolster ring at local z 4 mm, both faces). Handles never count. Anything
  else raises.
- Pooling applies to vessels and plates (a flat plate holds water in its well, a plate on
  edge drains); cutlery never pools. Pooling is a hard gate for the search and the planner,
  never a penalty. `mouth_up` (organized-policy angle rule) is still recorded beside `pools`.
- `baselines` (per-object exposure alone at its pose, the self-occlusion ceiling) is a
  diagnostic that never enters `S`; it is opt-in (`score_arrangement(..., baselines=False)`
  in the objective path, on in the reporting scripts). Rays run on CUDA when Warp sees a GPU
  (`exposure.DEVICE`, `--device cpu` to force); CPU and CUDA scores agree to about 1e-6
  (ray hits at triangle edges can differ: the best proposal scores 0.2737044 on CPU and
  0.2737058 on CUDA), so quote three decimals and never compare across devices at more.
- Legacy `source` modes kept for comparison: `hemisphere` (rev 1 uniform AO), `below`
  (rev 2), `per-rack-directions` (rev 3).

`FORMULA.md` under `results/exposure/frigidaire/` is written by the summary script from the
`FORMULA` constant in `frigidaire_exposure_summary.py`; keep that constant and this section
in agreement.

## Files

| File | Role |
|---|---|
| `src/dishsim_frigidaire/exposure.py` | scorer: `food_contact`, `surface_samples`, `rack_sources`, `cast` (Warp), `exposure_of`, `pools`, `load_state`, `score_state`, `score_arrangement`, `sanity_pair` |
| `scripts/evaluation/frigidaire_exposure_scene.py` | demo scene: organized random_06 + plate on edge + fork/knife/tablespoon in the basket -> `scene/<pair>_plus.json`, `_scores.json`, `_method.png` (the method figure) |
| `scripts/evaluation/frigidaire_exposure_scene_render.py` | Kit, `--enable_cameras`: Isaac RTX still of that scene and, with `--orbit-seconds N`, `<state_id>_orbit.mp4` (camera sweeps across the front, 1280x960, H.264 via imageio's bundled ffmpeg); no physics step |
| `scripts/evaluation/frigidaire_exposure_demo.py` | one pair: sanity/pair/bars/evidence png, turntable.gif, rays.gif |
| `scripts/evaluation/frigidaire_exposure_summary.py` | all seven pairs, `--source`, `--ceiling-weight`, `--ceiling-sweep`, `--convergence <pair>`; writes FORMULA.md |
| `scripts/evaluation/frigidaire_exposure_search.py` | samples feasible alternatives from the organized candidate pool (random greedy + MILP), drops pooling proposals, ranks the rest |
| `scripts/evaluation/frigidaire_exposure_settled_best.py` | turns a settled attempt folder into `state.json`, re-scores proposal and settled state at the same parameters, draws `settled_best.png` |
| `tests/test_exposure.py` | 22 Kit-free tests (analytic ray cases, lathe layout incl. spoons, fork/knife rules, loader re-seating, pooling incl. plates, feasibility, baselines opt-in) |

Inputs are the settled initial states under `results/initial_states/frigidaire/*_20260911_seed20260911/states/`.
They store the racks OUT (door 90°, slides at their limits); `load_state` re-seats every
object from `rack_local_pose` onto the racks-in origins in `asset.BODY_POSITIONS` (basket
objects onto the basket's own settled pose) and re-seats the basket by its settled offset to
the LowerRack. Each organized state re-arranges exactly the objects of the packing state with
the same name (seven pairs, bowls and mugs). No settled state contains a plate or cutlery;
the demo scene poses them geometrically.

## Commands

All Kit-free unless marked; run from the repo root. Warp runs on CUDA when available
(`--device cpu` to force). Outputs land under `results/exposure/frigidaire/` (2 TB drive,
root-owned; clean with `docker exec dishsim-isaac rm`).

```bash
scripts/run_py.sh -m pytest frigidaire/tests/test_exposure.py                                       # 3 s
scripts/run_py.sh frigidaire/scripts/evaluation/frigidaire_exposure_scene.py --pair random_06      # 6 s: scene json, scores, method.png
scripts/run_kit.sh frigidaire/scripts/evaluation/frigidaire_exposure_scene_render.py --headless --enable_cameras \
    --state results/exposure/frigidaire/scene/random_06_plus.json --out-dir results/exposure/frigidaire/scene \
    --orbit-seconds 12   # ~4 min: Isaac still + 12 s orbit video (omit --orbit-seconds for the still alone)
scripts/run_py.sh frigidaire/scripts/evaluation/frigidaire_exposure_demo.py --pair random_06        # 3 min, GIFs dominate
scripts/run_py.sh frigidaire/scripts/evaluation/frigidaire_exposure_summary.py --ceiling-sweep --convergence random_06   # ~1 min CUDA
scripts/run_py.sh frigidaire/scripts/evaluation/frigidaire_exposure_search.py --pair random_06     # ~2 min, MILP dominates
scripts/run_py.sh frigidaire/scripts/evaluation/frigidaire_exposure_settled_best.py                # 10 s
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

Isaac render of any settled state (Kit, GPU, ~2 min): `frigidaire_initial_state_render.py --headless
--enable_cameras --state <run>/states/<id>.json --out-dir <out>`. It validates a run layout:
`<run>/attempts/<state.source_attempt>/result.json` equal to `state.validation`,
`state.input_hashes.asset_hashes` (copy from a packing state) or `<run>/summary.json`,
`purpose` in {`randomized`, `highest`}, `accepted: true`, and it refuses to overwrite an
existing image or evidence file: use a fresh `--out-dir`. It rejects cutlery and the basket
rack; the demo scene has its own renderer (above).

## Results on record (2026-09-20, revision 5, ceiling weight 0)

Stale (2026-09-22): every score under `results/exposure/frigidaire/` was computed on the v3
racks and basket (archived as `build/frigidaire_collection/history/v3`) with arm radii
0.245 / 0.205. The racks and basket were rebuilt to tape measurements on 2026-09-22;
re-score at the current parameters before comparing with any v4 result.

| Result | Value |
|---|---|
| seven pairs, organized minus packing | +0.121 to +0.215; organized wins 7/7; every packing state infeasible (pooling) |
| ceiling-weight sweep (the robustness claim) | organized wins 7/7 at weight 0, 0.25, 0.5; 1/7 at the degenerate 1.0 |
| convergence (random_06, 32–128 directions × 200–1000 samples) | ordering preserved, scores move ≤ 0.001 |
| sanity | bowl mouth-down alone 0.613; plate 15 mm below its rim 0.026 |
| search, random_06 (9 bowls + 9 mugs) | 110 feasible proposals, 0 dropped for pooling; best 0.274, settled organized 0.242 |
| best proposal | 0.274 unsettled, 0.274 settled (difference 3e-5 at the same parameters and device), independently reproduced, all gates passed |
| demo scene (`scene/`) | organized random_06 + plate on edge + fork, knife, tablespoon: 22 objects, score 0.233, feasible; plate 0.174, fork 0.108, knife 0.176, tablespoon 0.080 (the basket cage and neighbours shade most rays) |

Folders: `summary/` (default), `summary_below/` (rev-2 rays for comparison), `scene/`
(revision-5 demo: Isaac still, orbit video, method figure), `demo_random_0N/` (per pair, with `isaac/`
renders; 00–05 were written under revision 3 and only differ in schema, the summary table is
authoritative), `search/` and `search/settle_best/`.

## Landmines

- Warp kernels must be defined in a file; `python -c` strings fail at kernel compile. The
  kernel cache is redirected to `outputs/warp_cache` (2 TB drive) at import.
- The container was moved to GPU 1 on 2026-09-17 (`DISHSIM_GPU=1 docker compose ... up -d`)
  because foreign jobs fill GPU 2; RTX renders need a GPU with room, the scorer needs almost none.
- The `ffmpeg` CLI is absent in the container (matplotlib animations are GIFs via Pillow), but
  `imageio` + `imageio_ffmpeg` are present, so `dishsim.media.VideoWriter` (H.264 MP4) works.
- Draining is looser than the organized policy's angle rule: this shallow bowl traps only
  about 1 mm at 100° from mouth-down, so `pools` is False there while `mouth_up` is True.
- Scores are only comparable across arrangements of the same objects; each object has its
  own self-occlusion ceiling (`baseline` in the record when `baselines=True`).
- A spoon whose bowl faces the ceiling scores 0 alone (no source faces its interior); the
  demo scene picks, per cutlery kind, the best-exposed head-down candidate of its compartment.
- A score stored in a manifest or an older record was computed under the defaults of its
  day; always re-score at the current parameters before comparing (settled_best does).

## Limitations to state with any result

Line-of-sight only (no splash, sheeting or pooling dynamics); direction-blind within a
source disc ("toward the centre" is invisible); arm discs, ceiling point and cosine weighting
are assumptions, not measurements; the ceiling nozzle is off by default and the ranking is
only claimed for weights up to 0.5; plate and cutlery rules are geometric and were never
validated against a settled state (cutlery cannot be settled reproducibly here); search
proposals are FCL-feasible, not settled, until run through step 2 above.

## HOTEC set: highest-exposure full load (2026-09-22)

The user asked for the arrangement of the whole HOTEC set (8 plates, 8 bowls, 8 cups; v2 massed
assets) that scores highest under this scorer, in the final twin (tape racks, v3-look upper floor,
95 mm v3-design basket), racking all 24 pieces. Script:
`frigidaire/scripts/evaluation/frigidaire_hotec_exposure_search.py` (Kit-free, CUDA).

**HOTEC kinds in the scorer.** The scorer keys kinds by name and builds food-contact surfaces from the
prototype lathe meshes, so the HOTEC pieces are registered as `hotec_plate` / `hotec_bowl` /
`hotec_cup` from their v2 USD visual meshes: a food-contact face looks inward (toward the lathe axis)
or up, and lies above the foot plane + 3 mm (interiors of bowl and cup, plate top; undersides, outer
walls and the foot recess excluded); the mouth ring is the interior's highest ring; pooling applies as
for the prototypes. Areas: plate 502 cm2, bowl 340 cm2, cup 215 cm2.

**Search.** Coordinate ascent from run v8 (the first complete load): every piece owns the realistic
candidate family of its kind (the HOTEC planner's slot families: plates in every front and rear gap x
leans x offsets, 1404 poses; bowls in the lower zones and every upper centre gap x tilts x lifts,
27966 poses; cups in the ten ladder slots x variants, 48 poses). One move re-poses one piece to
an FCL-free, non-nesting alternative given the other 23; alternatives are ranked by their cached
isolated exposure (appliance occluders only) and the best six plus two random ones are scored in full
(all 24 as occluders); a move is kept when S rises. Only complete loads exist by construction. Three
sweeps, seed 0, 26 moves, 34 min. The result is a sampled maximum, not a proof.

| load | S proposed | S settled (Isaac) | worst piece (proposed) |
|---|---:|---:|---:|
| run v8, first-free planner | 0.1595 | 0.1533 | 0.011 |
| run v9, exposure search | 0.2121 | 0.1971 | 0.020 |

What the search changed: the plates spread over both banks (front gaps 0, 3, 5, 7 and rear gaps 0, 3,
6, 8) instead of shingling in the front bank, which lifts every plate from 0.05 to 0.09 up to 0.15 to
0.28; that spacing leaves room for one bowl between the banks (`lower_mid`, x -165) and one over the
rear rows; every cup leans 8 deg toward the arm (0.13 to 0.27 instead of 0.06 to 0.24). The two low
pieces that remain are the bowl against the right wall ahead of the basket (0.02) and the rear bowl
under a plate shadow (0.04). Isaac settle of the best load (run v9): PASS, all 24 contained, max
placed-to-settled displacement 55 mm / 16 deg; re-scoring the settled poses gives 0.1971 (v8 settled 0.1533):
settling costs about 0.015 in S for both loads (plates relax onto the tines, cups slide down the slope).

Records: `results/exposure/frigidaire/hotec/` (`search.json` with the move history and per-piece
exposures and the v8 start score, `best_layout.json`, `settled_score.json`, `heatmap.png`), host
copies under `media/exposure_hotec/`; Isaac run `results/hotec/frigidaire/v9/`, `media/hotec_wheatstraw/v9/`;
figure `docs/figures/hotec_exposure_best.png` (settled still + heat map).

The HOTEC records above ran on the first tape build. On the height-fixed twin (2026-09-23,
`frigidaire/docs/geometry.md` "Outside heights") the v9 poses score 0.2119 proposed and 0.1979
settled (run v12); the full load v11 settles at 0.1564. Records: `results/exposure/frigidaire/hotec_heightfix/`
(`--sweeps 0 --insertion-gate` from the re-certified layout) and `docs/hotec_wheatstraw_asset.md`.

### Insertion-order gate (2026-09-22)

Collision-free rest poses are not enough for a human to reproduce a load: a piece placed early can
block the way a later one must travel. The gate in `frigidaire_hotec_exposure_search.py`
(`insertion_order`, `--insertion-gate`) models the hand as a straight vertical lowering from 30 cm
above the rest pose in the final orientation, with the loaded rack pulled out (the other rack is
ignored, the basket counts for the lower rack). Plates are the exception: lowered straight in their
final lean they would drive their underside onto the tine they lean over, which no hand does, so a
plate is lowered UPRIGHT and then rotated about its bottom edge to its lean (4 deg steps); rack contact
along a plate's path is not gated (the thin rim threads between the tines by hand), piece-to-piece
crossings are. A dependency "A before B" is recorded when A's path crosses B's rest pose; the load is
order-feasible when the dependency graph is acyclic and no bowl or cup path hits the rack. The order
is a topological sort (lower rack first, rear to front, left to right as the tie-break).

Result: run v8 and run v9 are both order-feasible (26 dependencies in v9, no cycle, no rack
hit), so the gated re-search was not needed and v9 stands as the best load. The v9 order, with the
human placement wording, is written into `results/exposure/frigidaire/hotec/best_layout_ordered.json`
(`insertion_order`, `insertion_dependencies`, `placement_instructions`) and, as a step list, into
`media/hotec_wheatstraw/v10/placement_instructions.md`. Run v10 replays that order in Isaac
(`frigidaire_hotec_load.py --sequential`: every piece starts on the counter, enters one at a time,
short settle after each step with the nudge of earlier pieces recorded, one still per step, then
the full settle and orbit).

Run v10 outcome (sequential insertion, Isaac): 23 of the 24 steps settle cleanly with no earlier piece
nudged; step 11, the bowl between the plate banks (`lower_mid`, x -165 over the short rows 3-4, seeded 60 mm
above the tines), does not: it drops onto the already-relaxed plates, tumbles about 97 deg to a mouth-up
pose and knocks the rear-left plate by 41 deg. The final settle still passes with all 24 pieces in their
racks, but the settled load scores 0.1859 with one pooling piece (v9 settled all at once:
0.1971, no pooling). So the geometric order gate is satisfied, and the sequence exposes a
physics-stability limit of that one pose: the mid-zone bowl is not reproducible by hand as specified.
Records: `results/hotec/frigidaire/v10/` (`hotec_v10_settle.json` has the per-step `sequential.steps`
with each step's settle verdict and the nudge of earlier pieces, `hotec_v10_step_NN.png` stills),
host copies `media/hotec_wheatstraw/v10/`; contact sheet `docs/figures/hotec_placement_steps.png`.
Next step if the load is to be reproduced by hand: exclude the mid zone from the bowl family and re-run
the search (the bowl returns to the rear rows as in run v8; S will drop a little), or seed that bowl on
the tines without the lift.
