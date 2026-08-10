# Bosch 800 digital twin — Stage A plan (portable)

> **STATUS 2026-08-10, end of day (added for machine-portability — this file survived the
> last instance change when nothing else did):** A0–A2 COMPLETE, A3 UNDERWAY. A2 verdict:
> UR5e at `side_winner` (x +0.475, y −0.525, z 0.400, yaw +101.25°) meets all four
> criteria. A3 landed: first full episode CLEARED (`bosch_ep2`: 560 mm pull + cup +
> tumbler placed 2/2) and the plate is NO LONGER PATH-BLOCKED (`bosch_plate_rematch`:
> planned in 16 s vs the v1 180 s timeout; failed only lateral settle 14.5 vs 12 mm).
> Assets/results archived: HF `shu4dev/dishsim-assets` tag `20260810_99e5cb9`.
> A3 remaining: plate settle tuning (release vs the 50 mm pitch), real-scale dishes
> (§A0 shortlist), Bosch capacity fill, legacy-mug anomaly root-cause (excluded via
> `config.reference_class`), middle-rack thinness (RackMatic-low lever), then Stage B.

Written 2026-08-10. Self-contained execution plan for re-anchoring `dishwasher_sim_isaaclab`
from the fictional ArtVIP compact machine to a **Bosch 800 Series 24″ built-in** digital
twin, plus a UR5e mount-feasibility study. Import this file to the executing instance; a
condensed copy also lives in the project's Claude memory (`bosch800-twin-plan.md`).

## Why

Sim-to-real: the current machine has no real counterpart (481×415×470 mm, two racks — a
configuration no product matches) and a 200 mm rack-travel artifact that shaped every
reachability result. A policy trained in a world with no real twin has nothing to transfer
to. The Bosch 800 is the canonical real target: 598 × 860 × 603 mm, 1321 mm open depth,
3 full-extension racks, 16 place settings.

Verified sources (2026-08-10):
- https://www.dimensions.com/element/bosch-800-series-recessed-handle-dishwasher-24 (exterior dims + 52″ open depth)
- https://www.pcrichard.com/bosch-800-series-24-in-top-control-smart-dishwasher-with-42-dba-sound-level-3rd-rack-crystaldry-and-pocket-stainless-steel/SHP78CM5N.html
- https://www.bestbuy.com/site/6559624.p?skuId=6559624 (SGX78C55UC)
- https://www.lowes.com/pd/1001042074 (SHXM78Z55N)

## Locked decisions

1. **Staged data acquisition**: Stage A builds from published data (installation/user
   manual PDFs from bosch-home.com, replacement-rack parts diagrams); every estimated
   value is TAGGED `estimated` in `docs/bosch800_source_data.md`. Stage B (when a physical
   unit exists) swaps estimates for caliper measurements.
2. **UR5e elevated/side mounts swept first**; UR10e only if the sweep proves it necessary.
3. **The v1 ArtVIP world stays intact and reproducible** behind a machine selector — the
   Bosch world is a switchable second machine, never a replacement.

## How the task changes (design intent)

Real racks extend FULLY on glides → placement happens in open air above the open door
(as humans load), replacing the artificial partial-travel mouth-insertion problem with the
real problems: 16-place-setting density at real ~25 mm tine pitch, real grasp moments
(700 g plates edge-pinched), long sweeping rack pulls (~500 mm), 3-rack state machine,
elevated/oblique approach directions. Real drinkware vs the 85 mm jaw: plates edge-pinch
at any diameter; drinkware must be curated to body ≤ ~78 mm or use the registry's
`handle_pinch` family (measured precedent: the 93 mm real YCB mug cannot be jaw-pinched).

## Architecture: machine selector

Copy the proven `apply_scenario` pattern (`config.py:119-146`):
- `config.MACHINES = {"artvip_compact": {...}, "bosch800": {...}}`, `MACHINE =
  "artvip_compact"` (default until v2 validates), `apply_machine(name)` mutating module
  globals pre-import. Per-machine: `MACHINE_GEN` (tub/door params), `RACK_GEN` overlay,
  joint names/travels, `DISHWASHER_POS_W`, countertop, cameras, base placements.
- Cache separation: non-default machine prepends `assets/cache/machines/<name>/`;
  v1 caches stay byte-stable.
- `geometry.config_hash()` gains `MACHINE_GEN` + machine name conditionally on non-default
  (mirroring the `object_spec` precedent at `geometry.py:59-64`) so v1 hashes are unchanged.

Key enabling facts (verified in-repo, session of 2026-08-10):
- The repo already injects procedural geometry into the articulated USD (countertop
  pattern, `usd_prep.py:123-171`); a fully self-authored machine USD is ~200-300 lines
  using APIs already exercised in-repo.
- Reusing the six canonical body/joint names (`E_body_5`, `E_door_4`, `E_shelf_1_04`,
  `E_shelf_03`, `RevoluteJoint_dishwasher_2_middle`, `PrismaticJoint_dishwasher_2_up/_down`)
  makes ~80 downstream references work unchanged. The third rack gets NEW names
  (`E_shelf_third` + joint) gated on the selector.
- A parametric tub built from explicit convex slabs takes the analytic decomposition path
  (`decompose_meshes.py:74-104`), retiring the fragile `config.COACD["E_body_5"]` override.
- Rack travel is just an authored prismatic limit — full extension (~0.50 m) is modelable;
  the travel-dependent literals are centralized (`RACK_LOWER/UPPER_EXT_M`,
  `RACK_TRAVEL_LIMITS_M`, `RACK_SLIDE_STEPS`) with two hardcoded stragglers
  (`capacity_fill.py` ramp, several tests).

## Stage A0 — Source data + dish shortlist (~3 h, research only)

1. Fetch Bosch 800 (SHP78CM5N-class) installation + user manual PDFs; extract every
   dimensioned drawing (door projection, leveling-leg range, interior clearances).
   Replacement-rack parts listings for overall rack dims. Record all URLs.
2. Write `docs/bosch800_source_data.md`: every number with source + provenance tag
   (`published` / `derived-from-drawing` / `estimated`). The `estimated` rows are the
   Stage-B caliper checklist.
3. Real-dish shortlist with published dims: plates (edge-pinch), cutlery at 1.0 scale,
   drinkware with body ≤ ~78 mm; flag classes needing `handle_pinch` or exclusion.

## Stage A1 — Parametric machine (~8 h, authoring)

1. `config.MACHINE_GEN["bosch800"]`: tub shell as explicit convex slabs (walls, roof, sump
   box, door-opening frame), door panel + dispenser bump + hinge anchor, **spray-arm
   cylinders** (bound under-rack clearance; absent from v1), 3 rack prismatic joints with
   full travel, RackMatic middle height as a parameter (one position in Stage A). Every
   value carries its A0 provenance.
2. `usd_prep.make_bosch800_usd()`: fully self-authored USD (no ArtVIP sublayers),
   following `_author_rack_meshes`/`_author_countertop` patterns; canonical names reused;
   third rack added; door locked open at 90°; `MACHINE_GEN`-hash stamp (extends
   `usd_prep.py:49-54`).
3. `rack_gen`: Bosch-dim overlays (lower ~535 × 520); third-rack shallow-tray builder +
   `flat_lay_third` placement mode in `placement.py` (per `docs/extending.md`).
4. Mounts: implement the `BASE_PLACEMENTS` mechanics from the original reachability plan
   (D5) — named placements with `base_pos_w`, `base_quat_w` (yaw), pedestal params incl.
   HEIGHT; conditional hash key incl. the quaternion (closes the known gap);
   full-transform `world_to_base`/`base_to_world` helpers converting the xy-subtraction
   sites (`scene.py:90`, `task/layout.py:205`, `run_task.py:781,881`,
   `capacity_fill.py:76,163`, `reach_map.py:143-159`, `goal_configs.py:104`).
   Real 900 mm counter beside the machine per placement.
5. **Kit-free calibration pre-gate** (small, pays for itself immediately): FCL probe of a
   candidate carry pose against the open-aperture gripper cluster — answers in seconds
   what previously took 25-min Kit calibration runs per guess.
6. Tests: `apply_machine` round-trip restores byte-stable v1 hashes; `MACHINE_GEN` hash
   coverage; third-rack geometry; selector-aware `resolve_rack_state`.

## Stage A2 — Bake + UR5e mount sweep (~6-8 h incl. runs)

1. Bring-up with the existing 14-class library: `build_state --machine bosch800` per state
   (extract → decompose → parity ≥95% → goal_configs).
2. Extend `base_pose_sweep`: pedestal-height as a sweep dimension + side-mount region
   (y beside the open door, z up to counter height, yaw toward the machine). Gates
   unchanged; rack pull/push now full-travel.
3. Success bar per rack: lower-extended destinations > 0, middle-extended > 0, third-rack
   flat-lay > 0, pick band on the 900 mm counter > 0.12 m.
4. **Deliverable: the measured verdict** — best UR5e mount + per-criterion counts, or the
   quantified UR10e case. Scorecards + heatmaps → `results/base_sweep/bosch800/`.

## Stage A3 — Real dishes (~4-6 h, after the A2 verdict)

Restore the retired authoring pipeline from git history (`build_object_assets.py` —
documented in `docs/extending.md`), add the A0 shortlist at real scale, calibrate grasps
on the winning mount (pre-gate first), bake funnels, run the first Bosch episodes +
capacity fill. Every grasp calibrated, never eyeballed.

## Stage B (future, physical unit)

Caliper protocol from the `estimated` rows; swap estimates → measurements; re-bake; then
the sim-to-real ladder continues: weld removal (friction-only carry), pose-noise
robustness spec, physics system-ID + domain randomization, learned policies.

## Speed levers

1. **vCPUs (biggest)**: the sweep is Kit-free, embarrassingly parallel (`--procs N`).
   ~1-2 h at 64 procs vs 6-10 h at 8.
2. **Second GPU/instance**: Kit runs serialize on one L4; two instances ≈ 2× on
   bake/validation days (sync `assets/cache` via the archive tarballs).
3. **The A1 calibration pre-gate** (above).
4. **Overnight chaining** of pre-authorized runs.

## Time estimate (64 vCPU / 256 GB / 1× L4)

A0 ≈ 3 h · A1 ≈ 8 h · A2 ≈ 6-8 h · A3 ≈ 4-6 h → **≈ 21-29 h ≈ 2-3 working days** with
overnight chaining; +30-50% contingency for discovery surprises (every migration so far
found one). 256 GB RAM is ample (64 sweep workers ≈ 25-35 GB). The single L4 keeps Kit
runs serial.

## Verification

- v1 intact: full pytest green under the default machine; all existing cache manifests
  hash-match; `run_trials.py` zero-diff.
- Bosch world: parity ≥95% per state; sweep artifacts on disk; every Kit run judged from
  log content (exit codes lie); PNG/MP4 evidence under `media/` per Isaac-side stage.
- Provenance: every `MACHINE_GEN` number traces to `docs/bosch800_source_data.md`;
  grep gate: no untagged estimates.

## Checkpoints

Ask-user before: the first Bosch bake, the sweep launch, at the A2 verdict (arm decision),
before A3 calibrations. Git stays with the user — each stage ends with a summary +
suggested commit message.
