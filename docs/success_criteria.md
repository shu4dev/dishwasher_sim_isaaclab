# Placement success criteria (RACK_GEN v4)

Defined for placing an object at a slot in the dishwasher's racks — teleport placement,
judged on the physics-settled pose. Frames: robot-base frame, meters, Z-up, XYZW quaternions.

## Slot model (per placement mode)

Since rack_gen v2/v3 the racks are procedurally generated (tine bank, bowl tines, cutlery
basket); each object class declares a placement mode in `config.OBJECTS` and
`dishsim.placement.derive_slots` dispatches:

| mode | classes | slot definition | evaluated as |
|---|---|---|---|
| `floor_stand` | mug, cup, tumbler, bowl (v4 — bowls stand) | v0 standing cell on the wire-floor grid (`derive_slots_from_rack`: footprint cells, `SLOT_RIM_INSET_M` inset, `SLOT_GRID_PITCH_M` pitch) | lateral ≤ `SLOT_TOL_LATERAL_M`, axis-vs-z tilt ≤ `SLOT_TOL_TILT_DEG`, bottom within 2 cm of the floor |
| `plate_slot` | plate, saucer, lid | one slot per front-bank gap (v4: 3 gaps at 40 mm pitch, no tie wire; the 11-gap rear bank is fill-only); disc face normal along the pitch direction, leaned +7° like the tines, bottom edge seated on the bank floor bar | face-normal deviation ≤ 12° (flip-insensitive), off-gap drift ≤ 12 mm, center height within 2 cm of nominal |
| `bowl_lean` | — (removed with RACK_GEN v4: the lean fixture is gone — bowls stand via `floor_stand`; see [known_limitations.md](known_limitations.md)) | — | — |
| `basket_drop` | fork, spoon, knife, serving_spoon | one slot per basket bay; the goal is a release hover 60 mm above the bay, head-down — gravity inserts | settled bbox center inside the bay volume, below the basket top |

Every mode is judged on the settled pose: the criteria must hold continuously for the last
`SETTLE_STEPS` = **300 physics steps** (5 s at dt = 1/60) after release.

Fill-only modes (`upside_down`, `stem_scallop`, `flat_lay`) are exercised by the capacity
fill (`scripts/setup/capacity_fill.py`), which gates per-item settle stability
(< 5 mm / 3° drift) and load-wide closability instead of slot-frame tolerances.

### Slot names

Slot names are **derived from the rack geometry**, not hardcoded: slot ids are positional
(`slot_id = len(slots)` in derivation order) and would silently re-point to different physical
cells if `SLOT_GRID_PITCH_M` or `SLOT_RIM_INSET_M` were retuned. Each mode gets the vocabulary
its grid actually has:

| mode | grid | names |
|---|---|---|
| `floor_stand` | 3 depth × 5 lateral | `near_centre`, `mid_left1`, `far_right2` |
| `plate_slot` | 3 front-bank gaps (lateral) | `gap_left1`, `gap_centre`, `gap_right1` |
| `basket_drop` | 3 bays (lateral — the v4 basket splits along x) | `gap_left1`, `gap_centre`, `gap_right1` |

Names are ordinal within the RACK, so they are invariant to how far it is pulled out; which
slots are *placeable* is not, and is measured per state (`scripts/setup/derive_slots.py`).

### Known: `floor_stand` tilt is not met by every class

The `floor_stand` criteria (±1.5 cm lateral, **10° tilt**) were validated on the **mug**, which
meets them reliably. The `cup` does not: released from the 12 mm `RELEASE_HOVER_M` onto the
wire floor it settles 12–24° tilted (measured on lateral-perfect placements: 23.95° and
12.77°), and the tumbler behaves similarly (~12°). Both are *stable* — they settle and stay —
they are simply outside a tolerance derived from a different object. Closing this needs
per-class placement tuning (a smaller release hover for tall/narrow items, or a per-class
`tol_tilt_deg` in `config.PLACEMENT_MODES`).

## Placeable capacity — what "full load = N" means (Bosch 800, v2)

The machine's geometric capacity (every slot the racks provide) and its certified capacity
are different numbers; only the second defines a full load. The planner
(`src/dishsim/capacity.py`, CLI `scripts/setup/plan_full_load.py`) counts an item only if ALL
of:

1. **Placeable** — the class's convex pieces are collision-free at the slot's nominal release
   pose in the EMPTY machine (`CollisionWorld.object_in_collision`; slots derive live via
   `placement.derive_slots`), the same predicate slot assignment uses;
2. **Jointly certified** — at plan time its release pose stays collision-free with every
   earlier item of the load resting at its own goal pose in the state's FCL world (without
   this, "N placeable slots" overcounts — adjacent rest poses overlap);
3. **Transition-safe** — its worst-case tolerance pose (full tilt + bottom allowance) clears
   the geometry that passes overhead during the scripted rack transitions by
   ≥ 15 mm (`capacity.Z_BUDGET_MARGIN_M`);
4. **Packable** — it clears every already-assigned slot under the shared occupancy rule
   (`src/dishsim/task/slotting.py`).

Historical context: the robot-era arm-reachable full load measured 22 items (2026-08-14,
Bosch @ side_winner, policy `plates_first`: `third_out` 14 forks + `placement` 8 — plate 2,
bowl 6); teleport placeability re-counts this without the reach constraint. For contrast the
teleport-based `capacity_fill.py` seats 29 on the v1 machine. Phases execute TOP-DOWN
(`capacity.LOADING_STATES`) so no transition ever stows the loaded lower rack
(known_limitations: the sill climb stalls under load); a planned load renders settled via
`scripts/evaluation/reveal_render.py --plan <plan JSON>`.

Two measured gates shrink raw placeability into the certified load:

- *settle reliability* (< 90 % excluded): upright cups and tumblers wedge into the coarse
  wire lattice about half the time on both racks (cup 53-76 %, tumbler 73 %; bowls 98 %) —
  see known_limitations "Upright drinkware". This empties the middle rack, whose only
  capacity-relevant load was drinkware: slots there are plentiful but not dependably
  placeable.
- *z-budget* (< 15 mm worst-case clearance excluded): a middle-rack tumbler would be struck
  by the third rack sliding overhead (−15.8 mm).

Measured rules the numbers depend on:

- **Plate occupancy is per-gap, not per-disc-radius.** The old circle rule read a plate as a
  70.6 mm sphere: with the 10 mm margin two plates needed 151.2 mm of separation, and even
  gaps 0 and 3 of the Bosch bank sit 150 mm apart — ONE plate per bank. Distinct tine gaps
  are now compatible by design (the bank's pitch is the separation; joint certification still
  vets each pair).
- **`middle_out` floor slots target the EXTENDED middle rack** (`E_shelf_03`). The original
  derivation hardcoded the lower rack, so the A2-era "middle 2" destinations actually
  measured placing into the STOWED lower rack through the tub mouth (1 cell per class).
  Re-derived and re-baked 2026-08-14 (`placement._floor_stand_rack_body`): the "middle-rack
  thinness" flagged at A2 was this derivation artifact, not a real limit (robot-era funnels
  after the fix: cup 41 / tumbler 39 of 56 slots on the extended rack). v1 derivations are
  byte-identical (the lower rack is the extended rack in every v1 state).
- **The middle rack takes cups only.** Worst-case z-budget against the third rack's underside
  (rail 0.635): cup +20.6 mm, tumbler −15.8 mm, mug +18.0 mm (excluded on Bosch anyway) —
  a "successfully placed" tumbler could be struck by the third rack sliding out overhead.

### Plate settle tolerances are per-machine (measured 2026-08-14)

`PLACEMENT_MODES` is machine-overridable since v2, and the Bosch overrides the `plate_slot`
settle tolerances: `tol_lateral_m` 0.012 → **0.018**, `tol_tilt_deg` 12 → **16**. The v1
values fit the ArtVIP bank's 40 mm pitch (free lateral half-play ≈ 11.7 mm); on the Bosch's
50 mm pitch a released disc rolls until it rests against a tine — how real plates sit — at
(50 − 3.2 − 14.4)/2 = 16.2 mm of play plus the matching extra lean. Probe of record
(`scripts/setup/probe_plate_settle.py`, 3 gaps × 8 releases — the same release-and-settle
physics a placement runs): lateral mean 14.1 / p95 15.0 / max 15.2 mm, tilt p95 14.0 / max
14.1°, bottoms clean — 0/24 passed the v1 tolerances, 24/24 pass the derived ones. The
criterion is unchanged in spirit: seated in its own gap, braced on its tines, not fallen
flat. No baked artifact depends on these values (they are runtime-evaluated and unhashed).
