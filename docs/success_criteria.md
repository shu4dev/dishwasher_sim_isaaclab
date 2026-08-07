# Placement success criteria (v0 mug + multi-object v1)

Defined for OMPL placing the carried object at a slot in the dishwasher's lower rack.
Frames: robot-base frame, meters, Z-up, XYZW quaternions.

## Slot model (per placement mode)

Since rack_gen v2/v3 the racks are procedurally generated (tine bank, bowl tines, cutlery
basket); each object class declares a placement mode in `config.OBJECTS` and
`dishsim.placement.derive_slots` dispatches:

| mode | classes | slot definition | evaluated as |
|---|---|---|---|
| `floor_stand` | mug, cup, tumbler, pitcher | v0 standing cell on the wire-floor grid (`derive_slots_from_rack`: footprint cells, `SLOT_RIM_INSET_M` inset, `SLOT_GRID_PITCH_M` pitch) | lateral ≤ `SLOT_TOL_LATERAL_M`, axis-vs-z tilt ≤ `SLOT_TOL_TILT_DEG`, bottom within 2 cm of the floor |
| `plate_slot` | plate, saucer, lid | one slot per tine-bank gap (11 gaps at 30 mm pitch); disc face normal along the pitch direction, leaned +7° like the tines, bottom edge seated on the bank floor bar | face-normal deviation ≤ 12° (flip-insensitive), off-gap drift ≤ 12 mm, center height within 2 cm of nominal |
| `bowl_lean` | bowl | lean positions against the bowl tines over the drinkware slope (opening down-slope at ~48°) | axis within 20° of the lean cone, lateral ≤ 4 cm, height within 6 cm |
| `basket_drop` | fork, spoon, knife, serving_spoon | one slot per basket bay; the goal is a release hover 60 mm above the bay, head-down — gravity inserts | settled bbox center inside the bay volume, below the basket top |

Fill-only modes (`upside_down`, `stem_scallop`, `flat_lay`) are exercised by the capacity
fill (`scripts/setup/capacity_fill.py`), which gates per-item settle stability
(< 5 mm / 3° drift) and load-wide closability instead of slot-frame tolerances.

## A trial is a SUCCESS iff all of:

1. **Pose within slot tolerance** (evaluated on physics-backed object pose, after settling):
   - lateral: object axis footprint center within `SLOT_TOL_LATERAL_M` (**±1.5 cm**) of the
     slot center (slot-frame x/y);
   - tilt: object axis within `SLOT_TOL_TILT_DEG` (**10°**) of the slot z-axis;
   - the object rests ON the wire floor (bottom within 2 cm of the floor plane — it cannot
     rest anywhere lower; guards against "success" on top of the rim wall).
2. **Contact discipline**, split by phase:
   - **while carried** (close → plan → execute): the two inner-finger pads must hold the mug
     within the calibrated force band (`GRIP_FORCE_MIN_N`–`GRIP_FORCE_MAX_N` static,
     ≤ `GRIP_FORCE_EXEC_MAX_N` dynamic — `docs/grasp_calibration.md`); every other partner on
     the mug, the mug's non-gripper external residual, and every robot body's *unexpected*
     contact (pads checked net-of-mug-reaction) stay under `CONTACT_FORCE_THRESH_N` (0.1 N);
   - **after release** (jaws opened, weld dropped, tool retracted): no force above
     `CONTACT_FORCE_THRESH_N` on any robot body, and the object contacts only the rack
     (`E_shelf_1_04`) — not the door, machine body, or ground.
3. **Stability**: criteria 1–2 hold continuously for the last `SETTLE_STEPS` = **300 physics
   steps** (5 s at dt = 1/60) after release.

## Failure taxonomy (recorded per trial in `results/*.json`)

| stage | meaning |
|---|---|
| `no-goal-config` | goal generation produced zero collision-free IK configs for the slot (funnel logged: pose samples → IK branches → limit filter → collision filter) |
| `grasp-fault` | the visible close did not land in the calibrated pad-force band (or something else touched the mug) |
| `planner-timeout` | RRT-Connect found no path within `PLAN_TIME_BUDGET_S` (5 s) |
| `execution-collision` | an unexpected contact, mug external force, or pad-force spike fired while tracking the path |
| `release-fault` | the jaws opened at the goal but pad forces did not vanish |
| `retract-collision` | contact ≥ `RETRACT_GRAZE_MAX_N` (2 N) while retracting the opened tool (lighter brushes are recorded as `retract: grazed …` and judged by the final window — the placed mug settles a few mm off-center while the jaws have ~2.6 mm/side clearance) |
| `unstable-after-release` | released, but criteria 1–3 not met at the end of settling |

Goal-config scarcity per slot is **signal, not error** — rear slots at the reach envelope are
expected to yield thin or empty goal sets; the per-slot funnel counts in
`assets/cache/slots/goal_sets.json` quantify exactly where feasibility dies.
