# v0 placement success criteria

Defined for the v0 task: OMPL places the carried object (YCB mug — the plate stand-in, since
`029_plate` is absent from the Isaac 6.0 asset bucket) at a slot in the dishwasher's lower
rack. Frames: robot-base frame, meters, Z-up, XYZW quaternions.

## Slot model

The ArtVIP `dishwasher_2` lower rack (`E_shelf_1_04`) is a shallow **wire basket** (~5 cm deep
wire grid), not a tined plate rack — measured from the Phase D decomposition
(`media/D/overlay_E_shelf_1_04.png`). The object therefore *stands on the wire floor*; a
**slot** is a standing cell on a grid derived from the basket geometry
(`dishsim.placement.derive_slots_from_rack`): footprint-sized cells, inset
`SLOT_RIM_INSET_M` from the rim walls, pitch = footprint + `SLOT_MIN_PITCH_M`. Slot frames sit
**on** the wire floor, z up. Derivation is recorded per slot (`source: derived`); if it ever
fails on a different asset, slots are hand-placed from the Phase C measurements
(`config.SLOTS_OVERRIDE`, `source: manual`).

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
