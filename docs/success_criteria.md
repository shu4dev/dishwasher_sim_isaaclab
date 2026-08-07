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

## Multi-object episodes (`scripts/experiment/run_task.py`)

An **episode** spawns N objects on the countertop and clears them one at a time. Each pick is
evaluated by exactly the criteria above and recorded in the existing per-trial schema, so
Phase 3 reads an episode as a set of trials with no changes. The episode adds one outcome on
top, recorded in `results/experiments/<run>/episodes/<ep>.json`:

| status | meaning |
|---|---|
| `cleared` | every object was picked and placed within tolerance |
| `partial` | at least one pick failed; the rest still ran (a bad grasp costs one object, not the episode) |
| `deadlock` | objects remain but none is pickable — nothing rests on them *and* no collision-free grasp exists, after the bounded recovery ladder is exhausted |
| `error` | the episode aborted before the loop finished |

### Starting from a stowed machine (`both_in`)

A scenario carrying a `rack_action` opens the machine before the first pick. `both_in` has **0 of
15 reachable slots** — you cannot place into a stowed rack — so the robot pulls the lower rack out
(`PrismaticJoint_dishwasher_2_down` → −0.20 m), reaching `placement`, and loads from there.

The episode therefore spans two machine states and two sets of collision caches. The rack phase
plans in the START state but carries nothing, so it needs only the scenario-level machine cache;
every pick plans in the POST state and needs the per-class caches. `POST_STATE` is *derived* from
the action rather than hardcoded, so a new scenario cannot silently load the wrong caches.

**The settle gate is not optional.** Every pick plans against a world posed at the post-action
extension. A rack that stopped short makes that world a fiction, so an action settling beyond
`RACK_SLIDE_TOL_M` ends the episode as `error` with no picks attempted.

The rack drive override **stays latched** after a successful action and accumulates across a
sequence. The scene's standing targets come from the pre-action scenario, so releasing the
override would drive the rack straight back in, and a second action that replaced the dict rather
than merging it would let the first rack slide shut behind the arm.

#### Measured: pulling the upper rack out too does not help

| class | `placement` (lower out, upper stowed) | `placement_open` (both out) |
|---|---|---|
| cup | 4 of 15 | **0 of 15** |
| tumbler | 4 of 15 | **0 of 15** |
| fork | 2 of 3 bays | **0 of 3** |

The lower rack is already at its mechanical limit at −0.20 m (both prismatic joints are limited to
`[-0.20, 0.00]`), and the rack is 0.287 m deep, so ~87 mm never leaves the mouth whatever you do.
Extending the upper rack does not free that space — it puts the upper rail over the *front* of the
lower rack and removes the reachability that did exist. `placement` is the best available state.

### Acquisition is not always a pick

Grasp families in `config.WELD_ACQUIRE_FAMILIES` (`edge_pinch`, `rim_edge`, `handle_pinch`) have no
calibrated countertop pick — the pads contact one-sided and drag a free-lying object during the
close. In an episode these are **acquired**: the arm flies to the pre-grasp hover as it would for a
real pick, then the object is snapped to the calibrated carry transform and the weld takes it. The
descent is deliberately not attempted, since that is precisely the uncalibrated part.

Every record labels this `acquired: "weld"`. **Never aggregate weld acquisitions and real picks into
one success rate.** Two checks are relaxed to match what actually runs — the layout's reachability
test and the grasp funnel's descent gate both stop at the hover for these classes. That is not a
loosening: gating on a descent that never executes rejected 344 of 400 fork layout draws.

A pool containing a thin-insertion mode (`basket_drop`, `plate_slot`) also forces a **per-piece**
payload cluster. The merged gripper+payload hull is a single convex wedge that cannot enter a
cutlery bay or a 30 mm tine gap.

### The home-anchored trajectory

Every episode's recorded trajectory **begins and ends at `config.HOME_Q`**, so any two episodes
are directly comparable and any episode can be replayed or chained from a known state.

- **Start.** The scene spawns the arm at `HOME_Q` and `hold_targets` re-arms the drives there,
  but the layout spawn→settle→resample loop steps physics an unbounded number of times first.
  The runner measures the drift, returns home if it exceeds `TASK["home_tol_rad"]`, and then
  asserts — the invariant is enforced, not assumed.
- **End.** After the sequencer returns, the arm parks at home and holds for `SETTLE_STEPS`. This
  runs *before* `rec.end` and the media finish, so the retreat is inside the `.npz` and the MP4
  and the final stills show the arm parked. It runs *after* the sequencer, whose placement
  verdicts are already decided — **parking the arm can never revise a pass/fail.**
- It also runs on the failure and exception paths: an episode that crashes still parks the arm
  and still writes its trajectory and record, because the failed episode is the interesting one.

| field | meaning |
|---|---|
| `start_home_err_rad` | max per-joint deviation from `HOME_Q` at the first pick, after any correction |
| `end_home_err_rad` | the same after the closing retreat — the evidence the trajectory is anchored |
| `home_return_status` | `skipped` / `planned` / `lerp` / `failed` |
| `post_home_displacement_mm` | how far each *placed* object moved during the retreat |

`home_return_status` is worth reading: the `lerp` fallback is a straight joint-space
interpolation that is **not** collision-checked (it exists because the arm may be wedged
somewhere the planner cannot escape). Pair it with `post_home_displacement_mm` before trusting
the final scene — that pair is why a retreat that nudges a placed cup is visible rather than
silent.

An object is **pickable** iff both gates pass, re-evaluated from measured world state before
every pick (never one pick ahead — removing an object lets its neighbours shift):

1. **support** — nothing rests on it (Stage B support graph);
2. **grasp** — a collision-free grasp exists *in the current world*, with every other object
   registered as an obstacle (Stage C; state-dependent, never precomputed).

### The support graph (Stage B)

`supporter -> {objects resting directly ON it}`; an object is pickable when its entry is empty.
Edges are direct only, so "can I pick this" stays a lookup rather than a search, and the graph is
rebuilt from measured poses before every pick. Two backends cross-check each other
(`TASK["support_backend"]`):

| backend | basis | blind spot |
|---|---|---|
| `geometric` | footprint-circle overlap + vertical band from the supporter's own bottom to its top plus `support_gap_m` | needs the band to be wide enough for nesting |
| `contact` | PhysX object-object forces, oriented by height (contact is symmetric, support is not) | only sees *touching*; two objects leaning together touch without supporting |
| `both` | geometric is authoritative, disagreements are logged | — |

Orienting every edge by height (higher rests on lower) makes a cycle impossible by construction.

The vertical band spans from the supporter's bottom rather than hugging its top precisely because
**cups nest**: a nested cup's base sits well below the rim of the one beneath it. A stricter
"bottom level with top" rule called a real, settled, physically supported stack unsupported — the
two backends disagreed on exactly that pair in a live run, which is what the `both` cross-check
exists to surface.

### Grasp availability and recovery (Stage C)

A grasp is searched over a yaw sweep about the object's own axis (nominal first, widening
symmetrically), checking the pre-grasp hover **and the descent beneath it**. Checking only the
hover — which is safe for one object alone on a counter — produced 56–60 N forearm contacts
mid-descent once neighbours existed. The target object is lifted out of the world for the check,
since an object must not veto its own grasp, and the last stretch of the descent is excluded
because the pads are inside the object's inflated hull there by design.

Every rejection is counted, so "unpickable" always says why: out of reach (no IK at any yaw),
blocked at the hover, or blocked on the descent.

When nothing is pickable and objects remain, a bounded ladder runs before failing:

1. **widen the grasp search** — denser yaw sweep; no physics, milliseconds, fires once;
2. **re-settle** — run physics briefly and rebuild both the support graph and grasp availability
   from what actually settled;
3. **deadlock** — end the episode naming every remaining object and its reason.

Capped by `TASK["max_recovery_attempts"]`. A non-prehensile push is deliberately *not* a rung:
the repository's safety model (calibrated pad-force bands, the wrist weld) is built around pinch
grasps, and an open-jaw push needs its own thresholds. Strategies are a registry so it can be
added behind the same interface when justified.

Before executing a pick, the object's measured pose is compared against the pose its grasp was
computed from; beyond `pose_stale_tol_m` / `_deg` the grasp is recomputed and the pick replanned,
counted per episode as `n_replans`.

### Configuring an episode

| Knob | Config | CLI |
|---|---|---|
| Composition, per type | `TASK["spawn_counts"]` — `{"cup": (2,4), "mug": 1}` | `--spawn "cup=2-4,mug=1"` |
| Class pool + total (fallback) | `TASK["classes"]`, `["n_objects"]` | `--classes`, `--n_objects` |
| Rack extensions | `TASK["rack_state"]` — a name or `{"lower_m", "upper_m"}` | `--scenario`, `--rack_lower_m`/`--rack_upper_m` |
| Goal slots, per type | `TASK["type_slots"]` — ordered slot NAMES | — |
| Goal slots, per mode | `TASK["slot_pools"]` — slot ids | — |
| Episode video camera | `TASK["video_camera"]`, `EPISODE_CAMERA` | `--video_camera` |
| Camera lens | `CAMERA_LENS`, `CAMERA_LENS_DEFAULT` | — |
| Pick-order heuristic | `TASK["cost_fn"]` | `--cost_fn` |
| Slot clearance | `TASK["slot_separation_margin_m"]` — 10 mm; **this is what caps the load at 2** | — |
| "Already home" tolerance | `TASK["home_tol_rad"]` | — |
| Which classes are weld-acquired | `WELD_ACQUIRE_FAMILIES` — by grasp FAMILY, not a class list | — |

A count range is drawn per episode from the layout seed, so the number of objects varies while
staying reproducible. Term order is irrelevant — the spec is expanded in sorted key order, so
`a=1,b=2` and `b=2,a=1` give the same episode — and the expanded list is then shuffled, so no
type permanently gets first claim on countertop space.

Rack extensions are part of `geometry.config_hash()` and caches are keyed by state *name*, so a
combination that has not been baked cannot be planned against. Both racks are independently
articulated, limits `[-0.20, 0.00] m` each (0 = stowed). An unbaked combination, or a state whose
caches are missing for a requested class, fails at startup naming
`scripts/setup/build_state.py --state <name> --classes <list>` — including the mug, whose cache
the machine world is always loaded from regardless of the class pool.

Slot names are **derived from the rack geometry**, not hardcoded: slot ids are positional
(`slot_id = len(slots)` in derivation order) and would silently re-point to different physical
cells if `SLOT_GRID_PITCH_M` or `SLOT_RIM_INSET_M` were retuned. Each mode gets the vocabulary
its grid actually has:

| mode | grid | names |
|---|---|---|
| `floor_stand` | 3 depth × 5 lateral | `near_centre`, `mid_left1`, `far_right2` |
| `plate_slot` | 11 tine gaps (lateral) | `gap_left5` … `gap_centre` … `gap_right5` |
| `bowl_lean` | 3 lean positions (depth) | `near`, `mid`, `far` |
| `basket_drop` | 3 bays (depth) | `bay_near`, `bay_mid`, `bay_far` |

Names are ordinal within the RACK, so they are invariant to how far it is pulled out; which
slots are *feasible* is not, and is measured per state. The four feasible `floor_stand` slots are
`near_left1`, `near_centre`, `mid_left1`, `mid_centre`.

### Measured capacity ceilings

These bound what an episode can be asked to do, and are measured rather than assumed:

| ceiling | value | why |
|---|---|---|
| pickable classes | 3 of 15 — `mug`, `cup`, `tumbler` | only the `rim_diam` grasp family has a calibrated countertop pick; `plate`/`bowl`/`fork` are `START_WELDED` and skip the pick entirely |
| usable lower-rack slots | 4 of 15 — ids 1, 2, 6, 7 | the other 11 funnel to zero goal configs on collision |
| simultaneous placements | **2** | those 4 form a 2×2 block at the 60 mm `SLOT_GRID_PITCH_M`; the grid deliberately *overlaps* (one object per trial), so only the ~85 mm diagonals are jointly occupiable |

The destination, not the countertop, is the bottleneck — the extended worktop holds far more
than the machine can accept from the robot.

### Known: `floor_stand` tilt is not met by every class

The `floor_stand` criteria (±1.5 cm lateral, **10° tilt**) were validated on the **mug**, which
meets them reliably. The `cup` does not: released from the 12 mm `RELEASE_HOVER_M` onto the wire
floor it settles 12–24° tilted, so it fails `unstable-after-release` on lateral-perfect
placements.

This is **pre-existing and class-specific, not a multi-object effect** — measured with the
unchanged single-object runner:

```
scripts/run_kit.sh scripts/experiment/run_trials.py --headless \
    --object cup --scenario placement --slots 1,7 --seeds 0 --repeats 1
  trial_01_00_0  unstable-after-release  lateral 0.0100 m  tilt 23.95°
  trial_07_00_0  unstable-after-release  lateral 0.0042 m  tilt 12.77°
```

The tumbler behaves similarly (~12°). Both are *stable* — they settle and stay — they are simply
outside a tolerance derived from a different object. Closing this needs per-class placement
tuning (a smaller release hover for tall/narrow items, or a per-class `tol_tilt_deg` in
`config.PLACEMENT_MODES`), which is calibration work rather than a task-layer change. Until then
the mug is the only class that reliably reports `cleared`.
