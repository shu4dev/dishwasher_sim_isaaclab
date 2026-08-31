# Known limitations and open items

Honest negative results and open engineering items, with the measured evidence for each.
Numbers below trace to recorded runs (see the Results section in the README) or to
`docs/success_criteria.md`.

## `config_hash` is not the only cache key (2026-08-28, reproduced)

`geometry.config_hash()` keys the *manifests*; `geometry.coacd_dir_for` independently digests
mesh bytes **plus the body's `config.COACD` params**, and `COACD` is absent from the
`config_hash` payload. Changing a static's decomposition therefore leaves every manifest
validating green while pointing at a piece directory that does not exist yet. The failure is
loud at load (`missing CoACD pieces for '<body>' — run scripts/setup/decompose_meshes.py
first`) but the staleness check cannot see it coming, and a restore-only box hits it because
the shipped archive predates the change.

Practical consequences, until the archive is re-cut (archive_assets.py, git history):
after restoring assets, run `scripts/run_py.sh scripts/setup/decompose_meshes.py` once per context (Kit-free,
seconds). The upside of the same asymmetry is that re-decomposing a static invalidates
**nothing** — it is the cheapest honest fix available in this codebase.

## CoACD's manifold preprocess inflates authored bodies (2026-08-28, measured)

The preprocess re-meshes onto a voxel grid and adds an **isotropic skin proportional to body
size**: `E_door_4` +4.09 mm, `E_body_5` +8.24 mm. This is phantom collision volume, not
geometry, and it has cost real capacity twice: it FCL-blocked every counter-buffer cell (worked
around by `rearrange.BUFFER_EXTRA_HOVER_M`) and then 5 of the 8 lower-rack plate gaps, whose
true clearance is 7.3-7.8 mm. Watertight authored bodies do not need the preprocess:
`COACD["E_door_4"]["preprocess_mode"] = "off"` decomposes the door exactly (overhang
4.087 → **0.000 mm**, volume 1.179x → **1.000x**, 0.3 s) and took plate placeability 1/8 → 7/8.

Raising `preprocess_resolution` instead is **strictly worse and already tried**: 200 still left
1.02 mm of skin, exploded 2 pieces into 45, and took 474 s. Do not retry it.

**`E_body_5` still carries its +8.24 mm skin.** Unpreprocessed it did not converge in 17 minutes
(its solid is 28 % of its convex hull), and its skin overhangs only the countertop, which no
rack slot touches and which already carries the `BUFFER_EXTRA_HOVER_M` allowance. Open item;
not a capacity blocker. Diagnosis rule of thumb: when a slot or pose is FCL-blocked by a
*decomposed authored body*, measure hull overhang against the source mesh before believing the
geometry — an axis-aligned bounds comparison is enough, the skin is isotropic.

## Only the `placement` rack state is Kit-validated (2026-08-28)

`third_out` (24 planned forks) has **never run under Kit**; fork drop-bounce from the 60 mm
release hover is the predicted failure (`"disturbed"` aborts), with `DISTURB_POS_M` or a
`flat_lay_third` carve-out in `IsaacOracle` as the sanctioned knobs. `middle_out` plans **zero**
items — cup fails the measured settle-reliability gate, tumbler fails the z-budget — and that
zero has not had the same honesty audit the lower rack just received, so it should be treated
as an unexamined verdict rather than a machine property.

## The stowed lower rack interpenetrates the tub (2026-08-28, measured)

`MACHINE_GEN["bosch800"]["racks"]["lower"]["rail_z"]` is 0.185 — exactly `tub.floor_z` — so the
rack's wire deck sits at z 0.1962, **24 mm inside the LowerSprayArm** (authored z 0.185-0.220).
In `both_in` the stowed rack collides with the tub body in **493 piece-pairs**, giving bowl
0/56 and plate 0/8 placeable; lifting the deck clear recovers bowl 21/56, plate 8/8. The unused
authored `"wire_z": 0.043` shows the intent (rail_z ≈ 0.230). `placement` never sees it because
the rack is outside the tub. Fixing it is a `MACHINE_GEN` change — hashed, so a full Bosch
rebake plus regeneration of the capacity plan and its settle verification.

## Capacity is limited by the dish and rack model, not the machine (2026-08-28)

Three modelling gaps cap the certified lower-rack load at 15 items:
- the dish library is ~half scale (plate = 139 mm disc, `scale` 0.54, an ArtVIP-era constraint)
  while this machine is published as taking 270-320 mm plates;
- the plate tine bank is modelled as **one** rank spanning 400 mm, where
  `docs/bosch800_source_data.md` derives ~500 mm across **two** rows (~20 plate positions) —
  that derivation *is* the 16-place-setting rating;
- the bowl candidate lattice is a 60 mm grid, capping the open floor at 8 bowls where the free
  area admits ~12 off-grid.
The first two are `RACK_GEN`/object-spec changes (full rebake); the lattice is not hashed.

## The bowl lean fixture was retired (RACK_GEN v4)

The robot-era gripper stabbed the rack at every hover of the 48° lean, so v4 removed the
lean fixture and bowls stand upright on the floor grid (`floor_stand`). The `bowl_lean`
placement mode was removed from the code with RACK_GEN v4 adoption.

## `placement_open` is a 0-placeable reference state

With the v4 slots in the lower rack's front band, the *extended* upper rack shadows all of
them: every class measures 0 placeable slots in `placement_open` (both racks out). The state
is kept as a bake-able reference and as the only real case exercising the rack-state
tie-break in `config.resolve_rack_state`; no v4 flow places in it.

## Capacity-fill realism edges

- **The saucer class is retired from the fill** (it stays in the object library). Measured
  over four fills: mid-gap saucers slip 21–26 mm / 32–42° during the rack stow (v3's apparent
  stability came from the old basket bracing them from below), and even the candy-cane-braced
  end gap is marginal — parked at 3.9° drift in one run, tipped 46° at stow in another.
  Capacity trades three saucers for a reliably closable rack.
- The wine-glass stemware lie-in (a physics stretch goal) has never settled within the
  stability gate; both glasses park in every run.

## Measured rack-design rules (from the retired design harness)

Recorded 2026-08-09 during the v4 redesign diagnostics; they constrain any future rack change:

- A goal pose that *leans* the payload onto a fixture is unplaceable by construction — it sits
  inside the `COLLISION_MARGIN_M` inflation. Release poses must hover in free corridor space
  and let gravity settle the lean (the `basket_drop` pattern).
- Tie wires cross the tine gaps *through the disc plane* — a placeable plate bank must not
  have them (`tine_tie_frac: None`).
- The fork-drop column clears the machine only through the open mouth front: drop columns
  must stay at rack-frame y where the world-x of the column is in front of the shell top
  (bays at y ≲ 0.10 measured safe, ≥ 0.126 measured capped).
- Slots at rack y ≳ 0.14 risk the stowed upper rack's roof (y ≥ 0.200, z ≥ 0.154) through the
  payload's lateral extent; feasibility there is yaw-dependent — measure, don't assume.

## Upright drinkware does not stand reliably on the Bosch wire racks (v2, measured)

The Bosch racks' OEM-derived lattice (runners at ~41 mm, crossbars at ~46 mm) is coarser
than the scaled drinkware bases (~50-60 mm), so a cup or tumbler released upright wedges
base-first into a wire cell roughly half the time. Release-at-goal probes
(the retired probe_plate_settle.py, git history; 2026-08-14, tolerances of record):

| class / rack | stable | note |
|---|---|---|
| bowl / lower | 59/60 (98 %) | wide base spans any cell — the control |
| tumbler / lower | 64/88 (73 %) | |
| cup / lower | 49/82 (60 %) | |
| cup / middle | 63/120 (53 %) | 228/300 (76 %) even anchored to wire crossings |

Consequences, encoded in `capacity.MEASURED_SETTLE_RELIABILITY` (bar: 90 %): cups and
tumblers are excluded from the certified load on both racks, and the middle rack — whose
only capacity-relevant load was drinkware — contributes zero certified destinations (its
`middle_out` state and scripted transition remain in the full-load flow). The real-world
fix is how humans load glasses: rim-down, or the middle rack's cup shelves — both need
placement modes this stack does not have, future work alongside the real-scale dish
library (larger bases would also clear the bar). The v1 machine never hit this: its v4
rack has a ~30 mm crossbar pitch.

## The loaded lower rack cannot be driven back over the door sill (v2, measured)

Stowing the Bosch lower rack under load stalls: with one bowl aboard the drive settled
176.9 mm short of stowed (phase_smoke6, 2026-08-14). Rolling OUT onto the door is downhill
and reliable — every extension transition uses that direction — but the return climb over
the authored sill/runway step defeats the rack drive. Re-authoring a sill ramp is a
`MACHINE_GEN` change (full Bosch rebake), deferred. Until then the full load fills TOP-DOWN
(`third_out` → `middle_out` → `placement`, see `capacity.LOADING_STATES`): every scripted
transition stows a tub-rail rack or extends the lower rack outward, and the load ends with
the machine open — consistent with the v0 door-locked-open semantics.
