# Known limitations and open items

Honest negative results and open engineering items, with the measured evidence for each.
Numbers below trace to recorded runs (see the Results section in the README) or to
`docs/success_criteria.md`.

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
