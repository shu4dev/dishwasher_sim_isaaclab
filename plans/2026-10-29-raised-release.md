# Bowls: let go up to 30 mm above the seat where the dish alone seats; same ring hold, carry and hand above
Slug: 2026-10-29-raised-release | From: plans/2026-10-29-ring-place.md, results/2026-10-29-ring-mount.md, results/2026-10-29-racks-out.md

## Goal
With round 17's foot-ring hold, mouth-down carry and hand above, each easy_bowls bowl is let go at the lowest height h in {0, 10, 20, 30} mm above its goal seat at which (a) the arm and gripper clear the scene Kit-free and (b) the v4 bowl alone, released from h at that slot, seats mouth down at least 4 of 5 times; the page's bowl line says how many of 7 seat.

## Decisions (from grilling, 2026-10-29 22:45-23:15 EDT)
- Why (user): at the seated release the pads sit 68-73 mm above the rack floor, below the tine tips (lower 95, upper 91 mm); bowls 05 and 06 are blocked by exactly that, 02 and 01 by the cabinet's top edge and the neighbour bowls (18-36 mm overlaps). "Change the approach a little": let go higher, same hold and carry.
- Release rule relaxed for bowls (user): a free fall of at most 30 mm among the tines is allowed where the dish alone, released from that height at that slot, seats mouth down >= 4/5 on the v4 bowl, measured in the same window before the robot tries it. Rejected: keeping the supported release and moving bowls to other slots; waiting for the sweep before deciding.
- Round (user): one box window: the drop curve on v4 at the blocked slots, then the D-ring placement of all 7 with the raised release; cap 40 billed min, raised to 45 at the plan grill.
- Heights (Kit-free sweep, kitfree_v1/release_height/, running since 23:00 EDT): the hand check at h = 0-100 mm per bowl decides who can clear at h <= 30; a bowl with no clearing h <= 30 keeps a "no reach" row.
- Plan grill (user, 23:10 EDT): cap 45 billed min (the five steps sum to about 37.5 with the 11.5 min stop; kill at T0+31, stop by T0+33); the curve measures 10, 20, 30 mm with 5 repeats at each blocked slot (60 drops, parallel per bowl); Version 27 only if bowls beat 3/7; seed 8's sampler certify at the end if time allows, dropped first if the window runs long. Rejected: cap 40, the clearing heights only, Version 27 regardless, a bowls-only window.
- Gate (coordinator): the box window opens only if the sweep shows at least one blocked bowl clearing at h <= 30 mm; the runner implements and dry-runs meanwhile and waits for the coordinator's go with the sweep table.
- Sweep result (23:20 EDT, kitfree_v1/release_height/table.md, 7 bowls x 8 heights, 0 mismatches vs racks_out at h = 0): by the hand check no blocked bowl clears at h <= 30. bowl_06's hand already clears at h = 0 at the flush travel (1 spin; 4 from h = 20) but its recorded goal is 6.9 mm inside a lower-rack tine Kit-free and stays inside to h = 75; bowl_01 first clears at h = 50 (1 spin) and 75 (3 spins), +2.7 mm at 30; bowl_02 never (wrist 3 / gripper 17-34 mm inside the tub-ceiling sheet, the bowl leaning 48 deg back under it); bowl_05 never (the forearm vs the pushed-in upper rack at the tub mouth replaces the tine as h grows; the bowl in a tine to h = 75). Pads clear the lower tine tips from h = 30, the upper from h = 20. Controls 07, 04, 03 pass at every h.

## Change
- frigidaire_robot_episode.py: `--release-height-table <json>`: h per bowl; the descent ends at the Kit-free seat + h by position (the sensor still ends it on an unexpected contact, recorded), open, 0.5 s, rise 100 mm, rest wait, jaw watch; rows "D-ring-h".
- frigidaire_drop_curve.py: `--gap-mm 10,20,30` at the goals_v4 poses of the blocked bowls on v4, the witness-order scene, 5 repeats, 2 mm / 2 deg jitter; the summary gives per bowl the lowest h with >= 4/5 seated and mouth down.
- A Kit-free chooser: h = the lowest height both the sweep and the curve allow; 07, 04, 03 at h = 0.
- Page: Version 27 = the bowl placement clip and sentence swapped only if the count beats 3/7 (the minimal page keeps its two sections).

## Run
Box, one window (the rules of 2026-10-29): 1 re-export the twin USD in the container (build_frigidaire.py needs PhysxSchema; the shipped USD still clamps the racks at -0.49 / -0.44) and md5 the twin against the Mac; 2 the drop curve, parallel per bowl; 3 the chooser; 4 the placement in the witness order; 5 replay + still, copy back, stop. Mac: the fake-Kit dry run first; the note and the page after.

## Works when
The sweep table (7 bowls x 8 heights: passing spins, smallest clearance, pair); the curve table (blocked bowls x heights x 5); the chosen h per bowl; 7 D-ring-h rows (stop vs plan, contact after the open, mouth, seated, AT_GOAL); results/2026-10-29-raised-release.md; Version 27 only if > 3/7.

## Assumed / skipped
- Assumed: a bowl blocked at h = 0 cannot touch the rack first, so its descent is a position stop at seat + h with the sensor as a guard; posture_v1 mount; the flush-out travel (the loaded rack out, the other in); 15 mm / 2 mm margins; the witness order 06, 02, 07, 04, 03, 05, 01; goals_v4 (05's not a rest pose, stays flagged); 20 mm/s descents.
- Skipped: other final poses (slots), the sampler re-run, cups and plates under Part B, the hub upload of the twin (the user's go).

## Outcome (2026-10-29 23:30 EDT): closed by the user before any box time
- The sweep (kitfree_v1/release_height/) opened nothing within 30 mm; the user chose to keep the old approach (the supported release: sensor descent to contact, open, no free fall) and to test instead whether the ring can be held sideways (the hand tilted off the bowl's axis), plans/2026-10-29-ring-tilt.md. The runner's phase-1 edits for the raised release are reverted; the chain's USD re-export step is kept as a standalone chore. No box window.
