# Hardware limits of the UR5e + Robotiq 2F-85 with every dish mouth-up: graspable, and reachable to which rack positions
Slug: 2026-10-23-hardware-limits-mouth-up | From: research/2026-10-23-robot-grasp-reach-limits.md (lit review done; the user was away, so the grill's three items are decided below)

## Goal
One object type at a time (HOTEC bowl, plate, cup), lying mouth-up on the counter: (1) is it graspable by the UR5e + 2F-85,
(2) which rack positions can the arm place it into. Output: a per-object, per-slot gate table with videos of the primary
cases, on a page the user reads by 09:30 ET with the conclusions defensible at that time; the sweep may continue after.

## Decisions (2026-10-23 05:45 ET, the session's; hardware fixed by the user: UR5e + 2F-85, no substitutes)
- Stack: the repo's robot stack (`code/frigidaire/src/dishsim_frigidaire/robot/`, `code/execution/frigidaire/`), mount
  fixed at the measured (0.45, -0.55, 0.90), yaw 2.60 rad (`docs/robot.md`; the mount file is not on this box, so it is
  written from those numbers). Isaac 4.5 on the box; the gripper colliders de-instanced (landmine 1); sleep threshold 0.
- Grasp families: bowl = the existing rim pinch; cup = the rim pinch with the cup's rim radius and wall; plate = a
  top-down rim pinch on the flat plate, as it lies (the literature says this fails on a rigid two-finger gripper: that is a
  result, not a bug; the surface-slide primitive is a later round). 12 yaw candidates per object, hover posture as today.
- Experiment 1, grasp gate on the counter (Isaac, per object, n = 10, +-5 mm position perturbation, seeded): G3 = lift
  0.15 m and hold 5 s with drift < 5 mm and no pad slip; record hold / slip / pivot / no-reach per trial, the measured
  pad forces, and the pivot angle on the flared rim. A trial counts only after the harness invariants pass.
- Experiment 2, reach sweep (per object, every rack site of the packer's families: plates the 19 lower gaps at the
  upright-with-lean-bias pose, bowls the lower zones' sites plus the 12 upper slots, cups the 12 glass slots; site poses
  from `slots.py` / the HOTEC families): Kit-free gates on the FCL model with both racks, the door and the counter in the
  world, the dish attached to the gripper: G0 IK on any of the 8 branches at the release pose; G1 collision-free IK at
  pre-grasp, grasp, lift, pre-place and release; G2 RRT-Connect from the counter grasp to the release, fixed budget,
  budget failures counted apart. Then Isaac on every slot that passes G2: G4 = release, settle, `evaluate_placement`
  at goal, n = 3 this round (n = 10 later). Per cell: the gate reached, the fraction of yaws passing, the branch used.
- Upper rack pushed in while the lower rack is loaded and vice versa (the measured geometry); the other rack's state
  is part of the cell.
- Videos: Isaac cameras crash on this box, so each video is a Warp replay of the logged trajectory (the arm's links as
  their USD visual meshes, or the FCL hulls if loading is slow, the dish, the racks, the counter; 10 fps, mp4): one
  grasp-gate success and the characteristic failure per object, one placement success and one failure per object. If
  the replay is not working by 08:15 ET, a six-frame strip per case instead, and the note says so.
- Deliverable: `data/media/robot/limits_v1/index.html` (from `frigidaire_robot_report.py` or a sibling) with the
  question, the setup, the conclusions defensible at the checkpoint, the grasp-gate table, the per-slot tables as rack
  heatmaps, the videos, the three-line lit-review summary, and a dated "not done" list. HARD CHECKPOINT: the page,
  its media and the run note are on the Mac and the runner has reported by 08:45 ET, whatever is finished; the box
  may keep running the rest. Parallel: the three objects as three Kit processes (cores 0-2), the Kit-free sweep on
  core 3.
- Box cost: start when the code is ready; stop by 09:20 ET unless the sweep is still producing; the self-stop is the
  backstop.

## Change
- `robot/grasp.py`: cup and plate rim families (parameters from the HOTEC catalog); a Kit-free `frigidaire_robot_reach.py`
  (G0-G2 sweep, table JSON); the grasp-gate and placement runs on `frigidaire_robot_episode.py`'s harness with a
  `--grasp-gate` / `--place-slot` mode; the Warp arm replay in the clips renderer; the page.

## Run
Box: assets mirror (`mirror_robot_usd.sh` with a box TMPDIR), `frigidaire_arm_model.py`, `frigidaire_robot_smoke.py`;
then Experiment 1 (three processes), the Kit-free sweep, Experiment 2 on the passing slots, the replays; copy back;
page; run note `results/2026-10-23-hardware-limits-mouth-up.md`.

## Works when
By 08:45 ET the page shows at least the grasp-gate table for the three objects with one video each and the Kit-free
reach table for at least the bowl; the conclusions on the page are only what the gates measured; the note lists what
is not done. The box is STOPPED or its remaining work is named.

## Assumed / skipped
Assumed: the 6.0 UR5e + 2F-85 assets still download; the de-instancing fix holds on this box. Skipped: the mount
search, the surface-slide plate primitive, mouth-down grasps, neighbours in the rack (single object only), n = 10.

## Round 2 (added 2026-10-23 07:35 ET after round 1 reported; the session's call, user away)
Round 1 (results/2026-10-23-hardware-limits-mouth-up.md): bowl G3 10/10, cup 9/10, plate 3/10 only with the approach
unchecked; packer poses 0/50 at G1; level mouth-up targets G2 15/19 bowl and 8/12 cup sites; Isaac G4 bowls 21/46,
cups 0/24 (released ~65 mm above where they rest, dropped, stayed in the rack). Round 2, same hardware, same mount:
- Cups: G4 at the 8 G2 slots with the release rule of the upright3 episode (lower until contact, then open), n = 3.
- Bowls: n = 10 at the 12 released sites of round 1 (Clopper-Pearson intervals); the front-right and rear-right sites
  with the same lower-until-contact release, to see whether the 21-35 deg tilt comes from releasing a pivoted bowl in
  the air.
- One diagnostic cell for the Isaac-versus-Kit-free hover disagreement (0.10, 0.125, 0.15 m) if time allows.
- Page Version 3 by 09:25 ET with the updated tables and any new pass video; report by 09:10 ET; box stopped by 09:15.

## Round 3 (added 2026-10-23 09:15 ET; user: run the next experiment and add failure videos)
Question: can a bowl or cup be placed at an orientation other than level, and which failure modes does each limit have on video.
- Orientation sweep, bowl and cup, every site that passed G2 in round 1 (15 bowl sites, 8 cup slots): target tilts 0, 30, 60,
  90, 120, 150 deg of the dish about its horizontal axis toward the site's lean side (0 = level mouth-up, 150 = near the
  packer's family angle), the rim pinch unchanged, the dish's pose in the hand as measured after the lift. Kit-free gates
  G0-G2 over all cells first (the tilt changes the held-dish geometry and the release pose); Isaac G4 with the contact
  release, n = 5, at every cell that passes G2; per site report the largest tilt placed and the first tilt that fails, with
  the cause (no IK / collision at pre-place / path / dropped / tilt over tolerance / not at goal).
- Cup contact detector: find why it fires 47-87 mm early on the right glass slots (round 2) and fix the one cause if it is
  the detector or the slot pose; rerun the right slots at level, n = 3.
- Plate surface-slide grasp gate on the counter (Voysey 2021's primitive: jaws horizontal, lower to the counter, rise 2 mm,
  slide 4 cm toward the plate centre, close on the edge), n = 25, with the 15 mm clearance check ON; report hold / slip /
  no approach; no plate placement unless the gate reaches 20 of 25.
- Failure videos (Warp replays), one per failure mode seen, labelled with the mode and the cell: bowl side-site tilt,
  bowl marginal lower-rear site, cup early contact on a right slot, cup drop from the level release, plate pinch slip, plate
  slide failure if any, orientation-sweep failure at the first failing tilt per object, a packer-pose collision at the
  pre-place (a Kit-free render of the colliding pose is enough).
- Page Version 4: the orientation table (site x tilt, gate reached), the fixed cup table, the plate slide gate, a
  "Failure modes" gallery with the videos, conclusions updated only where a gate changed; run note "Round 3" section.
- Box: one window, 3 Kit processes; hard cap 2 hours; stop and confirm STOPPED.
