# UR5e + Robotiq 2F-85 in the Frigidaire twin (bring-up 2026-09-29; hardware-limits study closed 2026-10-30)

Goal (user): execute an instance's moves with a real arm instead of teleports; test case easy_s0 (greedy plan).
Decisions: pure friction grasp (no weld); pedestal mount chosen by a reachability search; upper rack pushed in
while the lower rack is loaded (the extended upper rack covers the lower rack: no gap); one re-grasp per move.

## Code

| File | What |
|---|---|
| `code/frigidaire/src/dishsim_frigidaire/robot/ur5e.py` | articulation cfg (ported from on-corrallab; `robot_cfg` takes finger_joint's effort limit, stiffness and damping and the inner finger joints' gains since 2026-11-04, though PhysX gives the inner finger joints no drive), `deinstance_gripper`, `pad_material`, `set_gripper` (mimic-consistent inner fingers) |
| `robot/kin.py` | analytic UR5e IK/FK (ported unchanged) |
| `robot/grasp.py` | grasps of the HOTEC dishes: the upright rim pinch per kind (`bowl_grasps`, `KIND_RIM`: bowl, cup, plate), the bowl's foot-ring clamp and rim side point (lying), the mouth-down bowl's foot ring (`foot_grasps`, pads on the ring's outer or inner face; `foot_side_grasps`, the same outer face from a horizontal approach, both wrist sides), the cup's wall pinch (`wall_grasps`) and side pinch (`side_grasps`, the wrist on the mouth or base side); tilted variants that clear the counter slab (`tilted_grasps`) |
| `robot/rig.py` | Kit controller: joint moves with velocity feed-forward, IK lines, gripper, blocked-motion detection |
| `robot/collide.py` | FCL arm model (convex hull per link from the USD collision meshes, calibrated link offsets) vs appliance, counter, dishes, pedestal, floor; RRT-Connect in joint space: `rrt_connect` tries OMPL and falls back to the numpy `rrt_connect_np`, which planned every hardware-limits path (the dishsim-l4 container has no OMPL, 2026-10-23, and the Mac's Python environments have none either, checked 2026-10-31) |
| `code/execution/frigidaire/mirror_robot_usd.sh` | Isaac 6.0 UR5e + 2F-85 asset mirror (11 MB, data/assets/robots) |
| `code/execution/frigidaire/frigidaire_grip_probe.py` | friction gate G0 and the collider diagnostics; since 2026-11-04 the grip-force sweep (`--inverted --foot-clamp`: the steady per-pad force over a 5 s hold and the drive reading) |
| `code/execution/frigidaire/frigidaire_grasp_test.py` | generated grasps on one bowl, per pose class |
| `code/execution/frigidaire/frigidaire_arm_model.py` | exports the arm collision model to data/results/robot/arm_model.json |
| `code/execution/frigidaire/frigidaire_robot_mount.py` | Kit-free mount search on floor / counter grids (`--grid`): an elbow-up analytic IK solution (every branch with `--any-branch`) counts only where `free_reach` finds the FCL arm model free (`collide.ArmWorld.free` on `arm_model.json`) at the dish's start or goal grasp and at its 0.10 m hover; the record carries `collision_checked: true` (the module docstring's "NOT checked" sentence is stale) |
| `code/execution/frigidaire/frigidaire_robot_episode.py` | the episode: benchmark start state + robot, two phases, per-move report, video; since Phase 0 (2026-09-30) run under `robot/flags.py` profiles (`headline`, `legacy_upright3`) and the harness, one folder per `--run-id`; since 2026-10-23 also the hardware-limits modes: the grasp gate (`--grasp-gate`), slot placement (`--place-slot`) and goal placement (`--place-goal` with `--variant`, `--hold`, `--carry`, `--insert`, `--row-variant`, `--mount`) |
| `robot/flags.py` | named run flags: every benchmark relaxation (R4-R6), the R1/R7 thresholds, R2 gains (finger_joint's effort, stiffness and damping and the inner finger joints' gains; the episode's `--grip-drive` overrides them), D1-D3 rules, the counter cap, the diagnostic continue mode; `headline` and `legacy_upright3` profiles |
| `robot/harness.py` | Kit-side harness (plan Phase 0.3-0.4): sleep threshold 0, PhysX contact reports (touching pairs, normal force per pad), the seven auto-fail invariants, the diagnostic settle routine, fixed-camera video with the simulated clock, key frames |
| `robot/triallog.py` | JSONL trial log (sim_s and wall_s separate; attempt and config_id separate) |
| `code/execution/frigidaire/frigidaire_robot_conventions.py` | live-asset conventions, the 12-joint loaded close (D7 / H2 runtime test), the arm-collider press test (D12); fixture via `frigidaire_robot_conventions_fixture.py` |
| `code/execution/frigidaire/robot_asset_manifest.py` | `data/assets/robots/MANIFEST.sha256` (D11) |
| `code/execution/frigidaire/frigidaire_robot_report.py` | Kit-free report: plots from the trial logs, `data/artifacts/<run-id>/index.html`, `REPORT.md` (the 2026-09-30 one is now `results/2026-09-30-robot-phase0-report.md`) |
| `code/execution/frigidaire/frigidaire_robot_reach.py` | Kit-free reach sweep, gates G0-G2 on the FCL arm model (a counter grasp to every rack site); modes `--orient`, `--plate-in-hand`, `--counter-grid`, `--side-candidates`, `--side-spot`, `--foot-candidates`, `--mount-search` (writes `data/results/robot/mount/posture_v1.json`) |
| `code/execution/frigidaire/frigidaire_robot_replay.py` | Kit-free Warp replay of a logged trial (the arm and gripper as their collision hulls; the box's RTX renderer crashes with cameras): mp4, six-frame strip, stills, one mp4 per placing load (`--load-dir`, `--variant`) |
| `code/execution/frigidaire/frigidaire_drop_curve.py` | Kit: each dish of a placing load dropped alone (no robot) at gaps above the rack stop, judged seated on the goal's wires; `--goals`, `--perch-roll`, `--refresh-goals` (the refreshed goals `goals_v4_<tag>.json`), `--rejudge` (Kit-free) |
| `code/execution/frigidaire/frigidaire_tines_test.py` | Kit, no robot: does the rack alone tilt a level bowl at the right-side lower sites (tilt past 15 deg = the rack) |
| `code/execution/frigidaire/frigidaire_placing_loads.py` | stdlib: the Part B goal loads, single-kind subsets of certified sampler goals, into `data/results/robot/placing_v1/<load>/load.json`; `--summarize` |
| `code/execution/frigidaire/frigidaire_robot_limits_page.py` | stdlib: the 2026-10-23 limits round's page (G3 table, G0-G2 rack maps, G4 trials, replays); its helpers also serve the report page |
| `code/execution/frigidaire/frigidaire_robot_report_page.py` | Kit-free (matplotlib): the Hardware Limits Report page; `--minimal` builds the Version 26 content (the grasp table and clips, the best placement per kind) |
| `code/frigidaire/tests/test_robot_{kin,conventions,flags,harness}.py` | Kit-free tests (the conventions test reads `code/frigidaire/tests/fixtures/robot/conventions.json`) |

## Measured findings (landmines)

1. **Instanced gripper colliders make no contacts on Isaac 4.5** (the 6.0 asset): PhysX creates the shapes but
   the fingers close through a 30 mm cube, the ground and the bowl (0 N). `SetInstanceable(False)` on the gripper
   prims before `sim.reset` fixes it (cube held at 26-60 N). This was the 2026-08 "pads give 0 N" regression.
2. The TCP (`T_WRIST3_TCP`) is at the fingertip ENDS; the pads span ~6 mm beyond it to ~32 mm behind it; open
   pad faces are 87 mm apart, closed 2 mm; closing axis = TCP y. The hand is side-mounted (the TCP 130 mm off the wrist-3
   axis): see the hardware-limits study below.
3. A sleeping bowl floats when its support is teleported away (use `sleep_threshold = 0`).
4. Rim pinch of an upright HOTEC bowl: the flared rim lets it pivot ~30 mm on the lip, then it hangs stably
   (0.3 mm drift / 3 s at 40 N per pad; stiffness 400, effort 8 N m). Pad friction 1.0 changed nothing.
5. Mouth-down bowl: the 4 mm foot ring is the only top-down feature; a centred clamp is blocked by the widening
   body, an off-centre clamp held once when the bowl was placed between the open pads, but approached from
   above the inside pad lands on the bowl floor (4 mm below the ring top). Not reliable yet (2026-09-29). Since
   2026-10-29 the measured ring (r 29.70 to 32.86 mm, 5.57 mm tall) holds 10/10 with the pads closing on its outer
   face and 0/10 (slip) opening on its inner face (results/2026-10-29-foot-ring-grasp.md).
6. The implicit PD drives lag by damping x velocity / stiffness (0.16 rad on the wrists at 1.2 rad/s): joint
   paths need velocity feed-forward. The UR5e elbow limit is +-pi (planner bounds).
7. Geometry of the scene: the extended lower rack's rear ~12 cm is under the counter's front edge (y -0.30);
   the extended upper rack covers the lower rack. With the collision model no single floor mount reaches every
   easy_s0 start and goal: best (0.45, -0.55, 0.90) reaches 7/7 starts and 6/7 goal proxies (all IK branches).

## Phase 0 of plans/2026-09-29-easy-s0.md (2026-09-30)

The audit, flags, harness, invariants, convention tests and the generated report live in
`results/2026-09-30-robot-phase0-report.md` (generated as the repo root's `REPORT.md` by `frigidaire_robot_report.py`,
moved on 2026-10-31 with one header line added, otherwise never edited by hand) with the per-run pages
`data/artifacts/<run-id>/index.html`.
Runs: `data/results/robot/runs/<run-id>/` (records) + `data/artifacts/<run-id>/<trial-id>/` (trial.jsonl, video, frames, plots).
Two measured corrections to the text below: "143 s simulated" was the mp4 length (every run now logs `sim_s`); the rim
pinch rests the bowl's flared rim on the inner KNUCKLES (21-49 N), not only on the pads, which the plan's D4 invariant
rejects. Instanced arm colliders DO make contacts on 4.5 (press test), only the gripper's were dead.

## Status on 2026-09-29/30 (superseded: see "Hardware-limits study" below)

**2026-09-29: the easier case PASSES** (user's scope call after easy_s0 proved hard): `--test-case upright3`
(3 upright bowls on the counter -> 2 lower-rack spots with the upper rack pushed in, then 1 upper-rack spot;
release = lowered until contact, as the bowl hangs): 3/3 moves on the first attempt, all racked, none disturbed,
jaw 0.60 rad on the wall, 147.5 s simulated (measured 2026-09-30 by the harness; the earlier '143 s' was the mp4 length) / 7.9 min wall. Video `data/media/robot/robot_upright3/episode.mp4`, record
`data/results/robot/episodes/robot_upright3.json`, page https://claude.ai/artifact/Lkm8mUMhRBP8iddoLjKSMu.
Mount (0.45, -0.55, 0.90), yaw 2.60 rad (`data/results/robot/mount/easy_s0_any.json`, all IK branches).

What made it work, in order of discovery (each measured in this session): de-instanced gripper colliders; the
stable-tilted rim hold measured in the hand after each lift; velocity feed-forward (PD lag); real joint limits
(elbow +-pi); a per-link FCL arm model with the gripper placed by LIVE poses and one hull per gripper collider (a
single hull fills the jaw gap); contact exemptions for the gripper only; wrist-up IK preference (a flipped wrist
hangs its housing at the dish); grasp candidates paired with a hover posture whose approach line is continuous and
free; carrying: rise to Z_SAFE first (a joint move dipped the dish into the counter), 0.25 rad/s, smoothstep timing,
the carried dish checked against the arm's own links, a looser settle tolerance while carrying.

Open for easy_s0: mouth-down bowls (foot-ring clamp unreliable), the lying bowl under the counter edge, one goal no
mount reaches; placements into the benchmark's tilted mouth-down goal poses need a regrasp or a wrist flip.

## Hardware-limits study (2026-10-23 to 2026-10-30, closed)

Question: which HOTEC dishes the UR5e + 2F-85 can grasp, carry and seat at the sampler's goal poses in the Frigidaire
twin, in simulation only (Isaac Sim 4.5), from one pedestal mount (plans/2026-10-23-hardware-limits-mouth-up.md,
plans/2026-10-26-placing-goal.md). The goals are single-kind subsets of certified sampler loads, the 7 bowls of
`rest_easy` seed 5, the 8 plates of `bp_medium` seed 5 and the 8 cups of `rest_hard` seed 2 (`frigidaire_placing_loads.py`),
placed in the witness order with the earlier dishes at their goals. Mount of the closing rows:
`data/results/robot/mount/posture_v1.json`, (0.45, -0.70, 0.60) m, yaw 2.5632 rad. Page: Hardware Limits Report,
https://claude.ai/artifact/Fv7YPrB7mHdBStZSBEYs5H (private), the Version 26 content: the grasp table with the bowl and
cup gate clips, and the best placement per kind, three clips (`frigidaire_robot_report_page.py --minimal`). Records:
`data/results/robot/{limits_v1,exhaustive_w1,exhaustive_w2,placing_v1,kitfree_v1,drop_curve,perch_roll,goal_refresh,bowl_drop100,coax_mount}/`;
media under `data/media/robot/`.

Method:
- Kit-free, on the FCL arm model (`frigidaire_robot_reach.py`; convex hulls of the USD collision meshes in
  `data/results/robot/arm_model.json`): G0 an analytic IK solution on any of the 8 branches at the release pose, no
  collisions; G1 a collision-free IK solution at pre-grasp, grasp, lift, pre-place and release; G2 the approach, lift and
  insertion lines and an RRT-Connect leg (`rrt_connect_np`), all collision-free.
- Isaac: G3 is the grasp gate on the counter (`frigidaire_robot_episode.py --grasp-gate`, n = 10, lift 0.15 m, hold 5 s,
  drift under 5 mm, no pad slip). The placement episodes (`--place-goal`) weld the dish to the hand with a fixed joint
  at its in-hand pose, the gate's measured one (the plates' assumed; plans/2026-10-26-placing-goal.md), and carry it;
  bowls and plates are lowered at 20 mm/s until the emulated flange sensor reads more than 3.5 N on 2 ticks
  (`DESCENT_SPEED_M_S`, `SENSOR_TRIP_N`), cups are let go 100 to 125 mm above the goal; the settled dish is judged by
  its mouth direction and AT_GOAL, bowls and plates also seated on the goal's wires.
- Videos and stills are Warp replays of the logged trials, the arm drawn as its collision hulls
  (`frigidaire_robot_replay.py`), because the box's RTX renderer crashes with cameras.

Hand (measured; the configuration behind every number in this section except the coaxial-mount bullet): the twin's 2F-85
is side-mounted. The fixed joint `/ur5e/joints/robot_gripper_joint` (body0 `wrist_3_link`, body1 the gripper's
`base_link`, localRot0 xyzw (0.5, 0.5, 0.5, -0.5)) turns the approach axis to wrist_3 +y, 90 deg from the flange normal
and joint-6 axis (wrist_3 +z), and puts the TCP 130 mm off that axis: `base_z_to_w3_z_deg` 90.0 and `tcp_off_axis_mm`
130.0 in `data/results/robot/coax_mount/joint_read.json`, matching the live side mount (max error 0.0;
results/2026-10-30-gripper-mount.md); `ur5e.T_WRIST3_TCP_POS` = (0, 0.130, 0). The hulls agree: in `arm_model.json` (md5
376cf0ff) the gripper's 11 hulls span y -3.4 to 148.4 mm and z -37.5 to 37.5 mm in the wrist_3 frame, whose flange face
is z = 0 (the same extents as plans/2026-10-29-ring-tilt.md). A coaxial re-mount was tried on 2026-10-30 and reverted at
the user's request, so the side mount stays the twin's configuration.

Closing numbers (side-mounted hand; the placements on mount posture_v1, the cup and plate gates of 2026-10-23 to 10-27
on the earlier mount (0.45, -0.55, 0.90)):

| kind | grasp held (G3) | placed at the goal | notes |
|---|---|---|---|
| bowls | foot ring, pads on the ring's outer face, mouth down: 10/10 (v3 bowl; results/2026-10-29-foot-ring-grasp.md) | 3 of 7 seated mouth down at their refreshed goals (07, 04, 03; v4 bowl), carried mouth down with the hand above, lowered until the sensor tripped, let go (results/2026-10-29-ring-place.md) | every stop within 2 mm of its prediction, no jaw contact after the open (0/3); bowl_07 seated by the axis fallback (0.37 mm, 3.33 deg), re-judged after the window; no reach: 02 the tub, 05 the upper rack and the tub, 01 the basket, the lower rack and bowl_07, 06 the lower rack; bowl_05's refreshed goal is not a rest pose (18.6 mm drift); none of 825 pedestal mounts reaches more than 3/7 with the bowl's own path clear (results/2026-10-29-ring-mount.md) |
| plates | top-edge rim pinch, base side, rim 6 mm in: assumed, no measured pick on this hand (the flat rim pinch held 3/10 with the planner's 15 mm clearance waived, the slide 0/25; results/2026-10-23-hardware-limits-mouth-up.md) | front bank 4/4 seated on the goal's wires and settled, lowered along the tine axis (results/2026-10-28-plate-seat.md, window 2); rear bank 0/4, no reach (window 1) | plate_07 stopped on the plate 1.7 mm from its seat; 03, 06 and 02 stopped on the hand 31.7 to 34.9 mm above their seats and slid down the tines into them; the rear four have no collision-free IK continuous to the bound at any descent start from +50 to +150 mm (wrists 1 and 2 against the upper rack); let go the sampler's way (I5), 0 of the 4 front plates seated (results/2026-10-29-sampler-release.md) |
| cups | side pinch: 10/10 at 11 mm and 10/10 at 45 mm under the rim (results/2026-10-26-side-pinch.md); mouth down, 11 mm from the base with the wrist toward the base: 10/10 at posture_v1 (results/2026-11-04-cups-mouth-down.md) | 8 of 8 settled mouth down, let go 100 mm above the goal (cup_01: 125 mm) (results/2026-10-27-posture-fix.md); s0's cups picked mouth down and lowered until the sensor tripped on the seat: 3 of 5 seated (one dropped in carry, one with the hand on the rack 15 mm above its goal; results/2026-11-04-cups-mouth-down.md) | the hold was the 45 mm mouth-side pinch rolled 180 deg about the approach (the base-side pick held 0/10, no collision-free grasp); only cup_07 lies within the old S5 tolerance (15 mm lateral, 20 mm height, 15 deg), and cups 01, 06 and 04 rest tilted 44 to 46 deg; at the earlier mount (0.45, -0.55, 0.90) the drop gave 6/8 with 2 no reach (results/2026-10-27-cup-side-drop.md) |

Tried and not adopted (bowls unless named):
- Perch roll: each bowl alone released on its tine-tip perch at 66 to 90 deg seated mouth down 0/115 (mouth down 34/115,
  AT_GOAL 0/115); no bowl reached the 4/5 gate, so no robot round ran (results/2026-10-29-perch-roll.md).
- Raised release: let go up to 30 mm above the seat; the Kit-free hand sweep cleared no blocked bowl at 30 mm or less, and
  the round closed before any box time (plans/2026-10-29-raised-release.md).
- Sideways ring hold: the two-pad ring hold tilted about the pinch chord opened only bowl_05, at +45 deg, Kit-free, with
  the bowl's own path 1.9 mm into the rack at the descent start (over the 1 mm allowance); closed on 2026-10-30 before
  any box time (plans/2026-10-29-ring-tilt.md).
- Wall pinch: one fingertip in the foot recess (7.3 mm of inner-face engagement) opened bowl_02 and bowl_05 Kit-free, its
  moment about the pinch untested; closed with the sideways hold (same plan). The cups' wall pinch held 10/10
  (results/2026-10-26-placing-goal.md), but its first drop settled 7 of 8 cups, all mouth up
  (results/2026-10-27-cup-side-drop.md).
- Coaxial mount: the 2F-85 on the flange axis held the bowl foot ring 10/10, the cup side pinch 10/10 and the plate
  top-edge rim pinch 0/10 (all slip); Kit-free it reached bowls 3/7, plates front 4/4 and rear 4/4 (the assumed hold) and
  cups 0/8 at the posture-fix drop poses; reverted (results/2026-10-30-gripper-mount.md).
- 100 mm bowl drop: alone from 100 mm the bowls seated mouth down 28/35 (AT_GOAL 21/35); by the robot 4 of 7 rows ran
  before the kill, 1 seated by the wire rule (bowl_06, propped 62 deg, off goal), and bowl_04 tipped 74 deg although it
  seats 5/5 alone from the same height; about 85 billed minutes (results/2026-10-30-bowl-drop100.md).

Closed by the user on 2026-10-30 (12:31 EDT): bowls stay at the 3/7 of the seated ring-hold release, and the drop, the
raised release, the sideways holds and the mount fix are recorded as tried (plans/2026-10-30-bowl-drop100.md, Outcome).

## Grasp and place of sample s0 on Isaac Sim 6.2 (2026-11-04)

Plan plans/2026-11-04-grasp-place-s0.md, run note results/2026-11-04-grasp-place-s0.md. The 15 dishes of the three-kinds
sample s0 (data/results/bench/samples/frigidaire_fdpc4221as__hotec_wheatstraw__plate5-bowl5-cup5_a3__s0.json) are placed one
at a time in its dag order on the posture_v1 mount, each dish in its own Kit process:
`frigidaire_robot_episode.py --place-goal data/results/robot/samples_v1/<sample_id>/load.json --dishes <id> --attach none
--state <previous state>`. The dish is spawned alone at the counter spot (0.20, -0.12) in its pick orientation (plate face
up, bowl mouth down, cup mouth up), picked by its kind's grasp with one re-grasp, held by the pads alone (no fixed joint, no
pin) and placed by its kind's move. Bowls use D-ring: the F-out foot ring hold, carried mouth down, lowered until the
flange sensor trips. Cups use D-side-mouth (the coordinator's option B): the measured 45 mm mouth-side side pinch, turned
mouth down with the wrist below the cup and let go at the lowest collision-free height above the goal. The rolled
base-side hold of D-side-m2 has no counter pick (0/12 yaws), and the study's 8 of 8 D-side-m2 cups had been spawned welded
in the hand. Plates use I4: the grasp gate's rim pinch with the dish clearance waived, the goal turned about the plate
axis so the pinch is the top edge, lowered along the tine axis. A carried dish more than 5 mm from the pick's hold before
the descent (cups: before the release) is dropped in carry. A placed dish stays where the robot left it (the state file
<load dir>/state/after_<k>_<dish>.json), and a failed dish is parked off the scene.

- Gate: `frigidaire_robot_reach.py --sample-gate` checks each dish alone on the arm model (the pick at the spot, the
  kind's release, the carry legs). It passed 9 of 15; the six rejects fail at the descent start: bowl_04 (gripper and
  wrists 2 and 3 against the Cabinet), bowl_02 and bowl_03 (the held bowl against its rack), plate_01 and plate_04 (the
  rear bank: wrists or forearm against the Cabinet and the upper rack) and plate_02 (wrists against the pedestal).
- Isaac: cup_01 and bowl_01 are placed, seated on their goal's wires and within AT_GOAL. Bowls picked 2/5, carried 1/5,
  seated 1/5 (bowl_05 touched an upper-rack wire on its last carry leg and shifted 5.1 mm in the pads). Cups picked 5/5,
  carried 5/5, seated 1/5, at goal 2/5 (from 100 or 225 mm, cup_05 landed 13.6 mm off, three landed on their side or
  mouth up). Plates picked 0/5: both front plates were blocked on the waived approach (shoulder lift lag 0.152 rad over the
  0.15 limit, palm contact), and the re-grasp's rise to the safe height hit an IK branch jump.
- 6.2: the grip probe holds the bowl with and without the gripper de-instance (`--fix pads` and `--fix none`, 0.1 mm drift
  over 3 s each), so landmine 1 does not reproduce on 6.2. The LowerRack has 773 collision parts since the rack wheels
  became visual, and `rack_labels(n_parts)` drops the 8 trailing wheel entries of rack_names.json.
- Records: data/results/robot/samples_v1/<sample_id>/ (load.json, gate.json, bowl_supports.json, dishes.jsonl,
  rows.jsonl, state/, runs/); media data/media/robot/samples_v1/<sample_id>/ (clip_, release_ and landed_<dish>);
  window scripts and logs data/logs/robot_s0/.

## Cups mouth down, lowered until contact (2026-11-04)

Plan plans/2026-11-04-cups-mouth-down.md, run note results/2026-11-04-cups-mouth-down.md. The five cups of s0 alone, in the
dag order, each in its own Kit process on the posture_v1 mount: `frigidaire_robot_episode.py --place-goal
data/results/robot/samples_v1/<sample_id>__cups_down/load.json --dishes <id> --attach none --state <previous state>`. The load
(`frigidaire_placing_loads.py --sample <sample> --cups-down`) gives every cup the move D-side-down (`--hold side-down`): the cup
spawned mouth down at the counter spot (0.20, -0.12), pinched by the side pinch with the wrist toward the base, so the hand is
above the cup, carried mouth down as held, then the D-ring path: the descent start 50 to 150 mm above the goal, the descent
straight down at 20 mm/s until the flange sensor trips (3.5 N on 2 ticks), the jaws opened, 0.5 s, the open hand backed 20 mm
along -TCP z and risen 100 mm, the cups' seated test (the goal pose within 2 mm and 5 deg) and AT_GOAL.

- Hold: the plan's pinch 45 mm from the base (pads 60 mm above the rim) has no release. At every goal its closed jaws sit
  inside the UpperRack's glass-row tines and rim wires (contacts 38 to 78 mm above the rack frame), and the hand is touch-free
  only from 30 mm above the goal (cup_05: 40 mm; `<load dir>/hand_height.json`). The user chose the pinch 11 mm from the base
  (pads 93.9 mm above the rim, `side_below_top` 0.0939, the mirrored h11 pick) and D-seat's 2 mm final-descent rule
  (SEAT_ARM_MARGIN_M for the hand and the links from the descent start to the seat; the start and the carry keep 15 mm).
- Isaac grasp gate at the spot (`--grasp-gate cup --grasp-family side --side-below-top 0.0939 --side-wrist-side base
  --mouth-down`): held 10 of 10, jaws 0.270 rad, hold drift at most 0.015 mm, pivot at most 0.03 deg.
- Result: the Kit-free gate passed 4 of 5 (cup_05: the hand inside the UpperRack 15 mm above its goal at all 24 spins). Picked
  4/5, carried 3/5, seated 3/5, at goal 3/5. cup_01, cup_04 and cup_02 stopped on the cup at the seat (3.75 to 4.44 N, within
  0.1 mm of the goal height) and settled within 1.4 mm and 1.9 deg of it, with no jaw contact after the open. cup_03 touched an
  UpperRack wire on its last carry leg (4.1 N for 0.24 s) and moved 5.1 mm in the pads. Last round's drops from 100 to 225 mm
  seated 1 of 5.
- Records: <load dir>/ (load.json, gate.json, gate_h60.json, hand_height.json, hand_height_down11.json, the Isaac gate
  partA_cup_side_down11/ with its parts, dishes.jsonl, rows.jsonl, state/, runs/); media data/media/robot/samples_v1/
  <sample_id>__cups_down/ (clip_, release_ and landed_<cup>); window scripts and logs data/logs/robot_cups/.

## Three fixes: carry clearance, the leaning plate, the build release (2026-11-04)

Plans plans/2026-11-04-bowl-carry-clearance.md, plans/2026-11-04-plate-pick-presentation.md and plans/2026-11-04-deep-sites-gate.md
(phase 2, the amended Decisions), run note results/2026-11-04-three-fixes.md. Three per-kind loads of s0 (`frigidaire_placing_loads.py
--sample <sample> --fixes`: `<sample_id>__fixes/{carry,plates,sites}/load.json`), each dish in its own Kit process with `--attach none`
and the chain's state, three chains side by side on cores 0/4, 1/5 and 2/6.

- Rule 1 (every `--attach none` run, from the end of the pick): the held dish keeps 5 mm (FCL distance, `CARRY_CLEAR_M`,
  `collide.ArmWorld.held_clear_m`) from the scene on every planned carry sample and at the descent start, where the bench
  checker's pair rule had let 1 mm of overlap pass; RRT-Connect edges are checked at 0.02 rad (`collide.EDGE_RES_RAD`, was 0.03),
  the spacing of the followed waypoints; the descent start is the passing candidate 50 to 150 mm above the seat
  (`DESCENT_STARTS_M`; I4: 5 mm steps) with the largest held-dish clearance. The clearance of an axisymmetric dish does not
  depend on its spin, so the rule picks the highest passing start (125 or 150 mm in this round) and the spin by the joint move.
  Isaac: bowl_05, cup_03 and bowl_01 placed, every carry leg straight, drift at most 0.02 mm and no scene contact on any leg.
- Plates (`--grasp-family edge --presentation <plates load dir>/presentation.json`; move I4-lean30): the plate leaning 30 deg on a
  static backstop (a 20 x 160 x 100 mm box authored at /World/Backstop, a counter fixture for the harness, and a static box in the
  arm model), picked by the top-down edge pinch at its top edge (`grasp.edge_grasps`, the TCP 3 mm outside the rim, both rolls).
  The Kit-free gate passed plate_05, plate_03 and plate_04 (plate_04 with the pinch spun 30 deg for roll 180, 0 for roll 0); the
  Isaac grasp gate (run partA_plate_edge_lean30) held 0 of 10 (8 misses, 2 slips): the TCP reaches the planned pinch within
  0.1 mm, the mouth-side pad meets the rim's top edge and the base-side pad's tip the base face 27 mm inside the rim, and the
  hand is pushed 25 to 32 mm outward, 26 to 32 deg above the plate plane, until the jaws close on air (0.79 to 0.80 rad).
- The build release (moves D-ring-build and D-side-down-build; the load dish's `build_release` from the goal record's
  `moves_world` and witness step): the dish in the build's move orientation, the descent along the rack's up axis until the
  flange sensor trips or the build's move pose is reached, the dish raised 20 mm (`BUILD_UP_M`), let go, the hand away (cups:
  the 20 mm back-off), the settle judged on the goal's wires with AT_GOAL beside. cup_05 and bowl_03 left the hand within 0.8 mm
  and 1.4 deg of the build's release pose; cup_05 settled at goal (9.8 mm, 4.7 deg, not seated), bowl_03 on a tine (41 deg).
  bowl_02's carry found no RRT path in 15 s at six starts; a second try with `--transit-rrt-s 60` found the path and the bowl
  slipped 5.7 mm in the pads on that leg with no scene contact.
- Rows carry `carry_watch.legs` (per carry leg: the largest drift and the held dish's PhysX contacts with anything but the hand)
  and `summary.carry_clean`. Records: `<load dir>/` (load.json, gate.json, presentation.json, the Isaac gate run, dishes.jsonl,
  rows.jsonl, state/, runs/); media data/media/robot/samples_v1/<sample_id>__fixes/<chain>/; window scripts and logs
  data/logs/robot_fixes/.

## The s0 rerun: every measured fix in one window (2026-11-04)

Plan plans/2026-11-04-s0-rerun.md, run note results/2026-11-04-s0-rerun.md. All 15 dishes of s0 in one box window, in two state
chains, one per rack, each dish in its own Kit process on the posture_v1 mount: `frigidaire_robot_episode.py --place-goal
data/results/robot/samples_v1/<sample_id>__final/load.json --dishes <id> --attach none --transit-rrt-s 60 --state <the chain's
previous state>`. The load (`frigidaire_placing_loads.py --sample <sample> --final`) gives each kind the move that measured best.
Cups use D-side-down (cup_05 the build's release, D-side-down-build), bowls D-ring (bowl_02 and bowl_03 D-ring-build) and plates
I4-wall20. bowl_04, plate_01 and plate_02 are reject rows (unreachable on this mount) with no Kit process. The upper chain ran in
the dag order. The lower chain ran bowl_02, plate_03, plate_05, plate_04: plate_05 at its goal blocks plate_03's descent (wrist
1, the forearm, the hand), and plate_03 has no dag edges.

- Bowl tilt cap (`BOWL_CARRY_TILT_MAX_DEG`, `bowl_tilt_cap_z`, in ring_drop's carry_ok and the reach gate): the held bowl at most
  max(20 deg, its tilt at the descent start) from straight down on every carry configuration. The descent start holds the bowl
  in its goal orientation (26.7 to 61.4 deg from straight down for s0), so a flat 20 deg would block every bowl. Among the
  passing descent starts, those whose carry legs are all straight joint moves go first (`straight_first`, the row's
  start_choice). A bowl with no path at 60 s retries once at 120 s (`RING_RETRY_RRT_S`). Each carry leg records the dish's
  largest tilt and in-hand rotation.
- Plate wall pinch (`grasp.plate_wall_hold`, `grasp.plate_wall_grasps`, `--grasp-family plate-wall`): the closing axis along the
  outer wall's normal 20 mm down from the rim edge (hotec_gen's profile), the approach down the wall, the pinch at the rim point
  highest at the plate's pose, roll 180. Presented in its goal orientation, the plate leaning on a backstop fell 60 to 95 deg in
  the spawn settle in 5 of 10 gate trials (partA_plate_wall20_backstop: held 5 of 10, every reached pinch held). On a plank at
  58 deg with a 1 mm lip under the bottom rim (presentation.json, the boxes in its fixtures list) it rested within 1 mm, and the
  gate partA_plate_wall20 held 10 of 10 (pivot at most 0.44 deg, drift at most 0.15 mm). The plank runs used the window's copy
  data/logs/robot_final/frigidaire_robot_episode_fixtures.py, which authors a fixture list under /World/Backstop; the episode in
  code/ takes one box.
- Result: 7 of 15 placed (cup_01, bowl_05, plate_03, plate_04, cup_04, cup_03, bowl_01). Per kind picked, carried, seated and at
  goal: bowls 4, 3, 2, 2 of 5; cups 4, 4, 3, 3 of 5; plates 3, 2, 2, 2 of 5. plate_03 and plate_04 are the first plates this hand
  picked from the counter and placed with the pads alone (2.0 mm and 4.0 mm, 4.6 and 0.4 deg). bowl_02 had no carry path under its 45.0 deg cap, plate_05's line up the tine axis
  met plate_03 as it landed, and bowl_03 and cup_05 by the build's release did not seat. Two Kit processes (cup_05 try a, cup_02)
  died 20 s into the scene build on "malloc(): corrupted top size". cup_05 try b ran from the state after bowl_05, beside the
  chain; cup_02 had no time left.
- Grip: the gripper drive (finger_joint) has effort limit 8 N m and stiffness 400. The pads carry bowls and cups at 1.4 to 3.5 N
  each and the plate's wall pinch at 76 to 99 N; the real 2F-85 grips at 20 to 235 N.
- Records: <load dir>/ (load.json, presentation.json, presentation_backstop.json, gate.json, both gate runs, dishes.jsonl,
  rows.jsonl, state/, runs/); media data/media/robot/samples_v1/<sample_id>__final/ (clip_, release_ and landed_<dish>,
  s0_grasp_place.mp4 from `frigidaire_run_video.py`, contact_sheet.png); window scripts and logs data/logs/robot_final/.

## The side ring hold: the bowls held at the foot ring from the side (2026-11-04)

Plan plans/2026-11-04-ring-side-hold.md, run note results/2026-11-04-ring-side-hold.md. The five bowls of s0 with the move
D-ring-side (`frigidaire_placing_loads.py --sample <sample> --ring-side`: `<sample_id>__ring_side/load.json`), each bowl alone
(no other dish, no state chain, its rack out and the other pushed in, the racks at the goal frames) on the posture_v1 mount;
Isaac ran bowl_04, the slot the hand-above hold cannot reach.

- Hold (`grasp.foot_side_grasps`, `--grasp-family foot-side`): the bowl mouth down at the counter spot, the approach horizontal
  toward its axis at the ring's height, the closing axis horizontal across the ring, so the pads close on the ring's outer
  face at two opposite points (F-out's contact and jaw window, 0.10 to 0.45 rad). TCP x lies along the bowl axis, and the wrist
  side "base" puts the wrist above the bowl. The pad's edge toward the body (11.0 mm along TCP x) sits FOOT_MARGIN_M above the
  ring's base plane (foot_height 5.57 mm; the mesh's outer face runs to 10.82 mm before the body flares), which puts the TCP
  6.43 mm beyond the end face; the pad tips at the F-out contact angle (0.236 rad, 25.8 mm beyond the TCP) reach the axis.
- Kit-free pick (`frigidaire_robot_reach.py --foot-candidates --foot-families side`): the cup side pinch's rules plus the open
  gripper never touching the bowl on the approach (it keeps 11.4 mm). With the wrist toward the base 9 of 12 yaws pass, each at
  the gate's ten +-5 mm spots; with the wrist toward the mouth none does (wrist 2 and the forearm against the Counter and the
  Cabinet).
- Kit-free release (`--sample-gate`, both wrist sides per bowl, merged): the descent line under D-side-down's final-descent rule,
  the starts ordered by the goal tilt's pitch about the hand's closing axis (`side_pitch`, `pitch_order`: the smallest first in
  0.5 deg bins, then straight carry legs, then rule 1). The roll spins fail at the descent start (the gripper and the held
  bowl against the rack, the forearm against the Cabinet), so every passing upper-rack start is pitch-dominant (44 to 61 deg of
  the 45.7 to 61.4 deg goal tilts). G2: bowl_04
  (spin 15, 150 mm, pitch 46.3) and bowl_01; bowl_05, bowl_03 and bowl_02 have descent starts but no carry path (RRT-Connect at
  60 s, the 120 s retry, and for bowl_03 and bowl_02 one search at 300 s).
- Isaac gate (run partA_bowl_foot_side, yaw 5): held 10 of 10, jaws 0.2260 to 0.2263 rad, pivot 0.43 to 1.24 deg, drift at most
  0.002 mm over the 5 s hold, pads only, 1.8 to 2.2 N per pad at the end of the hold.
- bowl_04: picked and carried within 0.6 mm of the hold, but the bowl turned 49.9 deg in the pads on the RRT carry leg, from the
  planned 48.4 deg tilt to 12.2 deg at the descent start, since a torque about the line through the two pads meets torsional
  friction only. On that leg the held bowl touched the Cabinet. It first touched the UpperRack 76.2 mm above its goal, slid
  15.4 mm in the pads while the force stayed under the 3.5 N trip, stopped at 61.8 mm, and settled mouth down on a tine
  18.3 mm, 20.4 mm and 53.1 deg from the goal: not seated.
- bowl_04 try b (the user's retry, `--ring-side-order straight`: straight carry legs first, then rule 1, the pitch breaking
  ties): spin 45 at 150 mm (pitch 47.2 deg), one straight leg of 32.2 s with 293.5 deg of wrist 3 travel. The bowl held to a
  hand-held tilt of 16.6 deg, then turned in the pads about the closing axis (14 deg one way, then 38 deg the other), reached
  the descent start 13.2 deg from straight down, stopped on the UpperRack 48.5 mm above its goal (4.55 N, 1.5 mm of slip)
  and settled mouth down on the tine tops 21.8 mm, 46.1 mm and 38.7 deg from the goal: not seated. In both tries the
  in-hand rotation was about TCP y, the line through the two pads.
- Records: <load dir>/ (load.json, foot_side_candidates.json, gate.json, blockers.json, carry300.json, bowl_supports.json,
  partA_bowl_foot_side/, gate_table.json, dishes.jsonl, rows.jsonl, runs/); media data/media/robot/samples_v1/<sample_id>__ring_side/ (clip_, release_ and
  landed_bowl_04, clip_gate_t00); window scripts and logs data/logs/robot_ring_side/.

## Grip force: the finger drive against the real hand's squeeze (2026-11-04)

Plan plans/2026-11-04-grip-force.md, run note results/2026-11-04-grip-force.md. The pads held the bowls and cups at 1.4 to 3.5 N
per pad while the real 2F-85 grips at 20 to 235 N, and the side ring hold's bowl_04 turned 38 and 50 deg in the pads on its carry.

- Probe (`frigidaire_grip_probe.py --inverted --foot-clamp --fix pads --bowl-iters 16`): the shipped bowl mouth down on a support,
  the TCP straight down on its axis at F-out's depth (the ring's end face 21.24 mm below the TCP, 4.57 mm of the ring engaged),
  the close, the support dropped 10 cm, a 3 s hang, then a 5 s hold over which the harness's contact reports give the steady
  per-pad normal force. Today's drive (finger_joint effort 8 N m, stiffness 400, damping 1) reproduced the gate: 1.72 N per pad,
  a 73 N close peak, jaws 0.226 rad.
- Mechanism, as the eight runs show it: finger_joint's PhysX drive holds its 0.8 rad target with an 8 N m limit, yet at the
  ring's 0.57 rad error PhysX reports a projected joint force of only 0.17 N m, the torque the two pads take at 48.5 mm/rad each.
  The drive's output fits finger_joint's 1.0 rad/s joint velocity limit: it pushes stiffness times error only while
  k * err / (c + k dt) stays under 1 rad/s, which at the ring means c >= 0.57 k (the passing runs delivered k * err in full; near
  its target, 0.013 rad away, stiffness 400 with damping 10 pushed 5 N m). Stiffness 4000 left the steady force at 1.73 N. At
  stiffness 400, effort 16 and damping 3 pushed the pads through the ring at the close and the bowl fell, and damping 10 slid
  the bowl 31.5 mm until the jaws closed on one side of the ring wall. The inner finger joints carry no PhysX
  drive (no DriveAPI on their prims, drive type 0), so their gains (effort 10, stiffness 10, damping 0.05) do nothing, and all five
  mimic couplings are hard (naturalFrequency 0; deviations at most 3e-5 rad at 1.7 N, 2 to 3 mrad at 22 to 33 N).
- Sweep (data/results/robot/grip_force/, sweep.md): effort / stiffness / damping 8/5/5 gave 27.5 N per pad, 8/6/4 33 N and 8/4/2.5
  22.5 N (min 22.3 N, no close peak above 22.7 N, jaws 0.233 rad, 0.1 mm slip): the chosen drive, the lowest squeeze over 20 N and
  the damping nearest today's.
- Gates at 8/4/2.5 (`--grip-drive gripper_stiffness=4,gripper_damping=2.5`): the side ring hold held 10 of 10 (jaws 0.2292 to
  0.2293 rad, pivot 0.25 to 0.54 deg, drift at most 0.099 mm, 22.3 to 23.2 N per pad); the cup side-down pinch held 10 of 10 (jaws
  0.2719 to 0.2725 rad, 20.4 N per pad); the plate wall pinch on the plank held 2 of 10: with c >= 0.57 k the free close has a
  time constant of at least 0.57 s, so the jaws were at 0.49 rad when the rig lifted after its 1 s close, short of the plate wall
  at 0.76 rad.
- Placements at 8/4/2.5, each bowl alone, `--ring-side-order straight`: bowl_04 try c (spin 45 at 150 mm, the start of try b) and
  bowl_01 (spin 60 at 150 mm) were picked, carried on one straight leg, seated and at goal, with at most 1.07 and 0.83 deg of
  in-hand rotation; they landed 2.5 mm, 0.3 mm and 2.2 deg and 12.8 mm, 2.6 mm and 11.2 deg from their goals. Try b at today's
  drive turned 39.7 deg on the same leg. bowl_02 stays a reject row: no Kit-free carry path with either hold.
- flags.py R2 keeps today's values (the coordinator's option A): the drive is shared and 8/4/2.5 breaks the plate wall pinch.
  The user's decision for the next round is a per-hold drive command, bowls and cups at 8/4/2.5 and plates at 8/400/1.
- Records: data/results/robot/grip_force/ (gf_00 to gf_07, sweep.json, sweep.md); the gates partA_bowl_foot_side_gf (ring_side load
  dir), partA_cup_side_down11_gf (cups_down), partA_plate_wall20_gf (final; the window's fixtures copy), each with
  gate_table_<run>.json beside it; the try c and bowl_01 rows in the ring_side rows.jsonl (with grip_drive); media clip_, release_
  and landed_bowl_04_c and _bowl_01 in data/media/robot/samples_v1/<sample_id>__ring_side/; window scripts and logs
  data/logs/robot_grip_force/.

## Fifteen of fifteen: every dish of s0 in one run (2026-11-04 to 2026-11-05)

Plan plans/2026-11-04-fifteen-of-fifteen.md (its Decisions), run notes results/2026-11-04-fifteen-r1.md,
results/2026-11-04-fifteen-r2.md and results/2026-11-05-fifteen-r3.md. The goal: one run grasps and places all 15 dishes of s0
from the counter in two rack chains, the placed dishes staying, each judged seated and at goal by the s0 rerun's rules, with one
stitched video. Each dish runs in its own Kit process on the posture_v2 mount with the basket out, picked at the counter spot
(0.35, -0.12) and held by the pads alone at its hold's drive: `frigidaire_robot_episode.py --place-goal <load dir>/load.json
--dishes <id> --state <the chain's previous after_* file> --no-basket --attach none --bowl-supports <load dir>/bowl_supports.json
--mount data/results/robot/mount/posture_v2.json --shortcut-s 10 --transit-rrt-s 60 --max-wall-seconds 3000`, the bowls with
`--tilt-cap off` from round 2 on. Round 3 placed 12 of 15 in one run, every plate among them; the s0 rerun had placed 7. The
three rounds ran in one box window (2026-11-05 03:36 to 07:47 UTC, 251.6 billed minutes with the stop).

| round | what ran | placed (seated and at goal) | bowls / cups / plates | short of placed |
|---|---|---|---|---|
| 1 | the four Isaac grasp gates at posture_v2 with the basket out, 10 of 10 each; the capoff measurement; nine dishes, each in the scene the full run gives it (the six no run had placed, cup_02's retry, plate_03 and plate_04 again) | 5 of 9: plate_02, plate_03, plate_01, plate_04, cup_02 (plate_02, plate_01 and cup_02 for the first time) | 0 of 2 / 1 of 2 / 4 of 5 | plate_05 perched 87.8 mm up on the next slot's tines; bowl_02 and bowl_03 reached the descent bound with no sensor trip, 22.4 and 47.9 mm above their goals, the jaws closed; cup_05 settled 26.4 mm and 22.5 deg off |
| 2 | all 15 in two chains (LD2), each dish from its predecessor's state file | 11 of 15 | 3 of 5 / 4 of 5 / 4 of 5 | bowl_02 (no trip, parked 22.4 mm up); bowl_03 (tripped 75.1 mm up, rests 5.3 mm up, its floor wire 5.68 mm off); cup_05 (26.7 mm, 22.2 deg off); plate_02 (bowl_02 parked: 9.4 mm up between its tines, at goal, not seated) |
| 3 | the cup gate at the 5 mm pinch (10 of 10), five single-dish checks (LD3), all 15 in two chains (LD3C) | 12 of 15 | 3 of 5 / 4 of 5 / 5 of 5 | bowl_02, bowl_03, cup_05 (the causes below) |

Decisions (the user's, from the plan; times EST):
- Hardware frozen: the side-mounted 2F-85 and the finger drives' limits stay as they are, and the pedestal may move. The goal
  may be re-packed for the arm's reach only as a last resort, for a dish no mount reaches at any order and spin; no round
  needed it.
- Mount: `frigidaire_mount_sweep.py` ran the full sample gate on all 15 dishes at 36 pedestal mounts (base x 0.36 to 0.60,
  y -0.70 to -1.00, z 0.45 to 0.75, facing posture_v1's work centre), the spawn spot and the plank moving with the base along
  the counter's front. A candidate keeps the 13 sites reached so far and adds plate_02 or plate_01. The winner is posture_v2:
  base (0.60, -0.70, 0.60), yaw 2.6855 rad (posture_v1's plus 7.0 deg), spot (0.35, -0.12), 0.15 m from posture_v1, adding
  plate_02. The grid extended to x 0.75 and 0.90 (60 mounts, posture_v3.json) found the same base. With the basket in,
  plate_01 is G0 at every eligible mount (the wrists and the gripper against the basket).
- Drive: a per-hold command, as the real hand's force and speed are. flags.GRIP_DRIVE_BY_FAMILY closes the foot-out,
  foot-side, side and side-down holds at effort / stiffness / damping 8 / 4 / 2.5 (22 N per pad) and the plate-wall and edge
  holds at 8 / 400 / 1 (67 to 95 N at the plate wall in the gate); `--grip-drive` still overrides it.
- Tilt cap lifted: bowl_02's RRT carry with the axial hold at 22 N (`--tilt-cap off`, run fifteen_bowl_02_capoff) tilted the
  bowl 46.41 deg from straight down, 1.41 deg past its 45.0 deg cap, and turned it 0.99 deg in the pads (0.09 mm drift, no
  scene contact), under the 5 deg rule. Every bowl line of rounds 2 and 3 carries `--tilt-cap off`.
- Guarded insertion at the build sites (bowl_02, bowl_03, cup_05): D-ring-build and D-side-down-build became D-ring-insert
  and D-side-insert (the side ring hold at a build site: D-ring-side-insert). The rule allows no free fall: the dish is lowered
  along the site's insertion axis until the flange sensor trips, then the jaws open, the hand retreats and the dish settles.
  The retreat is the first clear one of RETREATS under the 2 mm rule (retreat_check; Kit-free insert_release_check): a, 20 mm
  back along -TCP z, then 100 mm up; b, 100 mm up the insertion axis, then the back-off; c, the rise alone. Round 1 bounded the
  descent 30 mm past the predicted stop and kept the jaws closed when no trip came. Since round 3 the descent ends at the
  predicted sensor stop (INSERT_STOP_RULE): a trip above it ends the descent there, and reaching it without a trip is a
  position stop (stop.kind "predicted"), followed by the pre-open check, the open, the retreat and the settle.
- Order: Kit-free, `frigidaire_access_order.py --mount data/results/robot/mount/posture_v2.json --dag soft --no-basket`. Per
  rack chain it finds a dag-valid linearization in which every dish passes the sample gate with the earlier dishes of its
  chain at their predicted landed poses (the s0 rerun's state files, then the ring-side round's, else the goal), each plate's
  pinch spin chosen at its goal. With `--dag soft` a plate's access edge may be reversed for the arm's chain when the plate
  passes with the later dish in the scene and its seat at its goal; a support edge never is, and the sample's dag stays as it
  is. The seat-at-goal rule (D3, SEAT_AT_GOAL_M, seat_at_goal) counts a seat only within 20 mm of the goal along the tine axis
  with rack wires as its only supports, since the gate had passed plate_02 perched on plate_05 as G2. The order reversed no
  edge (order.json): upper chain cup_01, bowl_05, bowl_04, cup_05, bowl_03, cup_04, cup_03, bowl_01, cup_02; lower chain
  bowl_02, plate_02, plate_05, plate_03, plate_04, plate_01, with plate_05 and plate_03 on the tine-guided drop.
  `frigidaire_placing_loads.py --sample <s0> --fifteen` writes the load from it.
- Basket out for the robot rounds (19:20 EST; the real basket lifts out): `--no-basket` in the episode, the reach gate, the
  mount sweep and the order tool. With the basket popped from the FCL world the gate places plate_01 at lower_rear_05 at spins
  0 and plus or minus 15 deg, seated on its goal wires with the whole lower chain in the scene (plate_04 before plate_01). s0's
  goal poses, the benchmark samples and S_ref keep the basket, and every robot row records basket false. Every round-1 Kit
  process logged `[basket] removed /World/InitialStateDishwasher/SilverwareBasket (385 colliders and 288 visual meshes
  deactivated ...)`.
- Plate drop: this hand serves the front pair plate_02 / plate_05 (slots 07 and 04, planes 80.6 mm apart, a 40.6 mm free gap)
  at the seat in neither order. With plate_02 standing the closed hand sits 21 to 34 mm inside plate_02 at plate_05's seat at
  every spin (it needs about 64 mm of gap), and with plate_05 standing plate_02's rim lands on plate_05 193 mm up. The user
  chose the tine-guided release (20:05 EST; D2, move I4-wall20-drop): the plate lowered along the tine axis to the lowest
  height where the arm keeps its 2 mm (65 mm above the seat for plate_05), provided its lowest rim point lies at least 20 mm
  below its slot's tine tips there (DROP_GUIDED_MIN_M; 37.3 mm for plate_05), then the open and the rise. Round 3 added three
  parts from round 1's ticks (00:05 EST): the descent line biased 5 mm toward the lean side (the load row's descent_bias_mm,
  default 1 mm), a rest wait after the open (drop_rest: the centroid under 2 mm/s on 12 consecutive ticks, 60 to 300 ticks)
  and a 20 mm back-off along the hand's axis before the rise (drop_retreat, else rise-only). In every round-3 drop the retreat
  check ruled the back-off out at the rested plate (in the single checks the open jaws touched it), and the hand rose without it.
- Insert sites, cup_05 (22:20 and 23:02 EST): the gate passes it at the build's stop, where the open hand clears the UpperRack
  by 2.3 mm, and at the predicted sensor stop 10 mm lower the fully open hand overlaps the UpperRack under every retreat. The
  partial open (PARTIAL_OPEN_CLEAR_M) opens the jaws only to the pad gap that held the dish plus 5 mm a side, tried where the
  full open fails the same 2 mm rule, with the gate and the episode judging at the same predicted stop; it does not free
  cup_05. The guided release (INSERT_DROP_RULE) carries the plate drop over: the descent stops at the lowest height where the
  open hand (full, then partial) and one retreat keep 2 mm, provided the dish's lowest point lies at least 20 mm below the wire
  tips in its goal footprint. Rounds 1 and 2 let cup_05 go 10.1 mm above its goal this way. bowl_02 and bowl_03 keep the
  sensor-trip release.
- cup_05's pinch (00:05 EST): the side pinch 5 mm from the base in place of 11 mm (the load row's side_below_top_m 0.0999 and
  hold_gate_run partA_cup_side_down5), so the palm sits 6 mm higher and the cup is released in contact. Its gate held 10 of 10,
  and in round 3 the sensor tripped 5.2 mm above the goal (the Kit-free prediction 4.8 mm).

The Isaac grasp gates in the load dir (`--n 10 --seed 0 --no-basket --mount data/results/robot/mount/posture_v2.json`, at the
spot (0.35, -0.12), each at its hold's drive; round 1, and the 5 mm cup gate in round 3):

| gate run | hold | drive | held | jaw (rad) | pivot (deg) | drift over 5 s (mm) | pad N (weaker pad) |
|---|---|---|---|---|---|---|---|
| partA_bowl_foot_out | axial ring hold (F-out) | 8 / 4 / 2.5 | 10 of 10 | 0.2277 to 0.2284 | 0.01 to 0.50 | 0.001 to 0.073 | 22.5 to 22.9 |
| partA_bowl_foot_side | side ring hold | 8 / 4 / 2.5 | 10 of 10 | 0.2292 | 0.40 to 0.50 | 0.087 to 0.100 | 22.5 to 23.2 |
| partA_cup_side_down11 | cup side-down pinch 11 mm from the base | 8 / 4 / 2.5 | 10 of 10 | 0.2719 to 0.2725 | 0.00 to 0.03 | 0.001 | 20.4 |
| partA_plate_wall20 | wall pinch on the plank | 8 / 400 / 1 | 10 of 10, 7 counted | 0.7596 to 0.7693 | 0.00 to 3.37 | 0.002 to 2.31 | 66.9 to 94.8 |
| partA_cup_side_down5 (round 3, LD3) | cup side-down pinch 5 mm from the base | 8 / 4 / 2.5 | 10 of 10 | 0.2789 to 0.2796 | 0.02 to 0.04 | 0.001 | 20.02 to 20.05 |

The plate gate counted 7 trials: the harness flagged pad_force (a pad above 235 N at some tick) in trials 5 and 9 and
carried_contact in trials 7 and 9. The round counted a gate as passed on its held trials, so every hold passed and no dish
became a reject row.

The three dishes short of placed after round 3 (the chain; the single checks stopped on the same causes):

| dish | slot, move | release | after the open | landing lateral / dz / tilt (mm, mm, deg) | cause |
|---|---|---|---|---|---|
| bowl_02 | lower_frontright, D-ring-insert | position stop at the predicted stop, 52.0 mm planned and 52.8 mm actual (no trip; the single check peaked at 1.52 N) | slid 53.3 mm down its tines, turning 27.0 deg | 2.3 / 0.2 / 14.1 | at goal, not seated: tilted 14.1 deg, no bottom wire within 2 mm |
| bowl_03 | upper_bowl_03, D-ring-insert | position stop, 80.0 mm planned and 81.5 mm actual (no contact in the single check) | slid 83.2 mm, turning 5.6 deg | 9.6 / 2.7 / 5.2 | at goal, not seated: its floor wire 2.95 mm off (2 mm needed) |
| cup_05 | glass_left_1, D-side-insert, the 5 mm pinch | sensor stop, 4.03 N at 5.2 mm above the goal | pad_L on the cup for 99 ticks (peak 0.22 N) | 26.4 / 2.3 / 22.6 | not at goal: toppled 22.6 deg onto pad_L and settled mouth down; the 11 mm pinch of rounds 1 and 2 ended the same way (26.3 to 26.7 mm, 22.2 to 22.7 deg) |

Round 3's chain (LD3C), each dish in its own Kit process from its predecessor's state file, the upper chain on cores 2/6 and
the lower on 0/4:

| # | chain | dish | slot | move | placed | landing lateral / dz / tilt (mm, mm, deg) |
|---|---|---|---|---|---|---|
| 1 | upper | cup_01 | glass_right_2 | D-side-down | yes | 1.8 / 0.2 / 1.1 |
| 2 | upper | bowl_05 | upper_bowl_01 | D-ring | yes | 0.9 / 0.3 / 0.9 |
| 3 | upper | bowl_04 | upper_bowl_11 | D-ring-side | yes | 1.8 / 0.2 / 1.4 |
| 4 | upper | cup_05 | glass_left_1 | D-side-insert | no: off goal | 26.4 / 2.3 / 22.6 |
| 5 | upper | bowl_03 | upper_bowl_03 | D-ring-insert | no: at goal, not seated | 9.6 / 2.7 / 5.2 |
| 6 | upper | cup_04 | glass_left_2 | D-side-down | yes | 0.8 / 0.1 / 2.2 |
| 7 | upper | cup_03 | glass_right_3 | D-side-down | yes | 2.0 / 0.2 / 0.8 |
| 8 | upper | bowl_01 | upper_bowl_07 | D-ring-side | yes | 12.8 / 2.6 / 11.2 |
| 9 | upper | cup_02 | glass_left_3 | D-side-down | yes | 1.7 / 0.2 / 0.3 |
| 10 | lower | bowl_02 | lower_frontright | D-ring-insert | no: at goal, not seated | 2.3 / 0.2 / 14.1 |
| 11 | lower | plate_02 | lower_front_07 | I4-wall20 | yes | 0.2 / 0.0 / 0.0 |
| 12 | lower | plate_05 | lower_front_04 | I4-wall20-drop | yes | 0.2 / 0.0 / 0.0 |
| 13 | lower | plate_03 | lower_front_00 | I4-wall20-drop | yes | 0.1 / 0.0 / 0.0 |
| 14 | lower | plate_04 | lower_rear_00 | I4-wall20 | yes | 2.0 / 0.5 / 4.4 |
| 15 | lower | plate_01 | lower_rear_05 | I4-wall20 | yes | 0.2 / 0.0 / 0.0 |

bowl_02 was parked after its row, so plate_02 ran without it in rounds 2 and 3: 9.4 mm up between its tines in round 2, seated
by its seat release (77.9 mm) in round 3. With plate_02 standing, plate_05's drop planned 60 mm (the gripper against plate_02
at 55 mm), released 64.4 mm up by position, rested 30.9 mm above its goal after 40 ticks and slid down. plate_02 and plate_05
are both seated in this chain, the pair the plan had found this hand could not serve at the seat in either order. In the single
check (LD3, the composed scene) the same drop planned 65 mm and plate_05 perched 98.2 mm up on the col 6 tine tips. The 20
replays of round 3 are at 3 of 3 PASS, and no round-3 Kit log has a Traceback, `free(): invalid pointer` or
`malloc(): corrupted top size`.

- Load dirs, each `data/results/robot/samples_v1/<s0>` plus a suffix, with
  <s0> = frigidaire_fdpc4221as__hotec_wheatstraw__plate5-bowl5-cup5_a3__s0: `__fifteen` (round 1: load.json, order.json,
  gate.json, gate_hardware.json, presentation.json, bowl_supports.json, the gate runs partA_bowl_foot_out, partA_bowl_foot_side,
  partA_cup_side_down11 and partA_plate_wall20, state_round1/ with the composed scenes, dishes.jsonl, rows.jsonl, state/,
  runs/); `__fifteen_r2` (round 2's chain, LD2, round 1's four partA_* dirs copied in; its first start, stopped 11 min in for
  the cup_05 remedy, is `__fifteen_r2__prestart_0413`); `__fifteen_r3` (round 3's single checks, LD3, adding
  partA_cup_side_down5 and gate_hardware_r3.json); `__fifteen_r3c` (round 3's chain, LD3C: a copy of LD3 with "load" and
  "media_dir" renamed, the four round-1 partA_* dirs linked and partA_cup_side_down5 copied in).
- hold_gate_run: a load row's side_below_top_m and hold_gate_run (the SIDE_DOWN_MOVES D-side-down, D-side-insert and
  D-side-down-build) set the cup's pinch height and the gate run whose closed jaws the descent check and the stop prediction
  use; the row's hold_source records them with load_dir_gate_present.
- Media: `data/media/robot/samples_v1/<s0>__fifteen{,_r2,_r3,_r3c}/` (clip_, release_ and landed_<dish>). The videos are
  `<s0>__fifteen_r2/s0_fifteen.mp4` (732 s, 40.2 MB) and `<s0>__fifteen_r3c/s0_fifteen_r3.mp4` (735.3 s, 40.1 MB), each 15
  title cards and 15 replay clips, h264 yuv420p 1280 x 536 at 10 fps, with contact_sheet.png, made on the Mac by the window
  copies of frigidaire_run_video.py (`data/logs/robot_fifteen_r2/run_video_fifteen.py`, `data/logs/robot_fifteen_r3/run_video_r3.py`).
- Mount files: `data/results/robot/mount/posture_v2.json` (with posture_v2.rows.jsonl and posture_v2_presentation.json) and
  posture_v3.json (the 60-mount grid). Window scripts and logs: `data/logs/robot_fifteen_r1/` (r1.sh, lines_r1.txt,
  capdecision.txt), `data/logs/robot_fifteen_r2/` (r2.sh), `data/logs/robot_fifteen_r3/` (r3.sh, lines_r3.txt).
- Flags (rows in code/execution/frigidaire/README.md): the episode's `--basket in|out` / `--no-basket` (remove_basket in the
  Kit scene before sim.reset, drop_basket_fcl in the arm's FCL world; default the load's "basket" field), `--tilt-cap on|off`
  and `--move <name>` (e.g. I4-wall20-drop; a load row's I4-wall20 with release.mode drop runs as I4-wall20-drop); the reach
  gate's `--mount` and `--no-basket` (use_basket, strip_basket) and its seat_at_goal, tine_guided, insert_release_check,
  open_release_check (variant, others) and cup_down_hold(below_top); `frigidaire_mount_sweep.py`;
  `frigidaire_access_order.py --dag soft|hard --no-basket`; `frigidaire_placing_loads.py --sample <s0> --fifteen` and the new
  `--rows` columns.

Landmines found in these rounds:
1. Wrist 3 lags 2.3 deg under the plate drive 8 / 400 / 1. The held plate then sits 3.4 mm toward the counter, and in round 1
   plate_05 met its own slot's tine tips about 80 mm up (the early trip). Kit-free, the 5 mm descent bias keeps the lagged
   plate off every tine above the 65 mm release.
2. A pad still touching a released plate drags it up on a rise-only retreat. In round 1 plate_05 slid down its slot to 35 mm
   after the open, the right pad lifted it 67 mm on the rise, and it stepped onto the next column's tines (87.8 mm up). Round 3
   added the rest wait and the back-off before the rise for this.
3. The ring-insert bowls slide on their tines at 2 to 2.7 N and stay under the 3.5 N on 2 ticks trip: bowl_02 never tripped
   in three rounds, and bowl_03's single trip (round 2, 75.1 mm up) let it go above its wires. Since round 3 these descents are
   bounded by position at the predicted stop.
4. The gripper palm does not move with the jaws. At cup_05's sensor stop the palm sits 1.0 mm from the UpperRack at every jaw
   width and the inner knuckle within 1.2 mm (11 mm higher the open hand clears by 2.3 mm), so the partial open cannot fix a
   palm contact.
5. The episode reads the descent check's closed-jaw hand (and the plates' in-hand pose) from the load dir's Isaac gate runs:
   partA_bowl_foot_out, partA_bowl_foot_side, partA_cup_side_down11 or the row's hold_gate_run, partA_plate_wall20. A load dir
   without them falls back silently to the older gates (placing_v1's F-out gate, the cups round's gate, PLATE_GATE);
   hold_source.load_dir_gate_present in the row says which. Copy or link the partA_* dirs into every new load dir, as LD2 and
   LD3C did.
6. samples_v1 on the box is root-owned, so an rsync from the Mac as ubuntu cannot create a new load dir there. Create it
   inside the container and chown it to 1000:1000 before the push.
