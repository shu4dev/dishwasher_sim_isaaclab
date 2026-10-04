# REPORT: plans/2026-09-29-easy-s0.md -- Phase 0 (audit and harness)

Generated 2026-09-30T14:11:45+00:00 by `frigidaire/scripts/evaluation/frigidaire_robot_report.py` from the trial logs, run records and the conventions record; nothing in the measured tables is typed by hand. Companion audit with path:line evidence: `plans/2026-09-29-easy-s0-map.md`.

Time columns: `sim s` = simulated seconds (physics ticks x 1/120 s), `wall s` = host wall-clock seconds; they are never mixed. `attempts` counts grasp attempts, `configs` counts distinct start configurations (`config_id`).

## 1. Phase status

| phase | status |
|---|---|
| 0 audit and harness | executed this session (this report); accept criteria below |
| 1 physics and model fidelity | not started (stop-and-report point after 1e) |
| 2 grasp library | not started |
| 3 placement at the real goals | not started |
| 4 reach, access, loading order | not started |
| 5 easy_s0 end to end | not started |

## 2. Required Phase 0 tests (FAIL (not run) when nothing ran)

| required test | status / evidence |
|---|---|
| 0.1 audit table | PASS: section 8 (D20 wording) |
| D13 git state of the robot stack (read from git) | HEAD `c1f606f 2026-09-30 add robot stack`; 23 robot files tracked; modified since HEAD: ['frigidaire/docs/robot.md', 'frigidaire/scripts/evaluation/frigidaire_robot_report.py', 'frigidaire/scripts/experiment/frigidaire_robot_episode.py', 'frigidaire/scripts/setup/mirror_robot_usd.sh', 'frigidaire/src/dishsim_frigidaire/robot/harness.py']; untracked: ['frigidaire/scripts/setup/robot_asset_manifest.py', 'frigidaire/tests/test_robot_harness.py'] (the executor never commits; the user commits from the summary) |
| 0.2 flags: 3-bowl test with the OLD settings reproduces the pre-Phase-0 PASS | PASS: legacy profile run `p0v2_upright3_legacy` reproduces the PASS verdict (3/3 physically ok, all racked; record `results/robot/runs/p0v2_upright3_legacy/robot_upright3.json`); the numbers are not byte-identical: jaw angles within 11.5 mrad, final poses within 16.4 mm of `results/robot/episodes/robot_upright3.json`; the pre-harness reproduction `results/robot/runs/p0_prerepro_upright3/robot_upright3.json` IS byte-identical, so the harness changed the timeline; A/B `p0v2_upright3_legacy_nosleep` (sleep authoring off, ANALYSIS): final poses within 0.00e+00 mm of the PASS -> the plan's never-sleep rule (D10) is the cause |
| 0.2 flags: 3-bowl test with the HEADLINE settings | PASS: ran (`p0v2_upright3_headline`, result FAIL, expected to fail the at-goal tolerance) |
| 0.2 flags: 3-bowl test, headline settings, diagnostic (invariants recorded, D18) | PASS: ran (`p0v2_upright3_headline_diag`, result FAIL) |
| 0.3 sleeping disabled | PASS: sleep threshold 0.0 authored on 3 dishes, 16 robot links, 2 articulation roots (`artifacts/p0_upright3_legacy/t00_nominal/trial.jsonl:L1`) |
| 0.3 settle routine (< 1 mm/s, 0.01 rad/s for 1 s, cap 5 s) -- D3: diagnostic column | PASS: 22 diagnostic settle rows over the runs (first: `...:L1140`) |
| 0.3 contact logging (pads, fingers, palm, arm links, bowls, racks, basket, counter, pedestal) | PASS: 305 contact_begin rows over the runs (PhysX contact reports, touching pairs only) |
| 0.4 invariant `arm_or_palm_contact` | implemented (`harness.Harness._check`); never fired in these runs |
| 0.4 invariant `carried_contact` | implemented (`harness.Harness._check`); fired in 36 place(s), e.g. `p0_upright3_legacy:t00_nominal:L556` |
| 0.4 invariant `teleport_after_reset` | implemented (`harness.Harness._check`); never fired in these runs |
| 0.4 invariant `sleeping_body` | implemented (`harness.Harness._check`); fired in 8 place(s), e.g. `p0v2_upright3_legacy_nosleep:t00_nominal:L67` |
| 0.4 invariant `nan_frame` | implemented (`harness.Harness._check`); never fired in these runs |
| 0.4 invariant `joint_limit` | implemented (`harness.Harness._check`); never fired in these runs |
| 0.4 invariant `pad_force` | implemented (`harness.Harness._check`); never fired in these runs |
| 0.5 convention tests on the live asset (approach sign, jaw axis, pad normals, finger signs) | PASS: conventions run `p0_conventions` (`artifacts/p0_conventions/conventions.json`), fixture `frigidaire/tests/fixtures/robot/conventions.json`; pytest below |
| 0.5 D7: mimic couplings during a loaded close (H2 runtime test) | PASS: all coupled joints track finger_joint with \|ratio\| = 0.9995..0.9999 at finger_joint 0.606 rad on the bowl rim |
| 0.5 D12: arm-collider press test | PASS: robot A: gripper de-instanced, arm colliders instanced (the episode before D12) -> plate caught at z 0.635 m on ['RobotA:forearm_link', 'RobotA:upper_arm_link', 'RobotA:wrist_1_link'] (peak 135 N); robot B: whole robot de-instanced (D12 candidate) -> plate caught at z 0.6351 m on ['RobotB:forearm_link', 'RobotB:upper_arm_link', 'RobotB:wrist_1_link'] (peak 135 N) |
| 0.5 Kit-free pytest (robot tests) | PASS: 23/23 passed in test_robot_conventions.py, test_robot_flags.py, test_robot_harness.py, test_robot_kin.py (`artifacts/p0_report/pytest_robot.xml`) |
| 0.6 JSONL trial logs + generated report | PASS: this file and `artifacts/<run-id>/index.html` are generated from the logs |
| 0.6 one full trial's video, key frames and plots present and linked | PASS: 8 trial(s), e.g. `artifacts/p0_upright3_legacy/t00_nominal/video.mp4`, 39 key frames, 4 plots |
| accept: 3-bowl test through the harness | PASS: `p0v2_upright3_legacy` |
| accept: easy_s0 through the harness (expected to fail) | PASS: ran, result FAIL (`p0v2_easy_s0_headline`) |
| accept: per-bowl failure causes for easy_s0 (diagnostic continue, D18) | PASS: `p0v2_easy_s0_headline_diag`, section 5 |

## 3. Runs

| run | instance | profile | result | moves ok | attempts | configs | benchmark success | final hold | invariants (distinct) | sim s / wall s (moves) |
|---|---|---|---|---|---|---|---|---|---|---|
| p0_prerepro_upright3 | robot_upright3 | (pre-harness code path, old record format) | **PASS** | 3/3 | 3 | - | None | None | - | - / 460.8 |
| p0_upright3_legacy | robot_upright3 | legacy_upright3 | **PASS** | 0/3 | 3 | 1 | None | None | {"carried_contact": 13} | 147.5 / 638.6 |
| p0_upright3_headline | robot_upright3 | headline | **FAIL** | 0/1 | 1 | 1 | False | True | {"carried_contact": 2} | 24.417 / 105.9 |
| p0v2_upright3_legacy | robot_upright3 | legacy_upright3 | **PASS** | 0/3 | 3 | 1 | None | None | {"carried_contact": 6} | 147.5 / 656.0 |
| p0v2_upright3_legacy_nosleep | robot_upright3 | legacy_upright3 | **PASS** | 0/3 | 3 | 1 | None | None | {"sleeping_body": 8, "carried_contact": 6} | 148.192 / 565.5 |
| p0v2_upright3_headline | robot_upright3 | headline | **FAIL** | 0/1 | 1 | 1 | False | True | {"carried_contact": 1} | 24.425 / 115.1 |
| p0v2_upright3_headline_diag | robot_upright3 | headline +diagnostic | **FAIL** | 0/3 | 6 | 1 | False | True | {"carried_contact": 7} | 147.283 / 707.6 |
| p0v2_easy_s0_headline | easy_s0 | headline | **FAIL** | 0/1 | 2 | 1 | False | True | {} | 9.75 / 65.2 |
| p0v2_easy_s0_headline_diag | easy_s0 | headline +diagnostic | **FAIL** | 0/9 | 14 | 1 | False | True | {"carried_contact": 1} | 169.333 / 1286.4 |

`result`: legacy profile = the old rule (every move physically ok, all racked; invariants recorded only); headline profile = benchmark success (every dish within the at-goal tolerance AND end check accepted) AND final hold passed AND no invariant AND no relaxation flag. Runs with `--test-case` or `--diagnostic-continue` are labelled non-headline by construction.

### 3.1 Regression: the 3-bowl test with the old settings

| dish | PASS record ok | legacy run ok (physical) | jaw rad (PASS) | jaw rad (legacy run) |
|---|---|---|---|---|
| bowl_01 | True | True | [0.6018974184989929] | [0.6072564125061035] |
| bowl_02 | True | True | [0.6026384234428406] | [0.6009778380393982] |
| bowl_03 | True | True | [0.6066892147064209] | [0.5951396822929382] |

Final poses: max difference 1.64e+01 mm vs `results/robot/episodes/robot_upright3.json` (the pre-Phase-0 PASS, backed up at `results/robot/backup_pre_phase0_20260929/`). Pre-harness reproduction with the same code path: `results/robot/runs/p0_prerepro_upright3/robot_upright3.json` (PASS 3/3, 460.8 s wall vs 472.1 s).

Harness verdict on the same run: invariants {"carried_contact": 6} (distinct first occurrences; ticks in violation {"carried_contact": 8447}). The old rule PASSES while the plan's D4 invariant fails it: see section 6.

## 4. Per bowl (every run)

| run | bowl | start | goal rack | moves | reached | distance (lat mm, dz mm, tilt deg) | racked in | cause | class | invariants | log |
|---|---|---|---|---|---|---|---|---|---|---|---|
| p0_upright3_legacy | bowl_01 | Counter | LowerRack | [1] | True | None | LowerRack | attempt 1: invariant x1 | invariant | carried_contact | `artifacts/p0_upright3_legacy/t00_nominal/trial.jsonl:L1141` `artifacts/p0_upright3_legacy/t00_nominal/trial.jsonl:L556` |
| p0_upright3_legacy | bowl_02 | Counter | LowerRack | [2] | True | None | LowerRack | attempt 1: invariant x1 | invariant | carried_contact | `artifacts/p0_upright3_legacy/t00_nominal/trial.jsonl:L2041` `artifacts/p0_upright3_legacy/t00_nominal/trial.jsonl:L1412` |
| p0_upright3_legacy | bowl_03 | Counter | UpperRack | [3] | True | None | UpperRack | attempt 1: invariant x1 | invariant | carried_contact | `artifacts/p0_upright3_legacy/t00_nominal/trial.jsonl:L3119` `artifacts/p0_upright3_legacy/t00_nominal/trial.jsonl:L2597` |
| p0_upright3_headline | bowl_01 | Counter | LowerRack | [1] | False | [396.0, 569.0, 0.0] | None | attempt 1: invariant x1, other x1 | invariant | carried_contact | `artifacts/p0_upright3_headline/t00_nominal/trial.jsonl:L557` `artifacts/p0_upright3_headline/t00_nominal/trial.jsonl:L556` |
| p0_upright3_headline | bowl_02 | Counter | LowerRack | [] | False | [607.6, 569.0, 0.0] | None | not attempted: move 1 (goal bowl_01): auto-fail invariant carried_contact: {'dish': 'bowl_01', 'partner': 'right_inner_knuckle', 'category': 'finger', 'allowed': ['pad', 'counter', 'rack', 'Counter'], 'force_N': 0.0, 'co | not attempted |  |  |
| p0_upright3_headline | bowl_03 | Counter | UpperRack | [] | False | [567.1, 194.0, 0.0] | None | not attempted: move 1 (goal bowl_01): auto-fail invariant carried_contact: {'dish': 'bowl_01', 'partner': 'right_inner_knuckle', 'category': 'finger', 'allowed': ['pad', 'counter', 'rack', 'Counter'], 'force_N': 0.0, 'co | not attempted |  |  |
| p0v2_upright3_legacy | bowl_01 | Counter | LowerRack | [1] | True | None | LowerRack | attempt 1: invariant x1 | invariant | carried_contact | `artifacts/p0v2_upright3_legacy/t00_nominal/trial.jsonl:L1146` `artifacts/p0v2_upright3_legacy/t00_nominal/trial.jsonl:L570` |
| p0v2_upright3_legacy | bowl_02 | Counter | LowerRack | [2] | True | None | LowerRack | attempt 1: invariant x1 | invariant | carried_contact | `artifacts/p0v2_upright3_legacy/t00_nominal/trial.jsonl:L2034` `artifacts/p0v2_upright3_legacy/t00_nominal/trial.jsonl:L1417` |
| p0v2_upright3_legacy | bowl_03 | Counter | UpperRack | [3] | True | None | UpperRack | attempt 1: invariant x1 | invariant | carried_contact | `artifacts/p0v2_upright3_legacy/t00_nominal/trial.jsonl:L3106` `artifacts/p0v2_upright3_legacy/t00_nominal/trial.jsonl:L2592` |
| p0v2_upright3_legacy_nosleep | bowl_01 | Counter | LowerRack | [1] | True | None | LowerRack | attempt 1: invariant x1 | invariant | carried_contact, sleeping_body | `artifacts/p0v2_upright3_legacy_nosleep/t00_nominal/trial.jsonl:L1153` `artifacts/p0v2_upright3_legacy_nosleep/t00_nominal/trial.jsonl:L247` |
| p0v2_upright3_legacy_nosleep | bowl_02 | Counter | LowerRack | [2] | True | None | LowerRack | attempt 1: invariant x1 | invariant | carried_contact, sleeping_body | `artifacts/p0v2_upright3_legacy_nosleep/t00_nominal/trial.jsonl:L2046` `artifacts/p0v2_upright3_legacy_nosleep/t00_nominal/trial.jsonl:L1154` |
| p0v2_upright3_legacy_nosleep | bowl_03 | Counter | UpperRack | [3] | True | None | UpperRack | attempt 1: invariant x1 | invariant | carried_contact, sleeping_body | `artifacts/p0v2_upright3_legacy_nosleep/t00_nominal/trial.jsonl:L3158` `artifacts/p0v2_upright3_legacy_nosleep/t00_nominal/trial.jsonl:L2470` |
| p0v2_upright3_headline | bowl_01 | Counter | LowerRack | [1] | False | [397.1, 569.0, 0.0] | None | attempt 1: invariant x1, other x1 | invariant | carried_contact | `artifacts/p0v2_upright3_headline/t00_nominal/trial.jsonl:L571` `artifacts/p0v2_upright3_headline/t00_nominal/trial.jsonl:L570` |
| p0v2_upright3_headline | bowl_02 | Counter | LowerRack | [] | False | [607.6, 569.0, 0.0] | None | not attempted: trial ended early (move 1 (goal bowl_01): auto-fail invariant carried_contact) | not attempted |  |  |
| p0v2_upright3_headline | bowl_03 | Counter | UpperRack | [] | False | [567.1, 194.0, 0.0] | None | not attempted: trial ended early (move 1 (goal bowl_01): auto-fail invariant carried_contact) | not attempted |  |  |
| p0v2_upright3_headline_diag | bowl_01 | Counter | LowerRack | [1] | False | [353.5, 569.0, 0.0] | None | attempt 1: invariant x1, transport blocked (lag limit) x1; attempt 2: transport blocked (lag limit) x1 | transport blocked (lag limit) | carried_contact | `artifacts/p0v2_upright3_headline_diag/t00_nominal/trial.jsonl:L1251` `artifacts/p0v2_upright3_headline_diag/t00_nominal/trial.jsonl:L570` |
| p0v2_upright3_headline_diag | bowl_02 | Counter | LowerRack | [2] | False | [544.9, 569.0, 0.0] | None | attempt 1: invariant x1, transport blocked (lag limit) x1; attempt 2: transport blocked (lag limit) x1 | transport blocked (lag limit) | carried_contact | `artifacts/p0v2_upright3_headline_diag/t00_nominal/trial.jsonl:L2087` `artifacts/p0v2_upright3_headline_diag/t00_nominal/trial.jsonl:L1396` |
| p0v2_upright3_headline_diag | bowl_03 | Counter | UpperRack | [3] | False | [533.4, 194.0, 0.0] | None | attempt 1: invariant x1, transport blocked (lag limit) x1; attempt 2: transport blocked (lag limit) x1 | transport blocked (lag limit) | carried_contact | `artifacts/p0v2_upright3_headline_diag/t00_nominal/trial.jsonl:L3165` `artifacts/p0v2_upright3_headline_diag/t00_nominal/trial.jsonl:L2483` |
| p0v2_easy_s0_headline | bowl_01 | LowerRack | UpperRack | [1] | False | [274.9, 286.5, 74.0] | LowerRack | attempt 1: unreachable x1; attempt 2: unreachable x1 | unreachable |  | `artifacts/p0v2_easy_s0_headline/t00_nominal/trial.jsonl:L307` |
| p0v2_easy_s0_headline | bowl_02 | Counter | LowerRack | [] | False | [355.7, 653.3, 91.4] | None | not attempted: trial ended early (move 1 (park bowl_01) failed twice) | not attempted |  |  |
| p0v2_easy_s0_headline | bowl_03 | Counter | LowerRack | [] | False | [749.0, 655.0, 40.1] | None | not attempted: trial ended early (move 1 (park bowl_01) failed twice) | not attempted |  |  |
| p0v2_easy_s0_headline | bowl_04 | Counter | UpperRack | [] | False | [531.5, 343.5, 45.3] | None | not attempted: trial ended early (move 1 (park bowl_01) failed twice) | not attempted |  |  |
| p0v2_easy_s0_headline | bowl_05 | Counter | LowerRack | [] | False | [433.1, 664.4, 61.3] | None | not attempted: trial ended early (move 1 (park bowl_01) failed twice) | not attempted |  |  |
| p0v2_easy_s0_headline | bowl_06 | Counter | LowerRack | [] | False | [569.4, 718.6, 10.6] | None | not attempted: trial ended early (move 1 (park bowl_01) failed twice) | not attempted |  |  |
| p0v2_easy_s0_headline | bowl_07 | LowerRack | UpperRack | [] | False | [356.5, 306.1, 148.6] | LowerRack | not attempted: trial ended early (move 1 (park bowl_01) failed twice) | not attempted |  |  |
| p0v2_easy_s0_headline_diag | bowl_01 | LowerRack | UpperRack | [1, 8] | False | [274.9, 286.5, 74.0] | LowerRack | attempt 1: unreachable x1; attempt 2: unreachable x1 | unreachable |  | `artifacts/p0v2_easy_s0_headline_diag/t00_nominal/trial.jsonl:L3660` |
| p0v2_easy_s0_headline_diag | bowl_02 | Counter | LowerRack | [3] | False | [368.8, 653.3, 91.4] | None | attempt 1: acquire x5, IK branch jump (rise to the safe height) x1; attempt 2: IK branch jump (rise to the safe height) x6 | IK branch jump (rise to the safe height) |  | `artifacts/p0v2_easy_s0_headline_diag/t00_nominal/trial.jsonl:L3354` |
| p0v2_easy_s0_headline_diag | bowl_03 | Counter | LowerRack | [4] | False | [749.0, 655.0, 40.1] | None | attempt 1: unreachable x1; attempt 2: unreachable x1 | unreachable |  | `artifacts/p0v2_easy_s0_headline_diag/t00_nominal/trial.jsonl:L3385` |
| p0v2_easy_s0_headline_diag | bowl_04 | Counter | UpperRack | [7] | False | [531.5, 343.5, 45.3] | None | attempt 1: unreachable x1; attempt 2: unreachable x1 | unreachable |  | `artifacts/p0v2_easy_s0_headline_diag/t00_nominal/trial.jsonl:L3659` |
| p0v2_easy_s0_headline_diag | bowl_05 | Counter | LowerRack | [5] | False | [433.1, 664.4, 61.3] | None | attempt 1: unreachable x1; attempt 2: unreachable x1 | unreachable |  | `artifacts/p0v2_easy_s0_headline_diag/t00_nominal/trial.jsonl:L3416` |
| p0v2_easy_s0_headline_diag | bowl_06 | Counter | LowerRack | [6] | False | [569.4, 718.6, 10.6] | None | attempt 1: unreachable x1; attempt 2: unreachable x1 | unreachable |  | `artifacts/p0v2_easy_s0_headline_diag/t00_nominal/trial.jsonl:L3447` |
| p0v2_easy_s0_headline_diag | bowl_07 | LowerRack | UpperRack | [2, 9] | False | [444.1, 310.2, 141.6] | LowerRack | attempt 1: transport blocked (lag limit) x1; attempt 2: invariant x1, transport blocked (lag limit) x1 | transport blocked (lag limit) | carried_contact | `artifacts/p0v2_easy_s0_headline_diag/t00_nominal/trial.jsonl:L3661` `artifacts/p0v2_easy_s0_headline_diag/t00_nominal/trial.jsonl:L1379` |

## 5. easy_s0 under the harness

### p0v2_easy_s0_headline (headline)

result FAIL, abort `move 1 (park bowl_01) failed twice`, end check `aborted`, counter-full refusals 0, moves ok 0/1, sim 9.75 s / wall 65.2 s.

| move | kind | dish | ok | physically ok | attempts | class | cause | sim s | wall s |
|---|---|---|---|---|---|---|---|---|---|
| 1 | park | bowl_01 | False | False | 2 | unreachable | attempt 1: unreachable x1; attempt 2: unreachable x1 | 1.5 | 10.1 |

### p0v2_easy_s0_headline_diag (headline + diagnostic continue (D18, not headline))

result FAIL, abort `None`, end check `not_all_racked`, counter-full refusals 0, moves ok 0/9, sim 169.333 s / wall 1286.4 s.

| move | kind | dish | ok | physically ok | attempts | class | cause | sim s | wall s |
|---|---|---|---|---|---|---|---|---|---|
| 1 | park | bowl_01 | False | False | 2 | unreachable | attempt 1: unreachable x1; attempt 2: unreachable x1 | 1.5 | 12.5 |
| 2 | park | bowl_07 | False | False | 2 | transport blocked (lag limit) | attempt 1: transport blocked (lag limit) x1; attempt 2: invariant x1, transport blocked (lag limit) x1 | 65.833 | 492.6 |
| 3 | goal | bowl_02 | False | False | 2 | IK branch jump (rise to the safe height) | attempt 1: acquire x5, IK branch jump (rise to the safe height) x1; attempt 2: IK branch jump (rise to the safe height) x6 | 78.75 | 562.6 |
| 4 | goal | bowl_03 | False | False | 2 | unreachable | attempt 1: unreachable x1; attempt 2: unreachable x1 | 1.5 | 19.7 |
| 5 | goal | bowl_05 | False | False | 2 | unreachable | attempt 1: unreachable x1; attempt 2: unreachable x1 | 1.5 | 25.2 |
| 6 | goal | bowl_06 | False | False | 2 | unreachable | attempt 1: unreachable x1; attempt 2: unreachable x1 | 1.5 | 27.6 |
| 7 | goal | bowl_04 | False | False | 2 | unreachable | attempt 1: unreachable x1; attempt 2: unreachable x1 | 1.5 | 26.5 |
| 8 | goal | bowl_01 | False | False | 0 | skipped | skipped: the dish failed at move 1 (D18) | 0.0 | 0.0 |
| 9 | goal | bowl_07 | False | False | 0 | skipped | skipped: the dish failed at move 2 (D18) | 0.0 | 0.0 |

## 6. Measured findings of Phase 0

- **Conventions on the live asset** (`artifacts/p0_conventions/conventions.json`): TCP axes = the commanded frame ([[1.0, 0.0, -0.0], [0.0, -1.0, -0.0], [-0.0, 0.0, -1.0]]); wrist-3 -> TCP along +z by 0.130 m; pads at y = -52.2 / 52.2 mm open, -9.8 / 9.8 mm closed (gap 19.6 mm); pad face normals [0.0, 0.9999, -0.0111] / [-0.0, -0.9999, -0.0111] (TCP frame).
- **Finger-joint signs**: unloaded close ratios {"right_outer_knuckle_joint": 1.0, "left_inner_finger_joint": -1.0, "right_inner_finger_joint": 1.0, "left_inner_finger_knuckle_joint": -1.0, "right_inner_finger_knuckle_joint": -1.0} vs the payload's mimic gearings {"right_outer_knuckle_joint": 1.0, "right_inner_finger_joint": 1.0, "right_inner_finger_knuckle_joint": -1.0, "left_inner_finger_knuckle_joint": -1.0, "left_inner_finger_joint": -1.0} and the code's INNER_FINGER_SIGNS {"left_inner_finger_joint": -1.0, "right_inner_finger_joint": 1.0}: match = True.
- **D7 / H2 runtime**: loaded close on the bowl rim stops at finger_joint 0.606 rad with ratios {"right_outer_knuckle_joint": 0.9995, "left_inner_finger_joint": -0.9999, "right_inner_finger_joint": 0.9997, "left_inner_finger_knuckle_joint": -0.9998, "right_inner_finger_knuckle_joint": -0.9997}: PhysX on Isaac 4.5 honours all five mimic couplings (the four rotX instances on Z-axis joints included).
- **Contact monitor vs Isaac Lab ContactSensor** on the same close: sensor (whole inner-finger bodies) [7.46, 11.174] N, contact reports by robot part {"RobotB:pad_R": 11.205, "RobotB:left_inner_finger": 7.46, "RobotB:pad_L": 0.0, "RobotB:left_inner_knuckle": 0.0} N -- the sensor's left value is carried by the finger side, not the pad.
- **D12 press test**: robot A (gripper de-instanced, arm colliders instanced (the episode before D12)): plate rest z 0.635 m, links ['RobotA:forearm_link', 'RobotA:upper_arm_link', 'RobotA:wrist_1_link']; robot B (whole robot de-instanced (D12 candidate)): plate rest z 0.6351 m, links ['RobotB:forearm_link', 'RobotB:upper_arm_link', 'RobotB:wrist_1_link']. Instanced arm colliders DO make contacts on 4.5 (only the gripper's were dead): the episode keeps `deinstance_root = /World/Robot/Gripper`; de-instancing the whole robot is available behind the flag.

- **In-hand settle after the lift** (`p0_upright3_legacy`): move 1 bowl_01: settle displacement 27.67 mm / 20.73 deg, held True (`artifacts/p0_upright3_legacy/t00_nominal/trial.jsonl:L625`); move 2 bowl_02: settle displacement 28.13 mm / 21.09 deg, held True (`artifacts/p0_upright3_legacy/t00_nominal/trial.jsonl:L1481`); move 3 bowl_03: settle displacement 28.91 mm / 21.49 deg, held True (`artifacts/p0_upright3_legacy/t00_nominal/trial.jsonl:L2667`).
- **Arm tracking error** (`p0_upright3_legacy`, max |measured - commanded| in mrad over 20 Hz samples; plot `artifacts/p0_upright3_legacy/t00_nominal/plots/joint_tracking.png`): carrying a dish (1454 samples): shoulder_pan 1.1, shoulder_lift 22.3, elbow 28.8, wrist_1 41.8, wrist_2 7.2, wrist_3 50.2; free (1735 samples): shoulder_pan 2.0, shoulder_lift 11.6, elbow 14.0, wrist_1 28.7, wrist_2 6.3, wrist_3 52.6. The rig's pre-R7 settled limit is 30 mrad, the legacy carry limit 80 mrad (Phase 1a measures the static error per posture).
- **Invariants fired** (`p0_upright3_legacy`): carried_contact at move 1 close: {"dish": "bowl_01", "partner": "right_inner_knuckle", "category": "finger", "allowed": ["pad", "counter", "rack", "Counter"], "force_N": 0.0, "colliders": ["Defeatured_2F_85_PAD_OPEN_finger3step", "Shell_163", "Shell_164 (`artifacts/p0_upright3_legacy/t00_nominal/trial.jsonl:L556`); carried_contact at move 1 lift: {"dish": "bowl_01", "partner": "right_inner_finger", "category": "finger", "allowed": ["pad", "counter", "rack", "Counter"], "force_N": 0.0, "colliders": ["Defeatured_2F_85_PAD_OPEN_finger4step", "Shell_163", "Shell_164" (`artifacts/p0_upright3_legacy/t00_nominal/trial.jsonl:L575`); carried_contact at move 1 lift: {"dish": "bowl_01", "partner": "left_inner_knuckle", "category": "finger", "allowed": ["pad", "counter", "rack", "Counter"], "force_N": 0.0, "colliders": ["Defeatured_2F_85_PAD_OPEN_finger3step", "Shell_164", "Shell_188" (`artifacts/p0_upright3_legacy/t00_nominal/trial.jsonl:L585`); carried_contact at move 1 lift: {"dish": "bowl_01", "partner": "left_inner_finger", "category": "finger", "allowed": ["pad", "counter", "rack", "Counter"], "force_N": 0.0, "colliders": ["Defeatured_2F_85_PAD_OPEN_finger4step", "Shell_163", "Shell_164"] (`artifacts/p0_upright3_legacy/t00_nominal/trial.jsonl:L587`); carried_contact at move 1 insertion: {"dish": "bowl_01", "partner": "SilverwareBasket", "category": "basket", "allowed": ["pad", "counter", "rack"], "force_N": 19.95, "colliders": ["Shell_100", "Shell_101", "Wire_00040", "Wire_00041", "Wire_00042"]} (`artifacts/p0_upright3_legacy/t00_nominal/trial.jsonl:L1054`); carried_contact at move 2 close: {"dish": "bowl_02", "partner": "right_inner_knuckle", "category": "finger", "allowed": ["pad", "counter", "rack", "Counter"], "force_N": 0.0, "colliders": ["Defeatured_2F_85_PAD_OPEN_finger3step", "Shell_163", "Shell_164 (`artifacts/p0_upright3_legacy/t00_nominal/trial.jsonl:L1412`); carried_contact at move 2 lift: {"dish": "bowl_02", "partner": "right_inner_finger", "category": "finger", "allowed": ["pad", "counter", "rack", "Counter"], "force_N": 0.0, "colliders": ["Defeatured_2F_85_PAD_OPEN_finger4step", "Shell_163", "Shell_164" (`artifacts/p0_upright3_legacy/t00_nominal/trial.jsonl:L1432`); carried_contact at move 2 lift: {"dish": "bowl_02", "partner": "left_inner_knuckle", "category": "finger", "allowed": ["pad", "counter", "rack", "Counter"], "force_N": 0.0, "colliders": ["Defeatured_2F_85_PAD_OPEN_finger3step", "Shell_164", "Shell_188" (`artifacts/p0_upright3_legacy/t00_nominal/trial.jsonl:L1441`).
- **The rim pinch loads the inner knuckles** (`p0_upright3_legacy`): ['bowl_01', 'right_inner_knuckle'] peak 21 N for 0.86 s, min separation -0.00017 m (`artifacts/p0_upright3_legacy/t00_nominal/trial.jsonl:L581`); ['bowl_01', 'left_inner_knuckle'] peak 49 N for 23.02 s, min separation -0.00017 m (`artifacts/p0_upright3_legacy/t00_nominal/trial.jsonl:L1059`); ['bowl_02', 'right_inner_knuckle'] peak 21 N for 0.86 s, min separation -0.00018 m (`artifacts/p0_upright3_legacy/t00_nominal/trial.jsonl:L1437`); ['bowl_02', 'left_inner_knuckle'] peak 48 N for 25.23 s, min separation -9e-05 m (`artifacts/p0_upright3_legacy/t00_nominal/trial.jsonl:L1960`); ['bowl_03', 'right_inner_knuckle'] peak 11 N for 0.86 s, min separation -0.00012 m (`artifacts/p0_upright3_legacy/t00_nominal/trial.jsonl:L2622`); ['bowl_03', 'left_inner_knuckle'] peak 49 N for 20.12 s, min separation -0.00021 m (`artifacts/p0_upright3_legacy/t00_nominal/trial.jsonl:L3040`). The bowl's flared rim rests on the knuckle collider after the ~28 mm in-hand pivot: the D4 allowed-contact list (pads only) is violated by the legacy grasp.
- **Arm tracking error** (`p0_upright3_headline`, max |measured - commanded| in mrad over 20 Hz samples; plot `artifacts/p0_upright3_headline/t00_nominal/plots/joint_tracking.png`): carrying a dish (26 samples): shoulder_pan 0.1, shoulder_lift 1.4, elbow 2.9, wrist_1 1.6, wrist_2 0.7, wrist_3 4.2; free (886 samples): shoulder_pan 0.4, shoulder_lift 4.1, elbow 3.6, wrist_1 4.6, wrist_2 2.0, wrist_3 1.7. The rig's pre-R7 settled limit is 30 mrad, the legacy carry limit 80 mrad (Phase 1a measures the static error per posture).
- **Invariants fired** (`p0_upright3_headline`): carried_contact at move 1 close: {"dish": "bowl_01", "partner": "right_inner_knuckle", "category": "finger", "allowed": ["pad", "counter", "rack", "Counter"], "force_N": 0.0, "colliders": ["Defeatured_2F_85_PAD_OPEN_finger3step", "Shell_163", "Shell_164 (`artifacts/p0_upright3_headline/t00_nominal/trial.jsonl:L556`); carried_contact at move None end: {"dish": "bowl_01", "partner": "right_inner_knuckle", "category": "finger", "allowed": ["pad", "counter", "rack", "Counter"], "force_N": 2.42, "colliders": ["Defeatured_2F_85_PAD_OPEN_finger3step", "Shell_163", "Shell_16 (`artifacts/p0_upright3_headline/t00_nominal/trial.jsonl:L558`).
- **The rim pinch loads the inner knuckles** (`p0_upright3_headline`): ['bowl_01', 'right_inner_knuckle'] peak 32 N for 0.05 s, min separation -0.0001 m (`artifacts/p0_upright3_headline/t00_nominal/trial.jsonl:L561`). The bowl's flared rim rests on the knuckle collider after the ~28 mm in-hand pivot: the D4 allowed-contact list (pads only) is violated by the legacy grasp.
- **In-hand settle after the lift** (`p0v2_upright3_legacy`): move 1 bowl_01: settle displacement 27.67 mm / 20.73 deg, held True (`artifacts/p0v2_upright3_legacy/t00_nominal/trial.jsonl:L634`); move 2 bowl_02: settle displacement 28.13 mm / 21.09 deg, held True (`artifacts/p0v2_upright3_legacy/t00_nominal/trial.jsonl:L1481`); move 3 bowl_03: settle displacement 28.91 mm / 21.49 deg, held True (`artifacts/p0v2_upright3_legacy/t00_nominal/trial.jsonl:L2656`).
- **Arm tracking error** (`p0v2_upright3_legacy`, max |measured - commanded| in mrad over 20 Hz samples; plot `artifacts/p0v2_upright3_legacy/t00_nominal/plots/joint_tracking.png`): carrying a dish (1454 samples): shoulder_pan 1.1, shoulder_lift 22.3, elbow 28.8, wrist_1 41.8, wrist_2 7.2, wrist_3 50.2; free (1735 samples): shoulder_pan 2.0, shoulder_lift 11.6, elbow 14.0, wrist_1 28.7, wrist_2 6.3, wrist_3 52.6. The rig's pre-R7 settled limit is 30 mrad, the legacy carry limit 80 mrad (Phase 1a measures the static error per posture).
- **Invariants fired** (`p0v2_upright3_legacy`): carried_contact at move 1 close: {"dish": "bowl_01", "partner": "right_inner_knuckle", "category": "finger", "allowed": ["pad", "counter", "rack", "basket", "Counter"], "force_N": 2.376, "colliders": ["Defeatured_2F_85_PAD_OPEN_finger3step", "Shell_187" (`artifacts/p0v2_upright3_legacy/t00_nominal/trial.jsonl:L570`); carried_contact at move 1 lift: {"dish": "bowl_01", "partner": "left_inner_knuckle", "category": "finger", "allowed": ["pad", "counter", "rack", "basket", "Counter"], "force_N": 1.223, "colliders": ["Defeatured_2F_85_PAD_OPEN_finger3step", "Shell_164"] (`artifacts/p0v2_upright3_legacy/t00_nominal/trial.jsonl:L596`); carried_contact at move 2 close: {"dish": "bowl_02", "partner": "right_inner_knuckle", "category": "finger", "allowed": ["pad", "counter", "rack", "basket", "Counter"], "force_N": 1.757, "colliders": ["Defeatured_2F_85_PAD_OPEN_finger3step", "Shell_163" (`artifacts/p0v2_upright3_legacy/t00_nominal/trial.jsonl:L1417`); carried_contact at move 2 lift: {"dish": "bowl_02", "partner": "left_inner_knuckle", "category": "finger", "allowed": ["pad", "counter", "rack", "basket", "Counter"], "force_N": 12.226, "colliders": ["Defeatured_2F_85_PAD_OPEN_finger3step", "Shell_164" (`artifacts/p0v2_upright3_legacy/t00_nominal/trial.jsonl:L1443`); carried_contact at move 3 close: {"dish": "bowl_03", "partner": "right_inner_knuckle", "category": "finger", "allowed": ["pad", "counter", "rack", "basket", "Counter"], "force_N": 8.006, "colliders": ["Defeatured_2F_85_PAD_OPEN_finger3step", "Shell_163" (`artifacts/p0v2_upright3_legacy/t00_nominal/trial.jsonl:L2592`); carried_contact at move 3 lift: {"dish": "bowl_03", "partner": "left_inner_knuckle", "category": "finger", "allowed": ["pad", "counter", "rack", "basket", "Counter"], "force_N": 24.389, "colliders": ["Defeatured_2F_85_PAD_OPEN_finger3step", "Shell_163" (`artifacts/p0v2_upright3_legacy/t00_nominal/trial.jsonl:L2618`).
- **The rim pinch loads the inner knuckles** (`p0v2_upright3_legacy`): ['bowl_01', 'right_inner_knuckle'] peak 21 N for 0.75 s, min separation -0.00017 m (`artifacts/p0v2_upright3_legacy/t00_nominal/trial.jsonl:L591`); ['bowl_01', 'left_inner_knuckle'] peak 49 N for 22.98 s, min separation -0.00017 m (`artifacts/p0v2_upright3_legacy/t00_nominal/trial.jsonl:L1064`); ['bowl_02', 'right_inner_knuckle'] peak 21 N for 0.75 s, min separation -0.00018 m (`artifacts/p0v2_upright3_legacy/t00_nominal/trial.jsonl:L1438`); ['bowl_02', 'left_inner_knuckle'] peak 48 N for 25.18 s, min separation -9e-05 m (`artifacts/p0v2_upright3_legacy/t00_nominal/trial.jsonl:L1954`); ['bowl_03', 'right_inner_knuckle'] peak 11 N for 0.75 s, min separation -0.00012 m (`artifacts/p0v2_upright3_legacy/t00_nominal/trial.jsonl:L2613`); ['bowl_03', 'left_inner_knuckle'] peak 49 N for 20.08 s, min separation -0.00021 m (`artifacts/p0v2_upright3_legacy/t00_nominal/trial.jsonl:L3027`). The bowl's flared rim rests on the knuckle collider after the ~28 mm in-hand pivot: the D4 allowed-contact list (pads only) is violated by the legacy grasp.
- **In-hand settle after the lift** (`p0v2_upright3_legacy_nosleep`): move 1 bowl_01: settle displacement 28.2 mm / 21.12 deg, held True (`artifacts/p0v2_upright3_legacy_nosleep/t00_nominal/trial.jsonl:L641`); move 2 bowl_02: settle displacement 27.81 mm / 20.87 deg, held True (`artifacts/p0v2_upright3_legacy_nosleep/t00_nominal/trial.jsonl:L1491`); move 3 bowl_03: settle displacement 27.87 mm / 20.66 deg, held True (`artifacts/p0v2_upright3_legacy_nosleep/t00_nominal/trial.jsonl:L2689`).
- **Arm tracking error** (`p0v2_upright3_legacy_nosleep`, max |measured - commanded| in mrad over 20 Hz samples; plot `artifacts/p0v2_upright3_legacy_nosleep/t00_nominal/plots/joint_tracking.png`): carrying a dish (1469 samples): shoulder_pan 1.8, shoulder_lift 20.4, elbow 23.4, wrist_1 43.2, wrist_2 7.2, wrist_3 51.7; free (1733 samples): shoulder_pan 2.0, shoulder_lift 12.1, elbow 10.1, wrist_1 31.7, wrist_2 5.8, wrist_3 47.2. The rig's pre-R7 settled limit is 30 mrad, the legacy carry limit 80 mrad (Phase 1a measures the static error per posture).
- **Invariants fired** (`p0v2_upright3_legacy_nosleep`): sleeping_body at move None rack: {"bodies": ["bowl_01", "bowl_02", "bowl_03"]} (`artifacts/p0v2_upright3_legacy_nosleep/t00_nominal/trial.jsonl:L67`); sleeping_body at move 1 pick: {"bodies": ["bowl_01", "bowl_02", "bowl_03"]} (`artifacts/p0v2_upright3_legacy_nosleep/t00_nominal/trial.jsonl:L247`); sleeping_body at move 1 close: {"bodies": ["bowl_02", "bowl_03"]} (`artifacts/p0v2_upright3_legacy_nosleep/t00_nominal/trial.jsonl:L572`); carried_contact at move 1 close: {"dish": "bowl_01", "partner": "right_inner_knuckle", "category": "finger", "allowed": ["pad", "counter", "rack", "basket"], "force_N": 2.282, "colliders": ["Defeatured_2F_85_PAD_OPEN_finger3step", "Shell_187", "Shell_18 (`artifacts/p0v2_upright3_legacy_nosleep/t00_nominal/trial.jsonl:L577`); carried_contact at move 1 lift: {"dish": "bowl_01", "partner": "left_inner_knuckle", "category": "finger", "allowed": ["pad", "counter", "rack", "basket"], "force_N": 2.05, "colliders": ["Defeatured_2F_85_PAD_OPEN_finger3step", "Shell_164"]} (`artifacts/p0v2_upright3_legacy_nosleep/t00_nominal/trial.jsonl:L603`); sleeping_body at move 2 pick: {"bodies": ["bowl_02", "bowl_03"]} (`artifacts/p0v2_upright3_legacy_nosleep/t00_nominal/trial.jsonl:L1154`); sleeping_body at move 2 close: {"bodies": ["bowl_03"]} (`artifacts/p0v2_upright3_legacy_nosleep/t00_nominal/trial.jsonl:L1420`); carried_contact at move 2 close: {"dish": "bowl_02", "partner": "right_inner_knuckle", "category": "finger", "allowed": ["pad", "counter", "rack", "basket"], "force_N": 2.087, "colliders": ["Defeatured_2F_85_PAD_OPEN_finger3step", "Shell_187", "Shell_18 (`artifacts/p0v2_upright3_legacy_nosleep/t00_nominal/trial.jsonl:L1427`).
- **The rim pinch loads the inner knuckles** (`p0v2_upright3_legacy_nosleep`): ['bowl_01', 'right_inner_knuckle'] peak 20 N for 0.76 s, min separation -0.00014 m (`artifacts/p0v2_upright3_legacy_nosleep/t00_nominal/trial.jsonl:L598`); ['bowl_01', 'left_inner_knuckle'] peak 48 N for 22.98 s, min separation -0.00018 m (`artifacts/p0v2_upright3_legacy_nosleep/t00_nominal/trial.jsonl:L1071`); ['bowl_02', 'right_inner_knuckle'] peak 20 N for 0.75 s, min separation -0.00012 m (`artifacts/p0v2_upright3_legacy_nosleep/t00_nominal/trial.jsonl:L1448`); ['bowl_02', 'left_inner_knuckle'] peak 49 N for 25.17 s, min separation -0.0002 m (`artifacts/p0v2_upright3_legacy_nosleep/t00_nominal/trial.jsonl:L1964`); ['bowl_03', 'right_inner_knuckle'] peak 17 N for 0.73 s, min separation -0.00014 m (`artifacts/p0v2_upright3_legacy_nosleep/t00_nominal/trial.jsonl:L2646`); ['bowl_03', 'left_inner_knuckle'] peak 49 N for 20.75 s, min separation -0.00017 m (`artifacts/p0v2_upright3_legacy_nosleep/t00_nominal/trial.jsonl:L3073`). The bowl's flared rim rests on the knuckle collider after the ~28 mm in-hand pivot: the D4 allowed-contact list (pads only) is violated by the legacy grasp.
- **Arm tracking error** (`p0v2_upright3_headline`, max |measured - commanded| in mrad over 20 Hz samples; plot `artifacts/p0v2_upright3_headline/t00_nominal/plots/joint_tracking.png`): carrying a dish (11 samples): shoulder_pan 0.1, shoulder_lift 0.3, elbow 0.9, wrist_1 0.7, wrist_2 0.3, wrist_3 1.0; free (901 samples): shoulder_pan 0.4, shoulder_lift 1.9, elbow 4.2, wrist_1 3.8, wrist_2 1.9, wrist_3 12.4. The rig's pre-R7 settled limit is 30 mrad, the legacy carry limit 80 mrad (Phase 1a measures the static error per posture).
- **Invariants fired** (`p0v2_upright3_headline`): carried_contact at move 1 close: {"dish": "bowl_01", "partner": "right_inner_knuckle", "category": "finger", "allowed": ["pad", "counter", "rack", "basket", "Counter"], "force_N": 2.376, "colliders": ["Defeatured_2F_85_PAD_OPEN_finger3step", "Shell_187" (`artifacts/p0v2_upright3_headline/t00_nominal/trial.jsonl:L570`).
- **The rim pinch loads the inner knuckles** (`p0v2_upright3_headline`): ['bowl_01', 'right_inner_knuckle'] peak 25 N for 0.03 s, min separation -0.0001 m (`artifacts/p0v2_upright3_headline/t00_nominal/trial.jsonl:L574`). The bowl's flared rim rests on the knuckle collider after the ~28 mm in-hand pivot: the D4 allowed-contact list (pads only) is violated by the legacy grasp.
- **In-hand settle after the lift** (`p0v2_upright3_headline_diag`): move 1 bowl_01: settle displacement 27.67 mm / 20.73 deg, drift 0.003 mm over 2.0 s, held True (`artifacts/p0v2_upright3_headline_diag/t00_nominal/trial.jsonl:L674`); move 1 bowl_01: settle displacement 27.7 mm / 20.8 deg, drift 0.002 mm over 2.0 s, held True (`artifacts/p0v2_upright3_headline_diag/t00_nominal/trial.jsonl:L1068`); move 2 bowl_02: settle displacement 28.63 mm / 21.29 deg, drift 0.075 mm over 2.0 s, held True (`artifacts/p0v2_upright3_headline_diag/t00_nominal/trial.jsonl:L1500`); move 2 bowl_02: settle displacement 27.9 mm / 21.11 deg, drift 0.002 mm over 2.0 s, held True (`artifacts/p0v2_upright3_headline_diag/t00_nominal/trial.jsonl:L1904`); move 3 bowl_03: settle displacement 27.77 mm / 20.9 deg, drift 0.001 mm over 2.0 s, held True (`artifacts/p0v2_upright3_headline_diag/t00_nominal/trial.jsonl:L2587`); move 3 bowl_03: settle displacement 27.44 mm / 20.88 deg, drift 0.001 mm over 2.0 s, held True (`artifacts/p0v2_upright3_headline_diag/t00_nominal/trial.jsonl:L2982`).
- **Arm tracking error** (`p0v2_upright3_headline_diag`, max |measured - commanded| in mrad over 20 Hz samples; plot `artifacts/p0v2_upright3_headline_diag/t00_nominal/plots/joint_tracking.png`): carrying a dish (1674 samples): shoulder_pan 0.5, shoulder_lift 3.2, elbow 5.6, wrist_1 5.3, wrist_2 4.6, wrist_3 48.9; free (1478 samples): shoulder_pan 3.8, shoulder_lift 7.8, elbow 4.0, wrist_1 2.1, wrist_2 3.0, wrist_3 2.9. The rig's pre-R7 settled limit is 30 mrad, the legacy carry limit 80 mrad (Phase 1a measures the static error per posture).
- **Invariants fired** (`p0v2_upright3_headline_diag`): carried_contact at move 1 close: {"dish": "bowl_01", "partner": "right_inner_knuckle", "category": "finger", "allowed": ["pad", "counter", "rack", "basket", "Counter"], "force_N": 2.376, "colliders": ["Defeatured_2F_85_PAD_OPEN_finger3step", "Shell_187" (`artifacts/p0v2_upright3_headline_diag/t00_nominal/trial.jsonl:L570`); carried_contact at move 1 lift: {"dish": "bowl_01", "partner": "left_inner_knuckle", "category": "finger", "allowed": ["pad", "counter", "rack", "basket", "Counter"], "force_N": 1.223, "colliders": ["Defeatured_2F_85_PAD_OPEN_finger3step", "Shell_164"] (`artifacts/p0v2_upright3_headline_diag/t00_nominal/trial.jsonl:L596`); carried_contact at move 2 close: {"dish": "bowl_02", "partner": "right_inner_knuckle", "category": "finger", "allowed": ["pad", "counter", "rack", "basket", "Counter"], "force_N": 1.641, "colliders": ["Defeatured_2F_85_PAD_OPEN_finger3step", "Shell_163" (`artifacts/p0v2_upright3_headline_diag/t00_nominal/trial.jsonl:L1396`); carried_contact at move 2 lift: {"dish": "bowl_02", "partner": "left_inner_knuckle", "category": "finger", "allowed": ["pad", "counter", "rack", "basket", "Counter"], "force_N": 1.645, "colliders": ["Defeatured_2F_85_PAD_OPEN_finger3step", "Shell_164"] (`artifacts/p0v2_upright3_headline_diag/t00_nominal/trial.jsonl:L1422`); carried_contact at move 2 settle: {"dish": "bowl_02", "partner": "bowl_01", "category": "dish", "allowed": ["pad", "counter", "rack", "basket"], "force_N": 3.396, "colliders": ["Shell_144", "Shell_153"]} (`artifacts/p0v2_upright3_headline_diag/t00_nominal/trial.jsonl:L1673`); carried_contact at move 3 close: {"dish": "bowl_03", "partner": "right_inner_knuckle", "category": "finger", "allowed": ["pad", "counter", "rack", "basket", "Counter"], "force_N": 7.213, "colliders": ["Defeatured_2F_85_PAD_OPEN_finger3step", "Shell_163" (`artifacts/p0v2_upright3_headline_diag/t00_nominal/trial.jsonl:L2483`); carried_contact at move 3 lift: {"dish": "bowl_03", "partner": "left_inner_knuckle", "category": "finger", "allowed": ["pad", "counter", "rack", "basket", "Counter"], "force_N": 21.651, "colliders": ["Defeatured_2F_85_PAD_OPEN_finger3step", "Shell_163" (`artifacts/p0v2_upright3_headline_diag/t00_nominal/trial.jsonl:L2509`).
- **The rim pinch loads the inner knuckles** (`p0v2_upright3_headline_diag`): ['bowl_01', 'right_inner_knuckle'] peak 21 N for 0.75 s, min separation -0.00017 m (`artifacts/p0v2_upright3_headline_diag/t00_nominal/trial.jsonl:L591`); ['bowl_01', 'left_inner_knuckle'] peak 49 N for 11.75 s, min separation -0.00017 m (`artifacts/p0v2_upright3_headline_diag/t00_nominal/trial.jsonl:L842`); ['bowl_01', 'right_inner_knuckle'] peak 18 N for 0.75 s, min separation -0.00017 m (`artifacts/p0v2_upright3_headline_diag/t00_nominal/trial.jsonl:L986`); ['bowl_01', 'left_inner_knuckle'] peak 49 N for 11.74 s, min separation -9e-05 m (`artifacts/p0v2_upright3_headline_diag/t00_nominal/trial.jsonl:L1236`); ['bowl_02', 'right_inner_knuckle'] peak 21 N for 0.75 s, min separation -0.00011 m (`artifacts/p0v2_upright3_headline_diag/t00_nominal/trial.jsonl:L1417`); ['bowl_02', 'left_inner_knuckle'] peak 47 N for 11.75 s, min separation -0.00017 m (`artifacts/p0v2_upright3_headline_diag/t00_nominal/trial.jsonl:L1668`). The bowl's flared rim rests on the knuckle collider after the ~28 mm in-hand pivot: the D4 allowed-contact list (pads only) is violated by the legacy grasp.
- **In-hand settle after the lift** (`p0v2_easy_s0_headline_diag`): move 2 bowl_07: settle displacement 49.4 mm / 33.22 deg, drift 0.095 mm over 2.0 s, held True (`artifacts/p0v2_easy_s0_headline_diag/t00_nominal/trial.jsonl:L801`); move 2 bowl_07: settle displacement 23.62 mm / 18.32 deg, drift 0.019 mm over 2.0 s, held True (`artifacts/p0v2_easy_s0_headline_diag/t00_nominal/trial.jsonl:L1460`).
- **Arm tracking error** (`p0v2_easy_s0_headline_diag`, max |measured - commanded| in mrad over 20 Hz samples; plot `artifacts/p0v2_easy_s0_headline_diag/t00_nominal/plots/joint_tracking.png`): carrying a dish (859 samples): shoulder_pan 9.5, shoulder_lift 28.4, elbow 31.3, wrist_1 26.4, wrist_2 17.2, wrist_3 58.6; free (2755 samples): shoulder_pan 6.6, shoulder_lift 21.8, elbow 20.4, wrist_1 25.9, wrist_2 9.1, wrist_3 15.1. The rig's pre-R7 settled limit is 30 mrad, the legacy carry limit 80 mrad (Phase 1a measures the static error per posture).
- **Invariants fired** (`p0v2_easy_s0_headline_diag`): carried_contact at move 2 lift: {"dish": "bowl_07", "partner": "left_inner_knuckle", "category": "finger", "allowed": ["pad", "counter", "rack", "basket", "LowerRack"], "force_N": 12.433, "colliders": ["Defeatured_2F_85_PAD_OPEN_finger3step", "Shell_14 (`artifacts/p0v2_easy_s0_headline_diag/t00_nominal/trial.jsonl:L1379`).
- **The rim pinch loads the inner knuckles** (`p0v2_easy_s0_headline_diag`): ['bowl_07', 'left_inner_knuckle'] peak 36 N for 16.85 s, min separation -0.00025 m (`artifacts/p0v2_easy_s0_headline_diag/t00_nominal/trial.jsonl:L1727`). The bowl's flared rim rests on the knuckle collider after the ~28 mm in-hand pivot: the D4 allowed-contact list (pads only) is violated by the legacy grasp.

## 7. Deviations from the benchmark, with their flags

Every relaxation is a named field of `frigidaire/src/dishsim_frigidaire/robot/flags.py` (headline default off); each run's record stores its resolved flags and `deviations_from_headline`.

| run | flag | plan item | value | headline value |
|---|---|---|---|---|
| p0_upright3_legacy | test_case | R4 | "upright3" | null |
| p0_upright3_legacy | place_mode | R5 | "lower_until_contact" | "goal_pose" |
| p0_upright3_legacy | judge | R6 | "racked_in" | "at_goal" |
| p0_upright3_legacy | disturbance | D1 | "any_neighbour" | "benchmark" |
| p0_upright3_legacy | end_check | D2 | "racked_in" | "benchmark" |
| p0_upright3_legacy | move_settle | D3 | "legacy" | "benchmark" |
| p0_upright3_legacy | final_hold_observation_s | D3 | 0.0 | 2.0 |
| p0_upright3_legacy | reextend_before_scoring | R3/D2 | false | true |
| p0_upright3_legacy | hold_gate | R1/D9 | "legacy_rise" | "drift" |
| p0_upright3_legacy | tol_approach | R7/D8 | 0.06 | null |
| p0_upright3_legacy | tol_joint_move_held | R7/D8 | 0.08 | null |
| p0_upright3_legacy | tol_rrt | R7/D8 | 0.08 | null |
| p0_upright3_legacy | tol_rise | R7/D8 | 0.08 | null |
| p0_upright3_legacy | tol_lift | R7/D8 | 0.08 | null |
| p0_upright3_legacy | tol_reaim | R7/D8 | 0.08 | null |
| p0_upright3_legacy | counter_cap | D19 | false | true |
| p0_upright3_legacy | invariants_abort | Phase 0.4 | false | true |
| p0_upright3_legacy | diag_settle_active | D3 | false | true |
| p0_upright3_headline | test_case | R4 | "upright3" | null |
| p0v2_upright3_legacy | test_case | R4 | "upright3" | null |
| p0v2_upright3_legacy | place_mode | R5 | "lower_until_contact" | "goal_pose" |
| p0v2_upright3_legacy | judge | R6 | "racked_in" | "at_goal" |
| p0v2_upright3_legacy | disturbance | D1 | "any_neighbour" | "benchmark" |
| p0v2_upright3_legacy | end_check | D2 | "racked_in" | "benchmark" |
| p0v2_upright3_legacy | move_settle | D3 | "legacy" | "benchmark" |
| p0v2_upright3_legacy | final_hold_observation_s | D3 | 0.0 | 2.0 |
| p0v2_upright3_legacy | reextend_before_scoring | R3/D2 | false | true |
| p0v2_upright3_legacy | hold_gate | R1/D9 | "legacy_rise" | "drift" |
| p0v2_upright3_legacy | tol_approach | R7/D8 | 0.06 | null |
| p0v2_upright3_legacy | tol_joint_move_held | R7/D8 | 0.08 | null |
| p0v2_upright3_legacy | tol_rrt | R7/D8 | 0.08 | null |
| p0v2_upright3_legacy | tol_rise | R7/D8 | 0.08 | null |
| p0v2_upright3_legacy | tol_lift | R7/D8 | 0.08 | null |
| p0v2_upright3_legacy | tol_reaim | R7/D8 | 0.08 | null |
| p0v2_upright3_legacy | counter_cap | D19 | false | true |
| p0v2_upright3_legacy | invariants_abort | Phase 0.4 | false | true |
| p0v2_upright3_legacy | diag_settle_active | D3 | false | true |
| p0v2_upright3_legacy_nosleep | test_case | R4 | "upright3" | null |
| p0v2_upright3_legacy_nosleep | place_mode | R5 | "lower_until_contact" | "goal_pose" |
| p0v2_upright3_legacy_nosleep | judge | R6 | "racked_in" | "at_goal" |
| p0v2_upright3_legacy_nosleep | disturbance | D1 | "any_neighbour" | "benchmark" |
| p0v2_upright3_legacy_nosleep | end_check | D2 | "racked_in" | "benchmark" |
| p0v2_upright3_legacy_nosleep | move_settle | D3 | "legacy" | "benchmark" |
| p0v2_upright3_legacy_nosleep | final_hold_observation_s | D3 | 0.0 | 2.0 |
| p0v2_upright3_legacy_nosleep | reextend_before_scoring | R3/D2 | false | true |
| p0v2_upright3_legacy_nosleep | hold_gate | R1/D9 | "legacy_rise" | "drift" |
| p0v2_upright3_legacy_nosleep | tol_approach | R7/D8 | 0.06 | null |
| p0v2_upright3_legacy_nosleep | tol_joint_move_held | R7/D8 | 0.08 | null |
| p0v2_upright3_legacy_nosleep | tol_rrt | R7/D8 | 0.08 | null |
| p0v2_upright3_legacy_nosleep | tol_rise | R7/D8 | 0.08 | null |
| p0v2_upright3_legacy_nosleep | tol_lift | R7/D8 | 0.08 | null |
| p0v2_upright3_legacy_nosleep | tol_reaim | R7/D8 | 0.08 | null |
| p0v2_upright3_legacy_nosleep | counter_cap | D19 | false | true |
| p0v2_upright3_legacy_nosleep | sleep_threshold | D10 | null | 0.0 |
| p0v2_upright3_legacy_nosleep | invariants_abort | Phase 0.4 | false | true |
| p0v2_upright3_legacy_nosleep | diag_settle_active | D3 | false | true |
| p0v2_upright3_headline | test_case | R4 | "upright3" | null |
| p0v2_upright3_headline_diag | test_case | R4 | "upright3" | null |
| p0v2_upright3_headline_diag | continue_after_failed_dish | D18 | true | false |
| p0v2_upright3_headline_diag | invariants_abort | Phase 0.4 | false | true |
| p0v2_easy_s0_headline_diag | continue_after_failed_dish | D18 | true | false |
| p0v2_easy_s0_headline_diag | invariants_abort | Phase 0.4 | false | true |

Interpretations taken by the harness (flagged for the user, plan Section 6):

- D4: the silverware basket counts as rack furniture for the carried dish (`harness.CARRIED_ALLOWED`), because the benchmark's support rule seats it on the lower rack; a carried bowl touching the basket is therefore allowed, a bowl touching a finger body or knuckle is not.
- D1: 'at its goal before the move' uses the benchmark tolerance on the pre-move pose plus the dishes this trial already placed at their goals.
- D10 (sleep threshold 0 on the dish prims and robot links): besides the false-pass guard, it is a precondition of the contact logging: PhysX emits no contact reports for sleeping bodies (NVIDIA's RigidContactView test sets `physxRigidBody:sleepThreshold = 0` for that reason, comment 'disable sleeping, because sleeping bodies don't get contact reports'); the A/B run with the asset thresholds shows the bowls asleep on the counter.
- R3 in the headline: the rack motions are scripted environment actions (logged as `rack` rows); both racks are re-extended before the final hold and the end check.
- Privileged information used by the controller (plan Section 3): dish poses read from the simulator (grasp candidates, park/goal targets, disturbance and at-goal judging); the in-hand dish pose measured from the simulator after every lift (placement uses it); gravity disabled on every robot link: idealised gravity compensation (plan decision D6); the analytic IK / FCL arm model is calibrated to the simulated links (collide.calibrate).

## 8. Audit: fixes 1-13 and R1-R7 -> code (Phase 0.1; D20 wording)

Commits: none of fixes 1-13 / R1-R7 has a commit of its own (the whole stack was untracked when the audit was made; a pre-Phase-0 snapshot with sha256 sums sits in `artifacts/_baseline_pre_phase0/`). Git now: HEAD `c1f606f 2026-09-30 add robot stack`, 23 robot files tracked, modified since HEAD ['frigidaire/docs/robot.md', 'frigidaire/scripts/evaluation/frigidaire_robot_report.py', 'frigidaire/scripts/experiment/frigidaire_robot_episode.py', 'frigidaire/scripts/setup/mirror_robot_usd.sh', 'frigidaire/src/dishsim_frigidaire/robot/harness.py'], untracked ['frigidaire/scripts/setup/robot_asset_manifest.py', 'frigidaire/tests/test_robot_harness.py'].

| item | plan text | code | verdict | correction |
|---|---|---|---|---|
| 1 | 6.0 asset layers converted on the host in a temporary env | `mirror_robot_usd.sh:38-58` | confirmed | already scripted; unpinned usd-core, no output hashes (D11 pending: manifest written this phase, pin unverifiable without a re-download) |
| 2 | gripper colliders instanced, un-instanced before reset | `ur5e.py:77-91`, episode `before_reset` | confirmed | arm colliders were still instanced; the D12 press test (this phase) shows they DO make contacts on 4.5 |
| 3 | inner finger drives fought the close; now mirror finger_joint | `ur5e.py:25-32`, `rig.py:56` | confirmed | signs MEASURED 2026-07-31 (on-corrallab), not copied; the mimic couplings are live (D7 test this phase) |
| 4 | PD lag 0.16 rad at speed; velocity feed-forward | `rig.py:128-150` | confirmed | gains unchanged since on-corrallab |
| 5 | planner allowed +-2pi; real limits now | `kin.py:42-51`, `rig.py:93-95`, `collide.py:213-215` | partly | D20(a): the +-2pi planner is an unrecoverable draft; the ported code already had elbow +-pi |
| 6 | IK prefers wrist-up; grasp paired with an approach on the same branch | episode `goal_qs`, `pick` | confirmed by reading, untested | no test covers the pairing |
| 7 | long descents crossed a wrist singularity | episode `transit_to`, `plan_to` | confirmed | D20(b): the code's diagnosis is an IK branch jump, not a singularity |
| 8 | gripper hull misplaced; built from live poses | `collide.py:29-77` | confirmed |  |
| 9 | one hull per collider | `collide.py:75-76` | confirmed | stale docstrings `collide.py:3-4, :30` |
| 10 | contact exemption near the grasp: gripper only | `collide.py:143-144`, episode `ALLOW_CONTACT_M` | confirmed |  |
| 11 | pedestal, floor, carried bowl in the planning model | episode `add_static_box`, `collide.py:149-170` | confirmed |  |
| 12 | rise to 1.25 m first | flags `z_safe_m`, episode `transit_to` | confirmed |  |
| 13 | smooth starts/stops, 0.25 rad/s carry | `rig.py:115, :165`, episode `plan_to` | partly | 0.24 rad/s is the smoothstep PEAK; RRT paths are not smoothstepped |
| R1 | grasp gate slip < 5 mm -> stable after it settles | flags `hold_gate`; `hold_verdict` | confirmed | three gates coexisted (probe, grasp test, episode); now one rule per profile (D9): headline = drift <= 5 mm over 2 s |
| R2 | finger drive stiffness/effort changed; pad material added | flags `gripper_effort/stiffness`; `ur5e.pad_material` | confirmed | the gain change tripled the pad force (13 -> 41 N); only the material made no difference |
| R3 | upper rack pushed in while loading the lower rack | flags `upper_in_for_lower`, `reextend_before_scoring` | partly | declared scripted rack motion; both racks re-extended before scoring (D2); the old episode left them in/out |
| R4 | easy_s0 replaced by 3 upright bowls | flags `test_case` | confirmed | behind the flag, default off |
| R5 | lowered mouth-up until contact | flags `place_mode` | confirmed | behind the flag, default off (`goal_pose`) |
| R6 | success = in the rack, stable, nothing disturbed | flags `judge`, `disturbance`, `end_check`, `move_settle` | partly | 'stable' was never gated; headline = at-goal tolerance + benchmark settle + D1 disturbance + D2 end check |
| R7 | blocked lag limits widened to 0.06 / 0.08 | flags `tol_*`, `settled_lag_rad`, `lag_max_rad` | partly | widened only the post-move check; the legacy profile freezes the call-site values; headline = rig defaults (D8) |
| O1 | mouth-down bowls unreliable; lying bowl untested | `grasp.py` foot/side; records grasp_test/* | refuted (untested half) | D20(d): side grasp never executed in isolation (the 'lying' test rolled upright); attempted twice in the scene and failed |
| O2 | rear 12 cm of the lower rack under the counter; no pedestal reaches all goals | `grasp.py:86-91`, mount records | confirmed | the goal proxy was a TOP-DOWN grasp (H5 premise confirmed) |
| scene | 'pick bowls off a countertop' | easy_s0.json objects | partly | D20(e): 5 counter + 2 lower-rack starts (bowl_01 lying, bowl_07 upright) |
| time | '143 s simulated' (robot.md) | mp4 length | partly | D20(c): derived from the video, no logged field; every run now logs sim_s and wall_s separately |

### Decisions taken by the user during Phase 0

| date | item | decision |
|---|---|---|
| 2026-09-29 | D13 commit before Phase 0 | user: continue without a commit (a pre-Phase-0 snapshot went to artifacts/_baseline_pre_phase0/); the user committed the stack later (see the git status row) |
| 2026-09-29 | artifacts/ location | user: symlink onto the 2 TB drive, gitignored (root disk gains nothing) |
| 2026-09-30 | easy_s0 diagnostic run budget | user: allow up to 45 min (--max-wall-seconds 2700, the last 5 min for scoring) |
| 2026-09-30 | D4 and the knuckle contacts | user: keep D4 as decided (knuckle contact = auto-fail; Phase 2's grasps must keep the rim on the pads) |
| 2026-09-30 | D4 and the silverware basket | user: the basket counts as rack furniture for the carried dish (flagged interpretation kept) |

## 9. Hypotheses H1-H6 (Phase 1 measures; Phase 0 status)

| H | claim | status after Phase 0 |
|---|---|---|
| H1 | the arm lag is mostly gravity sag | mechanism off in config: gravity is disabled on every robot link (`ur5e.py:49`, labelled privileged, D6); Phase 1a measures the residual static error |
| H2 | the conversion dropped the 2F-85 linkage coupling | refuted at the asset level (five PhysxMimicJointAPI in the untouched payload); RUNTIME TESTED THIS PHASE, see the conventions section: all five couplings hold during a loaded close |
| H3 | the pad material had no effect for a mechanical reason | (a) refuted: the bowl authors no combine mode; (b) refuted: the material binds the live collider prims; (c) consistent with the records; Phase 1c measures |
| H4 | the bowl's physics model is wrong | refuted by the asset: mass authored 0.067375 kg (measured), 192 exact convex shells with open cavity and recess; Phase 1d verifies at runtime (D5) |
| H5 | the pedestal verdict assumed the old placement | confirmed by code (top-down goal proxy in the mount search); Phase 4 |
| H6 | some goals may be inaccessible to the bowl alone | untested; only the benchmark's sequence certificate exists; Phase 4 |

## 10. Process errors the harness makes impossible

| error | mechanism |
|---|---|
| a false pass from a sleeping body | sleep threshold 0 on every dish prim and robot link (`harness.author_sleep_and_reports`), `is_sleeping` polled every 12 ticks -> invariant `sleeping_body` |
| drive lag misread as the pads stalling | pad forces come from PhysX contact reports (normal component per pad collider), never from the drive lag; the jaw-angle filter stays a stand-in for object detection |
| a flipped approach sign | `test_approach_axis_points_from_the_wrist_to_the_fingertips` on the live asset (fixture) + `grasp.unit` guards |
| a NaN closing axis from two coincident points | `grasp.unit` raises on non-finite or degenerate axes before any division (`test_degenerate_axes_raise_before_use`) |
| a grasp test that teleported the arm into the bowl | every state write after the reset raises `teleport_after_reset` (backend `set_rigid_pose`/`restore` and `rig.teleport_arm` are guarded); analysis scripts that teleport are labelled ANALYSIS |
| a required side-grasp test that was skipped | the report lists every required test and marks absent ones FAIL (not run) |
| wall time labelled as sim time | every trial-log row carries `sim_s` (tick x 1/120 s) and `wall_s` (monotonic) as separate fields; the video overlay shows both |
| attempts counted as distinct loads | rows carry `attempt` and `config_id` (a digest of the reset poses) separately; the tables count attempts and configurations in their own columns |

## 11. Media and plots

### p0_prerepro_upright3
- page: `artifacts/p0_prerepro_upright3/index.html`

### p0_upright3_legacy
- page: `artifacts/p0_upright3_legacy/index.html`
- k_over_n: ![k_over_n](artifacts/p0_upright3_legacy/summary/k_over_n.png)
- settle_hist: ![settle_hist](artifacts/p0_upright3_legacy/summary/settle_hist.png)
- t00_nominal video: [`artifacts/p0_upright3_legacy/t00_nominal/video.mp4`](artifacts/p0_upright3_legacy/t00_nominal/video.mp4) (11.0 MB, 1280x720, 15 fps of simulated time)
- t00_nominal joint_tracking: ![joint_tracking](artifacts/p0_upright3_legacy/t00_nominal/plots/joint_tracking.png)
- t00_nominal pad_force: ![pad_force](artifacts/p0_upright3_legacy/t00_nominal/plots/pad_force.png)
- t00_nominal bowl_displacement: ![bowl_displacement](artifacts/p0_upright3_legacy/t00_nominal/plots/bowl_displacement.png)
- t00_nominal contact_timeline: ![contact_timeline](artifacts/p0_upright3_legacy/t00_nominal/plots/contact_timeline.png)
- t00_nominal key frames (39): ![001_start](artifacts/p0_upright3_legacy/t00_nominal/frames/001_start.png) ![002_pre-grasp](artifacts/p0_upright3_legacy/t00_nominal/frames/002_pre-grasp.png) ![003_invariant_carried_contact](artifacts/p0_upright3_legacy/t00_nominal/frames/003_invariant_carried_contact.png) ![004_close](artifacts/p0_upright3_legacy/t00_nominal/frames/004_close.png) ![005_invariant_carried_contact](artifacts/p0_upright3_legacy/t00_nominal/frames/005_invariant_carried_contact.png) ![006_invariant_carried_contact](artifacts/p0_upright3_legacy/t00_nominal/frames/006_invariant_carried_contact.png) ![007_invariant_carried_contact](artifacts/p0_upright3_legacy/t00_nominal/frames/007_invariant_carried_contact.png) ![008_lift-off](artifacts/p0_upright3_legacy/t00_nominal/frames/008_lift-off.png) ![009_insertion_start](artifacts/p0_upright3_legacy/t00_nominal/frames/009_insertion_start.png) ![010_invariant_carried_contact](artifacts/p0_upright3_legacy/t00_nominal/frames/010_invariant_carried_contact.png) ![011_release](artifacts/p0_upright3_legacy/t00_nominal/frames/011_release.png) ![012_after_settle](artifacts/p0_upright3_legacy/t00_nominal/frames/012_after_settle.png)

### p0_upright3_headline
- page: `artifacts/p0_upright3_headline/index.html`
- k_over_n: ![k_over_n](artifacts/p0_upright3_headline/summary/k_over_n.png)
- t00_nominal video: [`artifacts/p0_upright3_headline/t00_nominal/video.mp4`](artifacts/p0_upright3_headline/t00_nominal/video.mp4) (3.9 MB, 1280x720, 15 fps of simulated time)
- t00_nominal joint_tracking: ![joint_tracking](artifacts/p0_upright3_headline/t00_nominal/plots/joint_tracking.png)
- t00_nominal pad_force: ![pad_force](artifacts/p0_upright3_headline/t00_nominal/plots/pad_force.png)
- t00_nominal bowl_displacement: ![bowl_displacement](artifacts/p0_upright3_headline/t00_nominal/plots/bowl_displacement.png)
- t00_nominal contact_timeline: ![contact_timeline](artifacts/p0_upright3_headline/t00_nominal/plots/contact_timeline.png)
- t00_nominal key frames (6): ![001_start](artifacts/p0_upright3_headline/t00_nominal/frames/001_start.png) ![002_pre-grasp](artifacts/p0_upright3_headline/t00_nominal/frames/002_pre-grasp.png) ![003_invariant_carried_contact](artifacts/p0_upright3_headline/t00_nominal/frames/003_invariant_carried_contact.png) ![004_invariant_carried_contact](artifacts/p0_upright3_headline/t00_nominal/frames/004_invariant_carried_contact.png) ![005_finished](artifacts/p0_upright3_headline/t00_nominal/frames/005_finished.png) ![closeup_preclose_bowl_01](artifacts/p0_upright3_headline/t00_nominal/frames/closeup_preclose_bowl_01.png)

### p0v2_upright3_legacy
- page: `artifacts/p0v2_upright3_legacy/index.html`
- k_over_n: ![k_over_n](artifacts/p0v2_upright3_legacy/summary/k_over_n.png)
- settle_hist: ![settle_hist](artifacts/p0v2_upright3_legacy/summary/settle_hist.png)
- t00_nominal video: [`artifacts/p0v2_upright3_legacy/t00_nominal/video.mp4`](artifacts/p0v2_upright3_legacy/t00_nominal/video.mp4) (11.1 MB, 1280x720, 15 fps of simulated time)
- t00_nominal joint_tracking: ![joint_tracking](artifacts/p0v2_upright3_legacy/t00_nominal/plots/joint_tracking.png)
- t00_nominal pad_force: ![pad_force](artifacts/p0v2_upright3_legacy/t00_nominal/plots/pad_force.png)
- t00_nominal bowl_displacement: ![bowl_displacement](artifacts/p0v2_upright3_legacy/t00_nominal/plots/bowl_displacement.png)
- t00_nominal contact_timeline: ![contact_timeline](artifacts/p0v2_upright3_legacy/t00_nominal/plots/contact_timeline.png)
- t00_nominal key frames (32): ![001_start](artifacts/p0v2_upright3_legacy/t00_nominal/frames/001_start.png) ![002_pre-grasp](artifacts/p0v2_upright3_legacy/t00_nominal/frames/002_pre-grasp.png) ![003_invariant_carried_contact](artifacts/p0v2_upright3_legacy/t00_nominal/frames/003_invariant_carried_contact.png) ![004_close](artifacts/p0v2_upright3_legacy/t00_nominal/frames/004_close.png) ![005_invariant_carried_contact](artifacts/p0v2_upright3_legacy/t00_nominal/frames/005_invariant_carried_contact.png) ![006_lift-off](artifacts/p0v2_upright3_legacy/t00_nominal/frames/006_lift-off.png) ![007_insertion_start](artifacts/p0v2_upright3_legacy/t00_nominal/frames/007_insertion_start.png) ![008_release](artifacts/p0v2_upright3_legacy/t00_nominal/frames/008_release.png) ![009_after_settle](artifacts/p0v2_upright3_legacy/t00_nominal/frames/009_after_settle.png) ![010_pre-grasp](artifacts/p0v2_upright3_legacy/t00_nominal/frames/010_pre-grasp.png) ![011_invariant_carried_contact](artifacts/p0v2_upright3_legacy/t00_nominal/frames/011_invariant_carried_contact.png) ![012_close](artifacts/p0v2_upright3_legacy/t00_nominal/frames/012_close.png)

### p0v2_upright3_legacy_nosleep
- page: `artifacts/p0v2_upright3_legacy_nosleep/index.html`
- k_over_n: ![k_over_n](artifacts/p0v2_upright3_legacy_nosleep/summary/k_over_n.png)
- settle_hist: ![settle_hist](artifacts/p0v2_upright3_legacy_nosleep/summary/settle_hist.png)
- t00_nominal video: [`artifacts/p0v2_upright3_legacy_nosleep/t00_nominal/video.mp4`](artifacts/p0v2_upright3_legacy_nosleep/t00_nominal/video.mp4) (11.1 MB, 1280x720, 15 fps of simulated time)
- t00_nominal joint_tracking: ![joint_tracking](artifacts/p0v2_upright3_legacy_nosleep/t00_nominal/plots/joint_tracking.png)
- t00_nominal pad_force: ![pad_force](artifacts/p0v2_upright3_legacy_nosleep/t00_nominal/plots/pad_force.png)
- t00_nominal bowl_displacement: ![bowl_displacement](artifacts/p0v2_upright3_legacy_nosleep/t00_nominal/plots/bowl_displacement.png)
- t00_nominal contact_timeline: ![contact_timeline](artifacts/p0v2_upright3_legacy_nosleep/t00_nominal/plots/contact_timeline.png)
- t00_nominal key frames (40): ![001_start](artifacts/p0v2_upright3_legacy_nosleep/t00_nominal/frames/001_start.png) ![002_invariant_sleeping_body](artifacts/p0v2_upright3_legacy_nosleep/t00_nominal/frames/002_invariant_sleeping_body.png) ![003_invariant_sleeping_body](artifacts/p0v2_upright3_legacy_nosleep/t00_nominal/frames/003_invariant_sleeping_body.png) ![004_pre-grasp](artifacts/p0v2_upright3_legacy_nosleep/t00_nominal/frames/004_pre-grasp.png) ![005_invariant_sleeping_body](artifacts/p0v2_upright3_legacy_nosleep/t00_nominal/frames/005_invariant_sleeping_body.png) ![006_invariant_carried_contact](artifacts/p0v2_upright3_legacy_nosleep/t00_nominal/frames/006_invariant_carried_contact.png) ![007_close](artifacts/p0v2_upright3_legacy_nosleep/t00_nominal/frames/007_close.png) ![008_invariant_carried_contact](artifacts/p0v2_upright3_legacy_nosleep/t00_nominal/frames/008_invariant_carried_contact.png) ![009_lift-off](artifacts/p0v2_upright3_legacy_nosleep/t00_nominal/frames/009_lift-off.png) ![010_insertion_start](artifacts/p0v2_upright3_legacy_nosleep/t00_nominal/frames/010_insertion_start.png) ![011_release](artifacts/p0v2_upright3_legacy_nosleep/t00_nominal/frames/011_release.png) ![012_after_settle](artifacts/p0v2_upright3_legacy_nosleep/t00_nominal/frames/012_after_settle.png)

### p0v2_upright3_headline
- page: `artifacts/p0v2_upright3_headline/index.html`
- k_over_n: ![k_over_n](artifacts/p0v2_upright3_headline/summary/k_over_n.png)
- t00_nominal video: [`artifacts/p0v2_upright3_headline/t00_nominal/video.mp4`](artifacts/p0v2_upright3_headline/t00_nominal/video.mp4) (3.9 MB, 1280x720, 15 fps of simulated time)
- t00_nominal joint_tracking: ![joint_tracking](artifacts/p0v2_upright3_headline/t00_nominal/plots/joint_tracking.png)
- t00_nominal pad_force: ![pad_force](artifacts/p0v2_upright3_headline/t00_nominal/plots/pad_force.png)
- t00_nominal bowl_displacement: ![bowl_displacement](artifacts/p0v2_upright3_headline/t00_nominal/plots/bowl_displacement.png)
- t00_nominal contact_timeline: ![contact_timeline](artifacts/p0v2_upright3_headline/t00_nominal/plots/contact_timeline.png)
- t00_nominal key frames (5): ![001_start](artifacts/p0v2_upright3_headline/t00_nominal/frames/001_start.png) ![002_pre-grasp](artifacts/p0v2_upright3_headline/t00_nominal/frames/002_pre-grasp.png) ![003_invariant_carried_contact](artifacts/p0v2_upright3_headline/t00_nominal/frames/003_invariant_carried_contact.png) ![004_finished](artifacts/p0v2_upright3_headline/t00_nominal/frames/004_finished.png) ![closeup_preclose_bowl_01](artifacts/p0v2_upright3_headline/t00_nominal/frames/closeup_preclose_bowl_01.png)

### p0v2_upright3_headline_diag
- page: `artifacts/p0v2_upright3_headline_diag/index.html`
- k_over_n: ![k_over_n](artifacts/p0v2_upright3_headline_diag/summary/k_over_n.png)
- settle_hist: ![settle_hist](artifacts/p0v2_upright3_headline_diag/summary/settle_hist.png)
- t00_nominal video: [`artifacts/p0v2_upright3_headline_diag/t00_nominal/video.mp4`](artifacts/p0v2_upright3_headline_diag/t00_nominal/video.mp4) (11.4 MB, 1280x720, 15 fps of simulated time)
- t00_nominal joint_tracking: ![joint_tracking](artifacts/p0v2_upright3_headline_diag/t00_nominal/plots/joint_tracking.png)
- t00_nominal pad_force: ![pad_force](artifacts/p0v2_upright3_headline_diag/t00_nominal/plots/pad_force.png)
- t00_nominal bowl_displacement: ![bowl_displacement](artifacts/p0v2_upright3_headline_diag/t00_nominal/plots/bowl_displacement.png)
- t00_nominal contact_timeline: ![contact_timeline](artifacts/p0v2_upright3_headline_diag/t00_nominal/plots/contact_timeline.png)
- t00_nominal key frames (45): ![001_start](artifacts/p0v2_upright3_headline_diag/t00_nominal/frames/001_start.png) ![002_pre-grasp](artifacts/p0v2_upright3_headline_diag/t00_nominal/frames/002_pre-grasp.png) ![003_invariant_carried_contact](artifacts/p0v2_upright3_headline_diag/t00_nominal/frames/003_invariant_carried_contact.png) ![004_close](artifacts/p0v2_upright3_headline_diag/t00_nominal/frames/004_close.png) ![005_invariant_carried_contact](artifacts/p0v2_upright3_headline_diag/t00_nominal/frames/005_invariant_carried_contact.png) ![006_lift-off](artifacts/p0v2_upright3_headline_diag/t00_nominal/frames/006_lift-off.png) ![007_fail_place_bowl_01_0](artifacts/p0v2_upright3_headline_diag/t00_nominal/frames/007_fail_place_bowl_01_0.png) ![008_after_settle](artifacts/p0v2_upright3_headline_diag/t00_nominal/frames/008_after_settle.png) ![009_pre-grasp](artifacts/p0v2_upright3_headline_diag/t00_nominal/frames/009_pre-grasp.png) ![010_close](artifacts/p0v2_upright3_headline_diag/t00_nominal/frames/010_close.png) ![011_lift-off](artifacts/p0v2_upright3_headline_diag/t00_nominal/frames/011_lift-off.png) ![012_fail_place_bowl_01_1](artifacts/p0v2_upright3_headline_diag/t00_nominal/frames/012_fail_place_bowl_01_1.png)

### p0v2_easy_s0_headline
- page: `artifacts/p0v2_easy_s0_headline/index.html`
- k_over_n: ![k_over_n](artifacts/p0v2_easy_s0_headline/summary/k_over_n.png)
- t00_nominal video: [`artifacts/p0v2_easy_s0_headline/t00_nominal/video.mp4`](artifacts/p0v2_easy_s0_headline/t00_nominal/video.mp4) (3.1 MB, 1280x720, 15 fps of simulated time)
- t00_nominal joint_tracking: ![joint_tracking](artifacts/p0v2_easy_s0_headline/t00_nominal/plots/joint_tracking.png)
- t00_nominal pad_force: ![pad_force](artifacts/p0v2_easy_s0_headline/t00_nominal/plots/pad_force.png)
- t00_nominal bowl_displacement: ![bowl_displacement](artifacts/p0v2_easy_s0_headline/t00_nominal/plots/bowl_displacement.png)
- t00_nominal contact_timeline: ![contact_timeline](artifacts/p0v2_easy_s0_headline/t00_nominal/plots/contact_timeline.png)
- t00_nominal key frames (2): ![001_start](artifacts/p0v2_easy_s0_headline/t00_nominal/frames/001_start.png) ![002_finished](artifacts/p0v2_easy_s0_headline/t00_nominal/frames/002_finished.png)

### p0v2_easy_s0_headline_diag
- page: `artifacts/p0v2_easy_s0_headline_diag/index.html`
- k_over_n: ![k_over_n](artifacts/p0v2_easy_s0_headline_diag/summary/k_over_n.png)
- settle_hist: ![settle_hist](artifacts/p0v2_easy_s0_headline_diag/summary/settle_hist.png)
- t00_nominal video: [`artifacts/p0v2_easy_s0_headline_diag/t00_nominal/video.mp4`](artifacts/p0v2_easy_s0_headline_diag/t00_nominal/video.mp4) (11.2 MB, 1280x720, 15 fps of simulated time)
- t00_nominal joint_tracking: ![joint_tracking](artifacts/p0v2_easy_s0_headline_diag/t00_nominal/plots/joint_tracking.png)
- t00_nominal pad_force: ![pad_force](artifacts/p0v2_easy_s0_headline_diag/t00_nominal/plots/pad_force.png)
- t00_nominal bowl_displacement: ![bowl_displacement](artifacts/p0v2_easy_s0_headline_diag/t00_nominal/plots/bowl_displacement.png)
- t00_nominal contact_timeline: ![contact_timeline](artifacts/p0v2_easy_s0_headline_diag/t00_nominal/plots/contact_timeline.png)
- t00_nominal key frames (34): ![001_start](artifacts/p0v2_easy_s0_headline_diag/t00_nominal/frames/001_start.png) ![002_pre-grasp](artifacts/p0v2_easy_s0_headline_diag/t00_nominal/frames/002_pre-grasp.png) ![003_close](artifacts/p0v2_easy_s0_headline_diag/t00_nominal/frames/003_close.png) ![004_lift-off](artifacts/p0v2_easy_s0_headline_diag/t00_nominal/frames/004_lift-off.png) ![005_fail_place_bowl_07_0](artifacts/p0v2_easy_s0_headline_diag/t00_nominal/frames/005_fail_place_bowl_07_0.png) ![006_after_settle](artifacts/p0v2_easy_s0_headline_diag/t00_nominal/frames/006_after_settle.png) ![007_pre-grasp](artifacts/p0v2_easy_s0_headline_diag/t00_nominal/frames/007_pre-grasp.png) ![008_close](artifacts/p0v2_easy_s0_headline_diag/t00_nominal/frames/008_close.png) ![009_invariant_carried_contact](artifacts/p0v2_easy_s0_headline_diag/t00_nominal/frames/009_invariant_carried_contact.png) ![010_lift-off](artifacts/p0v2_easy_s0_headline_diag/t00_nominal/frames/010_lift-off.png) ![011_fail_place_bowl_07_1](artifacts/p0v2_easy_s0_headline_diag/t00_nominal/frames/011_fail_place_bowl_07_1.png) ![012_after_settle](artifacts/p0v2_easy_s0_headline_diag/t00_nominal/frames/012_after_settle.png)

## 12. Open problems and questions for the user

- The plan's D4 invariant fails the only passing grasp: the rim pinch loads the inner knuckles (section 6); user ruling 2026-09-30: D4 stays, Phase 2's grasps must keep the rim on the pads (opening = wall + 10-15 mm per side, the plan's G1).
- Under the pre-R7 lag limits (D8) every carry of the 3-bowl case is judged blocked at the top of the rise: wrist_3 lags 46-49 mrad while a bowl is held (gravity is off on the links, so it is not sag) against the 30 mrad settled limit; Phase 1a's contact-first blocked detection is the planned remedy.
- After a failed transport the episode releases the dish where the hand is (up to 30 cm above the counter), as the old episode did; a lowered abort release is a control change for Phase 2+, not Phase 0.
- D11 (asset script): `assets/robots/MANIFEST.sha256` is written from the files present; the usd-core version that produced the converted layers is unrecorded and cannot be verified without a re-download (which D11 forbids).
- D13: git HEAD `c1f606f 2026-09-30 add robot stack`; robot files modified since HEAD ['frigidaire/docs/robot.md', 'frigidaire/scripts/evaluation/frigidaire_robot_report.py', 'frigidaire/scripts/experiment/frigidaire_robot_episode.py', 'frigidaire/scripts/setup/mirror_robot_usd.sh', 'frigidaire/src/dishsim_frigidaire/robot/harness.py'], untracked ['frigidaire/scripts/setup/robot_asset_manifest.py', 'frigidaire/tests/test_robot_harness.py'] (the user commits; see the summary's suggested message).
- Phase 1 (1a-1e) has not started; every hypothesis row above is a code/asset status, not a measurement, except H2's runtime test.

## Machine constraints

Safety: one container (`dishsim-isaac`) on GPU 1, at most two Kit jobs staggered 90 s with a 6 GB free check, jobs stopped by recorded PID only, nothing outside this repo touched. Privacy: everything on the box (results/robot, artifacts/, logs/robot on the 2 TB drive); no uploads. Space: the root disk gained nothing (artifacts/ is a symlink onto the 2 TB drive; Kit's per-run log is redirected there with `--/log/file`).
