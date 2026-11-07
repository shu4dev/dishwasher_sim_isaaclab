#!/usr/bin/env python3
"""A UR5e + Robotiq 2F-85 executes an instance's moves in the Frigidaire twin (no teleport, no weld), under the
Phase 0 harness of plans/2026-09-29-easy-s0.md.

    code/util/run_kit.sh code/execution/frigidaire/frigidaire_robot_episode.py --headless --enable_cameras \\
        --instance data/results/benchmark/frigidaire_hotec/instances/easy/easy_s0.json --profile headline --run-id <id>

Profiles (code/frigidaire/src/dishsim_frigidaire/robot/flags.py): ``headline`` = every benchmark relaxation off, the
benchmark's disturbance rule (D1), end check (D2) and per-move settle (D3), the counter cap (D19), the rig's pre-R7
limits (D8) and the drift hold gate (D9); ``legacy_upright3`` = the 2026-09-29 upright3 PASS frozen (3 upright
bowls, lowered until contact, judged racked-in; the regression test's old settings). ``--test-case upright3`` runs
the 3-bowl case under any profile (R4); ``--diagnostic-continue`` (D18, never headline) skips a dish after its two
attempts instead of aborting and records invariant violations without stopping, so one run yields a cause per bowl.

The instance start is rebuilt as in the benchmark (frigidaire_bench_kit.py reset). The arm stands on the pedestal of
the profile's mount file (D17). Sequence (user decisions 2026-09-29): phase L with the UPPER rack pushed in (R3, a
declared scripted rack motion: the extended upper rack covers the lower rack): dishes whose goal is in the upper
rack are parked on the counter first (top of a stack first), then every lower-rack goal is loaded in the goal's
lean-on order; the upper rack is pulled out; phase U loads it. Each move: lift to a safe height, joint move above the
dish, straight line down, close (pads only), lift, the in-hand hold gate, move above the target, line down, open,
retreat, settle; one re-grasp per move.

Every physics step goes through the harness (code/frigidaire/src/dishsim_frigidaire/robot/harness.py): PhysX contact
reports, auto-fail invariants, the JSONL trial log (simulated and wall time as separate fields), a fixed-camera
1280x720 video with the simulated time overlaid, key frames. Outputs: data/results/robot/runs/<run-id>/<instance>.json
(the record) and data/artifacts/<run-id>/<trial-id>/ (trial.jsonl, trial_physics.jsonl, video.mp4, frames/).
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import sys
import time
import traceback

ROOT = Path(__file__).resolve().parents[3]
RUNS = ROOT / "data/results/robot/runs"        # one folder per run (plan D15); the pre-Phase-0 records stay in episodes/
ARTIFACTS = ROOT / "data/artifacts"            # per-run media/logs; a symlink onto the 2 TB drive (user 2026-09-29)
ROBOT_CAMERA = {"robot": ((-1.9, -2.7, 1.95), (.05, -.25, .62), {"focal_length": 42., "horizontal_aperture": 36.})}
VIDEO_HW = (720, 1280)                    # fixed camera, 1280 x 720 (user 2026-09-29)
VIDEO_FPS = 15                            # one frame per 8 physics steps at 120 Hz = real-time simulated seconds
DISTURB_MM, DISTURB_DEG = 10., 20.        # the any-neighbour rule's thresholds (= rearrange.DISTURB_POS_M / _ROT_DEG)
ALLOW_CONTACT_M = .06                     # planner: the grasped dish may touch the gripper in the last 60 mm (fix 10)
ALLOW_STRADDLE_M = .12                    # wall / base pinch: the open pads straddle the cup ~7 mm off each side, inside the
                                          # 15 mm margin over the whole 0.10 m approach: the gripper may touch it all along
SCORING_GRACE_S = 300.                    # of --max-wall-seconds, kept for the end section (final hold, end check)
CODE_FILES = ("code/execution/frigidaire/frigidaire_robot_episode.py",
              "code/frigidaire/src/dishsim_frigidaire/robot/flags.py", "code/frigidaire/src/dishsim_frigidaire/robot/harness.py",
              "code/frigidaire/src/dishsim_frigidaire/robot/triallog.py", "code/frigidaire/src/dishsim_frigidaire/robot/rig.py",
              "code/frigidaire/src/dishsim_frigidaire/robot/ur5e.py", "code/frigidaire/src/dishsim_frigidaire/robot/grasp.py",
              "code/frigidaire/src/dishsim_frigidaire/robot/collide.py", "code/frigidaire/src/dishsim_frigidaire/robot/kin.py",
              "code/planner/frigidaire/frigidaire_bench.py", "code/planner/frigidaire/frigidaire_bench_kit.py",
              "code/frigidaire/src/dishsim_frigidaire/initial_state_runtime.py")


def code_hashes():
    return {f: hashlib.sha256((ROOT / f).read_bytes()).hexdigest()[:16] for f in CODE_FILES}


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--instance", type=Path, default=None, help="required unless --grasp-gate / --place-slot")
    parser.add_argument("--grasp-gate", choices=("bowl", "cup", "plate"), default=None,
                        help="limits round (plans/2026-10-23-hardware-limits-mouth-up.md, Experiment 1): one mouth-up dish on "
                             "the counter, --n trials of pick (one candidate), lift 0.15 m, hold 5 s (G3)")
    parser.add_argument("--place-slot", choices=("bowl", "cup", "plate"), default=None,
                        help="limits round, Experiment 2 (G4): pick from the counter, place at each --sites site of --reach, settle")
    parser.add_argument("--n", type=int, default=10, help="trials per object (grasp gate) or per site (place-slot)")
    parser.add_argument("--seed", type=int, default=0, help="seed of the +-5 mm counter-pose perturbation")
    parser.add_argument("--sites", default="", help="place-slot: comma-separated site names of --reach")
    parser.add_argument("--site-n", default="", help="place-slot: per-site trial counts name=n,... (overrides --n there)")
    parser.add_argument("--dump-contacts", action="store_true",
                        help="grasp gate: per tick from the first gripper contact on, every contact partner of the dish "
                             "(collider paths, normal force) and the gap between each gripper knuckle's VISUAL mesh and the "
                             "dish's visual mesh (FCL), to dump_contacts.jsonl (the knuckle-contact check)")
    parser.add_argument("--counter-grid", type=Path, default=None,
                        help="grasp gate: pick spots from frigidaire_robot_reach.py --counter-grid's counter_grid.json, the "
                             "3x3 subset (i, j in 0, 2, 4), --n trials per spot")
    parser.add_argument("--grid-g2-only", action="store_true", help="--counter-grid: only the spots with a Kit-free G2 path")
    parser.add_argument("--grid-part", default="0/1", help="--counter-grid: k/m runs every m-th spot from the k-th (split runs)")
    parser.add_argument("--out-root", type=Path, default=None, help="limits records root (default data/results/robot/limits_v1)")
    parser.add_argument("--target", choices=("family", "level", "tilt"), default="family",
                        help="place-slot target: the packer's pose at the site, or the dish as held (mouth up, level) at the "
                             "site's clear height (reach.json world pose)")
    parser.add_argument("--reach", type=Path, default=ROOT / "data/results/robot/limits_v1/reach.json")
    parser.add_argument("--waive-dish-clearance", action="store_true",
                        help="limits round, plate: the approach line is NOT collision-checked and the hover ignores the dish "
                             "(the planner's 15 mm margin rejects every plate approach) so the PHYSICAL pinch is measured; "
                             "contacts stay recorded by the harness")
    parser.add_argument("--release", choices=("air", "down"), default="air",
                        help="place-slot release: air = at the target pose (place()); down = the upright3 rule (place_down(): "
                             "lowered as it hangs to the target x, y until contact, then opened); G4 is then judged against "
                             "the dish pose at the release")
    parser.add_argument("--plate-slide", action="store_true",
                        help="limits round 3, grasp gate: Voysey 2021's surface slide on the plate (jaws horizontal, lower "
                             "finger 2 mm above the counter, slide 40 mm toward the centre, close, lift) with the 15 mm "
                             "clearance check on; only the gripper may touch the counter and the plate on the descent and slide")
    parser.add_argument("--contact-lag", type=float, default=None,
                        help="place_down's lag threshold read as contact (flag place_down_contact_lag, default 0.05)")
    parser.add_argument("--down-orient", choices=("hang", "target"), default="hang",
                        help="down release: lower the dish as it hangs (round 2) or at the target orientation (round 3)")
    parser.add_argument("--orient", type=Path, default=ROOT / "data/results/robot/limits_v1/orient.json",
                        help="--target tilt: the orientation sweep's cells (frigidaire_robot_reach.py --orient)")
    parser.add_argument("--tilts", default="0,30,60,90,120,150",
                        help="--target tilt: tilts tried per site in this order; a site stops at its first failing tilt")
    parser.add_argument("--tilts-adaptive", action="store_true",
                        help="--target tilt: a site's climb stops at its first Isaac 0/n tilt (the site's limit)")
    parser.add_argument("--yaw-retry", default="",
                        help="place-slot level: reach.jsonl yaw indices tried in this order, --n trials each, the target "
                             "turned about the dish axis so the TCP sits at counter grasp k's yaw (2 pi k / 12); the first "
                             "yaw that releases is topped up to --yaw-top trials and ends the site")
    parser.add_argument("--yaw-top", type=int, default=10, help="--yaw-retry: trials at the first yaw that releases")
    parser.add_argument("--plate-in-hand", action="store_true",
                        help="place-slot plate: the plate spawned in the gripper at --orient's measured in-hand pose (the "
                             "jaws close on it), no grasp; targets = --orient's cells (frigidaire_robot_reach.py --plate-in-hand)")
    parser.add_argument("--presentation", type=Path, default=None,
                        help="plans/2026-11-04-plate-pick-presentation.md (amended): a plate presentation json (frigidaire_robot_"
                             "reach.py --sample-gate writes <plates load dir>/presentation.json): the plate spawned leaning on its "
                             "backstop (a static box authored at /World/Backstop and in the arm model), picked by --grasp-family "
                             "edge; a samples_v1 load's dish names it in its pick block")
    parser.add_argument("--grasp-family", choices=("rim", "wall", "side", "foot-out", "foot-in", "foot-side", "edge", "plate-wall"),
                        default="rim",
                        help="grasp gate: wall = the cup wall pinch (robot/grasp.py wall_grasps, 2026-10-26) instead of the "
                             "mouth-up rim pinch; side = the horizontal cup side pinch (side_grasps, lifted straight up); "
                             "foot-out / foot-in (plans/2026-10-29-foot-ring-grasp.md, bowl) = the bowl MOUTH DOWN on the "
                             "counter, the TCP down its axis: the open pads close on the foot ring's outer face (out) or the "
                             "closed fingertips enter the recess and OPEN on its inner face (in), grasp.foot_grasps at the best "
                             "yaw of --foot-candidates; plate-wall (plans/2026-11-04-plate-wall-pinch.md, with --presentation) = "
                             "the plate's wall-normal pinch of the presentation's wall block (grasp.plate_wall_grasps), lifted "
                             "straight up; foot-side (plans/2026-11-04-ring-side-hold.md, bowl) = the side ring hold: the bowl mouth "
                             "down, the approach horizontal at the ring's height, the pads on the ring's outer face "
                             "(grasp.foot_side_grasps) in the Kit-free gate's order, lifted straight up")
    parser.add_argument("--foot-candidates", type=Path, default=None,
                        help="--grasp-family foot-*: frigidaire_robot_reach.py --foot-candidates' output (the best yaw per family; "
                             "default placing_v1/foot/candidates.json); foot-side: a gate.json with gate_pick (default "
                             "FOOT_SIDE_GATE_JSON) or a --foot-families side output, its ranked [yaw_k, wrist_side] pairs")
    parser.add_argument("--trial-offset", type=int, default=0,
                        help="grasp gate: number the trials from this index and skip that many +-5 mm draws first, so a run "
                             "split over processes keeps the perturbations of one --n run (trials offset .. offset + n - 1)")
    parser.add_argument("--side-below-top", type=float, default=.011,
                        help="--grasp-family side: the pad centres this far under the rim (m; one height per gate run)")
    parser.add_argument("--side-wrist-side", choices=("mouth", "base"), default=None,
                        help="--grasp-family side: only this wrist side (grasp.WRIST_SIDES; default both)")
    parser.add_argument("--mouth-down", action="store_true",
                        help="--grasp-gate cup --grasp-family side (plans/2026-11-04-cups-mouth-down.md): the cup spawned mouth "
                             "down at the counter spot (its rim 2 mm above the slab)")
    parser.add_argument("--counter-front-y", type=float, default=-.17,
                        help="the counter slab's front edge y (m; plans/2026-10-27-cup-side-drop.md: -0.17 for every robot run "
                             "from 2026-10-27, the benchmark's slab -0.30); back edge and top unchanged")
    parser.add_argument("--counter-y", type=float, default=None,
                        help="the counter spot's y (m; the spot's x kept): frigidaire_robot_reach.py --side-spot. The spot "
                             "(plans/2026-11-04-fifteen-of-fifteen.md, counter_spot): the mount file's best.spot_xy when it has "
                             "one, else the --place-goal load's pick_spot_xy, else LIMITS_COUNTER_XY (0.20, -0.12); every dish "
                             "is spawned and set down there, a --presentation is moved with it")
    parser.add_argument("--place-goal", type=Path, default=None,
                        help="plans/2026-10-26-placing-goal.md Part B: a placing_v1/<load>/load.json; per dish in its order "
                             "the earlier dishes at their goal poses, this dish fixed to the gripper (a fixed joint) at the "
                             "measured in-hand pose, steps S1-S6 (first failure wins), rows to <load>/dishes.jsonl")
    parser.add_argument("--regrasp", action="store_true",
                        help="--place-goal variant R for every dish whose W row failed at S1 or S2: flip and set down mouth "
                             "down on the counter spot (R1), re-pick from above by the foot ring / across the cup's base "
                             "(R2), re-attach, then S1-S6")
    parser.add_argument("--regrasp-ideal", action="store_true",
                        help="--place-goal variant R-ideal for the same dishes: the dish TELEPORTED mouth down onto the counter "
                             "spot and settled (an idealized R1: no rim / wall / rim-hold set-down exists), then R2 and S1-S6")
    parser.add_argument("--settle-only", action="store_true",
                        help="--place-goal: the subset settle check only (every dish at its goal, 150 ticks)")
    parser.add_argument("--dishes", default="", help="--place-goal: only these dishes (comma list)")
    parser.add_argument("--variant", choices=("S", "D", "I"), default=None,
                        help="--place-goal variant S (plans/2026-10-26-side-pinch.md): every cup held by the side pinch at the "
                             "measured in-hand pose of its best --side-candidates candidate, then S1-S6; D "
                             "(plans/2026-10-27-drop-test.md): the dish carried as held to the lowest collision-free pose above "
                             "its goal and let go, the earlier dishes never reset; I (plans/2026-10-27-bowls-plates-down.md, "
                             "plates): the plate welded at an assumed top-edge rim hold, carried at its goal orientation above "
                             "its goal, lowered in 5 mm steps until it touches, is blocked or reaches the goal height, let go")
    parser.add_argument("--hold", choices=("as-held", "side-base", "rim-flip", "rim-lip", "foot-out", "side-mouth", "side-down"),
                        default="as-held",
                        help="--variant D: as-held = the counter hold kept (the drop test); side-base (plans/2026-10-27-cup-side-"
                             "drop.md, cups) = the base-side pinch's gate median hold (placing_v1/partA_cup_side_base45), the "
                             "drop pose its hold flipped (cup axis down, TCP horizontal); rows variant D-side; rim-flip "
                             "(plans/2026-10-27-bowls-plates-down.md, bowls) = the bowl gate's median rim pinch turned about its "
                             "closing axis to the smallest mouth-down tilt whose first 50 mm of fall clears the gripper; rows D-flip; "
                             "rim-lip (plans/2026-10-27-bowls-lip-plates-fix.md, bowls) = that rim pinch carried at the goal's exact "
                             "axis with the pinched rim point at the lip (the rim point highest at the goal), spins 0, +-15, +-30 deg "
                             "about the bowl axis; rows D-lip; side-mouth (plans/2026-11-04-grasp-place-s0.md, cups, the coordinator's "
                             "option B) = the 45 mm mouth-side side pinch as picked, the drop pose the hold turned mouth down "
                             "(TCP x up, the wrist below the cup); rows D-side-mouth; side-down (plans/2026-11-04-cups-mouth-"
                             "down.md, cups, a samples_v1 load's move D-side-down) = the cup picked mouth down by the side pinch "
                             "60 mm under the rim with the wrist toward the base (above the cup), carried mouth down as held, the "
                             "D-ring descent (lowered straight down until the flange sensor trips), the jaws opened, the hand "
                             "backed CUP_BACK_M along -TCP z, then the D-ring rise; the cups' seated test; rows D-side-down")
    parser.add_argument("--goals", type=Path, default=None,
                        help="--place-goal: refreshed goal poses (frigidaire_drop_curve.py --refresh-goals' goals_v4.json, the goal "
                             "records' frames) replacing the load's goal poses (plans/2026-10-29-ring-place.md)")
    parser.add_argument("--foot-hold", type=Path, default=ROOT / "data/results/robot/placing_v1/partA_bowl_foot_out/inhand.json",
                        help="--hold foot-out: the F-out gate's in-hand summary (M_tcp_to_bowl of its median-pivot trial)")
    parser.add_argument("--foot-goal", type=Path, default=ROOT / "data/results/robot/kitfree_v1/foot_goal/foot_goal.json",
                        help="--no-reach: the Kit-free goal table whose F-out goal-pose row gives each listed bowl's blocker")
    parser.add_argument("--no-reach", default="",
                        help="--hold foot-out: these bowls (comma list) get 'no reach' rows with the Kit-free blocker and are placed "
                             "at their (refreshed) goals at their turn")
    parser.add_argument("--extra-row", default="",
                        help="--hold foot-out: one more flagged row per 'dish:arm_margin_m' (comma list) at the dish's own turn, "
                             "right after its no-reach row, the scene as it is then (the upper rack pushed in, else both racks out "
                             "when only that passes Kit-free); the dish then teleported to its goal (plans/2026-10-29-ring-place.md "
                             "+ the coordinator 2026-10-29: bowl_06:0.010)")
    parser.add_argument("--bowl-supports", type=Path, default=None,
                        help="the bowl seated test's goal_supports json (default kitfree_v1/insert_bowl/goal_supports.json)")
    parser.add_argument("--release-height-mm", default=None,
                        help="--hold foot-out (plans/2026-10-30-bowl-drop100.md, rows D-ring-h100): the jaws open this far above "
                             "the refreshed goal (mm, the bowl centroid along the rack's up axis), as the cups are released: "
                             "'100' for every bowl, or '100,bowl_01:50' (the default first, then per-bowl values); the Kit-free "
                             "plan's descent line ends there and starts RING_ABOVE_M above it; the descent stops there by "
                             "position (the flange sensor still ends it on an earlier contact, recorded), then the open, 0.5 s, "
                             "the 100 mm rise and the judgement as D-ring; a --no-reach bowl's why is the Kit-free release-height "
                             "sweep's row at its height (kitfree_v1/release_height/table.json)")
    parser.add_argument("--carry", choices=("nearest", "overlean", "seat", "above"), default=None,
                        help="--hold rim-lip (plans/2026-10-27-carry-rules.md, rows D-lip2): before the arm moves, a Kit-free "
                             "search over every free home branch x spin (-45..45 deg, 5 deg steps) x elbow-up IK branch of the drop "
                             "pose; the lowest free height first, there the smallest largest-joint travel over transit_to's two legs "
                             "(start -> the drop pose at the safe height -> the drop pose), ties the sum; straight collision-free legs "
                             "first; the arm starts at that combo's home branch and drops from its height and spin. overlean "
                             "(plans/2026-10-28-perch-release.md, rows D-perch): the bowl carried tilted steeper than its goal "
                             "(turned about the lip's horizontal rim tangent) so it touches the rack before the hand; a Kit-free "
                             "combo search (tilt x spin x home branch x IK branch; bowl first, then a mouth-down static roll, the "
                             "lowest perch, the smallest largest-joint travel); carried to 50 mm above the perch, lowered at 20 mm/s "
                             "until the emulated flange force sensor trips (> 3.5 N on 2 ticks), let go there, the hand backing "
                             "out along its approach (the I3 retreat). seat (plans/2026-10-29-bowl-last.md, rows D-seat): the combo "
                             "search over tilt (the goal lean to 90) x spin x home x IK ranked by the predicted stop's height above "
                             "the seat (bowl stops before hand stops, then the mouth-down roll, then travel), the 2 mm descent "
                             "margin for the links, the pre-descent settle and the stop re-predicted from the measured pose, the "
                             "sensed descent, the hand-away retreat before the settle, the seated test on the goal's wires")
    parser.add_argument("--insert", choices=("seat", "sampler"), default=None,
                        help="--variant I (plans/2026-10-28-plate-seat.md, rows I4): the plate held on its BASE face at the "
                             "assumed top-edge hold with --rim-depth-m; per plate a Kit-free seat (the plate alone lowered along "
                             "the tine axis, 1 mm toward the basket, the earlier plates where they landed); carried to 50 mm above "
                             "it, lowered along the tine axis at 20 mm/s until the emulated flange sensor trips (> 3.5 N on 2 "
                             "ticks; bounded 30 mm under the seat), let go there, the I3 retreat, the settle, the seated test. "
                             "sampler (plans/2026-10-29-sampler-release.md; --variant I plates = rows I5, --variant D --hold "
                             "rim-lip bowls = rows D-sampler): the sampler's release pose and the hold of kitfree_v1/"
                             "sampler_release/final_pose_table.json (plates upright with the 3 mm lean bias, mirror top-edge "
                             "hold; bowls in the packed pose, lip pinch); a dish whose release pose is infeasible there is placed "
                             "at its goal pose instead (row 'no reach: sampler release infeasible'); else carried to 50 mm above "
                             "the release pose, lowered straight down at 20 mm/s until the flange sensor trips, up 20 mm, the "
                             "jaws opened, the hand straight up 0.10 m, the settle, seated on the goal's wires (bowls: and mouth "
                             "down)")
    parser.add_argument("--tilt-table", type=Path, default=None,
                        help="--carry overlean (plans/2026-10-29-perch-roll.md, rows D-roll): the perch-roll curve's best_tilt.json "
                             "(frigidaire_drop_curve.py --perch-roll); a bowl runs only when gated there (its best tilt seated "
                             "mouth down >= 4/5), at that tilt only; the others are placed at their goals (rows 'not gated'); after "
                             "the open at contact and the I3 back-out, the rest wait, then the judgement (seated on the goal's "
                             "wires AND mouth down); a gripper-vs-bowl contact after the jaws open fails the row (jaw contact)")
    parser.add_argument("--jaw-check", action="store_true",
                        help="--tilt-table: keep only the spins whose open hand (gripper hulls + wrist 3 at the release joint "
                             "pose) stays >= 2 mm from the bowl along every recorded dish-alone trajectory of the curve at that "
                             "tilt (all repeats); none: the row 'not releasable'")
    parser.add_argument("--plate-hold", type=Path, default=None,
                        help="--insert seat (window 2): a hold_base.json (kitfree_v1/insert_plate/hold_base/) whose "
                             "holds[<--plate-hold-kind>][plate][depth mm].M_tcp_to_plate replaces the base-face hold")
    parser.add_argument("--plate-hold-kind", default="wall", help="--plate-hold: the construction (wall or mirror)")
    parser.add_argument("--rim-depth-m", type=float, default=None,
                        help="--insert seat: the assumed hold's rim depth into the pads (rounds 8-10: 0.019); the TCP moves "
                             "back along its approach by 0.019 - depth, the plate's pose in hand otherwise unchanged")
    parser.add_argument("--transit-rrt-s", type=float, default=15.,
                        help="transit_to's RRT-Connect wall budget per joint plan (s; the episode's 15 s by default)")
    parser.add_argument("--ring-side-order", choices=("pitch", "straight"), default=None,
                        help="D-ring-side (plans/2026-11-04-ring-side-hold.md): the descent starts' order. pitch = the smallest pitch "
                             "about the closing axis first (0.5 deg bins), straight carry legs first within a bin, then rule 1 (try a); "
                             "straight = the s0 rerun's order: straight carry legs first, then rule 1's, the pitch breaking its ties "
                             "(the user's retry of bowl_04, 2026-11-04). Default (plans/2026-11-04-fifteen-of-fifteen.md): the "
                             "--place-goal load row's ring_side_order when it has one, else pitch; given, it wins (the row records "
                             "the source)")
    parser.add_argument("--narrow-retry-s", type=float, default=None,
                        help="plans/2026-11-04-fifteen-of-fifteen.md, the narrow passages: when a joint plan's RRT-Connect (the "
                             "--transit-rrt-s budget; a bowl carry after its RING_RETRY_RRT_S retry too) finds no path, one more "
                             "search of this many seconds rooted at every goal (collide.rrt_connect's retry_s: the carry's other "
                             "passing descent starts beside its own, collide.reached_goal names the one reached), the margins "
                             "unchanged; default collide.NARROW_RETRY_S (180 s), 0 = off (the searches of before)")
    parser.add_argument("--shortcut-s", type=float, default=2.,
                        help="RRT-Connect shortcutting budget per joint plan (s; plans/2026-10-27-posture-fix.md: 10, was 2)")
    parser.add_argument("--side-base-run", default=None,
                        help="--hold side-base: the base-side gate run (placing_v1/<run>) whose median hold the drop uses when "
                             ">= 5 trials held (default partA_cup_side_base45), else the h45 mouth hold rolled 180 deg")
    parser.add_argument("--row-variant", default=None,
                        help="--place-goal: the rows' variant label (default D / D-side / ...; posture fix: D-side-m2)")
    parser.add_argument("--side-candidates", type=Path, default=ROOT / "data/results/robot/placing_v1/side/candidates.json",
                        help="--variant S: frigidaire_robot_reach.py --side-candidates' output")
    parser.add_argument("--attach", choices=("joint", "pin", "none"), default="joint",
                        help="--place-goal: a PhysX fixed joint (default; falls back to pin when it does not hold) or pin "
                             "(the dish written to TCP @ M after every rig step); none (plans/2026-11-04-grasp-place-s0.md, a "
                             "samples_v1 load, one dish per process): the dish spawned alone at the counter spot in its pick "
                             "orientation, picked by its kind's grasp (PICK_TRIES candidates, the 0.15 m lift, the hold gate) "
                             "and held by the pads alone; an in-hand drift over CARRY_DROP_M from the pick's hold before the "
                             "descent (cups: the release) is 'dropped in carry'")
    parser.add_argument("--state", type=Path, default=None,
                        help="--attach none: the scene the previous dish left (<load dir>/state/after_<k>_<dish>.json: the "
                             "placed dishes' settled poses, the rack joints, the failed dishes, which stay parked); this "
                             "dish's own state is written there after its row")
    parser.add_argument("--inhand-cup", type=Path, default=ROOT / "data/results/robot/placing_v1/partA_cup_wall",
                        help="--place-goal: the cup wall-pinch gate run whose median hold is the cup's in-hand pose")
    parser.add_argument("--interleave", action="store_true",
                        help="place-slot: trial-major order (one trial per site per lap, sites in the --sites order), so a "
                             "run cut short leaves every site with about the same n")
    parser.add_argument("--diag-hover", action="store_true",
                        help="limits round 2: before the trials, count Isaac's free hover IK / approach lines per grasp at "
                             "hovers 0.10, 0.125, 0.15 m and dump the arm model's calibration (diag_hover.json)")
    parser.add_argument("--legacy-tolerances", action="store_true",
                        help="R7: every blocked-motion settle tolerance at 0.08 rad (the upright3 values, plus the rig's "
                             "settled lag): a carried, pivoted bowl lags wrist_3 by 0.042-0.044 rad after the rise and the "
                             "insertion (limits round, 2026-10-23)")
    parser.add_argument("--gate-racks", choices=("upper_in", "both_out"), default="upper_in",
                        help="grasp gate rack state (upper_in = the episode's phase L: upper rack pushed in)")
    parser.add_argument("--grip-drive", default=None,
                        help="plans/2026-11-04-grip-force.md: finger drive overrides of flags R2 as name=value pairs, comma-separated "
                             "(gripper_effort, gripper_stiffness, gripper_damping: finger_joint; finger_effort, finger_stiffness, "
                             "finger_damping: the inner finger joints), e.g. gripper_effort=16; they override the hold's own drive "
                             "(plans/2026-11-04-fifteen-of-fifteen.md: flags.GRIP_DRIVE_BY_FAMILY by the grasp family, foot-out, "
                             "foot-side, side and side-down at 8 / 4 / 2.5, plate-wall and edge at 8 / 400 / 1, the rest the profile's)")
    parser.add_argument("--tilt-cap", choices=("on", "off"), default="on",
                        help="plans/2026-11-04-fifteen-of-fifteen.md: the bowl carry's tilt cap (BOWL_CARRY_TILT_MAX_DEG, "
                             "bowl_tilt_cap_z: max(20 deg, the tilt at the descent start) on every carry configuration); off = the "
                             "round 1 measurement (bowl_02's carry at the calibrated drive: the in-hand rotation per leg in the "
                             "row), the carry rule then mouth down and the hand above only")
    parser.add_argument("--basket", choices=("in", "out"), default=None,
                        help="plans/2026-11-04-fifteen-of-fifteen.md (basket out, the user 2026-11-04 19:20 EST; the real basket lifts "
                             "out): out = the twin without its SilverwareBasket, in the Kit scene (its Collisions and Visuals scopes "
                             "deactivated before sim.reset, the shapeless body kept with gravity off for the backend's indices; the log "
                             "line '[basket] removed <prim path>') and in the arm's FCL world (popped after every update_components); "
                             "default: the --place-goal load's top-level \"basket\" field (false = out), else in. Every row and the "
                             "report record basket and its source; a row whose gate.json / load row was computed with the other basket "
                             "state records basket_gate.mismatch. The robot rounds only (--grasp-gate, --place-slot, --place-goal)")
    parser.add_argument("--no-basket", dest="basket", action="store_const", const="out",
                        help="= --basket out (the robot rounds' twin without its silverware basket)")
    parser.add_argument("--move", default=None,
                        help="a samples_v1 --place-goal load (one dish): run the dish with this move (a SAMPLE_MOVE_ARGS name, e.g. "
                             "I4-wall20-drop: plans/2026-11-04-fifteen-of-fifteen.md D2's tine-guided plate release) instead of its "
                             "load row's; default the row's move, I4-wall20 with the row's release.mode drop run as I4-wall20-drop "
                             "(sample_move_of); the row records the move and its source (move_run)")
    parser.add_argument("--profile", choices=("headline", "legacy_upright3"), default="headline")
    parser.add_argument("--test-case", choices=("upright3",), default=None,
                        help="R4 (benchmark relaxation): 3 upright bowls on the counter -> 2 lower-rack + 1 upper-rack "
                             "spots on the scene of --instance")
    parser.add_argument("--diagnostic-continue", action="store_true",
                        help="D18 (NOT headline): skip a dish after its two attempts and go on; invariants are recorded "
                             "without stopping the dish, so one run yields a failure cause per bowl")
    parser.add_argument("--deinstance-root", default=None, help="D12: override the profile's de-instance root")
    parser.add_argument("--no-sleep-authoring", action="store_true",
                        help="ANALYSIS: leave the asset sleep thresholds untouched (A/B attribution of a changed regression result)")
    parser.add_argument("--mount", type=Path, default=None, help="override the profile's mount file (D17)")
    parser.add_argument("--max-moves", type=int, default=None)
    parser.add_argument("--video", action="store_true", help="accepted for compatibility: media are always recorded")
    parser.add_argument("--no-media", action="store_true", help="explicitly run without cameras (logged as such)")
    parser.add_argument("--usd", type=Path, default=ROOT / "data/assets/frigidaire_fdpc4221as/usd/fdpc4221as.usdc")
    parser.add_argument("--max-wall-seconds", type=float, default=1700.,
                        help="wall budget of the physics loop (D16: no single run beyond 30 min without asking)")
    parser.add_argument("--run-id", default=None,
                        help="every output goes to data/results/robot/runs/<run-id>/ (record, backend dir) and "
                             "data/artifacts/<run-id>/<trial-id>/ (trial log, media); never overwritten (D15)")
    parser.add_argument("--trial-id", default="t00_nominal")
    sys.path[:0] = [str(ROOT / "code/src")]
    from dishsim.app import add_args, launch   # Kit-free; boots Kit below
    add_args(parser)
    args = parser.parse_args()
    args.mode = ("grasp_gate" if args.grasp_gate else "place_slot" if args.place_slot else
                 "place_goal" if args.place_goal else None)
    if args.place_goal is not None and "moves" in json.loads(args.place_goal.read_text()):   # a samples_v1 load
        load_ = json.loads(args.place_goal.read_text())
        ids_ = [x for x in args.dishes.split(",") if x]
        if len(ids_) != 1 or args.attach != "none":
            raise SystemExit("[RESULT] FAIL a samples_v1 load runs one dish per process (--dishes <id>) with --attach none")
        d_ = next(d for d in load_["dishes"] if d["name"] == ids_[0])
        mv_, args.move_source = sample_move_of(d_)                # I4-wall20 with release.mode drop: I4-wall20-drop (D2)
        if args.move is not None:                                 # --move: the dish's move given (the row's recorded beside it)
            if args.move not in SAMPLE_MOVE_ARGS:
                raise SystemExit(f"[RESULT] FAIL --move {args.move!r} is not one of {sorted(SAMPLE_MOVE_ARGS)}")
            mv_, args.move_source = args.move, f"--move {args.move} (the load row's move: {d_.get('move')})"
        for k_, v_ in SAMPLE_MOVE_ARGS.get(mv_, SAMPLE_KIND_ARGS[d_["kind"]]).items():   # the dish's move (the
            setattr(args, k_, v_)                                 # kind's default: the study's rows) and the gate's pick
        if mv_ in SIDE_DOWN_MOVES:                                # round 3 (cup_05, the user 2026-11-05 00:05 EST): the row's hold
            if d_.get("side_below_top_m") is not None:            # height (side_below_top_m) and its Isaac gate run in the load
                args.side_below_top = float(d_["side_below_top_m"])   # dir (hold_gate_run), else SAMPLE_MOVE_ARGS' and
            args.side_below_top_source = ("the load row's side_below_top_m" if d_.get("side_below_top_m") is not None   # CUP_DOWN_
                                          else f"SAMPLE_MOVE_ARGS[{mv_!r}]")                                          # GATE_RUN
            args.hold_gate_run = d_.get("hold_gate_run") or None
        if (d_.get("pick") or {}).get("presentation"):            # the plate's presentation (its backstop) beside the load
            args.presentation = args.place_goal.parent / d_["pick"]["presentation"]
    if args.move is not None and getattr(args, "move_source", None) is None:
        raise SystemExit("[RESULT] FAIL --move needs a samples_v1 --place-goal load (one dish per process, --attach none)")
    if args.foot_candidates is None:                  # foot-side: the Kit-free gate's order (plans/2026-11-04-ring-side-hold.md)
        args.foot_candidates = (FOOT_SIDE_GATE_JSON if args.grasp_family == "foot-side" else
                                ROOT / "data/results/robot/placing_v1/foot/candidates.json")
    elif not args.foot_candidates.is_absolute():
        args.foot_candidates = ROOT / args.foot_candidates
    if args.presentation is not None and not args.presentation.is_absolute():
        args.presentation = ROOT / args.presentation
    if args.presentation is not None and args.grasp_family not in ("edge", "plate-wall"):
        raise SystemExit("[RESULT] FAIL --presentation needs --grasp-family edge or plate-wall (a plate pinch at its presentation)")
    if args.attach == "none" and args.state is not None and not args.state.is_absolute():
        args.state = ROOT / args.state
    if args.mode is None and args.instance is None:
        raise SystemExit("[RESULT] FAIL --instance is required without --grasp-gate / --place-slot")
    args.basket_present, args.basket_source = basket_state(   # plans/2026-11-04-fifteen-of-fifteen.md: basket out
        args.basket, json.loads(args.place_goal.read_text()) if args.place_goal is not None else None)
    if not args.basket_present and args.mode is None:
        raise SystemExit("[RESULT] FAIL --no-basket / --basket out is for the robot rounds (--grasp-gate, --place-slot, --place-goal): "
                         "the instance episode's end check (backend.hold's basket_supported gate) needs the basket")
    if args.bowl_supports is not None:                # the seated test reads the refreshed goals' supports
        global BOWL_SUPPORTS_JSON
        BOWL_SUPPORTS_JSON = args.bowl_supports if args.bowl_supports.is_absolute() else ROOT / args.bowl_supports
    if args.carry == "above" and not (args.variant == "D" and args.hold == "foot-out"):
        raise SystemExit("[RESULT] FAIL --carry above needs --variant D --hold foot-out")
    if args.release_height_mm is not None and not (args.variant == "D" and args.hold == "foot-out"):
        raise SystemExit("[RESULT] FAIL --release-height-mm needs --variant D --hold foot-out")
    if args.carry and args.carry != "above" and not (args.variant == "D" and args.hold == "rim-lip"):
        raise SystemExit(f"[RESULT] FAIL --carry {args.carry} needs --variant D --hold rim-lip")
    if (args.tilt_table is not None or args.jaw_check) and args.carry != "overlean":
        raise SystemExit("[RESULT] FAIL --tilt-table / --jaw-check need --carry overlean")
    if args.jaw_check and args.tilt_table is None:
        raise SystemExit("[RESULT] FAIL --jaw-check needs --tilt-table (the curve's recorded trajectories)")
    if args.insert == "seat" and (args.variant != "I" or (args.rim_depth_m is None and args.attach != "none")):
        raise SystemExit(f"[RESULT] FAIL --insert {args.insert} needs --variant I and --rim-depth-m")
    if args.insert == "sampler" and not (args.variant == "I" or (args.variant == "D" and args.hold == "rim-lip")):
        raise SystemExit("[RESULT] FAIL --insert sampler needs --variant I (plates) or --variant D --hold rim-lip (bowls)")
    args.cameras = bool(getattr(args, "enable_cameras", False))
    if not args.cameras and not args.no_media:
        raise SystemExit("[RESULT] FAIL counted trials record media: launch with --enable_cameras (or pass --no-media)")
    stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
    args.run_id = args.run_id or f"{args.profile}_{args.instance.stem if args.instance else args.mode}_{stamp}"
    args.out_dir = RUNS / args.run_id
    args.trial_dir = ARTIFACTS / args.run_id / args.trial_id
    if args.mode:                                     # the limits round keeps its records and replays together
        args.out_dir = (args.out_root or LIMITS) / args.run_id
        args.trial_dir = args.out_dir / "trials"
    if args.out_dir.exists() or args.trial_dir.exists():
        raise SystemExit(f"[RESULT] FAIL refusing to overwrite run {args.run_id} ({args.out_dir} or {args.trial_dir} exists)")
    args.out_dir.mkdir(parents=True)
    args.trial_dir.mkdir(parents=True)
    app = launch(args)
    args.app = app
    sys.path[:0] = [str(ROOT / "code/src"), str(ROOT / "code/frigidaire/src"), str(ROOT / "code/planner/frigidaire")]
    from dishsim_frigidaire.robot import flags as F
    overrides = {}
    if args.test_case:
        overrides["test_case"] = args.test_case
    if args.diagnostic_continue:
        overrides.update(continue_after_failed_dish=True, invariants_abort=False)
    if args.deinstance_root:
        overrides["deinstance_root"] = args.deinstance_root
    if args.no_sleep_authoring:
        overrides["sleep_threshold"] = None
    if args.mount:
        overrides["mount"] = str(args.mount)
    if not args.basket_present:                       # flags.basket (a benchmark relaxation, recorded in every run)
        overrides["basket"] = False
    if args.mode:                                     # invariants recorded per trial, never abort the trial loop
        overrides.update(invariants_abort=False, counter_cap=False)
    if args.contact_lag is not None:
        overrides["place_down_contact_lag"] = float(args.contact_lag)
    if args.legacy_tolerances:
        overrides.update(tol_approach=.08, tol_joint_move_held=.08, tol_rrt=.08, tol_rise=.08, tol_lift=.08, tol_reaim=.08,
                         settled_lag_rad=.08)       # place()'s insertion line settles with the rig default
    if args.mode == "grasp_gate":                     # G3: hold 5 s (the 0.15 m lift: pick's lift_m)
        overrides.update(hold_window_s=5.)
    drive_over = {}                                   # --grip-drive: overrides of the hold's drive (flags R2 per family)
    for kv in (args.grip_drive or "").split(","):
        if kv.strip():
            k_, v_ = (s.strip() for s in kv.split("=", 1))
            if k_ not in F.GRIP_DRIVE_FIELDS:
                raise SystemExit(f"[RESULT] FAIL --grip-drive {k_!r} is not one of {F.GRIP_DRIVE_FIELDS}")
            drive_over[k_] = float(v_)
    overrides.update(drive_over)
    flags = F.resolve(args.profile, grip_family=grip_family_of(args), **overrides)
    global LIMITS_COUNTER_XY                          # plans/2026-11-04-fifteen-of-fifteen.md: the run's counter spot (every
    load_ = json.loads(args.place_goal.read_text()) if args.place_goal is not None else None   # spawn and set-down, the gates',
    spot_, args.spot_source = counter_spot(flags.mount, load_, args.counter_y)                 # the presentation moved with it)
    LIMITS_COUNTER_XY = spot_
    args.spot_xy = [float(v) for v in spot_]
    print(f"[INFO] counter spot {args.spot_xy}: {args.spot_source}", flush=True)
    print(f"[INFO] basket {'in' if flags.basket else 'out'}: {args.basket_source}", flush=True)
    if load_ is not None and load_.get("mount_file") and not same_file(load_["mount_file"], flags.mount):
        print(f"[WARN] the load was built for the mount {load_['mount_file']}; this run stands on {flags.mount}", flush=True)
    if args.mode == "place_goal":
        inst = goal_instance(json.loads(args.place_goal.read_text()))
    elif args.mode:
        kind = args.grasp_gate or args.place_slot
        if args.grasp_family.startswith("foot") and kind != "bowl":
            raise SystemExit("[RESULT] FAIL --grasp-family foot-* needs --grasp-gate bowl")
        if args.mouth_down and not (kind == "cup" and args.grasp_family == "side"):
            raise SystemExit("[RESULT] FAIL --mouth-down needs --grasp-gate cup --grasp-family side")
        if args.grasp_family in ("edge", "plate-wall") and (kind != "plate" or args.presentation is None):
            raise SystemExit(f"[RESULT] FAIL --grasp-family {args.grasp_family} needs --grasp-gate plate --presentation <json>")
        inst = limits_instance(kind, mouth_down=args.grasp_family.startswith("foot") or args.mouth_down,
                               pres=load_presentation(args.presentation, LIMITS_COUNTER_XY))
    else:
        inst = json.loads(args.instance.read_text())
    if flags.test_case:
        inst = test_instance(inst, flags.test_case)
    iid = inst["instance_id"]
    report = {"instance": iid, "result": "FAIL", "started_utc": datetime.now(timezone.utc).isoformat(), "moves": [],
              "run_id": args.run_id, "trial_id": args.trial_id, "profile": flags.profile, "headline": F.is_headline(flags),
              "flags": F.to_dict(flags), "deviations_from_headline": [list(d) for d in F.deviations(flags)],
              "trial_dir": str(args.trial_dir.relative_to(ROOT)),
              "spot": {"xy": args.spot_xy, "source": args.spot_source, "mount_file": flags.mount,
                       "presentation_file": None if args.presentation is None else rel_path(args.presentation),
                       "rule": "plans/2026-11-04-fifteen-of-fifteen.md: every dish spawned and set down at this counter spot (the "
                               "mount file's best.spot_xy, else the load's pick_spot_xy, else LIMITS_COUNTER_XY; --counter-y sets "
                               "y); a --presentation moved with it (frigidaire_robot_reach.shift_presentation)"},
              "narrow_retry_s": narrow_budget(args),
              "basket": bool(flags.basket), "basket_source": args.basket_source,
              "grip_drive": {**F.grip_drive(flags), "override": drive_over or None,
                             "rule": "flags R2 per hold (plans/2026-11-04-fifteen-of-fifteen.md): finger_joint spawned with the grasp "
                                     "family's drive (flags.GRIP_DRIVE_BY_FAMILY; a family not listed: the profile's fields), "
                                     "--grip-drive on top; one family per Kit process"}}
    closers = []
    try:
        episode(args, inst, report, flags, closers)
    except Exception:
        report["error"] = traceback.format_exc()
        print(report["error"], flush=True)
        for fn in closers:                                             # keep the mp4 and the trial log readable
            try:
                fn()
            except Exception:                                          # noqa: BLE001
                pass
    report["finished_utc"] = datetime.now(timezone.utc).isoformat()
    record = args.out_dir / f"{iid}.json"
    record.write_text(json.dumps(report, indent=1, default=lambda o: o.tolist() if hasattr(o, "tolist") else str(o)) + "\n")
    ok = [m for m in report["moves"] if m.get("ok")]
    v = report.get("verdicts", {})
    print(f"[RESULT] {report['result']} robot episode {iid} ({flags.profile}{', diagnostic' if flags.continue_after_failed_dish else ''}): "
          f"{len(ok)}/{len(report['moves'])} moves ok, end {report.get('end_check', {}).get('outcome')}, abort {report.get('abort')}, "
          f"legacy rule {v.get('legacy_rule')}, benchmark success {v.get('benchmark_success')}, "
          f"invariants {v.get('invariant_counts')}, harness {v.get('harness')}", flush=True)
    try:
        from dishsim.media import release_sim_for_close
        release_sim_for_close()
    finally:
        app.close()


LIMITS = ROOT / "data/results/robot/limits_v1"
LIMITS_COUNTER_XY = (.20, -.12)       # = frigidaire_robot_reach.COUNTER_XY: the --side-spot sweep's spot at the -0.17 edge
                                      # (2026-10-27; it was (.20, -.16) with the edge at -0.30); main() sets the run's spot here
                                      # (counter_spot: the mount's spot_xy, else the load's pick_spot_xy, else this)


def rel_path(p):
    """``p`` relative to the repo root when it lies under it (a record's path), else as given."""
    try:
        return str(Path(p).resolve().relative_to(ROOT))
    except ValueError:
        return str(p)


def same_file(a, b):
    """Two repo paths (relative to the root or absolute) name the same file."""
    pa, pb = Path(a), Path(b)
    return (pa if pa.is_absolute() else ROOT / pa).resolve() == (pb if pb.is_absolute() else ROOT / pb).resolve()


def counter_spot(mount_file, load=None, counter_y=None):
    """plans/2026-11-04-fifteen-of-fifteen.md: the run's counter spot (x, y), where every dish is spawned and set down: the mount
    file's best.spot_xy when it has one (frigidaire_mount_sweep.py's posture_v2.json: the spot moved with the arm along the
    counter's front edge), else the --place-goal load's top-level pick_spot_xy, else LIMITS_COUNTER_XY; ``counter_y``
    (--counter-y) sets y. (xy, source)"""
    try:
        p_ = Path(mount_file)
        sp_ = (json.loads((p_ if p_.is_absolute() else ROOT / p_).read_text()).get("best") or {}).get("spot_xy")
    except (OSError, TypeError, ValueError):
        sp_ = None
    if sp_ is not None and len(sp_) >= 2:
        xy, src = (float(sp_[0]), float(sp_[1])), f"the mount file's best.spot_xy ({mount_file})"
    elif load is not None and load.get("pick_spot_xy") is not None:
        xy, src = (float(load["pick_spot_xy"][0]), float(load["pick_spot_xy"][1])), f"the load's pick_spot_xy ({load.get('load')})"
    else:
        xy, src = (float(LIMITS_COUNTER_XY[0]), float(LIMITS_COUNTER_XY[1])), "LIMITS_COUNTER_XY (the study's spot)"
    if counter_y is not None:
        xy, src = (xy[0], float(counter_y)), src + "; y from --counter-y"
    return xy, src


def load_presentation(path, xy):
    """--presentation: the plate presentation json moved with the run's counter spot ``xy`` (frigidaire_robot_reach.
    shift_presentation, the sample gate's own move: the plate, its backstop and its fixtures translated by xy minus the spot it
    was built at, its "spot_xy", else (0.20, -0.12); unmoved when they agree); None without a path."""
    if path is None:
        return None
    sys.path.insert(0, str(ROOT / "code/execution/frigidaire"))
    import frigidaire_robot_reach as RC
    return RC.shift_presentation(json.loads(Path(path).read_text()), xy)


def presentation_boxes(pres):
    """A presentation's static boxes: its "fixtures" (the goal plank and its lip; the s0 rerun's window copy data/logs/robot_final/
    frigidaire_robot_episode_fixtures.py, now the episode's), else its backstop alone. [{id, center_m, size_m, axes,
    quaternion_xyzw}]"""
    return [dict(b_) for b_ in (pres.get("fixtures") or [dict(pres["backstop"], id=pres["backstop"].get("id", "Backstop"))])]


def extra_fixtures(pres):
    """A presentation's fixtures other than its backstop (the plank's lip) = frigidaire_robot_reach.extra_fixtures: the arm model
    holds the backstop as "Backstop" and these by their ids, as the sample gate does."""
    b_id = (pres.get("backstop") or {}).get("id", "Backstop")
    return [f_ for f_ in pres.get("fixtures") or [] if f_.get("id") not in (b_id, "Backstop")]


def narrow_budget(args):
    """--narrow-retry-s (plans/2026-11-04-fifteen-of-fifteen.md): the narrow passages' retry budget (s), collide.NARROW_RETRY_S
    when not given; 0 = off."""
    v = getattr(args, "narrow_retry_s", None)
    if v is None:
        sys.path.insert(0, str(ROOT / "code/frigidaire/src"))
        from dishsim_frigidaire.robot.collide import NARROW_RETRY_S
        return float(NARROW_RETRY_S)
    return max(0., float(v))


# plans/2026-11-04-fifteen-of-fifteen.md, basket out (the user, 2026-11-04 19:20 EST; the real basket lifts out): the robot rounds load
# the twin without its silverware basket; s0's goal poses, the benchmark samples and S_ref keep it
BASKET_NAME = "SilverwareBasket"
BASKET_PARTS = ("Collisions", "Visuals")      # the basket body's collider scope (385 capsules) and visual scope (288 meshes)


def basket_flag(v):
    """A record's basket field as True (in) / False (out) / None (not recorded): a bool, "in" / "out", or a dict with "present"."""
    if isinstance(v, dict):
        v = v.get("present")
    if isinstance(v, bool):
        return v
    if isinstance(v, str) and v.lower() in ("in", "out"):
        return v.lower() == "in"
    return None


def basket_state(choice, load=None):
    """The run's basket (True = in) and its source: --basket in|out (--no-basket = out) when given, else the --place-goal load's
    top-level "basket" field (false = out), else in (the twin as built)."""
    if choice is not None:
        return choice == "in", ("--no-basket" if choice == "out" else "--basket in")
    b_ = basket_flag((load or {}).get("basket"))
    if b_ is not None:
        return b_, f"the --place-goal load's basket field ({(load or {}).get('load')}: {(load or {}).get('basket')!r})"
    return True, "default: the twin as built (no --no-basket / --basket, no basket field in a --place-goal load)"


def remove_basket(stage, twin_prim):
    """Basket out in the Kit scene, before sim.reset (the episode's before_reset): the basket body's Collisions and Visuals scopes
    (BASKET_PARTS) deactivated, so nothing touches it and nothing renders it; the shapeless body stays (its authored mass, gravity
    off so it never moves) because the backend's RigidObject, its pose reads and its contact view index the five components
    (initial_state_runtime.IsaacInitialStateBackend). The four other components, the racks' joints and the backend's joint gate are
    untouched. Prints "[basket] removed <prim path>". The record."""
    from pxr import PhysxSchema, Usd, UsdGeom, UsdPhysics
    path = f"{twin_prim}/{BASKET_NAME}"
    body = stage.GetPrimAtPath(path)
    if not body or not body.IsValid():
        raise RuntimeError(f"basket out: no basket prim at {path}")
    rec = {"prim": path, "deactivated": {}}
    for part in BASKET_PARTS:
        p_ = stage.GetPrimAtPath(f"{path}/{part}")
        if not p_ or not p_.IsValid():
            raise RuntimeError(f"basket out: no {part} scope under {path}")
        rec["deactivated"][f"{path}/{part}"] = {
            "colliders": sum(1 for x in Usd.PrimRange(p_) if x.HasAPI(UsdPhysics.CollisionAPI)),
            "meshes": sum(1 for x in Usd.PrimRange(p_) if x.IsA(UsdGeom.Mesh))}
        p_.SetActive(False)
    PhysxSchema.PhysxRigidBodyAPI.Apply(body).CreateDisableGravityAttr(True)
    rec["colliders_left"] = sum(1 for x in Usd.PrimRange(body) if x.HasAPI(UsdPhysics.CollisionAPI))
    if rec["colliders_left"]:
        raise RuntimeError(f"basket out: {rec['colliders_left']} colliders left under {path}")
    n_c = sum(v["colliders"] for v in rec["deactivated"].values())
    n_m = sum(v["meshes"] for v in rec["deactivated"].values())
    rec.update(gravity="off (physxRigidBody:disableGravity)", rule=" ".join(remove_basket.__doc__.split(". The record")[0].split()))
    print(f"[basket] removed {path} ({n_c} colliders and {n_m} visual meshes deactivated: {', '.join(BASKET_PARTS)}; the shapeless "
          f"body kept, gravity off)", flush=True)
    return rec


def drop_basket_fcl(checker):
    """Basket out in the episode's FCL arm world (frigidaire_robot_reach.strip_basket's rule, the basket_gate.py measurement):
    ``checker``'s update_components wrapped on this instance so that the basket's body is popped from checker.components right
    after the five are built; every check that walks the components (collide.ArmWorld collisions / held_clearance, scene_targets,
    seat_search, seat_line_hits, the retreat check) no longer sees it; the frames keep all five (initial_state_candidates._frames
    requires them). The live record (pops counted)."""
    orig = checker.update_components
    log = {"popped": 0, "fcl_objects": None, "rule": " ".join(drop_basket_fcl.__doc__.split(" The live")[0].split())}

    def update_components(component_frames):
        orig(component_frames)
        b_ = (checker.components or {}).pop(BASKET_NAME, None)
        if b_ is not None:
            log["popped"] += 1
            log["fcl_objects"] = len(b_["objects"])
    checker.update_components = update_components
    b0_ = (checker.components or {}).pop(BASKET_NAME, None)   # components built before the wrap (a world made earlier)
    if b0_ is not None:
        log["popped"] += 1
        log["fcl_objects"] = len(b0_["objects"])
    return log
PIVOT_DEG = 10.                       # a held dish rotated more than this in the hand after the lift counts as "pivot"
TRAJ_EVERY = 12                       # ticks between replay samples (10 fps of simulated time at 120 Hz)


def counter_pose(xy, mouth_down=False, kind="bowl"):
    """The gate's counter pose of the dish: mouth up 2 mm above the slab, or (the foot families; the cup of
    plans/2026-11-04-cups-mouth-down.md) mouth down, its rim 2 mm above the slab and its base-centred origin on top."""
    if not mouth_down:
        return {"position_m": [xy[0], xy[1], .914 + .002], "quaternion_xyzw": [0., 0., 0., 1.]}
    sys.path.insert(0, str(ROOT / "code/frigidaire/src"))
    from dishsim_frigidaire.robot import grasp as G
    h = G.KIND_RIM["cup"][2] if kind == "cup" else G.foot_ring()["bowl_height"]
    return {"position_m": [xy[0], xy[1], .914 + h + .002], "quaternion_xyzw": [1., 0., 0., 0.]}


def limits_instance(kind, mouth_down=False, pres=None):
    """One mouth-up dish of ``kind`` on the counter (mouth down for the foot families and --mouth-down; ``pres``: the plate
    leaning on its presentation's backstop), on the benchmark's extended baseline (no instance file needed)."""
    base = json.loads((ROOT / "data/results/benchmark/frigidaire_hotec/baseline/result.json").read_text())
    snap = base.get("snapshot", base)
    oid = f"{kind}_01"
    pose = presentation_pose(pres)[0] if pres else counter_pose(LIMITS_COUNTER_XY, mouth_down, kind)
    poses = {**{n: v for n, v in snap["poses"].items() if not n.startswith(("plate_", "bowl_", "cup_"))}, oid: pose}
    return {"instance_id": f"limits_{kind}", "tier": "easy", "baseline": snap, "initial_snapshot": {"poses": poses},
            "objects": [{"object_id": oid, "kind": kind, "start": "Counter", "rack": "LowerRack", "pose_world": pose}],
            "counter": {"start_count": 1, "cap": 3}, "support": [],
            "goal": {"objects": {oid: {"rack": "LowerRack", "slot": "counter", "settled_pose_world": pose}}, "order": []}}


TEST_COUNTER_XY = ((.15, -.20), (.35, -.20), (.25, .00))        # upright on the counter (front edge y -0.30)
TEST_TARGETS = (("LowerRack", (.14, -.12)), ("LowerRack", (-.02, -.20)), ("UpperRack", (.12, -.12)))   # rack-local xy, robot side
TEST_TARGET_Z = .13                    # rack-local base height of the release pose; lowered further until contact


def test_instance(scene, case):
    """A small robot test on the scene (baseline, rack frames) of ``scene``: upright bowls, rack-front targets."""
    frames = scene["initial_snapshot"]["poses"]
    objects, goal = [], {}
    for k, (xy, (rack, local)) in enumerate(zip(TEST_COUNTER_XY, TEST_TARGETS), 1):
        oid = f"bowl_{k:02d}"
        pose = {"position_m": [xy[0], xy[1], .914 + .002], "quaternion_xyzw": [0., 0., 0., 1.]}
        objects.append({"object_id": oid, "kind": "bowl", "start": "Counter", "rack": "LowerRack", "pose_world": pose})
        f = frames[rack]["position_m"]
        target = {"position_m": [f[0] + local[0], f[1] + local[1], f[2] + TEST_TARGET_Z], "quaternion_xyzw": [0., 0., 0., 1.]}
        goal[oid] = {"rack": rack, "slot": f"test_{rack}_{k}", "settled_pose_world": target, "settled_rack_local_pose":
                     {"position_m": [local[0], local[1], TEST_TARGET_Z], "quaternion_xyzw": [0., 0., 0., 1.]}}
    poses = {**{n: v for n, v in frames.items() if not n.startswith(("plate_", "bowl_", "cup_"))},
             **{o["object_id"]: o["pose_world"] for o in objects}}
    return {"instance_id": f"robot_{case}", "tier": "easy", "objects": objects, "baseline": scene["baseline"],
            "initial_snapshot": {"poses": poses}, "counter": {"start_count": len(objects), "cap": len(objects) + 2},
            "support": [], "goal": {"objects": goal, "order": []}, "test_case": case, "scene_from": scene["instance_id"]}


def failure_class(text):
    """The plan's Phase 5 failure classes from a failure reason (acquire / in-hand / transport collision / insertion /
    out of tolerance / disturbed neighbour / unreachable / infeasible goal), plus the harness's own."""
    t = (text or "").lower()
    if t.startswith("invariant"):
        return "invariant"
    if t.startswith("not attempted"):
        return "not attempted"
    if "counter-full" in t:
        return "counter-full (refused)"
    if "skipped" in t:
        return "skipped"
    if "no collision-free grasp" in t or "blocked on the approach" in t or "jaws at" in t:
        return "acquire" if "no collision-free grasp" not in t else "unreachable"
    if "not held" in t or "dropped" in t:
        return "in-hand"
    if "lag_rad" in t or "blocked" in t:                  # the rig's blocked-motion verdict (R7/D8 lag limits)
        return "transport blocked (lag limit)" if ("safe height" in t or "no path" in t) else "blocked (lag limit)"
    if "ik branch jump" in t:                             # no branch-continuous IK along a straight TCP line
        return "IK branch jump (rise to the safe height)" if "safe height" in t else "IK branch jump (insertion)"
    if "no path" in t or "no collision-free ik" in t or "safe height" in t:
        return "transport collision"
    if "line collides" in t or "descending" in t or "lowering" in t or "ik branch jump" in t:
        return "insertion"
    if "unstable" in t:
        return "unstable after release"
    if "tolerance" in t or "not at goal" in t or "not racked" in t or "off the counter" in t:
        return "out of tolerance"
    if "disturbed" in t:
        return "disturbed neighbour"
    return "other"


def episode(args, inst, report, flags, closers=None):
    import importlib.util
    import numpy as np
    spec = importlib.util.spec_from_file_location("frigidaire_bench", ROOT / "code/planner/frigidaire/frigidaire_bench.py")
    B = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(B)
    import frigidaire_bench_kit as K
    from isaaclab.assets import Articulation
    from dishsim import rearrange as R
    from dishsim_frigidaire.initial_state_runtime import IsaacInitialStateBackend
    from dishsim_frigidaire.random_pose_experiment import BudgetExpired
    from dishsim_frigidaire.random_poses import LIMITS, source_geometry_domains
    from dishsim_frigidaire.random_pose_runtime import EXTENSION, RACK_JOINT
    from dishsim_frigidaire.loading import visual_points
    from dishsim_frigidaire import planner as P
    from dishsim_frigidaire.robot import flags as F, grasp as G, harness as HN, kin, triallog as TL, ur5e
    from dishsim_frigidaire.robot.rig import Rig, T_of
    report["counter"] = G.pull_counter(B, args.counter_front_y)   # before the Kit slab, the FCL slab and the counter band

    iid, tier = inst["instance_id"], inst["tier"]
    ids = [o["object_id"] for o in inst["objects"]]
    kinds = {o["object_id"]: o["kind"] for o in inst["objects"]}
    counter_ids = [o["object_id"] for o in inst["objects"] if o["start"] == "Counter"]
    points = {kind: visual_points(B.ASSETS / f"{kind}.usda") for kind in B.KINDS}
    centroids = B.kind_centroids(points)
    mount_path = Path(flags.mount) if Path(flags.mount).is_absolute() else ROOT / flags.mount
    mount = json.loads(mount_path.read_text())["best"]
    yaw = mount["yaw_rad"]
    T_wb = np.eye(4)
    T_wb[:3, :3] = [[math.cos(yaw), -math.sin(yaw), 0], [math.sin(yaw), math.cos(yaw), 0], [0, 0, 1]]
    T_wb[:3, 3] = mount["base"]
    report["mount"] = mount["base"] + [yaw]
    report["mount_file"] = flags.mount
    entries = [{"object_id": o["object_id"], "kind": o["kind"], "rack": o.get("rack", "LowerRack"), "pose_world": o["pose_world"]}
               for o in inst["objects"]]
    holder, cams, hook = {}, {}, {}
    pres = load_presentation(getattr(args, "presentation", None), LIMITS_COUNTER_XY)   # the plate's backstop (its fixtures), moved
    if pres:                                                                            # with the run's counter spot
        report["presentation"] = {"file": str(args.presentation), **{k: pres.get(k) for k in ("name", "yaw_k", "lean_deg",
                                                                                              "plate_pose", "backstop", "edge",
                                                                                              "facing_deg", "wall", "spot_xy",
                                                                                              "moved_from_xy")},
                                  "fixtures": [b_["id"] for b_ in presentation_boxes(pres)],
                                  "kit": "every fixture a static box under /World/Backstop (its plank and lip), the backstop alone "
                                         "at /World/Backstop without a fixture list; the arm model: Backstop + the other fixtures"}

    class Backend(IsaacInitialStateBackend):
        """Counter dishes ride along; every tick feeds the harness; state writes pass the teleport guard."""

        def __init__(self, *a, **kw):
            super().__init__(*a, **kw)
            self.assignments = {k: v for k, v in self.assignments.items() if k not in set(counter_ids)}

        def tick(self):
            frames, joints = super().tick()
            fn = hook.get("tick")
            if fn is not None:
                fn(frames)
            return frames, joints

        def set_rigid_pose(self, obj, pose):
            fn = hook.get("teleport")
            if fn is not None:
                fn(f"set_rigid_pose {obj.cfg.prim_path}")
            return super().set_rigid_pose(obj, pose)

        def restore(self, snapshot):
            fn = hook.get("teleport")
            if fn is not None:
                fn("restore")
            return super().restore(snapshot)

    def before_reset(backend):
        import omni.usd
        stage = omni.usd.get_context().get_stage()
        if not flags.basket:                             # plans/2026-11-04-fifteen-of-fifteen.md, basket out: the twin without its
            holder["basket"] = remove_basket(stage, backend.prim)   # silverware basket (its colliders and visuals), before sim.reset
        # the robot and its pedestal (a static column under the base)
        import isaaclab.sim as sim_utils
        from isaaclab_physx.sim.schemas import PhysxCollisionCfg
        ped = sim_utils.CylinderCfg(radius=.08, height=float(mount["base"][2]),
                                    collision_props=[sim_utils.UsdPhysicsCollisionCfg(), PhysxCollisionCfg()],
                                    visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(.35, .36, .40)))
        ped.func("/World/Pedestal", ped, translation=(mount["base"][0], mount["base"][1], mount["base"][2] / 2))
        q_mount = (0., 0., math.sin(yaw / 2), math.cos(yaw / 2))   # XYZW (Isaac Lab 3.0)
        holder["robot"] = Articulation(ur5e.robot_cfg(prim_path="/World/Robot", pos=tuple(mount["base"]), rot_xyzw=q_mount,
                                                       gripper_effort=flags.gripper_effort, gripper_stiffness=flags.gripper_stiffness,
                                                       gripper_damping=flags.gripper_damping, finger_effort=flags.finger_effort,
                                                       finger_stiffness=flags.finger_stiffness, finger_damping=flags.finger_damping))
        holder["deinstanced"] = ur5e.deinstance_gripper(stage)
        if flags.deinstance_root != "/World/Robot/Gripper":                # D12: the whole robot
            holder["deinstanced_root"] = {flags.deinstance_root: HN.deinstance(stage, flags.deinstance_root)}
        holder["pads"] = ur5e.pad_material(stage)
        if pres:                                         # the plate presentation's backstop: a static box on the counter; a
            from pxr import Gf, PhysxSchema as PS_, UsdGeom, UsdPhysics as UP_   # fixture list (the plank and its lip): every box
            if pres.get("fixtures"):                     # under /World/Backstop (the harness's counter fixture), as the s0 rerun's
                UsdGeom.Xform.Define(stage, "/World/Backstop")   # window copy built them (plans/2026-11-04-fifteen-of-fifteen.md)
            for b_ in presentation_boxes(pres):
                cube = UsdGeom.Cube.Define(stage, f"/World/Backstop/{b_['id']}" if pres.get("fixtures") else "/World/Backstop")
                cube.CreateSizeAttr(1.)
                cube.CreateDisplayColorAttr([Gf.Vec3f(.55, .45, .35)])
                xf = UsdGeom.Xformable(cube.GetPrim())
                xf.ClearXformOpOrder()
                xf.AddTranslateOp().Set(Gf.Vec3d(*[float(v) for v in b_["center_m"]]))
                q_ = b_["quaternion_xyzw"]
                xf.AddOrientOp().Set(Gf.Quatf(float(q_[3]), float(q_[0]), float(q_[1]), float(q_[2])))
                xf.AddScaleOp().Set(Gf.Vec3f(*[float(v) for v in b_["size_m"]]))
                UP_.CollisionAPI.Apply(cube.GetPrim())
                PS_.PhysxCollisionAPI.Apply(cube.GetPrim())
            holder["backstop"] = "/World/Backstop"
        holder["authored"] = HN.author_sleep_and_reports(stage, [f"/World/InitialStateDishes/{oid}" for oid in backend.objects],
                                                         sleep_threshold=flags.sleep_threshold)
        if args.cameras:
            from dishsim.media import CameraRig
            sys.path.insert(0, str(ROOT / "code/util/frigidaire"))
            from frigidaire_initial_state_render import item_tints
            K.tint_scene(item_tints([{"object_id": oid} for oid in backend.objects]), B.COUNTER_COLOR)
            cams["rig"] = CameraRig(ROBOT_CAMERA, hw=VIDEO_HW)

    slab = ("/World/Counter", tuple(B.COUNTER["size_m"]), tuple(B.COUNTER["center_m"]))
    t_start = time.monotonic()
    backend = Backend(args.usd, args.out_dir / "_kit" / iid, device="cpu", domains=source_geometry_domains(),
                      deadline=t_start + max(60., args.max_wall_seconds - SCORING_GRACE_S), app=args.app, candidates=entries,
                      extra_statics=[slab], tableware=B.tableware(), before_reset=before_reset)
    robot = holder["robot"]
    report.update(deinstanced=holder["deinstanced"], deinstanced_root=holder.get("deinstanced_root"),
                  pad_colliders=holder["pads"], authored=holder["authored"], basket_removal=holder.get("basket"))
    if "rig" in cams:
        cams["rig"].apply_poses(backend.sim.device)
        backend.tick()                                   # one physics step before any render (Fabric)

    # ---------------------------------------------------------------- trial log, media, rig, harness
    baseline, start_poses = inst["baseline"], inst["initial_snapshot"]["poses"]
    cfg = TL.config_id(start_poses, ids)
    tlog = TL.TrialLog(args.trial_dir / "trial.jsonl", run_id=args.run_id, trial_id=args.trial_id, config=cfg,
                       profile=flags.profile, clock=lambda: (backend.tick_index, backend.tick_index * backend.dt))
    holder["n_moves"] = None

    def overlay_lines():
        h = holder.get("harness")
        c = tlog.ctx
        return [f"{args.run_id} / {args.trial_id}   profile {flags.profile}"
                f"{' (diagnostic, not headline)' if flags.continue_after_failed_dish else ''}",
                f"simulated t = {h.trial_time() if h else 0.:7.2f} s      wall = {tlog.wall():6.1f} s",
                f"move {c.get('move') or '-'}/{holder['n_moves'] or '-'}  {c.get('dish') or ''}   "
                f"attempt {'-' if c.get('attempt') is None else c['attempt'] + 1}   {c.get('phase') or ''}"]

    media = None
    if "rig" in cams:
        media = HN.Media(backend=backend, cams=cams["rig"], cam_name="robot", trial_dir=args.trial_dir,
                         overlay_text=overlay_lines, fps=VIDEO_FPS)
    if closers is not None:
        closers += [tlog.close] + ([media.close] if media is not None else [])
    watch = {}

    def on_step():
        if watch.get("oid"):
            T_t = rig.tcp()
            off = float(np.linalg.norm(T_now(watch["oid"])[:3, 3] - (T_t @ watch["M"])[:3, 3]))
            watch["n"] = watch.get("n", 0) + 1
            if off > watch.get("max", 0.):
                watch["max"] = off
            for lim in (.01, .03):
                if off > lim and f"at_{lim}" not in watch:
                    watch[f"at_{lim}"] = {"step": watch["n"], "tcp_z": round(float(T_t[2, 3]), 3),
                                          "qd_max": round(float(np.abs(robot.data.joint_vel.torch[0, rig.arm].cpu().numpy()).max()), 3)}

    rig = Rig(backend.sim, robot, T_wb, on_step=on_step, physics_step=backend.tick, lag_max=flags.lag_max_rad,
              settled_lag=flags.settled_lag_rad, update_robot=False,
              on_teleport=lambda what: hook["teleport"](what) if hook.get("teleport") else None)
    try:                                               # plans/2026-11-04-fifteen-of-fifteen.md: the hold's drive as spawned, read
        report.setdefault("grip_drive", {})["finger_joint"] = rig.set_grip_drive(flags.gripper_stiffness, flags.gripper_damping)
    except Exception as e_:                            # noqa: BLE001  back from finger_joint (written only where it differs)
        report.setdefault("grip_drive", {})["finger_joint"] = {"error": f"{type(e_).__name__}: {e_}"}
    harness = HN.Harness(backend=backend, robot=robot, tlog=tlog, flags=flags, ids=ids, arm_joint_ids=rig.arm,
                         joint_limits_arm=kin.JOINT_LIMITS, media=media)
    holder["harness"] = harness
    hook["tick"] = harness.on_tick
    hook["teleport"] = harness.on_teleport
    backend.trace = open(args.trial_dir / "trial_physics.jsonl", "w", buffering=1)
    tlog.write("meta", instance=iid, instance_file=str(args.instance), scene_from=inst.get("scene_from"), tier=tier,
               profile=flags.profile, headline=F.is_headline(flags), flags=F.to_dict(flags),
               deviations_from_headline=[list(d) for d in F.deviations(flags)], privileged=list(F.PRIVILEGED),
               mount=report["mount"], mount_file=flags.mount, argv=sys.argv, code_sha256_16=code_hashes(),
               deinstanced=holder["deinstanced"], deinstanced_root=holder.get("deinstanced_root"),
               pad_colliders=holder["pads"], authored=holder["authored"], ids=ids, kinds=kinds,
               starts={o["object_id"]: o["start"] for o in inst["objects"]},
               goal_racks={oid: g["rack"] for oid, g in inst["goal"]["objects"].items()},
               counter=inst.get("counter"), camera=ROBOT_CAMERA, video_hw=list(VIDEO_HW), video_fps=VIDEO_FPS,
               media=None if media is None else {"video": "video.mp4", "frame_every_ticks": HN.FRAME_EVERY},
               no_media_reason=None if media is not None else "run launched with --no-media (no camera sensors)",
               sample_every_ticks=HN.SAMPLE_EVERY, dt_s=backend.dt,
               sim_time="sim_s = backend tick x 1/120 s; t_trial_s counts from the reset start",
               wall_time="wall_s = time.monotonic() since the trial log opened", record=str(args.out_dir.relative_to(ROOT)))

    def T_now(oid):
        p = backend.poses()[oid]
        return T_of(p["position_m"], p["quaternion_xyzw"])

    def key(tag, label=None, renders=4):
        """A key frame of the fixed camera (+ a trial-log row); None without media."""
        if media is None:
            return None
        path = media.key_frame(tag, label, renders=renders)
        tlog.write("key_frame", tag=tag, frame=path)
        return path

    def close_still(tag):
        """A close-up of the gripper from the robot camera's side (a temporary pose of the camera rig)."""
        if "rig" not in cams:
            return None
        from PIL import Image
        import torch
        c = rig.tcp()[:3, 3]
        cam = cams["rig"].cams.get("robot")
        if cam is None:
            return None
        eye = torch.tensor([c + np.array([-.35, -.35, .12])], dtype=torch.float32)
        cam.set_world_poses_from_view(eye, torch.tensor([c], dtype=torch.float32))
        for _ in range(30):
            backend.sim.render()
        cams["rig"].update(backend.sim.get_physics_dt())
        img = cams["rig"].grab_one("robot")
        cams["rig"].apply_poses(backend.sim.device)         # back to the wide view
        path = args.trial_dir / "frames" / f"closeup_{tag}.png"
        path.parent.mkdir(parents=True, exist_ok=True)
        Image.fromarray(img).save(path)
        tlog.write("key_frame", tag=f"closeup {tag}", frame=f"frames/closeup_{tag}.png")
        return f"frames/closeup_{tag}.png"

    def still(tag):
        return key(tag, renders=24)

    # ---------------------------------------------------------------- start state (benchmark reset) + robot home
    tlog.set(phase="reset")
    backend.restore(baseline)
    for _ in range(12):
        backend.tick()
    T_home = G.tcp_frame(np.array([.25, -.35, flags.z_safe_m]), [1., 0., 0.])       # TCP down above the work area
    q_home = rig.ik(T_home, np.asarray(ur5e.HOME_Q))
    if q_home is None:
        raise RuntimeError("no IK for the home TCP pose")
    rig.teleport_arm(q_home)
    for oid in ids:
        backend.set_rigid_pose(backend.objects[oid], start_poses[oid])
    backend.loaded = True
    backend.assignments = {o["object_id"]: o["rack"] for o in inst["objects"] if o["start"] != "Counter"}
    rig.step(K.RESET_TICKS)
    now = backend.poses()
    dev = {oid: K.pose_distance(start_poses[oid], now[oid]) for oid in ids}
    reset_ok = all(d[0] <= .010 and d[1] <= 15. for d in dev.values())
    mount_pairs = harness.arm_guard()
    tlog.write("reset", reset_ok=reset_ok, reset_rule="every dish within 10 mm / 15 deg of its start (bench_kit reset)",
               deviation_mm_deg={k: [round(v[0] * 1e3, 2), round(v[1], 2)] for k, v in dev.items()},
               start_contacts=sorted("|".join(k) for k in harness.contacts.active),
               backend_contact_pairs=sorted("|".join(p) for p in backend.latest_contact["pairs"]),
               mount_pairs_exempt=[list(p) for p in mount_pairs],
               poses={oid: now[oid] for oid in ids})
    report["reset"] = {"ok": reset_ok, "worst_mm": round(max(v[0] for v in dev.values()) * 1e3, 2)}
    report["stills"] = {"start": still("start")}
    from dishsim_frigidaire.robot import collide as C
    import omni.usd
    checker = B.bench_checker()
    if not flags.basket:                                 # basket out in the arm's FCL world too (every update_components)
        report["basket_fcl"] = drop_basket_fcl(checker)
    arm = C.ArmWorld(checker, P.slab_body(checker, B.COUNTER), C.link_hulls(omni.usd.get_context().get_stage(), robot=robot),
                     C.calibrate(robot, rig.q(), T_wb), T_wb, rig.T_w3_tcp)
    bx, by, bz = mount["base"]
    arm.add_static_box("Pedestal", (bx, by, bz / 2 - .01), (.18, .18, bz - .02))   # the column, 1 cm under the base
    arm.add_static_box("Floor", (0., 0., -.05), (6., 6., .1))
    if pres:                                             # the backstop in the arm model (it stays on the counter), and the
        b_ = pres["backstop"]                            # presentation's other fixtures (the plank's lip) by their ids, as the
        arm.statics["Backstop"] = box_body(arm.fcl, "Backstop", b_["center_m"], b_["size_m"], b_["axes"], np)   # sample gate
        for f_ in extra_fixtures(pres):
            arm.statics[f_["id"]] = box_body(arm.fcl, f_["id"], f_["center_m"], f_["size_m"], f_["axes"], np)
    report["arm_model"] = {k: len(v) for k, v in arm.hulls.items()}

    def sync_world():
        now = backend.poses()
        from dishsim_frigidaire.initial_state_candidates import COMPONENT_NAMES
        checker.update_components({name: now[name] for name in COMPONENT_NAMES})
        arm.set_dishes({oid: checker._body(kinds[oid], now[oid], oid) for oid in ids})

    # ---------------------------------------------------------------- the move list
    goal_world = {oid: T_of(g["settled_pose_world"]["position_m"], g["settled_pose_world"]["quaternion_xyzw"])
                  for oid, g in inst["goal"]["objects"].items()}
    goal_rack = {oid: g["rack"] for oid, g in inst["goal"]["objects"].items()}
    order = [tuple(e) for e in inst["goal"].get("order", [])]
    support = [tuple(e) for e in inst.get("support", [])]           # (a, b): b rests on a
    upper = [oid for oid in ids if goal_rack[oid] == "UpperRack"]
    lower = [oid for oid in ids if goal_rack[oid] == "LowerRack"]

    def topo(group):
        out, left = [], list(group)
        while left:
            free = [d for d in left if not any(a in left and b == d for a, b in order)]
            pick = free[0] if free else left[0]
            out.append(pick)
            left.remove(pick)
        return out

    def stack_top_first(group):
        """Dishes something rests on go after the dish on top of them (support edges)."""
        out, left = [], list(group)
        while left:
            free = [d for d in left if not any(a == d and b in left for a, b in support)]
            pick = free[0] if free else left[0]
            out.append(pick)
            left.remove(pick)
        return out

    starts_in = {o["object_id"]: o["start"] for o in inst["objects"]}
    park_first = stack_top_first([d for d in upper if starts_in[d] != "Counter" or any(a == d for a, _ in support)])
    plan = [("park", d) for d in park_first] + [("goal", d) for d in topo(lower)] + [("extend_upper", None)] + \
           [("goal", d) for d in topo(upper)]
    report["plan"] = [[k, d] for k, d in plan]
    holder["n_moves"] = sum(1 for k, _ in plan if k != "extend_upper")
    tlog.write("note", what="plan", plan=report["plan"], n_moves=holder["n_moves"])
    parked = {}

    # ---------------------------------------------------------------- primitives
    def goal_qs(T_tcp, **kw):
        """Collision-free IK solutions of a TCP pose (elbow up, unwound inside +-190 deg: rig.py), the smallest max joint
        move from the current joints first (plans/2026-10-27-posture-fix.md; the "wrist_1 highest" rule of 2026-09-29
        retired: the collision model keeps the wrist housing off the dishes)."""
        q_now = rig.q()
        qs = [q for q in rig.ik_all(T_tcp, q_now) if arm.free(q, **kw)]
        return sorted(qs, key=lambda q: float(np.abs(q - q_now).max()))

    plan_log = []                                        # the path type of every joint plan (plans/2026-10-27-carry-rules.md)
    leg_now = {"leg": None}                              # plans/2026-11-04-bowl-carry-clearance.md: the leg being followed
    narrow = {"on": True, "budget_s": narrow_budget(args), "log": []}   # plans/2026-11-04-fifteen-of-fifteen.md: plan_to's narrow-
                                                         # passage retry (the ring moves' carry turns it off for its own, rooted
                                                         # at every passing descent start); log = every retry of the run

    def plan_to(q_goal, **kw):
        """A checked straight joint move, else RRT-Connect; True when executed. Appends "straight" / "rrt" (/ "rrt: none"
        when RRT-Connect found no path) to ``plan_log``; ``leg_now`` names the leg while it is followed. The narrow passage
        (plans/2026-11-04-fifteen-of-fifteen.md, --narrow-retry-s): no path at --transit-rrt-s -> one more search of
        narrow["budget_s"] (collide.rrt_connect's retry_s, seed 2; its first search skipped, the one above failed), recorded in
        narrow["log"]; "rrt (narrow retry)" when it found one."""
        q0 = rig.q()
        slow = .004 if kw.get("held") else .008
        k_leg = len(plan_log) + 1
        if arm.path_free(q0, q_goal, step=.02, **kw):
            plan_log.append("straight")
            leg_now["leg"] = f"leg {k_leg} (straight)"
            try:
                return rig.move_joint(q_goal, max_step=slow, settle_tol=flags.tol_joint_move_held if kw.get("held") else None)
            finally:
                leg_now["leg"] = None
        path = C.rrt_connect(arm, q0, [q_goal], time_s=args.transit_rrt_s, collide_kw=kw, shortcut_s=args.shortcut_s)
        tag_ = "rrt"
        if path is None and narrow["on"] and narrow["budget_s"] > 0:
            t_n = time.monotonic()
            path = C.rrt_connect(arm, q0, [q_goal], time_s=0., collide_kw=kw, shortcut_s=args.shortcut_s,
                                 retry_s=narrow["budget_s"])
            narrow["log"].append({"leg": k_leg, "budget_s": narrow["budget_s"], "goals": 1,
                                  "reached": C.reached_goal(path, [q_goal]), "found": path is not None,
                                  "seconds": round(time.monotonic() - t_n, 1), "phase": tlog.ctx.get("phase"),
                                  "dish": tlog.ctx.get("dish")})
            tag_ = "rrt (narrow retry)"
        plan_log.append(tag_ if path is not None else "rrt: none")
        if path is None:
            return False
        leg_now["leg"] = f"leg {k_leg} ({tag_})"
        try:
            return rig.follow(path, steps_per=10 if kw.get("held") else 5, settle_tol=flags.tol_rrt)
        finally:
            leg_now["leg"] = None

    def transit_to(T_target_hover, q_hover=None, rise_m=None, **kw):
        """Straight up to Z_SAFE, joint-plan to above the target (same IK branch as ``q_hover``), joint-plan down to
        ``q_hover`` (a single joint move dipped the carried dish into the counter; a long straight descent jumped
        IK branches, 2026-09-29). ``rise_m``: straight up by this much instead (the D-ring pick at the counter spot, where
        Z_SAFE is out of reach); 0 = no rise (a D-ring retry from where the last try left the arm)."""
        sync_world()
        dz = .002 if kw.get("held") else .004
        now = rig.tcp()
        if rise_m == 0.:
            pass
        elif rise_m is not None:
            up = now.copy()
            up[2, 3] += rise_m
            qs, why = line_free(now, up, max(20, int(rise_m / dz)), **kw)
            if qs is None or not rig.follow(qs, settle_tol=flags.tol_rise):
                return f"lifting {rise_m * 1e3:.0f} mm from the counter: {why or getattr(rig, 'last_fail', None)}"
        elif now[2, 3] < flags.z_safe_m - .01:
            up = now.copy()
            up[2, 3] = flags.z_safe_m
            qs, why = line_free(now, up, max(20, int((flags.z_safe_m - now[2, 3]) / dz)), **kw)
            if qs is None or not rig.follow(qs, settle_tol=flags.tol_rise):
                return f"lifting to the safe height: {why or getattr(rig, 'last_fail', None)}"
        if q_hover is None:
            qs = goal_qs(T_target_hover, **kw)
            if not qs:
                return "no collision-free IK at the hover"
            q_hover = qs[0]
        above = np.array(T_target_hover, copy=True)
        above[2, 3] = flags.z_safe_m
        q_above = next((q for q in [rig.ik(above, q_hover)] if q is not None and arm.free(q, **kw)), None)
        if q_above is not None and not plan_to(q_above, **kw):
            return f"no path above the target {getattr(rig, 'last_fail', None)}"
        if not plan_to(q_hover, **kw):
            return f"no path down to the hover {getattr(rig, 'last_fail', None)}"
        return None

    def line_free(T_from, T_to, n, allow=(), **kw):
        """Straight TCP line: IK waypoints (branch-continuous) that are all collision-free; ``allow`` = dishes the
        fingers may touch in the last ALLOW_CONTACT_M (the one being grasped or released)."""
        qs = rig.line_q(T_from, T_to, n, rig.q())
        if qs is None:
            return None, "IK branch jump on the line"
        d_total = float(np.linalg.norm(np.asarray(T_to)[:3, 3] - np.asarray(T_from)[:3, 3]))
        for i, q in enumerate(qs):
            left = d_total * (1 - i / max(1, len(qs) - 1))
            may = tuple(allow) if left < ALLOW_CONTACT_M else ()
            if not arm.free(q, ignore=tuple(kw.get("ignore", ())), held=kw.get("held"), gripper_may_touch=may):
                return None, (f"line collides at {round(left * 1e3)} mm from the end: "
                              f"{arm.collisions(q, ignore=tuple(kw.get('ignore', ())), held=kw.get('held'), gripper_may_touch=may)}")
        return qs, None

    fails = []

    def in_hand_window(oid, n):
        """The drift gate's window (D9): n steps with the arm still; max in-hand displacement from the window's first
        pose, and whether the dish touched anything but the gripper."""
        M0 = np.linalg.inv(rig.tcp()) @ T_now(oid)
        drift, foreign = 0., set()
        for _ in range(n):
            rig.step(1)
            M = np.linalg.inv(rig.tcp()) @ T_now(oid)
            drift = max(drift, float(np.linalg.norm(M[:3, 3] - M0[:3, 3])))
            for k_, act in harness.contacts.pairs_with(oid).items():
                if act["last"] == backend.tick_index:
                    cat = act["cats"][1] if k_[0] == oid else act["cats"][0]
                    if cat not in ("pad", "finger", "palm"):
                        foreign.add(k_[1] if k_[0] == oid else k_[0])
        return drift, not foreign, sorted(foreign)

    lift_m = .15 if args.mode == "grasp_gate" or args.attach == "none" else None   # G3 lift (None: back to the hover)

    foot = args.grasp_family.startswith("foot")
    foot_side = args.grasp_family.split("-")[1] if foot else None
    side_rank = None                                     # foot-side: the gate's [yaw_k, wrist_side] order (a load: per dish, gate_row)
    if foot and foot_side == "side":                     # plans/2026-11-04-ring-side-hold.md: the open jaws, the Kit-free order
        foot_hulls = {"side": arm.hulls["gripper"]}
        foot_yaws = []
        fc = json.loads(args.foot_candidates.read_text()) if args.foot_candidates.is_file() else {}
        side_rank = ((fc.get("gate_pick") or {}).get("ranked") if "gate_pick" in fc else
                     ((fc.get("families") or {}).get("side") or {}).get("ranked"))
        report["foot"] = {"side": foot_side, "candidates": str(args.foot_candidates), "ranked": side_rank,
                          "plan": G.foot_side_grasps(np.diag([1., -1., -1., 1.]), 1)[0]["plan"]}
    elif foot:                                           # F-in moves with the jaws CLOSED: the closed jaws' hulls (the bowl gate)
        foot_hulls = {"out": arm.hulls["gripper"], "in": closed_gripper_hulls(C, np)}
        fc = json.loads(args.foot_candidates.read_text())["families"][foot_side]
        foot_yaws = [int(k_) for k_ in fc.get("ranked_yaws", [])]          # the Kit-free ranking: the best first
        report["foot"] = {"side": foot_side, "candidates": str(args.foot_candidates), "ranked_yaws": foot_yaws,
                          "best_yaw_k": foot_yaws[0] if foot_yaws else None, "kitfree_pass": fc.get("pass"),
                          "plan": G.foot_plan(foot_side)}

    def gate_row(oid):
        """The load's Kit-free gate row of a dish (frigidaire_robot_reach.py --sample-gate's gate.json beside the load), {}."""
        if args.place_goal is None:
            return {}
        try:
            return json.loads((Path(args.place_goal).parent / "gate.json").read_text())["dishes"].get(oid) or {}
        except (OSError, ValueError, KeyError):
            return {}

    def edge_order(oid, gs):
        """The edge pinch's rolls in the Kit-free gate's order (its passing rolls first): the dish's row of the load's gate.json;
        the Isaac grasp gate (no load): the plates load's gate.json beside the presentation (the same pick at the spot)."""
        order = [str(r) for r in ((gate_row(oid).get("release") or {}).get("roll_order") or [])]
        if not order and getattr(args, "presentation", None) is not None:
            try:
                rows_ = json.loads((Path(args.presentation).parent / "gate.json").read_text())["dishes"].values()
                order = next(([str(r) for r in r_["release"]["roll_order"]] for r_ in rows_
                              if (r_.get("release") or {}).get("roll_order")), [])
            except (OSError, ValueError, KeyError):
                order = []
        report["edge_roll_order"] = order
        return sorted(gs, key=lambda g: order.index(str(g["roll_deg"])) if str(g["roll_deg"]) in order else len(order))

    def pick(oid, att, tries=6, waive=False):
        sync_world()
        T_obj = T_now(oid)
        ranked_ = None
        if foot and foot_side == "side":                               # the gate's (yaw, wrist side) order: the load's gate.json
            arm.hulls["gripper"] = foot_hulls["side"]                  # row of this dish, else the --foot-candidates order
            ranked_ = (gate_row(oid).get("pick") or {}).get("ranked") if args.place_goal is not None else side_rank
            cands = foot_side_order(G.foot_side_grasps(T_obj), ranked_)
            report.setdefault("foot_side_order", {})[oid] = ranked_
        elif foot:
            arm.hulls["gripper"] = foot_hulls[foot_side]
            gs_ = G.foot_grasps(T_obj, side=foot_side)                  # the Kit-free passing yaws in their rank order
            cands = [gs_[k_] for k_ in foot_yaws] if foot_yaws else gs_
        else:
            cands = None
        cands = cands if foot else (G.wall_grasps(T_obj, dish=kinds[oid]) if args.grasp_family == "wall" else
                 [g for w in G.WRIST_SIDES if args.side_wrist_side in (None, w)
                  for g in G.side_grasps(T_obj, below_top=args.side_below_top, dish=kinds[oid], wrist_side=w)]
                 if args.grasp_family == "side" else
                 edge_order(oid, G.edge_grasps(T_obj, pres["edge"]["rim_r_m"], pres["edge"]["edge_z_m"], pres["edge"]["out_m"]))
                 if args.grasp_family == "edge"           # the leaning plate's top-down edge pinch, both rolls (the
                 else G.plate_wall_grasps(T_obj, pres["wall"]["T_hold"], pres["wall"]["spins_deg"], pres["wall"]["rolls"])
                 if args.grasp_family == "plate-wall"     # (Kit-free gate's passing rolls first); the wall pinch at the chosen
                 else G.tilted_grasps(T_obj, dish=kinds[oid]))   # spin, the chosen roll first
        scored = []
        base_xy = np.asarray(mount["base"][:2])
        cand_why = []
        for g in cands:
            H = G.hover_along(g["T_tcp"], flags.hover_m)
            hov = goal_qs(H, **({"ignore": (oid,)} if waive else {}))[:4]
            cand_why.append(f"{len(hov)} free hover IK")
            for q_h in hov:
                qs_line = rig.line_q(H, g["T_tcp"], 100, q_h)
                if qs_line is None:
                    cand_why[-1] += "; line IK jump"
                    continue
                d = float(np.linalg.norm(g["T_tcp"][:3, 3] - H[:3, 3]))
                allow_m = (ALLOW_STRADDLE_M if g["kind"] in ("wall", "base") or args.grasp_family in ("side", "foot-side")
                           else ALLOW_CONTACT_M)                       # the open pads straddle the cup / the ring all along
                kws = [({"ignore": (oid,)} if waive else
                        {"gripper_may_touch": (oid,) if d * (1 - i / 99) < allow_m else ()}) for i in range(len(qs_line))]
                bad = None if waive else next((i for i, q in enumerate(qs_line) if not arm.free(q, **kws[i])), None)
                ok = bad is None
                if not ok:
                    cand_why[-1] += f"; line hits at {bad}/100 {arm.collisions(qs_line[bad], **kws[bad])[:2]}"
                if ok:
                    to_base = base_xy - T_obj[:2, 3]
                    side = g["T_tcp"][:2, 3] - T_obj[:2, 3]
                    facing = float(np.dot(side, to_base) / (np.linalg.norm(side) * np.linalg.norm(to_base) + 1e-9))
                    scored.append((-facing, g, q_h, qs_line))
                    break
        if not scored:
            info = []
            for g in cands[:4]:
                sg = rig.ik_all(g["T_tcp"], rig.q())
                sh = rig.ik_all(G.hover_along(g["T_tcp"], flags.hover_m), rig.q())
                info.append({"kind": g["kind"], "tilt": g.get("tilt_deg"), "tcp": np.round(g["T_tcp"][:3, 3], 3).tolist(),
                             "ik_grasp": len(sg), "hit_grasp": arm.collisions(sg[0], ignore=(oid,), first=False)[:4] if sg else None,
                             "ik_hover": len(sh), "hit_hover": arm.collisions(sh[0], first=False)[:4] if sh else None})
            return None, f"no collision-free grasp ({len(cands)} candidates): {info} per candidate: {cand_why}"
        if not ranked_:                                                # foot-side with the gate's order: kept as ranked
            scored.sort(key=lambda s: s[0])
        why = None
        tried = []
        for _, g, q_h, qs_line in scored[:tries]:
            rig.gripper(foot_side == "in")                             # F-in travels with the jaws closed
            H = G.hover_along(g["T_tcp"], flags.hover_m)
            tlog.set(phase="transit to the pre-grasp")
            why = transit_to(H, q_hover=q_h)
            if why:
                fails.append(still(f"fail_{oid}_{len(fails)}"))
                tried.append(why[:90])
                continue
            tlog.set(phase="approach")
            key("pre-grasp")
            qs = qs_line
            if not rig.follow(qs, settle_tol=flags.tol_approach):      # the open pads may touch the rim at the end
                why = f"blocked on the approach {getattr(rig, 'last_fail', None)}"
                tried.append(why[:90])
                continue
            Tn = rig.tcp()
            report.setdefault("grasp_pose_check", []).append({"dish": oid, "cmd": np.round(g["T_tcp"][:3, 3], 3).tolist(),
                "got": np.round(Tn[:3, 3], 3).tolist(), "cmd_z": np.round(g["T_tcp"][:3, 2], 2).tolist(), "got_z": np.round(Tn[:3, 2], 2).tolist()})
            if "rig" in cams and len(tried) == 0:
                report.setdefault("grasp_stills", []).append(close_still(f"preclose_{oid}"))
            partners = harness.set_carried(oid)                        # D4 window: close -> release
            tlog.set(phase="open on the ring" if foot_side == "in" else "close")
            theta = rig.gripper(foot_side != "in")                     # F-in: the open command presses the pads outward
            key("close")
            if "rig" in cams and len(tried) == 0:
                report.setdefault("grasp_stills", []).append(close_still(f"closed_{oid}"))
            tlog.write("note", what="opened" if foot_side == "in" else "closed", jaw_rad=round(theta, 4), at_close_partners=partners,
                       grasp=g["kind"], tilt_deg=g.get("tilt_deg"))
            jw = (G.FOOT_JAW_RAD[foot_side] if foot else
                  G.WALL_JAW_RAD if g["kind"] in ("wall", "base") or args.grasp_family == "side" else flags.jaw_wall_rad)
            if not (jw[0] <= theta < jw[1]):         # the jaws closed on air or on the bowl body
                why = f"jaws at {theta:.2f} rad: no wall between the pads"
                tried.append(why)
                harness.clear_carried()
                rig.gripper(False)
                rig.move_line(H, n=60)
                continue
            z0 = T_now(oid)[2, 3]
            M_close = np.linalg.inv(rig.tcp()) @ T_now(oid)
            tlog.set(phase="lift")
            H_up = ((G.hover(g["T_tcp"], lift_m) if args.grasp_family in ("side", "edge", "plate-wall", "foot-side")
                     else G.hover_along(g["T_tcp"], lift_m))
                    if lift_m else H)                                  # the side and edge pinches lift straight up
            qs_up = rig.line_q(rig.tcp(), H_up, 80, rig.q())
            ok_lift = qs_up is not None and rig.follow(qs_up, settle_tol=flags.tol_lift)   # the dish swings on the pinch
            key("lift-off")
            tlog.set(phase="in-hand settle")
            rig.step(flags.in_hand_settle_ticks)                   # let it settle in the hand (the pivot)
            T_t, T_o = rig.tcp(), T_now(oid)
            M_set = np.linalg.inv(T_t) @ T_o
            gate = {"rise_mm": round(float(T_o[2, 3] - z0) * 1e3, 2),
                    "dist_to_tcp_mm": round(float(np.linalg.norm(T_o[:3, 3] - T_t[:3, 3])) * 1e3, 2),
                    "settle_displacement_mm": round(float(np.linalg.norm(M_set[:3, 3] - M_close[:3, 3])) * 1e3, 2),
                    "settle_rotation_deg": round(R.rot_angle_deg(M_close, M_set), 2), "jaw_rad": round(theta, 4),
                    "lift_ok": bool(ok_lift), "in_hand_settle_ticks": flags.in_hand_settle_ticks}
            if flags.hold_gate == "drift":
                tlog.set(phase="hold window")
                n = round(flags.hold_window_s / backend.dt)
                drift, in_hand, foreign = in_hand_window(oid, n)
                T_t, T_o = rig.tcp(), T_now(oid)
                held, rule = F.hold_verdict(flags, in_hand=in_hand, window_s=n * backend.dt, window_drift_m=drift)
                gate.update(window_s=round(n * backend.dt, 4), drift_mm=round(drift * 1e3, 3), in_hand=in_hand,
                            foreign_contacts=foreign)
                if foot:                                       # the contacts at the end of the hold (pads, knuckles, ...)
                    gate.update(pad_N_hold={k_: round(float(v), 3) for k_, v in harness.contacts.pad_N.items()},
                                contacts_end={"|".join(k_): {"cats": list(v["cats"]), "peak_N": round(float(v["peak_N"]), 3),
                                                             "since_tick": v["since"], "last_tick": v["last"]}
                                              for k_, v in harness.contacts.pairs_with(oid).items()})
            else:
                held, rule = F.hold_verdict(flags, rise_m=float(T_o[2, 3] - z0),
                                            dist_to_tcp_m=float(np.linalg.norm(T_o[:3, 3] - T_t[:3, 3])))
            gate.update(held=held, rule=rule)
            tlog.write("hold", **gate)
            att.setdefault("holds", []).append(gate)
            if held:
                return {"grasp": g["kind"], "yaw": g["yaw"], "tilt_deg": g.get("tilt_deg"), "theta": theta,
                        "below_top_m": g.get("below_top"), "wrist_side": g.get("wrist_side"), "M": np.linalg.inv(T_t) @ T_o,
                        "roll_deg": g.get("roll_deg"), "T_tcp_planned": np.round(g["T_tcp"], 6).tolist(),
                        **({"yaw_k": g["yaw_k"], "side": g["side"], "theta_contact_planned": round(g["plan"]["theta_contact"], 4),
                            "engaged_mm_planned": round(g["plan"]["engaged_m"] * 1e3, 2)} if foot else {})}, None
            why = (f"not held after the lift (jaws {theta:.2f}, lift ok {ok_lift}, dish rose {1e3 * (T_o[2, 3] - z0):.0f} mm, "
                   f"{1e3 * np.linalg.norm(T_o[:3, 3] - T_t[:3, 3]):.0f} mm from the TCP; gate {rule})")
            tried.append(why)
            harness.clear_carried()
            rig.gripper(False)
            rig.step(60)
        return None, f"{why} (tried {len(tried)}: {tried})"

    SLIDE_M, SLIDE_START_OUT_M, SLIDE_LIFT_ABOVE_M = .040, .010, .080

    def slide_pick(oid, att):
        """Voysey 2021's surface slide on a flat dish (round 3, the plate): the closing axis vertical, the open gripper's
        lowest point 2 mm above the counter, the fingertips (TCP) SLIDE_START_OUT_M outside the rim, a 40 mm slide toward
        the centre, close on the edge, lift 0.15 m, the hold gate. The 15 mm clearance stays on for the arm links and for
        the hover; only the gripper may come near the counter and the plate on the descent and the slide."""
        sync_world()
        T_obj = T_now(oid)
        o = T_obj[:3, 3]
        rim_r = G.KIND_RIM[kinds[oid]][0]
        inv_tcp = np.linalg.inv(rig.T_w3_tcp)
        verts = np.concatenate([np.asarray(v) for _, v in arm.hulls["gripper"]])
        v_tcp = (inv_tcp @ np.c_[verts, np.ones(len(verts))].T).T[:, :3]
        d_low = float(-v_tcp[:, 1].min())                   # open gripper below the TCP along -y (y = up below)
        top = B.COUNTER["top_z_m"]
        base_xy = np.asarray(mount["base"][:2])
        cands = []
        for k in range(12):
            phi = 2 * np.pi * k / 12
            n = np.array([np.cos(phi), np.sin(phi), 0.])
            T = np.eye(4)
            T[:3, 1] = [0., 0., 1.]                          # closing axis vertical: the jaws horizontal
            T[:3, 2] = -n                                    # approach toward the plate centre
            T[:3, 0] = np.cross(T[:3, 1], T[:3, 2])
            T[:3, 3] = o + (rim_r + SLIDE_START_OUT_M) * n
            T[2, 3] = top + .002 + d_low
            T_end = T.copy()
            T_end[:3, 3] -= SLIDE_M * n
            facing = float(np.dot(n[:2], base_xy - o[:2]) / (np.linalg.norm(base_xy - o[:2]) + 1e-9))
            cands.append((-facing, k, T, T_end))
        cands.sort(key=lambda c: c[0])
        allow = {"gripper_may_touch": ("Counter", oid)}
        info = []
        for _, k, T, T_end in cands:
            H = T.copy()
            H[2, 3] += SLIDE_LIFT_ABOVE_M
            hov = goal_qs(H)
            if not hov:
                info.append(f"yaw {k}: no collision-free hover IK")
                continue
            line = rig.line_q(H, T, 50, hov[0])
            slide = None if line is None else rig.line_q(T, T_end, 40, line[-1])
            if line is None or slide is None:
                info.append(f"yaw {k}: IK branch jump on the descent or the slide")
                continue
            bad = next((q for q in line + slide if not arm.free(q, **allow)), None)
            if bad is not None:
                info.append(f"yaw {k}: {arm.collisions(bad, **allow)[:2]}")
                continue
            tlog.set(phase="transit to the slide start")
            why = transit_to(H, q_hover=hov[0])
            if why:
                return None, f"no approach: {why}"
            tlog.set(phase="slide descent")
            qs_d = rig.line_q(rig.tcp(), T, 50, rig.q())
            if qs_d is None or not rig.follow(qs_d, settle_tol=flags.tol_approach):
                return None, f"no approach: blocked on the descent {getattr(rig, 'last_fail', None)}"
            tlog.set(phase="slide")
            qs_s = rig.line_q(rig.tcp(), T_end, 40, rig.q())
            slid = qs_s is not None and rig.follow(qs_s, settle_tol=flags.tol_approach)
            partners = harness.set_carried(oid)
            tlog.set(phase="close")
            theta = rig.gripper(True)
            tlog.write("note", what="closed (slide)", jaw_rad=round(theta, 4), at_close_partners=partners, yaw=k, slid=bool(slid))
            if not (flags.jaw_wall_rad[0] <= theta < flags.jaw_wall_rad[1]):
                harness.clear_carried()
                rig.gripper(False)
                return None, f"jaws at {theta:.2f} rad: no wall between the pads (slide yaw {k}, slid {bool(slid)})"
            z0 = T_now(oid)[2, 3]
            M_close = np.linalg.inv(rig.tcp()) @ T_now(oid)
            tlog.set(phase="lift")
            up = rig.tcp()
            up[2, 3] += .15
            qs_up = rig.line_q(rig.tcp(), up, 80, rig.q())
            ok_lift = qs_up is not None and rig.follow(qs_up, settle_tol=flags.tol_lift)
            tlog.set(phase="in-hand settle")
            rig.step(flags.in_hand_settle_ticks)
            T_t, T_o = rig.tcp(), T_now(oid)
            M_set = np.linalg.inv(T_t) @ T_o
            gate = {"rise_mm": round(float(T_o[2, 3] - z0) * 1e3, 2), "slid": bool(slid), "yaw": k,
                    "dist_to_tcp_mm": round(float(np.linalg.norm(T_o[:3, 3] - T_t[:3, 3])) * 1e3, 2),
                    "settle_displacement_mm": round(float(np.linalg.norm(M_set[:3, 3] - M_close[:3, 3])) * 1e3, 2),
                    "settle_rotation_deg": round(R.rot_angle_deg(M_close, M_set), 2), "jaw_rad": round(theta, 4),
                    "lift_ok": bool(ok_lift), "in_hand_settle_ticks": flags.in_hand_settle_ticks, "d_low_mm": round(d_low * 1e3, 1)}
            tlog.set(phase="hold window")
            nh = round(flags.hold_window_s / backend.dt)
            drift, in_hand, foreign = in_hand_window(oid, nh)
            held, rule = F.hold_verdict(flags, in_hand=in_hand, window_s=nh * backend.dt, window_drift_m=drift)
            gate.update(window_s=round(nh * backend.dt, 4), drift_mm=round(drift * 1e3, 3), in_hand=in_hand,
                        foreign_contacts=foreign, held=held, rule=rule)
            tlog.write("hold", **gate)
            att.setdefault("holds", []).append(gate)
            if held:
                return {"grasp": "slide", "yaw": float(k), "tilt_deg": None, "theta": theta, "M": np.linalg.inv(T_t) @ T_o}, None
            harness.clear_carried()
            rig.gripper(False)
            rig.step(60)
            return None, (f"not held after the lift (slide yaw {k}, jaws {theta:.2f}, lift ok {ok_lift}, dish rose "
                          f"{1e3 * (T_o[2, 3] - z0):.0f} mm; gate {rule})")
        return None, f"no collision-free slide approach (12 yaws): {info}"

    def release_and_retreat(H):
        harness.clear_carried()                                    # D4 window ends at the release command
        tlog.set(phase="release")
        rig.gripper(False)
        key("release")
        tlog.set(phase="retreat")

    def place(oid, T_target, held):
        T_tcp_place = np.asarray(T_target) @ np.linalg.inv(held["M"])
        carry = {"held": (kinds[oid], held["M"], oid), "ignore": (oid,)}
        T_rel = T_tcp_place.copy()
        T_rel[2, 3] += flags.release_above_m
        H = G.hover_along(T_tcp_place, flags.hover_m)
        tlog.set(phase="transport")
        why = transit_to(H, **carry)
        if why:
            fails.append(still(f"fail_place_{oid}_{len(fails)}"))
            return why
        qs, why = line_free(rig.tcp(), T_rel, 60, **carry)
        if qs is None:
            return why or "blocked descending to the place pose (no line)"
        tlog.set(phase="insertion")
        key("insertion start")
        if not rig.follow(qs):
            return why or f"blocked descending to the place pose {getattr(rig, 'last_fail', None)}"
        release_and_retreat(H)
        qs_up = rig.line_q(rig.tcp(), H, 40, rig.q())
        if qs_up is not None:
            rig.follow(qs_up)
        if flags.move_settle == "legacy":
            rig.step(120)
        return None

    def place_down(oid, T_target, held, depth=.12):
        """R5 (legacy): release pose = the target lowered until the dish meets the rack (arm blocked) or ``depth``
        below it. The dish may swing further on the pinch during the transit: it is re-measured above the spot and
        the hand re-aimed so the dish (not the hand) is over the target."""
        T_tcp_place = np.asarray(T_target) @ np.linalg.inv(held["M"])
        carry = {"held": (kinds[oid], held["M"], oid), "ignore": (oid,)}
        H = G.hover_along(T_tcp_place, flags.hover_m)
        watch.clear()
        watch.update(oid=oid, M=np.array(held["M"], copy=True))
        tlog.set(phase="transport")
        why = transit_to(H, **carry)
        held["transit_watch"] = {k: v for k, v in watch.items() if k not in ("oid", "M")}
        watch.clear()
        if why:
            fails.append(still(f"fail_place_{oid}_{len(fails)}"))
            return why
        rig.step(90)
        if np.linalg.norm(T_now(oid)[:3, 3] - rig.tcp()[:3, 3]) > .2:
            fails.append(still(f"dropped_{oid}_{len(fails)}"))
            return f"dropped in transit {held.get('transit_watch')}"
        held["M"] = np.linalg.inv(rig.tcp()) @ T_now(oid)         # the in-hand pose after the transit
        T_target = np.array(T_target, copy=True)
        T_target[:3, :3] = (rig.tcp() @ held["M"])[:3, :3]
        T_tcp_place = T_target @ np.linalg.inv(held["M"])
        H = G.hover_along(T_tcp_place, flags.hover_m)
        fix = rig.line_q(rig.tcp(), H, 40, rig.q())
        if fix is not None:
            rig.follow(fix, settle_tol=flags.tol_reaim)
        low = T_tcp_place.copy()
        low[2, 3] -= depth
        qs = rig.line_q(rig.tcp(), low, 90, rig.q())
        if qs is None:
            return "IK branch jump lowering to the rack"
        rel0 = rig.tcp()[2, 3] - T_now(oid)[2, 3]
        contact = None
        tlog.set(phase="insertion")
        key("insertion start")
        for i, q in enumerate(qs):                    # stop at the first contact: the dish stops following the hand
            rig.set_arm(q)
            rig.step(3)
            closer = rel0 - (rig.tcp()[2, 3] - T_now(oid)[2, 3])     # the hand came down onto the resting dish
            if rig.lag() > flags.place_down_contact_lag or closer > .004:
                contact = {"waypoint": i, "of": len(qs), "hand_closer_mm": round(closer * 1e3, 1), "lag": round(rig.lag(), 3)}
                break
        held["contact"] = contact
        held["T_release"] = T_now(oid)                # the dish pose when the pads open (limits round 2 judge)
        release_and_retreat(H)
        up = rig.line_q(rig.tcp(), H, 50, rig.q())
        if up is not None:
            rig.follow(up)
        rig.step(120)
        return None

    placed_at_goal = set()

    def at_goal(k, pose):
        if k not in goal_world:
            return False
        T = T_of(pose["position_m"], pose["quaternion_xyzw"])
        return bool(B.within_goal(kinds[k], B.goal_distance(centroids[kinds[k]], T, goal_world[k])))

    def disturbance(oid, before, now):
        """Both rules on the same data: ``strict`` = any other dish beyond 10 mm / 20 deg (the old episode rule);
        ``fatal``/``nudged`` = the benchmark's rule (D1): fatal only when the neighbour was at its goal before the
        move or ends outside the counter and the racks (frigidaire_bench_kit.BenchOracle._fatal)."""
        moved = []
        for k in ids:
            if k == oid:
                continue
            d = K.pose_distance(before[k], now[k])
            if d[0] * 1e3 > DISTURB_MM or d[1] > DISTURB_DEG:
                moved.append([k, round(d[0] * 1e3, 1), round(d[1], 1)])
        protected = {k for k in ids if k != oid and (k in placed_at_goal or at_goal(k, before[k]))}
        frames = {r: now[r] for r in ("LowerRack", "UpperRack")}
        fatal = [m[0] for m in moved if m[0] in protected or (
            B.racked_in(points[kinds[m[0]]], now[m[0]], frames) is None
            and not P.in_counter_band(T_of(now[m[0]]["position_m"], now[m[0]]["quaternion_xyzw"]), B.COUNTER))]
        return {"strict": moved, "fatal": fatal, "nudged": [m[0] for m in moved if m[0] not in fatal],
                "protected": sorted(protected)}

    def judge_racked(oid, rack, before):
        """R6 (legacy): in the intended rack, drift over 120 ticks (recorded, never gated), any-neighbour rule."""
        now = backend.poses()
        frames = {r: now[r] for r in ("LowerRack", "UpperRack")}
        got = B.racked_in(points[kinds[oid]], now[oid], frames)
        rig.step(120)
        later = backend.poses()
        drift = K.pose_distance(now[oid], later[oid])
        disturbed = [[o, round(K.pose_distance(before[o], later[o])[0] * 1e3, 1)] for o in ids if o != oid
                     and (K.pose_distance(before[o], later[o])[0] * 1e3 > DISTURB_MM or K.pose_distance(before[o], later[o])[1] > DISTURB_DEG)]
        return {"at_target": got == rack, "racked_in": got, "drift_mm": round(drift[0] * 1e3, 1), "disturbed": disturbed,
                "distance": None, "rules": disturbance(oid, before, later)}

    def judge_any(oid, T_target, before):
        """The old episode judge (legacy parks and goals): at-goal tolerance vs the target + any-neighbour rule."""
        now = backend.poses()
        T = T_of(now[oid]["position_m"], now[oid]["quaternion_xyzw"])
        dist = B.goal_distance(centroids[kinds[oid]], T, T_target)
        disturbed = []
        for other in ids:
            if other == oid:
                continue
            d = K.pose_distance(before[other], now[other])
            if d[0] * 1e3 > DISTURB_MM or d[1] > DISTURB_DEG:
                disturbed.append([other, round(d[0] * 1e3, 1), round(d[1], 1)])
        return {"at_target": bool(B.within_goal(kinds[oid], dist)), "distance": [round(dist[0] * 1e3, 1), round(dist[1] * 1e3, 1),
                round(dist[2], 1)], "disturbed": disturbed, "rules": disturbance(oid, before, now)}

    def bench_settle(oid, kind_, T_target, before):
        """D3 headline: the benchmark's per-move settle (150 ticks, drift of the moved dish over the last 60 ticks
        <= 5 mm / 3 deg, settle deviation <= 0.08 m, contact depth < 2 mm) and the move verdict: at the goal
        (tolerance) or on the counter (park), no fatal disturbance (D1)."""
        tlog.set(phase="settle")
        hist = []
        for s in range(K.SETTLE_TICKS):
            rig.step(1)
            if s >= K.SETTLE_TICKS - K.WINDOW_TICKS:
                hist.append(T_now(oid))
        now = backend.poses()
        T = T_of(now[oid]["position_m"], now[oid]["quaternion_xyzw"])
        drift_p = float(np.linalg.norm(hist[-1][:3, 3] - hist[0][:3, 3]))
        drift_deg = R.rot_angle_deg(hist[0], hist[-1])
        dev_p = float(np.linalg.norm(T[:3, 3] - np.asarray(T_target)[:3, 3]))
        deep = max((d for name, d in backend.latest_contact["depths_m"].items() if oid in name.split("|")), default=0.)
        settled = (drift_p <= R.STABLE_POS_M and drift_deg <= R.STABLE_ROT_DEG and dev_p <= K.DEV_MAX_M
                   and deep < LIMITS["peak_penetration_m"])
        key("after settle")
        out = {"settle": {"ticks": K.SETTLE_TICKS, "window_ticks": K.WINDOW_TICKS, "drift_mm": round(drift_p * 1e3, 2),
                          "drift_deg": round(drift_deg, 2), "settle_dev_mm": round(dev_p * 1e3, 1),
                          "contact_depth_mm": round(deep * 1e3, 2), "passed": bool(settled),
                          "rule": "rearrange.STABLE_POS_M 5 mm / STABLE_ROT_DEG 3 deg, bench_kit.DEV_MAX_M 0.08 m, "
                                  "LIMITS peak_penetration_m 2 mm"},
               "rules": disturbance(oid, before, now)}
        if kind_ == "goal":
            dist = B.goal_distance(centroids[kinds[oid]], T, goal_world[oid])
            out.update(at_target=bool(B.within_goal(kinds[oid], dist)),
                       distance=[round(dist[0] * 1e3, 1), round(dist[1] * 1e3, 1), round(dist[2], 1)],
                       tolerance=list(B.AT_GOAL[kinds[oid]]))
        else:
            out.update(at_target=bool(P.in_counter_band(T, B.COUNTER)), distance=None, tolerance="counter band")
        out["disturbed"] = out["rules"]["fatal"]
        tlog.write("settle", what="per-move settle (D3)", **out)
        return out

    def end_check(now):
        """The benchmark's end check, copied from frigidaire_bench_kit.run() (a closure there, not importable): tub
        wall on the extended state, then both racks retracted (the other order after a rack-speed flake, with the
        benchmark's put-back) and whole-mesh containment (backend.close_and_contain). Scoring, not execution."""
        frames = {r: now[r] for r in ("LowerRack", "UpperRack")}
        racked = {oid: B.racked_in(points[kinds[oid]], now[oid], frames) for oid in ids}
        if any(v is None for v in racked.values()):
            return {"ran": False, "outcome": "not_all_racked", "racked": racked}
        tub = {oid: B.tub_clear(points[kinds[oid]], *(lambda l: (l["position_m"], l["quaternion_xyzw"]))(
            P.local_from_world(frames[racked[oid]], now[oid]))) for oid in ids}
        if not all(tub.values()):
            return {"ran": True, "outcome": "tub_wall", "racked": racked, "tub_clear": tub}
        tlog.set(phase="end check")
        backend.assignments = dict(racked)
        prepared = [{"object_id": oid, "kind": kinds[oid]} for oid in ids]
        pre = {oid: now[oid] for oid in ids}
        tried = []
        for order_ in (("UpperRack", "LowerRack"), ("LowerRack", "UpperRack")):
            rec_ = {"loaded_rack_motions": []}
            backend.maximum_penetration_m, backend.peak_event, backend.global_peak_event = 0., None, None  # clean record
            backend.close_and_contain(rec_, order_, prepared)
            deepest = sorted(backend.latest_contact["depths_m"].items(), key=lambda kv: -kv[1])[:3]
            tried.append({"order": list(order_), "outcome": rec_.get("outcome"), "reason": rec_.get("reason"),
                          "deepest_contacts_mm": [[name, round(depth * 1e3, 2)] for name, depth in deepest],
                          "motions": [{k_: m.get(k_) for k_ in ("rack", "passed", "reason", "failed_step", "unsupported_object_ids",
                                                                 "maximum_penetration_m", "peak_measured_rack_speed_m_s")}
                                      for m in rec_.get("loaded_rack_motions", [])]})
            if rec_.get("outcome") != "closure_failure":
                break
            backend.ramp({RACK_JOINT[r]: EXTENSION[r] for r in ("LowerRack", "UpperRack")}, "reextend_after_flake")
            for oid in ids:
                backend.set_rigid_pose(backend.objects[oid], pre[oid])
            for _ in range(K.RESET_TICKS):
                backend.tick()
        key("end check")
        return {"ran": True, "outcome": rec_.get("outcome"), "reason": rec_.get("reason"), "orders_tried": tried,
                "racked": racked, "tub_clear": tub,
                "containment": {k_: v["contained"] for k_, v in rec_.get("final_containment", {}).items()},
                "rack_speed_flakes": sum(t["outcome"] == "closure_failure" for t in tried[:-1])}

    def diag_after_move():
        """The plan's velocity settle routine (D3: diagnostic column). Active (steps <= 5 s) in the headline profile;
        the legacy profile keeps its old timeline and records only the dish speeds at this instant."""
        if flags.diag_settle_active:
            res = harness.diag_settle(rig.step)
        else:
            sp = harness.dish_speeds()
            res = {"settled": None, "passive": True, "peak_v_m_s": round(max(v[0] for v in sp.values()), 5),
                   "peak_w_rad_s": round(max(v[1] for v in sp.values()), 5), "rule": dict(HN.DIAG_SETTLE)}
        tlog.write("settle", what="diagnostic velocity routine (plan Phase 0.3; D3: never a gate)", **res)
        return res

    def cause_of(rec):
        for att in rec.get("attempts", [])[::-1]:
            if att.get("invariants"):
                first = att["invariants"][0]
                return f"invariant: {first}" + ("" if att.get("ok_physical") is None else f" (physical outcome: {'ok' if att.get('ok_physical') else att.get('physical_cause')})")
            if att.get("physical_cause"):
                return att["physical_cause"]
        return rec.get("cause") or "failed"

    def physical_cause(att, kind_):
        if att.get("pick") is None:
            return att.get("why") or "pick failed"
        if att.get("place_why"):
            return att["place_why"]
        j = att.get("judge") or {}
        if j.get("settle") and not j["settle"]["passed"]:
            s = j["settle"]
            return f"unstable after release (drift {s['drift_mm']} mm / {s['drift_deg']} deg, dev {s['settle_dev_mm']} mm, depth {s['contact_depth_mm']} mm)"
        if j.get("disturbed"):
            return f"disturbed neighbour {j['disturbed']}"
        if not j.get("at_target"):
            if kind_ == "park":
                return "off the counter after the park"
            if flags.judge == "racked_in":
                return f"not racked in {goal_rack.get(att.get('dish'))} (got {j.get('racked_in')})"
            return f"out of tolerance {j.get('distance')} (lat mm, dz mm, tilt deg; limits {j.get('tolerance')})"
        return None

    def hover_diag(hovers=(.10, .125, .15)):
        """Isaac's side of the hover disagreement: per grasp and hover, the IK count, the free hover IK and whether one
        approach line is free (the pick's own rule), plus the arm model's calibration and hull bounds."""
        sync_world()
        oid = ids[0]
        T_obj = T_now(oid)
        out = {"T_obj": T_obj.tolist(), "rig_q": rig.q().tolist(), "T_wb": T_wb.tolist(),
               "X_link": {k: np.asarray(v).tolist() for k, v in arm.X.items()},
               "hull_bounds": {k: [np.min([np.min(v, 0) for _, v in lst], 0).tolist(), np.max([np.max(v, 0) for _, v in lst], 0).tolist(), len(lst)]
                               for k, lst in arm.hulls.items()}, "hovers": {}}
        for hv in hovers:
            rows_ = []
            for g in G.tilted_grasps(T_obj, dish=kinds[oid]):
                H = G.hover_along(g["T_tcp"], hv)
                qs = goal_qs(H)
                ok, why_ = False, None
                for q_h in qs[:4]:
                    line = rig.line_q(H, g["T_tcp"], 100, q_h)
                    if line is None:
                        why_ = "IK jump"
                        continue
                    d = float(np.linalg.norm(g["T_tcp"][:3, 3] - H[:3, 3]))
                    bad = next((i for i, q in enumerate(line) if not arm.free(
                        q, gripper_may_touch=(oid,) if d * (1 - i / 99) < ALLOW_CONTACT_M else ())), None)
                    if bad is None:
                        ok = True
                        break
                    why_ = f"hits at {bad}: {arm.collisions(line[bad], gripper_may_touch=(oid,))[:2]}"
                rows_.append({"yaw": g["yaw"], "ik": len(rig.ik_all(H, rig.q())), "free_hover": len(qs), "approach_ok": ok,
                              "why": why_, "q_free": [q.tolist() for q in qs[:4]]})
            out["hovers"][f"{hv:.3f}"] = rows_
        return out

    if args.mode == "place_goal":                 # Part B: every closure above, by name (types.SimpleNamespace)
        import types
        return place_goal_trials(args, report, flags, types.SimpleNamespace(**locals()))
    if args.mode:
        return limits_trials(args, report, backend, robot, rig, harness, tlog, hook, inst, kinds, ids, goal_world, q_home,
                             pick, place, bench_settle, T_now, np, K, B, EXTENSION, RACK_JOINT, T_of, place_down, hover_diag,
                             slide_pick, narrow=narrow)

    # ---------------------------------------------------------------- phase L: upper rack in (R3, scripted)
    t_run = time.monotonic()
    s_run = backend.tick_index
    report["upper_rack"] = []
    failed_dishes = {}
    refusals = 0
    n_done = 0
    try:
        if flags.upper_in_for_lower:
            tlog.set(phase="rack")
            backend.ramp({RACK_JOINT["UpperRack"]: 0.}, "robot_upper_in")
            report["upper_rack"].append({"at_move": 0, "state": "in"})
            tlog.write("rack", rack="UpperRack", state="in", joints=backend.joints(),
                       declared="R3: scripted environment action (the extended upper rack covers the lower rack)")
        for kind_, oid in plan:
            if args.max_moves is not None and n_done >= args.max_moves:
                report["stopped"] = f"--max-moves {args.max_moves}"
                break
            if kind_ == "extend_upper":
                tlog.set(move=None, dish=None, attempt=None, phase="rack")
                rig.gripper(False)
                transit_to(T_home)
                backend.ramp({RACK_JOINT["UpperRack"]: EXTENSION["UpperRack"]}, "robot_upper_out")
                report["upper_rack"].append({"at_move": n_done, "state": "out"})
                tlog.write("rack", rack="UpperRack", state="out", joints=backend.joints(), declared="R3: scripted environment action")
                continue
            tlog.set(move=n_done + 1, dish=oid, attempt=None, phase="move")
            rec = {"move": n_done + 1, "dish": oid, "kind": kind_, "attempts": []}
            t0, s0 = time.monotonic(), backend.tick_index
            if oid in failed_dishes:                                       # D18: skip a dish after its two attempts
                rec.update(ok=False, skipped=True, cause=f"skipped: the dish failed at move {failed_dishes[oid]} (D18)")
            elif kind_ == "park" and flags.counter_cap:                    # D19: the benchmark's counter cap
                now = backend.poses()
                count = sum(P.in_counter_band(T_of(now[k]["position_m"], now[k]["quaternion_xyzw"]), B.COUNTER) for k in ids)
                if (not P.in_counter_band(T_now(oid), B.COUNTER)) and count >= inst["counter"]["cap"]:
                    refusals += 1
                    tlog.write("refusal", kind="counter-full", counter_count=int(count), cap=inst["counter"]["cap"])
                    rec.update(ok=False, refused="counter-full", cause=f"counter-full: {count} dishes on the counter, cap {inst['counter']['cap']} (D19)")
            if "ok" not in rec:
                if kind_ == "park":
                    spot = next(s for s in flags.park_spots if tuple(s) not in parked.values())
                    parked[oid] = tuple(spot)
                for attempt in range(2):
                    tlog.set(attempt=attempt, phase="pick")
                    before = backend.poses()
                    n_viol = len(harness.violations)
                    att = {"attempt": attempt, "dish": oid}
                    try:
                        held, why = pick(oid, att)
                        att.update(pick=None if held is None else {k: v for k, v in held.items() if k != "M"}, why=why)
                        if held is not None:
                            if kind_ == "park":
                                # level the dish in the hand's current up/down sense, rim or foot 3 mm above the counter
                                T_o = rig.tcp() @ held["M"]
                                up = 1. if T_o[2, 2] >= 0 else -1.
                                yz = math.atan2(T_o[1, 0], T_o[0, 0])             # keep the dish's current yaw (less wrist motion)
                                Rz = np.array([[math.cos(yz), -math.sin(yz), 0], [math.sin(yz), math.cos(yz), 0], [0, 0, 1]])
                                T_target = np.eye(4)
                                T_target[:3, :3] = Rz @ (np.diag([1., up, up]) if up < 0 else np.eye(3))
                                base_z = B.COUNTER["top_z_m"] + .003 + (G.HEIGHT if up < 0 else 0.)
                                T_target[:3, 3] = (spot[0], spot[1], base_z)
                            else:
                                T_target = goal_world[oid]
                            if kind_ == "goal" and flags.place_mode == "lower_until_contact":   # R5 (benchmark relaxation)
                                T_target = np.array(rig.tcp() @ held["M"], copy=True)   # lower it as it hangs: same rotation,
                                T_target[:3, 3] = goal_world[oid][:3, 3]                 # released above the rack spot
                                why = place_down(oid, T_target, held)
                            else:
                                why = place(oid, T_target, held)
                            att["place_why"] = why
                            if flags.move_settle == "benchmark":
                                att["judge"] = bench_settle(oid, kind_, T_target, before)
                                ok = why is None and att["judge"]["settle"]["passed"] and att["judge"]["at_target"] and not att["judge"]["disturbed"]
                            elif kind_ == "goal" and flags.judge == "racked_in":                       # R6 (benchmark relaxation)
                                att["judge"] = judge_racked(oid, goal_rack[oid], before)
                                key("after settle")
                                ok = why is None and att["judge"]["at_target"] and not att["judge"]["disturbed"]
                            else:
                                att["judge"] = judge_any(oid, T_target, before)
                                key("after settle")
                                ok = why is None and (att["judge"]["at_target"] or kind_ == "park") and not att["judge"]["disturbed"]
                                if kind_ == "park" and why is None:
                                    ok = not att["judge"]["disturbed"] and B.COUNTER["top_z_m"] - .01 < T_now(oid)[2, 3] < B.COUNTER["top_z_m"] + .2
                            att["ok_physical"] = bool(ok)
                            att["diag_settle"] = diag_after_move()
                        else:
                            att["ok_physical"] = False
                    except HN.InvariantAbort as exc:
                        att.update(ok_physical=None, aborted_by_invariant=str(exc))
                    viol = harness.violations[n_viol:]
                    att["invariants"] = sorted({v["name"] for v in viol})
                    att["invariant_lines"] = sorted({v["line"] for v in viol if v.get("line")})
                    if not att["ok_physical"]:
                        att["physical_cause"] = physical_cause(att, kind_) if att["ok_physical"] is False else "stopped by the invariant"
                    att["ok"] = bool(att["ok_physical"]) and not att["invariants"]
                    att["sim_s"] = round((backend.tick_index - s0) * backend.dt, 3)
                    rec["attempts"].append(att)
                    if att.get("aborted_by_invariant"):
                        break
                    if att["ok_physical"]:
                        break                          # an invariant after a physically good move fails it, never re-picks it
                    rig.gripper(False)
                    harness.clear_carried()
                rec["ok_physical"] = bool(rec["attempts"][-1].get("ok_physical"))
                rec["ok"] = rec["ok_physical"] and not any(a.get("invariants") for a in rec["attempts"])
                if not rec["ok"]:
                    rec["cause"] = cause_of(rec)
            rec.setdefault("ok_physical", False)
            rec["failure_class"] = None if rec["ok"] else failure_class(rec.get("cause"))
            rec["seconds_wall"] = round(time.monotonic() - t0, 1)
            rec["sim_s"] = round((backend.tick_index - s0) * backend.dt, 3)
            now = backend.poses()
            rec["poses_after"] = {k: now[k] for k in ids}
            report["moves"].append(rec)
            n_done += 1
            if rec["ok"] and kind_ == "goal":
                placed_at_goal.add(oid)
            tlog.write("move", kind=kind_, ok=rec["ok"], ok_physical=rec["ok_physical"], cause=rec.get("cause"), failure_class=rec["failure_class"],
                       attempts=len(rec["attempts"]), skipped=rec.get("skipped", False), refused=rec.get("refused"),
                       move_sim_s=rec["sim_s"], move_wall_s=rec["seconds_wall"],
                       invariants=sorted({n for a in rec["attempts"] for n in a.get("invariants", [])}))
            print(f"[INFO] move {n_done} {kind_} {oid}: {'ok' if rec['ok'] else 'FAIL'} "
                  f"{[(a.get('why'), a.get('place_why'), (a.get('judge') or {}).get('distance'), a.get('invariants')) for a in rec['attempts']]}"
                  f"{'' if rec['ok'] else ' cause: ' + str(rec.get('cause'))[:200]}", flush=True)
            if harness.aborted and flags.invariants_abort:                 # headline: an auto-fail invariant ends the trial
                name = str(rec["attempts"][-1].get("aborted_by_invariant", "")).split(":")[0]
                report["abort"] = f"move {n_done} ({kind_} {oid}): auto-fail invariant {name}"
                break
            if not rec["ok_physical"]:                                     # record-only invariants never change the plan
                if flags.continue_after_failed_dish:
                    failed_dishes.setdefault(oid, n_done)
                    continue
                if rec.get("refused"):
                    continue                                               # a refusal is non-fatal, as in the benchmark
                report["abort"] = f"move {n_done} ({kind_} {oid}) failed twice"
                break
    except HN.InvariantAbort as exc:                                   # raised outside a pick/place (a ramp, a transit)
        report["abort"] = report.get("abort") or f"auto-fail invariant outside a move: {str(exc).split(':')[0]}"
    except BudgetExpired as exc:                                       # the move loop's share of the wall budget
        report["abort"] = report.get("abort") or f"wall-budget: {exc}"
        report["moves"] = [m for m in report["moves"] if "ok" in m]
        tlog.write("note", what="wall budget expired during the moves", detail=str(exc), grace_s=SCORING_GRACE_S)
    report["seconds_wall"] = round(time.monotonic() - t_run, 1)
    report["sim_s_moves"] = round((backend.tick_index - s_run) * backend.dt, 3)
    report["failure_stills"] = [f for f in fails if f]
    report["counter_full_refusals"] = refusals

    # ---------------------------------------------------------------- end of the trial
    harness.abort_on = False
    backend.deadline = t_start + args.max_wall_seconds                  # the scoring grace period
    tlog.set(move=None, dish=None, attempt=None, phase="end")
    harness.clear_carried()
    rig.gripper(False)
    if flags.end_check == "racked_in":                                   # the old episode end (legacy)
        transit_to(T_home)
        report["stills"]["finished"] = still("finished")
        now = backend.poses()
        report["final_poses"] = {oid: now[oid] for oid in ids}
        report["final_components"] = {r: now[r] for r in ("LowerRack", "UpperRack", "SilverwareBasket")}
        racked = {oid: B.racked_in(points[kinds[oid]], now[oid], {r: now[r] for r in ("LowerRack", "UpperRack")}) for oid in ids}
        report["end_check"] = {"outcome": "all_racked" if all(v is not None for v in racked.values()) else "not_all_racked", "racked": racked}
        final_hold, solved = None, None
        per_goal = {oid: {"racked_in": racked[oid], "intended": goal_rack[oid], "ok": racked[oid] == goal_rack[oid]} for oid in ids}
    else:
        park = transit_to(T_home)
        report["arm_parked"] = park is None
        if park:
            report["park_why"] = park
        js = backend.joints()
        reext = {RACK_JOINT[r]: EXTENSION[r] for r in ("LowerRack", "UpperRack")
                 if abs(js[RACK_JOINT[r]] - EXTENSION[r]) > LIMITS["rack_endpoint_m"]}
        if reext and flags.reextend_before_scoring:                     # D2/R3: both racks at the benchmark's extension
            tlog.set(phase="rack")
            backend.ramp(reext, "robot_reextend_before_scoring")
            tlog.write("rack", state="re-extended before scoring", joints=backend.joints(), commanded=reext,
                       declared="D2: both racks back at the benchmark's extension before scoring")
            report["upper_rack"].append({"at_move": n_done, "state": "out (re-extended before scoring)"})
        tlog.set(phase="final hold")
        now = backend.poses()
        frames = {r: now[r] for r in ("LowerRack", "UpperRack")}
        racked = {oid: B.racked_in(points[kinds[oid]], now[oid], frames) for oid in ids}
        backend.assignments = {oid: r for oid, r in racked.items() if r is not None}
        hold = backend.hold(phase="robot_final_hold", goal=backend.goal(("LowerRack", "UpperRack")), timeout=12.,
                            observation=flags.final_hold_observation_s)
        final_hold = {k: hold.get(k) for k in ("passed", "settled", "reason", "simulated_seconds", "observation_required_s",
                                               "observation_completed_s", "contacts_ok", "dish_supported",
                                               "unsupported_object_ids", "endpoints_ok", "peak_penetration_m")}
        tlog.write("settle", what="final hold (D3: backend.hold, LIMITS, 2 s observation)", **final_hold)
        report["final_hold"] = final_hold
        report["stills"]["finished"] = still("finished")
        now = backend.poses()                                            # the scored state (after the final hold)
        report["final_poses"] = {oid: now[oid] for oid in ids}
        report["final_components"] = {r: now[r] for r in ("LowerRack", "UpperRack", "SilverwareBasket")}
        report["final_diag_settle"] = diag_after_move()                  # diagnostic only, after the snapshot
        per_goal = {}
        for oid in ids:
            T = T_of(now[oid]["position_m"], now[oid]["quaternion_xyzw"])
            dist = B.goal_distance(centroids[kinds[oid]], T, goal_world[oid])
            per_goal[oid] = {"at_goal": bool(B.within_goal(kinds[oid], dist)),
                             "distance": [round(dist[0] * 1e3, 1), round(dist[1] * 1e3, 1), round(dist[2], 1)],
                             "racked_in": B.racked_in(points[kinds[oid]], now[oid], {r: now[r] for r in ("LowerRack", "UpperRack")})}
        solved = all(v["at_goal"] for v in per_goal.values())
        harness.scoring = True                                           # the end check below is SCORING, not execution
        report["end_check"] = end_check(now) if report.get("abort") is None and "stopped" not in report else {"ran": False, "outcome": "aborted"}
        tlog.write("note", what="end check (D2)", **{k: v for k, v in report["end_check"].items() if k != "racked"})
    report["per_goal"] = per_goal
    report["solved"] = solved
    # ---------------------------------------------------------------- verdicts and per-bowl causes
    counts = {}                                                          # distinct (name, move, partner) occurrences
    for v in harness.violations:
        counts[v["name"]] = counts.get(v["name"], 0) + 1
    report["invariant_ticks"] = dict(harness.tick_counts)
    completed = bool(report["moves"]) and "abort" not in report and "stopped" not in report
    if flags.end_check == "racked_in":
        # the old episode's rule (no invariants existed): every move physically ok, no abort, every dish racked
        legacy_ok = completed and all(m.get("ok_physical") for m in report["moves"]) and report["end_check"]["outcome"] == "all_racked"
        verdicts = {"legacy_rule": "PASS" if legacy_ok else "FAIL", "benchmark_success": None, "final_hold": None,
                    "invariants_clean": not harness.violations, "invariant_counts": counts,
                    "harness": "PASS" if (legacy_ok and not harness.violations) else "FAIL"}
        report["result"] = verdicts["legacy_rule"]
    else:
        success = bool(solved) and report["end_check"].get("outcome") == "accepted"
        hold_ok = bool(report.get("final_hold", {}).get("passed"))
        verdicts = {"legacy_rule": None, "benchmark_success": success, "final_hold": hold_ok,
                    "invariants_clean": not harness.violations, "invariant_counts": counts,
                    "headline_flags": F.is_headline(flags),
                    "harness": "PASS" if (success and hold_ok and not harness.violations and F.is_headline(flags)) else "FAIL"}
        report["result"] = verdicts["harness"]
    report["verdicts"] = verdicts
    per_bowl = {}
    for oid in ids:
        mv = [m for m in report["moves"] if m["dish"] == oid]
        bad = next((m for m in mv if not m["ok"]), None)
        pg = per_goal[oid]
        reached = pg.get("at_goal", pg.get("ok"))
        if reached and not bad:
            cause = None
        elif bad:
            cause = bad.get("cause")
        elif not mv:
            cause = f"not attempted: trial ended early ({report.get('abort') or report.get('stopped') or 'no move planned'})"
        else:
            cause = f"not at its goal at the end {pg.get('distance')}"
        per_bowl[oid] = {"start": starts_in[oid], "goal_rack": goal_rack[oid], "moves": [m["move"] for m in mv],
                         "reached": bool(reached), "distance": pg.get("distance"), "racked_in": pg.get("racked_in"),
                         "cause": cause, "failure_class": None if cause is None else failure_class(cause),
                         "invariants": sorted({n for m in mv for a in m.get("attempts", []) for n in a.get("invariants", [])})}
    report["per_bowl"] = per_bowl
    if media is not None:
        media.close()
        report["video"] = str((args.trial_dir / "video.mp4").relative_to(ROOT))
        report["video_frames"] = media.frames
    tlog.write("end", verdicts=verdicts, result=report["result"], per_bowl=per_bowl, per_goal=per_goal,
               end_check={k: v for k, v in report["end_check"].items()}, final_hold=report.get("final_hold"),
               abort=report.get("abort"), stopped=report.get("stopped"), counter_full_refusals=refusals,
               moves=[{k: m.get(k) for k in ("move", "dish", "kind", "ok", "cause", "failure_class", "sim_s", "seconds_wall", "skipped", "refused")}
                      | {"attempts": len(m.get("attempts", []))} for m in report["moves"]],
               invariant_first_lines={f"{n}@move{mv}@{sig}": ln for (n, mv, sig), ln in harness.first.items()},
               invariant_ticks=harness.tick_counts,
               contact_report_counts=harness.contacts.counts, sleep_unsupported=harness.sleep_unsupported,
               trial_sim_s=round(harness.trial_time(), 3), trial_wall_s=round(tlog.wall(), 1),
               video=None if media is None else {"path": "video.mp4", "frames": media.frames, "fps": VIDEO_FPS},
               final_poses=report["final_poses"], final_components=report["final_components"])
    tlog.close()
    backend.trace.close()
    backend.trace = None


def robot_meshes(robot):
    """{body index: [collision-mesh points in that body's frame]} for the Warp replay (authored transforms within one
    rigid body are static; the bodies are placed per frame by their logged poses)."""
    import numpy as np
    import omni.usd
    from pxr import Usd, UsdGeom, UsdPhysics
    stage = omni.usd.get_context().get_stage()
    cache = UsdGeom.XformCache()
    paths = list(robot.root_view.link_paths[0])
    out = {}
    for prim in Usd.PrimRange(stage.GetPrimAtPath("/World/Robot"), Usd.TraverseInstanceProxies()):
        if not (prim.HasAPI(UsdPhysics.CollisionAPI) and prim.IsA(UsdGeom.Mesh)):
            continue
        body = prim
        while body and body.IsValid() and not body.HasAPI(UsdPhysics.RigidBodyAPI):
            body = body.GetParent()
        if not body or not body.IsValid() or str(body.GetPath()) not in paths:
            continue
        rel = np.linalg.inv(np.asarray(cache.GetLocalToWorldTransform(body)).T) @ np.asarray(cache.GetLocalToWorldTransform(prim)).T
        pts = np.asarray(UsdGeom.Mesh(prim).GetPointsAttr().Get(), dtype=float)
        loc = (rel @ np.c_[pts, np.ones(len(pts))].T).T[:, :3]
        out.setdefault(paths.index(str(body.GetPath())), []).append(np.round(loc, 5).tolist())
    return {"body_paths": paths, "hulls": out}


KNUCKLE_GAP_TOUCH_MM = .5             # knuckle-contact check: a visual gap at or below this is a touch of the meshes


def knuckle_dump_setup(robot, oid, np):
    """FCL triangle meshes, each in its rigid body's frame: every Mesh prim under a gripper body whose name contains
    "knuckle" (visual = purpose not guide, without CollisionAPI when such prims exist, else the collision-API prims,
    whose render geometry is the visual mesh), and the dish's visual meshes (no CollisionAPI)."""
    import fcl
    import omni.usd
    from pxr import Usd, UsdGeom, UsdPhysics
    stage = omni.usd.get_context().get_stage()
    cache = UsdGeom.XformCache()
    paths = list(robot.root_view.link_paths[0])

    def tri_mesh(prim, body):
        rel = np.linalg.inv(np.asarray(cache.GetLocalToWorldTransform(body)).T) @ np.asarray(cache.GetLocalToWorldTransform(prim)).T
        m = UsdGeom.Mesh(prim)
        pts = np.asarray(m.GetPointsAttr().Get(), dtype=float)
        loc = (rel @ np.c_[pts, np.ones(len(pts))].T).T[:, :3]
        counts, idx, tris, o = m.GetFaceVertexCountsAttr().Get(), m.GetFaceVertexIndicesAttr().Get(), [], 0
        for c in counts:                               # fan triangulation
            tris += [(idx[o], idx[o + i], idx[o + i + 1]) for i in range(1, c - 1)]
            o += c
        return loc, np.asarray(tris, dtype=np.int32)

    def bvh(parts):
        v = np.concatenate([p[0] for p in parts])
        offs = np.cumsum([0] + [len(p[0]) for p in parts[:-1]])
        t = np.concatenate([p[1] + o for p, o in zip(parts, offs)])
        b = fcl.BVHModel()
        b.beginModel(len(v), len(t))
        b.addSubModel(v.astype(np.float64), t)
        b.endModel()
        return fcl.CollisionObject(b, fcl.Transform()), v

    prims, bodies = [], {}
    for prim in Usd.PrimRange(stage.GetPrimAtPath("/World/Robot"), Usd.TraverseInstanceProxies()):
        if not prim.IsA(UsdGeom.Mesh):
            continue
        body = prim
        while body and body.IsValid() and not body.HasAPI(UsdPhysics.RigidBodyAPI):
            body = body.GetParent()
        if not body or not body.IsValid() or str(body.GetPath()) not in paths or "knuckle" not in body.GetName():
            continue
        coll = prim.HasAPI(UsdPhysics.CollisionAPI)
        purpose = UsdGeom.Imageable(prim).ComputePurpose()
        approx = prim.GetAttribute("physics:approximation").Get() if coll else None
        prims.append({"path": str(prim.GetPath()), "body": body.GetName(), "collision": coll, "purpose": str(purpose),
                      "approximation": None if approx is None else str(approx), "n_points": len(UsdGeom.Mesh(prim).GetPointsAttr().Get() or [])})
        bodies.setdefault(paths.index(str(body.GetPath())), {"name": body.GetName(), "vis": [], "col": []})[
            "col" if coll else "vis"].append((prim, body, str(purpose)))
    knuckles = {}
    for bi, b in bodies.items():
        use = [x for x in b["vis"] if x[2] != "guide"] or b["col"]
        obj, v = bvh([tri_mesh(p, body) for p, body, _ in use])
        knuckles[bi] = {"name": b["name"], "obj": obj, "source": "visual" if b["vis"] else "collision-api mesh",
                        "prims": [str(p.GetPath()) for p, _, _ in use]}
    dish_root = stage.GetPrimAtPath(f"/World/InitialStateDishes/{oid}")
    dbody = dish_root
    for prim in Usd.PrimRange(dish_root):
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            dbody = prim
            break
    dparts = [tri_mesh(p, dbody) for p in Usd.PrimRange(dish_root)
              if p.IsA(UsdGeom.Mesh) and not p.HasAPI(UsdPhysics.CollisionAPI)]
    dobj, dv = bvh(dparts)
    out = {"knuckles": knuckles, "dish_obj": dobj, "rim_z": float(dv[:, 2].max()), "trial": None, "on": False,
           "file": None, "prims": prims, "dish_body": str(dbody.GetPath()), "dish_visual_prims": len(dparts)}
    print(f"[INFO] knuckle dump: bodies {[(k['name'], k['source'], len(k['prims'])) for k in knuckles.values()]}, "
          f"dish visual parts {len(dparts)}, rim z {out['rim_z']:.4f} m", flush=True)
    return out


def knuckle_dump_tick(dump, harness, robot, frames, oid, backend, rig, np, T_of):
    """One tick of the knuckle-contact dump (from the first tick a gripper collider touches the dish)."""
    import fcl
    rows = getattr(harness.contacts, "step_rows", {})
    mine = [(key, r) for key, r in rows.items() if oid in key]
    if not dump["on"]:
        if not any(c in ("pad", "finger", "palm") for _, r in mine for c in r["cats"]):
            return
        dump["on"] = True
    T_d = T_of(frames[oid]["position_m"], frames[oid]["quaternion_xyzw"])
    dump["dish_obj"].setTransform(fcl.Transform(T_d[:3, :3], T_d[:3, 3]))
    bp = robot.data.body_link_pos_w.torch[0].cpu().numpy()
    bq = robot.data.body_link_quat_w.torch[0].cpu().numpy()
    gaps = {}
    for bi, kn in dump["knuckles"].items():
        T_k = T_of(bp[bi], bq[bi])                     # XYZW (Isaac Lab 3.0)
        kn["obj"].setTransform(fcl.Transform(T_k[:3, :3], T_k[:3, 3]))
        hit = fcl.collide(kn["obj"], dump["dish_obj"], fcl.CollisionRequest(), fcl.CollisionResult()) > 0
        req, res = fcl.DistanceRequest(enable_nearest_points=True), fcl.DistanceResult()
        d = fcl.distance(kn["obj"], dump["dish_obj"], req, res)
        p_dish = np.linalg.inv(T_d) @ np.r_[np.asarray(res.nearest_points[1], dtype=float), 1.]
        gaps[kn["name"]] = {"overlap": bool(hit), "gap_mm": None if hit else round(float(max(d, 0.)) * 1e3, 2),
                            "below_rim_mm": round((dump["rim_z"] - float(p_dish[2])) * 1e3, 1)}
    row = {"trial": dump["trial"], "spot": dump["spot"], "tick": backend.tick_index, "jaw": round(float(rig.theta), 4),
           "tilt_deg": round(float(np.degrees(np.arccos(np.clip(T_d[2, 2], -1., 1.)))), 2),
           "contacts": [{"partner": key[1] if key[0] == oid else key[0], "cats": list(r["cats"]), "N": round(r["N"], 3),
                         "F": round(r["F"], 3), "sep_mm": round(r["sep"] * 1e3, 2), "colliders": sorted(r.get("paths", ()))}
                        for key, r in mine],
           "knuckle_visual": gaps}
    with open(dump["file"], "a") as f:
        f.write(json.dumps(row) + "\n")
    dump["rows"] += 1
    dump["last"] = row
    st = dump.setdefault("stats", {})
    for name, g in gaps.items():
        s_ = st.setdefault((dump["trial"], name), {"min_gap_mm": None, "overlap_ticks": 0})
        if g["overlap"]:
            s_["overlap_ticks"] += 1
        elif s_["min_gap_mm"] is None or g["gap_mm"] < s_["min_gap_mm"]:
            s_["min_gap_mm"] = g["gap_mm"]
    for c in row["contacts"]:
        if "knuckle" in c["partner"]:
            k_ = st.setdefault((dump["trial"], "collider:" + c["partner"]), {"ticks": 0, "peak_N": 0.})
            k_["ticks"] += 1
            k_["peak_N"] = max(k_["peak_N"], c["N"])


def knuckle_dump_summary(dump):
    """Per trial: the last dumped tick (the measured pivot at the end of the hold) and the per-knuckle extremes."""
    last = dump.get("last") or {}
    st = {k[1]: v for k, v in dump.get("stats", {}).items() if k[0] == dump["trial"]}
    touch = [c["partner"] for c in last.get("contacts", []) if "knuckle" in c["partner"]]
    vis_touch = [n for n, g in last.get("knuckle_visual", {}).items()
                 if g["overlap"] or (g["gap_mm"] is not None and g["gap_mm"] <= KNUCKLE_GAP_TOUCH_MM)]
    verdict = (None if not touch else "real contact" if any(n in vis_touch for n in touch)
               else "collider artefact")
    return {"ticks": dump["rows"], "last_tick": {k: last.get(k) for k in ("tick", "jaw", "tilt_deg", "contacts", "knuckle_visual")},
            "stats": st, "knuckle_colliders_touching_at_end": touch, "knuckle_visuals_touching_at_end": vis_touch,
            "verdict": verdict}


def limits_trials(args, report, backend, robot, rig, harness, tlog, hook, inst, kinds, ids, goal_world, q_home,
                  pick, place, bench_settle, T_now, np, K, B, EXTENSION, RACK_JOINT, T_of, place_down=None, hover_diag=None,
                  slide_pick=None, narrow=None):
    """plans/2026-10-23-hardware-limits-mouth-up.md: the grasp gate (G3) or the placement trials (G4) on one dish.
    Each trial: the arm teleported home, the dish teleported to the counter spot +-5 mm (seeded), the racks set for the
    cell, 300 ticks, then the guard re-armed (a trial counts only when no harness invariant fired in it). The spot: the run's
    (main()'s counter_spot, LIMITS_COUNTER_XY); ``narrow`` = the episode's plan_to retry log (a row lists its own retries)."""
    from dishsim_frigidaire.random_poses import compose_pose
    oid = ids[0]
    kind = kinds[oid]
    rng = np.random.default_rng(args.seed)
    harness.abort_on = False
    pres = load_presentation(getattr(args, "presentation", None), LIMITS_COUNTER_XY)   # the leaning plate (moved with the spot)
    spot_rec = {"spot_xy": [float(v) for v in LIMITS_COUNTER_XY], "spot_source": getattr(args, "spot_source", None),
                "mount_file": report.get("mount_file"),
                "presentation_file": None if getattr(args, "presentation", None) is None else rel_path(args.presentation),
                "basket": bool(report.get("basket", True)), "basket_source": report.get("basket_source")}   # every trial row
    args.trial_dir.mkdir(parents=True, exist_ok=True)
    (args.trial_dir / "robot_meshes.json").write_text(json.dumps(robot_meshes(robot)) + "\n")
    traj = {"on": False, "rows": []}

    dump = knuckle_dump_setup(robot, oid, np) if args.dump_contacts else None
    if dump is not None:
        dump["file"] = args.out_dir / "dump_contacts.jsonl"
        report["knuckle_dump"] = {"prims": dump["prims"], "dish_body": dump["dish_body"], "rim_z_m": dump["rim_z"],
                                  "dish_visual_prims": dump["dish_visual_prims"],
                                  "bodies": {k["name"]: {"source": k["source"], "prims": k["prims"]} for k in dump["knuckles"].values()},
                                  "touch_rule_mm": KNUCKLE_GAP_TOUCH_MM}

    dish_contacts = {}                                  # per trial: every touching pair with the dish (foot families)

    def traj_tick(frames):
        harness.on_tick(frames)
        if traj["on"] and args.grasp_family.startswith("foot"):
            for k_, row_ in (getattr(harness.contacts, "step_rows", None) or {}).items():
                if oid in k_:
                    c_ = dish_contacts.setdefault("|".join(k_), {"cats": list(row_["cats"]), "peak_N": 0., "ticks": 0,
                                                                 "first_tick": backend.tick_index})
                    c_["peak_N"] = max(c_["peak_N"], float(row_["N"]))
                    c_["ticks"] += 1
                    c_["last_tick"] = backend.tick_index
                    c_.setdefault("colliders", set()).update(row_.get("colliders") or ())   # the bowl's Shell_<band*24+sector>
        if dump is not None and dump["trial"] is not None:
            knuckle_dump_tick(dump, harness, robot, frames, oid, backend, rig, np, T_of)
        if traj["on"] and backend.tick_index % TRAJ_EVERY == 0:
            d, lo, up = frames[oid], frames["LowerRack"], frames["UpperRack"]
            # link quats stay WXYZ in trajectory files (their readers and every 4.5-era file expect it)
            traj["rows"].append((backend.tick_index * backend.dt, robot.data.body_link_pos_w.torch[0].cpu().numpy().copy(),
                                 robot.data.body_link_quat_w.torch[0][:, [3, 0, 1, 2]].cpu().numpy().copy(),
                                 np.r_[d["position_m"], d["quaternion_xyzw"]], np.r_[lo["position_m"], lo["quaternion_xyzw"]],
                                 np.r_[up["position_m"], up["quaternion_xyzw"]], float(rig.theta)))
    hook["tick"] = traj_tick

    def save_traj(tag):
        r = traj["rows"]
        traj["on"] = False
        if not r:
            return None
        path = args.trial_dir / f"traj_{tag}.npz"
        np.savez_compressed(path, t=np.array([x[0] for x in r]), body_pos=np.stack([x[1] for x in r]),
                            body_quat_wxyz=np.stack([x[2] for x in r]), dish=np.stack([x[3] for x in r]),
                            lower=np.stack([x[4] for x in r]), upper=np.stack([x[5] for x in r]), jaw=np.array([x[6] for x in r]))
        traj["rows"] = []
        return str(path.relative_to(ROOT))

    def set_racks(rack):
        """A lower-rack cell has the upper rack pushed in, an upper-rack cell the lower rack (None = both out)."""
        want = {r: EXTENSION[r] for r in ("LowerRack", "UpperRack")}
        if rack is not None:
            want["UpperRack" if rack == "LowerRack" else "LowerRack"] = 0.
        js = backend.joints()
        move = {RACK_JOINT[r]: v for r, v in want.items() if abs(js[RACK_JOINT[r]] - v) > .005}
        if move:
            backend.ramp(move, f"limits_racks_{rack}")

    def spawn_in_hand():
        """--plate-in-hand: the arm teleported to the Kit-free sweep's start (the home TCP on its first IK branch free
        with the plate held), the plate re-posed at TCP @ M every tick while the open jaws close on it, then left to
        physics for RESET_TICKS (the guard is still off: these are reset teleports)."""
        from dishsim_frigidaire import planner as P
        M0 = np.array(orient["measured_M"]["plate"]["M"])
        rig.teleport_arm(np.array(orient["q_start"]))
        rig.step(5)
        obj = backend.objects[oid]

        def pin():
            backend.set_rigid_pose(obj, P.pose_dict(rig.tcp() @ M0))
        pin()
        rig.step(2)
        orig = rig.on_step
        rig.on_step = lambda: (orig(), pin())
        try:
            theta = rig.gripper(True)
        finally:
            rig.on_step = orig
        rig.step(K.RESET_TICKS)
        M1 = np.linalg.inv(rig.tcp()) @ T_now(oid)
        cosang = (np.trace(M0[:3, :3].T @ M1[:3, :3]) - 1) / 2
        harness.pad_max_interval = {"pad_L": 0., "pad_R": 0.}
        harness.arm_guard()
        return {"jaw_rad": round(theta, 4), "drift_mm": round(float(np.linalg.norm(M1[:3, 3] - M0[:3, 3])) * 1e3, 2),
                "rotation_deg": round(float(np.degrees(np.arccos(np.clip(cosang, -1., 1.)))), 2),
                "dist_to_tcp_mm": round(float(np.linalg.norm(M1[:3, 3])) * 1e3, 1)}

    def reset(t, rack=None, xy=LIMITS_COUNTER_XY):
        harness.armed = False
        harness.clear_carried()
        rig.gripper(False)
        rig.teleport_arm(q_home)
        rig.step(5)                                    # the PD targets follow the teleport BEFORE a rack ramp: a ramp
        set_racks(rack)                                # with stale targets swept the arm through the moving upper rack
        if args.plate_in_hand:
            return spawn_in_hand()
        d = rng.uniform(-.005, .005, 2)
        if pres:                                       # the leaning plate: the draw's part along the backstop (it stays on it)
            pose, sh = presentation_pose(pres, d)
        else:
            pose = counter_pose((xy[0] + d[0], xy[1] + d[1]), mouth_down=args.grasp_family.startswith("foot") or args.mouth_down,
                                kind=kind)
        backend.set_rigid_pose(backend.objects[oid], pose)
        rig.step(K.RESET_TICKS)
        harness.pad_max_interval = {"pad_L": 0., "pad_R": 0.}
        harness.arm_guard()
        if pres:                                       # the spawn's rest against the planned leaning pose
            T_s, T_p = T_now(oid), T_of(pres["plate_pose"]["position_m"], pres["plate_pose"]["quaternion_xyzw"])
            return {"draw_mm": [round(float(v) * 1e3, 2) for v in d], "along_backstop_mm": sh,
                    "rest_vs_pose_mm": round(float(np.linalg.norm(T_s[:3, 3] - T_p[:3, 3] - np.r_[np.asarray(sh) / 1e3, 0.])) * 1e3, 2),
                    "rest_vs_pose_deg": round(float(np.degrees(np.arccos(np.clip((np.trace(T_s[:3, :3].T @ T_p[:3, :3]) - 1) / 2,
                                                                                  -1., 1.)))), 2)}
        return [round(float(v) * 1e3, 2) for v in d]

    def outcome(held, why, gate):
        if held is not None:
            return "pivot" if gate and gate.get("settle_rotation_deg", 0.) >= PIVOT_DEG else "hold"
        w = why or ""
        if (w.startswith(("no collision-free grasp", "no approach", "no collision-free slide")) or "no path" in w
                or "safe height" in w or "no collision-free IK" in w):
            return "no-reach"
        if "jaws at" in w:
            return "miss"
        if "not held" in w:
            return "slip"
        if "blocked on the approach" in w:
            return "blocked"
        return "other"

    if args.diag_hover and hover_diag is not None:
        set_racks("LowerRack")
        rig.step(K.RESET_TICKS)
        (args.out_dir / "diag_hover.json").write_text(json.dumps(hover_diag()) + "\n")
        print("[INFO] hover diagnostic written", flush=True)
    rows_path = args.out_dir / "trials.jsonl"
    rows = []
    jobs = [(None, t) for t in range(args.trial_offset, args.trial_offset + args.n)]
    for _ in range(args.trial_offset):                 # the draws of the trials before the offset (a split --n run)
        rng.uniform(-.005, .005, 2)
    if args.mode == "place_slot":
        reach = json.loads(args.reach.read_text())
        want = [x for x in args.sites.split(",") if x]
        named = {s["site"]: s for s in reach["sites"][kind]}
        if set(want) - set(named):
            raise RuntimeError(f"unknown sites {sorted(set(want) - set(named))}")
        sites = [named[x] for x in want]
        site_n = {s["site"]: args.n for s in sites}
        site_n.update({k: int(v) for k, v in (x.rsplit("=", 1) for x in args.site_n.split(",") if x)})
        jobs = ([(s, t) for t in range(max(site_n.values())) for s in sites if t < site_n[s["site"]]] if args.interleave
                else [(s, t) for s in sites for t in range(site_n[s["site"]])])
    if args.counter_grid is not None:                  # the 3x3 subset of the Kit-free 5x5 grid of pick spots
        spots = [s for s in json.loads(args.counter_grid.read_text())["kinds"][kind]["spots"]
                 if s["i"] % 2 == 0 and s["j"] % 2 == 0 and (s["gate"] == "G2" or not args.grid_g2_only)]
        k_, m_ = (int(v) for v in args.grid_part.split("/"))
        jobs = ([(None, (sp, t)) for t in range(args.n) for sp in spots[k_::m_]] if args.interleave   # trial-major
                else [(None, (sp, t)) for sp in spots[k_::m_] for t in range(args.n)])
    t_run = time.monotonic()
    orient = json.loads(args.orient.read_text()) if args.target == "tilt" else None

    def one(k, site, t, tilt=None, cell=None, yaw_k=None):
        t0, s0 = time.monotonic(), backend.tick_index
        n_nr = len(narrow["log"]) if narrow else 0     # the plan_to narrow retries before this trial
        spot = None
        if isinstance(t, tuple):                       # --counter-grid: (spot, trial)
            spot, t = t
        tlog.set(move=k + 1, dish=oid, attempt=0, phase="trial reset")
        rack = site["rack"] if site else ("LowerRack" if args.gate_racks == "upper_in" else None)
        perturb = reset(t, rack, tuple(spot["xy"]) if spot else LIMITS_COUNTER_XY)
        if dump is not None:
            dump.update(trial=t, spot=None if spot is None else [spot["i"], spot["j"]], on=False, rows=0, last=None)
        n_viol = len(harness.violations)
        dish_contacts.clear()
        traj["rows"], traj["on"] = [], True
        att = {"attempt": 0, "dish": oid}
        row = {"mode": args.mode, "kind": kind, "trial": t, "perturb_mm": perturb, "site": site["site"] if site else None,
               "rack": rack, "racks": "upper rack pushed in" if rack == "LowerRack" else "lower rack pushed in"
               if rack == "UpperRack" else "both racks extended", "waive_dish_clearance": bool(args.waive_dish_clearance),
               "tilt_deg": None if args.plate_in_hand else tilt,
               "grasp_family": "in hand" if args.plate_in_hand else "slide" if args.plate_slide else
               "wall pinch" if args.grasp_family == "wall" else
               "side ring hold: the pads on the foot ring's outer face, the approach horizontal (the bowl mouth down)"
               if args.grasp_family == "foot-side" else
               f"foot ring, {args.grasp_family.split('-')[1]}" if args.grasp_family.startswith("foot") else
               f"edge pinch at the top edge, the plate leaning on its backstop ({pres.get('name')})" if args.grasp_family == "edge" else
               f"wall-normal pinch {pres['wall']['depth_mm']} mm down the wall, the plate on its backstop ({pres.get('name')})"
               if args.grasp_family == "plate-wall" else
               f"side pinch, pads {args.side_below_top * 1e3:.0f} mm under the rim"
               + (f", wrist toward the {args.side_wrist_side}" if args.side_wrist_side else "")
               + (", the cup mouth down" if args.mouth_down else "") if args.grasp_family == "side"
               else "rim pinch", "counter_front_y": args.counter_front_y, "counter_xy": list(spot["xy"] if spot else LIMITS_COUNTER_XY),
               "spot": None if spot is None else {"i": spot["i"], "j": spot["j"], "xy": spot["xy"], "kitfree_gate": spot["gate"]},
               **spot_rec}
        if yaw_k is not None:
            row.update(yaw_retry=True, yaw=int(yaw_k))
        if args.plate_in_hand:                         # no grasp: held when the spawned plate hangs in the jaws
            row.update(perturb_mm=None, spawn=perturb)
            T_t, T_o = rig.tcp(), T_now(oid)
            ok = float(np.linalg.norm(T_o[:3, 3] - T_t[:3, 3])) < .2
            held = {"grasp": "in hand", "yaw": None, "tilt_deg": None, "theta": perturb["jaw_rad"],
                    "M": np.linalg.inv(T_t) @ T_o} if ok else None
            why = None if ok else f"plate not held after the in-hand spawn {perturb}"
            if ok:
                harness.set_carried(oid)
        elif args.plate_slide:
            held, why = slide_pick(oid, att)
        else:
            held, why = pick(oid, att, tries=1 if args.mode == "grasp_gate" else 6, waive=args.waive_dish_clearance)
        gate = (att.get("holds") or [None])[-1]
        row.update(outcome=("in hand" if held else "spawn-drop") if args.plate_in_hand else outcome(held, why, gate),
                   g3=held is not None, why=why, hold=gate,
                   grasp=None if held is None else {k_: v for k_, v in held.items() if k_ != "M"},
                   pad_N_end=dict(harness.contacts.pad_N), pad_N_max=dict(harness.pad_max_interval))
        if site is not None and held is not None:
            frame = backend.poses()[site["rack"]]
            if args.target == "tilt":                  # the orientation sweep's pose, turned to its first G2 yaw
                T_goal = T_of(cell["world"]["position_m"], cell["world"]["quaternion_xyzw"])
                phi = cell["phi_rad"]
                Rz = np.array([[np.cos(phi), -np.sin(phi), 0.], [np.sin(phi), np.cos(phi), 0.], [0., 0., 1.]])
                T_goal[:3, :3] = T_goal[:3, :3] @ Rz
                row["phi_rad"] = phi
            elif args.target == "level":               # world pose on the extended baseline frame of the site's rack
                lv = site["world"]["level"]
                T_goal = T_of(lv["position_m"], lv["quaternion_xyzw"])
                row["rack_frame_live"] = frame
                if yaw_k is not None:                  # the TCP at counter grasp yaw_k's yaw about the (symmetric) dish
                    dphi = 2 * np.pi * yaw_k / 12 - float(held["yaw"])
                    T_goal[:3, :3] = T_goal[:3, :3] @ np.array([[np.cos(dphi), -np.sin(dphi), 0.],
                                                                [np.sin(dphi), np.cos(dphi), 0.], [0., 0., 1.]])
                    row["target_dphi_rad"] = round(float(dphi), 4)
            else:
                p, q = compose_pose(frame["position_m"], frame["quaternion_xyzw"], site["local"]["position_m"],
                                    site["local"]["quaternion_xyzw"])
                T_goal = T_of(p.tolist(), q.tolist())
            row["target"] = "family-inhand" if args.plate_in_hand else args.target
            row["release"] = args.release
            before = backend.poses()
            if args.release == "down":                 # lowered until contact, then opened (upright3 rule, R5)
                if args.down_orient == "target":
                    T_t = np.array(T_goal, copy=True)
                else:                                  # round 2: as it hangs
                    T_t = np.array(rig.tcp() @ held["M"], copy=True)
                    T_t[:3, 3] = T_goal[:3, 3]
                goal_world[oid] = T_t
                pw = place_down(oid, T_t, held)
                T_judge = held.get("T_release", T_t)
                row.update(contact=held.get("contact"), release_pose=np.round(T_judge[:3, 3], 4).tolist(),
                           below_target_mm=round(float(T_goal[2, 3] - T_judge[2, 3]) * 1e3, 1),
                           release_tilt_deg=round(float(np.degrees(np.arccos(np.clip(T_judge[2, 2], -1., 1.)))), 1),
                           down_orient=args.down_orient)
            else:
                goal_world[oid] = T_goal
                pw = place(oid, T_goal, held)
                T_judge = T_goal
            goal_world[oid] = T_judge
            judge = bench_settle(oid, "goal", T_judge, before)
            T_fin = T_now(oid)
            row["final_tilt_deg"] = round(float(np.degrees(np.arccos(np.clip(T_fin[2, 2], -1., 1.)))), 1)
            row.update(place_why=pw, judge={k_: judge.get(k_) for k_ in ("settle", "at_target", "distance", "tolerance")},
                       g4=bool(pw is None and judge["settle"]["passed"] and judge["at_target"]))
        elif site is not None:
            row["g4"] = False
        if dump is not None:
            row["dump"] = knuckle_dump_summary(dump)
            dump["trial"] = None
        if args.grasp_family.startswith("foot"):          # knuckle and pad contacts over the trial (the close onward)
            row["dish_contacts"] = {k_: {**v, "peak_N": round(v["peak_N"], 3), "colliders": sorted(v.get("colliders", ()))}
                                    for k_, v in dish_contacts.items()}
            gr = sorted({k_ for k_, v in dish_contacts.items() if any(c in ("finger", "palm") for c in v["cats"])})
            row["knuckle_contact"] = bool(gr)
            row["non_pad_gripper_pairs"] = gr
        viol = harness.violations[n_viol:]
        row["invariants"] = sorted({v["name"] for v in viol})
        row["counted"] = not row["invariants"]
        tag = (f"{site['site']}_inhand" if args.plate_in_hand else
               f"{site['site']}_{args.target}{'' if tilt is None else int(tilt)}{'' if yaw_k is None else f'_y{yaw_k}'}"
               f"{'_down' if args.release == 'down' else ''}"
               if site else ("slide" if args.plate_slide else f"spot{spot['i']}{spot['j']}" if spot else "gate"))
        row["traj"] = save_traj(f"{kind}_{tag}_s{args.seed}_t{t:02d}".replace("@", "_"))
        row["narrow_retries"] = list(narrow["log"][n_nr:]) if narrow else []   # plan_to's narrow-passage retries in this trial
        row["sim_s"] = round((backend.tick_index - s0) * backend.dt, 2)
        row["wall_s"] = round(time.monotonic() - t0, 1)
        rows.append(row)
        with open(rows_path, "a") as f:
            f.write(json.dumps(row, default=lambda o: o.tolist() if hasattr(o, "tolist") else str(o)) + "\n")
        print(f"[INFO] trial {k + 1} {kind} {row['site'] or ('counter spot %s' % row['spot']['xy'] if spot else 'counter')}{'' if tilt is None else f' tilt {tilt}'} t{t}: "
              f"{row['outcome']}{'' if site is None else ' G4 ' + str(row.get('g4'))} jaw {None if not gate else gate.get('jaw_rad')} "
              f"pivot {None if not gate else gate.get('settle_rotation_deg')} drift {None if not gate else gate.get('drift_mm')} "
              f"inv {row['invariants']} {row['wall_s']} s  why {str(why)[:160]}", flush=True)
        return row

    if args.target == "tilt":                          # per site: tilts in order, stop at the first failing tilt
        k = 0
        cells = {(c["site"], c["tilt_deg"]): c for c in orient["cells"][kind]}
        tilts = [float(x) for x in args.tilts.split(",")]
        report["tilt_runs"] = []
        for site in sites:
            for tilt in tilts:
                cell = cells.get((site["site"], tilt))
                if cell is None or cell.get("gate") != "G2":
                    report["tilt_runs"].append({"site": site["site"], "tilt_deg": tilt, "stop": "kit-free",
                                                "gate": None if cell is None else cell.get("gate"),
                                                "why": None if cell is None else cell.get("why")})
                    with open(args.out_dir / "tilt_stops.jsonl", "a") as f:
                        f.write(json.dumps(report["tilt_runs"][-1]) + "\n")
                    print(f"[INFO] {site['site']} tilt {tilt}: Kit-free {None if cell is None else cell.get('gate')} -> skipped", flush=True)
                    continue                           # a Kit-free failure is recorded; only an Isaac 0/n ends the site
                g4 = 0
                for t in range(args.n):
                    g4 += bool(one(k, site, t, tilt, cell).get("g4"))
                    k += 1
                report["tilt_runs"].append({"site": site["site"], "tilt_deg": tilt, "g4": g4, "n": args.n})
                if g4 == 0:                            # recorded; the next tilt may still fit the tines better
                    with open(args.out_dir / "tilt_stops.jsonl", "a") as f:
                        f.write(json.dumps({"site": site["site"], "tilt_deg": tilt, "stop": "isaac", "g4": 0, "n": args.n}) + "\n")
                    if args.tilts_adaptive:            # the site's Isaac limit: its climb ends here
                        report["tilt_runs"][-1]["limit"] = True
                        break
    elif args.yaw_retry:                               # other reach.jsonl yaws at a site the planner refused at yaw 0
        k = 0
        report["yaw_retry"] = []
        for site in sites:
            res = {"site": site["site"], "yaws_tried": [], "released_at_yaw": None, "g4": None}
            for yk in (int(x) for x in args.yaw_retry.split(",")):
                res["yaws_tried"].append(yk)
                rs = []
                for t in range(args.n):
                    rs.append(one(k, site, t, yaw_k=yk))
                    k += 1
                if any(r.get("g3") and "place_why" in r and r["place_why"] is None for r in rs):
                    for t in range(args.n, args.yaw_top):
                        rs.append(one(k, site, t, yaw_k=yk))
                        k += 1
                    res.update(released_at_yaw=yk, g4=f"{sum(bool(r.get('g4')) for r in rs)}/{len(rs)}")
                    break
            report["yaw_retry"].append(res)
            (args.out_dir / "yaw_retry.json").write_text(json.dumps(report["yaw_retry"]) + "\n")
    else:
        for k, (site, t) in enumerate(jobs):
            one(k, site, t)
    report["trials"] = rows
    report["seconds_wall"] = round(time.monotonic() - t_run, 1)
    n_g3 = sum(r["g3"] for r in rows)
    n_g4 = sum(bool(r.get("g4")) for r in rows)
    report["result"] = "PASS"
    report["summary"] = {"trials": len(rows), "g3": n_g3, "g4": n_g4 if args.mode == "place_slot" else None,
                         "outcomes": {o: sum(r["outcome"] == o for r in rows) for o in sorted({r["outcome"] for r in rows})},
                         "counted": sum(r["counted"] for r in rows)}
    print(f"[RESULT] PASS limits {args.mode} {kind}: {report['summary']}", flush=True)
    tlog.close()
    backend.trace.close()
    backend.trace = None


# ---------------------------------------------------------------------------- Part B: placing the goal with the dish in hand
DROP_HEIGHTS_M = tuple(round(.025 * k, 3) for k in range(1, 15))   # D: the dish centroid 25-350 mm above the goal centroid
DROP_TRIES = 6                            # D: drop poses with a free IK tried by transit_to before "no reach" (3 in the
                                          # first drop test; 6 since plans/2026-10-27-cup-side-drop.md: its no-paths were RRT timeouts)
SIDE_BASE_RUN, SIDE_MOUTH_RUN = "partA_cup_side_base45", "partA_cup_side_h45"   # placing_v1/: --hold side-base's gate; its
                                          # fallback when that gate held 0/10: the measured mouth-side hold rolled 180 deg


def flip_down(R0, np):
    """--hold side-base's drop orientation: the hold flipped, TCP x (the cup axis, base to mouth) straight down and the
    approach (TCP z) the hold's, made horizontal, so the wrist (along -TCP x) rides above the mouth-down cup."""
    z = np.array([R0[0][2], R0[1][2], 0.])
    z /= np.linalg.norm(z)
    x = np.array([0., 0., -1.])
    return np.column_stack([x, np.cross(z, x), z])


def flip_up(R0, np):
    """--hold side-mouth's drop orientation (plans/2026-11-04-grasp-place-s0.md, the coordinator's option B): the mouth-side
    hold turned 180 deg about its approach, TCP x (the cup axis, mouth to base) straight UP and the approach (TCP z) the
    hold's, made horizontal, so the cup hangs mouth down with the wrist (along -TCP x) below it."""
    z = np.array([R0[0][2], R0[1][2], 0.])
    z /= np.linalg.norm(z)
    x = np.array([0., 0., 1.])
    return np.column_stack([x, np.cross(z, x), z])


def drop_tcp(R0, m_c, g_c, h, k, np):
    """Variant D's drop pose: the TCP orientation R0 turned 2 pi k / 12 about the vertical, the dish centroid (``m_c`` in
    the TCP frame) ``h`` above the goal centroid ``g_c``."""
    c, s_ = math.cos(2 * math.pi * k / 12), math.sin(2 * math.pi * k / 12)
    Rt = np.array([[c, -s_, 0.], [s_, c, 0.], [0., 0., 1.]]) @ np.asarray(R0)
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = Rt, np.asarray(g_c) + np.array([0., 0., h]) - Rt @ np.asarray(m_c)
    return T


# plans/2026-10-27-bowls-plates-down.md: bowls D-flip, plates I
FLIP_TILTS_DEG = (0., 15., 30., 45., 60., 75.)             # D-flip: the bowl axis this far from straight down, first clear wins
FALL_DROPS_M = tuple(round(.005 * k, 3) for k in range(11))  # D-flip fall check: the released bowl 0-50 mm straight down
LOWER_STEP_M = .005                                          # I: the plate lowered along the vertical in 5 mm steps
PLATE_SIDES = ("mouth", "base")                              # I: the gripper body on the plate's mouth face / base face


def rodrigues_R(u, ang, np):
    u = np.asarray(u, dtype=float) / np.linalg.norm(u)
    K_ = np.array([[0., -u[2], u[1]], [u[2], 0., -u[0]], [-u[1], u[0], 0.]])
    return np.eye(3) + math.sin(ang) * K_ + (1. - math.cos(ang)) * K_ @ K_


def flip_orients(R0, a_tcp, t_deg, np):
    """--hold rim-flip: the hold's TCP orientation ``R0`` turned about its closing axis (R0's y, through the TCP) so the bowl
    axis (``a_tcp``, TCP frame) points down at ``t_deg`` from straight down: [(R, turn_deg, axis_tilt_deg)], one per side. The
    measured hold pivoted about TCP x, so its axis is not perpendicular to the closing axis: a tilt below the smallest
    reachable one gives the full flip (the turn with the axis closest to straight down) alone."""
    u = np.asarray(R0)[:, 1] / np.linalg.norm(np.asarray(R0)[:, 1])
    v = np.asarray(R0) @ np.asarray(a_tcp, dtype=float)
    vp = v - (v @ u) * u
    A, Bc, Cz = vp[2], np.cross(u, vp)[2], (v @ u) * u[2]
    Rr, phi0 = math.hypot(A, Bc), math.atan2(Bc, A)
    kap = (-math.cos(math.radians(t_deg)) - Cz) / Rr      # cos(phi - phi0) for a bowl-axis z of -cos(t)
    phis = [phi0 + math.pi] if kap <= -1. else [phi0 + math.acos(min(1., kap)), phi0 - math.acos(min(1., kap))]
    out = []
    for phi in phis:
        R = rodrigues_R(u, phi, np) @ np.asarray(R0)
        a = R @ np.asarray(a_tcp, dtype=float)
        out.append((R, round(math.degrees((phi + math.pi) % (2 * math.pi) - math.pi), 1),
                    round(math.degrees(math.acos(float(np.clip(-a[2], -1., 1.)))), 1)))
    return out


LIP_SPINS_DEG = (0., 15., -15., 30., -30.)   # D-lip (plans/2026-10-27-bowls-lip-plates-fix.md): the bowl spun about its own
                                             # axis from the exact lip, tried in this order within each drop height
RELEASE_BACK_MAX_M = .10                     # I (plans/2026-10-27-carry-rules.md, rows I3): the open hand backs up to this far
                                             # along -TCP z through the collision-checked line, stopping at its last free
                                             # waypoint (round 8: 0.10 m unchecked, wrist 2 into the basket at 633 N; round 9:
                                             # 25 mm, then the rise lifted plate_07 by its rim)
BACK_STEP_M = .005                           # I3: the back-off line's waypoint spacing
RIM_CLEAR_M = .010                           # I3: the hand rises only when the open gripper hulls are this far (FCL distance)
                                             # from the released plate at the backed pose
CARRY_SPINS_DEG = tuple(float(s) for s in range(-45, 50, 5))   # --carry nearest (plans/2026-10-27-carry-rules.md): the rim spin
                                             # scan, -45..45 deg in 5 deg steps (the pinch on the upper, mouth-facing quarter)


def two_legs(rig, arm, q0, T_drop, q_drop, z_safe, kw, np):
    """transit_to's joint path from ``q0`` (the arm at the safe height) to ``q_drop``: via the drop pose shifted to ``z_safe``
    in the drop orientation (IK nearest ``q_drop``) when that has a collision-free IK, else straight to ``q_drop``.
    (legs [(q_a, q_b)], per-joint travel |dq leg 1| + |dq leg 2| in rad)."""
    above = np.array(T_drop, copy=True)
    above[2, 3] = z_safe
    q_a = rig.ik(above, q_drop)
    if q_a is not None and not arm.free(q_a, **kw):
        q_a = None
    q0, q_drop = np.asarray(q0, dtype=float), np.asarray(q_drop, dtype=float)
    legs = [(q0, q_a), (q_a, q_drop)] if q_a is not None else [(q0, q_drop)]
    return legs, sum(np.abs(b - a) for a, b in legs)


def carry_search(rig, arm, homes, pose_of, kw, z_safe, heights=DROP_HEIGHTS_M, spins=CARRY_SPINS_DEG, keep=DROP_TRIES,
                 step=.02):
    """--carry nearest (plans/2026-10-27-carry-rules.md), Kit-free, before the arm moves. Combos = every free home branch
    (``homes``) x spin x elbow-up IK branch of the drop pose ``pose_of(h, spin)`` (unwound toward that home branch). The lowest
    height with any collision-free drop IK wins (a cheap IK + single-pose filter, heights in order); there each combo's cost =
    the largest per-joint travel over transit_to's two legs (start -> the drop pose at ``z_safe``, IK nearest the drop branch
    -> the drop pose), ties by the sum over joints. Combos whose legs are both straight collision-free joint moves
    (arm.path_free at ``step``) rank first: checked lazily in cost order, the first straight combo fixes the home branch, then
    that branch's combos until ``keep`` straight ones; no straight combo anywhere: the cheapest combo's branch (RRT-Connect).
    {"chosen": combo or None, "ranked": that branch's combos (straight first, then cost), "scan", "n", "checked", "straight",
    "best_per_home", "seconds"}; a combo = {"b", "h", "spin", "T", "q_drop", "legs", "cost", "sum", "kinds", "straight"}."""
    import numpy as np
    t0 = time.monotonic()
    homes = [np.asarray(q, dtype=float) for q in homes]
    memo = {}

    def free(q):
        key = tuple(np.round((np.asarray(q) + np.pi) % (2 * np.pi) - np.pi, 6))
        if key not in memo:
            memo[key] = arm.free(q, **kw)
        return memo[key]

    scan, combos = [], []
    for h in heights:
        n_ik = 0
        for s_ in spins:
            T = pose_of(h, s_)
            for b, qh in enumerate(homes):
                for qd in rig.ik_all(T, qh):
                    n_ik += 1
                    if free(qd):
                        combos.append({"b": b, "h": h, "spin": float(s_), "T": T, "q_drop": qd})
        scan.append({"height_m": h, "ik": n_ik, "free": len(combos)})
        if combos:
            break
    out = {"scan": scan, "n": len(combos), "checked": 0, "straight": 0, "best_per_home": {}}
    if not combos:
        return {**out, "chosen": None, "ranked": [], "seconds": round(time.monotonic() - t0, 1)}
    for c in combos:
        c["legs"], per = two_legs(rig, arm, homes[c["b"]], c["T"], c["q_drop"], z_safe, kw, np)
        c["cost"], c["sum"] = float(per.max()), float(per.sum())
        bp = out["best_per_home"]
        if c["b"] not in bp or c["cost"] < bp[c["b"]]:
            bp[c["b"]] = c["cost"]
    combos.sort(key=lambda c: (c["cost"], c["sum"]))
    chosen_b = None
    for c in combos:
        if chosen_b is not None and c["b"] != chosen_b:
            continue
        c["kinds"] = ["straight" if arm.path_free(a, b_, step=step, **kw) else "rrt" for a, b_ in c["legs"]]
        c["straight"] = all(k == "straight" for k in c["kinds"])
        out["checked"] += 1
        if c["straight"]:
            chosen_b = c["b"] if chosen_b is None else chosen_b
            out["straight"] += 1
            if out["straight"] >= keep:
                break
    if chosen_b is None:
        chosen_b = combos[0]["b"]
    ranked = sorted([c for c in combos if c["b"] == chosen_b], key=lambda c: (not c.get("straight", False), c["cost"], c["sum"]))
    return {**out, "chosen": ranked[0], "ranked": ranked, "seconds": round(time.monotonic() - t0, 1)}


def combo_rec(c, homes, np):
    """A carry_search combo as a JSON record (degrees)."""
    deg = lambda q: None if q is None else [round(float(v), 1) for v in np.degrees(q)]
    kinds = c.get("kinds") or [None] * len(c["legs"])
    return {"home_q_deg": deg(homes[c["b"]]), "height_m": c["h"], "spin_deg": c["spin"], "q_drop_deg": deg(c["q_drop"]),
            "via_q_deg": deg(c["legs"][0][1]) if len(c["legs"]) == 2 else None,
            "cost_deg": round(float(np.degrees(c["cost"])), 1), "sum_deg": round(float(np.degrees(c["sum"])), 1),
            "travel_deg": deg(sum(np.abs(b - a) for a, b in c["legs"])),
            "legs": ({"leg1": kinds[0], "leg2": kinds[1]} if len(c["legs"]) == 2 else
                     {"leg1": kinds[0], "leg2": "none (no free IK above the drop pose: one leg)"}),
            "straight": c.get("straight")}


def lip_tcp(M, T_goal, c, h, spin_deg, np):
    """--hold rim-lip's drop pose: the bowl welded at the measured rim pinch ``M`` (TCP -> dish), at its goal's exact axis
    ``a`` (base to mouth) with the pinched rim point (the radial direction r_p of the TCP origin in the dish frame) at the
    lip, the rim point highest at the goal (n = world +z projected onto the mouth plane), turned ``spin_deg`` about the bowl
    axis: R_dish = [n, a x n, a] @ [r_p, ez x r_p, ez]^T; the dish centroid ``c`` (dish frame) ``h`` above the goal
    centroid. The bowl ends spun about its own axis relative to the goal quaternion (axisymmetric). (T_tcp, T_dish)."""
    ez = np.array([0., 0., 1.])
    Mi = np.linalg.inv(np.asarray(M, dtype=float))
    r_p = np.array([Mi[0, 3], Mi[1, 3], 0.]) / math.hypot(Mi[0, 3], Mi[1, 3])
    F_d = np.column_stack([r_p, np.cross(ez, r_p), ez])
    T_goal = np.asarray(T_goal, dtype=float)
    a = T_goal[:3, 2] / np.linalg.norm(T_goal[:3, 2])
    n = ez - (ez @ a) * a
    n = rodrigues_R(a, math.radians(spin_deg), np) @ (n / np.linalg.norm(n))
    R = np.column_stack([n, np.cross(a, n), a]) @ F_d.T
    T_d = np.eye(4)
    T_d[:3, :3] = R
    T_d[:3, 3] = T_goal[:3, :3] @ np.asarray(c) + T_goal[:3, 3] + np.array([0., 0., h]) - R @ np.asarray(c)
    return T_d @ Mi, T_d


def lean_deg(T, np):
    """The dish axis's angle from straight down (deg): a mouth-down goal leans 31-69 deg in the easy_bowls load."""
    return round(float(np.degrees(np.arccos(np.clip(-np.asarray(T)[2, 2], -1., 1.)))), 1)


def blocking_pairs(rig, arm, poses, np):
    """A no-reach row's why (Kit-free arm model): every elbow-up IK solution inside the posture limits of each (label,
    T_tcp, collide kwargs) in ``poses`` and the collision pairs each one hits, counted over the solutions."""
    from collections import Counter
    cnt, n_q, no_ik = Counter(), 0, []
    for label, T, kw in poses:
        qs = rig.ik_all(T, rig.q())
        if not qs:
            no_ik.append(label)
        for q in qs:
            n_q += 1
            cnt.update({f"{a}|{b}" for a, b in arm.collisions(q, first=False, **kw)})
    return {"poses": len(poses), "ik_solutions": n_q, "no_ik_poses": no_ik, "pairs": cnt.most_common(8),
            "rule": "pairs (link|what; held|link = the welded dish against an arm link) over every elbow-up IK solution of "
                    "every pose tried, counted once per solution; the arm model's 15 mm margin, the held-dish margin"}


def plate_holds(T_goal, G, np):
    """Variant I's assumed hold (no plate pick exists): grasp.py's KIND_RIM plate rim pinch (TCP 10 mm under the rim top,
    closing radially across the rim wall, approach along the plate's -axis, zero pivot) at the rim point that is highest at
    the goal orientation, so the plate hangs below the TCP at its goal lean and slot yaw. {side: (M, T_tcp at the goal)}:
    "mouth" = the gripper body on the plate's mouth face; "base" = wrist 3 turned 180 deg (the TCP frame turned 180 deg
    about its x axis, the joint axis of wrist 3), the gripper body on the base face."""
    rim_r, wall, height, insert = G.KIND_RIM["plate"]
    T_goal = np.asarray(T_goal, dtype=float)
    a = T_goal[:3, 2]
    n = np.array([0., 0., 1.]) - a[2] * a
    n /= np.linalg.norm(n)
    T = np.eye(4)
    T[:3, 1], T[:3, 2] = n, -a
    T[:3, 0] = np.cross(n, -a)
    T[:3, 3] = T_goal[:3, 3] + (height - insert) * a + (rim_r - wall / 2) * n
    Rx = np.diag([1., -1., -1., 1.])
    return {"mouth": (np.linalg.inv(T) @ T_goal, T), "base": (Rx @ np.linalg.inv(T) @ T_goal, T @ Rx)}


# ---------------------------------------------------------------- plans/2026-10-28-perch-release.md: rows D-perch (--carry overlean)
OVERLEAN_STEP_DEG = 5.                       # the carried tilt: the goal's, then 5 deg steps up to 90 (and 90 itself)
PERCH_H_TOP_M, PERCH_H_FLOOR_M = .350, -.300 # the Kit-free straight-down lowering runs from 350 mm above the goal centroid
PERCH_COM_DISH_M = (0., 0., .0338672)        # the bowl's uniform-density centre of mass in its dish frame (trimesh on the
                                             # visual mesh, watertight; data/results/robot/kitfree_v1/bowl_com.json)
DESCENT_ABOVE_M = .050                       # the guarded descent starts at the lowest free multiple of DESCENT_GRID_M at or
DESCENT_GRID_M = .025                        # above the predicted perch + 50 mm (the bowl centroid above the goal centroid)
DESCENT_BELOW_M = .030                       # and is bounded 30 mm below the predicted perch
DESCENT_SPEED_M_S = .020                     # 20 mm/s straight down in world z: 1 mm per 6 physics ticks at 120 Hz
SENSOR_TRIP_N, SENSOR_TRIP_TICKS = 3.5, 2    # UR5e flange F/T: range 50 N, precision 3.5 N, accuracy 4.0 N (the bowl weighs
                                             # 0.66 N); trip at more than 3.5 N on 2 consecutive ticks
SENSOR_TOOL_CATS = ("pad", "finger", "palm") # harness.classify categories on the tool side (+ wrist_3_link, the held bowl)
SENSOR_SCENE_CATS = ("rack", "basket", "appliance", "counter")   # the scene side (+ every other dish)
PERCH_LINKS = ("upper_arm_link", "forearm_link", "wrist_1_link", "wrist_2_link", "wrist_3_link")   # the hand-first check's links
PERCH_REFINE_M = .003                        # a spin's perch is re-lowered from the tilt's spin-0 perch + 3 mm (the bowl is
                                             # axisymmetric: the spin moves only the hand; the 192 pieces differ by < 1 mm)


def overlean_tilts(T_goal, np):
    """The carried tilts (deg from straight down): the goal's, then +5 deg steps up to 90, and 90."""
    t0 = float(np.degrees(np.arccos(np.clip(-np.asarray(T_goal)[2, 2], -1., 1.))))
    out = [t0] + [t0 + OVERLEAN_STEP_DEG * k for k in range(1, 40) if t0 + OVERLEAN_STEP_DEG * k <= 90. + 1e-9]
    if out[-1] < 90. - 1e-6:
        out.append(90.)
    return out


def goal_hor(T_goal, np):
    """The horizontal unit direction of the goal axis (base to mouth): the side the mouth faces."""
    a = np.asarray(T_goal, dtype=float)[:3, 2]
    a = a / np.linalg.norm(a)
    h = a - a[2] * np.array([0., 0., 1.])
    return h / np.linalg.norm(h)


def overlean_tcp(M, T_goal, c, h, spin_deg, theta_deg, np):
    """--carry overlean's bowl pose (scratch overlean/overlean.py): lip_tcp with the goal axis a turned about the horizontal
    rim tangent at the lip (t = a x n) to ``theta_deg`` from straight down, the mouth still facing ``goal_hor``: a' = sin th
    hor - cos th ez, the lip n' = cos th hor + sin th ez (the rim point highest), spun ``spin_deg`` about a'; the dish
    centroid ``c`` at the goal centroid's x, y and ``h`` above its z. At theta = the goal's tilt it is lip_tcp. (T_tcp, T_dish)"""
    ez = np.array([0., 0., 1.])
    Mi = np.linalg.inv(np.asarray(M, dtype=float))
    r_p = np.array([Mi[0, 3], Mi[1, 3], 0.]) / math.hypot(Mi[0, 3], Mi[1, 3])
    F_d = np.column_stack([r_p, np.cross(ez, r_p), ez])
    T_goal = np.asarray(T_goal, dtype=float)
    hd, th = goal_hor(T_goal, np), math.radians(theta_deg)
    a = math.sin(th) * hd - math.cos(th) * ez
    n = rodrigues_R(a, math.radians(spin_deg), np) @ (math.cos(th) * hd + math.sin(th) * ez)
    R = np.column_stack([n, np.cross(a, n), a]) @ F_d.T
    T_d = np.eye(4)
    T_d[:3, :3] = R
    T_d[:3, 3] = T_goal[:3, :3] @ np.asarray(c) + T_goal[:3, 3] + np.array([0., 0., h]) - R @ np.asarray(c)
    return T_d @ Mi, T_d


class HullList(list):
    """[(fcl.Convex, vertices)] like ArmWorld.hulls["gripper"], with the Isaac trial whose last frame posed them (``run`` dir,
    ``traj`` name; closed_gripper_hulls): the partial open's linkage (gripper_linkage) reads the same trial. None: no trial."""
    run = traj = None


def closed_gripper_hulls(C, np, run=None, traj="traj_bowl_gate_t01.npz"):
    """The gripper's colliders with the jaws closed on the bowl rim (the bowl gate's median hold, trial 1, last frame; scratch
    guarded/geom.py closed_grip) as convex hulls in wrist_3's frame, [(fcl.Convex, vertices)] like ArmWorld.hulls["gripper"]
    (a HullList: the trial rides along as .run / .traj for the partial open, plans/2026-11-04-fifteen-of-fifteen.md)."""
    from dishsim_frigidaire.robot.rig import T_of
    run = Path(run) if run else ROOT / "data/results/robot/limits_v1/gate2_bowl"
    rm = json.loads((run / "trials/robot_meshes.json").read_text())
    paths = rm["body_paths"]
    z = np.load(run / "trials" / traj)

    def Tb(i):
        w = z["body_quat_wxyz"][-1, i]
        return T_of(z["body_pos"][-1, i], (w[1], w[2], w[3], w[0]))
    T3i = np.linalg.inv(Tb(next(j for j, p in enumerate(paths) if p.endswith("/wrist_3_link"))))
    out = HullList()
    out.run, out.traj = run, traj
    for j, p in enumerate(paths):
        if "/Gripper/" not in p:
            continue
        for pts in rm["hulls"].get(str(j), []):
            P = np.c_[np.asarray(pts, dtype=float), np.ones(len(pts))]
            out.append(C._hull((T3i @ Tb(j) @ P.T).T[:, :3]))
    return out


def scene_targets(arm, ignore=(), statics=True):
    """[(name, body)]: the appliance components, the counter slab, the static boxes (pedestal, floor) and the dishes."""
    out = list((arm.checker.components or {}).items())
    if arm.slab is not None:
        out.append(("Counter", arm.slab))
    if statics:
        out += list(arm.statics.items())
    return out + [(o, b) for o, b in arm.dishes.items() if o not in ignore]


def _box_gap(lo, hi, body, np):
    blo, bhi = body["bounds"]
    return float(max(0., np.max(np.r_[blo - hi, lo - bhi])))


def _mgr_dist(fcl, mgr, other):
    data = fcl.DistanceData(request=fcl.DistanceRequest())
    mgr.distance(other, data, fcl.defaultDistanceCallback)
    return float(data.result.min_distance)


MIN_CLEAR_CUTOFF_M = .10                     # min_clear's box cutoff: a returned distance over it bounds nothing (a target its box
                                             # pruned may lie closer), so a descent never steps further than this


def clear_step_mm(d, cutoff=MIN_CLEAR_CUTOFF_M):
    """plans/2026-11-04-fifteen-of-fifteen.md: a 1 mm-grid descent's next step (whole mm) from a min_clear distance ``d`` (m) at
    ``cutoff``: the clearance floored to whole mm, at least 1 mm, at most the cutoff (a larger ``d`` came from the bodies the box
    test kept: plate_03's seat search stepped past its seat's wires and seated 8 mm low), the cutoff when nothing is within it."""
    cap = int(round(cutoff * 1e3))
    if not math.isfinite(float(d)):
        return cap
    return max(1, min(cap, int(math.floor(float(d) * 1e3))))


def min_clear(fcl, objs, targets, np, cutoff=MIN_CLEAR_CUTOFF_M):
    """objs [(name, fcl object or manager, lo, hi)] vs targets [(name, body)]: (d, obj, target) of the smallest FCL distance
    (<= 0 touching; inf = nothing within ``cutoff`` by box, so the distance is at least that). A d over ``cutoff`` is the
    smallest over the targets the box test kept only (the true distance is at least min(d, cutoff)): step by clear_step_mm."""
    best = (np.inf, None, None)
    for nm, o, lo, hi in objs:
        for tn, b in targets:
            if _box_gap(lo, hi, b, np) > min(cutoff, best[0]):
                continue
            d = _mgr_dist(fcl, b["manager"], o)
            if d < best[0]:
                best = (d, nm, tn)
    return best


def contact_pos(fcl, mgr_a, mgr_b, np):
    data = fcl.CollisionData(request=fcl.CollisionRequest(num_max_contacts=8, enable_contact=True))
    mgr_a.collide(mgr_b, data, fcl.defaultCollisionCallback)
    cs = data.result.contacts
    return np.asarray(cs[0].pos, dtype=float) if cs else None


def part_name(arm, target, p, np):
    """'LowerRack:tine/post@z+102' / 'floor wire' / 'rim wire' (capsule axis vertical / low / high in the rack frame) for the
    rack and basket part nearest the world point ``p``; the target's own name otherwise."""
    fcl = arm.fcl
    comps = arm.checker.components or {}
    if target not in comps or p is None or "/" in target:
        return target
    body = comps[target]
    s = fcl.CollisionObject(fcl.Sphere(1e-4), fcl.Transform(np.eye(3), np.asarray(p, dtype=float)))
    best = (np.inf, None)
    for i, o in enumerate(body["objects"]):
        if np.linalg.norm(np.asarray(o.getTranslation()) - p) > .3:
            continue
        d = fcl.distance(s, o, fcl.DistanceRequest(), fcl.DistanceResult())
        if d < best[0]:
            best = (d, i)
    if best[1] is None:
        return target
    part = arm.checker.parts[target][best[1]]
    if "/Wire" not in part.name:
        return f"{target}:{part.name.split('/')[-1]}"
    R = np.asarray(body["objects"][best[1]].getRotation())
    F = np.eye(4)                                     # the component frame from part 0: world = F @ local
    o0, p0 = body["objects"][0], arm.checker.parts[target][0]
    F[:3, :3] = np.asarray(o0.getRotation()) @ p0.rotation.T
    F[:3, 3] = np.asarray(o0.getTranslation()) - F[:3, :3] @ p0.translation
    z_loc = (np.linalg.inv(F) @ np.r_[p, 1.])[2]
    cls = "tine/post" if abs(R[2, 2]) > .5 else ("floor wire" if part.translation[2] < .03 else "rim wire")
    return f"{target}:{cls}@z{z_loc * 1e3:+.0f}mm"


def perch_descent(arm, kind, dish_at, hid, targets, np, h_top=PERCH_H_TOP_M, h_floor=PERCH_H_FLOOR_M, top_free=False):
    """The held bowl alone lowered straight down (world -z): ``dish_at(h)`` = its pose with the centroid h above the goal
    centroid. The first height on the 1 mm grid where it touches a target (FCL distance <= 0); steps = the clearance floored to
    whole mm (at least 1 mm, at most 0.10 m: clear_step_mm), so no touch on the grid is skipped. Starts at ``h_top``, or
    the first mm below it where the bowl is free (``top_free``: None instead when it is not free at ``h_top``). (h, target,
    contact point, start h) or None."""
    fcl = arm.fcl
    h, start = round(h_top, 3), None
    while h > h_floor:
        b = arm.checker._body(kind, _pose_of(dish_at(h)), hid)
        d, _, tn = min_clear(fcl, [("bowl", b["manager"], *b["bounds"])], targets, np)
        if start is None:
            if d <= 0:
                if top_free:
                    return None
                h = round(h - .001, 3)
                continue
            start = h
        if d <= 0:
            tb = next(t for n_, t in targets if n_ == tn)
            return h, tn, contact_pos(fcl, b["manager"], tb["manager"], np), start
        h = round(h - clear_step_mm(d) / 1e3, 3)       # at most min_clear's cutoff (plans/2026-11-04-fifteen-of-fifteen.md)
    return None


def _pose_of(T):
    from scipy.spatial.transform import Rotation
    return {"position_m": [float(v) for v in T[:3, 3]], "quaternion_xyzw": Rotation.from_matrix(T[:3, :3]).as_quat().tolist()}


def hand_objs(arm, q, closed_hulls, links=PERCH_LINKS):
    """The hand at joint state q: the open gripper's hulls, the closed gripper's hulls (both rigid with wrist 3) and the links
    upper arm .. wrist 3, as (name, fcl object, lo, hi)."""
    import numpy as np
    fcl = arm.fcl
    lp = arm.link_poses(q)
    out = [(lk, o, lo, hi) for lk, o, lo, hi in arm.arm_objects(q, ("shoulder_link",)) if lk in links or lk == "gripper"]
    out = [("gripper (open)" if lk == "gripper" else lk, o, lo, hi) for lk, o, lo, hi in out]
    W3 = lp["wrist_3_link"]
    for g, v in closed_hulls:
        o = fcl.CollisionObject(g, fcl.Transform(W3[:3, :3], W3[:3, 3]))
        p = np.asarray(v) @ W3[:3, :3].T + W3[:3, 3]
        out.append(("gripper (closed)", o, p.min(0), p.max(0)))
    return out


def hand_first(rig, arm, T_c, q_c, h_b, h_start, closed_hulls, targets, np):
    """The straight descent from ``h_start`` down to the predicted perch ``h_b`` (the TCP pose ``T_c`` at h_b moved up, the IK
    continuous with ``q_c``): None when the hand (hand_objs) stays clear (FCL distance > 0) of every target at every height of
    the 1 mm grid from h_start down to h_b inclusive (steps: the gripper clearance, half the links' clearance, floored to whole
    mm, at least 1 mm, at most 0.10 m: clear_step_mm); else {"h_mm", "body", "target"} of the first touch (or {"ik": h_mm} when
    the IK line breaks)."""
    from dishsim_frigidaire.robot.rig import IK_JUMP_MAX_RAD
    fcl = arm.fcl
    h, q_prev = round(h_start, 3), None
    while True:
        T = np.array(T_c, copy=True)
        T[2, 3] += h - h_b
        q = rig.ik(T, q_c if q_prev is None else q_prev)
        if q is None or (q_prev is not None and float(np.abs(q - q_prev).max()) > IK_JUMP_MAX_RAD):
            return {"ik": round(h * 1e3)}
        q_prev = q
        objs = hand_objs(arm, q, closed_hulls)
        dg = min_clear(fcl, [o for o in objs if o[0].startswith("gripper")], targets, np)
        dl = min_clear(fcl, [o for o in objs if not o[0].startswith("gripper")], targets, np)
        d = min(dg, dl, key=lambda x: x[0])
        if d[0] <= 0:
            return {"h_mm": round(h * 1e3), "body": d[1], "target": d[2]}
        if h <= h_b + 1e-9:
            return None
        step = clear_step_mm(min(dg[0], dl[0] / 2)) / 1e3   # at most min_clear's cutoff (plans/2026-11-04-fifteen-of-fifteen.md)
        h = round(max(h - step, h_b), 3)


JAW_MARGIN_M = .002                # --jaw-check: the open hand at least 2 mm from the rolling bowl at every recorded sample
OPEN_STEPS_TICKS = 60              # rig.gripper(False)'s open ramp (dishsim_frigidaire.robot.rig.OPEN_STEPS), one tick per step
JAW_GRACE_TICKS = 6                # D-roll: a gripper-bowl contact counts as after the open from the end of the open ramp + 50 ms
JAW_LINKS = ("gripper", "wrist_3_link")   # the open gripper's hulls and wrist 3 (rigid with the TCP)


def tilt_table(path):
    """The perch-roll curve's best_tilt.json and its drops.jsonl rows (frigidaire_drop_curve.py --perch-roll)."""
    bt = json.loads(Path(path).read_text())
    rows = [json.loads(x) for x in (Path(path).parent / "drops.jsonl").read_text().splitlines() if x.strip()]
    return bt, rows


def jaw_bodies(arm, rows, oid, tilt, F_shift, np):
    """The bowl's collision bodies along the curve's recorded dish-alone trajectories at ``tilt`` (every dropped repeat, every
    20 Hz sample), each pose moved by ``F_shift`` (the rack's frame now x the curve's inverse): [(repeat, t, body)]."""
    from dishsim_frigidaire.robot.rig import T_of
    out = []
    for r in rows:
        if r.get("dish") != oid or not r.get("dropped") or abs(float(r.get("tilt_deg", -1e9)) - float(tilt)) > 1e-6:
            continue
        for smp in r.get("traj") or []:
            T = np.asarray(F_shift) @ T_of(smp[1:4], smp[4:8])
            out.append((r["repeat"], smp[0], arm.checker._body("bowl", _pose_of(T), oid)))
    return out


def jaw_fn(arm, bodies, np, margin=JAW_MARGIN_M):
    """q -> {"ok", "min_mm", "repeat", "t_s", "part"}: the open gripper and wrist 3 at q vs the recorded bowl bodies; ok when
    the FCL distance is >= ``margin`` at every sample (stops at the first one under it)."""
    memo = {}

    def check(q):
        k = tuple(np.round((np.asarray(q) + np.pi) % (2 * np.pi) - np.pi, 6))
        if k in memo:
            return memo[k]
        objs = [o for o in arm.arm_objects(q, ("shoulder_link",)) if o[0] in JAW_LINKS]
        worst = {"ok": True, "min_mm": None, "repeat": None, "t_s": None, "part": None}
        best_d = np.inf
        for rep, t, b in bodies:
            d, nm, _ = min_clear(arm.fcl, objs, [("bowl", b)], np, cutoff=.05)
            if d < best_d:
                best_d = d
                worst.update(min_mm=round(float(d) * 1e3, 2) if np.isfinite(d) else None, repeat=rep, t_s=t, part=nm)
            if d < margin:
                worst["ok"] = False
                break
        memo[k] = worst
        return worst
    return check


def overlean_search(rig, arm, homes, M, T_goal, c, oid, kind, kw, z_safe, closed_hulls, keep=DROP_TRIES,
                    spins=CARRY_SPINS_DEG, step=.02, tilts_only=None, jaw=None):
    """--carry overlean (plans/2026-10-28-perch-release.md), Kit-free, before the arm moves. Per carried tilt (overlean_tilts)
    the bowl alone lowered straight down from 350 mm (perch_descent at spin 0): the perch h_b (the bowl's height above its goal
    at first contact), the contact point and the static roll (the bowl's uniform-density centre of mass on the mouth side of
    the contact, resolved along goal_hor = mouth down). Combos = tilt x spin (-45..45, 5 deg) x free home branch (``homes``) x
    elbow-up IK branch of the contact pose; a combo passes when the contact pose is free (the arm model's 15 mm arm / 2 mm held
    margins) with the open AND the closed jaws, a descent start exists (the lowest DESCENT_GRID_M multiple >= h_b + 50 mm whose
    IK, continuous with the contact branch along the straight line, is free with both jaws) and the bowl is first (hand_first:
    no gripper or link touch from the descent start down to h_b). Ranked mouth-down first, then the lowest perch (the spin's
    own, re-lowered from the tilt's), then the smallest largest-joint travel over transit_to's two legs (home -> the descent
    start at ``z_safe`` -> the descent start), ties the sum. Tilts are evaluated in (roll, perch) order until the best combo's
    home branch has ``keep`` passing combos and no later tilt can tie it. {"chosen", "ranked" (the chosen branch's passing
    combos), "tilts", "counts", "first_fail", "seconds"}; a combo = {"b", "theta", "spin", "h_b", "target", "part", "cp",
    "com_mouth_mm", "com_lateral_mm", "mouth_down", "T_c", "q_c", "h_start", "T_start", "q_start", "legs", "cost", "sum"}."""
    import numpy as np
    t0 = time.monotonic()
    homes = [np.asarray(q, dtype=float) for q in homes]
    targets = scene_targets(arm, ignore=(oid,))
    bowl_targets = [t for t in targets if t[0] not in arm.statics]
    hd = goal_hor(T_goal, np)
    ez = np.array([0., 0., 1.])
    com = np.asarray(PERCH_COM_DISH_M, dtype=float)
    open_hulls = arm.hulls["gripper"]
    memo, memo_start, memo_first = {}, {}, {}
    counts = {"tilts": 0, "pairs": 0, "pairs_free": 0, "combos": 0, "no_start": 0, "hand_first": 0, "passing": 0}
    first_fail = {}

    def wkey(q):
        return tuple(np.round((np.asarray(q) + np.pi) % (2 * np.pi) - np.pi, 6))

    def free2(q):
        k = wkey(q)
        if k not in memo:
            ok = arm.free(q, **kw)
            if ok:
                arm.hulls["gripper"] = closed_hulls
                try:
                    ok = arm.free(q, **kw)
                finally:
                    arm.hulls["gripper"] = open_hulls
            memo[k] = ok
        return memo[k]

    def roll(T_d, cp):
        off = (T_d[:3, :3] @ com + T_d[:3, 3]) - cp
        return round(float(off @ hd) * 1e3, 1), round(float(off @ np.cross(ez, hd)) * 1e3, 1)

    tilts, jaw_tab = [], {}
    for th in (overlean_tilts(T_goal, np) if tilts_only is None else tilts_only):
        r = perch_descent(arm, kind, lambda h, th=th: overlean_tcp(M, T_goal, c, h, 0., th, np)[1], oid, bowl_targets, np)
        rec = {"theta": round(th, 2), "h_b_mm": None}
        if r is not None:
            h, tn, cp, start = r
            cm, cl = roll(overlean_tcp(M, T_goal, c, h, 0., th, np)[1], cp) if cp is not None else (None, None)
            rec.update(h_b_mm=round(h * 1e3), h_b=h, target=tn, cp=None if cp is None else cp.tolist(), start_mm=round(start * 1e3),
                       com_mouth_mm=cm, com_lateral_mm=cl, mouth_down=bool(cm is not None and cm > 0.),
                       part=part_name(arm, tn, cp, np))
        tilts.append(rec)
    counts["tilts"] = len(tilts)
    order = sorted([t for t in tilts if t["h_b_mm"] is not None], key=lambda t: (not t["mouth_down"], t["h_b_mm"], t["theta"]))

    def descent_start(T_c, q_c, h_b):
        k0 = int(math.ceil((h_b + DESCENT_ABOVE_M) / DESCENT_GRID_M - 1e-9))
        for k in range(k0, k0 + 12):
            H = round(k * DESCENT_GRID_M, 3)
            T_H = np.array(T_c, copy=True)
            T_H[2, 3] += H - h_b
            line = rig.line_q(T_c, T_H, max(3, int(round((H - h_b) / .005)) + 1), q_c)
            if line is None or not free2(line[-1]):
                continue
            return H, T_H, line[-1]
        return None

    K = lambda x: (not x["mouth_down"], x["h_b_mm"], x["cost"], x["sum"])
    passing, b_star = [], None

    def kth_key():                                     # the keep-th passing combo of the chosen home branch (None: < keep)
        ps = sorted([p for p in passing if p["b"] == b_star], key=K)
        return K(ps[keep - 1]) if len(ps) >= keep else None

    for t in order:
        kk = kth_key()
        if kk is not None and (not t["mouth_down"], t["h_b_mm"] - 1) > kk[:2]:
            break                                      # no spin of a later tilt can tie the kept combos (perch +-1 mm)
        th, cand = t["theta"], []
        for s_ in spins:
            counts["pairs"] += 1
            T_tcp, _ = overlean_tcp(M, T_goal, c, t["h_b"], s_, th, np)
            qcs = [(b, q) for b, qh in enumerate(homes) for q in rig.ik_all(T_tcp, qh)]
            if not any(free2(q) for _, q in qcs):
                ff = first_fail.setdefault(f"{th:.1f}", {})
                ff["no free IK at the contact pose"] = ff.get("no free IK at the contact pose", 0) + 1
                continue
            r = perch_descent(arm, kind, lambda h: overlean_tcp(M, T_goal, c, h, s_, th, np)[1], oid, bowl_targets, np,
                              h_top=t["h_b"] + PERCH_REFINE_M, top_free=True) if s_ != 0. else (
                t["h_b"], t["target"], None if t["cp"] is None else np.asarray(t["cp"]), None)
            if r is None:                              # not free 3 mm above the tilt's perch: the full lowering
                r = perch_descent(arm, kind, lambda h: overlean_tcp(M, T_goal, c, h, s_, th, np)[1], oid, bowl_targets, np)
            if r is None:
                continue
            h_b, tn, cp = r[0], r[1], r[2]
            T_c, T_d = overlean_tcp(M, T_goal, c, h_b, s_, th, np)
            cm, cl = roll(T_d, cp) if cp is not None else (t["com_mouth_mm"], t["com_lateral_mm"])
            qcs = [(b, q) for b, qh in enumerate(homes) for q in rig.ik_all(T_c, qh) if free2(q)]
            if not qcs:
                continue
            if jaw is not None:                        # plans/2026-10-29-perch-roll.md: the rolling bowl vs the open hand
                res_ = [(b, q, jaw(q)) for b, q in qcs]
                okj = [r_ for r_ in res_ if r_[2]["ok"]]
                bestj = max((r_[2] for r_ in res_), key=lambda x: -1e9 if x["min_mm"] is None else x["min_mm"])
                jaw_tab[f"{s_:+.0f}"] = {"tilt_deg": round(th, 1), "candidates": len(res_), "passing": len(okj),
                                         "best_min_mm": bestj["min_mm"], "best_at": [bestj["repeat"], bestj["t_s"], bestj["part"]]}
                qcs = [(b, q) for b, q, r_ in okj]
                if not qcs:
                    counts["jaw"] = counts.get("jaw", 0) + 1
                    ff = first_fail.setdefault(f"{th:.1f}", {})
                    ff["jaw: the rolling bowl meets the open hand"] = ff.get("jaw: the rolling bowl meets the open hand", 0) + 1
                    continue
            counts["pairs_free"] += 1
            for b, q_c in qcs:
                kq = (th, s_, wkey(q_c))
                if kq not in memo_start:
                    memo_start[kq] = descent_start(T_c, q_c, h_b)
                st = memo_start[kq]
                counts["combos"] += 1
                if st is None:
                    counts["no_start"] += 1
                    ff = first_fail.setdefault(f"{th:.1f}", {})
                    ff["no free descent start"] = ff.get("no free descent start", 0) + 1
                    continue
                H, T_H, q_H = st
                legs, per = two_legs(rig, arm, homes[b], T_H, q_H, z_safe, kw, np)
                cand.append({"b": b, "theta": th, "spin": float(s_), "h_b": h_b, "h_b_mm": round(h_b * 1e3), "target": tn,
                             "part": t["part"] if tn == t["target"] else part_name(arm, tn, cp, np),
                             "cp": None if cp is None else np.asarray(cp).tolist(), "com_mouth_mm": cm, "com_lateral_mm": cl,
                             "mouth_down": bool(cm is not None and cm > 0.), "T_c": T_c, "q_c": q_c, "h_start": H,
                             "T_start": T_H, "q_start": q_H, "legs": legs, "cost": float(per.max()), "sum": float(per.sum()),
                             "kq": kq})
        cand.sort(key=K)
        for x in cand:
            if b_star is not None and x["b"] != b_star:
                continue
            kk = kth_key()
            if kk is not None and K(x) >= kk:
                break
            if x["kq"] not in memo_first:
                memo_first[x["kq"]] = hand_first(rig, arm, x["T_c"], x["q_c"], x["h_b"], x["h_start"], closed_hulls, targets, np)
            hf = memo_first[x["kq"]]
            x["hand_first"] = hf
            if hf is not None:
                counts["hand_first"] += 1
                ff = first_fail.setdefault(f"{th:.1f}", {})
                key_ = "hand first: " + (f"IK line breaks at {hf['ik']} mm" if "ik" in hf else f"{hf['body']} vs {hf['target']}")
                ff[key_] = ff.get(key_, 0) + 1
                continue
            counts["passing"] += 1
            passing.append(x)
            if b_star is None:
                b_star = x["b"]
    ranked = sorted([p for p in passing if p["b"] == b_star], key=lambda x: (not x["mouth_down"], x["h_b_mm"], x["cost"], x["sum"]))
    return {"chosen": ranked[0] if ranked else None, "ranked": ranked[:keep], "tilts": tilts, "counts": counts,
            "first_fail": first_fail, "seconds": round(time.monotonic() - t0, 1), "jaw": jaw_tab if jaw is not None else None}


def perch_rec(c, homes, np):
    """An overlean_search combo as a JSON record (degrees, mm)."""
    deg = lambda q: None if q is None else [round(float(v), 1) for v in np.degrees(q)]
    return {"home_q_deg": deg(homes[c["b"]]), "tilt_deg": round(c["theta"], 1), "spin_deg": c["spin"], "perch_mm": c["h_b_mm"],
            "contact": c["target"], "part": c["part"], "com_mouth_mm": c["com_mouth_mm"], "com_lateral_mm": c["com_lateral_mm"],
            "roll": "mouth-down" if c["mouth_down"] else "mouth-up", "descent_start_mm": round(c["h_start"] * 1e3),
            "q_contact_deg": deg(c["q_c"]), "q_start_deg": deg(c["q_start"]),
            "via_q_deg": deg(c["legs"][0][1]) if len(c["legs"]) == 2 else None,
            "cost_deg": round(float(np.degrees(c["cost"])), 1), "sum_deg": round(float(np.degrees(c["sum"])), 1),
            "travel_deg": deg(sum(np.abs(b - a) for a, b in c["legs"]))}


# ---------------------------------------------------------------- plans/2026-10-28-plate-seat.md: rows I4 (--insert seat)
RIM_DEPTH_TODAY_M = .019          # the assumed top-edge hold's rim depth into the pads in rounds 8-10 (rows I, I2, I3)
SEAT_DX_M = .001                  # the plate 1 mm toward the basket (+x of the LowerRack frame) from its goal
SEAT_H_TOP_M = .350               # the Kit-free lowering along the tine axis starts this far above the goal (along the axis)
SEAT_PEN_MAX_M = .001             # the 1 mm overlap allowance (FCL piece-by-piece depth: convex piece vs wire capsule)
SEAT_START_ABOVE_M = .050         # the guarded descent starts 50 mm above the Kit-free seat along the tine axis,
SEAT_START_STEP_M, SEAT_START_MAX_M = .005, .150   # raised in 5 mm steps while it has no free IK continuous to the bound
SEATED_TOL_M = .002               # seated: within 2 mm of a goal bottom wire and of one of the goal's two tine columns
SEAT_PLAN_RRT_S = 45.             # window 2: the Kit-free RRT-Connect budget for leg 1 (home -> the top of the tine-axis line)
SEAT_LINE_STEP_M = .005           # window 2: waypoint spacing of the lines along the tine axis
SEAT_LINE_JUMP_RAD = .3           # window 2: the line up the tine axis ends at an IK jump over this per 5 mm step
SEAT_LINE_MIN_M = .100            # window 2: the line up the tine axis must be at least this long (else no carry)
SEAT_TOUCH_M = .001               # the wire lists (settled vs goal): rack parts within 1 mm
RACK_NAMES_JSON = ROOT / "data/results/robot/kitfree_v1/insert_plate/rack_names.json"   # LowerRack part -> wire group
PLATE_GATE = (ROOT / "data/results/robot/limits_v1/gate2_plate", "traj_plate_gate_t00.npz")   # jaws closed on a plate rim


def rack_labels(n_parts=None):
    """Per LowerRack collision part (index): its wire group ('tine row0 col5', 'tine base rail (row 0)', 'floor long wire
    02', 'floor cross wire 05', ...; the USD wire groups listed in kitfree_v1/insert_plate/rack_names.json). ``n_parts`` = the
    LowerRack's collision part count: since the rack-wheels round (2026-11-02) the 8 wheels are visual only, so the list's
    trailing 'Wheel' entries are dropped when exactly they make the difference."""
    names = json.loads(RACK_NAMES_JSON.read_text())
    if n_parts is not None and len(names) > n_parts and all(k == "Wheel" for k, _ in names[n_parts:]):
        names = names[:n_parts]
    out = []
    for key, _ in names:
        if key is None or str(key).startswith("Wheel"):
            out.append("wheel")
        elif key.startswith("TineBank"):
            out.append(f"tine base rail (row {key[8]})" if key.endswith("Base") else
                       f"tine row{key[8]} col{int(key.split('Tooth')[1])}")
        elif key.startswith("FloorCrossU"):
            out.append(f"floor cross wire {key[-2:]}")
        elif key.startswith("FloorLongU"):
            out.append(f"floor long wire {key[-2:]}")
        else:
            out.append(key)
    return out


def closed_hulls_named(C, np, run, traj):
    """closed_gripper_hulls with a name per hull: [(name, fcl.Convex, vertices in wrist_3's frame)] (the inner finger's second
    hull = its finger pad)."""
    from dishsim_frigidaire.robot.rig import T_of
    rm = json.loads((Path(run) / "trials/robot_meshes.json").read_text())
    paths = rm["body_paths"]
    z = np.load(Path(run) / "trials" / traj)

    def Tb(i):
        w = z["body_quat_wxyz"][-1, i]
        return T_of(z["body_pos"][-1, i], (w[1], w[2], w[3], w[0]))
    T3i = np.linalg.inv(Tb(next(j for j, p in enumerate(paths) if p.endswith("/wrist_3_link"))))
    out = []
    for j, p in enumerate(paths):
        if "/Gripper/" not in p:
            continue
        nm = {"base_link": "palm"}.get(p.split("/")[-1], p.split("/")[-1])
        for k, pts in enumerate(rm["hulls"].get(str(j), [])):
            P = np.c_[np.asarray(pts, dtype=float), np.ones(len(pts))]
            g, v = C._hull((T3i @ Tb(j) @ P.T).T[:, :3])
            out.append((nm.replace("inner_finger", "finger_pad") if nm.endswith("inner_finger") and k == 1 else nm, g, v))
    return out


def tine_axis(F_rack, np):
    """(the rack's x in world, the tine axis in world (unit, up along the tines, leaning toward +x of the rack by
    atan(tine tip offset / tine height)), that lean in deg)."""
    from dishsim_frigidaire.geometry import PARAMETERS as GP
    lr = GP["lower_rack"]
    th = math.atan2(lr["tine_tip_offset_x"], lr["tine_height"])
    R = np.asarray(F_rack, dtype=float)[:3, :3]
    return R[:, 0].copy(), R @ np.array([math.sin(th), 0., math.cos(th)]), math.degrees(th)


def depth_of(fcl, a, b):
    """FCL signed distance of a manager (convex pieces) to an object or a manager: > 0 the gap; <= 0 minus the deepest
    piece-pair penetration (convex piece vs wire capsule; the manager pair's own depth is not used)."""
    data = fcl.DistanceData(request=fcl.DistanceRequest())
    a.distance(b, data, fcl.defaultDistanceCallback)
    d = float(data.result.min_distance)
    if d > 0:
        return d
    cd = fcl.CollisionData(request=fcl.CollisionRequest(num_max_contacts=64, enable_contact=True))
    a.collide(b, cd, fcl.defaultCollisionCallback)
    deps = [float(c.penetration_depth) for c in cd.result.contacts]
    return -max(deps) if deps else 0.


def rack_aabbs(arm, np):
    """World AABBs of the LowerRack's collision parts (capsules: the segment +- the radius; other parts: +- 3 cm)."""
    lo, hi = [], []
    for o, p in zip(arm.checker.components["LowerRack"]["objects"], arm.checker.parts["LowerRack"]):
        c, g = np.asarray(o.getTranslation(), dtype=float), p.geometry
        if type(g).__name__ == "Capsule":
            ax = np.asarray(o.getRotation(), dtype=float)[:, 2] * g.lz / 2
            lo.append(np.minimum(c - ax, c + ax) - g.radius)
            hi.append(np.maximum(c - ax, c + ax) + g.radius)
        else:
            lo.append(c - .03)
            hi.append(c + .03)
    return np.asarray(lo), np.asarray(hi)


def dish_contacts(arm, kind, T, hid, scene, labels, aabbs, np, cutoff=0.):
    """[(signed d, scene body, label)] of the dish at T within ``cutoff``: the LowerRack's wires one by one (label = the wire
    group), every other scene body as one (its name); d = depth_of."""
    fcl = arm.fcl
    b = arm.checker._body(kind, _pose_of(T), hid)
    lo, hi = b["bounds"]
    out = []
    for nm, sb in scene:
        if _box_gap(lo, hi, sb, np) > cutoff + .002 or _mgr_dist(fcl, b["manager"], sb["manager"]) > cutoff:
            continue
        if nm == "LowerRack":
            rlo, rhi = aabbs
            for j in np.nonzero(np.all(rlo <= hi + cutoff + .002, axis=1) & np.all(rhi >= lo - cutoff - .002, axis=1))[0]:
                d = depth_of(fcl, b["manager"], sb["objects"][j])
                if d <= cutoff:
                    out.append((d, nm, labels[j]))
        else:
            out.append((depth_of(fcl, b["manager"], sb["manager"]), nm, nm))
    return sorted(out, key=lambda x: x[0])


def seat_search(arm, kind, hid, T_c, v, scene, labels, np, h_top=SEAT_H_TOP_M, h_end=-.060):
    """The dish alone lowered along ``v`` (unit, the tine axis pointing up) from T_c moved ``h_top`` up it, on a 1 mm grid: the
    first touch (FCL distance <= 0; steps = the clearance floored to whole mm, at most 100 mm: clear_step_mm, so a target the
    box test pruned is never stepped past), then 1 mm steps while the overlap stays within
    SEAT_PEN_MAX_M; the seat = the last height within it (the scratch's lower_fast rule), its supports = the distinct rack wires
    and bodies touching there. Heights in mm along v above T_c. Starting inside a body: the highest free height below it."""
    fcl, v = arm.fcl, np.asarray(v, dtype=float)
    aabbs = rack_aabbs(arm, np)

    def at(h):
        T = np.array(T_c, dtype=float, copy=True)
        T[:3, 3] += h / 1e3 * v
        return T

    def dmin(h):
        b = arm.checker._body(kind, _pose_of(at(h)), hid)
        return min_clear(fcl, [("dish", b["manager"], *b["bounds"])], scene, np)

    h, above = int(round(h_top * 1e3)), None
    d = dmin(h)
    if d[0] <= 0:
        above = d[2]
        while h > h_end * 1e3 and d[0] <= 0:
            h -= 5
            d = dmin(h)
        if d[0] <= 0:
            return {"first_mm": None, "seat_mm": None, "blocked_above_by": above}
        for hh in range(h + 4, h, -1):
            if dmin(hh)[0] > 0:
                h = hh
                break
    entry, first = h, None
    while h > h_end * 1e3:
        d = dmin(h)
        if d[0] <= 0:
            first = h
            break
        h -= clear_step_mm(d[0])
    if first is None:
        return {"first_mm": None, "seat_mm": None, "entry_mm": entry, "blocked_above_by": above}
    cs_first = dish_contacts(arm, kind, at(first), hid, scene, labels, aabbs, np)
    seat, cs_seat, pens, h = first, cs_first, [], first
    while h > h_end * 1e3:
        h -= 1
        cs = dish_contacts(arm, kind, at(h), hid, scene, labels, aabbs, np)
        pen = -min([c[0] for c in cs] + [0.])
        pens.append([h, round(pen * 1e3, 2)])
        if pen > SEAT_PEN_MAX_M:
            break
        seat, cs_seat = h, cs
    fmt = lambda cs: [[lab, round(dd * 1e3, 2)] for dd, _, lab in cs]
    sup = sorted({lab for _, _, lab in cs_seat})
    return {"entry_mm": entry, "blocked_above_by": above, "first_mm": first, "first_contacts": fmt(cs_first),
            "seat_mm": seat, "contacts": fmt(cs_seat), "supports": sup, "n_supports": len(sup),
            "on_dish": sorted({lab for _, nm, lab in cs_seat if nm.startswith(("plate_", "bowl_", "cup_"))}),
            "next_mm_overlap_mm": pens[-1][1] if pens else None, "T_seat": at(seat)}


def hull_objs(fcl, hulls, T_tcp, T_w3_tcp, np):
    """[(name, fcl object, lo, hi)] of named gripper hulls [(name, geom, verts in wrist_3's frame)] at the TCP pose."""
    W3 = np.asarray(T_tcp) @ np.linalg.inv(T_w3_tcp)
    out = []
    for nm, g, v in hulls:
        p = np.asarray(v) @ W3[:3, :3].T + W3[:3, 3]
        out.append((nm, fcl.CollisionObject(g, fcl.Transform(W3[:3, :3], W3[:3, 3])), p.min(0), p.max(0)))
    return out


def hand_touch(arm, T_tcp_seat, v, h_from, h_to, hulls, scene, np):
    """The closed gripper (the jaws on the rim, rigid with the TCP) moved along ``v`` from h_from down to h_to (m above the
    seat): the first height on the 1 mm grid where it touches a scene body (FCL distance <= 0; steps = the clearance floored to
    whole mm, at most 100 mm: clear_step_mm). {"h_mm", "part", "body"} or None."""
    v, h = np.asarray(v, dtype=float), int(round(h_from * 1e3))
    while h >= int(round(h_to * 1e3)):
        T = np.array(T_tcp_seat, dtype=float, copy=True)
        T[:3, 3] += h / 1e3 * v
        d = min_clear(arm.fcl, hull_objs(arm.fcl, hulls, T, arm.T_w3_tcp, np), scene, np)
        if d[0] <= 0:
            return {"h_mm": h, "part": d[1], "body": d[2]}
        h -= clear_step_mm(d[0])                       # at most min_clear's cutoff (plans/2026-11-04-fifteen-of-fifteen.md)
    return None


def seated_test(arm, kind, hid, T_final, T_goal, slot, labels, np, centroid=None):
    """The settled dish vs its goal's own supports (Kit-free; T_goal in the rack's current frame): bottom = the goal's tine base
    rails and floor wires touching at the goal (<= 0.1 mm); columns = the slot's two tine columns g (near, base side) and g+1
    (far, mouth side), both rows of the bank. seated = within SEATED_TOL_M of a bottom wire AND of one column's tine. Wire lists:
    the rack parts within SEAT_TOUCH_M at the settled pose and at the goal."""
    aabbs, scene = rack_aabbs(arm, np), [("LowerRack", arm.checker.components["LowerRack"])]

    def per_label(T, cutoff):
        best = {}
        for d, _, lab in dish_contacts(arm, kind, T, hid, scene, labels, aabbs, np, cutoff=cutoff):
            best[lab] = min(best.get(lab, np.inf), d)
        return best
    g_ = int(slot.split("_")[-1])
    rows_ = (0, 1) if "_front_" in slot else (4, 5)
    goal_d, fin_d = per_label(T_goal, .002), per_label(T_final, .010)
    bottom = sorted(l_ for l_, d in goal_d.items() if d <= 1e-4 and l_.startswith(("tine base rail", "floor")))
    cols = {"near": [f"tine row{r} col{g_}" for r in rows_], "far": [f"tine row{r} col{g_ + 1}" for r in rows_]}

    def nearest(labs):
        ds = [(fin_d[l_], l_) for l_ in labs if l_ in fin_d]
        return None if not ds else [min(ds)[1], round(float(min(ds)[0]) * 1e3, 2)]
    nb, nn, nf = nearest(bottom), nearest(cols["near"]), nearest(cols["far"])
    b_ok = nb is not None and nb[1] <= SEATED_TOL_M * 1e3
    c_ok = any(x is not None and x[1] <= SEATED_TOL_M * 1e3 for x in (nn, nf))
    pose_ok = None
    if not bottom:                                     # round 14: a goal with no bottom wire within 0.1 mm (the rear bank,
        pose_ok = pose_seated(T_final, T_goal, centroid, np)   # wedged between its tines): the goal-pose test (2 mm / 5 deg)
    w_fin = sorted(l_ for l_, d in fin_d.items() if d <= SEAT_TOUCH_M)
    w_goal = sorted(l_ for l_, d in goal_d.items() if d <= SEAT_TOUCH_M)
    return {"seated": bool(b_ok and c_ok) if bottom else pose_ok[0], "bottom_ok": b_ok if bottom else None,
            "column_ok": bool(c_ok), "goal_pose_test": None if pose_ok is None else
            {"ok": pose_ok[0], "centroid_mm": pose_ok[1], "axis_deg": pose_ok[2]},
            "goal_bottom": bottom, "bottom_nearest": nb, "columns": {"near": f"col{g_}", "far": f"col{g_ + 1}"},
            "near_column_nearest": nn, "far_column_nearest": nf, "wires_final": w_fin, "wires_goal": w_goal,
            "wires_matched": sorted(set(w_fin) & set(w_goal)),
            "rule": "seated = the settled plate within 2 mm (FCL distance) of one of the goal's bottom wires (tine base rails / "
                    "floor wires touching at the goal) AND within 2 mm of a tine of the slot's near column g or far column g+1 "
                    "(both rows of the bank); a goal with no bottom wire touching (the rear bank, wedged between its tines): "
                    "the goal-pose test instead (centroid within 2 mm, axis within 5 deg); wire lists = rack parts within "
                    "1 mm; the hand away or not, as the retreat left it"}


# ---------------------------------------------------------------- plans/2026-10-29-bowl-last.md: rows D-seat (--carry seat)
SEAT_ARM_MARGIN_M = .002          # D-seat: the arm links' margin over the final descent (contact-tolerant; the carry keeps 15 mm)
SEAT_LINKS = ("upper_arm_link", "forearm_link", "wrist_1_link", "wrist_2_link")   # the links held to that margin
SEAT_START_GRID_N = 6             # D-seat: descent starts tried per pair, the lowest DESCENT_GRID_M multiple >= stop + 50 mm and 5 above
REST_TOL, REST_EVERY, REST_CAP = (.0005, .5), 30, 600   # at rest: < 0.5 mm / 0.5 deg over 30 ticks, at most 5 s
PRE_DESCENT_LAG_RAD, PRE_DESCENT_CAP = .01, 60          # hold the targets at the descent start until the lag is under this
AWAY_STEP_M, AWAY_WP_M, AWAY_BOWL_M, AWAY_RACK_M = .060, .005, .010, .015   # the hand-away retreat
AWAY_DIP_M, AWAY_FLOOR_MIN_M = .002, .0005   # a hand released inside the 15 mm margin may come 2 mm closer, never under 0.5 mm
BOWL_SEATED_TOL_M = .002          # D-seat seated: a goal bottom support and one goal tine within 2 mm (and mouth down)
RING_SPOT_XY = (.20, -.12)        # plans/2026-10-29-ring-place.md: the bowl starts mouth down at the F-out gate's counter spot
                                  # (= LIMITS_COUNTER_XY; since plans/2026-11-04-fifteen-of-fifteen.md the run's counter_spot)
RING_ABOVE_M = .050               # the carry ends 50 mm above the (refreshed) goal along the rack's up axis (the descent start)
RING_SPINS = 24                   # the goal turned about the bowl axis in 15 deg steps (the bowl and the ring hold are symmetric)
RING_LINE_N = 11                  # the Kit-free descent line: 5 mm waypoints over the 50 mm
RING_WAIT_TICKS = 60              # after the open: 0.5 s
RING_RISE_M = .100                # then the open hand straight up 100 mm
RING_LIFT_M = .150                # the carry's lift from the counter spot = the F-out gate's G3 lift (z_safe 1.25 m is out of
                                  # reach there: the elbow straightens at +0.16 m, Kit-free 2026-10-29)
BOWL_SUPPORTS_JSON = ROOT / "data/results/robot/kitfree_v1/insert_bowl/goal_supports.json"
GOALS_WAIT_S = 900.               # --goals: the longest wait for the refreshed goals (and --bowl-supports) to appear
RELEASE_SWEEP_JSON = ROOT / "data/results/robot/kitfree_v1/release_height/table.json"   # D-ring-h100: the no-reach rows' why


def tool_hulls(arm, closed_hulls):
    """The tool rigid with the TCP, in wrist 3's frame: the closed gripper's hulls + wrist 3's own hulls [(geom, verts)]."""
    return list(closed_hulls) + list(arm.hulls.get("wrist_3_link", ()))


def tool_descent(arm, tcp_at, hulls, targets, np, h_top=PERCH_H_TOP_M, h_floor=PERCH_H_FLOOR_M):
    """The tool (``hulls``, rigid with the TCP) lowered straight down, ``tcp_at(h)`` = the TCP pose with the bowl h above its
    goal: the first height on the 1 mm grid from h_top where it touches a target (FCL distance <= 0; steps = the clearance
    floored to whole mm, at most 0.10 m: clear_step_mm, so a target the box test pruned is never stepped past). (h, target) or
    None."""
    fcl = arm.fcl
    W3i = np.linalg.inv(arm.T_w3_tcp)
    h = round(h_top, 3)
    while h > h_floor:
        W3 = np.asarray(tcp_at(h)) @ W3i
        objs = []
        for g, v in hulls:
            p = np.asarray(v) @ W3[:3, :3].T + W3[:3, 3]
            objs.append(("tool", fcl.CollisionObject(g, fcl.Transform(W3[:3, :3], W3[:3, 3])), p.min(0), p.max(0)))
        d, _, tn = min_clear(fcl, objs, targets, np)
        if d <= 0:
            return h, tn
        h = round(h - clear_step_mm(d) / 1e3, 3)
    return None


def links_clear(arm, q, targets, np, margin=SEAT_ARM_MARGIN_M):
    """The arm links SEAT_LINKS at joint state q at least ``margin`` from every target (FCL distance). (ok, (d, link, target))"""
    objs = [o for o in arm.arm_objects(q, ("shoulder_link",)) if o[0] in SEAT_LINKS]
    d = min_clear(arm.fcl, objs, targets, np, cutoff=max(.05, 2 * margin))
    return bool(d[0] >= margin), d


def seat_line_hits(arm, q, tool, targets, np, margin=SEAT_ARM_MARGIN_M):
    """D-side-down's final-descent rule (D-seat's contact-tolerant one; the user, 2026-11-04): at joint state q the links
    SEAT_LINKS (links_clear) and the hand (``tool``: the closed jaws and wrist 3, rigid with wrist 3) at least ``margin`` from
    every target (FCL distance). [] when clear, else ["<part>|<target> <d> mm", ...]."""
    ok, (d, nm, tn) = links_clear(arm, q, targets, np, margin)
    out = [] if ok else [f"{nm}|{tn} {d * 1e3:.1f} mm"]
    W3 = arm.link_poses(q)["wrist_3_link"]
    objs = []
    for g, v in tool:
        p = np.asarray(v) @ W3[:3, :3].T + W3[:3, 3]
        objs.append(("hand", arm.fcl.CollisionObject(g, arm.fcl.Transform(W3[:3, :3], W3[:3, 3])), p.min(0), p.max(0)))
    d2, _, tn2 = min_clear(arm.fcl, objs, targets, np, cutoff=max(.05, 2 * margin))
    if d2 < margin:
        out.append(f"hand|{tn2} {d2 * 1e3:.1f} mm")
    return out


# plans/2026-11-04-fifteen-of-fifteen.md, the plate pair plate_02 / plate_05 (the user, 2026-11-04 20:05 EST): the tine-guided plate
# release (D2), the seat at goal (D3) and the guarded insertion's retreat (D4, the coordinator)
DROP_GUIDED_MIN_M = .020          # D2: a plate is let go above its seat only with its lowest rim point this far below its slot's tine
                                  # tips (the tines guide the fall); else it is not released (no free fall)
SEAT_AT_GOAL_M = .020             # D3: a predicted seat counts only within this of the goal along the tine axis, on rack wires alone
DROP_RULE = ("D2, the tine-guided plate release: the plate lowered along the tine axis to the lowest descent checkpoint (every 5 mm "
             "and the seat) down to which the arm (the closed hand: the jaws and wrist 3; the open gripper hull; the links upper_arm, "
             "forearm, wrist_1, wrist_2) keeps SEAT_ARM_MARGIN_M (2 mm) from every scene body, provided the plate's lowest rim point "
             "is at least 20 mm below its slot's tine tips there (the rack's FCL tines); in Isaac lowered until the flange sensor "
             "trips or that height is reached, the jaws opened, the open hand up the tine axis (no back-off), the settle; judged by "
             "AT_GOAL's plate rule and the seated test; the plain I4-wall20 (release at the seat) when a start's full descent is clear")


def open_hull_hits(arm, q, targets, np, margin=SEAT_ARM_MARGIN_M):
    """The open gripper's hulls (the arm model's "gripper", rigid with wrist 3) at joint state q at least ``margin`` from every
    target (FCL distance). [] when clear, else ["gripper|<target> <d> mm"]."""
    objs = [o for o in arm.arm_objects(q, ("shoulder_link",)) if o[0] == "gripper"]
    d, _, tn = min_clear(arm.fcl, objs, targets, np, cutoff=max(.05, 2 * margin))
    return [] if d >= margin else [f"gripper|{tn} {d * 1e3:.1f} mm"]


def drop_line_hits(arm, q, tool, targets, np, margin=SEAT_ARM_MARGIN_M):
    """D2's rule at joint state q: D-seat's final-descent rule (seat_line_hits: the links SEAT_LINKS and the hand, the closed jaws
    and wrist 3) and the open gripper hull (the jaws about to open), each at least ``margin`` from every target. [] when clear."""
    return seat_line_hits(arm, q, tool, targets, np, margin) or open_hull_hits(arm, q, targets, np, margin)


def slot_tine_tips(arm, slot, labels, np):
    """{label: tip z (world, m)} of the slot's tines in the arm model's LowerRack (its FCL capsules, at the rack's pose now): the
    columns g and g+1 of the slot lower_<bank>_<g> in both rows of its bank (seated_test's columns); a tip = the capsule's highest
    end plus its radius."""
    g_ = int(str(slot).split("_")[-1])
    rows_ = (0, 1) if "_front_" in str(slot) else (4, 5)
    want = {f"tine row{r} col{c}" for r in rows_ for c in (g_, g_ + 1)}
    comp = arm.checker.components["LowerRack"]
    tips = {}
    for i, (o, p) in enumerate(zip(comp["objects"], arm.checker.parts["LowerRack"])):
        if i >= len(labels) or labels[i] not in want or type(p.geometry).__name__ != "Capsule":
            continue
        c = np.asarray(o.getTranslation(), dtype=float)
        ax = np.asarray(o.getRotation(), dtype=float)[:, 2] * p.geometry.lz / 2
        tips[labels[i]] = max(tips.get(labels[i], -np.inf), float(max((c + ax)[2], (c - ax)[2]) + p.geometry.radius))
    return tips


def plate_rim_low(T, points, np, n=360):
    """The plate's lowest rim point at pose T: its rim circle (the visual points' largest radius at their top height: frigidaire_
    robot_reach.plate_world's rim_r and height) sampled every 360 / n deg. (z, point)"""
    P = np.asarray(points, dtype=float)
    r, h = float(np.linalg.norm(P[:, :2], axis=1).max()), float(P[:, 2].max())
    th = np.radians(np.arange(0., 360., 360. / n))
    T = np.asarray(T, dtype=float)
    rim = np.c_[r * np.cos(th), r * np.sin(th), np.full(len(th), h)] @ T[:3, :3].T + T[:3, 3]
    k = int(np.argmin(rim[:, 2]))
    return float(rim[k, 2]), rim[k]


def tine_guided(arm, slot, labels, T, points, np):
    """D2's guidance of the plate at pose T: its lowest rim point's depth below its slot's tine tips (the lowest of slot_tine_tips),
    mm (None: no tine). {"guided_mm", "tip_z_m", "rim_low_z_m", "tips_z_m"}"""
    tips = slot_tine_tips(arm, slot, labels, np)
    z_rim, _ = plate_rim_low(T, points, np)
    tip = min(tips.values()) if tips else None
    return {"guided_mm": None if tip is None else round((tip - z_rim) * 1e3, 1), "tip_z_m": None if tip is None else round(tip, 5),
            "rim_low_z_m": round(z_rim, 5), "tips_z_m": {k: round(v, 5) for k, v in sorted(tips.items())}}


RETREATS = ("a", "b", "c")        # D4 (the coordinator, plans/2026-11-04-fifteen-of-fifteen.md): the guarded insertion's retreat after the
RETREAT_RULES = {                 # open, the first clear one in this order (= frigidaire_robot_reach.RETREATS / RETREAT_RULES)
    "a": "back-off then rise: CUP_BACK_M (20 mm) along -TCP z, then RING_RISE_M (100 mm) straight up",
    "b": "rise along the insertion axis then back-off: RING_RISE_M (100 mm) along the rack's up axis, then CUP_BACK_M (20 mm) along "
         "-TCP z",
    "c": "rise only: RING_RISE_M (100 mm) straight up"}


def retreat_poses(T0, up, variant, np):
    """[(stage, s m, TCP pose)] of retreat ``variant`` (RETREAT_RULES) from the release TCP pose T0: back-off poses every 5 mm along
    -TCP z, rise poses every 10 mm (straight up; variant b along ``up``, the insertion axis), as frigidaire_robot_reach.
    open_release_check spaces them."""
    T0 = np.asarray(T0, dtype=float)
    out = []

    def back_off(T_from):
        for j in range(1, int(round(CUP_BACK_M / .005)) + 1):
            T = T_from.copy()
            T[:3, 3] -= .005 * j * T0[:3, 2]
            out.append(("back-off", .005 * j, T))

    def rise(T_from, ax):
        for j in range(1, int(round(RING_RISE_M / .010)) + 1):
            T = T_from.copy()
            T[:3, 3] += .010 * j * ax
            out.append(("rise", .010 * j, T))
    if variant == "b":
        rise(T0, np.asarray(up, dtype=float) / np.linalg.norm(up))
        back_off(out[-1][2])
    else:
        if variant == "a":
            back_off(T0)
        rise(out[-1][2] if out else T0, np.array([0., 0., 1.]))
    return out


def retreat_check(arm, ik, q0, T0, up, targets, np, dish=None, judge_release=False):
    """D4: the guarded insertion's retreats (RETREATS, in order) from the open hand at the TCP pose T0 (joints q0): every pose of
    retreat_poses IK-continuous from the last (``ik(T, q_seed)``, at most 0.1 rad per joint and pose), the open gripper and wrist 3
    (JAW_LINKS) at least JAW_MARGIN_M and the links SEAT_LINKS at least SEAT_ARM_MARGIN_M from every target (``targets``: the
    released dish, named ``dish`` among them, the appliance, the other dishes, the statics); the first clear variant is the retreat.
    ``judge_release``: the open hand at T0 must pass too (the plan, as the gate's open_release_check judges the release); without
    it T0 is recorded only (in Isaac the jaws are already open there and only the motion is chosen). -> {"variant": the first clear
    or None, "variants": {v: {"ok", "why", "stages": {stage: {"jaws_mm", "jaws", "dish_mm", "links_mm", "links", ...}}}},
    "release": {...}, "rule"}"""
    mm = lambda d: None if not np.isfinite(d) else round(float(d) * 1e3, 1)
    dish_t = [t for t in targets if dish is not None and t[0] == dish]

    def measure(q):
        objs = arm.arm_objects(q, ("shoulder_link",))
        jaws, links = [o for o in objs if o[0] in JAW_LINKS], [o for o in objs if o[0] in SEAT_LINKS]
        dj, dl = min_clear(arm.fcl, jaws, targets, np, cutoff=.05), min_clear(arm.fcl, links, targets, np, cutoff=.05)
        dd = min_clear(arm.fcl, jaws, dish_t, np, cutoff=.05) if dish_t else (np.inf, None, None)
        return (bool(dj[0] >= JAW_MARGIN_M and dl[0] >= SEAT_ARM_MARGIN_M),
                {"jaws_mm": mm(dj[0]), "jaws": None if dj[1] is None else f"{dj[1]}|{dj[2]}", "dish_mm": mm(dd[0]),
                 "links_mm": mm(dl[0]), "links": None if dl[1] is None else f"{dl[1]}|{dl[2]}"})

    def keep_min(st, rec):
        for k_, nm_ in (("jaws_mm", "jaws"), ("dish_mm", None), ("links_mm", "links")):
            if rec[k_] is not None and (st.get(k_) is None or rec[k_] < st[k_]):
                st[k_] = rec[k_]
                if nm_:
                    st[nm_] = rec[nm_]
            st.setdefault(k_, None)
    ok0, rel = measure(np.asarray(q0, dtype=float))
    out = {"variant": None, "variants": {}, "release": {**rel, "ok": ok0, "judged": bool(judge_release)},
           "rule": "D4: the first clear of " + "; ".join(f"({v_}) {RETREAT_RULES[v_]}" for v_ in RETREATS)
                   + f": every pose IK-continuous, the open gripper and wrist 3 {JAW_MARGIN_M * 1e3:.0f} mm and the links "
                     f"{', '.join(SEAT_LINKS)} {SEAT_ARM_MARGIN_M * 1e3:.0f} mm from the released dish, the appliance, the other "
                     f"dishes and the statics (FCL)" + ("; the open hand at the release itself too" if judge_release else
                                                       "; the release pose recorded, not judged (the jaws already open there)")}
    for v_ in RETREATS:
        stages, prev, why = {}, np.asarray(q0, dtype=float), None
        if judge_release and not ok0:
            why = f"the open hand at the release: jaws {rel['jaws_mm']} mm ({rel['jaws']}), links {rel['links_mm']} mm ({rel['links']})"
        for stage, s_, T in retreat_poses(T0, up, v_, np):
            q = ik(T, prev)
            if q is None or float(np.abs(np.asarray(q) - prev).max()) > .1:
                stages.setdefault(stage, {})["ik_jump_at_mm"] = round(s_ * 1e3)
                why = why or f"IK branch jump on the {stage} at {s_ * 1e3:.0f} mm"
                break
            prev = np.asarray(q, dtype=float)
            ok, rec = measure(prev)
            keep_min(stages.setdefault(stage, {}), rec)
            if not ok and why is None:
                why = (f"the {stage} at {s_ * 1e3:.0f} mm: jaws {rec['jaws_mm']} mm ({rec['jaws']}), links {rec['links_mm']} mm "
                       f"({rec['links']})")
        out["variants"][v_] = {"ok": why is None, "why": why, "stages": stages}
        if why is None:
            out["variant"] = v_
            break
    return out


def retreat_brief(rt):
    """A retreat_check record in one line: the chosen variant, else every variant's first failure."""
    if rt.get("variant"):
        return f"retreat ({rt['variant']})"
    return "; ".join(f"({v_}) {r_.get('why')}" for v_, r_ in (rt.get("variants") or {}).items())


# plans/2026-11-04-fifteen-of-fifteen.md, round 3 (the user, 2026-11-05 00:05 EST): plate_05's drop (the descent line's lateral bias
# per load row, the rest wait after the open, the back-off before the rise) and the insert moves' position stop at the predicted stop
DESCENT_BIAS_MM = SEAT_DX_M * 1e3   # the load row's "descent_bias_mm" when it has none: the I4 descent line (the plate's goal) moved
                                    # this far toward the basket (+x of the LowerRack frame, the plate's lean side); SEAT_DX_M's 1 mm
DROP_REST_SPEED_M_S = .002        # D2 after the open: the released plate at rest = its centroid speed under 2 mm/s ...
DROP_REST_TICKS = 12              # ... on this many consecutive ticks
DROP_REST_CAP_TICKS = 300         # the wait after the open capped at this many ticks; RING_WAIT_TICKS (60) the minimum
DROP_BACK_M, DROP_BACK_STEP_M = .020, .005   # D2's retreat: back 20 mm along -TCP z in 5 mm steps, then the rise up the tine axis
DROP_RISE_STEP_M = .010           # the rise's checked poses every 10 mm (the last at the start height)
DROP_RETREATS = ("back-off", "rise-only")
DROP_RETREAT_RULE = ("D2's retreat (the user, 2026-11-05 00:05 EST): after the open and the rest wait, (back-off) the open hand back "
                     "DROP_BACK_M (20 mm) along -TCP z in 5 mm steps, then up the tine axis by the descent's length (back to the "
                     "start height), when every pose is IK-continuous (at most 0.1 rad per joint from the last) and keeps the open "
                     "gripper and wrist 3 (JAW_LINKS) JAW_MARGIN_M and the links SEAT_LINKS SEAT_ARM_MARGIN_M (2 mm) from the scene, the "
                     "rack and the released plate (Kit-free at its seat, in Isaac at its pose when the retreat starts); else (rise-only) "
                     "the open hand straight up the tine axis (the earlier D2 retreat). Drop rows only")
DROP_REST_RULE = ("D2's rest wait (the user, 2026-11-05 00:05 EST): after the jaws open the hand waits until the released plate rests "
                  f"(its centroid speed under {DROP_REST_SPEED_M_S * 1e3:.0f} mm/s on {DROP_REST_TICKS} consecutive ticks), at least "
                  f"RING_WAIT_TICKS ({RING_WAIT_TICKS}) and at most {DROP_REST_CAP_TICKS} ticks after the open's ramp; drop rows only")
INSERT_STOP_RULE = ("the insert moves' position stop (the user, 2026-11-05 00:05 EST): the guarded insertion is bounded at the "
                    "predicted sensor stop (the row's stop_pred; none predicted: the build's stop, or the goal); a sensor trip above "
                    "it ends the descent there as before; reaching it without a trip is a position stop (stop.kind 'predicted'): the "
                    "pre-open release check at the actual pose (the full, then the partial open; retreats a, b, c), the open, the "
                    "retreat, the settle, judged seated and at goal; the jaws stay closed only when no retreat is clear. The 30 mm "
                    "past the stop is no longer descended")


def descent_bias_mm(row):
    """The I4 descent line's lateral bias of a load row (mm toward the plate's lean side, +x of the LowerRack frame): the row's
    "descent_bias_mm", else DESCENT_BIAS_MM (today's 1 mm)."""
    b = (row or {}).get("descent_bias_mm")
    return float(DESCENT_BIAS_MM if b is None else b)


def drop_retreat_poses(T0, v, h_rise, variant, np):
    """[(stage, s m, TCP pose)] of D2's retreat ``variant`` (DROP_RETREATS) from the release TCP pose T0: the back-off poses every
    DROP_BACK_STEP_M along -TCP z (variant back-off), then the rise up the tine axis ``v`` by ``h_rise`` every DROP_RISE_STEP_M."""
    T0, v = np.asarray(T0, dtype=float), np.asarray(v, dtype=float)
    v = v / np.linalg.norm(v)
    out, T_b = [], T0
    if variant == "back-off":
        for j in range(1, int(round(DROP_BACK_M / DROP_BACK_STEP_M)) + 1):
            T = T0.copy()
            T[:3, 3] -= DROP_BACK_STEP_M * j * T0[:3, 2]
            out.append(("back-off", DROP_BACK_STEP_M * j, T))
        T_b = out[-1][2]
    h_rise = max(float(h_rise), 0.)
    for j in range(1, int(math.ceil(h_rise / DROP_RISE_STEP_M - 1e-9)) + 1):
        s_ = min(DROP_RISE_STEP_M * j, h_rise)
        T = T_b.copy()
        T[:3, 3] += s_ * v
        out.append(("rise", s_, T))
    return out


def drop_retreat_check(arm, ik, q0, T0, v, h_rise, targets, np, dish=None):
    """DROP_RETREAT_RULE: the back-off then the rise from the open hand at the TCP pose T0 (joints q0), every pose IK-continuous
    (``ik(T, q_seed)``, at most 0.1 rad per joint from the last), the open gripper and wrist 3 at least JAW_MARGIN_M and SEAT_LINKS
    at least SEAT_ARM_MARGIN_M from every target (``targets``: the released plate, named ``dish`` among them, the appliance, the
    other dishes, the statics). Clear: variant "back-off" (back_mm 20); else "rise-only" (the earlier retreat, whose check is
    recorded too). -> {"variant", "back_mm", "checks": {variant: {"ok", "why", "stages": {stage: minima}}}, "release", "rule"}"""
    mm = lambda d: None if not np.isfinite(d) else round(float(d) * 1e3, 1)
    dish_t = [t for t in targets if dish is not None and t[0] == dish]

    def measure(q):
        objs = arm.arm_objects(q, ("shoulder_link",))
        jaws, links = [o for o in objs if o[0] in JAW_LINKS], [o for o in objs if o[0] in SEAT_LINKS]
        dj, dl = min_clear(arm.fcl, jaws, targets, np, cutoff=.05), min_clear(arm.fcl, links, targets, np, cutoff=.05)
        dd = min_clear(arm.fcl, jaws, dish_t, np, cutoff=.05) if dish_t else (np.inf, None, None)
        return (bool(dj[0] >= JAW_MARGIN_M and dl[0] >= SEAT_ARM_MARGIN_M),
                {"jaws_mm": mm(dj[0]), "jaws": None if dj[1] is None else f"{dj[1]}|{dj[2]}", "dish_mm": mm(dd[0]),
                 "links_mm": mm(dl[0]), "links": None if dl[1] is None else f"{dl[1]}|{dl[2]}"})

    def keep_min(st, rec):
        for k_, nm_ in (("jaws_mm", "jaws"), ("dish_mm", None), ("links_mm", "links")):
            if rec[k_] is not None and (st.get(k_) is None or rec[k_] < st[k_]):
                st[k_] = rec[k_]
                if nm_:
                    st[nm_] = rec[nm_]
            st.setdefault(k_, None)
    ok0, rel = measure(np.asarray(q0, dtype=float))
    out = {"variant": None, "back_mm": 0., "rise_mm": round(max(float(h_rise), 0.) * 1e3, 1), "checks": {},
           "release": {**rel, "ok": ok0}, "rule": DROP_RETREAT_RULE}
    for v_ in DROP_RETREATS:
        stages, prev, why = {}, np.asarray(q0, dtype=float), None
        for stage, s_, T in drop_retreat_poses(T0, v, h_rise, v_, np):
            q = ik(T, prev)
            if q is None or float(np.abs(np.asarray(q) - prev).max()) > .1:
                stages.setdefault(stage, {})["ik_jump_at_mm"] = round(s_ * 1e3)
                why = why or f"IK branch jump on the {stage} at {s_ * 1e3:.0f} mm"
                break
            prev = np.asarray(q, dtype=float)
            ok, rec = measure(prev)
            keep_min(stages.setdefault(stage, {}), rec)
            if not ok and why is None:
                why = (f"the {stage} at {s_ * 1e3:.0f} mm: jaws {rec['jaws_mm']} mm ({rec['jaws']}), links {rec['links_mm']} mm "
                       f"({rec['links']})")
        out["checks"][v_] = {"ok": why is None, "why": why, "stages": stages}
        if v_ == "back-off" and why is None:
            out.update(variant="back-off", back_mm=round(DROP_BACK_M * 1e3, 1))
            break
    if out["variant"] is None:                         # the earlier retreat: the rise alone, whatever its own check says
        out["variant"] = "rise-only"
    return out


# plans/2026-11-04-fifteen-of-fifteen.md, cup_05 (the user, 2026-11-04 22:20 EST): the partial open at the guarded insertion's release
PARTIAL_OPEN_CLEAR_M = .005       # the release width: the pad gap that holds the dish at the pinch plus this much per side (10 mm in
                                  # all), a finger_joint position the real 2F-85 commands (= frigidaire_robot_reach.PARTIAL_OPEN_CLEAR_M)
OPEN_MODES = ("full", "partial")  # the open at a guarded insertion's release, in this order: the first clear of OPEN_MODES x RETREATS
PARTIAL_OPEN_TOL_RAD = .005       # Kit, the partial open: after its ramp and RING_WAIT_TICKS the hand moves once finger_joint is this
PARTIAL_OPEN_WAIT_TICKS = 360     # close to its target, waiting at most this many ticks more (3 s; the 8 / 4 / 2.5 drive lags ~0.6 s)
PARTIAL_OPEN_RULE = ("the open at a guarded insertion's release (the user, 2026-11-04 22:20 EST): the full open (finger_joint 0, the arm's "
                     "open gripper hulls) by default; where no retreat of RETREATS is clear with it (the open hand at the release judged "
                     "too: JAW_MARGIN_M, SEAT_ARM_MARGIN_M), the partial open: the jaws to the pad gap holding the dish at the pinch "
                     "(grasp.pad_at at the hold's finger angle: the hold trial's last frame at the plan, the measured finger_joint at the "
                     "sensed stop in Isaac) plus PARTIAL_OPEN_CLEAR_M (5 mm) per side, the finger_joint target by grasp.theta_at_inner, "
                     "its hull the hold trial's 2F-85 linkage at that angle (gripper_hulls_at); the first clear of (full, partial) x "
                     "(a, b, c) wins; the retreat runs with that width held (finger_joint within PARTIAL_OPEN_TOL_RAD before the hand "
                     "moves); the 2 mm rule, the retreats and every other limit unchanged")
_LINKAGE, _OPEN_HULLS = {}, {}


def gripper_linkage(run, traj, np):
    """The 2F-85 linkage of one Isaac trial, Kit-free (the partial open): per gripper body its pose in wrist 3's frame at the trial's
    first frame (the jaws open, finger_joint 0: the arm model's open hulls within 0.01 mm) and at its last (closed on the dish:
    closed_gripper_hulls), its collider points (robot_meshes.json, in its own frame), and the motion between: a knuckle or an outer
    finger turns about its pivot (the axis from the two poses, the pivot point solved in the plane of motion), an inner finger (the
    pad's body) translates on its side's outer knuckle's circle (the parallelogram: the crank vector solved from its displacement; its
    own turn, under 0.02 deg, kept proportional). -> {"theta_closed": the left outer knuckle's turn (finger_joint at the last
    frame), "bodies": {name: {"X0", "X1", "angle", "axis", "pivot", "pts", "crank"...}}, "run", "traj"}"""
    from scipy.spatial.transform import Rotation
    from dishsim_frigidaire.robot.rig import T_of
    key = (str(run), str(traj))
    if key in _LINKAGE:
        return _LINKAGE[key]
    rm = json.loads((Path(run) / "trials/robot_meshes.json").read_text())
    paths = rm["body_paths"]
    z = np.load(Path(run) / "trials" / traj)

    def Tb(k, i):
        w = z["body_quat_wxyz"][k, i]
        return T_of(z["body_pos"][k, i], (w[1], w[2], w[3], w[0]))
    iw3 = next(j for j, p in enumerate(paths) if p.endswith("/wrist_3_link"))
    W0, W1 = np.linalg.inv(Tb(0, iw3)), np.linalg.inv(Tb(-1, iw3))
    bodies = {}
    for j, p in enumerate(paths):
        if "/Gripper/" not in p:
            continue
        X0, X1 = W0 @ Tb(0, j), W1 @ Tb(-1, j)
        R_ = X1[:3, :3] @ X0[:3, :3].T
        rv = Rotation.from_matrix(R_).as_rotvec()
        ang = float(np.linalg.norm(rv))
        b = {"X0": X0, "X1": X1, "angle": ang, "axis": rv / ang if ang > 1e-9 else np.array([0., 0., 1.]), "pivot": None,
             "pts": [np.asarray(h, dtype=float) for h in rm["hulls"].get(str(j), [])]}
        if ang > 1e-3:                                 # a turning body: its pivot on the axis, in the plane of X0's origin
            A = np.vstack([np.eye(3) - R_, b["axis"][None]])
            b["pivot"] = np.linalg.lstsq(A, np.r_[X1[:3, 3] - R_ @ X0[:3, 3], b["axis"] @ X0[:3, 3]], rcond=None)[0]
        bodies[p.split("/")[-1]] = b
    for nm, b in bodies.items():                       # an inner finger: its side's outer knuckle's circle (the parallelogram)
        d1, k = b["X1"][:3, 3] - b["X0"][:3, 3], bodies.get(nm.split("_")[0] + "_outer_knuckle")
        if b["pivot"] is None and k is not None and k["pivot"] is not None and float(np.linalg.norm(d1)) > 1e-6:
            A = np.vstack([rodrigues_R(k["axis"], k["angle"], np) - np.eye(3), k["axis"][None]])
            b.update(crank=np.linalg.lstsq(A, np.r_[d1, 0.], rcond=None)[0], crank_axis=k["axis"], crank_angle=k["angle"])
    k0 = bodies.get("left_outer_knuckle")
    if k0 is None or k0["angle"] < 1e-3:
        raise RuntimeError(f"{run}/trials/{traj}: no left outer knuckle turn between the first and the last frame")
    _LINKAGE[key] = {"theta_closed": k0["angle"], "bodies": bodies, "run": run, "traj": traj}
    return _LINKAGE[key]


def _linkage_pose(b, s, np):
    """A gripper body's pose (wrist 3's frame) at s = theta / theta_closed of its trial's linkage (gripper_linkage): s 0 the first
    frame, s 1 the last."""
    X0, X = b["X0"], np.eye(4)
    R_ = rodrigues_R(b["axis"], s * b["angle"], np)
    X[:3, :3] = R_ @ X0[:3, :3]
    if b["pivot"] is not None:
        X[:3, 3] = b["pivot"] + R_ @ (X0[:3, 3] - b["pivot"])
    elif b.get("crank") is not None:
        X[:3, 3] = X0[:3, 3] + (rodrigues_R(b["crank_axis"], s * b["crank_angle"], np) - np.eye(3)) @ b["crank"]
    else:
        X[:3, 3] = X0[:3, 3] + s * (b["X1"][:3, 3] - X0[:3, 3])
    return X


def linkage_pad_gap(link, theta, np):
    """The finger pads' inner gap (m) at finger angle ``theta`` of a trial's linkage: the two pads (the inner fingers' second
    collider) facing each other along the line through their centres. None without both pads."""
    s, P = float(theta) / link["theta_closed"], {}
    for side in ("left", "right"):
        b = link["bodies"].get(f"{side}_inner_finger")
        if b is None or len(b["pts"]) < 2:
            return None
        X = _linkage_pose(b, s, np)
        P[side] = b["pts"][1] @ X[:3, :3].T + X[:3, 3]
    e = P["right"].mean(0) - P["left"].mean(0)
    e = e / np.linalg.norm(e)
    return float((P["right"] @ e).min() - (P["left"] @ e).max())


def gripper_hulls_at(C, np, link, theta):
    """The gripper's colliders at finger angle ``theta`` (rad, 0 = open) as convex hulls in wrist 3's frame (a HullList like
    ArmWorld.hulls["gripper"], no trial attached), from ``link`` (gripper_linkage): s = theta / theta_closed, every turning body
    turned s times its open-to-closed angle about its pivot, every inner finger moved along its crank's circle by s times its side's
    angle (the pads parallel); s 0 gives the open frame (= the arm model's open hulls), s 1 the closed frame (= closed_gripper_hulls).
    Cached per (trial, theta)."""
    key = (str(link["run"]), str(link["traj"]), round(float(theta), 6))
    if key not in _OPEN_HULLS:
        s, out = float(theta) / link["theta_closed"], HullList()
        for b in link["bodies"].values():
            X = _linkage_pose(b, s, np)
            for P in b["pts"]:
                out.append(C._hull(P @ X[:3, :3].T + X[:3, 3]))
        _OPEN_HULLS[key] = out
    return _OPEN_HULLS[key]


def release_width(theta_hold, clear=PARTIAL_OPEN_CLEAR_M):
    """The partial open's command: the pad gap holding the dish at finger angle ``theta_hold`` by the grasp's width-to-joint mapping
    (grasp.pad_at: twice the pad's inner face offset from the closing axis) plus ``clear`` per side, and the finger angle that opens
    to it (grasp.theta_at_inner). None when that gap reaches the full open's (grasp.PAD_OPEN_M, finger_joint 0)."""
    from dishsim_frigidaire.robot import grasp as G
    y_h = G.pad_at(float(theta_hold))[0]
    y = y_h + float(clear)
    if y >= G.PAD_OPEN_M[0] - 1e-6:
        return None
    return {"theta_hold_rad": float(theta_hold), "gap_hold_mm": 2e3 * y_h, "open_mm": 2e3 * y, "open_theta_rad": float(G.theta_at_inner(y))}


def open_full_record():
    """The full open's record: finger_joint 0, the arm's open gripper hulls, the pad gap by grasp's mapping (grasp.PAD_OPEN_M)."""
    from dishsim_frigidaire.robot import grasp as G
    return {"open_mode": "full", "open_mm": round(2e3 * G.PAD_OPEN_M[0], 2), "open_theta_rad": 0.,
            "open_hull": "the arm's open gripper hulls (the gripper colliders at finger_joint 0, the arm model)"}


def partial_open(C, np, closed, theta_hold=None, clear=PARTIAL_OPEN_CLEAR_M):
    """The partial open at a release (PARTIAL_OPEN_RULE): the hold's trial (``closed``: the closed jaws, a HullList) gives the linkage;
    the hold's finger angle ``theta_hold`` (None: the trial's own at its last frame, the hold the closed hand models; in Isaac the
    measured finger_joint at the sensed stop) the command (release_width, ``clear`` per side); the hull there (gripper_hulls_at).
    -> {"open_mode": "partial", "open_mm" (the commanded pad gap, grasp's mapping), "open_theta_rad" (the finger_joint target),
    "theta_hold_rad", "gap_hold_mm", "open_mm_linkage" / "gap_hold_mm_linkage" (the trial linkage's pad gaps there), "open_hull"
    (the source), "hulls"} or {"open_mode": "partial", "hulls": None, "why"} (no partial open)."""
    from dishsim_frigidaire.robot import grasp as G
    run, traj = getattr(closed, "run", None), getattr(closed, "traj", None)
    if run is None or traj is None:
        return {"open_mode": "partial", "hulls": None, "why": "no partial open: the closed hand carries no Isaac trial (no linkage)"}
    src = rel_path(Path(run) / "trials" / traj)
    try:
        link = gripper_linkage(run, traj, np)
    except (OSError, ValueError, KeyError, IndexError, RuntimeError) as e_:
        return {"open_mode": "partial", "hulls": None, "why": f"no partial open: the linkage of {src}: {type(e_).__name__}: {e_}"}
    th_h = link["theta_closed"] if theta_hold is None else float(theta_hold)
    w = release_width(th_h, clear)
    if w is None:
        return {"open_mode": "partial", "hulls": None, "theta_hold_rad": round(th_h, 4),
                "why": f"no partial open: the pad gap at the hold ({2e3 * G.pad_at(th_h)[0]:.1f} mm) plus 2 x {clear * 1e3:.0f} mm reaches "
                       f"the full open ({2e3 * G.PAD_OPEN_M[0]:.1f} mm)"}
    g_h, g_o = linkage_pad_gap(link, th_h, np), linkage_pad_gap(link, w["open_theta_rad"], np)
    return {"open_mode": "partial", "open_mm": round(w["open_mm"], 2), "open_theta_rad": round(w["open_theta_rad"], 6),
            "theta_hold_rad": round(th_h, 4), "gap_hold_mm": round(w["gap_hold_mm"], 2),
            "theta_hold_source": "the hold trial's last frame" if theta_hold is None else "the measured finger_joint",
            "clear_mm_per_side": round(clear * 1e3, 2),
            "gap_hold_mm_linkage": None if g_h is None else round(g_h * 1e3, 2),
            "open_mm_linkage": None if g_o is None else round(g_o * 1e3, 2),
            "open_hull": (f"the 2F-85 linkage of {src} (first frame open, last frame closed on the dish at finger_joint "
                          f"{link['theta_closed']:.4f} rad) at finger_joint {w['open_theta_rad']:.4f} rad"),
            "hulls": gripper_hulls_at(C, np, link, w["open_theta_rad"])}


def open_record(op):
    """An open's record for a row: ``op`` without its hulls (None stays None)."""
    return None if op is None else {k_: v_ for k_, v_ in op.items() if k_ != "hulls"}


def retreat_check_open(arm, ik, q0, T0, up, targets, np, dish=None, judge_release=False, partial=None):
    """The open at a guarded insertion's release (PARTIAL_OPEN_RULE): retreat_check with the full open (the arm's open gripper hulls)
    first; when none of its retreats is clear, again with the partial open's hulls (``partial``: partial_open's record, or a callable
    returning it, called only then), the 2 mm rule unchanged; the first clear of OPEN_MODES x RETREATS wins. -> the chosen open's
    retreat_check record (none clear: the full open's) with "open" (the chosen open's record, None when none is clear) and "modes"
    ({mode: {"variant", "why", "release"}} of every open tried; "full_variants" when the partial open won)."""
    full = retreat_check(arm, ik, q0, T0, up, targets, np, dish=dish, judge_release=judge_release)
    modes = {"full": {"variant": full["variant"], "why": None if full["variant"] else retreat_brief(full),
                      "release": full["release"]}}
    if full["variant"] is not None:
        return {**full, "open": open_full_record(), "modes": modes}
    pt_ = partial() if callable(partial) else partial
    if not pt_ or not pt_.get("hulls"):
        modes["partial"] = {"variant": None, "why": (pt_ or {}).get("why") or "no partial open offered", "release": None}
        return {**full, "open": None, "modes": modes}
    keep = arm.hulls["gripper"]
    arm.hulls["gripper"] = pt_["hulls"]
    try:
        pr = retreat_check(arm, ik, q0, T0, up, targets, np, dish=dish, judge_release=judge_release)
    finally:
        arm.hulls["gripper"] = keep
    modes["partial"] = {"variant": pr["variant"], "why": None if pr["variant"] else retreat_brief(pr), "release": pr["release"],
                        "open_mm": pt_.get("open_mm"), "open_theta_rad": pt_.get("open_theta_rad")}
    if pr["variant"] is None:
        return {**full, "open": None, "modes": modes}
    return {**pr, "open": open_record(pt_), "modes": modes, "full_variants": full["variants"]}


# plans/2026-11-04-fifteen-of-fifteen.md, cup_05 (the user, 2026-11-04 23:02 EST): the guided release, D2's plate drop carried to the
# guarded insertion as the fallback after the sensor-trip release (full, then partial) finds no clear retreat at any start
DROP_CHECK_MM = 5                 # its checkpoints: every 5 mm up from the predicted stop, plus the build's stop (the descent line's
                                  # end); the arm walked IK-continuously in 1 mm steps between them (D2's spacing)
INSERT_DROP_RULE = ("the guided release of a guarded insertion (the user, 2026-11-04 23:02 EST; D2's plate drop carried to the insert "
                    "moves): only when no start clears the sensor-trip release (no (full, partial) x (a, b, c) clear 2 mm at the "
                    "predicted sensor stop), per such start the hand holding the dish in its goal orientation is walked up the rack's "
                    "up axis from the predicted stop (IK-continuous, 1 mm steps), checked every DROP_CHECK_MM (5 mm) and at the build's "
                    "stop: h_end = the lowest checkpoint where the open hand at the chosen width (full, then partial) and SEAT_LINKS "
                    "keep 2 mm from every target at the release pose and one retreat (a, b, c) clears 2 mm throughout (the dish at its "
                    "release pose among the targets), provided the dish's lowest point there is at least DROP_GUIDED_MIN_M (20 mm) "
                    "below the tips of the rack's wires inside its goal footprint (footprint_tips: the highest capsule top there), else "
                    "'not tine-guided'; candidates by the lowest h_end, full before partial, the held clearance, the start; in Isaac the "
                    "descent ends there by position (an earlier sensor trip: the release re-checked at the actual pose), the guidance "
                    "re-measured (under 20 mm: the jaws stay closed), the release re-checked (full, then partial, a, b, c; none: the "
                    "jaws stay closed), the jaws opened to the chosen width and held through the retreat; never below the predicted "
                    "stop, never unguided, no margin loosened")


def footprint_tips(arm, rack, T_goal, points, np, n=48):
    """The tips of the rack wires around a dish at its goal (the guided release's guidance, INSERT_DROP_RULE): the dish's footprint
    (its visual ``points`` at ``T_goal``: the largest horizontal distance from the vertical through their centroid) and every FCL
    capsule of ``rack`` (arm.checker.components / parts, at the rack's pose now) sampled at ``n`` points along its axis; a sample
    within the footprint radius plus the capsule's radius of that vertical lies inside; per capsule its highest inside sample plus its
    radius (its top there); the tips = the highest of them. -> {"tips_z_m", "capsules", "tops_z_m" (the five highest),
    "footprint_r_m", "center_xy"}"""
    T_goal = np.asarray(T_goal, dtype=float)
    P = np.asarray(points, dtype=float) @ T_goal[:3, :3].T + T_goal[:3, 3]
    cxy = P[:, :2].mean(0)
    r_fp = float(np.linalg.norm(P[:, :2] - cxy, axis=1).max())
    tops = []
    for o, p in zip(arm.checker.components[rack]["objects"], arm.checker.parts[rack]):
        g = p.geometry
        if type(g).__name__ != "Capsule":
            continue
        c = np.asarray(o.getTranslation(), dtype=float)
        s = c + np.linspace(-1., 1., n)[:, None] * (np.asarray(o.getRotation(), dtype=float)[:, 2] * g.lz / 2)
        inside = np.linalg.norm(s[:, :2] - cxy, axis=1) <= r_fp + g.radius
        if inside.any():
            tops.append(float(s[inside, 2].max() + g.radius))
    tops.sort(reverse=True)
    return {"tips_z_m": tops[0] if tops else None, "capsules": len(tops), "tops_z_m": [round(t, 5) for t in tops[:5]],
            "footprint_r_m": round(r_fp, 4), "center_xy": [round(float(v), 4) for v in cxy]}


def dish_low_z(T, points, np):
    """The lowest world z of a dish's visual ``points`` at the pose T (a mouth-down cup: its rim)."""
    T = np.asarray(T, dtype=float)
    return float((np.asarray(points, dtype=float) @ T[:3, :3].T + T[:3, 3])[:, 2].min())


def release_pose_clear(arm, q, targets, np):
    """retreat_check's release test at joint state q: the open hand (JAW_LINKS) JAW_MARGIN_M and SEAT_LINKS SEAT_ARM_MARGIN_M from
    every target. (ok, {"jaws_mm", "jaws", "links_mm", "links"})"""
    mm = lambda d: None if not np.isfinite(d) else round(float(d) * 1e3, 1)
    objs = arm.arm_objects(q, ("shoulder_link",))
    dj = min_clear(arm.fcl, [o for o in objs if o[0] in JAW_LINKS], targets, np, cutoff=.05)
    dl = min_clear(arm.fcl, [o for o in objs if o[0] in SEAT_LINKS], targets, np, cutoff=.05)
    return (bool(dj[0] >= JAW_MARGIN_M and dl[0] >= SEAT_ARM_MARGIN_M),
            {"jaws_mm": mm(dj[0]), "jaws": None if dj[1] is None else f"{dj[1]}|{dj[2]}", "links_mm": mm(dl[0]),
             "links": None if dl[1] is None else f"{dl[1]}|{dl[2]}"})


def insert_drop_walk(arm, ik, kind, oid, T_st, q_st, T_dst, up, h_stop, h_line, h_top, targets, partial, tips, points, np):
    """The guided release at one start (INSERT_DROP_RULE): from the predicted stop (the TCP ``T_st`` at joints ``q_st``, the dish at
    ``T_dst``, its centroid ``h_stop`` m above the goal's) the hand walked up the rack's up axis ``up`` (the dish's centroid rising
    1 mm per step, IK-continuous by ``ik``, at most 0.1 rad per joint and step) to ``h_top``; at every DROP_CHECK_MM checkpoint and at
    ``h_line`` (the descent line's end) while the dish's lowest point (dish_low_z) stays DROP_GUIDED_MIN_M under ``tips``
    (footprint_tips): the open hand at the full open (the arm's hulls), then the partial open (``partial``: partial_open's record or a
    callable, built on first need) at the release pose (release_pose_clear) and, where that is clear, its retreats (retreat_check,
    the release judged too) with the dish at its release pose among ``targets``; h_end = the first clear checkpoint. -> {"ok",
    "h_end_m", "release_mm", "stop_mm", "drop_mm", "guided_mm", "open", "variant", "release", "variants", "q_end", "T_tcp_end",
    "T_dish_end", "first_violation", "tips_z_m", "rim_low_z_m", "checked", "why"}"""
    up = np.asarray(up, dtype=float) / np.linalg.norm(up)
    uz = max(float(up[2]), 1e-6)
    tz = (tips or {}).get("tips_z_m")
    out = {"ok": False, "h_end_m": None, "release_mm": None, "stop_mm": round(h_stop * 1e3, 1), "drop_mm": None, "guided_mm": None,
           "open": None, "variant": None, "release": None, "variants": None, "first_violation": None,
           "tips_z_m": None if tz is None else round(tz, 5), "rim_low_z_m": None, "checked": [], "why": None, "mode": "drop",
           "guided_min_mm": round(DROP_GUIDED_MIN_M * 1e3)}
    if tz is None:
        out["why"] = "no rack wire inside the dish's goal footprint (no guidance)"
        return out
    n_top = int(math.floor((h_top - h_stop) * 1e3 + 1e-9))
    k_line = (h_line - h_stop) * 1e3
    steps = sorted(set(range(0, n_top + 1)) | ({round(k_line, 4)} if 0. < k_line <= n_top else set()))
    prev, pt_ = np.asarray(q_st, dtype=float), {}

    def part():
        if "p" not in pt_:
            pt_["p"] = partial() if callable(partial) else partial
        return pt_["p"]
    for k in steps:
        off = (k / 1e3) / uz * up
        T = np.array(T_st, dtype=float, copy=True)
        T[:3, 3] += off
        q = prev if k == 0 else ik(T, prev)
        if q is None or float(np.abs(np.asarray(q) - prev).max()) > .1:
            out["why"] = f"IK branch jump {k:.0f} mm above the predicted stop"
            return out
        prev = np.asarray(q, dtype=float)
        if not (abs(k - round(k)) < 1e-6 and int(round(k)) % DROP_CHECK_MM == 0) and abs(k - k_line) > 1e-6:
            continue
        T_d = np.array(T_dst, dtype=float, copy=True)
        T_d[:3, 3] += off
        low = dish_low_z(T_d, points, np)
        g_mm = round((tz - low) * 1e3, 1)
        h_mm = round((h_stop + k / 1e3) * 1e3, 1)
        if g_mm < DROP_GUIDED_MIN_M * 1e3 - 1e-9:
            out["why"] = (f"not tine-guided: no clear checkpoint up to {h_mm} mm above the goal, where the dish's lowest point is "
                          f"{g_mm} mm below the wire tips (at least {DROP_GUIDED_MIN_M * 1e3:.0f} mm)")
            out["checked"].append({"h_mm": h_mm, "guided_mm": g_mm, "clear": None})
            return out
        tg_ = list(targets) + [(oid, arm.checker._body(kind, _pose_of(T_d), oid))]
        rec, found = {"h_mm": h_mm, "guided_mm": g_mm}, None
        for mode in OPEN_MODES:
            hulls = None
            if mode == "partial":
                p_ = part()
                if not p_ or not p_.get("hulls"):
                    rec["partial"] = {"why": (p_ or {}).get("why") or "no partial open"}
                    break
                hulls = p_["hulls"]
            keep = arm.hulls["gripper"]
            if hulls is not None:
                arm.hulls["gripper"] = hulls
            try:
                ok_, r_ = release_pose_clear(arm, prev, tg_, np)
                rc = retreat_check(arm, ik, prev, T, up, tg_, np, dish=oid, judge_release=True) if ok_ else None
            finally:
                arm.hulls["gripper"] = keep
            rec[mode] = {**r_, "variant": None if rc is None else rc["variant"],
                         "why": None if rc is None or rc["variant"] else retreat_brief(rc)}
            if rc is not None and rc["variant"] is not None:
                found = (mode, rc)
                break
        rec["clear"] = None if found is None else found[0]
        out["checked"].append(rec)
        if found is None:
            if out["first_violation"] is None:
                f_ = rec.get("full") or {}
                out["first_violation"] = {"at_mm": h_mm, "pair": f_.get("jaws") if (f_.get("jaws_mm") or 0.) < JAW_MARGIN_M * 1e3
                                          else f_.get("links"), "full": rec.get("full"), "partial": rec.get("partial")}
            continue
        mode, rc = found
        out.update(ok=True, h_end_m=h_stop + k / 1e3, release_mm=h_mm, drop_mm=round(k, 1), guided_mm=g_mm,
                   open=open_full_record() if mode == "full" else open_record(part()), variant=rc["variant"],
                   release=rc["release"], variants=rc["variants"], q_end=prev, T_tcp_end=T, T_dish_end=T_d,
                   rim_low_z_m=round(low, 5), why=None)
        return out
    out["why"] = f"no clear checkpoint up to the descent start ({round((h_stop + n_top / 1e3) * 1e3, 1)} mm above the goal)"
    return out


def cup_gate_hulls(C, np, gate_dir):
    """The closed jaws of a cup side-pinch gate run (its median-pivot held trial, the last frame) in wrist 3's frame, and the
    source; ``gate_dir`` missing or with no held trial: the h45 gate's (CUP_H45_GATE)."""
    for run in (gate_dir, CUP_H45_GATE):
        f = None if run is None else Path(run) / "trials.jsonl"
        if f is None or not f.is_file():
            continue
        held = sorted([r for r in (json.loads(x) for x in f.read_text().splitlines() if x.strip()) if r.get("g3") and r.get("traj")],
                      key=lambda r: r["hold"]["settle_rotation_deg"])
        if held:
            tp = Path(held[len(held) // 2]["traj"])
            return closed_gripper_hulls(C, np, run=ROOT / tp.parent.parent, traj=tp.name), f"{tp} (median-pivot held trial, last frame)"
    raise RuntimeError(f"no cup gate with a held trial at {gate_dir} or {CUP_H45_GATE}")


def seat_bowl_search(rig, arm, homes, M, T_goal, c, oid, kind, kw, z_safe, closed_hulls, keep=DROP_TRIES,
                     spins=CARRY_SPINS_DEG):
    """--carry seat (plans/2026-10-29-bowl-last.md), Kit-free, before the arm moves. Per carried tilt (overlean_tilts: the goal's
    lean, then 5 deg steps to 90) the bowl alone lowered straight down from 350 mm (perch_descent, spin 0: the bowl is
    axisymmetric) gives its first touch h_b (the bowl's height above its goal), the contact and the static roll; per spin
    (-45..45, 5 deg) the tool (closed jaws + wrist 3, rigid with the TCP) lowered with it gives its first touch h_t; the
    predicted stop = the higher of the two (the hand's when h_t > h_b), the release height above the seat. Pairs ranked by
    (the stop height, a bowl stop before a hand stop, mouth-down roll at the bowl's own perch); per pair, combos = free home
    branch x elbow-up IK branch of the descent start (the lowest DESCENT_GRID_M multiple >= stop + 50 mm whose IK is free at the
    carry margins with the open and the closed jaws) whose straight line down to the stop is IK-continuous with the links
    (SEAT_LINKS) >= SEAT_ARM_MARGIN_M from the scene; within a pair the smallest largest-joint travel over transit_to's two legs
    (home -> the descent start at z_safe -> the descent start), ties the sum. Pairs are evaluated in rank order until the best
    combo's home branch has ``keep`` combos and no later pair can tie them."""
    import numpy as np
    t0 = time.monotonic()
    homes = [np.asarray(q, dtype=float) for q in homes]
    targets = scene_targets(arm, ignore=(oid,))
    bowl_targets = [t for t in targets if t[0] not in arm.statics]
    tool = tool_hulls(arm, closed_hulls)
    hd = goal_hor(T_goal, np)
    ez = np.array([0., 0., 1.])
    com = np.asarray(PERCH_COM_DISH_M, dtype=float)
    open_hulls = arm.hulls["gripper"]
    memo = {}
    counts = {"tilts": 0, "pairs": 0, "hand_stops": 0, "pairs_tried": 0, "no_start": 0, "links": 0, "passing": 0}

    def wkey(q):
        return tuple(np.round((np.asarray(q) + np.pi) % (2 * np.pi) - np.pi, 6))

    def free2(q):
        k = wkey(q)
        if k not in memo:
            ok = arm.free(q, **kw)
            if ok:
                arm.hulls["gripper"] = closed_hulls
                try:
                    ok = arm.free(q, **kw)
                finally:
                    arm.hulls["gripper"] = open_hulls
            memo[k] = ok
        return memo[k]

    def roll(T_d, cp):
        off = (T_d[:3, :3] @ com + T_d[:3, 3]) - cp
        return round(float(off @ hd) * 1e3, 1), round(float(off @ np.cross(ez, hd)) * 1e3, 1)

    tilts, pairs = [], []
    for th in overlean_tilts(T_goal, np):
        r = perch_descent(arm, kind, lambda h, th=th: overlean_tcp(M, T_goal, c, h, 0., th, np)[1], oid, bowl_targets, np)
        rec = {"theta": round(th, 2), "h_b_mm": None}
        tilts.append(rec)
        if r is None:
            continue
        h_b, tn, cp, _ = r
        cm, cl = roll(overlean_tcp(M, T_goal, c, h_b, 0., th, np)[1], cp) if cp is not None else (None, None)
        rec.update(h_b_mm=round(h_b * 1e3), h_b=h_b, target=tn, part=part_name(arm, tn, cp, np), com_mouth_mm=cm,
                   com_lateral_mm=cl, mouth_down=bool(cm is not None and cm > 0.))
        for s_ in spins:
            counts["pairs"] += 1
            ht = tool_descent(arm, lambda h, s_=s_, th=th: overlean_tcp(M, T_goal, c, h, s_, th, np)[0], tool, targets, np,
                              h_floor=h_b - 1e-6)
            hand = ht is not None and ht[0] > h_b + 1e-9
            counts["hand_stops"] += int(hand)
            stop = ht[0] if hand else h_b
            pairs.append({"theta": th, "spin": float(s_), "stop": stop, "stop_mm": round(stop * 1e3), "body": "hand" if hand else "bowl",
                          "stop_target": ht[1] if hand else tn, "h_b": h_b, "h_b_mm": round(h_b * 1e3), "target": tn,
                          "part": rec["part"], "com_mouth_mm": cm, "com_lateral_mm": cl, "mouth_down": rec["mouth_down"]})
    counts["tilts"] = len(tilts)
    K0 = lambda p: (p["stop_mm"], p["body"] != "bowl", not p["mouth_down"])
    pairs.sort(key=lambda p: (K0(p), p["theta"], abs(p["spin"])))
    K = lambda x: K0(x) + (x["cost"], x["sum"])
    passing, b_star = [], None

    def kth():
        ps = sorted([x for x in passing if x["b"] == b_star], key=K)
        return K(ps[keep - 1]) if len(ps) >= keep else None

    first_fail = {}
    for p in pairs:
        kk = kth()
        if kk is not None and K0(p) > kk[:3]:
            break
        counts["pairs_tried"] += 1
        T_c, T_d = overlean_tcp(M, T_goal, c, p["stop"], p["spin"], p["theta"], np)
        k0 = int(math.ceil((p["stop"] + DESCENT_ABOVE_M) / DESCENT_GRID_M - 1e-9))
        got = False
        for k in range(k0, k0 + SEAT_START_GRID_N):
            H = round(k * DESCENT_GRID_M, 3)
            T_H = np.array(T_c, copy=True)
            T_H[2, 3] += H - p["stop"]
            for b, qh in enumerate(homes):
                for q_H in rig.ik_all(T_H, qh):
                    if not free2(q_H):
                        continue
                    line = rig.line_q(T_H, T_c, max(3, int(round((H - p["stop"]) / .005)) + 1), q_H)
                    if line is None:
                        first_fail["IK line breaks"] = first_fail.get("IK line breaks", 0) + 1
                        continue
                    bad = next((lc for lc in (links_clear(arm, q, targets, np) for q in [line[-1]] + line[::3]) if not lc[0]), None)
                    if bad is not None:
                        counts["links"] += 1
                        key_ = f"{bad[1][1]} vs {bad[1][2]} within 2 mm"
                        first_fail[key_] = first_fail.get(key_, 0) + 1
                        continue
                    legs, per = two_legs(rig, arm, qh, T_H, q_H, z_safe, kw, np)
                    passing.append({**p, "b": b, "T_c": T_c, "q_c": line[-1], "h_start": H, "T_start": T_H, "q_start": q_H,
                                    "legs": legs, "cost": float(per.max()), "sum": float(per.sum())})
                    counts["passing"] += 1
                    got = True
            if got:
                break
        if not got:
            counts["no_start"] += 1
        elif b_star is None:
            b_star = min(passing, key=K)["b"]
    ranked = sorted([x for x in passing if x["b"] == b_star], key=K)
    return {"chosen": ranked[0] if ranked else None, "ranked": ranked[:keep], "tilts": tilts, "counts": counts,
            "first_fail": first_fail, "pairs_top": [{k: p[k] for k in ("theta", "spin", "stop_mm", "body", "stop_target", "h_b_mm",
                                                                       "mouth_down")} for p in pairs[:12]],
            "seconds": round(time.monotonic() - t0, 1)}


def seat_rec(c, homes, np):
    """A seat_bowl_search combo as a JSON record (degrees, mm)."""
    deg = lambda q: None if q is None else [round(float(v), 1) for v in np.degrees(q)]
    return {"home_q_deg": deg(homes[c["b"]]), "tilt_deg": round(c["theta"], 1), "spin_deg": c["spin"], "stop_mm": c["stop_mm"],
            "stop_body": c["body"], "stop_target": c["stop_target"], "perch_mm": c["h_b_mm"], "contact": c["target"],
            "part": c["part"], "com_mouth_mm": c["com_mouth_mm"], "roll": "mouth-down" if c["mouth_down"] else "mouth-up",
            "descent_start_mm": round(c["h_start"] * 1e3), "q_start_deg": deg(c["q_start"]),
            "via_q_deg": deg(c["legs"][0][1]) if len(c["legs"]) == 2 else None,
            "cost_deg": round(float(np.degrees(c["cost"])), 1), "sum_deg": round(float(np.degrees(c["sum"])), 1),
            "travel_deg": deg(sum(np.abs(b - a) for a, b in c["legs"]))}


_WIRES = {}


def wire_parts(checker, comp):
    """{physical wire id: [part index]} of a rack's collision parts: consecutive '/Wire' capsules whose end points meet within
    0.5 mm are one bent wire (kitfree_v1/overlean/overlean.py wire_ids, the numbering of insert_bowl/goal_supports.json)."""
    if comp not in _WIRES:
        import numpy as np
        ids, cur, prev = [], -1, None
        for p_ in checker.parts[comp]:
            L_ = getattr(p_.geometry, "lz", 0.)
            ax = p_.rotation[:, 2]
            ends = [p_.translation + ax * L_ / 2, p_.translation - ax * L_ / 2]
            if prev is None or "/Wire" not in p_.name or min(np.linalg.norm(e - f) for e in ends for f in prev) > 5e-4:
                cur += 1
            ids.append(cur)
            prev = ends
        out = {}
        for j, w in enumerate(ids):
            out.setdefault(w, []).append(j)
        _WIRES[comp] = out
    return _WIRES[comp]


_TINES = {}


def tine_parts(checker, comp):
    """{tine id: [part index]}: a tine = a maximal run of consecutive near-vertical '/Wire' capsules within one physical
    wire (insert_bowl/ib.py _tine_ids, the numbering of goal_supports.json's "tines")."""
    if comp not in _TINES:
        wid = {}
        for w, js in wire_parts(checker, comp).items():
            for j in js:
                wid[j] = w
        out, cur, prev = {}, -1, (None, False)
        for i, p_ in enumerate(checker.parts[comp]):
            vert = "/Wire" in p_.name and abs(p_.rotation[2, 2]) > .5
            if vert and not (prev[0] == wid[i] and prev[1]):
                cur += 1
            if vert:
                out.setdefault(cur, []).append(i)
            prev = (wid[i], vert)
        _TINES[comp] = out
    return _TINES[comp]


def pose_seated(T_final, T_goal, centroid, np, pos_m=.002, rot_deg=5.):
    """The goal-pose test (the fallback of a goal with no bottom support; the drop curve's cup rule): the dish centroid within
    ``pos_m`` and its AXIS (+Z) within ``rot_deg`` of the goal's, so a spin about its own axis does not count (the HOTEC dishes
    are lathes; the round-17 run compared the full rotation, which failed bowl_07 placed at spin 255 by 107 deg).
    (ok, centroid mm, axis deg)"""
    c = np.r_[np.zeros(3) if centroid is None else np.asarray(centroid, dtype=float), 1.]
    d = float(np.linalg.norm((np.asarray(T_final) @ c)[:3] - (np.asarray(T_goal) @ c)[:3]))
    a = float(np.degrees(np.arccos(np.clip(np.asarray(T_goal)[:3, 2] @ np.asarray(T_final)[:3, 2], -1., 1.))))
    return bool(d <= pos_m and a <= rot_deg), round(d * 1e3, 2), round(a, 2)


def rack_part_cls(checker, comp, j):
    """insert_bowl/ib.py's class of a rack part: tine (axis within 60 deg of vertical), floor wire (rack-local z < 30 mm),
    rim wire; other components' wires: '<comp> wire'."""
    p_ = checker.parts[comp][j]
    if "/Wire" not in p_.name:
        return f"{comp}:{p_.name.split('/')[-1]}"
    if comp not in ("LowerRack", "UpperRack"):
        return f"{comp} wire"
    return "tine" if abs(p_.rotation[2, 2]) > .5 else ("floor wire" if p_.translation[2] < .03 else "rim wire")


def bowl_seated_test(arm, kind, hid, T_final, oid, np, others=None, T_goal=None, centroid=None):
    """D-seat seated (Kit-free, the racks where they are now): the settled bowl within BOWL_SEATED_TOL_M (FCL distance) of one
    of its goal's bottom supports (floor wires touching at the goal; kitfree_v1/insert_bowl/goal_supports.json) AND of one of
    the goal's tines; the mouth test is the caller's. Round 14: the goal's wires are physical wires (wire_parts); a goal that
    rests on a dish instead of a floor wire (bowl_05 on bowl_06) takes that dish as its bottom support (its body from
    ``others`` {name: pose dict}, else the arm model's scene dish); a goal with no bottom support (bowl_07, on two tines) needs
    every goal tine within the tolerance."""
    sup = json.loads(BOWL_SUPPORTS_JSON.read_text()).get(oid) or {}
    wires = sup.get("wires") or []
    bottom = [w for w in wires if "floor wire" in w["cls"] and w.get("touch")] or [w for w in wires if "floor wire" in w["cls"]]
    tines = [w for w in wires if "tine" in w["cls"] and w.get("touch")] or [w for w in wires if "tine" in w["cls"]]
    dish_sup = [str(o["body"]).split(":", 1)[1] for o in sup.get("others") or [] if str(o.get("body", "")).startswith("goal:")]
    b = arm.checker._body(kind, _pose_of(T_final), hid)
    comps = arm.checker.components or {}
    tol = BOWL_SEATED_TOL_M * 1e3

    def dist(w):
        """goal_supports.json names a PHYSICAL wire (comp:wire id, kitfree_v1/overlean/overlean.py wire_ids) and the classes
        of its parts touching at the goal; the distance is to that wire's parts of those classes (round 14 fix: round 13 read
        the id as a part index)."""
        comp, idx = w["wire"].split(":")
        if comp not in comps:
            return None
        js = [j for j in wire_parts(arm.checker, comp).get(int(idx), []) if rack_part_cls(arm.checker, comp, j) in
              w["cls"].split("/")]
        tids = [int(str(t).strip("()").split(",")[1]) for t in w.get("tines") or [] if comp in str(t)]
        if "tine" in w["cls"] and tids:              # the goal's own tine(s), not every tine of the physical wire
            tp = tine_parts(arm.checker, comp)
            js = [j for t in tids for j in tp.get(t, [])]
        if not js:
            return None
        return round(max(min(depth_of(arm.fcl, b["manager"], comps[comp]["objects"][j]) for j in js), -1.) * 1e3, 2)
    db = [(dist(w), w["wire"], w["cls"]) for w in bottom]
    if not bottom:
        for dn in dish_sup:                            # the goal rests on a dish: that dish where it is now
            body = (arm.checker._body("bowl", (others or {})[dn], dn) if dn in (others or {}) else
                    (getattr(arm, "dishes", None) or {}).get(dn))
            if body is not None:
                db.append((round(max(depth_of(arm.fcl, b["manager"], body["manager"]), -1.) * 1e3, 2), f"dish {dn}", "dish"))
    dt = [(dist(w), w["wire"], w["cls"]) for w in tines]
    nb = min((x for x in db if x[0] is not None), default=None)
    nt = min((x for x in dt if x[0] is not None), default=None)
    has_bottom = bool(bottom or dish_sup)
    ok_b = nb is not None and nb[0] <= tol
    ok_t = nt is not None and nt[0] <= tol
    pose_ok = None
    if not has_bottom:                                 # two goal tines and nothing under it (bowl_07): the goal-pose test
        T_g = np.asarray(T_goal if T_goal is not None else T_final, dtype=float)
        pose_ok = pose_seated(T_final, T_g, centroid, np) if T_goal is not None else (False, None, None)
    return {"seated_wires": bool(ok_b and ok_t) if has_bottom else pose_ok[0], "bottom_ok": ok_b if has_bottom else None,
            "tine_ok": bool(ok_t), "bottom_nearest": nb, "tine_nearest": nt, "goal_bottom": [w["wire"] for w in bottom] +
            [f"dish {d}" for d in dish_sup if not bottom], "goal_tines": [w["wire"] for w in tines],
            "goal_pose_test": None if pose_ok is None else {"ok": pose_ok[0], "centroid_mm": pose_ok[1], "axis_deg": pose_ok[2]},
            "rule": "the settled bowl within 2 mm (FCL distance) of one of its goal's touching floor wires (else the dish its "
                    "goal rests on) AND of one of its goal's own touching tines (kitfree_v1/insert_bowl/goal_supports.json: "
                    "physical wire and tine ids); a goal with no bottom support (on two tines only): the goal-pose test "
                    "(centroid within 2 mm, axis within 5 deg); success also needs mouth down"}


# plans/2026-11-04-grasp-place-s0.md: --attach none on a samples_v1 load (one dish per process, the pads alone)
CARRY_DROP_M = .005                       # the carried dish this far from the pick's hold (TCP @ M) before the descent: dropped
PICK_TRIES = 2                            # the pick's candidates tried per dish (one re-grasp), then a failed row
SAMPLE_KIND_ARGS = {                      # each kind's move (the study's best rows) and the grasp gate's pick
    "bowl": {"variant": "D", "hold": "foot-out", "carry": "above", "row_variant": "D-ring", "grasp_family": "foot-out",
             "legacy_tolerances": True},
    "cup": {"variant": "D", "hold": "side-mouth", "row_variant": "D-side-mouth", "grasp_family": "side",
            "side_below_top": .045, "side_wrist_side": "mouth"},
    "plate": {"variant": "I", "insert": "seat", "row_variant": "I4", "grasp_family": "rim", "legacy_tolerances": True,
              "rim_depth_m": None}}
SAMPLE_MOVE_ARGS = {                      # a load dish's "move" other than its kind's default (plans/2026-11-04-cups-mouth-down.md)
    "D-side-down": {"variant": "D", "hold": "side-down", "row_variant": "D-side-down", "grasp_family": "side",
                    "side_below_top": .0939, "side_wrist_side": "base"}}   # pads 11 mm from the base (the cup's 104.9 mm minus
                                          # 11): the mirrored h11 pick (the user, 2026-11-04, after the 60 mm hold's Kit-free reject)
SAMPLE_MOVE_ARGS.update({                 # plans/2026-11-04-deep-sites-gate.md (amended): the build's own release, the hand above;
    "D-ring-build": {**SAMPLE_KIND_ARGS["bowl"], "row_variant": "D-ring-build"},        # plans/2026-11-04-plate-pick-presentation
    "D-side-down-build": {**SAMPLE_MOVE_ARGS["D-side-down"], "row_variant": "D-side-down-build"},   # .md (amended): the plate
    "I4-lean30": {**SAMPLE_KIND_ARGS["plate"], "row_variant": "I4-lean30", "grasp_family": "edge"}})   # leaning 30 deg on its
                                          # backstop, the top-down edge pinch
SAMPLE_MOVE_ARGS["I4-wall20"] = {**SAMPLE_KIND_ARGS["plate"], "row_variant": "I4-wall20", "grasp_family": "plate-wall"}
                                          # plans/2026-11-04-plate-wall-pinch.md phase 2: the plate standing in its goal orientation
                                          # on its backstop, the wall-normal pinch 20 mm down the wall, then I4
SAMPLE_MOVE_ARGS["D-ring-side"] = {**SAMPLE_KIND_ARGS["bowl"], "row_variant": "D-ring-side", "grasp_family": "foot-side"}
                                          # plans/2026-11-04-ring-side-hold.md: the bowl picked mouth down by the side ring hold,
                                          # then D-side-down's path (the D-ring descent with the sensor stop, the open, the 20 mm
                                          # back-off along -TCP z, the rise) and the bowls' seated test
# plans/2026-11-04-fifteen-of-fifteen.md: the guarded insertion, the successors of D-ring-build and D-side-down-build (the old
# names keep their behaviour). The dish carried to its descent start in the build's move orientation (a dish with
# build_release; else its goal's), lowered along the rack's up axis (the build's insertion axis) until the flange sensor trips,
# the jaws opened there (no raise, no position bound at the move pose), the hand backed CUP_BACK_M along -TCP z (its approach),
# the rise, the settle. No trip by the bound (DESCENT_BELOW_M under the predicted stop) = the row "no sensor stop" with the jaws
# kept closed: never a position stop, never a free fall. The holds: the axial ring (foot-out), the mouth-down cup side pinch,
# the side ring (foot-side).
SAMPLE_MOVE_ARGS.update({
    "D-ring-insert": {**SAMPLE_KIND_ARGS["bowl"], "row_variant": "D-ring-insert", "guarded_insert": True},
    "D-side-insert": {**SAMPLE_MOVE_ARGS["D-side-down"], "row_variant": "D-side-insert", "guarded_insert": True},
    "D-ring-side-insert": {**SAMPLE_MOVE_ARGS["D-ring-side"], "row_variant": "D-ring-side-insert", "guarded_insert": True}})
INSERT_MOVES = tuple(m for m, a in SAMPLE_MOVE_ARGS.items() if a.get("guarded_insert"))
SIDE_DOWN_MOVES = ("D-side-down", "D-side-insert", "D-side-down-build")   # the mouth-down cup's side pinch: a load row's
                                          # side_below_top_m and hold_gate_run (round 3, cup_05 at 5 mm from the base) apply to these
SAMPLE_MOVE_ARGS["I4-wall20-drop"] = {**SAMPLE_MOVE_ARGS["I4-wall20"], "row_variant": "I4-wall20-drop", "plate_drop": True}
                                          # plans/2026-11-04-fifteen-of-fifteen.md (D2, the user 20:05 EST): I4-wall20, and when no
                                          # descent start's full descent to the seat is clear, the tine-guided release (DROP_RULE)


def sample_move_of(d):
    """A samples_v1 load row's move as the episode runs it and its source: the row's "move" (None: the kind's default), I4-wall20
    run as I4-wall20-drop when the row's release block says {"mode": "drop", ...} (D2, the access order's plate pair)."""
    mv = d.get("move")
    if mv == "I4-wall20" and str((d.get("release") or {}).get("mode") or "").lower() == "drop":
        return "I4-wall20-drop", "the load row's move I4-wall20 with release.mode drop"
    return mv, "the load row's move" if mv else "the kind's default (no move in the load row)"


def grip_family_of(args):
    """The hold whose drive flags R2 carries (flags.GRIP_DRIVE_BY_FAMILY, plans/2026-11-04-fifteen-of-fifteen.md): the run's
    --grasp-family (a samples_v1 load's dish: its move's), "side-down" for the mouth-down cup side pinch (--grasp-family side
    with --hold side-down or --mouth-down)."""
    fam = getattr(args, "grasp_family", None)
    if fam == "side" and (getattr(args, "hold", None) == "side-down" or getattr(args, "mouth_down", False)):
        return "side-down"
    return fam


FOOT_SIDE_GATE_RUN = "partA_bowl_foot_side"   # the side ring hold's Isaac grasp gate in the load dir (its median-pivot held
                                          # trial's closed jaws are the descent check's hand; the F-out hold's before it exists)
FOOT_SIDE_GATE_JSON = (ROOT / "data/results/robot/samples_v1/frigidaire_fdpc4221as__hotec_wheatstraw__plate5-bowl5-cup5_a3__s0__"
                       "ring_side/gate.json")   # --grasp-gate bowl --grasp-family foot-side: the candidate order (gate_pick)


def side_pitch(T_b, T_t):
    """plans/2026-11-04-ring-side-hold.md: the held bowl's tilt from straight down at its pose ``T_b`` about the horizontal axis
    n = (0, 0, -1) x its axis, split over the hand at the TCP pose ``T_t``: (pitch about the closing axis TCP y, roll about the
    approach TCP z, tilt), deg, as tilt * |n . y| and tilt * |n . z| (n lies in the TCP's y-z plane for the side hold)."""
    a = [float(T_b[i][2]) for i in range(3)]
    tilt = math.degrees(math.acos(max(-1., min(1., -a[2]))))
    n = [a[1], -a[0], 0.]                             # (0, 0, -1) x a
    nn = math.hypot(n[0], n[1])
    if nn < 1e-9:
        return 0., 0., tilt
    n = [n[0] / nn, n[1] / nn, 0.]
    dy = sum(n[i] * float(T_t[i][1]) for i in range(3))
    dz = sum(n[i] * float(T_t[i][2]) for i in range(3))
    return tilt * abs(dy), tilt * abs(dz), tilt


def pitch_order(cands, straight=()):
    """The D-ring-side order of passing descent starts (each with "pitch_deg"): the smallest pitch first (0.5 deg bins: the spin
    whose goal tilt is a roll of the hand where one exists), within a bin the ``straight`` indices first (the coordinator's
    straight legs first), then the given order (rule 1's). Returns the indices."""
    s_ = set(straight)
    return sorted(range(len(cands)), key=lambda i: (round(float(cands[i]["pitch_deg"]) * 2.) / 2., i not in s_, i))


def foot_side_order(gs, ranked):
    """The side ring hold's candidates ``gs`` (grasp.foot_side_grasps) in the Kit-free gate's order ``ranked`` ([[yaw_k,
    wrist_side], ...]: the passing ones only); without one, all of them (the pick then sorts by facing)."""
    if not ranked:
        return list(gs)
    by = {(g["yaw_k"], g["wrist_side"]): g for g in gs}
    return [by[(int(k), str(w))] for k, w in ranked if (int(k), str(w)) in by]


def ring_tool_hulls(C, np, load_dir, foot_hold):
    """The closed jaws on the bowl's ring in wrist 3's frame and their source: the side ring hold's Isaac gate in ``load_dir``
    (FOOT_SIDE_GATE_RUN, its median-pivot held trial's last frame) when it has a held trial, else the F-out hold's closed jaws
    (axial_ring_hulls: the load dir's FOOT_OUT_GATE_RUN, else ``foot_hold``'s inhand.json trial): the same contact on the ring's
    outer face."""
    gd = Path(load_dir) / FOOT_SIDE_GATE_RUN if load_dir is not None else None
    if gd is not None and (gd / "trials.jsonl").is_file():
        try:
            run, traj, src = gate_trial(gd)
            return closed_gripper_hulls(C, np, run=run, traj=traj), src
        except RuntimeError:
            pass
    hulls, src = axial_ring_hulls(C, np, load_dir, foot_hold)
    return hulls, f"{src}; the F-out hold's closed jaws on the same ring face (no {FOOT_SIDE_GATE_RUN} held trial in the load dir)"


FOOT_OUT_GATE_RUN = "partA_bowl_foot_out"    # plans/2026-11-04-fifteen-of-fifteen.md: the axial ring hold's Isaac grasp gate in the
                                          # load dir (round 1, at the run's mount and drive): its median-pivot held trial's closed
                                          # jaws are the descent check's hand; --foot-hold's (placing_v1, 8 / 400 / 1) before it exists


def axial_ring_hulls(C, np, load_dir, foot_hold):
    """The axial ring hold's (F-out) closed jaws in wrist 3's frame and their source: the load dir's FOOT_OUT_GATE_RUN (its
    median-pivot held trial's last frame) when it has a held trial, else ``foot_hold``'s (--foot-hold, the inhand.json of
    placing_v1/partA_bowl_foot_out: its median-pivot trial, the last frame)."""
    gd = Path(load_dir) / FOOT_OUT_GATE_RUN if load_dir is not None else None
    if gd is not None and (gd / "trials.jsonl").is_file():
        try:
            run, traj, src = gate_trial(gd)
            return closed_gripper_hulls(C, np, run=run, traj=traj), src
        except RuntimeError:
            pass
    fh = json.loads(Path(foot_hold).read_text())
    run = Path(foot_hold).parent
    rows_ = [json.loads(x) for x in (run / "trials.jsonl").read_text().splitlines() if x.strip()]
    r_ = next(x for x in rows_ if x["trial"] == fh["inhand"]["trial"])
    return (closed_gripper_hulls(C, np, run=run, traj=Path(r_["traj"]).name),
            f"{rel_path(run / 'trials' / Path(r_['traj']).name)} (--foot-hold {rel_path(foot_hold)}: trial {fh['inhand']['trial']}, "
            f"the median-pivot hold, last frame)")
PLATE_WALL_GATE_RUN = "partA_plate_wall20"   # the wall pinch's Isaac grasp gate in the load dir (its median-pivot held trial's
                                          # closed jaws are the I4 descent check's hand; PLATE_GATE before it exists)
BOWL_CARRY_TILT_MAX_DEG = 20.             # plans/2026-11-04-s0-rerun.md: the bowl carry rule (ring_drop's carry_ok) also bounds the
                                          # held bowl's tilt from straight down on every carry sample
RING_RETRY_RRT_S = 120.                   # plans/2026-11-04-s0-rerun.md: a bowl with no carry path at --transit-rrt-s gets one
                                          # retry at this RRT-Connect budget (compute only), then its no-reach row


def bowl_tilt_cap_z(T_goal):
    """The tilt cap on a held bowl's axis z (mouth down: z < 0): its tilt from straight down at most BOWL_CARRY_TILT_MAX_DEG, or
    its own tilt at the descent start (``T_goal``, the bowl at its reference pose) when that is larger, so the descent start
    itself stays reachable (every s0 bowl goal is 27 to 61 deg from straight down)."""
    return max(-math.cos(math.radians(BOWL_CARRY_TILT_MAX_DEG)), float(T_goal[2][2])) + 1e-4
CARRY_CLEAR_M = .005                      # plans/2026-11-04-bowl-carry-clearance.md (amended, rule 1, --attach none): the held dish
                                          # at least this far (FCL distance) from the scene on every planned carry sample and at the
                                          # descent start (collide.ArmWorld.held_clear_m; the checker's pair rule passed 1 mm overlap)
DESCENT_STARTS_M = (.050, .075, .100, .125, .150)   # rule 1: the descent start candidates above the seat (D-ring, D-side-down,
                                          # the build release); the one with the largest held-dish clearance among those that pass
CLEAR_CUTOFF_M = .10                      # the clearance ranking's cutoff (nothing within 100 mm ranks as 100 mm)
BUILD_UP_M = .020                         # the build release: after the sensed stop the dish raised this much, then let go (the
                                          # build released 20 mm above its lowest clear pose; witness release_lift - lowest_lift)
PLATE_EDGE_GATE_RUN = "partA_plate_edge_lean30"   # the edge pinch's Isaac grasp gate in the plates load dir (its median-pivot
                                          # held trial's closed jaws are the I4 descent check's hand; PLATE_GATE before it exists)


def box_body(fcl, name, center, size, Rm, np):
    """A static box (center, size, axes Rm = its local x, y, z as columns) in the checker's body format."""
    Rm, c = np.asarray(Rm, dtype=float), np.asarray(center, dtype=float)
    o = fcl.CollisionObject(fcl.Box(*[float(s) for s in size]), fcl.Transform(Rm, c))
    mg = fcl.DynamicAABBTreeCollisionManager()
    mg.registerObjects([o])
    mg.setup()
    corners = np.array([[x, y, z] for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)]) * np.asarray(size, dtype=float) / 2 @ Rm.T + c
    return {"id": name, "kind": "static", "objects": [o], "manager": mg, "bounds": (corners.min(0), corners.max(0))}


def presentation_pose(pres, d=(0., 0.)):
    """The plate's spawn pose of a presentation (frigidaire_robot_reach.py --sample-gate's presentation.json): its pose leaning on
    the backstop, shifted by the part of the +-5 mm draw ``d`` (x, y) along the backstop's long axis (the plate keeps its 1 mm
    gap and its 2 mm above the counter). (pose dict, the applied shift in mm [x, y])"""
    p, t = pres["plate_pose"], pres["backstop"]["axes"]
    tx, ty = float(t[0][1]), float(t[1][1])           # the box's local y (its 160 mm width) in world
    n_ = math.hypot(tx, ty)
    s = (d[0] * tx + d[1] * ty) / n_ ** 2 if n_ > 1e-9 else 0.
    sh = (s * tx, s * ty)
    return ({"position_m": [p["position_m"][0] + sh[0], p["position_m"][1] + sh[1], p["position_m"][2]],
             "quaternion_xyzw": list(p["quaternion_xyzw"])}, [round(sh[0] * 1e3, 2), round(sh[1] * 1e3, 2)])


def gate_trial(gate_dir, fallback=None):
    """(run dir, traj name, source) of the median-pivot held trial of an Isaac grasp gate run (``gate_dir`` with trials.jsonl,
    the trajectories under the trial's own run); ``fallback`` = (run, traj) when the gate has no held trial yet."""
    f = None if gate_dir is None else Path(gate_dir) / "trials.jsonl"
    if f is not None and f.is_file():
        held = sorted([r for r in (json.loads(x) for x in f.read_text().splitlines() if x.strip()) if r.get("g3") and r.get("traj")],
                      key=lambda r: r["hold"]["settle_rotation_deg"])
        if held:
            tp = Path(held[len(held) // 2]["traj"])
            return ROOT / tp.parent.parent, tp.name, f"{tp} (median-pivot held trial, last frame)"
    if fallback is None:
        raise RuntimeError(f"no grasp gate with a held trial at {gate_dir}")
    return Path(fallback[0]), fallback[1], f"{Path(fallback[0]).name}/{fallback[1]} (fallback: no held trial at {gate_dir})"
CUP_DOWN_ABOVES_M = (.050, .075, .100, .125, .150)   # D-side-down: the descent start this far above the goal (the lowest free
                                          # per spin), the D-ring rule's 50 mm and DESCENT_GRID_M steps to 150 mm
CUP_BACK_M = .020                         # D-side-down: after the open and the wait the hand backs 20 mm along -TCP z (its
                                          # horizontal approach), then the D-ring rise
CUP_DOWN_GATE_RUN = "partA_cup_side_down11"   # the mouth-down cup's Isaac grasp gate in the load dir (its median trial's closed
                                          # jaws are the descent check's and the stop prediction's tool; the split parts merged there)
CUP_DOWN_GATE_DIR = (ROOT / "data/results/robot/samples_v1/frigidaire_fdpc4221as__hotec_wheatstraw__plate5-bowl5-cup5_a3__s0__"
                     "cups_down" / CUP_DOWN_GATE_RUN)   # the cups round's merged gate (held 10/10): a load dir without its own
CUP_H45_GATE = ROOT / "data/results/robot/placing_v1/partA_cup_side_h45"   # the closed jaws before that gate exists (jaws
                                          # 0.212 rad on the 64.6 mm wall: the measured angle nearest the 57.6 mm wall 11 mm up)


class CarryDropped(RuntimeError):
    """--attach none: raised from the tick hook when the carried dish leaves the hand (over CARRY_DROP_M)."""


GOAL_TOL = (.015, .020, 15.)              # S5 (G4, the plan): lateral m, |dz| m, tilt deg, every kind
KEEP_TOL = (.015, 15.)                    # S6: an earlier dish within 15 mm / 15 deg of its goal (in its rack's frame)
RAISES_M = (.10, .15, .20, .25, .30, .35) # S1: the dish at its goal orientation this far above its goal
SPAWN_SETTLE_TICKS = 60                   # the earlier dishes settle at their goal poses before the robot moves
RRT_S = 10.                               # S3 / R1: RRT-Connect budget per pre-place IK solution
SET_DOWN_TILTS = (0., 30., 45., 60.)      # R1: the dish mouth down, tilted this much from straight down
SET_DOWN_DROP_M = .020                    # R1: released with its lowest point this far above the counter
REPICK_TILTS_DEG = (0., 15., -15., 25., -25., 35., -35.)   # R2: the grasp leaned about the TCP x (tangent) axis when the
                                          # straight-down foot clamp is blocked (wrist_1 / forearm over a wide mouth-down dish)
STEPS = ("R1", "R2", "S1", "S2", "S3", "S4", "S5", "S6")


# ---------------------------------------------------------------- plans/2026-10-29-sampler-release.md: rows I5 / D-sampler (--insert sampler)
SAMPLER_TABLE_JSON = ROOT / "data/results/robot/kitfree_v1/sampler_release/final_pose_table.json"
SAMPLER_ABOVE_M = .050            # the carry ends 50 mm above the sampler's release pose (the descent start)
SAMPLER_UP_M = .020               # after the sensed stop the TCP rises 20 mm (the dish still welded), then the jaws open
SAMPLER_RISE_M = .10              # then the open hand goes straight up 0.10 m
SAMPLER_STOP_COARSE_M, SAMPLER_STOP_FINE_M = .002, .00025   # the Kit-free stop: 2 mm grid, bisected to 0.25 mm
SAMPLER_STOP_TO_M = -.100         # ... searched down to 100 mm under the release pose
PLATE_WALL_DRDZ = .625            # hb_lib.mirror_tcp: the plate rim wall's slope (the mirror plane's normal)


def sampler_mirror_tcp(T_plate, d_mm, G, np, mirror=True):
    """The final-pose check's plate top-edge hold at the plate pose T_plate (kitfree_v1/sampler_release/sr_hand.py, after
    insert_plate/hold_base/hb_lib): 'mouth d<d>' = plate_holds' mouth-face hold backed off along its approach so the rim is
    d mm into the pads; 'mirror d<d>' = that TCP reflected through the rim wall's mid plane (normal n - 0.625 a, through the
    wall's mid point at the rim point highest at this pose), its y axis flipped: the hand body on the base side. T_tcp."""
    ez = np.array([0., 0., 1.])
    rim_r, wall, height, _ = G.KIND_RIM["plate"]
    T_plate = np.asarray(T_plate, dtype=float)
    _, Tm = plate_holds(T_plate, G, np)["mouth"]
    Tm = np.array(Tm, dtype=float)
    Tm[:3, 3] -= (19 - d_mm) / 1e3 * Tm[:3, 2]
    if not mirror:
        return Tm
    a = T_plate[:3, 2] / np.linalg.norm(T_plate[:3, 2])
    n = ez - a[2] * a
    n /= np.linalg.norm(n)
    w0 = T_plate[:3, 3] + height * a + (rim_r - wall / 2) * n
    nu = n - PLATE_WALL_DRDZ * a
    nu /= np.linalg.norm(nu)
    T = np.eye(4)
    T[:3, :3] = (np.eye(3) - 2 * np.outer(nu, nu)) @ Tm[:3, :3] @ np.diag([1., -1., 1.])
    p_ = Tm[:3, 3]
    T[:3, 3] = p_ - 2 * ((p_ - w0) @ nu) * nu
    return T


def sampler_bowl_tcp(T_bowl, M_meas, spin_deg, np):
    """The final-pose check's bowl lip pinch at the bowl pose T_bowl (sr_hand.py): the measured hold M_meas (TCP -> bowl)
    turned about the bowl's own axis so that the pinched rim point (the TCP origin's radial direction in the dish frame) is
    the rim point highest at this pose, then spin_deg more. T_tcp (the bowl stays at T_bowl; axisymmetric)."""
    ez = np.array([0., 0., 1.])
    T_bowl = np.asarray(T_bowl, dtype=float)
    Mi = np.linalg.inv(np.asarray(M_meas, dtype=float))
    r_p = np.array([Mi[0, 3], Mi[1, 3], 0.])
    r_p /= np.linalg.norm(r_p)
    a = T_bowl[:3, 2]
    n = ez - (ez @ a) * a
    n /= np.linalg.norm(n)
    n_d = T_bowl[:3, :3].T @ n
    phi = math.atan2(np.cross(r_p, n_d)[2], r_p @ n_d) + math.radians(spin_deg)
    Rz = np.eye(4)
    Rz[:2, :2] = [[math.cos(phi), -math.sin(phi)], [math.sin(phi), math.cos(phi)]]
    return T_bowl @ Rz @ Mi


def sampler_rows(load_name):
    """The final-pose table's rows of one load, by dish."""
    t = json.loads(SAMPLER_TABLE_JSON.read_text())
    return {r["dish"]: r for r in t["rows"] if r["load"] == load_name}


def dish_clear(arm, kind, T, hid, targets, np, cutoff=.01):
    """(FCL distance, target) of the dish alone at T to the nearest of ``targets`` (<= 0 touching; inf beyond cutoff)."""
    b = arm.checker._body(kind, _pose_of(T), hid)
    lo, hi = b["bounds"]
    best = (np.inf, None)
    for tn, tb in targets:
        if _box_gap(lo, hi, tb, np) > cutoff:
            continue
        d = _mgr_dist(arm.fcl, tb["manager"], b["manager"])
        if d < best[0]:
            best = (d, tn)
    return best


def sampler_stop(arm, kind, hid, T_rel, targets, np, h_to=SAMPLER_STOP_TO_M):
    """The dish alone lowered straight down (world -z) from the sampler's release pose T_rel: the first height (m, relative to
    T_rel) whose FCL distance to ``targets`` is <= 0, on a SAMPLER_STOP_COARSE_M grid bisected to SAMPLER_STOP_FINE_M.
    {"h_m", "body", "touch_at_release"} or {"h_m": None} when nothing is touched down to h_to."""
    def at(h):
        T = np.array(T_rel, dtype=float, copy=True)
        T[2, 3] += h
        return T
    d0 = dish_clear(arm, kind, at(0.), hid, targets, np)
    if d0[0] <= 0:
        return {"h_m": 0., "body": d0[1], "touch_at_release": True}
    hi_, h = 0., -SAMPLER_STOP_COARSE_M
    while h >= h_to - 1e-9:
        d = dish_clear(arm, kind, at(h), hid, targets, np)
        if d[0] <= 0:
            lo_, body = h, d[1]
            while hi_ - lo_ > SAMPLER_STOP_FINE_M + 1e-9:
                mid = (hi_ + lo_) / 2
                dm = dish_clear(arm, kind, at(mid), hid, targets, np)
                if dm[0] <= 0:
                    lo_, body = mid, dm[1]
                else:
                    hi_ = mid
            return {"h_m": round(lo_, 5), "body": body, "touch_at_release": False}
        hi_ = h
        h = round(h - SAMPLER_STOP_COARSE_M, 6)
    return {"h_m": None, "body": None, "touch_at_release": False}


def park_pose(i):
    """Dishes not in play rest on the ground far from the robot (the backend's own spawn row is at x >= 3 m)."""
    return {"position_m": [3. + .4 * i, -1., .002], "quaternion_xyzw": [0., 0., 0., 1.]}


def goal_instance(load):
    """Part B scene: the benchmark's extended baseline (= the goal records' rack frames), the load's dishes parked."""
    base = json.loads((ROOT / "data/results/benchmark/frigidaire_hotec/baseline/result.json").read_text())
    snap = base.get("snapshot", base)
    objs = [{"object_id": d["name"], "kind": d["kind"], "start": "Counter", "rack": d["rack"], "pose_world": park_pose(i)}
            for i, d in enumerate(load["dishes"])]
    poses = {**{n: v for n, v in snap["poses"].items() if not n.startswith(("plate_", "bowl_", "cup_"))},
             **{o["object_id"]: o["pose_world"] for o in objs}}
    goal = {d["name"]: {"rack": d["rack"], "slot": d["slot"], "settled_pose_world":
                        {"position_m": d["goal_pose"]["pos"], "quaternion_xyzw": d["goal_pose"]["quat_xyzw"]}}
            for d in load["dishes"]}
    return {"instance_id": f"placing_{load['load']}", "tier": "easy", "baseline": snap, "initial_snapshot": {"poses": poses},
            "objects": objs, "counter": {"start_count": 0, "cap": 99}, "support": [], "goal": {"objects": goal, "order": []}}


def place_goal_trials(args, report, flags, ns):
    """Part B of plans/2026-10-26-placing-goal.md. Per dish of the load in its order: the earlier dishes at their goal
    poses (60 ticks), this dish fixed to wrist_3 by a PhysX fixed joint at the in-hand pose (jaws closed on it), the arm at
    home (its first IK branch free with the dish held). S1 reorient: a collision-free IK with the dish at its goal
    orientation 0.10-0.35 m above the goal; S2 G1: collision-free IK at the pre-place (0.10 m straight above the release)
    and the release (the goal + release_above_m); S3 G2: a joint path (straight or RRT-Connect) to the pre-place and the
    vertical insertion line; S1-S3 on the arm model, no physics. S4 carry + insert in Isaac, the joint removed and the
    jaws opened; S5 150-tick settle vs the goal (15 mm lateral / 20 mm height / 15 deg); S6 the earlier dishes within
    15 mm / 15 deg of their goals in their racks' frames. First failure wins. --regrasp (variant R) adds R1 (flip, set
    down mouth down on the counter spot) and R2 (re-pick from above, re-attach) before S1. --settle-only: the subset
    settle check."""
    import numpy as np
    import omni.usd
    from pxr import Gf, Sdf, UsdPhysics
    from scipy.spatial.transform import Rotation
    from dishsim_frigidaire import planner as P
    from dishsim_frigidaire.initial_state_runtime import IsaacInitialStateBackend
    from dishsim_frigidaire.robot.rig import IK_JUMP_MAX_RAD
    sys.path.insert(0, str(ROOT / "code/execution/frigidaire"))
    import frigidaire_robot_reach as RC
    backend, rig, harness, tlog, hook, robot = ns.backend, ns.rig, ns.harness, ns.tlog, ns.hook, ns.robot
    arm, G, C, K, B, T_of, T_now = ns.arm, ns.G, ns.C, ns.K, ns.B, ns.T_of, ns.T_now
    goal_qs, transit_to, sync_world, kinds = ns.goal_qs, ns.transit_to, ns.sync_world, ns.kinds
    load = json.loads(args.place_goal.read_text())
    L, out = load["load"], args.place_goal.parent
    names = [d["name"] for d in load["dishes"]]
    meta = {d["name"]: d for d in load["dishes"]}
    spot_xy, spot_src = counter_spot(flags.mount, load, getattr(args, "counter_y", None))   # plans/2026-11-04-fifteen-of-
    pres_f_ = getattr(args, "presentation", None)                                         # fifteen.md: the run's counter spot
    spot_rec = {"spot_xy": [float(v) for v in spot_xy], "spot_source": spot_src, "mount_file": str(flags.mount),
                "presentation_file": None if pres_f_ is None else rel_path(pres_f_),
                "basket": bool(getattr(flags, "basket", True)),   # plans/2026-11-04-fifteen-of-fifteen.md: every row records the
                "basket_source": getattr(args, "basket_source", None) or "flags.basket"}   # basket and its source
    report["spot"] = {**(report.get("spot") or {}), "xy": spot_rec["spot_xy"], "source": spot_src, "mount_file": str(flags.mount),
                      "presentation_file": spot_rec["presentation_file"]}
    goal = {n: T_of(meta[n]["goal_pose"]["pos"], meta[n]["goal_pose"]["quat_xyzw"]) for n in names}
    F_goal = {r: T_of(f["position_m"], f["quaternion_xyzw"]) for r, f in load["frames"].items()}
    stage = omni.usd.get_context().get_stage()
    inv = np.linalg.inv
    pose = P.pose_dict
    harness.abort_on = False
    t_run = time.monotonic()
    goals_src = None
    if args.goals is not None:                         # plans/2026-10-29-ring-place.md: the goals refreshed on the current bowl
        t_w = time.monotonic()                         # the window launches this run beside the refresh: wait for its goals
        while not (args.goals.is_file() and (args.bowl_supports is None or BOWL_SUPPORTS_JSON.is_file())):   # and supports
            if time.monotonic() - t_w > GOALS_WAIT_S:
                raise RuntimeError(f"--goals {args.goals} / --bowl-supports {BOWL_SUPPORTS_JSON}: not there after "
                                   f"{GOALS_WAIT_S:.0f} s")
            time.sleep(2.)
        print(f"[INFO] goals and supports there after {time.monotonic() - t_w:.0f} s wait", flush=True)
        report["goals_wait_s"] = round(time.monotonic() - t_w, 1)
        gv = json.loads(args.goals.read_text())
        for n_, g_ in gv["bowls"].items():
            if n_ in goal and g_.get("goal_pose"):
                goal[n_] = T_of(g_["goal_pose"]["pos"], g_["goal_pose"]["quat_xyzw"])
                ns.goal_world[n_] = goal[n_]
        goals_src = {"file": str(args.goals), "bowls": sorted(gv["bowls"]), "rule": gv.get("rule")}
        report["goals"] = goals_src
    rel_h = {}                                         # plans/2026-10-30-bowl-drop100.md: the release height per bowl (m)
    if args.release_height_mm is not None:
        parts_ = [x.strip() for x in str(args.release_height_mm).split(",") if x.strip()]
        over_ = dict(x.split(":", 1) for x in parts_[1:])
        if not parts_ or ":" in parts_[0] or ":" in "".join(over_.values()) or any(k_ not in names for k_ in over_):
            raise RuntimeError(f"--release-height-mm {args.release_height_mm!r}: the default (mm) first, then bowl:mm of this load")
        rel_h = {n_: float(over_.get(n_, parts_[0])) / 1e3 for n_ in names}
        report["release_heights"] = {"spec": args.release_height_mm, "h_mm": {n_: round(v_ * 1e3, 1) for n_, v_ in rel_h.items()},
                                     "rule": "the bowl centroid this far above its refreshed goal's (along the rack's up axis) "
                                             "when the jaws open; the descent starts RING_ABOVE_M above that"}
        print(f"[INFO] release heights (mm): {report['release_heights']['h_mm']}", flush=True)
    if (args.variant == "D" and args.hold in ("rim-lip", "foot-out")) or args.variant == "I":   # plans/2026-10-27-bowls-lip-plates-fix.md:
        arm.held_margin = C.HELD_ARM_MARGIN_M     # the welded dish may come 2 mm from the arm's links (15 mm to the scene)

    def rot_deg(A, Bm):
        return float(np.degrees(np.arccos(np.clip((np.trace(A[:3, :3].T @ Bm[:3, :3]) - 1) / 2, -1., 1.))))

    def finish():
        tlog.close()
        backend.trace.close()
        backend.trace = None

    def set_racks(rack):
        """None = both racks extended (the goal records' frames); a lower-rack dish has the upper rack pushed in, an
        upper-rack dish the lower rack (the limits rounds' rule)."""
        want = {r: ns.EXTENSION[r] for r in ("LowerRack", "UpperRack")}
        if rack is not None:
            want["UpperRack" if rack == "LowerRack" else "LowerRack"] = 0.
        js = backend.joints()
        move = {ns.RACK_JOINT[r]: v for r, v in want.items() if abs(js[ns.RACK_JOINT[r]] - v) > .005}
        if move:
            backend.ramp(move, f"place_goal_racks_{rack}")

    if args.settle_only:                               # the subset settle check
        tlog.set(phase="subset settle")
        harness.armed = False
        set_racks(None)
        for n in names:
            backend.set_rigid_pose(backend.objects[n], pose(goal[n]))
        rig.step(K.SETTLE_TICKS)
        now = backend.poses()
        per = {}
        for n in names:
            T = T_of(now[n]["position_m"], now[n]["quaternion_xyzw"])
            per[n] = {"drift_mm": round(float(np.linalg.norm(T[:3, 3] - goal[n][:3, 3])) * 1e3, 2),
                      "drift_deg": round(rot_deg(T, goal[n]), 2)}
        mx_mm, mx_deg = max(v["drift_mm"] for v in per.values()), max(v["drift_deg"] for v in per.values())
        res = {"load": L, "ok": bool(mx_mm <= 15. and mx_deg <= 15.), "max_drift_mm": mx_mm, "max_drift_deg": mx_deg,
               "per_dish": per, "ticks": K.SETTLE_TICKS, "racks": "both extended (the goal record's frames)",
               "rule": "every dish within 15 mm / 15 deg of its goal pose after 150 ticks, all teleported at once",
               "final_poses": {n: now[n] for n in names}}
        (out / "subset_settle.json").write_text(json.dumps(res, indent=1) + "\n")
        report.update(result="PASS", subset_settle={k: res[k] for k in ("ok", "max_drift_mm", "max_drift_deg")})
        print(f"[RESULT] PASS subset settle {L}: ok {res['ok']} max {mx_mm} mm / {mx_deg} deg {per}", flush=True)
        return finish()

    (args.trial_dir / "robot_meshes.json").write_text(json.dumps(robot_meshes(robot)) + "\n")
    traj = {"on": False, "rows": [], "oid": None, "others": []}

    post = {}                                          # plans/2026-10-27-posture-fix.md: joint range and carry travel per dish

    def post_reset():
        post.update(max=np.zeros(6), travel=np.zeros(6), last=None, carry=False, up=True, down_ticks=0, ticks=0, start=None,
                    cmax=np.zeros(6))

    jaw_watch = {"on": False, "oid": None, "hits": []}   # plans/2026-10-29-perch-roll.md: gripper-bowl contacts, every tick
    carry_watch = {"on": False}                        # plans/2026-11-04-grasp-place-s0.md: the pads-alone carry, every tick

    def traj_tick(frames):
        harness.on_tick(frames)
        if carry_watch.get("on"):                      # the dish's offset from TCP @ M (the pick's hold), every tick
            d_ = frames[carry_watch["oid"]]
            off_ = float(np.linalg.norm(T_of(d_["position_m"], d_["quaternion_xyzw"])[:3, 3]
                                        - (rig.tcp() @ carry_watch["M"])[:3, 3]))
            ph_ = carry_watch["phase"]
            carry_watch["max_mm"][ph_] = max(carry_watch["max_mm"].get(ph_, 0.), round(off_ * 1e3, 2))
            carry_watch["last_mm"] = round(off_ * 1e3, 2)
            if ph_ == "carry":                         # plans/2026-11-04-bowl-carry-clearance.md: per carry leg, the drift and
                lg_ = ns.leg_now["leg"] or str(tlog.ctx.get("phase"))      # the held dish's scene contacts (not the hand's)
                L_ = carry_watch.setdefault("legs", {}).setdefault(lg_, {"max_mm": 0., "ticks": 0, "first_tick": int(backend.tick_index),
                                                                         "contacts": {}})
                L_["max_mm"] = max(L_["max_mm"], round(off_ * 1e3, 2))
                T_d_, T_h_ = T_of(d_["position_m"], d_["quaternion_xyzw"]), rig.tcp() @ carry_watch["M"]   # plans/2026-11-04-s0-
                L_["max_tilt_deg"] = max(L_.get("max_tilt_deg", 0.), round(math.degrees(math.acos(max(-1., min(1., -float(T_d_[2, 2]))))), 2))
                L_["max_inhand_rot_deg"] = max(L_.get("max_inhand_rot_deg", 0.), round(rot_deg(T_h_, T_d_), 2))   # rerun.md: the dish's
                L_["last_tilt_deg"] = round(math.degrees(math.acos(max(-1., min(1., -float(T_d_[2, 2]))))), 2)    # tilt from straight
                L_["last_inhand_rot_deg"] = round(rot_deg(T_h_, T_d_), 2)  # down and its rotation from TCP @ M, per leg
                L_["ticks"] += 1
                for (a_, b_), r_ in (getattr(harness.contacts, "step_rows", None) or {}).items():
                    if carry_watch["oid"] not in (a_, b_):
                        continue
                    oc_ = r_["cats"][1] if a_ == carry_watch["oid"] else r_["cats"][0]
                    if oc_ in ("pad", "finger", "palm"):
                        continue
                    c_ = L_["contacts"].setdefault(f"{a_}|{b_}", {"category": oc_, "first_tick": int(backend.tick_index), "ticks": 0,
                                                                  "peak_N": 0.})
                    c_["ticks"] += 1
                    c_["peak_N"] = max(c_["peak_N"], round(float(r_["N"]), 3))
                    c_["last_tick"] = int(backend.tick_index)
            if ph_ == "carry" and off_ > CARRY_DROP_M:
                carry_watch.update(on=False, dropped={"tick": int(backend.tick_index), "drift_mm": round(off_ * 1e3, 2),
                                                      "phase": tlog.ctx.get("phase"), "dish": d_,
                                                      "tcp": [round(float(v), 4) for v in rig.tcp()[:3, 3]]})
                raise CarryDropped(f"the {kinds[carry_watch['oid']]} left the hand: {off_ * 1e3:.1f} mm from the pick's hold "
                                   f"during '{tlog.ctx.get('phase')}' (limit {CARRY_DROP_M * 1e3:.0f} mm)")
        if jaw_watch["on"]:
            for (a_, b_), r_ in (getattr(harness.contacts, "step_rows", None) or {}).items():
                if jaw_watch["oid"] in (a_, b_) and any(c_ in SENSOR_TOOL_CATS for c_ in r_["cats"]):
                    jaw_watch["hits"].append([backend.tick_index, f"{a_}|{b_}", round(float(r_["N"]), 3)])
        if traj["on"] and backend.tick_index % TRAJ_EVERY == 0:
            traj_row(frames)
        if post:
            q = robot.data.joint_pos.torch[0, rig.arm].cpu().numpy().astype(float)
            if post["carry"] and post["last"] is not None:
                post["travel"] += np.abs(q - post["last"])
            if post["carry"]:
                post["cmax"] = np.maximum(post["cmax"], np.abs(q))
            post["last"], post["ticks"] = q, post["ticks"] + 1
            post["max"] = np.maximum(post["max"], np.abs(q))
            lift, elbow = (q[1:3] + np.pi) % (2 * np.pi) - np.pi
            if not (-np.pi < lift < 0. < elbow < np.pi):
                post["up"], post["down_ticks"] = False, post["down_ticks"] + 1

    def posture():
        deg = lambda v: [round(float(x), 1) for x in np.degrees(v)]
        jn = ("shoulder_pan", "shoulder_lift", "elbow", "wrist_1", "wrist_2", "wrist_3")
        return {"max_abs_deg": deg(post["max"]), "past_180": [j for j, v in zip(jn, np.degrees(post["max"])) if v > 180.],
                "elbow_up": bool(post["up"]), "elbow_down_ticks": int(post["down_ticks"]),
                "carry_travel_deg": deg(post["travel"]), "carry_travel_sum_deg": round(float(np.degrees(post["travel"].sum())), 1),
                "carry_start_deg": None if post["start"] is None else deg(post["start"]),
                "carry_max_abs_deg": deg(post["cmax"]),
                "end_deg": None if post["last"] is None else deg(post["last"]), "ticks": int(post["ticks"]),
                "joints": list(jn), "rule": "max |q| per joint over the dish's whole trial (reset, carry, release, retreat; "
                "teleports included: the reset's q_home teleport counts), carry max = max |q| from the drop start to the "
                "release only, travel = sum |dq| per joint over the carry (drop start to release, every try), "
                "elbow up = shoulder lift in (-180, 0) and elbow in (0, 180) at every tick"}

    def traj_row(frames):
        d, lo, up = frames[traj["oid"]], frames["LowerRack"], frames["UpperRack"]
        oth = np.array([np.r_[frames[n]["position_m"], frames[n]["quaternion_xyzw"]] for n in traj["others"]]).reshape(-1, 7)
        # link quats stay WXYZ in trajectory files (their readers and every 4.5-era file expect it)
        traj["rows"].append((backend.tick_index * backend.dt, robot.data.body_link_pos_w.torch[0].cpu().numpy().copy(),
                             robot.data.body_link_quat_w.torch[0][:, [3, 0, 1, 2]].cpu().numpy().copy(),
                             np.r_[d["position_m"], d["quaternion_xyzw"]],
                             np.r_[lo["position_m"], lo["quaternion_xyzw"]], np.r_[up["position_m"], up["quaternion_xyzw"]],
                             float(rig.theta), oth))
    hook["tick"] = traj_tick

    def save_traj(tag):
        r = traj["rows"]
        traj["on"] = False
        if not r:
            return None
        path = args.trial_dir / f"traj_{tag}.npz"
        np.savez_compressed(path, t=np.array([x[0] for x in r]), body_pos=np.stack([x[1] for x in r]),
                            body_quat_wxyz=np.stack([x[2] for x in r]), dish=np.stack([x[3] for x in r]),
                            lower=np.stack([x[4] for x in r]), upper=np.stack([x[5] for x in r]),
                            jaw=np.array([x[6] for x in r]), others=np.stack([x[7] for x in r]),
                            kind=np.array(kinds[traj["oid"]]), others_names=np.array(list(traj["others"]), dtype=str),
                            others_kinds=np.array([kinds[n] for n in traj["others"]], dtype=str),
                            mount=np.asarray(report["mount"][:3], dtype=float), counter_front_y=np.array(args.counter_front_y),
                            **({"fixtures": np.array([list(b_["center_m"]) + list(b_["size_m"])   # every box (the plank
                                                      + list(np.asarray(b_["axes"], dtype=float).ravel())   # and its lip)
                                                      for b_ in presentation_boxes(ns.pres)])}
                               if ns.pres else {}))
        traj["rows"] = []
        return str(path.relative_to(ROOT))

    # ---------------------------------------------------------------- the attachment
    att, att_mode = {}, {"pin": args.attach == "pin", "none": args.attach == "none",
                         "why": f"--attach {args.attach}" if args.attach in ("pin", "none") else None}

    def attach(oid, T_d=None):
        """A PhysX fixed joint wrist_3 -> dish at their current relative pose, excluded from the articulation (so the
        robot's articulation is unchanged); the fallback (the joint did not hold on a first try): the dish written to that
        pose after every rig step."""
        T_d = T_now(oid) if T_d is None else np.asarray(T_d)
        X = rig.T_w3_tcp @ inv(rig.tcp()) @ T_d
        att.update(oid=oid, X=X, M=inv(rig.tcp()) @ T_d)
        if att_mode["none"]:                           # plans/2026-11-04-grasp-place-s0.md: the pads alone, nothing authored
            return
        if att_mode["pin"]:
            obj, orig = backend.objects[oid], rig.on_step

            def on_step():
                orig()
                IsaacInitialStateBackend.set_rigid_pose(backend, obj, pose(rig.tcp() @ inv(rig.T_w3_tcp) @ X))
            rig.on_step, att["orig"] = on_step, orig
            return
        q = Rotation.from_matrix(X[:3, :3]).as_quat()       # xyzw
        path = f"/World/PlaceGoalAttach/{oid}_{backend.tick_index}"
        j = UsdPhysics.FixedJoint.Define(stage, path)
        j.CreateBody0Rel().SetTargets([Sdf.Path("/World/Robot/wrist_3_link")])
        j.CreateBody1Rel().SetTargets([Sdf.Path(f"/World/InitialStateDishes/{oid}")])
        j.CreateLocalPos0Attr().Set(Gf.Vec3f(*[float(v) for v in X[:3, 3]]))
        j.CreateLocalRot0Attr().Set(Gf.Quatf(float(q[3]), float(q[0]), float(q[1]), float(q[2])))
        j.CreateLocalPos1Attr().Set(Gf.Vec3f(0., 0., 0.))
        j.CreateLocalRot1Attr().Set(Gf.Quatf(1., 0., 0., 0.))
        j.CreateExcludeFromArticulationAttr().Set(True)
        att["path"] = path

    def detach():
        if carry_watch.get("on"):                      # the release (or a reset): the carry watch ends here
            carry_watch.update(on=False, at_release_mm=carry_watch.get("last_mm"))
        if att.get("path"):
            stage.RemovePrim(att["path"])
        if att.get("orig") is not None:
            rig.on_step = att["orig"]
        att.clear()

    def attach_drift(oid, ticks=12):
        X = att["X"]
        rig.step(ticks)
        return float(np.linalg.norm((inv(rig.tcp() @ inv(rig.T_w3_tcp)) @ T_now(oid))[:3, 3] - X[:3, 3]))

    Ms = {}
    carry_state = {}                                   # --carry nearest: reset()'s search result for drop()

    def in_hand(kind):
        if kind not in Ms:
            if kind == "plate" and variant != "D":     # no plate grasp exists: the rim-pinch frame with zero pivot
                T_o = RC.counter_obj()
                g = G.bowl_grasps(T_o, RC.N_YAW, "plate")[0]
                Ms[kind] = (inv(g["T_tcp"]) @ T_o, {"source": "idealized: the plate rim-pinch frame (grasp.py KIND_RIM), "
                                                              "zero pivot", "pivot_deg": 0.})
            elif side_base and kind == "cup":          # the base-side gate's median hold, else the mouth hold rolled
                run = RC.PLACING / (args.side_base_run or SIDE_BASE_RUN)
                held_ = [r for r in (json.loads(x) for x in (run / "trials.jsonl").read_text().splitlines() if x.strip())
                         if r.get("g3") and r.get("traj")] if (run / "trials.jsonl").is_file() else []
                if len(held_) >= 5:                    # >= 5/10 held (plans/2026-10-27-posture-fix.md)
                    M, src = RC.measured_M(kind, run)
                    how = f"median hold of {run.name} ({len(held_)} held)"
                else:
                    M0, src = RC.measured_M(kind, RC.PLACING / SIDE_MOUTH_RUN)
                    M = RC.side_M(M0, 0, "base")
                    how = (f"{run.name} held {len(held_)} ({'not run' if not (run / 'trials.jsonl').is_file() else 'under 5'}): "
                           f"the median hold of {SIDE_MOUTH_RUN} (wrist toward the mouth) rolled 180 deg about the approach")
                Ms[kind] = (M, {"source": how, "trial": src["trial"], "pivot_deg": src["pivot_deg"], "T_tcp": src["T_tcp"],
                                "below_top_m": .045, "wrist_side": "base"})
            else:
                run = args.inhand_cup if kind == "cup" else None
                M, src = RC.measured_M(kind, run)
                Ms[kind] = (M, {"source": f"median hold of {run.name if run else 'limits_v1/gate2_' + kind}",
                                "trial": src["trial"], "pivot_deg": src["pivot_deg"], "T_tcp": src["T_tcp"]})
        return Ms[kind]

    def off_goal(earlier):
        """[name, mm, deg] of the earlier dishes beyond 15 mm / 15 deg of their goals, each in its rack's frame (one rack
        is pushed in while the robot works)."""
        now = backend.poses()
        res = []
        for n in earlier:
            r = meta[n]["rack"]
            A = inv(T_of(now[r]["position_m"], now[r]["quaternion_xyzw"])) @ T_of(now[n]["position_m"], now[n]["quaternion_xyzw"])
            Bg = inv(F_goal[r]) @ goal[n]
            d, a_ = float(np.linalg.norm(A[:3, 3] - Bg[:3, 3])), rot_deg(A, Bg)
            if d > KEEP_TOL[0] or a_ > KEEP_TOL[1]:
                res.append([n, round(d * 1e3, 1), round(a_, 1)])
        return res

    rack_override = {}                                 # D-ring extra try: a dish's rack state other than its own (None = both out)

    def rack_of(o):
        return rack_override[o] if o in rack_override else meta[o]["rack"]

    def reset(i, oid, M, keep=None):
        """``keep`` (variant D): the dishes already dropped stay where they lie (no rack reset, no teleport)."""
        tlog.set(move=i + 1, dish=oid, attempt=0, phase="trial reset")
        harness.armed = False
        harness.clear_carried()
        detach()
        backend.set_rigid_pose(backend.objects[oid], park_pose(i))   # parked BEFORE the jaws open (round 8's plate face
        rig.gripper(False)                                           # switch dropped the plate for 0.75 s)
        rests = {}
        if (seat_bowl or seat or sampler or roll or ring) and keep:   # plans/2026-10-29-bowl-last.md: every dish at rest first
            rests["before_teleport"] = rest_wait(list(keep))
        rig.teleport_arm(ns.q_home)
        post_reset()                                   # the joint log starts at home (the previous dish's retreat is its own)
        rig.step(5)
        if (seat_bowl or seat or sampler or roll or ring) and keep:   # ... and again before the search
            rests["before_search"] = rest_wait(list(keep))
        if keep is None:
            set_racks(None)
            for j, n in enumerate(names):
                if n != oid:
                    backend.set_rigid_pose(backend.objects[n], pose(goal[n]) if j < i else park_pose(j))
        rig.step(2)
        if carry or perch or seat_bowl or seat or sampler or ring:   # the search sees the racks in this dish's state
            set_racks(rack_of(oid))
        sync_world()
        kw = {"held": (kinds[oid], M, oid), "ignore": (oid,)}
        # plans/2026-10-27-bowls-plates-down.md: with the dish in hand the start is the collision-free elbow-up home IK with
        # the smallest max |q|; ties (plans/2026-10-27-bowls-lip-plates-fix.md) by the smallest SUM of |q - HOME_Q| (round 8's
        # largest single joint picked the pan -174 branch: 164-229 deg of pan per carry)
        # (coordinator, 2026-10-27: the 2 mm held-dish margin applies here too: a held bowl frees the wrist-3-at-0 branches,
        # wrist 1 10.6 mm from it; a plate or cup still overlaps them)
        HQ = np.asarray(ns.ur5e.HOME_Q, dtype=float)
        all_ = rig.ik_all(ns.T_home, HQ)
        branches = [{"q_deg": [round(float(v), 1) for v in np.degrees(q)],
                     "max_abs_deg": round(float(np.degrees(np.abs(q).max())), 1),
                     "hits": sorted({f"{a}|{b}" for a, b in arm.collisions(q, first=False, **kw)})} for q in all_]
        free_ = [q for q, b in zip(all_, branches) if not b["hits"]]
        q_start = (min(free_, key=lambda q: (round(float(np.abs(q).max()), 6), float(np.abs(q - HQ).sum())))
                   if free_ else ns.q_home)
        over = sorted({h_ for b in branches for h_ in b["hits"]})
        home_why = (f"{len(free_)}/{len(all_)} elbow-up home branches free at the {arm.held_margin * 1e3:.0f} mm held-dish "
                    f"margin" + (f"; the others overlap ({', '.join(over)})" if over else "")
                    + ("; chosen: the smallest max |q|, ties the smallest sum of |q - HOME_Q|" if free_
                       else "; none free: q_home"))
        carry_rec = None
        if carry:                                      # plans/2026-10-27-carry-rules.md: the Kit-free combo search decides
            carry_state.clear()
            if free_:
                pose_of = lambda h, s_: lip_tcp(M, goal[oid], ns.centroids[kinds[oid]], h, s_, np)[0]
                res = carry_search(rig, arm, free_, pose_of, kw, flags.z_safe_m)
                carry_state.update(res, homes=free_)
                carry_rec = {k: res[k] for k in ("scan", "n", "checked", "straight", "seconds")}
                carry_rec.update(best_cost_per_home_deg={json.dumps([round(float(v), 1) for v in np.degrees(free_[b])]):
                                                         round(float(np.degrees(c)), 1) for b, c in res["best_per_home"].items()},
                                 chosen=None if res["chosen"] is None else combo_rec(res["chosen"], free_, np),
                                 ranked=[combo_rec(c, free_, np) for c in res["ranked"][:DROP_TRIES]],
                                 rule="combos = free home branch x spin (-45..45 deg, 5 deg) x elbow-up IK branch of the drop "
                                      "pose; the lowest height with a collision-free drop IK; there straight two-leg paths "
                                      "first (arm.path_free, 0.02 rad), then the smallest largest-joint travel over start -> "
                                      "the drop pose at the safe height -> the drop pose, ties the sum; ranked = the chosen "
                                      "branch's combos (the drop tries)")
                if res["chosen"] is not None:
                    q_start = free_[res["chosen"]["b"]]
                    home_why += ("; --carry nearest: the home branch of the best combo (cost "
                                 f"{carry_rec['chosen']['cost_deg']} deg, legs {carry_rec['chosen']['legs']})")
                else:
                    home_why += "; --carry nearest: no free drop combo at any height (the rule above)"
        if perch:                                      # plans/2026-10-28-perch-release.md: the Kit-free over-lean search
            carry_state.clear()
            if free_:
                t_only, jaw_ = None, None
                if roll:                               # plans/2026-10-29-perch-roll.md: the curve's best tilt, the jaw filter
                    t_only = [float(tt["bowls"][oid]["best_tilt"])]
                    if args.jaw_check:
                        sync_world()
                        r_ = meta[oid]["rack"]
                        now_ = backend.poses()
                        F_s = T_of(now_[r_]["position_m"], now_[r_]["quaternion_xyzw"]) @ inv(F_goal[r_])
                        jb = jaw_bodies(arm, tt_rows, oid, t_only[0], F_s, np)
                        jaw_ = jaw_fn(arm, jb, np)
                        jaw_info.update(samples=len(jb), repeats=len({x[0] for x in jb}))
                res = overlean_search(rig, arm, free_, M, goal[oid], ns.centroids[kinds[oid]], oid, kinds[oid], kw,
                                      flags.z_safe_m, closed_hulls(), tilts_only=t_only, jaw=jaw_)
                carry_state.update(res, homes=free_)
                carry_rec = {"tilts": res["tilts"], "counts": res["counts"], "first_fail": res["first_fail"],
                             "seconds": res["seconds"], "jaw": res.get("jaw"),
                             "chosen": None if res["chosen"] is None else perch_rec(res["chosen"], free_, np),
                             "ranked": [perch_rec(c, free_, np) for c in res["ranked"]],
                             "rule": "combos = carried tilt (the goal's + 5 deg steps to 90) x spin (-45..45 deg, 5 deg) x free "
                                     "home branch x elbow-up IK branch of the contact pose (the bowl alone lowered straight "
                                     "down from 350 mm to its first touch on the 1 mm grid); pass = the contact pose free (15 mm "
                                     "arm / 2 mm held margins) with the open and the closed jaws, a free descent start (the "
                                     "lowest 25 mm-grid height >= perch + 50 mm, IK continuous with the contact branch) and no "
                                     "gripper or link touch from there down to the perch; ranked mouth-down static roll first, "
                                     "then the lowest perch, then the smallest largest-joint travel over transit_to's two legs "
                                     "to the descent start, ties the sum; ranked = the chosen home branch's passing combos"}
                if res["chosen"] is not None:
                    q_start = free_[res["chosen"]["b"]]
                    home_why += ("; --carry overlean: the home branch of the best combo (tilt "
                                 f"{carry_rec['chosen']['tilt_deg']} deg, spin {carry_rec['chosen']['spin_deg']:+.0f}, perch "
                                 f"{carry_rec['chosen']['perch_mm']} mm, cost {carry_rec['chosen']['cost_deg']} deg)")
                else:
                    home_why += "; --carry overlean: no passing combo (no perch)"
        if seat_bowl:                                  # plans/2026-10-29-bowl-last.md: the Kit-free seat search
            carry_state.clear()
            if free_:
                sync_world()
                res = seat_bowl_search(rig, arm, free_, M, goal[oid], ns.centroids[kinds[oid]], oid, kinds[oid], kw,
                                       flags.z_safe_m, closed_hulls())
                carry_state.update(res, homes=free_)
                carry_rec = {"tilts": res["tilts"], "counts": res["counts"], "first_fail": res["first_fail"],
                             "pairs_top": res["pairs_top"], "seconds": res["seconds"],
                             "chosen": None if res["chosen"] is None else seat_rec(res["chosen"], free_, np),
                             "ranked": [seat_rec(c, free_, np) for c in res["ranked"]],
                             "rule": "pairs = carried tilt (the goal's, + 5 deg steps to 90) x spin (-45..45, 5 deg); the "
                                     "predicted stop = the higher first touch of the bowl alone and of the tool (closed jaws + "
                                     "wrist 3) lowered straight down; ranked by the stop's height above the seat, a bowl stop "
                                     "before a hand stop, then the mouth-down roll at the bowl's perch; combos = free home branch "
                                     "x elbow-up IK branch of the descent start (lowest 25 mm-grid height >= stop + 50 mm, free "
                                     "at 15 mm / 2 mm held with both jaws) whose straight line down to the stop is IK-continuous "
                                     "with the links >= 2 mm from the scene; then the smallest largest-joint travel"}
                if res["chosen"] is not None:
                    q_start = free_[res["chosen"]["b"]]
                    home_why += (f"; --carry seat: the home branch of the best combo (tilt {carry_rec['chosen']['tilt_deg']} deg, "
                                 f"spin {carry_rec['chosen']['spin_deg']:+.0f}, stop {carry_rec['chosen']['stop_body']} "
                                 f"{carry_rec['chosen']['stop_mm']} mm, cost {carry_rec['chosen']['cost_deg']} deg)")
                else:
                    home_why += "; --carry seat: no passing combo"
        if seat:                                       # window 2 of plans/2026-10-28-plate-seat.md: the home branch by leg 1
            carry_state.clear()
            sp_ = seat_plan(oid, M, free_)
            carry_state.update(sp_)
            ch_ = sp_.get("chosen")
            carry_rec = {"seat_mm": (sp_.get("sk") or {}).get("seat_mm"), "seat_s": sp_.get("seat_s"),
                         "plan_s": sp_.get("plan_s"), "lines": sp_.get("lines"), "choices": sp_.get("choices"),
                         "chosen": None if ch_ is None else {"home_q_deg": [round(float(x), 1) for x in np.degrees(free_[ch_["b"]])],
                                                             "leg1": ch_["leg1"], "descent_start_mm": round(ch_["h_s"] * 1e3),
                                                             "move_deg": round(float(np.degrees(ch_["move"])), 1)}}
            if ch_ is not None:
                q_start = free_[ch_["b"]]
                home_why += (f"; --insert seat: the home branch whose leg 1 to the top of the tine-axis line is "
                             f"{ch_['leg1']} (largest joint move {carry_rec['chosen']['move_deg']} deg)")
            else:
                home_why += "; --insert seat: no home branch with a planned leg 1 (or no descent start)"
        if sampler:                                    # plans/2026-10-29-sampler-release.md: the home branch by leg 1
            carry_state.clear()
            sp_ = sampler_plan(oid, M, free_)
            carry_state.update(sp_)
            ch_ = sp_.get("chosen")
            carry_rec = {"release": sp_.get("release"), "stop_pred": sp_.get("stop_pred"), "plan_s": sp_.get("plan_s"),
                         "lines": sp_.get("lines"), "choices": sp_.get("choices"),
                         "chosen": None if ch_ is None else {"home_q_deg": [round(float(x), 1) for x in np.degrees(free_[ch_["b"]])],
                                                             "leg1": ch_["leg1"], "descent_start_mm": round(ch_["h_s"] * 1e3),
                                                             "move_deg": round(float(np.degrees(ch_["move"])), 1)}}
            if ch_ is not None:
                q_start = free_[ch_["b"]]
                home_why += (f"; --insert sampler: the home branch whose leg 1 to the top of the line above the release is "
                             f"{ch_['leg1']} (largest joint move {carry_rec['chosen']['move_deg']} deg)")
            else:
                home_why += "; --insert sampler: no home branch with a planned leg 1 (or no descent start)"
        if ring:                                       # plans/2026-10-29-ring-place.md: picked at the gate's counter spot (the
            carry_state.clear()                        # run's: RING_SPOT_XY = the study's, plans/2026-11-04-fifteen-of-fifteen.md)
            T_spot = T_of(*(lambda p_: (p_["position_m"], p_["quaternion_xyzw"]))(counter_pose(spot_xy, mouth_down=True)))
            pick_all = rig.ik_all(T_spot @ inv(M), np.asarray(ns.q_home))
            pick_free = sorted([q for q in pick_all if arm.free(q, **kw)], key=lambda q: float(np.abs(q - ns.q_home).max()))
            rp_ = ring_plan(oid, M, pick_free[0]) if pick_free else {"ranked": [], "counts": {"pick_ik": len(pick_all)},
                                                                       "first_fail": "no collision-free IK at the counter pick"}
            carry_state.update(rp_)
            carry_rec = {k: rp_.get(k) for k in ("counts", "first_fail", "seconds", "spins")}
            carry_rec.update(pick_ik=len(pick_all), pick_free=len(pick_free),
                             chosen=None if not rp_.get("ranked") else ring_rec(rp_["ranked"][0]),
                             ranked=[ring_rec(c) for c in (rp_.get("ranked") or [])[:DROP_TRIES]])
            if pick_free:
                q_start = pick_free[0]
                home_why = (f"--hold foot-out: the bowl mouth down at the run's counter spot {list(spot_xy)}, the arm at its "
                            f"pick IK ({len(pick_free)}/{len(pick_all)} elbow-up IK free with the bowl held; the smallest "
                            f"largest joint move from the gate's home)")
            else:
                home_why = f"--hold foot-out: no collision-free pick IK at the counter spot ({len(pick_all)} IK)"
        rig.teleport_arm(q_start)
        rig.step(5)
        T_d = rig.tcp() @ M
        backend.set_rigid_pose(backend.objects[oid], pose(T_d))
        attach(oid, T_d)
        drift = attach_drift(oid)
        if drift > .003 and not att_mode["pin"]:       # the fixed joint did not hold: the pin fallback from now on
            att_mode.update(pin=True, why=f"fixed joint drift {drift * 1e3:.1f} mm after 12 ticks ({oid})")
            print(f"[WARN] {att_mode['why']}: pinning the dish instead", flush=True)
            detach()
            T_d = rig.tcp() @ M
            backend.set_rigid_pose(backend.objects[oid], pose(T_d))
            attach(oid, T_d)
            drift = attach_drift(oid)
        rig.step(SPAWN_SETTLE_TICKS - 12)
        set_racks(rack_of(oid))
        theta = rig.gripper(True)
        sync_world()
        harness.pad_max_interval = {"pad_L": 0., "pad_R": 0.}
        harness.arm_guard()
        harness.set_carried(oid)
        M_now = inv(rig.tcp()) @ T_now(oid)
        return {"attach": "pin (fallback)" if att_mode["pin"] else "fixed joint", "attach_drift_mm": round(drift * 1e3, 2),
                "jaw_rad": round(theta, 4), "in_hand_vs_M_mm": round(float(np.linalg.norm(M_now[:3, 3] - M[:3, 3])) * 1e3, 2),
                "home_branch_free": bool(arm.free(q_start, **kw)), "q_start": np.round(q_start, 4).tolist(),
                "q_start_deg": [round(float(v), 1) for v in np.degrees(q_start)], "home_free_solutions": len(free_),
                "home_rule": "the collision-free elbow-up IK of T_home with the smallest max |q| (ties: the smallest sum of "
                             "|q - HOME_Q|)", "held_margin_mm": round(arm.held_margin * 1e3, 1), "home_branches": branches,
                "home_why": home_why, ("ring_plan" if ring else "perch_search" if perch else "seat_search" if seat_bowl else "seat_plan" if seat
                                       else "sampler_plan" if sampler else "carry_search"): carry_rec,
                "rest_waits": rests,
                "off_before_robot": off_goal(names[:i] if keep is None else keep)}

    # ---------------------------------------------------------------- S1-S3 (the arm model, no physics)
    def line_from(T0, T1, n, q_seed, kw):
        qs = rig.line_q(T0, T1, n, q_seed)
        if qs is None:
            return None, "IK branch jump on the line"
        for k_, q in enumerate(qs):
            if not arm.free(q, **kw):
                return None, f"collides at waypoint {k_}/{n}: {arm.collisions(q, **kw)[:2]}"
        return qs, None

    def hits_of(T, kw):
        qs = rig.ik_all(T, rig.q())
        return f"{len(qs)} IK solutions, first hits {arm.collisions(qs[0], first=False, **kw)[:3] if qs else None}"

    def plan_place(oid, M, T_goal):
        """(failing step or None, why, info) of S1-S3 from the current joints."""
        kw = {"held": (kinds[oid], M, oid), "ignore": (oid,)}
        sync_world()
        info = {}
        for h in RAISES_M:
            T_d = np.array(T_goal, copy=True)
            T_d[2, 3] += h
            if goal_qs(T_d @ inv(M), **kw):
                info["s1_raise_m"] = h
                break
        else:
            T_d = np.array(T_goal, copy=True)
            T_d[2, 3] += RAISES_M[0]
            return "S1", (f"no collision-free IK with the dish at its goal orientation 0.10-0.35 m above its goal "
                          f"(+0.10 m: {hits_of(T_d @ inv(M), kw)})"), info
        T_rel = np.asarray(T_goal) @ inv(M)
        T_rel[2, 3] += flags.release_above_m
        T_pp = T_rel.copy()
        T_pp[2, 3] += flags.hover_m                     # vertical insertion, as the sampler's move
        q_pp, q_rel = goal_qs(T_pp, **kw), goal_qs(T_rel, **kw)
        info.update(n_free_preplace=len(q_pp), n_free_release=len(q_rel))
        if not q_pp or not q_rel:
            return "S2", "; ".join(f"no collision-free IK at the {w} ({hits_of(T, kw)})" for w, q_, T in
                                   (("pre-place", q_pp, T_pp), ("release", q_rel, T_rel)) if not q_), info
        q0, why = rig.q(), None
        for q in q_pp[:3]:
            qs_ins, why_ins = line_from(T_pp, T_rel, 60, q, kw)
            if qs_ins is None:
                why = f"insertion line: {why_ins}"
                continue
            if arm.path_free(q0, q, step=.02, **kw):
                return None, None, {**info, "path": "straight joint move", "plan": (q, None, qs_ins, T_rel, T_pp)}
            t0 = time.monotonic()
            path = C.rrt_connect(arm, q0, [q], time_s=RRT_S, collide_kw=kw, seed=1, shortcut_s=args.shortcut_s)
            info["rrt_s"] = round(time.monotonic() - t0, 1)
            if path is None:
                why = (f"no joint path to the pre-place (RRT-Connect {RRT_S:.0f} s"
                       f"{'; the start is in collision: ' + str(arm.collisions(q0, **kw)[:2]) if not arm.free(q0, **kw) else ''})")
                continue
            return None, None, {**info, "path": f"RRT-Connect, {len(path)} waypoints", "plan": (q, path, qs_ins, T_rel, T_pp)}
        return "S3", why, info

    # ---------------------------------------------------------------- S4-S6 (Isaac)
    def carry(q, path):
        tlog.set(phase="transport")
        ok = (rig.move_joint(q, max_step=.004, settle_tol=flags.tol_joint_move_held) if path is None
              else rig.follow(path, steps_per=10, settle_tol=flags.tol_rrt))
        return None if ok else f"blocked in the carry {getattr(rig, 'last_fail', None)}"

    retreat = {}

    def release(oid, back=False):
        """``back`` (variant I, rows I3: plans/2026-10-27-carry-rules.md): the open hand backs along -TCP z (its approach) up to
        RELEASE_BACK_MAX_M through a collision-checked line (waypoints every BACK_STEP_M; the arm model with the released plate
        a scene dish, nothing ignored; as line_free lets the fingers touch the dish being released near it, the gripper alone
        may touch this plate in the first ALLOW_CONTACT_M of the back-off), truncated at its last free waypoint; it then rises
        flags.hover_m in world z only when the open gripper hulls are RIM_CLEAR_M or more from the released plate (FCL
        distance at the backed pose), else it stops there ("skipped: fingers at the rim"). Without ``back``: the rise alone.
        ``retreat`` gets each leg's outcome."""
        T_r = T_now(oid)
        detach()
        harness.clear_carried()
        tlog.set(phase="release")
        t_open0 = backend.tick_index
        rig.gripper(False)
        tlog.set(phase="retreat")
        retreat.clear()
        retreat.update(open_start_tick=t_open0, open_end_tick=backend.tick_index)
        if back:
            sync_world()                               # the released plate where it lies now
            T0 = rig.tcp()
            bk = T0.copy()
            bk[:3, 3] -= RELEASE_BACK_MAX_M * bk[:3, 2]
            n = int(round(RELEASE_BACK_MAX_M / BACK_STEP_M)) + 1
            qs, blocked, last = [], None, -1
            prev = rig.q()
            for j in range(n):                         # uniform 5 mm waypoints, branch-continuous (rig.line_q's rule)
                T_j = T0.copy()
                T_j[:3, 3] -= BACK_STEP_M * j * T0[:3, 2]
                q = rig.ik(T_j, prev)
                if q is None or float(np.abs(q - prev).max()) > IK_JUMP_MAX_RAD:
                    blocked = f"IK branch jump at {j * BACK_STEP_M * 1e3:.0f} mm"
                    break
                may = (oid,) if j * BACK_STEP_M < ALLOW_CONTACT_M else ()
                hits = arm.collisions(q, first=False, gripper_may_touch=may)
                if hits:
                    blocked = {"at_mm": round(j * BACK_STEP_M * 1e3), "pairs": sorted({f"{a}|{b}" for a, b in hits})}
                    break
                qs.append(q)
                prev, last = q, j
            back_ok = None
            if last >= 1:                              # a smooth line to the last free waypoint (its own smoothstep)
                T_end = T0.copy()
                T_end[:3, 3] -= BACK_STEP_M * last * T0[:3, 2]
                line = rig.line_q(T0, T_end, max(3, last + 1), rig.q())
                back_ok = None if line is None else bool(rig.follow(line))
            T_b = rig.tcp()
            retreat.update(back_mm=round(float(-(T_b[:3, 3] - T0[:3, 3]) @ T0[:3, 2]) * 1e3, 1),
                           back_free_mm=round(max(last, 0) * BACK_STEP_M * 1e3), back_followed=back_ok,
                           back_blocked_by=blocked if blocked else f"none (the full {RELEASE_BACK_MAX_M * 1e3:.0f} mm free)")
            T_p = T_now(oid)                           # the rim test: the open gripper vs the released plate at the backed pose
            fc = arm.fall_clearance(T_b, (kinds[oid], inv(T_b) @ T_p, oid), (0.,))
            rim = fc["per_link_mm"].get("gripper", fc["min_mm"])
            retreat["rim_clear_mm"] = rim
            if rim < RIM_CLEAR_M * 1e3:
                retreat["rise"] = f"skipped: fingers at the rim ({rim:.1f} mm)"
                return pose(T_r)
        up = rig.tcp()
        up[2, 3] += flags.hover_m
        qs = rig.line_q(rig.tcp(), up, 40, rig.q())
        retreat["rise"] = None if qs is None else bool(rig.follow(qs))
        return pose(T_r)

    def drop_retreat_plan(oid, kind, T_s, q_s, h_s, h_rel, v, T_seat):
        """DROP_RETREAT_RULE Kit-free before the descent (round 3): the TCP walked down the tine axis from the descent start to the
        planned release (IK-continuous, 1 mm steps), the released plate at its planned seat ``T_seat`` among the targets;
        drop_retreat_check's record (no joints), or None when the walk breaks."""
        v = np.asarray(v, dtype=float)
        prev, T = np.asarray(q_s, dtype=float), np.asarray(T_s, dtype=float).copy()
        n_ = int(round((float(h_s) - float(h_rel)) * 1e3))
        for j in range(1, n_ + 1):
            T = np.asarray(T_s, dtype=float).copy()
            T[:3, 3] -= j * 1e-3 * v
            q = rig.ik(T, prev)
            if q is None or float(np.abs(np.asarray(q) - prev).max()) > .1:
                return {"variant": None, "why": f"IK branch jump {n_ - j} mm above the release on the walk down"}
            prev = np.asarray(q, dtype=float)
        tg_ = scene_targets(arm, ignore=(oid,)) + [(oid, arm.checker._body(kind, _pose_of(T_seat), oid))]
        rt_ = drop_retreat_check(arm, rig.ik, prev, T, v, float(h_s) - float(h_rel), tg_, np, dish=oid)
        rt_["judged_at"] = "Kit-free: the planned release, the released plate at its planned seat"
        return rt_

    def drop_rest_wait(oid, v, T_seat):
        """DROP_REST_RULE (round 3, the user 2026-11-05 00:05 EST): after the open, one tick at a time until the released plate's
        centroid speed has stayed under DROP_REST_SPEED_M_S for DROP_REST_TICKS consecutive ticks, at least RING_WAIT_TICKS and
        at most DROP_REST_CAP_TICKS ticks. The record: ticks_to_rest (the tick, after the open's ramp, at which the final slow
        streak reached DROP_REST_TICKS; None = capped), the ticks waited, the last and the largest speed, and the plate's height
        along the tine axis ``v`` above its planned seat ``T_seat`` (and its centroid above the goal's, world z) when the retreat
        starts."""
        c_ = np.r_[ns.centroids[kinds[oid]], 1.]
        dt = float(backend.dt)
        T_0 = T_now(oid)
        p_prev = (T_0 @ c_)[:3]
        slow, n, sp_, sp_max = 0, 0, 0., 0.
        tlog.set(phase="rest wait")
        while n < DROP_REST_CAP_TICKS:
            rig.step(1)
            n += 1
            p_ = (T_now(oid) @ c_)[:3]
            sp_ = float(np.linalg.norm(p_ - p_prev)) / dt
            p_prev = p_
            sp_max = max(sp_max, sp_)
            slow = slow + 1 if sp_ < DROP_REST_SPEED_M_S else 0
            if slow >= DROP_REST_TICKS and n >= RING_WAIT_TICKS:
                break
        rested = slow >= DROP_REST_TICKS
        T_p = T_now(oid)
        v_ = np.asarray(v, dtype=float)
        return {"rested": bool(rested), "ticks_to_rest": (n - slow + DROP_REST_TICKS) if rested else None, "wait_ticks": n,
                "speed_mm_s_last": round(sp_ * 1e3, 2), "speed_mm_s_max": round(sp_max * 1e3, 1),
                "plate_above_seat_axis_mm": round(float((T_p[:3, 3] - np.asarray(T_seat)[:3, 3]) @ v_) * 1e3, 1),
                "plate_above_goal_mm": round((float((T_p @ c_)[2]) - float((goal[oid] @ c_)[2])) * 1e3, 1),
                "moved_mm": round(float(np.linalg.norm((T_p @ c_)[:3] - (T_0 @ c_)[:3])) * 1e3, 1),
                "rule": DROP_REST_RULE}

    def release_up(oid, v, h_up, row=None, T_seat=None, plan=None):
        """D2's release (plans/2026-11-04-fifteen-of-fifteen.md, the tine-guided plate release): the jaws opened where the descent
        stopped; round 3 (the user, 2026-11-05 00:05 EST): the rest wait (drop_rest_wait), then DROP_RETREAT_RULE checked at the
        released plate's pose now (drop_retreat_check): the back-off 20 mm along -TCP z when clear, then the open hand up the tine
        axis ``v`` by ``h_up`` (back to the descent start's height); else the rise alone (the earlier retreat). ``retreat`` gets
        the legs' outcome; ``row`` gets "drop_rest" and "drop_retreat" (``plan``: the Kit-free check at the seat)."""
        T_r = T_now(oid)
        detach()
        harness.clear_carried()
        tlog.set(phase="release")
        t_open0 = backend.tick_index
        rig.gripper(False)
        retreat.clear()
        retreat.update(open_start_tick=t_open0, open_end_tick=backend.tick_index, back_mm=0.)
        h_up = max(float(h_up), 0.)
        v = np.asarray(v, dtype=float)
        rest_ = drop_rest_wait(oid, v, T_seat if T_seat is not None else T_r)
        tlog.set(phase="retreat check")
        sync_world()                                   # the released plate where it rests now
        T0 = rig.tcp()
        rt_ = drop_retreat_check(arm, rig.ik, rig.q(), T0, v, h_up, scene_targets(arm), np, dish=oid)
        tlog.set(phase="retreat")
        retreat.update(variant=rt_["variant"], rest_wait_ticks=rest_["wait_ticks"])
        if rt_["variant"] == "back-off":               # 20 mm along -TCP z (the approach), then the rise
            T1 = T0.copy()
            T1[:3, 3] -= DROP_BACK_M * T0[:3, 2]
            qs_b = rig.line_q(T0, T1, 20, rig.q())
            retreat.update(back_mm=round(DROP_BACK_M * 1e3, 1),
                           back=None if qs_b is None else bool(rig.follow(qs_b, settle_tol=flags.tol_rise)),
                           back_done_mm=round(float(-(rig.tcp()[:3, 3] - T0[:3, 3]) @ T0[:3, 2]) * 1e3, 1))
        T0r = rig.tcp()
        T1 = T0r.copy()
        T1[:3, 3] += h_up * v
        qs = rig.line_q(T0r, T1, max(3, int(round(h_up / SEAT_LINE_STEP_M)) + 1), rig.q())
        retreat.update(rise_axis="the tine axis", rise_mm=round(h_up * 1e3, 1),
                       rise=None if qs is None else bool(rig.follow(qs, settle_tol=flags.tol_rise)))
        if row is not None:
            row["drop_rest"] = rest_
            row["drop_retreat"] = {"variant": rt_["variant"], "back_mm": rt_["back_mm"], "rise_mm": rt_["rise_mm"],
                                   "checks": rt_["checks"], "release": rt_["release"], "judged_at": "the released plate's pose "
                                   "when the retreat starts (after the rest wait), the open hand at the stop", "plan": plan,
                                   "rule": DROP_RETREAT_RULE}
        return pose(T_r)

    def fail(row, step, why):
        row["steps"][step], row["failed_at"], row["why"] = "fail", step, why

    def place_steps(i, oid, row, T_goal=None):
        T_goal = goal[oid] if T_goal is None else T_goal       # variant S: the goal turned about the cup's own axis
        step, why, info = plan_place(oid, att["M"], T_goal)
        row["plan"] = {k: v for k, v in info.items() if k != "plan"}
        s13 = ("S1", "S2", "S3")
        for s_ in s13[:s13.index(step) if step else 3]:
            row["steps"][s_] = "pass"
        if step:
            return fail(row, step, why)
        q, path, qs_ins, T_rel, T_pp = info["plan"]
        why = carry(q, path)
        if why is None:
            tlog.set(phase="insertion")
            if not rig.follow(qs_ins):
                why = f"blocked on the insertion line {getattr(rig, 'last_fail', None)}"
        if why:
            return fail(row, "S4", why)
        row["steps"]["S4"] = "pass"
        row["release_pose"] = release(oid)
        tlog.set(phase="settle")
        rig.step(K.SETTLE_TICKS)
        T = T_now(oid)
        lat, dz, tilt = B.goal_distance(ns.centroids[kinds[oid]], T, T_goal)
        row.update(error={"lateral_mm": round(lat * 1e3, 1), "dz_mm": round(dz * 1e3, 1), "tilt_deg": round(tilt, 1)},
                   disturbed=off_goal(names[:i]), final_pose=pose(T))
        e = row["error"]
        if not (lat <= GOAL_TOL[0] and dz <= GOAL_TOL[1] and tilt <= GOAL_TOL[2]):
            return fail(row, "S5", f"settled {e['lateral_mm']} mm lateral, {e['dz_mm']} mm height, {e['tilt_deg']} deg "
                                   f"off its goal (limits 15 / 20 / 15)")
        row["steps"]["S5"] = "pass"
        if row["disturbed"]:
            return fail(row, "S6", f"earlier dishes off their goals: {row['disturbed']}")
        row["steps"]["S6"] = "pass"

    # ---------------------------------------------------------------- variant R: flip and set down (R1), re-pick (R2)
    def set_down(oid, row):
        kind, M = kinds[oid], att["M"]
        kw = {"held": (kind, M, oid), "ignore": (oid,)}
        sync_world()
        q0, top, pts = rig.q(), B.COUNTER["top_z_m"], np.asarray(ns.points[kind])
        found, n_rel, n_pre, n_line, rrt_left = None, 0, 0, 0, 3
        for tilt in SET_DOWN_TILTS:
            for k in range(12):
                Rm = Rotation.from_euler("xz", [np.pi - np.radians(tilt), 2 * np.pi * k / 12]).as_matrix()
                T_d = np.eye(4)
                T_d[:3, :3] = Rm
                T_d[:3, 3] = (spot_xy[0], spot_xy[1], top + SET_DOWN_DROP_M - float((pts @ Rm.T)[:, 2].min()))
                T_t = T_d @ inv(M)
                if not goal_qs(T_t, **kw):
                    continue
                n_rel += 1
                T_pre = T_t.copy()
                T_pre[2, 3] += flags.hover_m
                for q_pre in goal_qs(T_pre, **kw)[:2]:
                    n_pre += 1
                    line, _ = line_from(T_pre, T_t, 50, q_pre, kw)
                    if line is None:
                        continue
                    n_line += 1
                    if arm.path_free(q0, q_pre, step=.02, **kw):
                        found = (tilt, k, q_pre, None, line)
                    elif rrt_left > 0:
                        rrt_left -= 1
                        path = C.rrt_connect(arm, q0, [q_pre], time_s=RRT_S, collide_kw=kw, seed=1, shortcut_s=args.shortcut_s)
                        found = None if path is None else (tilt, k, q_pre, path, line)
                    if found:
                        break
                if found:
                    break
            if found:
                break
        row["set_down"] = {"cells_release_ik": n_rel, "preplace_ik": n_pre, "free_lines": n_line}
        if not found:
            return fail(row, "R1", (f"no collision-free set-down mouth down over the counter spot (tilts 0-60 deg x 12 yaws: "
                                    f"{n_rel} cells with a free release IK, {n_line} free lowering lines, no path)"))
        tilt, k, q_pre, path, line = found
        row["set_down"].update(tilt_deg=tilt, yaw_k=k)
        why = carry(q_pre, path)
        if why is None:
            tlog.set(phase="set-down lowering")
            if not rig.follow(line):
                why = f"blocked lowering to the counter {getattr(rig, 'last_fail', None)}"
        if why:
            return fail(row, "R1", why)
        release(oid)
        rig.step(K.SETTLE_TICKS)
        T = T_now(oid)
        off = float(np.hypot(T[0, 3] - spot_xy[0], T[1, 3] - spot_xy[1]))
        row["set_down"].update(settled=pose(T), axis_z=round(float(T[2, 2]), 3), off_spot_mm=round(off * 1e3, 1))
        if not (T[2, 2] < G.INVERTED and P.in_counter_band(T, B.COUNTER) and off < .06):
            return fail(row, "R1", f"not mouth down at the spot after the set-down (axis z {T[2, 2]:.2f}, {off * 1e3:.0f} mm off)")
        row["steps"]["R1"] = "pass"

    def ideal_set_down(oid, row):
        """R-ideal: R1 idealized -- the attachment removed, the jaws opened (the arm stays at home), the dish teleported
        mouth down onto the counter spot (rim 3 mm above the slab) and settled; R1 itself is not performed (skip)."""
        detach()
        harness.clear_carried()
        rig.gripper(False)
        harness.armed = False
        T = np.eye(4)
        T[:3, :3] = np.diag([1., -1., -1.])
        T[:3, 3] = (spot_xy[0], spot_xy[1], B.COUNTER["top_z_m"] + .003 + G.KIND_RIM[kinds[oid]][2])
        backend.set_rigid_pose(backend.objects[oid], pose(T))
        rig.step(K.SETTLE_TICKS)
        sync_world()
        harness.arm_guard()
        T = T_now(oid)
        row["set_down"] = {"idealized": "teleported mouth down onto the counter spot, settled 150 ticks", "settled": pose(T),
                           "axis_z": round(float(T[2, 2]), 3)}
        row.update(idealized="flip", note="the set-down was teleported (mouth down onto the counter spot, settled 150 ticks); "
                                          "R2 and S1-S6 ran as in R")

    def repick(oid, row):
        kind, T_o = kinds[oid], T_now(oid)
        base = (G.wall_grasps(T_o, dish="cup") if kind == "cup"
                else [g for g in G.bowl_grasps(T_o, 12, kind) if g["kind"] == "foot"])
        sync_world()
        base_xy = np.asarray(ns.mount["base"][:2])
        base = sorted(base, key=lambda g: -float(np.dot(g["T_tcp"][:2, 3] - T_o[:2, 3], base_xy - T_o[:2, 3])))
        found, why_n = None, {}
        for b_ in REPICK_TILTS_DEG:                    # straight down first, then leaned about the tangent axis
            for g in base:
                T_g = np.array(g["T_tcp"], copy=True)
                T_g[:3, :3] = Rotation.from_rotvec(np.radians(b_) * T_g[:3, 0]).as_matrix() @ T_g[:3, :3]
                H = G.hover_along(T_g, flags.hover_m)
                d = float(np.linalg.norm(T_g[:3, 3] - H[:3, 3]))
                allow_m = ALLOW_STRADDLE_M if g["kind"] in ("wall", "base") else ALLOW_CONTACT_M
                hov = goal_qs(H)[:3]
                if not hov:
                    why_n["no free hover IK"] = why_n.get("no free hover IK", 0) + 1
                for q_h in hov:
                    qs = rig.line_q(H, T_g, 100, q_h)
                    bad = None if qs is None else next((j for j, q in enumerate(qs) if not arm.free(
                        q, gripper_may_touch=(oid,) if d * (1 - j / 99) < allow_m else ())), None)
                    if qs is not None and bad is None:
                        found = ({**g, "T_tcp": T_g, "tilt_deg": b_}, H, q_h, qs)
                        break
                    w_ = "line IK jump" if qs is None else "approach hits " + ",".join(sorted({f"{a}-{b}" for a, b in arm.collisions(
                        qs[bad], gripper_may_touch=(oid,) if d * (1 - bad / 99) < allow_m else (), first=False)}))
                    why_n[w_] = why_n.get(w_, 0) + 1
                if found:
                    break
            if found:
                break
        row["repick_search"] = why_n
        if not found:
            return fail(row, "R2", f"no collision-free {'base' if kind == 'cup' else 'foot-ring'} grasp from above "
                                   f"({len(base)} yaws x tilts {REPICK_TILTS_DEG} deg): {why_n}")
        g, H, q_h, qs = found
        row["repick"] = {"grasp": g["kind"], "yaw": round(g["yaw"], 3), "tilt_deg": g["tilt_deg"]}
        tlog.set(phase="re-pick transit")
        why = transit_to(H, q_hover=q_h)
        if why:
            return fail(row, "R2", f"no approach: {why}")
        tlog.set(phase="re-pick approach")
        if not rig.follow(qs, settle_tol=flags.tol_approach):
            return fail(row, "R2", f"blocked on the approach {getattr(rig, 'last_fail', None)}")
        tlog.set(phase="re-pick close")
        theta = rig.gripper(True)
        jw = G.WALL_JAW_RAD if g["kind"] in ("wall", "base") else flags.jaw_wall_rad
        row["repick"]["jaw_rad"] = round(theta, 4)
        if not (jw[0] <= theta < jw[1]):
            return fail(row, "R2", f"jaws at {theta:.2f} rad: no wall between the pads")
        attach(oid)
        harness.set_carried(oid)
        tlog.set(phase="re-pick lift")
        up = rig.line_q(rig.tcp(), H, 60, rig.q())
        if up is None or not rig.follow(up, settle_tol=flags.tol_lift):
            return fail(row, "R2", f"blocked lifting {getattr(rig, 'last_fail', None)}")
        row["repick"]["M"] = np.round(att["M"], 4).tolist()
        row["steps"]["R2"] = "pass"

    # ---------------------------------------------------------------- variant D: let go above the goal (plans/2026-10-27-drop-test.md)
    def drop(i, oid, row, landed):
        """The dish as held (the counter hold's TCP orientation, turned about the vertical in 12 yaws) with its centroid
        over the goal centroid's x, y and DROP_HEIGHTS_M above it: the lowest pose with a collision-free IK (the racks in
        this dish's state, the earlier dishes where they lie), carried by transit_to (up to DROP_TRIES poses, else "no
        reach": the dish is parked), let go (joint removed, jaws opened, retreat) and the benchmark's settle; settled =
        its drift window (5 mm / 3 deg over the last 60 of 150 ticks) alone."""
        kind, M = kinds[oid], att["M"]
        kw = {"held": (kind, M, oid), "ignore": (oid,)}
        c = np.r_[ns.centroids[kind], 1.]
        g_c, m_c = (goal[oid] @ c)[:3], (M @ c)[:3]
        R0 = np.asarray(row["in_hand"]["T_tcp"])[:3, :3]
        if side_base:                                      # the cup flipped mouth down, the wrist above it
            R0 = flip_down(R0, np)
        elif side_mouth:                                   # the mouth-side hold turned mouth down, the wrist below it
            R0 = flip_up(R0, np)
        before = backend.poses()
        sync_world()
        if flip:                                           # the bowl turned mouth down about the closing axis
            R0 = choose_flip(oid, row, R0, M, m_c, g_c, kw)
        pose_of = ((lambda h, k: lip_tcp(M, goal[oid], ns.centroids[kind], h, k, np)[0]) if lip else
                   (lambda h, k: drop_tcp(R0, m_c, g_c, h, k, np)))
        tries, found = 0, None
        why = (f"no collision-free IK at any drop height ({DROP_HEIGHTS_M[0]}-{DROP_HEIGHTS_M[-1]} m above the goal) and "
               + ("spin (-45..45 deg about the bowl axis, 5 deg steps) and home branch" if carry else
                  "spin (0, +-15, +-30 deg about the bowl axis)" if lip else "yaw"))
        post.update(carry=True, start=rig.q())
        for cb in (carry_state.get("ranked") or [])[:DROP_TRIES] if carry else ():
            q_d, h, k, T = cb["q_drop"], cb["h"], cb["spin"], cb["T"]
            if not arm.free(q_d, **kw):                    # the world has not changed since the search
                continue
            tries += 1
            tlog.set(phase="carry to the drop pose")
            ns.plan_log.clear()
            w_ = transit_to(T, q_hover=q_d, **kw)
            legs = list(ns.plan_log)
            if w_ is None:
                found = (h, k)
                row["drop_move_deg"] = round(float(np.degrees(np.abs(q_d - np.asarray(row["reset"]["q_start"])).max())), 1)
                row["fall_clearance"] = arm.fall_clearance(T, kw["held"], FALL_DROPS_M, q=q_d)
                rec = combo_rec(cb, carry_state["homes"], np)
                row.update(carry_cost_pred=rec["cost_deg"], carry_sum_pred=rec["sum_deg"], carry_travel_pred=rec["travel_deg"],
                           legs_pred=rec["legs"], combo=rec, combo_rank=tries,
                           legs={"leg1": legs[0] if legs else None,
                                 "leg2": legs[1] if len(legs) > 1 else "none (no free IK above the drop pose: one leg)",
                                 "plans": legs})
                break
            why = f"no path to {tries} drop pose(s), the last at +{h:.3f} m spin {k:+.0f} deg: {w_} (legs {legs})"
        for h in DROP_HEIGHTS_M if not carry else ():
            q0_ = rig.q()                                  # within the lowest free height: the smallest joint move first
            cand = []
            for k in (LIP_SPINS_DEG if lip else range(12)):
                qs = goal_qs(pose_of(h, k), **kw)
                if qs:
                    cand.append((float(np.abs(qs[0] - q0_).max()), k))
            if not lip:                                    # D-lip: the spins in LIP_SPINS_DEG's order (exact lip first)
                cand.sort()
            for move, k in cand:
                T = pose_of(h, k)
                qs = goal_qs(T, **kw)                      # from where the arm is now (an earlier try may have moved it)
                if not qs:
                    continue
                tries += 1
                tlog.set(phase="carry to the drop pose")
                w_ = transit_to(T, q_hover=qs[0], **kw)
                if w_ is None:
                    found = (h, k)
                    row["drop_move_deg"] = round(float(np.degrees(move)), 1)
                    if lip:                                # listed, not a gate: the bowl moved 0-50 mm straight down
                        row["fall_clearance"] = arm.fall_clearance(T, kw["held"], FALL_DROPS_M, q=qs[0])
                    break
                why = (f"no path to {tries} drop pose(s), the last at +{h:.3f} m "
                       + (f"spin {k:+.0f} deg" if lip else f"yaw {30 * k} deg") + f": {w_}")
                if tries >= DROP_TRIES:
                    break
            if found or tries >= DROP_TRIES:
                break
        row["drop_tries"] = tries
        post["carry"] = False
        if found is None:
            harness.armed = False                          # never dropped: the dish leaves the scene
            detach()
            harness.clear_carried()
            backend.set_rigid_pose(backend.objects[oid], park_pose(i))
            if tries == 0 and lip:                         # no free IK anywhere: what the arm model says blocks it
                row["blocking"] = blocking_pairs(rig, arm, [(f"+{h * 1e3:.0f} mm spin {k:+.0f}", pose_of(h, k), kw)
                                                            for h in DROP_HEIGHTS_M
                                                            for k in (CARRY_SPINS_DEG if carry else LIP_SPINS_DEG)], np)
            return fail(row, "no reach", why)
        if lip:
            row.update(drop_height_m=found[0], spin_deg=found[1])
        else:
            row.update(drop_height_m=found[0], drop_yaw_deg=30 * found[1])
        if carry:
            qs_ = row["reset"]["q_start_deg"]
            row["start_branch"] = {"q_start_deg": qs_, "wrist_2_deg": qs_[4], "wrist_3_deg": qs_[5]}
        T_r = T_now(oid)
        carry_watch["arrived"] = True                      # --attach none: the carry ends at the drop pose
        row["release_pose"] = release(oid)
        s_ = ns.bench_settle(oid, "goal", T_r, before)
        T = T_now(oid)
        st = s_["settle"]
        settled = st["drift_mm"] <= ns.R.STABLE_POS_M * 1e3 and st["drift_deg"] <= ns.R.STABLE_ROT_DEG
        dist = s_["distance"]                              # B.goal_distance: lateral mm, |dz| mm, tilt deg
        row.update(settled=bool(settled), settle=st, final_pose=pose(T), mouth="up" if T[2, 2] > 0 else "down",
                   axis_z=round(float(T[2, 2]), 3), error={"lateral_mm": dist[0], "dz_mm": dist[1], "tilt_deg": dist[2]},
                   landed_on=sorted(b if a == oid else a for a, b in harness.contacts.pairs_with(oid)),
                   moved_earlier=[m for m in s_["rules"]["strict"] if m[0] in landed])
        if lip:                                            # the lean (axis from straight down) as landed vs the goal's
            row.update(lean_deg={"final": lean_deg(T, np), "goal": lean_deg(goal[oid], np)}, lean_from_goal_deg=dist[2])
        if side_mouth:                                     # plans/2026-11-04-grasp-place-s0.md: the drop curve's cup rule
            rk_ = meta[oid]["rack"]
            now_ = backend.poses()
            T_gr = T_of(now_[rk_]["position_m"], now_[rk_]["quaternion_xyzw"]) @ inv(F_goal[rk_]) @ goal[oid]
            ps_ = pose_seated(T, T_gr, ns.centroids[kind], np)
            row.update(seated={"seated": bool(ps_[0] and settled), "goal_pose_test": {"ok": ps_[0], "centroid_mm": ps_[1],
                                                                                    "axis_deg": ps_[2]},
                               "rule": "the cup settled with its centroid within 2 mm and its axis within 5 deg of the goal "
                                       "pose in the rack's frame now (frigidaire_drop_curve.py's cup rule)"},
                       at_goal=bool(B.within_goal(kind, (dist[0] / 1e3, dist[1] / 1e3, dist[2]))))
        landed.append(oid)
        if not settled:
            fail(row, "not settled", f"drift {st['drift_mm']} mm / {st['drift_deg']} deg over the last {K.WINDOW_TICKS} "
                                     f"ticks (limits 5 mm / 3 deg)")
        elif (side_base or side_mouth or flip or lip) and T[2, 2] >= 0:   # D-side / D-flip / D-lip: settled AND mouth down
            fail(row, "mouth up", f"settled mouth up (axis z {T[2, 2]:.2f})")
        elif side_mouth and not row["seated"]["seated"]:
            fail(row, "not seated", f"mouth down, off its goal pose (centroid {ps_[1]} mm, axis {ps_[2]} deg; limits 2 mm / 5 deg)")

    # ---------------------------------------------------------------- D-perch (plans/2026-10-28-perch-release.md)
    _closed = {}

    def closed_hulls():
        if "h" not in _closed:
            _closed["h"] = closed_gripper_hulls(C, np)
        return _closed["h"]

    def flange_force(oid):
        """The emulated UR5e flange F/T reading of the last tick: the summed PhysX normal force of the touching pairs between
        the tool (gripper pads, fingers, knuckles, palm; wrist_3_link; the held bowl) and the scene (racks, basket, tub and
        the other appliance parts, counter, every other dish). Tool-internal pairs (the pads squeezing the welded bowl at 88 /
        50 N, wrist 3 and the gripper) and the pedestal are not counted. (sum N, [[pair, N], ...] largest first)"""
        rows_ = getattr(harness.contacts, "step_rows", None) or {}
        tot, parts = 0., []
        for (a, b), r_ in rows_.items():
            ca, cb_ = r_["cats"]
            tool_a, tool_b = ca in SENSOR_TOOL_CATS or a in ("wrist_3_link", oid), cb_ in SENSOR_TOOL_CATS or b in ("wrist_3_link", oid)
            scene_a = ca in SENSOR_SCENE_CATS or (ca == "dish" and a != oid)
            scene_b = cb_ in SENSOR_SCENE_CATS or (cb_ == "dish" and b != oid)
            if (tool_a and scene_b) or (tool_b and scene_a):
                tot += float(r_["N"])
                parts.append([f"{a}|{b}", round(float(r_["N"]), 3)])
        return tot, sorted(parts, key=lambda x: -x[1])

    def lower_until_contact(oid, cb):
        """The guarded descent: from the descent start straight down in world z at DESCENT_SPEED_M_S (one IK waypoint per
        physics tick, 1/6 mm apart, IK-continuous from the carry's end joints, velocity feed-forward as rig.follow), the
        emulated flange sensor (flange_force) read after EVERY tick; it trips when the sum exceeds SENSOR_TRIP_N on
        SENSOR_TRIP_TICKS consecutive ticks and the arm then holds its current joint targets. Bounded DESCENT_BELOW_M under the
        predicted perch: no trip there = "no contact" (released there). The stop record."""
        kind = kinds[oid]
        dt = backend.dt
        dz = DESCENT_SPEED_M_S * dt
        u = np.asarray(cb.get("dir", (0., 0., 1.)), dtype=float)   # I4 (plans/2026-10-28-plate-seat.md): along the tine axis
        T_H = np.asarray(cb["T_start"], dtype=float)
        h_end_ = cb.get("h_end")                       # D2 (the tine-guided plate release): the descent bounded at this height above
        depth = cb["h_start"] - (float(h_end_) if h_end_ is not None else cb["h_b"] - DESCENT_BELOW_M)   # the seat instead
        qs, prev, ik_end = [], np.asarray(cb["q_start"], dtype=float), None
        for j in range(1, int(round(depth / dz)) + 1):
            T = T_H.copy()
            T[:3, 3] -= j * dz * u
            q = rig.ik(T, prev)
            if q is None or float(np.abs(q - prev).max()) > .05:     # 1/6 mm steps: a larger jump is a branch change
                ik_end = round(j * dz * 1e3, 1)
                break
            qs.append(q)
            prev = q
        c_ = np.r_[ns.centroids[kind], 1.]
        g_z = float((goal[oid] @ c_)[2])
        X = att.get("X")

        def weld_drift():
            if X is None:
                return None
            W3 = rig.tcp() @ inv(rig.T_w3_tcp)
            return round(float(np.linalg.norm((inv(W3) @ T_now(oid))[:3, 3] - X[:3, 3])) * 1e3, 2)

        def bowl_h(T_b):
            return round((float((T_b @ c_)[2]) - g_z) * 1e3, 1)
        T_d0, T_plan = T_now(oid), T_H @ np.asarray(att["M"])
        start = {"lag_rad": round(rig.lag(), 4), "weld_drift_mm": weld_drift(), "bowl_h_mm": bowl_h(T_d0),
                 "dish_vs_plan_mm": round(float(np.linalg.norm(T_d0[:3, 3] - T_plan[:3, 3])) * 1e3, 1),
                 "dish_vs_plan_deg": round(rot_deg(T_d0, T_plan), 2),
                 "tcp_vs_plan_mm": round(float(np.linalg.norm(rig.tcp()[:3, 3] - T_H[:3, 3])) * 1e3, 1)}
        z0 = float(rig.tcp()[:3, 3] @ u)
        vel = robot.data.default_joint_vel.torch.clone()
        q_cmd = rig.target[0, rig.arm].cpu().numpy().astype(float)
        over, trip, trace, zs, first, blocked, j = 0, None, [], [z0], None, None, 0
        peak = (0., None)
        mark = backend.tick_index
        tlog.set(phase="guarded descent")
        for j, q in enumerate(qs, 1):
            vel[0, rig.arm] = rig.torch.tensor((q - q_cmd) / dt, dtype=vel.dtype)
            robot.actuators.target_command.set_velocity_index(value=vel)
            rig.set_arm(q)
            rig.step(1)
            q_cmd = q
            f, parts = flange_force(oid)
            zs.append(float(rig.tcp()[:3, 3] @ u))
            if f > 0.:
                trace.append([j, round(f, 3)])
                if first is None:
                    first = {"tick": j, "force_N": round(f, 3), "pairs": parts[:4], "bowl_h_mm": bowl_h(T_now(oid)),
                             "tcp_down_mm": round((z0 - zs[-1]) * 1e3, 1)}
                if f > peak[0]:
                    peak = (f, j)
            over = over + 1 if f > SENSOR_TRIP_N else 0
            if over >= SENSOR_TRIP_TICKS:
                trip = (j, f, parts)
                break
            if rig.lag() > rig.lag_max:
                rig._fail(j, len(qs))
                blocked = getattr(rig, "last_fail", None)
                break
        rig._stop_vel(vel)                             # the arm holds its current joint targets
        T_b = T_now(oid)
        v = [(zs[k - 6] - zs[k]) / (6 * dt) * 1e3 for k in range(6, len(zs))]
        end_ = (first["tick"] if first else len(zs) - 1)
        steady = [x for k, x in enumerate(v, 6) if 24 <= k <= end_]
        other = sorted({"|".join(k) for k, act in harness.contacts.active.items() if act["last"] >= mark
                        and "arm" in act["cats"] and "wrist_3_link" not in k
                        and any(ct in SENSOR_SCENE_CATS + ("dish",) for ct in act["cats"])})
        kind_ = "none" if trip is None else ("bowl" if any(oid in p.split("|") for p, _ in trip[2]) else "hand")
        if trip is not None and kind != "bowl":        # I4: the plate's own pairs vs the tool's (pads, fingers, palm, wrist 3)
            own = any(oid in p.split("|") for p, _ in trip[2])
            tool = any(oid not in p.split("|") for p, _ in trip[2])
            kind_ = kind if own and not tool else "hand" if tool and not own else f"{kind}+hand"
        return {"kind": kind_, "ticks": j, "force_N": None if trip is None else round(trip[1], 2),
                "pairs": [] if trip is None else trip[2][:6], "commanded_mm": round(j * dz * 1e3, 2),
                "descent_mm": round((z0 - float(rig.tcp()[:3, 3] @ u)) * 1e3, 1), "bowl_h_mm": bowl_h(T_b),
                "dish_h_mm": bowl_h(T_b), "dir": [round(float(x), 5) for x in u],
                "weld_drift_mm": weld_drift(), "first_contact": first, "peak_N": round(peak[0], 2), "peak_tick": peak[1],
                "bound_mm": round(depth * 1e3, 1), "ik_end_mm": ik_end, "blocked": blocked, "start": start,
                "speed_mm_s": {"target": DESCENT_SPEED_M_S * 1e3, "steady_mean": round(float(np.mean(steady)), 2) if steady else None,
                               "steady_min": round(float(min(steady)), 2) if steady else None,
                               "steady_max": round(float(max(steady)), 2) if steady else None,
                               "within_10pct": round(float(np.mean([abs(x / (DESCENT_SPEED_M_S * 1e3) - 1.) <= .1
                                                                    for x in steady])), 3) if steady else None,
                               "rule": "TCP z over 6-tick windows, from tick 24 to the first sensed contact"},
                "other_arm_contacts": other, "trace": trace[:600],
                "rule": f"sum of the PhysX normal forces of the tool (pads, fingers, knuckles, palm, wrist 3, the held bowl) vs "
                        f"scene (racks, basket, tub, counter, other dishes) pairs, read every tick; trip at > {SENSOR_TRIP_N} N on "
                        f"{SENSOR_TRIP_TICKS} consecutive ticks; {DESCENT_SPEED_M_S * 1e3:.0f} mm/s; bounded "
                        + (f"{DESCENT_BELOW_M * 1e3:.0f} mm under the predicted perch" if h_end_ is None else
                           cb.get("bound_rule") or f"at {float(h_end_) * 1e3:.0f} mm above the seat (D2's planned release height)")}

    def perch_drop(i, oid, row, landed):
        """--carry overlean (rows D-perch): the ranked combos of reset()'s overlean_search (up to DROP_TRIES; the arm already on
        their home branch) carried by transit_to to the descent start (the bowl at its carried tilt and spin, 50-75 mm above
        its predicted perch), lowered by lower_until_contact, let go where it stopped (the joint removed, the jaws opened), the
        open hand backed out along -TCP z through the checked line and risen only clear of the rim (release(back=True), I3),
        the benchmark's settle; success = settled and mouth down. No passing combo: "no perch" (the bowl is parked)."""
        kind, M = kinds[oid], att["M"]
        kw = {"held": (kind, M, oid), "ignore": (oid,)}
        before = backend.poses()
        sync_world()
        homes = carry_state.get("homes") or []
        row["tilt_deg"] = {"goal": round(float(np.degrees(np.arccos(np.clip(-goal[oid][2, 2], -1., 1.)))), 1), "carried": None}
        post.update(carry=True, start=rig.q())
        tries, found = 0, None
        why = f"no passing combo: {json.dumps(carry_state.get('counts'))}; per tilt {json.dumps(carry_state.get('first_fail'))}"
        for cb in (carry_state.get("ranked") or [])[:DROP_TRIES]:
            if not arm.free(cb["q_start"], **kw):           # the world has not changed since the search
                continue
            tries += 1
            legs_pred = ["straight" if arm.path_free(a, b_, step=.02, **kw) else "rrt" for a, b_ in cb["legs"]]
            tlog.set(phase="carry to the descent start")
            ns.plan_log.clear()
            w_ = transit_to(cb["T_start"], q_hover=cb["q_start"], **kw)
            legs = list(ns.plan_log)
            if w_ is None:
                found = cb
                rec = perch_rec(cb, homes, np)
                one = "none (no free IK above the descent start: one leg)"
                row.update(combo=rec, combo_rank=tries, carry_cost_pred=rec["cost_deg"], carry_sum_pred=rec["sum_deg"],
                           carry_travel_pred=rec["travel_deg"],
                           legs_pred={"leg1": legs_pred[0], "leg2": legs_pred[1] if len(legs_pred) > 1 else one},
                           legs={"leg1": legs[0] if legs else None, "leg2": legs[1] if len(legs) > 1 else one, "plans": legs})
                break
            why = (f"no path to {tries} descent start(s), the last tilt {cb['theta']:.1f} spin {cb['spin']:+.0f} at "
                   f"+{cb['h_start'] * 1e3:.0f} mm: {w_} (legs {legs})")
        row["drop_tries"] = tries
        if roll:
            row.update(gate=tt["bowls"].get(oid), jaw_check=carry_state.get("jaw"), jaw_samples=dict(jaw_info))
        if found is None:
            post["carry"] = False
            harness.armed = False                          # never lowered: the bowl leaves the scene
            detach()
            harness.clear_carried()
            backend.set_rigid_pose(backend.objects[oid], park_pose(i))
            if roll:                                   # D-roll: the bowl at its goal for the bowls after it (the curve's scene)
                T_g_, rest_ = place_at_goal(oid)
                landed.append(oid)
                row.update(placed_at_goal=True, final_pose=pose(T_g_), rest=rest_)
                jt = carry_state.get("jaw") or {}
                if args.jaw_check and jt and not any(v["passing"] for v in jt.values()):
                    return fail(row, "not releasable", "not releasable: the rolling bowl meets the open hand at every spin "
                                                       f"(jaw check per spin: {json.dumps(jt)[:600]})")
            if not carry_state.get("ranked"):              # what the arm model says blocks the contact poses (as no reach)
                row["blocking"] = blocking_pairs(rig, arm, [
                    (f"tilt {t['theta']:.1f} spin {s_:+.0f}", overlean_tcp(M, goal[oid], ns.centroids[kind], t["h_b"], s_,
                                                                            t["theta"], np)[0], kw)
                    for t in carry_state.get("tilts") or [] if t.get("h_b") is not None for s_ in CARRY_SPINS_DEG], np)
            return fail(row, "no reach" if carry_state.get("ranked") else "no perch", why)
        cb = found
        qs_ = row["reset"]["q_start_deg"]
        row["start_branch"] = {"q_start_deg": qs_, "wrist_2_deg": qs_[4], "wrist_3_deg": qs_[5]}
        row["tilt_deg"]["carried"] = round(cb["theta"], 1)
        row.update(spin_deg=cb["spin"], perch_mm={"pred": cb["h_b_mm"], "measured": None},
                   descent_start_mm=round(cb["h_start"] * 1e3),
                   roll_pred={"sign": "mouth-down" if cb["mouth_down"] else "mouth-up", "com_offset_mm": cb["com_mouth_mm"],
                              "com_lateral_mm": cb["com_lateral_mm"], "contact": cb["target"], "part": cb["part"],
                              "rule": "the bowl's uniform-density centre of mass minus the Kit-free FCL contact point, along the "
                                      "goal axis's horizontal direction (+ = on the mouth side: the static roll turns the mouth "
                                      "down)"})
        stop = lower_until_contact(oid, cb)
        post["carry"] = False
        row.update(stop=stop, descent_mm=stop["descent_mm"], weld_drift_mm=stop["weld_drift_mm"])
        row["perch_mm"]["measured"] = stop["bowl_h_mm"]
        row["perch_mm"]["diff"] = round(stop["bowl_h_mm"] - cb["h_b_mm"], 1)
        T_r = T_now(oid)
        if roll:
            jaw_watch.update(on=True, oid=oid, hits=[])
        row["release_pose"] = release(oid, back=True)
        if roll:
            return roll_finish(i, oid, row, landed, before, cb, T_r)
        row["retreat"] = {**retreat, "rule": f"the jaws opened where the descent stopped; back up to {RELEASE_BACK_MAX_M * 1e3:.0f} "
                                             f"mm along -TCP z through the collision-checked line ({BACK_STEP_M * 1e3:.0f} mm "
                                             f"waypoints, 15 mm margin, the released bowl a scene dish; the gripper may touch it in "
                                             f"the first {ALLOW_CONTACT_M * 1e3:.0f} mm), truncated at the last free waypoint; then "
                                             f"up {flags.hover_m * 1e3:.0f} mm in world z only when the open gripper hulls are >= "
                                             f"{RIM_CLEAR_M * 1e3:.0f} mm from the bowl (True = followed, False = blocked)"}
        s_ = ns.bench_settle(oid, "goal", T_r, before)
        T = T_now(oid)
        st = s_["settle"]
        settled = st["drift_mm"] <= ns.R.STABLE_POS_M * 1e3 and st["drift_deg"] <= ns.R.STABLE_ROT_DEG
        dist = s_["distance"]
        row.update(settled=bool(settled), settle=st, final_pose=pose(T), mouth="up" if T[2, 2] > 0 else "down",
                   axis_z=round(float(T[2, 2]), 3), error={"lateral_mm": dist[0], "dz_mm": dist[1], "tilt_deg": dist[2]},
                   lean_deg={"final": lean_deg(T, np), "goal": lean_deg(goal[oid], np)}, lean_from_goal_deg=dist[2],
                   landed_on=sorted(b if a == oid else a for a, b in harness.contacts.pairs_with(oid)),
                   moved_earlier=[m for m in s_["rules"]["strict"] if m[0] in landed])
        landed.append(oid)
        if not settled:
            fail(row, "not settled", f"drift {st['drift_mm']} mm / {st['drift_deg']} deg over the last {K.WINDOW_TICKS} "
                                     f"ticks (limits 5 mm / 3 deg)")
        elif T[2, 2] >= 0:
            fail(row, "mouth up", f"settled mouth up (axis z {T[2, 2]:.2f})")

    def roll_finish(i, oid, row, landed, before, cb, T_r):
        """D-roll (plans/2026-10-29-perch-roll.md) after release(back=True): round 13's rest wait, the benchmark's settle, the
        judgement with the hand away: seated on the goal's wires (the fixed bowl test) AND mouth down; AT_GOAL and the roll vs
        the static prediction recorded; any gripper (pad, finger, palm) vs bowl contact from the end of the open ramp + 50 ms on
        fails the row by rule ("jaw contact"), the physical outcome recorded beside it."""
        kind = kinds[oid]
        row["retreat"] = {**retreat, "rule": "the jaws opened where the descent stopped; the I3 back-out along -TCP z through the "
                                             "checked line, the rise only clear of the rim (release(back=True))"}
        rest = rest_wait([oid])
        s_ = ns.bench_settle(oid, "goal", T_r, before)
        jaw_watch["on"] = False
        T = T_now(oid)
        st = s_["settle"]
        settled = st["drift_mm"] <= ns.R.STABLE_POS_M * 1e3 and st["drift_deg"] <= ns.R.STABLE_ROT_DEG
        dist = s_["distance"]
        sync_world()
        rk = meta[oid]["rack"]
        now_ = backend.poses()
        F_r = T_of(now_[rk]["position_m"], now_[rk]["quaternion_xyzw"])
        sw = bowl_seated_test(arm, kind, oid, T, oid, np, T_goal=F_r @ inv(F_goal[rk]) @ goal[oid], centroid=ns.centroids[kind])
        mouth = "up" if T[2, 2] > 0 else "down"
        t_after = (retreat.get("open_start_tick") or 0) + OPEN_STEPS_TICKS + JAW_GRACE_TICKS
        hits = list(jaw_watch["hits"])
        after = [h for h in hits if h[0] > t_after]
        during = [h for h in hits if h[0] <= t_after]
        lean_f = lean_deg(T, np)
        roll_m = "mouth-down" if lean_f < cb["theta"] - 2. else "mouth-up" if lean_f > cb["theta"] + 2. else "stays"
        row.update(settled=bool(settled), settle=st, rest_wait=rest, final_pose=pose(T), mouth=mouth, axis_z=round(float(T[2, 2]), 3),
                   error={"lateral_mm": dist[0], "dz_mm": dist[1], "tilt_deg": dist[2]},
                   at_goal=bool(ns.B.within_goal(kind, (dist[0] / 1e3, dist[1] / 1e3, dist[2]))),
                   lean_deg={"final": lean_f, "goal": lean_deg(goal[oid], np)}, lean_from_goal_deg=dist[2],
                   seated={**sw, "seated": bool(sw["seated_wires"] and T[2, 2] < 0)},
                   roll_measured=roll_m, roll_agrees=None if roll_m == "stays" else bool(
                       roll_m == ("mouth-down" if cb["mouth_down"] else "mouth-up")),
                   jaw_contact={"after_open": sorted({h[1] for h in after}), "after_open_ticks": len(after),
                                "first_after_tick": after[0][0] - (retreat.get("open_start_tick") or 0) if after else None,
                                "peak_N": max((h[2] for h in after), default=0.), "during_open": sorted({h[1] for h in during}),
                                "rule": f"gripper (pad, finger, palm) vs bowl contacts every tick from the open on; 'after the "
                                        f"open' = after the {OPEN_STEPS_TICKS}-tick open ramp + {JAW_GRACE_TICKS} ticks"},
                   landed_on=sorted(b if a == oid else a for a, b in harness.contacts.pairs_with(oid)),
                   moved_earlier=[m for m in s_["rules"]["strict"] if m[0] in landed])
        landed.append(oid)
        outcome = (f"physically: {'settled' if settled else 'not settled'}, mouth {mouth}, "
                   f"{'seated' if row['seated']['seated'] else 'not seated'}, {'at goal' if row['at_goal'] else 'off goal'}")
        if after:
            fail(row, "jaw contact", f"the gripper touched the bowl after the jaws opened ({', '.join(sorted({h[1] for h in after}))}, "
                                     f"{len(after)} ticks, peak {row['jaw_contact']['peak_N']} N); {outcome}")
        elif not settled:
            fail(row, "not settled", f"drift {st['drift_mm']} mm / {st['drift_deg']} deg over the last {K.WINDOW_TICKS} ticks; {outcome}")
        elif not row["seated"]["seated"]:
            fail(row, "not seated", f"mouth {mouth}, wires: bottom {sw.get('bottom_nearest')}, tine {sw.get('tine_nearest')}; {outcome}")

    # ---------------------------------------------------------------- D-seat (plans/2026-10-29-bowl-last.md, --carry seat)
    def rest_wait(dishes, cap=REST_CAP):
        """Step until none of ``dishes`` moved more than REST_TOL (m, deg) over REST_EVERY ticks, at most ``cap`` ticks."""
        if not dishes:
            return {"ticks": 0}
        Tp = lambda p: T_of(p["position_m"], p["quaternion_xyzw"])
        prev, n, mv, rt = backend.poses(), 0, None, None
        while n < cap:
            rig.step(REST_EVERY)
            n += REST_EVERY
            now = backend.poses()
            mv = max(float(np.linalg.norm(np.subtract(now[d]["position_m"], prev[d]["position_m"]))) for d in dishes)
            rt = max(rot_deg(Tp(now[d]), Tp(prev[d])) for d in dishes)
            prev = now
            if mv <= REST_TOL[0] and rt <= REST_TOL[1]:
                break
        return {"ticks": n, "at_rest": bool(mv is not None and mv <= REST_TOL[0] and rt <= REST_TOL[1]),
                "last_move_mm": None if mv is None else round(mv * 1e3, 2), "last_turn_deg": None if rt is None else round(rt, 2)}

    def predict_stop(oid, T_tcp, T_dish, closed):
        """The stop re-predicted from a measured pose: the welded bowl (at T_dish) and the tool (closed jaws + wrist 3, at
        T_tcp) lowered straight down together; heights = the bowl's height above its goal (centroid z). (h_stop, body,
        target, h_bowl, h_tool, h_now)"""
        kind = kinds[oid]
        c_ = np.r_[ns.centroids[kind], 1.]
        h_now = float((np.asarray(T_dish) @ c_)[2]) - float((goal[oid] @ c_)[2])
        sync_world()
        targets = scene_targets(arm, ignore=(oid,))

        def shift(T, h):
            T2 = np.array(T, dtype=float, copy=True)
            T2[2, 3] += h - h_now
            return T2
        rb = perch_descent(arm, kind, lambda h: shift(T_dish, h), oid, [t for t in targets if t[0] not in arm.statics], np,
                           h_top=h_now)
        rt_ = tool_descent(arm, lambda h: shift(T_tcp, h), tool_hulls(arm, closed), targets, np, h_top=h_now,
                           h_floor=(rb[0] if rb else PERCH_H_FLOOR_M) - 1e-6)
        h_b = rb[0] if rb else None
        hand = rt_ is not None and (h_b is None or rt_[0] > h_b + 1e-9)
        if h_b is None and not hand:
            return None, None, None, None, None, h_now
        return ((rt_[0], "hand", rt_[1]) if hand else (h_b, "bowl", rb[1])) + (h_b, rt_[0] if rt_ else None, h_now)

    def hand_away(oid):
        """After the release, once the bowl is at rest: the first of straight up, -approach, +-mouth, +-rim tangent (AWAY_STEP_M
        each, AWAY_WP_M waypoints, IK-continuous) along which the open gripper and the links never come closer to the scene
        (the resting bowl excluded) than min(AWAY_RACK_M, their clearance at the release pose) and which ends with the open
        gripper AWAY_BOWL_M or more from the resting bowl (Kit-free, the bowl's Isaac pose); executed, then the hover rise when
        that line keeps the same clearance rule. None passing: "hand stuck"."""
        rest = rest_wait([oid])
        sync_world()
        T_b0, T0, q0 = T_now(oid), rig.tcp(), rig.q()
        targets = scene_targets(arm, ignore=(oid,))
        bowl_t = [(oid, arm.dishes[oid])] if oid in arm.dishes else []
        a_m = T_b0[:3, 2] / np.linalg.norm(T_b0[:3, 2])
        dirs = [("up", np.array([0., 0., 1.])), ("-approach", -T0[:3, 2]), ("+mouth", a_m), ("-mouth", -a_m),
                ("+tangent", T0[:3, 0]), ("-tangent", -T0[:3, 0])]

        def clear_at(q):                               # signed: FCL distance, minus the penetration depth when touching
            best = np.inf
            for _, o, lo, hi in arm.arm_objects(q, ("shoulder_link",)):
                for _, b in targets:
                    if _box_gap(lo, hi, b, np) > .05:
                        continue
                    best = min(best, depth_of(arm.fcl, b["manager"], o))
            return best

        def bowl_gap(T_tcp):
            if not bowl_t:
                return np.inf
            return min_clear(arm.fcl, hull_objs(arm.fcl, [("open", g, v) for g, v in arm.hulls["gripper"]], T_tcp, arm.T_w3_tcp, np),
                             bowl_t, np, cutoff=.05)[0]
        c0 = clear_at(q0)                              # already inside the 15 mm margin: no closer than 2 mm under the start
        floor_ = AWAY_RACK_M if not np.isfinite(c0) or c0 >= AWAY_RACK_M else c0 - AWAY_DIP_M
        tested, chosen = [], None
        for name, d in dirs:
            n = int(round(AWAY_STEP_M / AWAY_WP_M))
            prev, ok, why, qs = q0, True, None, []
            for j in range(1, n + 1):
                T = T0.copy()
                T[:3, 3] += d * AWAY_WP_M * j
                q = rig.ik(T, prev)
                if q is None or float(np.abs(q - prev).max()) > IK_JUMP_MAX_RAD:
                    ok, why = False, f"IK branch jump at {j * AWAY_WP_M * 1e3:.0f} mm"
                    break
                cj = clear_at(q)
                if cj < floor_ - 1e-4:
                    ok, why = False, f"{cj * 1e3:.1f} mm from the scene at {j * AWAY_WP_M * 1e3:.0f} mm (floor {floor_ * 1e3:.1f})"
                    break
                qs.append(q)
                prev = q
            T_e = T0.copy()
            T_e[:3, 3] += d * AWAY_STEP_M
            gap = bowl_gap(T_e) if ok else np.inf
            if ok and gap < AWAY_BOWL_M:
                ok, why = False, f"open gripper {gap * 1e3:.1f} mm from the bowl at the end"
            tested.append({"direction": name, "ok": bool(ok), "why": why,
                           "bowl_gap_mm": None if not np.isfinite(gap) else round(float(gap) * 1e3, 1)})
            if ok:
                chosen = (name, d)
                break
        out = {"rest": rest, "start_clear_mm": None if not np.isfinite(c0) else round(float(c0) * 1e3, 1), "tested": tested,
               "direction": None if chosen is None else chosen[0]}
        if chosen is None:
            out.update(hand_away=False, outcome="hand stuck")
            return out
        T_end = T0.copy()
        T_end[:3, 3] += chosen[1] * AWAY_STEP_M
        line = rig.line_q(T0, T_end, int(round(AWAY_STEP_M / AWAY_WP_M)) + 1, q0)
        out["followed"] = None if line is None else bool(rig.follow(line))
        up = rig.tcp()
        T1 = up.copy()
        up[2, 3] += flags.hover_m
        rq = rig.line_q(T1, up, 40, rig.q())
        rise_ok = rq is not None and all(clear_at(q) >= min(AWAY_RACK_M, floor_) - 1e-4 for q in rq[::4])
        out["rise"] = (bool(rig.follow(rq)) if rise_ok else "skipped: the rise comes within the clearance floor")
        T_b1 = T_now(oid)
        out.update(hand_away=True, outcome="hand away",
                   moved_mm=round(float(np.linalg.norm(T_b1[:3, 3] - T_b0[:3, 3])) * 1e3, 1),
                   moved_deg=round(rot_deg(T_b1, T_b0), 1))
        out["disturbed"] = bool(out["moved_mm"] > DISTURB_MM or out["moved_deg"] > DISTURB_DEG)
        return out

    def seat_drop(i, oid, row, landed):
        """--carry seat (rows D-seat): reset()'s seat_bowl_search combos (up to DROP_TRIES, the arm on their home branch)
        carried by transit_to to the descent start; the arm holds its targets until the lag is under PRE_DESCENT_LAG_RAD (at
        most PRE_DESCENT_CAP ticks); the stop re-predicted from the measured TCP and bowl poses (predict_stop); lowered by
        lower_until_contact from the measured pose, bounded 30 mm under the re-predicted stop; let go where it stopped (joint
        removed, jaws opened); hand_away; the benchmark's settle judged only after that; seated = bowl_seated_test AND mouth
        down. No passing combo: "no reach" (the bowl is parked)."""
        kind, M = kinds[oid], att["M"]
        kw = {"held": (kind, M, oid), "ignore": (oid,)}
        before = backend.poses()
        sync_world()
        homes = carry_state.get("homes") or []
        row["tilt_deg"] = {"goal": round(float(np.degrees(np.arccos(np.clip(-goal[oid][2, 2], -1., 1.)))), 1), "carried": None}
        post.update(carry=True, start=rig.q())
        tries, found, fails = 0, None, []
        why = f"no passing combo: {json.dumps(carry_state.get('counts'))}; {json.dumps(carry_state.get('first_fail'))}"
        for cb in (carry_state.get("ranked") or [])[:DROP_TRIES]:
            if not arm.free(cb["q_start"], **kw):
                continue
            tries += 1
            tlog.set(phase="carry to the descent start")
            ns.plan_log.clear()
            w_ = transit_to(cb["T_start"], q_hover=cb["q_start"], **kw)
            if w_ is None:
                found = cb
                row.update(combo=seat_rec(cb, homes, np), combo_rank=tries, legs={"plans": list(ns.plan_log)})
                break
            fails.append(f"tilt {cb['theta']:.1f} spin {cb['spin']:+.0f} +{cb['h_start'] * 1e3:.0f} mm: {w_} (legs {list(ns.plan_log)})")
            why = f"no path to {tries} descent start(s), the last: {fails[-1]}"
        row.update(drop_tries=tries, carry_fails=fails)
        if found is None:
            post["carry"] = False
            harness.armed = False
            detach()
            harness.clear_carried()
            backend.set_rigid_pose(backend.objects[oid], park_pose(i))
            return fail(row, "no reach", why)
        cb = found
        qs_ = row["reset"]["q_start_deg"]
        row["start_branch"] = {"q_start_deg": qs_, "wrist_2_deg": qs_[4], "wrist_3_deg": qs_[5]}
        row["tilt_deg"]["carried"] = round(cb["theta"], 1)
        row.update(spin_deg=cb["spin"], descent_start_mm=round(cb["h_start"] * 1e3),
                   stop_pred={"kind": cb["body"], "height_mm": cb["stop_mm"], "target": cb["stop_target"],
                              "perch_mm": cb["h_b_mm"]},
                   roll_pred={"sign": "mouth-down" if cb["mouth_down"] else "mouth-up", "com_offset_mm": cb["com_mouth_mm"],
                              "com_lateral_mm": cb["com_lateral_mm"], "contact": cb["target"], "part": cb["part"],
                              "rule": "the bowl's uniform-density centre of mass minus the Kit-free contact of the bowl alone "
                                      "lowered at the carried tilt, along the goal axis's horizontal direction (+ = the static "
                                      "roll turns the mouth down)"})
        n_hold, lag0 = 0, rig.lag()                    # the pre-descent settle
        while rig.lag() > PRE_DESCENT_LAG_RAD and n_hold < PRE_DESCENT_CAP:
            rig.step(1)
            n_hold += 1
        T_m, T_b = rig.tcp(), T_now(oid)
        h_s, body_s, tgt_s, h_bw, h_tl, h_now = predict_stop(oid, T_m, T_b, closed_hulls())
        row["pre_descent"] = {"ticks": n_hold, "lag_before_rad": round(float(lag0), 4), "lag_after_rad": round(float(rig.lag()), 4),
                              "tcp_vs_plan_mm": round(float(np.linalg.norm(T_m[:3, 3] - np.asarray(cb["T_start"])[:3, 3])) * 1e3, 1),
                              "bowl_h_mm": round(h_now * 1e3, 1)}
        row["stop_pred"]["re_pred"] = {"kind": body_s, "height_mm": None if h_s is None else round(h_s * 1e3, 1),
                                       "target": tgt_s, "bowl_mm": None if h_bw is None else round(h_bw * 1e3, 1),
                                       "tool_mm": None if h_tl is None else round(h_tl * 1e3, 1),
                                       "rule": "the welded bowl and the tool (closed jaws + wrist 3) lowered straight down from "
                                               "the measured TCP / bowl poses after the pre-descent settle"}
        h_ref = h_s if h_s is not None else cb["stop"]
        stop = lower_until_contact(oid, {"T_start": T_m, "q_start": rig.q(), "h_start": h_now, "h_b": h_ref})
        post["carry"] = False
        meas = stop["bowl_h_mm"]
        pred = None if h_s is None else round(h_s * 1e3, 1)
        row.update(stop={**stop, "height_mm": meas}, descent_mm=stop["descent_mm"], weld_drift_mm=stop["weld_drift_mm"],
                   stop_check={"pred_mm": pred, "pred_kind": body_s, "measured_mm": meas, "measured_kind": stop["kind"],
                               "within_2mm": None if pred is None or stop["kind"] == "none" else bool(abs(meas - pred) <= 2.),
                               "body_match": None if stop["kind"] in ("none", "predicted") else bool(stop["kind"] == body_s)})
        T_r = T_now(oid)
        row["release_pose"] = pose(T_r)
        detach()
        harness.clear_carried()
        tlog.set(phase="release")
        rig.gripper(False)
        tlog.set(phase="hand away")
        row["retreat"] = hand_away(oid)
        s_ = ns.bench_settle(oid, "goal", T_r, before)
        T = T_now(oid)
        st = s_["settle"]
        settled = st["drift_mm"] <= ns.R.STABLE_POS_M * 1e3 and st["drift_deg"] <= ns.R.STABLE_ROT_DEG
        dist = s_["distance"]
        sync_world()
        now_ = backend.poses()
        F_r = T_of(now_[meta[oid]["rack"]]["position_m"], now_[meta[oid]["rack"]]["quaternion_xyzw"])
        sw = bowl_seated_test(arm, kind, oid, T, oid, np, T_goal=F_r @ inv(F_goal[meta[oid]["rack"]]) @ goal[oid],
                              centroid=ns.centroids[kind])
        away = bool(row["retreat"].get("hand_away"))
        row.update(settled=bool(settled), settle=st, final_pose=pose(T), mouth="up" if T[2, 2] > 0 else "down",
                   axis_z=round(float(T[2, 2]), 3), error={"lateral_mm": dist[0], "dz_mm": dist[1], "tilt_deg": dist[2]},
                   lean_deg={"final": lean_deg(T, np), "goal": lean_deg(goal[oid], np)}, lean_from_goal_deg=dist[2],
                   seated={**sw, "seated": bool(sw["seated_wires"] and T[2, 2] < 0)}, hand_away=away,
                   landed_on=sorted(b if a == oid else a for a, b in harness.contacts.pairs_with(oid)),
                   moved_earlier=[m for m in s_["rules"]["strict"] if m[0] in landed])
        landed.append(oid)
        if not away:
            fail(row, "hand stuck", f"no retreat direction separates the hand: {row['retreat']['tested']}")
        elif not settled:
            fail(row, "not settled", f"drift {st['drift_mm']} mm / {st['drift_deg']} deg over the last {K.WINDOW_TICKS} "
                                     f"ticks (limits 5 mm / 3 deg)")
        elif not row["seated"]["seated"]:
            fail(row, "not seated", f"mouth {row['mouth']}, wires: bottom {sw['bottom_nearest']}, tine {sw['tine_nearest']}")

    # ---------------------------------------------------------------- D-flip and I (plans/2026-10-27-bowls-plates-down.md)
    def first_drop(R, m_c, g_c, kw):
        """The drop search's first candidate for TCP orientation R: the lowest height with a collision-free IK, there the
        yaw with the smallest joint move: (h, k, q) or None."""
        q0_ = rig.q()
        for h in DROP_HEIGHTS_M:
            cand = []
            for k in range(12):
                qs = goal_qs(drop_tcp(R, m_c, g_c, h, k, np), **kw)
                if qs:
                    cand.append((float(np.abs(qs[0] - q0_).max()), k, qs[0]))
            if cand:
                _, k, q = min(cand, key=lambda x: x[:2])
                return h, k, q
        return None

    def choose_flip(oid, row, R0, M, m_c, g_c, kw):
        """The tilt scan: per tilt (FLIP_TILTS_DEG, both turn directions) the Kit-free fall check -- the bowl released at
        that orientation and moved 0-50 mm straight down must not touch the open gripper or wrist_3 (rigid with the TCP),
        nor wrist_1 / wrist_2 at the drop posture (the drop search's first pose) -- the first clearing tilt wins (within it
        the larger clearance); none clearing: the full flip, flagged palm. Returns the TCP orientation for the drop."""
        a_tcp, scan, seen, chosen = M[:3, 2], [], {}, None
        for t in FLIP_TILTS_DEG:
            recs = []
            for R_, turn, ax in flip_orients(R0, a_tcp, t, np):
                if turn in seen:                           # below the smallest reachable tilt: the full flip again
                    scan.append({"tilt_deg": t, "turn_deg": turn, "axis_tilt_deg": ax, "same_as_tilt_deg": seen[turn]})
                    continue
                seen[turn] = t
                fc = arm.fall_clearance(drop_tcp(R_, m_c, g_c, DROP_HEIGHTS_M[0], 0, np), kw["held"], FALL_DROPS_M)
                recs.append(({"tilt_deg": t, "turn_deg": turn, "axis_tilt_deg": ax, "gripper_wrist3": fc,
                              "clear": fc["min_mm"] > 0.}, R_))
            for rec, R_ in sorted(recs, key=lambda x: -x[0]["gripper_wrist3"]["min_mm"]):
                if rec["clear"] and chosen is None:        # the wrists 1-2 at the drop posture
                    first = first_drop(R_, m_c, g_c, kw)
                    if first is None:
                        rec["drop_posture"] = "no collision-free drop IK at this tilt (wrists 1-2 not checked)"
                    else:
                        fq = arm.fall_clearance(drop_tcp(R_, m_c, g_c, first[0], first[1], np), kw["held"], FALL_DROPS_M,
                                                q=first[2])
                        rec.update(drop_posture={"height_m": first[0], "yaw_deg": 30 * first[1]}, at_drop_posture=fq,
                                   clear=fq["min_mm"] > 0.)
                    if rec["clear"]:
                        chosen = (rec, R_)
                scan.append(rec)
            if chosen:
                break
        full = None
        if chosen is None:                                 # none clears: the full flip (closest to straight down), palm
            R_, turn, ax = flip_orients(R0, a_tcp, 0., np)[0]
            full = {"tilt_deg": 0., "turn_deg": turn, "axis_tilt_deg": ax}
        rec, R_ = chosen if chosen else (full, R_)
        row.update(flip_tilt_deg=rec["tilt_deg"], flip_clear=chosen is not None, flip_turn_deg=rec["turn_deg"],
                   flip_axis_tilt_deg=rec["axis_tilt_deg"], flip_scan=scan,
                   flip_note=None if chosen else "palm: no tilt's first 50 mm of fall clears the gripper; let go from the "
                                                 "full flip (the bowl axis closest to straight down)")
        print(f"[INFO] {oid} flip: tilt {rec['tilt_deg']} (turn {rec['turn_deg']} deg, axis {rec['axis_tilt_deg']} deg from "
              f"down) clear {chosen is not None}; scan " + "; ".join(
                  f"{s['tilt_deg']:.0f}/{s['turn_deg']:+.0f}: " + (f"same as {s['same_as_tilt_deg']:.0f}" if "same_as_tilt_deg" in s
                  else f"{s['gripper_wrist3']['min_mm']} mm ({s['gripper_wrist3']['link']} at {s['gripper_wrist3']['at_drop_mm']} mm)"
                  + (f", wrists at the drop pose {s['at_drop_posture']['min_mm']} mm ({s['at_drop_posture']['link']})"
                     if isinstance(s.get("at_drop_posture"), dict) else "")) for s in scan), flush=True)
        return R_

    def in_slot(oid, T):
        """Variant I: the plate between its slot's tine pair. The slot lower_<bank>_<g> is the gap between tine columns g
        and g+1 of the bank's two rows (geometry.lower_tine_positions, rack-local; the tines lean toward +x by
        tine_tip_offset_x over their height). In slot = the plate's visual points within 5 mm (y) of either row and below
        its tines' tops exist, and every one lies between the two columns' tine axes at its own height (+- the tine radius)."""
        from dishsim_frigidaire.geometry import PARAMETERS as GP, lower_tine_heights, lower_tine_positions
        slot = meta[oid]["slot"]
        _, bank, g = slot.split("_")
        g = int(g)
        rows_ = (0, 1) if bank == "front" else (-2, -1)
        xs, ys = lower_tine_positions()
        hts, lr = lower_tine_heights(), GP["lower_rack"]
        now = backend.poses()
        F = T_of(now["LowerRack"]["position_m"], now["LowerRack"]["quaternion_xyzw"])
        pts = np.asarray(ns.points[kinds[oid]], dtype=float)
        P_ = (inv(F) @ np.asarray(T) @ np.c_[pts, np.ones(len(pts))].T).T[:, :3]
        sel, top = np.zeros(len(P_), bool), np.zeros(len(P_))
        for r in rows_:
            m_ = (np.abs(P_[:, 1] - ys[r]) < .005) & (P_[:, 2] < lr["tine_base_z"] + hts[r])
            sel |= m_
            top[m_] = hts[r]
        info = {"slot": slot, "columns_x_mm": [round(xs[g] * 1e3, 1), round(xs[g + 1] * 1e3, 1)], "points": int(sel.sum()),
                "rule": "plate points within 5 mm of the slot's two tine rows and below the tine tops all between the "
                        "column pair's tine axes at their height (+- the tine radius)"}
        if not sel.any():
            return False, {**info, "why": "no plate point below the tine tops at the slot's rows"}
        lean = lr["tine_tip_offset_x"] * np.clip((P_[sel, 2] - lr["tine_base_z"]) / top[sel], 0., 1.)
        x, tol = P_[sel, 0], lr["tine_diameter"] / 2    # a point beside a tine (within the 5 mm row band) may pass its axis
        ok = bool(np.all(x > xs[g] + lean - tol) and np.all(x < xs[g + 1] + lean + tol))
        info.update(plate_x_mm=[round(float(x.min()) * 1e3, 1), round(float(x.max()) * 1e3, 1)])
        return ok, info

    def insert(i, oid, row, landed):
        """Variant I: the plate welded at the assumed top-edge rim hold, carried at its goal's exact orientation (lean and
        slot yaw) to the lowest collision-free pose 25-350 mm above its goal (both gripper sides per height, the smaller
        joint move first; transit_to, DROP_TRIES poses with an IK, else no reach), lowered along the vertical in 5 mm steps
        (rig.follow, the harness armed) until the plate touches the rack or a dish (the harness contact report), the arm is
        blocked (rig.follow fails) or the goal height is reached; then the joint removed, the jaws opened, the hand backed
        25 mm along its approach and raised 0.10 m (release(back=True)), and the benchmark's settle; success = settled. A
        face switch re-welds the plate at home (reset() parks it before the jaws open). No reach with no free IK on either
        face: the arm model's blocking pairs per face in the row (plans/2026-10-27-bowls-lip-plates-fix.md)."""
        kind, T_g, holds = kinds[oid], goal[oid], row.pop("_holds")
        before = backend.poses()
        sync_world()
        tries, found = 0, None
        why = f"no collision-free IK at any height ({DROP_HEIGHTS_M[0]}-{DROP_HEIGHTS_M[-1]} m above the goal) on either side"
        post.update(carry=True, start=rig.q())
        for h in DROP_HEIGHTS_M:
            T_p = np.array(T_g, copy=True)
            T_p[2, 3] += h
            q0_ = rig.q()
            cand = []
            for side in PLATE_SIDES:
                qs = goal_qs(T_p @ inv(holds[side]), held=(kind, holds[side], oid), ignore=(oid,))
                if qs:
                    cand.append((float(np.abs(qs[0] - q0_).max()), side))
            for move, side in sorted(cand):
                M = holds[side]
                kw_s = {"held": (kind, M, oid), "ignore": (oid,)}
                if side != row["side"]:                    # the other gripper side: re-welded at its hold, from home
                    row["reset"] = reset(i, oid, M, keep=list(landed))
                    row.update(side=side, side_switches=row["side_switches"] + 1)
                    post.update(carry=True, start=rig.q())
                T_t = T_p @ inv(M)
                qs = goal_qs(T_t, **kw_s)
                if not qs:
                    continue
                tries += 1
                tlog.set(phase="carry to the insertion pose")
                w_ = transit_to(T_t, q_hover=qs[0], **kw_s)
                if w_ is None:
                    found = (h, side, T_t)
                    row["carry_move_deg"] = round(float(np.degrees(move)), 1)
                    break
                why = f"no path to {tries} pose(s), the last at +{h:.3f} m with the gripper on the {side} face: {w_}"
                if tries >= DROP_TRIES:
                    break
            if found or tries >= DROP_TRIES:
                break
        row["drop_tries"] = tries
        if found is None:
            post["carry"] = False
            harness.armed = False                          # never inserted: the plate leaves the scene
            detach()
            harness.clear_carried()
            backend.set_rigid_pose(backend.objects[oid], park_pose(i))
            if tries == 0:                                 # no free IK on either face: what the arm model says blocks it
                row["blocking"] = {}
                for side in PLATE_SIDES:
                    kw_s = {"held": (kind, holds[side], oid), "ignore": (oid,)}
                    poses = []
                    for h in DROP_HEIGHTS_M:
                        T_p = np.array(T_g, copy=True)
                        T_p[2, 3] += h
                        poses.append((f"+{h * 1e3:.0f} mm", T_p @ inv(holds[side]), kw_s))
                    row["blocking"][side] = blocking_pairs(rig, arm, poses, np)
            return fail(row, "no reach", why)
        h, side, T_t = found
        row.update(start_height_m=h, in_hand={**row["in_hand"], "side": side})
        tlog.set(phase="insertion: lowering")
        stop, low, hit, n = "goal", 0, [], int(round(h / LOWER_STEP_M))
        for j in range(1, n + 1):
            T0_, T1_ = T_t.copy(), T_t.copy()
            T0_[2, 3] -= LOWER_STEP_M * (j - 1)
            T1_[2, 3] -= LOWER_STEP_M * j
            mark = backend.tick_index
            qs = rig.line_q(T0_, T1_, 3, rig.q())
            ok = qs is not None and rig.follow(qs)
            low = j
            hit = sorted({b if a == oid else a for (a, b), act in harness.contacts.pairs_with(oid).items()
                          if act["last"] >= mark and (act["cats"][1] if a == oid else act["cats"][0]) in ("rack", "dish")})
            if hit:
                stop = "contact"
                break
            if not ok:
                stop = "blocked"
                row["blocked"] = "IK branch jump on the 5 mm step" if qs is None else getattr(rig, "last_fail", None)
                break
        post["carry"] = False
        row.update(lowered_mm=round(low * LOWER_STEP_M * 1e3), stop_reason=stop, stop_partners=hit,
                   tcp_lowered_mm=round(float(T_t[2, 3] - rig.tcp()[2, 3]) * 1e3, 1))
        T_r = T_now(oid)
        row["release_pose"] = release(oid, back=True)
        row["retreat"] = {**retreat, "rule": f"back up to {RELEASE_BACK_MAX_M * 1e3:.0f} mm along -TCP z through the "
                                             f"collision-checked line ({BACK_STEP_M * 1e3:.0f} mm waypoints, 15 mm margin, the "
                                             f"released plate a scene dish; the gripper may touch it in the first "
                                             f"{ALLOW_CONTACT_M * 1e3:.0f} mm), truncated at the last free waypoint; then up "
                                             f"{flags.hover_m * 1e3:.0f} mm in world z only when the open gripper hulls are "
                                             f">= {RIM_CLEAR_M * 1e3:.0f} mm from the plate (True = followed, False = blocked)"}
        s_ = ns.bench_settle(oid, "goal", T_r, before)
        T = T_now(oid)
        st = s_["settle"]
        settled = st["drift_mm"] <= ns.R.STABLE_POS_M * 1e3 and st["drift_deg"] <= ns.R.STABLE_ROT_DEG
        dist = s_["distance"]
        lean = lambda T_: round(float(np.degrees(np.arcsin(np.clip(T_[2, 2], -1., 1.)))), 1)   # + = mouth tilted up
        ok_slot, slot_info = in_slot(oid, T)
        row.update(settled=bool(settled), settle=st, final_pose=pose(T), axis_z=round(float(T[2, 2]), 3),
                   error={"lateral_mm": dist[0], "dz_mm": dist[1], "tilt_deg": dist[2]}, lean_from_goal_deg=dist[2],
                   lean_deg={"final": lean(T), "goal": lean(T_g)}, in_slot=ok_slot, in_slot_detail=slot_info,
                   landed_on=sorted(b if a == oid else a for a, b in harness.contacts.pairs_with(oid)),
                   moved_earlier=[m for m in s_["rules"]["strict"] if m[0] in landed])
        landed.append(oid)
        if not settled:
            fail(row, "not settled", f"drift {st['drift_mm']} mm / {st['drift_deg']} deg over the last {K.WINDOW_TICKS} "
                                     f"ticks (limits 5 mm / 3 deg)")

    # ---------------------------------------------------------------- I4 (plans/2026-10-28-plate-seat.md, --insert seat)
    _seat = {}

    def seat_lib():
        if not _seat:
            run_, traj_, src_ = (gate_trial(out / PLATE_EDGE_GATE_RUN, fallback=PLATE_GATE) if args.grasp_family == "edge" else
                                 gate_trial(out / PLATE_WALL_GATE_RUN, fallback=PLATE_GATE) if args.grasp_family == "plate-wall"
                                 else (PLATE_GATE[0], PLATE_GATE[1], f"{PLATE_GATE[0].name}/{PLATE_GATE[1]}"))
            _seat.update(labels=rack_labels(len(arm.checker.parts["LowerRack"])), closed=closed_hulls_named(C, np, run_, traj_),
                         closed_src=src_, open=[(f"open gripper hull {k}", g, v) for k, (g, v) in enumerate(arm.hulls["gripper"])])
            _seat["tool"] = tool_hulls(arm, [(g, v) for _, g, v in _seat["closed"]])   # the closed jaws + wrist 3 (D-seat's hand)
            if len(_seat["labels"]) != len(arm.checker.parts["LowerRack"]):
                raise RuntimeError(f"rack_names.json lists {len(_seat['labels'])} parts, the LowerRack has "
                                   f"{len(arm.checker.parts['LowerRack'])}")
        return _seat

    def pinch_spin(oid, roll=None):
        """The plate's pinch spin from the rim point highest at its goal (deg) and its source (plans/2026-11-04-fifteen-of-
        fifteen.md): the --place-goal load row's pinch_spin_deg (the access order's spin against the plate's neighbours), else
        the load's gate row for the held roll (plate_04's front spin, frigidaire_robot_reach.py --sample-gate), else the gate
        row's chosen spin, else 0. (spin_deg, source)"""
        if (meta.get(oid) or {}).get("pinch_spin_deg") is not None:
            return float(meta[oid]["pinch_spin_deg"]), "the load row's pinch_spin_deg"
        rel_ = (ns.gate_row(oid).get("release") or {})
        pr_ = (rel_.get("per_roll") or {}).get(str(roll)) or {}
        if pr_.get("pass"):
            return float(pr_["spin_deg"]), f"the gate row's release.per_roll[{roll}]"
        if rel_.get("pinch_spin_deg") is not None:
            return float(rel_["pinch_spin_deg"]), "the gate row's release.pinch_spin_deg"
        return 0., "0 (no pinch spin in the load row or the gate row)"

    def settle_landed(landed, max_ticks=240, every=30, tol=(.0005, .5)):
        """Before the Kit-free seat search: step until no landed dish moved more than tol (m, deg) over ``every`` ticks (round
        11's trap: a dish left on the open fingers is still falling after the reset's teleport)."""
        if not landed:
            return {"ticks": 0}
        prev, n, mv, rt = backend.poses(), 0, None, None
        Tp = lambda p: T_of(p["position_m"], p["quaternion_xyzw"])
        while n < max_ticks:
            rig.step(every)
            n += every
            now = backend.poses()
            mv = max(float(np.linalg.norm(np.subtract(now[d]["position_m"], prev[d]["position_m"]))) for d in landed)
            rt = max(rot_deg(Tp(now[d]), Tp(prev[d])) for d in landed)
            prev = now
            if mv <= tol[0] and rt <= tol[1]:
                break
        return {"ticks": n, "last_move_mm": round(mv * 1e3, 2), "last_turn_deg": round(rt, 2),
                "rule": f"{every}-tick windows until no landed dish moves > {tol[0] * 1e3:.1f} mm / {tol[1]} deg, at most "
                        f"{max_ticks} ticks"}

    def seat_plan(oid, M, homes):
        """--insert seat, Kit-free, in reset() (window 2 of plans/2026-10-28-plate-seat.md), the landed plates at rest. The plate
        alone lowered along the tine axis from its goal moved SEAT_DX_M toward the basket (seat_search; the earlier plates where
        they landed): the predicted seat; the closed / open gripper's clearance there; the descent start = the lowest of 50,
        55, .. 150 mm above the seat along the tine axis with a collision-free IK (15 mm arm / 2 mm held margins) continuous down
        to the bound; from it the straight line UP the tine axis to the TCP height z_safe (IK-continuous, free at the same
        margins): its top q_top; the home branch = the free branch whose leg 1 to q_top is a straight collision-free joint move,
        else the one whose Kit-free RRT-Connect (SEAT_PLAN_RRT_S, seed 1) finds a path, ties by the smallest largest-joint
        move. --attach none (plans/2026-11-04-fifteen-of-fifteen.md): D3, a seat more than SEAT_AT_GOAL_M from the goal along the
        tine axis or with a dish among its supports is a perch ("perch", nothing searched); D2 (args.plate_drop: I4-wall20-drop),
        when no start's full descent to the seat is clear: the tine-guided release's starts (DROP_RULE: the lowest release height
        down to which drop_line_hits stays clear, the plate's rim DROP_GUIDED_MIN_M under its slot's tine tips there; "drop").
        {"v", "lean_t", "sk", "seat_s", ...}"""
        kind, T_g = kinds[oid], goal[oid]
        kw = {"held": (kind, M, oid), "ignore": (oid,)}
        lib = seat_lib()
        sync_world()
        t0_ = time.monotonic()
        now = backend.poses()
        F = T_of(now["LowerRack"]["position_m"], now["LowerRack"]["quaternion_xyzw"])
        x_dir, v, lean_t = tine_axis(F, np)
        bias_m = descent_bias_mm(meta[oid]) / 1e3     # round 3: the load row's lateral bias (default SEAT_DX_M's 1 mm)
        T_c = np.array(T_g, copy=True)
        T_c[:3, 3] += bias_m * x_dir
        sk = seat_search(arm, kind, oid, T_c, v, scene_targets(arm, ignore=(oid,), statics=False), lib["labels"], np)
        out = {"v": v, "lean_t": lean_t, "sk": sk, "seat_s": round(time.monotonic() - t0_, 1), "homes": homes, "bias_m": bias_m}
        if sk.get("seat_mm") is None:
            return out
        T_seat = sk.pop("T_seat")
        c_ = np.r_[ns.centroids[kind], 1.]
        T_ts = T_seat @ inv(M)
        scene_h = scene_targets(arm, ignore=(oid,))
        fmt = lambda d: [round(max(float(d[0]), 0.) * 1e3, 1) if np.isfinite(d[0]) else None, d[1], d[2]]
        dishes_ = [t for t in scene_h if t[0] in arm.dishes]
        out.update(T_seat=T_seat, T_ts=T_ts, seat_z=round((float((T_seat @ c_)[2]) - float((T_g @ c_)[2])) * 1e3, 1),
                   cl={"closed": fmt(min_clear(arm.fcl, hull_objs(arm.fcl, lib["closed"], T_ts, arm.T_w3_tcp, np), scene_h, np)),
                       "open": fmt(min_clear(arm.fcl, hull_objs(arm.fcl, lib["open"], T_ts, arm.T_w3_tcp, np), scene_h, np)),
                       "closed_vs_dishes": fmt(min_clear(arm.fcl, hull_objs(arm.fcl, lib["closed"], T_ts, arm.T_w3_tcp, np),
                                                         dishes_, np)),
                       "rule": "FCL distance (mm, <= 0 = touching; None = nothing within 0.10 m) of the gripper hulls at the "
                               "predicted seat: closed = the jaws on a plate rim (gate2_plate t00, last frame), open = the "
                               "planner's open jaws; vs the scene (appliance, counter, pedestal, floor, other dishes) and vs "
                               "the other dishes only"})
        rule1 = att_mode["none"]                       # rule 1 + the I4 final-descent rule (plans/2026-11-04-plate-pick-presentation
        seat_ax = float(sk["seat_mm"]) + bias_m * float(x_dir @ v) * 1e3   # D3: the seat's offset from the goal along the axis
        on_ = list(sk.get("on_dish") or [])
        out["seat_at_goal"] = {"ok": bool(abs(seat_ax) <= SEAT_AT_GOAL_M * 1e3 and not on_), "seat_from_goal_mm": round(seat_ax, 1),
                               "on_dish": on_, "supports": sk.get("supports"), "limit_mm": round(SEAT_AT_GOAL_M * 1e3),
                               "rule": "D3 (plans/2026-11-04-fifteen-of-fifteen.md): a predicted seat counts only within "
                                       f"{SEAT_AT_GOAL_M * 1e3:.0f} mm of the goal along the tine axis and with rack wires as its "
                                       "only supports; else a perch, the move not attempted"}
        if rule1 and not out["seat_at_goal"]["ok"]:    # D3: perch predicted, no start searched
            out.update(perch=out["seat_at_goal"], plan_s=round(time.monotonic() - t0_, 1))
            return out
        tool_, tgt_ = (lib["tool"], scene_h) if rule1 else (None, None)   # .md amended): every start 50..150 mm that passes,
        cands, n_free, poses_tried, dbreaks = [], 0, [], {}                # the 2 mm rule on the line down to the seat
        drop_on = rule1 and bool(getattr(args, "plate_drop", False))       # D2: I4-wall20-drop (or the load row's release mode drop)
        dcands = []                                    # D2: (release mm above the seat, h_s, T_s, q, first violation) per start, branch
        h_s = SEAT_START_ABOVE_M
        while h_s <= SEAT_START_MAX_M + 1e-9 and (rule1 or not cands):
            T_s = T_ts.copy()
            T_s[:3, 3] += h_s * v
            poses_tried.append((f"+{h_s * 1e3:.0f} mm", T_s, kw))
            for q in [q for q in rig.ik_all(T_s, rig.q()) if arm.free(q, **kw)]:
                n_free += 1
                prev, brk = q, None
                n_dn = int(round(h_s * 1e3))
                d_rel, d_fv = None, None               # D2: the drop rule's lowest passing checkpoint (mm above the seat) and its
                if drop_on:                            # first violation [mm, pair], the start itself first
                    h0_ = drop_line_hits(arm, q, tool_, tgt_, np)
                    d_rel, d_fv = (None, [n_dn, h0_[0]]) if h0_ else (n_dn, None)
                for j in range(1, n_dn + int(round(DESCENT_BELOW_M * 1e3)) + 1):
                    T_j = T_s.copy()
                    T_j[:3, 3] -= j * 1e-3 * v
                    qq = rig.ik(T_j, prev)
                    if qq is None or float(np.abs(qq - prev).max()) > .1:
                        brk = f"IK branch jump {n_dn - j:+d} mm from the seat"
                        if drop_on and d_fv is None and j <= n_dn:
                            d_fv = [n_dn - j, "IK branch jump"]
                        break
                    if rule1 and j <= n_dn and (j % 5 == 0 or j == n_dn):   # D-seat's 2 mm rule: the hand and SEAT_LINKS
                        hh_ = seat_line_hits(arm, qq, tool_, tgt_, np)
                        if drop_on and d_fv is None:   # D2's rule: D-seat's and the open gripper hull
                            hd_ = hh_ or open_hull_hits(arm, qq, tgt_, np)
                            if hd_:
                                d_fv = [n_dn - j, hd_[0]]
                            else:
                                d_rel = n_dn - j
                        if hh_:
                            brk = f"final descent {n_dn - j} mm above the seat: {hh_[0]}"
                            break
                    prev = qq
                if drop_on and d_rel is not None:
                    dcands.append((d_rel, h_s, T_s, q, d_fv))
                if brk is None:
                    cl_ = (arm.held_clearance(q, (kind, M, oid), ignore=(oid,), cutoff=CLEAR_CUTOFF_M) if rule1
                           else (None, None))
                    cands.append((h_s, T_s, q, None if cl_[0] is None else min(float(cl_[0]), CLEAR_CUTOFF_M), cl_[1]))
                    if rule1:
                        break                          # one IK branch per start height
                else:
                    bk_ = "IK branch jump" if brk.startswith("IK") else brk.split(": ")[-1].rsplit(" ", 2)[0]
                    dbreaks[bk_] = dbreaks.get(bk_, 0) + 1
            if not cands or rule1:
                h_s = round(h_s + SEAT_START_STEP_M, 3)
        rels = [{"mode": "seat", "release_mm": 0} for _ in cands]   # the plain I4-wall20: released at the sensed seat
        if drop_on and not cands:                      # D2: no start's full descent is clear: the tine-guided release
            gd_, slot_ = {}, meta[oid]["slot"]
            for c in dcands:                           # the plate alone at each release height: its rim vs its slot's tine tips
                if c[0] not in gd_:
                    T_p = T_seat.copy()
                    T_p[:3, 3] += c[0] / 1e3 * v
                    pc_ = dish_clear(arm, kind, T_p, oid, scene_h, np)
                    gd_[c[0]] = {**tine_guided(arm, slot_, lib["labels"], T_p, ns.points[kind], np),
                                 "plate_clear_mm": None if not np.isfinite(pc_[0]) else round(float(pc_[0]) * 1e3, 1),
                                 "plate_clear_to": pc_[1]}
            ok_ = [c for c in dcands if (gd_[c[0]]["guided_mm"] if gd_[c[0]]["guided_mm"] is not None else -1e9)
                   >= DROP_GUIDED_MIN_M * 1e3]
            low_ = min(dcands, key=lambda c: c[0]) if dcands else None
            out["drop"] = {"ok": bool(ok_), "candidates": len(dcands), "guided": len(ok_),
                           "release_mm_min": None if low_ is None else low_[0],
                           "at_min": None if low_ is None else {**gd_[low_[0]], "first_violation": low_[4],
                                                                "start_mm": round(low_[1] * 1e3)},
                           "guided_min_mm": round(DROP_GUIDED_MIN_M * 1e3), "rule": DROP_RULE}
            if not ok_:
                out["drop"]["why"] = ("no descent start with a free IK passes the drop rule at the start itself" if low_ is None else
                                      f"not tine-guided at {low_[0]} mm: the plate's lowest rim point "
                                      f"{gd_[low_[0]]['guided_mm']} mm below its slot's tine tips there (at least "
                                      f"{DROP_GUIDED_MIN_M * 1e3:.0f} mm needed; first violation {low_[4]}); nothing released")
            for c in ok_:
                cl_ = arm.held_clearance(c[3], (kind, M, oid), ignore=(oid,), cutoff=CLEAR_CUTOFF_M)
                cands.append((c[1], c[2], c[3], None if cl_[0] is None else min(float(cl_[0]), CLEAR_CUTOFF_M), cl_[1]))
                rels.append({"mode": "drop", "release_mm": c[0], "first_violation": c[4], **gd_[c[0]]})
        if rule1:                                      # the lowest release first (the seat: 0), then the largest held-plate
            ord_ = sorted(range(len(cands)), key=lambda k: (rels[k]["release_mm"], -cands[k][3], cands[k][0]))   # clearance,
            cands, rels = [cands[k] for k in ord_], [rels[k] for k in ord_]   # then the lowest start
        out.update(n_free=n_free, poses_tried=poses_tried, starts=[(h, T, q) for h, T, q, _, _ in cands],
                   start_clear={f"{round(h * 1e3)}": [None if d is None else round(d * 1e3, 1), nm] for h, _, _, d, nm in cands},
                   descent_breaks=dbreaks, closed_src=lib.get("closed_src"),
                   rule=None if not rule1 else
                   f"rule 1: every descent start {SEAT_START_ABOVE_M * 1e3:.0f}..{SEAT_START_MAX_M * 1e3:.0f} mm above the seat "
                   f"(5 mm steps) with a collision-free IK (15 mm arm, the held plate {CARRY_CLEAR_M * 1e3:.0f} mm from the scene), "
                   f"IK-continuous to {DESCENT_BELOW_M * 1e3:.0f} mm under the seat and the hand (closed jaws {lib.get('closed_src')} "
                   f"+ wrist 3) and the links {SEAT_ARM_MARGIN_M * 1e3:.0f} mm from the scene every 5 mm down to the seat; "
                   f"ranked by the held plate's clearance at the start (cut at {CLEAR_CUTOFF_M * 1e3:.0f} mm), then the height")
        cands = [(h, T, q) for h, T, q, _, _ in cands]
        lines, choices = [], []
        for k_c, (h_s, T_s, q_s) in enumerate(cands[:DROP_TRIES] if rule1 else cands):   # the line up the tine axis toward z_safe
            h_full = (flags.z_safe_m - float(T_s[2, 3])) / float(v[2])
            n_up = max(3, int(round(h_full / SEAT_LINE_STEP_M)))
            prev, q_top, h_top, why_up = q_s, None, 0., None
            for j in range(1, n_up + 1):               # 5 mm steps, IK-continuous and free; the highest good point is the top
                T_j = T_s.copy()
                T_j[:3, 3] += h_full * j / n_up * v
                q = rig.ik(T_j, prev)
                if q is None or float(np.abs(q - prev).max()) > SEAT_LINE_JUMP_RAD:
                    why_up = f"IK branch jump {h_full * j / n_up * 1e3:.0f} mm up the axis"
                    break
                if not arm.free(q, **kw):
                    why_up = f"collides {h_full * j / n_up * 1e3:.0f} mm up the axis: {arm.collisions(q, first=False, **kw)[:2]}"
                    break
                prev, q_top, h_top = q, q, h_full * j / n_up
            if q_top is None or h_top < SEAT_LINE_MIN_M - 1e-9:
                lines.append({"h_s_mm": round(h_s * 1e3), "line": f"under {SEAT_LINE_MIN_M * 1e3:.0f} mm free: {why_up}"})
                continue
            T_top = T_s.copy()
            T_top[:3, 3] += h_top * v
            lines.append({"h_s_mm": round(h_s * 1e3), "line": f"free {h_top * 1e3:.0f} of {h_full * 1e3:.0f} mm" +
                          (f" (stopped: {why_up})" if why_up else " (to z_safe)"),
                          "q_top_deg": [round(float(x), 1) for x in np.degrees(q_top)]})
            recs = []
            for b, qh in enumerate(homes):             # straight joint moves first, on every free home branch
                qh = np.asarray(qh, dtype=float)
                recs.append({"b": b, "h_s": h_s, "T_s": T_s, "q_s": q_s, "T_top": T_top, "q_top": q_top, "h_top": h_top,
                             "move": float(np.abs(q_top - qh).max()), "path": None, "release": rels[k_c],
                             "leg1": "straight" if arm.path_free(qh, q_top, step=.02, **kw) else None})
            if not any(r_["leg1"] == "straight" for r_ in recs):
                for r_ in sorted(recs, key=lambda x: x["move"]):   # then Kit-free RRT-Connect, the smallest move first
                    t_r = time.monotonic()
                    path = C.rrt_connect(arm, np.asarray(homes[r_["b"]], dtype=float), [q_top], time_s=SEAT_PLAN_RRT_S,
                                         collide_kw=kw, seed=1, shortcut_s=1.)
                    r_.update(leg1="rrt" if path is not None else "rrt: none", path=path, rrt_s=round(time.monotonic() - t_r, 1))
                    if path is not None:
                        break
            choices += [r_ for r_ in recs if r_["leg1"] is not None]
            if any(r_["leg1"] in ("straight", "rrt") for r_ in recs):
                break
        lost_ = [c for c in choices if c["leg1"] == "rrt: none"]
        nb_ = narrow_budget(args)
        if rule1 and nb_ > 0 and lost_ and not any(c["leg1"] in ("straight", "rrt") for c in choices):
            b0_ = lost_[0]["b"]                        # the narrow passage (plans/2026-11-04-fifteen-of-fifteen.md; = frigidaire_
            roots_ = [c for c in lost_ if c["b"] == b0_]   # robot_reach.i4_carry's retry): one search of --narrow-retry-s from the
            goals_ = [np.asarray(c["q_top"], dtype=float) for c in roots_]   # home rooted at the top of every start whose leg 1
            t_r = time.monotonic()                     # found no path (their lines up the tine axis checked above), seed 2, the
            path = C.rrt_connect(arm, np.asarray(homes[b0_], dtype=float), goals_, time_s=0., collide_kw=kw, seed=1,
                                 shortcut_s=1., retry_s=nb_)   # same held clearance and margins
            k_ = C.reached_goal(path, goals_)
            out["narrow_retry"] = {"budget_s": nb_, "goals": len(goals_), "reached": k_, "found": path is not None,
                                   "seconds": round(time.monotonic() - t_r, 1),
                                   "roots_start_mm": [round(c["h_s"] * 1e3) for c in roots_],
                                   "reached_start_mm": None if k_ is None else round(roots_[k_]["h_s"] * 1e3),
                                   "rule": "every start's leg 1 found no path in SEAT_PLAN_RRT_S: one collide.rrt_connect search "
                                           "of --narrow-retry-s (retry_s, seed 2) from the home rooted at the top of every such "
                                           "start's tine-axis line, the held clearance and the margins unchanged"}
            if path is not None:
                choices.append({**roots_[k_], "leg1": "rrt (narrow retry)", "path": path, "rrt_s": out["narrow_retry"]["seconds"]})
        out["lines"] = lines
        ok = [c for c in choices if c["leg1"] != "rrt: none"]
        ok.sort(key=lambda c: (c["leg1"] != "straight", c["move"], c["h_s"]))
        out["choices"] = [{"home_q_deg": [round(float(x), 1) for x in np.degrees(homes[c["b"]])], "h_s_mm": round(c["h_s"] * 1e3),
                           "leg1": c["leg1"], "move_deg": round(float(np.degrees(c["move"])), 1), "rrt_s": c.get("rrt_s"),
                           "release_mm": (c.get("release") or {}).get("release_mm")}
                          for c in choices]
        out["chosen"] = ok[0] if ok else None
        out["plan_s"] = round(time.monotonic() - t0_, 1)
        return out

    def insert_seat(i, oid, row, landed):
        """--insert seat (rows I4, window 2): the plate welded at the assumed base-face hold (--rim-depth-m); reset()'s seat_plan
        (the seat, the hand's clearance there, the descent start, the line up the tine axis, the home branch by leg 1). Isaac:
        leg 1 from home to the top of the tine-axis line (the planned straight move or the planned RRT path), the straight
        line down the tine axis to the descent start (IK-continuous, 15 mm arm / 2 mm held margins), the closed gripper's first
        touch on the way down (Kit-free, the landed plates as they lie: the predicted hand stop), lower_until_contact along the
        tine axis (20 mm/s, the flange sensor), release where it stopped (joint removed, jaws opened), the I3 retreat, the
        benchmark's settle, the seated test. --attach none (plans/2026-11-04-fifteen-of-fifteen.md): "perch predicted" (D3) and "not
        tine-guided" (D2) rows before the carry; the chosen start's release (D2's drop: lowered to its planned release height or the
        sensor's trip, the guidance re-measured at a position stop, the jaws opened, up the tine axis, judged by AT_GOAL and the
        seated test) recorded under "release"."""
        kind, T_g, M = kinds[oid], goal[oid], att["M"]
        kw = {"held": (kind, M, oid), "ignore": (oid,)}
        lib = seat_lib()
        sp = dict(carry_state)
        before = backend.poses()
        sync_world()
        v, sk = sp["v"], sp["sk"]
        bias_mm_ = round(float(sp.get("bias_m", SEAT_DX_M)) * 1e3, 1)   # round 3: the load row's descent_bias_mm (default 1)
        row.update(path={"kind": "tine axis", "lean_deg": round(sp["lean_t"], 2), "dx_mm": bias_mm_,
                         "descent_bias_mm": bias_mm_, "bias_from": "the load row" if meta[oid].get("descent_bias_mm") is not None
                         else f"the default ({DESCENT_BIAS_MM:.0f} mm)",
                         "dir": [round(float(x), 5) for x in v],
                         "rule": f"the plate at its goal orientation, moved {bias_mm_:g} mm toward the basket (+x of the LowerRack, "
                                 "the load row's descent_bias_mm), lowered along the tine axis (up along the tines, leaning toward "
                                 "the basket)"},
                   hold_depth_mm=None if args.rim_depth_m is None else round(args.rim_depth_m * 1e3, 1),
                   seat_search={**sk, "seconds": sp["seat_s"]},
                   carry_plan={"lines": sp.get("lines"), "choices": sp.get("choices"), "seconds": sp.get("plan_s"),
                               "narrow_retry": sp.get("narrow_retry"),
                               "rule": "leg 1 = home -> the top of the tine-axis line at the TCP height z_safe (straight joint move "
                                       "first, else Kit-free RRT-Connect); leg 2 = the straight Cartesian line down the tine "
                                       "axis to the descent start"})
        post.update(carry=True, start=rig.q())
        for k_ in ("tol_rrt", "tol_joint_move_held", "tol_rise"):     # no gate decides placing (window 1's 0.105 rad end lag)
            object.__setattr__(flags, k_, float(rig.lag_max))
        row["carry_settle_tol_rad"] = float(rig.lag_max)

        def park(step_, why_):
            post["carry"] = False
            harness.armed = False
            detach()
            harness.clear_carried()
            backend.set_rigid_pose(backend.objects[oid], park_pose(i))
            return fail(row, step_, why_)
        if sk.get("seat_mm") is None:
            return park("no seat", f"the plate alone along the tine axis touches nothing above {sk}")
        row["seat_at_goal"] = sp.get("seat_at_goal")   # D3 (plans/2026-11-04-fifteen-of-fifteen.md): the seat within 20 mm, on wires
        if sp.get("perch"):                            # D3: a perch predicted, the move not attempted
            pc_ = sp["perch"]
            return park("perch predicted", f"the Kit-free seat {pc_['seat_from_goal_mm']} mm from the goal along the tine axis "
                                           f"(within {pc_['limit_mm']} mm counts), supports {pc_['supports']}"
                                           + (f", on {pc_['on_dish']}" if pc_["on_dish"] else "")
                                           + ": a perch, not attempted (the plate parked after the row)")
        T_ts, seat_z = sp["T_ts"], sp["seat_z"]
        row["hand_clear_at_seat_mm"] = sp["cl"]
        c_ = np.r_[ns.centroids[kind], 1.]
        g_z = float((T_g @ c_)[2])
        ch = sp.get("chosen")
        dp_ = sp.get("drop")                           # D2: the tine-guided release's search (no start's full descent clear)
        if dp_ is not None:
            row["drop_search"] = dp_
        if ch is None and dp_ is not None and not dp_["ok"]:
            return park("not tine-guided" if dp_.get("release_mm_min") is not None else "no reach", dp_["why"])
        if ch is None:
            if not sp.get("starts"):
                row["blocking"] = blocking_pairs(rig, arm, sp["poses_tried"][::5] + [("the seat", T_ts, kw)], np)
                return park("no reach", f"no collision-free IK continuous to the bound at any descent start "
                                        f"+{SEAT_START_ABOVE_M * 1e3:.0f}..{SEAT_START_MAX_M * 1e3:.0f} mm above the seat "
                                        f"({sp.get('n_free')} free IK solutions, none continuous)")
            return park("no reach", f"no carry: the tine-axis lines {sp.get('lines')}; leg 1 per home branch {sp.get('choices')}")
        h_s, T_s, q_s, T_top, q_top = ch["h_s"], ch["T_s"], ch["q_s"], ch["T_top"], ch["q_top"]
        tlog.set(phase="carry: leg 1 to the top of the tine-axis line")
        ns.plan_log.clear()
        if ch["leg1"] == "straight":
            ns.plan_log.append("straight")
            ok1 = rig.move_joint(q_top, max_step=.004, settle_tol=flags.tol_joint_move_held)
        else:                                          # "rrt" or (I4) "rrt (narrow retry)", seat_plan's narrow passage
            ns.plan_log.append(f"{ch['leg1']} (planned in reset)")
            ok1 = rig.follow(ch["path"], steps_per=10, settle_tol=flags.tol_rrt)
        row["drop_tries"] = 1
        if not ok1:
            return park("no reach", f"leg 1 blocked in Isaac: {getattr(rig, 'last_fail', None)}")
        tlog.set(phase="carry: down the tine axis")
        n_dn = max(3, int(round(float((T_top[:3, 3] - T_s[:3, 3]) @ v) / SEAT_LINE_STEP_M)) + 1)
        down, why_dn = line_from(T_top, T_s, n_dn, rig.q(), kw)
        if down is None:
            return park("no reach", f"the line down the tine axis: {why_dn}")
        ok2 = rig.follow(down, settle_tol=flags.tol_rise)
        ns.plan_log.append("tine-axis line" if ok2 else "tine-axis line: blocked")
        if not ok2:
            return park("no reach", f"the line down the tine axis blocked in Isaac: {getattr(rig, 'last_fail', None)}")
        legs = list(ns.plan_log)
        moved = max((float(np.linalg.norm(np.subtract(backend.poses()[d]["position_m"], before[d]["position_m"])))
                     for d in landed), default=0.)
        sync_world()                                   # the landed plates where they lie after the carry
        scene_h = scene_targets(arm, ignore=(oid,))
        ht = hand_touch(arm, T_ts, v, h_s, -DESCENT_BELOW_M, lib["closed"], scene_h, np)
        hand_z = None if ht is None else round(seat_z + ht["h_mm"] * float(v[2]), 1)
        hz = lambda T_: round((float((np.asarray(T_) @ c_)[2]) - g_z) * 1e3, 1)
        qs_ = row["reset"]["q_start_deg"]
        row.update(start_branch={"q_start_deg": qs_, "wrist_2_deg": qs_[4], "wrist_3_deg": qs_[5]}, legs={"plans": legs},
                   descent_start_mm=round(h_s * 1e3), descent_start_above_50=bool(h_s > SEAT_START_ABOVE_M + 1e-9),
                   carry_moved_landed_mm=round(moved * 1e3, 2), hand_touch_pred=ht,
                   in_hand={**row["in_hand"], "side": "mouth face (the counter pick)" if att_mode["none"] else "base"})
        carry_watch.update(phase="descent", arrived=True)   # --attach none: the carry ends at the descent start
        rel_ = dict(ch.get("release") or {"mode": "seat", "release_mm": 0})
        drop_ = rel_.get("mode") == "drop"             # D2: down to the planned release height (an earlier sensor trip ends it)
        cb_ = {"T_start": T_s, "q_start": q_s, "h_start": h_s, "h_b": 0., "dir": v}
        dr_plan_ = None
        if drop_:
            cb_["h_end"] = float(rel_["release_mm"]) / 1e3
            dr_plan_ = drop_retreat_plan(oid, kind, T_s, q_s, h_s, cb_["h_end"], v, T_ts @ M)
        stop = lower_until_contact(oid, cb_)
        post["carry"] = False
        meas = stop["dish_h_mm"]
        kind_s = stop["kind"]
        pred = seat_z if kind_s == kind else hand_z if kind_s == "hand" else None
        row.update(stop=stop, descent_mm=stop["descent_mm"], weld_drift_mm=stop["weld_drift_mm"],
                   seat_mm={"pred": seat_z, "pred_axis_mm": sk["seat_mm"], "measured": meas,
                            "diff": round(meas - seat_z, 1),
                            "rule": "the plate centroid's height above its goal centroid (world z, mm): pred = the Kit-free "
                                    "seat, measured = at the stop"},
                   hand_stop_mm={"pred": hand_z, "pred_axis_mm": None if ht is None else ht["h_mm"],
                                 "measured": meas, "diff": None if hand_z is None else round(meas - hand_z, 1),
                                 "rule": "the plate centroid's height above its goal centroid when the closed gripper first "
                                         "touches the scene on the Kit-free descent (pred) vs at the stop (measured)"},
                   stop_check={"kind": kind_s, "pred_mm": pred, "measured_mm": meas,
                               "within_2mm": None if pred is None else bool(abs(meas - pred) <= 2.)})
        T_r = T_now(oid)
        if att_mode["none"]:                           # the release record (D2): the plain I4-wall20 at the seat, or the drop
            T_seat_ = T_ts @ M
            sync_world()
            g_now = tine_guided(arm, meta[oid]["slot"], lib["labels"], T_r, ns.points[kind], np)
            row["release"] = {**rel_, "planned_release_mm": rel_.get("release_mm"),
                              "actual_release_mm": round(float((rig.tcp()[:3, 3] - T_ts[:3, 3]) @ v) * 1e3, 1),
                              "plate_above_seat_mm": round(float((T_r[:3, 3] - T_seat_[:3, 3]) @ v) * 1e3, 1),
                              "guided_actual_mm": g_now["guided_mm"], "stop_by": "position" if kind_s == "none" else f"sensor ({kind_s})",
                              "load_row": meta[oid].get("release"),
                              "rule": DROP_RULE if drop_ else "released at the sensed seat: a start's full descent clear (the plain "
                                                              "I4-wall20)"}
            row["release"].pop("release_mm", None)
            if drop_ and kind_s == "none" and (g_now["guided_mm"] is None or g_now["guided_mm"] < DROP_GUIDED_MIN_M * 1e3):
                return fail(row, "not tine-guided",   # stopped by position (the plan's height, an IK end, a block) with the rim
                            f"not tine-guided at {row['release']['actual_release_mm']} mm: the plate's lowest rim point "   # not low
                            f"{g_now['guided_mm']} mm below its slot's tine tips at the stop (at least {DROP_GUIDED_MIN_M * 1e3:.0f} "
                            f"mm needed; planned {rel_.get('guided_mm')} mm at {rel_.get('release_mm')} mm); the jaws stay "
                            f"closed, nothing released, the plate parked after the row")
        if drop_:
            row["release_pose"] = release_up(oid, v, stop["descent_mm"] / 1e3, row=row, T_seat=T_ts @ M, plan=dr_plan_)
            row["retreat"] = {**retreat, "rule": "D2: the jaws opened where the descent stopped (the flange sensor's trip or the "
                                                 "planned release height), the rest wait (DROP_REST_RULE), then the back-off 20 mm "
                                                 "along -TCP z when DROP_RETREAT_RULE's check is clear at the plate's pose then, and "
                                                 "the open hand up the tine axis back to the descent start's height (else the rise "
                                                 "alone)"}
        else:
            row["release_pose"] = release(oid, back=True)
            row["retreat"] = {**retreat, "rule": f"the jaws opened where the descent stopped; back up to {RELEASE_BACK_MAX_M * 1e3:.0f} "
                                                 f"mm along -TCP z through the collision-checked line ({BACK_STEP_M * 1e3:.0f} mm "
                                                 f"waypoints, 15 mm margin, the released plate a scene dish; the gripper may touch it "
                                                 f"in the first {ALLOW_CONTACT_M * 1e3:.0f} mm), truncated at the last free waypoint; "
                                                 f"then up {flags.hover_m * 1e3:.0f} mm in world z only when the open gripper hulls "
                                                 f"are >= {RIM_CLEAR_M * 1e3:.0f} mm from the plate (True = followed, False = blocked)"}
        s_ = ns.bench_settle(oid, "goal", T_r, before)
        T = T_now(oid)
        st = s_["settle"]
        settled = st["drift_mm"] <= ns.R.STABLE_POS_M * 1e3 and st["drift_deg"] <= ns.R.STABLE_ROT_DEG
        dist = s_["distance"]
        lean = lambda T_: round(float(np.degrees(np.arcsin(np.clip(T_[2, 2], -1., 1.)))), 1)   # + = mouth tilted up
        ok_slot, slot_info = in_slot(oid, T)
        now = backend.poses()
        F_now = T_of(now["LowerRack"]["position_m"], now["LowerRack"]["quaternion_xyzw"])
        sync_world()
        seated = seated_test(arm, kind, oid, T, F_now @ inv(F_goal["LowerRack"]) @ T_g, meta[oid]["slot"], lib["labels"], np,
                             centroid=ns.centroids[kind])
        row.update(settled=bool(settled), settle=st, final_pose=pose(T), axis_z=round(float(T[2, 2]), 3),
                   error={"lateral_mm": dist[0], "dz_mm": dist[1], "tilt_deg": dist[2]}, lean_from_goal_deg=dist[2],
                   lean_deg={"final": lean(T), "goal": lean(T_g)}, in_slot=ok_slot, in_slot_detail=slot_info,
                   seated=seated, height_final_mm=hz(T),
                   landed_on=sorted(b if a == oid else a for a, b in harness.contacts.pairs_with(oid)),
                   moved_earlier=[m for m in s_["rules"]["strict"] if m[0] in landed])
        landed.append(oid)
        at_g = None
        if drop_:                                      # D2: the landing by AT_GOAL's plate rule, beside the seated test
            at_g = bool(B.within_goal(kind, (dist[0] / 1e3, dist[1] / 1e3, dist[2])))
            row["at_goal"] = at_g
            row["release"]["landing"] = {"lateral_mm": dist[0], "dz_mm": dist[1], "tilt_deg": dist[2], "at_goal": at_g,
                                         "seated": bool(seated["seated"]), "tolerance": list(B.AT_GOAL[kind])}
        if not settled:
            fail(row, "not settled", f"drift {st['drift_mm']} mm / {st['drift_deg']} deg over the last {K.WINDOW_TICKS} "
                                     f"ticks (limits 5 mm / 3 deg)")
        elif at_g is False:
            fail(row, "not at goal", f"the tine-guided release landed {dist[0]} mm lateral, {dist[1]} mm height, {dist[2]} deg "
                                     f"off its goal (AT_GOAL plate {list(B.AT_GOAL[kind])})")
        elif not seated["seated"]:
            fail(row, "not seated", f"settled off its goal's wires: bottom {seated['bottom_nearest']}, near column "
                                    f"{seated['near_column_nearest']}, far column {seated['far_column_nearest']}")

    # ---------------------------------------------------------------- plans/2026-10-29-sampler-release.md: rows I5 / D-sampler
    def sampler_plan(oid, M, homes):
        """--insert sampler, Kit-free, in reset(), the landed dishes at rest: the sampler's release pose (final_pose_table.json,
        re-expressed in the dish's rack frame now), the dish alone lowered straight down from it (sampler_stop: the predicted
        rack stop), the TCP there (M); the descent start = the lowest of 50, 55, .. 150 mm above the release with a
        collision-free IK (15 mm arm / 2 mm held margins), free down to the release and IK-continuous to the bound (the
        predicted stop - DESCENT_BELOW_M); from it the straight world-z line up to z_safe or its last free IK-continuous
        waypoint (5 mm steps): its top q_top; the home branch whose leg 1 to q_top is a straight collision-free joint move,
        else the one whose Kit-free RRT-Connect (SEAT_PLAN_RRT_S, seed 1) finds a path, ties by the smallest largest-joint
        move."""
        kind = kinds[oid]
        kw = {"held": (kind, M, oid), "ignore": (oid,)}
        sync_world()
        t0_ = time.monotonic()
        rack = meta[oid]["rack"]
        now = backend.poses()
        F = T_of(now[rack]["position_m"], now[rack]["quaternion_xyzw"])
        fp = samp[oid]
        T_rel = F @ inv(F_goal[rack]) @ T_of(fp["release_pose"]["position_m"], fp["release_pose"]["quaternion_xyzw"])
        c_ = np.r_[ns.centroids[kind], 1.]
        g_z = float((goal[oid] @ c_)[2])               # the load's goal (as lower_until_contact's heights)
        st = sampler_stop(arm, kind, oid, T_rel, scene_targets(arm, ignore=(oid,), statics=False), np)
        T_ts = T_rel @ inv(M)
        v = np.array([0., 0., 1.])
        rel_h = round((float((T_rel @ c_)[2]) - g_z) * 1e3, 1)
        out = {"homes": homes, "T_rel": T_rel, "T_ts": T_ts, "v": v, "g_z": g_z,
               "release": {"pose": pose(T_rel), "hold": fp["hold"], "lift_mm": fp["release_lift_mm"],
                           "lowest_clear_lift_mm": fp["lowest_clear_lift_mm"], "above_goal_mm": rel_h,
                           "table_above_goal_mm": fp["release_above_goal_mm"],
                           "rack_vs_goal_frame_mm": round(float(np.linalg.norm(F[:3, 3] - F_goal[rack][:3, 3])) * 1e3, 2)},
               "stop_pred": {**st, "above_goal_mm": None if st["h_m"] is None else round(rel_h + st["h_m"] * 1e3, 1),
                             "below_release_mm": None if st["h_m"] is None else round(-st["h_m"] * 1e3, 2),
                             "rule": "the dish alone lowered straight down from the sampler's release pose: its first touch "
                                     "(FCL distance <= 0) of the appliance, the counter or another dish (2 mm grid bisected "
                                     "to 0.25 mm); above_goal = the dish centroid above its goal centroid"}}
        if st["h_m"] is None:
            out["plan_s"] = round(time.monotonic() - t0_, 1)
            return out
        h_b = st["h_m"]
        cands, n_free, poses_tried, brks = [], 0, [], []
        h_s = SAMPLER_ABOVE_M
        while h_s <= SEAT_START_MAX_M + 1e-9 and not cands:
            T_s = T_ts.copy()
            T_s[2, 3] += h_s
            poses_tried.append((f"+{h_s * 1e3:.0f} mm", T_s, kw))
            for q in [q for q in rig.ik_all(T_s, rig.q()) if arm.free(q, **kw)]:
                n_free += 1
                prev, brk = q, None
                for j in range(1, int(round((h_s - h_b + DESCENT_BELOW_M) * 1e3)) + 1):
                    T_j = T_s.copy()
                    T_j[2, 3] -= j * 1e-3
                    qq = rig.ik(T_j, prev)
                    if qq is None or float(np.abs(qq - prev).max()) > .1:
                        brk = f"IK jump {j} mm down"
                        break
                    if h_s - j * 1e-3 >= -1e-9 and j % 5 == 0 and not arm.free(qq, **kw):
                        brk = f"collides {j} mm down: {arm.collisions(qq, first=False, **kw)[:2]}"
                        break
                    prev = qq
                if brk is None:
                    cands.append((h_s, T_s, q))
                else:
                    brks.append(f"+{h_s * 1e3:.0f}: {brk}")
            if not cands:
                h_s = round(h_s + SEAT_START_STEP_M, 3)
        out.update(n_free=n_free, poses_tried=poses_tried, starts=[(h, T, q) for h, T, q in cands], descent_breaks=brks[:6])
        lines, choices = [], []
        for h_s, T_s, q_s in cands:                     # the line straight up toward z_safe
            h_full = flags.z_safe_m - float(T_s[2, 3])
            n_up = max(3, int(round(h_full / SEAT_LINE_STEP_M)))
            prev, q_top, h_top, why_up = q_s, None, 0., None
            for j in range(1, n_up + 1):
                T_j = T_s.copy()
                T_j[2, 3] += h_full * j / n_up
                q = rig.ik(T_j, prev)
                if q is None or float(np.abs(q - prev).max()) > SEAT_LINE_JUMP_RAD:
                    why_up = f"IK ends or jumps {h_full * j / n_up * 1e3:.0f} mm up (z {float(T_j[2, 3]):.3f} m)"
                    break
                if not arm.free(q, **kw):
                    why_up = f"collides {h_full * j / n_up * 1e3:.0f} mm up: {arm.collisions(q, first=False, **kw)[:2]}"
                    break
                prev, q_top, h_top = q, q, h_full * j / n_up
            if q_top is None or h_top < SEAT_LINE_MIN_M - 1e-9:
                lines.append({"h_s_mm": round(h_s * 1e3), "line": f"under {SEAT_LINE_MIN_M * 1e3:.0f} mm free: {why_up}"})
                continue
            T_top = T_s.copy()
            T_top[2, 3] += h_top
            lines.append({"h_s_mm": round(h_s * 1e3), "line": f"free {h_top * 1e3:.0f} of {h_full * 1e3:.0f} mm to z "
                          f"{float(T_top[2, 3]):.3f} m" + (f" (stopped: {why_up})" if why_up else " (to z_safe)"),
                          "q_top_deg": [round(float(x), 1) for x in np.degrees(q_top)]})
            recs = []
            for b, qh in enumerate(homes):             # straight joint moves first, on every free home branch
                qh = np.asarray(qh, dtype=float)
                recs.append({"b": b, "h_s": h_s, "T_s": T_s, "q_s": q_s, "T_top": T_top, "q_top": q_top, "h_top": h_top,
                             "move": float(np.abs(q_top - qh).max()), "path": None,
                             "leg1": "straight" if arm.path_free(qh, q_top, step=.02, **kw) else None})
            if not any(r_["leg1"] == "straight" for r_ in recs):
                for r_ in sorted(recs, key=lambda x: x["move"]):   # then Kit-free RRT-Connect, the smallest move first
                    t_r = time.monotonic()
                    path = C.rrt_connect(arm, np.asarray(homes[r_["b"]], dtype=float), [q_top], time_s=SEAT_PLAN_RRT_S,
                                         collide_kw=kw, seed=1, shortcut_s=1.)
                    r_.update(leg1="rrt" if path is not None else "rrt: none", path=path, rrt_s=round(time.monotonic() - t_r, 1))
                    if path is not None:
                        break
            choices += [r_ for r_ in recs if r_["leg1"] is not None]
            if any(r_["leg1"] in ("straight", "rrt") for r_ in recs):
                break
        out["lines"] = lines
        ok = [c for c in choices if c["leg1"] != "rrt: none"]
        ok.sort(key=lambda c: (c["leg1"] != "straight", c["move"], c["h_s"]))
        out["choices"] = [{"home_q_deg": [round(float(x), 1) for x in np.degrees(homes[c["b"]])], "h_s_mm": round(c["h_s"] * 1e3),
                           "leg1": c["leg1"], "move_deg": round(float(np.degrees(c["move"])), 1), "rrt_s": c.get("rrt_s")}
                          for c in choices]
        out["chosen"] = ok[0] if ok else None
        out["plan_s"] = round(time.monotonic() - t0_, 1)
        return out

    def place_at_goal(oid):
        """--insert sampler, a dish whose sampler release pose is infeasible (the final-pose table): its rack out, the dish
        teleported to its goal pose in the rack's frame now, at rest (rest_wait). (T at rest, rest record)"""
        rack = meta[oid]["rack"]
        set_racks(rack)
        now = backend.poses()
        F = T_of(now[rack]["position_m"], now[rack]["quaternion_xyzw"])
        backend.set_rigid_pose(backend.objects[oid], pose(F @ inv(F_goal[rack]) @ goal[oid]))
        rest = rest_wait([oid])
        return T_now(oid), rest

    def insert_sampler(i, oid, row, landed):
        """--insert sampler (rows I5 / D-sampler): the dish welded at the final-pose table's hold; reset()'s sampler_plan.
        Isaac: leg 1 from home to the top of the line above the descent start (the planned straight move or RRT path), the
        straight line down to the descent start (IK-continuous, 15 mm arm / 2 mm held margins), the pre-descent settle,
        lower_until_contact straight down (20 mm/s, the flange sensor; bounded DESCENT_BELOW_M under the predicted stop), the
        TCP up SAMPLER_UP_M with the dish, the jaws opened (joint removed), the hand straight up SAMPLER_RISE_M, the
        benchmark's settle, seated on the goal's wires (plates: seated_test; bowls: bowl_seated_test AND mouth down)."""
        kind, M = kinds[oid], att["M"]
        kw = {"held": (kind, M, oid), "ignore": (oid,)}
        sp = dict(carry_state)
        before = backend.poses()
        sync_world()
        row.update(sampler_release=sp.get("release"), stop_pred=sp.get("stop_pred"),
                   carry_plan={"lines": sp.get("lines"), "choices": sp.get("choices"), "seconds": sp.get("plan_s"),
                               "descent_breaks": sp.get("descent_breaks"),
                               "rule": "leg 1 = home -> the top of the straight world-z line above the descent start (to z_safe "
                                       "or its last free IK-continuous waypoint; straight joint move first, else Kit-free "
                                       "RRT-Connect); leg 2 = that line down to the descent start, 50 mm above the sampler's "
                                       "release pose"})
        post.update(carry=True, start=rig.q())
        for k_ in ("tol_rrt", "tol_joint_move_held", "tol_rise"):     # no gate decides placing (window 1's 0.105 rad end lag)
            object.__setattr__(flags, k_, float(rig.lag_max))
        row["carry_settle_tol_rad"] = float(rig.lag_max)

        def park(step_, why_):
            post["carry"] = False
            harness.armed = False
            detach()
            harness.clear_carried()
            backend.set_rigid_pose(backend.objects[oid], park_pose(i))
            return fail(row, step_, why_)
        st_p = sp.get("stop_pred") or {}
        if st_p.get("h_m") is None:
            return park("no seat", "the dish alone lowered straight down from the sampler's release pose touches nothing down "
                                   f"to {abs(SAMPLER_STOP_TO_M) * 1e3:.0f} mm under it")
        ch = sp.get("chosen")
        if ch is None:
            if not sp.get("starts"):
                row["blocking"] = blocking_pairs(rig, arm, sp["poses_tried"][::5] + [("the release", sp["T_ts"], kw)], np)
                return park("no reach", f"no collision-free IK free to the release and continuous to the bound at any descent "
                                        f"start +{SAMPLER_ABOVE_M * 1e3:.0f}..{SEAT_START_MAX_M * 1e3:.0f} mm above the release "
                                        f"({sp.get('n_free')} free IK solutions; {sp.get('descent_breaks')})")
            return park("no reach", f"no carry: the lines up {sp.get('lines')}; leg 1 per home branch {sp.get('choices')}")
        h_s, T_s, q_s, T_top, q_top = ch["h_s"], ch["T_s"], ch["q_s"], ch["T_top"], ch["q_top"]
        tlog.set(phase="carry: leg 1 to the top of the line above the release")
        ns.plan_log.clear()
        if ch["leg1"] == "straight":
            ns.plan_log.append("straight")
            ok1 = rig.move_joint(q_top, max_step=.004, settle_tol=flags.tol_joint_move_held)
        else:                                          # "rrt" or (I4) "rrt (narrow retry)", seat_plan's narrow passage
            ns.plan_log.append(f"{ch['leg1']} (planned in reset)")
            ok1 = rig.follow(ch["path"], steps_per=10, settle_tol=flags.tol_rrt)
        row["drop_tries"] = 1
        if not ok1:
            return park("no reach", f"leg 1 blocked in Isaac: {getattr(rig, 'last_fail', None)}")
        tlog.set(phase="carry: straight down to the descent start")
        n_dn = max(3, int(round(float(T_top[2, 3] - T_s[2, 3]) / SEAT_LINE_STEP_M)) + 1)
        down, why_dn = line_from(T_top, T_s, n_dn, rig.q(), kw)
        if down is None:
            return park("no reach", f"the line down to the descent start: {why_dn}")
        ok2 = rig.follow(down, settle_tol=flags.tol_rise)
        ns.plan_log.append("line down" if ok2 else "line down: blocked")
        if not ok2:
            return park("no reach", f"the line down to the descent start blocked in Isaac: {getattr(rig, 'last_fail', None)}")
        legs = list(ns.plan_log)
        moved = max((float(np.linalg.norm(np.subtract(backend.poses()[d]["position_m"], before[d]["position_m"])))
                     for d in landed), default=0.)
        n_hold, lag0 = 0, rig.lag()                    # the pre-descent settle (rows D-seat's rule)
        while rig.lag() > PRE_DESCENT_LAG_RAD and n_hold < PRE_DESCENT_CAP:
            rig.step(1)
            n_hold += 1
        sync_world()                                   # the landed dishes where they lie after the carry
        T_ts, h_b, g_z, v = sp["T_ts"], st_p["h_m"], sp["g_z"], sp["v"]
        T_m = rig.tcp()
        h_now = float(T_m[2, 3] - T_ts[2, 3])
        closed_named = (seat_lib()["closed"] if kind == "plate" else
                        [(f"closed hull {k}", g, vv) for k, (g, vv) in enumerate(closed_hulls())])
        ht = hand_touch(arm, T_ts, v, h_now, h_b - DESCENT_BELOW_M, closed_named, scene_targets(arm, ignore=(oid,)), np)
        c_ = np.r_[ns.centroids[kind], 1.]
        rel_h = sp["release"]["above_goal_mm"]
        hand_h = None if ht is None else round(rel_h + ht["h_mm"], 1)
        qs_ = row["reset"]["q_start_deg"]
        row.update(start_branch={"q_start_deg": qs_, "wrist_2_deg": qs_[4], "wrist_3_deg": qs_[5]}, legs={"plans": legs},
                   descent_start_mm=round(h_s * 1e3), descent_start_above_50=bool(h_s > SAMPLER_ABOVE_M + 1e-9),
                   carry_moved_landed_mm=round(moved * 1e3, 2), hand_touch_pred=ht,
                   pre_descent={"ticks": n_hold, "lag_before_rad": round(float(lag0), 4), "lag_after_rad": round(float(rig.lag()), 4),
                                "tcp_vs_plan_mm": round(float(np.linalg.norm(T_m[:3, 3] - T_s[:3, 3])) * 1e3, 1),
                                "above_release_mm": round(h_now * 1e3, 1)})
        stop = lower_until_contact(oid, {"T_start": T_m, "q_start": rig.q(), "h_start": h_now, "h_b": h_b, "dir": v})
        meas = stop["dish_h_mm"]
        kind_s = stop["kind"]
        pred = st_p["above_goal_mm"] if kind_s.startswith(kind) else hand_h if kind_s == "hand" else None
        row.update(stop=stop, descent_mm=stop["descent_mm"], weld_drift_mm=stop["weld_drift_mm"],
                   stop_check={"kind": kind_s, "pred_mm": pred, "pred_dish_mm": st_p["above_goal_mm"], "pred_hand_mm": hand_h,
                               "measured_mm": meas, "within_2mm": None if pred is None else bool(abs(meas - pred) <= 2.),
                               "rule": "the dish centroid above its goal centroid (mm) at the sensed stop vs the Kit-free "
                                       "stop: the dish alone lowered from the release pose (dish stops) or the closed jaws' "
                                       "first touch (hand stops)"})
        tlog.set(phase=f"up {SAMPLER_UP_M * 1e3:.0f} mm before the release")
        T0 = rig.tcp()
        T_up = T0.copy()
        T_up[2, 3] += SAMPLER_UP_M
        qs_up = rig.line_q(T0, T_up, 13, rig.q())
        up_ok = None if qs_up is None else bool(rig.follow(qs_up, settle_tol=flags.tol_rise))
        post["carry"] = False
        T_r = T_now(oid)
        row["release_pose"] = pose(T_r)
        row["release_h_mm"] = round((float((T_r @ c_)[2]) - g_z) * 1e3, 1)
        row["up_before_release"] = {"followed": up_ok, "tcp_up_mm": round(float(rig.tcp()[2, 3] - T0[2, 3]) * 1e3, 1),
                                    "dish_above_stop_mm": round(row["release_h_mm"] - meas, 1)}
        detach()
        harness.clear_carried()
        tlog.set(phase="release")
        mark = backend.tick_index
        rig.gripper(False)
        tlog.set(phase=f"hand straight up {SAMPLER_RISE_M * 1e3:.0f} mm")
        T1 = rig.tcp()
        T2 = T1.copy()
        T2[2, 3] += SAMPLER_RISE_M
        qs_r = rig.line_q(T1, T2, 40, rig.q())
        rise_ok = None if qs_r is None else bool(rig.follow(qs_r))
        touch = sorted({"|".join(k) for k, act in harness.contacts.active.items() if act["last"] >= mark and oid in k
                        and any(c_t in SENSOR_TOOL_CATS + ("arm",) for c_t in act["cats"])})
        row["retreat"] = {"rise": rise_ok, "rise_mm": round(float(rig.tcp()[2, 3] - T1[2, 3]) * 1e3, 1),
                          "hand_touched_dish_after_release": touch,
                          "rule": f"the jaws opened {SAMPLER_UP_M * 1e3:.0f} mm above the sensed stop, then the open hand "
                                  f"straight up {SAMPLER_RISE_M * 1e3:.0f} mm in world z (no wait; the Kit-free check: the open "
                                  f"hand clears the tipping plate); touched = gripper or arm contacts with the dish from the "
                                  f"release on"}
        s_ = ns.bench_settle(oid, "goal", T_r, before)
        T = T_now(oid)
        st = s_["settle"]
        settled = st["drift_mm"] <= ns.R.STABLE_POS_M * 1e3 and st["drift_deg"] <= ns.R.STABLE_ROT_DEG
        dist = s_["distance"]
        sync_world()
        rack = meta[oid]["rack"]
        now = backend.poses()
        F_now = T_of(now[rack]["position_m"], now[rack]["quaternion_xyzw"])
        if kind == "plate":
            ok_slot, slot_info = in_slot(oid, T)
            sw = seated_test(arm, kind, oid, T, F_now @ inv(F_goal[rack]) @ goal[oid], meta[oid]["slot"], seat_lib()["labels"], np,
                             centroid=ns.centroids[kind])
            seated = {**sw}
            row.update(in_slot=ok_slot, in_slot_detail=slot_info)
        else:
            sw = bowl_seated_test(arm, kind, oid, T, oid, np, T_goal=F_now @ inv(F_goal[rack]) @ goal[oid],
                                  centroid=ns.centroids[kind])
            seated = {**sw, "seated": bool(sw["seated_wires"] and T[2, 2] < 0)}
        row.update(settled=bool(settled), settle=st, final_pose=pose(T), axis_z=round(float(T[2, 2]), 3),
                   mouth=None if kind == "plate" else ("up" if T[2, 2] > 0 else "down"),
                   error={"lateral_mm": dist[0], "dz_mm": dist[1], "tilt_deg": dist[2]}, lean_from_goal_deg=dist[2],
                   lean_deg={"final": lean_deg(T, np), "goal": lean_deg(goal[oid], np)}, seated=seated,
                   height_final_mm=round((float((T @ c_)[2]) - g_z) * 1e3, 1),
                   landed_on=sorted(b if a == oid else a for a, b in harness.contacts.pairs_with(oid)),
                   moved_earlier=[m for m in s_["rules"]["strict"] if m[0] in landed])
        landed.append(oid)
        if not settled:
            fail(row, "not settled", f"drift {st['drift_mm']} mm / {st['drift_deg']} deg over the last {K.WINDOW_TICKS} "
                                     f"ticks (limits 5 mm / 3 deg)")
        elif not seated["seated"]:
            fail(row, "not seated", (f"mouth {row['mouth']}, " if kind != "plate" else "") +
                 (f"wires: bottom {sw.get('bottom_nearest')}, near column {sw.get('near_column_nearest')}, far column "
                  f"{sw.get('far_column_nearest')}" if kind == "plate" else
                  f"wires: bottom {sw.get('bottom_nearest')}, tine {sw.get('tine_nearest')}"))

    # ---------------------------------------------------------------- D-ring (plans/2026-10-29-ring-place.md, --carry above)
    def hand_hits(q, oid, M):
        """The hand at q: the planner's check with the bowl held, without the held bowl's own scene contacts (a note, not a
        criterion): the links and the gripper vs the scene (arm.margin) and the held bowl vs the links (arm.held_margin)."""
        hits = arm.collisions(q, held=(kinds[oid], M, oid), ignore=(oid,), first=False, held_scene=False)
        return [f"{a}|{b}" for a, b in hits if a != "held" or b in C.ARM_LINKS]

    def ring_plan(oid, M, q_pick):
        """Kit-free, per spin about the bowl axis (RING_SPINS): the TCP at the refreshed goal (in the rack's frame now) turned by
        the spin, RING_ABOVE_M up along the rack's up axis = the descent start; an elbow-up IK there free at the margins with the
        bowl held, the straight line down to the goal IK-continuous with the hand clear at every 5 mm (hand_hits), the TCP above
        the bowl's centroid at the goal; ranked by the smallest largest joint move from the pick IK. D-side-down (the cup): the
        descent start the lowest of CUP_DOWN_ABOVES_M that passes per spin, ranked by that height first; the line down to the
        goal checked by D-seat's final-descent rule instead (seat_line_hits: the hand and the links 2 mm from the scene).
        --attach none (rule 1, plans/2026-11-04-bowl-carry-clearance.md amended): every start of DESCENT_STARTS_M (50..150 mm)
        that passes with the held dish CARRY_CLEAR_M from the scene (arm.held_clear_m) is a candidate, ranked by its held-dish
        clearance (largest first), then the move. The build release (a load dish with "build_release", plans/2026-11-04-deep-
        sites-gate.md amended): the dish at the build's move pose (moves_world) instead of the goal, the descent line down to the
        build's stop (lowest_lift above the move pose), the hand above the dish at the release BUILD_UP_M above the stop. The
        guarded insertion (INSERT_MOVES, plans/2026-11-04-fifteen-of-fifteen.md): the same starts and line, the release at the
        stop itself (no BUILD_UP_M raise); D4 (plans/2026-11-04-fifteen-of-fifteen.md): a start counts only when a retreat
        (retreat_check, the open hand judged too) is clear at the predicted stop (insert_stop: where the sensor will trip, below the
        line's end when the dish or the tool touches only lower); the open there (PARTIAL_OPEN_RULE): the full open, else the partial
        open, the first clear of (full, partial) x (a, b, c)."""
        t0_ = time.monotonic()
        kind = kinds[oid]
        rule1 = att_mode["none"]
        build = meta[oid].get("build_release") if att_mode["none"] else None
        insert = bool(getattr(args, "guarded_insert", False))
        aboves = DESCENT_STARTS_M if rule1 else CUP_DOWN_ABOVES_M if side_down else (RING_ABOVE_M,)
        seat_tool = tool_hulls(arm, ring_hulls()) if side_down or ring_side else None
        rk = meta[oid]["rack"]
        sync_world()
        now_ = backend.poses()
        F_r = T_of(now_[rk]["position_m"], now_[rk]["quaternion_xyzw"])
        T_g0 = F_r @ inv(F_goal[rk]) @ goal[oid]
        T_m0 = (F_r @ inv(F_goal[rk]) @ T_of(build["move_pose"]["position_m"], build["move_pose"]["quaternion_xyzw"])
                if build else T_g0)                    # the reference pose: the goal, or the build's move pose
        up = F_r[:3, 2] / np.linalg.norm(F_r[:3, 2])
        c_ = np.r_[ns.centroids[kind], 1.]
        kw_ = {"held": (kind, M, oid), "ignore": (oid,)}
        seat_targets = scene_targets(arm, ignore=(oid,)) if side_down or ring_side else None
        rt_targets = scene_targets(arm, ignore=(oid,)) if insert and rule1 else None   # D4: the retreat check's scene
        rt_cache, rt_rejects, stop_cache = {}, [], {}  # D4: per (spin, release joints) the retreat check; the starts it rejected;
        stop_tool = tool_hulls(arm, ring_hulls()) if insert and rule1 else None   # per spin the predicted stop (the closed tool)
        counts, ranked, first_fail = {}, [], None
        drop_pool = []                                 # INSERT_DROP_RULE: the starts whose sensor-trip release found no clear retreat
        _pp = {}

        def plan_partial():                            # the partial open (PARTIAL_OPEN_RULE): the hold trial's linkage at its own finger
            if "p" not in _pp:                         # angle + 5 mm a side, built the first time a full open fails
                _pp["p"] = partial_open(C, np, ring_hulls())
            return _pp["p"]

        def insert_stop(T_e, T_tg):
            """D4 at plan time: the guarded insertion releases where the flange sensor trips, which can lie below the descent line's
            end: the dish at the line end and the closed tool (the hold's jaws + wrist 3) lowered together along the rack's up axis
            (predict_stop's rule: the dish's first touch of the scene, the tool's when it comes first), down to DESCENT_BELOW_M under
            the goal. -> {"dz_m": below the line end (0: touching there), "h_mm": the dish centroid above the goal, "body", "target"}"""
            h0 = float((T_e @ c_)[2] - (T_g0 @ c_)[2])

            def at(T, h):
                T2 = np.array(T, dtype=float, copy=True)
                T2[:3, 3] += (h - h0) / max(float(up[2]), 1e-6) * up
                return T2

            def first_touch(objs_at, tg_, h_floor):
                h = h0
                while h > h_floor:
                    d, _, tn = min_clear(arm.fcl, objs_at(h), tg_, np)
                    if d <= 0:
                        return h, tn
                    h = round(h - clear_step_mm(d) / 1e3, 4)
                return None

            def dish_objs(h):
                b_ = arm.checker._body(kind, _pose_of(at(T_e, h)), oid)
                return [("dish", b_["manager"], *b_["bounds"])]

            def tool_objs(h):
                W3 = at(T_tg, h) @ inv(arm.T_w3_tcp)
                o_ = []
                for g, v in stop_tool:
                    p = np.asarray(v) @ W3[:3, :3].T + W3[:3, 3]
                    o_.append(("tool", arm.fcl.CollisionObject(g, arm.fcl.Transform(W3[:3, :3], W3[:3, 3])), p.min(0), p.max(0)))
                return o_
            hd = first_touch(dish_objs, [t for t in rt_targets if t[0] not in arm.statics], -DESCENT_BELOW_M)
            ht = first_touch(tool_objs, rt_targets, (hd[0] if hd else -DESCENT_BELOW_M) - 1e-6)
            hit = (ht[0], "hand", ht[1]) if ht is not None and (hd is None or ht[0] > hd[0]) else (
                (hd[0], "dish", hd[1]) if hd is not None else None)
            if hit is None:                            # nothing touched: the line end (the run would end in "no sensor stop")
                return {"dz_m": 0., "h_mm": round(h0 * 1e3, 1), "body": None, "target": None}
            return {"dz_m": max(0., h0 - hit[0]), "h_mm": round(hit[0] * 1e3, 1), "body": hit[1], "target": hit[2]}
        h_rel = rel_h.get(oid, 0.)                     # D-ring-h100: the release this far above the goal (D-ring: 0, the goal)
        lift = float(build["lowest_lift_mm"]) / 1e3 if build else h_rel   # the descent line's end above the reference
        up_rel = BUILD_UP_M if build and not insert else 0.   # the build: the dish let go this far above its stop (the insertion:
                                                       # at the stop)

        def bump(k_, why_=None):
            nonlocal first_fail
            counts[k_] = counts.get(k_, 0) + 1
            if first_fail is None and why_:
                first_fail = why_
        for k in range(RING_SPINS):
            sp = 2 * np.pi * k / RING_SPINS
            Rz = np.eye(4)
            Rz[:2, :2] = [[np.cos(sp), -np.sin(sp)], [np.sin(sp), np.cos(sp)]]
            T_g = T_m0 @ Rz
            T_e = T_g.copy()                           # the dish at the descent line's end: the (spun) goal + h_rel up the rack's
            T_e[:3, 3] += lift * up                    # axis; the build: its stop above the move pose
            T_tg = T_e @ inv(M)
            T_r = T_e.copy()                           # the dish at the release (the build: BUILD_UP_M above the stop)
            T_r[:3, 3] += up_rel * up
            T_tr = T_r @ inv(M)
            above = float(T_tr[2, 3] - (T_r @ c_)[2])
            if above <= 0.:
                bump("hand not above", f"spin {360. * k / RING_SPINS:.0f}: the TCP {above * 1e3:.0f} mm from the centroid at the release")
                continue
            if T_g[2, 2] >= 0.:                       # the carry rule (ring_drop's carry_ok): the bowl mouth down
                bump("mouth not down", f"spin {360. * k / RING_SPINS:.0f}: the bowl axis z {T_g[2, 2]:.2f}")
                continue
            for ab in aboves:                          # D-ring: 50 mm only; D-side-down: the lowest that passes; rule 1: all
                T_ds = T_tg.copy()
                T_ds[:3, 3] += ab * up
                n_line = int(round(ab / .005)) + 1     # 5 mm waypoints (RING_LINE_N at 50 mm)
                qs = rig.ik_all(T_ds, q_pick)
                found = None
                if not qs:
                    bump("no IK", f"spin {360. * k / RING_SPINS:.0f}: no elbow-up IK at the descent start +{ab * 1e3:.0f} mm")
                    continue
                for q in qs:
                    if not arm.free(q, **kw_):
                        bump("start not free", f"spin {360. * k / RING_SPINS:.0f}: the descent start +{ab * 1e3:.0f} mm collides "
                                               f"{sorted({a + '|' + b for a, b in arm.collisions(q, first=False, **kw_)})[:3]}")
                        continue
                    line = rig.line_q(T_ds, T_tg, n_line, q)
                    if line is None:
                        bump("line IK jump", f"spin {360. * k / RING_SPINS:.0f}: IK branch jump on the descent line")
                        continue
                    bad = next(((j, h_) for j, qq in enumerate(line) for h_ in
                                [seat_line_hits(arm, qq, seat_tool, seat_targets, np) if side_down or ring_side else
                                 hand_hits(qq, oid, M)]
                                if h_), None)          # D-side-down, D-ring-side: D-seat's final-descent rule (2 mm, the hand, links)
                    if bad is not None:
                        bump("descent not clear", f"spin {360. * k / RING_SPINS:.0f}: the hand on the descent line "
                                                  f"{(lift + ab * (1 - bad[0] / (n_line - 1))) * 1e3:.0f} mm above the "
                                                  f"{'move pose' if build else 'goal'}: {bad[1][:3]}")
                        continue
                    found = (q, line[-1])
                    break
                if found is None:
                    continue
                rt_ = None
                if insert and rule1:                   # D4 (plans/2026-11-04-fifteen-of-fifteen.md, the coordinator): the retreat after the
                    if k not in stop_cache:            # open where the insertion will stop (insert_stop), the first clear of (a), (b),
                        stop_cache[k] = insert_stop(T_e, T_tg)   # (c), the open hand there judged too (the gate's rule at its release)
                    st_ = stop_cache[k]
                    T_st = T_tg.copy()
                    T_st[:3, 3] -= st_["dz_m"] * up
                    q_st = np.asarray(found[1], dtype=float)
                    if st_["dz_m"] > 1e-4:             # IK-continuous from the line's end down to the predicted stop
                        ln_ = rig.line_q(T_tg, T_st, max(3, int(round(st_["dz_m"] / .005)) + 1), q_st)
                        q_st = None if ln_ is None else np.asarray(ln_[-1], dtype=float)
                    drop_ctx = None
                    if q_st is None:
                        rt_ = {"variant": None, "variants": {}, "release": None, "stop": st_,
                               "why": f"IK branch jump between the line's end and the predicted stop {st_['h_mm']} mm"}
                    else:
                        key_ = (k, tuple(np.round(q_st, 3)))
                        if key_ not in rt_cache:
                            T_dst = T_e.copy()
                            T_dst[:3, 3] -= st_["dz_m"] * up
                            rt_cache[key_] = {**retreat_check_open(arm, rig.ik, q_st, T_st, up,
                                                                   rt_targets + [(oid, arm.checker._body(kind, pose(T_dst), oid))],
                                                                   np, dish=oid, judge_release=True, partial=plan_partial),
                                              "stop": st_}   # the full open, else the partial open (PARTIAL_OPEN_RULE)
                        rt_ = rt_cache[key_]
                        T_dst_ = T_e.copy()
                        T_dst_[:3, 3] -= st_["dz_m"] * up
                        drop_ctx = (T_st, q_st, T_dst_)
                    if rt_["variant"] is None:
                        pm_ = (rt_.get("modes") or {}).get("partial")
                        bump("retreat not clear", f"spin {360. * k / RING_SPINS:.0f}: no clear retreat after the open at the predicted "
                                                  f"stop {st_['h_mm']} mm ({st_['body']}|{st_['target']}; +{ab * 1e3:.0f} mm start): "
                                                  f"{rt_.get('why') or retreat_brief(rt_)}"
                                                  + (f"; the partial open: {pm_.get('why')}" if pm_ else ""))
                        if len(rt_rejects) < DROP_TRIES:
                            rt_rejects.append({"spin_deg": 360. * k / RING_SPINS, "start_mm": round(ab * 1e3), "stop": st_,
                                               "release": rt_["release"], "variants": rt_["variants"], "why": rt_.get("why"),
                                               "modes": rt_.get("modes")})
                        if drop_ctx is not None:       # INSERT_DROP_RULE: a candidate for the guided release
                            drop_pool.append({"k": k, "ab": ab, "found": found, "T_ds": T_ds.copy(), "T_tg": T_tg, "T_g": T_g,
                                              "T_e": T_e, "T_r": T_r, "T_tr": T_tr, "above": above, "st": st_, "ctx": drop_ctx})
                        continue
                bump("pass")
                q_s, q_e = found
                h_s = float((T_ds @ M @ c_)[2] - (T_g0 @ c_)[2])
                touch = None
                if oid in rel_h:                       # a note: the held bowl's own first touch on the descent line
                    line_ = rig.line_q(T_ds, T_tg, RING_LINE_N, q_s)
                    touch = next((round((h_rel + RING_ABOVE_M * (1 - j / (RING_LINE_N - 1))) * 1e3, 1)
                                  for j, qq in enumerate(line_ or [])
                                  if any(a == "held" and b not in C.ARM_LINKS for a, b in arm.collisions(qq, first=False, **kw_))),
                                 None)
                cl_ = arm.held_clearance(q_s, (kind, M, oid), ignore=(oid,), cutoff=CLEAR_CUTOFF_M) if rule1 else (None, None)
                ranked.append({"spin": 360. * k / RING_SPINS, "q_start": q_s, "T_start": T_ds, "T_goal_tcp": T_tg, "T_goal": T_g,
                               "h_start": h_s, "above_mm": round(above * 1e3, 1), "up": up, "above_m": ab,
                               "move": float(np.abs(q_s - q_pick).max()), "h_rel": h_rel, "bowl_touch_mm": touch,
                               "clear_m": None if cl_[0] is None else min(float(cl_[0]), CLEAR_CUTOFF_M), "clear_to": cl_[1],
                               "q_end": q_e, "T_end": T_e, "T_rel": T_r, "T_tcp_rel": T_tr, "build": bool(build), "insert": insert,
                               "h_end": float((T_e @ c_)[2] - (T_g0 @ c_)[2]), "h_move": float((T_g @ c_)[2] - (T_g0 @ c_)[2]),
                               "retreat_pred": rt_,
                               **(dict(zip(("pitch_deg", "roll_deg", "tilt_deg"), (round(v_, 2) for v_ in side_pitch(T_g, T_tg))))
                                  if ring_side else {})})
                if not rule1:                          # D-ring / D-side-down before rule 1: the lowest passing start per spin
                    break
        drop_ = None
        if insert and rule1 and not ranked and drop_pool:   # INSERT_DROP_RULE: no start clears the sensor-trip release
            pts_ = np.asarray(ns.points[kind], dtype=float)
            tips_ = footprint_tips(arm, rk, T_g0, pts_, np)
            tried_ = []
            for e_ in drop_pool:
                T_st_, q_st_, T_dst_ = e_["ctx"]
                h_line_ = float((e_["T_e"] @ c_)[2] - (T_g0 @ c_)[2])
                dw_ = insert_drop_walk(arm, rig.ik, kind, oid, T_st_, q_st_, T_dst_, up, h_line_ - e_["st"]["dz_m"], h_line_,
                                       h_line_ + e_["ab"], rt_targets, plan_partial, tips_, pts_, np)
                tried_.append({"spin_deg": 360. * e_["k"] / RING_SPINS, "start_mm": round(e_["ab"] * 1e3), "ok": dw_["ok"],
                               "release_mm": dw_["release_mm"], "stop_mm": dw_["stop_mm"], "guided_mm": dw_["guided_mm"],
                               "open_mode": (dw_["open"] or {}).get("open_mode"), "variant": dw_["variant"],
                               "first_violation": dw_["first_violation"], "why": dw_["why"]})
                if not dw_["ok"]:
                    continue
                q_s, q_e = e_["found"]
                cl_ = arm.held_clearance(q_s, (kind, M, oid), ignore=(oid,), cutoff=CLEAR_CUTOFF_M)
                rt_p = {"variant": dw_["variant"], "variants": dw_["variants"], "release": dw_["release"], "stop": e_["st"],
                        "open": dw_["open"]}
                ranked.append({"spin": 360. * e_["k"] / RING_SPINS, "q_start": q_s, "T_start": e_["T_ds"], "T_goal_tcp": e_["T_tg"],
                               "T_goal": e_["T_g"], "h_start": float((e_["T_ds"] @ M @ c_)[2] - (T_g0 @ c_)[2]),
                               "above_mm": round(e_["above"] * 1e3, 1), "up": up, "above_m": e_["ab"],
                               "move": float(np.abs(q_s - q_pick).max()), "h_rel": h_rel, "bowl_touch_mm": None,
                               "clear_m": None if cl_[0] is None else min(float(cl_[0]), CLEAR_CUTOFF_M), "clear_to": cl_[1],
                               "q_end": q_e, "T_end": e_["T_e"], "T_rel": e_["T_r"], "T_tcp_rel": e_["T_tr"], "build": bool(build),
                               "insert": insert, "h_end": h_line_, "h_move": float((e_["T_g"] @ c_)[2] - (T_g0 @ c_)[2]),
                               "retreat_pred": rt_p,
                               "drop": {k_: v_ for k_, v_ in dw_.items() if k_ not in ("q_end", "T_tcp_end", "T_dish_end", "checked",
                                                                                     "variants", "release")},
                               "drop_checked": dw_["checked"],
                               **(dict(zip(("pitch_deg", "roll_deg", "tilt_deg"), (round(v_, 2) for v_ in side_pitch(e_["T_g"],
                                                                                                                    e_["T_tg"]))))
                                  if ring_side else {})})
            ranked.sort(key=lambda c: (c["drop"]["release_mm"], 0 if (c["drop"]["open"] or {}).get("open_mode") == "full" else 1,
                                       -(c["clear_m"] or 0.), c["above_m"], c["spin"]))
            drop_ = {"candidates": len(ranked), "tried": tried_, "tips": tips_, "rule": INSERT_DROP_RULE}
        if rule1 and not drop_:                        # rule 1: the largest held-dish clearance first, then the move
            ranked.sort(key=lambda c: (-c["clear_m"], c["move"], c["above_m"], c["spin"]))
        elif not drop_:                                # (the guided release keeps its own order, INSERT_DROP_RULE)
            ranked.sort(key=lambda c: (c["above_m"], c["move"], c["spin"]))   # D-ring: every start at 50 mm (the move decides)
        if ring_side and side_order == "pitch" and not drop_:   # plans/2026-11-04-ring-side-hold.md: the smallest pitch about the
            ranked = [ranked[i_] for i_ in pitch_order(ranked)]   # closing axis first (a roll of the hand where one exists), rule 1 within
        elif ring_side and not drop_:                  # --ring-side-order straight: rule 1's order, the pitch breaking its ties
            ranked.sort(key=lambda c: (-c["clear_m"], c["move"], c["pitch_deg"], c["above_m"], c["spin"]))
        return {"ranked": ranked, "counts": counts, "first_fail": first_fail, "seconds": round(time.monotonic() - t0_, 1),
                "spins": RING_SPINS, "retreat_rejects": rt_rejects if insert and rule1 else None, "drop": drop_,
                "rule": None if not rule1 else
                f"rule 1: every descent start {', '.join(f'{a * 1e3:.0f}' for a in aboves)} mm above the "
                f"{'build stop' if build else 'goal'} (x {RING_SPINS} spins) free with the held {kind} {CARRY_CLEAR_M * 1e3:.0f} mm "
                f"from the scene and the descent line clear; ranked by "
                + ("the goal tilt's pitch about the hand's closing axis (smallest first, 0.5 deg bins), then "
                   if ring_side and side_order == "pitch" else "")
                + f"the held {kind}'s clearance at the start (cut at {CLEAR_CUTOFF_M * 1e3:.0f} mm), then the largest joint move "
                  f"from the pick" + ("; the guarded insertion: released at the stop" if insert else ""),
                "build_release": None if not build else {k_: build.get(k_) for k_ in ("lowest_lift_mm", "release_lift_mm", "stop",
                                                                                      "start_lift_mm", "record")},
                "guarded_insert": insert}

    def ring_rec(c):
        rec_ = {"spin_deg": round(c["spin"], 1), "move_deg": round(float(np.degrees(c["move"])), 1),
                "descent_start_q_deg": [round(float(v), 1) for v in np.degrees(c["q_start"])], "tcp_above_mm": c["above_mm"],
                "h_start_mm": round(c["h_start"] * 1e3, 1), "descent_start_above_goal_mm": round(c["above_m"] * 1e3)}
        if c.get("clear_m") is not None:               # rule 1: the held dish's clearance at the start
            rec_.update(held_clear_mm=round(c["clear_m"] * 1e3, 1), held_clear_to=c.get("clear_to"))
        if c.get("pitch_deg") is not None:             # D-ring-side: the goal tilt split over the hand (side_pitch)
            rec_.update({k_: c[k_] for k_ in ("pitch_deg", "roll_deg", "tilt_deg")})
        if c.get("retreat_pred") is not None:          # D4: the plan's retreat at the predicted stop (the first clear variant)
            rec_["retreat_pred"] = c["retreat_pred"]["variant"]
            rec_["stop_pred_mm"] = (c["retreat_pred"].get("stop") or {}).get("h_mm")
            op_ = c["retreat_pred"].get("open") or {}  # the plan's open there (PARTIAL_OPEN_RULE)
            rec_["open_pred"], rec_["open_mm_pred"] = op_.get("open_mode"), op_.get("open_mm")
        if c.get("drop"):                              # INSERT_DROP_RULE: the guided release's plan
            rec_["drop"] = {k_: c["drop"].get(k_) for k_ in ("release_mm", "stop_mm", "drop_mm", "guided_mm", "variant", "tips_z_m",
                                                             "rim_low_z_m")}
            rec_["drop"]["open_mode"] = (c["drop"].get("open") or {}).get("open_mode")
        if c.get("build"):                             # the build release: the stop and the release above the move pose
            rec_.update(descent_start_above_stop_mm=round(c["above_m"] * 1e3), h_end_mm=round(c["h_end"] * 1e3, 1),
                        h_move_mm=round(c["h_move"] * 1e3, 1))
        if rel_h:                                      # D-ring-h100: the planned release height and the bowl-touch note
            rec_.update(release_h_mm=round(c.get("h_rel", 0.) * 1e3, 1), bowl_touch_mm=c.get("bowl_touch_mm"),
                        bowl_touch_rule="Kit-free note, not a criterion: the highest point of the descent line (mm above "
                                        "the goal) where the held bowl passes the bench checker's 1 mm allowance into the "
                                        "scene; None = clear")
        return rec_

    def ring_hulls():
        """The gripper at the F-out hold: the colliders at the last frame of the gate's median-pivot trial (wrist 3's frame):
        the load dir's FOOT_OUT_GATE_RUN when it has a held trial, else --foot-hold's (axial_ring_hulls; plans/2026-11-04-
        fifteen-of-fifteen.md). D-side-down: the mouth-down cup gate's (CUP_DOWN_GATE_RUN in the load dir) median-pivot held
        trial, else the cups round's, else the h45 gate's (cup_gate_hulls). The source in _closed["ring_src"]."""
        if "ring" not in _closed and side_down:        # round 3: the load row's hold_gate_run, else CUP_DOWN_GATE_RUN
            run_ = getattr(args, "hold_gate_run", None) or CUP_DOWN_GATE_RUN
            gd_ = out / run_ if (out / run_ / "trials.jsonl").is_file() else CUP_DOWN_GATE_DIR
            _closed["ring"], _closed["ring_src"] = cup_gate_hulls(C, np, gd_)
        if "ring" not in _closed and ring_side:        # the side ring hold's gate in the load dir, else the F-out hold's jaws
            _closed["ring"], _closed["ring_src"] = ring_tool_hulls(C, np, out, args.foot_hold)
        if "ring" not in _closed:                      # the axial ring hold: the load dir's F-out gate, else --foot-hold's
            _closed["ring"], _closed["ring_src"] = axial_ring_hulls(C, np, out, args.foot_hold)
        return _closed["ring"]

    def hold_source(oid):
        """plans/2026-11-04-fifteen-of-fifteen.md: the row's record of where the descent check's hand (the closed jaws rigid with
        wrist 3) came from: the load dir's round-1 Isaac gate run of this hold when it has a held trial, else today's source."""
        try:
            if ring or side_down:
                ring_hulls()
                src_ = _closed.get("ring_src")
            elif seat:
                src_ = seat_lib().get("closed_src")
            else:
                src_ = None
        except Exception as e_:                        # noqa: BLE001  a missing gate file: recorded, the row goes on
            src_ = f"unreadable: {type(e_).__name__}: {e_}"
        runs_ = (FOOT_SIDE_GATE_RUN if ring_side else FOOT_OUT_GATE_RUN if ring else
                 (getattr(args, "hold_gate_run", None) or CUP_DOWN_GATE_RUN) if side_down
                 else PLATE_EDGE_GATE_RUN if args.grasp_family == "edge" else PLATE_WALL_GATE_RUN if args.grasp_family == "plate-wall"
                 else None)
        return {"descent_hand": src_, "load_dir_gate": None if runs_ is None else rel_path(out / runs_),
                "load_dir_gate_present": None if runs_ is None else (out / runs_ / "trials.jsonl").is_file(),
                **({"side_below_top_m": round(float(args.side_below_top), 5),          # round 3: the cup's hold height and gate
                    "side_below_top_source": getattr(args, "side_below_top_source", None),   # run (the load row's, else the
                    "hold_gate_run": getattr(args, "hold_gate_run", None) or CUP_DOWN_GATE_RUN,   # move's defaults)
                    "hold_gate_run_source": "the load row's hold_gate_run" if getattr(args, "hold_gate_run", None)
                    else "CUP_DOWN_GATE_RUN"} if side_down else {}),
                "rule": "the load dir's Isaac grasp gate of this hold (round 1: partA_bowl_foot_out, partA_bowl_foot_side, "
                        "partA_cup_side_down11, partA_plate_wall20) when it has a held trial (its median-pivot trial's last "
                        "frame), else today's source (--foot-hold's placing_v1 F-out gate, the cups round's gate, the plates' "
                        "PLATE_GATE)"}

    def carry_metrics(k0, k1, oid, M):
        """Over the replay samples k0..k1 (the carry): the largest bowl axis z (< 0 = mouth down) and the smallest TCP height
        above the bowl's centroid (the hand above the bowl)."""
        c_ = np.r_[ns.centroids[kinds[oid]], 1.]
        az, hz = [], []
        for r_ in traj["rows"][k0:k1]:
            d_ = r_[3]
            T_d = T_of(d_[:3], d_[3:7])
            w3p, w3q = r_[1][rig.w3], r_[2][rig.w3]
            T_t = T_of(w3p, (w3q[1], w3q[2], w3q[3], w3q[0])) @ rig.T_w3_tcp
            az.append(float(T_d[2, 2]))
            hz.append(float(T_t[2, 3] - (T_d @ c_)[2]))
        return {"samples": len(az), "max_axis_z": round(max(az), 3) if az else None,
                "mouth_down_throughout": bool(az and max(az) < 0.),
                "min_tcp_above_centroid_mm": round(min(hz) * 1e3, 1) if hz else None,
                "hand_above_throughout": bool(hz and min(hz) > 0.),
                "rule": "every replay sample from the carry start to the descent start: the bowl axis z and the TCP height above "
                        "the bowl's centroid"}

    def ring_drop(i, oid, row, landed):
        """--carry above (rows D-ring): reset()'s ring_plan spins (up to DROP_TRIES) carried by transit_to (the rise from the
        counter pick to the safe height, the via-point above the descent start, down to it; straight joint legs first, RRT-Connect
        only when blocked); the pre-descent settle; the stop predicted from the measured poses (the welded bowl and the gripper at
        the F-out hold lowered straight down); lower_until_contact along the rack's up axis; the joint removed and the jaws
        opened (the jaw watch on from the open), RING_WAIT_TICKS, the open hand straight up RING_RISE_M, round 13's rest wait, the
        benchmark's settle; seated = bowl_seated_test on the refreshed goal AND mouth down; AT_GOAL beside it; a gripper-bowl
        contact after the open ramp (+ JAW_GRACE_TICKS) fails the row by rule ("jaw contact"), the physical outcome recorded.
        The guarded insertion (INSERT_MOVES): the descent bounded DESCENT_BELOW_M under the predicted stop only, the jaws opened
        at the sensed stop, the CUP_BACK_M back-off along -TCP z before the rise; no trip = "no sensor stop", the jaws closed.
        D4 (plans/2026-11-04-fifteen-of-fifteen.md, the coordinator): at the sensed stop, before the open, retreat_check (the open hand
        there judged too) must find a clear retreat, else the jaws stay closed ("no clear retreat"); after the open and the wait the
        retreat is the first of RETREATS clear in the episode's FCL model with the dish where it lies (none: "no clear retreat", the
        hand stays); the plan (ring_plan) keeps only the starts whose retreat is clear at the predicted stop. The open (PARTIAL_OPEN_RULE):
        before it, the full open first, else the partial open (the measured hold's pad gap + 5 mm a side); the jaws opened to that
        width, held there through the retreat; the row's "release" records it."""
        kind, M = kinds[oid], att["M"]
        kw = {"held": (kind, M, oid), "ignore": (oid,)}
        insert = bool(getattr(args, "guarded_insert", False))   # plans/2026-11-04-fifteen-of-fifteen.md
        cap_on = getattr(args, "tilt_cap", "on") != "off"      # --tilt-cap off: the round 1 measurement (no bowl tilt cap)
        before = backend.poses()
        sync_world()
        post.update(carry=True, start=rig.q())
        k0 = len(traj["rows"])
        tries, found, fails = 0, None, []
        why = f"no passing spin: {json.dumps(carry_state.get('counts'))}; {carry_state.get('first_fail')}"
        if carry_state.get("retreat_rejects") and not carry_state.get("ranked"):   # D4: every passing start's retreat blocked
            r0_ = carry_state["retreat_rejects"][0]
            why += (f"; D4: no clear retreat at any passing start (first: spin {r0_['spin_deg']:.0f}, +{r0_['start_mm']} mm: "
                    f"{retreat_brief(r0_)})")
        from dishsim_frigidaire.robot import kin
        c_ = np.r_[ns.centroids[kind], 1.]
        z_pick = float(carry_state.get("z_grasp", rig.tcp()[2, 3]))   # --attach none: the grasp's TCP z (the pick lifted)

        cap_ = {"z": 0.}                              # the bowl's tilt cap per descent start (bowl_tilt_cap_z)

        def straight_first(ranked, kw_, cap_d, kind_):
            """The coordinator's preference (2026-11-04): the passing descent starts whose carry legs from here (transit_to's
            legs: the arm already lifted, the target raised to z_safe in its IK branch when free, then down) are all straight
            joint moves under the carry rule, in rule 1's order, then the rest in rule 1's order. (ranked list, the record)"""
            t_ = time.monotonic()
            q0_ = rig.q()
            ok_, n_chk = [], 0
            arm.carry_ok = carry_ok
            try:
                for i_, cd_ in enumerate(ranked):
                    if kind_ == "bowl":
                        cap_d["z"] = bowl_tilt_cap_z(cd_["T_goal"])
                    ab_ = np.array(cd_["T_start"], copy=True)
                    ab_[2, 3] = flags.z_safe_m
                    qa_ = rig.ik(ab_, cd_["q_start"])
                    qa_ = qa_ if qa_ is not None and arm.free(qa_, **kw_) else None
                    lg_ = [(q0_, qa_), (qa_, cd_["q_start"])] if qa_ is not None else [(q0_, cd_["q_start"])]
                    n_chk += 1
                    if all(arm.path_free(a_, b_, step=.02, **kw_) for a_, b_ in lg_):
                        ok_.append(i_)
                        if len(ok_) >= DROP_TRIES:
                            break
            finally:
                arm.carry_ok = None
            order_ = (pitch_order(ranked, ok_) if ring_side and side_order == "pitch" else   # D-ring-side: straight first
                      ok_ + [i_ for i_ in range(len(ranked)) if i_ not in ok_])          # within each pitch bin
            rec_ = {"straight_candidates": len(ok_), "checked": n_chk, "ranked": len(ranked),
                    "rule1_rank_of_first": (order_[0] + 1) if order_ else None, "first_all_straight": bool(order_ and order_[0] in ok_),
                    "seconds": round(time.monotonic() - t_, 1),
                    **({"ring_side_order": side_order, "ring_side_order_source": side_order_src} if ring_side else {}),
                    "rule": "the coordinator's preference (2026-11-04): among the passing descent starts, those whose carry "
                            "legs are all straight joint moves (checked every 0.02 rad under the carry rule) first, then rule "
                            "1's order (the largest held-dish clearance, then the move)"}
            return [ranked[i_] for i_ in order_], rec_

        def carry_ok(q):                               # the plan: the bowl mouth down throughout, the hand above it
            T_t = rig.T_wb @ kin.fk_all_links(np.asarray(q, dtype=float))["wrist_3_link"] @ rig.T_w3_tcp
            T_b = T_t @ M
            return bool(T_b[2, 2] < 0. and T_t[2, 3] > (T_b @ c_)[2] and (kind != "bowl" or not cap_on or T_b[2, 2] <= cap_["z"]))
        rk_all = carry_state.get("ranked") or []
        if rk_all and rk_all[0].get("drop"):           # INSERT_DROP_RULE: the guided release's own order (the lowest release first)
            ranked_, choice_ = list(rk_all), {"drop_order": True, "ranked": len(rk_all),
                                              "rule": "the guided release's candidates in their order (INSERT_DROP_RULE): the lowest "
                                                      "release height, full before partial, the held clearance, the start"}
        else:
            ranked_, choice_ = straight_first(rk_all, kw, cap_, kind)
        order_all = list(ranked_)                      # every passing descent start in the carry's order (the narrow retry's goals)
        ranked_ = ranked_[:DROP_TRIES]
        row["start_choice"] = choice_
        retry_ = {"budget_s": None, "done": False}
        tried_ = []
        nr_ = getattr(ns, "narrow", None)              # plans/2026-11-04-fifteen-of-fifteen.md: plan_to's own narrow retry is off
        nr_on = nr_["on"] if nr_ else None             # over the tries (a 180 s retry per start would multiply); the carry's
        if nr_:                                        # own runs once after them, rooted at every passing start
            nr_["on"] = False
        try:
            while ranked_:
                cb = ranked_.pop(0)
                tried_.append(cb)
                tries += 1
                tlog.set(phase="carry to the descent start" + (f" (retry, RRT {retry_['budget_s']:.0f} s)" if retry_["done"] else ""))
                ns.plan_log.clear()
                if kind == "bowl":                         # plans/2026-11-04-s0-rerun.md: the tilt cap at this start
                    cap_["z"] = bowl_tilt_cap_z(cb["T_goal"])
                arm.carry_ok = carry_ok                    # every leg's configs (straight checks and RRT validity)
                lifted = float(rig.tcp()[2, 3]) > z_pick + RING_LIFT_M - .010      # a retry starts where the last try left it
                try:
                    w_ = transit_to(cb["T_start"], q_hover=cb["q_start"], rise_m=0. if lifted else RING_LIFT_M, **kw)
                finally:
                    arm.carry_ok = None
                if w_ is None:
                    found = cb
                    row.update(combo=ring_rec(cb), combo_rank=tries, legs={"plans": list(ns.plan_log)},
                               lift_mm=round(RING_LIFT_M * 1e3))
                    break
                fails.append(f"spin {cb['spin']:.0f}: {w_} (legs {list(ns.plan_log)})")
                why = f"no path to {tries} descent start(s), the last: {fails[-1]}"
                if (not ranked_ and kind == "bowl" and att_mode["none"] and not retry_["done"]
                        and any("rrt: none" in f_ for f_ in fails)):   # one retry at RING_RETRY_RRT_S (compute only): the first start
                    c0_ = next(c_ for c_, f_ in zip(tried_, fails) if "rrt: none" in f_)
                    retry_.update(budget_s=RING_RETRY_RRT_S, done=True, budget_before_s=args.transit_rrt_s, spin=c0_["spin"])
                    args.transit_rrt_s = RING_RETRY_RRT_S  # whose RRT found no path, from where the arm is now
                    ranked_ = [c0_]
        finally:
            if nr_:
                nr_["on"] = nr_on
        narrow_s = narrow_budget(args)
        if found is None and att_mode["none"] and narrow_s > 0 and any("rrt: none" in f_ for f_ in fails):
            c0_ = next(c_ for c_, f_ in zip(tried_, fails) if "rrt: none" in f_)   # the narrow passage (plans/2026-11-04-fifteen-of-
            all_ = [c0_] + [c_ for c_ in order_all if c_ is not c0_]             # fifteen.md): from where the arm is, one search
            goals_ = [np.asarray(c_["q_start"], dtype=float) for c_ in all_]      # of narrow_s rooted at EVERY passing descent
            if kind == "bowl":                         # start (the first whose RRT found none first); one tilt cap for all: a
                cap_["z"] = bowl_tilt_cap_z(c0_["T_goal"])   # spin turns the bowl about its own axis, its tilt stays
            q0_ = rig.q()
            tlog.set(phase=f"carry to the descent start (narrow retry, RRT {narrow_s:.0f} s, {len(goals_)} starts)")
            ns.plan_log.clear()
            arm.carry_ok = carry_ok
            t_n = time.monotonic()
            try:
                path_ = C.rrt_connect(arm, q0_, goals_, time_s=0., collide_kw=kw, shortcut_s=args.shortcut_s, retry_s=narrow_s)
            finally:
                arm.carry_ok = None
            k_ = C.reached_goal(path_, goals_)
            nr_rec = {"budget_s": narrow_s, "goals": len(goals_), "reached": k_, "found": path_ is not None,
                      "seconds": round(time.monotonic() - t_n, 1), "first_spin_deg": c0_["spin"],
                      "first_above_mm": round(c0_["above_m"] * 1e3), "from_q_deg": [round(float(v), 1) for v in np.degrees(q0_)],
                      "rule": "after every try's RRT-Connect (and a bowl's RING_RETRY_RRT_S retry) found no path: one "
                              "collide.rrt_connect search of --narrow-retry-s (retry_s, seed 2) from where the arm is, rooted at "
                              "every passing descent start (goals[0] = the first start whose RRT found none, then the carry's "
                              "order), the carry rule and the margins unchanged; reached = collide.reached_goal's index"}
            if path_ is not None:
                cb_ = all_[k_]
                nr_rec.update(reached_spin_deg=cb_["spin"], reached_above_mm=round(cb_["above_m"] * 1e3), waypoints=len(path_))
                ns.plan_log.append("rrt (narrow retry)")
                ns.leg_now["leg"] = f"leg {len(ns.plan_log)} (rrt, narrow retry)"
                try:
                    ok_ = rig.follow(path_, steps_per=10, settle_tol=flags.tol_rrt)
                finally:
                    ns.leg_now["leg"] = None
                nr_rec["followed"] = bool(ok_)
                tries += 1
                if ok_:
                    found = cb_
                    row.update(combo=ring_rec(cb_), combo_rank=tries, legs={"plans": list(ns.plan_log)},
                               lift_mm=round(RING_LIFT_M * 1e3))
                else:
                    fails.append(f"narrow retry to spin {cb_['spin']:.0f}: blocked in Isaac {getattr(rig, 'last_fail', None)}")
                    why = f"no path to {tries} descent start(s), the last: {fails[-1]}"
            else:
                ns.plan_log.append("rrt: none (narrow retry)")
            row["narrow_retry"] = nr_rec
        if kind == "bowl" and (carry_state.get("ranked") or []):   # the coordinator's option 1 (2026-11-04): the cap per bowl
            c1_ = found or carry_state["ranked"][0]
            z1_ = float(c1_["T_goal"][2, 2])
            row["tilt_cap"] = {"cap_deg": round(math.degrees(math.acos(min(1., -(bowl_tilt_cap_z(c1_["T_goal"]) - 1e-4)))), 2),
                               "goal_tilt_deg": round(math.degrees(math.acos(min(1., -z1_))), 2),
                               "flat_cap_deg": BOWL_CARRY_TILT_MAX_DEG, "on": cap_on,
                               "rule": "the held bowl's tilt from straight down at most max(20 deg, its tilt at the descent start) "
                                       "on every carry configuration (straight-leg checks, RRT samples and edges)" + (
                                           "" if cap_on else "; OFF for this run (--tilt-cap off, the round 1 measurement of "
                                           "plans/2026-11-04-fifteen-of-fifteen.md): the carry rule is the bowl mouth down and the "
                                           "hand above it only, cap_deg is what the cap would have been")}
        if retry_["done"]:
            args.transit_rrt_s = retry_["budget_before_s"]
            row["carry_retry"] = {k_: retry_.get(k_) for k_ in ("budget_s", "budget_before_s", "spin")}
            row["carry_retry"]["found"] = found is not None
        row.update(drop_tries=tries, carry_fails=fails)
        if found is None and att_mode["none"]:         # plans/2026-11-04-grasp-place-s0.md: no teleport; the bowl is parked
            post["carry"] = False
            if retry_["done"]:
                why += f"; one retry at RRT-Connect {RING_RETRY_RRT_S:.0f} s (spin {retry_['spin']:.0f}) found no path either"
            nr_rec = row.get("narrow_retry")
            if nr_rec is not None:
                why += (f"; the narrow retry ({nr_rec['budget_s']:.0f} s, rooted at {nr_rec['goals']} starts) "
                        + ("found no path either" if not nr_rec["found"] else "found a path, blocked in Isaac"))
            return fail(row, "no reach", why + f"; the {kind} stays in the hand (parked off the scene after the row)")
        if found is None:
            post["carry"] = False
            harness.armed = False
            detach()
            harness.clear_carried()
            T_g_, rest_ = place_at_goal(oid)
            landed.append(oid)
            row.update(placed_at_goal=True, final_pose=pose(T_g_), rest=rest_)
            return fail(row, "no reach", why + "; placed at its goal pose (teleported, at rest) for the bowls after it")
        cb = found
        row["carry"] = carry_metrics(k0, len(traj["rows"]), oid, M)
        h_rel = cb.get("h_rel", 0.) if rel_h else None  # D-ring-h100 (plans/2026-10-30-bowl-drop100.md): the release height
        row.update(spin_deg=cb["spin"], descent_start_mm=round(((h_rel or 0.) + cb["above_m"]) * 1e3),
                   up_axis=[round(float(x), 5) for x in cb["up"]])
        if h_rel is not None:
            row["release_h_plan_mm"] = round(h_rel * 1e3, 1)
        n_hold, lag0 = 0, rig.lag()                    # the pre-descent settle (round 13)
        while rig.lag() > PRE_DESCENT_LAG_RAD and n_hold < PRE_DESCENT_CAP:
            rig.step(1)
            n_hold += 1
        T_m, T_b = rig.tcp(), T_now(oid)
        h_s, body_s, tgt_s, h_bw, h_tl, h_now = predict_stop(oid, T_m, T_b, ring_hulls())
        row["pre_descent"] = {"ticks": n_hold, "lag_before_rad": round(float(lag0), 4), "lag_after_rad": round(float(rig.lag()), 4),
                              "tcp_vs_plan_mm": round(float(np.linalg.norm(T_m[:3, 3] - np.asarray(cb["T_start"])[:3, 3])) * 1e3, 1),
                              "bowl_h_mm": round(h_now * 1e3, 1)}
        row["stop_pred"] = {"kind": body_s, "height_mm": None if h_s is None else round(h_s * 1e3, 1), "target": tgt_s,
                            "bowl_mm": None if h_bw is None else round(h_bw * 1e3, 1),
                            "tool_mm": None if h_tl is None else round(h_tl * 1e3, 1),
                            "rule": "the welded bowl and the gripper at the F-out hold (the gate's last frame) lowered straight "
                                    "down from the measured TCP / bowl poses after the pre-descent settle; heights = the bowl "
                                    "centroid above the refreshed goal's", "tool_hulls": _closed.get("ring_src", "the F-out gate")}
        h_ref = h_s if h_s is not None else 0.
        carry_watch.update(phase="descent", arrived=True)   # --attach none: the carry ends at the descent start
        if insert:                                     # plans/2026-11-04-fifteen-of-fifteen.md: the guarded insertion, bounded
            h_ins = h_s if h_s is not None else cb["h_end"]   # DESCENT_BELOW_M under the predicted stop (none predicted: the
            if cb.get("build"):                        # build's stop, or the goal at a goal site), never at the move pose
                bd_ = meta[oid]["build_release"]
                row["build_release"] = {k_: bd_.get(k_) for k_ in ("lowest_lift_mm", "release_lift_mm", "stop", "start_lift_mm",
                                                                  "record", "move_pose")}
                row["build_release"].update(
                    stop_above_goal_mm_plan=round(cb["h_end"] * 1e3, 1), move_above_goal_mm=round(cb["h_move"] * 1e3, 1),
                    rule="the dish in the build's move orientation (moves_world), lowered along the rack's up axis (the build's "
                         "insertion axis) until the flange sensor trips, the jaws opened there (the guarded insertion: no raise, "
                         "no position bound at the move pose), the hand backed off along its approach and risen, settled by PhysX")
            row["guarded_insert"] = {
                "bound_above_goal_mm": round(h_ins * 1e3, 1),   # round 3 (INSERT_STOP_RULE): the bound is the predicted stop
                "bound_from": ("the predicted stop" if h_s is not None else "the build's stop (no stop predicted)"
                               if cb.get("build") else "the goal (no stop predicted)"),
                "stop_mode": "predicted", "stop_rule": INSERT_STOP_RULE,
                "back_mm": round(CUP_BACK_M * 1e3), "rise_mm": round(RING_RISE_M * 1e3),
                "rule": f"lowered along the rack's up axis at {DESCENT_SPEED_M_S * 1e3:.0f} mm/s until the flange sensor trips "
                        f"(> {SENSOR_TRIP_N} N on {SENSOR_TRIP_TICKS} ticks), bounded at the predicted stop (round 3); the jaws "
                        f"opened at the stop after D4's pre-open check, {RING_WAIT_TICKS} ticks, the first clear retreat (a: the "
                        f"open hand back {CUP_BACK_M * 1e3:.0f} mm along -TCP z, then up {RING_RISE_M * 1e3:.0f} mm); no trip by "
                        f"the predicted stop: a position stop there, released the same way (INSERT_STOP_RULE); an IK end or a "
                        f"block before it: 'no sensor stop', the jaws stay closed"}
            if cb.get("drop"):                         # INSERT_DROP_RULE: the descent ends at the planned release by position
                row["guarded_insert"].update(drop_release_mm=cb["drop"]["release_mm"],
                                             drop_rule="the guided release: bounded by position at the planned release height "
                                                       "(release_mm above the goal) or an earlier sensor trip")
            stop = lower_until_contact(oid, {"T_start": T_m, "q_start": rig.q(), "h_start": h_now, "h_b": h_ins,
                                             "dir": cb["up"], **({"h_end": cb["drop"]["h_end_m"]} if cb.get("drop") else
                                                                 {"h_end": h_ins, "bound_rule": "at the predicted stop "
                                                                  f"({h_ins * 1e3:.1f} mm above the goal; INSERT_STOP_RULE)"})})
            if (not cb.get("drop") and stop["kind"] == "none" and stop.get("ik_end_mm") is None
                    and not stop.get("blocked")):     # INSERT_STOP_RULE: no trip by the predicted stop = a position stop there
                stop["kind"] = "predicted"
                stop["predicted"] = {"planned_mm": round(h_ins * 1e3, 1), "actual_mm": stop["bowl_h_mm"],
                                     "diff_mm": round(stop["bowl_h_mm"] - h_ins * 1e3, 1),
                                     "from": row["guarded_insert"]["bound_from"], "peak_N": stop.get("peak_N"),
                                     "rule": INSERT_STOP_RULE}
        elif cb.get("build"):                          # the build release: lowered until the sensor trips, bounded at the
            bd_ = meta[oid]["build_release"]           # build's move pose (its lowest pose, lift 0)
            row["build_release"] = {k_: bd_.get(k_) for k_ in ("lowest_lift_mm", "release_lift_mm", "stop", "start_lift_mm",
                                                              "record", "move_pose")}
            row["build_release"].update(
                stop_above_goal_mm_plan=round(cb["h_end"] * 1e3, 1), move_above_goal_mm=round(cb["h_move"] * 1e3, 1),
                rule="the dish in the build's move orientation (moves_world), lowered along the rack's up axis until the flange "
                     "sensor trips (bounded by position at the move pose, the build's lowest pose), raised "
                     f"{BUILD_UP_M * 1e3:.0f} mm, let go, the hand away (as the build: released 20 mm above its lowest clear "
                     "pose, settled by PhysX)")
            stop = lower_until_contact(oid, {"T_start": T_m, "q_start": rig.q(), "h_start": h_now,
                                             "h_b": cb["h_move"] + DESCENT_BELOW_M, "dir": cb["up"]})
        elif h_rel is None:                            # D-ring: lowered until the sensor trips (round 17)
            stop = lower_until_contact(oid, {"T_start": T_m, "q_start": rig.q(), "h_start": h_now, "h_b": h_ref,
                                             "dir": cb["up"]})
        else:                                          # D-ring-h100: by position to the goal + h; an earlier trip ends it there
            stop = lower_until_contact(oid, {"T_start": T_m, "q_start": rig.q(), "h_start": h_now,
                                             "h_b": h_rel + DESCENT_BELOW_M, "dir": cb["up"]})
        post["carry"] = False
        meas = stop["bowl_h_mm"]
        pred = None if h_s is None else round(h_s * 1e3, 1)
        row.update(stop={**stop, "height_mm": meas}, descent_mm=stop["descent_mm"], weld_drift_mm=stop["weld_drift_mm"],
                   stop_check={"pred_mm": pred, "pred_kind": body_s, "measured_mm": meas, "measured_kind": stop["kind"],
                               "within_2mm": None if pred is None or stop["kind"] == "none" else bool(abs(meas - pred) <= 2.),
                               "body_match": None if stop["kind"] in ("none", "predicted") else bool(stop["kind"] == body_s)})
        if h_rel is not None:                          # the stop vs the plan (the release height); the sensor's prediction kept
            plan_mm = round(h_rel * 1e3, 1)
            row["stop_check"] = {"plan_mm": plan_mm, "measured_mm": meas, "measured_kind": stop["kind"],
                                 "by": "position" if stop["kind"] == "none" else f"sensor ({stop['kind']})",
                                 "within_2mm": bool(abs(meas - plan_mm) <= 2.), "pred_mm": pred, "pred_kind": body_s,
                                 "bowl_touch_mm_kitfree": cb.get("bowl_touch_mm"),
                                 "rule": "plan = the release height h above the refreshed goal (the bowl centroid, along the "
                                         "rack's up axis); the descent ends there by position unless the flange sensor trips "
                                         "first (pred = the first contact of the bowl and the closed jaws lowered straight "
                                         "down from the measured pose, Kit-free)"}
        if cb.get("build"):                            # the stop vs the build's stop; then the dish raised BUILD_UP_M
            row["stop_check"] = {"plan_mm": round(cb["h_end"] * 1e3, 1), "measured_mm": meas, "measured_kind": stop["kind"],
                                 "by": ("no trip (the guarded insertion's bound)" if insert else "position (the move pose)")
                                 if stop["kind"] == "none" else "position (the predicted stop, INSERT_STOP_RULE)"
                                 if stop["kind"] == "predicted" else f"sensor ({stop['kind']})",
                                 "within_2mm": bool(abs(meas - cb["h_end"] * 1e3) <= 2.), "pred_mm": pred, "pred_kind": body_s,
                                 "rule": "plan = the build's stop (its lowest clear pose, lowest_lift above the move pose), the "
                                         "dish centroid above the goal's (mm, world z); measured = at the sensed stop"}
        drop_stop = bool(insert and cb.get("drop") and stop["kind"] == "none" and stop.get("ik_end_mm") is None
                         and not stop.get("blocked"))  # INSERT_DROP_RULE: the planned position stop at the release height
        if drop_stop:                                  # the guidance re-measured at the dish's actual pose
            tz_ = cb["drop"].get("tips_z_m")
            low_ = dish_low_z(T_now(oid), ns.points[kind], np)
            g_now_ = None if tz_ is None else round((tz_ - low_) * 1e3, 1)
            row["drop_release"] = {"release_mm_plan": cb["drop"]["release_mm"], "release_mm": meas, "stop_mm": cb["drop"]["stop_mm"],
                                   "guided_mm": g_now_, "guided_mm_plan": cb["drop"]["guided_mm"], "tips_z_m": tz_,
                                   "rim_low_z_m": round(low_, 5), "rule": INSERT_DROP_RULE}
            if g_now_ is None or g_now_ < DROP_GUIDED_MIN_M * 1e3:
                return fail(row, "not tine-guided", f"at the guided release's position stop ({meas} mm above the goal) the {kind}'s "
                                                    f"lowest point is {g_now_} mm below the wire tips (at least "
                                                    f"{DROP_GUIDED_MIN_M * 1e3:.0f} mm); the jaws stay closed, the {kind} is parked "
                                                    f"after the row")
        if insert and stop["kind"] == "none" and not drop_stop:   # never a position stop: the jaws stay closed, the dish is parked after
            end_ = (f"the descent IK ended {stop['ik_end_mm']} mm down" if stop.get("ik_end_mm") is not None else   # the row
                    "the arm was blocked (lag)" if stop.get("blocked") else f"the bound ({stop['bound_mm']} mm) was reached")
            return fail(row, "no sensor stop", f"the guarded insertion ended without a sensor trip: {end_} after "
                                               f"{stop['descent_mm']} mm of descent, the {kind} {meas} mm above its goal (peak "
                                               f"{stop['peak_N']} N; predicted stop {pred} mm, {body_s}); the jaws stay closed (never "
                                               f"a position stop, never a free fall), the {kind} is parked after the row")
        open_, op_full = None, {}                      # the open at the release (PARTIAL_OPEN_RULE): the full open, else the partial

        def jaw_now():                                 # finger_joint's measured position (rad, 0 = open)
            return float(rig.robot.data.joint_pos.torch[0, rig.fj])
        if insert:                                     # D4 before the open: the open hand at the sensed stop and a clear retreat
            tlog.set(phase="retreat check (before the open)")   # from it, the dish held where it stopped; else the jaws stay closed
            sync_world()
            jaw0_ = jaw_now()                          # the hold now: the partial open's width (the pad gap at it + 5 mm a side)

            def run_partial():
                if "p" not in op_full:
                    op_full["p"] = partial_open(C, np, ring_hulls(), theta_hold=jaw0_)
                return op_full["p"]
            rt0_ = retreat_check_open(arm, rig.ik, rig.q(), rig.tcp(), cb["up"], scene_targets(arm), np, dish=oid, judge_release=True,
                                      partial=run_partial)
            open_ = rt0_["open"]
            row["retreat_before_open"] = {"variant": rt0_["variant"], "release": rt0_["release"], "variants": rt0_["variants"],
                                          "rule": rt0_["rule"], "open": open_, "modes": rt0_.get("modes"),
                                          "jaw_at_stop_rad": round(jaw0_, 4), "open_rule": PARTIAL_OPEN_RULE}
            if rt0_["variant"] is None:
                pm_ = (rt0_.get("modes") or {}).get("partial")
                return fail(row, "no clear retreat", f"at the {'predicted (position) ' if stop['kind'] == 'predicted' else 'sensed '}"
                                                     f"stop ({meas} mm above the goal), before the open: "
                                                     f"{retreat_brief(rt0_)}" + (f"; the partial open: {pm_.get('why')}" if pm_ else "")
                                                     + f"; the jaws stay closed (nothing released), the {kind} is parked after the row")
        if cb.get("build") and not insert:
            tlog.set(phase=f"up {BUILD_UP_M * 1e3:.0f} mm before the release")
            T0_ = rig.tcp()
            T_up_ = T0_.copy()
            T_up_[:3, 3] += BUILD_UP_M * np.asarray(cb["up"], dtype=float)
            qs_up_ = rig.line_q(T0_, T_up_, 13, rig.q())
            up_ok_ = None if qs_up_ is None else bool(rig.follow(qs_up_, settle_tol=flags.tol_rise))
            T_b_ = T_now(oid)
            row["up_before_release"] = {"followed": up_ok_, "tcp_up_mm": round(float((rig.tcp()[:3, 3] - T0_[:3, 3]) @ cb["up"]) * 1e3, 1),
                                        "dish_up_mm": round((float((T_b_ @ c_)[2]) - float((goal[oid] @ c_)[2])) * 1e3 - meas, 1),
                                        "in_hand_mm": carry_watch.get("last_mm")}
        T_r = T_now(oid)
        row["release_pose"] = pose(T_r)
        if cb.get("build"):                            # measured: the dish centroid above the goal's and above the move pose
            row["release_h_mm"] = round((float((T_r @ c_)[2]) - float((goal[oid] @ c_)[2])) * 1e3, 1)
            row["build_release"]["release_above_move_mm"] = round(row["release_h_mm"] - cb["h_move"] * 1e3, 1)
        if h_rel is not None:                          # measured: the bowl centroid above the goal's when the jaws open
            c_r = np.r_[ns.centroids[kind], 1.]
            row["release_h_mm"] = round((float((T_r @ c_r)[2]) - float((goal[oid] @ c_r)[2])) * 1e3, 1)
        detach()
        harness.clear_carried()
        jaw_watch.update(on=True, oid=oid, hits=[])
        tlog.set(phase="release")
        t_open0 = backend.tick_index
        partial_ = bool(open_ and open_.get("open_mode") == "partial")
        if partial_:                                   # PARTIAL_OPEN_RULE: finger_joint to the chosen width (grasp.theta_at_inner)
            rig.gripper(False, to=open_["open_theta_rad"])
        else:
            rig.gripper(False)
        retreat.clear()
        retreat.update(open_start_tick=t_open0, open_end_tick=backend.tick_index)
        tlog.set(phase="wait")
        rig.step(RING_WAIT_TICKS)
        if partial_:                                   # the width held before the hand moves: finger_joint within PARTIAL_OPEN_TOL_RAD
            n_w = 0                                    # of its target, at most PARTIAL_OPEN_WAIT_TICKS more
            while abs(jaw_now() - open_["open_theta_rad"]) > PARTIAL_OPEN_TOL_RAD and n_w < PARTIAL_OPEN_WAIT_TICKS:
                rig.step(1)
                n_w += 1
            retreat.update(open_hold_ticks=n_w, jaw_at_retreat_rad=round(jaw_now(), 4))
        if insert:                                     # the row's release record (PARTIAL_OPEN_RULE): the open used and the plan's
            row["release"] = {**(open_ or {}), "planned": (cb.get("retreat_pred") or {}).get("open"),
                              "jaw_at_stop_rad": row["retreat_before_open"]["jaw_at_stop_rad"],
                              "jaw_after_open_rad": round(jaw_now(), 4), "rule": PARTIAL_OPEN_RULE,
                              "mode": "drop" if drop_stop else "seat", "planned_mode": "drop" if cb.get("drop") else "seat"}
            if cb.get("drop"):                         # INSERT_DROP_RULE: the guided release's plan and what Isaac did
                dp_ = cb["drop"]
                row["release"].update(release_mm=dp_["release_mm"], release_mm_actual=meas, stop_mm=dp_["stop_mm"],
                                      drop_mm=dp_["drop_mm"], guided_mm=(row.get("drop_release") or {}).get("guided_mm"),
                                      guided_mm_plan=dp_["guided_mm"], first_violation=dp_["first_violation"],
                                      variant=dp_["variant"], tips_z_m=dp_["tips_z_m"],
                                      rim_low_z_m=(row.get("drop_release") or {}).get("rim_low_z_m", dp_["rim_low_z_m"]),
                                      drop_rule=INSERT_DROP_RULE)
        v_rt = "a" if (side_down or ring_side or insert) else "c"   # today's retreat: the back-off then the rise (the side holds and
        if insert:                                     # the insertion), the rise alone (D-ring); D4 at an insert site: the first clear of
            tlog.set(phase="retreat check")            # (a), (b), (c) in the episode's FCL model, before the hand moves
            sync_world()                               # the released dish where it lies now; the open's own hull (the partial open's
            keep_ = arm.hulls["gripper"]               # when it was chosen)
            if partial_:
                arm.hulls["gripper"] = op_full["p"]["hulls"]
            try:
                rt_ = retreat_check(arm, rig.ik, rig.q(), rig.tcp(), cb["up"], scene_targets(arm), np, dish=oid)
            finally:
                arm.hulls["gripper"] = keep_
            v_rt = rt_["variant"]
            retreat.update(variant=v_rt, variants=rt_["variants"], release_stage=rt_["release"], check_rule=rt_["rule"],
                           planned_variant=(cb.get("retreat_pred") or {}).get("variant"))
            if v_rt is None:                           # never a colliding retreat: the hand stays, the dish is parked after the row
                jaw_watch["on"] = False
                row.update(final_pose=pose(T_now(oid)))
                row["retreat"] = {**retreat, "rule": "D4: no retreat variant clear after the open; none executed"}
                return fail(row, "no clear retreat", f"after the open at the sensed stop no retreat is clear (episode FCL): "
                                                     f"{retreat_brief(rt_)}; the hand stays, the {kind} is parked after the row")

        def back_leg():                                # the open hand back CUP_BACK_M along -TCP z (its approach: horizontal for the
            tlog.set(phase="back-off")                 # side holds, up the bowl's axis for the axial ring hold)
            T0 = rig.tcp()
            T1 = T0.copy()
            T1[:3, 3] -= CUP_BACK_M * T0[:3, 2]
            qs_bk = rig.line_q(T0, T1, 20, rig.q())
            retreat.update(back_mm=round(CUP_BACK_M * 1e3), back=None if qs_bk is None else bool(rig.follow(qs_bk)))

        def rise_leg(ax=None):                         # the open hand up RING_RISE_M: straight up, or along ``ax`` (D4's (b))
            tlog.set(phase="rise")
            T0 = rig.tcp()
            T1 = T0.copy()
            if ax is None:
                T1[2, 3] += RING_RISE_M
            else:
                T1[:3, 3] += RING_RISE_M * np.asarray(ax, dtype=float) / np.linalg.norm(ax)
            qs_up = rig.line_q(T0, T1, 40, rig.q())
            retreat.update(rise_axis="straight up" if ax is None else "the insertion axis",
                           rise=None if qs_up is None else bool(rig.follow(qs_up)))
        if v_rt == "a":
            back_leg()
            rise_leg()
        elif v_rt == "b":
            rise_leg(cb["up"])
            back_leg()
        else:
            rise_leg()
        retreat.update(wait_ticks=RING_WAIT_TICKS, rise_mm=round(RING_RISE_M * 1e3))
        ring_finish(i, oid, row, landed, before, T_r)

    def ring_finish(i, oid, row, landed, before, T_r):
        """D-ring after the rise: round 13's rest wait, the benchmark's settle, the judgement (roll_finish without the roll)."""
        kind = kinds[oid]
        insert = bool(getattr(args, "guarded_insert", False))
        row["retreat"] = {**retreat, "rule": ("the jaws opened where the flange sensor stopped the descent (the guarded insertion), "
                                              "0.5 s, " if insert else
                                              f"the dish raised {BUILD_UP_M * 1e3:.0f} mm from the sensed stop, then the jaws "
                                              f"opened (the build release), 0.5 s, " if row.get("build_release") else
                                              "the jaws opened where the descent stopped, 0.5 s, ") + (
            f"then retreat ({retreat.get('variant')}) {RETREAT_RULES.get(retreat.get('variant'), '')}, the first of (a), (b), (c) "
            f"clear in the episode's FCL model before the hand moved (D4)" if insert else
            f"the open hand back {CUP_BACK_M * 1e3:.0f} mm along -TCP z, then the open hand straight up 100 mm"
            if side_down or ring_side else "the open hand straight up 100 mm")}
        rest = rest_wait([oid])
        s_ = ns.bench_settle(oid, "goal", T_r, before)
        jaw_watch["on"] = False
        T = T_now(oid)
        if (row.get("stop") or {}).get("kind") == "predicted":   # INSERT_STOP_RULE: the slide the settle produced, open to rest
            c_sl = np.r_[ns.centroids[kind], 1.]
            d_sl = (T @ c_sl)[:3] - (T_r @ c_sl)[:3]
            row["stop"]["slide"] = {"centroid_mm": round(float(np.linalg.norm(d_sl)) * 1e3, 1), "dz_mm": round(float(d_sl[2]) * 1e3, 1),
                                    "lateral_mm": round(float(np.linalg.norm(d_sl[:2])) * 1e3, 1),
                                    "axis_turn_deg": round(float(np.degrees(np.arccos(np.clip(float(T_r[:3, 2] @ T[:3, 2]), -1., 1.)))), 2),
                                    "rule": "the dish's centroid motion and its axis turn from its pose when the jaws opened to its "
                                            "pose after the rest wait and the settle"}
        st = s_["settle"]
        settled = st["drift_mm"] <= ns.R.STABLE_POS_M * 1e3 and st["drift_deg"] <= ns.R.STABLE_ROT_DEG
        dist = s_["distance"]
        sync_world()
        rk = meta[oid]["rack"]
        now_ = backend.poses()
        F_r = T_of(now_[rk]["position_m"], now_[rk]["quaternion_xyzw"])
        if side_down:                                  # the cups' seated test (plans/2026-11-04-grasp-place-s0.md, unchanged)
            ps_ = pose_seated(T, F_r @ inv(F_goal[rk]) @ goal[oid], ns.centroids[kind], np)
            sw = {"seated_wires": bool(ps_[0]), "goal_pose_test": {"ok": ps_[0], "centroid_mm": ps_[1], "axis_deg": ps_[2]},
                  "rule": "the cup settled with its centroid within 2 mm and its axis within 5 deg of the goal pose in the "
                          "rack's frame now (frigidaire_drop_curve.py's cup rule), and mouth down"}
        else:
            sw = bowl_seated_test(arm, kind, oid, T, oid, np, others={n_: now_[n_] for n_ in landed if n_ != oid},
                                  T_goal=F_r @ inv(F_goal[rk]) @ goal[oid], centroid=ns.centroids[kind])
        mouth = "up" if T[2, 2] > 0 else "down"
        t_after = (retreat.get("open_start_tick") or 0) + OPEN_STEPS_TICKS + JAW_GRACE_TICKS
        hits = list(jaw_watch["hits"])
        after = [h for h in hits if h[0] > t_after]
        during = [h for h in hits if h[0] <= t_after]
        row.update(settled=bool(settled), settle=st, rest_wait=rest, final_pose=pose(T), mouth=mouth, axis_z=round(float(T[2, 2]), 3),
                   error={"lateral_mm": dist[0], "dz_mm": dist[1], "tilt_deg": dist[2]},
                   at_goal=bool(ns.B.within_goal(kind, (dist[0] / 1e3, dist[1] / 1e3, dist[2]))),
                   lean_deg={"final": lean_deg(T, np), "goal": lean_deg(goal[oid], np)}, lean_from_goal_deg=dist[2],
                   seated={**sw, "seated": bool(sw["seated_wires"] and T[2, 2] < 0)},
                   jaw_contact={"after_open": sorted({h[1] for h in after}), "after_open_ticks": len(after),
                                "first_after_tick": after[0][0] - (retreat.get("open_start_tick") or 0) if after else None,
                                "peak_N": max((h[2] for h in after), default=0.), "during_open": sorted({h[1] for h in during}),
                                "rule": f"gripper (pad, finger, palm) vs bowl contacts every tick from the open on; 'after the "
                                        f"open' = after the {OPEN_STEPS_TICKS}-tick open ramp + {JAW_GRACE_TICKS} ticks"},
                   landed_on=sorted(b if a == oid else a for a, b in harness.contacts.pairs_with(oid)),
                   moved_earlier=[m for m in s_["rules"]["strict"] if m[0] in landed])
        landed.append(oid)
        outcome = (f"physically: {'settled' if settled else 'not settled'}, mouth {mouth}, "
                   f"{'seated' if row['seated']['seated'] else 'not seated'}, {'at goal' if row['at_goal'] else 'off goal'}")
        row["outcome_physical"] = outcome
        if after:
            fail(row, "jaw contact", f"the gripper touched the {kind} after the jaws opened ({', '.join(sorted({h[1] for h in after}))}, "
                                     f"{len(after)} ticks, peak {row['jaw_contact']['peak_N']} N); {outcome}")
        elif not settled:
            fail(row, "not settled", f"drift {st['drift_mm']} mm / {st['drift_deg']} deg over the last {K.WINDOW_TICKS} ticks; {outcome}")
        elif not row["seated"]["seated"]:
            fail(row, "not seated", f"mouth {mouth}, " + (
                f"off its goal pose (centroid {ps_[1]} mm, axis {ps_[2]} deg; limits 2 mm / 5 deg)" if side_down else
                f"wires: bottom {sw.get('bottom_nearest')}, tine {sw.get('tine_nearest')}") + f"; {outcome}")

    def ring_blocker(oid):
        """--no-reach: the Kit-free goal table's F-out goal-pose row (as recorded) for this bowl: its blocker and why. D-ring-h100:
        the Kit-free release-height sweep's row (kitfree_v1/release_height/table.json) at the bowl's release height."""
        if oid in rel_h:
            h_mm = int(round(rel_h[oid] * 1e3))
            try:
                rows_h = json.loads(RELEASE_SWEEP_JSON.read_text())["rows"]
            except Exception:                          # noqa: BLE001
                return f"the Kit-free release-height sweep ({RELEASE_SWEEP_JSON.name} unreadable)"
            r_ = next((x for x in rows_h if x["bowl"] == oid and int(x["h_mm"]) == h_mm), None)
            if r_ is None:
                return f"the Kit-free release-height sweep (no row at {h_mm} mm)"
            b_ = r_.get("reported") or {}
            return (f"Kit-free (kitfree_v1/release_height, the F-out hold released {h_mm} mm above the goal, 12 spins, the 15 mm "
                    f"margin along the 50 mm above): {r_['n_pass']}/12 spins pass, IK at {r_['ik_spins']}/12; reported spin "
                    f"{b_.get('spin_deg')} ({b_.get('branch')}): {b_.get('why')}, min clearance {b_.get('clear_mm')} mm "
                    f"({b_.get('pair')})")
        try:
            fg = json.loads(args.foot_goal.read_text())
        except Exception:                              # noqa: BLE001
            return "the Kit-free goal check (foot_goal.json unreadable)"
        r_ = next((x for x in fg["rows"] if x["bowl"] == oid and x["hold"] == "F-out" and x["target"] == "goal pose"
                   and x.get("variant", "as recorded") == "as recorded"), None)
        if r_ is None:
            return "the Kit-free goal check (no F-out goal-pose row)"
        b_ = r_["best"]
        return (f"Kit-free (kitfree_v1/foot_goal, the F-out hold at the goal, 12 spins): {r_['spins_pass']}/12 spins pass, reach "
                f"{r_['spins_reach']}/12; best spin {b_.get('spin_deg')}: {b_.get('why')} (min clearance {b_.get('min_clear_mm')} mm, "
                f"{b_.get('blocker')})")

    # ---------------------------------------------------------------- plans/2026-11-04-grasp-place-s0.md (--attach none)
    def restore_state():
        """--state: the scene the previous dish left: the racks at its joints, the placed dishes teleported to their recorded
        (settled) poses while the guard is off, then at rest; the failed dishes stay parked (goal_instance parks every dish)."""
        if args.state is None:
            return {"file": None, "placed": {}, "failed": []}
        st = json.loads(args.state.read_text())
        harness.armed = False
        js = backend.joints()
        move = {j: float(v) for j, v in (st.get("rack_joints") or {}).items() if abs(js[j] - float(v)) > .005}
        if move:
            backend.ramp(move, "state_racks")
        for n, p_ in st["placed"].items():
            backend.set_rigid_pose(backend.objects[n], p_)
        rig.step(SPAWN_SETTLE_TICKS)
        rest = rest_wait(list(st["placed"]))
        now = backend.poses()
        drift = {n: round(float(np.linalg.norm(np.subtract(now[n]["position_m"], p_["position_m"]))) * 1e3, 2)
                 for n, p_ in st["placed"].items()}
        return {"file": str(args.state.relative_to(ROOT)) if args.state.is_relative_to(ROOT) else str(args.state),
                "placed": dict(st["placed"]), "failed": list(st.get("failed") or []), "rack_joints_moved": move, "rest": rest,
                "drift_mm": drift}

    def pick_reset(i, oid, row):
        """--attach none: the dish spawned alone at the counter spot in its pick orientation (the placed dishes where the state
        left them, the racks in this dish's state), the arm at the gate's home, the episode's pick (the kind's grasp, up to
        PICK_TRIES candidates, the 0.15 m lift, the in-hand settle, the drift hold gate); the hold M = the in-hand pose after
        the hold window; then the kind's Kit-free plan from the lift pose with M (D-ring: ring_plan; I4: the goal turned so the
        pinch is the rim point highest, seat_plan; the cups' drop search runs in drop()); the carry watch starts."""
        kind = kinds[oid]
        tlog.set(move=i + 1, dish=oid, attempt=0, phase="trial reset")
        harness.armed = False
        harness.clear_carried()
        detach()
        arm.held_clear_m = None                        # the pick's own checks hold no dish; rule 1 starts after the pick
        rig.gripper(False)
        rig.teleport_arm(ns.q_home)
        post_reset()
        rig.step(5)                                    # the PD targets follow the teleport before a rack ramp
        set_racks(rack_of(oid))
        spot = (presentation_pose(ns.pres)[0] if ns.pres and kind == "plate" else
                counter_pose(spot_xy, mouth_down=kind == "bowl" or side_down, kind=kind))
        backend.set_rigid_pose(backend.objects[oid], spot)
        rig.step(K.RESET_TICKS)
        rests = {"spawn": rest_wait([oid] + list(landed))}
        sync_world()
        harness.pad_max_interval = {"pad_L": 0., "pad_R": 0.}
        harness.arm_guard()
        traj.update(on=True, rows=[], oid=oid, others=list(landed))
        traj_row(backend.poses())
        patt = {}
        edge = args.grasp_family in ("edge", "plate-wall")   # the presented plate's pinch keeps the full clearance (no waiver)
        from dishsim_frigidaire.robot.flags import grip_drive
        gd_ = grip_drive(flags)                        # plans/2026-11-04-fifteen-of-fifteen.md: the hold's drive (flags R2 per
        try:                                           # family, spawned by robot_cfg), asserted on finger_joint before the pick
            gd_["finger_joint"] = rig.set_grip_drive(flags.gripper_stiffness, flags.gripper_damping)
        except Exception as e_:                        # noqa: BLE001  the spawn's drive stands; the failure is recorded
            gd_["finger_joint"] = {"error": f"{type(e_).__name__}: {e_}"}
        gd_.update(override=report.get("grip_drive", {}).get("override"),
                   rule="finger_joint spawned with the hold's drive (flags R2 per grasp family, --grip-drive on top); before "
                        "the pick its PhysX stiffness and damping read back and written only where they differ (Rig."
                        "set_grip_drive); the effort limit is the spawn's")
        row["grip_drive"] = gd_
        res, why = ns.pick(oid, patt, tries=PICK_TRIES, waive=kind == "plate" and not edge)
        holds_ = patt.get("holds") or []
        pk = {"held": res is not None, "why": why, "holds_tried": len(holds_), "hold_gate": holds_[-1] if holds_ else None,
              "holds": holds_, "spot": spot, "waive_dish_clearance": kind == "plate" and not edge,
              "presentation": None if not (ns.pres and kind == "plate") else
              {k_: ns.pres.get(k_) for k_ in ("name", "yaw_k", "lean_deg", "backstop", "edge", "facing_deg", "wall")},
              "grasp": None if res is None else {k_: v_ for k_, v_ in res.items() if k_ != "M"},
              "rule": f"the episode's pick(): the kind's grasp candidates at the spot facing the robot first, up to {PICK_TRIES} "
                      f"(one re-grasp), each the transit, the approach line, the close with the kind's jaw window, the 0.15 m "
                      f"lift, the in-hand settle and the drift hold gate ({flags.hold_window_s:.0f} s, "
                      f"{flags.hold_drift_max_m * 1e3:.0f} mm); " + ("the presented plate's wall-normal pinch (the presentation's "
                      "spin and rolls) at full clearance" if args.grasp_family == "plate-wall" else "the leaning plate's top-down "
                      "edge pinch (both rolls) at full clearance" if edge else "the plate with the dish clearance waived as its "
                      "gate ran")}
        row["pick"] = pk
        q_ = rig.q()
        rec = {"attach": "none (the pads alone)", "spawn": spot, "rest_waits": rests, "q_start": np.round(q_, 4).tolist(),
               "q_start_deg": [round(float(v), 1) for v in np.degrees(q_)]}
        if res is None:
            return rec
        attach(oid, T_now(oid))                        # the hold record only (no joint, no pin)
        M = att["M"]
        T_t = rig.tcp()
        pk["M"] = np.round(M, 6).tolist()
        pk["dish_from_tcp_mm"] = round(float(np.linalg.norm(M[:3, 3])) * 1e3, 1)
        row["in_hand"] = {"source": "measured after the pick's hold window (no assumed hold)", "T_tcp": T_t.tolist(),
                          "M": pk["M"], "pivot_deg": (pk["hold_gate"] or {}).get("settle_rotation_deg")}
        carry_state.clear()
        arm.held_clear_m = CARRY_CLEAR_M               # rule 1 (plans/2026-11-04-bowl-carry-clearance.md, amended): the held
        rec["carry_clear_mm"] = CARRY_CLEAR_M * 1e3    # dish 5 mm from the scene on every planned carry sample and at the start
        if ring or side_down:                          # D-side-down: the D-ring plan with the cup's hold
            rp_ = ring_plan(oid, M, q_)
            carry_state.update(rp_, z_grasp=float(T_t[2, 3]) - RING_LIFT_M)
            rec["ring_plan"] = {k: rp_.get(k) for k in ("counts", "first_fail", "seconds", "spins", "retreat_rejects")}
            if rp_.get("drop"):                        # INSERT_DROP_RULE: the guided release's search (only when it ran)
                rec["ring_plan"]["drop"] = rp_["drop"]
            rec["ring_plan"].update(chosen=None if not rp_.get("ranked") else ring_rec(rp_["ranked"][0]),
                                    ranked=[ring_rec(c) for c in (rp_.get("ranked") or [])[:DROP_TRIES]])
        elif seat:
            spin_, spin_src = pinch_spin(oid, (pk.get("grasp") or {}).get("roll_deg"))   # the load row's spin, else the gate's
                                                       # for the held roll (plate_04: toward the front)
            goal[oid] = sampler_bowl_tcp(goal[oid], M, spin_, np) @ M   # the top edge: the pinch at the rim point highest (+ spin)
            rec["pinch_spin_deg"] = spin_
            rec["pinch_spin_source"] = spin_src
            sp_ = seat_plan(oid, M, [q_])
            carry_state.update(sp_)
            ch_ = sp_.get("chosen")
            rec["seat_plan"] = {"seat_mm": (sp_.get("sk") or {}).get("seat_mm"), "seat_s": sp_.get("seat_s"),
                                "plan_s": sp_.get("plan_s"), "lines": sp_.get("lines"), "choices": sp_.get("choices"),
                                "narrow_retry": sp_.get("narrow_retry"), "goal_spun": pose(goal[oid]),
                                "seat_at_goal": sp_.get("seat_at_goal"), "perch": sp_.get("perch"),   # D3
                                "drop": sp_.get("drop"), "plate_drop": bool(getattr(args, "plate_drop", False)),   # D2
                                "chosen": None if ch_ is None else {"leg1": ch_["leg1"], "descent_start_mm": round(ch_["h_s"] * 1e3),
                                                                    "move_deg": round(float(np.degrees(ch_["move"])), 1),
                                                                    "release": ch_.get("release")}}
        carry_watch.clear()
        carry_watch.update(on=True, oid=oid, M=np.asarray(M, dtype=float), phase="carry", max_mm={}, arrived=False)
        return rec

    def sample_summary(oid, row):
        """The plan's columns of one dish: picked (held after the lift), carried (at the descent start, cups the drop pose,
        within CARRY_DROP_M of the hold), seated (the kind's test and settled), at goal (frigidaire_bench.AT_GOAL), cause."""
        kind = kinds[oid]
        pk = row.get("pick") or {}
        dropped = row.get("failed_at") == "dropped in carry"
        picked = bool(pk.get("held"))
        carried = picked and bool(carry_watch.get("arrived")) and not dropped
        sd = row.get("seated")
        seated = bool(sd.get("seated") if isinstance(sd, dict) else sd) and bool(row.get("settled"))
        legs_ = carry_watch.get("legs") or {}
        carry_legs = {lg: {"max_drift_mm": v["max_mm"], "contacts": sorted(v["contacts"]), "ticks": v["ticks"],
                           **{k_: v.get(k_) for k_ in ("max_tilt_deg", "max_inhand_rot_deg", "last_tilt_deg", "last_inhand_rot_deg")}}
                      for lg, v in legs_.items()}
        clean = picked and bool(legs_) and all(not v["contacts"] and v["max_mm"] < 1. for v in legs_.values())
        now = backend.poses()
        fp = row.get("final_pose") or now[oid]
        rk = meta[oid]["rack"]
        T_gr = T_of(now[rk]["position_m"], now[rk]["quaternion_xyzw"]) @ inv(F_goal[rk]) @ ns.goal_world[oid]
        dist = B.goal_distance(ns.centroids[kind], T_of(fp["position_m"], fp["quaternion_xyzw"]), T_gr)
        at_g = bool(carried and B.within_goal(kind, dist))
        return {"picked": picked, "carried": carried, "seated": seated, "at_goal": at_g,
                "goal_distance": {"lateral_mm": round(dist[0] * 1e3, 1), "dz_mm": round(dist[1] * 1e3, 1),
                                  "tilt_deg": round(dist[2], 1), "tolerance": list(B.AT_GOAL[kind])},
                "cause": None if row.get("failed_at") is None else f"{row['failed_at']}: {row.get('why')}",
                "pick": {k_: pk.get(k_) for k_ in ("held", "why", "holds_tried", "dish_from_tcp_mm")},
                "carry_drift_mm": dict(carry_watch.get("max_mm") or {}), "carry_limit_mm": CARRY_DROP_M * 1e3,
                "carry_legs": carry_legs, "carry_clean": bool(clean),
                "rule": "picked = held after the pick's lift and hold gate; carried = picked and at the descent start (cups: the "
                        "drop pose) with the dish within the limit of the pick's hold all the way; seated = the kind's seated "
                        "test (bowls the goal's wires and mouth down, plates the goal's wires, cups the goal pose within 2 mm / "
                        "5 deg) and settled; at goal = carried and within frigidaire_bench.AT_GOAL of the goal pose"}

    def write_state(i, oid, row, prev):
        """The scene this dish leaves for the next one: the placed dishes (this one only when its row passed) as they lie, at
        rest, and the rack joints; a failed dish is parked off the scene (the guard off) and listed as failed."""
        ok = row.get("failed_at") is None
        harness.armed = False
        if not ok:
            detach()
            harness.clear_carried()
            backend.set_rigid_pose(backend.objects[oid], park_pose(i))
            rig.gripper(False)
        placed = list(prev["placed"]) + ([oid] if ok else [])
        rest = rest_wait(placed)
        now = backend.poses()
        js = backend.joints()
        st = {"load": L, "after": oid, "order_index": i, "ok": ok, "placed": {n: now[n] for n in placed},
              "failed": list(prev["failed"]) + ([] if ok else [oid]),
              "rack_joints": {ns.RACK_JOINT[r]: float(js[ns.RACK_JOINT[r]]) for r in ("LowerRack", "UpperRack")},
              "rack_frames": {r: now[r] for r in ("LowerRack", "UpperRack")}, "rest": rest, "state_in": prev.get("file"),
              "rule": "placed = every dish whose row passed (seated and settled), as the robot left it; failed dishes are "
                      "parked off the scene; the next dish's process restores this scene (--state)"}
        path = out / "state" / f"after_{i:02d}_{oid}.json"
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(st, indent=1) + "\n")
        return path

    def basket_gate(oid):
        """plans/2026-11-04-fifteen-of-fifteen.md (basket out): the basket state this dish's Kit-free rows were computed with vs this
        run's: the gate row (gate.json beside the load: the row's "basket", else the file's top-level "basket"; neither: in, as
        every gate before the basket decision ran) and the load row (its "basket", else the load's top-level; neither: in)."""
        run_ = bool(getattr(flags, "basket", True))
        try:
            gj_ = json.loads((Path(args.place_goal).parent / "gate.json").read_text())
        except (OSError, ValueError, TypeError):
            gj_ = None
        g_, g_src = basket_flag((((gj_ or {}).get("dishes") or {}).get(oid) or {}).get("basket")), "the gate row's basket"
        if g_ is None and gj_ is not None:
            g_, g_src = basket_flag(gj_.get("basket")), "gate.json's top-level basket"
        if g_ is None:
            g_, g_src = (None, "no gate.json beside the load") if gj_ is None else (
                True, "gate.json without a basket field: in (every gate before the basket decision ran with it)")
        l_, l_src = basket_flag(meta[oid].get("basket")), "the load row's basket"
        if l_ is None:
            l_, l_src = basket_flag(load.get("basket")), "the load's top-level basket"
        if l_ is None:
            l_, l_src = True, "the load without a basket field: in"
        mis_ = [w_ for w_, b_ in (("gate", g_), ("load", l_)) if b_ is not None and b_ != run_]
        return {"run": run_, "gate": g_, "gate_source": g_src, "load": l_, "load_source": l_src, "mismatch": bool(mis_),
                "mismatch_with": mis_, "rule": "mismatch = the gate row or the load row computed with the other basket state than "
                                                "this run's (their predictions, holds and start choices assume that scene)"}

    def sample_dish(i, oid):
        """One dish of a samples_v1 load (--attach none): the state restored, the pick, the kind's move with the pads alone,
        the row (dishes.jsonl, with the summary columns) and the next state."""
        t0, s0, n_viol = time.monotonic(), backend.tick_index, len(harness.violations)
        nr_ = getattr(ns, "narrow", None)              # the episode's plan_to narrow retries (a fake Kit's ns has none)
        n_nr = len(nr_["log"]) if nr_ else 0
        arm.held_clear_m = None                        # rule 1 is set after the pick (pick_reset)
        prev = restore_state()
        landed[:] = list(prev["placed"])
        row = {"load": L, "dish": oid, "kind": kinds[oid], "grasp": meta[oid]["grasp"], "move": meta[oid].get("move"),
               "order_index": i, "variant": label, "attached": False, "attach_mode": "none (the pads alone)",
               "counter_front_y": args.counter_front_y, "steps": {}, "failed_at": None, "why": None, "error": None,
               "disturbed": [], "clip": None, "rack": meta[oid]["rack"], "slot": meta[oid]["slot"], "mount": report["mount"],
               "mount_file": str(flags.mount), "shortcut_s": args.shortcut_s, "held_margin_mm": round(arm.held_margin * 1e3, 1),
               "arm_margin_mm": round(arm.margin * 1e3, 1), "legacy_tolerances": bool(args.legacy_tolerances),
               "state_in": {k: prev.get(k) for k in ("file", "failed", "rest", "drift_mm", "rack_joints_moved")} |
                           {"placed": sorted(prev["placed"])}, **spot_rec,
               "transit_rrt_s": args.transit_rrt_s, "narrow_retry_s": narrow_budget(args)}
        if ring_side:                                  # D-ring-side's start order and who set it (plans/2026-11-04-fifteen-of-fifteen.md)
            row["ring_side_order"] = {"order": side_order, "source": side_order_src}
        row["move_run"] = {"move": label, "source": getattr(args, "move_source", None)}   # I4-wall20 with release.mode drop: -drop
        row["basket_gate"] = basket_gate(oid)          # the gate's / the load's basket state vs this run's
        if row["basket_gate"]["mismatch"]:
            print(f"[WARN] {oid}: basket {'in' if row['basket_gate']['run'] else 'out'} in this run, the "
                  f"{' and '.join(row['basket_gate']['mismatch_with'])} computed with it "
                  f"{'in' if not row['basket_gate']['run'] else 'out'} ({row['basket_gate']['gate_source']}; "
                  f"{row['basket_gate']['load_source']})", flush=True)
        row["reset"] = pick_reset(i, oid, row)
        if row["pick"]["held"]:
            try:
                (ring_drop if ring or side_down else drop if variant == "D" else insert_seat)(i, oid, row, landed)
            except CarryDropped as e_:                 # the arm stops where it is; the dish falls where it falls
                post["carry"] = False
                vel_ = robot.data.default_joint_vel.torch.clone()
                vel_[0, rig.arm] = 0.
                robot.actuators.target_command.set_velocity_index(value=vel_)
                rig.set_arm(rig.q())
                harness.clear_carried()
                rest_ = rest_wait([oid])
                row.update(final_pose=pose(T_now(oid)), dropped=carry_watch.get("dropped"), rest_after_drop=rest_,
                           disturbed=off_goal([n for n in landed if n != oid]))
                fail(row, "dropped in carry", str(e_))
        else:
            fail(row, "pick", row["pick"]["why"])
        row["carry_watch"] = {k: carry_watch.get(k) for k in ("max_mm", "at_release_mm", "arrived", "dropped", "legs")}
        row["carry_watch"].update(limit_mm=CARRY_DROP_M * 1e3, rule="the dish origin's distance from TCP @ M (the pick's hold) "
                                  "every tick; over the limit in the carry (before the descent; cups before the release) = "
                                  "dropped in carry; the descent's own drift recorded under 'descent'; legs = per carry leg "
                                  "(transit_to's joint plans, I4's leg 1 and tine-axis line) the largest drift and the held "
                                  "dish's PhysX contacts with anything but the hand")
        row["hold_source"] = hold_source(oid)        # plans/2026-11-04-fifteen-of-fifteen.md: the descent check's hand
        if "side_below_top_m" in row["hold_source"]:   # round 3: the cup's hold height and its gate run, in the row
            row.update(side_below_top_m=row["hold_source"]["side_below_top_m"], hold_gate_run=row["hold_source"]["hold_gate_run"])
        row["narrow_retries"] = list(nr_["log"][n_nr:]) if nr_ else []   # plan_to's own narrow retries (the pick's transit, ...)
        row["posture"] = posture()
        row["traj"] = save_traj(f"{oid}_{label}")
        row["invariants"] = sorted({v["name"] for v in harness.violations[n_viol:]})
        row["summary"] = sample_summary(oid, row)
        row["state_out"] = str(write_state(i, oid, row, prev).resolve().relative_to(ROOT))   # --place-goal may be relative
        row["record"] = str((args.out_dir / f"{ns.iid}.json").relative_to(ROOT))
        row["sim_s"] = round((backend.tick_index - s0) * backend.dt, 2)
        row["wall_s"] = round(time.monotonic() - t0, 1)
        rows.append(row)
        with open(rows_path, "a") as f:
            f.write(json.dumps(row, default=lambda o: o.tolist() if hasattr(o, "tolist") else str(o)) + "\n")
        sm = row["summary"]
        print(f"[INFO] {L} {label} {oid} ({i + 1}/{len(names)}): {'PLACED' if row['failed_at'] is None else 'fail ' + str(row['failed_at'])} "
              f"picked {sm['picked']} carried {sm['carried']} seated {sm['seated']} at goal {sm['at_goal']} "
              f"clean {sm['carry_clean']} drift {sm['carry_drift_mm']} goal {sm['goal_distance']} stop "
              f"{ {k: (row.get('stop') or {}).get(k) for k in ('kind', 'force_N', 'ticks', 'descent_mm')} } "
              f"{row['wall_s']} s  why {str(row['why'])[:300]}", flush=True)

    # ---------------------------------------------------------------- the dishes
    rows_path = out / "dishes.jsonl"
    variant = args.variant or ("R-ideal" if args.regrasp_ideal else "R" if args.regrasp else "W")
    side_base = variant == "D" and args.hold == "side-base"
    side_mouth = variant == "D" and args.hold == "side-mouth"          # plans/2026-11-04-grasp-place-s0.md (cups, option B)
    flip = variant == "D" and args.hold == "rim-flip"                 # plans/2026-10-27-bowls-plates-down.md (bowls)
    lip = variant == "D" and args.hold == "rim-lip"                   # plans/2026-10-27-bowls-lip-plates-fix.md (bowls)
    carry = lip and args.carry == "nearest"                           # plans/2026-10-27-carry-rules.md (bowls)
    perch = lip and args.carry == "overlean"                          # plans/2026-10-28-perch-release.md (bowls)
    seat_bowl = lip and args.carry == "seat"                          # plans/2026-10-29-bowl-last.md (bowls, rows D-seat)
    dropish = variant in ("D", "I")                                   # the earlier dishes stay where they landed
    seat = variant == "I" and args.insert == "seat"                   # plans/2026-10-28-plate-seat.md (plates, rows I4)
    sampler = args.insert == "sampler"                                # plans/2026-10-29-sampler-release.md (I5, D-sampler)
    samp = sampler_rows(L) if sampler else {}
    roll = perch and args.tilt_table is not None                      # plans/2026-10-29-perch-roll.md (D-roll)
    ring = variant == "D" and args.hold == "foot-out"                 # plans/2026-10-29-ring-place.md (bowls, rows D-ring)
    side_down = variant == "D" and args.hold == "side-down"           # plans/2026-11-04-cups-mouth-down.md (cups, D-ring path)
    ring_side = ring and args.grasp_family == "foot-side"             # plans/2026-11-04-ring-side-hold.md (bowls, D-side-down's path)
    def ring_side_order_of(oid):
        """D-ring-side's descent-start order for this dish and its source (plans/2026-11-04-fifteen-of-fifteen.md): an explicit
        --ring-side-order, else the load row's ring_side_order (the access order's, bowl_04 and bowl_01: straight), else pitch."""
        if getattr(args, "ring_side_order", None):
            return str(args.ring_side_order), "--ring-side-order"
        v_ = (meta.get(oid) or {}).get("ring_side_order")
        if v_:
            if v_ not in ("pitch", "straight"):
                raise RuntimeError(f"{oid}: the load row's ring_side_order {v_!r} is not pitch or straight")
            return str(v_), "the load row's ring_side_order"
        return "pitch", "the default (no --ring-side-order, no ring_side_order in the load row)"
    side_order, side_order_src = "pitch", None        # D-ring-side's start order (try b of bowl_04: straight): set per dish below
    no_reach = {x for x in args.no_reach.split(",") if x} if ring else set()
    extra = dict(x.split(":") for x in args.extra_row.split(",") if x) if ring else {}

    def extra_try(i, oid, mg, landed):
        """The flagged arm-margin try at the dish's own turn, right after its no-reach row, the scene as it is then
        (plans/2026-10-29-ring-place.md; the coordinator, 2026-10-29): the racks as normal (the upper rack pushed in) unless
        Kit-free no spin passes there but some pass with both racks out (then both out, flagged); no spin in either: no reach
        with both blockers. Whatever it does physically is recorded; then the dish is teleported to its refreshed goal so the
        dishes after it see the planned scene."""
        t0, s0, n_viol = time.monotonic(), backend.tick_index, len(harness.violations)
        arm.margin = float(mg)
        fh = json.loads(args.foot_hold.read_text())
        M = np.asarray(fh["inhand"]["M_tcp_to_bowl"], dtype=float)
        row = {"load": L, "dish": oid, "kind": kinds[oid], "variant": label, "extra": True,
               "grasp": "foot ring, the pads on the ring's outer face (the F-out gate's median hold), the bowl mouth down",
               "flag": f"extra try: the arm margin lowered to {float(mg) * 1e3:.0f} mm (default {C.MARGIN_M * 1e3:.0f}; its "
                       f"Kit-free clearance at the goal 10.2 mm), at its own turn right after its no-reach row, the scene as "
                       f"it is then",
               "order_index": i, "attached": True, "counter_front_y": args.counter_front_y, "steps": {}, "failed_at": None,
               "why": None, "error": None, "disturbed": [], "clip": None, "in_hand": {"source": f"{args.foot_hold}",
                                                                                     "trial": fh["inhand"]["trial"]},
               "rack": meta[oid]["rack"], "slot": meta[oid]["slot"], "mount": report["mount"], "mount_file": str(flags.mount),
               "shortcut_s": args.shortcut_s, "held_margin_mm": round(arm.held_margin * 1e3, 1),
               "arm_margin_mm": round(arm.margin * 1e3, 1), **spot_rec}
        if oid in landed:
            landed.remove(oid)
        row["reset"] = reset(i, oid, M, keep=list(landed))
        plans = {"upper rack pushed in": {k: carry_state.get(k) for k in ("counts", "first_fail")}}
        row["racks"] = "upper rack pushed in"
        if not carry_state.get("ranked"):
            rack_override[oid] = None
            row["reset_upper_in"] = row["reset"]
            row["reset"] = reset(i, oid, M, keep=list(landed))
            plans["both racks out"] = {k: carry_state.get(k) for k in ("counts", "first_fail")}
            row["racks"] = ("both racks out (flagged: no spin passes Kit-free with the upper rack pushed in)"
                            if carry_state.get("ranked") else "no spin passes in either rack state")
        row["ring_plans"] = plans
        traj.update(on=True, rows=[], oid=oid, others=list(landed))
        traj_row(backend.poses())
        ring_drop(i, oid, row, landed)
        rack_override.pop(oid, None)
        if row["failed_at"] == "no reach" and not (carry_state.get("ranked") or []):
            row["why"] = ("no passing spin with the upper rack pushed in (" + json.dumps(plans["upper rack pushed in"]["counts"])
                          + f"; {plans['upper rack pushed in']['first_fail']})"
                          + (" nor with both racks out (" + json.dumps(plans["both racks out"]["counts"])
                             + f"; {plans['both racks out']['first_fail']})" if "both racks out" in plans else "")
                          + "; placed at its goal pose (teleported, at rest)")
        row["attach_mode"] = row["reset"]["attach"]
        row["posture"] = posture()
        row["traj"] = save_traj(f"{L}_{oid}_{label}_margin{int(round(float(mg) * 1e3))}")
        row["invariants"] = sorted({v["name"] for v in harness.violations[n_viol:]})
        arm.margin = C.MARGIN_M
        if not row.get("placed_at_goal"):              # the physical outcome is recorded above; the dish to its goal
            detach()
            harness.clear_carried()
            rig.teleport_arm(ns.q_home)
            rig.step(5)
            T_g_, rest_ = place_at_goal(oid)
            row["after"] = {"teleported_to_goal": True, "pose": pose(T_g_), "rest": rest_,
                            "rule": "after the try the dish is teleported to its refreshed goal (at rest) so the dishes after "
                                    "it see the planned scene"}
        if oid not in landed:
            landed.append(oid)
        row["sim_s"] = round((backend.tick_index - s0) * backend.dt, 2)
        row["wall_s"] = round(time.monotonic() - t0, 1)
        rows.append(row)
        with open(rows_path, "a") as f:
            f.write(json.dumps(row, default=lambda o: o.tolist() if hasattr(o, "tolist") else str(o)) + "\n")
        print(f"[INFO] {L} {label} {oid} EXTRA (arm margin {float(mg) * 1e3:.0f} mm, {row['racks']}): "
              f"{'PLACED' if row['failed_at'] is None else 'fail ' + row['failed_at']} stop "
              f"{ {k: (row.get('stop') or {}).get(k) for k in ('kind', 'force_N', 'pairs', 'ticks', 'height_mm')} } check "
              f"{row.get('stop_check')} seated {(row.get('seated') or {}).get('seated')} mouth {row.get('mouth')} at goal "
              f"{row.get('at_goal')} jaw {(row.get('jaw_contact') or {}).get('after_open')} {row['wall_s']} s  why "
              f"{str(row['why'])[:300]}", flush=True)
    tt, tt_rows = tilt_table(args.tilt_table) if roll else ({}, [])
    jaw_info = {}
    label = args.row_variant or (f"D-ring-h{args.release_height_mm.split(',')[0].strip()}" if ring and rel_h else "D-ring" if ring
                                 else "D-roll" if roll else "I5" if sampler and variant == "I" else "D-sampler" if sampler
                                 else "D-side" if side_base else "D-side-mouth" if side_mouth else "D-side-down" if side_down
                                 else "D-flip" if flip else "D-perch" if perch else "D-seat" if seat_bowl
                                 else "D-lip2" if carry else "D-lip" if lip else "I4" if seat else variant)
    side = json.loads(args.side_candidates.read_text())["cups"] if variant == "S" else {}
    todo = list(names)
    if args.regrasp or args.regrasp_ideal:
        w = {}
        for line_ in rows_path.read_text().splitlines():
            r_ = json.loads(line_) if line_.strip() else None
            if r_ and r_["variant"] == "W":
                w[r_["dish"]] = r_
        todo = [n for n in names if w.get(n, {}).get("failed_at") in ("S1", "S2")]
    if args.dishes:
        todo = [n for n in todo if n in set(args.dishes.split(","))]
    rows, landed = [], []
    for oid in todo:
        i = names.index(oid)
        side_order, side_order_src = ring_side_order_of(oid)   # the closures (ring_plan, ring_drop) read it at call time
        if att_mode["none"]:                       # plans/2026-11-04-grasp-place-s0.md: a samples_v1 load, the pads alone
            sample_dish(i, oid)
            continue
        t0, s0, n_viol = time.monotonic(), backend.tick_index, len(harness.violations)
        T_goal, cand = None, side.get(oid)
        if variant == "S":                         # the best Kit-free candidate's measured hold and its turned goal
            best = cand["best"]
            M, src = (None, None) if best is None else (np.array(best["M"]), {k: best[k] for k in (
                "below_top_m", "yaw_k", "wrist_side", "gate_run", "trial", "pivot_deg", "goal_turn_deg")})
            T_goal = None if best is None else np.array(best["T_goal"])
        elif sampler:                              # the final-pose table's hold at the sampler's release pose
            fp = samp[oid]
            T_rel0 = T_of(fp["release_pose"]["position_m"], fp["release_pose"]["quaternion_xyzw"])
            hold_ = fp["hold"]
            if kinds[oid] == "plate":
                d_mm = int(hold_.split("d")[-1])
                T_tcp0 = sampler_mirror_tcp(T_rel0, d_mm, G, np, mirror=hold_.startswith("mirror"))
                hold_src = (f"the plate upright at the sampler's release pose, '{hold_}': the top-edge rim hold (plate_holds "
                            f"'mouth') backed off so the rim is {d_mm} mm into the pads"
                            + (", reflected through the rim wall's mid plane (the hand body on the base side)"
                               if hold_.startswith("mirror") else ""))
            else:
                M_meas, src_m = RC.measured_M(kinds[oid])
                spin_ = float(hold_.split()[-1])
                T_tcp0 = sampler_bowl_tcp(T_rel0, M_meas, spin_, np)
                hold_src = (f"the bowl gate's median rim pinch (trial {src_m['trial']}) turned about the bowl axis so the "
                            f"pinched rim point is the highest at the sampler's release pose (the packed pose + "
                            f"{fp['release_lift_mm'] - fp['lowest_clear_lift_mm']:.0f} mm), spin {spin_:+.0f} deg")
            M = inv(T_tcp0) @ T_rel0
            src = {"source": f"assumed (no {kinds[oid]} pick to this pose exists): {hold_src}; kitfree_v1/sampler_release/"
                             f"final_pose_table.json ({fp['verdict']}); zero pivot", "pivot_deg": 0., "T_tcp": T_tcp0.tolist(),
                   "hold": hold_, "side": "base" if hold_.startswith("mirror") else "mouth" if kinds[oid] == "plate" else None}
        elif variant == "I" and seat and args.plate_hold is not None:   # window 2: the hold from hold_base.json
            hb = json.loads(args.plate_hold.read_text())
            hj = hb["holds"][args.plate_hold_kind][oid][str(int(round(args.rim_depth_m * 1e3)))]
            M = T_of(hj["M_tcp_to_plate"]["position_m"], hj["M_tcp_to_plate"]["quaternion_xyzw"])
            T_tcp0 = goal[oid] @ inv(M)
            holds = {"base": (M, T_tcp0)}
            src = {"source": f"assumed (no plate pick exists): the base-face top-edge hold '{args.plate_hold_kind}' of "
                             f"{args.plate_hold.relative_to(ROOT) if args.plate_hold.is_absolute() else args.plate_hold} "
                             f"({hb['construction'].get(args.plate_hold_kind, '')[:300]}), rim depth "
                             f"{args.rim_depth_m * 1e3:.0f} mm; zero pivot", "pivot_deg": 0., "T_tcp": T_tcp0.tolist(),
                   "side": "base", "hold": args.plate_hold_kind, "rim_depth_mm": round(args.rim_depth_m * 1e3, 1)}
        elif variant == "I" and seat:              # I4: the base face only, the TCP backed off to the assumed rim depth
            M0, T0 = plate_holds(goal[oid], G, np)["base"]
            s_back = RIM_DEPTH_TODAY_M - args.rim_depth_m
            Tz = np.eye(4)
            Tz[2, 3] = s_back
            M, T_tcp0 = Tz @ M0, T0 @ inv(Tz)
            holds = {"base": (M, T_tcp0)}
            src = {"source": f"assumed (no plate pick exists): grasp.py's KIND_RIM plate rim pinch at the rim point highest at "
                             f"the goal orientation, the gripper body on the BASE face (wrist 3 turned 180 deg), the rim "
                             f"{args.rim_depth_m * 1e3:.0f} mm into the pads: the TCP {s_back * 1e3:.0f} mm further back along "
                             f"its approach than rounds 8-10 ({RIM_DEPTH_TODAY_M * 1e3:.0f} mm), the plate's pose in hand "
                             f"otherwise unchanged; zero pivot", "pivot_deg": 0., "T_tcp": T_tcp0.tolist(), "side": "base",
                   "rim_depth_mm": round(args.rim_depth_m * 1e3, 1), "tcp_back_mm": round(s_back * 1e3, 1)}
        elif variant == "I":                       # the assumed top-edge rim hold, the gripper on the mouth face first
            holds = plate_holds(goal[oid], G, np)
            M = holds["mouth"][0]
            src = {"source": "assumed (no plate pick exists): grasp.py's KIND_RIM plate rim pinch at the rim point highest at "
                             "the goal orientation (TCP 10 mm under the rim top, closing radially across the rim wall), "
                             "zero pivot; side mouth = the gripper body on the mouth face, base = wrist 3 turned 180 deg",
                   "pivot_deg": 0., "T_tcp": holds["mouth"][1].tolist(), "side": "mouth"}
        elif ring:                                 # plans/2026-10-29-ring-place.md: the F-out gate's measured hold
            fh = json.loads(args.foot_hold.read_text())
            M = np.asarray(fh["inhand"]["M_tcp_to_bowl"], dtype=float)
            src = {"source": f"the F-out gate's median-pivot hold ({args.foot_hold.parent.name} trial {fh['inhand']['trial']}, "
                             f"held {fh['held']}/{fh['trials']}): the bowl origin {fh['inhand']['origin_mm']} mm from the TCP, axis "
                             f"{fh['inhand']['axis_off_tcp_z_deg']} deg off", "trial": fh["inhand"]["trial"],
                   "pivot_deg": fh["inhand"]["pivot_deg"]}
        else:
            M, src = in_hand(kinds[oid])
        row = {"load": L, "dish": oid, "kind": kinds[oid], "grasp": "side pinch" if variant == "S" else
               "side pinch, wrist toward the base, pads 45 mm under the rim" if side_base else
               "assumed rim hold" if variant == "D" and kinds[oid] == "plate" else
               f"assumed rim hold at the plate's top edge, base face, rim {args.rim_depth_m * 1e3:.0f} mm into the pads"
               if seat else
               f"the final-pose table's hold at the sampler's release pose ({samp[oid]['hold']})" if sampler else
               "assumed rim hold at the plate's top edge" if variant == "I" else
               "foot ring, the pads on the ring's outer face (the F-out gate's median hold), the bowl mouth down" if ring else
               "rim pinch (the bowl gate's median hold), turned about the closing axis to fall mouth down" if flip else
               "rim pinch (the bowl gate's median hold) at the lip, the bowl turned steeper than its goal about the lip's "
               "horizontal rim tangent (over-lean)" if perch else
               "rim pinch (the bowl gate's median hold) at the lip, carried at the tilt (goal lean .. 90 deg) whose predicted "
               "stop is lowest above the seat" if seat_bowl else
               "rim pinch (the bowl gate's median hold) at the lip: the bowl at its goal axis, the pinched rim point highest"
               if lip else meta[oid]["grasp"],
               "order_index": i, "variant": label, "attached": True, "counter_front_y": args.counter_front_y,
               "steps": {} if dropish else {s: "skip" for s in STEPS}, "failed_at": None, "why": None, "error": None,
               "disturbed": [], "clip": None, "in_hand": src, "rack": meta[oid]["rack"], "slot": meta[oid]["slot"],
               "mount": report["mount"], "mount_file": str(flags.mount), "shortcut_s": args.shortcut_s,
               "held_margin_mm": round(arm.held_margin * 1e3, 1), "arm_margin_mm": round(arm.margin * 1e3, 1), **spot_rec}
        if cand is not None:
            row["candidates"] = cand["summary"]
        if variant == "I" and not sampler:
            row.update(side="base" if seat else "mouth", side_switches=0, _holds={k: v[0] for k, v in holds.items()})
        if seat:
            row.pop("_holds")
        if sampler:
            fp = samp[oid]
            row.update(side=src.get("side"), sampler_table={k: fp.get(k) for k in (
                "hold", "verdict", "feasible", "stop", "lowest_clear_lift_mm", "release_lift_mm", "release_above_goal_mm",
                "release_xy_off_goal_mm", "release_axis_vs_goal_deg", "release_lean_deg", "goal_lean_deg")})
        if roll and not (tt["bowls"].get(oid) or {}).get("gate"):   # not gated by the perch-roll curve: placed at its goal
            g_ = tt["bowls"].get(oid) or {}
            T_g_, rest_ = place_at_goal(oid)
            landed.append(oid)
            row.update(attached=False, attach_mode=None, traj=None, invariants=[], placed_at_goal=True, final_pose=pose(T_g_),
                       rest=rest_, gate=g_, sim_s=round((backend.tick_index - s0) * backend.dt, 2),
                       wall_s=round(time.monotonic() - t0, 1))
            fail(row, "not gated", (f"the perch-roll curve's best tilt {g_.get('best_tilt')} deg: {g_.get('seated_down')}/"
                                    f"{g_.get('drops')} seated mouth down (gate >= 4)" if g_ else "not in the perch-roll curve")
                 + "; placed at its goal pose (teleported, at rest) for the bowls after it")
            rows.append(row)
            with open(rows_path, "a") as f:
                f.write(json.dumps(row, default=lambda o: o.tolist() if hasattr(o, "tolist") else str(o)) + "\n")
            print(f"[INFO] {L} {label} {oid} ({i + 1}/{len(names)}): not gated ({row['why'][:160]})", flush=True)
            continue
        if ring and oid in no_reach:               # plans/2026-10-29-ring-place.md: the Kit-free blocked bowls, at their goals
            T_g_, rest_ = place_at_goal(oid)
            landed.append(oid)
            row.update(attached=False, attach_mode=None, traj=None, invariants=[], placed_at_goal=True, final_pose=pose(T_g_),
                       rest=rest_, sim_s=round((backend.tick_index - s0) * backend.dt, 2), wall_s=round(time.monotonic() - t0, 1))
            fail(row, "no reach", ring_blocker(oid) + "; placed at its goal pose (teleported, at rest) for the bowls after it")
            rows.append(row)
            with open(rows_path, "a") as f:
                f.write(json.dumps(row, default=lambda o: o.tolist() if hasattr(o, "tolist") else str(o)) + "\n")
            print(f"[INFO] {L} {label} {oid} ({i + 1}/{len(names)}): fail no reach ({row['why'][:200]})", flush=True)
            if oid in extra:
                extra_try(i, oid, extra[oid], landed)
            continue
        if sampler and not samp[oid]["feasible"]:  # infeasible at the sampler's release pose: placed at its goal, no Isaac arm
            T_g_, rest_ = place_at_goal(oid)
            landed.append(oid)
            row.update(attached=False, attach_mode=None, traj=None, invariants=[], placed_at_goal=True, final_pose=pose(T_g_),
                       rest=rest_, sim_s=round((backend.tick_index - s0) * backend.dt, 2), wall_s=round(time.monotonic() - t0, 1))
            fail(row, "no reach", f"sampler release infeasible: {samp[oid]['verdict']} (hold {samp[oid]['hold']}; Kit-free, "
                                  f"kitfree_v1/sampler_release/final_pose_table.json); placed at its goal pose (teleported, at "
                                  f"rest) for the dishes after it")
            rows.append(row)
            with open(rows_path, "a") as f:
                f.write(json.dumps(row, default=lambda o: o.tolist() if hasattr(o, "tolist") else str(o)) + "\n")
            print(f"[INFO] {L} {label} {oid} ({i + 1}/{len(names)}): fail no reach (sampler release infeasible: "
                  f"{samp[oid]['verdict']}); placed at its goal, rest {rest_}", flush=True)
            continue
        if variant == "S" and M is None:           # no candidate passes the counter pick and the goal: no Isaac
            row.update(attached=False, attach_mode=None, traj=None, invariants=[], sim_s=0., wall_s=0.)
            row["steps"].update({s_: "pass" for s_ in ("S1",) if cand["failed_at"] == "S2"})
            fail(row, cand["failed_at"], cand["why"])
            rows.append(row)
            with open(rows_path, "a") as f:
                f.write(json.dumps(row) + "\n")
            print(f"[INFO] {L} {variant} {oid} ({i + 1}/{len(names)}): fail {row['failed_at']} (Kit-free, no candidate)  "
                  f"why {str(row['why'])[:220]}", flush=True)
            continue
        row["reset"] = reset(i, oid, M, keep=list(landed) if dropish else None)
        traj.update(on=True, rows=[], oid=oid, others=list(landed) if dropish else names[:i])
        if dropish:
            traj_row(backend.poses())                  # the arm at home, dish in hand: every dish has a replay sample
            (insert_sampler if sampler else ring_drop if ring else perch_drop if perch else seat_drop if seat_bowl
             else drop if variant == "D" else
             insert_seat if seat else insert)(i, oid, row, landed)
        elif variant == "R":
            set_down(oid, row)
        elif variant == "R-ideal":
            ideal_set_down(oid, row)
        if variant not in ("W", "D", "I") and row["failed_at"] is None:
            repick(oid, row)
        if not dropish and row["failed_at"] is None:
            place_steps(i, oid, row, T_goal)
        row["attach_mode"] = row["reset"]["attach"]
        row["posture"] = posture()
        if carry or perch or seat_bowl or ring:        # measured: the largest single-joint travel over the carry (every try)
            row["carry_cost_deg"] = max(row["posture"]["carry_travel_deg"])
            row["carry_sum_deg"] = row["posture"]["carry_travel_sum_deg"]
        row["traj"] = save_traj(f"{L}_{oid}_{label}")
        row["invariants"] = sorted({v["name"] for v in harness.violations[n_viol:]})
        row["sim_s"] = round((backend.tick_index - s0) * backend.dt, 2)
        row["wall_s"] = round(time.monotonic() - t0, 1)
        rows.append(row)
        with open(rows_path, "a") as f:
            f.write(json.dumps(row, default=lambda o: o.tolist() if hasattr(o, "tolist") else str(o)) + "\n")
        extra = "" if not dropish else (
            (f"hold {(row.get('in_hand') or {}).get('hold')} release +{(row.get('sampler_release') or {}).get('above_goal_mm')} mm "
             f"stop pred {(row.get('stop_pred') or {}).get('above_goal_mm')} start +{row.get('descent_start_mm')} mm stop "
             f"{ {k: (row.get('stop') or {}).get(k) for k in ('kind', 'force_N', 'pairs', 'ticks', 'descent_mm', 'dish_h_mm', 'weld_drift_mm')} } "
             f"check {row.get('stop_check')} up {row.get('up_before_release')} retreat {row.get('retreat')} seated "
             f"{(row.get('seated') or {}).get('seated')} mouth {row.get('mouth')} in slot {row.get('in_slot')} "
             if sampler else
             f"depth {row.get('hold_depth_mm')} mm seat {row.get('seat_mm')} hand pred {(row.get('hand_stop_mm') or {}).get('pred')} "
             f"start +{row.get('descent_start_mm')} mm stop "
             f"{ {k: (row.get('stop') or {}).get(k) for k in ('kind', 'force_N', 'pairs', 'ticks', 'descent_mm', 'weld_drift_mm')} } "
             f"check {row.get('stop_check')} clear {row.get('hand_clear_at_seat_mm', {}).get('closed')} seated "
             f"{(row.get('seated') or {}).get('seated')} wires {(row.get('seated') or {}).get('wires_final')} in slot "
             f"{row.get('in_slot')} retreat back {(row.get('retreat') or {}).get('back_mm')} rise {(row.get('retreat') or {}).get('rise')} "
             if seat else
             f"side {row.get('side')} (switches {row.get('side_switches')}) from +{row.get('start_height_m')} m lowered "
             f"{row.get('lowered_mm')} mm stop {row.get('stop_reason')} {row.get('stop_partners')} in slot {row.get('in_slot')} "
             f"lean {row.get('lean_deg')} " if variant == "I" else
             f"{'flip tilt %s clear %s ' % (row.get('flip_tilt_deg'), row.get('flip_clear')) if flip else ''}"
             f"{'spin %s fall clearance %s ' % (row.get('spin_deg'), (row.get('fall_clearance') or {}).get('min_mm')) if lip else ''}"
             f"{'cost pred %s measured %s legs %s pred %s ' % (row.get('carry_cost_pred'), row.get('carry_cost_deg'), (row.get('legs') or {}).get('plans'), row.get('legs_pred')) if carry or perch else ''}"
             f"{'tilt %s spin %s perch %s stop %s ' % (row.get('tilt_deg'), row.get('spin_deg'), row.get('perch_mm'), {k: (row.get('stop') or {}).get(k) for k in ('kind', 'force_N', 'pairs', 'ticks', 'descent_mm', 'weld_drift_mm')}) if perch else ''}"
             f"{'seated %s at goal %s roll %s (agrees %s) jaw %s ' % ((row.get('seated') or {}).get('seated'), row.get('at_goal'), row.get('roll_measured'), row.get('roll_agrees'), row.get('jaw_contact')) if roll else ''}"
             f"{'spin %s stop pred %s stop %s check %s carry %s seated %s at goal %s jaw %s ' % (row.get('spin_deg'), row.get('stop_pred'), {k: (row.get('stop') or {}).get(k) for k in ('kind', 'force_N', 'pairs', 'ticks', 'height_mm', 'weld_drift_mm')}, row.get('stop_check'), row.get('carry'), (row.get('seated') or {}).get('seated'), row.get('at_goal'), (row.get('jaw_contact') or {}).get('after_open')) if ring else ''}"
             f"{'tilt %s spin %s stop pred %s stop %s check %s retreat %s seated %s ' % (row.get('tilt_deg'), row.get('spin_deg'), row.get('stop_pred'), {k: (row.get('stop') or {}).get(k) for k in ('kind', 'force_N', 'pairs', 'ticks', 'height_mm', 'weld_drift_mm')}, row.get('stop_check'), {k: (row.get('retreat') or {}).get(k) for k in ('outcome', 'direction', 'moved_mm', 'disturbed')}, (row.get('seated') or {}).get('seated')) if seat_bowl else ''}"
             f"drop {row.get('drop_height_m')} m mouth {row.get('mouth')} ")
            + f"landed on {row.get('landed_on')} moved {row.get('moved_earlier')} start {row['reset'].get('q_start_deg')} "
              f"max|q| {row['posture']['max_abs_deg']} up {row['posture']['elbow_up']} travel {row['posture']['carry_travel_deg']} ")
        print(f"[INFO] {L} {label} {oid} ({i + 1}/{len(names)}): {'PLACED' if row['failed_at'] is None else 'fail ' + row['failed_at']}"
              f" {row['steps']} error {row['error']} disturbed {row['disturbed']} attach {row['reset']['attach']} {extra}"
              f"{row['wall_s']} s  why {str(row['why'])[:220]}", flush=True)
    detach()
    report["rows"] = rows
    report["attach_mode"] = dict(att_mode)
    report["seconds_wall"] = round(time.monotonic() - t_run, 1)
    counts = {s: sum(r["failed_at"] == s for r in rows) for s in STEPS + ("no reach", "no perch", "no seat", "hand stuck",
                                                                         "not settled", "not seated", "mouth up", "not gated",
                                                                         "not releasable", "jaw contact", "pick",
                                                                         "dropped in carry", "no sensor stop", "no clear retreat",
                                                                         "perch predicted", "not tine-guided", "not at goal")
              if any(r["failed_at"] == s for r in rows)}
    report["summary"] = {"load": L, "variant": label, "dishes": len(rows), "placed": sum(r["failed_at"] is None for r in rows),
                         "settled_any": sum(bool(r.get("settled")) for r in rows), "failed_at": counts}
    report["result"] = "PASS"
    down = side_base or flip or lip
    if seat or seat_bowl or sampler or roll or ring or side_down:
        report["summary"].update(stop_kinds={k: sum((r.get("stop") or {}).get("kind") == k for r in rows)
                                             for k in sorted({(r.get("stop") or {}).get("kind") for r in rows if r.get("stop")})},
                                 stop_within_2mm=sum(bool((r.get("stop_check") or {}).get("within_2mm")) for r in rows))
    if roll or ring or side_down:
        report["summary"].update(seated_mouth_down=sum(bool((r.get("seated") or {}).get("seated")) for r in rows),
                                 jaw_contact=sum(r.get("failed_at") == "jaw contact" for r in rows),
                                 at_goal=sum(bool(r.get("at_goal")) for r in rows))
        if roll:
            report["summary"]["gated"] = list(tt.get("gated") or [])
    print(f"[RESULT] PASS place-goal {L} {label}: {('seated mouth down %d, ' % report['summary']['seated_mouth_down']) if roll or ring or side_down else ''}{'seated and settled' if seat or (sampler and variant == 'I') else 'seated mouth down' if sampler else 'seated mouth down, hand away' if seat_bowl else 'settled mouth down' if down else 'settled' if dropish else 'placed'} "
          f"{report['summary']['placed']}/{len(rows)}{', settled any %d' % report['summary']['settled_any'] if down or seat else ''}, "
          f"failed at {counts}, "
          + (f"stops {report['summary']['stop_kinds']}, within 2 mm {report['summary']['stop_within_2mm']}, " if seat or seat_bowl or sampler or roll or ring or side_down else "")
          + f"attach {att_mode}", flush=True)
    return finish()


if __name__ == "__main__":
    main()
