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
    parser.add_argument("--grasp-family", choices=("rim", "wall", "side"), default="rim",
                        help="grasp gate: wall = the cup wall pinch (robot/grasp.py wall_grasps, 2026-10-26) instead of the "
                             "mouth-up rim pinch; side = the horizontal cup side pinch (side_grasps, lifted straight up)")
    parser.add_argument("--side-below-top", type=float, default=.011,
                        help="--grasp-family side: the pad centres this far under the rim (m; one height per gate run)")
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
    parser.add_argument("--variant", choices=("S",), default=None,
                        help="--place-goal variant S (plans/2026-10-26-side-pinch.md): every cup held by the side pinch at the "
                             "measured in-hand pose of its best --side-candidates candidate, then S1-S6")
    parser.add_argument("--side-candidates", type=Path, default=ROOT / "data/results/robot/placing_v1/side/candidates.json",
                        help="--variant S: frigidaire_robot_reach.py --side-candidates' output")
    parser.add_argument("--attach", choices=("joint", "pin"), default="joint",
                        help="--place-goal: a PhysX fixed joint (default; falls back to pin when it does not hold) or pin "
                             "(the dish written to TCP @ M after every rig step)")
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
    from isaaclab.app import AppLauncher
    AppLauncher.add_app_launcher_args(parser)
    args = parser.parse_args()
    args.mode = ("grasp_gate" if args.grasp_gate else "place_slot" if args.place_slot else
                 "place_goal" if args.place_goal else None)
    if args.mode is None and args.instance is None:
        raise SystemExit("[RESULT] FAIL --instance is required without --grasp-gate / --place-slot")
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
    app = AppLauncher(args).app
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
    if args.mode:                                     # invariants recorded per trial, never abort the trial loop
        overrides.update(invariants_abort=False, counter_cap=False)
    if args.contact_lag is not None:
        overrides["place_down_contact_lag"] = float(args.contact_lag)
    if args.legacy_tolerances:
        overrides.update(tol_approach=.08, tol_joint_move_held=.08, tol_rrt=.08, tol_rise=.08, tol_lift=.08, tol_reaim=.08,
                         settled_lag_rad=.08)       # place()'s insertion line settles with the rig default
    if args.mode == "grasp_gate":                     # G3: hold 5 s (the 0.15 m lift: pick's lift_m)
        overrides.update(hold_window_s=5.)
    flags = F.resolve(args.profile, **overrides)
    if args.mode == "place_goal":
        inst = goal_instance(json.loads(args.place_goal.read_text()))
    elif args.mode:
        kind = args.grasp_gate or args.place_slot
        inst = limits_instance(kind)
    else:
        inst = json.loads(args.instance.read_text())
    if flags.test_case:
        inst = test_instance(inst, flags.test_case)
    iid = inst["instance_id"]
    report = {"instance": iid, "result": "FAIL", "started_utc": datetime.now(timezone.utc).isoformat(), "moves": [],
              "run_id": args.run_id, "trial_id": args.trial_id, "profile": flags.profile, "headline": F.is_headline(flags),
              "flags": F.to_dict(flags), "deviations_from_headline": [list(d) for d in F.deviations(flags)],
              "trial_dir": str(args.trial_dir.relative_to(ROOT))}
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
LIMITS_COUNTER_XY = (.20, -.16)       # = frigidaire_robot_reach.COUNTER_XY (the plate's edge stays 26 mm inside the slab)
PIVOT_DEG = 10.                       # a held dish rotated more than this in the hand after the lift counts as "pivot"
TRAJ_EVERY = 12                       # ticks between replay samples (10 fps of simulated time at 120 Hz)


def limits_instance(kind):
    """One mouth-up dish of ``kind`` on the counter, on the benchmark's extended baseline (no instance file needed)."""
    base = json.loads((ROOT / "data/results/benchmark/frigidaire_hotec/baseline/result.json").read_text())
    snap = base.get("snapshot", base)
    oid = f"{kind}_01"
    pose = {"position_m": [LIMITS_COUNTER_XY[0], LIMITS_COUNTER_XY[1], .914 + .002], "quaternion_xyzw": [0., 0., 0., 1.]}
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
        # the robot and its pedestal (a static column under the base)
        import isaaclab.sim as sim_utils
        ped = sim_utils.CylinderCfg(radius=.08, height=float(mount["base"][2]), collision_props=sim_utils.CollisionPropertiesCfg(),
                                    visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(.35, .36, .40)))
        ped.func("/World/Pedestal", ped, translation=(mount["base"][0], mount["base"][1], mount["base"][2] / 2))
        qw = (math.cos(yaw / 2), 0., 0., math.sin(yaw / 2))
        holder["robot"] = Articulation(ur5e.robot_cfg(prim_path="/World/Robot", pos=tuple(mount["base"]), rot_wxyz=qw,
                                                       gripper_effort=flags.gripper_effort, gripper_stiffness=flags.gripper_stiffness))
        holder["deinstanced"] = ur5e.deinstance_gripper(stage)
        if flags.deinstance_root != "/World/Robot/Gripper":                # D12: the whole robot
            holder["deinstanced_root"] = {flags.deinstance_root: HN.deinstance(stage, flags.deinstance_root)}
        holder["pads"] = ur5e.pad_material(stage)
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
                  pad_colliders=holder["pads"], authored=holder["authored"])
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
                                          "qd_max": round(float(np.abs(robot.data.joint_vel[0, rig.arm].cpu().numpy()).max()), 3)}

    rig = Rig(backend.sim, robot, T_wb, on_step=on_step, physics_step=backend.tick, lag_max=flags.lag_max_rad,
              settled_lag=flags.settled_lag_rad, update_robot=False,
              on_teleport=lambda what: hook["teleport"](what) if hook.get("teleport") else None)
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
    arm = C.ArmWorld(checker, P.slab_body(checker, B.COUNTER), C.link_hulls(omni.usd.get_context().get_stage(), robot=robot),
                     C.calibrate(robot, rig.q(), T_wb), T_wb, rig.T_w3_tcp)
    bx, by, bz = mount["base"]
    arm.add_static_box("Pedestal", (bx, by, bz / 2 - .01), (.18, .18, bz - .02))   # the column, 1 cm under the base
    arm.add_static_box("Floor", (0., 0., -.05), (6., 6., .1))
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
        """Collision-free IK solutions of a TCP pose, wrist-UP first (wrist_1 highest above the TCP: a flipped
        wrist hangs its housing at the dish's height, measured 2026-09-29), then nearest to the current joints."""
        qs = [q for q in rig.ik_all(T_tcp, rig.q()) if arm.free(q, **kw)]
        return sorted(qs, key=lambda q: (-round(float(arm.link_poses(q)["wrist_1_link"][2, 3]), 2),
                                         float(np.abs(q - rig.q()).max())))

    def plan_to(q_goal, **kw):
        """A checked straight joint move, else RRT-Connect; True when executed."""
        q0 = rig.q()
        slow = .004 if kw.get("held") else .008
        if arm.path_free(q0, q_goal, step=.02, **kw):
            return rig.move_joint(q_goal, max_step=slow, settle_tol=flags.tol_joint_move_held if kw.get("held") else None)
        path = C.rrt_connect(arm, q0, [q_goal], time_s=15., collide_kw=kw)
        return path is not None and rig.follow(path, steps_per=10 if kw.get("held") else 5, settle_tol=flags.tol_rrt)

    def transit_to(T_target_hover, q_hover=None, **kw):
        """Straight up to Z_SAFE, joint-plan to above the target (same IK branch as ``q_hover``), joint-plan down to
        ``q_hover`` (a single joint move dipped the carried dish into the counter; a long straight descent jumped
        IK branches, 2026-09-29)."""
        sync_world()
        dz = .002 if kw.get("held") else .004
        now = rig.tcp()
        if now[2, 3] < flags.z_safe_m - .01:
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

    lift_m = .15 if args.mode == "grasp_gate" else None      # G3 lift height (None: back to the hover)

    def pick(oid, att, tries=6, waive=False):
        sync_world()
        T_obj = T_now(oid)
        cands = (G.wall_grasps(T_obj, dish=kinds[oid]) if args.grasp_family == "wall" else
                 [g for w in G.WRIST_SIDES for g in G.side_grasps(T_obj, below_top=args.side_below_top, dish=kinds[oid],
                                                                   wrist_side=w)] if args.grasp_family == "side"
                 else G.tilted_grasps(T_obj, dish=kinds[oid]))
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
                allow_m = ALLOW_STRADDLE_M if g["kind"] in ("wall", "base") or args.grasp_family == "side" else ALLOW_CONTACT_M
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
        scored.sort(key=lambda s: s[0])
        why = None
        tried = []
        for _, g, q_h, qs_line in scored[:tries]:
            rig.gripper(False)
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
            tlog.set(phase="close")
            theta = rig.gripper(True)
            key("close")
            if "rig" in cams and len(tried) == 0:
                report.setdefault("grasp_stills", []).append(close_still(f"closed_{oid}"))
            tlog.write("note", what="closed", jaw_rad=round(theta, 4), at_close_partners=partners, grasp=g["kind"],
                       tilt_deg=g.get("tilt_deg"))
            jw = G.WALL_JAW_RAD if g["kind"] in ("wall", "base") or args.grasp_family == "side" else flags.jaw_wall_rad
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
            H_up = ((G.hover(g["T_tcp"], lift_m) if args.grasp_family == "side" else G.hover_along(g["T_tcp"], lift_m))
                    if lift_m else H)                                  # the side pinch lifts straight up
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
            else:
                held, rule = F.hold_verdict(flags, rise_m=float(T_o[2, 3] - z0),
                                            dist_to_tcp_m=float(np.linalg.norm(T_o[:3, 3] - T_t[:3, 3])))
            gate.update(held=held, rule=rule)
            tlog.write("hold", **gate)
            att.setdefault("holds", []).append(gate)
            if held:
                return {"grasp": g["kind"], "yaw": g["yaw"], "tilt_deg": g.get("tilt_deg"), "theta": theta,
                        "below_top_m": g.get("below_top"), "wrist_side": g.get("wrist_side"), "M": np.linalg.inv(T_t) @ T_o}, None
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
                             slide_pick)

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
    paths = list(robot.root_physx_view.link_paths[0])
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
    paths = list(robot.root_physx_view.link_paths[0])

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
    bp = robot.data.body_link_pos_w[0].cpu().numpy()
    bq = robot.data.body_link_quat_w[0].cpu().numpy()
    gaps = {}
    for bi, kn in dump["knuckles"].items():
        w = bq[bi]
        T_k = T_of(bp[bi], (w[1], w[2], w[3], w[0]))
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
                  slide_pick=None):
    """plans/2026-10-23-hardware-limits-mouth-up.md: the grasp gate (G3) or the placement trials (G4) on one dish.
    Each trial: the arm teleported home, the dish teleported to the counter spot +-5 mm (seeded), the racks set for the
    cell, 300 ticks, then the guard re-armed (a trial counts only when no harness invariant fired in it)."""
    from dishsim_frigidaire.random_poses import compose_pose
    oid = ids[0]
    kind = kinds[oid]
    rng = np.random.default_rng(args.seed)
    harness.abort_on = False
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

    def traj_tick(frames):
        harness.on_tick(frames)
        if dump is not None and dump["trial"] is not None:
            knuckle_dump_tick(dump, harness, robot, frames, oid, backend, rig, np, T_of)
        if traj["on"] and backend.tick_index % TRAJ_EVERY == 0:
            d, lo, up = frames[oid], frames["LowerRack"], frames["UpperRack"]
            traj["rows"].append((backend.tick_index * backend.dt, robot.data.body_link_pos_w[0].cpu().numpy().copy(),
                                 robot.data.body_link_quat_w[0].cpu().numpy().copy(),
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
        pose = {"position_m": [xy[0] + d[0], xy[1] + d[1], .914 + .002],
                "quaternion_xyzw": [0., 0., 0., 1.]}
        backend.set_rigid_pose(backend.objects[oid], pose)
        rig.step(K.RESET_TICKS)
        harness.pad_max_interval = {"pad_L": 0., "pad_R": 0.}
        harness.arm_guard()
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
    jobs = [(None, t) for t in range(args.n)]
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
        spot = None
        if isinstance(t, tuple):                       # --counter-grid: (spot, trial)
            spot, t = t
        tlog.set(move=k + 1, dish=oid, attempt=0, phase="trial reset")
        rack = site["rack"] if site else ("LowerRack" if args.gate_racks == "upper_in" else None)
        perturb = reset(t, rack, tuple(spot["xy"]) if spot else LIMITS_COUNTER_XY)
        if dump is not None:
            dump.update(trial=t, spot=None if spot is None else [spot["i"], spot["j"]], on=False, rows=0, last=None)
        n_viol = len(harness.violations)
        traj["rows"], traj["on"] = [], True
        att = {"attempt": 0, "dish": oid}
        row = {"mode": args.mode, "kind": kind, "trial": t, "perturb_mm": perturb, "site": site["site"] if site else None,
               "rack": rack, "racks": "upper rack pushed in" if rack == "LowerRack" else "lower rack pushed in"
               if rack == "UpperRack" else "both racks extended", "waive_dish_clearance": bool(args.waive_dish_clearance),
               "tilt_deg": None if args.plate_in_hand else tilt,
               "grasp_family": "in hand" if args.plate_in_hand else "slide" if args.plate_slide else
               "wall pinch" if args.grasp_family == "wall" else
               f"side pinch, pads {args.side_below_top * 1e3:.0f} mm under the rim" if args.grasp_family == "side" else "rim pinch",
               "spot": None if spot is None else {"i": spot["i"], "j": spot["j"], "xy": spot["xy"], "kitfree_gate": spot["gate"]}}
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
        viol = harness.violations[n_viol:]
        row["invariants"] = sorted({v["name"] for v in viol})
        row["counted"] = not row["invariants"]
        tag = (f"{site['site']}_inhand" if args.plate_in_hand else
               f"{site['site']}_{args.target}{'' if tilt is None else int(tilt)}{'' if yaw_k is None else f'_y{yaw_k}'}"
               f"{'_down' if args.release == 'down' else ''}"
               if site else ("slide" if args.plate_slide else f"spot{spot['i']}{spot['j']}" if spot else "gate"))
        row["traj"] = save_traj(f"{kind}_{tag}_s{args.seed}_t{t:02d}".replace("@", "_"))
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
    sys.path.insert(0, str(ROOT / "code/execution/frigidaire"))
    import frigidaire_robot_reach as RC
    backend, rig, harness, tlog, hook, robot = ns.backend, ns.rig, ns.harness, ns.tlog, ns.hook, ns.robot
    arm, G, C, K, B, T_of, T_now = ns.arm, ns.G, ns.C, ns.K, ns.B, ns.T_of, ns.T_now
    goal_qs, transit_to, sync_world, kinds = ns.goal_qs, ns.transit_to, ns.sync_world, ns.kinds
    load = json.loads(args.place_goal.read_text())
    L, out = load["load"], args.place_goal.parent
    names = [d["name"] for d in load["dishes"]]
    meta = {d["name"]: d for d in load["dishes"]}
    goal = {n: T_of(meta[n]["goal_pose"]["pos"], meta[n]["goal_pose"]["quat_xyzw"]) for n in names}
    F_goal = {r: T_of(f["position_m"], f["quaternion_xyzw"]) for r, f in load["frames"].items()}
    stage = omni.usd.get_context().get_stage()
    inv = np.linalg.inv
    pose = P.pose_dict
    harness.abort_on = False
    t_run = time.monotonic()

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

    def traj_tick(frames):
        harness.on_tick(frames)
        if traj["on"] and backend.tick_index % TRAJ_EVERY == 0:
            d, lo, up = frames[traj["oid"]], frames["LowerRack"], frames["UpperRack"]
            oth = np.array([np.r_[frames[n]["position_m"], frames[n]["quaternion_xyzw"]] for n in traj["others"]]).reshape(-1, 7)
            traj["rows"].append((backend.tick_index * backend.dt, robot.data.body_link_pos_w[0].cpu().numpy().copy(),
                                 robot.data.body_link_quat_w[0].cpu().numpy().copy(), np.r_[d["position_m"], d["quaternion_xyzw"]],
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
                            jaw=np.array([x[6] for x in r]), others=np.stack([x[7] for x in r]))
        traj["rows"] = []
        return str(path.relative_to(ROOT))

    # ---------------------------------------------------------------- the attachment
    att, att_mode = {}, {"pin": args.attach == "pin", "why": "--attach pin" if args.attach == "pin" else None}

    def attach(oid, T_d=None):
        """A PhysX fixed joint wrist_3 -> dish at their current relative pose, excluded from the articulation (so the
        robot's articulation is unchanged); the fallback (the joint did not hold on a first try): the dish written to that
        pose after every rig step."""
        T_d = T_now(oid) if T_d is None else np.asarray(T_d)
        X = rig.T_w3_tcp @ inv(rig.tcp()) @ T_d
        att.update(oid=oid, X=X, M=inv(rig.tcp()) @ T_d)
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

    def in_hand(kind):
        if kind not in Ms:
            if kind == "plate":                        # no plate grasp exists: the rim-pinch frame with zero pivot
                T_o = RC.counter_obj()
                g = G.bowl_grasps(T_o, RC.N_YAW, "plate")[0]
                Ms[kind] = (inv(g["T_tcp"]) @ T_o, {"source": "idealized: the plate rim-pinch frame (grasp.py KIND_RIM), "
                                                              "zero pivot", "pivot_deg": 0.})
            else:
                run = args.inhand_cup if kind == "cup" else None
                M, src = RC.measured_M(kind, run)
                Ms[kind] = (M, {"source": f"median hold of {run.name if run else 'limits_v1/gate2_' + kind}",
                                "trial": src["trial"], "pivot_deg": src["pivot_deg"]})
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

    def reset(i, oid, M):
        tlog.set(move=i + 1, dish=oid, attempt=0, phase="trial reset")
        harness.armed = False
        harness.clear_carried()
        detach()
        rig.gripper(False)
        rig.teleport_arm(ns.q_home)
        rig.step(5)
        set_racks(None)
        for j, n in enumerate(names):
            if n != oid:
                backend.set_rigid_pose(backend.objects[n], pose(goal[n]) if j < i else park_pose(j))
        backend.set_rigid_pose(backend.objects[oid], park_pose(i))
        rig.step(2)
        sync_world()
        kw = {"held": (kinds[oid], M, oid), "ignore": (oid,)}
        q_start = ns.q_home if arm.free(ns.q_home, **kw) else next(
            (q for q in rig.ik_all(ns.T_home, ns.q_home) if arm.free(q, **kw)), ns.q_home)
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
        set_racks(meta[oid]["rack"])
        theta = rig.gripper(True)
        sync_world()
        harness.pad_max_interval = {"pad_L": 0., "pad_R": 0.}
        harness.arm_guard()
        harness.set_carried(oid)
        M_now = inv(rig.tcp()) @ T_now(oid)
        return {"attach": "pin (fallback)" if att_mode["pin"] else "fixed joint", "attach_drift_mm": round(drift * 1e3, 2),
                "jaw_rad": round(theta, 4), "in_hand_vs_M_mm": round(float(np.linalg.norm(M_now[:3, 3] - M[:3, 3])) * 1e3, 2),
                "home_branch_free": bool(arm.free(q_start, **kw)), "q_start": np.round(q_start, 4).tolist(),
                "off_before_robot": off_goal(names[:i])}

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
            path = C.rrt_connect(arm, q0, [q], time_s=RRT_S, collide_kw=kw, seed=1)
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

    def release(oid):
        T_r = T_now(oid)
        detach()
        harness.clear_carried()
        tlog.set(phase="release")
        rig.gripper(False)
        tlog.set(phase="retreat")
        up = rig.tcp()
        up[2, 3] += flags.hover_m
        qs = rig.line_q(rig.tcp(), up, 40, rig.q())
        if qs is not None:
            rig.follow(qs)
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
                T_d[:3, 3] = (LIMITS_COUNTER_XY[0], LIMITS_COUNTER_XY[1], top + SET_DOWN_DROP_M - float((pts @ Rm.T)[:, 2].min()))
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
                        path = C.rrt_connect(arm, q0, [q_pre], time_s=RRT_S, collide_kw=kw, seed=1)
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
        off = float(np.hypot(T[0, 3] - LIMITS_COUNTER_XY[0], T[1, 3] - LIMITS_COUNTER_XY[1]))
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
        T[:3, 3] = (LIMITS_COUNTER_XY[0], LIMITS_COUNTER_XY[1], B.COUNTER["top_z_m"] + .003 + G.KIND_RIM[kinds[oid]][2])
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

    # ---------------------------------------------------------------- the dishes
    rows_path = out / "dishes.jsonl"
    variant = "S" if args.variant == "S" else "R-ideal" if args.regrasp_ideal else "R" if args.regrasp else "W"
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
    rows = []
    for oid in todo:
        i = names.index(oid)
        t0, s0, n_viol = time.monotonic(), backend.tick_index, len(harness.violations)
        T_goal, cand = None, side.get(oid)
        if variant == "S":                         # the best Kit-free candidate's measured hold and its turned goal
            best = cand["best"]
            M, src = (None, None) if best is None else (np.array(best["M"]), {k: best[k] for k in (
                "below_top_m", "yaw_k", "wrist_side", "gate_run", "trial", "pivot_deg", "goal_turn_deg")})
            T_goal = None if best is None else np.array(best["T_goal"])
        else:
            M, src = in_hand(kinds[oid])
        row = {"load": L, "dish": oid, "kind": kinds[oid], "grasp": "side pinch" if variant == "S" else meta[oid]["grasp"],
               "order_index": i, "variant": variant,
               "attached": True, "steps": {s: "skip" for s in STEPS}, "failed_at": None, "why": None, "error": None,
               "disturbed": [], "clip": None, "in_hand": src, "rack": meta[oid]["rack"], "slot": meta[oid]["slot"]}
        if cand is not None:
            row["candidates"] = cand["summary"]
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
        row["reset"] = reset(i, oid, M)
        traj.update(on=True, rows=[], oid=oid, others=names[:i])
        if variant == "R":
            set_down(oid, row)
        elif variant == "R-ideal":
            ideal_set_down(oid, row)
        if variant != "W" and row["failed_at"] is None:
            repick(oid, row)
        if row["failed_at"] is None:
            place_steps(i, oid, row, T_goal)
        row["attach_mode"] = row["reset"]["attach"]
        row["traj"] = save_traj(f"{L}_{oid}_{variant}")
        row["invariants"] = sorted({v["name"] for v in harness.violations[n_viol:]})
        row["sim_s"] = round((backend.tick_index - s0) * backend.dt, 2)
        row["wall_s"] = round(time.monotonic() - t0, 1)
        rows.append(row)
        with open(rows_path, "a") as f:
            f.write(json.dumps(row, default=lambda o: o.tolist() if hasattr(o, "tolist") else str(o)) + "\n")
        print(f"[INFO] {L} {variant} {oid} ({i + 1}/{len(names)}): {'PLACED' if row['failed_at'] is None else 'fail ' + row['failed_at']}"
              f" {row['steps']} error {row['error']} disturbed {row['disturbed']} attach {row['reset']['attach']} "
              f"{row['wall_s']} s  why {str(row['why'])[:220]}", flush=True)
    detach()
    report["rows"] = rows
    report["attach_mode"] = dict(att_mode)
    report["seconds_wall"] = round(time.monotonic() - t_run, 1)
    counts = {s: sum(r["failed_at"] == s for r in rows) for s in STEPS if any(r["failed_at"] == s for r in rows)}
    report["summary"] = {"load": L, "variant": variant, "dishes": len(rows), "placed": sum(r["failed_at"] is None for r in rows),
                         "failed_at": counts}
    report["result"] = "PASS"
    print(f"[RESULT] PASS place-goal {L} {variant}: placed {report['summary']['placed']}/{len(rows)}, failed at {counts}, "
          f"attach {att_mode}", flush=True)
    return finish()


if __name__ == "__main__":
    main()
