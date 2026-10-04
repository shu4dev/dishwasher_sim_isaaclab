#!/usr/bin/env python3
"""A UR5e + Robotiq 2F-85 executes an instance's moves in the Frigidaire twin (no teleport, no weld), under the
Phase 0 harness of plans/2026-09-29-easy-s0.md.

    code/scripts/run_kit.sh code/frigidaire/scripts/experiment/frigidaire_robot_episode.py --headless --enable_cameras \\
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

ROOT = Path(__file__).resolve().parents[4]
RUNS = ROOT / "data/results/robot/runs"        # one folder per run (plan D15); the pre-Phase-0 records stay in episodes/
ARTIFACTS = ROOT / "data/artifacts"            # per-run media/logs; a symlink onto the 2 TB drive (user 2026-09-29)
ROBOT_CAMERA = {"robot": ((-1.9, -2.7, 1.95), (.05, -.25, .62), {"focal_length": 42., "horizontal_aperture": 36.})}
VIDEO_HW = (720, 1280)                    # fixed camera, 1280 x 720 (user 2026-09-29)
VIDEO_FPS = 15                            # one frame per 8 physics steps at 120 Hz = real-time simulated seconds
DISTURB_MM, DISTURB_DEG = 10., 20.        # the any-neighbour rule's thresholds (= rearrange.DISTURB_POS_M / _ROT_DEG)
ALLOW_CONTACT_M = .06                     # planner: the grasped dish may touch the gripper in the last 60 mm (fix 10)
SCORING_GRACE_S = 300.                    # of --max-wall-seconds, kept for the end section (final hold, end check)
CODE_FILES = ("code/frigidaire/scripts/experiment/frigidaire_robot_episode.py",
              "code/frigidaire/src/dishsim_frigidaire/robot/flags.py", "code/frigidaire/src/dishsim_frigidaire/robot/harness.py",
              "code/frigidaire/src/dishsim_frigidaire/robot/triallog.py", "code/frigidaire/src/dishsim_frigidaire/robot/rig.py",
              "code/frigidaire/src/dishsim_frigidaire/robot/ur5e.py", "code/frigidaire/src/dishsim_frigidaire/robot/grasp.py",
              "code/frigidaire/src/dishsim_frigidaire/robot/collide.py", "code/frigidaire/src/dishsim_frigidaire/robot/kin.py",
              "code/frigidaire/scripts/experiment/frigidaire_bench.py", "code/frigidaire/scripts/experiment/frigidaire_bench_kit.py",
              "code/frigidaire/src/dishsim_frigidaire/initial_state_runtime.py")


def code_hashes():
    return {f: hashlib.sha256((ROOT / f).read_bytes()).hexdigest()[:16] for f in CODE_FILES}


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--instance", type=Path, required=True)
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
    parser.add_argument("--usd", type=Path, default=ROOT / "data/build/frigidaire_collection/usd/fdpc4221as.usdc")
    parser.add_argument("--max-wall-seconds", type=float, default=1700.,
                        help="wall budget of the physics loop (D16: no single run beyond 30 min without asking)")
    parser.add_argument("--run-id", default=None,
                        help="every output goes to data/results/robot/runs/<run-id>/ (record, backend dir) and "
                             "data/artifacts/<run-id>/<trial-id>/ (trial log, media); never overwritten (D15)")
    parser.add_argument("--trial-id", default="t00_nominal")
    from isaaclab.app import AppLauncher
    AppLauncher.add_app_launcher_args(parser)
    args = parser.parse_args()
    args.cameras = bool(getattr(args, "enable_cameras", False))
    if not args.cameras and not args.no_media:
        raise SystemExit("[RESULT] FAIL counted trials record media: launch with --enable_cameras (or pass --no-media)")
    stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
    args.run_id = args.run_id or f"{args.profile}_{args.instance.stem}_{stamp}"
    args.out_dir = RUNS / args.run_id
    args.trial_dir = ARTIFACTS / args.run_id / args.trial_id
    if args.out_dir.exists() or args.trial_dir.exists():
        raise SystemExit(f"[RESULT] FAIL refusing to overwrite run {args.run_id} ({args.out_dir} or {args.trial_dir} exists)")
    args.out_dir.mkdir(parents=True)
    args.trial_dir.mkdir(parents=True)
    app = AppLauncher(args).app
    args.app = app
    sys.path[:0] = [str(ROOT / "code/src"), str(ROOT / "code/frigidaire/src"), str(ROOT / "code/frigidaire/scripts/experiment")]
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
    flags = F.resolve(args.profile, **overrides)
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
    spec = importlib.util.spec_from_file_location("frigidaire_bench", ROOT / "code/frigidaire/scripts/experiment/frigidaire_bench.py")
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
            sys.path.insert(0, str(ROOT / "code/frigidaire/scripts/evaluation"))
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

    def pick(oid, att):
        sync_world()
        T_obj = T_now(oid)
        cands = G.tilted_grasps(T_obj)
        scored = []
        base_xy = np.asarray(mount["base"][:2])
        for g in cands:
            H = G.hover_along(g["T_tcp"], flags.hover_m)
            for q_h in goal_qs(H)[:4]:
                qs_line = rig.line_q(H, g["T_tcp"], 100, q_h)
                if qs_line is None:
                    continue
                d = float(np.linalg.norm(g["T_tcp"][:3, 3] - H[:3, 3]))
                ok = all(arm.free(q, gripper_may_touch=(oid,) if d * (1 - i / 99) < ALLOW_CONTACT_M else ())
                         for i, q in enumerate(qs_line))
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
            return None, f"no collision-free grasp ({len(cands)} candidates): {info}"
        scored.sort(key=lambda s: s[0])
        why = None
        tried = []
        for _, g, q_h, qs_line in scored[:6]:
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
            if not (flags.jaw_wall_rad[0] <= theta < flags.jaw_wall_rad[1]):         # the jaws closed on air or on the bowl body
                why = f"jaws at {theta:.2f} rad: no wall between the pads"
                tried.append(why)
                harness.clear_carried()
                rig.gripper(False)
                rig.move_line(H, n=60)
                continue
            z0 = T_now(oid)[2, 3]
            M_close = np.linalg.inv(rig.tcp()) @ T_now(oid)
            tlog.set(phase="lift")
            qs_up = rig.line_q(rig.tcp(), H, 80, rig.q())
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
                        "M": np.linalg.inv(T_t) @ T_o}, None
            why = (f"not held after the lift (jaws {theta:.2f}, lift ok {ok_lift}, dish rose {1e3 * (T_o[2, 3] - z0):.0f} mm, "
                   f"{1e3 * np.linalg.norm(T_o[:3, 3] - T_t[:3, 3]):.0f} mm from the TCP; gate {rule})")
            tried.append(why)
            harness.clear_carried()
            rig.gripper(False)
            rig.step(60)
        return None, f"{why} (tried {len(tried)}: {tried})"

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


if __name__ == "__main__":
    main()
