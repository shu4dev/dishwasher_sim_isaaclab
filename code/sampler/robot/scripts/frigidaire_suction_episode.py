#!/usr/bin/env python3
"""The UR5e with the EPick cup places sample s0's dishes from random counter spawns in the Frigidaire twin
(plans/2026-11-10-suction-random-spawn.md).

    code/util/run_kit.sh code/sampler/robot/scripts/frigidaire_suction_episode.py --headless --enable_cameras \\
        --check data/results/robot/suction_s0/check.json --run D --chain upper --video [--only cup_01]

Driver (no --dish; boots no Kit): the rack chain's dishes in the dag order, one Kit process per dish (this script with --dish),
each restoring the scene its predecessor left (--state). A dish without a hold in check.json for the run stays out (a row, no
process). A process that dies on "malloc(): corrupted top size" is killed by its process group and run once more; one still
alive 90 s after its [RESULT] line is killed. Writes <check dir>/<run>/<chain>/rows.jsonl and, with --video, joins the dish
clips into data/media/robot/suction_s0/suction_<run>_<chain>.mp4.

Dish process (--dish): the twin backend (initial_state_runtime) on CPU physics with PhysX scene queries on (the surface gripper's
raycast), the counter slab pulled to y -0.17, the basket out, the dish's rack out and the other pushed in, the pedestal, the UR5e
and the EPick body (epick.author) on posture_v2; the chain's earlier dishes where the state file left them. The dish spawns at its
check.json pose (run D mouth down, run U mouth up) and settles 2 s; the arm goes from home to the pre-pick (a straight joint move,
else RRT-Connect), along the tool axis to the hold, closes (150 ms; a 2 mm back-off and a second close if nothing attaches), lifts
100 mm, carries the dish to the approach start (straight, else RRT-Connect with the held dish 5 mm clear), follows the straight
approach to the release (the goal raised 3 mm, from the measured hold), opens (180 ms; an RTX still), retreats 50 mm along the
tool axis and waits 150 ticks. Judged: at rest (the last 60 ticks drift within 5 mm / 3 deg), at goal (frigidaire_bench.AT_GOAL
of the goal in the rack's current frame) and mouth down; placed = all three. A failed dish stays where it landed. Writes the row,
the still, the clip and the next state file.
"""
from __future__ import annotations

import argparse
import copy
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import traceback

ROOT = Path(__file__).resolve().parents[4]
MEDIA = ROOT / "data/media/robot/suction_s0"
LOGS = ROOT / "data/logs/robot"
USD = ROOT / "data/assets/frigidaire_fdpc4221as/usd/fdpc4221as.usdc"
CAMERA = {"robot": ((-1.9, -2.7, 1.95), (.05, -.25, .62), {"focal_length": 42., "horizontal_aperture": 36.})}   # the old episode's view
VIDEO_HW, VIDEO_EVERY = (960, 1280), 8          # a frame per 8 ticks: 15 fps of simulated time
STILL_RENDERS = 48
SPAWN_TICKS, STATE_TICKS, SETTLE_TICKS, WINDOW_TICKS = 240, 120, 150, 60
REST_MM, REST_DEG = 5., 3.
CLOSE_TRIES_M = (0., .002)                      # the tip at the hold, then 2 mm back
RRT_S = 60.


def dirs(args):
    rdir = Path(args.check).resolve().parent / args.run / args.chain
    mdir = MEDIA / args.run / args.chain
    for d in (rdir, mdir, LOGS):
        d.mkdir(parents=True, exist_ok=True)
    return rdir, mdir


# ---------------------------------------------------------------------------------------------------- driver (no Kit)

def run_one(cmd, log, limit):
    """One dish process -> its end: "result", "result, killed in close", "malloc", "timeout" or "exit <code>"."""
    with open(log, "w") as f:
        p = subprocess.Popen(cmd, stdout=f, stderr=subprocess.STDOUT, cwd=ROOT, start_new_session=True)
    t0, t_res = time.monotonic(), None

    def kill():
        try:
            os.killpg(p.pid, signal.SIGKILL)          # this child's own process group, never a broad pkill
        except ProcessLookupError:
            pass
        p.wait()
    while p.poll() is None:
        time.sleep(5)
        txt = log.read_text(errors="ignore")
        if "malloc(): corrupted top size" in txt:
            kill()
            return "malloc"
        if t_res is None and "[RESULT]" in txt:
            t_res = time.monotonic()
        if t_res is not None and time.monotonic() - t_res > 90:
            kill()
            return "result, killed in close"
        if time.monotonic() - t0 > limit:
            kill()
            return "timeout"
    return "result" if "[RESULT]" in log.read_text(errors="ignore") else f"exit {p.returncode}"


def drive(args):
    chk = json.loads(Path(args.check).read_text())
    rdir, mdir = dirs(args)
    only = set(args.only.split(",")) if args.only else None
    state, rows, t0 = None, [], time.monotonic()
    for k, oid in enumerate(chk["chains"][args.chain]):
        if only and oid not in only:
            continue
        d = chk["dishes"][oid]
        run = d["runs"][args.run]
        if not run["hold"]:
            blockers = sorted({b for h in d["holds"].values() for b in h.get("blockers", [])})
            rows.append({"dish": oid, "kind": d["kind"], "run": args.run, "chain": args.chain, "k": k, "hold": None,
                         "outcome": "stays out", "cause": run["cause"], "blockers": blockers})
            print(f"[INFO] {oid}: stays out ({run['cause']})", flush=True)
            continue
        cmd = [str(ROOT / "code/util/run_kit.sh"), str(Path(__file__).resolve()), "--headless", "--check", str(args.check),
               "--run", args.run, "--chain", args.chain, "--dish", oid, "--k", str(k), "--max-wall-seconds", str(args.max_wall_seconds)]
        cmd += ["--enable_cameras", "--video"] if args.video else []
        cmd += ["--state", str(state)] if state else []
        log = LOGS / f"suction_{args.run}_{args.chain}_{k:02d}_{oid}.log"
        rowp = rdir / f"row_{k:02d}_{oid}.json"
        rowp.unlink(missing_ok=True)
        ends = []
        for attempt in (1, 2):
            ends.append(run_one(cmd, log, args.max_wall_seconds + 300))
            print(f"[INFO] {oid}: attempt {attempt} {ends[-1]} ({time.monotonic() - t0:.0f} s)", flush=True)
            if ends[-1] != "malloc":
                break
            log.rename(log.with_name(log.stem + f"_malloc{attempt}.log"))
        row = json.loads(rowp.read_text()) if rowp.exists() else {"dish": oid, "kind": d["kind"], "run": args.run,
                                                                    "chain": args.chain, "k": k, "outcome": "no row"}
        row["process"] = {"ends": ends, "log": str(log.relative_to(ROOT))}
        rows.append(row)
        st = rdir / f"after_{k:02d}_{oid}.json"
        state = st if st.exists() else state
    (rdir / "rows.jsonl").write_text("".join(json.dumps(r) + "\n" for r in rows))
    video = None
    clips = sorted(str(p) for p in mdir.glob("clip_*.mp4") if not only or any(o in p.name for o in only))
    if args.video and clips:
        import imageio_ffmpeg
        lst = mdir / "clips.txt"
        lst.write_text("".join(f"file '{c}'\n" for c in clips))
        video = MEDIA / f"suction_{args.run}_{args.chain}{'_only' if only else ''}.mp4"
        subprocess.run([imageio_ffmpeg.get_ffmpeg_exe(), "-y", "-loglevel", "error", "-f", "concat", "-safe", "0", "-i", str(lst),
                        "-c", "copy", str(video)], check=True)
    placed = [r["dish"] for r in rows if r.get("placed")]
    print(f"[RESULT] PASS suction chain {args.run} {args.chain}: placed {len(placed)} {placed}; rows "
          f"{[(r['dish'], r.get('outcome')) for r in rows]}; video {video}; {time.monotonic() - t0:.0f} s", flush=True)


# ---------------------------------------------------------------------------------------------------- dish process (Kit)

def remove_basket(stage, twin):
    """Basket out before sim.reset: its collider and visual scopes deactivated, the shapeless body kept with gravity off (the
    backend's RigidObject indexes it)."""
    from pxr import PhysxSchema, Usd, UsdPhysics
    path = f"{twin}/SilverwareBasket"
    n = 0
    for part in ("Collisions", "Visuals"):
        p = stage.GetPrimAtPath(f"{path}/{part}")
        if not p or not p.IsValid():
            raise RuntimeError(f"basket out: no {part} under {path}")
        n += sum(1 for x in Usd.PrimRange(p) if x.HasAPI(UsdPhysics.CollisionAPI))
        p.SetActive(False)
    PhysxSchema.PhysxRigidBodyAPI.Apply(stage.GetPrimAtPath(path)).CreateDisableGravityAttr(True)
    print(f"[basket] removed {path} ({n} colliders deactivated)", flush=True)
    return n


def tint(stage, tints, counter_color):
    """One colour per dish (CLAUDE.md) and a visible counter slab (frigidaire_bench_kit.tint_scene's rule)."""
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdShade

    def material(name, color, roughness):
        m = UsdShade.Material.Define(stage, f"/World/SuctionMaterials/{name}")
        sh = UsdShade.Shader.Define(stage, m.GetPath().AppendChild("Surface"))
        sh.CreateIdAttr("UsdPreviewSurface")
        sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
        sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(roughness)
        m.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
        return m
    for path, m in [(f"/World/InitialStateDishes/{o}", material(o, c, .36)) for o, c in tints.items()] + \
            [("/World/Counter", material("Counter", counter_color, .5))]:
        for child in Usd.PrimRange(stage.GetPrimAtPath(path)):
            if child.IsA(UsdGeom.Gprim):
                UsdShade.MaterialBindingAPI.Apply(child).Bind(m, bindingStrength=UsdShade.Tokens.strongerThanDescendants)


def dish_main(args):
    from dishsim.app import launch
    app = launch(args)
    t_wall = time.monotonic()
    rdir, mdir = dirs(args)
    oid, k_ = args.dish, args.k
    row = {"dish": oid, "run": args.run, "chain": args.chain, "k": k_, "outcome": "error", "placed": False}
    rec = {"vw": None}
    try:
        episode(args, app, rdir, mdir, row, rec)
    except Exception:
        row["error"] = traceback.format_exc()
        print(row["error"], flush=True)
    finally:
        if rec["vw"] is not None:
            rec["vw"].close()
    row["wall_s"] = round(time.monotonic() - t_wall, 1)
    (rdir / f"row_{k_:02d}_{oid}.json").write_text(json.dumps(row, indent=1) + "\n")
    e = row.get("judge", {})
    print(f"[RESULT] {'FAIL' if row.get('error') else 'PASS'} suction {args.run} {args.chain} {oid}: outcome {row['outcome']} placed "
          f"{row['placed']} picked {row.get('picked')} carried {row.get('carried')} released {row.get('released')} at rest "
          f"{e.get('at_rest')} at goal {e.get('at_goal')} mouth down {e.get('mouth_down')} error {e.get('lateral_mm')} / "
          f"{e.get('dz_mm')} mm / {e.get('tilt_deg')} deg; cause {row.get('cause')}; {row['wall_s']} s", flush=True)
    try:
        from dishsim.media import release_sim_for_close
        release_sim_for_close()
    finally:
        app.close()


def episode(args, app, rdir, mdir, row, rec):
    import numpy as np
    import torch
    import omni.usd
    import isaaclab.sim as sim_utils
    from isaaclab.assets import Articulation
    from pxr import PhysxSchema, Usd, UsdPhysics
    sim_utils.enable_extension("isaacsim.robot.surface_gripper")     # before authoring (robot schema) and before play
    from isaacsim.robot.surface_gripper import _surface_gripper
    from isaaclab_physx.assets import SurfaceGripper
    from sampler.initial_state_runtime import IsaacInitialStateBackend
    from sampler.random_poses import source_geometry_domains
    from sampler.random_pose_runtime import RACK_JOINT
    from sampler.robot import collide as C, epick, harness as HN, ur5e
    from sampler.robot.rig import Rig
    import frigidaire_suction_check as SC
    sys.path.insert(0, str(ROOT / "code/sampler/scripts"))
    from frigidaire_initial_state_render import item_tints

    chk = json.loads(Path(args.check).read_text())
    oid, run = args.dish, args.run
    d = chk["dishes"][oid]
    kind, rack = d["kind"], d["rack"]
    hold = d["runs"][run]["hold"]
    cand = next(c for h in d["holds"].values() for c in h["candidates"] if c["label"] == hold)
    pick = d["spawn"]["picks"][run]
    H, T_goal = np.asarray(cand["H"]), SC.T_pose(d["goal"])
    ids = chk["chains"][args.chain]
    kinds = {o: chk["dishes"][o]["kind"] for o in ids}
    state = json.loads(Path(args.state).read_text()) if args.state else {"dishes": {}}
    row.update(kind=kind, rack=rack, slot=d.get("slot"), hold=hold, standoff_mm=cand.get("standoff_mm"),
               approach=[cand.get("approach"), cand.get("approach_m")], spawn=d["spawn"]["xy_yaw"], spawn_draw=d["spawn"]["draws"],
               spawn_pose=pick["spawn_pose"], state_in=args.state, scene=sorted(state["dishes"]))
    B = SC.bench()
    T_wb, mount = SC.mount_T(ROOT / chk["mount"])
    base = json.loads((B.OUT / "baseline" / "result.json").read_text())["snapshot"]
    holder = {}

    def before_reset(backend):
        stage = omni.usd.get_context().get_stage()
        scene = next(p for p in stage.Traverse() if p.IsA(UsdPhysics.Scene))
        # scene queries on (the surface gripper's raycast): initial_state_runtime's SimulationCfg leaves them off, and PhysX
        # reads this scene attribute at play (sim.reset), after Isaac Lab wrote its own value at SimulationContext creation
        PhysxSchema.PhysxSceneAPI(scene).CreateEnableSceneQuerySupportAttr().Set(True)
        holder["basket_colliders_off"] = remove_basket(stage, backend.prim)
        ped = sim_utils.CylinderCfg(radius=.08, height=float(mount["base"][2]),
                                    collision_props=[sim_utils.UsdPhysicsCollisionCfg()],
                                    visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(.35, .36, .40)))
        ped.func("/World/Pedestal", ped, translation=(mount["base"][0], mount["base"][1], mount["base"][2] / 2))
        yaw = mount["yaw_rad"]
        holder["robot"] = Articulation(ur5e.robot_cfg(pos=tuple(mount["base"]), rot_xyzw=(0., 0., math.sin(yaw / 2), math.cos(yaw / 2))))
        w3 = next(str(p.GetPath()) for p in Usd.PrimRange(stage.GetPrimAtPath("/World/Robot"))
                  if p.GetName() == "wrist_3_link" and p.HasAPI(UsdPhysics.RigidBodyAPI))
        holder["gripper"] = SurfaceGripper(epick.gripper_cfg(epick.author(stage, w3, "/World/Robot/epick")))
        HN.author_sleep_and_reports(stage, [f"/World/InitialStateDishes/{o}" for o in backend.objects])
        tint(stage, item_tints([{"object_id": o} for o in backend.objects]), B.COUNTER_COLOR)
        if args.video:
            from dishsim.media import CameraRig
            holder["cams"] = CameraRig(CAMERA, hw=VIDEO_HW)

    slab = ("/World/Counter", tuple(B.COUNTER["size_m"]), tuple(B.COUNTER["center_m"]))
    entries = [{"object_id": o, "kind": kinds[o], "rack": chk["dishes"][o]["rack"], "pose_world": chk["dishes"][o]["goal"]} for o in ids]
    backend = IsaacInitialStateBackend(USD, rdir / "_kit" / oid, device="cpu", domains=source_geometry_domains(),
                                       deadline=time.monotonic() + args.max_wall_seconds, app=app, candidates=entries,
                                       extra_statics=[slab], tableware=B.tableware(), before_reset=before_reset)
    robot, gripper, cams = holder["robot"], holder["gripper"], holder.get("cams")
    view, gstat = gripper.gripper_view, _surface_gripper.GripperStatus
    dish_path = f"/World/InitialStateDishes/{oid}"
    film = {"on": False, "phase": "reset", "n": 0}
    font = None
    if cams is not None:
        from PIL import Image, ImageDraw, ImageFont
        import matplotlib
        from dishsim.media import VideoWriter
        font = ImageFont.truetype(str(Path(matplotlib.get_data_path()) / "fonts/ttf/DejaVuSans.ttf"), 24)
        cams.apply_poses("cpu")
        if args.video:
            rec["vw"] = VideoWriter(str(mdir / f"clip_{args.k:02d}_{oid}.mp4"), fps=round(1 / (backend.dt * VIDEO_EVERY)))

    def caption(img, text):
        im = Image.fromarray(img)
        dr = ImageDraw.Draw(im, "RGBA")
        dr.rectangle([0, 0, dr.textlength(text, font=font) + 24, 44], fill=(0, 0, 0, 160))
        dr.text((12, 9), text, font=font, fill=(255, 255, 255, 255))
        return np.asarray(im)

    def on_step():
        if rec["vw"] is not None and film["on"]:
            film["n"] += 1
            if film["n"] % VIDEO_EVERY == 0:
                backend.sim.render()
                cams.update(backend.dt)
                rec["vw"].add(caption(cams.grab_one("robot"), f"run {run}  {args.chain} rack  {args.k + 1}. {oid} ({hold})  "
                                                              f"{film['phase']}  t {film['n'] * backend.dt:5.1f} s"))
        w = film.get("watch")
        if w is not None:
            w["max_mm"] = max(w["max_mm"], float(np.linalg.norm(T_now(oid)[:3, 3] - (rig.tcp() @ w["M"])[:3, 3])) * 1e3)

    def still(name, target):
        if cams is None:
            return None
        cam = cams.cams["robot"]
        c = np.asarray(target, dtype=float)
        cam.set_world_poses_from_view(eyes=torch.tensor([c + np.array([-.45, -.55, .30])], dtype=torch.float32),
                                      targets=torch.tensor([c], dtype=torch.float32))
        for _ in range(STILL_RENDERS):
            backend.sim.render()
        cams.update(backend.dt)
        p = mdir / f"{name}_{args.k:02d}_{oid}.png"
        Image.fromarray(caption(cams.grab_one("robot"), f"run {run}  {oid} ({hold})  {name}")).save(p)
        cams.apply_poses("cpu")
        return str(p.relative_to(ROOT))

    def T_now(o):
        return SC.T_pose(backend.poses()[o])

    def status():
        s = view.get_surface_gripper_status()[0]
        return s if isinstance(s, str) else gstat(s).name

    def held_now():
        return any(g == dish_path or g.startswith(dish_path + "/") for g in view.get_gripped_objects()[0])

    # ---- scene: the arm home before any physics step, the racks, the earlier dishes as the previous process left them
    rig = Rig(backend.sim, robot, T_wb, on_step=on_step, physics_step=backend.tick)
    q_home = np.asarray(chk["q_home"])
    rig.teleport_arm(q_home)
    other = "UpperRack" if rack == "LowerRack" else "LowerRack"
    snap = copy.deepcopy(base)
    snap["joints"][RACK_JOINT[other]] = 0.                            # the other rack pushed in
    backend.restore(snap)
    rig.teleport_arm(q_home)
    rig.step(30)
    for o, p in state["dishes"].items():
        backend.set_rigid_pose(backend.objects[o], p)
    rig.step(STATE_TICKS)
    bi = list(robot.body_names).index("base_link")
    T_sim_base = SC.T_of(robot.data.body_link_pos_w.torch[0, bi].cpu().numpy(), robot.data.body_link_quat_w.torch[0, bi].cpu().numpy())
    now = backend.poses()
    row["setup"] = {"base_vs_mount_mm": round(float(np.linalg.norm(T_sim_base[:3, 3] - T_wb[:3, 3])) * 1e3, 3),
                    "racks": {j: round(v, 4) for j, v in backend.joints().items()},
                    "state_drift_mm": {o: round(float(np.linalg.norm(np.subtract(now[o]["position_m"], p["position_m"]))) * 1e3, 2)
                                       for o, p in state["dishes"].items()},
                    "basket_colliders_off": holder["basket_colliders_off"]}
    from sampler.initial_state_candidates import COMPONENT_NAMES
    W = SC.World(B, rack, ROOT / chk["mount"], frames={c: now[c] for c in COMPONENT_NAMES})
    W.arm.set_dishes({o: W.body(kinds[o], T_now(o), o) for o in state["dishes"]})

    def plan(q0, q1, held=None):
        """A straight joint move when it is clear under the transit rule, else RRT-Connect (60 s, then the 180 s retry)."""
        W.arm.margin, W.arm.held_clear_m = SC.TRANSIT_MARGIN_M, SC.CARRY_CLEAR_M
        if W.arm.path_free(q0, q1, step=.02, held=held):
            return "straight", None
        t = time.monotonic()
        path = C.rrt_connect(W.arm, q0, [q1], time_s=RRT_S, seed=1, collide_kw={"held": held}, retry_s=C.NARROW_RETRY_S)
        return ("rrt" if path else None), {"path": path, "plan_s": round(time.monotonic() - t, 1)}

    def go(kind_, info, q1):
        return rig.move_joint(q1) if kind_ == "straight" else rig.follow(info["path"], steps_per=3)

    def first_clear(T, seed, held=None, mode="transit", may=()):
        for q in W.kr.ik_all(T, seed):
            if not W.hits(q, mode, held, may):
                return q
        return None

    def judge_and_save():
        """150 ticks with the arm still; at rest, at goal (in the rack's current frame) and mouth down; the next state file holds
        every dish of the scene where it lies, this one included (a failed dish stays where it landed)."""
        film["phase"] = "settle"
        hist = []
        for s_ in range(SETTLE_TICKS):
            rig.step(1)
            if s_ >= SETTLE_TICKS - WINDOW_TICKS:
                hist.append(T_now(oid))
        now = backend.poses()
        T_end = hist[-1]
        drift_mm = float(np.linalg.norm(hist[-1][:3, 3] - hist[0][:3, 3])) * 1e3
        drift_deg = float(np.degrees(np.arccos(np.clip((np.trace(hist[0][:3, :3].T @ hist[-1][:3, :3]) - 1) / 2, -1., 1.))))
        T_goal_now = SC.T_pose(now[rack]) @ np.linalg.inv(SC.T_pose(base["poses"][rack])) @ T_goal
        dist = B.goal_distance(np.asarray(backend.points[kind]).mean(axis=0), T_end, T_goal_now)
        j = {"at_rest": bool(drift_mm <= REST_MM and drift_deg <= REST_DEG), "drift_mm": round(drift_mm, 2), "drift_deg": round(drift_deg, 2),
             "at_goal": bool(B.within_goal(kind, dist)), "lateral_mm": round(dist[0] * 1e3, 1), "dz_mm": round(dist[1] * 1e3, 1),
             "tilt_deg": round(dist[2], 1), "tolerance": list(B.AT_GOAL[kind]), "mouth_down": bool(T_end[2, 2] < 0.), "pose": now[oid]}
        row["judge"] = j
        row["placed"] = bool(row["released"] and j["at_rest"] and j["at_goal"] and j["mouth_down"])
        if row["outcome"].startswith("released") and not row["placed"]:
            row["cause"] = "; ".join(w for w, ok in (("not at rest", j["at_rest"]), ("not at goal", j["at_goal"]),
                                                     ("mouth up", j["mouth_down"])) if not ok)
        st = {"after": oid, "k": args.k, "run": run, "chain": args.chain, "dishes": {o: now[o] for o in [*state["dishes"], oid]},
              "rack_joints": backend.joints(), "rule": "every dish this chain spawned so far, where it lies (placed or not)"}
        (rdir / f"after_{args.k:02d}_{oid}.json").write_text(json.dumps(st, indent=1) + "\n")
        film["on"] = False

    def put_back(why):
        """The dish lowered to where it was picked and let go (it stays on the counter)."""
        film["phase"] = f"put back ({why})"
        rig.move_line(T_pick_now)
        view.apply_gripper_action([-1.])
        rig.step(round(epick.RELEASE_S / backend.dt) + 10)
        rig.move_line(SC.along(rig.tcp(), -SC.PRE_PICK_M))

    # ---- spawn and pick
    film.update(on=True, phase="spawn")
    backend.set_rigid_pose(backend.objects[oid], pick["spawn_pose"])
    rig.step(SPAWN_TICKS)
    T0 = T_now(oid)
    row["spawn_settle_mm"] = round(float(np.linalg.norm(T0[:3, 3] - SC.T_pose(pick["spawn_pose"])[:3, 3])) * 1e3, 2)
    W.arm.dishes[oid] = W.body(kind, T0, oid)
    T_pick_now = T0 @ H
    T_pre = SC.along(T_pick_now, -SC.PRE_PICK_M)
    row["picked"] = row["carried"] = row["released"] = False
    q_pre = first_clear(T_pre, np.asarray(pick["q_pre"]))
    leg = plan(rig.q(), q_pre) if q_pre is not None else (None, None)
    row["transit"] = {"kind": leg[0], "plan_s": (leg[1] or {}).get("plan_s")}
    if q_pre is None or leg[0] is None:
        row.update(outcome="not picked", cause="no clear pre-pick" if q_pre is None else "no transit path home -> pre-pick")
        return judge_and_save()
    film["phase"] = "to the pick"
    if not go(*leg, q_pre):
        row.update(outcome="not picked", cause=f"transit blocked: {getattr(rig, 'last_fail', None)}")
        return judge_and_save()
    attach = []
    for so in CLOSE_TRIES_M:
        film["phase"] = "pick: approach"
        line_ok = rig.move_line(SC.along(T_pick_now, -so), n=40)
        film["phase"] = "pick: close"
        view.apply_gripper_action([1.])
        t_att, trace = None, []
        for i in range(1, round(epick.RETRY_INTERVAL_S / backend.dt) + 1):
            rig.step(1)
            trace.append(status())
            if t_att is None and held_now():
                t_att = i
            if t_att is not None and trace[-1] == "Closed":
                break
        rig.step(max(0, round(epick.GRIP_S / backend.dt) - i))
        attach.append({"standoff_mm": so * 1e3, "line_ok": line_ok, "attach_ms": None if t_att is None else round(t_att * backend.dt * 1e3, 1),
                       "status": status(), "gripped": list(view.get_gripped_objects()[0])})
        if held_now():
            break
        view.apply_gripper_action([-1.])
        rig.step(round(epick.RELEASE_S / backend.dt))
    row["attach"] = attach
    if not held_now():
        rig.move_line(T_pre)
        row.update(outcome="not picked", cause="nothing attached at the hold")
        return judge_and_save()
    M = np.linalg.inv(rig.tcp()) @ T_now(oid)                      # the measured hold
    W.arm.dishes.pop(oid, None)
    held = (kind, M, oid)
    film["phase"] = "lift"
    z0 = T_now(oid)[2, 3]
    lift_ok = rig.move_line(SC.up(rig.tcp(), SC.LIFT_M), n=40)
    rise = (T_now(oid)[2, 3] - z0) * 1e3
    row["pick"] = {"lift_ok": lift_ok, "rise_mm": round(float(rise), 1), "hold_vs_plan_mm": round(float(np.linalg.norm((np.linalg.inv(H) - M)[:3, 3])) * 1e3, 2)}
    row["picked"] = bool(held_now() and rise > SC.LIFT_M * 1e3 - 10.)
    if not row["picked"]:
        row.update(outcome="not picked", cause=f"the lift: rise {rise:.0f} mm, held {held_now()}")
        if held_now():
            put_back("lift")
        return judge_and_save()

    # ---- carry, approach, release
    T_rel = SC.up(T_goal, SC.RELEASE_UP_M) @ np.linalg.inv(M)
    T_app = np.array(T_rel)
    T_app[:3, 3] -= cand["approach_m"] * np.asarray(cand["approach_dir"])
    q_app = first_clear(T_app, np.asarray(cand["q_app"]), held)
    leg = plan(rig.q(), q_app, held) if q_app is not None else (None, None)
    row["carry"] = {"kind": leg[0], "plan_s": (leg[1] or {}).get("plan_s"), "q_app_clear": q_app is not None}
    if leg[0] is None:
        put_back("no carry path")
        row.update(outcome="not carried", cause="no clear approach start for the measured hold" if q_app is None else
                   "no carry path (RRT-Connect 60 s + 180 s)")
        return judge_and_save()
    film.update(phase="carry", watch={"M": M, "max_mm": 0.})
    moved = go(*leg, q_app)
    row["carry"].update(moved=moved, fail=None if moved else getattr(rig, "last_fail", None),
                        drift_max_mm=round(film["watch"]["max_mm"], 2))
    row["carried"] = bool(moved and held_now() and film["watch"]["max_mm"] < 5.)
    if not row["carried"]:
        row.update(outcome="not carried", cause=f"carry: moved {moved}, held {held_now()}, drift {film['watch']['max_mm']:.1f} mm")
        film.pop("watch")
        if held_now():
            view.apply_gripper_action([-1.])
            rig.step(round(epick.RELEASE_S / backend.dt) + 10)
        return judge_and_save()
    film["phase"] = "approach"
    app_ok = rig.move_line(T_rel, n=60)
    row["approach_ok"] = app_ok
    row["approach_fail"] = None if app_ok else getattr(rig, "last_fail", None)
    film.pop("watch")
    film["phase"] = "release"
    view.apply_gripper_action([-1.])
    rig.step(round(epick.RELEASE_S / backend.dt) + 10)
    row["released"] = bool(not held_now() and status() == "Open")
    row["release_tcp_err_mm"] = round(float(np.linalg.norm(rig.tcp()[:3, 3] - T_rel[:3, 3])) * 1e3, 2)
    row["still"] = still("release", T_goal[:3, 3])
    film["phase"] = "retreat"
    row["retreat_ok"] = rig.move_line(SC.along(rig.tcp(), -SC.RETREAT_M), n=30)
    row["outcome"] = "released" if app_ok else "released short (approach blocked)"
    return judge_and_save()


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sys.path[:0] = [str(ROOT / "code"), str(ROOT / "code/util"), str(Path(__file__).resolve().parent)]
    from dishsim.app import add_args
    add_args(parser)
    parser.add_argument("--check", type=Path, required=True)
    parser.add_argument("--run", choices=("D", "U"), required=True)
    parser.add_argument("--chain", choices=("upper", "lower"), required=True)
    parser.add_argument("--only", default=None, help="comma-separated dishes of the chain (the smoke)")
    parser.add_argument("--video", action="store_true", help="1280x960 clips (needs --enable_cameras)")
    parser.add_argument("--max-wall-seconds", type=float, default=1800.)
    parser.add_argument("--dish", default=None, help="internal: the driver's dish process")
    parser.add_argument("--k", type=int, default=0, help="internal: the dish's index in its chain")
    parser.add_argument("--state", type=Path, default=None, help="internal: the previous dish's state file")
    args = parser.parse_args()
    if args.video and not args.enable_cameras:
        parser.error("--video needs --enable_cameras")
    if args.dish is None:
        drive(args)
    else:
        dish_main(args)


if __name__ == "__main__":
    main()
