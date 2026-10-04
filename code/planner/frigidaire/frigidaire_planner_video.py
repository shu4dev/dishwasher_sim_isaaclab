#!/usr/bin/env python3
"""Stop-motion video of one planner episode with Isaac RTX: the messy counter pile and racks, move by move.

    code/util/run_kit.sh code/planner/frigidaire/frigidaire_planner_video.py \
        --instance data/results/planner/frigidaire/instances/s0_n24.json \
        --episode data/results/planner/frigidaire/episodes/s0_n24_planner.json \
        --out-dir data/results/planner/frigidaire/video --headless --enable_cameras

Same scene, lights, camera and per-item tints as frigidaire_initial_state_render.py, plus the
worktop slab as a visual box. Every object starts at its settled initial pose with dynamics
disabled; each executed move re-poses one prim (no physics), holds for --hold-frames frames
with a caption, and the last arrangement is held longer. Writes <instance>_<algorithm>.mp4,
the first and last frames as PNGs, and a small evidence JSON.

HOTEC benchmark episodes (frigidaire_bench_kit.py) work too: the instance's tableware map and
counter slab are used, every executed move shows the Isaac-SETTLED poses of all dishes after it
(``poses_after``, so a disturbed neighbour moves as well), refusals are skipped, and S with the
racked count comes from the episode's ``.analysis.json`` (frigidaire_bench.py --analyze).
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import sys
import time
import traceback

ROOT = Path(__file__).resolve().parents[3]


def render(args, report):
    import numpy as np
    import omni.usd
    import isaaclab.sim as sim_utils
    from isaaclab.sim import SimulationContext
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade
    from dishsim.media import CameraRig, VideoWriter
    from dishsim.quats import xyzw_to_wxyz
    from dishsim import legacy_data_path
    from dishsim.transforms import T_to_pos_quat
    from dishsim_frigidaire.asset import COMPONENT_FILES
    from dishsim_frigidaire.planner import COUNTER, in_counter_band
    from frigidaire_initial_state_render import CAMERA, WIDTH, HEIGHT, item_tints, caption_fonts, digest

    inst = json.loads(args.instance.read_text())
    episode = json.loads(args.episode.read_text())
    if inst.get("purpose") not in ("planner_instance", "hotec_bench_instance") or episode.get("instance") != inst["instance_id"]:
        raise ValueError("Instance and episode do not match")
    bench = inst["purpose"] == "hotec_bench_instance"
    stem = f"{inst['instance_id']}__{episode['track']}__{episode['algorithm']}" if bench else f"{inst['instance_id']}_{episode['algorithm']}"
    counter = inst["counter"] if "size_m" in inst.get("counter", {}) else COUNTER      # the bench's 1.8 m slab
    usd_of = ({kind: Path(legacy_data_path(path)) for kind, path in inst["tableware"].items()} if inst.get("tableware") else
              {o["kind"]: args.usd.parent / "tableware" / f"{o['kind']}.usdc" for o in inst["objects"]})
    analysis_path = args.episode.with_suffix(".analysis.json")
    trace = {t["move"]: t for t in json.loads(analysis_path.read_text())["trace"]} if analysis_path.is_file() else {}
    s_final = json.loads(analysis_path.read_text())["S_final"] if analysis_path.is_file() else None
    video_path = args.out_dir / f"{stem}.mp4"
    if video_path.exists():
        raise FileExistsError(f"Refusing to overwrite video: {video_path}")
    saved = inst["initial_snapshot"]["poses"]
    tints = item_tints([{"object_id": o["object_id"]} for o in inst["objects"]])
    kinds = {o["object_id"]: o["kind"] for o in inst["objects"]}
    report.update(instance=str(args.instance), episode=str(args.episode), instance_sha256=digest(args.instance),
                  episode_sha256=digest(args.episode), moves=len(episode["moves"]), camera=CAMERA,
                  runtime={"isaac_sim": Path("/isaac-sim/VERSION").read_text().strip()},
                  scope="Isaac RTX stop-motion of a planned move sequence; dynamics disabled; no physics; poses as commanded.")
    sim = SimulationContext(sim_utils.SimulationCfg(dt=1/120, device=args.device, use_fabric=False))
    stage = omni.usd.get_context().get_stage()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.)
    ground = sim_utils.CuboidCfg(size=(20., 20., .05),
                                 visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(.20, .225, .255), roughness=.8))
    ground.func("/World/Ground", ground, translation=(0., 0., -.025))
    slab = sim_utils.CuboidCfg(size=tuple(counter["size_m"]),
                               visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(.72, .70, .66), roughness=.5))
    slab.func("/World/Counter", slab, translation=tuple(counter["center_m"]))
    fill = sim_utils.DomeLightCfg(intensity=1200, color=(.92, .95, 1.))
    fill.func("/World/Fill", fill)
    key = sim_utils.DistantLightCfg(intensity=2400, angle=15, color=(1., .97, .93))
    key.func("/World/Key", key, orientation=(.9238795, .3826834, 0, 0))
    rim = sim_utils.DistantLightCfg(intensity=1100, angle=20, color=(.86, .92, 1.))
    rim.func("/World/Rim", rim, orientation=(.7071068, -.5, .5, 0))
    ops = {}

    def set_pose(xf, position, quaternion_xyzw):
        q = np.asarray(quaternion_xyzw, dtype=float)
        q /= np.linalg.norm(q)
        w, x, y, z = xyzw_to_wxyz(q)
        xf[0].Set(Gf.Vec3d(*[float(v) for v in position]))
        xf[1].Set(Gf.Quatd(float(w), Gf.Vec3d(float(x), float(y), float(z))))

    def referenced_body(name, filename, pose, tint=None):
        path = f"/World/Measured/{name}"
        prim = stage.DefinePrim(path, "Xform")
        prim.GetReferences().AddReference(str(filename))
        xf = UsdGeom.Xformable(prim)
        xf.ClearXformOpOrder()
        ops[name] = (xf.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble),
                     xf.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble))
        set_pose(ops[name], pose["position_m"], pose["quaternion_xyzw"])
        material = None
        if tint is not None:
            material = UsdShade.Material.Define(stage, f"/World/ItemMaterials/{name}")
            shader = UsdShade.Shader.Define(stage, material.GetPath().AppendChild("Surface"))
            shader.CreateIdAttr("UsdPreviewSurface")
            shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*tint))
            shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(.36)
            material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
        for child in Usd.PrimRange(prim):
            if child.HasAPI(UsdPhysics.RigidBodyAPI):
                UsdPhysics.RigidBodyAPI(child).CreateRigidBodyEnabledAttr(False)
            if child.HasAPI(UsdPhysics.CollisionAPI):
                UsdPhysics.CollisionAPI(child).CreateCollisionEnabledAttr(False)
            if child.IsA(UsdPhysics.Joint):
                UsdPhysics.Joint(child).CreateJointEnabledAttr(False)
            if material and child.IsA(UsdGeom.Gprim):
                UsdShade.MaterialBindingAPI.Apply(child).Bind(material, bindingStrength=UsdShade.Tokens.strongerThanDescendants)

    for name, filename in COMPONENT_FILES.items():
        referenced_body(name, args.usd.parent / filename, saved[name])
    for o in inst["objects"]:
        referenced_body(o["object_id"], usd_of[o["kind"]], saved[o["object_id"]], tints[o["object_id"]])
    if bench:                                     # the benchmark camera also frames the 1.8 m counter pile
        sys.path.insert(0, str(ROOT / "code/planner/frigidaire"))
        from frigidaire_bench import BENCH_CAMERA
    rig = CameraRig(BENCH_CAMERA if bench else CAMERA, hw=(HEIGHT, WIDTH))
    report["camera"] = BENCH_CAMERA if bench else CAMERA
    sim.reset()
    rig.apply_poses(sim.device)
    for _ in range(args.warmup_renders):          # the path tracer needs a few dozen renders before frames are clean
        sim.render()
        rig.update(sim.get_physics_dt())
    from PIL import Image, ImageDraw
    fonts, _ = caption_fonts()
    size = (args.video_width, args.video_width * HEIGHT // WIDTH)
    bar = size[1] // 12
    font = fonts["detail"].font_variant(size=max(12, bar * 2 // 5))
    video = VideoWriter(str(video_path), fps=args.fps)
    frames_written, stills = 0, {}

    def capture(text, hold, tag=None):
        nonlocal frames_written
        for _ in range(args.renders_per_frame):
            sim.render()
            rig.update(sim.get_physics_dt())
        pixels = rig.grab_one("initial")
        if pixels.std() <= 5 or pixels.max() <= 60:
            raise ValueError("Isaac camera produced an empty frame")
        frame = Image.fromarray(pixels).resize(size, Image.LANCZOS)
        draw = ImageDraw.Draw(frame)
        draw.rectangle((0, 0, size[0], bar), fill=(18, 28, 40))
        draw.text((12, bar // 6), text, fill=(218, 229, 239), font=font)
        arr = np.asarray(frame)
        for _ in range(hold):
            video.add(arr)
            frames_written += 1
        if tag:
            path = args.out_dir / f"{stem}_{tag}.png"
            frame.save(path)
            stills[tag] = path.name

    def respawn(name, pose):
        # Re-spawn the moved prim at its new pose: editing the xform ops of a referenced prim in a
        # running session does not reach the renderer here, re-referencing does (organized renderer).
        stage.RemovePrim(f"/World/Measured/{name}")
        referenced_body(name, usd_of[kinds[name]], pose, tints[name])

    def score_text(k):
        t = trace.get(k)
        return f"   S = {t['S_racked']:.3f} ({t['racked']}/{len(kinds)} racked)" if t else ""

    try:
        n_objects = len(inst["objects"])
        counter_n = sum(o["start"] == "Counter" for o in inst["objects"])
        if bench:
            executed = [mv for mv in episode["moves"] if "poses_after" in mv]
            n = len(executed)
            shown = {oid: saved[oid] for oid in kinds}
            capture(f"{inst['instance_id']}: {counter_n} of {n_objects} dishes on the counter (allowance {inst['counter']['cap']}) · "
                    f"{episode['track']} track · {episode['algorithm']}" + score_text(0), args.hold_frames * 2, tag="start")
            for k, mv in enumerate(executed, 1):
                after = mv["poses_after"]
                for oid, pose in after.items():
                    if np.abs(np.subtract(pose["position_m"], shown[oid]["position_m"])).max() > 1e-3 or \
                            np.abs(np.subtract(pose["quaternion_xyzw"], shown[oid]["quaternion_xyzw"])).max() > 1e-3:
                        respawn(oid, pose)
                        shown[oid] = pose
                T = np.eye(4); T[:3, 3] = after[mv["item_id"]]["position_m"]
                where = ("failed settle, put back" if mv.get("kind") == "failed-settle" else
                         "counter (buffer)" if in_counter_band(T, counter) else "rack")
                capture(f"move {k}/{n}: {mv['item_id'].replace('_', ' ')} -> {where}" + score_text(k),
                        args.hold_frames, tag=("end" if k == n else None))
            ok = episode.get("success")
            capture((f"done: {n} moves, every dish racked" if ok else f"stopped: {episode.get('abort') or episode.get('end_check', {}).get('outcome')}")
                    + (f", S = {s_final:.3f} (S_ref {inst['goal']['S_ref']:.3f})" if s_final is not None else ""), args.hold_frames * 3)
        else:
            n = len(episode["moves"])
            capture(f"{inst['instance_id']}: {n_objects} objects, {counter_n} piled on the counter, "
                    f"{n_objects - counter_n} unorganized inside · {episode['algorithm']}", args.hold_frames * 2, tag="start")
            for k, mv in enumerate(episode["moves"], 1):
                T = np.asarray(mv["T_base_obj"], dtype=float)
                pos, quat = T_to_pos_quat(T)
                respawn(mv["item_id"], {"position_m": pos.tolist(), "quaternion_xyzw": quat.tolist()})
                where = "counter (buffer)" if in_counter_band(T) else mv.get("kind", "goal")
                s = mv.get("score_after")
                text = (f"move {k}/{n}: {kinds[mv['item_id']].replace('_', ' ')} -> {where}"
                        + (f"   S = {s:.3f} ({mv.get('inside_count')} inside)" if s is not None else ""))
                capture(text, args.hold_frames, tag=("end" if k == n else None))
            capture(f"done: {n} moves, every object inside, S = {episode['planned']['score']:.3f}" if episode.get("planned")
                    else f"stopped: {episode.get('abort')}", args.hold_frames * 3)
    finally:
        video.close()
    if video.frames != frames_written or video_path.stat().st_size < 50_000:
        raise ValueError(f"video incomplete: {video.frames} frames, {video_path.stat().st_size} bytes")
    report.update(result="PASS", video_file=video_path.name, video_sha256=digest(video_path), frames=frames_written,
                  fps=args.fps, resolution=list(size), stills=stills)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--instance", type=Path, required=True)
    parser.add_argument("--episode", type=Path, required=True)
    parser.add_argument("--usd", type=Path, default=ROOT / "data/build/frigidaire_collection/usd/fdpc4221as.usdc")
    parser.add_argument("--out-dir", type=Path, required=True)
    parser.add_argument("--fps", type=int, default=24)
    parser.add_argument("--hold-frames", type=int, default=18)
    parser.add_argument("--video-width", type=int, default=1280)
    parser.add_argument("--renders-per-frame", type=int, default=12, help="RTX renders after each re-spawn before the frame is grabbed")
    parser.add_argument("--warmup-renders", type=int, default=32)
    from isaaclab.app import AppLauncher
    AppLauncher.add_app_launcher_args(parser)
    parser.set_defaults(headless=True, device="cpu")
    args = parser.parse_args()
    if not args.enable_cameras:
        raise SystemExit("Rendering requires --enable_cameras")
    args.instance, args.episode, args.usd, args.out_dir = (p.resolve() for p in (args.instance, args.episode, args.usd, args.out_dir))
    args.out_dir.mkdir(parents=True, exist_ok=True)
    report = {"schema_version": 1, "result": "FAIL", "started_utc": datetime.now(timezone.utc).isoformat()}
    app, started = None, time.monotonic()
    track = json.loads(args.episode.read_text()).get("track")
    evidence_path = args.out_dir / f"{args.episode.stem}{'__' + track if track else ''}_video_evidence.json"
    if evidence_path.exists():
        raise FileExistsError(f"Refusing to overwrite: {evidence_path}")
    try:
        app = AppLauncher(args).app
        sys.path[:0] = [str(ROOT / "code/src"), str(ROOT / "code/frigidaire/src"), str(ROOT / "code/util/frigidaire")]
        render(args, report)
    except Exception as exc:
        traceback.print_exc()
        report.update(result="FAIL", error=repr(exc))
    finally:
        report.update(finished_utc=datetime.now(timezone.utc).isoformat(), wall_seconds=time.monotonic() - started)
        evidence_path.write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
        print(f"[RESULT] {report['result']}: {evidence_path}", flush=True)
        if app is not None:
            from dishsim.media import release_sim_for_close
            release_sim_for_close()
            app.close()
    return 0 if report["result"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
