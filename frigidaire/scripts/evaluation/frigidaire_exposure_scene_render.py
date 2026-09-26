#!/usr/bin/env python3
"""Render the exposure demo scene (a settled load plus a plate on edge and cutlery) with Isaac RTX.

    scripts/run_kit.sh frigidaire/scripts/evaluation/frigidaire_exposure_scene_render.py \
        --state results/exposure/frigidaire/scene/random_06_plus.json \
        --out-dir results/exposure/frigidaire/scene --headless --enable_cameras

Same scene, lights, camera and per-item tints as frigidaire_initial_state_render.py, applied
to the geometric demo file written by frigidaire_exposure_scene.py: no experiment validation,
no physics step (dynamics are disabled on every prim), the added objects are posed, not
settled. Writes <state_id>_initial.png (1920x1440) and <state_id>_render_evidence.json; with
--orbit-seconds N also <state_id>_orbit.mp4, the camera sweeping across the front of the machine
(high front view down into the racks, round to each side and back; loops), no physics either.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import sys
import time
import traceback

ROOT = Path(__file__).resolve().parents[3]
ORBIT_YAW_DEG, ORBIT_YAW_SWING_DEG = -90., 70.      # front of the machine is -y; sweep +-70 deg about it
ORBIT_ELEV_DEG, ORBIT_ELEV_SWING_DEG = 32., 16.     # 48 deg (down into the racks) to 16 deg (low front)
ORBIT_DISTANCE_SCALE = .82                          # closer than the still so the machine fills the video frame


def orbit_eye(k, n, target, distance):
    """Camera position for frame k of n: one closed loop across the front, smooth at the seam."""
    u = 2 * math.pi * k / n
    yaw = math.radians(ORBIT_YAW_DEG + ORBIT_YAW_SWING_DEG * math.sin(u))
    elev = math.radians(ORBIT_ELEV_DEG + ORBIT_ELEV_SWING_DEG * math.cos(u))
    return (target[0] + distance * math.cos(elev) * math.cos(yaw),
            target[1] + distance * math.cos(elev) * math.sin(yaw),
            target[2] + distance * math.sin(elev))


def render(args, report):
    # All project/scene/USD imports happen after AppLauncher initialized Kit.
    import numpy as np
    import omni.usd
    import isaaclab.sim as sim_utils
    from isaaclab.sim import SimulationContext
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade
    from dishsim.media import CameraRig
    from dishsim.quats import xyzw_to_wxyz
    from dishsim_frigidaire.asset import COMPONENT_FILES
    from frigidaire_initial_state_render import CAMERA, WIDTH, HEIGHT, item_tints, caption_fonts, digest

    state = json.loads(args.state.read_text())
    if state.get("purpose") != "exposure_demo":
        raise ValueError("Expected a demo scene written by frigidaire_exposure_scene.py")
    identity = state["state_id"]
    output = args.out_dir / f"{identity}_initial.png"
    if output.exists():
        raise FileExistsError(f"Refusing to overwrite image: {output}")
    saved = state["initial_snapshot"]["poses"]
    tints = item_tints(state["objects"])
    counts = state["counts"]
    report.update(state_id=identity, state_sha256=digest(args.state), state_path=str(args.state), counts=counts,
                  added=state.get("demo_added", []), camera=CAMERA,
                  runtime={"isaac_sim": Path("/isaac-sim/VERSION").read_text().strip()},
                  scope="Isaac RTX rendering of a geometric demo scene; dynamics disabled in this render scene; "
                        "the added objects are posed, not settled; no physics validation.")
    sim = SimulationContext(sim_utils.SimulationCfg(dt=1/120, device=args.device, use_fabric=False))
    stage = omni.usd.get_context().get_stage()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.)
    ground = sim_utils.CuboidCfg(size=(20., 20., .05),
                                 visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(.20, .225, .255), roughness=.8))
    ground.func("/World/Ground", ground, translation=(0., 0., -.025))
    fill = sim_utils.DomeLightCfg(intensity=1200, color=(.92, .95, 1.))
    fill.func("/World/Fill", fill)
    key = sim_utils.DistantLightCfg(intensity=2400, angle=15, color=(1., .97, .93))
    key.func("/World/Key", key, orientation=(.9238795, .3826834, 0, 0))
    rim = sim_utils.DistantLightCfg(intensity=1100, angle=20, color=(.86, .92, 1.))
    rim.func("/World/Rim", rim, orientation=(.7071068, -.5, .5, 0))

    def referenced_body(name, filename, pose, tint=None):
        path = f"/World/Measured/{name}"
        prim = stage.DefinePrim(path, "Xform")
        prim.GetReferences().AddReference(str(filename))
        xf = UsdGeom.Xformable(prim)
        xf.ClearXformOpOrder()
        xf.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*pose["position_m"]))
        q = np.asarray(pose["quaternion_xyzw"], dtype=float)
        q /= np.linalg.norm(q)
        w, x, y, z = xyzw_to_wxyz(q)
        xf.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(float(w), Gf.Vec3d(float(x), float(y), float(z))))
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
    for obj in state["objects"]:
        name = obj["object_id"]
        referenced_body(name, args.usd.parent / "tableware" / f"{obj['kind']}.usdc", saved[name], tints[name])
    rig = CameraRig(CAMERA, hw=(HEIGHT, WIDTH))
    sim.reset()
    rig.apply_poses(sim.device)
    for _ in range(16):
        sim.render()
        rig.update(sim.get_physics_dt())
    pixels = rig.grab_one("initial")
    if pixels.shape != (HEIGHT, WIDTH, 3) or float(pixels.std()) <= 5 or int(pixels.max()) <= 60:
        raise ValueError("Isaac camera produced an empty or malformed RGB frame")
    from PIL import Image, ImageDraw
    picture = Image.fromarray(pixels)
    draw = ImageDraw.Draw(picture)
    draw.rectangle((0, 0, WIDTH, 160), fill=(18, 28, 40))
    fonts, font_evidence = caption_fonts()
    by_kind, by_rack = counts["by_kind"], counts["by_rack"]
    draw.text((34, 18), "Exposure demo scene: organized load + plate on edge + cutlery in the basket",
              fill=(246, 249, 252), font=fonts["title"])
    detail = (f"{counts['total']} items  |  " + ", ".join(f"{n} {k.replace('_', ' ')}" for k, n in by_kind.items())
              + f"  |  Lower: {by_rack.get('LowerRack', 0)}  Upper: {by_rack.get('UpperRack', 0)}  "
                f"Basket: {by_rack.get('SilverwareBasket', 0)}")
    draw.text((36, 80), detail, fill=(218, 229, 239), font=fonts["detail"])
    draw.text((36, 118), "Both racks extended · door open · Isaac RTX · added plate and cutlery are geometric poses, not settled",
              fill=(190, 207, 222), font=fonts["detail"])
    picture.save(output)
    report.update(image_file=output.name, image_sha256=digest(output), resolution=[WIDTH, HEIGHT],
                  caption_fonts=font_evidence, rgb_std=float(pixels.std()), tints=tints)
    if args.orbit_seconds > 0:
        import torch
        from dishsim.media import VideoWriter
        eye0, target = CAMERA["initial"][0], CAMERA["initial"][1]
        distance = ORBIT_DISTANCE_SCALE * math.dist(eye0, target)
        video_path = args.out_dir / f"{identity}_orbit.mp4"
        if video_path.exists():
            raise FileExistsError(f"Refusing to overwrite video: {video_path}")
        n, size = int(round(args.orbit_seconds * args.fps)), (args.video_width, args.video_width * HEIGHT // WIDTH)
        cam, targets = rig.cams["initial"], torch.tensor([target], dtype=torch.float32, device=sim.device)
        video = VideoWriter(str(video_path), fps=args.fps)
        bar = size[1] // 16
        try:
            for k in range(n):
                cam.set_world_poses_from_view(
                    eyes=torch.tensor([orbit_eye(k, n, target, distance)], dtype=torch.float32, device=sim.device),
                    targets=targets)
                for _ in range(args.renders_per_frame):
                    sim.render()
                    rig.update(sim.get_physics_dt())
                frame = Image.fromarray(rig.grab_one("initial")).resize(size, Image.LANCZOS)
                draw = ImageDraw.Draw(frame)
                draw.rectangle((0, 0, size[0], bar), fill=(18, 28, 40))
                draw.text((12, bar // 5), "Exposure demo scene · Isaac RTX · plate and cutlery posed, not settled",
                          fill=(218, 229, 239), font=fonts["detail"].font_variant(size=max(12, bar // 2)))
                video.add(np.asarray(frame))
                if k % (2 * args.fps) == 0:
                    print(f"[INFO] orbit frame {k}/{n}", flush=True)
        finally:
            video.close()
        if video.frames != n or video_path.stat().st_size < 100_000:
            raise ValueError(f"Orbit video incomplete: {video.frames}/{n} frames, {video_path.stat().st_size} bytes")
        report.update(video_file=video_path.name, video_sha256=digest(video_path), video_frames=n, video_fps=args.fps,
                      video_resolution=list(size), orbit={"seconds": args.orbit_seconds, "distance_m": distance,
                      "yaw_deg": [ORBIT_YAW_DEG, ORBIT_YAW_SWING_DEG], "elevation_deg": [ORBIT_ELEV_DEG, ORBIT_ELEV_SWING_DEG],
                      "renders_per_frame": args.renders_per_frame})
    report.update(result="PASS")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--state", type=Path, required=True)
    parser.add_argument("--usd", type=Path, default=ROOT / "build/frigidaire_collection/usd/fdpc4221as.usdc")
    parser.add_argument("--out-dir", type=Path, required=True)
    parser.add_argument("--orbit-seconds", type=float, default=0., help="also write <state_id>_orbit.mp4 of this length")
    parser.add_argument("--fps", type=int, default=24)
    parser.add_argument("--video-width", type=int, default=1280, help="video width; height keeps the 4:3 still aspect")
    parser.add_argument("--renders-per-frame", type=int, default=3, help="RTX renders per video frame after the camera moves")
    from isaaclab.app import AppLauncher
    AppLauncher.add_app_launcher_args(parser)
    parser.set_defaults(headless=True, device="cpu")
    args = parser.parse_args()
    if not args.enable_cameras:
        raise SystemExit("Rendering requires --enable_cameras")
    args.state, args.usd, args.out_dir = args.state.resolve(), args.usd.resolve(), args.out_dir.resolve()
    args.out_dir.mkdir(parents=True, exist_ok=True)
    report = {"schema_version": 1, "result": "FAIL", "started_utc": datetime.now(timezone.utc).isoformat()}
    app, started = None, time.monotonic()
    evidence_path = args.out_dir / f"{args.state.stem}_render_evidence.json"
    if evidence_path.exists():
        raise FileExistsError(f"Refusing to overwrite render evidence: {evidence_path}")
    try:
        app = AppLauncher(args).app
        sys.path[:0] = [str(ROOT / "src"), str(ROOT / "frigidaire/src"), str(Path(__file__).parent)]
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
