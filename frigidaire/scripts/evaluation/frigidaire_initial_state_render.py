#!/usr/bin/env python3
"""Render one measured accepted initial state with Isaac RTX, without advancing physics.

    scripts/run_kit.sh frigidaire/scripts/evaluation/frigidaire_initial_state_render.py \
        --state <run>/states/highest.json --out-dir <run>/renders --headless --enable_cameras

One 1920x1440 image and its saved-versus-rendered transform audit are emitted.
The scene replays current USD component geometry at measured world poses; it
does not run a new physical validation or alter the saved experiment verdict.
"""
from __future__ import annotations

import argparse
import colorsys
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import re
import sys
import time
import traceback


ROOT = Path(__file__).resolve().parents[3]
WIDTH, HEIGHT = 1920, 1440
CAMERA = {"initial": ((1.6, -2.1, 1.65), (0., -.34, .46),
                       {"focal_length": 44., "horizontal_aperture": 36.})}


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def item_tints(objects):
    """Distinct deterministic colors for every independent item, independent of class."""
    return {identity: list(colorsys.hsv_to_rgb((.07+index*.6180339887498949) % 1., .55, .90))
            for index, identity in enumerate(sorted(obj["object_id"] for obj in objects))}


def label_for(state):
    return "Highest validated load found" if state["purpose"] == "highest" else "Randomized validated initial state"


def caption_fonts():
    """Use the same full-size fonts on hosts and minimal Isaac containers."""
    from PIL import ImageFont
    import matplotlib

    directory = Path(matplotlib.get_data_path()) / "fonts" / "ttf"
    specifications = {"title": ("DejaVuSans-Bold.ttf", 42), "detail": ("DejaVuSans.ttf", 27)}
    fonts, evidence = {}, {}
    for label, (filename, size) in specifications.items():
        path = directory / filename
        fonts[label] = ImageFont.truetype(str(path), size)
        evidence[label] = {"path": str(path), "sha256": digest(path), "size_px": size}
    return fonts, evidence


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
    from frigidaire_initial_state_report import validate_state, pose_error

    state = json.loads(args.state.read_text())
    counts = validate_state(state)
    source_attempt = state.get("source_attempt")
    if not isinstance(source_attempt, str) or not re.fullmatch(r"[A-Za-z_][A-Za-z0-9_]*", source_attempt):
        raise ValueError("State must reference a saved validation attempt")
    source_result = args.state.parent.parent / "attempts" / source_attempt / "result.json"
    if json.loads(source_result.read_text()) != state["validation"]:
        raise ValueError("State validation differs from its saved source attempt")
    identity = state.get("state_id")
    if not isinstance(identity, str) or not re.fullmatch(r"[A-Za-z_][A-Za-z0-9_]*", identity):
        raise ValueError("State ID must be a safe USD/file identifier")
    for obj in state["objects"]:
        if not re.fullmatch(r"[A-Za-z_][A-Za-z0-9_]*", obj["object_id"]):
            raise ValueError("Object ID must be a valid USD identifier")
    output = args.out_dir / f"{identity}_initial.png"
    if output.exists():
        raise FileExistsError(f"Refusing to overwrite image: {output}")
    assets = dict(state.get("input_hashes", {}).get("asset_hashes", {}))
    summary_path = args.state.parent.parent / "summary.json"
    if not assets and summary_path.is_file():
        assets = json.loads(summary_path.read_text()).get("input_hashes", {}).get("asset_hashes", {})
    required = [args.usd.name, *COMPONENT_FILES.values(), *(f"tableware/{kind}.usdc" for kind in {obj["kind"] for obj in state["objects"]})]
    for relative in required:
        path = args.usd.parent / relative
        if not path.is_file() or digest(path) != assets.get(relative):
            raise ValueError(f"Missing or changed experiment USD: {relative}")
    report.update(state_id=identity, state_sha256=digest(args.state), state_path=str(args.state),
                  asset_hashes={name: assets[name] for name in required}, counts=counts,
                  label=label_for(state), initial_joints=state["initial_snapshot"]["joints"],
                  camera=CAMERA, source_hashes={str(path.relative_to(ROOT)): digest(path) for path in
                      (Path(__file__), Path(__file__).with_name("frigidaire_initial_state_report.py"))},
                  validation_sha256=digest(source_result),
                  runtime={"isaac_sim": Path("/isaac-sim/VERSION").read_text().strip()},
                  scope="Isaac RTX rendering of saved measured initial poses; dynamics disabled in this render scene only. "
                        "No new physics validation, geometry scaling, pose settling, or hidden dishes.")
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
    saved = state["initial_snapshot"]["poses"]
    tints = item_tints(state["objects"])
    roots = {}

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
        roots[name] = prim

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
    cache = UsdGeom.XformCache()
    errors, rendered = {}, {}
    for name, prim in roots.items():
        matrix = cache.GetLocalToWorldTransform(prim)
        quaternion = matrix.ExtractRotationQuat()
        pose = {"position_m": list(matrix.ExtractTranslation()),
                "quaternion_xyzw": [*quaternion.GetImaginary(), quaternion.GetReal()]}
        rendered[name] = pose
        errors[name] = pose_error(saved[name], pose)
    if any(value["position_error_m"] > 1e-7 or value["orientation_error_deg"] > 1e-4 for value in errors.values()):
        raise ValueError("Rendered component/object transforms differ from saved measured poses")
    from PIL import Image, ImageDraw
    picture = Image.fromarray(pixels)
    draw = ImageDraw.Draw(picture)
    draw.rectangle((0, 0, WIDTH, 160), fill=(18, 28, 40))
    fonts, font_evidence = caption_fonts()
    title_font, detail_font = fonts["title"], fonts["detail"]
    draw.text((34, 18), report["label"], fill=(246, 249, 252), font=title_font)
    by_kind = counts["by_kind"]
    detail = (f"{counts['total']} dishes  |  {by_kind.get('dinner_plate', 0)} plates, {by_kind.get('bowl', 0)} bowls, "
              f"{by_kind.get('mug', 0)} mugs  |  Lower: {counts['by_rack'].get('LowerRack', 0)}  "
              f"Upper: {counts['by_rack'].get('UpperRack', 0)}")
    draw.text((36, 80), detail, fill=(218, 229, 239), font=detail_font)
    draw.text((36, 118), "Measured loaded initial state · both racks extended · door open · Isaac RTX", fill=(190, 207, 222), font=detail_font)
    picture.save(output)
    report.update(result="PASS", image_file=output.name, image_sha256=digest(output), resolution=[WIDTH, HEIGHT],
                  caption_fonts=font_evidence,
                  rgb_std=float(pixels.std()), tints=tints, rendered_poses=rendered, transform_errors=errors,
                  transform_tolerances={"position_m": 1e-7, "orientation_deg": 1e-4},
                  max_position_error_m=max(v["position_error_m"] for v in errors.values()),
                  max_orientation_error_deg=max(v["orientation_error_deg"] for v in errors.values()),
                  state_unchanged=digest(args.state) == report["state_sha256"])
    if not report["state_unchanged"]:
        raise ValueError("Source state changed during render")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--state", type=Path, required=True)
    parser.add_argument("--usd", type=Path, default=ROOT / "build/frigidaire_collection/usd/fdpc4221as.usdc")
    parser.add_argument("--out-dir", type=Path, required=True)
    from isaaclab.app import AppLauncher
    AppLauncher.add_app_launcher_args(parser)
    parser.set_defaults(headless=True, device="cpu")
    args = parser.parse_args()
    if not args.enable_cameras:
        raise SystemExit("Rendering requires --enable_cameras")
    args.state, args.usd, args.out_dir = args.state.resolve(), args.usd.resolve(), args.out_dir.resolve()
    args.out_dir.mkdir(parents=True, exist_ok=True)
    report = {"schema_version": 1, "result": "FAIL", "started_utc": datetime.now(timezone.utc).isoformat()}
    app, started, identity = None, time.monotonic(), args.state.stem
    evidence_path = args.out_dir / f"{identity}_render_evidence.json"
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
        report.update(finished_utc=datetime.now(timezone.utc).isoformat(), wall_seconds=time.monotonic()-started)
        evidence_path.write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
        print(f"[RESULT] {report['result']}: {evidence_path}", flush=True)
        if app is not None:
            from dishsim.media import release_sim_for_close
            release_sim_for_close()
            app.close()
    return 0 if report["result"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
