#!/usr/bin/env python3
"""Render original and organized measured states in one Isaac RTX session.

    scripts/run_kit.sh frigidaire/scripts/evaluation/frigidaire_organized_render.py \
        --batch RUN/summary.json --out-dir RUN/renders --headless --enable_cameras

Images are static replays of saved measurements. Physics is never advanced here.
An original packing may fail organization; only accepted counterparts get after views.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import re
import sys
import time
import traceback

ROOT = Path(__file__).resolve().parents[3]
WIDTH, HEIGHT = 1920, 1440


def build_jobs(args):
    from frigidaire_organized_report import digest, local_path, portable_path, read_json, validate_organized_state, validate_source_state

    jobs = []
    if args.batch:
        directory = args.batch.resolve().parent
        summary = read_json(args.batch)
        selected = set(args.state_ids.split(",")) if args.state_ids else None
        for row in summary["states"]:
            identity = row["source_state_id"]
            if selected is not None and identity not in selected:
                continue
            source_path = portable_path(row["source_state"], directory)
            source = read_json(source_path)
            validate_source_state(source)
            if digest(source_path) != row.get("source_state_sha256"):
                raise ValueError("Original state differs from its recorded source hash")
            jobs.append({"source_state_id": identity, "view": "before", "path": source_path, "state": source,
                         "summary": summary, "source_path": source_path})
            if row.get("status") == "accepted":
                path = local_path(directory, row["accepted_state"])
                state = read_json(path)
                validate_organized_state(state, source)
                if state.get("source_state_sha256") != digest(source_path):
                    raise ValueError("Counterpart does not hash its unchanged original")
                for attempt, validation in ((state["source_attempt"], state["validation"]),
                                            (state["reproduction"]["source_attempt"], state["reproduction"]["validation"])):
                    if read_json(local_path(directory, f"attempts/{attempt}/result.json")) != validation:
                        raise ValueError("Counterpart validation differs from its saved attempt")
                if state["source_attempt"] == state["reproduction"]["source_attempt"]:
                    raise ValueError("Counterpart requires a separate fresh reproduction attempt")
                jobs.append({"source_state_id": identity, "view": "after", "path": path, "state": state,
                             "summary": summary, "source_path": source_path})
    else:
        path = args.state.resolve()
        state = read_json(path)
        source_path = args.source_state.resolve() if args.source_state else path
        source = read_json(source_path)
        if args.view == "after":
            if source_path == path:
                raise ValueError("An after view requires --source-state for inventory validation")
            validate_organized_state(state, source)
            if state.get("source_state_sha256") != digest(source_path):
                raise ValueError("Counterpart does not hash its unchanged original")
            for attempt, validation in ((state["source_attempt"], state["validation"]),
                                        (state["reproduction"]["source_attempt"], state["reproduction"]["validation"])):
                if read_json(local_path(path.parent.parent, f"attempts/{attempt}/result.json")) != validation:
                    raise ValueError("Counterpart validation differs from its saved attempt")
            if state["source_attempt"] == state["reproduction"]["source_attempt"]:
                raise ValueError("Counterpart requires a separate fresh reproduction attempt")
        else:
            validate_source_state(state)
        summary_path = path.parent.parent / "summary.json"
        jobs.append({"source_state_id": source["state_id"], "view": args.view, "path": path, "state": state,
                     "summary": read_json(summary_path) if summary_path.is_file() else {}, "source_path": source_path})
    keys = [(job["source_state_id"], job["view"]) for job in jobs]
    if len(set(keys)) != len(keys) or not jobs:
        raise ValueError("Render jobs must be nonempty and unique")
    for job in jobs:
        if not re.fullmatch(r"[A-Za-z_][A-Za-z0-9_]*", job["source_state_id"]):
            raise ValueError("Source state ID must be a safe filename")
    return jobs


def label_for(job):
    label = "Original packing" if job["view"] == "before" else "Organized counterpart"
    return f"{label} · {job['source_state_id']}"


def render_batch(args, jobs):
    # Kit must be running before these imports.
    import numpy as np
    import omni.usd
    import isaaclab.sim as sim_utils
    from isaaclab.sim import SimulationContext
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade
    from dishsim.media import CameraRig
    from dishsim.quats import xyzw_to_wxyz
    from dishsim_frigidaire.asset import COMPONENT_FILES
    from frigidaire_initial_state_render import CAMERA, caption_fonts, item_tints
    from frigidaire_organized_report import digest, pose_error, state_counts
    from PIL import Image, ImageDraw

    sim = SimulationContext(sim_utils.SimulationCfg(dt=1/120, device=args.device, use_fabric=False))
    stage = omni.usd.get_context().get_stage()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.)
    ground = sim_utils.CuboidCfg(size=(20., 20., .05), visual_material=sim_utils.PreviewSurfaceCfg(
        diffuse_color=(.20, .225, .255), roughness=.8))
    ground.func("/World/Ground", ground, translation=(0., 0., -.025))
    fill = sim_utils.DomeLightCfg(intensity=1200, color=(.92, .95, 1.))
    fill.func("/World/Fill", fill)
    key = sim_utils.DistantLightCfg(intensity=2400, angle=15, color=(1., .97, .93))
    key.func("/World/Key", key, orientation=(.9238795, .3826834, 0, 0))
    rim = sim_utils.DistantLightCfg(intensity=1100, angle=20, color=(.86, .92, 1.))
    rim.func("/World/Rim", rim, orientation=(.7071068, -.5, .5, 0))
    rig = CameraRig(CAMERA, hw=(HEIGHT, WIDTH))
    sim.reset()
    rig.apply_poses(sim.device)
    fonts, font_evidence = caption_fonts()
    written = []
    for job in jobs:
        started = time.monotonic()
        identity = f"{job['source_state_id']}_{job['view']}"
        evidence_path = args.out_dir / f"{identity}_render_evidence.json"
        output = args.out_dir / f"{identity}.png"
        state, path = job["state"], job["path"]
        state_hash = digest(path)
        if output.exists() or evidence_path.exists():
            raise FileExistsError(f"Refusing to overwrite render evidence: {identity}")
        record = {"schema_version": 1, "result": "FAIL", "source_state_id": job["source_state_id"], "view": job["view"],
                  "state_path": str(path), "state_sha256": state_hash, "source_state_path": str(job["source_path"]),
                  "source_state_sha256": digest(job["source_path"]), "started_utc": datetime.now(timezone.utc).isoformat()}
        try:
            assets = state.get("input_hashes", {}).get("asset_hashes", {}) or job["summary"].get("input_hashes", {}).get("asset_hashes", {})
            if not assets:
                source_summary = job["source_path"].parent.parent / "summary.json"
                assets = json.loads(source_summary.read_text()).get("input_hashes", {}).get("asset_hashes", {})
            required = [args.usd.name, *COMPONENT_FILES.values(), *(f"tableware/{kind}.usdc" for kind in {obj["kind"] for obj in state["objects"]})]
            for relative in required:
                if digest(args.usd.parent / relative) != assets.get(relative):
                    raise ValueError(f"Missing or changed experiment USD: {relative}")
            stage.RemovePrim("/World/Measured")
            stage.RemovePrim("/World/ItemMaterials")
            saved = state["initial_snapshot"]["poses"]
            tints = item_tints(state["objects"])
            roots = {}

            def body(name, filename, pose, tint=None):
                if not re.fullmatch(r"[A-Za-z_][A-Za-z0-9_]*", name):
                    raise ValueError("Object IDs must be safe USD identifiers")
                prim = stage.DefinePrim(f"/World/Measured/{name}", "Xform")
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
                body(name, args.usd.parent / filename, saved[name])
            for obj in state["objects"]:
                body(obj["object_id"], args.usd.parent / "tableware" / f"{obj['kind']}.usdc", saved[obj["object_id"]], tints[obj["object_id"]])
            for _ in range(16):
                sim.render()
                rig.update(sim.get_physics_dt())
            pixels = rig.grab_one("initial")
            if pixels.shape != (HEIGHT, WIDTH, 3) or float(pixels.std()) <= 5 or int(pixels.max()) <= 60:
                raise ValueError("Isaac camera produced an empty or malformed RGB frame")
            cache, rendered, errors = UsdGeom.XformCache(), {}, {}
            for name, prim in roots.items():
                matrix = cache.GetLocalToWorldTransform(prim)
                q = matrix.ExtractRotationQuat()
                rendered[name] = {"position_m": list(matrix.ExtractTranslation()), "quaternion_xyzw": [*q.GetImaginary(), q.GetReal()]}
                errors[name] = pose_error(saved[name], rendered[name])
            if any(error["position_error_m"] > 1e-7 or error["orientation_error_deg"] > 1e-4 for error in errors.values()):
                raise ValueError("Rendered transforms differ from saved measured poses")
            counts = state_counts(state)
            picture = Image.fromarray(pixels)
            draw = ImageDraw.Draw(picture)
            draw.rectangle((0, 0, WIDTH, 160), fill=(18, 28, 40))
            draw.text((34, 18), label_for(job), fill=(246, 249, 252), font=fonts["title"])
            detail = (f"{counts['total']} dishes  |  {counts['by_kind'].get('dinner_plate', 0)} plates, "
                      f"{counts['by_kind'].get('bowl', 0)} bowls, {counts['by_kind'].get('mug', 0)} mugs  |  "
                      f"Lower: {counts['by_rack'].get('LowerRack', 0)}  Upper: {counts['by_rack'].get('UpperRack', 0)}")
            draw.text((36, 80), detail, fill=(218, 229, 239), font=fonts["detail"])
            note = "Original packing; organization assessed separately" if job["view"] == "before" else "Accepted organization and door cycle; exact source inventory"
            draw.text((36, 118), note + " · Isaac RTX", fill=(190, 207, 222), font=fonts["detail"])
            picture.save(output)
            unchanged = digest(path) == state_hash
            if not unchanged:
                raise ValueError("Source state changed during render")
            record.update(result="PASS", image_file=output.name, image_sha256=digest(output), label=label_for(job), counts=counts,
                          asset_hashes={name: assets[name] for name in required}, resolution=[WIDTH, HEIGHT], camera=CAMERA,
                          tints=tints, caption_fonts=font_evidence, rgb_std=float(pixels.std()), rendered_poses=rendered,
                          transform_errors=errors, state_unchanged=True,
                          max_position_error_m=max(value["position_error_m"] for value in errors.values()),
                          max_orientation_error_deg=max(value["orientation_error_deg"] for value in errors.values()),
                          runtime={"isaac_sim": Path("/isaac-sim/VERSION").read_text().strip()},
                          source_hashes={str(source.relative_to(ROOT)): digest(source) for source in (
                              Path(__file__), Path(__file__).with_name("frigidaire_organized_report.py"),
                              Path(__file__).with_name("frigidaire_initial_state_render.py"),
                              Path(__file__).with_name("frigidaire_initial_state_report.py"))},
                          scope="Measured initial pose replay in a static Isaac RTX scene; no physics advancement or new physical verdict.")
        except Exception as exc:
            traceback.print_exc()
            record.update(result="FAIL", error=repr(exc))
        finally:
            record.update(finished_utc=datetime.now(timezone.utc).isoformat(), wall_seconds=time.monotonic()-started)
            evidence_path.write_text(json.dumps(record, indent=2, allow_nan=False) + "\n")
            print(f"[RENDER] {record['result']}: {identity}", flush=True)
            written.append(record)
    return written


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--batch", type=Path, help="Organized experiment summary.json")
    source.add_argument("--state", type=Path)
    parser.add_argument("--source-state", type=Path)
    parser.add_argument("--view", choices=("before", "after"), default="after")
    parser.add_argument("--state-ids", help="Comma-separated source IDs to render from a batch")
    parser.add_argument("--usd", type=Path, default=ROOT / "build/frigidaire_collection/usd/fdpc4221as.usdc")
    parser.add_argument("--out-dir", type=Path, required=True)
    from isaaclab.app import AppLauncher
    AppLauncher.add_app_launcher_args(parser)
    parser.set_defaults(headless=True, device="cpu")
    args = parser.parse_args()
    if not args.enable_cameras:
        raise SystemExit("Rendering requires --enable_cameras")
    args.out_dir, args.usd = args.out_dir.resolve(), args.usd.resolve()
    args.out_dir.mkdir(parents=True, exist_ok=True)
    app, records = None, []
    try:
        app = AppLauncher(args).app
        sys.path[:0] = [str(ROOT / "src"), str(ROOT / "frigidaire/src"), str(Path(__file__).parent)]
        records = render_batch(args, build_jobs(args))
    except Exception:
        traceback.print_exc()
    finally:
        passed = bool(records) and all(record["result"] == "PASS" for record in records)
        print(f"[RESULT] {'PASS' if passed else 'FAIL'}: organized render batch ({len(records)} images)", flush=True)
        if app is not None:
            from dishsim.media import release_sim_for_close
            release_sim_for_close()
            app.close()
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
