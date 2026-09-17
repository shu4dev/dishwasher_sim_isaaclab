#!/usr/bin/env python3
"""Configure portable glass rendering and record unchanged Frigidaire evidence inputs.

This Kit-free step only edits RTX metadata on the exported loaded scene. Run it
after physics export, and repeat with --require-render-pass after rendering.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import shlex
import sys

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "frigidaire/src"))
from dishsim_frigidaire.paths import ASSET_DIR, IMAGE_DIR
from dishsim_frigidaire.usd_bootstrap import ensure_usd

SETTINGS = {"rtx:raytracing:fractionalCutoutOpacity": True,
            "rtx:translucency:enabled": True}
REFERENCES = [
    "https://docs.omniverse.nvidia.com/materials-and-rendering/latest/rtx-renderer_rt_legacy.html",
    "https://docs.omniverse.nvidia.com/workflows/latest/rtx_rt-dh-setup.html",
]


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def project_path(path):
    resolved = Path(path).resolve()
    try:
        return ROOT / resolved.relative_to(ROOT)
    except ValueError:
        for name in ("assets", "media", "logs", "outputs", "results"):
            logical_root = ROOT / name
            try:
                return logical_root / resolved.relative_to(logical_root.resolve())
            except ValueError:
                continue
        # Outside the project or its storage roots (e.g. a symlink-resolved data
        # drive path): keep the absolute path rather than aborting the run.
        return resolved


def relative(path):
    path = project_path(path)
    try:
        return str(path.relative_to(ROOT))
    except ValueError:
        return str(path)


def scene_content_digest(layer):
    """Canonicalize the scene excluding only the two rendering settings we own."""
    from pxr import Sdf
    copy = Sdf.Layer.CreateAnonymous("frigidaire_render_metadata.usda")
    copy.ImportFromString(layer.ExportToString())
    data = dict(copy.customLayerData)
    rendering = dict(data.get("renderSettings", {}))
    for key in SETTINGS:
        rendering.pop(key, None)
    if rendering:
        data["renderSettings"] = rendering
    else:
        data.pop("renderSettings", None)
    copy.customLayerData = data
    return hashlib.sha256(copy.ExportToString().encode()).hexdigest()


def check_inputs(report, asset_dir, manifest):
    actual_assets = {str(path.relative_to(asset_dir)): digest(path)
                     for path in sorted(asset_dir.rglob("*"))
                     if path.is_file() and path.suffix in (".usd", ".usda", ".usdc")
                     and path.name != "full_load.usda"}
    if actual_assets != report.get("asset_hashes"):
        raise ValueError("Evidence asset hashes do not match the current appliance/tableware USDs")
    sources = report.get("source_hashes")
    if not sources or any(digest(project_path(ROOT / name)) != value for name, value in sources.items()):
        raise ValueError("Evidence executable-source hashes are stale")
    if report.get("manifest_sha256") != digest(manifest):
        raise ValueError("Evidence manifest hash is stale")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--asset-dir", type=Path, default=ASSET_DIR)
    parser.add_argument("--out-dir", type=Path, default=IMAGE_DIR / "full_load")
    parser.add_argument("--manifest", type=Path)
    parser.add_argument("--require-render-pass", action="store_true")
    parser.add_argument("--render-log", type=Path, help="Optional completed render log to fingerprint")
    args = parser.parse_args()
    asset_dir, out_dir = project_path(args.asset_dir), project_path(args.out_dir)
    manifest = project_path(args.manifest or asset_dir / "full_load_manifest.json")
    scene = asset_dir / "full_load.usda"
    physics_path, renders_path = out_dir / "physics.json", out_dir / "renders.json"
    physics_sha = digest(physics_path)
    physics = json.loads(physics_path.read_text())
    if physics.get("result") != "PASS" or not physics.get("completed") or not physics.get("validated_counts"):
        raise ValueError("Rendering configuration requires a completed, validated full-load physics PASS")
    check_inputs(physics, asset_dir, manifest)

    ensure_usd()
    from pxr import Sdf
    layer = Sdf.Layer.FindOrOpen(str(scene))
    if layer is None:
        raise ValueError(f"Cannot open exported loaded scene: {scene}")
    content_before = scene_content_digest(layer)
    data = dict(layer.customLayerData)
    rendering = dict(data.get("renderSettings", {}))
    changed = any(rendering.get(key) is not value for key, value in SETTINGS.items())
    if changed:
        rendering.update(SETTINGS)
        data["renderSettings"] = rendering
        layer.customLayerData = data
        layer.Save()
    content_after = scene_content_digest(layer)
    if content_before != content_after:
        raise RuntimeError("Unexpected scene change outside the owned rendering metadata")
    if digest(physics_path) != physics_sha:
        raise RuntimeError("Physics report changed during rendering configuration")

    renders = json.loads(renders_path.read_text()) if renders_path.is_file() else None
    render_verified = False
    render_note = "No render report exists yet"
    if renders is not None:
        try:
            check_inputs(renders, asset_dir, manifest)
            if renders.get("physics_sha256") != physics_sha:
                raise ValueError("Render report references different physics evidence")
            if renders.get("result") != "PASS":
                raise ValueError("Render report has not passed all image checks")
            if not renders.get("images") or any(
                    not item.get("passed") or digest(project_path(out_dir / item["file"])) != item["sha256"]
                    for item in renders["images"]):
                raise ValueError("Rendered image checks or current image hashes do not match")
            render_verified, render_note = True, "PASS report and all current image hashes verified"
        except (KeyError, ValueError, OSError) as error:
            render_note = str(error)

    runtime_path = out_dir / "render_runtime.json"
    runtime = json.loads(runtime_path.read_text()) if runtime_path.is_file() else None
    live_settings = None
    if render_verified:
        if (runtime is None or runtime.get("result") != "PASS"
                or runtime.get("physics_sha256") != physics_sha
                or runtime.get("renders_sha256") != digest(renders_path)
                or not runtime.get("physics_unchanged") or not runtime.get("render_calls")):
            render_verified, render_note = False, "Matching successful live renderer-settings provenance is required"
        elif any(digest(project_path(ROOT / name)) != value for name, value in runtime["source_hashes"].items()):
            render_verified, render_note = False, "Render wrapper source hash is stale"
        else:
            observations = runtime.get("setting_observations", [])
            expected = {"/" + key.replace(":", "/"): value for key, value in SETTINGS.items()}
            if not observations or any(any(item.get("after", {}).get(key) is not value
                                           for key, value in expected.items()) for item in observations):
                render_verified, render_note = False, "Live renderer transparency readback did not pass"
            else:
                live_settings = observations[-1]["after"]

    render_argv = ["scripts/run_kit.sh", "frigidaire/scripts/evaluation/frigidaire_full_load_render.py",
                   "--headless", "--device", "cpu", "--enable_cameras", "--render-only",
                   "--rendering_mode", "quality",
                   "--kit_args=--/rtx/raytracing/fractionalCutoutOpacity=true",
                   "--usd", relative(asset_dir / "fdpc4221as.usdc"),
                   "--manifest", relative(manifest), "--out_dir", relative(out_dir)]
    report = {
        "schema_version": 1, "recorded_at_utc": datetime.now(timezone.utc).isoformat(),
        "result": "PASS" if render_verified else "CONFIGURED_AWAITING_RENDER_PASS",
        "scope": "Renderer configuration and evidence provenance; existing physics results are unchanged",
        "settings": SETTINGS, "setting_storage": "USD rootLayer.customLayerData.renderSettings",
        "scene": relative(scene), "scene_sha256": digest(scene),
        "scene_content_without_owned_render_settings_sha256": content_after,
        "physics": relative(physics_path), "physics_sha256": physics_sha,
        "renders": relative(renders_path) if renders else None,
        "renders_sha256": digest(renders_path) if renders else None,
        "render_inputs_and_images_verified": render_verified, "render_verification_note": render_note,
        "render_command": shlex.join(render_argv),
        "runtime_setting_readback": live_settings,
        "runtime_setting_note": "Live values come from the separately hashed render-only wrapper report",
        "render_runtime": relative(runtime_path) if runtime else None,
        "render_runtime_sha256": digest(runtime_path) if runtime else None,
        "asset_hashes": physics["asset_hashes"],
        "source_hashes": {**physics["source_hashes"], **(runtime.get("source_hashes", {}) if runtime else {}), **{relative(path): digest(path) for path in
            (Path(__file__), ROOT / "frigidaire/src/dishsim_frigidaire/usd_bootstrap.py")}},
        "input_sha256": {str(scene.relative_to(asset_dir)): digest(scene)},
        "manifest_sha256": physics["manifest_sha256"],
        "references": REFERENCES,
    }
    if args.render_log:
        log = project_path(args.render_log)
        report["render_log"] = {"file": relative(log), "sha256": digest(log)}
    destination = out_dir / "render_settings.json"
    temporary = destination.with_suffix(".json.tmp")
    temporary.write_text(json.dumps(report, indent=2) + "\n")
    temporary.replace(destination)
    print(json.dumps({"result": report["result"], "scene_metadata_changed": changed,
                      "physics_sha256": physics_sha, "report": relative(destination)}, indent=2))
    if args.require_render_pass and not render_verified:
        raise SystemExit(f"Render evidence is not ready: {render_note}")


if __name__ == "__main__":
    main()
