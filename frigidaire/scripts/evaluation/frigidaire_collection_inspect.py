#!/usr/bin/env python3
"""Inspect current USD composition and portability without starting Kit."""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import shutil
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "frigidaire/src"))
from dishsim_frigidaire.paths import COLLECTION_DIR


def inspect_collection(collection):
    from pxr import Usd, UsdGeom, UsdUtils
    from dishsim_frigidaire import asset, geometry
    import numpy as np

    collection = Path(collection).absolute()
    directory = collection / "usd"
    files = sorted(p for p in directory.rglob("*") if p.suffix in (".usd", ".usda", ".usdc"))
    required = [directory / name for name in ("fdpc4221as.usdc", "example_scene.usda", *asset.COMPONENT_FILES.values())]
    if not all(p.is_file() for p in required):
        raise FileNotFoundError("Build the current complete appliance into %s first" % directory)
    checks = {name: True for name in ("dependencies_resolve", "relative_references", "stage_conventions",
              "components_present", "polished_racks", "basket_position", "relocated_dependencies_resolve")}
    details = {}
    for path in files:
        stage = Usd.Stage.Open(str(path))
        layers, _, unresolved = UsdUtils.ComputeAllDependencies(str(path))
        absolute = [str(ref) for layer in layers for ref in layer.GetExternalReferences()
                    if Path(str(ref)).is_absolute() or "://" in str(ref)]
        checks["dependencies_resolve"] &= not bool(unresolved)
        checks["relative_references"] &= not bool(absolute)
        checks["stage_conventions"] &= bool(stage and stage.GetDefaultPrim()
            and UsdGeom.GetStageMetersPerUnit(stage) == 1. and UsdGeom.GetStageUpAxis(stage) == "Z")
        details[str(path.relative_to(directory))] = {"unresolved": list(map(str, unresolved)), "absolute_references": absolute}
    stage = Usd.Stage.Open(str(directory / "fdpc4221as.usdc"))
    checks["components_present"] = str(stage.GetDefaultPrim().GetPath()) == asset.ROOT and all(
        stage.GetPrimAtPath(asset.ROOT + "/" + name + "/Visuals")
        and stage.GetPrimAtPath(asset.ROOT + "/" + name + "/Collisions") for name in asset.COMPONENT_FILES)
    counts = {}
    for name, prefix, expected in (("UpperRack", "BowlComb", 52), ("LowerRack", "TineBank", 72)):
        collision = stage.GetPrimAtPath(asset.ROOT + "/" + name + "/Collisions")
        families = {p.GetAttribute("wireFamily").Get() for p in Usd.PrimRange(collision)
                    if p.GetAttribute("wireFamily")} if collision else set()
        counts[name] = len([f for f in families if f.startswith(prefix) and "_Tooth" in f])
        checks["polished_racks"] &= counts[name] == expected
        key = "upper_rack" if name == "UpperRack" else "lower_rack"
        checks["polished_racks"] &= (stage.GetPrimAtPath(asset.ROOT + "/" + name)
            .GetAttribute("geometryRevision").Get() == geometry.PARAMETERS[key]["geometry_revision"])
    parameters = json.loads((directory / "parameters.json").read_text())
    for key in ("upper_rack", "lower_rack"):
        checks["polished_racks"] &= parameters["geometry"][key]["geometry_revision"] == geometry.PARAMETERS[key]["geometry_revision"]
    basket = stage.GetPrimAtPath(asset.ROOT + "/SilverwareBasket")
    translation = basket.GetAttribute("xformOp:translate").Get() if basket else None
    checks["basket_position"] = translation is not None and bool(np.allclose(translation, [.2135, .128, .226], atol=1e-9, rtol=0))
    with tempfile.TemporaryDirectory(prefix="frigidaire-relocated-", dir=collection.parent) as temporary:
        relocated = Path(temporary) / "usd"
        shutil.copytree(directory, relocated)
        for original in files:
            path = relocated / original.relative_to(directory)
            layers, assets, unresolved = UsdUtils.ComputeAllDependencies(str(path))
            local = [Path(layer.realPath).resolve() for layer in layers if layer.realPath]
            local += [Path(str(p)).resolve() for p in assets]
            checks["relocated_dependencies_resolve"] &= (not unresolved and
                all(p == relocated.resolve() or relocated.resolve() in p.parents for p in local))
    digest = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
    sources = (Path(__file__).resolve(), Path(asset.__file__).resolve(), Path(geometry.__file__).resolve())
    return {"result": "PASS" if all(checks.values()) else "FAIL", "scope": "USD composition; no Isaac physics",
            "checks": checks, "tine_counts": counts, "dependencies": details,
            "asset_hashes": {str(p.relative_to(directory)): digest(p) for p in files},
            "parameters_sha256": digest(directory / "parameters.json"),
            "source_hashes": {str(p.relative_to(ROOT)): digest(p) for p in sources}}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--collection-dir", type=Path, default=COLLECTION_DIR)
    args = parser.parse_args()
    from dishsim_frigidaire.usd_bootstrap import ensure_usd
    ensure_usd()
    report = inspect_collection(args.collection_dir)
    output = args.collection_dir / "validation/composition.json"
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, indent=2) + "\n")
    print("[RESULT] %s: current appliance composition and relocation" % report["result"])
    return 0 if report["result"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
