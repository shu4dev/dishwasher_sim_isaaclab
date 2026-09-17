# Copyright (c) 2026, dishsim project.
# SPDX-License-Identifier: BSD-3-Clause
"""Build, inspect, and optionally install the photo-informed standalone lower rack.

Host (numpy only): python3 frigidaire/scripts/setup/polish_lower_rack.py --preview
Runtime: scripts/run_py.sh frigidaire/scripts/setup/polish_lower_rack.py --install

The installation replaces only the LowerRack subtree in the appliance and its loose
rack export. Original files are backed up by content hash before either is replaced.
"""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import sys

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(ROOT / "frigidaire/src"))

from dishsim_frigidaire.lower_rack_asset import MATERIALS, build  # noqa: E402


def preview(rack, output):
    """Diagnostic geometry view. Explicitly NOT an Isaac Sim render."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection
    import numpy as np

    fig = plt.figure(figsize=(14, 14), facecolor="#edf0f2")
    views = [(54, -52, "Front / right"), (80, -90, "Top"),
             (28, -90, "Front"), (35, -178, "Left")]
    # A single collection per view sorts all wire triangles together.
    triangles, colors = [], []
    light = np.array([-.4, -.5, .76])
    for mesh in rack.meshes.values():
        p, t = np.asarray(mesh.points), np.asarray(mesh.faces)
        n = np.asarray(mesh.normals)[t].mean(axis=1)
        shade = 0.50 + 0.50*np.clip(n@light, 0, 1)
        color = np.asarray(MATERIALS[mesh.material][0]) ** (1/2.2)
        triangles.extend(p[t])
        colors.extend(np.clip(shade[:, None]*color, 0, 1))
    for index, (elev, azim, label) in enumerate(views):
        ax = fig.add_subplot(2, 2, index+1, projection="3d")
        ax.add_collection3d(Poly3DCollection(triangles, facecolors=colors,
                                           edgecolors="none", linewidths=0))
        ax.set_xlim(-.28, .28)
        ax.set_ylim(-.30, .28)
        ax.set_zlim(-.03, .18)
        if hasattr(ax, "set_box_aspect"):
            ax.set_box_aspect((.56, .58, .21))
        else:
            # Ubuntu 20.04's matplotlib predates set_box_aspect: use equal data
            # ranges so a 4 mm wire retains its physical proportions in all axes.
            ax.set_ylim(-.29, .27)
            ax.set_zlim(-.222, .338)
            ax.dist = 7
        ax.set_proj_type("ortho")
        ax.view_init(elev=elev, azim=azim)
        ax.set_axis_off()
        ax.set_facecolor("#edf0f2")
        ax.set_title(label, fontsize=14, color="#293b48", pad=-15)
    fig.suptitle("Lower rack geometry inspection — NOT an Isaac Sim render", fontsize=19, y=.97)
    fig.subplots_adjust(left=.01, right=.99, top=.94, bottom=.01, wspace=-.08, hspace=-.08)
    # Older mplot3d stretches its projection to the axes rectangle. A square
    # display box preserves wheel circles and the square basket footprint.
    for ax in fig.axes:
        box = ax.get_position()
        size = min(box.width, box.height)
        ax.set_position([box.x0+(box.width-size)/2, box.y0+(box.height-size)/2, size, size])
    fig.savefig(output, dpi=110)
    plt.close(fig)


def _sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def install(source_usda, asset_dir):
    """Stage and validate both USDC files before any installation; retain root physics."""
    from pxr import Sdf, Usd, UsdGeom, UsdPhysics

    asset_dir = Path(asset_dir)
    source_stage = Usd.Stage.Open(str(source_usda))
    assert source_stage and source_stage.GetDefaultPrim().GetPath() == Sdf.Path("/LowerRack")
    source = source_stage.GetRootLayer()
    pending = []
    report = {"files": [], "isaac_sim_validated": False}
    for filename in ("lower_rack.usdc", "bosch800.usdc"):
        original_path = asset_dir / filename
        original = Usd.Stage.Open(str(original_path))
        assert original, original_path
        rack_path = (original.GetDefaultPrim().GetPath() if filename == "lower_rack.usdc"
                     else original.GetDefaultPrim().GetPath().AppendChild("LowerRack"))
        old_root = original.GetPrimAtPath(rack_path)
        assert old_root and old_root.HasAPI(UsdPhysics.RigidBodyAPI), rack_path
        # Keep original asset paths and opinions verbatim. Flattening would resolve
        # relative textures to absolute paths and break the portable asset bundle.
        before = original.GetRootLayer()
        assert before.GetPrimAtPath(rack_path), "Expected the prebuilt rack in the root layer"
        edited = Sdf.Layer.CreateAnonymous("polished.usda")
        edited.TransferContent(before)
        Sdf.CopySpec(source, Sdf.Path("/LowerRack"), edited, rack_path)
        # Copy only original root attributes, including mass, inertia, transform and
        # velocity state. New material bindings and all children belong to the revision.
        for attribute in old_root.GetAuthoredAttributes():
            Sdf.CopySpec(before, attribute.GetPath(), edited, attribute.GetPath())
        schemas = list(dict.fromkeys(source_stage.GetDefaultPrim().GetAppliedSchemas()
                                     + old_root.GetAppliedSchemas()))
        edited.GetPrimAtPath(rack_path).SetInfo("apiSchemas", Sdf.TokenListOp.CreateExplicit(schemas))
        # Preserve any extra authored root APIs from the original body.
        staged = Usd.Stage.Open(edited)
        staged_root = staged.GetPrimAtPath(rack_path)
        revision = staged_root.GetAttribute("geometryRevision")
        revision.Set("bottom_rack_four_views_v1")
        assert len([p for p in Usd.PrimRange(staged_root) if p.IsA(UsdGeom.Mesh)]) == 23
        assert len([p for p in Usd.PrimRange(staged_root) if p.HasAPI(UsdPhysics.RigidBodyAPI)]) == 1
        # Every material relationship must resolve after the /LowerRack path remap.
        for prim in Usd.PrimRange(staged_root):
            for rel in prim.GetRelationships():
                for target in rel.GetTargets():
                    assert staged.GetObjectAtPath(target), (prim.GetPath(), target)
        # Exact layer comparison outside the replaced subtree guards the middle rack,
        # upper tray, cabinet, joints, shared materials, and all their authored values.
        a, b = Sdf.Layer.CreateAnonymous(), Sdf.Layer.CreateAnonymous()
        a.TransferContent(before)
        b.TransferContent(edited)
        removal = Sdf.BatchNamespaceEdit()
        removal.Add(rack_path, Sdf.Path.emptyPath)
        assert a.Apply(removal) and b.Apply(removal)
        assert a.ExportToString() == b.ExportToString(), "Change outside LowerRack"
        # A sibling temporary file keeps any relative texture paths correct.
        temporary = asset_dir / (filename + ".polishing.usdc")
        assert edited.Export(str(temporary))
        check = Usd.Stage.Open(str(temporary))
        assert check and check.GetPrimAtPath(rack_path.AppendPath("Visuals/WireHandle"))
        for prim in check.Traverse():
            if prim.IsA(UsdGeom.Mesh):
                mesh = UsdGeom.Mesh(prim)
                valid, reason = UsdGeom.Mesh.ValidateTopology(mesh.GetFaceVertexIndicesAttr().Get(),
                    mesh.GetFaceVertexCountsAttr().Get(), len(mesh.GetPointsAttr().Get()))
                assert valid, (prim.GetPath(), reason)
        pending.append((original_path, temporary))
    backup_dir = asset_dir / "backups" / "lower_rack_polish"
    backup_dir.mkdir(parents=True, exist_ok=True)
    # Back up BOTH originals first. A failure during replacement restores the pair.
    backups = []
    for original, temporary in pending:
        digest = _sha(original)
        backup = backup_dir / (original.stem + "_" + digest[:16] + original.suffix)
        if not backup.exists():
            shutil.copy2(original, backup)
        assert _sha(backup) == digest
        backups.append((original, backup))
        report["files"].append({"asset": str(original), "backup": str(backup),
                                "before_sha256": digest, "after_sha256": _sha(temporary)})
    try:
        for original, temporary in pending:
            temporary.replace(original)
    except BaseException:
        for original, backup in backups:
            shutil.copy2(backup, original)
        raise
    report["result"] = "PASS (USD installation; simulation still requires capture)"
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out_dir", type=Path, default=ROOT / "build/bosch800_lower_rack")
    parser.add_argument("--asset_dir", type=Path, default=ROOT / "assets/models/bosch800")
    parser.add_argument("--preview", action="store_true")
    parser.add_argument("--install", action="store_true")
    args = parser.parse_args()
    rack, report = build(args.out_dir)
    print("[RESULT]", report["result"], flush=True)
    print(f"[INFO] {len(rack.meshes)} meshes; {report['colliders']} colliders; size {report['size_m']} m", flush=True)
    if args.preview:
        preview(rack, args.out_dir / "geometry_preview.png")
        print("[INFO] Diagnostic preview saved (not Isaac Sim).", flush=True)
    if args.install:
        result = install(args.out_dir / "lower_rack.usda", args.asset_dir)
        (args.out_dir / "installation.json").write_text(json.dumps(result, indent=2)+"\n")
        print("[RESULT]", result["result"], flush=True)


if __name__ == "__main__":
    main()
