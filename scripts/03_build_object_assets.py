# Copyright (c) 2026, dishsim project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Build the multi-object asset library (physics USDs + Kit-free meshes + evidence renders).

Sources (see ``config.OBJECTS``):
- ``ycb16k``: official YCB textured scans (``<id>_google_16k.tgz`` from the
  ycb-benchmarks S3 mirror) — downloaded, scaled per the registry's documented scale
  factor, auto-oriented to the canonical frame (Z-up dishware / length-along-X utensils,
  origin at the bbox center), measured, and authored as a textured USD with physics APIs.
- ``procedural``: :mod:`dishsim.prop_gen` builders — authored as per-part USD meshes, each
  with an exact convex-hull collider (no CoACD slop on thin shells).
- ``isaac_ycb`` (the mug): built by ``scripts/01_make_prop_physics_usd.py``; skipped here.

Numeric provenance: measured dims are printed per object and compared against the registry
(±2 mm gate). On mismatch the block to freeze into ``config.OBJECTS`` is printed and the
run FAILs — freeze the measured numbers, then re-run to green.

Outputs:
- ``assets/props/<name>_physics.usd`` (+ ``assets/props/textures/<name>.png``)
- ``assets/props/meshes/<name>.obj`` (body-frame, Kit-free consumers)
- ``assets/props/parts/<name>/piece_%03d.obj`` (procedural analytic collision parts)
- ``media/assets/<name>_views.png`` + ``media/assets/library_sheet.png``

Runs without Kit boot (pxr + trimesh only): ``scripts/run_kit.sh scripts/03_build_object_assets.py``
"""

import argparse
import os
import sys
import tarfile
import urllib.request

import numpy as np

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(PROJECT_ROOT, "src"))

from dishsim import config  # noqa: E402

YCB_URL = "http://ycb-benchmarks.s3.amazonaws.com/data/google/{sid}_google_16k.tgz"
DOWNLOAD_DIR = os.path.join(config.ASSETS_DIR, "props", "_download", "ycb16k")
PROPS_DIR = os.path.join(config.ASSETS_DIR, "props")
MEDIA_DIR = os.path.join(PROJECT_ROOT, "media", "assets")

parser = argparse.ArgumentParser(description="Build the multi-object asset library.")
parser.add_argument("--objects", type=str, default="all",
                    help="Comma list of registry names (default: every non-mug object).")
parser.add_argument("--force", action="store_true", help="Rebuild even if the USD exists.")
args = parser.parse_args()

MISMATCHES: list[str] = []


def download_ycb(sid: str) -> str:
    """Download + extract one google_16k tgz; returns the textured.obj path."""
    os.makedirs(DOWNLOAD_DIR, exist_ok=True)
    tgz = os.path.join(DOWNLOAD_DIR, f"{sid}_google_16k.tgz")
    if not os.path.exists(tgz):
        print(f"[INFO] downloading {sid} ...")
        urllib.request.urlretrieve(YCB_URL.format(sid=sid), tgz)
    obj_dir = os.path.join(DOWNLOAD_DIR, sid, "google_16k")
    if not os.path.exists(os.path.join(obj_dir, "textured.obj")):
        with tarfile.open(tgz) as tf:
            tf.extractall(DOWNLOAD_DIR, filter="data")
    path = os.path.join(obj_dir, "textured.obj")
    if not os.path.exists(path):
        raise FileNotFoundError(f"{sid}: textured.obj not found under {obj_dir}")
    return path


def orient_to_canonical(spec, mesh):
    """Rotate the source scan into the canonical frame (before scaling).

    Dishware (axis z): google_16k scans sit naturally (z up, opening up) — identity.
    Utensils (axis x): PCA-align longest principal direction -> x, thinnest -> z, then put
    the wider end (the head) at +x.
    """
    import trimesh  # noqa: PLC0415

    if tuple(spec.axis_obj) != (1.0, 0.0, 0.0):
        return mesh
    v = mesh.vertices - mesh.vertices.mean(axis=0)
    cov = np.cov(v.T)
    evals, evecs = np.linalg.eigh(cov)  # ascending
    R = np.column_stack([evecs[:, 2], evecs[:, 1], evecs[:, 0]])  # longest->x, thinnest->z
    if np.linalg.det(R) < 0:
        R[:, 1] *= -1.0
    mesh = mesh.copy()
    mesh.vertices = v @ R  # rotate into PCA frame (R columns = new axes in old coords)
    # head (wider end in y) at +x
    x, y = mesh.vertices[:, 0], mesh.vertices[:, 1]
    lo, hi = np.percentile(x, [30, 70])
    if y[x < lo].std() > y[x > hi].std():
        mesh.vertices[:, 0] *= -1.0
        mesh.vertices[:, 1] *= -1.0  # keep det +1 (rotate 180 about z)
    return mesh


def measure(spec, mesh) -> dict:
    """Measured dims in the canonical frame; returns the freeze block values."""
    mn, mx = mesh.bounds
    half = (mx - mn) / 2.0
    axis = int(np.argmax(spec.axis_obj))
    height = float(mx[axis] - mn[axis])
    v = mesh.vertices
    if tuple(spec.axis_obj) == (0.0, 0.0, 1.0):
        rim = float(np.hypot(v[:, 0], v[:, 1]).max())
    else:  # length-along-x utensils: rim := max half-width
        rim = float(np.abs(v[:, 1]).max())
    return {"bbox_half": tuple(np.round(half, 4)), "height_m": round(height, 4),
            "rim_radius_m": round(rim, 4)}


def check_measured(spec, meas) -> None:
    dev = max(abs(a - b) for a, b in zip(meas["bbox_half"], spec.bbox_half))
    if spec.source != "procedural":
        # procedural rim/height are definitional (prop_gen builds FROM the registry); the
        # radial probe also reads box diagonals for rectangular parts, which is not the
        # rim_edge grasp semantic (distance to the grasped wall)
        dev = max(dev, abs(meas["height_m"] - spec.height_m), abs(meas["rim_radius_m"] - spec.rim_radius_m))
    tag = "OK" if dev <= 0.002 else "MISMATCH"
    print(f"[{tag}] {spec.name}: measured bbox_half={meas['bbox_half']} height={meas['height_m']} "
          f"rim_r={meas['rim_radius_m']} (registry dev {dev * 1000:.1f} mm)")
    if dev > 0.002:
        MISMATCHES.append(spec.name)
        print(f"    freeze into config.OBJECTS[{spec.name!r}]: bbox_half={meas['bbox_half']}, "
              f"height_m={meas['height_m']}, rim_radius_m={meas['rim_radius_m']}")


def _author_physics(stage, root_prim, mass_kg: float, mesh_prims, approximation: str) -> None:
    from pxr import UsdPhysics  # noqa: PLC0415

    UsdPhysics.RigidBodyAPI.Apply(root_prim)
    mass = UsdPhysics.MassAPI.Apply(root_prim)
    mass.CreateMassAttr(float(mass_kg))
    for prim in mesh_prims:
        UsdPhysics.CollisionAPI.Apply(prim)
        mesh_col = UsdPhysics.MeshCollisionAPI.Apply(prim)
        mesh_col.CreateApproximationAttr(approximation)


def _author_mesh_prim(stage, path: str, mesh, uv=None, color=None):
    from pxr import Gf, Sdf, UsdGeom  # noqa: PLC0415

    prim = UsdGeom.Mesh.Define(stage, path)
    prim.CreatePointsAttr([Gf.Vec3f(*p) for p in mesh.vertices.astype(float)])
    prim.CreateFaceVertexCountsAttr([3] * len(mesh.faces))
    prim.CreateFaceVertexIndicesAttr([int(i) for i in mesh.faces.reshape(-1)])
    prim.CreateSubdivisionSchemeAttr("none")
    if uv is not None:
        pv = UsdGeom.PrimvarsAPI(prim).CreatePrimvar(
            "st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.vertex
        )
        pv.Set([Gf.Vec2f(*t) for t in uv.astype(float)])
    if color is not None:
        prim.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    return prim


def author_usd_textured(spec, mesh, tex_src: str, out_path: str) -> None:
    from pxr import Sdf, Usd, UsdShade  # noqa: PLC0415

    stage = Usd.Stage.CreateNew(out_path)
    root = stage.DefinePrim("/Root", "Xform")
    stage.SetDefaultPrim(root)
    uv = getattr(mesh.visual, "uv", None)
    if uv is not None and len(uv) != len(mesh.vertices):
        uv = None
    prim = _author_mesh_prim(stage, "/Root/geom", mesh, uv=uv)
    if uv is not None and tex_src:
        import shutil  # noqa: PLC0415

        tex_dir = os.path.join(PROPS_DIR, "textures")
        os.makedirs(tex_dir, exist_ok=True)
        tex_dst = os.path.join(tex_dir, f"{spec.name}.png")
        shutil.copyfile(tex_src, tex_dst)
        mat = UsdShade.Material.Define(stage, "/Root/Looks/mat")
        pbr = UsdShade.Shader.Define(stage, "/Root/Looks/mat/pbr")
        pbr.CreateIdAttr("UsdPreviewSurface")
        pbr.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.4)
        st_reader = UsdShade.Shader.Define(stage, "/Root/Looks/mat/st_reader")
        st_reader.CreateIdAttr("UsdPrimvarReader_float2")
        st_reader.CreateInput("varname", Sdf.ValueTypeNames.String).Set("st")
        tex = UsdShade.Shader.Define(stage, "/Root/Looks/mat/tex")
        tex.CreateIdAttr("UsdUVTexture")
        tex.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(f"./textures/{spec.name}.png")
        tex.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(
            st_reader.CreateOutput("result", Sdf.ValueTypeNames.Float2)
        )
        pbr.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(
            tex.CreateOutput("rgb", Sdf.ValueTypeNames.Float3)
        )
        mat.CreateSurfaceOutput().ConnectToSource(
            pbr.CreateOutput("surface", Sdf.ValueTypeNames.Token)
        )
        UsdShade.MaterialBindingAPI.Apply(prim.GetPrim()).Bind(mat)
    _author_physics(stage, root, spec.mass_kg, [prim.GetPrim()], "convexDecomposition")
    stage.GetRootLayer().Save()
    print(f"[INFO] wrote {out_path} (mass {spec.mass_kg} kg, textured={uv is not None})")


def author_usd_parts(spec, parts, color, out_path: str) -> None:
    from pxr import Usd  # noqa: PLC0415

    stage = Usd.Stage.CreateNew(out_path)
    root = stage.DefinePrim("/Root", "Xform")
    stage.SetDefaultPrim(root)
    prims = []
    for i, part in enumerate(parts):
        prims.append(_author_mesh_prim(stage, f"/Root/part_{i:03d}", part, color=color).GetPrim())
    _author_physics(stage, root, spec.mass_kg, prims, "convexHull")
    stage.GetRootLayer().Save()
    print(f"[INFO] wrote {out_path} ({len(parts)} convex parts, mass {spec.mass_kg} kg)")


def export_meshes(spec, mesh, parts=None) -> None:
    import trimesh  # noqa: PLC0415

    mesh_dir = os.path.join(PROPS_DIR, "meshes")
    os.makedirs(mesh_dir, exist_ok=True)
    geo = trimesh.Trimesh(vertices=mesh.vertices, faces=mesh.faces, process=False)
    geo.export(os.path.join(mesh_dir, f"{spec.name}.obj"))
    if parts is not None:
        part_dir = os.path.join(PROPS_DIR, "parts", spec.name)
        os.makedirs(part_dir, exist_ok=True)
        for i, part in enumerate(parts):
            part.export(os.path.join(part_dir, f"piece_{i:03d}.obj"))


def render_views(spec, mesh, out_png: str) -> None:
    import matplotlib  # noqa: PLC0415

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt  # noqa: PLC0415
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection  # noqa: PLC0415

    fig = plt.figure(figsize=(9, 3.2))
    views = [(90, -90, "top"), (0, -90, "front"), (28, -50, "iso")]
    for k, (elev, azim, tag) in enumerate(views):
        ax = fig.add_subplot(1, 3, k + 1, projection="3d")
        tri = mesh.vertices[mesh.faces]
        ax.add_collection3d(Poly3DCollection(tri, alpha=0.55, facecolor="#7fa8c9", edgecolor="none"))
        r = float(np.abs(mesh.bounds).max()) * 1.15
        ax.set_xlim(-r, r); ax.set_ylim(-r, r); ax.set_zlim(-r, r)
        ax.view_init(elev=elev, azim=azim)
        ax.set_axis_off()
        ax.set_title(tag, fontsize=8)
    dims = " x ".join(f"{2 * h * 1000:.0f}" for h in spec.bbox_half)
    fig.suptitle(f"{spec.name} ({spec.source}, scale {spec.scale}) — {dims} mm, {spec.mass_kg} kg",
                 fontsize=9)
    fig.tight_layout()
    fig.savefig(out_png, dpi=120)
    plt.close(fig)


def build_one(name: str) -> str | None:
    import trimesh  # noqa: PLC0415

    spec = config.OBJECTS[name]
    out_usd = spec.usd_path
    if os.path.exists(out_usd) and not args.force:
        print(f"[SKIP] {name}: {out_usd} exists")
        return None
    if spec.source == "ycb16k":
        obj_path = download_ycb(spec.source_id)
        mesh = trimesh.load(obj_path, force="mesh", process=False)
        mesh = orient_to_canonical(spec, mesh)
        mesh.apply_scale(spec.scale)
        mesh.apply_translation(-mesh.bounds.mean(axis=0))
        meas = measure(spec, mesh)
        check_measured(spec, meas)
        tex = os.path.join(os.path.dirname(obj_path), "texture_map.png")
        author_usd_textured(spec, mesh, tex if os.path.exists(tex) else None, out_usd)
        export_meshes(spec, mesh)
    elif spec.source == "procedural":
        from dishsim import prop_gen  # noqa: PLC0415

        parts, color = prop_gen.build(spec.source_id)
        mesh = trimesh.util.concatenate(parts)
        meas = measure(spec, mesh)
        check_measured(spec, meas)
        author_usd_parts(spec, parts, color, out_usd)
        export_meshes(spec, mesh, parts=parts)
    else:
        print(f"[SKIP] {name}: source {spec.source} is built by scripts/01")
        return None
    png = os.path.join(MEDIA_DIR, f"{name}_views.png")
    render_views(spec, mesh, png)
    return png


def main() -> None:
    os.makedirs(MEDIA_DIR, exist_ok=True)
    names = (
        [n for n in config.OBJECTS if config.OBJECTS[n].source != "isaac_ycb"]
        if args.objects == "all"
        else [n.strip() for n in args.objects.split(",")]
    )
    pngs = []
    for name in names:
        png = build_one(name)
        if png:
            pngs.append((name, png))
    if pngs:
        from PIL import Image  # noqa: PLC0415

        from dishsim.media import contact_sheet  # noqa: PLC0415

        images = [np.asarray(Image.open(p).convert("RGB")) for _, p in pngs]
        labels = [name for name, _ in pngs]
        contact_sheet(images, labels, os.path.join(MEDIA_DIR, "library_sheet.png"), cols=2)
        print(f"[INFO] contact sheet: {os.path.join(MEDIA_DIR, 'library_sheet.png')}")
    if MISMATCHES:
        print(f"[RESULT] FAIL — freeze measured dims for: {', '.join(MISMATCHES)}")
    else:
        print("[RESULT] PASS")


if __name__ == "__main__":
    main()
