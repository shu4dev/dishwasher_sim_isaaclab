# Copyright (c) 2026, dishsim project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Phase D (venv side, no Kit): convex pieces for every cached body + rack feasibility probes.

Reads ``assets/cache/scene_state.json`` and fills ``assets/cache/coacd/<name>_<hash>/`` with
convex ``piece_*.obj`` files for every body flagged ``coacd``. Two paths:

- **Racks** (``config.RACK_GEN`` bodies): the pieces are rack_gen's exact convex parts — no
  CoACD, zero decomposition slop on the 3 mm wires. The extracted mesh is first verified against
  the regenerated geometry (bounds + surface area) to catch a stale cache or a scale
  pre-compensation bug.
- **Everything else** (dishwasher shell/door, carried object): CoACD with ``config.COACD``.

The piece-dir hash covers mesh bytes + parameters, so config changes invalidate automatically.

Rack probes (replacing the old flat-basket center probe, which tines would legitimately hit):
object-sized boxes hovering over each OPEN zone must be FCL-free, thin plate-shaped boxes in
sampled gaps between plate tines must be free, and the same box centered ON a tine must collide
(negative control: catches missing or misplaced pieces).

Run with:
    /workspace/isaaclab/env_isaaclab/bin/python scripts/13_decompose_meshes.py [--force]
"""

import argparse
import os
import sys

import numpy as np

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(PROJECT_ROOT, "src"))

import fcl  # noqa: E402
import trimesh  # noqa: E402

from dishsim import config  # noqa: E402
from dishsim.geometry import coacd_dir_for, load_manifest  # noqa: E402

parser = argparse.ArgumentParser(description="CoACD decomposition of the collision cache.")
parser.add_argument("--force", action="store_true", help="Re-decompose even if outputs exist.")
parser.add_argument("--scenario", type=str, default="both_out",
                    help="Rack-state scenario (see config.SCENARIOS).")
args = parser.parse_args()

config.apply_scenario(args.scenario)
CACHE = config.scenario_cache_dir()

FAILURES: list[str] = []


def check(name: str, ok: bool, detail: str = "") -> None:
    print(f"[{'OK' if ok else 'FAIL'}] {name}{': ' + detail if detail else ''}")
    if not ok:
        FAILURES.append(name)


def coacd_params_for(name: str) -> dict:
    if name in config.COACD:
        return config.COACD[name]
    return config.COACD["default"]


def decompose(name: str, mesh_rel: str) -> str | None:
    """Fill the piece dir for one body; returns the dir, or None when the rack_gen alignment
    gate failed (recorded in FAILURES; the piece dir is left empty)."""
    out_dir = coacd_dir_for(name, mesh_rel, CACHE)
    cached = os.path.isdir(out_dir) and not args.force and bool(os.listdir(out_dir))

    if name in config.RACK_GEN:  # analytic path: the generator's parts ARE the decomposition
        from dishsim import rack_gen  # noqa: PLC0415

        # The alignment gate runs on EVERY invocation (it is cheap): a failed run must never
        # self-heal into a "cached" PASS, and a partially written dir must never be accepted.
        parts = rack_gen.build_rack(config.RACK_GEN[name])
        mesh = trimesh.load(os.path.join(CACHE, mesh_rel), force="mesh")
        ref = rack_gen.merged_mesh(parts)
        dev = float(np.abs(np.asarray(mesh.bounds) - np.asarray(ref.bounds)).max())
        area_err = abs(float(mesh.area) / float(ref.area) - 1.0)
        ok = dev < 2e-3 and area_err < 0.02
        check(
            f"rack_gen alignment ({name})",
            ok,
            f"extracted vs generated: bounds dev {dev * 1e3:.2f} mm, area err {area_err * 100:.2f}% "
            "(stale cache or scale pre-compensation bug -> re-run scripts/12)",
        )
        if not ok:  # never leave a populated-but-unverified piece dir behind
            if os.path.isdir(out_dir):
                for old in os.listdir(out_dir):
                    os.remove(os.path.join(out_dir, old))
            return None  # caller must skip probes/overlay: they would be vacuous or crash
        if cached and len(os.listdir(out_dir)) == len(parts):
            print(f"[INFO] {name}: cached at {out_dir} ({len(parts)} pieces, alignment re-verified)")
            return out_dir
        os.makedirs(out_dir, exist_ok=True)
        for old in os.listdir(out_dir):
            os.remove(os.path.join(out_dir, old))
        for i, part in enumerate(parts):
            part.mesh.export(os.path.join(out_dir, f"piece_{i:03d}.obj"))
        print(f"[INFO] {name}: {len(parts)} analytic pieces (rack_gen) -> {out_dir}")
        return out_dir

    if cached:
        print(f"[INFO] {name}: cached at {out_dir} ({len(os.listdir(out_dir))} pieces)")
        return out_dir
    import coacd  # noqa: PLC0415

    coacd.set_log_level("error")
    mesh = trimesh.load(os.path.join(CACHE, mesh_rel), force="mesh")
    params = coacd_params_for(name)
    parts = coacd.run_coacd(coacd.Mesh(mesh.vertices, mesh.faces), **params)
    os.makedirs(out_dir, exist_ok=True)
    for old in os.listdir(out_dir):
        os.remove(os.path.join(out_dir, old))
    for i, (verts, faces) in enumerate(parts):
        piece = trimesh.Trimesh(vertices=np.asarray(verts), faces=np.asarray(faces))
        piece.export(os.path.join(out_dir, f"piece_{i:03d}.obj"))
    print(f"[INFO] {name}: {len(parts)} pieces (params {params}) -> {out_dir}")
    return out_dir


def fcl_convex(mesh: trimesh.Trimesh) -> fcl.Convex:
    hull = mesh.convex_hull
    faces = np.hstack([np.full((len(hull.faces), 1), 3, dtype=np.int64), hull.faces.astype(np.int64)])
    return fcl.Convex(hull.vertices.astype(np.float64), len(hull.faces), faces.flatten())


def rack_probes(rack_name: str, out_dir: str) -> None:
    """rack_gen feasibility gates against the pieces ON DISK (what CollisionWorld will load)."""
    from dishsim import rack_gen  # noqa: PLC0415

    params = config.RACK_GEN[rack_name]
    mgr = fcl.DynamicAABBTreeCollisionManager()
    objs = []
    for f in sorted(os.listdir(out_dir)):
        piece = trimesh.load(os.path.join(out_dir, f), force="mesh")
        objs.append(fcl.CollisionObject(fcl_convex(piece)))
    mgr.registerObjects(objs)
    mgr.setup()

    def hits(extents, center) -> bool:
        probe = fcl.CollisionObject(fcl.Box(*extents), fcl.Transform(np.asarray(center, dtype=float)))
        cdata = fcl.CollisionData()
        mgr.collide(probe, cdata, fcl.defaultCollisionCallback)
        return bool(cdata.result.is_collision)

    # object-sized box hovering over each open zone at the Phase-E release height: must be free
    for i, (ext, ctr) in enumerate(rack_gen.mug_probes(params)):
        got = hits(ext, ctr)
        check(
            f"mug-zone probe ({rack_name} #{i})",
            not got,
            f"object box over open zone {i} {'collides' if got else 'is free'} ({len(objs)} pieces)",
        )
    if "plate_rows_y" in params:
        # thin plate boxes in sampled tine gaps: must be free (the rack's actual purpose)
        for i, (ext, ctr) in enumerate(rack_gen.plate_gap_probes(params)):
            got = hits(ext, ctr)
            check(
                f"plate-gap probe ({rack_name} #{i})",
                not got,
                f"plate box in tine gap at x={ctr[0]:.3f} {'collides' if got else 'is free'}",
            )
        # negative control ON a tine: must collide, else pieces are missing/misplaced
        ext, ctr = rack_gen.plate_tine_negative_probe(params)
        got = hits(ext, ctr)
        check(
            f"tine negative control ({rack_name})",
            got,
            f"box centered on the tine column at x={ctr[0]:.3f} {'collides as expected' if got else 'is UNEXPECTEDLY free'}",
        )


def render_overlay(name: str, mesh_rel: str, pieces_dir: str, out_png: str) -> None:
    """Decomposed convex pieces (colored, translucent) over the source mesh (black points)."""
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection

    src = trimesh.load(os.path.join(CACHE, mesh_rel), force="mesh")
    pieces = [trimesh.load(os.path.join(pieces_dir, f), force="mesh") for f in sorted(os.listdir(pieces_dir))]
    rng = np.random.default_rng(0)
    pts = src.vertices[rng.choice(len(src.vertices), min(4000, len(src.vertices)), replace=False)]
    mn, mx = src.bounds
    fig = plt.figure(figsize=(16, 5.5))
    cmap = plt.get_cmap("tab20")
    for k, (elev, azim, label) in enumerate([(22, -60, "iso"), (88, -90, "top"), (2, -90, "side")]):
        ax = fig.add_subplot(1, 3, k + 1, projection="3d")
        ax.scatter(pts[:, 0], pts[:, 1], pts[:, 2], s=0.3, c="k", alpha=0.35, linewidths=0)
        for i, p in enumerate(pieces):
            hull = p.convex_hull
            ax.add_collection3d(
                Poly3DCollection(hull.vertices[hull.faces], alpha=0.30, facecolor=cmap(i % 20), edgecolor="none")
            )
        ax.set_xlim(mn[0], mx[0])
        ax.set_ylim(mn[1], mx[1])
        ax.set_zlim(mn[2], mx[2])
        ax.set_box_aspect((mx - mn))
        ax.view_init(elev=elev, azim=azim)
        ax.set_title(f"{name} — {label} ({len(pieces)} pieces)")
        ax.set_axis_off()
    fig.tight_layout()
    os.makedirs(os.path.dirname(out_png), exist_ok=True)
    fig.savefig(out_png, dpi=110)
    plt.close(fig)
    print(f"[INFO] overlay: {out_png}")


def main() -> None:
    try:
        manifest = load_manifest(CACHE)
    except FileNotFoundError:
        raise SystemExit(
            f"[FAIL] no cache at {CACHE} — run scripts/12_extract_geometry.py "
            f"--scenario {config.SCENARIO_NAME} first"
        ) from None
    media_dir = config.scenario_media_dir("D")
    for name, entry in manifest["statics"].items():
        if entry.get("coacd"):
            out = decompose(name, entry["mesh"])
            if out is None:  # alignment gate failed — already recorded, skip dependent steps
                continue
            if name in config.RACK_GEN:
                rack_probes(name, out)
            render_overlay(name, entry["mesh"], out, os.path.join(media_dir, f"overlay_{name}.png"))
    out = decompose("object", manifest["object"]["mesh"])
    render_overlay("object", manifest["object"]["mesh"], out, os.path.join(media_dir, "overlay_object.png"))
    print(f"[RESULT] {'PASS' if not FAILURES else 'FAIL: ' + ', '.join(FAILURES)}")
    if FAILURES:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
