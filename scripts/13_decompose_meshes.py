# Copyright (c) 2026, dishsim project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Phase D (venv side, no Kit): CoACD-decompose the cached meshes + tine-gap feasibility probe.

Reads ``assets/cache/scene_state.json``, decomposes every body flagged ``coacd`` (racks,
dishwasher body/door, carried object) with the per-body parameters from ``config.COACD``, and
writes convex pieces to ``assets/cache/coacd/<name>_<hash>/piece_*.obj``. The hash covers mesh
bytes + parameters, so changing ``config.COACD`` automatically invalidates.

The built-in gap probe places an object-sized box at the lower-rack basket center and requires
it FCL-free vs the decomposed rack: if a too-coarse decomposition seals the basket interior,
every placement would falsely read "in collision" — decomposition quality is a feasibility
parameter, not just an accuracy one.

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
parser.add_argument("--scenario", type=str, default="lower_out",
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
    if name == "object":
        return config.COACD["object"]
    return config.COACD["default"]


def decompose(name: str, mesh_rel: str) -> str:
    import coacd  # noqa: PLC0415

    coacd.set_log_level("error")
    out_dir = coacd_dir_for(name, mesh_rel, CACHE)
    if os.path.isdir(out_dir) and not args.force and os.listdir(out_dir):
        print(f"[INFO] {name}: cached at {out_dir} ({len(os.listdir(out_dir))} pieces)")
        return out_dir
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


def tine_gap_probe(rack_name: str, rack_mesh_rel: str, out_dir: str) -> None:
    """Object-sized box at the basket center must be collision-free vs the decomposed rack."""
    rack_mesh = trimesh.load(os.path.join(CACHE, rack_mesh_rel), force="mesh")
    mn, mx = rack_mesh.bounds
    # basket floor: the rack's wire base sits near the bbox bottom; the object stands on it
    probe_size = (
        2 * config.OBJECT_BBOX_HALF[0] + 0.005,
        2 * config.OBJECT_BBOX_HALF[1] + 0.005,
        config.OBJECT_HEIGHT_M,
    )
    center = (mn + mx) / 2.0
    probe_z = mn[2] + 0.025 + probe_size[2] / 2.0  # ~25 mm above bbox bottom = above the base wires
    probe = fcl.CollisionObject(
        fcl_convex(trimesh.creation.box(extents=probe_size)),
        fcl.Transform(np.array([center[0], center[1], probe_z])),
    )
    mgr = fcl.DynamicAABBTreeCollisionManager()
    objs = []
    for f in sorted(os.listdir(out_dir)):
        piece = trimesh.load(os.path.join(out_dir, f), force="mesh")
        objs.append(fcl.CollisionObject(fcl_convex(piece)))
    mgr.registerObjects(objs)
    mgr.setup()
    cdata = fcl.CollisionData()
    mgr.collide(probe, cdata, fcl.defaultCollisionCallback)
    check(
        f"tine-gap probe ({rack_name})",
        not cdata.result.is_collision,
        f"object-sized box at basket center {'collides' if cdata.result.is_collision else 'is free'} "
        f"({len(objs)} pieces)",
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
            if name == "E_shelf_1_04":
                tine_gap_probe(name, entry["mesh"], out)
            render_overlay(name, entry["mesh"], out, os.path.join(media_dir, f"overlay_{name}.png"))
    out = decompose("object", manifest["object"]["mesh"])
    render_overlay("object", manifest["object"]["mesh"], out, os.path.join(media_dir, "overlay_object.png"))
    print(f"[RESULT] {'PASS' if not FAILURES else 'FAIL: ' + ', '.join(FAILURES)}")
    if FAILURES:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
