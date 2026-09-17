#!/usr/bin/env python3
"""Illustrate one saved accepted pose per dish/rack cell without running Isaac.

Actual source-generated visual triangles, including the mug handle, are placed
at measured final world poses. Geometry-source and staged-USD hashes must match
the recorded experiment. The image is a reconstruction, not a new simulation
or Isaac render. Uses NumPy, SciPy and Matplotlib from the existing container.

    scripts/run_py.sh frigidaire/scripts/evaluation/frigidaire_random_pose_views.py \
        --run-dir results/random_poses/frigidaire/smoke_20260910_01 \
        --out /tmp/frigidaire_accepted_pose_examples.png
"""
from __future__ import annotations

import argparse
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
sys.path[:0] = [str(ROOT / "src"), str(ROOT / "frigidaire/src")]

from dishsim_frigidaire import geometry, tableware
from dishsim_frigidaire.random_pose_assets import validate_inputs
from dishsim_frigidaire.random_poses import KINDS, RACKS, transform_vertices


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def load_run(run_dir, usd_path):
    """Require finalized physical records and exactly matching visual sources."""
    run_dir, usd_path = Path(run_dir), Path(usd_path)
    metadata = json.loads((run_dir / "metadata.json").read_text())
    summary = json.loads((run_dir / "summary.json").read_text())
    accepted = json.loads((run_dir / "accepted_poses.json").read_text())["trials"]
    if not metadata.get("finished_utc") or summary.get("status") in {"running", "not_run", "preflight_only"}:
        raise ValueError("Visualization requires a finalized run, not live/preflight artifacts")
    if not summary.get("simulation_run"):
        raise ValueError("No physical random-pose trials ran")
    expected = metadata.get("inputs", {}).get("asset_hashes")
    actual = validate_inputs(usd_path)["asset_hashes"]
    if expected != actual:
        raise ValueError("Current staged geometry does not match this run's asset hashes")
    required_sources = [ROOT / "frigidaire/src/dishsim_frigidaire" / (name + ".py")
                        for name in ("geometry", "tableware", "asset")]
    for path in required_sources:
        key = str(path.relative_to(ROOT))
        if metadata.get("source_hashes", {}).get(key) != digest(path):
            raise ValueError("Visual geometry source differs from recorded run: " + key)
    if len(accepted) != summary.get("totals", {}).get("accepted"):
        raise ValueError("Accepted records do not match the finalized summary")
    cells = {(kind, rack): [] for kind in KINDS for rack in RACKS}
    for trial in accepted:
        key = (trial["kind"], trial["rack"])
        if trial.get("outcome") != "accepted" or key not in cells:
            raise ValueError("Invalid accepted trial")
        if not trial.get("final_pose") or not trial.get("final_hold", {}).get("poses"):
            raise ValueError("Accepted trial is missing measured final world frames")
        cells[key].append(trial)
    selected = {key: min(values, key=lambda trial: (trial["sample_index"], trial["trial_id"]))
                if values else None for key, values in cells.items()}
    return metadata, summary, selected


def primitive_triangles(solid):
    """Tessellate visible authored boxes/cylinders; sizes are never inferred."""
    center = np.asarray(solid["center"], dtype=float)
    if solid["kind"] == "box":
        points = np.array([[x, y, z] for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)], dtype=float)
        points = center + points * np.asarray(solid["size"]) / 2
        faces = [[0, 1, 3], [0, 3, 2], [4, 6, 7], [4, 7, 5], [0, 4, 5], [0, 5, 1],
                 [2, 3, 7], [2, 7, 6], [0, 2, 6], [0, 6, 4], [1, 5, 7], [1, 7, 3]]
        return points[np.asarray(faces)]
    if solid["kind"] != "cylinder":
        raise ValueError("Unsupported visible primitive: " + solid["kind"])
    axis = "XYZ".index(solid.get("axis", "Z"))
    radial = [i for i in range(3) if i != axis]
    angles = np.arange(32) * 2*np.pi/32
    ring = np.zeros((32, 3))
    ring[:, radial[0]] = solid["radius"]*np.cos(angles)
    ring[:, radial[1]] = solid["radius"]*np.sin(angles)
    ends = [center + ring + np.eye(3)[axis]*sign*solid["height"]/2 for sign in (-1, 1)]
    triangles = []
    for i in range(32):
        j = (i+1) % 32
        triangles.extend([[ends[0][i], ends[0][j], ends[1][j]],
                          [ends[0][i], ends[1][j], ends[1][i]]])
        triangles.extend([[end.mean(axis=0), end[i], end[j]] for end in ends])
    return np.asarray(triangles)


def component_triangles(component):
    parts = [np.asarray(mesh.points)[np.asarray(mesh.faces)] for mesh in component["meshes"].values()]
    parts.extend(primitive_triangles(solid) for solid in component["solids"] if solid.get("visual", True))
    return np.concatenate(parts)


def dish_triangles(kind):
    visuals = tableware.tableware_geometry(kind)["visuals"]
    return np.concatenate([np.asarray(points)[np.asarray(faces)] for points, _, faces in visuals])


def transformed_triangles(triangles, pose):
    return transform_vertices(triangles.reshape(-1, 3), pose["position_m"],
                              pose["quaternion_xyzw"]).reshape(-1, 3, 3)


def make_figure(run_dir, usd_path, output):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.colors import to_rgb
    from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 — registers 3D projection on older Matplotlib
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection

    metadata, summary, selected = load_run(run_dir, usd_path)
    components = geometry.build_components()
    racks = {rack: component_triangles(components[rack]) for rack in RACKS}
    basket = component_triangles(components["SilverwareBasket"])
    dishes = {kind: dish_triangles(kind) for kind in KINDS}
    colors = {"dinner_plate": "#2983ba", "bowl": "#d76635", "mug": "#17816d"}
    fig = plt.figure(figsize=(13, 15), facecolor="#f9fafb")
    details = []

    def add_mesh(ax, triangles, color, alpha=1.):
        normals = np.cross(triangles[:, 1]-triangles[:, 0], triangles[:, 2]-triangles[:, 0])
        lengths = np.linalg.norm(normals, axis=1)
        normals /= np.maximum(lengths[:, None], 1e-12)
        light = np.array([-.3, -.4, 1.])
        light /= np.linalg.norm(light)
        shade = .64 + .36*np.abs(normals @ light)
        rgba = np.column_stack((np.asarray(to_rgb(color))[None, :] * shade[:, None],
                                np.full(len(triangles), alpha)))
        collection = Poly3DCollection(triangles, facecolors=rgba, edgecolors="none", linewidths=0)
        ax.add_collection3d(collection)

    for row, kind in enumerate(KINDS):
        for col, rack in enumerate(RACKS):
            ax = fig.add_subplot(3, 2, row*2+col+1, projection="3d")
            ax.set_facecolor("#f9fafb")
            trial = selected[(kind, rack)]
            cell = next(cell for cell in summary["cells"] if cell["kind"] == kind and cell["rack"] == rack)
            title = kind.replace("_", " ").title() + " / " + ("lower rack" if rack == "LowerRack" else "upper rack")
            ax.set_title(title, fontsize=13, pad=8)
            if trial is None:
                ax.text2D(.5, .53, "No accepted samples", transform=ax.transAxes,
                          ha="center", va="center", fontsize=15, color="#677789")
                ax.text2D(.5, .43, f"0 accepted / {cell['attempted']} attempted", transform=ax.transAxes,
                          ha="center", va="center", fontsize=11, color="#677789")
                ax.set_axis_off()
                details.append({"kind": kind, "rack": rack, "trial_id": None})
                continue
            frames = trial["final_hold"]["poses"]
            rack_pose = frames[rack]
            add_mesh(ax, transformed_triangles(racks[rack], rack_pose), "#a6b0bb", .43)
            if rack == "LowerRack":
                add_mesh(ax, transformed_triangles(basket, frames["SilverwareBasket"]), "#77818c", .16)
            add_mesh(ax, transformed_triangles(dishes[kind], trial["final_pose"]), colors[kind])
            base_z = rack_pose["position_m"][2]
            ax.set_xlim(-.30, .30)
            ax.set_ylim(-.315, .33)
            ax.set_zlim(base_z-.04, min(.83, base_z+.36))
            if hasattr(ax, "set_box_aspect"):
                ax.set_box_aspect((.60, .645, min(.83, base_z+.36)-(base_z-.04)))
            ax.view_init(elev=35, azim=-61)
            # This is a pose illustration; numeric axes obscure the trial labels.
            # World coordinates remain available in the sidecar and run records.
            ax.set_axis_off()
            ax.text2D(.5, -.025, trial["trial_id"], transform=ax.transAxes,
                      ha="center", fontsize=10, color="#1c354a")
            ax.text2D(.5, -.07, f"Representative of {cell['accepted']} accepted / {cell['attempted']} attempted",
                      transform=ax.transAxes, ha="center", fontsize=9, color="#657385")
            details.append({"kind": kind, "rack": rack, "trial_id": trial["trial_id"],
                            "dish_pose": deepcopy(trial["final_pose"]), "rack_pose": deepcopy(rack_pose),
                            "dish_visual_triangles": len(dishes[kind]), "rack_visual_triangles": len(racks[rack])})
    fig.suptitle("Accepted random dish poses — reconstructed from saved Isaac final poses", fontsize=15, y=.975)
    fig.text(.5, .947, "One independent trial per panel • racks retracted • door remains open and is omitted",
             ha="center", fontsize=11, color="#536272")
    fig.text(.5, .022, "Static 3D illustration from matching generated visual meshes; no new simulation or Isaac render.\n"
             "Exact saved dish, rack and basket transforms. Shell and other rack omitted for visibility.\n"
             f"Run: {Path(run_dir).name} · first accepted trial per cell · world metres",
             ha="center", fontsize=10, color="#536272")
    fig.subplots_adjust(left=.03, right=.97, top=.91, bottom=.095, hspace=.24, wspace=.04)
    output = Path(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=150, facecolor=fig.get_facecolor())
    plt.close(fig)
    provenance = {"kind": "saved_pose_reconstruction", "new_simulation": False,
                  "isaac_render": False, "run_dir": str(Path(run_dir).resolve()),
                  "figure_sha256": digest(output), "script_sha256": digest(__file__),
                  "input_sha256": {name: digest(Path(run_dir)/name)
                                   for name in ("metadata.json", "summary.json", "accepted_poses.json")},
                  "geometry_source_sha256": digest(geometry.__file__),
                  "tableware_source_sha256": digest(tableware.__file__),
                  "asset_hashes": metadata["inputs"]["asset_hashes"],
                  "selection": "First accepted sample index in each dish/rack cell", "panels": details}
    output.with_suffix(".json").write_text(json.dumps(provenance, indent=2) + "\n")
    return output


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--run-dir", type=Path, required=True)
    parser.add_argument("--usd", type=Path, default=ROOT/"build/frigidaire_collection/usd/fdpc4221as.usdc")
    parser.add_argument("--out", type=Path, required=True, help="PNG output, with provenance JSON beside it")
    args = parser.parse_args()
    if args.out.suffix.lower() != ".png":
        parser.error("--out must end in .png")
    output = make_figure(args.run_dir, args.usd, args.out)
    print("[VISUALIZATION] " + str(output))


if __name__ == "__main__":
    main()
