#!/usr/bin/env python3
# Copyright (c) 2026, dishsim project.
# SPDX-License-Identifier: BSD-3-Clause
"""Inspect the generated upper rack with NumPy/Matplotlib, without Isaac or USD.

Writes an annotated overhead PNG/SVG, front and oblique mesh projections, and a
measurement JSON. Measurements use generated tine base centers and the outer
wire-rim envelope. These are source-geometry previews, not simulation evidence.
"""
import argparse
import hashlib
import json
from pathlib import Path
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.collections import PolyCollection
from matplotlib.colors import to_rgb
import numpy as np

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "frigidaire/src"))
from dishsim_frigidaire.paths import IMAGE_DIR
from dishsim_frigidaire import geometry


INK = "#23384b"
MUTED = "#66798a"
ACCENT = "#007f83"
PALE = "#c1ccd3"
PAPER = "#fbfcfd"


def measurements(component):
    """Measure paths actually generated, and check their parameter alignment."""
    teeth = [(name, np.asarray(path) * 1000, radius * 1000)
             for name, path, radius in component["wires"]
             if name.startswith("BowlComb") and "_Tooth" in name]
    if not teeth:
        raise ValueError("Generated upper rack has no tine paths")
    bases = np.asarray([path[0] for _, path, _ in teeth])
    tips = np.asarray([path[-1] for _, path, _ in teeth])
    xs, ys = np.unique(bases[:, 0]), np.unique(bases[:, 1])
    expected_x, expected_y = geometry.upper_tine_positions()
    if (len(teeth) != len(xs) * len(ys)
            or not np.allclose(xs, np.asarray(expected_x) * 1000)
            or not np.allclose(ys, np.asarray(expected_y) * 1000)):
        raise ValueError("Generated tine paths do not match the parameter grid")
    rim = [(np.asarray(path) * 1000, radius * 1000)
           for name, path, radius in component["wires"] if name.startswith("TopRim")]
    rim_min = np.min([path.min(axis=0) - radius for path, radius in rim], axis=0)
    rim_max = np.max([path.max(axis=0) + radius for path, radius in rim], axis=0)
    report = {
        "model": geometry.PARAMETERS["model"],
        "component": "UpperRack",
        "status": "SOURCE_GEOMETRY_PREVIEW; not installed-USD or Isaac validation",
        "source_sha256": hashlib.sha256(Path(geometry.__file__).read_bytes()).hexdigest(),
        "units": "mm",
        "coordinate_system": "rack-local; X left to right, -Y front, +Y rear, Z up",
        "datum": "tine base centers to outer wire-rim edges; pitch is center to center",
        "columns_left_to_right": len(xs),
        "positions_front_to_back": len(ys),
        "tine_count": len(teeth),
        "wire_rim_width": float(rim_max[0] - rim_min[0]),
        "wire_rim_depth": float(rim_max[1] - rim_min[1]),
        "column_x": xs.tolist(),
        "row_y_front_to_back": ys.tolist(),
        "column_gaps_left_to_right": np.diff(xs).tolist(),
        "row_pitches_front_to_back": np.diff(ys).tolist(),
        "margins": {"left": float(xs[0] - rim_min[0]),
                    "right": float(rim_max[0] - xs[-1]),
                    "rear": float(rim_max[1] - ys[-1]),
                    "front": float(ys[0] - rim_min[1])},
        "tine_diameters": sorted(set(2 * r for _, _, r in teeth)),
        "tine_vertical_rises": np.unique(np.round(tips[:, 2] - bases[:, 2], 8)).tolist(),
        "tine_rearward_leans": np.unique(np.round(tips[:, 1] - bases[:, 1], 8)).tolist(),
        "rear_tip_center_margin": float(rim_max[1] - tips[:, 1].max()),
        "tines": [{"name": name, "base_center": path[0].tolist(),
                   "tip_center": path[-1].tolist(), "diameter": 2 * radius}
                  for name, path, radius in teeth],
    }
    return report, bases, rim_min, rim_max


def dimension(ax, start, end, label, text_offset=(0, 0), rotation=0):
    ax.annotate("", xy=end, xytext=start,
                arrowprops={"arrowstyle": "<->", "color": INK, "lw": .85,
                            "shrinkA": 0, "shrinkB": 0}, zorder=8)
    middle = (np.asarray(start) + end) / 2 + text_offset
    ax.text(*middle, label, ha="center", va="center", color=INK,
            fontsize=9, rotation=rotation, zorder=9,
            bbox={"facecolor": PAPER, "edgecolor": "none", "pad": 2})


def solid_triangles(solid):
    """Tessellate authored primitives for orthographic inspection only."""
    center = np.asarray(solid["center"])
    if solid["kind"] == "box":
        corners = np.asarray([[x, y, z] for x in [-1, 1] for y in [-1, 1]
                              for z in [-1, 1]])
        points = center + corners * np.asarray(solid["size"]) / 2
        faces = [[0, 1, 3], [0, 3, 2], [4, 6, 7], [4, 7, 5],
                 [0, 4, 5], [0, 5, 1], [2, 3, 7], [2, 7, 6],
                 [0, 2, 6], [0, 6, 4], [1, 5, 7], [1, 7, 3]]
        return points[np.asarray(faces)] * 1000
    if solid["kind"] != "cylinder":
        raise ValueError("Unsupported primitive: " + solid["kind"])
    axis = "XYZ".index(solid["axis"])
    radial = [i for i in range(3) if i != axis]
    angles = np.arange(32) * (2 * np.pi / 32)
    ring = np.zeros((32, 3))
    ring[:, radial[0]] = solid["radius"] * np.cos(angles)
    ring[:, radial[1]] = solid["radius"] * np.sin(angles)
    rings = []
    for sign in [-1, 1]:
        points = center + ring
        points[:, axis] += sign * solid["height"] / 2
        rings.append(points)
    faces = []
    for i in range(32):
        j = (i + 1) % 32
        faces.extend([[rings[0][i], rings[0][j], rings[1][j]],
                      [rings[0][i], rings[1][j], rings[1][i]]])
        for points in rings:
            faces.append([points.mean(axis=0), points[i], points[j]])
    return np.asarray(faces) * 1000


def overhead(component, report, bases, rim_min, rim_max, out_dir):
    fig, ax = plt.subplots(figsize=(10, 11), facecolor=PAPER)
    fig.subplots_adjust(left=.04, right=.98, top=.90, bottom=.105)
    ax.set_facecolor(PAPER)
    ax.set_xlim(-365, 345)
    ax.set_ylim(-365, 390)
    ax.set_aspect("equal")
    ax.axis("off")
    fig.canvas.draw()
    points_per_mm = ax.get_window_extent().width / 710 * 72 / fig.dpi
    for name, path, radius in component["wires"]:
        path = np.asarray(path) * 1000
        is_tine = name.startswith("BowlComb")
        ax.plot(path[:, 0], path[:, 1], color=ACCENT if is_tine else PALE,
                lw=2 * radius * 1000 * points_per_mm, solid_capstyle="round",
                zorder=3 if is_tine else 1)
    for solid in component["solids"]:
        triangles = solid_triangles(solid)
        ax.add_collection(PolyCollection(triangles[:, :, :2], facecolors=MUTED,
                                         edgecolors="none", zorder=2))
    ax.scatter(bases[:, 0], bases[:, 1], s=18, c=ACCENT, edgecolors=PAPER,
               linewidths=.5, zorder=5)
    xs = report["column_x"]
    ys = report["row_y_front_to_back"]
    for i, x in enumerate(xs):
        ax.plot([x, x], [rim_max[1] + 10, 350], color=PALE, lw=.7)
        ax.text(x, 303, "C%d" % (i + 1), ha="center", color=ACCENT,
                fontsize=10, weight="bold")
    for i, y in enumerate(ys):
        ax.text(xs[-1] + 17, y, "%02d" % (i + 1), ha="left", va="center",
                color=ACCENT, fontsize=7.5,
                bbox={"facecolor": PAPER, "edgecolor": "none", "pad": .5})
    for a, b in zip(xs[:-1], xs[1:]):
        dimension(ax, (a, 340), (b, 340), "%g mm" % (b - a), (0, 0))
    dimension(ax, (rim_min[0], 377), (rim_max[0], 377),
              "%g mm outer rim" % report["wire_rim_width"])
    for x in [rim_min[0], rim_max[0]]:
        ax.plot([x, x], [rim_max[1] + 8, 384], color=PALE, lw=.7)
    dimension(ax, (-326, rim_min[1]), (-326, rim_max[1]),
              "%g mm outer rim" % report["wire_rim_depth"], rotation=90)
    for y in [rim_min[1], rim_max[1]]:
        ax.plot([-336, rim_min[0] - 8], [y, y], color=PALE, lw=.7)
    for a, b in [(rim_min[0], xs[0]), (xs[-1], rim_max[0])]:
        dimension(ax, (a, -214), (b, -214), "%g mm" % (b - a))
    for y in [rim_min[1], ys[0], ys[-1], rim_max[1]]:
        ax.plot([rim_max[0] + 8, 303], [y, y], color=PALE, lw=.7)
    dimension(ax, (291, rim_min[1]), (291, ys[0]),
              "%.2f mm" % report["margins"]["front"], rotation=90)
    dimension(ax, (291, ys[-1]), (291, rim_max[1]),
              "%g mm" % report["margins"]["rear"], (22, 0), rotation=90)
    pitch_i = len(ys) // 2
    dimension(ax, (200, ys[pitch_i - 1]), (200, ys[pitch_i]),
              "%g mm pitch" % (ys[pitch_i] - ys[pitch_i - 1]), (54, 0))
    ax.text(0, -312, "FRONT  /  −Y", ha="center", color=INK,
            fontsize=11, weight="bold")
    ax.text(280, 279, "REAR / +Y", ha="center", color=MUTED, fontsize=8)
    ax.text(0, -343, "●  Tine base center     |     Rows numbered front to back",
            ha="center", color=ACCENT, fontsize=10)
    fig.text(.065, .963, "UPPER RACK · TINE LAYOUT", fontsize=19, color=INK,
             weight="bold", va="top")
    fig.text(.065, .931, "%d columns across × %d positions deep = %d tines" % (
        report["columns_left_to_right"], report["positions_front_to_back"],
        report["tine_count"]), fontsize=12, color=MUTED, va="top")
    fig.text(.065, .063, "All dimensions: mm. Margins: tine base center to outer wire-rim edge.",
             fontsize=9, color=INK)
    fig.text(.065, .042, "Generated wire paths and authored wheel solids · source geometry preview · no Isaac validation",
             fontsize=8, color=MUTED)
    for extension in ["png", "svg"]:
        fig.savefig(out_dir / ("upper_rack_overhead." + extension), dpi=180,
                    facecolor=PAPER)
    plt.close(fig)


def mesh_view(component, view, title, subtitle, destination):
    """Project actual generated triangles, sorted in orthographic camera depth."""
    view = np.asarray(view, dtype=float)
    view /= np.linalg.norm(view)
    right = np.cross([0., 0., 1.], view)
    right /= np.linalg.norm(right)
    up = np.cross(view, right)
    projection = np.stack([right, up, view], axis=1)
    triangles, colors = [], []
    for name, mesh in component["meshes"].items():
        faces = np.asarray(mesh.points)[np.asarray(mesh.faces)] * 1000
        triangles.append(faces)
        color = to_rgb(ACCENT if name.startswith("BowlComb") else "#a5b4bf")
        colors.extend([color] * len(faces))
    for solid in component["solids"]:
        faces = solid_triangles(solid)
        triangles.append(faces)
        colors.extend([to_rgb("#354958")] * len(faces))
    triangles = np.concatenate(triangles)
    normals = np.cross(triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0])
    normals /= np.maximum(np.linalg.norm(normals, axis=1)[:, None], 1e-12)
    light = np.asarray([-.4, -.6, 1.])
    light /= np.linalg.norm(light)
    shade = .62 + .38 * np.abs(normals @ light)
    colors = np.asarray(colors) * shade[:, None]
    projected = triangles @ projection
    order = np.argsort(projected[:, :, 2].mean(axis=1), kind="stable")
    fig, ax = plt.subplots(figsize=(12, 6.8), facecolor=PAPER)
    fig.subplots_adjust(left=.04, right=.98, top=.85, bottom=.13)
    ax.set_facecolor(PAPER)
    ax.add_collection(PolyCollection(projected[order, :, :2], facecolors=colors[order],
                                     edgecolors="none", antialiaseds=False))
    low = projected[:, :, :2].min(axis=(0, 1))
    high = projected[:, :, :2].max(axis=(0, 1))
    ax.set_xlim(low[0] - 25, high[0] + 25)
    ax.set_ylim(low[1] - 20, high[1] + 20)
    ax.set_aspect("equal")
    ax.axis("off")
    fig.text(.065, .955, title, fontsize=18, color=INK, weight="bold", va="top")
    fig.text(.065, .902, subtitle, fontsize=11, color=MUTED, va="top")
    fig.text(.065, .065, "Teal: revised tines and base rails   ·   Gray: existing rack wires   ·   Dark: wheels and fittings",
             fontsize=9, color=INK)
    fig.text(.065, .035, "Orthographic projection of generated meshes and authored solids · no simulation or installed-USD claim",
             fontsize=8, color=MUTED)
    fig.savefig(destination, dpi=180, facecolor=PAPER)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out-dir", type=Path,
                        default=IMAGE_DIR / "upper_rack")
    args = parser.parse_args()
    component = geometry._upper_rack()
    report, bases, rim_min, rim_max = measurements(component)
    args.out_dir.mkdir(parents=True, exist_ok=True)
    overhead(component, report, bases, rim_min, rim_max, args.out_dir)
    mesh_view(component, [0, -1, 0], "UPPER RACK · FRONT VIEW",
              "Looking from −Y: four tine columns, connected rails, and contoured floor channels",
              args.out_dir / "upper_rack_front.png")
    mesh_view(component, [1, -1.35, 1.15], "UPPER RACK · OBLIQUE VIEW",
              "Generated rounded wires and tips; tine rails follow the actual floor profile",
              args.out_dir / "upper_rack_oblique.png")
    report["artifacts"] = ["upper_rack_overhead.png", "upper_rack_overhead.svg",
                           "upper_rack_front.png", "upper_rack_oblique.png"]
    report["artifact_sha256"] = {name: hashlib.sha256((args.out_dir / name).read_bytes()).hexdigest()
                                 for name in report["artifacts"]}
    (args.out_dir / "measurements.json").write_text(json.dumps(report, indent=2) + "\n")
    print("[RESULT] GENERATED: %d tines; previews and measurements in %s" % (
        report["tine_count"], args.out_dir))


if __name__ == "__main__":
    main()
