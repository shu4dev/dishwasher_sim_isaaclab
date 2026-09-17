#!/usr/bin/env python3
# Copyright (c) 2026, dishsim project.
# SPDX-License-Identifier: BSD-3-Clause
"""Inspect generated lower-rack tines and the basket at its assembly pose.

Requires only NumPy and Matplotlib. Overhead PNG/SVG show generated wire paths,
all tine base centers, and a basket outline; the oblique PNG projects the actual
generated meshes and authored solids. No USD or Isaac validation is performed.
"""
import argparse
import hashlib
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.collections import PolyCollection
from matplotlib.colors import to_rgb
import numpy as np

from frigidaire_upper_rack_preview import (
    ROOT, INK, MUTED, ACCENT, PALE, PAPER, dimension, geometry, solid_triangles,
)
from dishsim_frigidaire.paths import IMAGE_DIR


BASKET = "#a46a32"


def measurements(rack, basket, basket_offset):
    teeth = [(name, np.asarray(path) * 1000, radius * 1000)
             for name, path, radius in rack["wires"]
             if name.startswith("TineBank") and "_Tooth" in name]
    if not teeth:
        raise ValueError("Generated lower rack has no tine paths")
    bases = np.asarray([path[0] for _, path, _ in teeth])
    tips = np.asarray([path[-1] for _, path, _ in teeth])
    xs, ys = np.unique(bases[:, 0]), np.unique(bases[:, 1])
    expected_x, expected_y = geometry.lower_tine_positions()
    if (len(teeth) != len(xs) * len(ys)
            or not np.allclose(xs, np.asarray(expected_x) * 1000)
            or not np.allclose(ys, np.asarray(expected_y) * 1000)):
        raise ValueError("Generated tine paths do not match the parameter grid")
    rim = [(np.asarray(path) * 1000, radius * 1000)
           for name, path, radius in rack["wires"] if name.startswith("UpperRim")]
    rim_min = np.min([path.min(axis=0) - radius for path, radius in rim], axis=0)
    rim_max = np.max([path.max(axis=0) + radius for path, radius in rim], axis=0)
    basket_paths = [(np.asarray(path) * 1000 + basket_offset * 1000, radius * 1000)
                    for _, path, radius in basket["wires"]]
    basket_min = np.min([path.min(axis=0) - r for path, r in basket_paths], axis=0)
    basket_max = np.max([path.max(axis=0) + r for path, r in basket_paths], axis=0)
    basket_origin = np.asarray(geometry.PARAMETERS["origins"]["SilverwareBasket"])
    report = {
        "model": geometry.PARAMETERS["model"],
        "component": "LowerRack with SilverwareBasket at its assembly pose",
        "status": "SOURCE_GEOMETRY_PREVIEW; not installed-USD or Isaac validation",
        "source_sha256": hashlib.sha256(Path(geometry.__file__).read_bytes()).hexdigest(),
        "units": "mm",
        "coordinate_system": "lower-rack-local; X left to right, -Y front, +Y rear, Z up",
        "datum": "tine base centers to outer wire-rim edges; pitch is center to center",
        "columns_left_to_right": len(xs),
        "rows_front_to_back": len(ys),
        "tine_count": len(teeth),
        "wire_rim_width": float(rim_max[0] - rim_min[0]),
        "wire_rim_depth": float(rim_max[1] - rim_min[1]),
        "column_x": xs.tolist(),
        "row_y_front_to_back": ys.tolist(),
        "column_pitches_left_to_right": np.diff(xs).tolist(),
        "row_pitches_front_to_back": np.diff(ys).tolist(),
        "margins": {"left": float(xs[0] - rim_min[0]),
                    "right": float(rim_max[0] - xs[-1]),
                    "rear": float(rim_max[1] - ys[-1]),
                    "front": float(ys[0] - rim_min[1])},
        "tine_diameters": sorted(set(2 * r for _, _, r in teeth)),
        "tine_vertical_rises": np.unique(np.round(tips[:, 2] - bases[:, 2], 8)).tolist(),
        "tine_rightward_leans": np.unique(np.round(tips[:, 0] - bases[:, 0], 8)).tolist(),
        "right_tip_center_margin": float(rim_max[0] - tips[:, 0].max()),
        "basket": {"assembly_origin": (basket_origin * 1000).tolist(),
                   "lower_rack_relative_origin": (basket_offset * 1000).tolist(),
                   "right_shift_from_previous_assembly": float((basket_origin[0] - .186) * 1000),
                   "capsule_bounds_min": basket_min.tolist(),
                   "capsule_bounds_max": basket_max.tolist(),
                   "overhead_display": "generated top/bottom rim and handle outlines only; lattice omitted for tine readability"},
        "tines": [{"name": name, "base_center": path[0].tolist(),
                   "tip_center": path[-1].tolist(), "diameter": 2 * radius}
                  for name, path, radius in teeth],
    }
    return report, bases, rim_min, rim_max


def overhead(rack, basket, basket_offset, report, bases, rim_min, rim_max, out_dir):
    fig, ax = plt.subplots(figsize=(11, 11.7), facecolor=PAPER)
    fig.subplots_adjust(left=.04, right=.98, top=.90, bottom=.11)
    ax.set_facecolor(PAPER)
    ax.set_xlim(-390, 380)
    ax.set_ylim(-390, 420)
    ax.set_aspect("equal")
    ax.axis("off")
    fig.canvas.draw()
    points_per_mm = ax.get_window_extent().width / 770 * 72 / fig.dpi
    for name, path, radius in rack["wires"]:
        path = np.asarray(path) * 1000
        is_tine = name.startswith("TineBank")
        ax.plot(path[:, 0], path[:, 1], color=ACCENT if is_tine else PALE,
                lw=2 * radius * 1000 * points_per_mm, solid_capstyle="round",
                zorder=3 if is_tine else 1)
    for solid in rack["solids"]:
        ax.add_collection(PolyCollection(solid_triangles(solid)[:, :, :2],
                                         facecolors=MUTED, edgecolors="none", zorder=2))
    for name, path, radius in basket["wires"]:
        if not name.startswith(("TopRim", "BottomRim", "HandleUpper")):
            continue
        path = (np.asarray(path) + basket_offset) * 1000
        ax.plot(path[:, 0], path[:, 1], color=BASKET,
                lw=2 * radius * 1000 * points_per_mm, solid_capstyle="round", zorder=4)
    ax.scatter(bases[:, 0], bases[:, 1], s=19, c=ACCENT, edgecolors=PAPER,
               linewidths=.5, zorder=6)
    xs, ys = report["column_x"], report["row_y_front_to_back"]
    for i, x in enumerate(xs):
        ax.text(x, 317, "%02d" % (i + 1), ha="center", color=ACCENT,
                fontsize=8, weight="bold")
    for i, y in enumerate(ys):
        ax.text(xs[0] - 17, y, "R%d" % (i + 1), ha="right", va="center",
                color=ACCENT, fontsize=9, weight="bold",
                bbox={"facecolor": PAPER, "edgecolor": "none", "pad": 1})
    pitch_i = len(xs) // 2 - 1
    for x in xs[pitch_i:pitch_i + 2]:
        ax.plot([x, x], [328, 356], color=PALE, lw=.7)
    dimension(ax, (xs[pitch_i], 347), (xs[pitch_i + 1], 347),
              "%.4f mm pitch × %d gaps" % (xs[pitch_i + 1] - xs[pitch_i], len(xs) - 1),
              (0, 22))
    dimension(ax, (rim_min[0], 407), (rim_max[0], 407),
              "%g mm outer rim" % report["wire_rim_width"])
    for x in [rim_min[0], rim_max[0]]:
        ax.plot([x, x], [rim_max[1] + 8, 414], color=PALE, lw=.7)
    dimension(ax, (-350, rim_min[1]), (-350, rim_max[1]),
              "%g mm outer rim" % report["wire_rim_depth"], rotation=90)
    for y in [rim_min[1], rim_max[1]]:
        ax.plot([-360, rim_min[0] - 8], [y, y], color=PALE, lw=.7)
    for a, b in [(rim_min[0], xs[0]), (xs[-1], rim_max[0])]:
        dimension(ax, (a, -232), (b, -232), "%g mm" % (b - a))
    for y in [rim_min[1], ys[0], ys[-1], rim_max[1]]:
        ax.plot([rim_max[0] + 8, 324], [y, y], color=PALE, lw=.7)
    dimension(ax, (313, rim_min[1]), (313, ys[0]),
              "%g mm front" % report["margins"]["front"], rotation=90)
    dimension(ax, (313, ys[-1]), (313, rim_max[1]),
              "%g mm rear" % report["margins"]["rear"], rotation=90)
    dimension(ax, (-260, ys[1]), (-260, ys[2]),
              "%.3f mm pitch" % (ys[2] - ys[1]), (-30, 0), rotation=90)
    ax.text(basket_offset[0] * 1000, basket_offset[1] * 1000,
            "BASKET · +%g mm RIGHT" % report["basket"]["right_shift_from_previous_assembly"],
            ha="center", va="center", rotation=90, color=BASKET, fontsize=9,
            bbox={"facecolor": PAPER, "edgecolor": "none", "pad": 2}, zorder=7)
    ax.text(0, -326, "FRONT  /  −Y", ha="center", color=INK,
            fontsize=11, weight="bold")
    ax.text(302, 311, "REAR / +Y", ha="center", color=MUTED, fontsize=8)
    ax.text(0, -359, "●  Tine base center     |     Rows numbered front to back",
            ha="center", color=ACCENT, fontsize=10)
    fig.text(.065, .963, "LOWER RACK · TINE LAYOUT", fontsize=19, color=INK,
             weight="bold", va="top")
    fig.text(.065, .931, "%d columns across × %d rows deep = %d tines · basket at assembly pose" % (
        report["columns_left_to_right"], report["rows_front_to_back"], report["tine_count"]),
        fontsize=11, color=MUTED, va="top")
    fig.text(.065, .069, "All dimensions: mm. Margins: tine base center to outer wire-rim edge.",
             fontsize=9, color=INK)
    fig.text(.065, .047, "Basket shown as generated rim/handle outline so all tine bases remain visible.",
             fontsize=9, color=BASKET)
    fig.text(.065, .026, "Generated source geometry · not installed-USD inspection or Isaac validation",
             fontsize=8, color=MUTED)
    for extension in ["png", "svg"]:
        fig.savefig(out_dir / ("lower_rack_overhead." + extension), dpi=180, facecolor=PAPER)
    plt.close(fig)


def oblique(rack, basket, basket_offset, report, destination):
    view = np.asarray([1., -1.4, 1.4])
    view /= np.linalg.norm(view)
    right = np.cross([0., 0., 1.], view)
    right /= np.linalg.norm(right)
    projection = np.stack([right, np.cross(view, right), view], axis=1)
    triangles, colors = [], []
    for component, offset, is_basket in [(rack, np.zeros(3), False),
                                         (basket, basket_offset, True)]:
        for name, mesh in component["meshes"].items():
            faces = (np.asarray(mesh.points)[np.asarray(mesh.faces)] + offset) * 1000
            triangles.append(faces)
            color = BASKET if is_basket else ACCENT if name.startswith("TineBank") else "#a5b4bf"
            colors.extend([to_rgb(color)] * len(faces))
        for solid in component["solids"]:
            faces = solid_triangles(solid) + offset * 1000
            triangles.append(faces)
            colors.extend([to_rgb(BASKET if is_basket else "#354958")] * len(faces))
    triangles = np.concatenate(triangles)
    normals = np.cross(triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0])
    normals /= np.maximum(np.linalg.norm(normals, axis=1)[:, None], 1e-12)
    light = np.asarray([-.4, -.6, 1.])
    light /= np.linalg.norm(light)
    colors = np.asarray(colors) * (.62 + .38 * np.abs(normals @ light))[:, None]
    projected = triangles @ projection
    order = np.argsort(projected[:, :, 2].mean(axis=1), kind="stable")
    fig, ax = plt.subplots(figsize=(12, 8.5), facecolor=PAPER)
    fig.subplots_adjust(left=.04, right=.98, top=.86, bottom=.12)
    ax.set_facecolor(PAPER)
    ax.add_collection(PolyCollection(projected[order, :, :2], facecolors=colors[order],
                                     edgecolors="none", antialiaseds=False))
    low, high = projected[:, :, :2].min(axis=(0, 1)), projected[:, :, :2].max(axis=(0, 1))
    ax.set_xlim(low[0] - 25, high[0] + 25)
    ax.set_ylim(low[1] - 20, high[1] + 20)
    ax.set_aspect("equal")
    ax.axis("off")
    fig.text(.065, .955, "LOWER RACK · BASKET ASSEMBLY VIEW", fontsize=18,
             color=INK, weight="bold", va="top")
    fig.text(.065, .907, "Generated lower rack and basket meshes · %d tines · basket shifted %g mm right" % (
        report["tine_count"], report["basket"]["right_shift_from_previous_assembly"]),
             fontsize=11, color=MUTED, va="top")
    fig.text(.065, .066, "Teal: tines/base rails   ·   Gray: existing rack wires   ·   Bronze: unchanged basket shape",
             fontsize=9, color=INK)
    fig.text(.065, .036, "Orthographic source-geometry projection at assembly-relative poses · no Isaac or installed-USD validation",
             fontsize=8, color=MUTED)
    fig.savefig(destination, dpi=180, facecolor=PAPER)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out-dir", type=Path,
                        default=IMAGE_DIR / "lower_rack")
    args = parser.parse_args()
    rack, basket = geometry._lower_rack(), geometry._basket()
    origins = geometry.PARAMETERS["origins"]
    basket_offset = np.asarray(origins["SilverwareBasket"]) - np.asarray(origins["LowerRack"])
    report, bases, rim_min, rim_max = measurements(rack, basket, basket_offset)
    args.out_dir.mkdir(parents=True, exist_ok=True)
    overhead(rack, basket, basket_offset, report, bases, rim_min, rim_max, args.out_dir)
    oblique(rack, basket, basket_offset, report, args.out_dir / "lower_rack_oblique.png")
    report["artifacts"] = ["lower_rack_overhead.png", "lower_rack_overhead.svg",
                           "lower_rack_oblique.png"]
    report["artifact_sha256"] = {name: hashlib.sha256((args.out_dir / name).read_bytes()).hexdigest()
                                 for name in report["artifacts"]}
    (args.out_dir / "measurements.json").write_text(json.dumps(report, indent=2) + "\n")
    print("[RESULT] GENERATED: %d tines; previews and measurements in %s" % (
        report["tine_count"], args.out_dir))


if __name__ == "__main__":
    main()
