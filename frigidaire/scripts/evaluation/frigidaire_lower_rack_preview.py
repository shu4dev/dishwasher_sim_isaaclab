#!/usr/bin/env python3
# Copyright (c) 2026, dishsim project.
# SPDX-License-Identifier: BSD-3-Clause
"""Inspect generated lower-rack tines and the tape-measured basket at its seat.

Requires only NumPy and Matplotlib. Overhead PNG/SVG show generated wire paths,
all tine base centers (and the bay positions the basket omits), and a basket
outline; the oblique PNG projects the actual generated meshes and authored
solids. Three further views show the basket component alone at its own origin.
No USD or Isaac validation is performed.
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
BASKET_OUTLINE_FAMILIES = ("TopRim", "TopLip", "BottomRim", "BottomReinforcement", "HandleUpper", "HandleLower")


def removed_tines():
    """Grid positions the basket bay omits, as (column, row, x, y) in mm."""
    xs, ys = geometry.lower_tine_positions()
    mask = geometry.lower_tine_mask()
    return [{"column": int(c), "row": int(r), "x": float(xs[c] * 1000), "y": float(ys[r] * 1000)}
            for r, c in zip(*np.nonzero(~mask))]


def _pitch(values):
    values = np.unique(np.round(np.asarray(values, dtype=float), 9))
    return float(np.diff(values).mean()) if len(values) > 1 else None


def basket_measurements(basket):
    """Measure the generated basket component at its own origin (mm)."""
    p = geometry.PARAMETERS["silverware_basket"]
    paths = [(name, np.asarray(path) * 1000, radius * 1000) for name, path, radius in basket["wires"]]
    low = np.min([path.min(axis=0) - r for _, path, r in paths], axis=0)
    high = np.max([path.max(axis=0) + r for _, path, r in paths], axis=0)
    handle = [(path, r) for name, path, r in paths if name.startswith("Handle")]
    handle_top = max(float(path[:, 2].max() + r) for path, r in handle)
    upper = next(path for name, path, _ in paths if name == "HandleUpper")

    def family_envelope(prefix):
        fam = [(path, r) for name, path, r in paths if name.startswith(prefix)]
        low = np.min([path.min(axis=0) - r for path, r in fam], axis=0)
        high = np.max([path.max(axis=0) + r for path, r in fam], axis=0)
        return (high - low).tolist()

    top_env, floor_env = family_envelope("TopRim"), family_envelope("BottomRim")
    lattice_r = p["wire_radii"]["lattice"] * 1000

    def family_axis(prefix, axis):
        return [path[0, axis] for name, path, _ in paths if name.startswith(prefix)]

    floor_cross = _pitch(family_axis("BottomCrossRib", 1))
    floor_long = _pitch(family_axis("BottomLongRib", 0))
    wall_upright = _pitch(family_axis("LongWall1_Upright", 1))
    wall_course = _pitch(family_axis("LongWall1_Course", 2))
    end_upright = _pitch(family_axis("EndWall1_Upright", 0))
    return {
        "geometry_revision": p["geometry_revision"],
        "tape_measured_body_mm": {"length_y": p["length_y"] * 1000, "width_x": p["width_x"] * 1000,
                                  "body_height": p["body_height"] * 1000, "handle_top_z": p["handle_top_z"] * 1000},
        "envelope_datum": p["envelope_datum"],
        "capsule_envelope_min": low.tolist(),
        "capsule_envelope_max": high.tolist(),
        "capsule_envelope_size": (high - low).tolist(),
        "handle_top": handle_top,
        "handle": {"style": p["handle"]["style"], "plane_x": float(upper[0, 0]), "feet_y_abs": float(abs(upper[0, 1])),
                   "flat_top_half_span": p["handle"]["flat_top_half_span"] * 1000, "top": handle_top},
        "top_rim_envelope_size": top_env[:2],
        "floor_envelope_size": floor_env[:2],
        "taper_ratios_floor_over_rim": [floor_env[0] / top_env[0], floor_env[1] / top_env[1]],
        "design": p["design"],
        "compartment_layout": p["compartment_layout"],
        "compartment_centres_xy": [[x * 1000, y * 1000] for x, y in p["compartment_centres_xy"]],
        "floor_aperture_estimate": {
            "cross_rib_pitch_y": floor_cross, "long_rib_pitch_x": floor_long,
            "lattice_wire_diameter": 2 * lattice_r,
            "clear_opening_x_by_y": [floor_long - 2 * lattice_r, floor_cross - 2 * lattice_r],
            "method": "rib pitch minus one lattice wire diameter; ribs are generated on the parameter counts"},
        "wall_aperture_estimate": {
            "long_wall_upright_pitch_y": wall_upright, "end_wall_upright_pitch_x": end_upright,
            "course_pitch_z": wall_course, "lattice_wire_diameter": 2 * lattice_r,
            "clear_opening_long_wall": [wall_upright - 2 * lattice_r, wall_course - 2 * lattice_r],
            "clear_opening_end_wall": [end_upright - 2 * lattice_r, wall_course - 2 * lattice_r]},
        "wire_radii": {key: value * 1000 for key, value in p["wire_radii"].items()},
        "seat": {"rack": p["seat"]["rack"], "origin_in_rack": [v * 1000 for v in p["seat"]["origin_in_rack_m"]],
                 "rule": p["seat"]["rule"]},
        "removed_tines": removed_tines(),
        "wire_count": len(paths),
    }


def measurements(rack, basket, basket_offset):
    p = geometry.PARAMETERS["lower_rack"]
    teeth = [(name, np.asarray(path) * 1000, radius * 1000)
             for name, path, radius in rack["wires"]
             if name.startswith("TineBank") and "_Tooth" in name]
    if not teeth:
        raise ValueError("Generated lower rack has no tine paths")
    bases = np.asarray([path[0] for _, path, _ in teeth])
    tips = np.asarray([path[-1] for _, path, _ in teeth])
    expected_x, expected_y = geometry.lower_tine_positions()
    expected_x, expected_y = np.asarray(expected_x) * 1000, np.asarray(expected_y) * 1000
    mask, heights = geometry.lower_tine_mask(), geometry.lower_tine_heights() * 1000
    xs, ys = np.unique(bases[:, 0]), np.unique(bases[:, 1])
    on_grid = np.isclose(bases[:, 0][:, None], expected_x[None, :]).any(axis=1).all()
    if (len(teeth) != int(mask.sum()) or not on_grid
            or not np.allclose(ys, expected_y)):
        raise ValueError("Generated tine paths do not match the parameter grid")
    heights_by_row = []
    for row, y in enumerate(ys):
        in_row = np.isclose(bases[:, 1], y)
        present = np.sort(bases[in_row, 0])
        wanted = expected_x[mask[row]]
        if len(present) != len(wanted) or not np.allclose(present, wanted):
            raise ValueError("Row %d tine positions do not match the parameter mask" % row)
        rises = np.unique(np.round(tips[in_row, 2] - bases[in_row, 2], 8))
        if len(rises) != 1 or not np.isclose(rises[0], heights[row]):
            raise ValueError("Row %d tine height does not match the parameters" % row)
        heights_by_row.append(float(rises[0]))
    rim = [(np.asarray(path) * 1000, radius * 1000)
           for name, path, radius in rack["wires"] if name.startswith("UpperRim")]
    rim_min = np.min([path.min(axis=0) - radius for path, radius in rim], axis=0)
    rim_max = np.max([path.max(axis=0) + radius for path, radius in rim], axis=0)
    basket_paths = [(np.asarray(path) * 1000 + basket_offset * 1000, radius * 1000)
                    for _, path, radius in basket["wires"]]
    basket_min = np.min([path.min(axis=0) - r for path, r in basket_paths], axis=0)
    basket_max = np.max([path.max(axis=0) + r for path, r in basket_paths], axis=0)
    basket_origin = np.asarray(geometry.PARAMETERS["origins"]["SilverwareBasket"])
    full_rows = [row for row in range(len(ys)) if mask[row].all()]
    report = {
        "model": geometry.PARAMETERS["model"],
        "component": "LowerRack with SilverwareBasket at its seat",
        "geometry_revision": p["geometry_revision"],
        "status": "SOURCE_GEOMETRY_PREVIEW; not installed-USD or Isaac validation",
        "source_sha256": hashlib.sha256(Path(geometry.__file__).read_bytes()).hexdigest(),
        "units": "mm",
        "coordinate_system": "lower-rack-local; X left to right, -Y front, +Y rear, Z up",
        "datum": "tine base centers to outer wire-rim edges; pitch is center to center",
        "columns_left_to_right": len(xs),
        "rows_front_to_back": len(ys),
        "tine_count": len(teeth),
        "tines_per_row": mask.sum(axis=1).tolist(),
        "full_rows": full_rows,
        "removed_tines": removed_tines(),
        "tine_heights_by_row": heights_by_row,
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
        "margin_status": "derived from the outer rim, tine counts and pitch; the right margin is measured on the full front rows",
        "derived_margins_mm": {key: value * 1000 for key, value in p["tine_margins"].items()},
        "user_margins_tape_mm": {key: value * 1000 for key, value in p["tine_margins_tape_m"].items()},
        "margin_derivation": p["tine_margin_derivation"],
        "tine_diameters": sorted(set(2 * r for _, _, r in teeth)),
        "tine_vertical_rises": np.unique(np.round(tips[:, 2] - bases[:, 2], 8)).tolist(),
        "tine_rightward_leans": np.unique(np.round(tips[:, 0] - bases[:, 0], 8)).tolist(),
        "right_tip_center_margin": float(rim_max[0] - tips[:, 0].max()),
        "basket": {"assembly_origin": (basket_origin * 1000).tolist(),
                   "lower_rack_relative_origin": (basket_offset * 1000).tolist(),
                   "capsule_bounds_min": basket_min.tolist(),
                   "capsule_bounds_max": basket_max.tolist(),
                   "overhead_display": "generated top/bottom rims and lip, handle loop and partition top edges only; lattice omitted for tine readability",
                   **basket_measurements(basket)},
        "tines": [{"name": name, "base_center": path[0].tolist(),
                   "tip_center": path[-1].tolist(), "diameter": 2 * radius}
                  for name, path, radius in teeth],
    }
    return report, bases, rim_min, rim_max


def _basket_outline(name):
    return name.startswith(BASKET_OUTLINE_FAMILIES) or (name.startswith("Partition") and name.endswith("_TopEdge"))


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
        if not _basket_outline(name):
            continue
        path = (np.asarray(path) + basket_offset) * 1000
        ax.plot(path[:, 0], path[:, 1], color=BASKET,
                lw=2 * radius * 1000 * points_per_mm, solid_capstyle="round", zorder=4)
    ax.scatter(bases[:, 0], bases[:, 1], s=19, c=ACCENT, edgecolors=PAPER,
               linewidths=.5, zorder=6)
    removed = report["removed_tines"]
    ax.scatter([t["x"] for t in removed], [t["y"] for t in removed], s=26, facecolors="none",
               edgecolors=ACCENT, linewidths=.9, zorder=6)
    xs, ys = report["column_x"], report["row_y_front_to_back"]
    for i, x in enumerate(xs):
        ax.text(x, 317, "%02d" % (i + 1), ha="center", color=ACCENT,
                fontsize=8, weight="bold")
    for i, y in enumerate(ys):
        ax.text(xs[0] - 17, y, "R%d · %g" % (i + 1, report["tine_heights_by_row"][i]), ha="right", va="center",
                color=ACCENT, fontsize=9, weight="bold",
                bbox={"facecolor": PAPER, "edgecolor": "none", "pad": 1})
    pitch_i = len(xs) // 2 - 1
    for x in xs[pitch_i:pitch_i + 2]:
        ax.plot([x, x], [328, 356], color=PALE, lw=.7)
    dimension(ax, (xs[pitch_i], 347), (xs[pitch_i + 1], 347),
              "%g mm pitch × %d gaps" % (xs[pitch_i + 1] - xs[pitch_i], len(xs) - 1),
              (0, 22))
    dimension(ax, (rim_min[0], 407), (rim_max[0], 407),
              "%g mm outer rim" % report["wire_rim_width"])
    for x in [rim_min[0], rim_max[0]]:
        ax.plot([x, x], [rim_max[1] + 8, 414], color=PALE, lw=.7)
    dimension(ax, (-350, rim_min[1]), (-350, rim_max[1]),
              "%g mm outer rim" % report["wire_rim_depth"], rotation=90)
    for y in [rim_min[1], rim_max[1]]:
        ax.plot([-360, rim_min[0] - 8], [y, y], color=PALE, lw=.7)
    tape = report["user_margins_tape_mm"]
    for a, b, side in [(rim_min[0], xs[0], "left"), (xs[-1], rim_max[0], "right")]:
        dimension(ax, (a, -232), (b, -232), "%.2f mm\n(derived; tape %g)" % (b - a, tape[side]))
    for y in [rim_min[1], ys[0], ys[-1], rim_max[1]]:
        ax.plot([rim_max[0] + 8, 324], [y, y], color=PALE, lw=.7)
    dimension(ax, (313, rim_min[1]), (313, ys[0]),
              "%.2f mm front (derived; tape %g)" % (report["margins"]["front"], tape["front"]), rotation=90)
    dimension(ax, (313, ys[-1]), (313, rim_max[1]),
              "%.2f mm rear (derived; tape %g)" % (report["margins"]["rear"], tape["rear"]), rotation=90)
    dimension(ax, (-260, ys[1]), (-260, ys[2]),
              "%g mm pitch" % (ys[2] - ys[1]), (-30, 0), rotation=90)
    body = report["basket"]["tape_measured_body_mm"]
    ax.text(basket_offset[0] * 1000, basket_offset[1] * 1000,
            "BASKET %g×%g" % (body["length_y"], body["width_x"]),
            ha="center", va="center", rotation=90, color=BASKET, fontsize=9,
            bbox={"facecolor": PAPER, "edgecolor": "none", "pad": 2}, zorder=7)
    ax.text(0, -326, "FRONT  /  −Y", ha="center", color=INK,
            fontsize=11, weight="bold")
    ax.text(238, 311, "REAR / +Y", ha="center", color=MUTED, fontsize=8)
    ax.text(0, -359, "●  Tine base center     ○  Omitted bay position     |     R# · tine height mm, rows numbered front to back",
            ha="center", color=ACCENT, fontsize=9.5)
    fig.text(.065, .963, "LOWER RACK · TINE LAYOUT", fontsize=19, color=INK,
             weight="bold", va="top")
    fig.text(.065, .931, "%d columns across × %d rows deep = %d tines (%d bay positions omitted) · tape-measured basket at its seat" % (
        report["columns_left_to_right"], report["rows_front_to_back"], report["tine_count"], len(removed)),
        fontsize=11, color=MUTED, va="top")
    fig.text(.065, .069, "All dimensions: mm. Margins: tine base center to outer wire-rim edge, "
             "derived from the tape outer size, counts and pitch (tape margins in brackets).",
             fontsize=9, color=INK)
    fig.text(.065, .047, "Basket shown as its generated rim, handle and partition outline so all tine bases remain visible.",
             fontsize=9, color=BASKET)
    fig.text(.065, .026, "Generated source geometry · not installed-USD inspection or Isaac validation",
             fontsize=8, color=MUTED)
    for extension in ["png", "svg"]:
        fig.savefig(out_dir / ("lower_rack_overhead." + extension), dpi=180, facecolor=PAPER)
    plt.close(fig)


def mesh_view(parts, view, title, subtitle, legend, destination, figsize=(12, 8.5)):
    """Orthographic projection of generated meshes and authored solids.

    parts: (component, offset_m, is_basket) triples; view: direction toward the camera.
    """
    view = np.asarray(view, dtype=float)
    view /= np.linalg.norm(view)
    right = np.cross([0., 0., 1.], view)
    if np.linalg.norm(right) < 1e-9:      # straight down: keep +X to the right
        right = np.asarray([1., 0., 0.])
    right /= np.linalg.norm(right)
    projection = np.stack([right, np.cross(view, right), view], axis=1)
    triangles, colors = [], []
    for component, offset, is_basket in parts:
        offset = np.asarray(offset, dtype=float)
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
    fig, ax = plt.subplots(figsize=figsize, facecolor=PAPER)
    fig.subplots_adjust(left=.04, right=.98, top=.86, bottom=.12)
    ax.set_facecolor(PAPER)
    ax.add_collection(PolyCollection(projected[order, :, :2], facecolors=colors[order],
                                     edgecolors="none", antialiaseds=False))
    low, high = projected[:, :, :2].min(axis=(0, 1)), projected[:, :, :2].max(axis=(0, 1))
    ax.set_xlim(low[0] - 25, high[0] + 25)
    ax.set_ylim(low[1] - 20, high[1] + 20)
    ax.set_aspect("equal")
    ax.axis("off")
    fig.text(.065, .955, title, fontsize=18, color=INK, weight="bold", va="top")
    fig.text(.065, .907, subtitle, fontsize=11, color=MUTED, va="top")
    fig.text(.065, .066, legend, fontsize=9, color=INK)
    fig.text(.065, .036, "Orthographic source-geometry projection · no Isaac or installed-USD validation",
             fontsize=8, color=MUTED)
    fig.savefig(destination, dpi=180, facecolor=PAPER)
    plt.close(fig)


def oblique(rack, basket, basket_offset, report, destination):
    body = report["basket"]["tape_measured_body_mm"]
    mesh_view([(rack, np.zeros(3), False), (basket, basket_offset, True)], [1., -1.4, 1.4],
              "LOWER RACK · BASKET ASSEMBLY VIEW",
              "Generated lower rack and basket meshes · %d tines (%d bay positions omitted) · basket seated at [%g, %g, %g] mm" % (
                  report["tine_count"], len(report["removed_tines"]), *report["basket"]["lower_rack_relative_origin"]),
              "Teal: tines/base rails   ·   Gray: existing rack wires   ·   Bronze: tape-measured %g x %g x %g mm basket" % (
                  body["length_y"], body["width_x"], body["body_height"]),
              destination)


def basket_views(basket, report, out_dir):
    body = report["basket"]["tape_measured_body_mm"]
    size = report["basket"]["capsule_envelope_size"]
    legend = "Bronze: generated basket wires · envelope %.1f x %.1f x %.1f mm (X x Y x Z) · handle top %.1f mm" % (
        size[0], size[1], size[2], report["basket"]["handle_top"])
    subtitle = "Tape-measured %g x %g x %g mm body · loop handle to %g mm · %s compartments\nBasket origin at the bottom-face center; X across the width, Y along the length, Z up" % (
        body["length_y"], body["width_x"], body["body_height"], body["handle_top_z"], report["basket"]["compartment_layout"])
    for name, view, title in [("basket_front.png", [0., -1., 0.], "SILVERWARE BASKET · FRONT VIEW (from −Y)"),
                              ("basket_top.png", [0., 0., 1.], "SILVERWARE BASKET · TOP VIEW (from +Z)"),
                              ("basket_oblique.png", [1., -1.4, 1.1], "SILVERWARE BASKET · OBLIQUE VIEW")]:
        mesh_view([(basket, np.zeros(3), True)], view, title, subtitle, legend, out_dir / name, figsize=(9, 7.5))


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
    basket_views(basket, report, args.out_dir)
    report["artifacts"] = ["lower_rack_overhead.png", "lower_rack_overhead.svg",
                           "lower_rack_oblique.png", "basket_front.png", "basket_top.png",
                           "basket_oblique.png"]
    report["artifact_sha256"] = {name: hashlib.sha256((args.out_dir / name).read_bytes()).hexdigest()
                                 for name in report["artifacts"]}
    (args.out_dir / "measurements.json").write_text(json.dumps(report, indent=2) + "\n")
    print("[RESULT] GENERATED: %d tines; previews and measurements in %s" % (
        report["tine_count"], args.out_dir))


if __name__ == "__main__":
    main()
