# Copyright (c) 2026, dishsim project.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Rack layout arithmetic the slot derivation and the capacity planner read (numpy only).

Verbatim from the rack generator (``rack_gen.py``, moved to ``data/assets/bosch800/source/`` on 2026-10-21 with the
rest of the Bosch 800 machine authoring): the derived vertical levels, the plate-tine x grid, the basket bays and
the tray's flat-lay zones, all pure functions of one ``config.RACK_GEN`` entry. The built racks themselves ship as
the per-state machine USDs and their collision caches in ``data/assets/bosch800``.
"""

import numpy as np


def _z_levels(p: dict) -> dict:
    r_heavy = p["wire_dia_heavy"] / 2.0
    r_load = p["wire_dia_load"] / 2.0
    r_light = p["wire_dia_light"] / 2.0
    ch = p.get("channel")
    z_runner = r_load + (ch["drop"] if ch else 0.0)  # channel runners sit at r_load (global z-min 0)
    z_cross = z_runner + r_load + r_light  # crossbars rest tangent on the runners
    return {
        "r_heavy": r_heavy,
        "r_load": r_load,
        "r_light": r_light,
        "z_runner": z_runner,
        # heavy outermost runners: top-aligned with the load runners where possible, but never
        # below z=0 (the upper rack has no channel, so bottom-aligning at r_heavy wins there)
        "z_runner_heavy": max(z_runner + r_load - r_heavy, r_heavy),
        "z_cross": z_cross,
        "z_bot_rail": max(z_runner, r_heavy),  # bottom perimeter rail center
        "floor_top": z_cross + r_light,  # what the object stands on; the slot-datum level
        "tine_base": z_cross + r_light,
    }


def _plate_tine_xs(p: dict) -> np.ndarray:
    x0, x1 = p["plate_tine_xspan"]
    n = int(round((x1 - x0) / p["plate_tine_pitch"])) + 1
    return x0 + np.arange(n) * p["plate_tine_pitch"]


def basket_bays(params: dict) -> list[tuple[float, float, float, float]]:
    """Interior (x0, x1, y0, y1) of each basket compartment (design frame)."""
    b = params["basket"]
    t = b["wall_t"]
    y0, y1 = b["y"][0] + t, b["y"][1] - t
    lo_edges = [b["x"][0] + t] + [d + t / 2.0 for d in b["dividers_x"]]
    hi_edges = [d - t / 2.0 for d in b["dividers_x"]] + [b["x"][1] - t]
    return [(lo, hi, y0, y1) for lo, hi in zip(lo_edges, hi_edges)]


#: Ramp run [m] joining the wing floor to the dropped channel floor on each tray crossbar
#: (the same short-diagonal idiom as the v1 tracking-channel dip in :func:`_bar_segments`;
#: longer here because the tray channel drops 28 mm, not 4).
_TRAY_RAMP_RUN = 0.010


def _tray_z_levels(p: dict) -> dict:
    """Derived vertical layout for the tray builder (same convention as :func:`_z_levels`).

    The dropped center channel carries the global z-minimum: channel-span runners bottom
    exactly at z=0, wing runners are raised by ``channel.drop``, crossbars rest tangent on
    the runners, and the plane an object lies on is the crossbar top.
    """
    r_heavy = p["wire_dia_heavy"] / 2.0
    r_light = p["wire_dia_light"] / 2.0
    ch = p.get("channel")
    drop = ch["drop"] if ch else 0.0
    z_runner = r_light + drop  # wing runners; channel runners sit at r_light (bottoms at z=0)
    z_cross = z_runner + 2.0 * r_light  # crossbars rest tangent on the runners
    return {
        "r_heavy": r_heavy,
        "r_light": r_light,
        "drop": drop,
        "z_runner": z_runner,
        "z_cross": z_cross,
        "z_bot_rail": max(z_runner, r_heavy),  # bottom perimeter rail center
        "floor_top": z_cross + r_light,  # wing floor plane (the flat-lay resting level)
    }


def tray_zones(params: dict) -> list[tuple[str, tuple[float, float, float, float], float]]:
    """Usable flat-lay areas as ``(name, (x0, x1, y0, y1), floor_z)``, left wing to right wing.

    Wing bounds run from the rim inner face to the channel edge; the channel zone keeps only
    its flat mid span (the crossbar ramps eat ``_TRAY_RAMP_RUN`` on each side). ``floor_z``
    is that zone's resting plane (wing plane, or the dropped channel floor). Without a
    ``channel`` key the whole floor is one ``"floor"`` zone.
    """
    W, D = params["footprint"]
    zl = _tray_z_levels(params)
    x_lo, x_hi = 2.0 * zl["r_heavy"], W - 2.0 * zl["r_heavy"]  # inner face of the rim wires
    y_lo, y_hi = 2.0 * zl["r_heavy"], D - 2.0 * zl["r_heavy"]
    z_wing = zl["floor_top"]
    ch = params.get("channel")
    if not ch:
        return [("floor", (x_lo, x_hi, y_lo, y_hi), z_wing)]
    cx0, cx1 = ch["x"]
    return [
        ("wing_l", (x_lo, cx0, y_lo, y_hi), z_wing),
        ("channel", (cx0 + _TRAY_RAMP_RUN, cx1 - _TRAY_RAMP_RUN, y_lo, y_hi), z_wing - zl["drop"]),
        ("wing_r", (cx1, x_hi, y_lo, y_hi), z_wing),
    ]


def floor_top_z(params: dict) -> float:
    """Top of the open-zone floor wires (crossbar tops).

    For the LOWER rack this is also the level placement.py's 95th-percentile slot datum lands
    on (asserted in tests). The upper rack's ascending wall ramps sit higher inside the 15 mm
    percentile band, so its datum would come out a few mm above this — nothing derives an
    upper-rack datum today, but don't extend slot derivation there without revisiting that.
    """
    return _z_levels(params)["floor_top"]
