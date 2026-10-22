# Copyright (c) 2026, dishsim project.
# SPDX-License-Identifier: BSD-3-Clause
"""FDPC4221AS geometry as the experiments read it: the asset folder's ``parameters.json`` and ``components.json``.

Coordinates are metres, X across the appliance, -Y toward its front, and Z up. The generator (``PARAMETERS``
literal, derivation and mesh builders) moved to ``data/assets/frigidaire_fdpc4221as/source/geometry.py`` on
2026-10-21; this module only loads what it produced: ``PARAMETERS`` is ``parameters.json["geometry"]`` (the derived
parameter table), :func:`components` is the exported ``build_components()`` output (meshes, wire paths, solids,
sites), and the tine-grid helpers below are the generator's own functions over ``PARAMETERS``. ``Mesh`` stays for
the tableware shapes (``tableware.tableware_geometry`` sweeps the mug handle with it). numpy only.
"""
from dataclasses import dataclass, field
from functools import lru_cache
import json
import math

import numpy as np

from .paths import COMPONENTS_PATH, PARAMETERS_PATH

if not PARAMETERS_PATH.is_file():
    raise FileNotFoundError(f"{PARAMETERS_PATH} is missing: data/assets is a copy of the HF dataset "
                            "shu4dev/dishsim-assets (see data/assets/README.md)")
#: the whole parameters.json (geometry, body_positions_m, candidate_sites, joint_limits_usd, ...)
PARAMETERS_JSON = json.loads(PARAMETERS_PATH.read_text())
PARAMETERS = PARAMETERS_JSON["geometry"]


def _unit(v):
    v = np.asarray(v, dtype=float)
    norm = np.linalg.norm(v)
    if norm < 1e-12:
        raise ValueError("Cannot normalize zero-length vector")
    return v / norm


@dataclass
class Mesh:
    material: str
    points: list = field(default_factory=list)
    normals: list = field(default_factory=list)
    faces: list = field(default_factory=list)

    def tube(self, path, radius, sides=12):
        """Sweep a smooth tube with parallel-transport normals and round caps."""
        path = np.asarray(path, dtype=float)
        tangents = np.empty_like(path)
        tangents[0], tangents[-1] = _unit(path[1]-path[0]), _unit(path[-1]-path[-2])
        for i in range(1, len(path)-1):
            tangents[i] = _unit(_unit(path[i]-path[i-1])+_unit(path[i+1]-path[i]))
        reference = np.eye(3)[np.argmin(np.abs(tangents[0]))]
        u = _unit(np.cross(reference, tangents[0]))
        angles = np.arange(sides)*2*math.pi/sides
        frames = []
        for t in tangents:
            u = _unit(u-t*np.dot(u, t))
            v = np.cross(t, u)
            frames.append(np.cos(angles)[:, None]*u+np.sin(angles)[:, None]*v)
        rings, normals = [], []
        for theta in [-math.pi/3, -math.pi/6]:
            n = math.cos(theta)*frames[0]+math.sin(theta)*tangents[0]
            rings.append(path[0]+radius*n)
            normals.append(n)
        for p, frame in zip(path, frames):
            rings.append(p+radius*frame)
            normals.append(frame)
        for theta in [math.pi/6, math.pi/3]:
            n = math.cos(theta)*frames[-1]+math.sin(theta)*tangents[-1]
            rings.append(path[-1]+radius*n)
            normals.append(n)
        offset = len(self.points)
        self.points.extend(np.asarray(rings).reshape(-1, 3).tolist())
        self.normals.extend(np.asarray(normals).reshape(-1, 3).tolist())
        for i in range(len(rings)-1):
            for j in range(sides):
                a, b = offset+i*sides+j, offset+i*sides+(j+1)%sides
                c, d = offset+(i+1)*sides+(j+1)%sides, offset+(i+1)*sides+j
                self.faces.extend([(a, b, c), (a, c, d)])
        for ring, p, normal, reverse in [
            (offset, path[0]-radius*tangents[0], -tangents[0], True),
            (offset+(len(rings)-1)*sides, path[-1]+radius*tangents[-1], tangents[-1], False),
        ]:
            pole = len(self.points)
            self.points.append(p.tolist())
            self.normals.append(normal.tolist())
            for j in range(sides):
                a, b = ring+j, ring+(j+1)%sides
                self.faces.append((pole, b, a) if reverse else (pole, a, b))


def lower_tine_positions():
    """Return column X and front-to-back row Y base positions in local metres.

    The outer rim size, counts, pitches and the left/front margins are the tape
    measurements: columns run from the left margin at the column pitch, rows from
    the front margin at the five tape row gaps (see
    PARAMETERS["lower_rack"]["tine_margin_derivation"]). Placement and inspection
    consume the same generated grid, including positions that the basket bay omits
    (see lower_tine_mask).
    """
    p = PARAMETERS["lower_rack"]
    m = p["tine_margins"]
    xs = -p["wire_width"]/2 + m["left"] + np.arange(p["tines_per_bank"]) * p["tine_pitch"]["x"]
    ys = -p["wire_depth"]/2 + m["front"] + np.concatenate([[0.], np.cumsum(p["tine_pitch"]["y"])])
    return xs, ys


def lower_tine_heights():
    """Vertical rise per row, front to back; the short rows are the tape-measured 45 mm ones."""
    p = PARAMETERS["lower_rack"]
    heights = np.full(p["tine_banks"], float(p["tine_height"]))
    heights[list(p["tine_short_rows"])] = p["tine_short_height"]
    return heights


def lower_basket_footprint():
    """Basket outer footprint in the lower rack frame plus the tine-omission clearance."""
    basket = PARAMETERS["silverware_basket"]
    seat = basket["seat"]["origin_in_rack_m"]
    hx, hy = basket["width_x"]/2, basket["length_y"]/2
    return {"x": (seat[0]-hx, seat[0]+hx), "y": (seat[1]-hy, seat[1]+hy),
            "clearance": PARAMETERS["lower_rack"]["basket_bay"]["clearance"]}


def lower_tine_mask():
    """Boolean (rows, columns) array; False where the basket bay omits the tine.

    A tine is omitted when its base centre, or the tip it leans toward +X, lies
    inside the basket footprint grown by the bay clearance.
    """
    xs, ys = lower_tine_positions()
    footprint = lower_basket_footprint()
    c = footprint["clearance"]
    tips = xs + PARAMETERS["lower_rack"]["tine_tip_offset_x"]
    inside_x = (tips >= footprint["x"][0]-c) & (xs <= footprint["x"][1]+c)
    inside_y = (ys >= footprint["y"][0]-c) & (ys <= footprint["y"][1]+c)
    return ~(inside_y[:, None] & inside_x[None, :])


def lower_tine_present(column, row):
    return bool(lower_tine_mask()[row, column])


def lower_plate_gaps(bank):
    """Gap indices between consecutive present tines in both rows of a plate bank."""
    rows = (0, 1) if bank == "front" else (-2, -1)
    mask = lower_tine_mask()
    both = mask[rows[0]] & mask[rows[1]]
    return [i for i in range(len(both)-1) if both[i] and both[i+1]]


def upper_tine_positions():
    """Return column X and front-to-back base Y positions in rack-local metres.

    Columns follow the tape column gaps, symmetric about the centre; positions run
    at the tape pitch from the tape front margin. The same grid defines geometry,
    loading gaps and inspection; absent positions come from upper_tine_mask.
    """
    p = PARAMETERS["upper_rack"]
    front_y = -p["wire_depth"]/2 + p["tine_front_margin"]
    ys = front_y + np.arange(p["tines_per_bank"])*p["tine_spacing"]
    return np.array(p["tine_bank_x"], dtype=float), ys


def upper_tine_mask():
    """Boolean (positions, columns) array; False at the absent center positions."""
    p = PARAMETERS["upper_rack"]
    mask = np.ones((p["tines_per_bank"], p["tine_banks"]), dtype=bool)
    absent = p["tine_absent"]
    for column in absent["columns"]:
        mask[list(absent["indices"]), column] = False
    return mask


def upper_tine_present(column, index):
    return bool(upper_tine_mask()[index, column])


def upper_tine_gaps(column, regular_only=True):
    """(lower, upper) position indices of consecutive present tines in one column.

    The absent center positions of the middle columns leave one wide opening;
    regular_only drops it so saucer slots stay one pitch wide.
    """
    present = np.flatnonzero(upper_tine_mask()[:, column])
    gaps = [(int(a), int(b)) for a, b in zip(present[:-1], present[1:])]
    return [g for g in gaps if g[1]-g[0] == 1] if regular_only else gaps


def tine_counts():
    return {"UpperRack": int(upper_tine_mask().sum()), "LowerRack": int(lower_tine_mask().sum())}


@lru_cache(maxsize=1)
def components():
    """``build_components()`` of the generator, as exported to ``components.json``: per component its mass,
    visual ``meshes`` (``Mesh`` with points and faces), wire paths ``(name, path, radius)``, ``solids``, ``sites``
    and ``site_quats``. Every float round-trips exactly (JSON repr), so the arrays equal the generator's."""
    raw = json.loads(COMPONENTS_PATH.read_text())
    return {name: {"mass": c["mass"],
                   "meshes": {key: Mesh(m["material"], m["points"], [], m["faces"]) for key, m in c["meshes"].items()},
                   "wires": [(key, np.asarray(path, dtype=float), radius) for key, path, radius in c["wires"]],
                   "solids": c["solids"], "sites": c["sites"],
                   **({"site_quats": c["site_quats"]} if c["site_quats"] else {})}
            for name, c in raw.items()}
