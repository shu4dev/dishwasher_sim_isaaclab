#!/usr/bin/env python3
# Copyright (c) 2026, dishsim project.
# SPDX-License-Identifier: BSD-3-Clause
"""Measure generated lower-rack/basket capsules without USD, FCL, or Isaac.

The fixture audit only confirms sampled centerline penetrations into authored
collider solids. Absence of a witness does not establish fixture feasibility.
"""
import argparse
import ast
import hashlib
import itertools
import json
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "frigidaire/src"))
from dishsim_frigidaire.paths import VALIDATION_DIR
from dishsim_frigidaire import geometry


def segment_pairs(p0, p1, q0, q1):
    """Distances and closest points from one segment to N segments.

    Enumerate all four endpoint projections and the unconstrained interior
    solution. Endpoint projections cover parallel and degenerate segments.
    """
    p0, p1 = np.asarray(p0, dtype=float), np.asarray(p1, dtype=float)
    q0, q1 = np.atleast_2d(q0).astype(float), np.atleast_2d(q1).astype(float)
    u, v, w = p1-p0, q1-q0, p0-q0
    uu, vv = u @ u, np.einsum("ij,ij->i", v, v)
    candidates = []
    for q in (q0, q1):
        s = np.clip((q-p0) @ u / (uu if uu else 1.), 0., 1.)
        candidates.append((p0+s[:, None]*u, q))
    for p in (p0, p1):
        t = np.clip(np.einsum("ij,ij->i", p-q0, v)/np.where(vv > 0, vv, 1.), 0., 1.)
        candidates.append((np.broadcast_to(p, q0.shape), q0+t[:, None]*v))
    # The cross product avoids cancellation in uu*vv - uv*uv for nearly
    # parallel segments. A truly parallel pair is handled by the endpoints.
    cross = np.cross(u, v)
    denominator = np.einsum("ij,ij->i", cross, cross)
    safe = np.where(denominator > 0, denominator, 1.)
    s = np.einsum("ij,ij->i", np.cross(-w, v), cross)/safe
    t = np.einsum("ij,ij->i", np.cross(-w, u), cross)/safe
    valid = (denominator > 0) & (s >= 0) & (s <= 1) & (t >= 0) & (t <= 1)
    candidates.append((p0+s[:, None]*u, q0+t[:, None]*v))
    distances = np.stack([np.linalg.norm(a-b, axis=1) for a, b in candidates])
    distances[-1, ~valid] = np.inf
    which, index = distances.argmin(axis=0), np.arange(len(q0))
    return (distances[which, index],
            np.stack([a for a, _ in candidates])[which, index],
            np.stack([b for _, b in candidates])[which, index])


def segments(wires, translation=(0., 0., 0.)):
    entries = []
    translation = np.asarray(translation)
    for name, path, radius in wires:
        for index, (a, b) in enumerate(zip(path[:-1], path[1:])):
            entries.append((name, index, a+translation, b+translation, radius))
    return entries


def minimum_clearance(first, second):
    """Signed minimum capsule clearance, with a reproducible wire witness."""
    if not first or not second:
        raise ValueError("Both capsule sets must be nonempty")
    starts = np.asarray([s[2] for s in second])
    ends = np.asarray([s[3] for s in second])
    radii = np.asarray([s[4] for s in second])
    best = None
    for name, index, a, b, radius in first:
        distances, on_first, on_second = segment_pairs(a, b, starts, ends)
        clearance = distances-radius-radii
        i = int(np.argmin(clearance))
        if best is None or clearance[i] < best["clearance_mm"]/1000:
            best = {
                "clearance_mm": float(clearance[i]*1000),
                "rack_wire": name, "rack_segment": index,
                "basket_wire": second[i][0], "basket_segment": second[i][1],
                "rack_centerline_point_mm": (on_first[i]*1000).tolist(),
                "basket_centerline_point_mm": (on_second[i]*1000).tolist(),
                "radii_mm": [float(radius*1000), float(radii[i]*1000)],
            }
    return best


def envelope(entries):
    return (np.min([np.minimum(s[2], s[3])-s[4] for s in entries], axis=0),
            np.max([np.maximum(s[2], s[3])+s[4] for s in entries], axis=0))


def basket_clearance_report(lower=None, basket=None, basket_translation=None):
    lower = geometry._lower_rack() if lower is None else lower
    basket = geometry._basket() if basket is None else basket
    if basket_translation is None:
        basket_translation = (np.asarray(geometry.PARAMETERS["origins"]["SilverwareBasket"])
                              - geometry.PARAMETERS["origins"]["LowerRack"])
    rack_segments = segments(lower["wires"])
    basket_segments = segments(basket["wires"], basket_translation)
    groups = {name: [] for name in ("tines_and_base_rails", "right_wall", "floor", "remaining_rack")}
    for entry in rack_segments:
        name, _, a, b, _ = entry
        if name.startswith("TineBank"):
            group = "tines_and_base_rails"
        elif name.startswith("FloorCrossU"):
            # The horizontal central floor ends at X +/-237 mm; its rounded
            # transitions and sloping uprights belong to the wall audit.
            group = ("right_wall" if max(a[0], b[0]) > .23700001 else
                     "remaining_rack" if min(a[0], b[0]) < -.23700001 else "floor")
        elif name.startswith("FloorLongU"):
            group = "remaining_rack" if max(abs(a[1]), abs(b[1])) > .26600001 else "floor"
        else:
            group = "remaining_rack"
        groups[group].append(entry)
    clearances = {name: minimum_clearance(entries, basket_segments)
                  for name, entries in groups.items()}
    basket_low, basket_high = envelope(basket_segments)
    _, tines_high = envelope(groups["tines_and_base_rails"])
    gap = float((basket_low[0]-tines_high[0])*1000)
    checks = {
        "tine_clearance_at_least_5_mm": clearances["tines_and_base_rails"]["clearance_mm"] >= 5.-1e-8,
        "right_wall_clearance_at_least_3_mm": clearances["right_wall"]["clearance_mm"] >= 3.-1e-8,
        "remaining_rack_clearance_positive": clearances["remaining_rack"]["clearance_mm"] > 0,
        "floor_not_interpenetrating": clearances["floor"]["clearance_mm"] >= -1e-8,
        "conservative_x_envelope_gap_at_least_5_mm": gap >= 5.-1e-8,
    }
    # Wheels/hubs are independent solids; a positive Z gap proves their
    # separation without approximating cylinders as capsule paths.
    solid_gaps = []
    for solid in lower["solids"]:
        if solid["kind"] == "cylinder":
            half_z = solid["height"]/2 if solid["axis"] == "Z" else solid["radius"]
        elif solid["kind"] == "box":
            half_z = solid["size"][2]/2
        else:
            raise ValueError("Unsupported rack solid: " + solid["kind"])
        solid_gaps.append((float((basket_low[2]-solid["center"][2]-half_z)*1000), solid["name"]))
    solid_gap, solid_name = min(solid_gaps)
    checks["rack_solids_below_basket"] = solid_gap > 0
    brackets = np.asarray(lower["meshes"]["WheelBrackets"].points)
    bracket_gap = float((basket_low[2]-brackets[:, 2].max())*1000)
    checks["visible_wheel_brackets_below_basket"] = bracket_gap > 0
    return {
        "passed": all(checks.values()), "checks": checks,
        "basket_origin_in_rack_mm": (np.asarray(basket_translation)*1000).tolist(),
        "basket_capsule_bounds_mm": [(basket_low*1000).tolist(), (basket_high*1000).tolist()],
        "conservative_x_envelope_gap_mm": gap, "minimum_capsule_clearances": clearances,
        "minimum_solid_z_gap_mm": solid_gap, "closest_solid_by_z": solid_name,
        "visible_bracket_z_gap_mm": bracket_gap,
        "floor_interpretation": "Floor clearance is reported separately; a positive initial gap does not demonstrate settled support.",
    }


def _fixture_literal(node, name):
    """Read the authored fixture constants, without importing USD/trimesh."""
    if isinstance(node, ast.IfExp):
        test = node.test
        if not (isinstance(test, ast.Compare) and isinstance(test.left, ast.Name)
                and test.left.id == "name" and len(test.ops) == 1
                and isinstance(test.ops[0], ast.Eq)):
            raise ValueError("Unsupported fixture conditional")
        return _fixture_literal(node.body if name == ast.literal_eval(test.comparators[0]) else node.orelse, name)
    if isinstance(node, ast.Dict):
        return {ast.literal_eval(k): _fixture_literal(v, name) for k, v in zip(node.keys, node.values)}
    if isinstance(node, (ast.Tuple, ast.List)):
        return [_fixture_literal(v, name) for v in node.elts]
    return ast.literal_eval(node)


def fixture_specs():
    source = ROOT / "frigidaire/src/dishsim_frigidaire/asset.py"
    function = next(n for n in ast.parse(source.read_text()).body
                    if isinstance(n, ast.FunctionDef) and n.name == "write_fixtures")
    calls = [n for n in ast.walk(function) if isinstance(n, ast.Call)
             and isinstance(n.func, ast.Name) and n.func.id == "_solid"]
    def solid(suffix, kind):
        call = next(n for n in calls if isinstance(n.args[1], ast.BinOp)
                    and isinstance(n.args[1].right, (ast.Str, ast.Constant))
                    and ast.literal_eval(n.args[1].right) == suffix)
        return _fixture_literal(call.args[2], kind)
    wall = next(n.value for n in ast.walk(function) if isinstance(n, ast.Assign)
                and any(isinstance(t, ast.Name) and t.id == "wall" for t in n.targets))
    sector_loop = next(n for n in ast.walk(function) if isinstance(n, ast.For)
                       and isinstance(n.target, ast.Name) and n.target.id == "j")
    if not (isinstance(sector_loop.iter, ast.Call)
            and isinstance(sector_loop.iter.func, ast.Name)
            and sector_loop.iter.func.id == "range" and len(sector_loop.iter.args) == 1):
        raise ValueError("Unsupported authored fixture sector loop")
    sectors = ast.literal_eval(sector_loop.iter.args[0])
    return {"plate": solid("/Collisions/Disc", "plate"),
            "bowl": {"base": solid("/Collisions/Base", "bowl"),
                     "wall": _fixture_literal(wall, "bowl"), "sectors": sectors}}, source


def _hull_planes(vertices):
    planes = []
    for indices in itertools.combinations(range(len(vertices)), 3):
        a, b, c = vertices[list(indices)]
        normal = np.cross(b-a, c-a)
        length = np.linalg.norm(normal)
        if length < 1e-14:
            continue
        normal /= length
        values = (vertices-a) @ normal
        if values.max() <= 1e-12:
            pass
        elif values.min() >= -1e-12:
            normal = -normal
        else:
            continue
        planes.append(np.r_[normal, -a @ normal])
    return np.unique(np.round(planes, 12), axis=0)


def _cylinder_signed_distance(points, spec):
    p = points-np.asarray(spec["center"])
    if spec["axis"] != "Z":
        raise ValueError("Fixture audit expects local Z cylinders")
    d = np.column_stack((np.linalg.norm(p[:, :2], axis=1)-spec["radius"],
                         np.abs(p[:, 2])-spec["height"]/2))
    return np.linalg.norm(np.maximum(d, 0.), axis=1)+np.minimum(d.max(axis=1), 0.)


def fixture_audit(lower):
    specs, source = fixture_specs()
    sampled, names = [], []
    for name, path, _ in lower["wires"]:
        for a, b in zip(path[:-1], path[1:]):
            count = max(2, int(np.ceil(np.linalg.norm(b-a)/.0005))+1)
            sampled.extend(np.linspace(a, b, count))
            names.extend([name]*count)
    sampled = np.asarray(sampled)
    reports = {}
    for kind, spec in specs.items():
        origin = np.asarray(lower["sites"][kind])
        quat = np.asarray(lower.get("site_quats", {}).get(kind, [1., 0., 0., 0.]))
        quat /= np.linalg.norm(quat)
        w, x, y, z = quat
        rotation = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                             [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                             [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
        points = (sampled-origin) @ rotation
        signed = _cylinder_signed_distance(points, spec if kind == "plate" else spec["base"])
        collider_names = np.array(["Disc" if kind == "plate" else "Base"]*len(points), dtype=object)
        if kind == "bowl":
            for sector in range(spec["sectors"]):
                angles = np.array([sector, sector+1])*2*np.pi/spec["sectors"]
                vertices = np.asarray([[r*np.cos(a), r*np.sin(a), z]
                                       for a in angles for r, z in spec["wall"]])
                planes = _hull_planes(vertices)
                values = (points @ planes[:, :3].T+planes[:, 3]).max(axis=1)
                # Only negative values prove membership in the convex piece;
                # positive plane distances are not a Euclidean distance bound.
                better = (values < 0) & (values < signed)
                signed[better] = values[better]
                collider_names[better] = "Wall_%d" % sector
        i = int(np.argmin(signed))
        witness = None
        if signed[i] < -1e-8:
            witness = {"rack_wire": names[i], "fixture_collider": str(collider_names[i]),
                       "point_in_rack_mm": (sampled[i]*1000).tolist(),
                       "point_in_fixture_mm": (points[i]*1000).tolist(),
                       "centerline_interior_depth_mm": float(-signed[i]*1000)}
        reports[kind] = {
            "status": "CONFIRMED_INITIAL_COLLISION" if witness else "NO_SAMPLED_CENTERLINE_PENETRATION_FOUND",
            "site_position_m": origin.tolist(), "site_quaternion_wxyz": quat.tolist(),
            "collision_witness": witness,
            "scope": "Authored fixture collider constants, current source site, and rack wire centerline samples at <=0.5 mm; no dynamic, tableware-catalog, or installed-USD validation.",
        }
    return reports, source


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, default=VALIDATION_DIR / "lower_rack_clearance.json")
    args = parser.parse_args()
    lower = geometry._lower_rack()
    report = basket_clearance_report(lower=lower)
    fixtures, fixture_source = fixture_audit(lower)
    report.update({
        "status": "SOURCE_GEOMETRY_ONLY; not installed-USD, FCL, or Isaac validation",
        "coordinate_system": "lower-rack local metres; report distances in mm",
        "source_sha256": hashlib.sha256(Path(geometry.__file__).read_bytes()).hexdigest(),
        "fixture_source_sha256": hashlib.sha256(fixture_source.read_bytes()).hexdigest(),
        "verifier_source_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        "fixture_audit": fixtures,
        "limits": "Capsule checks cover rack wires, tine rails and basket paths. Solids/brackets are separated by Z bounds. Static fixture penetration witnesses do not certify loading capacity or stable seating.",
    })
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(report, indent=2)+"\n")
    print("[RESULT] %s source basket clearance; %s" % ("PASS" if report["passed"] else "FAIL", args.out))
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
