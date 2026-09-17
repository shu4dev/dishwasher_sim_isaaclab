# Copyright (c) 2026, dishsim project.
# SPDX-License-Identifier: BSD-3-Clause
"""Sampling and measurement rules for the single-dish random-pose experiment.

This module imports neither USD nor Isaac. Poses use metres and XYZW
quaternions. Samples are independent proposals, not distinct resting states or
an estimate of the finite number of all possible continuous dish poses.
"""
from __future__ import annotations

from copy import deepcopy
from functools import lru_cache

import numpy as np


KINDS = ("dinner_plate", "bowl", "mug")
RACKS = ("LowerRack", "UpperRack")
CELLS = tuple((kind, rack) for kind in KINDS for rack in RACKS)
LIMITS = {
    "physics_dt_s": 1 / 120,
    "settle_timeout_s": 12.,
    "rest_window_s": 1.,
    "rack_endpoint_m": .005,
    "root_position_span_m": .005,
    "quaternion_span_deg": 3.,
    "peak_mesh_point_speed_m_s": .03,
    "containment_tolerance_m": .001,
    "peak_penetration_m": .002,
    "median_max_penetration_m": .001,
    "rack_speed_m_s": .10,
}


def cell_generators(seed):
    """Stable independent streams; skipping a rack never renumbers the others."""
    if isinstance(seed, (bool, np.bool_)) or not isinstance(seed, (int, np.integer)) or seed < 0:
        raise ValueError("seed must be a nonnegative integer")
    streams = np.random.SeedSequence(int(seed)).spawn(len(CELLS))
    return {cell: np.random.default_rng(stream) for cell, stream in zip(CELLS, streams)}


def round_robin_cells(samples_per_cell, skipped_racks=()):
    """Yield (kind, rack, zero-based sample index), fairly interleaving cells."""
    if (isinstance(samples_per_cell, (bool, np.bool_))
            or not isinstance(samples_per_cell, (int, np.integer)) or samples_per_cell < 0):
        raise ValueError("samples_per_cell must be a nonnegative integer")
    skipped = set(skipped_racks)
    if skipped - set(RACKS):
        raise ValueError("skipped_racks contains an unknown rack")
    for sample_index in range(int(samples_per_cell)):
        for kind, rack in CELLS:
            if rack not in skipped:
                yield kind, rack, sample_index


def _vector(value, size, name):
    result = np.asarray(value, dtype=float)
    if result.shape != (size,) or not np.isfinite(result).all():
        raise ValueError(f"{name} must contain {size} finite numbers")
    return result


def _quaternion(value):
    result = _vector(value, 4, "quaternion")
    norm = np.linalg.norm(result)
    if norm < 1e-12:
        raise ValueError("quaternion must have nonzero norm")
    return result / norm


def _vertices(value):
    result = np.asarray(value, dtype=float)
    if result.ndim != 2 or result.shape[1] != 3 or not len(result) or not np.isfinite(result).all():
        raise ValueError("vertices must be a nonempty array of finite XYZ points")
    return result


def _bounds(lower, upper):
    lower = _vector(lower, 3, "lower bounds")
    upper = _vector(upper, 3, "upper bounds")
    if (upper <= lower).any():
        raise ValueError("bounds must have positive extent along every axis")
    return lower, upper


def quaternion_matrix_xyzw(quaternion):
    """Return the active rotation matrix without depending on scipy or Kit."""
    x, y, z, w = _quaternion(quaternion)
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y)],
        [2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x)],
        [2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)],
    ])


def uniform_quaternion_xyzw(rng):
    """Gaussian normalization gives the uniform distribution on SO(3)."""
    while True:
        quaternion = rng.normal(size=4)
        norm = np.linalg.norm(quaternion)
        if norm > 1e-12:
            return quaternion / norm


def transform_vertices(vertices, position, quaternion):
    return _vertices(vertices) @ quaternion_matrix_xyzw(quaternion).T + _vector(position, 3, "position")


def _multiply_quaternions(first, second):
    a, b = _quaternion(first), _quaternion(second)
    return _quaternion(np.r_[a[3]*b[:3] + b[3]*a[:3] + np.cross(a[:3], b[:3]),
                             a[3]*b[3] - np.dot(a[:3], b[:3])])


def compose_pose(parent_position, parent_quaternion, local_position, local_quaternion):
    """Return (position ndarray, XYZW quaternion ndarray) in the parent frame."""
    position = (_vector(parent_position, 3, "parent position")
                + quaternion_matrix_xyzw(parent_quaternion) @ _vector(local_position, 3, "local position"))
    return position, _multiply_quaternions(parent_quaternion, local_quaternion)


def relative_pose(world_position, world_quaternion, frame_position, frame_quaternion):
    """Express a world pose in a measured, possibly rotated rack/body frame."""
    inverse = _quaternion(frame_quaternion) * [-1, -1, -1, 1]
    position = quaternion_matrix_xyzw(inverse) @ (
        _vector(world_position, 3, "world position") - _vector(frame_position, 3, "frame position"))
    return position, _multiply_quaternions(inverse, world_quaternion)


def sample_pose(rng, visual_vertices, bounds):
    """Uniformly sample rotation and rotated visual bounding-box center.

    The center domain is not shrunk to fit a particular orientation. Every
    proposal counts, including protruding or initially colliding proposals.
    Actor origins need not equal visual centers (bowls and mug handles do not).
    """
    lower, upper = _bounds(bounds["lower_m"], bounds["upper_m"])
    quaternion = uniform_quaternion_xyzw(rng)
    rotated = _vertices(visual_vertices) @ quaternion_matrix_xyzw(quaternion).T
    rotated_center = (rotated.min(axis=0) + rotated.max(axis=0)) / 2
    center = rng.uniform(lower, upper)
    return {"position_m": (center-rotated_center).tolist(),
            "quaternion_xyzw": quaternion.tolist(), "bbox_center_m": center.tolist()}


def vertices_contained(vertices, lower, upper, tolerance=LIMITS["containment_tolerance_m"]):
    """Check every visual vertex, including handles, in the enclosure frame."""
    points = _vertices(vertices)
    lower, upper = _bounds(lower, upper)
    if not np.isfinite(tolerance) or tolerance < 0:
        raise ValueError("containment tolerance must be finite and nonnegative")
    return bool(((points >= lower-tolerance) & (points <= upper+tolerance)).all())


def motion_is_settled(metrics):
    """Fail closed unless a full trailing window satisfies existing limits.

    Accepts the result of load_validation.pose_motion_metrics, which measures
    actual mesh-point motion and treats opposite quaternion signs identically.
    """
    keys = ("root_position_span_m", "quaternion_span_deg", "peak_mesh_point_speed_m_s")
    try:
        values = [float(metrics[key]) for key in keys]
        duration = float(metrics["sample_duration_s"])
        return (np.isfinite(duration) and duration >= LIMITS["rest_window_s"]-1e-9
                and all(np.isfinite(value) and 0 <= value < LIMITS[key]
                        for key, value in zip(keys, values)))
    except (KeyError, TypeError, ValueError):
        return False


@lru_cache(maxsize=1)
def _source_geometry_domains():
    from .geometry import PARAMETERS, build_components

    components = build_components()
    solids = {solid["name"]: solid for solid in components["Cabinet"]["solids"]}

    def surface(name, axis, side):
        solid = solids[name]
        return float(solid["center"][axis] + side*solid["size"][axis]/2)

    floor_names = {"LowerRack": ("FloorCrossU_", "FloorLongU_"),
                   "UpperRack": ("ContouredCrossU_", "LongitudinalCradle_")}
    floors = {rack: [(name, np.asarray(path), radius)
                     for name, path, radius in components[rack]["wires"]
                     if name.startswith(floor_names[rack])] for rack in RACKS}
    if any(not paths for paths in floors.values()):
        raise ValueError("Geometry does not contain recognized rack floor wires")
    floor_top = {rack: min(float(path[:, 2].min()+radius) for _, path, radius in floors[rack])
                 for rack in RACKS}
    upper_underside = (PARAMETERS["origins"]["UpperRack"][2]
                       + min(float(path[:, 2].min()-radius) for _, path, radius in floors["UpperRack"]))
    ceiling = surface("TubCeiling", 2, -1)
    bounds = {}
    for rack, parameter in zip(RACKS, ("lower_rack", "upper_rack")):
        dimensions = PARAMETERS[parameter]
        half_width, half_depth = dimensions["wire_width"]/2, dimensions["wire_depth"]/2
        overhead = upper_underside if rack == "LowerRack" else ceiling
        lower = [-half_width, -half_depth, floor_top[rack]]
        upper = [half_width, half_depth, overhead-PARAMETERS["origins"][rack][2]]
        _bounds(lower, upper)
        bounds[rack] = {"frame": rack, "lower_m": lower, "upper_m": upper}
    interior = {"frame": "Cabinet",
                "lower_m": [surface("TubLeft", 0, 1),
                            float(PARAMETERS["interior_fit"]["closed_liner_inner_y"]),
                            surface("TubFloor", 2, 1)],
                "upper_m": [surface("TubRight", 0, -1), surface("TubBack", 1, -1), ceiling]}
    _bounds(interior["lower_m"], interior["upper_m"])
    return {"sampling_bounds_by_rack": bounds, "interior_bounds": interior,
            "derivation": {"source": "dishsim_frigidaire.geometry.PARAMETERS and build_components()",
                           "floor_datum": "lowest generated floor-wire upper surface, including filleted paths",
                           "floor_wire_names": {rack: [name for name, _, _ in floors[rack]] for rack in RACKS},
                           "floor_top_local_m": floor_top,
                           "lower_overhead_cabinet_z_m": upper_underside,
                           "upper_overhead_cabinet_z_m": ceiling,
                           "xy_datum": "outer wire-rim width and depth; no orientation-dependent shrinkage",
                           "interior_datum": "inner tub faces and closed door liner plane; door stays open",
                           "geometry_status": "modeled estimates; runtime must verify staged source provenance"}}


def source_geometry_domains():
    """JSON-serializable source-derived domains, copied to prevent mutation."""
    return deepcopy(_source_geometry_domains())
