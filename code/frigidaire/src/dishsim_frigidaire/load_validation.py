"""Kit-free integrity checks at the Frigidaire full-load simulation boundary."""
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
import re


def usdc_hashes(directory):
    directory = Path(directory)
    return {str(path.relative_to(directory)): hashlib.sha256(path.read_bytes()).hexdigest()
            for path in sorted(directory.rglob("*.usdc")) if path.is_file()}


def _finite_vector(value, length, label):
    if not isinstance(value, (list, tuple)) or len(value) != length:
        raise ValueError(f"{label} must have {length} numeric elements")
    if any(isinstance(x, bool) or not isinstance(x, (int, float)) or not math.isfinite(x) for x in value):
        raise ValueError(f"{label} contains a nonfinite or nonnumeric value")
    return value


def validate_manifest(manifest, usd_path, catalog, body_positions):
    """Reject stale or inconsistent inputs before creating any simulation objects.

    The recorded asset_dir is provenance only: a portable asset directory may move.
    All files are read beneath the explicitly supplied USD directory; manifest paths
    are never followed. USD geometry itself is validated by the asset build/tests.
    """
    usd_path = Path(usd_path)
    if not usd_path.is_file() or usd_path.suffix != ".usdc":
        raise ValueError(f"Expected an existing appliance .usdc file: {usd_path}")
    if not isinstance(manifest, dict) or manifest.get("schema_version") != 1:
        raise ValueError("Full-load manifest requires schema_version 1")
    actual_hashes = usdc_hashes(usd_path.parent)
    recorded_hashes = manifest.get("geometry_hashes")
    if not isinstance(recorded_hashes, dict) or recorded_hashes != actual_hashes:
        expected = recorded_hashes if isinstance(recorded_hashes, dict) else {}
        missing = sorted(set(expected)-set(actual_hashes))
        added = sorted(set(actual_hashes)-set(expected))
        changed = sorted(key for key in set(expected) & set(actual_hashes) if expected[key] != actual_hashes[key])
        raise ValueError(f"Stale manifest geometry_hashes: missing={missing}, added={added}, changed={changed}; replan the load")
    if manifest.get("catalog") != catalog:
        raise ValueError("Manifest catalog differs from the fixed tableware dimensions, masses, or origins")
    catalog_file = usd_path.parent / "tableware/catalog.json"
    if not catalog_file.is_file():
        raise ValueError("The authored tableware catalog.json is missing")
    authored = json.loads(catalog_file.read_text()).get("items", {})
    if set(authored) != set(catalog) or any(any(authored[kind].get(key) != value for key, value in spec.items())
                                          for kind, spec in catalog.items()):
        raise ValueError("Authored tableware catalog disagrees with the manifest's fixed catalog")
    if any(authored[kind].get("sha256") != actual_hashes.get(f"tableware/{kind}.usdc") for kind in catalog):
        raise ValueError("Authored tableware catalog hashes disagree with the USD files")
    frames = manifest.get("body_positions_m")
    if not isinstance(frames, dict) or set(frames) != set(body_positions):
        raise ValueError("Manifest supporting body frames do not match this appliance")
    for name, expected in body_positions.items():
        actual = _finite_vector(frames[name], 3, f"body_positions_m.{name}")
        if any(abs(a-b) > 1e-9 for a, b in zip(actual, expected)):
            raise ValueError(f"Stale manifest supporting frame: {name}; replan the load")
    entries = manifest.get("objects")
    if not isinstance(entries, list) or not entries:
        raise ValueError("Manifest objects must be a nonempty list")
    seen = set()
    rack_names = {"lower": "LowerRack", "upper": "UpperRack", "basket": "SilverwareBasket"}
    for entry in entries:
        if not isinstance(entry, dict):
            raise ValueError("Every object entry must be a dictionary")
        identity = entry.get("id")
        if not isinstance(identity, str) or not re.fullmatch(r"[A-Za-z_][A-Za-z0-9_]*", identity) or identity in seen:
            raise ValueError(f"Object ID must be unique and a valid USD identifier: {identity!r}")
        seen.add(identity)
        kind = entry.get("kind")
        if kind not in catalog:
            raise ValueError(f"Unknown tableware kind for {identity}: {kind!r}")
        allowed = catalog[kind]["rack"]
        allowed = allowed if isinstance(allowed, (list, tuple)) else [allowed]
        if entry.get("rack") not in {rack_names[name] for name in allowed}:
            raise ValueError(f"Incorrect supporting rack for {identity}: {entry.get('rack')!r}")
        filename = usd_path.parent / "tableware" / f"{kind}.usdc"
        if not filename.is_file():
            raise ValueError(f"Tableware asset missing: {filename}")
        _finite_vector(entry.get("position"), 3, f"{identity}.position")
        hover = entry.get("release_hover_m", .003)
        if isinstance(hover, bool) or not isinstance(hover, (int, float)) or not math.isfinite(hover) or not 0 <= hover <= .01:
            raise ValueError(f"{identity}.release_hover_m must be finite and between 0 and 10 mm")
        quaternion = _finite_vector(entry.get("quaternion_xyzw"), 4, f"{identity}.quaternion_xyzw")
        if abs(sum(x*x for x in quaternion)-1.) > 1e-5:
            raise ValueError(f"{identity}.quaternion_xyzw must be normalized")
    expected_counts = dict(Counter(entry["kind"] for entry in entries))
    supplied_counts = manifest.get("counts")
    if (not isinstance(supplied_counts, dict) or supplied_counts != expected_counts or
            any(isinstance(value, bool) or not isinstance(value, int) or value <= 0 for value in supplied_counts.values())):
        raise ValueError(f"Manifest counts disagree with the object list: expected {expected_counts}")
    return {"object_count": len(entries), "counts": expected_counts, "usdc_file_count": len(actual_hashes)}


def externally_supported_indices(active_pairs):
    """A body's convex subparts contacting itself cannot establish support."""
    return {index for first, second in active_pairs if first != second for index in (first, second)}


def pose_motion_metrics(positions, quaternions_wxyz, local_extrema, dt):
    """Unsmoothed mesh motion from actor poses, independent of solver velocity fields."""
    import numpy as np
    positions = np.asarray(positions, dtype=float)
    quaternions = np.asarray(quaternions_wxyz, dtype=float)
    points = np.asarray(local_extrema, dtype=float)
    if positions.ndim != 2 or positions.shape[1] != 3 or len(positions) < 2:
        raise ValueError("Motion measurements require at least two three-dimensional actor positions")
    if quaternions.shape != (len(positions), 4) or points.ndim != 2 or points.shape[1] != 3 or not len(points):
        raise ValueError("Motion measurements require matching quaternions and actual local mesh points")
    if not np.isfinite(positions).all() or not np.isfinite(quaternions).all() or not np.isfinite(points).all() or dt <= 0:
        raise ValueError("Motion measurements must be finite and have a positive timestep")
    norms = np.linalg.norm(quaternions, axis=1)
    if (norms < 1e-12).any():
        raise ValueError("Actor orientation cannot be a zero quaternion")
    quaternions /= norms[:, None]
    w, x, y, z = quaternions.T
    rotation = np.stack((1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y),
                         2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x),
                         2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)), axis=1).reshape(-1, 3, 3)
    world = np.einsum("nij,kj->nki", rotation, points) + positions[:, None, :]
    peak_speed = float(np.linalg.norm(np.diff(world, axis=0), axis=2).max()/dt)
    angles = 2*np.arccos(np.clip(np.abs(quaternions @ quaternions[0]), 0., 1.))
    return {"root_position_span_m": float(np.ptp(positions, axis=0).max()),
            "peak_mesh_point_speed_m_s": peak_speed,
            "quaternion_span_deg": float(np.degrees(angles.max())),
            "sample_count": len(positions), "sample_duration_s": (len(positions)-1)*dt}
