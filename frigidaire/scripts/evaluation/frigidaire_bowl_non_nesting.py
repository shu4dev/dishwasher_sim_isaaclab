#!/usr/bin/env python3
# Copyright (c) 2026, dishsim project.
# SPDX-License-Identifier: BSD-3-Clause
"""Audit the two full-size bowls in every measured full-load hold (no Kit).

A positive visual-mesh separating-plane gap is a direct certificate. When bowl
slabs touch, inspect their actual capped inner cavities: both foot centers must
remain outside the other's cavity, and any local rim/base intrusion must remain
within the physics report's existing contact tolerance. This distinguishes
shallow numerical contact from seating a bowl's bottom inside another bowl.
"""
from pathlib import Path
import argparse
import hashlib
import json
import sys

import numpy as np
from scipy.spatial import ConvexHull
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "frigidaire/src"))
from dishsim_frigidaire.paths import ASSET_DIR, IMAGE_DIR


def visual_mesh(filename):
    from pxr import Usd, UsdGeom, UsdPhysics
    stage = Usd.Stage.Open(str(filename))
    inverse = UsdGeom.XformCache().GetLocalToWorldTransform(stage.GetDefaultPrim()).GetInverse()
    cache, vertices, triangles = UsdGeom.XformCache(), [], []
    for prim in stage.Traverse():
        if not prim.IsA(UsdGeom.Mesh) or prim.HasAPI(UsdPhysics.CollisionAPI):
            continue
        mesh = UsdGeom.Mesh(prim)
        if any(n != 3 for n in mesh.GetFaceVertexCountsAttr().Get()):
            raise ValueError("Bowl audit expects the actual triangulated visual mesh")
        transform = np.asarray(cache.GetLocalToWorldTransform(prim) * inverse).T
        points = np.asarray(mesh.GetPointsAttr().Get(), dtype=float)
        faces = np.asarray(mesh.GetFaceVertexIndicesAttr().Get(), dtype=int).reshape(-1, 3)
        offset = sum(len(p) for p in vertices)
        vertices.append(points @ transform[:3, :3].T + transform[:3, 3])
        triangles.append(faces + offset)
    if not vertices:
        raise ValueError("The bowl USD contains no visual triangles")
    return np.concatenate(vertices), np.concatenate(triangles)


def cavity_planes(vertices, faces):
    """Extract the convex inner cavity from actual outward-oriented mesh faces."""
    triangles = vertices[faces]
    centers = triangles.mean(axis=1)
    normal = np.cross(triangles[:, 1]-triangles[:, 0], triangles[:, 2]-triangles[:, 0])
    radial = np.einsum("ij,ij->i", centers[:, :2], normal[:, :2])
    mouth_z = float(vertices[:, 2].max())
    inner = ((radial < -1e-14) | ((np.abs(radial) < 1e-14) & (normal[:, 2] > 1e-14)))
    inner &= centers[:, 2] < mouth_z - 1e-7
    selected = np.unique(faces[inner].ravel())
    hull = ConvexHull(vertices[selected])
    # The capped cavity is convex for this catalog bowl. Validate that every
    # extracted interior triangle lies on its boundary, not inside a filled hull.
    center_distances = centers[inner] @ hull.equations[:, :3].T + hull.equations[:, 3]
    if np.max(np.abs(center_distances.max(axis=1))) > 1e-6:
        raise ValueError("Bowl cavity is not convex; a hull would invent interior space")
    equations = np.unique(np.round(hull.equations, 11), axis=0)
    return equations, mouth_z


def cavity_intrusion(points, faces, planes, mouth_z):
    """Clip the other bowl's triangles against the open cavity's halfspaces.

    A 0.1 micrometre inward offset excludes coplanar contact. This is numerical
    predicate precision, not a packing clearance or relaxation of physics gates.
    """
    equations = planes.copy()
    equations[:, 3] += 1e-7
    distances = points @ equations[:, :3].T + equations[:, 3]
    deepest, intersection_count = 0., 0
    for face in faces:
        if np.any(np.all(distances[face] > 0., axis=0)):
            continue
        polygon = points[face].copy()
        for plane in equations:
            values = polygon @ plane[:3] + plane[3]
            if np.all(values <= 0.):
                continue
            if np.all(values > 0.):
                polygon = np.empty((0, 3))
                break
            clipped = []
            for i, current in enumerate(polygon):
                previous = polygon[i-1]
                first, second = values[i-1], values[i]
                if (first <= 0.) != (second <= 0.):
                    clipped.append(previous + first/(first-second)*(current-previous))
                if second <= 0.:
                    clipped.append(current)
            polygon = np.asarray(clipped)
        if len(polygon) >= 3:
            area = sum(np.linalg.norm(np.cross(polygon[i]-polygon[0], polygon[i+1]-polygon[0]))
                       for i in range(1, len(polygon)-1)) / 2
            if area > 1e-14:
                intersection_count += 1
                deepest = max(deepest, mouth_z-float(polygon[:, 2].min()))
    return max(0., deepest), intersection_count


def audit(physics_path, bowl_usd, output):
    raw = physics_path.read_bytes()
    source = json.loads(raw)
    digest = hashlib.sha256(bowl_usd.read_bytes()).hexdigest()
    points, faces = visual_mesh(bowl_usd)
    planes, mouth_z = cavity_planes(points, faces)
    foot_center = np.array([0., 0., points[:, 2].min()])
    tolerance = float(source["thresholds"]["peak_penetration_m"])
    required = {"settled_closed", "open_retracted", "first_extended", "loaded_retracted", "loaded_closed", "final_extended"}
    report = {"result": "INCOMPLETE", "source_physics_sha256": hashlib.sha256(raw).hexdigest(),
              "bowl_visual_usd_sha256": digest, "script_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
              "source_physics_result": source.get("result"), "contact_tolerance_m": tolerance,
              "contact_tolerance_source": "physics.thresholds.peak_penetration_m; unchanged",
              "method": "visual separating planes; exact triangle clipping against mesh-derived inner cavity when slabs overlap",
              "non_nesting_definition": "A bowl foot center is outside the other's inner cavity; any shallow rim/base intrusion is within the existing contact tolerance.",
              "states": {}}
    valid_geometry = digest == source.get("asset_hashes", {}).get("tableware/bowl.usdc")
    for name, state in source.get("states", {}).items():
        bowls = sorted((identity, item) for identity, item in state["objects"].items() if item["kind"] == "bowl")
        if len(bowls) != 2:
            raise ValueError(f"{name}: expected exactly two measured bowls, received {len(bowls)}")
        rotations = [Rotation.from_quat(np.roll(item["quaternion_wxyz"], -1)).as_matrix() for _, item in bowls]
        origins = [np.asarray(item["position_m"]) for _, item in bowls]
        transformed = [points @ rotation.T + origin for rotation, origin in zip(rotations, origins)]
        mean_axis = rotations[0][:, 2] + rotations[1][:, 2]
        axes = [rotations[0][:, 2], rotations[1][:, 2], origins[1]-origins[0]]
        if np.linalg.norm(mean_axis) > 1e-10:
            axes.insert(0, mean_axis)
        measurements = []
        for axis in axes:
            normal = axis / np.linalg.norm(axis)
            a, b = transformed[0] @ normal, transformed[1] @ normal
            gap = float(b.min()-a.max())
            if float(a.min()-b.max()) > gap:
                normal, gap = -normal, float(a.min()-b.max())
            measurements.append({"normal_world": normal.tolist(), "signed_gap_m": gap})
        best = max(measurements, key=lambda record: record["signed_gap_m"])
        record = {"bowls": [identity for identity, _ in bowls],
                  "mean_axis_signed_gap_m": measurements[0]["signed_gap_m"],
                  "best_plane": best, "certified_non_nested": best["signed_gap_m"] > 1e-7,
                  "certificate": "positive separating plane" if best["signed_gap_m"] > 1e-7 else "inner cavity containment"}
        if not record["certified_non_nested"]:
            checks = []
            for first, second in ((0, 1), (1, 0)):
                other = (transformed[second]-origins[first]) @ rotations[first]
                other_foot = ((foot_center @ rotations[second].T + origins[second])-origins[first]) @ rotations[first]
                foot_inside = bool(np.all(planes[:, :3] @ other_foot + planes[:, 3] < -1e-7))
                depth, count = cavity_intrusion(other, faces, planes, mouth_z)
                checks.append({"cavity_owner": bowls[first][0], "other_bowl": bowls[second][0],
                               "other_foot_center_in_cavity": foot_inside,
                               "other_foot_above_mouth_plane_m": float(other_foot[2]-mouth_z),
                               "maximum_surface_intrusion_below_mouth_m": depth,
                               "intersecting_visual_triangles": count})
            record["containment"] = checks
            record["certified_non_nested"] = all(not check["other_foot_center_in_cavity"]
                                                 and check["maximum_surface_intrusion_below_mouth_m"] <= tolerance for check in checks)
        report["states"][name] = record
    complete = source.get("result") == "PASS" and source.get("completed") and source.get("validated_counts")
    complete = bool(complete and required <= set(report["states"]) and valid_geometry)
    report["matches_measured_geometry"] = valid_geometry
    report["minimum_best_plane_signed_gap_m"] = min((r["best_plane"]["signed_gap_m"] for r in report["states"].values()), default=None)
    report["result"] = ("PASS" if all(r["certified_non_nested"] for r in report["states"].values()) else "FAIL") if complete else "INCOMPLETE"
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps({"result": report["result"], "states": report["states"]}, indent=2), flush=True)
    print(f"[RESULT] {report['result']}: measured two-bowl non-nesting audit", flush=True)
    return report["result"] == "PASS"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--physics", type=Path, default=IMAGE_DIR / "full_load/physics.json")
    parser.add_argument("--bowl-usd", type=Path, default=ASSET_DIR / "tableware/bowl.usdc")
    parser.add_argument("--output", type=Path, default=IMAGE_DIR / "full_load/non_nesting.json")
    args = parser.parse_args()
    from dishsim_frigidaire.usd_bootstrap import ensure_usd
    ensure_usd()
    return audit(args.physics, args.bowl_usd, args.output)


if __name__ == "__main__":
    raise SystemExit(0 if main() else 2)
