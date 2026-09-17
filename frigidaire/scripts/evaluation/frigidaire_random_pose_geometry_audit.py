#!/usr/bin/env python3
"""Independently audit accepted final visual geometry; does not rerun physics.

Run with scripts/run_py.sh after the run has finalized. SciPy is used by the
original visual builders. This script changes no experiment records or limits.
"""
import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[3]


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def rotate(points, quaternion):
    """Active XYZW quaternion rotation, independent of experiment helpers."""
    points = np.asarray(points, dtype=float)
    quaternion = np.asarray(quaternion, dtype=float)
    if quaternion.shape != (4,) or not np.isfinite(quaternion).all():
        raise ValueError("Invalid XYZW quaternion")
    norm = float(np.linalg.norm(quaternion))
    if abs(norm-1.) > 1e-5:
        raise ValueError("Quaternion is not normalized")
    quaternion = quaternion/norm
    vector, scalar = quaternion[:3], quaternion[3]
    return points + 2*np.cross(vector, np.cross(vector, points)+scalar*points)


def position(pose):
    result = np.asarray(pose["position_m"], dtype=float)
    if result.shape != (3,) or not np.isfinite(result).all():
        raise ValueError("Invalid pose position")
    return result


def audit(args):
    run_dir = args.run_dir.resolve()
    repo = args.repo_root.resolve()
    report = {
        "schema_version": 1, "status": "FAIL", "passed": False,
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "run_dir": str(run_dir), "accepted_count": 0, "checked_count": 0,
        "bounds_discrepancy_limit_m": 1e-7,
        "max_bounds_discrepancy_m": None, "min_signed_interior_clearance_m": None,
        "world_contained_count": 0, "cabinet_contained_count": 0,
        "max_world_vs_cabinet_clearance_difference_m": None,
        "max_world_vs_cabinet_vertex_difference_m": None,
        "source_hashes": {}, "trials": [], "errors": [],
        "verification_scope": "Regenerated final visual mesh bounds and enclosure containment only; no collision audit or physics replay.",
        "audit_note": "Cabinet identity is a descriptive diagnostic. Small numerical drift is permitted by the original protocol; it is not an additional acceptance condition. Both world-envelope and measured Cabinet-frame containment are checked at the recorded tolerance. Raw experiment data and thresholds are unchanged.",
    }
    try:
        accepted_file, metadata_file = run_dir/"accepted_poses.json", run_dir/"metadata.json"
        report["input_hashes"] = {"accepted_poses.json": digest(accepted_file),
                                  "metadata.json": digest(metadata_file),
                                  "audit_script": digest(__file__)}
        accepted = json.loads(accepted_file.read_text())
        metadata = json.loads(metadata_file.read_text())
        trials = accepted["trials"]
        report["accepted_count"] = len(trials)
        if not trials:
            raise ValueError("No accepted poses available to audit")
        if metadata.get("run_status") in (None, "running") or not metadata.get("finished_utc"):
            raise ValueError("Run metadata has not been finalized")
        if accepted["pose_frames"]["final_pose"] != "world":
            raise ValueError("Expected world-space final poses")
        source_root = repo/"frigidaire/src"
        for filename in ("tableware.py", "geometry.py"):
            relative = "frigidaire/src/dishsim_frigidaire/"+filename
            current = digest(repo/relative)
            expected = metadata["source_hashes"][relative]
            matches = current == expected
            report["source_hashes"][relative] = {
                "recorded_sha256": expected, "actual_sha256": current, "matches": matches}
            if not matches:
                raise ValueError("Recorded source hash differs: "+relative)
            if filename == "tableware.py" and current != metadata["inputs"]["tableware_source_sha256"]:
                raise ValueError("Tableware provenance hash differs from source hash")
        sys.path.insert(0, str(source_root))
        from dishsim_frigidaire.tableware import tableware_geometry

        interior = metadata["domains"]["interior_bounds"]
        if interior["frame"] != "Cabinet":
            raise ValueError("Expected Cabinet-frame enclosure")
        lower, upper = np.asarray(interior["lower_m"]), np.asarray(interior["upper_m"])
        if lower.shape != (3,) or upper.shape != (3,) or not np.isfinite([lower, upper]).all() or (upper <= lower).any():
            raise ValueError("Invalid enclosure bounds")
        tolerance = float(metadata["thresholds"]["containment_tolerance_m"])
        if not np.isfinite(tolerance) or abs(tolerance-.001) > 1e-12:
            raise ValueError("Expected the recorded 1 mm containment tolerance")
        report["containment_tolerance_m"] = tolerance
        report["interior_bounds"] = interior
        meshes = {}
        for kind in sorted({trial["kind"] for trial in trials}):
            geometry = tableware_geometry(kind)
            meshes[kind] = np.concatenate([np.asarray(visual[0], dtype=float) for visual in geometry["visuals"]])
            if not np.isfinite(meshes[kind]).all():
                raise ValueError("Nonfinite source mesh: "+kind)
        report["visual_vertex_counts"] = {kind: len(mesh) for kind, mesh in meshes.items()}
        report["mug_handle_included"] = "mug" in meshes
        seen = set()
        for trial in trials:
            identifier = trial["trial_id"]
            if identifier in seen or trial["outcome"] != "accepted":
                raise ValueError("Duplicate or non-accepted trial: "+identifier)
            seen.add(identifier)
            pose = trial["final_pose"]
            world = rotate(meshes[trial["kind"]], pose["quaternion_xyzw"]) + position(pose)
            actual_bounds = np.array([world.min(axis=0), world.max(axis=0)])
            recorded_bounds = np.asarray(trial["final_mesh_bounds_m"], dtype=float)
            if recorded_bounds.shape != (2, 3) or not np.isfinite(recorded_bounds).all():
                raise ValueError("Invalid recorded final bounds: "+identifier)
            discrepancy = float(np.max(np.abs(actual_bounds-recorded_bounds)))
            # Enclosure is defined in the stationary cabinet frame. Use the
            # recorded measured frame rather than assuming world and Cabinet agree.
            cabinet = trial["final_hold"]["poses"]["Cabinet"]
            cabinet_shift = float(np.linalg.norm(position(cabinet)))
            cabinet_shift_max_abs = float(np.max(np.abs(position(cabinet))))
            cabinet_rotation_matrix_difference = float(np.max(np.abs(rotate(np.eye(3), cabinet["quaternion_xyzw"])-np.eye(3))))
            cabinet_quaternion = np.asarray(cabinet["quaternion_xyzw"], dtype=float)
            cabinet_angle_rad = float(2*np.arctan2(np.linalg.norm(cabinet_quaternion[:3]), abs(cabinet_quaternion[3])))
            cabinet_identity = cabinet_shift_max_abs <= 1e-7 and cabinet_rotation_matrix_difference <= 1e-7
            inverse_quaternion = np.asarray(cabinet["quaternion_xyzw"])*[-1, -1, -1, 1]
            local = rotate(world-position(cabinet), inverse_quaternion)
            clearance = float(min(np.min(local-lower), np.min(upper-local)))
            world_clearance = float(min(np.min(world-lower), np.min(upper-world)))
            cabinet_contained = bool(((local >= lower-tolerance) & (local <= upper+tolerance)).all())
            world_contained = bool(((world >= lower-tolerance) & (world <= upper+tolerance)).all())
            contained = cabinet_contained and world_contained
            passed = discrepancy <= report["bounds_discrepancy_limit_m"] and contained
            report["trials"].append({"trial_id": identifier, "kind": trial["kind"], "rack": trial["rack"],
                                     "visual_vertex_count": len(world), "bounds_discrepancy_m": discrepancy,
                                     "min_signed_interior_clearance_m": clearance,
                                     "world_min_signed_interior_clearance_m": world_clearance,
                                     "cabinet_min_signed_interior_clearance_m": clearance,
                                     "world_vs_cabinet_clearance_difference_m": abs(world_clearance-clearance),
                                     "world_vs_cabinet_vertex_difference_m": float(np.linalg.norm(world-local, axis=1).max()),
                                     "world_contained": world_contained, "cabinet_contained": cabinet_contained,
                                     "cabinet_translation_norm_m": cabinet_shift,
                                     "cabinet_translation_max_abs_m": cabinet_shift_max_abs,
                                     "cabinet_rotation_angle_rad": cabinet_angle_rad,
                                     "cabinet_rotation_matrix_max_difference": cabinet_rotation_matrix_difference,
                                     "contained": contained, "cabinet_world_identity": cabinet_identity, "passed": passed})
            report["checked_count"] += 1
            if not passed:
                report["errors"].append("Final geometry audit failed: "+identifier)
        report["max_bounds_discrepancy_m"] = max(trial["bounds_discrepancy_m"] for trial in report["trials"])
        report["min_signed_interior_clearance_m"] = min(trial["min_signed_interior_clearance_m"] for trial in report["trials"])
        report["world_min_signed_interior_clearance_m"] = min(trial["world_min_signed_interior_clearance_m"] for trial in report["trials"])
        report["cabinet_min_signed_interior_clearance_m"] = report["min_signed_interior_clearance_m"]
        report["world_contained_count"] = sum(trial["world_contained"] for trial in report["trials"])
        report["cabinet_contained_count"] = sum(trial["cabinet_contained"] for trial in report["trials"])
        report["cabinet_world_identity_diagnostic_count"] = sum(trial["cabinet_world_identity"] for trial in report["trials"])
        for key in ("world_vs_cabinet_clearance_difference_m", "world_vs_cabinet_vertex_difference_m",
                    "cabinet_translation_norm_m", "cabinet_translation_max_abs_m",
                    "cabinet_rotation_angle_rad", "cabinet_rotation_matrix_max_difference"):
            report["max_"+key] = max(trial[key] for trial in report["trials"])
        report["passed"] = not report["errors"] and report["checked_count"] == report["accepted_count"]
        report["status"] = "PASS" if report["passed"] else "FAIL"
    except Exception as exc:
        report["errors"].append(str(exc))
    args.out_file.parent.mkdir(parents=True, exist_ok=True)
    args.out_file.write_text(json.dumps(report, indent=2, sort_keys=True, allow_nan=False)+"\n")
    print(json.dumps({key: report[key] for key in (
        "status", "accepted_count", "checked_count", "max_bounds_discrepancy_m",
        "min_signed_interior_clearance_m", "errors")}, sort_keys=True))
    return 0 if report["passed"] else 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", required=True, type=Path)
    parser.add_argument("--out-file", type=Path, help="Defaults to RUN_DIR/independent_geometry_audit.json")
    parser.add_argument("--repo-root", type=Path, default=ROOT)
    args = parser.parse_args()
    if args.out_file is None:
        args.out_file = args.run_dir/"independent_geometry_audit.json"
    return audit(args)


if __name__ == "__main__":
    raise SystemExit(main())
