#!/usr/bin/env python3
"""Export measured poses and source visual meshes for the offline HTML viewer.

This is a read-only reconstruction of an existing, finalized experiment. It
does not run Isaac, invent new poses, or change the experiment's conclusions.

    scripts/run_py.sh frigidaire/scripts/evaluation/frigidaire_random_pose_viewer_data.py \
        --run-dir results/random_poses/frigidaire/complete_20260910_seed0
"""
from __future__ import annotations

import argparse
from collections import Counter
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import sys

import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[3]
sys.path[:0] = [str(ROOT / "src"), str(ROOT / "frigidaire/src")]

from dishsim_frigidaire import geometry
from dishsim_frigidaire.random_poses import KINDS, RACKS
from frigidaire_random_pose_views import component_triangles, dish_triangles, load_run

COMPONENTS = ("Cabinet", "Door", "LowerRack", "UpperRack", "SilverwareBasket")
OUTCOME_LABELS = {
    "accepted": "Accepted placement",
    "initial_collision": "Rejected before release: initial geometry collision",
    "closure_failure": "Rack or door endpoint check failed",
    "lost_support": "Dish was not supported by the selected rack",
    "outside_dishwasher": "Final dish geometry extended outside the enclosure tolerance",
    "settle_timeout": "A stable rest window was not reached",
    "simulation_error": "Numerically unresolved; feasibility is unknown",
    "interrupted": "Evaluation interrupted; feasibility is unknown",
}


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def pose(value):
    """Preserve the original recorded floats; never quantize measured poses."""
    if value is None:
        return None
    p = np.asarray(value["position_m"], dtype=float)
    q = np.asarray(value["quaternion_xyzw"], dtype=float)
    if p.shape != (3,) or q.shape != (4,) or not np.isfinite(p).all() or not np.isfinite(q).all():
        raise ValueError("Invalid recorded pose")
    if abs(np.linalg.norm(q)-1.) > 1e-5:
        raise ValueError("Recorded quaternion is not approximately unit length")
    return {"position_m": list(value["position_m"]),
            "quaternion_xyzw": list(value["quaternion_xyzw"])}


def mesh(triangles):
    """Losslessly index repeated triangle vertices; source coordinates stay float64."""
    triangles = np.asarray(triangles, dtype=float)
    if triangles.ndim != 3 or triangles.shape[1:] != (3, 3) or not np.isfinite(triangles).all():
        raise ValueError("Invalid visual triangles")
    vertices, indices = np.unique(triangles.reshape(-1, 3), axis=0, return_inverse=True)
    if not np.array_equal(vertices[indices].reshape(triangles.shape), triangles):
        raise AssertionError("Mesh indexing changed source geometry")
    return {"positions": vertices.reshape(-1).tolist(), "indices": indices.tolist(),
            "vertex_count": len(vertices), "triangle_count": len(triangles),
            "bounds": {"min": vertices.min(axis=0).tolist(), "max": vertices.max(axis=0).tolist()},
            "frame": "component_local", "simplified": False}


def measured_stage(trial, name):
    trial_pose = trial.get(name + "_pose")
    if trial_pose is None:
        return None
    window = trial.get("final_hold" if name == "final" else "settle", {})
    frames = {key: pose(value) for key, value in window.get("poses", {}).items()
              if key in COMPONENTS}
    if set(frames) != set(COMPONENTS):
        raise ValueError("Measured stage lacks component frames: " + trial["trial_id"])
    saved_endpoint = pose(trial_pose)
    dish_pose = pose(window["poses"][trial["kind"]])
    if trial["outcome"] == "accepted" and dish_pose != saved_endpoint:
        raise ValueError("Accepted dish and scene frame timestamps disagree: " + trial["trial_id"])
    return {"dish": dish_pose, "frames": frames,
            "saved_endpoint_dish": saved_endpoint,
            "endpoint_matches_scene_snapshot": saved_endpoint == dish_pose,
            "frame_provenance": "Exact complete measured scene snapshot from the saved stage window. For failed trials, the separately recorded dish endpoint can occur a few ticks later; that standalone pose is also exported.",
            "label": "After rack closure attempt" if name == "final" else "After settling attempt, rack open",
            "accepted": trial["outcome"] == "accepted" and name == "final",
            "rest_window_passed": bool(window.get("settled")),
            "joints": window.get("joints", {})}


def export(run_dir, usd_path, output_dir):
    run_dir, output_dir = run_dir.resolve(), output_dir.resolve()
    allowed = (ROOT / "outputs/random_pose_viewer").resolve()
    if output_dir != allowed and allowed not in output_dir.parents:
        raise ValueError("Viewer data must be written under outputs/random_pose_viewer/")
    if run_dir == output_dir or run_dir in output_dir.parents:
        raise ValueError("Never write visualization output into finalized experiment data")
    metadata, summary, _ = load_run(run_dir, usd_path)
    rows = [json.loads(line) for line in (run_dir / "trials.jsonl").read_text().splitlines() if line.strip()]
    accepted = json.loads((run_dir / "accepted_poses.json").read_text())["trials"]
    audit = json.loads((run_dir / "independent_geometry_audit.json").read_text())
    analysis = json.loads((run_dir / "analysis.json").read_text())
    if audit.get("status") != "PASS" or analysis.get("status") != "PASS":
        raise ValueError("The accepted geometry and evidence audits must both pass")
    for name, expected in audit["input_hashes"].items():
        if name in ("accepted_poses.json", "metadata.json") and digest(run_dir / name) != expected:
            raise ValueError("Geometry audit refers to different input bytes: " + name)
    if len(rows) != summary["totals"]["attempted"] or len({r["trial_id"] for r in rows}) != len(rows):
        raise ValueError("Trial rows/counts are inconsistent")
    if {r["trial_id"]: r for r in rows if r["outcome"] == "accepted"} != {r["trial_id"]: r for r in accepted}:
        raise ValueError("Accepted replay records differ from raw trials")
    audited = {row["trial_id"]: row for row in audit["trials"]}
    evidence = {row["trial_id"]: row for row in analysis["audits"]}
    if set(audited) != {r["trial_id"] for r in accepted} or set(evidence) != set(audited):
        raise ValueError("Audited IDs differ from accepted IDs")

    components = geometry.build_components()
    meshes = {name: mesh(component_triangles(components[name])) for name in COMPONENTS}
    meshes.update({kind: mesh(dish_triangles(kind)) for kind in KINDS})
    for kind in KINDS:
        if meshes[kind]["triangle_count"] != metadata["inputs"]["catalog"][kind]["visual_triangles"]:
            raise ValueError("Visual triangle count differs from staged catalog: " + kind)

    trials = []
    maximum_release_discrepancy = 0.
    for trial in rows:
        local, rack_pose, released = trial["sampled_pose"], trial["initial_rack_pose"], trial["released_pose"]
        # Independent composition check ensures rack-local samples are not
        # accidentally rendered in world coordinates with their rack offset lost.
        rack_rotation = Rotation.from_quat(rack_pose["quaternion_xyzw"])
        released_position = rack_rotation.apply(local["position_m"]) + np.asarray(rack_pose["position_m"])
        released_rotation = rack_rotation * Rotation.from_quat(local["quaternion_xyzw"])
        position_error = float(np.max(np.abs(released_position-np.asarray(released["position_m"]))))
        rotation_error = float((released_rotation.inv()*Rotation.from_quat(released["quaternion_xyzw"])).magnitude())
        maximum_release_discrepancy = max(maximum_release_discrepancy, position_error)
        if position_error > 1e-12 or rotation_error > 1e-12:
            raise ValueError("Saved release is not the composed sampled pose: " + trial["trial_id"])
        item_audit = audited.get(trial["trial_id"])
        strict = (item_audit["world_min_signed_interior_clearance_m"] >= 0
                  and item_audit["cabinet_min_signed_interior_clearance_m"] >= 0) if item_audit else None
        final, settled = measured_stage(trial, "final"), measured_stage(trial, "settled")
        metrics = {key: trial.get(key) for key in (
            "maximum_cycle_penetration_m", "maximum_settle_penetration_m", "wall_seconds")}
        metrics.update({"world_clearance_m": item_audit["world_min_signed_interior_clearance_m"] if item_audit else None,
                        "cabinet_clearance_m": item_audit["cabinet_min_signed_interior_clearance_m"] if item_audit else None,
                        "final_endpoint_error_m": evidence.get(trial["trial_id"], {}).get("final_saved_endpoint_error_m"),
                        "settled_peak_penetration_m": trial.get("settle", {}).get("peak_penetration_m"),
                        "settled_median_penetration_m": trial.get("settle", {}).get("median_max_penetration_m")})
        trials.append({
            "id": trial["trial_id"], "kind": trial["kind"], "rack": trial["rack"],
            "index": trial["sample_index"], "outcome": trial["outcome"], "strict": strict,
            "reason": trial.get("reason") or OUTCOME_LABELS.get(trial["outcome"], trial["outcome"]),
            "outcome_label": OUTCOME_LABELS.get(trial["outcome"], trial["outcome"]),
            "unresolved": trial["outcome"] in ("simulation_error", "interrupted"),
            "sampled_local": local,
            "sampled": {"dish": pose(released), "frames": {trial["rack"]: pose(rack_pose)},
                        "label": ("Rejected initial proposal, never physically dropped" if trial["outcome"] == "initial_collision"
                                  else "Initial release pose, rack open"),
                        "frame_provenance": "Exact saved proposed world pose and selected rack initialization pose. The proposed world pose is computed before collision rejection; its presence alone does not mean a physical drop occurred. Other component reset poses were not saved and are omitted.",
                        "released_into_physics": trial["outcome"] != "initial_collision",
                        "joints": trial.get("reset_joints", {})},
            "settled": settled, "final": final, "metrics": metrics,
            "closure_attempted": bool(trial.get("rack_motion")),
            "trace_file": trial.get("trace_file"),
        })
    strict_count = sum(trial["strict"] is True for trial in trials)
    cells = []
    for cell in summary["cells"]:
        cells.append({**cell, "strict": sum(t["strict"] is True and t["kind"] == cell["kind"]
                                            and t["rack"] == cell["rack"] for t in trials)})
    interior = metadata["domains"]["interior_bounds"]
    data = {
        "schema_version": 1,
        "meta": {
            "run_id": run_dir.name, "created_utc": datetime.now(timezone.utc).isoformat(),
            "experiment_finished_utc": metadata["finished_utc"], "seed": metadata["seed"],
            "units": "metres", "up_axis": "Z", "quaternion_order": "XYZW", "pose_frame": "world",
            "kinds": list(KINDS), "racks": list(RACKS), "catalog": metadata["inputs"]["catalog"],
            "new_simulation": False, "isaac_render": False, "door_closure_tested": False,
            "interpretation": "All tested samples, not every possible pose. Each is an independent one-dish trial; counts are not unique resting arrangements or simultaneous capacity.",
            "reconstruction": "Source visual triangles at exact saved poses. Authored box primitives are exact; analytic cylinders use 32 radial segments. No source triangle mesh was simplified.",
            "sampled_context": "Only the selected rack has an exact saved reset transform; other components are omitted in the release view. The enclosure outline is the nominal world envelope.",
            "thresholds": metadata["thresholds"], "runtime": metadata.get("runtime", {}),
            "source_run_relative": str(run_dir.relative_to(ROOT.resolve())) if ROOT.resolve() in run_dir.parents else str(run_dir),
            "source_hashes": {name: digest(run_dir / name) for name in (
                "metadata.json", "summary.json", "trials.jsonl", "accepted_poses.json", "analysis.json", "independent_geometry_audit.json")},
            "exporter_sha256": digest(__file__),
            "geometry_source_sha256": {name: digest(ROOT / name) for name in (
                "frigidaire/src/dishsim_frigidaire/geometry.py", "frigidaire/src/dishsim_frigidaire/tableware.py",
                "frigidaire/scripts/evaluation/frigidaire_random_pose_views.py")},
            "maximum_release_composition_error_m": maximum_release_discrepancy,
            "audits": {"accepted_geometry": "PASS", "accepted_evidence": "PASS", "scope": "Accepted records only; numerical failures remain unresolved"},
        },
        "summary": {"totals": {**summary["totals"], "strict": strict_count}, "cells": cells,
                    "outcomes": dict(Counter(t["outcome"] for t in trials)),
                    "stages": {stage: sum(t[stage] is not None for t in trials) for stage in ("sampled", "settled", "final")}},
        "bounds": {"min": interior["lower_m"], "max": interior["upper_m"], "frame": "Cabinet", "display_frame": "nominal_world"},
        "sampling_bounds": metadata["domains"]["sampling_bounds_by_rack"],
        "meshes": meshes, "trials": trials,
    }
    output_dir.mkdir(parents=True, exist_ok=True)
    output = output_dir / "viewer_data.json"
    output.write_text(json.dumps(data, separators=(",", ":"), allow_nan=False)+"\n")
    print(json.dumps({"output": str(output), "bytes": output.stat().st_size,
                      "attempted": len(trials), "accepted": len(accepted), "strict": strict_count,
                      "stages": data["summary"]["stages"],
                      "triangles": {name: m["triangle_count"] for name, m in meshes.items()}}, indent=2))
    return output


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--run-dir", required=True, type=Path)
    parser.add_argument("--usd", type=Path, default=ROOT / "build/frigidaire_collection/usd/fdpc4221as.usdc")
    parser.add_argument("--output-dir", type=Path, default=ROOT / "outputs/random_pose_viewer")
    args = parser.parse_args()
    export(args.run_dir, args.usd, args.output_dir)


if __name__ == "__main__":
    main()
