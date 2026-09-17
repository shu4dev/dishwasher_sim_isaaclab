#!/usr/bin/env python3
# Copyright (c) 2026, dishsim project.
# SPDX-License-Identifier: BSD-3-Clause
"""Open the exported loaded USD directly and verify its authored closed hold.

    scripts/run_kit.sh frigidaire/scripts/evaluation/frigidaire_loaded_scene_smoke.py \
        --headless --device cpu --scene assets/models/frigidaire_fdpc4221as/usd/full_load.usda

This smoke test uses no appliance spawning helper, drive-update helper, pose reset,
or per-frame control commands. Isaac Core uses ``set_defaults=False`` so the USD's
120 Hz physics scene, gravity, and closed joint drives remain authoritative.
"""
import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
import sys
import time

from isaaclab.app import AppLauncher

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "frigidaire/src"))
from dishsim_frigidaire.paths import ASSET_DIR, IMAGE_DIR
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--scene", type=Path, default=ASSET_DIR / "full_load.usda")
parser.add_argument("--settled", type=Path, help="Defaults to full_load_settled.json beside the scene")
parser.add_argument("--out-dir", type=Path, default=IMAGE_DIR / "full_load")
parser.add_argument("--expected-count", type=int, default=67)
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
if args.device != "cpu":
    parser.error("The portable-scene smoke test uses CPU physics; pass --device cpu")
args.scene = args.scene.resolve()
args.settled = (args.settled or args.scene.with_name("full_load_settled.json")).resolve()
args.out_dir = args.out_dir.resolve()
started = time.monotonic()
app = AppLauncher(args).app

import numpy as np  # noqa: E402
from scipy.spatial.transform import Rotation  # noqa: E402
from pxr import PhysxSchema, Usd, UsdGeom, UsdPhysics, UsdUtils  # noqa: E402

from dishsim.media import release_sim_for_close  # noqa: E402

PREFIX = "/LoadedFrigidaire"
APPLIANCE = PREFIX + "/Appliance"
CONTACT_CAPACITY = 262144
CONTACT_PERSISTENCE_M = 1e-6
JOINT_TOLERANCES = {"door_hinge": math.radians(.5), "lower_slide": .005, "upper_slide": .005}
report = {"result": "INCOMPLETE", "scene": str(args.scene), "settled": str(args.settled),
          "scope": "Direct exported USD import and three-second authored closed hold; no helper control",
          "thresholds": {"maximum_rack_relative_axis_drift_m": .010,
                         "final_second_axis_span_m": .005, "minimum_relative_origin_z_m": -.035,
                         "retained_contact_surface_motion_m": CONTACT_PERSISTENCE_M,
                         "joint_endpoints": JOINT_TOLERANCES}, "gates": []}
sim = None


def save():
    args.out_dir.mkdir(parents=True, exist_ok=True)
    report["wall_seconds"] = time.monotonic() - started
    (args.out_dir / "standalone_smoke.json").write_text(json.dumps(report, indent=2) + "\n")


def gate(name, passed, **details):
    report["gates"].append({"name": name, "passed": bool(passed), **details})
    print(f"[{'OK' if passed else 'FAIL'}] {name}: {json.dumps(details)}", flush=True)


def check_budget():
    if time.monotonic() - started > 5 * 60:
        raise TimeoutError("Standalone smoke exceeded its five-minute wall-clock budget")


def authored_settings(stage):
    scene = stage.GetPrimAtPath(PREFIX + "/Physics")
    settings = {"scene": {str(attr.GetName()): str(attr.Get()) for attr in scene.GetAttributes()
                           if attr.GetName().startswith(("physics:", "physxScene:"))}, "drives": {}}
    for name in JOINT_TOLERANCES:
        prim = stage.GetPrimAtPath(APPLIANCE + "/Joints/" + name)
        settings["drives"][name] = {str(attr.GetName()): str(attr.Get()) for attr in prim.GetAttributes()
                                   if attr.GetName().startswith(("drive:", "physxJoint:"))}
    return settings


def select_opened_scene(stage, manager):
    """Populate Isaac 4.5's runtime scene registry without authoring USD.

    In 4.5, stage-open clears this registry and installs a listener for *new*
    PhysicsScene prims. An already composed scene can consequently be absent.
    Both warmup and PhysicsContext._step read the registry, falling back to
    60 Hz when it is empty. This release has no public scene-selection setter.
    Refresh its process-local registry from the sole existing scene, using a
    typed wrapper rather than PhysxSceneAPI.Apply (which would author USD).
    """
    scene_paths = [str(prim.GetPath()) for prim in stage.Traverse() if prim.IsA(UsdPhysics.Scene)]
    expected_path = PREFIX + "/Physics"
    if scene_paths != [expected_path]:
        raise RuntimeError(f"Expected one authored PhysicsScene, found {scene_paths}")
    registry = manager._physics_scene_apis
    before = []
    for path, api in registry.items():
        prim = api.GetPrim()
        before.append({"path": str(path), "valid": bool(prim),
                       "current_stage": bool(prim) and prim.GetStage() == stage,
                       "physics_hz": api.GetTimeStepsPerSecondAttr().Get() if prim else None})
    registry.clear()
    registry[expected_path] = PhysxSchema.PhysxSceneAPI(stage.GetPrimAtPath(expected_path))
    report["runtime_scene_selection"] = {
        "compatibility": "Isaac Sim 4.5 existing-stage registry refresh; no USD parameters written",
        "registered_before": before, "selected_scene": expected_path,
        "selected_physics_dt": manager.get_physics_dt(expected_path),
        "default_physics_dt": manager.get_physics_dt(),
    }


def surface_motion_bound(reference, current, radius):
    """Conservative motion of any point within radius of a rigid body origin."""
    reference, current = np.asarray(reference, dtype=float), np.asarray(current, dtype=float)
    before = reference[3:] / np.linalg.norm(reference[3:])
    after = current[3:] / np.linalg.norm(current[3:])
    quaternion_chord = min(np.linalg.norm(after-before), np.linalg.norm(after+before))
    return float(np.linalg.norm(current[:3]-reference[:3]) + 2*radius*quaternion_chord)


def main():
    global sim
    import carb.settings
    import omni.usd
    from isaacsim.core.api import SimulationContext
    from isaacsim.core.api.sensors import RigidContactView
    from isaacsim.core.simulation_manager import SimulationManager

    manifest = json.loads(args.settled.read_text())
    entries = manifest["objects"]
    identities = [entry["id"] for entry in entries]
    expected_counts = dict(Counter(entry["kind"] for entry in entries))
    gate("completed settled manifest", manifest.get("physics_result") == "PASS"
         and manifest.get("validated_counts", {}).get("total") == len(entries)
         and manifest.get("validated_counts", {}).get("by_type") == expected_counts
         and len(set(identities)) == len(entries) == args.expected_count,
         total=len(entries), expected_total=args.expected_count, by_type=expected_counts)
    if not all(item["passed"] for item in report["gates"]):
        raise ValueError("The exported settled manifest is not the requested validated load")
    report["input_sha256"] = {args.scene.name: hashlib.sha256(args.scene.read_bytes()).hexdigest(),
                              args.settled.name: hashlib.sha256(args.settled.read_bytes()).hexdigest()}
    report["script_sha256"] = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    if not omni.usd.get_context().open_stage(str(args.scene)):
        raise RuntimeError("Isaac could not open the exported scene")
    app.update()
    stage = omni.usd.get_context().get_stage()
    layers, assets, unresolved = UsdUtils.ComputeAllDependencies(str(args.scene))
    absolute_refs = [str(reference) for layer in layers for reference in layer.GetExternalReferences()
                     if Path(str(reference)).is_absolute() or "://" in str(reference)]
    gate("portable dependencies resolve", not unresolved and not absolute_refs,
         layer_count=len(layers), unresolved=list(map(str, unresolved)), absolute_references=absolute_refs)
    default = stage.GetDefaultPrim()
    physics_prim = stage.GetPrimAtPath(PREFIX + "/Physics")
    physics = PhysxSchema.PhysxSceneAPI(physics_prim)
    hz = int(physics.GetTimeStepsPerSecondAttr().Get())
    gate("authored CPU scene at 120 Hz", str(default.GetPath()) == PREFIX and hz == 120
         and UsdGeom.GetStageMetersPerUnit(stage) == 1. and UsdGeom.GetStageUpAxis(stage) == "Z"
         and physics.GetEnableGPUDynamicsAttr().Get() is False
         and physics.GetEnableCCDAttr().Get() is True,
         default_prim=str(default.GetPath()), physics_hz=hz,
         gpu_dynamics=physics.GetEnableGPUDynamicsAttr().Get(),
         scene_ccd=physics.GetEnableCCDAttr().Get())
    bodies = {str(prim.GetPath()) for prim in stage.Traverse()
              if prim.HasAPI(UsdPhysics.RigidBodyAPI)
              and UsdPhysics.RigidBodyAPI(prim).GetRigidBodyEnabledAttr().Get() is not False}
    expected_paths = {PREFIX + "/Tableware/" + identity for identity in identities}
    authored_paths = {path for path in bodies if path.startswith(PREFIX + "/Tableware/")}
    static_dishes = [path for path in authored_paths
                     if UsdPhysics.RigidBodyAPI(stage.GetPrimAtPath(path)).GetKinematicEnabledAttr().Get()]
    gate("every counted object is an independent dynamic body", authored_paths == expected_paths
         and not static_dishes, tableware_body_count=len(authored_paths), kinematic_dishes=static_dishes)
    drive_records = {}
    for name in JOINT_TOLERANCES:
        prim = stage.GetPrimAtPath(APPLIANCE + "/Joints/" + name)
        drive = UsdPhysics.DriveAPI(prim, "angular" if name == "door_hinge" else "linear")
        drive_records[name] = {"target": drive.GetTargetPositionAttr().Get(),
                               "stiffness": drive.GetStiffnessAttr().Get(),
                               "damping": drive.GetDampingAttr().Get(),
                               "max_force": drive.GetMaxForceAttr().Get()}
    gate("finite authored closed-hold drives", all(d["target"] == 0. and d["stiffness"] > 0.
         and d["damping"] > 0. and 0. < d["max_force"] < 1e5 for d in drive_records.values()),
         drives=drive_records)
    if not all(item["passed"] for item in report["gates"]):
        raise ValueError("Export integrity checks failed")
    settings_before = authored_settings(stage)
    cache = UsdGeom.XformCache()
    initial_basket = np.asarray(cache.GetLocalToWorldTransform(
        stage.GetPrimAtPath(APPLIANCE + "/SilverwareBasket"))).T
    initial_lower = np.asarray(cache.GetLocalToWorldTransform(stage.GetPrimAtPath(APPLIANCE + "/LowerRack"))).T
    basket_relative_reference = initial_lower[:3, :3].T @ (initial_basket[:3, 3]-initial_lower[:3, 3])

    # Preserve the loaded scene's configuration. Only observation/reporting is
    # enabled; no rigid body, drive, force, pose, or gain is written here.
    carb.settings.get_settings().set_bool("/physics/disableContactProcessing", False)
    select_opened_scene(stage, SimulationManager)
    SimulationManager.set_physics_sim_device("cpu")
    # Supply the existing scene's rate to Core's rendering/timeline bookkeeping
    # as well. The selected registry above controls actual warmup/physics steps.
    sim = SimulationContext(set_defaults=False, physics_prim_path=PREFIX + "/Physics",
                            physics_dt=1/hz, rendering_dt=1/hz, backend="numpy", device="cpu")
    contact_paths = [APPLIANCE + "/" + name for name in
                     ("Cabinet", "Door", "LowerRack", "UpperRack", "SilverwareBasket")]
    contact_paths.extend(PREFIX + "/Tableware/" + identity for identity in identities)
    sleep_settings = {path: stage.GetPrimAtPath(path).GetAttribute("physxRigidBody:sleepThreshold").Get()
                      for path in contact_paths}
    contacts = RigidContactView(prim_paths_expr=contact_paths,
                               filter_paths_expr=[contact_paths.copy() for _ in contact_paths],
                               name="standalone_loaded_contacts", max_contact_count=CONTACT_CAPACITY,
                               prepare_contact_sensors=True, disable_stablization=False)
    bbox = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render", "proxy", "guide"], False, True)
    body_radii = {}
    for path in contact_paths:
        bounds = bbox.ComputeUntransformedBound(stage.GetPrimAtPath(path)).ComputeAlignedBox()
        body_radii[path] = float(np.linalg.norm(np.maximum(np.abs(bounds.GetMin()), np.abs(bounds.GetMax()))))
    gate("contact observation prepared without changing sleep settings",
         all(stage.GetPrimAtPath(path).HasAPI(PhysxSchema.PhysxContactReportAPI)
             and PhysxSchema.PhysxContactReportAPI(stage.GetPrimAtPath(path)).GetThresholdAttr().Get() == 0.
             and math.isfinite(radius) and radius > 0. for path, radius in body_radii.items())
         and carb.settings.get_settings().get_as_bool("/physics/disableContactProcessing") is False,
         prepared_bodies=len(body_radii), maximum_body_radius_m=max(body_radii.values()),
         disable_contact_processing=carb.settings.get_settings().get_as_bool("/physics/disableContactProcessing"))
    sim.reset()
    contacts.initialize()
    view = sim.physics_sim_view
    dishes = view.create_rigid_body_view(PREFIX + "/Tableware/*")
    basket = view.create_rigid_body_view(APPLIANCE + "/SilverwareBasket")
    appliance = view.create_articulation_view(APPLIANCE + "/Cabinet")
    dish_index = {path.rsplit("/", 1)[-1]: index for index, path in enumerate(dishes.prim_paths)}
    link_index = {name: index for index, name in enumerate(appliance.shared_metatype.link_names)}
    joint_index = {name: index for index, name in enumerate(appliance.shared_metatype.dof_names)}
    gate("physics views include all exported bodies and joints", set(dish_index) == set(identities)
         and set(joint_index) == set(JOINT_TOLERANCES), tableware_count=len(dish_index), joints=list(joint_index))
    dt = sim.get_physics_dt()
    settings_after = authored_settings(stage)
    changes = []
    for category in ("scene", "drives"):
        before, after = settings_before[category], settings_after[category]
        for name in sorted(set(before) | set(after)):
            if before.get(name) != after.get(name):
                changes.append({"category": category, "name": name,
                                "authored": before.get(name), "runtime": after.get(name)})
    gate("runtime honors authored physics and drives", abs(dt-1/hz) < 1e-12
         and abs(SimulationManager.get_physics_dt(PREFIX + "/Physics")-1/hz) < 1e-12
         and not changes, physics_dt=dt, authored_physics_dt=1/hz, changed_settings=changes,
         runtime_scene_selection=report["runtime_scene_selection"])
    if not all(item["passed"] for item in report["gates"]):
        raise ValueError("Runtime changed the exported configuration or omitted bodies")

    records = {identity: {"maximum_axis_drift_m": 0., "minimum_relative_z_m": math.inf,
                           "final_second_positions": []} for identity in identities}
    joint_peaks = dict.fromkeys(JOINT_TOLERANCES, 0.)
    basket_peak = 0.
    contact_edges, retained_contacts, maximum_contacts, overflow = set(), {}, 0, False
    contact_sample_count = 0
    contact_count_by_step = []
    edge_count_by_step = []
    # Observe the actual PhysX step events independently of Core's dt getter.
    # A correct USD attribute alone does not prove the simulator used that dt.
    observed_step_durations = []
    sim.add_physics_callback("standalone_step_timing", lambda value: observed_step_durations.append(float(value)))
    for step in range(3*hz):
        check_budget()
        sim.step(render=False)
        transforms = np.asarray(dishes.get_transforms()).copy()
        link_transforms = np.asarray(appliance.get_link_transforms())[0].copy()
        basket_transform = np.asarray(basket.get_transforms())[0].copy()
        q = np.asarray(appliance.get_dof_positions())[0]
        frames = {name: link_transforms[link_index[name]] for name in ("LowerRack", "UpperRack")}
        frames["SilverwareBasket"] = basket_transform
        body_poses = {APPLIANCE + "/" + name: link_transforms[link_index[name]]
                      for name in ("Cabinet", "Door", "LowerRack", "UpperRack")}
        body_poses[APPLIANCE + "/SilverwareBasket"] = basket_transform
        body_poses.update({PREFIX + "/Tableware/" + identity: transforms[index]
                           for identity, index in dish_index.items()})
        if (not np.isfinite(transforms).all() or not np.isfinite(q).all()
                or not np.isfinite(link_transforms).all() or not np.isfinite(basket_transform).all()
                or any(np.linalg.norm(pose[3:]) < .99 for pose in body_poses.values())):
            raise RuntimeError("Nonfinite standalone physics state")
        for name in joint_peaks:
            joint_peaks[name] = max(joint_peaks[name], abs(float(q[joint_index[name]])))
        lower = frames["LowerRack"]
        basket_relative = Rotation.from_quat(lower[3:]).inv().apply(basket_transform[:3]-lower[:3])
        basket_peak = max(basket_peak, float(np.max(np.abs(basket_relative-basket_relative_reference))))
        for entry in entries:
            identity = entry["id"]
            pose = transforms[dish_index[identity]]
            frame = frames[entry["rack"]]
            relative = Rotation.from_quat(frame[3:]).inv().apply(pose[:3]-frame[:3])
            reference = manifest["settled_objects"][identity]["rack_relative_position_m"]
            item = records[identity]
            item["maximum_axis_drift_m"] = max(item["maximum_axis_drift_m"], float(np.max(np.abs(relative-reference))))
            item["minimum_relative_z_m"] = min(item["minimum_relative_z_m"], float(relative[2]))
            if step >= 2*hz:
                item["final_second_positions"].append(pose[:3].tolist())
        # Sleeping pairs stop producing fresh contact reports. Retain a measured
        # pair only while both actual rigid poses preserve their entire geometry
        # to 1 micrometre; movement invalidates it until a new contact is seen.
        # Sampling starts at the first step and never wakes or edits a body.
        for pair, evidence in list(retained_contacts.items()):
            motion = max(surface_motion_bound(reference, body_poses[path], body_radii[path])
                         for path, reference in zip(pair, evidence["poses"]))
            if motion > CONTACT_PERSISTENCE_M:
                del retained_contacts[pair]
            else:
                evidence["maximum_surface_motion_since_contact_m"] = max(
                    evidence["maximum_surface_motion_since_contact_m"], motion)
        data = contacts.get_contact_force_data(dt=dt, clone=False)
        if data is None:
            raise RuntimeError("Standalone contact observations are unavailable")
        count_raw, start_raw = np.asarray(data[4]), np.asarray(data[5])
        buffers = [np.asarray(buffer) for buffer in data[:4]]
        forces, separations = buffers[0].reshape(-1), buffers[3].reshape(-1)
        if (count_raw.size != len(contact_paths)**2 or start_raw.shape != count_raw.shape
                or not np.isfinite(count_raw).all() or not np.isfinite(start_raw).all()
                or (count_raw < 0).any() or (start_raw < 0).any()
                or (count_raw > CONTACT_CAPACITY).any() or (start_raw > len(separations)).any()
                or not np.equal(count_raw, np.floor(count_raw)).all()
                or not np.equal(start_raw, np.floor(start_raw)).all()):
            raise RuntimeError("Invalid standalone contact counts or start indices")
        count = count_raw.astype(np.int64).reshape(len(contact_paths), len(contact_paths))
        starts = start_raw.astype(np.int64).reshape(count.shape)
        active = count > 0
        if any(np.any((starts+count)[active] > len(buffer)) for buffer in buffers):
            raise RuntimeError("Standalone contact range exceeds a contact buffer")
        for start, length in zip(starts[active], count[active]):
            if any(not np.isfinite(buffer[start:start+length]).all() for buffer in buffers):
                raise RuntimeError("Nonfinite active standalone contact data")
        contact_sample_count += 1
        contact_count_by_step.append(int(count.sum()))
        maximum_contacts = max(maximum_contacts, int(count.sum()))
        overflow = overflow or int(count.sum()) >= CONTACT_CAPACITY
        observed_pairs = set()
        for first, second in np.argwhere(count > 0):
            if first != second:
                pair = tuple(sorted((contact_paths[first], contact_paths[second])))
                observed_pairs.add(pair)
                if step >= 2*hz:
                    contact_edges.add(pair)
                begin, size = starts[first, second], count[first, second]
                retained_contacts[pair] = {"poses": [body_poses[path].copy() for path in pair],
                                           "last_observed_step": step+1,
                                           "contact_point_count": int(size),
                                           "minimum_separation_m": float(separations[begin:begin+size].min()),
                                           "maximum_normal_force_n": float(forces[begin:begin+size].max()),
                                           "maximum_surface_motion_since_contact_m": 0.}
        edge_count_by_step.append(len(observed_pairs))
        if (step+1) % hz == 0:
            print(f"[PROGRESS] Standalone authored hold {(step+1)/hz:.0f}/3 simulated seconds", flush=True)
    step_durations = np.asarray(observed_step_durations)
    gate("actual PhysX events cover three seconds at authored rate",
         len(step_durations) == 3*hz and np.isfinite(step_durations).all()
         and np.all(np.abs(step_durations-1/hz) < 1e-8),
         expected_step_count=3*hz, observed_step_count=len(step_durations),
         observed_seconds=float(step_durations.sum()),
         minimum_step_dt=float(step_durations.min()) if len(step_durations) else None,
         maximum_step_dt=float(step_durations.max()) if len(step_durations) else None)
    adjacency = {path: set() for path in contact_paths}
    contact_supported = set()
    for first, second in retained_contacts:
        adjacency[first].add(second)
        adjacency[second].add(first)
        contact_supported.update((first, second))

    def connected(source, target, allowed):
        pending, visited = [source], set()
        while pending:
            node = pending.pop()
            if node == target:
                return True
            if node not in visited:
                visited.add(node)
                pending.extend(adjacency[node] & allowed - visited)
        return False

    rack_members = {name: {PREFIX + "/Tableware/" + entry["id"] for entry in entries if entry["rack"] == name}
                    for name in ("LowerRack", "UpperRack", "SilverwareBasket")}
    entry_racks = {entry["id"]: entry["rack"] for entry in entries}
    failures = {}
    for identity, item in records.items():
        item["final_second_axis_span_m"] = float(np.ptp(item.pop("final_second_positions"), axis=0).max())
        object_path = PREFIX + "/Tableware/" + identity
        rack = entry_racks[identity]
        target = APPLIANCE + "/" + rack
        item["external_support_contact"] = object_path in contact_supported
        item["connected_to_assigned_rack"] = connected(object_path, target, rack_members[rack] | {target})
        reasons = []
        if item["maximum_axis_drift_m"] > .010:
            reasons.append("rack-relative drift exceeds 10 mm")
        if item["minimum_relative_z_m"] <= -.035:
            reasons.append("origin fell below its rack floor")
        if item["final_second_axis_span_m"] >= .005:
            reasons.append("final-second span exceeds 5 mm")
        if not item["connected_to_assigned_rack"]:
            reasons.append("no contact path to assigned rack through same-rack tableware")
        if reasons:
            failures[identity] = reasons
    gate("all tableware remain supported and close to exported poses", not failures,
         object_count=len(records), failures=failures,
         maximum_axis_drift_m=max(r["maximum_axis_drift_m"] for r in records.values()),
         largest_final_second_span_m=max(r["final_second_axis_span_m"] for r in records.values()))
    basket_path, lower_path = APPLIANCE + "/SilverwareBasket", APPLIANCE + "/LowerRack"
    basket_supported = connected(basket_path, lower_path, rack_members["LowerRack"] | {basket_path, lower_path})
    gate("removable basket remains seated", basket_peak <= .010 and basket_supported,
         maximum_lower_rack_relative_axis_drift_m=basket_peak, contact_path_to_lower_rack=basket_supported)
    gate("authored drives keep all joints closed", all(joint_peaks[n] <= JOINT_TOLERANCES[n] for n in joint_peaks),
         maximum_absolute_joint_positions=joint_peaks)
    gate("support contact buffers valid", not overflow and maximum_contacts > 0,
         maximum_contacts=maximum_contacts, capacity=CONTACT_CAPACITY,
         samples=contact_sample_count, finite_nonnegative_indices=True, active_ranges_within_buffer=True)
    gate("authored scene and drives unchanged", authored_settings(stage) == settings_before)
    gate("authored body sleep thresholds unchanged", all(
        stage.GetPrimAtPath(path).GetAttribute("physxRigidBody:sleepThreshold").Get() == value
        for path, value in sleep_settings.items()), body_count=len(sleep_settings))
    report["objects"] = records
    report["support_graph"] = {"final_second_edges": [list(pair) for pair in sorted(contact_edges)],
                               "sample_count": contact_sample_count,
                               "contact_count_by_step": contact_count_by_step,
                               "actual_edge_count_by_step": edge_count_by_step,
                               "final_actual_edge_count": edge_count_by_step[-1],
                               "final_retained_edge_count": len(retained_contacts),
                               "final_edges_retained_from_earlier_step": sum(
                                   evidence["last_observed_step"] < 3*hz for evidence in retained_contacts.values()),
                               "body_radius_bounds_m": body_radii,
                               "retained_edges": [{"bodies": list(pair),
                                                   "last_observed_step": evidence["last_observed_step"],
                                                   "last_observed_seconds": evidence["last_observed_step"]/hz,
                                                   "contact_point_count": evidence["contact_point_count"],
                                                   "minimum_separation_m": evidence["minimum_separation_m"],
                                                   "maximum_normal_force_n": evidence["maximum_normal_force_n"],
                                                   "maximum_surface_motion_since_contact_m": evidence["maximum_surface_motion_since_contact_m"]}
                                                  for pair, evidence in sorted(retained_contacts.items())],
                               "policy": "Contacts sampled every actual physics step from initialization. A measured edge survives report disappearance only while both endpoint geometries remain within 1 micrometre of their last observed contact poses at every subsequent step. Each object must reach its assigned rack through same-rack tableware only. Cutlery targets the basket; the basket must reach the lower rack through lower-rack tableware or direct contact. No wake or sleep-parameter commands."}
    report["simulated_seconds"] = 3.
    report["result"] = "PASS" if all(item["passed"] for item in report["gates"]) else "FAIL"
    save()
    print(f"[RESULT] {report['result']}: direct loaded USD standalone smoke", flush=True)


try:
    main()
except Exception as error:
    report["result"] = "FAIL"
    report["error"] = str(error)
    save()
    print(f"[RESULT] FAIL: direct loaded USD standalone smoke: {error}", flush=True)
    raise
finally:
    release_sim_for_close(sim)
    app.close()
