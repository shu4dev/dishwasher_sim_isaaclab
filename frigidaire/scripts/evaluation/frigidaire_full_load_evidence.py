# Copyright (c) 2026, dishsim project.
# SPDX-License-Identifier: BSD-3-Clause
"""Validate and photograph a fixed Frigidaire mixed-load manifest in Isaac Sim.

The physics job measures independent rigid dishes through a complete rack/door cycle.
Render jobs replay its recorded physical states; they never invent a successful load.
``--settle-only`` produces placement diagnostics, not a capacity certification.
"""
import argparse
from collections import Counter
import hashlib
import html
import importlib.metadata
import json
import math
from pathlib import Path
import sys
import time

from isaaclab.app import AppLauncher

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "frigidaire/src"))
from dishsim_frigidaire.paths import IMAGE_DIR
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--manifest", type=Path, required=True)
parser.add_argument("--usd", type=Path, required=True)
parser.add_argument("--out-dir", "--out_dir", dest="out_dir", type=Path, default=IMAGE_DIR / "full_load")
mode = parser.add_mutually_exclusive_group()
mode.add_argument("--physics-only", action="store_true")
mode.add_argument("--render-only", action="store_true")
parser.add_argument("--settle-only", action="store_true")
parser.add_argument("--width", type=int, default=1920)
parser.add_argument("--height", type=int, default=1440)
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
args.usd = args.usd.resolve()
args.manifest = args.manifest.resolve()
args.out_dir = args.out_dir.resolve()
if args.settle_only and args.render_only:
    parser.error("--settle-only is a physics diagnostic, incompatible with --render-only")
if not args.physics_only and not args.enable_cameras:
    parser.error("Image capture requires --enable_cameras; use --physics-only for measurements")
launcher = AppLauncher(args)
simulation_app = launcher.app

import numpy as np  # noqa: E402
import torch  # noqa: E402
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, PhysxSchema  # noqa: E402

from dishsim.media import release_sim_for_close  # noqa: E402

HZ = 120
PRIM = "/World/Frigidaire"
JOINTS = ("door_hinge", "lower_slide", "upper_slide")
EXTENDED = {"door_hinge": math.pi / 2, "lower_slide": -.49, "upper_slide": -.44}
TOLERANCE = {"door_hinge": math.radians(.5), "lower_slide": .005, "upper_slide": .005}
COMMAND_SPEED = {"door_hinge": .35, "lower_slide": .10, "upper_slide": .10}
COMPONENTS = ("Cabinet", "Door", "LowerRack", "UpperRack", "SilverwareBasket")
CONTACT_CAPACITY = 262144


def json_write(path, data):
    path.write_text(json.dumps(data, indent=2, default=lambda v: v.item() if hasattr(v, "item") else str(v)) + "\n")


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def fingerprints():
    """Exclude generated loaded scenes from their own input hash set."""
    asset = {str(path.relative_to(args.usd.parent)): digest(path)
             for path in sorted(args.usd.parent.rglob("*"))
             if path.is_file() and path.suffix in (".usd", ".usda", ".usdc")
             and path.name != "full_load.usda"}
    sources = {str(path.relative_to(ROOT)): digest(path)
               for path in (Path(__file__).resolve(), ROOT / "frigidaire/src/dishsim_frigidaire/asset.py",
                            ROOT / "frigidaire/src/dishsim_frigidaire/load_validation.py")}
    return {"asset_hashes": asset, "source_hashes": sources, "manifest_sha256": digest(args.manifest)}


def counts(objects):
    return {"total": len(objects), "by_type": dict(sorted(Counter(x["kind"] for x in objects).items())),
            "by_rack": {rack: dict(sorted(Counter(x["kind"] for x in objects if x["rack"] == rack).items()))
                        for rack in ("LowerRack", "UpperRack", "SilverwareBasket")}}


def quat_matrix(q):
    w, x, y, z = np.asarray(q, dtype=float) / np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y)],
                     [2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x)],
                     [2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)]])


class Report:
    def __init__(self, inputs, manifest):
        self.started = time.monotonic()
        self.data = {**inputs, "asset": str(args.usd), "manifest": str(args.manifest),
                     "runtime": {"isaac_sim": Path("/isaac-sim/VERSION").read_text().strip(),
                                 "isaac_lab": importlib.metadata.version("isaaclab"),
                                 "device": args.device, "physics_hz": HZ},
                     "gates": [], "states": {}, "objects": {}, "pose_traces": [],
                     "candidate_counts": counts(manifest["objects"]),
                     "validated_counts": None,
                     "validation_scope": "settlement only" if args.settle_only else "physical initialization followed by a complete measured loaded cycle",
                     "initialization": {"result": "NOT_RUN", "states": {}, "pose_traces": [],
                                        "method": "12 s initial settle, gentle open/extend, 2 s hold, retract/close, 4 s hold; independent free bodies throughout"},
                     "thresholds": {"last_second_span_m": .005, "final_speed_m_s": .03,
                                    "final_second_quaternion_span_deg": 3.,
                                    "rack_relative_axis_drift_m": .010, "peak_penetration_m": .002,
                                    "median_worst_penetration_m": .001, "joint_endpoints": TOLERANCE},
                     "motion_profile": {"interpolation": "cubic smoothstep, zero endpoint commanded velocity",
                                        "maximum_commanded_speed": COMMAND_SPEED, "minimum_ramp_seconds": 3.5,
                                        "units": {"door_hinge": "rad/s", "lower_slide": "m/s", "upper_slide": "m/s"},
                                        "moves": []},
                     "capacity_claim": "Mixed load saturated within the manifest's finite placement patterns; not a global maximum."}

    def gate(self, name, passed, **measured):
        self.data["gates"].append({"name": name, "passed": bool(passed), **measured})
        print(f"[{'OK' if passed else 'FAIL'}] {name}: {json.dumps(measured)}", flush=True)

    def save(self, filename="physics.json", completed=False):
        self.data["wall_seconds"] = time.monotonic() - self.started
        passed = bool(self.data["gates"]) and all(g["passed"] for g in self.data["gates"])
        self.data["completed"] = completed
        self.data["result"] = ("PASS" if passed else "FAIL") if completed else "INCOMPLETE"
        if completed and passed and not args.settle_only:
            self.data["validated_counts"] = self.data["candidate_counts"]
        json_write(args.out_dir / filename, self.data)
        return completed and passed


def main():
    import carb.settings
    import isaaclab.sim as sim_utils
    import omni.usd
    from isaaclab.assets import RigidObject, RigidObjectCfg
    from isaaclab.sim import SimulationContext
    from isaacsim.core.api.sensors import RigidContactView
    from dishsim_frigidaire.asset import BODY_POSITIONS, spawn, apply_mode
    from dishsim_frigidaire.load_validation import validate_manifest, externally_supported_indices, pose_motion_metrics
    from dishsim_frigidaire.loading import visual_points
    from dishsim_frigidaire.tableware import CATALOG
    from dishsim.media import CameraRig

    args.out_dir.mkdir(parents=True, exist_ok=True)
    manifest = json.loads(args.manifest.read_text())
    validated_input = validate_manifest(manifest, args.usd, CATALOG, BODY_POSITIONS)
    entries = manifest["objects"]
    inputs = fingerprints()
    accepted = None
    if args.render_only:
        accepted = json.loads((args.out_dir / "physics.json").read_text())
        if accepted.get("result") != "PASS" or not accepted.get("validated_counts"):
            raise ValueError("Render mode requires completed full-cycle PASS physics with validated counts")
        if any(accepted.get(key) != value for key, value in inputs.items()):
            raise ValueError("Physics is stale: asset, manifest, or executable source changed")

    started = time.monotonic()
    sim = SimulationContext(sim_utils.SimulationCfg(dt=1/HZ, device=args.device, use_fabric=True,
                            physx=sim_utils.PhysxCfg(enable_ccd=True)))
    ground = sim_utils.CuboidCfg(size=(200., 200., .05), collision_props=sim_utils.CollisionPropertiesCfg(),
                               visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(.20, .225, .255), roughness=.8))
    ground.func("/World/Ground", ground, translation=(0., 0., -.025))
    fill = sim_utils.DomeLightCfg(intensity=1200, color=(.92, .95, 1.))
    fill.func("/World/Fill", fill)
    key = sim_utils.DistantLightCfg(intensity=2400, angle=15, color=(1., .97, .93))
    key.func("/World/Key", key, orientation=(.9238795, .3826834, 0, 0))
    rim = sim_utils.DistantLightCfg(intensity=1100, angle=20, color=(.86, .92, 1.))
    rim.func("/World/Rim", rim, orientation=(.7071068, -.5, .5, 0))
    dishwasher, basket = spawn(PRIM, mode="scripted", usd_path=str(args.usd))
    stage = omni.usd.get_context().get_stage()
    objects = {}
    motion_points = {}
    for kind in sorted({entry["kind"] for entry in entries} | {"basket"}):
        filename = (args.usd.parent / "silverware_basket.usdc" if kind == "basket" else
                    args.usd.parent / "tableware" / f"{kind}.usdc")
        points = visual_points(filename)
        extrema = [index for axis in range(3) for index in (int(np.argmin(points[:, axis])), int(np.argmax(points[:, axis])))]
        motion_points[kind] = points[extrema]
    for entry in entries:
        xyz = (np.asarray(BODY_POSITIONS[entry["rack"]]) + np.asarray(entry["position"])
               + (0, 0, entry.get("release_hover_m", .003)))
        x, y, z, w = entry["quaternion_xyzw"]
        tableware = args.usd.parent / "tableware" / f"{entry['kind']}.usdc"
        if not tableware.is_file():
            raise FileNotFoundError(tableware)
        objects[entry["id"]] = RigidObject(RigidObjectCfg(
            prim_path=f"/World/Tableware/{entry['id']}",
            spawn=sim_utils.UsdFileCfg(usd_path=str(tableware)),
            init_state=RigidObjectCfg.InitialStateCfg(pos=tuple(xyz), rot=(w, x, y, z))))

    inventory_paths = {}
    rig = None
    if not args.physics_only:
        for i, kind in enumerate(sorted({entry["kind"] for entry in entries})):
            path = f"/World/Inventory/{kind}"
            prim = stage.DefinePrim(path, "Xform")
            prim.GetReferences().AddReference(str(args.usd.parent / "tableware" / f"{kind}.usdc"))
            xf = UsdGeom.Xformable(prim)
            xf.ClearXformOpOrder()
            xf.AddTranslateOp().Set(Gf.Vec3d(10 + i*.8, 0, .8))
            for child in Usd.PrimRange(prim):
                if child.HasAPI(UsdPhysics.RigidBodyAPI):
                    UsdPhysics.RigidBodyAPI(child).CreateRigidBodyEnabledAttr(False)
                if child.HasAPI(UsdPhysics.CollisionAPI):
                    UsdPhysics.CollisionAPI(child).CreateCollisionEnabledAttr(False)
            UsdGeom.Imageable(prim).MakeInvisible()
            inventory_paths[kind] = path
        rig = CameraRig({"inspection": ((1.6, -2.1, 1.45), (0, -.3, .44),
                                         {"focal_length": 48., "horizontal_aperture": 36.})},
                        hw=(args.height, args.width))

    # Isaac Lab disables report processing by default; detailed PhysX contacts need it.
    carb.settings.get_settings().set_bool("/physics/disableContactProcessing", False)
    contact_paths = [f"{PRIM}/{name}" for name in COMPONENTS]
    contact_paths += [obj.cfg.prim_path for obj in objects.values()]
    contacts = RigidContactView(prim_paths_expr=contact_paths,
                               filter_paths_expr=[contact_paths.copy() for _ in contact_paths],
                               name="frigidaire_full_load_contacts", max_contact_count=CONTACT_CAPACITY,
                               disable_stablization=False)
    sim.reset()
    apply_mode(dishwasher, "scripted")
    contacts.initialize()
    if rig:
        rig.apply_poses(sim.device)
    dt = sim.get_physics_dt()
    indices = {name: dishwasher.joint_names.index(name) for name in JOINTS}
    target = dishwasher.data.default_joint_pos.clone()
    body_indices = {name: dishwasher.body_names.index(name) for name in ("LowerRack", "UpperRack")}
    conditioning_trace = None
    conditioning_failures = {}
    motion_phase = "measured_validation"
    rack_envelopes = {}
    for rack, filename in (("LowerRack", "lower_rack.usdc"), ("UpperRack", "upper_rack.usdc"),
                            ("SilverwareBasket", "silverware_basket.usdc")):
        points = visual_points(args.usd.parent / filename)
        rack_envelopes[rack] = np.array([points[:, :2].min(axis=0), points[:, :2].max(axis=0)])

    def tick(n=1):
        for _ in range(n):
            if time.monotonic() - started > 25*60:
                raise TimeoutError("This bounded evidence job exceeded 25 minutes")
            dishwasher.set_joint_position_target(target)
            dishwasher.write_data_to_sim()
            sim.step(render=False)
            dishwasher.update(dt)
            basket.update(dt)
            for obj in objects.values():
                obj.update(dt)
            if conditioning_trace is not None:
                frame_poses = {rack: body_pose(rack) for rack in rack_envelopes}
                positions = np.array([objects[entry["id"]].data.root_pos_w[0].cpu().numpy() for entry in entries])
                quaternions = np.array([objects[entry["id"]].data.root_quat_w[0].cpu().numpy() for entry in entries])
                conditioning_trace["positions_m"].append(positions.tolist())
                conditioning_trace["quaternions_wxyz"].append(quaternions.tolist())
                conditioning_trace["joint_positions"].append(joint_positions())
                for rack, (position, quaternion) in frame_poses.items():
                    conditioning_trace["rack_positions_m"][rack].append(position.tolist())
                    conditioning_trace["rack_quaternions_wxyz"][rack].append(quaternion.tolist())
                for entry, position in zip(entries, positions):
                    rack = entry["rack"]
                    origin, quaternion = frame_poses[rack]
                    relative = quat_matrix(quaternion).T @ (position-origin)
                    lo, hi = rack_envelopes[rack]
                    if relative[2] <= -.035 or (relative[:2] < lo).any() or (relative[:2] > hi).any():
                        conditioning_failures.setdefault(entry["id"], {
                            "segment": conditioning_trace["segment"],
                            "sample": len(conditioning_trace["positions_m"])-1,
                            "rack": rack, "rack_relative_position_m": relative.tolist(),
                            "reason": "left intended supporting rack envelope or fell below its floor"})
                lower_origin, lower_quaternion = frame_poses["LowerRack"]
                relative = quat_matrix(lower_quaternion).T @ (frame_poses["SilverwareBasket"][0]-lower_origin)
                lo, hi = rack_envelopes["LowerRack"]
                if relative[2] <= -.035 or (relative[:2] < lo).any() or (relative[:2] > hi).any():
                    conditioning_failures.setdefault("basket", {"segment": conditioning_trace["segment"],
                        "rack_relative_position_m": relative.tolist(), "reason": "basket left lower rack envelope"})

    def joint_positions():
        q = dishwasher.data.joint_pos[0].cpu().numpy()
        return {name: float(q[index]) for name, index in indices.items()}

    def body_pose(rack):
        if rack == "SilverwareBasket":
            return basket.data.root_pos_w[0].cpu().numpy().copy(), basket.data.root_quat_w[0].cpu().numpy().copy()
        i = body_indices[rack]
        return dishwasher.data.body_pos_w[0, i].cpu().numpy().copy(), dishwasher.data.body_quat_w[0, i].cpu().numpy().copy()

    def record_object(entry):
        obj = objects[entry["id"]]
        p, q = obj.data.root_pos_w[0].cpu().numpy().copy(), obj.data.root_quat_w[0].cpu().numpy().copy()
        origin, rotation = body_pose(entry["rack"])
        return {"kind": entry["kind"], "rack": entry["rack"], "position_m": p.tolist(),
                "quaternion_wxyz": q.tolist(), "rack_relative_position_m": (quat_matrix(rotation).T @ (p-origin)).tolist(),
                "linear_velocity_m_s": obj.data.root_lin_vel_w[0].cpu().numpy().tolist(),
                "angular_velocity_rad_s": obj.data.root_ang_vel_w[0].cpu().numpy().tolist()}

    def snapshot():
        lower_origin, lower_quaternion = body_pose("LowerRack")
        basket_position = basket.data.root_pos_w[0].cpu().numpy().copy()
        return {"joints": joint_positions(),
                "appliance_bodies": {name: {
                    "position_m": dishwasher.data.body_pos_w[0, i].cpu().numpy().tolist(),
                    "quaternion_wxyz": dishwasher.data.body_quat_w[0, i].cpu().numpy().tolist()}
                    for i, name in enumerate(dishwasher.body_names)},
                "basket": {"position_m": basket_position.tolist(),
                           "quaternion_wxyz": basket.data.root_quat_w[0].cpu().numpy().tolist(),
                           "lower_rack_relative_position_m": (quat_matrix(lower_quaternion).T @ (basket_position-lower_origin)).tolist()},
                "rack_frames": {name: {"position_m": body_pose(name)[0].tolist(),
                                       "quaternion_wxyz": body_pose(name)[1].tolist()}
                                for name in ("LowerRack", "UpperRack", "SilverwareBasket")},
                "objects": {entry["id"]: record_object(entry) for entry in entries}}

    def ramp(goal, seconds=3.5, label=None):
        initial = target.clone()
        duration = max(3.5, seconds, *(1.5*abs(float(initial[0, indices[name]])-value)/COMMAND_SPEED[name]
                                       for name, value in goal.items()))
        steps = math.ceil(duration * HZ)
        measured_peak = dict.fromkeys(goal, 0.)
        for step in range(1, steps+1):
            a = step/steps
            a = a*a*(3-2*a)
            for name, value in goal.items():
                target[0, indices[name]] = initial[0, indices[name]]*(1-a) + value*a
            tick()
            velocities = dishwasher.data.joint_vel[0].cpu().numpy()
            for name in goal:
                measured_peak[name] = max(measured_peak[name], abs(float(velocities[indices[name]])))
        report.data["motion_profile"]["moves"].append({
            "phase": motion_phase, "label": label or "joint move",
            "initial": {name: float(initial[0, indices[name]]) for name in goal}, "target": goal,
            "duration_seconds": steps/HZ,
            "peak_commanded_speed": {name: 1.5*abs(float(initial[0, indices[name]])-value)/(steps/HZ)
                                     for name, value in goal.items()},
            "peak_measured_speed": measured_peak})

    def array(value):
        return value.detach().cpu().numpy() if hasattr(value, "detach") else np.asarray(value)

    def read_contacts():
        data = contacts.get_contact_force_data(dt=dt)
        if data is None:
            raise RuntimeError("PhysX detailed contact data unavailable")
        separations = array(data[3]).reshape(-1)
        pair_counts, starts = array(data[4]).reshape(-1), array(data[5]).reshape(-1)
        overflow, invalid = False, False
        depths, active_pairs, deepest = [], [], None
        for pair_index, (start, count) in enumerate(zip(starts, pair_counts)):
            start, count = int(start), int(count)
            if count < 0 or start < 0:
                invalid = True
            if count <= 0:
                continue
            overflow = overflow or start+count > len(separations)
            valid = separations[start:start+count]
            if not len(valid):
                continue
            invalid = invalid or not bool(np.isfinite(valid).all())
            pair = (pair_index // len(contact_paths), pair_index % len(contact_paths))
            if pair[0] == pair[1]:
                continue
            active_pairs.append(pair)
            depth = max(0., -float(np.min(valid)))
            depths.append(depth)
            if deepest is None or depth > deepest["penetration_m"]:
                ix = start + int(np.argmin(valid))
                deepest = {"body_a": contact_paths[pair[0]], "body_b": contact_paths[pair[1]],
                           "penetration_m": depth, "position_m": array(data[1])[ix].tolist(),
                           "normal": array(data[2])[ix].tolist()}
        count = int(pair_counts.sum())
        overflow = overflow or count >= CONTACT_CAPACITY
        return {"peak_m": max(depths, default=0.), "count": count, "overflow": overflow,
                "invalid": invalid, "deepest": deepest, "pairs": active_pairs}

    report = Report(inputs, manifest)
    report.data["input_validation"] = validated_input
    report.data["scene_continuous_collision_detection"] = True
    report.data["initial_settlement_seconds"] = 12.
    report.data["stability_measurement"] = {
        "method": "Unsmoothed maximum finite-difference speed over six actual mesh extrema transformed by every actor pose in the final second",
        "sample_rate_hz": HZ, "speed_limit_m_s": .03, "root_span_limit_m": .005,
        "quaternion_span_limit_deg": 3., "filtering": "none",
        "raw_solver_velocity": "Retained separately; split-impulse solver velocity need not equal pose displacement per timestep",
        "source": "https://nvidia-omniverse.github.io/PhysX/physx/5.4.0/docs/RigidBodyDynamics.html#solver-iterations",
        "local_mesh_extrema_m": {kind: points.tolist() for kind, points in motion_points.items()}}
    report.data["scripted_drives"] = {name: {
        "stiffness": float(dishwasher.data.joint_stiffness[0, index]),
        "damping": float(dishwasher.data.joint_damping[0, index]),
        "effort_limit": float(dishwasher.data.joint_effort_limits[0, index]),
        "velocity_limit": float(dishwasher.data.joint_vel_limits[0, index]),
        "angular_units": "radians" if name == "door_hinge" else "not angular"}
        for name, index in indices.items()}
    reference = None

    def initialize_load():
        """Let contacts find a natural rest configuration through actual joint motion.

        This initialization is recorded separately from the unchanged measured cycle.
        No dish poses, velocities, fixing joints, or holding forces are commanded.
        """
        nonlocal conditioning_trace, conditioning_failures, motion_phase
        motion_phase = "physical_initialization"
        initialization = report.data["initialization"]
        initialization["result"] = "INCOMPLETE"
        initialization["rack_origin_xy_envelopes_m"] = {rack: bounds.tolist() for rack, bounds in rack_envelopes.items()}
        initialization["minimum_rack_relative_origin_z_m"] = -.035
        initialization["sample_rate_hz"] = HZ
        initialization["scope"] = "Continuous actor-origin floor and XY-envelope checks plus support contacts at holds; this phase supplies no validated counts"
        start_gate = len(report.data["gates"])

        def segment(name, goal=None, hold_seconds=None):
            nonlocal conditioning_trace, conditioning_failures
            conditioning_failures = {}
            conditioning_trace = {"schema_version": 1, "segment": name, "phase": motion_phase,
                "dt_s": dt, "object_ids": [entry["id"] for entry in entries], "quaternion_order": "WXYZ",
                "positions_m": [], "quaternions_wxyz": [], "joint_positions": [],
                "rack_positions_m": {rack: [] for rack in rack_envelopes},
                "rack_quaternions_wxyz": {rack: [] for rack in rack_envelopes}}
            samples = []
            if hold_seconds is not None:
                for step in range(round(hold_seconds*HZ)):
                    tick()
                    if step >= round((hold_seconds-1)*HZ) and step % 6 == 0:
                        samples.append(read_contacts())
            else:
                ramp(goal, label=name)
            trace = conditioning_trace
            conditioning_trace = None
            state = snapshot()
            trace_dir = args.out_dir / "pose_traces"
            trace_dir.mkdir(parents=True, exist_ok=True)
            trace_path = trace_dir / f"initialization_{name}.json"
            # Compact JSON keeps the full 120 Hz initialization trace bounded on disk.
            trace_path.write_text(json.dumps(trace, separators=(",", ":")) + "\n")
            trace_reference = {"segment": name, "file": str(trace_path.relative_to(args.out_dir)),
                "sha256": digest(trace_path), "samples_per_body": len(trace["positions_m"]), "dt_s": dt}
            state["pose_trace"] = trace_reference
            state["envelope_failures"] = dict(conditioning_failures)
            initialization["pose_traces"].append(trace_reference)
            report.gate(f"initialization {name}: no object leaves its intended rack", not conditioning_failures,
                        failures=conditioning_failures, samples=len(trace["positions_m"]))
            if samples:
                active = externally_supported_indices(pair for sample in samples for pair in sample["pairs"])
                unsupported = [entry["id"] for entry in entries
                               if contact_paths.index(objects[entry["id"]].cfg.prim_path) not in active]
                if contact_paths.index(f"{PRIM}/SilverwareBasket") not in active:
                    unsupported.append("basket")
                invalid = any(sample["invalid"] or sample["overflow"] for sample in samples)
                state["support_contacts"] = {"unsupported": unsupported, "invalid_or_overflow": invalid,
                    "peak_penetration_m": max(sample["peak_m"] for sample in samples),
                    "median_max_penetration_m": float(np.median([sample["peak_m"] for sample in samples])),
                    "maximum_count": max(sample["count"] for sample in samples)}
                report.gate(f"initialization {name}: support contacts available", not unsupported and not invalid,
                            **state["support_contacts"])
            initialization["states"][name] = state
            report.save()

        segment("initial_settle", hold_seconds=12.)
        segment("open_door", goal={"door_hinge": EXTENDED["door_hinge"]})
        segment("extend_racks", goal={"lower_slide": EXTENDED["lower_slide"], "upper_slide": EXTENDED["upper_slide"]})
        segment("extended_hold", hold_seconds=2.)
        segment("retract_racks", goal={"lower_slide": 0., "upper_slide": 0.})
        segment("close_door", goal={"door_hinge": 0.})
        segment("closed_hold", hold_seconds=4.)
        initialization["result"] = "PASS" if all(g["passed"] for g in report.data["gates"][start_gate:]) else "FAIL"
        initialization["gate_names"] = [g["name"] for g in report.data["gates"][start_gate:]]
        initialization["net_rack_relative_displacements_m"] = {
            identity: (np.asarray(initialization["states"]["closed_hold"]["objects"][identity]["rack_relative_position_m"])
                       - initialization["states"]["initial_settle"]["objects"][identity]["rack_relative_position_m"]).tolist()
            for identity in objects}
        motion_phase = "measured_validation"
        report.save()


    def await_initialization_readiness():
        """Require two complete quiet windows after a fixed minimum settling interval."""
        nonlocal conditioning_trace, conditioning_failures, motion_phase
        motion_phase = "initialization_readiness"
        readiness = {"result": "INCOMPLETE", "minimum_wait_seconds": 12.,
                     "maximum_wait_seconds": 30., "required_consecutive_windows": 2,
                     "window_seconds": 1., "windows": [], "elapsed_seconds": 0.,
                     "method": "No body commands or resets; same mesh-motion, quaternion, contact, and support criteria as measured holds",
                     "retention_reference": "Established by the unchanged measured settled_closed hold after readiness"}
        report.data["initialization"]["readiness"] = readiness
        consecutive = 0
        elapsed = 0.

        def window(name, seconds, evaluate):
            nonlocal conditioning_trace, conditioning_failures, elapsed
            conditioning_failures = {}
            frames = {rack: body_pose(rack) for rack in rack_envelopes}
            conditioning_trace = {"schema_version": 1, "segment": name, "phase": motion_phase,
                "dt_s": dt, "object_ids": [entry["id"] for entry in entries], "quaternion_order": "WXYZ",
                "positions_m": [[objects[e["id"]].data.root_pos_w[0].cpu().numpy().tolist() for e in entries]],
                "quaternions_wxyz": [[objects[e["id"]].data.root_quat_w[0].cpu().numpy().tolist() for e in entries]],
                "joint_positions": [joint_positions()],
                "rack_positions_m": {rack: [pose[0].tolist()] for rack, pose in frames.items()},
                "rack_quaternions_wxyz": {rack: [pose[1].tolist()] for rack, pose in frames.items()}}
            samples = []
            for step in range(round(seconds*HZ)):
                tick()
                if step >= round((seconds-1)*HZ) and step % 6 == 0:
                    samples.append(read_contacts())
            trace = conditioning_trace
            conditioning_trace = None
            state = snapshot()
            trace_dir = args.out_dir / "pose_traces"
            trace_dir.mkdir(parents=True, exist_ok=True)
            trace_path = trace_dir / f"initialization_{name}.json"
            trace_path.write_text(json.dumps(trace, separators=(",", ":")) + "\n")
            trace_reference = {"file": str(trace_path.relative_to(args.out_dir)), "sha256": digest(trace_path),
                               "samples_per_body": len(trace["positions_m"]), "dt_s": dt}
            record = {"name": name, "start_wait_seconds": elapsed, "end_wait_seconds": elapsed+seconds,
                      "eligible_for_readiness": evaluate, "pose_trace": trace_reference,
                      "state": state, "envelope_failures": dict(conditioning_failures)}
            elapsed += seconds
            readiness["elapsed_seconds"] = elapsed
            readiness["windows"].append(record)
            report.data["initialization"]["pose_traces"].append(dict(trace_reference, segment=name))
            if conditioning_failures:
                record["ready"] = False
                record["failures"] = dict(conditioning_failures)
                report.gate("initialization readiness preserves intended rack envelopes", False,
                            failures=conditioning_failures, elapsed_wait_seconds=elapsed)
                readiness["result"] = "FAIL"
                report.data["initialization"]["result"] = "FAIL"
                report.save()
                raise RuntimeError("A body left its intended rack during initialization readiness")
            if not evaluate:
                record["ready"] = None
                report.save()
                return False
            active = externally_supported_indices(pair for sample in samples for pair in sample["pairs"])
            motion_failures = {}
            measurements = {}
            for i, entry in enumerate(entries):
                identity = entry["id"]
                metric = pose_motion_metrics([row[i] for row in trace["positions_m"]],
                                             [row[i] for row in trace["quaternions_wxyz"]],
                                             motion_points[entry["kind"]], dt)
                supported = contact_paths.index(objects[identity].cfg.prim_path) in active
                measurements[identity] = dict(metric, has_support_contact=supported)
                reasons = []
                if metric["root_position_span_m"] >= .005:
                    reasons.append("root span exceeds 5 mm")
                if metric["peak_mesh_point_speed_m_s"] >= .03:
                    reasons.append("mesh-point speed exceeds 0.03 m/s")
                if metric["quaternion_span_deg"] >= 3.:
                    reasons.append("orientation span exceeds 3 degrees")
                if not supported:
                    reasons.append("no appliance/tableware support contact")
                if reasons:
                    motion_failures[identity] = reasons
            basket_motion = pose_motion_metrics(trace["rack_positions_m"]["SilverwareBasket"],
                trace["rack_quaternions_wxyz"]["SilverwareBasket"], motion_points["basket"], dt)
            basket_supported = contact_paths.index(f"{PRIM}/SilverwareBasket") in active
            if (basket_motion["root_position_span_m"] >= .005 or basket_motion["peak_mesh_point_speed_m_s"] >= .03
                    or basket_motion["quaternion_span_deg"] >= 3. or not basket_supported):
                motion_failures["basket"] = ["basket motion or support does not meet the measured-hold criteria"]
            peak = max(sample["peak_m"] for sample in samples)
            persistent = float(np.median([sample["peak_m"] for sample in samples]))
            invalid = any(sample["invalid"] or sample["overflow"] for sample in samples)
            contacts_ok = peak < .002 and persistent < .001 and not invalid and max(sample["count"] for sample in samples) > 0
            record.update(ready=not motion_failures and contacts_ok, failures=motion_failures,
                          object_motion=measurements, basket_motion=basket_motion,
                          contacts={"passed": contacts_ok, "peak_penetration_m": peak,
                                    "median_max_penetration_m": persistent, "invalid_or_overflow": invalid,
                                    "maximum_count": max(sample["count"] for sample in samples)})
            print(f"[INFO] initialization readiness at {elapsed:.1f}s: ready={record['ready']}, "
                  f"motion_failures={json.dumps(motion_failures)}, contacts_ok={contacts_ok}", flush=True)
            report.save()
            return record["ready"]

        window("readiness_minimum_hold", 12., False)
        while elapsed < 30. and consecutive < 2:
            ready = window(f"readiness_window_{len(readiness['windows']):02d}", 1., True)
            consecutive = consecutive+1 if ready else 0
            readiness["consecutive_passing_windows"] = consecutive
        passed = consecutive == 2
        readiness["result"] = "PASS" if passed else "FAIL"
        if not passed:
            report.data["initialization"]["result"] = "FAIL"
        report.gate("initialized load meets two consecutive readiness windows", passed,
                    elapsed_wait_seconds=elapsed, minimum_wait_seconds=12., maximum_wait_seconds=30.,
                    required_consecutive_windows=2, consecutive_passing_windows=consecutive)
        motion_phase = "measured_validation"
        report.save()
        if not passed:
            raise RuntimeError("Load did not meet unchanged readiness thresholds within 30 simulated seconds")


    def hold(name, seconds=2., goal=None):
        nonlocal reference
        positions = {entry["id"]: [] for entry in entries}
        orientations = {entry["id"]: [] for entry in entries}
        basket_positions = []
        basket_orientations = []
        samples = []
        for step in range(round(seconds*HZ)):
            tick()
            if step >= max(0, round((seconds-1)*HZ)-1):
                basket_positions.append(basket.data.root_pos_w[0].cpu().numpy().copy())
                basket_orientations.append(basket.data.root_quat_w[0].cpu().numpy().copy())
                for entry in entries:
                    positions[entry["id"]].append(objects[entry["id"]].data.root_pos_w[0].cpu().numpy().copy())
                    orientations[entry["id"]].append(objects[entry["id"]].data.root_quat_w[0].cpu().numpy().copy())
                if step % 6 == 0:
                    samples.append(read_contacts())
        state = snapshot()
        if reference is None:
            reference = state
        basket_drift = (np.asarray(state["basket"]["lower_rack_relative_position_m"])
                        - reference["basket"]["lower_rack_relative_position_m"])
        basket_motion = pose_motion_metrics(basket_positions, basket_orientations, motion_points["basket"], dt)
        basket_span = basket_motion["root_position_span_m"]
        basket_speed = basket_motion["peak_mesh_point_speed_m_s"]
        report.gate(f"{name}: removable basket retained", np.max(np.abs(basket_drift)) <= .010
                    and basket_span < .005 and basket_speed < .03 and basket_motion["quaternion_span_deg"] < 3.,
                    lower_rack_relative_drift_m=basket_drift.tolist(), final_second_span_m=basket_span,
                    final_speed_m_s=basket_speed, measured_motion=basket_motion,
                    raw_solver_final_speed_m_s=float(np.linalg.norm(basket.data.root_lin_vel_w[0].cpu().numpy())))
        if goal is not None:
            error = {key: state["joints"][key]-value for key, value in goal.items()}
            report.gate(f"{name}: joint endpoints", all(abs(value) <= TOLERANCE[key] for key, value in error.items()),
                        positions=state["joints"], errors=error)
        penetration = max(x["peak_m"] for x in samples)
        persistent = float(np.median([x["peak_m"] for x in samples]))
        overflow, invalid = any(x["overflow"] for x in samples), any(x["invalid"] for x in samples)
        report.gate(f"{name}: settled contacts", penetration < .002 and persistent < .001
                    and not overflow and not invalid and max(x["count"] for x in samples) > 0,
                    peak_penetration_m=penetration, median_max_penetration_m=persistent,
                    maximum_count=max(x["count"] for x in samples), capacity=CONTACT_CAPACITY,
                    buffer_overflow=overflow, invalid_data=invalid,
                    deepest=max(samples, key=lambda x: x["peak_m"])["deepest"])
        active = externally_supported_indices(pair for sample in samples for pair in sample["pairs"])
        failures = {}
        for entry in entries:
            identity = entry["id"]
            obj = state["objects"][identity]
            motion = pose_motion_metrics(positions[identity], orientations[identity], motion_points[entry["kind"]], dt)
            span = motion["root_position_span_m"]
            speed = motion["peak_mesh_point_speed_m_s"]
            drift = np.asarray(obj["rack_relative_position_m"]) - reference["objects"][identity]["rack_relative_position_m"]
            support = contact_paths.index(objects[identity].cfg.prim_path) in active
            # Ground is deliberately absent from the contact view: falling to the floor
            # cannot count as support. Require the object's origin to remain above its
            # supporting component's low floor, allowing centered tableware meshes.
            above_floor = obj["rack_relative_position_m"][2] > -.035
            reasons = []
            if span >= .005:
                reasons.append("final-second motion exceeds 5 mm")
            if speed >= .03:
                reasons.append("measured mesh-point speed exceeds 0.03 m/s")
            if motion["quaternion_span_deg"] >= 3.:
                reasons.append("final-second orientation span exceeds 3 degrees")
            if np.max(np.abs(drift)) > .010:
                reasons.append("rack-relative travel displacement exceeds 10 mm")
            if not support:
                reasons.append("no appliance/tableware support contact during the hold")
            if not above_floor:
                reasons.append("fell below the supporting rack")
            obj.update(final_second_span_m=span, final_speed_m_s=speed, relative_drift_m=drift.tolist(),
                       measured_motion=motion, raw_solver_final_speed_m_s=float(np.linalg.norm(obj["linear_velocity_m_s"])),
                       has_support_contact=support, above_support_floor=above_floor, failure_reasons=reasons)
            if reasons:
                failures[identity] = reasons
            summary = report.data["objects"].setdefault(identity, {"kind": entry["kind"], "rack": entry["rack"],
                                                                   "failures": {}, "states": {}})
            summary["states"][name] = obj
            if reasons:
                summary["failures"][name] = reasons
        report.gate(f"{name}: all {len(entries)} objects retained and stable", not failures, failures=failures,
                    largest_span_m=max(x["final_second_span_m"] for x in state["objects"].values()),
                    largest_relative_drift_m=max(max(abs(v) for v in x["relative_drift_m"]) for x in state["objects"].values()))
        state["contacts"] = {"peak_penetration_m": penetration, "median_max_penetration_m": persistent,
                             "deepest": max(samples, key=lambda x: x["peak_m"])["deepest"]}
        trace = {"schema_version": 1, "hold": name, "dt_s": dt, "quaternion_order": "WXYZ",
                 "hold_time_start_s": seconds-(len(basket_positions)-1)*dt,
                 "basket": {"positions_m": np.asarray(basket_positions).tolist(),
                            "quaternions_wxyz": np.asarray(basket_orientations).tolist()},
                 "objects": {entry["id"]: {"kind": entry["kind"],
                              "positions_m": np.asarray(positions[entry["id"]]).tolist(),
                              "quaternions_wxyz": np.asarray(orientations[entry["id"]]).tolist()}
                             for entry in entries}}
        trace_dir = args.out_dir / "pose_traces"
        trace_dir.mkdir(parents=True, exist_ok=True)
        trace_path = trace_dir / f"{name}.json"
        json_write(trace_path, trace)
        trace_reference = {"hold": name, "file": str(trace_path.relative_to(args.out_dir)),
                           "sha256": digest(trace_path), "samples_per_body": len(basket_positions), "dt_s": dt}
        state["pose_trace"] = trace_reference
        report.data["pose_traces"].append(trace_reference)
        report.data["states"][name] = state
        report.save()

    physics_ok = True
    if not args.render_only:
        scenes = [prim for prim in stage.Traverse() if prim.IsA(UsdPhysics.Scene)]
        ccd = [bool(PhysxSchema.PhysxSceneAPI(prim).GetEnableCCDAttr().Get()) for prim in scenes]
        report.gate("scene continuous collision detection enabled", len(ccd) == 1 and all(ccd),
                    scenes=[str(prim.GetPath()) for prim in scenes], enabled=ccd)
        report.gate("three named appliance joints", set(dishwasher.joint_names) == set(JOINTS), names=dishwasher.joint_names)
        if not args.settle_only:
            initialize_load()
            await_initialization_readiness()
        # Dense plate banks need time to finish small contact adjustments.
        # Allow twelve seconds, then measure the full final second with
        # the same unsmoothed speed, displacement and rotation thresholds.
        hold("settled_closed", seconds=12., goal=dict.fromkeys(JOINTS, 0.))
        if not args.settle_only:
            ramp({"door_hinge": EXTENDED["door_hinge"]})
            hold("open_retracted", goal={"door_hinge": EXTENDED["door_hinge"], "lower_slide": 0., "upper_slide": 0.})
            ramp({"lower_slide": EXTENDED["lower_slide"], "upper_slide": EXTENDED["upper_slide"]})
            hold("first_extended", goal=EXTENDED)
            ramp({"lower_slide": 0., "upper_slide": 0.})
            hold("loaded_retracted", goal={"lower_slide": 0., "upper_slide": 0.})
            ramp({"door_hinge": 0.})
            hold("loaded_closed", goal=dict.fromkeys(JOINTS, 0.))
            ramp({"door_hinge": EXTENDED["door_hinge"]})
            ramp({"lower_slide": EXTENDED["lower_slide"], "upper_slide": EXTENDED["upper_slide"]})
            hold("final_extended", goal=EXTENDED)
        physics_ok = report.save(completed=True)
        if physics_ok and not args.settle_only:
            export_scene(report.data, manifest)
        accepted = report.data

    render_ok = True
    if rig:
        if accepted.get("result") != "PASS" or not accepted.get("validated_counts"):
            raise RuntimeError("Images of a claimed full load require successful full-cycle physics")
        rendered = {**inputs, "images": [], "counts": accepted["validated_counts"],
                    "physics_sha256": digest(args.out_dir / "physics.json"),
                    "note": "Exact saved physical states from the accepted full-load cycle; static isolation hides other components only."}
        camera = rig.cams["inspection"]

        def visible(paths, value):
            for path in paths:
                imageable = UsdGeom.Imageable(stage.GetPrimAtPath(path))
                imageable.MakeVisible() if value else imageable.MakeInvisible()

        def set_root(obj, record):
            pose = torch.tensor([[*record["position_m"], *record["quaternion_wxyz"]]], device=sim.device, dtype=torch.float32)
            obj.write_root_pose_to_sim(pose)
            obj.write_root_velocity_to_sim(torch.zeros((1, 6), device=sim.device))

        def replay(name):
            state = accepted["states"][name]
            for joint, value in state["joints"].items():
                target[0, indices[joint]] = value
            dishwasher.write_joint_state_to_sim(target, torch.zeros_like(target))
            dishwasher.set_joint_position_target(target)
            set_root(basket, state["basket"])
            for identity, obj in objects.items():
                set_root(obj, state["objects"][identity])
            sim.forward()

        def photograph(stem, label, eye, center, state=None):
            from PIL import Image
            camera.set_world_poses_from_view(eyes=torch.tensor([eye], device=sim.device, dtype=torch.float32),
                                            targets=torch.tensor([center], device=sim.device, dtype=torch.float32))
            for _ in range(10):
                sim.render()
                rig.update(dt)
            pixels = rig.grab_one("inspection")
            output = args.out_dir / f"{stem}.png"
            Image.fromarray(pixels).save(output)
            valid = pixels.shape[:2] == (args.height, args.width) and pixels.std() > 5 and pixels.max() > 60
            rendered["images"].append({"file": output.name, "label": label, "state": state,
                                       "passed": bool(valid), "resolution": [pixels.shape[1], pixels.shape[0]],
                                       "rgb_std": float(pixels.std()), "sha256": digest(output)})
            print(f"[{'OK' if valid else 'FAIL'}] image {stem}", flush=True)

        all_paths = [f"{PRIM}/{name}" for name in COMPONENTS] + [obj.cfg.prim_path for obj in objects.values()]
        for state, stem, label in (("loaded_closed", "loaded_closed", "Validated load — door closed"),
                                   ("open_retracted", "loaded_open", "Validated load — open, racks retracted"),
                                   ("final_extended", "loaded_extended", "Validated mixed load — both racks extended")):
            replay(state)
            visible(all_paths, True)
            photograph(stem, label, (1.6, -2.1, 1.65), (0, -.34, .46), state)
        photograph("loaded_front", "Validated mixed load — front", (0, -2.6, 1.05), (0, -.3, .46), "final_extended")
        photograph("loaded_left", "Validated mixed load — left", (-1.7, -1.8, 1.55), (0, -.34, .46), "final_extended")
        photograph("loaded_top", "Validated mixed load — overhead", (0, -.40, 2.85), (0, -.39, .45), "final_extended")
        for rack, stem in (("LowerRack", "lower"), ("UpperRack", "upper")):
            selected = [f"{PRIM}/{rack}"]
            wanted = {rack}
            if rack == "LowerRack":
                selected.append(f"{PRIM}/SilverwareBasket")
                wanted.add("SilverwareBasket")
            selected.extend(objects[entry["id"]].cfg.prim_path for entry in entries if entry["rack"] in wanted)
            visible(all_paths, False)
            visible(selected, True)
            origin = np.asarray(accepted["states"]["final_extended"]["rack_frames"][rack]["position_m"])
            center = origin + (0, 0, .10)
            photograph(f"{stem}_loaded_top", f"{stem.title()} rack — accepted load, overhead",
                       center+(0, -.012, 1.9), center, "final_extended")
            photograph(f"{stem}_loaded_oblique", f"{stem.title()} rack — accepted load, oblique",
                       center+(-1.05, -1.20, 1.05), center, "final_extended")
        visible(all_paths, False)
        for kind, path in inventory_paths.items():
            visible([path], True)
            box = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render]).ComputeWorldBound(
                stage.GetPrimAtPath(path)).ComputeAlignedRange()
            center, size = np.array(box.GetMidpoint()), np.array(box.GetSize())
            distance = max(float(size.max())*2.8, .42)
            direction = np.array([-.65, -1., .9])
            photograph(f"inventory_{kind}", f"{kind.replace('_', ' ').title()} × {accepted['validated_counts']['by_type'][kind]}",
                       center+direction/np.linalg.norm(direction)*distance, center)
            visible([path], False)
        rendered["contact_sheets"] = write_gallery(rendered)
        rendered["result"] = "PASS" if all(x["passed"] for x in rendered["images"]) else "FAIL"
        json_write(args.out_dir / "renders.json", rendered)
        render_ok = rendered["result"] == "PASS"
    merge_reports(inputs)
    print(f"[RESULT] {'PASS' if physics_ok and render_ok else 'FAIL'} — requested full-load evidence run", flush=True)


def export_scene(report, manifest):
    """Export a runnable, lit CPU scene with measured poses and finite closed-hold drives."""
    from pxr import PhysxSchema, UsdLux, UsdShade
    path = args.usd.parent / "full_load.usda"
    if path.exists():
        path.unlink()
    stage = Usd.Stage.CreateNew(str(path))
    root = UsdGeom.Xform.Define(stage, "/LoadedFrigidaire")
    stage.SetDefaultPrim(root.GetPrim())
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.)
    UsdPhysics.SetStageKilogramsPerUnit(stage, 1.)
    stage.SetTimeCodesPerSecond(HZ)
    scene = UsdPhysics.Scene.Define(stage, "/LoadedFrigidaire/Physics")
    scene.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
    scene.CreateGravityMagnitudeAttr(9.81)
    physics = PhysxSchema.PhysxSceneAPI.Apply(scene.GetPrim())
    physics.CreateTimeStepsPerSecondAttr(HZ)
    physics.CreateEnableGPUDynamicsAttr(False)
    physics.CreateEnableCCDAttr(True)
    physics.CreateBroadphaseTypeAttr("MBP")
    physics.CreateSolverTypeAttr("TGS")
    floor = UsdGeom.Cube.Define(stage, "/LoadedFrigidaire/Ground")
    floor.CreateSizeAttr(1.)
    xf = UsdGeom.Xformable(floor)
    xf.AddTranslateOp().Set(Gf.Vec3d(0, 0, -.025))
    xf.AddScaleOp().Set(Gf.Vec3f(6, 6, .05))
    UsdPhysics.CollisionAPI.Apply(floor.GetPrim())
    floor.CreateDisplayColorAttr([Gf.Vec3f(.20, .225, .255)])
    material = UsdShade.Material.Define(stage, "/LoadedFrigidaire/Looks/Ground")
    shader = UsdShade.Shader.Define(stage, "/LoadedFrigidaire/Looks/Ground/Shader")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(.20, .225, .255))
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(.8)
    material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    UsdShade.MaterialBindingAPI.Apply(floor.GetPrim()).Bind(material)
    fill = UsdLux.DomeLight.Define(stage, "/LoadedFrigidaire/Fill")
    fill.CreateIntensityAttr(1200.)
    fill.CreateColorAttr(Gf.Vec3f(.92, .95, 1.))
    key = UsdLux.DistantLight.Define(stage, "/LoadedFrigidaire/Key")
    key.CreateIntensityAttr(2400.)
    key.CreateAngleAttr(15.)
    key.CreateColorAttr(Gf.Vec3f(1., .97, .93))
    UsdGeom.Xformable(key).AddRotateXYZOp().Set(Gf.Vec3f(30, -25, -25))
    camera = UsdGeom.Camera.Define(stage, "/LoadedFrigidaire/Camera")
    camera.CreateFocalLengthAttr(48.)
    camera.CreateHorizontalApertureAttr(36.)
    camera.CreateClippingRangeAttr(Gf.Vec2f(.05, 100.))
    camera_matrix = Gf.Matrix4d(1).SetLookAt(Gf.Vec3d(1.6, -2.1, 1.65), Gf.Vec3d(0, -.34, .46),
                                           Gf.Vec3d(0, 0, 1)).GetInverse()
    UsdGeom.Xformable(camera).AddTransformOp().Set(camera_matrix)
    appliance = stage.DefinePrim("/LoadedFrigidaire/Appliance", "Xform")
    appliance.GetReferences().AddReference(args.usd.name)
    state = report["states"]["loaded_closed"]
    for name, record in state["appliance_bodies"].items():
        prim = stage.GetPrimAtPath(f"/LoadedFrigidaire/Appliance/{name}")
        xf = UsdGeom.Xformable(prim)
        xf.ClearXformOpOrder()
        xf.AddTranslateOp().Set(Gf.Vec3d(*record["position_m"]))
        w, x, y, z = record["quaternion_wxyz"]
        xf.AddOrientOp().Set(Gf.Quatf(w, Gf.Vec3f(x, y, z)))
    for name, values in report["scripted_drives"].items():
        joint = stage.GetPrimAtPath(f"/LoadedFrigidaire/Appliance/Joints/{name}")
        angular = name == "door_hinge"
        drive = UsdPhysics.DriveAPI.Apply(joint, "angular" if angular else "linear")
        # apply_mode's live PhysX gains use radians. USD angular drive gains are
        # per degree; this matches Isaac Core Articulation.set_gains' USD branch.
        scale = math.pi/180 if angular else 1.
        drive.CreateTypeAttr("force")
        drive.CreateStiffnessAttr(values["stiffness"]*scale)
        drive.CreateDampingAttr(values["damping"]*scale)
        drive.CreateMaxForceAttr(values["effort_limit"])
        drive.CreateTargetPositionAttr(0.)
        drive.CreateTargetVelocityAttr(0.)
        PhysxSchema.PhysxJointAPI.Apply(joint).CreateMaxJointVelocityAttr(values["velocity_limit"]/scale)
    appliance.GetAttribute("mode").Set("scripted closed hold; edit USD joint drive targets with the documented gentle profile")
    for entry in manifest["objects"]:
        prim = stage.DefinePrim(f"/LoadedFrigidaire/Tableware/{entry['id']}", "Xform")
        prim.GetReferences().AddReference(f"tableware/{entry['kind']}.usdc")
        record = state["objects"][entry["id"]]
        xf = UsdGeom.Xformable(prim)
        xf.ClearXformOpOrder()
        xf.AddTranslateOp().Set(Gf.Vec3d(*record["position_m"]))
        w, x, y, z = record["quaternion_wxyz"]
        xf.AddOrientOp().Set(Gf.Quatf(w, Gf.Vec3f(x, y, z)))
    # Basket remains free; preserve its actual settled pose instead of its nominal site.
    basket = stage.GetPrimAtPath("/LoadedFrigidaire/Appliance/SilverwareBasket")
    xf = UsdGeom.Xformable(basket)
    xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(Gf.Vec3d(*state["basket"]["position_m"]))
    w, x, y, z = state["basket"]["quaternion_wxyz"]
    xf.AddOrientOp().Set(Gf.Quatf(w, Gf.Vec3f(x, y, z)))
    root.GetPrim().SetCustomData({"validated_object_count": len(manifest["objects"]),
                                  "manifest_sha256": report["manifest_sha256"],
                                  "physics_report": "See ../images/full_load/physics.json in the collection",
                                  "mount": "Fixed appliance at world origin; relocating an ancestor alone does not relocate its world anchor",
                                  "joint_target_units": "door_hinge degrees; lower_slide and upper_slide metres"})
    stage.GetRootLayer().Save()
    settled = {**manifest, "validated_counts": report["validated_counts"], "physics_result": "PASS",
               "settled_objects": state["objects"], "source_manifest_sha256": report["manifest_sha256"]}
    json_write(args.usd.parent / "full_load_settled.json", settled)


def write_gallery(rendered):
    from PIL import Image, ImageDraw, ImageFont
    font = ImageFont.load_default(size=20)
    sheets = []
    for stem, items, columns in (
        ("contact_sheet", [x for x in rendered["images"] if x["file"] in
                           ("loaded_extended.png", "loaded_front.png", "lower_loaded_top.png", "upper_loaded_top.png",
                            "lower_loaded_oblique.png", "upper_loaded_oblique.png")], 3),
        ("inventory", [x for x in rendered["images"] if x["file"].startswith("inventory_")], 5)):
        width, height = (640, 480) if columns == 3 else (384, 288)
        rows = math.ceil(len(items)/columns)
        sheet = Image.new("RGB", (columns*width, rows*(height+32)), (24, 33, 43))
        draw = ImageDraw.Draw(sheet)
        for i, item in enumerate(items):
            x, y = (i % columns)*width, (i // columns)*(height+32)
            with Image.open(args.out_dir / item["file"]) as source:
                sheet.paste(source.convert("RGB").resize((width, height), Image.Resampling.LANCZOS), (x, y+32))
            label = item["label"].replace(" — accepted load, ", " — ").replace("Validated mixed load — ", "Load — ")
            draw.text((x+8, y+4), label, font=font, fill=(240, 240, 240))
        output = args.out_dir / f"{stem}.png"
        sheet.save(output)
        sheets.append({"file": output.name, "sha256": digest(output), "sources": [x["file"] for x in items]})
    cards = "\n".join(f'<figure><a href="{html.escape(x["file"])}"><img loading="lazy" src="{html.escape(x["file"])}" '
                        f'alt="{html.escape(x["label"])}"></a><figcaption>{html.escape(x["label"])}</figcaption></figure>'
                        for x in rendered["images"])
    rows = "".join(f"<tr><td>{html.escape(kind.replace('_', ' ').title())}</td><td>{count}</td></tr>"
                   for kind, count in rendered["counts"]["by_type"].items())
    (args.out_dir / "index.html").write_text('<!doctype html><html lang="en"><meta charset="utf-8">'
        '<meta name="viewport" content="width=device-width, initial-scale=1"><title>Frigidaire full load</title>'
        '<style>body{font:16px system-ui;background:#18212b;color:#edf1f4;margin:2rem}main{display:grid;'
        'grid-template-columns:repeat(auto-fit,minmax(320px,1fr));gap:1rem}figure{margin:0;background:#263440}'
        'img{display:block;width:100%}figcaption,td{padding:.6rem}a{color:#acd9ff}</style>'
        f'<h1>Frigidaire — {rendered["counts"]["total"]} validated objects</h1>'
        '<p>A mixed load saturated within documented placement patterns, using fixed-size tableware. '
        'These images replay the settled states measured during the complete loaded rack and door cycle.</p>'
        '<p><a href="physics.json">Physics measurements</a> · <a href="inventory.png">Object inventory</a> · '
        '<a href="contact_sheet.png">Load contact sheet</a></p>'
        f'<table><tr><th>Object</th><th>Count</th></tr>{rows}</table><main>{cards}</main></html>')
    return sheets


def merge_reports(inputs):
    states = {}
    for name in ("physics", "renders"):
        path = args.out_dir / f"{name}.json"
        if not path.exists():
            states[name] = "NOT_RUN"
            continue
        data = json.loads(path.read_text())
        states[name] = data.get("result", "INCOMPLETE") if all(data.get(k) == v for k, v in inputs.items()) else "STALE"
        if name == "physics" and states[name] == "PASS" and not data.get("validated_counts"):
            states[name] = "DIAGNOSTIC_ONLY"
        if name == "renders" and states[name] == "PASS" and data.get("physics_sha256") != digest(args.out_dir / "physics.json"):
            states[name] = "STALE"
    result = "PASS" if all(s == "PASS" for s in states.values()) else "FAIL" if "FAIL" in states.values() else "INCOMPLETE"
    json_write(args.out_dir / "evidence.json", {**inputs, "result": result, "reports": states, "image_index": "index.html"})


if __name__ == "__main__":
    try:
        main()
    except BaseException as exc:
        import traceback
        traceback.print_exc()
        args.out_dir.mkdir(parents=True, exist_ok=True)
        filename = "renders.json" if args.render_only else "physics.json"
        try:
            inputs = fingerprints()
            previous = args.out_dir / filename
            if previous.exists():
                diagnostic = json.loads(previous.read_text())
                if not all(diagnostic.get(key) == value for key, value in inputs.items()):
                    diagnostic = dict(inputs)
                diagnostic.update(result="FAIL", exception=repr(exc))
            else:
                diagnostic = {**inputs, "result": "FAIL", "exception": repr(exc)}
            if not args.render_only:
                diagnostic["validated_counts"] = None
            json_write(previous, diagnostic)
            merge_reports(inputs)
        except Exception as secondary:
            print(f"[ERROR] Could not save failure report: {secondary!r}", flush=True)
        print(f"[RESULT] FAIL — {exc!r}", flush=True)
    finally:
        release_sim_for_close()
        simulation_app.close()
