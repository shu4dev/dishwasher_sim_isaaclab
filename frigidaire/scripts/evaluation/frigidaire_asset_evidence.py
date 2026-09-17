# Copyright (c) 2026, dishsim project.
# SPDX-License-Identifier: BSD-3-Clause
"""Measure the FDPC4221AS asset and photograph its actual USD geometry in Isaac Sim.

Run physics and images separately to keep each Kit job short::

    scripts/run_kit.sh frigidaire/scripts/evaluation/frigidaire_asset_evidence.py --headless --assembly-only --physics-only
    scripts/run_kit.sh frigidaire/scripts/evaluation/frigidaire_asset_evidence.py --headless --assembly-only --enable_cameras --render-only

``evidence.json`` merges only reports whose scope, asset hashes, and source hashes match.
Assembly-only evidence measures the appliance and removable basket without loading dishes.
A render-only run cannot certify unexecuted physics. The gallery is ``index.html``.
"""
import argparse
import hashlib
import importlib.metadata
import json
import math
from pathlib import Path
import sys
import time

from isaaclab.app import AppLauncher

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(ROOT / "frigidaire/src"))
from dishsim_frigidaire.paths import ASSET_DIR, IMAGE_DIR, SOURCE_ROOT

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--usd", type=Path, default=ASSET_DIR / "fdpc4221as.usdc")
parser.add_argument("--out-dir", "--out_dir", dest="out_dir", type=Path, default=IMAGE_DIR / "assembly")
parser.add_argument("--assembly-only", action="store_true",
                    help="Validate and render the appliance and removable basket without fixture loading.")
mode = parser.add_mutually_exclusive_group()
mode.add_argument("--physics-only", action="store_true")
mode.add_argument("--render-only", action="store_true")
parser.add_argument("--width", type=int, default=1920)
parser.add_argument("--height", type=int, default=1440)
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
if not args.physics_only and not args.enable_cameras:
    parser.error("Image capture requires --enable_cameras; use --physics-only for measurements alone.")
launcher = AppLauncher(args)
simulation_app = launcher.app

import numpy as np  # noqa: E402
import torch  # noqa: E402
from pxr import Gf, Usd, UsdGeom, UsdPhysics  # noqa: E402

from dishsim.media import release_sim_for_close  # noqa: E402

JOINTS = ("door_hinge", "lower_slide", "upper_slide")
EXTENDED = {"door_hinge": math.pi / 2, "lower_slide": -.49, "upper_slide": -.44}
TOLERANCE = {"door_hinge": math.radians(.5), "lower_slide": .005, "upper_slide": .005}
COMMAND_SPEED = {"door_hinge": .35, "lower_slide": .10, "upper_slide": .10}
COMPONENTS = {"cabinet": "Cabinet", "door": "Door", "lower_rack": "LowerRack",
              "upper_rack": "UpperRack", "silverware_basket": "SilverwareBasket"}
FIXTURE_SITES = {"plate": ("LowerRack", "plate"), "bowl": ("LowerRack", "bowl"),
                 "cup": ("UpperRack", "cup"), "utensil": ("SilverwareBasket", "utensil")}
PRIM = "/World/Frigidaire"
HZ = 120


def json_write(path, data):
    path.write_text(json.dumps(data, indent=2, default=lambda v: v.item() if hasattr(v, "item") else str(v)) + "\n")


def hashes(directory):
    return {str(p.relative_to(directory)): hashlib.sha256(p.read_bytes()).hexdigest()
            for p in sorted(directory.rglob("*")) if p.is_file() and p.suffix in (".usd", ".usda", ".usdc")
            and p.name != "full_load.usda"}


def source_hashes():
    package = SOURCE_ROOT / "src/dishsim_frigidaire"
    sources = [Path(__file__).resolve(), *(p for p in sorted(package.rglob("*"))
                                        if p.is_file() and p.suffix in (".py", ".json"))]
    return {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest()
            for p in sources}


def evidence_scope():
    return "assembly" if args.assembly_only else "fixtures"


def version_info():
    version_file = Path("/isaac-sim/VERSION")
    return {"isaac_sim": version_file.read_text().strip() if version_file.exists() else "unavailable",
            "isaac_lab": importlib.metadata.version("isaaclab"), "device": args.device,
            "renderer": "Isaac Sim RTX camera sensor", "physics_hz": HZ}


def quaternion_angle(first, second):
    first, second = np.asarray(first, dtype=float), np.asarray(second, dtype=float)
    similarity = abs(float(np.dot(first, second) / (np.linalg.norm(first) * np.linalg.norm(second))))
    return 2 * math.acos(min(1., similarity))


def fixture_axis_angle(first, second):
    # Circular fixture spin about its own normal is irrelevant to support; compare
    # the plate normal / bowl and cup vertical axes rather than full quaternions.
    def axis(q):
        w, x, y, z = np.asarray(q, dtype=float) / np.linalg.norm(q)
        return np.array([2 * (x*z + w*y), 2 * (y*z - w*x), 1 - 2 * (x*x + y*y)])
    return math.acos(float(np.clip(np.dot(axis(first), axis(second)), -1, 1)))


def collect_sites(stage):
    """Read authored fixture centers and orientations; never synthesize a missing site."""
    cache = UsdGeom.XformCache()
    sites = {}
    for kind, (body, needle) in FIXTURE_SITES.items():
        parent = stage.GetPrimAtPath(f"{PRIM}/{body}/Manipulation")
        candidates = [p for p in Usd.PrimRange(parent) if needle in p.GetName().lower()] if parent else []
        if not candidates:
            raise ValueError(f"Required {kind} fixture site missing under {body}/Manipulation")
        prim = next((p for p in candidates if p.GetName().lower() == needle), candidates[0])
        matrix = cache.GetLocalToWorldTransform(prim)
        quaternion = prim.GetAttribute("quatWXYZ").Get()
        if quaternion is None:
            q = matrix.ExtractRotationQuat()
            quaternion = [q.GetReal(), *q.GetImaginary()]
        sites[kind] = {"path": str(prim.GetPath()), "body": body,
                       "position": list(matrix.ExtractTranslation()), "quat_wxyz": list(quaternion)}
    return sites


class Evidence:
    def __init__(self, asset_hashes):
        self.started = time.monotonic()
        self.data = {"asset": str(args.usd), "asset_hashes": asset_hashes,
                     "scope": evidence_scope(),
                     "runtime": version_info(), "gates": [], "measurements": {}, "source_hashes": source_hashes(),
                     "motion_profile": {"interpolation": "cubic smoothstep, zero endpoint commanded velocity",
                                        "maximum_commanded_speed": COMMAND_SPEED, "minimum_ramp_seconds": 3.5,
                                        "scope": "scripted ramps; passive impulses and empty relocation test are separate probes",
                                        "units": {"door_hinge": "rad/s", "lower_slide": "m/s", "upper_slide": "m/s"},
                                        "moves": []}}

    def gate(self, name, passed, **measured):
        passed = bool(passed)
        self.data["gates"].append({"name": name, "passed": passed,
                                    "elapsed_wall_seconds": round(time.monotonic()-self.started, 3), **measured})
        print(f"[{'OK' if passed else 'FAIL'}] {name}: {json.dumps(measured)}", flush=True)

    def save(self, filename):
        self.data["result"] = "PASS" if self.data["gates"] and all(g["passed"] for g in self.data["gates"]) else "FAIL"
        json_write(args.out_dir / filename, self.data)
        return self.data["result"] == "PASS"


def main():
    import isaaclab.sim as sim_utils
    import omni.usd
    from isaaclab.assets import RigidObject, RigidObjectCfg
    from isaaclab.sim import SimulationContext
    from dishsim_frigidaire.asset import BODY_POSITIONS, spawn, apply_mode, step_passive
    from dishsim.media import CameraRig

    started = time.monotonic()
    args.out_dir.mkdir(parents=True, exist_ok=True)
    asset_dir = args.usd.parent
    asset_hashes = hashes(asset_dir)
    if not args.usd.is_file():
        raise FileNotFoundError(args.usd)
    # Live measurements use physics-backed data buffers. Fabric avoids synchronizing
    # thousands of individual wire-collider transforms back to USD every physics tick.
    sim = SimulationContext(sim_utils.SimulationCfg(dt=1 / HZ, device=args.device, use_fabric=True))
    ground = sim_utils.CuboidCfg(size=(200., 200., .05),
                                  collision_props=sim_utils.CollisionPropertiesCfg(),
                                  visual_material=sim_utils.PreviewSurfaceCfg(
                                      diffuse_color=(.20, .225, .255), roughness=.8))
    ground.func("/World/Ground", ground, translation=(0., 0., -.025))
    dome = sim_utils.DomeLightCfg(intensity=1200, color=(.92, .95, 1.0))
    dome.func("/World/Fill", dome)
    key = sim_utils.DistantLightCfg(intensity=2400, angle=15, color=(1.0, .97, .93))
    key.func("/World/Key", key, orientation=(.9238795, .3826834, 0, 0))
    rim = sim_utils.DistantLightCfg(intensity=1100, angle=20, color=(.86, .92, 1.0))
    rim.func("/World/Rim", rim, orientation=(.7071068, -.5, .5, 0))
    dishwasher, basket = spawn(PRIM, mode="scripted", usd_path=str(args.usd))
    stage = omni.usd.get_context().get_stage()
    fixture_kinds = () if args.assembly_only else tuple(FIXTURE_SITES)
    sites = collect_sites(stage) if fixture_kinds else {}
    objects = {"basket": basket}
    for index, kind in enumerate(fixture_kinds):
        path = asset_dir / "fixtures" / f"{kind}.usdc"
        if not path.is_file():
            raise FileNotFoundError(path)
        objects[kind] = RigidObject(RigidObjectCfg(
            prim_path=f"/World/Fixtures/{kind}",
            spawn=sim_utils.UsdFileCfg(usd_path=str(path)),
            init_state=RigidObjectCfg.InitialStateCfg(pos=(5 + index * .5, 4, .6)),
        ))
    offset_dishwasher = offset_basket = None
    if not args.render_only:
        offset_dishwasher, offset_basket = spawn("/World/Relocated", position=(2.6, 2.4, 0), yaw=.63,
                                                mode="scripted", usd_path=str(args.usd))
        UsdGeom.Imageable(stage.GetPrimAtPath("/World/Relocated")).MakeInvisible()

    displays = {}
    exploded = {}
    if not args.physics_only:
        # Reference exact component USDs, disabling dynamics only in this inspection scene.
        # Each has its own space so cameras can show every side, including the basket floor.
        for i, (stem, name) in enumerate(COMPONENTS.items()):
            for group, translation in (("Components", (10 + i * 3, 0, 1.0)),
                                       ("Exploded", (28 + (-.9 if name == "Door" else 0),
                                                     -.8 if name in ("LowerRack", "Door") else .15,
                                                     1.05 if name == "UpperRack" else
                                                     .48 if name == "LowerRack" else
                                                     .52 if name == "SilverwareBasket" else
                                                     .18 if name == "Door" else 0))):
                if group == "Exploded" and name == "SilverwareBasket":
                    translation = (28.7, -.8, .51)
                prim_path = f"/World/{group}/{name}"
                prim = stage.DefinePrim(prim_path, "Xform")
                prim.GetReferences().AddReference(str(asset_dir / f"{stem}.usdc"))
                xform = UsdGeom.Xformable(prim)
                xform.ClearXformOpOrder()
                xform.AddTranslateOp().Set(Gf.Vec3d(*translation))
                if group == "Exploded" and name == "Door":
                    xform.AddRotateXOp().Set(90)
                for child in Usd.PrimRange(prim):
                    if child.HasAPI(UsdPhysics.RigidBodyAPI):
                        UsdPhysics.RigidBodyAPI(child).CreateRigidBodyEnabledAttr(False)
                    if child.HasAPI(UsdPhysics.CollisionAPI):
                        UsdPhysics.CollisionAPI(child).CreateCollisionEnabledAttr(False)
                    if child.IsA(UsdPhysics.Joint):
                        UsdPhysics.Joint(child).CreateJointEnabledAttr(False)
                (displays if group == "Components" else exploded)[stem] = prim_path
        rig = CameraRig({"inspection": ((1.6, -2.1, 1.45), (0, -.30, .44),
                                        {"focal_length": 48., "horizontal_aperture": 36.})},
                        hw=(args.height, args.width))
    else:
        rig = None
    contact_view = None
    if rig:
        import carb.settings
        from isaacsim.core.api.sensors import RigidContactView
        # Isaac Lab disables contact report processing by default. Its ContactSensor
        # enables the same setting; the core detailed-contact view needs it explicitly.
        carb.settings.get_settings().set_bool("/physics/disableContactProcessing", False)
        contact_paths = [f"{PRIM}/{name}" for name in COMPONENTS.values()]
        contact_paths += [f"/World/Fixtures/{kind}" for kind in fixture_kinds]
        contact_view = RigidContactView(prim_paths_expr=contact_paths,
                                        filter_paths_expr=[list(contact_paths) for _ in contact_paths],
                                        name="frigidaire_inspection_contacts", max_contact_count=8192,
                                        disable_stablization=False)
    sim.reset()
    if contact_view:
        contact_view.initialize()
    if rig:
        rig.apply_poses(sim.device)
    dt = sim.get_physics_dt()
    jidx = {name: dishwasher.joint_names.index(name) for name in JOINTS}
    target = dishwasher.data.default_joint_pos.clone()
    offset_target = offset_dishwasher.data.default_joint_pos.clone() if offset_dishwasher else None
    passive = False

    def tick(n=1, render=False):
        for _ in range(n):
            if time.monotonic() - started > 25 * 60:
                raise TimeoutError("Evidence job exceeded its 25-minute execution budget; split physics and rendering.")
            if passive:
                step_passive(dishwasher)
            else:
                dishwasher.set_joint_position_target(target)
            dishwasher.write_data_to_sim()
            if offset_dishwasher:
                offset_dishwasher.set_joint_position_target(offset_target)
                offset_dishwasher.write_data_to_sim()
            sim.step(render=render)
            dishwasher.update(dt)
            for obj in objects.values():
                obj.update(dt)
            if offset_dishwasher:
                offset_dishwasher.update(dt)
                offset_basket.update(dt)

    def set_pose(obj, position, quaternion=(1, 0, 0, 0)):
        obj.write_root_pose_to_sim(torch.tensor([[*position, *quaternion]], device=sim.device, dtype=torch.float32))
        obj.write_root_velocity_to_sim(torch.zeros((1, 6), device=sim.device))

    def joint_positions():
        q = dishwasher.data.joint_pos[0].cpu().numpy()
        return {name: float(q[jidx[name]]) for name in JOINTS}

    def ramp(goal, seconds=3.5):
        start = target.clone()
        duration = max(3.5, seconds, *(1.5*abs(float(start[0, jidx[name]])-value)/COMMAND_SPEED[name]
                                       for name, value in goal.items()))
        steps = math.ceil(duration * HZ)
        measured_peak = dict.fromkeys(goal, 0.)
        for k in range(1, steps + 1):
            a = k / steps
            a = a * a * (3 - 2 * a)
            for name, value in goal.items():
                target[0, jidx[name]] = start[0, jidx[name]] * (1 - a) + value * a
            tick()
            velocities = dishwasher.data.joint_vel[0].cpu().numpy()
            for name in goal:
                measured_peak[name] = max(measured_peak[name], abs(float(velocities[jidx[name]])))
        report.data["motion_profile"]["moves"].append({
            "initial": {name: float(start[0, jidx[name]]) for name in goal}, "target": goal,
            "duration_seconds": steps/HZ,
            "peak_commanded_speed": {name: 1.5*abs(float(start[0, jidx[name]])-value)/(steps/HZ)
                                     for name, value in goal.items()},
            "peak_measured_speed": measured_peak})
        tick(HZ)

    def site_pose(kind):
        site = sites[kind]
        position = list(site["position"])
        joint = "lower_slide" if site["body"] in ("LowerRack", "SilverwareBasket") else "upper_slide"
        position[1] += joint_positions()[joint]
        return position, site["quat_wxyz"]

    def measure_target(report, name, goal):
        actual = joint_positions()
        error = {j: actual[j] - v for j, v in goal.items()}
        report.gate(name, all(abs(error[j]) <= TOLERANCE[j] for j in goal),
                    positions=actual, errors=error, tolerance=TOLERANCE)

    def translation_probe(obj, displacement, speed=.10):
        """Ideal velocity grasp with contacts enabled; report tracking and disturbance.

        This is an access fixture, not a robot/gripper model. Root velocities are set,
        root poses are never overwritten during the path, so blocked motion is measured.
        """
        initial = obj.data.root_pos_w[0].cpu().numpy().copy()
        direction = np.asarray(displacement, dtype=float)
        distance = float(np.linalg.norm(direction))
        steps = max(1, round(distance / speed * HZ))
        velocity = direction / (steps * dt)
        # PhysX applies gravity before integrating position. The ideal held-object
        # controller supplies that known acceleration; contact impulses still act.
        commanded_velocity = velocity + np.array([0., 0., 9.81 * dt])
        max_lag = 0.
        rack_before = joint_positions()
        for i in range(steps):
            obj.write_root_velocity_to_sim(torch.tensor([[*commanded_velocity, 0., 0., 0.]], device=sim.device,
                                                        dtype=torch.float32))
            tick()
            actual = obj.data.root_pos_w[0].cpu().numpy()
            expected = initial + direction * (i + 1) / steps
            max_lag = max(max_lag, float(np.linalg.norm(actual - expected)))
        obj.write_root_velocity_to_sim(torch.zeros((1, 6), device=sim.device))
        end = obj.data.root_pos_w[0].cpu().numpy().copy()
        rack_error = max(abs(joint_positions()[j] - rack_before[j]) for j in ("lower_slide", "upper_slide"))
        return {"requested_m": direction.tolist(), "actual_m": (end - initial).tolist(),
                "initial_position_m": initial.tolist(), "final_position_m": end.tolist(),
                "max_tracking_error_m": max_lag, "rack_disturbance_m": rack_error}

    def translation_path(obj, displacements, speed=.10):
        segments = [translation_probe(obj, displacement, speed=speed) for displacement in displacements]
        return {"segments": segments, "requested_m": np.sum(displacements, axis=0).tolist(),
                "actual_m": np.sum([s["actual_m"] for s in segments], axis=0).tolist(),
                "max_tracking_error_m": max(s["max_tracking_error_m"] for s in segments),
                "rack_disturbance_m": max(s["rack_disturbance_m"] for s in segments)}

    physics_ok = True
    if not args.render_only:
        report = Evidence(asset_hashes)
        if fixture_kinds:
            report.data["sites"] = sites
            report.data["fixture_dimensions"] = {"plate_diameter_m": .260, "bowl_diameter_m": .140,
                                                  "cup_diameter_m": .080, "cup_height_m": .100,
                                                  "utensil_length_m": .180}
        warmup_started = time.monotonic()
        tick(HZ)
        report.data["one_second_warmup_wall_seconds"] = time.monotonic() - warmup_started
        print(f"[INFO] {HZ} physics steps on {sim.device}: "
              f"{report.data['one_second_warmup_wall_seconds']:.3f} wall seconds", flush=True)
        report.gate("three named joints", set(dishwasher.joint_names) == set(JOINTS),
                    names=dishwasher.joint_names)
        measure_target(report, "empty closed endpoints", dict.fromkeys(JOINTS, 0.))
        ramp({"door_hinge": EXTENDED["door_hinge"]})
        measure_target(report, "empty door fully open", {"door_hinge": EXTENDED["door_hinge"]})
        ramp({"lower_slide": EXTENDED["lower_slide"], "upper_slide": EXTENDED["upper_slide"]})
        measure_target(report, "empty racks full extension", EXTENDED)

        # Grasp/lift the empty removable basket.
        ramp({"upper_slide": 0.})
        basket_before = basket.data.root_pos_w[0].cpu().numpy().copy()
        # The rear-right basket partly remains below the upper rack when the lower
        # rack is extended. Clear the lower tines, draw forward out from beneath
        # the upper floor, and only then complete the lift.
        basket_lift = translation_path(basket, ((0, 0, .14), (0, -.14, 0), (0, 0, .09)))
        basket_replace = translation_path(basket, ((0, 0, -.09), (0, .14, 0), (0, 0, -.13)))
        tick(2 * HZ)
        basket_after = basket.data.root_pos_w[0].cpu().numpy().copy()
        report.gate("basket lift and replacement", basket_lift["max_tracking_error_m"] < .02
                    and basket_replace["max_tracking_error_m"] < .02
                    and np.linalg.norm(basket_after - basket_before) < .03,
                    lift=basket_lift, replace=basket_replace, settled_drift_m=(basket_after-basket_before).tolist())

        if args.assembly_only:
            # Complete the empty cycle before the obstruction and passive probes.
            ramp({"lower_slide": 0., "upper_slide": 0.})
            measure_target(report, "empty racks retracted", {"lower_slide": 0., "upper_slide": 0.})
            ramp({"door_hinge": 0.})
            measure_target(report, "empty door closed after cycle", dict.fromkeys(JOINTS, 0.))
            ramp({"door_hinge": EXTENDED["door_hinge"]})
            ramp({"lower_slide": EXTENDED["lower_slide"], "upper_slide": EXTENDED["upper_slide"]})
            measure_target(report, "empty re-open and extend", EXTENDED)
            ramp({"upper_slide": 0.})
        else:
            # Insert each full-size fixture from above its extended rack. Fixtures remain free
            # dynamic bodies after release; no fixing joint or pose reset carries them along.
            for kind in ("plate", "utensil", "bowl", "cup"):
                if kind == "cup":
                    ramp({"upper_slide": EXTENDED["upper_slide"]})
                position, quaternion = site_pose(kind)
                drop = np.asarray(position) + (0, 0, .14)
                set_pose(objects[kind], drop, quaternion)
                tick()
                insertion = translation_probe(objects[kind], (0, 0, -.12), speed=.08)
                report.gate(f"{kind} insertion access", insertion["max_tracking_error_m"] < .025
                            and insertion["rack_disturbance_m"] < .01, **insertion)
            tail = {kind: [] for kind in FIXTURE_SITES}
            orientation_tail = {kind: [] for kind in FIXTURE_SITES}
            for step in range(3 * HZ):
                tick()
                if step >= 2 * HZ:
                    for kind in tail:
                        tail[kind].append(objects[kind].data.root_pos_w[0].cpu().numpy().copy())
                        orientation_tail[kind].append(objects[kind].data.root_quat_w[0].cpu().numpy().copy())
            before = {}
            before_orientation = {}
            for kind in FIXTURE_SITES:
                obj = objects[kind]
                actual = obj.data.root_pos_w[0].cpu().numpy().copy()
                expected, _ = site_pose(kind)
                span = float(np.ptp(np.asarray(tail[kind]), axis=0).max())
                speed = float(torch.linalg.vector_norm(obj.data.root_lin_vel_w[0]))
                drift = float(np.linalg.norm(actual[:2] - np.asarray(expected)[:2]))
                report.gate(f"{kind} supported and settled", drift < .06 and actual[2] > expected[2] - .09
                            and span < .005 and speed < .03,
                            position_m=actual.tolist(), initial_center_m=expected, xy_drift_m=drift,
                            final_second_span_m=span, final_speed_m_s=speed,
                            quaternion_wxyz=obj.data.root_quat_w[0].cpu().numpy().tolist(),
                            angular_velocity_rad_s=obj.data.root_ang_vel_w[0].cpu().numpy().tolist())
                if kind != "utensil":
                    angle = fixture_axis_angle(orientation_tail[kind][-1], sites[kind]["quat_wxyz"])
                    angular_span = max(fixture_axis_angle(q, orientation_tail[kind][0]) for q in orientation_tail[kind])
                    report.gate(f"{kind} keeps intended orientation", angle < math.radians(12)
                                and angular_span < math.radians(3), orientation_error_deg=math.degrees(angle),
                                final_second_rotation_span_deg=math.degrees(angular_span),
                                tilt_tolerance_deg=12, rotation_span_tolerance_deg=3)
                before[kind] = actual
                before_orientation[kind] = obj.data.root_quat_w[0].cpu().numpy().copy()
            report.data["measurements"]["loaded_extended"] = {k: v.tolist() for k, v in before.items()}
            ramp({"lower_slide": 0., "upper_slide": 0.}, seconds=3.5)
            measure_target(report, "loaded racks retracted", {"lower_slide": 0., "upper_slide": 0.})
            for kind, initial in before.items():
                actual = objects[kind].data.root_pos_w[0].cpu().numpy()
                joint = "lower_slide" if sites[kind]["body"] in ("LowerRack", "SilverwareBasket") else "upper_slide"
                delta = actual - initial
                report.gate(f"{kind} retained through rack travel", abs(delta[1] + EXTENDED[joint]) < .010
                            and abs(delta[0]) < .010 and abs(delta[2]) < .010,
                            displacement_m=delta.tolist(), expected_y_m=-EXTENDED[joint], relative_drift_tolerance_m=.010)
                if kind != "utensil":
                    angle = fixture_axis_angle(objects[kind].data.root_quat_w[0].cpu().numpy(), before_orientation[kind])
                    report.gate(f"{kind} orientation retained through travel", angle < math.radians(12),
                                orientation_error_deg=math.degrees(angle), tilt_tolerance_deg=12,
                                reference="actual settled orientation before rack motion")
            ramp({"door_hinge": 0.})
            measure_target(report, "loaded door closed", dict.fromkeys(JOINTS, 0.))
            ramp({"door_hinge": EXTENDED["door_hinge"]})
            ramp({"lower_slide": EXTENDED["lower_slide"], "upper_slide": EXTENDED["upper_slide"]}, seconds=3.5)
            measure_target(report, "loaded re-open and extend", EXTENDED)
            # Isolate vertical access to each rack, as in actual dishwasher loading.
            for kind in ("cup", "bowl", "utensil", "plate"):
                if sites[kind]["body"] in ("LowerRack", "SilverwareBasket"):
                    ramp({"upper_slide": 0.})
                extraction = translation_probe(objects[kind], (0, 0, .28), speed=.10)
                report.gate(f"{kind} removal access", extraction["max_tracking_error_m"] < .025
                            and extraction["rack_disturbance_m"] < .01, **extraction)
                set_pose(objects[kind], (5 + list(FIXTURE_SITES).index(kind) * .5, 4, .6))

        # An extended rack must stop the door. This checks actual self-contact, not only
        # scripted sequencing. Restore the open door before retracting the rack.
        ramp({"door_hinge": 0.}, seconds=1.5)
        blocked = joint_positions()
        report.gate("extended lower rack obstructs door closure", blocked["door_hinge"] > math.radians(20)
                    and blocked["lower_slide"] < -.40, positions=blocked)
        ramp({"door_hinge": EXTENDED["door_hinge"]})
        ramp({"lower_slide": -.25, "upper_slide": -.20})

        # Force at each handle, expressed in its link frame. Direct pose changes are not
        # used during these probes; drives are disabled by the public passive API.
        apply_mode(dishwasher, "passive")
        passive = True
        for joint, body, force in (("lower_slide", "LowerRack", (0, -18., 0)),
                                   ("upper_slide", "UpperRack", (0, -14., 0))):
            index = dishwasher.body_names.index(body)
            initial = joint_positions()[joint]
            handle = stage.GetPrimAtPath(f"{PRIM}/{body}/Manipulation/handle_center")
            handle_pos = UsdGeom.Xformable(handle).GetLocalTransformation().ExtractTranslation() if handle else Gf.Vec3d(0, -.24, .10)
            dishwasher.set_external_force_and_torque(
                forces=torch.tensor([[force]], device=sim.device),
                torques=torch.zeros((1, 1, 3), device=sim.device),
                positions=torch.tensor([[list(handle_pos)]], device=sim.device, dtype=torch.float32), body_ids=[index])
            tick(round(.30 * HZ))
            travel = joint_positions()[joint] - initial
            dishwasher.set_external_force_and_torque(torch.zeros((1, dishwasher.num_bodies, 3), device=sim.device),
                                                       torch.zeros((1, dishwasher.num_bodies, 3), device=sim.device))
            report.gate(f"passive {joint} handle force moves rack", travel < -.003,
                        force_n=list(force), duration_s=.30, displacement_m=travel)
        apply_mode(dishwasher, "scripted")
        passive = False
        target[:] = dishwasher.data.joint_pos
        ramp({"lower_slide": 0., "upper_slide": 0.})
        ramp({"door_hinge": math.radians(45)})
        apply_mode(dishwasher, "passive")
        passive = True
        door_index = dishwasher.body_names.index("Door")
        initial_door = joint_positions()["door_hinge"]
        dishwasher.set_external_force_and_torque(
            forces=torch.tensor([[[0., -15., 0.]]], device=sim.device),
            torques=torch.zeros((1, 1, 3), device=sim.device),
            positions=torch.tensor([[[0., 0., .56]]], device=sim.device), body_ids=[door_index])
        tick(round(.35 * HZ))
        door_travel = joint_positions()["door_hinge"] - initial_door
        dishwasher.set_external_force_and_torque(torch.zeros((1, dishwasher.num_bodies, 3), device=sim.device),
                                                   torch.zeros((1, dishwasher.num_bodies, 3), device=sim.device))
        report.gate("passive door handle force opens door", door_travel > math.radians(1),
                    force_n=[0, -15, 0], lever_arm_m=.56, duration_s=.35, displacement_rad=door_travel)
        apply_mode(dishwasher, "scripted")
        passive = False
        target[:] = dishwasher.data.joint_pos
        ramp({"door_hinge": math.pi / 2})

        # The world-side anchor must preserve the requested translated/yawed spawn pose.
        offset_ids = {n: offset_dishwasher.joint_names.index(n) for n in JOINTS}
        base_initial = offset_dishwasher.data.root_pos_w[0].cpu().numpy().copy()
        for name, value in EXTENDED.items():
            offset_target[0, offset_ids[name]] = value
            tick(4 * HZ)
        base_final = offset_dishwasher.data.root_pos_w[0].cpu().numpy()
        offset_q = offset_dishwasher.data.joint_pos[0].cpu().numpy()
        base_quaternion = offset_dishwasher.data.root_quat_w[0].cpu().numpy()
        desired_quaternion = np.array([math.cos(.63 / 2), 0, 0, math.sin(.63 / 2)])
        quaternion_error = 2 * math.acos(min(1., abs(float(np.dot(base_quaternion, desired_quaternion)))))
        report.gate("translated and rotated spawn remains anchored", np.linalg.norm(base_final - (2.6, 2.4, 0)) < .002
                    and np.linalg.norm(base_final - base_initial) < .001 and quaternion_error < math.radians(.5),
                    root_position_m=base_final.tolist(), requested_position_m=[2.6, 2.4, 0],
                    yaw_error_rad=quaternion_error)
        report.gate("relocated articulation full travel", all(abs(offset_q[offset_ids[j]] - EXTENDED[j]) <= TOLERANCE[j]
                                                               for j in JOINTS),
                    positions={j: float(offset_q[offset_ids[j]]) for j in JOINTS})
        report.data["wall_seconds"] = time.monotonic() - started
        physics_ok = report.save("physics.json")

    if rig:
        render_report = Evidence(asset_hashes)
        contact_report = Evidence(asset_hashes)
        contact_report.data["measurement_scope"] = "Signed PhysX contact separation at settled inspection states, excluding disabled display geometry. Motion is validated separately in physics.json."
        render_report.data["images"] = []
        render_report.data["component_view_conventions"] = {
            "appliance_and_racks": "front from -Y; right from +X; front_left from -X/-Y",
            "silverware_basket": "front long handle side from +X; right end from +Y; front_left from +X/-Y",
        }
        camera = rig.cams["inspection"]

        def visible(paths, value):
            for path in paths:
                imageable = UsdGeom.Imageable(stage.GetPrimAtPath(path))
                if value:
                    imageable.MakeVisible()
                else:
                    imageable.MakeInvisible()

        def photograph(stem, label, eye, look_at):
            from PIL import Image
            camera.set_world_poses_from_view(eyes=torch.tensor([eye], device=sim.device, dtype=torch.float32),
                                            targets=torch.tensor([look_at], device=sim.device, dtype=torch.float32))
            for _ in range(10):
                sim.render()
                rig.update(dt)
            pixels = rig.grab_one("inspection")
            path = args.out_dir / f"{stem}.png"
            Image.fromarray(pixels).save(path)
            passed = pixels.shape[:2] == (args.height, args.width) and pixels.std() > 5 and pixels.max() > 60
            render_report.gate(f"image {stem}", passed, resolution=[pixels.shape[1], pixels.shape[0]],
                               rgb_std=float(pixels.std()), bytes=path.stat().st_size)
            render_report.data["images"].append({"file": path.name, "label": label,
                                                "eye_m": list(eye), "target_m": list(look_at),
                                                "sha256": hashlib.sha256(path.read_bytes()).hexdigest()})

        def set_state(door, lower, upper, loaded=False):
            for name, value in zip(JOINTS, (door, lower, upper)):
                target[0, jidx[name]] = value
            dishwasher.write_joint_state_to_sim(target, torch.zeros_like(target))
            dishwasher.set_joint_position_target(target)
            basket_position = np.asarray(BODY_POSITIONS["SilverwareBasket"]) + (0, lower, 0)
            set_pose(basket, basket_position)
            for index, kind in enumerate(fixture_kinds):
                if loaded:
                    site = sites[kind]
                    position = list(site["position"])
                    position[1] += lower if site["body"] in ("LowerRack", "SilverwareBasket") else upper
                    position[2] += .010
                    set_pose(objects[kind], position, site["quat_wxyz"])
                else:
                    set_pose(objects[kind], (5 + index * .5, 4, .6))
            tick(HZ)

        def inspect_contacts(name):
            def array(value):
                return value.detach().cpu().numpy() if hasattr(value, "detach") else np.asarray(value)
            depths, counts, overflow, invalid_data = [], [], False, False
            deepest = None
            for i in range(HZ):
                tick()
                if i % 6:
                    continue
                data = contact_view.get_contact_force_data(dt=dt)
                if data is None:
                    raise RuntimeError("PhysX contact data unavailable")
                separations = array(data[3]).reshape(-1)
                pair_counts = array(data[4]).reshape(-1)
                starts = array(data[5]).reshape(-1)
                count = int(pair_counts.sum())
                valid = []
                for pair_index, (start, n) in enumerate(zip(starts, pair_counts)):
                    start, n = int(start), int(n)
                    invalid_data = invalid_data or start < 0 or n < 0
                    if n:
                        overflow = overflow or start+n > len(separations)
                        valid.extend(separations[start:start+n])
                        index = start + int(np.argmin(separations[start:start+n]))
                        if deepest is None or separations[index] < deepest["separation_m"]:
                            deepest = {"sensor_body": contact_paths[pair_index // len(contact_paths)],
                                       "filter_body": contact_paths[pair_index % len(contact_paths)],
                                       "separation_m": float(separations[index]),
                                       "position_m": array(data[1])[index].tolist(),
                                       "normal": array(data[2])[index].tolist(),
                                       "normal_force_n": float(array(data[0]).reshape(-1)[index])}
                # A full contact buffer is a potential truncation, never evidence of
                # clearance. Pair offsets/counts must also remain inside the allocation.
                overflow = overflow or count >= contact_view.max_contact_count
                invalid_data = invalid_data or not bool(np.isfinite(valid).all())
                counts.append(count)
                depths.append(max(0., -min(valid)) if valid else 0.)
            peak = float(max(depths))
            persistent = float(np.median(depths))
            contact_report.data["measurements"][name] = {
                "deepest_contact": deepest,
                "objects": {kind: {"position_m": obj.data.root_pos_w[0].cpu().numpy().tolist(),
                                    "quaternion_wxyz": obj.data.root_quat_w[0].cpu().numpy().tolist(),
                                    "linear_velocity_m_s": obj.data.root_lin_vel_w[0].cpu().numpy().tolist(),
                                    "angular_velocity_rad_s": obj.data.root_ang_vel_w[0].cpu().numpy().tolist()}
                            for kind, obj in objects.items()},
            }
            contact_report.gate(f"{name} settled contacts", not overflow and not invalid_data and max(counts) > 0
                                and peak < .002 and persistent < .001,
                                sample_duration_s=1, samples=len(depths), peak_penetration_m=peak,
                                median_max_penetration_m=persistent, peak_tolerance_m=.002,
                                persistent_tolerance_m=.001, maximum_contact_count=max(counts),
                                contact_buffer_capacity=contact_view.max_contact_count, buffer_overflow=overflow,
                                invalid_contact_data=invalid_data)

        set_state(0, 0, 0)
        # Same focal length and the actual authored components throughout the gallery.
        assembly_paths = [PRIM] + [obj.cfg.prim_path for obj in objects.values()]
        visible(assembly_paths + list(displays.values()) + list(exploded.values()), False)
        for stem, prim_path in displays.items():
            visible([prim_path], True)
            box = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render]).ComputeWorldBound(
                stage.GetPrimAtPath(prim_path)).ComputeAlignedRange()
            center = np.array(box.GetMidpoint())
            size = np.array(box.GetSize())
            distance = max(float(size.max()) * 2.45, .55)
            views = {"front": np.array([0, -1, .10]), "right": np.array([1, 0, .15]),
                     "front_left": np.array([-.75, -1, .80]), "top": np.array([0, -.02, 1.])}
            if stem == "silverware_basket":
                # The basket reference's "front" is its long handle side (+X),
                # unlike the appliance and rack photographs, whose front is -Y.
                views.update(front=np.array([1., 0, .10]), right=np.array([0, 1., .15]),
                             front_left=np.array([1., -.75, .80]))
            for view, direction in views.items():
                eye = center + direction / np.linalg.norm(direction) * distance
                photograph(f"{stem}_{view}", f"{stem.replace('_', ' ').title()} — {view.replace('_', ' ')}",
                           eye, center)
            if stem in ("lower_rack", "upper_rack"):
                close_center = center + (-.09, -.045, -.02)
                photograph(f"{stem}_wire_closeup", f"{stem.replace('_', ' ').title()} — wire and tine bends",
                           close_center + (-.30, -.38, .33), close_center)
            if stem == "silverware_basket":
                photograph("silverware_basket_underside", "Silverware basket — underside and open lattice",
                           center + (.28, -.34, -.35), center)
            visible([prim_path], False)
        visible(exploded.values(), True)
        photograph("assembled_exploded", "Exploded components", (30.7, -3.5, 2.5), (28., -.20, .61))
        visible(exploded.values(), False)
        visible(assembly_paths, True)
        states = (("closed", (0, 0, 0)), ("open_retracted", (math.pi / 2, 0, 0)),
                  ("lower_extended", (math.pi / 2, -.49, 0)),
                  ("upper_extended", (math.pi / 2, 0, -.44)),
                  ("both_extended", (math.pi / 2, -.49, -.44)))
        for name, state in states:
            set_state(*state)
            inspect_contacts(name)
            photograph(f"assembled_{name}", f"Assembled — {name.replace('_', ' ')}",
                       (1.65, -2.2, 1.6), (0, -.33, .43))
        if fixture_kinds:
            set_state(math.pi / 2, -.49, -.44, loaded=True)
            tick(2 * HZ)
            inspect_contacts("loaded")
            photograph("assembled_loaded", "Assembled — representative full-size load", (1.5, -2.0, 1.7), (0, -.36, .43))
            photograph("assembled_loaded_front", "Assembled load — front", (0, -2.6, 1.0), (0, -.25, .43))
        render_report.data["wall_seconds"] = time.monotonic() - started
        render_report.data["image_note"] = "Rendered from exact component USD references and articulated assembly. Inspection poses are set before settling; physics certification is recorded separately."
        render_report.data["contact_sheet"] = write_gallery(render_report.data["images"])
        render_ok = render_report.save("renders.json")
        contact_ok = contact_report.save("contacts.json")
        render_ok = render_ok and contact_ok
    else:
        render_ok = True

    merge_reports(asset_hashes)
    print(f"[RESULT] {'PASS' if physics_ok and render_ok else 'FAIL'} — requested evidence run; "
          f"combined certification: {json.loads((args.out_dir / 'evidence.json').read_text())['result']}", flush=True)


def write_gallery(images):
    import html
    from PIL import Image, ImageDraw, ImageFont
    extended_view = (("assembled_both_extended.png", "Both racks extended") if args.assembly_only
                     else ("assembled_loaded.png", "Loaded appliance"))
    sheet_sources = [("assembled_closed.png", "Closed appliance"), extended_view,
                     ("lower_rack_front_left.png", "Lower rack"), ("upper_rack_front_left.png", "Upper rack"),
                     ("silverware_basket_front_left.png", "Silverware basket"), ("assembled_exploded.png", "Exploded components")]
    thumbnails = []
    for filename, _ in sheet_sources:
        with Image.open(args.out_dir / filename) as source:
            thumbnails.append(np.asarray(source.convert("RGB").resize((640, 480), Image.Resampling.LANCZOS)))
    sheet_path = args.out_dir / "contact_sheet.png"
    sheet = Image.new("RGB", (1920, 1016), (20, 20, 20))
    draw = ImageDraw.Draw(sheet)
    font = ImageFont.load_default(size=20)
    for i, (thumbnail, (_, label)) in enumerate(zip(thumbnails, sheet_sources)):
        row, column = divmod(i, 3)
        x, y = column * 640, row * 508
        sheet.paste(Image.fromarray(thumbnail), (x, y+28))
        draw.text((x+10, y+3), label, font=font, fill=(240, 240, 240))
    sheet.save(sheet_path)
    entries = "\n".join(f'<figure><a href="{html.escape(item["file"])}"><img loading="lazy" src="{html.escape(item["file"])}" '
                        f'alt="{html.escape(item["label"])}"></a><figcaption>{html.escape(item["label"])}</figcaption></figure>'
                        for item in images)
    (args.out_dir / "index.html").write_text('<!doctype html><html lang="en"><meta charset="utf-8">'
        '<meta name="viewport" content="width=device-width, initial-scale=1"><title>Frigidaire FDPC4221AS geometry</title>'
        '<style>body{font:16px system-ui;background:#18212b;color:#edf1f4;margin:2rem}main{display:grid;'
        'grid-template-columns:repeat(auto-fit,minmax(320px,1fr));gap:1rem}figure{margin:0;background:#263440;'
        'border-radius:8px;overflow:hidden}img{display:block;width:100%}figcaption{padding:1rem}a{color:#acd9ff}</style>'
        '<h1>Frigidaire FDPC4221AS</h1><p>Photographs of the generated USD geometry in Isaac Sim. '
        'Select any image for the full resolution. <a href="evidence.json">Measured validation</a>. '
        '<a href="contact_sheet.png">Six-view contact sheet</a>.</p>'
        f'<main>{entries}</main></html>')
    lines = ["# Frigidaire FDPC4221AS image index", "", "Rendered from the generated USD geometry in Isaac Sim.", ""]
    lines.extend(f'- [{entry["label"]}]({entry["file"]})' for entry in images)
    (args.out_dir / "image_index.md").write_text("\n".join(lines) + "\n")
    return {"file": sheet_path.name, "source_images": [name for name, _ in sheet_sources],
            "resolution": [1920, 1016], "method": "Full-frame rendered PNGs resized into a labeled 3 by 2 grid",
            "sha256": hashlib.sha256(sheet_path.read_bytes()).hexdigest()}


def merge_reports(asset_hashes):
    reports = {}
    current_sources = source_hashes()
    for name in ("physics", "renders", "contacts"):
        path = args.out_dir / f"{name}.json"
        if path.exists():
            data = json.loads(path.read_text())
            matches = (data.get("asset_hashes") == asset_hashes
                       and data.get("source_hashes") == current_sources
                       and data.get("scope") == evidence_scope())
            reports[name] = data if matches else {"result": "STALE"}
        else:
            reports[name] = {"result": "NOT_RUN"}
    statuses = [r["result"] for r in reports.values()]
    combined = "PASS" if all(r == "PASS" for r in statuses) else "FAIL" if "FAIL" in statuses else "INCOMPLETE"
    json_write(args.out_dir / "evidence.json", {"result": combined, "scope": evidence_scope(),
                                                "asset_hashes": asset_hashes, "source_hashes": current_sources,
                                                "reports": reports, "image_index": "index.html"})


if __name__ == "__main__":
    try:
        main()
    except BaseException as exc:
        import traceback
        traceback.print_exc()
        args.out_dir.mkdir(parents=True, exist_ok=True)
        json_write(args.out_dir / "failure.json", {"result": "FAIL", "scope": evidence_scope(), "exception": repr(exc)})
        current_hashes = hashes(args.usd.parent)
        filename = "renders.json" if args.render_only else "physics.json"
        json_write(args.out_dir / filename, {"result": "FAIL", "scope": evidence_scope(), "asset_hashes": current_hashes,
                                             "source_hashes": source_hashes(), "exception": repr(exc)})
        merge_reports(current_hashes)
        print("[RESULT] FAIL — exception; see failure.json", flush=True)
    finally:
        release_sim_for_close()
        simulation_app.close()
