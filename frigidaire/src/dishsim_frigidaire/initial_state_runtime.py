"""Joint Isaac validation of reusable accepted-pose dish arrangements.

Create one backend per fresh Kit process. Constructor imports are deliberately
Kit-free; instantiate only after AppLauncher. All public poses are world metres
and XYZW, unless explicitly named rack_local_pose. The door remains open.
"""
from __future__ import annotations

from collections import deque
from copy import deepcopy
import json
import math
from pathlib import Path
import re
import time

import numpy as np

from dishsim.quats import wxyz_to_xyzw, xyzw_to_wxyz
from .random_pose_experiment import check_deadline, BudgetExpired
from .random_pose_runtime import (COMPONENTS, JOINTS, RACK_JOINT, EXTENSION,
                                  CONTACT_CAPACITY, CONTACT_PERSISTENCE_M, ContactTracker)
from .random_poses import (KINDS, RACKS, LIMITS, compose_pose, relative_pose,
                           quaternion_matrix_xyzw, motion_is_settled,
                           transform_vertices, vertices_contained)

OBSERVATION_SECONDS = 5.
RACK_COMMAND_SPEED_M_S = .08
RACK_SPEED_MEASUREMENT_TOLERANCE_M_S = 1e-5


def support_paths(pairs, assignments):
    """Contact chains must reach each object's assigned rack through dishes.

    Other appliance bodies are never transit nodes. The basket is available to
    lower-rack objects only, and qualifies only if directly seated on that rack.
    This is a contact-connectivity gate, not a force or wrench certificate.
    """
    edges = {tuple(sorted(pair)) for pair in pairs if len(pair) == 2 and pair[0] != pair[1]}
    identities = set(assignments)
    result = {}
    for identity, rack in assignments.items():
        allowed = identities | {rack}
        if rack == "LowerRack" and tuple(sorted(("SilverwareBasket", rack))) in edges:
            allowed.add("SilverwareBasket")
        neighbors = {name: [] for name in allowed}
        for first, second in edges:
            if first in allowed and second in allowed:
                neighbors[first].append(second)
                neighbors[second].append(first)
        queue = deque([[identity]])
        seen = {identity}
        result[identity] = None
        while queue:
            path = queue.popleft()
            if path[-1] == rack:
                result[identity] = path
                break
            for other in sorted(neighbors[path[-1]]):
                if other not in seen:
                    seen.add(other)
                    queue.append(path+[other])
    return result


def all_vertex_step_speed(points, previous, current, dt):
    """Maximum unsmoothed speed over every authored visual mesh vertex."""
    before = transform_vertices(points, previous["position_m"], previous["quaternion_xyzw"])
    after = transform_vertices(points, current["position_m"], current["quaternion_xyzw"])
    return float(np.linalg.norm(after-before, axis=1).max()/dt)


def window_motion_metrics(rows, name, dt):
    """Exact window metrics using all-vertex step speeds computed at 120 Hz."""
    positions = np.array([row["poses"][name]["position_m"] for row in rows])
    quaternions = np.array([row["poses"][name]["quaternion_xyzw"] for row in rows])
    quaternions /= np.linalg.norm(quaternions, axis=1)[:, None]
    angles = 2*np.arccos(np.clip(np.abs(quaternions @ quaternions[0]), 0., 1.))
    return {"root_position_span_m": float(np.ptp(positions, axis=0).max()),
            "quaternion_span_deg": float(np.degrees(angles.max())),
            "peak_mesh_point_speed_m_s": max(row["speeds"][name] for row in list(rows)[1:]),
            "sample_count": len(rows), "sample_duration_s": (len(rows)-1)*dt}


def retain_peak_event(previous, contact, poses, joints, *, phase, step, dt):
    """Preserve the exact 120 Hz worst event even between regular trace frames."""
    peak = float(contact["peak_m"])
    if previous is not None and peak <= previous["peak_penetration_m"]:
        return previous
    return {"step": step, "simulated_seconds": step*dt, "phase": phase,
            "peak_penetration_m": peak, "contact_pair_penetration_m": deepcopy(contact["depths_m"]),
            "raw_contact_pairs": sorted(contact["raw_pairs"]),
            "retained_only_contact_pairs": sorted(contact["retained_only_pairs"]),
            "poses": deepcopy(poses), "joints": deepcopy(joints)}


def normalize_candidates(candidates):
    """Check object identity without imposing catalog rack recommendations."""
    result, seen, candidate_ids = [], set(), set()
    for index, original in enumerate(candidates):
        entry = deepcopy(original)
        identity = entry.get("object_id", entry.get("id", f"dish_{index:04d}"))
        if (not isinstance(identity, str) or not re.fullmatch(r"[A-Za-z_][A-Za-z0-9_]*", identity)
                or identity in seen or identity in COMPONENTS):
            raise ValueError(f"Invalid or duplicate independent object ID: {identity!r}")
        seen.add(identity)
        candidate_id = entry.get("candidate_id")
        if candidate_id is not None:
            if candidate_id in candidate_ids:
                raise ValueError("An individual candidate may appear only once in a state")
            candidate_ids.add(candidate_id)
        if entry.get("kind") not in KINDS or entry.get("rack") not in RACKS:
            raise ValueError(f"Unsupported kind or assigned rack for {identity}")
        pose = entry.get("rack_local_pose", entry.get("pose_world"))
        if pose is None:
            pose = {"position_m": entry.get("position_m", entry.get("position")),
                    "quaternion_xyzw": entry.get("quaternion_xyzw", entry.get("quaternion"))}
            entry["pose_world"] = pose
        position = np.asarray(pose["position_m"], dtype=float)
        quaternion = np.asarray(pose["quaternion_xyzw"], dtype=float)
        if (position.shape != (3,) or quaternion.shape != (4,)
                or not np.isfinite(position).all() or not np.isfinite(quaternion).all()
                or abs(float(np.linalg.norm(quaternion))-1.) > 1e-5):
            raise ValueError(f"Invalid finite normalized pose for {identity}")
        entry["object_id"] = identity
        result.append(entry)
    return result


class MultiContactTracker(ContactTracker):
    def update(self, data, poses):
        result = super().update(data, poses)
        counts = data[4]
        counts = counts.detach().cpu().numpy() if hasattr(counts, "detach") else np.asarray(counts)
        raw = {tuple(sorted((self.names[first], self.names[second])))
               for first, second in np.argwhere(counts.reshape(len(self.names), -1) > 0)
               if first != second}
        result["raw_pairs"] = raw
        result["retained_only_pairs"] = result["pairs"]-raw
        result["depths_m"] = {"|".join(pair): float(item["depth_m"])
                              for pair, item in self.retained.items()}
        return result


class IsaacInitialStateBackend:
    """Fresh independent bodies, measured common frames, and loaded rack checks."""
    def __init__(self, usd_path, output_dir, *, device="cpu", domains,
                 deadline=float("inf"), app, candidates=()):
        import carb.settings
        import omni.usd
        import torch
        from pxr import PhysxSchema, UsdPhysics
        import isaaclab.sim as sim_utils
        from isaaclab.assets import RigidObject, RigidObjectCfg
        from isaaclab.sim import SimulationContext
        from isaacsim.core.api.sensors import RigidContactView
        from .asset import spawn, apply_mode, COMPONENT_FILES
        from .loading import visual_points

        if device != "cpu":
            raise ValueError("This experiment requires CPU physics")
        self.entries = normalize_candidates(candidates)
        self.assignments = {entry["object_id"]: entry["rack"] for entry in self.entries}
        self.directory = Path(output_dir)
        self.directory.mkdir(parents=True, exist_ok=True)
        self.usd_path = Path(usd_path)
        self.torch, self.deadline, self.app = torch, deadline, app
        self.domains = domains
        self.dt, self.hz = LIMITS["physics_dt_s"], 120
        self.phase, self.tick_index = "initialization", 0
        self.trace = None
        self.loaded = False
        self.maximum_penetration_m = 0.
        self.peak_event = None
        self.global_peak_event = None
        self.latest_contact = {"pairs": set(), "raw_pairs": set(), "retained_only_pairs": set(),
                               "peak_m": 0., "count": 0, "depths_m": {}}
        self.previous_frames = None
        self.latest_speeds = {}
        self.last_hold = None
        self.baseline = None
        check_deadline(deadline)
        self.sim = SimulationContext(sim_utils.SimulationCfg(dt=self.dt, device=device,
            use_fabric=True, physx=sim_utils.PhysxCfg(enable_ccd=True)))
        ground = sim_utils.CuboidCfg(size=(200., 200., .05),
                                    collision_props=sim_utils.CollisionPropertiesCfg())
        ground.func("/World/Ground", ground, translation=(0., 0., -.025))
        self.prim = "/World/InitialStateDishwasher"
        self.dishwasher, self.basket = spawn(self.prim, mode="scripted", usd_path=str(self.usd_path))
        self.objects = {}
        self.points = {kind: visual_points(self.usd_path.parent/"tableware"/(kind+".usdc"))
                       for kind in set(entry["kind"] for entry in self.entries)}
        for index, entry in enumerate(self.entries):
            identity = entry["object_id"]
            self.objects[identity] = RigidObject(RigidObjectCfg(
                prim_path="/World/InitialStateDishes/"+identity,
                spawn=sim_utils.UsdFileCfg(usd_path=str(self.usd_path.parent/"tableware"/(entry["kind"]+".usdc"))),
                init_state=RigidObjectCfg.InitialStateCfg(pos=(3.+index*.4, 0., .3))))
        stage = omni.usd.get_context().get_stage()
        for identity, obj in self.objects.items():
            body = stage.GetPrimAtPath(obj.cfg.prim_path)
            rigid = UsdPhysics.RigidBodyAPI(body)
            physx = PhysxSchema.PhysxRigidBodyAPI(body)
            if (not rigid.GetRigidBodyEnabledAttr().Get() or rigid.GetKinematicEnabledAttr().Get()
                    or physx.GetDisableGravityAttr().Get() or not physx.GetEnableCCDAttr().Get()):
                raise ValueError("Expected dynamic gravity-enabled CCD dish: "+identity)
        scenes = [prim for prim in stage.Traverse() if prim.IsA(UsdPhysics.Scene)]
        if len(scenes) != 1 or not PhysxSchema.PhysxSceneAPI(scenes[0]).GetEnableCCDAttr().Get():
            raise RuntimeError("Expected one CCD physics scene")
        carb.settings.get_settings().set_bool("/physics/disableContactProcessing", False)
        paths = [self.prim+"/"+name for name in COMPONENTS]
        paths += [obj.cfg.prim_path for obj in self.objects.values()]
        self.contacts = RigidContactView(prim_paths_expr=paths,
            filter_paths_expr=[paths.copy() for _ in paths], name="initial_state_contacts",
            max_contact_count=CONTACT_CAPACITY, disable_stablization=False)
        self.sim.reset()
        apply_mode(self.dishwasher, "scripted")
        self.contacts.initialize()
        if set(self.dishwasher.joint_names) != set(JOINTS):
            raise RuntimeError("Appliance joint names do not match the experiment")
        self.indices = {name: self.dishwasher.joint_names.index(name) for name in JOINTS}
        self.body_indices = {name: self.dishwasher.body_names.index(name) for name in COMPONENTS[:-1]}
        self.target = self.dishwasher.data.default_joint_pos.clone()
        geometry = {name: visual_points(self.usd_path.parent/filename)
                    for name, filename in COMPONENT_FILES.items()}
        geometry.update({entry["object_id"]: self.points[entry["kind"]] for entry in self.entries})
        radii = {name: float(np.linalg.norm(points, axis=1).max()) for name, points in geometry.items()}
        self.motion_points = {name: geometry[name] for name in (*self.objects, "SilverwareBasket")}
        self.tracker = MultiContactTracker((*COMPONENTS, *self.objects), radii)
        self.initial = self.snapshot()
        self.runtime_settings = {"physics_hz": self.hz, "device": device, "scene_ccd": True,
            "tableware_ccd": True, "contact_capacity": CONTACT_CAPACITY,
            "contact_persistence_m": CONTACT_PERSISTENCE_M, "trace_hz": 10,
            "motion_points": "every authored visual mesh vertex", "rest_checks_hz": self.hz,
            "observation_seconds": OBSERVATION_SECONDS, "thresholds": dict(LIMITS),
            "initial_settle_early_abort_on_peak_penetration": True,
            "commanded_rack_speed_cap_m_s": RACK_COMMAND_SPEED_M_S,
            "measured_rack_speed_cap_m_s": LIMITS["rack_speed_m_s"],
            "rack_speed_measurement_tolerance_m_s": RACK_SPEED_MEASUREMENT_TOLERANCE_M_S,
            "dynamic_gravity_enabled_dishes": True, "fresh_scene_per_arrangement": True,
            "support_rule": "contact path through dishes to assigned rack; seated basket allowed for lower rack",
            "drive_settings": {name: {"stiffness": float(self.dishwasher.data.joint_stiffness[0, i]),
                                       "damping": float(self.dishwasher.data.joint_damping[0, i])}
                               for name, i in self.indices.items()}}

    @staticmethod
    def _pose(position, quaternion_wxyz):
        position = position.detach().cpu().numpy().copy()
        quaternion = wxyz_to_xyzw(quaternion_wxyz.detach().cpu().numpy().copy())
        if not np.isfinite(position).all() or not np.isfinite(quaternion).all():
            raise RuntimeError("Nonfinite rigid body pose")
        return {"position_m": position.tolist(), "quaternion_xyzw": quaternion.tolist()}

    def poses(self):
        result = {name: self._pose(self.dishwasher.data.body_pos_w[0, i],
                                   self.dishwasher.data.body_quat_w[0, i])
                  for name, i in self.body_indices.items()}
        result["SilverwareBasket"] = self._pose(self.basket.data.root_pos_w[0], self.basket.data.root_quat_w[0])
        result.update({name: self._pose(obj.data.root_pos_w[0], obj.data.root_quat_w[0])
                       for name, obj in self.objects.items()})
        return result

    def joints(self):
        values = self.dishwasher.data.joint_pos[0].detach().cpu().numpy()
        if not np.isfinite(values).all():
            raise RuntimeError("Nonfinite joint positions")
        return {name: float(values[i]) for name, i in self.indices.items()}

    def snapshot(self):
        velocities = {name: obj.data.root_vel_w[0].detach().cpu().numpy().tolist()
                      for name, obj in {**self.objects, "SilverwareBasket": self.basket}.items()}
        return {"joints": self.joints(), "poses": self.poses(),
                "velocities_world": velocities, "step": self.tick_index}

    def set_rigid_pose(self, obj, pose):
        values = [*pose["position_m"], *xyzw_to_wxyz(pose["quaternion_xyzw"])]
        obj.write_root_pose_to_sim(self.torch.tensor([values], dtype=self.torch.float32,
                                                     device=self.sim.device))
        obj.write_root_velocity_to_sim(self.torch.zeros((1, 6), device=self.sim.device))
        obj.reset()

    def restore(self, snapshot):
        """State writes are restricted to initialization before dish release."""
        self.loaded = False
        for index, obj in enumerate(self.objects.values()):
            self.set_rigid_pose(obj, {"position_m": [3.+index*.4, 0., .3],
                                      "quaternion_xyzw": [0., 0., 0., 1.]})
        for name, value in snapshot["joints"].items():
            self.target[0, self.indices[name]] = value
        self.dishwasher.write_joint_state_to_sim(self.target, self.torch.zeros_like(self.target))
        self.dishwasher.write_root_velocity_to_sim(self.torch.zeros((1, 6), device=self.sim.device))
        self.dishwasher.reset()
        self.dishwasher.set_joint_position_target(self.target)
        self.set_rigid_pose(self.basket, snapshot["poses"]["SilverwareBasket"])
        self.tracker.clear()
        self.previous_frames = None
        self.maximum_penetration_m = 0.
        self.peak_event = None
        self.global_peak_event = None

    def tick(self):
        check_deadline(self.deadline)
        if not self.app.is_running():
            raise RuntimeError("Isaac application stopped during the experiment")
        self.dishwasher.set_joint_position_target(self.target)
        self.dishwasher.write_data_to_sim()
        self.sim.step(render=False)
        self.dishwasher.update(self.dt)
        self.basket.update(self.dt)
        for obj in self.objects.values():
            obj.update(self.dt)
        frames, joints = self.poses(), self.joints()
        self.latest_contact = self.tracker.update(
            self.contacts.get_contact_force_data(dt=self.dt, clone=False), frames)
        self.maximum_penetration_m = max(self.maximum_penetration_m, self.latest_contact["peak_m"])
        self.peak_event = retain_peak_event(self.peak_event, self.latest_contact, frames, joints,
                                           phase=self.phase, step=self.tick_index+1, dt=self.dt)
        if (self.global_peak_event is None
                or self.peak_event["peak_penetration_m"] > self.global_peak_event["peak_penetration_m"]):
            self.global_peak_event = self.peak_event
        names = (*self.objects, "SilverwareBasket") if self.loaded else ("SilverwareBasket",)
        self.latest_speeds = {name: (0. if self.previous_frames is None else
            all_vertex_step_speed(self.motion_points[name], self.previous_frames[name], frames[name], self.dt))
            for name in names}
        self.previous_frames = frames
        self.tick_index += 1
        if self.trace is not None and self.tick_index % 12 == 0:
            paths = support_paths(self.latest_contact["pairs"], self.assignments) if self.loaded else {}
            row = {"step": self.tick_index, "simulated_seconds": self.tick_index*self.dt,
                   "phase": self.phase, "joints": joints, "poses": frames,
                   "peak_penetration_m": self.latest_contact["peak_m"],
                   "contact_pairs": sorted(self.latest_contact["pairs"]),
                   "raw_contact_pairs": sorted(self.latest_contact["raw_pairs"]),
                   "retained_only_contact_pairs": sorted(self.latest_contact["retained_only_pairs"]),
                   "contact_pair_penetration_m": self.latest_contact["depths_m"],
                   "support_paths": paths, "mesh_point_speeds_m_s": self.latest_speeds}
            self.trace.write(json.dumps(row, allow_nan=False)+"\n")
        return frames, joints

    @staticmethod
    def goal(extended=()):
        goal = {"door_hinge": math.pi/2, "lower_slide": 0., "upper_slide": 0.}
        goal.update({RACK_JOINT[rack]: EXTENSION[rack] for rack in extended})
        return goal

    @staticmethod
    def endpoint_ok(values, goal):
        return all(abs(values[name]-value) <= (math.radians(.5) if name == "door_hinge"
                                               else LIMITS["rack_endpoint_m"])
                   for name, value in goal.items())

    def hold(self, *, phase, goal, timeout=12., observation=0., abort_on_peak=False):
        """Qualify by timeout, then require every trailing 1 s window for observation.

        Every window is evaluated at 120 Hz. A failed early observation can
        requalify within the original settle allowance. Five uninterrupted
        passing seconds remain mandatory; failed observations are recorded.
        """
        self.phase = phase
        window = deque(maxlen=round(LIMITS["rest_window_s"]/self.dt)+1)
        names = (*self.objects, "SilverwareBasket") if self.loaded else ("SilverwareBasket",)
        assignments = self.assignments if self.loaded else {}
        first_pass = None
        observation_restarts = []
        observation_started_count = 0
        checks = 0
        result = {"passed": False, "settled": False, "reason": "no complete qualifying rest window"}
        for step in range(math.ceil((timeout+observation)/self.dt)+1):
            frames, joints = self.tick()
            if (abort_on_peak and self.loaded
                    and self.maximum_penetration_m >= LIMITS["peak_penetration_m"]):
                result = {"passed": False, "settled": False, "early_abort": True,
                          "abort_gate": "maximum_initial_penetration",
                          "reason": "irreversible initial peak penetration failure",
                          "peak_penetration_m": self.maximum_penetration_m,
                          "simulated_seconds": (step+1)*self.dt,
                          "rest_window_count": checks, "observation_required_s": observation,
                          "observation_completed_s": 0., "continuous_passing_window_count": 0,
                          "observation_started_count": observation_started_count,
                          "observation_restarts": observation_restarts, "joints": joints}
                self.last_hold = result
                return result
            paths = support_paths(self.latest_contact["pairs"], assignments)
            row = {"poses": frames, "joints": joints, "speeds": self.latest_speeds,
                   "peak_m": self.latest_contact["peak_m"], "support_paths": paths,
                   "basket_supported": tuple(sorted(("SilverwareBasket", "LowerRack")))
                                       in self.latest_contact["pairs"]}
            window.append(row)
            if len(window) < window.maxlen:
                continue
            checks += 1
            metrics = {name: window_motion_metrics(window, name, self.dt) for name in names}
            peaks = [row["peak_m"] for row in window]
            unsupported = [identity for identity in assignments
                           if any(row["support_paths"][identity] is None for row in window)]
            result = {"passed": False, "settled": all(motion_is_settled(v) for v in metrics.values()),
                      "motion": metrics, "simulated_seconds": (step+1)*self.dt,
                      "peak_penetration_m": max(peaks), "median_max_penetration_m": float(np.median(peaks)),
                      "contacts_ok": bool(max(peaks) < LIMITS["peak_penetration_m"]
                          and np.median(peaks) < LIMITS["median_max_penetration_m"]),
                      "basket_supported": all(row["basket_supported"] for row in window),
                      "dish_supported": not unsupported, "unsupported_object_ids": unsupported,
                      "support_paths": paths,
                      "endpoints_ok": all(self.endpoint_ok(row["joints"], goal) for row in window),
                      "joints": joints, "rest_window_count": checks,
                      "observation_required_s": observation,
                      "observation_completed_s": 0. if first_pass is None else (step-first_pass)*self.dt,
                      "observation_started_count": observation_started_count,
                      "observation_restarts": observation_restarts,
                      "continuous_passing_window_count": 0 if first_pass is None else step-first_pass+1}
            passed = all(result[key] for key in
                         ("settled", "contacts_ok", "basket_supported", "dish_supported", "endpoints_ok"))
            self.last_hold = result
            if passed and first_pass is None:
                if (step+1)*self.dt > timeout+1e-12:
                    result["reason"] = "no qualifying rest window before settle timeout"
                    return result
                first_pass = step
                observation_started_count += 1
                result["observation_started_count"] = observation_started_count
                result["continuous_passing_window_count"] = 1
                result["qualification_simulated_seconds"] = (step+1)*self.dt
            if first_pass is not None:
                result["qualification_simulated_seconds"] = (first_pass+1)*self.dt
                if not passed:
                    failed_gates = [key for key in ("settled", "contacts_ok", "basket_supported",
                                                    "dish_supported", "endpoints_ok") if not result[key]]
                    observation_restarts.append({"failed_at_simulated_seconds": (step+1)*self.dt,
                        "qualification_simulated_seconds": (first_pass+1)*self.dt,
                        "passing_duration_before_failed_step_s": max(0., (step-first_pass-1)*self.dt),
                        "failed_gates": failed_gates,
                        "unsupported_object_ids": unsupported,
                        "motion": metrics, "peak_penetration_m": max(peaks),
                        "median_max_penetration_m": float(np.median(peaks))})
                    first_pass = None
                    result["observation_completed_s"] = 0.
                    result["continuous_passing_window_count"] = 0
                    if (step+1)*self.dt >= timeout:
                        result["reason"] = "continuous observation failed after the settle allowance; cannot restart"
                        return result
                    continue
                if step-first_pass >= round(observation/self.dt):
                    result.update(passed=True, reason="all required trailing rest windows passed")
                    return result
            elif (step+1)*self.dt >= timeout:
                result["reason"] = "no qualifying rest window before settle timeout"
                return result
        result["reason"] = "continuous observation did not complete"
        return result

    def ramp(self, goal, phase, *, check_loaded=False):
        self.phase = phase
        initial = self.target.clone()
        duration = max(3.5, *(1.5*abs(float(initial[0, self.indices[name]])-value)
                             /(.35 if name == "door_hinge" else RACK_COMMAND_SPEED_M_S)
                             for name, value in goal.items()))
        steps = math.ceil(duration/self.dt)
        result = {"passed": True, "duration_s": steps*self.dt, "goal": goal,
                  "command_profile": "cubic smoothstep", "maximum_penetration_m": 0.,
                  "unsupported_object_ids": [], "peak_measured_rack_speed_m_s": 0.,
                  "measured_speed_ok": True,
                  "commanded_rack_speed_cap_m_s": RACK_COMMAND_SPEED_M_S}
        previous = self.joints()
        for step in range(1, steps+1):
            fraction = step/steps
            blend = fraction*fraction*(3.-2.*fraction)
            for name, value in goal.items():
                self.target[0, self.indices[name]] = initial[0, self.indices[name]]*(1-blend)+value*blend
            frames, joints = self.tick()
            result["peak_measured_rack_speed_m_s"] = max(result["peak_measured_rack_speed_m_s"],
                *(abs(joints[name]-previous[name])/self.dt for name in ("upper_slide", "lower_slide")))
            previous = joints
            result["maximum_penetration_m"] = max(result["maximum_penetration_m"], self.latest_contact["peak_m"])
            if check_loaded:
                if result["peak_measured_rack_speed_m_s"] > (LIMITS["rack_speed_m_s"]
                                                            +RACK_SPEED_MEASUREMENT_TOLERANCE_M_S):
                    result.update(passed=False, measured_speed_ok=False, failed_step=step,
                                  reason="measured rack speed exceeded cap plus floating-point measurement tolerance")
                    return result
                paths = support_paths(self.latest_contact["pairs"], self.assignments)
                unsupported = [identity for identity, path in paths.items() if path is None]
                basket_supported = tuple(sorted(("SilverwareBasket", "LowerRack"))) in self.latest_contact["pairs"]
                if (unsupported or not basket_supported
                        or self.latest_contact["peak_m"] >= LIMITS["peak_penetration_m"]):
                    result.update(passed=False, failed_step=step, support_paths=paths,
                                  unsupported_object_ids=unsupported, basket_supported=basket_supported,
                                  reason="support or penetration gate failed during loaded retraction")
                    return result
        return result

    def prepare_baseline(self, snapshot=None):
        if snapshot is None:
            self.restore(self.initial)
            initial_hold = self.hold(phase="baseline_initial_settle",
                                     goal={name: 0. for name in JOINTS})
            if not initial_hold["passed"]:
                return {"result": "FAIL", "reason": "closed empty baseline did not settle", "hold": initial_hold}
            self.ramp({"door_hinge": math.pi/2}, "baseline_open_door")
            self.ramp({"upper_slide": EXTENSION["UpperRack"],
                       "lower_slide": EXTENSION["LowerRack"]}, "baseline_extend_both_racks")
        else:
            self.restore(snapshot.get("snapshot", snapshot))
        hold = self.hold(phase="baseline_extended_hold", goal=self.goal(RACKS))
        measured = self.snapshot()
        # Parked objects are implementation details, absent from the baseline.
        measured["poses"] = {name: measured["poses"][name] for name in COMPONENTS}
        measured["velocities_world"] = {"SilverwareBasket": measured["velocities_world"]["SilverwareBasket"]}
        record = {"result": "PASS" if hold["passed"] else "FAIL", "hold": hold,
                  "snapshot": measured, "joints": measured["joints"], "poses": measured["poses"],
                  "runtime": self.runtime_settings}
        if hold["passed"]:
            self.baseline = measured
        return record

    def _prepare_objects(self, frames):
        result = deepcopy(self.entries)
        for entry in result:
            local = entry.get("rack_local_pose")
            if local is not None:
                parent = frames[entry["rack"]]
                position, quaternion = compose_pose(parent["position_m"], parent["quaternion_xyzw"],
                                                     local["position_m"], local["quaternion_xyzw"])
                entry["pose_world"] = {"position_m": position.tolist(), "quaternion_xyzw": quaternion.tolist()}
        return result

    def evaluate(self, *, order=("UpperRack", "LowerRack"), baseline=None):
        if tuple(sorted(order)) != tuple(sorted(RACKS)):
            raise ValueError("Retraction order must contain both racks once")
        if not self.entries:
            raise ValueError("A loaded arrangement must contain at least one object")
        record = {"outcome": "unresolved", "order": list(order), "runtime": self.runtime_settings,
                  "initial_snapshot": None, "final_snapshot": None,
                  "candidate_objects": deepcopy(self.entries), "object_count": len(self.entries),
                  "trace_file": "trace.jsonl", "loaded_rack_motions": []}
        started = time.monotonic()
        with (self.directory/"trace.jsonl").open("w", buffering=1) as trace:
            self.trace = trace
            try:
                self._evaluate(record, order, baseline)
            except BudgetExpired as exc:
                record.update(outcome="timeout", reason=str(exc), last_hold=self.last_hold)
            except Exception as exc:
                record.update(outcome="simulation_error", reason=repr(exc), last_hold=self.last_hold)
                import traceback
                record["traceback"] = traceback.format_exc()
            finally:
                self.trace = None
                record["wall_seconds"] = time.monotonic()-started
                record["maximum_observed_penetration_m"] = (self.global_peak_event["peak_penetration_m"]
                    if self.global_peak_event is not None else self.maximum_penetration_m)
                record["maximum_observed_penetration_event"] = self.global_peak_event
                if self.loaded:
                    period = "settle" if record["initial_snapshot"] is None else "cycle"
                    record.setdefault(f"maximum_{period}_penetration_m", self.maximum_penetration_m)
                    record.setdefault(f"maximum_{period}_penetration_event", self.peak_event)
                try:
                    record["last_snapshot"] = self.snapshot()
                except Exception:
                    pass
        (self.directory/"physics.json").write_text(json.dumps(record, indent=2, allow_nan=False)+"\n")
        return record

    def _evaluate(self, record, order, baseline):
        from .initial_state_candidates import InitialCollisionChecker
        if self.baseline is None or baseline is not None:
            baseline_record = self.prepare_baseline(baseline)
            record["baseline"] = baseline_record
            if baseline_record["result"] != "PASS":
                record.update(outcome="baseline_failure", reason="Empty common baseline failed")
                return
        frames = self.poses()
        record["initialized_components"] = {name: frames[name] for name in COMPONENTS}
        prepared = self._prepare_objects(frames)
        record["proposed_objects"] = prepared
        checker = InitialCollisionChecker(self.usd_path.parent, penetration_limit_m=.001)
        geometry = checker.check_arrangement(prepared, record["initialized_components"])
        record["initial_geometry"] = geometry
        if not geometry["valid"]:
            record.update(outcome="initial_collision", reason="Actual initialized geometry failed preflight")
            return
        for entry in prepared:
            self.set_rigid_pose(self.objects[entry["object_id"]], entry["pose_world"])
        self.loaded = True
        self.tracker.clear()
        self.previous_frames = None
        self.maximum_penetration_m = 0.
        self.peak_event = None
        self.global_peak_event = None
        settled = self.hold(phase="loaded_initial_settle_and_observation", goal=self.goal(RACKS),
                            timeout=12., observation=OBSERVATION_SECONDS, abort_on_peak=True)
        record["settled"] = settled
        record["measured_loaded_snapshot"] = self.snapshot()
        record["maximum_settle_penetration_m"] = self.maximum_penetration_m
        record["maximum_settle_penetration_event"] = self.peak_event
        if self.maximum_penetration_m >= LIMITS["peak_penetration_m"]:
            record.update(outcome="penetration_failure", reason="Excessive penetration during joint initial settling")
            return
        if not settled["passed"]:
            record.update(outcome="settle_failure", reason=settled.get("reason"))
            return
        record["initial_snapshot"] = self.snapshot()
        displacement = {}
        for entry in prepared:
            initial = entry["pose_world"]
            measured = record["initial_snapshot"]["poses"][entry["object_id"]]
            quaternion_a = np.asarray(initial["quaternion_xyzw"])
            quaternion_b = np.asarray(measured["quaternion_xyzw"])
            cosine = abs(float(quaternion_a @ quaternion_b)/(np.linalg.norm(quaternion_a)*np.linalg.norm(quaternion_b)))
            displacement[entry["object_id"]] = {
                "translation_m": float(np.linalg.norm(np.asarray(measured["position_m"])-initial["position_m"])),
                "rotation_deg": float(np.degrees(2*np.arccos(np.clip(cosine, 0., 1.))))}
        record["proposal_to_initial_displacement"] = displacement
        self.maximum_penetration_m = 0.
        self.peak_event = None
        extended = set(RACKS)
        for rack in order:
            motion = self.ramp({RACK_JOINT[rack]: 0.}, "loaded_retract_"+rack, check_loaded=True)
            record["loaded_rack_motions"].append({"rack": rack, **motion})
            if not motion["passed"]:
                record.update(outcome="closure_failure", reason=motion["reason"],
                              maximum_cycle_penetration_m=self.maximum_penetration_m,
                              maximum_cycle_penetration_event=self.peak_event)
                return
            extended.remove(rack)
            hold = self.hold(phase="loaded_hold_after_"+rack, goal=self.goal(extended),
                             timeout=12., observation=OBSERVATION_SECONDS if not extended else 0.)
            record["loaded_rack_motions"][-1]["hold"] = hold
            if not hold["passed"]:
                record.update(outcome="closure_failure", reason=hold.get("reason"),
                              maximum_cycle_penetration_m=self.maximum_penetration_m,
                              maximum_cycle_penetration_event=self.peak_event)
                return
        record["final_hold"] = hold
        record["final_snapshot"] = self.snapshot()
        record["maximum_cycle_penetration_m"] = self.maximum_penetration_m
        record["maximum_cycle_penetration_event"] = self.peak_event
        interior = self.domains["interior_bounds"]
        cabinet = record["final_snapshot"]["poses"]["Cabinet"]
        containment = {}
        for entry in prepared:
            name = entry["object_id"]
            pose = record["final_snapshot"]["poses"][name]
            vertices = transform_vertices(self.points[entry["kind"]], pose["position_m"], pose["quaternion_xyzw"])
            cabinet_vertices = ((vertices-np.asarray(cabinet["position_m"]))
                                @ quaternion_matrix_xyzw(cabinet["quaternion_xyzw"]))
            in_world = bool(vertices_contained(vertices, interior["lower_m"],
                interior["upper_m"], tolerance=LIMITS["containment_tolerance_m"]))
            in_cabinet = bool(vertices_contained(cabinet_vertices, interior["lower_m"],
                interior["upper_m"], tolerance=LIMITS["containment_tolerance_m"]))
            containment[name] = {"contained": in_world and in_cabinet,
                "world_contained": in_world, "cabinet_frame_contained": in_cabinet,
                "mesh_bounds_m": [vertices.min(0).tolist(), vertices.max(0).tolist()],
                "cabinet_frame_mesh_bounds_m": [cabinet_vertices.min(0).tolist(), cabinet_vertices.max(0).tolist()]}
        record["final_containment"] = containment
        if not all(value["contained"] for value in containment.values()):
            record.update(outcome="outside_dishwasher", reason="Final whole visual mesh exceeds cabinet interior")
        elif self.maximum_penetration_m >= LIMITS["peak_penetration_m"]:
            record.update(outcome="penetration_failure", reason="Excessive penetration during loaded retraction")
        else:
            record.update(outcome="accepted", reason="All joint loaded-state and retraction checks passed")
