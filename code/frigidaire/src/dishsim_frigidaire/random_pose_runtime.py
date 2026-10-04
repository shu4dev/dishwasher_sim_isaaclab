"""Isaac backend for independent random dish drops and measured rack retraction.

Kit imports occur only when constructing the backend, after AppLauncher. No
teleports occur between releasing a candidate and measuring its final outcome.
"""
from __future__ import annotations

from collections import deque
import json
import math
from pathlib import Path
import time

import numpy as np

from dishsim.quats import wxyz_to_xyzw, xyzw_to_wxyz
from .load_validation import pose_motion_metrics
from .random_pose_experiment import check_deadline
from .random_poses import (KINDS, LIMITS, compose_pose, motion_is_settled,
                           transform_vertices, vertices_contained)


COMPONENTS = ("Cabinet", "Door", "LowerRack", "UpperRack", "SilverwareBasket")
JOINTS = ("door_hinge", "lower_slide", "upper_slide")
RACK_JOINT = {"LowerRack": "lower_slide", "UpperRack": "upper_slide"}
EXTENSION = {"LowerRack": -.49, "UpperRack": -.44}
CONTACT_CAPACITY = 262144
CONTACT_PERSISTENCE_M = 1e-6


def surface_motion_bound(reference, current, radius):
    before = np.asarray(reference["quaternion_xyzw"], dtype=float)
    after = np.asarray(current["quaternion_xyzw"], dtype=float)
    before, after = before/np.linalg.norm(before), after/np.linalg.norm(after)
    chord = min(np.linalg.norm(after-before), np.linalg.norm(after+before))
    displacement = np.asarray(current["position_m"])-reference["position_m"]
    return float(np.linalg.norm(displacement)+2*radius*chord)


class ContactTracker:
    """Retain witnessed sleeping contacts only while both surfaces are unchanged."""
    def __init__(self, names, radii):
        self.names, self.radii = tuple(names), radii
        self.retained = {}

    def clear(self):
        self.retained.clear()

    def update(self, data, poses):
        if data is None or len(data) != 6:
            raise RuntimeError("Detailed PhysX contact data unavailable")

        def array(value):
            return value.detach().cpu().numpy() if hasattr(value, "detach") else np.asarray(value)

        buffers = [array(value) for value in data[:4]]
        separations = buffers[3].reshape(-1)
        counts, starts = array(data[4]), array(data[5])
        size = len(self.names)
        if (counts.size != size*size or counts.shape != starts.shape
                or not np.isfinite(counts).all() or not np.isfinite(starts).all()
                or (counts < 0).any() or (starts < 0).any()
                or not np.equal(counts, np.floor(counts)).all()
                or not np.equal(starts, np.floor(starts)).all()):
            raise RuntimeError("Invalid PhysX contact counts or start indices")
        counts = counts.astype(np.int64).reshape(size, size)
        starts = starts.astype(np.int64).reshape(size, size)
        active = counts > 0
        total = int(counts.sum())
        if total >= CONTACT_CAPACITY or any(np.any((starts+counts)[active] > len(b)) for b in buffers):
            raise RuntimeError("PhysX contact buffer overflow")
        for pair, evidence in list(self.retained.items()):
            if any(surface_motion_bound(old, poses[name], self.radii[name]) > CONTACT_PERSISTENCE_M
                   for name, old in zip(pair, evidence["poses"])):
                del self.retained[pair]
        observed = {}
        for first, second in np.argwhere(active):
            begin, count = starts[first, second], counts[first, second]
            if any(not np.isfinite(buffer[begin:begin+count]).all() for buffer in buffers):
                raise RuntimeError("Nonfinite active PhysX contact data")
            if first == second:
                continue
            pair = tuple(sorted((self.names[first], self.names[second])))
            depth = max(0., -float(separations[begin:begin+count].min()))
            observed[pair] = max(observed.get(pair, 0.), depth)
        for pair, depth in observed.items():
            self.retained[pair] = {"poses": [poses[name] for name in pair], "depth_m": depth}
        return {"pairs": set(self.retained), "count": total,
                "peak_m": max((item["depth_m"] for item in self.retained.values()), default=0.)}


def supported(pairs, kind, rack):
    edge = lambda a, b: tuple(sorted((a, b))) in pairs
    return edge(kind, rack) or (rack == "LowerRack" and edge(kind, "SilverwareBasket")
                               and edge("SilverwareBasket", "LowerRack"))


class IsaacPoseBackend:
    def __init__(self, usd_path, output_dir, *, device, domains, deadline, app):
        import carb.settings
        import omni.usd
        import torch
        from pxr import PhysxSchema, UsdPhysics
        import isaaclab.sim as sim_utils
        from isaaclab.assets import RigidObject, RigidObjectCfg
        from isaaclab.sim import SimulationContext
        from isaacsim.core.api.sensors import RigidContactView
        from .asset import spawn, apply_mode
        from .loading import visual_points
        from .random_pose_assets import LiveCollisionWorld

        self.torch, self.deadline, self.app = torch, deadline, app
        self.directory = Path(output_dir)
        self.domains, self.baselines = domains, {}
        self.dt = LIMITS["physics_dt_s"]
        self.hz = round(1/self.dt)
        self.trace = None
        self.tick_index = 0
        self.phase = "initialization"
        self.active_kind = None
        self.latest_contact = {"pairs": set(), "peak_m": 0., "count": 0}
        self.maximum_penetration_m = 0.
        self.world = LiveCollisionWorld(Path(usd_path).parent)
        self.points = self.world.points
        check_deadline(deadline)
        self.sim = SimulationContext(sim_utils.SimulationCfg(
            dt=self.dt, device=device, use_fabric=True,
            physx=sim_utils.PhysxCfg(enable_ccd=True)))
        ground = sim_utils.CuboidCfg(size=(30., 30., .05),
                                    collision_props=sim_utils.CollisionPropertiesCfg())
        ground.func("/World/Ground", ground, translation=(0., 0., -.025))
        self.prim = "/World/RandomPoseDishwasher"
        self.dishwasher, self.basket = spawn(self.prim, mode="scripted", usd_path=str(usd_path))
        self.objects = {}
        for index, kind in enumerate(KINDS):
            self.objects[kind] = RigidObject(RigidObjectCfg(
                prim_path="/World/RandomPoseDishes/"+kind,
                spawn=sim_utils.UsdFileCfg(usd_path=str(Path(usd_path).parent/"tableware"/(kind+".usdc"))),
                init_state=RigidObjectCfg.InitialStateCfg(pos=(3.+index, 0., .3))))
        stage = omni.usd.get_context().get_stage()
        for kind, obj in self.objects.items():
            body = stage.GetPrimAtPath(obj.cfg.prim_path)
            if not PhysxSchema.PhysxRigidBodyAPI(body).GetEnableCCDAttr().Get():
                raise ValueError("Tableware CCD is disabled: "+kind)
        scenes = [prim for prim in stage.Traverse() if prim.IsA(UsdPhysics.Scene)]
        if len(scenes) != 1 or not PhysxSchema.PhysxSceneAPI(scenes[0]).GetEnableCCDAttr().Get():
            raise RuntimeError("Expected one physics scene with continuous collision detection")
        carb.settings.get_settings().set_bool("/physics/disableContactProcessing", False)
        paths = [self.prim+"/"+name for name in COMPONENTS]
        paths += [self.objects[kind].cfg.prim_path for kind in KINDS]
        self.contacts = RigidContactView(
            prim_paths_expr=paths, filter_paths_expr=[paths.copy() for _ in paths],
            name="random_pose_contacts", max_contact_count=CONTACT_CAPACITY,
            disable_stablization=False)
        self.sim.reset()
        apply_mode(self.dishwasher, "scripted")
        self.contacts.initialize()
        if set(self.dishwasher.joint_names) != set(JOINTS):
            raise RuntimeError("Appliance joint names do not match the experiment")
        self.indices = {name: self.dishwasher.joint_names.index(name) for name in JOINTS}
        self.body_indices = {name: self.dishwasher.body_names.index(name) for name in COMPONENTS[:-1]}
        self.target = self.dishwasher.data.default_joint_pos.clone()
        geometry = dict(self.points)
        filenames = {"Cabinet": "cabinet", "Door": "door", "LowerRack": "lower_rack",
                     "UpperRack": "upper_rack", "SilverwareBasket": "silverware_basket"}
        for name, filename in filenames.items():
            geometry[name] = visual_points(Path(usd_path).parent/(filename+".usdc"))
        self.radii = {name: float(np.linalg.norm(points, axis=1).max()) for name, points in geometry.items()}
        self.motion_points = {}
        for name in (*KINDS, "SilverwareBasket"):
            points = geometry[name]
            extrema = [i for axis in range(3) for i in (np.argmin(points[:, axis]), np.argmax(points[:, axis]))]
            self.motion_points[name] = points[extrema]
        self.tracker = ContactTracker((*COMPONENTS, *KINDS), self.radii)
        self.initial = self.snapshot()
        self.runtime_settings = {
            "physics_hz": self.hz, "device": device, "scene_ccd": True, "tableware_ccd": True,
            "contact_capacity": CONTACT_CAPACITY, "contact_persistence_m": CONTACT_PERSISTENCE_M,
            "trace_hz": 10,
            "drive_settings": {name: {
                "stiffness": float(self.dishwasher.data.joint_stiffness[0, index]),
                "damping": float(self.dishwasher.data.joint_damping[0, index])}
                for name, index in self.indices.items()}}

    @staticmethod
    def _pose(position, quaternion_wxyz):
        position = position.detach().cpu().numpy().copy()
        quaternion = wxyz_to_xyzw(quaternion_wxyz.detach().cpu().numpy().copy())
        if not np.isfinite(position).all() or not np.isfinite(quaternion).all():
            raise RuntimeError("Nonfinite rigid body pose")
        return {"position_m": position.tolist(), "quaternion_xyzw": quaternion.tolist()}

    def poses(self):
        result = {name: self._pose(self.dishwasher.data.body_pos_w[0, index],
                                   self.dishwasher.data.body_quat_w[0, index])
                  for name, index in self.body_indices.items()}
        result["SilverwareBasket"] = self._pose(self.basket.data.root_pos_w[0], self.basket.data.root_quat_w[0])
        for kind, obj in self.objects.items():
            result[kind] = self._pose(obj.data.root_pos_w[0], obj.data.root_quat_w[0])
        return result

    def joints(self):
        values = self.dishwasher.data.joint_pos[0].detach().cpu().numpy()
        if not np.isfinite(values).all():
            raise RuntimeError("Nonfinite joint positions")
        return {name: float(values[index]) for name, index in self.indices.items()}

    def snapshot(self):
        return {"joints": self.joints(), "poses": self.poses()}

    def set_rigid_pose(self, obj, pose):
        values = [*pose["position_m"], *xyzw_to_wxyz(pose["quaternion_xyzw"])]
        tensor = self.torch.tensor([values], dtype=self.torch.float32, device=self.sim.device)
        obj.write_root_pose_to_sim(tensor)
        obj.write_root_velocity_to_sim(self.torch.zeros((1, 6), device=self.sim.device))
        obj.reset()

    def restore(self, snapshot):
        """Only trial initialization may write body or joint state directly."""
        self.active_kind = None
        for index, kind in enumerate(KINDS):
            self.set_rigid_pose(self.objects[kind], {"position_m": [3.+index, 0., .3],
                                                    "quaternion_xyzw": [0., 0., 0., 1.]})
        for name, value in snapshot["joints"].items():
            self.target[0, self.indices[name]] = value
        self.dishwasher.write_joint_state_to_sim(self.target, self.torch.zeros_like(self.target))
        self.dishwasher.write_root_velocity_to_sim(self.torch.zeros((1, 6), device=self.sim.device))
        self.dishwasher.reset()
        self.dishwasher.set_joint_position_target(self.target)
        self.set_rigid_pose(self.basket, snapshot["poses"]["SilverwareBasket"])
        self.tracker.clear()
        self.maximum_penetration_m = 0.

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
        frames = self.poses()
        joints = self.joints()
        # The tracker consumes these arrays immediately and retains no buffer
        # references. Avoid copying full-capacity contact buffers every step.
        self.latest_contact = self.tracker.update(self.contacts.get_contact_force_data(dt=self.dt, clone=False), frames)
        self.maximum_penetration_m = max(self.maximum_penetration_m, self.latest_contact["peak_m"])
        self.tick_index += 1
        if self.trace is not None and self.tick_index % max(1, self.hz//10) == 0:
            row = {"step": self.tick_index, "phase": self.phase, "joints": joints,
                   "dish": frames.get(self.active_kind), "basket": frames["SilverwareBasket"],
                   "rack_frames": {rack: frames[rack] for rack in RACK_JOINT},
                   "peak_penetration_m": self.latest_contact["peak_m"],
                   "contact_pairs": sorted(self.latest_contact["pairs"])}
            self.trace.write(json.dumps(row, allow_nan=False)+"\n")
        return frames, joints

    def ramp(self, goal, phase):
        self.phase = phase
        initial = self.target.clone()
        duration = max(3.5, *(1.5*abs(float(initial[0, self.indices[name]])-value)
                              /(.35 if name == "door_hinge" else LIMITS["rack_speed_m_s"])
                              for name, value in goal.items()))
        steps = math.ceil(duration/self.dt)
        for step in range(1, steps+1):
            fraction = step/steps
            blend = fraction*fraction*(3.-2.*fraction)
            for name, value in goal.items():
                self.target[0, self.indices[name]] = initial[0, self.indices[name]]*(1-blend)+value*blend
            self.tick()
        return {"duration_s": steps*self.dt, "goal": goal}

    def endpoint_ok(self, values, goal):
        return all(abs(values[name]-value) <= (math.radians(.5) if name == "door_hinge"
                                               else LIMITS["rack_endpoint_m"])
                   for name, value in goal.items())

    def goal(self, rack=None):
        goal = {"door_hinge": math.pi/2, "lower_slide": 0., "upper_slide": 0.}
        if rack is not None:
            goal[RACK_JOINT[rack]] = EXTENSION[rack]
        return goal

    def hold(self, *, kind=None, rack=None, phase, goal=None, timeout=None):
        """Wait for an observed one-second rest window, not commanded-pose fidelity."""
        self.phase = phase
        timeout = LIMITS["settle_timeout_s"] if timeout is None else timeout
        window = deque(maxlen=round(LIMITS["rest_window_s"]/self.dt)+1)
        names = ("SilverwareBasket",) if kind is None else ("SilverwareBasket", kind)
        result = {"settled": False}
        for step in range(math.ceil(timeout/self.dt)):
            frames, joints = self.tick()
            window.append((frames, joints, self.latest_contact))
            if len(window) < window.maxlen or step % 12 != 0:
                continue
            metrics = {name: pose_motion_metrics(
                [row[0][name]["position_m"] for row in window],
                [xyzw_to_wxyz(row[0][name]["quaternion_xyzw"]) for row in window],
                self.motion_points[name], self.dt) for name in names}
            peaks = [row[2]["peak_m"] for row in window]
            result = {
                "settled": all(motion_is_settled(value) for value in metrics.values()),
                "motion": metrics, "simulated_seconds": (step+1)*self.dt,
                "peak_penetration_m": max(peaks), "median_max_penetration_m": float(np.median(peaks)),
                "contacts_ok": bool(max(peaks) < LIMITS["peak_penetration_m"]
                                    and np.median(peaks) < LIMITS["median_max_penetration_m"]),
                "basket_supported": all(tuple(sorted(("SilverwareBasket", "LowerRack")))
                                        in row[2]["pairs"] for row in window),
                "dish_supported": None if kind is None else all(
                    supported(row[2]["pairs"], kind, rack) for row in window),
                "endpoints_ok": goal is None or all(self.endpoint_ok(row[1], goal) for row in window),
                "joints": joints, "poses": frames,
            }
            if (result["settled"] and result["endpoints_ok"] and result["contacts_ok"]
                    and result["basket_supported"] and (kind is None or result["dish_supported"])):
                return result
        return result

    def prepare_baseline(self, rack):
        self.restore(self.initial)
        self.hold(phase="baseline_initial_settle")
        self.ramp({"door_hinge": math.pi/2}, "baseline_open_door")
        self.ramp({RACK_JOINT[rack]: EXTENSION[rack]}, "baseline_extend_"+rack)
        extended = self.hold(phase="baseline_extended_hold", goal=self.goal(rack))
        snapshot = self.snapshot()
        self.maximum_penetration_m = 0.
        self.ramp({RACK_JOINT[rack]: 0.}, "baseline_retract_"+rack)
        closed = self.hold(phase="baseline_retracted_hold", goal=self.goal())
        checks = ("settled", "endpoints_ok", "contacts_ok", "basket_supported")
        passed = (all(hold.get(key, False) for hold in (extended, closed) for key in checks)
                  and self.maximum_penetration_m < LIMITS["peak_penetration_m"])
        record = {"result": "PASS" if passed else "FAIL", "extended": extended, "retracted": closed,
                  "maximum_cycle_penetration_m": self.maximum_penetration_m}
        if passed:
            self.baselines[rack] = snapshot
        return record

    def evaluate_trial(self, trial):
        kind, rack = trial["kind"], trial["rack"]
        trace_dir = self.directory/"traces"
        trace_dir.mkdir(exist_ok=True)
        trace_path = trace_dir/(trial["trial_id"]+".jsonl")
        trial["trace_file"] = str(trace_path.relative_to(self.directory))
        with trace_path.open("x", buffering=1) as trace:
            self.trace = trace
            try:
                return self._evaluate_trial(trial, kind, rack)
            finally:
                self.trace = None

    def _evaluate_trial(self, trial, kind, rack):
        self.restore(self.baselines[rack])
        reset = self.hold(phase="trial_reset", goal=self.goal(rack), timeout=3.)
        if not all(reset.get(key, False) for key in ("settled", "endpoints_ok", "contacts_ok", "basket_supported")):
            raise RuntimeError("Empty extended baseline could not be reproduced")
        trial["reset_joints"] = reset["joints"]
        frames = self.poses()
        trial["initial_rack_pose"] = frames[rack]
        local = trial["sampled_pose"]
        position, quaternion = compose_pose(frames[rack]["position_m"], frames[rack]["quaternion_xyzw"],
                                            local["position_m"], local["quaternion_xyzw"])
        world_pose = {"position_m": position.tolist(), "quaternion_xyzw": quaternion.tolist()}
        trial["released_pose"] = world_pose
        self.world.update_components({name: frames[name] for name in COMPONENTS})
        if self.world.collides(kind, world_pose):
            return {"outcome": "initial_collision"}
        self.active_kind = kind
        self.set_rigid_pose(self.objects[kind], world_pose)
        self.maximum_penetration_m = 0.
        started = time.monotonic()
        settled = self.hold(kind=kind, rack=rack, phase="dish_settle", goal=self.goal(rack))
        trial["settled_pose"] = self.poses()[kind]
        trial["settle"] = settled
        trial["settle_wall_seconds"] = time.monotonic()-started
        trial["maximum_settle_penetration_m"] = self.maximum_penetration_m
        if not settled.get("settled"):
            return {"outcome": "settle_timeout", "reason": "No rest window before closure"}
        if not settled["endpoints_ok"]:
            return {"outcome": "closure_failure", "reason": "Dish displaced the open rack or door"}
        if not settled["dish_supported"]:
            return {"outcome": "lost_support", "reason": "Dish did not come to rest on its selected rack"}
        if not settled["contacts_ok"] or not settled["basket_supported"]:
            return {"outcome": "simulation_error", "reason": "Invalid settled contacts or basket support"}
        self.maximum_penetration_m = 0.
        trial["rack_motion"] = self.ramp({RACK_JOINT[rack]: 0.}, "loaded_retraction")
        closure_peak = self.maximum_penetration_m
        final = self.hold(kind=kind, rack=rack, phase="final_hold", goal=self.goal())
        trial["final_pose"] = self.poses()[kind]
        trial["final_rack_pose"] = self.poses()[rack]
        trial["final_hold"] = final
        trial["maximum_cycle_penetration_m"] = max(closure_peak, self.maximum_penetration_m)
        if not final.get("endpoints_ok"):
            return {"outcome": "closure_failure", "reason": "Rack did not remain fully retracted"}
        if not final.get("settled"):
            return {"outcome": "settle_timeout", "reason": "No post-closure rest window"}
        if not final["dish_supported"]:
            return {"outcome": "lost_support", "reason": "Dish no longer supported by its selected rack"}
        pose = trial["final_pose"]
        vertices = transform_vertices(self.points[kind], pose["position_m"], pose["quaternion_xyzw"])
        interior = self.domains["interior_bounds"]
        contained = vertices_contained(vertices, interior["lower_m"], interior["upper_m"],
                                       tolerance=LIMITS["containment_tolerance_m"])
        trial["final_mesh_bounds_m"] = [vertices.min(0).tolist(), vertices.max(0).tolist()]
        if not contained:
            return {"outcome": "outside_dishwasher"}
        if not final["contacts_ok"] or not final["basket_supported"]:
            return {"outcome": "simulation_error", "reason": "Invalid final contacts or basket support"}
        # Contacts throughout the actual rack motion are observed. Large transient
        # interpenetration cannot be excused by an apparently valid endpoint.
        if trial["maximum_cycle_penetration_m"] >= LIMITS["peak_penetration_m"]:
            return {"outcome": "simulation_error", "reason": "Excessive penetration during rack cycle"}
        return {"outcome": "accepted"}
