"""Strict organization and complete appliance-cycle validation in Isaac Sim.

The legacy multi-dish validator is unchanged. This opt-in backend adds direct
rack support, semantic organization, measured door motion, and reopening.
Imports remain Kit-free until backend construction after AppLauncher.
"""
from __future__ import annotations

from collections import deque
from copy import deepcopy
import json
import math
import time

import numpy as np

from .initial_state_runtime import (IsaacInitialStateBackend, OBSERVATION_SECONDS,
    RACK_COMMAND_SPEED_M_S, RACK_SPEED_MEASUREMENT_TOLERANCE_M_S, window_motion_metrics)
from .random_pose_runtime import COMPONENTS, RACK_JOINT, EXTENSION
from .random_pose_experiment import BudgetExpired
from .random_poses import (LIMITS, RACKS, motion_is_settled, transform_vertices,
                           quaternion_matrix_xyzw, vertices_contained)

DOOR_COMMAND_SPEED_RAD_S = .35
DOOR_MEASURED_SPEED_RAD_S = .65
DOOR_SPEED_MEASUREMENT_TOLERANCE_RAD_S = 1e-5
DOOR_ENDPOINT_RAD = math.radians(.5)
ORGANIZATION_GEOMETRY_HZ = 10


def direct_rack_support(pairs, assignments):
    edges = {tuple(sorted(pair)) for pair in pairs}
    return {identity: tuple(sorted((identity, rack))) in edges
            for identity, rack in assignments.items()}


def forbidden_dish_contacts(pairs, assignments):
    """Dishes may touch their racks, but never each other or the door."""
    names = set(assignments)
    return sorted({tuple(sorted(pair)) for pair in pairs
                   if len(set(pair)) == 2 and
                   ((pair[0] in names and pair[1] in names)
                    or ("Door" in pair and any(name in names for name in pair)))})


def closed_goal():
    return {"door_hinge": 0., "lower_slide": 0., "upper_slide": 0.}


def blocked_door_evidence(motion, hold, peak_event, latest_contact):
    """Negative control must witness the intended door/rack obstruction."""
    pair = ("Door", "LowerRack")
    peak_event = peak_event or {}
    depths = [float(peak_event.get("contact_pair_penetration_m", {}).get("Door|LowerRack", 0.)),
              float(latest_contact.get("depths_m", {}).get("Door|LowerRack", 0.))]
    pairs = {tuple(sorted(value)) for value in latest_contact.get("pairs", [])}
    pairs.update(tuple(sorted(value)) for value in peak_event.get("raw_contact_pairs", []))
    pairs.update(tuple(sorted(value)) for value in motion.get("failed_event", {}).get("contact_pairs", []))
    witnessed = pair in pairs
    endpoint_blocked = bool(hold is not None and not hold.get("endpoints_ok", True))
    return {"witnessed_door_lower_rack_contact": witnessed,
            "door_lower_rack_peak_penetration_m": max(depths),
            "closed_endpoint_blocked": endpoint_blocked,
            "valid": witnessed and (max(depths) >= LIMITS["peak_penetration_m"] or endpoint_blocked)}


class IsaacOrganizedStateBackend(IsaacInitialStateBackend):
    def __init__(self, *args, organization_policy=None, **kwargs):
        super().__init__(*args, **kwargs)
        from .organization import default_policy, OrganizationGeometry
        self.organization_policy = organization_policy or default_policy()
        self.organization_geometry = OrganizationGeometry(self.usd_path.parent)
        self._organization_cache_key = None
        self._organization_cache = None
        self.organization_evaluation_count = 0
        self.organization_cache_hits = 0
        self.last_organization = None
        self.runtime_settings.update({
            "organization_policy": deepcopy(self.organization_policy),
            "support_rule": "every dish directly contacts its assigned rack; no dish or basket chains",
            "forbidden_loaded_contacts": ["dish-dish", "dish-Door"],
            "organization_checks_hz": ORGANIZATION_GEOMETRY_HZ,
            "orientation_checks_hz": self.hz,
            "organization_intersample_clearance_guarantee": False,
            "organization_cache_rule": "reuse only numerically identical object and component poses and support flags",
            "door_cycle_enabled": True,
            "commanded_door_speed_cap_rad_s": DOOR_COMMAND_SPEED_RAD_S,
            "measured_door_speed_cap_rad_s": DOOR_MEASURED_SPEED_RAD_S,
            "door_speed_measurement_tolerance_rad_s": DOOR_SPEED_MEASUREMENT_TOLERANCE_RAD_S,
            "door_endpoint_tolerance_rad": DOOR_ENDPOINT_RAD,
            "reopen_and_extend_validation": True})

    def organization(self, frames=None, support=None):
        from .organization import evaluate_organization
        frames = self.poses() if frames is None else frames
        support = direct_rack_support(self.latest_contact["pairs"], self.assignments) if support is None else support
        key = tuple((name, tuple(frames[name]["position_m"]), tuple(frames[name]["quaternion_xyzw"]))
                    for name in sorted(frames)) + tuple(sorted(support.items()))
        if key != self._organization_cache_key:
            self._organization_cache = evaluate_organization(
                self.entries, poses=frames, geometry=self.organization_geometry,
                policy=self.organization_policy, component_frames=frames, direct_support=support)
            self._organization_cache_key = key
            self.organization_evaluation_count += 1
        else:
            self.organization_cache_hits += 1
        self.last_organization = self._organization_cache
        with (self.directory/"organization_trace.jsonl").open("a", buffering=1) as stream:
            stream.write(json.dumps({"step": self.tick_index, "simulated_seconds": self.tick_index*self.dt,
                "phase": self.phase, "poses": frames, "direct_rack_support": support,
                "organization": self.last_organization}, allow_nan=False)+"\n")
        return self.last_organization

    def hold(self, *, phase, goal, timeout=12., observation=0., abort_on_peak=False):
        """Require direct support/orientation at 120 Hz, exact geometry at 10 Hz.

        Each physical trailing one-second window is checked at 120 Hz. Once a
        physical window qualifies, exact organization is checked at the current
        step and at 10 Hz plus the final snapshot. A failure restarts the full
        observation only within the original settling allowance.
        """
        from .organization import orientation_metrics
        self.phase = phase
        window = deque(maxlen=round(LIMITS["rest_window_s"]/self.dt)+1)
        names = (*self.objects, "SilverwareBasket") if self.loaded else ("SilverwareBasket",)
        assignments = self.assignments if self.loaded else {}
        first_pass, checks, organization_checks = None, 0, 0
        started_count, restarts = 0, []
        last_geometry_step, organization = None, None
        geometry_observations = []
        result = {"passed": False, "reason": "no complete qualifying rest window"}
        for step in range(math.ceil((timeout+observation)/self.dt)+1):
            frames, joints = self.tick()
            if abort_on_peak and self.loaded and self.maximum_penetration_m >= LIMITS["peak_penetration_m"]:
                result.update(settled=False, early_abort=True, abort_gate="maximum_initial_penetration",
                    reason="irreversible initial peak penetration failure",
                    peak_penetration_m=self.maximum_penetration_m,
                    simulated_seconds=(step+1)*self.dt, rest_window_count=checks,
                    observation_required_s=observation, observation_completed_s=0.,
                    continuous_passing_window_count=0, observation_started_count=started_count,
                    observation_restarts=restarts, joints=joints)
                self.last_hold = result
                return result
            support = direct_rack_support(self.latest_contact["pairs"], assignments)
            forbidden_now = forbidden_dish_contacts(self.latest_contact["pairs"], assignments)
            if (self.loaded and "initial_settle" not in phase and
                    (not all(support.values()) or forbidden_now
                     or self.latest_contact["peak_m"] >= LIMITS["peak_penetration_m"])):
                result.update(passed=False, early_abort=True,
                    reason="direct support, forbidden contact, or penetration failed during loaded cycle hold",
                    direct_rack_support=support, forbidden_contact_pairs=forbidden_now,
                    failed_event={"step": self.tick_index, "phase": phase, "poses": deepcopy(frames),
                                  "joints": deepcopy(joints), "contact_pairs": sorted(self.latest_contact["pairs"])})
                self.last_hold = result
                return result
            row = {"poses": frames, "speeds": self.latest_speeds, "joints": joints,
                   "peak_m": self.latest_contact["peak_m"], "support": support,
                   "forbidden": forbidden_now,
                   "basket_supported": tuple(sorted(("SilverwareBasket", "LowerRack"))) in self.latest_contact["pairs"]}
            row["orientation_valid"] = (not self.loaded or all(
                orientation_metrics(entry["kind"], entry["rack"], frames[entry["object_id"]],
                                    self.organization_policy)["valid"] for entry in self.entries))
            window.append(row)
            if len(window) < window.maxlen:
                continue
            checks += 1
            metrics = {name: window_motion_metrics(window, name, self.dt) for name in names}
            peaks = [value["peak_m"] for value in window]
            unsupported = [name for name in assignments if any(not value["support"][name] for value in window)]
            forbidden = sorted({pair for value in window for pair in value["forbidden"]})
            result = {"passed": False, "settled": all(motion_is_settled(v) for v in metrics.values()),
                "motion": metrics, "simulated_seconds": (step+1)*self.dt,
                "peak_penetration_m": max(peaks), "median_max_penetration_m": float(np.median(peaks)),
                "contacts_ok": bool(max(peaks) < LIMITS["peak_penetration_m"] and
                                     np.median(peaks) < LIMITS["median_max_penetration_m"]),
                "basket_supported": all(value["basket_supported"] for value in window),
                "dish_supported": not unsupported, "direct_rack_support": support,
                "support_paths": {name: [name, assignments[name]] if support[name] else None for name in assignments},
                "unsupported_object_ids": unsupported,
                "forbidden_contacts_ok": not forbidden, "forbidden_contact_pairs": forbidden,
                "orientation_valid": all(value["orientation_valid"] for value in window),
                "endpoints_ok": all(self.endpoint_ok(value["joints"], goal) for value in window),
                "joints": joints, "rest_window_count": checks,
                "observation_required_s": observation,
                "observation_completed_s": 0. if first_pass is None else (step-first_pass)*self.dt,
                "observation_started_count": started_count, "observation_restarts": restarts,
                "continuous_passing_window_count": 0 if first_pass is None else step-first_pass+1,
                "organization_checks": organization_checks, "organization_valid": not self.loaded}
            physical_keys = ("settled", "contacts_ok", "basket_supported", "dish_supported",
                             "forbidden_contacts_ok", "endpoints_ok", "orientation_valid")
            physical = all(result[key] for key in physical_keys)
            if physical and self.loaded:
                final_step = first_pass is not None and step-first_pass >= round(observation/self.dt)
                if last_geometry_step is None or step-last_geometry_step >= round(self.hz/ORGANIZATION_GEOMETRY_HZ) or final_step:
                    organization_checks += 1
                    organization = self.organization(frames, support)
                    last_geometry_step = step
                    geometry_observations.append({"step": self.tick_index,
                        "simulated_seconds": (step+1)*self.dt, "valid": bool(organization["valid"]),
                        "minimum_certified_clearance_m": organization.get("separation", {}).get("minimum_certified_clearance_m"),
                        "minimum_opening_exposure": min((value["unobstructed_fraction"] for value in
                            organization.get("opening_exposure", [])), default=None),
                        "intrusion_count": len(organization.get("nesting", {}).get("intrusions", []))})
                result.update(organization_valid=bool(organization["valid"]),
                              organization_checks=organization_checks,
                              organization_geometry_hz=ORGANIZATION_GEOMETRY_HZ,
                              organization=deepcopy(organization))
            result["organization_observations"] = geometry_observations
            passed = physical and result["organization_valid"]
            self.last_hold = result
            if passed and first_pass is None:
                if (step+1)*self.dt > timeout+1e-12:
                    result["reason"] = "no qualifying rest and organization window before settle timeout"
                    return result
                first_pass = step
                started_count += 1
                result.update(observation_started_count=started_count, continuous_passing_window_count=1)
            if first_pass is not None:
                result["qualification_simulated_seconds"] = (first_pass+1)*self.dt
                if not passed:
                    restarts.append({"failed_at_simulated_seconds": (step+1)*self.dt,
                        "qualification_simulated_seconds": (first_pass+1)*self.dt,
                        "passing_duration_before_failed_step_s": max(0., (step-first_pass-1)*self.dt),
                        "failed_gates": [key for key in (*physical_keys, "organization_valid") if not result[key]],
                        "unsupported_object_ids": unsupported,
                        "organization_violations": deepcopy((self.last_organization or {}).get("violations", []))})
                    first_pass = None
                    result.update(observation_completed_s=0., continuous_passing_window_count=0)
                    if (step+1)*self.dt >= timeout:
                        result["reason"] = "continuous observation failed after settle allowance"
                        return result
                elif step-first_pass >= round(observation/self.dt):
                    observed = [value for value in geometry_observations
                                if value["simulated_seconds"] >= (first_pass+1)*self.dt-1e-12]
                    result["continuous_observation_geometry_samples"] = len(observed)
                    result["minimum_observed_clearance_m"] = min((value["minimum_certified_clearance_m"]
                        for value in observed if value["minimum_certified_clearance_m"] is not None), default=None)
                    result["minimum_observed_opening_exposure"] = min((value["minimum_opening_exposure"]
                        for value in observed if value["minimum_opening_exposure"] is not None), default=None)
                    result.update(passed=True, reason="all physical and organization observation steps passed")
                    return result
            elif (step+1)*self.dt >= timeout:
                result["reason"] = "no qualifying rest and organization window before settle timeout"
                return result
        result["reason"] = "continuous observation did not complete"
        return result

    def ramp(self, goal, phase, *, check_loaded=False, check_empty=False):
        self.phase = phase
        initial = self.target.clone()
        duration = max(3.5, *(1.5*abs(float(initial[0, self.indices[name]])-value)/
            (DOOR_COMMAND_SPEED_RAD_S if name == "door_hinge" else RACK_COMMAND_SPEED_M_S)
            for name, value in goal.items()))
        steps = math.ceil(duration/self.dt)
        result = {"passed": True, "duration_s": steps*self.dt, "goal": goal,
            "command_profile": "cubic smoothstep", "maximum_penetration_m": 0.,
            "unsupported_object_ids": [], "peak_measured_rack_speed_m_s": 0.,
            "peak_measured_door_speed_rad_s": 0., "measured_speed_ok": True,
            "commanded_rack_speed_cap_m_s": RACK_COMMAND_SPEED_M_S,
            "commanded_door_speed_cap_rad_s": DOOR_COMMAND_SPEED_RAD_S}
        previous = self.joints()
        for step in range(1, steps+1):
            fraction = step/steps
            blend = fraction*fraction*(3.-2.*fraction)
            for name, value in goal.items():
                self.target[0, self.indices[name]] = initial[0, self.indices[name]]*(1-blend)+value*blend
            frames, joints = self.tick()
            rack_speed = max(abs(joints[name]-previous[name])/self.dt for name in ("upper_slide", "lower_slide"))
            door_speed = abs(joints["door_hinge"]-previous["door_hinge"])/self.dt
            if rack_speed > result["peak_measured_rack_speed_m_s"]:
                result["peak_measured_rack_speed_m_s"] = rack_speed
                result["rack_speed_peak_event"] = {"step": self.tick_index, "phase": phase, "joints": joints}
            if door_speed > result["peak_measured_door_speed_rad_s"]:
                result["peak_measured_door_speed_rad_s"] = door_speed
                result["door_speed_peak_event"] = {"step": self.tick_index, "phase": phase, "joints": joints}
            previous = joints
            result["maximum_penetration_m"] = max(result["maximum_penetration_m"], self.latest_contact["peak_m"])
            if check_loaded or check_empty:
                if (rack_speed > LIMITS["rack_speed_m_s"]+RACK_SPEED_MEASUREMENT_TOLERANCE_M_S
                        or door_speed > DOOR_MEASURED_SPEED_RAD_S+DOOR_SPEED_MEASUREMENT_TOLERANCE_RAD_S):
                    result.update(passed=False, measured_speed_ok=False, failed_step=step,
                                  reason="measured articulation speed exceeded cap plus numerical tolerance")
                    return result
                support = direct_rack_support(self.latest_contact["pairs"], self.assignments if check_loaded else {})
                unsupported = [name for name, value in support.items() if not value]
                forbidden = forbidden_dish_contacts(self.latest_contact["pairs"], self.assignments if check_loaded else {})
                basket_supported = tuple(sorted(("SilverwareBasket", "LowerRack"))) in self.latest_contact["pairs"]
                if unsupported or forbidden or not basket_supported or self.latest_contact["peak_m"] >= LIMITS["peak_penetration_m"]:
                    result.update(passed=False, failed_step=step,
                        direct_rack_support=support, unsupported_object_ids=unsupported,
                        forbidden_contact_pairs=forbidden, basket_supported=basket_supported,
                        failed_event={"step": self.tick_index, "phase": phase, "poses": deepcopy(frames),
                                      "joints": deepcopy(joints), "contact_pairs": sorted(self.latest_contact["pairs"])},
                        reason="direct support, forbidden contact, or penetration gate failed during motion")
                    return result
        result["final_joints"] = self.joints()
        return result

    def containment(self, snapshot):
        interior = self.domains["interior_bounds"]
        cabinet = snapshot["poses"]["Cabinet"]
        result = {}
        for entry in self.entries:
            pose = snapshot["poses"][entry["object_id"]]
            vertices = transform_vertices(self.points[entry["kind"]], pose["position_m"], pose["quaternion_xyzw"])
            local = (vertices-np.asarray(cabinet["position_m"])) @ quaternion_matrix_xyzw(cabinet["quaternion_xyzw"])
            world_ok = bool(vertices_contained(vertices, interior["lower_m"], interior["upper_m"], tolerance=LIMITS["containment_tolerance_m"]))
            cabinet_ok = bool(vertices_contained(local, interior["lower_m"], interior["upper_m"], tolerance=LIMITS["containment_tolerance_m"]))
            result[entry["object_id"]] = {"contained": world_ok and cabinet_ok,
                "world_contained": world_ok, "cabinet_frame_contained": cabinet_ok,
                "mesh_bounds_m": [vertices.min(0).tolist(), vertices.max(0).tolist()],
                "cabinet_frame_mesh_bounds_m": [local.min(0).tolist(), local.max(0).tolist()]}
        return result

    def _evaluate(self, record, order, baseline):
        super()._evaluate(record, order, baseline)
        record["organization_policy"] = deepcopy(self.organization_policy)
        record["organization_trace_file"] = "organization_trace.jsonl"
        if record.get("settled", {}).get("organization") is not None:
            record["organization_initial"] = record["settled"]["organization"]
        if record["outcome"] != "accepted":
            if record["outcome"] == "settle_failure" and self.last_organization and not self.last_organization["valid"]:
                record.update(outcome="organization_failure", reason="settled arrangement failed organization")
            return
        record["outcome"] = "unresolved"
        record["rack_closed_snapshot"] = record["final_snapshot"]
        record["rack_closed_hold"] = record["final_hold"]
        record["door_close_motion"] = self.ramp({"door_hinge": 0.}, "loaded_close_door", check_loaded=True)
        if not record["door_close_motion"]["passed"]:
            record.update(outcome="door_closure_failure", reason=record["door_close_motion"]["reason"])
            return
        hold = self.hold(phase="loaded_door_closed_observation", goal=closed_goal(), observation=OBSERVATION_SECONDS)
        record["door_closed_hold"] = record["final_hold"] = hold
        record["organization_closed"] = deepcopy(hold.get("organization", self.last_organization))
        record["final_snapshot"] = self.snapshot()
        record["final_containment"] = self.containment(record["final_snapshot"])
        if not hold["passed"]:
            record.update(outcome="door_closure_failure", reason=hold["reason"])
            return
        if not all(value["contained"] for value in record["final_containment"].values()):
            record.update(outcome="outside_dishwasher", reason="door-closed whole-mesh containment failed")
            return
        record["door_open_motion"] = self.ramp({"door_hinge": math.pi/2}, "loaded_reopen_door", check_loaded=True)
        if not record["door_open_motion"]["passed"]:
            record.update(outcome="reopen_failure", reason=record["door_open_motion"]["reason"])
            return
        door_open_hold = self.hold(phase="loaded_door_open_hold", goal=self.goal())
        record["door_open_hold"] = door_open_hold
        if not door_open_hold["passed"]:
            record.update(outcome="reopen_failure", reason=door_open_hold["reason"])
            return
        extended = set()
        record["loaded_rack_extension_motions"] = []
        for rack in reversed(order):
            motion = self.ramp({RACK_JOINT[rack]: EXTENSION[rack]}, "loaded_extend_"+rack, check_loaded=True)
            record["loaded_rack_extension_motions"].append({"rack": rack, **motion})
            if not motion["passed"]:
                record.update(outcome="reopen_failure", reason=motion["reason"])
                return
            extended.add(rack)
            hold = self.hold(phase="loaded_reopened_hold_after_"+rack, goal=self.goal(extended),
                             observation=OBSERVATION_SECONDS if len(extended) == 2 else 0.)
            record["loaded_rack_extension_motions"][-1]["hold"] = hold
            if not hold["passed"]:
                record.update(outcome="reopen_failure", reason=hold["reason"])
                return
        record["reopened_hold"] = hold
        record["organization_reopened"] = deepcopy(hold.get("organization", self.last_organization))
        record["reopened_snapshot"] = self.snapshot()
        record["maximum_cycle_penetration_m"] = self.maximum_penetration_m
        record["maximum_cycle_penetration_event"] = self.peak_event
        record["organization_evaluation_count"] = self.organization_evaluation_count
        record["organization_exact_pose_cache_hits"] = self.organization_cache_hits
        if self.maximum_penetration_m >= LIMITS["peak_penetration_m"]:
            record.update(outcome="penetration_failure", reason="full appliance cycle exceeded penetration limit")
        else:
            record.update(outcome="accepted", reason="organization, loaded rack closure, door closure, and reopening passed")

    def control(self, kind):
        if kind not in ("empty_cycle", "blocked_door") or self.entries:
            raise ValueError("Control requires empty candidates and a known control kind")
        record = {"outcome": "control_failed", "control": kind, "runtime": self.runtime_settings,
                  "motions": [], "trace_file": "trace.jsonl"}
        with (self.directory/"trace.jsonl").open("w", buffering=1) as trace:
            self.trace = trace
            try:
                baseline = self.prepare_baseline()
                record["baseline"] = baseline
                if baseline["result"] != "PASS":
                    record["reason"] = "fresh empty extended baseline failed"
                    return record
                self.maximum_penetration_m = 0.
                self.peak_event = self.global_peak_event = None
                racks = ("UpperRack",) if kind == "blocked_door" else ("UpperRack", "LowerRack")
                extended = set(RACKS)
                for rack in racks:
                    motion = self.ramp({RACK_JOINT[rack]: 0.}, "control_retract_"+rack, check_empty=True)
                    record["motions"].append({"rack": rack, **motion})
                    if not motion["passed"]:
                        record["reason"] = motion["reason"]
                        return record
                    extended.remove(rack)
                    hold = self.hold(phase="control_hold_after_"+rack, goal=self.goal(extended))
                    record["motions"][-1]["hold"] = hold
                    if not hold["passed"]:
                        record["reason"] = hold["reason"]
                        return record
                # Negative control deliberately bypasses the normal closed-rack
                # precondition to test actual articulated obstruction response.
                motion = self.ramp({"door_hinge": 0.}, "control_close_door", check_empty=True)
                record["door_close_motion"] = motion
                target = self.goal(extended)
                target["door_hinge"] = 0.
                hold = self.hold(phase="control_closed_hold", goal=target,
                                 observation=OBSERVATION_SECONDS) if motion["passed"] else None
                record["door_closed_hold"] = hold
                record["closed_snapshot"] = self.snapshot()
                closed = bool(motion["passed"] and hold and hold["passed"])
                if kind == "blocked_door":
                    record["obstruction_event"] = deepcopy(motion.get("failed_event", self.peak_event))
                    record["expected_obstruction_detected"] = not closed
                    evidence = blocked_door_evidence(motion, hold, self.peak_event, self.latest_contact)
                    record["physical_obstruction_evidence"] = evidence
                    record.update(outcome="control_passed" if not closed and evidence["valid"] else "control_failed",
                                  reason="extended lower rack must physically prevent validated door closure")
                    return record
                if not closed:
                    record["reason"] = "empty appliance could not close and hold"
                    return record
                motion = self.ramp({"door_hinge": math.pi/2}, "control_reopen_door", check_empty=True)
                record["door_open_motion"] = motion
                if not motion["passed"]:
                    record["reason"] = motion["reason"]
                    return record
                hold = self.hold(phase="control_reopened_door_hold", goal=self.goal())
                record["door_open_hold"] = hold
                if not hold["passed"]:
                    record["reason"] = hold["reason"]
                    return record
                extended = set()
                for rack in ("LowerRack", "UpperRack"):
                    motion = self.ramp({RACK_JOINT[rack]: EXTENSION[rack]}, "control_extend_"+rack, check_empty=True)
                    record["motions"].append({"rack": rack, "direction": "extend", **motion})
                    if not motion["passed"]:
                        record["reason"] = motion["reason"]
                        return record
                    extended.add(rack)
                    hold = self.hold(phase="control_extended_hold_after_"+rack, goal=self.goal(extended),
                                     observation=OBSERVATION_SECONDS if len(extended) == 2 else 0.)
                    record["motions"][-1]["hold"] = hold
                    if not hold["passed"]:
                        record["reason"] = hold["reason"]
                        return record
                record["reopened_snapshot"] = self.snapshot()
                record.update(outcome="control_passed", reason="fresh empty rack and door cycle passed")
                return record
            except BudgetExpired as exc:
                record.update(outcome="timeout", reason=str(exc))
                return record
            finally:
                self.trace = None
                record["maximum_observed_penetration_m"] = self.maximum_penetration_m
                record["maximum_observed_penetration_event"] = self.global_peak_event
                (self.directory/"physics.json").write_text(json.dumps(record, indent=2, allow_nan=False)+"\n")
