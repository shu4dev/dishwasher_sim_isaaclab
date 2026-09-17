"""Kit-free safety gates and full-cycle sequencing for organized states."""
from copy import deepcopy
import math
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from dishsim_frigidaire.organization import default_policy
from dishsim_frigidaire.organization_runtime import (
    IsaacOrganizedStateBackend, blocked_door_evidence, closed_goal, direct_rack_support, forbidden_dish_contacts)
from dishsim_frigidaire.initial_state_runtime import IsaacInitialStateBackend

POSE = {"position_m": [0., 0., .3], "quaternion_xyzw": [1., 0., 0., 0.]}


class ContactTests(unittest.TestCase):
    def test_unrelated_penetration_is_not_blocked_door_evidence(self):
        peak = {"peak_penetration_m": .003,
                "contact_pair_penetration_m": {"LowerRack|SilverwareBasket": .003},
                "raw_contact_pairs": [("Door", "LowerRack"), ("LowerRack", "SilverwareBasket")]}
        self.assertFalse(blocked_door_evidence({}, None, peak, {})["valid"])

    def test_targeted_contact_and_blocked_endpoint_are_negative_control_evidence(self):
        contact = {"pairs": [("Door", "LowerRack")], "depths_m": {"Door|LowerRack": .0001}}
        self.assertTrue(blocked_door_evidence({}, {"endpoints_ok": False}, None, contact)["valid"])
        contact["depths_m"]["Door|LowerRack"] = .0021
        self.assertTrue(blocked_door_evidence({}, None, None, contact)["valid"])

    def test_only_direct_assigned_rack_contact_counts(self):
        pairs = [("a", "b"), ("b", "LowerRack"), ("c", "SilverwareBasket"),
                 ("SilverwareBasket", "LowerRack"), ("d", "UpperRack")]
        self.assertEqual(direct_rack_support(pairs, dict.fromkeys("abcd", "LowerRack")),
                         {"a": False, "b": True, "c": False, "d": False})

    def test_dishes_and_door_forbidden_rack_contact_allowed(self):
        pairs = [("a", "LowerRack"), ("a", "b"), ("Door", "b"),
                 ("Door", "LowerRack"), ("a", "a")]
        self.assertEqual(forbidden_dish_contacts(pairs, {"a": "LowerRack", "b": "UpperRack"}),
                         [("Door", "b"), ("a", "b")])


class ObservationTests(unittest.TestCase):
    def backend(self, invalid_geometry_at=None, unsupported_at=None):
        backend = IsaacOrganizedStateBackend.__new__(IsaacOrganizedStateBackend)
        backend.dt, backend.hz, backend.loaded = 1/120, 120, True
        backend.objects = {"dish": object()}
        backend.entries = [{"object_id": "dish", "rack": "LowerRack", "kind": "mug"}]
        backend.assignments = {"dish": "LowerRack"}
        backend.organization_policy = default_policy()
        backend.steps, backend.tick_index = 0, 0
        backend.last_hold = backend.last_organization = None
        backend.maximum_penetration_m = 0.
        backend.geometry_steps = []

        def tick():
            backend.steps += 1
            backend.tick_index = backend.steps
            pairs = {tuple(sorted(("SilverwareBasket", "LowerRack")))}
            if backend.steps != unsupported_at:
                pairs.add(tuple(sorted(("dish", "LowerRack"))))
            backend.latest_contact = {"pairs": pairs, "peak_m": .0002}
            backend.latest_speeds = {"dish": 0., "SilverwareBasket": 0.}
            return {"dish": POSE, "SilverwareBasket": POSE}, backend.goal(("LowerRack", "UpperRack"))

        def organization(frames, support):
            backend.geometry_steps.append(backend.steps)
            value = {"valid": backend.steps != invalid_geometry_at, "violations": []}
            backend.last_organization = value
            return value

        backend.tick, backend.organization = tick, organization
        return backend

    def test_five_seconds_with_120hz_orientation_and_10hz_geometry(self):
        backend = self.backend()
        hold = backend.hold(phase="loaded_initial_settle_and_observation", goal=backend.goal(("LowerRack", "UpperRack")), observation=5.)
        self.assertTrue(hold["passed"])
        self.assertEqual(backend.steps, 721)
        self.assertEqual(hold["continuous_passing_window_count"], 601)
        self.assertEqual(hold["organization_checks"], 51)
        self.assertEqual(backend.geometry_steps[0], 121)
        self.assertEqual(backend.geometry_steps[-1], 721)
        self.assertEqual(hold["observation_completed_s"], 5.)

    def test_failed_geometry_restarts_full_observation(self):
        backend = self.backend(invalid_geometry_at=325)
        hold = backend.hold(phase="loaded_initial_settle_and_observation", goal=backend.goal(("LowerRack", "UpperRack")), observation=5.)
        self.assertTrue(hold["passed"])
        self.assertEqual(hold["observation_started_count"], 2)
        self.assertEqual(hold["observation_restarts"][0]["failed_gates"], ["organization_valid"])
        self.assertGreater(backend.steps, 721)
        self.assertEqual(hold["continuous_passing_window_count"], 601)

    def test_support_loss_during_cycle_is_immediate_failure(self):
        backend = self.backend(unsupported_at=17)
        hold = backend.hold(phase="loaded_door_closed_observation", goal=backend.goal(("LowerRack", "UpperRack")), observation=5.)
        self.assertFalse(hold["passed"])
        self.assertEqual(backend.steps, 17)
        self.assertEqual(hold["failed_event"]["step"], 17)

    def test_initial_contact_can_qualify_after_release(self):
        backend = self.backend(unsupported_at=17)
        hold = backend.hold(phase="loaded_initial_settle_and_observation", goal=backend.goal(("LowerRack", "UpperRack")), observation=5.)
        self.assertTrue(hold["passed"])
        self.assertGreater(backend.steps, 721)


class FullCycleTests(unittest.TestCase):
    def backend(self, fail_close=False):
        backend = IsaacOrganizedStateBackend.__new__(IsaacOrganizedStateBackend)
        backend.organization_policy = default_policy()
        backend.last_organization = {"valid": True}
        backend.maximum_penetration_m = .0001
        backend.peak_event = None
        backend.organization_evaluation_count = backend.organization_cache_hits = 0
        backend.motion_calls, backend.hold_calls = [], []

        def ramp(goal, phase, **kwargs):
            backend.motion_calls.append((phase, goal))
            return {"passed": not (fail_close and phase == "loaded_close_door"), "reason": "blocked"}

        def hold(**kwargs):
            backend.hold_calls.append(kwargs)
            return {"passed": True, "organization": {"valid": True}}

        backend.ramp, backend.hold = ramp, hold
        backend.snapshot = lambda: {"poses": {}, "joints": closed_goal()}
        backend.containment = lambda snapshot: {"dish": {"contained": True}}
        return backend

    @staticmethod
    def legacy_pass(backend, record, order, baseline):
        record.update(outcome="accepted", final_snapshot={"legacy": True},
                      final_hold={"passed": True}, settled={"organization": {"valid": True}})

    def test_accepted_requires_close_reopen_and_both_extensions(self):
        backend, record = self.backend(), {}
        with patch.object(IsaacInitialStateBackend, "_evaluate", self.legacy_pass):
            backend._evaluate(record, ("UpperRack", "LowerRack"), None)
        self.assertEqual(record["outcome"], "accepted")
        self.assertEqual([item[0] for item in backend.motion_calls],
                         ["loaded_close_door", "loaded_reopen_door", "loaded_extend_LowerRack", "loaded_extend_UpperRack"])
        self.assertEqual(backend.hold_calls[0]["goal"], closed_goal())
        self.assertEqual(backend.hold_calls[0]["observation"], 5.)
        self.assertEqual(backend.hold_calls[-1]["observation"], 5.)
        self.assertEqual(record["rack_closed_snapshot"], {"legacy": True})
        self.assertIn("reopened_snapshot", record)

    def test_blocked_door_never_retains_legacy_acceptance(self):
        backend, record = self.backend(fail_close=True), {}
        with patch.object(IsaacInitialStateBackend, "_evaluate", self.legacy_pass):
            backend._evaluate(record, ("UpperRack", "LowerRack"), None)
        self.assertEqual(record["outcome"], "door_closure_failure")
        self.assertEqual(len(backend.motion_calls), 1)
        self.assertNotIn("reopened_snapshot", record)

    def test_door_endpoint_threshold(self):
        goal = closed_goal()
        self.assertTrue(IsaacOrganizedStateBackend.endpoint_ok({**goal, "door_hinge": math.radians(.49)}, goal))
        self.assertFalse(IsaacOrganizedStateBackend.endpoint_ok({**goal, "door_hinge": math.radians(.51)}, goal))


if __name__ == "__main__":
    unittest.main()
