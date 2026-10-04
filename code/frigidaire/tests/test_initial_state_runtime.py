"""Kit-free protocol tests for the multi-object physics measurement seam."""
from copy import deepcopy
import json
import math
import unittest

import numpy as np

from dishsim_frigidaire.initial_state_runtime import (
    IsaacInitialStateBackend, all_vertex_step_speed, normalize_candidates,
    support_paths, window_motion_metrics, retain_peak_event,
)


POSE = {"position_m": [0., 0., .3], "quaternion_xyzw": [0., 0., 0., 1.]}


class ContactSupportTests(unittest.TestCase):
    def test_stack_reaches_assigned_rack(self):
        paths = support_paths([("a", "b"), ("b", "c"), ("c", "LowerRack")],
                              {"a": "LowerRack", "b": "LowerRack", "c": "LowerRack"})
        self.assertEqual(paths["a"], ["a", "b", "c", "LowerRack"])

    def test_floating_cluster_and_self_contact_do_not_establish_support(self):
        paths = support_paths([("a", "b"), ("a", "a")], {"a": "LowerRack", "b": "LowerRack"})
        self.assertEqual(paths, {"a": None, "b": None})

    def test_cabinet_and_wrong_rack_are_not_support_paths(self):
        pairs = [("a", "Cabinet"), ("Cabinet", "LowerRack"), ("b", "UpperRack")]
        self.assertEqual(support_paths(pairs, {"a": "LowerRack", "b": "LowerRack"}),
                         {"a": None, "b": None})

    def test_basket_must_be_directly_seated_and_only_supports_lower(self):
        pairs = [("a", "SilverwareBasket"), ("b", "SilverwareBasket")]
        assignments = {"a": "LowerRack", "b": "UpperRack"}
        self.assertEqual(support_paths(pairs, assignments), {"a": None, "b": None})
        paths = support_paths(pairs+[("SilverwareBasket", "LowerRack")], assignments)
        self.assertEqual(paths["a"], ["a", "SilverwareBasket", "LowerRack"])
        self.assertIsNone(paths["b"])


class CandidateBoundaryTests(unittest.TestCase):
    def entry(self, candidate_id="a"):
        return {"candidate_id": candidate_id, "kind": "dinner_plate", "rack": "UpperRack",
                "rack_local_pose": deepcopy(POSE)}

    def test_preserves_unusual_but_accepted_kind_rack_assignment(self):
        result = normalize_candidates([self.entry()])
        self.assertEqual(result[0]["rack"], "UpperRack")
        self.assertEqual(result[0]["object_id"], "dish_0000")

    def test_duplicate_candidate_and_object_rejected(self):
        with self.assertRaisesRegex(ValueError, "only once"):
            normalize_candidates([self.entry(), self.entry()])
        first, second = self.entry("a"), self.entry("b")
        first["object_id"] = second["object_id"] = "same"
        with self.assertRaisesRegex(ValueError, "duplicate"):
            normalize_candidates([first, second])

    def test_nonfinite_or_unnormalized_rejected(self):
        entry = self.entry()
        entry["rack_local_pose"]["quaternion_xyzw"] = [0., 0., 0., 2.]
        with self.assertRaises(ValueError):
            normalize_candidates([entry])


class MotionTests(unittest.TestCase):
    def test_speed_uses_vertex_with_largest_rotational_motion(self):
        points = np.array([[1., 0., 0.], [0., 1., 0.], [1., 1., 0.]])
        rotated = deepcopy(POSE)
        angle = .01
        rotated["quaternion_xyzw"] = [0., 0., math.sin(angle/2), math.cos(angle/2)]
        speed = all_vertex_step_speed(points, POSE, rotated, .1)
        self.assertAlmostEqual(speed, 2*math.sqrt(2)*math.sin(angle/2)/.1)

    def test_window_excludes_speed_before_its_first_pose(self):
        rows = [{"poses": {"dish": POSE}, "speeds": {"dish": 9.}},
                {"poses": {"dish": POSE}, "speeds": {"dish": .01}}]
        result = window_motion_metrics(rows, "dish", .1)
        self.assertEqual(result["peak_mesh_point_speed_m_s"], .01)
        self.assertEqual(result["sample_duration_s"], .1)


class PeakEvidenceTests(unittest.TestCase):
    def test_short_contact_spike_between_trace_frames_is_preserved(self):
        previous = None
        poses = {"dish": deepcopy(POSE)}
        trace_peaks = []
        for step in range(1, 25):
            depth = .003 if step == 7 else .00001
            contact = {"peak_m": depth, "depths_m": {"LowerRack|dish": depth},
                       "raw_pairs": {("LowerRack", "dish")}, "retained_only_pairs": set()}
            previous = retain_peak_event(previous, contact, poses, {"lower_slide": -.49},
                                         phase="loaded_initial_settle_and_observation", step=step, dt=1/120)
            if step % 12 == 0:
                trace_peaks.append(depth)
        self.assertLess(max(trace_peaks), .002)
        self.assertEqual(previous["step"], 7)
        self.assertEqual(previous["peak_penetration_m"], .003)
        self.assertEqual(previous["contact_pair_penetration_m"], {"LowerRack|dish": .003})
        poses["dish"]["position_m"][0] = 10.
        self.assertEqual(previous["poses"]["dish"]["position_m"][0], 0.)
        self.assertEqual(json.loads(json.dumps(previous))["raw_contact_pairs"], [["LowerRack", "dish"]])


class ObservationTests(unittest.TestCase):
    def backend(self, fail_at=None, support_from=1):
        backend = IsaacInitialStateBackend.__new__(IsaacInitialStateBackend)
        backend.dt = 1/120
        backend.loaded = True
        backend.objects = {"dish": object()}
        backend.assignments = {"dish": "LowerRack"}
        backend.steps = 0
        backend.last_hold = None

        def tick():
            backend.steps += 1
            pairs = {tuple(sorted(("SilverwareBasket", "LowerRack")))}
            if fail_at != backend.steps and backend.steps >= support_from:
                pairs.add(tuple(sorted(("dish", "LowerRack"))))
            backend.latest_contact = {"pairs": pairs, "peak_m": .0002}
            backend.latest_speeds = {"dish": 0., "SilverwareBasket": 0.}
            return {"dish": POSE, "SilverwareBasket": POSE}, backend.goal(("LowerRack", "UpperRack"))

        backend.tick = tick
        return backend

    def test_full_five_second_observation_is_required(self):
        backend = self.backend()
        result = backend.hold(phase="test", goal=backend.goal(("LowerRack", "UpperRack")), observation=5.)
        self.assertTrue(result["passed"])
        self.assertEqual(backend.steps, 721)
        self.assertEqual(result["rest_window_count"], 601)
        self.assertAlmostEqual(result["observation_completed_s"], 5.)
        self.assertEqual(json.loads(json.dumps(result, allow_nan=False)), result)

    def test_irreversible_initial_penetration_aborts_after_one_step(self):
        backend = self.backend()
        original_tick = backend.tick

        def penetrated_tick():
            result = original_tick()
            backend.maximum_penetration_m = .002
            backend.latest_contact["peak_m"] = .002
            return result

        backend.tick = penetrated_tick
        result = backend.hold(phase="initial", goal=backend.goal(("LowerRack", "UpperRack")),
                              observation=5., abort_on_peak=True)
        self.assertFalse(result["passed"])
        self.assertTrue(result["early_abort"])
        self.assertEqual(backend.steps, 1)
        self.assertEqual(result["rest_window_count"], 0)
        self.assertEqual(result["peak_penetration_m"], .002)

    def test_subthreshold_initial_transient_still_requires_full_observation(self):
        backend = self.backend()
        original_tick = backend.tick

        def briefly_penetrated_tick():
            result = original_tick()
            backend.maximum_penetration_m = .0015
            backend.latest_contact["peak_m"] = .0015 if backend.steps == 1 else .0002
            return result

        backend.tick = briefly_penetrated_tick
        result = backend.hold(phase="initial", goal=backend.goal(("LowerRack", "UpperRack")),
                              observation=5., abort_on_peak=True)
        self.assertTrue(result["passed"])
        self.assertFalse(result.get("early_abort", False))
        self.assertEqual(backend.steps, 721)
        self.assertEqual(result["observation_completed_s"], 5.)

    def test_failed_early_observation_requalifies_and_restarts_full_five_seconds(self):
        backend = self.backend(fail_at=321)
        result = backend.hold(phase="test", goal=backend.goal(("LowerRack", "UpperRack")), observation=5.)
        self.assertTrue(result["passed"])
        self.assertEqual(backend.steps, 1042)
        self.assertEqual(result["continuous_passing_window_count"], 601)
        self.assertEqual(result["observation_started_count"], 2)
        self.assertEqual(len(result["observation_restarts"]), 1)
        self.assertEqual(result["observation_restarts"][0]["unsupported_object_ids"], ["dish"])
        self.assertEqual(result["observation_completed_s"], 5.)

    def test_observation_failure_after_settle_allowance_cannot_restart(self):
        backend = self.backend(fail_at=1500, support_from=1200)
        result = backend.hold(phase="test", goal=backend.goal(("LowerRack", "UpperRack")), observation=5.)
        self.assertFalse(result["passed"])
        self.assertEqual(backend.steps, 1500)
        self.assertEqual(result["observation_completed_s"], 0.)
        self.assertIn("cannot restart", result["reason"])

    def test_qualification_at_twelve_seconds_still_requires_until_seventeen(self):
        backend = self.backend(support_from=1320)
        result = backend.hold(phase="test", goal=backend.goal(("LowerRack", "UpperRack")), observation=5.)
        self.assertTrue(result["passed"])
        self.assertEqual(result["qualification_simulated_seconds"], 12.)
        self.assertEqual(result["simulated_seconds"], 17.)
        self.assertEqual(backend.steps, 2040)
        self.assertEqual(result["continuous_passing_window_count"], 601)

    def test_qualification_later_than_twelve_seconds_is_not_allowed(self):
        backend = self.backend(support_from=1321)
        result = backend.hold(phase="test", goal=backend.goal(("LowerRack", "UpperRack")), observation=5.)
        self.assertFalse(result["passed"])
        self.assertEqual(backend.steps, 1440)
        self.assertEqual(result["observation_started_count"], 0)

    def test_no_support_cannot_qualify_on_motion_alone(self):
        backend = self.backend()
        tick = backend.tick

        def unsupported_tick():
            frames, joints = tick()
            backend.latest_contact["pairs"].discard(tuple(sorted(("dish", "LowerRack"))))
            return frames, joints

        backend.tick = unsupported_tick
        result = backend.hold(phase="test", goal=backend.goal(("LowerRack", "UpperRack")), timeout=2., observation=5.)
        self.assertFalse(result["passed"])
        self.assertEqual(backend.steps, 240)
        self.assertTrue(result["settled"])
        self.assertFalse(result["dish_supported"])


if __name__ == "__main__":
    unittest.main()
