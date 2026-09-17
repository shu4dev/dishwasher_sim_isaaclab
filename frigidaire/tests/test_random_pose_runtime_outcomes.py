"""Exercise real trial decisions through a small, observable physics boundary.

The backend is constructed without Kit; only observation, stepping and writes are
faked. Pose composition, support interpretation, containment and outcome logic
are the production implementation.
"""
from copy import deepcopy
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT / "src"), str(ROOT / "frigidaire/src")]
from dishsim_frigidaire.random_pose_experiment import BudgetExpired
from dishsim_frigidaire.random_pose_runtime import (COMPONENTS, ContactTracker,
                                                   IsaacPoseBackend, supported)


def pose(position, quaternion=(0., 0., 0., 1.)):
    return {"position_m": list(position), "quaternion_xyzw": list(quaternion)}


class TrialHarness:
    """Supply measured snapshots while retaining the actual _evaluate_trial."""
    def __init__(self):
        self.kind, self.rack = "mug", "LowerRack"
        self.backend = backend = IsaacPoseBackend.__new__(IsaacPoseBackend)
        self.events = []
        self.writes = []
        self.collision = False
        self.ramp_peak = self.settle_peak = self.final_peak = 0.
        self.budget_during_ramp = False
        self.support_before = self.support_after = self.rack
        self.final_joint_offset = 0.
        self.frames = {name: pose((0., 0., 0.)) for name in COMPONENTS}
        self.frames[self.rack] = pose((0., -.49, .215), (0., 0., np.sqrt(.5), np.sqrt(.5)))
        self.frames[self.kind] = pose((3., 0., .3))
        # Large displacement and a full inversion are intentional: settling is allowed.
        self.settled = pose((.12, -.4, .35), (1., 0., 0., 0.))
        self.final = pose((.12, .09, .35), (1., 0., 0., 0.))
        self.updated_components = None
        self.queried_pose = None
        backend.baselines = {self.rack: {"baseline": True}}
        backend.objects = {self.kind: object()}
        backend.points = {self.kind: np.array([[-.05, -.04, -.05], [.15, .04, .05]])}
        backend.domains = {"interior_bounds": {"lower_m": [-.277, -.2855, .152],
                                               "upper_m": [.277, .3015, .817]}}
        backend.maximum_penetration_m = 0.
        backend.active_kind = None
        backend.restore = self.restore
        backend.hold = self.hold
        backend.ramp = self.ramp
        backend.set_rigid_pose = self.write
        backend.poses = lambda: deepcopy(self.frames)
        backend.world = SimpleNamespace(update_components=self.update_components, collides=self.collides)
        self.trial = {"trial_id": "mug_LowerRack_00000", "kind": self.kind, "rack": self.rack,
                      "sampled_pose": pose((.1, .2, .1)), "settled_pose": None, "final_pose": None}

    def restore(self, snapshot):
        self.events.append("restore")
        if snapshot != {"baseline": True}:
            raise AssertionError("Wrong baseline restored")

    def update_components(self, frames):
        self.updated_components = deepcopy(frames)
        self.events.append("update_world")

    def collides(self, kind, world_pose):
        self.queried_pose = deepcopy(world_pose)
        self.events.append("collision_query")
        return self.collision

    def write(self, obj, world_pose):
        self.events.append("release")
        self.writes.append(deepcopy(world_pose))
        self.frames[self.kind] = deepcopy(world_pose)

    def hold(self, *, phase, kind=None, rack=None, goal=None, timeout=None):
        self.events.append(phase)
        joints = self.backend.goal(self.rack) if phase != "final_hold" else self.backend.goal()
        support_body = self.support_before
        if phase == "dish_settle":
            self.frames[self.kind] = deepcopy(self.settled)
            self.backend.maximum_penetration_m = max(self.backend.maximum_penetration_m, self.settle_peak)
        elif phase == "final_hold":
            self.frames[self.kind] = deepcopy(self.final)
            self.frames[self.rack] = pose((0., .008, .215))
            joints["lower_slide"] += self.final_joint_offset
            support_body = self.support_after
            self.backend.maximum_penetration_m = max(self.backend.maximum_penetration_m, self.final_peak)
        edges = {tuple(sorted((self.kind, support_body))), tuple(sorted(("SilverwareBasket", "LowerRack")))}
        return {"settled": True, "endpoints_ok": self.backend.endpoint_ok(joints, goal or {}),
                "contacts_ok": True, "basket_supported": True,
                "dish_supported": supported(edges, self.kind, self.rack), "joints": joints}

    def ramp(self, goal, phase):
        self.events.append(phase)
        if self.budget_during_ramp:
            raise BudgetExpired("Budget expired during loaded rack travel")
        self.backend.maximum_penetration_m = max(self.backend.maximum_penetration_m, self.ramp_peak)
        return {"goal": goal, "duration_s": 7.35}

    def evaluate(self):
        return self.backend._evaluate_trial(self.trial, self.kind, self.rack)


class TrialOutcomeTests(unittest.TestCase):
    def test_initial_collision_uses_measured_rotated_rack_and_never_releases(self):
        harness = TrialHarness()
        harness.collision = True
        self.assertEqual(harness.evaluate()["outcome"], "initial_collision")
        self.assertEqual(harness.writes, [])
        self.assertEqual(set(harness.updated_components), set(COMPONENTS))
        np.testing.assert_allclose(harness.queried_pose["position_m"], [-.2, -.39, .315], atol=1e-12)
        np.testing.assert_allclose(harness.queried_pose["quaternion_xyzw"],
                                   [0., 0., np.sqrt(.5), np.sqrt(.5)], atol=1e-12)
        self.assertNotIn("loaded_retraction", harness.events)

    def test_large_settle_rotation_and_displacement_are_preserved_without_later_teleports(self):
        harness = TrialHarness()
        # Initial impact penetration is not part of the later rack-motion certificate.
        harness.settle_peak = .02
        self.assertEqual(harness.evaluate()["outcome"], "accepted")
        self.assertEqual(harness.trial["settled_pose"], harness.settled)
        self.assertEqual(harness.trial["final_pose"], harness.final)
        self.assertEqual(len(harness.writes), 1)
        self.assertEqual(harness.events, ["restore", "trial_reset", "update_world", "collision_query",
                                          "release", "dish_settle", "loaded_retraction", "final_hold"])
        self.assertEqual(harness.trial["maximum_cycle_penetration_m"], 0.)

    def test_blocked_rack_endpoint_is_closure_failure(self):
        harness = TrialHarness()
        harness.final_joint_offset = -.02
        self.assertEqual(harness.evaluate()["outcome"], "closure_failure")
        self.assertFalse(harness.trial["final_hold"]["endpoints_ok"])

    def test_door_or_other_rack_cannot_support_the_selected_lower_rack_trial(self):
        for body in ("Door", "UpperRack", "Cabinet"):
            with self.subTest(body=body):
                harness = TrialHarness()
                harness.support_before = body
                self.assertEqual(harness.evaluate()["outcome"], "lost_support")
                self.assertNotIn("loaded_retraction", harness.events)

    def test_support_lost_during_retraction_is_not_accepted(self):
        harness = TrialHarness()
        harness.support_after = "Door"
        self.assertEqual(harness.evaluate()["outcome"], "lost_support")
        self.assertIsNotNone(harness.trial["final_pose"])

    def test_full_mesh_protrusion_rejects_an_origin_inside_the_tub(self):
        harness = TrialHarness()
        harness.final = pose((.20, .09, .35))
        self.assertEqual(harness.evaluate()["outcome"], "outside_dishwasher")
        self.assertLess(harness.final["position_m"][0], .277)
        self.assertGreater(harness.trial["final_mesh_bounds_m"][1][0], .277)

    def test_ramp_or_final_hold_penetration_remains_unresolved(self):
        for phase in ("ramp_peak", "final_peak"):
            with self.subTest(phase=phase):
                harness = TrialHarness()
                setattr(harness, phase, .003)
                self.assertEqual(harness.evaluate()["outcome"], "simulation_error")
                self.assertEqual(harness.trial["maximum_cycle_penetration_m"], .003)

    def test_budget_during_retraction_preserves_settled_pose_and_never_reteleports(self):
        harness = TrialHarness()
        harness.budget_during_ramp = True
        with self.assertRaises(BudgetExpired):
            harness.evaluate()
        self.assertEqual(harness.trial["settled_pose"], harness.settled)
        self.assertIsNone(harness.trial["final_pose"])
        self.assertEqual(len(harness.writes), 1)
        self.assertNotIn("final_hold", harness.events)


class ContactTrackerRegressionTests(unittest.TestCase):
    def test_symmetric_contact_rows_keep_the_largest_penetration(self):
        tracker = ContactTracker(("dish", "rack"), {"dish": .15, "rack": .5})
        poses = {name: pose((0., 0., 0.)) for name in ("dish", "rack")}
        data = (np.ones(2), np.zeros((2, 3)), np.zeros((2, 3)), np.array([-.003, -.0005]),
                np.array([[0, 1], [1, 0]]), np.array([[0, 0], [1, 0]]))
        observation = tracker.update(data, poses)
        self.assertEqual(observation["peak_m"], .003)
        self.assertEqual(observation["pairs"], {("dish", "rack")})
        empty = (np.zeros(0), np.zeros((0, 3)), np.zeros((0, 3)), np.zeros(0),
                 np.zeros((2, 2)), np.zeros((2, 2)))
        self.assertEqual(tracker.update(empty, poses)["peak_m"], .003)
        poses["dish"] = pose((.01, 0., 0.))
        self.assertEqual(tracker.update(empty, poses)["pairs"], set())


if __name__ == "__main__":
    unittest.main()
