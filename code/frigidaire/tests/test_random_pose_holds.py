"""Real rest-window logic with recorded observations, without starting physics."""
import json
import unittest

import numpy as np

from dishsim_frigidaire.random_pose_runtime import IsaacPoseBackend


class HoldTests(unittest.TestCase):
    def backend(self, support_after=0):
        backend = IsaacPoseBackend.__new__(IsaacPoseBackend)
        backend.dt = 1/120
        backend.motion_points = {name: np.array([[-.05, -.04, -.03], [.05, .04, .03]])
                                 for name in ("mug", "SilverwareBasket")}
        backend.steps = 0
        frames = {name: {"position_m": [0., 0., .3], "quaternion_xyzw": [0., 0., 0., 1.]}
                  for name in ("mug", "SilverwareBasket")}

        def tick():
            backend.steps += 1
            pairs = {tuple(sorted(("SilverwareBasket", "LowerRack")))}
            if backend.steps >= support_after:
                pairs.add(tuple(sorted(("mug", "LowerRack"))))
            backend.latest_contact = {"pairs": pairs, "peak_m": .0002, "count": 2}
            return frames, backend.goal()

        backend.tick = tick
        return backend

    def test_measured_hold_is_json_serializable(self):
        backend = self.backend()
        record = backend.hold(kind="mug", rack="LowerRack", phase="final", goal=backend.goal())
        self.assertTrue(record["settled"])
        self.assertTrue(record["dish_supported"])
        self.assertEqual(json.loads(json.dumps(record, allow_nan=False)), record)
        self.assertAlmostEqual(record["motion"]["mug"]["sample_duration_s"], 1.)

    def test_support_must_persist_through_full_rest_window(self):
        backend = self.backend(support_after=180)
        record = backend.hold(kind="mug", rack="LowerRack", phase="final", goal=backend.goal())
        self.assertTrue(record["dish_supported"])
        self.assertGreaterEqual(backend.steps, 300)
        self.assertGreater(record["simulated_seconds"], 2.)

    def test_motion_alone_cannot_end_an_unsupported_hold(self):
        backend = self.backend(support_after=10000)
        record = backend.hold(kind="mug", rack="LowerRack", phase="final", goal=backend.goal(), timeout=2.)
        self.assertEqual(backend.steps, 240)
        self.assertTrue(record["settled"])
        self.assertFalse(record["dish_supported"])


if __name__ == "__main__":
    unittest.main()
