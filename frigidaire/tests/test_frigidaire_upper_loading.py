"""Upper placement gaps follow the authored tine grid without requiring USD/FCL."""
import unittest
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np

from dishsim_frigidaire import loading
from dishsim_frigidaire.geometry import PARAMETERS, upper_tine_positions


class UpperLoadingTests(unittest.TestCase):
    def setUp(self):
        # Candidate support-height input; collision feasibility is checked later
        # by FCL against the actual tableware and rack assets.
        corners = np.array([[-.075, -.075, -.009], [.075, .075, .009]])
        self.world = SimpleNamespace(points={kind: corners for kind in loading.ORDER})

    def candidate_patterns(self):
        # Rotation conversion is independent of the slot coordinates under test.
        with patch.object(loading, "quaternion_xyzw", return_value=[0., 0., 0., 1.]):
            return loading.candidates(self.world)

    def test_twelve_saucer_slots_follow_actual_base_gaps(self):
        patterns = self.candidate_patterns()["saucer"]
        self.assertEqual(len(patterns), 24)  # two existing lean variants per gap
        self.assertEqual(len({c["slot"] for c in patterns}), 12)
        _, ys = upper_tine_positions()
        for index in range(12):
            pair = patterns[2*index:2*index+2]
            for entry, offset in zip(pair, [.010, .008]):
                self.assertEqual(entry["rack"], "UpperRack")
                self.assertEqual(entry["slot"], "saucer_%02d" % index)
                self.assertEqual(entry["position"][0], 0.)
                self.assertAlmostEqual(entry["position"][1]-offset,
                                       (ys[index]+ys[index+1])/2, places=10)

    def test_rear_margin_adjustment_only_moves_upper_saucer_slots(self):
        before = self.candidate_patterns()
        cfg = PARAMETERS["upper_rack"]
        with patch.dict(cfg, {"tine_rear_margin": cfg["tine_rear_margin"]+.005}):
            after = self.candidate_patterns()
        for kind in loading.ORDER:
            if kind != "saucer":
                self.assertEqual(before[kind], after[kind], kind)
                continue
            for old, new in zip(before[kind], after[kind]):
                self.assertAlmostEqual(new["position"][1]-old["position"][1], -.005)
                self.assertEqual(old["position"][0], new["position"][0])
                self.assertEqual(old["position"][2], new["position"][2])


if __name__ == "__main__":
    unittest.main()
