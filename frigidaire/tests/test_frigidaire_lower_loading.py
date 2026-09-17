"""Lower plate patterns follow the revised source grid without requiring USD/FCL."""
import unittest
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np

from dishsim_frigidaire import loading
from dishsim_frigidaire.geometry import PARAMETERS, lower_tine_positions


class LowerLoadingTests(unittest.TestCase):
    def patterns(self):
        corners = np.array([[-.130, -.130, -.010], [.130, .130, .010]])
        world = SimpleNamespace(points={kind: corners for kind in loading.ORDER})
        with patch.object(loading, "quaternion_xyzw", return_value=[0., 0., 0., 1.]):
            return loading.candidates(world)

    def test_plate_slots_use_eleven_column_gaps_and_outer_row_pairs(self):
        patterns = self.patterns()
        xs, ys = lower_tine_positions()
        mids = (xs[:-1]+xs[1:])/2
        for kind in ("dinner_plate", "salad_plate"):
            plates = patterns[kind]
            for bank, y, count in [("front", (ys[0]+ys[1])/2, 9),
                                    ("rear", (ys[-2]+ys[-1])/2, 11)]:
                selected = [c for c in plates if c["slot"].startswith("lower_"+bank)]
                self.assertEqual(len(selected), count*2)
                self.assertEqual(len({c["slot"] for c in selected}), count)
                for c in selected:
                    index = int(c["slot"].rsplit("_", 1)[-1])
                    offset = float(c["variant"].split("_offset")[1])
                    self.assertAlmostEqual(c["position"][0]-offset, mids[index], places=10)
                    self.assertAlmostEqual(c["position"][1], y, places=10)
                    self.assertNotIn("release_hover_m", c)
                    self.assertNotIn("valley", c["variant"])
                    if bank == "front":
                        self.assertLessEqual(mids[index], .105)

    def test_lower_margin_change_preserves_upper_bowl_and_utensil_patterns(self):
        before = self.patterns()
        with patch.dict(PARAMETERS["lower_rack"]["tine_margins"],
                        {"front": .109, "right": .130}):
            after = self.patterns()
        for kind in loading.ORDER:
            if kind in ("dinner_plate", "salad_plate"):
                self.assertNotEqual(before[kind], after[kind], kind)
            else:
                self.assertEqual(before[kind], after[kind], kind)


if __name__ == "__main__":
    unittest.main()
