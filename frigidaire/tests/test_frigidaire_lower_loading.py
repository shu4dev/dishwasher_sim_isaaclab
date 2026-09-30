"""Lower plate patterns follow the revised source grid without requiring USD/FCL."""
import unittest
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np

from dishsim_frigidaire import loading
from dishsim_frigidaire.geometry import (PARAMETERS, lower_basket_footprint, lower_plate_gaps,
                                         lower_tine_positions)


class LowerLoadingTests(unittest.TestCase):
    def patterns(self):
        corners = np.array([[-.130, -.130, -.010], [.130, .130, .010]])
        world = SimpleNamespace(points={kind: corners for kind in loading.ORDER})
        with patch.object(loading, "quaternion_xyzw", return_value=[0., 0., 0., 1.]):
            return loading.candidates(world)

    def test_plate_slots_use_present_column_gaps_and_outer_row_pairs(self):
        patterns = self.patterns()
        xs, ys = lower_tine_positions()
        mids = (xs[:-1]+xs[1:])/2
        # 12 columns at 31.8 mm give 11 front gaps, 1 of which (gap 10) is reserved for the bowls beside the
        # basket's left face (2 at the 36 mm pitch); the basket bay leaves the rear bank 9 gaps.
        bowl_zone_x = lower_basket_footprint()["x"][0]-.020
        front_gaps = [g for g in lower_plate_gaps("front") if mids[g] <= bowl_zone_x]
        rear_gaps = lower_plate_gaps("rear")
        self.assertEqual((len(front_gaps), len(rear_gaps)), (10, 9))
        inset = {"front": .025, "rear": -.015}
        for kind in ("dinner_plate", "salad_plate"):
            plates = patterns[kind]
            for bank, y, gaps in [("front", (ys[0]+ys[1])/2, front_gaps),
                                   ("rear", (ys[-2]+ys[-1])/2, rear_gaps)]:
                selected = [c for c in plates if c["slot"].startswith("lower_"+bank)]
                self.assertEqual(len(selected), len(gaps)*3)   # two leans + the rearward inset
                self.assertEqual({c["slot"] for c in selected}, {"lower_%s_%02d" % (bank, g) for g in gaps})
                for c in selected:
                    index = int(c["slot"].rsplit("_", 1)[-1])
                    offset = float(c["variant"].split("_offset")[1].split("_")[0])
                    self.assertAlmostEqual(c["position"][0]-offset, mids[index], places=10)
                    seat_y = y+inset[bank] if c["variant"].endswith("_inset") else y
                    self.assertAlmostEqual(c["position"][1], seat_y, places=10)
                    self.assertNotIn("release_hover_m", c)
                    self.assertNotIn("valley", c["variant"])
                    if bank == "front":
                        self.assertLessEqual(mids[index], bowl_zone_x)
                self.assertEqual(sum(c["variant"].endswith("_inset") for c in selected), len(gaps))

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
