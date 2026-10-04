"""Analytic capsule edge cases and the measured lower-rack basket bay."""
import importlib.util
from pathlib import Path
import unittest

import numpy as np

from dishsim_frigidaire import geometry

_PATH = Path(__file__).resolve().parents[3] / "code/initialization/frigidaire/frigidaire_lower_rack_clearance.py"
_SPEC = importlib.util.spec_from_file_location("lower_clearance", _PATH)
clearance = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(clearance)


class SegmentDistanceTests(unittest.TestCase):
    def check_distance(self, first, second, expected):
        distance, a, b = clearance.segment_pairs(*first, [second[0]], [second[1]])
        self.assertAlmostEqual(distance[0], expected, places=12)
        self.assertAlmostEqual(np.linalg.norm(a[0]-b[0]), expected, places=12)

    def test_interior_crossing_and_skew(self):
        self.check_distance(([-1, 0, 0], [1, 0, 0]), ([0, -1, 0], [0, 1, 0]), 0.)
        self.check_distance(([-1, 0, 0], [1, 0, 0]), ([0, -1, 2], [0, 1, 2]), 2.)

    def test_parallel_overlap_and_disjoint_endpoints(self):
        self.check_distance(([0, 0, 0], [2, 0, 0]), ([1, 3, 0], [3, 3, 0]), 3.)
        self.check_distance(([0, 0, 0], [1, 0, 0]), ([2, 0, 0], [3, 0, 0]), 1.)

    def test_nearly_parallel_interior_crossing(self):
        self.check_distance(([0, 0, 0], [1, 0, 0]),
                            ([.25, -1e-8, 0], [1.25, 1e-8, 0]), 0.)

    def test_points_and_zero_length_segments(self):
        self.check_distance(([0, 0, 0], [0, 0, 0]), ([-1, 2, 0], [1, 2, 0]), 2.)
        self.check_distance(([-1, 2, 0], [1, 2, 0]), ([0, 0, 0], [0, 0, 0]), 2.)
        self.check_distance(([0, 0, 0], [0, 0, 0]), ([0, 3, 4], [0, 3, 4]), 5.)

    def test_signed_radius_overlap(self):
        first = [("rack", 0, np.array([0., 0., 0.]), np.array([1., 0., 0.]), .1)]
        second = [("basket", 0, np.array([0., .15, 0.]), np.array([1., .15, 0.]), .1)]
        self.assertAlmostEqual(clearance.minimum_clearance(first, second)["clearance_mm"], -50.)

    def test_segment_rectangle_crossing(self):
        cross = clearance._segment_crosses_rectangle
        self.assertTrue(cross(np.array([0., 0., 0.]), np.array([1., 0., 0.]), (.5, 2.), (-.1, .1)))
        self.assertFalse(cross(np.array([0., 0., 0.]), np.array([.4, 0., 0.]), (.5, 2.), (-.1, .1)))
        self.assertFalse(cross(np.array([.6, 0., 0.]), np.array([.7, 0., 0.]), (.5, 2.), (.1, .2)))
        self.assertTrue(cross(np.array([.6, .15, 0.]), np.array([.6, .15, 0.]), (.5, 2.), (.1, .2)))


class BasketFitTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.lower, cls.basket = geometry._lower_rack(), geometry._basket()
        cls.seat = (np.asarray(geometry.PARAMETERS["origins"]["SilverwareBasket"])
                    - geometry.PARAMETERS["origins"]["LowerRack"])

    def test_approved_position_meets_measured_bay_clearances(self):
        report = clearance.basket_clearance_report(self.lower, self.basket)
        self.assertTrue(report["passed"], report["checks"])
        self.assertAlmostEqual(report["basket_origin_in_rack_mm"][0], 184.0)
        clearances = report["minimum_capsule_clearances"]
        self.assertGreaterEqual(clearances["tines_and_base_rails"]["clearance_mm"], 5.)
        self.assertGreaterEqual(clearances["right_wall"]["clearance_mm"], 3.)
        self.assertTrue(report["checks"]["no_rails_under_basket"])
        self.assertEqual(report["rails_under_basket"], [])
        self.assertEqual(len(report["removed_tines"]), 8)

    def test_tine_group_is_restricted_to_rows_beside_the_basket(self):
        """The full front plate rows stay out of the tine audit; only rows 3-6 lie beside the basket."""
        report = clearance.basket_clearance_report(self.lower, self.basket)
        witness = report["minimum_capsule_clearances"]["tines_and_base_rails"]
        self.assertIn(int(witness["rack_wire"][len("TineBank")]), (2, 3, 4, 5))
        self.assertGreater(report["tine_x_envelope_gap_mm"], 5.)
        window = report["basket_footprint_mm"]["tine_group_y_window"]
        self.assertAlmostEqual(window[0], -70.)
        self.assertAlmostEqual(window[1], 262.)
        thresholds = report["group_thresholds_mm"]
        self.assertAlmostEqual(thresholds["floor_x"], 225.18)
        self.assertAlmostEqual(thresholds["long_turn_y"], 255.67)   # 561 mm rim (2026-09-28); 256.67 at 563

    def test_rail_reaching_under_the_footprint_fails_without_any_contact(self):
        footprint = geometry.lower_basket_footprint()
        x, y = np.mean(footprint["x"]), np.mean(footprint["y"])
        rail = ("TineBank9_Base", np.array([[x-.05, y, -.060], [x+.01, y, -.060]]), .0021)
        lower = {**self.lower, "wires": self.lower["wires"]+[rail]}
        report = clearance.basket_clearance_report(lower, self.basket)
        self.assertFalse(report["checks"]["no_rails_under_basket"])
        self.assertEqual(report["rails_under_basket"][0]["rack_wire"], "TineBank9_Base")

    def test_excessive_right_translation_hits_sloping_wall(self):
        """With the v3 taper the bottom rim starts 8 mm from the wall bend: a 15 mm shift breaks the 3 mm
        gate, 17 mm already goes negative (-0.2 mm) and the probe uses 18 mm (-1.1 mm)."""
        report = clearance.basket_clearance_report(self.lower, self.basket, self.seat+[.018, 0, 0])
        self.assertFalse(report["passed"])
        self.assertLess(report["minimum_capsule_clearances"]["right_wall"]["clearance_mm"], 0.)

    def test_fixture_sites_are_audited_against_the_bay_geometry(self):
        reports, _ = clearance.fixture_audit(self.lower)
        self.assertEqual(reports["plate"]["status"], "NO_SAMPLED_CENTERLINE_PENETRATION_FOUND",
                         reports["plate"]["collision_witness"])
        self.assertIsNone(reports["plate"]["collision_witness"])
        self.assertEqual(reports["bowl"]["status"], "CONFIRMED_INITIAL_COLLISION")
        witness = reports["bowl"]["collision_witness"]
        self.assertTrue(witness["rack_wire"].startswith("TineBank"))
        self.assertGreater(witness["centerline_interior_depth_mm"], 1.)


if __name__ == "__main__":
    unittest.main()
