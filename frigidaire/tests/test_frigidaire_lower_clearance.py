"""Analytic capsule edge cases and the measured lower-rack basket bay."""
import importlib.util
from pathlib import Path
import unittest

import numpy as np

from dishsim_frigidaire import geometry

_PATH = Path(__file__).resolve().parents[1] / "scripts/evaluation/frigidaire_lower_rack_clearance.py"
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


class BasketFitTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.lower, cls.basket = geometry._lower_rack(), geometry._basket()

    def test_approved_position_meets_measured_bay_clearances(self):
        report = clearance.basket_clearance_report(self.lower, self.basket)
        self.assertTrue(report["passed"], report["checks"])
        self.assertAlmostEqual(report["basket_origin_in_rack_mm"][0], 213.5)
        self.assertAlmostEqual(report["conservative_x_envelope_gap_mm"], 5.23, places=8)
        self.assertGreater(report["minimum_capsule_clearances"]["right_wall"]["clearance_mm"], 3.)

    def test_excessive_right_translation_hits_sloping_wall(self):
        report = clearance.basket_clearance_report(self.lower, self.basket, [.218, .120, .011])
        self.assertFalse(report["passed"])
        self.assertLess(report["minimum_capsule_clearances"]["right_wall"]["clearance_mm"], 0.)

    def test_original_fixture_sites_have_concrete_penetration_witnesses(self):
        reports, _ = clearance.fixture_audit(self.lower)
        for kind in ("plate", "bowl"):
            self.assertEqual(reports[kind]["status"], "CONFIRMED_INITIAL_COLLISION")
            witness = reports[kind]["collision_witness"]
            self.assertTrue(witness["rack_wire"].startswith("TineBank"))
            self.assertGreater(witness["centerline_interior_depth_mm"], 1.)


if __name__ == "__main__":
    unittest.main()
