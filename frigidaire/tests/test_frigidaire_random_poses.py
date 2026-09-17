"""Host checks for unbiased sampling and measured acceptance rules (no Kit)."""
import json
import math
import unittest
from unittest.mock import patch

import numpy as np

from dishsim_frigidaire.load_validation import pose_motion_metrics
from dishsim_frigidaire.random_poses import (
    CELLS, LIMITS, RACKS, cell_generators, compose_pose, motion_is_settled,
    quaternion_matrix_xyzw, relative_pose, round_robin_cells, sample_pose,
    source_geometry_domains, transform_vertices, uniform_quaternion_xyzw,
    vertices_contained,
)


def authored_visuals(kind):
    from dishsim_frigidaire.tableware import tableware_geometry
    # The actual visual generation is NumPy-only. Hull generation produces
    # independent collider meshes, unused by these visual containment tests.
    # Skip that operation so the host suite does not require scipy/PhysX/USD.
    with patch("dishsim_frigidaire.tableware._hull",
               side_effect=lambda points: (np.asarray(points), np.empty((0, 3), dtype=int))):
        return tableware_geometry(kind)["visuals"]


class RandomPoseTests(unittest.TestCase):
    bounds = {"lower_m": [-.2, -.3, 0.], "upper_m": [.2, .3, .4]}
    points = np.array([[-.05, -.05, 0.], [.05, .05, .08], [.09, 0., .04]])

    def test_each_cell_is_reproducible_when_other_rack_is_skipped(self):
        first, second = cell_generators(17), cell_generators(17)
        expected = {(kind, rack, index): sample_pose(first[(kind, rack)], self.points, self.bounds)
                    for kind, rack, index in round_robin_cells(3)}
        for kind, rack, index in round_robin_cells(3, skipped_racks=("LowerRack",)):
            self.assertEqual(sample_pose(second[(kind, rack)], self.points, self.bounds),
                             expected[(kind, rack, index)])
        self.assertNotEqual(expected[("mug", "UpperRack", 0)], expected[("bowl", "UpperRack", 0)])

    def test_streams_are_independent_of_cell_processing_order(self):
        first, second = cell_generators(91), cell_generators(91)
        expected = {cell: sample_pose(first[cell], self.points, self.bounds) for cell in CELLS}
        actual = {cell: sample_pose(second[cell], self.points, self.bounds) for cell in reversed(CELLS)}
        self.assertEqual(expected, actual)
        self.assertNotEqual(expected[CELLS[0]], sample_pose(cell_generators(92)[CELLS[0]], self.points, self.bounds))

    def test_round_robin_exposes_all_six_unrestricted_combinations(self):
        schedule = list(round_robin_cells(2))
        self.assertEqual(schedule[:6], [(kind, rack, 0) for kind, rack in CELLS])
        self.assertEqual(schedule[6:], [(kind, rack, 1) for kind, rack in CELLS])
        self.assertIn(("dinner_plate", "UpperRack", 0), schedule)
        self.assertIn(("mug", "LowerRack", 0), schedule)
        self.assertEqual(list(round_robin_cells(2, RACKS)), [])
        for count in (-1, 1.5, True):
            with self.assertRaises(ValueError):
                list(round_robin_cells(count))

    def test_uniform_rotations_cover_so3_without_upright_bias(self):
        rng = np.random.default_rng(182)
        quaternions = np.asarray([uniform_quaternion_xyzw(rng) for _ in range(12000)])
        normals = np.asarray([quaternion_matrix_xyzw(q)[:, 2] for q in quaternions])
        np.testing.assert_allclose(np.linalg.norm(quaternions, axis=1), 1., atol=1e-14)
        np.testing.assert_allclose(normals.mean(axis=0), np.zeros(3), atol=.015)
        np.testing.assert_allclose(normals.T @ normals/len(normals), np.eye(3)/3, atol=.015)
        bins = np.histogram(normals[:, 2], bins=np.linspace(-1, 1, 11))[0]
        self.assertLess(np.max(np.abs(bins-len(normals)/10)), 150)
        self.assertGreater(np.count_nonzero(quaternions[:, 3] < 0), 5500)

    def test_sample_centers_are_uniform_and_not_shrunk_to_fit_dish(self):
        rng = np.random.default_rng(19)
        # Oversized geometry still receives a proposal: rejection is a trial outcome.
        oversized = self.points * 20
        samples = [sample_pose(rng, oversized, self.bounds) for _ in range(3000)]
        centers = np.asarray([sample["bbox_center_m"] for sample in samples])
        lower, upper = np.asarray(self.bounds["lower_m"]), np.asarray(self.bounds["upper_m"])
        normalized = (centers-lower)/(upper-lower)
        self.assertTrue(((normalized >= 0) & (normalized <= 1)).all())
        self.assertTrue((normalized.min(axis=0) < .005).all())
        self.assertTrue((normalized.max(axis=0) > .995).all())
        np.testing.assert_allclose(normalized.mean(axis=0), .5, atol=.02)

    def test_bowl_base_and_mug_handle_offsets_preserve_sampled_center(self):
        # Use actual authored visual geometry so both shifted actor origins and
        # the separate mug handle mesh participate in the calculation.
        for kind in ("bowl", "mug"):
            mesh = np.concatenate([part[0] for part in authored_visuals(kind)])
            for sample_index in range(12):
                pose = sample_pose(np.random.default_rng(sample_index), mesh, self.bounds)
                placed = transform_vertices(mesh, pose["position_m"], pose["quaternion_xyzw"])
                center = (placed.min(axis=0)+placed.max(axis=0))/2
                np.testing.assert_allclose(center, pose["bbox_center_m"], atol=1e-14)
                self.assertGreater(np.linalg.norm(np.asarray(pose["position_m"])-center), .001)

    def test_full_mesh_containment_includes_mug_handle(self):
        visuals = authored_visuals("mug")
        lower, upper = [-.05, -.05, -.06], [.05, .05, .06]
        self.assertTrue(vertices_contained(visuals[0][0], lower, upper))
        self.assertFalse(vertices_contained(np.concatenate([part[0] for part in visuals]), lower, upper))
        self.assertTrue(vertices_contained([[.0509, 0, 0]], lower, upper))
        self.assertFalse(vertices_contained([[.0511, 0, 0]], lower, upper))

    def test_pose_composition_and_inverse_use_measured_rotated_frames(self):
        frame_position = [.3, -.7, .2]
        frame_quaternion = [0, 0, math.sqrt(.5), math.sqrt(.5)]
        local_position = [.1, .2, .3]
        local_quaternion = [.5, .5, .5, .5]
        position, quaternion = compose_pose(frame_position, frame_quaternion, local_position, local_quaternion)
        np.testing.assert_allclose(position, [.1, -.6, .5], atol=1e-14)
        recovered_position, recovered_quaternion = relative_pose(position, quaternion, frame_position, frame_quaternion)
        np.testing.assert_allclose(recovered_position, local_position, atol=1e-14)
        np.testing.assert_allclose(quaternion_matrix_xyzw(recovered_quaternion), quaternion_matrix_xyzw(local_quaternion))
        expected = transform_vertices(transform_vertices(self.points, local_position, local_quaternion),
                                      frame_position, frame_quaternion)
        np.testing.assert_allclose(transform_vertices(self.points, position, quaternion), expected, atol=1e-14)

    def test_quaternion_sign_does_not_count_as_motion(self):
        count = 121
        quaternions = np.tile([1., 0., 0., 0.], (count, 1))  # measured WXYZ interface
        quaternions[::2] *= -1
        metrics = pose_motion_metrics(np.zeros((count, 3)), quaternions, self.points, LIMITS["physics_dt_s"])
        self.assertTrue(motion_is_settled(metrics))
        self.assertEqual(metrics["quaternion_span_deg"], 0)
        for key in ("root_position_span_m", "quaternion_span_deg", "peak_mesh_point_speed_m_s"):
            self.assertFalse(motion_is_settled(dict(metrics, **{key: LIMITS[key]})))
            self.assertFalse(motion_is_settled(dict(metrics, **{key: float("nan")})))
        self.assertFalse(motion_is_settled(dict(metrics, sample_duration_s=.99)))
        self.assertFalse(motion_is_settled({}))

    def test_source_domains_follow_generated_wire_floors_and_tub_surfaces(self):
        domains = source_geometry_domains()
        json.dumps(domains, allow_nan=False)
        self.assertEqual(set(domains["sampling_bounds_by_rack"]), set(RACKS))
        lower, upper = (domains["sampling_bounds_by_rack"][rack] for rack in RACKS)
        np.testing.assert_allclose(lower["lower_m"][:2], [-.54864/2, -.58166/2])
        np.testing.assert_allclose(upper["upper_m"][:2], [.508/2, .54864/2])
        self.assertAlmostEqual(lower["lower_m"][2], .002)
        # Rounded troughs are shallower than the unfilleted -18 mm control points.
        self.assertGreater(upper["lower_m"][2], -.018)
        self.assertLess(upper["lower_m"][2], -.01)
        self.assertAlmostEqual(upper["upper_m"][2], .817-.590)
        np.testing.assert_allclose(domains["interior_bounds"]["lower_m"], [-.277, -.2855, .152])
        np.testing.assert_allclose(domains["interior_bounds"]["upper_m"], [.277, .3015, .817])
        self.assertAlmostEqual(lower["upper_m"][2]+.215,
                               domains["derivation"]["lower_overhead_cabinet_z_m"])
        lower["lower_m"][0] = -999
        self.assertNotEqual(source_geometry_domains()["sampling_bounds_by_rack"]["LowerRack"]["lower_m"][0], -999)

    def test_invalid_geometry_and_seed_fail_before_simulation(self):
        for seed in (-1, 1.2, True):
            with self.assertRaises(ValueError):
                cell_generators(seed)
        for vertices in ([], [[float("nan"), 0, 0]], [[0, 1]]):
            with self.assertRaises(ValueError):
                sample_pose(np.random.default_rng(1), vertices, self.bounds)
        with self.assertRaises(ValueError):
            quaternion_matrix_xyzw([0, 0, 0, 0])
        with self.assertRaises(ValueError):
            vertices_contained(self.points, [0, 0, 0], [0, 1, 1])


if __name__ == "__main__":
    unittest.main()
