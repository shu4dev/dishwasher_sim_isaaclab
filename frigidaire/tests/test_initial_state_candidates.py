"""Geometry, sampling and finite-catalog solver contracts without launching Kit."""
import json
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from dishsim_frigidaire.initial_state_candidates import (
    COMPONENT_NAMES, InitialCollisionChecker, build_compatibility, generate_catalog, greedy_proposal, milp_proposal,
)
from dishsim_frigidaire.loading import Part
from dishsim_frigidaire.random_poses import compose_pose, relative_pose, quaternion_matrix_xyzw


def pose(p=(0., 0., 0.), q=(0., 0., 0., 1.)):
    return {"position_m": list(p), "quaternion_xyzw": list(q)}


def source(tmp_path, count=2):
    trials = []
    for i in range(count):
        rack_q = [0., 0., np.sin(.2 + i / 10), np.cos(.2 + i / 10)]
        rack_p = [i * .02, i * .01, .3]
        p, q = compose_pose(rack_p, rack_q, [.1, -.2, .05], [.2, 0., 0., np.sqrt(.96)])
        trials.append({"trial_id": f"bowl_LowerRack_{i:05}", "kind": "bowl", "rack": "LowerRack",
                       "outcome": "accepted", "final_rack_pose": pose(rack_p, rack_q),
                       "final_pose": pose(p, q)})
    path = tmp_path / "accepted.json"
    path.write_text(json.dumps({"trials": trials}))
    frames = {name: pose([1., -2., 3.], [0., np.sin(.3), 0., np.cos(.3)]) for name in COMPONENT_NAMES}
    return path, frames


def test_each_source_uses_own_measured_rack_frame(tmp_path):
    path, frames = source(tmp_path)
    catalog = generate_catalog(path, frames, variants_per_template=0)
    candidates = catalog["candidates"]
    np.testing.assert_allclose(candidates[0]["pose_world"]["position_m"], candidates[1]["pose_world"]["position_m"])
    for candidate in candidates:
        local = candidate["rack_local_pose"]
        np.testing.assert_allclose(local["position_m"], [.1, -.2, .05], atol=1e-14)
        p, q = relative_pose(**dict(world_position=candidate["pose_world"]["position_m"],
                                   world_quaternion=candidate["pose_world"]["quaternion_xyzw"],
                                   frame_position=frames["LowerRack"]["position_m"],
                                   frame_quaternion=frames["LowerRack"]["quaternion_xyzw"]))
        np.testing.assert_allclose(p, local["position_m"], atol=1e-14)
        assert abs(np.dot(q, local["quaternion_xyzw"])) == pytest.approx(1.)


def test_perturbations_are_bounded_reproducible_and_volume_uniform(tmp_path):
    path, frames = source(tmp_path, 176)
    catalog = generate_catalog(path, frames, seed=42)
    assert catalog == generate_catalog(path, frames, seed=42)
    assert catalog["candidate_count"] == 1584
    assert len({c["candidate_id"] for c in catalog["candidates"]}) == 1584
    values = []
    directions = []
    angles = []
    for candidate in catalog["candidates"]:
        delta = candidate["perturbation"]
        distance = np.linalg.norm(delta["translation_m"])
        assert distance <= .010
        assert 0 <= delta["rotation_angle_deg"] <= 10
        original = candidate["source_rack_local_pose"]
        varied = candidate["rack_local_pose"]
        np.testing.assert_allclose(np.array(varied["position_m"]) - original["position_m"], delta["translation_m"], atol=1e-15)
        actual_angle = np.degrees(2 * np.arccos(np.clip(abs(np.dot(original["quaternion_xyzw"], varied["quaternion_xyzw"])), 0, 1)))
        assert actual_angle == pytest.approx(delta["rotation_angle_deg"], abs=2e-6)
        np.testing.assert_allclose(quaternion_matrix_xyzw(varied["quaternion_xyzw"]),
                                   quaternion_matrix_xyzw(delta["rotation_quaternion_xyzw"])
                                   @ quaternion_matrix_xyzw(original["quaternion_xyzw"]), atol=1e-14)
        if candidate["variant_index"]:
            values.append((distance / .010) ** 3)
            directions.append(np.array(delta["translation_m"]) / distance)
            angles.append(delta["rotation_angle_deg"])
        else:
            assert distance == 0 and delta["rotation_angle_deg"] == 0
    # Uniform volume requires r^3 uniform, not radius uniform. This tolerance
    # comfortably covers this deterministic 1,408-sample draw and catches r=U.
    assert .47 < np.mean(values) < .53
    assert np.max(np.abs(np.mean(directions, axis=0))) < .06
    assert 4.7 < np.mean(angles) < 5.3


def test_rejected_sources_and_duplicate_ids_fail_closed(tmp_path):
    path, frames = source(tmp_path)
    document = json.loads(path.read_text())
    document["trials"][0]["outcome"] = "initial_collision"
    path.write_text(json.dumps(document))
    with pytest.raises(ValueError, match="Only accepted"):
        generate_catalog(path, frames)
    document["trials"][0]["outcome"] = "accepted"
    document["trials"][1]["trial_id"] = document["trials"][0]["trial_id"]
    path.write_text(json.dumps(document))
    with pytest.raises(ValueError, match="unique"):
        generate_catalog(path, frames)


def graph(n, conflicts=()):
    return {"complete": True, "allowed_indices": list(range(n)), "conflict_pairs": list(conflicts)}


def test_graph_solver_matches_known_independent_set_and_bound():
    # A five-cycle has independence number two, plus an isolated vertex = 3.
    problem = graph(6, [(0, 1), (1, 2), (2, 3), (3, 4), (4, 0)])
    proposal = milp_proposal(problem, seed=1, time_limit_s=2)
    assert proposal["count"] == 3
    assert proposal["geometric_cardinality_upper_bound"] == 3
    assert proposal["status"] == 0
    greedy = greedy_proposal(problem, seed=5)
    assert greedy == greedy_proposal(problem, seed=5)
    assert greedy["count"] == 3


def test_exact_nogood_keeps_superset_and_preserves_full_catalog_bound():
    problem = graph(3)
    proposal = milp_proposal(problem, excluded_sets=[{0, 1}], target_count=3, time_limit_s=2)
    assert proposal["selected_indices"] == [0, 1, 2]
    proposal = milp_proposal(problem, excluded_sets=[{0, 1}], target_count=2, time_limit_s=2)
    assert proposal["count"] == 2
    assert proposal["selected_indices"] != [0, 1]
    proposal = milp_proposal(problem, excluded_sets=[{0, 1, 2}], time_limit_s=2)
    assert proposal["count"] == 2
    assert proposal["geometric_cardinality_upper_bound"] == 3


def test_greedy_duplicate_prevention_and_incomplete_graph_rejection():
    problem = graph(3)
    proposal = greedy_proposal(problem, seed=0, target_count=2, excluded_sets=[{0, 1}])
    assert proposal["count"] == 2 and proposal["selected_indices"] != [0, 1]
    assert greedy_proposal(problem, seed=0, excluded_sets=[{0, 1, 2}])["selected_indices"] is None
    problem["complete"] = False
    with pytest.raises(ValueError, match="incomplete"):
        greedy_proposal(problem, seed=0)
    with pytest.raises(ValueError, match="incomplete"):
        milp_proposal(problem)


@pytest.fixture
def checker():
    fcl = pytest.importorskip("fcl")
    result = InitialCollisionChecker.__new__(InitialCollisionChecker)
    result.fcl = fcl
    result.penetration_limit_m = .001
    result.max_contacts_per_pair = 256
    result.request = fcl.CollisionRequest(enable_contact=True, num_max_contacts=256)
    result.parts = {name: [Part(fcl.Box(.1, .1, .1), np.eye(3), np.zeros(3), "test_box")]
                    for name in (*COMPONENT_NAMES, "bowl", "mug")}
    result.bounds = {name: (np.full(3, -.05), np.full(3, .05)) for name in result.parts}
    result.components = None
    return result


@pytest.mark.parametrize("distance, valid", [(.1001, True), (.1000, True), (.0995, True), (.0990, True), (.0985, False), (.0, False)])
def test_fcl_penetration_allowance_uses_contact_depth(checker, distance, valid):
    first = checker._body("bowl", pose(), "bowl_a")
    second = checker._body("bowl", pose([distance, 0, 0]), "bowl_b")
    result = checker.pair(first, second)
    assert result["valid"] == valid
    assert result["max_penetration_m"] == pytest.approx(max(0., .1-distance), abs=1e-10)
    json.dumps(result)


def test_multi_object_collision_and_component_frame_updates(checker):
    frames = {name: pose([10. + i, 0, 0]) for i, name in enumerate(COMPONENT_NAMES)}
    objects = [{"object_id": "a", "candidate_id": "same_template", "kind": "bowl", "pose_world": pose()},
               {"object_id": "b", "candidate_id": "same_template", "kind": "mug", "pose_world": pose([.09, 0, 0])}]
    result = checker.check_arrangement(objects, frames)
    assert not result["valid"] and result["pairs"][-1]["bodies"] == ["b", "a"]
    objects[1]["pose_world"] = pose([.15, 0, 0])
    assert checker.check_arrangement(objects, frames)["valid"]
    frames["LowerRack"] = pose()
    assert not checker.check_arrangement(objects, frames)["valid"]
    objects[1]["object_id"] = "a"
    with pytest.raises(ValueError, match="unique"):
        checker.check_arrangement(objects, frames)


def test_expired_geometry_budget_returns_partial_not_permissive_graph(monkeypatch):
    import time
    import dishsim_frigidaire.initial_state_candidates as module
    class FakeChecker:
        asset_sha256 = {}
        def __init__(self, *args):
            pass
        def update_components(self, frames):
            pass
        def candidate_body(self, entry):
            raise AssertionError("An expired search must not query another candidate")
    monkeypatch.setattr(module, "InitialCollisionChecker", FakeChecker)
    catalog = {"baseline_components": {}, "candidates": [{"candidate_id": "one"}],
               "source_accepted_sha256": "test"}
    result = build_compatibility(catalog, "/unused", deadline=time.monotonic() - 1)
    assert result["status"] == "time_budget_exhausted"
    assert result["complete"] is False
    assert result["initial_filter"] == []
    assert result["elapsed_s"] >= 0
    with pytest.raises(ValueError, match="incomplete"):
        greedy_proposal(result, 0)


def test_milp_cardinality_dominates_preferred_overlap():
    # Keeping preferred center 0 yields only one dish; rejecting it permits 3.
    problem = graph(4, [(0, 1), (0, 2), (0, 3)])
    proposal = milp_proposal(problem, preferred_indices=[0], seed=3, time_limit_s=2)
    assert proposal['selected_indices'] == [1, 2, 3]
    assert proposal['selected_preferred_count'] == 0
    assert proposal['geometric_cardinality_upper_bound'] == 3


def test_milp_prefers_known_objects_at_equal_cardinality_and_keeps_exact_nogoods():
    problem = graph(4, [(0, 1), (2, 3)])
    for seed in range(3):
        proposal = milp_proposal(problem, preferred_indices=[0, 2], seed=seed, time_limit_s=2)
        assert proposal['selected_indices'] == [0, 2]
        assert proposal['selected_preferred_count'] == proposal['preferred_count'] == 2
    proposal = milp_proposal(problem, preferred_indices=[0, 2], excluded_sets=[[0, 2]], time_limit_s=2)
    assert proposal['count'] == 2 and proposal['selected_preferred_count'] == 1
    assert proposal['geometric_cardinality_upper_bound'] == 2
